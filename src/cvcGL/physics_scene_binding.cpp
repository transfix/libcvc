/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <array>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/physics/world.h>
#include <map>
#include <mutex>
#include <unordered_map>

namespace cvc {
namespace gl {

using physics::body_id;
using physics::pose_tracker;

struct physics_scene_binding::impl {
  explicit impl(physics::world &w) : tracker(w) {}

  struct node_binding {
    std::weak_ptr<GraphicsNode> node;
    std::array<double, 16> offset{};
    bool has_offset = false;
  };
  struct group {
    std::vector<body_id> ids;
    std::shared_ptr<pose_sink> sink;
  };

  pose_tracker tracker;

  mutable std::mutex m;                        // everything below but the present_* scratch
  std::map<std::uint32_t, node_binding> nodes; // by body id; ordered, so poses go out in id order
  std::map<group_id, group> groups;
  std::unordered_map<std::uint32_t, int> uses; // per body: its node binding + its group memberships
  group_id next_group = 1;
  std::size_t last_posed = 0;
  std::weak_ptr<const GraphicsNode> frame; // the physics frame's node (none: the root)
  bool has_frame = false;

  std::mutex present_m; // one present() at a time; guards the scratch
  std::vector<pose_tracker::sample> samples;
  std::unordered_map<std::uint32_t, std::size_t> where;
  std::vector<pose_tracker::sample> group_poses;

  // A body's first user tracks it; the last one to leave stops tracking it.
  // Under m.
  void use(body_id id) {
    auto it = uses.find(id.value);
    if (it != uses.end()) {
      ++it->second;
      return;
    }
    tracker.track(id); // throws for a body the world does not hold
    uses.emplace(id.value, 1);
  }
  void release(body_id id) {
    auto it = uses.find(id.value);
    if (it == uses.end())
      return;
    if (--it->second == 0) {
      uses.erase(it);
      tracker.untrack(id);
    }
  }
};

namespace {

void multiply(const double a[16], const double b[16], double out[16]) {
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[r * 4 + k] * b[k * 4 + c];
      out[r * 4 + c] = s;
    }
}

} // namespace

physics_scene_binding::physics_scene_binding(physics::world &w)
    : _impl(std::make_unique<impl>(w)) {}
physics_scene_binding::~physics_scene_binding() = default;

void physics_scene_binding::bind(body_id id, const std::shared_ptr<GraphicsNode> &node,
                                 const double body_from_visual[16]) {
  impl::node_binding b;
  b.node = node;
  if (body_from_visual) {
    std::copy(body_from_visual, body_from_visual + 16, b.offset.begin());
    b.has_offset = true;
  }
  std::lock_guard<std::mutex> lock(_impl->m);
  auto it = _impl->nodes.find(id.value);
  if (it != _impl->nodes.end()) {
    it->second = b; // a new node or offset for a bound body
    return;
  }
  _impl->use(id);
  _impl->nodes.emplace(id.value, b);
}

void physics_scene_binding::unbind(body_id id) {
  std::lock_guard<std::mutex> lock(_impl->m);
  if (_impl->nodes.erase(id.value) != 0)
    _impl->release(id);
}

bool physics_scene_binding::bound(body_id id) const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->nodes.count(id.value) != 0;
}

physics_scene_binding::group_id physics_scene_binding::bind_group(const std::vector<body_id> &ids,
                                                                  pose_sink sink) {
  std::lock_guard<std::mutex> lock(_impl->m);
  std::size_t used = 0;
  try {
    for (; used < ids.size(); ++used)
      _impl->use(ids[used]);
  } catch (...) {
    for (std::size_t i = 0; i < used; ++i)
      _impl->release(ids[i]);
    throw;
  }
  const group_id g = _impl->next_group++;
  _impl->groups.emplace(g, impl::group{ids, std::make_shared<pose_sink>(std::move(sink))});
  return g;
}

void physics_scene_binding::unbind_group(group_id g) {
  std::lock_guard<std::mutex> lock(_impl->m);
  auto it = _impl->groups.find(g);
  if (it == _impl->groups.end())
    return;
  for (const body_id id : it->second.ids)
    _impl->release(id);
  _impl->groups.erase(it);
}

void physics_scene_binding::set_frame(const std::shared_ptr<const GraphicsNode> &frame) {
  std::lock_guard<std::mutex> lock(_impl->m);
  _impl->frame = frame;
  _impl->has_frame = static_cast<bool>(frame);
}

physics::pose_tracker &physics_scene_binding::tracker() { return _impl->tracker; }

std::uint64_t physics_scene_binding::follow(const cvc::world_clock &clock) {
  return _impl->tracker.follow(clock);
}

void physics_scene_binding::present(const cvc::world_clock &clock, double clock_alpha) {
  present(_impl->tracker.alpha(clock, clock_alpha));
}

void physics_scene_binding::present(double alpha) {
  std::lock_guard<std::mutex> plock(_impl->present_m);
  _impl->tracker.sample_all(alpha, _impl->samples);
  _impl->where.clear();
  for (std::size_t i = 0; i < _impl->samples.size(); ++i)
    _impl->where.emplace(_impl->samples[i].id.value, i);

  // Copy what to pose, then pose without the lock: a node's pose can run state
  // listeners on this thread, and one of them may bind or unbind.
  std::vector<std::pair<std::uint32_t, impl::node_binding>> nodes;
  std::vector<impl::group> groups;
  std::shared_ptr<const GraphicsNode> frame;
  bool frame_gone = false;
  {
    std::lock_guard<std::mutex> lock(_impl->m);
    if (_impl->has_frame) {
      frame = _impl->frame.lock();
      frame_gone = !frame;
    }
    nodes.assign(_impl->nodes.begin(), _impl->nodes.end());
    groups.reserve(_impl->groups.size());
    for (const auto &g : _impl->groups)
      groups.push_back(g.second);
  }

  std::vector<std::uint32_t> expired, vanished;
  std::size_t posed = 0;
  if (frame_gone)
    nodes.clear(); // no world to draw the bodies in; groups still get their poses
  for (const auto &[id, b] : nodes) {
    const std::shared_ptr<GraphicsNode> node = b.node.lock();
    if (!node) {
      expired.push_back(id);
      continue;
    }
    const auto it = _impl->where.find(id);
    if (it == _impl->where.end()) {
      if (!_impl->tracker.tracking(body_id{id}))
        vanished.push_back(id); // removed from the world: the node keeps its last pose
      continue;                 // otherwise not captured yet
    }
    double m[16];
    physics::pose_matrix(_impl->samples[it->second].pose, m);
    if (b.has_offset) {
      double mo[16];
      multiply(m, b.offset.data(), mo);
      node->setWorldPoseMatrix(mo, frame);
    } else {
      node->setWorldPoseMatrix(m, frame);
    }
    ++posed;
  }

  for (const impl::group &g : groups) {
    _impl->group_poses.clear();
    for (const body_id id : g.ids) {
      const auto it = _impl->where.find(id.value);
      if (it != _impl->where.end())
        _impl->group_poses.push_back(_impl->samples[it->second]);
    }
    if (*g.sink)
      (*g.sink)(_impl->group_poses);
  }

  std::lock_guard<std::mutex> lock(_impl->m);
  for (const std::uint32_t id : expired) {
    auto it = _impl->nodes.find(id);
    if (it != _impl->nodes.end() && it->second.node.expired()) {
      _impl->nodes.erase(it);
      _impl->release(body_id{id});
    }
  }
  for (const std::uint32_t id : vanished) {
    if (_impl->nodes.erase(id) != 0)
      _impl->release(body_id{id});
  }
  _impl->last_posed = posed;
}

std::size_t physics_scene_binding::node_count() const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->nodes.size();
}

std::size_t physics_scene_binding::group_count() const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->groups.size();
}

std::size_t physics_scene_binding::last_posed() const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->last_posed;
}

} // namespace gl
} // namespace cvc

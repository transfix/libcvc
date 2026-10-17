/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/core/clock_follower.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/pose_tracker.h>
#include <cvc/physics/world.h>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace cvc {
namespace physics {

pose interpolate(const pose &a, const pose &b, double alpha) {
  if (!(alpha > 0.0))
    return a; // also a NaN alpha: draw the older state rather than garbage
  if (alpha >= 1.0)
    return b;
  pose out;
  out.position = a.position + (b.position - a.position) * alpha;
  const quat &p = a.rotation;
  quat q = b.rotation;
  if (p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z < 0.0)
    q = {-q.w, -q.x, -q.y, -q.z}; // the same rotation, on the near side of p
  const double s = 1.0 - alpha;
  quat r{s * p.w + alpha * q.w, s * p.x + alpha * q.x, s * p.y + alpha * q.y,
         s * p.z + alpha * q.z};
  const double n = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
  if (n > 0.0)
    r = {r.w / n, r.x / n, r.y / n, r.z / n};
  else
    r = p; // only for non-unit input: unit quaternions on the same side never cancel
  out.rotation = r;
  return out;
}

void pose_matrix(const pose &p, double m[16]) {
  quat q = p.rotation;
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n > 0.0)
    q = {q.w / n, q.x / n, q.y / n, q.z / n};
  else
    q = {};
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  m[0] = 1.0 - 2.0 * (yy + zz);
  m[1] = 2.0 * (xy - wz);
  m[2] = 2.0 * (xz + wy);
  m[3] = p.position.x;
  m[4] = 2.0 * (xy + wz);
  m[5] = 1.0 - 2.0 * (xx + zz);
  m[6] = 2.0 * (yz - wx);
  m[7] = p.position.y;
  m[8] = 2.0 * (xz - wy);
  m[9] = 2.0 * (yz + wx);
  m[10] = 1.0 - 2.0 * (xx + yy);
  m[11] = p.position.z;
  m[12] = 0.0;
  m[13] = 0.0;
  m[14] = 0.0;
  m[15] = 1.0;
}

struct pose_tracker::impl {
  explicit impl(world &w) : w(w), fixed_dt(w.config().fixed_dt) {}

  struct entry {
    body_id id;
    bool has_newest = false;
    bool has_previous = false;
    bool snap = false; // the next capture starts the body afresh
    std::uint64_t newest_tick = 0;
    std::uint64_t previous_tick = 0;
    pose newest;
    pose previous;
  };

  world &w;
  const double fixed_dt; // a build-time world setting: it never changes

  mutable std::mutex m;         // entries, tick, generation
  std::vector<entry> entries;   // in tracking order
  std::uint64_t tick = 0;       // the newest capture's world tick
  std::uint64_t generation = 0; // bumped whenever entries is reordered or shrunk

  std::mutex follow_m; // one follower at a time
  std::optional<clock_follower> follower;

  std::vector<entry>::iterator find(body_id id) {
    return std::find_if(entries.begin(), entries.end(), [&](const entry &e) { return e.id == id; });
  }
  std::vector<entry>::const_iterator find(body_id id) const {
    return std::find_if(entries.begin(), entries.end(), [&](const entry &e) { return e.id == id; });
  }

  static pose at(const entry &e, double alpha) {
    if (e.has_previous && e.previous_tick + 1 == e.newest_tick)
      return interpolate(e.previous, e.newest, alpha);
    return e.newest;
  }

  void capture() {
    std::vector<body_id> ids;
    std::uint64_t gen = 0;
    {
      std::lock_guard<std::mutex> lock(m);
      ids.reserve(entries.size());
      for (const entry &e : entries)
        ids.push_back(e.id);
      gen = generation;
    }
    // Read the world without holding the tracker's lock, so samplers never wait
    // on a step another thread is running.
    const state_batch batch = w.states(ids);

    std::lock_guard<std::mutex> lock(m);
    // Bodies tracked or untracked since the copy: match by id. The common case
    // (nothing changed) matches by position.
    std::unordered_map<std::uint32_t, std::size_t> where;
    const bool moved = gen != generation;
    if (moved)
      for (std::size_t i = 0; i < entries.size(); ++i)
        where.emplace(entries[i].id.value, i);
    std::vector<body_id> gone;
    for (std::size_t k = 0; k < ids.size(); ++k) {
      entry *e = nullptr;
      if (!moved) {
        e = &entries[k];
      } else {
        const auto it = where.find(ids[k].value);
        if (it == where.end())
          continue; // untracked meanwhile
        e = &entries[it->second];
      }
      const std::optional<body_state> &s = batch.states[k];
      if (!s) {
        gone.push_back(ids[k]); // removed from the world
        continue;
      }
      if (e->has_newest && e->newest_tick == batch.tick)
        continue; // already captured at this tick
      if (e->snap) {
        e->has_previous = false;
        e->snap = false;
      } else if (e->has_newest && batch.tick > e->newest_tick) {
        e->previous = e->newest;
        e->previous_tick = e->newest_tick;
        e->has_previous = true;
      } else {
        e->has_previous = false; // the world went back (a restore): start again
      }
      e->newest = {s->position, s->rotation};
      e->newest_tick = batch.tick;
      e->has_newest = true;
    }
    for (const body_id id : gone) {
      const auto it = find(id);
      if (it != entries.end()) {
        entries.erase(it);
        ++generation;
      }
    }
    tick = batch.tick;
  }

  clock_follower &follower_for(const world_clock &clock) {
    if (!follower)
      follower = clock_follower::for_step(clock, fixed_dt);
    return *follower;
  }
};

pose_tracker::pose_tracker(world &w) : _impl(std::make_unique<impl>(w)) {}
pose_tracker::~pose_tracker() = default;

void pose_tracker::track(body_id id) {
  if (!_impl->w.contains(id))
    throw cvc::physics_error("pose_tracker::track: the world holds no body with this id");
  std::lock_guard<std::mutex> lock(_impl->m);
  if (_impl->find(id) != _impl->entries.end())
    return;
  impl::entry e;
  e.id = id;
  _impl->entries.push_back(e);
}

void pose_tracker::untrack(body_id id) {
  std::lock_guard<std::mutex> lock(_impl->m);
  const auto it = _impl->find(id);
  if (it == _impl->entries.end())
    return;
  _impl->entries.erase(it);
  ++_impl->generation;
}

bool pose_tracker::tracking(body_id id) const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->find(id) != _impl->entries.end();
}

std::size_t pose_tracker::size() const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->entries.size();
}

void pose_tracker::capture() { _impl->capture(); }

std::uint64_t pose_tracker::follow(const cvc::world_clock &clock) {
  std::lock_guard<std::mutex> lock(_impl->follow_m);
  if (!_impl->follower) {
    _impl->follower_for(clock); // anchors at the clock's tick
    _impl->capture();
    return 0;
  }
  const std::uint64_t n = _impl->follower->take(clock);
  if (n == 0)
    return 0;
  // The view needs the last two ticks, not the first and the last of a batch.
  if (n > 1) {
    _impl->w.step(n - 1);
    _impl->capture();
  }
  _impl->w.step();
  _impl->capture();
  return n;
}

double pose_tracker::alpha(const cvc::world_clock &clock, double clock_alpha) const {
  // Same period and phase as follow()'s cursor; alpha() reads neither cursor.
  const clock_follower f(ticks_per_step(_impl->fixed_dt, clock.fixed_dt()));
  return f.alpha(clock.tick(), clock_alpha);
}

void pose_tracker::snap(body_id id) {
  std::lock_guard<std::mutex> lock(_impl->m);
  const auto it = _impl->find(id);
  if (it != _impl->entries.end()) {
    it->has_previous = false; // drawn at its newest from now on
    it->snap = true;          // and the next capture does not carry the old state over
  }
}

std::vector<pose_tracker::sample> pose_tracker::sample_all(double alpha) const {
  std::vector<sample> out;
  sample_all(alpha, out);
  return out;
}

void pose_tracker::sample_all(double alpha, std::vector<sample> &out) const {
  out.clear();
  std::lock_guard<std::mutex> lock(_impl->m);
  out.reserve(_impl->entries.size());
  for (const impl::entry &e : _impl->entries)
    if (e.has_newest)
      out.push_back({e.id, impl::at(e, alpha)});
}

std::optional<pose> pose_tracker::sample_one(body_id id, double alpha) const {
  std::lock_guard<std::mutex> lock(_impl->m);
  const auto it = _impl->find(id);
  if (it == _impl->entries.end() || !it->has_newest)
    return std::nullopt;
  return impl::at(*it, alpha);
}

std::uint64_t pose_tracker::tick() const {
  std::lock_guard<std::mutex> lock(_impl->m);
  return _impl->tick;
}

} // namespace physics
} // namespace cvc

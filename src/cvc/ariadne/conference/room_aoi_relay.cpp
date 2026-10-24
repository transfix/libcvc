/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <atomic>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/conference/room_aoi_relay.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/holon.h>
#include <cvc/core/app.h>
#include <cvc/media/media_endpoint.h>
#include <cvc/media/media_frame.h>
#include <cvc/state/state.h> // SEPARATOR, findDescendant
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

using snapshot_entry = cvc::state_transport::snapshot_entry;

// A registered participant receives only the room tree, as every conference client's own
// enforced interest does.
const char *const kRoomTreePrefix = "conf";

// write_pose's change threshold (m, m/s, quaternion units): a subject whose state moved less than
// this since its last version wrote nothing new to its leaves either.
constexpr double kPoseEpsilon = 1e-4;

bool differs(double a, double b) { return std::fabs(a - b) > kPoseEpsilon; }

bool moved(const cvc::physics::body_state &a, const cvc::physics::body_state &b) {
  return differs(a.position.x, b.position.x) || differs(a.position.y, b.position.y) ||
         differs(a.position.z, b.position.z) || differs(a.rotation.w, b.rotation.w) ||
         differs(a.rotation.x, b.rotation.x) || differs(a.rotation.y, b.rotation.y) ||
         differs(a.rotation.z, b.rotation.z) || differs(a.linear_velocity.x, b.linear_velocity.x) ||
         differs(a.linear_velocity.y, b.linear_velocity.y) ||
         differs(a.linear_velocity.z, b.linear_velocity.z) ||
         differs(a.angular_velocity.x, b.angular_velocity.x) ||
         differs(a.angular_velocity.y, b.angular_velocity.y) ||
         differs(a.angular_velocity.z, b.angular_velocity.z);
}

// PID's pose subtree as push entries: its leaves as the relay's tree holds them, without the
// valueless intermediate nodes (pose, pose.pos, pose.vel, ...: a leaf's path recreates them), and
// the server_tick last, re-stamped with STAMP. Empty when the subject has no pose leaves yet.
std::vector<snapshot_entry> pose_entries(cvc::app &app, const cvc::state_cluster_shard &shard,
                                         const std::string &room, const std::string &pid,
                                         const std::string &stamp) {
  const std::string base = room_participant_path(room, pid) + cvc::state::SEPARATOR + "pose";
  std::vector<snapshot_entry> out;
  // shard.snapshot() would create a missing subtree's root node.
  if (!app.root().findDescendant(base))
    return out;
  const std::string tick_path = base + cvc::state::SEPARATOR + "server_tick";
  snapshot_entry tick;
  tick.path = tick_path;
  for (snapshot_entry &e : shard.snapshot(base)) {
    if (e.path == tick_path)
      tick = std::move(e);
    else if (!e.string_value.empty() || !e.data_type_name.empty())
      out.push_back(std::move(e));
  }
  if (out.empty())
    return out;
  tick.string_value = stamp;
  out.push_back(std::move(tick));
  return out;
}

} // namespace

// ---------------- the media gate's state ----------------

namespace {

// One (receiver, publisher, stream)'s thumbnail schedule (file-local, see room_aoi_relay.h).
struct gate_thumb {
  std::string receiver;
  std::string publisher;
  std::string stream_id;
  std::string codec;
  std::chrono::steady_clock::time_point due; // the next frame at or after this goes out
};

} // namespace

// What the gate closure owns: the snapshot it decides by, the thumbnail timers, and its counters.
// The driver swaps the snapshot in on the main loop; publisher reader threads read it.
struct room_aoi_relay::gate_state {
  explicit gate_state(double thumb_fps) {
    if (std::isfinite(thumb_fps) && thumb_fps > 0.0) {
      _thumbs_on = true;
      _thumb_every = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / thumb_fps));
    }
  }

  // Null (once the driver is gone) delivers everything.
  std::shared_ptr<const aoi_snapshot> load() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _snap;
  }
  void store(std::shared_ptr<const aoi_snapshot> s) {
    {
      std::lock_guard<std::mutex> lk(_mu);
      _snap.swap(s);
    }
    // The previous snapshot is released here, outside the lock.
  }

  // Whether RECEIVER gets F. A failure (an allocation in the timer map) delivers.
  bool admit(const cvc::media_frame &f, const std::string &receiver) noexcept {
    try {
      const std::shared_ptr<const aoi_snapshot> s = load();
      if (!s)
        return true;
      switch (s->media(receiver, f.origin_node_id, f.stream_id, f.codec)) {
      case aoi_media::deliver:
        full.fetch_add(1, std::memory_order_relaxed);
        return true;
      case aoi_media::thumbnail:
        if (thumbnail_due(f, receiver)) {
          thumbs.fetch_add(1, std::memory_order_relaxed);
          return true;
        }
        break;
      case aoi_media::drop:
        break;
      }
      dropped.fetch_add(1, std::memory_order_relaxed);
      return false;
    } catch (...) {
      return true;
    }
  }

  // Forget the timers of the pairs whose decision under S is no longer a thumbnail, so the next
  // time one becomes a thumbnail its first frame goes out at once. Called on the main loop with
  // each new snapshot.
  void sweep(const aoi_snapshot &s) {
    std::lock_guard<std::mutex> lk(_thumb_mu);
    for (auto it = _thumbs.begin(); it != _thumbs.end();) {
      const thumb &t = it->second;
      if (s.media(t.receiver, t.publisher, t.stream_id, t.codec) == aoi_media::thumbnail)
        ++it;
      else
        it = _thumbs.erase(it);
    }
  }

  std::atomic<std::uint64_t> full{0};
  std::atomic<std::uint64_t> thumbs{0};
  std::atomic<std::uint64_t> dropped{0};

private:
  using thumb = gate_thumb;

  // A pair's first thumbnail frame goes out at once; after that one per thumb interval, on the
  // interval's own schedule (so a 30 fps camera at 2 fps sends two a second), restarting from now
  // when the frames fell more than an interval behind it.
  bool thumbnail_due(const cvc::media_frame &f, const std::string &receiver) {
    if (!_thumbs_on)
      return false;
    const auto now = std::chrono::steady_clock::now();
    std::string key = receiver;
    key.push_back('\0');
    key += f.origin_node_id;
    key.push_back('\0');
    key += f.stream_id;
    std::lock_guard<std::mutex> lk(_thumb_mu);
    const auto it = _thumbs.find(key);
    if (it == _thumbs.end()) {
      _thumbs.emplace(std::move(key),
                      thumb{receiver, f.origin_node_id, f.stream_id, f.codec, now + _thumb_every});
      return true;
    }
    thumb &t = it->second;
    if (now < t.due)
      return false;
    t.due += _thumb_every;
    if (t.due <= now)
      t.due = now + _thumb_every;
    return true;
  }

  mutable std::mutex _mu;
  std::shared_ptr<const aoi_snapshot> _snap;
  bool _thumbs_on = false; // thumb_fps > 0; otherwise a thumbnailed camera is dropped
  std::chrono::steady_clock::duration _thumb_every{};
  std::mutex _thumb_mu;
  std::map<std::string, thumb> _thumbs; // receiver \0 publisher \0 stream_id
};

// ---------------- room_aoi_relay ----------------

namespace {

// The driver's bookkeeping (room_aoi_relay::book): file-local, see room_aoi_relay.h.
struct plane {
  cvc::state_transport *tp = nullptr;
  std::set<std::string> added;    // pids this driver registered (removed again on departure)
  std::set<std::string> filtered; // pids carrying this driver's filter
  std::map<std::string, std::uint64_t> filtered_seen; // deliveries_filtered at the last stats
};

struct filter_ref {
  std::shared_ptr<const cvc::state_delivery_filter> filter;
};
using filter_map = std::map<std::string, filter_ref>;

// One filter per present participant, bound to SNAP; none when SNAP is disabled.
filter_map filters_of(const std::shared_ptr<const aoi_snapshot> &snap) {
  filter_map out;
  if (!snap || !snap->enabled())
    return out;
  for (const std::string &pid : snap->participants())
    out.emplace(pid, filter_ref{std::make_shared<aoi_state_filter>(snap, pid)});
  return out;
}

void release(plane &pl, const std::string &pid) {
  pl.filtered_seen.erase(pid);
  cvc::state_peer_registry &reg = pl.tp->peers();
  if (pl.added.erase(pid)) {
    reg.remove_peer(pid);
    pl.filtered.erase(pid);
  } else if (pl.filtered.erase(pid)) {
    reg.set_filter(pid, nullptr);
  }
}

void install(plane &pl, const filter_map &filters, const std::string &cluster_id,
             const std::set<std::string> &departed) {
  cvc::state_peer_registry &reg = pl.tp->peers();
  for (const auto &kv : filters) {
    if (!reg.has_peer(kv.first)) {
      reg.add_peer(kv.first, cluster_id, std::string(), {kRoomTreePrefix});
      pl.added.insert(kv.first);
    }
    reg.set_filter(kv.first, kv.second.filter);
    pl.filtered.insert(kv.first);
  }
  for (const std::string &pid : departed)
    release(pl, pid);
}

} // namespace

struct room_aoi_relay::book {
  std::vector<plane> planes;
  std::map<std::string, cvc::physics::body_state> pose_ref; // the pose at its last version
  std::map<std::string, std::uint64_t> pose_version;        // tick of the last move > 1e-4
};

room_aoi_relay::room_aoi_relay(cvc::app &app, cvc::state_cluster_shard &shard, std::string room,
                               aoi_params p)
    : _app(app), _shard(shard), _aoi(std::move(room), std::move(p)),
      _book(std::make_unique<book>()),
      _gate(std::make_shared<gate_state>(_aoi.params().thumb_fps)) {
  _gate->store(_aoi.snapshot());
}

room_aoi_relay::~room_aoi_relay() {
  // The gates first: from here no new frame consults this driver, and a reader thread still
  // inside a copy of the gate delivers everything once the snapshot is gone.
  for (cvc::media_endpoint *ep : _endpoints)
    ep->set_media_gate({});
  _gate->store(nullptr);
  for (plane &pl : _book->planes) {
    cvc::state_peer_registry &reg = pl.tp->peers();
    for (const std::string &pid : pl.added)
      reg.remove_peer(pid);
    for (const std::string &pid : pl.filtered)
      if (!pl.added.count(pid))
        reg.set_filter(pid, nullptr);
  }
}

void room_aoi_relay::add_transport(cvc::state_transport *tp) {
  if (!tp)
    return;
  for (const plane &pl : _book->planes)
    if (pl.tp == tp)
      return;
  _book->planes.emplace_back();
  _book->planes.back().tp = tp;
  install(_book->planes.back(), filters_of(_aoi.snapshot()), _shard.cluster_id(), _aoi.departed());
}

void room_aoi_relay::add_media_endpoint(cvc::media_endpoint *ep) {
  if (!ep)
    return;
  for (const cvc::media_endpoint *e : _endpoints)
    if (e == ep)
      return;
  _endpoints.push_back(ep);
  // The closure owns the gate state, never `this`.
  ep->set_media_gate([st = _gate](const cvc::media_frame &f, const std::string &receiver) {
    return st->admit(f, receiver);
  });
}

void room_aoi_relay::step(const cvc::ariadne::holon &server, double now_s) {
  const std::uint64_t tick = server.tick();
  const std::vector<std::string> pids = server.participants();
  const std::set<std::string> here(pids.begin(), pids.end());
  std::vector<room_aoi::position> present;
  present.reserve(pids.size());
  for (const std::string &pid : pids) {
    const cvc::physics::body_state s = server.state(pid);
    present.push_back({pid, s.position.x, s.position.y});
    // A subject's version moves when its state does, by write_pose's measure, against the state
    // at its last version (so a slow drift still counts once it adds up).
    const auto [it, fresh] = _book->pose_ref.try_emplace(pid, s);
    if (fresh || moved(it->second, s)) {
      it->second = s;
      _book->pose_version[pid] = tick;
    }
  }
  for (auto it = _book->pose_ref.begin(); it != _book->pose_ref.end();) {
    if (here.count(it->first)) {
      ++it;
      continue;
    }
    _book->pose_version.erase(it->first);
    it = _book->pose_ref.erase(it);
  }
  if (_aoi.update(present, tick, now_s))
    apply(_aoi.snapshot());
  // Every step, so the inner pairs' bookkeeping keeps up with the live stream.
  push_poses(tick);
}

void room_aoi_relay::apply(const std::shared_ptr<const aoi_snapshot> &snap) {
  _gate->store(snap);
  _gate->sweep(*snap);
  // One filter per participant, shared by every registry.
  const filter_map filters = filters_of(snap);
  for (plane &pl : _book->planes)
    install(pl, filters, _shard.cluster_id(), _aoi.departed());
}

void room_aoi_relay::push_poses(std::uint64_t tick) {
  const std::vector<aoi_push> due = _aoi.take_pose_pushes(tick, _book->pose_version);
  if (due.empty() || _book->planes.empty())
    return;
  const std::string stamp = std::to_string(static_cast<unsigned long long>(tick));
  std::map<std::string, std::vector<snapshot_entry>> subtrees; // each subject read once a step
  for (const aoi_push &p : due) {
    std::vector<snapshot_entry> entries;
    for (const std::string &q : p.subjects) {
      auto it = subtrees.find(q);
      if (it == subtrees.end())
        it = subtrees.emplace(q, pose_entries(_app, _shard, _aoi.room(), q, stamp)).first;
      entries.insert(entries.end(), it->second.begin(), it->second.end());
    }
    if (entries.empty())
      continue;
    for (plane &pl : _book->planes) {
      const std::size_t n = pl.tp->push_snapshot_to(p.receiver, entries);
      _pushes += n;
      _push_entries += n * entries.size();
    }
  }
}

room_aoi_relay::stats room_aoi_relay::take_stats() {
  stats st;
  const std::shared_ptr<const aoi_snapshot> snap = _aoi.snapshot();
  const std::vector<std::string> &ps = snap->participants();
  for (std::size_t i = 0; i < ps.size(); ++i)
    for (std::size_t j = i + 1; j < ps.size(); ++j)
      switch (snap->tier(ps[i], ps[j])) {
      case aoi_tier::inner:
        ++st.inner;
        break;
      case aoi_tier::middle:
        ++st.middle;
        break;
      case aoi_tier::outer:
        ++st.outer;
        break;
      }
  st.pushes = std::exchange(_pushes, 0);
  st.push_entries = std::exchange(_push_entries, 0);
  st.media_full = _gate->full.exchange(0, std::memory_order_relaxed);
  st.media_thumb = _gate->thumbs.exchange(0, std::memory_order_relaxed);
  st.media_dropped = _gate->dropped.exchange(0, std::memory_order_relaxed);
  // The registries count per peer since registration; report what was added since the last call.
  for (plane &pl : _book->planes)
    for (const cvc::state_peer_registry::peer &peer : pl.tp->peers().snapshot()) {
      if (!pl.filtered.count(peer.node_id))
        continue;
      std::uint64_t &seen = pl.filtered_seen[peer.node_id];
      st.state_filtered += peer.deliveries_filtered >= seen ? peer.deliveries_filtered - seen
                                                            : peer.deliveries_filtered;
      seen = peer.deliveries_filtered;
    }
  return st;
}

std::shared_ptr<const aoi_snapshot> room_aoi_relay::snapshot() const { return _aoi.snapshot(); }

const aoi_params &room_aoi_relay::params() const noexcept { return _aoi.params(); }

// ---------------- the relay's room step ----------------

std::size_t relay_room_step(cvc::state_cluster_shard &shard,
                            const std::vector<cvc::state_transport *> &planes, room_aoi_relay *aoi,
                            const cvc::ariadne::holon &server, double now_s, bool trim_journal) {
  if (aoi)
    aoi->step(server, now_s);
  const std::vector<cvc::state_mutation> pending = shard.drain_local();
  for (const cvc::state_mutation &m : pending)
    for (cvc::state_transport *tp : planes)
      if (tp)
        tp->publish(m);
  if (trim_journal)
    shard.trim_published();
  return pending.size();
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/ariadne/conference/room_pose_relay.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/pose_stream.h>
#include <cvc/state/state_transport.h>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

std::uint32_t pose_send_interval_steps(double rtt_ms, std::uint32_t current) noexcept {
  if (!(rtt_ms >= 0.0))
    return current != 0 ? current : 2;
  // The thresholds move 10% away from the current rate, so it holds near one.
  double fast = 80.0, mid = 160.0;
  if (current == 1)
    fast *= 1.1;
  else if (current == 2)
    fast *= 0.9, mid *= 1.1;
  else if (current == 3)
    mid *= 0.9;
  return rtt_ms < fast ? 1 : (rtt_ms < mid ? 2 : 3);
}

namespace {
// One viewer's stream.
struct viewer {
  pose_stream_encoder enc;
  std::uint32_t interval = 0; // steps between snapshots (pose_send_interval_steps)
  std::uint32_t phase = 0;    // spreads viewers on the same interval over the steps
  std::uint64_t key_tick = 0; // its stream's last restart that was written (0: none yet)
  double rtt_ms = -1.0;
};
} // namespace

struct room_pose_relay::impl {
  config cfg;
  std::vector<cvc::state_transport *> transports;
  std::map<std::string, viewer> viewers;
  std::map<std::string, double> rtts; // measured before a viewer's first step, too
  std::uint32_t next_phase = 0;
  stats st;
};

room_pose_relay::room_pose_relay(config c) : _impl(std::make_unique<impl>()) { _impl->cfg = c; }
room_pose_relay::room_pose_relay() : room_pose_relay(config{}) {}
room_pose_relay::~room_pose_relay() = default;

void room_pose_relay::add_transport(cvc::state_transport *tp) {
  if (tp &&
      std::find(_impl->transports.begin(), _impl->transports.end(), tp) == _impl->transports.end())
    _impl->transports.push_back(tp);
}

void room_pose_relay::set_rtt(const std::string &pid, double rtt_ms) {
  _impl->rtts[pid] = rtt_ms;
  const auto it = _impl->viewers.find(pid);
  if (it != _impl->viewers.end()) {
    it->second.rtt_ms = rtt_ms;
    it->second.interval = pose_send_interval_steps(rtt_ms, it->second.interval);
  }
}

std::uint32_t room_pose_relay::interval_steps(const std::string &pid) const {
  const auto it = _impl->viewers.find(pid);
  return it == _impl->viewers.end() ? 0 : it->second.interval;
}

void room_pose_relay::step(const holon &server, std::uint64_t step, const aoi_snapshot *aoi) {
  impl &m = *_impl;
  const std::vector<std::string> pids = server.participants();
  // Forget viewers that left (their next connection starts a fresh stream).
  const std::set<std::string> present(pids.begin(), pids.end());
  for (auto it = m.viewers.begin(); it != m.viewers.end();)
    it = present.count(it->first) ? std::next(it) : m.viewers.erase(it);
  for (auto it = m.rtts.begin(); it != m.rtts.end();)
    it = present.count(it->first) ? std::next(it) : m.rtts.erase(it);
  if (pids.empty() || m.transports.empty())
    return;

  // Every participant's pose once.
  std::vector<std::pair<std::string, cvc::physics::body_state>> poses;
  poses.reserve(pids.size());
  for (const std::string &pid : pids)
    poses.emplace_back(pid, server.state(pid));
  const std::uint64_t tick = server.tick();

  std::vector<pose_subject> subjects;
  for (const std::string &pid : pids) {
    auto [it, fresh] = m.viewers.try_emplace(pid);
    viewer &v = it->second;
    if (fresh) {
      const auto r = m.rtts.find(pid);
      v.rtt_ms = r == m.rtts.end() ? -1.0 : r->second;
      v.interval = pose_send_interval_steps(v.rtt_ms);
      v.phase = m.next_phase++;
    }
    if ((step + v.phase) % v.interval != 0)
      continue;
    const bool key = v.key_tick == 0 || tick >= v.key_tick + m.cfg.keyframe_ticks;
    if (key)
      v.enc.reset(); // every participant it sees, named and absolute
    subjects.clear();
    for (const auto &[subject, state] : poses) {
      pose_subject s;
      s.pid = subject;
      s.state = state;
      const aoi_tier tier = aoi ? aoi->tier(pid, subject) : aoi_tier::inner;
      if (!key && tier != aoi_tier::inner) {
        const std::uint64_t last = v.enc.last_sent_tick(subject);
        s.due = last == 0 || tick >= last + m.cfg.push_interval_ticks;
      }
      subjects.push_back(std::move(s));
    }
    const cvc::pose_snapshot snap = v.enc.encode(tick, subjects, m.cfg.heartbeat_ticks);
    if (snap.entries.empty() && snap.gone.empty())
      continue; // nothing new for this viewer (it is all still, between heartbeats)
    std::size_t written = 0;
    for (cvc::state_transport *tp : m.transports)
      if ((written = tp->push_pose_snapshot_to(pid, snap)) > 0)
        break;
    if (written == 0) {
      ++m.st.unsent;
      continue; // not connected here (yet), or backed up: the next snapshot carries it
    }
    v.enc.commit();
    if (key)
      v.key_tick = tick;
    ++m.st.snapshots;
    m.st.entries += snap.entries.size();
  }
}

room_pose_relay::stats room_pose_relay::take_stats() {
  const stats s = _impl->st;
  _impl->st = stats{};
  return s;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

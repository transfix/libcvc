/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cstdint>
#include <cvc/ariadne/pose_stream.h>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {
constexpr double kMetre = 1000.0; // mm
constexpr double kUnit = 32767.0; // quaternion component scale

std::int32_t to_fixed(double v, double scale) {
  const double s = std::round(v * scale);
  if (!(s == s)) // NaN
    return 0;
  constexpr double lo = static_cast<double>(std::numeric_limits<std::int32_t>::min());
  constexpr double hi = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  return static_cast<std::int32_t>(s < lo ? lo : (s > hi ? hi : s));
}

// a - b and a + b, wrapping like the int32 the wire carries (a difference never overflows a
// decoder that adds it back the same way).
std::int32_t sub(std::int32_t a, std::int32_t b) {
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(a) - static_cast<std::uint32_t>(b));
}
std::int32_t add(std::int32_t a, std::int32_t b) {
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(a) + static_cast<std::uint32_t>(b));
}

// One connection's view of a participant: its slot and what it was last sent.
struct sent_state {
  std::uint32_t slot = 0;
  pose_q q;
  std::uint64_t tick = 0;
};
} // namespace

pose_q quantize_pose(const cvc::physics::body_state &s) {
  pose_q q;
  q.x = to_fixed(s.position.x, kMetre);
  q.y = to_fixed(s.position.y, kMetre);
  q.z = to_fixed(s.position.z, kMetre);
  q.vx = to_fixed(s.linear_velocity.x, kMetre);
  q.vy = to_fixed(s.linear_velocity.y, kMetre);
  q.vz = to_fixed(s.linear_velocity.z, kMetre);
  // One sign of the quaternion (w >= 0), so a rotation that did not change sends no difference.
  const double sign = s.rotation.w < 0.0 ? -1.0 : 1.0;
  q.qw = to_fixed(sign * s.rotation.w, kUnit);
  q.qx = to_fixed(sign * s.rotation.x, kUnit);
  q.qy = to_fixed(sign * s.rotation.y, kUnit);
  q.qz = to_fixed(sign * s.rotation.z, kUnit);
  return q;
}

cvc::physics::body_state dequantize_pose(const pose_q &q) {
  cvc::physics::body_state s;
  s.position = {q.x / kMetre, q.y / kMetre, q.z / kMetre};
  s.linear_velocity = {q.vx / kMetre, q.vy / kMetre, q.vz / kMetre};
  const double w = q.qw / kUnit, x = q.qx / kUnit, y = q.qy / kUnit, z = q.qz / kUnit;
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  s.rotation = n > 1e-9 ? cvc::physics::quat{w / n, x / n, y / n, z / n} : cvc::physics::quat{};
  s.active = true;
  return s;
}

// ── encoder ──

struct pose_stream_encoder::impl {
  // Committed: what the connection has.
  std::map<std::string, sent_state> sent;
  std::uint32_t next_slot = 1;
  std::uint32_t seq = 0;
  bool restart = true;
  // The last encode(), until it is committed.
  bool have_pending = false;
  std::map<std::string, sent_state> pending_sent;
  std::uint32_t pending_next_slot = 1;
  std::uint32_t pending_seq = 0;
};

pose_stream_encoder::pose_stream_encoder() : _impl(std::make_unique<impl>()) {}
pose_stream_encoder::~pose_stream_encoder() = default;
pose_stream_encoder::pose_stream_encoder(pose_stream_encoder &&) noexcept = default;
pose_stream_encoder &pose_stream_encoder::operator=(pose_stream_encoder &&) noexcept = default;

cvc::pose_snapshot pose_stream_encoder::encode(std::uint64_t tick,
                                               const std::vector<pose_subject> &subjects,
                                               std::uint64_t heartbeat_ticks) {
  impl &m = *_impl;
  cvc::pose_snapshot out;
  out.server_tick = tick;
  out.seq = m.seq + 1 == 0 ? 1 : m.seq + 1; // 0 is reserved for "restart"
  out.base_seq = m.restart ? 0 : m.seq;
  std::map<std::string, sent_state> next = m.restart ? std::map<std::string, sent_state>{} : m.sent;
  std::uint32_t next_slot = m.next_slot;
  std::set<std::string> present;
  for (const pose_subject &subject : subjects) {
    present.insert(subject.pid);
    const auto it = next.find(subject.pid);
    if (!subject.due)
      continue;
    const pose_q q = quantize_pose(subject.state);
    cvc::pose_entry e;
    if (it == next.end()) { // new to the connection: named, absolute
      sent_state st;
      st.slot = next_slot++;
      st.q = q;
      st.tick = tick;
      next.emplace(subject.pid, st);
      e.slot = st.slot;
      e.pid = subject.pid;
      e.x = q.x, e.y = q.y, e.z = q.z;
      e.vx = q.vx, e.vy = q.vy, e.vz = q.vz;
      e.qw = q.qw, e.qx = q.qx, e.qy = q.qy, e.qz = q.qz;
      out.entries.push_back(std::move(e));
      continue;
    }
    sent_state &st = it->second;
    const bool moved = q != st.q;
    if (!moved && tick < st.tick + heartbeat_ticks)
      continue;
    e.slot = st.slot;
    e.x = sub(q.x, st.q.x), e.y = sub(q.y, st.q.y), e.z = sub(q.z, st.q.z);
    e.vx = sub(q.vx, st.q.vx), e.vy = sub(q.vy, st.q.vy), e.vz = sub(q.vz, st.q.vz);
    e.qw = sub(q.qw, st.q.qw), e.qx = sub(q.qx, st.q.qx), e.qy = sub(q.qy, st.q.qy);
    e.qz = sub(q.qz, st.q.qz);
    st.q = q;
    st.tick = tick;
    out.entries.push_back(std::move(e));
  }
  for (auto it = next.begin(); it != next.end();) {
    if (present.count(it->first)) {
      ++it;
      continue;
    }
    out.gone.push_back(it->second.slot);
    it = next.erase(it);
  }
  m.pending_sent = std::move(next);
  m.pending_next_slot = next_slot;
  m.pending_seq = out.seq;
  m.have_pending = true;
  return out;
}

void pose_stream_encoder::commit() {
  impl &m = *_impl;
  if (!m.have_pending)
    return;
  m.sent = std::move(m.pending_sent);
  m.next_slot = m.pending_next_slot;
  m.seq = m.pending_seq;
  m.restart = false;
  m.have_pending = false;
}

void pose_stream_encoder::reset() {
  impl &m = *_impl;
  m.sent.clear();
  m.restart = true;
  m.have_pending = false;
}

std::uint64_t pose_stream_encoder::last_sent_tick(const std::string &pid) const {
  const auto it = _impl->sent.find(pid);
  return it == _impl->sent.end() ? 0 : it->second.tick;
}

// ── decoder ──

struct pose_stream_decoder::impl {
  struct slot_state {
    std::string pid;
    pose_q q;
  };
  std::map<std::uint32_t, slot_state> slots;
  bool started = false;
  std::uint32_t seq = 0;
  std::uint64_t gaps = 0;
};

pose_stream_decoder::pose_stream_decoder() : _impl(std::make_unique<impl>()) {}
pose_stream_decoder::~pose_stream_decoder() = default;
pose_stream_decoder::pose_stream_decoder(pose_stream_decoder &&) noexcept = default;
pose_stream_decoder &pose_stream_decoder::operator=(pose_stream_decoder &&) noexcept = default;

bool pose_stream_decoder::decode(const cvc::pose_snapshot &snap, std::vector<pose_sample> &out) {
  impl &m = *_impl;
  if (snap.base_seq == 0) {
    m.slots.clear();
    m.started = true;
  } else if (!m.started || snap.base_seq != m.seq) {
    ++m.gaps;
    return false;
  }
  m.seq = snap.seq;
  for (std::uint32_t g : snap.gone)
    m.slots.erase(g);
  for (const cvc::pose_entry &e : snap.entries) {
    impl::slot_state *st = nullptr;
    if (!e.pid.empty()) { // named: absolute
      impl::slot_state &n = m.slots[e.slot];
      n.pid = e.pid;
      n.q = pose_q{e.x, e.y, e.z, e.vx, e.vy, e.vz, e.qw, e.qx, e.qy, e.qz};
      st = &n;
    } else {
      const auto it = m.slots.find(e.slot);
      if (it == m.slots.end())
        continue; // a slot this stream never named: nothing to add the difference to
      st = &it->second;
      pose_q &q = st->q;
      q.x = add(q.x, e.x), q.y = add(q.y, e.y), q.z = add(q.z, e.z);
      q.vx = add(q.vx, e.vx), q.vy = add(q.vy, e.vy), q.vz = add(q.vz, e.vz);
      q.qw = add(q.qw, e.qw), q.qx = add(q.qx, e.qx), q.qy = add(q.qy, e.qy);
      q.qz = add(q.qz, e.qz);
    }
    out.push_back(pose_sample{st->pid, dequantize_pose(st->q)});
  }
  return true;
}

std::uint64_t pose_stream_decoder::gaps() const noexcept { return _impl->gaps; }

} // namespace ariadne
} // namespace cvc

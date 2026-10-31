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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/holon.h>
#include <exception>
#include <iterator>

namespace cvc {
namespace ariadne {

namespace {
// Observer order within a frame: integrate the sim (step phase), THEN record the
// presentation alpha (frame phase) -- matching tick_scene's advance -> follow ->
// present, so a frame observer never reads a pose the step observer has not yet
// produced this frame.
constexpr int kOrderIntegrate = 10;
constexpr int kOrderPresent = 10;

// Peer playout (holon::display_pose). Until this many samples have arrived the timeline snaps to
// its target (the first one read can be a still peer's heartbeat-old pose, whose tick says nothing
// of the delay); after it, it eases, and snaps only when this far (ticks) out.
constexpr int kSettleSamples = 3;
constexpr double kRetimeSnapTicks = 60.0;
// It follows its target through a lag of this time constant (s): each arrival nudges the target a
// fraction of a tick, and taking that up within a frame would twitch the peer's speed.
constexpr double kRetimeLagS = 0.5;
// The best arrival lead decays at this fraction of real time, so a slower route is adopted.
constexpr double kLeadDecay = 0.02;
// Lateness rises fast and falls slowly (a burst of late samples widens the delay at once, and it
// narrows again over seconds); the interval follows at this rate.
constexpr double kLateRise = 0.25, kLateFall = 0.02, kIntervalRate = 0.1;
constexpr std::size_t kMaxPeerSamples = 64;
constexpr double kStillSpeed = 0.05; // m/s: a sample this slow is a peer standing still

using cvc::physics::quat;
using cvc::physics::vec3;

double length(const vec3 &v) { return std::sqrt(cvc::physics::dot(v, v)); }
bool all_finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
quat mul(const quat &a, const quat &b) {
  return quat{
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
quat conj(const quat &q) { return quat{q.w, -q.x, -q.y, -q.z}; }
// Normalized, on the w >= 0 side (the shorter way round from the identity).
quat unit(quat q) {
  if (q.w < 0.0)
    q = quat{-q.w, -q.x, -q.y, -q.z};
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return n > 0.0 && std::isfinite(n) ? quat{q.w / n, q.x / n, q.y / n, q.z / n} : quat{};
}
} // namespace

holon::holon(holon_role role, holon_config cfg)
    : _role(role), _cfg(cfg), _clock(cfg.clock), _driver(_clock) {
  if (_cfg.physics) {
    try {
      _world = std::make_unique<avatar_world>(_clock.fixed_dt(), _cfg.body);
    } catch (const std::exception &e) {
      std::fprintf(stderr,
                   "holon: no avatar physics in this build (%s); avatars move kinematically and "
                   "do not collide\n",
                   e.what());
    }
  }
  _driver.add_step_observer(kOrderIntegrate,
                            [this](const cvc::clock_driver::clock_frame &f) { on_step(f); });
  _driver.add_frame_observer(kOrderPresent,
                             [this](const cvc::clock_driver::clock_frame &f) { on_present(f); });
}

void holon::add_participant(const std::string &pid, bool local,
                            const cvc::physics::body_state &start, bool pose_known) {
  participant p;
  p.local = local;
  p.pose_known = pose_known || _role == holon_role::server;
  p.sim.set_state(start);
  p.predicted = predicted_avatar(start);
  p.newest = start;
  p.spawn = start;
  _participants[pid] = p;
  // In the world: the server simulates everyone, a client its own avatar; a client's peers are
  // kinematic, held where they are shown -- once there is somewhere to show them (a peer whose
  // pose is not known yet gets its body with its first sample, apply_authoritative).
  if (!_world)
    return;
  const bool dynamic = _role == holon_role::server || local;
  if (dynamic || p.pose_known)
    _world->add_avatar(pid, start, dynamic);
  else
    _world->remove_avatar(pid); // re-registered pose-less: no body left at its old pose
}

void holon::remove_participant(const std::string &pid) {
  const auto it = _participants.find(pid);
  if (it != _participants.end() && _role == holon_role::client && it->second.have_seen) {
    _left[pid] = it->second.last_seen;
    if (_left.size() > kMaxLeft)
      _left.erase(_left.begin() == _left.find(pid) ? std::next(_left.begin()) : _left.begin());
  }
  _participants.erase(pid);
  if (_world)
    _world->remove_avatar(pid);
}

void holon::set_statics(const std::vector<avatar_static> &statics, std::uint64_t revision) {
  _statics_rev = revision;
  if (_world)
    _world->set_statics(statics);
}

std::uint64_t holon::left_tick(const std::string &pid) const {
  const auto it = _left.find(pid);
  return it == _left.end() ? 0 : it->second;
}

bool holon::has_participant(const std::string &pid) const {
  return _participants.find(pid) != _participants.end();
}

bool holon::pose_known(const std::string &pid) const {
  auto it = _participants.find(pid);
  return it != _participants.end() && it->second.pose_known;
}

bool holon::has_body(const std::string &pid) const {
  return _world && _participants.count(pid) && _world->has_avatar(pid);
}

std::vector<std::string> holon::participants() const {
  std::vector<std::string> out;
  out.reserve(_participants.size());
  for (const auto &kv : _participants)
    out.push_back(kv.first);
  return out;
}

avatar_intent holon::sanitize_intent(const avatar_intent &in) const {
  avatar_intent out = in;
  const cvc::physics::vec3 &m = in.move;
  if (!std::isfinite(m.x) || !std::isfinite(m.y) || !std::isfinite(m.z))
    out.move = cvc::physics::vec3{0.0, 0.0, 0.0};
  if (!_cfg.allow_vertical)
    out.move.z = 0.0;
  const double h2 = out.move.x * out.move.x + out.move.y * out.move.y;
  if (h2 > _cfg.max_speed * _cfg.max_speed) {
    const double k = _cfg.max_speed / std::sqrt(h2);
    out.move.x *= k;
    out.move.y *= k;
  }
  constexpr double kPi = 3.14159265358979323846;
  if (!std::isfinite(out.yaw))
    out.yaw = 0.0;
  else if (out.yaw > kPi || out.yaw < -kPi)
    out.yaw = std::remainder(out.yaw, 2.0 * kPi);
  return out;
}

namespace {
bool same_intent(const avatar_intent &a, const avatar_intent &b) {
  // Bitwise-ish: a NaN in `a` differs from whatever sanitize_intent made of it.
  return a.move.x == b.move.x && a.move.y == b.move.y && a.move.z == b.move.z && a.yaw == b.yaw;
}
} // namespace

void holon::set_intent(const std::string &pid, const avatar_intent &in) {
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  // A client may drive only its OWN (local) avatar; it cannot move a peer.
  if (_role == holon_role::client && !it->second.local)
    return;
  const avatar_intent ok = sanitize_intent(in);
  if (_role == holon_role::server && !same_intent(in, ok))
    ++_vetoes;
  it->second.intent = ok;
  it->second.queued.clear();
}

bool holon::override_state(const std::string &pid, const cvc::physics::body_state &feet) {
  if (_role != holon_role::server)
    return false;
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return false;
  if (_world)
    _world->place(pid, feet);
  else
    it->second.sim.set_state(feet);
  return true;
}

bool holon::set_mass(const std::string &pid, double kg) {
  if (_role != holon_role::server || !_world || !(kg > 0.0) || !std::isfinite(kg))
    return false;
  if (!_participants.count(pid))
    return false;
  _world->set_mass(pid, kg);
  return true;
}

void holon::queue_intent(const std::string &pid, const avatar_intent &in, std::uint64_t tick) {
  if (_role != holon_role::server)
    return;
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  participant &p = it->second;
  if (p.have_stamp && tick == p.last_stamp)
    return; // already queued (the host re-reads an unchanged intent)
  p.have_stamp = true;
  p.last_stamp = tick;
  // The quantum it can still take effect at: its own tick if that is still to come, else the
  // next one (late), and never before what is already queued (stamps from a client whose clock
  // moved backwards keep their arrival order). Stamped beyond the allowed lead: at once.
  const std::uint64_t next = _clock.tick() + 1;
  std::uint64_t at = tick < next ? next : tick;
  if (at > next + _cfg.max_intent_lead_ticks)
    at = next;
  if (!p.queued.empty() && at < p.queued.back().first)
    at = p.queued.back().first;
  const avatar_intent ok = sanitize_intent(in);
  if (!same_intent(in, ok))
    ++_vetoes;
  p.queued.emplace_back(at, ok);
}

void holon::apply_authoritative(const std::string &pid, const cvc::physics::body_state &auth,
                                std::uint64_t server_tick) {
  if (_role != holon_role::client)
    return; // the server IS the authority; nothing to apply
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  if (it->second.have_seen && server_tick <= it->second.last_seen)
    return; // seen already (an unchanged pose re-read), or older
  it->second.have_seen = true;
  it->second.last_seen = server_tick;
  it->second.pose_known = true;
  _left.erase(pid); // a real sample: whatever it left behind before is superseded
  // Count the server's ticks (see the header): re-align whenever the replica is more
  // than max_skew_ticks away -- on the first sample when the client joined a running
  // server, and after a stall. A replica already within the skew is left alone, so its
  // logged inputs still line up with the server's ticks and replay correctly.
  const std::uint64_t now = _clock.tick();
  const std::uint64_t target = server_tick + _lead;
  const bool behind = now < target;
  const std::uint64_t skew = behind ? target - now : now - target;
  const std::uint64_t tol = _lead == 0 ? _cfg.max_skew_ticks
                            : behind   ? _cfg.lead_slack_behind_ticks
                                       : _cfg.lead_slack_ahead_ticks;
  // Within reach, slew instead (holon_config::lead_slew): the logged inputs stay valid, so the
  // next sample reconciles without a jump. Keep slewing until within a tick of the target. The
  // first sample (joining) seeks if it must: there is nothing logged to keep yet, and slewing into
  // it would leave the inputs late meanwhile. After it, slew -- whether that sample seeked or found
  // the clock already in step (a client that started with the server never seeks to join).
  const bool can_slew = _lead > 0 && _cfg.lead_slew > 0.0 && skew <= _cfg.max_skew_ticks && _synced;
  _synced = true;
  if (can_slew && (skew > tol || (_slewing && skew > 1))) {
    _clock.set_scale(_cfg.clock.scale * (behind ? 1.0 + _cfg.lead_slew : 1.0 - _cfg.lead_slew));
    if (!_slewing)
      ++_slews;
    _slewing = true;
  } else if (_slewing) {
    _clock.set_scale(_cfg.clock.scale);
    _slewing = false;
  }
  if (skew > tol && !can_slew) {
    ++_realigns;
    _clock.seek_tick(target);
    // The logged inputs' ticks are on the old timeline: keep the state only. (Peers play out on
    // wall time, display_pose, so the seek does not move them.)
    for (auto &kv : _participants) {
      participant &q = kv.second;
      if (q.local) {
        q.predicted = predicted_avatar(q.predicted.state());
        q.log.clear();
        q.acked = false;
      }
    }
  }

  participant &p = it->second;
  if (p.local && _world) {
    world_reconcile(pid, p, auth, server_tick);
  } else if (p.local) {
    const cvc::physics::vec3 before = p.predicted.state().position;
    if (p.predicted.reconcile(auth, server_tick, _clock.fixed_dt(), _cfg.reconcile_tol)) {
      const cvc::physics::vec3 d = p.predicted.state().position - before;
      p.correction = p.correction + d;
      p.render_prev.position = p.render_prev.position + d;
    }
  } else {
    // A peer. Reject a stale/duplicate/reordered sample (server_tick <= the newest
    // one applied), exactly as predicted_avatar does for the local path: otherwise
    // an out-of-order sample would rewind the interpolation window (prev <- newer,
    // newest <- older) and visibly jerk the peer backward, or a duplicate would
    // collapse the window (prev == newest) and freeze it.
    if (p.have_sample && server_tick <= p.last_sample_tick)
      return;
    // A peer registered before its pose was known has no body yet: its first sample is the first
    // place it can be held (world_step_client keeps it at its shown pose from here on).
    if (_world && !_world->has_avatar(pid))
      _world->add_avatar(pid, auth, /*dynamic=*/false);
    receive_peer_sample(p, auth, server_tick);
    p.last_sample_tick = server_tick;
  }
}

void holon::receive_peer_sample(participant &p, const cvc::physics::body_state &auth,
                                std::uint64_t server_tick) {
  const double now = _peer_clock;
  const double lead = static_cast<double>(server_tick) - now;
  if (!p.have_sample) {
    p.have_sample = true;
    p.samples.assign(1, participant::sample{server_tick, auth});
    p.newest = auth;
    p.samples_seen = 1;
    p.arrival_lead = lead;
    p.last_arrival = now;
    p.lateness = 0.0;
    p.interval = static_cast<double>(_cfg.peer_interval_ticks);
    p.interval_known = false;
    p.last_moved = false;
    p.play_offset = -now + static_cast<double>(server_tick); // held at it (present_peer)
    p.have_shown = false;
    return;
  }
  const participant::sample last = p.samples.back();
  const std::uint64_t span = server_tick - last.tick;
  const double fixed_dt = _clock.fixed_dt();
  const double span_s = static_cast<double>(span) * fixed_dt;
  // Still: its velocity says so and it had not moved since the sample before (a velocity is not
  // trusted alone: a sender may leave it out).
  const bool was_still = length(last.s.linear_velocity) < kStillSpeed && !p.last_moved;

  // The timeline: the best lead decays while no sample beats it, then this one is measured
  // against it; the interval learns from samples of a peer on the move (a still one's heartbeat
  // gap is no interval).
  p.arrival_lead -= kLeadDecay * std::max(0.0, now - p.last_arrival);
  p.last_arrival = now;
  if (lead > p.arrival_lead)
    p.arrival_lead = lead;
  const double late = p.arrival_lead - lead;
  p.lateness += (late > p.lateness ? kLateRise : kLateFall) * (late - p.lateness);
  if (!was_still) {
    const auto spanned =
        static_cast<double>(std::min<std::uint64_t>(span, _cfg.max_peer_span_ticks));
    p.interval = p.interval_known ? p.interval + kIntervalRate * (spanned - p.interval) : spanned;
    p.interval_known = true;
  }
  if (p.samples_seen < kSettleSamples)
    ++p.samples_seen;

  // A move no walk explains (a teleport, a respawn): start the path again at it, shown at once.
  const double reach = (3.0 * _cfg.max_speed +
                        0.5 * (length(last.s.linear_velocity) + length(auth.linear_velocity))) *
                           span_s +
                       1.0;
  if (!(length(auth.position - last.s.position) <= reach)) {
    p.samples.assign(1, participant::sample{server_tick, auth});
    p.newest = auth;
    p.last_moved = false;
    p.have_shown = false;
    return;
  }
  // A peer standing still sends only a heartbeat, so whatever moved it happened within the last
  // interval before this sample, not across the whole gap: hold it until then.
  const auto step = static_cast<std::uint64_t>(std::max(1.0, std::round(p.interval)));
  if (was_still && span > step + 2) {
    participant::sample hold{server_tick - step, last.s};
    hold.s.linear_velocity = vec3{0.0, 0.0, 0.0};
    hold.s.angular_velocity = vec3{0.0, 0.0, 0.0};
    p.samples.push_back(hold);
  }
  p.samples.push_back(participant::sample{server_tick, auth});
  p.last_moved = length(auth.position - last.s.position) > 1e-3;
  while (p.samples.size() > kMaxPeerSamples)
    p.samples.pop_front();
  p.newest = auth;
  // Whatever this sample changes about the path where it is shown now eases out from here: the
  // shown pose stays put this instant (shown = path + ease, the path moved, the ease takes up the
  // difference).
  if (p.have_shown) {
    const cvc::physics::pose now_path = peer_path(p, p.render_tick);
    p.ease_pos = p.ease_pos + (p.path.position - now_path.position);
    p.ease_rot = unit(mul(p.ease_rot, mul(p.path.rotation, conj(now_path.rotation))));
    p.path = now_path;
    if (!(length(p.ease_pos) <= _cfg.peer_snap_distance)) {
      p.ease_pos = vec3{0.0, 0.0, 0.0};
      p.ease_rot = quat{};
    }
  }
}

cvc::physics::pose holon::peer_path(const participant &p, double t) const {
  const std::deque<participant::sample> &q = p.samples;
  const participant::sample &first = q.front(), &last = q.back();
  if (!(t > static_cast<double>(first.tick)))
    return cvc::physics::pose{first.s.position, first.s.rotation};
  const double fixed_dt = _clock.fixed_dt();
  if (t >= static_cast<double>(last.tick)) {
    // Past the newest: on along its velocity, for a while.
    const double ahead = std::min(t - static_cast<double>(last.tick),
                                  static_cast<double>(_cfg.peer_max_extrapolate_ticks)) *
                         fixed_dt;
    const vec3 v =
        all_finite(last.s.linear_velocity) ? last.s.linear_velocity : vec3{0.0, 0.0, 0.0};
    return cvc::physics::pose{last.s.position + v * ahead, last.s.rotation};
  }
  std::size_t i = 0;
  while (i + 2 < q.size() && static_cast<double>(q[i + 1].tick) <= t)
    ++i;
  const participant::sample &a = q[i], &b = q[i + 1];
  const double span = static_cast<double>(b.tick - a.tick);
  const double u = (t - static_cast<double>(a.tick)) / span;
  const double h = span * fixed_dt;
  // A cubic Hermite through both samples with their velocities as tangents: the peer moves with
  // the speed it had, accelerating and turning between samples rather than at them. A velocity
  // that does not fit the move (a sample whose velocity is missing, or one the physics clipped)
  // would overshoot; then the segment is a straight line.
  const vec3 d = b.s.position - a.s.position;
  const double fit = std::max(0.5 * length(d), 0.02);
  const vec3 va = a.s.linear_velocity, vb = b.s.linear_velocity;
  const bool use_vel =
      all_finite(va) && all_finite(vb) && length(va * h - d) <= fit && length(vb * h - d) <= fit;
  cvc::physics::pose out =
      cvc::physics::interpolate(cvc::physics::pose{a.s.position, a.s.rotation},
                                cvc::physics::pose{b.s.position, b.s.rotation}, u);
  if (use_vel) {
    const double u2 = u * u, u3 = u2 * u;
    out.position = a.s.position * (2.0 * u3 - 3.0 * u2 + 1.0) + va * (h * (u3 - 2.0 * u2 + u)) +
                   b.s.position * (3.0 * u2 - 2.0 * u3) + vb * (h * (u3 - u2));
  }
  return out;
}

void holon::present_peer(participant &p, double frame_ticks) {
  // The delay: one interval, the jitter margin and a tick for the frame the arrival is measured
  // at; at least an interval, at most peer_max_delay_ticks. The shown tick follows it at no more
  // than peer_retime of real time and through a lag (kRetimeLagS), or snaps while the timeline
  // settles or when far out.
  const double hi = std::max(p.interval + 1.0, static_cast<double>(_cfg.peer_max_delay_ticks));
  const double delay = std::min(hi, p.interval + _cfg.peer_jitter_margin * p.lateness + 1.0);
  // With one sample there is no delay to know yet: hold it until the second (which then starts the
  // walk from it, not from a guess it would have to step back from).
  const double want = p.samples_seen < 2 ? static_cast<double>(p.samples.back().tick) - _peer_clock
                                         : p.arrival_lead - delay;
  const double gap = want - p.play_offset;
  const double most = _cfg.peer_retime * frame_ticks;
  if (p.samples_seen < kSettleSamples || std::fabs(gap) > kRetimeSnapTicks)
    p.play_offset = want;
  else
    p.play_offset += std::clamp(
        gap * (1.0 - std::exp(-frame_ticks * _clock.fixed_dt() / kRetimeLagS)), -most, most);
  p.render_tick = _peer_clock + p.play_offset;
  // Samples wholly behind the shown tick are spent; keep the one it walks on from.
  while (p.samples.size() > 1 && static_cast<double>(p.samples[1].tick) <= p.render_tick)
    p.samples.pop_front();
  p.path = peer_path(p, p.render_tick);
  if (!p.have_shown) {
    p.ease_pos = vec3{0.0, 0.0, 0.0};
    p.ease_rot = quat{};
  } else {
    const double k =
        std::exp(-frame_ticks * _clock.fixed_dt() / std::max(1e-6, _cfg.peer_smoothing_s));
    p.ease_pos = p.ease_pos * k;
    p.ease_rot = unit(
        quat{1.0 + (p.ease_rot.w - 1.0) * k, p.ease_rot.x * k, p.ease_rot.y * k, p.ease_rot.z * k});
  }
  p.shown =
      cvc::physics::pose{p.path.position + p.ease_pos, unit(mul(p.ease_rot, p.path.rotation))};
  p.have_shown = true;
}

void holon::world_step_client(const std::string &self, const avatar_intent &in) {
  // Peers are kinematic: hold each where it is shown, so we bump into what we see.
  for (const auto &kv : _participants)
    if (!kv.second.local && _world->has_avatar(kv.first))
      _world->move_kinematic(kv.first, display_pose(kv.first));
  _world->drive(self, in);
  _world->step();
}

bool holon::world_reconcile(const std::string &pid, participant &p,
                            const cvc::physics::body_state &auth, std::uint64_t server_tick) {
  // predicted_avatar::reconcile over the world: a stale or duplicate ack is a no-op; a prediction
  // within tolerance at server_tick holds; otherwise snap to the authority and replay the logged
  // inputs after it (the peers held where they are shown now).
  if (p.acked && server_tick <= p.last_acked)
    return false;
  p.acked = true;
  p.last_acked = server_tick;
  const double tol2 = _cfg.reconcile_tol * _cfg.reconcile_tol;
  for (const participant::logged &e : p.log)
    if (e.tick == server_tick) {
      const cvc::physics::vec3 d = e.after.position - auth.position;
      if (cvc::physics::dot(d, d) <= tol2) {
        while (!p.log.empty() && p.log.front().tick <= server_tick)
          p.log.pop_front();
        return false;
      }
      break;
    }
  const cvc::physics::vec3 before = _world->state(pid).position;
  _world->place(pid, auth);
  while (!p.log.empty() && p.log.front().tick <= server_tick)
    p.log.pop_front();
  for (participant::logged &e : p.log) {
    world_step_client(pid, e.in);
    e.after = _world->state(pid);
  }
  const cvc::physics::vec3 d = _world->state(pid).position - before;
  p.correction = p.correction + d;
  p.render_prev.position = p.render_prev.position + d; // the interpolation moves with it
  return true;
}

void holon::on_step(const cvc::clock_driver::clock_frame &f) {
  const double dt = f.fixed_dt;
  const int steps = f.step.steps;
  if (_world) {
    // Rigid bodies. Nothing moves until the room's statics exist (there is nothing to stand on).
    if (_statics_rev == 0)
      return;
    for (int i = 0; i < steps; ++i) {
      const std::uint64_t tick = f.step.first_tick + 1 + static_cast<std::uint64_t>(i);
      if (_role == holon_role::server) {
        for (auto &kv : _participants) {
          participant &p = kv.second;
          while (!p.queued.empty() && p.queued.front().first <= tick) {
            p.intent = p.queued.front().second;
            p.queued.pop_front();
          }
          _world->drive(kv.first, p.intent);
        }
        _world->step();
        for (auto &kv : _participants)
          if (_world->state(kv.first).position.z < _cfg.respawn_below_z)
            _world->place(kv.first, kv.second.spawn); // fell off the world
      } else {
        const auto self = std::find_if(_participants.begin(), _participants.end(),
                                       [](const auto &kv) { return kv.second.local; });
        if (self == _participants.end())
          continue;
        self->second.render_prev = _world->state(self->first);
        self->second.have_render_prev = true;
        world_step_client(self->first, self->second.intent);
        participant::logged e;
        e.tick = tick;
        e.in = self->second.intent;
        e.after = _world->state(self->first);
        self->second.log.push_back(e);
        if (self->second.log.size() > predicted_avatar::kMaxLog)
          self->second.log.pop_front();
      }
    }
    return;
  }
  for (auto &kv : _participants) {
    participant &p = kv.second;
    if (_role == holon_role::server) {
      // Authoritative integration: advance over every quantum that fell due this
      // frame, each by the intent in force at its tick (a stamped intent takes over
      // at the tick it was queued for).
      for (int i = 0; i < steps; ++i) {
        const std::uint64_t tick = f.step.first_tick + 1 + static_cast<std::uint64_t>(i);
        while (!p.queued.empty() && p.queued.front().first <= tick) {
          p.intent = p.queued.front().second;
          p.queued.pop_front();
        }
        p.sim.step(p.intent, dt);
      }
    } else if (p.local) {
      // Client prediction: predict the local avatar one quantum at a time, tagging
      // each with its tick so reconcile can replay the right suffix.
      for (int i = 0; i < steps; ++i) {
        const std::uint64_t tick = f.step.first_tick + 1 + static_cast<std::uint64_t>(i);
        p.render_prev = p.predicted.state();
        p.have_render_prev = true;
        p.predicted.predict(tick, p.intent, dt);
      }
    }
    // Peers integrate nothing: they are driven by apply_authoritative samples.
  }
}

void holon::on_present(const cvc::clock_driver::clock_frame &f) {
  _alpha = f.alpha;
  if (_role != holon_role::client)
    return;
  // The peers' clock: wall time, at the rate the server's clock runs (its configured scale; this
  // replica's own clock is slewed and re-aligned toward its lead, which a peer must not see).
  const double frame_ticks =
      f.wall_dt > 0.0 && f.fixed_dt > 0.0 ? f.wall_dt * _cfg.clock.scale / f.fixed_dt : 0.0;
  _peer_clock += frame_ticks;
  for (auto &kv : _participants)
    if (!kv.second.local && kv.second.have_sample)
      present_peer(kv.second, frame_ticks);
}

void holon::advance(double wall_dt) { _driver.advance(wall_dt); }

cvc::physics::body_state holon::state(const std::string &pid) const {
  const participant &p = _participants.at(pid);
  if (_world && (_role == holon_role::server || p.local))
    return _world->state(pid);
  if (_role == holon_role::server)
    return p.sim.state();
  if (p.local)
    return p.predicted.state();
  return p.newest;
}

cvc::physics::pose holon::display_pose(const std::string &pid) const {
  const participant &p = _participants.at(pid);
  // The client's own avatar: between its last two quanta, by the frame's alpha.
  if (_role == holon_role::client && p.local && p.have_render_prev)
    return interpolate_peer(p.render_prev, _world ? _world->state(pid) : p.predicted.state(),
                            _alpha);
  if (_world && (_role == holon_role::server || p.local)) {
    const cvc::physics::body_state s = _world->state(pid);
    return cvc::physics::pose{s.position, s.rotation};
  }
  if (_role == holon_role::server) {
    const cvc::physics::body_state &s = p.sim.state();
    return cvc::physics::pose{s.position, s.rotation};
  }
  if (p.local) {
    const cvc::physics::body_state &s = p.predicted.state();
    return cvc::physics::pose{s.position, s.rotation};
  }
  // Peer: where this frame's playout put it (present_peer); before its first frame, its first
  // sample (or the pose it was added with).
  if (p.have_shown)
    return p.shown;
  if (p.have_sample)
    return cvc::physics::pose{p.samples.front().s.position, p.samples.front().s.rotation};
  return cvc::physics::pose{p.newest.position, p.newest.rotation};
}

cvc::physics::vec3 holon::take_correction(const std::string &pid) {
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return cvc::physics::vec3{0.0, 0.0, 0.0};
  const cvc::physics::vec3 c = it->second.correction;
  it->second.correction = cvc::physics::vec3{0.0, 0.0, 0.0};
  return c;
}

} // namespace ariadne
} // namespace cvc

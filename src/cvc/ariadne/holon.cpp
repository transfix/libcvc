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
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/holon.h>
#include <exception>

namespace cvc {
namespace ariadne {

namespace {
// Observer order within a frame: integrate the sim (step phase), THEN record the
// presentation alpha (frame phase) -- matching tick_scene's advance -> follow ->
// present, so a frame observer never reads a pose the step observer has not yet
// produced this frame.
constexpr int kOrderIntegrate = 10;
constexpr int kOrderPresent = 10;
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
                            const cvc::physics::body_state &start) {
  participant p;
  p.local = local;
  p.sim.set_state(start);
  p.predicted = predicted_avatar(start);
  p.newest = start;
  p.prev = start;
  p.spawn = start;
  _participants[pid] = p;
  // In the world: the server simulates everyone, a client its own avatar; a client's peers are
  // kinematic, held where they are shown.
  if (_world)
    _world->add_avatar(pid, start, _role == holon_role::server || local);
}

void holon::remove_participant(const std::string &pid) {
  _participants.erase(pid);
  if (_world)
    _world->remove_avatar(pid);
}

void holon::set_statics(const std::vector<avatar_static> &statics, std::uint64_t revision) {
  _statics_rev = revision;
  if (_world)
    _world->set_statics(statics);
}

bool holon::has_participant(const std::string &pid) const {
  return _participants.find(pid) != _participants.end();
}

std::vector<std::string> holon::participants() const {
  std::vector<std::string> out;
  out.reserve(_participants.size());
  for (const auto &kv : _participants)
    out.push_back(kv.first);
  return out;
}

void holon::set_intent(const std::string &pid, const avatar_intent &in) {
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  // A client may drive only its OWN (local) avatar; it cannot move a peer.
  if (_role == holon_role::client && !it->second.local)
    return;
  it->second.intent = in;
  it->second.queued.clear();
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
  p.queued.emplace_back(at, in);
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
  if (skew > tol) {
    _clock.seek_tick(target);
    for (auto &kv : _participants) {
      participant &q = kv.second;
      if (q.local) {
        // The logged inputs' ticks are on the old timeline: keep the state only.
        q.predicted = predicted_avatar(q.predicted.state());
        q.log.clear();
        q.acked = false;
      } else {
        // Shift the walk's start by the same jump, so the peer carries on from where
        // it is shown instead of restarting the walk (a visible step back).
        q.newest_arrival_tick = target > now ? q.newest_arrival_tick + (target - now)
                                             : (q.newest_arrival_tick > now - target
                                                    ? q.newest_arrival_tick - (now - target)
                                                    : 0);
      }
    }
  }

  participant &p = it->second;
  if (p.local && _world) {
    world_reconcile(pid, p, auth, server_tick);
  } else if (p.local) {
    const cvc::physics::vec3 before = p.predicted.state().position;
    if (p.predicted.reconcile(auth, server_tick, _clock.fixed_dt(), _cfg.reconcile_tol))
      p.correction = p.correction + (p.predicted.state().position - before);
  } else {
    // A peer. Reject a stale/duplicate/reordered sample (server_tick <= the newest
    // one applied), exactly as predicted_avatar does for the local path: otherwise
    // an out-of-order sample would rewind the interpolation window (prev <- newer,
    // newest <- older) and visibly jerk the peer backward, or a duplicate would
    // collapse the window (prev == newest) and freeze it.
    if (p.have_sample && server_tick <= p.last_sample_tick)
      return;
    if (!p.have_sample) {
      p.newest = auth;
      p.prev = auth;
      p.sample_span = 0;
      p.have_sample = true;
    } else {
      // Walk on from wherever the peer is SHOWN now (normally the old newest; earlier
      // when this sample came before the last walk finished), so a sample that
      // arrives early never makes the peer skip forward.
      const cvc::physics::pose shown = display_pose(pid);
      p.prev = p.newest;
      p.prev.position = shown.position;
      p.prev.rotation = shown.rotation;
      p.newest = auth;
      p.sample_span = std::min(server_tick - p.last_sample_tick, _cfg.max_peer_span_ticks);
    }
    p.last_sample_tick = server_tick;
    p.newest_arrival_tick = _clock.tick();
  }
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
  p.correction = p.correction + (_world->state(pid).position - before);
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
        p.predicted.predict(tick, p.intent, dt);
      }
    }
    // Peers integrate nothing: they are driven by apply_authoritative samples.
  }
}

void holon::on_present(const cvc::clock_driver::clock_frame &f) { _alpha = f.alpha; }

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
  // Peer: walk from prev to newest over the server's own interval between them,
  // timed on this clock from the newest sample's arrival (interpolate_peer clamps
  // the fraction to [0,1), so it never overshoots and holds at newest once done).
  if (p.sample_span == 0)
    return cvc::physics::pose{p.newest.position, p.newest.rotation};
  const std::uint64_t tick = _clock.tick();
  const double elapsed =
      (tick >= p.newest_arrival_tick ? static_cast<double>(tick - p.newest_arrival_tick) : 0.0) +
      _alpha;
  return interpolate_peer(p.prev, p.newest, elapsed / static_cast<double>(p.sample_span));
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

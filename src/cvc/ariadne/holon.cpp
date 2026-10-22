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

#include <cstdint>
#include <cvc/ariadne/holon.h>

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
  _participants[pid] = p;
}

void holon::remove_participant(const std::string &pid) { _participants.erase(pid); }

bool holon::has_participant(const std::string &pid) const {
  return _participants.find(pid) != _participants.end();
}

void holon::set_intent(const std::string &pid, const avatar_intent &in) {
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  // A client may drive only its OWN (local) avatar; it cannot move a peer.
  if (_role == holon_role::client && !it->second.local)
    return;
  it->second.intent = in;
}

void holon::apply_authoritative(const std::string &pid, const cvc::physics::body_state &auth,
                                std::uint64_t server_tick) {
  if (_role != holon_role::client)
    return; // the server IS the authority; nothing to apply
  auto it = _participants.find(pid);
  if (it == _participants.end())
    return;
  participant &p = it->second;
  if (p.local) {
    p.predicted.reconcile(auth, server_tick, _clock.fixed_dt(), _cfg.reconcile_tol);
  } else {
    // A peer: shift the interpolation window (prev <- old newest, newest <- auth).
    if (!p.have_newest) {
      p.newest = auth;
      p.prev = auth;
      p.have_newest = true;
    } else {
      p.prev = p.newest;
      p.newest = auth;
      p.have_prev = true;
    }
  }
}

void holon::on_step(const cvc::clock_driver::clock_frame &f) {
  const double dt = f.fixed_dt;
  const int steps = f.step.steps;
  for (auto &kv : _participants) {
    participant &p = kv.second;
    if (_role == holon_role::server) {
      // Authoritative integration: advance by the latest intent over every quantum
      // that fell due this frame.
      for (int i = 0; i < steps; ++i)
        p.sim.step(p.intent, dt);
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
  if (_role == holon_role::server)
    return p.sim.state();
  if (p.local)
    return p.predicted.state();
  return p.newest;
}

cvc::physics::pose holon::display_pose(const std::string &pid) const {
  const participant &p = _participants.at(pid);
  if (_role == holon_role::server) {
    const cvc::physics::body_state &s = p.sim.state();
    return cvc::physics::pose{s.position, s.rotation};
  }
  if (p.local) {
    const cvc::physics::body_state &s = p.predicted.state();
    return cvc::physics::pose{s.position, s.rotation};
  }
  // Peer: interpolate between the last two authoritative samples at this frame's
  // alpha (interpolate_peer clamps alpha to [0,1), so it never overshoots).
  return interpolate_peer(p.prev, p.newest, _alpha);
}

} // namespace ariadne
} // namespace cvc

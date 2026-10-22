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

#include <cmath>
#include <cvc/ariadne/avatar_sim.h>

namespace cvc {
namespace ariadne {

void avatar_sim::step(const avatar_intent &in, double dt) {
  // Linear kinematic integration: the body moves at the requested velocity. Pure
  // double arithmetic (vec3 + and *), no trig, so it is bit-reproducible.
  _state.linear_velocity = in.move;
  _state.position = _state.position + in.move * dt;
  _state.active = true;
}

void predicted_avatar::predict(std::uint64_t tick, const avatar_intent &in, double dt) {
  _sim.step(in, dt);
  logged e;
  e.tick = tick;
  e.in = in;
  e.predicted = _sim.state();
  _log.push_back(e);
}

bool predicted_avatar::reconcile(const cvc::physics::body_state &authoritative,
                                 std::uint64_t server_tick, double dt, double tol) {
  // What did we predict AT server_tick? (null if we never predicted that tick --
  // e.g. the server is ahead of us, or that input was already acknowledged.)
  const logged *at = nullptr;
  for (const auto &e : _log)
    if (e.tick == server_tick) {
      at = &e;
      break;
    }

  const double tol2 = tol * tol;
  const auto within_tol = [&](const cvc::physics::body_state &a,
                              const cvc::physics::body_state &b) {
    const cvc::physics::vec3 d = a.position - b.position;
    return cvc::physics::dot(d, d) <= tol2;
  };

  if (at != nullptr && within_tol(at->predicted, authoritative)) {
    // Prediction held: the server agrees with what we had at server_tick, so the
    // inputs after it (which we already applied) still stand. Just drop the inputs
    // the server has now acknowledged; keep predicting forward from our state.
    while (!_log.empty() && _log.front().tick <= server_tick)
      _log.pop_front();
    return false;
  }

  // Diverged (a server override, a dropped/extra step, or we have no record of
  // server_tick): adopt the authoritative state, then re-apply the inputs we
  // issued AFTER server_tick so the player's in-flight motion is preserved on top
  // of the correction rather than lost.
  std::deque<logged> replay;
  for (const auto &e : _log)
    if (e.tick > server_tick)
      replay.push_back(e);

  _sim.set_state(authoritative);
  _log.clear();
  for (logged &e : replay) {
    _sim.step(e.in, dt);
    e.predicted = _sim.state();
    _log.push_back(e);
  }
  return true;
}

cvc::physics::pose interpolate_peer(const cvc::physics::body_state &prev,
                                    const cvc::physics::body_state &newest, double alpha) {
  if (!(alpha >= 0.0)) // also catches NaN
    alpha = 0.0;
  if (alpha >= 1.0)
    alpha = std::nextafter(1.0, 0.0); // never reach or pass the newest sample
  const cvc::physics::pose a{prev.position, prev.rotation};
  const cvc::physics::pose b{newest.position, newest.rotation};
  return cvc::physics::interpolate(a, b, alpha);
}

} // namespace ariadne
} // namespace cvc

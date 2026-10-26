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

#ifndef __CVC_ARIADNE_AVATAR_SIM_H__
#define __CVC_ARIADNE_AVATAR_SIM_H__

#include <cstdint>
#include <cvc/core/namespace.h>
#include <cvc/physics/body.h>         // cvc::physics::body_state
#include <cvc/physics/pose_tracker.h> // cvc::physics::pose / interpolate
#include <cvc/physics/types.h>        // cvc::physics::vec3
#include <deque>

namespace cvc {
namespace ariadne {

// ----------------
// cvc::ariadne — headless avatar prediction / reconciliation (H1 slice 2)
// ----------------
// The one genuinely new mechanism H1 needs on top of the clock + replication it
// already has: a participant's avatar is server-authoritative, but a client must
// move its OWN avatar without waiting a round-trip, so it PREDICTS locally and
// RECONCILES when the server's authoritative state arrives; peers it never
// predicts, only interpolates between their last two authoritative samples.
//
// This is deliberately headless and engine-free: pure double arithmetic over a
// fixed timestep, no cvc::physics::world (Jolt), no GL, no state tree, no
// transport. That isolates the loop's correctness -- which is what is subtle --
// from the plumbing (the holon owns the clock_driver and the state wiring; that is
// slice 3+). The integrator is a linear stub; slice 3 swaps it for
// physics::world save/restore stepping behind the same predict/reconcile shape.

// The per-tick input that drives an avatar: a desired world-space linear velocity and
// the direction the avatar faces. The position integration is purely linear (no trig),
// so it stays bit-reproducible across platforms; the facing only sets the rotation,
// which reconcile does not compare (its tolerance is positional).
struct avatar_intent {
  cvc::physics::vec3 move; // desired linear velocity, world metres/second
  double yaw = 0.0;        // facing: radians about +Z from +X (Z-up), chosen by the user
};

// A deterministic, headless kinematic avatar: integrate an avatar_intent over
// fixed-dt quanta into a cvc::physics::body_state. position += velocity*dt and
// velocity = move; the rotation is the intent's facing (yaw about +Z). The position
// is pure double arithmetic, so the same intents from the same start at the same dt
// reproduce it BIT-FOR-BIT -- the property prediction and reconciliation rely on.
class avatar_sim {
public:
  avatar_sim() = default;
  explicit avatar_sim(const cvc::physics::body_state &start) : _state(start) {}

  const cvc::physics::body_state &state() const { return _state; }
  void set_state(const cvc::physics::body_state &s) { _state = s; }

  // Advance exactly one fixed quantum from `in`.
  void step(const avatar_intent &in, double dt);

private:
  cvc::physics::body_state _state;
};

// Client-side prediction + server reconciliation for ONE locally-controlled
// avatar on a fixed-dt clock. The client applies its own intents immediately
// (predict), keeping a log of inputs the server has not yet acknowledged; when an
// authoritative state for a past tick arrives it reconciles: if the state the
// client predicted at that tick already matches the authoritative one within `tol`
// metres, it keeps its prediction (no visible correction); otherwise it snaps to
// the authoritative state and REPLAYS the inputs issued after that tick, so the
// current prediction is corrected without discarding the player's in-flight input.
//
// Deterministic: with matching inputs and dt the replay reproduces the prediction
// exactly, so a correct prediction reconciles to a bit-identical state and costs
// nothing; only a genuine divergence (a server override, a dropped step) snaps.
class predicted_avatar {
public:
  predicted_avatar() = default;
  explicit predicted_avatar(const cvc::physics::body_state &start) : _sim(start) {}

  // Apply the local player's intent for `tick` and advance the prediction. Ticks
  // are expected monotonically increasing (the clock's quanta). The log keeps at
  // most kMaxLog unacknowledged entries (the oldest drop): with no acks for that
  // long the link is gone, and the next ack snaps + replays whatever is left.
  void predict(std::uint64_t tick, const avatar_intent &in, double dt);
  static constexpr std::size_t kMaxLog = 2400; // 20 s at the default 120 Hz

  // Reconcile against an authoritative state valid as of `server_tick`. Returns
  // true if a hard correction (snap + replay) was applied, false if the prediction
  // already held OR the ack was ignored. A `server_tick` at or below the highest
  // one already reconciled is a no-op (returns false, touches nothing): a
  // duplicate or reordered/stale ack -- which real datagram transports produce --
  // must not snap the avatar back to an old authoritative state and discard
  // progress, so only a strictly newer ack is acted on.
  bool reconcile(const cvc::physics::body_state &authoritative, std::uint64_t server_tick,
                 double dt, double tol);

  const cvc::physics::body_state &state() const { return _sim.state(); }
  // Inputs issued but not yet acknowledged by the server (awaiting replay).
  std::size_t pending() const { return _log.size(); }

private:
  struct logged {
    std::uint64_t tick = 0;
    avatar_intent in;
    cvc::physics::body_state predicted; // state AFTER applying `in` at `tick`
  };
  avatar_sim _sim;
  std::deque<logged> _log;
  std::uint64_t _last_acked = 0; // highest server_tick reconciled; guards non-monotonic acks
  bool _acked = false;           // whether _last_acked is meaningful yet
};

// Interpolate a NON-local peer avatar between its last two authoritative samples,
// NEVER extrapolating: alpha is clamped to [0, 1), so the result never passes the
// newest sample (a one-step display lag is correct; extrapolation would overshoot
// on a late sample). Thin wrapper over cvc::physics::interpolate on the pose
// (position + rotation) of each body_state.
cvc::physics::pose interpolate_peer(const cvc::physics::body_state &prev,
                                    const cvc::physics::body_state &newest, double alpha);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_AVATAR_SIM_H__

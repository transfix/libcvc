/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::ariadne avatar prediction / reconciliation (H1 slice 2). The
// properties worth pinning are the ones a real netcode loop lives or dies on:
// deterministic integration (so replay reproduces prediction), a perfect
// prediction that reconciles with NO correction, a server override that snaps the
// client and REPLAYS its in-flight inputs rather than dropping them, and a peer
// interpolation that never extrapolates past the newest sample.
//
// Every comparison here is lib-step vs lib-step (both the server reference and the
// client go through cvc::ariadne::avatar_sim::step), so bit-equality holds without
// -ffp-contract=off: there is no test-side recomputation of the lib's inline math.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::avatar_sim;
using cvc::ariadne::interpolate_peer;
using cvc::ariadne::predicted_avatar;
using cvc::physics::body_state;
using cvc::physics::vec3;

namespace {
std::uint64_t bits(double d) {
  std::uint64_t u = 0;
  std::memcpy(&u, &d, sizeof(u));
  return u;
}
// Bit-for-bit equality of two positions (exact; catches a 1-ulp replay drift).
void expect_pos_bit_eq(const vec3 &a, const vec3 &b) {
  EXPECT_EQ(bits(a.x), bits(b.x));
  EXPECT_EQ(bits(a.y), bits(b.y));
  EXPECT_EQ(bits(a.z), bits(b.z));
}
constexpr double k120 = 1.0 / 120.0;

// A varied, non-trivial intent for tick t (distinct components, not all equal).
avatar_intent intent_at(std::uint64_t t) {
  const double f = static_cast<double>(t);
  return avatar_intent{vec3{0.1 * f, -0.05 * f, 0.2}};
}
} // namespace

// ── determinism ─────────────────────────────────────────────────────────────

TEST(AvatarSimTest, StepIsBitReproducible) {
  avatar_sim a, b;
  for (std::uint64_t t = 1; t <= 200; ++t) {
    const avatar_intent in = intent_at(t);
    a.step(in, k120);
    b.step(in, k120);
  }
  expect_pos_bit_eq(a.state().position, b.state().position);
  expect_pos_bit_eq(a.state().linear_velocity, b.state().linear_velocity);
}

// ── perfect prediction costs nothing ────────────────────────────────────────

TEST(AvatarSimTest, PerfectPredictionReconcilesWithNoCorrection) {
  avatar_sim server;       // authoritative
  predicted_avatar client; // predicts the same inputs
  for (std::uint64_t t = 1; t <= 50; ++t) {
    const avatar_intent in = intent_at(t);
    client.predict(t, in, k120);
    server.step(in, k120);
    const bool corrected = client.reconcile(server.state(), t, k120, 1e-9);
    EXPECT_FALSE(corrected) << "a matching prediction must not snap (t=" << t << ")";
    expect_pos_bit_eq(client.state().position, server.state().position);
  }
  EXPECT_EQ(client.pending(), 0u) << "all inputs acknowledged";
}

// Client runs AHEAD of the server (prediction under latency): it predicts several
// ticks, the server acknowledges older ticks with matching inputs. No correction,
// and the unacknowledged inputs stay pending.
TEST(AvatarSimTest, PredictionAheadOfServerKeepsPendingInputs) {
  avatar_sim server;
  predicted_avatar client;
  std::vector<avatar_intent> issued;
  for (std::uint64_t t = 1; t <= 5; ++t) {
    const avatar_intent in = intent_at(t);
    issued.push_back(in);
    client.predict(t, in, k120); // client is now 5 ticks ahead
  }
  // Server has only integrated up to tick 2 so far.
  for (std::uint64_t t = 1; t <= 2; ++t)
    server.step(issued[t - 1], k120);
  const bool corrected = client.reconcile(server.state(), 2, k120, 1e-9);
  EXPECT_FALSE(corrected);
  EXPECT_EQ(client.pending(), 3u) << "inputs for ticks 3,4,5 remain unacknowledged";
}

// ── divergence snaps + replays, preserving in-flight input ──────────────────

TEST(AvatarSimTest, ServerOverrideSnapsAndReplaysInFlightInputs) {
  body_state start; // origin
  predicted_avatar client(start);
  const avatar_intent mv{vec3{1.0, 0.0, 0.0}}; // moving +x at 1 m/s
  for (std::uint64_t t = 1; t <= 5; ++t)
    client.predict(t, mv, k120); // client predicts x = 5*k120

  // The server overrides the avatar at tick 3 (a teleport the client could not
  // predict), with the same ongoing velocity.
  body_state auth;
  auth.position = vec3{100.0, 0.0, 0.0};
  auth.linear_velocity = mv.move;
  const bool corrected = client.reconcile(auth, /*server_tick=*/3, k120, 0.01);
  EXPECT_TRUE(corrected) << "a divergent authoritative state must snap";

  // Expected: adopt auth@3, then REPLAY the in-flight inputs for ticks 4 and 5.
  avatar_sim ref(auth);
  ref.step(mv, k120);
  ref.step(mv, k120);
  expect_pos_bit_eq(client.state().position, ref.state().position);
  EXPECT_EQ(client.pending(), 2u) << "ticks 4,5 are still in flight (not yet acked)";
  // The correction is real: the client moved from ~0.04 m to ~100 m.
  EXPECT_GT(client.state().position.x, 99.0);
}

// A later matching ack after a correction settles cleanly (no second snap).
TEST(AvatarSimTest, AckAfterCorrectionDoesNotReSnap) {
  predicted_avatar client;
  const avatar_intent mv{vec3{2.0, 0.0, 0.0}};
  for (std::uint64_t t = 1; t <= 4; ++t)
    client.predict(t, mv, k120);
  body_state auth;
  auth.position = vec3{50.0, 0.0, 0.0};
  auth.linear_velocity = mv.move;
  ASSERT_TRUE(client.reconcile(auth, 2, k120, 0.01)); // snap at 2, replay 3,4
  const body_state after_snap = client.state();

  // Now the server, having adopted the same post-snap trajectory, acks tick 3
  // with the state the client now holds for tick 3 (auth@2 + one replay step).
  avatar_sim server(auth);
  server.step(mv, k120); // server's tick-3 state
  const bool corrected = client.reconcile(server.state(), 3, k120, 0.01);
  EXPECT_FALSE(corrected) << "a matching ack after a correction must not re-snap";
  EXPECT_EQ(client.pending(), 1u) << "only tick 4 remains";
  // State advanced forward, never jumped backward.
  EXPECT_GE(client.state().position.x, after_snap.position.x);
}

// ── peer interpolation never extrapolates ───────────────────────────────────

TEST(AvatarSimTest, PeerInterpolationNeverOvershoots) {
  body_state prev, newest;
  prev.position = vec3{0.0, 0.0, 0.0};
  prev.rotation = cvc::physics::quat{1.0, 0.0, 0.0, 0.0};
  newest.position = vec3{10.0, 0.0, 0.0};
  newest.rotation = cvc::physics::quat{1.0, 0.0, 0.0, 0.0};

  EXPECT_DOUBLE_EQ(interpolate_peer(prev, newest, 0.0).position.x, 0.0); // alpha 0 = prev exactly
  EXPECT_NEAR(interpolate_peer(prev, newest, 0.5).position.x, 5.0, 1e-12);

  // alpha >= 1 is clamped below 1: the result approaches but never reaches or
  // passes the newest sample (no extrapolation / overshoot).
  EXPECT_LT(interpolate_peer(prev, newest, 1.0).position.x, 10.0);
  EXPECT_LT(interpolate_peer(prev, newest, 99.0).position.x, 10.0);
  EXPECT_GT(interpolate_peer(prev, newest, 99.0).position.x, 9.0);

  // Negative / NaN alpha clamps to prev (no behind-the-start extrapolation).
  EXPECT_DOUBLE_EQ(interpolate_peer(prev, newest, -5.0).position.x, 0.0);
  EXPECT_DOUBLE_EQ(interpolate_peer(prev, newest, std::nan("")).position.x, 0.0);
}

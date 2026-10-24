/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::ariadne::holon (H1 slice 3): the server-authoritative,
// client-replicable clock + avatar-pose bundle. The properties worth pinning:
// each holon owns its OWN clock (two never share the app clock); a server holon
// and a client holon stepped in-process reproduce slice 2's prediction /
// reconciliation through the holon API (perfect prediction converges bit-for-bit;
// a server override snaps the client); a client never drives a peer; and a peer is
// interpolated through the holon's frame alpha, never overshooting.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/holon.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::avatar_sim;
using cvc::ariadne::holon;
using cvc::ariadne::holon_config;
using cvc::ariadne::holon_role;
using cvc::physics::body_state;
using cvc::physics::vec3;

namespace {
std::uint64_t bits(double d) {
  std::uint64_t u = 0;
  std::memcpy(&u, &d, sizeof(u));
  return u;
}
void expect_pos_bit_eq(const vec3 &a, const vec3 &b) {
  EXPECT_EQ(bits(a.x), bits(b.x));
  EXPECT_EQ(bits(a.y), bits(b.y));
  EXPECT_EQ(bits(a.z), bits(b.z));
}
} // namespace

// ── each holon owns its own clock ───────────────────────────────────────────

TEST(HolonTest, OwnsItsOwnClockNotTheAppClock) {
  holon a(holon_role::server);
  holon b(holon_role::server);
  EXPECT_NE(&a.clock(), &b.clock()) << "each holon must own a distinct world_clock";
  const double dt = a.clock().fixed_dt();
  a.advance(dt); // advance a by one quantum
  EXPECT_EQ(a.tick(), 1u);
  EXPECT_EQ(b.tick(), 0u) << "advancing one holon must not tick another";
}

// ── server + client: perfect prediction converges bit-for-bit ───────────────

TEST(HolonTest, ServerClientPerfectPredictionConverges) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");                 // authoritative on the server
  client.add_participant("A", /*local=*/true); // the client controls A, predicted
  const double dt = server.clock().fixed_dt();
  const avatar_intent mv{vec3{2.0, 0.0, 0.0}};

  for (int frame = 1; frame <= 10; ++frame) {
    server.set_intent("A", mv);
    client.set_intent("A", mv); // the client predicts the same input it sends
    server.advance(dt);
    client.advance(dt);
    // Replicate the authoritative state to the client and reconcile.
    client.apply_authoritative("A", server.state("A"), server.tick());
    // A matching prediction holds (no snap) and is bit-identical to authoritative.
    expect_pos_bit_eq(client.state("A").position, server.state("A").position);
  }
  EXPECT_GT(server.state("A").position.x, 0.0); // actually moved
}

// ── server override snaps the client onto the authoritative trajectory ──────

TEST(HolonTest, ServerOverrideReconcilesOnClient) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");
  client.add_participant("A", /*local=*/true);
  const double dt = server.clock().fixed_dt();
  const avatar_intent mv_client{vec3{2.0, 0.0, 0.0}}; // client thinks A moves +x
  const avatar_intent mv_server{vec3{0.0, 2.0, 0.0}}; // server says A moves +y

  for (int frame = 1; frame <= 10; ++frame) {
    client.set_intent("A", mv_client);
    server.set_intent("A", mv_server);
    client.advance(dt);
    server.advance(dt);
  }
  // The client predicted +x; the authoritative state is +y. Reconcile snaps it.
  client.apply_authoritative("A", server.state("A"), server.tick());
  expect_pos_bit_eq(client.state("A").position, server.state("A").position);
  EXPECT_GT(client.state("A").position.y, 0.1) << "client adopted the server's +y motion";
  EXPECT_NEAR(client.state("A").position.x, 0.0, 1e-12)
      << "the client's wrong +x was corrected away";
}

// A server correction for a PAST tick snaps the client and replays the inputs it
// issued after that tick (slice 2's replay, driven through the holon). This also
// validates on_step's tick numbering: an off-by-one would replay the wrong suffix
// and the result would not match the reference.
TEST(HolonTest, ClientReplaysInFlightInputsOnServerCorrection) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  const double dt = client.clock().fixed_dt();
  const avatar_intent mv{vec3{1.0, 0.0, 0.0}}; // client predicts A moving +x
  for (int frame = 1; frame <= 5; ++frame) {   // predicts ticks 1..5
    client.set_intent("A", mv);
    client.advance(dt);
  }
  // Authoritative state for tick 3 diverges (the server teleported A to +y). The
  // client snaps to it and replays its in-flight inputs for ticks 4 and 5.
  body_state auth3;
  auth3.position = vec3{0.0, 100.0, 0.0};
  auth3.linear_velocity = mv.move;
  client.apply_authoritative("A", auth3, /*server_tick=*/3);

  avatar_sim ref(auth3); // auth@3 + replay of the two post-tick-3 inputs
  ref.step(mv, dt);
  ref.step(mv, dt);
  expect_pos_bit_eq(client.state("A").position, ref.state().position);
  EXPECT_GT(client.state("A").position.y, 99.0) << "adopted the server's +y teleport";
  EXPECT_GT(client.state("A").position.x, 0.0) << "replayed the in-flight +x inputs on top";
}

// ── a peer is sample-driven, not intent-driven ──────────────────────────────

TEST(HolonTest, PeerIsSampleDrivenNotIntentDriven) {
  holon client(holon_role::client);
  client.add_participant("peer", /*local=*/false);
  // set_intent on a non-local avatar is ignored (a client cannot drive a peer);
  // a peer only moves via authoritative samples, so with none it stays at origin.
  client.set_intent("peer", avatar_intent{vec3{9.0, 9.0, 9.0}});
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 5; ++i)
    client.advance(dt);
  EXPECT_DOUBLE_EQ(client.state("peer").position.x, 0.0);
}

// A stale / duplicate / reordered peer sample (server_tick at or below the newest
// applied) is ignored -- the interpolation window must not rewind or collapse.
TEST(HolonTest, PeerIgnoresStaleOrDuplicateSample) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  body_state s4, s5;
  s4.position = vec3{4.0, 0.0, 0.0};
  s5.position = vec3{5.0, 0.0, 0.0};
  client.apply_authoritative("B", s4, 4);
  client.apply_authoritative("B", s5, 5); // window is [s4 -> s5]
  // A reordered older sample (tick 4 again, different value) must be dropped.
  body_state stale;
  stale.position = vec3{-100.0, 0.0, 0.0};
  client.apply_authoritative("B", stale, 4);
  const double dt = client.clock().fixed_dt();
  client.advance(0.5 * dt); // alpha ~0.5 over the UNCHANGED window [4 -> 5]
  EXPECT_NEAR(client.display_pose("B").position.x, 4.5, 1e-9)
      << "stale sample must not rewind the interpolation window";
}

// ── a peer is interpolated through the holon's frame alpha, no overshoot ────

TEST(HolonTest, PeerInterpolatesThroughFrameAlpha) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();

  body_state s1;
  s1.position = vec3{0.0, 0.0, 0.0};
  body_state s2;
  s2.position = vec3{10.0, 0.0, 0.0};
  client.apply_authoritative("B", s1, 1);
  client.apply_authoritative("B", s2, 2); // window is now [s1, s2]

  // Advance half a quantum: no step falls due, but the frame alpha becomes ~0.5.
  client.advance(0.5 * dt);
  const double x_half = client.display_pose("B").position.x;
  EXPECT_GT(x_half, 0.0);
  EXPECT_LT(x_half, 10.0) << "interpolated strictly between the two samples";
  EXPECT_NEAR(x_half, 5.0, 1e-9);

  // A near-full-quantum frame approaches but never reaches/passes the newest sample.
  holon c2(holon_role::client);
  c2.add_participant("B", false);
  c2.apply_authoritative("B", s1, 1);
  c2.apply_authoritative("B", s2, 2);
  c2.advance(0.999 * dt);
  const double x_near = c2.display_pose("B").position.x;
  EXPECT_LT(x_near, 10.0) << "peer interpolation never overshoots newest";
  EXPECT_GT(x_near, 9.0) << "and it did advance most of the way (not frozen)";
}

// ── a peer walks smoothly across SPARSE samples (the live ~5 Hz relay) ──────
// Samples arrive every 24 quanta (5 Hz on a 120 Hz clock). The peer must walk from
// one to the next over those 24 quanta: monotonic, never past the newest sample,
// and actually moving (not held at the sample or flicking between the two).

TEST(HolonTest, PeerWalksAcrossSparseSamples) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  constexpr int kSpan = 24;

  double last_x = -1.0;
  int moved_frames = 0;
  for (int k = 0; k <= 4; ++k) {
    body_state s;
    s.position = vec3{static_cast<double>(k), 0.0, 0.0}; // 1 m per sample
    client.apply_authoritative("B", s, 100 + static_cast<std::uint64_t>(k * kSpan));
    for (int f = 0; f < kSpan; ++f) {
      client.advance(dt);
      const double x = client.display_pose("B").position.x;
      EXPECT_GE(x, last_x) << "sample " << k << " frame " << f << ": the peer never steps back";
      EXPECT_LE(x, static_cast<double>(k)) << "never past the newest sample";
      if (x > last_x)
        ++moved_frames;
      last_x = x;
    }
  }
  EXPECT_GT(last_x, 3.9) << "it walked (nearly) all the way to the last sample";
  EXPECT_GT(moved_frames, 4 * (kSpan - 2)) << "it moved on (nearly) every frame of each walk";
}

// ── a client replica counts the server's ticks ──────────────────────────────

TEST(HolonTest, ClientRealignsItsClockWhenSkewed) {
  holon_config cfg;
  cfg.max_skew_ticks = 60;
  holon client(holon_role::client, cfg);
  client.add_participant("A", /*local=*/true);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 500; ++i) // the replica has run on its own for a while
    client.advance(dt);
  ASSERT_EQ(client.tick(), 500u);

  // Joining a server whose clock is elsewhere: the first sample aligns to its tick.
  body_state s;
  s.position = vec3{3.0, 0.0, 0.0};
  client.apply_authoritative("A", s, 12);
  EXPECT_EQ(client.tick(), 12u);
  EXPECT_DOUBLE_EQ(client.state("A").position.x, 3.0) << "the local avatar adopts the sample";

  // Within max_skew_ticks the clocks are left alone (they run at the same rate).
  body_state b0, b1;
  b1.position = vec3{1.0, 0.0, 0.0};
  client.apply_authoritative("B", b0, 12);
  client.apply_authoritative("B", b1, 36); // B walks 0 -> 1 over 24 ticks
  for (int i = 0; i < 10; ++i)
    client.advance(dt);
  client.apply_authoritative("A", s, 40); // 18 ticks away: no re-alignment
  EXPECT_EQ(client.tick(), 22u);
  const double b_mid = client.display_pose("B").position.x;
  EXPECT_GT(b_mid, 0.3);
  EXPECT_LT(b_mid, 0.6);

  // Past it (a stalled frame loop), the clock re-aligns -- and the peer's walk carries on
  // from where it was shown rather than restarting.
  client.apply_authoritative("A", s, 400);
  EXPECT_EQ(client.tick(), 400u);
  EXPECT_NEAR(client.display_pose("B").position.x, b_mid, 1e-12);
}

// ── a reconcile jump is reported once, for the renderer to ease out ─────────

TEST(HolonTest, TakeCorrectionReportsReconcileJumpsOnce) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 3; ++i) {
    client.set_intent("A", avatar_intent{vec3{1.0, 0.0, 0.0}});
    client.advance(dt);
  }
  const vec3 before = client.state("A").position;

  // The prediction holds: no correction.
  body_state held;
  held.position = vec3{2.0 * dt, 0.0, 0.0}; // what A predicted at tick 2
  held.linear_velocity = vec3{1.0, 0.0, 0.0};
  client.apply_authoritative("A", held, 2);
  const vec3 none = client.take_correction("A");
  EXPECT_DOUBLE_EQ(none.x, 0.0);
  EXPECT_DOUBLE_EQ(none.y, 0.0);

  // The server overrides: the jump is reported, exactly, once.
  body_state moved;
  moved.position = vec3{0.0, 5.0, 0.0};
  client.apply_authoritative("A", moved, 3);
  const vec3 after = client.state("A").position;
  const vec3 c = client.take_correction("A");
  EXPECT_DOUBLE_EQ(c.x, after.x - before.x);
  EXPECT_DOUBLE_EQ(c.y, after.y - before.y);
  EXPECT_DOUBLE_EQ(client.take_correction("A").y, 0.0) << "taken, so cleared";
  EXPECT_DOUBLE_EQ(client.take_correction("B").y, 0.0) << "a peer has no corrections";
  EXPECT_DOUBLE_EQ(client.take_correction("nobody").y, 0.0);
}

// ── stamped intents: the server applies each at the tick the client did ─────

TEST(HolonTest, ServerAppliesAStampedIntentAtItsTick) {
  holon server(holon_role::server);
  server.add_participant("A");
  const double dt = server.clock().fixed_dt();
  const auto frames = [&](int n) {
    for (int i = 0; i < n; ++i)
      server.advance(dt);
  };
  frames(10);
  ASSERT_EQ(server.tick(), 10u);

  // Stamped for tick 15: ticks 11-14 still stand, 15-20 walk at 2 m/s.
  server.queue_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}}, 15);
  server.queue_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}}, 15); // re-read: a no-op
  frames(10);
  ASSERT_EQ(server.tick(), 20u);
  avatar_sim ref;
  for (int i = 0; i < 6; ++i)
    ref.step(avatar_intent{vec3{2.0, 0.0, 0.0}}, dt);
  expect_pos_bit_eq(server.state("A").position, ref.state().position);

  // A late stamp (its tick already integrated) takes effect at the next tick.
  server.queue_intent("A", avatar_intent{vec3{0.0, 0.0, 0.0}}, 12);
  server.advance(dt);
  expect_pos_bit_eq(server.state("A").position, ref.state().position);

  // A stamp far beyond the allowed lead (a client whose clock is off) also applies at once.
  server.queue_intent("A", avatar_intent{vec3{0.0, 3.0, 0.0}}, 1'000'000);
  server.advance(dt);
  EXPECT_DOUBLE_EQ(server.state("A").position.y, 3.0 * dt);

  // set_intent (untimed) drops anything still queued.
  server.queue_intent("A", avatar_intent{vec3{9.0, 0.0, 0.0}}, server.tick() + 5);
  server.set_intent("A", avatar_intent{vec3{0.0, 0.0, 0.0}});
  const vec3 before = server.state("A").position;
  frames(10);
  expect_pos_bit_eq(server.state("A").position, before);
}

// A client that runs AHEAD of the server by the round trip has its stamped intents reach the
// server before their tick, so the server's poses agree with its prediction exactly: across a
// walk with several key changes, reconcile corrects nothing. The same exchange with no lead (the
// intents arrive late) needs corrections -- the latency the lead removes.
namespace {
double total_correction(std::uint64_t lead) {
  constexpr int kDelay = 6; // frames (= ticks) each way
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");
  client.add_participant("A", /*local=*/true);
  client.set_lead_ticks(lead);
  const double dt = server.clock().fixed_dt();
  struct intent_msg {
    int at;
    avatar_intent in;
    std::uint64_t stamp;
  };
  struct sample_msg {
    int at;
    body_state s;
    std::uint64_t tick;
  };
  std::vector<intent_msg> up;
  std::vector<sample_msg> down;
  double corrected = 0.0;
  avatar_intent cur;
  for (int f = 0; f < 400; ++f) {
    // Keys: walk +x, strafe, stop, walk -y, stop.
    avatar_intent want = cur;
    if (f == 60)
      want.move = vec3{2.0, 0.0, 0.0};
    if (f == 110)
      want.move = vec3{0.0, 2.0, 0.0};
    if (f == 150)
      want.move = vec3{0.0, 0.0, 0.0};
    if (f == 200)
      want.move = vec3{0.0, -2.0, 0.0};
    if (f == 260)
      want.move = vec3{0.0, 0.0, 0.0};
    if (want.move.x != cur.move.x || want.move.y != cur.move.y) {
      cur = want;
      client.set_intent("A", cur);
      up.push_back({f + kDelay, cur, client.tick() + 1});
    }
    client.advance(dt);
    for (auto it = up.begin(); it != up.end();)
      if (it->at <= f) {
        server.queue_intent("A", it->in, it->stamp);
        it = up.erase(it);
      } else {
        ++it;
      }
    server.advance(dt);
    down.push_back({f + kDelay, server.state("A"), server.tick()});
    for (auto it = down.begin(); it != down.end();)
      if (it->at <= f) {
        client.apply_authoritative("A", it->s, it->tick);
        it = down.erase(it);
      } else {
        ++it;
      }
    const vec3 c = client.take_correction("A");
    if (f >= 60) // after the first sample aligned the clocks
      corrected += std::fabs(c.x) + std::fabs(c.y) + std::fabs(c.z);
  }
  return corrected;
}
} // namespace

TEST(HolonTest, ALeadingClientNeedsNoCorrections) {
  EXPECT_EQ(total_correction(/*lead=*/2 * 6 + 2), 0.0);
  EXPECT_GT(total_correction(/*lead=*/0), 0.1) << "without the lead the intents land late";
}

TEST(HolonTest, ClientAlignsAheadOfTheServerByItsLead) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  client.set_lead_ticks(14);
  body_state s;
  client.apply_authoritative("A", s, 100);
  EXPECT_EQ(client.tick(), 114u);
  // Within the slack it is left alone; falling further behind its target re-aligns.
  client.apply_authoritative("A", s, 104); // target 118, 4 behind
  EXPECT_EQ(client.tick(), 114u);
  client.apply_authoritative("A", s, 120); // target 134, 20 behind
  EXPECT_EQ(client.tick(), 134u);
}

TEST(HolonTest, PeerSampleSpanIsCapped) {
  holon_config cfg;
  cfg.max_peer_span_ticks = 30;
  holon client(holon_role::client, cfg);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  body_state s0, s1;
  s1.position = vec3{1.0, 0.0, 0.0};
  client.apply_authoritative("B", s0, 10);
  client.apply_authoritative("B", s1, 250); // after a long still spell (a heartbeat gap)
  for (int i = 0; i < 31; ++i)
    client.advance(dt);
  EXPECT_GT(client.display_pose("B").position.x, 0.99) << "walked in over 30 ticks, not 240";
}

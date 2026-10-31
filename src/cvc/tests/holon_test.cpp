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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/holon.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <gtest/gtest.h>
#include <limits>
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
// applied) is ignored -- the path must not rewind.
TEST(HolonTest, PeerIgnoresStaleOrDuplicateSample) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  body_state s4, s5;
  s4.position = vec3{4.0, 0.0, 0.0};
  s5.position = vec3{5.0, 0.0, 0.0};
  client.apply_authoritative("B", s4, 4);
  client.apply_authoritative("B", s5, 10);
  // A reordered older sample (tick 4 again, different value) must be dropped.
  body_state stale;
  stale.position = vec3{-100.0, 0.0, 0.0};
  client.apply_authoritative("B", stale, 4);
  client.apply_authoritative("B", stale, 10);
  double last = 4.0;
  for (int i = 0; i < 30; ++i) {
    client.advance(dt);
    const double x = client.display_pose("B").position.x;
    EXPECT_GE(x, last - 1e-12) << "frame " << i << ": a stale sample must not rewind the path";
    EXPECT_LE(x, 5.0 + 1e-12);
    last = x;
  }
  EXPECT_NEAR(last, 5.0, 1e-9) << "it walked to the newest sample and holds there";
}

namespace {
// A peer's samples as a relay sends them: the true path at every `every` ticks, each with its
// velocity, arriving `latency` ticks later plus `jitter` ticks drawn uniformly (a seeded LCG, so
// the run is the same every time), one in `drop` lost (0: none). The client runs frames of
// `frame_ticks` of wall time, applies the samples that have arrived, and records the peer.
struct peer_run {
  std::vector<double> t;     // the wall tick of each frame
  std::vector<vec3> shown;   // where the peer was shown
  std::vector<double> delay; // the server tick shown, behind the newest truth
};
template <class Path>
peer_run run_peer(holon &client, Path path, int every, double latency, double jitter, int drop,
                  double frame_ticks, int frames, std::uint32_t seed = 12345u) {
  struct pending {
    double arrive;
    std::uint64_t tick;
  };
  std::vector<pending> inflight;
  std::uint32_t lcg = seed;
  const auto rnd = [&lcg] {
    lcg = lcg * 1664525u + 1013904223u;
    return static_cast<double>(lcg >> 8) / static_cast<double>(1u << 24);
  };
  const double dt = client.clock().fixed_dt();
  peer_run out;
  double wall = 0.0;
  std::uint64_t next_sample = 100;
  const double t0 = 100.0; // the server's tick when the wall clock reads 0
  for (int f = 0; f < frames; ++f) {
    wall += frame_ticks;
    // The server's samples made by now, in flight; those that have arrived are applied in the
    // order they arrive (a late one after a newer is stale, and dropped by the holon).
    while (static_cast<double>(next_sample) <= t0 + wall) {
      const bool lost = drop > 0 && static_cast<int>(rnd() * drop) == 0;
      if (!lost)
        inflight.push_back(
            pending{static_cast<double>(next_sample) - t0 + latency + jitter * rnd(), next_sample});
      next_sample += static_cast<std::uint64_t>(every);
    }
    std::sort(inflight.begin(), inflight.end(),
              [](const pending &a, const pending &b) { return a.arrive < b.arrive; });
    while (!inflight.empty() && inflight.front().arrive <= wall) {
      const std::uint64_t tick = inflight.front().tick;
      inflight.erase(inflight.begin());
      client.apply_authoritative("B", path(static_cast<double>(tick) * dt), tick);
    }
    client.advance(frame_ticks * dt);
    out.t.push_back(wall);
    out.shown.push_back(client.display_pose("B").position);
  }
  return out;
}

// Walking a 4 m circle at 3 m/s, starting at rest and accelerating at 6 m/s^2.
body_state circle(double t) {
  constexpr double r = 4.0, top = 3.0, accel = 6.0;
  const double t_up = top / accel;
  const double arc = t < t_up ? 0.5 * accel * t * t : 0.5 * accel * t_up * t_up + top * (t - t_up);
  const double speed = t < t_up ? accel * t : top;
  const double a = arc / r;
  body_state s;
  s.position = vec3{r * std::cos(a), r * std::sin(a), 0.0};
  s.linear_velocity = vec3{-speed * std::sin(a), speed * std::cos(a), 0.0};
  s.rotation = cvc::physics::quat{std::cos(0.5 * (a + 1.5707963)), 0.0, 0.0,
                                  std::sin(0.5 * (a + 1.5707963))};
  return s;
}
} // namespace

// On time, every 6 ticks, at 60 fps: the peer is shown a fixed delay behind, exactly on its path,
// and it moves the same distance every frame. The delay is one interval and a tick (7) from the
// frame that took the sample in, which here is that frame's start, 2 ticks after the sample was
// made: 5 behind the server.
TEST(HolonTest, APeerOnTimeIsShownAFixedDelayBehindOnItsPath) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const auto line = [](double t) {
    body_state s;
    s.position = vec3{3.0 * t, 0.0, 0.0};
    s.linear_velocity = vec3{3.0, 0.0, 0.0};
    return s;
  };
  const peer_run r = run_peer(client, line, 6, 0.0, 0.0, 0, 2.0, 240);
  const double dt = client.clock().fixed_dt();
  for (std::size_t f = 60; f < r.shown.size(); ++f) {
    const double shown_tick = r.shown[f].x / (3.0 * dt);
    EXPECT_NEAR(100.0 + r.t[f] - shown_tick, 5.0, 1e-6) << "frame " << f;
    EXPECT_NEAR(r.shown[f].x - r.shown[f - 1].x, 3.0 * 2.0 * dt, 1e-9) << "frame " << f;
  }
}

// The live case the old walk stuttered in: samples every 6 ticks (the relay's ~20 Hz room step)
// arriving up to 40 ms late at random, one in twenty lost, a peer walking a circle, at 60 fps. The
// peer never stalls or lurches: every frame it moves close to its true speed, stays on its circle,
// and its shown acceleration is that of the walk, not of a stop and go.
TEST(HolonTest, APeerMovesSmoothlyThroughJitterAndLoss) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  const double frame_ticks = 2.0; // 60 fps
  const peer_run r =
      run_peer(client, circle, 6, 4.0, 0.040 / dt, 20, frame_ticks, 60 * 12, /*seed=*/777u);
  const double frame_s = frame_ticks * dt;
  int stalls = 0, off_speed = 0, n = 0;
  double max_radius_err = 0.0, accel2 = 0.0;
  for (std::size_t f = 120; f + 1 < r.shown.size(); ++f) { // after 2 s (at speed by then)
    const double d =
        std::sqrt(cvc::physics::dot(r.shown[f] - r.shown[f - 1], r.shown[f] - r.shown[f - 1]));
    const double speed = d / frame_s;
    if (speed < 0.3)
      ++stalls;
    if (speed < 0.75 * 3.0 || speed > 1.25 * 3.0)
      ++off_speed;
    const double radius = std::sqrt(r.shown[f].x * r.shown[f].x + r.shown[f].y * r.shown[f].y);
    max_radius_err = std::max(max_radius_err, std::fabs(radius - 4.0));
    const vec3 acc =
        (r.shown[f + 1] - r.shown[f] * 2.0 + r.shown[f - 1]) * (1.0 / (frame_s * frame_s));
    accel2 += cvc::physics::dot(acc, acc);
    ++n;
  }
  const double accel_rms = std::sqrt(accel2 / n);
  std::printf("jittered peer: %d stalls, %d of %d frames off speed, radius error %.3f m, "
              "acceleration rms %.2f m/s^2 (the walk's own: 2.25)\n",
              stalls, off_speed, n, max_radius_err, accel_rms);
  EXPECT_EQ(stalls, 0);
  EXPECT_LE(off_speed, n / 100) << "within 25% of its speed on 99% of frames";
  EXPECT_LT(max_radius_err, 0.05);
  EXPECT_LT(accel_rms, 6.0);
}

// A teleport (a respawn, an override) is no walk: the peer is shown there, not slid across.
TEST(HolonTest, APeerTeleportIsShownAtOnce) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();
  for (int k = 0; k < 10; ++k) {
    body_state s;
    s.position = vec3{0.01 * k, 0.0, 0.0};
    client.apply_authoritative("B", s, 10 + 6 * static_cast<std::uint64_t>(k));
    for (int f = 0; f < 6; ++f)
      client.advance(dt);
  }
  body_state away;
  away.position = vec3{50.0, 0.0, 0.0};
  client.apply_authoritative("B", away, 70);
  client.advance(dt);
  EXPECT_NEAR(client.display_pose("B").position.x, 50.0, 1e-9);
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

  // Within max_skew_ticks the clocks are left alone (they run at the same rate). B walks at
  // 1 m/s, a sample every 6 ticks.
  const auto b_at = [](std::uint64_t tick) {
    body_state b;
    b.position = vec3{(static_cast<double>(tick) - 12.0) / 120.0, 0.0, 0.0};
    b.linear_velocity = vec3{1.0, 0.0, 0.0};
    return b;
  };
  for (std::uint64_t t = 12; t <= 36; t += 6) {
    client.apply_authoritative("B", b_at(t), t);
    for (int i = 0; i < 6; ++i)
      client.advance(dt);
  }
  client.apply_authoritative("A", s, 40); // 2 ticks away: no re-alignment
  EXPECT_EQ(client.tick(), 42u);
  const double b_mid = client.display_pose("B").position.x;
  EXPECT_GT(b_mid, 0.05);
  EXPECT_LT(b_mid, 0.25);

  // Past it (a stalled frame loop), the clock re-aligns -- and the peer, played out on wall time,
  // is neither moved by the seek nor thrown off its walk after it.
  client.apply_authoritative("A", s, 400);
  EXPECT_EQ(client.tick(), 400u);
  EXPECT_NEAR(client.display_pose("B").position.x, b_mid, 1e-12);
  client.apply_authoritative("B", b_at(42), 42);
  client.advance(dt);
  EXPECT_NEAR(client.display_pose("B").position.x - b_mid, 1.0 / 120.0, 1e-6)
      << "a tick on at 1 m/s";
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
// The walk below over a link of kDelay frames each way, the client leading by `lead`. With
// `stepped_lead` the host raises the lead mid-walk (frame 120, walking), as arigl does when the
// round trip grows; `slew` is holon_config::lead_slew.
double total_correction(std::uint64_t lead, std::uint64_t stepped_lead = 0, double slew = 0.05) {
  constexpr int kDelay = 6; // frames (= ticks) each way
  holon server(holon_role::server);
  holon_config cfg;
  cfg.lead_slew = slew;
  holon client(holon_role::client, cfg);
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
    if (f == 120 && stepped_lead != 0)
      client.set_lead_ticks(stepped_lead);
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

// The round trip grows mid-walk and the host raises the lead by more than the slack (arigl does
// this from the measured RTT). Slewing the clock into the new lead keeps every logged input, so
// the walk still needs no correction; seeking to it drops them, and the next sample snaps the
// avatar back -- the jump a client saw of its own avatar whenever its ping moved.
TEST(HolonTest, ALongerLeadIsSlewedIntoWithoutACorrection) {
  EXPECT_EQ(total_correction(/*lead=*/14, /*stepped_lead=*/24, /*slew=*/0.05), 0.0);
  EXPECT_GT(total_correction(/*lead=*/14, /*stepped_lead=*/24, /*slew=*/0.0), 0.01)
      << "seeking instead drops the inputs in flight";
}

// A drift past the slack but within max_skew_ticks slews the clock: it runs fast until it is
// within a tick of the target, then at its scale again -- no seek, no dropped inputs.
TEST(HolonTest, ClientSlewsTowardANewLead) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  client.set_lead_ticks(14);
  body_state s;
  client.apply_authoritative("A", s, 100); // join: seeks to 114
  ASSERT_EQ(client.tick(), 114u);
  ASSERT_EQ(client.clock_realigns(), 1u);
  client.set_lead_ticks(24); // the round trip grew
  std::uint64_t server = 100;
  client.apply_authoritative("A", s, ++server); // target 125, 11 behind
  EXPECT_EQ(client.tick(), 114u) << "no seek";
  EXPECT_TRUE(client.clock_slewing());
  EXPECT_EQ(client.clock_slews(), 1u);
  const double dt = client.clock().fixed_dt();
  int frames = 0;
  while (client.clock_slewing() && frames < 2000) {
    client.advance(dt); // the server advances one tick a frame, the client a little more
    client.apply_authoritative("A", s, ++server);
    ++frames;
  }
  EXPECT_FALSE(client.clock_slewing()) << "converged after " << frames << " frames";
  const std::uint64_t target = server + 24;
  EXPECT_LE(client.tick() > target ? client.tick() - target : target - client.tick(), 1u);
  EXPECT_EQ(client.clock_realigns(), 1u) << "only the join seeked";
  EXPECT_DOUBLE_EQ(client.clock().scale(), 1.0) << "and the clock runs at its scale again";
}

// A client whose first sample finds its clock already in step (it started with the server, as a
// client of a fresh relay can) never seeks to join; a later drift past the slack still slews, as
// it does after a joining seek. Gated on a past seek instead, that first drift seeked and dropped
// the logged inputs: the relay loopback test StillOuterPeerReentryNoRealignNoFreeze saw a realign
// on a slow runner.
TEST(HolonTest, AClientInStepFromItsFirstSampleStillSlews) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  client.set_lead_ticks(14);
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 1000 && client.tick() < 114; ++i)
    client.advance(dt);
  ASSERT_EQ(client.tick(), 114u);
  body_state s;
  client.apply_authoritative("A", s, 100); // target 114: already there
  EXPECT_EQ(client.clock_realigns(), 0u) << "nothing to seek";
  client.set_lead_ticks(24);               // the round trip grew
  client.apply_authoritative("A", s, 101); // target 125, 11 behind
  EXPECT_EQ(client.clock_realigns(), 0u) << "a drift after the first sample slews, never seeks";
  EXPECT_TRUE(client.clock_slewing());
  EXPECT_EQ(client.clock_slews(), 1u);
}

// The client's own avatar is shown between its last two quanta by the frame's alpha, as a peer is:
// frames that fall between quanta still move it, instead of a stair-step of still frames and
// whole-quantum jumps (which the camera following it spread to the whole view).
TEST(HolonTest, TheLocalAvatarIsShownBetweenQuanta) {
  holon client(holon_role::client);
  client.add_participant("A", /*local=*/true);
  client.set_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}});
  const double dt = client.clock().fixed_dt();
  client.advance(1.5 * dt); // one quantum, alpha 0.5
  const double a = client.display_pose("A").position.x;
  client.advance(0.25 * dt); // no quantum, alpha 0.75
  const double b = client.display_pose("A").position.x;
  client.advance(0.5 * dt); // a quantum, alpha 0.25
  const double c = client.display_pose("A").position.x;
  EXPECT_NEAR(a, 0.5 * 2.0 * dt, 1e-9);
  EXPECT_NEAR(b, 0.75 * 2.0 * dt, 1e-9) << "a frame between quanta still moves it";
  EXPECT_NEAR(c, 1.25 * 2.0 * dt, 1e-9);
  EXPECT_NEAR(client.state("A").position.x, 2.0 * 2.0 * dt, 1e-9) << "the state is still exact";
}

TEST(HolonTest, ClientAlignsAheadOfTheServerByItsLead) {
  holon_config cfg;
  cfg.lead_slew = 0.0; // seek, to see the alignment at once (ClientSlewsTowardANewLead slews)
  holon client(holon_role::client, cfg);
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

// Every clock seek drops the local avatar's unacknowledged inputs, so after the one at join a
// client should see none; clock_realigns counts them, so a host can tell when it does.
TEST(HolonTest, RealignCounterCountsSeeks) {
  holon_config cfg;
  cfg.lead_slew = 0.0; // every drift past the slack seeks
  holon client(holon_role::client, cfg);
  client.add_participant("A", /*local=*/true);
  client.add_participant("B", /*local=*/false);
  client.set_lead_ticks(14);
  EXPECT_EQ(client.clock_realigns(), 0u);
  body_state s;
  client.apply_authoritative("A", s, 100); // joining: the clock seeks to 114
  ASSERT_EQ(client.tick(), 114u);
  EXPECT_EQ(client.clock_realigns(), 1u) << "the alignment at join counts";
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 30; ++i)
    client.advance(dt);
  ASSERT_EQ(client.tick(), 144u);

  // Target 112 + 14 = 126, 18 ticks behind the replica: within the slack, no seek.
  client.apply_authoritative("A", s, 112);
  EXPECT_EQ(client.tick(), 144u);
  EXPECT_EQ(client.clock_realigns(), 1u) << "an in-tolerance sample is not a re-alignment";

  // A sample stamped more than lead_slack_ahead_ticks (24) behind -- a peer whose tick a
  // heartbeat left old -- seeks the clock back, and counts.
  client.apply_authoritative("B", s, 105); // target 119, 25 behind
  EXPECT_EQ(client.tick(), 119u);
  EXPECT_EQ(client.clock_realigns(), 2u);
  client.apply_authoritative("B", s, 105); // already seen: not a sample, not a seek
  EXPECT_EQ(client.clock_realigns(), 2u);
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
  EXPECT_GT(client.display_pose("B").position.x, 0.99) << "walked in over an interval, not 240";
}

// ── physics: rigid-body avatars in a Jolt world ─────────────────────────────

namespace {
holon_config physics_config() {
  holon_config cfg;
  cfg.physics = true;
  return cfg;
}
// A 20 m floor (a slab whose top is z = 0) and a 1 m cube at the origin, as the default room.
std::vector<cvc::ariadne::avatar_static> room_statics() {
  cvc::ariadne::avatar_static ground;
  ground.center = vec3{0.0, 0.0, -0.1};
  ground.half_extents = vec3{10.0, 10.0, 0.1};
  cvc::ariadne::avatar_static cube;
  cube.center = vec3{0.0, 0.0, 0.5};
  cube.half_extents = vec3{0.5, 0.5, 0.5};
  return {ground, cube};
}
body_state at(double x, double y, double z = 0.0) {
  body_state s;
  s.position = vec3{x, y, z};
  return s;
}
void run(holon &h, int frames) {
  for (int i = 0; i < frames; ++i)
    h.advance(h.clock().fixed_dt());
}
} // namespace

TEST(HolonPhysicsTest, AnAvatarStandsOnTheFloorAndFallsOntoIt) {
  holon server(holon_role::server, physics_config());
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(4.0, 0.0, 1.0)); // a metre up
  run(server, 240);                                      // 2 s
  EXPECT_NEAR(server.state("A").position.z, 0.0, 0.02) << "gravity brought it down onto the floor";
  EXPECT_NEAR(server.state("A").position.x, 4.0, 1e-3) << "and it stayed where it landed";
}

TEST(HolonPhysicsTest, AnAvatarAcceleratesWithInertia) {
  holon server(holon_role::server, physics_config());
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(4.0, 0.0));
  run(server, 12);
  server.set_intent("A", avatar_intent{vec3{0.0, 2.0, 0.0}});
  run(server, 6); // 50 ms at max_accel 12: ~0.6 m/s, not 2
  const double v = server.state("A").linear_velocity.y;
  EXPECT_GT(v, 0.3);
  EXPECT_LT(v, 1.0) << "mass and a finite drive: no instant top speed";
  run(server, 60);
  EXPECT_NEAR(server.state("A").linear_velocity.y, 2.0, 0.05) << "then it reaches the intent's";
}

TEST(HolonPhysicsTest, AnAvatarStopsAtTheCube) {
  holon server(holon_role::server, physics_config());
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(-3.0, 0.0));
  server.set_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}});
  run(server, 360); // 3 s: far enough to have walked through it
  const vec3 p = server.state("A").position;
  EXPECT_LT(p.x, -0.5 - 0.25 + 0.02) << "its front face stops at the cube's";
  EXPECT_GT(p.x, -0.85);
  EXPECT_NEAR(p.y, 0.0, 0.05);
  EXPECT_NEAR(p.z, 0.0, 0.02) << "upright on the floor, not climbing or tipping";
}

TEST(HolonPhysicsTest, AvatarsCollideAndPushEachOther) {
  holon server(holon_role::server, physics_config());
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(-6.0, 5.0));
  server.add_participant("B", false, at(-4.0, 5.0)); // standing still, in A's way
  server.set_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}});
  run(server, 240); // 2 s
  const vec3 a = server.state("A").position, b = server.state("B").position;
  EXPECT_GE(b.x - a.x, 0.5 - 0.02) << "they never pass through each other";
  EXPECT_GT(b.x, -4.0 + 0.5) << "A shoved B out of its way";
  EXPECT_NEAR(a.z, 0.0, 0.02);
  EXPECT_NEAR(b.z, 0.0, 0.02);
}

TEST(HolonPhysicsTest, AnAvatarThatFallsOffTheWorldRespawns) {
  holon server(holon_role::server, physics_config());
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(9.0, 0.0));
  server.set_intent("A", avatar_intent{vec3{3.0, 0.0, 0.0}});
  run(server, 120); // walks off the edge at x = 10
  EXPECT_LT(server.state("A").position.z, -0.5) << "it fell";
  server.set_intent("A", avatar_intent{});
  run(server, 360);
  const vec3 p = server.state("A").position;
  EXPECT_NEAR(p.x, 9.0, 0.05) << "and came back where it joined";
  EXPECT_GT(p.z, -0.05);
}

// The leading client again, now with Jolt on both sides and the cube in the walk: the client's
// world predicts the stop at the cube exactly as the server simulates it, so reconcile has nothing
// to correct.
TEST(HolonPhysicsTest, ClientPredictionMatchesTheServersPhysics) {
  holon server(holon_role::server, physics_config());
  holon client(holon_role::client, physics_config());
  if (!server.physics() || !client.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  server.set_statics(room_statics(), 1);
  client.set_statics(room_statics(), 1);
  server.add_participant("A", false, at(-3.0, 0.2));
  client.add_participant("A", true, at(-3.0, 0.2));
  constexpr int kDelay = 6;
  client.set_lead_ticks(2 * kDelay + 2);
  const double dt = server.clock().fixed_dt();
  struct msg {
    int at;
    avatar_intent in;
    std::uint64_t tick;
    body_state s;
  };
  std::vector<msg> up, down;
  avatar_intent cur;
  double corrected = 0.0;
  for (int f = 0; f < 600; ++f) {
    avatar_intent want = cur;
    if (f == 60)
      want.move = vec3{2.0, 0.0, 0.0}; // into the cube
    if (f == 300)
      want.move = vec3{0.0, 2.0, 0.0}; // then along its face
    if (f == 420)
      want.move = vec3{0.0, 0.0, 0.0};
    if (want.move.x != cur.move.x || want.move.y != cur.move.y) {
      cur = want;
      client.set_intent("A", cur);
      up.push_back({f + kDelay, cur, client.tick() + 1, {}});
    }
    client.advance(dt);
    for (auto it = up.begin(); it != up.end();)
      if (it->at <= f) {
        server.queue_intent("A", it->in, it->tick);
        it = up.erase(it);
      } else {
        ++it;
      }
    server.advance(dt);
    down.push_back({f + kDelay, {}, server.tick(), server.state("A")});
    for (auto it = down.begin(); it != down.end();)
      if (it->at <= f) {
        client.apply_authoritative("A", it->s, it->tick);
        it = down.erase(it);
      } else {
        ++it;
      }
    const vec3 c = client.take_correction("A");
    if (f >= 60)
      corrected += std::fabs(c.x) + std::fabs(c.y) + std::fabs(c.z);
  }
  EXPECT_LT(server.state("A").position.x, -0.5) << "the server stopped it at the cube";
  EXPECT_LT(corrected, 1e-3) << "and the client predicted the same";
  EXPECT_NEAR(client.state("A").position.x, server.state("A").position.x, 0.05);
}

// A peer the host registered before its pose arrived (client_pump does, from the roster) is a
// member with nowhere to be yet: no kinematic body at its placeholder -- at the origin that would
// be an invisible obstacle inside the reference cube -- until its first sample puts one there.
// The placeholder here stands in the local avatar's path, so its absence is visible.
TEST(HolonPhysicsTest, PoseLessPeerHasNoBodyUntilFirstSample) {
  holon client(holon_role::client, physics_config());
  client.add_participant("A", /*local=*/true, at(-3.0, 5.0));
  client.add_participant("B", /*local=*/false, at(0.0, 5.0), /*pose_known=*/false);
  EXPECT_TRUE(client.has_participant("B")) << "a member like any other";
  EXPECT_TRUE(client.pose_known("A"));
  EXPECT_FALSE(client.pose_known("B"));
  EXPECT_FALSE(client.pose_known("nobody"));
  if (!client.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  client.set_statics(room_statics(), 1);
  EXPECT_TRUE(client.has_body("A")) << "the local avatar is predicted from the start";
  EXPECT_FALSE(client.has_body("B")) << "no body at the placeholder";
  EXPECT_FALSE(client.has_body("nobody"));

  // A walks through where the placeholder is: nothing there to stop it.
  client.set_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}});
  run(client, 300); // 2.5 s: about 4.8 m
  const double walked = client.state("A").position.x;
  EXPECT_GT(walked, 1.0) << "on past the placeholder";

  // B's first sample, 1.5 m ahead of A: a kinematic body at the sample, which A walks into.
  const double bx = walked + 1.5;
  client.apply_authoritative("B", at(bx, 5.0), client.tick());
  EXPECT_TRUE(client.pose_known("B"));
  EXPECT_TRUE(client.has_body("B"));
  run(client, 240);
  const vec3 a = client.state("A").position;
  EXPECT_LT(a.x, bx - 0.5 + 0.02) << "A's front face stops at B's";
  EXPECT_GT(a.x, bx - 0.6);
  EXPECT_NEAR(a.y, 5.0, 0.05);
}

// ── authority: the server's veto and overrides (§5h propose -> veto -> override) ──

TEST(HolonAuthorityTest, TheVetoCapsSpeedDropsTheVerticalAndRejectsNonFinite) {
  holon_config cfg;
  cfg.max_speed = 3.0;
  holon server(holon_role::server, cfg);
  const double nan = std::nan("");
  avatar_intent fast{vec3{30.0, 40.0, 5.0}};
  fast.yaw = 7.0; // more than a turn
  const avatar_intent a = server.sanitize_intent(fast);
  EXPECT_NEAR(a.move.x, 1.8, 1e-12) << "scaled down to max_speed, direction kept";
  EXPECT_NEAR(a.move.y, 2.4, 1e-12);
  EXPECT_EQ(a.move.z, 0.0) << "avatars walk; gravity owns z";
  EXPECT_NEAR(a.yaw, 7.0 - 2.0 * 3.14159265358979323846, 1e-12) << "wrapped to [-pi, pi]";
  avatar_intent bad{vec3{nan, 1.0, 0.0}};
  bad.yaw = std::numeric_limits<double>::infinity();
  const avatar_intent b = server.sanitize_intent(bad);
  EXPECT_EQ(b.move.x, 0.0);
  EXPECT_EQ(b.move.y, 0.0) << "a non-finite move is no move at all";
  EXPECT_EQ(b.yaw, 0.0);
  const avatar_intent ok{vec3{1.0, 1.0, 0.0}};
  const avatar_intent c = server.sanitize_intent(ok);
  EXPECT_TRUE(c.move.x == 1.0 && c.move.y == 1.0 && c.yaw == 0.0)
      << "an honest intent passes as is";

  // The server integrates the vetoed intent, and counts the veto.
  server.add_participant("A");
  const double dt = server.clock().fixed_dt();
  server.set_intent("A", avatar_intent{vec3{100.0, 0.0, 0.0}});
  for (int i = 0; i < 120; ++i)
    server.advance(dt);
  EXPECT_NEAR(server.state("A").position.x, 3.0, 1e-6) << "1 s at the 3 m/s cap, not 100";
  server.queue_intent("A", bad, server.tick() + 1);
  EXPECT_EQ(server.vetoes(), 2u);
}

// A client that asks for more than the cap predicts with the vetoed intent too, so it still agrees
// with the server: the veto costs an honest client nothing, and a client running a modified
// prediction is simply overruled on reconcile.
TEST(HolonAuthorityTest, AClientPredictsTheVetoedIntent) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");
  client.add_participant("A", true);
  const double dt = server.clock().fixed_dt();
  const avatar_intent greedy{vec3{50.0, 0.0, 0.0}};
  server.set_intent("A", greedy);
  client.set_intent("A", greedy);
  for (int f = 1; f <= 60; ++f) {
    server.advance(dt);
    client.advance(dt);
    client.apply_authoritative("A", server.state("A"), server.tick());
  }
  const vec3 c = client.take_correction("A");
  EXPECT_EQ(c.x, 0.0);
  EXPECT_DOUBLE_EQ(client.state("A").position.x, server.state("A").position.x);
  EXPECT_NEAR(server.state("A").position.x, 0.5 * 4.0, 1e-6) << "0.5 s at the default 4 m/s cap";
}

TEST(HolonAuthorityTest, TheServerTeleportsAnAvatarAndTheClientFollows) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");
  client.add_participant("A", true);
  const double dt = server.clock().fixed_dt();
  server.advance(dt);
  client.advance(dt);
  EXPECT_FALSE(client.override_state("A", at(9.0, 9.0))) << "only the authority overrides";
  ASSERT_TRUE(server.override_state("A", at(6.0, -2.0)));
  EXPECT_FALSE(server.override_state("nobody", at(0.0, 0.0)));
  server.advance(dt);
  client.advance(dt);
  client.apply_authoritative("A", server.state("A"), server.tick());
  EXPECT_DOUBLE_EQ(client.state("A").position.x, 6.0) << "the owner reconciles to the override";
  EXPECT_DOUBLE_EQ(client.state("A").position.y, -2.0);
}

TEST(HolonAuthorityTest, AReMassedAvatarIsHarderToPush) {
  // How far a 70 kg walker shoves a still avatar of mass `kg` in 2 s.
  const auto shove = [](double kg) {
    holon server(holon_role::server, physics_config());
    if (!server.physics())
      return -1.0;
    server.set_statics(room_statics(), 1);
    server.add_participant("A", false, at(-6.0, 5.0));
    server.add_participant("B", false, at(-4.0, 5.0));
    EXPECT_TRUE(server.set_mass("B", kg));
    server.set_intent("A", avatar_intent{vec3{2.0, 0.0, 0.0}});
    run(server, 240);
    return server.state("B").position.x - (-4.0);
  };
  const double light = shove(70.0), heavy = shove(2000.0);
  if (light < 0.0)
    GTEST_SKIP() << "no Jolt backend in this build";
  EXPECT_GT(light, 0.5) << "an equal mass is shoved aside";
  EXPECT_LT(heavy, 0.5 * light) << "a heavy one barely gives way";
  holon client(holon_role::client, physics_config());
  client.add_participant("A", true);
  EXPECT_FALSE(client.set_mass("A", 10.0)) << "only the authority re-masses";
}

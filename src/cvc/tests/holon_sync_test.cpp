/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::ariadne holon <-> state-tree sync over scalar keys (H1 slice 4):
// the pose/intent scalar-leaf round-trip, and an end-to-end where a participant's
// avatar is server-authoritative over REPLICATED scalar keys -- the client
// proposes intent, the server holon integrates it and writes the authoritative
// pose, and the client reconciles to it. No typed data() object, no DSL verb (the
// C++ verb + arisrv hosting are the live-integration follow-on): just the mapping
// and that it rides the existing conf.rooms.* replication.

#include <cstdint>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h>
#include <cvc/core/app.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::holon;
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

// ── scalar-leaf round-trip (one app) ────────────────────────────────────────

TEST(HolonSyncTest, PoseRoundTripsThroughScalarKeys) {
  cvc::app app;
  body_state s;
  s.position = vec3{1.5, -2.25, 3.125};
  s.rotation = cvc::physics::quat{0.5, -0.5, 0.5, -0.5};
  s.linear_velocity = vec3{0.1, 0.2, 0.3};
  s.angular_velocity = vec3{-0.4, 0.5, -0.6};
  cvc::ariadne::write_pose(app, "r", "A", s, /*server_tick=*/42);

  body_state out;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(app, "r", "A", out, tick));
  EXPECT_EQ(tick, 42u);
  // 17-sig-digit text recovers binary64 exactly.
  expect_pos_bit_eq(out.position, s.position);
  EXPECT_EQ(bits(out.rotation.w), bits(s.rotation.w));
  EXPECT_EQ(bits(out.rotation.z), bits(s.rotation.z));
  expect_pos_bit_eq(out.linear_velocity, s.linear_velocity);
  expect_pos_bit_eq(out.angular_velocity, s.angular_velocity);
}

TEST(HolonSyncTest, IntentRoundTripsThroughScalarKeys) {
  cvc::app app;
  const avatar_intent in{vec3{7.0, -8.0, 9.5}};
  cvc::ariadne::write_intent(app, "r", "A", in);
  avatar_intent out;
  ASSERT_TRUE(cvc::ariadne::read_intent(app, "r", "A", out));
  expect_pos_bit_eq(out.move, in.move);
}

TEST(HolonSyncTest, ReadReturnsFalseWhenAbsent) {
  cvc::app app;
  body_state s;
  std::uint64_t tick = 0;
  avatar_intent in;
  EXPECT_FALSE(cvc::ariadne::read_pose(app, "r", "ghost", s, tick));
  EXPECT_FALSE(cvc::ariadne::read_intent(app, "r", "ghost", in));
}

// ── end-to-end: server-authoritative pose over replicated scalar keys ───────

TEST(HolonSyncTest, ServerAuthoritativePoseReplicatesAndClientReconciles) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);

  holon server(holon_role::server);
  holon client(holon_role::client);
  const std::string room = "r", pid = "A";
  server.add_participant(pid);                 // authoritative on the server
  client.add_participant(pid, /*local=*/true); // the client predicts its own avatar
  const double dt = server.clock().fixed_dt();
  const avatar_intent mv{vec3{2.0, 0.0, 0.0}};

  for (int frame = 1; frame <= 8; ++frame) {
    // Client proposes its intent; it replicates to the server, which integrates it.
    client.set_intent(pid, mv);
    cvc::ariadne::write_intent(client_app, room, pid, mv);
    tp.pump_all();
    avatar_intent got;
    ASSERT_TRUE(cvc::ariadne::read_intent(server_app, room, pid, got)) << "frame " << frame;
    server.set_intent(pid, got);

    // Both advance one quantum.
    server.advance(dt);
    client.advance(dt);

    // The server writes the authoritative pose; it replicates; the client reconciles.
    cvc::ariadne::write_pose(server_app, room, pid, server.state(pid), server.tick());
    tp.pump_all();
    body_state auth;
    std::uint64_t stick = 0;
    ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, auth, stick)) << "frame " << frame;
    EXPECT_EQ(stick, server.tick());
    client.apply_authoritative(pid, auth, stick);

    // A perfect prediction holds; the client's avatar tracks the authoritative one.
    expect_pos_bit_eq(client.state(pid).position, server.state(pid).position);
  }
  EXPECT_GT(server.state(pid).position.x, 0.0);

  tp.unregister_shard(&ss);
  tp.unregister_shard(&cs);
  ss.detach();
  cs.detach();
}

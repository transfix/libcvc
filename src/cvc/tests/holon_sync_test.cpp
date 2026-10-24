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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/conference/room_intrinsics.h>
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

// server_pump drives the whole roster: it reads each present participant's intent,
// integrates the server holon, and writes each authoritative pose -- the loop
// arisrv runs. (Injected dt so the step count is deterministic in a tight loop.)
TEST(HolonSyncTest, ServerPumpIntegratesRosterFromIntents) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);
  const std::string room = "r";

  // Two participants join and propose intent on the client side.
  cvc::ariadne::conference::room_join(client_app, room, "A");
  cvc::ariadne::conference::room_join(client_app, room, "B");
  cvc::ariadne::write_intent(client_app, room, "A", avatar_intent{vec3{1.0, 0.0, 0.0}});
  cvc::ariadne::write_intent(client_app, room, "B", avatar_intent{vec3{0.0, 2.0, 0.0}});
  tp.pump_all(); // replicate the roster + intents to the server

  holon server(holon_role::server);
  const double dt = server.clock().fixed_dt();
  // The first pump registers both newcomers at their spawn points (then steps one quantum).
  cvc::ariadne::server_pump(server, server_app, room, dt);
  tp.pump_all();
  const vec3 a0 = server.state("A").position, b0 = server.state("B").position;
  // Spawned on the 4 m ring around the origin, apart from each other.
  EXPECT_NEAR(std::hypot(a0.x, a0.y), 4.0, 1.0 * dt + 1e-9);
  EXPECT_NEAR(std::hypot(b0.x, b0.y), 4.0, 2.0 * dt + 1e-9);
  EXPECT_GT(std::hypot(a0.x - b0.x, a0.y - b0.y), 1.0) << "joiners never stack";
  for (int frame = 1; frame < 5; ++frame) {
    cvc::ariadne::server_pump(server, server_app, room, dt);
    tp.pump_all();
  }

  // The server integrated each avatar from its own intent and wrote authoritative
  // poses, tagged with the holon tick.
  body_state pa, pb;
  std::uint64_t ta = 0, tb = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(server_app, room, "A", pa, ta));
  ASSERT_TRUE(cvc::ariadne::read_pose(server_app, room, "B", pb, tb));
  EXPECT_GT(pa.position.x, a0.x) << "A moved +x per its intent";
  EXPECT_DOUBLE_EQ(pa.position.y, a0.y);
  EXPECT_GT(pb.position.y, b0.y) << "B moved +y per its intent";
  EXPECT_DOUBLE_EQ(pb.position.x, b0.x);
  EXPECT_EQ(ta, server.tick());

  // And the authoritative poses replicate back to the client.
  tp.pump_all();
  body_state ca;
  std::uint64_t ct = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, "A", ca, ct));
  EXPECT_GT(ca.position.x, 0.0);

  tp.unregister_shard(&ss);
  tp.unregister_shard(&cs);
  ss.detach();
  cs.detach();
}

// client_pump end to end, timed like the live relay: clients pump every quantum, the
// server every 24 (~5 Hz). Client A predicts its own avatar and stays near the
// authoritative one; client B sees A as a peer that moves forward smoothly.
TEST(HolonSyncTest, ClientPumpPredictsSelfAndWalksPeersSmoothly) {
  cvc::app server_app, a_app, b_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard as(a_app, "conf", "A");
  cvc::state_cluster_shard bs(b_app, "conf", "B");
  for (cvc::state_cluster_shard *s : {&ss, &as, &bs}) {
    s->attach();
    tp.register_shard(s);
  }
  const std::string room = "r";
  cvc::ariadne::conference::room_join(a_app, room, "A");
  cvc::ariadne::conference::room_join(b_app, room, "B");
  tp.pump_all();

  holon server(holon_role::server);
  holon a(holon_role::client), b(holon_role::client);
  const double dt = server.clock().fixed_dt();
  constexpr int kServerEvery = 24;

  cvc::ariadne::client_pump(a, a_app, room, "A", dt); // registers A (local) + B (peer)
  cvc::ariadne::propose_intent(a, a_app, room, "A", avatar_intent{vec3{2.0, 0.0, 0.0}});
  tp.pump_all();

  double last_peer_x = -1.0;
  for (int frame = 1; frame <= 10 * kServerEvery; ++frame) {
    if (frame % kServerEvery == 0)
      cvc::ariadne::server_pump(server, server_app, room, kServerEvery * dt);
    tp.pump_all();
    cvc::ariadne::client_pump(a, a_app, room, "A", dt);
    cvc::ariadne::client_pump(b, b_app, room, "B", dt);
    ASSERT_TRUE(b.has_participant("A"));
    const double peer_x = b.display_pose("A").position.x;
    EXPECT_GE(peer_x, last_peer_x) << "frame " << frame << ": B never sees A step back";
    last_peer_x = peer_x;
  }

  const double server_x = server.state("A").position.x;
  EXPECT_GT(server_x, 3.9)
      << "the server moved A per its intent (2 m/s for 2 s), no dropped quanta";
  // A's own prediction stays within a relay interval of the authority.
  EXPECT_NEAR(a.display_pose("A").position.x, server_x, 2.0 * kServerEvery * dt + 1e-9);
  // B shows A at most one sample interval behind the authority.
  EXPECT_NEAR(last_peer_x, server_x, 2.0 * kServerEvery * dt + 1e-9);
  // Both replicas count the server's ticks.
  EXPECT_LE(a.tick() > server.tick() ? a.tick() - server.tick() : server.tick() - a.tick(),
            static_cast<std::uint64_t>(kServerEvery));
  EXPECT_LE(b.tick() > server.tick() ? b.tick() - server.tick() : server.tick() - b.tick(),
            static_cast<std::uint64_t>(kServerEvery));

  // A participant who leaves is dropped from the replicas, and from the server holon.
  ASSERT_TRUE(server.has_participant("B"));
  cvc::ariadne::conference::room_leave(b_app, room, "B");
  tp.pump_all();
  cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  EXPECT_FALSE(a.has_participant("B"));
  cvc::ariadne::server_pump(server, server_app, room, dt);
  EXPECT_FALSE(server.has_participant("B")) << "the server stops integrating a departed avatar";
  EXPECT_TRUE(server.has_participant("A"));

  for (cvc::state_cluster_shard *s : {&ss, &as, &bs}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

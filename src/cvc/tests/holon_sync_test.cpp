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
#include <cvc/ariadne/conference/room_scene.h>
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
  // B shows A one sample interval (and a tick) behind the authority: its playout delay with no
  // jitter (holon::display_pose).
  EXPECT_NEAR(last_peer_x, server_x, 2.0 * (kServerEvery + 2) * dt + 1e-9);
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

// client_pump registers whoever the roster lists, but one whose pose has not replicated yet is
// pose-less: a member with nothing to draw and, with physics, no peer body at the placeholder
// origin. Its first replicated pose makes it known, where the server put it.
TEST(HolonSyncTest, ClientPumpMarksPoseLessPeersUnknown) {
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

  holon a(holon_role::client, cvc::ariadne::room_holon_config());
  const double dt = a.clock().fixed_dt();
  cvc::ariadne::client_pump(a, a_app, room, "A", dt); // the roster is here, no pose yet
  ASSERT_TRUE(a.has_participant("A"));
  ASSERT_TRUE(a.has_participant("B")) << "membership does not wait for a pose";
  EXPECT_FALSE(a.pose_known("A"));
  EXPECT_FALSE(a.pose_known("B"));
  if (a.physics()) {
    EXPECT_TRUE(a.has_body("A")) << "the local avatar is predicted from the start";
    EXPECT_FALSE(a.has_body("B")) << "no peer body at the placeholder origin";
  }

  // The server spawns both on its ring and publishes their poses.
  holon server(holon_role::server);
  cvc::ariadne::server_pump(server, server_app, room, dt);
  tp.pump_all();
  cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  EXPECT_TRUE(a.pose_known("A"));
  EXPECT_TRUE(a.pose_known("B"));
  if (a.physics())
    EXPECT_TRUE(a.has_body("B")) << "a kinematic body, at its first sample";
  const vec3 shown = a.display_pose("B").position, spawned = server.state("B").position;
  EXPECT_DOUBLE_EQ(shown.x, spawned.x);
  EXPECT_DOUBLE_EQ(shown.y, spawned.y);
  EXPECT_GT(std::hypot(shown.x, shown.y), 3.0) << "on the spawn ring, not at the origin";

  // A client that joins once the poses have replicated knows every one from its first pump.
  holon b(holon_role::client);
  cvc::ariadne::client_pump(b, b_app, room, "B", dt);
  EXPECT_TRUE(b.pose_known("A"));
  EXPECT_TRUE(b.pose_known("B"));

  for (cvc::state_cluster_shard *s : {&ss, &as, &bs}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

// A peer that leaves and comes back under the same pid (arigl reconnects in-process) still has
// the pose subtree it left in our tree, and the relay sends nothing newer while it is departed.
// That leftover is no pose for its rejoin: registering it from it would show the peer where it
// left and seek our clock back to the old tick. A newer sample makes it known; so does the low
// tick of a restarted relay, whose clock starts again near zero.
TEST(HolonSyncTest, RejoinDoesNotTakeTheLeftoverPose) {
  cvc::app app;
  const std::string room = "r";
  cvc::ariadne::conference::room_join(app, room, "A");
  cvc::ariadne::conference::room_join(app, room, "B");
  const auto at = [](double x, double y) {
    cvc::physics::body_state s;
    s.position = {x, y, 0.0};
    return s;
  };
  holon a(holon_role::client, cvc::ariadne::room_holon_config());
  const double dt = a.clock().fixed_dt();
  cvc::ariadne::write_pose(app, room, "B", at(5.0, 0.0), 5000);
  cvc::ariadne::client_pump(a, app, room, "A", dt);
  ASSERT_TRUE(a.pose_known("B"));
  EXPECT_EQ(a.left_tick("B"), 0u);

  cvc::ariadne::conference::room_leave(app, room, "B");
  cvc::ariadne::client_pump(a, app, room, "A", dt);
  ASSERT_FALSE(a.has_participant("B"));
  EXPECT_EQ(a.left_tick("B"), 5000u);

  // Back: the leftover (tick 5000) is no pose, frame after frame, and moves no clock.
  cvc::ariadne::conference::room_join(app, room, "B");
  const std::uint64_t realigns = a.clock_realigns();
  for (int i = 0; i < 3; ++i) {
    cvc::ariadne::client_pump(a, app, room, "A", dt);
    ASSERT_TRUE(a.has_participant("B"));
    EXPECT_FALSE(a.pose_known("B")) << "frame " << i;
    if (a.physics())
      EXPECT_FALSE(a.has_body("B")) << "no body where it left";
  }
  EXPECT_EQ(a.clock_realigns(), realigns) << "no seek back to the old tick";

  // The relay respawns it: a newer sample makes it known, where it is now.
  cvc::ariadne::write_pose(app, room, "B", at(-4.0, 1.0), 5600);
  cvc::ariadne::client_pump(a, app, room, "A", dt);
  ASSERT_TRUE(a.pose_known("B"));
  EXPECT_DOUBLE_EQ(a.display_pose("B").position.x, -4.0);
  EXPECT_EQ(a.left_tick("B"), 0u) << "forgotten once a real sample is applied";

  // Left again, and the relay restarts: its first tick is far below the old one, so it is new.
  cvc::ariadne::conference::room_leave(app, room, "B");
  cvc::ariadne::client_pump(a, app, room, "A", dt);
  ASSERT_EQ(a.left_tick("B"), 5600u);
  cvc::ariadne::conference::room_join(app, room, "B");
  cvc::ariadne::write_pose(app, room, "B", at(2.0, 2.0), 300);
  cvc::ariadne::client_pump(a, app, room, "A", dt);
  EXPECT_TRUE(a.pose_known("B")) << "a restarted relay's low tick is not the leftover";
}

// ── journaling: one mutation per written pose leaf ──────────────────────────

namespace {
// Publish what SHARD journaled since its last drain to the transport's other shards; returns how
// many mutations that was (what the writes since then cost on the wire).
std::size_t publish_drained(cvc::state_transport_inproc &tp, cvc::state_cluster_shard &shard) {
  const std::vector<cvc::state_mutation> drained = shard.drain_local();
  for (const cvc::state_mutation &m : drained)
    tp.publish(m);
  return drained.size();
}
} // namespace

// A pose has 14 leaves (13 doubles and the server_tick). Each one written journals ONE mutation: a
// changed leaf its value, not a touch and then the value.
TEST(HolonSyncTest, ChangedOnlyPoseWritesOneMutationPerChangedLeaf) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);
  const std::string room = "r", pid = "A";
  constexpr std::uint64_t kHeartbeat = 120;
  body_state s;
  s.position = vec3{1.0, 2.0, 0.0};
  s.linear_velocity = vec3{0.5, 0.0, 0.0};
  body_state got;
  std::uint64_t tick = 0;

  // A fresh pose: every leaf, once, and it converges on the client.
  cvc::ariadne::write_pose(server_app, room, pid, s, 100, kHeartbeat);
  EXPECT_EQ(publish_drained(tp, ss), 14u);
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, got, tick));
  EXPECT_EQ(tick, 100u);
  expect_pos_bit_eq(got.position, s.position);
  expect_pos_bit_eq(got.linear_velocity, s.linear_velocity);

  // One leaf moved: that leaf and the tick.
  s.position.x = 1.5;
  cvc::ariadne::write_pose(server_app, room, pid, s, 110, kHeartbeat);
  EXPECT_EQ(publish_drained(tp, ss), 2u);
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, got, tick));
  EXPECT_EQ(tick, 110u);
  expect_pos_bit_eq(got.position, s.position);

  // Unchanged within the heartbeat: nothing.
  cvc::ariadne::write_pose(server_app, room, pid, s, 150, kHeartbeat);
  EXPECT_EQ(publish_drained(tp, ss), 0u);

  // Unchanged a heartbeat on: the tick alone.
  cvc::ariadne::write_pose(server_app, room, pid, s, 230, kHeartbeat);
  EXPECT_EQ(publish_drained(tp, ss), 1u);
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, got, tick));
  EXPECT_EQ(tick, 230u);
  expect_pos_bit_eq(got.position, s.position);

  tp.unregister_shard(&ss);
  tp.unregister_shard(&cs);
  ss.detach();
  cs.detach();
}

// Heartbeat 0 is the forced rewrite: every leaf goes out again, changed or not, once each -- so a
// peer that saw none of the earlier writes converges from that one write.
TEST(HolonSyncTest, ForcedPoseRewritesEveryLeafOnce) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);
  const std::string room = "r", pid = "A";
  body_state s;
  s.position = vec3{-2.0, 0.5, 0.0};
  s.rotation = cvc::physics::quat{0.5, -0.5, 0.5, -0.5};

  cvc::ariadne::write_pose(server_app, room, pid, s, 100);
  EXPECT_EQ(publish_drained(tp, ss), 14u) << "a fresh pose";
  cvc::ariadne::write_pose(server_app, room, pid, s, 100);
  EXPECT_EQ(publish_drained(tp, ss), 14u) << "unchanged, even the tick: each leaf re-sent once";
  s.position.y = -1.0;
  cvc::ariadne::write_pose(server_app, room, pid, s, 101);
  EXPECT_EQ(publish_drained(tp, ss), 14u) << "one moved: still once per leaf, not twice for it";

  // A peer that joins now (an inproc transport replays nothing) is re-hydrated by the next one.
  cvc::app late_app;
  cvc::state_cluster_shard ls(late_app, "conf", "late");
  ls.attach();
  tp.register_shard(&ls);
  cvc::ariadne::write_pose(server_app, room, pid, s, 102);
  EXPECT_EQ(publish_drained(tp, ss), 14u);
  body_state got;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(late_app, room, pid, got, tick));
  EXPECT_EQ(tick, 102u);
  expect_pos_bit_eq(got.position, s.position);
  EXPECT_EQ(bits(got.rotation.w), bits(s.rotation.w));
  EXPECT_EQ(bits(got.rotation.z), bits(s.rotation.z));
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, got, tick));
  EXPECT_EQ(tick, 102u);
  expect_pos_bit_eq(got.position, s.position);

  for (cvc::state_cluster_shard *sh : {&ss, &cs, &ls}) {
    tp.unregister_shard(sh);
    sh->detach();
  }
}

// A leaf can exist, empty, before its first write -- made by a lookup that creates nodes
// (state::operator()), which signals nothing, so the state adapter is not observing it yet. Its
// first value must still journal: it is touched first, which attaches it.
TEST(HolonSyncTest, PreCreatedEmptyLeafStillReplicates) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);
  const std::string room = "r", pid = "A";
  const std::string sep = cvc::state::SEPARATOR;
  const std::string x_path = cvc::ariadne::conference::room_participant_path(room, pid) + sep +
                             "pose" + sep + "pos" + sep + "x";
  server_app.root()(x_path); // created after the shard attached: present, empty, unobserved
  ASSERT_NE(server_app.root().findDescendant(x_path), nullptr);

  body_state s;
  s.position = vec3{3.25, -1.0, 0.0};
  cvc::ariadne::write_pose(server_app, room, pid, s, 7, /*heartbeat_ticks=*/120);
  EXPECT_EQ(publish_drained(tp, ss), 14u) << "the pre-created leaf journaled too";
  body_state got;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(client_app, room, pid, got, tick));
  EXPECT_EQ(tick, 7u);
  expect_pos_bit_eq(got.position, s.position);

  tp.unregister_shard(&ss);
  tp.unregister_shard(&cs);
  ss.detach();
  cs.detach();
}

// ── timing: stamped intents, changed-only poses ─────────────────────────────

TEST(HolonSyncTest, IntentStampRoundTrips) {
  cvc::app app;
  avatar_intent in{vec3{1.0, 0.0, 0.0}};
  in.yaw = 0.75;
  cvc::ariadne::write_intent(app, "r", "A", in, 1234);
  avatar_intent out;
  std::uint64_t stamp = 0;
  ASSERT_TRUE(cvc::ariadne::read_intent(app, "r", "A", out, &stamp));
  EXPECT_EQ(stamp, 1234u);
  EXPECT_DOUBLE_EQ(out.yaw, 0.75);
  cvc::ariadne::write_intent(app, "r", "A", in); // untimed
  ASSERT_TRUE(cvc::ariadne::read_intent(app, "r", "A", out, &stamp));
  EXPECT_EQ(stamp, 0u);
}

TEST(HolonSyncTest, ProposeIntentStampsTheNextTick) {
  cvc::app app;
  holon h(holon_role::client);
  h.add_participant("A", /*local=*/true);
  for (int i = 0; i < 5; ++i)
    h.advance(h.clock().fixed_dt());
  const std::uint64_t proposed =
      cvc::ariadne::propose_intent(h, app, "r", "A", avatar_intent{vec3{1.0, 0.0, 0.0}});
  avatar_intent out;
  std::uint64_t stamp = 0;
  ASSERT_TRUE(cvc::ariadne::read_intent(app, "r", "A", out, &stamp));
  EXPECT_EQ(stamp, h.tick() + 1);
  EXPECT_EQ(proposed, stamp) << "propose_intent returns its stamp";
}

// read_current_intent refuses an intent stamped before our last proposal (an echo of an earlier
// one) and takes our own, a newer one, or one the document wrote (stamp 0).
TEST(HolonSyncTest, ReadCurrentIntentRefusesAnEchoOfAnEarlierProposal) {
  cvc::app app;
  avatar_intent out;
  EXPECT_FALSE(cvc::ariadne::read_current_intent(app, "r", "A", 0, out)) << "none in the tree";
  const avatar_intent earlier{vec3{1.0, 0.0, 0.0}}, later{vec3{0.0, 1.0, 0.0}};
  cvc::ariadne::write_intent(app, "r", "A", earlier, 10); // what the relay echoes back
  out = later;
  EXPECT_FALSE(cvc::ariadne::read_current_intent(app, "r", "A", 20, out));
  EXPECT_DOUBLE_EQ(out.move.y, 1.0) << "out is untouched";
  cvc::ariadne::write_intent(app, "r", "A", later, 20); // our own proposal
  ASSERT_TRUE(cvc::ariadne::read_current_intent(app, "r", "A", 20, out));
  EXPECT_DOUBLE_EQ(out.move.y, 1.0);
  cvc::ariadne::write_intent(app, "r", "A", earlier, 25); // newer than our proposal
  ASSERT_TRUE(cvc::ariadne::read_current_intent(app, "r", "A", 20, out));
  EXPECT_DOUBLE_EQ(out.move.x, 1.0);
  cvc::ariadne::write_intent(app, "r", "A", later); // the document's (stamp 0)
  ASSERT_TRUE(cvc::ariadne::read_current_intent(app, "r", "A", 20, out));
  EXPECT_DOUBLE_EQ(out.move.y, 1.0);
}

TEST(HolonSyncTest, WritePoseWithAHeartbeatWritesOnlyChanges) {
  cvc::app app;
  body_state s;
  s.position = vec3{1.0, 2.0, 0.0};
  cvc::ariadne::write_pose(app, "r", "A", s, 100, /*heartbeat_ticks=*/120);
  body_state out;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(app, "r", "A", out, tick));
  EXPECT_EQ(tick, 100u);

  // Unchanged, within the heartbeat: nothing is written (not even the tick).
  cvc::ariadne::write_pose(app, "r", "A", s, 150, 120);
  ASSERT_TRUE(cvc::ariadne::read_pose(app, "r", "A", out, tick));
  EXPECT_EQ(tick, 100u);
  // Unchanged, a heartbeat later: the tick is re-stamped.
  cvc::ariadne::write_pose(app, "r", "A", s, 220, 120);
  ASSERT_TRUE(cvc::ariadne::read_pose(app, "r", "A", out, tick));
  EXPECT_EQ(tick, 220u);
  // Moved: the changed leaf and the tick are written.
  s.position.x = 1.5;
  cvc::ariadne::write_pose(app, "r", "A", s, 230, 120);
  ASSERT_TRUE(cvc::ariadne::read_pose(app, "r", "A", out, tick));
  EXPECT_EQ(tick, 230u);
  EXPECT_DOUBLE_EQ(out.position.x, 1.5);
  EXPECT_DOUBLE_EQ(out.position.y, 2.0);
}

// server_pump queues a stamped intent for its tick rather than applying it on read.
TEST(HolonSyncTest, ServerPumpAppliesAStampedIntentAtItsTick) {
  cvc::app app;
  const std::string room = "r";
  cvc::ariadne::conference::room_join(app, room, "A");
  holon server(holon_role::server);
  const double dt = server.clock().fixed_dt();
  cvc::ariadne::server_pump(server, app, room, dt); // registers A (tick 1)
  const vec3 start = server.state("A").position;
  cvc::ariadne::write_intent(app, room, "A", avatar_intent{vec3{2.0, 0.0, 0.0}}, server.tick() + 5);
  for (int i = 0; i < 4; ++i) // ticks 2-5: before the stamp
    cvc::ariadne::server_pump(server, app, room, dt);
  EXPECT_DOUBLE_EQ(server.state("A").position.x, start.x) << "not before its tick";
  cvc::ariadne::server_pump(server, app, room, dt); // tick 6: the stamp
  EXPECT_DOUBLE_EQ(server.state("A").position.x, start.x + 2.0 * dt);
}

// The room scene's objects become physics statics: the floor as a slab under z = 0, the cube as
// a box standing on it.
TEST(HolonSyncTest, RoomStaticsComeFromTheRoomScene) {
  cvc::app app;
  ASSERT_TRUE(cvc::ariadne::conference::seed_default_scene(app, "r"));
  const std::vector<cvc::ariadne::avatar_static> st = cvc::ariadne::room_statics(app, "r");
  ASSERT_EQ(st.size(), 2u);
  const cvc::ariadne::avatar_static &ground = st[0].half_extents.x > 5.0 ? st[0] : st[1];
  const cvc::ariadne::avatar_static &cube = st[0].half_extents.x > 5.0 ? st[1] : st[0];
  EXPECT_DOUBLE_EQ(ground.half_extents.x, 10.0);
  EXPECT_NEAR(ground.center.z + ground.half_extents.z, 0.0, 1e-9) << "the slab's top is the floor";
  EXPECT_DOUBLE_EQ(cube.half_extents.z, 0.5);
  EXPECT_DOUBLE_EQ(cube.center.z, 0.5);
  EXPECT_TRUE(cvc::ariadne::room_holon_config().physics);
}

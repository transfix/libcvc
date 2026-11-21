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
#include <cvc/ariadne/conference/room_avatar.h>
#include <cvc/ariadne/conference/room_entities.h>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_rules.h>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/controls.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h>
#include <cvc/ariadne/section.h>     // section_epoch
#include <cvc/ariadne/world_rules.h> // world_rules
#include <cvc/core/app.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>
#include <vector>

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

// The relay's avatar policy reaches every holon through the pumps -- the relay's own and the
// client's -- and the relay publishes each reserve beside the pose: the client's prediction of its
// own spending stays with the authority's, and what it shows as stamina is the relay's count.
TEST(HolonSyncTest, TheEnergyPolicyAndEachReserveReachTheClient) {
  cvc::app server_app, a_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard as(a_app, "conf", "A");
  for (cvc::state_cluster_shard *s : {&ss, &as}) {
    s->attach();
    tp.register_shard(s);
  }
  const std::string room = "r";
  cvc::ariadne::holon_config cfg;
  cfg.physics = true;
  holon server(holon_role::server, cfg), a(holon_role::client, cfg);
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  cvc::ariadne::conference::seed_default_scene(server_app, room); // something to stand on
  cvc::ariadne::avatar_policy policy;
  policy.energy.reserve_max = 2000.0; // small, so a walk visibly spends it
  cvc::ariadne::conference::write_avatar_policy(server_app, room, policy);
  cvc::ariadne::conference::room_join(a_app, room, "A");
  tp.pump_all();

  const double dt = server.clock().fixed_dt();
  constexpr int kServerEvery = 6;
  cvc::ariadne::server_pump(server, server_app, room, dt);
  tp.pump_all();
  cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  EXPECT_DOUBLE_EQ(server.energy_policy().reserve_max, 2000.0) << "the relay runs the room's";
  EXPECT_DOUBLE_EQ(a.energy_policy().reserve_max, 2000.0) << "and so does the client";
  EXPECT_EQ(a.energy_policy_revision(), server.energy_policy_revision());

  cvc::ariadne::propose_intent(a, a_app, room, "A", avatar_intent{vec3{4.0, 0.0, 0.0}});
  tp.pump_all();
  for (int frame = 1; frame <= 360; ++frame) { // 3 s
    if (frame % kServerEvery == 0)
      cvc::ariadne::server_pump(server, server_app, room, kServerEvery * dt);
    tp.pump_all();
    cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  }
  cvc::ariadne::avatar_energy se, ae;
  ASSERT_TRUE(server.energy("A", se));
  ASSERT_TRUE(a.energy("A", ae));
  EXPECT_LT(se.reserve, 1500.0) << "the walk spent from the authority's reserve";
  double published = 0.0;
  std::uint64_t at_tick = 0;
  ASSERT_TRUE(cvc::ariadne::conference::read_avatar_reserve(a_app, room, "A", published, at_tick))
      << "the relay published it";
  EXPECT_NEAR(published, se.reserve, 0.005 * 2000.0 + 1.0);
  EXPECT_NEAR(ae.reserve, se.reserve, 60.0) << "the prediction keeps up with the authority";

  for (cvc::state_cluster_shard *s : {&ss, &as}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

// The world's rules (WD2) reach every holon through the pumps, the relay's own and the client's,
// from the same leaves. A relay that restarts writes its sections again at revision 1, which the
// client already holds from the old one; their new epoch is what tells them apart. A change while
// the room runs takes effect on the tick it names, in both.
TEST(HolonSyncTest, TheRulesReachEveryHolonAndARestartedRelaysAreTakenAfresh) {
  namespace rc = cvc::ariadne::conference;
  cvc::app server_app, a_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard as(a_app, "conf", "A");
  for (cvc::state_cluster_shard *s : {&ss, &as}) {
    s->attach();
    tp.register_shard(s);
  }
  const std::string room = "r";
  holon server(holon_role::server), a(holon_role::client);
  rc::seed_default_scene(server_app, room);
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  rules.policy.max_speed = 3.0;
  rules.archetypes[0].body.height = 1.9;
  rules.controllers[0].params["walk_speed"] = 1.5;
  rules.physics.gravity = vec3{0.0, 0.0, -3.7};
  rc::write_room_rules(server_app, room, rules);
  rc::room_join(a_app, room, "A");
  tp.pump_all();
  const double dt = server.clock().fixed_dt();
  const auto pump = [&] {
    cvc::ariadne::server_pump(server, server_app, room, dt);
    tp.pump_all();
    cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  };
  pump();
  for (holon *h : {&server, &a}) {
    EXPECT_EQ(h->rules().policy.max_speed, 3.0);
    EXPECT_EQ(h->body().height, 1.9);
    EXPECT_EQ(h->controller().param("walk_speed", 0.0), 1.5);
    EXPECT_EQ(h->rules().physics.gravity.z, -3.7);
  }

  // The relay restarts: the same revision, another epoch.
  cvc::ariadne::avatar_policy p = rules.policy;
  p.max_speed = 2.0;
  const std::uint64_t other = cvc::ariadne::section_epoch(server_app) ^ 1;
  const cvc::ariadne::section_stamp restarted =
      rc::write_avatar_policy(server_app, room, p, 0, other);
  ASSERT_EQ(restarted.rev, a.rules_stamp(cvc::ariadne::rules_section::policy).rev)
      << "the revision the client already holds";
  pump();
  EXPECT_EQ(server.rules().policy.max_speed, 2.0);
  EXPECT_EQ(a.rules().policy.max_speed, 2.0) << "taken afresh by its epoch";

  // A change while the room runs: from the tick it names, in both holons.
  p.max_speed = 5.0;
  const std::uint64_t at = rc::rules_activation_tick(server.tick());
  rc::write_avatar_policy(server_app, room, p, at, other);
  pump();
  for (holon *h : {&server, &a}) {
    EXPECT_EQ(h->rules_at(at - 1).policy.max_speed, 2.0);
    EXPECT_EQ(h->rules_at(at).policy.max_speed, 5.0);
  }

  for (cvc::state_cluster_shard *s : {&ss, &as}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

// The room scene is a revisioned section too: a relay that restarts with another world writes it
// at revision 1, which the client already holds from the old relay, and the epoch is what tells
// them apart. The client's statics come from the new world alone, not from what the old relay left
// in its tree, so its own prediction stands on the new floor before the new relay's samples say so.
TEST(HolonSyncTest, ARestartedRelaysSceneRebuildsTheStatics) {
  namespace rc = cvc::ariadne::conference;
  cvc::app server_app, restarted_app, a_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard ns(restarted_app, "conf", "server2");
  cvc::state_cluster_shard as(a_app, "conf", "A");
  for (cvc::state_cluster_shard *s : {&ss, &as}) {
    s->attach();
    tp.register_shard(s);
  }
  const std::string room = "r";
  cvc::ariadne::holon_config cfg;
  cfg.physics = true;
  holon server(holon_role::server, cfg), a(holon_role::client, cfg);
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  rc::room_world world{rc::default_scene_env(), rc::default_scene_objects()};
  cvc::ariadne::SceneNode pillar = world.objects[1]; // a second box, well off the spawn ring
  pillar.id = "pillar";
  pillar.position[0] = pillar.position[1] = 6.0f;
  world.objects.push_back(pillar);
  const cvc::ariadne::section_stamp first = rc::write_room_world(server_app, room, world);
  ASSERT_EQ(first.rev, 1u);
  rc::room_join(a_app, room, "A");
  tp.pump_all();
  const double dt = server.clock().fixed_dt();
  const auto pump = [&] {
    cvc::ariadne::server_pump(server, server_app, room, dt);
    tp.pump_all();
    cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  };
  for (int frame = 0; frame < 60; ++frame)
    pump();
  for (holon *h : {&server, &a})
    EXPECT_TRUE(h->statics_stamp().same(first));
  EXPECT_EQ(cvc::ariadne::room_statics(a_app, room).size(), 3u) << "ground, ref, pillar";
  ASSERT_TRUE(a.has_body("A"));
  EXPECT_NEAR(a.state("A").position.z, 0.0, 0.05) << "the client's avatar stands on the floor";

  // The relay restarts with another world: no pillar, and its floor a metre lower. Its tree starts
  // empty, so it writes revision 1 again, under another epoch, and never removes the pillar it
  // never knew. Only the client steps, so nothing but its own statics can move its avatar.
  tp.unregister_shard(&ss);
  ss.detach();
  ns.attach();
  tp.register_shard(&ns);
  world.objects.pop_back();
  for (cvc::ariadne::SceneNode &o : world.objects)
    if (o.id == "ground")
      o.position[2] = -1.0f;
  const cvc::ariadne::section_stamp restarted = rc::write_room_world(
      restarted_app, room, world, cvc::ariadne::section_epoch(restarted_app) ^ 1);
  ASSERT_EQ(restarted.rev, a.statics_stamp().rev) << "the revision the client already holds";
  tp.pump_all();
  for (int frame = 0; frame < 120; ++frame)
    cvc::ariadne::client_pump(a, a_app, room, "A", dt);
  EXPECT_TRUE(a.statics_stamp().same(restarted)) << "taken afresh by its epoch";
  EXPECT_EQ(cvc::ariadne::room_statics(a_app, room).size(), 2u)
      << "from the new world alone: the old relay's pillar is not among them";
  EXPECT_NEAR(a.state("A").position.z, -1.0, 0.05) << "and the prediction stands on the new floor";
  holon relay(holon_role::server, cfg);
  cvc::ariadne::server_pump(relay, restarted_app, room, dt);
  EXPECT_TRUE(relay.statics_stamp().same(restarted)) << "the new relay's own holon too";

  for (cvc::state_cluster_shard *s : {&ns, &as}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

// A relay that streams the poses (room_pose_relay) skips the tree's copy on most steps
// (write_poses false); the reserve still goes out on those steps, since no snapshot carries it.
TEST(HolonSyncTest, TheReserveIsWrittenOnStepsThatSkipThePoses) {
  cvc::app server_app;
  const std::string room = "r";
  cvc::ariadne::holon_config cfg;
  cfg.physics = true;
  holon server(holon_role::server, cfg);
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  cvc::ariadne::conference::seed_default_scene(server_app, room);
  cvc::ariadne::avatar_policy policy;
  policy.energy.reserve_max = 2000.0;
  cvc::ariadne::conference::write_avatar_policy(server_app, room, policy);
  cvc::ariadne::conference::room_join(server_app, room, "A");
  cvc::ariadne::write_intent(server_app, room, "A", avatar_intent{vec3{4.0, 0.0, 0.0}});
  const double dt = server.clock().fixed_dt();
  cvc::ariadne::server_pump(server, server_app, room, dt); // registers A and writes its pose
  body_state pose;
  std::uint64_t pose_tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(server_app, room, "A", pose, pose_tick));

  for (int step = 0; step < 120; ++step) // a second's walk, no pose write-back
    cvc::ariadne::server_pump(server, server_app, room, dt, /*write_poses=*/false);
  body_state later;
  std::uint64_t later_tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(server_app, room, "A", later, later_tick));
  EXPECT_EQ(later_tick, pose_tick) << "no pose was written on those steps";
  cvc::ariadne::avatar_energy e;
  ASSERT_TRUE(server.energy("A", e));
  EXPECT_LT(e.reserve, 1900.0) << "the walk spent";
  double published = 0.0;
  std::uint64_t reserve_tick = 0;
  ASSERT_TRUE(cvc::ariadne::conference::read_avatar_reserve(server_app, room, "A", published,
                                                            reserve_tick));
  EXPECT_GT(reserve_tick, pose_tick) << "but the reserve was";
  EXPECT_NEAR(published, e.reserve, 0.005 * 2000.0 + 1.0);
}

// An avatar standing still with a full reserve: once its reserve is published the relay writes no
// more of it -- the re-stamp is for a reserve still drifting from the published one -- and a walk
// publishes again. Each reserve write journals one mutation per leaf that changed.
TEST(HolonSyncTest, AFullReserveAtRestIsNotRestamped) {
  cvc::app server_app;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  ss.attach();
  const std::string room = "r";
  cvc::ariadne::holon_config cfg;
  cfg.physics = true;
  holon server(holon_role::server, cfg);
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  cvc::ariadne::conference::seed_default_scene(server_app, room);
  cvc::ariadne::avatar_policy policy;
  policy.energy.reserve_max = 2000.0;
  cvc::ariadne::conference::write_avatar_policy(server_app, room, policy);
  cvc::ariadne::conference::room_join(server_app, room, "A");
  const auto energy_writes = [&] {
    std::size_t n = 0;
    for (const cvc::state_mutation &m : ss.drain_local())
      n += m.path.find(".energy.") != std::string::npos;
    return n;
  };
  ss.drain_local(); // the policy's own leaves
  const double dt = server.clock().fixed_dt();
  cvc::ariadne::server_pump(server, server_app, room, dt);
  double published = 0.0;
  std::uint64_t at_tick = 0;
  ASSERT_TRUE(
      cvc::ariadne::conference::read_avatar_reserve(server_app, room, "A", published, at_tick));
  EXPECT_DOUBLE_EQ(published, 2000.0) << "published full";
  EXPECT_EQ(energy_writes(), 2u) << "the reserve and its tick, once each";

  for (int step = 0; step < 1200; ++step) // 10 s at rest: two of the old 5 s re-stamps
    cvc::ariadne::server_pump(server, server_app, room, dt, /*write_poses=*/step % 120 == 0);
  EXPECT_EQ(energy_writes(), 0u) << "a full reserve at rest is not re-stamped";
  std::uint64_t still_tick = 0;
  ASSERT_TRUE(
      cvc::ariadne::conference::read_avatar_reserve(server_app, room, "A", published, still_tick));
  EXPECT_EQ(still_tick, at_tick);

  cvc::ariadne::write_intent(server_app, room, "A", avatar_intent{vec3{4.0, 0.0, 0.0}});
  for (int step = 0; step < 120; ++step)
    cvc::ariadne::server_pump(server, server_app, room, dt, /*write_poses=*/false);
  EXPECT_GT(energy_writes(), 0u) << "a walk spends, and that is published";
  ASSERT_TRUE(
      cvc::ariadne::conference::read_avatar_reserve(server_app, room, "A", published, still_tick));
  EXPECT_LT(published, 2000.0);
  EXPECT_GT(still_tick, at_tick);

  // The writer itself: an unchanged reserve re-stamped is its tick alone; nothing new, nothing.
  cvc::ariadne::conference::write_avatar_reserve(server_app, room, "A", published, 900000);
  EXPECT_EQ(energy_writes(), 1u);
  cvc::ariadne::conference::write_avatar_reserve(server_app, room, "A", published, 900000);
  EXPECT_EQ(energy_writes(), 0u);
  cvc::ariadne::conference::write_avatar_reserve(server_app, room, "A", published - 50.0, 900100);
  EXPECT_EQ(energy_writes(), 2u);
  ss.detach();
}

// POSE_HEARTBEAT_S paces the tree's re-stamp of an avatar standing still: about a second by
// default, for clients that keep their clocks by it, and longer for a relay whose pose stream
// carries that heartbeat (arisrv writes the tree's copy each second and re-stamps it every 5 s).
TEST(HolonSyncTest, ThePoseHeartbeatPacesTheTreesRestamp) {
  for (const double heartbeat_s : {1.0, 5.0}) {
    cvc::app server_app;
    cvc::state_cluster_shard ss(server_app, "conf", "server");
    ss.attach();
    const std::string room = "r";
    holon server(holon_role::server, cvc::ariadne::holon_config{}); // kinematic: exactly still
    cvc::ariadne::conference::room_join(server_app, room, "A");
    const double dt = server.clock().fixed_dt();
    cvc::ariadne::server_pump(server, server_app, room, dt, true, heartbeat_s);
    ss.drain_local();
    std::size_t ticks = 0, others = 0;
    for (int step = 1; step <= 1200; ++step) { // 10 s, the tree's copy written every second
      cvc::ariadne::server_pump(server, server_app, room, dt, step % 120 == 0, heartbeat_s);
      for (const cvc::state_mutation &m : ss.drain_local()) {
        const std::size_t at = m.path.find(".pose.");
        if (at == std::string::npos)
          continue;
        if (m.path.compare(at, std::string::npos, ".pose.server_tick") == 0)
          ++ticks;
        else
          ++others;
      }
    }
    EXPECT_EQ(ticks, heartbeat_s == 1.0 ? 10u : 2u) << "heartbeat " << heartbeat_s << " s";
    EXPECT_EQ(others, 0u) << "an avatar standing still writes only its tick";
    ss.detach();
  }
}

// A client joining a room where a peer stands still: the pose stream brings current samples, the
// room tree a pose stamped at the peer's last heartbeat. The stale stamp is no clock reading, so
// the join re-aligns the clock once, however old the tree's tick is.
TEST(HolonSyncTest, AJoinerTakesNoClockReadingFromAStaleTreeTick) {
  cvc::app app;
  const std::string room = "r";
  holon client(holon_role::client, cvc::ariadne::holon_config{}); // kinematic: the clock alone
  client.set_lead_ticks(14);
  cvc::ariadne::conference::room_join(app, room, "A");
  cvc::ariadne::conference::room_join(app, room, "B");
  // The tree as the relay last wrote it: B still since tick 4400 (a 5 s heartbeat ago), A new.
  body_state a, b;
  a.position = vec3{0.0, 0.0, 0.0};
  b.position = vec3{4.0, 0.0, 0.0};
  cvc::ariadne::write_pose(app, room, "B", b, 4400);
  cvc::ariadne::write_pose(app, room, "A", a, 5000);
  // The stream's sample of A lands first (a frame applies the snapshots before client_pump).
  client.add_participant("A", /*local=*/true);
  cvc::ariadne::apply_pose_samples(client, {cvc::ariadne::pose_sample{"A", a}}, 5000);
  ASSERT_EQ(client.clock_realigns(), 1u) << "the join";
  const double dt = client.clock().fixed_dt();
  cvc::ariadne::client_pump(client, app, room, "A", dt); // takes B's pose from the tree
  EXPECT_TRUE(client.pose_known("B")) << "B is placed from the tree";
  // Then B's current sample on the stream.
  cvc::ariadne::apply_pose_samples(client, {cvc::ariadne::pose_sample{"B", b}}, 5002);
  cvc::ariadne::client_pump(client, app, room, "A", dt);
  EXPECT_EQ(client.clock_realigns(), 1u) << "no seek back to B's tree tick, nor forward again";
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

// A turn in place (mouse look while standing) is a yaw-only intent: the live room's holons, Jolt
// and all, turn A at once in its own prediction, at the relay, and in what B shows of A.
TEST(HolonSyncTest, ATurnInPlaceReachesTheRelayAndThePeer) {
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
  const cvc::ariadne::holon_config cfg = cvc::ariadne::room_holon_config();
  holon server(holon_role::server, cfg), a(holon_role::client, cfg), b(holon_role::client, cfg);
  if (!server.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  cvc::ariadne::conference::seed_default_scene(server_app, room); // something to stand on
  cvc::ariadne::conference::room_join(a_app, room, "A");
  cvc::ariadne::conference::room_join(b_app, room, "B");
  tp.pump_all();
  const double dt = server.clock().fixed_dt();
  constexpr int kServerEvery = 6;
  const auto yaw_of = [](const cvc::physics::quat &q) { return 2.0 * std::atan2(q.z, q.w); };
  const auto run = [&](int frames) {
    for (int frame = 1; frame <= frames; ++frame) {
      if (frame % kServerEvery == 0)
        cvc::ariadne::server_pump(server, server_app, room, kServerEvery * dt);
      tp.pump_all();
      cvc::ariadne::client_pump(a, a_app, room, "A", dt);
      cvc::ariadne::client_pump(b, b_app, room, "B", dt);
    }
  };
  run(120); // joined, posed and settled on the floor
  ASSERT_TRUE(b.has_participant("A"));
  const vec3 start = server.state("A").position;
  // A quarter turn in 0.1 rad steps, one proposal every 6 quanta (20 Hz), never walking.
  double yaw = 0.0;
  for (int step = 0; step < 16; ++step) {
    yaw += 0.1;
    avatar_intent turn;
    turn.yaw = yaw;
    cvc::ariadne::propose_intent(a, a_app, room, "A", turn);
    run(6);
    EXPECT_NEAR(yaw_of(a.display_pose("A").rotation), yaw, 1e-6)
        << "A turns at once, step " << step;
  }
  run(120); // the relay's next poses, and B's playout delay
  EXPECT_NEAR(yaw_of(server.state("A").rotation), yaw, 1e-6) << "the relay turned A";
  body_state tree;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(b_app, room, "A", tree, tick));
  EXPECT_NEAR(yaw_of(tree.rotation), yaw, 1e-6) << "its pose reached B";
  EXPECT_NEAR(yaw_of(b.display_pose("A").rotation), yaw, 1e-3) << "and B shows A turned";
  const vec3 end = server.state("A").position;
  EXPECT_LT(std::hypot(end.x - start.x, end.y - start.y), 0.01) << "turning is not walking";

  for (cvc::state_cluster_shard *s : {&ss, &as, &bs}) {
    tp.unregister_shard(s);
    s->detach();
  }
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

// The drive axes and the jump on the wire (section 18.6): a participant's intent carries them,
// shares for the drive axes ("nan" for none), and the jump held or not.
TEST(HolonSyncTest, TheDriveAxesAndTheJumpRideTheIntent) {
  cvc::app app;
  const std::string room = "garage";
  avatar_intent in;
  in.steer = -0.25;
  in.throttle = -1.0;
  in.brake = 0.5;
  in.jump = true;
  cvc::ariadne::write_intent(app, room, "ann", in, 42);
  avatar_intent out;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_intent(app, room, "ann", out, &tick));
  EXPECT_EQ(tick, 42u);
  EXPECT_DOUBLE_EQ(out.steer, -0.25);
  EXPECT_DOUBLE_EQ(out.throttle, -1.0);
  EXPECT_DOUBLE_EQ(out.brake, 0.5);
  EXPECT_TRUE(out.jump);
  cvc::ariadne::write_intent(app, room, "ann", avatar_intent{}, 43);
  ASSERT_TRUE(cvc::ariadne::read_intent(app, room, "ann", out, &tick));
  EXPECT_TRUE(std::isnan(out.steer) && std::isnan(out.throttle) && std::isnan(out.brake))
      << "a walker's intent has none";
  EXPECT_FALSE(out.jump);

  // The holon's veto holds the shares to their ranges.
  holon h(holon_role::server);
  avatar_intent wild;
  wild.steer = 3.0;
  wild.throttle = -2.0;
  wild.brake = -1.0;
  const avatar_intent held = h.sanitize_intent(wild);
  EXPECT_DOUBLE_EQ(held.steer, 1.0);
  EXPECT_DOUBLE_EQ(held.throttle, -1.0);
  EXPECT_DOUBLE_EQ(held.brake, 0.0);
}

// Asking to get in (section 18.6): each ask has the next seq, the server answers each once in
// drive_status, and asking again after a refusal is a new ask. With no world attached the server
// refuses, and says why.
TEST(HolonSyncTest, AnAskToDriveIsAnsweredOnce) {
  cvc::app app;
  const std::string room = "garage";
  cvc::ariadne::conference::room_join(app, room, "ann");
  holon server(holon_role::server, cvc::ariadne::room_holon_config());
  cvc::ariadne::server_pump(server, app, room, 1.0 / 60.0);
  EXPECT_EQ(cvc::ariadne::read_drive_status(app, room, "ann").seq, 0u) << "no ask, no answer";
  EXPECT_EQ(cvc::ariadne::request_drive(app, room, "ann", 7), 1u);
  cvc::ariadne::server_pump(server, app, room, 1.0 / 60.0);
  cvc::ariadne::drive_status st = cvc::ariadne::read_drive_status(app, room, "ann");
  EXPECT_EQ(st.seq, 1u);
  EXPECT_FALSE(st.ok);
  EXPECT_NE(st.why.find("no world"), std::string::npos) << st.why;
  EXPECT_EQ(cvc::ariadne::read_driving(app, room, "ann"), 0u);
  // Answered once: the status stays as it is while the ask does.
  cvc::state &status =
      app.root()(cvc::ariadne::conference::room_participant_path(room, "ann") + ".drive_status");
  status.value(std::string("1 refused: kept"));
  cvc::ariadne::server_pump(server, app, room, 1.0 / 60.0);
  EXPECT_EQ(cvc::ariadne::read_drive_status(app, room, "ann").why, "kept");
  // A new ask, and getting out, which needs nothing.
  EXPECT_EQ(cvc::ariadne::request_drive(app, room, "ann", 0), 2u);
  cvc::ariadne::server_pump(server, app, room, 1.0 / 60.0);
  st = cvc::ariadne::read_drive_status(app, room, "ann");
  EXPECT_EQ(st.seq, 2u);
  EXPECT_TRUE(st.ok);
}

// The control schemes (section 18.6): walk, car and boat, each with its own bindings; a car brakes
// and reverses on keys of their own, a boat has no brake. The resolver ramps the steering, fires
// a hold once per press, and does not let the E that got you in get you straight back out.
TEST(ControlSchemes, EachSchemeHasItsOwnBindings) {
  using cvc::ariadne::control_resolver;
  using cvc::ariadne::control_scheme;
  const control_scheme walk = cvc::ariadne::default_control_scheme("walk");
  const control_scheme car = cvc::ariadne::default_control_scheme("car");
  const control_scheme boat = cvc::ariadne::default_control_scheme("boat");
  ASSERT_EQ(walk.id, "walk");
  ASSERT_NE(walk.find("jump"), nullptr);
  EXPECT_EQ(walk.find("jump")->binding.keys, (std::vector<std::string>{"Space"}));
  EXPECT_EQ(walk.find("strafe_left")->binding.keys, (std::vector<std::string>{"A", "Left"}));
  EXPECT_EQ(car.find("throttle")->binding.keys, (std::vector<std::string>{"W", "Up"}));
  EXPECT_EQ(car.find("reverse")->binding.keys, (std::vector<std::string>{"S", "Down"}));
  EXPECT_EQ(car.find("brake")->binding.keys, (std::vector<std::string>{"Space"}));
  EXPECT_EQ(car.find("strafe_left"), nullptr) << "a car can't strafe";
  EXPECT_EQ(boat.find("brake"), nullptr) << "a boat has no brake";
  EXPECT_NE(boat.find("reverse"), nullptr);
  EXPECT_DOUBLE_EQ(car.find("interact")->binding.hold, 0.4);
  EXPECT_TRUE(cvc::ariadne::default_control_scheme("hovercraft").id.empty());

  std::set<std::string> down;
  const control_resolver::held_fn held = [&](const std::string &k) { return down.count(k) != 0; };
  control_resolver r(walk);
  // E held: nothing until 0.4 s, then once.
  down = {"E"};
  r.update(held, 0.2);
  EXPECT_FALSE(r.fired("interact"));
  EXPECT_NEAR(r.progress("interact"), 0.5, 1e-9);
  r.update(held, 0.25);
  EXPECT_TRUE(r.fired("interact"));
  r.update(held, 1.0);
  EXPECT_FALSE(r.fired("interact")) << "once per press";
  // In the car now, E still held: let go first.
  r.set_scheme(car);
  r.update(held, 1.0);
  EXPECT_FALSE(r.fired("interact"));
  EXPECT_DOUBLE_EQ(r.progress("interact"), 0.0);
  down.clear();
  r.update(held, 0.1);
  down = {"E"};
  r.update(held, 0.3);
  EXPECT_FALSE(r.fired("interact")) << "let go before the hold: nothing";
  down.clear();
  r.update(held, 0.1);
  EXPECT_DOUBLE_EQ(r.progress("interact"), 0.0) << "and the hold starts over";
  // The steering ramps to the lock over 0.25 s; the throttle is at once.
  down = {"A", "W"};
  r.update(held, 0.1);
  avatar_intent in;
  r.drive_axes(in);
  EXPECT_NEAR(in.steer, 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(in.throttle, 1.0);
  EXPECT_DOUBLE_EQ(in.brake, 0.0);
  r.update(held, 0.5);
  r.drive_axes(in);
  EXPECT_DOUBLE_EQ(in.steer, 1.0);
  down = {"S", "Space"};
  r.update(held, 0.1);
  r.drive_axes(in);
  EXPECT_NEAR(in.steer, 0.6, 1e-9) << "back toward straight as it ramped out";
  EXPECT_DOUBLE_EQ(in.throttle, -1.0) << "reverse";
  EXPECT_DOUBLE_EQ(in.brake, 1.0) << "and the brake, a key of its own";
  // A rebinding takes effect at once.
  r.set_keys("brake", {"B"});
  r.update(held, 0.1);
  r.drive_axes(in);
  EXPECT_DOUBLE_EQ(in.brake, 0.0);
}

// A gamepad plays the same actions (OW6b): walking on the left stick, looking on the right,
// jumping on A, getting in and out on X held; driving, the left stick steers, RT is the throttle
// and LT the brake (a boat's reverse), LB a car's reverse.
TEST(ControlSchemes, AGamepadPlaysTheSameActions) {
  using cvc::ariadne::control_resolver;
  using cvc::ariadne::control_scheme;
  const control_scheme walk = cvc::ariadne::default_control_scheme("walk");
  const control_scheme car = cvc::ariadne::default_control_scheme("car");
  const control_scheme boat = cvc::ariadne::default_control_scheme("boat");
  EXPECT_EQ(walk.find("forward")->binding.pad, (std::vector<std::string>{"LeftY-"}));
  EXPECT_EQ(walk.find("look_right")->binding.pad, (std::vector<std::string>{"RightX+"}));
  EXPECT_EQ(walk.find("jump")->binding.pad, (std::vector<std::string>{"A"}));
  EXPECT_EQ(walk.find("interact")->binding.pad, (std::vector<std::string>{"X"}));
  EXPECT_TRUE(walk.find("look_up")->binding.keys.empty()) << "the mouse looks on its own";
  EXPECT_EQ(car.find("steer_left")->binding.pad, (std::vector<std::string>{"LeftX-"}));
  EXPECT_EQ(car.find("throttle")->binding.pad, (std::vector<std::string>{"RT"}));
  EXPECT_EQ(car.find("brake")->binding.pad, (std::vector<std::string>{"LT"}));
  EXPECT_EQ(car.find("reverse")->binding.pad, (std::vector<std::string>{"LB"}));
  EXPECT_EQ(boat.find("reverse")->binding.pad, (std::vector<std::string>{"LT"}));
  EXPECT_NE(car.find("look_left"), nullptr) << "and looks around while it drives";

  std::map<std::string, double> pad;
  const control_resolver::pad_fn read = [&](const std::string &n) {
    const auto it = pad.find(n);
    return it == pad.end() ? 0.0 : it->second;
  };
  const control_resolver::held_fn none = [](const std::string &) { return false; };
  control_resolver r(walk);
  // A resting stick's drift is nothing; past the deadzone, the rest of its travel.
  pad = {{"LeftY", -0.1}, {"LeftX", 0.1}};
  r.update(none, 1.0 / 60.0, read);
  double f = 1.0, right = 1.0;
  r.walk_axes(f, right);
  EXPECT_DOUBLE_EQ(f, 0.0);
  EXPECT_DOUBLE_EQ(right, 0.0);
  EXPECT_FALSE(r.from_pad()) << "nothing pressed yet";
  pad = {{"LeftY", -1.0}, {"LeftX", -0.575}};
  r.update(none, 1.0 / 60.0, read);
  r.walk_axes(f, right);
  EXPECT_DOUBLE_EQ(f, 1.0) << "pushed forward (Y is down)";
  EXPECT_NEAR(right, -0.5, 1e-9) << "and half way left, past the deadzone";
  EXPECT_TRUE(r.from_pad());
  // The right stick looks; A jumps.
  pad = {{"RightX", 1.0}, {"RightY", -0.575}, {"A", 1.0}};
  r.update(none, 1.0 / 60.0, read);
  double lx = 0.0, ly = 0.0;
  r.look_axes(lx, ly);
  EXPECT_DOUBLE_EQ(lx, 1.0);
  EXPECT_NEAR(ly, 0.5, 1e-9) << "pushed up looks up";
  EXPECT_TRUE(r.held("jump"));
  // X held gets in, as E does.
  pad = {{"X", 1.0}};
  r.update(none, 0.2, read);
  EXPECT_NEAR(r.progress("interact"), 0.5, 1e-9);
  r.update(none, 0.25, read);
  EXPECT_TRUE(r.fired("interact"));
  // In the car, X still held: let go first, as with E.
  r.set_scheme(car);
  r.update(none, 1.0, read);
  EXPECT_FALSE(r.fired("interact"));
  // The stick steers as it is pushed, with no ramp; the triggers are the throttle and the brake.
  pad = {{"LeftX", 1.0}, {"RT", 0.62}, {"LT", 0.24}};
  r.update(none, 1.0 / 60.0, read);
  avatar_intent in;
  r.drive_axes(in);
  EXPECT_DOUBLE_EQ(in.steer, -1.0) << "right at once";
  EXPECT_NEAR(in.throttle, (0.62 - 0.05) / 0.95, 1e-9);
  EXPECT_NEAR(in.brake, (0.24 - 0.05) / 0.95, 1e-9);
  pad = {{"LB", 1.0}};
  r.update(none, 1.0 / 60.0, read);
  r.drive_axes(in);
  EXPECT_DOUBLE_EQ(in.throttle, -1.0) << "LB is a car's reverse";
  EXPECT_DOUBLE_EQ(in.brake, 0.0);
  // A key and the pad together: the larger; a key held makes the keys the input in use.
  std::set<std::string> down = {"D"};
  const control_resolver::held_fn held = [&](const std::string &k) { return down.count(k) != 0; };
  pad = {{"LeftX", -0.575}};
  r.update(held, 0.1, read);
  r.drive_axes(in);
  EXPECT_NEAR(in.steer, 0.5 - 0.4, 1e-9) << "the stick's half left against D's ramp";
  EXPECT_FALSE(r.from_pad());
  // The boat: LT is its reverse.
  r.set_scheme(boat);
  pad = {{"LT", 1.0}};
  r.update(none, 1.0 / 60.0, read);
  r.drive_axes(in);
  EXPECT_DOUBLE_EQ(in.throttle, -1.0);
  EXPECT_DOUBLE_EQ(in.brake, 0.0);
  // Rebound: an inverted stick, read the other way.
  r.set_pad("throttle", {"RightY-"});
  pad = {{"RightY", -1.0}};
  r.update(none, 1.0 / 60.0, read);
  EXPECT_DOUBLE_EQ(r.value("throttle"), 1.0);
  cvc::ariadne::control_binding b;
  b.pad = {"RightY-"};
  b.invert = true;
  EXPECT_DOUBLE_EQ(cvc::ariadne::pad_reading(b, read), 0.0) << "inverted: pulled back reads it";
  pad = {{"RightY", 1.0}};
  EXPECT_DOUBLE_EQ(cvc::ariadne::pad_reading(b, read), 1.0);
  EXPECT_DOUBLE_EQ(cvc::ariadne::pad_reading(b, {}), 0.0) << "no gamepad";
}

// While the relay says we drive (section 18.6), our pose is the vehicle's, which we do not
// predict: client_pump plays it out from the relay's samples as a peer's, and predicts it again
// once we are out.
TEST(HolonSyncTest, OurOwnPoseIsPlayedOutWhileWeDrive) {
  cvc::app app;
  const std::string room = "garage";
  cvc::ariadne::conference::room_join(app, room, "ann");
  body_state at;
  at.position = vec3{1.0, 2.0, 0.0};
  cvc::ariadne::write_pose(app, room, "ann", at, 100);
  holon client(holon_role::client);
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0);
  ASSERT_TRUE(client.has_participant("ann"));
  EXPECT_TRUE(client.is_local("ann")) << "walking: predicted";
  cvc::state &driving =
      app.root()(cvc::ariadne::conference::room_participant_path(room, "ann") + ".driving");
  driving.value(std::string("5"));
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0);
  EXPECT_TRUE(client.has_participant("ann"));
  EXPECT_FALSE(client.is_local("ann")) << "driving: played out";
  driving.value(std::string("0"));
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0);
  EXPECT_TRUE(client.is_local("ann")) << "out: predicted again";
}

// The vehicle the prompt names (section 18.6): the nearest drivable one within its reach of us,
// one ahead of us before one behind; never one that is not drivable, nor one out of reach.
TEST(HolonSyncTest, TheDriveTargetIsTheNearestDrivableVehicleInReach) {
  namespace conf = cvc::ariadne::conference;
  cvc::app app;
  const std::string room = "garage";
  const auto vehicle = [](std::uint64_t id, bool drivable) {
    conf::room_entity e;
    e.id = id;
    e.archetype = "humvee";
    e.drivable = drivable;
    e.reach = 3.0;
    e.size = {4.0, 2.0, 1.5};
    return e;
  };
  conf::write_room_entities(
      app, room, {vehicle(1, true), vehicle(2, true), vehicle(3, false), vehicle(4, true)});
  cvc::ariadne::entity_playout playout;
  const auto place = [&](std::uint64_t id, double x, double y) {
    body_state s;
    s.position = vec3{x, y, 0.75};
    EXPECT_TRUE(playout.apply(conf::entity_pid(id), s, 10));
  };
  place(1, 4.0, 0.0);  // ahead of us (+x), its box from 2 m
  place(2, -3.5, 0.0); // behind us, its box from 1.5 m: nearer, but behind
  place(3, 0.0, 1.5);  // right beside us, not drivable
  place(4, 20.0, 0.0); // drivable, out of reach
  playout.pump(app, room, 1.0 / 60.0);
  double gap = 0.0;
  EXPECT_EQ(cvc::ariadne::drive_target(playout, vec3{0.0, 0.0, 0.0}, 0.0, &gap), 1u)
      << "the one ahead";
  EXPECT_NEAR(gap, 2.0, 0.05);
  EXPECT_EQ(cvc::ariadne::drive_target(playout, vec3{0.0, 0.0, 0.0}, 3.14159265, &gap), 2u)
      << "turned round, the other";
  EXPECT_EQ(cvc::ariadne::drive_target(playout, vec3{40.0, 40.0, 0.0}, 0.0), 0u) << "none in reach";
}

// The same world on a client (section 18.5, OW5): with the room's entities, client_pump holds each
// vehicle it shows in the holon's world where it is shown -- not one we drive, nor the one that is
// our body -- and lets go of one the room no longer lists.
TEST(HolonSyncTest, TheClientPumpHoldsTheVehiclesWeShow) {
  namespace conf = cvc::ariadne::conference;
  cvc::app app;
  const std::string room = "garage";
  conf::seed_default_scene(app, room);
  conf::room_join(app, room, "ann");
  body_state feet;
  feet.position = vec3{-6.0, 5.0, 0.0};
  cvc::ariadne::write_pose(app, room, "ann", feet, 100);
  const auto vehicle = [](std::uint64_t id) {
    conf::room_entity e;
    e.id = id;
    e.archetype = "humvee";
    e.size = {4.5, 2.0, 1.5};
    return e;
  };
  conf::room_entity driven = vehicle(2), body = vehicle(3);
  driven.driver = "ann";
  body.avatar = "ann";
  conf::write_room_entities(app, room, {vehicle(1), driven, body});
  cvc::ariadne::entity_playout playout;
  for (std::uint64_t id : {1, 2, 3}) {
    body_state s;
    s.position = vec3{4.0 * static_cast<double>(id), 0.0, 0.75};
    s.linear_velocity = vec3{1.0, 0.0, 0.0};
    ASSERT_TRUE(playout.apply(conf::entity_pid(id), s, 100));
  }
  holon client(holon_role::client, cvc::ariadne::room_holon_config());
  if (!client.physics())
    GTEST_SKIP() << "no Jolt backend in this build";
  playout.pump(app, room, 1.0 / 60.0);
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_EQ(client.held_entities(), std::vector<std::uint64_t>{1})
      << "not the one ann drives, nor the one that is ann";
  body_state box;
  ASSERT_TRUE(client.held_entity_state(1, box));
  EXPECT_NEAR(box.position.x, 4.0, 0.2) << "where the playout shows it";

  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0);
  EXPECT_TRUE(client.held_entities().empty()) << "no entities given: none held";
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_EQ(client.held_entities(), std::vector<std::uint64_t>{1});
  conf::write_room_entities(app, room, {driven, body}); // the room's vehicle 1 is gone
  playout.pump(app, room, 1.0 / 60.0);
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_TRUE(client.held_entities().empty()) << "gone from the room, let go";
}

// The vehicle we drive, predicted (section 18.5, OW5b): with the room's entities saying how it
// runs (the kinematic plant's numbers, room_entity::drive), client_pump keeps us local and the
// holon predicts it; a vehicle on another plant is played out as before; out, the walker again.
TEST(HolonSyncTest, TheClientPumpPredictsTheVehicleWeDrive) {
  namespace conf = cvc::ariadne::conference;
  cvc::app app;
  const std::string room = "garage";
  conf::room_join(app, room, "ann");
  body_state feet;
  feet.position = vec3{1.0, 2.0, 0.0};
  cvc::ariadne::write_pose(app, room, "ann", feet, 100);
  conf::room_entity kinematic, jolt;
  kinematic.id = 5;
  kinematic.plant = "kinematic";
  kinematic.drive.wheelbase = 3.3;
  kinematic.drive.lock = 0.45;
  kinematic.drive.vmax = 9.0;
  kinematic.drive.a_max = 2.4;
  kinematic.drive.rear_x = -1.65;
  kinematic.drive.tick_steps = 7;
  kinematic.drive.nsub = 5;
  kinematic.drive.phase = 2;
  jolt.id = 6;
  jolt.plant = "jolt";
  conf::write_room_entities(app, room, {kinematic, jolt});
  {
    const std::vector<conf::room_entity> back = conf::read_room_entities(app, room);
    ASSERT_EQ(back.size(), 2u);
    EXPECT_EQ(back[0].drive, kinematic.drive) << "the numbers round-trip";
    EXPECT_FALSE(back[1].drive.valid());
  }
  cvc::ariadne::entity_playout playout;
  playout.pump(app, room, 1.0 / 60.0);
  holon client(holon_role::client);
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_TRUE(client.is_local("ann"));
  EXPECT_FALSE(client.predicts_vehicle("ann")) << "walking";

  cvc::state &driving = app.root()(conf::room_participant_path(room, "ann") + ".driving");
  driving.value(std::string("5"));
  body_state car;
  car.position = vec3{4.0, 2.0, 1.2};
  car.linear_velocity = vec3{3.0, 0.0, 0.0};
  cvc::ariadne::write_pose(app, room, "ann", car, 200);
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_TRUE(client.is_local("ann")) << "driving a vehicle we can run: still ours";
  EXPECT_TRUE(client.predicts_vehicle("ann"));
  EXPECT_NEAR(client.state("ann").position.x, 4.0, 0.2) << "placed by the vehicle's pose";
  const double x0 = client.state("ann").position.x;
  for (int i = 0; i < 30; ++i)
    cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_GT(client.state("ann").position.x, x0 + 0.5) << "and run on from there";
  // The host draws it where it is predicted (arigl), not where its samples put it.
  playout.apply(conf::entity_pid(5), car, 200);
  playout.pump(app, room, 1.0 / 60.0);
  playout.show_predicted(5, client.display_pose("ann"));
  cvc::physics::pose drawn;
  ASSERT_TRUE(playout.pose(5, drawn));
  EXPECT_DOUBLE_EQ(drawn.position.x, client.display_pose("ann").position.x);
  playout.show_predicted(0, cvc::physics::pose{});
  ASSERT_TRUE(playout.pose(5, drawn));
  EXPECT_LT(drawn.position.x, client.display_pose("ann").position.x - 0.3) << "its samples'";

  driving.value(std::string("6"));
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_FALSE(client.is_local("ann")) << "one on another plant: played out";
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0);
  EXPECT_FALSE(client.is_local("ann")) << "no entities: played out";

  driving.value(std::string("0"));
  cvc::ariadne::client_pump(client, app, room, "ann", 1.0 / 60.0, &playout);
  EXPECT_TRUE(client.is_local("ann"));
  EXPECT_FALSE(client.predicts_vehicle("ann")) << "out: the walker again";
}

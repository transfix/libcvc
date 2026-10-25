/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Room control-plane model (roadmap §4b/§5a), first slice: trust-all self-insert join/roster/leave
// on the SHARED room tree, exercised through the reusable free functions (room_join / room_roster /
// room_leave) that the DSL verbs wrap. The two-node tests mirror the cross-node state-convergence
// harness from state_transport_inproc_test (two apps + two shards + an inproc transport +
// pump_all); no gRPC and no Runtime/backend are needed because the model is pure replicated state.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_net.h>
#include <cvc/ariadne/conference/room_write_gate.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace conf = cvc::ariadne::conference;

namespace {

const conf::room_participant *by_pid(const std::vector<conf::room_participant> &r,
                                     const std::string &pid) {
  for (const conf::room_participant &p : r)
    if (p.pid == pid)
      return &p;
  return nullptr;
}

// Publish what SHARD journaled since its last drain to the transport's other shards, keeping the
// drained mutations in OUT; returns how many there were (what the writes since then cost on the
// wire).
std::size_t publish_drained(cvc::state_transport_inproc &tp, cvc::state_cluster_shard &shard,
                            std::vector<cvc::state_mutation> *out = nullptr) {
  const std::vector<cvc::state_mutation> drained = shard.drain_local();
  for (const cvc::state_mutation &m : drained)
    tp.publish(m);
  if (out)
    *out = drained;
  return drained.size();
}

} // namespace

// The model works standalone (no cluster): a join writes a participant subtree that the roster
// reads back in pid order, and a leave retracts it.
TEST(RoomIntrinsics, RosterReadsLocalJoinsAndLeaves) {
  cvc::app app;
  conf::room_join(app, "standup", "A", "Alice");
  conf::room_join(app, "standup", "B", "Bob");

  std::vector<conf::room_participant> r = conf::room_roster(app, "standup");
  ASSERT_EQ(r.size(), 2u);
  EXPECT_EQ(r[0].pid, "A"); // children() returns names in order, so the roster is pid-ordered
  EXPECT_EQ(r[0].name, "Alice");
  EXPECT_EQ(r[1].pid, "B");
  EXPECT_EQ(r[1].name, "Bob");

  // An unknown room is simply empty, never an error.
  EXPECT_TRUE(conf::room_roster(app, "no-such-room").empty());

  // A (soft) leave drops the participant from the present roster.
  conf::room_leave(app, "standup", "A");
  std::vector<conf::room_participant> r2 = conf::room_roster(app, "standup");
  ASSERT_EQ(r2.size(), 1u);
  EXPECT_EQ(r2[0].pid, "B");
}

// A join on node A replicates into node B's tree (and vice-versa): each sees the full roster after
// a single pump, with names intact — proving the participant subtree writes cross the replication
// boundary (the touch()-before-write idiom in room_join).
TEST(RoomIntrinsics, JoinReplicatesAcrossNodes) {
  cvc::app aA, aB;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tp.register_shard(&sA);
  tp.register_shard(&sB);

  conf::room_join(aA, "standup", sA.local_node_id(), "Alice");
  conf::room_join(aB, "standup", sB.local_node_id(), "Bob");
  tp.pump_all();

  std::vector<conf::room_participant> ra = conf::room_roster(aA, "standup");
  std::vector<conf::room_participant> rb = conf::room_roster(aB, "standup");
  EXPECT_EQ(ra.size(), 2u) << "A should see both participants after replication";
  EXPECT_EQ(rb.size(), 2u) << "B should see both participants after replication";

  ASSERT_NE(by_pid(ra, "A"), nullptr);
  ASSERT_NE(by_pid(ra, "B"), nullptr);
  EXPECT_EQ(by_pid(ra, "B")->name, "Bob") << "B's profile replicated into A";
  ASSERT_NE(by_pid(rb, "A"), nullptr);
  ASSERT_NE(by_pid(rb, "B"), nullptr);
  EXPECT_EQ(by_pid(rb, "A")->name, "Alice") << "A's profile replicated into B";

  tp.unregister_shard(&sA);
  tp.unregister_shard(&sB);
  sA.detach();
  sB.detach();
}

// A leave retracts the participant on the leaving node, and the retraction replicates to the peer.
TEST(RoomIntrinsics, LeaveRetractsParticipantAcrossNodes) {
  cvc::app aA, aB;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tp.register_shard(&sA);
  tp.register_shard(&sB);

  conf::room_join(aA, "standup", "A", "Alice");
  conf::room_join(aB, "standup", "B", "Bob");
  tp.pump_all();
  ASSERT_EQ(conf::room_roster(aB, "standup").size(), 2u);

  conf::room_leave(aA, "standup", "A");
  tp.pump_all();

  std::vector<conf::room_participant> ra = conf::room_roster(aA, "standup");
  ASSERT_EQ(ra.size(), 1u) << "A retracted its own participant";
  EXPECT_EQ(ra[0].pid, "B");

  std::vector<conf::room_participant> rb = conf::room_roster(aB, "standup");
  ASSERT_EQ(rb.size(), 1u) << "A's leave replicated to B";
  EXPECT_EQ(rb[0].pid, "B");

  tp.unregister_shard(&sA);
  tp.unregister_shard(&sB);
  sA.detach();
  sB.detach();
}

// A participant's voice.talking leaf costs one mutation per change: writing the value it already
// holds journals nothing, and a flip journals the new value alone (not a touch that re-sends the
// old one first). A is a client whose shard is rooted at the room tree, B a whole-tree relay; the
// leaf reaches B either way.
TEST(RoomIntrinsics, VoiceLeafRoundTripsChangedOnly) {
  cvc::app aA, aB;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard sA(aA, "C", "A", "conf");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tp.register_shard(&sA);
  tp.register_shard(&sB);
  const std::string leaf = conf::room_participant_path("standup", "A") + ".voice.talking";

  // Never written: absent, and the out-parameter is left alone.
  bool talking = true;
  EXPECT_FALSE(conf::read_participant_voice(aA, "standup", "A", talking));
  EXPECT_TRUE(talking);

  // The first write, twice: one mutation, carrying the value.
  conf::write_participant_voice(aA, "standup", "A", true);
  conf::write_participant_voice(aA, "standup", "A", true);
  std::vector<cvc::state_mutation> sent;
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].path, leaf);
  EXPECT_EQ(sent[0].string_value, "1");
  talking = false;
  ASSERT_TRUE(conf::read_participant_voice(aA, "standup", "A", talking));
  EXPECT_TRUE(talking);
  talking = false;
  ASSERT_TRUE(conf::read_participant_voice(aB, "standup", "A", talking)) << "replicated to B";
  EXPECT_TRUE(talking);

  // The same value again: nothing to send.
  conf::write_participant_voice(aA, "standup", "A", true);
  EXPECT_EQ(publish_drained(tp, sA), 0u);

  // A flip: the new value, once.
  conf::write_participant_voice(aA, "standup", "A", false);
  conf::write_participant_voice(aA, "standup", "A", false);
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].path, leaf);
  EXPECT_EQ(sent[0].string_value, "0");
  talking = true;
  ASSERT_TRUE(conf::read_participant_voice(aB, "standup", "A", talking));
  EXPECT_FALSE(talking) << "B sees A stop talking";

  // And back: still one per change.
  conf::write_participant_voice(aA, "standup", "A", true);
  EXPECT_EQ(publish_drained(tp, sA), 1u);
  ASSERT_TRUE(conf::read_participant_voice(aB, "standup", "A", talking));
  EXPECT_TRUE(talking);

  // A forced write re-sends an unchanged value: the relay's re-broadcast can echo a stale copy of
  // A's own leaf back, so A's tree may already hold the value A flips to without B having it.
  cvc::state::instance(aB)(leaf).value(std::string("0")); // B lost the last flip
  conf::write_participant_voice(aA, "standup", "A", true, /*force=*/true);
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].path, leaf);
  EXPECT_EQ(sent[0].string_value, "1");
  ASSERT_TRUE(conf::read_participant_voice(aB, "standup", "A", talking));
  EXPECT_TRUE(talking) << "the forced re-send heals B";

  // The leaf sits beside the roster, not in it: no presence, so no participant appears.
  EXPECT_TRUE(conf::room_roster(aB, "standup").empty());
  bool other = true;
  EXPECT_FALSE(conf::read_participant_voice(aB, "standup", "B", other)) << "B never wrote one";
  EXPECT_TRUE(other);

  tp.unregister_shard(&sA);
  tp.unregister_shard(&sB);
  sA.detach();
  sB.detach();
}

// The bound-writes rule (room_write_gate): a participant writes only its own subtree, minus the
// keys the relay owns there, with value, touch and data ops; every other path is the relay's.
TEST(RoomWriteGate, AParticipantWritesOnlyItsOwnSubtree) {
  const conf::room_write_gate gate("standup");
  const std::string alice = conf::room_participant_path("standup", "alice");
  const std::string bob = conf::room_participant_path("standup", "bob");
  using op = cvc::state_mutation_op;

  // What clients write (room_join, write_intent, room_net, room_publish).
  for (const char *key : {"presence.joined", "profile.name", "intent.move.x", "intent.yaw",
                          "intent.tick", "net.rtt_ms", "net.in_kbps", "voice.talking",
                          "streams.camera.endpoint", "streams.camera.token"})
    EXPECT_TRUE(gate.may_write("alice", alice + "." + key, op::set_value)) << key;
  EXPECT_TRUE(gate.may_write("alice", alice + ".intent.move.x", op::touch));
  EXPECT_TRUE(gate.may_write("alice", alice + ".avatar", op::set_data));

  // The relay's keys inside alice's own subtree.
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose.pos.x", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose.server_tick", op::touch));
  EXPECT_FALSE(gate.may_write("alice", alice + ".net.relay.rtt_ms", op::set_value));
  EXPECT_TRUE(gate.may_write("alice", alice + ".net.relayed", op::set_value))
      << "a sibling of net.relay is alice's";
  EXPECT_FALSE(gate.may_write("alice", alice + ".streams.camera.ready", op::set_value))
      << "the SFU relay marks a stream ready";
  EXPECT_FALSE(gate.may_write("alice", alice + ".device", op::set_value))
      << "the relay writes the device tag";

  // Paths the state tree would trim or skip onto one of those keys.
  for (const std::string &p :
       {alice + "..pose.pos.x", alice + ".pose .pos.x", alice + ". pose.pos.x",
        alice + ".\tpose.pos.x", alice + ".net..relay.rtt_ms", alice + ".pose.pos.x.",
        "." + alice + ".intent.move.x", alice + "..intent.move.x"})
    EXPECT_FALSE(gate.may_write("alice", p, op::set_value)) << '"' << p << '"';
  // At most 16 segments and 512 bytes.
  std::string deep = alice;
  while (std::count(deep.begin(), deep.end(), '.') < 15)
    deep += ".a";
  EXPECT_TRUE(gate.may_write("alice", deep, op::set_value)) << "16 segments";
  EXPECT_FALSE(gate.may_write("alice", deep + ".a", op::set_value)) << "17 segments";
  EXPECT_FALSE(gate.may_write("alice", alice + "." + std::string(600, 'a'), op::set_value))
      << "longer than 512 bytes";

  // Everyone else's, and the room's.
  EXPECT_FALSE(gate.may_write("alice", bob + ".intent.move.x", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + "2.intent.move.x", op::set_value))
      << "alice2 is not alice";
  EXPECT_FALSE(
      gate.may_write("alice", "conf.rooms.standup.scene.objects.box.present", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", "conf.rooms.standup.aoi.inner_m", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", "conf.rooms.standup", op::touch));
  EXPECT_FALSE(
      gate.may_write("alice", conf::room_participant_path("other", "alice") + ".x", op::set_value))
      << "another room";
  EXPECT_FALSE(gate.may_write("alice", "ui.docs.ari.conf.self", op::set_value));

  // Ops no client uses, even on its own keys.
  for (op o : {op::delete_subtree, op::reset_node, op::delegate_subtree, op::revoke_delegation,
               op::set_read_only, op::set_hidden, op::set_comment, op::set_property})
    EXPECT_FALSE(gate.may_write("alice", alice + ".intent.move.x", o));
  EXPECT_FALSE(gate.may_write("alice", alice, op::delete_subtree));

  // A pid must be one identifier: "bob.pose" would otherwise own bob's pose.
  EXPECT_FALSE(gate.may_write("bob.pose", bob + ".pose.pos.x", op::set_value));
  EXPECT_FALSE(gate.may_write("", alice + ".intent.move.x", op::set_value));

  // Messages go on the same paths.
  cvc::state_message msg;
  msg.path = alice + ".voice.talking";
  EXPECT_TRUE(gate.admit("alice", msg));
  msg.path = bob + ".voice.talking";
  EXPECT_FALSE(gate.admit("alice", msg));
}

// The binding a gate holds: one connection per node id, freed by its holder only.
TEST(RoomWriteGate, BindsOneConnectionPerNodeId) {
  conf::room_write_gate gate("standup");
  int a = 0, b = 0;
  EXPECT_TRUE(gate.bind("alice", &a));
  EXPECT_TRUE(gate.bind("alice", &a)) << "the holder again";
  EXPECT_FALSE(gate.bind("alice", &b)) << "a second claim";
  EXPECT_TRUE(gate.bind("bob", &b));
  gate.release("alice", &b); // not the holder: no effect
  EXPECT_EQ(gate.holder("alice"), &a);
  gate.release("alice", &a);
  EXPECT_EQ(gate.holder("alice"), nullptr);
  EXPECT_TRUE(gate.bind("alice", &b));
  EXPECT_FALSE(gate.bind("", &a)) << "an empty node id is never bound";
}

// A holder silent for longer than takeover_after() loses its id to the next claim (a peer whose
// old connection never closed, reconnecting); a live one keeps it.
TEST(RoomWriteGate, ASilentHolderIsTakenOver) {
  conf::room_write_gate gate("standup");
  gate.set_takeover_after(std::chrono::milliseconds(1000));
  const auto now_ns = [] {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  int old_conn = 0, fresh_conn = 0;
  std::atomic<std::int64_t> old_rx{now_ns()}, fresh_rx{now_ns()};
  std::atomic<bool> old_displaced{false}, fresh_displaced{false};
  ASSERT_TRUE(gate.bind("alice", &old_conn, &old_rx, &old_displaced));
  EXPECT_FALSE(gate.bind("alice", &fresh_conn, &fresh_rx, &fresh_displaced)) << "a live holder";
  EXPECT_FALSE(old_displaced.load());

  old_rx.store(now_ns() - 5'000'000'000); // the holder has not been heard from for 5 s
  EXPECT_TRUE(gate.bind("alice", &fresh_conn, &fresh_rx, &fresh_displaced)) << "taken over";
  EXPECT_TRUE(old_displaced.load()) << "the old holder is told";
  EXPECT_EQ(gate.holder("alice"), &fresh_conn);
  EXPECT_EQ(gate.total_takeovers(), 1u);
  EXPECT_FALSE(gate.bind("alice", &old_conn, &old_rx, &old_displaced))
      << "and cannot take it back while the new holder is live";
  gate.release("alice", &old_conn); // its close does not free what it no longer holds
  EXPECT_EQ(gate.holder("alice"), &fresh_conn);
}

// A participant's device tag (room_net.h): stable for one device in one room, different in another
// room or for another device, and none without a device id.
TEST(RoomNet, ADeviceTagIsPerRoom) {
  const std::string dev = "0123456789abcdef0123456789abcdef";
  const std::string tag = conf::room_device_tag("standup", dev);
  ASSERT_EQ(tag.size(), 16u);
  EXPECT_EQ(conf::room_device_tag("standup", dev), tag) << "stable";
  EXPECT_NE(conf::room_device_tag("other", dev), tag) << "says nothing about another room";
  EXPECT_NE(conf::room_device_tag("standup", "fedcba9876543210fedcba9876543210"), tag);
  EXPECT_EQ(tag.find(dev.substr(0, 8)), std::string::npos) << "not the id itself";
  EXPECT_EQ(conf::room_device_tag("standup", ""), "");

  cvc::app app;
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"), "");
  conf::write_participant_device(app, "standup", "alice", tag);
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"), tag);
}

// write_link_devices tags a present participant from the accepted link that holds its pid and
// proved its device key, and from no other.
TEST(RoomNet, LinkDevicesTagOnlyVerifiedHolders) {
  const auto link = [](const std::string &pid, bool verified) {
    cvc::state_link_report r;
    r.transport = "ws";
    r.remote_node_id = pid;
    r.remote_device_id = std::string(32, pid[0]);
    r.accepted = true;
    r.holds_id = true;
    r.device_verified = verified;
    return r;
  };
  std::vector<cvc::state_link_report> links = {link("alice", true), link("bob", false),
                                               link("carol", true), link("dave", true)};
  links[2].holds_id = false; // carol's duplicate claim
  links[3].accepted = false; // a link this relay dialed
  cvc::app app;
  conf::write_link_devices(app, "standup", links, {"alice", "bob", "carol", "dave"});
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"),
            conf::room_device_tag("standup", links[0].remote_device_id));
  EXPECT_EQ(conf::read_participant_device(app, "standup", "bob"), "") << "unverified";
  EXPECT_EQ(conf::read_participant_device(app, "standup", "carol"), "") << "not its holder";
  EXPECT_EQ(conf::read_participant_device(app, "standup", "dave"), "") << "not accepted";

  cvc::app absent;
  conf::write_link_devices(absent, "standup", links, {"bob"});
  EXPECT_EQ(conf::read_participant_device(absent, "standup", "alice"), "") << "not present";

  // alice's pid taken over by a link that proved no key: the tag her device proved goes.
  std::vector<cvc::state_link_report> taken = {link("alice", false)};
  taken[0].remote_device_id = std::string(32, 'z');
  conf::write_link_devices(app, "standup", taken, {"alice"});
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"), "") << "taken over";
  conf::write_link_devices(app, "standup", links, {"alice"});
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"),
            conf::room_device_tag("standup", links[0].remote_device_id))
      << "and back when her verified device holds it again";
  conf::write_link_devices(app, "standup", {}, {"alice"});
  EXPECT_EQ(conf::read_participant_device(app, "standup", "alice"),
            conf::room_device_tag("standup", links[0].remote_device_id))
      << "a pid no link holds keeps its tag until it leaves";
}

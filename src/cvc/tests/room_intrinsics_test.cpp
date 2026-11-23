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
#include <cstdio>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_net.h>
#include <cvc/ariadne/conference/room_rungs.h>
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

// The traffic meters (room_net.h) are measured every second but published only when a reading
// moves past its band -- a tenth of the published value, or 2 kb/s or 5 ms if that is more -- as
// two significant figures, with one mutation per change. An idle link writes nothing, so its own
// meters stop adding to what they measure. A is a client rooted at the room tree, B the relay.
TEST(RoomIntrinsics, NetReadingsReplicateOnlyWhenTheyMove) {
  cvc::app aA, aB;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard sA(aA, "C", "A", "conf");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tp.register_shard(&sA);
  tp.register_shard(&sB);
  const std::string net = conf::room_participant_path("standup", "A") + ".net";
  const auto leaf = [&](cvc::app &app, const std::string &path) {
    cvc::state *n = app.root().findDescendant(path);
    return n ? n->value() : std::string("(absent)");
  };

  // The first readings: one mutation each, rounded to two significant figures.
  conf::write_participant_net(aA, "standup", "A", {42.3, 12.34, 3.21});
  std::vector<cvc::state_mutation> sent;
  ASSERT_EQ(publish_drained(tp, sA, &sent), 3u);
  EXPECT_EQ(leaf(aB, net + ".rtt_ms"), "42");
  EXPECT_EQ(leaf(aB, net + ".in_kbps"), "12");
  EXPECT_EQ(leaf(aB, net + ".out_kbps"), "3.2");
  conf::net_sample got;
  ASSERT_TRUE(conf::read_participant_net(aB, "standup", "A", got)) << "replicated to B";
  EXPECT_DOUBLE_EQ(got.rtt_ms, 42.0);
  EXPECT_DOUBLE_EQ(got.in_kbps, 12.0);
  EXPECT_DOUBLE_EQ(got.out_kbps, 3.2);

  // Next second's readings, each inside its band: nothing to send.
  conf::write_participant_net(aA, "standup", "A", {45.9, 13.9, 5.1});
  conf::write_participant_net(aA, "standup", "A", {38.0, 10.1, 1.3});
  EXPECT_EQ(publish_drained(tp, sA), 0u);

  // One moves: that one, once.
  conf::write_participant_net(aA, "standup", "A", {44.0, 30.4, 4.0});
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].path, net + ".in_kbps");
  EXPECT_EQ(sent[0].string_value, "30");

  // A busy link: the band grows with the reading, a tenth of it.
  conf::write_participant_net(aA, "standup", "A", {44.0, 1234.5, 4.0});
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].string_value, "1200");
  conf::write_participant_net(aA, "standup", "A", {44.0, 1310.0, 4.0}); // 110 < 120
  EXPECT_EQ(publish_drained(tp, sA), 0u);
  conf::write_participant_net(aA, "standup", "A", {44.0, 1360.0, 4.0});
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].string_value, "1400");
  conf::write_participant_net(aA, "standup", "A", {44.0, 0.43, 4.0}); // and back to idle
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].string_value, "0.4");

  // Unmeasured is a change, both ways, and staying unmeasured is not.
  conf::write_participant_net(aA, "standup", "A", {-1.0, 0.43, 4.0});
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].string_value, "-1");
  conf::write_participant_net(aA, "standup", "A", {-1.0, 0.43, 4.0});
  EXPECT_EQ(publish_drained(tp, sA), 0u);
  conf::write_participant_net(aA, "standup", "A", {1.5, 0.43, 4.0});
  ASSERT_EQ(publish_drained(tp, sA, &sent), 1u);
  EXPECT_EQ(sent[0].string_value, "1.5");
  ASSERT_TRUE(conf::read_participant_net(aB, "standup", "A", got));
  EXPECT_DOUBLE_EQ(got.rtt_ms, 1.5);
  EXPECT_DOUBLE_EQ(got.in_kbps, 0.4);

  // A forced write re-sends the readings as published, for a relay that lost them (a client back
  // after a reconnect): one touch each, the values unchanged.
  cvc::state::instance(aB)(net + ".in_kbps").value(std::string("")); // B lost them
  sB.drain_local();
  conf::write_participant_net(aA, "standup", "A", {1.6, 0.5, 4.1}, /*force=*/true);
  ASSERT_EQ(publish_drained(tp, sA, &sent), 3u);
  EXPECT_EQ(leaf(aB, net + ".in_kbps"), "0.4") << "the forced re-send heals B";
  EXPECT_EQ(leaf(aA, net + ".rtt_ms"), "1.5") << "and leaves the published readings alone";

  // The relay's meters work the same way: its view of A's link, and its totals.
  conf::write_relay_net(aB, "standup", "A", {2.0, 8.0, 16.0});
  conf::write_server_net(aB, "standup", {8.0, 16.0, 1});
  EXPECT_EQ(sB.drain_local().size(), 6u);
  conf::write_relay_net(aB, "standup", "A", {2.4, 9.1, 17.0});
  conf::write_server_net(aB, "standup", {9.1, 17.0, 1});
  EXPECT_TRUE(sB.drain_local().empty()) << "an idle relay writes nothing";
  conf::write_server_net(aB, "standup", {9.1, 17.0, 2});
  const std::vector<cvc::state_mutation> links = sB.drain_local();
  ASSERT_EQ(links.size(), 1u) << "a link count that changes is one mutation";
  EXPECT_EQ(links[0].string_value, "2");
  conf::server_net totals;
  ASSERT_TRUE(conf::read_server_net(aB, "standup", totals));
  EXPECT_DOUBLE_EQ(totals.out_kbps, 16.0);
  EXPECT_EQ(totals.links, 2);

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
  for (const char *key :
       {"presence.joined", "intent.move.x", "intent.yaw", "intent.tick", "intent.throttle",
        "intent.jump", "drive", "net.rtt_ms", "net.in_kbps", "voice.talking", "video.off",
        "audio.opus", "streams.camera.endpoint", "streams.camera.token"})
    EXPECT_TRUE(gate.may_write("alice", alice + "." + key, op::set_value)) << key;
  EXPECT_TRUE(gate.may_write("alice", alice + ".intent.move.x", op::touch));
  EXPECT_TRUE(gate.may_write("alice", alice + ".avatar", op::set_data));

  // The relay's keys inside alice's own subtree.
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose.pos.x", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + ".pose.server_tick", op::touch));
  EXPECT_FALSE(gate.may_write("alice", alice + ".net.relay.rtt_ms", op::set_value));
  // Its username is the relay's too (client_registry.h): it asks with server.set.
  EXPECT_FALSE(gate.may_write("alice", alice + ".profile.name", op::set_value));
  EXPECT_FALSE(gate.may_write("alice", alice + ".profile.name", op::touch));
  EXPECT_FALSE(gate.may_write("alice", alice + ".net.relay.video", op::set_value))
      << "the relay picks alice's video rung";
  EXPECT_TRUE(gate.may_write("alice", alice + ".net.relayed", op::set_value))
      << "a sibling of net.relay is alice's";
  EXPECT_FALSE(gate.may_write("alice", alice + ".streams.camera.ready", op::set_value))
      << "the SFU relay marks a stream ready";
  EXPECT_FALSE(gate.may_write("alice", alice + ".device", op::set_value))
      << "the relay writes the device tag";
  EXPECT_FALSE(gate.may_write("alice", alice + ".driving", op::set_value))
      << "the relay says what alice drives";
  EXPECT_FALSE(gate.may_write("alice", alice + ".drive_status", op::set_value))
      << "and answers her ask to";

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
  for (op o : {op::reset_node, op::delegate_subtree, op::revoke_delegation, op::set_read_only,
               op::set_hidden, op::set_comment, op::set_property})
    EXPECT_FALSE(gate.may_write("alice", alice + ".intent.move.x", o));

  // A delete of its own nodes replicates (state_sync_adapter journals a removal), but not one of
  // the participant itself, another's, or one that would take a key of the relay's with it.
  EXPECT_TRUE(gate.may_write("alice", alice + ".intent.move.x", op::delete_subtree));
  EXPECT_TRUE(gate.may_write("alice", alice + ".intent", op::delete_subtree));
  EXPECT_TRUE(gate.may_write("alice", alice + ".profile.avatar", op::delete_subtree));
  EXPECT_TRUE(gate.may_write("alice", alice + ".streams.camera.endpoint", op::delete_subtree));
  EXPECT_FALSE(gate.may_write("alice", alice, op::delete_subtree)) << "leaving is the relay's";
  EXPECT_FALSE(gate.may_write("alice", bob + ".intent", op::delete_subtree));
  for (const char *relays : {".pose", ".pose.pos", ".device", ".profile", ".profile.name", ".net",
                             ".net.relay", ".streams", ".streams.camera", ".streams.camera.ready",
                             ".energy", ".driving", ".drive_status", ".archetype_status"})
    EXPECT_FALSE(gate.may_write("alice", alice + relays, op::delete_subtree)) << relays;

  // A pid must be one identifier: "bob.pose" would otherwise own bob's pose.
  EXPECT_FALSE(gate.may_write("bob.pose", bob + ".pose.pos.x", op::set_value));
  EXPECT_FALSE(gate.may_write("", alice + ".intent.move.x", op::set_value));
  // And not a bridged user's: no connection joins as one.
  const std::string bridged = conf::bridged_pid("nmdc", "alice");
  EXPECT_FALSE(gate.may_write(bridged,
                              conf::room_participant_path("standup", bridged) + ".presence.joined",
                              op::set_value));
  EXPECT_FALSE(gate.may_write("alice", conf::room_bridge_path("standup") + ".rev", op::set_value))
      << "the bridged users are the relay's to write";

  // Messages go on the same paths.
  cvc::state_message msg;
  msg.path = alice + ".voice.talking";
  EXPECT_TRUE(gate.admit("alice", msg));
  msg.path = bob + ".voice.talking";
  EXPECT_FALSE(gate.admit("alice", msg));
}

// A client's own profile.name write (client_registry.h) is refused, and handed on as a rename the
// relay may grant, so a client that predates server.set keeps its name. Another participant's,
// or a non-canonical spelling of its own, is only refused.
TEST(RoomWriteGate, AClientsOwnNameWriteIsARenameRequest) {
  conf::room_write_gate gate("standup");
  std::vector<std::pair<std::string, std::string>> asked;
  gate.set_name_requests(
      [&](const std::string &pid, const std::string &name) { asked.emplace_back(pid, name); });
  const std::string alice = conf::room_participant_path("standup", "alice");
  const std::string bob = conf::room_participant_path("standup", "bob");
  const auto write = [](const std::string &path, const std::string &v,
                        cvc::state_mutation_op op = cvc::state_mutation_op::set_value) {
    cvc::state_mutation m;
    m.path = path;
    m.op = op;
    m.string_value = v;
    return m;
  };
  EXPECT_FALSE(gate.admit("alice", write(alice + ".profile.name", "Alice")));
  ASSERT_EQ(asked.size(), 1u);
  EXPECT_EQ(asked[0].first, "alice");
  EXPECT_EQ(asked[0].second, "Alice");
  EXPECT_FALSE(
      gate.admit("alice", write(alice + ".profile.name", "", cvc::state_mutation_op::touch)));
  EXPECT_FALSE(gate.admit("alice", write(bob + ".profile.name", "Bob")));
  EXPECT_FALSE(gate.admit("alice", write(alice + ".profile..name", "A2")));
  EXPECT_TRUE(gate.admit("alice", write(alice + ".presence.joined", "1")));
  EXPECT_EQ(asked.size(), 1u) << "only its own name, set as a value, asks";
}

// A participant is the source of what it may write, so the relay's re-broadcast never sends it back
// (state_inbound_gate::sources); the relay's own keys and other participants' are still sent.
TEST(RoomWriteGate, AParticipantSourcesWhatItWrites) {
  const conf::room_write_gate gate("standup");
  const std::string alice = conf::room_participant_path("standup", "alice");
  const std::string bob = conf::room_participant_path("standup", "bob");
  EXPECT_TRUE(gate.sources("alice", alice + ".intent.move.x"));
  EXPECT_TRUE(gate.sources("alice", alice + ".intent.tick"));
  EXPECT_TRUE(gate.sources("alice", alice + ".presence.joined"));
  EXPECT_FALSE(gate.sources("alice", alice + ".profile.name")) << "the relay's username";
  EXPECT_FALSE(gate.sources("alice", alice + ".pose.pos.x")) << "the relay's pose";
  EXPECT_FALSE(gate.sources("alice", alice + ".device")) << "the relay's device tag";
  EXPECT_FALSE(gate.sources("alice", bob + ".intent.move.x")) << "another participant";
  EXPECT_FALSE(gate.sources("alice", "conf.rooms.standup.scene")) << "the room";
  const cvc::state_inbound_gate open;
  EXPECT_FALSE(open.sources("alice", alice + ".intent.move.x")) << "a plain gate sources nothing";
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

// ---------------------------------------------------------------------------------------------
// room_rung_controller (room_rungs.h): each viewer's video rung, adapted to its link.

namespace {

// One viewer's link, one pass a second: its round trip, the media frames sent and dropped this
// pass, and what the relay sent it (kbit/s).
struct rung_sim {
  explicit rung_sim(conf::rung_params p = conf::rung_params()) : ctl(p) {}
  cvc::video_rung pass(double rtt_ms, int dropped_pct = 0, double kbps = 500.0,
                       double rtt_min_ms = 10.0) {
    sent += static_cast<std::uint64_t>(100 - dropped_pct);
    dropped += static_cast<std::uint64_t>(dropped_pct);
    bytes += static_cast<std::uint64_t>(kbps * 1000.0 / 8.0);
    t += 1.0;
    cvc::state_link_report r;
    r.transport = "ws";
    r.remote_node_id = "v";
    r.accepted = true;
    r.stats.rtt_ms = rtt_ms;
    r.stats.rtt_last_ms = rtt_ms;
    r.stats.rtt_min_ms = rtt_min_ms;
    r.stats.media_sent = sent;
    r.stats.media_dropped = dropped;
    r.stats.bytes_out = bytes;
    last = ctl.step({r}, t)["v"];
    return last.rung;
  }
  // N passes alike; the rung after the last.
  cvc::video_rung passes(int n, double rtt_ms, int dropped_pct = 0, double kbps = 500.0) {
    cvc::video_rung r = cvc::video_rung::full;
    for (int i = 0; i < n; ++i)
      r = pass(rtt_ms, dropped_pct, kbps);
    return r;
  }
  conf::room_rung_controller ctl;
  std::uint64_t sent = 0, dropped = 0, bytes = 0;
  double t = 0.0;
  conf::rung_sample last;
};

} // namespace

// A viewer whose frames queue (the round trip far above its minimum) steps down one rung per
// two bad seconds, to the floor and no further.
TEST(RoomRungs, AQueueingViewerStepsDownToTheFloor) {
  rung_sim sim;
  EXPECT_EQ(sim.passes(5, 12.0), cvc::video_rung::full) << "a good link stays full";
  EXPECT_EQ(sim.pass(260.0), cvc::video_rung::full) << "one bad second is not enough";
  EXPECT_EQ(sim.pass(260.0), cvc::video_rung::half);
  EXPECT_TRUE(sim.last.changed);
  EXPECT_NEAR(sim.last.queue_ms, 250.0, 1e-9);
  EXPECT_EQ(sim.passes(2, 260.0), cvc::video_rung::quarter);
  EXPECT_EQ(sim.passes(2, 260.0), cvc::video_rung::thumb);
  EXPECT_EQ(sim.passes(10, 260.0), cvc::video_rung::thumb) << "the floor (default thumb)";
}

TEST(RoomRungs, DroppedFramesStepAViewerDown) {
  rung_sim sim;
  EXPECT_EQ(sim.passes(3, 12.0, /*dropped_pct=*/2), cvc::video_rung::full)
      << "a few drops are noise";
  EXPECT_EQ(sim.passes(2, 12.0, /*dropped_pct=*/20), cvc::video_rung::half);
  EXPECT_NEAR(sim.last.drop, 0.2, 1e-9);
}

// Up is slow: a recovered link waits up_after_s of good seconds per rung. A step up that turns bad
// before its wait was over doubles the next wait.
TEST(RoomRungs, AViewerStepsBackUpSlowlyAndWaitsLongerAfterAFailedTry) {
  rung_sim sim;
  sim.passes(2, 12.0);
  ASSERT_EQ(sim.passes(2, 260.0), cvc::video_rung::half);
  EXPECT_EQ(sim.passes(10, 12.0), cvc::video_rung::half) << "nine good seconds are not ten";
  EXPECT_EQ(sim.pass(12.0), cvc::video_rung::full) << "ten are";
  // The step up does not hold: back down within its 10 s wait, so the next wait is 20 s.
  ASSERT_EQ(sim.passes(2, 260.0), cvc::video_rung::half);
  EXPECT_EQ(sim.passes(20, 12.0), cvc::video_rung::half) << "the wait doubled after the failed try";
  EXPECT_EQ(sim.pass(12.0), cvc::video_rung::full);
}

TEST(RoomRungs, TheServerSettingsBoundEveryViewer) {
  conf::rung_params best_half;
  best_half.max_rung = cvc::video_rung::half;
  rung_sim capped(best_half);
  EXPECT_EQ(capped.passes(30, 12.0), cvc::video_rung::half) << "never above --video-max";

  conf::rung_params floor_quarter;
  floor_quarter.floor_rung = cvc::video_rung::quarter;
  rung_sim floored(floor_quarter);
  EXPECT_EQ(floored.passes(20, 400.0), cvc::video_rung::quarter) << "never below --video-floor";

  conf::rung_params fixed;
  fixed.adapt = false;
  fixed.max_rung = cvc::video_rung::quarter;
  rung_sim pinned(fixed);
  EXPECT_EQ(pinned.passes(5, 400.0, 50), cvc::video_rung::quarter) << "--adapt off: --video-max";
  EXPECT_EQ(pinned.passes(30, 12.0), cvc::video_rung::quarter);

  conf::rung_params budget;
  budget.viewer_kbps_max = 1000.0;
  rung_sim budgeted(budget);
  EXPECT_EQ(budgeted.passes(3, 12.0, 0, /*kbps=*/900.0), cvc::video_rung::full);
  EXPECT_EQ(budgeted.passes(2, 12.0, 0, /*kbps=*/1600.0), cvc::video_rung::half)
      << "over --viewer-kbps-max";
  EXPECT_NEAR(budgeted.last.out_kbps, 1600.0, 1.0);
}

// Only a viewer's own link counts: a link the relay dialed or one that does not hold its pid is
// no viewer, and a viewer whose link went is forgotten.
TEST(RoomRungs, OnlyAViewersHoldingLinkCounts) {
  conf::room_rung_controller ctl;
  cvc::state_link_report dialed, duplicate, viewer;
  dialed.remote_node_id = "peer";
  duplicate.remote_node_id = "v";
  duplicate.accepted = true;
  duplicate.holds_id = false;
  viewer.remote_node_id = "v";
  viewer.accepted = true;
  const auto out = ctl.step({dialed, duplicate, viewer}, 1.0);
  EXPECT_EQ(out.size(), 1u);
  EXPECT_EQ(out.count("v"), 1u);
  EXPECT_TRUE(ctl.step({}, 2.0).empty());

  // A viewer whose link is gone is forgotten: back, it starts again at the best rung, not at the
  // one it left on.
  rung_sim sim;
  ASSERT_EQ(sim.passes(2, 260.0), cvc::video_rung::half);
  EXPECT_TRUE(sim.ctl.step({}, sim.t + 0.5).empty());
  EXPECT_EQ(sim.pass(12.0), cvc::video_rung::full);
}

// A bridged user's pid: a state name, one per (system, id), whatever the id holds.
TEST(RoomBridge, PidsAreStateNamesAndDistinct) {
  EXPECT_EQ(conf::bridged_pid("nmdc", "Joe.R"), "_nmdc_Joe_2eR");
  EXPECT_EQ(conf::bridged_pid("nmdc", "a_b"), "_nmdc_a_5fb");
  EXPECT_NE(conf::bridged_pid("nmdc", "a_b"), conf::bridged_pid("nmdc", "a.b"));
  EXPECT_NE(conf::bridged_pid("nmdc", "x"), conf::bridged_pid("irc", "x"));
  for (const std::string uid : {"Joe.R", "[ops]Ann", "\xc3\xa9t\xc3\xa9", "a b", "_"})
    EXPECT_TRUE(cvc::state::isValidStateName(conf::bridged_pid("nmdc", uid))) << uid;
  EXPECT_TRUE(conf::is_bridged_pid(conf::bridged_pid("nmdc", "x")));
  EXPECT_FALSE(conf::is_bridged_pid("alice"));
  for (const char *via : {"", "NMDC", "1x", "nm-dc", "n_m", "abcdefghijklmnopq"})
    EXPECT_EQ(conf::bridged_pid(via, "x"), "") << via;
  EXPECT_EQ(conf::bridged_pid("nmdc", ""), "");
  EXPECT_TRUE(conf::valid_bridge_via("abcdefghijklmnop")) << "16 characters";
}

// Bridged users are listed apart from the participants, in pid order; the relay's every change
// moves bridge.rev, a repeat with nothing new does not, and leaving is soft.
TEST(RoomBridge, PresenceListsThemApartAndLeavingIsSoft) {
  cvc::app app;
  conf::room_join(app, "standup", "alice", "Alice");
  std::string why;
  EXPECT_EQ(conf::room_bridge_rev(app, "standup"), 0u);
  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "zed", "Zed", "DC++ 0.868", why))
      << why;
  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "Ann.B", "", "", why)) << why;
  const std::uint64_t rev = conf::room_bridge_rev(app, "standup");
  EXPECT_GT(rev, 0u);

  const std::vector<conf::room_bridged_user> b = conf::room_bridged(app, "standup");
  ASSERT_EQ(b.size(), 2u);
  EXPECT_EQ(b[0].pid, "_nmdc_Ann_2eB");
  EXPECT_EQ(b[0].uid, "Ann.B");
  EXPECT_EQ(b[0].name, "Ann.B") << "no name: the uid";
  EXPECT_EQ(b[1].pid, "_nmdc_zed");
  EXPECT_EQ(b[1].name, "Zed");
  EXPECT_EQ(b[1].via, "nmdc");
  EXPECT_EQ(b[1].detail, "DC++ 0.868");
  ASSERT_EQ(conf::room_roster(app, "standup").size(), 1u) << "the participants are untouched";

  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "zed", "Zed", "DC++ 0.868", why));
  EXPECT_EQ(conf::room_bridge_rev(app, "standup"), rev) << "nothing new";
  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "zed", "Zed\nZ", "", why));
  EXPECT_GT(conf::room_bridge_rev(app, "standup"), rev);
  conf::room_bridged_user u;
  ASSERT_TRUE(conf::room_bridged_user_of(app, "standup", "_nmdc_zed", u));
  EXPECT_EQ(u.name, "Zed Z") << "one line";
  EXPECT_EQ(u.detail, "");

  EXPECT_TRUE(conf::room_bridge_absent(app, "standup", "nmdc", "zed"));
  EXPECT_FALSE(conf::room_bridge_absent(app, "standup", "nmdc", "zed")) << "gone already";
  EXPECT_FALSE(conf::room_bridged_user_of(app, "standup", "_nmdc_zed", u));
  ASSERT_EQ(conf::room_bridged(app, "standup").size(), 1u);
  cvc::state *present =
      app.root().findDescendant("conf.rooms.standup.bridge.users._nmdc_zed.present");
  ASSERT_NE(present, nullptr) << "soft: the node stays";
  EXPECT_EQ(present->value(), "0");
  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "zed", "Zed", "", why));
  EXPECT_EQ(conf::room_bridged(app, "standup").size(), 2u) << "back on the same node";

  // Another relay process (a new chat epoch): what the earlier one wrote is not listed.
  app.root()("conf.rooms.standup.chat.epoch").value("another");
  EXPECT_TRUE(conf::room_bridged(app, "standup").empty());
  EXPECT_FALSE(conf::room_bridged_user_of(app, "standup", "_nmdc_zed", u));
  ASSERT_TRUE(conf::room_bridge_presence(app, "standup", "nmdc", "zed", "Zed", "", why));
  EXPECT_EQ(conf::room_bridged(app, "standup").size(), 1u);
}

// What a bridged user may not be.
TEST(RoomBridge, PresenceRefusesWhatCannotBeOne) {
  cvc::app app;
  std::string why;
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "NMDC", "x", "", "", why));
  EXPECT_NE(why.find("via"), std::string::npos) << why;
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "nmdc", "", "", "", why));
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "nmdc", std::string(129, 'x'), "", "", why));
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "nmdc", "bad\xff", "", "", why));
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "nmdc", "x", std::string(129, 'n'), "", why));
  conf::bridge_limits limits;
  limits.present_max = 2;
  ASSERT_TRUE(conf::room_bridge_presence(app, "r", "nmdc", "a", "", "", why, limits));
  ASSERT_TRUE(conf::room_bridge_presence(app, "r", "nmdc", "b", "", "", why, limits));
  EXPECT_FALSE(conf::room_bridge_presence(app, "r", "nmdc", "c", "", "", why, limits));
  EXPECT_NE(why.find("2 bridged users"), std::string::npos) << why;
  EXPECT_TRUE(conf::room_bridge_presence(app, "r", "nmdc", "a", "A", "", why, limits))
      << "one present already may change";
  EXPECT_TRUE(conf::room_bridged(app, "r").size() == 2u);
}

// A hub's worth (the 6000-user soak's size) log in at once and half leave: each presence keeps
// the count without walking the others, and the roster reads them all.
TEST(RoomBridge, AHubsWorthComeAndGo) {
  cvc::app app;
  std::string why;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 6000; ++i)
    ASSERT_TRUE(
        conf::room_bridge_presence(app, "r", "nmdc", "user" + std::to_string(i), "", "a line", why))
        << why;
  for (int i = 0; i < 6000; i += 2)
    ASSERT_TRUE(conf::room_bridge_absent(app, "r", "nmdc", "user" + std::to_string(i)));
  const auto t1 = std::chrono::steady_clock::now();
  EXPECT_EQ(conf::room_bridged(app, "r").size(), 3000u);
  const auto t2 = std::chrono::steady_clock::now();
  EXPECT_EQ(app.root().findDescendant("conf.rooms.r.bridge.count")->value(), "3000");
  std::printf("[ RoomBridge ] 6000 presences + 3000 absences: %.0f ms; a roster of 3000: %.0f ms\n",
              std::chrono::duration<double, std::milli>(t1 - t0).count(),
              std::chrono::duration<double, std::milli>(t2 - t1).count());
}

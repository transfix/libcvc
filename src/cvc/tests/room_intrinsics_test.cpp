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

#include <cstddef>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_net.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
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

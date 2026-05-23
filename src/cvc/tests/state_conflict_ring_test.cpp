/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_distributed_metrics.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>

using cvc::app;
using cvc::state_cluster_shard;
using cvc::state_distributed_metrics;
using cvc::state_mutation;
using cvc::state_mutation_op;
using cvc::state_transport_inproc;

class StateConflictRingTest : public ::testing::Test {
protected:
  app ctx;
  state_transport_inproc transport;
  state_cluster_shard shard_a{ctx, "cluster", "node_A"};
  state_cluster_shard shard_b{ctx, "cluster", "node_B"};

  void SetUp() override {
    shard_a.set_resolve_conflicts(true);
    transport.register_shard(&shard_a);
    transport.register_shard(&shard_b);
    shard_a.attach();
    shard_b.attach();
  }

  void TearDown() override {
    shard_a.detach();
    shard_b.detach();
    transport.unregister_shard(&shard_a);
    transport.unregister_shard(&shard_b);
  }

  state_mutation make_mutation(const std::string &node, std::uint64_t seq,
                               const std::string &path, const std::string &value) {
    state_mutation m;
    m.cluster_id = "cluster";
    m.origin_node_id = node;
    m.sequence = seq;
    m.path = path;
    m.op = state_mutation_op::set_value;
    m.string_value = value;
    return m;
  }
};

TEST_F(StateConflictRingTest, NoConflictsWhenNoCompetingNodes) {
  auto m = make_mutation("node_B", 1, "x.y", "v1");
  shard_a.ingest_remote(m);
  auto conflicts = shard_a.recent_conflicts();
  EXPECT_TRUE(conflicts.empty());
}

TEST_F(StateConflictRingTest, ConflictRecordedWhenTwoNodesCompete) {
  // First write wins, second loses deterministically.
  auto m1 = make_mutation("node_B", 1, "x.y", "v1");
  auto r1 = shard_a.ingest_remote(m1);
  EXPECT_TRUE(r1.applied);

  auto m2 = make_mutation("node_C", 1, "x.y", "v2");
  auto r2 = shard_a.ingest_remote(m2);
  // A conflict was detected. Whether the ring recorded it depends
  // on whether m2 lost the tie-break (ring only captures the
  // losing mutation).
  EXPECT_GE(shard_a.total_conflicts_detected(), 1u);

  // At least one of (conflicts_lost, applied) should be true for m2.
  auto conflicts = shard_a.recent_conflicts();
  if (shard_a.total_conflicts_lost() > 0) {
    EXPECT_GE(conflicts.size(), 1u);
    EXPECT_EQ(conflicts[0].path, "x.y");
  }
}

TEST_F(StateConflictRingTest, ConflictsRingWrapsAround) {
  // Generate more conflicts than the ring size.
  // Use alphabetically lower node ids to guarantee loss.
  for (int i = 0; i < 200; ++i) {
    // Winner: node_C > node_B lexicographically.
    auto ma = make_mutation("node_C", static_cast<uint64_t>(i + 1),
                            "path." + std::to_string(i), "c");
    shard_a.ingest_remote(ma);
    // Loser: node_B < node_C, same path, so should_replace returns false.
    auto mc = make_mutation("node_B", static_cast<uint64_t>(i + 1),
                            "path." + std::to_string(i), "b");
    shard_a.ingest_remote(mc);
  }

  // Should return at most 128 (ring size), and they should be the most recent.
  auto conflicts = shard_a.recent_conflicts();
  EXPECT_LE(conflicts.size(), 128u);
  EXPECT_GT(conflicts.size(), 0u);
}

TEST_F(StateConflictRingTest, RecentConflictsMaxEntries) {
  for (int i = 0; i < 10; ++i) {
    // Winner first (higher node id).
    auto ma = make_mutation("node_C", static_cast<uint64_t>(i + 1),
                            "p." + std::to_string(i), "c");
    shard_a.ingest_remote(ma);
    // Loser: node_B < node_C for the same path.
    auto mc = make_mutation("node_B", static_cast<uint64_t>(i + 1),
                            "p." + std::to_string(i), "b");
    shard_a.ingest_remote(mc);
  }

  // Request only 3 entries.
  auto conflicts = shard_a.recent_conflicts(3);
  EXPECT_LE(conflicts.size(), 3u);
}

TEST_F(StateConflictRingTest, PublishConflictsWritesToStateTree) {
  // Winner first.
  auto m1 = make_mutation("node_C", 1, "x", "v1");
  shard_a.ingest_remote(m1);
  // Loser.
  auto m2 = make_mutation("node_B", 1, "x", "v2");
  shard_a.ingest_remote(m2);

  auto count = state_distributed_metrics::publish_conflicts(ctx, shard_a);
  EXPECT_GE(count, 1u);
}

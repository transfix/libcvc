/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_hash_partition.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using cvc::state_hash_partition;

class StateHashPartitionTest : public ::testing::Test {
protected:
  state_hash_partition part;
};

TEST_F(StateHashPartitionTest, EmptyMapReturnsNoOwner) {
  EXPECT_TRUE(part.owner_of("foo.bar").empty());
  EXPECT_FALSE(part.owns("node-1", "foo.bar"));
  EXPECT_EQ(part.size(), 0u);
}

TEST_F(StateHashPartitionTest, AssignAndLookup) {
  part.assign("node-1", 0, UINT32_MAX / 2);
  part.assign("node-2", UINT32_MAX / 2, UINT32_MAX);
  EXPECT_EQ(part.size(), 2u);

  // Every path should resolve to either node-1 or node-2.
  for (int i = 0; i < 50; ++i) {
    std::string path = "test.path." + std::to_string(i);
    std::string owner = part.owner_of(path);
    EXPECT_TRUE(owner == "node-1" || owner == "node-2")
        << "path=" << path << " owner=" << owner;
  }
}

TEST_F(StateHashPartitionTest, UniformPartition) {
  std::vector<std::string> nodes = {"A", "B", "C", "D"};
  part.assign_uniform(nodes);
  EXPECT_EQ(part.size(), 4u);

  // Check that each node gets at least some paths.
  std::map<std::string, int> counts;
  for (int i = 0; i < 1000; ++i) {
    std::string path = "path." + std::to_string(i);
    counts[part.owner_of(path)]++;
  }
  for (const auto &n : nodes) {
    EXPECT_GT(counts[n], 0) << "node " << n << " got zero paths";
  }
}

TEST_F(StateHashPartitionTest, OwnsMatchesOwnerOf) {
  part.assign_uniform({"x", "y"});
  for (int i = 0; i < 100; ++i) {
    std::string path = "p." + std::to_string(i);
    std::string owner = part.owner_of(path);
    EXPECT_TRUE(part.owns(owner, path));
    // The other node should NOT own it.
    std::string other = (owner == "x") ? "y" : "x";
    EXPECT_FALSE(part.owns(other, path));
  }
}

TEST_F(StateHashPartitionTest, SnapshotReturnsRanges) {
  part.assign("n1", 0, 100);
  part.assign("n2", 100, 200);
  auto snap = part.snapshot();
  EXPECT_EQ(snap.size(), 2u);
  EXPECT_EQ(snap[0].node_id, "n1");
  EXPECT_EQ(snap[0].range_begin, 0u);
  EXPECT_EQ(snap[0].range_end, 100u);
  EXPECT_EQ(snap[1].node_id, "n2");
}

TEST_F(StateHashPartitionTest, ClearRemovesAllRanges) {
  part.assign_uniform({"a", "b", "c"});
  EXPECT_EQ(part.size(), 3u);
  part.clear();
  EXPECT_EQ(part.size(), 0u);
  EXPECT_TRUE(part.owner_of("test").empty());
}

TEST_F(StateHashPartitionTest, HashIsDeterministic) {
  auto h1 = state_hash_partition::hash("foo.bar");
  auto h2 = state_hash_partition::hash("foo.bar");
  EXPECT_EQ(h1, h2);
  // Different paths should (very likely) hash differently.
  auto h3 = state_hash_partition::hash("baz.qux");
  EXPECT_NE(h1, h3);
}

TEST_F(StateHashPartitionTest, SingleNodeOwnsEverything) {
  part.assign_uniform({"solo"});
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(part.owner_of("path." + std::to_string(i)), "solo");
  }
}

TEST_F(StateHashPartitionTest, ConcurrentAccessIsSafe) {
  part.assign_uniform({"A", "B", "C"});
  std::atomic<bool> stop{false};
  auto reader = [&]() {
    while (!stop.load()) {
      (void)part.owner_of("test.concurrent");
      (void)part.size();
    }
  };
  auto writer = [&]() {
    for (int i = 0; i < 100 && !stop.load(); ++i) {
      part.assign_uniform({"X", "Y"});
      part.assign_uniform({"A", "B", "C"});
    }
  };
  std::thread t1(reader), t2(reader), t3(writer);
  t3.join();
  stop.store(true);
  t1.join();
  t2.join();
}

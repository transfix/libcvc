/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License version
  2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_replica.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

} // namespace

TEST(StateReplicaTest, LocalNodeIsSelfPeer) {
  cvc::state_replica r("nodeA");
  EXPECT_EQ(r.local_node_id(), "nodeA");
  EXPECT_TRUE(r.has_peer("nodeA"));
  EXPECT_EQ(r.peer_count(), 1u);
}

TEST(StateReplicaTest, AddRemovePeers) {
  cvc::state_replica r("nodeA");
  r.add_peer("nodeB");
  r.add_peer("nodeC");
  r.add_peer("nodeB"); // idempotent
  EXPECT_EQ(r.peer_count(), 3u);
  EXPECT_TRUE(r.has_peer("nodeB"));
  EXPECT_TRUE(r.remove_peer("nodeB"));
  EXPECT_FALSE(r.has_peer("nodeB"));
  EXPECT_FALSE(r.remove_peer("nodeA")); // cannot remove self
}

TEST(StateReplicaTest, LastAppliedAdvances) {
  cvc::state_replica r("nodeA");
  r.add_peer("nodeB");
  EXPECT_EQ(r.last_applied("nodeB"), 0u);
  EXPECT_EQ(r.set_last_applied("nodeB", 10), 0u);
  EXPECT_EQ(r.last_applied("nodeB"), 10u);
  // Older sequence does not regress.
  EXPECT_EQ(r.set_last_applied("nodeB", 5), 10u);
  EXPECT_EQ(r.last_applied("nodeB"), 10u);
  EXPECT_EQ(r.set_last_applied("nodeB", 20), 10u);
  EXPECT_EQ(r.last_applied("nodeB"), 20u);
}

TEST(StateReplicaTest, LastAppliedAutoRegistersUnknownPeer) {
  cvc::state_replica r("nodeA");
  r.set_last_applied("nodeX", 7);
  EXPECT_TRUE(r.has_peer("nodeX"));
  EXPECT_EQ(r.last_applied("nodeX"), 7u);
}

TEST(StateReplicaTest, ClockAdvancesMonotonically) {
  cvc::state_replica r("nodeA");
  r.observe_local(5);
  r.observe_local(3); // ignored
  r.observe_remote("nodeB", 9);
  r.observe_remote("nodeB", 4); // ignored
  EXPECT_EQ(r.clock_component("nodeA"), 5u);
  EXPECT_EQ(r.clock_component("nodeB"), 9u);
  EXPECT_EQ(r.clock_component("missing"), 0u);
}

TEST(StateReplicaTest, ClockSnapshotMatchesComponents) {
  cvc::state_replica r("nodeA");
  r.observe_local(11);
  r.observe_remote("nodeB", 7);
  auto snap = r.clock_snapshot();
  EXPECT_EQ(snap.at("nodeA"), 11u);
  EXPECT_EQ(snap.at("nodeB"), 7u);
}

TEST(StateReplicaTest, CompareClocksRelationships) {
  using cc = cvc::state_replica::clock_compare;
  std::unordered_map<std::string, std::uint64_t> a{{"A", 1}, {"B", 2}};
  std::unordered_map<std::string, std::uint64_t> b{{"A", 1}, {"B", 3}};
  std::unordered_map<std::string, std::uint64_t> c{{"A", 2}, {"B", 1}};
  EXPECT_EQ(cvc::state_replica::compare_clocks(a, a), cc::equal);
  EXPECT_EQ(cvc::state_replica::compare_clocks(a, b), cc::less_than);
  EXPECT_EQ(cvc::state_replica::compare_clocks(b, a), cc::greater_than);
  EXPECT_EQ(cvc::state_replica::compare_clocks(a, c), cc::concurrent);
}

TEST(StateReplicaTest, ShouldReplaceUsesOriginThenSequence) {
  cvc::state_mutation current;
  current.origin_node_id = "nodeA";
  current.sequence = 5;
  current.path = "x";

  cvc::state_mutation incoming;
  incoming.origin_node_id = "nodeA";
  incoming.sequence = 6;
  incoming.path = "x";
  EXPECT_TRUE(cvc::state_replica::should_replace(current, incoming));

  incoming.sequence = 4;
  EXPECT_FALSE(cvc::state_replica::should_replace(current, incoming));

  // Different origin: lex compare of origin_node_id.
  cvc::state_mutation b_in;
  b_in.origin_node_id = "nodeB";
  b_in.sequence = 1;
  b_in.path = "x";
  EXPECT_TRUE(cvc::state_replica::should_replace(current, b_in));
  EXPECT_FALSE(cvc::state_replica::should_replace(b_in, current));
}

TEST(StateReplicaTest, SeenDetectsLoops) {
  cvc::state_replica r("nodeA");
  EXPECT_FALSE(r.seen("nodeA", 1, /*record*/ true));
  EXPECT_TRUE(r.seen("nodeA", 1, /*record*/ false));
  // Recording the same again should not double-count.
  EXPECT_TRUE(r.seen("nodeA", 1, /*record*/ true));
  EXPECT_EQ(r.seen_size(), 1u);

  EXPECT_FALSE(r.seen("nodeB", 7, true));
  EXPECT_FALSE(r.seen("nodeB", 8, true));
  EXPECT_TRUE(r.seen("nodeB", 7, false));
  EXPECT_EQ(r.seen_size(), 3u);

  r.clear_seen();
  EXPECT_EQ(r.seen_size(), 0u);
  EXPECT_FALSE(r.seen("nodeA", 1, false));
}

TEST(StateReplicaTest, MarkAlive) {
  cvc::state_replica r("nodeA");
  r.add_peer("nodeB");
  auto ps = r.peers();
  for (const auto &p : ps)
    EXPECT_TRUE(p.alive);
  r.mark_alive("nodeB", false);
  ps = r.peers();
  for (const auto &p : ps)
    if (p.node_id == "nodeB")
      EXPECT_FALSE(p.alive);
}

TEST(StateReplicaStressTest, OptionalConcurrentObserveStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run replica "
                    "stress tests";
  }
  cvc::state_replica r("nodeA");
  for (int i = 0; i < 8; ++i)
    r.add_peer("node" + std::to_string(i));

  const int kThreads = 8;
  const int kPerThread = 20000;
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      std::string nid = "node" + std::to_string(t);
      for (int i = 0; i < kPerThread; ++i) {
        r.observe_remote(nid, static_cast<std::uint64_t>(i));
        r.set_last_applied(nid, static_cast<std::uint64_t>(i));
        r.seen(nid, static_cast<std::uint64_t>(i), true);
      }
    });
  }
  for (auto &th : threads)
    th.join();
  for (int t = 0; t < kThreads; ++t) {
    std::string nid = "node" + std::to_string(t);
    EXPECT_EQ(r.last_applied(nid),
              static_cast<std::uint64_t>(kPerThread - 1));
    EXPECT_EQ(r.clock_component(nid),
              static_cast<std::uint64_t>(kPerThread - 1));
  }
}

TEST(StateReplicaPerformanceTest, OptionalSeenInsertThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run replica "
                    "performance smoke tests";
  }
  cvc::state_replica r("nodeA");
  const int kIters = 100000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i)
    r.seen("nodeA", static_cast<std::uint64_t>(i), true);
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs =
      std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
          .count();
  std::cerr << "[replica perf] " << kIters << " seen-record in " << secs
            << "s (" << (kIters / secs) << "/s)\n";
  EXPECT_LT(secs, 10.0);
  EXPECT_EQ(r.seen_size(), static_cast<std::size_t>(kIters));
}

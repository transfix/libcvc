/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/core/state_authority_map.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

} // namespace

TEST(StateAuthorityMapTest, EmptyResolveReturnsInvalid) {
  cvc::state_authority_map m;
  EXPECT_EQ(m.size(), 0u);
  auto r = m.resolve("a.b.c");
  EXPECT_FALSE(r.valid);
}

TEST(StateAuthorityMapTest, RootDelegationCoversEverything) {
  cvc::state_authority_map m;
  m.delegate("", "root-cluster", "grpc://root:1");
  auto r = m.resolve("anything.here");
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.cluster_id, "root-cluster");
  EXPECT_EQ(r.endpoint, "grpc://root:1");
}

TEST(StateAuthorityMapTest, LongestPrefixWins) {
  cvc::state_authority_map m;
  m.delegate("", "root");
  m.delegate("a", "A-cluster");
  m.delegate("a.b", "AB-cluster");
  m.delegate("a.b.c", "ABC-cluster");

  EXPECT_EQ(m.resolve("a.b.c.d.e").cluster_id, "ABC-cluster");
  EXPECT_EQ(m.resolve("a.b.x").cluster_id, "AB-cluster");
  EXPECT_EQ(m.resolve("a.y").cluster_id, "A-cluster");
  EXPECT_EQ(m.resolve("z").cluster_id, "root");
  EXPECT_EQ(m.size(), 4u);
}

TEST(StateAuthorityMapTest, ExactPathMatchesItself) {
  cvc::state_authority_map m;
  m.delegate("a.b", "AB");
  EXPECT_EQ(m.resolve("a.b").cluster_id, "AB");
  EXPECT_TRUE(m.has_exact("a.b"));
  EXPECT_FALSE(m.has_exact("a"));
  EXPECT_FALSE(m.has_exact("a.b.c"));
}

TEST(StateAuthorityMapTest, RevokeRemovesOnlyExactEntry) {
  cvc::state_authority_map m;
  m.delegate("a", "A");
  m.delegate("a.b", "AB");
  EXPECT_TRUE(m.revoke("a.b"));
  EXPECT_FALSE(m.revoke("a.b")); // already gone
  EXPECT_FALSE(m.has_exact("a.b"));
  // Falls back to "a"
  EXPECT_EQ(m.resolve("a.b.c").cluster_id, "A");
  EXPECT_EQ(m.size(), 1u);
}

TEST(StateAuthorityMapTest, DelegateOverwrites) {
  cvc::state_authority_map m;
  m.delegate("a.b", "old");
  m.delegate("a.b", "new", "grpc://new:2", 12345);
  auto r = m.resolve("a.b");
  EXPECT_EQ(r.cluster_id, "new");
  EXPECT_EQ(r.endpoint, "grpc://new:2");
  EXPECT_EQ(r.expires_at_ns, 12345u);
  EXPECT_EQ(m.size(), 1u);
}

TEST(StateAuthorityMapTest, NormalizationStripsDots) {
  cvc::state_authority_map m;
  m.delegate(".a.b.", "AB");
  EXPECT_EQ(m.resolve("a.b.c").cluster_id, "AB");
  EXPECT_EQ(m.resolve(".a.b.").cluster_id, "AB");
  EXPECT_TRUE(m.has_exact("a.b"));
}

TEST(StateAuthorityMapTest, SnapshotOrderedMostSpecificFirst) {
  cvc::state_authority_map m;
  m.delegate("a", "A");
  m.delegate("a.b.c", "ABC");
  m.delegate("a.b", "AB");
  auto snap = m.snapshot();
  ASSERT_EQ(snap.size(), 3u);
  EXPECT_EQ(snap[0].first, "a.b.c");
  EXPECT_EQ(snap[1].first, "a.b");
  EXPECT_EQ(snap[2].first, "a");
}

TEST(StateAuthorityMapTest, ClearEmptiesMap) {
  cvc::state_authority_map m;
  m.delegate("a", "A");
  m.delegate("a.b", "AB");
  m.clear();
  EXPECT_EQ(m.size(), 0u);
  EXPECT_FALSE(m.resolve("a.b").valid);
}

TEST(StateAuthorityMapStressTest, OptionalConcurrentDelegateResolve) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run authority "
                    "map stress tests";
  }
  cvc::state_authority_map m;
  const int kWriters = 4;
  const int kReaders = 4;
  const int kPerThread = 5000;
  std::atomic<bool> stop{false};
  std::vector<std::thread> threads;
  for (int t = 0; t < kWriters; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kPerThread; ++i) {
        std::string path = "cluster" + std::to_string(t) + ".node" + std::to_string(i % 16);
        m.delegate(path, "cluster" + std::to_string(t));
      }
    });
  }
  for (int t = 0; t < kReaders; ++t) {
    threads.emplace_back([&]() {
      while (!stop.load()) {
        auto r = m.resolve("cluster0.node3.deep.path");
        (void)r;
      }
    });
  }
  for (int t = 0; t < kWriters; ++t)
    threads[t].join();
  stop.store(true);
  for (int t = kWriters; t < kWriters + kReaders; ++t)
    threads[t].join();
  EXPECT_GT(m.size(), 0u);
}

TEST(StateAuthorityMapPerformanceTest, OptionalResolveThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run authority "
                    "map performance smoke tests";
  }
  cvc::state_authority_map m;
  for (int i = 0; i < 256; ++i) {
    std::string p = "ns." + std::to_string(i) + ".sub";
    m.delegate(p, "c" + std::to_string(i));
  }
  const int kIters = 100000;
  auto start = std::chrono::steady_clock::now();
  std::size_t hits = 0;
  for (int i = 0; i < kIters; ++i) {
    auto r = m.resolve("ns." + std::to_string(i % 256) + ".sub.deep.path");
    if (r.valid)
      ++hits;
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  std::cerr << "[authority_map perf] " << kIters << " resolves in " << secs << "s ("
            << (kIters / secs) << "/s)\n";
  EXPECT_EQ(hits, static_cast<std::size_t>(kIters));
  EXPECT_LT(secs, 10.0);
}

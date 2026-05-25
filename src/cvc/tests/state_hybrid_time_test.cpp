/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/core/state_hybrid_time.h>
#include <gtest/gtest.h>
#include <thread>
#include <vector>

using cvc::hybrid_clock;
using cvc::hybrid_time;

// ---------------------------------------------------------------
// hybrid_time struct tests
// ---------------------------------------------------------------

TEST(HybridTimeTest, DefaultIsZero) {
  hybrid_time t;
  EXPECT_EQ(t.wall_ms, 0u);
  EXPECT_EQ(t.logical, 0u);
  EXPECT_EQ(t.packed(), 0u);
  EXPECT_FALSE(static_cast<bool>(t));
}

TEST(HybridTimeTest, PackUnpackRoundTrip) {
  hybrid_time t{123456789, 42};
  auto packed = t.packed();
  auto t2 = hybrid_time::from_packed(packed);
  EXPECT_EQ(t2.wall_ms, t.wall_ms);
  EXPECT_EQ(t2.logical, t.logical);
}

TEST(HybridTimeTest, PackedOrderIsCorrect) {
  hybrid_time a{100, 0};
  hybrid_time b{100, 1};
  hybrid_time c{101, 0};
  EXPECT_LT(a, b);
  EXPECT_LT(b, c);
  EXPECT_GT(c, a);
}

TEST(HybridTimeTest, ComparisonOperators) {
  hybrid_time a{10, 5};
  hybrid_time b{10, 5};
  hybrid_time c{10, 6};
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  EXPECT_LE(a, b);
  EXPECT_GE(a, b);
  EXPECT_LT(a, c);
  EXPECT_GT(c, a);
}

TEST(HybridTimeTest, BoolConversionNonZero) {
  hybrid_time t{1, 0};
  EXPECT_TRUE(static_cast<bool>(t));
}

TEST(HybridTimeTest, MaxLogicalValue) {
  hybrid_time t{1, 0xFFFF};
  auto packed = t.packed();
  auto t2 = hybrid_time::from_packed(packed);
  EXPECT_EQ(t2.wall_ms, 1u);
  EXPECT_EQ(t2.logical, 0xFFFF);
}

// ---------------------------------------------------------------
// hybrid_clock tests
// ---------------------------------------------------------------

TEST(HybridClockTest, NowIsMonotonicallyIncreasing) {
  hybrid_clock clk;
  auto t1 = clk.now();
  auto t2 = clk.now();
  auto t3 = clk.now();
  EXPECT_LT(t1, t2);
  EXPECT_LT(t2, t3);
}

TEST(HybridClockTest, CurrentDoesNotAdvance) {
  hybrid_clock clk;
  auto t = clk.now();
  auto c1 = clk.current();
  auto c2 = clk.current();
  EXPECT_EQ(c1, c2);
  EXPECT_EQ(c1, t);
}

TEST(HybridClockTest, UpdateMergesRemote) {
  hybrid_clock clk;
  auto local = clk.now();

  // Simulate a remote timestamp far in the future.
  hybrid_time remote{local.wall_ms + 10000, 5};
  auto merged = clk.update(remote);

  // Merged should be strictly after both local and remote.
  EXPECT_GT(merged, local);
  EXPECT_GT(merged, remote);
}

TEST(HybridClockTest, UpdateNeverGoesBackward) {
  hybrid_clock clk;
  auto t1 = clk.now();

  // Update with an old timestamp.
  hybrid_time old{1, 0};
  auto t2 = clk.update(old);
  EXPECT_GT(t2, t1);
}

TEST(HybridClockTest, ConcurrentNowIsSafe) {
  hybrid_clock clk;
  std::vector<hybrid_time> results(1000);
  auto worker = [&](int start, int count) {
    for (int i = start; i < start + count; ++i)
      results[i] = clk.now();
  };
  std::thread t1(worker, 0, 500);
  std::thread t2(worker, 500, 500);
  t1.join();
  t2.join();

  // All timestamps should be unique.
  std::set<std::uint64_t> packed_set;
  for (const auto &t : results)
    packed_set.insert(t.packed());
  EXPECT_EQ(packed_set.size(), results.size());
}

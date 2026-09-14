/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/rng.h for the full header).
*/

// Determinism, range, insertion-stability and stream-independence of the hashed
// RNG — the properties that make world generation reproducible.

#include <cmath>
#include <cvc/lsys/rng.h>
#include <gtest/gtest.h>
#include <unordered_set>

using namespace cvc::lsys;

TEST(LsysRng, UniformInRangeAndMeanHalf) {
  double sum = 0;
  const int N = 200000;
  for (int i = 0; i < N; ++i) {
    double u = uni(1234, stream::placement, cell_id(i % 100 - 50, i / 100 - 50), 0);
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
    sum += u;
  }
  EXPECT_NEAR(sum / N, 0.5, 0.01);
}

TEST(LsysRng, PureFunctionAndOrderIndependent) {
  // Same coordinates -> same value, regardless of when it is drawn.
  EXPECT_EQ(hash4(7, 3, 42, 1), hash4(7, 3, 42, 1));
  EXPECT_DOUBLE_EQ(uni(7, stream::size, cell_id(3, 4), 1), uni(7, stream::size, cell_id(3, 4), 1));
  // Different draw ordinal -> (almost surely) different value.
  EXPECT_NE(uni(7, stream::size, cell_id(3, 4), 1), uni(7, stream::size, cell_id(3, 4), 2));
}

TEST(LsysRng, StreamsAreIndependent) {
  // The salt-audit invariant at its root: two streams at the same element/draw
  // decorrelate, so perturbing a cosmetic stream cannot move a placement draw.
  int diff = 0;
  for (int i = 0; i < 1000; ++i) {
    double a = uni(9, stream::placement, cell_id(i, 0), 0);
    double b = uni(9, stream::sway, cell_id(i, 0), 0);
    if (a != b)
      ++diff;
  }
  EXPECT_EQ(diff, 1000);
}

TEST(LsysRng, InsertionStabilityViaCellId) {
  // cell_id is a Morton code of the coordinate, NOT a loop counter, so a value
  // keyed on a cell is invariant to how many other cells exist / their order.
  double a = uni(5, stream::placement, cell_id(-12, 7), 0);
  double b = uni(5, stream::placement, cell_id(-12, 7), 0);
  EXPECT_DOUBLE_EQ(a, b);
  // Distinct cells -> distinct ids.
  std::unordered_set<std::uint64_t> ids;
  for (int y = -20; y <= 20; ++y)
    for (int x = -20; x <= 20; ++x)
      ids.insert(cell_id(x, y));
  EXPECT_EQ(ids.size(), std::size_t(41 * 41));
}

TEST(LsysRng, PathIdStable) {
  EXPECT_EQ(path_id(100, 3), path_id(100, 3));
  EXPECT_NE(path_id(100, 3), path_id(100, 4));
  EXPECT_NE(path_id(101, 3), path_id(100, 3));
}

TEST(LsysRng, NormalRoughMoments) {
  double m = 0, s2 = 0;
  const int N = 200000;
  for (int i = 0; i < N; ++i) {
    double z = nrand(3, stream::size, i, 0, 0.0, 1.0);
    m += z;
  }
  m /= N;
  for (int i = 0; i < N; ++i) {
    double z = nrand(3, stream::size, i, 0, 0.0, 1.0);
    s2 += (z - m) * (z - m);
  }
  s2 /= N;
  EXPECT_NEAR(m, 0.0, 0.02);
  EXPECT_NEAR(std::sqrt(s2), 1.0, 0.03);
}

TEST(LsysRng, IrandInRange) {
  int lo = 100000, hi = -100000;
  for (int i = 0; i < 50000; ++i) {
    int v = irand(2, stream::maturity, i, 0, 1, 4);
    ASSERT_GE(v, 1);
    ASSERT_LE(v, 4);
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  EXPECT_EQ(lo, 1);
  EXPECT_EQ(hi, 4);
}

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::algo's exact Euclidean distance transform: distances by brute force, the one-pass pair and
// the row-at-a-time form the same bits as two plain calls, and cvc::nav::edt2_squared is the same
// function (nav_test's NavEdtExact checks the bits against GRL-SNAM's reference kernel).

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/algo/distance_transform.h>
#include <cvc/nav/grid_nav.h>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using cvc::algo::edt2_squared;
using cvc::algo::edt2_squared_both;
using cvc::algo::edt2_squared_rows;

namespace {

std::vector<std::uint8_t> scatter(int rows, int cols, std::uint32_t seed, int percent) {
  std::vector<std::uint8_t> m(std::size_t(rows) * cols);
  for (std::uint8_t &v : m) {
    seed = seed * 1664525u + 1013904223u;
    v = (seed >> 8) % 100 < std::uint32_t(percent) ? 1 : 0;
  }
  return m;
}

bool same_bits(const std::vector<double> &a, const std::vector<double> &b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

} // namespace

TEST(AlgoDistanceTransform, IsTheSquaredDistanceToTheNearestSeed) {
  const int R = 23, C = 31;
  const std::vector<std::uint8_t> m = scatter(R, C, 7u, 6);
  const std::vector<double> d = edt2_squared(m.data(), R, C);
  ASSERT_EQ(d.size(), m.size());
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) {
      double best = std::numeric_limits<double>::infinity();
      for (int rr = 0; rr < R; ++rr)
        for (int cc = 0; cc < C; ++cc)
          if (m[std::size_t(rr) * C + cc])
            best = std::min(best, double((rr - r) * (rr - r) + (cc - c) * (cc - c)));
      EXPECT_EQ(d[std::size_t(r) * C + c], best) << r << "," << c;
    }
}

TEST(AlgoDistanceTransform, AGridWithNoSeedIsFar) {
  const std::vector<std::uint8_t> m(12, 0);
  for (double v : edt2_squared(m.data(), 3, 4))
    EXPECT_GE(v, cvc::algo::edt_far);
  EXPECT_TRUE(edt2_squared(nullptr, 0, 4).empty());
  EXPECT_TRUE(edt2_squared(nullptr, 4, 0).empty());
}

TEST(AlgoDistanceTransform, BothIsTheMaskAndItsComplementBitForBit) {
  for (int percent : {0, 3, 30, 97, 100}) {
    SCOPED_TRACE(percent);
    const int R = 41, C = 17;
    const std::vector<std::uint8_t> m = scatter(R, C, 99u + percent, percent);
    std::vector<std::uint8_t> inv(m.size());
    for (std::size_t i = 0; i < m.size(); ++i)
      inv[i] = m[i] ? 0 : 1;
    std::vector<double> to_seed(m.size()), to_other(m.size());
    edt2_squared_both(m.data(), R, C, to_seed.data(), to_other.data());
    EXPECT_TRUE(same_bits(to_seed, edt2_squared(m.data(), R, C)));
    EXPECT_TRUE(same_bits(to_other, edt2_squared(inv.data(), R, C)));
  }
}

TEST(AlgoDistanceTransform, RowsComeBottomUpWithTheSameBits) {
  const int R = 19, C = 27;
  const std::vector<std::uint8_t> m = scatter(R, C, 3u, 10);
  std::vector<double> rowS(C), rowN(C), gotS(m.size()), gotN(m.size());
  std::vector<int> order;
  edt2_squared_rows(
      m.data(), R, C, true,
      [&](int, double *&s, double *&n) {
        s = rowS.data();
        n = rowN.data();
      },
      [&](int r) {
        order.push_back(r);
        std::memcpy(&gotS[std::size_t(r) * C], rowS.data(), C * sizeof(double));
        std::memcpy(&gotN[std::size_t(r) * C], rowN.data(), C * sizeof(double));
      });
  ASSERT_EQ(order.size(), std::size_t(R));
  for (int i = 0; i < R; ++i)
    EXPECT_EQ(order[std::size_t(i)], R - 1 - i);
  std::vector<double> wantS(m.size()), wantN(m.size());
  edt2_squared_both(m.data(), R, C, wantS.data(), wantN.data());
  EXPECT_TRUE(same_bits(gotS, wantS));
  EXPECT_TRUE(same_bits(gotN, wantN));
}

TEST(AlgoDistanceTransform, NavsTransformIsThisOne) {
  const int R = 64, C = 48;
  const std::vector<std::uint8_t> m = scatter(R, C, 1234u, 5);
  EXPECT_TRUE(same_bits(cvc::nav::edt2_squared(m.data(), R, C), edt2_squared(m.data(), R, C)));
}

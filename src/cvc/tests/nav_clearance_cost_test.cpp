// clearance_cost: the per-cell A* surcharge that buys standoff from buildings.
//
// The reference values below come from sdf_nav.clearance_cost on the same 8x8
// occupancy, so this is a parity gate and not just a self-consistency check --
// a native route that costs cells differently from the torch reference would
// send the two stacks down different streets, which no rollout parity test
// looks at.
#include <cmath>
#include <cvc/nav/grid_nav.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::nav::astar;
using cvc::nav::clearance_cost;

namespace {

// A 2x2 block at rows/cols 3..4 of an 8x8 grid.
std::vector<std::uint8_t> block_world() {
  std::vector<std::uint8_t> occ(64, 0);
  for (int r = 3; r <= 4; ++r)
    for (int c = 3; c <= 4; ++c)
      occ[r * 8 + c] = 1;
  return occ;
}

// sdf_nav.clearance_cost(occ, d_safe=3.0, gamma=2.0), row-major.
// sdf_nav.clearance_cost(occ, d_safe=3.0, gamma=2.0), row-major, at FULL
// precision. The first draft of this table was rounded to six decimals and
// asserted at 1e-9, which fails by ~2.5e-7 on the diagonal cells -- the values
// were right and the transcription was not.
const double kD = 0.3431457505076194;                           // 2*(3 - sqrt(8))
const double kE = 1.5278640450004204;                           // 2*(3 - sqrt(5))
const double kF = 3.1715728752538097;                           // 2*(3 - sqrt(2))
const double kPy[64] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
                        0.0, kD,  kE,  2.0, 2.0, kE,  kD,  0.0, //
                        0.0, kE,  kF,  4.0, 4.0, kF,  kE,  0.0, //
                        0.0, 2.0, 4.0, 6.0, 6.0, 4.0, 2.0, 0.0, //
                        0.0, 2.0, 4.0, 6.0, 6.0, 4.0, 2.0, 0.0, //
                        0.0, kE,  kF,  4.0, 4.0, kF,  kE,  0.0, //
                        0.0, kD,  kE,  2.0, 2.0, kE,  kD,  0.0, //
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

} // namespace

TEST(NavClearanceCost, MatchesTheTorchReference) {
  const auto occ = block_world();
  const auto got = clearance_cost(occ.data(), 8, 8, 3.0, 2.0);
  ASSERT_EQ(got.size(), 64u);
  for (int i = 0; i < 64; ++i)
    EXPECT_NEAR(got[i], kPy[i], 1e-9) << "cell " << i;
}

TEST(NavClearanceCost, PaysNothingBeyondDSafe) {
  // The corners of this world are >= 3 cells from the block, so they are free.
  const auto occ = block_world();
  const auto c = clearance_cost(occ.data(), 8, 8, 3.0, 2.0);
  EXPECT_DOUBLE_EQ(c[0], 0.0);
  EXPECT_DOUBLE_EQ(c[7], 0.0);
  EXPECT_DOUBLE_EQ(c[63], 0.0);
}

TEST(NavClearanceCost, PeaksAtGammaTimesDSafeInsideTheWall) {
  // Clearance is 0 inside a blocked cell, so the surcharge is gamma * d_safe.
  const auto occ = block_world();
  const auto c = clearance_cost(occ.data(), 8, 8, 3.0, 2.0);
  EXPECT_NEAR(c[3 * 8 + 3], 6.0, 1e-12);
}

TEST(NavClearanceCost, RouteKeepsMoreStandoffThanTheShortestPath) {
  // The behaviour the surcharge exists for. A shortest path scrapes the block
  // because that is what shortest means.
  const auto occ = block_world();
  const auto cost = clearance_cost(occ.data(), 8, 8, 3.0, 2.0);
  const auto shortest = astar(occ.data(), 8, 8, 0, 3, 7, 4, nullptr);
  const auto standoff = astar(occ.data(), 8, 8, 0, 3, 7, 4, cost.data());
  ASSERT_FALSE(shortest.empty());
  ASSERT_FALSE(standoff.empty());

  auto worst = [&](const std::vector<int> &p) {
    double m = 1e9;
    for (std::size_t i = 0; i + 1 < p.size(); i += 2) {
      double best = 1e9;
      for (int r = 3; r <= 4; ++r)
        for (int c = 3; c <= 4; ++c)
          best = std::min(best, std::hypot(double(p[i] - r), double(p[i + 1] - c)));
      m = std::min(m, best);
    }
    return m;
  };
  EXPECT_GT(worst(standoff), worst(shortest));
}

TEST(NavClearanceCost, ANarrowCorridorStillRoutes) {
  // Additive and never forbidding: a corridor tighter than d_safe everywhere
  // degrades to the shortest path instead of stranding the agent.
  std::vector<std::uint8_t> occ(40 * 40, 0);
  for (int r = 0; r < 40; ++r)
    for (int c = 0; c < 40; ++c)
      if (c < 18 || c >= 22)
        occ[r * 40 + c] = 1;
  const auto cost = clearance_cost(occ.data(), 40, 40, 6.0, 1.5);
  const auto path = astar(occ.data(), 40, 40, 1, 20, 38, 20, cost.data());
  EXPECT_FALSE(path.empty()) << "a sub-d_safe corridor must still route";
}

/*
  Copyright 2007-2011 The University of Texas at Austin

  Unit tests for cvc/nav/grid_nav.h — the belief-space grid navigation kernels
  (EDT / build_sdf / inflate / line_of_sight / nearest_free / A* / simplify).

  These are hand-verifiable golden cases. The exhaustive cross-language
  bit-identity check against the GRL-SNAM Python reference lives in that repo's
  test suite (tests/test_nav_cpp_parity.py); here we lock in the small,
  human-checkable invariants so a refactor that breaks the port fails in
  libcvc's own CI without needing Python or pycvc.
*/

#include <cmath>
#include <cstdint>
#include <cvc/nav/grid_nav.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::nav;

namespace {

// row-major helper
std::vector<std::uint8_t> grid(int rows, int cols, std::initializer_list<int> v) {
  std::vector<std::uint8_t> g;
  g.reserve(rows * cols);
  for (int x : v)
    g.push_back((std::uint8_t)(x ? 1 : 0));
  EXPECT_EQ((int)g.size(), rows * cols);
  return g;
}

} // namespace

// ─── EDT ────────────────────────────────────────────────────────────────────

TEST(NavEdt, SingleSeedGivesSquaredEuclidean) {
  // 1x5 line with the seed at column 2: squared distances 4,1,0,1,4.
  const auto m = grid(1, 5, {0, 0, 1, 0, 0});
  const auto d = edt2_squared(m.data(), 1, 5);
  const std::vector<double> want = {4, 1, 0, 1, 4};
  ASSERT_EQ(d.size(), want.size());
  for (size_t i = 0; i < want.size(); ++i)
    EXPECT_DOUBLE_EQ(d[i], want[i]) << "cell " << i;
}

TEST(NavEdt, TwoDCornerSeed) {
  // 3x3, single seed at (0,0): squared distance = r*r + c*c.
  const auto m = grid(3, 3, {1, 0, 0, 0, 0, 0, 0, 0, 0});
  const auto d = edt2_squared(m.data(), 3, 3);
  const std::vector<double> want = {0, 1, 4, 1, 2, 5, 4, 5, 8};
  for (size_t i = 0; i < want.size(); ++i)
    EXPECT_DOUBLE_EQ(d[i], want[i]) << "cell " << i;
}

TEST(NavEdt, EmptyGridIsAllInf) {
  const auto m = grid(2, 2, {0, 0, 0, 0});
  const auto d = edt2_squared(m.data(), 2, 2);
  for (double v : d)
    EXPECT_GT(v, 1e19); // no seed -> ~1e20
}

// ─── build_sdf ──────────────────────────────────────────────────────────────

TEST(NavSdf, SignAndScaleAcrossAWall) {
  // A single building column in the middle; phi must be negative inside it and
  // positive in the free cells, and scale linearly with cell width.
  const int rows = 1, cols = 5;
  const auto occ = grid(rows, cols, {0, 0, 1, 0, 0});
  // world spans 0..4 over 5 cells -> cell_w = 1.0; scale 0.5.
  const auto f = build_sdf(occ.data(), rows, cols, 0.0, 0.0, 4.0, 0.0, 0.5);
  EXPECT_EQ(f.rows, rows);
  EXPECT_EQ(f.cols, cols);
  EXPECT_LT(f.phi[2], 0.0f); // inside the building
  EXPECT_GT(f.phi[0], 0.0f); // free
  EXPECT_GT(f.phi[4], 0.0f); // free
  // phi[2]: dist_to_building=0, dist_to_free=1 -> (0-1) * cell_w(1) * scale(.5) = -0.5
  EXPECT_FLOAT_EQ(f.phi[2], -0.5f);
  // phi at col 0: dist_to_building=2, dist_to_free=0 -> 2 * cell_w(1) * scale(.5)=1.0
  EXPECT_FLOAT_EQ(f.phi[0], 1.0f);
  EXPECT_FLOAT_EQ(f.phi[1], 0.5f); // one cell from the wall
  // unit normals
  for (int i = 0; i < rows * cols; ++i) {
    const float mag = std::sqrt(f.normal_x[i] * f.normal_x[i] + f.normal_y[i] * f.normal_y[i]);
    EXPECT_NEAR(mag, 1.0f, 1e-5f) << "cell " << i;
  }
}

// ─── inflate ────────────────────────────────────────────────────────────────

TEST(NavInflate, FourConnectedDilationOneStep) {
  const int rows = 3, cols = 3;
  const auto occ = grid(rows, cols, {0, 0, 0, 0, 1, 0, 0, 0, 0});
  const auto out = inflate(occ.data(), rows, cols, 1);
  // center plus its 4 neighbours set; corners stay clear.
  const std::vector<int> want = {0, 1, 0, 1, 1, 1, 0, 1, 0};
  for (int i = 0; i < rows * cols; ++i)
    EXPECT_EQ((int)out[i], want[i]) << "cell " << i;
}

TEST(NavInflate, ZeroCellsIsBooleanCopy) {
  const auto occ = grid(2, 2, {1, 0, 0, 1});
  const auto out = inflate(occ.data(), 2, 2, 0);
  EXPECT_EQ((int)out[0], 1);
  EXPECT_EQ((int)out[1], 0);
  EXPECT_EQ((int)out[2], 0);
  EXPECT_EQ((int)out[3], 1);
}

// ─── line_of_sight ──────────────────────────────────────────────────────────

TEST(NavLineOfSight, ClearAndBlocked) {
  const int rows = 1, cols = 5;
  const auto clear = grid(rows, cols, {0, 0, 0, 0, 0});
  EXPECT_TRUE(line_of_sight(clear.data(), rows, cols, 0, 0, 0, 4));
  const auto wall = grid(rows, cols, {0, 0, 1, 0, 0});
  EXPECT_FALSE(line_of_sight(wall.data(), rows, cols, 0, 0, 0, 4));
  // endpoint == start
  EXPECT_TRUE(line_of_sight(clear.data(), rows, cols, 0, 2, 0, 2));
}

// ─── nearest_free ───────────────────────────────────────────────────────────

TEST(NavNearestFree, AlreadyFreeIsIdentity) {
  const auto occ = grid(3, 3, {0, 0, 0, 0, 1, 0, 0, 0, 0});
  auto p = nearest_free(occ.data(), 3, 3, 0, 0, 12);
  EXPECT_EQ(p.first, 0);
  EXPECT_EQ(p.second, 0);
}

TEST(NavNearestFree, SnapsOutOfAnObstacleTopLeftBiased) {
  // center blocked; the scan order visits rad=1, dr=-1 first, so it returns the
  // up-left neighbourhood before others. (r+dr,c+dc) with dr=-1,dc=-1 => (0,0).
  const auto occ = grid(3, 3, {0, 0, 0, 0, 1, 0, 0, 0, 0});
  auto p = nearest_free(occ.data(), 3, 3, 1, 1, 12);
  EXPECT_EQ(p.first, 0);
  EXPECT_EQ(p.second, 0);
}

TEST(NavNearestFree, NoneWhenBoxedIn) {
  const auto occ = grid(3, 3, {1, 1, 1, 1, 1, 1, 1, 1, 1});
  auto p = nearest_free(occ.data(), 3, 3, 1, 1, 12);
  EXPECT_EQ(p.first, -1);
  EXPECT_EQ(p.second, -1);
}

// ─── astar ──────────────────────────────────────────────────────────────────

TEST(NavAstar, StraightLineOnOpenGrid) {
  const int rows = 1, cols = 5;
  const auto occ = grid(rows, cols, {0, 0, 0, 0, 0});
  const auto path = astar(occ.data(), rows, cols, 0, 0, 0, 4, nullptr);
  // 5 cells, flattened r,c pairs
  const std::vector<int> want = {0, 0, 0, 1, 0, 2, 0, 3, 0, 4};
  EXPECT_EQ(path, want);
}

TEST(NavAstar, StartEqualsGoal) {
  const auto occ = grid(3, 3, {0, 0, 0, 0, 0, 0, 0, 0, 0});
  const auto path = astar(occ.data(), 3, 3, 1, 1, 1, 1, nullptr);
  const std::vector<int> want = {1, 1};
  EXPECT_EQ(path, want);
}

TEST(NavAstar, DiagonalDoesNotCutCorners) {
  // A single wall at (1,2) flanks the (1,1)->(0,2) diagonal, so that shortcut
  // clips the wall's corner and is forbidden. The search must detour through
  // (0,1). A naive 8-connected A* that allowed corner-cutting would instead
  // return the single diagonal step {1,1, 0,2}.
  //  . . g       g = goal (0,2)
  //  . s X       s = start (1,1), X = wall (1,2)
  //  . . .
  const int rows = 3, cols = 3;
  const auto occ = grid(rows, cols, {0, 0, 0, 0, 0, 1, 0, 0, 0});
  const auto path = astar(occ.data(), rows, cols, 1, 1, 0, 2, nullptr);
  const std::vector<int> want = {1, 1, 0, 1, 0, 2};
  EXPECT_EQ(path, want);
}

TEST(NavAstar, UnreachableReturnsEmpty) {
  // goal walled off by a full column of obstacles
  const int rows = 3, cols = 3;
  const auto occ = grid(rows, cols, {0, 1, 0, 0, 1, 0, 0, 1, 0});
  const auto path = astar(occ.data(), rows, cols, 0, 0, 0, 2, nullptr);
  EXPECT_TRUE(path.empty());
}

TEST(NavAstar, DiagonalShortcutWhenClear) {
  // open 3x3: the cheapest route from (0,0) to (2,2) is two diagonals.
  const int rows = 3, cols = 3;
  const auto occ = grid(rows, cols, {0, 0, 0, 0, 0, 0, 0, 0, 0});
  const auto path = astar(occ.data(), rows, cols, 0, 0, 2, 2, nullptr);
  const std::vector<int> want = {0, 0, 1, 1, 2, 2};
  EXPECT_EQ(path, want);
}

// ─── simplify ───────────────────────────────────────────────────────────────

TEST(NavSimplify, StringPullsAStraightRun) {
  const int rows = 1, cols = 5;
  const auto occ = grid(rows, cols, {0, 0, 0, 0, 0});
  const std::vector<int> path = {0, 0, 0, 1, 0, 2, 0, 3, 0, 4};
  const auto s = simplify(occ.data(), rows, cols, path.data(), 5);
  const std::vector<int> want = {0, 0, 0, 4}; // collapses to endpoints
  EXPECT_EQ(s, want);
}

TEST(NavSimplify, ShortPathUnchanged) {
  const auto occ = grid(1, 3, {0, 0, 0});
  const std::vector<int> path = {0, 0, 0, 1};
  const auto s = simplify(occ.data(), 1, 3, path.data(), 2);
  EXPECT_EQ(s, path);
}

// ─── batched / threaded kernels ─────────────────────────────────────────────

namespace {

// deterministic pseudo-random occupancy (no <random> dependency)
std::vector<std::uint8_t> pseudo_grid(int rows, int cols, unsigned seed) {
  std::vector<std::uint8_t> g(rows * cols);
  unsigned x = seed * 2654435761u + 1u;
  for (int i = 0; i < rows * cols; ++i) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g[i] = (x % 100u) < 22u ? 1 : 0; // ~22% blocked
  }
  g[0] = 0;
  g[rows * cols - 1] = 0;
  return g;
}

} // namespace

TEST(NavBatch, AstarBatchIsByteIdenticalToSerial) {
  const int rows = 12, cols = 12, N = 20;
  std::vector<std::vector<std::uint8_t>> grids;
  grids.reserve(N);
  std::vector<astar_query> qs;
  for (int i = 0; i < N; ++i) {
    grids.push_back(pseudo_grid(rows, cols, 100u + i));
    astar_query q;
    q.occ = grids.back().data();
    q.start_r = i % rows;
    q.start_c = (i * 3) % cols;
    q.goal_r = (rows - 1) - (i % rows);
    q.goal_c = (cols - 1) - ((i * 5) % cols);
    q.cost = nullptr;
    qs.push_back(q);
  }
  const auto batch = astar_batch(qs, rows, cols, 4); // 4 threads
  ASSERT_EQ((int)batch.size(), N);
  for (int i = 0; i < N; ++i) {
    const auto serial = astar(qs[i].occ, rows, cols, qs[i].start_r, qs[i].start_c, qs[i].goal_r,
                              qs[i].goal_c, nullptr);
    EXPECT_EQ(batch[i], serial) << "query " << i;
  }
}

TEST(NavBatch, BuildSdfBatchIsByteIdenticalToSerial) {
  const int rows = 10, cols = 14, N = 8;
  std::vector<std::vector<std::uint8_t>> grids;
  std::vector<const std::uint8_t *> occs;
  for (int i = 0; i < N; ++i) {
    grids.push_back(pseudo_grid(rows, cols, 7u + i));
    grids.back()[i % (rows * cols)] = 1; // ensure a building exists
  }
  for (auto &g : grids)
    occs.push_back(g.data());
  const auto batch = build_sdf_batch(occs, rows, cols, 0.0, 0.0, 13.0, 9.0, 0.1, 3);
  ASSERT_EQ((int)batch.size(), N);
  for (int i = 0; i < N; ++i) {
    const auto s = build_sdf(occs[i], rows, cols, 0.0, 0.0, 13.0, 9.0, 0.1);
    EXPECT_EQ(batch[i].phi, s.phi) << "phi " << i;
    EXPECT_EQ(batch[i].normal_x, s.normal_x) << "nx " << i;
    EXPECT_EQ(batch[i].normal_y, s.normal_y) << "ny " << i;
  }
}

TEST(NavBatch, EmptyBatchIsFine) {
  std::vector<astar_query> none;
  EXPECT_TRUE(astar_batch(none, 8, 8, 4).empty());
}

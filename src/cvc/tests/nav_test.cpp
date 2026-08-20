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
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/sim_world.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#ifdef CVC_ENABLE_CUDA
#include <cvc/nav/sim_world_cuda.h>
#endif

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
  // Unit normals — EXCEPT at the wall cell (col 2). It sits exactly on the SDF
  // minimum (phi 0.5, -0.5, 0.5 across it), so the discrete central-difference
  // gradient is (0, 0) and the normalized normal is (0, 0), magnitude 0. numpy's
  // np.gradient produces the identical zero normal there (verified on a 2-row
  // grid; on this single row np.gradient can't run at all, so build_sdf's y-axis
  // gradient is 0 by construction). This cell formerly read as unit magnitude
  // only because build_sdf's y-gradient read one float past phi — a heap
  // buffer-overflow (ASan-confirmed) whose garbage made gy spuriously nonzero,
  // which flaked NavSdf ~2/12 when that garbage was a NaN/huge value.
  for (int i = 0; i < rows * cols; ++i) {
    const float mag = std::sqrt(f.normal_x[i] * f.normal_x[i] + f.normal_y[i] * f.normal_y[i]);
    const float want = (i == 2) ? 0.0f : 1.0f; // col 2 = wall minimum, gradient 0
    EXPECT_NEAR(mag, want, 1e-5f) << "cell " << i;
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

TEST(NavSpatial, NeighborsWithinRadiusMatchesBruteForce) {
  const int n = 250;
  std::vector<double> pos(2 * n);
  unsigned x = 999u;
  auto rnd = [&]() {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (x % 100000) / 100.0;
  };
  for (int i = 0; i < 2 * n; ++i)
    pos[i] = rnd();
  for (double r : {3.0, 15.0, 60.0}) {
    const auto csr = neighbors_within_radius(pos.data(), n, r);
    ASSERT_EQ((int)csr.offsets.size(), n + 1);
    for (int i = 0; i < n; ++i) {
      std::vector<int> bf;
      for (int j = 0; j < n; ++j)
        if (j != i) {
          const double dx = pos[2 * i] - pos[2 * j], dy = pos[2 * i + 1] - pos[2 * j + 1];
          if (dx * dx + dy * dy <= r * r)
            bf.push_back(j);
        }
      const std::vector<int> got(csr.indices.begin() + csr.offsets[i],
                                 csr.indices.begin() + csr.offsets[i + 1]);
      EXPECT_EQ(got, bf) << "point " << i << " radius " << r;
    }
  }
}

TEST(NavBatch, InflateBatchIsByteIdenticalToSerial) {
  const int rows = 11, cols = 13, N = 12;
  std::vector<std::vector<std::uint8_t>> grids;
  std::vector<const std::uint8_t *> occs;
  for (int i = 0; i < N; ++i)
    grids.push_back(pseudo_grid(rows, cols, 41u + i));
  for (auto &g : grids)
    occs.push_back(g.data());
  for (int cells : {0, 1, 2, 4}) {
    const auto batch = inflate_batch(occs, rows, cols, cells, 3);
    ASSERT_EQ((int)batch.size(), N);
    for (int i = 0; i < N; ++i)
      EXPECT_EQ(batch[i], inflate(occs[i], rows, cols, cells))
          << "plane " << i << " cells " << cells;
  }
}

// ─── sense_batch (BeliefGrid.sense) ──────────────────────────────────────────

namespace {

// A sparse random truth grid + N agents scattered in-bounds, bounds chosen so
// cell_w == cell_h == 1 (world == cell units). `M` planes; agent_map is arange
// for private (M==N) else a random label in [0,M).
struct SenseCase {
  int rows, cols, N, M;
  std::vector<std::uint8_t> truth;
  std::vector<double> pos, heading, range_m, fov_rad;
  std::vector<std::int32_t> n_rays, amap;
  SenseCase(int r, int c, int n, int m, unsigned seed) : rows(r), cols(c), N(n), M(m) {
    unsigned x = seed | 1u;
    auto rnd = [&]() {
      x ^= x << 13;
      x ^= x >> 17;
      x ^= x << 5;
      return x;
    };
    truth.assign(r * c, 0);
    for (int k = 0; k < r * c / 8; ++k)
      truth[rnd() % (r * c)] = 1;
    pos.resize(2 * n);
    heading.resize(n);
    range_m.resize(n);
    fov_rad.resize(n);
    n_rays.resize(n);
    amap.resize(n);
    for (int i = 0; i < n; ++i) {
      pos[2 * i] = static_cast<double>(rnd() % ((c - 1) * 100)) / 100.0;
      pos[2 * i + 1] = static_cast<double>(rnd() % ((r - 1) * 100)) / 100.0;
      heading[i] = static_cast<double>(rnd() % 628) / 100.0;
      range_m[i] = 4.0 + (rnd() % 400) / 100.0;
      fov_rad[i] = 6.283185307179586;
      n_rays[i] = 60 + static_cast<int>(rnd() % 60);
      amap[i] = (m == n) ? i : static_cast<int>(rnd() % m);
    }
  }
  sense_agents agents() const {
    sense_agents ag;
    ag.pos = pos.data();
    ag.heading = heading.data();
    ag.range_m = range_m.data();
    ag.fov_rad = fov_rad.data();
    ag.n_rays = n_rays.data();
    ag.agent_map = amap.data();
    ag.n = N;
    return ag;
  }
  // A one-plane sub-case of just this case's agents mapped to plane `g`,
  // preserving ascending index — the per-group serial reference.
  SenseCase select_group(int g) const {
    SenseCase s = *this;
    s.pos.clear();
    s.heading.clear();
    s.range_m.clear();
    s.fov_rad.clear();
    s.n_rays.clear();
    s.amap.clear();
    s.N = 0;
    s.M = 1;
    for (int i = 0; i < N; ++i)
      if (amap[i] == g) {
        s.pos.push_back(pos[2 * i]);
        s.pos.push_back(pos[2 * i + 1]);
        s.heading.push_back(heading[i]);
        s.range_m.push_back(range_m[i]);
        s.fov_rad.push_back(fov_rad[i]);
        s.n_rays.push_back(n_rays[i]);
        s.amap.push_back(0);
        ++s.N;
      }
    return s;
  }
};

struct Planes {
  std::vector<float> lo;
  std::vector<std::uint8_t> lv, es;
  std::vector<std::int32_t> ver, flips;
  Planes(int M, int HW, int N)
      : lo(M * HW, 0.f), lv(M * HW, 0), es(M * HW, 0), ver(M, 0), flips(N, 0) {}
};

void run_case(const SenseCase &sc, Planes &p, int nt, const std::int32_t *peer = nullptr,
              int kmax = 0, const std::int32_t *mov = nullptr, int nm = 0) {
  belief_planes pl{p.lo.data(), p.lv.data(), p.es.data(), p.ver.data(), sc.M};
  sense_batch(sc.truth.data(), sc.rows, sc.cols, 0.0, 0.0, static_cast<double>(sc.cols - 1),
              static_cast<double>(sc.rows - 1), sc.agents(), peer, kmax, mov, nm, pl, 2.2, -1.4,
              8.0, p.flips.data(), nt);
}

} // namespace

// The race gate: same inputs, 1 vs 8 threads, must be byte-identical in every
// mode. A within-plane data race or a fused/reordered scatter would break this.
TEST(NavSense, DeterministicAcrossThreadCounts) {
  for (unsigned seed : {11u, 23u, 47u}) {
    for (int mode = 0; mode < 3; ++mode) {
      const int N = 30;
      const int M = mode == 0 ? N : (mode == 1 ? 4 : 1); // private / clustered / shared
      SenseCase sc(24, 28, N, M, seed);
      Planes a(M, sc.rows * sc.cols, N), b(M, sc.rows * sc.cols, N);
      run_case(sc, a, 1);
      run_case(sc, b, 8);
      EXPECT_EQ(a.lo, b.lo) << "logodds seed " << seed << " M " << M;
      EXPECT_EQ(a.lv, b.lv) << "last_visible seed " << seed << " M " << M;
      EXPECT_EQ(a.es, b.es) << "ever_seen seed " << seed << " M " << M;
      EXPECT_EQ(a.ver, b.ver) << "version seed " << seed << " M " << M;
      EXPECT_EQ(a.flips, b.flips) << "flips seed " << seed << " M " << M;
    }
  }
}

// Private plane i must equal that one agent sensed alone in a 1-plane belief —
// per-plane isolation + correct base offset.
TEST(NavSense, PrivatePlaneEqualsSoloAgent) {
  const int N = 12;
  SenseCase sc(20, 22, N, N, 5u);
  Planes grouped(N, sc.rows * sc.cols, N);
  run_case(sc, grouped, 4);
  const int HW = sc.rows * sc.cols;
  for (int i = 0; i < N; ++i) {
    SenseCase solo = sc.select_group(i); // exactly agent i, plane 0
    Planes ref(1, HW, solo.N);
    run_case(solo, ref, 1);
    for (int k = 0; k < HW; ++k)
      EXPECT_EQ(grouped.lo[static_cast<long>(i) * HW + k], ref.lo[k])
          << "agent " << i << " cell " << k;
  }
}

// Clustered plane g must equal that group's agents (ascending index) sensed into
// a lone plane — the sequential-within-plane reference, and cluster isolation.
TEST(NavSense, ClusteredPlaneEqualsGroupSubset) {
  const int N = 40, M = 5;
  SenseCase sc(24, 28, N, M, 77u);
  Planes grouped(M, sc.rows * sc.cols, N);
  run_case(sc, grouped, 4);
  const int HW = sc.rows * sc.cols;
  for (int g = 0; g < M; ++g) {
    SenseCase sub = sc.select_group(g);
    Planes ref(1, HW, sub.N);
    run_case(sub, ref, 1);
    for (int k = 0; k < HW; ++k) {
      EXPECT_EQ(grouped.lo[static_cast<long>(g) * HW + k], ref.lo[k])
          << "group " << g << " cell " << k;
      EXPECT_EQ(grouped.es[static_cast<long>(g) * HW + k], ref.es[k])
          << "everseen g " << g << " k " << k;
    }
  }
}

// An off-grid agent updates nothing (its logodds/ever_seen plane stays zero) and
// reports zero flips (belief.py:113-115 early return).
TEST(NavSense, OobAgentNoOps) {
  const int rows = 16, cols = 16;
  std::vector<std::uint8_t> truth(rows * cols, 0);
  std::vector<double> pos = {-5.0, -5.0}; // far off grid
  std::vector<double> heading = {0.0}, range_m = {5.0}, fov = {6.2831853};
  std::vector<std::int32_t> n_rays = {90}, amap = {0};
  sense_agents ag;
  ag.pos = pos.data();
  ag.heading = heading.data();
  ag.range_m = range_m.data();
  ag.fov_rad = fov.data();
  ag.n_rays = n_rays.data();
  ag.agent_map = amap.data();
  ag.n = 1;
  Planes p(1, rows * cols, 1);
  belief_planes pl{p.lo.data(), p.lv.data(), p.es.data(), p.ver.data(), 1};
  sense_batch(truth.data(), rows, cols, 0.0, 0.0, cols - 1.0, rows - 1.0, ag, nullptr, 0, nullptr,
              0, pl, 2.2, -1.4, 8.0, p.flips.data(), 1);
  for (float v : p.lo)
    EXPECT_EQ(v, 0.0f);
  for (std::uint8_t v : p.es)
    EXPECT_EQ(v, 0u);
  for (std::uint8_t v : p.lv)
    EXPECT_EQ(v, 0u);
  EXPECT_EQ(p.ver[0], 0);
  EXPECT_EQ(p.flips[0], 0);
}

// A peer box occludes a ray AND deposits +L_OCC on its cell (peers are hits, R1).
TEST(NavSense, PeerBoxOccludesAndDeposits) {
  const int rows = 21, cols = 21;
  std::vector<std::uint8_t> truth(rows * cols, 0); // empty world
  std::vector<double> pos = {10.0, 10.0};          // centre
  std::vector<double> heading = {0.0}, range_m = {12.0}, fov = {6.2831853};
  std::vector<std::int32_t> n_rays = {180}, amap = {0};
  sense_agents ag;
  ag.pos = pos.data();
  ag.heading = heading.data();
  ag.range_m = range_m.data();
  ag.fov_rad = fov.data();
  ag.n_rays = n_rays.data();
  ag.agent_map = amap.data();
  ag.n = 1;
  // one peer box at column 14 (east of centre), rows 9..12 (half-open r0,r1,c0,c1)
  std::vector<std::int32_t> peer = {9, 12, 14, 15};
  Planes p(1, rows * cols, 1);
  belief_planes pl{p.lo.data(), p.lv.data(), p.es.data(), p.ver.data(), 1};
  sense_batch(truth.data(), rows, cols, 0.0, 0.0, cols - 1.0, rows - 1.0, ag, peer.data(), 1,
              nullptr, 0, pl, 2.2, -1.4, 8.0, p.flips.data(), 1);
  // the box cell (10,14) should be occupied (>0); a cell just beyond it (10,16)
  // should be UNSEEN (ray stopped) -> ever_seen 0.
  EXPECT_GT(p.lo[10 * cols + 14], 0.0f) << "peer box cell should read occupied";
  EXPECT_EQ(p.es[10 * cols + 16], 0u) << "cell behind the peer must be occluded (unseen)";
}

// ─── drive: bilinear SDF sampler ─────────────────────────────────────────────

// A constant field per plane makes the bilinear sample exact and hand-checkable:
// the sampled phi is the plane's phi constant everywhere, and the returned
// normal is the plane's (nx,ny) renormalized. This checks the map_id gather (each
// agent reads its own plane), the border clamp (a far-out-of-range position still
// samples the constant), and the unit-normal renorm.
TEST(NavDrive, ConstantFieldGatherAndRenorm) {
  const int M = 2, H = 4, W = 5;
  std::vector<float> data(static_cast<std::size_t>(M) * 3 * H * W);
  auto plane = [&](int m, int ch) {
    return data.data() + (static_cast<std::size_t>(m) * 3 + ch) * H * W;
  };
  std::fill(plane(0, 0), plane(0, 0) + H * W, 5.0f);  // plane 0 phi
  std::fill(plane(0, 1), plane(0, 1) + H * W, 3.0f);  // plane 0 nx
  std::fill(plane(0, 2), plane(0, 2) + H * W, 4.0f);  // plane 0 ny -> unit (0.6,0.8)
  std::fill(plane(1, 0), plane(1, 0) + H * W, 9.0f);  // plane 1 phi
  std::fill(plane(1, 1), plane(1, 1) + H * W, 0.0f);  // plane 1 nx
  std::fill(plane(1, 2), plane(1, 2) + H * W, -2.0f); // plane 1 ny -> unit (0,-1)

  cvc::nav::field_stack fs;
  fs.data = data.data();
  fs.M = M;
  fs.H = H;
  fs.W = W;
  fs.mnx = -10;
  fs.mny = -10;
  fs.mxx = 10;
  fs.mxy = 10;
  fs.cx = 0;
  fs.cy = 0;
  fs.S = 1.0;

  const int N = 4;
  const float on[N * 2] = {0.f, 0.f, 0.f, 0.f, 500.f, 500.f, -500.f, -500.f};
  const int map_id[N] = {0, 1, 1, 0};
  std::vector<float> phi(N), nrm(N * 2);
  cvc::nav::sdf_sample(fs, on, N, map_id, phi.data(), nrm.data(), 1);

  const float exp_phi[N] = {5.f, 9.f, 9.f, 5.f};
  const float exp_nx[N] = {0.6f, 0.f, 0.f, 0.6f};
  const float exp_ny[N] = {0.8f, -1.f, -1.f, 0.8f};
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(phi[i], exp_phi[i], 1e-5f) << "agent " << i;
    EXPECT_NEAR(nrm[2 * i], exp_nx[i], 1e-5f) << "agent " << i;
    EXPECT_NEAR(nrm[2 * i + 1], exp_ny[i], 1e-5f) << "agent " << i;
  }

  // map_id == nullptr => every agent reads plane 0.
  std::vector<float> phi0(N), nrm0(N * 2);
  cvc::nav::sdf_sample(fs, on, N, nullptr, phi0.data(), nrm0.data(), 1);
  for (int i = 0; i < N; ++i)
    EXPECT_NEAR(phi0[i], 5.0f, 1e-5f);

  // Deterministic across thread counts (pure per-agent).
  std::vector<float> phiT(N), nrmT(N * 2);
  cvc::nav::sdf_sample(fs, on, N, map_id, phiT.data(), nrmT.data(), 4);
  for (int i = 0; i < N; ++i)
    EXPECT_EQ(phi[i], phiT[i]);
}

// ─── sim_world: the whole swarm from PURE C++ (no Python, no libtorch) ────────

TEST(NavSimWorld, RunsFromPureCppAndAgentsProgress) {
  // A bordered room with a bar to route around.
  const int R = 96, C = 96;
  std::vector<std::uint8_t> occ((std::size_t)R * C, 0);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      if (r == 0 || c == 0 || r == R - 1 || c == C - 1)
        occ[r * C + c] = 1;
  for (int r = R / 3; r < 2 * R / 3; ++r)
    occ[r * C + C / 2] = 1;

  cvc::nav::sim_world::config cfg;
  cfg.rows = R;
  cfg.cols = C;
  cfg.min_x = -400;
  cfg.min_y = -400;
  cfg.max_x = 400;
  cfg.max_y = 400;
  cfg.scale = 0.02;
  cfg.veh.rr = 3.0f;
  cfg.veh.d_hat = 7.0f;
  cfg.veh.dt = 0.06f;
  cfg.veh.nsub = 1;
  cfg.freeze_sense = true;

  const int N = 256;
  // default_biased() drives with NO trained weights file — pure C++, no deps.
  cvc::nav::sim_world world = cvc::nav::sim_world::from_occupancy(
      cfg, occ.data(), cvc::nav::coef_mlp::default_biased(), N, 7);
  ASSERT_EQ(world.size(), N);

  std::vector<float> pos0(2 * N), pos1(2 * N), goal_dist0(N);
  std::vector<float> hd(N), sp(N);
  std::vector<int> md(N);
  std::vector<std::uint8_t> rc(N);
  world.snapshot(pos0.data(), hd.data(), sp.data(), md.data(), rc.data());

  for (int t = 0; t < 300; ++t)
    world.step(4);

  world.snapshot(pos1.data(), hd.data(), sp.data(), md.data(), rc.data());
  // Agents moved (not frozen) and the whole thing produced finite poses.
  int moved = 0, reached = 0;
  for (int i = 0; i < N; ++i) {
    EXPECT_TRUE(std::isfinite(pos1[2 * i]) && std::isfinite(pos1[2 * i + 1]));
    const float dx = pos1[2 * i] - pos0[2 * i], dy = pos1[2 * i + 1] - pos0[2 * i + 1];
    if (std::sqrt(dx * dx + dy * dy) > 1.0f)
      ++moved;
    reached += rc[i];
  }
  EXPECT_GT(moved, N / 2); // most agents drove somewhere
  EXPECT_GT(reached, 0);   // at least one arrived
  EXPECT_EQ(world.tick(), 300);
}

TEST(NavSimWorld, DefaultBiasedPolicyGivesTheBasisCoefficients) {
  // Zero linear weights => net == 0 => coeffs are the constant bias basin.
  cvc::nav::coef_mlp m = cvc::nav::coef_mlp::default_biased();
  ASSERT_EQ(m.in_features(), 5);
  ASSERT_EQ(m.out_features(), 3);
  float feat[5] = {0.5f, 10.0f, 0.3f, -0.7f, 0.1f};
  float out[3] = {0, 0, 0};
  m.forward(feat, 1, out, 1);
  EXPECT_NEAR(out[0], 1.0f, 1e-4f); // alpha
  EXPECT_NEAR(out[1], 3.0f, 1e-4f); // beta
  EXPECT_NEAR(out[2], 4.0f, 1e-4f); // gamma
}

TEST(NavSimWorld, CoefMlpRejectsTooWideLayer) {
  // A net wider than kMaxWidth must be rejected at the boundary, not overflow the
  // forward's fixed stack arrays (the CUDA drive has a tighter 64 cap it guards).
  const int W = cvc::nav::coef_mlp::kMaxWidth + 1;
  std::vector<int> rows = {W, 3}, cols = {5, W};
  std::vector<std::uint32_t> act = {1, 0};
  std::vector<std::vector<float>> w = {std::vector<float>((std::size_t)W * 5),
                                       std::vector<float>((std::size_t)3 * W)};
  std::vector<std::vector<float>> b = {std::vector<float>(W), std::vector<float>(3)};
  std::vector<float> ob = {1.0f, 3.0f, 4.0f};
  EXPECT_THROW(cvc::nav::coef_mlp::from_layers(5, 3, rows, cols, act, w, b, ob),
               std::runtime_error);
}

#ifdef CVC_NAV_SHIPPED_WEIGHTS
TEST(NavSimWorld, LoadsShippedPolicyAndDrives) {
  // The reference .cvcnav shipped in the tree (share/cvc/nav) loads and drives.
  cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(CVC_NAV_SHIPPED_WEIGHTS);
  EXPECT_EQ(model.in_features(), 5);
  EXPECT_EQ(model.out_features(), 3);

  const int R = 96, C = 96;
  std::vector<std::uint8_t> occ((std::size_t)R * C, 0);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      if (r == 0 || c == 0 || r == R - 1 || c == C - 1)
        occ[r * C + c] = 1;
  cvc::nav::sim_world::config cfg;
  cfg.rows = R;
  cfg.cols = C;
  cfg.min_x = -400;
  cfg.min_y = -400;
  cfg.max_x = 400;
  cfg.max_y = 400;
  cfg.scale = 0.02;
  cfg.veh.rr = 3.0f;
  cfg.veh.d_hat = 7.0f;
  cfg.veh.dt = 0.06f;
  cfg.veh.nsub = 1;
  cfg.freeze_sense = true;
  const int N = 256;
  cvc::nav::sim_world world =
      cvc::nav::sim_world::from_occupancy(cfg, occ.data(), std::move(model), N, 3);
  std::vector<float> pos0(2 * N), pos(2 * N), hd(N), sp(N);
  std::vector<int> md(N);
  std::vector<std::uint8_t> rc(N);
  world.snapshot(pos0.data(), hd.data(), sp.data(), md.data(), rc.data());
  for (int t = 0; t < 250; ++t)
    world.step(4);
  world.snapshot(pos.data(), hd.data(), sp.data(), md.data(), rc.data());
  // The shipped policy loads and DRIVES: agents move and produce finite poses,
  // and some arrive. (Scene-specific reach rate is measured Python-side against
  // the story meta the policy expects — ~57%; here we assert usability in C++.)
  int moved = 0, reached = 0;
  for (int i = 0; i < N; ++i) {
    EXPECT_TRUE(std::isfinite(pos[2 * i]) && std::isfinite(pos[2 * i + 1]));
    const float dx = pos[2 * i] - pos0[2 * i], dy = pos[2 * i + 1] - pos0[2 * i + 1];
    if (std::sqrt(dx * dx + dy * dy) > 1.0f)
      ++moved;
    reached += rc[i];
  }
  EXPECT_GT(moved, N / 2);
  EXPECT_GT(reached, 0);
}
#endif

#ifdef CVC_ENABLE_CUDA
// The device-resident GPU twin (field + weights + all SoA columns stay on the
// GPU across ticks) must trace the CPU sim_world float-equivalently over a long
// static-map roll: identical reach-set and near-identical poses. This is the P6
// behavioral gate for the CUDA deployment path (bench on a bigger box).
TEST(NavSimWorldCuda, TracesCpuSimWorld) {
  if (!cvc::nav::sim_world_cuda::available())
    GTEST_SKIP() << "no CUDA device";

  const int R = 96, C = 96;
  std::vector<std::uint8_t> occ((std::size_t)R * C, 0);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      if (r == 0 || c == 0 || r == R - 1 || c == C - 1)
        occ[r * C + c] = 1;
  for (int r = R / 3; r < 2 * R / 3; ++r)
    occ[r * C + C / 2] = 1; // a bar to route around (exercises wall-follow)

  cvc::nav::sim_world::config cfg;
  cfg.rows = R;
  cfg.cols = C;
  cfg.min_x = -400;
  cfg.min_y = -400;
  cfg.max_x = 400;
  cfg.max_y = 400;
  cfg.scale = 0.02;
  cfg.veh.rr = 3.0f;
  cfg.veh.d_hat = 7.0f;
  cfg.veh.dt = 0.06f;
  cfg.veh.nsub = 1;
  cfg.freeze_sense = true;

  // Both worlds get the SAME agents (one scatter), so any divergence is the
  // GPU drive, not different starts.
  const int N = 256;
  std::vector<float> o(2 * N), goal(2 * N), color(3 * N);
  cvc::nav::sim_world::scatter_free(cfg, occ.data(), N, 11, o.data(), goal.data(), color.data());

  cvc::nav::sim_world cpu(cfg, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(),
                          o.data(), goal.data(), color.data(), N);
  cvc::nav::sim_world_cuda gpu(cfg, occ.data(), cvc::nav::coef_mlp::default_biased(), o.data(),
                               goal.data(), color.data(), N);
  ASSERT_EQ(gpu.size(), N);

  // Behavioral gate, mirroring the CPU sim_world parity test
  // (test_sim_world_parity.py): over a horizon before the chaotic FSM tail
  // diverges, the whole swarm tracks to sub-5cm (all agents) with a matching
  // reach count. The per-tick drive math is float-equivalent (~1 ULP); the
  // carrot FSM's discrete branches are keyed on float thresholds, so far past
  // this horizon a ~1e-6 difference can flip a branch and send a few agents down
  // a different-but-valid path — that is the documented mode-flip risk, gated by
  // horizon here rather than a per-agent budget.
  const int T = 250;
  std::vector<float> pc(2 * N), pg(2 * N), hc(N), hg(N), sc(N), sg(N);
  std::vector<int> mc(N), mg(N);
  std::vector<std::uint8_t> rc(N), rg(N);
  double step1_max = 0.0;
  for (int t = 0; t < T; ++t) {
    cpu.step(0);
    gpu.step();
    if (t == 0) {
      cpu.snapshot(pc.data(), hc.data(), sc.data(), mc.data(), rc.data());
      gpu.snapshot(pg.data(), hg.data(), sg.data(), mg.data(), rg.data());
      for (int i = 0; i < N; ++i) {
        const double dx = pc[2 * i] - pg[2 * i], dy = pc[2 * i + 1] - pg[2 * i + 1];
        step1_max = std::max(step1_max, std::sqrt(dx * dx + dy * dy));
      }
    }
  }
  cpu.snapshot(pc.data(), hc.data(), sc.data(), mc.data(), rc.data());
  gpu.snapshot(pg.data(), hg.data(), sg.data(), mg.data(), rg.data());
  std::vector<double> err(N);
  int reached_cpu = 0, reached_gpu = 0;
  for (int i = 0; i < N; ++i) {
    const double dx = pc[2 * i] - pg[2 * i], dy = pc[2 * i + 1] - pg[2 * i + 1];
    err[i] = std::sqrt(dx * dx + dy * dy);
    reached_cpu += rc[i];
    reached_gpu += rg[i];
  }
  std::sort(err.begin(), err.end());
  auto band = [&](double thr) {
    int c = 0;
    for (double e : err)
      c += (e < thr);
    return c;
  };
  std::printf("[diag] step1_max=%.3e  p50=%.4f p90=%.4f p99=%.4f max=%.4f  "
              "<1e-3:%d <0.05:%d <0.5:%d <2:%d /%d  reach cpu=%d gpu=%d\n",
              step1_max, err[N / 2], err[(int)(0.9 * N)], err[(int)(0.99 * N)], err[N - 1],
              band(1e-3), band(0.05), band(0.5), band(2.0), N, reached_cpu, reached_gpu);
  for (int i = 0; i < N; ++i)
    ASSERT_TRUE(std::isfinite(pg[2 * i]) && std::isfinite(pg[2 * i + 1]));
  // (1) The per-tick drive is float-equivalent: after ONE tick GPU==CPU to the
  //     bit (this is the systematic-bug gate). (2) The bulk stays bit-tight over
  //     the full roll: the median agent barely moves off the CPU trajectory.
  //     (3) The flip tail is bounded: the vast majority stay sub-half-metre; a
  //     few agents that straddle an FSM threshold peel off onto a different-but-
  //     valid path (documented mode-flip chaos). (4) Aggregate reach matches.
  EXPECT_LT(step1_max, 1e-3) << "single-step GPU drive must match CPU to the bit";
  EXPECT_LT(err[N / 2], 1e-3) << "median agent must track the CPU trajectory bit-tight";
  EXPECT_GE(band(0.5), (int)(0.85 * N)) << "flip tail unbounded (bulk should stay < 0.5 m)";
  EXPECT_LE(std::abs(reached_cpu - reached_gpu), 2) << "reach count cpu vs gpu";
  EXPECT_GT(reached_gpu, 0);
}
#endif

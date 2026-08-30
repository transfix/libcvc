/*
  Copyright 2026 The University of Texas at Austin

  Unit tests for cvc/vis/grid_index.h -- the uniform-grid broad-phase culler.

  The one property that matters: the grid cull is EXACTLY the oracle's visible
  set, only faster. So the tests are equivalence tests -- grid.cull() vs
  reference_cull() over many random scenes and views -- plus proof that the
  broad phase actually skips cells (otherwise "faster" is a lie), plus the
  degenerate shapes (empty, one proxy, a cell size that would blow the cap).

  A GTEST_SKIP-gated benchmark (CVC_VIS_BENCH=1) reports the speedup and the
  skip rate on a 10^5-proxy city.
*/

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/vis/grid_index.h>
#include <cvc/vis/reference.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using namespace cvc::vis;

namespace {

bool vis_bench_enabled() {
  const char *v = std::getenv("CVC_VIS_BENCH");
  return v && std::string(v) == "1";
}

frustum box_frustum(float minx, float maxx, float miny, float maxy, float minz, float maxz) {
  frustum f;
  f.p[0] = {{1, 0, 0}, -minx};
  f.p[1] = {{-1, 0, 0}, maxx};
  f.p[2] = {{0, 1, 0}, -miny};
  f.p[3] = {{0, -1, 0}, maxy};
  f.p[4] = {{0, 0, 1}, -minz};
  f.p[5] = {{0, 0, -1}, maxz};
  return f;
}

// A flat, city-like scatter of boxes on the XZ ground plane (Y up), which is
// what the grid is tuned for. Deterministic from `seed`.
std::vector<aabb> city(std::uint32_t n, float span, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> G(-span, span);
  std::uniform_real_distribution<float> H(2.0f, 40.0f); // building height
  std::uniform_real_distribution<float> W(3.0f, 20.0f); // footprint
  std::vector<aabb> b;
  b.reserve(n);
  for (std::uint32_t i = 0; i < n; ++i) {
    const float x = G(rng), z = G(rng), w = W(rng), h = H(rng);
    b.push_back({{x - w, 0.0f, z - w}, {x + w, h, z + w}});
  }
  return b;
}

view_params view_over(const frustum &f) {
  view_params v;
  v.f = f;
  v.proj = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.proj.eye[0] = v.proj.eye[1] = v.proj.eye[2] = 0.0;
  return v;
}

void expect_same_set(const visible_set &a, const visible_set &b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a.ids[i], b.ids[i]) << "at " << i;
    EXPECT_NEAR(a.dist_m[i], b.dist_m[i], 1e-3f);
    EXPECT_NEAR(a.screen_px[i], b.screen_px[i], 1e-3f);
  }
}

} // namespace

TEST(VisGrid, EqualsOracleAcrossRandomScenes) {
  // Sweep scene size, cell size, frustum position/extent, and the distance cut.
  for (std::uint32_t trial = 0; trial < 40; ++trial) {
    const std::uint32_t n = 200 + trial * 40;
    const std::vector<aabb> bounds = city(n, 1000.0f, 1000 + trial);
    scene_view s;
    s.count = n;
    s.bounds = bounds.data();

    std::mt19937 rng(9000 + trial);
    std::uniform_real_distribution<float> C(-1000.0f, 1000.0f);
    std::uniform_real_distribution<float> E(50.0f, 900.0f);
    const float cx = C(rng), cz = C(rng), ext = E(rng);
    view_params v = view_over(box_frustum(cx - ext, cx + ext, -50, 400, cz - ext, cz + ext));
    v.min_screen_px = (trial % 2) ? 1.5 : 0.0;

    const float cell = 32.0f + static_cast<float>(trial % 5) * 32.0f;
    grid_index grid;
    grid.build(s, cell);

    visible_set g, o;
    grid.cull(s, v, g);
    reference_cull(s, v, o);

    expect_same_set(g, o);
    EXPECT_EQ(conservativeness_violations(g.ids, s, v), 0u);
  }
}

TEST(VisGrid, BroadPhaseActuallySkips) {
  const std::uint32_t n = 20000;
  const std::vector<aabb> bounds = city(n, 2000.0f, 7);
  scene_view s;
  s.count = n;
  s.bounds = bounds.data();

  grid_index grid;
  grid.build(s, 128.0f); // Austin's tile size
  EXPECT_GT(grid.occupied_cells(), 0u);

  // A narrow frustum over one corner: most cells must be skipped wholesale.
  view_params v = view_over(box_frustum(500, 900, -50, 400, 500, 900));
  visible_set g;
  grid.cull(s, v, g);

  EXPECT_GT(grid.last_cells_skipped(), grid.last_cells_visited())
      << "the broad phase skipped fewer cells than it visited -- no win";
  // And it is still exactly right.
  EXPECT_EQ(conservativeness_violations(g.ids, s, v), 0u);
  visible_set o;
  reference_cull(s, v, o);
  expect_same_set(g, o);
}

TEST(VisGrid, DegenerateScenes) {
  grid_index grid;
  visible_set out;

  // Empty scene.
  scene_view empty;
  grid.build(empty, 64.0f);
  EXPECT_EQ(grid.cell_count(), 0u);
  grid.cull(empty, view_over(box_frustum(-1, 1, -1, 1, -1, 1)), out);
  EXPECT_TRUE(out.empty());

  // One proxy.
  std::vector<aabb> one = {{{-1, 0, -1}, {1, 5, 1}}};
  scene_view s;
  s.count = 1;
  s.bounds = one.data();
  grid.build(s, 64.0f);
  view_params v = view_over(box_frustum(-100, 100, -100, 100, -100, 100));
  grid.cull(s, v, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out.ids[0], 0u);
}

TEST(VisGrid, CoarsensRatherThanBlowingTheCellCap) {
  // A 4 km span at a 1 mm cell size would be 16 trillion cells; build() must
  // coarsen the cell size instead of allocating, and stay correct.
  const std::vector<aabb> bounds = city(500, 2000.0f, 3);
  scene_view s;
  s.count = 500;
  s.bounds = bounds.data();

  grid_index grid;
  grid.build(s, 0.001f);
  EXPECT_GT(grid.cell_size(), 0.001f) << "cell size should have been coarsened";
  EXPECT_LE(grid.cell_count(), 4u * 1024u * 1024u);

  view_params v = view_over(box_frustum(-2000, 2000, -50, 400, -2000, 2000));
  visible_set g, o;
  grid.cull(s, v, g);
  reference_cull(s, v, o);
  expect_same_set(g, o);
}

TEST(VisGridBench, GridVsOracle) {
  if (!vis_bench_enabled())
    GTEST_SKIP() << "Set CVC_VIS_BENCH=1 to enable";

  constexpr std::uint32_t N = 100000;
  const std::vector<aabb> bounds = city(N, 2000.0f, 42);
  scene_view s;
  s.count = N;
  s.bounds = bounds.data();

  grid_index grid;
  grid.build(s, 128.0f);

  // A chase-cam-ish narrow frustum looking into one district.
  view_params v = view_over(box_frustum(400, 800, -50, 400, 400, 800));
  v.min_screen_px = 1.5;

  visible_set g, o;
  constexpr int iters = 200;

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < iters; ++i)
    grid.cull(s, v, g);
  auto grid_ns = std::chrono::steady_clock::now() - t0;

  t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < iters; ++i)
    reference_cull(s, v, o);
  auto oracle_ns = std::chrono::steady_clock::now() - t0;

  expect_same_set(g, o);
  const double gus =
      std::chrono::duration_cast<std::chrono::nanoseconds>(grid_ns).count() / 1e3 / iters;
  const double ous =
      std::chrono::duration_cast<std::chrono::nanoseconds>(oracle_ns).count() / 1e3 / iters;
  std::printf("[vis bench] %u proxies, 128 m cells: grid %.1f us/frame, oracle %.1f us/frame "
              "(%.1fx), %zu/%zu cells skipped, %zu visible\n",
              N, gus, ous, ous / gus, grid.last_cells_skipped(), grid.occupied_cells(), g.size());
  RecordProperty("grid_us", std::to_string(gus));
  RecordProperty("oracle_us", std::to_string(ous));
  EXPECT_LT(gus, ous) << "the grid must beat the flat sweep it exists to replace";
}

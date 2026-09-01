// nav_common_test — unit tests for the bounds-safe compositing that backs the finale's
// 2-D picture-in-picture minimap. These reproduce the exact heap overflow that crashed
// nav_finale (a minimap larger than the frame it is drawn onto, and off-map agent dots)
// and prove nav_common's blit_clamped / plot_disc contain every write. The buffer is
// fenced with sentinel bytes on both sides; an out-of-bounds write corrupts a fence.

#include "nav_common.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/nav/grid_nav.h>
#include <gtest/gtest.h>
#include <vector>

namespace {
constexpr int kGuard = 8192;
constexpr unsigned char kSentinel = 0xAB;

// A dw*dh*3 RGB canvas fenced by kGuard sentinel bytes on each side. dst() points at
// the canvas; fences_intact() fails if any write landed outside [dst, dst+dw*dh*3).
struct Fenced {
  std::vector<unsigned char> buf;
  int dw, dh;
  Fenced(int w, int h)
      : buf(static_cast<std::size_t>(2) * kGuard + static_cast<std::size_t>(w) * h * 3, kSentinel),
        dw(w), dh(h) {}
  unsigned char *dst() { return buf.data() + kGuard; }
  std::size_t body() const { return static_cast<std::size_t>(dw) * dh * 3; }
  ::testing::AssertionResult fences_intact() const {
    for (int i = 0; i < kGuard; ++i)
      if (buf[i] != kSentinel)
        return ::testing::AssertionFailure() << "underflow " << (kGuard - i) << " bytes before dst";
    for (std::size_t i = kGuard + body(); i < buf.size(); ++i)
      if (buf[i] != kSentinel)
        return ::testing::AssertionFailure()
               << "overflow " << (i - (kGuard + body())) << " bytes past dst";
    return ::testing::AssertionSuccess();
  }
};
} // namespace

// The literal crash: a 180x180 minimap composited onto a frame far too small to hold
// it. The old inline copy wrote past the frame buffer -> heap corruption -> SIGSEGV.
TEST(NavCommonBlit, OversizedSourceOntoTinyFrameStaysInBounds) {
  Fenced f(48, 32); // smaller than the 180-px minimap in both dimensions
  std::vector<unsigned char> src(static_cast<std::size_t>(3) * 180 * 180, 200);
  navdemo::blit_clamped(f.dst(), f.dw, f.dh, src.data(), 180, 180, 3, f.dw - 12, 8);
  EXPECT_TRUE(f.fences_intact());
  // The visible top-left of the source still gets copied.
  EXPECT_EQ(f.dst()[(static_cast<std::size_t>(8) * f.dw + (f.dw - 12)) * 3], 200);
}

TEST(NavCommonBlit, NegativeOriginAndFullyOffscreenStayInBounds) {
  Fenced f(64, 48);
  std::vector<unsigned char> src(static_cast<std::size_t>(3) * 100 * 100, 150);
  navdemo::blit_clamped(f.dst(), f.dw, f.dh, src.data(), 100, 100, 3, -40, -30); // clipped top-left
  EXPECT_TRUE(f.fences_intact());
  navdemo::blit_clamped(f.dst(), f.dw, f.dh, src.data(), 100, 100, 3, 5000,
                        5000); // fully offscreen
  EXPECT_TRUE(f.fences_intact());
  navdemo::blit_clamped(f.dst(), f.dw, f.dh, src.data(), 100, 100, 3, 5000,
                        -5000); // fully offscreen
  EXPECT_TRUE(f.fences_intact());
}

TEST(NavCommonBlit, DimScalesSourceChannels) {
  Fenced f(16, 16);
  std::vector<unsigned char> src(static_cast<std::size_t>(3) * 4 * 4, 200);
  navdemo::blit_clamped(f.dst(), f.dw, f.dh, src.data(), 4, 4, 3, 2, 2, 0.5);
  EXPECT_TRUE(f.fences_intact());
  EXPECT_EQ(f.dst()[(static_cast<std::size_t>(2) * f.dw + 2) * 3], 100); // 200 * 0.5
}

TEST(NavCommonBlit, PlotDiscOutOfBoundsCentreAndHugeRadiusStayInBounds) {
  Fenced f(40, 40);
  navdemo::plot_disc(f.dst(), f.dw, f.dh, 1000, -500, 30, 255, 0, 0); // centre far off-screen
  navdemo::plot_disc(f.dst(), f.dw, f.dh, -1, -1, 100, 0, 255, 0);    // huge radius over a corner
  EXPECT_TRUE(f.fences_intact());
  // An in-bounds disc actually writes its centre.
  navdemo::plot_disc(f.dst(), f.dw, f.dh, 20, 20, 2, 10, 20, 30);
  EXPECT_TRUE(f.fences_intact());
  EXPECT_EQ(f.dst()[(static_cast<std::size_t>(20) * f.dw + 20) * 3 + 1], 20);
}

TEST(NavCommonBlit, PlotLineClipsAndWritesInBounds) {
  Fenced f(48, 32);
  navdemo::plot_line(f.dst(), f.dw, f.dh, -100, -50, 200, 90, 255, 0, 0);     // crosses the frame
  navdemo::plot_line(f.dst(), f.dw, f.dh, 5000, 5000, 6000, 6000, 0, 255, 0); // fully offscreen
  EXPECT_TRUE(f.fences_intact());
  navdemo::plot_line(f.dst(), f.dw, f.dh, 2, 2, 20, 2, 9, 8, 7); // in-bounds horizontal
  EXPECT_TRUE(f.fences_intact());
  EXPECT_EQ(f.dst()[(static_cast<std::size_t>(2) * f.dw + 2) * 3], 9);
  EXPECT_EQ(f.dst()[(static_cast<std::size_t>(2) * f.dw + 20) * 3], 9);
}

// occupancy_from_model must always produce an nx*ny grid and never index out of it.
TEST(NavCommonOccupancy, RasterizesTriangleWithinGrid) {
  navdemo::Bounds b{0, 0, 100, 100};
  cvc::geometry g;
  g.points().push_back({10, 10, 0});
  g.points().push_back({90, 10, 0});
  g.points().push_back({50, 90, 0});
  g.tris().push_back({0, 1, 2});
  const int nx = 64, ny = 64;
  const auto occ = navdemo::occupancy_from_model(g, b, nx, ny, 0);
  ASSERT_EQ(occ.size(), static_cast<std::size_t>(nx) * ny);
  long n = 0;
  for (auto v : occ)
    n += (v != 0);
  EXPECT_GT(n, 0);       // the triangle marked some cells
  EXPECT_LT(n, nx * ny); // but not the whole grid
  EXPECT_EQ(occ[0], 0);  // the (0,0) corner is outside the triangle
  // Dilation only grows the footprint.
  const auto occ1 = navdemo::occupancy_from_model(g, b, nx, ny, 1);
  ASSERT_EQ(occ1.size(), occ.size());
  long n1 = 0;
  for (auto v : occ1)
    n1 += (v != 0);
  EXPECT_GE(n1, n);
}

TEST(NavCommonOccupancy, EmptyMeshAndDegenerateBoundsAreSafe) {
  cvc::geometry g; // no triangles
  navdemo::Bounds ok{0, 0, 10, 10}, bad{0, 0, 0, 0};
  EXPECT_EQ(navdemo::occupancy_from_model(g, ok, 8, 8, 0).size(), 64u);
  EXPECT_EQ(navdemo::occupancy_from_model(g, bad, 8, 8, 0).size(), 64u); // degenerate -> all free
  EXPECT_EQ(navdemo::occupancy_from_model(g, ok, 1, 1, 0).size(), 1u);   // too small -> no crash
}

// ── plan_route standoff ──────────────────────────────────────────────────────
//
// The global spine is what the reactive drive follows, so a spine that scrapes
// the corners means the drive spends the whole run fighting its own barrier.

namespace {

// Cells to the nearest blocked cell, from the same EDT clearance_cost uses.
std::vector<double> clearance_of(const std::vector<std::uint8_t> &occ, int rows, int cols) {
  const auto d2 = cvc::nav::edt2_squared(occ.data(), rows, cols);
  std::vector<double> out(d2.size());
  for (std::size_t i = 0; i < d2.size(); ++i)
    out[i] = std::sqrt(d2[i]);
  return out;
}

// Worst clearance ALONG the polyline, sampled densely. Sampling only the
// waypoints flatters a string-pulled route: it has few of them, so the minimum
// skips the wall-hugging segments in between.
double worst_along(const navdemo::Route &rt, const std::vector<double> &clr,
                   const navdemo::Bounds &b, int rows, int cols) {
  double worst = 1e9;
  for (std::size_t k = 0; k + 1 < rt.wp.size(); ++k) {
    const auto &p0 = rt.wp[k], &p1 = rt.wp[k + 1];
    const int steps = 64;
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const double x = p0[0] + t * (p1[0] - p0[0]), y = p0[1] + t * (p1[1] - p0[1]);
      int c = static_cast<int>(std::lround((x - b.min_x) / (b.max_x - b.min_x) * (cols - 1)));
      int r = static_cast<int>(std::lround((y - b.min_y) / (b.max_y - b.min_y) * (rows - 1)));
      c = std::max(0, std::min(cols - 1, c));
      r = std::max(0, std::min(rows - 1, r));
      worst = std::min(worst, clr[static_cast<std::size_t>(r) * cols + c]);
    }
  }
  return worst;
}

} // namespace

TEST(NavCommonRoute, StandoffKeepsMoreClearanceThanTheShortestSpine) {
  const int N = 64;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(N) * N, 0);
  for (int r = 20; r < 44; ++r)
    for (int c = 28; c < 36; ++c)
      occ[r * N + c] = 1;
  const navdemo::Bounds b{0, 0, 64, 64};
  const auto clr = clearance_of(occ, N, N);

  const auto plain = navdemo::plan_route(occ.data(), N, N, b, 32, 5, 32, 58);
  const auto stand = navdemo::plan_route(occ.data(), N, N, b, 32, 5, 32, 58, 0, 6.0, 1.5);
  ASSERT_GE(plain.wp.size(), 2u);
  ASSERT_GE(stand.wp.size(), 2u);
  EXPECT_GT(worst_along(stand, clr, b, N, N), worst_along(plain, clr, b, N, N) + 1.0);
}

TEST(NavCommonRoute, StandoffRoutesAGapThatDilationWouldSeal) {
  // The reason to prefer the surcharge over inflate_cells. A 4-cell alley with
  // inflate_cells=6 stops existing; the surcharge just makes it expensive.
  const int N = 40;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(N) * N, 0);
  for (int r = 0; r < N; ++r)
    for (int c = 0; c < N; ++c)
      if (c < 18 || c >= 22)
        occ[r * N + c] = 1;
  const navdemo::Bounds b{0, 0, 40, 40};

  const auto stand = navdemo::plan_route(occ.data(), N, N, b, 20, 1, 20, 38, 0, 6.0, 1.5);
  // More than the bare goal fallback means A* actually found a way through.
  EXPECT_GT(stand.wp.size(), 1u) << "the surcharge must not strand a sub-standoff corridor";
}

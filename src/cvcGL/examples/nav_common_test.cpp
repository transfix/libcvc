// nav_common_test — unit tests for the bounds-safe compositing that backs the finale's
// 2-D picture-in-picture minimap. These reproduce the exact heap overflow that crashed
// nav_finale (a minimap larger than the frame it is drawn onto, and off-map agent dots)
// and prove nav_common's blit_clamped / plot_disc contain every write. The buffer is
// fenced with sentinel bytes on both sides; an out-of-bounds write corrupts a fence.

#include "nav_common.h"

#include <cstddef>
#include <cstdint>
#include <cvc/geometry/geometry.h>
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

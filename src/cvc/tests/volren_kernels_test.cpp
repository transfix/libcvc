// Unit tests for the cvc::volren detail kernels: the raw-buffer grid sampler,
// trilinear interpolation, the quadratic-B-spline gradient, the marching-cubes
// case tables, and the per-cell ray/isosurface intersection.
//
// All grids here are plain std::vector buffers viewed through grid_sampler
// (a non-owning view), so no cvc::app / cvc::volume is needed.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/volren/detail/cell_intersect.h>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volren/detail/occlusion.h>
#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/detail/shading.h>
#include <cvc/volren/detail/shadow_map.h>
#include <cvc/volren/detail/spline_gradient.h>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <vector>

using cvc::volren::ray;
using cvc::volren::vec3d;
using cvc::volren::detail::cube_edges;
using cvc::volren::detail::extract_contour;
using cvc::volren::detail::grid_sampler;
using cvc::volren::detail::in_triangle;
using cvc::volren::detail::intersect_isosurface_in_cell;
using cvc::volren::detail::intersect_triangle;
using cvc::volren::detail::mc_cell;
using cvc::volren::detail::mc_edges;
using cvc::volren::detail::mc_triangle;
using cvc::volren::detail::mc_vertex_from_binary;
using cvc::volren::detail::spline_gradient_cache;
using cvc::volren::detail::tri_cases;
using cvc::volren::detail::trilinear;

namespace {

// Fill a 4x4x4 buffer of T with val(i,j,k) = base + i*si + j*sj + k*sk.
template <typename T> std::vector<T> pattern4(double base, double si, double sj, double sk) {
  std::vector<T> buf(4 * 4 * 4);
  for (int k = 0; k < 4; ++k)
    for (int j = 0; j < 4; ++j)
      for (int i = 0; i < 4; ++i)
        buf[i + 4 * j + 16 * k] = T(base + i * si + j * sj + k * sk);
  return buf;
}

grid_sampler make_view(const void *data, cvc::data_type type, std::int64_t dim,
                       vec3d minb = {0, 0, 0}, vec3d span = {1, 1, 1}) {
  grid_sampler g;
  g.data = reinterpret_cast<const unsigned char *>(data);
  g.type = type;
  g.dimx = g.dimy = g.dimz = dim;
  g.minb = minb;
  g.span = span;
  return g;
}

// VTK MC vertex convention used by mc_edges / tri_cases.
constexpr int kVtkVertexCoord[8][3] = {
    {0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}, {0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1},
};

// A cell whose density is linear in local x: f = f0 + gain * x_local.
// Cell id (1,2,3) in a grid with orig (-0.5,-0.5,-0.5), anisotropic span.
mc_cell make_linear_x_cell(float f0, float gain) {
  mc_cell cell;
  cell.id[0] = 1;
  cell.id[1] = 2;
  cell.id[2] = 3;
  cell.orig = {-0.5, -0.5, -0.5};
  cell.span = {0.25, 0.2, 0.125};
  float vals[8];
  for (int b = 0; b < 8; ++b)
    vals[b] = f0 + gain * float(b & 1); // bit0 of the binary corner index is x
  cell.from_binary_corners(vals);
  return cell;
}

} // namespace

// ============================================================================
// grid_sampler
// ============================================================================

TEST(GridSampler, AtReadsUCharBuffer) {
  const auto buf = pattern4<unsigned char>(0, 1, 4, 16); // max 63, fits
  const grid_sampler g = make_view(buf.data(), cvc::UChar, 4);
  EXPECT_NEAR(g.at(0, 0, 0), 0.f, 0.f);
  EXPECT_NEAR(g.at(3, 0, 0), 3.f, 0.f);
  EXPECT_NEAR(g.at(1, 2, 3), 1.f + 8.f + 48.f, 0.f);
  EXPECT_NEAR(g.at(3, 3, 3), 63.f, 0.f);
}

TEST(GridSampler, AtReadsUShortBuffer) {
  const auto buf = pattern4<std::uint16_t>(0, 1, 10, 100);
  const grid_sampler g = make_view(buf.data(), cvc::UShort, 4);
  EXPECT_NEAR(g.at(2, 0, 0), 2.f, 0.f);
  EXPECT_NEAR(g.at(0, 3, 0), 30.f, 0.f);
  EXPECT_NEAR(g.at(0, 0, 2), 200.f, 0.f);
  EXPECT_NEAR(g.at(3, 1, 2), 213.f, 0.f);
}

TEST(GridSampler, AtReadsFloatBuffer) {
  const auto buf = pattern4<float>(0.5, 1, 10, 100);
  const grid_sampler g = make_view(buf.data(), cvc::Float, 4);
  EXPECT_NEAR(g.at(0, 0, 0), 0.5f, 0.f);
  EXPECT_NEAR(g.at(1, 2, 3), 321.5f, 0.f);
  EXPECT_NEAR(g.at(3, 3, 3), 333.5f, 0.f);
}

TEST(GridSampler, AtReadsDoubleBuffer) {
  const auto buf = pattern4<double>(0.25, 1, 10, 100);
  const grid_sampler g = make_view(buf.data(), cvc::Double, 4);
  EXPECT_NEAR(g.at(0, 0, 0), 0.25f, 1e-6f);
  EXPECT_NEAR(g.at(2, 1, 3), 312.25f, 1e-4f);
  EXPECT_NEAR(g.at(3, 3, 3), 333.25f, 1e-4f);
}

TEST(GridSampler, AtReadsIntBufferIncludingNegatives) {
  const auto buf = pattern4<std::int32_t>(-50, 1, 10, 100);
  const grid_sampler g = make_view(buf.data(), cvc::Int, 4);
  EXPECT_NEAR(g.at(0, 0, 0), -50.f, 0.f);
  EXPECT_NEAR(g.at(3, 0, 0), -47.f, 0.f);
  EXPECT_NEAR(g.at(0, 2, 1), 70.f, 0.f);
  EXPECT_NEAR(g.at(3, 3, 3), 283.f, 0.f);
}

TEST(GridSampler, AtClampedClampsOutOfRangeIndicesToEdges) {
  const auto buf = pattern4<float>(0, 1, 10, 100);
  const grid_sampler g = make_view(buf.data(), cvc::Float, 4);
  EXPECT_NEAR(g.at_clamped(-3, 1, 2), g.at(0, 1, 2), 0.f);
  EXPECT_NEAR(g.at_clamped(5, 3, 7), g.at(3, 3, 3), 0.f);
  EXPECT_NEAR(g.at_clamped(2, -1, 4), g.at(2, 0, 3), 0.f);
  EXPECT_NEAR(g.at_clamped(-1, -1, -1), g.at(0, 0, 0), 0.f);
  // In-range indices pass through untouched.
  EXPECT_NEAR(g.at_clamped(1, 2, 3), g.at(1, 2, 3), 0.f);
}

TEST(GridSampler, CornersFollowBinaryBitOrdering) {
  const auto buf = pattern4<float>(0, 1, 10, 100); // val = i + 10j + 100k
  const grid_sampler g = make_view(buf.data(), cvc::Float, 4);

  // Cell (0,0,0): corner index bit0 = +i, bit1 = +j, bit2 = +k.
  float c[8];
  g.corners(0, 0, 0, c);
  const float expect0[8] = {0, 1, 10, 11, 100, 101, 110, 111};
  for (int v = 0; v < 8; ++v)
    EXPECT_NEAR(c[v], expect0[v], 0.f) << "corner " << v;

  // Interior cell (2,1,2).
  g.corners(2, 1, 2, c);
  const float expect1[8] = {212, 213, 222, 223, 312, 313, 322, 323};
  for (int v = 0; v < 8; ++v)
    EXPECT_NEAR(c[v], expect1[v], 0.f) << "corner " << v;
}

TEST(GridSampler, LocalWeightsMapWorldPointIntoUnitCube) {
  const auto buf = pattern4<float>(0, 1, 10, 100);
  const grid_sampler g =
      make_view(buf.data(), cvc::Float, 4, {-0.5, -0.5, -0.5}, {0.25, 0.5, 0.125});
  const vec3d p{-0.5 + (1 + 0.25) * 0.25, -0.5 + (2 + 0.75) * 0.5, -0.5 + (0 + 0.5) * 0.125};
  float w[3];
  g.local_weights(p, 1, 2, 0, w);
  EXPECT_NEAR(w[0], 0.25f, 1e-6f);
  EXPECT_NEAR(w[1], 0.75f, 1e-6f);
  EXPECT_NEAR(w[2], 0.5f, 1e-6f);

  // A cell-corner point yields weight 0.
  g.local_weights(vec3d{-0.5 + 2 * 0.25, -0.5 + 1 * 0.5, -0.5 + 2 * 0.125}, 2, 1, 2, w);
  EXPECT_NEAR(w[0], 0.f, 1e-6f);
  EXPECT_NEAR(w[1], 0.f, 1e-6f);
  EXPECT_NEAR(w[2], 0.f, 1e-6f);
}

TEST(GridSampler, CellIndexAcceptsCellsZeroThroughDimMinusTwo) {
  const auto buf = pattern4<float>(0, 1, 10, 100);
  const grid_sampler g =
      make_view(buf.data(), cvc::Float, 4, {-0.5, -0.5, -0.5}, {0.25, 0.25, 0.25});
  std::int64_t idx[3];

  // Interior of cell (0,0,0).
  ASSERT_TRUE(g.cell_index(vec3d{-0.4, -0.4, -0.4}, idx));
  EXPECT_EQ(idx[0], 0);
  EXPECT_EQ(idx[1], 0);
  EXPECT_EQ(idx[2], 0);

  // Interior of the last cell (dim-2 = 2): ratio dim-1.5 truncates to dim-2.
  ASSERT_TRUE(g.cell_index(vec3d{-0.5 + 2.5 * 0.25, -0.5 + 2.5 * 0.25, -0.5 + 2.5 * 0.25}, idx));
  EXPECT_EQ(idx[0], 2);
  EXPECT_EQ(idx[1], 2);
  EXPECT_EQ(idx[2], 2);

  // Mixed per-axis cells.
  ASSERT_TRUE(g.cell_index(vec3d{-0.5 + 0.5 * 0.25, -0.5 + 1.5 * 0.25, -0.5 + 2.5 * 0.25}, idx));
  EXPECT_EQ(idx[0], 0);
  EXPECT_EQ(idx[1], 1);
  EXPECT_EQ(idx[2], 2);
}

TEST(GridSampler, CellIndexTruncatesPointsJustLeftOfMinIntoCellZero) {
  const auto buf = pattern4<float>(0, 1, 10, 100);
  const grid_sampler g =
      make_view(buf.data(), cvc::Float, 4, {-0.5, -0.5, -0.5}, {0.25, 0.25, 0.25});
  std::int64_t idx[3];

  // Epsilon left of minb: (p - minb)/span is a tiny negative that truncates
  // to 0 -- accepted into cell 0 (the legacy box-face robustness).
  ASSERT_TRUE(g.cell_index(vec3d{-0.5 - 1e-9, -0.5 - 1e-9, -0.5 - 1e-9}, idx));
  EXPECT_EQ(idx[0], 0);
  EXPECT_EQ(idx[1], 0);
  EXPECT_EQ(idx[2], 0);

  // Anywhere within one span left still truncates into cell 0.
  ASSERT_TRUE(g.cell_index(vec3d{-0.5 - 0.5 * 0.25, -0.4, -0.4}, idx));
  EXPECT_EQ(idx[0], 0);
}

TEST(GridSampler, CellIndexRejectsPointsBeyondOneSpanOutOrPastMax) {
  const auto buf = pattern4<float>(0, 1, 10, 100);
  const grid_sampler g =
      make_view(buf.data(), cvc::Float, 4, {-0.5, -0.5, -0.5}, {0.25, 0.25, 0.25});
  std::int64_t idx[3];

  // More than one span left of minb truncates to -1: rejected.
  EXPECT_FALSE(g.cell_index(vec3d{-0.5 - 1.5 * 0.25, -0.4, -0.4}, idx));
  // Past the max face (ratio >= dim-1): rejected, per axis.
  EXPECT_FALSE(g.cell_index(vec3d{-0.5 + 3.5 * 0.25, -0.4, -0.4}, idx));
  EXPECT_FALSE(g.cell_index(vec3d{-0.4, -0.5 + 3.5 * 0.25, -0.4}, idx));
  EXPECT_FALSE(g.cell_index(vec3d{-0.4, -0.4, -0.5 + 3.5 * 0.25}, idx));
}

// ============================================================================
// trilinear
// ============================================================================

TEST(Trilinear, ReturnsExactCornerValuesAtUnitWeights) {
  const float v[8] = {5.f, -3.f, 7.f, 2.f, 11.f, 4.f, -8.f, 9.f};
  for (int c = 0; c < 8; ++c) {
    const float w[3] = {float(c & 1), float((c >> 1) & 1), float((c >> 2) & 1)};
    EXPECT_NEAR(trilinear(w, v), v[c], 0.f) << "corner " << c;
  }
}

TEST(Trilinear, ReproducesAffineFieldAtArbitraryWeights) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uni(0.f, 1.f);
  const float a = 1.5f, b = 2.25f, c = -3.5f, d = 0.75f;
  float v[8];
  for (int corner = 0; corner < 8; ++corner)
    v[corner] =
        a + b * float(corner & 1) + c * float((corner >> 1) & 1) + d * float((corner >> 2) & 1);
  for (int trial = 0; trial < 20; ++trial) {
    const float w[3] = {uni(rng), uni(rng), uni(rng)};
    const float expected = a + b * w[0] + c * w[1] + d * w[2];
    EXPECT_NEAR(trilinear(w, v), expected, 1e-5f)
        << "w=(" << w[0] << "," << w[1] << "," << w[2] << ")";
  }
}

TEST(Trilinear, MatchesHandComputedMidpointAverages) {
  const float v[8] = {5.f, -3.f, 7.f, 2.f, 11.f, 4.f, -8.f, 9.f};

  const float wc[3] = {0.5f, 0.5f, 0.5f};
  float sum = 0.f;
  for (int i = 0; i < 8; ++i)
    sum += v[i];
  EXPECT_NEAR(trilinear(wc, v), sum / 8.f, 1e-6f);

  const float wx[3] = {0.5f, 0.f, 0.f};
  EXPECT_NEAR(trilinear(wx, v), 0.5f * (v[0] + v[1]), 1e-6f);
  const float wy[3] = {0.f, 0.5f, 0.f};
  EXPECT_NEAR(trilinear(wy, v), 0.5f * (v[0] + v[2]), 1e-6f);
  const float wz[3] = {0.f, 0.f, 0.5f};
  EXPECT_NEAR(trilinear(wz, v), 0.5f * (v[0] + v[4]), 1e-6f);
  // A face midpoint (x=1 face).
  const float wf[3] = {1.f, 0.5f, 0.5f};
  EXPECT_NEAR(trilinear(wf, v), 0.25f * (v[1] + v[3] + v[5] + v[7]), 1e-6f);
}

// ============================================================================
// spline_gradient_cache
// ============================================================================

TEST(SplineGradient, RecoversConstantGradientOfLinearField) {
  // f = 3 + 2x - 5y + 0.5z in voxel-index units (span = 1, minb = 0).
  const std::int64_t dim = 12;
  std::vector<float> buf(std::size_t(dim * dim * dim));
  for (std::int64_t k = 0; k < dim; ++k)
    for (std::int64_t j = 0; j < dim; ++j)
      for (std::int64_t i = 0; i < dim; ++i)
        buf[std::size_t(i + dim * j + dim * dim * k)] =
            3.f + 2.f * float(i) - 5.f * float(j) + 0.5f * float(k);
  const grid_sampler g = make_view(buf.data(), cvc::Float, dim);

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uni(0.f, 1.f);
  const std::int64_t cells[4][3] = {{2, 2, 2}, {5, 4, 6}, {8, 8, 8}, {3, 7, 5}};
  const float fixed_w[3][3] = {{0.f, 0.f, 0.f}, {1.f, 1.f, 1.f}, {0.5f, 0.5f, 0.5f}};

  spline_gradient_cache cache;
  for (const auto &cell : cells) {
    for (const auto &w : fixed_w) {
      const vec3d grad = cache.evaluate(g, cell, w);
      EXPECT_NEAR(grad.x, 2.0, 2e-4);
      EXPECT_NEAR(grad.y, -5.0, 5e-4);
      EXPECT_NEAR(grad.z, 0.5, 1e-4);
    }
    for (int trial = 0; trial < 4; ++trial) {
      const float w[3] = {uni(rng), uni(rng), uni(rng)};
      const vec3d grad = cache.evaluate(g, cell, w);
      EXPECT_NEAR(grad.x, 2.0, 2e-4);
      EXPECT_NEAR(grad.y, -5.0, 5e-4);
      EXPECT_NEAR(grad.z, 0.5, 1e-4);
    }
  }
}

TEST(SplineGradient, CacheRevisitsCellWithResultsIdenticalToFreshEvaluation) {
  // Non-trivial field so different cells really produce different gradients.
  const std::int64_t dim = 12;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uni(0.f, 100.f);
  std::vector<float> buf(std::size_t(dim * dim * dim));
  for (auto &v : buf)
    v = uni(rng);
  const grid_sampler g = make_view(buf.data(), cvc::Float, dim);

  const std::int64_t cell_a[3] = {2, 3, 4};
  const std::int64_t cell_b[3] = {5, 5, 5};
  const float wa[3] = {0.3f, 0.7f, 0.2f};
  const float wb[3] = {0.6f, 0.1f, 0.9f};

  // Reference values from single-purpose caches.
  spline_gradient_cache fresh_a, fresh_b;
  const vec3d ref_a = fresh_a.evaluate(g, cell_a, wa);
  const vec3d ref_b = fresh_b.evaluate(g, cell_b, wb);

  // One cache driven A -> B -> A must refresh correctly on every switch.
  spline_gradient_cache cache;
  const vec3d first_a = cache.evaluate(g, cell_a, wa);
  EXPECT_EQ(first_a.x, ref_a.x);
  EXPECT_EQ(first_a.y, ref_a.y);
  EXPECT_EQ(first_a.z, ref_a.z);

  const vec3d mid_b = cache.evaluate(g, cell_b, wb);
  EXPECT_EQ(mid_b.x, ref_b.x);
  EXPECT_EQ(mid_b.y, ref_b.y);
  EXPECT_EQ(mid_b.z, ref_b.z);

  const vec3d back_a = cache.evaluate(g, cell_a, wa);
  EXPECT_EQ(back_a.x, ref_a.x);
  EXPECT_EQ(back_a.y, ref_a.y);
  EXPECT_EQ(back_a.z, ref_a.z);

  // The two cells genuinely differ (guards against a vacuous pass).
  EXPECT_TRUE(ref_a.x != ref_b.x || ref_a.y != ref_b.y || ref_a.z != ref_b.z);
}

// ============================================================================
// mc tables
// ============================================================================

TEST(McTables, VertexFromBinaryIsPermutationOfZeroToSeven) {
  bool seen[8] = {};
  for (int v = 0; v < 8; ++v) {
    ASSERT_GE(mc_vertex_from_binary[v], 0);
    ASSERT_LT(mc_vertex_from_binary[v], 8);
    EXPECT_FALSE(seen[mc_vertex_from_binary[v]])
        << "duplicate binary index " << mc_vertex_from_binary[v];
    seen[mc_vertex_from_binary[v]] = true;
  }
  for (int b = 0; b < 8; ++b)
    EXPECT_TRUE(seen[b]) << "binary index " << b << " never mapped";
}

TEST(McTables, VertexRemapMatchesVtkCornerCoordinates) {
  // mc_vertex_from_binary[v] must be the raster/binary index (x + 2y + 4z) of
  // MC vertex v's VTK coordinates.
  for (int v = 0; v < 8; ++v) {
    const int binary =
        kVtkVertexCoord[v][0] + 2 * kVtkVertexCoord[v][1] + 4 * kVtkVertexCoord[v][2];
    EXPECT_EQ(mc_vertex_from_binary[v], binary) << "MC vertex " << v;
  }
}

TEST(McTables, TriCaseEdgesAppearInCubeEdgesAndTripleUp) {
  for (int code = 0; code < 256; ++code) {
    const int nedges = cube_edges[code][0];
    ASSERT_LE(nedges, 12) << "code " << code;
    bool present[12] = {};
    for (int e = 0; e < nedges; ++e) {
      const int edge = cube_edges[code][1 + e];
      ASSERT_LT(edge, 12) << "code " << code;
      present[edge] = true;
    }

    int nverts = 0;
    while (nverts < 16 && tri_cases[code][nverts] != -1)
      ++nverts;
    ASSERT_LT(nverts, 16) << "code " << code << " missing -1 terminator";
    EXPECT_EQ(nverts % 3, 0) << "code " << code;

    for (int t = 0; t < nverts; ++t) {
      const int edge = tri_cases[code][t];
      ASSERT_GE(edge, 0) << "code " << code;
      ASSERT_LT(edge, 12) << "code " << code;
      EXPECT_TRUE(present[edge]) << "code " << code << ": tri edge " << edge
                                 << " not in cube_edges";
    }
  }
}

TEST(McTables, EdgeInfoEndpointsDifferOnlyAlongDirAndMatchStartOffset) {
  for (int e = 0; e < 12; ++e) {
    const auto &ei = mc_edges[e];
    ASSERT_GE(ei.dir, 0);
    ASSERT_LE(ei.dir, 2);
    const int *c1 = kVtkVertexCoord[ei.v1];
    const int *c2 = kVtkVertexCoord[ei.v2];
    for (int axis = 0; axis < 3; ++axis) {
      if (axis == ei.dir) {
        // The edge runs from 0 to 1 along its own axis.
        EXPECT_EQ(c1[axis], 0) << "edge " << e;
        EXPECT_EQ(c2[axis], 1) << "edge " << e;
      } else {
        EXPECT_EQ(c1[axis], c2[axis]) << "edge " << e << " axis " << axis;
      }
    }
    // (di,dj,dk) is the start corner's cell offset.
    EXPECT_EQ(ei.di, c1[0]) << "edge " << e;
    EXPECT_EQ(ei.dj, c1[1]) << "edge " << e;
    EXPECT_EQ(ei.dk, c1[2]) << "edge " << e;
  }
}

// ============================================================================
// extract_contour
// ============================================================================

TEST(ExtractContour, NoTrianglesWhenCellEntirelyBelowOrAboveIsovalue) {
  mc_cell cell;
  cell.span = {1, 1, 1};
  mc_triangle tris[5];

  for (int v = 0; v < 8; ++v)
    cell.func[v] = 1.f; // all below iso = 5
  EXPECT_EQ(extract_contour(5.f, cell, tris), 0);

  for (int v = 0; v < 8; ++v)
    cell.func[v] = 9.f; // all above iso = 5
  EXPECT_EQ(extract_contour(5.f, cell, tris), 0);
}

TEST(ExtractContour, LinearFieldAlongXYieldsPlanarQuadAtCrossing) {
  const float f0 = 10.f, gain = 4.f, iso = 11.f;
  const mc_cell cell = make_linear_x_cell(f0, gain);
  const double xc = double((iso - f0) / gain); // local crossing = 0.25
  const double plane_x = cell.orig.x + cell.span.x * (double(cell.id[0]) + xc);

  const double ylo = cell.orig.y + cell.span.y * double(cell.id[1]);
  const double yhi = ylo + cell.span.y;
  const double zlo = cell.orig.z + cell.span.z * double(cell.id[2]);
  const double zhi = zlo + cell.span.z;

  mc_triangle tris[5];
  const int nt = extract_contour(iso, cell, tris);
  ASSERT_EQ(nt, 2);

  for (int t = 0; t < nt; ++t) {
    for (int v = 0; v < 3; ++v) {
      const vec3d &p = tris[t].vert[v];
      EXPECT_NEAR(p.x, plane_x, 1e-5) << "tri " << t << " vert " << v;
      EXPECT_GE(p.y, ylo - 1e-9);
      EXPECT_LE(p.y, yhi + 1e-9);
      EXPECT_GE(p.z, zlo - 1e-9);
      EXPECT_LE(p.z, zhi + 1e-9);
    }
  }
}

// ============================================================================
// intersect_isosurface_in_cell
// ============================================================================

TEST(IsosurfaceIntersect, AxisRayThroughCellHitsLinearFieldCrossing) {
  const float f0 = 10.f, gain = 4.f, iso = 11.f;
  const mc_cell cell = make_linear_x_cell(f0, gain);
  const double xc = double((iso - f0) / gain); // 0.25
  const double plane_x = cell.orig.x + cell.span.x * (double(cell.id[0]) + xc);

  // +x ray through the middle of the cell's y/z extent, unit direction.
  const double ylo = cell.orig.y + cell.span.y * double(cell.id[1]);
  const double zlo = cell.orig.z + cell.span.z * double(cell.id[2]);
  ray r;
  r.origin = {-0.4, ylo + 0.5 * cell.span.y, zlo + 0.5 * cell.span.z};
  r.direction = {1.0, 0.0, 0.0};

  float w[3] = {-1.f, -1.f, -1.f};
  double t_hit = -1.0;
  ASSERT_TRUE(intersect_isosurface_in_cell(r, iso, cell, w, t_hit));
  EXPECT_NEAR(w[0], float(xc), 1e-4f);
  EXPECT_NEAR(w[1], 0.5f, 1e-4f);
  EXPECT_NEAR(w[2], 0.5f, 1e-4f);
  // Unit direction: t is the world distance from the origin to the plane.
  EXPECT_NEAR(t_hit, plane_x - r.origin.x, 1e-5);
}

TEST(IsosurfaceIntersect, RayPointingAwayFromCrossingMisses) {
  const mc_cell cell = make_linear_x_cell(10.f, 4.f);
  // Origin past the crossing plane (x = -0.1875) heading further +x: the
  // isosurface is behind the origin, so t < 0 and the driver must miss.
  ray r;
  r.origin = {0.2, 0.0, -0.0625};
  r.direction = {1.0, 0.0, 0.0};
  float w[3];
  double t_hit = 0.0;
  EXPECT_FALSE(intersect_isosurface_in_cell(r, 11.f, cell, w, t_hit));
}

TEST(IsosurfaceIntersect, RayOutsideCellCrossSectionMissesLaterally) {
  const mc_cell cell = make_linear_x_cell(10.f, 4.f);
  // Parallel to +x but at y far above the cell (cell y range [-0.1, 0.1]):
  // the ray crosses the isosurface plane outside the cell's triangles.
  ray r;
  r.origin = {-0.4, 0.5, -0.0625};
  r.direction = {1.0, 0.0, 0.0};
  float w[3];
  double t_hit = 0.0;
  EXPECT_FALSE(intersect_isosurface_in_cell(r, 11.f, cell, w, t_hit));

  // Same laterally below in z (cell z range [-0.125, 0]).
  r.origin = {-0.4, 0.0, -0.4};
  EXPECT_FALSE(intersect_isosurface_in_cell(r, 11.f, cell, w, t_hit));
}

TEST(IsosurfaceIntersect, DegenerateFlatInputsNeitherCrashNorYieldNan) {
  // All corners exactly AT the isovalue: strict `<` classification gives
  // code 0, so there is nothing to hit -- must return false cleanly.
  mc_cell cell = make_linear_x_cell(10.f, 4.f);
  for (int v = 0; v < 8; ++v)
    cell.func[v] = 5.f;
  ray r;
  r.origin = {-0.4, 0.0, -0.0625};
  r.direction = {1.0, 0.0, 0.0};
  float w[3];
  double t_hit = 0.0;
  EXPECT_FALSE(intersect_isosurface_in_cell(r, 5.f, cell, w, t_hit));

  // Zero-area (collinear) triangle: plane normal is the zero vector, so the
  // intersection must report a miss instead of dividing by zero.
  mc_triangle flat;
  flat.vert[0] = {0, 0, 0};
  flat.vert[1] = {1, 1, 1};
  flat.vert[2] = {2, 2, 2};
  vec3d point;
  const double t = intersect_triangle(r, flat, point);
  EXPECT_FALSE(std::isnan(t));
  EXPECT_LT(t, 0.0);

  // Fully collapsed triangle: the projection inside-test must reject it.
  mc_triangle collapsed;
  collapsed.vert[0] = collapsed.vert[1] = collapsed.vert[2] = vec3d{1, 2, 3};
  EXPECT_FALSE(in_triangle(vec3d{1, 2, 3}, collapsed, vec3d{}));
}

// ============================================================================
// Volumetric shadows: the light-view camera fit and the map lookup
// (detail/shadow_map.h)
// ============================================================================

namespace {

cvc::bounding_box boxOf(double x0, double y0, double z0, double x1, double y1, double z1) {
  return cvc::bounding_box(x0, y0, z0, x1, y1, z1);
}

cvc::volren::light lightToward(double x, double y, double z) {
  cvc::volren::light l;
  l.direction = {x, y, z};
  return l;
}

} // namespace

TEST(ShadowMap, FitLightCameraCoversBounds) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::detail::fit_light_camera;

  // Both branches of the up-vector choice, straddling the 0.9 threshold, plus
  // an off-center box so a missing re-centering would show up.
  const std::array<std::array<double, 3>, 8> dirs = {{
      {0, 0, 1},          // straight up: the +Y fallback branch
      {0, 0, -1},         // straight down: the +Y fallback branch
      {0.89, 0.0, 0.456}, // |z| just under 0.9: the +Z branch
      {0.1, 0.0, 0.995},  // |z| just over 0.9: the +Y branch
      {1, 0, 0},
      {0, 1, 0},
      {0.3, -0.6, 0.7},
      {-0.5, 0.5, -0.7},
  }};
  const cvc::bounding_box b = boxOf(-2.0, 1.0, -0.5, 3.0, 4.0, 6.0);

  for (const std::array<double, 3> &d : dirs) {
    camera cam;
    shadow_view v;
    ASSERT_TRUE(fit_light_camera(lightToward(d[0], d[1], d[2]), b, 256, cam, v))
        << "no fit for direction " << d[0] << "," << d[1] << "," << d[2];
    ASSERT_NO_THROW(cam.basis()) << "the fitted camera is degenerate";
    EXPECT_EQ(v.width, 256);
    EXPECT_EQ(v.height, 256);
    EXPECT_NEAR(v.texel_world, 2.0 * v.parallel_scale / 256.0, 1e-15);

    // Every corner of the box must project INSIDE the map and sit in front of
    // the light's eye plane -- that is what "the map covers the scene" means.
    for (int corner = 0; corner < 8; ++corner) {
      const std::array<double, 3> p = {corner & 1 ? b.maxx : b.minx, corner & 2 ? b.maxy : b.miny,
                                       corner & 4 ? b.maxz : b.minz};
      int ix = -1, iy = -1;
      double depth = 0.0;
      EXPECT_TRUE(v.project(p, ix, iy, depth)) << "a scene corner falls outside the shadow map";
      EXPECT_GE(ix, 0);
      EXPECT_LT(ix, 256);
      EXPECT_GE(iy, 0);
      EXPECT_LT(iy, 256);
      EXPECT_GT(depth, 0.0) << "a scene corner is behind the light's eye plane";
    }

    // The frame is orthonormal and right-handed, and `forward` points AWAY
    // from the light (a sign flip here inverts every shadow).
    const cvc::volren::vec3d right(v.right), up(v.up), fwd(v.forward);
    EXPECT_NEAR(dot(right, right), 1.0, 1e-12);
    EXPECT_NEAR(dot(up, up), 1.0, 1e-12);
    EXPECT_NEAR(dot(fwd, fwd), 1.0, 1e-12);
    EXPECT_NEAR(dot(right, up), 0.0, 1e-12);
    EXPECT_NEAR(dot(right, fwd), 0.0, 1e-12);
    EXPECT_NEAR(dot(up, fwd), 0.0, 1e-12);
    const cvc::volren::vec3d toward_light = normalized(cvc::volren::vec3d(d[0], d[1], d[2]));
    EXPECT_NEAR(dot(fwd, toward_light), -1.0, 1e-12);
  }

  // Degenerate inputs cast nothing rather than throwing or producing a
  // garbage frame.
  camera cam;
  cvc::volren::shadow_view v;
  EXPECT_FALSE(fit_light_camera(lightToward(0, 0, 0), b, 256, cam, v));
  EXPECT_FALSE(fit_light_camera(lightToward(std::numeric_limits<double>::quiet_NaN(), 0, 1), b, 256,
                                cam, v));
  EXPECT_FALSE(fit_light_camera(lightToward(0, 0, 1), boxOf(1, 1, 1, 1, 1, 1), 256, cam, v));

  // Resolution clamps into [min_shadow_resolution, max_raster_dim].
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), b, 1, cam, v));
  EXPECT_EQ(v.width, cvc::volren::limits::min_shadow_resolution);
}

TEST(ShadowMap, ProjectRoundTrips) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::vec3d;
  using cvc::volren::detail::fit_light_camera;

  camera cam;
  shadow_view v;
  ASSERT_TRUE(
      fit_light_camera(lightToward(0.3, -0.6, 0.7), boxOf(-1, -1, -1, 1, 1, 1), 64, cam, v));

  // The world point at texel (ix, iy)'s CENTER must project back to exactly
  // that texel.  project() inverts ray_generator::at, so this is the check
  // that the two agree -- a half-texel slip or a v-flip fails it.
  const vec3d eye(v.eye), right(v.right), up(v.up), fwd(v.forward);
  for (const int ix : {0, 1, 17, 32, 62, 63})
    for (const int iy : {0, 1, 17, 32, 62, 63}) {
      const double u = (double(ix) + 0.5) / 64.0 * 2.0 - 1.0;
      const double vv = 1.0 - (double(iy) + 0.5) / 64.0 * 2.0;
      const vec3d p = eye + right * (u * v.parallel_scale) + up * (vv * v.parallel_scale) +
                      fwd * 2.5; // any depth: the texel is depth-independent
      int bx = -1, by = -1;
      double depth = 0.0;
      ASSERT_TRUE(v.project(p.to_array(), bx, by, depth));
      EXPECT_EQ(bx, ix);
      EXPECT_EQ(by, iy);
      EXPECT_NEAR(depth, 2.5, 1e-12);
    }

  // One texel outside the map is a miss, not a wrapped index.
  const vec3d outside = eye + right * (v.parallel_scale + 2.0 * v.texel_world) + fwd * 1.0;
  int ix = 0, iy = 0;
  double depth = 0.0;
  EXPECT_FALSE(v.project(outside.to_array(), ix, iy, depth));

  // NaN is rejected in the double domain, before any cast could be undefined.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(v.project({nan, 0.0, 0.0}, ix, iy, depth));
  EXPECT_FALSE(v.project({0.0, nan, 0.0}, ix, iy, depth));
  EXPECT_FALSE(v.project({std::numeric_limits<double>::infinity(), 0.0, 0.0}, ix, iy, depth));
}

TEST(ShadowMap, BiasFormulaAndVisibility) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::vec3d;
  using cvc::volren::detail::fit_light_camera;
  using cvc::volren::detail::shadow_bias;
  using cvc::volren::detail::shadow_visibility;

  camera cam;
  shadow_view v;
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), boxOf(-1, -1, -1, 1, 1, 1), 128, cam, v));

  // At normal incidence the slope term vanishes and the bias is exactly the
  // latch quantum times its scale.
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 1.0, 1.0), 0.25);
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 2.0, 1.0, -1.0), 0.5); // |n.l|, so the sign is irrelevant
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 0.0, 0.1), 0.25); // slope_scale 0 kills the slope term

  // Monotone in the angle, and saturating at the cos floor rather than
  // diverging: a flat-gradient sample must get a large-but-finite bias (i.e.
  // read as LIT), never an infinity or a NaN.
  double previous = shadow_bias(v, 0.25, 1.0, 1.0, 1.0);
  for (const double ndotl : {0.9, 0.7, 0.5, 0.3, 0.15, 0.1, 0.05, 0.0}) {
    const double b = shadow_bias(v, 0.25, 1.0, 1.0, ndotl);
    EXPECT_GE(b, previous);
    EXPECT_TRUE(std::isfinite(b));
    previous = b;
  }
  const double floored = shadow_bias(v, 0.25, 1.0, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 1.0, 0.0), floored);
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 1.0, std::numeric_limits<double>::quiet_NaN()),
                   floored);

  // The lookup itself, against a map written by hand.
  std::vector<float> depth(std::size_t(v.width) * v.height, std::numeric_limits<float>::infinity());
  const vec3d eye(v.eye), fwd(v.forward);
  const vec3d probe = eye + fwd * 3.0; // dead center of the map, depth 3
  int ix = 0, iy = 0;
  double d = 0.0;
  ASSERT_TRUE(v.project(probe.to_array(), ix, iy, d));
  ASSERT_NEAR(d, 3.0, 1e-12);

  // A texel the light ray missed is +inf: never shadows, at any bias.
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), probe, 0.0, 1.0f), 1.f);
  // An occluder in front of the probe shadows it, scaled by strength.
  depth[std::size_t(iy) * v.width + ix] = 2.0f;
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), probe, 0.0, 1.0f), 0.f);
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), probe, 0.0, 0.25f), 0.75f);
  // ... unless the bias swallows the gap.
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), probe, 1.5, 1.0f), 1.f);
  // An occluder BEHIND the probe never shadows it.
  depth[std::size_t(iy) * v.width + ix] = 4.0f;
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), probe, 0.0, 1.0f), 1.f);
  // A null map and a point off the map both fail LIT.
  EXPECT_FLOAT_EQ(shadow_visibility(v, nullptr, probe, 0.0, 1.0f), 1.f);
  const vec3d off = eye + vec3d(v.right) * (v.parallel_scale * 3.0) + fwd * 3.0;
  EXPECT_FLOAT_EQ(shadow_visibility(v, depth.data(), off, 0.0, 1.0f), 1.f);
}

// ---------------------------------------------------------------------------
// Deep shadow maps (shadow_mode::deep): the knot grid and the two-channel
// lookup (detail::shadow_visibility_deep)
// ---------------------------------------------------------------------------

TEST(ShadowMap, DeepKnotGridBracketsTheSceneExactly) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::detail::fit_light_camera;

  const cvc::bounding_box b = boxOf(-2.0, 1.0, -0.5, 3.0, 4.0, 6.0);
  const std::array<std::array<double, 3>, 4> dirs = {
      {{0, 0, 1}, {1, 0, 0}, {0.3, -0.6, 0.7}, {-0.5, 0.5, -0.7}}};

  for (const std::array<double, 3> &d : dirs) {
    camera cam;
    shadow_view v;
    ASSERT_TRUE(fit_light_camera(lightToward(d[0], d[1], d[2]), b, 128, cam, v, 8));
    EXPECT_EQ(v.slices, 8);
    EXPECT_GT(v.slice_dz, 0.0);

    // The grid must BRACKET every scene corner, and tightly: a knot spent
    // outside the marched region is a knot wasted, and a corner outside the
    // grid is a receiver whose profile is extrapolated.
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (int corner = 0; corner < 8; ++corner) {
      const std::array<double, 3> p = {corner & 1 ? b.maxx : b.minx, corner & 2 ? b.maxy : b.miny,
                                       corner & 4 ? b.maxz : b.minz};
      int ix = 0, iy = 0;
      double depth = 0.0;
      ASSERT_TRUE(v.project(p, ix, iy, depth));
      lo = std::min(lo, depth);
      hi = std::max(hi, depth);
    }
    EXPECT_NEAR(v.depth_min, lo, 1e-12) << "the knot grid does not start at the scene";
    EXPECT_NEAR(v.depth_max(), hi, 1e-12) << "the knot grid does not end at the scene";
  }

  // slices == 0 leaves the view a HARD map -- that is the whole switch.
  camera cam;
  shadow_view v;
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), b, 128, cam, v));
  EXPECT_EQ(v.slices, 0);
  EXPECT_EQ(v.slice_dz, 0.0);
}

TEST(ShadowMap, DeepVisibilityChannelsAndInterpolation) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::vec3d;
  using cvc::volren::detail::fit_light_camera;
  using cvc::volren::detail::shadow_visibility;
  using cvc::volren::detail::shadow_visibility_deep;

  // A light straight down +z over a 2-unit cube: depth_min .. depth_max spans
  // exactly the box, and 4 slices put knots every half unit.
  camera cam;
  shadow_view v;
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), boxOf(-1, -1, -1, 1, 1, 1), 128, cam, v, 4));
  ASSERT_EQ(v.slices, 4);
  ASSERT_NEAR(v.slice_dz, 0.5, 1e-12);

  const std::size_t plane = std::size_t(v.width) * std::size_t(v.height);
  std::vector<float> prof(plane * std::size_t(v.slices + 1), 0.f);
  for (std::size_t i = 0; i < plane; ++i)
    prof[i] = std::numeric_limits<float>::infinity();

  const vec3d eye(v.eye), fwd(v.forward);
  const auto at = [&](double depth) { return eye + fwd * depth; };
  int ix = 0, iy = 0;
  double d0 = 0.0;
  ASSERT_TRUE(v.project(at(v.depth_min).to_array(), ix, iy, d0));
  const std::size_t texel = std::size_t(iy) * std::size_t(v.width) + std::size_t(ix);
  const auto knot = [&](int j) -> float & { return prof[std::size_t(j) * plane + texel]; };

  // An empty profile never shadows, whatever the depth.
  for (const double f : {0.0, 0.25, 0.5, 0.9, 1.0})
    EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), at(v.depth_min + f * 2.0), 0.0, 1.0f),
                    1.f);

  // ---- the PROFILE channel: piecewise linear in accumulated alpha ---------
  // Knots 1..4 sit at depth_min + {0.5, 1.0, 1.5, 2.0}; knot 0 is implicitly 0.
  knot(1) = 0.0f;
  knot(2) = 0.5f;
  knot(3) = 0.5f;
  knot(4) = 0.75f;
  const struct {
    double depth;
    float alpha;
  } cases[] = {
      {0.00, 0.00f},  // at knot 0
      {0.25, 0.00f},  // halfway to knot 1, both 0
      {0.50, 0.00f},  // knot 1
      {0.75, 0.25f},  // halfway from 0 to 0.5
      {1.00, 0.50f},  // knot 2
      {1.25, 0.50f},  // flat stretch
      {1.50, 0.50f},  // knot 3
      {1.75, 0.625f}, // halfway from 0.5 to 0.75
      {2.00, 0.75f},  // knot 4, the last one
      {2.50, 0.75f},  // past the grid: held, never extrapolated
  };
  for (const auto &c : cases) {
    const vec3d p = at(v.depth_min + c.depth);
    EXPECT_NEAR(shadow_visibility_deep(v, prof.data(), p, 0.0, 1.0f), 1.f - c.alpha, 1e-6)
        << "depth " << c.depth;
    // strength scales the ATTENUATION, exactly as it does in the hard path.
    EXPECT_NEAR(shadow_visibility_deep(v, prof.data(), p, 0.0, 0.5f), 1.f - 0.5f * c.alpha, 1e-6)
        << "depth " << c.depth << " at half strength";
  }

  // The bias shifts the QUERY toward the light: at depth 1.0 with a 0.5 bias
  // the lookup reads knot 1, not knot 2.
  EXPECT_NEAR(shadow_visibility_deep(v, prof.data(), at(v.depth_min + 1.0), 0.5, 1.0f), 1.f, 1e-6);

  // ---- the TERMINAL channel: exact, and it OVERRIDES the slices -----------
  // Set the terminal in the middle of the flat 0.5 stretch.  In front of it the
  // profile still answers; behind it the answer is the hard one.
  prof[texel] = float(v.depth_min + 1.25);
  EXPECT_NEAR(shadow_visibility_deep(v, prof.data(), at(v.depth_min + 1.0), 0.0, 1.0f), 0.5f, 1e-6);
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), at(v.depth_min + 1.5), 0.0, 1.0f), 0.f);
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), at(v.depth_min + 1.5), 0.0, 0.25f), 0.75f);
  // ... and the bias swallows the gap the same way it does for a hard map.
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), at(v.depth_min + 1.5), 1.0, 1.0f), 1.f);

  // ---- the identity that makes deep a superset of hard --------------------
  // With an all-or-nothing profile the deep lookup and the hard lookup are the
  // SAME function of (depth, terminal, bias, strength) -- which is why an
  // opaque occluder renders byte-identically in either mode.
  std::vector<float> depth_only(plane, std::numeric_limits<float>::infinity());
  for (int j = 1; j <= v.slices; ++j)
    knot(j) = 0.f;
  for (const double term : {0.4, 1.1, 1.9, 3.0}) {
    prof[texel] = float(v.depth_min + term);
    depth_only[texel] = prof[texel];
    for (const double probe : {0.1, 0.5, 1.0, 1.5, 2.0})
      for (const double bias : {0.0, 0.3})
        for (const float strength : {1.0f, 0.4f}) {
          const vec3d p = at(v.depth_min + probe);
          EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), p, bias, strength),
                          shadow_visibility(v, depth_only.data(), p, bias, strength))
              << "term " << term << " probe " << probe << " bias " << bias;
        }
  }

  // ---- degenerate inputs fail LIT, the non-destructive direction ----------
  const vec3d probe = at(v.depth_min + 1.0);
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, nullptr, probe, 0.0, 1.0f), 1.f);
  shadow_view hard = v;
  hard.slices = 0;
  EXPECT_FLOAT_EQ(shadow_visibility_deep(hard, prof.data(), probe, 0.0, 1.0f), 1.f);
  const vec3d off = eye + vec3d(v.right) * (v.parallel_scale * 3.0) + fwd * 1.0;
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), off, 0.0, 1.0f), 1.f);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), vec3d(0.0, 0.0, nan), 0.0, 1.0f), 1.f);
}

// ---------------------------------------------------------------------------
// Percentage-closer filtering (shadow_settings::pcf_radius / pcf_taps)
// ---------------------------------------------------------------------------

TEST(ShadowMap, PcfGridIsInertUnlessBothKnobsAskForIt) {
  using cvc::volren::detail::pcf_half_width;
  using cvc::volren::detail::pcf_offset;

  // Radius 0, negative and NaN are all inert -- the callers branch on a 0 half
  // width to reach the historical single-tap expression, so this IS the
  // byte-identity guarantee at the kernel level.
  EXPECT_EQ(pcf_half_width(0.f, 7), 0);
  EXPECT_EQ(pcf_half_width(-1.f, 7), 0);
  EXPECT_EQ(pcf_half_width(std::numeric_limits<float>::quiet_NaN(), 7), 0);
  // So is a single tap, from the other knob.
  EXPECT_EQ(pcf_half_width(4.f, 1), 0);
  EXPECT_EQ(pcf_half_width(4.f, 0), 0); // clamps up to min_pcf_taps == 1
  EXPECT_EQ(pcf_half_width(4.f, -9), 0);

  EXPECT_EQ(pcf_half_width(4.f, 3), 1);
  EXPECT_EQ(pcf_half_width(4.f, 5), 2);
  EXPECT_EQ(pcf_half_width(4.f, 7), 3);
  // Even values round DOWN to the odd grid below (a grid with no center tap
  // would displace the shadow by half a tap spacing).
  EXPECT_EQ(pcf_half_width(4.f, 4), 1);
  EXPECT_EQ(pcf_half_width(4.f, 6), 2);
  // And the ceiling is a hard clamp, because the tap count is its square.
  EXPECT_EQ(pcf_half_width(4.f, 999), (cvc::volren::limits::max_pcf_taps - 1) / 2);

  // The tap grid spans exactly [-radius, +radius], centered, rounded to texels.
  EXPECT_EQ(pcf_offset(-1, 1, 4.0), -4);
  EXPECT_EQ(pcf_offset(0, 1, 4.0), 0);
  EXPECT_EQ(pcf_offset(1, 1, 4.0), 4);
  EXPECT_EQ(pcf_offset(-2, 2, 5.0), -5);
  EXPECT_EQ(pcf_offset(-1, 2, 5.0), -3); // 2.5 rounds away from zero, both ways
  EXPECT_EQ(pcf_offset(1, 2, 5.0), 3);
  // A radius narrower than the grid collapses taps onto one texel: a filter
  // thinner than a texel cannot blur, and does not pretend to.
  for (int i = -1; i <= 1; ++i)
    EXPECT_EQ(pcf_offset(i, 1, 0.25), 0);

  // SYMMETRY is the property, not the rounding rule that delivers it: the
  // filter is a box average CENTERED on the receiver's own texel, so the tap
  // grid must be its own mirror image for every radius -- including the
  // half-integer tap positions where round-half-up would put the footprint's
  // centroid off the sample and slide the whole penumbra by a texel.
  for (int k = 1; k <= (cvc::volren::limits::max_pcf_taps - 1) / 2; ++k)
    for (double radius = 0.125; radius <= 16.0; radius += 0.125)
      for (int i = 1; i <= k; ++i)
        ASSERT_EQ(pcf_offset(-i, k, radius), -pcf_offset(i, k, radius))
            << "asymmetric tap grid at k=" << k << " radius=" << radius << " i=" << i;
  // The grid still spans the full radius and stays monotone in i, so no tap
  // steps backwards and the outermost pair is exactly +/- the radius.
  for (int k = 1; k <= 3; ++k)
    for (double radius = 0.125; radius <= 16.0; radius += 0.125) {
      EXPECT_EQ(pcf_offset(k, k, radius), int(std::floor(radius + 0.5)));
      for (int i = 1; i <= k; ++i)
        EXPECT_GE(pcf_offset(i, k, radius), pcf_offset(i - 1, k, radius));
    }
}

TEST(ShadowMap, PcfAveragesTheNeighbourhoodOnBothPayloads) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::vec3d;
  using cvc::volren::detail::fit_light_camera;
  using cvc::volren::detail::shadow_visibility;
  using cvc::volren::detail::shadow_visibility_deep;

  camera cam;
  shadow_view v;
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), boxOf(-1, -1, -1, 1, 1, 1), 64, cam, v, 4));

  const vec3d eye(v.eye), fwd(v.forward);
  const vec3d probe = eye + fwd * 3.0; // dead center of the map
  int ix = 0, iy = 0;
  double depth = 0.0;
  ASSERT_TRUE(v.project(probe.to_array(), ix, iy, depth));

  const std::size_t plane = std::size_t(v.width) * std::size_t(v.height);
  std::vector<float> hard(plane, std::numeric_limits<float>::infinity());
  // Occlude the LEFT half of the 3x3 neighbourhood: three of nine taps block.
  for (int j = -1; j <= 1; ++j)
    hard[std::size_t(iy + j) * v.width + (ix - 1)] = 2.0f;

  // Unfiltered: the CENTER texel is unoccluded, so the receiver is fully lit --
  // and this is the value the filtered lookups must be an average AROUND.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 0.0, 1.0f), 1.f);
  // 3x3 over a 1-texel radius: 3 of 9 taps blocked.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 0.0, 1.0f, 1.f, 3), 6.f / 9.f);
  // strength scales each tap identically, so it factors straight out.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 0.0, 0.5f, 1.f, 3),
                  (6.f + 3.f * 0.5f) / 9.f);
  // A radius of 2 with 3 taps samples columns ix-2, ix, ix+2 -- which MISSES
  // the occluded column entirely.  Radius and tap count are independent knobs
  // and this is the sharpest statement of it.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 0.0, 1.0f, 2.f, 3), 1.f);
  // 5 taps over radius 2 hit columns ix-2..ix+2, so the occluded column is back
  // -- 3 of its 5 taps, since the occluder is only three texels tall.  A wider
  // footprint therefore DILUTES a small occluder, which is what a penumbra is.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 0.0, 1.0f, 2.f, 5), 22.f / 25.f);
  // The bias still applies per tap, so a bias that swallows the gap lights the
  // whole footprint.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), probe, 1.5, 1.0f, 1.f, 3), 1.f);

  // ---- the same, on a deep payload ---------------------------------------
  std::vector<float> prof(plane * std::size_t(v.slices + 1), 0.f);
  for (std::size_t i = 0; i < plane; ++i)
    prof[i] = std::numeric_limits<float>::infinity();
  // The left column accumulates alpha 0.5 by the first knot and holds it.
  for (int j = -1; j <= 1; ++j)
    for (int k = 1; k <= v.slices; ++k)
      prof[std::size_t(k) * plane + std::size_t(iy + j) * v.width + (ix - 1)] = 0.5f;
  const vec3d deep_probe = eye + fwd * v.depth_max();
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), deep_probe, 0.0, 1.0f), 1.f);
  // Three taps at 1 - 0.5 and six at 1: the average of the RECONSTRUCTED
  // visibilities, which is what a partially occluded footprint means.
  EXPECT_FLOAT_EQ(shadow_visibility_deep(v, prof.data(), deep_probe, 0.0, 1.0f, 1.f, 3),
                  (6.f + 3.f * 0.5f) / 9.f);

  // A filter that reaches off the map counts those taps LIT, the same
  // non-destructive direction a lookup off the map takes.
  const vec3d corner = eye + vec3d(v.right) * (v.parallel_scale * 0.999) +
                       vec3d(v.up) * (v.parallel_scale * 0.999) + fwd * 3.0;
  int cx = 0, cy = 0;
  double cd = 0.0;
  ASSERT_TRUE(v.project(corner.to_array(), cx, cy, cd));
  ASSERT_EQ(cx, v.width - 1);
  std::fill(hard.begin(), hard.end(), 2.0f); // everything blocks
  // 3x3 at the corner texel: 4 taps land on the map, 5 fall off and read lit.
  EXPECT_FLOAT_EQ(shadow_visibility(v, hard.data(), corner, 0.0, 1.0f, 1.f, 3), 5.f / 9.f);
}

TEST(ShadowMap, PcfWidensTheSlopeBiasByExactlyTheFootprint) {
  using cvc::volren::camera;
  using cvc::volren::shadow_view;
  using cvc::volren::detail::fit_light_camera;
  using cvc::volren::detail::shadow_bias;

  camera cam;
  shadow_view v;
  ASSERT_TRUE(fit_light_camera(lightToward(0, 0, 1), boxOf(-1, -1, -1, 1, 1, 1), 128, cam, v));

  // Radius 0 multiplies by exactly 1.0 -- the pre-PCF expression, which is what
  // keeps an unfiltered render byte-identical rather than merely close.
  const double n_dot_l = 0.6;
  const double base = shadow_bias(v, 0.25, 1.0, 1.0, n_dot_l);
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 1.0, n_dot_l, 0.0), base);

  // The constant term never scales; only the lateral (slope) term does, by
  // 1 + 2R -- the ratio of the filter's reach (R + 0.5 texels) to the half
  // texel the unfiltered comparison covers.
  const double slope = base - 0.25;
  EXPECT_GT(slope, 0.0);
  for (const double r : {0.5, 1.0, 4.0, 16.0})
    EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 1.0, n_dot_l, r), 0.25 + slope * (1.0 + 2.0 * r));
  // With the slope term switched off there is nothing for the filter to widen.
  EXPECT_DOUBLE_EQ(shadow_bias(v, 0.25, 1.0, 0.0, n_dot_l, 8.0), 0.25);
}

// A single tap must not widen the slope bias.  pcf_half_width() collapses
// taps == 1 to a single-tap LOOKUP, but the bias is widened by (1 + 2*radius)
// to cover the filter footprint -- so passing the raw radius alongside taps==1
// biased for a filter that was not there.  The render-level inertness test did
// not catch it because its scene is not bias-sensitive; this asserts the
// contract directly, where it cannot hide.
TEST(ShadowBias, SingleTapDoesNotWidenTheSlopeBias) {
  cvc::volren::shadow_view v{};
  v.texel_world = 0.05;
  const double n_dot_l = 0.5; // a real slope, so the widening term is non-zero

  EXPECT_FLOAT_EQ(cvc::volren::detail::pcf_effective_radius(8.f, 1), 0.f)
      << "taps==1 is a single tap";
  EXPECT_FLOAT_EQ(cvc::volren::detail::pcf_effective_radius(0.f, 7), 0.f)
      << "radius 0 is a single tap";
  EXPECT_FLOAT_EQ(cvc::volren::detail::pcf_effective_radius(8.f, 7), 8.f)
      << "a real filter keeps its radius";

  const double unfiltered = cvc::volren::detail::shadow_bias(v, 0.25, 1.0, 1.0, n_dot_l, 0.0);
  const double taps1 = cvc::volren::detail::shadow_bias(
      v, 0.25, 1.0, 1.0, n_dot_l, double(cvc::volren::detail::pcf_effective_radius(8.f, 1)));
  EXPECT_DOUBLE_EQ(taps1, unfiltered)
      << "taps==1 widened the bias -- the documented byte-identical contract is broken";

  // And the widening is still real when the filter actually is wider.
  const double wide = cvc::volren::detail::shadow_bias(
      v, 0.25, 1.0, 1.0, n_dot_l, double(cvc::volren::detail::pcf_effective_radius(8.f, 7)));
  EXPECT_GT(wide, unfiltered) << "a genuine PCF footprint must still widen the bias";
}

// ---------------------------------------------------------------------------
// Ambient occlusion (detail/occlusion.h)
// ---------------------------------------------------------------------------

TEST(Occlusion, ConeIsExactOnOpenAndOnSealedSpace) {
  using cvc::volren::detail::sdf_occlusion;

  // A 16^3 grid over [0,15]^3 holding the SDF of the half-space z >= 8, i.e.
  // f = z - 8: outside (above) positive, and EXACTLY a distance field, so the
  // estimator's two exact cases can be asserted as equalities.
  grid_sampler g;
  std::vector<float> buf(16 * 16 * 16);
  for (int k = 0; k < 16; ++k)
    for (int j = 0; j < 16; ++j)
      for (int i = 0; i < 16; ++i)
        buf[std::size_t(i) + 16 * (std::size_t(j) + 16 * std::size_t(k))] = float(k) - 8.f;
  g.data = reinterpret_cast<const unsigned char *>(buf.data());
  g.type = cvc::Float;
  g.dimx = g.dimy = g.dimz = 16;
  g.minb = {0, 0, 0};
  g.span = {1, 1, 1};

  const vec3d on_surface(7.5, 7.5, 8.0);
  const vec3d up(0, 0, 1);
  // Open sky above a plane: f(p + n*h) == h at every tap, so every term
  // vanishes analytically.  The residual is the float round trip of the field
  // (the sample is interpolated in float against a distance computed in
  // double), i.e. ~1e-8 of a unit-scale field -- five orders of magnitude below
  // one 0-255 level, which is what "no occlusion on an open surface" has to
  // mean for an estimator that reads the data rather than the geometry.
  for (const int samples : {1, 3, 5, 8, 16})
    EXPECT_NEAR(sdf_occlusion(g, on_surface, up, 0.0, 3.0, samples), 0.f, 1e-6f) << samples;

  // Pointing INTO the solid is total occlusion: f goes negative immediately, so
  // every tap clamps to 1.
  for (const int samples : {1, 5, 16})
    EXPECT_FLOAT_EQ(sdf_occlusion(g, on_surface, vec3d(0, 0, -1), 0.0, 3.0, samples), 1.f)
        << samples;

  // Sideways along the surface: the plane is exactly at distance 0 the whole
  // way, so (h - 0) / h == 1 at every tap -- a point in the plane of a wall
  // sees half the sky, and this estimator reports the wall, not the sky.
  EXPECT_FLOAT_EQ(sdf_occlusion(g, on_surface, vec3d(1, 0, 0), 0.0, 3.0, 5), 1.f);

  // The isovalue is subtracted, so an OFFSET surface behaves identically: the
  // z == 10 level set has open sky above it too.
  EXPECT_NEAR(sdf_occlusion(g, vec3d(7.5, 7.5, 10.0), up, 2.0, 3.0, 5), 0.f, 1e-6f);

  // A cone that leaves the grid counts those taps UNOCCLUDED at full weight --
  // renormalizing onto the taps that remain would darken a surface as its cone
  // ran out of data, which is backwards.
  EXPECT_NEAR(sdf_occlusion(g, vec3d(7.5, 7.5, 14.0), up, 6.0, 40.0, 4), 0.f, 1e-6f);
  // Fully outside: nothing is sampled at all.
  EXPECT_FLOAT_EQ(sdf_occlusion(g, vec3d(-50, -50, -50), up, 0.0, 1.0, 5), 0.f);
  // Degenerate sample counts do not divide by zero.
  EXPECT_FLOAT_EQ(sdf_occlusion(g, on_surface, vec3d(0, 0, -1), 0.0, 3.0, 0), 0.f);

  // A DEGENERATE direction reads as UNOCCLUDED, and this one matters: a
  // flat-gradient sample normalizes to {0,0,0} (the vrNormalize contract), the
  // cone would then sample the surface itself at every tap -- distance 0
  // against a travelled h -- and report FULLY SEALED.  That is exactly the
  // sample that already gets no diffuse and no specular, so it would go black
  // instead of falling back to ambient.
  EXPECT_FLOAT_EQ(sdf_occlusion(g, on_surface, vec3d(0, 0, 0), 0.0, 3.0, 5), 0.f);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FLOAT_EQ(sdf_occlusion(g, on_surface, vec3d(nan, nan, nan), 0.0, 3.0, 5), 0.f);
}

TEST(Occlusion, FalloffIsNearWeightedAndTheRadiusKeepsMeaning) {
  using cvc::volren::detail::sdf_occlusion;

  // f = z - 8 again, but only up to z == 10, above which a second slab closes
  // the space: a NEAR ceiling and a FAR ceiling can then be told apart.
  grid_sampler g;
  std::vector<float> buf(16 * 16 * 16);
  for (int k = 0; k < 16; ++k)
    for (int j = 0; j < 16; ++j)
      for (int i = 0; i < 16; ++i)
        // Distance to the nearer of the floor (z=8) and a ceiling at z=13.
        buf[std::size_t(i) + 16 * (std::size_t(j) + 16 * std::size_t(k))] =
            float(std::min(double(k) - 8.0, 13.0 - double(k)));
  g.data = reinterpret_cast<const unsigned char *>(buf.data());
  g.type = cvc::Float;
  g.dimx = g.dimy = g.dimz = 16;
  g.minb = {0, 0, 0};
  g.span = {1, 1, 1};

  const vec3d p(7.5, 7.5, 8.0), up(0, 0, 1);
  // A cone that never reaches the ceiling sees open sky.
  EXPECT_FLOAT_EQ(sdf_occlusion(g, p, up, 0.0, 2.0, 8), 0.f);
  // One that does is occluded, and MORE occluded the further it reaches: the
  // 1/i falloff must not have flattened the far half of the cone into noise,
  // which is exactly what a 1/2^i geometric falloff does at this sample count.
  const float near_reach = sdf_occlusion(g, p, up, 0.0, 4.0, 8);
  const float far_reach = sdf_occlusion(g, p, up, 0.0, 5.0, 8);
  EXPECT_GT(near_reach, 0.f);
  EXPECT_GT(far_reach, near_reach + 0.02f)
      << "the radius knob stopped changing the answer -- the falloff is too steep";

  // Near occluders still dominate: moving the SAME ceiling closer occludes more.
  std::vector<float> low(16 * 16 * 16);
  for (int k = 0; k < 16; ++k)
    for (int j = 0; j < 16; ++j)
      for (int i = 0; i < 16; ++i)
        low[std::size_t(i) + 16 * (std::size_t(j) + 16 * std::size_t(k))] =
            float(std::min(double(k) - 8.0, 11.0 - double(k)));
  grid_sampler g2 = g;
  g2.data = reinterpret_cast<const unsigned char *>(low.data());
  EXPECT_GT(sdf_occlusion(g2, p, up, 0.0, 5.0, 8), far_reach);
}

TEST(Occlusion, LocalOutwardUndoesTheVoxelSpan) {
  using cvc::volren::detail::local_outward;

  grid_sampler g;
  g.span = {1.0, 1.0, 1.0};
  // Isotropic: the direction is just the normalized gradient.
  vec3d n = local_outward(g, vec3d(3, 0, 4));
  EXPECT_NEAR(n.x, 0.6, 1e-12);
  EXPECT_NEAR(n.z, 0.8, 1e-12);

  // Anisotropic: the spline gradient is in value-per-CELL, so a cell twice as
  // wide in x means half the world-space slope there.  Without the divide the
  // cone would march off-axis and compare the field against the wrong distance.
  g.span = {2.0, 1.0, 1.0};
  n = local_outward(g, vec3d(2, 0, 1));
  EXPECT_NEAR(n.x, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(n.z, 1.0 / std::sqrt(2.0), 1e-12);

  // A flat gradient normalizes to zero (the vrNormalize contract) rather than
  // producing NaNs, so a featureless sample simply gets no cone direction.
  n = local_outward(g, vec3d(0, 0, 0));
  EXPECT_EQ(n.x, 0.0);
  EXPECT_EQ(n.y, 0.0);
  EXPECT_EQ(n.z, 0.0);
}

// ---------------------------------------------------------------------------
// Ambient shaping (detail/shading.h)
// ---------------------------------------------------------------------------

TEST(Shading, AmbientScaleIsFlatUntilTheHemisphereIsOn) {
  using cvc::volren::hemisphere_ambient;
  using cvc::volren::detail::ambient_scale;

  hemisphere_ambient hemi; // disabled, both colours white
  for (const vec3d &n : {vec3d(0, 0, 1), vec3d(0, 0, -1), vec3d(1, 0, 0)}) {
    const std::array<float, 3> a = ambient_scale(0.375f, hemi, n, 1.f);
    EXPECT_FLOAT_EQ(a[0], 0.375f);
    EXPECT_FLOAT_EQ(a[1], 0.375f);
    EXPECT_FLOAT_EQ(a[2], 0.375f);
  }

  // Enabled but neutral: a0 == a1 makes `a0 + (a1 - a0) * f` exact for EVERY
  // normal, which is why turning the hemisphere on with its defaults is a
  // no-op rather than a rounding-sized change.
  hemi.enabled = true;
  for (const vec3d &n : {vec3d(0, 0, 1), vec3d(0, 0, -1), vec3d(0.6, 0, 0.8)}) {
    const std::array<float, 3> a = ambient_scale(0.375f, hemi, n, 1.f);
    EXPECT_EQ(a[0], 0.375f);
    EXPECT_EQ(a[1], 0.375f);
    EXPECT_EQ(a[2], 0.375f);
  }

  // Sky overhead, ground underfoot, equator exactly halfway.
  hemi.sky = {0.f, 0.f, 1.f};
  hemi.ground = {1.f, 0.f, 0.f};
  std::array<float, 3> a = ambient_scale(1.f, hemi, vec3d(0, 0, 1), 1.f);
  EXPECT_FLOAT_EQ(a[0], 0.f);
  EXPECT_FLOAT_EQ(a[2], 1.f);
  a = ambient_scale(1.f, hemi, vec3d(0, 0, -1), 1.f);
  EXPECT_FLOAT_EQ(a[0], 1.f);
  EXPECT_FLOAT_EQ(a[2], 0.f);
  a = ambient_scale(1.f, hemi, vec3d(1, 0, 0), 1.f);
  EXPECT_FLOAT_EQ(a[0], 0.5f);
  EXPECT_FLOAT_EQ(a[2], 0.5f);

  // `up` is honoured, and a DEGENERATE up puts every normal at the equator
  // rather than producing a NaN tint.
  hemi.up = {0.0, 0.0, 5.0}; // normalized on use
  a = ambient_scale(1.f, hemi, vec3d(0, 0, 1), 1.f);
  EXPECT_FLOAT_EQ(a[2], 1.f);
  hemi.up = {0.0, 0.0, 0.0};
  a = ambient_scale(1.f, hemi, vec3d(0, 0, 1), 1.f);
  EXPECT_FLOAT_EQ(a[0], 0.5f);
  EXPECT_FLOAT_EQ(a[2], 0.5f);

  // Occlusion multiplies the whole triple, after the tint.
  hemi.enabled = false;
  a = ambient_scale(0.5f, hemi, vec3d(0, 0, 1), 0.25f);
  EXPECT_FLOAT_EQ(a[0], 0.125f);
  EXPECT_FLOAT_EQ(a[1], 0.125f);
  EXPECT_FLOAT_EQ(a[2], 0.125f);
}

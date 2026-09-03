// Unit tests for the cvc::volren detail kernels: the raw-buffer grid sampler,
// trilinear interpolation, the quadratic-B-spline gradient, the marching-cubes
// case tables, and the per-cell ray/isosurface intersection.
//
// All grids here are plain std::vector buffers viewed through grid_sampler
// (a non-owning view), so no cvc::app / cvc::volume is needed.

#include <cmath>
#include <cstdint>
#include <cvc/volren/detail/cell_intersect.h>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volren/detail/sampler.h>
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

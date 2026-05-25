/*
  Copyright 2007-2011 The University of Texas at Austin

  Phase 2 unit tests for cvc/algorithm.h mesh-extraction APIs.

  Drives:
    - cvc::iso() with all extraction_method values (DUALLIB, FASTCONTOURING, LIBISOCONTOUR)
    - cvc::iso() with multiple normal_type values
    - cvc::iso() with optional property volume
    - cvc::tetrahedralize() with NO_IMPROVE / GEO_FLOW
    - cvc::hexahedralize() with NO_IMPROVE
    - cvc::tetrahedralize2()

  These indirectly exercise LBIE::Mesher and FastContouring::ContourExtractor.
*/

#include <cmath>
#include <cvc/utility/algorithm.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/core/types.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>

using namespace cvc;

namespace {

// Build an implicit-sphere scalar volume: f(x,y,z) = r - distance_to_center.
// Positive inside the sphere, negative outside. Useful for isovalue=0 extraction.
volume make_sphere_volume(app &ctx, unsigned int n, double radius) {
  bounding_box bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
  volume v(ctx, dimension(n, n, n), Float, bbox);
  const double dx = 2.0 / double(n - 1);
  for (unsigned int k = 0; k < n; ++k)
    for (unsigned int j = 0; j < n; ++j)
      for (unsigned int i = 0; i < n; ++i) {
        double x = -1.0 + i * dx;
        double y = -1.0 + j * dx;
        double z = -1.0 + k * dx;
        double d = std::sqrt(x * x + y * y + z * z);
        v(i, j, k, radius - d);
      }
  return v;
}

} // namespace

class AlgorithmTest : public ::testing::Test {
protected:
  app ctx;
};

// ---------------------------------------------------------------------------
// iso() - extraction methods
// ---------------------------------------------------------------------------

TEST_F(AlgorithmTest, IsoDualLibProducesGeometry) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g = cvc::iso(v, 0.0, DUALLIB);
  EXPECT_GT(g.num_points(), 0u);
  EXPECT_GT(g.num_tris(), 0u);
}

TEST_F(AlgorithmTest, IsoFastContouringProducesGeometry) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g = cvc::iso(v, 0.0, FASTCONTOURING);
  EXPECT_GT(g.num_points(), 0u);
  EXPECT_GT(g.num_tris(), 0u);
}

TEST_F(AlgorithmTest, IsoLibIsocontourProducesGeometry) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g;
  ASSERT_NO_THROW(g = cvc::iso(v, 0.0, LIBISOCONTOUR));
  EXPECT_GT(g.num_points(), 0u);
  EXPECT_GT(g.num_tris(), 0u);
}

// ---------------------------------------------------------------------------
// iso() - normal types
// ---------------------------------------------------------------------------

TEST_F(AlgorithmTest, IsoCentralDifferenceNormals) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g = cvc::iso(v, 0.0, DUALLIB, 0, CENTRAL_DIFFERENCE);
  EXPECT_GT(g.num_points(), 0u);
}

TEST_F(AlgorithmTest, IsoBSplineInterpolationNormals) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g = cvc::iso(v, 0.0, DUALLIB, 0, BSPLINE_INTERPOLATION);
  EXPECT_GT(g.num_points(), 0u);
}

// ---------------------------------------------------------------------------
// iso() - property volume interpolation
// ---------------------------------------------------------------------------

TEST_F(AlgorithmTest, IsoWithPropertyVolume) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  // Build a property volume colored by X coordinate
  volume prop(ctx, dimension(16, 16, 16), Float, bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0));
  for (unsigned int k = 0; k < 16; ++k)
    for (unsigned int j = 0; j < 16; ++j)
      for (unsigned int i = 0; i < 16; ++i)
        prop(i, j, k, double(i));

  geometry g;
  ASSERT_NO_THROW(g = cvc::iso(v, 0.0, DUALLIB, 0, BSPLINE_CONVOLUTION, prop));
  EXPECT_GT(g.num_points(), 0u);
}

// ---------------------------------------------------------------------------
// tetrahedralize / hexahedralize
// ---------------------------------------------------------------------------

TEST_F(AlgorithmTest, TetrahedralizeBasic) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g;
  ASSERT_NO_THROW(g = cvc::tetrahedralize(v, 0.0, DUALLIB, NO_IMPROVE));
  // Tet meshes produce some geometry; topology may be tris (boundary faces) or
  // explicit tetra storage. We just check non-empty output.
  EXPECT_GT(g.num_points(), 0u);
}

TEST_F(AlgorithmTest, TetrahedralizeWithImprovement) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g;
  ASSERT_NO_THROW(g = cvc::tetrahedralize(v, 0.0, DUALLIB, GEO_FLOW, BSPLINE_CONVOLUTION, 1));
  EXPECT_GT(g.num_points(), 0u);
}

TEST_F(AlgorithmTest, HexahedralizeBasic) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g;
  ASSERT_NO_THROW(g = cvc::hexahedralize(v, 0.0, DUALLIB, NO_IMPROVE));
  EXPECT_GT(g.num_points(), 0u);
}

TEST_F(AlgorithmTest, Tetrahedralize2Basic) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g;
  // tetrahedralize2 uses LBIE::TETRA2 internally; on synthetic input it may
  // legitimately return an empty mesh. Just exercise the code path.
  ASSERT_NO_THROW(g = cvc::tetrahedralize2(v, 0.0, DUALLIB, NO_IMPROVE));
  (void)g;
}

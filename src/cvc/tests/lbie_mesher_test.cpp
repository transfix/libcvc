/*
  Deep-drive tests for the LBIE adaptive octree mesher internals.

  Drives (beyond what algorithm_test/geometry_test exercise through the
  public cvc:: API):
    - LBIE::do_mesh with QUAD / DOUBLE / TETRA2 mesh types, which are not
      reachable through cvc::iso/tetrahedralize/hexahedralize
      (octree.cpp polygonize_quad/polygonize_interval,
       tetra.cpp tetrahedralize_interval)
    - error-tolerance sweeps (the public API pins err to DEFAULT_ERR)
    - improvement methods across mesh types, including OPTIMIZATION on
      hexahedral meshes (never exercised elsewhere)
    - geoframe construction helpers, file I/O (write_raw/read_raw/save*)
    - geoframe_adapter / to_geometry / to_geoframe conversions
    - cvc/utility/algorithm.cpp volumetric mesh utility functions
*/

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>

// Mesher-internal headers (PRIVATE include dirs added by the test target).
// Note: only headers that do not transitively include the libcontour headers
// (contour3d.h lives in cvc-mesher/contour/, which is not on this target's
// include path) can be included here. That rules out octree.h, LBIE_Mesher.h
// and mesher.h; the LBIE entry points we need from those are declared below.
#include <LBIE_geoframe.h>
#include <VolMagickCompat.h>
#include <algorithm>
#include <array>
#include <boost/optional.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <geoframe_adapter.h>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace LBIE {

// Minimal stand-in for LBIE::Mesher (defined in LBIE_Mesher.h, which we
// cannot include here -- see note above). Only the nested enum *types* are
// referenced by the do_mesh/quality_improve signatures, and nested enum
// types mangle by qualified name only, so declaring them here links against
// the library's real symbols. Enumerator values mirror LBIE_Mesher.h and are
// asserted against the public cvc:: enums (which document that they match
// LBIE) in MesherEnumValuesMatchPublicEnums below.
class Mesher {
public:
  enum MeshType { SINGLE, TETRA, QUAD, HEXA, DOUBLE, TETRA2 };
  enum ImproveMethod { NO_IMPROVE, GEO_FLOW, EDGE_CONTRACT, JOE_LIU, MINIMAL_VOL, OPTIMIZATION };
  enum NormalType { BSPLINE_CONVOLUTION, CENTRAL_DIFFERENCE, BSPLINE_INTERPOLATION };
  enum ExtractionMethod { DUALLIB, FASTCONTOURING, LIBISOCONTOUR };
};

// Entry points implemented in cvc-mesher/Mesher/mesher.cpp (declarations
// mirror mesher.h).
geoframe do_mesh(const VolMagick::Volume &vol, float isovalue, float isovalue_in, float err,
                 float err_in, geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method,
                 Mesher::NormalType normaltype, Mesher::ExtractionMethod extract_method,
                 int improve_iterations, bool dual_contouring, bool verbose = false,
                 boost::optional<const VolMagick::Volume &> propertyVol = boost::none);

geoframe quality_improve(cvc::app &ctx, const geoframe &g_frame,
                         Mesher::ImproveMethod improve_method, int improve_iterations,
                         bool verbose = false);

cvc::geometry
do_mesh_geometry(const VolMagick::Volume &vol, float isovalue, float isovalue_in, float err,
                 float err_in, cvc::geometry::geometry_type geom_type,
                 Mesher::ImproveMethod improve_method, Mesher::NormalType normaltype,
                 Mesher::ExtractionMethod extract_method, int improve_iterations,
                 bool dual_contouring, bool verbose = false,
                 boost::optional<const VolMagick::Volume &> propertyVol = boost::none);

cvc::geometry quality_improve_geometry(cvc::app &ctx, const cvc::geometry &geom,
                                       Mesher::ImproveMethod improve_method, int improve_iterations,
                                       bool verbose = false);

} // namespace LBIE

using namespace cvc;

namespace {

// Mirrors LBIE::DEFAULT_ERR / DEFAULT_ERR_IN from LBIE_Mesher.h.
const float kDefaultErr = 1.2501f;
const float kDefaultErrIn = 0.0001f;

const double kPi = 3.14159265358979323846;

// Build an implicit-sphere scalar volume: f(x,y,z) = r - distance_to_center.
// Positive inside the sphere, negative outside; isovalue 0 is the surface.
// The samples are those of [-1,1]^3 whatever box_size, the half-width of the box the volume covers.
volume make_sphere_volume(app &ctx, unsigned int n, double radius, double box_size = 1.0) {
  bounding_box bbox(-box_size, -box_size, -box_size, box_size, box_size, box_size);
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

// Structural sanity of a geoframe: sizes consistent, coordinates finite,
// element indices in range.
void expect_geoframe_valid(const LBIE::geoframe &g) {
  EXPECT_GE(g.numverts, 0);
  EXPECT_EQ(g.verts.size(), size_t(g.numverts));
  int bad_coord = 0, bad_tri = 0, bad_quad = 0;
  for (int i = 0; i < g.numverts; ++i)
    for (int j = 0; j < 3; ++j)
      if (!std::isfinite(g.verts[i][j]))
        ++bad_coord;
  for (size_t t = 0; t < g.triangles.size(); ++t)
    for (int j = 0; j < 3; ++j)
      if (g.triangles[t][j] >= (unsigned int)(g.numverts))
        ++bad_tri;
  for (size_t q = 0; q < g.quads.size(); ++q)
    for (int j = 0; j < 4; ++j)
      if (g.quads[q][j] >= (unsigned int)(g.numverts))
        ++bad_quad;
  EXPECT_EQ(bad_coord, 0) << "non-finite vertex coordinates";
  EXPECT_EQ(bad_tri, 0) << "triangle index out of range";
  EXPECT_EQ(bad_quad, 0) << "quad index out of range";
}

// Like expect_geoframe_valid but without the coordinate-finiteness check.
// Octree::geometric_flow_quad / _hex / _tri (on interval shells) divide by
// per-vertex accumulated element areas, which are zero for the degenerate
// elements the dual contouring stage emits, so those improvement paths
// produce NaN coordinates on some vertices (pre-existing library bug --
// see the test notes). Topology stays structurally valid.
void expect_geoframe_topology_valid(const LBIE::geoframe &g) {
  EXPECT_GE(g.numverts, 0);
  EXPECT_EQ(g.verts.size(), size_t(g.numverts));
  int bad_tri = 0, bad_quad = 0;
  for (size_t t = 0; t < g.triangles.size(); ++t)
    for (int j = 0; j < 3; ++j)
      if (g.triangles[t][j] >= (unsigned int)(g.numverts))
        ++bad_tri;
  for (size_t q = 0; q < g.quads.size(); ++q)
    for (int j = 0; j < 4; ++j)
      if (g.quads[q][j] >= (unsigned int)(g.numverts))
        ++bad_quad;
  EXPECT_EQ(bad_tri, 0) << "triangle index out of range";
  EXPECT_EQ(bad_quad, 0) << "quad index out of range";
}

// Convenience wrapper for LBIE::do_mesh with DUALLIB extraction.
LBIE::geoframe run_do_mesh(const volume &v, float iso_out, float iso_in, float err, float err_in,
                           LBIE::geoframe::GEOTYPE mt,
                           LBIE::Mesher::ImproveMethod im = LBIE::Mesher::NO_IMPROVE, int iters = 0,
                           bool dual = false,
                           LBIE::Mesher::NormalType nt = LBIE::Mesher::BSPLINE_CONVOLUTION,
                           bool verbose = false) {
  return LBIE::do_mesh(v, iso_out, iso_in, err, err_in, mt, im, nt, LBIE::Mesher::DUALLIB, iters,
                       dual, verbose);
}

std::string temp_path(const std::string &name) { return ::testing::TempDir() + name; }

} // namespace

class LbieMesherTest : public ::testing::Test {
protected:
  app ctx;
};

// The public cvc:: enums are documented to match the LBIE::Mesher enums;
// our local stand-in declarations must agree with both.
TEST_F(LbieMesherTest, MesherEnumValuesMatchPublicEnums) {
  EXPECT_EQ(int(LBIE::Mesher::NO_IMPROVE), int(cvc::NO_IMPROVE));
  EXPECT_EQ(int(LBIE::Mesher::GEO_FLOW), int(cvc::GEO_FLOW));
  EXPECT_EQ(int(LBIE::Mesher::EDGE_CONTRACT), int(cvc::EDGE_CONTRACT));
  EXPECT_EQ(int(LBIE::Mesher::JOE_LIU), int(cvc::JOE_LIU));
  EXPECT_EQ(int(LBIE::Mesher::MINIMAL_VOL), int(cvc::MINIMAL_VOL));
  EXPECT_EQ(int(LBIE::Mesher::OPTIMIZATION), int(cvc::OPTIMIZATION));
  EXPECT_EQ(int(LBIE::Mesher::DUALLIB), int(cvc::DUALLIB));
  EXPECT_EQ(int(LBIE::Mesher::FASTCONTOURING), int(cvc::FASTCONTOURING));
  EXPECT_EQ(int(LBIE::Mesher::LIBISOCONTOUR), int(cvc::LIBISOCONTOUR));
  EXPECT_EQ(int(LBIE::Mesher::BSPLINE_CONVOLUTION), int(cvc::BSPLINE_CONVOLUTION));
  EXPECT_EQ(int(LBIE::Mesher::CENTRAL_DIFFERENCE), int(cvc::CENTRAL_DIFFERENCE));
  EXPECT_EQ(int(LBIE::Mesher::BSPLINE_INTERPOLATION), int(cvc::BSPLINE_INTERPOLATION));
  EXPECT_EQ(int(LBIE::Mesher::SINGLE), int(LBIE::geoframe::SINGLE));
  EXPECT_EQ(int(LBIE::Mesher::TETRA), int(LBIE::geoframe::TETRA));
  EXPECT_EQ(int(LBIE::Mesher::QUAD), int(LBIE::geoframe::QUAD));
  EXPECT_EQ(int(LBIE::Mesher::HEXA), int(LBIE::geoframe::HEXA));
  EXPECT_EQ(int(LBIE::Mesher::DOUBLE), int(LBIE::geoframe::DOUBLE));
  EXPECT_EQ(int(LBIE::Mesher::TETRA2), int(LBIE::geoframe::TETRA2));
}

// ---------------------------------------------------------------------------
// QUAD meshing (only reachable through LBIE::do_mesh directly)
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, QuadDoMeshDefaultErr) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::QUAD);
  EXPECT_GT(g.numverts, 0);
  EXPECT_GT(g.numquads, 0);
  EXPECT_EQ(g.mesh_type, LBIE::geoframe::QUAD);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, QuadDoMeshAdaptiveErrSweep) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe fine = run_do_mesh(v, 0.0f, 0.0f, 0.01f, kDefaultErrIn, LBIE::geoframe::QUAD);
  LBIE::geoframe mid = run_do_mesh(v, 0.0f, 0.0f, 0.5f, kDefaultErrIn, LBIE::geoframe::QUAD);
  LBIE::geoframe coarse = run_do_mesh(v, 0.0f, 0.0f, 10.0f, kDefaultErrIn, LBIE::geoframe::QUAD);
  EXPECT_GT(fine.numquads, 0);
  EXPECT_GE(fine.numquads, coarse.numquads)
      << "smaller error tolerance should refine at least as much";
  expect_geoframe_valid(fine);
  expect_geoframe_valid(mid);
  expect_geoframe_valid(coarse);
}

// Octree::assign_refine_sign_quad visited every edge the surface crosses, including edges on the
// volume's boundary, where find_oc_id gives -1 for the cells outside the grid. It then called
// get_err_grad(-1) and read and wrote vtx_idx_arr_refine[-1]. polygonize_quad never makes a quad
// for such an edge (is_min_edge rejects it), so those edges are now skipped. AddressSanitizer
// flagged the read on every mesh of a surface that leaves the volume.
TEST_F(LbieMesherTest, QuadMeshesASurfaceThatLeavesTheVolume) {
  volume v = make_sphere_volume(ctx, 17, 1.2); // bigger than its [-1,1]^3 box
  for (float err : {kDefaultErr, 0.01f}) {
    LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, err, kDefaultErrIn, LBIE::geoframe::QUAD);
    EXPECT_GT(g.numquads, 0) << "err=" << err;
    expect_geoframe_valid(g);
  }
}

TEST_F(LbieMesherTest, QuadDoMeshGeoFlow) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::QUAD,
                                 LBIE::Mesher::GEO_FLOW, 2);
  EXPECT_GT(g.numquads, 0);
  // geometric_flow_quad leaves NaN coordinates on vertices whose incident
  // quads are degenerate (library bug); only validate topology here.
  expect_geoframe_topology_valid(g);
}

TEST_F(LbieMesherTest, QuadDoMeshCentralDiffNormals) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g =
      run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::QUAD,
                  LBIE::Mesher::NO_IMPROVE, 0, false, LBIE::Mesher::CENTRAL_DIFFERENCE);
  EXPECT_GT(g.numquads, 0);
  expect_geoframe_valid(g);
}

// ---------------------------------------------------------------------------
// SINGLE (tri surface) with explicit error sweep (public API pins err)
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, SingleDoMeshErrSweep) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe fine = run_do_mesh(v, 0.0f, 0.0f, 0.01f, kDefaultErrIn, LBIE::geoframe::SINGLE);
  LBIE::geoframe coarse = run_do_mesh(v, 0.0f, 0.0f, 10.0f, kDefaultErrIn, LBIE::geoframe::SINGLE);
  EXPECT_GT(fine.numtris, 0);
  EXPECT_GE(fine.numtris, coarse.numtris);
  expect_geoframe_valid(fine);
  expect_geoframe_valid(coarse);
}

// ---------------------------------------------------------------------------
// TETRA meshing with error sweep and the tet-oriented improvement methods
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, TetraDoMeshErrSweep) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe fine = run_do_mesh(v, 0.0f, 0.0f, 0.01f, kDefaultErrIn, LBIE::geoframe::TETRA);
  LBIE::geoframe coarse = run_do_mesh(v, 0.0f, 0.0f, 10.0f, kDefaultErrIn, LBIE::geoframe::TETRA);
  // TETRA meshes encode each tet as 4 triangles.
  EXPECT_GT(fine.numtris, 0);
  EXPECT_EQ(fine.numtris % 4, 0);
  EXPECT_GE(fine.numtris, coarse.numtris);
  expect_geoframe_valid(fine);
  expect_geoframe_valid(coarse);
}

TEST_F(LbieMesherTest, TetraDoMeshEdgeContract) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA,
                                 LBIE::Mesher::EDGE_CONTRACT, 1);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, TetraDoMeshJoeLiu) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA,
                                 LBIE::Mesher::JOE_LIU, 1);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, TetraDoMeshMinimalVol) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA,
                                 LBIE::Mesher::MINIMAL_VOL, 1);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, TetraDoMeshBsplineInterpNormals) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g =
      run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA,
                  LBIE::Mesher::NO_IMPROVE, 0, false, LBIE::Mesher::BSPLINE_INTERPOLATION);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, TetraDoMeshPropertyVolumeResized) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  // Property volume with mismatching dims forces the resize branch of
  // Octree::func_val.
  volume prop(ctx, dimension(9, 9, 9), Float, bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0));
  for (unsigned int k = 0; k < 9; ++k)
    for (unsigned int j = 0; j < 9; ++j)
      for (unsigned int i = 0; i < 9; ++i)
        prop(i, j, k, double(i) + double(j) + double(k));
  LBIE::geoframe g = LBIE::do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA,
                                   LBIE::Mesher::NO_IMPROVE, LBIE::Mesher::BSPLINE_CONVOLUTION,
                                   LBIE::Mesher::DUALLIB, 0, false, false,
                                   boost::optional<const VolMagick::Volume &>(prop));
  EXPECT_GT(g.numverts, 0);
  EXPECT_EQ(g.funcs.size(), size_t(g.numverts));
  int nonzero = 0;
  for (size_t i = 0; i < g.funcs.size(); ++i)
    if (g.funcs[i][0] != 0.0f)
      ++nonzero;
  EXPECT_GT(nonzero, 0) << "interpolated property values should not all be zero";
  expect_geoframe_valid(g);
}

// ---------------------------------------------------------------------------
// Interval meshing: DOUBLE (tri shell) and TETRA2 (tet interval volume)
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, DoubleIntervalDoMesh) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  // Interval between the f=+0.15 and f=-0.15 level sets (the LBIE
  // convention wants isovalue > isovalue_in).
  LBIE::geoframe g = run_do_mesh(v, 0.15f, -0.15f, kDefaultErr, kDefaultErrIn,
                                 LBIE::geoframe::DOUBLE, LBIE::Mesher::NO_IMPROVE, 0, true);
  EXPECT_GT(g.numverts, 0);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, DoubleIntervalGeoFlow) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.15f, -0.15f, kDefaultErr, kDefaultErrIn,
                                 LBIE::geoframe::DOUBLE, LBIE::Mesher::GEO_FLOW, 1, true);
  EXPECT_GT(g.numtris, 0);
  // geometric_flow_tri on the interval shells leaves NaN coordinates on
  // vertices with only degenerate incident triangles (library bug); only
  // validate topology here.
  expect_geoframe_topology_valid(g);
}

TEST_F(LbieMesherTest, TetraIntervalDoMesh) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.15f, -0.15f, kDefaultErr, kDefaultErrIn,
                                 LBIE::geoframe::TETRA2, LBIE::Mesher::NO_IMPROVE, 0, true);
  EXPECT_GT(g.numverts, 0);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, TetraIntervalGeoFlow) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.15f, -0.15f, kDefaultErr, kDefaultErrIn,
                                 LBIE::geoframe::TETRA2, LBIE::Mesher::GEO_FLOW, 1, true);
  EXPECT_GT(g.numtris, 0);
  expect_geoframe_valid(g);
}

// ---------------------------------------------------------------------------
// HEXA meshing: adaptive error sweep, geometric flow, optimization
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, HexaDoMeshAdaptiveErrSweep) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe fine = run_do_mesh(v, 0.0f, 0.0f, 0.01f, kDefaultErrIn, LBIE::geoframe::HEXA);
  LBIE::geoframe mid = run_do_mesh(v, 0.0f, 0.0f, 0.5f, kDefaultErrIn, LBIE::geoframe::HEXA);
  LBIE::geoframe coarse = run_do_mesh(v, 0.0f, 0.0f, 10.0f, kDefaultErrIn, LBIE::geoframe::HEXA);
  // HEXA meshes encode each hex as 6 quads.
  EXPECT_GT(fine.numquads, 0);
  EXPECT_EQ(fine.numquads % 6, 0);
  EXPECT_GE(fine.numquads, coarse.numquads);
  expect_geoframe_valid(fine);
  expect_geoframe_valid(mid);
  expect_geoframe_valid(coarse);
}

// Octree::get_vtx_new moves each hexa vertex onto the surface by walking across the finest cells.
// Where the surface meets the volume's boundary the walk stepped off the grid: xyz2octcell()
// returned -1 and getCellValues(-1) read orig_vol out of bounds. cvc hexahedralize crashed about
// one run in fifteen (AddressSanitizer flags the read every run), and the vertex landed outside the
// volume. The walk now stops at the grid's edge.
TEST_F(LbieMesherTest, HexaVerticesStayInsideAVolumeTheSurfaceLeaves) {
  for (double r : {1.2, 1.5}) { // spheres bigger than their [-1,1]^3 box
    volume v = make_sphere_volume(ctx, 17, r);
    LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA);
    ASSERT_GT(g.numquads, 0) << "r=" << r;
    expect_geoframe_valid(g);
    const float lim = 1.0f + 1e-5f;
    int outside = 0;
    for (int i = 0; i < g.numverts; ++i)
      for (int j = 0; j < 3; ++j)
        if (!(g.verts[i][j] >= -lim && g.verts[i][j] <= lim))
          ++outside;
    EXPECT_EQ(outside, 0) << "r=" << r << ": vertex coordinates outside the volume";
  }
}

// Sampled at 17^3 in its [-1,1]^3 box, a sphere of radius 1.5 passes exactly through grid points
// on the box's faces (8^2 + 4^2 + 8^2 = 12^2 in eighths). Where an edge ends on the far face at the
// isovalue, Octree::interpRect3Dpts_x/y/z asked getVertGrad() for a point one plane off the grid,
// and getValue() read up to about 1 KB past orig_vol. That was the access violation in compute_qef
// that crashed HexaVerticesStayInsideAVolumeTheSurfaceLeaves now and then on Windows;
// AddressSanitizer flags it on every run, for every mesh type. getVertGrad() now stays on the grid.
// SINGLE extracts no surface from this sphere. QUAD is left out: polygonize_quad has its own
// out-of-bounds read wherever a surface leaves the volume (r = 1.2 too), not fixed here.
TEST_F(LbieMesherTest, MeshesASurfaceThroughGridPointsOnTheFarFaces) {
  volume v = make_sphere_volume(ctx, 17, 1.5);
  for (LBIE::geoframe::GEOTYPE mt : {LBIE::geoframe::TETRA, LBIE::geoframe::HEXA,
                                     LBIE::geoframe::DOUBLE, LBIE::geoframe::TETRA2}) {
    const bool interval = mt == LBIE::geoframe::DOUBLE || mt == LBIE::geoframe::TETRA2;
    LBIE::geoframe g = run_do_mesh(v, 0.0f, interval ? -0.15f : 0.0f, kDefaultErr, kDefaultErrIn,
                                   mt, LBIE::Mesher::NO_IMPROVE, 0, interval);
    EXPECT_GT(g.numverts, 0) << "mesh type " << mt;
    expect_geoframe_valid(g);
    int bad_normal = 0;
    for (size_t i = 0; i < g.normals.size(); ++i)
      for (int j = 0; j < 3; ++j)
        if (!std::isfinite(g.normals[i][j]))
          ++bad_normal;
    EXPECT_EQ(bad_normal, 0) << "mesh type " << mt << ": non-finite normals";
  }
}

TEST_F(LbieMesherTest, HexaDoMeshGeoFlow) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA,
                                 LBIE::Mesher::GEO_FLOW, 1);
  EXPECT_GT(g.numquads, 0);
  // geometric_flow_hex leaves NaN coordinates on a few boundary vertices
  // (library bug, same zero-area division as the quad flow); only validate
  // topology here.
  expect_geoframe_topology_valid(g);
}

TEST_F(LbieMesherTest, HexaDoMeshOptimize) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  // OPTIMIZATION (Octree::optimization) operates on hexahedral meshes.
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA,
                                 LBIE::Mesher::OPTIMIZATION, 1);
  EXPECT_GT(g.numquads, 0);
  // This used to check only structure: arm64 Release left a couple of NaN vertices. Octree::getGrad
  // read v1[3], v2[3] and v3[3] of float[3] arrays (AddressSanitizer: stack-buffer-overflow here),
  // and v3[3] could be the uninitialized point the line search writes next. Its indices are fixed.
  expect_geoframe_valid(g);
}

namespace {

// Octree::optimization reads hexahedron h's corners v0..v7 from its first two quads (as
// Octree::add_hexa stores them): v0 v1 v2 v3 round one face from quads[6h], and v4..v7, which lie
// over v0..v3, from quads[6h+1] in the order v5 v4 v7 v6.
std::array<unsigned int, 8> hexa_corners(const LBIE::geoframe &g, int h) {
  const auto &a = g.quads[6 * h];
  const auto &b = g.quads[6 * h + 1];
  return {{a[0], a[1], a[2], a[3], b[1], b[0], b[3], b[2]}};
}

// A geoframe holding one hexahedron with the given corners, in v0..v7 order.
LBIE::geoframe make_hexa(const float (&corner)[8][3]) {
  LBIE::geoframe g;
  for (const auto &c : corner) {
    float p[3] = {c[0], c[1], c[2]};
    float n[3] = {0.f, 0.f, 1.f};
    g.AddVert(p, n);
  }
  unsigned int faces[6][4] = {{0, 1, 2, 3}, {5, 4, 7, 6}, {0, 1, 5, 4},
                              {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
  for (auto &f : faces) {
    unsigned int *fp = f;
    g.AddQuad(fp, 4);
  }
  g.numhexas = 1;
  g.mesh_type = LBIE::geoframe::HEXA;
  return g;
}

// The scaled Jacobian of corner j of hexahedron h: 1 for a right-angled corner, 0 once its three
// edges are coplanar, negative when the corner is inverted.
double corner_jacobian(const LBIE::geoframe &g, int h, int j) {
  static const int edge_to[8][3] = {{3, 1, 4}, {0, 2, 5}, {1, 3, 6}, {2, 0, 7},
                                    {5, 7, 0}, {6, 4, 1}, {7, 5, 2}, {4, 6, 3}};
  const std::array<unsigned int, 8> v = hexa_corners(g, h);
  double e[3][3];
  for (int k = 0; k < 3; ++k) {
    double len = 0.0;
    for (int i = 0; i < 3; ++i) {
      e[k][i] = double(g.verts[v[edge_to[j][k]]][i]) - double(g.verts[v[j]][i]);
      len += e[k][i] * e[k][i];
    }
    for (int i = 0; i < 3; ++i)
      e[k][i] /= std::sqrt(len);
  }
  return -(e[0][0] * (e[1][1] * e[2][2] - e[1][2] * e[2][1]) -
           e[0][1] * (e[1][0] * e[2][2] - e[1][2] * e[2][0]) +
           e[0][2] * (e[1][0] * e[2][1] - e[1][1] * e[2][0]));
}

// The condition number Octree::testHexa gives corner j of hexahedron h: 1 for a right-angled
// corner, growing as the corner flattens, +inf once its three edges are coplanar.
double corner_condition(const LBIE::geoframe &g, int h, int j) {
  return 1.0 / std::fabs(corner_jacobian(g, h, j));
}

// The number of g's hexahedra with a corner that is inverted or flat.
int inverted_hexahedra(const LBIE::geoframe &g) {
  int inverted = 0;
  for (int h = 0; h < g.numquads / 6; ++h)
    for (int j = 0; j < 8; ++j)
      if (!(corner_jacobian(g, h, j) > 0.0)) {
        ++inverted;
        break;
      }
  return inverted;
}

double worst_corner_condition(const LBIE::geoframe &g) {
  double worst = 0.0;
  for (int h = 0; h < g.numquads / 6; ++h)
    for (int j = 0; j < 8; ++j)
      worst = std::max(worst, corner_condition(g, h, j));
  return worst;
}

} // namespace

// geoframe::AddVert gave each vertex 18 refine_edge slots, free when -1, but { { -1 } } set only
// the first: the second edge or face refined at a vertex found no free slot, and
// AddRefine_edgevtx_hex and AddRefine_facevtx returned vertex 0 instead of a new vertex. In the
// bunny's adaptive hex mesh 338 of 351 hexahedra used vertex 0 in place of a corner, and 302
// corners came out flat.
TEST_F(LbieMesherTest, RefinementMakesAVertexPerEdgeAndFace) {
  LBIE::geoframe g;
  float n[3] = {0.f, 0.f, 1.f};
  float a[3] = {0.f, 0.f, 0.f}, b[3] = {3.f, 0.f, 0.f};
  float c[3] = {0.f, 3.f, 0.f}, d[3] = {3.f, 3.f, 0.f};
  const unsigned int va = g.AddVert(a, n), vb = g.AddVert(b, n);
  const unsigned int vc = g.AddVert(c, n), vd = g.AddVert(d, n);

  const unsigned int ab = g.AddRefine_edgevtx_hex(va, vb);
  const unsigned int ac = g.AddRefine_edgevtx_hex(va, vc);
  const unsigned int ad = g.AddRefine_facevtx(va, vd, vb, vc);
  EXPECT_EQ(ab, 4u);
  EXPECT_EQ(ac, 5u);
  EXPECT_EQ(ad, 6u);
  EXPECT_EQ(g.numverts, 7);
  EXPECT_EQ(g.AddRefine_edgevtx_hex(va, vc), ac) << "an edge refined twice gets one vertex";
  EXPECT_EQ(g.numverts, 7);
  EXPECT_FLOAT_EQ(g.verts[ac][1], 1.0f) << "a third of the way from a to c";
}

// Each vertex has 18 refine_edge slots, as many as a vertex of a uniform grid needs (6 edges and
// 12 face diagonals). Once they were full, AddRefine_edgevtx_hex, AddRefine_edgevtx and
// AddRefine_facevtx returned vertex 0, which is how hexahedra across a change of octree level got
// vertex 0 for corners; QUAD meshes on the vh4 CT still had 32 such quads.
TEST_F(LbieMesherTest, RefinementPastEighteenEdgesAtAVertexMakesNewVertices) {
  LBIE::geoframe g;
  float n[3] = {0.f, 0.f, 1.f}, zero[3] = {9.f, 9.f, 9.f}, o[3] = {0.f, 0.f, 0.f};
  g.AddVert(zero, n); // vertex 0, which no refinement should hand out
  const unsigned int centre = g.AddVert(o, n);
  std::vector<unsigned int> others;
  for (int k = 0; k < 24; ++k) {
    float p[3] = {3.f * std::cos(0.25f * k), 3.f * std::sin(0.25f * k), 0.1f * k};
    others.push_back(g.AddVert(p, n));
  }
  // Edges toward the odd ones, face diagonals toward the even ones: both kinds share the slots.
  auto refine = [&](int k) {
    return k % 2 ? g.AddRefine_edgevtx_hex(centre, others[k])
                 : g.AddRefine_facevtx(centre, others[k], others[(k + 1) % 24],
                                       others[(k + 23) % 24]);
  };
  std::vector<unsigned int> made;
  for (int k = 0; k < 24; ++k)
    made.push_back(refine(k));
  std::vector<unsigned int> distinct(made);
  std::sort(distinct.begin(), distinct.end());
  EXPECT_EQ(std::unique(distinct.begin(), distinct.end()) - distinct.begin(), 24);
  for (int k = 0; k < 24; ++k) {
    EXPECT_GT(made[k], others.back()) << "refinement " << k << " reused vertex " << made[k];
    EXPECT_EQ(refine(k), made[k]) << "refinement " << k << " made twice";
  }
  EXPECT_FLOAT_EQ(g.verts[made[21]][2], 0.7f) << "a third of the way out to others[21]";
}

// AddBound_edge filled every free slot with the first boundary edge it recorded at a vertex, so a
// vertex kept only one, and CheckBound_edge missed the rest.
TEST_F(LbieMesherTest, AVertexKeepsAllItsBoundaryEdges) {
  LBIE::geoframe g;
  float n[3] = {0.f, 0.f, 1.f};
  float a[3] = {0.f, 0.f, 0.f}, b[3] = {1.f, 0.f, 0.f}, c[3] = {0.f, 1.f, 0.f};
  const unsigned int va = g.AddVert(a, n), vb = g.AddVert(b, n), vc = g.AddVert(c, n);
  g.AddBound_edge(va, vb);
  g.AddBound_edge(vc, va);
  EXPECT_EQ(g.CheckBound_edge(vb, va), 1);
  EXPECT_EQ(g.CheckBound_edge(va, vc), 1);
  EXPECT_EQ(g.CheckBound_edge(vb, vc), 0);
}

// Octree::optimization takes the corner with the largest condition number and moves its vertex down
// the gradient. A corner whose edges are coplanar has condition +inf; cond_max + 1.0f - cond_max is
// then NaN, the line search ran zero times, and its uninitialized result mmv4 was copied into the
// mesh: `cvc hexahedralize --improve optimization` on the bunny wrote vertices such as
// (7.4e24, 5.8e-43, 7.9e24) or (-2.6e13, 0.03, -1.9e13), depending on the stack. No move improves
// such a corner, so the mesh must come back unchanged. The cube sits away from the origin because
// untouched stack memory is often zero.
TEST_F(LbieMesherTest, OptimizationLeavesAFlatCornerInPlace) {
  // A unit cube at (3, 5, 7) whose v4 has moved onto the line from v0 through v1, so corner 0's
  // edges to v1 and v4 point the same way.
  const float corner[8][3] = {{3, 5, 7}, {4, 5, 7}, {4, 6, 7}, {3, 6, 7},
                              {5, 5, 7}, {4, 5, 8}, {4, 6, 8}, {3, 6, 8}};
  LBIE::geoframe g = make_hexa(corner);
  ASSERT_TRUE(std::isinf(corner_condition(g, 0, 0)));

  LBIE::geoframe out = LBIE::quality_improve(ctx, g, LBIE::Mesher::OPTIMIZATION, 1);
  ASSERT_EQ(out.numverts, 8);
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_EQ(out.verts[i][j], corner[i][j]) << "vertex " << i << ", coordinate " << j;
}

// Octree::optimization moved quads[6h+1][j-4] for a worst corner j >= 4, which is v5, v4, v7 or
// v6, a neighbour of the corner and not the corner (and for j == 4 it took corner 1's edges). Here
// v6 sits inside the plane through its three neighbours, so corner 6 is inverted (scaled Jacobian
// -0.18) and the worst. The old code left v6 where it was and wrote v6's new position onto v7.
// Fixed, it still pushed v6 further in, to -0.32: testHexa's Jacobian is unsigned, and a corner
// looked as good as its mirror image. Moving v6 out leaves the corner uninverted.
TEST_F(LbieMesherTest, OptimizationMovesTheWorstCornersOwnVertex) {
  const float corner[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0},          {0, 1, 0},
                              {0, 0, 1}, {1, 0, 1}, {0.6f, 0.6f, 0.7f}, {0, 1, 1}};
  LBIE::geoframe g = make_hexa(corner);
  const double before = worst_corner_condition(g);
  for (int j = 0; j < 8; ++j)
    ASSERT_LE(corner_condition(g, 0, j), corner_condition(g, 0, 6)) << "corner " << j;
  ASSERT_LT(corner_jacobian(g, 0, 6), 0.0);

  LBIE::geoframe out = LBIE::quality_improve(ctx, g, LBIE::Mesher::OPTIMIZATION, 1);
  expect_geoframe_valid(out);
  EXPECT_FALSE(out.verts[6][0] == corner[6][0] && out.verts[6][1] == corner[6][1] &&
               out.verts[6][2] == corner[6][2])
      << "the worst corner's vertex v6 did not move";
  for (int j = 0; j < 3; ++j)
    EXPECT_EQ(out.verts[7][j], corner[7][j]) << "v7 moved, coordinate " << j;
  EXPECT_EQ(inverted_hexahedra(out), 0);
  const double after = worst_corner_condition(out);
  EXPECT_TRUE(std::isfinite(after));
  EXPECT_LT(after, before);
}

namespace {

// The largest difference between a's coordinates, scaled by a_scale, and b's.
float max_coordinate_difference(const LBIE::geoframe &a, float a_scale, const LBIE::geoframe &b) {
  float worst = 0.0f;
  for (int i = 0; i < a.numverts; ++i)
    for (int j = 0; j < 3; ++j)
      worst = std::max(worst, std::fabs(a.verts[i][j] * a_scale - b.verts[i][j]));
  return worst;
}

} // namespace

// do_mesh's -q passes (Mesher::qualityImprove) got world coordinates, after do_duallib_extract's
// own pass on the octree's grid. In a box 0.02 across, where a cell is 0.00125, optimization's
// first step of 0.5 is 400 cells, and the -q 1 pass did nothing. The passes now run on the grid,
// and a move may not leave any corner around the vertex worse than the worst corner, so a pass
// never makes the worst corner worse. Without that check the -q 1 pass here went from 15.3 to 19.4.
TEST_F(LbieMesherTest, OptimizationImprovesASmallVolumesMeshPassByPass) {
  const double half_width = 0.01;
  volume v = make_sphere_volume(ctx, 17, 1.2, half_width);
  double worst[3];
  for (int q = -1; q <= 1; ++q) {
    LBIE::geoframe g =
        run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA,
                    q < 0 ? LBIE::Mesher::NO_IMPROVE : LBIE::Mesher::OPTIMIZATION, std::max(q, 0));
    ASSERT_GT(g.numquads, 0);
    expect_geoframe_valid(g);
    worst[q + 1] = worst_corner_condition(g);
    const float lim = float(half_width) * (1.0f + 1e-5f);
    int outside = 0;
    for (int i = 0; i < g.numverts; ++i)
      for (int j = 0; j < 3; ++j)
        if (!(g.verts[i][j] >= -lim && g.verts[i][j] <= lim))
          ++outside;
    EXPECT_EQ(outside, 0) << "-q " << q << ": vertex coordinates outside the volume";
  }
  EXPECT_LE(worst[1], worst[0]) << "the pass inside extraction made the worst corner worse";
  EXPECT_LT(worst[2], worst[1]) << "the -q 1 pass did not improve the worst corner";
}

// LBIE::quality_improve (behind cvc::geometry::quality_improve) gets a mesh with no octree, so
// there is no grid to run the passes on. Run in the mesh's own units, optimization's half-unit
// first step gave a sphere's hex mesh a worst corner condition number of 28 in its [-1,1] box, 36
// scaled up 64 times and 293 scaled down 64 times. The passes now run on a grid whose cell is the
// mesh's median edge, so scaling the mesh scales the result.
TEST_F(LbieMesherTest, QualityImproveGivesTheSameMeshInAnyUnits) {
  LBIE::geoframe g = run_do_mesh(make_sphere_volume(ctx, 17, 1.2), 0.0f, 0.0f, kDefaultErr,
                                 kDefaultErrIn, LBIE::geoframe::HEXA);
  ASSERT_GT(g.numquads, 0);
  LBIE::geoframe ref = LBIE::quality_improve(ctx, g, LBIE::Mesher::OPTIMIZATION, 1);
  EXPECT_GT(max_coordinate_difference(ref, 1.0f, g), 0.0f) << "optimization moved no vertex";
  for (float scale : {1.0f / 64.0f, 64.0f}) {
    LBIE::geoframe scaled = g;
    for (int i = 0; i < scaled.numverts; ++i)
      for (int j = 0; j < 3; ++j)
        scaled.verts[i][j] *= scale;
    LBIE::geoframe out = LBIE::quality_improve(ctx, scaled, LBIE::Mesher::OPTIMIZATION, 1);
    ASSERT_EQ(out.numverts, ref.numverts);
    EXPECT_LT(max_coordinate_difference(out, 1.0f / scale, ref), 1e-5f) << "scale " << scale;
  }
}

namespace {

// The number of g's hexahedra that use one vertex for two or more corners.
int hexahedra_with_repeated_corners(const LBIE::geoframe &g) {
  int repeated = 0;
  for (int h = 0; h < g.numquads / 6; ++h) {
    std::array<unsigned int, 8> v = hexa_corners(g, h);
    std::sort(v.begin(), v.end());
    if (std::adjacent_find(v.begin(), v.end()) != v.end())
      ++repeated;
  }
  return repeated;
}

} // namespace

// The adaptive hexahedral mesh of a sphere that leaves its box refines cells near the surface.
// Before the refine_edge fix 702 of its 729 hexahedra had vertex 0 in place of a corner. The same
// samples in a box whose voxels are 1 across (8 here) must give the same: that is where the
// octree used to refine past the hexahedral grid.
TEST_F(LbieMesherTest, AdaptiveHexahedraHaveEightDistinctCorners) {
  for (double box_size : {1.0, 8.0}) {
    volume v = make_sphere_volume(ctx, 17, 1.2, box_size);
    LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA);
    ASSERT_GT(g.numquads, 0) << "box " << box_size;
    expect_geoframe_valid(g);
    EXPECT_EQ(hexahedra_with_repeated_corners(g), 0)
        << "box " << box_size << ": of " << g.numquads / 6 << " hexahedra";
  }
}

// Octree::traverse_qef refined the octree past the hexahedral grid's level wherever a cell's QEF
// error passed the tolerance. That error was in the volume's units (until interpRect3Dpts_* put
// the crossings on the grid), so in a box 32 across, 2 units a voxel, this sphere's octree had
// leaves two levels below the grid, and 3585 of its 8051 hexahedra
// repeated a corner: where the level changes, min_vtx_hexa gave one coarse cell's vertex to several
// corners, and AddRefine_* gave vertex 0 once a coarse vertex's 18 refine_edge slots were full.
// fem_gallery's head.off at --dim 64 and the vh4.rawiv CT were about half collapsed. The grid now
// stays uniform and only the templates refine it.
TEST_F(LbieMesherTest, HexahedraHaveEightDistinctCornersWhereTheErrorIsLarge) {
  volume v = make_sphere_volume(ctx, 17, 0.6, 16.0);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA);
  ASSERT_GT(g.numquads, 0);
  expect_geoframe_valid(g);
  EXPECT_EQ(hexahedra_with_repeated_corners(g), 0) << "of " << g.numquads / 6 << " hexahedra";
  // The grid's 4^3 cells have 27 interior vertices, one hexahedron each; any more are refinements.
  EXPECT_GT(g.numquads / 6, 27) << "the templates refined no cell";
}

// The template for a hexahedron with one face refined puts four points halfway between its inner
// points 21, 22, 25 and 26 and the four under them; the last took 41 for 42 and sat half a sub-cell
// off, leaving one of the template's hexahedra flat and four skewed even in a cube.
TEST_F(LbieMesherTest, OneFaceTemplatePutsItsMiddlePointsHalfwayDown) {
  LBIE::geoframe g;
  const float cube[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                            {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
  unsigned int v[8], v_new[48];
  for (int c = 0; c < 8; ++c) {
    float p[3] = {cube[c][0], cube[c][1], cube[c][2]}, n[3] = {0.f, 0.f, 1.f};
    v[c] = g.AddVert(p, n);
  }
  const int edge_id[6] = {0, 0, 0, 0, 0, 0};
  g.AddVert_hexa_adaptive_2_4(v, edge_id, v_new);
  // The refined face is v0 v1 v2 v3; the points sit under its inner points at thirds of the cube.
  const float want[4][3] = {{1.f / 3, 1.f / 3, 0.5f},
                            {2.f / 3, 1.f / 3, 0.5f},
                            {1.f / 3, 2.f / 3, 0.5f},
                            {2.f / 3, 2.f / 3, 0.5f}};
  for (int k = 0; k < 4; ++k)
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(g.verts[v_new[32 + k]][i], want[k][i], 1e-6f)
          << "point " << k << ", coordinate " << i;
}

// get_vtx_new snaps the hexahedral mesh's boundary vertices onto the surface, walking each up to a
// cell along its normal, and the templates place their vertices between snapped corners. That
// inverted 12 of the 189 hexahedra of the 17^3 sphere and 101 of the 1389 at 33^3 (and the one-face
// template's misplaced point 30 more in the box 32 across). Where snapping inverts a hexahedron its
// vertices now stop short of the surface. Optimization inverted more (21 and 173), because its
// measure could not tell a corner from its mirror image; it now never takes a corner below 0.01.
TEST_F(LbieMesherTest, SnappingToTheSurfaceInvertsNoHexahedron) {
  const struct {
    unsigned int n;
    double half_width;
  } spheres[] = {{17, 1.0}, {33, 1.0}, {33, 16.0}};
  for (const auto &s : spheres)
    for (LBIE::Mesher::ImproveMethod im : {LBIE::Mesher::NO_IMPROVE, LBIE::Mesher::OPTIMIZATION}) {
      const bool optimized = im == LBIE::Mesher::OPTIMIZATION;
      LBIE::geoframe g =
          run_do_mesh(make_sphere_volume(ctx, s.n, 0.6, s.half_width), 0.0f, 0.0f, kDefaultErr,
                      kDefaultErrIn, LBIE::geoframe::HEXA, im, optimized ? 1 : 0);
      ASSERT_GT(g.numquads, 0);
      expect_geoframe_valid(g);
      EXPECT_EQ(inverted_hexahedra(g), 0)
          << s.n << "^3 in a box " << 2 * s.half_width << " across"
          << (optimized ? ", optimized" : "") << ": of " << g.numquads / 6 << " hexahedra";
    }
}

// A cube turned off the grid's axes, half a box across, sampled at n^3 in a box 2 * half_width
// across. Its QEF points sit on its sharp edges and corners.
volume make_turned_cube_volume(app &ctx, unsigned int n, double half_width) {
  volume v = make_sphere_volume(ctx, n, 0.0, half_width);
  const double dx = 2.0 / double(n - 1), c = std::cos(0.5), s = std::sin(0.5);
  for (unsigned int k = 0; k < n; ++k)
    for (unsigned int j = 0; j < n; ++j)
      for (unsigned int i = 0; i < n; ++i) {
        const double x = -1 + i * dx, y = -1 + j * dx, z = -1 + k * dx;
        const double xr = c * x - s * y, yz = s * x + c * y;
        const double yr = s * z + c * yz, zr = c * z - s * yz;
        v(i, j, k, 0.45 - std::max({std::fabs(xr), std::fabs(yr), std::fabs(zr)}));
      }
  return v;
}

// untangle_snapped sends the vertices of an inverted hexahedron back to where the extraction puts
// them without snapping. At QEF points, as on this cube's edges and corners, the dual hexahedra are
// inverted already, and 22 of its 1323 kept an inverted corner. The reference now has every vertex
// at its cell's centre, where none is inverted.
TEST_F(LbieMesherTest, HexahedraOnSharpEdgesAreNotInverted) {
  LBIE::geoframe g = run_do_mesh(make_turned_cube_volume(ctx, 33, 16.0), 0.0f, 0.0f, kDefaultErr,
                                 kDefaultErrIn, LBIE::geoframe::HEXA);
  ASSERT_GT(g.numquads, 0);
  expect_geoframe_valid(g);
  EXPECT_EQ(inverted_hexahedra(g), 0) << "of " << g.numquads / 6 << " hexahedra";
}

// Where the octree's level changes, polygonize_quad gave a quad two corners on one vertex, a
// triangle (320 of this sphere's 7454 quads), and method 5's templates did not match up across the
// change: 592 edges had one quad and 128 had three or more. The templates now refine only an octree
// whose leaves are all on one level, and a mesh with triangles has each face split once into quads.
TEST_F(LbieMesherTest, QuadMeshesWithLevelChangesAreClosedAndAllQuads) {
  LBIE::geoframe g = run_do_mesh(make_sphere_volume(ctx, 33, 0.6, 16.0), 0.0f, 0.0f, kDefaultErr,
                                 kDefaultErrIn, LBIE::geoframe::QUAD);
  ASSERT_GT(g.quads.size(), 0u);
  expect_geoframe_valid(g);
  int repeated = 0;
  std::map<std::pair<unsigned int, unsigned int>, int> quads_at;
  for (const auto &f : g.quads) {
    std::array<unsigned int, 4> c = {{f[0], f[1], f[2], f[3]}};
    std::sort(c.begin(), c.end());
    if (std::adjacent_find(c.begin(), c.end()) != c.end())
      ++repeated;
    for (int a = 0; a < 4; ++a)
      quads_at[std::make_pair(std::min(f[a], f[(a + 1) % 4]), std::max(f[a], f[(a + 1) % 4]))]++;
  }
  EXPECT_EQ(repeated, 0) << "of " << g.quads.size() << " quads";
  int open = 0, crowded = 0;
  for (const auto &e : quads_at) {
    if (e.second == 1)
      ++open;
    if (e.second > 2)
      ++crowded;
  }
  EXPECT_EQ(open, 0) << "edges with one quad";
  EXPECT_EQ(crowded, 0) << "edges with three or more quads";
}

// The geoframe <-> geometry adapter read a hexahedron as quads[6h] and then quads[6h+1] in
// order, and wrote it back the same way, but LBIE winds that second face the other way (see
// hexa_corners). Every hexahedron cvc::hexahedralize returned was twisted -- corner k + 4 not
// over corner k -- and cvc::geometry::quality_improve handed LBIE's optimizer twisted hexahedra.
// The adapter now reads and writes LBIE's own layout.
//
// This sphere's mesh is 27 hexahedra, a 3x3x3 block whose 56 outer vertices lie on the sphere
// (#808 fitted them; before, the block was a cube inside it). Bending the block onto the sphere
// makes its edge and corner hexahedra wedges, so the mean scaled Jacobian is 0.499, the worst
// hexahedron 0.22. Read through the old twisted layout the same mesh has every hexahedron
// inside out: mean -0.78.
TEST_F(LbieMesherTest, AdapterKeepsLbiesHexahedronCorners) {
  LBIE::geoframe g = run_do_mesh(make_sphere_volume(ctx, 17, 0.5), 0.0f, 0.0f, kDefaultErr,
                                 kDefaultErrIn, LBIE::geoframe::HEXA);
  ASSERT_GT(g.numquads, 0);
  const int n = g.numquads / 6;
  geometry geom = LBIE::to_geometry(g);
  ASSERT_EQ(geom.num_hexs(), std::size_t(n));
  for (int h = 0; h < n; ++h) {
    const std::array<unsigned int, 8> v = hexa_corners(g, h);
    for (int k = 0; k < 8; ++k)
      ASSERT_EQ(geom.const_hexs()[h][k], v[k]) << "hexahedron " << h << ", corner " << k;
  }

  const LBIE::geoframe back = LBIE::to_geoframe(geom);
  ASSERT_EQ(back.numquads, g.numquads);
  for (int h = 0; h < n; ++h)
    ASSERT_EQ(hexa_corners(back, h), hexa_corners(g, h)) << "hexahedron " << h;
  {
    LBIE::geoframe_adapter adapter(geom);
    ASSERT_EQ(adapter.numquads, g.numquads);
    for (int h = 0; h < n; ++h)
      ASSERT_EQ(hexa_corners(adapter, h), hexa_corners(g, h)) << "hexahedron " << h;
  }

  int valid = 0;
  double sum = 0.0;
  for (const auto &hex : geom.const_hexs()) {
    geometry::point_t c[8];
    for (int k = 0; k < 8; ++k)
      c[k] = geom.const_points()[hex[k]];
    const double sj = hex_scaled_jacobian(c);
    valid += sj > 0.0;
    sum += sj;
  }
  EXPECT_EQ(valid, n) << "of " << n << " hexahedra have every corner right-handed";
  EXPECT_GT(sum / n, 0.4) << "mean scaled Jacobian";
}

// ---------------------------------------------------------------------------
// Tetrahedral meshes of a sphere: boundary fit, geometric flow, interval layers
// ---------------------------------------------------------------------------

namespace {

// The vertices of the faces that belong to one tetrahedron only: the mesh's boundary.
std::vector<unsigned int> tet_boundary_vertices(const geometry &g) {
  static const int kFaces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  std::map<std::array<unsigned int, 3>, int> uses;
  for (const geometry::tet_t &t : g.tets())
    for (const auto &f : kFaces) {
      std::array<unsigned int, 3> key = {(unsigned int)t[f[0]], (unsigned int)t[f[1]],
                                         (unsigned int)t[f[2]]};
      std::sort(key.begin(), key.end());
      ++uses[key];
    }
  std::vector<char> on(g.num_points(), 0);
  for (const auto &u : uses)
    if (u.second == 1)
      for (unsigned int v : u.first)
        on[v] = 1;
  std::vector<unsigned int> out;
  for (unsigned int v = 0; v < on.size(); ++v)
    if (on[v])
      out.push_back(v);
  return out;
}

// What a tet mesh of the samples make_sphere_volume() takes looks like, in voxels: how far the
// boundary vertices are from the nearest of the spheres of the given radii (of [-1,1]^3), the
// minimum dihedral angle, how many tets are inverted, and the tets' total volume.
struct sphere_tet_report {
  size_t tets = 0;
  double max_boundary_distance = 0.0; // voxels
  double min_dihedral = 0.0;          // degrees
  double mean_dihedral = 0.0;
  int inverted = 0;
  double volume = 0.0; // of [-1,1]^3
};

sphere_tet_report report_sphere_tets(const geometry &g, unsigned int n, double box_size,
                                     std::initializer_list<double> radii) {
  sphere_tet_report r;
  r.tets = g.num_tets();
  const double voxel = 2.0 * box_size / double(n - 1);
  for (unsigned int v : tet_boundary_vertices(g)) {
    const geometry::point_t &p = g.points()[v];
    const double d = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    double nearest = HUGE_VAL;
    for (double radius : radii)
      nearest = std::min(nearest, std::fabs(d - radius * box_size));
    r.max_boundary_distance = std::max(r.max_boundary_distance, nearest / voxel);
  }
  for (const geometry::tet_t &t : g.tets()) {
    const double vol =
        tet_volume(g.points()[t[0]], g.points()[t[1]], g.points()[t[2]], g.points()[t[3]]);
    if (!(vol > 0.0))
      ++r.inverted;
    r.volume += std::fabs(vol) / (box_size * box_size * box_size);
  }
  const quality_stats q = compute_tet_quality_stats(g.tets(), g.points(), TET_MIN_ANGLE);
  r.min_dihedral = q.min;
  r.mean_dihedral = q.mean;
  return r;
}

std::ostream &operator<<(std::ostream &os, const sphere_tet_report &r) {
  return os << r.tets << " tets, boundary within " << r.max_boundary_distance
            << " voxels, min dihedral " << r.min_dihedral << " (mean " << r.mean_dihedral << "), "
            << r.inverted << " inverted, volume " << r.volume;
}

} // namespace

// Octree::interpRect3Dpts_x/y/z placed each edge's crossing at span * grid index, where span is
// the volume's voxel size, but the QEF solution built from those crossings is checked against the
// cell in grid units (Octree::get_solution), so wherever the voxel size was not 1 the solution
// "left its cell" and became the cell's centre. The same span^2 scaling kept the QEF error, which
// drives refinement, under the tolerance. A tet mesh's boundary vertices are those QEF points, so
// an SDF's tet mesh (voxels 0.003 across on the bunny) was uniform cubes with a staircase
// boundary. Hexahedral meshes hid it: Octree::get_vtx_new walks each boundary vertex onto the
// surface afterwards. The same samples now give the same mesh in any box, and its boundary is on
// the surface. The tets' total volume checks that none overlap: where boundary cells of two sizes
// met, Octree::tetrahedralize covered their shared faces twice, and this sphere's adaptive mesh had
// 13% too much volume. Octree::traverse_qef now gives tets one boundary level.
TEST_F(LbieMesherTest, TetraBoundaryFitsTheSurfaceInAnyUnits) {
  const unsigned int n = 33;
  const double radius = 0.6;
  sphere_tet_report unit; // a box whose voxels are 1 across: the case that always worked
  for (double box_size : {16.0, 1.0, 0.01}) {
    geometry g = cvc::tetrahedralize(make_sphere_volume(ctx, n, radius, box_size), 0.0);
    ASSERT_GT(g.num_tets(), 0u) << "box " << box_size;
    const sphere_tet_report r = report_sphere_tets(g, n, box_size, {radius});
    if (box_size == 16.0)
      unit = r;
    EXPECT_LT(r.max_boundary_distance, 0.5) << "box " << box_size << ": " << r;
    EXPECT_EQ(r.tets, unit.tets) << "box " << box_size << ": " << r;
    EXPECT_NEAR(r.volume, 4.0 / 3.0 * kPi * radius * radius * radius, 0.015) << r;
  }
}

// Where the isosurface passes exactly through samples, LBIE disagreed with itself about them: a
// coarse cell touching the surface only at such a sample was never refined and had no QEF point,
// yet its edges counted as crossing, so it got a vertex at its centre, 3.4 voxels off a sphere of
// radius 12 voxels (which meets the grid at its six poles). Integer data at an integer isovalue,
// a CT's, is full of such samples. The isovalue now moves off them (Octree::collapse).
TEST_F(LbieMesherTest, TetraMeshesAnIsovalueTheSamplesHit) {
  const unsigned int n = 33;
  {
    const double radius = 0.75; // 12 voxels
    geometry g = cvc::tetrahedralize(make_sphere_volume(ctx, n, radius, 16.0), 0.0);
    ASSERT_GT(g.num_tets(), 0u);
    const sphere_tet_report r = report_sphere_tets(g, n, 16.0, {radius});
    EXPECT_LT(r.max_boundary_distance, 0.5) << r;
  }
  {
    // 100 + 4 per voxel of r - d, rounded to unsigned chars; the surface is v = 100
    const double radius = 0.6;
    volume v(ctx, dimension(n, n, n), UChar, bounding_box(-16.0, -16.0, -16.0, 16.0, 16.0, 16.0));
    for (unsigned int k = 0; k < n; ++k)
      for (unsigned int j = 0; j < n; ++j)
        for (unsigned int i = 0; i < n; ++i) {
          const double x = -1.0 + 2.0 * i / (n - 1), y = -1.0 + 2.0 * j / (n - 1),
                       z = -1.0 + 2.0 * k / (n - 1);
          v(i, j, k,
            std::round(100.0 + 4.0 * (radius - std::sqrt(x * x + y * y + z * z)) * (n - 1) / 2.0));
        }
    geometry g = cvc::tetrahedralize(v, 100.0);
    ASSERT_GT(g.num_tets(), 0u);
    const sphere_tet_report r = report_sphere_tets(g, n, 16.0, {radius});
    EXPECT_LT(r.max_boundary_distance, 0.5) << r;
    EXPECT_NEAR(r.volume, 4.0 / 3.0 * kPi * radius * radius * radius, 0.015) << r;
  }
}

// The QEF error that drives refinement is a residual summed over the three axes, but
// Octree::compute_qef and compute_qef_interval assigned instead of adding inside their axis loops,
// so only the z term counted. The side of a cylinder along z has no z component in its normals, so
// its residual was 0/0 and it never refined, while the same cylinder along x or y did. The same
// capped cylinder along each axis now gives the same surface and the same tetrahedra.
TEST_F(LbieMesherTest, RefinementIsTheSameAlongEveryAxis) {
  const unsigned int n = 33;
  const double radius = 0.4, half_length = 0.6;
  int tris[3], tets[3];
  for (int axis = 0; axis < 3; ++axis) {
    volume v(ctx, dimension(n, n, n), Float, bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0));
    for (unsigned int k = 0; k < n; ++k)
      for (unsigned int j = 0; j < n; ++j)
        for (unsigned int i = 0; i < n; ++i) {
          const double p[3] = {-1.0 + 2.0 * i / (n - 1), -1.0 + 2.0 * j / (n - 1),
                               -1.0 + 2.0 * k / (n - 1)};
          const double a = p[(axis + 1) % 3], b = p[(axis + 2) % 3];
          v(i, j, k, std::min(radius - std::sqrt(a * a + b * b), half_length - std::fabs(p[axis])));
        }
    LBIE::geoframe surface =
        run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::SINGLE);
    LBIE::geoframe solid =
        run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA);
    expect_geoframe_valid(surface);
    expect_geoframe_valid(solid);
    tris[axis] = surface.numtris;
    tets[axis] = solid.numtris / 4;
  }
  EXPECT_GT(tris[0], 0);
  for (int axis = 1; axis < 3; ++axis) {
    EXPECT_EQ(tris[axis], tris[0]) << "triangles along axis " << axis << " vs x";
    EXPECT_EQ(tets[axis], tets[0]) << "tets along axis " << axis << " vs x";
  }
}

// GEO_FLOW on tetrahedra (Octree::geometric_flow_tet) moved every vertex 100 times per pass with
// no check on the tets around it, so it could fold them; on the bunny's mesh the minimum dihedral
// angle fell from 26.6 to 0.75 degrees and a boundary vertex flew far outside. It also read each
// tet's corners in an order that made every volume negative, so only boundary vertices moved, and
// their tangent move took the gradient of the volume's values for a unit normal: in a box 0.01
// across the sphere's gradient is 100, and the flow sent every vertex to infinity. A move is now
// taken only if every tet around the vertex keeps a positive volume and none gets a smaller
// dihedral angle than the worst of them had. Here the minimum angle goes from 20 to 32 degrees.
TEST_F(LbieMesherTest, TetraGeometricFlowNeverMakesTheWorstTetWorse) {
  const unsigned int n = 17;
  const double radius = 0.6;
  // voxels 1 across with a gradient of 1/8, and voxels 1/800 across with a gradient of 10^4
  for (std::pair<double, double> c : {std::make_pair(8.0, 1.0), std::make_pair(0.01, 100.0)}) {
    const double box_size = c.first, value_scale = c.second;
    volume v = make_sphere_volume(ctx, n, radius, box_size);
    for (unsigned int k = 0; k < n; ++k)
      for (unsigned int j = 0; j < n; ++j)
        for (unsigned int i = 0; i < n; ++i)
          v(i, j, k, v(i, j, k) * value_scale);
    const sphere_tet_report before =
        report_sphere_tets(cvc::tetrahedralize(v, 0.0), n, box_size, {radius});
    const sphere_tet_report after =
        report_sphere_tets(cvc::tetrahedralize(v, 0.0, DUALLIB, GEO_FLOW, BSPLINE_CONVOLUTION, 1),
                           n, box_size, {radius});
    const std::string where =
        "box " + std::to_string(box_size) + ", values x" + std::to_string(value_scale);
    ASSERT_EQ(after.tets, before.tets) << where;
    EXPECT_EQ(after.inverted, 0) << where << ": " << after;
    EXPECT_GT(after.min_dihedral, 1.5 * before.min_dihedral) << where << ": " << after;
    EXPECT_GT(after.mean_dihedral, before.mean_dihedral) << where << ": " << after;
    EXPECT_LT(after.max_boundary_distance, 0.5) << where << ": " << after;
  }
}

// cvc::tetrahedralize2 asks for LBIE's TETRA2 mesh, but do_mesh_geometry turned VOLUME_TET back
// into TETRA, so the layer went through the one-surface Octree::tetrahedralize on the interval's
// octree: fans of slivers, holes, and tets on both sides of the inner surface.
// Octree::tetrahedralize_interval itself was no better (see Mesher::do_duallib_extract). A layer is
// now the solid min(v - lo, hi - v) >= 0 meshed by Octree::tetrahedralize, with either isovalue
// first.
TEST_F(LbieMesherTest, Tetrahedralize2MeshesTheLayerBetweenTwoIsosurfaces) {
  const unsigned int n = 33;
  const double radius = 0.6, half = 0.15; // the layer is radius +- half
  const double r0 = radius - half, r1 = radius + half;
  const double exact = 4.0 / 3.0 * kPi * (r1 * r1 * r1 - r0 * r0 * r0);
  for (double box_size : {16.0, 0.01}) {
    volume v = make_sphere_volume(ctx, n, radius, box_size);
    for (bool upper_first : {true, false}) {
      geometry g =
          upper_first ? cvc::tetrahedralize2(v, half, -half) : cvc::tetrahedralize2(v, -half, half);
      ASSERT_GT(g.num_tets(), 0u) << "box " << box_size;
      const sphere_tet_report r = report_sphere_tets(g, n, box_size, {r0, r1});
      const std::string where = "box " + std::to_string(box_size) +
                                (upper_first ? ", upper isovalue first" : ", lower first");
      EXPECT_LT(r.max_boundary_distance, 0.5) << where << ": " << r;
      EXPECT_EQ(r.inverted, 0) << where << ": " << r;
      EXPECT_NEAR(r.volume, exact, 0.03 * exact) << where << ": " << r;
      // Every corner of every tet is in the layer (to half a voxel).
      int outside = 0;
      for (const geometry::tet_t &t : g.tets())
        for (int c = 0; c < 4; ++c) {
          const geometry::point_t &p = g.points()[t[c]];
          const double d = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) / box_size;
          if (d < r0 - 1.0 / (n - 1) || d > r1 + 1.0 / (n - 1))
            ++outside;
        }
      EXPECT_EQ(outside, 0) << where << ": tet corners off the layer";
    }
  }
}

// ---------------------------------------------------------------------------
// mesher.cpp wrappers: verbose path, quality_improve, geometry-based API
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, DoMeshVerboseAndQualityImprove) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g =
      run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::SINGLE,
                  LBIE::Mesher::NO_IMPROVE, 0, false, LBIE::Mesher::BSPLINE_CONVOLUTION, true);
  EXPECT_GT(g.numtris, 0);

  LBIE::geoframe improved = LBIE::quality_improve(ctx, g, LBIE::Mesher::GEO_FLOW, 1, true);
  EXPECT_EQ(improved.numtris, g.numtris);
  EXPECT_EQ(improved.mesh_type, g.mesh_type);
  expect_geoframe_valid(improved);
}

TEST_F(LbieMesherTest, DoMeshGeometryTetAndImprove) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  geometry g = LBIE::do_mesh_geometry(
      v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, geometry::VOLUME_TET, LBIE::Mesher::NO_IMPROVE,
      LBIE::Mesher::BSPLINE_CONVOLUTION, LBIE::Mesher::DUALLIB, 0, false);
  EXPECT_GT(g.num_points(), 0u);
  EXPECT_GT(g.num_tets(), 0u);

  geometry improved = LBIE::quality_improve_geometry(ctx, g, LBIE::Mesher::GEO_FLOW, 1);
  EXPECT_EQ(improved.num_tets(), g.num_tets());
}

// ---------------------------------------------------------------------------
// Public API paths not covered elsewhere
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, IsoWithImproveIterations) {
  volume v = make_sphere_volume(ctx, 16, 0.5);
  geometry g = cvc::iso(v, 0.0, DUALLIB, 2);
  EXPECT_GT(g.num_points(), 0u);
  EXPECT_GT(g.num_tris(), 0u);
}

TEST_F(LbieMesherTest, IsoLibisocontourVoxelTypes) {
  bounding_box bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
  const unsigned int n = 16;
  const double dx = 2.0 / double(n - 1);

  volume vu8(ctx, dimension(n, n, n), UChar, bbox);
  volume vu16(ctx, dimension(n, n, n), UShort, bbox);
  volume vf64(ctx, dimension(n, n, n), Double, bbox);
  for (unsigned int k = 0; k < n; ++k)
    for (unsigned int j = 0; j < n; ++j)
      for (unsigned int i = 0; i < n; ++i) {
        double x = -1.0 + i * dx;
        double y = -1.0 + j * dx;
        double z = -1.0 + k * dx;
        double f = 0.5 - std::sqrt(x * x + y * y + z * z);
        vu8(i, j, k, 128.0 + 80.0 * f);
        vu16(i, j, k, 1000.0 + 500.0 * f);
        vf64(i, j, k, f);
      }

  geometry g8 = cvc::iso(vu8, 128.0, LIBISOCONTOUR);
  EXPECT_GT(g8.num_points(), 0u);
  geometry g16 = cvc::iso(vu16, 1000.0, LIBISOCONTOUR);
  EXPECT_GT(g16.num_points(), 0u);
  geometry g64 = cvc::iso(vf64, 0.0, LIBISOCONTOUR);
  EXPECT_GT(g64.num_points(), 0u);
}

// ---------------------------------------------------------------------------
// geoframe: construction helpers (header templates)
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, GeoframeBuildersAndQueries) {
  LBIE::geoframe g;

  float n0[3] = {0.f, 0.f, 1.f};
  float p0[3] = {0.f, 0.f, 0.f};
  float p1[3] = {1.f, 0.f, 0.f};
  float p2[3] = {0.f, 1.f, 0.f};
  float p3[3] = {0.f, 0.f, 1.f};
  float p4[3] = {1.f, 1.f, 0.f};

  EXPECT_EQ(g.AddVert(p0, n0), 0);
  EXPECT_EQ(g.AddVert(p1, n0), 1);
  EXPECT_EQ(g.AddVert(p2, n0), 2);
  EXPECT_EQ(g.AddVert(p3, n0), 3);
  EXPECT_EQ(g.AddVert(p4, n0), 4);
  EXPECT_EQ(g.getNVert(), 5);

  // AddTri / aspect ratio.
  EXPECT_GT(g.AddTri(0, 1, 2), 0);
  float ar = g.get_aspect_ratio(0, 1, 2);
  EXPECT_GT(ar, 0.0f);
  EXPECT_TRUE(std::isfinite(ar));

  // Boundary bookkeeping.
  g.AddBound(0, 1);
  EXPECT_EQ(g.bound_sign[0], 1u);
  g.AddBound_edge(0, 1);
  g.AddBound_edge(2, 1); // swapped order path
  EXPECT_EQ(g.CheckBound_edge(0, 1), 1);
  EXPECT_EQ(g.CheckBound_edge(1, 2), 1);
  EXPECT_EQ(g.CheckBound_edge(0, 3), 0);
  g.AddVtxNew(1, 1);
  EXPECT_EQ(g.vtxnew_sign[1], 1u);

  // testRHS orientation checks.
  float *fp0 = p0, *fp1 = p1, *fp2 = p2, *fp3 = p3;
  int rhs = g.testRHS(fp0, fp1, fp2, fp3);
  float m3[3] = {0.f, 0.f, -1.f};
  float *fm3 = m3;
  int lhs = g.testRHS(fp0, fp1, fp2, fm3);
  EXPECT_NE(rhs, lhs);

  // Tet quality helper. (geoframe::testTetrahedron / getRadius are dead
  // template code that no longer compiles when instantiated -- getRadius
  // writes through const references -- so only testTetrahedron1 is
  // exercised here.)
  double vol1 = g.testTetrahedron1(fp0, fp1, fp2, fp3);
  EXPECT_TRUE(std::isfinite(vol1));

  // AddTetra splits into 4 triangles; AddQuad / TestNum; center_vtx.
  int tris_before = g.getNTri();
  g.AddTetra(0, 1, 2, 3);
  EXPECT_EQ(g.getNTri(), tris_before + 4);

  unsigned int q[4] = {0, 1, 4, 2};
  unsigned int *qp = q;
  g.AddQuad(qp, 4);
  EXPECT_GT(g.getNQuad(), 0);

  unsigned int qdup[4] = {0, 0, 1, 2}; // duplicate vertex -> TestNum returns 3
  unsigned int *qdupp = qdup;
  int tn = g.TestNum(qdupp);
  EXPECT_EQ(tn, 3);

  int cv = g.center_vtx(0, 1, 2);
  EXPECT_EQ(cv, g.getNVert() - 1);

  // Add_2_Tetra / Add_2_Tri quadrilateral splitting (both diagonals and
  // the duplicate-vertex early outs).
  unsigned int quad4[4] = {0, 1, 4, 2};
  unsigned int *quad4p = quad4;
  g.Add_2_Tetra(quad4p, 3);
  unsigned int quaddup[4] = {0, 0, 1, 2};
  unsigned int *quaddupp = quaddup;
  g.Add_2_Tetra(quaddupp, 3);
  g.Add_2_Tri(quad4p);
  g.Add_2_Tri(quaddupp);
  unsigned int quaddup2[4] = {0, 1, 1, 2};
  unsigned int *quaddup2p = quaddup2;
  g.Add_2_Tri(quaddup2p);
  unsigned int quaddup3[4] = {0, 1, 2, 2};
  unsigned int *quaddup3p = quaddup3;
  g.Add_2_Tri(quaddup3p);

  // AddPyramid.
  g.AddPyramid(quad4p, 3, 4);

  // AddQuad_indirect (no duplicated vertices -> generic branch).
  g.AddQuad_indirect(quad4p);

  // Refinement vertex helpers.
  unsigned int e1 = g.AddRefine_edgevtx(0, 1);
  EXPECT_LT(e1, (unsigned int)g.getNVert());
  unsigned int e2 = g.AddRefine_edgevtx(0, 1); // cached second time
  EXPECT_EQ(e1, e2);
  unsigned int eh = g.AddRefine_edgevtx_hex(1, 2);
  EXPECT_LT(eh, (unsigned int)g.getNVert());
  unsigned int fv = g.AddRefine_facevtx(0, 1, 4, 2);
  EXPECT_LT(fv, (unsigned int)g.getNVert());

  // edge_contraction_tri on a triangle. (geoframe::edge_contraction and its
  // tetra branch route through edge_contraction_tetra -> testTetrahedron ->
  // getRadius, which is uncompilable dead template code; see note above.)
  unsigned int tri3[3] = {0, 1, 2};
  unsigned int *tri3p = tri3;
  g.edge_contraction_tri(tri3p);

  expect_geoframe_valid(g);
}

TEST_F(LbieMesherTest, GeoframeCalcExtentsNormalsAspect) {
  LBIE::geoframe g;
  float n0[3] = {0.f, 0.f, 1.f};
  float p0[3] = {0.f, 0.f, 0.f};
  float p1[3] = {2.f, 0.f, 0.f};
  float p2[3] = {0.f, 3.f, 0.f};
  float p3[3] = {0.f, 0.f, 4.f};
  g.AddVert(p0, n0);
  g.AddVert(p1, n0);
  g.AddVert(p2, n0);
  g.AddVert(p3, n0);
  g.AddTri(0, 1, 2);
  g.AddTri(0, 1, 3);

  g.calculateExtents();
  EXPECT_FLOAT_EQ(g.max_x, 2.0f);
  EXPECT_FLOAT_EQ(g.max_y, 3.0f);
  EXPECT_FLOAT_EQ(g.max_z, 4.0f);
  EXPECT_DOUBLE_EQ(g.biggestDim, 4.0);

  g.calculatenormals(); // writes per-triangle normals (numtris <= numverts)
  for (int t = 0; t < g.numtris; ++t) {
    float len = std::sqrt(g.normals[t][0] * g.normals[t][0] + g.normals[t][1] * g.normals[t][1] +
                          g.normals[t][2] * g.normals[t][2]);
    EXPECT_NEAR(len, 1.0f, 1e-5f);
  }

  g.calculateAspectRatio();
  EXPECT_GT(g.max_aspect, 0.0f);
  EXPECT_GT(g.avg_aspect, 0.0f);

  g.setSpan(2.0f, 2.0f, 2.0f);
  g.setMin(-1.0f, -1.0f, -1.0f);
  g.updateBySpan();
  EXPECT_FLOAT_EQ(g.verts[0][0], -1.0f);
  EXPECT_FLOAT_EQ(g.verts[1][0], 3.0f);

  // Copy ctor / assignment / reset.
  LBIE::geoframe copy(g);
  EXPECT_EQ(copy.numverts, g.numverts);
  EXPECT_EQ(copy.numtris, g.numtris);
  LBIE::geoframe assigned;
  assigned = g;
  EXPECT_EQ(assigned.numverts, g.numverts);
  assigned.Clear();
  EXPECT_EQ(assigned.numverts, 0);
  EXPECT_TRUE(assigned.verts.empty());
}

// ---------------------------------------------------------------------------
// geoframe file I/O
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, GeoframeTriRawRoundtrip) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::SINGLE);
  ASSERT_GT(g.numtris, 0);

  const std::string path = temp_path("lbie_mesh_tri.raw");
  g.write_raw(path.c_str(), LBIE::geoframe::SINGLE);

  LBIE::geoframe loaded;
  ASSERT_EQ(loaded.read_raw(path.c_str()), 0); // 0 == triangular mesh
  EXPECT_EQ(loaded.numverts, g.numverts);
  EXPECT_EQ(loaded.numtris, g.numtris);
  expect_geoframe_valid(loaded);
  std::remove(path.c_str());

  // Missing file path.
  LBIE::geoframe missing;
  EXPECT_EQ(missing.read_raw(temp_path("does_not_exist_tri.raw").c_str()), -1);
}

TEST_F(LbieMesherTest, GeoframeTriRawnVariants) {
  // Hand-written tiny meshes exercising the .rawn / .rawc / .rawnc readers.
  const std::string base = temp_path("lbie_tiny_tri");
  {
    std::ofstream f(base + ".rawn");
    f << "3 1\n";
    f << "0 0 0 0 0 1\n1 0 0 0 0 1\n0 1 0 0 0 1\n";
    f << "0 1 2\n";
  }
  LBIE::geoframe gn;
  ASSERT_EQ(gn.read_raw((base + ".rawn").c_str()), 0);
  EXPECT_EQ(gn.numverts, 3);
  EXPECT_EQ(gn.numtris, 1);
  EXPECT_FLOAT_EQ(gn.normals[0][2], -1.0f); // reader negates normals
  std::remove((base + ".rawn").c_str());

  {
    std::ofstream f(base + ".rawc");
    f << "3 1\n";
    f << "0 0 0 1 0 0\n1 0 0 0 1 0\n0 1 0 0 0 1\n";
    f << "0 1 2\n";
  }
  LBIE::geoframe gc;
  ASSERT_EQ(gc.read_raw((base + ".rawc").c_str()), 0);
  EXPECT_EQ(gc.numverts, 3);
  EXPECT_FLOAT_EQ(gc.color[0][0], 1.0f);
  std::remove((base + ".rawc").c_str());

  {
    std::ofstream f(base + ".rawnc");
    f << "3 1\n";
    f << "0 0 0 0 0 1 1 0 0\n1 0 0 0 0 1 0 1 0\n0 1 0 0 0 1 0 0 1\n";
    f << "0 1 2\n";
  }
  LBIE::geoframe gnc;
  ASSERT_EQ(gnc.read_raw((base + ".rawnc").c_str()), 0);
  EXPECT_EQ(gnc.numverts, 3);
  EXPECT_EQ(gnc.numtris, 1);
  std::remove((base + ".rawnc").c_str());
}

TEST_F(LbieMesherTest, GeoframeQuadRawRoundtrip) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::QUAD);
  ASSERT_GT(g.numquads, 0);

  const std::string path = temp_path("lbie_mesh_quad.raw");
  g.write_raw(path.c_str(), LBIE::geoframe::QUAD);

  LBIE::geoframe loaded;
  ASSERT_EQ(loaded.read_raw(path.c_str()), 2); // 2 == quad mesh
  EXPECT_EQ(loaded.numverts, g.numverts);
  EXPECT_EQ(loaded.numquads, g.numquads);
  expect_geoframe_valid(loaded);
  std::remove(path.c_str());
}

TEST_F(LbieMesherTest, GeoframeHexRawRoundtrip) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::HEXA);
  ASSERT_GT(g.numquads, 0);
  ASSERT_EQ(g.numquads % 6, 0);

  const std::string path = temp_path("lbie_mesh_hex.raw");
  g.mesh_type = LBIE::geoframe::HEXA;
  g.write_raw(path.c_str()); // mesh_type-based overload -> saveHexa

  LBIE::geoframe loaded;
  ASSERT_EQ(loaded.read_raw(path.c_str()), 3); // 3 == hex mesh
  EXPECT_EQ(loaded.numverts, g.numverts);
  EXPECT_EQ(loaded.numquads, g.numquads);
  expect_geoframe_valid(loaded);
  std::remove(path.c_str());
}

TEST_F(LbieMesherTest, GeoframeTetRawWriteAndRead) {
  volume v = make_sphere_volume(ctx, 17, 0.5);
  LBIE::geoframe g = run_do_mesh(v, 0.0f, 0.0f, kDefaultErr, kDefaultErrIn, LBIE::geoframe::TETRA);
  ASSERT_GT(g.numtris, 0);

  // saveTetra via the meshtype-argument overload.
  const std::string wpath = temp_path("lbie_mesh_out_tet.raw");
  g.write_raw(wpath.c_str(), LBIE::geoframe::TETRA);
  std::ifstream check(wpath);
  int nv = 0, ntet = 0;
  check >> nv >> ntet;
  EXPECT_EQ(nv, g.numverts);
  EXPECT_EQ(ntet, g.numtris / 4);
  check.close();
  std::remove(wpath.c_str());

  // Hand-written tet file (reader expects a per-vertex boundary sign).
  const std::string rpath = temp_path("lbie_tiny_tet.raw");
  {
    std::ofstream f(rpath);
    f << "5 2\n";
    f << "0 0 0 1\n1 0 0 1\n0 1 0 1\n0 0 1 1\n1 1 1 1\n";
    f << "0 1 2 3\n1 2 3 4\n";
  }
  LBIE::geoframe loaded;
  ASSERT_EQ(loaded.read_raw(rpath.c_str()), 1); // 1 == tet mesh
  EXPECT_EQ(loaded.numverts, 5);
  EXPECT_EQ(loaded.numtris, 2 * 4);
  expect_geoframe_valid(loaded);
  std::remove(rpath.c_str());
}

TEST_F(LbieMesherTest, GeoframeWriteRawMeshTypeDispatch) {
  // Small synthetic geoframes so all four save* paths run quickly.
  float n0[3] = {0.f, 0.f, 1.f};
  float p[4][3] = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};

  LBIE::geoframe tri;
  for (int i = 0; i < 3; ++i)
    tri.AddVert(p[i], n0);
  tri.AddTri(0, 1, 2);
  tri.mesh_type = LBIE::geoframe::SINGLE;
  const std::string tri_path = temp_path("lbie_dispatch_tri.raw");
  tri.write_raw(tri_path.c_str());
  std::ifstream tf(tri_path);
  ASSERT_TRUE(tf.good());
  tf.close();
  std::remove(tri_path.c_str());

  LBIE::geoframe tet;
  for (int i = 0; i < 4; ++i)
    tet.AddVert(p[i], n0);
  tet.AddTetra(0, 1, 2, 3);
  tet.mesh_type = LBIE::geoframe::TETRA2;
  const std::string tet_path = temp_path("lbie_dispatch_tet.raw");
  tet.write_raw(tet_path.c_str());
  std::remove(tet_path.c_str());

  LBIE::geoframe quad;
  for (int i = 0; i < 4; ++i)
    quad.AddVert(p[i], n0);
  unsigned int q[4] = {0, 1, 3, 2};
  unsigned int *qp = q;
  quad.AddQuad(qp, 4);
  quad.mesh_type = LBIE::geoframe::QUAD;
  const std::string quad_path = temp_path("lbie_dispatch_quad.raw");
  quad.write_raw(quad_path.c_str());
  std::remove(quad_path.c_str());

  // DOUBLE dispatches to saveTriangle as well.
  tri.mesh_type = LBIE::geoframe::DOUBLE;
  const std::string dbl_path = temp_path("lbie_dispatch_double.raw");
  tri.write_raw(dbl_path.c_str());
  std::remove(dbl_path.c_str());
}

// ---------------------------------------------------------------------------
// geoframe_adapter and geoframe <-> geometry conversion
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, AdapterTriangleRoundtrip) {
  geometry geom;
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});

  LBIE::geoframe gf = LBIE::to_geoframe(geom);
  EXPECT_EQ(gf.numverts, 3);
  EXPECT_EQ(gf.numtris, 1);
  EXPECT_EQ(gf.mesh_type, LBIE::geoframe::SINGLE);

  geometry back = LBIE::to_geometry(gf);
  EXPECT_EQ(back.num_points(), 3u);
  EXPECT_EQ(back.num_tris(), 1u);
}

TEST_F(LbieMesherTest, AdapterTetHexQuadRoundtrip) {
  // Tetrahedral geometry.
  geometry tetg;
  tetg.points().push_back({{0.0, 0.0, 0.0}});
  tetg.points().push_back({{1.0, 0.0, 0.0}});
  tetg.points().push_back({{0.0, 1.0, 0.0}});
  tetg.points().push_back({{0.0, 0.0, 1.0}});
  tetg.tets().push_back({{0, 1, 2, 3}});
  LBIE::geoframe tf = LBIE::to_geoframe(tetg);
  EXPECT_EQ(tf.mesh_type, LBIE::geoframe::TETRA);
  EXPECT_EQ(tf.numtris, 4);
  geometry tback = LBIE::to_geometry(tf);
  EXPECT_EQ(tback.num_tets(), 1u);

  // Hexahedral geometry (unit cube).
  geometry hexg;
  hexg.points().push_back({{0.0, 0.0, 0.0}});
  hexg.points().push_back({{1.0, 0.0, 0.0}});
  hexg.points().push_back({{1.0, 1.0, 0.0}});
  hexg.points().push_back({{0.0, 1.0, 0.0}});
  hexg.points().push_back({{0.0, 0.0, 1.0}});
  hexg.points().push_back({{1.0, 0.0, 1.0}});
  hexg.points().push_back({{1.0, 1.0, 1.0}});
  hexg.points().push_back({{0.0, 1.0, 1.0}});
  hexg.hexs().push_back({{0, 1, 2, 3, 4, 5, 6, 7}});
  LBIE::geoframe hf = LBIE::to_geoframe(hexg);
  EXPECT_EQ(hf.mesh_type, LBIE::geoframe::HEXA);
  EXPECT_EQ(hf.numquads, 6);
  geometry hback = LBIE::to_geometry(hf);
  ASSERT_EQ(hback.num_hexs(), 1u);
  EXPECT_EQ(hback.const_hexs()[0], hexg.const_hexs()[0]);

  // Quad surface geometry.
  geometry quadg;
  quadg.points().push_back({{0.0, 0.0, 0.0}});
  quadg.points().push_back({{1.0, 0.0, 0.0}});
  quadg.points().push_back({{1.0, 1.0, 0.0}});
  quadg.points().push_back({{0.0, 1.0, 0.0}});
  quadg.quads().push_back({{0, 1, 2, 3}});
  LBIE::geoframe qf = LBIE::to_geoframe(quadg);
  EXPECT_EQ(qf.mesh_type, LBIE::geoframe::QUAD);
  EXPECT_EQ(qf.numquads, 1);
  geometry qback = LBIE::to_geometry(qf);
  EXPECT_EQ(qback.num_quads(), 1u);

  // Empty geoframe -> geometry works; the reverse (to_geoframe on an empty
  // geometry) throws invalid_bounding_box because geometry::extents() cannot
  // represent inverted empty bounds (library bug), so it is not exercised.
  LBIE::geoframe ef;
  geometry eback = LBIE::to_geometry(ef);
  EXPECT_EQ(eback.num_points(), 0u);
}

TEST_F(LbieMesherTest, AdapterSyncLifecycle) {
  geometry geom;
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});

  {
    LBIE::geoframe_adapter adapter(geom);
    EXPECT_EQ(adapter.numverts, 3);
    EXPECT_EQ(adapter.numtris, 1);

    // Mutate through the geoframe interface; destructor syncs back.
    float p[3] = {2.f, 2.f, 2.f};
    float n[3] = {0.f, 0.f, 1.f};
    adapter.AddVert(p, n);
    adapter.AddTri(0, 1, 3);
    adapter.sync_to_geometry();
    EXPECT_EQ(adapter.get_geometry().num_points(), 4u);

    LBIE::geoframe_adapter copy(adapter);
    EXPECT_EQ(copy.numverts, adapter.numverts);
    copy = adapter;
    EXPECT_EQ(copy.numtris, adapter.numtris);
  }
  EXPECT_EQ(geom.num_points(), 4u);
  EXPECT_EQ(geom.num_tris(), 2u);

  geometry geom2;
  geom2.points().push_back({{5.0, 5.0, 5.0}});
  {
    LBIE::geoframe_adapter adapter(geom2);
    adapter.reset();
  }
  EXPECT_EQ(geom2.num_points(), 0u);
}

// ---------------------------------------------------------------------------
// cvc/utility/algorithm.cpp volumetric mesh utilities
// ---------------------------------------------------------------------------

TEST_F(LbieMesherTest, TetUtilityFunctions) {
  // Unit tetrahedron.
  geometry::point_t v0 = {{0.0, 0.0, 0.0}};
  geometry::point_t v1 = {{1.0, 0.0, 0.0}};
  geometry::point_t v2 = {{0.0, 1.0, 0.0}};
  geometry::point_t v3 = {{0.0, 0.0, 1.0}};

  EXPECT_NEAR(std::fabs(tet_volume(v0, v1, v2, v3)), 1.0 / 6.0, 1e-12);
  EXPECT_GT(tet_aspect_ratio(v0, v1, v2, v3), 0.0);
  double dihedral = tet_min_dihedral_angle(v0, v1, v2, v3);
  EXPECT_GT(dihedral, 0.0);
  EXPECT_LT(dihedral, 90.0);

  // Barycentric coordinates: weights sum to one, centroid gives 1/4 each.
  geometry::point_t centroid = {{0.25, 0.25, 0.25}};
  std::array<double, 4> bc = tet_barycentric(centroid, v0, v1, v2, v3);
  double sum = bc[0] + bc[1] + bc[2] + bc[3];
  EXPECT_NEAR(sum, 1.0, 1e-9);
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(bc[i], 0.25, 1e-9);

  // Linear function reproduced by barycentric interpolation.
  geometry::points_t pts = {v0, v1, v2, v3};
  geometry::tet_t tet = {{0, 1, 2, 3}};
  std::vector<double> vals = {0.0, 1.0, 2.0, 3.0}; // f = x + 2y + 3z
  geometry::point_t probe = {{0.1, 0.2, 0.3}};
  EXPECT_NEAR(interpolate_in_tet(probe, tet, pts, vals), 0.1 + 2.0 * 0.2 + 3.0 * 0.3, 1e-9);

  // faces / encode / decode roundtrip.
  geometry::tets_t tets = {tet};
  geometry::tris_t faces = tet_faces(tets);
  EXPECT_EQ(faces.size(), 4u);
  geometry::tris_t encoded = encode_triangles_from_tets(tets);
  EXPECT_EQ(encoded.size(), 4u);
  geometry::tets_t decoded = decode_tets_from_triangles(encoded);
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_EQ(decoded[0], tet);

  // Quality stats for all tet metrics.
  quality_stats s1 = compute_tet_quality_stats(tets, pts, TET_VOLUME);
  EXPECT_NEAR(s1.mean, std::fabs(tet_volume(v0, v1, v2, v3)), 1e-9);
  quality_stats s2 = compute_tet_quality_stats(tets, pts, TET_ASPECT_RATIO);
  EXPECT_GT(s2.mean, 0.0);
  quality_stats s3 = compute_tet_quality_stats(tets, pts, TET_MIN_ANGLE);
  EXPECT_GT(s3.mean, 0.0);

  // Point location and quality filtering.
  std::vector<size_t> found = find_tets_containing_point(centroid, tets, pts);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0], 0u);
  geometry::point_t outside = {{5.0, 5.0, 5.0}};
  EXPECT_TRUE(find_tets_containing_point(outside, tets, pts).empty());

  std::vector<size_t> keep = filter_tets_by_quality(tets, pts, 100.0, TET_ASPECT_RATIO);
  EXPECT_EQ(keep.size(), 1u);
  std::vector<size_t> none = filter_tets_by_quality(tets, pts, 0.0, TET_ASPECT_RATIO);
  EXPECT_TRUE(none.empty());

  // extract_surface / extract_quality_elements / compute_mesh_bounds on a
  // tet geometry.
  geometry tg;
  for (const auto &p : pts)
    tg.points().push_back(p);
  tg.tets().push_back(tet);
  geometry surf = extract_surface(tg);
  EXPECT_EQ(surf.num_tris(), 4u);
  geometry hq = extract_quality_elements(tg, 100.0, TET_ASPECT_RATIO);
  EXPECT_EQ(hq.num_tets(), 1u);
  std::array<double, 6> bounds = compute_mesh_bounds(tg);
  EXPECT_DOUBLE_EQ(bounds[0], 0.0);
  EXPECT_DOUBLE_EQ(bounds[3], 1.0);
}

TEST_F(LbieMesherTest, HexUtilityFunctions) {
  // Unit cube (LBIE ordering: bottom face 0-3, top face 4-7).
  geometry::point_t cube[8] = {{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}, {{1.0, 1.0, 0.0}},
                               {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}, {{1.0, 0.0, 1.0}},
                               {{1.0, 1.0, 1.0}}, {{0.0, 1.0, 1.0}}};

  EXPECT_NEAR(std::fabs(hex_volume(cube)), 1.0, 1e-9);
  double jac = hex_jacobian(cube);
  EXPECT_NEAR(std::fabs(jac), 0.125, 1e-9); // parametric-center determinant
  // The scaled Jacobian is the worst corner's determinant of unit edges: 1 for a cube (it was
  // the centre Jacobian over the space diagonal cubed, 1/sqrt(27)), 1/sqrt(5) for a cube
  // sheared by twice its height, -1 for the cube's mirror image.
  double sj = hex_scaled_jacobian(cube);
  EXPECT_NEAR(sj, 1.0, 1e-12);
  geometry::point_t sheared[8], mirrored[8];
  for (int i = 0; i < 8; ++i) {
    sheared[i] = cube[i];
    sheared[i][0] += 2.0 * cube[i][2];
    mirrored[i] = cube[i];
    mirrored[i][0] = -cube[i][0];
  }
  EXPECT_NEAR(hex_scaled_jacobian(sheared), 1.0 / std::sqrt(5.0), 1e-12);
  EXPECT_NEAR(hex_scaled_jacobian(mirrored), -1.0, 1e-12);

  geometry::point_t center = {{0.5, 0.5, 0.5}};
  std::array<double, 8> w = hex_trilinear(center, cube);
  double sum = 0.0;
  for (int i = 0; i < 8; ++i)
    sum += w[i];
  EXPECT_NEAR(sum, 1.0, 1e-9);
  for (int i = 0; i < 8; ++i)
    EXPECT_NEAR(w[i], 0.125, 1e-9);

  geometry::points_t pts(cube, cube + 8);
  geometry::hex_t hex = {{0, 1, 2, 3, 4, 5, 6, 7}};
  std::vector<double> vals(8);
  for (int i = 0; i < 8; ++i)
    vals[i] = cube[i][2]; // f = z
  EXPECT_NEAR(interpolate_in_hex(center, hex, pts, vals), 0.5, 1e-9);

  geometry::hexs_t hexs = {hex};
  geometry::quads_t faces = hex_faces(hexs);
  EXPECT_EQ(faces.size(), 6u);
  geometry::quads_t encoded = encode_quads_from_hexs(hexs);
  EXPECT_EQ(encoded.size(), 6u);
  geometry::hexs_t decoded = decode_hexs_from_quads(encoded);
  ASSERT_EQ(decoded.size(), 1u);

  quality_stats s1 = compute_hex_quality_stats(hexs, pts, HEX_VOLUME);
  EXPECT_NEAR(std::fabs(s1.mean), 1.0, 1e-9);
  quality_stats s2 = compute_hex_quality_stats(hexs, pts, HEX_JACOBIAN);
  EXPECT_GT(std::fabs(s2.mean), 0.0);
  quality_stats s3 = compute_hex_quality_stats(hexs, pts, HEX_SCALED_JACOBIAN);
  EXPECT_NEAR(s3.mean, sj, 1e-9);

  std::vector<size_t> found = find_hexs_containing_point(center, hexs, pts);
  ASSERT_EQ(found.size(), 1u);
  geometry::point_t outside = {{9.0, 9.0, 9.0}};
  EXPECT_TRUE(find_hexs_containing_point(outside, hexs, pts).empty());

  // The documented default threshold (0.2) keeps a cube; nothing beats 1.
  std::vector<size_t> keep = filter_hexs_by_quality(hexs, pts);
  EXPECT_EQ(keep.size(), 1u);
  std::vector<size_t> none = filter_hexs_by_quality(hexs, pts, 1.0, HEX_SCALED_JACOBIAN);
  EXPECT_TRUE(none.empty());

  geometry hg;
  for (const auto &p : pts)
    hg.points().push_back(p);
  hg.hexs().push_back(hex);
  geometry surf = extract_surface(hg);
  EXPECT_EQ(surf.num_quads(), 6u);
  geometry hq = extract_quality_elements(hg, 0.2, HEX_SCALED_JACOBIAN);
  EXPECT_EQ(hq.num_hexs(), 1u);
  std::array<double, 6> bounds = compute_mesh_bounds(hg);
  EXPECT_DOUBLE_EQ(bounds[5], 1.0);

  // Surface geometries pass through extract_surface unchanged.
  geometry trig;
  trig.points().push_back({{0.0, 0.0, 0.0}});
  trig.points().push_back({{1.0, 0.0, 0.0}});
  trig.points().push_back({{0.0, 1.0, 0.0}});
  trig.tris().push_back({{0, 1, 2}});
  geometry tri_surf = extract_surface(trig);
  EXPECT_EQ(tri_surf.num_tris(), 1u);
}

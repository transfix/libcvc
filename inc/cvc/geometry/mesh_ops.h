/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// mesh_ops.h -- surface and tetrahedral mesh processing for cvc::geometry,
// backed by libigl (https://libigl.github.io, MPL-2.0 core module only) and
// Eigen. This header is deliberately Eigen-free and libigl-free: it ships in
// the SDK and is read by SWIG, so consumers (cvcGL, pycvc, cvc-cli, volrover)
// need neither package to compile against it.
//
// Availability. Everything here is always declared. When libcvc is built
// without CVC_ENABLE_LIBIGL (the package was not found, or the option is OFF)
// every function below except mesh_ops_available(), colormap_rgb(),
// robust_range(), colormap_from_string() and to_string(colormap_kind) throws
// cvc::mesh_ops_unavailable; default-constructing a mesh_locator or a
// geodesic_solver and calling empty() (or num_faces()) on it is also safe.
// Query mesh_ops_available() at runtime, or test the PUBLIC compile definition
// CVC_ENABLE_LIBIGL, which cvc exports to its consumers.
//
// Conventions shared by every function:
//  * Surface functions read tris() plus quads(); each quad (a,b,c,d) is split
//    into the triangles (a,b,c) and (a,c,d), and "triangle index" below means
//    an index into tris() followed by the two triangles of each quad, in order.
//    Geometry is never re-triangulated in place unless a function says so.
//  * Volume (tet) functions read tets(). Tets of either orientation are
//    accepted; functions that need positive orientation orient an internal
//    copy (see orient_tets() to fix the input itself).
//  * Per-vertex outputs are aligned with points(); vertices not referenced by
//    any element get 0 (or NaN where documented).
//  * Indices are validated: an element index >= num_points() throws
//    cvc::mesh_ops_error, as does a mesh with more than INT_MAX vertices or
//    elements (libigl uses 32-bit indices). The operations that assemble
//    finite-element matrices -- COTAN_IMPLICIT smoothing, MEAN and GAUSSIAN
//    curvature, geodesic_solver, and everything in fem.h -- also throw above
//    INT_MAX/16 (~134 million) triangles or tets, where libigl's own 32-bit
//    arithmetic on multiples of the element count would overflow.
//  * Operators built on cotangent weights (the same operations) leave out
//    triangles too flat for libigl's edge-length formulas: height/length below
//    ~7e-7, or a zero-area triangle. Near such a sliver the mesh behaves as if
//    cut along it.
//  * Lengths are in the mesh's own unit; nothing is rescaled unless documented.
//  * Inputs are read through the const_*() accessors, so no copy-on-write
//    detach happens on a const geometry.

#ifndef __CVC_GEOMETRY_MESH_OPS_H__
#define __CVC_GEOMETRY_MESH_OPS_H__

#include <cstddef>
#include <cstdint>
#include <cvc/core/exception.h>
#include <cvc/geometry/geometry.h>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

CVC_DEF_EXCEPTION(mesh_ops_unavailable); // built without CVC_ENABLE_LIBIGL
CVC_DEF_EXCEPTION(mesh_ops_error);       // invalid input (bad index, empty mesh, ...)

// True when libcvc was built with CVC_ENABLE_LIBIGL.
bool mesh_ops_available();

// --------------------------------------------------------------------------
// Normals and orientation
// --------------------------------------------------------------------------

enum class normal_weighting {
  UNIFORM, // plain average of incident face normals
  AREA,    // area-weighted (what geometry::compute_normals() does today)
  ANGLE    // incident-angle-weighted: tessellation-independent (default)
};

// Replace g.normals() with unit per-vertex normals of the triangle surface.
// Unreferenced/degenerate vertices get (0,0,0). Returns g.
geometry &compute_vertex_normals(geometry &g, normal_weighting w = normal_weighting::ANGLE);

// Re-wind the triangle surface so every connected component is consistently
// oriented and faces outward (igl::bfs_orient + igl::orient_outward). This is
// what geometry::reorient() was always meant to do: it changes the WINDING,
// not just the normals. Quads are triangulated in place first (quads()
// cleared, their triangles appended to tris()). If g.normals() was aligned
// with points() it is recomputed (ANGLE). Returns the number of triangles
// whose winding was flipped.
std::size_t orient_outward(geometry &g);

// --------------------------------------------------------------------------
// Repair
// --------------------------------------------------------------------------

struct repair_params {
  // Merge vertices at most this distance apart (0 => exact duplicates only;
  // < 0 => do not weld). Merging is transitive: a chain of such pairs becomes
  // one vertex (the lowest-numbered, which keeps its position). Vertices with
  // a non-finite coordinate are never merged.
  double weld_epsilon = 0.0;
  bool remove_degenerate = true;   // drop faces with repeated indices or ~zero area
  bool remove_unreferenced = true; // drop vertices no face/line/element references
  bool orient = true;              // orient_outward() after cleaning
};

struct repair_report {
  std::size_t vertices_removed = 0;
  std::size_t faces_removed = 0;
  std::size_t faces_flipped = 0;
};

// Clean a triangle surface in place. Per-vertex attribute arrays that are
// aligned with points() (normals, colors, uvs, tangents, curvatures,
// functions, boundary) are remapped so each surviving vertex keeps its own
// values; arrays that were not aligned are cleared. lines()/tets()/hexs()
// indices are remapped too.
repair_report repair(geometry &g, const repair_params &p = repair_params());

// --------------------------------------------------------------------------
// Curvature
// --------------------------------------------------------------------------

enum class curvature_kind {
  MEAN,          // H = (k1 + k2) / 2   (cotangent Laplacian, Voronoi-area normalised)
  GAUSSIAN,      // K = k1 * k2         (angle defect / Voronoi area)
  MAX_PRINCIPAL, // k1                  (quadric fit, igl::principal_curvature)
  MIN_PRINCIPAL  // k2
};

struct curvature_params {
  unsigned radius = 5;   // neighbourhood for the principal-curvature quadric fit (min 2)
  bool use_kring = true; // radius is a k-ring (true) or a multiple of mean edge length
};

// Sign convention: for a closed surface whose triangles are wound so the
// normals face outward (see orient_outward), convex regions have positive
// curvature -- a sphere of radius r gives H ~= 1/r, K ~= 1/r^2, k1 ~= k2 ~= 1/r.
//
// Where the values are not curvature:
//  * MEAN and GAUSSIAN are NaN on boundary vertices (a vertex on an edge that
//    only one triangle has, e.g. the rim of an open scan or of a hole): there
//    the angle defect and the cotangent Laplacian measure the boundary curve's
//    turning, not the surface. Triangles too flat for cotangent weights (see
//    the conventions above) count as missing, so their corners are NaN too.
//  * MAX_PRINCIPAL / MIN_PRINCIPAL (and compute_curvature) are NaN where no
//    quadric can be fitted (fewer than 6 vertices within reach). The fit
//    itself works at boundary vertices, from a one-sided neighbourhood.
//  * Vertices in no triangle get 0 (all kinds).
// colormap_rgb() draws NaN mid-gray and robust_range() ignores it.
//
// Cost: principal curvature is linear in the total size of the vertices'
// neighbourhoods (one quadric fit per vertex); MEAN and GAUSSIAN are linear.

// Fill g.curvatures() with (k1, k2), k1 >= k2, per vertex. Returns g.
geometry &compute_curvature(geometry &g, const curvature_params &p = curvature_params());

// One curvature scalar per vertex.
std::vector<double> vertex_curvature(const geometry &g, curvature_kind kind,
                                     const curvature_params &p = curvature_params());

// --------------------------------------------------------------------------
// Smoothing
// --------------------------------------------------------------------------

struct smooth_params {
  enum method_t {
    COTAN_IMPLICIT,    // implicit mean-curvature flow: (M - t L) V' = M V (unconditionally stable)
    UNIFORM_LAPLACIAN, // explicit umbrella operator: V' = V + lambda (avg(N(V)) - V)
    TAUBIN             // lambda|mu explicit pair (shrink-free low-pass)
  };
  method_t method = COTAN_IMPLICIT;
  int iterations = 1;
  // COTAN_IMPLICIT: dimensionless time step; the step actually used is
  //   t = lambda * d^2 with d the bounding-box diagonal, so the result does not
  //   depend on the mesh's scale.
  // UNIFORM_LAPLACIAN: explicit step in (0, 1].
  // TAUBIN: explicit step in (0, 0.7] (typically 0.3-0.6), with
  //   mu = -lambda / (1 - 0.1 * lambda) (pass-band k_PB = 0.1); larger steps
  //   amplify noise instead of removing it.
  // A step outside these ranges throws cvc::mesh_ops_error.
  double lambda = 1e-3;
  bool fix_boundary = false; // keep boundary-loop vertices where they are
  // COTAN_IMPLICIT only: after each step translate and uniformly rescale so the
  // surface area and centroid are unchanged (counters flow shrinkage).
  bool preserve_area = true;
};

// Smooth the vertex positions of a triangle surface in place (double
// precision; topology untouched). Normals aligned with points() are
// recomputed (ANGLE). Returns g.
geometry &smooth(geometry &g, const smooth_params &p = smooth_params());

// --------------------------------------------------------------------------
// Spatial queries: closest point, ray casting, signed distance, winding number
// --------------------------------------------------------------------------

struct closest_point_result {
  std::vector<double> sq_distance; // squared distance per query
  std::vector<std::int64_t> face;  // triangle index (see conventions), -1 if the surface is empty
  geometry::points_t points;       // closest point on the surface per query
};

struct ray_hit {
  std::int64_t face = -1; // triangle index (see conventions)
  double t = 0;           // ray parameter: hit = origin + t * dir
  double u = 0, v = 0;    // barycentrics: hit = (1-u-v) A + u B + v C of that triangle
  geometry::point_t point = {{0, 0, 0}};
};

// An AABB tree over a triangle surface (plus a fast-winding-number BVH, built
// lazily on first use), reusable across many queries. Build it once per mesh
// version: it holds a COPY of the positions, so later edits to the source
// geometry are not seen. Copies share the same immutable tree. Queries are
// const and thread-safe.
class mesh_locator {
public:
  mesh_locator();
  explicit mesh_locator(const geometry &surface);
  void build(const geometry &surface);

  bool empty() const;
  std::size_t num_faces() const;

  closest_point_result closest_points(const geometry::points_t &queries) const;

  // First hit with t in [min_t, max_t]; dir need not be unit length, and
  // neither its length nor the mesh's unit or triangle count changes what is
  // hit (the test is double precision and scale-free). A ray within ~1e-14 rad
  // of a triangle's plane does not hit it. Returns false (and leaves hit
  // untouched) on a miss.
  bool intersect_ray(const geometry::point_t &origin, const geometry::vector_t &dir, ray_hit &hit,
                     double min_t = 0.0,
                     double max_t = std::numeric_limits<double>::infinity()) const;

  // Signed distance: exact AABB distance with the sign from the fast
  // generalized winding number (negative inside). Robust to holes, overlaps
  // and self-intersections -- unlike a pseudo-normal sign, it does not need a
  // watertight surface.
  std::vector<double> signed_distance(const geometry::points_t &queries) const;

  // Generalized winding number (~1 inside a closed outward surface, ~0 outside).
  std::vector<double> winding_number(const geometry::points_t &queries) const;

  struct impl; // defined in the libigl-backed implementation
private:
  std::shared_ptr<const impl> _impl;
};

// One-shot conveniences (build a temporary mesh_locator).
closest_point_result closest_points(const geometry &surface, const geometry::points_t &queries);
std::vector<double> signed_distance(const geometry &surface, const geometry::points_t &queries);

// --------------------------------------------------------------------------
// Geodesic distance (heat method, Crane et al. 2013) on triangle surfaces
// --------------------------------------------------------------------------

// Caches the two sparse factorizations, so each distance() is two
// back-substitutions. Build once per mesh version.
class geodesic_solver {
public:
  geodesic_solver();
  // t_scale multiplies the default heat time h^2 (h = mean edge length).
  explicit geodesic_solver(const geometry &surface, double t_scale = 1.0);
  bool empty() const;
  // Approximate geodesic distance from the source vertices to every vertex
  // (0 at the sources; unreachable components, and vertices in no usable
  // triangle, get +infinity). A connected part with no interior vertex (an
  // isolated triangle, a one-triangle-wide strip) gets exact shortest paths
  // along its edges instead. Throws cvc::mesh_ops_error if a solve produces a
  // non-finite distance.
  std::vector<double> distance(const std::vector<index_t> &sources) const;

  struct impl;

private:
  std::shared_ptr<const impl> _impl;
};

// One-shot convenience.
std::vector<double> geodesic_distance(const geometry &surface, const std::vector<index_t> &sources);

// --------------------------------------------------------------------------
// Tetrahedral meshes
// --------------------------------------------------------------------------

// Signed volume per tet (positive = right-handed (v1-v0, v2-v0, v3-v0)).
std::vector<double> tet_volumes(const geometry &tetmesh);

// Swap vertices 2 and 3 of every tet with negative signed volume, in place.
// Returns the number of tets flipped. (cvc::tetrahedralize() output decodes
// tets with arbitrary orientation; finite elements need them positive.)
std::size_t orient_tets(geometry &tetmesh);

// The boundary of tets() as an outward-wound triangle surface
// (igl::boundary_facets). The result keeps ALL of tetmesh's points (and any
// per-vertex colors/functions/normals aligned with them) so a per-vertex
// field on the tet mesh maps 1:1 onto it; interior points are simply
// unreferenced. Geometry type SURFACE_TRI.
geometry tet_boundary_surface(const geometry &tetmesh);

// The level set {field = isovalue} of a per-vertex scalar on a tet mesh, as a
// triangle surface (igl::marching_tets). functions() of the result carries the
// interpolated tetmesh.functions() when that array is aligned with points();
// colors() likewise.
geometry tet_isosurface(const geometry &tetmesh, const std::vector<double> &field, double isovalue);

// Planar cross-section {x : dot(normal, x) = offset} of a tet mesh -- a filled
// cut, with interpolated functions()/colors() as for tet_isosurface. Works where
// GPU clip planes are unavailable (WebGL).
geometry slice_tets(const geometry &tetmesh, const geometry::vector_t &normal, double offset);

// --------------------------------------------------------------------------
// Scalar field -> colour (always available, with or without libigl)
// --------------------------------------------------------------------------

enum class colormap_kind { VIRIDIS, MAGMA, PLASMA, INFERNO, TURBO, JET, PARULA, GRAY };

// "viridis", "magma", "plasma", "inferno", "turbo", "jet", "parula", "gray"
// (case-insensitive). Returns false (k untouched) for an unknown name.
bool colormap_from_string(const std::string &name, colormap_kind &k);
std::string to_string(colormap_kind k);

// Map values to packed 8-bit RGB (3 bytes per value), linearly from [lo, hi]
// onto the colormap. A NaN lo/hi is replaced by the min/max of the finite
// values; non-finite values map to mid-gray (128,128,128). Uses libigl's
// colormap tables when available, otherwise a close piecewise-linear
// approximation of the same maps.
std::vector<unsigned char> colormap_rgb(const std::vector<double> &values, colormap_kind k,
                                        double lo = std::numeric_limits<double>::quiet_NaN(),
                                        double hi = std::numeric_limits<double>::quiet_NaN());

// Percentile range of the finite values (e.g. 0.02/0.98) -- a colour range that
// is not dominated by a few outliers (curvature on a scanned mesh). Returns
// (NaN, NaN) when no value is finite.
std::pair<double, double> robust_range(const std::vector<double> &values, double lo_pct = 0.02,
                                       double hi_pct = 0.98);

} // namespace cvc

#endif // __CVC_GEOMETRY_MESH_OPS_H__

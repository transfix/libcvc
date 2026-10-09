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

// igl_detail.h -- INTERNAL seam between libcvc and libigl/Eigen (not installed).
//
// The public API (mesh_ops.h, fem.h) is implemented in mesh_ops.cpp/fem.cpp,
// which own cvc::geometry and never include Eigen or libigl. They call the
// functions declared here, which are implemented in the igl_*.cpp files of this
// directory. Those TUs include NO cvc header -- only this one, the standard
// library, Eigen and libigl -- so they compile outside cvc's context (NOMINMAX,
// WinSock, Boost, the SDF MIN/MAX macros) and can live in a separate static
// library. This header therefore uses standard types only.
//
// Arrays are packed row-major: V holds 3 doubles per vertex, F 3 ints per
// triangle, T 4 ints per tet; E is F or T as given by its simplex size (3 or 4).
// Indices are already validated (in range, < INT_MAX) by the caller.

#ifndef CVC_GEOMETRY_IGL_DETAIL_H
#define CVC_GEOMETRY_IGL_DETAIL_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace igl_detail {

typedef std::vector<double> dvec;
typedef std::vector<int> ivec;
typedef std::vector<char> mask;

// The igl TUs are compiled with hidden visibility while mesh_ops.cpp/fem.cpp,
// which catch `failure`, are not. Without an explicit attribute its inline
// (key-function-less) typeinfo is emitted with both visibilities, and Apple's
// ld64 warns about every libcvc.dylib link. The exception never leaves libcvc
// (igl_bridge::guarded() translates it), so hidden is right on both sides.
#if defined(__GNUC__) && !defined(_WIN32)
#define CVC_IGL_DETAIL_HIDDEN __attribute__((visibility("hidden")))
#else
#define CVC_IGL_DETAIL_HIDDEN
#endif

// Thrown by the igl TUs (which cannot see cvc's exception types); the cvc side
// translates it to cvc::mesh_ops_error.
class CVC_IGL_DETAIL_HIDDEN failure : public std::runtime_error {
public:
  explicit failure(const std::string &what) : std::runtime_error(what) {}
};

// ---- surfaces (igl_surface.cpp) --------------------------------------------

// weighting: 0 uniform, 1 area, 2 angle. N gets 3 doubles per vertex; vertices
// with no non-degenerate incident triangle get (0,0,0).
void vertex_normals(const dvec &V, const ivec &F, int weighting, dvec &N);

// bfs_orient + orient_outward. FF has F's size; each row is F's row or its
// reverse.
void orient_outward(const dvec &V, const ivec &F, ivec &FF);

// Twice the area of each triangle.
void double_areas(const dvec &V, const ivec &F, dvec &dblA);

// rep[i] = the smallest index of the vertices that weld with i. eps == 0:
// exactly equal positions. eps > 0: every pair at distance <= eps is joined,
// transitively (union-find), so a chain of such pairs becomes one class.
// Vertices with a non-finite coordinate never weld.
void weld_vertices(const dvec &V, double eps, ivec &rep);

// Facets that belong to exactly one element: edges (2 ints each) of triangles
// for simplex_size 3, triangles (3 ints) of tets for simplex_size 4. Tet facets
// face outward when the tets are positively oriented.
void boundary_facets(const ivec &E, int simplex_size, ivec &B);

// Principal curvatures k1 >= k2: igl::principal_curvature's quadric fit (same
// neighbourhoods, frame and fit), with the neighbourhoods gathered in time
// proportional to their size -- libigl clears a #V-sized visited array per
// vertex, O(#V^2) in all -- and safe for vertices above max(F) in sphere mode.
// ok[i] == 0 where no fit exists (fewer than 6 neighbours, or no normal).
void principal_curvature(const dvec &V, const ivec &F, unsigned radius, bool use_kring, dvec &k1,
                         dvec &k2, mask &ok);
// Signed mean curvature (positive where the surface bends away from its
// normals) and Gaussian curvature, both per unit Voronoi area, over the
// cot_safe triangles (igl_eigen.h). 0 where a vertex is in no such triangle;
// NaN on the boundary of those triangles (an edge with one of them), where
// neither formula measures the surface's curvature.
void mean_curvature(const dvec &V, const ivec &F, dvec &H);
void gaussian_curvature(const dvec &V, const ivec &F, dvec &K);

// Implicit cotangent mean-curvature flow, (M - t L) V' = M V per iteration with
// t = lambda * (bounding-box diagonal)^2, over the cot_safe triangles. Vertices
// with fixed[i] != 0, or in no such triangle, stay put. preserve_area rescales
// about the area centroid after each step (ignored when any vertex is fixed).
void smooth_implicit(dvec &V, const ivec &F, int iterations, double lambda, const mask &fixed,
                     bool preserve_area);
// Explicit umbrella steps V += lambda (avg(N(V)) - V); when mu != 0 each
// iteration also takes a mu step (Taubin).
void smooth_explicit(dvec &V, const ivec &F, int iterations, double lambda, double mu,
                     const mask &fixed);

// libigl colour table lookup, f in [0,1]. map: 0 viridis, 1 magma, 2 plasma,
// 3 inferno, 4 turbo, 5 parula.
void colormap(int map, double f, double rgb[3]);

// ---- spatial queries (igl_locator.cpp) ---------------------------------------

struct locator_state;
std::shared_ptr<locator_state> make_locator(const dvec &V, const ivec &F);
std::size_t locator_num_faces(const locator_state &s);
// Per query: squared distance (+inf if no faces), face (-1), closest point (NaN).
void locator_closest(const locator_state &s, const dvec &Q, dvec &sqd,
                     std::vector<std::int64_t> &face, dvec &C);
// First hit with t in [min_t, max_t]; false on a miss. min_t must be finite.
// A double-precision traversal of the AABB tree with a scale-free
// near-parallel test, so neither the mesh's unit nor |dir| decides what is hit.
bool locator_ray(const locator_state &s, const double origin[3], const double dir[3], double min_t,
                 double max_t, std::int64_t &face, double &t, double &u, double &v);
// Fast generalized winding number (built on first use, thread-safe).
void locator_winding(const locator_state &s, const dvec &Q, dvec &W);

// ---- tets (igl_tet.cpp) --------------------------------------------------------

void tet_volumes(const dvec &V, const ivec &T, dvec &vol);

// Marching tets of the per-vertex field S at iso. Faces are wound so their
// normals point toward increasing S. Output vertex i is sum_j w * V[j] over the
// triplets (bc_row == i, bc_col == j, bc_val == w).
void marching_tets(const dvec &V, const ivec &T, const dvec &S, double iso, dvec &SV, ivec &SF,
                   ivec &bc_row, ivec &bc_col, dvec &bc_val);

// ---- finite elements and geodesics (igl_fem.cpp) --------------------------------

struct triplets {
  ivec row, col;
  dvec val;
};

// keep[e] = 1 for the elements of E whose cotangent weights igl::cotmatrix
// computes reliably (cot_safe_triangle / cot_safe_tet in igl_eigen.h), else 0.
void well_shaped(const dvec &V, const ivec &E, int simplex_size, mask &keep);

// Elements E must be well_shaped and (tets) positively oriented. Matrices are
// #V x #V (V may have vertices no element references).
void stiffness(const dvec &V, const ivec &E, int simplex_size, triplets &K);
void mass(const dvec &V, const ivec &E, int simplex_size, bool lumped, triplets &M);
// Per-element constant gradient of u, 3 doubles per element of E.
void gradient(const dvec &V, const ivec &E, int simplex_size, const dvec &u, dvec &G);

// Minimiser of 0.5 u'Au - u'M f with A = mass_coef M + stiff_coef K (lumped M)
// and u(known) = Y, factored once and solved for any (f, Y).
struct quadratic_system;
std::shared_ptr<quadratic_system> make_quadratic_system(const dvec &V, const ivec &E,
                                                        int simplex_size, double mass_coef,
                                                        double stiff_coef, const ivec &known);
void solve_quadratic(const quadratic_system &s, const dvec &f, const dvec &Y, dvec &u);

// The k smallest eigenpairs of K x = lambda M x (lumped M) restricted to the
// vertices with active[i] != 0; vectors are M-normalised, 0 off the active set,
// eigenvalues ascending. Throws failure if the iterative solver (above 400
// active vertices) does not converge.
void eigenmodes(const dvec &V, const ivec &E, int simplex_size, const mask &active, int k,
                dvec &values, std::vector<dvec> &vectors);

// Heat-method geodesics over the cot_safe triangles, one factorisation per
// connected component. A component with no interior vertex (an isolated
// triangle, a one-triangle-wide strip) has nothing for the heat method's
// Dirichlet solve to solve for; it gets exact shortest paths along its edges.
struct geodesic_state;
std::shared_ptr<geodesic_state> make_geodesic(const dvec &V, const ivec &F, double t_scale);
// D: +inf for vertices not connected to any source, 0 at the sources. Throws
// failure if the solve produces a non-finite distance.
void geodesic_distance(const geodesic_state &s, const ivec &sources, dvec &D);

} // namespace igl_detail
} // namespace cvc

#endif // CVC_GEOMETRY_IGL_DETAIL_H

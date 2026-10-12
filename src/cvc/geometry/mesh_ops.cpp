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

// mesh_ops.cpp -- the cvc::geometry side of mesh_ops.h: argument checks,
// packing a geometry for the libigl code in igl/ (through igl/igl_detail.h),
// and writing results and per-vertex attributes back. Every public body lives
// here, in cvc's own objects, so the Windows export table covers it; this TU
// never sees Eigen or libigl. Built without CVC_ENABLE_LIBIGL, everything but
// the colour maps throws mesh_ops_unavailable.

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cvc/geometry/mesh_ops.h>
#include <limits>

#ifdef CVC_ENABLE_LIBIGL
#include "igl/igl_bridge.h"
#include "igl/igl_detail.h"
#endif

namespace cvc {

bool mesh_ops_available() {
#ifdef CVC_ENABLE_LIBIGL
  return true;
#else
  return false;
#endif
}

#ifdef CVC_ENABLE_LIBIGL

// ---- geometry <-> packed arrays -----------------------------------------------

namespace igl_bridge {

namespace {
int checked(index_t i, std::size_t n) {
  if (i >= n)
    throw mesh_ops_error("element index " + std::to_string(i) + " >= num_points() " +
                         std::to_string(n));
  return int(i);
}

void check_count(std::size_t n, const char *what) {
  if (n > std::size_t(INT_MAX))
    throw mesh_ops_error(std::string("more than INT_MAX ") + what +
                         " (libigl uses 32-bit indices)");
}
} // namespace

std::vector<double> positions(const geometry &g) {
  const points_t &P = g.const_points();
  check_count(P.size(), "vertices");
  std::vector<double> V(3 * P.size());
  for (std::size_t i = 0; i < P.size(); ++i)
    for (int c = 0; c < 3; ++c)
      V[3 * i + c] = P[i][c];
  return V;
}

std::vector<int> surface_triangles(const geometry &g) {
  const std::size_t n = g.const_points().size();
  const tris_t &T = g.const_tris();
  const quads_t &Q = g.const_quads();
  check_count(T.size() + 2 * Q.size(), "triangles");
  std::vector<int> F;
  F.reserve(3 * (T.size() + 2 * Q.size()));
  for (const tri_t &t : T)
    for (int c = 0; c < 3; ++c)
      F.push_back(checked(t[c], n));
  for (const quad_t &q : Q) {
    const int a = checked(q[0], n), b = checked(q[1], n), c = checked(q[2], n),
              d = checked(q[3], n);
    F.insert(F.end(), {a, b, c, a, c, d});
  }
  return F;
}

std::vector<int> tetrahedra(const geometry &g) {
  const std::size_t n = g.const_points().size();
  const tets_t &T = g.const_tets();
  check_count(T.size(), "tets");
  std::vector<int> E;
  E.reserve(4 * T.size());
  for (const tet_t &t : T)
    for (int c = 0; c < 4; ++c)
      E.push_back(checked(t[c], n));
  return E;
}

double diagonal(const std::vector<double> &V) {
  if (V.empty())
    return 0.0;
  double lo[3] = {V[0], V[1], V[2]}, hi[3] = {V[0], V[1], V[2]};
  for (std::size_t i = 0; i < V.size(); i += 3)
    for (int c = 0; c < 3; ++c) {
      lo[c] = std::min(lo[c], V[i + c]);
      hi[c] = std::max(hi[c], V[i + c]);
    }
  const double d = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                             (hi[2] - lo[2]) * (hi[2] - lo[2]));
  return std::isfinite(d) ? d : 0.0;
}

void check_fem_count(std::size_t elements, const char *what) {
  if (elements > kMaxFemElements)
    throw mesh_ops_error(std::string(what) + ": more than INT_MAX/16 (" +
                         std::to_string(kMaxFemElements) +
                         ") elements, where libigl's 32-bit index arithmetic overflows");
}

} // namespace igl_bridge

using igl_bridge::check_fem_count;
using igl_bridge::guarded;
using igl_bridge::positions;
using igl_bridge::surface_triangles;
using igl_bridge::tetrahedra;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<char> referenced(const std::vector<int> &E, std::size_t n) {
  std::vector<char> r(n, 0);
  for (int v : E)
    r[std::size_t(v)] = 1;
  return r;
}

std::vector<double> pack(const points_t &P) {
  std::vector<double> Q(3 * P.size());
  for (std::size_t i = 0; i < P.size(); ++i)
    for (int c = 0; c < 3; ++c)
      Q[3 * i + c] = P[i][c];
  return Q;
}

void unpack(const std::vector<double> &V, points_t &P) {
  P.resize(V.size() / 3);
  for (std::size_t i = 0; i < P.size(); ++i)
    for (int c = 0; c < 3; ++c)
      P[i][c] = V[3 * i + c];
}

bool aligned_normals(const geometry &g) {
  return g.num_points() > 0 && g.const_normals().size() == g.num_points();
}

// The per-vertex field S of a tet mesh at iso as a triangle surface, carrying
// the tet mesh's aligned functions()/colors() through the interpolation weights.
geometry level_set(const geometry &tetmesh, const std::vector<double> &S, double iso) {
  std::vector<double> SV, val;
  std::vector<int> SF, row, col;
  igl_detail::marching_tets(positions(tetmesh), tetrahedra(tetmesh), S, iso, SV, SF, row, col, val);
  geometry out;
  unpack(SV, out.points());
  tris_t &T = out.tris();
  T.resize(SF.size() / 3);
  for (std::size_t f = 0; f < T.size(); ++f)
    T[f] = {{index_t(SF[3 * f]), index_t(SF[3 * f + 1]), index_t(SF[3 * f + 2])}};
  const std::size_t n = tetmesh.num_points(), m = SV.size() / 3;
  const functions_t &fn = tetmesh.const_functions();
  if (fn.size() == n && n > 0) {
    functions_t &ofn = out.functions();
    ofn.assign(m, 0.0);
    for (std::size_t k = 0; k < val.size(); ++k)
      ofn[std::size_t(row[k])] += val[k] * fn[std::size_t(col[k])];
  }
  const colors_t &cl = tetmesh.const_colors();
  if (cl.size() == n && n > 0) {
    colors_t &ocl = out.colors();
    ocl.assign(m, color_t{{0, 0, 0}});
    for (std::size_t k = 0; k < val.size(); ++k)
      for (int c = 0; c < 3; ++c)
        ocl[std::size_t(row[k])][c] += val[k] * cl[std::size_t(col[k])][c];
  }
  out.set_geometry_type(geometry::SURFACE_TRI);
  return out;
}

// Keep the entries of the surviving vertices (newidx != -1), in order; clear an
// array that was not aligned with the n0 points.
template <class Vec> void compact(Vec &a, const std::vector<index_t> &newidx, std::size_t n0) {
  if (a.size() != n0) {
    a.clear();
    return;
  }
  Vec out;
  for (std::size_t i = 0; i < n0; ++i)
    if (newidx[i] != index_t(-1))
      out.push_back(a[i]);
  a.swap(out);
}

} // namespace

// ---- normals and orientation --------------------------------------------------

geometry &compute_vertex_normals(geometry &g, normal_weighting w) {
  guarded([&] {
    std::vector<double> N;
    const int weighting = w == normal_weighting::UNIFORM ? 0 : w == normal_weighting::AREA ? 1 : 2;
    igl_detail::vertex_normals(positions(g), surface_triangles(g), weighting, N);
    normals_t &out = g.normals();
    out.resize(N.size() / 3);
    for (std::size_t i = 0; i < out.size(); ++i)
      out[i] = {{N[3 * i], N[3 * i + 1], N[3 * i + 2]}};
  });
  return g;
}

std::size_t orient_outward(geometry &g) {
  return guarded([&]() -> std::size_t {
    const std::vector<int> F = surface_triangles(g);
    if (F.empty())
      return 0;
    std::vector<int> FF;
    igl_detail::orient_outward(positions(g), F, FF);
    std::size_t flipped = 0;
    for (std::size_t f = 0; f < F.size(); f += 3)
      if (FF[f] != F[f] || FF[f + 1] != F[f + 1] || FF[f + 2] != F[f + 2])
        ++flipped;
    const bool has_quads = !g.const_quads().empty();
    if (flipped == 0 && !has_quads)
      return 0;
    const bool renormal = aligned_normals(g);
    tris_t &T = g.tris();
    T.resize(FF.size() / 3);
    for (std::size_t f = 0; f < T.size(); ++f)
      T[f] = {{index_t(FF[3 * f]), index_t(FF[3 * f + 1]), index_t(FF[3 * f + 2])}};
    if (has_quads)
      g.quads().clear();
    if (renormal)
      compute_vertex_normals(g, normal_weighting::ANGLE);
    return flipped;
  });
}

// ---- repair ----------------------------------------------------------------------

repair_report repair(geometry &g, const repair_params &p) {
  return guarded([&] {
    repair_report report;
    const std::size_t n0 = g.num_points();
    const std::vector<double> V = positions(g);
    // Validate every element index before anything is modified.
    surface_triangles(g);
    tetrahedra(g);
    for (const line_t &l : g.const_lines())
      if (l[0] >= n0 || l[1] >= n0)
        throw mesh_ops_error("repair: line index >= num_points()");
    for (const hex_t &h : g.const_hexs())
      for (index_t v : h)
        if (v >= n0)
          throw mesh_ops_error("repair: hex index >= num_points()");

    // 1. Weld: point every element corner at its vertex's representative.
    std::vector<index_t> rep(n0);
    for (std::size_t i = 0; i < n0; ++i)
      rep[i] = i;
    bool welded = false;
    if (p.weld_epsilon >= 0 && n0 > 0) {
      std::vector<int> r;
      igl_detail::weld_vertices(V, p.weld_epsilon, r);
      for (std::size_t i = 0; i < n0; ++i) {
        rep[i] = index_t(r[i]);
        welded = welded || rep[i] != i;
      }
    }
    tris_t tris = g.const_tris();
    quads_t quads = g.const_quads();
    lines_t lines = g.const_lines();
    tets_t tets = g.const_tets();
    hexs_t hexs = g.const_hexs();
    const auto remap = [](auto &elems, const std::vector<index_t> &to) {
      for (auto &e : elems)
        for (index_t &v : e)
          v = to[v];
    };
    if (welded) {
      remap(tris, rep);
      remap(quads, rep);
      remap(lines, rep);
      remap(tets, rep);
      remap(hexs, rep);
    }

    // 2. Degenerate faces: a repeated corner or ~zero area.
    bool faces_changed = welded;
    if (p.remove_degenerate) {
      const double d = igl_bridge::diagonal(V);
      const double tol = 1e-12 * d * d; // area
      const auto area2 = [&](index_t a, index_t b, index_t c) {
        const double *A = &V[3 * a], *B = &V[3 * b], *C = &V[3 * c];
        const double u[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]};
        const double w[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
        const double x = u[1] * w[2] - u[2] * w[1], y = u[2] * w[0] - u[0] * w[2],
                     z = u[0] * w[1] - u[1] * w[0];
        return std::sqrt(x * x + y * y + z * z);
      };
      tris_t kept_tris;
      for (const tri_t &t : tris)
        if (t[0] != t[1] && t[1] != t[2] && t[0] != t[2] && 0.5 * area2(t[0], t[1], t[2]) > tol)
          kept_tris.push_back(t);
      quads_t kept_quads;
      for (const quad_t &q : quads) {
        bool distinct = true;
        for (int i = 0; i < 4; ++i)
          for (int j = i + 1; j < 4; ++j)
            distinct = distinct && q[i] != q[j];
        if (distinct && 0.5 * (area2(q[0], q[1], q[2]) + area2(q[0], q[2], q[3])) > tol)
          kept_quads.push_back(q);
      }
      report.faces_removed = (tris.size() - kept_tris.size()) + (quads.size() - kept_quads.size());
      faces_changed = faces_changed || report.faces_removed > 0;
      tris.swap(kept_tris);
      quads.swap(kept_quads);
    }

    // 3. Unreferenced vertices, and the per-vertex attributes along with them.
    std::vector<index_t> newidx(n0);
    std::size_t n1 = n0;
    if (p.remove_unreferenced) {
      std::vector<char> used(n0, 0);
      const auto mark = [&](const auto &elems) {
        for (const auto &e : elems)
          for (index_t v : e)
            used[v] = 1;
      };
      mark(tris);
      mark(quads);
      mark(lines);
      mark(tets);
      mark(hexs);
      n1 = 0;
      for (std::size_t i = 0; i < n0; ++i)
        newidx[i] = used[i] ? index_t(n1++) : index_t(-1);
    }
    if (n1 != n0) {
      remap(tris, newidx);
      remap(quads, newidx);
      remap(lines, newidx);
      remap(tets, newidx);
      remap(hexs, newidx);
      points_t P = g.const_points();
      compact(P, newidx, n0);
      g.points().swap(P);
      normals_t nrm = g.const_normals();
      compact(nrm, newidx, n0);
      g.normals().swap(nrm);
      colors_t col = g.const_colors();
      compact(col, newidx, n0);
      g.colors().swap(col);
      uvs_t uv = g.const_uvs();
      compact(uv, newidx, n0);
      g.uvs().swap(uv);
      tangents_t tan = g.const_tangents();
      compact(tan, newidx, n0);
      g.tangents().swap(tan);
      curvatures_t cur = g.const_curvatures();
      compact(cur, newidx, n0);
      g.curvatures().swap(cur);
      functions_t fn = g.const_functions();
      compact(fn, newidx, n0);
      g.functions().swap(fn);
      const boundary_t &b0 = g.const_boundary();
      boundary_t b1;
      if (b0.size() == n0)
        for (std::size_t i = 0; i < n0; ++i)
          if (newidx[i] != index_t(-1))
            b1.push_back(b0[i]);
      g.boundary().swap(b1);
      report.vertices_removed = n0 - n1;
    }
    if (faces_changed || n1 != n0) {
      g.tris().swap(tris);
      g.quads().swap(quads);
    }
    if (welded || n1 != n0) {
      g.lines().swap(lines);
      g.tets().swap(tets);
      g.hexs().swap(hexs);
    }

    if (p.orient)
      report.faces_flipped = orient_outward(g);
    return report;
  });
}

// ---- curvature ---------------------------------------------------------------------

geometry &compute_curvature(geometry &g, const curvature_params &p) {
  guarded([&] {
    const std::vector<int> F = surface_triangles(g);
    std::vector<double> k1, k2;
    std::vector<char> ok;
    igl_detail::principal_curvature(positions(g), F, p.radius, p.use_kring, k1, k2, ok);
    const std::vector<char> used = referenced(F, g.num_points());
    curvatures_t &out = g.curvatures();
    out.resize(k1.size());
    for (std::size_t i = 0; i < out.size(); ++i)
      out[i] = !used[i] ? curvature_t{{0.0, 0.0}}
               : ok[i]  ? curvature_t{{k1[i], k2[i]}}
                        : curvature_t{{kNaN, kNaN}};
  });
  return g;
}

std::vector<double> vertex_curvature(const geometry &g, curvature_kind kind,
                                     const curvature_params &p) {
  return guarded([&] {
    const std::vector<double> V = positions(g);
    const std::vector<int> F = surface_triangles(g);
    std::vector<double> out;
    switch (kind) {
    case curvature_kind::MEAN:
      check_fem_count(F.size() / 3, "vertex_curvature(MEAN)");
      igl_detail::mean_curvature(V, F, out);
      break;
    case curvature_kind::GAUSSIAN:
      check_fem_count(F.size() / 3, "vertex_curvature(GAUSSIAN)");
      igl_detail::gaussian_curvature(V, F, out);
      break;
    case curvature_kind::MAX_PRINCIPAL:
    case curvature_kind::MIN_PRINCIPAL: {
      std::vector<double> k1, k2;
      std::vector<char> ok;
      igl_detail::principal_curvature(V, F, p.radius, p.use_kring, k1, k2, ok);
      const std::vector<char> used = referenced(F, g.num_points());
      out = kind == curvature_kind::MAX_PRINCIPAL ? k1 : k2;
      for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = !used[i] ? 0.0 : ok[i] ? out[i] : kNaN;
    } break;
    }
    return out;
  });
}

// ---- smoothing -----------------------------------------------------------------------

geometry &smooth(geometry &g, const smooth_params &p) {
  guarded([&] {
    if (p.iterations <= 0)
      return;
    if (!(p.lambda > 0) || !std::isfinite(p.lambda))
      throw mesh_ops_error("smooth: lambda must be a positive, finite step");
    // The explicit steps are stable only for small enough lambda. Per iteration
    // a mode of the umbrella operator I - D^-1 A with eigenvalue k in [0, 2)
    // (~1.5 on a regular mesh) is scaled by 1 - lambda k (UNIFORM_LAPLACIAN:
    // |.| <= 1 needs lambda <= 1) or (1 - lambda k)(1 - mu k) (TAUBIN: at
    // lambda = 1, mu = -1.11 and k = 1.5 that is -1.33, and noise grows;
    // lambda <= 0.7 keeps it within [-1, 1] for k up to ~1.99).
    if (p.method == smooth_params::UNIFORM_LAPLACIAN && p.lambda > 1.0)
      throw mesh_ops_error("smooth: UNIFORM_LAPLACIAN needs lambda in (0, 1]");
    if (p.method == smooth_params::TAUBIN && p.lambda > 0.7)
      throw mesh_ops_error("smooth: TAUBIN needs lambda in (0, 0.7]");
    const std::vector<int> F = surface_triangles(g);
    if (F.empty())
      return;
    if (p.method == smooth_params::COTAN_IMPLICIT)
      check_fem_count(F.size() / 3, "smooth(COTAN_IMPLICIT)");
    std::vector<double> V = positions(g);
    std::vector<char> fixed;
    if (p.fix_boundary) {
      std::vector<int> B;
      igl_detail::boundary_facets(F, 3, B);
      fixed.assign(g.num_points(), 0);
      for (int v : B)
        fixed[std::size_t(v)] = 1;
    }
    switch (p.method) {
    case smooth_params::COTAN_IMPLICIT:
      igl_detail::smooth_implicit(V, F, p.iterations, p.lambda, fixed, p.preserve_area);
      break;
    case smooth_params::UNIFORM_LAPLACIAN:
      igl_detail::smooth_explicit(V, F, p.iterations, p.lambda, 0.0, fixed);
      break;
    case smooth_params::TAUBIN:
      igl_detail::smooth_explicit(V, F, p.iterations, p.lambda, -p.lambda / (1.0 - 0.1 * p.lambda),
                                  fixed);
      break;
    }
    const bool renormal = aligned_normals(g);
    unpack(V, g.points());
    if (renormal)
      compute_vertex_normals(g, normal_weighting::ANGLE);
  });
  return g;
}

// ---- spatial queries -------------------------------------------------------------------

struct mesh_locator::impl {
  std::shared_ptr<igl_detail::locator_state> state;
};

mesh_locator::mesh_locator() {}

mesh_locator::mesh_locator(const geometry &surface) { build(surface); }

void mesh_locator::build(const geometry &surface) {
  guarded([&] {
    auto im = std::make_shared<impl>();
    im->state = igl_detail::make_locator(positions(surface), surface_triangles(surface));
    _impl = im;
  });
}

bool mesh_locator::empty() const { return num_faces() == 0; }

std::size_t mesh_locator::num_faces() const {
  return _impl ? igl_detail::locator_num_faces(*_impl->state) : 0;
}

closest_point_result mesh_locator::closest_points(const geometry::points_t &queries) const {
  return guarded([&] {
    closest_point_result r;
    std::vector<double> C;
    if (_impl) {
      igl_detail::locator_closest(*_impl->state, pack(queries), r.sq_distance, r.face, C);
    } else {
      r.sq_distance.assign(queries.size(), std::numeric_limits<double>::infinity());
      r.face.assign(queries.size(), -1);
      C.assign(3 * queries.size(), kNaN);
    }
    unpack(C, r.points);
    return r;
  });
}

bool mesh_locator::intersect_ray(const geometry::point_t &origin, const geometry::vector_t &dir,
                                 ray_hit &hit, double min_t, double max_t) const {
  if (!std::isfinite(min_t))
    throw mesh_ops_error("intersect_ray: min_t must be finite");
  if (!_impl)
    return false;
  return guarded([&] {
    const double o[3] = {origin[0], origin[1], origin[2]};
    const double d[3] = {dir[0], dir[1], dir[2]};
    std::int64_t face;
    double t, u, v;
    if (!igl_detail::locator_ray(*_impl->state, o, d, min_t, max_t, face, t, u, v))
      return false;
    hit.face = face;
    hit.t = t;
    hit.u = u;
    hit.v = v;
    for (int c = 0; c < 3; ++c)
      hit.point[c] = o[c] + t * d[c];
    return true;
  });
}

std::vector<double> mesh_locator::signed_distance(const geometry::points_t &queries) const {
  return guarded([&] {
    std::vector<double> S(queries.size(), std::numeric_limits<double>::infinity());
    if (!_impl || empty())
      return S;
    const std::vector<double> Q = pack(queries);
    std::vector<double> sqd, C, W;
    std::vector<std::int64_t> face;
    igl_detail::locator_closest(*_impl->state, Q, sqd, face, C);
    igl_detail::locator_winding(*_impl->state, Q, W);
    // libigl's rule: inside when |w| > 1/2, whichever way the surface is wound.
    for (std::size_t i = 0; i < S.size(); ++i)
      S[i] = std::abs(W[i]) > 0.5 ? -std::sqrt(sqd[i]) : std::sqrt(sqd[i]);
    return S;
  });
}

std::vector<double> mesh_locator::winding_number(const geometry::points_t &queries) const {
  return guarded([&] {
    std::vector<double> W(queries.size(), 0.0);
    if (_impl)
      igl_detail::locator_winding(*_impl->state, pack(queries), W);
    return W;
  });
}

closest_point_result closest_points(const geometry &surface, const geometry::points_t &queries) {
  return mesh_locator(surface).closest_points(queries);
}

std::vector<double> signed_distance(const geometry &surface, const geometry::points_t &queries) {
  return mesh_locator(surface).signed_distance(queries);
}

// ---- geodesics ---------------------------------------------------------------------------

struct geodesic_solver::impl {
  std::shared_ptr<igl_detail::geodesic_state> state;
  std::size_t num_points = 0;
  bool empty = true;
};

geodesic_solver::geodesic_solver() {}

geodesic_solver::geodesic_solver(const geometry &surface, double t_scale) {
  if (!(t_scale > 0) || !std::isfinite(t_scale))
    throw mesh_ops_error("geodesic_solver: t_scale must be positive");
  guarded([&] {
    auto im = std::make_shared<impl>();
    const std::vector<int> F = surface_triangles(surface);
    check_fem_count(F.size() / 3, "geodesic_solver");
    im->state = igl_detail::make_geodesic(positions(surface), F, t_scale);
    im->num_points = surface.num_points();
    im->empty = F.empty();
    _impl = im;
  });
}

bool geodesic_solver::empty() const { return !_impl || _impl->empty; }

std::vector<double> geodesic_solver::distance(const std::vector<index_t> &sources) const {
  if (!_impl)
    return std::vector<double>();
  std::vector<int> src;
  src.reserve(sources.size());
  for (index_t s : sources) {
    if (s >= _impl->num_points)
      throw mesh_ops_error("geodesic_solver::distance: source " + std::to_string(s) +
                           " >= num_points()");
    src.push_back(int(s));
  }
  return guarded([&] {
    std::vector<double> D;
    igl_detail::geodesic_distance(*_impl->state, src, D);
    return D;
  });
}

std::vector<double> geodesic_distance(const geometry &surface,
                                      const std::vector<index_t> &sources) {
  return geodesic_solver(surface).distance(sources);
}

// ---- tetrahedral meshes -------------------------------------------------------------------

std::vector<double> tet_volumes(const geometry &tetmesh) {
  return guarded([&] {
    std::vector<double> vol;
    igl_detail::tet_volumes(positions(tetmesh), tetrahedra(tetmesh), vol);
    return vol;
  });
}

std::size_t orient_tets(geometry &tetmesh) {
  const std::vector<double> vol = tet_volumes(tetmesh);
  std::size_t flipped = 0;
  for (double v : vol)
    flipped += v < 0 ? 1 : 0;
  if (flipped) {
    tets_t &T = tetmesh.tets();
    for (std::size_t i = 0; i < T.size(); ++i)
      if (vol[i] < 0)
        std::swap(T[i][2], T[i][3]);
  }
  return flipped;
}

geometry tet_boundary_surface(const geometry &tetmesh) {
  return guarded([&] {
    const std::vector<double> V = positions(tetmesh);
    std::vector<int> T = tetrahedra(tetmesh);
    std::vector<double> vol;
    igl_detail::tet_volumes(V, T, vol);
    for (std::size_t i = 0; i < vol.size(); ++i)
      if (vol[i] < 0)
        std::swap(T[4 * i + 2], T[4 * i + 3]);
    std::vector<int> B;
    igl_detail::boundary_facets(T, 4, B);

    geometry out;
    const std::size_t n = tetmesh.num_points();
    out.points() = tetmesh.const_points();
    if (tetmesh.const_colors().size() == n)
      out.colors() = tetmesh.const_colors();
    if (tetmesh.const_functions().size() == n)
      out.functions() = tetmesh.const_functions();
    if (tetmesh.const_normals().size() == n)
      out.normals() = tetmesh.const_normals();
    tris_t &F = out.tris();
    F.resize(B.size() / 3);
    for (std::size_t f = 0; f < F.size(); ++f)
      F[f] = {{index_t(B[3 * f]), index_t(B[3 * f + 1]), index_t(B[3 * f + 2])}};
    out.set_geometry_type(geometry::SURFACE_TRI);
    return out;
  });
}

geometry tet_isosurface(const geometry &tetmesh, const std::vector<double> &field,
                        double isovalue) {
  if (field.size() != tetmesh.num_points())
    throw mesh_ops_error("tet_isosurface: field has " + std::to_string(field.size()) +
                         " values for " + std::to_string(tetmesh.num_points()) + " points");
  return guarded([&] { return level_set(tetmesh, field, isovalue); });
}

geometry slice_tets(const geometry &tetmesh, const geometry::vector_t &normal, double offset) {
  if (!(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2] > 0))
    throw mesh_ops_error("slice_tets: zero normal");
  const points_t &P = tetmesh.const_points();
  std::vector<double> S(P.size());
  for (std::size_t i = 0; i < P.size(); ++i)
    S[i] = normal[0] * P[i][0] + normal[1] * P[i][1] + normal[2] * P[i][2] - offset;
  return guarded([&] { return level_set(tetmesh, S, 0.0); });
}

#else // !CVC_ENABLE_LIBIGL

namespace {
[[noreturn]] void unavailable() {
  throw mesh_ops_unavailable("libcvc was built without CVC_ENABLE_LIBIGL");
}
} // namespace

geometry &compute_vertex_normals(geometry &, normal_weighting) { unavailable(); }
std::size_t orient_outward(geometry &) { unavailable(); }
repair_report repair(geometry &, const repair_params &) { unavailable(); }
geometry &compute_curvature(geometry &, const curvature_params &) { unavailable(); }
std::vector<double> vertex_curvature(const geometry &, curvature_kind, const curvature_params &) {
  unavailable();
}
geometry &smooth(geometry &, const smooth_params &) { unavailable(); }

struct mesh_locator::impl {};
mesh_locator::mesh_locator() {}
mesh_locator::mesh_locator(const geometry &) { unavailable(); }
void mesh_locator::build(const geometry &) { unavailable(); }
bool mesh_locator::empty() const { return true; }
std::size_t mesh_locator::num_faces() const { return 0; }
closest_point_result mesh_locator::closest_points(const geometry::points_t &) const {
  unavailable();
}
bool mesh_locator::intersect_ray(const geometry::point_t &, const geometry::vector_t &, ray_hit &,
                                 double, double) const {
  unavailable();
}
std::vector<double> mesh_locator::signed_distance(const geometry::points_t &) const {
  unavailable();
}
std::vector<double> mesh_locator::winding_number(const geometry::points_t &) const {
  unavailable();
}
closest_point_result closest_points(const geometry &, const geometry::points_t &) { unavailable(); }
std::vector<double> signed_distance(const geometry &, const geometry::points_t &) { unavailable(); }

struct geodesic_solver::impl {};
geodesic_solver::geodesic_solver() {}
geodesic_solver::geodesic_solver(const geometry &, double) { unavailable(); }
bool geodesic_solver::empty() const { return true; }
std::vector<double> geodesic_solver::distance(const std::vector<index_t> &) const { unavailable(); }
std::vector<double> geodesic_distance(const geometry &, const std::vector<index_t> &) {
  unavailable();
}

std::vector<double> tet_volumes(const geometry &) { unavailable(); }
std::size_t orient_tets(geometry &) { unavailable(); }
geometry tet_boundary_surface(const geometry &) { unavailable(); }
geometry tet_isosurface(const geometry &, const std::vector<double> &, double) { unavailable(); }
geometry slice_tets(const geometry &, const geometry::vector_t &, double) { unavailable(); }

#endif // CVC_ENABLE_LIBIGL

// ---- colour maps (always available) --------------------------------------------------------

namespace {

struct colormap_name {
  colormap_kind kind;
  const char *name;
};

const colormap_name kColormapNames[] = {
    {colormap_kind::VIRIDIS, "viridis"}, {colormap_kind::MAGMA, "magma"},
    {colormap_kind::PLASMA, "plasma"},   {colormap_kind::INFERNO, "inferno"},
    {colormap_kind::TURBO, "turbo"},     {colormap_kind::JET, "jet"},
    {colormap_kind::PARULA, "parula"},   {colormap_kind::GRAY, "gray"}};

#ifndef CVC_ENABLE_LIBIGL
// 17 evenly spaced samples (8-bit) of libigl's 256-entry tables, for builds
// without libigl. Order: viridis, magma, plasma, inferno, turbo, parula.
// Provenance: derived from libigl's colormap.cpp tables, so they carry the same
// origin as those tables (THIRD_PARTY_NOTICES.md, libigl colormap entry), even
// in a build with CVC_ENABLE_LIBIGL=OFF.
const unsigned char kColormapSamples[6][17][3] = {{{68, 1, 84},
                                                   {72, 24, 106},
                                                   {71, 45, 123},
                                                   {66, 64, 134},
                                                   {59, 82, 139},
                                                   {51, 99, 141},
                                                   {44, 114, 142},
                                                   {38, 130, 142},
                                                   {33, 145, 140},
                                                   {31, 159, 136},
                                                   {39, 173, 129},
                                                   {61, 188, 116},
                                                   {92, 200, 99},
                                                   {129, 211, 77},
                                                   {170, 220, 50},
                                                   {213, 226, 26},
                                                   {253, 231, 37}},
                                                  {{0, 0, 4},
                                                   {10, 8, 34},
                                                   {29, 17, 71},
                                                   {54, 16, 107},
                                                   {81, 18, 124},
                                                   {106, 28, 129},
                                                   {131, 38, 129},
                                                   {156, 46, 127},
                                                   {183, 55, 121},
                                                   {207, 64, 112},
                                                   {229, 80, 100},
                                                   {244, 105, 92},
                                                   {251, 135, 97},
                                                   {254, 165, 113},
                                                   {254, 194, 135},
                                                   {253, 224, 161},
                                                   {252, 253, 191}},
                                                  {{13, 8, 135},
                                                   {49, 5, 151},
                                                   {76, 2, 161},
                                                   {102, 0, 167},
                                                   {126, 3, 168},
                                                   {149, 17, 161},
                                                   {170, 35, 149},
                                                   {188, 53, 135},
                                                   {204, 71, 120},
                                                   {217, 88, 106},
                                                   {229, 107, 93},
                                                   {240, 127, 79},
                                                   {248, 148, 65},
                                                   {253, 171, 51},
                                                   {253, 195, 40},
                                                   {249, 221, 37},
                                                   {240, 249, 33}},
                                                  {{0, 0, 4},
                                                   {11, 7, 36},
                                                   {33, 12, 74},
                                                   {61, 9, 101},
                                                   {87, 16, 110},
                                                   {113, 25, 110},
                                                   {138, 34, 106},
                                                   {163, 44, 97},
                                                   {188, 55, 84},
                                                   {208, 69, 69},
                                                   {227, 89, 51},
                                                   {241, 113, 31},
                                                   {249, 140, 10},
                                                   {252, 170, 15},
                                                   {249, 201, 50},
                                                   {242, 232, 101},
                                                   {252, 255, 164}},
                                                  {{48, 18, 59},
                                                   {64, 64, 162},
                                                   {70, 107, 227},
                                                   {66, 148, 255},
                                                   {40, 188, 235},
                                                   {24, 221, 194},
                                                   {50, 242, 152},
                                                   {109, 254, 98},
                                                   {164, 252, 60},
                                                   {203, 237, 52},
                                                   {236, 209, 58},
                                                   {253, 174, 53},
                                                   {251, 129, 34},
                                                   {236, 83, 15},
                                                   {210, 49, 5},
                                                   {172, 23, 1},
                                                   {122, 4, 3}},
                                                  {{53, 42, 135},
                                                   {50, 67, 185},
                                                   {3, 98, 225},
                                                   {12, 116, 221},
                                                   {20, 132, 212},
                                                   {9, 151, 209},
                                                   {6, 166, 199},
                                                   {19, 176, 182},
                                                   {52, 184, 160},
                                                   {93, 190, 138},
                                                   {138, 191, 118},
                                                   {176, 189, 102},
                                                   {210, 187, 89},
                                                   {242, 185, 73},
                                                   {254, 201, 50},
                                                   {245, 222, 33},
                                                   {249, 251, 14}}};
#endif

int table_index(colormap_kind k) {
  switch (k) {
  case colormap_kind::VIRIDIS:
    return 0;
  case colormap_kind::MAGMA:
    return 1;
  case colormap_kind::PLASMA:
    return 2;
  case colormap_kind::INFERNO:
    return 3;
  case colormap_kind::TURBO:
    return 4;
  case colormap_kind::PARULA:
    return 5;
  default:
    return -1;
  }
}

// f in [0, 1] -> rgb in [0, 1].
void colormap_value(colormap_kind k, double f, double rgb[3]) {
  if (k == colormap_kind::GRAY) {
    rgb[0] = rgb[1] = rgb[2] = f;
    return;
  }
  if (k == colormap_kind::JET) {
    // MATLAB's jet (libigl's JET entry renders turbo instead).
    rgb[0] = std::clamp(1.5 - std::abs(4.0 * f - 3.0), 0.0, 1.0);
    rgb[1] = std::clamp(1.5 - std::abs(4.0 * f - 2.0), 0.0, 1.0);
    rgb[2] = std::clamp(1.5 - std::abs(4.0 * f - 1.0), 0.0, 1.0);
    return;
  }
  const int t = table_index(k);
#ifdef CVC_ENABLE_LIBIGL
  igl_detail::colormap(t, f, rgb);
#else
  const double x = f * 16.0;
  const int i = std::min(int(x), 15);
  const double w = x - i;
  for (int c = 0; c < 3; ++c)
    rgb[c] = ((1.0 - w) * kColormapSamples[t][i][c] + w * kColormapSamples[t][i + 1][c]) / 255.0;
#endif
}

} // namespace

bool colormap_from_string(const std::string &name, colormap_kind &k) {
  std::string lower(name);
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  for (const colormap_name &e : kColormapNames)
    if (lower == e.name) {
      k = e.kind;
      return true;
    }
  return false;
}

std::string to_string(colormap_kind k) {
  for (const colormap_name &e : kColormapNames)
    if (e.kind == k)
      return e.name;
  return "unknown";
}

std::vector<unsigned char> colormap_rgb(const std::vector<double> &values, colormap_kind k,
                                        double lo, double hi) {
  double vmin = std::numeric_limits<double>::infinity(), vmax = -vmin;
  for (double v : values)
    if (std::isfinite(v)) {
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
    }
  if (std::isnan(lo))
    lo = vmin;
  if (std::isnan(hi))
    hi = vmax;
  std::vector<unsigned char> out(3 * values.size(), 128);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i]))
      continue;
    double f = hi != lo ? (values[i] - lo) / (hi - lo) : 0.5;
    f = std::isnan(f) ? 0.5 : std::clamp(f, 0.0, 1.0);
    double rgb[3];
    colormap_value(k, f, rgb);
    for (int c = 0; c < 3; ++c)
      out[3 * i + c] = (unsigned char)std::lround(std::clamp(rgb[c], 0.0, 1.0) * 255.0);
  }
  return out;
}

std::pair<double, double> robust_range(const std::vector<double> &values, double lo_pct,
                                       double hi_pct) {
  std::vector<double> v;
  v.reserve(values.size());
  for (double x : values)
    if (std::isfinite(x))
      v.push_back(x);
  if (v.empty())
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  lo_pct = std::isnan(lo_pct) ? 0.0 : std::clamp(lo_pct, 0.0, 1.0);
  hi_pct = std::isnan(hi_pct) ? 1.0 : std::clamp(hi_pct, 0.0, 1.0);
  if (lo_pct > hi_pct)
    std::swap(lo_pct, hi_pct);
  const auto at = [&v](double pct) {
    const std::size_t k = std::size_t(std::floor(pct * double(v.size() - 1) + 0.5));
    std::nth_element(v.begin(), v.begin() + std::ptrdiff_t(k), v.end());
    return v[k];
  };
  const double a = at(lo_pct);
  const double b = at(hi_pct);
  return {a, b};
}

} // namespace cvc

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

// fem.cpp -- the cvc::geometry side of fem.h: element selection (domain,
// degenerate elements, tet orientation), boundary conditions and argument
// checks. Assembly and solves happen behind igl/igl_detail.h; like
// mesh_ops.cpp this TU never sees Eigen or libigl.

#include <algorithm>
#include <cmath>
#include <cvc/geometry/fem.h>
#include <numeric>

#ifdef CVC_ENABLE_LIBIGL
#include "igl/igl_bridge.h"
#include "igl/igl_detail.h"
#endif

namespace cvc {
namespace fem {

#ifdef CVC_ENABLE_LIBIGL

using igl_bridge::guarded;

namespace {

// The elements of a mesh, as fem.h selects them.
struct element_set {
  std::vector<double> V;
  int simplex_size = 3;
  std::size_t nv = 0;
  std::vector<int> all;           // every element, in element order
  std::vector<int> kept;          // the non-degenerate ones; tets positively oriented
  std::vector<std::size_t> which; // position in `all` of each kept element
  std::vector<char> active;       // vertex is a corner of a kept element
};

element_set gather(const geometry &mesh, domain d) {
  if (mesh.num_points() == 0)
    throw mesh_ops_error("fem: the mesh has no points");
  element_set es;
  es.V = igl_bridge::positions(mesh);
  es.nv = mesh.num_points();
  const bool volume = d == domain::VOLUME || (d == domain::AUTO && !mesh.const_tets().empty());
  es.simplex_size = volume ? 4 : 3;
  es.all = volume ? igl_bridge::tetrahedra(mesh) : igl_bridge::surface_triangles(mesh);
  if (es.all.empty())
    throw mesh_ops_error(volume ? "fem: the mesh has no tets" : "fem: the mesh has no triangles");
  igl_bridge::check_fem_count(es.all.size() / std::size_t(es.simplex_size), "fem");

  // Degenerate: |volume| (or area) below 1e-12 of the bounding-box scale, or
  // too flat for igl::cotmatrix's edge-length formulas (see fem.h).
  const double diag = igl_bridge::diagonal(es.V);
  const int ss = es.simplex_size;
  std::vector<double> measure;
  double tol;
  if (volume) {
    igl_detail::tet_volumes(es.V, es.all, measure);
    tol = 1e-12 * diag * diag * diag;
  } else {
    igl_detail::double_areas(es.V, es.all, measure);
    for (double &a : measure)
      a *= 0.5;
    tol = 1e-12 * diag * diag;
  }
  std::vector<char> shaped;
  igl_detail::well_shaped(es.V, es.all, ss, shaped);
  es.active.assign(es.nv, 0);
  for (std::size_t e = 0; e < measure.size(); ++e) {
    const double m = std::abs(measure[e]);
    if (!(m > 0) || m < tol || !std::isfinite(m) || !shaped[e])
      continue;
    const std::size_t at = es.kept.size();
    es.kept.insert(es.kept.end(), es.all.begin() + std::ptrdiff_t(ss * e),
                   es.all.begin() + std::ptrdiff_t(ss * (e + 1)));
    if (volume && measure[e] < 0)
      std::swap(es.kept[at + 2], es.kept[at + 3]);
    es.which.push_back(e);
    for (int c = 0; c < ss; ++c)
      es.active[std::size_t(es.all[ss * e + c])] = 1;
  }
  return es;
}

sparse_matrix to_sparse(const igl_detail::triplets &t, std::size_t n) {
  sparse_matrix s;
  s.rows = s.cols = n;
  s.row.assign(t.row.begin(), t.row.end());
  s.col.assign(t.col.begin(), t.col.end());
  s.value = t.val;
  return s;
}

// Known vertices of a solve: the Dirichlet ones (first value wins on a repeat),
// then every vertex no kept element touches, held at 0.
struct constraints {
  std::vector<int> known;
  std::vector<double> values;
};

constraints make_constraints(const element_set &es, const dirichlet &bc) {
  if (!bc.vertices.empty() && bc.values.size() != 1 && bc.values.size() != bc.vertices.size())
    throw mesh_ops_error("fem: dirichlet needs one value, or one per vertex");
  constraints c;
  std::vector<char> seen(es.nv, 0);
  for (std::size_t i = 0; i < bc.vertices.size(); ++i) {
    const index_t v = bc.vertices[i];
    if (v >= es.nv)
      throw mesh_ops_error("fem: dirichlet vertex " + std::to_string(v) + " >= num_points()");
    if (seen[v])
      continue;
    seen[v] = 1;
    c.known.push_back(int(v));
    c.values.push_back(bc.values.size() == 1 ? bc.values[0] : bc.values[i]);
  }
  for (std::size_t v = 0; v < es.nv; ++v)
    if (!es.active[v] && !seen[v]) {
      c.known.push_back(int(v));
      c.values.push_back(0.0);
    }
  return c;
}

// K alone is singular on any connected part with no Dirichlet vertex.
void require_anchored(const element_set &es, const dirichlet &bc) {
  std::vector<int> parent(es.nv);
  std::iota(parent.begin(), parent.end(), 0);
  const auto root = [&parent](int i) {
    while (parent[std::size_t(i)] != i)
      i = parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
    return i;
  };
  const int ss = es.simplex_size;
  for (std::size_t e = 0; e < es.kept.size(); e += std::size_t(ss))
    for (int c = 1; c < ss; ++c) {
      const int a = root(es.kept[e]), b = root(es.kept[e + std::size_t(c)]);
      if (a != b)
        parent[std::size_t(std::max(a, b))] = std::min(a, b);
    }
  std::vector<char> anchored(es.nv, 0);
  for (index_t v : bc.vertices)
    anchored[std::size_t(root(int(v)))] = 1;
  for (std::size_t v = 0; v < es.nv; ++v)
    if (es.active[v] && !anchored[std::size_t(root(int(v)))])
      throw mesh_ops_error("fem: a connected part of the mesh has no Dirichlet vertex "
                           "(the problem would be singular)");
}

} // namespace

std::vector<index_t> boundary_vertices(const geometry &mesh, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    std::vector<int> B;
    igl_detail::boundary_facets(es.all, es.simplex_size, B);
    std::vector<index_t> out(B.begin(), B.end());
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  });
}

sparse_matrix stiffness_matrix(const geometry &mesh, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    igl_detail::triplets K;
    igl_detail::stiffness(es.V, es.kept, es.simplex_size, K);
    return to_sparse(K, es.nv);
  });
}

sparse_matrix mass_matrix(const geometry &mesh, domain d, bool lumped) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    igl_detail::triplets M;
    igl_detail::mass(es.V, es.kept, es.simplex_size, lumped, M);
    return to_sparse(M, es.nv);
  });
}

std::vector<double> gradient(const geometry &mesh, const std::vector<double> &u, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    if (u.size() != es.nv)
      throw mesh_ops_error("fem::gradient: u needs one value per point");
    std::vector<double> g;
    igl_detail::gradient(es.V, es.kept, es.simplex_size, u, g);
    // Degenerate elements have no gradient; report 0 for them.
    std::vector<double> out(3 * (es.all.size() / std::size_t(es.simplex_size)), 0.0);
    for (std::size_t k = 0; k < es.which.size(); ++k)
      for (int c = 0; c < 3; ++c)
        out[3 * es.which[k] + std::size_t(c)] = g[3 * k + std::size_t(c)];
    return out;
  });
}

std::vector<double> solve_poisson(const geometry &mesh, const std::vector<double> &f,
                                  const dirichlet &bc, domain d) {
  if (bc.vertices.empty())
    throw mesh_ops_error("fem::solve_poisson: no Dirichlet condition (the problem is singular)");
  return guarded([&] {
    const element_set es = gather(mesh, d);
    std::vector<double> rhs;
    if (f.size() == 1)
      rhs.assign(es.nv, f[0]);
    else if (f.size() == es.nv)
      rhs = f;
    else
      throw mesh_ops_error("fem::solve_poisson: f needs one value, or one per point");
    const constraints c = make_constraints(es, bc);
    require_anchored(es, bc);
    const auto sys =
        igl_detail::make_quadratic_system(es.V, es.kept, es.simplex_size, 0.0, 1.0, c.known);
    std::vector<double> u;
    igl_detail::solve_quadratic(*sys, rhs, c.values, u);
    return u;
  });
}

std::vector<double> solve_laplace(const geometry &mesh, const dirichlet &bc, domain d) {
  if (bc.vertices.empty())
    throw mesh_ops_error("fem::solve_laplace: no Dirichlet condition (the problem is singular)");
  return solve_poisson(mesh, std::vector<double>(1, 0.0), bc, d);
}

std::vector<double> solve_heat(const geometry &mesh, const std::vector<double> &u0,
                               const heat_params &p, const dirichlet &bc, domain d,
                               const heat_progress &progress) {
  if (!(p.dt > 0) || !std::isfinite(p.dt) || !(p.kappa >= 0) || !std::isfinite(p.kappa))
    throw mesh_ops_error("fem::solve_heat: dt must be positive and kappa non-negative");
  return guarded([&] {
    const element_set es = gather(mesh, d);
    if (u0.size() != es.nv)
      throw mesh_ops_error("fem::solve_heat: u0 needs one value per point");
    if (p.steps <= 0)
      return u0;
    const constraints c = make_constraints(es, bc);
    // Backward Euler, (M + dt kappa K) u+ = M u: one factorization for all steps.
    const auto sys = igl_detail::make_quadratic_system(es.V, es.kept, es.simplex_size, 1.0,
                                                       p.dt * p.kappa, c.known);
    std::vector<double> u = u0, next;
    for (int step = 1; step <= p.steps; ++step) {
      igl_detail::solve_quadratic(*sys, u, c.values, next);
      u.swap(next);
      if (progress && !progress(step, u))
        break;
    }
    return u;
  });
}

eigen_result laplacian_eigenmodes(const geometry &mesh, int k, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    eigen_result r;
    igl_detail::eigenmodes(es.V, es.kept, es.simplex_size, es.active, k, r.values, r.vectors);
    return r;
  });
}

#else // !CVC_ENABLE_LIBIGL

namespace {
[[noreturn]] void unavailable() {
  throw mesh_ops_unavailable("libcvc was built without CVC_ENABLE_LIBIGL");
}
} // namespace

std::vector<index_t> boundary_vertices(const geometry &, domain) { unavailable(); }
sparse_matrix stiffness_matrix(const geometry &, domain) { unavailable(); }
sparse_matrix mass_matrix(const geometry &, domain, bool) { unavailable(); }
std::vector<double> gradient(const geometry &, const std::vector<double> &, domain) {
  unavailable();
}
std::vector<double> solve_poisson(const geometry &, const std::vector<double> &, const dirichlet &,
                                  domain) {
  unavailable();
}
std::vector<double> solve_laplace(const geometry &, const dirichlet &, domain) { unavailable(); }
std::vector<double> solve_heat(const geometry &, const std::vector<double> &, const heat_params &,
                               const dirichlet &, domain, const heat_progress &) {
  unavailable();
}
eigen_result laplacian_eigenmodes(const geometry &, int, domain) { unavailable(); }

#endif // CVC_ENABLE_LIBIGL

} // namespace fem
} // namespace cvc

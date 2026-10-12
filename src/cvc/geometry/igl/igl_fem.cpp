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

// igl_fem.cpp -- P1 finite-element operators and solves (igl::cotmatrix,
// igl::massmatrix, igl::grad, igl::min_quad_with_fixed), Laplacian eigenmodes,
// and heat-method geodesics (igl::heat_geodesics) behind igl_detail.h.

#include "igl_eigen.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <igl/avg_edge_length.h>
#include <igl/boundary_facets.h>
#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/heat_geodesics.h>
#include <igl/massmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <limits>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

namespace cvc {
namespace igl_detail {
namespace {

typedef Eigen::SparseMatrix<double> SpMat;

// Up to this many active vertices laplacian_eigenmodes solves the dense
// problem (~0.1 s at 400; ~1 s already at 700), above it iterates.
const int kDenseEigenLimit = 400;

SpMat stiffness_op(const Eigen::MatrixXd &V, const Eigen::MatrixXi &E) {
  SpMat L;
  igl::cotmatrix(V, E, L);
  pad_square(L, int(V.rows()));
  return -L;
}

SpMat mass_op(const Eigen::MatrixXd &V, const Eigen::MatrixXi &E, igl::MassMatrixType type) {
  SpMat M;
  igl::massmatrix(V, E, type, M);
  pad_square(M, int(V.rows()));
  return M;
}

igl::MassMatrixType lumped(int simplex_size) {
  return simplex_size == 3 ? igl::MASSMATRIX_TYPE_VORONOI : igl::MASSMATRIX_TYPE_BARYCENTRIC;
}

// Deterministic start vectors in [-1, 1] (splitmix64).
double hash_unit(std::uint64_t i) {
  std::uint64_t z = i + 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  z ^= z >> 31;
  return double(z >> 11) * (2.0 / 9007199254740992.0) - 1.0;
}

// Small problems: K x = l M x with diagonal M is the standard symmetric problem
// D^-1/2 K D^-1/2 y = l y, x = D^-1/2 y, which comes out M-orthonormal.
void dense_modes(const SpMat &K, const Eigen::VectorXd &m, int k, Eigen::VectorXd &S,
                 Eigen::MatrixXd &U) {
  const Eigen::VectorXd s = m.cwiseSqrt().cwiseInverse();
  const Eigen::MatrixXd A = s.asDiagonal() * Eigen::MatrixXd(K) * s.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  if (es.info() != Eigen::Success)
    throw failure("laplacian_eigenmodes: dense eigensolver failed");
  S = es.eigenvalues().head(k);
  U = s.asDiagonal() * es.eigenvectors().leftCols(k);
}

// M-orthonormalise the columns of Y (modified Gram-Schmidt, applied twice). A
// column that collapses into the span of the previous ones is restarted.
void m_orthonormalize(Eigen::MatrixXd &Y, const Eigen::VectorXd &m, std::uint64_t &seed) {
  for (Eigen::Index j = 0; j < Y.cols(); ++j)
    for (int attempt = 0;; ++attempt) {
      const double before = std::sqrt(Y.col(j).dot(m.asDiagonal() * Y.col(j)));
      for (int pass = 0; pass < 2; ++pass)
        for (Eigen::Index i = 0; i < j; ++i)
          Y.col(j) -= Y.col(i) * Y.col(i).dot(m.asDiagonal() * Y.col(j));
      const double nrm = std::sqrt(Y.col(j).dot(m.asDiagonal() * Y.col(j)));
      if (nrm > 1e-10 * before && std::isfinite(nrm)) {
        Y.col(j) /= nrm;
        break;
      }
      if (attempt == 3)
        throw failure("laplacian_eigenmodes: subspace iteration broke down");
      for (Eigen::Index r = 0; r < Y.rows(); ++r)
        Y(r, j) = hash_unit(seed++);
    }
}

// Large problems: shift-invert subspace iteration with Rayleigh-Ritz on a
// 2k-wide block, robust to repeated eigenvalues. igl::eigs is not used: its
// deflated power iteration hands back repeated, non-orthogonal modes for a
// degenerate spectrum (every sphere) and takes ~6x longer (10242-vertex
// sphere, k = 10: 11.6 s and wrong, against 1.8 s here).
void subspace_modes(const SpMat &K, const Eigen::VectorXd &m, int k, Eigen::VectorXd &S,
                    Eigen::MatrixXd &U) {
  const Eigen::Index n = K.rows();
  const Eigen::Index p = std::min<Eigen::Index>(n, std::max(2 * k, k + 8));
  double lmax = 0;
  for (Eigen::Index i = 0; i < n; ++i)
    lmax = std::max(lmax, std::abs(K.coeff(i, i)) / m(i));
  SpMat C = K;
  for (Eigen::Index i = 0; i < n; ++i)
    C.coeffRef(i, i) += 1e-6 * lmax * m(i);
  Eigen::SimplicialLDLT<SpMat> ldlt(C);
  if (ldlt.info() != Eigen::Success)
    throw failure("laplacian_eigenmodes: factorization failed");
  std::uint64_t seed = 0;
  Eigen::MatrixXd X(n, p);
  for (Eigen::Index j = 0; j < p; ++j)
    for (Eigen::Index i = 0; i < n; ++i)
      X(i, j) = hash_unit(seed++);
  Eigen::VectorXd prev = Eigen::VectorXd::Constant(k, std::numeric_limits<double>::infinity());
  Eigen::VectorXd theta;
  // Converged when no wanted Ritz value moves by more than 1e-12 of the largest
  // one, or 1e-13 of the spectrum's scale lmax: the zero modes (one per
  // connected part) only ever settle to round-off, ~1e-16 lmax, so with k at
  // most the number of parts a purely relative test is never met.
  bool converged = false;
  for (int it = 0; it < 500 && !converged; ++it) {
    Eigen::MatrixXd Y = ldlt.solve(m.asDiagonal() * X);
    m_orthonormalize(Y, m, seed);
    const Eigen::MatrixXd R = Y.transpose() * (K * Y);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (R + R.transpose()));
    if (es.info() != Eigen::Success || !es.eigenvalues().allFinite())
      throw failure("laplacian_eigenmodes: Rayleigh-Ritz eigensolver failed");
    theta = es.eigenvalues();
    X = Y * es.eigenvectors();
    const double change = (theta.head(k) - prev).cwiseAbs().maxCoeff();
    converged = change <= 1e-12 * std::abs(theta(k - 1)) + 1e-13 * lmax;
    prev = theta.head(k);
  }
  if (!converged)
    throw failure("laplacian_eigenmodes: the subspace iteration did not converge in 500 "
                  "iterations (a very clustered spectrum? ask for fewer modes)");
  S = theta.head(k);
  U = X.leftCols(k);
}

} // namespace

void well_shaped(const dvec &Vp, const ivec &Ep, int simplex_size, mask &keep) {
  const Eigen::MatrixXi E = to_matrix(Ep, simplex_size);
  keep.assign(std::size_t(E.rows()), 0);
  if (E.rows() == 0)
    return;
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  for (Eigen::Index e = 0; e < E.rows(); ++e)
    keep[std::size_t(e)] = simplex_size == 3 ? cot_safe_triangle(V, E(e, 0), E(e, 1), E(e, 2))
                                             : cot_safe_tet(V, E(e, 0), E(e, 1), E(e, 2), E(e, 3));
}

void stiffness(const dvec &V, const ivec &E, int simplex_size, triplets &K) {
  if (E.empty()) {
    K = triplets();
    return;
  }
  to_triplets(stiffness_op(to_matrix(V, 3), to_matrix(E, simplex_size)), K);
}

void mass(const dvec &V, const ivec &E, int simplex_size, bool lumped_mass, triplets &M) {
  if (E.empty()) {
    M = triplets();
    return;
  }
  to_triplets(mass_op(to_matrix(V, 3), to_matrix(E, simplex_size),
                      lumped_mass ? lumped(simplex_size) : igl::MASSMATRIX_TYPE_FULL),
              M);
}

void gradient(const dvec &Vp, const ivec &Ep, int simplex_size, const dvec &u, dvec &G) {
  const Eigen::MatrixXi E = to_matrix(Ep, simplex_size);
  const Eigen::Index m = E.rows();
  G.assign(std::size_t(3 * m), 0.0);
  if (m == 0)
    return;
  SpMat Gop;
  igl::grad(to_matrix(Vp, 3), E, Gop);
  const Eigen::VectorXd g = Gop * to_vector(u);
  // igl::grad stacks the x components of all elements, then y, then z.
  for (Eigen::Index e = 0; e < m; ++e)
    for (int d = 0; d < 3; ++d)
      G[std::size_t(3 * e + d)] = g(d * m + e);
}

struct quadratic_system {
  Eigen::Index n = 0;
  SpMat M;
  Eigen::VectorXi known;
  bool all_known = false;
  igl::min_quad_with_fixed_data<double> data;
};

std::shared_ptr<quadratic_system> make_quadratic_system(const dvec &Vp, const ivec &Ep,
                                                        int simplex_size, double mass_coef,
                                                        double stiff_coef, const ivec &known) {
  auto s = std::make_shared<quadratic_system>();
  const int nv = int(Vp.size() / 3);
  s->n = nv;
  s->M.resize(nv, nv);
  SpMat A(nv, nv);
  if (!Ep.empty()) {
    const Eigen::MatrixXd V = to_matrix(Vp, 3);
    const Eigen::MatrixXi E = to_matrix(Ep, simplex_size);
    s->M = mass_op(V, E, lumped(simplex_size));
    A = stiff_coef * stiffness_op(V, E) + mass_coef * s->M;
  }
  s->known = known.empty() ? Eigen::VectorXi(0)
                           : Eigen::VectorXi(Eigen::Map<const Eigen::VectorXi>(
                                 known.data(), Eigen::Index(known.size())));
  s->all_known = Eigen::Index(known.size()) >= s->n;
  if (s->all_known)
    return s;
  if (!igl::min_quad_with_fixed_precompute(A, s->known, SpMat(), true, s->data))
    throw failure("the system is singular: is every connected part of the mesh held by a "
                  "Dirichlet condition?");
  return s;
}

void solve_quadratic(const quadratic_system &s, const dvec &f, const dvec &Y, dvec &u) {
  const Eigen::VectorXd Yv = to_vector(Y);
  Eigen::VectorXd Z;
  if (s.all_known) {
    Z = Eigen::VectorXd::Zero(s.n);
    for (Eigen::Index i = 0; i < s.known.size(); ++i)
      Z(s.known(i)) = Yv(i);
  } else {
    const Eigen::VectorXd B = -(s.M * to_vector(f));
    igl::min_quad_with_fixed_solve(s.data, B, Yv, Eigen::VectorXd(), Z);
  }
  if (!Z.allFinite())
    throw failure("the solve produced non-finite values (singular system?)");
  from_vector(Z, u);
}

void eigenmodes(const dvec &Vp, const ivec &Ep, int simplex_size, const mask &active, int k,
                dvec &values, std::vector<dvec> &vectors) {
  const int nv = int(Vp.size() / 3);
  values.clear();
  vectors.clear();
  std::vector<int> local(std::size_t(nv), -1), global;
  for (int i = 0; i < nv; ++i)
    if (active[std::size_t(i)]) {
      local[std::size_t(i)] = int(global.size());
      global.push_back(i);
    }
  const int na = int(global.size());
  if (na == 0 || k <= 0 || Ep.empty())
    return;
  k = std::min(k, na);
  Eigen::MatrixXd V(na, 3);
  for (int j = 0; j < na; ++j)
    V.row(j) << Vp[3 * std::size_t(global[std::size_t(j)])],
        Vp[3 * std::size_t(global[std::size_t(j)]) + 1],
        Vp[3 * std::size_t(global[std::size_t(j)]) + 2];
  Eigen::MatrixXi E = to_matrix(Ep, simplex_size);
  for (Eigen::Index i = 0; i < E.size(); ++i)
    E.data()[i] = local[std::size_t(E.data()[i])];
  const SpMat K = stiffness_op(V, E);
  const SpMat M = mass_op(V, E, lumped(simplex_size));
  const Eigen::VectorXd m = M.diagonal();
  if (!(m.minCoeff() > 0))
    throw failure("laplacian_eigenmodes: a vertex has no mass");

  Eigen::VectorXd S;
  Eigen::MatrixXd U;
  if (na <= kDenseEigenLimit)
    dense_modes(K, m, k, S, U);
  else
    subspace_modes(K, m, k, S, U);

  values.assign(S.data(), S.data() + S.size());
  vectors.assign(std::size_t(k), dvec(std::size_t(nv), 0.0));
  for (int i = 0; i < k; ++i) {
    // Fix the arbitrary sign: the largest-magnitude entry is positive.
    Eigen::Index big = 0;
    U.col(i).cwiseAbs().maxCoeff(&big);
    const double sign = U(big, i) < 0 ? -1.0 : 1.0;
    for (int j = 0; j < na; ++j)
      vectors[std::size_t(i)][std::size_t(global[std::size_t(j)])] = sign * U(j, i);
  }
}

// ---- heat-method geodesics ----------------------------------------------------

struct geodesic_component {
  std::vector<int> global; // component vertex -> mesh vertex
  // Heat method, unless every vertex of the component is a boundary vertex:
  // libigl's Dirichlet solve would then have no unknown (a failed assertion
  // in min_quad_with_fixed), and the heat method has nothing to smooth over
  // anyway. Such a component (an isolated triangle, a one-triangle-wide strip)
  // gets exact shortest paths along its edges instead.
  bool along_edges = false;
  igl::HeatGeodesicsData<double> data;
  std::vector<std::vector<std::pair<int, double>>> edges; // along_edges: (vertex, length)
};

namespace {
int find_root(std::vector<int> &parent, int i) {
  while (parent[std::size_t(i)] != i)
    i = parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
  return i;
}
} // namespace

struct geodesic_state {
  int nv = 0;
  std::vector<int> component; // per vertex; -1 when in no triangle
  std::vector<int> local;     // per vertex, its index within its component
  std::vector<std::unique_ptr<geodesic_component>> parts;
};

std::shared_ptr<geodesic_state> make_geodesic(const dvec &Vp, const ivec &Fp, double t_scale) {
  auto s = std::make_shared<geodesic_state>();
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi F0 = to_matrix(Fp, 3);
  s->nv = int(V.rows());
  s->component.assign(std::size_t(s->nv), -1);
  s->local.assign(std::size_t(s->nv), -1);
  if (F0.rows() == 0)
    return s;
  // Triangles igl::cotmatrix cannot weigh (zero area, or so flat that their
  // edge-length area is noise: see cot_safe_triangle) are left out. One such
  // triangle would otherwise turn its whole component's distances into NaN.
  const Eigen::MatrixXi F = cot_safe_faces(V, F0);
  if (F.rows() == 0)
    return s;
  const double h = igl::avg_edge_length(V, F);
  const double t = t_scale * h * h;
  if (!(t > 0) || !std::isfinite(t))
    throw failure("geodesic_solver: degenerate heat time");

  // Connected components through shared vertices, each with its own solver:
  // the Neumann/Poisson systems are singular across disconnected pieces.
  std::vector<int> parent(std::size_t(s->nv));
  std::iota(parent.begin(), parent.end(), 0);
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    for (int c = 1; c < 3; ++c) {
      const int a = find_root(parent, F(f, 0)), b = find_root(parent, F(f, c));
      if (a != b)
        parent[std::size_t(std::max(a, b))] = std::min(a, b);
    }
  std::vector<int> part_of_root(std::size_t(s->nv), -1);
  std::vector<std::vector<int>> part_faces;
  for (Eigen::Index f = 0; f < F.rows(); ++f) {
    const int r = find_root(parent, F(f, 0));
    if (part_of_root[std::size_t(r)] < 0) {
      part_of_root[std::size_t(r)] = int(part_faces.size());
      part_faces.emplace_back();
      s->parts.emplace_back(new geodesic_component);
    }
    part_faces[std::size_t(part_of_root[std::size_t(r)])].push_back(int(f));
  }
  for (std::size_t p = 0; p < s->parts.size(); ++p) {
    geodesic_component &gc = *s->parts[p];
    Eigen::MatrixXi Fc(Eigen::Index(part_faces[p].size()), 3);
    for (std::size_t i = 0; i < part_faces[p].size(); ++i)
      for (int c = 0; c < 3; ++c) {
        const int v = F(part_faces[p][i], c);
        if (s->component[std::size_t(v)] < 0) {
          s->component[std::size_t(v)] = int(p);
          s->local[std::size_t(v)] = int(gc.global.size());
          gc.global.push_back(v);
        }
        Fc(Eigen::Index(i), c) = s->local[std::size_t(v)];
      }
    const Eigen::Index nc = Eigen::Index(gc.global.size());
    Eigen::MatrixXd Vc(nc, 3);
    for (std::size_t j = 0; j < gc.global.size(); ++j)
      Vc.row(Eigen::Index(j)) = V.row(gc.global[j]);
    Eigen::MatrixXi O;
    igl::boundary_facets(Fc, O);
    std::vector<char> rim(std::size_t(nc), 0);
    for (Eigen::Index i = 0; i < O.size(); ++i)
      rim[std::size_t(O.data()[i])] = 1;
    gc.along_edges = std::find(rim.begin(), rim.end(), char(0)) == rim.end();
    if (gc.along_edges) {
      gc.edges.assign(std::size_t(nc), std::vector<std::pair<int, double>>());
      for (Eigen::Index f = 0; f < Fc.rows(); ++f)
        for (int c = 0; c < 3; ++c) {
          const int a = Fc(f, c), b = Fc(f, (c + 1) % 3);
          const double l = (Vc.row(a) - Vc.row(b)).norm();
          gc.edges[std::size_t(a)].emplace_back(b, l);
          gc.edges[std::size_t(b)].emplace_back(a, l);
        }
    } else if (!igl::heat_geodesics_precompute(Vc, Fc, t, gc.data)) {
      throw failure("geodesic_solver: factorization failed");
    }
  }
  return s;
}

namespace {
// Multi-source Dijkstra along a component's edges.
void edge_distances(const geodesic_component &gc, const std::vector<int> &sources,
                    Eigen::VectorXd &d) {
  d = Eigen::VectorXd::Constant(Eigen::Index(gc.edges.size()),
                                std::numeric_limits<double>::infinity());
  typedef std::pair<double, int> entry;
  std::priority_queue<entry, std::vector<entry>, std::greater<entry>> heap;
  for (int v : sources) {
    d(v) = 0;
    heap.emplace(0.0, v);
  }
  while (!heap.empty()) {
    const entry top = heap.top();
    heap.pop();
    if (top.first > d(top.second))
      continue;
    for (const auto &e : gc.edges[std::size_t(top.second)])
      if (top.first + e.second < d(e.first)) {
        d(e.first) = top.first + e.second;
        heap.emplace(d(e.first), e.first);
      }
  }
}
} // namespace

void geodesic_distance(const geodesic_state &s, const ivec &sources, dvec &D) {
  D.assign(std::size_t(s.nv), std::numeric_limits<double>::infinity());
  std::vector<std::vector<int>> gamma(s.parts.size());
  for (int v : sources)
    if (s.component[std::size_t(v)] >= 0)
      gamma[std::size_t(s.component[std::size_t(v)])].push_back(s.local[std::size_t(v)]);
  for (std::size_t p = 0; p < s.parts.size(); ++p) {
    if (gamma[p].empty())
      continue;
    const geodesic_component &gc = *s.parts[p];
    Eigen::VectorXd d;
    if (gc.along_edges) {
      edge_distances(gc, gamma[p], d);
    } else {
      const Eigen::VectorXi g =
          Eigen::Map<const Eigen::VectorXi>(gamma[p].data(), Eigen::Index(gamma[p].size()));
      igl::heat_geodesics_solve(gc.data, g, d);
    }
    // Eigen's LDLT does not reject a NaN pivot: report a broken solve rather
    // than hand back NaN distances.
    if (!d.allFinite())
      throw failure("geodesic_distance: the heat method produced a non-finite distance");
    for (std::size_t j = 0; j < gc.global.size(); ++j)
      D[std::size_t(gc.global[j])] = std::max(d(Eigen::Index(j)), 0.0);
  }
  for (int v : sources)
    D[std::size_t(v)] = 0.0;
}

} // namespace igl_detail
} // namespace cvc

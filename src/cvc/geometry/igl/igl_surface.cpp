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

// igl_surface.cpp -- libigl triangle-surface operators behind igl_detail.h:
// normals, orientation, welding, curvature, smoothing and colour tables.

#include "igl_eigen.h"

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <igl/adjacency_list.h>
#include <igl/bfs_orient.h>
#include <igl/boundary_facets.h>
#include <igl/colormap.h>
#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/orient_outward.h>
#include <igl/per_vertex_normals.h>
#include <limits>
#include <utility>
#include <vector>

namespace cvc {
namespace igl_detail {
namespace {

// The rows of F whose three corners are distinct.
Eigen::MatrixXi proper_faces(const Eigen::MatrixXi &F) {
  Eigen::MatrixXi out(F.rows(), 3);
  Eigen::Index m = 0;
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    if (F(f, 0) != F(f, 1) && F(f, 1) != F(f, 2) && F(f, 0) != F(f, 2))
      out.row(m++) = F.row(f);
  out.conservativeResize(m, 3);
  return out;
}

// The rows of F with a positive, finite area (the ones that have a normal).
Eigen::MatrixXi faces_with_area(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
  if (F.rows() == 0)
    return F;
  Eigen::VectorXd dblA;
  igl::doublearea(V, F, dblA);
  Eigen::MatrixXi out(F.rows(), 3);
  Eigen::Index m = 0;
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    if (std::isfinite(dblA(f)) && dblA(f) > 0)
      out.row(m++) = F.row(f);
  out.conservativeResize(m, 3);
  return out;
}

Eigen::VectorXd voronoi_area(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
  Eigen::SparseMatrix<double> M;
  igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);
  pad_square(M, int(V.rows()));
  return M.diagonal();
}

std::vector<char> referenced(const Eigen::MatrixXi &F, Eigen::Index n) {
  std::vector<char> r(std::size_t(n), 0);
  for (Eigen::Index i = 0; i < F.size(); ++i)
    r[std::size_t(F.data()[i])] = 1;
  return r;
}

// The vertices on an edge that exactly one triangle of F has.
std::vector<char> on_boundary(const Eigen::MatrixXi &F, Eigen::Index n) {
  std::vector<char> b(std::size_t(n), 0);
  if (F.rows() == 0)
    return b;
  Eigen::MatrixXi E;
  igl::boundary_facets(F, E);
  for (Eigen::Index i = 0; i < E.size(); ++i)
    b[std::size_t(E.data()[i])] = 1;
  return b;
}

int find_root(std::vector<int> &parent, int i) {
  while (parent[std::size_t(i)] != i)
    i = parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
  return i;
}

// Union by smallest index: every class's root is its smallest member.
void join(std::vector<int> &parent, int a, int b) {
  a = find_root(parent, a);
  b = find_root(parent, b);
  if (a != b)
    parent[std::size_t(std::max(a, b))] = std::min(a, b);
}

double bounding_diagonal(const Eigen::MatrixXd &V, const std::vector<char> &ref) {
  Eigen::RowVector3d lo = Eigen::RowVector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::RowVector3d hi = -lo;
  for (Eigen::Index i = 0; i < V.rows(); ++i)
    if (ref[std::size_t(i)]) {
      lo = lo.cwiseMin(V.row(i));
      hi = hi.cwiseMax(V.row(i));
    }
  return (hi - lo).allFinite() ? (hi - lo).norm() : 0.0;
}

// Area and area-weighted centroid of a triangle surface.
void area_centroid(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, double &area,
                   Eigen::RowVector3d &c) {
  Eigen::VectorXd dblA;
  igl::doublearea(V, F, dblA);
  area = 0;
  c.setZero();
  for (Eigen::Index f = 0; f < F.rows(); ++f) {
    area += dblA(f);
    c += dblA(f) * (V.row(F(f, 0)) + V.row(F(f, 1)) + V.row(F(f, 2))) / 3.0;
  }
  if (area > 0)
    c /= area;
  area *= 0.5;
}

} // namespace

void vertex_normals(const dvec &Vp, const ivec &Fp, int weighting, dvec &N) {
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = faces_with_area(V, proper_faces(to_matrix(Fp, 3)));
  Eigen::MatrixXd Nm = Eigen::MatrixXd::Zero(V.rows(), 3);
  if (F.rows() > 0) {
    const igl::PerVertexNormalsWeightingType w =
        weighting == 0   ? igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM
        : weighting == 1 ? igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA
                         : igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE;
    igl::per_vertex_normals(V, F, w, Nm);
    for (Eigen::Index i = 0; i < Nm.rows(); ++i)
      if (!Nm.row(i).allFinite())
        Nm.row(i).setZero();
  }
  from_matrix(Nm, N);
}

void orient_outward(const dvec &Vp, const ivec &Fp, ivec &FFp) {
  FFp = Fp;
  const Eigen::MatrixXi F = to_matrix(Fp, 3);
  if (F.rows() == 0)
    return;
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  Eigen::MatrixXi FF, out;
  Eigen::VectorXi C, I;
  igl::bfs_orient(F, FF, C);
  igl::orient_outward(V, FF, C, out, I);
  from_matrix(out, FFp);
}

void double_areas(const dvec &Vp, const ivec &Fp, dvec &dblA) {
  const Eigen::MatrixXi F = to_matrix(Fp, 3);
  dblA.clear();
  if (F.rows() == 0)
    return;
  Eigen::VectorXd A;
  igl::doublearea(to_matrix(Vp, 3), F, A);
  from_vector(A, dblA);
}

void weld_vertices(const dvec &Vp, double eps, ivec &rep) {
  // Not igl::remove_duplicate_vertices: with eps > 0 it merges the vertices
  // whose round(x / eps) agree, so two points 1e-12 apart on either side of a
  // rounding boundary never merge while points sqrt(3) eps apart in one cell
  // do. Here eps is a distance. The class representative is its smallest
  // member, so the surviving vertices keep their input order.
  const int n = int(Vp.size() / 3);
  rep.resize(std::size_t(n));
  std::vector<int> parent(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    parent[std::size_t(i)] = i;
  const auto at = [&Vp](int i, int c) { return Vp[3 * std::size_t(i) + std::size_t(c)]; };
  std::vector<int> finite;
  finite.reserve(std::size_t(n));
  for (int i = 0; i < n; ++i)
    if (std::isfinite(at(i, 0)) && std::isfinite(at(i, 1)) && std::isfinite(at(i, 2)))
      finite.push_back(i);

  if (!(eps > 0) || !std::isfinite(eps * eps)) {
    if (eps > 0) { // eps^2 overflows (eps > ~1e154): treated as infinite, all weld
      for (int i : finite)
        join(parent, finite.front(), i);
    } else { // exact duplicates: equal rows end up adjacent once sorted
      std::sort(finite.begin(), finite.end(), [&at](int a, int b) {
        for (int c = 0; c < 3; ++c)
          if (at(a, c) != at(b, c))
            return at(a, c) < at(b, c);
        return a < b;
      });
      for (std::size_t k = 1; k < finite.size(); ++k) {
        const int a = finite[k - 1], b = finite[k];
        if (at(a, 0) == at(b, 0) && at(a, 1) == at(b, 1) && at(a, 2) == at(b, 2))
          join(parent, a, b);
      }
    }
  } else {
    // Bucket into cells of size 2 eps (so rounding in x / cell can never put
    // two points within eps more than one cell apart), then test every pair
    // in the same or a neighbouring cell against the true distance.
    struct cell_entry {
      std::int64_t c[3];
      int i;
    };
    const double cell = 2 * eps, eps2 = eps * eps, cap = 4e18;
    std::vector<cell_entry> cells;
    cells.reserve(finite.size());
    for (int i : finite) {
      cell_entry e;
      e.i = i;
      for (int c = 0; c < 3; ++c) {
        const double q = std::floor(at(i, c) / cell);
        e.c[c] = std::int64_t(std::isnan(q) ? 0.0 : std::max(-cap, std::min(cap, q)));
      }
      cells.push_back(e);
    }
    const auto before = [](const cell_entry &a, const cell_entry &b) {
      if (a.c[0] != b.c[0])
        return a.c[0] < b.c[0];
      if (a.c[1] != b.c[1])
        return a.c[1] < b.c[1];
      return a.c[2] < b.c[2];
    };
    std::sort(cells.begin(), cells.end(), [&before](const cell_entry &a, const cell_entry &b) {
      return before(a, b) || (!before(b, a) && a.i < b.i);
    });
    const auto try_join = [&](int a, int b) {
      double d2 = 0;
      for (int c = 0; c < 3; ++c)
        d2 += (at(a, c) - at(b, c)) * (at(a, c) - at(b, c));
      if (d2 <= eps2)
        join(parent, a, b);
    };
    for (std::size_t s = 0; s < cells.size(); ++s) {
      const cell_entry &e = cells[s];
      // The rest of its own cell, then the 13 neighbouring cells that sort
      // after it: every pair of adjacent cells is visited once.
      for (std::size_t t = s + 1; t < cells.size() && !before(e, cells[t]); ++t)
        try_join(e.i, cells[t].i);
      for (int dx = 0; dx <= 1; ++dx)
        for (int dy = dx ? -1 : 0; dy <= 1; ++dy)
          for (int dz = (dx || dy) ? -1 : 1; dz <= 1; ++dz) {
            cell_entry key = e;
            key.c[0] += dx;
            key.c[1] += dy;
            key.c[2] += dz;
            const auto range = std::equal_range(cells.begin(), cells.end(), key, before);
            for (auto it = range.first; it != range.second; ++it)
              try_join(e.i, it->i);
          }
    }
  }
  for (int i = 0; i < n; ++i)
    rep[std::size_t(i)] = find_root(parent, i);
}

void boundary_facets(const ivec &Ep, int simplex_size, ivec &Bp) {
  const Eigen::MatrixXi E = to_matrix(Ep, simplex_size);
  Bp.clear();
  if (E.rows() == 0)
    return;
  Eigen::MatrixXi B;
  igl::boundary_facets(E, B);
  from_matrix(B, Bp);
}

// A port of igl::principal_curvature (Panozzo et al.'s CurvatureCalculator in
// its default configuration: area-weighted vertex normals, the projection-plane
// check, a least-squares quadric over the neighbourhood). Two upstream defects
// made it unusable as is:
//  * every vertex allocates and clears a #V-sized visited array, which is
//    O(#V^2) memory traffic (~125 GB of memset at 1M vertices); here one stamp
//    array serves every vertex;
//  * its adjacency list has max(F)+1 entries and the sphere search indexes it
//    with every vertex, reading past its end for any vertex above max(F).
// Otherwise it computes what upstream does, except that the tangent frame
// falls back to another neighbour when upstream's would be degenerate, and the
// fit runs in coordinates scaled by the neighbourhood's size, so its rank
// decisions do not depend on the mesh's unit.
void principal_curvature(const dvec &Vp, const ivec &Fp, unsigned radius, bool use_kring, dvec &k1,
                         dvec &k2, mask &ok) {
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = faces_with_area(V, proper_faces(to_matrix(Fp, 3)));
  const Eigen::Index n = V.rows();
  k1.assign(std::size_t(n), 0.0);
  k2.assign(std::size_t(n), 0.0);
  ok.assign(std::size_t(n), 0);
  if (F.rows() == 0)
    return;
  const unsigned r = std::max(radius, 2u); // libigl's minimum
  const std::size_t kMin = 6;              // libigl's minimum neighbourhood
  std::vector<std::vector<int>> adj;
  igl::adjacency_list(F, adj);
  adj.resize(std::size_t(n));
  Eigen::MatrixXd N;
  igl::per_vertex_normals(V, F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA, N);
  // Sphere mode: r mean edge lengths, every face's three edges counted.
  double sum = 0;
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    for (int j = 0; j < 3; ++j)
      sum += (V.row(F(f, j)) - V.row(F(f, (j + 1) % 3))).norm();
  const double sphere = double(r) * sum / double(3 * F.rows());

  std::vector<int> stamp(std::size_t(n), -1); // == i: seen while gathering for i
  std::vector<int> vv, queue, depth, kept;
  std::vector<std::pair<double, int>> extra; // sphere mode's candidates, a min-heap
  const auto nearer = [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
    return a.first > b.first;
  };
  for (int i = 0; i < int(n); ++i) {
    if (adj[std::size_t(i)].empty())
      continue;
    vv.clear();
    stamp[std::size_t(i)] = i;
    if (use_kring) {
      // Breadth first, out to r edges.
      depth.assign(1, 0);
      vv.push_back(i);
      for (std::size_t q = 0; q < vv.size(); ++q) {
        if (depth[q] >= int(r))
          continue;
        for (int j : adj[std::size_t(vv[q])])
          if (stamp[std::size_t(j)] != i) {
            stamp[std::size_t(j)] = i;
            vv.push_back(j);
            depth.push_back(depth[q] + 1);
          }
      }
    } else {
      // Breadth first through the vertices closer than `sphere`; if that finds
      // fewer than kMin, grow by the nearest further vertices.
      const Eigen::RowVector3d me = V.row(i);
      queue.assign(1, i);
      extra.clear();
      for (std::size_t q = 0; q < queue.size(); ++q) {
        const int v = queue[q];
        vv.push_back(v);
        for (int j : adj[std::size_t(v)])
          if (stamp[std::size_t(j)] != i) {
            stamp[std::size_t(j)] = i;
            const double d = (me - V.row(j)).norm();
            if (d < sphere) {
              queue.push_back(j);
            } else if (vv.size() < kMin) {
              extra.emplace_back(d, j);
              std::push_heap(extra.begin(), extra.end(), nearer);
            }
          }
      }
      while (!extra.empty() && vv.size() < kMin) {
        std::pop_heap(extra.begin(), extra.end(), nearer);
        const int v = extra.back().second;
        extra.pop_back();
        vv.push_back(v);
        for (int j : adj[std::size_t(v)])
          if (stamp[std::size_t(j)] != i) {
            stamp[std::size_t(j)] = i;
            extra.emplace_back((me - V.row(j)).norm(), j);
            std::push_heap(extra.begin(), extra.end(), nearer);
          }
      }
    }
    if (vv.size() < kMin)
      continue;
    // Drop neighbours facing away from the vertex, if enough remain.
    const Eigen::RowVector3d ni = N.row(i);
    kept.clear();
    for (int j : vv)
      if (N.row(j).dot(ni) > 0)
        kept.push_back(j);
    if (kept.size() >= kMin && kept.size() < vv.size())
      vv.swap(kept);
    const double nn = ni.norm();
    if (!(nn > 0) || !std::isfinite(nn))
      continue;
    const Eigen::RowVector3d z = ni / nn;

    // Tangent frame (x, y, z) at the vertex; x points at the lowest-numbered
    // neighbour, as upstream (its symmetrised shape operator below is not
    // invariant under a rotation of the frame once the fitted plane tilts).
    // Should that neighbour project to (nearly) nothing, the one with the
    // longest tangential offset is used.
    const auto tangent = [&](int j) {
      Eigen::RowVector3d t = V.row(j) - V.row(i);
      return Eigen::RowVector3d(t - t.dot(z) * z);
    };
    Eigen::RowVector3d x = tangent(adj[std::size_t(i)][0]);
    if (!(x.squaredNorm() > 1e-24 * (V.row(adj[std::size_t(i)][0]) - V.row(i)).squaredNorm()))
      for (int j : adj[std::size_t(i)])
        if (tangent(j).squaredNorm() > x.squaredNorm())
          x = tangent(j);
    const double xn = x.norm();
    if (!(xn > 0) || !std::isfinite(xn))
      continue;
    x /= xn;
    const Eigen::RowVector3d y = z.cross(x).normalized();

    // h = a u^2 + b u v + c v^2 + d u + e v in the frame, fitted in units of
    // the neighbourhood's RMS tangential radius s.
    const Eigen::Index m = Eigen::Index(vv.size());
    Eigen::MatrixXd uvh(m, 3);
    for (Eigen::Index k = 0; k < m; ++k) {
      const Eigen::RowVector3d w = V.row(vv[std::size_t(k)]) - V.row(i);
      uvh.row(k) << w.dot(x), w.dot(y), w.dot(z);
    }
    const double s = std::sqrt((uvh.col(0).squaredNorm() + uvh.col(1).squaredNorm()) / double(m));
    if (!(s > 0) || !std::isfinite(s))
      continue;
    uvh /= s;
    Eigen::MatrixXd A(m, 5);
    for (Eigen::Index k = 0; k < m; ++k) {
      const double u = uvh(k, 0), v = uvh(k, 1);
      A.row(k) << u * u, u * v, v * v, u, v;
    }
    const Eigen::VectorXd q = A.completeOrthogonalDecomposition().solve(uvh.col(2));
    const double a = q(0) / s, b = q(1) / s, c = q(2) / s, d = q(3), e = q(4);

    // libigl's shape operator of the fitted graph at the vertex (its first and
    // second fundamental forms there), symmetrised as upstream does; the
    // principal curvatures are minus its eigenvalues.
    const double E = 1.0 + d * d, Fm = d * e, G = 1.0 + e * e;
    const double nz = 1.0 / std::sqrt(d * d + e * e + 1.0);
    const double L = 2.0 * a * nz, M = b * nz, Nn = 2.0 * c * nz;
    const double det = E * G - Fm * Fm;
    const double p00 = (L * G - M * Fm) / det, p01 = (M * E - L * Fm) / det,
                 p11 = (Nn * E - M * Fm) / det;
    const double mean = 0.5 * (p00 + p11), rad = std::hypot(0.5 * (p00 - p11), p01);
    const double c1 = rad - mean, c2 = -rad - mean; // c1 >= c2
    if (!std::isfinite(c1) || !std::isfinite(c2))
      continue;
    k1[std::size_t(i)] = c1;
    k2[std::size_t(i)] = c2;
    ok[std::size_t(i)] = 1;
  }
}

void mean_curvature(const dvec &Vp, const ivec &Fp, dvec &H) {
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = cot_safe_faces(V, to_matrix(Fp, 3));
  const Eigen::Index n = V.rows();
  H.assign(std::size_t(n), 0.0);
  if (F.rows() == 0)
    return;
  Eigen::SparseMatrix<double> L;
  igl::cotmatrix(V, F, L);
  pad_square(L, int(n));
  const Eigen::VectorXd area = voronoi_area(V, F);
  const std::vector<char> rim = on_boundary(F, n);
  // -M^-1 L V is the mean-curvature normal 2 H n; the sign comes from the
  // orientation of the triangles through the vertex normal. On a boundary
  // vertex L V also holds the boundary curve's own curvature, so H is NaN.
  const Eigen::MatrixXd LV = L * V;
  dvec N;
  ivec Fv;
  from_matrix(F, Fv);
  vertex_normals(Vp, Fv, 2, N);
  for (Eigen::Index i = 0; i < n; ++i) {
    if (rim[std::size_t(i)]) {
      H[std::size_t(i)] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    if (!(area(i) > 0))
      continue;
    const Eigen::RowVector3d hn = -LV.row(i) / area(i);
    const Eigen::RowVector3d nv(N[3 * i], N[3 * i + 1], N[3 * i + 2]);
    const double h = 0.5 * hn.norm();
    H[std::size_t(i)] = hn.dot(nv) < 0 ? -h : h;
  }
}

void gaussian_curvature(const dvec &Vp, const ivec &Fp, dvec &K) {
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = cot_safe_faces(V, to_matrix(Fp, 3));
  const Eigen::Index n = V.rows();
  K.assign(std::size_t(n), 0.0);
  if (F.rows() == 0)
    return;
  // Angle defect 2 pi - sum(theta), the angles from atan2 (igl::internal_angles
  // takes an unclamped acos of the law of cosines). At a boundary vertex the
  // defect is the boundary's turning, not curvature: NaN there.
  const double kTwoPi = 6.283185307179586476925286766559;
  Eigen::VectorXd theta = Eigen::VectorXd::Zero(n);
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    for (int j = 0; j < 3; ++j) {
      const Eigen::RowVector3d o = V.row(F(f, j));
      const Eigen::RowVector3d u = V.row(F(f, (j + 1) % 3)) - o;
      const Eigen::RowVector3d w = V.row(F(f, (j + 2) % 3)) - o;
      theta(F(f, j)) += std::atan2(u.cross(w).norm(), u.dot(w));
    }
  const Eigen::VectorXd area = voronoi_area(V, F);
  const std::vector<char> rim = on_boundary(F, n);
  for (Eigen::Index i = 0; i < n; ++i)
    if (rim[std::size_t(i)])
      K[std::size_t(i)] = std::numeric_limits<double>::quiet_NaN();
    else if (area(i) > 0)
      K[std::size_t(i)] = (kTwoPi - theta(i)) / area(i);
}

void smooth_implicit(dvec &Vp, const ivec &Fp, int iterations, double lambda, const mask &fixed,
                     bool preserve_area) {
  Eigen::MatrixXd U = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = cot_safe_faces(U, to_matrix(Fp, 3));
  const Eigen::Index n = U.rows();
  if (F.rows() == 0 || iterations <= 0 || !(lambda > 0))
    return;
  const std::vector<char> ref = referenced(F, n);
  const double d = bounding_diagonal(U, ref);
  const double t = lambda * d * d;
  if (!(t > 0))
    return;

  // Unreferenced and pinned vertices are the "known" rows: they keep their
  // positions, and dropping them keeps A(unknown, unknown) positive definite.
  std::vector<int> known;
  bool any_fixed = false;
  for (Eigen::Index i = 0; i < n; ++i) {
    const bool pinned = !fixed.empty() && fixed[std::size_t(i)] && ref[std::size_t(i)];
    any_fixed = any_fixed || pinned;
    if (pinned || !ref[std::size_t(i)])
      known.push_back(int(i));
  }
  if (Eigen::Index(known.size()) == n)
    return;
  const Eigen::VectorXi K =
      Eigen::Map<const Eigen::VectorXi>(known.data(), Eigen::Index(known.size()));
  const bool rescale = preserve_area && !any_fixed;
  double area0 = 0;
  Eigen::RowVector3d c0;
  if (rescale)
    area_centroid(U, F, area0, c0);

  // L stays the operator of the input surface (conformalized MCF, Kazhdan et
  // al. 2012): re-assembling it each step lets slivers blow the flow up.
  Eigen::SparseMatrix<double> L;
  igl::cotmatrix(U, F, L);
  for (int it = 0; it < iterations; ++it) {
    const Eigen::VectorXd m = voronoi_area(U, F);
    Eigen::SparseMatrix<double> M(n, n);
    M.reserve(Eigen::VectorXi::Constant(n, 1));
    for (Eigen::Index i = 0; i < n; ++i)
      M.insert(i, i) = m(i);
    const Eigen::SparseMatrix<double> A = M - t * L;
    const Eigen::MatrixXd B = -(M * U);
    Eigen::MatrixXd Y(K.size(), 3);
    for (Eigen::Index k = 0; k < K.size(); ++k)
      Y.row(k) = U.row(K(k));
    igl::min_quad_with_fixed_data<double> data;
    if (!igl::min_quad_with_fixed_precompute(A, K, Eigen::SparseMatrix<double>(), true, data))
      throw failure("implicit smoothing: factorization failed");
    Eigen::MatrixXd next;
    igl::min_quad_with_fixed_solve(data, B, Y, Eigen::MatrixXd(0, 3), next);
    if (!next.allFinite())
      throw failure("implicit smoothing: non-finite result");
    U = next;
    if (rescale) {
      double area;
      Eigen::RowVector3d c;
      area_centroid(U, F, area, c);
      if (area > 0) {
        const double s = std::sqrt(area0 / area);
        for (Eigen::Index i = 0; i < n; ++i)
          if (ref[std::size_t(i)])
            U.row(i) = (U.row(i) - c) * s + c0;
      }
    }
  }
  from_matrix(U, Vp);
}

void smooth_explicit(dvec &Vp, const ivec &Fp, int iterations, double lambda, double mu,
                     const mask &fixed) {
  Eigen::MatrixXd U = to_matrix(Vp, 3);
  const Eigen::MatrixXi F = proper_faces(to_matrix(Fp, 3));
  const Eigen::Index n = U.rows();
  if (F.rows() == 0 || iterations <= 0)
    return;
  std::vector<std::vector<int>> adj;
  igl::adjacency_list(F, adj);
  adj.resize(std::size_t(n));
  // Jacobi update: every vertex moves toward the average of its neighbours'
  // positions from the previous pass.
  const auto step = [&](double s) {
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, 3);
    for (Eigen::Index i = 0; i < n; ++i) {
      const std::vector<int> &nb = adj[std::size_t(i)];
      if (nb.empty() || (!fixed.empty() && fixed[std::size_t(i)]))
        continue;
      Eigen::RowVector3d avg = Eigen::RowVector3d::Zero();
      for (int j : nb)
        avg += U.row(j);
      D.row(i) = s * (avg / double(nb.size()) - U.row(i));
    }
    U += D;
  };
  for (int it = 0; it < iterations; ++it) {
    step(lambda);
    if (mu != 0)
      step(mu);
  }
  from_matrix(U, Vp);
}

void colormap(int map, double f, double rgb[3]) {
  static const igl::ColorMapType maps[] = {igl::COLOR_MAP_TYPE_VIRIDIS, igl::COLOR_MAP_TYPE_MAGMA,
                                           igl::COLOR_MAP_TYPE_PLASMA,  igl::COLOR_MAP_TYPE_INFERNO,
                                           igl::COLOR_MAP_TYPE_TURBO,   igl::COLOR_MAP_TYPE_PARULA};
  igl::colormap(maps[std::clamp(map, 0, 5)], f, rgb[0], rgb[1], rgb[2]);
}

} // namespace igl_detail
} // namespace cvc

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

// igl_locator.cpp -- closest point, ray casting and winding numbers over a
// triangle surface: an igl::AABB built once (rays walk it with their own
// double-precision test), plus an igl::FastWindingNumberBVH built on first use.

#include "igl_eigen.h"

#include <cmath>
#include <igl/AABB.h>
#include <igl/fast_winding_number.h>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

namespace cvc {
namespace igl_detail {

struct locator_state {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::AABB<Eigen::MatrixXd, 3> tree;
  // The winding-number BVH is evaluated in float, so it is built over V moved
  // to the centre of its bounding box and scaled to unit size: world-scale
  // coordinates would otherwise lose most of their precision.
  Eigen::RowVector3d center = Eigen::RowVector3d::Zero();
  double scale = 1.0;
  mutable std::once_flag fwn_once;
  mutable igl::FastWindingNumberBVH fwn;
};

std::shared_ptr<locator_state> make_locator(const dvec &V, const ivec &F) {
  auto s = std::make_shared<locator_state>();
  s->V = to_matrix(V, 3);
  s->F = to_matrix(F, 3);
  if (s->F.rows() == 0)
    return s;
  s->tree.init(s->V, s->F);
  Eigen::RowVector3d lo = s->V.row(s->F(0, 0)), hi = lo;
  for (Eigen::Index i = 0; i < s->F.size(); ++i) {
    lo = lo.cwiseMin(s->V.row(s->F.data()[i]));
    hi = hi.cwiseMax(s->V.row(s->F.data()[i]));
  }
  s->center = 0.5 * (lo + hi);
  const double diag = (hi - lo).norm();
  s->scale = diag > 0 && std::isfinite(diag) ? diag : 1.0;
  return s;
}

std::size_t locator_num_faces(const locator_state &s) { return std::size_t(s.F.rows()); }

void locator_closest(const locator_state &s, const dvec &Qp, dvec &sqd,
                     std::vector<std::int64_t> &face, dvec &C) {
  const std::size_t n = Qp.size() / 3;
  if (s.F.rows() == 0 || n == 0) {
    sqd.assign(n, std::numeric_limits<double>::infinity());
    face.assign(n, -1);
    C.assign(3 * n, std::numeric_limits<double>::quiet_NaN());
    return;
  }
  const Eigen::MatrixXd Q = to_matrix(Qp, 3);
  Eigen::VectorXd D;
  Eigen::VectorXi I;
  Eigen::MatrixXd P;
  s.tree.squared_distance(s.V, s.F, Q, D, I, P);
  from_vector(D, sqd);
  face.assign(I.data(), I.data() + I.size());
  from_matrix(P, C);
}

// ---- ray casting ----------------------------------------------------------------
//
// Not igl::AABB::intersect_ray. Its leaf test (intersect_triangle1) rejects a
// hit when |det| <= 1e-6, an ABSOLUTE bound on det = e1 . (dir x e2) = 2 area
// |dir| cos(angle): a mesh in metres with millimetre triangles, a unit-size
// mesh with ~1e6 triangles, or a short dir (b - a for a segment) is silently
// never hit. It also keeps t in float, so of two hits closer than ~1e-7
// relative it can return the farther. The tree's nodes are public, so the
// traversal below walks them in double with a scale-free test instead.

namespace {

typedef igl::AABB<Eigen::MatrixXd, 3> tree_t;

// The parameter interval [t0, t1] of o + t d inside box b, clipped to the
// given [t0, t1]; false when it is empty. Each slab bound is widened by 1e-12
// relative (the division's rounding is ~2e-16), and a NaN bound never prunes.
bool clip_to_box(const Eigen::AlignedBox<double, 3> &b, const Eigen::RowVector3d &o,
                 const Eigen::RowVector3d &d, double &t0, double &t1) {
  for (int c = 0; c < 3; ++c) {
    const double lo = b.min()(c), hi = b.max()(c);
    if (lo > hi)
      return false; // an empty box
    if (d(c) == 0) {
      if (o(c) < lo || o(c) > hi)
        return false;
      continue;
    }
    double ta = (lo - o(c)) / d(c), tb = (hi - o(c)) / d(c);
    if (ta > tb)
      std::swap(ta, tb);
    ta -= 1e-12 * std::abs(ta);
    tb += 1e-12 * std::abs(tb);
    if (ta > t0)
      t0 = ta;
    if (tb < t1)
      t1 = tb;
    if (t0 > t1)
      return false;
  }
  return true;
}

// Moller-Trumbore in double against face f. A hit needs a determinant that is
// not ~0 RELATIVE to |e1 x e2| |d| (the ray is not within 1e-14 rad of the
// triangle's plane), barycentrics in the triangle (to 1e-12, so a ray through
// a shared edge does not slip between its two triangles), and t in [t0, t1].
bool hit_face(const locator_state &s, int f, const Eigen::RowVector3d &o,
              const Eigen::RowVector3d &d, double dnorm, double t0, double t1, double &t, double &u,
              double &v) {
  const Eigen::RowVector3d a = s.V.row(s.F(f, 0));
  const Eigen::RowVector3d e1 = s.V.row(s.F(f, 1)) - a;
  const Eigen::RowVector3d e2 = s.V.row(s.F(f, 2)) - a;
  const Eigen::RowVector3d p = d.cross(e2);
  const double det = e1.dot(p);
  if (!std::isfinite(det) || !(std::abs(det) > 1e-14 * e1.cross(e2).norm() * dnorm))
    return false;
  const Eigen::RowVector3d tv = o - a;
  const Eigen::RowVector3d q = tv.cross(e1);
  const double tt = e2.dot(q) / det, uu = tv.dot(p) / det, vv = d.dot(q) / det;
  const double tol = 1e-12;
  if (!(uu >= -tol && vv >= -tol && uu + vv <= 1 + tol))
    return false;
  if (!(tt >= t0 && tt <= t1))
    return false;
  t = tt;
  u = uu;
  v = vv;
  return true;
}

} // namespace

bool locator_ray(const locator_state &s, const double origin[3], const double dir[3], double min_t,
                 double max_t, std::int64_t &face, double &t, double &u, double &v) {
  if (s.F.rows() == 0 || !std::isfinite(min_t) || !(max_t >= min_t))
    return false;
  const Eigen::RowVector3d o(origin[0], origin[1], origin[2]);
  const Eigen::RowVector3d d(dir[0], dir[1], dir[2]);
  const double dnorm = d.norm();
  if (!o.allFinite() || !d.allFinite() || !(dnorm > 0))
    return false;

  // Depth-first, nearer child first, pruning every box the ray cannot reach
  // before the best hit so far.
  int best_face = -1;
  double best_t = max_t, best_u = 0, best_v = 0;
  std::vector<std::pair<const tree_t *, double>> stack; // node, entry t
  stack.reserve(64);
  {
    double t0 = min_t, t1 = best_t;
    if (clip_to_box(s.tree.m_box, o, d, t0, t1))
      stack.emplace_back(&s.tree, t0);
  }
  while (!stack.empty()) {
    const tree_t *node = stack.back().first;
    const double entry = stack.back().second;
    stack.pop_back();
    if (entry > best_t)
      continue;
    if (node->is_leaf()) {
      double tt, uu, vv;
      const int f = node->m_primitive;
      if (f >= 0 && hit_face(s, f, o, d, dnorm, min_t, best_t, tt, uu, vv) &&
          (best_face < 0 || tt < best_t || (tt == best_t && f < best_face))) {
        best_face = f;
        best_t = tt;
        best_u = uu;
        best_v = vv;
      }
      continue;
    }
    std::pair<const tree_t *, double> next[2];
    int k = 0;
    for (const tree_t *child : {node->m_left, node->m_right}) {
      double t0 = min_t, t1 = best_t;
      if (child && clip_to_box(child->m_box, o, d, t0, t1))
        next[k++] = std::make_pair(child, t0);
    }
    // Push the farther child first so the nearer one is searched first.
    if (k == 2 && next[0].second < next[1].second)
      std::swap(next[0], next[1]);
    for (int i = 0; i < k; ++i)
      stack.push_back(next[i]);
  }
  if (best_face < 0)
    return false;
  face = best_face;
  t = best_t;
  u = best_u;
  v = best_v;
  return true;
}

void locator_winding(const locator_state &s, const dvec &Qp, dvec &W) {
  const std::size_t n = Qp.size() / 3;
  if (s.F.rows() == 0 || n == 0) {
    W.assign(n, 0.0);
    return;
  }
  std::call_once(s.fwn_once, [&s] {
    const Eigen::MatrixXf Vf = ((s.V.rowwise() - s.center) / s.scale).cast<float>();
    igl::fast_winding_number(Vf, s.F, 2, s.fwn);
  });
  const Eigen::MatrixXd Q = (to_matrix(Qp, 3).rowwise() - s.center) / s.scale;
  Eigen::VectorXd w;
  igl::fast_winding_number(s.fwn, 2.0f, Q, w);
  from_vector(w, W);
}

} // namespace igl_detail
} // namespace cvc

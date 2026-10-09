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

// igl_eigen.h -- packed std::vector <-> Eigen conversions, and the element-shape
// tests that guard igl::cotmatrix, shared by the igl_*.cpp TUs. INTERNAL, and
// included only by those TUs: no cvc TU may see Eigen.
//
// Inputs are always COPIED into plain MatrixXd/MatrixXi, never wrapped in an
// Eigen::Map: libigl's AABB, winding-number and distance templates are
// instantiated on the matrix type they are given, and only the plain types are
// what upstream builds and tests.

#ifndef CVC_GEOMETRY_IGL_EIGEN_H
#define CVC_GEOMETRY_IGL_EIGEN_H

#include "igl_detail.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <utility>

namespace cvc {
namespace igl_detail {

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatXd;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatXi;

inline Eigen::MatrixXd to_matrix(const dvec &a, int cols) {
  const Eigen::Index rows = Eigen::Index(a.size() / cols);
  if (rows == 0)
    return Eigen::MatrixXd(0, cols);
  return Eigen::Map<const RowMatXd>(a.data(), rows, cols);
}

inline Eigen::MatrixXi to_matrix(const ivec &a, int cols) {
  const Eigen::Index rows = Eigen::Index(a.size() / cols);
  if (rows == 0)
    return Eigen::MatrixXi(0, cols);
  return Eigen::Map<const RowMatXi>(a.data(), rows, cols);
}

inline void from_matrix(const Eigen::MatrixXd &m, dvec &a) {
  a.resize(std::size_t(m.size()));
  if (m.size())
    Eigen::Map<RowMatXd>(a.data(), m.rows(), m.cols()) = m;
}

inline void from_matrix(const Eigen::MatrixXi &m, ivec &a) {
  a.resize(std::size_t(m.size()));
  if (m.size())
    Eigen::Map<RowMatXi>(a.data(), m.rows(), m.cols()) = m;
}

inline Eigen::VectorXd to_vector(const dvec &a) {
  if (a.empty())
    return Eigen::VectorXd(0);
  return Eigen::Map<const Eigen::VectorXd>(a.data(), Eigen::Index(a.size()));
}

inline void from_vector(const Eigen::VectorXd &v, dvec &a) {
  a.assign(v.data(), v.data() + v.size());
}

// The triplets of a sparse matrix, explicit zeros dropped.
inline void to_triplets(const Eigen::SparseMatrix<double> &A, triplets &out) {
  out.row.clear();
  out.col.clear();
  out.val.clear();
  out.row.reserve(std::size_t(A.nonZeros()));
  out.col.reserve(std::size_t(A.nonZeros()));
  out.val.reserve(std::size_t(A.nonZeros()));
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
      if (it.value() != 0.0) {
        out.row.push_back(int(it.row()));
        out.col.push_back(int(it.col()));
        out.val.push_back(it.value());
      }
}

// libigl sizes some operators by max(F)+1 instead of #V (massmatrix on
// triangles): pad to n x n so trailing unreferenced vertices keep their rows.
inline void pad_square(Eigen::SparseMatrix<double> &A, int n) {
  if (A.rows() < n || A.cols() < n)
    A.conservativeResize(n, n);
}

// ---- element shape: what igl::cotmatrix can be trusted with -------------------
//
// igl::cotmatrix works INTRINSICALLY, from edge lengths rounded to double: a
// triangle's area comes from Kahan's Heron formula, whose factor c - (a - b)
// (sorted lengths a >= b >= c) is the triangle inequality's slack. On a
// near-collinear "cap" that slack (~ 2 h^2 / a for height h) drowns in the
// lengths' rounding (~ eps a) once h / a < ~1e-8: the area comes out 0 (or NaN,
// which libigl turns into 0) and the cotangents +-inf. A positive extrinsic
// (cross-product) area does not prevent this. Every consumer of cotmatrix
// (smoothing, mean curvature, geodesics, fem) therefore keeps only elements
// that pass these tests, which bound the relative error of the computed area
// by ~1e-3.

// A triangle whose slack exceeds 1e-12 of its longest edge: not flatter than
// h / a ~ 7e-7. Repeated corners and zero-length edges fail too.
inline bool cot_safe_triangle(const Eigen::MatrixXd &V, int i0, int i1, int i2) {
  // The lengths as igl::squared_edge_lengths + sqrt form them.
  double a = std::sqrt((V.row(i1) - V.row(i2)).squaredNorm());
  double b = std::sqrt((V.row(i2) - V.row(i0)).squaredNorm());
  double c = std::sqrt((V.row(i0) - V.row(i1)).squaredNorm());
  if (!(std::isfinite(a) && std::isfinite(b) && std::isfinite(c)))
    return false;
  if (a < b)
    std::swap(a, b);
  if (b < c)
    std::swap(b, c);
  if (a < b)
    std::swap(a, b);
  const double slack = c - (a - b);
  if (!(slack > 1e-12 * a))
    return false;
  const double arg = (a + (b + c)) * slack * (c + (a - b)) * (a + (b - c));
  return arg > 0 && std::isfinite(arg);
}

// A tet with |6 V| > 1e-6 l_max^3. cotmatrix gets a tet's volume from its six
// edge lengths (Kahan), whose V^2 carries an error of ~eps l^6; this keeps the
// computed volume (and each face's area) to ~1e-3 or better.
inline bool cot_safe_tet(const Eigen::MatrixXd &V, int i0, int i1, int i2, int i3) {
  const Eigen::RowVector3d e1 = V.row(i1) - V.row(i0);
  const Eigen::RowVector3d e2 = V.row(i2) - V.row(i0);
  const Eigen::RowVector3d e3 = V.row(i3) - V.row(i0);
  const double six_vol = std::abs(e1.dot(e2.cross(e3)));
  double l2 = std::max(e1.squaredNorm(), std::max(e2.squaredNorm(), e3.squaredNorm()));
  l2 = std::max(l2, (V.row(i2) - V.row(i1)).squaredNorm());
  l2 = std::max(l2, (V.row(i3) - V.row(i1)).squaredNorm());
  l2 = std::max(l2, (V.row(i3) - V.row(i2)).squaredNorm());
  if (!(std::isfinite(six_vol) && std::isfinite(l2)))
    return false;
  return six_vol > 1e-6 * l2 * std::sqrt(l2);
}

// The rows of F (triangles) that are cot_safe_triangle.
inline Eigen::MatrixXi cot_safe_faces(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
  Eigen::MatrixXi out(F.rows(), 3);
  Eigen::Index m = 0;
  for (Eigen::Index f = 0; f < F.rows(); ++f)
    if (cot_safe_triangle(V, F(f, 0), F(f, 1), F(f, 2)))
      out.row(m++) = F.row(f);
  out.conservativeResize(m, 3);
  return out;
}

} // namespace igl_detail
} // namespace cvc

#endif // CVC_GEOMETRY_IGL_EIGEN_H

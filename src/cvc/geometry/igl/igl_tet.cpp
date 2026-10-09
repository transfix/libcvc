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

// igl_tet.cpp -- tetrahedral volumes and level sets (igl::volume,
// igl::marching_tets) behind igl_detail.h.

#include "igl_eigen.h"

#include <igl/marching_tets.h>
#include <igl/volume.h>
#include <utility>

namespace cvc {
namespace igl_detail {

void tet_volumes(const dvec &Vp, const ivec &Tp, dvec &vol) {
  const Eigen::MatrixXi T = to_matrix(Tp, 4);
  vol.clear();
  if (T.rows() == 0)
    return;
  Eigen::VectorXd v;
  igl::volume(to_matrix(Vp, 3), T, v);
  from_vector(v, vol);
}

void marching_tets(const dvec &Vp, const ivec &Tp, const dvec &Sp, double iso, dvec &SV, ivec &SF,
                   ivec &bc_row, ivec &bc_col, dvec &bc_val) {
  SV.clear();
  SF.clear();
  bc_row.clear();
  bc_col.clear();
  bc_val.clear();
  const Eigen::MatrixXd V = to_matrix(Vp, 3);
  const Eigen::MatrixXi T = to_matrix(Tp, 4);
  if (T.rows() == 0 || V.rows() < 4)
    return;
  const Eigen::VectorXd S = to_vector(Sp);
  Eigen::MatrixXd outV;
  Eigen::MatrixXi outF;
  Eigen::VectorXi J;
  Eigen::SparseMatrix<double> BC;
  igl::marching_tets(V, T, S, iso, outV, outF, J, BC);

  // Inside one tet the level set is planar and separates the corners above iso
  // from the rest, so a face faces "up the field" when its normal points toward
  // any such corner.
  for (Eigen::Index f = 0; f < outF.rows(); ++f) {
    const Eigen::Index t = J(f);
    int above = -1;
    for (int c = 0; c < 4 && above < 0; ++c)
      if (S(T(t, c)) > iso)
        above = T(t, c);
    if (above < 0)
      continue;
    const Eigen::RowVector3d a = outV.row(outF(f, 0));
    const Eigen::RowVector3d e1 = outV.row(outF(f, 1)) - a;
    const Eigen::RowVector3d e2 = outV.row(outF(f, 2)) - a;
    const Eigen::RowVector3d n = e1.cross(e2);
    if (n.dot(V.row(above) - a) < 0)
      std::swap(outF(f, 1), outF(f, 2));
  }
  from_matrix(outV, SV);
  from_matrix(outF, SF);
  triplets bc;
  to_triplets(BC, bc);
  bc_row.swap(bc.row);
  bc_col.swap(bc.col);
  bc_val.swap(bc.val);
}

} // namespace igl_detail
} // namespace cvc

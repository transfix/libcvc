/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// grid_math.h — tiny shared grid numerics used by more than one nav TU.
// Extracted from grid_nav.cpp so material.cpp reuses the SAME float32
// np.gradient transcription instead of drifting a copy. Bit-identical
// pure move; do not "tidy" the arithmetic.

#ifndef CVC_NAV_DETAIL_GRID_MATH_H
#define CVC_NAV_DETAIL_GRID_MATH_H

namespace cvc {
namespace nav {
namespace detail {

// np.gradient along one axis (edge_order=1, unit spacing), evaluated in float32
// at index i of a length-L line whose neighbours are `stride` apart in `a`.
inline float grad1d(const float *a, int i, int L, int stride) {
  // A length-1 axis has no neighbour to difference against — np.gradient itself
  // errors here (edge_order=1 needs >=2 samples), so there is no reference value
  // to match; the gradient along a degenerate axis is 0. Guard this before the
  // i==0 / i==L-1 branches, which would otherwise both fire and read a[stride]
  // (one past the row on a single-row/-column grid) — a heap OOB read whose
  // garbage flipped build_sdf's unit normals (NavSdf flake, seen under ASan).
  if (L <= 1)
    return 0.0f;
  if (i == 0)
    return a[stride] - a[0];
  if (i == L - 1)
    return a[i * stride] - a[(i - 1) * stride];
  return (a[(i + 1) * stride] - a[(i - 1) * stride]) / 2.0f;
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif

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

// Shewchuk's exact predicates (cvcpkg's `shewchuk-predicates`), with Shewchuk's signs. Internal to
// libcvc: every caller goes through these functions instead of including <shewchuk/predicates.hpp>.
//
// The reason: Shewchuk's expansion sums read one element past an input expansion on their last
// pass (fast_expansion_sum_zeroelim's `f[++findex]`). The value is never used, but ASan reports
// the read as a stack-buffer-overflow in orient3dadapt and insphereadapt. So exact_predicates.cpp
// is the only file that includes the header, and the sanitizer builds compile that one file
// without ASan (src/cvc/CMakeLists.txt). The inline copies of the header's functions then exist
// only in that file, so no instrumented copy can be linked in their place.

#ifndef CVC_GEOMETRY_EXACT_PREDICATES_H
#define CVC_GEOMETRY_EXACT_PREDICATES_H

#ifdef CVC_WITH_SHEWCHUK

namespace cvc::exact {

// Positive when d is below the plane through a, b, c (a, b, c counterclockwise seen from above),
// negative above, zero when the four points are coplanar. Each argument is 3 doubles.
double orient3d(const double *a, const double *b, const double *c, const double *d);

// Positive when e is inside the sphere through a, b, c, d (for orient3d(a, b, c, d) > 0),
// negative outside, zero on it. Each argument is 3 doubles.
double insphere(const double *a, const double *b, const double *c, const double *d,
                const double *e);

// Positive when a, b, c are counterclockwise, negative clockwise, zero when collinear. Each
// argument is 2 doubles.
double orient2d(const double *a, const double *b, const double *c);

} // namespace cvc::exact

#endif // CVC_WITH_SHEWCHUK

#endif // CVC_GEOMETRY_EXACT_PREDICATES_H

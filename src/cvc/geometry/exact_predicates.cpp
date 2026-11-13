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

// The one file that includes Shewchuk's predicates (see exact_predicates.h for why).

#include "exact_predicates.h"

#ifdef CVC_WITH_SHEWCHUK

#include <shewchuk/predicates.hpp>

namespace cvc::exact {

double orient3d(const double *a, const double *b, const double *c, const double *d) {
  return shewchuk::detail::orient3d(a, b, c, d);
}

double insphere(const double *a, const double *b, const double *c, const double *d,
                const double *e) {
  return shewchuk::detail::insphere(a, b, c, d, e);
}

double orient2d(const double *a, const double *b, const double *c) {
  return shewchuk::detail::orient2d(a, b, c);
}

} // namespace cvc::exact

#endif // CVC_WITH_SHEWCHUK

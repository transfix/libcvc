/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#include <stdexcept>

#include "Tiling_region.h"
#include "Wedge.h"
#include "Composite_tiling_region.h"
#include "All_tiling_region.h"
#include "print_utils.h"

namespace cvc::tiler {

const Number_type Tiling_region::AMBIGUOUS_Z_HOME = std::numeric_limits<Number_type>::quiet_NaN();

HTiling_region Tiling_region::overlapping_vertex(const Point_2& p, const Point_2& q, const Point_2& r,
						 const Point_2& a, const Point_2& b, const Point_2& c)
{
  if (!xy_equal(q, b))
    throw std::logic_error("overlapping_vertex requires that q == b");

  HWedge LSV = Wedge::LS(p, q, r, AMBIGUOUS_Z_HOME);
  HWedge RSV = Wedge::RS(p, q, r, AMBIGUOUS_Z_HOME);
  HWedge LSVp = Wedge::LS(a, b, c, AMBIGUOUS_Z_HOME);
  HWedge RSVp = Wedge::RS(a, b, c, AMBIGUOUS_Z_HOME);

  HTiling_region L(LSV);
  HTiling_region R(RSV);
  HTiling_region Lp(LSVp);
  HTiling_region Rp(RSVp);

  if (LSV->is_empty_intersection(*LSVp) && RSV->is_empty_intersection(*RSVp)) {
    return All_tiling_region::empty(AMBIGUOUS_Z_HOME);
  }

  if (LSV->is_empty_intersection(*LSVp)) {
    return Rp->get_complement() & Wedge::whitelist(*RSV, q);
  }

  if (RSV->is_empty_intersection(*RSVp)) {
    return Lp->get_complement() & Wedge::whitelist(*LSV, q);
  }

  return ((L & Lp) | (R & Rp))->get_complement();
}

Tiling_region::Tiling_region(Number_type z_home) : _z_home(z_home)
{ 
}

Tiling_region::~Tiling_region()
{
}

bool Tiling_region::has_z_home() const 
{ 
  return _z_home == _z_home; 
}

Number_type Tiling_region::z_home() const 
{ 
  if (!has_z_home()) {
    throw logic_error("z_home is ambiguous");
  }
  return _z_home; 
}

Number_type Tiling_region::z_home_nothrow() const 
{ 
  return _z_home; 
}

HTiling_region operator&(HTiling_region a, HTiling_region b)
{ 
  return Composite_tiling_region::composite(a, b, true, a->z_home_nothrow());
}

HTiling_region operator|(HTiling_region a, HTiling_region b)
{ 
  return Composite_tiling_region::composite(a, b, false, a->z_home_nothrow());
}

} // namespace cvc::tiler

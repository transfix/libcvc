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

#include "Point_tiling_region.h"
#include "print_utils.h"

namespace cvc::tiler {

bool Point_tiling_region::contains(const Point_3& point) const
{
  if (_z)
    return _inverse == !xyz_equal(point, _point);
  return _inverse == !xy_equal(point, _point);
}

bool Point_tiling_region::contains(const Segment_3& segment) const
{
  if (_z) {
    if (_inverse)
      return !xyz_equal(segment.source(), _point) && !xyz_equal(segment.target(), _point);
    return xyz_equal(segment.source(), _point) || xyz_equal(segment.target(), _point);
  }
  if (_inverse)
    return !xy_equal(segment.source(), _point) && !xy_equal(segment.target(), _point);
  return xy_equal(segment.source(), _point) || xy_equal(segment.target(), _point);
}

HTiling_region Point_tiling_region::get_complement() const
{
  return HTiling_region(new Point_tiling_region(_point, !_inverse, _z, z_home_nothrow()));
}
std::ostream& Point_tiling_region::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Point_tiling_region& region)
{
  out <<    
    "p = " << pp(region._point) << 
    " inverse = " << region._inverse;
  return out;
}

} // namespace cvc::tiler


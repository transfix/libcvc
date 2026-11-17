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

#include "All_tiling_region.h"
#include "print_utils.h"

namespace cvc::tiler {

All_tiling_region::All_tiling_region(Number_type z_home) : Tiling_region(z_home), _empty(false)
{ 
}

All_tiling_region::All_tiling_region(bool empty, Number_type z_home) : Tiling_region(z_home), _empty(empty)
{ 
}

All_tiling_region::~All_tiling_region()
{
}

bool All_tiling_region::contains(const Point_3& point) const
{
  return !_empty;
}

bool All_tiling_region::contains(const Segment_3& segment) const
{
  return !_empty;
}

HTiling_region All_tiling_region::get_complement() const
{
  return HTiling_region(new All_tiling_region(!_empty));
}

std::ostream& All_tiling_region::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const All_tiling_region& region)
{
  if (region._empty)
    out << "EMPTY";
  else
    out << "ALL";
  return out;
}

} // namespace cvc::tiler

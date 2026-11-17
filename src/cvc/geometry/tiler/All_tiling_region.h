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

#ifndef __ALL_TILING_REGION_H__
#define __ALL_TILING_REGION_H__

#include <boost/shared_ptr.hpp>


#include "common.h"
#include "Tiling_region.h"

namespace cvc::tiler {

class All_tiling_region : public Tiling_region
{
public:
  static HTiling_region empty(Number_type z_home) { return HTiling_region(new All_tiling_region(true, z_home)); }

public:
  All_tiling_region(Number_type z_home);

  All_tiling_region(bool empty, Number_type z_home);

public:
  virtual ~All_tiling_region();

  virtual bool contains(const Point_3& point) const;

  virtual bool contains(const Segment_3& segment) const;

  virtual HTiling_region get_complement() const;

  virtual std::ostream& print(std::ostream& out) const;

  friend std::ostream& operator<<(std::ostream& out, const All_tiling_region& region);

private:
  bool _empty;
};

} // namespace cvc::tiler

#endif

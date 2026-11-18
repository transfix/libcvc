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

#ifndef __TILING_REGION_H__
#define __TILING_REGION_H__

#include "common.h"

#include <boost/shared_ptr.hpp>
#include <limits>

namespace cvc::tiler {

class Tiling_region;
typedef boost::shared_ptr<const Tiling_region> HTiling_region;

class Tiling_region {
public:
  static const Number_type AMBIGUOUS_Z_HOME;

  static HTiling_region overlapping_vertex(const Point_2 &p, const Point_2 &q, const Point_2 &r,
                                           const Point_2 &a, const Point_2 &b, const Point_2 &c);

public:
  Tiling_region(Number_type z_home);

  virtual ~Tiling_region();

  virtual bool contains(const Point_3 &point) const = 0;

  virtual bool contains(const Segment_3 &segment) const = 0;

  virtual HTiling_region get_complement() const = 0;

  bool has_z_home() const;

  Number_type z_home() const;

  // This version returns NaN if there is no z_home.
  Number_type z_home_nothrow() const;

  virtual std::ostream &print(std::ostream &out) const = 0;

  friend HTiling_region operator&(HTiling_region a, HTiling_region b);

  friend HTiling_region operator|(HTiling_region a, HTiling_region b);

  friend std::ostream &operator<<(std::ostream &out, const Tiling_region &region) {
    region.print(out);
    return out;
  }

private:
  Number_type _z_home;
};

} // namespace cvc::tiler

#endif

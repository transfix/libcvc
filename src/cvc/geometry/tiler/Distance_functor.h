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

#ifndef __DISTANCE_FUNCTOR_H__
#define __DISTANCE_FUNCTOR_H__

#include "common.h"

namespace cvc::tiler {

//------------------------------------------------------------------------------
// Distance_functor class
//------------------------------------------------------------------------------
template <typename Point>
class Distance_functor
{
public:
  Distance_functor()
  {}

  Distance_functor(const Point& s)
  { _source = s; }

  bool operator()( const Point& a, const Point& b ) const
  { return has_larger_distance_to_point(_source, b, a); }

private:
  Point _source;
};

//------------------------------------------------------------------------------
// dist_functor
//
/// Utility function to create a distance functor based on a point.
//------------------------------------------------------------------------------
template <typename Point>
Distance_functor<Point> dist_functor(const Point& point)
{
  return Distance_functor<Point>(point);
}

typedef Distance_functor<Point_2> Distance_functor_2;
typedef Distance_functor<Point_3> Distance_functor_3;

} // namespace cvc::tiler

#endif

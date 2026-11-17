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

//-----------------------
// Multi-tiler operations
//-----------------------

#ifndef __MTILER_OPERATIONS__
#define __MTILER_OPERATIONS__

#include "common.h"
#include "Tiler_workspace.h"
#include "triangle_utils.h"

namespace cvc::tiler {

class Corner
{
public:
  Corner() : _count(0) {}
  Corner(const Point_3& p) : _count(1) 
  { _points[0] = p; }
  Corner(const Point_3& p0, const Point_3& p1) : _count(2) 
  { _points[0] = p0; _points[1] = p1; }

  const Point_3& operator[](size_t i) const
  { 
    if (i >= _count)
      throw logic_error("Unexpected size");
    return _points[i]; 
  }
  size_t size() const { return _count; }

private:
  size_t _count;
  Point_3 _points[2];
};

class DistanceFunctor
{
public:
  DistanceFunctor()
  {}

  DistanceFunctor(const Point_2& s)
  { _source = s; }

  bool operator()( const Point_2& a, const Point_2& b ) const
  { return has_larger_distance_to_point(_source, b, a); }

private:
  Point_2 _source;
};

bool point_intersection_2(const Segment_3& a, const Segment_3& b, Point_3& ia, Point_3& ib);


bool point_intersection_2(const Segment_3& a, const Segment_3& b, Point_3& ia);

// template <typename Tiles_iter>
// bool split(Tile_handle tile, const Segment2points& seg2pts, const Segment2points& seg2pts2, TW_handle w, Tiles_iter new_tiles);

template <typename TW_iterator>
void multi_test(TW_iterator begin, TW_iterator end, Number_type epsilon);

void mtest_output(vector<boost::shared_ptr<Triangle> >& new_yellow, vector<boost::shared_ptr<Triangle> >& new_green,
		  list<boost::shared_ptr<Triangle> >& yellow, list<boost::shared_ptr<Triangle> >& green);

} // namespace cvc::tiler

#endif

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

#ifndef __CUT_H__
#define __CUT_H__

#include "Intersections.h"
#include "Polyline.h"
#include "common.h"
#include "triangle_utils.h"

#include <boost/unordered_set.hpp>

namespace cvc::tiler {

Polygon_2
make_poly(const Triangle &triangle,
          const boost::unordered_map<Segment_3_, boost::unordered_set<Point_3>> &seg2points);
Polygon_2 make_poly(const Triangle &triangle,
                    const boost::unordered_map<Point_3, Segment_3> &pt2seg);
Polygon_2 make_poly(const Triangle &triangle,
                    const boost::unordered_map<Point_3, Segment_3_> &pt2seg);

pair<Polygon_2, Polygon_2> cut_polygon_with_line(const Polygon_2 &p, const Segment_3 &cut);
pair<Polygon_2, Polygon_2> cut_polygon_with_polyline(const Polygon_2 &p, const Polyline_2 &cut);

template <typename Polyline_iter, typename Polygon_iter>
void cut_polygon_with_polylines(const Polygon_2 &p, Polyline_iter cuts_begin,
                                Polyline_iter cuts_end, Polygon_iter polys);

template <typename Tile_handle, typename Cuts_iter, typename Poly_iter>
void cut_tile_with_polylines(
    Tile_handle tile, Cuts_iter cuts_begin, Cuts_iter cuts_end, Poly_iter new_polys,
    const Intersections<Tile_handle> &ints, int i,
    boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges);

template <typename Polyline_iter, typename Triangle_iter>
void cut_into_triangles(const Polygon_2 &p, Polyline_iter cuts_begin, Polyline_iter cuts_end,
                        Triangle_iter triangles);

template <typename Tile_handle, typename Cuts_iter, typename Tiles_iter>
void cut_into_triangles(Tile_handle tile, Cuts_iter cuts_begin, Cuts_iter cuts_end,
                        Tiles_iter new_tiles, const Intersections<Tile_handle> &ints, int i);

std::pair<Triangle, Triangle> decompose_triangle(const Triangle &triangle, const Segment_3 &edge,
                                                 const Point_3 &point);

template <typename Out_iter>
void decompose_triangle(
    const Triangle &triangle,
    const boost::unordered_map<Segment_3_undirected, list<Point_3>> &edge2points, Out_iter output);

} // namespace cvc::tiler

#endif

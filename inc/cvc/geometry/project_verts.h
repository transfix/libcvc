/*
  Copyright 2008 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolumeRover.

  VolumeRover is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolumeRover is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  Joe R. - transfix@ices.utexas.edu - project_verts.h - v0.1 - 10/16/2008

  This code simply projects a collection of input vertices to a triangulated surface.

  ForwardIterator_vertex - a ForwardIterator that iterates across vertex elements.
                           Each vertex element type should provide operator[] access to
                           vertex components.
  InputIterator_vertex   - an InputIterator that iterates across vertex elements.
                           Each vertex element type should provide operator[] access to
                           vertex components.
  InputIterator_tri      - an InputIterator that iterates across triangle elements
                           Each triangle element type should provide operator[] access to
                           triangle components.  Each triangle component should be an index
                           into the container pointed to by InputIterator_vertex

  Example:

  {
    typedef boost::array<double,3> point_t;
    typedef boost::array<int,3>    triangle_t;
    std::vector<point_t> input_verts, reference_verts;
    std::vector<triangle_t> reference_tris;

    //... fill up containers

    project_verts::project(input_verts.begin(),
                           input_verts.end(),
                           reference_verts.begin(),
                           reference_verts.end(),
                           reference_tris.begin(),
                           reference_tris.end());
  }
*/

#ifndef __PROJECT_VERTS_H__
#define __PROJECT_VERTS_H__

#include <CGAL/K_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/basic.h> //error handling stuff
#include <CGAL/exceptions.h>
#include <CGAL/squared_distance_3.h>
#include <algorithm>
#include <boost/array.hpp>
#include <iterator>
#include <list>
#include <map>
#include <vector>

namespace project_verts {
template <class ForwardIterator_vertex, class InputIterator_vertex, class InputIterator_tri>
inline void
project(ForwardIterator_vertex input_verts_begin, ForwardIterator_vertex input_verts_end,
        InputIterator_vertex reference_verts_begin, InputIterator_vertex reference_verts_end,
        InputIterator_tri reference_tris_begin, InputIterator_tri reference_tris_end) {
  using namespace std;

  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point_d;
  typedef K::Triangle_3 Triangle_3;
  typedef K::Plane_3 Plane_3;
  typedef K::Point_2 Point_2;
  typedef K::Segment_2 Segment_2;
  typedef CGAL::Search_traits_3<K> TreeTraits;
  typedef CGAL::K_neighbor_search<TreeTraits> Neighbor_search;
  typedef Neighbor_search::Tree Tree;

  typedef boost::array<unsigned int, 3> triangle_t;

  Tree *tree;

  vector<Point_d> reference_verts;
  vector<triangle_t> reference_tris;

  {
    for (InputIterator_vertex i = reference_verts_begin; i != reference_verts_end; i++)
      reference_verts.push_back(Point_d((*i)[0], (*i)[1], (*i)[2]));
    tree = new Tree(reference_verts.begin(), reference_verts.end());
  }

  // copy reference tris and find all the triangles that each point is a part of
  map<Point_d, list<unsigned int>> neighbor_tris;
  for (InputIterator_tri i = reference_tris_begin; i != reference_tris_end; i++) {
    triangle_t tri;
    copy(i->begin(), i->end(), tri.begin());
    reference_tris.push_back(tri);
    for (triangle_t::iterator j = tri.begin(); j != tri.end(); j++)
      neighbor_tris[reference_verts[*j]].push_back(distance(reference_tris_begin, i));
  }

  // now iterate through all input vertices and project them to the closest face
  // of reference geom
  for (ForwardIterator_vertex i = input_verts_begin; i != input_verts_end; i++) {
    Point_d query((*i)[0], (*i)[1], (*i)[2]);
    Neighbor_search search(
        *tree, query,
        10); // instead of 10, consider using reference_verts.size() if it's not too slow...

    if (search.begin() == search.end())
      throw runtime_error("Neighbor search yielded zero results. Kd-tree empty?");

    Point_d closest_point_with_tris = search.begin()->first;
    for (Neighbor_search::iterator ni = search.begin();
         ni != search.end() && neighbor_tris[closest_point_with_tris].empty(); ni++)
      closest_point_with_tris = ni->first;

    list<unsigned int> &closest_tris = neighbor_tris[closest_point_with_tris];
    if (closest_tris.empty())
      continue; // this might happen if we have a lot of free points

    // now find the closest tri in the list via the closest point on each tri to the query
    unsigned int tri_idx = closest_tris.front();
    Triangle_3 closest_tri(reference_verts[reference_tris[tri_idx][0]],
                           reference_verts[reference_tris[tri_idx][1]],
                           reference_verts[reference_tris[tri_idx][2]]);
    Point_d closest_point = closest_tri.supporting_plane().projection(query);
    if (!closest_tri.has_on(closest_point)) {
      // if the point isn't within the triangle after projection,
      // move it to the closest point on the triangle
      // which would be on one of the triangle's edges
      Plane_3 closest_tri_plane = closest_tri.supporting_plane();
      Segment_2 tri_lines[3] = {Segment_2(closest_tri_plane.to_2d(closest_tri[0]),
                                          closest_tri_plane.to_2d(closest_tri[1])),
                                Segment_2(closest_tri_plane.to_2d(closest_tri[1]),
                                          closest_tri_plane.to_2d(closest_tri[2])),
                                Segment_2(closest_tri_plane.to_2d(closest_tri[2]),
                                          closest_tri_plane.to_2d(closest_tri[0]))};

      Point_2 closest_points_on_current_line[3];
      for (int j = 0; j < 3; j++) {
        Point_2 P0 = tri_lines[j].source();
        Point_2 P1 = tri_lines[j].target();
        Point_2 PS = closest_tri_plane.to_2d(query);
        double r =
            -(((P1.x() - P0.x()) * (P0.x() + PS.x()) + (P1.y() - P0.y()) * (P0.y() + PS.y())) /
              CGAL::squared_distance(P1, P0));
        r = max(1.0, min(0.0, r)); // clamp r so our point is on the tri segment
        closest_points_on_current_line[j] = P0 + r * (P1 - P0);
      }

      // calculate distances for the 3 closest points and grab the absolute closest
      map<double, Point_2> distances;
      for (int j = 0; j < 3; j++)
        distances[CGAL::squared_distance(closest_points_on_current_line[j],
                                         closest_tri_plane.to_2d(query))] =
            closest_points_on_current_line[j];
      closest_point = closest_tri_plane.to_3d(distances.begin()->second);
    }

    for (list<unsigned int>::iterator j = closest_tris.begin(); j != closest_tris.end(); j++) {
      Triangle_3 current_tri(reference_verts[reference_tris[*j][0]],
                             reference_verts[reference_tris[*j][1]],
                             reference_verts[reference_tris[*j][2]]);
      Point_d new_closest_point = current_tri.supporting_plane().projection(query);
      if (!current_tri.has_on(new_closest_point)) {
        Plane_3 current_tri_plane = current_tri.supporting_plane();
        Segment_2 tri_lines[3] = {Segment_2(current_tri_plane.to_2d(current_tri[0]),
                                            current_tri_plane.to_2d(current_tri[1])),
                                  Segment_2(current_tri_plane.to_2d(current_tri[1]),
                                            current_tri_plane.to_2d(current_tri[2])),
                                  Segment_2(current_tri_plane.to_2d(current_tri[2]),
                                            current_tri_plane.to_2d(current_tri[0]))};
        Point_2 closest_points_on_current_line[3];
        for (int j = 0; j < 3; j++) {
          Point_2 P0 = tri_lines[j].source();
          Point_2 P1 = tri_lines[j].target();
          Point_2 PS = current_tri_plane.to_2d(query);
          double r =
              -(((P1.x() - P0.x()) * (P0.x() + PS.x()) + (P1.y() - P0.y()) * (P0.y() + PS.y())) /
                CGAL::squared_distance(P1, P0));
          r = max(1.0, min(0.0, r)); // clamp r so our point is on the tri segment
          closest_points_on_current_line[j] = P0 + r * (P1 - P0);
        }

        // calculate distances for the 3 closest points and grab the absolute closest
        map<double, Point_2> distances;
        for (int j = 0; j < 3; j++)
          distances[CGAL::squared_distance(closest_points_on_current_line[j],
                                           current_tri_plane.to_2d(query))] =
              closest_points_on_current_line[j];
        new_closest_point = current_tri_plane.to_3d(distances.begin()->second);
      }

      if (CGAL::squared_distance(new_closest_point, query) <
          CGAL::squared_distance(closest_point, query))
        closest_point = new_closest_point;
    }

    (*i)[0] = closest_point.x();
    (*i)[1] = closest_point.y();
    (*i)[2] = closest_point.z();
  }

  delete tree;
}
} // namespace project_verts

#endif

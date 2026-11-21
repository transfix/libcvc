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

#ifndef __TILER_OPERATIONS_H__
#define __TILER_OPERATIONS_H__

#include "Contour.h"
#include "Correspondences.h"
#include "Hierarchy.h"
#include "Tiler_workspace.h"
#include "Tiles.h"
#include "Tiling_region.h"
#include "Vertex_completion_map.h"
#include "Vertex_map.h"
#include "contour_utils.h"
#include "stepping.h"
#include "tiler_defines.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <list>
#include <map>

namespace cvc::tiler {

/// Augments the given polygons in place
void augment(Polygon_2 &P, Polygon_2 &Q);

/// Outputs augmented contours
/// ContourIterator - iterator over contours
//
// A pair of contours augmented at a time, charged by their vertices (stepping.h); `augmented` is
// the legacy return value once it ends. The iterators' contours must outlive the task.
template <typename ContourIterator, typename OutputIterator>
step_task augment_steps(ContourIterator bottom_start, ContourIterator bottom_end,
                        ContourIterator top_start, ContourIterator top_end,
                        OutputIterator bottom_aug, OutputIterator top_aug, bool &augmented) {
  using namespace std;

  typedef list<Contour_handle> Container;
  typedef Container::iterator iterator;

  bool ret = false;

  list<Contour_handle> top, bottom;
  for (ContourIterator it = top_start; it != top_end; ++it) {
    Contour_handle copy = (*it)->copy();
    top.push_back(copy);
    *top_aug = copy;
    ++top_aug;
  }
  for (ContourIterator it = bottom_start; it != bottom_end; ++it) {
    Contour_handle copy = (*it)->copy();
    bottom.push_back(copy);
    *bottom_aug = copy;
    ++bottom_aug;
  }

  for (iterator top_it = top.begin(); top_it != top.end(); ++top_it) {
    for (iterator bottom_it = bottom.begin(); bottom_it != bottom.end(); ++bottom_it) {
      Polygon_2 &P = (*bottom_it)->polygon();
      Polygon_2 &Q = (*top_it)->polygon();
      Polygon_2 oldp = P;
      Polygon_2 oldq = Q;
      augment(P, Q);
      ret = ret || (oldp.size() < P.size() || oldq.size() < Q.size());
      TILER_UNITS(sweep_units(oldp.size() + oldq.size()));
    }
  }

  augmented = ret;
}

template <typename ContourIterator, typename OutputIterator>
bool augment(ContourIterator bottom_start, ContourIterator bottom_end, ContourIterator top_start,
             ContourIterator top_end, OutputIterator bottom_aug, OutputIterator top_aug) {
  bool augmented = false;
  run_to_end(
      augment_steps(bottom_start, bottom_end, top_start, top_end, bottom_aug, top_aug, augmented));
  return augmented;
}

/// Returns true if a tile can exist between two contours
/// according to the requirements given in Bajaj96, theorem 6.
// template <typename Contour_handle>
bool can_tile(Contour_handle c1, Contour_handle c2, const Hierarchy &h1, const Hierarchy &h2,
              Number_type overlap);

/// Finds all contours that can be tiled to the given contour
/// Yes, I know the return value looks scary, but it is simply a map of contour handles to a
/// list of contour handles:
//
// A pair of contours tested at a time, charged by their vertices (stepping.h).
template <typename ContourIterator>
step_task find_correspondences_steps(ContourIterator start1, ContourIterator end1,
                                     const Hierarchy &h1, ContourIterator start2,
                                     ContourIterator end2, const Hierarchy &h2, Number_type overlap,
                                     Correspondences &correspondences) {

  size_t count = 0;
  for (ContourIterator it1 = start1; it1 != end1; ++it1)
    for (ContourIterator it2 = start2; it2 != end2; ++it2) {
      if (can_tile(*it1, *it2, h1, h2, overlap)) {
        correspondences.add(*it1, *it2);
        ++count;
      }
      TILER_UNITS(overlap_test_units((*it1)->polygon().size() + (*it2)->polygon().size()));
    }
  TILER_TRACE(logger, "Number of correspondences: " << count);
}

template <typename ContourIterator>
Correspondences find_correspondences(ContourIterator start1, ContourIterator end1,
                                     const Hierarchy &h1, ContourIterator start2,
                                     ContourIterator end2, const Hierarchy &h2,
                                     Number_type overlap) {
  Correspondences correspondences;
  run_to_end(
      find_correspondences_steps(start1, end1, h1, start2, end2, h2, overlap, correspondences));
  return correspondences;
}

// template <typename Vertex_circulator, typename Hierarchy>
// Tiling_region tiling_region(Polygon_2::Vertex_circulator vertex, const Hierarchy& h);

/// ContourIterator iterates over contours
template <typename ContourIterator>
Vertex_map<HTiling_region>
find_tiling_regions(ContourIterator start1, ContourIterator end1, const Hierarchy &h1,
                    ContourIterator start2, ContourIterator end2, const Hierarchy &h2,
                    Number_type z1, Number_type z2, const Vertices &vertices);
/// The same, a contour vertex at a time, charged by the other slice's vertices it is located among.
template <typename ContourIterator>
step_task find_tiling_regions_steps(ContourIterator start1, ContourIterator end1,
                                    const Hierarchy &h1, ContourIterator start2,
                                    ContourIterator end2, const Hierarchy &h2, Number_type z1,
                                    Number_type z2, const Vertices &vertices,
                                    Vertex_map<HTiling_region> &regions);
// {
//   typedef Polygon_2::Vertex_circulator Vertex_circulator;

//   Vertex_map<Tiling_region> regions(1000);

//   for (ContourIterator it1 = start1; it1 != end1; ++it1)
//   {
//     Contour_handle contour = *it1;
//     Vertex_circulator beg = contour->polygon().vertices_circulator();
//     Vertex_circulator it = beg;
//     do
//     {
//       Point_3 v = vertices.get(contour, it);
//       regions[v] = tiling_region(it, h2);
//       ++it;
//     } while (it != beg);
//   }

//   for (ContourIterator it2 = start2; it2 != end2; ++it2)
//   {
//     Contour_handle contour = *it2;
//     Vertex_circulator beg = contour->polygon().vertices_circulator();
//     Vertex_circulator it = beg;
//     do
//     {
//       Point_3 v = vertices.get(contour, it);
//       regions[v] = tiling_region(it, h1);
//       ++it;
//     } while (it != beg);
//   }

//   return regions;
// }

// Used to compare distances from 2 points to another point
class Dist_cmp {
public:
  Dist_cmp(const Point_2 &p) : _point(p) {}
  //   Dist_cmp(const Point_3& v) : _point(v.point()) {}

  bool operator()(const Point_2 &a, const Point_2 &b) {
    return has_smaller_distance_to_point(_point, a, b);
  }

  //   bool operator()(const Point_3& a, const Point_3& b)
  //   {
  //     return has_smaller_distance_to_point(_point, a.point(), b.point());
  //   }

private:
  Point_2 _point;
};

/// ContourIterator iterates over contours
template <typename ContourIterator>
OTV_table build_OTV_table(ContourIterator contours_begin, ContourIterator contours_end,
                          ContourIterator c1_begin, ContourIterator c1_end,
                          ContourIterator c2_begin, ContourIterator c2_end,
                          const Vertices &vertices, const Correspondences &correspondences,
                          const Vertex_map<HTiling_region> &tiling_regions, const Hierarchies &h,
                          Banned &banned);
/// The same, a contour vertex at a time (its candidates sorted) and a candidate tested at a time,
/// each charged by its size (stepping.h).
template <typename ContourIterator>
step_task build_OTV_table_steps(ContourIterator contours_begin, ContourIterator contours_end,
                                ContourIterator c1_begin, ContourIterator c1_end,
                                ContourIterator c2_begin, ContourIterator c2_end,
                                const Vertices &vertices, const Correspondences &correspondences,
                                const Vertex_map<HTiling_region> &tiling_regions,
                                const Hierarchies &h, Banned &banned, OTV_table &otv_table);

// {
//   typedef Correspondences::const_iterator corr_iterator;
//   typedef Polygon_2::Vertex_iterator Vertex_iterator;

//   using namespace std;

//   OTV_table otv_table;

//   for (ContourIterator it = contours_begin; it != contours_end; ++it)
//   {
//     Contour_handle contour = *it;

//     // get vertices of all contours that corresponding to the current contour
//     vector<Point_3> points;
//     for (corr_iterator cit = correspondences.begin(contour); cit != correspondences.end(contour);
//     ++cit)
//       vertices.get_vertices(*cit, back_inserter(points));

//     // loop through each vertex v on current contour; sort all corresponding vertices
//     // according to distance from v.
//     for (Vertex_iterator vit = contour->polygon().vertices_begin(); vit !=
//     contour->polygon().vertices_end(); ++vit)
//     {
//       Point_3 vertex = vertices.get(contour, vit);
//       sort(points.begin(), points.end(), Dist_cmp(vertex));

//       // loop through sorted vertices until we find a valid point
//       bool found = false;
//       for (typename vector<Point_3>::const_iterator pit = points.begin(); !found && pit !=
//       points.end(); ++pit)
//       {
// 	const Point_3& test_vertex = *pit;
// 	Point_2 test_point = test_vertex;//test_vertex.point();
// 	found = (test_point == vertex.point());
// 	if (!found)
// 	{
// 	  Segment_2 segment(vertex.point(), test_point);
// 	  found = (tiling_regions[vertex].contains(test_point) &&
// 		   !intersects_proper(segment, c1_begin, c1_end) &&
// 		   !intersects_proper(segment, c2_begin, c2_end));
// 	}
// 	if (found)
// 	  otv_table[vertex] =test_vertex;
//       }
//     }
//   }

//   return otv_table;
// }

bool is_OTV_pair(const Point_3 &v1, const Point_3 &v2, const OTV_table &otv_table);

/// See discussion of 6 cases of optimality in bajaj96.
int optimality(const Point_3 &u2, const Point_3 &u3, const Point_3 &v2, const Point_3 &v3,
               const Tiler_workspace &w);

void find_banned(Tiler_workspace &w);

//
// Units: the hierarchies, the vertices and the workspace, each charged by the contours' vertices
// (stepping.h), and those of find_correspondences_steps, find_tiling_regions_steps and
// build_OTV_table_steps. The workspace is `w` once it returns.
template <typename ContourIterator>
step_task pre_tile_steps(ContourIterator bottom_start, ContourIterator bottom_end,
                         ContourIterator top_start, ContourIterator top_end, bool do_augment,
                         Number_type overlap, boost::shared_ptr<Tiler_workspace> &w) {
  typedef std::vector<Contour_handle> Contours;

  Contours bottom, top, all;
  Hierarchy bottom_h, top_h;
  Hierarchies hierarchies;
  Correspondences correspondences;
  Vertices vertices;
  // (Each Vertex_map holds 10000 entries, whatever the contours.)
  Vertex_map<HTiling_region> tiling_regions;
  TILER_UNITS(vertex_map_units());
  OTV_table otv_table;
  TILER_UNITS(vertex_map_units());

  Number_type bottomz;
  if (bottom_start != bottom_end) {
    bottomz = (*bottom_start)->polygon()[0].z();
  } else {
    bottomz = (*top_start)->polygon()[0].z() - 1;
  }
  Number_type topz;
  if (top_start != top_end) {
    topz = (*top_start)->polygon()[0].z();
  } else {
    topz = bottomz + 1;
  }

  //   Number_type bottomz = (*bottom_start)->polygon()[0].z();
  //   Number_type topz = (*top_start)->polygon()[0].z();
  Number_type midz = (topz + bottomz) / 2.0;

  if (do_augment) {
    TILER_TRACE(logger, "Augmenting contours...");
    augment(bottom_start, bottom_end, top_start, top_end, back_inserter(bottom),
            back_inserter(top));
  } else {
    bottom.insert(bottom.end(), bottom_start, bottom_end);
    top.insert(top.end(), top_start, top_end);
  }

  all.insert(all.end(), bottom.begin(), bottom.end());
  all.insert(all.end(), top.begin(), top.end());
  std::size_t bottom_n = 0, top_n = 0;
  for (const Contour_handle &c : bottom)
    bottom_n += c->polygon().size();
  for (const Contour_handle &c : top)
    top_n += c->polygon().size();
  SK_UNIT;

  TILER_TRACE(logger, "Creating hierarchies...");
  TILER_TRACE(logger, "Creating bottom augmented hierarchy...");
  bottom_h = Hierarchy(bottom.begin(), bottom.end(), Hierarchy_policy::FORCE_CCW);
  TILER_UNITS(hierarchy_units(bottom_n));
  TILER_TRACE(logger, "Creating top augmented hierarchy...");
  top_h = Hierarchy(top.begin(), top.end(), Hierarchy_policy::FORCE_CCW);
  TILER_UNITS(hierarchy_units(top_n));

  //   hierarchies[bottom[0]->slice()] = bottom_h;
  //   hierarchies[top[0]->slice()] = top_h;
  hierarchies[bottomz] = bottom_h;
  hierarchies[topz] = top_h;

  TILER_TRACE(logger, "Creating vertices...");
  vertices = Vertices::create(all.begin(), all.end());
  TILER_UNITS(table_units(bottom_n + top_n));

  // Get correspondences
  TILER_TRACE(logger, "Finding correspondences...");
  SK_RUN(find_correspondences_steps(bottom.begin(), bottom.end(), bottom_h, top.begin(), top.end(),
                                    top_h, overlap, correspondences));

  // Get the tiling regions
  TILER_TRACE(logger, "Finding tiling regions...");
  SK_RUN(find_tiling_regions_steps(bottom.begin(), bottom.end(), bottom_h, top.begin(), top.end(),
                                   top_h, bottomz, topz, vertices, tiling_regions));

  Banned banned;

  // Build the OTV table
  TILER_TRACE(logger, "Building OTV table...");
  SK_RUN(build_OTV_table_steps(all.begin(), all.end(), bottom.begin(), bottom.end(), top.begin(),
                               top.end(), vertices, correspondences, tiling_regions, hierarchies,
                               banned, otv_table));

  w = Tiler_workspace::create(all, bottom, top, correspondences, vertices, tiling_regions,
                              otv_table, hierarchies, midz);
  TILER_UNITS(table_units(bottom_n + top_n));

  // Finding banned vertices
  TILER_TRACE(logger, "Finding banned vertices...");
  w->set_banned(banned);
}

template <typename ContourIterator>
boost::shared_ptr<Tiler_workspace>
pre_tile(ContourIterator bottom_start, ContourIterator bottom_end, ContourIterator top_start,
         ContourIterator top_end, bool do_augment, Number_type overlap) {
  boost::shared_ptr<Tiler_workspace> w;
  run_to_end(pre_tile_steps(bottom_start, bottom_end, top_start, top_end, do_augment, overlap, w));
  return w;
}

void build_tiling_table(Tiler_workspace &workspace);

void build_tiling_table_phase1(Tiler_workspace &workspace);
/// An OTV pair at a time, and a chord step at a time in passes 2-4, each charged by the
/// workspace's vertices (stepping.h).
step_task build_tiling_table_phase1_steps(Tiler_workspace &workspace);

void build_tiling_table_phase2(Tiler_workspace &workspace);
/// An untiled region at a time (its partition charged by its size), and within one, a point of its
/// fill at a time (medial_axis_stable_steps).
step_task build_tiling_table_phase2_steps(Tiler_workspace &workspace);

} // namespace cvc::tiler

#endif

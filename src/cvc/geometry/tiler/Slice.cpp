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

#include "Slice.h"

#include "Distance_functor.h"
#include "polygon_utils.h"
#include "print_utils.h"
#include "remove_contour_intersections.h"

#include <sstream>

namespace cvc::tiler {

void Slice::remove_collinear(Number_type epsilon) {
  for (Map::iterator it = _map.begin(); it != _map.end(); ++it) {
    Contour_container &comp_contours = it->second;
    for (Contour_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      c->polygon() = cvc::tiler::remove_collinear(c->polygon(), epsilon);
    }
  }
}

void Slice::remove_intersections(Number_type delta, tiler_context &ctx) {
  Logger logger = ctx.logger("tiler.Slice.remove_intersections");

  // Put all polygons in the slice into polys, regardless of component.
  vector<Contour_handle> contours;
  list<Contour_handle> nested;

  // Add all contours to a hierarchy
  list<Contour_handle> all_contours;
  for (Map::iterator it = _map.begin(); it != _map.end(); ++it) {
    Contour_container &comp_contours = it->second;
    for (Contour_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      all_contours.push_back(c);
    }
  }
  // Hierarchy h(all_contours.begin(), all_contours.end(), Hierarchy_policy::FORCE_CCW);

  for (Map::iterator it = _map.begin(); it != _map.end(); ++it) {
    Contour_container &comp_contours = it->second;
    for (Contour_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      if (!c->polygon().is_simple()) {
        throw logic_error("Contour is not simple before intersection removal...");
      }
      // TILER_TRACE(logger, "Contour is at level " << h.level(c));
      // if (!c->is_counterclockwise_oriented())
      // if (h.parent(c)) {
      // 	nested.push_back(c);
      // 	// c->reverse_orientation();
      // 	TILER_DEBUG(logger, "Nested polygon -- not included in remove_intersections: " <<
      // pp(c->polygon()));
      // }
      // else {
      contours.push_back(c);
      // }
    }
  }

  // Remove all contour intersections by dilating, removing
  // intersections, then eroding.
  list<Contour_handle> new_contours;
  remove_contour_intersections(contours.begin(), contours.end(), delta, back_inserter(new_contours),
                               ctx);

  _map.clear();
  // insert(new_contours.begin(), new_contours.end());
  for (list<Contour_handle>::const_iterator it = new_contours.begin(); it != new_contours.end();
       ++it) {
    Contour_handle ch = *it;
    if (ch->polygon().size() < 3) {
      TILER_WARN(logger, "Contour " << ch->info().name()
                                    << " is not simple after intersection removal: "
                                    << pp(ch->polygon()) << " -- skipping");
    } else if (!ch->polygon().is_simple()) {
      TILER_WARN(logger, "Contour " << ch->info().name()
                                    << " is not simple after intersection removal: "
                                    << pp(ch->polygon()) << " -- skipping");
    } else {
      push_back(ch);
    }

    // if (ch->info().name() == "a359") {
    //   TILER_TRACE(logger, "a359: " << pp(ch->polygon()));
    // }
  }

  insert(nested.begin(), nested.end());
}

void Slice::validate() {

  for (Map::iterator it = _map.begin(); it != _map.end(); ++it) {
    Contour_container &comp_contours = it->second;
    for (Contour_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      if (!c->polygon().is_simple()) {
        throw logic_error("Contour is not simple: component " + c->info().object_name() + ": " +
                          pp(c->polygon()));
      }
    }
  }
}

Number_type Slice::z() const {
  for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it) {
    const Contour_container &comp_contours = it->second;
    for (Contour_const_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      return c->slice();
    }
  }
}

void Slice::augment(const boost::unordered_map<Segment_3_undirected, list<Point_3>> &edge2points) {
  for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it) {
    const Contour_container &comp_contours = it->second;
    for (Contour_const_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      Polygon_2 &p = c->polygon();
      Polygon_2 newp;
      Polygon_2::Edge_const_circulator begin = p.edges_circulator();
      Polygon_2::Edge_const_circulator ci = begin;
      do {
        Segment_2 seg = *ci;
        Segment_3_undirected useg(Segment_3(seg.source(), seg.target()));
        if (edge2points.find(useg) == edge2points.end()) {
          newp.push_back(seg.source());
        } else {
          // There are points on this edge that need to be augmented.
          list<Point_3> points = edge2points.find(useg)->second;
          points.sort(Distance_functor<Point_3>(seg.source()));
          points.push_front(seg.source());
          list<Point_3>::iterator new_end = unique(points.begin(), points.end());
          new_end = remove(points.begin(), new_end, seg.target());
          for (list<Point_3>::iterator p_it = points.begin(); p_it != new_end; ++p_it) {
            newp.push_back(*p_it);
          }
        }
        ++ci;
      } while (ci != begin);
      p = newp;
    }
  }
}

string Slice::to_string() const {
  stringstream ss;
  ss << "z = " << z() << "; ";
  for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it) {
    const string &component = it->first;
    ss << component << ": ";
    const Contour_container &comp_contours = it->second;
    for (Contour_const_iterator c_it = comp_contours.begin(); c_it != comp_contours.end(); ++c_it) {
      Contour_handle c = *c_it;
      ss << pp(c->polygon()) << " ";
    }
    ss << "; ";
  }
  return ss.str();
}

} // namespace cvc::tiler

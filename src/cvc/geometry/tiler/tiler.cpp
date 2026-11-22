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

#include "tiler.h"

#include "cut.h"
#include "intersection.h"
#include "print_utils.h"
#include "set_utils.h"
#include "sweep_line_visitors.h"
#include "tiler_operations.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace cvc::tiler {

//------------------------------------------------------------------------------
// tile_one_component
//
/// Tiles one component across two slices
//------------------------------------------------------------------------------
//
// The workspace is `workspace` once it returns. Its units are pre_tile_steps',
// build_tiling_table_phase1_steps' and build_tiling_table_phase2_steps'.
template <typename Contour_iter>
step_task tile_one_component(Contour_iter bottom_begin, Contour_iter bottom_end,
                             Contour_iter top_begin, Contour_iter top_end,
                             const Tiler_options &options, tiler_context &ctx,
                             TW_handle &workspace) {
  using namespace std;

  // Pre-tile: augment, generate hierarchy, get correspondences, build otv table
  SK_RUN(pre_tile_steps(bottom_begin, bottom_end, top_begin, top_end, false,
                        options.correspondence_overlap(), workspace));
  Tiler_workspace &w = *workspace;
  // (pre_tile's tables, freed as it returned.)
  TILER_UNITS(table_units(w.vertices.size()));
  w.set_log(&ctx.log);

  // Perform tiling phase 1
  boost::shared_ptr<Tiler_callback> callback(new Tiler_callback());
  w.set_callback(callback);
  SK_RUN(build_tiling_table_phase1_steps(w));

  // Perform tiling phase 2
  SK_RUN(build_tiling_table_phase2_steps(w)); //, options.interp_untiled_regions());
}

class Tiler_intersection {
public:
  Tiler_intersection(size_t a_id, size_t b_id, Number_type a_z, Number_type b_z)
      : _a_id(a_id), _b_id(b_id), _a_z(a_z), _b_z(b_z) {}
  ~Tiler_intersection() {}

  size_t a_id() const { return _a_id; }
  size_t b_id() const { return _b_id; }
  Number_type a_z() const { return _a_z; }
  Number_type b_z() const { return _b_z; }

  bool operator==(const Tiler_intersection &t) const {
    return _a_id == t._a_id && _b_id == t._b_id && _a_z == t._a_z && _a_z == t._a_z;
  }

private:
  size_t _a_id, _b_id;
  Number_type _a_z, _b_z;
};

std::size_t hash_value(const Tiler_intersection &t) {
  std::size_t seed = 0;
  boost::hash_combine(seed, t.a_id());
  boost::hash_combine(seed, t.b_id());
  boost::hash_combine(seed, t.a_z());
  boost::hash_combine(seed, t.b_z());
  return seed;
}

//------------------------------------------------------------------------------
// get_intersections
//
/// Gets intersections between multiple components using the sweep line
/// algorithm.  pairs will contain pairs of component IDs representing components
/// that intersect each other.  The component IDs match the iteration order of
/// tw_begin and tw_end.
//------------------------------------------------------------------------------
template <typename TW_iter, typename Out_iter>
void get_intersections(TW_iter tw_begin, TW_iter tw_end, Out_iter pairs) {
  vector<Segment_2> segments;
  vector<SL_point> points;
  size_t comp_id = 0;
  for (TW_iter it = tw_begin; it != tw_end; ++it) {
    TW_handle tw = *it;
    for (Tile_iterator tile_it = tw->tiles_begin(); tile_it != tw->tiles_end(); ++tile_it) {
      const Triangle &tile = **tile_it;
      for (int i = 0; i < 3; ++i) {
        Segment_3 s_3 = edge(i, tile);
        Segment_2 s_2(s_3.source(), s_3.target());
        Segment_2_undirected s_u(s_2);
        segments.push_back(s_2);
        points.push_back(SL_point(s_2.source(), segments.size() - 1, s_u, comp_id));
      }
    }
    ++comp_id;
  }

  std::stable_sort(points.begin(), points.end()); // ties in input order

  typedef list<SL_intersection> Intersection_list;
  typedef Intersection_visitor<back_insert_iterator<Intersection_list>> Visitor;
  Intersection_list intersections;
  Visitor v(back_inserter(intersections));
  sweep_line_multi_comp(points.begin(), points.end(), segments, comp_id, v);

  vector<TW_handle> tws(tw_begin, tw_end);
  for (list<SL_intersection>::iterator it = intersections.begin(); it != intersections.end();
       ++it) {
    size_t a_id, b_id;
    boost::tie(a_id, b_id) = it->components();
    Tiler_intersection ti(a_id, b_id, 0, 1);
    *pairs++ = ti;
  }
}

//------------------------------------------------------------------------------
// augment_slice_pair
//
/// Augments the contours of two consecutive slices in place: one step of the
/// legacy augment_slices(), which ran this over every pair in order.
//------------------------------------------------------------------------------
//
// A contour validated, or a pair of contours augmented, at a time (stepping.h);
// `augmented_some_point` is the legacy return value once it ends.
step_task augment_slice_pair_steps(Slice &bottom, Slice &top, bool &augmented_some_point) {
  augmented_some_point = false;

  SK_RUN(bottom.validate_steps());
  SK_RUN(top.validate_steps());

  // Get all components in the two slices
  vector<string> components;
  {
    boost::unordered_set<string> bottom_components, top_components;
    bottom.components(inserter(bottom_components, bottom_components.end()));
    top.components(inserter(top_components, top_components.end()));
    set_intersection(bottom_components, top_components, back_inserter(components));
  }

  // For each component, augment the component's contours.
  for (vector<string>::const_iterator it = components.begin(); it != components.end(); ++it) {
    const string &component = *it;

    // Put all of the component's contours in
    // bc (bottom) and tc (top).
    list<Contour_handle> bc, tc;
    bc.insert(bc.end(), bottom.begin(component), bottom.end(component));
    tc.insert(tc.end(), top.begin(component), top.end(component));

    // Augment the contours, putting the results in bac and tac.
    list<Contour_handle> bac, tac;
    bool b = false;
    SK_RUN(augment_steps(bc.begin(), bc.end(), tc.begin(), tc.end(), back_inserter(bac),
                         back_inserter(tac), b));
    augmented_some_point = augmented_some_point || b;

    // Update the slices with the augmented contours.
    bottom.replace(component, bac.begin(), bac.end());
    top.replace(component, tac.begin(), tac.end());
  }
  SK_RUN(bottom.validate_steps());
  SK_RUN(top.validate_steps());
}

namespace {

std::size_t vertex_count(const Slice &s) {
  std::size_t n = 0;
  list<string> components;
  s.components(back_inserter(components));
  for (const string &c : components)
    for (Slice::Contour_const_iterator it = s.begin(c); it != s.end(c); ++it)
      n += (*it)->polygon().size();
  return n;
}

} // namespace

//------------------------------------------------------------------------------
// tile
//
/// Tiles multiple components across multiple slices
//------------------------------------------------------------------------------
step_task tile(vector<Slice> &slices, const Tiler_options &options, tiler_context &ctx,
               tile_output &out) {
  using namespace std;
  Logger logger = ctx.logger("tiler.tile");

  typedef boost::shared_ptr<Triangle> Triangle_handle;

  map<string, list<Triangle>> triangles_by_component;

  // Shares of the progress bar, by the work each phase does.
  const double curate_end = 0.15, augment_end = 0.2, tile_end = 0.9;
  std::size_t total_vertices = 0;
  for (const Slice &s : slices)
    total_vertices += vertex_count(s);
  if (total_vertices == 0)
    total_vertices = 1;

  // Contours after intersection removal may be very slightly different
  // even if there were no intersections since intersection removal
  // performs an expansion and contraction of each contour.
  ctx.stage("curating", 0.05, curate_end);
  TILER_INFO(logger, "Removing 2D contour intersections");
  std::size_t done_vertices = 0;
  for (vector<Slice>::iterator it = slices.begin(); it != slices.end(); ++it) {
    Slice &s = *it;
    done_vertices += vertex_count(s);
    SK_RUN(s.validate_steps());
    SK_RUN(s.remove_collinear_steps(options.collinear_epsilon()));
    SK_RUN(s.validate_steps());

    if (options.contour_curation_delta() > 0) {
      SK_RUN(s.remove_intersections_steps(options.contour_curation_delta(), ctx));
      SK_RUN(s.validate_steps());
    }
    ctx.advance(double(done_vertices) / double(total_vertices));
    SK_UNIT;
  }

  // Augment every pair of consecutive slices, in order, until a pass adds no point. Progress
  // counts the first pass's pairs; a pass after it (rare) adds none.
  ctx.stage("augmenting", curate_end, augment_end);
  for (bool augmented = true; augmented;) {
    augmented = false;
    for (std::size_t i = 0; i + 1 < slices.size(); ++i) {
      bool b = false;
      SK_RUN(augment_slice_pair_steps(slices[i], slices[i + 1], b));
      augmented = b || augmented;
      ctx.advance(double(i + 1) / double(slices.size() - 1));
      SK_UNIT;
    }
  }

  // Iterate through the slices
  ctx.stage("tiling", augment_end, tile_end);
  boost::unordered_map<Segment_3_undirected, list<Point_3>> edge2points;
  for (std::size_t i = 0; i + 1 < slices.size(); ++i) {
    Slice &bottom_slice = slices[i];
    const Slice &bottom = slices[i];
    const Slice &top = slices[i + 1];

    if (!bottom.empty() || !top.empty()) {
      if (bottom.empty()) {
        TILER_INFO(logger, "Tiling slice " << top.z());
        ctx.label = "slice " + boost::lexical_cast<string>(top.z());
      } else if (top.empty()) {
        TILER_INFO(logger, "Tiling slice " << bottom.z());
        ctx.label = "slice " + boost::lexical_cast<string>(bottom.z());
      } else {
        TILER_INFO(logger, "Tiling slices " << bottom.z() << " - " << top.z());
        ctx.label = "slices " + boost::lexical_cast<string>(bottom.z()) + " - " +
                    boost::lexical_cast<string>(top.z());
      }
    }

    bottom_slice.augment(edge2points);

    boost::unordered_set<string> bottom_components, top_components;
    bottom.components(inserter(bottom_components, bottom_components.end()));
    top.components(inserter(top_components, top_components.end()));
    vector<string> components;
    set_union(bottom_components, top_components, back_inserter(components));

    vector<TW_handle> slice_workspaces;
    vector<list<Triangle_handle>> tiles;

    // Tile each component in between these two slices.  Add resultant workspaces to
    // slice_workspaces and resultant tiles to tiles.
    for (vector<string>::const_iterator it = components.begin(); it != components.end(); ++it) {
      const string &component = *it;

      TW_handle w;
      SK_RUN(tile_one_component(bottom.begin(component), bottom.end(component),
                                top.begin(component), top.end(component), options, ctx, w));

      slice_workspaces.push_back(w);
      list<Triangle_handle> t(w->tiles_begin(), w->tiles_end());
      tiles.push_back(t);
      TILER_UNITS(copy_units(2 * t.size()));
    }

    // After tiling is complete, remove any intersections between slices
    // that were created.
    if (options.remove_intersections()) {
      if (!bottom.empty() && !top.empty()) {
        TILER_INFO(logger,
                   "Removing intersections between slices " << bottom.z() << " - " << top.z());
      }

      // Find all tiles that intersect between components
      boost::unordered_set<Tiler_intersection> intersections;
      get_intersections(slice_workspaces.begin(), slice_workspaces.end(),
                        inserter(intersections, intersections.end()));

      // For each intersecting component pair, remove intersections
      for (const auto &ti : intersections) {
        size_t a = ti.a_id(), b = ti.b_id();
        TW_handle twa = slice_workspaces[a];
        TW_handle twb = slice_workspaces[b];

        vector<Triangle_handle> new_yellow, new_green;
        Number_type yz = ti.a_z();
        Number_type gz = ti.b_z();
        Number_type zmin = min(twa->zmin(), twb->zmin());
        Number_type zmax = max(twa->zmax(), twb->zmax());
        Number_type zdelta =
            fabs(zmax - zmin) * options.contour_curation_delta() / options.z_scale();

        if (zmin != zmax) {
          try {
            remove_intersections(slice_workspaces[a], slice_workspaces[b], tiles[a].begin(),
                                 tiles[a].end(), tiles[b].begin(), tiles[b].end(), yz, gz,
                                 back_inserter(new_yellow), back_inserter(new_green), zdelta,
                                 edge2points);

            tiles[a].clear();
            tiles[a].insert(tiles[a].end(), new_yellow.begin(), new_yellow.end());
            tiles[b].clear();
            tiles[b].insert(tiles[b].end(), new_green.begin(), new_green.end());
          } catch (std::logic_error &e) {
            TILER_ERROR(logger, "Failed to remove intersections between "
                                    << components[a] << " - " << components[b] << " slices " << zmin
                                    << " - " << zmax << ": " << e.what());
          }
        } else {
          TILER_ERROR(
              logger,
              "Intersection occurred on same slice.  Failing to remove intersections between "
                  << components[a] << " (" << twa->zmin() << " - " << twa->zmax() << ") and "
                  << components[b] << " (" << twb->zmin() << " - " << twb->zmax() << ")");
        }
        SK_UNIT;
      }
    }

    for (std::size_t j = 0; j < slice_workspaces.size(); ++j) {
      const list<Triangle_handle> &t = tiles[j];
      const string &component = components[j];
      for (list<Triangle_handle>::const_iterator th_it = t.begin(); th_it != t.end(); ++th_it)
        triangles_by_component[component].push_back(**th_it);
      TILER_UNITS(copy_units(t.size()));
    }

    // The pair's workspaces, freed one at a time.
    for (std::size_t j = 0; j < slice_workspaces.size(); ++j) {
      const std::size_t n = slice_workspaces[j]->vertices.size(), m = tiles[j].size();
      tiles[j].clear();
      slice_workspaces[j].reset();
      TILER_UNITS(table_units(n) + m);
    }

    ctx.advance(double(i + 1) / double(slices.size() - 1));
  }
  ctx.label.clear();

  // Put tiles into the output, taking into consideration any tiles that need
  // to be augmented and decomposed due to inducing points on an edge that
  // exists on a slice.
  ctx.stage("assembling", tile_end, 0.95);
  std::size_t total_triangles = 0, done_triangles = 0;
  for (const auto &entry : triangles_by_component)
    total_triangles += entry.second.size();
  for (auto &entry : triangles_by_component) {
    const string &component = entry.first;
    list<Triangle> &triangles = entry.second;
    list<Triangle> &result = out.triangles[component];
    // Each triangle is erased once done, so none is left for one unit to free at the end.
    for (list<Triangle>::iterator it = triangles.begin(); it != triangles.end();
         it = triangles.erase(it)) {
      const Triangle &tri = *it;

      // Augment and decompose triangles as necessary.  New triangles
      // are appended to the end of the list.
      list<Triangle> new_triangles;
      bool failed = false;
      try {
        decompose_triangle(tri, edge2points, back_inserter(new_triangles));
      } catch (logic_error &e) {
        TILER_ERROR(logger, "Decomposing of triangle failed: " << e.what());
        failed = true;
      }
      if (!failed && new_triangles.size() > 1) {
        for (list<Triangle>::const_iterator t_it = new_triangles.begin();
             t_it != new_triangles.end(); ++t_it) {
          triangles.push_back(*t_it);
        }
      } else {
        result.push_back(tri);
      }
      ++done_triangles;
      if (total_triangles > 0)
        ctx.advance(double(done_triangles) / double(total_triangles));
      SK_UNIT;
    }
  }
}

} // namespace cvc::tiler

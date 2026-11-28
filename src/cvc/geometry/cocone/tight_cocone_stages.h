/*
  Copyright 2007-2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (TightCocone)
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// Tight Cocone's stages for the ports that start from them (VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4):
// cvc::tight_cocone (V1), and next surface curation (V2) and super-secondary structures (V3),
// which run their stable manifolds on the marked triangulation these stages leave.
//
//   tight_cocone_front    TightCocone's surfaceReconstruction() up to its surface: optionally
//                         Robust Cocone, then the Delaunay triangulation, the Voronoi vertices and
//                         Tight Cocone's marking (every cell's `outside`, the surface facets'
//                         cocone flags on both sides)
//   extract_surface       op.cpp's wt_to_geometry: the flagged facets as triangles, each facing
//                         out of its inside cell

#ifndef CVC_GEOMETRY_COCONE_TIGHT_COCONE_STAGES_H
#define CVC_GEOMETRY_COCONE_TIGHT_COCONE_STAGES_H

#include "cocone.h"
#include "triangulate.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tight_cocone.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc::cocone {

// What tight_cocone_front did besides marking the triangulation.
struct front_counts {
  std::size_t duplicates = 0;     // input points merged as repeats (by either triangulation)
  std::size_t robust_dropped = 0; // distinct points Robust Cocone left out
  // Per point of the triangulation left (a vertex's `tc`): its index in the input. Empty when the
  // two are the same (no Robust Cocone).
  std::vector<std::size_t> input_of_point;
};

// TightCocone's surfaceReconstruction() up to its surface, on pts with P, its progress spread over
// [lo, hi]. Phases as tight_cocone_stepper lists them. Errors are Errors, "<who>: ...". The
// legacy robust mode's bounding box is not kept: nothing on this path reads it.
template <class Error>
step_task tight_cocone_front(std::vector<std::array<double, 3>> pts, tight_cocone_params P,
                             std::string who, Triangulation &triang, diagnostics &diag,
                             stage_progress &sp, double lo, double hi, front_counts &counts) {
  const double f = hi - lo;
  double at = lo;
  if (P.robust) {
    // 1. Robust Cocone on the triangulation of every point: the points on a smooth surface.
    SK_RUN(triangulate<Error>(std::move(pts), who, triang, sp, at, at + 0.15 * f,
                              counts.duplicates));
    at += 0.17 * f;
    sp.stage("voronoi", at, at + 0.03 * f, triang.num_cells() + triang.number_of_vertices());
    SK_RUN(initialize(triang));
    SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
    at += 0.03 * f;
    sp.stage("robust cocone", at, at + 0.05 * f, 6 * triang.num_cells());
    std::vector<Point> kept;
    SK_RUN(robust_cocone(P.big_ball_ratio, P.theta_ff, P.theta_if, triang, kept));
    at += 0.05 * f;
    counts.robust_dropped = triang.number_of_vertices() - kept.size();
    // robust_cocone lists the points it keeps in vertex order: the same walk gives their inputs.
    counts.input_of_point.clear();
    counts.input_of_point.reserve(kept.size());
    pts.clear();
    pts.reserve(kept.size());
    for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); vit++) {
      SK_UNIT;
      if (!vit->on_smooth_surface())
        continue;
      counts.input_of_point.push_back(std::size_t(vit->tc));
      const Point &p = vit->point();
      pts.push_back({p.x(), p.y(), p.z()});
    }
    if (pts.size() < 4) {
      char buf[256];
      std::snprintf(buf, sizeof buf,
                    "%s: Robust Cocone kept %zu points, fewer than the four a triangulation "
                    "needs; try robust = false or a larger big_ball_ratio",
                    who.c_str(), pts.size());
      throw Error(buf);
    }
  }

  // 2. The Delaunay triangulation, the Voronoi vertices and the cell radii.
  const double tri_span = (P.robust ? 0.15 : 0.25) * f;
  SK_RUN(triangulate<Error>(std::move(pts), who, triang, sp, at, at + tri_span,
                            counts.duplicates));
  at += tri_span + 0.02 * f;
  const std::size_t C = triang.num_cells(), V = triang.number_of_vertices();
  sp.stage("voronoi", at, at + 0.05 * f, C + V);
  SK_RUN(initialize(triang));
  SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
  at += 0.05 * f;

  // 3. Tight Cocone: the cells inside and outside, and the facets between them.
  sp.stage("tight cocone", at, hi, 16 * C);
  SK_RUN(tcocone(P.cocone_phi, P.sharp_phi, P.flat_phi, P.flatness_ratio, triang));
}

// What extract_surface found besides the surface.
struct surface_counts {
  std::size_t one_sided = 0; // flagged facets with both cells on one side (legacy: asserted none)
  std::size_t inside_cells = 0;
};

// op.cpp's wt_to_geometry: the finite facets whose cocone flag is set, as triangles. A facet's
// corners are its cell's in the legacy order; it faces out of its inside cell (counterclockwise
// seen from outside), by an exact orientation test where the legacy took a determinant's sign in
// doubles. The surface's points are the triangulation's points it uses, in point order, and
// point_index gives each one's input index (input_of_point, or the point's own index when that is
// empty). A unit is a facet or a point.
step_task extract_surface(const Triangulation &triang,
                          const std::vector<std::size_t> &input_of_point, geometry &surface,
                          std::vector<std::size_t> &point_index, surface_counts &counts);

} // namespace cvc::cocone

#endif // CVC_GEOMETRY_COCONE_TIGHT_COCONE_STAGES_H

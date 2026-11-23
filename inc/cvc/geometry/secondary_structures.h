/*
  Copyright 2008, 2026 The University of Texas at Austin

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

// cvc::find_secondary_structures: a protein's helices and sheets from the isosurface of its
// density map, through the surface's medial axis. Ported from VolumeRover's SecondaryStructures
// (Goswami and Bajaj, UT 2008; transfix/volrover src/SecondaryStructures, LGPL-2.1, the copy TexMol
// also carried), per docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md T5. Its stages:
//   1. the 3D Delaunay triangulation of the surface's vertices (cvc::tet_complex);
//   2. Tight Cocone, which splits the cells into inside and outside;
//   3. the inner medial axis: every Voronoi facet inside, peeled from its free boundary down to the
//      facets the angle and ratio conditions choose (so the axis keeps the shape's topology);
//   4. U1 and U2, the unstable manifolds of the axis's index-1 and index-2 saddles, made into one
//      skeleton graph: U1's Voronoi facets (polygons, each with the length of its dual Delaunay
//      edge as width) and U2's chains (edges, each with the radius of the tube around it);
//   5. sheets: the polygons' edge-connected components, largest first; a sheet is one of the
//      largest `sheet_count` components of at least `sheet_min_area`, drawn where its polygons'
//      widths lie in [sheet_min_width, sheet_max_width];
//   6. helices: the U2 edges outside the sheets whose widths lie in [helix_min_width,
//      helix_max_width] (a helix's density is a tube about 2.5 Å across); their points clustered
//      within `helix_join_distance`, each cluster split into runs of roughly one direction, and a
//      line fitted to each run (its principal axis, spanning the run's points); the longest
//      `helix_count` runs of at least `helix_min_length` are the helices, drawn as cylinders.
// Stages 1 and 2 and U1 and U2 are the cocone core cvc::skeletonize uses (P1); the medial axis,
// the graph and the selection are SecondaryStructures' own.
//
// The legacy library split the work as its GUI did: the skeleton once, then the helices and sheets
// again whenever a slider moved. So does this: the result keeps the skeleton graph, and
// select_secondary_structures() re-runs stages 5 and 6 on it with other parameters.
//
// What the port fixes on the way in (each a patch of the legacy build in tools/volrover-goldens,
// so its A/B compares like with like):
//   - a chain step between two cells with no common facet got the smaller cell's squared radius as
//     its width, where every other width is a length; it gets the radius;
//   - a cluster of one candidate point (no other within the join distance) made an empty run, which
//     the legacy fit divided by zero and read past an empty array for; empty runs are skipped;
//   - three CGAL_assertion()s with side effects in the graph's construction (get_eid(), which
//     finds the edge the code then updates), which release builds compiled out, leaving the
//     index -1 and writing before the edge array; the lookups happen here (the core already
//     makes the five calls its own copies hid in assertions);
//   - helices of equal length: the legacy std::map kept one; all are kept, in a stable order;
//   - an axis parallel to z gave a cylinder of NaNs (the rotation axis cross(z, a) normalised);
//   - it wrote debug files to the current directory (the medial axis, the watertight surface, the
//     curves, the cylinders as VRML), printed timings, coloured with rand(), and its command-line
//     shim called exit(1); none of that is here. Its Robust Cocone option filtered a first
//     triangulation's points into a file the second triangulation never read, so it is left out.
// Candidate points are clustered in the order of their coordinates, not the cells' memory order,
// so the result does not depend on how the triangulation stored its cells. Two things follow the
// core rather than the legacy copies: the polygon test for U1's saddles also tries the degenerate
// triangles of a polygon whose Voronoi vertices coincide (SecondaryStructures' copy skipped them;
// Skeletonization's, which the core ports, did not), and Tight Cocone and U2 keep the legacy
// results up to the cells' order and the rounding of their Voronoi vertices. On a synthetic surface
// the port is the legacy library exactly; on 1GRL's map it agrees on 99% of the polygons and every
// helix width (tools/volrover-goldens, secondary_structures_test).
//
// Resumable (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9): secondary_stepper runs every stage as a
// phase, a unit at a time; find_secondary_structures() is the stepper at budget 0, and
// find_secondary_structures_job() its result_job. The outputs' geometries belong to the app the
// call names. Nothing here is process-wide. Needs libcvc built with Shewchuk's predicates
// (CVC_WITH_SHEWCHUK), as cvc::tet_complex does; without them every entry throws.

#ifndef CVC_GEOMETRY_SECONDARY_STRUCTURES_H
#define CVC_GEOMETRY_SECONDARY_STRUCTURES_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

class app;

class secondary_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The defaults are the legacy library's (skel.cpp, helix.cpp) and its GUIs'. Angles are in
// radians, lengths in the surface's units (Å for a molecule's map). A param_table
// (cvc/geometry/secondary_structures_params.h) gives the state keys, CLI flags and keywords.
struct secondary_params {
  // Tight Cocone (as cvc::skeleton_params).
  double cocone_phi = 3.14159265358979323846 / 8.0;      // half the cocone's opening angle
  double sharp_phi = 2.0 * 3.14159265358979323846 / 3.0; // the angle of sharp edges
  double flat_phi = 3.14159265358979323846 / 3.0;        // the flatness test's angle
  double flatness_ratio = 1.2 * 1.2;                     // the squared thinness factor
  // The medial axis.
  double medial_theta = 22.5 * 3.14159265358979323846 / 180.0;
  double medial_ratio = 8.0 * 8.0;
  // Helices.
  double helix_min_width = 2.0;     // the U2 edge widths kept (a tube's radius)
  double helix_max_width = 3.0;     //
  double helix_join_distance = 4.0; // points nearer than this are in one cluster
  double helix_min_length = 3.0;    // shorter runs are not helices
  int helix_count = 20;             // the longest this many are the helices
  double helix_radius = 1.8;        // the cylinders' radius, for drawing
  // Sheets.
  double sheet_min_width = 2.0; // the polygon widths drawn (a slab's thickness)
  double sheet_max_width = 3.0; //
  double sheet_min_area = 10.0; // smaller components are never sheets
  int sheet_count = 2;          // the largest this many components are the sheets
};

// The skeleton graph (the legacy Skel): U1's polygons and U2's chains, with their widths.
struct secondary_skeleton {
  struct edge {
    std::uint32_t a = 0, b = 0; // its points
    double width = 0;           // U2: the tube's radius around it; 0 for U1's edges
    int faces = 0;              // the polygons it bounds (more than 2: a non-manifold edge)
    bool on_u1 = false, on_u2 = false;
  };
  struct face {
    std::vector<std::uint32_t> points; // a Voronoi facet's vertices, in order around it
    double width = 0;                  // the length of its dual Delaunay edge (a slab's thickness)
    int component = -1;                // in component_area
  };
  std::vector<std::array<double, 3>> points; // Voronoi vertices inside, then U2's saddle points
  std::vector<edge> edges;
  std::vector<face> faces;
  std::vector<double> component_area; // per edge-connected component of the faces
};

struct secondary_helix {
  // The axis: the line fitted to the helix's points, from where their projections end (`start`
  // the larger projection, along the principal direction with its largest component positive).
  std::array<double, 3> start{}, end{};
  double length = 0;                         // |end - start|
  double radius = 0;                         // the cylinder's, helix_radius
  std::vector<std::array<double, 3>> points; // the skeleton points it was fitted to
};

struct secondary_sheet {
  int component = -1;       // in the skeleton's components
  double area = 0;          // the whole component's area
  std::size_t polygons = 0; // its polygons drawn (width in the window)
};

struct secondary_structures {
  std::vector<secondary_helix> helices; // longest first, at most helix_count
  std::vector<secondary_sheet> sheets;  // largest first, at most sheet_count
  // Triangles: each helix a closed cylinder (the legacy 100-sided one), magenta.
  geometry helix_geometry;
  std::vector<int> helix_of_tri;
  // Triangles: each sheet's polygons in the width window, fanned from their centroids, a colour
  // per sheet.
  geometry sheet_geometry;
  std::vector<int> sheet_of_tri;
  // Lines: every edge of the skeleton graph.
  geometry skeleton_geometry;
  // What the legacy GUI's histograms showed: the widths of the U2 edges that can be helices (no
  // polygon, ends not in one component) and of every polygon.
  std::vector<double> helix_widths, sheet_widths;
  int helix_candidates = 0; // runs of at least helix_min_length, before the count
  int sheet_candidates = 0; // components of at least sheet_min_area, before the count
  secondary_skeleton skeleton;
  std::vector<std::string> warnings;
};

// The secondary structures of a surface's vertices (only those its triangles and quads use, or all
// of its points when it has neither). Throws secondary_error for fewer than four distinct points,
// points all in one plane or on one line, a coordinate that is not finite, a face index past the
// points, invalid parameters, or a build without Shewchuk's predicates.
secondary_structures find_secondary_structures(app &ctx, const geometry &surface,
                                               const secondary_params &params = {});

// Stages 5 and 6 again, on a skeleton a run kept (the legacy GUI's sliders). Throws
// secondary_error for invalid parameters or a skeleton whose indices are out of range.
secondary_structures select_secondary_structures(app &ctx, secondary_skeleton skeleton,
                                                 const secondary_params &params = {});

// find_secondary_structures, a slice at a time (§1.9). Phases: the triangulation's ("checking",
// "ordering", "inserting", "compacting", "circumcentres"), then "building", "voronoi", "tight
// cocone", "medial axis", "u1", "u2", "skeleton", "sheets", "helices" and "output". A unit is one
// iteration of the loop running: a point, cell, facet, edge or polygon visited, a step of a walk or
// peel, a candidate point clustered; 16 of the triangulation's own units. A failure is a
// secondary_error thrown from step() (and rethrown by later steps and take()). cancel() is the one
// thread-safe member: the step running (or the next) ends the stepper with no result. Not
// thread-safe otherwise, but it may move between threads between steps. Constructing one only
// stores its input (sharing the geometry's arrays).
class secondary_stepper {
public:
  secondary_stepper(app &ctx, const geometry &surface, secondary_params params = {});
  ~secondary_stepper();
  secondary_stepper(secondary_stepper &&) noexcept;
  secondary_stepper &operator=(secondary_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  secondary_structures take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<secondary_structures>>
find_secondary_structures_job(app &ctx, const geometry &surface, secondary_params params = {});

} // namespace cvc

#endif // CVC_GEOMETRY_SECONDARY_STRUCTURES_H

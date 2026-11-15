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

// cvc::skeletonize: the skeleton of a point cloud (or of a mesh's vertices), ported from
// VolumeRover's Skeletonization (Goswami and Bajaj, UT 2008; transfix/volrover src/Skeletonization)
// per docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2. Its stages:
//   1. optionally, Robust Cocone: keep the points that lie on a smooth surface;
//   2. the 3D Delaunay triangulation (cvc::tet_complex, exact, CGAL's triangulation);
//   3. Tight Cocone: a watertight surface through the points, which splits the cells into inside
//      and outside;
//   4. the medial axis: the Voronoi facets chosen by the angle and ratio conditions;
//   5. U1, the unstable manifolds of the index-1 saddles, clustered into planar patches: the
//      sheets;
//   6. U2, the unstable manifolds of the index-2 saddles, a graph made into polylines: the
//      strands;
//   7. clean-up: the largest patches are the sheets, each smaller one becomes a star of segments
//      from its centroid.
// Only the inside of the surface is skeletonized (the legacy code's protein setting).
//
// The result keeps both parts, which VolumeRover2 did not (it dropped the sheets and the strands'
// connectivity), and has none of the legacy code's process-wide state: its bounding box was a
// global that the first call filled and every later call reused, its colours came from srand(0)
// and rand(), and its degenerate cases wrote debug files to the current directory. Here the box
// is the call's own, colours are a function of the index, and degenerate cases are warnings.
//
// Resumable (§1.9): skeleton_stepper runs every stage as a phase, a unit at a time; skeletonize()
// is the stepper at budget 0, and skeletonize_job() its result_job. Needs libcvc built with
// Shewchuk's predicates (CVC_WITH_SHEWCHUK, as cvc::tet_complex does); without them every entry
// throws skeleton_error.

#ifndef CVC_GEOMETRY_SKELETON_H
#define CVC_GEOMETRY_SKELETON_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

class skeleton_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The defaults are the legacy Parameters' (Skeletonization.h) and VolumeRover2's dialog's.
// Angles are in radians. A param_table (cvc/geometry/skeleton_params.h) gives the state keys,
// CLI flags, DSL dict and pycvc keywords.
struct skeleton_params {
  // Robust Cocone, the pre-filter for noisy samples.
  bool robust = false;
  double big_ball_ratio = 1.0 / 16.0; // a ball is big above this times its corners' (nnd)^2
  double theta_if = 5.0 * 3.14159265358979323846 / 180.0;  // deep with the infinite ball
  double theta_ff = 10.0 * 3.14159265358979323846 / 180.0; // deep between finite balls
  // Tight Cocone.
  double cocone_phi = 3.14159265358979323846 / 8.0;      // half the cocone's opening angle
  double sharp_phi = 2.0 * 3.14159265358979323846 / 3.0; // the angle of sharp edges
  double flat_phi = 3.14159265358979323846 / 3.0;        // the flatness test's angle
  double flatness_ratio = 1.2 * 1.2;                     // the squared thinness factor
  // The medial axis.
  double medial_theta = 22.5 * 3.14159265358979323846 / 180.0;
  double medial_ratio = 8.0 * 8.0;
  // Sheets: the largest `sheet_count` planar clusters are sheets, or, with discard_small, every
  // cluster of at least `threshold` times the largest's area; the others become stars.
  bool discard_small = false;
  double threshold = 0.1;
  int sheet_count = 2;
  // Strands: also output, per polyline, the Voronoi vertices whose cell radius lies in (2, 3),
  // the legacy protein-helix filter (the `helix` geometry).
  bool helix_window = false;
};

struct skeleton {
  // Lines: every polyline of U2 inside the surface, as consecutive segments (green), then the
  // stars of the planar clusters that are not sheets, a segment from the cluster's weighted
  // centroid to each of its active boundary points (blue). Shared points are merged.
  geometry strands;
  // Triangles: the sheets' Voronoi facets, each convex, fanned from its first vertex (zero-area
  // triangles left out); a colour per sheet. Shared points are merged within a sheet.
  geometry sheets;
  // Lines, only with helix_window: per polyline, its Voronoi vertices outside the sheets whose
  // cell radius lies in (2, 3), red at maxima of the distance function, green elsewhere.
  geometry helix;
  // Per strands line: the polyline it is on (0 the shortest), or -1 - c for a star of cluster c.
  std::vector<int> strand_of_line;
  // Per sheets triangle: its sheet, 0 the largest.
  std::vector<int> sheet_of_tri;
  int polylines = 0;      // the polylines in `strands`
  int clusters = 0;       // the planar clusters found
  int sheet_clusters = 0; // of them, the sheets
  std::vector<std::string> warnings;
};

// The skeleton of `points`. Throws skeleton_error for fewer than four distinct points, points
// all in one plane, a coordinate that is not finite, invalid parameters, or a build without
// Shewchuk's predicates.
skeleton skeletonize(const std::vector<std::array<double, 3>> &points,
                     const skeleton_params &params = {});
// The skeleton of a geometry's vertices (only the points are read).
skeleton skeletonize(const geometry &points_or_mesh, const skeleton_params &params = {});

// skeletonize, a slice at a time (§1.9). Phases: the triangulation's ("checking", "ordering",
// "inserting", "compacting", "circumcentres"; twice with `robust`, around "robust cocone"),
// then "building", "voronoi", "tight cocone", "medial axis", "sheets", "strands", "clean-up"
// and "output". A unit is one iteration of the loop running: a point, cell, facet or edge
// visited, or one step of a walk (Tight Cocone's umbrella and boundary walks, the flow along
// U1 and U2), each bounded by the local degree; 16 of the triangulation's own units. A failure is
// a skeleton_error thrown from step() (and rethrown by later steps and take()). cancel() is the
// one thread-safe member: the step running (or the next) ends the stepper with no result; a
// result_job around it ends "cancelled". Not thread-safe otherwise, but it may move between
// threads between steps. Constructing one only stores its input.
class skeleton_stepper {
public:
  skeleton_stepper(std::vector<std::array<double, 3>> points, skeleton_params params = {});
  skeleton_stepper(const geometry &points_or_mesh, skeleton_params params = {});
  ~skeleton_stepper();
  skeleton_stepper(skeleton_stepper &&) noexcept;
  skeleton_stepper &operator=(skeleton_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  skeleton take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<skeleton_stepper> skeletonize_stepper(std::vector<std::array<double, 3>> points,
                                                      skeleton_params params = {});
std::unique_ptr<result_job<skeleton>> skeletonize_job(std::vector<std::array<double, 3>> points,
                                                      skeleton_params params = {});
std::unique_ptr<result_job<skeleton>> skeletonize_job(const geometry &points_or_mesh,
                                                      skeleton_params params = {});

} // namespace cvc

#endif // CVC_GEOMETRY_SKELETON_H

/*
  Copyright 2007-2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (Tight Cocone)
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

// cvc::tight_cocone: a watertight surface through a point cloud, ported from VolumeRover's
// TightCocone (Dey and Goswami's Tight Cocone; Goswami, UT 2007-2008; transfix/volrover
// src/TightCocone) per docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4, V1. Its stages:
//   1. optionally, Robust Cocone: keep the points that lie on a smooth surface (a noisy sample);
//   2. the 3D Delaunay triangulation (cvc::tet_complex, exact, CGAL's triangulation);
//   3. Tight Cocone: the Delaunay facets of a watertight surface through the points, which split
//      the cells into inside and outside;
//   4. the surface: those facets as triangles, each facing out of the inside cell
//      (op.cpp's wt_to_geometry).
// The skeleton (cvc/geometry/skeleton.h), PocketTunnel and SecondaryStructures run the same
// stages 2 and 3 first; they share libcvc's one cocone core.
//
// What changed from the legacy surfaceReconstruction():
//   - the surface's points are the input points it uses, in input order, and point_index says
//     which; the legacy wrote every point it triangulated, used or not;
//   - its robust mode never reset its bounding box between its two triangulations (a global the
//     first call filled). Nothing on the surface path reads that box (only the medial-axis
//     writer, which the surface never calls), so it changed no surface; the port has no box;
//   - no CGAL (GPL): the triangulation is cvc::tet_complex, with Shewchuk's exact predicates;
//   - no statics, no stderr, and no debug files: degenerate cases are counted as warnings.
//
// Resumable (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9): tight_cocone_stepper runs every stage as a
// phase, a unit at a time; tight_cocone() is the stepper at budget 0, and tight_cocone_job() its
// result_job. Needs libcvc built with Shewchuk's predicates (CVC_WITH_SHEWCHUK); without them every
// entry throws tight_cocone_error.

#ifndef CVC_GEOMETRY_TIGHT_COCONE_H
#define CVC_GEOMETRY_TIGHT_COCONE_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

class tight_cocone_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The defaults are the legacy TightCocone::Parameters' (tight_cocone.h) and VolumeRover2's
// dialog's; the skeleton's Tight Cocone fields have the same names and defaults. Angles are in
// radians. A param_table (cvc/geometry/tight_cocone_params.h) gives the state keys, the `cvc
// cocone` flags, the DSL dict cocone.tight takes and pycvc's keywords.
struct tight_cocone_params {
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
};

struct tight_cocone_result {
  // Triangles: the surface, each counterclockwise seen from outside (its normal points out of the
  // inside cell). Its points are the input points the surface uses, in input order; no colours.
  geometry surface;
  // Per surface point: its index in the input points (the first, where the input repeats a
  // point).
  std::vector<std::size_t> point_index;
  std::size_t points = 0;         // the input points
  std::size_t duplicates = 0;     // of them, repeats of an earlier point, merged
  std::size_t robust_dropped = 0; // of the distinct ones, those Robust Cocone left out
  std::size_t cells = 0;          // the finite Delaunay cells of the points the surface came from
  std::size_t inside_cells = 0;   // of them, those Tight Cocone left inside the surface
  std::vector<std::string> warnings;
};

// The surface through `points`. Throws tight_cocone_error for fewer than four distinct points,
// points all in one plane, a coordinate that is not finite, invalid parameters, Robust Cocone
// keeping fewer than four points, or a build without Shewchuk's predicates.
tight_cocone_result tight_cocone(const std::vector<std::array<double, 3>> &points,
                                 const tight_cocone_params &params = {});
// The surface through a geometry's vertices (only the points are read).
tight_cocone_result tight_cocone(const geometry &points_or_mesh,
                                 const tight_cocone_params &params = {});

// tight_cocone, a slice at a time (§1.9). Phases: "checking", the triangulation's ("ordering",
// "inserting", "compacting", "circumcentres"; twice with `robust`, around "voronoi" and "robust
// cocone"), then "building", "voronoi", "tight cocone" and "surface". A unit is one iteration of
// the loop running: a point, cell or facet visited, or one step of Tight Cocone's umbrella and
// boundary walks, each bounded by the local degree; 16 of the triangulation's own units. A failure
// is a tight_cocone_error thrown from step() (and rethrown by later steps and take()). cancel() is
// the one thread-safe member: the step running (or the next) ends the stepper with no result; a
// result_job around it ends "cancelled". Not thread-safe otherwise, but it may move between
// threads between steps. Constructing one only stores its input.
class tight_cocone_stepper {
public:
  tight_cocone_stepper(std::vector<std::array<double, 3>> points, tight_cocone_params params = {});
  tight_cocone_stepper(const geometry &points_or_mesh, tight_cocone_params params = {});
  ~tight_cocone_stepper();
  tight_cocone_stepper(tight_cocone_stepper &&) noexcept;
  tight_cocone_stepper &operator=(tight_cocone_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  tight_cocone_result take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<tight_cocone_result>>
tight_cocone_job(std::vector<std::array<double, 3>> points, tight_cocone_params params = {});
std::unique_ptr<result_job<tight_cocone_result>> tight_cocone_job(const geometry &points_or_mesh,
                                                                  tight_cocone_params params = {});

} // namespace cvc

#endif // CVC_GEOMETRY_TIGHT_COCONE_H

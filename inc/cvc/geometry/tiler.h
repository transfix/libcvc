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

// cvc::tile_contours: surfaces through planar contours on a stack of slices, ported from
// VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover src/ContourTiler) per
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3. ContourTiler reimplements Bajaj, Coyle and
// Lin's tiler ("Arbitrary topology shape reconstruction from planar cross sections", GMIP 58(6),
// 1996) for many components, with the removal of intersections between them. For every pair of
// consecutive slices and every component, it
//   1. finds which contours correspond (the paper's theorem 6) and each vertex's tiling region;
//   2. tiles what the optimality rules allow (phase 1), then
//   3. fills the untiled regions left over (phase 2): each region split into convex pieces, a
//      vertex at each piece's centroid and on each diagonal between pieces, lifted off the slice
//      planes by natural-neighbour interpolation;
//   4. optionally removes the intersections between components' tiles (remove_intersections).
// Before tiling, each slice's contours are curated: collinear vertices dropped and contours of
// one slice pushed at least curation_delta apart (dilate, subtract, erode), and contours of
// consecutive slices augmented with each other's vertices where their projections cross.
//
// The legacy library ran on CGAL; this port runs on its own kernel and cvc::planar (no CGAL, no
// GPL code, §1.2): the snapping grid and Shewchuk's exact predicates, Clipper2 Booleans and
// straight-skeleton offsets, CDT triangulations and convex partitions, and Sibson interpolation.
// Every contour vertex is first snapped onto a grid of the stack (a power-of-two unit about 2^-31
// of its extent), so a vertex moves by at most half that. The untiled regions' fill uses a
// different convex partition than CGAL's (fewer pieces), so their interior vertices differ from
// the legacy library's; the tiles between contour vertices elsewhere are the legacy's (§3, §9
// item 2).
//
// Nothing here holds state outside the call: the legacy log4cplus loggers (process-wide) are a
// log in the call's own context, and what they warned of is the result's `warnings`.
//
// Resumable (§1.9): tile_stepper does the work a unit at a time, tile_contours() is the stepper at
// budget 0, and tile_contours_job() its result_job. Built only with cvc::planar
// (CVC_WITH_PLANAR); without it every entry throws tiler_error.

#ifndef CVC_GEOMETRY_TILER_H
#define CVC_GEOMETRY_TILER_H

#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

class tiler_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The defaults are the legacy Tiler_options' and command line's. A param_table
// (cvc/geometry/tiler_params.h) gives the state keys, CLI flags, DSL dict and pycvc keywords.
struct tile_params {
  // A contour vertex whose triangle with its neighbours has at most this area (world units^2) is
  // dropped as collinear (legacy -e; VolumeRover2's dialog passed 1e-15).
  double collinear_epsilon = 0;
  // The least distance kept between contours of one slice: each is dilated by half of it, the
  // overlaps subtracted, and eroded back (legacy -C; the dialog passed 0.01). 0 skips curation.
  // Must be less than the spacing of every pair of slices tiled.
  double curation_delta = 1e-8;
  // Remove the intersections between different components' tiles (legacy -r); needs
  // curation_delta > 0.
  bool remove_intersections = false;
  // Contours of consecutive slices correspond only when their overlap exceeds this area (world
  // units^2) where their boundaries cross (legacy -v); negative: any overlap.
  double correspondence_overlap = -1;
  // Components tiled, by name, as regular expressions (Perl syntax, matched against the whole
  // name): a component is tiled when it matches an include (or there is none) and no exclude
  // (legacy -c and -x).
  std::vector<std::string> include;
  std::vector<std::string> exclude;
  // The slice indices tiled, inclusive (legacy -s); -1: the stack's first, or last.
  int z_first = -1;
  int z_last = -1;
  // Also tile an empty slice beyond each end of the range, so every component is closed there.
  // The legacy tiler closed a component only where its -s range reached past its contours.
  bool cap_ends = true;
  // One colour (r, g, b in [0, 1]) for every component's surface (legacy --color, 0-255); empty:
  // each component's colour from the stack.
  std::vector<double> color;
};

struct tile_result {
  // One surface per component tiled: triangles, with each vertex's normal and colour. Vertices are
  // shared between a component's triangles.
  std::map<std::string, geometry> components;
  // What the tiler skipped or could not do, each a sentence naming where ("tiler.tile: Failed to
  // remove intersections between a001 - a002 slices 3 - 4: ...").
  std::vector<std::string> warnings;
};

// The surfaces of the stack's components. Throws tiler_error for invalid parameters, a stack
// contour_problems() finds wrong, a curation_delta not below the slice spacing, or a build
// without cvc::planar. A contour that is not simple is skipped with a warning.
tile_result tile_contours(const contour_stack &stack, const tile_params &params = {});

// tile_contours, a slice at a time (§1.9). Phases: "snapping" (a unit per contour vertex),
// "curating" (a unit per slice), "augmenting" (per pair of slices, until a pass adds no
// vertex), "tiling" (per component of each pair of slices, and per pair of components whose
// intersections are removed), "assembling" (per triangle). label() names the slices being tiled.
// A failure is a tiler_error thrown from step() (and rethrown by later steps and take()).
// cancel() is the one thread-safe member: the step running (or the next) ends the stepper with no
// result; a result_job around it ends "cancelled". Not thread-safe otherwise, but it may move
// between threads between steps. Constructing one only stores its input.
class tile_stepper {
public:
  tile_stepper(contour_stack stack, tile_params params = {});
  ~tile_stepper();
  tile_stepper(tile_stepper &&) noexcept;
  tile_stepper &operator=(tile_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  std::string label() const; // "slices 3 - 4" while tiling, "" otherwise
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  tile_result take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<tile_stepper> tile_contours_stepper(contour_stack stack, tile_params params = {});
std::unique_ptr<result_job<tile_result>> tile_contours_job(contour_stack stack,
                                                           tile_params params = {});

// What is wrong with the parameters, each a sentence; empty when nothing is.
std::vector<std::string> tile_params_problems(const tile_params &params);

} // namespace cvc

#endif // CVC_GEOMETRY_TILER_H

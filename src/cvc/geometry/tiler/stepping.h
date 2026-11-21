/*
  Copyright 2026 The University of Texas at Austin

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

// How the vendored tiler is made resumable (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9, §3): as the
// skeleton port is (skeleton/stepping.h, whose step_task, SK_UNIT and SK_RUN it uses). Each long
// stage of a slice pair is a coroutine named <stage>_steps that yields once per contour pair,
// vertex, OTV pair, chord step or untiled region, its result in an out-parameter; the legacy
// function of the same name runs it to the end, so the legacy code and tests call it as before.
//
// A unit should cost about a microsecond (§1.9): the scheduler sizes a cooperative call by the
// clock, but starts each phase at 256 units and never asks for fewer than 16. So a call that
// cannot yield inside (a Clipper2 offset or Boolean, a sweep over contour edges, a simplicity test,
// a Delaunay triangulation) is charged by its size with TILER_UNITS: it yields once per unit it is
// worth, a step's budget counts it as that many, and the steps after it pay the rest off without
// working. A curated slice, for example, counts its contours' vertices, not 1. Yielding adds no
// work to the call itself, so the result is the same whatever the steps.

#ifndef CVC_GEOMETRY_TILER_STEPPING_H
#define CVC_GEOMETRY_TILER_STEPPING_H

#include "../skeleton/stepping.h"

#include <algorithm>
#include <cstddef>

namespace cvc::tiler {

using cocone::step_task;

// Runs a stage to its end.
inline void run_to_end(step_task t) {
  while (!t.resume()) {
  }
}

// The charges, from the size n (vertices, points or edges) a call takes. Measured in a native
// Release build (MSVC, Windows, 2026-11) on tiler_test's cases, P4's demo stack and the bunny,
// each is within a few times of a microsecond a unit, which is what the pacing needs. The n^2
// terms are the calls that grow faster than their input.
//
// Polygon_2::is_simple, which compares the boxes of every pair of edges.
inline std::size_t simple_test_units(std::size_t n) { return n / 8 + n * n / 1024; }
// A sweep line over n edges (augment1, get_intersecting_pairs).
inline std::size_t sweep_units(std::size_t n) { return 2 * n; }
// A wavefront offset (Clipper2 passes from event to event): a contour dilated or eroded.
inline std::size_t offset_units(std::size_t n) { return 3 * n + n * n / 256; }
// A Clipper2 Boolean: two contours differenced.
inline std::size_t boolean_units(std::size_t n) { return n + n * n / 2048; }
// can_tile: theorem 6's overlap test on two contours.
inline std::size_t overlap_test_units(std::size_t n) { return 3 * n; }
// A slice's Hierarchy: its contours checked and nested.
inline std::size_t hierarchy_units(std::size_t n) { return 4 * n; }
// Per-vertex tables made or freed (Vertices, a Tiler_workspace).
inline std::size_t table_units(std::size_t n) { return 2 * n + 64; }
// A Vertex_map, which holds 10000 entries whatever the contours.
inline std::size_t vertex_map_units() { return 256; }
// A pass over n vertices: a contour's collinear vertices, or a point located among contours.
inline std::size_t scan_units(std::size_t n) { return n / 16 + 1; }
// A segment tested against n contour edges (an OTV candidate).
inline std::size_t edge_test_units(std::size_t n) { return n / 4 + 4; }
// n OTV candidates sorted by distance.
inline std::size_t sort_units(std::size_t n) { return n / 16 + 1; }
// A tiling test (cases 1-6) in a workspace of n vertices.
inline std::size_t tile_test_units(std::size_t n) { return n / 2 + 1; }
// An untiled region's simplification and convex partition.
inline std::size_t untiled_units(std::size_t n) { return 16 * n + 64; }
// A Delaunay triangulation of n points and one natural-neighbour interpolation on it.
inline std::size_t delaunay_units(std::size_t n) { return 4 * n; }
// n tiles or handles copied or freed.
inline std::size_t copy_units(std::size_t n) { return n / 16 + 1; }

} // namespace cvc::tiler

// `n` units of work done in one piece: yields n times (at least once).
#define TILER_UNITS(n)                                                                             \
  for (std::size_t tiler_unit_ = 0, tiler_units_ = std::max<std::size_t>(1, (n));                  \
       tiler_unit_ < tiler_units_; ++tiler_unit_)                                                  \
  co_yield 0

#endif // CVC_GEOMETRY_TILER_STEPPING_H

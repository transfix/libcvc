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

//
// Ported: the legacy tile() mutated its input slices, wrote .rawc files and debug output
// (tiler_output.h), and logged through process-wide loggers. Here it runs on slices tile_contours()
// built (a copy), returns its triangles in a tile_output, logs to the call's tiler_context, and is
// a coroutine (the skeleton port's step_task, VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9): it yields
// a unit per slice curated, per slice pair augmented, per component tiled in each pair, per
// intersecting component pair whose intersections it removes, and per triangle decomposed. The
// order of the work is the legacy order.

#ifndef __TILER_H__
#define __TILER_H__

#include "Contour.h"
#include "Slice.h"
#include "Tile.h"
#include "Tiler_options.h"
#include "Tiler_workspace.h"
#include "stepping.h"

#include <boost/shared_ptr.hpp>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace cvc::tiler {

// What tile() makes: each component's triangles, decomposed where intersection removal put points
// on their edges.
struct tile_output {
  std::map<std::string, std::list<Triangle>> triangles;
};

// Tiles every component across consecutive slices (all of them, empty ones included: a component
// absent from one slice of a pair is capped there). The slices are curated and augmented in place.
step_task tile(std::vector<Slice> &slices, const Tiler_options &options, tiler_context &ctx,
               tile_output &out);

} // namespace cvc::tiler

#endif

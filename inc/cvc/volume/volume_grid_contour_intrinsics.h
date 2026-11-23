/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A volume grid's contours (cvc::grid_contours, cvc/volume/volume_grid_contours.h) in the DSL
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, P10):
//
//   (grid-contours.tile curves [params])          a dict: surfaces (object name to geometry) and
//   (grid-contours.tile-async curves [params])    warnings, as tiler.tile's; params are
//   (grid-contours.tile-start curves [params] target)   tile_params' (the "grid-contours.tile"
//                                                 table), lengths in object units. The three job
//                                                 forms of cvc/core/job_intrinsics.h: use -start in
//                                                 an Ariadne action
//   (grid-contours.info curves)                   a dict: frame (dims, origin, spacing), curves,
//                                                 points, and objects, a list of dicts of name,
//                                                 color, interpolation, samples, visible, curves,
//                                                 closed and points
//   (grid-contours.stack curves [axis])           the closed curves on the planes looking down
//                                                 axis ("z", the default, "y" or "x") as a
//                                                 contour_stack (what contours.* and tiler.tile
//                                                 take), splines drawn
//   (grid-contours.from-stack stack frame [axis]) a stack's polygons as closed curves on a frame:
//                                                 `frame` is curves (their frame) or a volume
//   (grid-contours.add-curve curves object axis index points [closed])
//                                                 new curves: `curves` (nil: none, on a unit frame)
//                                                 with a curve added to the object named (added,
//                                                 in the palette's next colour, when there is
//                                                 none), on the plane looking down axis at slice
//                                                 index; points are r u r u ... or (r u) pairs;
//                                                 closed
//                                                 (#t by default) needs three points or more
//   (params.defaults name), (params.check name dict)
//                                                 with the tables "grid-contours.tile",
//                                                 "grid-contours.object" and
//                                                 "grid-contours.display"
//
// A curves argument is a data object (from state-data-get: a volume_grid node's grid.curves, or
// what these return) or a state path naming a node whose data channel holds one, resolved under the
// calling program's root. Results are fresh data objects (payload std::shared_ptr<const
// grid_contours>, type name "cvc::grid_contours"); a program stores them with state-data-set, and a
// volume_grid node whose grid.curves is written takes them. No intrinsic reads or writes files.
//
// Hosts: register_grid_contour_intrinsics binds them in one environment; install_grid_contour_
// intrinsics adds them to the app's own action-intrinsic providers (key "cvc.grid-contours"), so
// that app's Ariadne documents get them and no other app's do.

#ifndef CVC_VOLUME_VOLUME_GRID_CONTOUR_INTRINSICS_H
#define CVC_VOLUME_VOLUME_GRID_CONTOUR_INTRINSICS_H

#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
struct grid_contours;
struct job_op;
namespace state_exec {
struct environment;
struct intrinsics_context;
struct value_tag;
} // namespace state_exec

// The data object curves travel in.
state_exec::value_tag grid_contours_value(grid_contours c);

// The grid-contours.tile op (cvc/core/job_intrinsics.h), its tasks run as `threading` says.
job_op grid_contours_tile_op(sliced_threading threading = sliced_threading::automatic);

// Register "grid-contours.tile", "grid-contours.object" and "grid-contours.display" in the app's
// parameter tables.
void register_grid_contour_params(cvc::app &app);

// Bind grid-contours.*, job.cancel and params.* in `env`, grid-contours.tile's tasks run as
// `threading` says. `ictx` resolves state paths; it and the app must outlive the environment's
// evaluations.
void register_grid_contour_intrinsics(std::shared_ptr<state_exec::environment> env,
                                      state_exec::intrinsics_context &ictx, cvc::app &app,
                                      sliced_threading threading = sliced_threading::automatic);

// Add them to the app's action-intrinsic providers under the key "cvc.grid-contours" (a second
// call replaces the first), and register the tables.
void install_grid_contour_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_GRID_CONTOUR_INTRINSICS_H

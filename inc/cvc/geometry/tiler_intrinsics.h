/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tiler.tile, cvc::tile_contours in the DSL (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6,
// §3), in the three forms cvc/core/job_intrinsics.h gives every port's op:
//
//   (tiler.tile stack [params])            the surfaces: the calling process waits on the job
//   (tiler.tile-async stack [params])      a future of them
//   (tiler.tile-start stack [params] target)
//                                          the task's pid at once; the task publishes the node
//                                          keys (§1.9: busy, status, progress, phase, pid)
//                                          under `target` and writes the result to
//                                          `target.result`. The form for Ariadne actions; a
//                                          node's target is `<node>.tiler` (§3).
//
// `stack` is a cvc::contour_stack: a data object holding one (contours.* make them), or a state
// path naming the node whose data channel does. `params` is a dict of tile_params' fields
// (cvc/geometry/tiler_params.h: "curation_delta", "remove_intersections", "include", ...), or nil
// for the defaults; (params.defaults "tiler.tile") lists them. The result is a dict:
//   "surfaces"   a dict from each component's name to a data object holding its cvc::geometry
//   "warnings"   a list of strings
// A failure (a bad stack or parameter, a curation_delta not below the slice spacing) raises at the
// call. No intrinsic reads or writes files.

#ifndef CVC_GEOMETRY_TILER_INTRINSICS_H
#define CVC_GEOMETRY_TILER_INTRINSICS_H

#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// The op, for a host that registers ops itself.
job_op tiler_tile_op(sliced_threading threading = sliced_threading::automatic);

// tiler.tile's three forms, and job.cancel and params.*, in `env`.
void register_tiler_intrinsics(state_exec::environment_ptr env,
                               state_exec::intrinsics_context &ictx, cvc::app &app);

// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "tiler" (registering again replaces it).
void add_tiler_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_GEOMETRY_TILER_INTRINSICS_H

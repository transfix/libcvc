/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// skeleton.compute, cvc::skeletonize in the DSL (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.6, §2), in the three forms cvc/core/job_intrinsics.h gives every port's op:
//
//   (skeleton.compute g [params])          the skeleton: the calling process waits on the job
//   (skeleton.compute-async g [params])    a future of it
//   (skeleton.compute-start g [params] target)
//                                          the task's pid at once; the task publishes the node
//                                          keys under `target` and writes the result to
//                                          `target.result`. The form for Ariadne actions.
//
// `g` is a cvc::geometry: a data object holding one, or a state path naming the node whose data
// channel does (only its points are read). `params` is a dict of skeleton_params' fields
// (cvc/geometry/skeleton_params.h: "robust", "cocone_phi", "sheet_count", ...), or nil for the
// defaults; (params.defaults "skeleton.compute") lists them. The result is a dict:
//   "strands", "sheets", "helix"   data objects holding the skeleton's cvc::geometry parts
//   "polylines", "clusters", "sheet_clusters"   counts
//   "warnings"                     a list of strings
// A failure (too few points, all in one plane, a bad parameter) raises at the call.

#ifndef CVC_GEOMETRY_SKELETON_INTRINSICS_H
#define CVC_GEOMETRY_SKELETON_INTRINSICS_H

#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// The op, for a host that registers ops itself.
job_op skeleton_compute_op(sliced_threading threading = sliced_threading::automatic);

// skeleton.compute's three forms, and job.cancel and params.*, in `env`.
void register_skeleton_intrinsics(state_exec::environment_ptr env,
                                  state_exec::intrinsics_context &ictx, cvc::app &app);

// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "skeleton" (registering again replaces it).
void add_skeleton_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_GEOMETRY_SKELETON_INTRINSICS_H

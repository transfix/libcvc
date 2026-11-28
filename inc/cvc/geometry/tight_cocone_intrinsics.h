/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cocone.tight, cvc::tight_cocone in the DSL (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4,
// V1; VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6), in the three forms cvc/core/job_intrinsics.h gives
// every port's op:
//
//   (cocone.tight g [params])          the surface: the calling process waits on the job
//   (cocone.tight-async g [params])    a future of it
//   (cocone.tight-start g [params] target)
//                                      the task's pid at once; the task publishes the node keys
//                                      under `target` and writes the result to `target.result`.
//                                      The form for Ariadne actions.
//
// `g` is a cvc::geometry: a data object holding one, or a state path naming the node whose data
// channel does, such as a dataset's "data.points.geometry" (only its points are read). `params` is
// a dict of tight_cocone_params' fields (cvc/geometry/tight_cocone_params.h: "robust",
// "cocone_phi", "flat_phi", ...), or nil for the defaults; (params.defaults "cocone.tight") lists
// them. The result is a dict:
//   "surface"                       a data object holding the surface's cvc::geometry; (data-put
//                                   NAME "geometry" surface) makes it a dataset the scene draws
//   "triangles", "surface_points"   its size
//   "points", "duplicates", "robust_dropped", "cells", "inside_cells"   tight_cocone_result's counts
//   "warnings"                      a list of strings
// A failure (too few points, all in one plane, a bad parameter) raises at the call.

#ifndef CVC_GEOMETRY_TIGHT_COCONE_INTRINSICS_H
#define CVC_GEOMETRY_TIGHT_COCONE_INTRINSICS_H

#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// The op, for a host that registers ops itself.
job_op tight_cocone_op(sliced_threading threading = sliced_threading::automatic);

// cocone.tight's three forms, and job.cancel and params.*, in `env`.
void register_tight_cocone_intrinsics(state_exec::environment_ptr env,
                                      state_exec::intrinsics_context &ictx, cvc::app &app);

// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "cvc.tight-cocone" (registering again replaces it). register_cvcgl_extensions
// calls it.
void install_tight_cocone_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_GEOMETRY_TIGHT_COCONE_INTRINSICS_H

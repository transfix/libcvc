/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// volume.filter, cvc::volume_filter_job in the DSL (cvc/volume/volume_filter.h;
// docs/roadmap/DATA_FROM_STATE_TREE.md, decision 2), in the three forms cvc/core/job_intrinsics.h
// gives every port's op:
//
//   (volume.filter vol [params])          the filtered volume: the calling process waits on the job
//   (volume.filter-async vol [params])    a future of it
//   (volume.filter-start vol [params] target)
//                                         the task's pid at once; the task publishes the node keys
//                                         under `target` and writes the result to `target.result`.
//                                         The form for Ariadne actions
//
// `vol` is a cvc::volume: a data object holding one, or a state path naming the node whose data
// channel does (a document's data.<name>.volume, a node's published volume). `params` is a dict of
// volume_filter_params' fields ("filter", "radiometric_sigma", "spatial_sigma", "radius"), or nil
// for the defaults; (params.defaults "volume.filter") lists them. The result is a data object
// holding a new std::shared_ptr<const cvc::volume>; the input is never written. A program stores
// the result where the views follow it, and every node viewing that key redraws on it:
//
//   (volume.filter-start "data.head.volume" (dict "radius" 1) "filtering")
//   ... once (state-get "filtering.status") is "done":
//   (state-data-set "data.head.volume" (state-data-get "filtering.result"))
//
// Like every intrinsic, none of these reads or writes a file.

#ifndef CVC_VOLUME_VOLUME_FILTER_INTRINSICS_H
#define CVC_VOLUME_VOLUME_FILTER_INTRINSICS_H

#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// The op, for a host that registers ops itself.
job_op volume_filter_op(sliced_threading threading = sliced_threading::automatic);

// volume.filter's three forms, and job.cancel and params.*, in `env`.
void register_volume_filter_intrinsics(state_exec::environment_ptr env,
                                       state_exec::intrinsics_context &ictx, cvc::app &app);

// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "cvc.volume-filter" (registering again replaces it), so no other app sees them.
// cvcGL's register_cvcgl_extensions installs it.
void install_volume_filter_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_FILTER_INTRINSICS_H

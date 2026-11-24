/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// segment.seeded and segment.critical-points, cvc::segment_seeded and cvc::critical_point_seeds in
// the DSL (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §5), in the three forms
// cvc/core/job_intrinsics.h gives every port's op:
//
//   (segment.seeded vol classes [params])         the segmentation: the calling process waits on
//                                                 the job
//   (segment.seeded-async vol classes [params])   a future of it
//   (segment.seeded-start vol classes [params] target)
//                                                 the task's pid at once; the task publishes the
//                                                 node keys under `target` and writes the result to
//                                                 `target.result`. The form for Ariadne actions
//
// `vol` is a cvc::volume and `classes` a cvc::point_classes (cvc/volume/point_classes.h): each a
// data object holding one, or a state path naming the node whose data channel does. `params` is a
// dict of seeded_segmentation_params' fields (cvc/volume/segmentation_params.h: "t_low", "t_high",
// "normalized_thresholds", "gamma", "automatic", "variable", "timestep", "class_volumes" and the
// critical points' "cp_*"), or nil for the defaults; (params.defaults "segment.seeded") lists them.
// With "automatic" "critical_points" the classes may be empty (point-classes.remove nil ...): the
// paper's feature and background seeds then seed the run alone. The result is a dict:
//   "labels"         a data object holding the labels, a UChar cvc::volume (255: no class)
//   "classes"        a list of data objects, a cvc::volume per class (empty without
//                    class_volumes)
//   "border"         a data object holding cvc::point_classes: one class, "border", of the voxels
//                    where a front stopped (point-classes.points reads them)
//   "class_names"    a list of strings, and "class_voxels" a list of counts
//   "automatic_seeds", "classified_seeds"   counts
//   "t_low_normalized", "t_high_normalized" the thresholds on the 0..255 scale
//   "warnings"       a list of strings
// A failure (no seeds, a seed outside the volume, a constant volume, a bad parameter) raises at
// the call. Like every intrinsic, none of these reads or writes a file.
//
// segment.critical-points, cvc::critical_point_seeds (the paper's seeds, as seed_mode::
// critical_points makes them), in the same three forms:
//
//   (segment.critical-points vol [params])
//   (segment.critical-points-async vol [params])
//   (segment.critical-points-start vol [params] target)
//
// `params` is the same dict (its "cp_*" fields, "normalized_thresholds", "variable" and
// "timestep" are the ones it reads). The result is a data object holding cvc::point_classes: two
// classes, "feature" then "background" (point-classes.points reads them), ready for segment.seeded
// or a volume_grid's points.
//
// segment.surfaces, cvc::class_surfaces (cvc/volume/class_surfaces.h: each class's surface, for a
// 3D view), in the same three forms:
//
//   (segment.surfaces labels [params])
//   (segment.surfaces-async labels [params])
//   (segment.surfaces-start labels [params] target)
//
// `labels` is a volume of integer labels (segment.seeded's "labels"); `params` a dict of
// class_surface_params' fields ("smoothing", "lambda", "mu", "close", "background", "labels"). The
// result is a dict: "labels" (the labels that have a surface, ascending), "surfaces" (a data object
// holding each one's cvc::geometry, in that order), "voxels" (each one's voxel count) and
// "warnings".

#ifndef CVC_VOLUME_SEGMENTATION_INTRINSICS_H
#define CVC_VOLUME_SEGMENTATION_INTRINSICS_H

#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// The ops, for a host that registers ops itself.
job_op segment_seeded_op(sliced_threading threading = sliced_threading::automatic);
job_op critical_point_seeds_op(sliced_threading threading = sliced_threading::automatic);
job_op class_surfaces_op(sliced_threading threading = sliced_threading::automatic);

// segment.seeded's, segment.critical-points' and segment.surfaces' three forms, and job.cancel
// and params.*, in `env`.
void register_segmentation_intrinsics(state_exec::environment_ptr env,
                                      state_exec::intrinsics_context &ictx, cvc::app &app);

// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "cvc.segment" (registering again replaces it), so no other app sees them.
void install_segmentation_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_VOLUME_SEGMENTATION_INTRINSICS_H

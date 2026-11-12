/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Volume routines as cvc::result_job<R>s (cvc/core/result_job.h): their resumable steppers, run on
// the app's scheduler a slice at a time (schedule_sliced_job), stepped by hand, or from pycvc, each
// owning its result. The blocking functions are these steppers run with budget 0.

#ifndef CVC_VOLUME_VOLUME_JOBS_H
#define CVC_VOLUME_VOLUME_JOBS_H

#include <cvc/core/result_job.h>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/resize_stepper.h>
#include <cvc/volume/volume.h>
#include <memory>

namespace cvc {

// compute_contour_tree(vol, opt) as a job (contour_tree_builder): phases "preparing", "sorting",
// "join tree", "split tree", "merging". The job keeps a shallow copy of the voxels (their app must
// outlive it); opt.progress, if set, is still called after every step.
inline std::unique_ptr<result_job<contour_tree>>
contour_tree_job(const voxels &vol, const contour_tree_options &opt = contour_tree_options()) {
  return make_result_job(contour_tree_builder(vol, opt));
}

// volume::resize(target) as a job (resize_stepper; one phase, "running"). Holds a copy-on-write
// copy of the source.
inline std::unique_ptr<result_job<volume>> resize_job(const volume &src, const dimension &target) {
  return make_result_job(resize_stepper(src, target));
}

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_JOBS_H

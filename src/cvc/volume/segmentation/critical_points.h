/*
  Copyright 2003, 2026 The University of Texas at Austin

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

// The paper's seeds (seed_mode::critical_points): SegMed's GVF_Compute (transfix/volrover
// src/Segmentation/SegMed/GVF.cpp, at the sha in tools/volrover-goldens/volrover.sha), whole, as a
// coroutine on a genseg_context. The legacy function returns right after its initial field (line
// 97), so its diffusion, critical points and clustering never ran; this runs them. With cp_field
// gradient_magnitude, cp_weights initial and cp_background_threshold the legacy tlow, its feature
// class is the legacy critical list, point for point (src/cvc/tests/critical_points_test.cpp,
// against tools/volrover-goldens' segmed goldens). What changed:
//   - the globals are members; nothing is written to a file or printed;
//   - the paper's field (the gradient of the volume, not of its gradient magnitude) and the
//     paper's weights (the field being diffused) are the defaults;
//   - SegMed kept only its field's minima above tlow; the paper's maxima seed the feature, its
//     minima the background, and its §4 thresholds move maxima and minima between the two;
//   - GVF_Cluster's recursion (one level per voxel of a run, which overflows the stack on a large
//     plateau) is an explicit stack visiting the voxels in the same order.
//
// Private to the segmentation port: cvc/volume/segmentation.h is the API.

#ifndef CVC_VOLUME_SEGMENTATION_CRITICAL_POINTS_H
#define CVC_VOLUME_SEGMENTATION_CRITICAL_POINTS_H

#include "genseg.h"

#include <cstddef>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <vector>

namespace cvc::genseg {

class critical_seeds {
public:
  // ---- inputs ----
  critical_field field = critical_field::image;
  gradient_vector_diffusion_params diffusion; // its normalize is always on (SegMed)
  // A maximum below feature_threshold joins the background, a minimum above
  // background_threshold the feature: on the 0..255 scale of the normalised dataset, or in the
  // voxels' units with data_unit_thresholds (run() rescales them as normalise() did the voxels).
  double feature_threshold = -std::numeric_limits<double>::infinity();
  double background_threshold = std::numeric_limits<double>::infinity();
  bool data_unit_thresholds = false;
  int sample = 0;
  // Throw segmentation_error (naming `who`) when there are no critical points, for a run that has
  // no other seeds.
  bool need_seeds = false;
  const char *who = "critical_point_seeds";

  // ---- outputs ----
  std::vector<std::size_t> feature, background; // linear voxel indices, x-fastest order
  std::size_t maxima = 0, minima = 0;           // before the thresholds and the thinning

  // Runs on ctx's dims and normalised dataset (read, never written), a unit at a time on ctx.
  task run(genseg_context &ctx);
  // About how many units run() takes for n voxels (for progress).
  std::size_t expected_units(std::size_t n) const;

private:
  task initial_field(genseg_context &ctx, const float *f);
  task diffuse(genseg_context &ctx);
  task classify(genseg_context &ctx);
  task thin(genseg_context &ctx, std::vector<unsigned char> &mask, std::vector<std::size_t> &out);

  vector_field _field;
  buffer<float> _edge; // |grad I| on 0..255, with critical_field::gradient_magnitude
  float _feature = 0.0f, _background = 0.0f; // the thresholds on the 0..255 scale
};

// `seeded_segmentation_params`' cp_* fields into `s`.
void configure(critical_seeds &s, const seeded_segmentation_params &p);

// The voxels of `in` into ctx's dims and dataset, as floats (read_data's dataset, before its
// normalisation), a unit per voxel. Throws segmentation_error naming `who` when the volume's voxel
// type is undefined, it is too large, a voxel is not finite, or it is constant. Shared by
// segment_stepper and critical_point_seeds_stepper.
task read_voxels(genseg_context &ctx, const volume &in, const char *who);

} // namespace cvc::genseg

#endif // CVC_VOLUME_SEGMENTATION_CRITICAL_POINTS_H

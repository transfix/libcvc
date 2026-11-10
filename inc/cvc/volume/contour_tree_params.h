/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// compute_contour_tree's options as a parameter table (cvc/core/param_table.h): their state keys
// and properties, CLI flags, DSL dict and pycvc keywords. contour_tree_options::progress is a
// callback, not a parameter, so it has no field.

#ifndef CVC_VOLUME_CONTOUR_TREE_PARAMS_H
#define CVC_VOLUME_CONTOUR_TREE_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/volume/contour_tree.h>
#include <tuple>

namespace cvc {

template <> struct param_table<contour_tree_options> : param_fields<contour_tree_options> {
  using P = contour_tree_options;
  static constexpr auto fields = std::tuple{
      field("max_dim", &P::max_dim, unit("voxel"),
            doc("resample so no axis exceeds this many voxels first (0: full resolution)")),
      field("quantize_u8", &P::quantize_u8,
            doc("build on the values quantized to 256 levels, as VolumeRover2 did")),
  };
};

} // namespace cvc

#endif // CVC_VOLUME_CONTOUR_TREE_PARAMS_H

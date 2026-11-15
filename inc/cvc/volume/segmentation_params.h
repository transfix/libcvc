/*
  Copyright 2026 The University of Texas at Austin

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

// The parameter table of cvc::seeded_segmentation_params (cvc/core/param_table.h): its state keys,
// CLI flags (--t-low, --t-high, ...), DSL dict and pycvc keywords, from one declaration
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5).

#ifndef CVC_VOLUME_SEGMENTATION_PARAMS_H
#define CVC_VOLUME_SEGMENTATION_PARAMS_H

#include <cmath>
#include <cvc/core/param_table.h>
#include <cvc/volume/segmentation.h>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <>
struct param_table<seeded_segmentation_params> : param_fields<seeded_segmentation_params> {
  using P = seeded_segmentation_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("t_low", &P::t_low, unit("data"),
            doc("voxels below this stop every front (data units, or 0..255 with "
                "normalized_thresholds)")),
      field("t_high", &P::t_high, unit("data"),
            doc("automatic seeds are the smoothed volume's local maxima above this (data units, "
                "or 0..255 with normalized_thresholds)")),
      field("normalized_thresholds", &P::normalized_thresholds,
            doc("t_low and t_high are on the legacy 0..255 scale the volume is rescaled to")),
      field("gamma", &P::gamma, range(-1.0, 0.0),
            doc("front speed exp(gamma * |grad I|), |grad I| rescaled to 0..255: fronts slow at "
                "edges")),
      field("automatic", &P::automatic,
            doc("automatic seeds added to the point classes' (none, local_maxima)")),
      field("variable", &P::variable, range(0, none),
            doc("the point classes of this variable seed the run")),
      field("timestep", &P::timestep, range(0, none),
            doc("the point classes of this timestep seed the run")),
      field("class_volumes", &P::class_volumes,
            doc("also output a volume per class: its voxels' values, zero elsewhere")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (std::isinf(p.t_low))
      problems.push_back("t_low: must be finite");
    if (std::isinf(p.t_high))
      problems.push_back("t_high: must be finite");
  }
};

} // namespace cvc

#endif // CVC_VOLUME_SEGMENTATION_PARAMS_H

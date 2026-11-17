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

// The parameter tables of cvc/volume/seed_generators.h (cvc/core/param_table.h): the state keys,
// CLI flags and pycvc keywords of capsid_seed_params and rotation_seed_params.

#ifndef CVC_VOLUME_SEED_GENERATORS_PARAMS_H
#define CVC_VOLUME_SEED_GENERATORS_PARAMS_H

#include <cmath>
#include <cvc/core/param_table.h>
#include <cvc/volume/seed_generators.h>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <> struct param_table<capsid_seed_params> : param_fields<capsid_seed_params> {
  using P = capsid_seed_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("t_low", &P::t_low, unit("data"),
            doc("only voxels above this count (0..255 with normalized_thresholds, the default; "
                "data units without)")),
      field("normalized_thresholds", &P::normalized_thresholds,
            doc("t_low is on the 0..255 scale the volume is rescaled to")),
      field("h", &P::h, range(0, 1000),
            doc("the capsid's triangulation number h (T = h^2 + hk + k^2)")),
      field("k", &P::k, range(0, 1000), doc("the capsid's triangulation number k")),
      field("variable", &P::variable, range(0, none), doc("the variable the classes mark")),
      field("timestep", &P::timestep, range(0, none), doc("the timestep the classes mark")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (std::isinf(p.t_low))
      problems.push_back("t_low: must be finite");
    if (p.h == 0 && p.k == 0)
      problems.push_back("h, k: not both 0 (T = h^2 + hk + k^2 must be at least 1)");
  }
};

template <> struct param_table<rotation_seed_params> : param_fields<rotation_seed_params> {
  using P = rotation_seed_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("folds", &P::folds, range(1, 10000),
            doc("copies, at the angles 2 pi n / folds about the axis")),
      field("axis", &P::axis, doc("the axis's direction (x, y, z), world units")),
      field("center", &P::center,
            doc("a point on the axis (x, y, z), world units; [] is the volume's centre")),
      field("variable", &P::variable, range(0, none), doc("the classes of this variable rotate")),
      field("timestep", &P::timestep, range(0, none), doc("the classes of this timestep rotate")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (p.axis.size() != 3)
      problems.push_back("axis: must be three numbers");
    else if (!std::isfinite(p.axis[0]) || !std::isfinite(p.axis[1]) || !std::isfinite(p.axis[2]))
      problems.push_back("axis: must be finite");
    else if (p.axis[0] == 0 && p.axis[1] == 0 && p.axis[2] == 0)
      problems.push_back("axis: must not be zero");
    if (!p.center.empty() && p.center.size() != 3)
      problems.push_back("center: must be three numbers, or none");
    for (double c : p.center)
      if (!std::isfinite(c)) {
        problems.push_back("center: must be finite");
        break;
      }
  }
};

} // namespace cvc

#endif // CVC_VOLUME_SEED_GENERATORS_PARAMS_H

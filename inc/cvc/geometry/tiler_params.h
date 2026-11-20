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

// tile_params' parameter table (cvc/core/param_table.h; VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5):
// its state keys and properties, the `cvc tile` flags, the DSL dict tiler.tile takes and pycvc's
// keywords.

#ifndef CVC_GEOMETRY_TILER_PARAMS_H
#define CVC_GEOMETRY_TILER_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/geometry/tiler.h>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <> struct param_table<tile_params> : param_fields<tile_params> {
  using P = tile_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("collinear_epsilon", &P::collinear_epsilon, unit("world^2"), range(0.0, none),
            doc("a contour vertex whose triangle with its neighbours has at most this area is "
                "dropped as collinear")),
      field("curation_delta", &P::curation_delta, unit("world"), range(0.0, none),
            doc("the least distance kept between contours of one slice (0: no curation); less "
                "than the slice spacing")),
      field("remove_intersections", &P::remove_intersections,
            doc("remove the intersections between different components' tiles (needs "
                "curation_delta > 0)")),
      field("correspondence_overlap", &P::correspondence_overlap, unit("world^2"),
            doc("crossing contours of consecutive slices correspond only above this overlap area "
                "(negative: any)")),
      field("include", &P::include,
            doc("components tiled, as regular expressions on the name (none: every one)")),
      field("exclude", &P::exclude, doc("components not tiled, as regular expressions")),
      field("z_first", &P::z_first, range(-1, none),
            doc("the first slice index tiled (-1: the stack's first)")),
      field("z_last", &P::z_last, range(-1, none),
            doc("the last slice index tiled (-1: the stack's last)")),
      field("cap_ends", &P::cap_ends,
            doc("close every component beyond each end of the slice range")),
      field("color", &P::color, range(0.0, 1.0),
            doc("one colour (r, g, b in [0, 1]) for every surface (empty: each component's)")),
  };
  // A first after last, patterns that aren't regular expressions, a colour of other than three
  // values, intersection removal without curation.
  static void check(const P &p, std::vector<std::string> &problems) {
    for (const std::string &s : tile_params_problems(p))
      problems.push_back(s);
  }
};

} // namespace cvc

#endif // CVC_GEOMETRY_TILER_PARAMS_H

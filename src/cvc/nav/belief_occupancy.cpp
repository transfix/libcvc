/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// belief_occupancy.cpp — see belief_occupancy.h. Built without -ffast-math so the
// float32 sigmoid tracks numpy; the threshold compare is the one place a 1-ULP
// probability difference could flip a cell, so the arithmetic is deliberate.

#include <algorithm>
#include <cmath>
#include <cvc/nav/belief_occupancy.h>

namespace cvc {
namespace nav {

void to_occupancy(const float *logodds, int rows, int cols, unknown_policy policy, double p_thresh,
                  double band, std::uint8_t *occ_out) {
  const long n = static_cast<long>(rows) * cols;
  if (policy == unknown_policy::optimistic) {
    // occ = p > max(p_thresh, 0.5 + band). numpy compares the float32 p against
    // the float64 threshold, upcasting p (exact) to double.
    const double thr = std::max(p_thresh, 0.5 + band);
    for (long i = 0; i < n; ++i) {
      const float p = 1.0f / (1.0f + std::exp(-logodds[i])); // float32 sigmoid
      occ_out[i] = (static_cast<double>(p) > thr) ? 1 : 0;
    }
  } else {
    // occ = !(p < min(1-p_thresh, 0.5 - band))  ==  p >= that threshold.
    const double thr = std::min(1.0 - p_thresh, 0.5 - band);
    for (long i = 0; i < n; ++i) {
      const float p = 1.0f / (1.0f + std::exp(-logodds[i]));
      occ_out[i] = (static_cast<double>(p) < thr) ? 0 : 1;
    }
  }
}

void composite_occupancy(const float *logodds, int rows, int cols, unknown_policy policy,
                         double p_thresh, double band, const double *dyn_stamp, double t_now,
                         double ttl_s, std::uint8_t *occ_out) {
  to_occupancy(logodds, rows, cols, policy, p_thresh, band, occ_out);
  if (!dyn_stamp)
    return;
  const long n = static_cast<long>(rows) * cols;
  for (long i = 0; i < n; ++i)
    if (!occ_out[i] && (t_now - dyn_stamp[i]) <= ttl_s) // DynamicLayer.occupancy
      occ_out[i] = 1;
}

void world_to_cell(double x, double y, double min_x, double min_y, double max_x, double max_y,
                   int rows, int cols, int &row, int &col) {
  const double cx = (x - min_x) / (max_x - min_x) * (cols - 1);
  const double cy = (y - min_y) / (max_y - min_y) * (rows - 1);
  row = static_cast<int>(std::rint(cy)); // int(round(cy)) — half-to-even
  col = static_cast<int>(std::rint(cx));
}

} // namespace nav
} // namespace cvc

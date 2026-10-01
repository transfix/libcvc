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
#include <cstring>
#include <cvc/nav/belief_occupancy.h>

namespace cvc {
namespace nav {

namespace {

// Thresholding is a pure function of each cell's float bits, and a belief plane
// is mostly long runs of one value (the +-l_clamp saturated prior, cells sensed
// back to the clamp), so a cell whose bits repeat its predecessor's reuses that
// decision instead of re-evaluating expf. Every distinct value still goes
// through the exact float32 sigmoid + compare below, so the raster is identical;
// it is just ~4x cheaper, which matters because step() composites every plane on
// every sense tick to find the ones that changed.
template <class Decide>
void threshold_runs(const float *logodds, long n, std::uint8_t *occ_out, Decide decide) {
  static_assert(sizeof(float) == sizeof(std::uint32_t), "float32 bit pattern");
  if (n <= 0)
    return;
  std::uint32_t last_bits;
  std::memcpy(&last_bits, logodds, sizeof last_bits);
  std::uint8_t last = decide(logodds[0]);
  occ_out[0] = last;
  for (long i = 1; i < n; ++i) {
    std::uint32_t bits;
    std::memcpy(&bits, logodds + i, sizeof bits);
    if (bits != last_bits) {
      last_bits = bits;
      last = decide(logodds[i]);
    }
    occ_out[i] = last;
  }
}

} // namespace

void to_occupancy(const float *logodds, int rows, int cols, unknown_policy policy, double p_thresh,
                  double band, std::uint8_t *occ_out) {
  const long n = static_cast<long>(rows) * cols;
  if (policy == unknown_policy::optimistic) {
    // occ = p > max(p_thresh, 0.5 + band). numpy compares the float32 p against
    // the float64 threshold, upcasting p (exact) to double.
    const double thr = std::max(p_thresh, 0.5 + band);
    threshold_runs(logodds, n, occ_out, [thr](float lo) -> std::uint8_t {
      const float p = 1.0f / (1.0f + std::exp(-lo)); // float32 sigmoid
      return (static_cast<double>(p) > thr) ? 1 : 0;
    });
  } else {
    // occ = !(p < min(1-p_thresh, 0.5 - band))  ==  p >= that threshold.
    const double thr = std::min(1.0 - p_thresh, 0.5 - band);
    threshold_runs(logodds, n, occ_out, [thr](float lo) -> std::uint8_t {
      const float p = 1.0f / (1.0f + std::exp(-lo));
      return (static_cast<double>(p) < thr) ? 0 : 1;
    });
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

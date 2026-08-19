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

// belief_occupancy.h — log-odds belief -> planning occupancy (port P5).
//
// The planning raster that feeds the (bit-identical) EDT / build_sdf, ported
// from grl_snam/belief.py: to_occupancy thresholds the belief probability under
// an unknown-space policy, composite_occupancy OR-s in the decaying dynamic
// layer, and world_to_cell maps a world point to a grid cell. This is the ONE
// new BIT surface between the raw belief and the field, and it is the delicate
// one: the probability MUST be computed as a float32 sigmoid (1/(1+expf(-lo)))
// and compared against the threshold exactly as numpy does, or a cell near the
// threshold flips and the whole downstream field changes. See
// docs/CVCNAV_CPP_PORT_ROADMAP.md §1 (the to_occupancy risk) and its parity test.

#ifndef __CVC_NAV_BELIEF_OCCUPANCY_H__
#define __CVC_NAV_BELIEF_OCCUPANCY_H__

#include <cstdint>

namespace cvc {
namespace nav {

// Unknown-space policy (belief.py to_occupancy): optimistic = unknown is free
// (drive in, replan on discovery); pessimistic = unknown is wall.
enum class unknown_policy { optimistic, pessimistic };

// Threshold the log-odds belief into a 0/1 occupancy raster (row-major
// rows*cols), bit-identical to BeliefGrid.to_occupancy. `logodds` is float32;
// the probability p = 1/(1+expf(-logodds)) is computed in float32 and the
// comparison is done exactly as numpy's (the float32 p widened to double vs the
// double threshold). optimistic: p > max(p_thresh, 0.5+band); pessimistic:
// !(p < min(1-p_thresh, 0.5-band)).
void to_occupancy(const float *logodds, int rows, int cols, unknown_policy policy, double p_thresh,
                  double band, std::uint8_t *occ_out);

// composite_occupancy (belief.py): to_occupancy OR the dynamic layer's current
// footprint. `dyn_stamp` (row-major rows*cols float64, -inf where never marked)
// is optional (nullptr => no dynamic layer); a cell is dynamically occupied iff
// (t_now - dyn_stamp) <= ttl_s.
void composite_occupancy(const float *logodds, int rows, int cols, unknown_policy policy,
                         double p_thresh, double band, const double *dyn_stamp, double t_now,
                         double ttl_s, std::uint8_t *occ_out);

// world_to_cell (BeliefGrid.world_to_cell): (x,y) world -> (row, col) via
// round-half-to-even, exactly as int(round(...)). bounds = (min_x,min_y,max_x,
// max_y). Not clamped or bounds-checked (matches the reference).
void world_to_cell(double x, double y, double min_x, double min_y, double max_x, double max_y,
                   int rows, int cols, int &row, int &col);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_BELIEF_OCCUPANCY_H__

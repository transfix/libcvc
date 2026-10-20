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

// belief_material.h — belief (observed) material: the material planes a belief
// plane has seen, for range-limited sensing. The spec is
// docs/roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md.
//
// The truth rasters (risk_raw, hard) are kept where the plane's sensor has seen
// and replaced by pinned values elsewhere; material_build and witness_gate then
// run unchanged on the result. This header adds only:
//   belief_material_inputs  the per-cell select (BIT);
//   belief_gate_surface     the gate's feasibility surface and its metres
//                           clearance (BIT; the sim_world::set_material chain);
//   seen_disc               the optional range-disc seen rule (BIT vs numpy).
// The default seen mask is the sense ray-cast's field of view, which
// sense_batch (grid_nav.h) already keeps; its material-range mask is an optional
// output there. belief_material.cpp is compiled with -ffp-contract=off (the disc
// test and the corner-sample coordinates would otherwise FMA-contract).

#ifndef CVC_NAV_BELIEF_MATERIAL_H
#define CVC_NAV_BELIEF_MATERIAL_H

#include <cstdint>

namespace cvc {
namespace nav {

// Which cells count as seen. raycast: the sense ray-cast's field of view
// (occluded; the default). disc: every corner sample within the material range
// of the agent's position, with no occlusion (it sees through walls; kept to
// reproduce a controller trained on a disc mask).
enum class seen_rule { raycast, disc };

struct belief_material_params {
  float unknown_risk = 0.0f;     // risk_raw on an unseen cell, in [0, 1]
  std::uint8_t unknown_hard = 0; // hard on an unseen cell, 0 or 1
  bool unknown_gate_hard = true; // an unseen cell blocks the witness gate's rays
  seen_rule rule = seen_rule::raycast;
  double material_range_m = 0.0; // 0 => the sense range; else 0 < range <= sense range
};

// Throws std::invalid_argument unless unknown_risk is finite and in [0, 1],
// unknown_hard is 0 or 1 and material_range_m is finite and >= 0. (The upper
// bound, the sense range, is checked where it is known: sim_world.)
void validate(const belief_material_params &p);

// risk_out[k] = seen[k] ? risk_raw[k] : unknown_risk;
// hard_out[k] = seen[k] ? (hard[k] != 0) : unknown_hard.
// All rasters row-major rows*cols. Validates p.
void belief_material_inputs(const float *risk_raw, const std::uint8_t *hard,
                            const std::uint8_t *seen, int rows, int cols,
                            const belief_material_params &p, float *risk_out,
                            std::uint8_t *hard_out);

// The witness gate's surface on belief planes:
//   gate_hard_out[k] = hard_b[k] | occ[k] | (unknown_gate_hard & !seen[k])   (0/1)
//   clear_m_out[k]   = (float)(sqrt(edt2_squared(gate_hard_out)) * cell_w)
// the f64 chain with one f32 store that sim_world::set_material and GRL-SNAM's
// MaterialRuntime use. `occ` may be null (no occupancy). Validates p.
void belief_gate_surface(const std::uint8_t *hard_b, const std::uint8_t *occ,
                         const std::uint8_t *seen, int rows, int cols, double cell_w,
                         const belief_material_params &p, std::uint8_t *gate_hard_out,
                         float *clear_m_out);

// seen_rule::disc for one agent: ORs into `seen` every corner sample (r, c) with
//   (x_c - px)*(x_c - px) + (y_r - py)*(y_r - py) <= R*R      (float64)
// where x_c = c * ((max_x - min_x) / (cols - 1)) + min_x and the last column is
// exactly max_x (np.linspace), y likewise. (px, py) is the agent's world position,
// not snapped to a cell. Returns true iff some byte of `seen` changed. R < 0 or
// a non-finite input marks nothing.
bool seen_disc(double px, double py, double R, int rows, int cols, double min_x, double min_y,
               double max_x, double max_y, std::uint8_t *seen);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_BELIEF_MATERIAL_H

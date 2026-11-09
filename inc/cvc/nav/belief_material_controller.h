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

// GRL-SNAM's range-limited belief-material controller (grl_snam/belief_material_controller.py),
// in C++: docs/roadmap/NAV-BELIEF-MATERIAL-CONTROLLER-ROADMAP.md. This header is its C1 part, the
// contract and the model inputs:
//
//   belief_material_contract    the .cvcnm meta record (key grl_snam.belief_material): what the
//                               net was trained with. from_meta / to_meta / validate (§1.1).
//   belief_header_mismatches    the .cvcnm v2 belief block against the controller's belief and
//                               patch settings (§1.2).
//   belief_runtime_mismatches   a material_config's constants against the contract (§1.3).
//   material_config_for         the material_config the contract implies on a grid (§1.3).
//   belief_material_controller  a loaded net that passed §1.2 and §1.1, with check_runtime.
//   build_belief_material_controller_inputs   one agent's robot-relative tokens and its
//                               3-channel belief patch (§2.1).
//   visible_obstacles           the obstacle set both Austin tools feed the net (§2.2).
//
// The Python controller is the reference. Its golden (GRL-SNAM's
// grl_snam.tools.belief_material_controller_golden, committed in src/cvc/tests/data) pins every
// value here bit for bit, except the goal token's dist (1 ulp: numpy's norm may go through BLAS).
// belief_material_controller.cpp is compiled with -ffp-contract=off, as belief_material.cpp is.
// decide, the batched decision, the rollout and the sim_world wiring are C2 and C3.

#ifndef CVC_NAV_BELIEF_MATERIAL_CONTROLLER_H
#define CVC_NAV_BELIEF_MATERIAL_CONTROLLER_H

#include <cstdint>
#include <cvc/nav/belief_material.h>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material.h>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

inline constexpr const char *kBeliefMaterialMetaKey = "grl_snam.belief_material";
inline constexpr const char *kBeliefMaterialRobotRelative = "robot_relative";

// What a belief-material .cvcnm was trained with, recorded as JSON in its meta trailer under
// kBeliefMaterialMetaKey (Python's BeliefMaterialContract). The defaults are Python's.
struct belief_material_contract {
  std::string feature_frame = kBeliefMaterialRobotRelative; // must be "robot_relative"
  double feature_scale_m = 20.0;       // tokens are (world - position) / this; finite, > 0
  double k_sharp = 5.0;                // hazard barrier steepness, 1/m; finite, > 0
  double d_hat_sdf_m = 3.0;            // hazard barrier reach, m; finite, > 0
  double material_gate_trigger = 0.30; // the witness gate's soft-risk trigger; in [0, 1]
  double geometry_d_hat_m = 3.0;       // obstacle barrier reach in the rollouts, m; finite, > 0

  // Parse a .cvcnm meta string, as BeliefMaterialContract.from_meta does. Throws
  // std::invalid_argument, its message prefixed with `source`, when the meta is empty or not JSON,
  // has no object under the key, names a field not above, holds a value that is not a JSON string
  // (feature_frame) or number (the rest), or breaks validate(). A missing field takes its default,
  // a JSON integer is read as a double, and other top-level keys are ignored. Like Python's
  // json.loads, the parse accepts NaN / Infinity / -Infinity and keeps the last of duplicate keys.
  static belief_material_contract from_meta(const std::string &meta,
                                            const std::string &source = "checkpoint");

  // The meta string recording this contract, byte-identical to Python's to_meta(): json.dumps with
  // sort_keys, ", " / ": " separators and Python's float repr (20.0, 1e-05, 1e+16). Validates
  // first.
  std::string to_meta() const;

  // The value rules (§1.1). Throws std::invalid_argument.
  void validate() const;
};

bool operator==(const belief_material_contract &a, const belief_material_contract &b);
inline bool operator!=(const belief_material_contract &a, const belief_material_contract &b) {
  return !(a == b);
}

// One field a check refused: its name (Python's), the value found and the value wanted, as text.
struct belief_contract_mismatch {
  std::string field, actual, expected;
};

// §1.2: the net's .cvcnm v2 belief block and patch size against the controller's settings, in
// Python's order (risk_channels, seen_rule, unknown_risk, unknown_hard, unknown_gate_hard,
// material_range_m, patch_extent_m, patch_size). Floats compare within 1e-6 (decision E7).
std::vector<belief_contract_mismatch> belief_header_mismatches(const coef_energy_net &net,
                                                               const belief_material_params &bp,
                                                               int patch_size,
                                                               double patch_extent_m);

// §1.3: a material_config's k_sharp, d_hat_m and gate.material_trigger against the contract
// (named k_sharp, d_hat_sdf_m and material_trigger, as Python names them), within 1e-6.
std::vector<belief_contract_mismatch> belief_runtime_mismatches(const belief_material_contract &c,
                                                                const material_config &mc);

// §1.3: the material_config the contract implies on a grid of cell width cell_w (metres), the twin
// of contract.material_params(...) under MaterialRuntime: the contract's k_sharp, d_hat and
// trigger, sigma 1, the gate horizon Python's GateParams sets in metres (25 m) rounded half-even
// to cells (at least 1), and hard_margin_m 0, which sim_world resolves to 2 cells (Python's None).
// A host builds its belief material from it, so check_runtime passes by construction. Throws
// std::invalid_argument unless cell_w is finite and > 0.
material_config material_config_for(const belief_material_contract &c, float lam_soft,
                                    float lam_hard, double cell_w);

// A loaded belief-material net that passed the header check (§1.2) and carries its contract
// (§1.1). The twin of BeliefMaterialController's construction and check_runtime; decide is C2.
class belief_material_controller {
public:
  // Takes the net. Throws std::invalid_argument (prefixed with `source`) unless the net has 3 risk
  // channels (so a .cvcnm v2), belief_header_mismatches is empty, its meta holds a valid contract,
  // bp's seen rule is raycast, patch_size >= 2 and patch_extent_m is finite and > 0. Checked in
  // Python's order, so the same broken file gives the same first error.
  belief_material_controller(coef_energy_net net, const belief_material_params &bp = {},
                             int patch_size = 25, double patch_extent_m = 100.0,
                             const std::string &source = "checkpoint");
  // Loads `path` (coef_energy_net::load: std::runtime_error on a bad file), then as above.
  static belief_material_controller from_cvcnm(const std::string &path,
                                               const belief_material_params &bp = {},
                                               int patch_size = 25, double patch_extent_m = 100.0);

  // §1.3: throws std::invalid_argument, naming every mismatch, unless belief_runtime_mismatches
  // is empty.
  void check_runtime(const material_config &mc) const;

  const belief_material_contract &contract() const { return contract_; }
  const coef_energy_net &net() const { return net_; }
  const belief_material_params &belief() const { return belief_; }
  int patch_size() const { return patch_size_; }
  double patch_extent_m() const { return patch_extent_m_; }

private:
  coef_energy_net net_;
  belief_material_params belief_;
  int patch_size_;
  double patch_extent_m_;
  belief_material_contract contract_;
};

// §2.1: one agent's model inputs.
struct belief_material_controller_inputs {
  std::vector<float> obs_feats;       // [N*6] [cx, cy, R, W, dx - cx, dy - cy], robot-relative / s
  std::vector<std::uint8_t> obs_mask; // [N], all 1: every supplied obstacle is a real token
  float goal_feats[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [dx, dy, dist, 1]
  std::vector<float> risk_patch;                  // [3*P*P] [r~, hard, seen]
};

// The belief planes the crop reads (row-major rows x cols over the world bounds): channel 0 of the
// belief material stack (r~), hard_b and the material seen mask.
struct belief_material_view {
  const float *risk = nullptr;
  const std::uint8_t *hard_b = nullptr;
  const std::uint8_t *seen = nullptr;
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
};

// §2.1: the twin of build_belief_material_controller_inputs. p = (px, py) and g = (gx, gy) are the
// agent's world position and goal; C [N*2], R [N], W [N] its already range-limited obstacles in
// world metres (null when N is 0). In float32, one operation at a time with s = (float)scale:
//   pf = (float)p, gf = (float)g, d = (gf - pf) / s, goal_feats = [d.x, d.y, |d|, 1],
//   obs_feats[k] = [l.x, l.y, (float)R[k] / s, (float)W[k], d.x - l.x, d.y - l.y],
//     l = ((float)C[k] - pf) / s,
//   risk_patch = material_patch_belief(planes, (double)pf, patch_extent_m, P, bp):
// the crop is centred on the float32 position, as Python's is. Throws std::invalid_argument
// unless p and g are finite, N >= 0 (with C, R and W when N > 0), feature_scale_m is finite and
// > 0, and the planes are set; material_patch_belief checks P, the extent and bp.
void build_belief_material_controller_inputs(double px, double py, double gx, double gy,
                                             const float *C, const float *R, const float *W, int N,
                                             const belief_material_view &planes,
                                             double patch_extent_m, int P, double feature_scale_m,
                                             const belief_material_params &bp,
                                             belief_material_controller_inputs &out);

// §2.2: the twin of GRL-SNAM's scan_visible_geometry (both Austin tools' obstacle source). The
// candidates are the cells with last_visible & truth (occupied cells this tick's rays touched); a
// cell's centre is min + index / (n - 1) * (max - min) in float64, stored as float32; its distance
// to (px, py) is computed in float64 from the float32 centre. The nearest max_obstacles are kept,
// nearest first, equal distances in row-major order (a stable sort, decision E5). Every radius is
// 0.5 * min(cell_w, cell_h) and every weight 1. Writes C [k*2], R [k], W [k] (each sized for
// max_obstacles) and returns k. Throws std::invalid_argument unless rows, cols >= 2, the bounds
// and position are finite with max > min, max_obstacles >= 0, and the buffers are set when
// max_obstacles > 0.
int visible_obstacles(const std::uint8_t *last_visible, const std::uint8_t *truth, int rows,
                      int cols, double min_x, double min_y, double max_x, double max_y, double px,
                      double py, int max_obstacles, float *C, float *R, float *W);

// Python's repr(float): the shortest round-trip digits, fixed when the decimal exponent is in
// [-4, 16) and with ".0" on a whole number, else d.ddde+XX (at least two exponent digits). nan
// and inf as json.dumps writes them (NaN, Infinity, -Infinity). Exposed for the tests.
std::string python_float_repr(double v);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_BELIEF_MATERIAL_CONTROLLER_H

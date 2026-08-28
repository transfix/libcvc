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

// material.h — material-aware navigation: per-cell terrain risk + hard-hazard
// data alongside the geometry SDF, ported from GRL-SNAM's material-aware
// extension (grl_snam/material.py is the NORMATIVE Python reference; the
// research provenance is GRL-SNAM's material_nav.py, itself a faithful port of
// github.com/SetasAditya/material-aware-grl-snam).
//
// The executed field adds two force terms to the drive (see drive_step_material):
//
//     F_soft = -lam_soft_eff * grad r~          lam_soft_eff = lam_soft * gate
//     db     = -sigmoid(k_sharp * (d_hat_m - phi_m))
//     F_hard = -lam_hard * db * grad phi
//
// with phi_m the UNSIGNED metres distance-to-hard-hazard (one-sided EDT — 0
// inside hazards, unlike build_sdf's signed field), and the frame-wise witness
// gate multiplying lam_soft ONLY (lam_hard is never gated).
//
// Fidelity tiers (mirroring the geometry kernels' contract):
//   material_build, witness_gate(+batch)  — BIT-identical to the Python
//     reference (float64 gate math, shared exact direction table, sequential
//     accumulation, round-half-even cells; the blur is a pinned-op-order
//     scipy-'reflect'-equivalent separable Gaussian). material.cpp must be
//     compiled with -ffp-contract=off (set in CMake) — the blur's f64
//     accumulate would otherwise FMA-contract on aarch64 and break the
//     cross-platform goldens.
//   material_sample, *_material rollouts  — FLOAT tier (same tier and op-order
//     discipline as sdf_sample/drive_step).
//
// The material planes are a PARALLEL [M,6,H,W] stack, deliberately not a 4th
// channel of the geometry field_stack — the 3-channel layout is a cross-ABI
// contract (torch/pycvc/CUDA).

#ifndef CVC_NAV_MATERIAL_H
#define CVC_NAV_MATERIAL_H

#include <cstdint>
#include <cvc/nav/drive.h>
#include <vector>

namespace cvc {
namespace nav {

// Borrowed [M][6][H][W] float32 material planes + the world transform (same
// convention as field_stack). Channels: 0 = r~ (smoothed risk, [0,1]),
// 1 = phi_m (metres to nearest hard cell), 2/3 = d r~ / d(normalized x/y),
// 4/5 = d phi / d(world x/y) (metres per metre — a near-unit direction field).
// Element (m, ch, r, c) at ((m*6 + ch)*H + r)*W + c.
struct material_stack {
  const float *data;
  int M, H, W;
  double mnx, mny, mxx, mxy; // world bounds
  double cx, cy;             // world center
  double S = 1.0;            // world -> normalized scale
};

// Owning result of material_build (one plane).
struct material_planes {
  int rows = 0, cols = 0;
  std::vector<float> risk, phi_m, grad_rx, grad_ry, grad_px, grad_py;
  // Pack into one contiguous [1,6,H,W] block (the material_stack layout).
  std::vector<float> stacked() const;
};

// The derived-plane pipeline, BIT-identical to MaterialGrid._derive:
//   risk    = gaussian_blur(risk_raw, sigma) clipped to [0,1]   (f32 store)
//   phi_m   = sqrt(edt2_squared(hard)) * cell_w                 (f64 chain, one f32 store)
//   grad r~ = np.gradient(risk_f32) / float(cell_w * scale)
//   grad ph = np.gradient(phi_m_f32) / float(cell_w)
// The blur: taps exp(-0.5/(sigma*sigma) * k*k), radius int(4*sigma + 0.5),
// SEQUENTIAL normalization; symmetric/edge-repeat padding (== scipy
// 'reflect'); f64 accumulation through both passes in tap order.
material_planes material_build(const float *risk_raw, const std::uint8_t *hard, int rows, int cols,
                               double cell_w, double scale, double sigma);

// 6-channel bilinear sample at normalized positions on[n*2] (torch grid_sample
// align_corners=True, border padding — the sdf_sample op chain). Outputs:
// risk[n], phi_m[n], grad_r[n*2], grad_phi[n*2]. map_id == nullptr => plane 0.
void material_sample(const material_stack &m, const float *on, int n, const int *map_id,
                     float *risk_out, float *phi_out, float *grad_r_out, float *grad_phi_out,
                     int num_threads = 0);

// ── frame-wise feasibility-witness gate ─────────────────────────────────────
// A local activation WITNESS (it never chooses the executed action): the gate
// is active iff a feasible, progress-making ray improves on the straight-to-
// goal ray's mean risk. gate_hard must already include occupancy (hard | occ):
// a ray through a building is not evidence of a feasible detour. clear_m is
// the metres clearance-to-gate_hard plane. Positions are CONTINUOUS CELL
// coordinates (row, col float64) — callers convert from world once.

struct gate_params {
  int primitive_count = 16; // 16 uses the shared exact direction table
  int horizon_cells = 12;
  double hard_margin_m = 1.0;
  double improvement_margin = 0.05; // mean-ray-risk units
  double material_trigger = 0.45;
  double progress_slack_cells = 0.5;
};

struct gate_decision {
  bool active = false;
  double nominal_risk = 0.0, best_risk = 0.0;
  int feasible_count = 0;
  double dir_r = 0.0, dir_c = 0.0; // best ray direction (rc)
  double end_r = 0.0, end_c = 0.0; // best ray endpoint (rc)
  double min_clearance_m = 0.0;    // NaN when no feasible ray
};

gate_decision witness_gate(const float *risk, const std::uint8_t *gate_hard, const float *clear_m,
                           int rows, int cols, double pos_r, double pos_c, double goal_r,
                           double goal_c, const gate_params &p);

// Batched gate over n agents; outputs are SoA columns. Byte-identical to n
// serial witness_gate calls (and to the Python batch reference).
void witness_gate_batch(const float *risk, const std::uint8_t *gate_hard, const float *clear_m,
                        int rows, int cols, const double *pos_rc, const double *goal_rc, int n,
                        const gate_params &p, std::uint8_t *active_out, double *nominal_out,
                        double *best_out, std::int32_t *count_out, int num_threads = 0);

// ── material coupling for the drive ─────────────────────────────────────────
// Optional per-rollout material forces. stack == nullptr => NO material: the
// *_material entry points then delegate to the plain rollout and are
// byte-identical to it (the sep_radius/sep_gain default-off house pattern).
// lam_soft[n] is the EFFECTIVE soft weight (the caller multiplies the gate
// in); lam_hard[n] is never gated. The material force joins BOTH bicycle
// couplings: the longitudinal projection (F . heading) AND the steering bias
// (alongside F_rep) — the source method integrates a point mass whose force
// bends the trajectory directly; a bicycle discards lateral force, and
// without the steer term the feature degenerates to speed modulation.
struct material_drive {
  const material_stack *stack = nullptr;
  const float *lam_soft = nullptr; // [n]
  const float *lam_hard = nullptr; // [n]
  float k_sharp = 1.25f;           // 1/m   (source constants: 5.0, 3.0 m — see
  float d_hat_m = 12.0f;           // m      grl_snam MaterialParams for why)
};

// Everything sim_world needs to run material-aware: force weights + barrier
// constants + gate parameters + the blur sigma used at set_material time.
// gate.hard_margin_m <= 0 means "2 grid cells" (the source's margin at its
// own resolution). Defaults match GRL-SNAM's MaterialParams (sim-frame
// retunes; see that class for why they differ from the source constants).
struct material_config {
  float lam_soft = 0.5f;
  float lam_hard = 1.0f;
  float k_sharp = 1.25f; // 1/m
  float d_hat_m = 12.0f; // m
  double sigma = 1.0;    // blur, in cells
  bool gate_enabled = true;
  gate_params gate; // horizon_cells/margins; hard_margin_m <= 0 => 2*cell_w
};

void bicycle_rollout_material(const field_stack &f, float *o, float *th, float *sp,
                              const float *goal, const float *al, const float *be, const float *ga,
                              int n, const int *map_id, const veh_params &v,
                              const material_drive &mat, float *minclr_out, int num_threads = 0);

void drive_step_material(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                         const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                         const material_drive &mat, float *minclr_out, int num_threads = 0);

namespace detail {
// One 6-channel bilinear sample (implemented in material.cpp so it compiles
// under -ffp-contract=off; called per substep by the material rollouts).
void material_sample_point(const material_stack &m, int plane, float onx, float ony, float &risk,
                           float &phi, float &grx, float &gry, float &gpx, float &gpy);
} // namespace detail

} // namespace nav
} // namespace cvc

#endif

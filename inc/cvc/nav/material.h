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

#include <cmath>
#include <cstdint>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/geom_rollout.h>
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
//
// Grouped material planes (P2a): risk/gate_hard/clear_m are [M,rows,cols] stacks
// (plane m at m*rows*cols); map_id[i] selects agent i's plane. map_id == nullptr
// => plane 0 for all (M == 1, the back-compat single-plane case), so old callers
// are unchanged. With map_id it is byte-identical to per-plane serial
// witness_gate — the agents mapped to one plane read the same rasters a single
// serial call over that plane would.
void witness_gate_batch(const float *risk, const std::uint8_t *gate_hard, const float *clear_m,
                        int rows, int cols, const double *pos_rc, const double *goal_rc, int n,
                        const gate_params &p, std::uint8_t *active_out, double *nominal_out,
                        double *best_out, std::int32_t *count_out, int num_threads = 0,
                        const int *map_id = nullptr);

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
  // Authority MULTIPLIERS on the reroute weights (default 1.0 = no-op). A lam-head net
  // OVERRIDES lam_soft/lam_hard with its learned per-agent columns, so a caller that wants to
  // live-scale material authority (e.g. a live UI force-bias slider) applies the factor here and
  // the drive multiplies the learned lam by it, instead of the caller's fixed lam being clobbered.
  float lam_soft_scale = 1.0f;
  float lam_hard_scale = 1.0f;
};

// Everything sim_world needs to run material-aware: force weights + barrier
// constants + gate parameters + the blur sigma used at set_material time.
// gate.hard_margin_m <= 0 means "2 grid cells" (the source's margin at its
// own resolution). Defaults match GRL-SNAM's MaterialParams (sim-frame
// retunes; see that class for why they differ from the source constants).
struct material_config {
  float lam_soft = 0.5f;
  float lam_hard = 1.0f;
  // Live authority multipliers on the reroute weights (default 1.0 = no-op). Scale the FIXED lam
  // columns AND (via material_drive) the LEARNED lam a lam-head net emits, so tuning material
  // authority at runtime composes with a trained policy instead of being overridden by it.
  float lam_soft_scale = 1.0f;
  float lam_hard_scale = 1.0f;
  float k_sharp = 1.25f; // 1/m
  float d_hat_m = 12.0f; // m
  double sigma = 1.0;    // blur, in cells
  // gate_enabled=false means the witness gate is BYPASSED so lam_soft_eff = lam_soft * 1 —
  // the soft-reroute force runs UNGATED (always-on at full lam_soft for every agent), NOT
  // turned off. It is the `gate` factor pinned to 1 in the lam_soft_eff = lam_soft * gate model
  // above, the fail-OPEN complement of the fail-CLOSED witness_gate kernel. To turn the soft
  // force OFF, set lam_soft = 0 or clear_material() — do not disable the gate. lam_hard is never
  // gated in either case. Defaults true (the gate is applied unless a caller opts out).
  bool gate_enabled = true;
  gate_params gate; // horizon_cells/margins; hard_margin_m <= 0 => 2*cell_w
};

void bicycle_rollout_material(const field_stack &f, float *o, float *th, float *sp,
                              const float *goal, const float *al, const float *be, const float *ga,
                              int n, const int *map_id, const veh_params &v,
                              const material_drive &mat, float *minclr_out, int num_threads = 0);

// Optional tel[n]: per-agent drive_telemetry out (captures mrisk + lam_soft on this path);
// null (default) is byte-identical to not asking.
void drive_step_material(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                         const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                         const material_drive &mat, float *minclr_out, int num_threads = 0,
                         drive_telemetry *tel = nullptr);

// ─── Fused material + external-force drive ───────────────────────────────────
// The learned grip/risk policy (the material path, INCLUDING a lam-head net) AND a generic
// ext_force (e.g. a downstream layer's steering force) applied in the SAME rollout — both forces
// summed into one accumulator and one steering bias (rollout_impl already fuses them; these entry
// points are the only ones that pass BOTH). Without this, a caller must choose material OR ext:
// sim_world's step() gives material priority, so attaching a material stack silently drops the ext
// force. Use the fused path when a host must steer on a trained grip/risk net WHILE its external
// force also pushes. A null ext.sample makes drive_step_material_ext byte-identical to
// drive_step_material; a null material stack makes it byte-identical to drive_step_ext.
void bicycle_rollout_material_ext(const field_stack &f, float *o, float *th, float *sp,
                                  const float *goal, const float *al, const float *be,
                                  const float *ga, int n, const int *map_id, const veh_params &v,
                                  const material_drive &mat, const ext_force &ext,
                                  float *minclr_out, int num_threads = 0);
// Optional tel[n]: captures mrisk + lam_soft AND ext_fx/fy; null (default) is inert.
void drive_step_material_ext(const field_stack &f, float *o, float *th, float *sp,
                             const float *carrot, const coef_mlp &model, int n, const int *map_id,
                             const veh_params &v, const material_drive &mat, const ext_force &ext,
                             float *minclr_out, int num_threads = 0,
                             drive_telemetry *tel = nullptr);

// ── an external force in the material rollouts ───────────────────────────────
// The deployed drive takes an external force alongside the material one
// (drive_step_material_ext with an ext_force, a hook for a caller's own force). material_ext puts
// the same force in the material trainer's rollouts, so the material heads learn in the
// presence of the force they face at deploy. Positions are the batch's: the frame and
// units of o, C and the rollout patch. F_ext joins
//   - the surrogate's force sum, ftot + F_ext (a point mass: `steer` has nothing to act on);
//   - the bicycle's and the dynamic vehicle's F = F_bar + F_goal + F_mat + F_ext, before
//     the division by mass (which the deployed drive does not make: the two agree at mass
//     1), and with `steer` their steering bias, (F_rep + F_mat + F_ext) . left, as
//     drive_step_material_ext adds ext_force.
// A null material_ext is the rollout without one, bit for bit. Exactly one source:
//   - `patch`: K force patches per agent, two channels (fx, fy) each on the rollout patch's
//     grid (Hp, Wp, centred on the start), [B*K*2*Hp*Wp], sampled bilinearly at the current
//     position as the rollout patch's channels are. J = dF/do is the patch's own gradient
//     (zero along a clamped border). `patch_of_step` [B*steps] picks agent b's patch at step
//     s (a schedule: sources switching on and off); null uses patch 0 throughout.
//   - `F`: per-step values [B*steps*2], and optionally J [B*steps*4] (dF/do, row-major
//     (dfx/dox, dfx/doy, dfy/dox, dfy/doy)) for the reverse.
//   - `sample`: a callback giving F and J at agent b's pose at step s, every `hold` steps
//     and held between (the diff rollout's bike_ext_fn with an agent index). C++ only. It
//     runs on the rollout's worker threads, once per sampled step of each forward and of
//     each VJP's recompute.
// The VJPs chain J^T dL/dF_ext into the position adjoint (a held window's sum at the step
// it was sampled), and given g_fext [B*steps*2] ADD dL/dF_ext per step into it. The
// rollouts throw std::invalid_argument on a malformed channel (no source or more than one,
// steps < max(H) where a per-step array needs it, J without F, hold < 1, K < 1, a patch
// index outside [0, K)).
struct material_ext {
  const float *patch = nullptr;       // [B*K*2*Hp*Wp]
  int K = 1;                          // patches per agent
  const int *patch_of_step = nullptr; // [B*steps], each in [0, K); null = patch 0
  const float *F = nullptr;           // [B*steps*2]
  const float *J = nullptr;           // [B*steps*4], optional with F
  void (*sample)(void *user, int b, int s, float ox, float oy, float *f, float *J) = nullptr;
  void *user = nullptr;
  int hold = 1;  // the callback's sampling period, in steps
  int steps = 0; // the per-step arrays' stride: patch_of_step, F, J and a VJP's g_fext
  int steer = 1; // F_ext also steers (bicycle and dynamic), as ext_force::steer
};

// ── U_nav along a material rollout ──────────────────────────────────────────────
// An optional per-step record of the potential the rollout's conservative force descends
// (detail/nav_potential.h, detail::material_potential): the obstacle discs' barriers, the goal
// spring and the patch's soft and hard terms, with the rollout's coefficients (constant along
// it). Damping is dissipation and F_ext a port, so neither is in it. A null record (the
// default) leaves the rollout bit for bit as it was; any of the pointers may be null. Agent b's
// rows start at b*(max_H+1) (potential, terms) and b*max_H (force), max_H = max(H).
//   - potential [B*(max_H+1)]: U at the state step s starts from, then at the end state (index
//     max_H). Steps past H[b] repeat the frozen state's.
//   - terms [B*(max_H+1)*4]: its terms (goal, barrier, soft, hard) in that order.
//   - force [B*max_H*2]: the conservative force step s applied (= -grad U at that state when the
//     patch's gradient channels are its value channels' gradients in the batch's units).
//
// The point-mass surrogate (integrate_surrogate_material) also keeps the per-step energy books
// (the white paper's Eq. 38; book_step below). kinetic has potential's rows; the rest are
// [B*max_H], step s of agent b at b*max_H + s:
//   - kinetic: m/2 |v|^2 at the state step s starts from, then at the end state.
//   - w_damp, w_port, w_clamp, w_cross: the damping work, the port work of F_ext, the clamp loss
//     and the clamp cross term (the last two are zero: this rollout has no speed clamp).
//   - residual: r_E; bound: the cheap bound on |r_E| (energy_residual_bound), every step.
//   - residual_sharp: the sharp check r_hat_E, two extra force evaluations a step, when `sharp`
//     is set (NaN otherwise).
// The books are those of the step taken from the rollout's state with its coefficients and F_ext,
// evaluated in double (detail::material_energy): they resolve r_E - r_hat_E, of order dt^3, which
// is below float round-off on U. So they leave out the rollout's own float rounding, O(eps) a
// step, and a step's books end where the double step ends, within that rounding of the next
// state. Steps past H[b] book zero. The bicycle and dynamic rollouts are not point masses:
// they throw std::invalid_argument when any of these is set.
struct rollout_energy {
  float *potential = nullptr;
  float *terms = nullptr;
  float *force = nullptr;
  float *kinetic = nullptr;
  float *w_damp = nullptr;
  float *w_port = nullptr;
  float *w_clamp = nullptr;
  float *w_cross = nullptr;
  float *residual = nullptr;
  float *residual_sharp = nullptr;
  float *bound = nullptr;
  bool sharp = false;
  // Any of the per-step books asked for (kinetic through bound).
  bool books() const {
    return kinetic || w_damp || w_port || w_clamp || w_cross || residual || residual_sharp || bound;
  }
};

// ── the per-step energy books (the white paper's Eq. 38, its companion's Part V §3.3) ──────────
// One semi-implicit step of a point mass m in a potential U fixed over the step (coefficients
// frozen), with damping gamma, an external (port) force F and a speed clamp:
//   v' = v + dt/m (-g - gamma v + F),  vt = clamp_vmax(v'),  x' = x + dt vt,  g = grad U(x).
// With H = m/2 |v|^2 + U, it books
//   W_gamma = dt/2 gamma v.(v + v'),       W_port  = dt/2 F.(v + v'),
//   W_clamp = m/2 (|v'|^2 - |vt|^2) >= 0,   W_cross = dt g.(vt - v'),
//   r_E = dH + W_gamma + W_clamp - W_port - W_cross
//       = dt^2/2 [vt' Hbar vt - g.(g + gamma v - F)/m]   exactly,
// Hbar the Hessian of U averaged along the step (2 int_0^1 (1-s) grad^2 U(x + s dx) ds). Two
// checks: the cheap bound |r_E| <= energy_residual_bound(...), every step, with the secant
// Lipschitz estimate L_hat = |grad U(x') - grad U(x)| / |x' - x|; and the sharp check
//   r_hat_E = dt^2/2 [vt' grad^2 U(x) vt - g.(g + gamma v - F)/m],  r_E - r_hat_E = O(dt^3).
// The bound catches work of order dt left out of the books (an unlogged force, a stale field,
// an unbooked clamp); the sharp check catches anything beyond third order, such as explicit
// Euler's systematic gain (x' = x + dt v: its r_E sits dt^2 g.(g + gamma v - F)/m above r_hat_E,
// second order, the same order as the bound, which it therefore passes).
struct book_step_args {
  double dt = 0.0, mass = 1.0, gamma = 0.0;
  double v[2] = {0.0, 0.0};  // velocity at the start
  double vp[2] = {0.0, 0.0}; // v', before the clamp
  double vt[2] = {0.0, 0.0}; // clamp_vmax(v'), the velocity the position stepped with
  double dx[2] = {0.0, 0.0}; // x' - x
  double g[2] = {0.0, 0.0};  // grad U(x)
  double g1[2] = {0.0, 0.0}; // grad U(x'), for L_hat
  double F[2] = {0.0, 0.0};  // the port force
  double U0 = 0.0, U1 = 0.0; // U(x), U(x')
  double vHv = NAN;          // vt' grad^2 U(x) vt for the sharp check; NaN skips it
};

struct step_books {
  double kinetic = 0.0; // m/2 |vt|^2, the end state's
  double delta_H = 0.0;
  double w_damp = 0.0, w_port = 0.0, w_clamp = 0.0, w_cross = 0.0;
  double residual = 0.0;       // r_E
  double residual_sharp = NAN; // r_hat_E; NaN without vHv
  double bound = 0.0;          // the cheap bound on |r_E|
};

step_books book_step(const book_step_args &a, double kappa_H = 2.0);

// The cheap bound on |r_E|, from norms: kappa_H dt^2/2 [|g| (|g| + gamma |v| + |F|) / m +
// L_hat |vt|^2]. A tolerance model, not a theorem: L_hat estimates the curvature, it does not
// bound it; kappa_H = 2 is the companion's safety factor.
double energy_residual_bound(double dt, double g, double gamma, double v, double F, double L_hat,
                             double vt, double mass = 1.0, double kappa_H = 2.0);

// ── obstacle-list material surrogate rollout ────────────────────────────────
// The faithful torch-free port of the source method's differentiable
// integrator (GRL-SNAM material_nav.integrate_surrogate_material): explicit
// obstacle discs (C,R) with per-obstacle IPC barriers, a 6-channel local patch
// (r~, phi, dr~/dx, dr~/dy, dphi/dx, dphi/dy) centred at the rollout start and
// bilinearly resampled at the CURRENT position each step, the soft/hard
// material forces, and semi-implicit Euler. This is the representation the
// learned coef_energy_net was trained on (per-obstacle alphas + lam_soft/
// lam_hard), and the forward the P5 training backward differentiates.
//
// B agents; N padded obstacles per agent (mask nonzero = valid); patch
// (B,6,Hp,Wp). o/v (B,2) are updated IN PLACE (o0/v0 -> oT/vT). The loop runs
// to max(H) with a per-agent active gate (matching the vectorized reference,
// including its min_clear updates at frozen positions past H[b]).
struct surrogate_material_params {
  float margin_factor = 0.5f;
  float mass = 1.0f;
  float d_hat_sdf = 3.0f; // metres
  float k_sharp = 5.0f;   // 1/m
};

void integrate_surrogate_material(
    float *o, float *v, const float *goal, const float *C, const float *R, const std::uint8_t *mask,
    const float *alphas, const float *beta, const float *gamma, const float *lam_soft,
    const float *lam_hard, const float *rollout_patch, const float *rr, const float *d_hat,
    const float *dt, const int *H, int B, int N, int Hp, int Wp, const surrogate_material_params &p,
    float *min_clear, float *cum_risk, float *hard_count, float *arc_length, int num_threads = 0,
    const material_ext *ext = nullptr, rollout_energy *energy = nullptr);

// VJP (reverse-mode adjoint) of integrate_surrogate_material — the P5 training
// backward. Given upstream grads on the differentiable outputs (oT, vT,
// min_clear, cum_risk, arc_length; hard_count is a step-function count and its
// grad is identically zero, so it takes no seed), it produces grads w.r.t. the
// learned coefficients (alphas, beta, gamma, lam_soft, lam_hard) — the leaves
// coef_energy_net emits. o0/v0/goal/C/R/patch are data (no grad). Full
// backward-through-time over the same forward (recomputed per agent); the
// per-step force VJPs are the detail/material_rollout.h primitives, so a CUDA
// twin reuses this exact adjoint. Validated by nav_material_rollout_grad_test
// (finite-difference gradcheck — the torch-independent ground truth). Grad
// buffers are ADDED into (zero them first if you want a fresh gradient).
void integrate_surrogate_material_vjp(
    const float *o0, const float *v0, const float *goal, const float *C, const float *R,
    const std::uint8_t *mask, const float *alphas, const float *beta, const float *gamma,
    const float *lam_soft, const float *lam_hard, const float *rollout_patch, const float *rr,
    const float *d_hat, const float *dt, const int *H, int B, int N, int Hp, int Wp,
    const surrogate_material_params &p, const float *g_oT, const float *g_vT,
    const float *g_min_clear, const float *g_cum_risk, const float *g_arc_length, float *g_alphas,
    float *g_beta, float *g_gamma, float *g_lam_soft, float *g_lam_hard, int num_threads = 0,
    const material_ext *ext = nullptr, float *g_fext = nullptr);

// ── the material rollout on the kinematic bicycle ────────────────────────────
// The same force law as integrate_surrogate_material, driven through the deployed
// kinematic bicycle instead of a point mass. The force law is the per-obstacle IPC
// barriers on the discs (C, R), the goal spring, damping, and the soft/hard material
// forces from the 6-channel patch. The bicycle is drive.cpp's controller and
// integrator, one substep per step:
//   - the longitudinal command is a = (F . heading - gamma * sp) / mass, clamped;
//   - pure pursuit toward the goal, plus a steering bias k_steer * tanh((F_rep + F_mat) . left);
//   - the corner, stop, creep and reverse governors;
//   - the d_cap steering limit and the kinematic update.
// The governor's clearance and normal are the nearest disc's: the role the SDF
// sample plays in the deployed drive. Training through it removes the point-mass
// gap between training and deployment (P8b).
//
// State per agent: position o (B,2), heading th (B) and signed speed sp (B), updated
// IN PLACE (start -> end). Lengths are in the batch's units (the vehicle params
// below included); dt[b] is the step. Like the surrogate, the loop runs to max(H)
// with a per-agent active gate, and min_clear is ungated. A step past H[b] leaves the
// state frozen.
struct bicycle_material_params {
  float L = 0.035f;       // wheelbase
  float delta_max = 0.6f; // steering lock, rad
  float a_max = 1.5f;     // longitudinal acceleration limit
  float a_lat_max = 1.0f; // lateral acceleration limit (corner governor, d_cap)
  float k_steer = 0.8f;   // steering-bias gain
  float vmax = 0.9f;      // speed limit
  bool allow_reverse = true;
};

// Throws std::invalid_argument unless L, a_max, a_lat_max and vmax are finite and
// > 0, delta_max is in (0, pi/2) and k_steer is finite and >= 0. Both bicycle
// rollouts call it first.
void validate(const bicycle_material_params &v);

void integrate_bicycle_material(float *o, float *th, float *sp, const float *goal, const float *C,
                                const float *R, const std::uint8_t *mask, const float *alphas,
                                const float *beta, const float *gamma, const float *lam_soft,
                                const float *lam_hard, const float *rollout_patch, const float *rr,
                                const float *d_hat, const float *dt, const int *H, int B, int N,
                                int Hp, int Wp, const surrogate_material_params &p,
                                const bicycle_material_params &v, float *min_clear, float *cum_risk,
                                float *hard_count, float *arc_length, int num_threads = 0,
                                const material_ext *ext = nullptr,
                                rollout_energy *energy = nullptr);

// VJP of integrate_bicycle_material: upstream grads on the differentiable outputs
// (oT, thT, spT, min_clear, cum_risk, arc_length; any may be null) -> grads w.r.t.
// the learned coefficients (alphas, beta, gamma, lam_soft, lam_hard), ADDED into the
// g_* buffers. The start state, goal, discs and patch are data. Grad flows through
// the branch and clamp each step TOOK, as autograd treats min/max/where. Validated by
// finite differences (nav_material_bike_rollout_test).
void integrate_bicycle_material_vjp(
    const float *o0, const float *th0, const float *sp0, const float *goal, const float *C,
    const float *R, const std::uint8_t *mask, const float *alphas, const float *beta,
    const float *gamma, const float *lam_soft, const float *lam_hard, const float *rollout_patch,
    const float *rr, const float *d_hat, const float *dt, const int *H, int B, int N, int Hp,
    int Wp, const surrogate_material_params &p, const bicycle_material_params &v, const float *g_oT,
    const float *g_thT, const float *g_spT, const float *g_min_clear, const float *g_cum_risk,
    const float *g_arc_length, float *g_alphas, float *g_beta, float *g_gamma, float *g_lam_soft,
    float *g_lam_hard, int num_threads = 0, const material_ext *ext = nullptr,
    float *g_fext = nullptr);

// L_multi on the bicycle: multi_start_penalty (geom_rollout.h) with its short rollouts
// driven through integrate_bicycle_material instead of a point mass.
//   - Same starts (multi_start_origins), each agent keeping its heading th0 and speed sp0.
//   - Same score: mean softplus(-min_clear / tau) over ms_h steps of ms_dt_mult * dt.
//   - Geometry only, as the original: the material forces are off.
// Returns the penalty and ADDS its grads w.r.t. alphas, beta and gamma into whichever
// g_* are non-null.
double multi_start_penalty_bicycle(const float *alphas, const float *beta, const float *gamma,
                                   const float *o0, const float *th0, const float *sp0,
                                   const float *goal, const float *C, const float *R,
                                   const std::uint8_t *mask, const float *rr, const float *d_hat,
                                   const float *dt, int B, int N, const multi_start_params &p,
                                   const surrogate_material_params &sp,
                                   const bicycle_material_params &v, float *g_alphas, float *g_beta,
                                   float *g_gamma, int num_threads = 0);

// ── the material rollout on the dynamic vehicle (P8b) ─────────────────────────
// integrate_bicycle_material's controller (the same force law, pure pursuit, governors
// and d_cap), its commands (delta, a) driving the dynamic single-track vehicle
// (diff::dyn_plant, detail/dyn_bike.h) in place of the kinematic update: tires, grip,
// load transfer and actuator lag, so the material policy learns what the vehicle can do.
//
// State per agent: the rear axle o (B,2), heading th (B), longitudinal speed vx (B), and
// x (B,4) = (vy, r, delta, a), all IN PLACE (start -> end). `dyn` is the vehicle in the
// batch's length units; its wheelbase lf + lr must be v.L. Arc length and cumulative risk
// follow the dynamic path. Like the bicycle, the loop runs to max(H) with a per-agent
// active gate. CPU only.
void integrate_dynamic_material(
    float *o, float *th, float *vx, float *x, const float *goal, const float *C, const float *R,
    const std::uint8_t *mask, const float *alphas, const float *beta, const float *gamma,
    const float *lam_soft, const float *lam_hard, const float *rollout_patch, const float *rr,
    const float *d_hat, const float *dt, const int *H, int B, int N, int Hp, int Wp,
    const surrogate_material_params &p, const bicycle_material_params &v,
    const diff::dyn_bike_params &dyn, float *min_clear, float *cum_risk, float *hard_count,
    float *arc_length, int num_threads = 0, const material_ext *ext = nullptr,
    rollout_energy *energy = nullptr);

// VJP of integrate_dynamic_material: upstream grads on (oT, thT, vxT, xT, min_clear,
// cum_risk, arc_length; any may be null) -> grads w.r.t. the learned coefficients, ADDED
// into the g_* buffers. The plant's part comes from its forward-mode duals
// (diff::dyn_plant_vjp), the controller's from the bicycle's hand-written reverse.
void integrate_dynamic_material_vjp(
    const float *o0, const float *th0, const float *vx0, const float *x0, const float *goal,
    const float *C, const float *R, const std::uint8_t *mask, const float *alphas,
    const float *beta, const float *gamma, const float *lam_soft, const float *lam_hard,
    const float *rollout_patch, const float *rr, const float *d_hat, const float *dt, const int *H,
    int B, int N, int Hp, int Wp, const surrogate_material_params &p,
    const bicycle_material_params &v, const diff::dyn_bike_params &dyn, const float *g_oT,
    const float *g_thT, const float *g_vxT, const float *g_xT, const float *g_min_clear,
    const float *g_cum_risk, const float *g_arc_length, float *g_alphas, float *g_beta,
    float *g_gamma, float *g_lam_soft, float *g_lam_hard, int num_threads = 0,
    const material_ext *ext = nullptr, float *g_fext = nullptr);

// L_multi on the dynamic vehicle: multi_start_penalty_bicycle with its short rollouts
// through integrate_dynamic_material, each from rest in its x (zeros) and the agent's
// heading th0 and speed vx0.
double multi_start_penalty_dynamic(const float *alphas, const float *beta, const float *gamma,
                                   const float *o0, const float *th0, const float *vx0,
                                   const float *goal, const float *C, const float *R,
                                   const std::uint8_t *mask, const float *rr, const float *d_hat,
                                   const float *dt, int B, int N, const multi_start_params &p,
                                   const surrogate_material_params &sp,
                                   const bicycle_material_params &v,
                                   const diff::dyn_bike_params &dyn, float *g_alphas, float *g_beta,
                                   float *g_gamma, int num_threads = 0);

// ── CUDA twins (defined in material_rollout.cu; CVC_ENABLE_CUDA only) ─────────
// The GPU transcription of the two rollout functions above: one thread per
// agent, reusing the SAME detail/material_rollout.h CVC_HD primitives, so the
// CPU finite-difference gradcheck covers the device adjoint too. Same signatures
// and semantics as the host versions (all pointers are HOST memory; the wrappers
// copy to/from the device). Validated float-equivalent + grad-parity against the
// CPU by nav_material_rollout_cuda_test. Gated by material_rollout_cuda_available()
// (built with CUDA AND a device present); the wrappers throw otherwise.
// Horizon cap: max(H) must be <= material_rollout_cuda_max_horizon() (the device
// backward stores the trajectory in per-thread local memory).
//
// Both probes are ALWAYS defined — the .cu supplies the real ones, and
// material_rollout.cpp a `false` / `0` stub for CPU-only builds — so a caller
// (or a language binding) can ask without a CVC_USING_CUDA guard.
bool material_rollout_cuda_available();

// Largest max(H) integrate_surrogate_material_vjp_cuda accepts; it THROWS above
// this rather than degrading, so callers wanting the silent CPU fallback must
// check the batch against it (material_loss_and_grad does). 0 on a CPU-only build.
int material_rollout_cuda_max_horizon();

void integrate_surrogate_material_cuda(
    float *o, float *v, const float *goal, const float *C, const float *R, const std::uint8_t *mask,
    const float *alphas, const float *beta, const float *gamma, const float *lam_soft,
    const float *lam_hard, const float *rollout_patch, const float *rr, const float *d_hat,
    const float *dt, const int *H, int B, int N, int Hp, int Wp, const surrogate_material_params &p,
    float *min_clear, float *cum_risk, float *hard_count, float *arc_length);

void integrate_surrogate_material_vjp_cuda(
    const float *o0, const float *v0, const float *goal, const float *C, const float *R,
    const std::uint8_t *mask, const float *alphas, const float *beta, const float *gamma,
    const float *lam_soft, const float *lam_hard, const float *rollout_patch, const float *rr,
    const float *d_hat, const float *dt, const int *H, int B, int N, int Hp, int Wp,
    const surrogate_material_params &p, const float *g_oT, const float *g_vT,
    const float *g_min_clear, const float *g_cum_risk, const float *g_arc_length, float *g_alphas,
    float *g_beta, float *g_gamma, float *g_lam_soft, float *g_lam_hard);

namespace detail {
// One 6-channel bilinear sample (implemented in material.cpp so it compiles
// under -ffp-contract=off; called per substep by the material rollouts).
void material_sample_point(const material_stack &m, int plane, float onx, float ony, float &risk,
                           float &phi, float &grx, float &gry, float &gpx, float &gpy);
} // namespace detail

} // namespace nav
} // namespace cvc

#endif

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

// drive.h — the torch-free reactive vehicle drive for cvc::nav.
//
// The GRL-SNAM navigation kernels (grid_nav.h) are a bit-identical numpy port;
// this header begins the DRIVE port — the per-agent inference/integration that
// today runs in Python/torch (SDFField.sample -> CoefMLP -> bicycle_rollout ->
// the carrot FSM). It is deliberately libtorch-free so a pure-C++ host (a
// renderer / game engine) can run the swarm with no Python at all.
//
// Fidelity contract (see docs/CVCNAV_CPP_PORT_ROADMAP.md §1): the boundary is
// the bilinear sample. Everything upstream (belief -> occupancy -> EDT -> the
// built SDF field) stays BIT-identical to numpy; from the sample onward the
// contract relaxes to FLOAT-EQUIVALENT (<= ~1 ULP of the torch reference), which
// is validated by a fuzz test, never wired transparently into the torch path.
// Interior arithmetic is float32 to track torch's float32; this TU inherits the
// no -ffast-math / no -ffp-contract=fast discipline of the nav kernels, and the
// design is SoA + branch-light so a CUDA drive.cu can share the same math and
// the same .cvcnav weights.

#ifndef __CVC_NAV_DRIVE_H__
#define __CVC_NAV_DRIVE_H__

#include <cstdint>

namespace cvc {
namespace nav {

class coef_mlp;

// A stack of M built SDF fields sampled by the drive. `data` is the borrowed
// [M][3][H][W] row-major block (channel 0 = phi, 1 = normal_x, 2 = normal_y;
// element (m,ch,r,c) at ((m*3+ch)*H + r)*W + c) — exactly the memory of the
// torch SDFField/BatchedSDFField `.field` tensor. The world<->grid constants are
// the SDFField's: a normalized (centered) position `on` maps to world via
// `w = on/S + center`, then to the [-1,1] grid via `g = 2*(w-min)/(max-min)-1`.
struct field_stack {
  const float *data = nullptr; // [M*3*H*W], borrowed
  int M = 0, H = 0, W = 0;
  double mnx = 0, mny = 0, mxx = 0, mxy = 0; // world bounds (min_x,min_y,max_x,max_y)
  double cx = 0, cy = 0;                     // world center
  double S = 1.0;                            // world -> normalized scale
};

// A single-plane grip raster (grl_snam.material.FrictionField). `mu == 1` is the
// REFERENCE DRY surface the vehicle constants are already quoted against, so a
// null `data` and a uniform-1 plane are both exactly the pre-grip rollout. Same
// [M][H][W] layout and world<->grid constants as `field_stack`, one channel:
// element (m,r,c) at (m*H + r)*W + c. Deliberately separate from the material
// stack because risk and grip are independent surface properties — ice is
// innocuous to look at and lethal to drive on, rubble is the reverse.
struct friction_field {
  const float *data = nullptr; // [M*H*W], borrowed
  int M = 0, H = 0, W = 0;
  double mnx = 0, mny = 0, mxx = 0, mxy = 0; // world bounds
  double cx = 0, cy = 0;                     // world center
  double S = 1.0;                            // world -> normalized scale
};

// Sample the field at `n` normalized positions `on` ([n*2], (x,y) interleaved,
// float32 to match torch); agent i samples plane `map_id[i]` (pass map_id ==
// nullptr for the shared case = plane 0 for all). Writes phi_out[n] and the
// L2-normalized outward normal normal_out[n*2] ((x,y) interleaved). Float-
// equivalent to SDFField.sample / BatchedSDFField.sample: torch grid_sample
// bilinear with align_corners=True and padding_mode="border", then
// nrm / (|nrm| + 1e-6). Agents are independent — threaded across `num_threads`
// workers (<=0 => hardware concurrency).
void sdf_sample(const field_stack &f, const float *on, int n, const int *map_id, float *phi_out,
                float *normal_out, int num_threads = 0);

// Local features for the coefficient net, float-equivalent to sdf_nav.coef_feats:
// feat = [phi, |goal-o|, gdir_x, gdir_y, gdir . unit_normal] per agent, where
// gdir = (goal-o)/(|goal-o|+1e-6). Samples the field at each `on` (agent i ->
// plane map_id[i]). `goal` is [n*2] normalized (the carrot). Writes feat_out
// [n*5] row-major.
// Passing `grip` appends the sampled mu as a SIXTH feature (stride 6 instead of
// 5), matching sdf_nav.coef_feats(friction=...). Without it the drive can only
// discover ice by standing on it, so anticipation has to reach the coefficients
// — and they cannot anticipate what they cannot see. A 6-feature net is not a
// retrain from scratch: sdf_nav.widen_coef_mlp lifts a trained 5-feature net to
// one whose mu column is zero, which is output-identical at init.
void coef_feats(const field_stack &f, const float *on, const float *goal, int n, const int *map_id,
                float *feat_out, int num_threads = 0, const friction_field *grip = nullptr);

// Fixed vehicle + integration parameters for the bicycle rollout (the SdfNavigator
// VEHICLE_DEFAULTS + meta): all float32 to match torch. `nsub` substeps per tick.
//
// The three optional refinements below are each inert at their defaults — a zero
// count, a zero width, a null pointer — so an unmodified caller gets the legacy
// trace bit-for-bit and every stored .cvcnav weight stays valid. They live in
// this struct rather than behind new entry points so `bicycle_rollout`,
// `bicycle_rollout_material` and `drive_step` all pick them up unchanged.
struct veh_params {
  float rr = 0, d_hat = 0, dt = 0, vmax = 0.9f; // meta / kw
  float L = 0.035f, delta_max = 0.6f, a_max = 1.5f, a_lat_max = 1.0f, k_steer = 0.8f;
  int nsub = 1;
  bool allow_reverse = true;

  // FOOTPRINT. `n_body == 0` = the legacy single disc of radius `rr` at the
  // rear axle. Otherwise `n_body` discs of radius `body_rr` at the longitudinal
  // offsets in `body_offsets` (normalized, along the heading, from the rear
  // axle); a car is {0, L/2, L} at body_rr ~ half-width. Clearance is the MIN
  // over discs and the barrier force their SUM, so the nose is pushed off a
  // wall the rear axle cannot see. `rr` is then unused by the drive, and the
  // governor / creep / nose-blocked margins switch to `body_rr` — sizing them
  // for a body 12x too fat is what made the tighter footprint worthless.
  //
  // SET `body_gain` WITH THIS (see below). The SUM is a K-times gain on the
  // learned `al`, which was fit for one sample point; uncorrected it does not
  // break the vehicle, it makes it TIMID — more standoff, fewer collisions,
  // longer to arrive, so it misses any fixed budget.
  const float *body_offsets = nullptr; // [n_body], borrowed
  int n_body = 0;
  float body_rr = 0.0f;
  // Scales the SUMMED barrier. 1/n_body cancels the K-times gain the sum puts on
  // the learned `al` (fit for ONE sample point). USE FULL RADIUS WITH 1/n.
  // Measured on the grl-snam city story, 5 seeds x 4 agents:
  //
  //   arm                    reach@700  reach@1600  pen/agent  clearance
  //   disc 0.150 (legacy)       45%         75%        2.9      2.92 m
  //   fp3 0.150, gain 1          0%         30%        2.8      3.65 m
  //   fp3 0.150, gain 1/3       35%         60%        2.8      3.65 m
  //   fp3 0.075, gain 1/3       50%         60%        7.2      2.59 m
  //
  // Gain-corrected it keeps the lower collision rate AND ~0.7 m more standoff.
  // The last row is the trap: smaller discs look best at a tight budget, but by
  // 1600 ticks they reach the SAME as full radius with 2.5x the collisions —
  // the extra reach was only ever borrowed from safety.
  float body_gain = 1.0f;

  // STEERING LOCK. 0 = none. The bicycle's `delta` is the virtual centre-wheel
  // angle; on a real Ackermann axle the INNER wheel reaches the mechanical lock
  // first, so the achievable virtual angle is atan(L/(L/tan(delta_max)+t/2)).
  // At t = 0.6 L that is 14% less steer and a 20% larger R_min.
  float track_width = 0.0f;

  // GRIP. Null = mu == 1 everywhere. Both actuator limits are friction-limited
  // in reality, so `a_max` and `a_lat_max` scale together with the sampled mu:
  // on ice the corner cap and the stopping governor collapse at the same time.
  // This is understeer-as-a-curvature-limit, NOT a sideslip skid — a kinematic
  // bicycle has no lateral velocity state, so the vehicle runs wide rather than
  // fishtailing, and it therefore carries MORE speed through a corner it fails
  // to take, not less. mu is sampled at the CURRENT pose, so a vehicle entering
  // ice at speed genuinely cannot brake in time; that is the intended failure
  // mode. See docs/MATERIAL_NAV.md "Grip" in the grl-snam repo.
  const friction_field *grip = nullptr;
};

// Kinematic-bicycle rollout — one drive tick of `v.nsub` substeps per agent,
// float-equivalent to sdf_nav.bicycle_rollout(steps=1). Each substep re-samples
// the field (unit normal) at the agent's current position for the IPC wall
// barrier; al/be/ga are the fixed per-agent coefficients (from coef_feats +
// coef_mlp). `goal` is [n*2] normalized (the carrot). Updates o[n*2], th[n],
// sp[n] IN PLACE and writes minclr_out[n] (min clearance seen). Agents are
// independent — threaded across `num_threads` workers.
void bicycle_rollout(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                     const float *al, const float *be, const float *ga, int n, const int *map_id,
                     const veh_params &v, float *minclr_out, int num_threads = 0);

// The whole per-agent drive for one tick, fused: sample -> coef_feats ->
// coef_mlp -> bicycle_rollout(nsub substeps), given the carrot each agent is
// chasing. Equivalent to calling coef_feats + model.forward + bicycle_rollout in
// sequence (a CUDA kernel fuses these into one launch). Updates o[n*2], th[n],
// sp[n] IN PLACE and writes minclr_out[n]. Agents are independent — threaded.
void drive_step(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                float *minclr_out, int num_threads = 0);

// ─── Carrot state machine (swarm.py._plan_carrot) ────────────────────────────

// The per-agent steering-carrot FSM state, as SoA columns (all length n; the
// pos_hist ring is n*40*2). The FSM reads and writes these in place. Each agent
// is independent (reads/writes only its own columns), so it is parallel.
struct fsm_state {
  int *stall = nullptr;                   // stall / displacement-stall counter
  int *mode = nullptr;                    // 0 = seek, 1 = wall-follow
  float *turn = nullptr;                  // wall-follow turn direction (+/-1)
  float *dhit = nullptr;                  // goal distance when wall-follow was entered
  float *best = nullptr;                  // best (closest) goal distance seen
  float *wall_entry = nullptr;            // [n*2] position where wall-follow entered
  std::uint8_t *we_valid = nullptr;       // wall_entry has been set
  const std::uint8_t *tracking = nullptr; // moving-goal (displacement-stall) mode
  float *pos_hist = nullptr;              // [n*40*2] displacement ring buffer
  int *hist_count = nullptr;              // ring fill count
  const std::uint8_t *parked = nullptr;   // braking / holding at goal
  const std::uint8_t *active = nullptr;   // agent participates this tick
};

struct carrot_params {
  float reach_tol = 0.8f; // normalized "arrived" radius
  float a_max = 1.5f;     // brake decel used by the parked branch
  float dt = 0.06f;       // world dt
};

// Advance the carrot FSM one tick and place each agent's steering carrot,
// float-equivalent to swarm.py._plan_carrot. `phi`/`nrm` are the shared-field
// sample at each agent's current position (phi[n], nrm[n*2] unit normals). The
// FSM columns in `s` are updated in place (and `sp` is decremented on the parked
// branch). Writes carrot_out[n*2] (normalized). Parallel across agents.
void carrot_step(const float *o, const float *goal, const float *th, float *sp, const float *phi,
                 const float *nrm, const fsm_state &s, int n, const carrot_params &p,
                 float *carrot_out, int num_threads = 0);

// ─── CUDA drive (device-resident; validation on this box, bench on a GPU box) ──
// Defined in nav/drive.cu, compiled only when CVC_ENABLE_CUDA. The nav .cu is
// built WITHOUT --use_fast_math (-fmad=false, IEEE div/sqrt) so it stays
// float-equivalent to the CPU/torch reference — the deployment GPU path a
// renderer/game engine uses when N outgrows the CPU. These allocate/copy per
// call (a launcher; the fused device-resident sim_world.cu is the next step).

// GPU bilinear sample of plane 0 at `n` normalized positions, float-equivalent to
// sdf_sample. Writes phi_out[n] + unit normal_out[n*2]. Runs on the default GPU.
// True when a CUDA device is actually present, so a test or a host can skip the
// GPU path instead of dying in cudaMalloc. Mirrors
// material_rollout_cuda_available(); compiled to `false` without CVC_ENABLE_CUDA.
bool drive_cuda_available();

void sdf_sample_cuda(const field_stack &f, const float *on, int n, float *phi_out,
                     float *normal_out);

// GPU bicycle rollout with GIVEN coefficients — the device twin of the CPU
// `bicycle_rollout`, float-equivalent to it and to sdf_nav.bicycle_rollout.
// Updates o[n*2], th[n], sp[n] in place and writes minclr_out[n]. Shared field
// (plane 0). `drive_step_cuda` fuses coef_feats + the MLP + this; the unfused
// entry point exists so the vehicle math can be validated against the torch
// reference WITHOUT a trained net in the comparison — which the CPU side always
// had and the GPU side did not, leaving the device rollout unattestable.
void bicycle_rollout_cuda(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                          const float *al, const float *be, const float *ga, int n,
                          const veh_params &v, float *minclr_out);

// GPU fused drive tick (sample -> coef_feats -> coef_mlp -> bicycle nsub), one
// thread per agent, float-equivalent to drive_step. Updates o[n*2], th[n], sp[n]
// in place and writes minclr_out[n]. Shared field (plane 0).
void drive_step_cuda(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                     const coef_mlp &model, int n, const veh_params &v, float *minclr_out);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DRIVE_H__

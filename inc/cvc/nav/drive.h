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
void coef_feats(const field_stack &f, const float *on, const float *goal, int n, const int *map_id,
                float *feat_out, int num_threads = 0);

// Fixed vehicle + integration parameters for the bicycle rollout (the SdfNavigator
// VEHICLE_DEFAULTS + meta): all float32 to match torch. `nsub` substeps per tick.
struct veh_params {
  float rr = 0, d_hat = 0, dt = 0, vmax = 0.9f; // meta / kw
  float L = 0.035f, delta_max = 0.6f, a_max = 1.5f, a_lat_max = 1.0f, k_steer = 0.8f;
  int nsub = 1;
  bool allow_reverse = true;
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

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DRIVE_H__

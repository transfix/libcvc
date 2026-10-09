/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// geom_rollout.h — the GEOMETRY-only surrogate rollout and the multi-start
// robustness penalty (L_multi) of GRL-SNAM's material trainer
// (surrogate_robust.py: integrate_surrogate_v2 + multi_start_penalty). This is
// the one training-loss term the material path (material_train.h) deferred: it
// backprops only into the geometry coefficients (alphas, beta, gamma), never the
// material heads, and runs a DISTINCT integrator — no material forces, no risk
// patch. Sharing the material rollout was impossible for exactly that reason, so
// it gets its own forward+adjoint here (reusing the CVC_HD IPC primitives in
// detail/material_rollout.h). Validated by nav_geom_rollout_grad_test
// (finite-difference gradchecks + goldens from the Python references).
//
// Two Python references exist and they disagree on the integrator:
//   - the material fork (SetasAditya/material-aware-grl-snam,
//     full_code/surrogate_robust.py), which train_material.py — this trainer's
//     normative source — imports: EXPLICIT Euler (position steps with the OLD
//     velocity, then the velocity updates). The default here.
//   - GRL-SNAM's grl_snam/surrogate_robust.py (semi-implicit since f5a015f):
//     the velocity updates first and the position steps with v_{n+1}. Selected
//     by semi_implicit = true.

#ifndef CVC_NAV_GEOM_ROLLOUT_H
#define CVC_NAV_GEOM_ROLLOUT_H

#include <cstdint>

namespace cvc {
namespace nav {

struct geom_rollout_params {
  float margin_factor = 0.5f; // R_eff = R + margin_factor * robot_radius
  float mass = 1.0f;
  bool semi_implicit = false; // false: explicit Euler (the fork); true: GRL-SNAM's v2
};

// Geometry rollout (integrate_surrogate_v2). B agents, N padded obstacles (mask
// nonzero = valid). o/v (B,2) updated IN PLACE (o0/v0 -> oT/vT); min_clear (B)
// is the minimum obstacle clearance over the rollout (ungated, matching the
// reference). Forces: F_goal = -beta*(o-goal), F_bar the per-obstacle IPC
// barrier, damping -gamma*v; a = F/mass. Explicit: o += active*dt*v (OLD v),
// then v += active*dt*a. Semi-implicit: v += active*dt*a, then o += active*dt*v.
void integrate_surrogate_v2(float *o, float *v, const float *goal, const float *C, const float *R,
                            const std::uint8_t *mask, const float *alphas, const float *beta,
                            const float *gamma, const float *rr, const float *d_hat,
                            const float *dt, const int *H, int B, int N,
                            const geom_rollout_params &p, float *min_clear, int num_threads = 0);

// VJP of integrate_surrogate_v2: upstream grads on (oT, vT, min_clear) -> grads
// w.r.t. (alphas, beta, gamma) (ADDED into the g_* buffers). o0/v0/goal/C/R are
// data. Any of the three upstream grad pointers may be null (treated as zero).
void integrate_surrogate_v2_vjp(const float *o0, const float *v0, const float *goal, const float *C,
                                const float *R, const std::uint8_t *mask, const float *alphas,
                                const float *beta, const float *gamma, const float *rr,
                                const float *d_hat, const float *dt, const int *H, int B, int N,
                                const geom_rollout_params &p, const float *g_oT, const float *g_vT,
                                const float *g_min_clear, float *g_alphas, float *g_beta,
                                float *g_gamma, int num_threads = 0);

struct multi_start_params {
  float margin_factor = 0.5f;
  float mass = 1.0f;
  float tau = 0.05f;       // clearance-hinge temperature
  int ms_h = 3;            // short-rollout horizon
  float ms_dt_mult = 4.0f; // dt' = ms_dt_mult * dt
  // Start sampling (GRL-SNAM #113). Each of the ms_count starts moves every agent
  // frac ~ U(frac_lo, frac_hi) of its clearance toward its nearest obstacle,
  // drawn per start and per agent from `seed`. The defaults are the legacy
  // deterministic single start (frac = 0.9, one rollout), byte-identical to the
  // pre-sampling port; GRL-SNAM's sampled default is ms_count = 10 (the fork's
  // TrainCfgMaterial), frac (0.8, 0.98). The bounds are double because the
  // Python takes them as Python floats: (hi - lo) is formed in double and only
  // then rounded to float, which is what makes the draws bit-identical.
  int ms_count = 1;
  double frac_lo = 0.9, frac_hi = 0.9; // 0 <= frac_lo <= frac_hi < 1
  std::uint32_t seed = 0;
  bool semi_implicit = false; // the rollout's integrator (see geom_rollout_params)
};

// The ms_count*B start fractions multi_start_penalty uses, start-major
// (fracs[k*B + b]). With frac_hi > frac_lo they are frac_lo + (frac_hi -
// frac_lo)*u, with u drawn from std::mt19937(seed) as (x & 0xFFFFFF) * 2^-24 —
// the stream torch.rand(B, generator=torch.Generator().manual_seed(seed),
// dtype=torch.float32) draws on the CPU, so a seed reproduces the Python draw
// bit for bit. A degenerate range draws nothing and fills frac_lo. Throws
// std::invalid_argument unless 0 <= frac_lo <= frac_hi < 1. Writes nothing when
// ms_count <= 0.
void multi_start_fracs(const multi_start_params &p, int B, float *fracs);

// L_multi = multi_start_penalty (surrogate_robust.py). For each of the ms_count
// starts (multi_start_fracs), move each agent frac of its clearance toward the
// nearest obstacle (a feasibility fallback moves it half a step AWAY instead if
// that penetrates), run a short geometry rollout, and score
//   L = mean_{k,b} softplus(-min_clear_{k,b} / tau).
// All ms_count rollouts run as one batch of ms_count*B agents. A degenerate frac
// range makes every start identical, so it is evaluated once whatever ms_count
// is. An agent with no valid obstacle contributes 0 (its clearance is +inf).
//
// The starts are data (built from o0, the nearest obstacle and the draw), so the
// gradient flows only through the rollout into (alphas, beta, gamma), and a
// fixed seed makes L a deterministic function of the coefficients — what a
// finite-difference check needs. The draw does not advance between calls: a
// training loop that wants fresh starts each step must vary the seed (the pycvc
// trainer uses seed + step). Returns L; ADDS dL/d(coef) into whichever of
// g_alphas/g_beta/g_gamma are non-null. Returns 0 with no grad when N == 0,
// B == 0 or ms_count <= 0. Throws std::invalid_argument on a bad frac range.
double multi_start_penalty(const float *alphas, const float *beta, const float *gamma,
                           const float *o0, const float *v0, const float *goal, const float *C,
                           const float *R, const std::uint8_t *mask, const float *rr,
                           const float *d_hat, const float *dt, const int *H, int B, int N,
                           const multi_start_params &p, float *g_alphas, float *g_beta,
                           float *g_gamma, int num_threads = 0);

} // namespace nav
} // namespace cvc

#endif

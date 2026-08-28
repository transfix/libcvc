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
// material heads, and runs a DISTINCT integrator — EXPLICIT Euler (position
// steps with the OLD velocity, then velocity updates), no material forces, no
// risk patch. Sharing the material rollout was impossible for exactly that
// reason, so it gets its own forward+adjoint here (reusing the CVC_HD IPC
// primitives in detail/material_rollout.h). Validated by
// nav_geom_rollout_grad_test (finite-difference gradcheck).

#ifndef CVC_NAV_GEOM_ROLLOUT_H
#define CVC_NAV_GEOM_ROLLOUT_H

#include <cstdint>

namespace cvc {
namespace nav {

struct geom_rollout_params {
  float margin_factor = 0.5f; // R_eff = R + margin_factor * robot_radius
  float mass = 1.0f;
};

// Explicit-Euler geometry rollout (integrate_surrogate_v2). B agents, N padded
// obstacles (mask nonzero = valid). o/v (B,2) updated IN PLACE (o0/v0 -> oT/vT);
// min_clear (B) is the minimum obstacle clearance over the rollout (ungated,
// matching the reference). Forces: F_goal = -beta*(o-goal), F_bar the per-
// obstacle IPC barrier, damping -gamma*v; a = F/mass; o += active*dt*v (OLD v);
// v += active*dt*a.
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
};

// L_multi = multi_start_penalty (surrogate_robust.py). The source's ms_count loop
// is DETERMINISTIC (no per-iteration randomness), so it is evaluated once: move
// each agent 90% of its clearance toward the nearest obstacle (a feasibility
// fallback nudges it back out if that penetrates), run a short geometry rollout,
// and score L = mean_b softplus(-min_clear_b / tau). The start point is data
// (built from o0 and the nearest obstacle), so the gradient flows only through
// the rollout into (alphas, beta, gamma). Returns L; if g_alphas/g_beta/g_gamma
// are non-null, ADDS dL/d(coef) into them. Returns 0 with no grad when N == 0.
double multi_start_penalty(const float *alphas, const float *beta, const float *gamma,
                           const float *o0, const float *v0, const float *goal, const float *C,
                           const float *R, const std::uint8_t *mask, const float *rr,
                           const float *d_hat, const float *dt, const int *H, int B, int N,
                           const multi_start_params &p, float *g_alphas, float *g_beta,
                           float *g_gamma, int num_threads = 0);

} // namespace nav
} // namespace cvc

#endif

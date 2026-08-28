/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_train.h — the torch-free P5 training STEP: the material-aware loss
// (GRL-SNAM train_material.py step_batch, stage 2) and its gradient w.r.t. the
// CoefEnergyNetMaterial weights. This is where P5's two adjoint families chain:
//
//   dL/d(weights)  =  backward_one            (coef_energy_net, PR #241)
//                  o  integrate_surrogate_material_vjp   (rollout, PR #240)
//                  o  dL/d(rollout outputs) + dL/d(model outputs directly)
//
// The model forward produces the coefficients, the surrogate rollout turns them
// into a trajectory, and the loss scores it; the backward walks that chain in
// reverse. Validated by nav_material_train_test — a finite-difference gradcheck
// of the FULL loss->weights gradient (the roadmap's P5 release gate), the same
// torch-independent ground truth as the per-op and rollout gradchecks.
//
// Loss (stage 2, w_selectivity = 0):
//   L = 0.3*w_traj*L_traj + 0.3*w_vel*L_vel + w_fric*L_fric + w_clear*L_clear
//       + L_nav + w_lreg*L_lreg
//   L_nav = cvar(J, alpha),  J = w_goal*||oT-goal||^2 + w_len*arc + w_risk*cum_risk
//                                + w_hard*hard_count
// with cvar's quantile eta DETACHED (a constant per step) -> the per-agent tail
// mask 1{J>eta}/((1-alpha)B). hard_count is a step-function count: its gradient
// is identically zero, so it shifts J's value but seeds nothing.
//
// SCOPE: L_multi (multi_start_penalty) is DEFERRED to a follow-up — it is a
// SEPARATE geometry rollout (surrogate_robust.integrate_surrogate_v2: EXPLICIT
// Euler, no material) that backprops only into alphas/beta/gamma, so it needs
// its own rollout adjoint rather than this material one. This module is the
// material-path loss; adding L_multi is additive (its grads sum into
// alphas/beta/gamma before backward_one).

#ifndef CVC_NAV_MATERIAL_TRAIN_H
#define CVC_NAV_MATERIAL_TRAIN_H

#include <cstdint>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material.h>

namespace cvc {
namespace nav {

// One padded training batch: B agents, N padded obstacles (obs_mask marks the
// valid ones — shared by the model and the rollout), a P*P risk patch for the
// model and an Hp*Wp 6-channel rollout patch. All pointers are borrowed.
struct material_batch {
  int B = 0, N = 0, P = 0, Hp = 0, Wp = 0;
  // model inputs
  const float *obs_feats = nullptr;       // [B*N*6]
  const std::uint8_t *obs_mask = nullptr; // [B*N] (also the rollout mask)
  const float *goal_feats = nullptr;      // [B*4]
  const float *risk_patch = nullptr;      // [B*2*P*P]
  // rollout inputs
  const float *o0 = nullptr, *v0 = nullptr, *goal = nullptr;  // [B*2]
  const float *C = nullptr;                                   // [B*N*2]
  const float *R = nullptr;                                   // [B*N]
  const float *rollout_patch = nullptr;                       // [B*6*Hp*Wp]
  const float *rr = nullptr, *d_hat = nullptr, *dt = nullptr; // [B]
  const int *H = nullptr;                                     // [B]
  // targets
  const float *o_tgt = nullptr, *v_tgt = nullptr; // [B*2]
  const float *gamma_o = nullptr;                 // [B]
};

struct material_loss_config {
  // aux + episode-cost weights (train_material.py TrainCfgMaterial defaults)
  float w_traj = 1.0f, w_vel = 0.5f, w_fric = 0.1f, w_clear = 5e-3f, w_lreg = 0.01f;
  float w_goal = 2.0f, w_len = 0.01f, w_risk = 1.0f, w_hard = 5.0f, cvar_alpha = 0.95f;
  float lam_soft_max = 5.0f, lam_hard_max = 10.0f; // for the lreg normalization
  surrogate_material_params rollout;               // margin_factor/mass/d_hat_sdf/k_sharp
  int num_threads = 0;
};

// Forward-only loss. `frozen_eta` fixes the CVaR quantile (a NaN means "compute
// it from J") — the gradcheck passes the base-point eta so the finite-difference
// sees the SAME detached quantile the analytic gradient used.
double material_loss(const coef_energy_net &model, const material_batch &b,
                     const material_loss_config &cfg, float frozen_eta);

// Loss + weight gradients (ADDED into `grads`; zero it first for a fresh
// gradient). Returns the scalar loss and, via `eta_out` (optional), the CVaR
// quantile it used (feed that back as material_loss's frozen_eta for gradcheck).
double material_loss_and_grad(const coef_energy_net &model, const material_batch &b,
                              const material_loss_config &cfg, coef_energy_net::param_grads &grads,
                              float *eta_out = nullptr);

// Cosine-annealed learning rate (torch CosineAnnealingLR): anneals lr0 -> eta_min
// over t_max steps; lr(t) = eta_min + (lr0-eta_min)*0.5*(1+cos(pi*t/t_max)).
float cosine_lr(float lr0, float eta_min, int t, int t_max);

// Adam (beta1=0.9, beta2=0.999, eps=1e-8) with GLOBAL-norm gradient clipping over
// all weight tensors (the coef_trainer optimizer, applied to the whole model).
// Moments are keyed like the model's params; step() updates the weights in place.
class material_adam {
public:
  explicit material_adam(const coef_energy_net &model, float grad_clip = 5.0f);
  // One update: global-norm-clip `grad` (as produced by material_loss_and_grad),
  // then bias-corrected Adam at learning rate `lr`.
  void step(coef_energy_net &model, const coef_energy_net::param_grads &grad, float lr);
  long steps() const { return t_; }

private:
  float b1_ = 0.9f, b2_ = 0.999f, eps_ = 1e-8f, grad_clip_ = 5.0f;
  long t_ = 0;
  coef_energy_net::param_grads m_, u_;
  std::vector<std::string> names_;
};

} // namespace nav
} // namespace cvc

#endif

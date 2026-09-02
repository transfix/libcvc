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
//       + w_multi*L_multi + L_nav + w_lreg*L_lreg
//   L_nav = cvar(J, alpha),  J = w_goal*||oT-goal||^2 + w_len*arc + w_risk*cum_risk
//                                + w_hard*hard_count
// with cvar's quantile eta DETACHED (a constant per step) -> the per-agent tail
// mask 1{J>eta}/((1-alpha)B). hard_count is a step-function count: its gradient
// is identically zero, so it shifts J's value but seeds nothing.
//
// L_multi (multi_start_penalty, geom_rollout.h) is a SEPARATE geometry rollout
// (EXPLICIT Euler, no material) that backprops only into alphas/beta/gamma; its
// grads sum into those seeds before backward_one. Set w_multi = 0 to drop it.

#ifndef CVC_NAV_MATERIAL_TRAIN_H
#define CVC_NAV_MATERIAL_TRAIN_H

#include <cstdint>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/geom_rollout.h>
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
  float w_multi = 0.5f;                            // multi-start robustness (L_multi)
  float lam_soft_max = 5.0f, lam_hard_max = 10.0f; // for the lreg normalization
  surrogate_material_params rollout;               // margin_factor/mass/d_hat_sdf/k_sharp
  multi_start_params multi;                        // ms_h/ms_dt_mult/tau (margin/mass synced)
  int num_threads = 0;
};

// Forward-only loss. `frozen_eta` fixes the CVaR quantile (a NaN means "compute
// it from J") — the gradcheck passes the base-point eta so the finite-difference
// sees the SAME detached quantile the analytic gradient used. This is the
// VALIDATION entry point: scoring a held-out split needs the loss without paying
// for (or perturbing) the backward. `use_cuda` routes the model forward and the
// rollout forward through their device twins on the same terms as
// material_loss_and_grad below.
double material_loss(const coef_energy_net &model, const material_batch &b,
                     const material_loss_config &cfg, float frozen_eta, bool use_cuda = false);

// Loss + weight gradients (ADDED into `grads`; zero it first for a fresh
// gradient). Returns the scalar loss and, via `eta_out` (optional), the CVaR
// quantile it used (feed that back as material_loss's frozen_eta for gradcheck).
//
// `use_cuda` routes the four heavy ops — the model forward/backward and the
// surrogate rollout forward/VJP — through their device twins (forward_batch_cuda,
// integrate_surrogate_material_cuda/_vjp_cuda, backward_batch_cuda), keeping the
// loss / seed-grad / multi-start glue on the CPU. Float-equivalent to the CPU path
// (validated by nav_material_train_cuda_test). It silently falls back to the CPU
// ops when the build has no CUDA, when no device is present, and — for the rollout
// VJP specifically — when max(H) exceeds material_rollout_cuda_max_horizon(), which
// that kernel rejects outright rather than degrading. So it is always safe to pass
// true: each device op falls back to its host twin instead of throwing. The
// multi-start term (L_multi) stays on the CPU regardless (there is no
// geom_rollout.cu), as does the loss/seed-grad glue.
double material_loss_and_grad(const coef_energy_net &model, const material_batch &b,
                              const material_loss_config &cfg, coef_energy_net::param_grads &grads,
                              float *eta_out = nullptr, bool use_cuda = false);

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

// CUDA-resident twin of material_adam (defined in material_train.cu; CVC_ENABLE_CUDA
// only). Same math — global-norm-clipped, bias-corrected Adam over the whole model
// — but the weights and the (m,u) moments stay on the DEVICE across steps: the
// weights are flattened + uploaded once at construction, each step() uploads only
// the flattened grad and runs the norm + update on device (coef_train.cu's
// grad_sqnorm_kernel + adam_kernel), and sync_to() downloads the trained weights
// back into a model. This is the optimizer half of a device-resident training loop;
// pair it with backward_batch_cuda for the model grads. Float-equivalent to
// material_adam (the norm reduction order differs from the CPU's sequential sum),
// validated by nav_material_adam_cuda_test. Throws without a CUDA device (guard with
// material_train_cuda_available()).
class material_adam_cuda {
public:
  explicit material_adam_cuda(const coef_energy_net &model, float grad_clip = 5.0f);
  ~material_adam_cuda();
  material_adam_cuda(const material_adam_cuda &) = delete;
  material_adam_cuda &operator=(const material_adam_cuda &) = delete;
  // One update from host grads (as produced by material_loss_and_grad or downloaded
  // from backward_batch_cuda): upload flattened grad, clip, bias-corrected Adam. The
  // device weights are updated in place — call sync_to to read them back.
  void step(const coef_energy_net::param_grads &grad, float lr);
  // Download the current device weights into `model`'s tensors (unflatten).
  void sync_to(coef_energy_net &model) const;
  long steps() const { return t_; }

private:
  float b1_ = 0.9f, b2_ = 0.999f, eps_ = 1e-8f, grad_clip_ = 5.0f;
  long t_ = 0;
  int P_ = 0; // total flattened param count
  std::vector<std::string> names_;
  std::vector<int> off_; // [names_.size()+1] flatten offsets
  // device buffers (raw — no CUDA types in this header)
  float *d_p_ = nullptr, *d_m_ = nullptr, *d_u_ = nullptr, *d_g_ = nullptr, *d_sq_ = nullptr;
};

// True when this build has CUDA AND a device is present. Mirrors
// coef_energy_cuda_available() / train_cuda_available().
bool material_train_cuda_available();

} // namespace nav
} // namespace cvc

#endif

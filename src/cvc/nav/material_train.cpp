/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_train.cpp — see material_train.h. The P5 training step: model forward
// -> surrogate rollout -> loss, and the reverse chain into weight gradients.
// Built with -ffp-contract=off (CMake) for op-order consistency with the forward
// and backward it composes.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/nav/material_train.h>
#include <limits>
#include <vector>

namespace cvc {
namespace nav {

namespace {

// torch.quantile(x, q) with the default linear interpolation over sorted x.
double quantile_linear(std::vector<double> x, double q) {
  if (x.empty())
    return 0.0;
  std::sort(x.begin(), x.end());
  const double pos = q * static_cast<double>(x.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  if (lo + 1 >= x.size())
    return x.back();
  const double frac = pos - static_cast<double>(lo);
  return x[lo] + frac * (x[lo + 1] - x[lo]);
}

inline float sigmoidf(float x) { return 1.0f / (1.0f + std::exp(-x)); }
inline float softplusf(float x) { return x > 20.0f ? x : std::log1p(std::exp(x)); }

// Everything the forward produces that the backward reuses.
struct fwd_cache {
  std::vector<float> alphas, beta, gamma, lam_soft, lam_hard, mu_lat; // model outputs
  std::vector<float> oT, vT, min_clear, cum_risk, hard_count, arc;    // rollout outputs
  std::vector<double> J;
  double eta = 0.0;
};

// Run the model + rollout + loss; if `cache` non-null, fill it. `frozen_eta`
// (NaN => compute from J) fixes the detached CVaR quantile.
double compute_loss(const coef_energy_net &model, const material_batch &b,
                    const material_loss_config &cfg, float frozen_eta, fwd_cache *cache) {
  const int B = b.B, N = b.N, P = b.P;

  // model forward (per agent)
  std::vector<float> alphas(static_cast<std::size_t>(B) * N, 0.0f), beta(B), gamma(B), lam_soft(B),
      lam_hard(B), mu_lat(B);
  for (int i = 0; i < B; ++i)
    model.forward_one(b.obs_feats + static_cast<std::size_t>(i) * N * 6, b.obs_mask + i * N, N,
                      b.goal_feats + static_cast<std::size_t>(i) * 4,
                      b.risk_patch + static_cast<std::size_t>(i) * 2 * P * P, P,
                      alphas.data() + static_cast<std::size_t>(i) * N, &beta[i], &gamma[i],
                      &lam_soft[i], &lam_hard[i], &mu_lat[i]);

  // rollout forward (o/v are updated in place from o0/v0)
  std::vector<float> oT(b.o0, b.o0 + 2 * B), vT(b.v0, b.v0 + 2 * B);
  std::vector<float> min_clear(B), cum_risk(B), hard_count(B), arc(B);
  integrate_surrogate_material(oT.data(), vT.data(), b.goal, b.C, b.R, b.obs_mask, alphas.data(),
                               beta.data(), gamma.data(), lam_soft.data(), lam_hard.data(),
                               b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B, N, b.Hp, b.Wp,
                               cfg.rollout, min_clear.data(), cum_risk.data(), hard_count.data(),
                               arc.data(), cfg.num_threads);

  // episode cost J and CVaR quantile
  std::vector<double> J(B);
  for (int i = 0; i < B; ++i) {
    const double gx = oT[2 * i] - b.goal[2 * i], gy = oT[2 * i + 1] - b.goal[2 * i + 1];
    J[i] = (double)cfg.w_goal * (gx * gx + gy * gy) + (double)cfg.w_len * arc[i] +
           (double)cfg.w_risk * cum_risk[i] + (double)cfg.w_hard * hard_count[i];
  }
  const double eta =
      std::isnan(frozen_eta) ? quantile_linear(J, cfg.cvar_alpha) : (double)frozen_eta;

  // scalar losses (double accumulation)
  const double invB = 1.0 / (double)B;
  double L_traj = 0.0, L_vel = 0.0, L_fric = 0.0, L_clear = 0.0, L_nav_tail = 0.0, L_lreg = 0.0;
  for (int i = 0; i < B; ++i) {
    const double tx = oT[2 * i] - b.o_tgt[2 * i], ty = oT[2 * i + 1] - b.o_tgt[2 * i + 1];
    L_traj += (tx * tx + ty * ty);
    const double vx = vT[2 * i] - b.v_tgt[2 * i], vy = vT[2 * i + 1] - b.v_tgt[2 * i + 1];
    L_vel += (vx * vx + vy * vy);
    const double go = std::min(std::max(b.gamma_o[i], 0.0f), 20.0f);
    const double dgf = gamma[i] - go;
    L_fric += dgf * dgf;
    L_clear += softplusf(-min_clear[i] / 0.05f);
    L_nav_tail += std::max(J[i] - eta, 0.0);
    const double lns = std::min(std::max(lam_soft[i] / cfg.lam_soft_max, 1e-6f), 1.0f - 1e-6f);
    const double lnh = std::min(std::max(lam_hard[i] / cfg.lam_hard_max, 1e-6f), 1.0f - 1e-6f);
    L_lreg += std::log(lns) + std::log(1.0 - lns) + std::log(lnh) + std::log(1.0 - lnh);
  }
  L_traj *= invB * 0.5; // mean over B*2 elements
  L_vel *= invB * 0.5;
  L_fric *= invB;
  L_clear *= invB;
  L_lreg *= -0.25 * invB;
  const double L_nav = eta + L_nav_tail * invB / (1.0 - (double)cfg.cvar_alpha);

  const double L = 0.3 * cfg.w_traj * L_traj + 0.3 * cfg.w_vel * L_vel + cfg.w_fric * L_fric +
                   cfg.w_clear * L_clear + L_nav + cfg.w_lreg * L_lreg;

  if (cache) {
    cache->alphas = std::move(alphas);
    cache->beta = std::move(beta);
    cache->gamma = std::move(gamma);
    cache->lam_soft = std::move(lam_soft);
    cache->lam_hard = std::move(lam_hard);
    cache->mu_lat = std::move(mu_lat);
    cache->oT = std::move(oT);
    cache->vT = std::move(vT);
    cache->min_clear = std::move(min_clear);
    cache->cum_risk = std::move(cum_risk);
    cache->hard_count = std::move(hard_count);
    cache->arc = std::move(arc);
    cache->J = std::move(J);
    cache->eta = eta;
  }
  return L;
}

} // namespace

double material_loss(const coef_energy_net &model, const material_batch &b,
                     const material_loss_config &cfg, float frozen_eta) {
  return compute_loss(model, b, cfg, frozen_eta, nullptr);
}

double material_loss_and_grad(const coef_energy_net &model, const material_batch &b,
                              const material_loss_config &cfg, coef_energy_net::param_grads &grads,
                              float *eta_out) {
  const int B = b.B, N = b.N, P = b.P;
  fwd_cache c;
  const double L = compute_loss(model, b, cfg, std::numeric_limits<float>::quiet_NaN(), &c);
  if (eta_out)
    *eta_out = (float)c.eta;

  const double invB = 1.0 / (double)B;
  const double tail_scale = invB / (1.0 - (double)cfg.cvar_alpha);

  // ── seed grads on the rollout outputs + direct model-output grads ──
  std::vector<float> g_oT(2 * B, 0.0f), g_vT(2 * B, 0.0f), g_min_clear(B, 0.0f),
      g_cum_risk(B, 0.0f), g_arc(B, 0.0f);
  std::vector<float> g_gamma_dir(B, 0.0f), g_lam_soft_dir(B, 0.0f), g_lam_hard_dir(B, 0.0f);
  for (int i = 0; i < B; ++i) {
    const double tail = (c.J[i] > c.eta) ? tail_scale : 0.0; // per-agent CVaR tail mask
    for (int k = 0; k < 2; ++k) {
      const double o = c.oT[2 * i + k];
      const double dtraj = 0.3 * cfg.w_traj * (o - b.o_tgt[2 * i + k]) * invB;
      const double dnav = tail * cfg.w_goal * 2.0 * (o - b.goal[2 * i + k]);
      g_oT[2 * i + k] = (float)(dtraj + dnav);
      g_vT[2 * i + k] = (float)(0.3 * cfg.w_vel * (c.vT[2 * i + k] - b.v_tgt[2 * i + k]) * invB);
    }
    g_arc[i] = (float)(tail * cfg.w_len);
    g_cum_risk[i] = (float)(tail * cfg.w_risk);
    // L_clear = mean softplus(-min_clear/0.05)
    const double s = sigmoidf(-c.min_clear[i] / 0.05f);
    g_min_clear[i] = (float)(cfg.w_clear * (-s / 0.05) * invB);
    // L_fric = mean (gamma - clamp(gamma_o))^2  -> direct grad on gamma
    const double go = std::min(std::max(b.gamma_o[i], 0.0f), 20.0f);
    g_gamma_dir[i] = (float)(cfg.w_fric * 2.0 * (c.gamma[i] - go) * invB);
    // L_lreg = -0.25 mean [log lns + log(1-lns) + log lnh + log(1-lnh)]
    const double lns = std::min(std::max(c.lam_soft[i] / cfg.lam_soft_max, 1e-6f), 1.0f - 1e-6f);
    const double lnh = std::min(std::max(c.lam_hard[i] / cfg.lam_hard_max, 1e-6f), 1.0f - 1e-6f);
    g_lam_soft_dir[i] =
        (float)(cfg.w_lreg * (-0.25 * invB) * (1.0 / lns - 1.0 / (1.0 - lns)) / cfg.lam_soft_max);
    g_lam_hard_dir[i] =
        (float)(cfg.w_lreg * (-0.25 * invB) * (1.0 / lnh - 1.0 / (1.0 - lnh)) / cfg.lam_hard_max);
  }

  // ── rollout backward -> grads on the model outputs ──
  std::vector<float> g_alphas(static_cast<std::size_t>(B) * N, 0.0f), g_beta(B, 0.0f),
      g_gamma(B, 0.0f), g_lam_soft(B, 0.0f), g_lam_hard(B, 0.0f);
  integrate_surrogate_material_vjp(
      b.o0, b.v0, b.goal, b.C, b.R, b.obs_mask, c.alphas.data(), c.beta.data(), c.gamma.data(),
      c.lam_soft.data(), c.lam_hard.data(), b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B, N, b.Hp,
      b.Wp, cfg.rollout, g_oT.data(), g_vT.data(), g_min_clear.data(), g_cum_risk.data(),
      g_arc.data(), g_alphas.data(), g_beta.data(), g_gamma.data(), g_lam_soft.data(),
      g_lam_hard.data(), cfg.num_threads);

  // add the direct (non-rollout) model-output grads
  for (int i = 0; i < B; ++i) {
    g_gamma[i] += g_gamma_dir[i];
    g_lam_soft[i] += g_lam_soft_dir[i];
    g_lam_hard[i] += g_lam_hard_dir[i];
  }

  // ── model backward -> weight grads (per agent, accumulate) ──
  for (int i = 0; i < B; ++i)
    model.backward_one(b.obs_feats + static_cast<std::size_t>(i) * N * 6, b.obs_mask + i * N, N,
                       b.goal_feats + static_cast<std::size_t>(i) * 4,
                       b.risk_patch + static_cast<std::size_t>(i) * 2 * P * P, P,
                       g_alphas.data() + static_cast<std::size_t>(i) * N, g_beta[i], g_gamma[i],
                       g_lam_soft[i], g_lam_hard[i], 0.0f, grads);

  return L;
}

} // namespace nav
} // namespace cvc

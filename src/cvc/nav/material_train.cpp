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
  std::vector<float> th0, sp0, thT, spT;                              // bicycle start and end
  std::vector<float> x0, xT; // dynamic: (vy, r, delta, a) at the start and end [B*4]
  std::vector<double> J;
  double eta = 0.0;
};

// Largest per-agent horizon in the batch — the quantity the device VJP caps.
int max_horizon(const material_batch &b) {
  int m = 0;
  for (int i = 0; i < b.B; ++i)
    m = std::max(m, b.H[i]);
  return m;
}

// The bicycle's start state: the batch's th0/sp0, else derived from v0 (speed |v0|,
// heading along v0, or toward the goal when v0 is zero).
void bicycle_start(const material_batch &b, std::vector<float> &th, std::vector<float> &sp) {
  th.resize(b.B);
  sp.resize(b.B);
  for (int i = 0; i < b.B; ++i) {
    const float vx = b.v0[2 * i], vy = b.v0[2 * i + 1];
    const float speed = std::sqrt(vx * vx + vy * vy);
    if (b.th0)
      th[i] = b.th0[i];
    else if (speed > 1e-6f)
      th[i] = std::atan2(vy, vx);
    else
      th[i] = std::atan2(b.goal[2 * i + 1] - b.o0[2 * i + 1], b.goal[2 * i] - b.o0[2 * i]);
    sp[i] = b.sp0 ? b.sp0[i] : speed;
  }
}

// Run the model + rollout + loss; if `cache` non-null, fill it. `frozen_eta`
// (NaN => compute from J) fixes the detached CVaR quantile.
double compute_loss(const coef_energy_net &model, const material_batch &b,
                    const material_loss_config &cfg, float frozen_eta, fwd_cache *cache,
                    bool use_cuda) {
  const int B = b.B, N = b.N, P = b.P;
  (void)use_cuda;

  // model forward (per agent, or one device batch — the padded batch is a ragged
  // batch with uniform obs_offsets[i] = i*N).
  std::vector<float> alphas(static_cast<std::size_t>(B) * N, 0.0f), beta(B), gamma(B), lam_soft(B),
      lam_hard(B), mu_lat(B);
#ifdef CVC_USING_CUDA
  if (use_cuda && coef_energy_cuda_available()) {
    std::vector<int> off(B + 1);
    for (int i = 0; i <= B; ++i)
      off[i] = i * N;
    model.forward_batch_cuda(b.obs_feats, b.obs_mask, off.data(), B, b.goal_feats, b.risk_patch, P,
                             alphas.data(), beta.data(), gamma.data(), lam_soft.data(),
                             lam_hard.data(), mu_lat.data());
  } else
#endif
    for (int i = 0; i < B; ++i)
      model.forward_one(b.obs_feats + static_cast<std::size_t>(i) * N * 6, b.obs_mask + i * N, N,
                        b.goal_feats + static_cast<std::size_t>(i) * 4,
                        b.risk_patch + static_cast<std::size_t>(i) * 2 * P * P, P,
                        alphas.data() + static_cast<std::size_t>(i) * N, &beta[i], &gamma[i],
                        &lam_soft[i], &lam_hard[i], &mu_lat[i]);

  // rollout forward (o/v are updated in place from o0/v0)
  std::vector<float> oT(b.o0, b.o0 + 2 * B), vT(b.v0, b.v0 + 2 * B);
  std::vector<float> min_clear(B), cum_risk(B), hard_count(B), arc(B);
  std::vector<float> th0, sp0, thT, spT, x0, xT;
  if (cfg.integrator == material_integrator::dynamic) {
    bicycle_start(b, th0, sp0); // heading, and the longitudinal speed vx
    thT = th0;
    spT = sp0;
    x0.assign(static_cast<std::size_t>(B) * 4, 0.0f);
    xT = x0;
    integrate_dynamic_material(
        oT.data(), thT.data(), spT.data(), xT.data(), b.goal, b.C, b.R, b.obs_mask, alphas.data(),
        beta.data(), gamma.data(), lam_soft.data(), lam_hard.data(), b.rollout_patch, b.rr, b.d_hat,
        b.dt, b.H, B, N, b.Hp, b.Wp, cfg.rollout, cfg.bicycle, cfg.dynamic, min_clear.data(),
        cum_risk.data(), hard_count.data(), arc.data(), cfg.num_threads);
    // vT: the rear axle's world velocity, (vx, vy - lr r) turned by the heading.
    for (int i = 0; i < B; ++i) {
      const float vs = xT[4 * i] - cfg.dynamic.lr * xT[4 * i + 1];
      const float c = std::cos(thT[i]), s = std::sin(thT[i]);
      vT[2 * i] = spT[i] * c - vs * s;
      vT[2 * i + 1] = spT[i] * s + vs * c;
    }
  } else if (cfg.integrator == material_integrator::bicycle) {
    bicycle_start(b, th0, sp0);
    thT = th0;
    spT = sp0;
    integrate_bicycle_material(oT.data(), thT.data(), spT.data(), b.goal, b.C, b.R, b.obs_mask,
                               alphas.data(), beta.data(), gamma.data(), lam_soft.data(),
                               lam_hard.data(), b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B, N,
                               b.Hp, b.Wp, cfg.rollout, cfg.bicycle, min_clear.data(),
                               cum_risk.data(), hard_count.data(), arc.data(), cfg.num_threads);
    for (int i = 0; i < B; ++i) {
      vT[2 * i] = spT[i] * std::cos(thT[i]);
      vT[2 * i + 1] = spT[i] * std::sin(thT[i]);
    }
  } else
#ifdef CVC_USING_CUDA
      if (use_cuda && material_rollout_cuda_available())
    integrate_surrogate_material_cuda(oT.data(), vT.data(), b.goal, b.C, b.R, b.obs_mask,
                                      alphas.data(), beta.data(), gamma.data(), lam_soft.data(),
                                      lam_hard.data(), b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B,
                                      N, b.Hp, b.Wp, cfg.rollout, min_clear.data(), cum_risk.data(),
                                      hard_count.data(), arc.data());
  else
#endif
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

  double L = 0.3 * cfg.w_traj * L_traj + 0.3 * cfg.w_vel * L_vel + cfg.w_fric * L_fric +
             cfg.w_clear * L_clear + L_nav + cfg.w_lreg * L_lreg;

  // L_multi (multi-start geometry robustness) — forward value
  if (cfg.w_multi != 0.0f && N > 0) {
    multi_start_params mp = cfg.multi;
    mp.margin_factor = cfg.rollout.margin_factor;
    mp.mass = cfg.rollout.mass;
    const double L_multi =
        cfg.integrator == material_integrator::dynamic
            ? multi_start_penalty_dynamic(alphas.data(), beta.data(), gamma.data(), b.o0,
                                          th0.data(), sp0.data(), b.goal, b.C, b.R, b.obs_mask,
                                          b.rr, b.d_hat, b.dt, B, N, mp, cfg.rollout, cfg.bicycle,
                                          cfg.dynamic, nullptr, nullptr, nullptr, cfg.num_threads)
        : cfg.integrator == material_integrator::bicycle
            ? multi_start_penalty_bicycle(alphas.data(), beta.data(), gamma.data(), b.o0,
                                          th0.data(), sp0.data(), b.goal, b.C, b.R, b.obs_mask,
                                          b.rr, b.d_hat, b.dt, B, N, mp, cfg.rollout, cfg.bicycle,
                                          nullptr, nullptr, nullptr, cfg.num_threads)
            : multi_start_penalty(alphas.data(), beta.data(), gamma.data(), b.o0, b.v0, b.goal, b.C,
                                  b.R, b.obs_mask, b.rr, b.d_hat, b.dt, b.H, B, N, mp, nullptr,
                                  nullptr, nullptr, cfg.num_threads);
    L += (double)cfg.w_multi * L_multi;
  }

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
    cache->th0 = std::move(th0);
    cache->sp0 = std::move(sp0);
    cache->thT = std::move(thT);
    cache->spT = std::move(spT);
    cache->x0 = std::move(x0);
    cache->xT = std::move(xT);
  }
  return L;
}

} // namespace

double material_loss(const coef_energy_net &model, const material_batch &b,
                     const material_loss_config &cfg, float frozen_eta, bool use_cuda) {
  return compute_loss(model, b, cfg, frozen_eta, nullptr, use_cuda);
}

double material_loss_and_grad(const coef_energy_net &model, const material_batch &b,
                              const material_loss_config &cfg, coef_energy_net::param_grads &grads,
                              float *eta_out, bool use_cuda) {
  const int B = b.B, N = b.N, P = b.P;
  (void)use_cuda;
  fwd_cache c;
  const double L =
      compute_loss(model, b, cfg, std::numeric_limits<float>::quiet_NaN(), &c, use_cuda);
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
  if (cfg.integrator == material_integrator::dynamic) {
    // vT = (vx c - vs s, vx s + vs c), vs = vy - lr r: L_vel's seed onto the vehicle's state.
    std::vector<float> g_thT(B), g_vxT(B), g_xT(static_cast<std::size_t>(B) * 4, 0.0f);
    for (int i = 0; i < B; ++i) {
      const float vx = c.spT[i], vs = c.xT[4 * i] - cfg.dynamic.lr * c.xT[4 * i + 1];
      const float cth = std::cos(c.thT[i]), sth = std::sin(c.thT[i]);
      const float gvx = g_vT[2 * i], gvy = g_vT[2 * i + 1];
      g_vxT[i] = gvx * cth + gvy * sth;
      const float g_vs = -gvx * sth + gvy * cth;
      g_xT[4 * i] = g_vs;
      g_xT[4 * i + 1] = -cfg.dynamic.lr * g_vs;
      g_thT[i] = gvx * (-vx * sth - vs * cth) + gvy * (vx * cth - vs * sth);
    }
    integrate_dynamic_material_vjp(
        b.o0, c.th0.data(), c.sp0.data(), c.x0.data(), b.goal, b.C, b.R, b.obs_mask,
        c.alphas.data(), c.beta.data(), c.gamma.data(), c.lam_soft.data(), c.lam_hard.data(),
        b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B, N, b.Hp, b.Wp, cfg.rollout, cfg.bicycle,
        cfg.dynamic, g_oT.data(), g_thT.data(), g_vxT.data(), g_xT.data(), g_min_clear.data(),
        g_cum_risk.data(), g_arc.data(), g_alphas.data(), g_beta.data(), g_gamma.data(),
        g_lam_soft.data(), g_lam_hard.data(), cfg.num_threads);
  } else if (cfg.integrator == material_integrator::bicycle) {
    // vT = sp * (cos th, sin th): carry L_vel's seed back onto the bicycle's state.
    std::vector<float> g_thT(B), g_spT(B);
    for (int i = 0; i < B; ++i) {
      const float cth = std::cos(c.thT[i]), sth = std::sin(c.thT[i]);
      g_spT[i] = g_vT[2 * i] * cth + g_vT[2 * i + 1] * sth;
      g_thT[i] = c.spT[i] * (-g_vT[2 * i] * sth + g_vT[2 * i + 1] * cth);
    }
    integrate_bicycle_material_vjp(
        b.o0, c.th0.data(), c.sp0.data(), b.goal, b.C, b.R, b.obs_mask, c.alphas.data(),
        c.beta.data(), c.gamma.data(), c.lam_soft.data(), c.lam_hard.data(), b.rollout_patch, b.rr,
        b.d_hat, b.dt, b.H, B, N, b.Hp, b.Wp, cfg.rollout, cfg.bicycle, g_oT.data(), g_thT.data(),
        g_spT.data(), g_min_clear.data(), g_cum_risk.data(), g_arc.data(), g_alphas.data(),
        g_beta.data(), g_gamma.data(), g_lam_soft.data(), g_lam_hard.data(), cfg.num_threads);
  } else
#ifdef CVC_USING_CUDA
  // The device VJP stores each agent's whole trajectory in per-thread local
  // memory, so it only accepts max(H) <= material_rollout_cuda_max_horizon() and
  // THROWS above it. use_cuda is documented as always safe, so check the batch
  // here and fall back to the (float-equivalent) host adjoint rather than letting
  // that throw escape. The device FORWARD carries no such cap, so a long-horizon
  // batch still gets the GPU forward — only this one op degrades.
  if (use_cuda && material_rollout_cuda_available() &&
      max_horizon(b) <= material_rollout_cuda_max_horizon())
    integrate_surrogate_material_vjp_cuda(
        b.o0, b.v0, b.goal, b.C, b.R, b.obs_mask, c.alphas.data(), c.beta.data(), c.gamma.data(),
        c.lam_soft.data(), c.lam_hard.data(), b.rollout_patch, b.rr, b.d_hat, b.dt, b.H, B, N, b.Hp,
        b.Wp, cfg.rollout, g_oT.data(), g_vT.data(), g_min_clear.data(), g_cum_risk.data(),
        g_arc.data(), g_alphas.data(), g_beta.data(), g_gamma.data(), g_lam_soft.data(),
        g_lam_hard.data());
  else
#endif
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

  // L_multi grads (geometry robustness) — add w_multi * dL_multi/d(alphas,beta,gamma)
  if (cfg.w_multi != 0.0f && N > 0) {
    multi_start_params mp = cfg.multi;
    mp.margin_factor = cfg.rollout.margin_factor;
    mp.mass = cfg.rollout.mass;
    std::vector<float> gm_al(static_cast<std::size_t>(B) * N, 0.0f), gm_be(B, 0.0f), gm_ga(B, 0.0f);
    if (cfg.integrator == material_integrator::dynamic)
      multi_start_penalty_dynamic(c.alphas.data(), c.beta.data(), c.gamma.data(), b.o0,
                                  c.th0.data(), c.sp0.data(), b.goal, b.C, b.R, b.obs_mask, b.rr,
                                  b.d_hat, b.dt, B, N, mp, cfg.rollout, cfg.bicycle, cfg.dynamic,
                                  gm_al.data(), gm_be.data(), gm_ga.data(), cfg.num_threads);
    else if (cfg.integrator == material_integrator::bicycle)
      multi_start_penalty_bicycle(c.alphas.data(), c.beta.data(), c.gamma.data(), b.o0,
                                  c.th0.data(), c.sp0.data(), b.goal, b.C, b.R, b.obs_mask, b.rr,
                                  b.d_hat, b.dt, B, N, mp, cfg.rollout, cfg.bicycle, gm_al.data(),
                                  gm_be.data(), gm_ga.data(), cfg.num_threads);
    else
      multi_start_penalty(c.alphas.data(), c.beta.data(), c.gamma.data(), b.o0, b.v0, b.goal, b.C,
                          b.R, b.obs_mask, b.rr, b.d_hat, b.dt, b.H, B, N, mp, gm_al.data(),
                          gm_be.data(), gm_ga.data(), cfg.num_threads);
    for (std::size_t i = 0; i < gm_al.size(); ++i)
      g_alphas[i] += cfg.w_multi * gm_al[i];
    for (int i = 0; i < B; ++i) {
      g_beta[i] += cfg.w_multi * gm_be[i];
      g_gamma[i] += cfg.w_multi * gm_ga[i];
    }
  }

  // ── model backward -> weight grads (per agent, accumulate; or one device batch) ──
#ifdef CVC_USING_CUDA
  if (use_cuda && coef_energy_cuda_available()) {
    std::vector<int> off(B + 1);
    for (int i = 0; i <= B; ++i)
      off[i] = i * N;
    const std::vector<float> g_mu_lat(B, 0.0f); // mu_lat is unused in training
    model.backward_batch_cuda(b.obs_feats, b.obs_mask, off.data(), B, b.goal_feats, b.risk_patch, P,
                              g_alphas.data(), g_beta.data(), g_gamma.data(), g_lam_soft.data(),
                              g_lam_hard.data(), g_mu_lat.data(), grads);
  } else
#endif
    for (int i = 0; i < B; ++i)
      model.backward_one(b.obs_feats + static_cast<std::size_t>(i) * N * 6, b.obs_mask + i * N, N,
                         b.goal_feats + static_cast<std::size_t>(i) * 4,
                         b.risk_patch + static_cast<std::size_t>(i) * 2 * P * P, P,
                         g_alphas.data() + static_cast<std::size_t>(i) * N, g_beta[i], g_gamma[i],
                         g_lam_soft[i], g_lam_hard[i], 0.0f, grads);

  return L;
}

float cosine_lr(float lr0, float eta_min, int t, int t_max) {
  if (t_max <= 0)
    return lr0;
  const double pi = 3.14159265358979323846;
  const int tt = t < t_max ? t : t_max;
  const float c = 0.5f * (1.0f + (float)std::cos(pi * (double)tt / (double)t_max));
  return eta_min + (lr0 - eta_min) * c;
}

material_adam::material_adam(const coef_energy_net &model, float grad_clip)
    : grad_clip_(grad_clip), m_(model.zero_grads()), u_(model.zero_grads()),
      names_(model.param_names()) {}

void material_adam::step(coef_energy_net &model, const coef_energy_net::param_grads &grad,
                         float lr) {
  ++t_;
  // global gradient norm over all weight tensors
  double sq = 0.0;
  for (const auto &nm : names_)
    for (float g : grad.at(nm))
      sq += (double)g * g;
  const float norm = (float)std::sqrt(sq);
  const float gscale = (grad_clip_ > 0.0f && norm > grad_clip_) ? grad_clip_ / norm : 1.0f;
  const float bc1 = 1.0f - std::pow(b1_, (float)t_);
  const float bc2 = 1.0f - std::pow(b2_, (float)t_);
  for (const auto &nm : names_) {
    std::vector<float> &w = model.mutable_param(nm);
    const std::vector<float> &gv = grad.at(nm);
    std::vector<float> &mv = m_.at(nm);
    std::vector<float> &uv = u_.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i) {
      const float g = gv[i] * gscale;
      mv[i] = b1_ * mv[i] + (1.0f - b1_) * g;
      uv[i] = b2_ * uv[i] + (1.0f - b2_) * g * g;
      const float mhat = mv[i] / bc1, uhat = uv[i] / bc2;
      w[i] -= lr * mhat / (std::sqrt(uhat) + eps_);
    }
  }
}

#ifndef CVC_USING_CUDA
// CPU-only build: material_train.cu supplies the real probe when there is one
// (CVC_USING_CUDA, not CVC_ENABLE_CUDA — see the note in drive.cpp).
bool material_train_cuda_available() { return false; }
#endif

} // namespace nav
} // namespace cvc

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_train_test — the P5 RELEASE GATE: a finite-difference gradcheck of
// the FULL material training loss w.r.t. the CoefEnergyNetMaterial weights. This
// exercises the whole chain — model forward -> surrogate rollout -> loss, then
// loss grads -> integrate_surrogate_material_vjp -> coef_energy_net::backward_one
// -> weight grads — against a torch-independent central finite difference.
//
// The CVaR quantile is detached (a per-step constant): material_loss_and_grad
// returns the base-point eta, and the finite difference passes it back as
// material_loss's frozen_eta, so the FD sees the SAME detached quantile the
// analytic gradient used (otherwise the recomputed-quantile term the source
// never differentiates would corrupt the check near the tail threshold).
//
// The default config's w_multi = 0.5 puts L_multi (the separate geometry
// rollout, material_train.h) inside the finite-differenced loss too. The sampled
// variants turn on GRL-SNAM #113's random starts at a FIXED seed: both the loss
// and its gradient then score the same draw, so the FD stays a deterministic
// function of the weights.

#include "coef_energy_test_model.h"
#include "material_batch_fixture.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using cvc::nav::coef_energy_net;
using cvc::nav::material_batch;
using cvc::nav::material_loss;
using cvc::nav::material_loss_and_grad;
using cvc::nav::material_loss_config;

namespace {

using cvc_test::Batch;
using cvc_test::make_batch;

void full_chain_gradcheck(const material_loss_config &cfg, const char *tag) {
  std::mt19937 rng(2024);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();

  // analytic gradient (fresh CVaR eta, returned for the FD to freeze)
  coef_energy_net::param_grads grads = m.zero_grads();
  float eta = 0.0f;
  const double L0 = material_loss_and_grad(m, mb, cfg, grads, &eta);
  ASSERT_TRUE(std::isfinite(L0));

  const std::vector<std::string> names = m.param_names();
  double gnorm2 = 0.0;
  for (const auto &nm : names)
    for (float x : grads.at(nm))
      gnorm2 += (double)x * x;
  const double gnorm = std::sqrt(gnorm2);
  ASSERT_GT(gnorm, 1e-3);

  const float eps = 1e-3f;
  auto floss = [&]() { return material_loss(m, mb, cfg, eta); }; // frozen eta

  // (1) directional FD along the full analytic gradient
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double Lp = floss();
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] -= 2.0f * eps * (float)(g[i] / gnorm);
  }
  const double Lm = floss();
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double dd_fd = (Lp - Lm) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gnorm) / (std::fabs(dd_fd) + gnorm + 1e-9);

  // (2) per-tensor spot check (large-gradient params only, where FD is reliable)
  double worst = 0.0;
  std::string worst_nm;
  int checked = 0;
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    int taken = 0;
    for (std::size_t i = 0; i < w.size() && taken < 5; ++i) {
      if (std::fabs(g[i]) < 5e-2)
        continue;
      const float orig = w[i];
      w[i] = orig + eps;
      const double lp = floss();
      w[i] = orig - eps;
      const double lm = floss();
      w[i] = orig;
      const double fd = (lp - lm) / (2.0 * eps);
      const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-4);
      if (rel > worst) {
        worst = rel;
        worst_nm = nm;
      }
      ++taken;
      ++checked;
    }
  }

  std::printf(
      "[material-train-e2e %s] L=%.4f |g|=%.3f dir_rel=%.3e checked=%d worst_rel=%.3e (%s)\n", tag,
      L0, gnorm, dir_rel, checked, worst, worst_nm.c_str());
  EXPECT_LT(dir_rel, 2e-2) << "full loss->weights gradient fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a weight gradient disagrees with finite differences (" << worst_nm
                         << ")";
  EXPECT_GE(checked, 30);
}

material_loss_config sampled_multi(bool semi_implicit) {
  material_loss_config cfg;
  cfg.multi.ms_count = 4;
  cfg.multi.frac_lo = 0.8;
  cfg.multi.frac_hi = 0.98;
  cfg.multi.seed = 5;
  cfg.multi.semi_implicit = semi_implicit;
  return cfg;
}

TEST(NavMaterialTrain, FullChainGradcheck) {
  full_chain_gradcheck(material_loss_config{}, "legacy"); // train_material.py defaults
}

TEST(NavMaterialTrain, FullChainGradcheckSampledMultiStart) {
  full_chain_gradcheck(sampled_multi(false), "sampled explicit");
}

TEST(NavMaterialTrain, FullChainGradcheckSampledSemiImplicitMultiStart) {
  full_chain_gradcheck(sampled_multi(true), "sampled semi");
}

// The sampled starts reach the trainer's loss: same weights and batch, a
// different L_multi (hence L) than the legacy single start and than another
// seed, while the loss and the gradient entry points agree on the value.
TEST(NavMaterialTrain, SampledMultiStartReachesTheLoss) {
  std::mt19937 rng(2024);
  const int P = 16;
  const coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  const material_loss_config legacy, s5 = sampled_multi(false);
  material_loss_config s6 = s5;
  s6.multi.seed = 6;
  coef_energy_net::param_grads grads = m.zero_grads();
  float eta = 0.0f;
  const double L5 = material_loss_and_grad(m, mb, s5, grads, &eta);
  const double l5 = material_loss(m, mb, s5, eta), l_legacy = material_loss(m, mb, legacy, eta),
               l6 = material_loss(m, mb, s6, eta);
  // The two entry points differ by the float-rounded CVaR quantile alone
  // (~1e-9 relative); a different draw moves L by orders of magnitude more.
  const double tol = 1e-7 * std::fabs(l5);
  std::printf("[material-train-ms] L5=%.12f l5=%.12f legacy=%.12f seed6=%.12f\n", L5, l5, l_legacy,
              l6);
  EXPECT_NEAR(L5, l5, tol) << "loss and grad scored different draws";
  EXPECT_GT(std::fabs(l5 - l_legacy), 100.0 * tol) << "sampling did not reach the loss";
  EXPECT_GT(std::fabs(l5 - l6), 100.0 * tol) << "the seed did not change the draw";
}

} // namespace

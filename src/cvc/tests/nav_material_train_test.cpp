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
// L_multi is out of scope here (a separate geometry rollout — see
// material_train.h); this gates the material-path loss.

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

TEST(NavMaterialTrain, FullChainGradcheck) {
  std::mt19937 rng(2024);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  material_loss_config cfg; // train_material.py defaults

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

  std::printf("[material-train-e2e] L=%.4f |g|=%.3f dir_rel=%.3e checked=%d worst_rel=%.3e (%s)\n",
              L0, gnorm, dir_rel, checked, worst, worst_nm.c_str());
  EXPECT_LT(dir_rel, 2e-2) << "full loss->weights gradient fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a weight gradient disagrees with finite differences (" << worst_nm
                         << ")";
  EXPECT_GE(checked, 30);
}

} // namespace

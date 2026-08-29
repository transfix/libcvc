/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_train_cuda_test — the device-resident material training step
// (material_loss_and_grad with use_cuda=true) must match the CPU path. use_cuda
// routes the four heavy ops (model forward/backward + surrogate rollout
// forward/VJP) through their device twins while the loss / seed-grad / multi-start
// glue stays on the CPU; this pins the end-to-end loss and weight gradients to the
// all-CPU path. FLOAT tier — the model and rollout are float-equivalent on device,
// so grads are compared by cosine + a loose rel, and the loss by rel. Auto-skips
// without a device.

#include "coef_energy_test_model.h"
#include "material_batch_fixture.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using namespace cvc::nav;

#ifndef CVC_ENABLE_CUDA
TEST(NavMaterialTrainCuda, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

TEST(NavMaterialTrainCuda, LossAndGradMatchesCpu) {
  if (!material_rollout_cuda_available() || !coef_energy_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  std::mt19937 rng(7);
  const int P = 16;
  coef_energy_net model = cvc_test::build_test_model(rng, P);
  cvc_test::Batch batch = cvc_test::make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  material_loss_config cfg; // defaults

  auto gc = model.zero_grads();
  float eta_c = 0.0f;
  const double Lc = material_loss_and_grad(model, mb, cfg, gc, &eta_c, /*use_cuda=*/false);
  auto gd = model.zero_grads();
  float eta_d = 0.0f;
  const double Ld = material_loss_and_grad(model, mb, cfg, gd, &eta_d, /*use_cuda=*/true);

  const double Lrel = std::fabs(Lc - Ld) / (std::fabs(Lc) + 1e-6);
  std::printf("[material-train-cuda] L_cpu=%.6f L_gpu=%.6f rel=%.3e  eta_c=%.4f eta_d=%.4f\n", Lc,
              Ld, Lrel, eta_c, eta_d);
  EXPECT_LT(Lrel, 2e-3) << "loss";

  float worst_rel = 0.0f, worst_cos = 1.0f;
  std::string wr, wc;
  for (const auto &name : model.param_names()) {
    const auto &a = gc.at(name);
    const auto &b = gd.at(name);
    double dot = 0, na = 0, nb = 0;
    float maxrel = 0.0f;
    for (std::size_t i = 0; i < a.size(); ++i) {
      dot += (double)a[i] * b[i];
      na += (double)a[i] * a[i];
      nb += (double)b[i] * b[i];
      maxrel = std::max(maxrel, std::fabs(a[i] - b[i]) / (std::fabs(a[i]) + 1e-3f));
    }
    const double cos = (na > 0 && nb > 0) ? dot / std::sqrt(na * nb) : 1.0;
    if (maxrel > worst_rel) {
      worst_rel = maxrel;
      wr = name;
    }
    if (cos < worst_cos) {
      worst_cos = cos;
      wc = name;
    }
    EXPECT_GT(cos, 0.999) << name << " cos";
  }
  std::printf("[material-train-cuda] grads worst_rel=%.3e (%s)  worst_cos=%.6f (%s)\n", worst_rel,
              wr.c_str(), worst_cos, wc.c_str());
  EXPECT_LT(worst_rel, 5e-2) << wr;
}

#endif // CVC_ENABLE_CUDA

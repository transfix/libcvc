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
#include <cvc/nav/material.h>
#include <cvc/nav/material_train.h>
#include <gtest/gtest.h>
#include <limits>
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

// use_cuda is documented as always safe. The device rollout VJP stores each
// agent's trajectory in per-thread local memory and REJECTS max(H) above
// material_rollout_cuda_max_horizon() — it used to let that throw escape all the
// way out of material_loss_and_grad. It must fall back to the host adjoint
// instead, and still agree with the all-CPU path.
TEST(NavMaterialTrainCuda, LongHorizonFallsBackInsteadOfThrowing) {
  if (!material_rollout_cuda_available() || !coef_energy_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  const int cap = material_rollout_cuda_max_horizon();
  ASSERT_GT(cap, 0) << "a CUDA build must report a real horizon cap";

  std::mt19937 rng(11);
  const int P = 16;
  coef_energy_net model = cvc_test::build_test_model(rng, P);
  cvc_test::Batch batch = cvc_test::make_batch(rng, /*B=*/4, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  // Push one agent past the device cap; the batch's max(H) is what gates.
  batch.H[0] = cap + 1;
  const material_batch mb = batch.view();
  material_loss_config cfg;

  auto gc = model.zero_grads();
  const double Lc = material_loss_and_grad(model, mb, cfg, gc, nullptr, /*use_cuda=*/false);
  auto gd = model.zero_grads();
  double Ld = 0.0;
  ASSERT_NO_THROW(Ld = material_loss_and_grad(model, mb, cfg, gd, nullptr, /*use_cuda=*/true))
      << "use_cuda must degrade to the host adjoint past the horizon cap, not throw";

  const double Lrel = std::fabs(Lc - Ld) / (std::fabs(Lc) + 1e-6);
  std::printf("[material-train-cuda] long-horizon H=%d (cap %d) L_cpu=%.6f L_gpu=%.6f rel=%.3e\n",
              batch.H[0], cap, Lc, Ld, Lrel);
  EXPECT_LT(Lrel, 2e-3);
  for (const auto &name : model.param_names()) {
    const auto &a = gc.at(name);
    const auto &b = gd.at(name);
    double dot = 0, na = 0, nb = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      dot += (double)a[i] * b[i];
      na += (double)a[i] * a[i];
      nb += (double)b[i] * b[i];
    }
    if (na > 0 && nb > 0)
      EXPECT_GT(dot / std::sqrt(na * nb), 0.999) << name;
  }
}

// The forward-only loss is the validation entry point; its device routing must
// agree with the CPU, and it must leave the weights untouched.
TEST(NavMaterialTrainCuda, ForwardOnlyLossMatchesCpu) {
  if (!material_rollout_cuda_available() || !coef_energy_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  std::mt19937 rng(5);
  const int P = 16;
  coef_energy_net model = cvc_test::build_test_model(rng, P);
  cvc_test::Batch batch = cvc_test::make_batch(rng, /*B=*/12, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  material_loss_config cfg;

  const std::vector<float> before = model.param("fuser.layers.0.linear1.weight");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const double Lc = material_loss(model, mb, cfg, nan, /*use_cuda=*/false);
  const double Ld = material_loss(model, mb, cfg, nan, /*use_cuda=*/true);
  const double rel = std::fabs(Lc - Ld) / (std::fabs(Lc) + 1e-6);
  std::printf("[material-loss-cuda] L_cpu=%.6f L_gpu=%.6f rel=%.3e\n", Lc, Ld, rel);
  EXPECT_LT(rel, 2e-3);
  // forward-only: no optimizer, no weight motion
  EXPECT_EQ(before, model.param("fuser.layers.0.linear1.weight"));
}

#endif // CVC_ENABLE_CUDA

// Builds WITHOUT CUDA must still LINK the three availability probes (they are
// declared unconditionally in the headers, so a binding can call them without a
// CVC_USING_CUDA guard) and must report no device.
TEST(NavMaterialTrainCuda, AvailabilityProbesAlwaysLink) {
  const bool rollout = cvc::nav::material_rollout_cuda_available();
  const bool model = cvc::nav::coef_energy_cuda_available();
  const bool train = cvc::nav::material_train_cuda_available();
  const int cap = cvc::nav::material_rollout_cuda_max_horizon();
  std::printf("[material-cuda-probes] rollout=%d model=%d train=%d max_horizon=%d\n", (int)rollout,
              (int)model, (int)train, cap);
#ifndef CVC_ENABLE_CUDA
  EXPECT_FALSE(rollout);
  EXPECT_FALSE(model);
  EXPECT_FALSE(train);
  EXPECT_EQ(cap, 0);
#else
  // With CUDA compiled in, the cap is the kernel's real bound whether or not a
  // device is present at runtime.
  EXPECT_GT(cap, 0);
#endif
}

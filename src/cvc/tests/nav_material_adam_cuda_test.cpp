/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_adam_cuda_test.cpp — the CUDA-resident optimizer
// (material_adam_cuda) must match the CPU material_adam over a multi-step grad
// sequence: same initial weights, same grads, same lr each step, then compare the
// trained weights. Two regimes: grads small enough that the global-norm clip never
// fires (then the Adam update is per-element identical → tight parity), and grads
// large enough that it fires every step (then the only divergence is the norm
// reduction order, device float vs CPU double → the FLOAT tier). Auto-skips
// without a device.

#include "coef_energy_test_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace cvc::nav;

#ifndef CVC_ENABLE_CUDA
TEST(NavMaterialAdamCuda, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

namespace {
// Fill a fresh param_grads (all model tensors) with N(0,scale) values.
coef_energy_net::param_grads random_grads(const coef_energy_net &model, std::mt19937 &rng,
                                          float scale) {
  auto g = model.zero_grads();
  std::normal_distribution<float> nd(0.0f, scale);
  for (auto &kv : g)
    for (auto &x : kv.second)
      x = nd(rng);
  return g;
}

// Run K steps of CPU material_adam and device material_adam_cuda from the same
// model + same grad/lr sequence; return (worst_rel, worst_cos) over all weights.
void compare_adam(const coef_energy_net &base, unsigned seed, int steps, float grad_scale,
                  float clip, const char *label) {
  coef_energy_net m_cpu = base, m_gpu = base;
  material_adam opt_cpu(m_cpu, clip);
  material_adam_cuda opt_gpu(m_gpu, clip);

  std::mt19937 rng(seed);
  for (int s = 0; s < steps; ++s) {
    auto g = random_grads(base, rng, grad_scale);
    const float lr = cosine_lr(3e-4f, 3e-5f, s, steps);
    opt_cpu.step(m_cpu, g, lr);
    opt_gpu.step(g, lr);
  }
  opt_gpu.sync_to(m_gpu);

  float worst_rel = 0.0f, worst_cos = 1.0f;
  std::string wr, wc;
  for (const auto &name : base.param_names()) {
    const auto &a = m_cpu.param(name);
    const auto &b = m_gpu.param(name);
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
  }
  std::printf("[material-adam-cuda %-9s] steps=%d worst_rel=%.3e (%s) worst_cos=%.6f (%s)\n", label,
              steps, worst_rel, wr.c_str(), worst_cos, wc.c_str());
  EXPECT_LT(worst_rel, 2e-3f) << label;
  EXPECT_GT(worst_cos, 0.9999) << label;
}
} // namespace

TEST(NavMaterialAdamCuda, MatchesCpu) {
  if (!material_train_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  std::mt19937 mrng(4242);
  coef_energy_net model = cvc_test::build_test_model(mrng, /*P=*/16);

  // No-clip regime: grads well under the clip norm → Adam update per-element
  // identical to the CPU, tight parity.
  compare_adam(model, 11, 30, 1e-3f, 5.0f, "noclip");
  // Clip-active regime: large grads → the global-norm clip fires every step, so the
  // gscale depends on the (float, device) norm — the FLOAT tier.
  compare_adam(model, 22, 30, 5e-1f, 5.0f, "clip");
  // A short run and a longer run to check the moment accumulation over time.
  compare_adam(model, 33, 5, 1e-2f, 5.0f, "short");
  compare_adam(model, 44, 80, 1e-2f, 5.0f, "long");
}

#endif // CVC_ENABLE_CUDA

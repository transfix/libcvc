/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_coef_energy_cuda_test.cpp — the CUDA CoefEnergyNetMaterial forward
// (coef_energy_net::forward_batch_cuda) must be FLOAT-equivalent to the CPU
// forward_batch over a ragged batch (agents with 0..k obstacles, some masked).
// Auto-skips without a device. The numpy-vs-CPU parity is covered cross-language
// (GRL-SNAM test_matnet_parity); here we pin CUDA == CPU on this box.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cvc/nav/coef_energy_net.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

#include "coef_energy_test_model.h"

using namespace cvc::nav;

#ifndef CVC_ENABLE_CUDA
TEST(NavCoefEnergyCuda, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

namespace {
float worst_rel(const std::vector<float> &a, const std::vector<float> &b) {
  float w = 0.0f;
  for (std::size_t i = 0; i < a.size(); ++i)
    w = std::max(w, std::fabs(a[i] - b[i]) / (std::fabs(a[i]) + 1e-3f));
  return w;
}
} // namespace

TEST(NavCoefEnergyCuda, ForwardMatchesCpu) {
  if (!coef_energy_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  std::mt19937 rng(123);
  const int P = 16;
  coef_energy_net net = cvc_test::build_test_model(rng, P);

  const int n = 6;
  const int nobs[n] = {0, 1, 3, 5, 2, 4};
  std::vector<int> off(n + 1, 0);
  for (int i = 0; i < n; ++i)
    off[i + 1] = off[i] + nobs[i];
  const int total = off[n];

  std::uniform_real_distribution<float> ur(-1.0f, 1.0f), up(0.0f, 1.0f);
  std::vector<float> obs_feats(std::max(total, 1) * 6), goal_feats(n * 4),
      risk_patch((std::size_t)n * 2 * P * P);
  std::vector<std::uint8_t> obs_mask(std::max(total, 1), 1);
  for (auto &v : obs_feats)
    v = ur(rng);
  for (auto &v : goal_feats)
    v = ur(rng);
  for (auto &v : risk_patch)
    v = up(rng);
  // mask the last obstacle of the even agents (exercises the key-padding path).
  for (int i = 0; i < n; ++i)
    if (nobs[i] > 0 && i % 2 == 0)
      obs_mask[off[i + 1] - 1] = 0;

  auto run = [&](bool cuda, std::vector<float> &al, std::vector<float> &be, std::vector<float> &ga,
                 std::vector<float> &ls, std::vector<float> &lh, std::vector<float> &ml) {
    al.assign(std::max(total, 1), -1.0f);
    be.assign(n, 0);
    ga.assign(n, 0);
    ls.assign(n, 0);
    lh.assign(n, 0);
    ml.assign(n, 0);
    if (cuda)
      net.forward_batch_cuda(obs_feats.data(), obs_mask.data(), off.data(), n, goal_feats.data(),
                             risk_patch.data(), P, al.data(), be.data(), ga.data(), ls.data(),
                             lh.data(), ml.data());
    else
      net.forward_batch(obs_feats.data(), obs_mask.data(), off.data(), n, goal_feats.data(),
                        risk_patch.data(), P, al.data(), be.data(), ga.data(), ls.data(), lh.data(),
                        ml.data(), 0);
  };

  std::vector<float> al_c, be_c, ga_c, ls_c, lh_c, ml_c, al_g, be_g, ga_g, ls_g, lh_g, ml_g;
  run(false, al_c, be_c, ga_c, ls_c, lh_c, ml_c);
  run(true, al_g, be_g, ga_g, ls_g, lh_g, ml_g);

  const float ra = worst_rel(al_c, al_g), rb = worst_rel(be_c, be_g), rg = worst_rel(ga_c, ga_g);
  const float rls = worst_rel(ls_c, ls_g), rlh = worst_rel(lh_c, lh_g), rml = worst_rel(ml_c, ml_g);
  std::printf("[coef-energy-cuda] worst_rel alpha=%.3e beta=%.3e gamma=%.3e lamS=%.3e lamH=%.3e "
              "mu=%.3e\n",
              ra, rb, rg, rls, rlh, rml);
  EXPECT_LT(ra, 3e-3f) << "alpha";
  EXPECT_LT(rb, 3e-3f) << "beta";
  EXPECT_LT(rg, 3e-3f) << "gamma";
  EXPECT_LT(rls, 3e-3f) << "lam_soft";
  EXPECT_LT(rlh, 3e-3f) << "lam_hard";
  EXPECT_LT(rml, 3e-3f) << "mu_lat";
}

// Device backward (accumulated over the batch) == CPU backward_one looped over the
// batch, per weight tensor (rel + cosine). The atomicAdd sum order differs from
// the CPU's sequential accumulate, so this is the FLOAT tier, not bit-exact.
//
// Swept over several batch shapes because agent T = 1 + n_obs drives the per-agent
// scratch layout: a no-obstacle agent (T=1) and a fully key-padded agent (only the
// goal token is attendable) are distinct edge cases from the generic ragged batch.
namespace {
// mask_mode: 0 = all valid, 1 = mask each even agent's last obstacle,
// 2 = fully mask agent 0's obstacles (every key but the goal token padded).
void check_backward(coef_energy_net &net, int P, unsigned seed, const std::vector<int> &nobs,
                    int mask_mode, const char *label) {
  const int n = static_cast<int>(nobs.size());
  std::vector<int> off(n + 1, 0);
  for (int i = 0; i < n; ++i)
    off[i + 1] = off[i] + nobs[i];
  const int total = off[n];

  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> ur(-1.0f, 1.0f), up(0.0f, 1.0f);
  std::vector<float> obs_feats(std::max(total, 1) * 6), goal_feats(n * 4),
      risk_patch((std::size_t)n * 2 * P * P);
  std::vector<std::uint8_t> obs_mask(std::max(total, 1), 1);
  for (auto &v : obs_feats)
    v = ur(rng);
  for (auto &v : goal_feats)
    v = ur(rng);
  for (auto &v : risk_patch)
    v = up(rng);
  if (mask_mode == 1) {
    for (int i = 0; i < n; ++i)
      if (nobs[i] > 0 && i % 2 == 0)
        obs_mask[off[i + 1] - 1] = 0;
  } else if (mask_mode == 2) {
    for (int j = off[0]; j < off[1]; ++j) // agent 0 fully masked
      obs_mask[j] = 0;
  }
  std::vector<float> ga(std::max(total, 1)), gb(n), gg(n), gls(n), glh(n), gml(n);
  for (auto *v : {&ga, &gb, &gg, &gls, &glh, &gml})
    for (auto &x : *v)
      x = ur(rng);

  auto gc = net.zero_grads();
  for (int i = 0; i < n; ++i) {
    const int o = off[i], nob = off[i + 1] - o;
    net.backward_one(obs_feats.data() + (std::size_t)o * 6, obs_mask.data() + o, nob,
                     goal_feats.data() + (std::size_t)i * 4,
                     risk_patch.data() + (std::size_t)i * 2 * P * P, P, ga.data() + o, gb[i], gg[i],
                     gls[i], glh[i], gml[i], gc);
  }
  auto gd = net.zero_grads();
  net.backward_batch_cuda(obs_feats.data(), obs_mask.data(), off.data(), n, goal_feats.data(),
                          risk_patch.data(), P, ga.data(), gb.data(), gg.data(), gls.data(),
                          glh.data(), gml.data(), gd);

  float worst_rel = 0.0f, worst_cos = 1.0f;
  std::string wr_name, wc_name;
  for (const auto &name : net.param_names()) {
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
      wr_name = name;
    }
    if (cos < worst_cos) {
      worst_cos = cos;
      wc_name = name;
    }
    EXPECT_LT(maxrel, 5e-3f) << label << " / " << name << " rel";
    EXPECT_GT(cos, 0.9999) << label << " / " << name << " cos";
  }
  std::printf("[coef-energy-cuda-bwd %-10s] worst_rel=%.3e (%s)  worst_cos=%.6f (%s)\n", label,
              worst_rel, wr_name.c_str(), worst_cos, wc_name.c_str());
}
} // namespace

TEST(NavCoefEnergyCuda, BackwardMatchesCpu) {
  if (!coef_energy_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  const int P = 16;
  std::mt19937 mrng(321);
  coef_energy_net net = cvc_test::build_test_model(mrng, P);

  check_backward(net, P, 321, {0, 1, 3, 5, 2, 4}, 1, "ragged");   // generic + a T=1 agent
  check_backward(net, P, 777, {0, 0, 0}, 0, "all-t1");            // every agent no-obstacle
  check_backward(net, P, 909, {3, 2}, 2, "all-masked");           // agent 0's keys all padded
  check_backward(net, P, 101, {6, 1, 0, 4}, 1, "wide");           // larger max_obs + singleton + T=1
  check_backward(net, P, 55, {1}, 0, "single");                   // single-agent, single obstacle
}

#endif // CVC_ENABLE_CUDA

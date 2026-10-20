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
#include <cstdlib>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
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

// with_ext puts an external force in every rollout the loss scores (material_batch::ext): a
// smooth force patch per agent, about 0.3, so the weights' gradient also runs through its J.
void full_chain_gradcheck(const material_loss_config &cfg, const char *tag, bool with_ext = false) {
  std::mt19937 rng(2024);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  material_batch mb = batch.view();
  std::vector<float> fpatch;
  cvc::nav::material_ext ext;
  if (with_ext) {
    const int Hp = batch.Hp, Wp = batch.Wp;
    fpatch.assign(static_cast<std::size_t>(batch.B) * 2 * Hp * Wp, 0.0f);
    for (int i = 0; i < batch.B; ++i)
      for (int r = 0; r < Hp; ++r)
        for (int c = 0; c < Wp; ++c) {
          const float x = (float)c / (Wp - 1), y = (float)r / (Hp - 1), ph = 0.3f * i;
          const std::size_t base = (std::size_t)i * 2 * Hp * Wp + (std::size_t)r * Wp + c;
          fpatch[base] = 0.3f * std::sin(5.0f * x + 2.0f * y + ph);
          fpatch[base + (std::size_t)Hp * Wp] = 0.3f * std::cos(3.0f * x - 4.0f * y + ph);
        }
    ext.patch = fpatch.data();
    mb.ext = &ext;
  }

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

// The full loss -> weights gradient with the loss taken through the kinematic
// bicycle (P8b) instead of the point-mass surrogate. Vehicle limits sized for the
// fixture's scale (positions within a few units, dt = 0.1).
TEST(NavMaterialTrain, FullChainGradcheckBicycle) {
  material_loss_config cfg;
  cfg.integrator = cvc::nav::material_integrator::bicycle;
  cfg.bicycle.L = 0.3f;
  cfg.bicycle.a_max = 2.0f;
  cfg.bicycle.a_lat_max = 1.5f;
  cfg.bicycle.vmax = 1.5f;
  full_chain_gradcheck(cfg, "bicycle");
}

// The same chain through the dynamic vehicle (P8b): the bicycle's controller and limits,
// a unit-mass vehicle of its wheelbase with grip (mu g 8.8) above them.
TEST(NavMaterialTrain, FullChainGradcheckDynamic) {
  material_loss_config cfg;
  cfg.integrator = cvc::nav::material_integrator::dynamic;
  cfg.bicycle.L = 0.3f;
  cfg.bicycle.a_max = 2.0f;
  cfg.bicycle.a_lat_max = 1.5f;
  cfg.bicycle.vmax = 1.5f;
  cvc::nav::diff::dyn_bike_params &d = cfg.dynamic;
  d.m = 1.0f;
  d.lf = d.lr = 0.15f;
  d.Iz = d.m * d.lf * d.lr;
  d.h = 0.05f;
  d.g = 9.81f;
  d.mu = 0.9f;
  d.B_f = 10.0f;
  d.B_r = 12.0f;
  d.tau_steer = 0.05f;
  d.tau_drive = 0.05f;
  d.eps = 0.05f;
  d.v_blend = 0.1f;
  d.v_width = 0.03f;
  full_chain_gradcheck(cfg, "dynamic");
}

// The same chains with an external force in the rollouts (material_batch::ext, a force patch per
// agent): the material heads train in the presence of the force they face at deploy.
TEST(NavMaterialTrain, FullChainGradcheckWithAnExternalForce) {
  full_chain_gradcheck(material_loss_config{}, "surrogate + ext", true);
}

TEST(NavMaterialTrain, FullChainGradcheckBicycleWithAnExternalForce) {
  material_loss_config cfg;
  cfg.integrator = cvc::nav::material_integrator::bicycle;
  cfg.bicycle.L = 0.3f;
  cfg.bicycle.a_max = 2.0f;
  cfg.bicycle.a_lat_max = 1.5f;
  cfg.bicycle.vmax = 1.5f;
  full_chain_gradcheck(cfg, "bicycle + ext", true);
}

// The loss's terms (material_loss_parts) add up to the loss both entry points return.
TEST(NavMaterialTrain, TheLossPartsAddUpToTheLoss) {
  std::mt19937 rng(2024);
  const int P = 16;
  const coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  const material_loss_config cfg;
  cvc::nav::material_loss_parts pg, pl;
  coef_energy_net::param_grads grads = m.zero_grads();
  float eta = 0.0f;
  const double L = material_loss_and_grad(m, mb, cfg, grads, &eta, false, &pg);
  const double l = material_loss(m, mb, cfg, eta, false, &pl);
  const auto sum = [&](const cvc::nav::material_loss_parts &q) {
    return 0.3 * cfg.w_traj * q.traj + 0.3 * cfg.w_vel * q.vel + cfg.w_fric * q.fric +
           cfg.w_clear * q.clear + q.nav + cfg.w_lreg * q.lreg + cfg.w_multi * q.multi;
  };
  EXPECT_EQ(pg.L, L);
  EXPECT_NEAR(sum(pg), L, 1e-12 * std::fabs(L));
  EXPECT_EQ(pl.L, l);
  EXPECT_NEAR(sum(pl), l, 1e-12 * std::fabs(l));
  EXPECT_NEAR(pg.eta, eta, 1e-6 * std::fabs(pg.eta));
}

// The native trainer's golden for GRL-SNAM's torch material trainer (a torch twin of this loss,
// held to it): with CVC_MATERIAL_TRAIN_GOLDEN_DIR set, writes into that directory
//   material_train_golden.json: the config, the batch (the 17 arrays), an external force (two
//     force patches per agent, the odd agents switching at step 1), the parameter index, and
//     each step's loss, CVaR eta and terms over 3 Adam steps plus the loss after them;
//   material_train_golden_w0.cvcnm / _w3.cvcnm: the weights before and after the 3 steps;
//   material_train_golden_grad0.f32: step 0's weight gradients, little-endian float32,
//     concatenated in the index's order.
// Not committed: the blobs are 2.5 MB. Without the variable this only runs the trainer.
TEST(NavMaterialTrain, TorchTrainerGolden) {
  std::mt19937 rng(2024);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  material_batch mb = batch.view();
  const int B = batch.B, Hp = batch.Hp, Wp = batch.Wp, K = 2;
  int steps = 0;
  for (int h : batch.H)
    steps = std::max(steps, h);
  std::vector<float> fpatch(static_cast<std::size_t>(B) * K * 2 * Hp * Wp);
  std::vector<int> fsched(static_cast<std::size_t>(B) * steps);
  for (int i = 0; i < B; ++i) {
    for (int k = 0; k < K; ++k)
      for (int r = 0; r < Hp; ++r)
        for (int c = 0; c < Wp; ++c) {
          const float x = (float)c / (Wp - 1), y = (float)r / (Hp - 1), ph = 0.7f * k + 0.3f * i;
          const std::size_t base = ((std::size_t)i * K + k) * 2 * Hp * Wp + (std::size_t)r * Wp + c;
          fpatch[base] = 0.3f * std::sin(5.0f * x + 2.0f * y + ph);
          fpatch[base + (std::size_t)Hp * Wp] = 0.3f * std::cos(3.0f * x - 4.0f * y + ph);
        }
    for (int s = 0; s < steps; ++s)
      fsched[(std::size_t)i * steps + s] = (s >= 1 && i % 2 == 1) ? 1 : 0;
  }
  cvc::nav::material_ext ext;
  ext.patch = fpatch.data();
  ext.K = K;
  ext.patch_of_step = fsched.data();
  ext.steps = steps;
  mb.ext = &ext;

  const material_loss_config cfg; // the defaults: surrogate, w_multi 0.5, the legacy start
  const float lr = 1e-3f, grad_clip = 5.0f;
  const char *dir = std::getenv("CVC_MATERIAL_TRAIN_GOLDEN_DIR");
  namespace fs = std::filesystem;
  if (dir)
    m.save((fs::path(dir) / "material_train_golden_w0.cvcnm").string());
  cvc::nav::material_adam opt(m, grad_clip);
  std::vector<cvc::nav::material_loss_parts> parts(4);
  coef_energy_net::param_grads g0;
  for (int t = 0; t < 4; ++t) {
    coef_energy_net::param_grads grads = m.zero_grads();
    float eta = 0.0f;
    material_loss_and_grad(m, mb, cfg, grads, &eta, false, &parts[t]);
    ASSERT_TRUE(std::isfinite(parts[t].L));
    if (t == 0)
      g0 = grads;
    if (t < 3)
      opt.step(m, grads, lr);
  }
  std::printf("[material-train-golden] loss %.6f -> %.6f over 3 steps (eta %.4f)\n", parts[0].L,
              parts[3].L, parts[0].eta);
  EXPECT_LT(parts[3].L, parts[0].L);
  if (!dir)
    return;
  m.save((fs::path(dir) / "material_train_golden_w3.cvcnm").string());
  const auto num = [](double x) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", x);
    return std::string(buf);
  };
  const auto arr = [&](const auto &v) {
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i)
      out += (i ? ", " : "") + num((double)v[i]);
    return out + "]";
  };
  std::ofstream bin((fs::path(dir) / "material_train_golden_grad0.f32").string(), std::ios::binary);
  std::ostringstream js;
  js << "{\n \"format\": \"cvc-material-train-golden-1\",\n";
  js << " \"config\": {\"integrator\": \"surrogate\", \"w_traj\": " << num(cfg.w_traj)
     << ", \"w_vel\": " << num(cfg.w_vel) << ", \"w_fric\": " << num(cfg.w_fric)
     << ", \"w_clear\": " << num(cfg.w_clear) << ", \"w_lreg\": " << num(cfg.w_lreg)
     << ", \"w_goal\": " << num(cfg.w_goal) << ", \"w_len\": " << num(cfg.w_len)
     << ", \"w_risk\": " << num(cfg.w_risk) << ", \"w_hard\": " << num(cfg.w_hard)
     << ", \"cvar_alpha\": " << num(cfg.cvar_alpha) << ", \"w_multi\": " << num(cfg.w_multi)
     << ", \"lam_soft_max\": " << num(cfg.lam_soft_max)
     << ", \"lam_hard_max\": " << num(cfg.lam_hard_max)
     << ", \"margin_factor\": " << num(cfg.rollout.margin_factor)
     << ", \"mass\": " << num(cfg.rollout.mass) << ", \"d_hat_sdf\": " << num(cfg.rollout.d_hat_sdf)
     << ", \"k_sharp\": " << num(cfg.rollout.k_sharp) << ", \"tau\": " << num(cfg.multi.tau)
     << ", \"ms_h\": " << cfg.multi.ms_h << ", \"ms_dt_mult\": " << num(cfg.multi.ms_dt_mult)
     << ", \"ms_count\": " << cfg.multi.ms_count << ", \"frac_lo\": " << num(cfg.multi.frac_lo)
     << ", \"frac_hi\": " << num(cfg.multi.frac_hi)
     << ", \"semi_implicit\": " << (cfg.multi.semi_implicit ? 1 : 0)
     << ", \"grad_clip\": " << num(grad_clip) << ", \"lr\": " << num(lr) << "},\n";
  js << " \"batch\": {\"B\": " << B << ", \"N\": " << batch.N << ", \"P\": " << batch.P
     << ", \"Hp\": " << Hp << ", \"Wp\": " << Wp << ",\n  \"obs_feats\": " << arr(batch.obs_feats)
     << ",\n  \"obs_mask\": " << arr(batch.obs_mask)
     << ",\n  \"goal_feats\": " << arr(batch.goal_feats)
     << ",\n  \"risk_patch\": " << arr(batch.risk_patch) << ",\n  \"o0\": " << arr(batch.o0)
     << ",\n  \"v0\": " << arr(batch.v0) << ",\n  \"goal\": " << arr(batch.goal)
     << ",\n  \"C\": " << arr(batch.C) << ",\n  \"R\": " << arr(batch.R)
     << ",\n  \"rollout_patch\": " << arr(batch.rollout_patch) << ",\n  \"rr\": " << arr(batch.rr)
     << ",\n  \"d_hat\": " << arr(batch.d_hat) << ",\n  \"dt\": " << arr(batch.dt)
     << ",\n  \"H\": " << arr(batch.H) << ",\n  \"o_tgt\": " << arr(batch.o_tgt)
     << ",\n  \"v_tgt\": " << arr(batch.v_tgt) << ",\n  \"gamma_o\": " << arr(batch.gamma_o)
     << "},\n";
  js << " \"ext\": {\"K\": " << K << ", \"steps\": " << steps
     << ", \"steer\": 1, \"patch\": " << arr(fpatch) << ",\n  \"patch_of_step\": " << arr(fsched)
     << "},\n";
  js << " \"params\": [\n";
  long offset = 0;
  const std::vector<std::string> names = m.param_names();
  for (std::size_t k = 0; k < names.size(); ++k) {
    const std::vector<float> &g = g0.at(names[k]);
    bin.write(reinterpret_cast<const char *>(g.data()),
              (std::streamsize)(g.size() * sizeof(float)));
    js << "  {\"name\": \"" << names[k] << "\", \"offset\": " << offset
       << ", \"count\": " << g.size() << "}" << (k + 1 < names.size() ? "," : "") << "\n";
    offset += (long)g.size();
  }
  js << " ],\n \"steps\": [\n";
  for (int t = 0; t < 4; ++t) {
    const cvc::nav::material_loss_parts &q = parts[t];
    js << "  {\"step\": " << t << ", \"L\": " << num(q.L) << ", \"eta\": " << num(q.eta)
       << ", \"traj\": " << num(q.traj) << ", \"vel\": " << num(q.vel)
       << ", \"fric\": " << num(q.fric) << ", \"clear\": " << num(q.clear)
       << ", \"nav\": " << num(q.nav) << ", \"lreg\": " << num(q.lreg)
       << ", \"multi\": " << num(q.multi) << "}" << (t < 3 ? "," : "") << "\n";
  }
  js << " ]\n}\n";
  std::ofstream((fs::path(dir) / "material_train_golden.json").string(), std::ios::binary)
      << js.str();
  std::printf("  wrote the golden to %s (%ld gradient values)\n", dir, offset);
}

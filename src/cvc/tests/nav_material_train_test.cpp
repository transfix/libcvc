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

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/belief_material.h>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
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
// belief takes the 3-channel net ([r~, hard, seen] patches, a seen channel of mixed bits) with
// the training gate on every other agent (material_batch::gate_active).
void full_chain_gradcheck(const material_loss_config &cfg, const char *tag, bool with_ext = false,
                          bool belief = false) {
  std::mt19937 rng(2024);
  const int P = 16;
  cvc_test::test_model_dims dims;
  dims.risk_channels = belief ? 3 : 2;
  coef_energy_net m = cvc_test::build_test_model(rng, P, dims);
  Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  std::vector<std::uint8_t> gate;
  if (belief) {
    const std::size_t pp = static_cast<std::size_t>(P) * P;
    std::vector<float> rp(static_cast<std::size_t>(batch.B) * 3 * pp);
    for (int i = 0; i < batch.B; ++i)
      for (std::size_t k = 0; k < pp; ++k) {
        const bool seen = (k + i) % 3 != 0;
        rp[(i * 3 + 0) * pp + k] = seen ? batch.risk_patch[(i * 2 + 0) * pp + k] : 0.0f;
        rp[(i * 3 + 1) * pp + k] = seen ? batch.risk_patch[(i * 2 + 1) * pp + k] : 0.0f;
        rp[(i * 3 + 2) * pp + k] = seen ? 1.0f : 0.0f;
      }
    batch.risk_patch = std::move(rp);
    gate.resize(batch.B);
    for (int i = 0; i < batch.B; ++i)
      gate[i] = static_cast<std::uint8_t>(i % 2 == 0);
  }
  material_batch mb = batch.view();
  if (belief)
    mb.gate_active = gate.data();
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

// The same chain through a vehicle simulation (material_integrator::jolt), here the four-wheel
// model as its own plant (dyn4_rollout_plant): the plumbing of the Jolt rollout's tape and
// reverse, exact where its forward is the model (physics_jolt_rollout_test runs it on Jolt). The
// Humvee at the controller's wheelbase.
TEST(NavMaterialTrain, FullChainGradcheckOnAPlant) {
  material_loss_config cfg;
  cfg.integrator = cvc::nav::material_integrator::jolt;
  cfg.bicycle.L = 0.3f;
  cfg.bicycle.a_max = 2.0f;
  cfg.bicycle.a_lat_max = 1.5f;
  cfg.bicycle.vmax = 1.5f;
  cfg.dyn4 =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::humvee(), cfg.bicycle.L);
  cfg.plant = std::make_shared<cvc::nav::dyn4_rollout_plant>();
  full_chain_gradcheck(cfg, "plant");
}

// The same chains with an external force in the rollouts (material_batch::ext, a force patch per
// agent): the material heads train in the presence of the force they face at deploy.
// The belief net: 3 risk channels and the training gate, on the surrogate and the bicycle.
TEST(NavMaterialTrain, FullChainGradcheckBelief) {
  full_chain_gradcheck(material_loss_config{}, "belief", false, true);
}

TEST(NavMaterialTrain, FullChainGradcheckBeliefBicycle) {
  material_loss_config cfg;
  cfg.integrator = cvc::nav::material_integrator::bicycle;
  cfg.bicycle.L = 0.3f;
  cfg.bicycle.a_max = 2.0f;
  cfg.bicycle.a_lat_max = 1.5f;
  cfg.bicycle.vmax = 1.5f;
  full_chain_gradcheck(cfg, "belief bicycle", false, true);
}

// The training gate: a gated-off agent's lam_soft reaches neither the rollout nor (through it)
// the gradient, so its loss equals the loss with that agent's lam_soft head at 0 in the rollout,
// and with every agent gated on the loss is the ungated loss, byte for byte.
TEST(NavMaterialTrain, TheGateMultipliesLamSoftInTheRolloutOnly) {
  std::mt19937 rng(77);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, 8, 3, P, 13, 13);
  material_batch mb = batch.view();
  const material_loss_config cfg;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const double L_none = material_loss(m, mb, cfg, nan);
  std::vector<std::uint8_t> on(batch.B, 1), off(batch.B, 0);
  mb.gate_active = on.data();
  EXPECT_EQ(material_loss(m, mb, cfg, nan), L_none);
  coef_energy_net::param_grads g1 = m.zero_grads(), g2 = m.zero_grads();
  mb.gate_active = nullptr;
  const double La = material_loss_and_grad(m, mb, cfg, g1);
  mb.gate_active = on.data();
  const double Lb = material_loss_and_grad(m, mb, cfg, g2);
  EXPECT_EQ(La, Lb);
  for (const auto &nm : m.param_names())
    EXPECT_EQ(g1.at(nm), g2.at(nm)) << nm;
  mb.gate_active = off.data();
  const double L_off = material_loss(m, mb, cfg, nan);
  EXPECT_NE(L_off, L_none) << "the gate did not reach the rollout";
  EXPECT_TRUE(std::isfinite(L_off));
}

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
//   <prefix>.json: the config, the batch (the 17 arrays), an external force (two force patches
//     per agent, the odd agents switching at step 1), the parameter index, and each step's loss,
//     CVaR eta and terms over 3 Adam steps plus the loss after them;
//   <prefix>_w0.cvcnm / _w3.cvcnm: the weights before and after the 3 steps;
//   <prefix>_grad0.f32: step 0's weight gradients, little-endian float32, concatenated in the
//     index's order.
// Without the variable this only runs the trainer.
struct trainer_golden_case {
  const char *prefix;
  unsigned seed;
  int P, B, N, Hp, Wp;
  cvc_test::test_model_dims dims;
  // The belief net (golden 3 of docs/roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md): 3 risk channels,
  // each agent's patch cropped by material_patch_belief from golden 1's full planes (the
  // defaults, last tick, plane 0), and the training gate on agents 0 and 2.
  bool belief = false;
};

// The crop golden 3's patches come from: golden 1's full arrays, a 12 m patch around each
// position (on golden 1's [0, 31]^2 grid; the third reaches off the map).
constexpr double kBeliefExtent = 12.0;
const double kBeliefPos[4][2] = {{11.9, 7.4}, {24.1, 20.1}, {3.0, 29.5}, {20.0, 2.0}};

static std::vector<float> belief_crops(int B, int P) {
  namespace fs = std::filesystem;
  const fs::path g1 = fs::path(__FILE__).parent_path() / "data" / "belief_material_golden.json";
  boost::property_tree::ptree t;
  boost::property_tree::read_json(g1.string(), t);
  const int rows = t.get<int>("grid.rows"), cols = t.get<int>("grid.cols");
  std::vector<double> bnd;
  for (const auto &v : t.get_child("grid.bounds"))
    bnd.push_back(v.second.get_value<double>());
  auto plane = [&](const char *key) {
    std::vector<double> out;
    for (const auto &v : t.get_child(std::string("full.") + key))
      out.push_back(v.second.get_value<double>());
    return out;
  };
  const std::vector<double> rk = plane("risk"), hb = plane("hard_b"), sn = plane("material_seen");
  std::vector<float> risk(rk.begin(), rk.end());
  std::vector<std::uint8_t> hard(hb.size()), seen(sn.size());
  for (std::size_t k = 0; k < hb.size(); ++k) {
    hard[k] = hb[k] != 0.0;
    seen[k] = sn[k] != 0.0;
  }
  const std::size_t per = static_cast<std::size_t>(3) * P * P;
  std::vector<float> out(static_cast<std::size_t>(B) * per);
  for (int i = 0; i < B; ++i)
    cvc::nav::material_patch_belief(risk.data(), hard.data(), seen.data(), rows, cols, bnd[0],
                                    bnd[1], bnd[2], bnd[3], kBeliefPos[i % 4][0],
                                    kBeliefPos[i % 4][1], kBeliefExtent, P,
                                    cvc::nav::belief_material_params{}, out.data() + i * per);
  return out;
}

static void run_trainer_golden(const trainer_golden_case &gc) {
  std::mt19937 rng(gc.seed);
  const int P = gc.P;
  cvc_test::test_model_dims dims = gc.dims;
  if (gc.belief)
    dims.risk_channels = 3;
  coef_energy_net m = cvc_test::build_test_model(rng, P, dims);
  Batch batch = make_batch(rng, gc.B, gc.N, P, gc.Hp, gc.Wp);
  std::vector<std::uint8_t> gate;
  if (gc.belief) {
    batch.risk_patch = belief_crops(gc.B, P);
    gate.resize(gc.B);
    for (int i = 0; i < gc.B; ++i)
      gate[i] = static_cast<std::uint8_t>(i % 2 == 0);
  }
  material_batch mb = batch.view();
  if (gc.belief)
    mb.gate_active = gate.data();
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
    m.save((fs::path(dir) / (std::string(gc.prefix) + "_w0.cvcnm")).string());
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
  std::printf("[%s] loss %.6f -> %.6f over 3 steps (eta %.4f)\n", gc.prefix, parts[0].L, parts[3].L,
              parts[0].eta);
  EXPECT_LT(parts[3].L, parts[0].L);
  if (!dir)
    return;
  m.save((fs::path(dir) / (std::string(gc.prefix) + "_w3.cvcnm")).string());
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
  std::ofstream bin((fs::path(dir) / (std::string(gc.prefix) + "_grad0.f32")).string(),
                    std::ios::binary);
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
     << ",\n  \"v_tgt\": " << arr(batch.v_tgt) << ",\n  \"gamma_o\": " << arr(batch.gamma_o);
  if (gc.belief)
    js << ",\n  \"risk_channels\": 3, \"gate_active\": " << arr(gate);
  js << "},\n";
  if (gc.belief) {
    js << " \"crop\": {\"source\": \"belief_material_golden.json full (defaults, last tick, plane "
          "0)\", \"extent_m\": "
       << num(kBeliefExtent) << ", \"P\": " << P
       << ", \"unknown_risk\": 0, \"unknown_hard\": 0, \"positions\": [";
    for (int i = 0; i < gc.B; ++i)
      js << (i ? ", " : "") << "[" << num(kBeliefPos[i % 4][0]) << ", " << num(kBeliefPos[i % 4][1])
         << "]";
    js << "]},\n";
  }
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
  std::ofstream((fs::path(dir) / (std::string(gc.prefix) + ".json")).string(), std::ios::binary)
      << js.str();
  std::printf("  wrote %s to %s (%ld gradient values)\n", gc.prefix, dir, offset);
}

// The full-width golden (2.5 MB, not committed).
TEST(NavMaterialTrain, TorchTrainerGolden) {
  run_trainer_golden({"material_train_golden", 2024, 16, 16, 3, 13, 13, {}});
}

// The narrowest net (d_tok 16, 2 heads, 1 layer, d_risk 16) on a small batch: under 1 MB, so
// GRL-SNAM's CI can commit it and hold its torch trainer to it.
TEST(NavMaterialTrain, TorchTrainerGoldenSmall) {
  run_trainer_golden({"material_train_golden_small", 2025, 16, 4, 3, 9, 9, {16, 2, 1, 16}});
}

// Golden 3 (belief material): the small golden's widths and batch with the 3-channel net,
// patches cropped from golden 1 and the training gate. Its _w0/_w3 are .cvcnm version 2.
TEST(NavMaterialTrain, TorchTrainerGoldenBelief) {
  run_trainer_golden({"material_train_golden_belief", 2026, 16, 4, 3, 9, 9, {16, 2, 1, 16}, true});
}

// The width-generated parameter table is the fixed one at the default widths.
TEST(NavMaterialTrain, TestModelTableAtDefaultWidths) {
  const auto &fixed = cvc_test::coef_energy_tensors();
  const auto gen = cvc_test::coef_energy_tensors(cvc_test::test_model_dims{});
  ASSERT_EQ(gen.size(), fixed.size());
  for (std::size_t i = 0; i < gen.size(); ++i) {
    EXPECT_STREQ(gen[i].name, fixed[i].name) << i;
    EXPECT_EQ(gen[i].dims, fixed[i].dims) << fixed[i].name;
  }
}

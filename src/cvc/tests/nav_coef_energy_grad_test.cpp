/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_coef_energy_grad_test — per-op finite-difference gradchecks for the
// CoefEnergyNetMaterial backward primitives (detail/nn_ops.h). These ops all
// fail SILENTLY (a mis-scaled softmax or a dropped LayerNorm mean term still
// runs and merely descends the wrong loss), so each forward+backward pair is
// validated op-by-op against central finite differences BEFORE the end-to-end
// model gradcheck — the torch-independent ground truth (same discipline as
// nav_coef_train_test / nav_material_rollout_grad_test).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/detail/nn_ops.h>
#include <functional>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace nn = cvc::nav::detail::nn;
using cvc::nav::coef_energy_net;

namespace {

// Each test seeds its OWN rng so results are independent of test order (a shared
// rng makes the finite-difference conditioning order-dependent and flaky).
float U(std::mt19937 &rng, float lo, float hi) {
  return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}
std::vector<float> randv(std::mt19937 &rng, int n, float lo = -1.0f, float hi = 1.0f) {
  std::vector<float> v(n);
  for (auto &x : v)
    x = U(rng, lo, hi);
  return v;
}

// Central-difference gradcheck of `loss` (a closure over the mutable `p`) against
// the analytic gradient `g`. Returns worst relative error over params clearing
// the noise floor; sets `checked`.
double gradcheck(std::vector<float> &p, const std::function<double()> &loss,
                 const std::vector<float> &g, int &checked, float eps = 1e-3f) {
  double worst = 0.0;
  checked = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    if (std::fabs(g[i]) < 1e-3)
      continue;
    const float orig = p[i];
    p[i] = orig + eps;
    const double Lp = loss();
    p[i] = orig - eps;
    const double Lm = loss();
    p[i] = orig;
    const double fd = (Lp - Lm) / (2.0 * eps);
    const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6);
    worst = std::max(worst, rel);
    ++checked;
  }
  return worst;
}

double dot(const std::vector<float> &a, const std::vector<float> &q) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
    s += (double)a[i] * q[i];
  return s;
}

TEST(NavCoefEnergyGrad, LinearBackward) {
  std::mt19937 rng(100 + 1);
  const int rows = 3, in = 5, out = 4;
  std::vector<float> x = randv(rng, rows * in), W = randv(rng, out * in), b = randv(rng, out);
  const std::vector<float> q = randv(rng, rows * out);
  auto fwd = [&](std::vector<float> &y) {
    nn::linear(x.data(), rows, in, W.data(), b.data(), out, y.data());
  };
  auto loss = [&]() {
    std::vector<float> y(rows * out);
    fwd(y);
    return dot(y, q);
  };
  std::vector<float> gx(rows * in, 0.f), gW(out * in, 0.f), gb(out, 0.f);
  nn::linear_bwd(x.data(), rows, in, W.data(), out, q.data(), gx.data(), gW.data(), gb.data());
  int ck;
  double wx = gradcheck(x, loss, gx, ck);
  double wW = gradcheck(W, loss, gW, ck);
  double wb = gradcheck(b, loss, gb, ck);
  std::printf("[linear] worst gx=%.2e gW=%.2e gb=%.2e\n", wx, wW, wb);
  EXPECT_LT(wx, 5e-3);
  EXPECT_LT(wW, 5e-3);
  EXPECT_LT(wb, 5e-3);
}

TEST(NavCoefEnergyGrad, LayerNormBackward) {
  std::mt19937 rng(200 + 2);
  const int rows = 3, d = 8;
  const float eps = 1e-5f;
  std::vector<float> x = randv(rng, rows * d), g = randv(rng, d, 0.5f, 1.5f), b = randv(rng, d);
  const std::vector<float> q = randv(rng, rows * d);
  auto loss = [&]() {
    std::vector<float> y(rows * d);
    nn::layernorm(x.data(), rows, d, g.data(), b.data(), eps, y.data());
    return dot(y, q);
  };
  std::vector<float> gx(rows * d, 0.f), gg(d, 0.f), gb(d, 0.f);
  nn::layernorm_bwd(x.data(), rows, d, g.data(), eps, q.data(), gx.data(), gg.data(), gb.data());
  int ck;
  double wx = gradcheck(x, loss, gx, ck);
  double wg = gradcheck(g, loss, gg, ck);
  double wb = gradcheck(b, loss, gb, ck);
  std::printf("[layernorm] worst gx=%.2e gg=%.2e gb=%.2e\n", wx, wg, wb);
  EXPECT_LT(wx, 2e-2);
  EXPECT_LT(wg, 5e-3);
  EXPECT_LT(wb, 5e-3);
}

TEST(NavCoefEnergyGrad, MhaBackwardMasked) {
  std::mt19937 rng(300 + 3);
  const int T = 5, d = 8, nhead = 4;
  std::vector<std::uint8_t> pad = {0, 0, 1, 0, 1}; // two padded keys
  std::vector<float> x = randv(rng, T * d), w_in = randv(rng, 3 * d * d, -0.3f, 0.3f),
                     b_in = randv(rng, 3 * d, -0.2f, 0.2f), w_out = randv(rng, d * d, -0.5f, 0.5f),
                     b_out = randv(rng, d, -0.2f, 0.2f);
  const std::vector<float> q = randv(rng, T * d);
  auto loss = [&]() {
    std::vector<float> y(T * d);
    nn::mha(x.data(), T, d, nhead, pad.data(), w_in.data(), b_in.data(), w_out.data(), b_out.data(),
            y.data());
    return dot(y, q);
  };
  std::vector<float> gx(T * d, 0.f), gwi(3 * d * d, 0.f), gbi(3 * d, 0.f), gwo(d * d, 0.f),
      gbo(d, 0.f);
  nn::mha_bwd(x.data(), T, d, nhead, pad.data(), w_in.data(), b_in.data(), w_out.data(), q.data(),
              gx.data(), gwi.data(), gbi.data(), gwo.data(), gbo.data());
  int ck;
  double wx = gradcheck(x, loss, gx, ck);
  double wwi = gradcheck(w_in, loss, gwi, ck);
  double wbi = gradcheck(b_in, loss, gbi, ck);
  double wwo = gradcheck(w_out, loss, gwo, ck);
  double wbo = gradcheck(b_out, loss, gbo, ck);
  std::printf("[mha] worst gx=%.2e gw_in=%.2e gb_in=%.2e gw_out=%.2e gb_out=%.2e\n", wx, wwi, wbi,
              wwo, wbo);
  EXPECT_LT(wx, 3e-2);
  EXPECT_LT(wwi, 3e-2);
  EXPECT_LT(wbi, 3e-2);
  EXPECT_LT(wwo, 2e-2);
  EXPECT_LT(wbo, 2e-2);
}

TEST(NavCoefEnergyGrad, Conv2dBackward) {
  std::mt19937 rng(400 + 4);
  const int Cin = 2, H = 7, W = 7, Cout = 3, kH = 3, kW = 3, stride = 2, pad = 1;
  std::vector<float> x = randv(rng, Cin * H * W), w = randv(rng, Cout * Cin * kH * kW, -0.5f, 0.5f),
                     b = randv(rng, Cout, -0.2f, 0.2f);
  int Ho, Wo;
  {
    std::vector<float> probe;
    nn::conv2d(x.data(), Cin, H, W, w.data(), b.data(), Cout, kH, kW, stride, pad, probe, Ho, Wo);
  }
  const std::vector<float> q = randv(rng, Cout * Ho * Wo);
  auto loss = [&]() {
    std::vector<float> out;
    int ho, wo;
    nn::conv2d(x.data(), Cin, H, W, w.data(), b.data(), Cout, kH, kW, stride, pad, out, ho, wo);
    return dot(out, q);
  };
  std::vector<float> gx(Cin * H * W, 0.f), gw(Cout * Cin * kH * kW, 0.f), gb(Cout, 0.f);
  nn::conv2d_bwd(x.data(), Cin, H, W, w.data(), Cout, kH, kW, stride, pad, Ho, Wo, q.data(),
                 gx.data(), gw.data(), gb.data());
  int ck;
  double wx = gradcheck(x, loss, gx, ck);
  double ww = gradcheck(w, loss, gw, ck);
  double wb = gradcheck(b, loss, gb, ck);
  std::printf("[conv2d] worst gx=%.2e gw=%.2e gb=%.2e (Ho=%d Wo=%d)\n", wx, ww, wb, Ho, Wo);
  EXPECT_LT(wx, 5e-3);
  EXPECT_LT(ww, 5e-3);
  EXPECT_LT(wb, 5e-3);
}

// ── in-memory model builder (the CoefEnergyNetMaterial parameter table) ──────
struct TShape {
  const char *name;
  std::vector<int> dims;
};
const std::vector<TShape> kTensors = {
    {"goal_enc.0.weight", {64, 4}},
    {"goal_enc.0.bias", {64}},
    {"goal_enc.2.weight", {64, 64}},
    {"goal_enc.2.bias", {64}},
    {"obs_enc.0.weight", {128, 6}},
    {"obs_enc.0.bias", {128}},
    {"obs_enc.2.weight", {64, 128}},
    {"obs_enc.2.bias", {64}},
    {"fuser.layers.0.self_attn.in_proj_weight", {192, 64}},
    {"fuser.layers.0.self_attn.in_proj_bias", {192}},
    {"fuser.layers.0.self_attn.out_proj.weight", {64, 64}},
    {"fuser.layers.0.self_attn.out_proj.bias", {64}},
    {"fuser.layers.0.linear1.weight", {128, 64}},
    {"fuser.layers.0.linear1.bias", {128}},
    {"fuser.layers.0.linear2.weight", {64, 128}},
    {"fuser.layers.0.linear2.bias", {64}},
    {"fuser.layers.0.norm1.weight", {64}},
    {"fuser.layers.0.norm1.bias", {64}},
    {"fuser.layers.0.norm2.weight", {64}},
    {"fuser.layers.0.norm2.bias", {64}},
    {"fuser.layers.1.self_attn.in_proj_weight", {192, 64}},
    {"fuser.layers.1.self_attn.in_proj_bias", {192}},
    {"fuser.layers.1.self_attn.out_proj.weight", {64, 64}},
    {"fuser.layers.1.self_attn.out_proj.bias", {64}},
    {"fuser.layers.1.linear1.weight", {128, 64}},
    {"fuser.layers.1.linear1.bias", {128}},
    {"fuser.layers.1.linear2.weight", {64, 128}},
    {"fuser.layers.1.linear2.bias", {64}},
    {"fuser.layers.1.norm1.weight", {64}},
    {"fuser.layers.1.norm1.bias", {64}},
    {"fuser.layers.1.norm2.weight", {64}},
    {"fuser.layers.1.norm2.bias", {64}},
    {"alpha_head.0.weight", {64, 64}},
    {"alpha_head.0.bias", {64}},
    {"alpha_head.2.weight", {1, 64}},
    {"alpha_head.2.bias", {1}},
    {"beta_head.0.weight", {64, 64}},
    {"beta_head.0.bias", {64}},
    {"beta_head.2.weight", {1, 64}},
    {"beta_head.2.bias", {1}},
    {"gamma_head.0.weight", {64, 64}},
    {"gamma_head.0.bias", {64}},
    {"gamma_head.2.weight", {1, 64}},
    {"gamma_head.2.bias", {1}},
    {"risk_enc.net.0.weight", {16, 2, 3, 3}},
    {"risk_enc.net.0.bias", {16}},
    {"risk_enc.net.2.weight", {32, 16, 3, 3}},
    {"risk_enc.net.2.bias", {32}},
    {"risk_enc.net.4.weight", {64, 32, 3, 3}},
    {"risk_enc.net.4.bias", {64}},
    {"risk_enc.net.8.weight", {64, 1024}},
    {"risk_enc.net.8.bias", {64}},
    {"lam_soft_head.0.weight", {64, 128}},
    {"lam_soft_head.0.bias", {64}},
    {"lam_soft_head.2.weight", {1, 64}},
    {"lam_soft_head.2.bias", {1}},
    {"lam_hard_head.0.weight", {64, 128}},
    {"lam_hard_head.0.bias", {64}},
    {"lam_hard_head.2.weight", {1, 64}},
    {"lam_hard_head.2.bias", {1}},
    {"mu_lat_head.0.weight", {64, 128}},
    {"mu_lat_head.0.bias", {64}},
    {"mu_lat_head.2.weight", {1, 64}},
    {"mu_lat_head.2.bias", {1}},
};

void put32(std::vector<unsigned char> &b, std::uint32_t v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 4);
}
void put64(std::vector<unsigned char> &b, std::uint64_t v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 8);
}
void putf(std::vector<unsigned char> &b, float v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 4);
}

// Build a model with small random weights (norm weights ~1) at patch_size P.
coef_energy_net build_model(std::mt19937 &rng, int P) {
  std::vector<unsigned char> b;
  b.insert(b.end(), {'C', 'V', 'N', 'M'});
  put32(b, 1);
  put64(b, 0xABCDu);
  put32(b, 64);
  put32(b, 4);
  put32(b, 2);
  put32(b, 64);
  put32(b, static_cast<std::uint32_t>(P));
  putf(b, 5.0f);
  putf(b, 10.0f);
  putf(b, 5.0f);
  putf(b, 1e-5f);
  put32(b, static_cast<std::uint32_t>(kTensors.size()));
  for (const auto &ts : kTensors) {
    const std::string name = ts.name;
    put32(b, static_cast<std::uint32_t>(name.size()));
    b.insert(b.end(), name.begin(), name.end());
    put32(b, static_cast<std::uint32_t>(ts.dims.size()));
    std::size_t count = 1;
    for (int dv : ts.dims) {
      put32(b, static_cast<std::uint32_t>(dv));
      count *= static_cast<std::size_t>(dv);
    }
    const bool is_norm_w =
        name.find("norm") != std::string::npos && name.find("weight") != std::string::npos;
    for (std::size_t i = 0; i < count; ++i)
      putf(b, is_norm_w ? U(rng, 0.8f, 1.2f) : U(rng, -0.15f, 0.15f));
  }
  put32(b, 0); // meta_len
  return coef_energy_net::load_from_memory(b.data(), b.size());
}

TEST(NavCoefEnergyGrad, ModelBackwardEndToEnd) {
  std::mt19937 rng(777);
  const int P = 16, n_obs = 4;
  coef_energy_net m = build_model(rng, P);

  // inputs (data — no grad)
  std::vector<float> obs_feats = randv(rng, n_obs * 6, -1.0f, 1.0f);
  std::vector<std::uint8_t> obs_mask = {1, 1, 0, 1}; // one padded obstacle
  std::vector<float> goal_feats = randv(rng, 4, -1.0f, 1.0f);
  std::vector<float> risk_patch = randv(rng, 2 * P * P, 0.0f, 1.0f);
  // loss weights on every output (mu_lat included to exercise its head too)
  std::vector<float> qa = randv(rng, n_obs, -1.0f, 1.0f);
  const float qb = U(rng, -1, 1), qg = U(rng, -1, 1), qls = U(rng, -1, 1), qlh = U(rng, -1, 1),
              qmu = U(rng, -1, 1);

  auto loss = [&]() {
    std::vector<float> al(n_obs, 0.f);
    float be, ga, ls, lh, mu;
    m.forward_one(obs_feats.data(), obs_mask.data(), n_obs, goal_feats.data(), risk_patch.data(), P,
                  al.data(), &be, &ga, &ls, &lh, &mu);
    double L = qb * be + qg * ga + qls * ls + qlh * lh + qmu * mu;
    for (int i = 0; i < n_obs; ++i)
      L += (double)qa[i] * al[i];
    return L;
  };

  // analytic weight grads
  coef_energy_net::param_grads grads = m.zero_grads();
  m.backward_one(obs_feats.data(), obs_mask.data(), n_obs, goal_feats.data(), risk_patch.data(), P,
                 qa.data(), qb, qg, qls, qlh, qmu, grads);

  // (1) directional FD along the full analytic gradient
  const std::vector<std::string> names = m.param_names();
  double gnorm2 = 0.0;
  for (const auto &nm : names)
    for (float x : grads.at(nm))
      gnorm2 += (double)x * x;
  const double gnorm = std::sqrt(gnorm2);
  ASSERT_GT(gnorm, 1e-3);
  const float eps = 1e-3f;
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double Lp = loss();
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] -= 2.0f * eps * (float)(g[i] / gnorm);
  }
  const double Lm = loss();
  for (const auto &nm : names) { // restore
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double dd_fd = (Lp - Lm) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gnorm) / (std::fabs(dd_fd) + gnorm + 1e-9);

  // (2) per-tensor spot check: the highest-grad params in each weight tensor
  // (large |g| => the central FD is well above its conditioning floor).
  double worst = 0.0;
  std::string worst_nm;
  float worst_g = 0.0f, worst_fd = 0.0f;
  int checked = 0;
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    int taken = 0;
    for (std::size_t i = 0; i < w.size() && taken < 5; ++i) {
      if (std::fabs(g[i]) < 5e-2) // only large-gradient params (FD well-conditioned)
        continue;
      const float orig = w[i];
      w[i] = orig + eps;
      const double lp = loss();
      w[i] = orig - eps;
      const double lm = loss();
      w[i] = orig;
      const double fd = (lp - lm) / (2.0 * eps);
      const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-4);
      if (rel > worst) {
        worst = rel;
        worst_nm = nm;
        worst_g = g[i];
        worst_fd = (float)fd;
      }
      ++taken;
      ++checked;
    }
  }
  std::printf("[model-e2e] |g|=%.3f dir_rel=%.3e checked=%d worst_rel=%.3e (%s g=%.4f fd=%.4f)\n",
              gnorm, dir_rel, checked, worst, worst_nm.c_str(), worst_g, worst_fd);
  EXPECT_LT(dir_rel, 2e-2) << "model backward fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a weight gradient disagrees with finite differences";
  EXPECT_GE(checked, 40);
}

TEST(NavCoefEnergyGrad, AdaptiveAvgPoolBackward) {
  std::mt19937 rng(500 + 5);
  const int C = 3, H = 9, W = 9, o = 4;
  std::vector<float> x = randv(rng, C * H * W);
  std::vector<float> outp;
  nn::adaptive_avg_pool(x, C, H, W, o, outp);
  const std::vector<float> q = randv(rng, (int)outp.size());
  auto loss = [&]() {
    std::vector<float> op;
    nn::adaptive_avg_pool(x, C, H, W, o, op);
    return dot(op, q);
  };
  std::vector<float> gx(C * H * W, 0.f);
  nn::adaptive_avg_pool_bwd(C, H, W, o, q.data(), gx.data());
  int ck;
  double wx = gradcheck(x, loss, gx, ck);
  std::printf("[adaptive_avg_pool] worst gx=%.2e\n", wx);
  EXPECT_LT(wx, 5e-3);
}

} // namespace

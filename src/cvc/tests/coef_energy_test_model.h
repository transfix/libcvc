/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coef_energy_test_model.h — test-only helper: build a CoefEnergyNetMaterial with
// small random weights in memory (the full 66-tensor parameter table), for the
// P5 backward / training gradchecks.

#ifndef CVC_NAV_TESTS_COEF_ENERGY_TEST_MODEL_H
#define CVC_NAV_TESTS_COEF_ENERGY_TEST_MODEL_H

#include <cstdint>
#include <cvc/nav/coef_energy_net.h>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace cvc_test {

struct TShape {
  const char *name;
  std::vector<int> dims;
};

inline const std::vector<TShape> &coef_energy_tensors() {
  static const std::vector<TShape> kTensors = {
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
  return kTensors;
}

// The header's widths (d_tok, nhead, num_layers, d_risk); the rest of the net is fixed.
struct test_model_dims {
  int d_tok = 64, nhead = 4, num_layers = 2, d_risk = 64;
  int risk_channels = 2; // 3: the belief net ([r~, hard, seen]), written as .cvcnm version 2
};

// The parameter table for any widths, in coef_energy_tensors()'s order (which is its value at the
// default widths).
inline std::vector<TShape> coef_energy_tensors(const test_model_dims &n) {
  // TShape holds a const char *: a deque never moves its strings, so the names stay valid.
  static std::deque<std::string> names;
  const auto nm = [](std::string s) {
    names.push_back(std::move(s));
    return names.back().c_str();
  };
  const int d = n.d_tok, dr = n.d_risk;
  std::vector<TShape> t = {{"goal_enc.0.weight", {64, 4}}, {"goal_enc.0.bias", {64}},
                           {"goal_enc.2.weight", {d, 64}}, {"goal_enc.2.bias", {d}},
                           {"obs_enc.0.weight", {128, 6}}, {"obs_enc.0.bias", {128}},
                           {"obs_enc.2.weight", {d, 128}}, {"obs_enc.2.bias", {d}}};
  for (int L = 0; L < n.num_layers; ++L) {
    const std::string pre = "fuser.layers." + std::to_string(L) + ".";
    t.push_back({nm(pre + "self_attn.in_proj_weight"), {3 * d, d}});
    t.push_back({nm(pre + "self_attn.in_proj_bias"), {3 * d}});
    t.push_back({nm(pre + "self_attn.out_proj.weight"), {d, d}});
    t.push_back({nm(pre + "self_attn.out_proj.bias"), {d}});
    t.push_back({nm(pre + "linear1.weight"), {128, d}});
    t.push_back({nm(pre + "linear1.bias"), {128}});
    t.push_back({nm(pre + "linear2.weight"), {d, 128}});
    t.push_back({nm(pre + "linear2.bias"), {d}});
    t.push_back({nm(pre + "norm1.weight"), {d}});
    t.push_back({nm(pre + "norm1.bias"), {d}});
    t.push_back({nm(pre + "norm2.weight"), {d}});
    t.push_back({nm(pre + "norm2.bias"), {d}});
  }
  const auto head = [&](const std::string &h, int in) {
    t.push_back({nm(h + ".0.weight"), {64, in}});
    t.push_back({nm(h + ".0.bias"), {64}});
    t.push_back({nm(h + ".2.weight"), {1, 64}});
    t.push_back({nm(h + ".2.bias"), {1}});
  };
  for (const char *h : {"alpha_head", "beta_head", "gamma_head"})
    head(h, d);
  t.push_back({"risk_enc.net.0.weight", {16, n.risk_channels, 3, 3}});
  t.push_back({"risk_enc.net.0.bias", {16}});
  t.push_back({"risk_enc.net.2.weight", {32, 16, 3, 3}});
  t.push_back({"risk_enc.net.2.bias", {32}});
  t.push_back({"risk_enc.net.4.weight", {64, 32, 3, 3}});
  t.push_back({"risk_enc.net.4.bias", {64}});
  t.push_back({"risk_enc.net.8.weight", {dr, 1024}});
  t.push_back({"risk_enc.net.8.bias", {dr}});
  for (const char *h : {"lam_soft_head", "lam_hard_head", "mu_lat_head"})
    head(h, dr + d);
  return t;
}

inline void tm_put32(std::vector<unsigned char> &b, std::uint32_t v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 4);
}
inline void tm_put64(std::vector<unsigned char> &b, std::uint64_t v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 8);
}
inline void tm_putf(std::vector<unsigned char> &b, float v) {
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&v);
  b.insert(b.end(), p, p + 4);
}

// Model with small random weights (norm weights ~1) at patch_size P and the given widths.
inline cvc::nav::coef_energy_net build_test_model(std::mt19937 &rng, int P,
                                                  const test_model_dims &n = {}) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto U = [&](float lo, float hi) { return lo + (hi - lo) * (float)u(rng); };
  std::vector<unsigned char> b;
  b.insert(b.end(), {'C', 'V', 'N', 'M'});
  tm_put32(b, n.risk_channels == 3 ? 2 : 1);
  tm_put64(b, 0xABCDu);
  tm_put32(b, static_cast<std::uint32_t>(n.d_tok));
  tm_put32(b, static_cast<std::uint32_t>(n.nhead));
  tm_put32(b, static_cast<std::uint32_t>(n.num_layers));
  tm_put32(b, static_cast<std::uint32_t>(n.d_risk));
  tm_put32(b, static_cast<std::uint32_t>(P));
  tm_putf(b, 5.0f);
  tm_putf(b, 10.0f);
  tm_putf(b, 5.0f);
  tm_putf(b, 1e-5f);
  if (n.risk_channels == 3) { // the belief block (the spec's defaults, a 12 m patch)
    tm_put32(b, 3);
    tm_put32(b, 0);
    tm_putf(b, 0.0f);
    tm_put32(b, 0);
    tm_put32(b, 1);
    tm_putf(b, 0.0f);
    tm_putf(b, 12.0f);
  }
  const test_model_dims def;
  const bool is_default = n.d_tok == def.d_tok && n.nhead == def.nhead &&
                          n.num_layers == def.num_layers && n.d_risk == def.d_risk &&
                          n.risk_channels == def.risk_channels;
  const std::vector<TShape> generated = is_default ? std::vector<TShape>{} : coef_energy_tensors(n);
  const std::vector<TShape> &kTensors = is_default ? coef_energy_tensors() : generated;
  tm_put32(b, static_cast<std::uint32_t>(kTensors.size()));
  for (const auto &ts : kTensors) {
    const std::string name = ts.name;
    tm_put32(b, static_cast<std::uint32_t>(name.size()));
    b.insert(b.end(), name.begin(), name.end());
    tm_put32(b, static_cast<std::uint32_t>(ts.dims.size()));
    std::size_t count = 1;
    for (int dv : ts.dims) {
      tm_put32(b, static_cast<std::uint32_t>(dv));
      count *= static_cast<std::size_t>(dv);
    }
    const bool is_norm_w =
        name.find("norm") != std::string::npos && name.find("weight") != std::string::npos;
    for (std::size_t i = 0; i < count; ++i)
      tm_putf(b, is_norm_w ? U(0.8f, 1.2f) : U(-0.15f, 0.15f));
  }
  tm_put32(b, 0); // meta_len
  return cvc::nav::coef_energy_net::load_from_memory(b.data(), b.size());
}

} // namespace cvc_test

#endif

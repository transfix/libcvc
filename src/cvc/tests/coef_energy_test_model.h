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

// Model with small random weights (norm weights ~1) at patch_size P.
inline cvc::nav::coef_energy_net build_test_model(std::mt19937 &rng, int P) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto U = [&](float lo, float hi) { return lo + (hi - lo) * (float)u(rng); };
  std::vector<unsigned char> b;
  b.insert(b.end(), {'C', 'V', 'N', 'M'});
  tm_put32(b, 1);
  tm_put64(b, 0xABCDu);
  tm_put32(b, 64);
  tm_put32(b, 4);
  tm_put32(b, 2);
  tm_put32(b, 64);
  tm_put32(b, static_cast<std::uint32_t>(P));
  tm_putf(b, 5.0f);
  tm_putf(b, 10.0f);
  tm_putf(b, 5.0f);
  tm_putf(b, 1e-5f);
  const auto &kTensors = coef_energy_tensors();
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

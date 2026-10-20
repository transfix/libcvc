/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_belief_material_net_test: the belief net's side of docs/roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md
// (B3). The 3-channel .cvcnm (version 2 with its belief block; version 1 still byte-identical with
// its meta; the loader's checks), a zero seen channel equal to the 2-channel net in the forward
// and backward, and material_patch_belief's crop. The trainer's side (the gate, the 3-channel
// gradcheck and golden 3) is in nav_material_train_test.

#include "coef_energy_test_model.h"

#include <cstdint>
#include <cstring>
#include <cvc/nav/belief_material.h>
#include <cvc/nav/coef_energy_net.h>
#include <gtest/gtest.h>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {
std::uint32_t u32_at(const std::vector<unsigned char> &b, std::size_t off) {
  std::uint32_t v;
  std::memcpy(&v, b.data() + off, 4);
  return v;
}
float f32_at(const std::vector<unsigned char> &b, std::size_t off) {
  float v;
  std::memcpy(&v, b.data() + off, 4);
  return v;
}
constexpr std::size_t kBeliefBlock = 4 + 4 + 8 + 5 * 4 + 4 * 4; // after magic..eps
cvc_test::test_model_dims narrow(int channels) {
  cvc_test::test_model_dims d{16, 2, 1, 16};
  d.risk_channels = channels;
  return d;
}
} // namespace

TEST(NavBeliefMaterialNet, Version1RoundTripsByteForByteWithItsMeta) {
  std::mt19937 rng(1);
  coef_energy_net m = cvc_test::build_test_model(rng, 8, narrow(2));
  EXPECT_EQ(m.risk_channels(), 2);
  m.set_meta("{\"trained_by\": \"a test\"}");
  const std::vector<unsigned char> a = m.serialize();
  EXPECT_EQ(u32_at(a, 4), 1u);
  const coef_energy_net back = coef_energy_net::load_from_memory(a.data(), a.size());
  EXPECT_EQ(back.meta(), "{\"trained_by\": \"a test\"}");
  EXPECT_EQ(back.serialize(), a);
}

TEST(NavBeliefMaterialNet, Version2CarriesTheBeliefBlock) {
  std::mt19937 rng(2);
  coef_energy_net m = cvc_test::build_test_model(rng, 8, narrow(3));
  EXPECT_EQ(m.risk_channels(), 3);
  cvcnm_belief_header h;
  h.risk_channels = 3;
  h.seen_rule = 1;
  h.unknown_risk = 0.25f;
  h.unknown_hard = 1;
  h.unknown_gate_hard = 0;
  h.material_range_m = 7.0f;
  h.patch_extent_m = 100.0f;
  m.set_belief_header(h);
  const std::vector<unsigned char> a = m.serialize();
  EXPECT_EQ(u32_at(a, 4), 2u);
  // The block's layout: <IIfIIff> right after eps (GRL-SNAM matnet_export.BeliefHeader).
  EXPECT_EQ(u32_at(a, kBeliefBlock), 3u);
  EXPECT_EQ(u32_at(a, kBeliefBlock + 4), 1u);
  EXPECT_EQ(f32_at(a, kBeliefBlock + 8), 0.25f);
  EXPECT_EQ(u32_at(a, kBeliefBlock + 12), 1u);
  EXPECT_EQ(u32_at(a, kBeliefBlock + 16), 0u);
  EXPECT_EQ(f32_at(a, kBeliefBlock + 20), 7.0f);
  EXPECT_EQ(f32_at(a, kBeliefBlock + 24), 100.0f);
  const coef_energy_net back = coef_energy_net::load_from_memory(a.data(), a.size());
  EXPECT_EQ(back.risk_channels(), 3);
  EXPECT_EQ(back.belief_header().seen_rule, 1u);
  EXPECT_EQ(back.belief_header().patch_extent_m, 100.0f);
  EXPECT_EQ(back.serialize(), a);
  cvcnm_belief_header two = h;
  two.risk_channels = 2;
  EXPECT_THROW(m.set_belief_header(two), std::invalid_argument);
}

TEST(NavBeliefMaterialNet, TheLoaderRefusesMismatchesAndTruncation) {
  std::mt19937 rng(3);
  const std::vector<unsigned char> v2 = cvc_test::build_test_model(rng, 8, narrow(3)).serialize();
  const std::vector<unsigned char> v1 = cvc_test::build_test_model(rng, 8, narrow(2)).serialize();
  auto load = [](std::vector<unsigned char> b) {
    return coef_energy_net::load_from_memory(b.data(), b.size());
  };
  EXPECT_NO_THROW(load(v1));
  EXPECT_NO_THROW(load(v2));
  // A 3-channel encoder claimed as version 1, and an unknown version.
  std::vector<unsigned char> bad = v2;
  bad.erase(bad.begin() + kBeliefBlock, bad.begin() + kBeliefBlock + 28);
  const std::uint32_t one = 1, nine = 9;
  std::memcpy(bad.data() + 4, &one, 4);
  EXPECT_THROW(load(bad), std::runtime_error);
  bad = v1;
  std::memcpy(bad.data() + 4, &nine, 4);
  EXPECT_THROW(load(bad), std::runtime_error);
  // A belief block whose risk_channels is not 3.
  bad = v2;
  const std::uint32_t two = 2;
  std::memcpy(bad.data() + kBeliefBlock, &two, 4);
  EXPECT_THROW(load(bad), std::runtime_error);
  // A d_tok that no longer matches the tensors (header d_tok at byte 16).
  bad = v1;
  const std::uint32_t d32 = 32;
  std::memcpy(bad.data() + 16, &d32, 4);
  EXPECT_THROW(load(bad), std::runtime_error);
  // Truncated anywhere.
  for (std::size_t cut :
       {std::size_t(6), std::size_t(30), kBeliefBlock + 10, v2.size() / 2, v2.size() - 3})
    EXPECT_THROW(load(std::vector<unsigned char>(v2.begin(), v2.begin() + cut)), std::runtime_error)
        << cut;
}

TEST(NavBeliefMaterialNet, AZeroSeenChannelIsTheTwoChannelNet) {
  // Copy a 2-channel net's weights into a 3-channel one with zero seen-channel weights: the
  // forward is then identical for any seen channel, and the shared weights' gradients agree.
  std::mt19937 rng(4);
  const int P = 8;
  coef_energy_net two = cvc_test::build_test_model(rng, P, narrow(2));
  coef_energy_net three = cvc_test::build_test_model(rng, P, narrow(3));
  for (const std::string &nm : two.param_names()) {
    if (nm == "risk_enc.net.0.weight")
      continue;
    three.mutable_param(nm) = two.param(nm);
  }
  const std::vector<float> &w2 = two.param("risk_enc.net.0.weight");     // [16,2,3,3]
  std::vector<float> &w3 = three.mutable_param("risk_enc.net.0.weight"); // [16,3,3,3]
  for (int o = 0; o < 16; ++o)
    for (int c = 0; c < 3; ++c)
      for (int k = 0; k < 9; ++k)
        w3[(o * 3 + c) * 9 + k] = c < 2 ? w2[(o * 2 + c) * 9 + k] : 0.0f;
  const int n_obs = 2;
  const float obs[n_obs * 6] = {0.3f,  -0.2f, 0.4f, 1.0f, 0.5f, 0.6f,
                                -0.4f, 0.1f,  0.3f, 1.0f, 1.1f, 0.2f};
  const std::uint8_t mask[n_obs] = {1, 1};
  const float goal[4] = {0.7f, 0.4f, 0.806f, 1.0f};
  std::vector<float> p2(2 * P * P), p3(3 * P * P);
  for (int k = 0; k < P * P; ++k) {
    p2[k] = p3[k] = 0.01f * (k % 37);
    p2[P * P + k] = p3[P * P + k] = (k % 5 == 0) ? 1.0f : 0.0f;
    p3[2 * P * P + k] = (k % 3) ? 1.0f : 0.0f;
  }
  float a2[n_obs], a3[n_obs], s2[5], s3[5];
  two.forward_one(obs, mask, n_obs, goal, p2.data(), P, a2, &s2[0], &s2[1], &s2[2], &s2[3], &s2[4]);
  three.forward_one(obs, mask, n_obs, goal, p3.data(), P, a3, &s3[0], &s3[1], &s3[2], &s3[3],
                    &s3[4]);
  for (int i = 0; i < n_obs; ++i)
    EXPECT_EQ(a2[i], a3[i]);
  for (int i = 0; i < 5; ++i)
    EXPECT_EQ(s2[i], s3[i]) << i;
  coef_energy_net::param_grads g2 = two.zero_grads(), g3 = three.zero_grads();
  const float ga[n_obs] = {0.3f, -0.7f};
  two.backward_one(obs, mask, n_obs, goal, p2.data(), P, ga, 0.5f, -0.2f, 0.9f, 0.4f, 0.0f, g2);
  three.backward_one(obs, mask, n_obs, goal, p3.data(), P, ga, 0.5f, -0.2f, 0.9f, 0.4f, 0.0f, g3);
  for (const std::string &nm : two.param_names()) {
    if (nm == "risk_enc.net.0.weight") {
      for (int o = 0; o < 16; ++o)
        for (int c = 0; c < 2; ++c)
          for (int k = 0; k < 9; ++k)
            EXPECT_EQ(g2.at(nm)[(o * 2 + c) * 9 + k], g3.at(nm)[(o * 3 + c) * 9 + k]);
      continue;
    }
    EXPECT_EQ(g2.at(nm), g3.at(nm)) << nm;
  }
}

TEST(NavBeliefMaterial, PatchCropSamplesTheBeliefPlanes) {
  // A 9 x 9 grid on [0, 8]^2 with r~ = 0.1 r + 0.01 c (bilinear is exact on it), the top-left
  // quarter unseen and one hard cell.
  const int R = 9, C = 9;
  std::vector<float> risk(R * C);
  std::vector<std::uint8_t> hard(R * C, 0), seen(R * C, 1);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) {
      risk[r * C + c] = static_cast<float>(0.1 * r + 0.01 * c);
      if (r >= 5 && c <= 3)
        seen[r * C + c] = 0;
    }
  hard[2 * C + 6] = 1;
  belief_material_params p;
  p.unknown_risk = 0.5f;
  p.unknown_hard = 1;
  const int P = 5;
  std::vector<float> out(3 * P * P);
  // Centred on (6, 2) with a 4 m extent: offsets -2, -1, 0, 1, 2 land on cell centres.
  material_patch_belief(risk.data(), hard.data(), seen.data(), R, C, 0, 0, 8, 8, 6.0, 2.0, 4.0, P,
                        p, out.data());
  const int pp = P * P;
  for (int i = 0; i < P; ++i)
    for (int j = 0; j < P; ++j) {
      const int r = 2 + i - 2, c = 6 + j - 2, o = i * P + j;
      EXPECT_EQ(out[2 * pp + o], 1.0f);
      EXPECT_FLOAT_EQ(out[o], risk[r * C + c]);
      EXPECT_EQ(out[pp + o], hard[r * C + c] ? 1.0f : 0.0f);
    }
  // Centred on (1, 7): the unseen corner and the map's edge give the pinned values, ch2 = 0.
  material_patch_belief(risk.data(), hard.data(), seen.data(), R, C, 0, 0, 8, 8, 1.0, 7.0, 4.0, P,
                        p, out.data());
  for (int i = 0; i < P; ++i)
    for (int j = 0; j < P; ++j) {
      const int o = i * P + j;
      const int r = 7 + i - 2, c = 1 + j - 2;
      const bool s = r >= 0 && r < R && c >= 0 && c < C && seen[r * C + c];
      EXPECT_EQ(out[2 * pp + o], s ? 1.0f : 0.0f) << i << "," << j;
      if (!s) {
        EXPECT_EQ(out[o], 0.5f);
        EXPECT_EQ(out[pp + o], 1.0f);
      }
    }
  // Between cells, ch0 is the bilinear value of the (linear) field.
  material_patch_belief(risk.data(), hard.data(), seen.data(), R, C, 0, 0, 8, 8, 5.25, 1.5, 2.0, 2,
                        p, out.data());
  EXPECT_NEAR(out[0], 0.1 * 0.5 + 0.01 * 4.25, 1e-6);
  EXPECT_NEAR(out[3], 0.1 * 2.5 + 0.01 * 6.25, 1e-6);
  EXPECT_THROW(material_patch_belief(risk.data(), hard.data(), seen.data(), R, C, 0, 0, 8, 8, 1, 1,
                                     4.0, 1, p, out.data()),
               std::invalid_argument);
  EXPECT_THROW(material_patch_belief(risk.data(), hard.data(), seen.data(), R, C, 0, 0, 8, 8, 1, 1,
                                     0.0, 3, p, out.data()),
               std::invalid_argument);
}

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_coef_energy_test.cpp — the torch-free learned material coefficient net
// (cvc::nav::coef_energy_net). The heavy FLOAT parity vs the numpy math-path
// reference lives cross-language in GRL-SNAM tests/test_matnet_parity.py (the
// full model is ~0.8 MB, too large to embed here). This suite writes a
// deterministic .cvcnm in-test and checks: the loader round-trips, the forward
// runs on N=0 and N>0 with outputs in the capped ranges and masked alphas
// zeroed, the forward is deterministic, and forward_batch == per-agent
// forward_one across thread counts.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coef_energy_net.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

struct TShape {
  const char *name;
  std::vector<int> dims;
};

// The definitive CoefEnergyNetMaterial parameter table (name, shape).
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

void wr_u32(std::FILE *f, std::uint32_t v) { std::fwrite(&v, 4, 1, f); }
void wr_u64(std::FILE *f, std::uint64_t v) { std::fwrite(&v, 8, 1, f); }
void wr_f32(std::FILE *f, float v) { std::fwrite(&v, 4, 1, f); }

// Deterministic small weights so the forward stays numerically sane.
float wval(std::size_t k) { return 0.02f * static_cast<float>((k % 13) - 6); }

std::string write_test_cvcnm() {
  std::string path = std::string(std::tmpnam(nullptr)) + ".cvcnm";
  std::FILE *f = std::fopen(path.c_str(), "wb");
  std::fwrite("CVNM", 1, 4, f);
  wr_u32(f, 1);             // version
  wr_u64(f, 0xABCDEF1234u); // arch_hash (opaque here)
  wr_u32(f, 64);
  wr_u32(f, 4);
  wr_u32(f, 2);
  wr_u32(f, 64);
  wr_u32(f, 32); // patch_size
  wr_f32(f, 5.0f);
  wr_f32(f, 10.0f);
  wr_f32(f, 5.0f);
  wr_f32(f, 1e-5f);
  wr_u32(f, static_cast<std::uint32_t>(kTensors.size()));
  std::size_t counter = 1;
  for (const auto &ts : kTensors) {
    wr_u32(f, static_cast<std::uint32_t>(std::strlen(ts.name)));
    std::fwrite(ts.name, 1, std::strlen(ts.name), f);
    wr_u32(f, static_cast<std::uint32_t>(ts.dims.size()));
    std::size_t count = 1;
    for (int d : ts.dims) {
      wr_u32(f, static_cast<std::uint32_t>(d));
      count *= static_cast<std::size_t>(d);
    }
    for (std::size_t i = 0; i < count; ++i)
      wr_f32(f, wval(counter++));
  }
  wr_u32(f, 0); // meta_len
  std::fclose(f);
  return path;
}

} // namespace

TEST(NavCoefEnergyNet, LoadsAndReportsHyperparams) {
  const std::string path = write_test_cvcnm();
  const coef_energy_net m = coef_energy_net::load(path);
  std::remove(path.c_str());
  EXPECT_EQ(m.patch_size(), 32);
  EXPECT_EQ(m.arch_hash(), 0xABCDEF1234u);
  EXPECT_FLOAT_EQ(m.lam_soft_max(), 5.0f);
  EXPECT_FLOAT_EQ(m.lam_hard_max(), 10.0f);
}

TEST(NavCoefEnergyNet, ForwardRangesAndMaskZeroing) {
  const std::string path = write_test_cvcnm();
  const coef_energy_net m = coef_energy_net::load(path);
  std::remove(path.c_str());

  std::vector<float> patch(2 * 32 * 32);
  for (std::size_t i = 0; i < patch.size(); ++i)
    patch[i] = 0.5f * std::sin(0.01f * static_cast<float>(i));
  const float goal[4] = {1.0f, -0.5f, 1.118f, 1.0f};

  // N = 0
  {
    float b, g, ls, lh, ml;
    m.forward_one(nullptr, nullptr, 0, goal, patch.data(), 32, nullptr, &b, &g, &ls, &lh, &ml);
    for (float v : {b, g, ls, lh, ml})
      EXPECT_TRUE(std::isfinite(v));
    EXPECT_GE(b, 0.0f);
    EXPECT_GE(g, 0.0f);
    EXPECT_GT(ls, 0.0f);
    EXPECT_LT(ls, 5.0f);
    EXPECT_GT(lh, 0.0f);
    EXPECT_LT(lh, 10.0f);
    EXPECT_GT(ml, 0.0f);
    EXPECT_LT(ml, 5.0f);
  }

  // N = 4 with a masked obstacle
  {
    const int N = 4;
    std::vector<float> obs(N * 6);
    for (std::size_t i = 0; i < obs.size(); ++i)
      obs[i] = 0.3f * static_cast<float>((static_cast<int>(i) % 5) - 2);
    std::vector<std::uint8_t> mask = {1, 1, 0, 1};
    std::vector<float> a(N);
    float b, g, ls, lh, ml;
    m.forward_one(obs.data(), mask.data(), N, goal, patch.data(), 32, a.data(), &b, &g, &ls, &lh,
                  &ml);
    EXPECT_FLOAT_EQ(a[2], 0.0f); // masked -> exactly zero
    for (int i = 0; i < N; ++i)
      EXPECT_GE(a[i], 0.0f);
    EXPECT_TRUE(std::isfinite(b) && std::isfinite(ls));
  }
}

TEST(NavCoefEnergyNet, DeterministicAndBatchMatchesSerial) {
  const std::string path = write_test_cvcnm();
  const coef_energy_net m = coef_energy_net::load(path);
  std::remove(path.c_str());

  const int n = 5;
  std::vector<int> offs = {0, 2, 2, 5, 8, 10}; // ragged incl. an N=0 agent
  const int total = offs[n];
  std::vector<float> obs(total * 6), goal(n * 4), patch(n * 2 * 32 * 32);
  std::vector<std::uint8_t> mask(total, 1);
  for (std::size_t i = 0; i < obs.size(); ++i)
    obs[i] = 0.1f * static_cast<float>((static_cast<int>(i) % 7) - 3);
  for (std::size_t i = 0; i < goal.size(); ++i)
    goal[i] = 0.2f * static_cast<float>((static_cast<int>(i) % 4) - 1);
  for (std::size_t i = 0; i < patch.size(); ++i)
    patch[i] = 0.4f * std::cos(0.003f * static_cast<float>(i));
  mask[3] = 0;

  auto run_batch = [&](int threads, std::vector<float> &a, std::vector<float> &b,
                       std::vector<float> &g, std::vector<float> &ls, std::vector<float> &lh,
                       std::vector<float> &ml) {
    a.assign(total, -1.0f);
    b.assign(n, 0);
    g.assign(n, 0);
    ls.assign(n, 0);
    lh.assign(n, 0);
    ml.assign(n, 0);
    m.forward_batch(obs.data(), mask.data(), offs.data(), n, goal.data(), patch.data(), 32,
                    a.data(), b.data(), g.data(), ls.data(), lh.data(), ml.data(), threads);
  };

  std::vector<float> a1, b1, g1, ls1, lh1, ml1, a8, b8, g8, ls8, lh8, ml8;
  run_batch(1, a1, b1, g1, ls1, lh1, ml1);
  run_batch(8, a8, b8, g8, ls8, lh8, ml8);
  EXPECT_EQ(std::memcmp(a1.data(), a8.data(), a1.size() * 4), 0);
  EXPECT_EQ(std::memcmp(b1.data(), b8.data(), n * 4), 0);
  EXPECT_EQ(std::memcmp(ls1.data(), ls8.data(), n * 4), 0);

  // forward_batch == per-agent forward_one (byte-identical, same code path)
  for (int i = 0; i < n; ++i) {
    const int o0 = offs[i], ni = offs[i + 1] - o0;
    std::vector<float> ai(ni);
    float bi, gi, lsi, lhi, mli;
    m.forward_one(obs.data() + static_cast<std::size_t>(o0) * 6, mask.data() + o0, ni,
                  goal.data() + static_cast<std::size_t>(i) * 4,
                  patch.data() + static_cast<std::size_t>(i) * 2 * 32 * 32, 32, ai.data(), &bi, &gi,
                  &lsi, &lhi, &mli);
    EXPECT_EQ(std::memcmp(ai.data(), a1.data() + o0, ni * 4), 0) << "agent " << i;
    EXPECT_FLOAT_EQ(bi, b1[i]);
    EXPECT_FLOAT_EQ(lsi, ls1[i]);
  }
}

TEST(NavCoefEnergyNet, BadMagicThrows) {
  const char bad[16] = {'X', 'X', 'X', 'X', 0};
  EXPECT_THROW(coef_energy_net::load_from_memory(bad, sizeof(bad)), std::runtime_error);
}

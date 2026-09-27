// nav_material_deploy_test — the DEPLOYMENT contract for a learned grip/risk/lam coef net.
//
// The material-aware inference path (coef_mlp flags has_mu/has_risk/has_lam, the grip+risk feature
// columns in coef_feats, and the learned-lam extraction in drive_step_material) landed in #413, but
// nothing exercised the WHOLE deploy loop: a widened net serialized to .cvcnav, reloaded, and
// driven on the C++ host — the exact path a demo/game engine takes when it loads a trained policy.
// These tests are that guarantee, plus the fused material+ext drive (drive_step_material_ext) that
// lets a run steer on the learned grip/risk policy WHILE an external RF/comm force also pushes.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/material.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using namespace cvc::nav;
namespace fs = std::filesystem;

namespace {

// A minimal drive world (field + agents), self-contained so this file does not depend on another
// TU's fixtures. Mirrors the nav_material_test rollout_world grid so behaviour is representative.
struct deploy_world {
  int H = 16, W = 16, N = 16;
  std::vector<float> field, mu; // field: [3*H*W]; mu: [H*W] grip plane
  field_stack fs;
  friction_field grip;
  veh_params v;
  std::vector<float> o, th, sp, carrot;
  deploy_world() {
    field.assign(3 * H * W, 0.0f);
    mu.assign(H * W, 1.0f);
    for (int r = 0; r < H; ++r)
      for (int c = 0; c < W; ++c) {
        field[r * W + c] = 0.05f * static_cast<float>((r * 7 + c * 3) % 11) + 0.1f;
        field[H * W + r * W + c] = 0.6f;
        field[2 * H * W + r * W + c] = 0.8f;
        mu[r * W + c] = c < W / 2 ? 0.4f : 1.0f; // low grip on the -x half
      }
    auto setframe = [&](auto &g) {
      g.data = nullptr;
      g.M = 1;
      g.H = H;
      g.W = W;
      g.mnx = -10;
      g.mny = -10;
      g.mxx = 10;
      g.mxy = 10;
      g.cx = 0;
      g.cy = 0;
      g.S = 0.1;
    };
    setframe(fs);
    fs.data = field.data();
    setframe(grip);
    grip.data = mu.data();
    v.rr = 0.15f;
    v.d_hat = 0.35f;
    v.dt = 0.06f;
    v.nsub = 2;
    v.grip = &grip; // so the 6th (grip) feature column is built
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-0.8f, 0.8f);
    o.resize(2 * N);
    carrot.resize(2 * N);
    th.resize(N);
    sp.resize(N);
    for (int i = 0; i < N; ++i) {
      o[2 * i] = u(rng);
      o[2 * i + 1] = u(rng);
      carrot[2 * i] = u(rng);
      carrot[2 * i + 1] = u(rng);
      th[i] = u(rng);
      sp[i] = 0.3f + 0.2f * u(rng);
    }
  }
  // A risk material_stack (6 channels) with a +x risk gradient and no hard hazard nearby.
  material_stack risk_stack(std::vector<float> &store) const {
    const int hw = H * W;
    store.assign(6 * hw, 0.0f);
    for (int r = 0; r < H; ++r)
      for (int c = 0; c < W; ++c) {
        store[0 * hw + r * W + c] = static_cast<float>(c) / static_cast<float>(W - 1); // risk 0..1
        store[1 * hw + r * W + c] = 100.0f;                                            // phi_m far
        store[2 * hw + r * W + c] = 1.0f; // dr/dx > 0 -> soft force pushes -x
      }
    material_stack ms;
    ms.data = store.data();
    ms.M = 1;
    ms.H = H;
    ms.W = W;
    ms.mnx = -10;
    ms.mny = -10;
    ms.mxx = 10;
    ms.mxy = 10;
    ms.cx = 0;
    ms.cy = 0;
    ms.S = 0.1;
    return ms;
  }
};

// Build a single-layer coef net of the given width/flags. Zero weights => the output is the
// out_bias basin (constant (alpha,beta,gamma[,lam])), which is all the deploy/round-trip/fusion
// contracts need (they assert layout survival + force application, not a learned mapping).
static coef_mlp make_net(int in, int out, std::uint32_t flags, float lam = 0.0f) {
  std::vector<float> ob = {1.0f, 3.0f, 4.0f};
  if (out == 4)
    ob.push_back(lam);
  return coef_mlp::from_layers(in, out, {out}, {in}, {0},
                               {std::vector<float>((std::size_t)out * in, 0.0f)},
                               {std::vector<float>((std::size_t)out, 0.0f)}, ob, flags);
}

static std::string tmp_cvcnav(const char *tag) {
  return (fs::temp_directory_path() / (std::string("cvc_deploy_") + tag + ".cvcnav")).string();
}

} // namespace

// (1) The deploy contract: a widened grip+risk+lam net serializes to .cvcnav and reloads with its
// feature/output LAYOUT intact — the flags are what tells the drive which columns to build.
TEST(NavMaterialDeploy, WidenedNetRoundTripsThroughCvcnav) {
  coef_mlp net = make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, 0.7f);
  ASSERT_EQ(net.in_features(), 7);
  ASSERT_EQ(net.out_features(), 4);
  EXPECT_TRUE(net.has_mu());
  EXPECT_TRUE(net.has_risk());
  EXPECT_TRUE(net.has_lam());
  const std::string path = tmp_cvcnav("roundtrip");
  net.save(path);
  coef_mlp back = coef_mlp::load(path);
  EXPECT_EQ(back.in_features(), 7);
  EXPECT_EQ(back.out_features(), 4);
  EXPECT_TRUE(back.has_mu());
  EXPECT_TRUE(back.has_risk());
  EXPECT_TRUE(back.has_lam());
  fs::remove(path);
}

// (2) End-to-end: a LOADED 7-feature+lam net actually drives through the material path.
TEST(NavMaterialDeploy, LoadedGripRiskLamNetDrives) {
  deploy_world w;
  const std::string path = tmp_cvcnav("drive");
  make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, 0.5f).save(path);
  coef_mlp net = coef_mlp::load(path);
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  std::vector<float> ls(w.N, 0.5f), lh(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();
  std::vector<float> o = w.o, th = w.th, sp = w.sp, mc(w.N);
  // Must not throw (feature width 7 == 5+grip+risk, grip + risk sources present) and must move.
  ASSERT_NO_THROW(drive_step_material(w.fs, o.data(), th.data(), sp.data(), w.carrot.data(), net,
                                      w.N, nullptr, w.v, md, mc.data(), 1));
  int moved = 0;
  for (int i = 0; i < 2 * w.N; ++i)
    if (o[i] != w.o[i])
      ++moved;
  EXPECT_GT(moved, 0) << "a loaded grip/risk/lam net did not drive the agents";
  fs::remove(path);
}

// (3a) The fused drive applies BOTH the material force and the ext force in one tick.
static void ext_push_x(void *user, int, int, float, float, float *fx, float *fy) {
  *fx = *static_cast<float *>(user);
  *fy = 0.0f;
}
TEST(NavMaterialDeploy, FusedMaterialExtAppliesBothForces) {
  deploy_world w;
  coef_mlp net = make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, 0.5f);
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  std::vector<float> ls(w.N, 0.5f), lh(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();

  // material-only vs fused (material + a strong +x ext push): the fused trajectory must differ.
  std::vector<float> o_mat = w.o, th_mat = w.th, sp_mat = w.sp, mc_mat(w.N);
  drive_step_material(w.fs, o_mat.data(), th_mat.data(), sp_mat.data(), w.carrot.data(), net, w.N,
                      nullptr, w.v, md, mc_mat.data(), 1);
  float push = 3.0f;
  ext_force ef;
  ef.sample = &ext_push_x;
  ef.user = &push;
  std::vector<float> o_fus = w.o, th_fus = w.th, sp_fus = w.sp, mc_fus(w.N);
  drive_step_material_ext(w.fs, o_fus.data(), th_fus.data(), sp_fus.data(), w.carrot.data(), net,
                          w.N, nullptr, w.v, md, ef, mc_fus.data(), 1);
  int diff = 0;
  for (int i = 0; i < 2 * w.N; ++i)
    if (o_mat[i] != o_fus[i])
      ++diff;
  EXPECT_GT(diff, 0) << "the ext force was dropped on the fused path";

  // (3b) The house pattern: a null ext.sample makes the fused path byte-identical to material-only.
  ext_force nullef; // sample == nullptr
  std::vector<float> o_n = w.o, th_n = w.th, sp_n = w.sp, mc_n(w.N);
  drive_step_material_ext(w.fs, o_n.data(), th_n.data(), sp_n.data(), w.carrot.data(), net, w.N,
                          nullptr, w.v, md, nullef, mc_n.data(), 1);
  EXPECT_EQ(std::memcmp(o_mat.data(), o_n.data(), o_mat.size() * 4), 0);
  EXPECT_EQ(std::memcmp(th_mat.data(), th_n.data(), w.N * 4), 0);
  EXPECT_EQ(std::memcmp(sp_mat.data(), sp_n.data(), w.N * 4), 0);
}

// (3c) The given-coefficients twin: bicycle_rollout_material_ext degrades to each one-sided
// rollout.
TEST(NavMaterialDeploy, FusedRolloutDegradesToOneSided) {
  deploy_world w;
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  std::vector<float> ls(w.N, 0.5f), lh(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();
  std::vector<float> al(w.N, 1.0f), be(w.N, 3.0f), ga(w.N, 4.0f);

  // null ext == bicycle_rollout_material
  std::vector<float> o_m = w.o, th_m = w.th, sp_m = w.sp, mc_m(w.N);
  bicycle_rollout_material(w.fs, o_m.data(), th_m.data(), sp_m.data(), w.carrot.data(), al.data(),
                           be.data(), ga.data(), w.N, nullptr, w.v, md, mc_m.data(), 1);
  ext_force nullef;
  std::vector<float> o_me = w.o, th_me = w.th, sp_me = w.sp, mc_me(w.N);
  bicycle_rollout_material_ext(w.fs, o_me.data(), th_me.data(), sp_me.data(), w.carrot.data(),
                               al.data(), be.data(), ga.data(), w.N, nullptr, w.v, md, nullef,
                               mc_me.data(), 1);
  EXPECT_EQ(std::memcmp(o_m.data(), o_me.data(), o_m.size() * 4), 0);

  // null material == bicycle_rollout_ext
  float push = 2.0f;
  ext_force ef;
  ef.sample = &ext_push_x;
  ef.user = &push;
  material_drive md0; // stack == nullptr
  std::vector<float> o_e = w.o, th_e = w.th, sp_e = w.sp, mc_e(w.N);
  bicycle_rollout_ext(w.fs, o_e.data(), th_e.data(), sp_e.data(), w.carrot.data(), al.data(),
                      be.data(), ga.data(), w.N, nullptr, w.v, ef, mc_e.data(), 1);
  std::vector<float> o_0e = w.o, th_0e = w.th, sp_0e = w.sp, mc_0e(w.N);
  bicycle_rollout_material_ext(w.fs, o_0e.data(), th_0e.data(), sp_0e.data(), w.carrot.data(),
                               al.data(), be.data(), ga.data(), w.N, nullptr, w.v, md0, ef,
                               mc_0e.data(), 1);
  EXPECT_EQ(std::memcmp(o_e.data(), o_0e.data(), o_e.size() * 4), 0);
}

// (4) The learned lam head actually controls reroute strength: a net whose 4th output is a large
// lam_soft reroutes more (down the +x risk gradient => bigger -x deflection) than a lam=0 net.
TEST(NavMaterialDeploy, LearnedLamHeadDrivesReroute) {
  deploy_world w;
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  // The fixed material_drive lam columns are 0, so ALL reroute must come from the net's lam head.
  std::vector<float> ls(w.N, 0.0f), lh(w.N, 0.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();

  auto run = [&](float lam) {
    coef_mlp net = make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, lam);
    std::vector<float> o = w.o, th = w.th, sp = w.sp, mc(w.N);
    for (int step = 0; step < 8; ++step)
      drive_step_material(w.fs, o.data(), th.data(), sp.data(), w.carrot.data(), net, w.N, nullptr,
                          w.v, md, mc.data(), 1);
    double sx = 0;
    for (int i = 0; i < w.N; ++i)
      sx += o[2 * i]; // mean x: more reroute down the +x risk gradient => smaller (more -x)
    return sx / w.N;
  };
  const double x_lam0 = run(0.0f);
  const double x_lam_hi = run(2.0f);
  EXPECT_LT(x_lam_hi, x_lam0 - 1e-4)
      << "the learned lam head did not increase reroute away from +x risk (lam0=" << x_lam0
      << " lamhi=" << x_lam_hi << ")";
}

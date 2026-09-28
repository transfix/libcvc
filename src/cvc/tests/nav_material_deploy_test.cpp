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
#include <fstream>
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

// The two-head sigmoid (v2) twin of make_net: a single-layer net with ZERO linear weights whose
// lam heads' init lives in the last layer's bias (exactly as torch's add_lam_heads). With zero
// weights the pre-activation is that bias, so — by construction — forward gives the closed form
// abg=(1,3,4), lam_soft = smax*sigmoid(logit(soft/smax)) = soft, lam_hard = hmax*..(hard) = hard,
// which is the numpy reference the parity test checks against. out>=5 => two-head (lam_hard).
static coef_mlp make_sigmoid_net(int in, int out, std::uint32_t featflags, float soft_init,
                                 float hard_init, float smax = 5.0f, float hmax = 10.0f) {
  auto logit = [](float x) { return std::log(x / (1.0f - x)); };
  std::vector<float> b(static_cast<std::size_t>(out), 0.0f);
  if (out >= 4)
    b[3] = logit(soft_init / smax);
  if (out >= 5)
    b[4] = logit(hard_init / hmax);
  const std::vector<float> abg = {1.0f, 3.0f, 4.0f}; // v2 out_bias covers ONLY the abg columns
  return coef_mlp::from_layers(in, out, {out}, {in}, {0},
                               {std::vector<float>((std::size_t)out * in, 0.0f)}, {b}, abg,
                               featflags | coef_mlp::kFlagLamSigmoid, smax, hmax);
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

// (5) The SHIPPED cvc-dbg-weights nav policy layout: 6-input RISK-ONLY + lam head — has_risk() but
// NOT has_mu() (no grip column), out=4. grl-snam `coef_train --w-risk --learned-lam` produces
// exactly this (add_lam_head(add_risk_feature(CoefMLP()))), and it's what ships as
// coef_mlp_riskaware.cvcnav / demo3's --grip auto-load. The other tests above drive a 7-in
// grip+risk net; pin the shipped risk-only+lam contract too — it round-trips through .cvcnav and
// drives the material path (which a risk net REQUIRES; the grip field is present but unused as a
// feature since has_mu()==false).
TEST(NavMaterialDeploy, ShippedRiskOnlyLamNetRoundTripsAndDrives) {
  deploy_world w;
  const std::string path = tmp_cvcnav("risk_only_lam");
  make_net(6, 4, coef_mlp::kFlagFeatRisk, 0.5f).save(path);
  coef_mlp net = coef_mlp::load(path);
  EXPECT_EQ(net.in_features(), 6);
  EXPECT_EQ(net.out_features(), 4);
  EXPECT_TRUE(net.has_risk());
  EXPECT_FALSE(net.has_mu()); // risk-only: the kFlagFeatRisk disambiguates the 6-in net from grip
  EXPECT_TRUE(net.has_lam());
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  std::vector<float> ls(w.N, 0.5f), lh(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();
  std::vector<float> o = w.o, th = w.th, sp = w.sp, mc(w.N);
  // Must not throw (feature width 6 == 5+risk, risk source present; no grip feature needed) and
  // move.
  ASSERT_NO_THROW(drive_step_material(w.fs, o.data(), th.data(), sp.data(), w.carrot.data(), net,
                                      w.N, nullptr, w.v, md, mc.data(), 1));
  int moved = 0;
  for (int i = 0; i < 2 * w.N; ++i)
    if (o[i] != w.o[i])
      ++moved;
  EXPECT_GT(moved, 0) << "the shipped 6-in risk-only+lam net did not drive the agents";
  fs::remove(path);
}

// ── two-head sigmoid (.cvcnav format v2) ────────────────────────────────────────────────────
// The paper's two-head reroute: col 3 = lam_soft, col 4 = lam_hard, each lam_max*sigmoid(raw)
// (material_nav.py:175-176). These pin the v2 format contract the Python exporter must match.

// (6) A 5-output v2 net serializes to .cvcnav and reloads with its two-head layout AND the
// sigmoid ceilings intact — the deploy contract for a two-head policy.
TEST(NavMaterialDeploy, TwoHeadSigmoidNetRoundTripsAndKeepsMaxes) {
  coef_mlp net = make_sigmoid_net(6, 5, coef_mlp::kFlagFeatRisk, 2.0f, 4.0f);
  ASSERT_EQ(net.out_features(), 5);
  EXPECT_TRUE(net.has_lam());
  EXPECT_TRUE(net.has_lam_hard());
  EXPECT_TRUE(net.lam_sigmoid());
  EXPECT_EQ(net.format_version(), 2u);
  EXPECT_FLOAT_EQ(net.lam_soft_max(), 5.0f);
  EXPECT_FLOAT_EQ(net.lam_hard_max(), 10.0f);
  const std::string path = tmp_cvcnav("twohead");
  net.save(path);
  coef_mlp back = coef_mlp::load(path);
  EXPECT_EQ(back.format_version(), 2u);
  EXPECT_EQ(back.out_features(), 5);
  EXPECT_TRUE(back.has_lam_hard());
  EXPECT_TRUE(back.lam_sigmoid());
  EXPECT_FLOAT_EQ(back.lam_soft_max(), 5.0f);
  EXPECT_FLOAT_EQ(back.lam_hard_max(), 10.0f);
  // forward is byte-identical across the round-trip (save recovers the raw bias, load re-folds).
  float feat[6] = {0.2f, 5.0f, 0.3f, -0.4f, 0.1f, 0.7f};
  float a[5], b2[5];
  net.forward(feat, 1, a, 1);
  back.forward(feat, 1, b2, 1);
  EXPECT_EQ(std::memcmp(a, b2, sizeof(a)), 0);
  fs::remove(path);
}

// (7) Forward PARITY vs the numpy reference (~1e-4): a zero-weight v2 net's outputs are the
// closed form abg=(1,3,4), lam_soft = smax*sigmoid(logit(soft/smax)) = soft, lam_hard = hard,
// independent of the input; and the lam columns are bounded in [0, max].
TEST(NavMaterialDeploy, SigmoidLamColumnsMatchReferenceAndAreBounded) {
  const float smax = 5.0f, hmax = 10.0f, soft = 2.0f, hard = 7.0f;
  coef_mlp net = make_sigmoid_net(6, 5, coef_mlp::kFlagFeatRisk, soft, hard, smax, hmax);
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> u(-3.0f, 3.0f);
  for (int t = 0; t < 200; ++t) {
    float feat[6];
    for (float &x : feat)
      x = u(rng);
    float o[5];
    net.forward(feat, 1, o, 1);
    EXPECT_NEAR(o[0], 1.0f, 1e-4f);
    EXPECT_NEAR(o[1], 3.0f, 1e-4f);
    EXPECT_NEAR(o[2], 4.0f, 1e-4f);
    EXPECT_NEAR(o[3], soft, 1e-4f); // lam_soft = numpy reference
    EXPECT_NEAR(o[4], hard, 1e-4f); // lam_hard = numpy reference
    EXPECT_GE(o[3], 0.0f);
    EXPECT_LE(o[3], smax);
    EXPECT_GE(o[4], 0.0f);
    EXPECT_LE(o[4], hmax);
  }
}

// (7b) The sigmoid ceiling actually clamps: a net with huge lam-row weights drives the
// pre-activation to +/- inf, and the lam columns must saturate to {0, max}, never beyond.
TEST(NavMaterialDeploy, SigmoidLamStaysBoundedUnderExtremeInput) {
  const int in = 6, out = 5;
  std::vector<float> w((std::size_t)out * in, 0.0f);
  for (int j = 0; j < in; ++j) {
    w[(std::size_t)3 * in + j] = 100.0f;  // lam_soft row: huge +ve -> sigmoid -> 1
    w[(std::size_t)4 * in + j] = -100.0f; // lam_hard row: huge -ve -> sigmoid -> 0
  }
  const std::vector<float> abg = {1.0f, 3.0f, 4.0f};
  coef_mlp net = coef_mlp::from_layers(
      in, out, {out}, {in}, {0}, {w}, {std::vector<float>((std::size_t)out, 0.0f)}, abg,
      coef_mlp::kFlagFeatRisk | coef_mlp::kFlagLamSigmoid, 5.0f, 10.0f);
  float fpos[6] = {9, 9, 9, 9, 9, 9}, fneg[6] = {-9, -9, -9, -9, -9, -9}, o[5];
  net.forward(fpos, 1, o, 1);
  EXPECT_GE(o[3], 0.0f);
  EXPECT_LE(o[3], 5.0f);
  EXPECT_GE(o[4], 0.0f);
  EXPECT_LE(o[4], 10.0f);
  EXPECT_NEAR(o[3], 5.0f, 1e-3f); // saturates to lam_soft_max
  EXPECT_NEAR(o[4], 0.0f, 1e-3f);
  net.forward(fneg, 1, o, 1);
  EXPECT_LE(o[3], 5.0f);
  EXPECT_LE(o[4], 10.0f);
  EXPECT_NEAR(o[3], 0.0f, 1e-3f);
  EXPECT_NEAR(o[4], 10.0f, 1e-3f); // saturates to lam_hard_max
}

// (8) The ON-DISK v2 byte layout the Python exporter must match: version 2, the sigmoid+softplus
// flags, out_bias_len == 3 (abg only), then the two f32 ceilings, then the meta trailer.
TEST(NavMaterialDeploy, V2ByteLayoutMatchesExporterContract) {
  coef_mlp net = make_sigmoid_net(6, 5, coef_mlp::kFlagFeatRisk, 2.0f, 4.0f);
  const std::string path = tmp_cvcnav("v2bytes");
  net.save(path);
  std::ifstream f(path, std::ios::binary);
  std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)),
                              std::istreambuf_iterator<char>());
  f.close();
  const std::uint8_t *p = b.data();
  std::size_t off = 0;
  auto u32 = [&](std::size_t o) {
    std::uint32_t v;
    std::memcpy(&v, p + o, 4);
    return v;
  };
  auto f32 = [&](std::size_t o) {
    float v;
    std::memcpy(&v, p + o, 4);
    return v;
  };
  ASSERT_GE(b.size(), 32u);
  EXPECT_EQ(std::memcmp(p, "CVNV", 4), 0);
  EXPECT_EQ(u32(4), 2u); // format_version
  const std::uint32_t flags = u32(8);
  EXPECT_TRUE(flags & coef_mlp::kFlagLamSigmoid);
  EXPECT_TRUE(flags & coef_mlp::kFlagSoftplusLogExpm1);
  EXPECT_TRUE(flags & coef_mlp::kFlagFeatRisk);
  EXPECT_EQ(u32(12), 6u); // in
  EXPECT_EQ(u32(16), 5u); // out
  EXPECT_EQ(u32(20), 1u); // num_layers
  // header 32 + layer(12 + 5*6*4 + 5*4) = 32 + 12 + 120 + 20 = 184 -> out_bias_len
  off = 184;
  EXPECT_EQ(u32(off), 3u);              // out_bias_len == 3 (abg only)
  off += 4 + 3 * 4;                     // skip out_bias[3]
  EXPECT_FLOAT_EQ(f32(off), 5.0f);      // lam_soft_max
  EXPECT_FLOAT_EQ(f32(off + 4), 10.0f); // lam_hard_max
  EXPECT_EQ(u32(off + 8), 0u);          // meta_len (no meta)
  EXPECT_EQ(b.size(), off + 12);        // exact end of file
  fs::remove(path);
}

// (9) BACK-COMPAT: a v1 (out <= 4, no sigmoid flag) net still loads and forwards all-softplus,
// with its lam column read as softplus(log(expm1(init))) exactly as before the v2 bump.
TEST(NavMaterialDeploy, V1NetsStillLoadAsLegacyAllSoftplus) {
  coef_mlp v1 = make_net(6, 4, coef_mlp::kFlagFeatRisk, 0.5f);
  EXPECT_EQ(v1.format_version(), 1u);
  EXPECT_FALSE(v1.lam_sigmoid());
  const std::string path = tmp_cvcnav("v1compat");
  v1.save(path);
  coef_mlp back = coef_mlp::load(path);
  EXPECT_EQ(back.format_version(), 1u);
  EXPECT_FALSE(back.lam_sigmoid());
  EXPECT_TRUE(back.has_lam());
  EXPECT_FALSE(back.has_lam_hard());
  float feat[6] = {0, 0, 0, 0, 0, 0}, o[4];
  back.forward(feat, 1, o, 1);
  EXPECT_NEAR(o[0], 1.0f, 1e-4f); // abg basin (softplus fold)
  EXPECT_NEAR(o[1], 3.0f, 1e-4f);
  EXPECT_NEAR(o[2], 4.0f, 1e-4f);
  EXPECT_NEAR(o[3], 0.5f, 1e-4f); // lam_soft is the v1 softplus fold, NOT a sigmoid
  fs::remove(path);
}

// (10) The learned lam_hard head is CONSUMED by the drive: with the fixed material lam columns
// zeroed, a two-head net whose lam_hard output is large reroutes harder away from a near hard
// hazard (phi_m small, grad_phi = +x) than one whose lam_hard is small.
TEST(NavMaterialDeploy, LearnedLamHardHeadDrivesHardReroute) {
  deploy_world w;
  const int hw = w.H * w.W;
  std::vector<float> store(6 * hw, 0.0f);
  for (int r = 0; r < w.H; ++r)
    for (int c = 0; c < w.W; ++c) {
      store[0 * hw + r * w.W + c] = 0.0f; // risk ~0 (isolate the hard channel)
      store[1 * hw + r * w.W + c] = 2.0f; // phi_m small -> inside d_hat_m, hard barrier active
      store[4 * hw + r * w.W + c] = 1.0f; // grad_phi_x = +1 -> F_hard pushes +x (away from hazard)
    }
  material_stack ms;
  ms.data = store.data();
  ms.M = 1;
  ms.H = w.H;
  ms.W = w.W;
  ms.mnx = -10;
  ms.mny = -10;
  ms.mxx = 10;
  ms.mxy = 10;
  ms.cx = 0;
  ms.cy = 0;
  ms.S = 0.1;
  std::vector<float> ls(w.N, 0.0f), lh(w.N, 0.0f); // fixed columns 0: all reroute is the net's
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();
  auto run = [&](float hard) {
    coef_mlp net = make_sigmoid_net(6, 5, coef_mlp::kFlagFeatRisk, 0.01f, hard);
    std::vector<float> o = w.o, th = w.th, sp = w.sp, mc(w.N);
    for (int step = 0; step < 8; ++step)
      drive_step_material(w.fs, o.data(), th.data(), sp.data(), w.carrot.data(), net, w.N, nullptr,
                          w.v, md, mc.data(), 1);
    double sx = 0;
    for (int i = 0; i < w.N; ++i)
      sx += o[2 * i];
    return sx / w.N;
  };
  const double x_lo = run(0.05f);
  const double x_hi = run(9.0f);
  EXPECT_GT(x_hi, x_lo + 1e-4)
      << "the learned lam_hard head did not increase hard-hazard reroute (+x) (lo=" << x_lo
      << " hi=" << x_hi << ")";
}

// (11) The force-bias slider fix: material_drive.lam_soft_scale MULTIPLIES a lam-head net's
// LEARNED lam_soft (instead of the caller's fixed lam being clobbered by the net). A scale of 2
// reroutes ~twice as hard down the +x risk gradient as scale 1; scale 0 zeroes the soft force so
// the agents drive nearly straight — proving a live authority lever composes with a trained net.
TEST(NavMaterialDeploy, LamSoftScaleMultipliesLearnedReroute) {
  deploy_world w;
  std::vector<float> store;
  material_stack ms = w.risk_stack(store); // +x risk gradient -> soft force pushes -x
  // Fixed material lam columns are 0, so ALL reroute comes from the net's lam head * the scale.
  std::vector<float> ls(w.N, 0.0f), lh(w.N, 0.0f);
  auto run = [&](float scale) {
    // v1 lam-head net, lam_soft init 1.0; drive_step_material extracts col 3 and multiplies it
    // by md.lam_soft_scale.
    coef_mlp net = make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, 1.0f);
    material_drive md;
    md.stack = &ms;
    md.lam_soft = ls.data();
    md.lam_hard = lh.data();
    md.lam_soft_scale = scale;
    std::vector<float> o = w.o, th = w.th, sp = w.sp, mc(w.N);
    for (int step = 0; step < 8; ++step)
      drive_step_material(w.fs, o.data(), th.data(), sp.data(), w.carrot.data(), net, w.N, nullptr,
                          w.v, md, mc.data(), 1);
    double sx = 0;
    for (int i = 0; i < w.N; ++i)
      sx += o[2 * i]; // mean x: more reroute down the +x risk gradient => smaller (more -x)
    return sx / w.N;
  };
  const double x_s0 = run(0.0f); // scale 0: no soft force -> drives straight (largest mean x)
  const double x_s1 = run(1.0f);
  const double x_s2 = run(2.0f); // 2x reroute -> smallest mean x
  EXPECT_LT(x_s1, x_s0 - 1e-4) << "scale=1 did not reroute more than scale=0 (clobbered?)";
  EXPECT_LT(x_s2, x_s1 - 1e-4) << "scale=2 did not reroute more than scale=1 (scale ignored?)";
}

// Regression for cvcdbg #141 ("attach material stack to the STARTUP world under --grip"). The
// demo3 crash was a --grip startup world that auto-loaded the widened risk net but stepped it
// through the NON-material drive (no material stack attached) — which the plain drive REJECTS, so
// the throw escaped as an uncaught exception at the first frame. The demo binary is not in CI, so
// pin the underlying library invariant here: a terrain-risk net is rejected by drive_step and
// accepted by drive_step_material (with a stack attached). If a future change lets a risk net fall
// through the plain drive, this fails instead of the demo crashing.
TEST(NavMaterialDeploy, RiskNetRejectedByPlainDriveAcceptedByMaterialDrive) {
  deploy_world w;
  coef_mlp net = make_net(7, 4, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk, 0.5f);
  ASSERT_TRUE(net.has_risk());
  // Non-material drive on a risk net: throws (this is the exact throw the demo3 startup hit).
  std::vector<float> o1 = w.o, th1 = w.th, sp1 = w.sp, mc1(w.N);
  EXPECT_THROW(drive_step(w.fs, o1.data(), th1.data(), sp1.data(), w.carrot.data(), net, w.N,
                          nullptr, w.v, mc1.data(), 1),
               std::runtime_error);
  // Same net through the material drive with a stack attached: accepted (the fix's path).
  std::vector<float> store;
  material_stack ms = w.risk_stack(store);
  std::vector<float> ls(w.N, 0.5f), lh(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = ls.data();
  md.lam_hard = lh.data();
  std::vector<float> o2 = w.o, th2 = w.th, sp2 = w.sp, mc2(w.N);
  EXPECT_NO_THROW(drive_step_material(w.fs, o2.data(), th2.data(), sp2.data(), w.carrot.data(), net,
                                      w.N, nullptr, w.v, md, mc2.data(), 1));
}

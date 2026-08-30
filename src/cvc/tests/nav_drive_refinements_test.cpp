/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_drive_refinements_test — the three optional vehicle refinements on
// cvc::nav::bicycle_rollout: multi-disc footprint, inner-wheel steering lock,
// and material grip.
//
// Two classes of invariant, and the first matters more. INERTNESS: each
// refinement must be BYTE-identical to the legacy path at its default, because
// every stored .cvcnav weight and every golden trace depends on the drive not
// moving. EFFECT: each must actually do what it claims, asserted as a behaviour
// rather than a formula echo — a knob that is inert in both directions would
// pass the first class of test just as well.
//
// Everything here is torch-independent: the invariants are internal identities
// (a one-disc footprint IS the legacy sample; three coincident discs ARE one
// disc at three times the coefficient), so this gate holds with no reference
// implementation to hand. The float-equivalence check against torch itself
// lives in the grl-snam repo.

#include <cmath>
#include <cstring>
#include <cvc/nav/drive.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::nav::field_stack;
using cvc::nav::friction_field;
using cvc::nav::veh_params;

namespace {

constexpr int kN = 65;
constexpr double kMin = -100.0, kMax = 100.0, kScale = 0.05;
constexpr float kL = 0.035f;
constexpr float kRR = 0.15f;

// An analytic half-plane wall occupying x >= wall_x, so phi and the outward
// normal are exact and the test does not depend on the EDT.
struct world {
  std::vector<float> data; // [3][H][W]
  field_stack fs;

  explicit world(double wall_x = 50.0) : data(3 * kN * kN, 0.0f) {
    const double step = (kMax - kMin) / (kN - 1);
    for (int r = 0; r < kN; ++r)
      for (int c = 0; c < kN; ++c) {
        const double x = kMin + c * step;
        const int i = r * kN + c;
        data[i] = static_cast<float>((wall_x - x) * kScale); // + outside, - inside
        data[kN * kN + i] = -1.0f;                           // normal: +clearance is -x
        data[2 * kN * kN + i] = 0.0f;
      }
    fs.data = data.data();
    fs.M = 1;
    fs.H = kN;
    fs.W = kN;
    fs.mnx = kMin;
    fs.mny = kMin;
    fs.mxx = kMax;
    fs.mxy = kMax;
    fs.cx = 0.0;
    fs.cy = 0.0;
    fs.S = kScale;
  }
};

veh_params base_veh() {
  veh_params v;
  v.rr = kRR;
  v.d_hat = 0.35f;
  v.dt = 0.06f;
  v.vmax = 0.9f;
  v.L = kL;
  v.delta_max = 0.6f;
  v.a_max = 1.5f;
  v.a_lat_max = 1.0f;
  v.k_steer = 0.8f;
  v.nsub = 2;
  v.allow_reverse = true;
  return v;
}

struct result {
  std::vector<float> o, th, sp, mc;
  bool operator==(const result &r) const {
    return o == r.o && th == r.th && sp == r.sp && mc == r.mc; // exact, on purpose
  }
};

// One rollout of `steps` ticks from a fixed start state.
result roll(const world &w, const veh_params &v, int steps = 10, float x0 = 2.0f, float th0 = 0.0f,
            float sp0 = 0.3f, float goal_x = 3.0f, float al = 1.0f, float be = 3.0f,
            float ga = 4.0f, float goal_y = 0.0f) {
  result r;
  r.o = {x0, 0.0f};
  r.th = {th0};
  r.sp = {sp0};
  r.mc = {9.9f};
  const float goal[2] = {goal_x, goal_y};
  const float A[1] = {al}, B[1] = {be}, G[1] = {ga};
  std::vector<float> mc(1, 9.9f);
  for (int s = 0; s < steps; ++s) {
    cvc::nav::bicycle_rollout(w.fs, r.o.data(), r.th.data(), r.sp.data(), goal, A, B, G, 1, nullptr,
                              v, mc.data(), 1);
    r.mc[0] = std::min(r.mc[0], mc[0]);
  }
  return r;
}

std::vector<float> uniform_grip(float mu) { return std::vector<float>(kN * kN, mu); }

friction_field grip_of(const std::vector<float> &plane) {
  friction_field g;
  g.data = plane.data();
  g.M = 1;
  g.H = kN;
  g.W = kN;
  g.mnx = kMin;
  g.mny = kMin;
  g.mxx = kMax;
  g.mxy = kMax;
  g.cx = 0.0;
  g.cy = 0.0;
  g.S = kScale;
  return g;
}

} // namespace

// ── inertness ───────────────────────────────────────────────────────────────

TEST(NavDriveRefinements, SingleDiscFootprintReducesToLegacy) {
  // The refactor gate: one disc of radius rr at offset 0 IS the legacy sample,
  // so any later divergence is the footprint working, not a rewrite bug.
  const world w;
  const float off[1] = {0.0f};
  veh_params fp = base_veh();
  fp.body_offsets = off;
  fp.n_body = 1;
  fp.body_rr = kRR;
  EXPECT_TRUE(roll(w, base_veh()) == roll(w, fp));
}

TEST(NavDriveRefinements, UniformUnitGripIsLegacy) {
  const world w;
  const std::vector<float> plane = uniform_grip(1.0f);
  const friction_field g = grip_of(plane);
  veh_params v = base_veh();
  v.grip = &g;
  EXPECT_TRUE(roll(w, base_veh()) == roll(w, v));
}

TEST(NavDriveRefinements, ZeroTrackWidthIsLegacy) {
  const world w;
  veh_params v = base_veh();
  v.track_width = 0.0f;
  EXPECT_TRUE(roll(w, base_veh()) == roll(w, v));
}

// ── footprint ───────────────────────────────────────────────────────────────

TEST(NavDriveRefinements, MinOverDiscsBindsAtTheNose) {
  // Heading +x straight at the wall: the nose disc is nearest, so three discs
  // must report strictly LESS clearance than one of the same radius, by about
  // the wheelbase they are spread over.
  const world w;
  const float off[3] = {0.0f, 0.5f * kL, kL};
  veh_params fp = base_veh();
  fp.body_offsets = off;
  fp.n_body = 3;
  fp.body_rr = kRR;
  const float one = roll(w, base_veh(), 1).mc[0];
  const float three = roll(w, fp, 1).mc[0];
  EXPECT_LT(three, one);
  EXPECT_NEAR(three, one - kL, 2e-3f);
}

TEST(NavDriveRefinements, BarrierIsTheSumOverDiscs) {
  // Three COINCIDENT discs are exactly one disc at three times the barrier
  // coefficient. That identity is the precise statement of "the force is the
  // SUM" and cannot be satisfied by accident — which matters, because the
  // a_max clamp hides force differences whenever the goal spring is also
  // saturating the actuator.
  const world w;
  const float one_off[1] = {0.0f};
  const float three_off[3] = {0.0f, 0.0f, 0.0f};
  veh_params a = base_veh();
  a.body_offsets = one_off;
  a.n_body = 1;
  a.body_rr = 0.012f;
  veh_params b = a;
  b.body_offsets = three_off;
  b.n_body = 3;
  // be = ga = 0 isolates the barrier; al small enough not to saturate a_max.
  const result ra = roll(w, a, 3, 2.30f, 0.0f, 0.10f, 2.30f, 0.15f, 0.0f, 0.0f);
  const result rb = roll(w, b, 3, 2.30f, 0.0f, 0.10f, 2.30f, 0.05f, 0.0f, 0.0f);
  EXPECT_NEAR(ra.o[0], rb.o[0], 1e-6f);
  EXPECT_NEAR(ra.sp[0], rb.sp[0], 1e-6f);
}

// ── steering lock ───────────────────────────────────────────────────────────

TEST(NavDriveRefinements, InnerWheelLockWidensTheTurningCircle) {
  // The inner wheel reaches the mechanical lock first, so the achievable
  // VIRTUAL angle is smaller and R_min correspondingly larger. Closed form,
  // then the trajectory: at a speed where the steer limit (not the lateral cap)
  // binds, the locked vehicle must sweep a wider arc.
  const float t = 0.6f * kL;
  const float eff = std::atan(kL / (kL / std::tan(0.6f) + 0.5f * t));
  EXPECT_LT(eff, 0.6f);
  EXPECT_NEAR(eff, 0.5157f, 1e-3f);
  const float r_free = kL / std::tan(0.6f);
  const float r_lock = kL / std::tan(eff);
  EXPECT_NEAR(r_lock, r_free + 0.5f * t, 1e-6f);

  const world w(1e6); // far wall: an open field, so only steering is at play
  veh_params locked = base_veh();
  locked.track_width = t;
  // Measured as arc/dtheta, not heading-after-N-steps: at low speed both
  // vehicles complete the turn and the headings converge, while above
  // sp^2 > a_lat L / tan(delta_max) the LATERAL cap binds first and hides the
  // lock entirely. sp = 0.22 sits in the window where the steer limit binds.
  auto radius = [&](const veh_params &v) {
    const result r = roll(w, v, 6, 0.0f, 0.0f, 0.22f, 0.0f, 1.0f, 3.0f, 4.0f, 0.35f);
    const float dx = r.o[0], dy = r.o[1];
    return std::sqrt(dx * dx + dy * dy) / std::fabs(r.th[0]);
  };
  EXPECT_GT(radius(locked), radius(base_veh()));
}

// ── grip ────────────────────────────────────────────────────────────────────

TEST(NavDriveRefinements, IceShortensTheStopMargin) {
  // v_stop = sqrt(2 mu a_max (d - rr/2)) collapses with grip, so a vehicle that
  // meets a wall on ice gets closer before it can stop.
  const world w;
  const std::vector<float> dry = uniform_grip(1.0f), ice = uniform_grip(0.15f);
  const friction_field gd = grip_of(dry), gi = grip_of(ice);
  veh_params vd = base_veh(), vi = base_veh();
  vd.grip = &gd;
  vi.grip = &gi;
  // driving hard at the wall, goal behind it so the throttle stays on
  const float clr_dry = roll(w, vd, 40, 1.6f, 0.0f, 0.9f, 4.0f).mc[0];
  const float clr_ice = roll(w, vi, 40, 1.6f, 0.0f, 0.9f, 4.0f).mc[0];
  EXPECT_LT(clr_ice, clr_dry);
}

TEST(NavDriveRefinements, GripIsSampledNotGlobal) {
  // A patch must bite only where the vehicle stands on it, or this is a global
  // constant wearing a raster's clothes.
  const world w;
  std::vector<float> patch(kN * kN, 1.0f);
  for (int r = 0; r < kN; ++r)
    for (int c = 0; c < kN / 2; ++c)
      patch[r * kN + c] = 0.15f; // ice on the world's left half (x < 0)
  const friction_field g = grip_of(patch);
  veh_params v = base_veh();
  v.grip = &g;
  // Same manoeuvre on each half; only the surface differs.
  const float on_ice = roll(w, v, 30, -2.0f, 0.0f, 0.9f, -1.0f).sp[0];
  const float on_dry = roll(w, v, 30, 1.0f, 0.0f, 0.9f, 2.0f).sp[0];
  EXPECT_NE(on_ice, on_dry);
}

// ── CPU / CUDA parity ───────────────────────────────────────────────────────

#ifdef CVC_ENABLE_CUDA
TEST(NavDriveRefinements, CudaMatchesCpuWithEveryRefinement) {
  // The device drive must move like the CPU one WITH the refinements on. Not
  // bit-exact — the .cu stores world bounds as float where the CPU keeps them
  // double — but inside the ~1e-5 float-equivalence contract the port is held
  // to. Without this the GPU path could honour fewer constraints than the CPU
  // and nothing would say so.
  const world w;
  const float off[3] = {0.0f, 0.5f * kL, kL};
  const std::vector<float> plane = uniform_grip(0.3f);
  const friction_field g = grip_of(plane);
  veh_params v = base_veh();
  v.body_offsets = off;
  v.n_body = 3;
  v.body_rr = 0.012f;
  v.track_width = 0.6f * kL;
  v.grip = &g;

  const int n = 4;
  std::vector<float> o = {2.0f, 0.0f, 1.5f, 0.4f, 2.2f, -0.3f, 1.0f, 0.2f};
  std::vector<float> th = {0.0f, 0.4f, -0.7f, 1.2f};
  std::vector<float> sp = {0.3f, 0.5f, 0.1f, 0.7f};
  const std::vector<float> goal = {3.0f, 0.0f, 3.0f, 0.5f, 2.8f, -0.4f, 2.5f, 0.3f};
  const std::vector<float> al(n, 1.0f), be(n, 3.0f), ga(n, 4.0f);
  std::vector<float> o2 = o, th2 = th, sp2 = sp, mc(n), mc2(n);

  cvc::nav::bicycle_rollout(w.fs, o.data(), th.data(), sp.data(), goal.data(), al.data(), be.data(),
                            ga.data(), n, nullptr, v, mc.data(), 1);
  cvc::nav::bicycle_rollout_cuda(w.fs, o2.data(), th2.data(), sp2.data(), goal.data(), al.data(),
                                 be.data(), ga.data(), n, v, mc2.data());
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(o[2 * i], o2[2 * i], 1e-5f);
    EXPECT_NEAR(o[2 * i + 1], o2[2 * i + 1], 1e-5f);
    EXPECT_NEAR(th[i], th2[i], 1e-5f);
    EXPECT_NEAR(sp[i], sp2[i], 1e-5f);
    EXPECT_NEAR(mc[i], mc2[i], 1e-5f);
  }
}
#endif

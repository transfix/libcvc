// ariadne_scene_quality_test — the adaptive quality loop (scene_quality.h, O21) on modelled
// machines, and the water's tier table. Pure logic: no GPU, no app.
//
// The slow-machine cases are ABYSSAL's tools/quality-sim.mjs (at 849ff7f4): a frame costs
//   ms = K * tier factor * (render scale * dynamic scale)^2
// and the loop starts on its best tier on a machine that draws that at 1, 4 or 15 fps. Each must
// get to a playable frame (25 ms, 40 fps) within 12 s of wall time. Here the scale is how far out
// each tier's detail is drawn, which buys less than a render scale does, so the same machines are
// run again with a frame that depends on it far less (the scale buys at most a fifth).

#include <cmath>
#include <cvc/ariadne/scene_quality.h>
#include <functional>
#include <gtest/gtest.h>
#include <string>

namespace ari = cvc::ariadne;

namespace {

// ABYSSAL's per-tier factors and render scales (ultra .. potato).
constexpr double kTierFactor[ari::kQualityTiers] = {1.0, 0.62, 0.42, 0.30, 0.21};
constexpr double kRenderScale[ari::kQualityTiers] = {1.0, 1.0, 0.85, 0.72, 0.6};
constexpr double kPlayableMs = 25.0;

double abyssal_cost(double K, int tier, double scale) {
  const double s = kRenderScale[tier] * scale;
  return K * kTierFactor[tier] * s * s;
}

// The water's scale is distances, not pixels: at half scale a frame costs a tenth less.
double distance_cost(double K, int tier, double scale) {
  return K * kTierFactor[tier] * (0.8 + 0.2 * scale);
}

struct Outcome {
  double first_change = -1.0; // s of wall time
  double playable = -1.0;
  int tier = 0;
  double scale = 1.0;
};

// Run the loop against cost(tier, scale) until it has been playable for 5 s (or 180 s pass).
Outcome simulate(ari::AdaptiveQuality &q, const std::function<double(int, double)> &cost) {
  Outcome r;
  double t = 0.0;
  while (t < 180.0) {
    const double ms = cost(q.tier(), q.scale());
    const bool changed = q.tick(ms);
    t += ms / 1000.0;
    if (changed && r.first_change < 0.0)
      r.first_change = t;
    if (r.playable < 0.0 && ms <= kPlayableMs)
      r.playable = t;
    if (r.playable >= 0.0 && t > r.playable + 5.0)
      break;
  }
  r.tier = q.tier();
  r.scale = q.scale();
  return r;
}

} // namespace

TEST(AdaptiveQuality, SlowMachinesGetToAPlayableFrameInSeconds) {
  struct Machine {
    const char *name;
    double K; // ms a frame on the best tier at full scale
  };
  for (const Machine m :
       {Machine{"1 fps", 900.0}, Machine{"4 fps", 250.0}, Machine{"15 fps", 66.0}}) {
    const char *name = m.name;
    const double K = m.K;
    ari::AdaptiveQuality a(17.5); // ABYSSAL's target
    const Outcome r = simulate(a, [K](int t, double s) { return abyssal_cost(K, t, s); });
    EXPECT_GE(r.playable, 0.0) << name;
    EXPECT_LE(r.playable, 12.0) << name << ": playable after " << r.playable << " s";
    // The distances buy at most a fifth on the last tier: 1 and 4 fps machines can only be taken
    // to it (quickly), the 15 fps one to a playable frame.
    ari::AdaptiveQuality b(17.5);
    const auto cost = [K](int t, double s) { return distance_cost(K, t, s); };
    if (cost(ari::kQualityTiers - 1, 0.5) > kPlayableMs) {
      double t = 0.0;
      while (t < 12.0 && b.tier() < ari::kQualityTiers - 1) {
        const double ms = cost(b.tier(), b.scale());
        b.tick(ms);
        t += ms / 1000.0;
      }
      EXPECT_EQ(b.tier(), ari::kQualityTiers - 1) << name << " (distances): not at the last tier";
    } else {
      const Outcome d = simulate(b, cost);
      EXPECT_GE(d.playable, 0.0) << name << " (distances)";
      EXPECT_LE(d.playable, 12.0) << name << " (distances): playable after " << d.playable << " s";
    }
  }
}

TEST(AdaptiveQuality, AHealthyFrameRateIsLeftAlone) {
  ari::AdaptiveQuality q(1000.0 / 60.0);
  for (int i = 0; i < 600; ++i)
    EXPECT_FALSE(q.tick(16.6));
  EXPECT_EQ(q.tier(), 0);
  EXPECT_DOUBLE_EQ(q.scale(), 1.0);
}

TEST(AdaptiveQuality, OneCompileHitchIsNotATrend) {
  ari::AdaptiveQuality q(1000.0 / 60.0);
  for (int i = 0; i < 300; ++i)
    q.tick(i == 150 ? 2000.0 : 16.6);
  EXPECT_EQ(q.tier(), 0); // the median decides a panic, so one 2 s frame drops no tier
}

TEST(AdaptiveQuality, WindowsCloseOnFramesOrTime) {
  ari::AdaptiveQuality q(1000.0 / 30.0);
  for (int i = 0; i < 23; ++i)
    q.tick(10.0);
  EXPECT_EQ(q.windows(), 0); // 23 frames, 230 ms: still open
  q.tick(10.0);
  EXPECT_EQ(q.windows(), 1); // 24 frames
  EXPECT_DOUBLE_EQ(q.mean_ms(), 10.0);
  // At 2 fps a window closes on its fourth frame (2 s), not its 24th (12 s).
  for (int i = 0; i < 3; ++i)
    q.tick(500.0);
  EXPECT_EQ(q.windows(), 1);
  q.tick(500.0);
  EXPECT_EQ(q.windows(), 2);
  EXPECT_DOUBLE_EQ(q.median_ms(), 500.0);
}

TEST(AdaptiveQuality, TheStartsCooldownHoldsTheFirstTwoSeconds) {
  ari::AdaptiveQuality q(1000.0 / 30.0);
  double t = 0.0;
  while (t < 1.9) { // 200 ms frames: six times over, a panic once the cooldown ends
    EXPECT_FALSE(q.tick(200.0));
    t += 0.2;
  }
  bool moved = false;
  for (int i = 0; i < 10 && !moved; ++i)
    moved = q.tick(200.0);
  EXPECT_TRUE(moved);
  EXPECT_GE(q.tier(), 2); // log2(6) rounds to 3 tiers: one move, not three
}

TEST(AdaptiveQuality, ATierComesBackAfterACalmSpell) {
  ari::AdaptiveQuality q(1000.0 / 30.0);
  // 3 s of a machine at a crawl (a world being built), then one that is fast at every tier
  double t = 0.0;
  while (t < 3.0) {
    q.tick(300.0);
    t += 0.3;
  }
  const int low = q.tier();
  EXPECT_GT(low, 0);
  double back = -1.0;
  while (t < 300.0) {
    q.tick(8.0);
    t += 0.008;
    if (q.tier() == 0 && back < 0.0)
      back = t;
  }
  EXPECT_EQ(q.tier(), 0);
  EXPECT_DOUBLE_EQ(q.scale(), 1.0);
  EXPECT_LT(back, 150.0) << "tier 0 again after " << back << " s";
}

TEST(AdaptiveQuality, AMachineOnTheEdgeDoesNotSwing) {
  // Tier 0 is over budget at any scale (55 ms at full, 45 at half: over 1.25 x 33 ms), tier 1
  // is calm (12 ms). Every return to tier 0 is lost again; each waits twice as long as the last.
  ari::AdaptiveQuality q(1000.0 / 30.0);
  int changes = 0, tier = q.tier();
  double t = 0.0, atZero = 0.0;
  while (t < 600.0) {
    const double ms = q.tier() == 0 ? 35.0 + 20.0 * q.scale() : 12.0;
    q.tick(ms);
    t += ms / 1000.0;
    if (t > 30.0 && q.tier() == 0)
      atZero += ms / 1000.0;
    if (q.tier() != tier) {
      ++changes;
      tier = q.tier();
    }
  }
  EXPECT_LE(changes, 20);
  EXPECT_LT(atZero, 0.1 * 570.0); // most of the ten minutes at the tier that holds
}

TEST(AdaptiveQuality, ResetStartsAgain) {
  ari::AdaptiveQuality q(1000.0 / 30.0);
  for (int i = 0; i < 100; ++i)
    q.tick(400.0);
  EXPECT_GT(q.tier(), 0);
  q.reset(1);
  EXPECT_EQ(q.tier(), 1);
  EXPECT_DOUBLE_EQ(q.scale(), 1.0);
  for (int i = 0; i < 4; ++i)
    EXPECT_FALSE(q.tick(400.0)); // the start's cooldown again
}

TEST(AdaptiveQuality, TheSameFramesGiveTheSameMoves) {
  ari::AdaptiveQuality a(1000.0 / 30.0), b(1000.0 / 30.0);
  for (int i = 0; i < 4000; ++i) {
    const double ms = 20.0 + 60.0 * (0.5 + 0.5 * std::sin(i * 0.01));
    EXPECT_EQ(a.tick(ms), b.tick(ms));
    EXPECT_EQ(a.tier(), b.tier());
    EXPECT_EQ(a.scale(), b.scale());
  }
}

TEST(WaterQuality, EachTierDrawsNoMoreThanTheOneAbove) {
  for (int t = 1; t < ari::kQualityTiers; ++t) {
    const ari::WaterQuality a = ari::water_quality(t - 1, 1.0), b = ari::water_quality(t, 1.0);
    EXPECT_LE(b.shade, a.shade) << t;
    EXPECT_LE(b.foam, a.foam) << t;
    EXPECT_LE(b.waves, a.waves) << t;
    EXPECT_LE(b.far_normal, a.far_normal) << t;
    EXPECT_LE(int(b.persistent_foam), int(a.persistent_foam)) << t;
    EXPECT_LE(int(b.wake), int(a.wake)) << t;
    EXPECT_LE(b.wake_cells, a.wake_cells) << t;
    EXPECT_LE(b.cascades, a.cascades) << t;
    EXPECT_LE(b.spray, a.spray) << t;
    EXPECT_LE(b.spray_reach, a.spray_reach) << t;
    if (t >= 2) // tiers 2 to 4 draw no spray (its own draw: what they shed)
      EXPECT_EQ(b.spray, 0.0) << t;
  }
  const ari::WaterQuality full = ari::water_quality(0, 1.0);
  EXPECT_EQ(full.shade, 1.0);
  EXPECT_EQ(full.far_normal, 1.0);
  EXPECT_TRUE(full.persistent_foam && full.wake);
  EXPECT_EQ(full.cascades, 3);
  EXPECT_EQ(full.spray, 1.0);
  EXPECT_EQ(full.spray_reach, 1.0);
  const ari::WaterQuality least = ari::water_quality(ari::kQualityTiers - 1, 1.0);
  EXPECT_EQ(least.shade, 0.0);
  EXPECT_FALSE(least.persistent_foam || least.wake);
  EXPECT_EQ(least.cascades, 2);
  EXPECT_EQ(least.spray, 0.0);
}

TEST(WaterQuality, TheScaleScalesTheDistancesAndIsClamped) {
  const ari::WaterQuality h = ari::water_quality(1, 0.5);
  EXPECT_DOUBLE_EQ(h.shade, 0.25);
  EXPECT_DOUBLE_EQ(h.waves, 0.5);
  EXPECT_TRUE(h.persistent_foam);                          // the switches are the tier's alone
  EXPECT_DOUBLE_EQ(ari::water_quality(0, 0.1).shade, 0.5); // scale at least 0.5
  EXPECT_DOUBLE_EQ(ari::water_quality(0, 3.0).shade, 1.0);
  EXPECT_EQ(ari::water_quality(-2, 1.0).cascades, 3); // the tier clamped
  EXPECT_EQ(ari::water_quality(9, 1.0).cascades, 2);
  EXPECT_STREQ(ari::quality_tier_name(0), "full");
  EXPECT_STREQ(ari::quality_tier_name(7), "least");
}

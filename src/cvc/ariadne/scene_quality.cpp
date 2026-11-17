/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See scene_quality.h for the ABYSSAL (MIT) attribution
  that applies to AdaptiveQuality's loop.
*/

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/scene_quality.h>

namespace cvc {
namespace ariadne {

namespace {
constexpr int kMinFrames = 4;       // a window never closes on fewer
constexpr double kSampleMs = 400.0; // ... and closes after this long, if not 24 frames
constexpr double kMinScale = 0.5;
// A median this far over sheds tiers at once. ABYSSAL's is 4: there a step of the scale is a step
// of the render resolution, and pixels go as its square; here the scale moves distances, which buy
// far less, so a frame twice over its budget does not wait on it.
constexpr double kPanic = 2.0;
constexpr double kOver = 1.25, kUnder = 0.68;
constexpr double kCalm = 0.45;      // a window this far under, at full scale, is calm
constexpr double kProbation = 10.0; // s: a tier lost again this soon after coming back
constexpr double kCalmMax = 120.0;  // s
constexpr const char *kTierNames[kQualityTiers] = {"full", "high", "medium", "low", "least"};
} // namespace

const char *quality_tier_name(int tier) {
  return kTierNames[std::clamp(tier, 0, kQualityTiers - 1)];
}

AdaptiveQuality::AdaptiveQuality(double target_ms) : _target(std::clamp(target_ms, 1.0, 1000.0)) {
  reset(0);
}

void AdaptiveQuality::set_target_ms(double ms) { _target = std::clamp(ms, 1.0, 1000.0); }

void AdaptiveQuality::reset(int tier) {
  _tier = std::clamp(tier, 0, kQualityTiers - 1);
  _scale = 1.0;
  _cooldown = 2.0;
  _count = 0;
  _acc = 0.0;
  _calm = 0.0;
  _calmNeeded = 5.0;
  _sinceUpgrade = 1e30;
}

bool AdaptiveQuality::tick(double frame_ms) {
  frame_ms = std::max(frame_ms, 0.0);
  if (_count < kSampleFrames)
    _window[std::size_t(_count)] = frame_ms;
  _acc += frame_ms;
  ++_count;
  _cooldown -= frame_ms / 1000.0;
  _sinceUpgrade += frame_ms / 1000.0;

  // Close the window on frames or on wall time, whichever comes first: 24 frames are 0.4 s at
  // 60 fps and 12 s at 2, and twelve seconds of a frozen view is what the loop is for.
  if (_count < kMinFrames || (_count < kSampleFrames && _acc < kSampleMs))
    return false;
  const int n = std::min(_count, kSampleFrames);
  std::array<double, kSampleFrames> s = _window;
  std::sort(s.begin(), s.begin() + n);
  const double windowS = _acc / 1000.0;
  _mean = _acc / _count;
  _median =
      (n & 1) ? s[std::size_t(n / 2)] : 0.5 * (s[std::size_t(n / 2 - 1)] + s[std::size_t(n / 2)]);
  ++_windows;
  _acc = 0.0;
  _count = 0;
  if (_cooldown > 0.0)
    return false;

  const int tier0 = _tier;
  const double scale0 = _scale;
  const auto drop = [&](int tiers, double scale, double cooldown) {
    _tier = std::min(_tier + tiers, kQualityTiers - 1);
    _scale = scale;
    _cooldown = cooldown;
    _calm = 0.0;
    if (_sinceUpgrade < kProbation) // it came back too soon: wait longer next time
      _calmNeeded = std::min(2.0 * _calmNeeded, kCalmMax);
  };

  if (_median > kPanic * _target) {
    // A tier is worth about a halving of the cost: skip as many as the overshoot has octaves.
    const int tiers = std::max(1, int(std::lround(std::log2(_median / _target))));
    if (_tier < kQualityTiers - 1) {
      drop(tiers, kMinScale, 3.0);
    } else {
      _scale = kMinScale; // the last tier: the scale is all that is left
      _cooldown = 2.0;
    }
  } else if (_mean > kOver * _target) {
    _calm = 0.0;
    if (_scale <= kMinScale + 0.06 && _tier < kQualityTiers - 1) {
      drop(1, 1.0, 4.0);
    } else {
      _scale = std::max(kMinScale, _scale - 0.09);
      _cooldown = 0.9;
    }
  } else if (_mean < kCalm * _target && _scale >= 1.0 - 1e-9 && _tier > 0) {
    _calm += windowS;
    if (_calm >= _calmNeeded) { // a tier back, at half scale, the scale to climb from there
      --_tier;
      _scale = kMinScale;
      _cooldown = 3.0;
      _calm = 0.0;
      _sinceUpgrade = 0.0;
    }
  } else {
    _calm = 0.0;
    if (_mean < kUnder * _target && _scale < 1.0) {
      _scale = std::min(1.0, _scale + 0.045);
      _cooldown = 1.5;
    }
  }
  return _tier != tier0 || std::abs(_scale - scale0) > 1e-4;
}

WaterQuality water_quality(int tier, double scale) {
  WaterQuality q;
  const double s = std::clamp(scale, kMinScale, 1.0);
  switch (std::clamp(tier, 0, kQualityTiers - 1)) {
  case 0:
    break;
  case 1:
    q.shade = q.foam = 0.5;
    q.wake_cells = 128;
    break;
  case 2:
    q.shade = 0.25;
    q.foam = 0.0;
    q.waves = q.far_normal = 0.5;
    q.persistent_foam = false;
    q.wake_cells = 128;
    break;
  case 3:
    q.shade = q.foam = q.waves = 0.0;
    q.far_normal = 0.5;
    q.persistent_foam = q.wake = false;
    q.wake_cells = 128;
    q.cascades = 2;
    break;
  default:
    q.shade = q.foam = q.waves = 0.0;
    q.far_normal = 0.25;
    q.persistent_foam = q.wake = false;
    q.wake_cells = 128;
    q.cascades = 2;
    break;
  }
  q.shade *= s;
  q.foam *= s;
  q.waves *= s;
  q.far_normal *= s;
  return q;
}

} // namespace ariadne
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  AdaptiveQuality's loop is a C++ port of ABYSSAL's Quality.tick
  (github.com/Token-Gremlin/natural-disasters, src/core/Quality.js at 849ff7f4), MIT licensed,
  Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained.
*/

// scene_quality.h — adaptive quality (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §10.10, O21): a
// closed loop on the frame's wall time that steps how much the scene draws, so a GPU that cannot
// hold the frame rate gets back to it in seconds.
//
// The loop is scene-wide: a host that knows the frame's wall time (cvcGL's AriRuntime, outside
// its fixed clock) ticks one AdaptiveQuality and publishes where it stands as keys under the
// scene's prefix:
//   quality.adaptive    1 (the default): the loop runs; 0 holds the tier and scale where they are
//                       (and they can be set by hand)
//   quality.target_fps  the frame rate to hold (default 30)
//   quality.tier        written by the loop: 0 (everything) .. 4 (the least)
//   quality.scale       written by the loop: 0.5 .. 1, how far out each tier's detail is drawn
//   quality.frame_ms    written by the loop: the last window's mean frame time
// What draws reads them; the water node (water_quality below) is the first. Only drawing changes:
// nothing a tier or a scale sets reaches the physics, so a run is the same at every tier.

#ifndef __CVC_ARIADNE_SCENE_QUALITY_H__
#define __CVC_ARIADNE_SCENE_QUALITY_H__

#include <array>
#include <string>

namespace cvc {
class app;
namespace ariadne {

constexpr int kQualityTiers = 5; // ABYSSAL's ultra, high, medium, low, potato

// "full", "high", "medium", "low", "least" (clamped).
const char *quality_tier_name(int tier);

// The loop (ABYSSAL's Quality.tick). Frames are gathered into windows of 24 frames or 400 ms,
// whichever closes first (never fewer than 4), and each window decides, unless a cooldown from
// the last move is still running:
//   - panic: the window's MEDIAN over 2 x target (a median, so one shader compile is not a trend;
//     ABYSSAL's is 4 x, see scene_quality.cpp) drops round(log2(median / target)) tiers at once,
//     at half scale (cooldown 3 s); on the last tier, the scale goes to its floor (2 s);
//   - over: a MEAN over 1.25 x target takes 0.09 off the scale (0.9 s), or, at the floor, drops a
//     tier at full scale (4 s);
//   - under: a mean under 0.68 x target adds 0.045 to the scale (1.5 s).
// ABYSSAL never goes back up a tier. Here a tier comes back after a calm spell (every window's
// mean under 0.45 x target at full scale for 5 s), at half scale (3 s): a slow start (shaders
// compiling, a world being built) or a passing load on the machine does not cost the session its
// quality. A tier dropped again within 10 s of coming back doubles the next calm spell asked for
// (at most 120 s), so a machine on the edge between two tiers does not swing between them.
class AdaptiveQuality {
public:
  explicit AdaptiveQuality(double target_ms = 1000.0 / 30.0);

  void set_target_ms(double ms); // clamped to 1 .. 1000
  double target_ms() const { return _target; }

  // Start again at `tier`, full scale, with the start's cooldown (2 s), the calm spell asked for
  // back at 5 s, and an empty window.
  void reset(int tier = 0);

  // One frame's wall time (ms). True when tier() or scale() changed.
  bool tick(double frame_ms);

  int tier() const { return _tier; }
  double scale() const { return _scale; }
  // The last closed window: its mean and median frame time (0 before one closes), and how many
  // windows have closed.
  double mean_ms() const { return _mean; }
  double median_ms() const { return _median; }
  long long windows() const { return _windows; }

private:
  static constexpr int kSampleFrames = 24;
  double _target;
  int _tier = 0;
  double _scale = 1.0;
  double _cooldown = 2.0; // s
  std::array<double, kSampleFrames> _window{};
  int _count = 0;
  double _acc = 0.0; // ms
  double _mean = 0.0, _median = 0.0;
  long long _windows = 0;
  double _calm = 0.0;          // s of calm windows at full scale in a row
  double _calmNeeded = 5.0;    // s
  double _sinceUpgrade = 1e30; // s since a tier last came back
};

// What the water node draws at a tier and scale. The distances are scales on the node's own
// level-of-detail keys (water.lod_shade, lod_foam, lod_waves) and on how far the far grid's normal
// keeps the cascades (10 near grids for the long one); the switches turn a piece off outright.
struct WaterQuality {
  double shade = 1.0;          // water.lod_shade's scale (0: the simple shading everywhere)
  double foam = 1.0;           // the persistent foam's reach, a share of water.lod_foam (the
                               // whitecaps keep all of it: they cost nothing); 0 with it off
  double waves = 1.0;          // water.lod_waves's scale: the short cascade's reach
  double far_normal = 1.0;     // the far grid's normal's reach
  bool persistent_foam = true; // the foam a breaking wave leaves (OceanFoam's passes)
  bool wake = true;            // the bodies' waves (the eWave window's passes and reads)
  int wake_cells = 256;        // their window's cells a side (0.5 m each: 128 m, or 64 m at 128)
  // The window's fixed substep (s; ewave_params::substep): its passes a frame go as the frame's
  // time over it. 1/30 halves them against 1/60 and keeps open water's exact dispersion.
  double wake_substep = 1.0 / 60.0;
  int cascades = 3; // 2: the short cascade is not transformed or read
  // The wind-blown spray (§10.5): its particles, a share of water.spray_count (0: none drawn), and
  // its reach, a share of water.lod_spray. Tiers 2 to 4 draw none.
  double spray = 1.0;
  double spray_reach = 1.0;
};

// The tier table (tier clamped to 0 .. 4, scale to 0.5 .. 1; the distances are the tier's times
// the scale):
//   0 full    everything
//   1 high    the full shading and the persistent foam to half their distance; the wake's window
//             a quarter the cells (half the reach: 64 m); half the spray, to half its reach
//   2 medium  ... the full shading to a quarter; the short cascade and the far normal to half; no
//             persistent foam (the whitecaps alone take Monahan's share); no spray (its own draw)
//   3 low     the simple shading everywhere; two cascades (no short one); the far normal to half;
//             the wake still drawn, on the 64 m window in 1/30 s substeps (half the passes)
//   4 least   ... and the far normal to a quarter
// (What each saves, measured: OCEAN-AND-WEATHER-ROADMAP.md, O21's notes.)
WaterQuality water_quality(int tier, double scale);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_QUALITY_H__

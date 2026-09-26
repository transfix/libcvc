/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_stats_publish.h — bridge a base cvc::nav::episode_nav_stats snapshot into the running scene's
// cvc::state tree, off the render path, for realtime ImGui / the cvcGL UI DSL to read (SPEC 1 in
// CVC-DBG/cvcdbg docs/stats-statetree-and-training.md). This is the BASE + formation half
// (RF-free); the cvc::dbg RF sub-record is published by cvc/dbg/nav_stats_publish.h, and the
// belief/fog RASTERS take a separate data()-lane handle path (the publisher carries only
// value()-lane scalars). Keying is by the scene's SceneGraph prefix (getStatePrefix()), so
// concurrent sims under distinct prefixes never collide.
#pragma once

#include <cvc/gl/state_publisher.h>
#include <cvc/nav/nav_stats.h>
#include <string>

namespace cvc {
namespace gl {

// "<prefix>.nav_stats" — the per-scene nav-stats subtree root, matching the house sceneStatePath
// convention ("<prefix>.<subsystem>"). All leaves below live under it.
std::string navStatsStatePath(const std::string &scenePrefix);

// Bounds the path count of publish_nav_stats (the live tier can be ~N vehicles × ~15 leaves per
// tick).
struct nav_stats_publish_params {
  int per_vehicle_every =
      1; // publish per-vehicle leaves only for veh i where i % this == 0 (1 = all)
  bool emit_coverage = true;  // publish the episode .coverage.* belief-coverage fractions
  bool emit_formation = true; // publish the per-convoy .formation.<id>.* holding aggregates
  bool complete = false; // stamp meta.complete = 1 (final/aggregate dump at the episode boundary)
};

// Publish one base episode record into <prefix>.nav_stats.* through the scene's state_publisher
// (value lane, coalesced, drained by its off-render worker). finish() is idempotent (a live running
// snapshot), so call this every tick with the current snapshot; the publisher coalesces to the last
// value per path. `tick` stamps meta.tick. Sentinel doubles (min_sep_m / min_clearance_m == 1e30)
// are published as-is; a reader treats >= 1e29 as "unmeasured" (the value lane has no null).
void publish_nav_stats(cvc::gl::state_publisher &pub, const std::string &scenePrefix,
                       const cvc::nav::episode_nav_stats &e, long tick,
                       const nav_stats_publish_params &p = {});

} // namespace gl
} // namespace cvc

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

#include <cstdint>
#include <cvc/gl/state_publisher.h>
#include <cvc/nav/nav_stats.h>
#include <string>
#include <vector>

namespace cvc {
class app; // fwd (the raster publisher needs it for the cvc::volume ctor + the state root)
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

// ── belief/fog raster bridge (realtime visualization; data() lane) ──────────────
// Raster geometry for publish_nav_rasters: per-plane dims + the world bbox a reader needs to place
// the volume (rows/cols per plane; planes = sim_world::planes()).
struct nav_raster_dims {
  int rows = 0, cols = 0, planes = 0;
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
};

// Version-gate state for publish_nav_rasters — ONE per scene, held by the caller across ticks.
// Records the last-published version per plane (and truth), so an unchanged plane skips the
// (deep-copying) cvc::volume wrap.
struct nav_raster_pub_state {
  int truth_version = -1;          // -1 = truth not yet published (it is static — published once)
  std::vector<int> plane_versions; // last-published version per plane (sized to planes; -1 = never)
};

// Publish the belief/fog rasters into <prefix>.nav_stats.rasters.* for realtime viz. `truth` is
// [rows*cols] (one shared plane, published ONCE); `belief`/`everseen`/`lastvis` are
// [planes*rows*cols] (plane m at m*rows*cols) — the sim_world
// truth()/belief_occ(m)/ever_seen(m)/last_visible(m) surface. `versions[m]` is
// sim_world::plane_version(m). Each raster is wrapped as a z=1 cvc::volume and stored on the node's
// data() lane (a shallow-share handle; VolumeNode renders it), with the paired .version + .dims
// scalars on the value lane through `pub`. A plane's volumes are re-wrapped only when versions[m]
// changed vs `st` (the deep copy is version-gated). Same-process only: the data() lane does not
// cross a process boundary — a remote viewer opts into the brick escalation behind the identical
// version/dims.
void publish_nav_rasters(cvc::app &app, cvc::gl::state_publisher &pub,
                         const std::string &scenePrefix, const nav_raster_dims &dims,
                         const std::uint8_t *truth, const std::uint8_t *belief,
                         const std::uint8_t *everseen, const std::uint8_t *lastvis,
                         const int *versions, nav_raster_pub_state &st);

} // namespace gl
} // namespace cvc

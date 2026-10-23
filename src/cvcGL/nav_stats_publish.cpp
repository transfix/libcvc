/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_stats_publish.cpp — see inc/cvc/gl/nav_stats_publish.h.

#include <boost/any.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/nav_stats_publish.h>
#include <cvc/nav/sim_world.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <map>
#include <string>

namespace cvc {
namespace gl {

std::string navStatsStatePath(const std::string &scenePrefix) { return scenePrefix + ".nav_stats"; }

namespace {
// %.9g: precision-controlled so a sub-1e-6 fraction (e.g. phantom_frac / slot_error) is not
// flattened to "0.000000" by std::to_string's %f, and the 1e30 sentinel stays compact ("1e+30",
// still >= 1e29).
inline std::string d2s(double v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%.9g", v);
  return b;
}
// long long so int / long / std::int64_t widen without truncation on LLP32 (Windows) — matches the
// int64 sense_flips invariant; unsigned seed goes through std::to_string(unsigned) directly.
inline std::string i2s(long long v) { return std::to_string(v); }
inline std::string b2s(bool v) { return v ? "1" : "0"; }

// Per-convoy formation-holding aggregate (followers = formation_parent >= 0).
struct convoy_acc {
  double slot_err_sum = 0;
  int followers = 0;
  int in_slot = 0;       // followers that ended in-slot (formation_arrived)
  bool anchor_ok = true; // the convoy's anchor (parent < 0) reached the objective
};
} // namespace

namespace {

// What a nav_stats leaf is, for other applications: its comment and its "access" (all of these
// are telemetry) and "unit" properties. A leaf is described when it is first written, so a strided
// or untracked one stays unset.
struct leaf_meta {
  const char *key;
  const char *unit;
  const char *comment;
};

const leaf_meta kMeta[] = {
    {"meta.alive", "", "1 while this episode's stats are published."},
    {"meta.scene_id", nullptr, "The scene the episode runs in."},
    {"meta.checkpoint", nullptr, "The policy checkpoint driving the vehicles."},
    {"meta.seed", "", "The episode's seed."},
    {"meta.n_vehicles", "", "Vehicles in the episode."},
    {"meta.tick", "", "The sim tick these stats are as of."},
    {"meta.dt_s", "s", "Seconds per sim tick."},
    {"meta.complete", "", "1 on the final record of an episode."},
    {"episode.arrived", "", "Vehicles that reached their goal."},
    {"episode.success", "", "1 when every vehicle arrived within budget."},
    {"episode.makespan_s", "s", "When the last arriving vehicle arrived."},
    {"episode.mean_ttg_s", "s", "The arrivers' mean time to goal."},
    {"episode.penetration_pct", "%", "Vehicle-ticks with the vehicle centre in an occupied cell."},
    {"episode.total_veh_contacts", "", "Vehicle-ticks within min_gap of another vehicle."},
    {"episode.min_sep_m", "m", "The closest two vehicles came; >= 1e29 = unmeasured."},
    {"episode.mean_path_ratio", "1", "Mean path length over the straight-line distance."},
    {"episode.mean_turn_total_rad", "rad", "Mean total heading change per vehicle."},
    {"episode.total_fuel", "m/s", "The fleet's sum of |change of speed|: a fuel proxy, not fuel."},
    {"episode.total_energy_J", "J", "The fleet's energy drawn from its sources (a physics plant)."},
    {"episode.total_building_contacts", "",
     "The fleet's contacts with static geometry (a physics plant)."},
    {"episode.total_infeasible_slope_steps", "",
     "The fleet's vehicle-ticks with a footprint on infeasible slope (a terrain)."},
    {"episode.vehicles_rolled", "", "Vehicles that rolled over (a physics plant)."},
    {"episode.first_roll_s", "s",
     "When the first vehicle rolled over; < 0 = none (a physics plant)."},
    {"coverage.explored_frac", "1", "Belief cells ever seen."},
    {"coverage.visible_frac", "1", "Belief cells in view on the last frame."},
    {"coverage.believed_free_frac", "1", "Belief cells read as free."},
    {"coverage.phantom_frac", "1", "Belief cells read as occupied where the truth is free."},
};

const leaf_meta kFormation[] = {
    {"followers", "", "Vehicles holding station in this convoy."},
    {"slot_error_mean_m", "m", "The followers' mean distance from their formation slots."},
    {"form_arrival", "", "1 when every follower ended in its slot and the anchor arrived."},
};

const leaf_meta kVehicle[] = {
    {"arrived", "", "1 once the vehicle reached its goal."},
    {"time_to_goal_s", "s", "When it first reached its goal; < 0 = not yet."},
    {"closest_approach_m", "m", "The nearest it came to its goal."},
    {"stall_steps", "", "Ticks that failed to beat its closest approach."},
    {"min_clearance_m", "m", "Its least clearance to a wall; >= 1e29 = unmeasured."},
    {"penetration_steps", "", "Ticks with its centre in an occupied cell."},
    {"veh_contacts", "", "Ticks within min_gap of another vehicle."},
    {"fuel_used", "m/s", "Its sum of |change of speed|: a fuel proxy, not fuel."},
    {"sense_flips", "", "Belief cells its sensing flipped."},
    {"formation_parent", "", "The vehicle it holds station on; -1 = none (the anchor)."},
    {"slot_error_mean_m", "m", "Its mean distance from its formation slot."},
    {"formation_arrived", "", "1 when it ended in its formation slot."},
    {"energy_J", "J", "The energy it drew from its source, net of regeneration (a physics plant)."},
    {"energy_per_km", "J/km", "energy_J per kilometre of its path."},
    {"building_contacts", "", "Contacts its body started with static geometry (a physics plant)."},
    {"infeasible_slope_steps", "",
     "Ticks its footprint touched slope steeper than it can climb or cross (a terrain)."},
    {"rolled_over", "", "1 once it rolled over: tipped past 60 degrees for 1 s (a physics plant)."},
    {"rolled_over_s", "s", "When it tipped over for good; < 0 = never (a physics plant)."},
    {"tilt_max_deg", "deg", "The most its body tilted from upright (a physics plant)."},
    {"ltr_max", "1",
     "Its largest |lateral load transfer ratio|; 1 = a side lifts (a physics plant)."},
    {"a_lat_max_g", "g", "Its largest |lateral acceleration| (a physics plant)."},
    {"steer_sat_share", "1",
     "Share of ticks nav steered at 95% of its lock or more (a physics plant)."},
};

bool plant_leaf(const std::string &k) {
  return k == "rolled_over" || k == "rolled_over_s" || k == "tilt_max_deg" || k == "ltr_max" ||
         k == "a_lat_max_g" || k == "steer_sat_share";
}

void describe_leaf(cvc::app &ctx, const std::string &path, const leaf_meta &m) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    if (!s.comment().empty())
      return; // described already
    s.comment(m.comment);
    s.property("access", "telemetry");
    if (m.unit)
      s.property("unit", m.unit);
  } catch (const std::exception &) {
  }
}

template <std::size_t K>
void describe_leaves(cvc::app &ctx, const std::string &base, const leaf_meta (&t)[K],
                     bool (*keep)(const char *) = nullptr) {
  for (const leaf_meta &m : t)
    if (!keep || keep(m.key))
      describe_leaf(ctx, base + "." + m.key, m);
}

} // namespace

void publish_nav_stats(cvc::gl::state_publisher &pub, const std::string &scenePrefix,
                       const cvc::nav::episode_nav_stats &e, long tick,
                       const nav_stats_publish_params &p) {
  const std::string root = navStatsStatePath(scenePrefix);
  // Describe the leaves this call writes (each once): the energy ones only when tracked, the
  // coverage ones only when emitted.
  {
    const bool energy = e.energy_tracked, coverage = p.emit_coverage;
    for (const leaf_meta &m : kMeta) {
      const std::string k = m.key;
      if (!energy && (k == "episode.total_energy_J" || k == "episode.total_building_contacts"))
        continue;
      if (!e.terrain_tracked && k == "episode.total_infeasible_slope_steps")
        continue;
      if (!e.plant_tracked && (k == "episode.vehicles_rolled" || k == "episode.first_roll_s"))
        continue;
      if (!coverage && k.compare(0, 9, "coverage.") == 0)
        continue;
      describe_leaf(pub.ctx(), root + "." + k, m);
    }
  }

  // ── meta ──
  pub.publish(root + ".meta.alive", "1");
  pub.publish(root + ".meta.scene_id", e.scene_id);
  pub.publish(root + ".meta.checkpoint", e.checkpoint);
  pub.publish(root + ".meta.seed", std::to_string(e.seed)); // unsigned — no signed narrowing
  pub.publish(root + ".meta.n_vehicles", i2s(e.n_vehicles));
  pub.publish(root + ".meta.tick", i2s(tick));
  pub.publish(root + ".meta.dt_s", d2s(e.dt_s));
  pub.publish(root + ".meta.complete", b2s(p.complete));

  // ── episode ──
  const std::string ep = root + ".episode";
  pub.publish(ep + ".arrived", i2s(e.arrived));
  pub.publish(ep + ".success", b2s(e.success));
  pub.publish(ep + ".makespan_s", d2s(e.makespan_s));
  pub.publish(ep + ".mean_ttg_s", d2s(e.mean_ttg_s));
  pub.publish(ep + ".penetration_pct", d2s(e.penetration_pct));
  pub.publish(ep + ".total_veh_contacts", i2s(e.total_veh_contacts));
  pub.publish(ep + ".min_sep_m", d2s(e.min_sep_m));
  pub.publish(ep + ".mean_path_ratio", d2s(e.mean_path_ratio));
  pub.publish(ep + ".mean_turn_total_rad", d2s(e.mean_turn_total_rad));
  pub.publish(ep + ".total_fuel", d2s(e.total_fuel));
  if (e.energy_tracked) { // a physics plant measured them (nav_stats.h energy_tracked)
    pub.publish(ep + ".total_energy_J", d2s(e.total_energy_J));
    pub.publish(ep + ".total_building_contacts", i2s(e.total_building_contacts));
  }
  if (e.terrain_tracked) // a terrain was set (nav_stats.h terrain_tracked)
    pub.publish(ep + ".total_infeasible_slope_steps", i2s(e.total_infeasible_slope_steps));
  if (e.plant_tracked) { // a physics plant fed the plant ledger (nav_stats.h plant_tracked)
    pub.publish(ep + ".vehicles_rolled", i2s(e.vehicles_rolled));
    pub.publish(ep + ".first_roll_s", d2s(e.first_roll_s));
  }

  // ── coverage (episode belief-coverage fractions) ──
  if (p.emit_coverage) {
    const std::string cov = root + ".coverage";
    pub.publish(cov + ".explored_frac", d2s(e.coverage.explored_frac));
    pub.publish(cov + ".visible_frac", d2s(e.coverage.visible_frac));
    pub.publish(cov + ".believed_free_frac", d2s(e.coverage.believed_free_frac));
    pub.publish(cov + ".phantom_frac", d2s(e.coverage.phantom_frac));
  }

  // ── per-convoy formation aggregate (over ALL vehicles, independent of the per-vehicle stride) ──
  std::map<int, convoy_acc> convoys;
  if (p.emit_formation) {
    for (const auto &v : e.per_vehicle) {
      convoy_acc &c = convoys[v.convoy_id];
      if (v.formation_parent >= 0) { // follower
        ++c.followers;
        c.slot_err_sum += v.slot_error_mean_m;
        if (v.formation_arrived)
          ++c.in_slot;
      } else if (!v.arrived) { // anchor must reach the objective for the convoy to be "formed +
                               // arrived"
        c.anchor_ok = false;
      }
    }
    for (const auto &kv : convoys) {
      if (kv.second.followers == 0)
        continue; // no followers -> not a formation
      const std::string fp = root + ".formation." + std::to_string(kv.first);
      describe_leaves(pub.ctx(), fp, kFormation);
      pub.publish(fp + ".followers", i2s(kv.second.followers));
      pub.publish(fp + ".slot_error_mean_m", d2s(kv.second.slot_err_sum / kv.second.followers));
      // form_arrival: every follower in-slot AND the anchor arrived (the honest "formation
      // arrived").
      const bool ok = kv.second.in_slot == kv.second.followers && kv.second.anchor_ok;
      pub.publish(fp + ".form_arrival", b2s(ok));
    }
  }

  // ── per-vehicle (compact live tier; strided by per_vehicle_every) ──
  const int stride = p.per_vehicle_every > 1 ? p.per_vehicle_every : 1;
  for (std::size_t i = 0; i < e.per_vehicle.size(); ++i) {
    if (stride > 1 && (int)(i % stride) != 0)
      continue;
    const auto &v = e.per_vehicle[i];
    // Key by veh_index (the extension-record join key), not array position — identical on a
    // collector-produced record (position == veh_index) but self-consistent with an extension's
    // publisher for reordered/sparse ones.
    const std::string vp = root + ".veh." + std::to_string(v.veh_index);
    for (const leaf_meta &m : kVehicle) {
      const std::string k = m.key;
      if (!v.energy_tracked &&
          (k == "energy_J" || k == "energy_per_km" || k == "building_contacts"))
        continue;
      if (!v.terrain_tracked && k == "infeasible_slope_steps")
        continue;
      if (!v.plant_tracked && plant_leaf(k))
        continue;
      describe_leaf(pub.ctx(), vp + "." + k, m);
    }
    pub.publish(vp + ".arrived", b2s(v.arrived));
    pub.publish(vp + ".time_to_goal_s", d2s(v.time_to_goal_s));
    pub.publish(vp + ".closest_approach_m", d2s(v.closest_approach_m));
    pub.publish(vp + ".stall_steps", i2s(v.stall_steps));
    pub.publish(vp + ".min_clearance_m", d2s(v.min_clearance_m));
    pub.publish(vp + ".penetration_steps", i2s(v.penetration_steps));
    pub.publish(vp + ".veh_contacts", i2s(v.veh_contacts));
    pub.publish(vp + ".fuel_used", d2s(v.fuel_used));
    pub.publish(vp + ".sense_flips", i2s(v.sense_flips));
    pub.publish(vp + ".formation_parent", i2s(v.formation_parent));
    pub.publish(vp + ".slot_error_mean_m", d2s(v.slot_error_mean_m));
    pub.publish(vp + ".formation_arrived", b2s(v.formation_arrived));
    if (v.energy_tracked) {
      pub.publish(vp + ".energy_J", d2s(v.energy_J));
      pub.publish(vp + ".energy_per_km", d2s(v.energy_per_km));
      pub.publish(vp + ".building_contacts", i2s(v.building_contacts));
    }
    if (v.terrain_tracked)
      pub.publish(vp + ".infeasible_slope_steps", i2s(v.infeasible_slope_steps));
    if (v.plant_tracked) {
      pub.publish(vp + ".rolled_over", b2s(v.rolled_over));
      pub.publish(vp + ".rolled_over_s", d2s(v.rolled_over_s));
      pub.publish(vp + ".tilt_max_deg", d2s(v.tilt_max_deg));
      pub.publish(vp + ".ltr_max", d2s(v.ltr_max));
      pub.publish(vp + ".a_lat_max_g", d2s(v.a_lat_max_g));
      pub.publish(vp + ".steer_sat_share", d2s(v.steer_sat_share));
    }
  }
}

namespace {
// Wrap a [rows*cols] uint8 plane as a z=1 cvc::volume over the world bbox. The raw-pointer ctor
// deep-copies the bytes once (voxels memcpy), so `data` may be reused/freed after; the stored value
// then shallow-shares that fresh buffer (an immutable snapshot — never mutated after publish).
cvc::volume wrap_plane(cvc::app &app, const std::uint8_t *data, const nav_raster_dims &d) {
  return cvc::volume(app, data, cvc::dimension(d.cols, d.rows, 1), cvc::UChar,
                     cvc::bounding_box(d.min_x, d.min_y, 0.0, d.max_x, d.max_y, 1.0));
}
} // namespace

void publish_nav_rasters(cvc::app &app, cvc::gl::state_publisher &pub,
                         const std::string &scenePrefix, const nav_raster_dims &dims,
                         const std::uint8_t *truth, const std::uint8_t *belief,
                         const std::uint8_t *everseen, const std::uint8_t *lastvis,
                         const int *versions, nav_raster_pub_state &st) {
  if (dims.rows <= 0 || dims.cols <= 0 || dims.planes <= 0)
    return;
  const std::string rroot = navStatsStatePath(scenePrefix) + ".rasters";
  // size_t: a fog-of-war twin can carry O(N) planes over a large grid, so plane*cells must not
  // truncate on LLP64 (long is 32-bit on Windows) — matches the int64 discipline used for i2s.
  const std::size_t cells =
      static_cast<std::size_t>(dims.rows) * static_cast<std::size_t>(dims.cols);
  cvc::state &root = cvc::state::instance(app);

  // dims (value lane; cheap, re-published each call so a late subscriber still learns the
  // geometry).
  pub.publish(rroot + ".dims.rows", i2s(dims.rows));
  pub.publish(rroot + ".dims.cols", i2s(dims.cols));
  pub.publish(rroot + ".dims.planes", i2s(dims.planes));
  pub.publish(rroot + ".dims.min_x", d2s(dims.min_x));
  pub.publish(rroot + ".dims.min_y", d2s(dims.min_y));
  pub.publish(rroot + ".dims.max_x", d2s(dims.max_x));
  pub.publish(rroot + ".dims.max_y", d2s(dims.max_y));

  // truth is static — publish its handle once (data() lane), then bump the version to 0.
  if (st.truth_version < 0 && truth) {
    root(rroot + ".truth.data").data(boost::any(wrap_plane(app, truth, dims)));
    pub.publish(rroot + ".truth.version", "0");
    st.truth_version = 0;
  }

  // per-plane belief / ever_seen / last_visible, version-gated (skip the deep wrap for unchanged
  // planes). The "never published" sentinel is INT_MIN, NOT -1: sim_world::plane_version() returns
  // -1 for an out-of-range plane, so a -1 must still be publishable (it differs from INT_MIN)
  // rather than colliding with the sentinel and being frozen out forever.
  if (static_cast<int>(st.plane_versions.size()) != dims.planes)
    st.plane_versions.assign(dims.planes, std::numeric_limits<int>::min());
  // No versions[] -> we cannot tell what changed, so re-publish every plane each call (fresh, never
  // gated) rather than capturing once and freezing the fog.
  const bool gated = versions != nullptr;
  for (int m = 0; m < dims.planes; ++m) {
    const int v = gated ? versions[m] : 0;
    if (gated && v == st.plane_versions[m])
      continue; // unchanged this plane -> no re-wrap, no re-publish
    const std::string pp = rroot + ".plane." + std::to_string(m);
    const std::size_t off = static_cast<std::size_t>(m) * cells;
    if (belief)
      root(pp + ".belief.data").data(boost::any(wrap_plane(app, belief + off, dims)));
    if (everseen)
      root(pp + ".ever_seen.data").data(boost::any(wrap_plane(app, everseen + off, dims)));
    if (lastvis)
      root(pp + ".last_visible.data").data(boost::any(wrap_plane(app, lastvis + off, dims)));
    pub.publish(pp + ".version", i2s(v));
    st.plane_versions[m] = v;
  }
}

void publish_nav_rasters(cvc::app &app, cvc::gl::state_publisher &pub,
                         const std::string &scenePrefix, const cvc::nav::sim_world &world,
                         nav_raster_pub_state &st) {
  const cvc::nav::sim_world::config &cfg = world.current_config();
  nav_raster_dims dims;
  dims.rows = world.rows();
  dims.cols = world.cols();
  dims.planes = world.planes();
  dims.min_x = cfg.min_x;
  dims.min_y = cfg.min_y;
  dims.max_x = cfg.max_x;
  dims.max_y = cfg.max_y;
  if (dims.rows <= 0 || dims.cols <= 0 || dims.planes <= 0)
    return;
  std::vector<int> versions(static_cast<std::size_t>(dims.planes));
  for (int m = 0; m < dims.planes; ++m)
    versions[m] = world.raster_version(m);
  publish_nav_rasters(app, pub, scenePrefix, dims, world.truth(), world.belief_occ(0),
                      world.ever_seen(0), world.last_visible(0), versions.data(), st);
}

} // namespace gl
} // namespace cvc

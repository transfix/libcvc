/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_stats_publish.cpp — see inc/cvc/gl/nav_stats_publish.h.

#include <boost/any.hpp>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/nav_stats_publish.h>
#include <cvc/volume/volume.h>
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

void publish_nav_stats(cvc::gl::state_publisher &pub, const std::string &scenePrefix,
                       const cvc::nav::episode_nav_stats &e, long tick,
                       const nav_stats_publish_params &p) {
  const std::string root = navStatsStatePath(scenePrefix);

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
    // Key by veh_index (the RF-record join key), not array position — identical on a
    // collector-produced record (position == veh_index) but self-consistent with the RF publisher
    // for reordered/sparse ones.
    const std::string vp = root + ".veh." + std::to_string(v.veh_index);
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
  const long cells = static_cast<long>(dims.rows) * dims.cols;
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
  // planes).
  if (static_cast<int>(st.plane_versions.size()) != dims.planes)
    st.plane_versions.assign(dims.planes, -1);
  for (int m = 0; m < dims.planes; ++m) {
    const int v = versions ? versions[m] : 0;
    if (v == st.plane_versions[m])
      continue; // unchanged this plane -> no re-wrap, no re-publish
    const std::string pp = rroot + ".plane." + std::to_string(m);
    const long off = static_cast<long>(m) * cells;
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

} // namespace gl
} // namespace cvc

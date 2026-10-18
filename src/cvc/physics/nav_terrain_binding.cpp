/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "binding_meta.h"

#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/nav_terrain_binding.h>
#include <cvc/state/state.h>
#include <limits>

namespace cvc {
namespace physics {

namespace {

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// What the terrain's keys are (binding_meta.h's form), relative to <prefix>.nav_terrain.
const detail::key_meta kTerrainKeys[] = {
    {"max_grade", "knob", "1", 0.1, 1.5,
     "The tangent of the steepest slope the vehicle climbs (0.6 = 60%, 31 deg); steeper cells are "
     "infeasible. nav_plant::terrain_config_of gives a vehicle's."},
    {"tip_ratio", "knob", "1", 0, 2,
     "The vehicle's static stability factor, its track over twice its centre of gravity's height; "
     "0 = no tip limit."},
    {"tip_margin", "knob", "1", 0.5, 1,
     "The share of the tip slope (the slope whose tangent is tip_ratio) that is feasible."},
    {"sigma", "knob", "cell", 0, 3, "The grade cost's blur before its gradient is taken."},
    {"lam_grade", "knob", "1", 0, 10,
     "The soft grade force's weight: it leans the drive toward gentler ground (H1); 0 = off."},
    {"lam_grade_hard", "knob", "1", 0, 10,
     "The weight of the material-style barrier off infeasible slope (H1); 0 = off."},
    {"k_sharp", "knob", "1/m", 0.25, 4, "The barrier's sharpness."},
    {"d_hat_m", "knob", "m", 2, 30, "How far the barrier reaches."},
    {"block_infeasible", "knob", "", 0, 1,
     "1: infeasible cells plan as obstacles (they join the occupancy)."},
    {"governors", "knob", "", 0, 1,
     "1: the stopping and corner governors take gravity along and across the path (H2)."},
    {"active", "telemetry", "", kNone, kNone, "1 while the world has a terrain."},
    {"gravity", "build", "m/s^2", kNone, kNone, "The gravity the terrain's limits are in."},
    {"limit", "telemetry", "1", kNone, kNone,
     "The tangent of the steepest feasible slope: the lesser of max_grade and tip_margin x "
     "tip_ratio."},
    {"infeasible_cells", "telemetry", "", kNone, kNone, "Grid cells beyond the vehicle's limits."},
};

std::string base_of(const std::string &prefix) {
  return (prefix.empty() ? std::string() : prefix + ".") + "nav_terrain.";
}

// Describe the keys once: a node that has its comment is left alone.
void describe_keys(cvc::app &ctx, const std::string &base) {
  for (const detail::key_meta &m : kTerrainKeys) {
    try {
      if (!cvc::state::instance(ctx)(base + m.key).comment().empty())
        continue;
    } catch (const std::exception &) {
      continue;
    }
    detail::describe(ctx, base + m.key, m);
  }
}

template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

} // namespace

void pull_nav_terrain(cvc::app &ctx, const std::string &prefix, cvc::nav::sim_world &w) {
  if (!w.has_terrain())
    return;
  const std::string k = base_of(prefix);
  describe_keys(ctx, k);
  cvc::nav::terrain_config tc = w.terrain_settings();
  bool changed = false;
  // Read one key, seeding it with `now` when unset; `ok` says whether a value is valid.
  const auto read = [&](const char *name, double now, auto ok, auto apply) {
    std::string raw;
    try {
      raw = cvc::state::instance(ctx)(k + name).value();
    } catch (const std::exception &) {
    }
    if (raw.empty()) {
      write_val(ctx, k + name, now);
      return;
    }
    try {
      // A float setting comes back through its decimal text a little off: only a real change
      // rebuilds the planes.
      const double v = std::stod(raw);
      if (std::fabs(v - now) > 1e-6 * std::fmax(1.0, std::fabs(now)) && ok(v)) {
        apply(v);
        changed = true;
      }
    } catch (const std::exception &) {
      // an invalid value is ignored: the terrain keeps its last good one
    }
  };
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  const auto non_negative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  const auto flag = [](double v) { return v == 0.0 || v == 1.0; };
  read("max_grade", tc.max_grade, positive, [&](double v) { tc.max_grade = v; });
  read("tip_ratio", tc.tip_ratio, non_negative, [&](double v) { tc.tip_ratio = v; });
  read(
      "tip_margin", tc.tip_margin, [](double v) { return v > 0.0 && v <= 1.0; },
      [&](double v) { tc.tip_margin = v; });
  read("sigma", tc.sigma, non_negative, [&](double v) { tc.sigma = v; });
  read("lam_grade", tc.lam_grade, non_negative,
       [&](double v) { tc.lam_grade = static_cast<float>(v); });
  read("lam_grade_hard", tc.lam_grade_hard, non_negative,
       [&](double v) { tc.lam_grade_hard = static_cast<float>(v); });
  read("k_sharp", tc.k_sharp, positive, [&](double v) { tc.k_sharp = static_cast<float>(v); });
  read("d_hat_m", tc.d_hat_m, positive, [&](double v) { tc.d_hat_m = static_cast<float>(v); });
  read("block_infeasible", tc.block_infeasible ? 1.0 : 0.0, flag,
       [&](double v) { tc.block_infeasible = v != 0.0; });
  read("governors", tc.governors ? 1.0 : 0.0, flag, [&](double v) { tc.governors = v != 0.0; });
  if (changed)
    w.set_terrain_config(tc);
}

void publish_nav_terrain(cvc::app &ctx, const std::string &prefix, const cvc::nav::sim_world &w) {
  const std::string k = base_of(prefix);
  describe_keys(ctx, k);
  write_val(ctx, k + "active", w.has_terrain() ? 1 : 0);
  if (!w.has_terrain())
    return;
  const cvc::nav::terrain_planes &t = w.terrain();
  std::int64_t blocked = 0;
  for (const std::uint8_t c : t.infeasible)
    blocked += c != 0;
  write_val(ctx, k + "gravity", w.terrain_settings().gravity);
  write_val(ctx, k + "limit", t.limit);
  write_val(ctx, k + "infeasible_cells", blocked);
}

} // namespace physics
} // namespace cvc

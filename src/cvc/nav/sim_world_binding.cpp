/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// sim_world's settings in the state tree (sim_world::pull / publish). Kept out of sim_world.cpp so
// the state tree's headers (which pull in the platform's) never meet the drive's.

#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key_meta.h>
#include <limits>
#include <string>
#include <utility>

namespace cvc {
namespace nav {

namespace {

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
constexpr double kTwoPi = 6.283185307179586;

// <prefix>.nav: sensing, belief, arrival and the crowd.
const state_key_meta kNavKeys[] = {
    {"range_m", "knob", "m", 5, 300, "The sensor's range."},
    {"n_rays", "knob", "", 16, 1024, "Rays a sense casts per agent."},
    {"fov_rad", "knob", "rad", 0.5, kTwoPi, "The sensor's field of view; 2 pi = all round."},
    {"sense_every", "knob", "", 1, 30, "Ticks between senses (and belief rebuilds)."},
    {"freeze_sense", "knob", "", 0, 1,
     "1: a static, known map; the sense and rebuild are skipped (add_obstacle is then inert)."},
    {"l_occ", "knob", "1", 0.5, 5, "The log-odds a hit adds to a cell."},
    {"l_free", "knob", "1", -5, -0.1, "The log-odds a ray passing through takes from a cell."},
    {"optimistic", "knob", "", 0, 1, "1: unknown space plans as free; 0: as occupied."},
    {"p_thresh", "knob", "1", 0.05, 0.95, "The occupancy probability a cell plans as blocked at."},
    {"band", "knob", "1", 0, 0.5, "The hysteresis band around p_thresh."},
    {"ttl_s", "knob", "s", 0, 60, "How long a dynamic obstacle (add_obstacle) lasts."},
    {"reach_tol", "knob", "m", 0.5, 20, "How close to its goal an agent has arrived (and parks)."},
    {"sep_radius", "knob", "m", 0, 20,
     "Agents within this of each other nudge their carrots apart; 0 = off."},
    {"sep_gain", "knob", "1", 0, 5, "How hard the carrots are nudged apart; 0 = off."},
    {"min_gap", "knob", "m", 0, 10,
     "The hard de-overlap: no two agents' centres closer than this; 0 = off (skipped with a "
     "plant, whose bodies collide)."},
    {"min_gap_iters", "knob", "", 1, 16, "The de-overlap's relaxation sweeps a tick."},
    {"tick", "telemetry", "", kNone, kNone, "Ticks stepped."},
};

// <prefix>.nav.vehicle: veh_params' scalars.
const state_key_meta kVehicleKeys[] = {
    {"vmax", "knob", "m/s", 0.5, 40, "The top speed."},
    {"a_max", "knob", "m/s^2", 0.5, 15,
     "The acceleration and braking the drive commands and its stopping governor plans with."},
    {"a_lat_max", "knob", "m/s^2", 0.5, 15, "The corner cap's lateral acceleration."},
    {"wheelbase", "knob", "m", 0.2, 8, "The bicycle's wheelbase, L."},
    {"delta_max", "knob", "rad", 0.1, 1, "The bicycle's steering lock."},
    {"track_width", "knob", "m", 0, 3,
     "The track; the inner wheel's lock then limits the bicycle's angle. 0 = no track."},
    {"k_steer", "knob", "1", 0, 2, "How much the barrier and material forces steer."},
    {"nsub", "knob", "", 1, 16, "Integration substeps a tick."},
    {"allow_reverse", "knob", "", 0, 1, "1: the drive may back up out of a dead end."},
    {"rr", "knob", "m", 0.1, 5, "The single-disc footprint's radius (at the rear axle)."},
    {"d_hat", "knob", "m", 0.5, 30, "How far from a wall its barrier reaches."},
    {"body_rr", "knob", "m", 0, 5, "The multi-disc footprint's disc radius."},
    {"body_gain", "knob", "1", 0, 1,
     "The summed barrier's scale; 1 / discs cancels the sum's gain on the learned alpha."},
    {"mass", "knob", "kg", 1, 20000,
     "Carried for the fuel model and telemetry; the kinematic drive does not read it."},
    {"mu_lookahead", "knob", "m", 0, 40,
     "How far ahead the grip feature probes; must match the net's training."},
    {"mu_probes", "knob", "", 1, 8, "Probes along the grip feature's look-ahead."},
    {"risk_governor", "knob", "", 0, 1,
     "1: material risk (the worse of the material's and the terrain's) caps the speed."},
    {"risk_vmin", "knob", "m/s", 0, 40, "The risk governor's speed at risk 1."},
    {"risk_onset", "knob", "1", 0, 0.95, "The risk below which the governor does not slow."},
    {"risk_lookahead", "knob", "m", 0, 80,
     "How far ahead the risk governor probes, so the vehicle slows before it gets there."},
    {"risk_probes", "knob", "", 1, 8, "Probes along the risk governor's look-ahead."},
};

// <prefix>.nav.material: the material force's weights.
const state_key_meta kMaterialKeys[] = {
    {"lam_soft", "knob", "1", 0, 10,
     "The soft-risk force's weight (a net with a lam head overrides it per agent)."},
    {"lam_hard", "knob", "1", 0, 10, "The hard-hazard barrier's weight."},
    {"lam_soft_scale", "knob", "1", 0, 4,
     "A multiplier on the soft weight, fixed or learned: the lever for a force-bias control."},
    {"lam_hard_scale", "knob", "1", 0, 4, "A multiplier on the hard weight, fixed or learned."},
};

// <prefix>.nav.yield: agents out of each other's way (set_yield, nav/yield.h).
const state_key_meta kYieldKeys[] = {
    {"horizon_s", "knob", "s", 0, 10,
     "How far ahead intents reach and conflicts count; 0 = yielding off."},
    {"clearance", "knob", "m", 0, 30,
     "The corridor's half-width round a path; 0 = 3 x the vehicle's rr."},
    {"gain", "knob", "1", 0, 4,
     "How far an agent moves out of another's path, per unit intrusion."},
    {"pass_gain", "knob", "1", 0, 4, "How far it steers round one in its own path."},
    {"cooperation", "knob", "1", 0, 1, "How far a passer counts on the other moving aside."},
    {"min_len", "knob", "m", 0, 100, "An intent's shortest path; 0 = 4 x clearance."},
    {"min_speed", "knob", "m/s", 0, 10, "The speed floor arrival times are taken at."},
    {"near_dist", "knob", "m", -1, 100,
     "Along a path this close a conflict counts at any speed; 0 = 4 x clearance, < 0 = off."},
    {"steer_overrides", "knob", "", 0, 1,
     "1: a forward override_command's steer (a route's pursuit) takes the shift too."},
    {"steer_look_s", "knob", "s", 0, 5, "Seconds of speed the override correction looks ahead."},
    {"pass_time_s", "knob", "s", 0, 10,
     "The passing cap: no sooner at one in the path than this; it stops short of one that stays. "
     "0 = off."},
    {"pass_hold_s", "knob", "s", 0, 30,
     "Held this long behind one standing still, the hold is released; 0 = never (the default)."},
    {"creep_speed", "knob", "m/s", 0, 5,
     "An agent asked to move aside from a standstill creeps forward at this; 0 = no creep."},
    {"creep_need", "knob", "m", 0, 30, "The shift out of a path that calls for the creep; 0 = rr."},
};

// <prefix>.nav.terrain (P5).
const state_key_meta kTerrainKeys[] = {
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
     "1: infeasible cells also plan as obstacles (they join the occupancy); 0 (the default): "
     "slope is a material risk only (lam_grade, lam_grade_hard)."},
    {"governors", "knob", "", 0, 1,
     "1: the stopping and corner governors take gravity along and across the path (H2)."},
    {"gravity", "build", "m/s^2", kNone, kNone, "The gravity the terrain's limits are in."},
    {"limit", "telemetry", "1", kNone, kNone,
     "The tangent of the steepest feasible slope: the lesser of max_grade and tip_margin x "
     "tip_ratio."},
    {"infeasible_cells", "telemetry", "", kNone, kNone, "Grid cells beyond the vehicle's limits."},
};
const state_key_meta kTerrainActive[] = {
    {"active", "telemetry", "", kNone, kNone, "1 while the world has a terrain."},
};

// <prefix>.nav.settings: fixed when the world is built.
const state_key_meta kSettingsKeys[] = {
    {"rows", "build", "", kNone, kNone, "The grid's rows."},
    {"cols", "build", "", kNone, kNone, "The grid's columns."},
    {"min_x", "build", "m", kNone, kNone, "The grid's west edge."},
    {"min_y", "build", "m", kNone, kNone, "The grid's south edge."},
    {"max_x", "build", "m", kNone, kNone, "The grid's east edge."},
    {"max_y", "build", "m", kNone, kNone, "The grid's north edge."},
    {"cx", "build", "m", kNone, kNone, "The normalized frame's centre, x."},
    {"cy", "build", "m", kNone, kNone, "The normalized frame's centre, y."},
    {"scale", "build", "1/m", kNone, kNone, "Normalized units per metre (S)."},
    {"l_clamp", "build", "1", kNone, kNone, "The belief's log-odds clamp."},
    {"dt", "build", "s", kNone, kNone, "Seconds per tick."},
    {"agents", "build", "", kNone, kNone, "Agents in the world."},
    {"planes", "build", "", kNone, kNone, "Belief planes: 1 shared, one per group, or per agent."},
    {"plant", "build", nullptr, kNone, kNone,
     "What integrates the agents: kinematic (nav's bicycle) or jolt (a physics plant)."},
};

std::string join(const std::string &prefix, const char *rest) {
  return (prefix.empty() ? std::string() : prefix + ".") + rest;
}

template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

// Reads a group's keys under `base`: each is seeded with `now` when unset; a valid value that
// differs (beyond its decimal text's rounding) is applied.
class reader {
public:
  reader(cvc::app &ctx, std::string base) : ctx_(ctx), base_(std::move(base)) {}

  template <class Ok, class Apply> bool read(const char *key, double now, Ok ok, Apply apply) {
    const std::string path = base_ + "." + key;
    std::string raw;
    try {
      raw = cvc::state::instance(ctx_)(path).value();
    } catch (const std::exception &) {
    }
    if (raw.empty()) {
      write_val(ctx_, path, now);
      return false;
    }
    try {
      const double v = std::stod(raw);
      if (std::fabs(v - now) > 1e-6 * std::fmax(1.0, std::fabs(now)) && ok(v)) {
        apply(v);
        return true;
      }
    } catch (const std::exception &) {
      // an invalid value is ignored: the world keeps its last good one
    }
    return false;
  }

private:
  cvc::app &ctx_;
  std::string base_;
};

bool positive(double v) { return std::isfinite(v) && v > 0.0; }
bool non_negative(double v) { return std::isfinite(v) && v >= 0.0; }
bool flag(double v) { return v == 0.0 || v == 1.0; }
bool whole(double v) { return v >= 1.0 && v <= 1e6 && v == std::floor(v); }
double b2d(bool b) { return b ? 1.0 : 0.0; }

const char *plant_name(const vehicle_plant *p) {
  if (!p)
    return "kinematic";
  switch (p->kind()) {
  case plant_kind::jolt:
    return "jolt";
  default:
    return "kinematic";
  }
}

} // namespace

void sim_world::pull(cvc::app &ctx, const std::string &prefix) {
  const std::string base = join(prefix, "nav");
  const double S = cfg_.scale;
  const double L = 1.0 / S; // world metres per normalized unit
  {
    describe_state_keys(ctx, base, kNavKeys, true);
    reader r(ctx, base);
    config &c = cfg_;
    r.read("range_m", c.range_m, positive, [&](double v) { c.range_m = v; });
    r.read("n_rays", c.n_rays, whole, [&](double v) { c.n_rays = static_cast<int>(v); });
    r.read(
        "fov_rad", c.fov_rad, [](double v) { return v > 0.0 && v <= kTwoPi; },
        [&](double v) { c.fov_rad = v; });
    r.read("sense_every", c.sense_every, whole,
           [&](double v) { c.sense_every = static_cast<int>(v); });
    r.read("freeze_sense", b2d(c.freeze_sense), flag, [&](double v) { c.freeze_sense = v != 0.0; });
    r.read("l_occ", c.l_occ, positive, [&](double v) { c.l_occ = v; });
    r.read(
        "l_free", c.l_free, [](double v) { return std::isfinite(v) && v < 0.0; },
        [&](double v) { c.l_free = v; });
    r.read("optimistic", b2d(c.optimistic), flag, [&](double v) { c.optimistic = v != 0.0; });
    r.read(
        "p_thresh", c.p_thresh, [](double v) { return v > 0.0 && v < 1.0; },
        [&](double v) { c.p_thresh = v; });
    r.read("band", c.band, non_negative, [&](double v) { c.band = v; });
    r.read("ttl_s", c.ttl_s, non_negative, [&](double v) { c.ttl_s = v; });
    r.read("reach_tol", c.reach_tol * L, positive,
           [&](double v) { c.reach_tol = static_cast<float>(v * S); });
    r.read("sep_radius", c.sep_radius * L, non_negative,
           [&](double v) { c.sep_radius = static_cast<float>(v * S); });
    r.read("sep_gain", c.sep_gain, non_negative,
           [&](double v) { c.sep_gain = static_cast<float>(v); });
    r.read("min_gap", c.min_gap * L, non_negative,
           [&](double v) { c.min_gap = static_cast<float>(v * S); });
    r.read("min_gap_iters", c.min_gap_iters, whole,
           [&](double v) { c.min_gap_iters = static_cast<int>(v); });
  }
  {
    const std::string vb = base + ".vehicle";
    describe_state_keys(ctx, vb, kVehicleKeys, true);
    reader r(ctx, vb);
    veh_params &v = cfg_.veh;
    const auto scaled = [&](const char *key, float &f, double unit, bool (*ok)(double)) {
      r.read(key, f * unit, ok, [&](double x) { f = static_cast<float>(x / unit); });
    };
    scaled("vmax", v.vmax, L, positive);
    scaled("a_max", v.a_max, L, positive);
    scaled("a_lat_max", v.a_lat_max, L, positive);
    scaled("wheelbase", v.L, L, positive);
    r.read(
        "delta_max", v.delta_max, [](double x) { return x > 0.0 && x < 1.5707963267948966; },
        [&](double x) { v.delta_max = static_cast<float>(x); });
    scaled("track_width", v.track_width, L, non_negative);
    scaled("k_steer", v.k_steer, 1.0, non_negative);
    r.read("nsub", v.nsub, whole, [&](double x) { v.nsub = static_cast<int>(x); });
    r.read("allow_reverse", b2d(v.allow_reverse), flag,
           [&](double x) { v.allow_reverse = x != 0.0; });
    scaled("rr", v.rr, L, non_negative);
    scaled("d_hat", v.d_hat, L, non_negative);
    scaled("body_rr", v.body_rr, L, non_negative);
    scaled("body_gain", v.body_gain, 1.0, non_negative);
    scaled("mass", v.mass, 1.0, positive);
    scaled("mu_lookahead", v.mu_lookahead, L, non_negative);
    r.read("mu_probes", v.mu_probes, whole, [&](double x) { v.mu_probes = static_cast<int>(x); });
    r.read("risk_governor", b2d(v.risk_governor), flag,
           [&](double x) { v.risk_governor = x != 0.0; });
    scaled("risk_vmin", v.risk_vmin, L, non_negative);
    r.read(
        "risk_onset", v.risk_onset, [](double x) { return x >= 0.0 && x < 1.0; },
        [&](double x) { v.risk_onset = static_cast<float>(x); });
    scaled("risk_lookahead", v.risk_lookahead, L, non_negative);
    r.read("risk_probes", v.risk_probes, whole,
           [&](double x) { v.risk_probes = static_cast<int>(x); });
  }
  {
    const std::string yb = base + ".yield";
    describe_state_keys(ctx, yb, kYieldKeys, true);
    reader r(ctx, yb);
    yield_config &y = yield_cfg_;
    const auto scaled = [&](const char *key, float &f, double unit, bool (*ok)(double)) {
      r.read(key, f * unit, ok, [&](double x) { f = static_cast<float>(x / unit); });
    };
    scaled("horizon_s", y.horizon_s, 1.0, non_negative);
    scaled("clearance", y.clearance, L, non_negative);
    scaled("gain", y.gain, 1.0, non_negative);
    scaled("pass_gain", y.pass_gain, 1.0, non_negative);
    r.read(
        "cooperation", y.cooperation, [](double x) { return x >= 0.0 && x <= 1.0; },
        [&](double x) { y.cooperation = static_cast<float>(x); });
    scaled("min_len", y.min_len, L, non_negative);
    scaled("min_speed", y.min_speed, L, non_negative);
    r.read(
        "near_dist", y.near_dist * L, [](double x) { return std::isfinite(x); },
        [&](double x) { y.near_dist = static_cast<float>(x * S); });
    r.read("steer_overrides", b2d(y.steer_overrides), flag,
           [&](double x) { y.steer_overrides = x != 0.0; });
    scaled("steer_look_s", y.steer_look_s, 1.0, non_negative);
    scaled("pass_time_s", y.pass_time_s, 1.0, non_negative);
    scaled("pass_hold_s", y.pass_hold_s, 1.0, non_negative);
    scaled("creep_speed", y.creep_speed, L, non_negative);
    scaled("creep_need", y.creep_need, L, non_negative);
  }
  if (mat_on_) {
    const std::string mb = base + ".material";
    describe_state_keys(ctx, mb, kMaterialKeys, true);
    reader r(ctx, mb);
    material_config &m = mat_cfg_;
    const auto weight = [&](const char *key, float &f) {
      r.read(key, f, non_negative, [&](double x) { f = static_cast<float>(x); });
    };
    weight("lam_soft", m.lam_soft);
    weight("lam_hard", m.lam_hard);
    weight("lam_soft_scale", m.lam_soft_scale);
    weight("lam_hard_scale", m.lam_hard_scale);
  }
  if (terrain_on_) {
    const std::string tb = base + ".terrain";
    describe_state_keys(ctx, tb, kTerrainKeys, true);
    reader r(ctx, tb);
    terrain_config tc = ter_cfg_;
    bool changed = false;
    const auto value = [&](const char *key, double &d, bool (*ok)(double)) {
      changed |= r.read(key, d, ok, [&](double x) { d = x; });
    };
    const auto weight = [&](const char *key, float &f, bool (*ok)(double)) {
      changed |= r.read(key, f, ok, [&](double x) { f = static_cast<float>(x); });
    };
    const auto toggle = [&](const char *key, bool &b) {
      changed |= r.read(key, b2d(b), flag, [&](double x) { b = x != 0.0; });
    };
    value("max_grade", tc.max_grade, positive);
    value("tip_ratio", tc.tip_ratio, non_negative);
    changed |= r.read(
        "tip_margin", tc.tip_margin, [](double x) { return x > 0.0 && x <= 1.0; },
        [&](double x) { tc.tip_margin = x; });
    value("sigma", tc.sigma, non_negative);
    weight("lam_grade", tc.lam_grade, non_negative);
    weight("lam_grade_hard", tc.lam_grade_hard, non_negative);
    weight("k_sharp", tc.k_sharp, positive);
    weight("d_hat_m", tc.d_hat_m, positive);
    toggle("block_infeasible", tc.block_infeasible);
    toggle("governors", tc.governors);
    if (changed)
      set_terrain_config(tc);
  }
}

void sim_world::publish(cvc::app &ctx, const std::string &prefix) const {
  const std::string base = join(prefix, "nav");
  describe_state_keys(ctx, base, kNavKeys, true);
  write_val(ctx, base + ".tick", static_cast<std::int64_t>(gstep_));
  {
    const std::string sb = base + ".settings";
    describe_state_keys(ctx, sb, kSettingsKeys, true);
    write_val(ctx, sb + ".rows", rows_);
    write_val(ctx, sb + ".cols", cols_);
    write_val(ctx, sb + ".min_x", cfg_.min_x);
    write_val(ctx, sb + ".min_y", cfg_.min_y);
    write_val(ctx, sb + ".max_x", cfg_.max_x);
    write_val(ctx, sb + ".max_y", cfg_.max_y);
    write_val(ctx, sb + ".cx", cfg_.cx);
    write_val(ctx, sb + ".cy", cfg_.cy);
    write_val(ctx, sb + ".scale", cfg_.scale);
    write_val(ctx, sb + ".l_clamp", cfg_.l_clamp);
    write_val(ctx, sb + ".dt", static_cast<double>(cfg_.veh.dt));
    write_val(ctx, sb + ".agents", n_);
    write_val(ctx, sb + ".planes", M_);
    write_val(ctx, sb + ".plant", std::string(plant_name(plant_)));
  }
  const std::string tb = base + ".terrain";
  describe_state_keys(ctx, tb, kTerrainActive, true);
  write_val(ctx, tb + ".active", terrain_on_ ? 1 : 0);
  if (terrain_on_) {
    describe_state_keys(ctx, tb, kTerrainKeys, true);
    std::int64_t blocked = 0;
    for (const std::uint8_t c : ter_.infeasible)
      blocked += c != 0;
    write_val(ctx, tb + ".gravity", ter_cfg_.gravity);
    write_val(ctx, tb + ".limit", ter_.limit);
    write_val(ctx, tb + ".infeasible_cells", blocked);
  }
}

} // namespace nav
} // namespace cvc

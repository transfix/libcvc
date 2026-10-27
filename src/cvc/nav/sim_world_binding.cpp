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
    {"range_m", "knob", "m", 5, 300,
     "How far each agent's sensor sees when it maps its surroundings. Higher: maps more per "
     "look, at more compute. Default 60."},
    {"n_rays", "knob", "", 16, 1024,
     "Rays each agent casts per look, spread across its field of view. Higher: finer coverage, "
     "fewer thin obstacles missed, at more compute. Default 240."},
    {"fov_rad", "knob", "rad", 0.5, kTwoPi,
     "The sensor's field of view, centred on the heading; 2 pi (6.28) sees all round. "
     "Default 2 pi."},
    {"sense_every", "knob", "", 1, 30,
     "Ticks (one sim step each) between looks; the obstacle map is rebuilt after each. Higher: "
     "cheaper, but slower to notice a change. Default 4."},
    {"freeze_sense", "knob", "", 0, 1,
     "1: the map is static and known, so sensing and map rebuilds are skipped and obstacles "
     "added at run time never appear; 0: agents sense and map as they go. Default 0."},
    {"l_occ", "knob", "1", 0.5, 5,
     "How much one ray hit raises a cell's log-odds of being occupied. Higher: a cell becomes an "
     "obstacle after fewer hits, and false hits stick more. Default 2.2."},
    {"l_free", "knob", "1", -5, -0.1,
     "How much a ray passing through a cell lowers its log-odds of being occupied (a negative "
     "number). More negative: cells clear faster. Default -1.4."},
    {"optimistic", "knob", "", 0, 1,
     "1: unseen space plans as free (drive in, replan on discovery); 0: unseen space plans as "
     "occupied. Default 1."},
    {"p_thresh", "knob", "1", 0.05, 0.95,
     "The occupancy probability above which a cell plans as an obstacle (with optimistic 0, a "
     "cell plans as free below 1 - p_thresh). Lower: more cautious. Default 0.5."},
    {"band", "knob", "1", 0, 0.5,
     "Cells whose occupancy probability is within this of 0.5 count as unknown and follow "
     "optimistic, whatever p_thresh says. Higher: more evidence needed to decide. "
     "Default 0.15."},
    {"ttl_s", "knob", "s", 0, 60,
     "How long an obstacle added at run time stays on the map. Default 4."},
    {"reach_tol", "knob", "m", 0.5, 20,
     "How close to its goal an agent counts as arrived; it then stops and parks."},
    {"sep_radius", "knob", "m", 0, 20,
     "Agents closer than this push their carrots (the points they steer toward) apart, so a "
     "crowd steers round itself. Needs sep_gain too. 0: off. Default 0."},
    {"sep_gain", "knob", "m", 0, 10,
     "How hard nearby agents push each other's carrots apart: how far a neighbour at contact "
     "shifts an agent's carrot, falling off to nothing at sep_radius. Higher: wider spacing, more "
     "weaving. 0: off. Default 0."},
    {"min_gap", "knob", "m", 0, 10,
     "The closest two agents' centres may come: after each drive, closer pairs are pushed "
     "apart to this (never into a wall). Skipped on a physics plant, whose bodies collide. "
     "0: off. Default 0."},
    {"min_gap_iters", "knob", "", 1, 16,
     "Push-apart passes per tick for min_gap. More: a long column settles fully, at more "
     "compute. Default 4."},
    {"tick", "telemetry", "", kNone, kNone,
     "Ticks (one sim step each) stepped since the world was built."},
};

// <prefix>.nav.vehicle: veh_params' scalars.
const state_key_meta kVehicleKeys[] = {
    {"vmax", "knob", "m/s", 0.5, 40,
     "The top speed. On a physics plant the vehicle drives at no more than the lower of this "
     "and its own top speed."},
    {"a_max", "knob", "m/s^2", 0.5, 15,
     "The acceleration and braking the drive commands, and the braking its stopping speed limit "
     "plans with. Higher: livelier, but brakes later."},
    {"a_lat_max", "knob", "m/s^2", 0.5, 15,
     "The sideways acceleration the corner speed limit allows. Higher: faster through bends, "
     "less margin before sliding."},
    {"wheelbase", "knob", "m", 0.2, 8,
     "The distance between the axles in nav's bicycle model; with the steering lock it sets the "
     "tightest turn. A physics plant uses each vehicle's own."},
    {"delta_max", "knob", "rad", 0.1, 1,
     "The steering lock: the largest angle the bicycle model's front wheel turns. Higher: "
     "tighter turns. A physics plant's own lock caps it. Default 0.6."},
    {"track_width", "knob", "m", 0, 3,
     "The distance between the left and right wheels. The inner wheel then reaches the lock "
     "first, so the vehicle turns less tightly than delta_max allows. 0: no track. Default 0."},
    {"k_steer", "knob", "rad", 0, 2,
     "How much the push from walls, material and terrain adds to the steering: the most it can "
     "add, in radians. Higher: swerves harder away. Default 0.8."},
    {"nsub", "knob", "", 1, 16,
     "Integration substeps per tick for the kinematic drive. More: more accurate at speed, at "
     "more compute. A physics plant ignores it. Default 1."},
    {"allow_reverse", "knob", "", 0, 1,
     "1: the drive may back up, at up to a quarter of vmax, for example out of a dead end; "
     "0: forward only. Default 1."},
    {"rr", "knob", "m", 0.1, 5,
     "The radius of the single-disc footprint, centred on the rear axle, that clearances to "
     "walls are measured from. A multi-disc footprint (body_rr) replaces it."},
    {"d_hat", "knob", "m", 0.5, 30,
     "How far from a wall the barrier that pushes the vehicle away reaches; beyond it a wall "
     "exerts no push. Higher: a wider berth, but more hesitation in narrow gaps."},
    {"body_rr", "knob", "m", 0, 5,
     "The disc radius of the multi-disc footprint (discs along the body), used in place of rr "
     "when the app sets one up. A physics plant that knows its vehicles' size sets its own. "
     "Default 0."},
    {"body_gain", "knob", "1", 0, 1,
     "A scale on the multi-disc footprint's summed wall barrier. Set it to 1 / the number of "
     "discs so the steering policy feels the push it was trained with; at 1 the vehicle is "
     "timid. Default 1."},
    {"mass", "knob", "kg", 1, 20000,
     "The vehicle's mass, carried for the fuel model and telemetry; the kinematic drive does "
     "not use it. Default 1."},
    {"mu_lookahead", "knob", "m", 0, 40,
     "How far toward its carrot (the point it steers toward) a vehicle looks for the worst "
     "grip, for a grip-aware steering policy. Must match the policy's training."},
    {"mu_probes", "knob", "", 1, 8,
     "Grip samples along mu_lookahead. Must match the policy's training. Default 3."},
    {"risk_governor", "knob", "", 0, 1,
     "1: material risk (the worse of the material's and the terrain's) caps the speed, so "
     "vehicles slow on and before risky ground; 0: risk only steers. Default 0."},
    {"risk_vmin", "knob", "m/s", 0, 40,
     "The risk governor's speed at risk 1. From risk_onset up to risk 1 the cap falls in a "
     "straight line from vmax to this. Default 0."},
    {"risk_onset", "knob", "1", 0, 0.95,
     "The risk below which the risk governor does not slow the vehicle. Higher: slows only on "
     "riskier ground. Default 0.3."},
    {"risk_lookahead", "knob", "m", 0, 80,
     "How far ahead along its travel the risk governor looks, so the vehicle slows before it "
     "reaches risky ground. 0: underfoot only. Default 0."},
    {"risk_probes", "knob", "", 1, 8,
     "Risk samples along risk_lookahead. More: catches narrower risky patches. Default 3."},
};

// <prefix>.nav.material: the material force's weights.
const state_key_meta kMaterialKeys[] = {
    {"lam_soft", "knob", "m^2/s^2", 0, 4000,
     "The weight of the soft force that steers vehicles off risky ground: the push, in m/s^2, "
     "for each unit of risk gained per metre. Higher: wider detours round risk. A steering "
     "policy that outputs its own weight replaces it per vehicle. 0: off."},
    {"lam_hard", "knob", "m/s^2", 0, 200,
     "The weight of the barrier that keeps vehicles off hazardous material: its push at full "
     "strength. Higher: they keep further off it."},
    {"lam_soft_scale", "knob", "1", 0, 4,
     "A multiplier on the soft weight, whether fixed (lam_soft) or output by a trained policy, "
     "so it turns a trained policy's risk avoidance up or down too. Default 1."},
    {"lam_hard_scale", "knob", "1", 0, 4,
     "A multiplier on the hard weight, whether fixed (lam_hard) or output by a trained policy. "
     "Default 1."},
};

// <prefix>.nav.yield: agents out of each other's way (set_yield, nav/yield.h).
const state_key_meta kYieldKeys[] = {
    {"horizon_s", "knob", "s", 0, 10,
     "How far ahead agents announce the paths they mean to drive, and how soon a conflict on "
     "one must come to count. Higher: they make way earlier. 0: yielding off. Default 0."},
    {"clearance", "knob", "m", 0, 30,
     "The half-width of the corridor round an announced path that others move out of. "
     "Higher: they make more room. 0: 3 x the vehicle's rr. Default 0."},
    {"gain", "knob", "1", 0, 4,
     "How far an agent shifts its carrot (the point it steers toward) out of another's path, "
     "per metre it is inside that path's corridor. Higher: pulls aside further. Default 1."},
    {"pass_gain", "knob", "1", 0, 4,
     "How far an agent shifts its carrot to steer round another in its own path, per metre "
     "that one is inside its corridor. Higher: passes wider. Default 1."},
    {"cooperation", "knob", "1", 0, 1,
     "How far an agent passing another counts on that one moving aside. 0: it steers round "
     "alone; 1: it leaves the other its full share. Default 0.5."},
    {"min_len", "knob", "m", 0, 100,
     "The shortest path an agent announces, however slowly it moves. 0: 4 x clearance. "
     "Default 0."},
    {"min_speed", "knob", "m/s", 0, 10,
     "The speed below which arrival times are worked out as if at this speed, so a slow or "
     "stopped agent's conflicts still count."},
    {"near_dist", "knob", "m", -1, 100,
     "Along a path this close, a conflict counts whatever the speeds, so a stopped column still "
     "clears for one starting behind it. 0: 4 x clearance; negative: off (arrival times only). "
     "Default 0."},
    {"steer_overrides", "knob", "", 0, 1,
     "1: an agent the host steers itself (on a route, for example) also steers toward the "
     "yielding shift while driving forward; 0: the host's steer is left as commanded. "
     "Default 1."},
    {"steer_look_s", "knob", "s", 0, 5,
     "For an agent the host steers: how far ahead, in seconds at its speed, the yielding "
     "correction aims. Shorter: sharper correction. Default 0.5."},
    {"pass_time_s", "knob", "s", 0, 10,
     "Speed cap with another in its path: an agent drives no faster than would bring it to "
     "that one in this time, and stops short of one that stays put. Higher: more cautious. "
     "0: off. Default 1.5."},
    {"pass_hold_s", "knob", "s", 0, 30,
     "How long an agent held behind one standing still waits before it is released to drive "
     "on (the higher priority first); released, it may push into the one ahead. 0: never. "
     "Default 0."},
    {"creep_speed", "knob", "m/s", 0, 5,
     "An agent that must move out of another's path but is stopped or held creeps forward at "
     "this speed so it can steer aside. 0: no creep. Default 0."},
    {"creep_need", "knob", "m", 0, 30,
     "The smallest shift out of another's path that makes a stopped or held agent creep "
     "(creep_speed). 0: the vehicle's rr. Default 0."},
};

// <prefix>.nav.terrain (P5).
const state_key_meta kTerrainKeys[] = {
    {"max_grade", "knob", "1", 0.1, 1.5,
     "The steepest slope the vehicle can climb, as rise over run (0.6 = 60%, 31 degrees); "
     "steeper cells are infeasible. An app can derive it from the vehicle's tires and gearing. "
     "Default 0.6."},
    {"tip_ratio", "knob", "1", 0, 2,
     "The vehicle's static stability factor: its track over twice its centre of gravity's "
     "height. Slopes it could roll over on become infeasible. 0: no rollover limit. Default 0."},
    {"tip_margin", "knob", "1", 0.5, 1,
     "The share of the rollover slope (the slope whose rise over run is tip_ratio) that counts "
     "as feasible. Lower: more margin against rolling, less ground open. Default 0.8."},
    {"sigma", "knob", "m", 0, 50,
     "How far the grade cost (how near each cell's slope is to the limit) is blurred before the "
     "force takes its gradient. Higher: a smoother push that starts further out. Default: one "
     "map cell."},
    {"lam_grade", "knob", "m^2/s^2", 0, 4000,
     "The weight of the soft force that leans the drive toward gentler ground: the push, in "
     "m/s^2, for each unit of grade cost gained per metre. Higher: a stronger preference for flat "
     "ground, at the cost of longer paths. 0: off."},
    {"lam_grade_hard", "knob", "m/s^2", 0, 200,
     "The weight of the barrier that pushes vehicles off infeasible slope: its push at full "
     "strength. Unlike a wall, a vehicle sent across such a cell can still cross it. Higher: "
     "keeps further off. 0: off."},
    {"k_sharp", "knob", "1/m", 0.25, 4,
     "How sharply the infeasible-slope barrier rises around d_hat_m. Higher: a more sudden, "
     "wall-like edge; lower: a gradual push. Default 1.25."},
    {"d_hat_m", "knob", "m", 2, 30,
     "The distance from infeasible slope at which its barrier is at half strength. Higher: "
     "vehicles start keeping off from further away. Default 12."},
    {"block_infeasible", "knob", "", 0, 1,
     "1: infeasible cells also plan as obstacles, like walls; 0: slope is a material risk only "
     "(lam_grade, lam_grade_hard). Default 0."},
    {"governors", "knob", "", 0, 1,
     "1: the stopping and corner speed limits allow for gravity along and across the slope "
     "(less braking downhill); 0: they plan as on flat ground. Default 1."},
    {"gravity", "build", "m/s^2", kNone, kNone,
     "The gravity the slope-aware speed limits (governors) use. Default 9.80665."},
    {"limit", "telemetry", "1", kNone, kNone,
     "The steepest feasible slope, as rise over run: the lesser of max_grade and tip_margin x "
     "tip_ratio (max_grade alone when tip_ratio is 0)."},
    {"infeasible_cells", "telemetry", "", kNone, kNone,
     "Grid cells steeper than the vehicle's limit."},
};
const state_key_meta kTerrainActive[] = {
    {"active", "telemetry", "", kNone, kNone, "1: the world has a terrain; 0: it is flat."},
};

// <prefix>.nav.settings: fixed when the world is built.
const state_key_meta kSettingsKeys[] = {
    {"rows", "build", "", kNone, kNone, "Rows of the map grid (along y)."},
    {"cols", "build", "", kNone, kNone, "Columns of the map grid (along x)."},
    {"min_x", "build", "m", kNone, kNone, "The map grid's west edge (its smallest x)."},
    {"min_y", "build", "m", kNone, kNone, "The map grid's south edge (its smallest y)."},
    {"max_x", "build", "m", kNone, kNone, "The map grid's east edge (its largest x)."},
    {"max_y", "build", "m", kNone, kNone, "The map grid's north edge (its largest y)."},
    {"cx", "build", "m", kNone, kNone,
     "The x of the world point nav's internal normalized frame is centred on."},
    {"cy", "build", "m", kNone, kNone,
     "The y of the world point nav's internal normalized frame is centred on."},
    {"scale", "build", "1/m", kNone, kNone,
     "Nav's internal normalized units per metre. The tree shows metres; a library default "
     "given in normalized units is divided by this."},
    {"l_clamp", "build", "1", kNone, kNone,
     "The most a cell's log-odds of being occupied can reach either way; cells the prior map "
     "marks start at plus this, the rest at minus this."},
    {"dt", "build", "s", kNone, kNone, "Seconds of world time per tick (one sim step)."},
    {"agents", "build", "", kNone, kNone, "Agents (vehicles) in the world."},
    {"planes", "build", "", kNone, kNone,
     "Belief maps: 1 shared by every agent, one per group, or one per agent."},
    {"plant", "build", nullptr, kNone, kNone,
     "What moves the agents: kinematic (nav's own bicycle model) or jolt (a physics engine's "
     "vehicles)."},
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
  case plant_kind::dynamic:
    return "dynamic";
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
    r.read("sep_gain", c.sep_gain * L, non_negative,
           [&](double v) { c.sep_gain = static_cast<float>(v * S); });
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
    // unit: world per normalized. The soft force's weight multiplies a risk gradient per
    // normalized unit into a normalized acceleration (L^2 to the world's m^2/s^2); the barrier's
    // multiplies a unit direction (L to m/s^2). The scales are plain multipliers.
    const auto weight = [&](const char *key, float &f, double unit) {
      r.read(key, f * unit, non_negative, [&](double x) { f = static_cast<float>(x / unit); });
    };
    weight("lam_soft", m.lam_soft, L * L);
    weight("lam_hard", m.lam_hard, L);
    weight("lam_soft_scale", m.lam_soft_scale, 1.0);
    weight("lam_hard_scale", m.lam_hard_scale, 1.0);
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
    const auto weight = [&](const char *key, float &f, double unit, bool (*ok)(double)) {
      changed |= r.read(key, f * unit, ok, [&](double x) { f = static_cast<float>(x / unit); });
    };
    const auto toggle = [&](const char *key, bool &b) {
      changed |= r.read(key, b2d(b), flag, [&](double x) { b = x != 0.0; });
    };
    value("max_grade", tc.max_grade, positive);
    value("tip_ratio", tc.tip_ratio, non_negative);
    changed |= r.read(
        "tip_margin", tc.tip_margin, [](double x) { return x > 0.0 && x <= 1.0; },
        [&](double x) { tc.tip_margin = x; });
    // The blur is kept in cells (material_build); the tree has it in metres.
    const double cell_w = (cfg_.max_x - cfg_.min_x) / std::max(1, cols_ - 1);
    changed |=
        r.read("sigma", tc.sigma * cell_w, non_negative, [&](double x) { tc.sigma = x / cell_w; });
    // The grade force's weights in world units, as the material's (L^2: m^2/s^2; L: m/s^2).
    weight("lam_grade", tc.lam_grade, L * L, non_negative);
    weight("lam_grade_hard", tc.lam_grade_hard, L, non_negative);
    weight("k_sharp", tc.k_sharp, 1.0, positive);
    weight("d_hat_m", tc.d_hat_m, 1.0, positive);
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

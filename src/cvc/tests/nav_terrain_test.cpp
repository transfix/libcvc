/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Terrain in the navigator's energy (P5, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md; doc 31 D9),
// on the kinematic plant: the terrain planes, gravity in the governors (H2), the grade force
// (H1), never entering infeasible slope, the slope feature (H3) and the climbing energy in the
// route (H5). The Jolt plant's gates are in nav_plant_test.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/nav/vehicle_plant.h>
#include <functional>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m
constexpr int R = 201, C = 201;
constexpr double kLo = -100.0, kHi = 100.0; // 1 m cells

double x_of(int c) { return kLo + c; }
double y_of(int r) { return kLo + r; }

std::vector<float> heights(const std::function<double(double, double)> &h) {
  std::vector<float> z(static_cast<std::size_t>(R) * C);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      z[static_cast<std::size_t>(r) * C + c] = static_cast<float>(h(x_of(c), y_of(r)));
  return z;
}

// A Humvee-sized agent in the trained metric (as nav_plant_test's).
sim_world::config config(double vmax_mps = 8.0) {
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = kLo;
  c.max_x = c.max_y = kHi;
  c.scale = kS;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6;
  c.veh.vmax = static_cast<float>(vmax_mps * kS);
  c.veh.L = static_cast<float>(3.3 * kS);
  c.veh.delta_max = 0.457f;
  c.veh.a_max = static_cast<float>(2.0 * kS);
  c.veh.a_lat_max = static_cast<float>(0.25 * 9.80665 * kS);
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(4.0 * kS);
  return c;
}

const float kColor[3] = {1.0f, 1.0f, 1.0f};

sim_world make(const sim_world::config &c, const std::vector<std::uint8_t> &occ, double sx,
               double sy, double gx, double gy, coef_mlp net = coef_mlp::default_biased()) {
  const float o[2] = {static_cast<float>(sx * kS), static_cast<float>(sy * kS)};
  const float g[2] = {static_cast<float>(gx * kS), static_cast<float>(gy * kS)};
  return sim_world(c, occ.data(), occ.data(), std::move(net), o, g, kColor, 1);
}

struct pose {
  float x = 0, y = 0, heading = 0, speed = 0;
};

pose pose_of(const sim_world &w) {
  float p[2], h, s;
  int m;
  std::uint8_t rc;
  w.snapshot(p, &h, &s, &m, &rc);
  return {p[0], p[1], h, s};
}

// Steer an agent along a route (world metres) by retargeting its goal to the first route point
// at least `ahead` metres from it, past the last one reached.
struct follower {
  std::vector<double> xy; // [x0, y0, x1, y1, ...]
  std::size_t next = 0;
  void steer(sim_world &w, double ahead = 15.0) {
    const pose p = pose_of(w);
    const std::size_t n = xy.size() / 2;
    while (next + 1 < n && std::hypot(xy[2 * next] - p.x, xy[2 * next + 1] - p.y) < ahead)
      ++next;
    w.retarget(0, static_cast<float>(xy[2 * next] * kS), static_cast<float>(xy[2 * next + 1] * kS));
  }
};

std::vector<double> route(const std::vector<std::uint8_t> &occ, double sx, double sy, double gx,
                          double gy, const climb_cost &climb = {}) {
  const std::vector<int> p =
      astar(occ.data(), R, C, static_cast<int>(sy - kLo), static_cast<int>(sx - kLo),
            static_cast<int>(gy - kLo), static_cast<int>(gx - kLo), nullptr, climb);
  std::vector<double> xy;
  for (std::size_t k = 0; k + 1 < p.size(); k += 2) {
    xy.push_back(x_of(p[k + 1]));
    xy.push_back(y_of(p[k]));
  }
  return xy;
}

double length(const std::vector<double> &xy) {
  double l = 0.0;
  for (std::size_t k = 2; k + 1 < xy.size(); k += 2)
    l += std::hypot(xy[k] - xy[k - 2], xy[k + 1] - xy[k - 1]);
  return l;
}

} // namespace

// The planes: the slope is the height's gradient, the grade cost is sin(slope) over the sine of
// the steepest feasible slope, and the cells beyond it are infeasible; a tip ratio lowers the
// limit.
TEST(NavTerrain, BuildsTheSlopeTheGradeCostAndTheInfeasibleCells) {
  const std::vector<float> z = heights([](double x, double) { return 0.3 * x; });
  terrain_config tc;
  tc.max_grade = 0.6;
  const terrain_planes p = terrain_build(z.data(), R, C, 1.0, tc);
  const std::size_t mid = static_cast<std::size_t>(100) * C + 100;
  EXPECT_NEAR(p.slope[mid], 0.3, 1e-5);
  EXPECT_NEAR(p.slope[static_cast<std::size_t>(R) * C + mid], 0.0, 1e-5);
  const double s = 0.3 / std::sqrt(1.09), sl = 0.6 / std::sqrt(1.36);
  EXPECT_NEAR(p.grade_cost[mid], s / sl, 1e-5);
  EXPECT_EQ(p.infeasible[mid], 0);
  EXPECT_NEAR(p.limit, 0.6, 1e-12);
  tc.tip_ratio = 0.3; // a tall, narrow vehicle: 0.8 x 0.3 = 0.24 < 0.3
  const terrain_planes q = terrain_build(z.data(), R, C, 1.0, tc);
  EXPECT_NEAR(q.limit, 0.24, 1e-12);
  EXPECT_EQ(q.infeasible[mid], 1);
  EXPECT_FLOAT_EQ(q.grade_cost[mid], 1.0f);
}

// H2: gravity in the governors. Driving at a wall down a 15% grade the agent comes in slower
// than on the flat (it has less braking to stop with). Up the grade it comes in no faster: the
// command never brakes harder than a_max, so the plan does not count on gravity's help.
// Each run reports its speed as it passes 15 m from the wall, and where it stops.
TEST(NavTerrain, TheStoppingGovernorTakesGravity) {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  for (int r = 0; r < R; ++r)
    for (int c = 140; c < 143; ++c)
      occ[static_cast<std::size_t>(r) * C + c] = 1; // a wall across x = 40..42
  double at15[3], closest[3];
  const bool trace = std::getenv("TRACE_GOV") != nullptr;
  for (int k = 0; k < 3; ++k) {
    const double grade = k == 0 ? 0.0 : (k == 1 ? -0.15 : 0.15); // down toward the wall, up
    sim_world w = make(config(10.0), occ, -40.0, 0.0, 80.0, 0.0);
    terrain_config tc;
    tc.block_infeasible = false;
    if (k > 0) {
      const std::vector<float> z = heights([&](double x, double) { return grade * x; });
      w.set_terrain(z.data(), tc);
    }
    at15[k] = -1.0;
    closest[k] = 1e9;
    for (int t = 0; t < 600; ++t) {
      w.step();
      const pose p = pose_of(w);
      if (at15[k] < 0.0 && p.x >= 25.0)
        at15[k] = p.speed;
      closest[k] = std::min(closest[k], 40.0 - static_cast<double>(p.x));
      if (trace && t % 10 == 0)
        std::printf("    k %d t %d x %.2f v %.2f\n", k, t, p.x, p.speed);
    }
  }
  std::printf("  15 m from the wall: %.2f m/s on the flat, %.2f downhill, %.2f uphill; it stops "
              "%.2f, %.2f and %.2f m from it\n",
              at15[0], at15[1], at15[2], closest[0], closest[1], closest[2]);
  EXPECT_LT(at15[1], at15[0] - 0.2);
  EXPECT_NEAR(at15[2], at15[0], 0.05);
  for (int k = 0; k < 3; ++k)
    EXPECT_GT(closest[k], 1.0) << "it never reaches the wall";
}

// H1's hard barrier: a steep ridge (slope 1.2) lies across the way, with flat ground past its
// end. With block_infeasible its cells plan as obstacles, so the agent, on an A* route that sees
// them, never puts its footprint on infeasible slope.
TEST(NavTerrain, InfeasibleSlopeIsNeverEntered) {
  const auto ridge = [](double x, double y) {
    if (y > 40.0)
      return 0.0;
    return std::max(0.0, 6.0 - 1.2 * std::fabs(x)); // a ridge 6 m high, slopes 1.2
  };
  const std::vector<float> z = heights(ridge);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  sim_world w = make(config(), occ, -40.0, -20.0, 40.0, -20.0);
  terrain_config tc;
  tc.max_grade = 0.6;
  tc.block_infeasible = true;
  w.set_terrain(z.data(), tc);
  // The route sees the same infeasible cells.
  std::vector<std::uint8_t> blocked(occ);
  for (std::size_t k = 0; k < blocked.size(); ++k)
    blocked[k] = static_cast<std::uint8_t>(blocked[k] | w.terrain().infeasible[k]);
  follower f{route(blocked, -40.0, -20.0, 40.0, -20.0)};
  ASSERT_FALSE(f.xy.empty());
  double end_x = 0.0;
  for (int t = 0; t < 1500; ++t) {
    f.steer(w);
    w.step();
    end_x = pose_of(w).x;
  }
  std::printf("  route %.0f m round the ridge's end; %d ticks on infeasible slope; ended at x = "
              "%.1f\n",
              length(f.xy), w.infeasible_entries()[0], end_x);
  EXPECT_EQ(w.infeasible_entries()[0], 0);
  EXPECT_GT(end_x, 20.0) << "it got past the ridge";
}

// Slope is a risk, not a wall (the default): a terrace's 5 m ramp, steeper than the vehicle's limit
// (0.8 against 0.6), runs across the whole way, with no way round. Planned as a wall the agent
// never gets onto it; as the material risk it is by default, the agent weighs it and crosses to
// its goal on the terrace (30 s: it gets there in about 12).
TEST(NavTerrain, SteepSlopeIsARiskNotAWall) {
  const auto terrace = [](double x, double) { return std::min(4.0, std::max(0.0, 0.8 * x)); };
  const std::vector<float> z = heights(terrace);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  double furthest[2];
  int entries[2];
  for (int k = 0; k < 2; ++k) {
    sim_world w = make(config(), occ, -40.0, 0.0, 40.0, 0.0);
    terrain_config tc;
    EXPECT_FALSE(tc.block_infeasible) << "the default";
    tc.block_infeasible = k == 0;
    w.set_terrain(z.data(), tc);
    furthest[k] = -1e9;
    for (int t = 0; t < 500; ++t) {
      w.step();
      furthest[k] = std::max(furthest[k], static_cast<double>(pose_of(w).x));
    }
    entries[k] = w.infeasible_entries()[0];
  }
  std::printf("  as a wall: got to x = %.1f, %d ticks on the ramp; as a risk: x = %.1f, %d ticks\n",
              furthest[0], entries[0], furthest[1], entries[1]);
  EXPECT_LT(furthest[0], 0.0) << "a wall it never crosses";
  EXPECT_GT(furthest[1], 36.0) << "a risk it crosses, to its goal";
  EXPECT_GT(entries[1], 0);
}

// The risk governor holds an override_command (a route follower's) to its cap as it holds the
// drive: along a 22-degree sidehill (grade cost 0.72 under the default 31-degree limit), full
// throttle reaches the top speed without it and stays under v_risk with it.
TEST(NavTerrain, TheRiskGovernorHoldsAnOverrideCommand) {
  const std::vector<float> z = heights([](double, double y) { return 0.4 * y; });
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  double top[2];
  float cap = 0.0f;
  for (int k = 0; k < 2; ++k) {
    sim_world::config c = config();
    c.veh.risk_governor = k == 1;
    c.veh.risk_vmin = static_cast<float>(1.0 * kS);
    c.veh.risk_onset = 0.3f;
    sim_world w = make(c, occ, -80.0, 0.0, 90.0, 0.0);
    w.set_terrain(z.data(), terrain_config{});
    top[k] = 0.0;
    for (int t = 0; t < 300; ++t) {
      w.override_command(0, 0.0f, c.veh.a_max);
      w.step();
      top[k] = std::max(top[k], static_cast<double>(pose_of(w).speed)); // m/s
    }
    const float r = w.terrain().grade_cost[static_cast<std::size_t>(R / 2) * C + C / 2];
    cap = static_cast<float>(8.0 - (8.0 - 1.0) * (r - 0.3) / 0.7);
    std::printf("  grade cost %.2f: top speed %.2f m/s (governor %s), v_risk %.2f m/s\n", r, top[k],
                k ? "on" : "off", cap);
  }
  EXPECT_GT(top[0], 7.5) << "full throttle reaches the top speed without the governor";
  EXPECT_LE(top[1], cap + 0.01);
  EXPECT_GT(top[1], cap - 0.5) << "the cap, not a stop";
}

namespace {

// A plant that integrates whatever acceleration it is told, with no top speed of its own and
// no limits to report (nav's envelope then applies): whatever holds an override_command to
// nav's vmax on a plant has to be sim_world.
class free_plant : public vehicle_plant {
public:
  plant_kind kind() const override { return plant_kind::kinematic; }
  void attach(int n, const plant_state *start, const cvc::world_units::normalized_frame &,
              double dt) override {
    s_.assign(start, start + n);
    accel_.assign(static_cast<std::size_t>(n), 0.0f);
    dt_ = static_cast<float>(dt);
  }
  plant_state state(int i) const override { return s_[static_cast<std::size_t>(i)]; }
  plant_limits limits(int) const override { return {}; }
  void command(int i, float, float accel) override { accel_[static_cast<std::size_t>(i)] = accel; }
  void advance() override {
    for (std::size_t i = 0; i < s_.size(); ++i) {
      plant_state &s = s_[i];
      s.speed += accel_[i] * dt_;
      s.x += s.speed * std::cos(s.heading) * dt_;
      s.y += s.speed * std::sin(s.heading) * dt_;
    }
  }

private:
  std::vector<plant_state> s_;
  std::vector<float> accel_;
  float dt_ = 0.0f;
};

} // namespace

// An override_command (a route follower's, at its own route speed) is held to the agent's top
// speed on a plant, as the drive's own command is: full throttle on a plant that would speed up
// for ever stays at nav's 8 m/s.
TEST(NavTerrain, AnOverrideCommandIsHeldToTheTopSpeed) {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  const sim_world::config c = config(8.0);
  sim_world w = make(c, occ, -90.0, 0.0, 95.0, 0.0);
  free_plant plant;
  w.set_plant(&plant);
  double top = 0.0;
  for (int t = 0; t < 200; ++t) {
    w.override_command(0, 0.0f, 4.0f * c.veh.a_max);
    w.step();
    top = std::max(top, static_cast<double>(pose_of(w).speed)); // m/s
  }
  std::printf("  full throttle on a plant with no top speed: %.2f m/s (nav's vmax 8)\n", top);
  EXPECT_LE(top, 8.0 + 0.01);
  EXPECT_GT(top, 7.5) << "held at the top speed, not below it";
}

// H1, the soft force: with the grade cost's weight, the agent driving past a hill leans away
// from it, onto gentler ground.
TEST(NavTerrain, TheGradeForceLeansTowardGentlerGround) {
  const auto hill = [](double x, double y) {
    return 10.0 * std::exp(-(x * x + (y - 12.0) * (y - 12.0)) / (2.0 * 15.0 * 15.0));
  };
  const std::vector<float> z = heights(hill);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  double worst[2];
  for (int k = 0; k < 2; ++k) {
    sim_world w = make(config(), occ, -70.0, 0.0, 70.0, 0.0);
    terrain_config tc;
    tc.lam_grade = k == 0 ? 0.0f : 2.0f;
    w.set_terrain(z.data(), tc);
    const terrain_planes &tp = w.terrain();
    worst[k] = 0.0;
    for (int t = 0; t < 400; ++t) {
      w.step();
      const pose p = pose_of(w);
      const int c = static_cast<int>(std::lround(p.x - kLo));
      const int r = static_cast<int>(std::lround(p.y - kLo));
      if (r >= 0 && r < R && c >= 0 && c < C)
        worst[k] = std::max(
            worst[k], static_cast<double>(tp.grade_cost[static_cast<std::size_t>(r) * C + c]));
    }
  }
  std::printf("  the steepest grade cost driven: %.3f without the grade force, %.3f with it\n",
              worst[0], worst[1]);
  EXPECT_LT(worst[1], worst[0] - 0.02);
}

// H5: the climbing energy in the route. Over a 12 m hill in the way the plain route climbs it;
// with the energy-consistent climb cost (a metre of rise weighs 1 / C_rr metres of flat) the
// route goes round, climbing almost nothing for a detour under half again the distance. For a
// Humvee the climb is about 315 kJ, the detour about 30 kJ.
TEST(NavTerrain, TheRouteWeighsTheEnergyOfClimbing) {
  const auto hill = [](double x, double y) {
    return 12.0 * std::exp(-(x * x + y * y) / (2.0 * 18.0 * 18.0));
  };
  const std::vector<float> z = heights(hill);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  const auto climbed = [&](const std::vector<double> &xy) {
    double up = 0.0;
    for (std::size_t k = 2; k + 1 < xy.size(); k += 2)
      up += std::max(0.0, hill(xy[k], xy[k + 1]) - hill(xy[k - 2], xy[k - 1]));
    return up;
  };
  const std::vector<double> plain = route(occ, -70.0, 0.0, 70.0, 0.0);
  climb_cost cc;
  cc.height = z.data();
  cc.per_metre = climb_weight(0.015, 1.0);
  const std::vector<double> energy = route(occ, -70.0, 0.0, 70.0, 0.0, cc);
  std::printf("  plain route %.1f m climbing %.2f m; energy route %.1f m climbing %.2f m\n",
              length(plain), climbed(plain), length(energy), climbed(energy));
  EXPECT_LT(climbed(energy), 0.5 * climbed(plain));
  EXPECT_LT(length(energy), 1.5 * length(plain));
  // Without a climb cost the overload is the plain search, step for step.
  EXPECT_EQ(route(occ, -70.0, 0.0, 70.0, 0.0, climb_cost{}), plain);
}

// H3: the slope look-ahead feature, appended after grip and risk, and the v3 format that keeps a
// slope net off a host that would read it as a grip net.
TEST(NavTerrain, TheSlopeFeatureAndItsFormat) {
  const std::vector<float> z = heights([](double x, double) { return x > 0.0 ? 0.4 * x : 0.0; });
  terrain_config tc;
  const terrain_planes tp = terrain_build(z.data(), R, C, 1.0, tc);
  const material_planes mp =
      material_build(tp.grade_cost.data(), tp.infeasible.data(), R, C, 1.0, kS, tc.sigma);
  const std::vector<float> stack = mp.stacked();
  const material_stack ms{stack.data(), 1, R, C, kLo, kLo, kHi, kHi, 0.0, 0.0, kS};
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  std::vector<float> field(static_cast<std::size_t>(3) * R * C);
  build_sdf(occ.data(), R, C, kLo, kLo, kHi, kHi, kS, field.data(), field.data() + R * C,
            field.data() + 2 * R * C);
  field_stack fs;
  fs.data = field.data();
  fs.M = 1;
  fs.H = R;
  fs.W = C;
  fs.mnx = fs.mny = kLo;
  fs.mxx = fs.mxy = kHi;
  fs.S = kS;
  // An agent at x = -5 m heading for x = +20 m: the probe reaches 0.3 units (6 m) ahead, onto
  // the slope.
  const float on[2] = {static_cast<float>(-5.0 * kS), 0.0f};
  const float goal[2] = {static_cast<float>(20.0 * kS), 0.0f};
  float feat[6] = {};
  coef_feats(fs, on, goal, 1, nullptr, feat, 0, nullptr, 0.3f, 3, nullptr, 0.3f, 3, nullptr, &ms);
  EXPECT_GT(feat[5], 0.3f) << "the slope ahead reads in the sixth column";
  float flat[6] = {};
  const float on_flat[2] = {static_cast<float>(-60.0 * kS), 0.0f};
  coef_feats(fs, on_flat, goal, 1, nullptr, flat, 0, nullptr, 0.3f, 3, nullptr, 0.3f, 3, nullptr,
             &ms);
  EXPECT_FLOAT_EQ(flat[5], 0.0f) << "flat ground ahead reads 0";
  // The format: a slope net is v3, not a grip net, and round-trips.
  const coef_mlp net = coef_mlp::from_layers(6, 3, {3}, {6}, {0}, {std::vector<float>(18, 0.0f)},
                                             {std::vector<float>(3, 0.0f)}, {1.0f, 3.0f, 4.0f},
                                             coef_mlp::kFlagFeatSlope);
  EXPECT_EQ(net.format_version(), coef_mlp::kFormatVersionSlope);
  EXPECT_TRUE(net.has_slope());
  EXPECT_FALSE(net.has_mu());
  const std::string path = ::testing::TempDir() + "slope_net.cvcnav";
  net.save(path);
  const coef_mlp back = coef_mlp::load(path);
  EXPECT_TRUE(back.has_slope());
  EXPECT_EQ(back.format_version(), 3u);
  // The drive refuses a slope net without terrain, and takes it with one.
  sim_world ws = make(config(), occ, -40.0, 0.0, 40.0, 0.0, net);
  EXPECT_THROW(ws.step(), std::runtime_error);
  ws.set_terrain(z.data(), tc);
  EXPECT_NO_THROW(ws.step());
}

// Terrain risk is material risk. The risk look-ahead a risk net reads takes the terrain's grade
// cost: on terrain alone it is the slope probe's value, and over a material stack the worse of the
// two (without a terrain, exactly the material's as before). The drive telemetry's mrisk, which
// the nav ledger reduces, reports it with the weights the drive applied, and a risk net drives on
// a terrain with no material stack.
TEST(NavTerrain, TerrainRiskIsMaterialRisk) {
  const std::vector<float> z = heights([](double x, double) { return x > 0.0 ? 0.4 * x : 0.0; });
  terrain_config tc;
  const terrain_planes tp = terrain_build(z.data(), R, C, 1.0, tc);
  const material_planes mp =
      material_build(tp.grade_cost.data(), tp.infeasible.data(), R, C, 1.0, kS, tc.sigma);
  const std::vector<float> stack = mp.stacked();
  const material_stack ter{stack.data(), 1, R, C, kLo, kLo, kHi, kHi, 0.0, 0.0, kS};
  // a material stack with no risk anywhere
  const std::vector<float> none(static_cast<std::size_t>(R) * C, 0.0f);
  const std::vector<std::uint8_t> no_hard(static_cast<std::size_t>(R) * C, 0);
  const std::vector<float> mstack =
      material_build(none.data(), no_hard.data(), R, C, 1.0, kS, tc.sigma).stacked();
  const material_stack mat{mstack.data(), 1, R, C, kLo, kLo, kHi, kHi, 0.0, 0.0, kS};
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  std::vector<float> field(static_cast<std::size_t>(3) * R * C);
  build_sdf(occ.data(), R, C, kLo, kLo, kHi, kHi, kS, field.data(), field.data() + R * C,
            field.data() + 2 * R * C);
  field_stack fs;
  fs.data = field.data();
  fs.M = 1;
  fs.H = R;
  fs.W = C;
  fs.mnx = fs.mny = kLo;
  fs.mxx = fs.mxy = kHi;
  fs.S = kS;
  const float on[2] = {static_cast<float>(-5.0 * kS), 0.0f};
  const float goal[2] = {static_cast<float>(20.0 * kS), 0.0f};
  // columns: base 5, risk (5), slope (6)
  float alone[7] = {}, both[7] = {}, mat_only[7] = {};
  coef_feats(fs, on, goal, 1, nullptr, alone, 0, nullptr, 0.3f, 3, nullptr, 0.3f, 3, nullptr, &ter,
             0.3f, 3, &ter);
  coef_feats(fs, on, goal, 1, nullptr, both, 0, nullptr, 0.3f, 3, &mat, 0.3f, 3, nullptr, &ter,
             0.3f, 3, &ter);
  coef_feats(fs, on, goal, 1, nullptr, mat_only, 0, nullptr, 0.3f, 3, &mat, 0.3f, 3, nullptr, &ter);
  EXPECT_GT(alone[5], 0.3f) << "the slope ahead reads as risk";
  EXPECT_FLOAT_EQ(alone[5], alone[6]) << "on terrain alone, the slope probe's value";
  EXPECT_FLOAT_EQ(both[5], alone[5]) << "over a riskless material, the terrain's";
  EXPECT_FLOAT_EQ(mat_only[5], 0.0f) << "without the terrain's risk, the material's alone";

  // The telemetry: driving up the slope, mrisk reports the grade cost, and the weights the drive
  // applied are the terrain's (no material stack).
  sim_world w = make(config(), occ, -20.0, 0.0, 60.0, 0.0);
  w.set_terrain(z.data(), tc);
  w.set_capture_drive_telemetry(true);
  float peak = 0.0f, lam_hard = 0.0f;
  for (int t = 0; t < 300; ++t) {
    w.step();
    const drive_telemetry &d = w.drive_telemetry_data()[0];
    if (d.mrisk > peak) {
      peak = d.mrisk;
      lam_hard = d.lam_hard;
    }
  }
  std::printf("  the risk ahead: %.3f alone, %.3f over a riskless material; driven: mrisk peaks at "
              "%.3f, lam_hard %.1f\n",
              alone[5], both[5], peak, lam_hard);
  EXPECT_GT(peak, 0.3f) << "the slope driven reads as material risk";
  EXPECT_FLOAT_EQ(lam_hard, tc.lam_grade_hard);

  // A risk net drives on a terrain with no material stack, and refuses a world with neither.
  const coef_mlp net = coef_mlp::from_layers(6, 3, {3}, {6}, {0}, {std::vector<float>(18, 0.0f)},
                                             {std::vector<float>(3, 0.0f)}, {1.0f, 3.0f, 4.0f},
                                             coef_mlp::kFlagFeatRisk);
  ASSERT_TRUE(net.has_risk());
  sim_world wr = make(config(), occ, -40.0, 0.0, 40.0, 0.0, net);
  EXPECT_THROW(wr.step(), std::runtime_error);
  wr.set_terrain(z.data(), tc);
  EXPECT_NO_THROW(wr.step());
}

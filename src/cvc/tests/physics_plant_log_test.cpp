/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// physics_plant_log_test: cvc::physics::plant_log (cvc/physics/plant_log.h).
//   - The residual's own arithmetic: a run made by a model itself leaves that model no residual
//     (the kinematic bicycle's and diff::dyn_plant's), so the state, the command and the tick
//     line up.
//   - A Humvee slaloming on Jolt: the samples come at the tick, the command in row k + 1 is the
//     one nav held over the tick, the centre of mass moves with its logged twist, the swerves
//     pass 0.3 g, and the residuals are reported by lateral g.
//   - cut_flights and write_npy: the windows, their layout and the file.
//   - nav_plant_options::ground_height puts a start on the ground under it.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/plant_log.h>
#include <cvc/physics/world.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <random>
#include <string>
#include <system_error>
#include <vector>

using namespace cvc::physics;
using cvc::nav::sim_world;

namespace {

#define REQUIRE_JOLT()                                                                             \
  if (!backend_available(backend::jolt))                                                           \
  GTEST_SKIP() << "this libcvc was built without Jolt"

constexpr double kG = 9.80665;

// A run produced by the kinematic bicycle itself, about the rear axle, with weaving commands.
std::vector<planar_sample> kinematic_run(const vehicle_desc &d, double dt, int nsub, int ticks) {
  std::vector<planar_sample> run(1);
  run[0].vx = 6.0;
  const double L = d.wheelbase(), h = dt / nsub;
  for (int t = 1; t <= ticks; ++t) {
    planar_sample s = run.back();
    s.t += dt;
    s.steer_cmd = 0.15 * std::sin(0.3 * t);
    s.accel_cmd = 0.8 * std::cos(0.2 * t);
    s.drive = s.accel_cmd;
    for (int j = 0; j < nsub; ++j) {
      s.vx = s.vx + h * s.accel_cmd;
      s.psi = s.psi + h * (s.vx / L) * std::tan(s.steer_cmd);
      s.rear_x = s.rear_x + h * s.vx * std::cos(s.psi);
      s.rear_y = s.rear_y + h * s.vx * std::sin(s.psi);
    }
    s.a_lat = s.vx * s.vx * std::tan(s.steer_cmd) / L;
    run.push_back(s);
  }
  return run;
}

// A run produced by diff::dyn_plant itself (the vehicle `si`, by default dyn_bike_si_of's), from
// rest in its actuator, as one_tick_residuals steps it.
std::vector<planar_sample> dynamic_run(const vehicle_desc &d, double mu, double dt, int nsub,
                                       int ticks, const cvc::nav::dyn_bike_si *vehicle = nullptr) {
  cvc::world_units::normalized_frame si;
  si.scale = 1.0;
  const cvc::nav::diff::dyn_bike_params p =
      cvc::nav::dyn_bike_params_of(vehicle ? *vehicle : nav_plant::dyn_bike_si_of(d, mu), si);
  cvc::nav::diff::bike_veh lim{};
  lim.L = p.lf + p.lr;
  lim.vmax = 1e3f;
  lim.delta_max = static_cast<float>(d.max_steer);
  lim.hdt = static_cast<float>(dt / nsub);
  lim.allow_reverse = 1;
  cvc::nav::diff::dyn_state<double> m{};
  m.vx = 8.0;
  std::vector<planar_sample> run(1);
  run[0].vx = m.vx;
  for (int t = 1; t <= ticks; ++t) {
    planar_sample s;
    s.t = t * dt;
    s.steer_cmd = 0.2 * std::sin(0.25 * t);
    s.accel_cmd = std::cos(0.15 * t);
    for (int j = 0; j < nsub; ++j) {
      cvc::nav::diff::dyn_state<double> o;
      cvc::nav::diff::dyn_plant<double>(p, lim, m, s.steer_cmd, s.accel_cmd, 0.0, o);
      m = o;
    }
    s.rear_x = m.x;
    s.rear_y = m.y;
    s.psi = m.psi;
    s.vx = m.vx;
    s.vy = m.vy;
    s.r = m.r;
    s.steer = m.delta;
    s.a_lat = m.vx * m.r;
    run.push_back(s);
  }
  return run;
}

double total_ticks(const std::vector<residual_bin> &bins) {
  double n = 0;
  for (const residual_bin &b : bins)
    n += static_cast<double>(b.ticks);
  return n;
}

} // namespace

// (The run is made in this file and replayed in plant_log.cpp, which a compiler may contract into
// FMAs differently, so "none" is to rounding: 1e-9.)
TEST(PlantLog, ARunAModelMadeLeavesThatModelNoResidual) {
  const vehicle_desc d = vehicle_desc::humvee();
  for (const int nsub : {1, 2, 6}) {
    const plant_residuals k =
        one_tick_residuals({kinematic_run(d, 0.06, nsub, 300)}, 0.06, d, 0.9, nsub);
    EXPECT_EQ(total_ticks(k.kinematic), 300.0);
    for (const residual_bin &b : k.kinematic) {
      EXPECT_LT(b.position_max_m, 1e-9) << "nsub " << nsub;
      EXPECT_LT(b.heading_max_rad, 1e-9);
      EXPECT_LT(b.speed_max_mps, 1e-9);
    }
    const plant_residuals y =
        one_tick_residuals({dynamic_run(d, 0.9, 0.06, nsub, 300)}, 0.06, d, 0.9, nsub);
    EXPECT_EQ(total_ticks(y.dynamic), 300.0);
    for (const residual_bin &b : y.dynamic) {
      EXPECT_LT(b.position_max_m, 1e-9) << "nsub " << nsub;
      EXPECT_LT(b.heading_max_rad, 1e-9);
      EXPECT_LT(b.speed_max_mps, 1e-9);
      EXPECT_LT(b.lateral_rms_mps, 1e-9);
      EXPECT_LT(b.yaw_rate_rms, 1e-9);
    }
  }
  // Ticks at rest are skipped, and the bins must ascend.
  std::vector<planar_sample> rest(5);
  EXPECT_EQ(one_tick_residuals({rest}, 0.06, d).skipped_rest, 4u);
  // Off the ground or an impact: skipped too.
  std::vector<planar_sample> bumpy = kinematic_run(d, 0.06, 1, 4);
  bumpy[2].grounded = false;
  bumpy[4].a_lon = 2.0 * kG;
  const plant_residuals b = one_tick_residuals({bumpy}, 0.06, d);
  EXPECT_EQ(b.skipped_airborne, 2u);
  EXPECT_EQ(b.skipped_impact, 1u);
  EXPECT_EQ(b.skipped(), 3u);
  // The kinematic reference keeps the speed within the plan's top speed.
  std::vector<planar_sample> fast = kinematic_run(d, 0.06, 1, 1);
  fast[0].vmax = fast[1].vmax = 6.0;
  fast[1].accel_cmd = 50.0; // a command past what the speed cap allows
  const plant_residuals c = one_tick_residuals({fast}, 0.06, d);
  EXPECT_NEAR(c.kinematic[0].speed_max_mps + c.kinematic[1].speed_max_mps +
                  c.kinematic.back().speed_max_mps,
              std::fabs(fast[1].vx - 6.0), 1e-9);
  EXPECT_THROW(one_tick_residuals({rest}, 0.06, d, 0.9, 1, {0.3, 0.1}), std::invalid_argument);
}

// fit_dyn_bike finds a vehicle the spec sheet does not give from runs that vehicle made: every
// knob on its grid, the fitted cost about zero, and the spec sheet's cost well above it.
TEST(PlantLog, FitsTheDynamicModelToRunsOfAKnownVehicle) {
  const vehicle_desc d = vehicle_desc::humvee();
  cvc::nav::dyn_bike_si truth = nav_plant::dyn_bike_si_of(d, 0.9);
  truth.drive_lag = 0.04;
  truth.tire_B_front *= 0.55;
  truth.tire_B_rear *= 0.7;
  truth.yaw_inertia *= 1.2;
  truth.drag *= 0.5;
  truth.rolling_resistance *= 0.5;
  const std::vector<std::vector<planar_sample>> runs = {dynamic_run(d, 0.9, 0.06, 2, 400, &truth)};
  const dyn_bike_fit f = fit_dyn_bike(runs, 0.06, d, 0.9, 2, 3);
  std::printf("  fit: drive lag %.3f, steer lag %.3f, B x%.2f / x%.2f, Iz x%.2f, resistance x%.2f; "
              "cost %.3g -> %.3g\n",
              f.drive_lag, f.steer_lag, f.tire_b_front_factor, f.tire_b_rear_factor,
              f.yaw_inertia_factor, f.resistance_factor, f.cost_default, f.cost_fit);
  EXPECT_NEAR(f.drive_lag, 0.04, 1e-12);
  EXPECT_NEAR(f.steer_lag, 0.0, 1e-12);
  EXPECT_NEAR(f.tire_b_front_factor, 0.55, 1e-12);
  EXPECT_NEAR(f.tire_b_rear_factor, 0.7, 1e-12);
  EXPECT_NEAR(f.yaw_inertia_factor, 1.2, 1e-12);
  EXPECT_NEAR(f.resistance_factor, 0.5, 1e-12);
  EXPECT_LT(f.cost_fit, 1e-12);
  EXPECT_GT(f.cost_default, 1e-6);
}

TEST(PlantLog, CutsFlightsAndWritesThemAsNpy) {
  const vehicle_desc d = vehicle_desc::humvee();
  std::vector<std::vector<planar_sample>> runs = {kinematic_run(d, 0.02, 2, 120),
                                                  kinematic_run(d, 0.02, 2, 60)};
  runs[1][30].contacts = 1;     // a window with a contact is dropped
  runs[0][60].grounded = false; // and one with a wheel off the ground
  for (planar_sample &s : runs[0])
    s.steer = s.steer_cmd;
  const flight_set f = cut_flights(runs, 26, 25);
  // Run 0: 121 samples -> windows at 0, 25, 50, 75 (the wheel off the ground drops 50); run 1: 61
  // -> 0, 25 (the contact drops 25).
  EXPECT_EQ(f.flights, 4u);
  EXPECT_EQ(f.dropped, 2u);
  EXPECT_EQ(f.rows, 26u);
  ASSERT_EQ(f.data.size(), f.flights * f.rows * kPlanarWidth);
  const planar_sample &q = runs[0][25 + 3];
  const double *row = &f.data[(1 * f.rows + 3) * kPlanarWidth];
  EXPECT_EQ(row[0], q.x);
  EXPECT_EQ(row[2], q.psi);
  EXPECT_EQ(row[3], q.vx);
  EXPECT_EQ(row[6], q.drive);
  EXPECT_EQ(row[7], 0.5 * (runs[0][27].steer + q.steer)); // the steer's mean over the interval
  EXPECT_EQ(cut_flights(runs, 26, 25, 1.0, steer_input::command).data[(1 * f.rows + 3) * 8 + 7],
            q.steer_cmd);

  const std::string path =
      (std::filesystem::temp_directory_path() / "cvc_plant_log_test.npy").string();
  write_npy(path, f.data, {f.flights, f.rows, static_cast<std::size_t>(kPlanarWidth)});
  std::vector<char> bytes;
  {
    // Closed before the remove below: Windows refuses to delete an open file.
    std::ifstream in(path, std::ios::binary);
    bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  std::error_code ec;
  std::filesystem::remove(path, ec);
  ASSERT_GT(bytes.size(), 10u);
  EXPECT_EQ(std::memcmp(bytes.data(), "\x93NUMPY\x01\x00", 8), 0);
  const std::size_t hl = static_cast<unsigned char>(bytes[8]) |
                         (static_cast<std::size_t>(static_cast<unsigned char>(bytes[9])) << 8);
  EXPECT_EQ((10 + hl) % 64, 0u);
  const std::string header(bytes.data() + 10, hl);
  EXPECT_NE(header.find("'descr': '<f8'"), std::string::npos);
  EXPECT_NE(header.find("'shape': (4, 26, 8)"), std::string::npos) << header;
  ASSERT_EQ(bytes.size(), 10 + hl + f.data.size() * sizeof(double));
  EXPECT_EQ(std::memcmp(bytes.data() + 10 + hl, f.data.data(), f.data.size() * 8), 0);
}

// A Humvee slaloming on flat ground through nav and Jolt, logged at the tick.
TEST(PlantLog, LogsAHumveeSwervingOnJolt) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  sim_world::config c;
  c.rows = c.cols = 201;
  c.min_x = c.min_y = -400.0;
  c.max_x = c.max_y = 400.0;
  c.scale = 0.05;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.02f;
  c.veh.vmax = static_cast<float>(14.0 * c.scale);
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(4.0 * c.scale);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(c.rows) * c.cols, 0);
  const float o[2] = {static_cast<float>(-300 * c.scale), 0.0f};
  const float g[2] = {static_cast<float>(-240 * c.scale), static_cast<float>(25 * c.scale)};
  const float col[3] = {1, 1, 1};
  sim_world sim(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o, g, col, 1);
  plant_host_options ho;
  ho.plant.plan_with_vehicle = true;
  plant_host host(ho);
  host.use(sim, "humvee");
  nav_plant &plant = *host.plant();
  plant.set_lateral_g(0.9);
  plant_log log(sim, plant);
  log.start();
  float pos[2], gw[2];
  int side = 1;
  std::vector<float> cmd_s, cmd_a;
  for (int t = 0; t < 1500; ++t) {
    sim.snapshot(pos, nullptr, nullptr, nullptr, nullptr);
    sim.goals_world(gw);
    if (std::hypot(pos[0] - gw[0], pos[1] - gw[1]) < 25.0) {
      side = -side;
      sim.retarget(0, static_cast<float>((gw[0] + 70.0) * c.scale),
                   static_cast<float>(side * 30.0 * c.scale));
    }
    sim.step();
    log.record();
    cmd_s.push_back(sim.plant_steer_command()[0]);
    cmd_a.push_back(sim.plant_accel_command()[0]);
  }
  ASSERT_EQ(log.agents(), 1);
  const std::vector<planar_sample> &run = log.runs()[0];
  ASSERT_EQ(run.size(), 1501u);
  EXPECT_EQ(run[0].steer_cmd, 0.0);
  double peak = 0.0, kin_err = 0.0, vmax = 0.0;
  for (std::size_t k = 1; k < run.size(); ++k) {
    EXPECT_NEAR(run[k].t, 0.02 * k, 1e-9);
    // Row k's command is the one nav held over the tick that ended there, in SI.
    EXPECT_EQ(run[k].steer_cmd, cmd_s[k - 1]);
    EXPECT_NEAR(run[k].accel_cmd, cmd_a[k - 1] / c.scale,
                1e-9 * (1.0 + std::fabs(run[k].accel_cmd)));
    // The centre of mass moves with its logged twist (the trapezoid over the tick).
    const planar_sample &a = run[k - 1], &b = run[k];
    const double vxw = 0.5 * (a.vx * std::cos(a.psi) - a.vy * std::sin(a.psi) +
                              b.vx * std::cos(b.psi) - b.vy * std::sin(b.psi));
    const double vyw = 0.5 * (a.vx * std::sin(a.psi) + a.vy * std::cos(a.psi) +
                              b.vx * std::sin(b.psi) + b.vy * std::cos(b.psi));
    kin_err = std::max(kin_err, std::hypot((b.x - a.x) / 0.02 - vxw, (b.y - a.y) / 0.02 - vyw));
    peak = std::max(peak, std::fabs(b.a_lat) / kG);
    vmax = std::max(vmax, b.vx);
  }
  std::printf("  swerve: peak %.2f g, top speed %.1f m/s, |CoM velocity - twist| max %.3f m/s\n",
              peak, vmax, kin_err);
  EXPECT_GT(peak, 0.3) << "the swerves pass 0.3 g";
  EXPECT_LT(kin_err, 0.1);
  EXPECT_GT(vmax, 8.0);

  const plant_residuals res = one_tick_residuals(log.runs(), 0.02, d, 0.9, 2);
  for (const char *which : {"dynamic", "kinematic"}) {
    const std::vector<residual_bin> &bins =
        std::string(which) == "dynamic" ? res.dynamic : res.kinematic;
    std::printf("  %s residual over one 0.02 s tick:\n", which);
    for (const residual_bin &b : bins)
      if (b.ticks)
        std::printf("    %.1f-%.1f g: %4llu ticks, position rms %.4f m, heading rms %.5f rad, "
                    "speed rms %.4f m/s\n",
                    b.lo_g, std::isfinite(b.hi_g) ? b.hi_g : 9.9,
                    static_cast<unsigned long long>(b.ticks), b.position_rms_m, b.heading_rms_rad,
                    b.speed_rms_mps);
  }
  EXPECT_GT(total_ticks(res.dynamic), 1000.0);
  EXPECT_GT(res.dynamic.back().ticks + res.dynamic[res.dynamic.size() - 2].ticks, 0u)
      << "some ticks above 0.3 g";
  // One 0.02 s tick from the plant's own state: both models stay within centimetres.
  for (const residual_bin &b : res.dynamic)
    if (b.ticks) {
      EXPECT_LT(b.position_rms_m, 0.05);
      EXPECT_TRUE(std::isfinite(b.yaw_rate_rms));
    }
  host.use(sim, "kinematic");
}

TEST(PlantLog, GroundHeightPutsAStartOnTheGroundUnderIt) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::roboracer();
  double z[2];
  for (int k = 0; k < 2; ++k) {
    world_config wc;
    wc.backend = backend::jolt;
    world w(wc);
    body_desc g;
    g.shape = box{{50.0, 50.0, 0.5}};
    g.motion = motion_type::static_;
    g.position = {0.0, 0.0, 3.0 - 0.5}; // the ground's top at z = 3
    w.add_body(g);
    nav_plant_options po;
    if (k == 0)
      po.ground_z = 3.0;
    else
      po.ground_height = [](double, double) { return 3.0; };
    nav_plant plant(w, {d}, po);
    cvc::nav::plant_state s0{};
    cvc::world_units::normalized_frame f;
    f.scale = 0.05;
    plant.attach(1, &s0, f, 0.02);
    z[k] = w.vehicle_state(plant.vehicle(0)).body.position.z;
  }
  EXPECT_EQ(z[0], z[1]);
  EXPECT_GT(z[1], 3.0);
}

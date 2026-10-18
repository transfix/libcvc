/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The energy ledger (P3, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md): its gates.
//  1. Over a closed drive cycle on rolling terrain, |residual| <= 2% of the energy supplied.
//  2. Coasting with no input supplies nothing, and dKE + dPE = -dissipated within 2%.
//  3. Coasting downhill, the kinetic energy gained is -dPE - dissipated within 2%.
//  4. Regeneration never returns more than the brakes take.
// And the sources' arithmetic, and that measuring changes nothing.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace cvc::physics;

namespace {

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

world_config jolt_config() {
  world_config c;
  c.backend = backend::jolt;
  return c;
}

void flat_ground(world &w, double mu, double crr = 0.0) {
  const material_id m =
      w.add_material({"ground", static_cast<float>(mu), 0.0f, static_cast<float>(crr)});
  body_desc g;
  g.shape = box{{1500.0, 1500.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = m;
  w.add_body(g);
}

// A road of rolling hills along x: 1 m high, 50 m apart (at most 7 degrees), from x = -50 to 450.
double road_height(double x) { return 1.0 * std::sin(2.0 * 3.14159265358979 * x / 50.0); }

void rolling_road(world &w) {
  const material_id m = w.add_material({"road", 0.9f, 0.0f, 0.015f});
  const int cols = 501, rows = 41;
  std::vector<float> h(static_cast<std::size_t>(rows) * cols);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      h[static_cast<std::size_t>(r) * cols + c] = static_cast<float>(road_height(c - 50.0));
  auto field =
      std::make_shared<cvc::height_field>(rows, cols, -50.0, -20.0, 1.0, 1.0, std::move(h));
  body_desc g;
  g.shape = terrain{field};
  g.motion = motion_type::static_;
  g.material = m;
  w.add_body(g);
}

double speed_of(const world &w, vehicle_id v) { return w.vehicle_state(v).speed; }

// The ledger's change between two readings.
vehicle_energy change(const vehicle_energy &a, const vehicle_energy &b) {
  vehicle_energy d;
  d.supplied = b.supplied - a.supplied;
  d.brake = b.brake - a.brake;
  d.tire = b.tire - a.tire;
  d.aero = b.aero - a.aero;
  d.rolling = b.rolling - a.rolling;
  d.medium = b.medium - a.medium;
  d.drivetrain = b.drivetrain - a.drivetrain;
  d.drivetrain_split = b.drivetrain_split - a.drivetrain_split;
  d.suspension = b.suspension - a.suspension;
  d.chassis_damping = b.chassis_damping - a.chassis_damping;
  d.fields = b.fields - a.fields;
  d.stored = b.stored - a.stored;
  d.residual = b.residual - a.residual;
  d.kinetic = b.kinetic - a.kinetic;
  d.potential = b.potential - a.potential;
  d.spring = b.spring - a.spring;
  d.engine = b.engine - a.engine;
  d.idle = b.idle - a.idle;
  d.regen = b.regen - a.regen;
  d.source = b.source - a.source;
  d.fuel_used = b.fuel_used - a.fuel_used;
  d.distance = b.distance - a.distance;
  return d;
}

void print(const char *what, const vehicle_energy &d) {
  std::printf("  %s: supplied %.0f J; brake %.0f, tire %.0f, aero %.0f, rolling %.0f, drivetrain "
              "%.0f, suspension %.0f, chassis %.0f; stored %+.0f (KE %+.0f, PE %+.0f, spring "
              "%+.0f); residual %+.0f J\n",
              what, d.supplied, d.brake, d.tire, d.aero, d.rolling, d.drivetrain, d.suspension,
              d.chassis_damping, d.stored, d.kinetic, d.potential, d.spring, d.residual);
  std::printf("    idle %.0f J, drivetrain split %+.0f J\n", d.idle, d.drivetrain_split);
}

} // namespace

// Gate 1: accelerate to 10 m/s, hold the speed over five hills (the driver throttles up them and
// brakes down them), brake to a stop and stand: the residual stays within 2% of the energy
// supplied.
TEST(PhysicsEnergy, TheLedgerBalancesOverAClosedCycleOnRollingTerrain) {
  REQUIRE_JOLT();
  world w(jolt_config());
  rolling_road(w);
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, road_height(0.0) + 1.4});
  w.drive(v, {});
  w.step(200);
  const vehicle_energy start = w.vehicle_state(v).energy;
  int steps = 0;
  while (speed_of(w, v) < 10.0 && steps++ < 3000) {
    w.drive(v, {0.0, 1.5});
    w.step();
  }
  for (int i = 0; i < 2500; ++i) { // 25 s, about 250 m
    w.drive(v, {0.0, 10.0 - speed_of(w, v)});
    w.step();
  }
  w.drive(v, {0.0, -3.0});
  for (int i = 0; i < 1000 && speed_of(w, v) > 0.05; ++i)
    w.step();
  w.drive(v, {});
  w.step(300);
  const vehicle_state end = w.vehicle_state(v);
  const vehicle_energy d = change(start, end.energy);
  print("closed cycle", d);
  std::printf("  %.0f m, %.3f kg of diesel, %.0f Wh/km\n", d.distance, d.fuel_used,
              end.energy.wh_per_km());
  EXPECT_GT(end.body.position.x, 200.0);
  EXPECT_LT(std::fabs(end.speed), 0.1);
  EXPECT_GT(d.supplied, 1.0e5);
  EXPECT_GT(d.brake, 1.0e4) << "it braked down the hills and to the stop";
  EXPECT_LE(std::fabs(d.residual), 0.02 * d.supplied);
}

// Gate 2: from 15 m/s with no input (no throttle, no brake), the vehicle coasts: nothing is
// supplied, and what it stored falls by what it dissipated.
TEST(PhysicsEnergy, CoastingSuppliesNothingAndLosesWhatItDissipates) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9, 0.015);
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, 1.2});
  w.drive(v, {});
  w.step(200);
  for (int i = 0; i < 3000 && speed_of(w, v) < 15.0; ++i) {
    w.drive(v, {0.0, 2.0});
    w.step();
  }
  w.set_controls(v, {});
  w.step(5); // the throttle let go
  const vehicle_energy start = w.vehicle_state(v).energy;
  w.step(1000);
  const vehicle_energy d = change(start, w.vehicle_state(v).energy);
  print("coast", d);
  const double dissipated = d.dissipated();
  EXPECT_LT(speed_of(w, v), 14.0);
  EXPECT_GT(dissipated, 1.0e4);
  EXPECT_EQ(d.supplied, 0.0) << "nothing is supplied";
  EXPECT_NEAR(d.kinetic + d.potential + d.spring, d.idle - dissipated, 0.02 * dissipated);
}

// Gate 3: released at rest on a 6 degree ramp, in neutral, the vehicle rolls down: the kinetic
// energy it gains is the potential energy it loses, less what it dissipates.
TEST(PhysicsEnergy, CoastingDownhillGainsWhatItFallsLessWhatItDissipates) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const double angle = 6.0 * 3.14159265358979 / 180.0;
  const material_id m = w.add_material({"ramp", 0.9f, 0.0f, 0.015f});
  body_desc ramp;
  ramp.shape = box{{250.0, 20.0, 0.5}};
  ramp.motion = motion_type::static_;
  ramp.rotation = {std::cos(0.5 * angle), 0.0, std::sin(0.5 * angle), 0.0}; // +x end down
  ramp.material = m;
  w.add_body(ramp);
  const double x0 = -150.0;
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(),
                                     {x0, 0.0, -x0 * std::tan(angle) + 0.5 / std::cos(angle) + 1.2},
                                     {std::cos(0.5 * angle), 0.0, std::sin(0.5 * angle), 0.0});
  vehicle_controls hold;
  hold.brake = 1.0;
  w.set_controls(v, hold);
  w.step(300);
  vehicle_controls neutral;
  neutral.neutral = true;
  w.set_controls(v, neutral);
  const vehicle_energy start = w.vehicle_state(v).energy;
  w.step(800);
  const vehicle_energy d = change(start, w.vehicle_state(v).energy);
  print("downhill", d);
  // a = g (sin 6 - C_rr cos 6) = 0.88 m/s^2: about 7 m/s and 57 kJ after 8 s.
  EXPECT_GT(d.kinetic, 4.0e4);
  EXPECT_LT(d.potential, -5.0e4);
  EXPECT_EQ(d.supplied, 0.0);
  // In neutral the engine idles on its own: the idle it is held at, it dissipates.
  EXPECT_NEAR(d.kinetic, -d.potential - d.spring + d.idle - d.dissipated(), 0.02 * d.kinetic);
}

// Gate 4: a battery car brakes by regeneration, and never gets back more than its brakes take:
// regen_efficiency of the braking energy, at most max_regen_power.
TEST(PhysicsEnergy, RegenNeverReturnsMoreThanTheBrakesTake) {
  REQUIRE_JOLT();
  for (const double cap : {150.0, 2.0}) {
    world w(jolt_config());
    flat_ground(w, 0.9, 0.02);
    vehicle_desc d = vehicle_desc::roboracer();
    d.energy.max_regen_power = cap;
    const vehicle_id v = w.add_vehicle(d, {0.0, 0.0, 0.2});
    w.drive(v, {});
    w.step(100);
    bool charged = false;
    vehicle_energy last = w.vehicle_state(v).energy;
    const vehicle_energy start = last;
    for (int cycle = 0; cycle < 3; ++cycle) {
      for (int i = 0; i < 400 && speed_of(w, v) < 5.0; ++i) {
        w.drive(v, {0.0, 4.0});
        w.step();
      }
      for (int i = 0; i < 300; ++i) {
        w.drive(v, {0.0, -6.0});
        w.step();
        const vehicle_energy now = w.vehicle_state(v).energy;
        const vehicle_energy step = change(last, now);
        EXPECT_LE(step.regen, d.energy.regen_efficiency * step.brake + 1e-9);
        EXPECT_LE(step.regen, d.energy.regen_efficiency * cap * 0.01 + 1e-9);
        charged = charged || now.charge > last.charge;
        last = now;
      }
    }
    const vehicle_energy total = change(start, last);
    print("regen cycles", total);
    std::printf("  regen cap %.0f W: brakes took %.1f J, the battery got back %.1f J (%.0f%%); "
                "drew %.1f J for %.1f J of shaft work; charge %.5f\n",
                cap, total.brake, total.regen, 100.0 * total.regen / total.brake, total.source,
                total.engine, last.charge);
    EXPECT_GT(total.brake, 0.0);
    EXPECT_LE(total.regen, d.energy.regen_efficiency * total.brake);
    EXPECT_TRUE(charged) << "braking charged the battery";
    EXPECT_NEAR(total.source, (total.engine + total.idle) / d.energy.motor_efficiency,
                1e-6 * total.source + 1e-9);
    EXPECT_NEAR(last.charge, start.charge - (total.source - total.regen) / d.energy.capacity, 1e-9);
    EXPECT_LE(std::fabs(total.residual), 0.02 * (total.supplied + total.brake));
    EXPECT_GE(total.drivetrain, 0.0);
  }
}

// Fuel: shaft work over the thermal efficiency times the heating value, plus the idle flow; and
// an empty tank gives no more throttle.
TEST(PhysicsEnergy, FuelBurnsShaftWorkOverItsEfficiencyAndIdles) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9, 0.015);
  vehicle_desc d = vehicle_desc::humvee();
  d.energy.tank = 0.05; // kg: about 26 s of driving here
  const vehicle_id v = w.add_vehicle(d, {0.0, 0.0, 1.2});
  w.drive(v, {});
  w.step(100);
  const vehicle_energy start = w.vehicle_state(v).energy;
  for (int i = 0; i < 1500; ++i) {
    w.drive(v, {0.0, 1.0});
    w.step();
  }
  const vehicle_energy now = w.vehicle_state(v).energy;
  const vehicle_energy e = change(start, now);
  const double expected =
      e.engine / (d.energy.thermal_efficiency * d.energy.heating_value) + d.energy.idle_flow * 15.0;
  std::printf("  15 s: %.0f J of shaft work, %.5f kg of fuel (%.0f Wh/km over %.0f m)\n", e.engine,
              e.fuel_used, now.wh_per_km(), now.distance);
  EXPECT_NEAR(e.fuel_used, expected, 1e-9);
  EXPECT_NEAR(e.source, e.fuel_used * d.energy.heating_value, 1e-6 * e.source);
  EXPECT_NEAR(now.wh_per_km(), now.source / 3600.0 / (now.distance / 1000.0), 1e-9);
  EXPECT_NEAR(now.fuel_left, d.energy.tank - now.fuel_used, 1e-12);
  // Drive on until the tank is empty: then the throttle stays shut.
  for (int i = 0; i < 6000 && w.vehicle_state(v).energy.fuel_left > 0.0; ++i) {
    w.drive(v, {0.0, 1.0});
    w.step();
  }
  ASSERT_EQ(w.vehicle_state(v).energy.fuel_left, 0.0);
  w.drive(v, {0.0, 1.0});
  w.step(50);
  EXPECT_EQ(w.vehicle_state(v).throttle, 0.0);
}

// The measured step is Jolt's own: with and without the ledger, the same drive leaves the same
// state, bit for bit.
TEST(PhysicsEnergy, MeasuringTheLedgerChangesNothing) {
  REQUIRE_JOLT();
  const auto run = [](bool ledger) {
    world w(jolt_config());
    rolling_road(w);
    vehicle_desc d = vehicle_desc::humvee();
    d.energy_ledger = ledger;
    const vehicle_id v = w.add_vehicle(d, {0.0, 0.0, road_height(0.0) + 1.4});
    for (int i = 0; i < 900; ++i) {
      w.drive(v, {0.25 * std::sin(0.01 * i), i < 500 ? 2.0 : -4.0});
      w.step();
    }
    return w.vehicle_state(v);
  };
  const vehicle_state a = run(false), b = run(true);
  EXPECT_EQ(a.energy.supplied, 0.0) << "no ledger, no measurements";
  EXPECT_GT(b.energy.supplied, 0.0);
  EXPECT_EQ(a.body.position.x, b.body.position.x);
  EXPECT_EQ(a.body.position.z, b.body.position.z);
  EXPECT_EQ(a.body.linear_velocity.x, b.body.linear_velocity.x);
  EXPECT_EQ(a.body.rotation.w, b.body.rotation.w);
  EXPECT_EQ(a.engine_rpm, b.engine_rpm);
  EXPECT_EQ(a.gear, b.gear);
  ASSERT_EQ(a.wheels.size(), b.wheels.size());
  for (std::size_t i = 0; i < a.wheels.size(); ++i) {
    EXPECT_EQ(a.wheels[i].angular_velocity, b.wheels[i].angular_velocity);
    EXPECT_EQ(a.wheels[i].suspension_length, b.wheels[i].suspension_length);
    EXPECT_EQ(a.wheels[i].load, b.wheels[i].load);
  }
}

// The ledger is state: a save, a restore into a fresh world and the rest of the drive end where
// the original did, totals included.
TEST(PhysicsEnergy, TheLedgerReplaysAfterARestore) {
  REQUIRE_JOLT();
  const auto build = [](world &w) {
    rolling_road(w);
    return w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, road_height(0.0) + 1.4});
  };
  const auto drive = [](world &w, vehicle_id v, int from, int to) {
    for (int i = from; i < to; ++i) {
      w.drive(v, {0.1 * std::sin(0.01 * i), i < 400 ? 2.0 : -3.0});
      w.step();
    }
  };
  world a(jolt_config());
  const vehicle_id va = build(a);
  drive(a, va, 0, 300);
  const std::vector<std::uint8_t> mid = a.save();
  drive(a, va, 300, 600);
  world c(jolt_config());
  const vehicle_id vc = build(c);
  c.restore(mid);
  drive(c, vc, 300, 600);
  const vehicle_energy ea = a.vehicle_state(va).energy, ec = c.vehicle_state(vc).energy;
  EXPECT_EQ(ea.supplied, ec.supplied);
  EXPECT_EQ(ea.residual, ec.residual);
  EXPECT_EQ(ea.fuel_used, ec.fuel_used);
  EXPECT_EQ(c.state_hash(), a.state_hash());
}

// A published vehicle's ledger is in the state tree (doc 31's D11 names) and in the JSON
// snapshot, to the last digit.
TEST(PhysicsEnergy, ThePublishedVehicleCarriesItsLedger) {
  REQUIRE_JOLT();
  cvc::app a;
  world w(jolt_config());
  world_binding b(a, w);
  flat_ground(w, 0.9, 0.015);
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, 1.2});
  b.publish_vehicle("truck", v);
  cvc::state::instance(a)("physics.snapshot_json_every").value(std::string("1"));
  cvc::state::instance(a)("physics.vehicles.truck.command.accel").value(std::string("1.5"));
  for (int i = 0; i < 300; ++i)
    b.step();
  const vehicle_energy e = w.vehicle_state(v).energy;
  const auto key = [&a](const char *k) {
    return std::stod(
        cvc::state::instance(a)(std::string("physics.vehicles.truck.energy.") + k).value());
  };
  EXPECT_GT(e.supplied, 0.0);
  EXPECT_EQ(key("supplied_J"), e.supplied);
  EXPECT_EQ(key("residual_J"), e.residual);
  EXPECT_EQ(key("roll_J"), e.rolling);
  EXPECT_EQ(key("fuel_kg"), e.fuel_used);
  EXPECT_EQ(key("P_W"), e.power);
  EXPECT_EQ(key("Wh_per_km"), e.wh_per_km());
  std::istringstream in(cvc::state::instance(a)("physics.snapshot_json").value());
  boost::property_tree::ptree doc;
  boost::property_tree::read_json(in, doc);
  const boost::property_tree::ptree &truck = doc.get_child("vehicles").begin()->second;
  EXPECT_EQ(std::stod(truck.get<std::string>("state.energy.supplied")), e.supplied);
  EXPECT_EQ(truck.get<std::string>("desc.energy_source.type"), "fuel");
}

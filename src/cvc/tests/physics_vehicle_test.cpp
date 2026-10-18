/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::physics vehicles (vehicle.h). The P2 gates of
// docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md are envelopes, not curves:
//   (1) the Ackermann relation within 1e-4 rad, and the steer never slews past its rate;
//   (2) the driver model never presses throttle and brake together;
//   (3) the stopping distance within 10% of v^2 / (2 mu g);
//   (4) skidpad lateral acceleration at most mu g + 10%;
//   (5) a coastdown against the drag and rolling model, the rolling resistance from the
//       ground's material;
// and the vehicle reading the world's fields (field.h):
//   (6) a wheel in snow sinks to the Bekker depth and rides on the compacted surface;
//   (7) the snow's compaction resistance and traction;
//   (8) the aero takes a flow field's wind;
//   (9) the same run twice, and a save / restore / re-run, give the same hash.
// Jolt only; the refusals run on the null backend too.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace cvc::physics;

namespace {

constexpr double kG = 9.80665;

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

world_config jolt_config() {
  world_config c;
  c.backend = backend::jolt;
  return c;
}

cvc::app &ctx() {
  static cvc::app app;
  return app;
}

// A long flat ground, top at z = 0, of friction mu and rolling resistance crr.
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

// A Humvee standing on the ground at (x, y), settled.
vehicle_id settled_humvee(world &w, double x = 0.0, double y = 0.0) {
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {x, y, 1.2});
  w.drive(v, {});
  w.step(200);
  return v;
}

double speed_of(const world &w, vehicle_id v) { return w.vehicle_state(v).speed; }

// Drive straight up to `v0` with the throttle.
void accelerate_to(world &w, vehicle_id v, double v0) {
  for (int i = 0; i < 6000 && speed_of(w, v) < v0; ++i) {
    vehicle_controls c;
    c.throttle = 1.0;
    w.set_controls(v, c);
    w.step();
  }
}

std::shared_ptr<cvc::volume> uniform(double value, vec3 lo, vec3 hi) {
  auto v = std::make_shared<cvc::volume>(ctx(), cvc::dimension(2, 2, 2), cvc::Float,
                                         cvc::bounding_box(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z));
  v->fill(value);
  return v;
}

double cot(double a) { return std::cos(a) / std::sin(a); }

} // namespace

// --- refusals ------------------------------------------------------------------------

TEST(PhysicsVehicle, DescriptionsAndBackendsAreChecked) {
  world_config c;
  c.backend = backend::null;
  world n(c);
  EXPECT_THROW(n.add_vehicle(vehicle_desc::humvee(), {0, 0, 1}), cvc::unsupported_exception);
  if (!have_jolt())
    return;
  world w(jolt_config());
  vehicle_desc none = vehicle_desc::humvee();
  none.axles.clear();
  EXPECT_THROW(w.add_vehicle(none, {0, 0, 1}), cvc::physics_error);
  vehicle_desc heavy = vehicle_desc::humvee();
  heavy.mass = -1.0;
  EXPECT_THROW(w.add_vehicle(heavy, {0, 0, 1}), cvc::physics_error);
  vehicle_desc front_only = vehicle_desc::humvee();
  front_only.axles.pop_back(); // only the steered axle: no rear for Ackermann
  EXPECT_THROW(w.add_vehicle(front_only, {0, 0, 1}), cvc::physics_error);
  EXPECT_THROW(w.add_vehicle(vehicle_desc::humvee(), {0, 0, 1}, {2, 0, 0, 0}), cvc::physics_error);
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {0, 0, 1});
  EXPECT_EQ(w.vehicle_count(), 1u);
  const body_id chassis = w.vehicle_state(v).chassis;
  EXPECT_TRUE(w.contains(chassis));
  EXPECT_THROW(w.remove_body(chassis), cvc::physics_error);
  EXPECT_THROW(w.drive(v, {std::nan(""), 0.0}), cvc::physics_error);
  w.remove_vehicle(v);
  EXPECT_EQ(w.vehicle_count(), 0u);
  EXPECT_FALSE(w.contains(chassis));
  EXPECT_THROW(w.vehicle_state(v), cvc::physics_error);
}

// --- steering --------------------------------------------------------------------------

// Gate (1): the actuator slews at no more than max_steer_rate, and every steered wheel takes
// its Ackermann angle: cot(right) - cot(left) = track / wheelbase on a left turn.
TEST(PhysicsVehicle, TheSteerSlewsAtItsRateAndTheWheelsFollowAckermann) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  const vehicle_desc d = vehicle_desc::humvee();
  const double L = d.wheelbase(), track = d.track();
  EXPECT_NEAR(L, 3.30, 1e-12);
  EXPECT_NEAR(track, 1.82, 1e-12);
  const double rate = d.max_steer_rate * w.config().fixed_dt;
  for (const double target : {0.3, -0.42, 0.1, 0.45, 0.0}) {
    w.drive(v, {target, 0.0});
    double prev = w.vehicle_state(v).steer;
    for (int i = 0; i < 300; ++i) {
      w.step();
      const vehicle_state s = w.vehicle_state(v);
      EXPECT_LE(std::fabs(s.steer - prev), rate + 1e-12) << "target " << target;
      prev = s.steer;
      if (std::fabs(s.steer) > 1e-3) {
        // Front wheels: the Ackermann angles of the bicycle angle, to float precision.
        const double t = std::tan(s.steer);
        const double want_left = std::atan(L * t / (L - 0.91 * t));
        const double want_right = std::atan(L * t / (L + 0.91 * t));
        ASSERT_EQ(s.wheels.size(), 4u);
        EXPECT_NEAR(s.wheels[0].steer, want_left, 1e-4);
        EXPECT_NEAR(s.wheels[1].steer, want_right, 1e-4);
        EXPECT_NEAR(cot(s.wheels[1].steer) - cot(s.wheels[0].steer), track / L, 1e-3);
        EXPECT_EQ(s.wheels[2].steer, 0.0); // the rear axle does not steer
        EXPECT_EQ(s.wheels[3].steer, 0.0);
      }
    }
    EXPECT_NEAR(w.vehicle_state(v).steer, target, 1e-12) << "reached";
  }
  // Past the limit: clamped.
  w.drive(v, {2.0, 0.0});
  w.step(300);
  EXPECT_NEAR(w.vehicle_state(v).steer, d.max_steer, 1e-12);
}

// --- the driver ------------------------------------------------------------------------

// Gate (2): whatever a_cmd does, the driver presses the throttle or the brake, never both.
TEST(PhysicsVehicle, TheDriverNeverOverlapsThrottleAndBrake) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  std::uint64_t seed = 12345;
  int throttled = 0, braked = 0;
  for (int i = 0; i < 3000; ++i) {
    if (i % 60 == 0) {
      seed = seed * 6364136223846793005ull + 1442695040888963407ull;
      const double a = -6.0 + 9.0 * static_cast<double>(seed >> 40) / double(1u << 24);
      w.drive(v, {0.2 * std::sin(0.01 * i), a, (i / 900) % 3 == 2});
    }
    w.step();
    const vehicle_state s = w.vehicle_state(v);
    EXPECT_EQ(s.throttle * s.brake, 0.0) << "step " << i;
    throttled += s.throttle > 0.0;
    braked += s.brake > 0.0;
  }
  EXPECT_GT(throttled, 300);
  EXPECT_GT(braked, 300);
}

// Asked to reverse from rest, it backs up in reverse gear.
TEST(PhysicsVehicle, ReverseBacksUp) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  w.drive(v, {0.0, 1.0, true});
  w.step(300);
  const vehicle_state s = w.vehicle_state(v);
  EXPECT_LT(s.speed, -0.5);
  EXPECT_EQ(s.gear, -1);
}

// --- envelopes ---------------------------------------------------------------------------

// Gate (3): from 20 m/s, a full brake stops within 10% of v^2 / (2 mu g).
TEST(PhysicsVehicle, TheStoppingDistanceIsTheFrictionLimit) {
  REQUIRE_JOLT();
  const double mu = 0.8;
  world w(jolt_config());
  flat_ground(w, mu);
  const vehicle_id v = settled_humvee(w, -1200.0, 0.0);
  accelerate_to(w, v, 20.0);
  const vehicle_state s0 = w.vehicle_state(v);
  const double v0 = s0.speed;
  ASSERT_GT(v0, 19.5);
  vehicle_controls stop;
  stop.brake = 1.0;
  w.set_controls(v, stop);
  int steps = 0;
  while (speed_of(w, v) > 0.05 && steps < 2000) {
    w.step();
    ++steps;
  }
  const vehicle_state s1 = w.vehicle_state(v);
  const double d =
      std::hypot(s1.body.position.x - s0.body.position.x, s1.body.position.y - s0.body.position.y);
  const double ideal = v0 * v0 / (2.0 * mu * kG);
  std::printf("  stopped from %.2f m/s in %.2f m (v^2/2mug = %.2f m), %d steps\n", v0, d, ideal,
              steps);
  EXPECT_NEAR(d, ideal, 0.10 * ideal);
}

// Gate (4): on full lock with the throttle wound up, the lateral acceleration never passes
// mu g by more than 10%, and the run gets near the limit.
TEST(PhysicsVehicle, SkidpadLateralStaysWithinTheFrictionLimit) {
  REQUIRE_JOLT();
  const double mu = 0.7;
  world w(jolt_config());
  flat_ground(w, mu);
  const vehicle_id v = settled_humvee(w);
  double peak = 0.0;
  for (int i = 0; i < 2400; ++i) {
    vehicle_controls c;
    c.steer = 0.45;
    c.throttle = std::min(1.0, i / 1500.0);
    w.set_controls(v, c);
    w.step();
    if (i < 600)
      continue; // the steer slews in and the circle settles
    const vehicle_state s = w.vehicle_state(v);
    const double a_lat = std::fabs(s.speed * s.body.angular_velocity.z);
    peak = std::max(peak, a_lat);
  }
  std::printf("  peak lateral %.2f m/s^2 (mu g = %.2f)\n", peak, mu * kG);
  EXPECT_LE(peak, 1.10 * mu * kG);
  EXPECT_GE(peak, 0.6 * mu * kG);
}

// Gate (5): coasting in neutral, the speed follows dv/dt = -(C_rr g + rho C_d A v^2 / (2 m)),
// m including the wheels' rotational inertia; C_rr comes from the ground's material when it
// has one.
TEST(PhysicsVehicle, CoastdownFollowsTheDragAndRollingModel) {
  REQUIRE_JOLT();
  for (const double crr_ground : {0.0, 0.04}) {
    world w(jolt_config());
    flat_ground(w, 0.9, crr_ground);
    const vehicle_id v = settled_humvee(w, -1200.0, 0.0);
    accelerate_to(w, v, 20.0);
    vehicle_controls coast;
    coast.neutral = true;
    w.set_controls(v, coast);
    w.step(30); // the clutch lets go
    const vehicle_desc d = vehicle_desc::humvee();
    const double crr = crr_ground > 0.0 ? crr_ground : d.rolling_resistance;
    double m_eff = d.mass;
    for (const axle_desc &a : d.axles)
      m_eff += 2.0 * a.wheel_inertia / (a.wheel_radius * a.wheel_radius);
    const double k = 0.5 * d.air_density * d.drag_coefficient * d.frontal_area;
    double model = speed_of(w, v);
    const double dt = w.config().fixed_dt;
    for (int i = 0; i < 1500; ++i) { // 15 s
      w.step();
      model -= dt * (crr * kG * d.mass + k * model * model) / m_eff;
    }
    const vehicle_state s = w.vehicle_state(v);
    std::printf("  C_rr %.3f: coasted to %.3f m/s, the model %.3f m/s\n", crr, s.speed, model);
    EXPECT_NEAR(s.speed, model, 0.03 * model);
    // Applied from the previous step's load, which moves a little step to step.
    EXPECT_NEAR(s.wheels[0].rolling_resistance, crr * s.wheels[0].load,
                0.03 * crr * s.wheels[0].load);
  }
}

// --- media ---------------------------------------------------------------------------------

// Gate (6): in 0.4 m of snow of modulus k, each wheel sinks z = (3 N / (2 b k sqrt(D)))^(2/3)
// and rides on the compacted surface: the chassis stands depth - z higher than on bare ground.
TEST(PhysicsVehicle, WheelsSinkToTheBekkerDepthAndRideOnTheSnow) {
  REQUIRE_JOLT();
  const double k = 1.0e6, depth = 0.4;
  world bare(jolt_config());
  flat_ground(bare, 0.9);
  const vehicle_id vb = settled_humvee(bare);
  world snowy(jolt_config());
  flat_ground(snowy, 0.9);
  field snow;
  snow.kind = field_kind::medium;
  snow.volume = uniform(k, {-50, -50, 0.0}, {50, 50, depth});
  snow.traction = 0.3;
  snowy.add_field(snow);
  const vehicle_id vs = snowy.add_vehicle(vehicle_desc::humvee(), {0, 0, 1.6});
  snowy.drive(vs, {});
  snowy.step(400);
  const vehicle_state a = bare.vehicle_state(vb), b = snowy.vehicle_state(vs);
  const axle_desc wheel = vehicle_desc::humvee().axles[0];
  double rise = 0.0;
  for (std::size_t i = 0; i < b.wheels.size(); ++i) {
    const wheel_state &ws = b.wheels[i];
    ASSERT_TRUE(ws.contact);
    EXPECT_NEAR(ws.medium_depth, depth, 1e-3);
    const double want = std::pow(
        3.0 * ws.load / (2.0 * wheel.wheel_width * k * std::sqrt(2.0 * wheel.wheel_radius)),
        2.0 / 3.0);
    EXPECT_NEAR(ws.sinkage, want, 0.02 * want) << "wheel " << i;
    EXPECT_NEAR(ws.contact_point.z, depth - ws.sinkage, 0.01) << "wheel " << i;
    EXPECT_NEAR(ws.friction, 0.3, 1e-6) << "the snow's traction";
    rise += (depth - ws.sinkage) / b.wheels.size();
  }
  std::printf("  sinkage %.4f m, chassis %.4f m higher than on bare ground (depth - z = %.4f)\n",
              b.wheels[0].sinkage, b.body.position.z - a.body.position.z, rise);
  EXPECT_NEAR(b.body.position.z - a.body.position.z, rise, 0.01);
}

// Gate (7): coasting into the snow, the compaction resistance b k z^2 / 2 per wheel slows the
// vehicle at (sum R + C_rr m g) / m.
TEST(PhysicsVehicle, SnowHoldsTheVehicleBackByItsCompactionResistance) {
  REQUIRE_JOLT();
  const double k = 1.0e6, depth = 0.4;
  world w(jolt_config());
  flat_ground(w, 0.9);
  field snow;
  snow.kind = field_kind::medium;
  snow.volume = uniform(k, {-300, -50, 0.0}, {300, 50, depth});
  w.add_field(snow);
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {-250, 0, 1.6});
  w.drive(v, {});
  w.step(300);
  accelerate_to(w, v, 6.0);
  vehicle_controls coast;
  coast.neutral = true;
  w.set_controls(v, coast);
  w.step(30);
  const vehicle_state s0 = w.vehicle_state(v);
  double resistance = 0.0;
  for (const wheel_state &ws : s0.wheels)
    resistance += ws.medium_resistance + ws.rolling_resistance;
  w.step(50);
  const vehicle_state s1 = w.vehicle_state(v);
  const double decel = (s0.speed - s1.speed) / (50 * w.config().fixed_dt);
  const vehicle_desc d = vehicle_desc::humvee();
  const double z = s0.wheels[0].sinkage;
  const double per_wheel = 0.5 * d.axles[0].wheel_width * k * z * z;
  std::printf("  sinkage %.4f m: %.0f N a wheel, decel %.2f m/s^2 (model %.2f)\n", z, per_wheel,
              decel, resistance / d.mass);
  // Applied from the previous step's sinkage.
  EXPECT_NEAR(s0.wheels[0].medium_resistance, per_wheel, 0.02 * per_wheel);
  EXPECT_NEAR(decel, resistance / d.mass, 0.12 * resistance / d.mass);
}

// Gate (8): in a flow field the aero uses the air's density and wind.
TEST(PhysicsVehicle, TheAeroTakesTheFlowFieldsWind) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  field air;
  air.kind = field_kind::flow;
  const vec3 lo{-2000, -200, -10}, hi{2000, 200, 50};
  air.volume = uniform(1.1, lo, hi);
  air.velocity = {uniform(-8.0, lo, hi), uniform(3.0, lo, hi), uniform(0.0, lo, hi)};
  w.add_field(air);
  const vehicle_id v = settled_humvee(w, -1200.0, 0.0);
  accelerate_to(w, v, 12.0);
  w.step();
  const vehicle_state s = w.vehicle_state(v);
  EXPECT_NEAR(s.air_velocity.x, -8.0, 1e-9);
  EXPECT_NEAR(s.air_velocity.y, 3.0, 1e-9);
  // The force applied this step was from the velocity at its start; check its direction and
  // size against the relative air velocity now (they differ by one step's change).
  const vec3 u = s.body.linear_velocity - s.air_velocity;
  const double un = std::sqrt(dot(u, u));
  const vehicle_desc d = vehicle_desc::humvee();
  const double want = 0.5 * 1.1 * d.drag_coefficient * d.frontal_area * un * un;
  const double got = std::sqrt(dot(s.aero_force, s.aero_force));
  EXPECT_NEAR(got, want, 0.01 * want);
  EXPECT_LT(dot(s.aero_force, u), 0.0); // against the relative air
}

// --- determinism -----------------------------------------------------------------------------

// Gate (9): the same drive twice gives the same hash; a save, restore into a fresh world and
// re-run reproduces the original.
TEST(PhysicsVehicle, TheSameDriveTwiceAndAReplayGiveTheSameHash) {
  REQUIRE_JOLT();
  const auto build = [](world &w) {
    flat_ground(w, 0.85);
    field snow;
    snow.kind = field_kind::medium;
    snow.volume = uniform(5.0e5, {4, -20, 0.0}, {40, 20, 0.3});
    snow.ruts = true; // the rut maps are state too
    w.add_field(snow);
    return w.add_vehicle(vehicle_desc::humvee(), {0, 0, 1.2});
  };
  const auto drive = [](world &w, vehicle_id v, int from, int to) {
    for (int i = from; i < to; ++i) {
      w.drive(v, {0.3 * std::sin(0.004 * i), i < 400 ? 2.0 : -1.0});
      w.step();
    }
  };
  world a(jolt_config()), b(jolt_config());
  const vehicle_id va = build(a), vb = build(b);
  drive(a, va, 0, 300);
  const std::vector<std::uint8_t> mid = a.save();
  drive(a, va, 300, 600);
  drive(b, vb, 0, 600);
  EXPECT_EQ(a.state_hash(), b.state_hash());
  world c(jolt_config());
  const vehicle_id vc = build(c);
  c.restore(mid);
  drive(c, vc, 300, 600);
  EXPECT_EQ(c.state_hash(), a.state_hash());
  EXPECT_GT(a.vehicle_state(va).body.position.x, 5.0) << "it drove into the snow";
}

// The 1/10-scale racer drives straight and stays upright.
TEST(PhysicsVehicle, TheRoboRacerDrives) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 1.0);
  const vehicle_id v = w.add_vehicle(vehicle_desc::roboracer(), {0, 0, 0.2});
  w.drive(v, {});
  w.step(200);
  w.drive(v, {0.0, 3.0});
  w.step(150);
  const vehicle_state s = w.vehicle_state(v);
  std::printf("  roboracer at %.2f m/s after 1.5 s, gear %d, %.0f rpm\n", s.speed, s.gear,
              s.engine_rpm);
  EXPECT_GT(s.speed, 2.0);
  EXPECT_LT(std::fabs(s.body.position.y), 0.2);
  EXPECT_GT(
      1.0 - 2.0 * (s.body.rotation.x * s.body.rotation.x + s.body.rotation.y * s.body.rotation.y),
      0.95); // upright
}

// --- grades ----------------------------------------------------------------------------------

namespace {

// A ground tilted about y so the chassis's forward axis points uphill at `grade` (rise / run),
// and a Humvee settled on it facing uphill.
struct hill {
  world w{jolt_config()};
  vehicle_id v;
  double theta = 0.0;
  quat q;
  hill(double grade, double mu, double x = 0.0) {
    theta = std::atan(grade);
    q = {std::cos(-theta / 2), 0.0, std::sin(-theta / 2), 0.0}; // +x tilted up
    const material_id m = w.add_material({"ground", static_cast<float>(mu)});
    body_desc g;
    g.shape = box{{400.0, 60.0, 0.5}};
    g.motion = motion_type::static_;
    g.rotation = q;
    g.position = {0.5 * std::sin(theta), 0.0, -0.5 * std::cos(theta)}; // top through the origin
    g.material = m;
    w.add_body(g);
    const vec3 up{-std::sin(theta), 0.0, std::cos(theta)};
    v = w.add_vehicle(vehicle_desc::humvee(),
                      {x * std::cos(theta) + 1.2 * up.x, 0.0, x * std::sin(theta) + 1.2 * up.z}, q);
  }
  // Distance along the slope (uphill +) of the chassis from the origin.
  double along() const {
    const vec3 p = w.vehicle_state(v).body.position;
    return p.x * std::cos(theta) + p.z * std::sin(theta);
  }
};

} // namespace

// On a 15% grade the driver, asked for nothing, holds on its brakes.
TEST(PhysicsVehicle, TheDriverHoldsOnAGrade) {
  REQUIRE_JOLT();
  hill h(0.15, 0.9);
  h.w.drive(h.v, {});
  h.w.step(300);
  const double x0 = h.along();
  double moved = 0.0;
  for (int i = 0; i < 1000; ++i) {
    h.w.step();
    moved = std::max(moved, std::fabs(h.along() - x0));
  }
  std::printf("  held on 15%%: drifted %.4f m in 10 s, brake %.2f\n", moved,
              h.w.vehicle_state(h.v).brake);
  EXPECT_LT(moved, 0.02);
  EXPECT_EQ(h.w.vehicle_state(h.v).brake, 1.0);
}

// The hand brake alone (the rear axle) holds it there too.
TEST(PhysicsVehicle, TheHandBrakeHoldsOnAGrade) {
  REQUIRE_JOLT();
  hill h(0.15, 0.9);
  h.w.drive(h.v, {});
  h.w.step(300);
  vehicle_controls park;
  park.hand_brake = 1.0;
  h.w.set_controls(h.v, park);
  h.w.step(50);
  const double x0 = h.along();
  h.w.step(1000);
  std::printf("  hand brake on 15%%: drifted %.4f m in 10 s\n", std::fabs(h.along() - x0));
  EXPECT_LT(std::fabs(h.along() - x0), 0.02);
}

// Released in neutral it rolls back down at g (sin(theta) - C_rr cos(theta)), slowed by the wheels'
// rotational inertia.
TEST(PhysicsVehicle, ReleasedOnAGradeItRollsAtTheGradeLimit) {
  REQUIRE_JOLT();
  hill h(0.15, 0.9);
  h.w.drive(h.v, {});
  h.w.step(300);
  vehicle_controls loose;
  loose.neutral = true;
  h.w.set_controls(h.v, loose);
  const double x0 = h.along();
  h.w.step(300); // 3 s
  const vehicle_desc d = vehicle_desc::humvee();
  double m_eff = d.mass;
  for (const axle_desc &a : d.axles)
    m_eff += 2.0 * a.wheel_inertia / (a.wheel_radius * a.wheel_radius);
  const double a_model =
      kG * (std::sin(h.theta) - d.rolling_resistance * std::cos(h.theta)) * d.mass / m_eff;
  const double t = 3.0;
  const double want = 0.5 * a_model * t * t; // aero is under 0.1% at these speeds
  const double got = x0 - h.along();
  std::printf("  rolled %.3f m down in 3 s (model %.3f m)\n", got, want);
  EXPECT_NEAR(got, want, 0.05 * want);
}

// A hill start: from holding, a_cmd uphill moves it uphill, never rolling back past 0.3 m.
TEST(PhysicsVehicle, AHillStartDrivesUphill) {
  REQUIRE_JOLT();
  hill h(0.15, 0.9);
  h.w.drive(h.v, {});
  h.w.step(300);
  const double x0 = h.along();
  h.w.drive(h.v, {0.0, 1.0});
  double worst = 0.0;
  for (int i = 0; i < 600; ++i) {
    h.w.step();
    worst = std::min(worst, h.along() - x0);
  }
  const vehicle_state s = h.w.vehicle_state(h.v);
  std::printf("  hill start: %.2f m/s uphill after 6 s, rolled back at most %.3f m\n", s.speed,
              -worst);
  EXPECT_GT(s.speed, 1.0);
  EXPECT_GT(h.along() - x0, 3.0);
  EXPECT_GT(worst, -0.3);
}

// --- ruts --------------------------------------------------------------------------------------

namespace {

// Rutted snow, 0.4 m deep, over a long strip, and a world to drive in it.
field rutted_snow() {
  field snow;
  snow.kind = field_kind::medium;
  snow.volume = uniform(1.0e6, {-300, -20, 0.0}, {300, 20, 0.4});
  snow.ruts = true;
  snow.rut_cell = 0.05;
  return snow;
}

// Drive `v` at about 3 m/s for `steps`.
void cruise(world &w, const std::vector<vehicle_id> &vs, int steps) {
  for (int i = 0; i < steps; ++i) {
    for (const vehicle_id v : vs)
      w.drive(v, {0.0, speed_of(w, v) < 3.0 ? 1.0 : 0.0});
    w.step();
  }
}

} // namespace

// A vehicle leaves its ruts behind it: along both wheel tracks, as deep as the wheels sank and as
// wide as the tires, and nothing between them.
TEST(PhysicsVehicle, RutsStayBehindAVehicle) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const field_id snow = w.add_field(rutted_snow());
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {-250, 0, 1.6});
  w.drive(v, {});
  w.step(300);
  cruise(w, {v}, 1200);
  const double z = w.vehicle_state(v).wheels[0].sinkage;
  const rut_map ruts = w.field_ruts(snow);
  ASSERT_GT(ruts.cols, 0);
  // Sampled over 10 m the vehicle drove through, behind where it is now.
  const double x_now = w.vehicle_state(v).body.position.x;
  double mean = 0.0;
  int n = 0;
  for (double x = x_now - 20.0; x < x_now - 10.0; x += 0.5)
    for (const double y : {0.91, -0.91}) {
      mean += ruts.at(x, y);
      ++n;
      EXPECT_EQ(ruts.at(x, 0.0), 0.0f) << "between the tracks";
      EXPECT_EQ(ruts.at(x, y + (y > 0 ? 0.3 : -0.3)), 0.0f) << "outside the track";
    }
  mean /= n;
  // A track's width: the cells across it with a rut.
  int across = 0;
  for (double y = 0.5; y < 1.4; y += ruts.cell)
    across += ruts.at(x_now - 15.0, y) > 0.0f;
  std::printf("  rut %.4f m deep (the wheels sink %.4f m), %.2f m wide (tire %.2f m), x now %.1f\n",
              mean, z, across * ruts.cell, vehicle_desc::humvee().axles[0].wheel_width, x_now);
  EXPECT_NEAR(mean, z, 0.10 * z);
  EXPECT_NEAR(across * ruts.cell, vehicle_desc::humvee().axles[0].wheel_width, 2.5 * ruts.cell);
  // The snapshot counts the ruts as the map holds them.
  std::size_t cells = 0;
  float deepest = 0.0f;
  for (const float d : ruts.depth)
    if (d > 0.0f) {
      ++cells;
      deepest = std::max(deepest, d);
    }
  const world_snapshot snap = w.snapshot();
  ASSERT_EQ(snap.fields.size(), 1u);
  EXPECT_EQ(snap.fields[0].rut_cells, cells);
  EXPECT_EQ(snap.fields[0].rut_deepest, static_cast<double>(deepest));
  // New traction keeps the ruts; a new rut_cell starts them over.
  field f = w.get_field(snow);
  f.traction = 0.25;
  w.set_field(snow, f);
  EXPECT_EQ(w.field_ruts(snow).depth, ruts.depth);
  EXPECT_EQ(w.snapshot().fields[0].rut_cells, cells);
  f.rut_cell = 0.1;
  w.set_field(snow, f);
  EXPECT_EQ(w.snapshot().fields[0].rut_cells, 0u);
  EXPECT_EQ(w.field_ruts(snow).at(x_now - 15.0, 0.91), 0.0);
}

// A second vehicle in the first one's ruts sinks no deeper and is barely held back: the snow
// there is compacted already. Its rear wheels do the same in its own front wheels' ruts.
TEST(PhysicsVehicle, ASecondPassInTheRutsSinksNoFurtherAndRollsEasier) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  w.add_field(rutted_snow());
  const vehicle_id lead = w.add_vehicle(vehicle_desc::humvee(), {-250, 0, 1.6});
  const vehicle_id follow = w.add_vehicle(vehicle_desc::humvee(), {-265, 0, 1.6});
  w.drive(lead, {});
  w.drive(follow, {});
  w.step(300);
  cruise(w, {lead, follow}, 1500);
  const vehicle_state a = w.vehicle_state(lead), b = w.vehicle_state(follow);
  ASSERT_LT(b.body.position.x, a.body.position.x - 8.0);
  ASSERT_GT(b.body.position.x, -240.0) << "the follower is on the leader's ruts";
  const double lead_front = a.wheels[0].medium_resistance + a.wheels[1].medium_resistance;
  const double follow_front = b.wheels[0].medium_resistance + b.wheels[1].medium_resistance;
  const double lead_rear = a.wheels[2].medium_resistance + a.wheels[3].medium_resistance;
  std::printf("  compaction resistance: leader front %.0f N, rear %.0f N; follower front %.0f N\n",
              lead_front, lead_rear, follow_front);
  std::printf("  sinkage: leader front %.4f m, follower front %.4f m (rut found %.4f m)\n",
              a.wheels[0].sinkage, b.wheels[0].sinkage, b.wheels[0].rut);
  EXPECT_GT(lead_front, 1000.0);
  EXPECT_LT(follow_front, 0.10 * lead_front);
  EXPECT_LT(lead_rear, 0.10 * lead_front); // in its own front wheels' ruts
  EXPECT_NEAR(b.wheels[0].sinkage, a.wheels[0].sinkage, 0.05 * a.wheels[0].sinkage);
  EXPECT_NEAR(b.wheels[0].rut, a.wheels[0].sinkage, 0.10 * a.wheels[0].sinkage);
}

// --- tuning, the snapshot and the state tree ---------------------------------------------------

namespace {

std::string tree(cvc::app &a, const std::string &path) {
  return cvc::state::instance(a)(path).value();
}
double tree_num(cvc::app &a, const std::string &path) { return std::stod(tree(a, path)); }
void set_tree(cvc::app &a, const std::string &path, const std::string &v) {
  cvc::state::instance(a)(path).value(v);
}

} // namespace

// set_vehicle tunes a running vehicle: the facade's knobs (the steering limit) and the engine's
// (torque, the suspension's springs) both take effect.
TEST(PhysicsVehicle, SetVehicleTunesARunningVehicle) {
  REQUIRE_JOLT();
  // Ride height, then steer and speed after 2 s at full lock and full throttle.
  const auto run = [](bool tuned) {
    world w(jolt_config());
    flat_ground(w, 0.9);
    const vehicle_id v = settled_humvee(w);
    if (tuned) {
      vehicle_desc d = w.get_vehicle(v);
      d.max_steer = 0.2;
      d.engine_max_torque *= 0.5;
      for (axle_desc &a : d.axles)
        a.suspension_frequency *= 2.0;
      w.set_vehicle(v, d);
      EXPECT_EQ(w.get_vehicle(v).max_steer, 0.2);
    }
    w.drive(v, {});
    w.step(200);
    const double ride = w.vehicle_state(v).body.position.z;
    vehicle_controls c;
    c.steer = 0.6;
    c.throttle = 1.0;
    w.set_controls(v, c);
    w.step(200);
    const vehicle_state s = w.vehicle_state(v);
    return std::make_tuple(ride, s.steer, s.speed);
  };
  const auto [ride0, steer0, speed0] = run(false);
  const auto [ride1, steer1, speed1] = run(true);
  std::printf("  ride %.4f -> %.4f m (springs x2), steer %.3f -> %.3f rad, speed %.2f -> %.2f m/s "
              "(torque x0.5)\n",
              ride0, ride1, steer0, steer1, speed0, speed1);
  // The sag is g / (2 pi f)^2: 0.147 m at 1.3 Hz, 0.037 m at 2.6 Hz.
  EXPECT_NEAR(ride1 - ride0, 0.147 - 0.037, 0.02);
  EXPECT_NEAR(steer0, vehicle_desc::humvee().max_steer, 1e-9);
  EXPECT_NEAR(steer1, 0.2, 1e-9);
  EXPECT_LT(speed1, 0.75 * speed0);
}

// What is fixed when a vehicle is added cannot change, and a refused tuning changes nothing.
TEST(PhysicsVehicle, SetVehicleRefusesWhatIsFixed) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  const vehicle_desc before = w.get_vehicle(v);
  const auto refused = [&](vehicle_desc d, const char *named) {
    d.max_steer = 0.3; // a change that would be allowed, refused with the rest
    try {
      w.set_vehicle(v, d);
      ADD_FAILURE() << "accepted a change to " << named;
    } catch (const cvc::physics_error &e) {
      EXPECT_NE(std::string(e.what()).find(named), std::string::npos) << e.what();
    }
    EXPECT_EQ(w.get_vehicle(v).max_steer, before.max_steer);
  };
  vehicle_desc d = before;
  d.mass += 100.0;
  refused(d, "mass");
  d = before;
  d.axles[1].wheel_radius *= 1.1;
  refused(d, "axles[1].wheel_radius");
  d = before;
  d.gear_ratios.push_back(0.6);
  refused(d, "gear_ratios");
  d = before;
  d.axles[0].anti_roll_stiffness = 0.0; // the axle has a bar: it keeps one
  refused(d, "anti_roll_stiffness");
  d = before;
  d.max_steer = 2.0; // out of range, as add_vehicle would say
  EXPECT_THROW(w.set_vehicle(v, d), cvc::physics_error);
  EXPECT_THROW(w.set_vehicle(vehicle_id{}, before), cvc::physics_error);
  d = before;
  d.axles[0].anti_roll_stiffness = 30000.0; // a stiffer bar is a tuning
  EXPECT_NO_THROW(w.set_vehicle(v, d));
}

// snapshot() is the whole world at one tick: every body (the chassis too), every vehicle with
// its wheels, the materials and the contacts, equal to what the accessors read at that tick.
TEST(PhysicsVehicle, TheSnapshotHoldsEveryBodyAndVehicleAtOneTick) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  for (int i = 0; i < 3; ++i)
    w.add_body({box{{0.2, 0.2, 0.2}}, motion_type::dynamic, {5.0 + i, 4.0, 0.5 + i}});
  w.drive(v, {0.1, 1.0});
  w.step(50);
  const world_snapshot s = w.snapshot();
  EXPECT_EQ(s.tick, w.tick());
  EXPECT_DOUBLE_EQ(s.time, w.time());
  EXPECT_EQ(s.materials.size(), w.material_count());
  EXPECT_EQ(s.contacts.size(), w.contacts().size());
  ASSERT_NE(s.static_bodies, nullptr);
  ASSERT_EQ(s.bodies.size() + s.static_bodies->size(), w.body_count());
  ASSERT_EQ(s.static_bodies->size(), 1u); // the ground
  for (std::size_t i = 1; i < s.bodies.size(); ++i)
    EXPECT_LT(s.bodies[i - 1].id.value, s.bodies[i].id.value);
  std::vector<body_snapshot> every = s.bodies;
  every.insert(every.end(), s.static_bodies->begin(), s.static_bodies->end());
  for (const body_snapshot &b : every) {
    const body_state now = w.state(b.id);
    EXPECT_EQ(b.state.position.x, now.position.x);
    EXPECT_EQ(b.state.position.z, now.position.z);
    EXPECT_EQ(b.state.linear_velocity.x, now.linear_velocity.x);
    EXPECT_EQ(b.state.active, now.active);
    EXPECT_EQ(b.settings.material, w.get_body(b.id).material);
  }
  EXPECT_EQ((*s.static_bodies)[0].motion, motion_type::static_);
  ASSERT_EQ(s.vehicles.size(), 1u);
  const vehicle_state now = w.vehicle_state(v);
  EXPECT_EQ(s.vehicles[0].id, v);
  EXPECT_EQ(s.vehicles[0].desc.mass, vehicle_desc::humvee().mass);
  EXPECT_EQ(s.vehicles[0].state.speed, now.speed);
  ASSERT_EQ(s.vehicles[0].state.wheels.size(), now.wheels.size());
  EXPECT_EQ(s.vehicles[0].state.wheels[3].load, now.wheels[3].load);
  bool chassis = false;
  for (const body_snapshot &b : s.bodies)
    chassis = chassis || (b.id == now.chassis && b.motion == motion_type::dynamic);
  EXPECT_TRUE(chassis);
  // The static list is shared until a static body changes.
  w.step();
  const world_snapshot later = w.snapshot();
  EXPECT_EQ(later.static_bodies, s.static_bodies);
  EXPECT_EQ(later.static_version, s.static_version);
  w.add_body({box{{1.0, 1.0, 1.0}}, motion_type::static_, {40.0, 40.0, 1.0}});
  const world_snapshot after = w.snapshot();
  EXPECT_NE(after.static_version, s.static_version);
  EXPECT_EQ(after.static_bodies->size(), 2u);
  EXPECT_EQ(s.static_bodies->size(), 1u) << "an earlier snapshot keeps its own list";
  // A partial snapshot: just the vehicle and the ground.
  const world_snapshot part = w.snapshot({(*s.static_bodies)[0].id}, {v}, {});
  EXPECT_EQ(part.bodies.size(), 0u);
  ASSERT_EQ(part.static_bodies->size(), 1u);
  ASSERT_EQ(part.vehicles.size(), 1u);
  EXPECT_EQ(part.vehicles[0].state.speed, w.vehicle_state(v).speed);
  EXPECT_TRUE(part.contacts.empty());
}

// A vehicle published to the tree is driven, tuned and read there.
TEST(PhysicsVehicle, TheTreeDrivesTunesAndReadsAPublishedVehicle) {
  REQUIRE_JOLT();
  cvc::app a;
  world w(jolt_config());
  world_binding b(a, w);
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  const vehicle_id car = settled_humvee(w, 0.0, 10.0);
  EXPECT_THROW(b.publish_vehicle("truck.1", v), cvc::physics_error);
  EXPECT_THROW(b.publish_vehicle("ghost", vehicle_id{}), cvc::physics_error);
  b.publish_vehicle("truck", v);
  // A command the tree holds before the vehicle is published drives it.
  set_tree(a, "physics.vehicles.car.command.accel", "1.5");
  b.publish_vehicle("car", car);
  // Seeded: the commands at 0, the knobs at the vehicle's values.
  EXPECT_EQ(tree(a, "physics.vehicles.truck.command.accel"), "0");
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.tuning.max_steer"),
            vehicle_desc::humvee().max_steer);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.tuning.axles.1.suspension_frequency"),
            vehicle_desc::humvee().axles[1].suspension_frequency);
  set_tree(a, "physics.vehicles.truck.command.accel", "2");
  for (int i = 0; i < 200; ++i)
    b.step();
  const vehicle_state s = w.vehicle_state(v);
  EXPECT_GT(s.speed, 2.0);
  EXPECT_GT(speed_of(w, car), 1.5);
  // The keys are that tick's state, to the last digit.
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.speed"), s.speed);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.position.x"), s.body.position.x);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.rotation.w"), s.body.rotation.w);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.wheels.2.load"), s.wheels[2].load);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.wheels.0.contact_point.z"),
            s.wheels[0].contact_point.z);
  EXPECT_EQ(tree(a, "physics.vehicles.truck.gear"), std::to_string(s.gear));
  EXPECT_EQ(tree(a, "physics.vehicles.truck.chassis"), std::to_string(s.chassis.value));
  {
    // The same tick as JSON, for Python and remote readers.
    set_tree(a, "physics.snapshot_json_every", "1");
    b.step();
    std::istringstream in(tree(a, "physics.snapshot_json"));
    boost::property_tree::ptree doc;
    boost::property_tree::read_json(in, doc);
    EXPECT_EQ(doc.get<std::uint64_t>("tick"), w.tick());
    const vehicle_state now = w.vehicle_state(v);
    int vehicles = 0;
    for (const auto &e : doc.get_child("vehicles")) {
      if (e.second.get<std::uint32_t>("id") != v.value)
        continue;
      ++vehicles;
      EXPECT_EQ(std::stod(e.second.get<std::string>("state.speed")), now.speed);
      EXPECT_EQ(e.second.get_child("state.wheels").size(), now.wheels.size());
      EXPECT_EQ(std::stod(e.second.get<std::string>("desc.max_steer")),
                vehicle_desc::humvee().max_steer);
    }
    EXPECT_EQ(vehicles, 1);
    set_tree(a, "physics.snapshot_json_every", "0");
  }
  {
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i)
      b.push();
    std::printf(
        "  push() with two published vehicles: %.0f us\n",
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
            100);
  }
  // A program's own command stands until the tree's changes.
  w.drive(v, {0.0, -3.0});
  for (int i = 0; i < 100; ++i)
    b.step();
  EXPECT_LT(speed_of(w, v), s.speed - 1.0);
  // A tighter steering limit, then full lock, from the tree.
  set_tree(a, "physics.vehicles.truck.tuning.max_steer", "0.2");
  set_tree(a, "physics.vehicles.truck.command.steer", "0.5");
  for (int i = 0; i < 300; ++i)
    b.step();
  EXPECT_EQ(w.get_vehicle(v).max_steer, 0.2);
  EXPECT_NEAR(tree_num(a, "physics.vehicles.truck.steer"), 0.2, 1e-9);
  // An invalid knob is ignored, and so is one fixed at add_vehicle.
  set_tree(a, "physics.vehicles.truck.tuning.max_steer", "7");
  EXPECT_NO_THROW(b.step());
  EXPECT_EQ(w.get_vehicle(v).max_steer, 0.2);
  // Pedals.
  set_tree(a, "physics.vehicles.truck.controls.hand_brake", "1");
  b.step();
  b.step();
  EXPECT_EQ(w.vehicle_state(v).hand_brake, 1.0);
  // A vehicle the world no longer has stops being published.
  w.remove_vehicle(v);
  EXPECT_NO_THROW(b.step());
  EXPECT_NO_THROW(b.step());
}

// A field published to the tree is tuned there, and reports its ruts.
TEST(PhysicsVehicle, TheTreeTunesAFieldAndReadsItsRuts) {
  REQUIRE_JOLT();
  cvc::app a;
  world w(jolt_config());
  world_binding b(a, w);
  flat_ground(w, 0.9);
  const field_id snow = w.add_field(rutted_snow());
  const vehicle_id v = w.add_vehicle(vehicle_desc::humvee(), {-250, 0, 1.6});
  b.publish_field("snow", snow);
  b.publish_vehicle("truck", v);
  EXPECT_EQ(tree(a, "physics.fields.snow.traction"), "0"); // seeded
  set_tree(a, "physics.fields.snow.traction", "0.3");
  set_tree(a, "physics.vehicles.truck.command.accel", "1");
  for (int i = 0; i < 800; ++i)
    b.step();
  EXPECT_EQ(w.get_field(snow).traction, 0.3);
  EXPECT_NEAR(w.vehicle_state(v).wheels[0].friction, 0.3, 1e-6) << "the tires use the traction";
  const world_snapshot s = w.snapshot();
  ASSERT_EQ(s.fields.size(), 1u);
  EXPECT_GT(s.fields[0].rut_cells, 1000u);
  EXPECT_EQ(tree(a, "physics.fields.snow.kind"), "medium");
  EXPECT_EQ(std::stoull(tree(a, "physics.fields.snow.ruts.cells")), s.fields[0].rut_cells);
  EXPECT_EQ(tree_num(a, "physics.fields.snow.ruts.deepest"), s.fields[0].rut_deepest);
  EXPECT_GT(tree_num(a, "physics.vehicles.truck.wheels.0.sinkage"), 0.05);
  std::printf("  %zu rut cells, deepest %.4f m, after %.1f m\n", s.fields[0].rut_cells,
              s.fields[0].rut_deepest, w.vehicle_state(v).body.position.x + 250.0);
  set_tree(a, "physics.fields.snow.traction", "-1"); // invalid: ignored
  EXPECT_NO_THROW(b.step());
  EXPECT_EQ(w.get_field(snow).traction, 0.3);
}

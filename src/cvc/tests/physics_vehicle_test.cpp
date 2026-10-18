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
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <memory>
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

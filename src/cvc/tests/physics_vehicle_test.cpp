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
//   (9) the same run twice, and a save / restore / re-run, give the same hash;
//  (10) wading (vehicle_desc::wading): a Humvee in 0.5 m of water keeps its traction and slows at
//       its tires' and underbody's drag; past its fording depth its engine floods and it stays on
//       the bottom; a current and a swell push it; standing water soaks and aquaplanes its tires.
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
#include <cvc/physics/world_geometry.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/world/surface.h>
#include <cvc/world/weather.h>
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

// The actuator put at an angle (world::set_vehicle_steer, ARIADNE_WORLDS.md OW5r): the state
// reads it at once, the wheels take its Ackermann angles at the next step, and it slews on from
// there toward the command at its rate. Clamped to the lock; a non-finite angle is refused.
TEST(PhysicsVehicle, TheSteerCanBePutAtAnAngle) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  const vehicle_desc d = vehicle_desc::humvee();
  w.drive(v, {0.0, 0.0}); // the command: straight
  w.set_vehicle_steer(v, 0.3);
  EXPECT_NEAR(w.vehicle_state(v).steer, 0.3, 1e-12) << "the actuator there at once";
  w.step();
  const vehicle_state s = w.vehicle_state(v);
  const double rate = d.max_steer_rate * w.config().fixed_dt;
  EXPECT_NEAR(s.steer, 0.3 - rate, 1e-12) << "one step's slew back toward straight";
  ASSERT_EQ(s.wheels.size(), 4u);
  EXPECT_GT(s.wheels[0].steer, 0.25) << "the front wheels turned from it, not from straight";
  EXPECT_EQ(s.wheels[2].steer, 0.0);
  w.set_vehicle_steer(v, 5.0);
  EXPECT_NEAR(w.vehicle_state(v).steer, d.max_steer, 1e-12) << "clamped to the lock";
  w.set_vehicle_steer(v, -5.0);
  EXPECT_NEAR(w.vehicle_state(v).steer, -d.max_steer, 1e-12);
  EXPECT_THROW(w.set_vehicle_steer(v, std::nan("")), cvc::physics_error);
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

// The scene's wind (flow_velocity.h, O14): a crosswind pushes a car at highway speed aside,
// modestly. A Humvee at 25 m/s along +x in a steady 15 m/s wind across it (toward +y): the air at
// its centre of mass is the wind there (its profile at the centre's height), and the drag against
// the relative air has a side part of 1/2 rho C_d A |u| u_y.
TEST(PhysicsVehicle, ACrosswindPushesTheCarAside) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  field air;
  air.kind = field_kind::flow;
  air.wind = std::make_shared<wind_flow>(cvc::algo::wind_state{15.0, 1.5707963267948966, 0.0});
  w.add_field(air);
  const vehicle_id v = settled_humvee(w, -1200.0, 0.0);
  accelerate_to(w, v, 25.0);
  w.step();
  const vehicle_state s = w.vehicle_state(v);
  const vehicle_desc d = vehicle_desc::humvee();
  // The centre of mass in the world: q v q*, v' = v + 2 w (q x v) + 2 q x (q x v).
  const auto crossp = [](const vec3 &a, const vec3 &b) {
    return vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  };
  const quat &q = s.body.rotation;
  const vec3 qv{q.x, q.y, q.z}, cm = d.center_of_mass;
  const vec3 tq = crossp(qv, cm) * 2.0;
  const vec3 com = s.body.position + cm + tq * q.w + crossp(qv, tq);
  const vec3 want_air = air.wind->velocity(com, w.field_time());
  EXPECT_NEAR(s.air_velocity.y, want_air.y, 0.01 * want_air.y);
  EXPECT_GT(s.air_velocity.y, 7.0) << "the 15 m/s wind at a Humvee's centre of mass, 0.75 m up";
  const vec3 u = s.body.linear_velocity - s.air_velocity;
  const double un = std::sqrt(dot(u, u));
  const double side = -0.5 * d.air_density * d.drag_coefficient * d.frontal_area * un * u.y;
  std::printf("  a Humvee at %.1f m/s in a 15 m/s crosswind (%.1f m/s at its centre): %.0f N "
              "aside (%.2f m/s^2), the model %.0f N\n",
              s.speed, s.air_velocity.y, s.aero_force.y, s.aero_force.y / d.mass, side);
  EXPECT_NEAR(s.aero_force.y, side, 0.02 * std::fabs(side));
  EXPECT_GT(s.aero_force.y, 0.0) << "pushed downwind";
  EXPECT_LT(s.aero_force.y / d.mass, 0.5) << "a modest push: under 0.05 g";
}

// The aero reads a gusting wind at the world's field time. A Humvee stands with its wheels turned
// in a 10 m/s wind of gustiness 0.5, at field time 37 s: the air at its centre of mass is the wind
// there at 37 s. (The vehicle step's steering tangent once shadowed the field time, so the air was
// read at tan(steer) seconds; a steady wind, as above, never showed it.)
TEST(PhysicsVehicle, TheAeroReadsAGustingWindAtTheFieldTime) {
  REQUIRE_JOLT();
  world w(jolt_config());
  flat_ground(w, 0.9);
  const auto gusts = std::make_shared<wind_flow>(cvc::algo::wind_state{10.0, 0.4, 0.5});
  field air;
  air.kind = field_kind::flow;
  air.wind = gusts;
  w.add_field(air);
  const vehicle_id v = settled_humvee(w);
  w.drive(v, {0.4, 0.0}); // turn the wheels, standing on the brakes
  w.step(200);
  w.set_field_time(37.0);
  const vehicle_state s0 = w.vehicle_state(v);
  const double t0 = w.field_time();
  w.step(); // reads the air at s0's centre of mass, at t0
  const vehicle_state s1 = w.vehicle_state(v);
  const auto crossp = [](const vec3 &a, const vec3 &b) {
    return vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  };
  const quat &q = s0.body.rotation;
  const vec3 qv{q.x, q.y, q.z}, cm = vehicle_desc::humvee().center_of_mass;
  const vec3 tq = crossp(qv, cm) * 2.0;
  const vec3 com = s0.body.position + cm + tq * q.w + crossp(qv, tq);
  const vec3 want = gusts->velocity(com, t0);
  const vec3 wrong = gusts->velocity(com, std::tan(s1.steer));
  std::printf("  steer %.3f rad: the air %.3f, %.3f m/s; the wind at %.0f s %.3f, %.3f, at "
              "tan(steer) = %.3f s %.3f, %.3f\n",
              s1.steer, s1.air_velocity.x, s1.air_velocity.y, t0, want.x, want.y,
              std::tan(s1.steer), wrong.x, wrong.y);
  EXPECT_NEAR(s1.steer, 0.4, 1e-9);
  EXPECT_NEAR(s1.air_velocity.x, want.x, 1e-9);
  EXPECT_NEAR(s1.air_velocity.y, want.y, 1e-9);
  EXPECT_GT(std::hypot(want.x - wrong.x, want.y - wrong.y), 0.5)
      << "the gusts at the two times differ, so the test tells them apart";
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
  // The powertrain's knobs, seeded too: the converter, the driveline, the light shift points
  // and the friction curve.
  {
    const vehicle_desc h = vehicle_desc::humvee();
    for (const auto &k : std::vector<std::pair<std::string, double>>{
             {"clutch_strength", h.clutch_strength},
             {"converter_k", h.converter_k},
             {"converter_stall", h.converter_stall},
             {"converter_coupling", h.converter_coupling},
             {"driveline_efficiency", h.driveline_efficiency},
             {"shift_up_rpm_light", h.shift_up_rpm_light},
             {"shift_down_rpm_light", h.shift_down_rpm_light}})
      EXPECT_EQ(tree_num(a, "physics.vehicles.truck.tuning." + k.first), k.second) << k.first;
    EXPECT_EQ(tree(a, "physics.vehicles.truck.tuning.engine_friction_curve"),
              "700:78, 2040:87, 3400:128");
  }
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
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.accel_trim"), s.accel_trim);
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
  // The converter and the friction curve from the tree; a curve that does not parse is
  // ignored.
  set_tree(a, "physics.vehicles.truck.tuning.converter_stall", "1.5");
  set_tree(a, "physics.vehicles.truck.tuning.engine_friction_curve", "800:60; 3000:100");
  b.step();
  EXPECT_EQ(w.get_vehicle(v).converter_stall, 1.5);
  EXPECT_EQ(w.get_vehicle(v).engine_friction_curve,
            (std::vector<std::pair<double, double>>{{800.0, 60.0}, {3000.0, 100.0}}));
  set_tree(a, "physics.vehicles.truck.tuning.engine_friction_curve", "800:sixty");
  b.step();
  EXPECT_EQ(w.get_vehicle(v).engine_friction_curve.size(), 2u);
  set_tree(a, "physics.vehicles.truck.tuning.engine_friction_curve", "none");
  b.step();
  EXPECT_TRUE(w.get_vehicle(v).engine_friction_curve.empty());
  // Wading's knobs, seeded with the Humvee's: the deep-water fording kit from the tree; an
  // invalid value is ignored. Dry, its engine runs and no water pushes it.
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.tuning.wading.fording_depth"),
            vehicle_desc::humvee().wading.fording_depth);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.tuning.wading.tire_pressure"),
            vehicle_desc::humvee().wading.tire_pressure);
  set_tree(a, "physics.vehicles.truck.tuning.wading.fording_depth", "1.52");
  b.step();
  EXPECT_EQ(w.get_vehicle(v).wading.fording_depth, 1.52);
  set_tree(a, "physics.vehicles.truck.tuning.wading.fording_depth", "-1");
  b.step();
  EXPECT_EQ(w.get_vehicle(v).wading.fording_depth, 1.52);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.engine_flooded"), 0.0);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.water_force.z"), 0.0);
  EXPECT_EQ(tree_num(a, "physics.vehicles.truck.wheels.0.water_depth"), 0.0);
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

// Every key the binding owns says what it is: a comment and an access, a unit on a number, and a
// recommended range on a knob or a command.
TEST(PhysicsVehicle, EveryPublishedKeySaysWhatItIs) {
  REQUIRE_JOLT();
  cvc::app a;
  world w(jolt_config());
  world_binding b(a, w);
  flat_ground(w, 0.9);
  const vehicle_id v = settled_humvee(w);
  b.publish_vehicle("truck", v);
  set_tree(a, "physics.materials.mud.friction", "0.4");
  for (int i = 0; i < 5; ++i)
    b.step();
  cvc::state &root = cvc::state::instance(a)("physics");
  int leaves = 0;
  for (const std::string &full : root.children()) {
    cvc::state &n = cvc::state::instance(a)(full);
    if (!n.children().empty())
      continue; // a group, not a key
    ++leaves;
    EXPECT_FALSE(n.comment().empty()) << full << " has no comment";
    EXPECT_TRUE(n.has_property("access")) << full << " has no access";
  }
  EXPECT_GT(leaves, 150);
  const auto prop = [&](const std::string &k, const std::string &p) {
    return cvc::state::instance(a)("physics." + k).property(p);
  };
  EXPECT_EQ(prop("vehicles.truck.tuning.converter_k", "access"), "knob");
  EXPECT_EQ(prop("vehicles.truck.tuning.converter_k", "unit"), "rpm/sqrt(N m)");
  EXPECT_EQ(prop("vehicles.truck.tuning.converter_k", "min"), "0");
  EXPECT_EQ(prop("vehicles.truck.tuning.converter_k", "max"), "300");
  EXPECT_EQ(prop("vehicles.truck.tuning.axles.1.suspension_frequency", "unit"), "Hz");
  EXPECT_EQ(prop("vehicles.truck.speed", "access"), "telemetry");
  EXPECT_EQ(prop("vehicles.truck.speed", "unit"), "m/s");
  EXPECT_FALSE(cvc::state::instance(a)("physics.vehicles.truck.speed").has_property("min"));
  EXPECT_EQ(prop("vehicles.truck.command.accel", "access"), "command");
  EXPECT_EQ(prop("collision_steps", "max"), "32");
  EXPECT_EQ(prop("fixed_dt", "access"), "build");
  EXPECT_EQ(prop("materials.mud.friction", "max"), "2");
  EXPECT_EQ(prop("vehicles.truck.energy.Wh_per_km", "unit"), "Wh/km");
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

// O20 (F6), the weather into physics: a storm's rain lowers the road's friction through the one
// material table (apply_wetness, by cvc::world::weather::wetness where the vehicle is), so it
// stops in a longer distance on the wet road; as the road dries the distance comes back.
TEST(PhysicsVehicle, RainLowersTheRoadsTractionAndItComesBackAsItDries) {
  REQUIRE_JOLT();
  const cvc::world::surface_registry &surfaces = cvc::world::surface_registry::builtin();
  const cvc::world::surface_class &asphalt = surfaces[surfaces.by_name("asphalt")];
  // A storm crossing the vehicle's road at (-1200, 0), a full one 1000 m in radius, at 4 m/s.
  cvc::world::weather_params wp;
  wp.chance = 0.0;
  wp.steer_speed = 4.0;
  cvc::world::weather sky(wp);
  cvc::world::weather_event e;
  e.t = 0.0;
  e.x = -1200.0 - 3000.0;
  e.y = 0.0;
  e.life = 2400.0;
  e.radius = 1000.0;
  e.peak = 1.0;
  sky.add_event(e);
  const double soak = 300.0, dry = 600.0, rr = sky.rain_radius(1000.0, 1.0);
  const double out = (3000.0 + rr) / 4.0; // when its rain leaves the road
  auto stop_at = [&](double t, double &mu) {
    world w(jolt_config());
    // The road, a material named after its surface class, as add_world_geometry names it.
    const material_id m = w.add_material({"asphalt", asphalt.mu, 0.0f, asphalt.rolling_resistance});
    body_desc g;
    g.shape = box{{1500.0, 1500.0, 0.5}};
    g.motion = motion_type::static_;
    g.position = {0.0, 0.0, -0.5};
    g.material = m;
    w.add_body(g);
    const double wet = sky.wetness(t, -1200.0, 0.0, soak, dry);
    apply_wetness(w, wet, surfaces);
    mu = w.get_material(m).friction;
    const vehicle_id v = settled_humvee(w, -1200.0, 0.0);
    accelerate_to(w, v, 20.0);
    const vehicle_state s0 = w.vehicle_state(v);
    vehicle_controls stop;
    stop.brake = 1.0;
    w.set_controls(v, stop);
    for (int steps = 0; speed_of(w, v) > 0.05 && steps < 3000; ++steps)
      w.step();
    const vehicle_state s1 = w.vehicle_state(v);
    std::printf("  t = %.0f s: wetness %.2f, mu %.3f: stopped from %.2f m/s in %.2f m\n", t, wet,
                mu, s0.speed,
                std::hypot(s1.body.position.x - s0.body.position.x,
                           s1.body.position.y - s0.body.position.y));
    return std::make_pair(s0.speed, std::hypot(s1.body.position.x - s0.body.position.x,
                                               s1.body.position.y - s0.body.position.y));
  };
  double muDry = 0.0, muWet = 0.0, muDried = 0.0;
  const auto d0 = stop_at(0.0, muDry);               // before the storm
  const auto d1 = stop_at(out - 1.0, muWet);         // soaked
  const auto d2 = stop_at(out + 3.0 * dry, muDried); // three dry times after
  EXPECT_FLOAT_EQ(muDry, asphalt.mu);
  EXPECT_NEAR(muWet, asphalt.mu_wet, 1e-3);
  EXPECT_LT(muWet, muDry);
  EXPECT_GT(muDried, muDry - 0.05f);
  // The friction limit sets the stopping distance (TheStoppingDistanceIsTheFrictionLimit).
  auto ideal = [](double v0, double mu) { return v0 * v0 / (2.0 * mu * kG); };
  EXPECT_NEAR(d1.second, ideal(d1.first, muWet), 0.15 * ideal(d1.first, muWet));
  EXPECT_GT(d1.second, 1.3 * d0.second);              // the wet road stops it far later
  EXPECT_NEAR(d2.second, d0.second, 0.1 * d0.second); // and dried, it stops as it did
}

// --- wading (vehicle_desc::wading) -----------------------------------------------------------

namespace {

constexpr double kRho = 1000.0; // the tests' water, kg/m^3

// Water over the flat ground (top at z = 0) to `depth`, over x0 < x < x1 and |y| < 50, flowing at
// `current`.
field_id add_water(world &w, double x0, double x1, double depth, const vec3 &current = {}) {
  field water;
  water.kind = field_kind::fluid;
  const vec3 lo{x0, -50.0, -1.0}, hi{x1, 50.0, depth};
  water.volume = uniform(kRho, lo, hi);
  if (current.x != 0.0 || current.y != 0.0 || current.z != 0.0)
    water.velocity = {uniform(current.x, lo, hi), uniform(current.y, lo, hi),
                      uniform(current.z, lo, hi)};
  return w.add_field(water);
}

// The area of a circle of radius r below a chord h above its lowest point.
double segment(double r, double h) {
  h = std::max(0.0, std::min(2.0 * r, h));
  return r * r * std::acos((r - h) / r) - (r - h) * std::sqrt(h * (2.0 * r - h));
}

// The water's drag on a vehicle at `s` wading `depth` deep with the water flowing past it at u
// along (`along`) or across (!`along`) its axis (vehicle.h): every tire's displacement drag (its
// width times its immersed height, or its immersed side) and the underbody's.
double wading_drag(const vehicle_desc &d, const vehicle_state &s, double depth, double u,
                   bool along) {
  const wading_desc &g = d.wading;
  double cda = 0.0;
  for (const axle_desc &a : d.axles) {
    const double h = std::min(depth, 2.0 * a.wheel_radius);
    cda += 2.0 * g.wheel_drag * (along ? a.wheel_width * h : segment(a.wheel_radius, h));
  }
  const double bottom = s.body.position.z - d.half_extents.z - g.underbody_depth;
  const double hu = std::max(0.0, std::min(g.underbody_depth, depth - bottom));
  cda += g.underbody_drag * 2.0 * (along ? d.half_extents.y : d.half_extents.x) * hu;
  return 0.5 * kRho * cda * u * u;
}

double total_load(const vehicle_state &s) {
  double n = 0.0;
  for (const wheel_state &ws : s.wheels)
    n += ws.contact ? ws.load : 0.0;
  return n;
}

} // namespace

// A Humvee wading 0.5 m of still water (under its 0.76 m fording depth) keeps its footing: every
// wheel on the ground, its tires carrying all but what they float, and it pulls away as on land,
// if more slowly. Coasting, it slows at the water's drag on its tires and underbody plus their
// rolling resistance, and the ledger still balances with that work in its `fields`.
TEST(PhysicsVehicle, AHumveeWadingHalfAMetreSlowsButKeepsItsTraction) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  const double depth = 0.5;
  // Pulling away from rest in the water and on land: 3 s at full throttle, the speed each step.
  const auto pull_away = [&](bool wet, vehicle_state &rest, vehicle_state &after,
                             std::vector<double> &speeds) {
    world w(jolt_config());
    flat_ground(w, 0.9);
    if (wet)
      add_water(w, -200.0, 200.0, depth);
    const vehicle_id v = settled_humvee(w, -150.0, 0.0);
    rest = w.vehicle_state(v);
    vehicle_controls go;
    go.throttle = 1.0;
    w.set_controls(v, go);
    speeds.assign(1, rest.speed);
    for (int i = 0; i < static_cast<int>(3.0 / w.config().fixed_dt); ++i) {
      w.step();
      speeds.push_back(speed_of(w, v));
    }
    after = w.vehicle_state(v);
  };
  vehicle_state dry_rest, dry_go, wet_rest, wet_go;
  std::vector<double> dry_v, wet_v;
  pull_away(false, dry_rest, dry_go, dry_v);
  pull_away(true, wet_rest, wet_go, wet_v);
  // The tires' buoyancy: each displaces its share of the immersed cylinder.
  double floated = 0.0;
  for (const axle_desc &a : d.axles)
    floated += 2.0 * kRho * kG * segment(a.wheel_radius, depth) * a.wheel_width *
               d.wading.wheel_displacement;
  std::printf("  at rest: tires carry %.0f N dry, %.0f N wading (%.0f N floated by the model)\n",
              total_load(dry_rest), total_load(wet_rest), floated);
  for (const wheel_state &ws : wet_rest.wheels) {
    ASSERT_TRUE(ws.contact);
    EXPECT_NEAR(ws.water_depth, depth, 0.005);
  }
  EXPECT_NEAR(total_load(dry_rest) - total_load(wet_rest), floated, 0.05 * floated);
  EXPECT_FALSE(wet_go.engine_flooded);
  // Its traction: the tires deliver what they do on land at the same speed (the dry run's
  // acceleration there), less only the water's drag and plus the rolling resistance the floated
  // load no longer pays. Integrated over the 3 s, that model gives the wading speed.
  const double dt = 0.01;
  ASSERT_EQ(dt, jolt_config().fixed_dt);
  double m_eff = d.mass;
  for (const axle_desc &a : d.axles)
    m_eff += 2.0 * a.wheel_inertia / (a.wheel_radius * a.wheel_radius);
  const auto dry_accel = [&](double v) { // the dry run's acceleration at speed v
    for (std::size_t i = 1; i < dry_v.size(); ++i)
      if (dry_v[i] > v)
        return (dry_v[i] - dry_v[i - 1]) / dt;
    return 0.0;
  };
  double model_v = 0.0;
  for (std::size_t i = 1; i < wet_v.size(); ++i)
    model_v +=
        dt *
        (dry_accel(model_v) +
         (d.rolling_resistance * floated - wading_drag(d, wet_go, depth, model_v, true)) / m_eff);
  std::printf("  3 s at full throttle: %.2f m/s on land, %.2f m/s wading (the same traction less "
              "the water's drag: %.2f m/s)\n",
              dry_go.speed, wet_go.speed, model_v);
  // Slipping tires would leave it short of the model; it is a little ahead (its converter and
  // gears at a speed differ a little with the time it took to get there).
  EXPECT_LT(wet_go.speed, dry_go.speed);
  EXPECT_GT(wet_go.speed, 0.95 * model_v) << "it keeps its traction";
  EXPECT_LT(wet_go.speed, 1.1 * model_v);
  // Its tires slip a little more on their lighter loads (dry 0.08 at most here), but none spins:
  // at a slip of 0.3 the tire curve still holds nearly all of its peak.
  for (const wheel_state &ws : wet_go.wheels) {
    EXPECT_TRUE(ws.contact);
    EXPECT_LT(std::fabs(ws.longitudinal_slip), 0.3);
  }

  // Coasting from 4 m/s in neutral.
  world w(jolt_config());
  flat_ground(w, 0.9);
  add_water(w, -300.0, 300.0, depth);
  const vehicle_id v = settled_humvee(w, -280.0, 0.0);
  accelerate_to(w, v, 4.0);
  vehicle_controls coast;
  coast.neutral = true;
  w.set_controls(v, coast);
  w.step(30);
  const vehicle_state s0 = w.vehicle_state(v);
  w.step(60);
  const vehicle_state s1 = w.vehicle_state(v);
  const double u = 0.5 * (s0.speed + s1.speed);
  const double decel = (s0.speed - s1.speed) / (60 * dt);
  const double rolling = d.rolling_resistance * total_load(s0);
  const double aero = 0.5 * d.air_density * d.drag_coefficient * d.frontal_area * u * u;
  const double water = wading_drag(d, s0, depth, u, true);
  const double model = (rolling + aero + water) / m_eff;
  // The tire's drag as the readback has it, against AMC 25.1591's 1/2 rho C_d b d V^2.
  const axle_desc &a = d.axles[0];
  const double tire =
      0.5 * kRho * d.wading.wheel_drag * a.wheel_width * depth * s0.speed * s0.speed;
  std::printf("  coasting at %.2f m/s: %.0f N of water drag (%.0f N a tire, readback %.0f N), "
              "decel %.3f m/s^2 (model %.3f)\n",
              u, water, tire, s0.wheels[0].water_drag, decel, model);
  EXPECT_NEAR(s0.wheels[0].water_drag, tire, 0.05 * tire);
  EXPECT_NEAR(decel, model, 0.1 * model);
  EXPECT_LT(s0.water_force.x, 0.0) << "the water holds it back";
  // The ledger: the water's work is the fields' (and negative: the drag took energy out).
  const vehicle_energy &e = s1.energy;
  std::printf("  ledger: supplied %.0f J, fields %.0f J, residual %.0f J\n", e.supplied, e.fields,
              e.residual);
  EXPECT_LT(e.fields, 0.0);
  EXPECT_LT(std::fabs(e.residual), 0.02 * (e.supplied + std::fabs(e.fields)));
}

// At 1.2 m, past its fording depth, a Humvee's engine floods and it sits on the bottom: its body
// floods too, so the water lifts only the share it displaces, its tires keep a load and the
// commanded drive does nothing. With wading off, its chassis is a sealed 9.5 m^3 box, which
// floats it off its wheels.
TEST(PhysicsVehicle, InDeepWaterAHumveeStaysOnTheBottomAndItsEngineFloods) {
  REQUIRE_JOLT();
  const double depth = 1.2;
  const auto sit = [&](bool wading, vehicle_state &dry, vehicle_state &wet, double &fuel_before,
                       double &fuel_after) {
    vehicle_desc d = vehicle_desc::humvee();
    d.wading.enabled = wading;
    world a(jolt_config());
    flat_ground(a, 0.9);
    const vehicle_id va = a.add_vehicle(d, {0.0, 0.0, 1.2});
    a.drive(va, {});
    a.step(200);
    dry = a.vehicle_state(va);
    world w(jolt_config());
    flat_ground(w, 0.9);
    add_water(w, -50.0, 50.0, depth);
    const vehicle_id v = w.add_vehicle(d, {0.0, 0.0, 1.2});
    w.drive(v, {0.0, 1.5}); // try to drive off
    w.step(200);
    fuel_before = w.vehicle_state(v).energy.fuel_used;
    w.step(300);
    wet = w.vehicle_state(v);
    fuel_after = wet.energy.fuel_used;
  };
  vehicle_state dry, wet, sealed_dry, sealed;
  double f0 = 0.0, f1 = 0.0, g0 = 0.0, g1 = 0.0;
  sit(true, dry, wet, f0, f1);
  sit(false, sealed_dry, sealed, g0, g1);
  const vehicle_desc d = vehicle_desc::humvee();
  const double weight = d.mass * kG;
  std::printf("  wading: chassis %.3f m (%.3f m dry), tires carry %.0f of %.0f N, engine %s at "
              "%.0f rpm, %.2f m/s\n",
              wet.body.position.z, dry.body.position.z, total_load(wet), weight,
              wet.engine_flooded ? "flooded" : "running", wet.engine_rpm, wet.speed);
  std::printf("  sealed: chassis %.3f m, tires carry %.0f N\n", sealed.body.position.z,
              total_load(sealed));
  for (const wheel_state &ws : wet.wheels)
    EXPECT_TRUE(ws.contact);
  // It rides a little higher on its springs as they unload, but it does not float.
  EXPECT_NEAR(wet.body.position.z, dry.body.position.z, 0.15);
  EXPECT_GT(total_load(wet), 0.3 * weight);
  EXPECT_LT(total_load(wet), 0.7 * weight) << "the water lifts the share it displaces";
  EXPECT_TRUE(wet.engine_flooded);
  EXPECT_EQ(wet.throttle, 0.0);
  EXPECT_LT(wet.engine_rpm, 1.0) << "the engine has stopped";
  EXPECT_LT(std::fabs(wet.speed), 0.05) << "the drive command does nothing";
  EXPECT_EQ(f1, f0) << "a flooded engine burns no fuel";
  // Sealed, the box floats it: its tires carry next to nothing and it rides up (afloat, the box
  // draws 2676 / (1000 x 4.84 x 2.18) = 0.25 m, so its bottom stands 0.95 m up, not 0.57).
  EXPECT_GT(sealed.body.position.z, sealed_dry.body.position.z + 0.25);
  EXPECT_LT(total_load(sealed), 0.05 * weight);
  EXPECT_FALSE(sealed.engine_flooded) << "without wading it never floods";
}

// Driving through a ford: 0.5 m of water from x = 0, then 0.9 m from x = 40. The stock Humvee
// (0.76 m) drives through the shallow half, floods as its front axle reaches the deep half and
// coasts to a stop; one with the deep-water fording kit (1.52 m) drives on through.
TEST(PhysicsVehicle, PastItsFordingDepthTheEngineFloodsAndAKitFordsDeeper) {
  REQUIRE_JOLT();
  const auto ford = [](double fording, double &flooded_at, double &stopped_at, double &rpm,
                       double &fuel_after_flooding) {
    vehicle_desc d = vehicle_desc::humvee();
    d.wading.fording_depth = fording;
    world w(jolt_config());
    flat_ground(w, 0.9);
    add_water(w, 0.0, 40.0, 0.5);
    add_water(w, 40.0, 140.0, 0.9);
    const vehicle_id v = w.add_vehicle(d, {-40.0, 0.0, 1.2});
    w.drive(v, {});
    w.step(200);
    flooded_at = stopped_at = 1e9;
    double fuel_at_flood = 0.0;
    const int steps = static_cast<int>(80.0 / w.config().fixed_dt); // 140 m at 3 m/s, and spare
    for (int i = 0; i < steps; ++i) {
      const vehicle_state s = w.vehicle_state(v);
      w.drive(v, {0.0, s.speed < 3.0 ? 1.0 : 0.0}); // about 3 m/s
      w.step();
      const vehicle_state n = w.vehicle_state(v);
      if (n.engine_flooded && flooded_at > 1e8) {
        flooded_at = n.body.position.x + d.axles[0].x; // the front axle
        fuel_at_flood = n.energy.fuel_used;
      }
      if (flooded_at < 1e8 && std::fabs(n.speed) < 0.05 && stopped_at > 1e8)
        stopped_at = n.body.position.x;
      if (n.body.position.x > 100.0)
        break;
    }
    const vehicle_state s = w.vehicle_state(v);
    rpm = s.engine_rpm;
    fuel_after_flooding = s.energy.fuel_used - fuel_at_flood;
    return s.body.position.x;
  };
  double flooded = 0.0, stopped = 0.0, rpm = 0.0, fuel = 0.0;
  const double stock = ford(0.76, flooded, stopped, rpm, fuel);
  std::printf("  stock: flooded with its front axle at x = %.2f m, stopped at %.2f m, %.0f rpm\n",
              flooded, stopped, rpm);
  EXPECT_NEAR(flooded, 40.0, 1.0) << "as the front axle reached the deep water";
  EXPECT_LT(stopped, 60.0);
  EXPECT_NEAR(stock, stopped, 0.5) << "and it stays where it stopped";
  EXPECT_LT(rpm, 1.0);
  EXPECT_EQ(fuel, 0.0);
  double k_flooded = 0.0, k_stopped = 0.0, k_rpm = 0.0, k_fuel = 0.0;
  const double kit = ford(1.52, k_flooded, k_stopped, k_rpm, k_fuel);
  std::printf("  with the kit: reached x = %.2f m\n", kit);
  EXPECT_GT(kit, 100.0) << "it fords 0.9 m";
  EXPECT_GT(k_flooded, 1e8) << "and never floods";
}

// The water's velocity pushes a wading vehicle: a 1.5 m/s current along its axis drifts a Humvee
// standing in neutral downstream, against its tires' rolling resistance; one across it pushes it
// sideways by its tires' immersed sides and its underbody's (which its tires' grip holds).
TEST(PhysicsVehicle, ACurrentPushesAWadingHumvee) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  const double depth = 0.6, u = 1.5;
  vehicle_controls loose;
  loose.neutral = true;
  {
    world w(jolt_config());
    flat_ground(w, 0.9);
    add_water(w, -100.0, 100.0, depth, {u, 0.0, 0.0});
    const vehicle_id v = settled_humvee(w);
    const vehicle_state s0 = w.vehicle_state(v);
    w.set_controls(v, loose);
    w.step(static_cast<std::uint64_t>(5.0 / w.config().fixed_dt));
    const vehicle_state s1 = w.vehicle_state(v);
    std::printf("  along: pushed %.0f N at rest (model %.0f N), drifting %.2f m/s after 5 s\n",
                s0.water_force.x, wading_drag(d, s0, depth, u, true), s1.speed);
    EXPECT_NEAR(s0.water_force.x, wading_drag(d, s0, depth, u, true),
                0.05 * wading_drag(d, s0, depth, u, true));
    EXPECT_GT(s1.speed, 0.2) << "it drifts downstream";
    EXPECT_LT(s1.speed, u) << "never faster than the water";
  }
  {
    world w(jolt_config());
    flat_ground(w, 0.9);
    add_water(w, -100.0, 100.0, depth, {0.0, u, 0.0});
    const vehicle_id v = settled_humvee(w);
    w.set_controls(v, loose);
    w.step(60);
    const vehicle_state s = w.vehicle_state(v);
    const double model = wading_drag(d, s, depth, u, false);
    std::printf("  across: pushed %.0f N (model %.0f N), sideways at %.3f m/s\n", s.water_force.y,
                model, s.body.linear_velocity.y);
    EXPECT_NEAR(s.water_force.y, model, 0.05 * model);
    EXPECT_LT(std::fabs(s.body.linear_velocity.y), 0.05) << "the tires hold it";
  }
}

// The waves push it too: a swell's orbital motion (a free surface's velocity, fluid_surface.h)
// rocks a Humvee standing in neutral in 0.6 m of water back and forth along its axis.
TEST(PhysicsVehicle, AWadingHumveeRocksWithTheSwell) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  world w(jolt_config());
  flat_ground(w, 0.9);
  wave_train t;
  t.amplitude = 0.15;
  t.period_s = 5.0;
  t.dir_rad = 0.0; // crests across the vehicle, travelling along it
  const auto sea = std::make_shared<wave_train_surface>(0.6, 0.6, std::vector<wave_train>{t});
  field water;
  water.kind = field_kind::fluid;
  water.volume = uniform(kRho, {-100.0, -50.0, -1.0}, {100.0, 50.0, 1.0});
  water.surface = sea;
  w.add_field(water);
  const vehicle_id v = settled_humvee(w);
  vehicle_controls loose;
  loose.neutral = true;
  w.set_controls(v, loose);
  const int period = static_cast<int>(t.period_s / w.config().fixed_dt);
  double f_lo = 0.0, f_hi = 0.0, x_lo = 1e9, x_hi = -1e9, u_max = 0.0;
  for (int i = 0; i < 2 * period; ++i) {
    w.step();
    const vehicle_state s = w.vehicle_state(v);
    f_lo = std::min(f_lo, s.water_force.x);
    f_hi = std::max(f_hi, s.water_force.x);
    // The water's orbital speed where it meets the tires, half way up the still depth.
    u_max =
        std::max(u_max, std::fabs(sea->velocity({s.body.position.x, 0.0, 0.3}, w.field_time()).x));
    if (i >= period) {
      x_lo = std::min(x_lo, s.body.position.x);
      x_hi = std::max(x_hi, s.body.position.x);
    }
  }
  // The push the orbital motion would give a vehicle held still (it is not quite: it rocks).
  const double held = wading_drag(d, w.vehicle_state(v), 0.6, u_max, true);
  std::printf("  orbital speed up to %.2f m/s (%.0f N on a held vehicle); the push swings %.0f to "
              "%.0f N; it rocks %.3f m\n",
              u_max, held, f_lo, f_hi, x_hi - x_lo);
  // Both ways, the crests' push the harder: under a crest the water stands deeper on the tires
  // than under a trough (0.75 m against 0.45 m).
  EXPECT_GT(u_max, 0.3);
  EXPECT_LT(f_lo, -0.25 * held);
  EXPECT_GT(f_hi, 0.5 * held);
  EXPECT_GT(f_hi, -f_lo);
  EXPECT_GT(x_hi - x_lo, 0.005);
}

// A tire in water is on soaked ground (material::wet_friction, the rain's mu_wet), past 3 mm of
// it in full; and in standing water it aquaplanes as it nears 0.85 x 9 sqrt(p) knots (AMC
// 25.1591), here an underinflated 20 psi tire's 17.6 m/s. The rain's wetness and the water agree:
// a road the rain has already soaked is no slicker under water.
TEST(PhysicsVehicle, StandingWaterSoaksTheTiresGripAndItAquaplanes) {
  REQUIRE_JOLT();
  const cvc::world::surface_registry &surfaces = cvc::world::surface_registry::builtin();
  const cvc::world::surface_class &asphalt = surfaces[surfaces.by_name("asphalt")];
  vehicle_desc d = vehicle_desc::humvee();
  d.wading.tire_pressure = 20.0 * 6894.757;
  const double vp = 0.85 * 9.0 * 1852.0 / 3600.0 * std::sqrt(20.0);
  // The tires' mu at `speed`, over `depth` of water, the road soaked by rain to `rain`.
  const auto mu_at = [&](double depth, double speed, double rain) {
    world w(jolt_config());
    const material_id m = w.add_material(
        {"asphalt", asphalt.mu, 0.0f, asphalt.rolling_resistance, -1, asphalt.mu_wet});
    body_desc g;
    g.shape = box{{1500.0, 1500.0, 0.5}};
    g.motion = motion_type::static_;
    g.position = {0.0, 0.0, -0.5};
    g.material = m;
    w.add_body(g);
    apply_wetness(w, rain, surfaces);
    if (depth > 0.0)
      add_water(w, -1300.0, 1300.0, depth);
    const vehicle_id v = w.add_vehicle(d, {-1200.0, 0.0, 1.2});
    w.drive(v, {});
    w.step(200);
    accelerate_to(w, v, speed);
    w.step(); // a step at the speed
    const vehicle_state s = w.vehicle_state(v);
    return std::make_pair(s.wheels[2].friction, s.speed); // a rear wheel: it drives, not steers
  };
  const double mu = asphalt.mu, wet = asphalt.mu_wet;
  const auto slow = mu_at(0.02, 2.0, 0.0);
  const auto film = mu_at(0.001, 2.0, 0.0);
  const auto soaked = mu_at(0.02, 2.0, 1.0);
  const auto fast = mu_at(0.02, 14.0, 0.0);
  const auto dry_fast = mu_at(0.0, 14.0, 0.0);
  // The model (vehicle.h): soaked by the share s of 3 mm the water stands, then that share of the
  // way to its aquaplaning mu, (1 - (v / v_p)^2) of it and at least 0.05.
  const auto model = [&](double depth, double v) {
    const double s = std::min(1.0, depth / 0.003);
    const double m = mu + (wet - mu) * s;
    const double a = std::max(0.05, m * (1.0 - (v / vp) * (v / vp)));
    return m + (std::min(a, m) - m) * s;
  };
  std::printf("  mu dry %.3f, soaked %.3f; in 2 cm at %.1f m/s %.3f, 1 mm %.3f, rained on %.3f; "
              "at %.1f m/s %.3f (dry %.3f)\n",
              mu, wet, slow.second, slow.first, film.first, soaked.first, fast.second, fast.first,
              dry_fast.first);
  // Applied from the step before's speed, which the readback's differs from by one step.
  EXPECT_NEAR(slow.first, model(0.02, slow.second), 0.005);
  EXPECT_NEAR(film.first, model(0.001, film.second), 0.005) << "a third soaked";
  EXPECT_NEAR(soaked.first, slow.first, 0.005) << "the rain's soaking and the water's agree";
  EXPECT_NEAR(fast.first, model(0.02, fast.second), 0.01);
  EXPECT_LT(fast.first, 0.5 * wet) << "it aquaplanes";
  EXPECT_NEAR(dry_fast.first, mu, 1e-6) << "out of the water nothing changes";
}

// The flooded engine is state: a save after it floods restores it, and the run replays to the
// same hash. Wading's values are checked, and whether a vehicle wades is fixed when it is added.
TEST(PhysicsVehicle, WadingIsStateAndItsValuesAreChecked) {
  REQUIRE_JOLT();
  const auto build = [](world &w) {
    flat_ground(w, 0.85);
    add_water(w, 10.0, 60.0, 0.4);
    add_water(w, 60.0, 200.0, 1.0);
    return w.add_vehicle(vehicle_desc::humvee(), {0, 0, 1.2});
  };
  const auto drive = [](world &w, vehicle_id v, int from, int to) {
    for (int i = from; i < to; ++i) {
      w.drive(v, {0.05 * std::sin(0.01 * i), 1.5});
      w.step();
    }
  };
  world a(jolt_config()), b(jolt_config());
  const vehicle_id va = build(a), vb = build(b);
  int flooded = -1;
  for (int i = 0; i < 2400 && flooded < 0; i += 60) {
    drive(a, va, i, i + 60);
    if (a.vehicle_state(va).engine_flooded)
      flooded = i + 60;
  }
  ASSERT_GT(flooded, 0) << "it reached the deep water";
  const std::vector<std::uint8_t> mid = a.save();
  drive(a, va, flooded, flooded + 300);
  drive(b, vb, 0, flooded + 300);
  EXPECT_EQ(a.state_hash(), b.state_hash());
  world c(jolt_config());
  const vehicle_id vc = build(c);
  c.restore(mid);
  EXPECT_TRUE(c.vehicle_state(vc).engine_flooded);
  drive(c, vc, flooded, flooded + 300);
  EXPECT_EQ(c.state_hash(), a.state_hash());

  vehicle_desc bad = vehicle_desc::humvee();
  bad.wading.displacement = 1.5;
  EXPECT_THROW(a.add_vehicle(bad, {0, 20, 1.2}), cvc::physics_error);
  bad = vehicle_desc::humvee();
  bad.wading.fording_depth = -0.1;
  EXPECT_THROW(a.add_vehicle(bad, {0, 20, 1.2}), cvc::physics_error);
  vehicle_desc d = a.get_vehicle(va);
  d.wading.enabled = false;
  EXPECT_THROW(a.set_vehicle(va, d), cvc::physics_error);
  d = a.get_vehicle(va);
  d.wading.displacement = 0.5;
  EXPECT_THROW(a.set_vehicle(va, d), cvc::physics_error);
  d = a.get_vehicle(va);
  d.wading.fording_depth = 1.52; // fitting the kit is a tuning (it does not unflood the engine)
  d.wading.wheel_drag = 0.9;
  EXPECT_NO_THROW(a.set_vehicle(va, d));
  EXPECT_TRUE(a.vehicle_state(va).engine_flooded);
}

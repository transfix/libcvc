/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for boats (cvc/physics/boat.h; docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D3, O11b): a
// hull body driven by a propeller and steered by a rudder or an outboard, whose forces the world
// applies every step. With Jolt, a 4 x 1.5 x 0.6 m box boat of 900 kg on still water:
//   (1) full throttle reaches a terminal speed where the thrust balances the water's resistance,
//       and full astern backs it;
//   (2) the rudder turns it the way it should, faster and on a tighter circle the more it is put
//       over;
//   (3) a propeller out of the water pushes nothing, and an outboard turns the boat at low speed
//       where a rudder barely does;
//   (4) on a swell it pitches with the surface's slope;
//   (5) runs hash alike, the boats are in the state, and the scene's hash is pinned.
// The refusals, the actuators and the thrust law run on the null backend, everywhere.

#include <algorithm>
#include <cmath>
#include <cstdint>
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

constexpr double kPi = 3.14159265358979323846;

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

world_config config(backend b = backend::jolt) {
  world_config c;
  c.backend = b;
  return c;
}

cvc::app &ctx() {
  static cvc::app app;
  return app;
}

std::shared_ptr<cvc::volume> uniform(double value, vec3 lo, vec3 hi) {
  auto v = std::make_shared<cvc::volume>(ctx(), cvc::dimension(2, 2, 2), cvc::Float,
                                         cvc::bounding_box(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z));
  v->fill(value);
  return v;
}

// Still water up to z = 0, 1200 m square and 5 m deep.
field water() {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-600, -600, -5}, {600, 600, 0});
  f.damping = 1.0e4;
  return f;
}

// The same water with a free surface, its box reaching z = 2.
field sea(std::shared_ptr<const fluid_surface> surface) {
  field f = water();
  f.volume = uniform(1000.0, {-600, -600, -5}, {600, 600, 2});
  f.surface = std::move(surface);
  return f;
}

// A box hull with each face split into n x n squares, two triangles each (12 n^2 triangles), so
// the water's push on the sides is spread along them as the boat yaws.
std::shared_ptr<hull_mesh> split_box_hull(const vec3 &h, int n) {
  auto out = std::make_shared<hull_mesh>();
  const double e[3] = {h.x, h.y, h.z};
  const int faces[6][4] = {{0, +1, 1, 2}, {0, -1, 2, 1}, {1, +1, 2, 0},
                           {1, -1, 0, 2}, {2, +1, 0, 1}, {2, -1, 1, 0}};
  for (const auto &f : faces) {
    const int a = f[0], u = f[2], v = f[3];
    const std::uint32_t base = static_cast<std::uint32_t>(out->vertices.size());
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i) {
        double p[3];
        p[a] = f[1] * e[a];
        p[u] = e[u] * (-1.0 + 2.0 * i / n);
        p[v] = e[v] * (-1.0 + 2.0 * j / n);
        out->vertices.push_back({p[0], p[1], p[2]});
      }
    const auto at = [&](int i, int j) {
      return base + static_cast<std::uint32_t>(j * (n + 1) + i);
    };
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        out->triangles.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
        out->triangles.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
      }
  }
  return out;
}

// The test boat: a 4 x 1.5 x 0.6 m box of 900 kg, floating 0.15 m deep, its propeller and
// rudder 0.2 m under the waterline at the stern.
boat_desc skiff() {
  boat_desc d;
  const vec3 half{2.0, 0.75, 0.3};
  d.hull.shape = box{half};
  d.hull.hull = split_box_hull(half, 4);
  d.hull.mass = 900.0;
  d.hull.linear_damping = 0.0; // the water is its resistance
  d.hull.angular_damping = 0.0;
  d.hull.allow_sleeping = false;
  d.propeller.position = {-2.1, 0.0, -0.35};
  d.propeller.max_thrust = 2000.0;
  d.propeller.max_reverse_thrust = 1000.0;
  d.propeller.max_inflow_speed = 8.0;
  d.rudder.position = {-2.3, 0.0, -0.35};
  d.rudder.area = 0.12;
  d.actuator_time_constant = 0.5;
  return d;
}

// The hull's draft-level start: its centre 0.15 m above the still water.
constexpr double kFloat = 0.15;

quat about_z(double a) { return {std::cos(a / 2), 0.0, 0.0, std::sin(a / 2)}; }

double mean(const std::vector<double> &v) {
  double s = 0.0;
  for (const double x : v)
    s += x / double(v.size());
  return s;
}

double correlation(const std::vector<double> &a, const std::vector<double> &b) {
  const double ma = mean(a), mb = mean(b);
  double saa = 0.0, sbb = 0.0, sab = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    saa += (a[i] - ma) * (a[i] - ma);
    sbb += (b[i] - mb) * (b[i] - mb);
    sab += (a[i] - ma) * (b[i] - mb);
  }
  return sab / std::sqrt(saa * sbb);
}

} // namespace

// --- refusals, actuators and the thrust law (null backend) --------------------------------

TEST(PhysicsBoat, BadDescriptionsAndControlsAreRefused) {
  world w(config(backend::null));
  const auto refused = [&](boat_desc d, const char *why) {
    EXPECT_THROW(w.add_boat(d, {0, 0, 0}), cvc::physics_error) << why;
  };
  boat_desc d = skiff();
  d.hull.motion = motion_type::kinematic;
  d.hull.hull.reset();
  refused(d, "a hull that is not dynamic");
  d = skiff();
  d.propeller.direction = {0, 0, 0};
  refused(d, "no thrust direction");
  d = skiff();
  d.propeller.max_thrust = -1.0;
  refused(d, "negative thrust");
  d = skiff();
  d.propeller.position.y = std::nan("");
  refused(d, "a propeller not finite");
  d = skiff();
  d.propeller.outboard_max_angle = 2.0;
  refused(d, "an outboard past 90 degrees");
  d = skiff();
  d.rudder.area = -0.1;
  refused(d, "a negative rudder area");
  d = skiff();
  d.rudder.stall_angle = 0.0;
  refused(d, "no stall angle");
  d = skiff();
  d.rudder.max_angle = 1.7;
  refused(d, "a rudder past 90 degrees");
  d = skiff();
  d.actuator_time_constant = -1.0;
  refused(d, "a negative time constant");
  d = skiff();
  d.hull.mass = -1.0;
  refused(d, "add_body's own refusal");
  EXPECT_THROW(w.add_boat(skiff(), {0, 0, 0}, quat{2, 0, 0, 0}), cvc::physics_error);
  EXPECT_EQ(w.body_count(), 0u) << "a refused boat leaves no hull behind";
  EXPECT_EQ(w.boat_count(), 0u);

  const boat_id b = w.add_boat(skiff(), {0, 0, kFloat});
  EXPECT_EQ(w.boat_count(), 1u);
  EXPECT_EQ(w.body_count(), 1u);
  EXPECT_THROW(w.set_boat_controls(b, {1.5, 0.0}), cvc::physics_error);
  EXPECT_THROW(w.set_boat_controls(b, {0.0, -1.1}), cvc::physics_error);
  EXPECT_THROW(w.set_boat_controls(b, {std::nan(""), 0.0}), cvc::physics_error);
  const body_id hull = w.boat_state(b).hull;
  EXPECT_THROW(w.remove_body(hull), cvc::physics_error) << "remove the boat, not its hull";
  EXPECT_EQ(w.get_boat(b).rudder.area, skiff().rudder.area);
  w.remove_boat(b);
  EXPECT_EQ(w.boat_count(), 0u);
  EXPECT_EQ(w.body_count(), 0u) << "its hull went with it";
  EXPECT_FALSE(w.contains(hull));
  EXPECT_THROW(w.boat_state(b), cvc::physics_error);
  EXPECT_THROW(w.set_boat_controls(b, {}), cvc::physics_error);
}

// The actuators lag as a first-order system, and at rest in the water the propeller gives its
// rated thrust times the throttle: these need no engine (on the null backend nothing moves).
TEST(PhysicsBoat, TheActuatorsLagAndTheThrustIsRatedAtRest) {
  world w(config(backend::null));
  w.add_field(water());
  const boat_id b = w.add_boat(skiff(), {0, 0, kFloat});
  w.set_boat_controls(b, {1.0, -0.5});
  w.step(50); // 0.5 s, one time constant
  boat_state s = w.boat_state(b);
  EXPECT_NEAR(s.throttle, 1.0 - std::exp(-1.0), 1e-12);
  EXPECT_NEAR(s.rudder, -0.5 * (1.0 - std::exp(-1.0)), 1e-12);
  EXPECT_TRUE(s.propeller_wet);
  EXPECT_NEAR(s.thrust, s.throttle * 2000.0, 1e-9) << "no inflow at rest";
  EXPECT_NEAR(s.thrust_force.x, s.thrust, 1e-9);
  EXPECT_NEAR(s.rudder_angle, s.rudder * 0.6, 1e-12);
  EXPECT_EQ(s.rudder_lift, 0.0) << "no flow past the rudder";

  boat_desc instant = skiff();
  instant.actuator_time_constant = 0.0;
  const boat_id c = w.add_boat(instant, {10, 0, kFloat});
  w.set_boat_controls(c, {-0.5, 1.0});
  w.step();
  s = w.boat_state(c);
  EXPECT_EQ(s.throttle, -0.5);
  EXPECT_EQ(s.rudder, 1.0);
  EXPECT_NEAR(s.thrust, -0.5 * 1000.0, 1e-9) << "astern at its own rating";
}

// --- propulsion (Jolt) ---------------------------------------------------------------------

// Gate (1): full throttle from rest reaches a terminal speed where the thrust, T_max (1 - u_a /
// u_max), balances what the water holds back: the hull's resistance and the rudder's drag.
TEST(PhysicsBoat, FullThrottleReachesTheSpeedWhereThrustBalancesResistance) {
  REQUIRE_JOLT();
  world w(config());
  w.add_field(water());
  const boat_id b = w.add_boat(skiff(), {-300, 0, kFloat});
  w.step(200);
  w.set_boat_controls(b, {1.0, 0.0});
  w.step(8000); // 80 s
  std::vector<double> speed, thrust, hull, rudder, accel;
  double last = w.boat_state(b).speed;
  for (int i = 0; i < 1000; ++i) {
    w.step();
    const boat_state s = w.boat_state(b);
    speed.push_back(s.speed);
    thrust.push_back(s.thrust_force.x);
    hull.push_back(w.field_load_on(s.hull).force.x);
    rudder.push_back(s.rudder_force.x);
    accel.push_back((s.speed - last) / 0.01);
    last = s.speed;
  }
  const boat_state s = w.boat_state(b);
  const double u = mean(speed), t = mean(thrust), r = -mean(hull), rd = -mean(rudder);
  std::printf("[boat] terminal speed %.4f m/s, inflow %.4f m/s: thrust %.2f N, hull resistance "
              "%.2f N, rudder drag %.3f N, imbalance %.3f N, mean accel %.2e m/s^2\n",
              u, s.inflow, t, r, rd, t - r - rd, mean(accel));
  EXPECT_GT(u, 2.0) << "under way";
  EXPECT_NEAR(t, r + rd, 1e-3 * t) << "the thrust balances the resistance";
  EXPECT_NEAR(s.thrust, 2000.0 * (1.0 - s.inflow / 8.0), 1e-9) << "the thrust law";
  EXPECT_NEAR(s.inflow, u, 0.05 * u) << "the inflow is about the speed through the water";
  EXPECT_LT(std::fabs(mean(accel)), 1e-3) << "steady";
  EXPECT_NEAR(s.heading, 0.0, 1e-3) << "straight ahead";
  EXPECT_NEAR(s.body.position.y, 0.0, 0.05);
}

TEST(PhysicsBoat, AsternBacksIt) {
  REQUIRE_JOLT();
  world w(config());
  w.add_field(water());
  const boat_id b = w.add_boat(skiff(), {0, 0, kFloat});
  w.step(200);
  w.set_boat_controls(b, {-1.0, 0.0});
  w.step(3000);
  const boat_state s = w.boat_state(b);
  std::printf("[boat] full astern after 30 s: speed %.4f m/s, thrust %.2f N, x %.2f m\n", s.speed,
              s.thrust, s.body.position.x);
  EXPECT_LT(s.speed, -1.0) << "going astern";
  EXPECT_LT(s.thrust, 0.0);
  EXPECT_GT(s.thrust, -1000.0) << "falling off with the inflow from astern";
  EXPECT_LT(s.body.position.x, -20.0);
}

// Gate (3): a propeller out of the water pushes nothing.
TEST(PhysicsBoat, APropellerOutOfTheWaterGivesNoThrust) {
  REQUIRE_JOLT();
  world w(config());
  w.add_field(water());
  boat_desc d = skiff();
  d.propeller.position.z = 0.5; // over the deck
  const boat_id b = w.add_boat(d, {0, 0, kFloat});
  w.step(200);
  w.set_boat_controls(b, {1.0, 0.0});
  w.step(1000);
  const boat_state s = w.boat_state(b);
  std::printf("[boat] propeller in the air: thrust %.3f N, speed %.2e m/s\n", s.thrust, s.speed);
  EXPECT_FALSE(s.propeller_wet);
  EXPECT_EQ(s.thrust, 0.0);
  EXPECT_LT(std::fabs(s.speed), 1e-3);
  EXPECT_TRUE(s.rudder_wet) << "the rudder is still in";
}

// --- steering (Jolt) -----------------------------------------------------------------------

namespace {

struct turn {
  double yaw_rate = 0.0, yaw_spread = 0.0, speed = 0.0, drift = 0.0, radius = 0.0, diameter = 0.0,
         heading_after_5s = 0.0;
};

// Full ahead from rest with the rudder put over to `rudder` after 20 s; then, from 40 s after,
// one whole circle of the steady turn: its yaw rate (and how far it strays from its mean), speed,
// drift angle, U / r, and the circle's diameter, the mean of its extents across x and y.
turn steady_turn(double rudder) {
  world w(config());
  w.add_field(water());
  const boat_id b = w.add_boat(skiff(), {0, 0, kFloat});
  w.set_boat_controls(b, {1.0, 0.0});
  w.step(2000);
  const double h0 = w.boat_state(b).heading;
  w.set_boat_controls(b, {1.0, rudder});
  w.step(500);
  turn out;
  out.heading_after_5s = w.boat_state(b).heading - h0;
  w.step(3500);
  const int n =
      static_cast<int>(std::lround(2.0 * kPi / std::fabs(w.boat_state(b).yaw_rate) / 0.01));
  std::vector<double> r, u, drift;
  double lo[2] = {1e9, 1e9}, hi[2] = {-1e9, -1e9};
  for (int i = 0; i < n; ++i) {
    w.step();
    const boat_state s = w.boat_state(b);
    r.push_back(s.yaw_rate);
    u.push_back(std::hypot(s.body.linear_velocity.x, s.body.linear_velocity.y));
    drift.push_back(std::atan2(-s.sway, s.speed));
    const double p[2] = {s.body.position.x, s.body.position.y};
    for (int k = 0; k < 2; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
  out.yaw_rate = mean(r);
  for (const double x : r)
    out.yaw_spread = std::max(out.yaw_spread, std::fabs(x - out.yaw_rate));
  out.speed = mean(u);
  out.drift = mean(drift);
  out.radius = out.speed / std::fabs(out.yaw_rate);
  out.diameter = 0.5 * ((hi[0] - lo[0]) + (hi[1] - lo[1]));
  return out;
}

} // namespace

// Gate (2): a positive rudder turns the boat left going ahead, faster and on a tighter circle the
// further it is put over, and a negative one turns it right.
TEST(PhysicsBoat, TheRudderTurnsItFasterAndTighterTheMoreItIsPutOver) {
  REQUIRE_JOLT();
  const double rudders[3] = {0.2, 0.45, 0.9};
  turn t[3];
  for (int i = 0; i < 3; ++i) {
    t[i] = steady_turn(rudders[i]);
    std::printf("[boat] rudder %.2f (%.1f deg): yaw rate %.5f rad/s (+-%.1e) at %.4f m/s, drift "
                "%.4f rad, radius %.2f m (U/r), diameter %.2f m, heading %.3f rad 5 s after\n",
                rudders[i], rudders[i] * 0.6 * 180.0 / kPi, t[i].yaw_rate, t[i].yaw_spread,
                t[i].speed, t[i].drift, t[i].radius, t[i].diameter, t[i].heading_after_5s);
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_GT(t[i].yaw_rate, 0.0) << "turning left";
    EXPECT_LT(t[i].yaw_spread, 0.01 * t[i].yaw_rate) << "steadily";
    EXPECT_NEAR(t[i].diameter, 2.0 * t[i].radius, 0.02 * t[i].radius) << "on a circle";
    EXPECT_GT(t[i].drift, 0.0) << "the stern swung out, the bow in";
  }
  EXPECT_GT(t[1].yaw_rate, t[0].yaw_rate);
  EXPECT_GT(t[2].yaw_rate, t[1].yaw_rate);
  EXPECT_LT(t[1].radius, t[0].radius);
  EXPECT_LT(t[2].radius, t[1].radius);
  const turn right = steady_turn(-0.45);
  EXPECT_NEAR(right.yaw_rate, -t[1].yaw_rate, 1e-3 * t[1].yaw_rate) << "a mirror image";
}

// Gate (3): at low speed an outboard, swinging its thrust, turns the boat; a rudder in the same
// place, with no propeller wash over it, barely does.
TEST(PhysicsBoat, AnOutboardTurnsItAtLowSpeed) {
  REQUIRE_JOLT();
  const auto heading_after = [](bool outboard) {
    world w(config());
    w.add_field(water());
    boat_desc d = skiff();
    if (outboard) {
      d.propeller.outboard_max_angle = 0.5;
      d.rudder.area = 0.0;
    }
    const boat_id b = w.add_boat(d, {0, 0, kFloat});
    w.step(200);
    w.set_boat_controls(b, {0.2, 1.0});
    w.step(300);
    const boat_state s = w.boat_state(b);
    std::printf("[boat] %s at throttle 0.2: after 3 s heading %.4f rad, yaw rate %.4f rad/s, "
                "speed %.3f m/s, outboard %.3f rad\n",
                outboard ? "outboard" : "rudder  ", s.heading, s.yaw_rate, s.speed,
                s.outboard_angle);
    return s.heading;
  };
  const double outboard = heading_after(true), rudder = heading_after(false);
  EXPECT_GT(outboard, 0.0) << "turning left";
  EXPECT_GT(outboard, 3.0 * std::fabs(rudder)) << "the outboard turns it, the rudder barely";
}

// --- waves (Jolt) --------------------------------------------------------------------------

// Gate (4): on a swell along x the boat pitches with the surface: its bow's rise follows the
// surface's slope at its centre.
TEST(PhysicsBoat, ItPitchesWithTheSwell) {
  REQUIRE_JOLT();
  wave_train train;
  train.amplitude = 0.3;
  train.period_s = 5.0; // 39 m long, a slope of up to 0.048
  auto s = std::make_shared<wave_train_surface>(0.0, 0.0, std::vector<wave_train>{train});
  world w(config());
  w.add_field(sea(s));
  const boat_id b = w.add_boat(skiff(), {0, 0, kFloat});
  w.set_boat_controls(b, {0.3, 0.0});
  w.step(500);
  std::vector<double> pitch, slope;
  int dry = 0;
  for (int i = 0; i < 2000; ++i) {
    w.step();
    const boat_state st = w.boat_state(b);
    const quat &q = st.body.rotation;
    pitch.push_back(2.0 * (q.x * q.z - q.w * q.y)); // the forward axis's z
    const double x = st.body.position.x, y = st.body.position.y, t = w.field_time();
    slope.push_back((s->height(x + 0.5, y, t) - s->height(x - 0.5, y, t)) / 1.0);
    dry += st.propeller_wet ? 0 : 1;
  }
  double lo = 1e9, hi = -1e9;
  for (const double p : pitch) {
    lo = std::min(lo, p);
    hi = std::max(hi, p);
  }
  const double c = correlation(pitch, slope);
  std::printf("[boat] pitch on the swell: correlation with the slope %.4f, pitch %.4f to %.4f, "
              "propeller dry %d of 2000 steps\n",
              c, lo, hi, dry);
  EXPECT_GT(c, 0.9) << "in step with the surface's slope";
  EXPECT_GT(hi - lo, 0.05) << "it pitches";
}

// --- state (Jolt) --------------------------------------------------------------------------

namespace {

std::unique_ptr<world> boat_scene(const boat_desc &d) {
  auto w = std::make_unique<world>(config());
  wave_train train;
  train.amplitude = 0.2;
  train.period_s = 4.0;
  train.dir_rad = 0.3;
  train.phase = 0.7;
  w->add_field(sea(std::make_shared<wave_train_surface>(0.0, 0.0, std::vector<wave_train>{train})));
  const boat_id b = w->add_boat(d, {0, 0, kFloat}, about_z(0.4));
  w->set_boat_controls(b, {0.8, 0.5});
  w->set_field_time(3.0);
  return w;
}

} // namespace

// Gate (5): a boat keeps a run a function of its inputs, and is in the state: its description's
// digest, controls, actuators and forces are saved and hashed, and restore refuses a world
// whose boats differ.
TEST(PhysicsBoat, TheBoatIsInTheStateAndItsHash) {
  REQUIRE_JOLT();
  auto a = boat_scene(skiff()), b = boat_scene(skiff());
  a->step(250);
  for (int i = 0; i < 250; ++i)
    b->step(); // one call a step or many steps a call: the forces go on every step
  EXPECT_EQ(a->state_hash(), b->state_hash());
  boat_desc stronger = skiff();
  stronger.propeller.max_thrust = 2100.0;
  EXPECT_NE(boat_scene(skiff())->state_hash(), boat_scene(stronger)->state_hash())
      << "the description is in the hash";
  auto steered = boat_scene(skiff());
  steered->set_boat_controls(boat_id{0}, {0.8, -0.5});
  EXPECT_NE(steered->state_hash(), boat_scene(skiff())->state_hash()) << "the controls are too";

  const std::vector<std::uint8_t> saved = a->save();
  auto c = boat_scene(skiff());
  c->restore(saved);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  EXPECT_EQ(c->boat_state(boat_id{0}).thrust, a->boat_state(boat_id{0}).thrust);
  a->step(100);
  c->step(100);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  EXPECT_THROW(boat_scene(stronger)->restore(saved), cvc::physics_error) << "another boat";

  // The same hull as a plain body, with no boat: refused both ways.
  auto plain = std::make_unique<world>(config());
  plain->add_field(sea(std::make_shared<wave_train_surface>(
      0.0, 0.0, std::vector<wave_train>{wave_train{0.2, 4.0, 0.3, 0.7}})));
  body_desc hull = skiff().hull;
  hull.position = {0, 0, kFloat};
  plain->add_body(hull);
  EXPECT_THROW(plain->restore(saved), cvc::physics_error) << "a world without the boat";
  EXPECT_THROW(boat_scene(skiff())->restore(plain->save()), cvc::physics_error)
      << "a state without the boat";
}

// The boat scene, pinned. The boat's step is det:: math in a fixed order, so a cross-platform
// deterministic Jolt gives it on every platform.
TEST(PhysicsBoat, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  auto w = boat_scene(skiff());
  w->step(300);
  const std::uint64_t h = w->state_hash();
  std::printf("[physics-golden] boat swell scene after 300 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0xbb8e1b22f58136b2ull) << std::hex << "state hash " << h;
}

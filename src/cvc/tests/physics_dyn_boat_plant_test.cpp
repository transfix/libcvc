/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_boat_plant (dyn_boat_plant.h; docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D4,
// O12): nav's agents on boats as their live plant. With Jolt:
//   (1) the presets' envelopes are what the boats do; the tactical boat's hull grips sideways by
//       its own lift, without a fixed foil, turns at full lock without spinning, and planes;
//   (2) through the driver, a commanded speed and yaw rate are held on calm water and on a swell;
//   (3) the energy books balance, the integrator's residual a small part of the flow;
//   (4) nav drives the boats to their goals on open water, and turns them round to new ones;
//   (5) plant_host puts a sim_world's agents on boats by name, on its water;
//   (6) a host that steps the world runs the plant as it runs itself, a hull that hits a pier is
//       counted, and the scene's hash is pinned.
// The refusals and the hydrostatics run on the null backend, everywhere.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/physics/hull.h>
#include <cvc/physics/plant_host.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::physics;
using cvc::nav::plant_kind;
using cvc::nav::plant_medium;
using cvc::nav::sim_world;

namespace {

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

// Still water up to z = 0, 1200 m square and `depth` deep.
field water(double depth = 10.0) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-600, -600, -depth}, {600, 600, 0});
  f.damping = 1.0e4;
  return f;
}

// A swell on the same water: a wave train from `dir` (rad), the box reaching z = 3.
field swell(double amplitude = 0.3, double period = 5.0, double dir = 0.6) {
  field f = water();
  f.volume = uniform(1000.0, {-600, -600, -10}, {600, 600, 3});
  wave_train t;
  t.amplitude = amplitude;
  t.period_s = period;
  t.dir_rad = dir;
  f.surface = std::make_shared<wave_train_surface>(0.0, 0.0, std::vector<wave_train>{t});
  return f;
}

std::unique_ptr<world> water_world(const field &f) {
  auto w = std::make_unique<world>(config());
  w->add_field(f);
  return w;
}

cvc::world_units::normalized_frame si_frame() {
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return f;
}

cvc::nav::plant_state at(float x, float y, float heading = 0.0f) {
  cvc::nav::plant_state s;
  s.x = x;
  s.y = y;
  s.heading = heading;
  return s;
}

// kinetic now - kinetic at attach - the books: zero to rounding, by construction.
double identity_gap(const dyn_boat_plant &p, int i) {
  const dyn_boat_books &b = p.books(i);
  return b.kinetic - p.kinetic_at_attach(i) -
         (b.drive - b.brake - b.rudder - b.damping + b.water + b.gravity + b.residual);
}

double mean(const std::vector<double> &v) {
  double s = 0.0;
  for (const double x : v)
    s += x / double(v.size());
  return s;
}

constexpr int kPresets = 3;
const char *name_of(int k) { return k == 0 ? "launch" : k == 1 ? "jonboat" : "tactical"; }
dyn_boat_desc preset_of(int k) {
  return k == 0   ? dyn_boat_desc::launch()
         : k == 1 ? dyn_boat_desc::jonboat()
                  : dyn_boat_desc::tactical();
}

// Nav's commands for a speed and a yaw rate, as nav would give them: an acceleration that closes
// the speed's gap within a second (within 1 m/s^2), and the bicycle angle that turns at r there.
void hold(dyn_boat_plant &p, int i, double v, double r) {
  const cvc::nav::plant_state s = p.state(i);
  const double accel = std::max(-1.0, std::min(1.0, v - s.speed));
  p.command(i, static_cast<float>(std::atan(r * p.wheelbase(i) / v)), static_cast<float>(accel));
}

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m

// Agents on open water 200 m square, in nav's trained metric, heading east to goals 120 m on.
sim_world open_sim(int n) {
  constexpr int R = 101, C = 101;
  static const std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = -100.0;
  c.max_x = c.max_y = 100.0;
  c.scale = kS;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6;
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(4.0 * kS);
  std::vector<float> o(2 * n), g(2 * n), col(3 * n, 1.0f);
  for (int i = 0; i < n; ++i) {
    const double y = n == 1 ? 0.0 : -30.0 + 60.0 * i / (n - 1);
    o[2 * i] = static_cast<float>(-60 * kS);
    o[2 * i + 1] = static_cast<float>(y * kS);
    g[2 * i] = static_cast<float>(60 * kS);
    g[2 * i + 1] = static_cast<float>(y * kS);
  }
  return sim_world(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o.data(),
                   g.data(), col.data(), n);
}

std::vector<float> positions(const sim_world &sim) {
  std::vector<float> p(2 * sim.size());
  sim.snapshot(p.data(), nullptr, nullptr, nullptr, nullptr);
  return p;
}

} // namespace

// --- refusals and hydrostatics (null backend) ---------------------------------------------

// A box floats where it displaces its mass: the launch's 900 kg in 4 x 1.5 m of fresh water draws
// 0.15 m, so its centre (the origin) rides 0.15 m over the water; the jonboat's 350 kg in 3.6 x
// 1.4 m draws 0.0694 m.
TEST(DynBoatPlant, FloatsAtTheDraftItsHullGives) {
  EXPECT_NEAR(dyn_boat_plant::float_height(dyn_boat_desc::launch().boat, 900.0, 1000.0), 0.15,
              1e-9);
  EXPECT_NEAR(dyn_boat_plant::float_height(dyn_boat_desc::jonboat().boat, 350.0, 1000.0),
              0.25 - 350.0 / (3.6 * 1.4 * 1000.0), 1e-9);
  EXPECT_NEAR(dyn_boat_plant::float_height(dyn_boat_desc::launch().boat, 900.0, 1025.0),
              0.3 - 900.0 / (6.0 * 1025.0), 1e-9)
      << "sea water floats it higher";
  EXPECT_THROW(dyn_boat_plant::float_height(dyn_boat_desc::launch().boat, 4000.0, 1000.0),
               cvc::physics_error)
      << "3.6 m^3 of hull cannot carry 4 t";
  boat_desc no_mesh = dyn_boat_desc::launch().boat;
  no_mesh.hull.hull.reset();
  EXPECT_THROW(dyn_boat_plant::float_height(no_mesh, 900.0, 1000.0), cvc::physics_error);
}

// The tactical preset is its model's own hull (the vehicle-tactical-boat package's TacticalBoat,
// baked into 16 stations): a closed V hull, 6.85 m long and 2.33 m in beam, 7.8 m overall, that
// floats 0.49 m over its keel at 2800 kg; its collision shape the hull's convex hull; its water
// cut-out inside it at that waterline. The hull has its own lift; its rudder is its outboards'
// legs, at their propellers aft of the transom, swinging with them.
TEST(DynBoatPlant, TheTacticalBoatIsItsModelsHull) {
  const dyn_boat_desc d = dyn_boat_desc::tactical();
  ASSERT_TRUE(d.boat.hull.hull);
  const hull_mesh &h = *d.boat.hull.hull;
  EXPECT_EQ(h.vertices.size(), 242u);
  EXPECT_EQ(h.triangles.size(), 480u);
  EXPECT_DOUBLE_EQ(h.lift_coefficient, 0.56);
  EXPECT_EQ(d.boat.rudder.position.x, d.boat.propeller.position.x);
  EXPECT_EQ(d.boat.rudder.max_angle, d.boat.propeller.outboard_max_angle);
  EXPECT_TRUE(std::holds_alternative<convex_hull>(d.boat.hull.shape));
  EXPECT_DOUBLE_EQ(d.length_overall, 7.8);
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  for (const vec3 &v : h.vertices) {
    const double p[3] = {v.x, v.y, v.z};
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
  EXPECT_NEAR(hi[0] - lo[0], 6.85, 1e-9);
  EXPECT_NEAR(hi[1] - lo[1], 2.3296, 1e-3);
  const double waterline = -dyn_boat_plant::float_height(d.boat, d.boat.hull.mass, 1000.0);
  std::printf("[boat] tactical: waterline %.4f m over its keel; ", waterline - lo[2]);
  EXPECT_NEAR(waterline - lo[2], 0.488, 0.002);
  const auto box = hull_waterline_box(h, waterline);
  ASSERT_TRUE(box.has_value());
  std::printf("water cut-out: centre %+.3f m, half length %.3f m, half beam %.3f m\n", box->x,
              box->half_length, box->half_beam);
  EXPECT_LT(box->half_beam, 0.5 * (hi[1] - lo[1]));
  EXPECT_GT(box->half_beam, 0.3);
  EXPECT_GE(box->x - box->half_length, lo[0] - 1e-9);
  EXPECT_LT(box->x + box->half_length, hi[0]) << "short of its bow";
  EXPECT_GT(box->half_length, 2.0);
  EXPECT_LT(d.boat.rudder.position.x, lo[0]) << "the legs are aft of the transom";
}

TEST(DynBoatPlant, RefusesBadParametersAndStartsWithoutWater) {
  world w(config(backend::null));
  dyn_boat_plant_options o;
  EXPECT_THROW(dyn_boat_plant({dyn_boat_desc::launch()}, o), cvc::physics_error) << "no world";
  o.physics = &w;
  EXPECT_THROW(dyn_boat_plant({}, o), cvc::physics_error) << "no boats";
  const auto refused = [&](dyn_boat_desc d, const char *why) {
    EXPECT_THROW(dyn_boat_plant({d}, o), cvc::physics_error) << why;
  };
  dyn_boat_desc d = dyn_boat_desc::launch();
  d.max_speed = 0.0;
  refused(d, "no top speed");
  d = dyn_boat_desc::launch();
  d.turn_radius = -1.0;
  refused(d, "a negative turning radius");
  d = dyn_boat_desc::launch();
  d.turn_radius = std::nan("");
  refused(d, "a turning radius not finite");
  d = dyn_boat_desc::launch();
  d.boat.hull.hull.reset();
  refused(d, "no hull_mesh");
  d = dyn_boat_desc::launch();
  d.boat.propeller.max_thrust = -1.0;
  refused(d, "add_boat's own refusal");
  d = dyn_boat_desc::launch();
  d.boat.hull.motion = motion_type::kinematic;
  refused(d, "a hull that is not dynamic");
  dyn_boat_plant_options bad = o;
  bad.plant.metres_per_world_unit = 0.0;
  EXPECT_THROW(dyn_boat_plant({dyn_boat_desc::launch()}, bad), cvc::physics_error);
  EXPECT_THROW(dyn_boat_desc::preset("tug"), cvc::physics_error);

  dyn_boat_plant plant({dyn_boat_desc::launch(), dyn_boat_desc::jonboat()}, o);
  EXPECT_EQ(plant.kind(), plant_kind::dynamic);
  EXPECT_EQ(plant.medium(), plant_medium::water);
  const cvc::nav::plant_state starts[2] = {at(0.0f, 0.0f, 0.5f), at(20.0f, 5.0f, -1.0f)};
  EXPECT_THROW(plant.attach(2, starts, si_frame(), 0.06), cvc::physics_error)
      << "a world with no water";
  EXPECT_EQ(w.boat_count(), 0u);
  EXPECT_EQ(plant.size(), 0);

  w.add_field(water());
  const cvc::nav::plant_state off[2] = {at(0.0f, 0.0f), at(700.0f, 0.0f)};
  EXPECT_THROW(plant.attach(2, off, si_frame(), 0.06), cvc::physics_error)
      << "a start past the water";
  EXPECT_EQ(w.boat_count(), 0u) << "a refused attach leaves none of its boats";
  EXPECT_THROW(plant.attach(2, starts, si_frame(), 0.065), cvc::physics_error)
      << "a tick that is not a whole number of steps";

  // A pond 0.1 m deep: the launch draws 0.15 m.
  world shallow(config(backend::null));
  shallow.add_field(water(0.1));
  dyn_boat_plant_options so = o;
  so.physics = &shallow;
  dyn_boat_plant grounded({dyn_boat_desc::launch()}, so);
  EXPECT_THROW(grounded.attach(1, starts, si_frame(), 0.06), cvc::physics_error);
  EXPECT_EQ(shallow.boat_count(), 0u);
  dyn_boat_desc heavy = dyn_boat_desc::launch();
  heavy.boat.hull.mass = 5000.0;
  dyn_boat_plant sinks({heavy}, o);
  EXPECT_THROW(sinks.attach(1, starts, si_frame(), 0.06), cvc::physics_error);
  EXPECT_EQ(w.boat_count(), 0u);

  // On the water: each boat over its start, heading its way, at its draft.
  plant.attach(2, starts, si_frame(), 0.06);
  EXPECT_EQ(plant.substeps(), 6);
  ASSERT_EQ(plant.size(), 2);
  EXPECT_EQ(w.boat_count(), 2u);
  const body_state h0 = w.state(plant.hull(0)), h1 = w.state(plant.hull(1));
  EXPECT_NEAR(h0.position.z, 0.15, 1e-9);
  EXPECT_NEAR(h1.position.z, 0.25 - 350.0 / (3.6 * 1.4 * 1000.0), 1e-9);
  const cvc::nav::plant_state s1 = plant.state(1);
  EXPECT_NEAR(s1.x, 20.0f, 1e-5);
  EXPECT_NEAR(s1.y, 5.0f, 1e-5);
  EXPECT_NEAR(s1.heading, -1.0f, 1e-6);
  EXPECT_EQ(s1.speed, 0.0f);
  EXPECT_EQ(plant.desc(1).boat.hull.mass, 350.0);
  // The launch's envelope.
  const cvc::nav::plant_limits l = plant.limits(0);
  EXPECT_FLOAT_EQ(l.wheelbase, 4.0f);
  EXPECT_FLOAT_EQ(l.max_steer, static_cast<float>(std::atan(4.0 / 15.4)));
  EXPECT_FLOAT_EQ(l.a_brake, static_cast<float>(0.8 * 1000.0 / 900.0));
  EXPECT_FLOAT_EQ(l.latency, 0.6f);
  EXPECT_FLOAT_EQ(l.vmax, 3.19f);
  EXPECT_FLOAT_EQ(l.half_width, 0.75f);
  EXPECT_FLOAT_EQ(l.front, 2.0f);
  EXPECT_FLOAT_EQ(l.rear, 2.0f);
  // A new attach takes the old boats away; boats come and go at run time.
  plant.attach(2, starts, si_frame(), 0.06);
  EXPECT_EQ(w.boat_count(), 2u);
  EXPECT_EQ(plant.add_vehicle(at(-30.0f, 0.0f), 1), 2);
  EXPECT_EQ(w.boat_count(), 3u);
  EXPECT_EQ(plant.desc(2).boat.hull.mass, 350.0);
  plant.remove_vehicle(0);
  EXPECT_EQ(w.boat_count(), 2u);
  EXPECT_EQ(plant.size(), 2);
  EXPECT_THROW(plant.add_vehicle(at(0.0f, 0.0f), 5), std::invalid_argument);
  EXPECT_THROW(plant.add_vehicle(at(900.0f, 0.0f)), cvc::physics_error) << "no water there";
  EXPECT_EQ(plant.size(), 2);
}

// --- the presets (Jolt) ----------------------------------------------------------------------

// Gate (1): each preset's max_speed and turn_radius are what it does on calm water: its top speed
// at full throttle within 3 %, and its circle at full rudder and full throttle within 10 %.
TEST(DynBoatPlant, PresetsDoWhatTheySay) {
  REQUIRE_JOLT();
  for (int k = 0; k < kPresets; ++k) {
    auto w = water_world(water());
    dyn_boat_plant_options o;
    o.physics = w.get();
    o.driver = false; // the controls, raw
    dyn_boat_plant p({preset_of(k)}, o);
    const cvc::nav::plant_state s0 = at(-300.0f, 0.0f);
    p.attach(1, &s0, si_frame(), 0.06);
    for (int t = 0; t < 1000; ++t) { // 60 s, full ahead
      p.command(0, 0.0f, 50.0f);
      p.advance();
    }
    std::vector<double> u;
    for (int t = 0; t < 100; ++t) {
      p.command(0, 0.0f, 50.0f);
      p.advance();
      u.push_back(p.boat_state(0).speed);
    }
    const double top = mean(u);
    std::vector<double> speed, rate;
    for (int t = 0; t < 1500; ++t) { // 90 s, full rudder
      p.command(0, static_cast<float>(p.max_steer(0)), 50.0f);
      p.advance();
      if (t >= 1000) {
        const cvc::physics::boat_state b = p.boat_state(0);
        speed.push_back(std::hypot(b.speed, b.sway));
        rate.push_back(b.yaw_rate);
      }
    }
    const double radius = mean(speed) / mean(rate);
    std::printf("  %s: top speed %.4f m/s (says %.2f); full rudder: %.3f m/s at %.4f rad/s, radius "
                "%.2f m (says %.1f)\n",
                name_of(k), top, preset_of(k).max_speed, mean(speed), mean(rate), radius,
                preset_of(k).turn_radius);
    EXPECT_NEAR(top, preset_of(k).max_speed, 0.03 * preset_of(k).max_speed) << name_of(k);
    EXPECT_NEAR(radius, preset_of(k).turn_radius, 0.1 * preset_of(k).turn_radius) << name_of(k);
  }
}

// --- the tactical hull's lift (Jolt) -------------------------------------------------------

namespace {

constexpr double kDeg = 57.29577951308232;

vec3 cross_of(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// v rotated by the unit quaternion q.
vec3 rotated(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = cross_of(u, v) * 2.0;
  return v + t * q.w + cross_of(u, t);
}

// The tactical boat with its hull's lift_coefficient `lift` (0: none), with its legs or without.
dyn_boat_desc tactical_with(double lift, bool legs = true) {
  dyn_boat_desc d = dyn_boat_desc::tactical();
  auto m = std::make_shared<hull_mesh>(*d.boat.hull.hull);
  m->lift_coefficient = lift;
  d.boat.hull.hull = m;
  if (!legs)
    d.boat.rudder.area = 0.0;
  return d;
}

// Still water 40 km square, for runs at speed.
field open_water() {
  field f = water();
  f.volume = uniform(1000.0, {-20000, -20000, -10}, {20000, 20000, 0});
  return f;
}

struct turned_run {
  double straight = 0.0, speed = 0.0, rate = 0.0, drift = 0.0;
  double radius() const { return speed / std::fabs(rate); }
  double lateral() const { return speed * std::fabs(rate); } // m/s^2
};

// A boat at `throttle` on calm water, 30 s straight and then 48 s at full lock (`lock` +1 to
// port): its straight speed, and over the turn's last 18 s its mean speed, yaw rate and drift
// (the angle its course makes with its heading, + to port).
turned_run full_lock(const dyn_boat_desc &d, double throttle, double lock) {
  auto w = water_world(water());
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({d}, o);
  const cvc::nav::plant_state s0 = at(0.0f, 0.0f);
  p.attach(1, &s0, si_frame(), 0.06);
  p.hold_controls(0, {throttle, 0.0});
  for (int t = 0; t < 500; ++t)
    p.advance();
  turned_run r;
  r.straight = p.boat_state(0).speed;
  p.hold_controls(0, {throttle, lock});
  std::vector<double> speed, rate, drift;
  for (int t = 0; t < 800; ++t) {
    p.advance();
    if (t >= 500) {
      const cvc::physics::boat_state b = p.boat_state(0);
      speed.push_back(std::hypot(b.speed, b.sway));
      rate.push_back(b.yaw_rate);
      drift.push_back(std::atan2(b.sway, b.speed));
    }
  }
  r.speed = mean(speed);
  r.rate = mean(rate);
  r.drift = mean(drift);
  return r;
}

// The tactical hull alone (no boat), floating level at its draft and moving at 8 m/s ahead and
// `beta` (rad) to port: the water's load on it. The pose is set before each of two steps and the
// second step's load taken, so the triangles' wetted areas are the pose's and nothing slams.
field_load drifting(const hull_mesh &mesh, double beta) {
  const dyn_boat_desc d = dyn_boat_desc::tactical();
  world w(config());
  w.add_field(water());
  body_desc b = d.boat.hull;
  b.hull = std::make_shared<hull_mesh>(mesh);
  b.position = {0.0, 0.0, dyn_boat_plant::float_height(d.boat, b.mass, 1000.0)};
  const body_id id = w.add_body(b);
  body_state s = w.state(id);
  s.linear_velocity = {8.0 * std::cos(beta), 8.0 * std::sin(beta), 0.0};
  for (int k = 0; k < 2; ++k) {
    w.set_state(id, s);
    w.step();
  }
  return w.field_load_on(id);
}

} // namespace

// The tactical boat grips without a fixed foil: its hull's lift is its keel's and chines' grip.
//   (1) Drifting at 8 m/s from its level float, the lift more than triples the hull's side force
//       per radian of drift, growing about as the drift (where the pressure drag's grows faster):
//       its own share 0.2 m ahead of the centre of mass, the pressure drag's 0.7 m ahead, on the
//       bow. The hull's side force is ahead of its centre of mass either way, so the hull alone
//       yaws away from its course: the boat's legs, far aft, hold it straight.
//   (2) At full lock and full throttle it turns tighter, at more lateral acceleration and less
//       drift, than the same boat without the lift.
//   (3) Without the lift and the legs (the bare hull, as before its foil) it spins.
TEST(DynBoatPlant, TheTacticalHullGripsWithoutAFoil) {
  REQUIRE_JOLT();
  const dyn_boat_desc d = dyn_boat_desc::tactical();
  ASSERT_GT(d.boat.hull.hull->lift_coefficient, 0.0);
  hull_mesh bare = *d.boat.hull.hull;
  bare.lift_coefficient = 0.0;
  const double b2 = 2.0 / kDeg, b4 = 4.0 / kDeg;
  const field_load on_p2 = drifting(*d.boat.hull.hull, b2),
                   on_m2 = drifting(*d.boat.hull.hull, -b2),
                   on_p4 = drifting(*d.boat.hull.hull, b4);
  const field_load off_p2 = drifting(bare, b2), off_m2 = drifting(bare, -b2),
                   off_p4 = drifting(bare, b4);
  // Per radian, from +-2 degrees; the yaw moment's lever, N_z / Y.
  const double y_on = (on_p2.force.y - on_m2.force.y) / (2.0 * b2);
  const double y_off = (off_p2.force.y - off_m2.force.y) / (2.0 * b2);
  const double n_on = (on_p2.torque.z - on_m2.torque.z) / (2.0 * b2);
  const double n_off = (off_p2.torque.z - off_m2.torque.z) / (2.0 * b2);
  const double y_lift = y_on - y_off, x_lift = (n_on - n_off) / y_lift, x_drag = n_off / y_off;
  const double lift2 = on_p2.force.y - off_p2.force.y, lift4 = on_p4.force.y - off_p4.force.y;
  std::printf("[boat] tactical hull drifting at 8 m/s: side force %.0f N/rad without its lift "
              "(%+.2f m from its centre of mass), %.0f N/rad with it; the lift's %.0f N/rad at "
              "%+.2f m; from 2 to 4 degrees the lift's grows %.3f times, the pressure drag's "
              "%.3f\n",
              y_off, x_drag, y_on, y_lift, x_lift, lift4 / lift2, off_p4.force.y / off_p2.force.y);
  EXPECT_LT(y_off, 0.0) << "the water pushes back on a drift";
  EXPECT_LT(y_on, 3.0 * y_off) << "the lift more than triples the grip";
  EXPECT_GT(lift4 / lift2, 1.8) << "about as the drift";
  EXPECT_LT(lift4 / lift2, 2.6);
  EXPECT_LT(lift4 / lift2, off_p4.force.y / off_p2.force.y);
  EXPECT_GT(x_drag, 0.5) << "the pressure drag's side force is on the bow";
  EXPECT_LT(x_lift, 0.5 * x_drag) << "the lift's nearer the centre of mass";

  const turned_run lifted = full_lock(d, 1.0, 1.0),
                   unlifted = full_lock(tactical_with(0.0), 1.0, 1.0);
  const turned_run bare_hull = full_lock(tactical_with(0.0, false), 1.0, 1.0);
  const auto show = [](const char *what, const turned_run &r) {
    std::printf("  %s: %.3f m/s straight; full lock %.3f m/s at %.4f rad/s, radius %.2f m, %.2f "
                "m/s^2, drift %.2f deg\n",
                what, r.straight, r.speed, r.rate, r.radius(), r.lateral(), r.drift * kDeg);
  };
  show("with its lift", lifted);
  show("without", unlifted);
  show("bare hull", bare_hull);
  EXPECT_LT(lifted.radius(), 0.8 * unlifted.radius());
  EXPECT_GT(lifted.lateral(), 1.25 * unlifted.lateral());
  EXPECT_LT(std::fabs(lifted.drift), std::fabs(unlifted.drift));
  EXPECT_LT(bare_hull.radius(), 2.0) << "the bare hull spins on the spot";
}

// At full lock either way, from a crawl to full throttle, the tactical boat turns on a steady
// circle without spinning: 15-40 m across its range, under 20 degrees of drift, keeping most of
// its speed; to starboard as to port (the loft's quads are split the same way on both sides, so
// its triangles are not quite each other's mirror: the drift differs by a few per cent).
TEST(DynBoatPlant, TheTacticalBoatDoesNotSpinAtFullLock) {
  REQUIRE_JOLT();
  const dyn_boat_desc d = dyn_boat_desc::tactical();
  for (const double throttle : {0.15, 0.5, 1.0}) {
    const turned_run r = full_lock(d, throttle, 1.0);
    std::printf("  throttle %.2f: %.3f m/s straight; full lock %.3f m/s at %.4f rad/s, radius "
                "%.2f m, drift %.2f deg\n",
                throttle, r.straight, r.speed, r.rate, r.radius(), r.drift * kDeg);
    EXPECT_GT(r.rate, 0.0) << "to port";
    EXPECT_GT(r.radius(), 15.0) << throttle;
    EXPECT_LT(r.radius(), 40.0) << throttle;
    EXPECT_LT(std::fabs(r.drift), 20.0 / kDeg) << throttle;
    EXPECT_GT(r.speed, 0.75 * r.straight) << throttle;
    if (throttle == 1.0) {
      const turned_run s = full_lock(d, throttle, -1.0);
      std::printf("  to starboard: %.3f m/s at %.4f rad/s, radius %.2f m, drift %.2f deg\n",
                  s.speed, s.rate, s.radius(), s.drift * kDeg);
      EXPECT_LT(s.rate, 0.0);
      EXPECT_NEAR(s.radius(), r.radius(), 0.02 * r.radius());
      EXPECT_NEAR(s.drift, -r.drift, 0.06 * std::fabs(r.drift));
    }
  }
}

namespace {

struct planing_point {
  double speed = 0.0, thrust = 0.0, trim = 0.0, rise = 0.0, area = 0.0, length = 0.0;
};

// The hull's wetted area and the length along it of its wetted part, under calm water at z = 0.
void wetted(const hull_mesh &h, const body_state &s, double &area, double &length) {
  std::vector<vec3> wv(h.vertices.size());
  for (std::size_t i = 0; i < h.vertices.size(); ++i)
    wv[i] = s.position + rotated(s.rotation, h.vertices[i]);
  double x0 = 1e300, x1 = -1e300;
  area = 0.0;
  for (const auto &t : h.triangles) {
    // The triangle clipped to z < 0, and the body x of its corners there.
    vec3 q[4];
    int n = 0;
    const auto corner = [&](std::uint32_t ia, std::uint32_t ib, double f) {
      q[n++] = wv[ia] + (wv[ib] - wv[ia]) * f;
      const double x = h.vertices[ia].x + (h.vertices[ib].x - h.vertices[ia].x) * f;
      x0 = std::min(x0, x);
      x1 = std::max(x1, x);
    };
    for (int k = 0; k < 3; ++k) {
      const std::uint32_t ia = t[k], ib = t[(k + 1) % 3];
      const bool ina = wv[ia].z < 0.0, inb = wv[ib].z < 0.0;
      if (ina)
        corner(ia, ib, 0.0);
      if (ina != inb)
        corner(ia, ib, wv[ia].z / (wv[ia].z - wv[ib].z));
    }
    for (int k = 1; k + 1 < n; ++k) {
      const vec3 c = cross_of(q[k] - q[0], q[k + 1] - q[0]);
      area += 0.5 * std::sqrt(dot(c, c));
    }
  }
  length = x1 - x0;
}

// A boat on calm water at rest for 12 s, then at each throttle in turn for 36 s: at rest and at
// the end of each, the means over 3 s of its speed, thrust, trim (bow up), the rise of its hull's
// centroid and its wetted area and length. The trims and rises are from rest.
std::vector<planing_point> sweep(const dyn_boat_desc &d, const std::vector<double> &throttles) {
  auto w = water_world(open_water());
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({d}, o);
  const cvc::nav::plant_state s0 = at(-5000.0f, 0.0f);
  p.attach(1, &s0, si_frame(), 0.06);
  const hull_mesh &h = *d.boat.hull.hull;
  vec3 centroid;
  double volume = 0.0;
  for (const auto &t : h.triangles) {
    const vec3 &a = h.vertices[t[0]], &b = h.vertices[t[1]], &c = h.vertices[t[2]];
    const double v6 =
        dot(a, vec3{b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x});
    volume += v6 / 6.0;
    centroid = centroid + (a + b + c) * (v6 / 24.0);
  }
  centroid = centroid * (1.0 / volume);
  const auto measure = [&](int ticks) {
    planing_point m;
    for (int t = 0; t < ticks; ++t) {
      p.advance();
      const cvc::physics::boat_state b = p.boat_state(0);
      double area = 0.0, length = 0.0;
      wetted(h, b.body, area, length);
      m.speed += b.speed / ticks;
      m.thrust += b.thrust / ticks;
      m.trim += std::asin(rotated(b.body.rotation, {1.0, 0.0, 0.0}).z) / ticks;
      m.rise += (b.body.position.z + rotated(b.body.rotation, centroid).z) / ticks;
      m.area += area / ticks;
      m.length += length / ticks;
    }
    return m;
  };
  p.hold_controls(0, {0.0, 0.0});
  for (int t = 0; t < 150; ++t)
    p.advance();
  std::vector<planing_point> out{measure(50)};
  for (const double throttle : throttles) {
    p.hold_controls(0, {throttle, 0.0});
    for (int t = 0; t < 550; ++t)
      p.advance();
    out.push_back(measure(50));
    out.back().trim -= out.front().trim;
    out.back().rise -= out.front().rise;
  }
  out.front().trim = out.front().rise = 0.0;
  return out;
}

void print_sweep(const char *what, const dyn_boat_desc &d, const std::vector<double> &throttles,
                 const std::vector<planing_point> &s) {
  const double weight = d.boat.hull.mass * 9.80665;
  std::printf("  %s, %.0f kN: throttle, speed (m/s), Fn_vol, thrust/weight, trim (deg), rise "
              "(m), wetted area (m^2), wetted length (m)\n",
              what, d.boat.propeller.max_thrust / 1000.0);
  for (std::size_t i = 0; i < s.size(); ++i)
    std::printf("    %4.2f  %6.3f  %5.2f  %.4f  %6.3f  %6.4f  %6.3f  %5.3f\n",
                i ? throttles[i - 1] : 0.0, s[i].speed,
                s[i].speed / std::sqrt(9.80665 * std::cbrt(d.boat.hull.mass / 1000.0)),
                s[i].thrust / weight, s[i].trim * kDeg, s[i].rise, s[i].area, s[i].length);
}

} // namespace

// The tactical hull planes: as it goes faster it trims bow up, rises and runs on less of its
// bottom, by its lift. At its own 8 kN it is rising onto the plane at its top speed, where the
// hull without the lift barely rises; given 30 kN (and no thrust falloff short of 40 m/s) it planes
// on a third of its bottom, its trim levelling off and its resistance no longer growing as the
// speed squared. The triangle model has no wave-making resistance, so there is no hump in the
// resistance as such: it rises to a knee and flattens.
TEST(DynBoatPlant, TheTacticalHullPlanesAtSpeed) {
  REQUIRE_JOLT();
  const dyn_boat_desc d = dyn_boat_desc::tactical();
  const std::vector<double> throttles{0.1, 0.3, 0.6, 1.0};
  const std::vector<planing_point> s = sweep(d, throttles);
  print_sweep("tactical", d, throttles, s);
  for (std::size_t i = 1; i < s.size(); ++i) {
    EXPECT_GT(s[i].speed, s[i - 1].speed) << i;
    EXPECT_GT(s[i].rise, s[i - 1].rise) << i;
    EXPECT_GT(s[i].trim, s[i - 1].trim) << i;
    EXPECT_LT(s[i].area, s[i - 1].area) << i;
    EXPECT_LT(s[i].length, s[i - 1].length) << i;
  }
  const planing_point &top = s.back(), &rest = s.front();
  EXPECT_GT(top.rise, 0.10);
  EXPECT_GT(top.trim, 2.0 / kDeg);
  EXPECT_LT(top.trim, 4.0 / kDeg);
  EXPECT_LT(top.area, 0.82 * rest.area);
  EXPECT_LT(top.length, 0.95 * rest.length);
  const std::vector<double> full{1.0};
  const std::vector<planing_point> u = sweep(tactical_with(0.0), full);
  print_sweep("without its lift", tactical_with(0.0), full, u);
  EXPECT_LT(u.back().rise, 0.5 * top.rise);
  EXPECT_GT(u.back().area, 0.9 * rest.area);

  dyn_boat_desc fast = d;
  fast.boat.propeller.max_thrust = 30000.0;
  fast.boat.propeller.max_inflow_speed = 40.0;
  const std::vector<double> more{0.1, 0.2, 0.4, 0.7, 1.0};
  const std::vector<planing_point> f = sweep(fast, more);
  print_sweep("tactical", fast, more, f);
  const planing_point &a = f[1], &b = f.back();
  EXPECT_GT(b.speed, 25.0);
  EXPECT_GT(b.rise, 0.3) << "most of its 0.49 m draft";
  EXPECT_LT(b.area, 0.35 * rest.area);
  EXPECT_LT(b.length, 0.8 * rest.length);
  EXPECT_NEAR(b.trim, f[f.size() - 2].trim, 0.15 / kDeg) << "the trim levels off";
  const double ra = a.thrust / (a.speed * a.speed), rb = b.thrust / (b.speed * b.speed);
  std::printf("  resistance over speed squared: %.3f N s^2/m^2 at %.2f m/s, %.3f at %.2f m/s\n", ra,
              a.speed, rb, b.speed);
  EXPECT_LT(rb, 0.2 * ra) << "planing, it no longer resists as the speed squared";
}

// --- the driver (Jolt) ---------------------------------------------------------------------

namespace {

struct held {
  double speed = 0.0, rate = 0.0;
};

// Agent 0 of a plant on `f` holding 2.5 m/s straight for 40 s, then 2.5 m/s and 0.08 rad/s for 60
// s: the means over the last 10 s straight and the last 30 s of the turn (six of a 5 s swell's
// periods).
void hold_run(const field &f, int k, held &straight, held &turning) {
  auto w = water_world(f);
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({preset_of(k)}, o);
  const cvc::nav::plant_state s0 = at(-150.0f, 0.0f);
  p.attach(1, &s0, si_frame(), 0.06);
  std::vector<double> u, r;
  const auto run = [&](int ticks, int from, double v, double rate, held &out) {
    u.clear();
    r.clear();
    for (int t = 0; t < ticks; ++t) {
      hold(p, 0, v, rate);
      p.advance();
      if (t >= from) {
        u.push_back(p.state(0).speed);
        r.push_back(p.sample(0).r);
      }
    }
    out.speed = mean(u);
    out.rate = mean(r);
  };
  run(667, 500, 2.5, 0.0, straight);
  run(1000, 500, 2.5, 0.08, turning);
}

} // namespace

// Gate (2): through the driver, nav's commands hold 2.5 m/s within 2 % and 0.08 rad/s within 5 %
// on calm water, for both presets.
TEST(DynBoatPlant, HoldsASpeedAndAYawRateOnCalmWater) {
  REQUIRE_JOLT();
  for (int k = 0; k < 2; ++k) {
    held a, b;
    hold_run(water(), k, a, b);
    std::printf("  %s, calm: straight %.4f m/s, %.5f rad/s; turning %.4f m/s, %.5f rad/s\n",
                name_of(k), a.speed, a.rate, b.speed, b.rate);
    EXPECT_NEAR(a.speed, 2.5, 0.05) << name_of(k);
    EXPECT_NEAR(a.rate, 0.0, 0.004) << name_of(k);
    EXPECT_NEAR(b.speed, 2.5, 0.05) << name_of(k);
    EXPECT_NEAR(b.rate, 0.08, 0.004) << name_of(k);
  }
}

// ... and on a 0.3 m, 5 s swell from 0.6 rad, over whole periods: within 4 % and 8 %.
TEST(DynBoatPlant, HoldsASpeedAndAYawRateOnASwell) {
  REQUIRE_JOLT();
  for (int k = 0; k < 2; ++k) {
    held a, b;
    hold_run(swell(), k, a, b);
    std::printf("  %s, swell: straight %.4f m/s, %.5f rad/s; turning %.4f m/s, %.5f rad/s\n",
                name_of(k), a.speed, a.rate, b.speed, b.rate);
    EXPECT_NEAR(a.speed, 2.5, 0.1) << name_of(k);
    EXPECT_NEAR(a.rate, 0.0, 0.0064) << name_of(k);
    EXPECT_NEAR(b.speed, 2.5, 0.1) << name_of(k);
    EXPECT_NEAR(b.rate, 0.08, 0.0064) << name_of(k);
  }
}

// --- the books (Jolt) ----------------------------------------------------------------------

// Gate (3): a weave up to speed and a hard stop, on calm water and on the swell. The books close
// by construction; the integrator's residual is a small part of the flow; the thrust delivers
// (drive) and holds back (brake, the stop astern); the rudder takes energy; nav_stats' energy is
// the last tick's drive. At a steady speed on calm water the water takes what the thrust gives.
TEST(DynBoatPlant, TheBooksBalance) {
  REQUIRE_JOLT();
  for (int sea = 0; sea < 2; ++sea)
    for (int k = 0; k < 2; ++k) {
      auto w = water_world(sea ? swell() : water());
      dyn_boat_plant_options o;
      o.physics = w.get();
      dyn_boat_plant p({preset_of(k)}, o);
      const cvc::nav::plant_state s0 = at(-100.0f, 0.0f);
      p.attach(1, &s0, si_frame(), 0.06);
      double drive_ticks = 0.0;
      for (int t = 0; t < 750; ++t) { // 45 s
        const double time = 0.06 * t;
        const double steer = 0.6 * p.max_steer(0) * std::sin(2.0 * 3.14159265 * time / 15.0);
        const double accel = time < 30.0 ? std::min(1.0, 2.5 - p.state(0).speed) : -1.0;
        p.command(0, static_cast<float>(steer), static_cast<float>(accel));
        p.advance();
        drive_ticks += p.energy_step(0);
        EXPECT_EQ(p.energy_step(0), p.last_tick(0).drive);
      }
      const dyn_boat_books &b = p.books(0);
      std::printf("  %s, %s, 45 s: kinetic %.4g J, drive %.5g, brake %.4g, rudder %.4g, water "
                  "%.5g, gravity %.4g, residual %.3g (%.2e of the flow)\n",
                  name_of(k), sea ? "swell" : "calm", b.kinetic, b.drive, b.brake, b.rudder,
                  b.water, b.gravity, b.residual, std::fabs(b.residual) / b.flow());
      EXPECT_NEAR(identity_gap(p, 0), 0.0, 1e-9 * b.flow());
      EXPECT_NEAR(drive_ticks, b.drive, 1e-9 * b.drive);
      EXPECT_GT(b.drive, 0.0);
      EXPECT_GT(b.brake, 0.0) << "the stop backs the propeller";
      EXPECT_GT(b.rudder, 0.0);
      EXPECT_EQ(b.damping, 0.0) << "the presets' hulls have no damping of their own";
      EXPECT_LT(b.water, 0.0) << "the water takes energy overall";
      EXPECT_LT(std::fabs(b.residual) / b.flow(), 1e-3);
    }

  // Steady on calm water: over 6 s at 2.5 m/s the thrust's power goes into the water.
  auto w = water_world(water());
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({dyn_boat_desc::launch()}, o);
  const cvc::nav::plant_state s0 = at(-100.0f, 0.0f);
  p.attach(1, &s0, si_frame(), 0.06);
  for (int t = 0; t < 600; ++t) {
    hold(p, 0, 2.5, 0.0);
    p.advance();
  }
  const dyn_boat_books before = p.books(0);
  for (int t = 0; t < 100; ++t) {
    hold(p, 0, 2.5, 0.0);
    p.advance();
  }
  const dyn_boat_books &after = p.books(0);
  const double drive = after.drive - before.drive, water_w = after.water - before.water,
               rudder = after.rudder - before.rudder;
  std::printf("  launch steady at 2.5 m/s for 6 s: drive %.2f J, water %.2f J, rudder %.3f J\n",
              drive, water_w, rudder);
  EXPECT_NEAR(drive - rudder + water_w, 0.0, 0.02 * drive);
}

// --- nav drives the boat (Jolt) ------------------------------------------------------------

// Gate (4), the roadmap's exit for O12: nav drives the boats. A launch and a jonboat start at rest
// 120 m west of their goals on open water and reach them; then each is sent to a goal behind it
// and to the side, turns round and reaches that too. Their books balance all the way.
TEST(DynBoatPlant, NavDrivesTheBoatsToTheirGoals) {
  REQUIRE_JOLT();
  sim_world sim = open_sim(2);
  auto w = water_world(water());
  dyn_boat_plant_options o;
  o.physics = w.get();
  o.plant.metres_per_world_unit = 1.0;
  dyn_boat_plant plant({dyn_boat_desc::launch(), dyn_boat_desc::jonboat()}, o);
  sim.set_plant(&plant);
  EXPECT_EQ(sim.plant_type(), plant_kind::dynamic);
  const auto run = [&](int limit, const char *leg) {
    std::vector<std::uint8_t> reached(2, 0);
    int t = 0;
    for (; t < limit; ++t) {
      sim.step();
      sim.snapshot(nullptr, nullptr, nullptr, nullptr, reached.data());
      if (reached[0] && reached[1])
        break;
    }
    const std::vector<float> p = positions(sim);
    std::printf("  %s: launch at (%.1f, %.1f), jonboat at (%.1f, %.1f), reached %d %d after %.1f "
                "s\n",
                leg, p[0], p[1], p[2], p[3], reached[0], reached[1], 0.06 * t);
    EXPECT_TRUE(reached[0]) << leg << ": the launch did not reach its goal";
    EXPECT_TRUE(reached[1]) << leg << ": the jonboat did not reach its goal";
  };
  run(2500, "east 120 m");
  // Behind them and to the side: a turn round.
  sim.retarget(0, static_cast<float>(-40 * kS), static_cast<float>(-70 * kS));
  sim.retarget(1, static_cast<float>(-40 * kS), static_cast<float>(70 * kS));
  run(3000, "back west");
  for (int i = 0; i < 2; ++i) {
    const dyn_boat_books &b = plant.books(i);
    std::printf("  agent %d: drive %.5g J, brake %.4g, rudder %.4g, water %.5g, residual %.3g "
                "(%.2e of the flow)\n",
                i, b.drive, b.brake, b.rudder, b.water, b.residual,
                std::fabs(b.residual) / b.flow());
    EXPECT_NEAR(identity_gap(plant, i), 0.0, 1e-9 * b.flow());
    EXPECT_LT(std::fabs(b.residual) / b.flow(), 1e-3) << "agent " << i;
  }
  sim.set_plant(nullptr);
}

// --- plant_host (Jolt) ---------------------------------------------------------------------

// Gate (5): "dynamicboat" puts every agent on a launch on the host's water, and they drive east;
// "dynamicboat:<presets>" cycles presets; an unknown preset throws and changes nothing; without
// water the boats are refused and the agents are back on the bicycle; "kinematic" takes them off.
TEST(DynBoatPlant, PlantHostPutsTheAgentsOnBoatsByName) {
  EXPECT_TRUE(plant_host::is_dynamic_boat("dynamicboat"));
  EXPECT_TRUE(plant_host::is_dynamic_boat(" dynamicboat:jonboat"));
  EXPECT_FALSE(plant_host::is_dynamic_boat("dynamic"));
  EXPECT_FALSE(plant_host::is_dynamic("dynamicboat"));
  EXPECT_FALSE(plant_host::is_dynamic4w("dynamicboat"));
  EXPECT_NO_THROW(plant_host::check("dynamicboat"));
  EXPECT_NO_THROW(plant_host::check("dynamicboat:jonboat, launch"));
  EXPECT_THROW(plant_host::check("dynamicboat:tug"), cvc::physics_error);
  REQUIRE_JOLT();
  sim_world sim = open_sim(3);
  plant_host host;
  EXPECT_THROW(host.use(sim, "dynamicboat"), cvc::physics_error) << "no water to float them";
  EXPECT_EQ(sim.plant_type(), plant_kind::kinematic);
  EXPECT_EQ(host.driving(), "kinematic");
  EXPECT_TRUE(host.fallback());
  EXPECT_EQ(host.dynamic_boat(), nullptr);

  host.set_water(std::make_shared<field>(water()));
  EXPECT_EQ(host.use(sim, "dynamicboat"), "dynamicboat");
  EXPECT_FALSE(host.fallback());
  EXPECT_EQ(sim.plant_type(), plant_kind::dynamic);
  ASSERT_NE(host.dynamic_boat(), nullptr);
  EXPECT_EQ(host.active(), host.dynamic_boat());
  EXPECT_EQ(host.active()->medium(), plant_medium::water);
  EXPECT_EQ(host.dynamic(), nullptr);
  EXPECT_EQ(host.plant(), nullptr);
  ASSERT_NE(host.physics(), nullptr);
  EXPECT_EQ(host.physics()->boat_count(), 3u);
  EXPECT_EQ(host.dynamic_boat()->size(), 3);
  EXPECT_EQ(host.dynamic_boat()->desc(1).boat.hull.mass, 900.0);
  const float x0 = positions(sim)[2];
  for (int t = 0; t < 300; ++t)
    sim.step();
  const float drove = positions(sim)[2] - x0;
  std::printf("  on launches: agent 1 drove %.1f m east in 18 s\n", drove);
  EXPECT_GT(drove, 20.0f);

  const dyn_boat_plant *before = host.dynamic_boat();
  EXPECT_THROW(host.use(sim, "dynamicboat:tug"), cvc::physics_error);
  EXPECT_EQ(host.dynamic_boat(), before);
  EXPECT_EQ(host.driving(), "dynamicboat");

  EXPECT_EQ(host.use(sim, "dynamicboat:jonboat,launch"), "dynamicboat:jonboat,launch");
  ASSERT_NE(host.dynamic_boat(), nullptr);
  EXPECT_EQ(host.dynamic_boat()->desc(0).boat.hull.mass, 350.0);
  EXPECT_EQ(host.dynamic_boat()->desc(1).boat.hull.mass, 900.0);
  EXPECT_EQ(host.dynamic_boat()->desc(2).boat.hull.mass, 350.0);
  for (int t = 0; t < 10; ++t)
    sim.step();

  EXPECT_EQ(host.use(sim, "kinematic"), "kinematic");
  EXPECT_EQ(sim.plant_type(), plant_kind::kinematic);
  EXPECT_EQ(host.dynamic_boat(), nullptr);
  EXPECT_EQ(host.active(), nullptr);
}

// --- hosting, contacts and determinism (Jolt) ------------------------------------------------

namespace {

// Two boats on the swell under a fixed sequence of commands.
// The bicycle angle a triangle wave of 2.4 s, without libm, so the commands are the same bits on
// every platform; ahead for 10 s, then astern.
void scripted(dyn_boat_plant &p, int t) {
  const int phase = t % 40;
  const double tri = (phase < 20 ? phase : 40 - phase) / 10.0 - 1.0;
  for (int i = 0; i < p.size(); ++i)
    p.command(i, static_cast<float>(i ? -0.2 * tri : 0.2 * tri),
              static_cast<float>(t < 167 ? 0.8 : -0.5));
}

const cvc::nav::plant_state kTwo[2] = {at(0.0f, 0.0f, 0.4f), at(20.0f, 10.0f, -0.3f)};

} // namespace

// A host that steps the world itself (step_world off: before_world_step and after_world_step
// around each of its steps, advance() between ticks) runs the boats bit for bit as the plant's own
// stepping does, and books them alike.
TEST(DynBoatPlant, AHostThatStepsTheWorldRunsItTheSame) {
  REQUIRE_JOLT();
  auto wa = water_world(swell()), wb = water_world(swell());
  dyn_boat_plant_options oa, ob;
  oa.physics = wa.get();
  ob.physics = wb.get();
  ob.step_world = false;
  dyn_boat_plant a({dyn_boat_desc::launch(), dyn_boat_desc::jonboat()}, oa);
  dyn_boat_plant b({dyn_boat_desc::launch(), dyn_boat_desc::jonboat()}, ob);
  a.attach(2, kTwo, si_frame(), 0.06);
  b.attach(2, kTwo, si_frame(), 0.06);
  EXPECT_THROW(a.before_world_step(), cvc::physics_error) << "it steps its own world";
  for (int t = 0; t < 250; ++t) {
    scripted(a, t);
    scripted(b, t);
    a.advance();
    b.advance(); // ends the last tick, begins this one
    for (int k = 0; k < b.substeps(); ++k) {
      b.before_world_step();
      wb->step();
      b.after_world_step();
    }
    ASSERT_EQ(wa->state_hash(), wb->state_hash()) << "tick " << t;
  }
  b.advance(); // ends the last tick
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(a.books(i).drive, b.books(i).drive);
    EXPECT_EQ(a.books(i).water, b.books(i).water);
    EXPECT_EQ(a.books(i).residual, b.books(i).residual);
    EXPECT_EQ(a.books(i).kinetic, b.books(i).kinetic);
  }
}

// A run continued (OW5j): the world saved at a tick with the driver's memory (remember), then
// restored and resumed, steps on exactly as the run did from there. Restored without the memory,
// the driver starts afresh and the run differs.
TEST(DynBoatPlant, ARunRestoredWithItsDriversMemoryStepsOnTheSame) {
  REQUIRE_JOLT();
  auto w = water_world(water());
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({dyn_boat_desc::launch()}, o);
  const cvc::nav::plant_state start = at(0.0f, 0.0f);
  p.attach(1, &start, si_frame(), w->config().fixed_dt); // one world step a tick
  const auto drive = [&](int t) {
    if (t % 12 == 0)
      p.command(0, t < 300 ? 0.2f : -0.1f, t < 240 ? 1.5f : 0.5f);
    p.advance();
  };
  for (int t = 0; t < 200; ++t)
    drive(t);
  const std::vector<std::uint8_t> saved = w->save();
  const dyn_boat_plant::memory mem = p.remember(0);
  EXPECT_NE(mem.trim, 0.0) << "the speed loop has learnt something";
  for (int t = 200; t < 400; ++t)
    drive(t);
  const std::uint64_t run = w->state_hash();
  const cvc::nav::plant_state end = p.state(0);

  w->restore(saved);
  p.resume(0, mem);
  for (int t = 200; t < 400; ++t)
    drive(t);
  EXPECT_EQ(w->state_hash(), run) << "the same steps from there";
  EXPECT_EQ(p.state(0).x, end.x);
  EXPECT_EQ(p.state(0).speed, end.speed);

  w->restore(saved);
  p.resume(0, {});
  for (int t = 200; t < 400; ++t)
    drive(t);
  EXPECT_NE(w->state_hash(), run) << "a fresh driver steers another way";
}

// A hull driven into a pier (a static box) counts the contact as one with static geometry, as
// nav_stats' building contacts; open water counts none.
TEST(DynBoatPlant, AHullThatHitsAPierCountsIt) {
  REQUIRE_JOLT();
  auto w = water_world(water());
  body_desc pier;
  pier.shape = box{{1.0, 10.0, 4.0}};
  pier.motion = motion_type::static_;
  pier.position = {31.0, 0.0, -2.0}; // its west face at x = 30
  w->add_body(pier);
  dyn_boat_plant_options o;
  o.physics = w.get();
  o.driver = false;
  dyn_boat_plant p({dyn_boat_desc::launch()}, o);
  const cvc::nav::plant_state s0 = at(0.0f, 0.0f);
  p.attach(1, &s0, si_frame(), 0.06);
  int before = 0, hits = 0, first = -1;
  for (int t = 0; t < 500; ++t) {
    p.command(0, 0.0f, 50.0f);
    p.advance();
    const double bow = w->state(p.hull(0)).position.x + 2.0;
    (bow < 29.0 ? before : hits) += p.static_contacts_step(0);
    if (first < 0 && p.sample(0).contacts > 0)
      first = t;
  }
  std::printf("  the launch met the pier at tick %d (%.1f s); static contacts %d\n", first,
              0.06 * first, hits);
  EXPECT_EQ(before, 0) << "nothing to hit on open water";
  EXPECT_GT(hits, 0);
  EXPECT_LT(w->state(p.hull(0)).position.x, 29.0) << "the pier stopped it";
}

// The scripted two-boat scene on the swell, pinned. The plant's driver and books are det:: math
// in a fixed order, as the boats' step is, so a cross-platform deterministic Jolt gives it on every
// platform.
TEST(DynBoatPlant, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  auto w = water_world(swell(0.2, 4.0, 0.3));
  w->set_field_time(3.0);
  dyn_boat_plant_options o;
  o.physics = w.get();
  dyn_boat_plant p({dyn_boat_desc::launch(), dyn_boat_desc::jonboat()}, o);
  p.attach(2, kTwo, si_frame(), 0.06);
  for (int t = 0; t < 50; ++t) { // 300 steps
    scripted(p, t);
    p.advance();
  }
  const std::uint64_t h = w->state_hash();
  std::printf("[physics-golden] two boats on the plant after 300 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0x96abfc30fcae4fd4ull) << std::hex << "state hash " << h;
}

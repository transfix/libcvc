/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for the hull model (cvc/physics/hull.h; docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D2,
// O11a): a fluid pushing on a hull's triangles in place of the body's cells. With Jolt:
//   (1) a box hull at density 500 floats at half draft, where the cells float it, however finely
//       its faces are split;
//   (2) a heeled box's righting moment is the wall-sided one; a box tilted 20 degrees rights
//       itself, and a tall thin one tips over;
//   (3) a hull towed at constant speed feels its pressure drag and ITTC-57 friction, growing as
//       the square of the speed;
//   (4) a hull dropped onto still water comes to rest without gaining energy, and a sinking one
//       keeps its speed at depth;
//   (5) on a swell a floating hull heaves with the surface at its centre;
//   (6) runs hash alike, the hull is in the state, the body keeps its own copy of the hull, and
//       the scene's hash is pinned.
// The refusals run on the null backend, everywhere.

#include <algorithm>
#include <chrono>
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

constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

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

std::shared_ptr<cvc::volume> uniform(double value, vec3 lo, vec3 hi) {
  auto v = std::make_shared<cvc::volume>(ctx(), cvc::dimension(2, 2, 2), cvc::Float,
                                         cvc::bounding_box(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z));
  v->fill(value);
  return v;
}

// Still water up to z = 0, `across` metres square and 5 m deep, damped as field.h suggests for
// a metre-sized body.
field water(double across = 10.0, double damping = 1.0e4) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-across / 2, -across / 2, -5}, {across / 2, across / 2, 0});
  f.damping = damping;
  return f;
}

// Water 10 m across from z = -5 up to a box top of z = 1, its top a free surface.
field sea(std::shared_ptr<const fluid_surface> surface) {
  field f = water();
  f.volume = uniform(1000.0, {-5, -5, -5}, {5, 5, 1});
  f.surface = std::move(surface);
  return f;
}

std::shared_ptr<wave_train_surface> swell(double amplitude, double period) {
  wave_train t;
  t.amplitude = amplitude;
  t.period_s = period;
  t.dir_rad = 0.3;
  t.phase = 0.7;
  return std::make_shared<wave_train_surface>(0.0, 0.0, std::vector<wave_train>{t});
}

// A box hull with each face split into n x n squares, two triangles each (12 n^2 triangles).
std::shared_ptr<hull_mesh> split_box_hull(const vec3 &h, int n) {
  auto out = std::make_shared<hull_mesh>();
  const double e[3] = {h.x, h.y, h.z};
  // Each face: its normal's axis and sign, and in-plane axes u, v with u x v the normal.
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

// A 1 x 1 x 0.4 m box at `density`, its centre at height z, with its own box hull or none.
body_desc slab(double density, double z, bool hull = true) {
  body_desc d;
  d.shape = box{{0.5, 0.5, 0.2}};
  d.density = density;
  d.position = {0.0, 0.0, z};
  if (hull)
    d.hull = box_hull({0.5, 0.5, 0.2});
  return d;
}

quat about_x(double deg) {
  const double a = deg * kPi / 180.0;
  return {std::cos(a / 2), std::sin(a / 2), 0.0, 0.0};
}

double tilt(const quat &q) { return std::acos(std::min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))); }

double correlation(const std::vector<double> &a, const std::vector<double> &b, double *ratio) {
  double ma = 0.0, mb = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    ma += a[i] / double(a.size());
    mb += b[i] / double(b.size());
  }
  double saa = 0.0, sbb = 0.0, sab = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    saa += (a[i] - ma) * (a[i] - ma);
    sbb += (b[i] - mb) * (b[i] - mb);
    sab += (a[i] - ma) * (b[i] - mb);
  }
  if (ratio)
    *ratio = std::sqrt(saa / sbb);
  return sab / std::sqrt(saa * sbb);
}

} // namespace

// --- refusals (null backend) -------------------------------------------------

TEST(PhysicsHull, MalformedHullsAreRefused) {
  world_config c;
  c.backend = backend::null;
  world w(c);
  body_desc fixed = slab(500.0, 0.0);
  fixed.motion = motion_type::static_;
  EXPECT_THROW(w.add_body(fixed), cvc::physics_error) << "only a dynamic body has a hull";

  auto inside_out = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  for (auto &t : inside_out->triangles)
    std::swap(t[1], t[2]);
  body_desc d = slab(500.0, 0.0);
  d.hull = inside_out;
  EXPECT_THROW(w.add_body(d), cvc::physics_error) << "wound clockwise";

  auto out_of_range = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  out_of_range->triangles[3][1] = 8;
  d.hull = out_of_range;
  EXPECT_THROW(w.add_body(d), cvc::physics_error) << "an index past the vertices";

  auto not_finite = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  not_finite->vertices[2].z = std::nan("");
  d.hull = not_finite;
  EXPECT_THROW(w.add_body(d), cvc::physics_error) << "a vertex not finite";

  auto viscous = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  viscous->kinematic_viscosity = 0.0;
  d.hull = viscous;
  EXPECT_THROW(w.add_body(d), cvc::physics_error) << "no viscosity";

  EXPECT_EQ(w.body_count(), 0u) << "a refused body is not left behind";
  EXPECT_NO_THROW(w.add_body(slab(500.0, 0.0)));
}

TEST(PhysicsHull, ABoxHullHasTwelveOutwardTriangles) {
  const auto h = box_hull({1.0, 0.5, 0.25});
  ASSERT_EQ(h->vertices.size(), 8u);
  ASSERT_EQ(h->triangles.size(), 12u);
  double volume = 0.0;
  for (const auto &t : h->triangles) {
    const vec3 a = h->vertices[t[0]], b = h->vertices[t[1]], c = h->vertices[t[2]];
    const vec3 ab = b - a, ac = c - a;
    const vec3 n{ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x};
    EXPECT_GT(dot(n, (a + b + c) * (1.0 / 3.0)), 0.0) << "facing outward";
    volume +=
        dot(a, vec3{b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x}) / 6.0;
  }
  EXPECT_NEAR(volume, 2.0 * 1.0 * 0.5, 1e-12);
}

// --- hydrostatics ----------------------------------------------------------------

// The first step's load is Archimedes', at the centre of buoyancy.
TEST(PhysicsHull, BuoyancyIsTheDisplacedWeight) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water(10.0, 0.0));
  const body_id b = w.add_body(slab(500.0, -0.1)); // 0.3 of its 0.4 m under water
  const double draft = 0.0 - (w.state(b).position.z - 0.2);
  w.step();
  const field_load l = w.field_load_on(b);
  EXPECT_NEAR(l.immersed_volume, draft, 1e-12);
  EXPECT_NEAR(l.force.z, 1000.0 * kG * draft, 1e-8);
  EXPECT_NEAR(l.force.x, 0.0, 1e-9);
  EXPECT_NEAR(l.force.y, 0.0, 1e-9);
  EXPECT_NEAR(l.torque.x, 0.0, 1e-9);
  EXPECT_NEAR(l.torque.y, 0.0, 1e-9);
}

// The pressure acts at each triangle's centre of pressure: the first step's torque on a heeled
// box at rest is the wall-sided righting moment rho g V GZ, GZ = sin(theta) (GM + BM/2 tan^2
// theta), exactly while neither its deck nor its bottom edge crosses the waterline. The box's
// centre is on the still water, so its 0.2 m draft and its volume are kept as it heels.
TEST(PhysicsHull, TheRightingMomentIsTheWallSidedOne) {
  REQUIRE_JOLT();
  const double v = 0.2, t = 0.2, kb = t / 2, kg = 0.2, bm = 1.0 / (12.0 * t);
  const double gm = kb + bm - kg;
  for (const double deg : {10.0, 20.0}) {
    for (const int split : {1, 6}) {
      world w(jolt_config());
      w.add_field(water(10.0, 0.0));
      body_desc d = slab(500.0, 0.0);
      if (split > 1)
        d.hull = split_box_hull({0.5, 0.5, 0.2}, split);
      d.rotation = about_x(deg);
      const body_id b = w.add_body(d);
      const double th = tilt(w.state(b).rotation); // as the engine keeps it, in floats
      w.step();
      const field_load l = w.field_load_on(b);
      const double gz = std::sin(th) * (gm + 0.5 * bm * std::tan(th) * std::tan(th));
      const double moment = 1000.0 * kG * v * gz;
      std::printf("[hull] righting moment at %.0f deg, %zu triangles: %.6f N m (wall-sided "
                  "%.6f)\n",
                  deg, static_cast<std::size_t>(12 * split * split), -l.torque.x, moment);
      // The engine's float quaternion is unit to about 1e-8, which the volume shows.
      EXPECT_NEAR(l.immersed_volume, v, 1e-7 * v);
      EXPECT_NEAR(l.force.z, 1000.0 * kG * v, 1e-7 * 1000.0 * kG * v);
      EXPECT_NEAR(l.force.y, 0.0, 1e-7 * 1000.0 * kG * v);
      EXPECT_NEAR(-l.torque.x, moment, 1e-6 * moment) << "restoring, at the centre of pressure";
      EXPECT_NEAR(l.torque.y, 0.0, 1e-9);
      EXPECT_NEAR(l.torque.z, 0.0, 1e-9);
    }
  }
}

// Gate (1): at density 500 a box hull floats at half draft, within 1 %, where the cell model
// floats the same box, and a hull with its faces split 36 ways floats there too.
TEST(PhysicsHull, ABoxHullFloatsAtHalfDraftAsTheCellsDo) {
  REQUIRE_JOLT();
  auto settle = [](body_desc d) {
    world w(jolt_config());
    w.add_field(water());
    const body_id b = w.add_body(d);
    w.step(600);
    return w.state(b);
  };
  const body_state cells = settle(slab(500.0, 0.0, false));
  const body_state hull = settle(slab(500.0, 0.0));
  body_desc fine = slab(500.0, 0.0);
  fine.hull = split_box_hull({0.5, 0.5, 0.2}, 6);
  const body_state split = settle(fine);
  const double draft = 0.0 - (hull.position.z - 0.2);
  std::printf("[hull] draft: 12 triangles %.5f m, 432 triangles %.5f m, cells %.5f m\n", draft,
              0.0 - (split.position.z - 0.2), 0.0 - (cells.position.z - 0.2));
  EXPECT_NEAR(draft, 0.2, 0.002) << "half draft within 1 %";
  EXPECT_NEAR(hull.position.z, cells.position.z, 0.002) << "where the cells float it";
  EXPECT_NEAR(split.position.z, hull.position.z, 1e-4) << "however finely the faces are split";
  EXPECT_LT(std::fabs(hull.linear_velocity.z), 0.01);
  EXPECT_LT(tilt(hull.rotation), 0.01);
}

// Gate (2): a box tilted 20 degrees rights itself, the pressure acting at each triangle's centre
// of pressure. A box twice as tall as it is wide, floating at half its height, has its centre of
// mass above its metacentre and tips over from 5 degrees.
TEST(PhysicsHull, ATiltedBoxRightsItselfAndATallOneTipsOver) {
  REQUIRE_JOLT();
  {
    world w(jolt_config());
    w.add_field(water());
    body_desc d = slab(500.0, 0.0);
    d.rotation = about_x(20.0);
    const body_id b = w.add_body(d);
    w.step(800);
    std::printf("[hull] the slab's tilt after 8 s: %.3f deg\n",
                tilt(w.state(b).rotation) * 180 / kPi);
    EXPECT_LT(tilt(w.state(b).rotation), 5.0 * kPi / 180.0);
  }
  {
    world w(jolt_config());
    w.add_field(water());
    body_desc d;
    d.shape = box{{0.2, 0.2, 0.4}};
    d.hull = box_hull({0.2, 0.2, 0.4});
    d.density = 500.0;
    d.rotation = about_x(5.0);
    const body_id b = w.add_body(d);
    w.step(1000);
    std::printf("[hull] the tall box's tilt after 10 s: %.3f deg\n",
                tilt(w.state(b).rotation) * 180 / kPi);
    EXPECT_GT(tilt(w.state(b).rotation), 60.0 * kPi / 180.0);
  }
}

// --- resistance ------------------------------------------------------------------

// Gate (3): a 2 m hull towed at a constant speed through still water feels a resistance growing
// about as the speed squared: the pressure drag on its bow, (v.n)^2, and the ITTC-57 friction,
// whose coefficient falls slowly with the Reynolds number. The field's damping acts on heave only,
// so it adds nothing here.
TEST(PhysicsHull, TowingResistanceGrowsAsTheSpeedSquared) {
  REQUIRE_JOLT();
  auto tow = [](double v) {
    world w(jolt_config());
    w.add_field(water(40.0));
    body_desc d;
    d.shape = box{{1.0, 0.4, 0.25}};
    d.hull = box_hull({1.0, 0.4, 0.25});
    d.density = 500.0;
    d.position = {-15.0, 0.0, 0.0};
    d.allow_sleeping = false;
    const body_id b = w.add_body(d);
    w.step(300);
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < 150; ++i) {
      body_state s = w.state(b);
      s.linear_velocity.x = v;
      w.set_state(b, s);
      w.step();
      if (i >= 50) {
        sum += w.field_load_on(b).force.x;
        ++n;
      }
    }
    return -sum / n;
  };
  const double slow = tow(1.0), fast = tow(2.0);
  std::printf("[hull] towing resistance: %.2f N at 1 m/s, %.2f N at 2 m/s, ratio %.3f\n", slow,
              fast, fast / slow);
  EXPECT_GT(slow, 0.0) << "it resists";
  EXPECT_NEAR(fast / slow, 4.0, 0.4) << "about as v^2";
  // The bow's pressure drag, 1/2 rho C_d A v^2 over its 0.8 x 0.25 m under water, and the
  // ITTC-57 friction on the 2.6 m^2 of bottom and sides the water slides along.
  auto expected = [](double v) {
    const double lg = std::log10(v * 2.0 / 1.19e-6) - 2.0;
    return 0.5 * 1000.0 * 1.0 * 0.8 * 0.25 * v * v +
           0.5 * 1000.0 * (0.075 / (lg * lg)) * 2.6 * v * v;
  };
  std::printf("[hull] expected: %.2f N at 1 m/s, %.2f N at 2 m/s\n", expected(1.0), expected(2.0));
  EXPECT_NEAR(slow, expected(1.0), 0.03 * expected(1.0)) << "the damping does not slow surge";
  // The bow's drag, below the centre of mass, trims the hull bow down by about F 0.125 m over
  // rho g I_wp, 0.01 rad at 2 m/s, and its bow then meets about 4 % more water than at rest.
  EXPECT_NEAR(fast, expected(2.0), 0.06 * expected(2.0));
}

// --- stability ---------------------------------------------------------------------

// Gate (4): a hull dropped from a metre onto still water slams, bobs and comes to rest at half
// draft, and its energy (kinetic, gravity's and the water's, rho g A d^2 / 2 at draft d) never
// rises above what it was dropped with.
TEST(PhysicsHull, ADroppedHullComesToRestWithoutGainingEnergy) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  body_desc d = slab(500.0, 1.2);
  d.allow_sleeping = false;
  const body_id b = w.add_body(d);
  const double m = 200.0;
  auto energy = [&](const body_state &s) {
    const double draft = std::min(std::max(0.2 - s.position.z, 0.0), 0.4);
    const vec3 v = s.linear_velocity, o = s.angular_velocity;
    const double inertia = m * (1.0 + 0.16) / 12.0;
    return m * kG * s.position.z + 1000.0 * kG * draft * draft / 2.0 + 0.5 * m * dot(v, v) +
           0.5 * inertia * dot(o, o);
  };
  const double e0 = energy(w.state(b)), rest = 1000.0 * kG * 0.04 / 2.0;
  double highest = -1e300, fastest_end = 0.0;
  for (int i = 0; i < 900; ++i) {
    w.step();
    const body_state s = w.state(b);
    highest = std::max(highest, energy(s));
    if (i >= 800)
      fastest_end = std::max(fastest_end, std::sqrt(dot(s.linear_velocity, s.linear_velocity)));
  }
  const body_state s = w.state(b);
  std::printf("[hull] dropped from 1 m: %.1f J above rest at the drop, at most %.1f J after, "
              "%.3f J after 9 s\n",
              e0 - rest, highest - rest, energy(s) - rest);
  EXPECT_LE(highest, e0 + 0.01 * (e0 - rest)) << "no energy gained";
  EXPECT_LT(fastest_end, 0.01) << "at rest";
  EXPECT_NEAR(0.0 - (s.position.z - 0.2), 0.2, 0.002) << "at half draft";
}

// A hull denser than water sinks at a speed that does not depend on how deep it is: the field's
// damping sums to c V_sub, as the cells', and its top face, under water, adds nothing. The speed
// is where the excess weight meets the damping and the bottom's pressure drag,
// (rho_b - rho) V g = c V v + 1/2 rho C_d A v^2, as for the cells; both come out about 4 % faster,
// since the drag's implicit scale, (1 - e^-x)/x at x = 0.09 here, applies it a little short.
TEST(PhysicsHull, ASinkingHullKeepsItsSpeedAtDepth) {
  REQUIRE_JOLT();
  auto sink = [](bool hull, double &shallow, double &deep) {
    world w(jolt_config());
    w.add_field(water());
    body_desc d = slab(1200.0, -0.5, hull);
    d.allow_sleeping = false;
    const body_id b = w.add_body(d);
    shallow = deep = 0.0;
    for (int i = 0; i < 4000 && w.state(b).position.z > -3.5; ++i) {
      w.step();
      const body_state s = w.state(b);
      if (shallow == 0.0 && s.position.z < -1.5)
        shallow = -s.linear_velocity.z;
    }
    deep = -w.state(b).linear_velocity.z;
  };
  double hs = 0.0, hd = 0.0, cs = 0.0, cd = 0.0;
  sink(true, hs, hd);
  sink(false, cs, cd);
  const double c = 1.0e4, v = 0.4, excess = 200.0 * v * kG, a = 0.5 * 1000.0 * 1.0 * 1.0;
  const double expect = (-c * v + std::sqrt(c * v * c * v + 4.0 * a * excess)) / (2.0 * a);
  std::printf("[hull] sinking at 1.5 m and 3.5 m: hull %.4f, %.4f m/s; cells %.4f, %.4f m/s; "
              "expected %.4f m/s\n",
              hs, hd, cs, cd, expect);
  EXPECT_NEAR(hd, hs, 0.02 * hs) << "as fast deep as shallow";
  EXPECT_NEAR(hs, expect, 0.1 * expect);
  EXPECT_NEAR(hd, cd, 0.1 * cd) << "about as the cells sink";
}

// A light hull flung forward, then one spun, through water thick with drag: the drag never
// carries it past the water's velocity, however light it is. Its first step, where an explicit
// one would reverse it threefold, slows it without reversing it. Braking the bow below the centre
// of mass pitches the slab, and as it stops the pitching rocks it a few cm/s to and fro in surge;
// a spin slows smoothly. The hydrostatic push is explicit, as the cells' buoyancy is, and stable
// while the heave's sqrt(rho g A / m) times the step stays under 2: this 8 kg slab's is 0.35.
TEST(PhysicsHull, ALightHullNeverOvershootsTheWater) {
  REQUIRE_JOLT();
  // The slab floats 8 mm deep: its bow's drag rate times the step over its mass starts near 4,
  // where an explicit step would throw it backwards.
  auto run = [](const vec3 &v, const vec3 &spin, double &one, double &least, double &rise,
                double &end) {
    world w(jolt_config());
    field thick = water(40.0, 0.0);
    thick.drag_coefficient = 100.0;
    w.add_field(thick);
    body_desc d = slab(20.0, 0.2 - 0.4 * 20.0 / 1000.0);
    d.position.x = -15.0;
    d.linear_velocity = v;
    d.angular_velocity = spin;
    d.allow_sleeping = false;
    const body_id b = w.add_body(d);
    // The surge or the yaw rate, whichever was given.
    auto rate = [&](const body_state &s) {
      return v.x != 0.0 ? s.linear_velocity.x : s.angular_velocity.z;
    };
    double last = rate(w.state(b));
    least = last;
    rise = 0.0;
    for (int i = 0; i < 300; ++i) {
      w.step();
      const double r = rate(w.state(b));
      if (i == 0)
        one = r;
      least = std::min(least, r);
      rise = std::max(rise, r - last);
      last = r;
    }
    end = last;
  };
  double one = 0.0, least = 0.0, rise = 0.0, end = 0.0;
  run({8.0, 0.0, 0.0}, {}, one, least, rise, end);
  std::printf("[hull] a light hull in thick water: surge 8 m/s, %.4f after a step, %.4f after 3 s, "
              "least %.4f, largest rise in a step %.2e\n",
              one, end, least, rise);
  EXPECT_GT(one, 0.0) << "the first step does not reverse it";
  EXPECT_LT(one, 8.0) << "but slows it";
  EXPECT_GE(least, -0.08) << "rocking, not overshooting: within 1 % of the start";
  EXPECT_LE(rise, 0.08);
  EXPECT_LT(std::fabs(end), 0.08) << "it stops";
  run({}, {0.0, 0.0, 6.0}, one, least, rise, end);
  std::printf("[hull] spun: yaw 6 rad/s, %.4f after a step, %.4f after 3 s, least %.4f, largest "
              "rise in a step %.2e\n",
              one, end, least, rise);
  EXPECT_GT(one, 0.0);
  EXPECT_GE(least, 0.0) << "the yaw never reverses";
  EXPECT_LE(rise, 1e-6) << "nor grows";
  EXPECT_LT(end, 6.0) << "it slows";
}

// --- waves -----------------------------------------------------------------------------

// Gate (5): on a 0.2 m, 4 s swell a floating hull heaves with the surface at its centre.
TEST(PhysicsHull, AHullHeavesWithTheSwell) {
  REQUIRE_JOLT();
  const auto s = swell(0.2, 4.0);
  world w(jolt_config());
  w.add_field(sea(s));
  const body_id b = w.add_body(slab(500.0, 0.0));
  w.step(400);
  std::vector<double> z, eta;
  for (int i = 0; i < 1600; ++i) {
    w.step();
    const body_state st = w.state(b);
    z.push_back(st.position.z);
    eta.push_back(s->height(st.position.x, st.position.y, w.field_time()));
  }
  double ratio = 0.0;
  const double c = correlation(z, eta, &ratio);
  std::printf("[hull] heave on the swell: correlation %.4f, height ratio %.3f\n", c, ratio);
  EXPECT_GT(c, 0.9) << "in step with the surface at its centre";
  EXPECT_NEAR(ratio, 1.0, 0.2) << "about as high";
}

// --- state ---------------------------------------------------------------------------

namespace {

std::unique_ptr<world> swell_scene(std::shared_ptr<const hull_mesh> hull) {
  auto w = std::make_unique<world>(jolt_config());
  w->add_field(sea(swell(0.2, 4.0)));
  body_desc d = slab(500.0, 0.0, false);
  d.hull = std::move(hull);
  w->add_body(d);
  body_desc ball;
  ball.shape = sphere{0.25};
  ball.density = 300.0;
  ball.position = {1.5, -1.0, 0.5};
  w->add_body(ball);
  w->set_field_time(3.0);
  return w;
}

} // namespace

// Gate (6): a hull keeps a run a function of its inputs, and is in the state: its digest and its
// triangles' wetted areas are saved and hashed, and restore refuses a world whose hulls differ.
TEST(PhysicsHull, TheHullIsInTheStateAndItsHash) {
  REQUIRE_JOLT();
  auto a = swell_scene(box_hull({0.5, 0.5, 0.2})), b = swell_scene(box_hull({0.5, 0.5, 0.2}));
  a->step(250);
  b->step(250);
  EXPECT_EQ(a->state_hash(), b->state_hash());
  auto cells = swell_scene(nullptr);
  cells->step(250);
  EXPECT_NE(a->state_hash(), cells->state_hash()) << "the hull pushes otherwise than the cells";
  auto slammier = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  slammier->slam_coefficient = 0.6;
  EXPECT_NE(swell_scene(box_hull({0.5, 0.5, 0.2}))->state_hash(),
            swell_scene(slammier)->state_hash())
      << "the hull's description is in the hash";
  const std::vector<std::uint8_t> saved = a->save();
  auto c = swell_scene(box_hull({0.5, 0.5, 0.2}));
  c->restore(saved);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  a->step(100);
  c->step(100);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  EXPECT_THROW(swell_scene(slammier)->restore(saved), cvc::physics_error);
  EXPECT_THROW(swell_scene(nullptr)->restore(saved), cvc::physics_error) << "no hull";
  EXPECT_THROW(swell_scene(box_hull({0.5, 0.5, 0.2}))->restore(cells->save()), cvc::physics_error)
      << "a state without the hull";
}

// add_body keeps its own copy of the hull: a caller changing its mesh afterwards, even adding
// triangles, changes neither the run nor the hash.
TEST(PhysicsHull, TheBodyKeepsItsOwnCopyOfTheHull) {
  REQUIRE_JOLT();
  auto mine = std::make_shared<hull_mesh>(*box_hull({0.5, 0.5, 0.2}));
  auto a = swell_scene(mine), b = swell_scene(box_hull({0.5, 0.5, 0.2}));
  for (auto &p : mine->vertices)
    p = p * 2.0;
  mine->triangles.insert(mine->triangles.end(), mine->triangles.begin(), mine->triangles.end());
  mine->slam_coefficient = 2.0;
  a->step(100);
  b->step(100);
  EXPECT_EQ(a->state_hash(), b->state_hash());
}

// The swell scene with a hull, pinned. The hull's step is det:: math in a fixed order, so a
// cross-platform deterministic Jolt gives it on every platform.
TEST(PhysicsHull, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  auto w = swell_scene(box_hull({0.5, 0.5, 0.2}));
  w->step(300);
  const std::uint64_t h = w->state_hash();
  std::printf("[physics-golden] hull swell scene after 300 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0x2f71a1fe229afa75ull) << std::hex << "state hash " << h;
}

// --- cost --------------------------------------------------------------------------------

// What a hull costs per step on the swell, beside the same box on its cells: printed, with a
// loose bound so a pathological slowdown fails.
TEST(PhysicsHull, CostPerStep) {
  REQUIRE_JOLT();
  auto time = [](std::shared_ptr<const hull_mesh> hull) {
    auto w = swell_scene(std::move(hull));
    w->step(60);
    const auto t0 = std::chrono::steady_clock::now();
    w->step(600);
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / 600.0;
  };
  const double cells = time(nullptr);
  const double twelve = time(box_hull({0.5, 0.5, 0.2}));
  const auto fine = split_box_hull({0.5, 0.5, 0.2}, 6);
  const double many = time(fine);
  std::printf("[hull] ms per step on the swell: cells %.4f, 12 triangles %.4f, %zu triangles "
              "%.4f\n",
              cells, twelve, fine->triangles.size(), many);
  EXPECT_LT(many, 5.0);
}

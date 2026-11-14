/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for the shared immersion code (cvc/physics/immersion.h; docs/roadmap/
// OCEAN-AND-WEATHER-ROADMAP.md B5, C2, O9): a body's columns of immersed depth zeta over a grid
// window, which the wake windows take as their sources.
//   (1) a box at a known pose on still water gives the analytic column depths: square, turned
//       about z, and tilted;
//   (2) a sphere's columns are its chords; a convex hull of a box's corners and the box's hull
//       mesh (split finely, with sample points on its edges) give the box's columns;
//   (3) with Jolt: the volume of zeta is the immersed volume world::field_load_on reports for the
//       same body, pose and water: exactly for a level box, to the cells' resolution tilted, for
//       a sphere, a barrel and a hull, on still water and on a swell;
//   (4) the columns are the same bits on every platform (a pinned digest);
//   (5) the refusals.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/physics/immersion.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace cvc::physics;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

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

// Water 12 m square from z = -5 up to z = 0 (a flat top), or to a free surface under z = 1.
field water(std::shared_ptr<const fluid_surface> surface = nullptr) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-6, -6, -5}, {6, 6, surface ? 1.0 : 0.0});
  f.damping = 1.0e4;
  f.surface = std::move(surface);
  return f;
}

std::shared_ptr<wave_train_surface> swell() {
  wave_train t;
  t.amplitude = 0.2;
  t.period_s = 4.0;
  t.dir_rad = 0.3;
  t.phase = 0.7;
  return std::make_shared<wave_train_surface>(0.0, 0.0, std::vector<wave_train>{t});
}

// A rotation of angle a (rad) about the unit axis (x, y, z).
quat about(double a, double x, double y, double z) {
  const double s = std::sin(0.5 * a);
  return {std::cos(0.5 * a), x * s, y * s, z * s};
}
quat times(const quat &a, const quat &b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// A box hull with each face split into n x n squares, two triangles each (physics_hull_test's).
hull_mesh split_box_hull(const vec3 &h, int n) {
  hull_mesh out;
  const double e[3] = {h.x, h.y, h.z};
  const int faces[6][4] = {{0, +1, 1, 2}, {0, -1, 2, 1}, {1, +1, 2, 0},
                           {1, -1, 0, 2}, {2, +1, 0, 1}, {2, -1, 1, 0}};
  for (const auto &f : faces) {
    const int a = f[0], u = f[2], v = f[3];
    const std::uint32_t base = static_cast<std::uint32_t>(out.vertices.size());
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i) {
        double p[3];
        p[a] = f[1] * e[a];
        p[u] = e[u] * (-1.0 + 2.0 * i / n);
        p[v] = e[v] * (-1.0 + 2.0 * j / n);
        out.vertices.push_back({p[0], p[1], p[2]});
      }
    const auto at = [&](int i, int j) {
      return base + static_cast<std::uint32_t>(j * (n + 1) + i);
    };
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        out.triangles.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
        out.triangles.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
      }
  }
  return out;
}

// lsystem_coast's barrel: a cylinder along x of half length hl and radius r, 20 sides, as points.
std::vector<vec3> barrel_points(double hl, double r) {
  std::vector<vec3> pts;
  for (int k = 0; k < 20; ++k) {
    const double a = 2.0 * kPi * k / 20;
    for (int e = -1; e <= 1; e += 2)
      pts.push_back({e * hl, r * std::cos(a), r * std::sin(a)});
  }
  return pts;
}

// The overlap of [a0, a1] and [b0, b1].
double overlap(double a0, double a1, double b0, double b1) {
  return std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
}

std::uint64_t fnv(const std::vector<double> &v, std::uint64_t h = 1469598103934665603ull) {
  for (double x : v) {
    unsigned char b[sizeof x];
    std::memcpy(b, &x, sizeof x);
    for (unsigned char c : b) {
      h ^= c;
      h *= 1099511628211ull;
    }
  }
  return h;
}

} // namespace

// A level box 1.2 x 0.9 x 0.8 m, its bottom 0.25 m under still water at z = 0: each cell's zeta
// is the draft times the part of the cell's square inside the footprint, exactly, and the volume
// is the footprint times the draft.
TEST(PhysicsImmersion, ALevelBoxOnStillWaterHasTheAnalyticColumns) {
  const immersion_body box(shape{cvc::physics::box{{0.6, 0.45, 0.4}}});
  const vec3 c{0.13, -0.21, 0.15}; // bottom at -0.25
  immersion_water still;
  still.hi.z = 0.0;
  immersion_grid g;
  g.nx = g.ny = 24;
  g.dx = 0.25;
  g.origin_x = g.origin_y = -3.0;
  for (int samples : {1, 4, 16}) {
    g.samples = samples;
    std::vector<double> zeta(g.count(), 0.0);
    const double v = add_immersion(box, c, quat{}, still, g, zeta);
    double worst = 0.0;
    for (int j = 0; j < g.ny; ++j)
      for (int i = 0; i < g.nx; ++i) {
        const double x = g.origin_x + i * g.dx, y = g.origin_y + j * g.dx;
        double want = 0.0;
        if (samples == 1) {
          want = std::fabs(x - c.x) <= 0.6 && std::fabs(y - c.y) <= 0.45 ? 0.25 : 0.0;
        } else {
          // The sub-columns sample the square evenly: the exact share of them in the footprint.
          int in = 0;
          for (int b = 0; b < samples; ++b)
            for (int a = 0; a < samples; ++a) {
              const double sx = x + ((a + 0.5) / samples - 0.5) * g.dx,
                           sy = y + ((b + 0.5) / samples - 0.5) * g.dx;
              in += std::fabs(sx - c.x) <= 0.6 && std::fabs(sy - c.y) <= 0.45 ? 1 : 0;
            }
          want = 0.25 * in / double(samples * samples);
        }
        worst = std::max(worst, std::fabs(zeta[std::size_t(j) * g.nx + i] - want));
      }
    EXPECT_LT(worst, 1e-15) << samples << " sub-columns a side";
    // The area-weighted volume converges to the footprint times the draft (0.27 m^3).
    double area = 0.0;
    for (int j = 0; j < g.ny; ++j)
      for (int i = 0; i < g.nx; ++i) {
        const double x = g.origin_x + i * g.dx, y = g.origin_y + j * g.dx;
        area += overlap(x - 0.125, x + 0.125, c.x - 0.6, c.x + 0.6) *
                overlap(y - 0.125, y + 0.125, c.y - 0.45, c.y + 0.45);
      }
    std::printf("level box, %2d sub-columns a side: volume %.6f m^3 (exact 0.270000)\n", samples,
                v);
    if (samples == 16)
      EXPECT_NEAR(v, 0.27, 0.27 * 0.01);
    EXPECT_NEAR(area * 0.25, 0.27, 1e-12);
  }
}

// Turned about z, a column through the footprint is the draft and one outside it is 0; tilted,
// the column through the centre is the box's height over cos(tilt), cut at the water.
TEST(PhysicsImmersion, ATurnedAndATiltedBoxHaveTheirColumns) {
  const immersion_body box(shape{cvc::physics::box{{0.6, 0.45, 0.4}}});
  const vec3 c{0.0, 0.0, 0.1};
  const quat yaw = about(0.7, 0.0, 0.0, 1.0);
  const double cy = std::cos(0.7), sy = std::sin(0.7);
  for (double u : {-0.55, -0.3, 0.0, 0.4, 0.58, 0.62})
    for (double v : {-0.44, 0.0, 0.2, 0.46}) {
      const double x = c.x + cy * u - sy * v, y = c.y + sy * u + cy * v;
      const bool in = std::fabs(u) <= 0.6 && std::fabs(v) <= 0.45;
      EXPECT_NEAR(box.column(c, yaw, x, y, 0.0), in ? 0.3 : 0.0, 1e-12) << u << ", " << v;
      // Above the water, all of it.
      EXPECT_NEAR(box.column(c, yaw, x, y, 10.0), in ? 0.8 : 0.0, 1e-12);
    }
  // Tilted 25 degrees about x: the vertical through the centre leaves through the top and bottom
  // faces (0.45 > 0.4 tan 25), 0.8 / cos 25 long, centred on the box's centre.
  const double t = 25.0 * kPi / 180.0;
  const quat tilt = about(t, 1.0, 0.0, 0.0);
  const double len = 0.8 / std::cos(t);
  EXPECT_NEAR(box.column(c, tilt, 0.0, 0.0, 10.0), len, 1e-12);
  EXPECT_NEAR(box.column(c, tilt, 0.0, 0.0, 0.0), 0.5 * len - 0.1, 1e-12);
  EXPECT_NEAR(box.column(c, tilt, 0.0, 0.0, -1.0), 0.0, 0.0);
}

// A sphere's columns are its chords. The convex hull of a box's corners (and points inside it)
// and the box's hull mesh, split into 4 x 4 squares a face and sampled on their edges and
// corners, give the box's columns at a turned and tilted pose.
TEST(PhysicsImmersion, SphereHullAndMeshColumns) {
  const immersion_body ball(shape{sphere{0.5}});
  const vec3 c{1.0, 2.0, -0.1};
  for (double r : {0.0, 0.2, 0.45, 0.49, 0.5, 0.6}) {
    const double chord = r < 0.5 ? 2.0 * std::sqrt(0.25 - r * r) : 0.0;
    EXPECT_NEAR(ball.column(c, quat{}, c.x + r, c.y, 10.0), chord, 1e-12);
    EXPECT_NEAR(ball.column(c, quat{}, c.x, c.y - r, -0.1), 0.5 * chord, 1e-12);
  }

  const vec3 h{1.0, 0.5, 0.25};
  std::vector<vec3> corners;
  for (int k = 0; k < 8; ++k)
    corners.push_back({(k & 1 ? 1 : -1) * h.x, (k & 2 ? 1 : -1) * h.y, (k & 4 ? 1 : -1) * h.z});
  corners.push_back({0.1, 0.2, 0.0}); // inside: not a face
  corners.push_back({-0.5, 0.0, 0.1});
  const immersion_body hull(shape{convex_hull{corners}});
  EXPECT_EQ(hull.planes().size(), 6u);
  EXPECT_EQ(immersion_body(shape{convex_hull{barrel_points(0.5, 0.378)}}).planes().size(), 22u);
  const immersion_body box(shape{cvc::physics::box{h}});
  const immersion_body mesh(split_box_hull(h, 4));
  const immersion_body coarse(*box_hull(h));

  const quat pose = times(about(0.4, 0.0, 0.0, 1.0), about(0.3, 0.6, 0.8, 0.0));
  const vec3 at{0.2, -0.1, 0.05};
  double worst_hull = 0.0, worst_mesh = 0.0;
  int crossed = 0;
  for (int j = -60; j <= 60; ++j)
    for (int i = -60; i <= 60; ++i) {
      const double x = at.x + i * 0.02, y = at.y + j * 0.02;
      for (double top : {0.0, 0.1, 10.0}) {
        const double want = box.column(at, pose, x, y, top);
        crossed += want > 0.0 ? 1 : 0;
        worst_hull = std::max(worst_hull, std::fabs(hull.column(at, pose, x, y, top) - want));
        worst_mesh = std::max(worst_mesh, std::fabs(mesh.column(at, pose, x, y, top) - want));
        worst_mesh = std::max(worst_mesh, std::fabs(coarse.column(at, pose, x, y, top) - want));
      }
    }
  EXPECT_GT(crossed, 1000);
  EXPECT_LT(worst_hull, 1e-9);
  EXPECT_LT(worst_mesh, 1e-9);

  // Level, on a grid that puts sample points on the split faces' shared edges and corners
  // (multiples of 0.25 m), and on the outline's own edges: each point counts once. On the
  // outline the mesh is half-open, as a rasterizer is (the top-left rule): a point on its left or
  // top edge is inside and one on its right or bottom edge outside, where the box's slabs are
  // closed. A cell's sub-columns never fall exactly there in practice; what matters is that the
  // faces above and below decide alike, so a column is all or nothing, never half a crossing.
  for (int j = -8; j <= 8; ++j)
    for (int i = -8; i <= 8; ++i) {
      const double x = i * 0.125, y = j * 0.125;
      const bool open_edge = x == h.x || y == -h.y;
      const double want = open_edge ? 0.0 : box.column({}, quat{}, x, y, 0.1);
      EXPECT_NEAR(mesh.column({}, quat{}, x, y, 0.1), want, 1e-12) << x << ", " << y;
      EXPECT_NEAR(coarse.column({}, quat{}, x, y, 0.1), want, 1e-12) << x << ", " << y;
    }
}

// Unbounded water (a default immersion_water: no surface, no box) covers the whole body, so a
// hull mesh gives the same full columns as the box it outlines, not a dry body from inf - inf.
TEST(PhysicsImmersion, AMeshInUnboundedWaterIsWhollyUnder) {
  const vec3 h{0.6, 0.45, 0.4};
  const immersion_body box(shape{cvc::physics::box{h}});
  const immersion_body mesh(*box_hull(h));
  const immersion_body split(split_box_hull(h, 3));
  const quat pose = times(about(0.4, 0.0, 0.0, 1.0), about(0.3, 0.6, 0.8, 0.0));
  const vec3 at{0.3, -0.2, 0.1};
  const double inf = std::numeric_limits<double>::infinity();
  int crossed = 0;
  for (int j = -20; j <= 20; ++j)
    for (int i = -20; i <= 20; ++i) {
      const double x = at.x + i * 0.05 + 0.003, y = at.y + j * 0.05 + 0.007;
      const double want = box.column(at, pose, x, y, inf);
      crossed += want > 0.0 ? 1 : 0;
      EXPECT_NEAR(mesh.column(at, pose, x, y, inf), want, 1e-9) << x << ", " << y;
      EXPECT_NEAR(split.column(at, pose, x, y, inf), want, 1e-9) << x << ", " << y;
    }
  EXPECT_GT(crossed, 300);

  const immersion_water open; // no surface, unbounded
  immersion_grid g;
  g.nx = g.ny = 40;
  g.dx = 0.05;
  g.origin_x = at.x - 1.0;
  g.origin_y = at.y - 1.0;
  std::vector<double> zb(g.count(), 0.0), zm(g.count(), 0.0);
  const double vb = add_immersion(box, at, pose, open, g, zb);
  const double vm = add_immersion(mesh, at, pose, open, g, zm);
  EXPECT_NEAR(vb, 8.0 * h.x * h.y * h.z, 0.02 * 8.0 * h.x * h.y * h.z); // the box's volume
  EXPECT_NEAR(vm, vb, 1e-9);
}

// The volume of zeta is the immersed volume the field step finds for the same body, at the pose
// and field time of the step (field_load_on reports the step's load, computed before it moves).
TEST(PhysicsImmersion, TheVolumeIsTheFieldsImmersedVolume) {
  REQUIRE_JOLT();
  struct body_case {
    const char *name;
    body_desc desc;
    double tolerance; // relative
  };
  std::vector<body_case> cases;
  {
    body_desc d;
    d.shape = cvc::physics::box{{0.6, 0.4, 0.3}};
    d.position = {0.0, 0.0, 0.1}; // bottom 0.2 m under
    d.density = 500.0;
    cases.push_back({"level box", d, 1e-9});
    d.rotation = times(about(0.35, 1.0, 0.0, 0.0), about(0.2, 0.0, 1.0, 0.0));
    d.position = {1.0, -0.5, 0.05};
    cases.push_back({"tilted box", d, 0.03});
    body_desc s;
    s.shape = sphere{0.4};
    s.position = {-1.5, 1.0, 0.1};
    s.density = 500.0;
    cases.push_back({"sphere", s, 0.05});
    body_desc b;
    b.shape = convex_hull{barrel_points(0.5, 0.378)};
    b.position = {2.0, 2.0, 0.05};
    b.rotation = about(0.6, 0.0, 0.0, 1.0);
    b.density = 200.0;
    cases.push_back({"barrel", b, 0.05});
    body_desc h;
    h.shape = cvc::physics::box{{0.6, 0.4, 0.3}};
    h.hull = box_hull({0.6, 0.4, 0.3});
    h.position = {-2.0, -2.0, 0.08};
    h.rotation = times(about(0.3, 0.0, 1.0, 0.0), about(0.5, 0.0, 0.0, 1.0));
    h.density = 500.0;
    cases.push_back({"tilted hull", h, 0.01});
  }
  for (bool wavy : {false, true}) {
    world_config cfg;
    cfg.backend = backend::jolt;
    cfg.gravity = {0.0, 0.0, -9.80665};
    world w(cfg);
    const field f = wavy ? water(swell()) : water();
    w.add_field(f);
    if (wavy)
      w.set_field_time(1.3);
    std::vector<body_id> ids;
    for (const body_case &bc : cases)
      ids.push_back(w.add_body(bc.desc));
    std::vector<body_state> before;
    for (body_id id : ids)
      before.push_back(w.state(id));
    const double t = w.field_time();
    w.step();
    const immersion_water sea = immersion_water::of(f, t);
    immersion_grid g;
    g.nx = g.ny = 240;
    g.dx = 0.05;
    g.origin_x = g.origin_y = -6.0;
    g.samples = 4;
    for (std::size_t k = 0; k < cases.size(); ++k) {
      if (wavy && cases[k].tolerance < 1e-6)
        cases[k].tolerance = 0.01; // the cells read the surface at their centres, a column at its
                                   // cell's point
      std::vector<double> zeta(g.count(), 0.0);
      const double v = add_immersion(immersion_body::of(cases[k].desc), before[k].position,
                                     before[k].rotation, sea, g, zeta);
      const double field_v = w.field_load_on(ids[k]).immersed_volume;
      std::printf("%s, %s: zeta's volume %.6f m^3, the field's %.6f m^3 (%+.3f %%)\n",
                  wavy ? "swell" : "still", cases[k].name, v, field_v,
                  100.0 * (v - field_v) / field_v);
      EXPECT_GT(field_v, 0.0);
      EXPECT_NEAR(v, field_v, cases[k].tolerance * field_v) << cases[k].name;
    }
  }
}

// The columns of four bodies (a turned box, a sphere, a barrel's hull and a split hull mesh) on a
// swell, over a 64 x 64 window, hash to the same digest on every platform: IEEE arithmetic and
// sqrt only, the swell's det:: math, and no FMA contraction (here and in the library). Measured
// on Windows (MSVC); CI checks Linux and macOS.
TEST(PhysicsImmersion, IsTheSameOnEveryPlatform) {
  const auto sea = swell();
  immersion_water w;
  w.surface = sea.get();
  w.time = 2.25;
  w.hi.z = 1.0;
  immersion_grid g;
  g.nx = g.ny = 64;
  g.dx = 0.125;
  g.origin_x = -4.0;
  g.origin_y = -3.875;
  g.samples = 3;
  auto scene = [&](std::vector<double> &zeta) {
    zeta.assign(g.count(), 0.0);
    double v = 0.0;
    // Rotations from exact unit quaternions (squares summing to 1), so nothing libm computes
    // enters.
    v += add_immersion(immersion_body(shape{cvc::physics::box{{0.6, 0.45, 0.4}}}),
                       {-2.0, -1.5, 0.1}, quat{0.5, 0.5, 0.5, 0.5}, w, g, zeta);
    v += add_immersion(immersion_body(shape{sphere{0.45}}), {1.5, -1.75, 0.05}, quat{}, w, g, zeta);
    v += add_immersion(immersion_body(shape{convex_hull{{{-0.5, -0.36, -0.36},
                                                         {0.5, -0.36, -0.36},
                                                         {-0.5, 0.36, -0.36},
                                                         {0.5, 0.36, -0.36},
                                                         {-0.5, -0.36, 0.36},
                                                         {0.5, -0.36, 0.36},
                                                         {-0.5, 0.36, 0.36},
                                                         {0.5, 0.36, 0.36},
                                                         {0.0, 0.0, 0.5}}}}),
                       {-1.0, 1.5, 0.0}, quat{0.6, 0.0, 0.0, 0.8}, w, g, zeta);
    v += add_immersion(immersion_body(split_box_hull({1.0, 0.5, 0.25}, 3)), {1.5, 1.5, 0.05},
                       quat{0.8, 0.0, 0.6, 0.0}, w, g, zeta);
    return v;
  };
  std::vector<double> zeta, again;
  const double v = scene(zeta);
  scene(again);
  const std::uint64_t d = fnv(zeta);
  std::printf("immersion digest 0x%016llx, volume %.17g m^3\n", static_cast<unsigned long long>(d),
              v);
  EXPECT_EQ(fnv(again), d);
  EXPECT_GT(v, 0.5);
  EXPECT_EQ(d, 0xb582404bf028ed63ull) << "measured on Windows (MSVC)";
}

TEST(PhysicsImmersion, RefusesBadInput) {
  EXPECT_THROW(immersion_body(shape{mesh{}}), std::invalid_argument);
  EXPECT_THROW(immersion_body(shape{terrain{}}), std::invalid_argument);
  EXPECT_THROW(immersion_body(shape{cvc::physics::box{{0.0, 1.0, 1.0}}}), std::invalid_argument);
  EXPECT_THROW(immersion_body(shape{sphere{-1.0}}), std::invalid_argument);
  EXPECT_THROW(immersion_body(shape{convex_hull{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}}}),
               std::invalid_argument);
  EXPECT_THROW(immersion_body(shape{convex_hull{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}}}}),
               std::invalid_argument); // flat
  hull_mesh bad = *box_hull({1, 1, 1});
  bad.triangles[3][1] = 99;
  EXPECT_THROW(immersion_body{bad}, std::invalid_argument);
  const immersion_body box(shape{cvc::physics::box{{1, 1, 1}}});
  immersion_grid g;
  g.nx = g.ny = 4;
  std::vector<double> zeta(16, 0.0), wrong(15, 0.0);
  const immersion_water w;
  EXPECT_THROW(add_immersion(box, {}, quat{}, w, g, wrong), std::invalid_argument);
  g.samples = 0;
  EXPECT_THROW(add_immersion(box, {}, quat{}, w, g, zeta), std::invalid_argument);
  g.samples = 4;
  g.dx = 0.0;
  EXPECT_THROW(add_immersion(box, {}, quat{}, w, g, zeta), std::invalid_argument);
  g.dx = 0.5;
  EXPECT_THROW(add_immersion(box, {std::nan(""), 0, 0}, quat{}, w, g, zeta), std::invalid_argument);
  EXPECT_NO_THROW(add_immersion(box, {}, quat{}, w, g, zeta));
}

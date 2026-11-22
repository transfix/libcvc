/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc/volume/class_surfaces.h, the surfaces of a label volume's classes (VolumeGridRover's
// Segmentation tab's 3D view, docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P9), against the
// §1.7 rows that apply:
//   - T3, known shapes: one voxel is an octahedron; a block, its chamfered box; a ball, near its
//     volume once smoothed; two touching blocks meet on the same vertices;
//   - T4, invariants over seeded random labels: every surface closed, each edge in two triangles
//     that run it opposite ways, wound to face out (a positive enclosed volume), no degenerate
//     triangles, every vertex inside the volume's box grown by half a voxel;
//   - T5: a voxel that is no label, bad parameters; odd inputs as warnings;
//   - T6: two apps, two threads, the same result; T7: the stepper as the blocking call, bit for
//     bit, at every budget, with its phases and a bounded step; T8: the parameter table.
// Runs natively and under node (the ports lane, T14).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/class_surfaces.h>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr int none = 255;

// A UChar label volume of n^3 voxels, all `none`, with the box [0, n-1]^3 (unit spacing).
cvc::volume labels(cvc::app &a, int nx, int ny, int nz) {
  cvc::volume v(a, cvc::dimension(nx, ny, nz), cvc::UChar,
                cvc::bounding_box(0, 0, 0, nx - 1, ny - 1, nz - 1));
  std::fill(v.data_ptr(), v.data_ptr() + std::size_t(nx) * ny * nz,
            static_cast<unsigned char>(none));
  return v;
}

void set(cvc::volume &v, int i, int j, int k, int label) {
  v(static_cast<cvc::uint64>(i), static_cast<cvc::uint64>(j), static_cast<cvc::uint64>(k),
    double(label));
}

// The volume a closed, outward surface encloses (the divergence theorem).
double enclosed(const cvc::geometry &g) {
  double s = 0.0;
  for (const auto &t : g.tris()) {
    const auto &a = g.points()[t[0]], &b = g.points()[t[1]], &c = g.points()[t[2]];
    s += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
         a[2] * (b[0] * c[1] - b[1] * c[0]);
  }
  return s / 6.0;
}

// Closed and consistently wound: every directed edge once, and its reverse once.
::testing::AssertionResult closed_and_oriented(const cvc::geometry &g) {
  std::map<std::pair<std::uint64_t, std::uint64_t>, int> directed;
  for (const auto &t : g.tris()) {
    if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2])
      return ::testing::AssertionFailure() << "a triangle repeats a vertex";
    for (int e = 0; e < 3; ++e)
      ++directed[{t[e], t[(e + 1) % 3]}];
  }
  for (const auto &kv : directed) {
    if (kv.second != 1)
      return ::testing::AssertionFailure() << "edge " << kv.first.first << "-" << kv.first.second
                                           << " runs " << kv.second << " times the same way";
    const auto back = directed.find({kv.first.second, kv.first.first});
    if (back == directed.end())
      return ::testing::AssertionFailure()
             << "edge " << kv.first.first << "-" << kv.first.second << " has no reverse: open";
  }
  return ::testing::AssertionSuccess();
}

bool same_geometry(const cvc::geometry &a, const cvc::geometry &b) {
  return a.points() == b.points() && a.tris() == b.tris() && a.normals() == b.normals();
}

bool same(const cvc::class_surface_result &a, const cvc::class_surface_result &b) {
  if (a.labels != b.labels || a.voxels != b.voxels || a.warnings != b.warnings ||
      a.surfaces.size() != b.surfaces.size())
    return false;
  for (std::size_t i = 0; i < a.surfaces.size(); ++i)
    if (!same_geometry(a.surfaces[i], b.surfaces[i]))
      return false;
  return true;
}

// Seeded random labels: `classes` grown from random centres, each voxel taking its nearest centre's
// label within `radius` (else none), so classes touch each other and the volume's faces.
cvc::volume random_labels(cvc::app &a, int n, int classes, double radius, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, double(n - 1));
  std::vector<std::array<double, 3>> c(classes);
  for (auto &p : c)
    p = {u(rng), u(rng), u(rng)};
  cvc::volume v = labels(a, n, n, n);
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double best = radius * radius;
        int lab = none;
        for (int q = 0; q < classes; ++q) {
          const double d = (i - c[q][0]) * (i - c[q][0]) + (j - c[q][1]) * (j - c[q][1]) +
                           (k - c[q][2]) * (k - c[q][2]);
          if (d < best) {
            best = d;
            lab = q;
          }
        }
        set(v, i, j, k, lab);
      }
  return v;
}

// ---- T3: known shapes -------------------------------------------------------------------------

TEST(ClassSurfaces, OneVoxelIsAnOctahedronFacingOut) {
  cvc::app a;
  cvc::volume v = labels(a, 5, 5, 5);
  set(v, 2, 3, 1, 7);
  cvc::class_surface_params p;
  p.smoothing = 0;
  const cvc::class_surface_result r = cvc::class_surfaces(v, p);
  ASSERT_EQ(r.labels, std::vector<int>{7});
  ASSERT_EQ(r.voxels, std::vector<std::size_t>{1});
  const cvc::geometry &g = r.surfaces[0];
  EXPECT_EQ(g.points().size(), 6u);
  EXPECT_EQ(g.tris().size(), 8u);
  EXPECT_TRUE(closed_and_oriented(g));
  EXPECT_NEAR(enclosed(g), 1.0 / 6.0, 1e-12) << "an octahedron with half-voxel radius";
  for (std::size_t q = 0; q < g.points().size(); ++q) {
    const auto &pt = g.points()[q];
    const double d[3] = {pt[0] - 2, pt[1] - 3, pt[2] - 1};
    EXPECT_NEAR(std::fabs(d[0]) + std::fabs(d[1]) + std::fabs(d[2]), 0.5, 1e-12);
    // Normals point away from the voxel.
    const auto &nn = g.normals()[q];
    EXPECT_GT(nn[0] * d[0] + nn[1] * d[1] + nn[2] * d[2], 0.0);
    EXPECT_NEAR(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2], 1.0, 1e-12);
  }
  EXPECT_EQ(r.surface_of(7), &r.surfaces[0]);
  EXPECT_EQ(r.surface_of(3), nullptr);
  EXPECT_EQ(r.triangle_count(), 8u);
  EXPECT_TRUE(r.warnings.empty());
}

TEST(ClassSurfaces, ABlockIsItsChamferedBoxInObjectSpace) {
  cvc::app a;
  // 10 x 8 x 6 voxels over a box with spacings 0.5, 1 and 2, the block 4 x 3 x 2 voxels.
  cvc::volume v(a, cvc::dimension(10, 8, 6), cvc::UChar, cvc::bounding_box(1, 2, 3, 5.5, 9, 13));
  std::fill(v.data_ptr(), v.data_ptr() + 480, static_cast<unsigned char>(none));
  for (int k = 2; k < 4; ++k)
    for (int j = 3; j < 6; ++j)
      for (int i = 4; i < 8; ++i)
        set(v, i, j, k, 0);
  cvc::class_surface_params p;
  p.smoothing = 0;
  const cvc::class_surface_result r = cvc::class_surfaces(v, p);
  ASSERT_EQ(r.labels, std::vector<int>{0});
  const cvc::geometry &g = r.surfaces[0];
  EXPECT_TRUE(closed_and_oriented(g));
  // Between the voxel centres' box (3 x 2 x 1 voxels) and the box half a voxel out (4 x 3 x 2),
  // in object units (voxel 0.5 x 1 x 2).
  const double unit = 0.5 * 1.0 * 2.0;
  EXPECT_GT(enclosed(g), 3 * 2 * 1 * unit);
  EXPECT_LT(enclosed(g), 4 * 3 * 2 * unit);
  // Every vertex within the block's voxels' box grown by half a voxel, in object space.
  for (const auto &pt : g.points()) {
    EXPECT_GE(pt[0], 1 + 0.5 * 3.5 - 1e-12);
    EXPECT_LE(pt[0], 1 + 0.5 * 7.5 + 1e-12);
    EXPECT_GE(pt[1], 2 + 1.0 * 2.5 - 1e-12);
    EXPECT_LE(pt[1], 2 + 1.0 * 5.5 + 1e-12);
    EXPECT_GE(pt[2], 3 + 2.0 * 1.5 - 1e-12);
    EXPECT_LE(pt[2], 3 + 2.0 * 3.5 + 1e-12);
  }
}

TEST(ClassSurfaces, SmoothingKeepsABallsVolume) {
  cvc::app a;
  const int n = 28;
  const double c = 13.5, rad = 9.0;
  cvc::volume v = labels(a, n, n, n);
  std::size_t inside = 0;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        if ((i - c) * (i - c) + (j - c) * (j - c) + (k - c) * (k - c) <= rad * rad) {
          set(v, i, j, k, 2);
          ++inside;
        }
  cvc::class_surface_params rough;
  rough.smoothing = 0;
  const cvc::class_surface_result r0 = cvc::class_surfaces(v, rough);
  const cvc::class_surface_result r1 = cvc::class_surfaces(v); // 10 passes
  ASSERT_EQ(r0.labels, std::vector<int>{2});
  ASSERT_EQ(r1.labels, std::vector<int>{2});
  EXPECT_EQ(r1.voxels[0], inside);
  const double ball = 4.0 / 3.0 * 3.14159265358979 * rad * rad * rad;
  const double v0 = enclosed(r0.surfaces[0]), v1 = enclosed(r1.surfaces[0]);
  std::cout << "ball r=9: " << inside << " voxels, surfaces enclose " << v0 << " (steps) and " << v1
            << " (smoothed), the ball " << ball << "\n";
  EXPECT_NEAR(v1, v0, 0.03 * v0) << "Taubin's smoothing keeps the volume";
  EXPECT_NEAR(v1, ball, 0.06 * ball);
  EXPECT_TRUE(closed_and_oriented(r1.surfaces[0]));
  // Smoothed, every vertex lies near the sphere (the steps are gone).
  double worst = 0.0;
  for (const auto &pt : r1.surfaces[0].points()) {
    const double d = std::sqrt((pt[0] - c) * (pt[0] - c) + (pt[1] - c) * (pt[1] - c) +
                               (pt[2] - c) * (pt[2] - c));
    worst = std::max(worst, std::fabs(d - rad));
  }
  EXPECT_LT(worst, 0.75);
}

TEST(ClassSurfaces, TouchingClassesMeetOnTheSameVertices) {
  cvc::app a;
  cvc::volume v = labels(a, 8, 6, 6);
  for (int k = 1; k < 5; ++k)
    for (int j = 1; j < 5; ++j)
      for (int i = 1; i < 7; ++i)
        set(v, i, j, k, i < 4 ? 0 : 1);
  cvc::class_surface_params p;
  p.smoothing = 0;
  const cvc::class_surface_result r = cvc::class_surfaces(v, p);
  ASSERT_EQ(r.labels, (std::vector<int>{0, 1}));
  EXPECT_TRUE(closed_and_oriented(r.surfaces[0]));
  EXPECT_TRUE(closed_and_oriented(r.surfaces[1]));
  // The interface, x = 3.5: both surfaces have the same vertices there.
  const auto at_interface = [](const cvc::geometry &g) {
    std::vector<cvc::geometry::point_t> out;
    for (const auto &pt : g.points())
      if (std::fabs(pt[0] - 3.5) < 1e-12)
        out.push_back(pt);
    std::sort(out.begin(), out.end());
    return out;
  };
  const auto i0 = at_interface(r.surfaces[0]), i1 = at_interface(r.surfaces[1]);
  EXPECT_FALSE(i0.empty());
  EXPECT_EQ(i0, i1);
}

TEST(ClassSurfaces, OpenAtTheFacesWithoutClose) {
  cvc::app a;
  cvc::volume v = labels(a, 6, 6, 6);
  for (int k = 0; k < 6; ++k)
    for (int j = 0; j < 6; ++j)
      for (int i = 0; i < 3; ++i)
        set(v, i, j, k, 4); // the half x < 3, touching five faces
  cvc::class_surface_params p;
  p.smoothing = 0;
  p.close = false;
  const cvc::class_surface_result open = cvc::class_surfaces(v, p);
  ASSERT_EQ(open.labels, std::vector<int>{4});
  EXPECT_FALSE(closed_and_oriented(open.surfaces[0])) << "a plane across the volume, open";
  for (const auto &pt : open.surfaces[0].points())
    EXPECT_NEAR(pt[0], 2.5, 1e-12);
  p.close = true;
  EXPECT_TRUE(closed_and_oriented(cvc::class_surfaces(v, p).surfaces[0]));

  // A label filling the volume has no surface without close: a warning says so.
  cvc::volume full = labels(a, 3, 3, 3);
  std::fill(full.data_ptr(), full.data_ptr() + 27, static_cast<unsigned char>(1));
  p.close = false;
  const cvc::class_surface_result f = cvc::class_surfaces(full, p);
  EXPECT_TRUE(f.labels.empty());
  ASSERT_EQ(f.warnings.size(), 1u);
  EXPECT_NE(f.warnings[0].find("fills the volume"), std::string::npos);
}

TEST(ClassSurfaces, OnlyTheLabelsAskedFor) {
  cvc::app a;
  const cvc::volume v = random_labels(a, 16, 4, 6.0, 3);
  cvc::class_surface_params p;
  p.labels = {2, 0, 9};
  const cvc::class_surface_result r = cvc::class_surfaces(v, p);
  EXPECT_EQ(r.labels, (std::vector<int>{0, 2}));
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("label 9 is not in the volume"), std::string::npos);
  const cvc::class_surface_result all = cvc::class_surfaces(v);
  EXPECT_EQ(all.labels, (std::vector<int>{0, 1, 2, 3}));
  EXPECT_TRUE(same_geometry(*r.surface_of(2), *all.surface_of(2)))
      << "a label's surface does not depend on the others asked for";
  // The background is a label like any other once it is not the background.
  p.labels.clear();
  p.background = 0;
  EXPECT_EQ(cvc::class_surfaces(v, p).labels, (std::vector<int>{1, 2, 3, 255}));
}

TEST(ClassSurfaces, NoClassesNoSurfaces) {
  cvc::app a;
  const cvc::class_surface_result r = cvc::class_surfaces(labels(a, 4, 4, 4));
  EXPECT_TRUE(r.labels.empty());
  EXPECT_TRUE(r.surfaces.empty());
  EXPECT_TRUE(r.warnings.empty());
  // One voxel thick: still closed, with close.
  cvc::volume flat = labels(a, 5, 5, 1);
  set(flat, 2, 2, 0, 0);
  const cvc::class_surface_result f = cvc::class_surfaces(flat);
  ASSERT_EQ(f.labels, std::vector<int>{0});
  EXPECT_TRUE(closed_and_oriented(f.surfaces[0]));
}

// ---- T4: invariants -----------------------------------------------------------------------------

TEST(ClassSurfaces, RandomLabelsGiveClosedOutwardSurfaces) {
  cvc::app a;
  for (std::uint32_t seed = 1; seed <= 6; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    const int n = 14 + int(seed);
    const cvc::volume v = random_labels(a, n, 2 + int(seed % 5), 3.0 + seed, seed);
    for (const int smoothing : {0, 6}) {
      cvc::class_surface_params p;
      p.smoothing = smoothing;
      const cvc::class_surface_result r = cvc::class_surfaces(v, p);
      ASSERT_FALSE(r.labels.empty());
      std::size_t voxels = 0;
      for (std::size_t q = 0; q < r.labels.size(); ++q) {
        const cvc::geometry &g = r.surfaces[q];
        EXPECT_TRUE(closed_and_oriented(g)) << "label " << r.labels[q];
        EXPECT_GT(enclosed(g), 0.0) << "label " << r.labels[q] << " faces out";
        EXPECT_EQ(g.normals().size(), g.points().size());
        for (const auto &pt : g.points())
          for (int d = 0; d < 3; ++d) {
            EXPECT_GE(pt[d], -0.5 - 1e-9);
            EXPECT_LE(pt[d], n - 0.5 + 1e-9);
          }
        voxels += r.voxels[q];
      }
      std::size_t count = 0;
      for (std::size_t i = 0; i < std::size_t(n) * n * n; ++i)
        count += v.data_ptr()[i] != none;
      EXPECT_EQ(voxels, count);
    }
  }
}

// ---- T5: bad input ------------------------------------------------------------------------------

TEST(ClassSurfaces, BadInputThrowsWithAUsefulMessage) {
  cvc::app a;
  cvc::volume f(a, cvc::dimension(4, 4, 4), cvc::Float);
  f(1, 2, 3, 2.5);
  try {
    (void)cvc::class_surfaces(f);
    FAIL() << "no throw";
  } catch (const cvc::class_surfaces_error &e) {
    EXPECT_NE(std::string(e.what()).find("voxel (1, 2, 3) holds 2.5"), std::string::npos)
        << e.what();
  }
  f(1, 2, 3, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW((void)cvc::class_surfaces(f), cvc::class_surfaces_error);
  f(1, 2, 3, 1e12);
  EXPECT_THROW((void)cvc::class_surfaces(f), cvc::class_surfaces_error);

  cvc::class_surface_params p;
  p.mu = -0.2; // not below -lambda
  try {
    (void)cvc::class_surfaces(labels(a, 3, 3, 3), p);
    FAIL() << "no throw";
  } catch (const cvc::class_surfaces_error &e) {
    EXPECT_NE(std::string(e.what()).find("mu"), std::string::npos) << e.what();
  }
  p = {};
  p.smoothing = -1;
  EXPECT_THROW((void)cvc::class_surfaces(labels(a, 3, 3, 3), p), cvc::class_surfaces_error);
  p.smoothing = 0;
  p.mu = -0.2; // no smoothing: mu is not used
  EXPECT_NO_THROW((void)cvc::class_surfaces(labels(a, 3, 3, 3), p));
}

TEST(ClassSurfaces, EveryIntegerVoxelTypeGivesTheSameSurfaces) {
  cvc::app a;
  const cvc::volume v = random_labels(a, 12, 3, 5.0, 11);
  const cvc::class_surface_result ref = cvc::class_surfaces(v);
  for (cvc::data_type t : {cvc::UShort, cvc::UInt, cvc::Float, cvc::Double, cvc::Int, cvc::Int64}) {
    cvc::volume w(a, cvc::dimension(12, 12, 12), t, v.boundingBox());
    for (std::size_t i = 0; i < 12 * 12 * 12; ++i)
      w(static_cast<cvc::uint64>(i), double(v.data_ptr()[i]));
    EXPECT_TRUE(same(cvc::class_surfaces(w), ref)) << cvc::data_type_strings[t];
  }
}

// ---- T6: no singletons --------------------------------------------------------------------------

TEST(ClassSurfaces, TwoAppsAndTwoThreadsShareNothing) {
  cvc::app a1, a2;
  const cvc::volume v1 = random_labels(a1, 18, 4, 6.0, 21);
  const cvc::volume v2 = random_labels(a2, 15, 3, 5.0, 22);
  cvc::class_surface_params p1, p2;
  p2.smoothing = 3;
  const cvc::class_surface_result alone1 = cvc::class_surfaces(v1, p1);
  const cvc::class_surface_result alone2 = cvc::class_surfaces(v2, p2);
  cvc::class_surface_result t1, t2;
  std::thread th1([&] { t1 = cvc::class_surfaces(v1, p1); });
  std::thread th2([&] { t2 = cvc::class_surfaces(v2, p2); });
  th1.join();
  th2.join();
  EXPECT_TRUE(same(t1, alone1));
  EXPECT_TRUE(same(t2, alone2));
  // Order: B then A equals A then B.
  EXPECT_TRUE(same(cvc::class_surfaces(v2, p2), alone2));
  EXPECT_TRUE(same(cvc::class_surfaces(v1, p1), alone1));
}

// ---- T7: resumable ------------------------------------------------------------------------------

TEST(ClassSurfaces, StepperIsResumable) {
  cvc::app a;
  const cvc::volume v = random_labels(a, 16, 3, 6.0, 5);
  cvc::class_surface_params p;
  p.smoothing = 3;
  cvc::test::t7_subject<cvc::class_surfaces_stepper, cvc::class_surface_result> s;
  s.name = "class surfaces 16^3, 3 labels";
  s.make = [&] { return std::make_unique<cvc::class_surfaces_stepper>(v, p); };
  s.take = [](cvc::class_surfaces_stepper &st) { return st.take(); };
  s.equal = same;
  s.blocking = [&] { return cvc::class_surfaces(v, p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "reading", "extracting", "smoothing", "normals"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST(ClassSurfaces, AStepDoesABoundedAmountOfWork) {
  cvc::app a;
  std::size_t last = 0;
  for (const int n : {10, 20}) {
    cvc::class_surfaces_stepper st(random_labels(a, n, 3, n / 3.0, 9));
    std::size_t steps = 0;
    while (!st.step(64))
      ++steps;
    std::cout << n << "^3: " << steps << " steps of 64 units\n";
    EXPECT_GT(steps, std::size_t(n) * n * n * 2 / 64) << "at least two units a voxel";
    EXPECT_GT(steps, last);
    last = steps;
  }
}

TEST(ClassSurfaces, ACancelledJobGivesNothing) {
  cvc::app a;
  auto job = cvc::class_surfaces_job(random_labels(a, 16, 3, 5.0, 2));
  job->step(100);
  EXPECT_FALSE(job->done());
  job->cancel();
  EXPECT_TRUE(job->step(100));
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  cvc::volume f(a, cvc::dimension(3, 3, 3), cvc::Float);
  f(0, 0, 0, 0.5);
  auto bad = cvc::class_surfaces_job(f);
  EXPECT_TRUE(bad->step(0));
  EXPECT_TRUE(bad->failed());
  EXPECT_NE(bad->error().find("not a label"), std::string::npos);
  EXPECT_THROW((void)bad->result(), cvc::class_surfaces_error);
}

// ---- T8: the parameter table
// ----------------------------------------------------------------------

TEST(ClassSurfaces, ParameterTable) {
  cvc::app a;
  cvc::state &s = cvc::state::instance(a)("surf");
  const auto read = cvc::params_from_state<cvc::class_surface_params>(s);
  EXPECT_TRUE(read.ok()) << read.message();
  EXPECT_EQ(s("smoothing").value(), "10");
  EXPECT_EQ(s("close").value(), "1");
  EXPECT_EQ(s("background").value(), "255");
  cvc::class_surface_params p;
  p.smoothing = 4;
  p.labels = {1, 3};
  p.close = false;
  cvc::params_to_state(p, s);
  const auto back = cvc::params_from_state<cvc::class_surface_params>(s);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.smoothing, 4);
  EXPECT_EQ(back.value.labels, (std::vector<int>{1, 3}));
  EXPECT_FALSE(back.value.close);
  s("smoothing").value("lots");
  EXPECT_FALSE(cvc::params_from_state<cvc::class_surface_params>(s).ok());
  cvc::params_describe_state<cvc::class_surface_params>(s);
  EXPECT_EQ(s("smoothing").property("max"), "1000");
  EXPECT_FALSE(s("lambda").property("doc").empty());
}

} // namespace

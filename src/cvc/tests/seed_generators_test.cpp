/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The seed generators from VolUtils (cvc/volume/seed_generators.h, transfix/libcvc#123; the
// ports plan's P6): capsid_critical_points (criticalPntsToSeed) and rotated_seeds (SeedProduce,
// SeedProduceRegardingAxis). No legacy goldens: VolUtils left volrover's master before the pinned
// commit, and needs the whole VolMagick to build, so the tests check what the legacy code computes
// by its definition (T2-T4), bad input (T5), threads (T6), the stepper (T7) and the tables (T8).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/seed_generators.h>
#include <cvc/volume/seed_generators_params.h>
#include <cvc/volume/segmentation.h>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

cvc::volume blobs(cvc::app &a, int n, const std::vector<std::array<double, 5>> &b, double noise,
                  unsigned seed = 1) {
  cvc::volume vol(a, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(0, 0, 0, n - 1, n - 1, n - 1));
  std::uint32_t s = seed;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double v = 0.02;
        for (const auto &bl : b) {
          const double d2 =
              (i - bl[0]) * (i - bl[0]) + (j - bl[1]) * (j - bl[1]) + (k - bl[2]) * (k - bl[2]);
          v += bl[4] * std::exp(-d2 / (2.0 * bl[3] * bl[3]));
        }
        s = 1664525u * s + 1013904223u;
        v += noise * (double(s) / 4294967296.0 - 0.5);
        vol(i, j, k, std::max(v, 0.0));
      }
  return vol;
}

std::vector<std::array<int, 3>> points_of(const cvc::point_class &c) {
  std::vector<std::array<int, 3>> out;
  for (std::size_t p = 0; p + 2 < c.points.size(); p += 3)
    out.push_back({c.points[p], c.points[p + 1], c.points[p + 2]});
  return out;
}

// ---- capsid_critical_points -----------------------------------------------------------------

TEST(SeedGenerators, CapsidMaximaAreTheBrightLocalMaxima) {
  // A small volume has many fewer maxima than 3000 T, so the threshold falls to t_low + 20: the
  // maxima class is every local maximum of the rescaled volume above it.
  cvc::app a;
  const cvc::volume vol =
      blobs(a, 20, {{6, 6, 6, 2.0, 1.0}, {14, 13, 12, 2.5, 0.8}, {5, 15, 14, 1.5, 0.5}}, 0.05, 3);
  cvc::capsid_seed_params p;
  p.t_low = 30;
  const cvc::point_classes r = cvc::capsid_critical_points(vol, p);
  ASSERT_EQ(r.classes.size(), 2u);
  EXPECT_EQ(r.classes[0].name, "maxima");
  EXPECT_EQ(r.classes[1].name, "saddles");
  // By the definition, on the rescaled volume.
  const int n = 20;
  std::vector<double> d(std::size_t(n) * n * n);
  double mn = 1e300, mx = -1e300;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double v = float(vol(i, j, k));
        d[(std::size_t(k) * n + j) * n + i] = v;
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
  for (double &v : d)
    v = float(((v - float(mn)) / (double(float(mx)) - float(mn))) * 255.0);
  std::vector<std::array<int, 3>> want;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double v = d[(std::size_t(k) * n + j) * n + i];
        if (!(v > 30 && v > 50))
          continue;
        bool top = true;
        for (int z = std::max(k - 1, 0); z <= std::min(k + 1, n - 1); ++z)
          for (int y = std::max(j - 1, 0); y <= std::min(j + 1, n - 1); ++y)
            for (int x = std::max(i - 1, 0); x <= std::min(i + 1, n - 1); ++x)
              if (v < d[(std::size_t(z) * n + y) * n + x])
                top = false;
        if (top)
          want.push_back({i, j, k});
      }
  EXPECT_EQ(points_of(r.classes[0]), want);
  EXPECT_FALSE(want.empty());
  // The two brighter blobs' peaks are among them (to a voxel: the noise may move a peak).
  for (const std::array<int, 3> c : {std::array<int, 3>{6, 6, 6}, {14, 13, 12}}) {
    bool hit = false;
    for (const auto &q : want)
      hit |= std::abs(q[0] - c[0]) <= 1 && std::abs(q[1] - c[1]) <= 1 && std::abs(q[2] - c[2]) <= 1;
    EXPECT_TRUE(hit) << c[0] << " " << c[1] << " " << c[2];
  }
  // Few maxima, so the gradient's maxima ("saddles") come too, about as many.
  EXPECT_FALSE(r.classes[1].points.empty());
  std::cout << r.classes[0].size() << " maxima, " << r.classes[1].size() << " saddles\n";
}

TEST(SeedGenerators, CapsidThresholdKeepsAbout3000TMaxima) {
  // Noise on 48^3 (a flat blob lifting it above 0) gives about 48^3 / 27 = 4096 maxima; with
  // T = 1 (h 1, k 0) the threshold keeps a little over 3000 of them, and with so many the saddles
  // are skipped.
  cvc::app a;
  const cvc::volume vol = blobs(a, 48, {{24, 24, 24, 1000.0, 0.6}}, 1.0, 9);
  cvc::capsid_seed_params p;
  p.h = 1;
  p.k = 0;
  p.t_low = 0;
  const cvc::point_classes r = cvc::capsid_critical_points(vol, p);
  EXPECT_GE(r.classes[0].size(), 3000u);
  EXPECT_LT(r.classes[0].size(), 4500u);
  EXPECT_TRUE(r.classes[1].points.empty()) << "more than 600 T maxima: no saddles";
  // A larger T keeps more of them.
  p.h = 2;
  p.k = 1;
  EXPECT_GT(cvc::capsid_critical_points(vol, p).classes[0].size(), r.classes[0].size());
}

TEST(SeedGenerators, CapsidDataUnitThreshold) {
  cvc::app a;
  const cvc::volume vol = blobs(a, 16, {{8, 8, 8, 3.0, 1.0}}, 0.1, 5);
  cvc::capsid_seed_params p;
  p.t_low = 40;
  const cvc::point_classes norm = cvc::capsid_critical_points(vol, p);
  double mn = 1e300, mx = -1e300;
  for (std::size_t v = 0; v < vol.XDim() * vol.YDim() * vol.ZDim(); ++v) {
    mn = std::min(mn, double(float(vol(v))));
    mx = std::max(mx, double(float(vol(v))));
  }
  p.normalized_thresholds = false;
  p.t_low = mn + 40.0 / 255.0 * (mx - mn);
  EXPECT_TRUE(cvc::capsid_critical_points(vol, p) == norm);
}

TEST(SeedGenerators, CapsidBadInput) {
  cvc::app a;
  cvc::volume flat(a, cvc::dimension(6, 6, 6), cvc::Float);
  for (std::size_t v = 0; v < 216; ++v)
    flat(v, 2.0);
  try {
    cvc::capsid_critical_points(flat);
    ADD_FAILURE() << "no exception";
  } catch (const cvc::segmentation_error &e) {
    EXPECT_NE(std::string(e.what()).find("constant"), std::string::npos) << e.what();
  }
  cvc::capsid_seed_params p;
  p.h = 0;
  p.k = 0;
  try {
    cvc::capsid_critical_points(blobs(a, 8, {{4, 4, 4, 2, 1}}, 0.0), p);
    ADD_FAILURE() << "no exception";
  } catch (const cvc::segmentation_error &e) {
    EXPECT_NE(std::string(e.what()).find("h, k"), std::string::npos) << e.what();
  }
}

TEST(SeedGenerators, CapsidStepperIsResumable) {
  cvc::app a;
  const cvc::volume vol = blobs(a, 20, {{6, 6, 6, 2.0, 1.0}, {14, 13, 12, 2.5, 0.8}}, 0.1, 4);
  cvc::test::t7_subject<cvc::capsid_critical_points_stepper, cvc::point_classes> s;
  s.name = "capsid_critical_points 20^3";
  s.make = [&] { return std::make_unique<cvc::capsid_critical_points_stepper>(vol); };
  s.take = [](cvc::capsid_critical_points_stepper &st) { return st.take(); };
  s.equal = [](const cvc::point_classes &x, const cvc::point_classes &y) { return x == y; };
  s.blocking = [&] { return cvc::capsid_critical_points(vol); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "normalising", "maxima", "saddles"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
}

// ---- rotated_seeds --------------------------------------------------------------------------

TEST(SeedGenerators, RotatedAboutTheVolumesAxis) {
  // SeedProduce: about z through the volume's centre. On a 21^3 unit grid the centre is voxel
  // (10, 10, z); a quarter turn takes (14, 10, 3) to (10, 14, 3), and so on round.
  cvc::app a;
  const cvc::volume vol(a, cvc::dimension(21, 21, 21), cvc::UChar,
                        cvc::bounding_box(0, 0, 0, 20, 20, 20));
  cvc::point_classes seeds;
  seeds.add("s", 14, 10, 3);
  seeds.add("s", 10, 12, 7);
  cvc::rotation_seed_params p;
  p.folds = 4;
  const cvc::point_classes r = cvc::rotated_seeds(seeds, vol, p);
  ASSERT_EQ(r.classes.size(), 4u);
  EXPECT_EQ(r.classes[0].name, "Class 0");
  EXPECT_EQ(r.classes[3].name, "Class 3");
  using P = std::vector<std::array<int, 3>>;
  EXPECT_EQ(points_of(r.classes[0]), (P{{14, 10, 3}, {10, 12, 7}}));
  EXPECT_EQ(points_of(r.classes[1]), (P{{10, 14, 3}, {8, 10, 7}}));
  EXPECT_EQ(points_of(r.classes[2]), (P{{6, 10, 3}, {10, 8, 7}}));
  EXPECT_EQ(points_of(r.classes[3]), (P{{10, 6, 3}, {12, 10, 7}}));
  // The legacy colours: yellow, red, green, magenta.
  EXPECT_EQ(r.classes[1].color, (std::vector<double>{1.0, 0.0, 0.0}));
}

TEST(SeedGenerators, RotatedFromACutAndAboutAnyAxis) {
  cvc::app a;
  const cvc::volume vol(a, cvc::dimension(21, 21, 21), cvc::UChar,
                        cvc::bounding_box(-10, -10, -10, 10, 10, 10));
  // Seeds marked on a cut covering x, y, z in [0, 5] with half-voxel spacing: (4, 2, 6) there is
  // world (2, 1, 3), voxel (12, 11, 13) of the whole.
  cvc::point_classes seeds;
  seeds.add("s", 4, 2, 6);
  cvc::rotation_seed_params p;
  p.folds = 2;
  p.axis = {1.0, 0.0, 0.0};
  p.center = {0.0, 0.0, 0.0};
  const cvc::point_classes r = cvc::rotated_seeds(seeds, cvc::dimension(11, 11, 11),
                                                  cvc::bounding_box(0, 0, 0, 5, 5, 5), vol, p);
  using P = std::vector<std::array<int, 3>>;
  EXPECT_EQ(points_of(r.classes[0]), (P{{12, 11, 13}}));
  EXPECT_EQ(points_of(r.classes[1]), (P{{12, 9, 7}})) << "a half turn about x: (2, -1, -3)";
  // A rotation that lands outside the volume is dropped.
  cvc::point_classes distant;
  distant.add("s", 20, 10, 10); // world (10, 0, 0)
  p.axis = {0.0, 0.0, 1.0};
  p.center = {5.0, 0.0, 0.0};
  p.folds = 2;
  const cvc::point_classes d = cvc::rotated_seeds(distant, vol, p);
  EXPECT_EQ(d.classes[0].size(), 1u);
  EXPECT_EQ(d.classes[1].size(), 1u) << "(0, 0, 0) is inside";
  p.center = {10.0, 0.0, 0.0};
  distant = cvc::point_classes{};
  distant.add("s", 0, 10, 10); // world (-10, 0, 0): its half turn about x = 10 is (30, 0, 0)
  EXPECT_EQ(cvc::rotated_seeds(distant, vol, p).classes[1].size(), 0u);
}

TEST(SeedGenerators, RotatedBadInput) {
  cvc::app a;
  const cvc::volume vol(a, cvc::dimension(5, 5, 5), cvc::UChar);
  cvc::point_classes seeds;
  seeds.add("s", 1, 1, 1);
  cvc::rotation_seed_params p;
  p.folds = 0;
  EXPECT_THROW(cvc::rotated_seeds(seeds, vol, p), cvc::segmentation_error);
  p = {};
  p.axis = {0, 0, 0};
  EXPECT_THROW(cvc::rotated_seeds(seeds, vol, p), cvc::segmentation_error);
  p = {};
  p.center = {1, 2};
  EXPECT_THROW(cvc::rotated_seeds(seeds, vol, p), cvc::segmentation_error);
}

// ---- T6, T8 ---------------------------------------------------------------------------------

TEST(SeedGenerators, CallsAreIndependent) {
  cvc::app a;
  const cvc::volume va = blobs(a, 18, {{9, 9, 9, 3.0, 1.0}}, 0.2, 2);
  const cvc::volume vb = blobs(a, 16, {{4, 4, 4, 2.0, 1.0}, {11, 11, 11, 2.0, 0.7}}, 0.1, 3);
  cvc::capsid_seed_params pb;
  pb.t_low = 20;
  const cvc::point_classes ra = cvc::capsid_critical_points(va),
                           rb = cvc::capsid_critical_points(vb, pb);
  cvc::point_classes ta, tb;
  std::thread t1([&] { ta = cvc::capsid_critical_points(va); });
  std::thread t2([&] { tb = cvc::capsid_critical_points(vb, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(ta == ra);
  EXPECT_TRUE(tb == rb);
}

TEST(SeedGenerators, ParameterTables) {
  cvc::app a;
  cvc::rotation_seed_params p;
  p.folds = 5;
  p.axis = {0.0, 1.0, 1.0};
  p.center = {1.5, 2.0, -3.0};
  cvc::state &node = a.root()("doc.rot");
  cvc::params_to_state(p, node);
  const auto back = cvc::params_from_state<cvc::rotation_seed_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.folds, 5);
  EXPECT_EQ(back.value.axis, p.axis);
  EXPECT_EQ(back.value.center, p.center);
  cvc::capsid_seed_params c;
  c.h = 3;
  c.k = 3;
  cvc::state &cn = a.root()("doc.capsid");
  cvc::params_to_state(c, cn);
  const auto cb = cvc::params_from_state<cvc::capsid_seed_params>(cn);
  ASSERT_TRUE(cb.ok()) << cb.message();
  EXPECT_EQ(cb.value.h, 3);
  EXPECT_EQ(cb.value.normalized_thresholds, true);
  const cvc::capsid_seed_params d; // the legacy defaults
  EXPECT_EQ(d.h, 2);
  EXPECT_EQ(d.k, 1);
  EXPECT_EQ(d.t_low, 0.0);
}

} // namespace

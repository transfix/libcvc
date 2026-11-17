/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The paper mode of the GenSeg port (seed_mode::critical_points, cvc::critical_point_seeds;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5 steps 3-4, P6), by the §1.7 matrix: T1 SegMed's
// diffused field and critical points against the legacy library's (tools/volrover-goldens, segmed),
// T3 known shapes, T4 invariants, T5 bad input, T6 calls and apps independent, T7 resumable, T8 the
// parameter table.

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

namespace {

// ---- helpers ------------------------------------------------------------------------------

struct grid {
  int x = 0, y = 0, z = 0;
  std::vector<float> v; // x fastest
  std::size_t n() const { return v.size(); }
  std::size_t at(int i, int j, int k) const { return (std::size_t(k) * y + j) * x + i; }
};

cvc::volume to_volume(cvc::app &a, const grid &g, cvc::data_type t = cvc::Float) {
  cvc::volume vol(a, cvc::dimension(g.x, g.y, g.z), t);
  for (std::size_t i = 0; i < g.n(); ++i)
    vol(i, double(g.v[i]));
  return vol;
}

// Gaussian blobs (centre, sigma, height) over a background of 0.02, plus a little deterministic
// noise when `noise` > 0.
grid blobs(int n, const std::vector<std::pair<std::array<double, 3>, std::array<double, 2>>> &b,
           double noise = 0.0, unsigned seed = 1) {
  grid g;
  g.x = g.y = g.z = n;
  g.v.resize(std::size_t(n) * n * n);
  std::uint32_t s = seed;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double v = 0.02;
        for (const auto &bl : b) {
          const double d2 = (i - bl.first[0]) * (i - bl.first[0]) +
                            (j - bl.first[1]) * (j - bl.first[1]) +
                            (k - bl.first[2]) * (k - bl.first[2]);
          v += bl.second[1] * std::exp(-d2 / (2.0 * bl.second[0] * bl.second[0]));
        }
        if (noise > 0) {
          s = 1664525u * s + 1013904223u;
          v += noise * (double(s) / 4294967296.0 - 0.5);
        }
        g.v[g.at(i, j, k)] = float(std::max(v, 0.0));
      }
  return g;
}

grid two_blobs(int n = 24) {
  return blobs(n, {{{7, 8, 12}, {2.5, 1.0}}, {{16, 15, 12}, {2.5, 0.9}}});
}

std::string slurp(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

std::vector<double> json_numbers(const std::string &json, const std::string &key) {
  std::vector<double> out;
  std::size_t p = json.find("\"" + key + "\"");
  if (p == std::string::npos)
    return out;
  p = json.find(':', p) + 1;
  while (p < json.size() && std::isspace((unsigned char)json[p]))
    ++p;
  int depth = 0;
  std::string num;
  auto flush = [&] {
    if (!num.empty())
      out.push_back(std::stod(num));
    num.clear();
  };
  for (; p < json.size(); ++p) {
    const char ch = json[p];
    if (ch == '[') {
      ++depth;
    } else if (ch == ']') {
      flush();
      if (--depth <= 0)
        break;
    } else if (std::isdigit((unsigned char)ch) || ch == '-' || ch == '.' || ch == 'e' ||
               ch == 'E' || ch == '+') {
      num += ch;
    } else {
      flush();
      if (depth == 0 && ch == ',')
        break;
    }
  }
  flush();
  return out;
}

float le_float(const std::string &bytes, std::size_t i) {
  const auto *b = reinterpret_cast<const unsigned char *>(bytes.data()) + 4 * i;
  const std::uint32_t u = std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) |
                          (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24);
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

std::vector<std::array<int, 3>> points_of(const cvc::point_class &c) {
  std::vector<std::array<int, 3>> out;
  for (std::size_t p = 0; p + 2 < c.points.size(); p += 3)
    out.push_back({c.points[p], c.points[p + 1], c.points[p + 2]});
  return out;
}

bool same_seeds(const cvc::point_classes &a, const cvc::point_classes &b) { return a == b; }

// SegMed's parameters (GVF.cpp): its field, its weights, its tlow.
cvc::seeded_segmentation_params segmed_params(double tlow) {
  cvc::seeded_segmentation_params p;
  p.normalized_thresholds = true;
  p.cp_field = cvc::critical_field::gradient_magnitude;
  p.cp_weights = cvc::diffusion_weights::initial;
  p.cp_background_threshold = tlow;
  p.cp_iterations = 10;
  p.cp_k = 2.0;
  p.cp_sample = 20;
  return p;
}

// ---- T1: SegMed's field and critical points (tools/volrover-goldens, segmed) ------------------

struct segmed_case {
  grid g;
  double tlow = 0;
};

segmed_case load_segmed(const std::string &name) {
  const std::string dir = std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/segmed/";
  const std::string json = slurp(dir + name + ".json");
  segmed_case c;
  const std::vector<double> dims = json_numbers(json, "dims");
  if (dims.size() != 3)
    return c;
  c.g.x = int(dims[0]);
  c.g.y = int(dims[1]);
  c.g.z = int(dims[2]);
  c.tlow = json_numbers(json, "tlow").at(0);
  const std::string raw = slurp(dir + name + ".raw");
  c.g.v.resize(std::size_t(c.g.x) * c.g.y * c.g.z);
  if (raw.size() != c.g.v.size() * 4) {
    c.g.v.clear();
    return c;
  }
  for (std::size_t i = 0; i < c.g.v.size(); ++i)
    c.g.v[i] = le_float(raw, i);
  return c;
}

// SegMed's initial field, as the legacy read_data, GetGradient and InitGVF compute it: the volume
// rescaled to 0..255, |grad| by the 3x3x3 operator rescaled to 0..255, and the gradient of that,
// divided by its longest vector's length. (The port computes it inside critical_point_seeds; this
// copy lets the test hand gradient_vector_diffusion the same field the legacy gvfflow() got.)
cvc::vector_field segmed_initial_field(const grid &in) {
  const int X = in.x, Y = in.y, Z = in.z;
  const std::size_t n = in.n();
  std::vector<float> d(in.v);
  float mn = d[0], mx = d[0];
  for (float f : d) {
    mn = std::min(mn, f);
    mx = std::max(mx, f);
  }
  for (float &f : d)
    f = 255 * (f - mn) / (mx - mn);
  auto I = [&](int i, int j, int k) { return (std::size_t(k) * Y + j) * X + i; };
  // The operator, on f: the three components at (x, y, z).
  auto grad = [&](const std::vector<float> &f, int x, int y, int z, float &gx, float &gy,
                  float &gz) {
    gx = gy = gz = 0.0f;
    for (int j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++)
      for (int k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
        const float t = f[I(std::min(x + 1, X - 1), j, k)] - f[I(std::max(x - 1, 0), j, k)];
        gx += t;
        if (j == y || k == z)
          gx += t;
        if (j == y && k == z)
          gx += 2.0f * t;
      }
    for (int i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
      for (int k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
        const float t = f[I(i, std::min(y + 1, Y - 1), k)] - f[I(i, std::max(y - 1, 0), k)];
        gy += t;
        if (i == x || k == z)
          gy += t;
        if (i == x && k == z)
          gy += 2.0f * t;
      }
    for (int i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
      for (int j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++) {
        const float t = f[I(i, j, std::min(z + 1, Z - 1))] - f[I(i, j, std::max(z - 1, 0))];
        gz += t;
        if (i == x || j == y)
          gz += t;
        if (i == x && j == y)
          gz += 2.0f * t;
      }
  };
  std::vector<float> edge(n);
  float gmin = 999999, gmax = 0;
  for (int z = 0; z < Z; z++)
    for (int y = 0; y < Y; y++)
      for (int x = 0; x < X; x++) {
        float gx, gy, gz;
        grad(d, x, y, z, gx, gy, gz);
        const float g = std::sqrt(gx * gx + gy * gy + gz * gz);
        edge[I(x, y, z)] = g;
        gmin = std::min(gmin, g);
        gmax = std::max(gmax, g);
      }
  if (gmin < gmax)
    for (float &e : edge)
      e = 255.0f * (e - gmin) / (gmax - gmin);
  cvc::vector_field f(X, Y, Z);
  gmin = 999999;
  gmax = 0;
  for (int z = 0; z < Z; z++)
    for (int y = 0; y < Y; y++)
      for (int x = 0; x < X; x++) {
        float gx, gy, gz;
        grad(edge, x, y, z, gx, gy, gz);
        const std::size_t v = I(x, y, z);
        f.x[v] = gx;
        f.y[v] = gy;
        f.z[v] = gz;
        const float g = std::sqrt(gx * gx + gy * gy + gz * gz);
        gmin = std::min(gmin, g);
        gmax = std::max(gmax, g);
      }
  if (gmin < gmax)
    for (std::size_t v = 0; v < n; v++) {
      f.x[v] /= gmax;
      f.y[v] /= gmax;
      f.z[v] /= gmax;
    }
  return f;
}

// Returns how many critical voxels SegMed's thinning dropped from the legacy list.
std::size_t check_segmed(const std::string &name) {
  SCOPED_TRACE(name);
  const segmed_case c = load_segmed(name);
  if (c.g.v.empty()) {
    ADD_FAILURE() << "no input for " << name;
    return 0;
  }
  const std::string goldens =
      std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/segmed/";

  // The diffused field: cvc::gradient_vector_diffusion on SegMed's initial field, with SegMed's
  // weights, against what the legacy gvfflow() left in `velocity`.
  cvc::gradient_vector_diffusion_params dp;
  dp.weights = cvc::diffusion_weights::initial;
  const cvc::vector_field field = cvc::gradient_vector_diffusion(segmed_initial_field(c.g), dp);
  const std::string gf = slurp(goldens + name + ".field.f32");
  if (gf.size() != 12 * c.g.n()) {
    ADD_FAILURE() << "the field golden has " << gf.size() << " bytes";
    return 0;
  }
  double worst = 0;
  std::size_t equal = 0;
  for (std::size_t v = 0; v < c.g.n(); ++v) {
    const float got[3] = {field.x[v], field.y[v], field.z[v]};
    for (int a = 0; a < 3; ++a) {
      const float want = le_float(gf, 3 * v + a);
      worst = std::max(worst, double(std::fabs(got[a] - want)));
      if (got[a] == want)
        ++equal;
    }
  }
  EXPECT_LE(worst, 1e-6) << "the diffused field differs from the legacy library's";

  // The critical points: critical_point_seeds with SegMed's parameters gives the legacy list as its
  // feature class (SegMed's minima above tlow, thinned), in the legacy order.
  cvc::app a;
  const cvc::point_classes seeds =
      cvc::critical_point_seeds(to_volume(a, c.g), segmed_params(c.tlow));
  if (seeds.classes.size() != 2) {
    ADD_FAILURE() << "not two classes";
    return 0;
  }
  EXPECT_EQ(seeds.classes[0].name, "feature");
  EXPECT_EQ(seeds.classes[1].name, "background");
  std::vector<std::array<int, 3>> want;
  std::istringstream in(slurp(goldens + name + ".critical.txt"));
  int i, j, k;
  while (in >> i >> j >> k)
    want.push_back({i, j, k});
  const std::vector<std::array<int, 3>> got = points_of(seeds.classes[0]);
  EXPECT_EQ(got, want);
  // Unthinned, for the record: how much of the list is SegMed's thinning.
  cvc::seeded_segmentation_params all = segmed_params(c.tlow);
  all.cp_sample = 0;
  const std::size_t unthinned = cvc::critical_point_seeds(to_volume(a, c.g), all).classes[0].size();
  std::cout << name << ": field to " << worst << " (" << equal << " of " << 3 * c.g.n()
            << " components bit for bit), " << want.size() << " legacy critical points"
            << (got == want ? ", as the legacy library" : ", NOT as the legacy library") << " ("
            << unthinned << " before the thinning; " << seeds.classes[1].size()
            << " background seeds besides)\n";
  return unthinned - std::min(unthinned, got.size());
}

TEST(CriticalPoints, MatchSegMed) {
  std::size_t thinned = 0;
  for (const char *name : {"many_blobs", "plateau", "noisy_blob", "noisy_slab"})
    thinned += check_segmed(name);
  EXPECT_GT(thinned, 0u) << "no case exercises SegMed's thinning";
}

// ---- T3: known shapes -------------------------------------------------------------------------

double distance(const std::array<int, 3> &p, const std::array<double, 3> &c) {
  return std::sqrt((p[0] - c[0]) * (p[0] - c[0]) + (p[1] - c[1]) * (p[1] - c[1]) +
                   (p[2] - c[2]) * (p[2] - c[2]));
}

TEST(CriticalPoints, TwoBlobsGiveAFeatureSeedEach) {
  // The paper's mode: the gradient of the volume sinks at each blob's peak, a maximum, which seeds
  // the feature; the background's minima lie far from the blobs.
  cvc::app a;
  const grid g = two_blobs();
  const cvc::point_classes s = cvc::critical_point_seeds(to_volume(a, g));
  ASSERT_EQ(s.classes.size(), 2u);
  const auto feature = points_of(s.classes[0]), background = points_of(s.classes[1]);
  for (const std::array<double, 3> c : {std::array<double, 3>{7, 8, 12}, {16, 15, 12}}) {
    double best = 1e9;
    for (const auto &p : feature)
      best = std::min(best, distance(p, c));
    EXPECT_LE(best, 1.0) << "no feature seed at the blob at " << c[0] << ", " << c[1] << ", "
                         << c[2];
  }
  EXPECT_FALSE(background.empty());
  for (const auto &p : background)
    EXPECT_GT(std::min(distance(p, {7, 8, 12}), distance(p, {16, 15, 12})), 3.0);
  std::cout << feature.size() << " feature and " << background.size() << " background seeds\n";
}

TEST(CriticalPoints, SegmentingWithNoPointClassesGivesFeatureAndBackground) {
  // §5: "without a click, this gives the paper's fully automatic two-class segmentation".
  cvc::app a;
  const grid g = two_blobs();
  cvc::seeded_segmentation_params p;
  p.automatic = cvc::seed_mode::critical_points;
  const cvc::seeded_segmentation r = cvc::segment_seeded(to_volume(a, g), cvc::point_classes{}, p);
  ASSERT_EQ(r.class_names, (std::vector<std::string>{"feature", "background"}));
  const unsigned char *lab = r.labels.data_ptr();
  EXPECT_EQ(lab[g.at(7, 8, 12)], 0) << "a blob's peak is feature";
  EXPECT_EQ(lab[g.at(16, 15, 12)], 0);
  EXPECT_GT(r.class_voxels[0], 0u);
  EXPECT_GT(r.class_voxels[1], 0u);
  EXPECT_EQ(r.automatic_seeds, r.classified_seeds);
  EXPECT_GT(r.automatic_seeds, 0u);
  // The same seeds as critical_point_seeds gives.
  const cvc::point_classes s = cvc::critical_point_seeds(to_volume(a, g), p);
  EXPECT_EQ(r.automatic_seeds, s.classes[0].size() + s.classes[1].size());
  std::cout << "feature " << r.class_voxels[0] << " voxels, background " << r.class_voxels[1]
            << ", " << r.border.size() << " border voxels\n";
}

TEST(CriticalPoints, PointClassesAddGroups) {
  // With point classes, the paper's seeds are two more groups after them; a point class's seed
  // wins the voxel it shares with a critical point.
  cvc::app a;
  const grid g = two_blobs();
  cvc::point_classes pc;
  pc.add("left", 7, 8, 12);
  cvc::seeded_segmentation_params p;
  p.automatic = cvc::seed_mode::critical_points;
  const cvc::seeded_segmentation r = cvc::segment_seeded(to_volume(a, g), pc, p);
  ASSERT_EQ(r.class_names, (std::vector<std::string>{"left", "feature", "background"}));
  const unsigned char *lab = r.labels.data_ptr();
  EXPECT_EQ(lab[g.at(7, 8, 12)], 0) << "the point class's seed wins its voxel";
  EXPECT_EQ(lab[g.at(16, 15, 12)], 1) << "the other blob is feature";
  EXPECT_GT(r.class_voxels[2], 0u);
}

// ---- T4: invariants ---------------------------------------------------------------------------

TEST(CriticalPoints, ThresholdsMoveSeedsBetweenTheClasses) {
  // The paper's §4: with no thinning, the thresholds only move seeds between the two classes.
  cvc::app a;
  const grid g = blobs(20, {{{6, 6, 10}, {2.0, 1.0}}, {{14, 13, 9}, {2.0, 0.4}}}, 0.15, 7);
  const cvc::volume vol = to_volume(a, g);
  cvc::seeded_segmentation_params p;
  p.cp_sample = 0;
  const cvc::point_classes base = cvc::critical_point_seeds(vol, p);
  const std::size_t total = base.classes[0].size() + base.classes[1].size();
  ASSERT_GT(total, 0u);
  std::set<std::array<int, 3>> all;
  for (const auto &c : base.classes)
    for (const auto &q : points_of(c))
      all.insert(q);
  EXPECT_EQ(all.size(), total) << "no voxel is in both classes";
  for (const double ft : {0.1, 0.5, 2.0}) {
    for (const double bt : {-1.0, 0.1, 0.5}) {
      p.cp_feature_threshold = ft;
      p.cp_background_threshold = bt;
      const cvc::point_classes s = cvc::critical_point_seeds(vol, p);
      std::set<std::array<int, 3>> moved;
      for (const auto &c : s.classes)
        for (const auto &q : points_of(c))
          moved.insert(q);
      EXPECT_EQ(moved, all) << "thresholds " << ft << ", " << bt;
      // Every feature seed is at or above the feature threshold or a minimum above the background
      // threshold; every background seed below one or the other.
      for (const auto &q : points_of(s.classes[0])) {
        const float v = g.v[g.at(q[0], q[1], q[2])];
        EXPECT_TRUE(v >= float(ft) || v > float(bt)) << q[0] << " " << q[1] << " " << q[2];
      }
    }
  }
  // A feature threshold above every voxel and a background threshold of +inf: nothing is feature.
  p.cp_feature_threshold = 10.0;
  p.cp_background_threshold = std::numeric_limits<double>::infinity();
  const cvc::point_classes none = cvc::critical_point_seeds(vol, p);
  EXPECT_TRUE(none.classes[0].points.empty());
  EXPECT_EQ(none.classes[1].size(), total);
}

TEST(CriticalPoints, ThinningKeepsASubsetInOrder) {
  cvc::app a;
  const grid g = blobs(20, {{{10, 10, 10}, {4.0, 1.0}}}, 0.4, 11);
  const cvc::volume vol = to_volume(a, g);
  cvc::seeded_segmentation_params p;
  p.cp_sample = 0;
  const cvc::point_classes all = cvc::critical_point_seeds(vol, p);
  for (const int sample : {1, 5, 20}) {
    p.cp_sample = sample;
    const cvc::point_classes thin = cvc::critical_point_seeds(vol, p);
    for (int c = 0; c < 2; ++c) {
      const auto every = points_of(all.classes[c]), kept = points_of(thin.classes[c]);
      EXPECT_LE(kept.size(), every.size());
      std::set<std::array<int, 3>> pool(every.begin(), every.end());
      for (const auto &q : kept)
        EXPECT_TRUE(pool.count(q)) << "a kept seed that is not a critical point";
      const auto x_fastest = [&](const std::array<int, 3> &u, const std::array<int, 3> &w) {
        return g.at(u[0], u[1], u[2]) < g.at(w[0], w[1], w[2]);
      };
      EXPECT_TRUE(std::is_sorted(kept.begin(), kept.end(), x_fastest));
      if (!every.empty())
        EXPECT_FALSE(kept.empty()) << "each run keeps its first voxel";
    }
  }
}

TEST(CriticalPoints, DataUnitThresholdsAreTheNormalisedOnes) {
  cvc::app a;
  grid g = blobs(18, {{{9, 9, 9}, {3.0, 1.0}}}, 0.2, 3);
  for (float &f : g.v)
    f = 3.0f * f - 0.5f; // data units with a negative minimum
  const float mn = *std::min_element(g.v.begin(), g.v.end());
  const float mx = *std::max_element(g.v.begin(), g.v.end());
  cvc::seeded_segmentation_params p;
  p.cp_sample = 0;
  p.cp_feature_threshold = 1.2;
  p.cp_background_threshold = 0.1;
  const cvc::point_classes data = cvc::critical_point_seeds(to_volume(a, g), p);
  p.normalized_thresholds = true;
  p.cp_feature_threshold = 255 * (float(1.2) - mn) / (mx - mn);
  p.cp_background_threshold = 255 * (float(0.1) - mn) / (mx - mn);
  const cvc::point_classes norm = cvc::critical_point_seeds(to_volume(a, g), p);
  EXPECT_TRUE(same_seeds(data, norm));
}

TEST(CriticalPoints, EveryVoxelTypeGivesTheSameSeeds) {
  cvc::app a;
  grid g = two_blobs(16);
  for (float &f : g.v)
    f = std::round(f * 200.0f); // integers that every type holds
  const cvc::point_classes ref = cvc::critical_point_seeds(to_volume(a, g, cvc::Float));
  for (const cvc::data_type t :
       {cvc::UChar, cvc::UShort, cvc::UInt, cvc::Double, cvc::Int, cvc::Int64, cvc::UInt64})
    EXPECT_TRUE(same_seeds(cvc::critical_point_seeds(to_volume(a, g, t)), ref)) << int(t);
}

// ---- T5: bad input ----------------------------------------------------------------------------

template <class F> std::string message_of(F f) {
  try {
    f();
  } catch (const cvc::segmentation_error &e) {
    return e.what();
  } catch (const std::exception &e) {
    return std::string("WRONG TYPE: ") + e.what();
  }
  return "no exception";
}

TEST(CriticalPoints, BadInputThrowsWithAUsefulMessage) {
  cvc::app a;
  grid flat = two_blobs(8);
  std::fill(flat.v.begin(), flat.v.end(), 3.0f);
  EXPECT_NE(message_of([&] { cvc::critical_point_seeds(to_volume(a, flat)); }).find("constant"),
            std::string::npos);
  grid nan = two_blobs(8);
  nan.v[nan.at(1, 2, 3)] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_NE(
      message_of([&] { cvc::critical_point_seeds(to_volume(a, nan)); }).find("(1, 2, 3) is NaN"),
      std::string::npos);
  cvc::seeded_segmentation_params p;
  p.cp_iterations = -1;
  EXPECT_NE(message_of([&] {
              cvc::critical_point_seeds(to_volume(a, two_blobs(8)), p);
            }).find("cp_iterations"),
            std::string::npos);
  p = {};
  p.cp_k = -2;
  EXPECT_NE(
      message_of([&] { cvc::critical_point_seeds(to_volume(a, two_blobs(8)), p); }).find("cp_k"),
      std::string::npos);
  p = {};
  p.cp_sample = -1;
  EXPECT_NE(message_of([&] {
              cvc::critical_point_seeds(to_volume(a, two_blobs(8)), p);
            }).find("cp_sample"),
            std::string::npos);
  // segment_seeded: without critical points, no point classes is still "no seeds".
  EXPECT_NE(message_of([&] {
              cvc::segment_seeded(to_volume(a, two_blobs(8)), cvc::point_classes{});
            }).find("no seeds"),
            std::string::npos);
  // With them, 253 point classes is one too many.
  cvc::point_classes many;
  for (int c = 0; c < 253; ++c)
    many.add("c" + std::to_string(c), 0, 0, 0);
  p = {};
  p.automatic = cvc::seed_mode::critical_points;
  EXPECT_NE(message_of([&] {
              cvc::segment_seeded(to_volume(a, two_blobs(8)), many, p);
            }).find("more than 252"),
            std::string::npos);
}

// ---- T6: no singletons ------------------------------------------------------------------------

TEST(CriticalPoints, CallsAreIndependent) {
  // A then B equals B then A equals each alone, and two threads with different parameters give
  // what each gives alone.
  cvc::app a;
  const cvc::volume va = to_volume(a, two_blobs());
  const cvc::volume vb = to_volume(a, blobs(20, {{{10, 10, 10}, {4.0, 1.0}}}, 0.3, 5));
  cvc::seeded_segmentation_params pa, pb;
  pb.cp_field = cvc::critical_field::gradient_magnitude;
  pb.cp_weights = cvc::diffusion_weights::initial;
  pb.cp_background_threshold = 0.3;
  pb.cp_iterations = 4;
  const cvc::point_classes ra = cvc::critical_point_seeds(va, pa);
  const cvc::point_classes rb = cvc::critical_point_seeds(vb, pb);
  EXPECT_TRUE(same_seeds(cvc::critical_point_seeds(vb, pb), rb));
  EXPECT_TRUE(same_seeds(cvc::critical_point_seeds(va, pa), ra));
  cvc::point_classes ta, tb;
  std::thread t1([&] { ta = cvc::critical_point_seeds(va, pa); });
  std::thread t2([&] { tb = cvc::critical_point_seeds(vb, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same_seeds(ta, ra));
  EXPECT_TRUE(same_seeds(tb, rb));
}

TEST(CriticalPoints, TwoAppsInOneProcess) {
  cvc::app a1, a2;
  cvc::seeded_segmentation_params p;
  p.automatic = cvc::seed_mode::critical_points;
  const grid g = two_blobs(16);
  const cvc::seeded_segmentation r1 = cvc::segment_seeded(to_volume(a1, g), {}, p);
  const cvc::seeded_segmentation r2 = cvc::segment_seeded(to_volume(a2, g), {}, p);
  EXPECT_EQ(&r1.labels.ctx(), &a1);
  EXPECT_EQ(&r2.labels.ctx(), &a2);
  EXPECT_EQ(std::memcmp(r1.labels.data_ptr(), r2.labels.data_ptr(), g.n()), 0);
}

// ---- T7: resumable ----------------------------------------------------------------------------

TEST(CriticalPoints, StepperIsResumable) {
  cvc::app a;
  const cvc::volume vol = to_volume(a, blobs(20, {{{10, 10, 10}, {4.0, 1.0}}}, 0.3, 9));
  cvc::seeded_segmentation_params p;
  p.cp_background_threshold = 0.4;
  p.cp_feature_threshold = 0.2;
  cvc::test::t7_subject<cvc::critical_point_seeds_stepper, cvc::point_classes> s;
  s.name = "critical_point_seeds 20^3";
  s.make = [&] { return std::make_unique<cvc::critical_point_seeds_stepper>(vol, p); };
  s.take = [](cvc::critical_point_seeds_stepper &st) { return st.take(); };
  s.equal = same_seeds;
  s.blocking = [&] { return cvc::critical_point_seeds(vol, p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "normalising", "seeding"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

bool same_segmentation(const cvc::seeded_segmentation &a, const cvc::seeded_segmentation &b) {
  return static_cast<const cvc::voxels &>(a.labels) == static_cast<const cvc::voxels &>(b.labels) &&
         a.border == b.border && a.class_names == b.class_names &&
         a.class_voxels == b.class_voxels && a.automatic_seeds == b.automatic_seeds &&
         a.warnings == b.warnings;
}

TEST(CriticalPoints, SegmentStepperStaysSlicedInPaperMode) {
  cvc::app a;
  const cvc::volume vol = to_volume(a, two_blobs(20));
  cvc::point_classes pc;
  pc.add("left", 7, 8, 12);
  cvc::seeded_segmentation_params p;
  p.automatic = cvc::seed_mode::critical_points;
  cvc::test::t7_subject<cvc::segment_stepper, cvc::seeded_segmentation> s;
  s.name = "segment_seeded critical_points 20^3";
  s.make = [&] { return std::make_unique<cvc::segment_stepper>(vol, pc, p); };
  s.take = [](cvc::segment_stepper &st) { return st.take(); };
  s.equal = same_segmentation;
  s.blocking = [&] { return cvc::segment_seeded(vol, pc, p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "normalising", "seeding", "marching", "writing"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST(CriticalPoints, AStepDoesABoundedAmountOfWork) {
  // However large the volume, a step of budget 64 is short: a 40^3 run takes many more steps than
  // a 16^3 one, each about as short.
  cvc::app a;
  for (const int n : {16, 40}) {
    cvc::critical_point_seeds_stepper st(
        to_volume(a, blobs(n, {{{n / 2.0, n / 2.0, n / 2.0}, {n / 5.0, 1.0}}}, 0.2, 4)));
    std::size_t steps = 0;
    while (!st.step(64))
      ++steps;
    std::cout << n << "^3: " << steps << " steps of 64 units\n";
    EXPECT_GT(steps, std::size_t(n) * n * n * 10 / 64) << "at least ten units a voxel";
  }
}

TEST(CriticalPoints, ACancelledStepperGivesNothing) {
  cvc::app a;
  auto job = cvc::critical_point_seeds_job(to_volume(a, two_blobs(16)));
  job->step(100);
  EXPECT_FALSE(job->done());
  job->cancel();
  EXPECT_TRUE(job->step(100));
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  grid flat = two_blobs(8);
  std::fill(flat.v.begin(), flat.v.end(), 1.0f);
  auto bad = cvc::critical_point_seeds_job(to_volume(a, flat));
  EXPECT_TRUE(bad->step(0));
  EXPECT_TRUE(bad->failed());
  EXPECT_NE(bad->error().find("constant"), std::string::npos);
  EXPECT_THROW((void)bad->result(), cvc::segmentation_error);
}

// ---- T8: the parameter table ------------------------------------------------------------------

TEST(CriticalPoints, ParameterTable) {
  cvc::app a;
  cvc::seeded_segmentation_params p;
  p.automatic = cvc::seed_mode::critical_points;
  p.cp_iterations = 7;
  p.cp_k = 1.5;
  p.cp_feature_threshold = 0.25;
  p.cp_field = cvc::critical_field::gradient_magnitude;
  p.cp_weights = cvc::diffusion_weights::initial;
  p.cp_sample = 3;
  cvc::state &node = a.root()("doc.seg");
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("automatic").value(), "critical_points");
  EXPECT_EQ(node("cp_background_threshold").value(), "inf");
  EXPECT_EQ(node("cp_field").property("choices"), "image,gradient_magnitude");
  EXPECT_EQ(node("cp_weights").property("choices"), "current,initial");
  const auto back = cvc::params_from_state<cvc::seeded_segmentation_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.automatic, cvc::seed_mode::critical_points);
  EXPECT_EQ(back.value.cp_iterations, 7);
  EXPECT_EQ(back.value.cp_k, 1.5);
  EXPECT_EQ(back.value.cp_feature_threshold, 0.25);
  EXPECT_TRUE(std::isinf(back.value.cp_background_threshold) &&
              back.value.cp_background_threshold > 0);
  EXPECT_EQ(back.value.cp_field, cvc::critical_field::gradient_magnitude);
  EXPECT_EQ(back.value.cp_weights, cvc::diffusion_weights::initial);
  EXPECT_EQ(back.value.cp_sample, 3);
  // The defaults: SegMed's numbers, the paper's field and weights, no reclassification.
  const cvc::seeded_segmentation_params d;
  EXPECT_EQ(d.automatic, cvc::seed_mode::local_maxima) << "the default stays GenSeg's (§5 step 5)";
  EXPECT_EQ(d.cp_iterations, 10);
  EXPECT_EQ(d.cp_k, 2.0);
  EXPECT_EQ(d.cp_sample, 0);
  EXPECT_EQ(d.cp_field, cvc::critical_field::image);
  EXPECT_EQ(d.cp_weights, cvc::diffusion_weights::current);
  EXPECT_TRUE(std::isinf(d.cp_feature_threshold) && d.cp_feature_threshold < 0);
  // A hyphenated choice is read too ("critical-points", as the CLI's --automatic takes it).
  node("automatic").value("critical-points");
  const auto hy = cvc::params_from_state<cvc::seeded_segmentation_params>(node);
  ASSERT_TRUE(hy.ok()) << hy.message();
  EXPECT_EQ(hy.value.automatic, cvc::seed_mode::critical_points);
  node("cp_k").value("-1");
  EXPECT_FALSE(cvc::params_from_state<cvc::seeded_segmentation_params>(node).ok());
}

} // namespace

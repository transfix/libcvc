/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::segment_seeded (cvc/volume/segmentation.h), the VolumeRover GenSeg port
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5, P5): legacy parity (T1), the pieces (T2),
// known shapes (T3), invariants (T4), bad input (T5), no singletons (T6), resumable (T7) and the
// parameter table (T8). Runs natively and under node (the ports lane). With CVC_VOLROVER_DATA set,
// also legacy parity and agreement with the published segmentations on the group's real maps.

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <map>
#include <random>
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

// Classes "c0", "c1", ... in that order, with the given points (class, i, j, k).
cvc::point_classes classes_of(const std::vector<std::array<int, 4>> &pts) {
  cvc::point_classes pc;
  int classes = 0;
  for (const auto &p : pts)
    classes = std::max(classes, p[0] + 1);
  for (int c = 0; c < classes; ++c) {
    cvc::point_class k;
    k.name = "c" + std::to_string(c);
    pc.set_class(k);
  }
  for (const auto &p : pts)
    pc.add("c" + std::to_string(p[0]), p[1], p[2], p[3]);
  return pc;
}

const unsigned char *labels_of(const cvc::seeded_segmentation &r) { return r.labels.data_ptr(); }

// The same voxels, type, dimensions and bounding box (volume::operator== is not const).
bool same_volume(const cvc::volume &a, const cvc::volume &b) {
  return static_cast<const cvc::voxels &>(a) == static_cast<const cvc::voxels &>(b) &&
         a.boundingBox() == b.boundingBox();
}

bool same(const cvc::seeded_segmentation &a, const cvc::seeded_segmentation &b) {
  if (!same_volume(a.labels, b.labels) || a.classes.size() != b.classes.size())
    return false;
  for (std::size_t c = 0; c < a.classes.size(); ++c)
    if (!same_volume(a.classes[c], b.classes[c]))
      return false;
  return a.border == b.border && a.class_names == b.class_names &&
         a.class_voxels == b.class_voxels && a.automatic_seeds == b.automatic_seeds &&
         a.classified_seeds == b.classified_seeds && a.t_low_normalized == b.t_low_normalized &&
         a.t_high_normalized == b.t_high_normalized && a.warnings == b.warnings;
}

std::string slurp(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

// The numbers in the value of `key` in a small JSON text: one number, or every number inside
// the bracketed array (nested arrays flattened) that follows the key.
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

// ---- T1: legacy parity (tools/volrover-goldens) ---------------------------------------------

struct legacy_case {
  grid g;
  double tlow = 0, thigh = 0;
  std::vector<std::array<int, 4>> seeds;
};

legacy_case load_case(const std::string &name) {
  const std::string dir = std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/genseg/";
  const std::string json = slurp(dir + name + ".json");
  legacy_case c;
  const std::vector<double> dims = json_numbers(json, "dims");
  if (dims.size() != 3)
    return c;
  c.g.x = int(dims[0]);
  c.g.y = int(dims[1]);
  c.g.z = int(dims[2]);
  c.tlow = json_numbers(json, "tlow").at(0);
  c.thigh = json_numbers(json, "thigh").at(0);
  const std::vector<double> s = json_numbers(json, "seeds");
  for (std::size_t i = 0; i + 3 < s.size(); i += 4)
    c.seeds.push_back({int(s[i]), int(s[i + 1]), int(s[i + 2]), int(s[i + 3])});
  const std::string raw = slurp(dir + name + ".raw");
  c.g.v.resize(std::size_t(c.g.x) * c.g.y * c.g.z);
  if (raw.size() != c.g.v.size() * 4) {
    c.g.v.clear();
    return c;
  }
  for (std::size_t i = 0; i < c.g.v.size(); ++i) { // float32, little-endian
    const auto *b = reinterpret_cast<const unsigned char *>(raw.data()) + 4 * i;
    const std::uint32_t u = std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) |
                            (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24);
    std::memcpy(&c.g.v[i], &u, 4);
  }
  return c;
}

cvc::seeded_segmentation_params legacy_params(const legacy_case &c) {
  cvc::seeded_segmentation_params p;
  p.t_low = c.tlow;
  p.t_high = c.thigh;
  p.normalized_thresholds = true; // the legacy entry's integers, on the 0..255 scale
  return p;
}

void check_parity(const std::string &name) {
  SCOPED_TRACE(name);
  const legacy_case c = load_case(name);
  ASSERT_FALSE(c.g.v.empty()) << "no input for " << name;
  cvc::app a;
  const cvc::seeded_segmentation r =
      cvc::segment_seeded(to_volume(a, c.g), classes_of(c.seeds), legacy_params(c));
  const std::string goldens =
      std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/genseg/";
  const std::string gl = slurp(goldens + name + ".labels.u8");
  ASSERT_EQ(gl.size(), c.g.n());
  // The goldens read a label where the legacy _subunit_ file was nonzero, and it held the voxel
  // rescaled to 0..255: so a labelled voxel at the volume's minimum (rescaled to 0) reads as no
  // label there.
  const float mn = *std::min_element(c.g.v.begin(), c.g.v.end());
  const unsigned char *lab = labels_of(r);
  std::size_t differ = 0, labelled = 0;
  for (std::size_t v = 0; v < c.g.n(); ++v) {
    const unsigned char got = c.g.v[v] == mn ? 255 : lab[v];
    const unsigned char want = static_cast<unsigned char>(gl[v]);
    if (want != 255)
      ++labelled;
    if (got != want && differ++ < 10)
      ADD_FAILURE() << "voxel " << v << " (" << v % c.g.x << ", " << (v / c.g.x) % c.g.y << ", "
                    << v / (std::size_t(c.g.x) * c.g.y) << "): " << int(got) << ", legacy "
                    << int(want);
  }
  EXPECT_EQ(differ, 0u) << "voxels whose label differs from the legacy library's";
  // The border voxels, in the legacy file's order.
  std::vector<std::array<int, 3>> border;
  std::istringstream in(slurp(goldens + name + ".indices.txt"));
  int i, j, k;
  while (in >> i >> j >> k)
    border.push_back({i, j, k});
  EXPECT_EQ(r.border, border);
  std::cout << name << ": " << labelled << " labelled voxels, " << border.size()
            << " border voxels, " << r.automatic_seeds << " automatic seeds (" << r.classified_seeds
            << " classified by the sweep)"
            << (differ == 0 && r.border == border ? ", as the legacy library"
                                                  : ", NOT as the legacy library")
            << "\n";
}

// The cases (tools/volrover-goldens/make_inputs.py says what each covers). In the first three every
// automatic seed lies under a user seed; auto_seeds_sweep, blob_touching_shell and noisy_many_seeds
// (316 automatic seeds, 302 of them classified by the threshold sweep) exercise the sweep.
TEST(Segmentation, MatchesTheLegacyLibrary) {
  check_parity("two_blobs");
  check_parity("three_classes_noisy");
  check_parity("two_seeds_one_class");
  check_parity("auto_seeds_sweep");
  check_parity("blob_touching_shell");
  check_parity("noisy_many_seeds");
}

// ---- synthetic volumes ----------------------------------------------------------------------

struct bump {
  double x, y, z, sigma, height = 1.0;
};

// Gaussian bumps, plus an optional spherical shell (centre, radius, thickness).
grid field(int n, const std::vector<bump> &bumps, std::array<double, 5> shell = {0, 0, 0, 0, 0}) {
  grid g;
  g.x = g.y = g.z = n;
  g.v.resize(std::size_t(n) * n * n);
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double s = 0.0;
        for (const bump &b : bumps) {
          const double d2 = (i - b.x) * (i - b.x) + (j - b.y) * (j - b.y) + (k - b.z) * (k - b.z);
          s += b.height * std::exp(-d2 / (2 * b.sigma * b.sigma));
        }
        if (shell[3] > 0) {
          const double r =
              std::sqrt((i - shell[0]) * (i - shell[0]) + (j - shell[1]) * (j - shell[1]) +
                        (k - shell[2]) * (k - shell[2]));
          s += std::exp(-(r - shell[3]) * (r - shell[3]) / (2 * shell[4] * shell[4]));
        }
        g.v[g.at(i, j, k)] = float(s);
      }
  return g;
}

// Seeded smooth noise in [0, 1].
grid noise(int n, std::uint32_t seed, int blurs = 3) {
  grid g;
  g.x = g.y = g.z = n;
  g.v.resize(std::size_t(n) * n * n);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(0.0f, 1.0f);
  for (float &f : g.v)
    f = u(rng);
  std::vector<float> t(g.v.size());
  for (int b = 0; b < blurs; ++b) {
    for (int k = 0; k < n; ++k)
      for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
          float s = 0.0f;
          int c = 0;
          for (int dk = -1; dk <= 1; ++dk)
            for (int dj = -1; dj <= 1; ++dj)
              for (int di = -1; di <= 1; ++di) {
                const int a = i + di, bb = j + dj, cc = k + dk;
                if (a < 0 || bb < 0 || cc < 0 || a >= n || bb >= n || cc >= n)
                  continue;
                s += g.v[g.at(a, bb, cc)];
                ++c;
              }
          t[g.at(i, j, k)] = s / float(c);
        }
    g.v.swap(t);
  }
  const auto [lo, hi] = std::minmax_element(g.v.begin(), g.v.end());
  const float mn = *lo, mx = *hi;
  for (float &f : g.v)
    f = (f - mn) / (mx - mn);
  return g;
}

// The voxels as the segmentation rescales them (read_data's formula, in float).
std::vector<float> rescaled(const grid &g) {
  const auto [lo, hi] = std::minmax_element(g.v.begin(), g.v.end());
  const float mn = *lo, mx = *hi;
  std::vector<float> r(g.n());
  for (std::size_t i = 0; i < g.n(); ++i)
    r[i] = 255 * (g.v[i] - mn) / (mx - mn);
  return r;
}

std::size_t count_class(const cvc::seeded_segmentation &r, unsigned char c) {
  const unsigned char *lab = labels_of(r);
  std::size_t n = 0;
  for (std::size_t v = 0; v < r.labels.XDim() * r.labels.YDim() * r.labels.ZDim(); ++v)
    n += lab[v] == c;
  return n;
}

// ---- T2: the pieces -------------------------------------------------------------------------

TEST(Segmentation, RescaleAndClassVolumes) {
  // The thresholds as used, the class volumes in the input's type, and the labels' grid.
  cvc::app a;
  grid g = field(16, {{4, 8, 8, 2.0}, {12, 8, 8, 2.0}});
  cvc::volume vol = to_volume(a, g, cvc::Double);
  vol.boundingBox(cvc::bounding_box(-1, -2, -3, 1, 2, 3));
  cvc::seeded_segmentation_params p;
  p.t_low = 0.25;
  p.t_high = 0.75;
  const auto r = cvc::segment_seeded(vol, classes_of({{0, 4, 8, 8}, {1, 12, 8, 8}}), p);
  const std::vector<float> s = rescaled(g);
  const auto [lo, hi] = std::minmax_element(g.v.begin(), g.v.end());
  EXPECT_FLOAT_EQ(float(r.t_low_normalized), 255 * (0.25f - *lo) / (*hi - *lo));
  EXPECT_FLOAT_EQ(float(r.t_high_normalized), 255 * (0.75f - *lo) / (*hi - *lo));
  EXPECT_EQ(r.labels.voxelType(), cvc::UChar);
  EXPECT_EQ(r.labels.boundingBox(), vol.boundingBox());
  ASSERT_EQ(r.classes.size(), 2u);
  const unsigned char *lab = labels_of(r);
  for (std::size_t c = 0; c < 2; ++c) {
    EXPECT_EQ(r.classes[c].voxelType(), cvc::Double);
    EXPECT_EQ(r.classes[c].boundingBox(), vol.boundingBox());
    for (std::size_t v = 0; v < g.n(); ++v)
      EXPECT_EQ(r.classes[c](v), lab[v] == c ? vol(v) : 0.0) << v;
  }
  EXPECT_EQ(r.class_voxels[0], count_class(r, 0));
  EXPECT_EQ(r.class_voxels[1], count_class(r, 1));
  EXPECT_EQ(r.class_names, (std::vector<std::string>{"c0", "c1"}));
  // Without class volumes, the same labels.
  p.class_volumes = false;
  const auto r2 = cvc::segment_seeded(vol, classes_of({{0, 4, 8, 8}, {1, 12, 8, 8}}), p);
  EXPECT_TRUE(r2.classes.empty());
  EXPECT_TRUE(same_volume(r2.labels, r.labels));
}

TEST(Segmentation, ClassesOfTheVariableAndTimestep) {
  cvc::app a;
  const grid g = field(16, {{4, 8, 8, 2.0}, {12, 8, 8, 2.0}});
  cvc::point_classes pc;
  pc.add("left", 4, 8, 8);
  pc.add("other", 12, 8, 8, 1, 0); // variable 1: not this run's
  pc.add("right", 12, 8, 8);
  cvc::seeded_segmentation_params p;
  p.t_low = 0.25;
  p.t_high = 2.0; // no automatic seeds
  const auto r = cvc::segment_seeded(to_volume(a, g), pc, p);
  EXPECT_EQ(r.class_names, (std::vector<std::string>{"left", "right"}));
  EXPECT_EQ(labels_of(r)[g.at(12, 8, 8)], 1);
  p.variable = 1;
  const auto r1 = cvc::segment_seeded(to_volume(a, g), pc, p);
  EXPECT_EQ(r1.class_names, (std::vector<std::string>{"other"}));
  EXPECT_EQ(labels_of(r1)[g.at(12, 8, 8)], 0);
  EXPECT_EQ(labels_of(r1)[g.at(4, 8, 8)], cvc::seeded_segmentation::unlabelled);
}

// ---- T3: known shapes -----------------------------------------------------------------------

TEST(Segmentation, TwoBlobsGiveTwoLabels) {
  cvc::app a;
  const grid g = field(32, {{9, 16, 16, 2.5}, {23, 16, 16, 2.5}});
  cvc::seeded_segmentation_params p;
  p.t_low = 0.2; // data units: the bumps peak at 1
  p.t_high = 0.9;
  const auto r =
      cvc::segment_seeded(to_volume(a, g), classes_of({{0, 9, 16, 16}, {1, 23, 16, 16}}), p);
  const unsigned char *lab = labels_of(r);
  std::size_t left = 0, right = 0;
  for (int k = 0; k < 32; ++k)
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i) {
        const unsigned char l = lab[g.at(i, j, k)];
        if (l == 0) {
          EXPECT_LT(i, 16);
          ++left;
        } else if (l == 1) {
          EXPECT_GT(i, 16);
          ++right;
        }
        // The blobs' cores are labelled by their own blob's class. (Not the flanks: a step costs
        // 0.01 exp(0.12 |grad I|), |grad I| rescaled to 0..255, so where the gradient is near its
        // maximum, on each blob's steepest flank, the cost passes MAX_TIME and the front stops,
        // as the legacy library's did.)
        if (g.v[g.at(i, j, k)] > 0.85f)
          EXPECT_EQ(l, i < 16 ? 0 : 1) << i << " " << j << " " << k;
      }
  EXPECT_GE(left, 7u);
  EXPECT_EQ(left, right) << "the blobs are mirror images";
  EXPECT_EQ(r.automatic_seeds, 0u) << "the two peaks are the seeds";
  std::cout << "two blobs: " << left << " + " << right << " voxels\n";
}

TEST(Segmentation, OneSeedOfEachClassSplitsAtTheRidge) {
  // Two overlapping bumps: one connected feature with a saddle between the peaks. A seed of
  // each class on each peak: the fronts meet at the saddle.
  cvc::app a;
  const grid g = field(32, {{12, 16, 16, 3.0}, {20, 16, 16, 3.0}});
  cvc::seeded_segmentation_params p;
  p.t_low = 0.1;
  p.t_high = 2.0;
  p.automatic = cvc::seed_mode::none;
  const auto r =
      cvc::segment_seeded(to_volume(a, g), classes_of({{0, 12, 16, 16}, {1, 20, 16, 16}}), p);
  const unsigned char *lab = labels_of(r);
  for (int k = 0; k < 32; ++k)
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i) {
        const unsigned char l = lab[g.at(i, j, k)];
        if (l == 0)
          EXPECT_LE(i, 17);
        if (l == 1)
          EXPECT_GE(i, 15);
      }
  const std::size_t n0 = count_class(r, 0), n1 = count_class(r, 1);
  EXPECT_GT(n0, 100u);
  EXPECT_NEAR(double(n0), double(n1), 0.05 * double(n0 + n1));
  bool border_at_saddle = false;
  for (const auto &b : r.border)
    border_at_saddle |= b[0] >= 15 && b[0] <= 17 && b[1] == 16 && b[2] == 16;
  EXPECT_TRUE(border_at_saddle) << "the fronts met at the saddle";
}

TEST(Segmentation, AShellAroundABlobGivesTwoNestedLabels) {
  cvc::app a;
  const grid g = field(32, {{16, 16, 16, 2.0}}, {16, 16, 16, 11.0, 1.2});
  cvc::seeded_segmentation_params p;
  p.t_low = 0.3;
  p.t_high = 2.0;
  const auto r =
      cvc::segment_seeded(to_volume(a, g), classes_of({{0, 16, 16, 16}, {1, 27, 16, 16}}), p);
  const unsigned char *lab = labels_of(r);
  for (int k = 0; k < 32; ++k)
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i) {
        const double d =
            std::sqrt(double((i - 16) * (i - 16) + (j - 16) * (j - 16) + (k - 16) * (k - 16)));
        const unsigned char l = lab[g.at(i, j, k)];
        if (l == 0)
          EXPECT_LT(d, 6.0);
        if (l == 1)
          EXPECT_GT(d, 7.0);
        if (g.v[g.at(i, j, k)] > 0.5f) // the blob's core and the whole shell
          EXPECT_EQ(l, d < 6.0 ? 0 : 1) << i << " " << j << " " << k;
      }
}

// ---- T4: invariants -------------------------------------------------------------------------

// The 6-neighbours of voxel (i, j, k), clamped to the volume as the legacy update clamps them
// (at an edge the voxel itself stands in for the missing neighbour).
template <class F> void clamped_neighbours(const grid &g, int i, int j, int k, F f) {
  f(std::max(i - 1, 0), j, k);
  f(std::min(i + 1, g.x - 1), j, k);
  f(i, std::max(j - 1, 0), k);
  f(i, std::min(j + 1, g.y - 1), k);
  f(i, j, std::max(k - 1, 0));
  f(i, j, std::min(k + 1, g.z - 1));
}

// Checks the invariants on the segmentation of g; returns how many automatic seeds the sweep
// classified.
std::size_t check_invariants(const grid &g, const std::vector<std::array<int, 4>> &seeds,
                             const cvc::seeded_segmentation_params &p, bool automatic) {
  cvc::app a;
  const auto r = cvc::segment_seeded(to_volume(a, g), classes_of(seeds), p);
  const unsigned char *lab = labels_of(r);
  const std::vector<float> s = rescaled(g);
  const float tl = float(r.t_low_normalized);
  std::vector<char> is_border(g.n(), 0), is_seed(g.n(), 0);
  for (const auto &b : r.border)
    is_border[g.at(b[0], b[1], b[2])] = 1;
  for (const auto &q : seeds)
    is_seed[g.at(q[1], q[2], q[3])] = 1;
  std::size_t labelled = 0;
  for (int k = 0; k < g.z; ++k)
    for (int j = 0; j < g.y; ++j)
      for (int i = 0; i < g.x; ++i) {
        const std::size_t v = g.at(i, j, k);
        const unsigned char l = lab[v];
        if (is_border[v]) {
          // A front stopped here: next to (or, at an edge, at) a voxel below t_low or of
          // another class.
          EXPECT_NE(l, cvc::seeded_segmentation::unlabelled);
          bool why = false;
          clamped_neighbours(g, i, j, k, [&](int a2, int b2, int c2) {
            const std::size_t u = g.at(a2, b2, c2);
            why |= s[u] < tl || (lab[u] != l && lab[u] != cvc::seeded_segmentation::unlabelled);
          });
          EXPECT_TRUE(why) << "border voxel " << i << " " << j << " " << k;
        }
        if (l == cvc::seeded_segmentation::unlabelled)
          continue;
        ++labelled;
        if (automatic || is_seed[v])
          continue;
        // Fronts spread only through voxels whose neighbours are all at or above t_low: a
        // labelled voxel's front came from a neighbour of its class that is not a border voxel.
        if (!is_border[v])
          clamped_neighbours(g, i, j, k, [&](int a2, int b2, int c2) {
            EXPECT_GE(s[g.at(a2, b2, c2)], tl) << i << " " << j << " " << k;
          });
        bool parent = false;
        clamped_neighbours(g, i, j, k, [&](int a2, int b2, int c2) {
          const std::size_t u = g.at(a2, b2, c2);
          parent |= u != v && lab[u] == l && (!is_border[u] || is_seed[u]);
        });
        EXPECT_TRUE(parent) << "voxel " << i << " " << j << " " << k << " has no front behind it";
      }
  if (!automatic) {
    // Every labelled voxel connects to a seed of its class through voxels of its class.
    std::vector<char> reached(g.n(), 0);
    std::vector<std::size_t> queue;
    for (const auto &q : seeds) {
      const std::size_t v = g.at(q[1], q[2], q[3]);
      if (lab[v] == q[0] && !reached[v]) {
        reached[v] = 1;
        queue.push_back(v);
      }
    }
    for (std::size_t h = 0; h < queue.size(); ++h) {
      const std::size_t v = queue[h];
      const int i = int(v % g.x), j = int((v / g.x) % g.y), k = int(v / (std::size_t(g.x) * g.y));
      clamped_neighbours(g, i, j, k, [&](int a2, int b2, int c2) {
        const std::size_t u = g.at(a2, b2, c2);
        if (!reached[u] && lab[u] == lab[v]) {
          reached[u] = 1;
          queue.push_back(u);
        }
      });
    }
    EXPECT_EQ(queue.size(), labelled) << "labelled voxels not connected to a seed of their class";
  }
  return r.classified_seeds;
}

TEST(Segmentation, Invariants) {
  for (std::uint32_t seed = 1; seed <= 4; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    const grid g = noise(20, seed);
    std::mt19937 rng(seed * 7919u);
    std::uniform_int_distribution<int> at(0, 19);
    std::vector<std::array<int, 4>> seeds;
    for (int c = 0; c < 3; ++c)
      for (int m = 0; m < 2; ++m)
        seeds.push_back({c, at(rng), at(rng), at(rng)});
    cvc::seeded_segmentation_params p;
    p.normalized_thresholds = true;
    p.t_low = 60;
    p.t_high = 170;
    p.automatic = cvc::seed_mode::none;
    check_invariants(g, seeds, p, false);
    p.automatic = cvc::seed_mode::local_maxima;
    check_invariants(g, seeds, p, true);
  }
}

// make_inputs.py's volume(): Gaussian blobs over a background, plus its generator's uniform noise,
// computed in double and stored as float32, as the script does.
grid make_inputs_volume(int n, const std::vector<bump> &blobs, double background, double noise,
                        std::uint32_t seed) {
  grid g;
  g.x = g.y = g.z = n;
  g.v.reserve(std::size_t(n) * n * n);
  std::uint32_t state = seed;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double v = background;
        for (const bump &b : blobs) {
          const double d2 = (i - b.x) * (i - b.x) + (j - b.y) * (j - b.y) + (k - b.z) * (k - b.z);
          v += b.height * std::exp(-d2 / (2.0 * b.sigma * b.sigma));
        }
        if (noise != 0.0) {
          state = 1664525u * state + 1013904223u;
          v += noise * (double(state) / 4294967296.0 - 0.5);
        }
        g.v.push_back(float(std::max(v, 0.0)));
      }
  return g;
}

TEST(Segmentation, RunsWhereTheLegacyLibraryCrashed) {
  // noisy_many_seeds with noise 0.5 instead of 0.45: a seed the sweep classifies has nowhere to
  // go, and the legacy march then pops an empty heap (reading before its arrays; the legacy
  // executable crashed). The port ends that march and goes on.
  const grid g = make_inputs_volume(24, {{12, 12, 12, 30.0, 1.0}}, 0.02, 0.5, 16);
  cvc::seeded_segmentation_params p;
  p.normalized_thresholds = true;
  p.t_low = 60;
  p.t_high = 140;
  const std::vector<std::array<int, 4>> seeds{{0, 7, 12, 12}, {1, 17, 12, 12}, {2, 12, 12, 18}};
  EXPECT_GT(check_invariants(g, seeds, p, true), 200u);
}

TEST(Segmentation, LabelsNeverCrossAWallBelowTLow) {
  for (const cvc::seed_mode m : {cvc::seed_mode::none, cvc::seed_mode::local_maxima}) {
    grid g = noise(20, 11);
    for (int k = 0; k < 20; ++k)
      for (int j = 0; j < 20; ++j)
        g.v[g.at(10, j, k)] = 0.0f; // the wall
    cvc::app a;
    cvc::seeded_segmentation_params p;
    p.normalized_thresholds = true;
    p.t_low = 40;
    p.t_high = 150;
    p.automatic = m;
    const auto r = cvc::segment_seeded(
        to_volume(a, g), classes_of({{0, 3, 5, 5}, {1, 6, 15, 12}, {0, 2, 17, 3}}), p);
    const unsigned char *lab = labels_of(r);
    std::size_t left = 0;
    for (int k = 0; k < 20; ++k)
      for (int j = 0; j < 20; ++j)
        for (int i = 0; i < 20; ++i) {
          const unsigned char l = lab[g.at(i, j, k)];
          if (i > 10)
            EXPECT_EQ(l, cvc::seeded_segmentation::unlabelled) << i << " " << j << " " << k;
          if (i == 10 && l != cvc::seeded_segmentation::unlabelled) {
            const auto &b = r.border;
            EXPECT_NE(std::find(b.begin(), b.end(), std::array<int, 3>{i, j, k}), b.end())
                << "a wall voxel is labelled only as a border voxel";
          }
          left += i < 10 && l != cvc::seeded_segmentation::unlabelled;
        }
    EXPECT_GT(left, 100u);
  }
}

// ---- T5: bad input ----------------------------------------------------------------------------

std::string error_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const cvc::segmentation_error &e) {
    return e.what();
  } catch (const std::exception &e) {
    return std::string("wrong exception type: ") + e.what();
  }
  return "no exception";
}

TEST(Segmentation, BadInputThrowsWithAUsefulMessage) {
  cvc::app a;
  const grid g = field(16, {{8, 8, 8, 3.0}});
  const cvc::volume vol = to_volume(a, g);
  cvc::seeded_segmentation_params p;
  p.t_low = 0.2;
  p.t_high = 0.9;
  auto msg = [&](const cvc::point_classes &pc, const cvc::seeded_segmentation_params &q) {
    return error_of([&] { (void)cvc::segment_seeded(vol, pc, q); });
  };
  // No seeds at all, and none of this variable.
  EXPECT_NE(msg(cvc::point_classes{}, p).find("no seeds"), std::string::npos);
  {
    cvc::point_classes pc;
    pc.add("x", 1, 1, 1, 0, 2);
    EXPECT_NE(msg(pc, p).find("no seeds"), std::string::npos);
  }
  // A seed outside the volume.
  EXPECT_NE(msg(classes_of({{0, 8, 8, 8}, {1, 3, 16, 2}}), p)
                .find("seed (3, 16, 2) of class 'c1' is outside the 16x16x16 volume"),
            std::string::npos);
  // Bad parameters.
  {
    cvc::seeded_segmentation_params q = p;
    q.gamma = 0.5;
    EXPECT_NE(msg(classes_of({{0, 8, 8, 8}}), q).find("gamma"), std::string::npos);
    q = p;
    q.t_low = std::numeric_limits<double>::infinity();
    EXPECT_NE(msg(classes_of({{0, 8, 8, 8}}), q).find("t_low"), std::string::npos);
    q = p;
    q.t_high = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(msg(classes_of({{0, 8, 8, 8}}), q).find("t_high"), std::string::npos);
  }
  // More than 254 classes.
  {
    cvc::point_classes pc;
    for (int c = 0; c < 255; ++c)
      pc.add("k" + std::to_string(c), c % 16, (c / 16) % 16, 0);
    EXPECT_NE(msg(pc, p).find("more than 254"), std::string::npos);
  }
  // A constant volume.
  {
    grid c = g;
    std::fill(c.v.begin(), c.v.end(), 3.0f);
    const std::string m = error_of(
        [&] { (void)cvc::segment_seeded(to_volume(a, c), classes_of({{0, 8, 8, 8}}), p); });
    EXPECT_NE(m.find("constant"), std::string::npos) << m;
  }
  // A NaN voxel, and an infinite one.
  for (const float bad :
       {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    grid c = g;
    c.v[c.at(5, 6, 7)] = bad;
    const std::string m = error_of(
        [&] { (void)cvc::segment_seeded(to_volume(a, c), classes_of({{0, 8, 8, 8}}), p); });
    EXPECT_NE(m.find("voxel (5, 6, 7) is"), std::string::npos) << m;
  }
}

TEST(Segmentation, OddSeedsAreWarnings) {
  cvc::app a;
  const grid g = field(16, {{8, 8, 8, 3.0}});
  const cvc::volume vol = to_volume(a, g);
  cvc::seeded_segmentation_params p;
  p.t_low = 0.2;
  p.t_high = 2.0;
  // A seed below t_low: it labels itself and the neighbours its front stops at.
  const auto r = cvc::segment_seeded(vol, classes_of({{0, 8, 8, 8}, {1, 0, 0, 0}}), p);
  ASSERT_FALSE(r.warnings.empty());
  EXPECT_NE(r.warnings.back().find("seed (0, 0, 0) of class 'c1' is below t_low"),
            std::string::npos)
      << r.warnings.back();
  EXPECT_EQ(labels_of(r)[g.at(0, 0, 0)], 1);
  EXPECT_LE(count_class(r, 1), 4u);
  // A class with no points, and a voxel that is a point of two classes.
  cvc::point_classes pc = classes_of({{0, 8, 8, 8}, {2, 9, 8, 8}});
  pc.add("c1", 8, 8, 8);
  const auto r2 = cvc::segment_seeded(vol, pc, p);
  std::string all;
  for (const std::string &w : r2.warnings)
    all += w + "\n";
  EXPECT_NE(all.find("voxel (8, 8, 8) is a point of classes 'c0' and 'c1'; 'c1' has it"),
            std::string::npos)
      << all;
  cvc::point_classes empty = classes_of({{0, 8, 8, 8}});
  cvc::point_class none;
  none.name = "nobody";
  empty.set_class(none);
  const auto r3 = cvc::segment_seeded(vol, empty, p);
  ASSERT_FALSE(r3.warnings.empty());
  EXPECT_NE(r3.warnings.front().find("'nobody' has no points"), std::string::npos);
  EXPECT_EQ(r3.class_voxels.size(), 2u);
  EXPECT_EQ(r3.class_voxels[1], 0u);
}

TEST(Segmentation, EveryVoxelTypeGivesTheSameLabels) {
  cvc::app a;
  grid g = noise(16, 5);
  for (float &f : g.v)
    f = std::round(f * 100.0f); // integers 0..100, exact in every type
  cvc::seeded_segmentation_params p;
  p.t_low = 30;
  p.t_high = 80;
  const auto seeds = classes_of({{0, 3, 3, 3}, {1, 12, 12, 12}, {2, 3, 12, 8}});
  const auto ref = cvc::segment_seeded(to_volume(a, g), seeds, p);
  for (const cvc::data_type t : {cvc::UChar, cvc::UShort, cvc::UInt, cvc::Double, cvc::UInt64,
                                 cvc::Char, cvc::Int, cvc::Int64}) {
    const auto r = cvc::segment_seeded(to_volume(a, g, t), seeds, p);
    EXPECT_TRUE(same_volume(r.labels, ref.labels)) << cvc::data_type_strings[t];
    EXPECT_EQ(r.border, ref.border) << cvc::data_type_strings[t];
    ASSERT_EQ(r.classes.size(), 3u);
    EXPECT_EQ(r.classes[1].voxelType(), t);
  }
}

// ---- T6: no singletons ------------------------------------------------------------------------

TEST(Segmentation, CallsAreIndependent) {
  cvc::app a;
  const legacy_case c = load_case("three_classes_noisy");
  ASSERT_FALSE(c.g.v.empty());
  const cvc::volume va = to_volume(a, c.g);
  const cvc::point_classes sa = classes_of(c.seeds);
  const cvc::seeded_segmentation_params pa = legacy_params(c);
  const grid gb = noise(20, 3);
  const cvc::volume vb = to_volume(a, gb);
  const cvc::point_classes sb = classes_of({{0, 2, 2, 2}, {1, 17, 17, 17}});
  cvc::seeded_segmentation_params pb;
  pb.t_low = 0.3;
  pb.t_high = 0.6;
  pb.gamma = -0.3;
  const auto ra = cvc::segment_seeded(va, sa, pa);
  const auto rb = cvc::segment_seeded(vb, sb, pb);
  // B after A, and A after B, equal each alone (the legacy globals carried over between runs).
  EXPECT_TRUE(same(cvc::segment_seeded(vb, sb, pb), rb));
  EXPECT_TRUE(same(cvc::segment_seeded(va, sa, pa), ra));
  // Two at once on two threads, with different parameters.
  std::optional<cvc::seeded_segmentation> ta, tb;
  std::thread t1([&] { ta.emplace(cvc::segment_seeded(va, sa, pa)); });
  std::thread t2([&] { tb.emplace(cvc::segment_seeded(vb, sb, pb)); });
  t1.join();
  t2.join();
  ASSERT_TRUE(ta && tb);
  EXPECT_TRUE(same(*ta, ra));
  EXPECT_TRUE(same(*tb, rb));
}

TEST(Segmentation, TwoAppsInOneProcess) {
  // Each app's volumes, segmented at once, give what each gives alone, and the results belong to
  // the input's app.
  cvc::app a1, a2;
  const grid g1 = noise(18, 21), g2 = field(18, {{5, 9, 9, 2.0}, {13, 9, 9, 2.0}});
  cvc::seeded_segmentation_params p1, p2;
  p1.t_low = 0.35;
  p1.t_high = 0.7;
  p2.t_low = 0.2;
  p2.t_high = 0.9;
  const auto s1 = classes_of({{0, 1, 1, 1}, {1, 16, 16, 16}}),
             s2 = classes_of({{0, 5, 9, 9}, {1, 13, 9, 9}});
  const auto alone1 = cvc::segment_seeded(to_volume(a1, g1), s1, p1);
  const auto alone2 = cvc::segment_seeded(to_volume(a2, g2), s2, p2);
  std::optional<cvc::seeded_segmentation> r1, r2;
  std::thread t1([&] { r1.emplace(cvc::segment_seeded(to_volume(a1, g1), s1, p1)); });
  std::thread t2([&] { r2.emplace(cvc::segment_seeded(to_volume(a2, g2), s2, p2)); });
  t1.join();
  t2.join();
  ASSERT_TRUE(r1 && r2);
  EXPECT_TRUE(same(*r1, alone1));
  EXPECT_TRUE(same(*r2, alone2));
  EXPECT_EQ(&r1->labels.ctx(), &a1);
  EXPECT_EQ(&r2->labels.ctx(), &a2);
  EXPECT_EQ(&r2->classes.at(1).ctx(), &a2);
}

// ---- T7: resumable ----------------------------------------------------------------------------

TEST(Segmentation, StepperIsResumable) {
  cvc::app a;
  const legacy_case c = load_case("auto_seeds_sweep"); // every phase has work, the sweep too
  ASSERT_FALSE(c.g.v.empty());
  const cvc::volume vol = to_volume(a, c.g);
  const cvc::point_classes seeds = classes_of(c.seeds);
  const cvc::seeded_segmentation_params p = legacy_params(c);
  cvc::test::t7_subject<cvc::segment_stepper, cvc::seeded_segmentation> s;
  s.name = "genseg auto_seeds_sweep 28^3";
  s.make = [&] { return std::make_unique<cvc::segment_stepper>(vol, seeds, p); };
  s.take = [](cvc::segment_stepper &st) { return st.take(); };
  s.equal = same;
  s.blocking = [&] { return cvc::segment_seeded(vol, seeds, p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase :
       {"checking", "normalising", "seeding", "classifying", "marching", "writing"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST(Segmentation, AStepPopsABoundedNumberOfVoxels) {
  // However large the volume, a step of budget b does about b units: a 64^3 run takes many more
  // steps at budget 64 than a 16^3 one, each about as short.
  cvc::app a;
  for (const int n : {16, 48}) {
    const grid g = noise(n, 9, 2);
    cvc::seeded_segmentation_params p;
    p.normalized_thresholds = true;
    p.t_low = 50;
    p.t_high = 200;
    cvc::segment_stepper st(to_volume(a, g), classes_of({{0, 1, 1, 1}, {1, n - 2, n - 2, n - 2}}),
                            p);
    std::size_t steps = 0;
    while (!st.step(64))
      ++steps;
    std::cout << n << "^3: " << steps << " steps of 64 units\n";
    EXPECT_GT(steps, std::size_t(n) * n * n * 10 / 64) << "at least ten units a voxel";
  }
}

TEST(Segmentation, ACancelledStepperGivesNothing) {
  cvc::app a;
  const grid g = noise(16, 2);
  cvc::seeded_segmentation_params p;
  p.t_low = 0.2;
  p.t_high = 0.7;
  auto job = cvc::segment_seeded_job(to_volume(a, g), classes_of({{0, 1, 1, 1}}), p);
  job->step(100);
  EXPECT_FALSE(job->done());
  job->cancel();
  EXPECT_TRUE(job->step(100));
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  // A failure: the job ends "failed" with the stepper's message.
  auto bad = cvc::segment_seeded_job(to_volume(a, g), cvc::point_classes{}, p);
  EXPECT_TRUE(bad->step(0));
  EXPECT_TRUE(bad->failed());
  EXPECT_NE(bad->error().find("no seeds"), std::string::npos);
  EXPECT_THROW((void)bad->result(), cvc::segmentation_error);
}

// ---- T8: the parameter table ------------------------------------------------------------------

TEST(Segmentation, ParameterTable) {
  cvc::app a;
  cvc::seeded_segmentation_params p;
  p.t_low = 40.5;
  p.t_high = 200;
  p.normalized_thresholds = true;
  p.gamma = -0.25;
  p.automatic = cvc::seed_mode::none;
  p.variable = 2;
  p.timestep = 3;
  p.class_volumes = false;
  EXPECT_TRUE(cvc::params_validate(p).empty());
  std::size_t fields = 0;
  cvc::for_each_param<cvc::seeded_segmentation_params>([&](const auto &) { ++fields; });
  EXPECT_EQ(fields, 15u);
  cvc::state &node = a.root()("doc.grid.segment");
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("t_low").value(), "40.5");
  EXPECT_EQ(node("automatic").value(), "none");
  EXPECT_EQ(node("automatic").property("choices"), "none,local_maxima,critical_points");
  EXPECT_EQ(node("gamma").property("min"), "-1");
  const auto back = cvc::params_from_state<cvc::seeded_segmentation_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.t_low, p.t_low);
  EXPECT_EQ(back.value.t_high, p.t_high);
  EXPECT_EQ(back.value.normalized_thresholds, true);
  EXPECT_EQ(back.value.gamma, p.gamma);
  EXPECT_EQ(back.value.automatic, cvc::seed_mode::none);
  EXPECT_EQ(back.value.variable, 2);
  EXPECT_EQ(back.value.timestep, 3);
  EXPECT_EQ(back.value.class_volumes, false);
  // Defaults seeded, bad values reported.
  cvc::state &fresh = a.root()("doc.grid2.segment");
  const auto seeded = cvc::params_from_state<cvc::seeded_segmentation_params>(fresh);
  ASSERT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("gamma").value(), "-0.12");
  EXPECT_EQ(fresh("automatic").value(), "local_maxima");
  fresh("automatic").value("watershed");
  fresh("gamma").value("0.5");
  const auto bad = cvc::params_from_state<cvc::seeded_segmentation_params>(fresh);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("automatic"), std::string::npos) << bad.message();
  EXPECT_NE(bad.message().find("gamma"), std::string::npos) << bad.message();
}

// ---- Real maps (opt-in) ---------------------------------------------------------------------
//
// The group's own cryo-EM maps, the kind the legacy library was written for, are not in the
// repository. Set CVC_VOLROVER_DATA to the directory that holds them (its Ribosome/ and
// Zeyun-Segmented-Maps/ folders) and these run; without it they are skipped. The expected labels
// and border voxels are the legacy library's: tools/volrover-goldens' genseg_legacy on the same
// map, seeds and thresholds (2026-11-16), fingerprinted here since the maps cannot be copied in.

// FNV-1a, 64 bits.
std::uint64_t fnv1a(const unsigned char *p, std::size_t n,
                    std::uint64_t h = 0xcbf29ce484222325ull) {
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 0x100000001b3ull;
  }
  return h;
}

// The labels, one byte a voxel, x fastest.
std::uint64_t labels_fingerprint(const cvc::seeded_segmentation &r) {
  return fnv1a(labels_of(r), r.labels.XDim() * r.labels.YDim() * r.labels.ZDim());
}

// The border voxels in their order, each as three little-endian uint32 (i, j, k).
std::uint64_t border_fingerprint(const std::vector<std::array<int, 3>> &border) {
  std::uint64_t h = 0xcbf29ce484222325ull;
  for (const auto &b : border)
    for (int c : b) {
      const auto u = static_cast<std::uint32_t>(c);
      const unsigned char le[4] = {
          static_cast<unsigned char>(u), static_cast<unsigned char>(u >> 8),
          static_cast<unsigned char>(u >> 16), static_cast<unsigned char>(u >> 24)};
      h = fnv1a(le, 4, h);
    }
  return h;
}

// `rel` under CVC_VOLROVER_DATA, or "" with the reason to skip in `why`.
std::string real_map(const std::string &rel, std::string &why) {
  const char *dir = std::getenv("CVC_VOLROVER_DATA");
  if (!dir || !*dir) {
    why = "set CVC_VOLROVER_DATA to the group's map directory to run this";
    return {};
  }
  const std::string path = std::string(dir) + "/" + rel;
  if (!std::ifstream(path, std::ios::binary)) {
    why = path + " is not there";
    return {};
  }
  return path;
}

std::size_t labelled_voxels(const cvc::seeded_segmentation &r) {
  std::size_t n = 0;
  for (std::size_t c : r.class_voxels)
    n += c;
  return n;
}

// The ribosome EMD-1056 (130^3) with one seed in each subunit, at the thresholds the 30S/50S
// split was published with (its images are named _110_180): the legacy library's labels and
// border, and close to that split (Dice 0.901 and 0.929, measured).
TEST(SegmentationRealMaps, RibosomeAsTheLegacyLibraryAndThePublishedSplit) {
  std::string why;
  const std::string map = real_map("Ribosome/FromEM/emd_1056.rawiv", why);
  const std::string s30 =
      map.empty() ? "" : real_map("Ribosome/FromEM/emd_1056_subunit_30s.rawiv", why);
  const std::string s50 =
      s30.empty() ? "" : real_map("Ribosome/FromEM/emd_1056_subunit_50s.rawiv", why);
  if (s50.empty())
    GTEST_SKIP() << why;
  cvc::app a;
  const cvc::volume vol(a, map);
  cvc::seeded_segmentation_params p;
  p.t_low = 110;
  p.t_high = 180;
  p.normalized_thresholds = true;
  p.class_volumes = false;
  const cvc::seeded_segmentation r =
      cvc::segment_seeded(vol, classes_of({{0, 124, 65, 74}, {1, 78, 79, 39}}), p);
  EXPECT_EQ(r.class_voxels, (std::vector<std::size_t>{54585, 75792}));
  EXPECT_EQ(r.border.size(), 58032u);
  EXPECT_EQ(r.automatic_seeds, 415u);
  EXPECT_EQ(r.classified_seeds, 415u);
  EXPECT_EQ(labels_fingerprint(r), 0x89fc5160f3adf5f8ull) << "labels differ from the legacy run";
  EXPECT_EQ(border_fingerprint(r.border), 0xd7f70063fc5ee087ull)
      << "border voxels differ from the legacy run";

  // Against the published split: the subunits' densities, zero outside each.
  const cvc::volume sub[2] = {cvc::volume(a, s30), cvc::volume(a, s50)};
  const unsigned char *lab = labels_of(r);
  const std::size_t n = vol.XDim() * vol.YDim() * vol.ZDim();
  for (int c = 0; c < 2; ++c) {
    ASSERT_EQ(sub[c].XDim() * sub[c].YDim() * sub[c].ZDim(), n);
    std::size_t ref = 0, got = 0, both = 0;
    for (std::size_t v = 0; v < n; ++v) {
      const bool in_ref = sub[c](v) != 0.0, in_got = lab[v] == c;
      ref += in_ref;
      got += in_got;
      both += in_ref && in_got;
    }
    const double dice = 2.0 * double(both) / double(ref + got);
    std::cout << (c == 0 ? "30S" : "50S") << ": " << ref << " voxels published, " << got
              << " labelled, Dice " << dice << "\n";
    EXPECT_GE(dice, c == 0 ? 0.89 : 0.92);
  }
}

// GroEL at 6 A (W. Chiu's map, 128^3), one seed in each of its 14 subunits (the densest voxel
// of each, by the subunits the legacy virus pipeline cut it into), tlow 115 / thigh 180: the
// legacy library's labels and border. Its 409 automatic seeds all go through the threshold sweep,
// which skips the searches that would find nothing (genseg.cpp): this pins that the labels stay
// the legacy ones. (Against those subunits, which 7-fold symmetry cut, the fronts do poorly: a
// seed on a density peak is walled in, as on the capsid below.)
TEST(SegmentationRealMaps, GroELAsTheLegacyLibrary) {
  std::string why;
  const std::string map = real_map("W_Chiu/GroEL-6A.rawiv", why);
  if (map.empty())
    GTEST_SKIP() << why;
  cvc::app a;
  const cvc::volume vol(a, map);
  cvc::seeded_segmentation_params p;
  p.t_low = 115;
  p.t_high = 180;
  p.normalized_thresholds = true;
  p.class_volumes = false;
  const cvc::seeded_segmentation r = cvc::segment_seeded(vol,
                                                         classes_of({{0, 49, 43, 59},
                                                                     {1, 37, 65, 58},
                                                                     {2, 67, 88, 44},
                                                                     {3, 68, 89, 59},
                                                                     {4, 86, 77, 59},
                                                                     {5, 76, 47, 54},
                                                                     {6, 71, 39, 59},
                                                                     {7, 47, 84, 67},
                                                                     {8, 69, 90, 67},
                                                                     {9, 75, 81, 73},
                                                                     {10, 89, 52, 67},
                                                                     {11, 73, 47, 84},
                                                                     {12, 48, 43, 67},
                                                                     {13, 47, 46, 83}}),
                                                         p);
  EXPECT_EQ(r.automatic_seeds, 409u);
  EXPECT_EQ(r.classified_seeds, 409u);
  EXPECT_EQ(labelled_voxels(r), 114692u);
  EXPECT_EQ(r.border.size(), 55652u);
  EXPECT_EQ(labels_fingerprint(r), 0x037d421d1363a2deull) << "labels differ from the legacy run";
  EXPECT_EQ(border_fingerprint(r.border), 0xfbc92b101ebdebbdull)
      << "border voxels differ from the legacy run";
}

// The capsid layer of EMD-1011 (256^3) cut into its 252 capsomers, one seed in each: the
// densest voxel of the capsomer whose 3x3x3 neighbourhood lies inside it, by the capsomer index
// the legacy virus pipeline wrote. With 252 classes and no automatic seeds, the legacy library's
// labels and border. Against that index, the default gamma traps most fronts (18.6% of the
// index in the same capsomer, measured): a seed on a density peak sits behind steep flanks while
// flat ground floods first. At gamma -0.04 the fronts share the layer out (48.0%, 73% of what
// the run labels; the index also covers voxels outside the layer the run is given).
TEST(SegmentationRealMaps, VirusCapsidInto252CapsomersAsTheLegacyLibrary) {
  std::string why;
  const std::string capsid =
      real_map("Zeyun-Segmented-Maps/emd_1011/Subunit/emd_1011_capsid.rawiv", why);
  const std::string index =
      capsid.empty()
          ? ""
          : real_map("Zeyun-Segmented-Maps/emd_1011/Subunit/emd_1011_capsid_index.rawiv", why);
  if (index.empty())
    GTEST_SKIP() << why;
  cvc::app a;
  const cvc::volume vol(a, capsid);
  const cvc::volume idx(a, index);
  const int nx = int(vol.XDim()), ny = int(vol.YDim()), nz = int(vol.ZDim());
  const std::size_t n = std::size_t(nx) * ny * nz;
  ASSERT_EQ(idx.XDim() * idx.YDim() * idx.ZDim(), n);
  constexpr int background = 1000; // the index marks voxels outside every capsomer 10000
  std::vector<int> region(n);
  for (std::size_t v = 0; v < n; ++v)
    region[v] = int(idx(v));
  std::map<int, std::size_t> seed_of; // capsomer -> its seed voxel
  const std::size_t sy = std::size_t(nx), sz = std::size_t(nx) * ny;
  for (int k = 1; k < nz - 1; ++k)
    for (int j = 1; j < ny - 1; ++j)
      for (int i = 1; i < nx - 1; ++i) {
        const std::size_t v = k * sz + j * sy + i;
        const int c = region[v];
        if (c >= background)
          continue;
        auto it = seed_of.find(c);
        if (it != seed_of.end() && vol(v) <= vol(it->second))
          continue;
        bool inside = true;
        for (int dk = -1; dk <= 1 && inside; ++dk)
          for (int dj = -1; dj <= 1 && inside; ++dj)
            for (int di = -1; di <= 1 && inside; ++di)
              inside = region[v + dk * sz + dj * sy + di] == c;
        if (inside)
          seed_of[c] = v;
      }
  ASSERT_EQ(seed_of.size(), 252u);
  std::vector<std::array<int, 4>> seeds;
  std::vector<int> capsomer_of_class;
  for (const auto &[c, v] : seed_of) {
    seeds.push_back({int(seeds.size()), int(v % sy), int((v / sy) % ny), int(v / sz)});
    capsomer_of_class.push_back(c);
  }
  const cvc::point_classes pc = classes_of(seeds);

  cvc::seeded_segmentation_params p;
  p.t_low = 1; // the layer is zero outside the capsid
  p.t_high = 255;
  p.normalized_thresholds = true;
  p.class_volumes = false;
  const cvc::seeded_segmentation r = cvc::segment_seeded(vol, pc, p);
  EXPECT_EQ(r.automatic_seeds, 0u);
  EXPECT_EQ(labelled_voxels(r), 1981954u);
  EXPECT_EQ(r.border.size(), 407841u);
  EXPECT_EQ(labels_fingerprint(r), 0x1570c18876a8bdbcull) << "labels differ from the legacy run";
  EXPECT_EQ(border_fingerprint(r.border), 0x4bfe23ecde106732ull)
      << "border voxels differ from the legacy run";

  // The share of the index's voxels that land in their own capsomer, and of the labelled ones.
  const auto agreement = [&](const cvc::seeded_segmentation &s, double &of_labelled) {
    const unsigned char *lab = labels_of(s);
    std::size_t in_index = 0, labelled = 0, same = 0;
    for (std::size_t v = 0; v < n; ++v) {
      if (region[v] >= background)
        continue;
      ++in_index;
      if (lab[v] == cvc::seeded_segmentation::unlabelled)
        continue;
      ++labelled;
      same += capsomer_of_class[lab[v]] == region[v];
    }
    of_labelled = double(same) / double(labelled);
    return double(same) / double(in_index);
  };
  double default_of_labelled = 0, gentle_of_labelled = 0;
  const double at_default = agreement(r, default_of_labelled);
  p.gamma = -0.04;
  p.automatic = cvc::seed_mode::none;
  const double gentle = agreement(cvc::segment_seeded(vol, pc, p), gentle_of_labelled);
  std::cout << "capsomers as the index: gamma -0.12 " << at_default << " (" << default_of_labelled
            << " of the labelled), gamma -0.04 " << gentle << " (" << gentle_of_labelled << ")\n";
  EXPECT_GE(gentle, 0.47);
  EXPECT_GE(gentle_of_labelled, 0.72);
  EXPECT_GT(gentle, at_default);
}

} // namespace

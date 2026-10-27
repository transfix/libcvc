/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::tf::model (presets, normalize, sampling, bake, the JSON state string, lowering, CSV) and
// the ColorTable2 .vinay / .ctbl codecs.

#include <algorithm>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/tf/model.h>
#include <cvc/tf/vinay.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

using namespace cvc::tf;

namespace {

color_node cn(double at, float r, float g, float b) {
  color_node n;
  n.at = at;
  n.r = r;
  n.g = g;
  n.b = b;
  return n;
}

alpha_node an(double at, float a) {
  alpha_node n;
  n.at = at;
  n.a = a;
  return n;
}

isocontour iso_at(const std::string &id, double at) {
  isocontour iso;
  iso.id = id;
  iso.at = at;
  return iso;
}

void expect_color(const color_node &n, double at, float r, float g, float b) {
  EXPECT_DOUBLE_EQ(n.at, at);
  EXPECT_FLOAT_EQ(n.r, r);
  EXPECT_FLOAT_EQ(n.g, g);
  EXPECT_FLOAT_EQ(n.b, b);
}

void expect_alpha(const alpha_node &n, double at, float a) {
  EXPECT_DOUBLE_EQ(n.at, at);
  EXPECT_FLOAT_EQ(n.a, a);
}

// An independent piecewise-linear reference over (x, y) knots, clamped at the ends.
double ref(const std::vector<std::pair<double, double>> &k, double t) {
  if (t <= k.front().first)
    return k.front().second;
  for (std::size_t i = 1; i < k.size(); ++i)
    if (t < k[i].first) {
      const double u = (t - k[i - 1].first) / (k[i].first - k[i - 1].first);
      return k[i - 1].second + (k[i].second - k[i - 1].second) * u;
    }
  return k.back().second;
}

std::uint8_t to_byte(double v) { return static_cast<std::uint8_t>(std::floor(v * 255.0 + 0.5)); }

// ColorTable2's default transfer function, as write_transfer_function wrote it (colortable2.md
// §5.2) -- endpoints first, six significant digits.
const char kDefaultVinay[] = "Anthony and Vinay are Great.\n"
                             "Alphamap\n"
                             "Number of nodes\n"
                             "4\n"
                             "Position and opacity\n"
                             "0 0\n"
                             "1 1\n"
                             "0.25 0.75\n"
                             "0.75 0.25\n"
                             "ColorMap\n"
                             "Number of nodes\n"
                             "4\n"
                             "Position and RGB\n"
                             "0 0 0 0\n"
                             "1 0 1 0\n"
                             "0.5 1 0 0\n"
                             "0.75 1 1 0\n"
                             "IsocontourMap\n"
                             "Number of nodes\n"
                             "0\n"
                             "Position\n";

const char kDefaultJson[] =
    "{\"color\":[[0,0,0,0],[0.5,1,0,0],[0.75,1,1,0],[1,0,1,0]],"
    "\"alpha\":[[0,0],[0.25,0.75],[0.75,0.25],[1,1]],\"isocontours\":[],\"domain\":\"auto\","
    "\"opacity_cubed\":false,\"resolution\":256}";

model parsed(const std::string &s) {
  model m;
  std::string err;
  EXPECT_TRUE(from_string(s, m, &err)) << err << " in " << s;
  return m;
}

std::string vinay_error_text(const std::string &text) {
  try {
    read_vinay_string(text);
  } catch (const vinay_error &e) {
    return e.what();
  }
  return std::string();
}

std::vector<std::string> split_lines(const std::string &s) {
  std::vector<std::string> out;
  std::istringstream in(s);
  for (std::string l; std::getline(in, l);)
    out.push_back(l);
  return out;
}

} // namespace

// --- model basics ---------------------------------------------------------------------------

TEST(TfModel, DefaultModelIsColorTable2s) {
  const model m = model::default_model();
  ASSERT_EQ(m.color.size(), 4u);
  expect_color(m.color[0], 0.0, 0, 0, 0);
  expect_color(m.color[1], 0.5, 1, 0, 0);
  expect_color(m.color[2], 0.75, 1, 1, 0);
  expect_color(m.color[3], 1.0, 0, 1, 0);
  ASSERT_EQ(m.alpha.size(), 4u);
  expect_alpha(m.alpha[0], 0.0, 0.f);
  expect_alpha(m.alpha[1], 0.25, 0.75f);
  expect_alpha(m.alpha[2], 0.75, 0.25f);
  expect_alpha(m.alpha[3], 1.0, 1.f);
  EXPECT_TRUE(m.isocontours.empty());
  EXPECT_TRUE(m.domain.automatic);
  EXPECT_FALSE(m.opacity_cubed);
  EXPECT_EQ(m.resolution, 256);
  model n = m;
  n.normalize();
  EXPECT_EQ(n, m);
}

TEST(TfModel, PresetsAreNormalizedAndDistinct) {
  const std::vector<std::string> &names = model::preset_names();
  ASSERT_EQ(names.size(), 7u);
  EXPECT_EQ(names.front(), "default");
  for (const std::string &name : names) {
    SCOPED_TRACE(name);
    const model m = model::preset(name);
    model n = m;
    n.normalize();
    EXPECT_EQ(n, m);
    ASSERT_GE(m.color.size(), 2u);
    ASSERT_GE(m.alpha.size(), 2u);
    EXPECT_EQ(m.color.front().at, 0.0);
    EXPECT_EQ(m.color.back().at, 1.0);
    EXPECT_EQ(m.alpha.front().at, 0.0);
    EXPECT_EQ(m.alpha.back().at, 1.0);
    EXPECT_EQ(transparent(m), name == "isolines");
    for (const std::string &other : names) {
      if (other != name) {
        EXPECT_NE(m, model::preset(other)) << other;
      }
    }
  }
  EXPECT_EQ(model::preset("default"), model::default_model());
  EXPECT_EQ(model::preset("no-such-preset"), model::default_model());

  const model gray = model::preset("grayscale");
  ASSERT_EQ(gray.color.size(), 2u);
  expect_color(gray.color[1], 1.0, 1, 1, 1);
  EXPECT_FLOAT_EQ(gray.alpha_at(0.5), 0.5f);

  const model rainbow = model::preset("rainbow");
  ASSERT_EQ(rainbow.color.size(), 5u);
  expect_color(rainbow.color[0], 0.0, 0, 0, 1);
  expect_color(rainbow.color[2], 0.5, 0, 1, 0);
  expect_color(rainbow.color[4], 1.0, 1, 0, 0);

  const model hot = model::preset("hot");
  ASSERT_EQ(hot.color.size(), 4u);
  expect_color(hot.color[1], 0.33, 1, 0, 0);
  expect_color(hot.color[3], 1.0, 1, 1, 1);

  const model cool = model::preset("cool");
  expect_color(cool.color.front(), 0.0, 0, 1, 1);
  expect_color(cool.color.back(), 1.0, 1, 0, 1);

  const model xray = model::preset("xray");
  ASSERT_EQ(xray.color.size(), 2u);
  expect_color(xray.color[0], 0.0, 0, 0, 0);
  expect_color(xray.color[1], 1.0, 1, 1, 1);
  EXPECT_FLOAT_EQ(xray.alpha.back().a, 0.25f);

  const model iso = model::preset("isolines");
  ASSERT_EQ(iso.isocontours.size(), 2u);
  EXPECT_EQ(iso.isocontours[0].id, "iso1");
  EXPECT_EQ(iso.isocontours[0].at, 0.25);
  EXPECT_EQ(iso.isocontours[1].id, "iso2");
  EXPECT_EQ(iso.isocontours[1].at, 0.75);
}

// --- normalize ------------------------------------------------------------------------------

TEST(TfModel, NormalizeInsertsEndpointsByColorTable2Rule) {
  model m;
  m.color = {cn(0.3, 1, 0, 0), cn(0.6, 0, 0, 1)};
  m.alpha = {an(0.4, 0.5f)};
  m.normalize();
  // At 0 the first in-range node is copied; at 1 the missing endpoint is white / opacity 1.
  ASSERT_EQ(m.color.size(), 4u);
  expect_color(m.color[0], 0.0, 1, 0, 0);
  expect_color(m.color[1], 0.3, 1, 0, 0);
  expect_color(m.color[2], 0.6, 0, 0, 1);
  expect_color(m.color[3], 1.0, 1, 1, 1);
  ASSERT_EQ(m.alpha.size(), 3u);
  expect_alpha(m.alpha[0], 0.0, 0.5f);
  expect_alpha(m.alpha[1], 0.4, 0.5f);
  expect_alpha(m.alpha[2], 1.0, 1.f);
}

TEST(TfModel, NormalizeEmptyListsBecomeRamps) {
  model m;
  m.normalize();
  ASSERT_EQ(m.color.size(), 2u);
  expect_color(m.color[0], 0.0, 0, 0, 0);
  expect_color(m.color[1], 1.0, 1, 1, 1);
  ASSERT_EQ(m.alpha.size(), 2u);
  expect_alpha(m.alpha[0], 0.0, 0.f);
  expect_alpha(m.alpha[1], 1.0, 1.f);
}

TEST(TfModel, NormalizeOutOfRangeNodes) {
  model m;
  m.color = {cn(-0.5, 1, 0, 0), cn(0.5, 0, 1, 0), cn(1.5, 0, 0, 1)};
  m.alpha = {an(-1.0, 0.3f), an(2.0, 0.9f)};
  m.normalize();
  // 0 copies the first node at/after 0 (the in-range green); 1 copies the node beyond 1 (blue);
  // the out-of-range nodes themselves are pruned.
  ASSERT_EQ(m.color.size(), 3u);
  expect_color(m.color[0], 0.0, 0, 1, 0);
  expect_color(m.color[1], 0.5, 0, 1, 0);
  expect_color(m.color[2], 1.0, 0, 0, 1);
  // With nothing in range, both endpoints copy the node beyond 1.
  ASSERT_EQ(m.alpha.size(), 2u);
  expect_alpha(m.alpha[0], 0.0, 0.9f);
  expect_alpha(m.alpha[1], 1.0, 0.9f);
}

TEST(TfModel, NormalizeSortsDedupesAndClamps) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  model m;
  m.color = {cn(1.0, 0, 0, 0), cn(0.5, 1.5f, -0.2f, std::numeric_limits<float>::quiet_NaN()),
             cn(0.0, 1, 1, 1), cn(0.5, 0, 0, 1), cn(nan, 1, 0, 0)};
  m.alpha = {an(0.75, 2.f), an(0.25, 0.3f), an(0.25, 0.9f), an(0.0, 0.f), an(1.0, 1.f)};
  m.normalize();
  ASSERT_EQ(m.color.size(), 3u); // the NaN position is dropped, the second 0.5 deduped
  expect_color(m.color[0], 0.0, 1, 1, 1);
  expect_color(m.color[1], 0.5, 1, 0, 0); // the first 0.5 wins; channels clamped, NaN -> 0
  expect_color(m.color[2], 1.0, 0, 0, 0);
  ASSERT_EQ(m.alpha.size(), 4u);
  expect_alpha(m.alpha[1], 0.25, 0.3f);
  expect_alpha(m.alpha[2], 0.75, 1.f);
}

TEST(TfModel, NormalizeIsocontourIdsAndPositions) {
  model m = model::default_model();
  isocontour over = iso_at("over", 1.4);
  over.has_opacity = true;
  over.opacity = 3.f;
  m.isocontours = {iso_at("", 0.5),
                   iso_at("iso1", 0.2),
                   iso_at("bone", 0.3),
                   iso_at("bone", 0.1),
                   iso_at("bone_2", 0.9),
                   iso_at("under", -0.2),
                   over};
  m.normalize();
  ASSERT_EQ(m.isocontours.size(), 7u);
  // Sorted by position, ids unique: the empty one skips the used iso1, the second "bone" skips
  // the existing bone_2.
  const std::vector<std::pair<std::string, double>> want = {
      {"under", 0.0}, {"bone_3", 0.1}, {"iso1", 0.2}, {"bone", 0.3},
      {"iso2", 0.5},  {"bone_2", 0.9}, {"over", 1.0}};
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT_EQ(m.isocontours[i].id, want[i].first) << i;
    EXPECT_DOUBLE_EQ(m.isocontours[i].at, want[i].second) << i;
  }
  EXPECT_FLOAT_EQ(m.isocontours.back().opacity, 1.f);
  model again = m;
  again.normalize();
  EXPECT_EQ(again, m);
}

TEST(TfModel, NormalizeIsIdempotent) {
  model m;
  m.color = {cn(0.7, 0.2f, 0.4f, 0.6f), cn(-3, 1, 1, 1), cn(0.1, 0.5f, 0.5f, 0.5f)};
  m.alpha = {an(1.5, 0.4f), an(0.6, 0.2f)};
  m.isocontours = {iso_at("a", 0.3), iso_at("a", 0.3), iso_at("", 2.0)};
  m.domain.automatic = false;
  m.domain.min = 9;
  m.domain.max = 3;
  m.resolution = -4;
  m.normalize();
  model n = m;
  n.normalize();
  EXPECT_EQ(n, m);
  EXPECT_EQ(to_string(n), to_string(m));
}

TEST(TfModel, NormalizeDomainAndResolution) {
  model m;
  m.domain.automatic = false;
  m.domain.min = 255;
  m.domain.max = -1;
  m.resolution = 0;
  m.normalize();
  EXPECT_FALSE(m.domain.automatic);
  EXPECT_EQ(m.domain.min, -1);
  EXPECT_EQ(m.domain.max, 255);
  EXPECT_EQ(m.resolution, 256);
  m.resolution = 1 << 24;
  m.domain.max = std::numeric_limits<double>::infinity();
  m.normalize();
  EXPECT_EQ(m.resolution, 1 << 16);
  EXPECT_TRUE(m.domain.automatic);
}

// --- sampling and baking --------------------------------------------------------------------

TEST(TfModel, SamplingIsPiecewiseLinear) {
  const model m = model::default_model();
  float rgb[3];
  m.color_at(0.25, rgb);
  EXPECT_FLOAT_EQ(rgb[0], 0.5f);
  EXPECT_FLOAT_EQ(rgb[1], 0.f);
  m.color_at(0.625, rgb);
  EXPECT_FLOAT_EQ(rgb[0], 1.f);
  EXPECT_FLOAT_EQ(rgb[1], 0.5f);
  EXPECT_FLOAT_EQ(rgb[2], 0.f);
  EXPECT_FLOAT_EQ(m.alpha_at(0.5), 0.5f);
  EXPECT_FLOAT_EQ(m.alpha_at(0.125), 0.375f);
  // t is clamped (NaN reads as 0).
  EXPECT_FLOAT_EQ(m.alpha_at(-3.0), 0.f);
  EXPECT_FLOAT_EQ(m.alpha_at(7.0), 1.f);
  EXPECT_FLOAT_EQ(m.alpha_at(std::numeric_limits<double>::quiet_NaN()), 0.f);
  const rgba s = m.sample(0.875);
  EXPECT_FLOAT_EQ(s.r, 0.5f);
  EXPECT_FLOAT_EQ(s.g, 1.f);
  EXPECT_FLOAT_EQ(s.a, 0.625f);
}

TEST(TfModel, EmptyListsSampleBlackTransparent) {
  const model m; // not normalized
  float rgb[3] = {1, 1, 1};
  m.color_at(0.5, rgb);
  EXPECT_EQ(rgb[0], 0.f);
  EXPECT_EQ(rgb[1], 0.f);
  EXPECT_EQ(rgb[2], 0.f);
  EXPECT_EQ(m.alpha_at(0.5), 0.f);
  EXPECT_TRUE(transparent(m));
}

TEST(TfModel, OpacityCubedSampling) {
  model m = model::default_model();
  m.opacity_cubed = true;
  EXPECT_FLOAT_EQ(m.alpha_at(0.5), 0.125f);
  EXPECT_FLOAT_EQ(m.alpha_at(0.25), 0.421875f);
  EXPECT_FLOAT_EQ(m.alpha_at(1.0), 1.f);
  EXPECT_FLOAT_EQ(m.sample(0.125).a, 0.375f * 0.375f * 0.375f);
}

TEST(TfModel, IsocontourColourAndOpacity) {
  model m = model::default_model();
  isocontour iso = iso_at("bone", 0.625);
  float rgb[3];
  m.isocontour_rgb(iso, rgb); // the ramp at its position
  EXPECT_FLOAT_EQ(rgb[0], 1.f);
  EXPECT_FLOAT_EQ(rgb[1], 0.5f);
  EXPECT_FLOAT_EQ(m.isocontour_opacity(iso), 1.f);
  iso.has_rgb = true;
  iso.r = 0.1f;
  iso.g = 0.2f;
  iso.b = 0.3f;
  iso.has_opacity = true;
  iso.opacity = 0.4f;
  m.isocontour_rgb(iso, rgb);
  EXPECT_FLOAT_EQ(rgb[0], 0.1f);
  EXPECT_FLOAT_EQ(rgb[2], 0.3f);
  EXPECT_FLOAT_EQ(m.isocontour_opacity(iso), 0.4f);
}

TEST(TfModel, BakeDefaultMatchesPiecewiseLinearDefinition) {
  const model m = model::default_model();
  const std::vector<std::uint8_t> t = m.bake_rgba8(256);
  ASSERT_EQ(t.size(), 256u * 4);
  // Over t = i/255 the default ramps are integer-valued in bytes, so these are exact.
  for (int i = 0; i < 256; ++i) {
    const double x = i / 255.0;
    const int r = x < 0.5 ? 2 * i : x <= 0.75 ? 255 : 1020 - 4 * i;
    const int g = x < 0.5 ? 0 : x <= 0.75 ? 4 * i - 510 : 255;
    const int a = x <= 0.25 ? 3 * i : x <= 0.75 ? 255 - i : 3 * i - 510;
    SCOPED_TRACE(i);
    EXPECT_EQ(t[4 * i + 0], r);
    EXPECT_EQ(t[4 * i + 1], g);
    EXPECT_EQ(t[4 * i + 2], 0);
    EXPECT_EQ(t[4 * i + 3], a);
  }
  // A few entries spelled out (VolumeRover2's getTable gave alpha 191 / 190 at 63 / 64: its
  // cumulative-index quirk, deliberately not reproduced).
  EXPECT_EQ(t[4 * 63 + 3], 189);
  EXPECT_EQ(t[4 * 64 + 0], 128);
  EXPECT_EQ(t[4 * 64 + 3], 191);
  EXPECT_EQ(t[4 * 255 + 1], 255);
  EXPECT_EQ(t[4 * 255 + 3], 255);
}

TEST(TfModel, BakeAnySize) {
  const model m = model::preset("rainbow");
  EXPECT_TRUE(m.bake(1).empty());
  EXPECT_TRUE(m.bake(0).empty());
  EXPECT_TRUE(m.bake_rgba8(1).empty());
  const std::vector<std::pair<double, double>> r = {{0, 0}, {0.25, 0}, {0.5, 0}, {0.75, 1}, {1, 1}};
  const std::vector<std::pair<double, double>> g = {{0, 0}, {0.25, 1}, {0.5, 1}, {0.75, 1}, {1, 0}};
  const std::vector<std::pair<double, double>> b = {{0, 1}, {0.25, 1}, {0.5, 0}, {0.75, 0}, {1, 0}};
  const std::vector<std::pair<double, double>> a = {{0, 0}, {0.5, 0.5}, {1, 1}};
  for (const int size : {2, 3, 17, 1024}) {
    SCOPED_TRACE(size);
    const std::vector<rgba> f = m.bake(size);
    const std::vector<std::uint8_t> q = m.bake_rgba8(size);
    ASSERT_EQ(f.size(), static_cast<std::size_t>(size));
    ASSERT_EQ(q.size(), static_cast<std::size_t>(size) * 4);
    for (int i = 0; i < size; ++i) {
      const double x = double(i) / double(size - 1);
      EXPECT_NEAR(f[i].r, ref(r, x), 1e-6);
      EXPECT_NEAR(f[i].g, ref(g, x), 1e-6);
      EXPECT_NEAR(f[i].b, ref(b, x), 1e-6);
      EXPECT_NEAR(f[i].a, ref(a, x), 1e-6);
      EXPECT_EQ(q[4 * i + 0], to_byte(ref(r, x)));
      EXPECT_EQ(q[4 * i + 1], to_byte(ref(g, x)));
      EXPECT_EQ(q[4 * i + 2], to_byte(ref(b, x)));
      EXPECT_EQ(q[4 * i + 3], to_byte(ref(a, x)));
    }
    // Exact ends.
    EXPECT_EQ(f.front().b, 1.f);
    EXPECT_EQ(f.back().r, 1.f);
    EXPECT_EQ(f.back().a, 1.f);
  }
}

TEST(TfModel, BakeCubed) {
  model m = model::default_model();
  m.opacity_cubed = true;
  const std::vector<std::pair<double, double>> a = {{0, 0}, {0.25, 0.75}, {0.75, 0.25}, {1, 1}};
  const std::vector<std::uint8_t> q = m.bake_rgba8(256);
  for (int i = 0; i < 256; ++i) {
    const double v = ref(a, i / 255.0);
    EXPECT_EQ(q[4 * i + 3], to_byte(v * v * v)) << i;
  }
  EXPECT_EQ(q[3], 0);
  EXPECT_EQ(q[4 * 255 + 3], 255);
}

TEST(TfModel, IsocontourIdsAndLookup) {
  model m;
  EXPECT_EQ(m.next_isocontour_id(), "iso1");
  m.isocontours = {iso_at("iso1", 0.1), iso_at("iso3", 0.2)};
  EXPECT_EQ(m.next_isocontour_id(), "iso2");
  EXPECT_EQ(m.find_isocontour("iso3"), 1);
  EXPECT_EQ(m.find_isocontour("iso1"), 0);
  EXPECT_EQ(m.find_isocontour("nope"), -1);
  m.isocontours.push_back(iso_at("iso2", 0.3));
  EXPECT_EQ(m.next_isocontour_id(), "iso4");
}

TEST(TfModel, EqualityComparesMeaning) {
  model a = model::default_model();
  a.isocontours = {iso_at("x", 0.5)};
  model b = a;
  b.isocontours[0].r = 0.25f; // no override flag: not part of the model
  b.isocontours[0].opacity = 0.5f;
  b.domain.min = 42; // automatic domain: bounds unused
  EXPECT_EQ(a, b);
  b.isocontours[0].has_rgb = true;
  EXPECT_NE(a, b);
  b = a;
  a.domain.automatic = b.domain.automatic = false;
  b.domain.max = 2;
  EXPECT_NE(a, b);
  b = a;
  b.resolution = 512;
  EXPECT_NE(a, b);
  b = a;
  b.opacity_cubed = true;
  EXPECT_NE(a, b);
  b = a;
  b.isocontours[0].id = "y";
  EXPECT_NE(a, b);
  b = a;
  b.alpha[1].a = 0.7f;
  EXPECT_NE(a, b);
}

TEST(TfModel, EffectiveDomain) {
  model m;
  double lo = 0, hi = 0;
  effective_domain(m, false, 5, 6, lo, hi);
  EXPECT_EQ(lo, 0.0);
  EXPECT_EQ(hi, 1.0);
  effective_domain(m, true, -10, 30, lo, hi);
  EXPECT_EQ(lo, -10.0);
  EXPECT_EQ(hi, 30.0);
  effective_domain(m, true, 7, 7, lo, hi); // degenerate widens
  EXPECT_EQ(lo, 7.0);
  EXPECT_EQ(hi, 8.0);
  effective_domain(m, true, 9, 3, lo, hi);
  EXPECT_EQ(lo, 9.0);
  EXPECT_EQ(hi, 10.0);
  m.domain.automatic = false;
  m.domain.min = 100;
  m.domain.max = 200;
  effective_domain(m, true, -10, 30, lo, hi); // explicit wins over the data range
  EXPECT_EQ(lo, 100.0);
  EXPECT_EQ(hi, 200.0);
  EXPECT_DOUBLE_EQ(to_data(0.25, lo, hi), 125.0);
  EXPECT_DOUBLE_EQ(to_normalized(150.0, lo, hi), 0.5);
  EXPECT_EQ(to_normalized(3.0, 2.0, 2.0), 0.0);
}

// --- the JSON state string ------------------------------------------------------------------

TEST(TfModelJson, DefaultModelExactText) {
  EXPECT_EQ(to_string(model::default_model()), kDefaultJson);
  EXPECT_EQ(parsed(kDefaultJson), model::default_model());
}

TEST(TfModelJson, RoundTripIsStable) {
  std::vector<model> models;
  for (const std::string &name : model::preset_names())
    models.push_back(model::preset(name));
  model custom = model::preset("hot");
  isocontour bone = iso_at("bone \"quoted\" \\ id", 0.6);
  bone.has_rgb = true;
  bone.r = 1.f;
  bone.g = 0.5f;
  bone.b = 0.25f;
  bone.has_opacity = true;
  bone.opacity = 0.75f;
  custom.isocontours = {bone, iso_at("skin", 0.3)};
  custom.domain.automatic = false;
  custom.domain.min = -1024.125;
  custom.domain.max = 3071.1;
  custom.opacity_cubed = true;
  custom.resolution = 1024;
  custom.normalize();
  models.push_back(custom);
  for (const model &m : models) {
    const std::string s = to_string(m);
    SCOPED_TRACE(s);
    model back;
    std::string err;
    ASSERT_TRUE(from_string(s, back, &err)) << err;
    EXPECT_EQ(back, m);
    EXPECT_EQ(to_string(back), s);
  }
}

TEST(TfModelJson, CompactPrecision) {
  model m = model::default_model();
  m.color[1].at = 0.123456789012345;
  m.color[1].r = 0.123456789f;
  m.domain.automatic = false;
  m.domain.min = 0.1;
  m.domain.max = 1.0 / 3.0;
  const std::string s = to_string(m);
  // Positions exact (shortest of %.15g..%.17g), colours %.6g.
  EXPECT_NE(s.find("[0.123456789012345,0.123457,0,0]"), std::string::npos) << s;
  // Data-unit bounds are exact but shortest.
  EXPECT_NE(s.find("\"domain\":[0.1,0.3333333333333333]"), std::string::npos) << s;
  const model back = parsed(s);
  EXPECT_EQ(back.domain.max, 1.0 / 3.0);
  EXPECT_EQ(to_string(parsed(to_string(back))), to_string(back)); // a second trip is a no-op
}

TEST(TfModelJson, PositionsSurviveTheStringExactly) {
  // fe_controls' slider writes fe.mesh.isovalue 0.37 on a [0.0123, 2.7] volume; the binding
  // stores its position at full precision and every later reader parses the string. At 9 digits
  // the lowered isovalue came back 5.7e-10 off -- outside fe_lab's 1e-12 relative check, so any
  // edit re-wrote the isovalue and scheduled a remesh -- and a uint32-wide range moved it by a
  // whole data unit. Colour / alpha positions placed in data units have the same problem.
  struct range_case {
    double lo, hi, v;
  };
  for (const range_case &c :
       {range_case{0.0123, 2.7, 0.37}, range_case{0.0, 4294967295.0, 1234567891.0}}) {
    SCOPED_TRACE(c.v);
    const double at = to_normalized(c.v, c.lo, c.hi);
    model m = model::default_model();
    m.isocontours = {iso_at("iso1", at)};
    m.color.push_back(cn(at / 3, 0.5f, 0.25f, 1.f));
    m.alpha.push_back(an(1.0 / 3.0 + at / 7, 0.5f));
    m.normalize();
    model back;
    ASSERT_TRUE(from_string(to_string(m), back));
    EXPECT_EQ(back, m); // every position bit-identical
    EXPECT_EQ(back.isocontours[0].at, at);
    const double iv = isovalues(back, c.lo, c.hi)[0];
    EXPECT_EQ(iv, isovalues(m, c.lo, c.hi)[0]);
    EXPECT_LE(std::fabs(iv - c.v), 1e-12 * std::max(1.0, std::fabs(c.v)));
    EXPECT_EQ(to_string(back), to_string(m));
  }
}

TEST(TfModelJson, MapAndTupleFormsAreEqual) {
  const model map_form = parsed(R"({"dims": 1, "domain": "auto", "resolution": 256,
      "opacity_cubed": false,
      "color": [{"at": 0.0, "rgb": [0,0,0]}, {"at": 0.5, "rgb": [1,0,0]},
                {"at": 1.0, "rgb": [0,1,0]}],
      "alpha": [{"at": 0.0, "a": 0.0}, {"at": 0.25, "a": 0.75}, {"at": 1.0, "a": 1.0}],
      "isocontours": [{"at": 0.6, "id": "bone"}]})");
  const model tuple_form = parsed(R"({"color":[[0,0,0,0],[0.5,1,0,0],[1,0,1,0]],
      "alpha":[[0,0],[0.25,0.75],[1,1]],"isocontours":[{"id":"bone","at":0.6}]})");
  EXPECT_EQ(map_form, tuple_form);
  ASSERT_EQ(map_form.color.size(), 3u);
  ASSERT_EQ(map_form.isocontours.size(), 1u);
  EXPECT_EQ(map_form.isocontours[0].id, "bone");
  EXPECT_FALSE(map_form.isocontours[0].has_rgb);

  EXPECT_EQ(parsed(R"({"domain":{"min":0,"max":255}})"), parsed(R"({"domain":[0,255]})"));
  const model d = parsed(R"({"domain":[0,255]})");
  EXPECT_FALSE(d.domain.automatic);
  EXPECT_EQ(d.domain.max, 255.0);
}

TEST(TfModelJson, DimsMustBeOne) {
  model m = model::preset("cool");
  std::string err;
  EXPECT_TRUE(from_string(R"({"dims":1})", m, &err)) << err;
  m = model::preset("cool");
  EXPECT_FALSE(from_string(R"({"dims":2,"color":[]})", m, &err));
  EXPECT_NE(err.find("dims"), std::string::npos) << err;
  EXPECT_EQ(m, model::preset("cool")); // a failed parse leaves the output alone
  EXPECT_FALSE(from_string(R"({"dims":"one"})", m, &err));
}

TEST(TfModelJson, EmptyAndMissingKeys) {
  EXPECT_EQ(parsed(""), model::default_model());
  EXPECT_EQ(parsed("  \n"), model::default_model());
  EXPECT_EQ(parsed("{}"), model::default_model()); // a missing key keeps default_model()'s
  const model m = parsed(R"({"alpha":[[0,0],[1,0]],"unknown":{"x":[1,2,3]},"version":7})");
  ASSERT_EQ(m.color.size(), 4u); // the default colours
  expect_color(m.color[2], 0.75, 1, 1, 0);
  EXPECT_TRUE(transparent(m));
  EXPECT_TRUE(m.domain.automatic);
  EXPECT_EQ(m.resolution, 256);
  // A present key replaces, even with an empty list (then normalize()'s black->white ramp).
  const model e = parsed(R"({"color":[],"isocontours":[0.5]})");
  ASSERT_EQ(e.color.size(), 2u);
  expect_color(e.color[1], 1.0, 1, 1, 1);
  EXPECT_EQ(e.alpha.size(), 4u);
  EXPECT_EQ(e.isocontours.size(), 1u);
}

TEST(TfModelJson, ReaderNormalizes) {
  const model m = parsed(R"({"color":[[0.5,2,0,0]],"alpha":[[0.8,0.4],[0.2,0.6],[0.2,0.1]],
      "isocontours":[1.5,{"at":-1,"id":"a"},{"at":0.5,"id":"a"}],"resolution":0})");
  ASSERT_EQ(m.color.size(), 3u);
  expect_color(m.color[0], 0.0, 1, 0, 0);
  expect_color(m.color[2], 1.0, 1, 1, 1);
  ASSERT_EQ(m.alpha.size(), 4u);
  expect_alpha(m.alpha[1], 0.2, 0.6f);
  ASSERT_EQ(m.isocontours.size(), 3u);
  EXPECT_EQ(m.isocontours[0].id, "a");
  EXPECT_EQ(m.isocontours[0].at, 0.0);
  EXPECT_EQ(m.isocontours[1].id, "a_2");
  EXPECT_EQ(m.isocontours[2].id, "iso1");
  EXPECT_EQ(m.isocontours[2].at, 1.0);
  EXPECT_EQ(m.resolution, 256);
}

TEST(TfModelJson, IsocontourForms) {
  const model m = parsed(R"({"isocontours":[0.3,{"id":"bone","at":0.6,"rgb":[1,1,1],
      "opacity":0.5},{"id":7,"at":0.9},{"id":null,"at":0.1,"rgb":null}]})");
  ASSERT_EQ(m.isocontours.size(), 4u);
  EXPECT_EQ(m.isocontours[0].id, "iso2"); // the null id at 0.1 (iso1 went to the bare 0.3)
  EXPECT_EQ(m.isocontours[1].id, "iso1");
  EXPECT_EQ(m.isocontours[1].at, 0.3);
  EXPECT_EQ(m.isocontours[2].id, "bone");
  EXPECT_TRUE(m.isocontours[2].has_rgb);
  EXPECT_TRUE(m.isocontours[2].has_opacity);
  EXPECT_FLOAT_EQ(m.isocontours[2].opacity, 0.5f);
  EXPECT_EQ(m.isocontours[3].id, "7");
}

TEST(TfModelJson, MalformedInputFails) {
  for (const char *bad :
       {"{", "[1,2]", "{\"color\":[[0,1,2]]}", "{\"color\":{}}", "{\"alpha\":[[0]]}",
        "{\"alpha\":[{\"at\":0}]}", "{\"isocontours\":[{\"id\":\"x\"}]}",
        "{\"isocontours\":[\"x\"]}", "{\"domain\":\"manual\"}", "{\"domain\":[1]}",
        "{\"opacity_cubed\":\"yes\"}", "{\"resolution\":\"big\"}", "{\"color\":[]} trailing",
        "nope"}) {
    model m = model::preset("hot");
    std::string err;
    EXPECT_FALSE(from_string(bad, m, &err)) << bad;
    EXPECT_EQ(err.rfind("tf model: ", 0), 0u) << err;
    EXPECT_EQ(m, model::preset("hot")) << bad;
  }
}

TEST(TfModelJson, RawTableForm) {
  const model m = parsed(R"({"dims":1,"domain":{"min":0,"max":255},
      "table":{"size":3,"rgba":[0,0,0,0, 1,0,0,0.5, 1,1,1,1]}})");
  ASSERT_EQ(m.color.size(), 3u);
  expect_color(m.color[1], 0.5, 1, 0, 0);
  ASSERT_EQ(m.alpha.size(), 3u);
  expect_alpha(m.alpha[1], 0.5, 0.5f);
  EXPECT_FALSE(m.domain.automatic);
  const std::vector<std::uint8_t> q = m.bake_rgba8(3);
  EXPECT_EQ(q[4], 255);
  EXPECT_EQ(q[7], 128);
  model out;
  EXPECT_FALSE(from_string(R"({"table":{"size":4,"rgba":[0,0,0,0]}})", out));
  // Explicit node lists win over a table.
  EXPECT_EQ(parsed(R"({"alpha":[[0,0],[1,1]],"table":{"rgba":[1,0,0,1]}})").color.back().g, 1.f);
}

TEST(TfModelJson, LocaleIndependent) {
  const char *const comma_locales[] = {"de_DE.UTF-8", "de_DE.utf8",  "de_DE", "German",
                                       "de-DE",       "fr_FR.UTF-8", "French"};
  const char *got = nullptr;
  for (const char *name : comma_locales)
    if ((got = std::setlocale(LC_NUMERIC, name)) != nullptr)
      break;
  char probe[16] = {0};
  if (got)
    std::snprintf(probe, sizeof(probe), "%.1f", 0.5);
  if (!got || std::string(probe) != "0,5") {
    std::setlocale(LC_NUMERIC, "C");
    GTEST_SKIP() << "no decimal-comma locale installed";
  }
  const std::string s = to_string(model::default_model());
  model back;
  const bool ok = from_string(s, back);
  const std::string csv = format_csv({0.5, -1.25});
  std::vector<double> v;
  const bool csv_ok = parse_csv("0.5,-1.25", v);
  std::setlocale(LC_NUMERIC, "C");
  EXPECT_EQ(s, kDefaultJson);
  EXPECT_TRUE(ok);
  EXPECT_EQ(back, model::default_model());
  EXPECT_EQ(csv, "0.5,-1.25");
  EXPECT_TRUE(csv_ok);
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[1], -1.25);
}

namespace {

// Every number codec of the model and .vinay files, run under whatever locale is installed.
struct codec_run {
  std::string json, csv, vinay, ctbl;
  bool json_ok = false, csv_ok = false, vinay_ok = false;
  model json_back, vinay_back;
  std::vector<double> csv_back;
};

model locale_probe_model() {
  model m = model::preset("hot");
  m.color.push_back(cn(1.0 / 3.0, 0.123456789f, 0.5f, 0.25f));
  m.alpha.push_back(an(0.2718281828459045, 0.314159f));
  isocontour bone = iso_at("bone", 2.0 / 3.0);
  bone.has_rgb = true;
  bone.r = 0.75f;
  bone.has_opacity = true;
  bone.opacity = 0.5f;
  m.isocontours = {bone};
  m.domain.automatic = false; // written exactly: append_exact's read-back check runs too
  m.domain.min = -1024.125;
  m.domain.max = 1.0 / 3.0;
  m.opacity_cubed = true;
  m.normalize();
  return m;
}

const std::vector<double> kCsvProbe = {0.5, -1.25, 1.0 / 3.0, 6.02214076e23, -2.5e-300};

codec_run run_codecs(const model &m) {
  codec_run r;
  r.json = to_string(m);
  r.json_ok = from_string(r.json, r.json_back);
  r.csv = format_csv(kCsvProbe);
  r.csv_ok = parse_csv(r.csv, r.csv_back);
  r.vinay = write_vinay_string(m);
  try {
    r.vinay_back = read_vinay_string(r.vinay);
    r.vinay_ok = true;
  } catch (const vinay_error &) {
  }
  r.ctbl = write_ctbl_string(m, 4);
  return r;
}

void expect_same_codecs(const codec_run &got, const codec_run &ref) {
  EXPECT_EQ(got.json, ref.json);
  EXPECT_TRUE(got.json_ok) << got.json;
  EXPECT_EQ(got.json_back, ref.json_back); // (colours are %.6g: not `m` itself)
  EXPECT_EQ(got.csv, ref.csv);
  EXPECT_TRUE(got.csv_ok) << got.csv;
  EXPECT_EQ(got.csv_back, kCsvProbe);
  EXPECT_EQ(got.vinay, ref.vinay);
  EXPECT_TRUE(got.vinay_ok) << got.vinay;
  EXPECT_EQ(got.vinay_back, ref.vinay_back);
  EXPECT_EQ(got.ctbl, ref.ctbl);
}

// A C++ numpunct no codec may use: '/' decimal point, '\'' thousands separator every digit.
struct odd_numpunct : std::numpunct<char> {
  char do_decimal_point() const override { return '/'; }
  char do_thousands_sep() const override { return '\''; }
  std::string do_grouping() const override { return "\1"; }
};

} // namespace

// The codecs ignore the global C++ locale (a stream that is not imbued would pick this one up).
TEST(TfModelJson, AnyGlobalCppLocale) {
  const model m = locale_probe_model();
  const codec_run ref = run_codecs(m);
  ASSERT_TRUE(ref.json_ok && ref.csv_ok && ref.vinay_ok);
  const std::locale prev =
      std::locale::global(std::locale(std::locale::classic(), new odd_numpunct));
  const codec_run got = run_codecs(m);
  std::locale::global(prev);
  expect_same_codecs(got, ref);
  EXPECT_EQ(to_string(model::default_model()), kDefaultJson);
}

// ... and the C locale's decimal point whatever it is -- not only a decimal comma: glibc's fa_IR
// writes U+066B (two UTF-8 bytes), Windows' fa-IR writes '/'.
TEST(TfModelJson, AnyCDecimalPoint) {
  const model m = locale_probe_model();
  const codec_run ref = run_codecs(m);
  ASSERT_TRUE(ref.json_ok && ref.csv_ok && ref.vinay_ok);
  const std::string prev = std::setlocale(LC_ALL, nullptr);
  const char *const odd_locales[] = {"fa_IR.UTF-8",  "fa_IR.utf8",  "fa_IR", "ps_AF.UTF-8",
                                     "ps_AF.utf8",   "fa-IR.UTF-8", "fa-IR", "fa",
                                     "ckb_IQ.UTF-8", "ar_AE.UTF-8"};
  std::string used, probe;
  for (const char *name : odd_locales) {
    if (!std::setlocale(LC_ALL, name))
      continue;
    char buf[32] = {0};
    std::snprintf(buf, sizeof(buf), "%.1f", 0.5);
    if (std::string(buf) != "0.5" && std::string(buf) != "0,5") {
      used = name;
      probe = buf;
      break;
    }
  }
  if (used.empty()) {
    std::setlocale(LC_ALL, prev.c_str());
    GTEST_SKIP() << "no locale with a decimal point other than '.' or ',' installed";
  }
  const codec_run got = run_codecs(m);
  const std::string def = to_string(model::default_model());
  std::setlocale(LC_ALL, prev.c_str());
  SCOPED_TRACE(used);
  EXPECT_NE(probe, "0.5"); // printf really does write another decimal point here
  expect_same_codecs(got, ref);
  EXPECT_EQ(def, kDefaultJson);
}

// --- lowering -------------------------------------------------------------------------------

TEST(TfModelLowering, TablesInDataUnits) {
  model m = model::default_model();
  m.isocontours = {iso_at("a", 0.6), iso_at("b", 0.0)};
  m.normalize();
  EXPECT_FALSE(transparent(m));
  EXPECT_EQ(color_table(m, 10, 20),
            (std::vector<double>{10, 0, 0, 0, 15, 1, 0, 0, 17.5, 1, 1, 0, 20, 0, 1, 0}));
  EXPECT_EQ(opacity_table(m, 10, 20), (std::vector<double>{10, 0, 12.5, 0.75, 17.5, 0.25, 20, 1}));
  const std::vector<double> iso = isovalues(m, 10, 20);
  ASSERT_EQ(iso.size(), 2u);
  EXPECT_DOUBLE_EQ(iso[0], 10.0); // model order (sorted): b then a
  EXPECT_DOUBLE_EQ(iso[1], 16.0);
}

TEST(TfModelLowering, CubedOpacityIsResampled) {
  model m = model::default_model();
  m.opacity_cubed = true;
  // 64 grid points (i/63) merged with the nodes; the interior ones at .25 / .75 miss the grid.
  const std::vector<double> ot = opacity_table(m, -5, 5, 64);
  ASSERT_EQ(ot.size(), 2u * 66);
  EXPECT_EQ(ot[0], -5.0);
  EXPECT_EQ(ot[1], 0.0);
  EXPECT_EQ(ot[130], 5.0);
  EXPECT_EQ(ot[131], 1.0);
  std::vector<double> ts = {0.25, 0.75};
  for (int i = 0; i < 64; ++i)
    ts.push_back(i / 63.0);
  std::sort(ts.begin(), ts.end());
  for (std::size_t i = 0; i < ts.size(); ++i) {
    EXPECT_NEAR(ot[2 * i], -5 + 10 * ts[i], 1e-12) << i;
    EXPECT_NEAR(ot[2 * i + 1], m.alpha_at(ts[i]), 1e-6) << i;
  }
  // The default count is the model's resolution.
  EXPECT_EQ(opacity_table(m, 0, 1).size(), 2u * (256 + 2));
  m.resolution = 5; // the grid 0, .25, .5, .75, 1 already holds every node
  EXPECT_EQ(opacity_table(m, 0, 1).size(), 10u);
  EXPECT_EQ(opacity_table(m, 0, 1, 1).size(), 8u); // at least the two ends, plus the nodes
  EXPECT_EQ(color_table(m, 0, 1).size(), 16u);     // colour is not resampled
}

namespace {

// The renderers' reading of a flat value,a table: linear between entries, held past the ends.
double table_at(const std::vector<double> &t, double x) {
  const std::size_t n = t.size() / 2;
  if (x <= t[0])
    return t[1];
  for (std::size_t i = 1; i < n; ++i)
    if (x < t[2 * i]) {
      const double u = (x - t[2 * i - 2]) / (t[2 * i] - t[2 * i - 2]);
      return t[2 * i - 1] + (t[2 * i + 1] - t[2 * i - 1]) * u;
    }
  return t[2 * n - 1];
}

} // namespace

TEST(TfModelLowering, CubedOpacityKeepsNodeFeatures) {
  // A one-node spike far narrower than a 64-point grid's spacing (the grid sampled only its flanks:
  // ~0.009 drawn where the editor shows 1), and a 0 -> 1 step over 0.001 (smeared over ~0.016).
  model spike;
  spike.alpha = {an(0, 0), an(0.49, 0), an(0.5, 1), an(0.51, 0), an(1, 0)};
  model step;
  step.alpha = {an(0, 0), an(0.3, 0), an(0.301, 1), an(1, 1)};
  for (model *m : {&spike, &step}) {
    m->opacity_cubed = true;
    m->normalize();
  }
  for (const int samples : {64, 0, 2}) {
    SCOPED_TRACE(samples);
    for (const model *m : {&spike, &step}) {
      const std::vector<double> ot = opacity_table(*m, 0, 100, samples);
      ASSERT_GE(ot.size(), 2 * m->alpha.size());
      for (std::size_t i = 2; i < ot.size(); i += 2)
        EXPECT_LT(ot[i - 2], ot[i]) << i; // strictly increasing values
      // Every node's own (cubed) opacity is in the table, where the renderers read it.
      for (const alpha_node &a : m->alpha)
        EXPECT_NEAR(table_at(ot, 100 * a.at), m->alpha_at(a.at), 1e-6) << a.at;
    }
    EXPECT_NEAR(table_at(opacity_table(spike, 0, 100, samples), 50), 1.0, 1e-6);
    const std::vector<double> st = opacity_table(step, 0, 100, samples);
    EXPECT_NEAR(table_at(st, 30.0), 0.0, 1e-6);
    EXPECT_NEAR(table_at(st, 30.1), 1.0, 1e-6);
  }
  // The default sample count follows `resolution`: every grid point i/(res-1) and every node,
  // nothing else.
  spike.resolution = 17;
  const std::vector<double> ot = opacity_table(spike, 0, 100);
  std::vector<double> want = {0.49, 0.51}; // 0, .5 and 1 are on the grid
  for (int i = 0; i < 17; ++i)
    want.push_back(i / 16.0);
  std::sort(want.begin(), want.end());
  ASSERT_EQ(ot.size(), 2 * want.size());
  for (std::size_t i = 0; i < want.size(); ++i)
    EXPECT_NEAR(ot[2 * i], 100 * want[i], 1e-12) << i;
}

TEST(TfModelLowering, TransparentModelLowersToEmptyTables) {
  const model m = model::preset("isolines");
  EXPECT_TRUE(transparent(m));
  EXPECT_TRUE(color_table(m, 0, 100).empty());
  EXPECT_TRUE(opacity_table(m, 0, 100).empty());
  model cubed = m;
  cubed.opacity_cubed = true;
  EXPECT_TRUE(opacity_table(cubed, 0, 100).empty());
  EXPECT_EQ(isovalues(m, 0, 100), (std::vector<double>{25, 75}));
}

TEST(TfModelLowering, FromTablesRoundTrip) {
  const std::vector<std::pair<double, double>> ranges = {{0.1, 0.3}, {-1000, 3.7}, {0, 255}};
  for (const std::string name : {"default", "rainbow", "hot", "xray"}) {
    for (const auto &range : ranges) {
      SCOPED_TRACE(name + " " + std::to_string(range.first) + ".." + std::to_string(range.second));
      const model m = model::preset(name);
      const model back =
          from_tables(color_table(m, range.first, range.second),
                      opacity_table(m, range.first, range.second), range.first, range.second);
      ASSERT_EQ(back.color.size(), m.color.size()); // no spurious endpoint from rounding
      ASSERT_EQ(back.alpha.size(), m.alpha.size());
      for (std::size_t i = 0; i < m.color.size(); ++i) {
        EXPECT_NEAR(back.color[i].at, m.color[i].at, 1e-12);
        EXPECT_EQ(back.color[i].r, m.color[i].r);
        EXPECT_EQ(back.color[i].g, m.color[i].g);
        EXPECT_EQ(back.color[i].b, m.color[i].b);
      }
      for (std::size_t i = 0; i < m.alpha.size(); ++i) {
        EXPECT_NEAR(back.alpha[i].at, m.alpha[i].at, 1e-12);
        EXPECT_EQ(back.alpha[i].a, m.alpha[i].a);
      }
      EXPECT_EQ(back.color.back().at, 1.0);
      EXPECT_EQ(back.alpha.back().at, 1.0);
    }
  }
}

TEST(TfModelLowering, EmptyTablesReadAsTransparentBlack) {
  const model none = from_tables({}, {}, 0, 1);
  EXPECT_TRUE(transparent(none));
  float rgb[3];
  none.color_at(0.7, rgb);
  EXPECT_EQ(rgb[0], 0.f);
  // ... so a transparent model survives lowering and raising.
  EXPECT_TRUE(transparent(from_tables(color_table(none, 0, 1), opacity_table(none, 0, 1), 0, 1)));
  const model colour_only = from_tables({0, 1, 0, 0, 10, 0, 0, 1}, {}, 0, 10);
  EXPECT_TRUE(transparent(colour_only));
  expect_color(colour_only.color.back(), 1.0, 0, 0, 1);
  // A partial trailing entry is ignored.
  EXPECT_EQ(from_tables({0, 1, 0, 0, 5}, {0, 0.5, 9}, 0, 10).alpha.front().a, 0.5f);
}

TEST(TfModelLowering, FromCoupledPoints) {
  const model m = from_coupled_points({0, 0, 0, 0, 0, 50, 1, 0, 0, 0.5, 100, 1, 1, 1, 1}, 0, 100);
  ASSERT_EQ(m.color.size(), 3u);
  expect_color(m.color[0], 0.0, 0, 0, 0);
  expect_color(m.color[1], 0.5, 1, 0, 0);
  expect_color(m.color[2], 1.0, 1, 1, 1);
  ASSERT_EQ(m.alpha.size(), 3u);
  expect_alpha(m.alpha[1], 0.5, 0.5f);
  // Points outside the range fall to normalize()'s endpoint rule.
  const model w = from_coupled_points({-50, 1, 0, 0, 1, 50, 0, 1, 0, 0.5}, 0, 100);
  expect_color(w.color.front(), 0.0, 0, 1, 0);
  expect_color(w.color.back(), 1.0, 1, 1, 1);
}

TEST(TfModelLowering, FromTablesKeepsHardSteps) {
  // A legacy scene's points {100 red} {100 blue} {200 blue} over [0, 200] (as scene_realize hands
  // them over, with the ramp sampled at both ends): red up to 100, blue above -- a step
  // cvc::volren::transfer_function keeps.
  const model m = from_coupled_points(
      {0, 1, 0, 0, 1, 100, 1, 0, 0, 1, 100, 0, 0, 1, 1, 200, 0, 0, 1, 1}, 0, 200);
  ASSERT_EQ(m.color.size(), 4u);
  expect_color(m.color[1], 0.5, 1, 0, 0);
  EXPECT_EQ(m.color[2].at, 0.5 + 1e-9); // the later point, a documented hair above
  expect_color(m.color[2], 0.5 + 1e-9, 0, 0, 1);
  float rgb[3];
  m.color_at(0.5 - 1e-6, rgb);
  EXPECT_EQ(rgb[0], 1.f);
  EXPECT_EQ(rgb[2], 0.f);
  m.color_at(0.5 + 1e-6, rgb);
  EXPECT_EQ(rgb[0], 0.f);
  EXPECT_EQ(rgb[2], 1.f);
  // It survives the state string ...
  model back;
  ASSERT_TRUE(from_string(to_string(m), back));
  EXPECT_EQ(back, m);
  // ... and lowers back to the step: the pair at 100 and a hair above (at most span * 1e-9, the
  // gap tf_binding's separate_steps opens for any step it hands a renderer).
  const std::vector<double> ct = color_table(m, 0, 200);
  ASSERT_EQ(ct.size(), 16u);
  EXPECT_EQ(ct[4], 100.0);
  EXPECT_EQ(ct[5], 1.0); // red
  EXPECT_GT(ct[8], 100.0);
  EXPECT_LE(ct[8] - 100.0, 200 * 1e-9 * (1 + 1e-6));
  EXPECT_EQ(ct[11], 1.0); // blue
  const model again = from_tables(ct, opacity_table(m, 0, 200), 0, 200);
  ASSERT_EQ(again.color.size(), 4u);
  EXPECT_NEAR(again.color[2].at, 0.5 + 1e-9, 1e-15);
  EXPECT_EQ(again.color[2].b, 1.f);

  // An opacity step at the top end keeps the LATER point at 1 (what holds past the range).
  const model top = from_tables({}, {0, 0, 10, 0.25, 10, 1}, 0, 10);
  ASSERT_EQ(top.alpha.size(), 3u);
  expect_alpha(top.alpha[1], 1.0 - 1e-9, 0.25f);
  EXPECT_EQ(top.alpha[2].at, 1.0);
  EXPECT_EQ(top.alpha[2].a, 1.f);

  // A run of three whose next point is closer than the gap splits that room instead of passing
  // the point; table order is kept.
  const model tight = from_tables({}, {0, 0, 5, 0.2, 5, 0.4, 5, 0.6, 5 + 1e-9, 0.8, 10, 1}, 0, 10);
  ASSERT_EQ(tight.alpha.size(), 6u);
  const float want[] = {0.f, 0.2f, 0.4f, 0.6f, 0.8f, 1.f};
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(tight.alpha[i].a, want[i]) << i;
    if (i)
      EXPECT_LT(tight.alpha[i - 1].at, tight.alpha[i].at) << i;
  }
  EXPECT_EQ(tight.alpha[1].at, 0.5);
}

TEST(TfModelCsv, FormatAndParseRoundTrip) {
  const std::vector<double> v = {0.1, 1.0 / 3.0, -2.5e-300, 1e300, 0.0, -7, 123456789.125};
  const std::string s = format_csv(v);
  std::vector<double> back;
  ASSERT_TRUE(parse_csv(s, back)) << s;
  EXPECT_EQ(back, v);
  EXPECT_EQ(format_csv({0.1}), "0.10000000000000001"); // %.17g, like the renderer keys
  EXPECT_EQ(format_csv({}), "");
  EXPECT_EQ(format_csv({1, 2.5}), "1,2.5");
}

TEST(TfModelCsv, ParseIsStrict) {
  std::vector<double> v = {9};
  EXPECT_TRUE(parse_csv("", v));
  EXPECT_TRUE(v.empty());
  EXPECT_TRUE(parse_csv(" 1, 2 ,3e2 ", v));
  EXPECT_EQ(v, (std::vector<double>{1, 2, 300}));
  for (const char *bad :
       {"1,,2", "1,2,", ",1", "abc", "1 2", "1e", "nan", "inf", " ", "0x10", "1;2", "--1"}) {
    v = {9};
    EXPECT_FALSE(parse_csv(bad, v)) << bad;
    EXPECT_TRUE(v.empty()) << bad;
  }
}

// --- .vinay / .ctbl -------------------------------------------------------------------------

TEST(TfVinay, ColorTable2DefaultFileReadsAsDefaultModel) {
  std::vector<std::string> warnings;
  EXPECT_EQ(read_vinay_string(kDefaultVinay, &warnings), model::default_model());
  EXPECT_TRUE(warnings.empty()) << warnings.front();
  std::istringstream in(kDefaultVinay);
  EXPECT_EQ(read_vinay(in), model::default_model());
}

TEST(TfVinay, WriterReproducesColorTable2Layout) {
  EXPECT_EQ(write_vinay_string(model::default_model()), kDefaultVinay);
  std::ostringstream out;
  write_vinay(out, model::default_model());
  EXPECT_EQ(out.str(), kDefaultVinay);
  // First node, last node, then the interior ones.
  const std::vector<std::string> lines = split_lines(write_vinay_string(model::preset("rainbow")));
  ASSERT_GE(lines.size(), 20u);
  EXPECT_EQ(lines[7], "0.5 0.5");
  EXPECT_EQ(lines[8], "ColorMap");
  EXPECT_EQ(lines[10], "5");
  EXPECT_EQ(lines[12], "0 0 0 1");
  EXPECT_EQ(lines[13], "1 1 0 0");
  EXPECT_EQ(lines[14], "0.25 0 1 1");
  EXPECT_EQ(lines[16], "0.75 1 1 0");
}

TEST(TfVinay, TolerantOfCrlfTabsAndBlanks) {
  const std::string text = "\xEF\xBB\xBF"
                           "Anthony and Vinay are Great.  \r\n"
                           "Alphamap\r\n"
                           "Number  of nodes\r\n"
                           "  4\t\r\n"
                           "Position and opacity\r\n"
                           "0\t0\r\n"
                           "1   1   \r\n"
                           "\r\n"
                           "0.25 \t 0.75\r\n"
                           "0.75 0.25\r\n"
                           "colormap\r\n"
                           "Number of nodes\r\n"
                           "4\r\n"
                           "Position and RGB\r\n"
                           "0 0 0 0\r\n"
                           "1\t0\t1\t0\r\n"
                           "0.5 1 0 0\r\n"
                           "0.75  1  1  0\r\n"
                           "IsocontourMap\r\n"
                           "Number of nodes\r\n"
                           "0\r\n"
                           "Position\r\n"
                           "\r\n"
                           "   \r\n";
  std::vector<std::string> warnings;
  EXPECT_EQ(read_vinay_string(text, &warnings), model::default_model());
  EXPECT_TRUE(warnings.empty()) << warnings.front();
}

TEST(TfVinay, WrongMagicLineOnlyWarns) {
  std::string text = kDefaultVinay;
  text.replace(0, text.find('\n'), "Anthony and Vinay are great!");
  std::vector<std::string> warnings;
  EXPECT_EQ(read_vinay_string(text, &warnings), model::default_model());
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_EQ(warnings[0].rfind("line 1: ", 0), 0u) << warnings[0];

  // A file that starts straight at a section keeps that section.
  warnings.clear();
  const model m = read_vinay_string(std::string(kDefaultVinay).substr(29), &warnings);
  EXPECT_EQ(m, model::default_model());
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("missing"), std::string::npos) << warnings[0];

  // Missing labels and stray lines are warnings too.
  warnings.clear();
  const model n = read_vinay_string("Anthony and Vinay are Great.\nsome note\nAlphamap\n2\n"
                                    "0 0.5\n1 0.5\n",
                                    &warnings);
  EXPECT_EQ(warnings.size(), 3u); // the stray line, the count label, the position label
  ASSERT_EQ(n.alpha.size(), 2u);
  EXPECT_EQ(n.alpha[1].a, 0.5f);
}

TEST(TfVinay, NotAVinayFileThrows) {
  EXPECT_THROW(read_vinay_string("hello\nworld\n"), vinay_error);
  EXPECT_THROW(read_vinay_string(""), vinay_error);
  // The magic alone is a valid (empty) file.
  model blank;
  blank.normalize();
  EXPECT_EQ(read_vinay_string("Anthony and Vinay are Great.\n"), blank);
}

TEST(TfVinay, StructuralErrorsCarryTheLineNumber) {
  const std::string head = "Anthony and Vinay are Great.\n";
  struct bad_case {
    std::string text;
    const char *line;
  };
  const std::vector<bad_case> cases = {
      // a short node line
      {head + "Alphamap\nNumber of nodes\n2\nPosition and opacity\n0 0\n0.5\n", "line 7:"},
      // a non-numeric value
      {head + "ColorMap\nNumber of nodes\n1\nPosition and RGB\n0.5 1 abc 0\n", "line 6:"},
      // a section header without a count
      {head + "ColorMap\nNumber of nodes\nfour\n", "line 4:"},
      {head + "IsocontourMap\n", "line 2:"},
      // a list cut short
      {head + "Alphamap\nNumber of nodes\n3\nPosition and opacity\n0 0\n1 1\n",
       "line 8 (end of file):"},
      // too many fields, comma separators
      {head + "Alphamap\nNumber of nodes\n1\nPosition and opacity\n0 0 0\n", "line 6:"},
      {head + "Alphamap\nNumber of nodes\n1\nPosition and opacity\n0,0\n", "line 6:"},
  };
  for (const bad_case &c : cases) {
    const std::string what = vinay_error_text(c.text);
    EXPECT_NE(what.find(c.line), std::string::npos) << c.text << "\n-> " << what;
  }
}

TEST(TfVinay, IsocontoursGetIdsInPositionOrder) {
  std::vector<std::string> warnings;
  const model m = read_vinay_string("Anthony and Vinay are Great.\nIsocontourMap\n"
                                    "Number of nodes\n3\nPosition\n0.7\n0.2\n0.7\n",
                                    &warnings);
  ASSERT_EQ(m.isocontours.size(), 2u);
  EXPECT_EQ(m.isocontours[0].id, "iso1");
  EXPECT_EQ(m.isocontours[0].at, 0.2);
  EXPECT_EQ(m.isocontours[1].id, "iso2");
  EXPECT_EQ(m.isocontours[1].at, 0.7);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_EQ(warnings[0].rfind("line 8: ", 0), 0u) << warnings[0];
}

TEST(TfVinay, SectionsInAnyOrderAccumulate) {
  const model m = read_vinay_string("Anthony and Vinay are Great.\n"
                                    "ColorMap\nNumber of nodes\n2\nPosition and RGB\n"
                                    "0 1 0 0\n1 0 0 1\n"
                                    "Alphamap\nNumber of nodes\n1\nPosition and opacity\n0 0\n"
                                    "Alphamap\nNumber of nodes\n1\nPosition and opacity\n1 0.5\n");
  ASSERT_EQ(m.color.size(), 2u);
  expect_color(m.color[0], 0.0, 1, 0, 0);
  ASSERT_EQ(m.alpha.size(), 2u);
  expect_alpha(m.alpha[1], 1.0, 0.5f);
}

TEST(TfVinay, WriteReadRoundTripIsExact) {
  model m;
  m.color = {cn(0.0, 0.1f, 0.2f, 0.3f), cn(0.1 + 1e-12, 0.123456789f, 1.f / 3.f, 0.7f),
             cn(1.0 / 3.0, 0.999999f, 0.000001f, 0.5f), cn(1.0, 1.f, 0.f, 2.f / 3.f)};
  m.alpha = {an(0.0, 0.01f), an(0.2718281828459045, 0.314159f), an(1.0, 0.9f)};
  m.isocontours = {iso_at("iso1", 0.2 + 1e-15), iso_at("iso2", 2.0 / 3.0)};
  m.normalize();
  const std::string text = write_vinay_string(m);
  std::vector<std::string> warnings;
  const model back = read_vinay_string(text, &warnings);
  EXPECT_EQ(back, m) << text;
  EXPECT_TRUE(warnings.empty());
  EXPECT_EQ(write_vinay_string(back), text);
}

TEST(TfVinay, FileReadWrite) {
  namespace fs = std::filesystem;
  const fs::path path = fs::temp_directory_path() / "cvc_tf_model_test_roundtrip.vinay";
  const model m = model::preset("hot");
  write_vinay_file(path.string(), m);
  EXPECT_EQ(read_vinay_file(path.string()), m);
  std::error_code ec;
  fs::remove(path, ec);
  EXPECT_THROW(read_vinay_file((fs::temp_directory_path() / "cvc_tf_no_such_file.vinay").string()),
               vinay_error);
  // A file error names the path as well as the line.
  const fs::path bad = fs::temp_directory_path() / "cvc_tf_model_test_bad.vinay";
  {
    std::ofstream out(bad.string(), std::ios::binary);
    out << "Anthony and Vinay are Great.\nAlphamap\nNumber of nodes\nx\n";
  }
  try {
    read_vinay_file(bad.string());
    ADD_FAILURE() << "expected vinay_error";
  } catch (const vinay_error &e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(bad.string() + ", line 4:"), std::string::npos) << what;
  }
  fs::remove(bad, ec);
}

TEST(TfVinay, CtblIsTheBakedTable) {
  const model m = model::default_model();
  const std::vector<std::string> rows = split_lines(write_ctbl_string(m));
  ASSERT_EQ(rows.size(), 257u);
  EXPECT_EQ(rows[0], "256");
  EXPECT_EQ(rows[1], "0 0 0 0");
  EXPECT_EQ(rows[65], "128 0 0 191");
  EXPECT_EQ(rows[256], "0 255 0 255");
  const std::vector<std::uint8_t> t = m.bake_rgba8(256);
  for (int i = 0; i < 256; ++i) {
    std::istringstream row(rows[i + 1]);
    int r = -1, g = -1, b = -1, a = -1;
    row >> r >> g >> b >> a;
    EXPECT_EQ(r, t[4 * i]);
    EXPECT_EQ(g, t[4 * i + 1]);
    EXPECT_EQ(b, t[4 * i + 2]);
    EXPECT_EQ(a, t[4 * i + 3]);
  }
  EXPECT_EQ(split_lines(write_ctbl_string(m, 16)).size(), 17u);
  EXPECT_EQ(split_lines(write_ctbl_string(m, 0)).front(), "2");
  model cubed = m;
  cubed.opacity_cubed = true; // honoured (ColorTable2 hard-coded cubing in its .ctbl writer)
  const std::vector<std::string> crow = split_lines(write_ctbl_string(cubed));
  EXPECT_EQ(crow[65],
            "128 0 0 " + std::to_string(int(std::floor(std::pow(191 / 255.0, 3) * 255 + 0.5))));
}

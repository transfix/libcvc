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

// Unit tests for cvc::compute_contour_spectrum, cvc::contour_spectrum_builder (the same
// computation a budget at a time) and the libContour parity oracle.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/volume/contour_spectrum.h>
#include <cvc/volume/volume.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc;

namespace {

constexpr double kPi = 3.14159265358979323846;

// |p| sampled on an n^3 grid over [-1, 1]^3: level set w is the sphere of radius w.
volume sphere_field(app &ctx, uint64 n) {
  volume v(ctx, dimension(n, n, n), Float, bounding_box(-1, -1, -1, 1, 1, 1));
  const double h = 2.0 / static_cast<double>(n - 1);
  for (uint64 k = 0; k < n; ++k)
    for (uint64 j = 0; j < n; ++j)
      for (uint64 i = 0; i < n; ++i) {
        const double x = -1.0 + h * i, y = -1.0 + h * j, z = -1.0 + h * k;
        v(i, j, k, std::sqrt(x * x + y * y + z * z));
      }
  return v;
}

// f = x (in voxels) on a 33 x 17 x 9 grid with unit spacing: level set w is the plane x = w, of
// area 16 * 8, and the sublevel set has volume 128 w.
volume ramp_field(app &ctx) {
  volume v(ctx, dimension(33, 17, 9), Float, bounding_box(0, 0, 0, 32, 16, 8));
  for (uint64 k = 0; k < 9; ++k)
    for (uint64 j = 0; j < 17; ++j)
      for (uint64 i = 0; i < 33; ++i)
        v(i, j, k, static_cast<double>(i));
  return v;
}

// round(scale * distance from the grid centre in voxels) + offset: integers, so every voxel type
// holds them exactly.
volume integer_field(app &ctx, uint64 n, data_type type, double scale, double offset,
                     const bounding_box &box = bounding_box(0, 0, 0, 1, 1, 1)) {
  volume v(ctx, dimension(n, n, n), type, box);
  const double c = 0.5 * static_cast<double>(n - 1);
  for (uint64 k = 0; k < n; ++k)
    for (uint64 j = 0; j < n; ++j)
      for (uint64 i = 0; i < n; ++i) {
        const double x = i - c, y = j - c, z = k - c;
        v(i, j, k, std::round(scale * std::sqrt(x * x + y * y + z * z)) + offset);
      }
  return v;
}

double peak(const std::vector<double> &f) {
  double m = 0.0;
  for (double v : f)
    m = std::max(m, std::fabs(v));
  return m;
}

bool all_finite(const contour_spectrum &cs) {
  for (const std::vector<double> *f :
       {&cs.isovalue, &cs.surface_area, &cs.min_volume, &cs.max_volume, &cs.gradient})
    for (double v : *f)
      if (!std::isfinite(v))
        return false;
  return std::isfinite(cs.total_volume);
}

void expect_same(const contour_spectrum &a, const contour_spectrum &b) {
  ASSERT_EQ(a.size(), b.size());
  EXPECT_DOUBLE_EQ(a.domain_min, b.domain_min);
  EXPECT_DOUBLE_EQ(a.domain_max, b.domain_max);
  EXPECT_DOUBLE_EQ(a.total_volume, b.total_volume);
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.isovalue[i], b.isovalue[i]) << i;
    EXPECT_DOUBLE_EQ(a.surface_area[i], b.surface_area[i]) << i;
    EXPECT_DOUBLE_EQ(a.min_volume[i], b.min_volume[i]) << i;
    EXPECT_DOUBLE_EQ(a.max_volume[i], b.max_volume[i]) << i;
    EXPECT_DOUBLE_EQ(a.gradient[i], b.gradient[i]) << i;
  }
}

// ---- for the builder tests ----

// splitmix64: the same stream on every platform (the <random> distributions are not).
struct rng {
  std::uint64_t s;
  std::uint64_t next() {
    std::uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
  }
  double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; } // [0, 1)
};

// Uniform noise; integers 0..100 (+ offset) for the integer types, so every type holds them.
volume noise_field(app &ctx, const dimension &d, data_type type, std::uint64_t seed,
                   double offset = 0.0, const bounding_box &box = bounding_box(0, 0, 0, 3, 2, 1)) {
  volume v(ctx, d, type, box);
  rng r{seed};
  const bool real = type == Float || type == Double;
  for (uint64 k = 0; k < d.zdim; ++k)
    for (uint64 j = 0; j < d.ydim; ++j)
      for (uint64 i = 0; i < d.xdim; ++i)
        v(i, j, k, (real ? r.uniform() : std::floor(101.0 * r.uniform())) + offset);
  return v;
}

// sin x sin y sin z over the box: smooth, with interior critical points.
volume smooth_field(app &ctx, const dimension &d) {
  volume v(ctx, d, Float, bounding_box(0, 0, 0, 1, 1, 1));
  for (uint64 k = 0; k < d.zdim; ++k)
    for (uint64 j = 0; j < d.ydim; ++j)
      for (uint64 i = 0; i < d.xdim; ++i)
        v(i, j, k,
          std::sin(0.37 * i + 0.2) * std::sin(0.29 * j + 0.5) * std::sin(0.23 * k + 0.9) +
              0.01 * i);
  return v;
}

// The finite extrema of a volume's voxels, read one at a time (never through its min/max cache).
std::pair<double, double> finite_extrema(const volume &v) {
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (uint64 k = 0; k < v.ZDim(); ++k)
    for (uint64 j = 0; j < v.YDim(); ++j)
      for (uint64 i = 0; i < v.XDim(); ++i) {
        const double x = v(i, j, k);
        if (std::isfinite(x)) {
          lo = std::min(lo, x);
          hi = std::max(hi, x);
        }
      }
  return {lo, hi};
}

bool same_bits(double a, double b) {
  std::uint64_t x, y;
  std::memcpy(&x, &a, sizeof x);
  std::memcpy(&y, &b, sizeof y);
  return x == y;
}

// Bit-for-bit equality of two spectra (EXPECT_DOUBLE_EQ would allow 4 ulps).
void expect_identical(const contour_spectrum &a, const contour_spectrum &b) {
  ASSERT_EQ(a.size(), b.size());
  EXPECT_TRUE(same_bits(a.domain_min, b.domain_min)) << a.domain_min << " vs " << b.domain_min;
  EXPECT_TRUE(same_bits(a.domain_max, b.domain_max)) << a.domain_max << " vs " << b.domain_max;
  EXPECT_TRUE(same_bits(a.total_volume, b.total_volume))
      << a.total_volume << " vs " << b.total_volume;
  const std::pair<const char *, const std::vector<double> contour_spectrum::*> fns[] = {
      {"isovalue", &contour_spectrum::isovalue},
      {"surface_area", &contour_spectrum::surface_area},
      {"min_volume", &contour_spectrum::min_volume},
      {"max_volume", &contour_spectrum::max_volume},
      {"gradient", &contour_spectrum::gradient}};
  for (const auto &[name, fn] : fns) {
    const std::vector<double> &x = a.*fn, &y = b.*fn;
    ASSERT_EQ(x.size(), y.size()) << name;
    std::size_t bad = 0, first = 0;
    for (std::size_t i = 0; i < x.size(); ++i)
      if (!same_bits(x[i], y[i]) && bad++ == 0)
        first = i;
    EXPECT_EQ(bad, 0u) << name << " differs at " << bad << " samples, first [" << first
                       << "]: " << x[first] << " vs " << y[first];
  }
}

int phase_rank(const std::string &phase) {
  if (phase == "domain")
    return 0;
  if (phase == "resampling")
    return 1;
  if (phase == "integrating")
    return 2;
  if (phase == "done")
    return 3;
  return -1;
}

// One stepped run: the phase before each step, and the budget, work_done() and progress() of
// each, and what opt.progress was called with.
struct trace {
  contour_spectrum result;
  std::vector<std::string> phase;
  std::vector<std::size_t> budget;
  std::vector<uint64> work;
  std::vector<double> progress, reported;
};

trace run_stepped(const volume &vol, contour_spectrum_options opt,
                  const std::function<std::size_t()> &next_budget) {
  trace t;
  opt.progress = [&t](double p) {
    t.reported.push_back(p);
    return true;
  };
  contour_spectrum_builder builder(vol, opt);
  EXPECT_FALSE(builder.done());
  EXPECT_EQ(builder.progress(), 0.0);
  EXPECT_STREQ(builder.phase(), "domain");
  EXPECT_EQ(builder.work_done(), 0u);
  for (bool finished = false; !finished;) {
    t.phase.push_back(builder.phase());
    t.budget.push_back(next_budget());
    finished = builder.step(t.budget.back());
    t.work.push_back(builder.work_done());
    t.progress.push_back(builder.progress());
    if (t.phase.size() > 50000000u) {
      ADD_FAILURE() << "the builder does not finish";
      break;
    }
  }
  EXPECT_TRUE(builder.done());
  EXPECT_STREQ(builder.phase(), "done");
  EXPECT_TRUE(builder.step(1)); // a finished builder stays finished, and reports nothing more
  t.result = builder.take();
  return t;
}

// The invariants of every stepped run: each step does at least its budget (unless it finished)
// and at most budget + max_overrun(samples) units (the largest indivisible piece is one cell);
// progress is monotone in [0, 1], ends at exactly 1 and is what opt.progress saw, once per step;
// the phases come in order.
void check_trace(const trace &t, std::size_t samples) {
  const std::size_t steps = t.work.size();
  ASSERT_GT(steps, 0u);
  ASSERT_EQ(t.reported.size(), steps); // once per step
  const uint64 slack = contour_spectrum_builder::max_overrun(samples);
  for (std::size_t s = 0; s < steps; ++s) {
    const uint64 delta = t.work[s] - (s ? t.work[s - 1] : 0);
    const uint64 budget = t.budget[s];
    const bool last = s + 1 == steps;
    if (budget == 0) {
      ASSERT_TRUE(last) << "budget 0 must run to the end (step " << s << ")";
    } else {
      ASSERT_LE(delta, budget + slack) << "step " << s << " overran, phase " << t.phase[s];
      if (!last)
        ASSERT_GE(delta, budget) << "step " << s << " stopped early, phase " << t.phase[s];
    }
    ASSERT_GE(t.progress[s], 0.0);
    ASSERT_LE(t.progress[s], 1.0);
    if (s > 0) {
      ASSERT_GE(t.progress[s], t.progress[s - 1]) << "step " << s;
      ASSERT_GE(phase_rank(t.phase[s]), phase_rank(t.phase[s - 1])) << "step " << s;
    }
    ASSERT_GE(phase_rank(t.phase[s]), 0) << t.phase[s];
    ASSERT_LT(phase_rank(t.phase[s]), 3) << t.phase[s];
    ASSERT_EQ(t.reported[s], t.progress[s]) << "step " << s;
  }
  EXPECT_EQ(t.progress.back(), 1.0);
}

std::function<std::size_t()> fixed_budget(std::size_t b) {
  return [b] { return b; };
}

// Budgets from 1 to ~20000, log-uniform, with an occasional 0 ending the run.
std::function<std::size_t()> random_budget(std::uint64_t seed) {
  auto r = std::make_shared<rng>(rng{seed});
  return [r] {
    if (r->next() % 997 == 0)
      return std::size_t(0);
    return static_cast<std::size_t>(std::exp(r->uniform() * std::log(20000.0))) + 1;
  };
}

struct spectrum_case {
  std::string name;
  volume vol;
  contour_spectrum_options opt;
  // The min/max cache the volume runs with. It cannot ride in `vol`: a volume copy keeps the cache
  // only when both ends are set, so a half-set one would be gone by the time the case runs.
  std::optional<double> cached_min, cached_max;
};

} // namespace

class ContourSpectrumTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(ContourSpectrumTest, SphereDistanceFieldMatchesClosedForm) {
  const volume vol = sphere_field(ctx, 64);
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = 0.0;
  opt.domain_max = 1.0;
  const contour_spectrum cs = compute_contour_spectrum(vol, opt);
  ASSERT_EQ(cs.size(), 256u);
  EXPECT_DOUBLE_EQ(cs.isovalue.front(), 0.0);
  EXPECT_DOUBLE_EQ(cs.isovalue.back(), 1.0);
  EXPECT_NEAR(cs.total_volume, 8.0, 1e-9);
  EXPECT_TRUE(all_finite(cs));

  // The gradient integrates |grad f|^2 in index units: |grad f| = 1 per world unit, so h per voxel.
  const double h = 2.0 / 63.0;
  for (double w : {0.3, 0.5, 0.7, 0.9}) {
    const double area = cs.at(&contour_spectrum::surface_area, w);
    EXPECT_NEAR(area / (4.0 * kPi * w * w), 1.0, 0.03) << "w = " << w;
    EXPECT_NEAR(cs.at(&contour_spectrum::min_volume, w) / (4.0 / 3.0 * kPi * w * w * w), 1.0, 0.03)
        << "w = " << w;
    EXPECT_NEAR(cs.at(&contour_spectrum::gradient, w) / (area * h * h), 1.0, 0.03) << "w = " << w;
  }
  for (std::size_t b = 0; b < cs.size(); ++b) {
    EXPECT_NEAR(cs.max_volume[b], cs.total_volume - cs.min_volume[b], 1e-9) << b;
    if (b > 0) {
      EXPECT_GE(cs.min_volume[b], cs.min_volume[b - 1] - 1e-12) << b;
    }
  }
}

TEST_F(ContourSpectrumTest, LinearRampHasConstantAreaAndLinearVolume) {
  const contour_spectrum cs = compute_contour_spectrum(ramp_field(ctx));
  ASSERT_EQ(cs.size(), 256u);
  EXPECT_DOUBLE_EQ(cs.domain_min, 0.0);
  EXPECT_DOUBLE_EQ(cs.domain_max, 32.0);
  EXPECT_NEAR(cs.total_volume, 4096.0, 1e-9);
  // f = x has unit index-space gradient, so the gradient function equals the area.
  for (std::size_t b = 1; b + 1 < cs.size(); ++b) {
    EXPECT_NEAR(cs.surface_area[b], 128.0, 0.64) << b;
    EXPECT_NEAR(cs.gradient[b], 128.0, 0.64) << b;
    EXPECT_NEAR(cs.min_volume[b], 128.0 * cs.isovalue[b], 4096.0 * 0.002) << b;
    EXPECT_NEAR(cs.max_volume[b], 4096.0 - cs.min_volume[b], 1e-9) << b;
  }
  EXPECT_NEAR(cs.min_volume.front(), 0.0, 1e-9);
  EXPECT_NEAR(cs.max_volume.front(), 4096.0, 1e-9);

  // libContour's area formula halves the middle term of every tetrahedron's cross section, so
  // even a plane comes out short, by up to 25% between voxel values; its volume is unaffected.
  contour_spectrum_options legacy;
  legacy.libcontour_area = true;
  const contour_spectrum lc = compute_contour_spectrum(ramp_field(ctx), legacy);
  double lowest = 128.0;
  for (std::size_t b = 1; b + 1 < lc.size(); ++b) {
    EXPECT_LE(lc.surface_area[b], cs.surface_area[b] + 1e-9) << b;
    EXPECT_DOUBLE_EQ(lc.min_volume[b], cs.min_volume[b]) << b;
    lowest = std::min(lowest, lc.surface_area[b]);
  }
  EXPECT_LT(lowest, 0.8 * 128.0);
}

TEST_F(ContourSpectrumTest, ConstantVolumeIsAllZero) {
  volume vol(ctx, dimension(8, 8, 8), Float, bounding_box(0, 0, 0, 7, 7, 7));
  vol.fill(7.0);
  const contour_spectrum cs = compute_contour_spectrum(vol);
  ASSERT_EQ(cs.size(), 256u);
  EXPECT_TRUE(all_finite(cs));
  EXPECT_NEAR(cs.total_volume, 343.0, 1e-9);
  for (std::size_t b = 0; b < cs.size(); ++b) {
    EXPECT_EQ(cs.isovalue[b], 7.0);
    EXPECT_EQ(cs.surface_area[b], 0.0);
    EXPECT_EQ(cs.min_volume[b], 0.0);
    EXPECT_EQ(cs.max_volume[b], 0.0);
    EXPECT_EQ(cs.gradient[b], 0.0);
  }

  // Over a real domain the sublevel set jumps from empty to everything at the constant.
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = 6.0;
  opt.domain_max = 8.0;
  const contour_spectrum step = compute_contour_spectrum(vol, opt);
  EXPECT_TRUE(all_finite(step));
  for (std::size_t b = 0; b < step.size(); ++b) {
    EXPECT_EQ(step.surface_area[b], 0.0) << b;
    EXPECT_EQ(step.gradient[b], 0.0) << b;
    EXPECT_NEAR(step.min_volume[b], step.isovalue[b] < 7.0 ? 0.0 : 343.0, 1e-9) << b;
  }

  // Under max_dim it reports the volume of the grid max_dim gives.
  volume big(ctx, dimension(40, 40, 40), Float);
  big.fill(1.0);
  contour_spectrum_options capped;
  capped.max_dim = 20;
  capped.world_units = false;
  EXPECT_NEAR(compute_contour_spectrum(big, capped).total_volume, 19.0 * 19.0 * 19.0, 1e-9);

  // An explicitly empty domain is degenerate too, and so is one too narrow to scale.
  opt.domain_min = opt.domain_max = 3.0;
  const contour_spectrum empty = compute_contour_spectrum(ramp_field(ctx), opt);
  EXPECT_EQ(peak(empty.surface_area) + peak(empty.min_volume) + peak(empty.gradient), 0.0);
  EXPECT_EQ(empty.isovalue.front(), 3.0);
  opt.domain_min = 0.0;
  opt.domain_max = 1e-320;
  const contour_spectrum tiny = compute_contour_spectrum(ramp_field(ctx), opt);
  EXPECT_TRUE(all_finite(tiny));
  EXPECT_EQ(peak(tiny.surface_area) + peak(tiny.min_volume) + peak(tiny.gradient), 0.0);
}

TEST_F(ContourSpectrumTest, NonFiniteVoxelsAreSkipped) {
  volume vol = ramp_field(ctx);
  vol(10, 8, 4, std::numeric_limits<double>::quiet_NaN());
  vol(20, 8, 4, std::numeric_limits<double>::infinity());
  const contour_spectrum cs = compute_contour_spectrum(vol);
  EXPECT_TRUE(all_finite(cs));
  EXPECT_DOUBLE_EQ(cs.domain_min, 0.0); // the finite extrema
  EXPECT_DOUBLE_EQ(cs.domain_max, 32.0);
  // Each of the two interior voxels touches 8 unit cells, which drop out.
  EXPECT_NEAR(cs.total_volume, 4096.0 - 16.0, 1e-9);
  EXPECT_NEAR(cs.max_volume.front(), cs.total_volume, 1e-9);
  for (std::size_t b = 1; b + 1 < cs.size(); ++b)
    EXPECT_LE(cs.surface_area[b], 128.0 + 0.64) << b;
}

TEST_F(ContourSpectrumTest, TotalVolumeLeavesOutNonFiniteCellsOnEveryDomain) {
  // A cell with a NaN or infinite corner is left out of total_volume whatever the domain, a
  // degenerate one (all-zero functions) included: the same volume has one total_volume.
  for (data_type t : {Float, Double}) {
    SCOPED_TRACE(data_type_strings[t]);
    volume vol(ctx, dimension(8, 8, 8), t, bounding_box(0, 0, 0, 7, 7, 7));
    vol.fill(7.0);
    vol(3, 4, 5, std::numeric_limits<double>::quiet_NaN()); // interior: 8 unit cells
    vol(0, 0, 0, std::numeric_limits<double>::infinity());  // a corner of the grid: 1 cell
    contour_spectrum_options opt;
    const contour_spectrum flat = compute_contour_spectrum(vol, opt);
    EXPECT_EQ(flat.domain_min, 7.0); // the finite extrema: a degenerate domain
    EXPECT_EQ(flat.domain_max, 7.0);
    EXPECT_NEAR(flat.total_volume, 343.0 - 9.0, 1e-9);
    EXPECT_EQ(peak(flat.surface_area) + peak(flat.min_volume) + peak(flat.max_volume), 0.0);
    opt.has_domain = true;
    opt.domain_min = 6.0;
    opt.domain_max = 8.0;
    EXPECT_NEAR(compute_contour_spectrum(vol, opt).total_volume, 334.0, 1e-9);
    opt.domain_min = opt.domain_max = 7.0; // an explicitly degenerate one
    EXPECT_NEAR(compute_contour_spectrum(vol, opt).total_volume, 334.0, 1e-9);

    // Under max_dim the non-finite voxels spread through the resample the same way on every domain.
    opt.max_dim = 5;
    opt.domain_min = 6.0;
    opt.domain_max = 8.0;
    const double capped = compute_contour_spectrum(vol, opt).total_volume;
    EXPECT_LT(capped, 343.0);
    EXPECT_GT(capped, 0.0);
    opt.domain_min = opt.domain_max = 7.0;
    EXPECT_EQ(compute_contour_spectrum(vol, opt).total_volume, capped);
    opt.has_domain = false;
    EXPECT_EQ(compute_contour_spectrum(vol, opt).total_volume, capped);
  }

  // No finite voxel at all: the domain collapses to [0, 0] and no cell counts.
  volume nan(ctx, dimension(5, 5, 5), Float, bounding_box(0, 0, 0, 4, 4, 4));
  nan.fill(std::numeric_limits<double>::quiet_NaN());
  const contour_spectrum none = compute_contour_spectrum(nan);
  EXPECT_TRUE(all_finite(none));
  EXPECT_EQ(none.total_volume, 0.0);
  contour_spectrum_options unit;
  unit.has_domain = true;
  EXPECT_EQ(compute_contour_spectrum(nan, unit).total_volume, 0.0);

  // Integer voxels are always finite: the whole grid.
  volume ints(ctx, dimension(8, 8, 8), Int, bounding_box(0, 0, 0, 7, 7, 7));
  ints.fill(7.0);
  EXPECT_NEAR(compute_contour_spectrum(ints).total_volume, 343.0, 1e-9);
}

TEST_F(ContourSpectrumTest, DomainsTooWideToScaleThrow) {
  // The gradient goes back to data units by (width / 255)^2, which overflows past 255 *
  // sqrt(DBL_MAX) (~3.4e156), as the width itself does past DBL_MAX: such a domain is refused up
  // front (it came back as NaN isovalues and NaN or infinite gradients).
  const double big = std::numeric_limits<double>::max();
  const volume ramp = ramp_field(ctx);
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = -big;
  opt.domain_max = big; // the width overflows
  EXPECT_THROW(compute_contour_spectrum(ramp, opt), contour_spectrum_error);
  opt.domain_min = 0.0;
  opt.domain_max = 4e156; // a finite width whose squared scale overflows
  EXPECT_THROW(compute_contour_spectrum(ramp, opt), contour_spectrum_error);
  contour_spectrum_builder given(ramp, opt);
  EXPECT_THROW(given.step(1), contour_spectrum_error); // a given domain: on the first step
  EXPECT_TRUE(given.done());

  // A Float64 raster whose nodata value is -DBL_MAX, next to ordinary data: the default domain
  // spans ~DBL_MAX.
  volume nodata(ctx, dimension(6, 6, 6), Double, bounding_box(0, 0, 0, 5, 5, 5));
  for (uint64 k = 0; k < 6; ++k)
    for (uint64 j = 0; j < 6; ++j)
      for (uint64 i = 0; i < 6; ++i)
        nodata(i, j, k, static_cast<double>(i + j + k));
  nodata(0, 0, 0, std::numeric_limits<double>::lowest());
  try {
    compute_contour_spectrum(nodata);
    ADD_FAILURE() << "a domain of width ~DBL_MAX was integrated";
  } catch (const contour_spectrum_error &e) {
    EXPECT_NE(std::string(e.what()).find("too wide"), std::string::npos) << e.what();
  }

  // Narrower domains scale, but a gradient can still overflow (data rising 1e154 per voxel:
  // |grad f|^2 = 1e308 over an area of 128): it is refused when reached, so a returned spectrum
  // is always finite.
  const auto scaled_ramp = [&](double slope) {
    volume v(ctx, dimension(33, 17, 9), Double, bounding_box(0, 0, 0, 32, 16, 8));
    for (uint64 k = 0; k < 9; ++k)
      for (uint64 j = 0; j < 17; ++j)
        for (uint64 i = 0; i < 33; ++i)
          v(i, j, k, slope * static_cast<double>(i));
    return v;
  };
  EXPECT_THROW(compute_contour_spectrum(scaled_ramp(1e154)), contour_spectrum_error);

  // One that fits is computed: 1e150 per voxel, a gradient of 128e300.
  const contour_spectrum tall = compute_contour_spectrum(scaled_ramp(1e150));
  const contour_spectrum flat = compute_contour_spectrum(ramp_field(ctx));
  EXPECT_TRUE(all_finite(tall));
  ASSERT_EQ(tall.size(), flat.size());
  for (std::size_t b = 1; b + 1 < tall.size(); ++b) {
    EXPECT_NEAR(tall.surface_area[b], flat.surface_area[b], 1e-9 * 128.0) << b;
    EXPECT_NEAR(tall.gradient[b] / 1e300, flat.gradient[b], 1e-9 * 128.0) << b;
  }
}

TEST_F(ContourSpectrumTest, RejectsBadInput) {
  volume flat(ctx, dimension(8, 8, 1), Float);
  volume thin(ctx, dimension(1, 8, 8), Float);
  volume narrow(ctx, dimension(8, 1, 8), Float);
  EXPECT_THROW(compute_contour_spectrum(flat), contour_spectrum_error);
  EXPECT_THROW(compute_contour_spectrum(thin), contour_spectrum_error);
  EXPECT_THROW(compute_contour_spectrum(narrow), contour_spectrum_error);
  EXPECT_THROW(compute_contour_spectrum_libcontour(flat), contour_spectrum_error);

  const volume ok = ramp_field(ctx);
  contour_spectrum_options opt;
  opt.samples = 1;
  EXPECT_THROW(compute_contour_spectrum(ok, opt), contour_spectrum_error);
  opt.samples = 16;
  opt.has_domain = true;
  opt.domain_min = 0.0;
  opt.domain_max = std::numeric_limits<double>::infinity();
  EXPECT_THROW(compute_contour_spectrum(ok, opt), contour_spectrum_error);
  opt.domain_max = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(compute_contour_spectrum(ok, opt), contour_spectrum_error);
}

TEST_F(ContourSpectrumTest, VoxelTypesAgree) {
  // Values 0..117 fit every type, so all of them must give bit-for-bit the same spectrum.
  const contour_spectrum ref = compute_contour_spectrum(integer_field(ctx, 16, Double, 9.0, 0.0));
  EXPECT_DOUBLE_EQ(ref.domain_min, 8.0); // no voxel at the centre of an even grid
  EXPECT_GT(peak(ref.surface_area), 0.0);
  for (data_type t : {UChar, UShort, UInt, Float, UInt64, Char, Int, Int64}) {
    SCOPED_TRACE(data_type_strings[t]);
    expect_same(compute_contour_spectrum(integer_field(ctx, 16, t, 9.0, 0.0)), ref);
  }

  // Shifted below zero (signed types): the isovalues shift, the functions do not change.
  for (data_type t : {Char, Int, Int64, Float, Double}) {
    SCOPED_TRACE(data_type_strings[t]);
    const contour_spectrum cs = compute_contour_spectrum(integer_field(ctx, 16, t, 9.0, -60.0));
    ASSERT_EQ(cs.size(), ref.size());
    EXPECT_DOUBLE_EQ(cs.domain_min, ref.domain_min - 60.0);
    for (std::size_t b = 0; b < cs.size(); ++b) {
      EXPECT_NEAR(cs.isovalue[b], ref.isovalue[b] - 60.0, 1e-12);
      EXPECT_DOUBLE_EQ(cs.surface_area[b], ref.surface_area[b]);
      EXPECT_DOUBLE_EQ(cs.min_volume[b], ref.min_volume[b]);
      EXPECT_DOUBLE_EQ(cs.gradient[b], ref.gradient[b]);
    }
  }
}

TEST_F(ContourSpectrumTest, WorldUnitsScaleAreaAndVolume) {
  const volume unit = integer_field(ctx, 24, Float, 4.0, 0.0, bounding_box(0, 0, 0, 1, 1, 1));
  const volume twice = integer_field(ctx, 24, Float, 4.0, 0.0, bounding_box(5, 5, 5, 7, 7, 7));
  const contour_spectrum a = compute_contour_spectrum(unit);
  const contour_spectrum b = compute_contour_spectrum(twice);
  ASSERT_EQ(a.size(), b.size());
  EXPECT_NEAR(a.total_volume, 1.0, 1e-12);
  EXPECT_NEAR(b.total_volume, 8.0, 1e-12);
  const double ta = peak(a.surface_area), tv = peak(a.min_volume), tg = peak(a.gradient);
  ASSERT_GT(ta, 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_NEAR(b.surface_area[i], 4.0 * a.surface_area[i], 1e-9 * ta) << i;
    EXPECT_NEAR(b.min_volume[i], 8.0 * a.min_volume[i], 1e-9 * tv) << i;
    EXPECT_NEAR(b.max_volume[i], 8.0 * a.max_volume[i], 1e-9 * tv) << i;
    // The gradient weight is in index units whatever the box; only the area scales it.
    EXPECT_NEAR(b.gradient[i], 4.0 * a.gradient[i], 1e-9 * tg) << i;
  }

  // In index units the box does not matter, and one voxel spacing (1/23 here) is the unit.
  contour_spectrum_options idx;
  idx.world_units = false;
  const contour_spectrum ia = compute_contour_spectrum(unit, idx);
  const contour_spectrum ib = compute_contour_spectrum(twice, idx);
  EXPECT_NEAR(ia.total_volume, 23.0 * 23.0 * 23.0, 1e-9);
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_NEAR(ia.surface_area[i], ib.surface_area[i], 1e-9 * ta * 23.0 * 23.0) << i;
    EXPECT_NEAR(ia.surface_area[i], a.surface_area[i] * 23.0 * 23.0, 1e-9 * ta * 23.0 * 23.0) << i;
    EXPECT_NEAR(ia.min_volume[i], a.min_volume[i] * 23.0 * 23.0 * 23.0, 1e-9 * tv * 12167.0) << i;
  }
}

TEST_F(ContourSpectrumTest, NarrowDomainCountsWhatLiesBelowIt) {
  const volume vol = ramp_field(ctx);
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = 8.0;
  opt.domain_max = 16.0;
  const contour_spectrum cs = compute_contour_spectrum(vol, opt);
  const contour_spectrum full = compute_contour_spectrum(vol);
  ASSERT_EQ(cs.size(), 256u);
  EXPECT_DOUBLE_EQ(cs.isovalue.front(), 8.0);
  EXPECT_DOUBLE_EQ(cs.isovalue.back(), 16.0);
  for (std::size_t b = 0; b < cs.size(); ++b) {
    const double w = cs.isovalue[b];
    EXPECT_NEAR(cs.surface_area[b], 128.0, 0.64) << b;
    // The half of the volume below the domain is counted (libContour dropped it).
    EXPECT_NEAR(cs.min_volume[b], 128.0 * w, 4096.0 * 0.002) << b;
    EXPECT_NEAR(cs.surface_area[b], full.at(&contour_spectrum::surface_area, w), 0.64) << b;
    EXPECT_NEAR(cs.min_volume[b], full.at(&contour_spectrum::min_volume, w), 4096.0 * 0.002) << b;
  }
  // A reversed domain is the same domain.
  std::swap(opt.domain_min, opt.domain_max);
  expect_same(compute_contour_spectrum(vol, opt), cs);
}

TEST_F(ContourSpectrumTest, AnySampleCount) {
  const volume vol = sphere_field(ctx, 32);
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = 0.0;
  opt.domain_max = 1.0;
  opt.samples = 64;
  const contour_spectrum coarse = compute_contour_spectrum(vol, opt);
  opt.samples = 1000;
  const contour_spectrum fine = compute_contour_spectrum(vol, opt);
  ASSERT_EQ(coarse.size(), 64u);
  ASSERT_EQ(fine.size(), 1000u);
  for (const contour_spectrum *cs : {&coarse, &fine}) {
    EXPECT_EQ(cs->surface_area.size(), cs->size());
    EXPECT_EQ(cs->min_volume.size(), cs->size());
    EXPECT_EQ(cs->max_volume.size(), cs->size());
    EXPECT_EQ(cs->gradient.size(), cs->size());
    EXPECT_DOUBLE_EQ(cs->isovalue.back(), 1.0);
  }
  // Each sample is evaluated on its own, so where the two grids meet (every 1/9) they agree.
  for (std::size_t m = 0; m <= 9; ++m) {
    const std::size_t c = 7 * m, f = 111 * m;
    EXPECT_NEAR(coarse.isovalue[c], fine.isovalue[f], 1e-12);
    EXPECT_NEAR(coarse.surface_area[c], fine.surface_area[f], 1e-9 * peak(fine.surface_area)) << m;
    EXPECT_NEAR(coarse.min_volume[c], fine.min_volume[f], 1e-9 * peak(fine.min_volume)) << m;
    EXPECT_NEAR(coarse.gradient[c], fine.gradient[f], 1e-9 * peak(fine.gradient)) << m;
  }
}

TEST_F(ContourSpectrumTest, ProgressAndCancel) {
  // compute_contour_spectrum steps its builder 2^22 units at a time when it has a callback and
  // reports once per step, after it: monotone, ending with 1.
  const volume vol = sphere_field(ctx, 48);
  contour_spectrum_options opt;
  opt.samples = 1000;
  std::vector<double> seen;
  opt.progress = [&](double p) {
    seen.push_back(p);
    return true;
  };
  const contour_spectrum reported = compute_contour_spectrum(vol, opt);
  ASSERT_GE(seen.size(), 3u);
  EXPECT_GT(seen.front(), 0.0);
  EXPECT_EQ(seen.back(), 1.0);
  EXPECT_TRUE(std::is_sorted(seen.begin(), seen.end()));
  opt.progress = nullptr; // one step to the end: the same result
  expect_identical(compute_contour_spectrum(vol, opt), reported);

  seen.clear();
  opt.progress = [&](double p) {
    seen.push_back(p);
    return seen.size() < 3;
  };
  EXPECT_THROW(compute_contour_spectrum(vol, opt), contour_spectrum_error);
  EXPECT_EQ(seen.size(), 3u);
}

TEST_F(ContourSpectrumTest, AtInterpolatesAndClamps) {
  contour_spectrum cs;
  cs.domain_min = 0.0;
  cs.domain_max = 10.0;
  cs.isovalue = {0.0, 5.0, 10.0};
  cs.surface_area = {0.0, 10.0, 30.0};
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 2.5), 5.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 5.0), 10.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 7.5), 20.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 10.0), 30.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 11.0), 30.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, std::nan("")), 0.0);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::isovalue, 7.5), 7.5);
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::gradient, 5.0), 0.0); // empty function
  EXPECT_DOUBLE_EQ(cs.at(nullptr, 5.0), 0.0);
  cs.domain_max = cs.domain_min; // degenerate: the first sample
  EXPECT_DOUBLE_EQ(cs.at(&contour_spectrum::surface_area, 4.0), 0.0);
  EXPECT_DOUBLE_EQ(contour_spectrum().at(&contour_spectrum::min_volume, 1.0), 0.0);
}

TEST_F(ContourSpectrumTest, MaxDimResamplesACopy) {
  const volume vol = sphere_field(ctx, 64);
  const void *before = vol.data_ptr();
  const double probe = vol(10, 20, 30);
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_min = 0.0;
  opt.domain_max = 1.0;
  opt.max_dim = 32;
  const contour_spectrum resampled = compute_contour_spectrum(vol, opt);
  EXPECT_EQ(vol.XDim(), 64u);
  EXPECT_EQ(static_cast<const void *>(vol.data_ptr()), before);
  EXPECT_EQ(vol(10, 20, 30), probe);
  EXPECT_NEAR(resampled.total_volume, 8.0, 1e-9);
  const double w = 0.5;
  EXPECT_NEAR(resampled.at(&contour_spectrum::surface_area, w) / (4.0 * kPi * w * w), 1.0, 0.03);
  EXPECT_NEAR(resampled.at(&contour_spectrum::min_volume, w) / (4.0 / 3.0 * kPi * w * w * w), 1.0,
              0.03);

  // In index units the grid actually integrated sets the unit: 31 cells per axis.
  opt.world_units = false;
  EXPECT_NEAR(compute_contour_spectrum(vol, opt).total_volume, 31.0 * 31.0 * 31.0, 1e-9);
  // A cap at or above every axis is a no-op.
  opt.world_units = true;
  opt.max_dim = 0;
  const contour_spectrum full = compute_contour_spectrum(vol, opt);
  opt.max_dim = 64;
  expect_same(compute_contour_spectrum(vol, opt), full);
}

TEST_F(ContourSpectrumTest, ConcurrentCallsOnOneVolume) {
  const volume vol = integer_field(ctx, 20, UShort, 5.0, 3.0);
  const contour_spectrum serial = compute_contour_spectrum(vol);
  contour_spectrum r1, r2;
  std::thread t1([&] { r1 = compute_contour_spectrum(vol); });
  std::thread t2([&] { r2 = compute_contour_spectrum(vol); });
  t1.join();
  t2.join();
  expect_same(r1, serial);
  expect_same(r2, serial);
  EXPECT_FALSE(vol.minIsSet()); // the default domain did not touch the lazy min/max cache
}

TEST_F(ContourSpectrumTest, MatchesLibContour) {
  // uchar data spanning exactly 0..255: the libContour domain (the data extrema) is then the
  // default domain, and the map onto the integration scale is the identity.
  const double scale = 255.0 / (8.0 * std::sqrt(3.0));
  const volume vol = integer_field(ctx, 17, UChar, scale, 0.0, bounding_box(-1, -1, -1, 1, 1, 1));
  if (!contour_spectrum_libcontour_available()) {
    EXPECT_THROW(compute_contour_spectrum_libcontour(vol), contour_spectrum_error);
    GTEST_SKIP() << "libcvc was built without CVC_ENABLE_MESHER";
  }
  for (bool world : {true, false}) {
    SCOPED_TRACE(world ? "world units" : "index units");
    contour_spectrum_options opt;
    opt.world_units = world;
    opt.libcontour_area = true;
    const contour_spectrum native = compute_contour_spectrum(vol, opt);
    const contour_spectrum legacy = compute_contour_spectrum_libcontour(vol, world);
    ASSERT_EQ(native.size(), 256u);
    ASSERT_EQ(legacy.size(), 256u);
    EXPECT_DOUBLE_EQ(legacy.domain_min, 0.0);
    EXPECT_DOUBLE_EQ(legacy.domain_max, 255.0);
    EXPECT_NEAR(legacy.total_volume, native.total_volume, 1e-9 * native.total_volume);
    // The only difference left is libContour's float arithmetic. Area and gradient (a sample sums
    // only the few tetrahedra its isosurface crosses) agree to ~1e-5 of their peak. The volumes
    // add every tetrahedron below a sample into one float, 4096 cells x 5 here, which drifts
    // ~1e-4 (libContour's min_volume at the top sample misses the grid volume by that much, and
    // more on larger grids): hence 1e-4 and 1e-3, each about 10x the drift measured.
    const double tol_area = 1e-4, tol_volume = 1e-3;
    const double pa = peak(native.surface_area), pv = peak(native.min_volume),
                 pg = peak(native.gradient);
    for (std::size_t b = 0; b < 256; ++b) {
      EXPECT_NEAR(native.isovalue[b], legacy.isovalue[b], 1e-4) << b;
      EXPECT_NEAR(native.surface_area[b], legacy.surface_area[b], tol_area * pa) << b;
      EXPECT_NEAR(native.gradient[b], legacy.gradient[b], tol_area * pg) << b;
      EXPECT_NEAR(native.min_volume[b], legacy.min_volume[b], tol_volume * pv) << b;
      // libContour's "Max Volume" is min_volume.back() - min_volume.
      EXPECT_NEAR(native.min_volume.back() - native.min_volume[b], legacy.max_volume[b],
                  tol_volume * pv)
          << b;
    }

    // The exact area only adds the half of the middle term libContour drops.
    opt.libcontour_area = false;
    const contour_spectrum exact = compute_contour_spectrum(vol, opt);
    for (std::size_t b = 0; b < 256; ++b) {
      EXPECT_GE(exact.surface_area[b], native.surface_area[b]) << b;
      EXPECT_DOUBLE_EQ(exact.min_volume[b], native.min_volume[b]) << b;
    }
  }

  // The other voxel types reach libContour as float, with the same answer.
  const contour_spectrum as_uchar = compute_contour_spectrum_libcontour(vol);
  for (data_type t : {UShort, Float, Double}) {
    SCOPED_TRACE(data_type_strings[t]);
    const contour_spectrum cs = compute_contour_spectrum_libcontour(
        integer_field(ctx, 17, t, scale, 0.0, bounding_box(-1, -1, -1, 1, 1, 1)));
    ASSERT_EQ(cs.size(), as_uchar.size());
    for (std::size_t b = 0; b < cs.size(); ++b) {
      EXPECT_FLOAT_EQ(static_cast<float>(cs.surface_area[b]),
                      static_cast<float>(as_uchar.surface_area[b]));
      EXPECT_FLOAT_EQ(static_cast<float>(cs.min_volume[b]),
                      static_cast<float>(as_uchar.min_volume[b]));
    }
  }

  // A constant volume would make libContour divide by zero; the oracle answers with zeros.
  volume flat(ctx, dimension(6, 6, 6), UChar);
  flat.fill(9.0);
  const contour_spectrum zeros = compute_contour_spectrum_libcontour(flat);
  EXPECT_TRUE(all_finite(zeros));
  EXPECT_EQ(peak(zeros.surface_area) + peak(zeros.min_volume) + peak(zeros.gradient), 0.0);
}

// ---- contour_spectrum_builder ------------------------------------------------------------------

TEST_F(ContourSpectrumTest, BuilderMatchesOneShotOnEverySchedule) {
  std::vector<spectrum_case> cases;
  const auto add = [&](std::string name, const volume &vol, const contour_spectrum_options &opt) {
    cases.push_back({std::move(name), vol, opt});
  };
  const contour_spectrum_options plain;
  contour_spectrum_options o;

  add("noise", noise_field(ctx, dimension(13, 11, 9), Float, 1), plain);
  o = {};
  o.max_dim = 6;
  add("noise, max_dim 6", noise_field(ctx, dimension(13, 11, 9), Float, 2), o);
  o = {};
  o.has_domain = true;
  o.domain_min = 0.0;
  o.domain_max = 1.0;
  add("sphere", sphere_field(ctx, 16), o);
  o.max_dim = 9;
  o.world_units = false;
  add("sphere, max_dim 9, index units", sphere_field(ctx, 16), o);
  o = {};
  o.libcontour_area = true;
  add("ramp, libcontour area", ramp_field(ctx), o);
  o = {};
  o.has_domain = true;
  o.domain_min = 16.0; // reversed
  o.domain_max = 8.0;
  add("ramp, narrow reversed domain", ramp_field(ctx), o);
  const volume smooth = smooth_field(ctx, dimension(17, 15, 13));
  add("smooth", smooth, plain);
  o = {};
  o.max_dim = 2;
  add("smooth, max_dim 2", smooth, o);
  o.max_dim = 1; // acts as 2
  add("smooth, max_dim 1", smooth, o);
  o.max_dim = 16;
  add("smooth, one axis capped", smooth_field(ctx, dimension(40, 10, 10)), o);
  add("1D along x", noise_field(ctx, dimension(40, 2, 2), Double, 3), plain);
  add("1D along y", noise_field(ctx, dimension(2, 40, 2), Double, 4), plain);
  add("1D along z", noise_field(ctx, dimension(2, 2, 40), Double, 5), plain);
  add("2D", noise_field(ctx, dimension(30, 30, 2), UShort, 6), plain);
  add("thin slab", noise_field(ctx, dimension(33, 17, 3), Float, 7), plain);
  for (std::size_t samples : {2, 7, 300}) {
    o = {};
    o.samples = samples;
    add(std::to_string(samples) + " samples", smooth_field(ctx, dimension(9, 9, 9)), o);
  }

  // Constants: a degenerate default domain (the scan, then the resample and the cells' check for
  // a non-finite corner, but no integration), and a real one.
  volume flat(ctx, dimension(8, 8, 8), Float, bounding_box(0, 0, 0, 7, 7, 7));
  flat.fill(7.0);
  add("constant", flat, plain);
  o = {};
  o.max_dim = 4;
  add("constant, max_dim 4", flat, o);
  o = {};
  o.has_domain = true;
  o.domain_min = 6.0;
  o.domain_max = 8.0;
  add("constant, domain", flat, o);
  o.max_dim = 5;
  add("constant, domain, max_dim 5", flat, o);

  volume holes = ramp_field(ctx);
  holes(10, 8, 4, std::numeric_limits<double>::quiet_NaN());
  holes(20, 8, 4, std::numeric_limits<double>::infinity());
  holes(0, 0, 0, -std::numeric_limits<double>::infinity());
  add("NaN and inf", holes, plain);
  o = {};
  o.max_dim = 12;
  add("NaN and inf, max_dim 12", holes, o);

  // The min/max cache: both set (no scan), or only one end (a scan for the other).
  add("cached min/max", smooth_field(ctx, dimension(10, 9, 8)), plain);
  cases.back().cached_min = -2.0;
  cases.back().cached_max = 3.0;
  add("cached min only", smooth_field(ctx, dimension(10, 9, 8)), plain);
  cases.back().cached_min = -2.0;
  add("cached max only", smooth_field(ctx, dimension(9, 10, 8)), plain);
  cases.back().cached_max = 3.0;

  // A degenerate domain with non-finite voxels: the functions are zero, but total_volume still
  // leaves out the cells they touch (the grid is checked, and resampled first under max_dim).
  volume flat_holes(ctx, dimension(8, 8, 8), Double, bounding_box(0, 0, 0, 7, 7, 7));
  flat_holes.fill(7.0);
  flat_holes(3, 4, 5, std::numeric_limits<double>::quiet_NaN());
  flat_holes(7, 7, 0, std::numeric_limits<double>::infinity());
  add("constant with NaN and inf", flat_holes, plain);
  o = {};
  o.max_dim = 5;
  add("constant with NaN and inf, max_dim 5", flat_holes, o);
  volume all_nan(ctx, dimension(6, 5, 4), Float);
  all_nan.fill(std::numeric_limits<double>::quiet_NaN());
  add("all NaN", all_nan, plain);

  // Every voxel type (64 samples: noise touches most of them in every cell).
  for (data_type t : {UChar, UShort, UInt, Float, Double, UInt64, Char, Int, Int64}) {
    const std::string type = data_type_strings[t];
    o = {};
    o.samples = 64;
    add(type, noise_field(ctx, dimension(11, 10, 9), t, 10 + t), o);
    o.max_dim = 7;
    add(type + ", max_dim 7", noise_field(ctx, dimension(11, 10, 9), t, 20 + t), o);
  }
  for (data_type t : {Char, Int, Int64, Float, Double}) {
    o = {};
    o.samples = 64;
    o.max_dim = 6;
    add(std::string(data_type_strings[t]) + " below zero, max_dim 6",
        noise_field(ctx, dimension(9, 10, 11), t, 30 + t, -60.0), o);
  }

  std::size_t half_cached = 0;
  for (const spectrum_case &c : cases) {
    SCOPED_TRACE(c.name);
    // The volume every run of the case reads, by reference: c.vol's voxels with c's cache.
    volume vol(c.vol);
    if (c.cached_min)
      vol.min(*c.cached_min);
    if (c.cached_max)
      vol.max(*c.cached_max);
    half_cached += vol.minIsSet() != vol.maxIsSet();
    const contour_spectrum ref = compute_contour_spectrum(vol, c.opt);
    ASSERT_EQ(ref.size(), c.opt.samples);
    if (!c.opt.has_domain && (vol.minIsSet() || vol.maxIsSet())) {
      // The default domain takes a set end from the cache and scans for an unset one.
      const std::pair<double, double> scanned = finite_extrema(vol);
      EXPECT_EQ(ref.domain_min, vol.minIsSet() ? vol.min() : scanned.first);
      EXPECT_EQ(ref.domain_max, vol.maxIsSet() ? vol.max() : scanned.second);
    }
    for (std::size_t budget : {1, 7, 64, 4096, 0}) {
      SCOPED_TRACE("budget " + std::to_string(budget));
      const trace t = run_stepped(vol, c.opt, fixed_budget(budget));
      check_trace(t, c.opt.samples);
      expect_identical(t.result, ref);
      if (budget == 0)
        EXPECT_EQ(t.work.size(), 1u);
    }
    for (std::uint64_t seed : {11u, 12u}) {
      SCOPED_TRACE("random budgets, seed " + std::to_string(seed));
      const trace t = run_stepped(vol, c.opt, random_budget(seed + 1000 * c.name.size()));
      check_trace(t, c.opt.samples);
      expect_identical(t.result, ref);
    }
  }
  // The min-only and max-only cases ran with their half-set cache (the default domain's other
  // branch), not with none.
  EXPECT_EQ(half_cached, 2u);
}

TEST_F(ContourSpectrumTest, EveryPhaseIsSliced) {
  // A phase with W units of work spans at least W / (budget + max_overrun) steps (a step does at
  // most that much, and the one entering a phase may finish the previous one), so no phase, and
  // nothing within a z-slab or within a row, runs as one blocking piece. The work of the scan (1
  // per voxel) and of the resample (8 per voxel) is known exactly, the integration's from below
  // (1 per voxel converted and 200 per cell, plus 2 * samples).
  struct phase_case {
    std::string name;
    volume vol;
    contour_spectrum_options opt;
    uint64 domain, resampling, integrating;
  };
  std::vector<phase_case> cases;
  contour_spectrum_options o;
  o.samples = 16;
  o.max_dim = 24;
  cases.push_back({"scan, resample, integrate", noise_field(ctx, dimension(40, 36, 20), Float, 41),
                   o, 40 * 36 * 20, 8 * 24 * 24 * 20, 24 * 24 * 20 + 200 * 23 * 23 * 19 + 32});
  o = {};
  o.has_domain = true;
  o.domain_min = -1.0;
  o.domain_max = 1.0;
  cases.push_back({"one z-slab", smooth_field(ctx, dimension(220, 180, 2)), o, 0, 0,
                   220 * 180 * 2 + 200 * 219 * 179 + 512});
  o.samples = 16;
  cases.push_back({"one row", noise_field(ctx, dimension(3000, 2, 2), Double, 42), o, 0, 0,
                   3000 * 4 + 200 * 2999 + 32});

  for (const phase_case &c : cases) {
    SCOPED_TRACE(c.name);
    const contour_spectrum ref = compute_contour_spectrum(c.vol, c.opt);
    for (std::size_t budget : {1, 7, 64, 4096}) {
      SCOPED_TRACE("budget " + std::to_string(budget));
      const trace t = run_stepped(c.vol, c.opt, fixed_budget(budget));
      check_trace(t, c.opt.samples);
      expect_identical(t.result, ref);
      uint64 domain = 0, resampling = 0, integrating = 0;
      for (const std::string &p : t.phase) {
        domain += p == "domain";
        resampling += p == "resampling";
        integrating += p == "integrating";
      }
      const uint64 most = budget + contour_spectrum_builder::max_overrun(c.opt.samples);
      EXPECT_GE((domain + 1) * most, c.domain) << domain << " domain steps";
      EXPECT_GE((resampling + 1) * most, c.resampling) << resampling << " resampling steps";
      EXPECT_GE((integrating + 1) * most, c.integrating) << integrating << " integrating steps";
      // A phase with more than two steps' worth of work took more than one step.
      const std::pair<uint64, uint64> per_phase[] = {
          {domain, c.domain}, {resampling, c.resampling}, {integrating, c.integrating}};
      for (const auto &[steps, work] : per_phase)
        if (work > 2 * most)
          EXPECT_GE(steps, 2u) << work << " units";
      EXPECT_EQ(resampling > 0, c.resampling > 0);
      EXPECT_GE(t.work.back(), c.domain + c.resampling + c.integrating);
    }
  }
}

TEST_F(ContourSpectrumTest, ResampleMatchesVoxelsResize) {
  // max_dim resamples into the builder's own buffer, a slice at a time, with the samples where
  // voxels::resize puts them and its conversion to the voxel type: the spectrum of a volume
  // resized by voxels::resize first is bit-identical.
  struct shape {
    dimension from;
    uint64 cap;
  };
  const shape shapes[] = {{dimension(17, 13, 11), 8},
                          {dimension(9, 9, 9), 5},
                          {dimension(12, 30, 7), 10},
                          {dimension(5, 6, 40), 2},
                          {dimension(33, 3, 4), 31}};
  for (data_type t : {UChar, UShort, UInt, Float, Double, UInt64, Char, Int, Int64}) {
    for (const shape &s : shapes) {
      SCOPED_TRACE(std::string(data_type_strings[t]) + " " + std::to_string(s.from.xdim) + "x" +
                   std::to_string(s.from.ydim) + "x" + std::to_string(s.from.zdim) + " to " +
                   std::to_string(s.cap));
      const volume vol = noise_field(ctx, s.from, t, 7 * t + s.cap);
      contour_spectrum_options opt;
      opt.has_domain = true; // the resized copy's own extrema would differ
      opt.domain_min = 0.0;
      opt.domain_max = t == Float || t == Double ? 1.0 : 100.0;
      opt.max_dim = s.cap;
      const contour_spectrum mine = compute_contour_spectrum(vol, opt);
      volume resized(vol);
      resized.resize(dimension(std::min(s.from.xdim, s.cap), std::min(s.from.ydim, s.cap),
                               std::min(s.from.zdim, s.cap)));
      opt.max_dim = 0;
      expect_identical(mine, compute_contour_spectrum(resized, opt));
    }
  }
  // Non-finite voxels propagate through both the same way.
  volume holes = sphere_field(ctx, 20);
  holes(3, 4, 5, std::numeric_limits<double>::quiet_NaN());
  holes(19, 19, 19, std::numeric_limits<double>::infinity());
  contour_spectrum_options opt;
  opt.has_domain = true;
  opt.domain_max = 1.0;
  opt.max_dim = 13;
  const contour_spectrum mine = compute_contour_spectrum(holes, opt);
  volume resized(holes);
  resized.resize(dimension(13, 13, 13));
  opt.max_dim = 0;
  expect_identical(mine, compute_contour_spectrum(resized, opt));
  EXPECT_LT(mine.total_volume, 8.0);
}

TEST_F(ContourSpectrumTest, BuilderCancelsThroughProgress) {
  const volume vol = noise_field(ctx, dimension(20, 18, 16), Float, 99);
  for (const char *where : {"domain", "resampling", "integrating"}) {
    SCOPED_TRACE(where);
    bool cancel = false;
    int calls = 0;
    contour_spectrum_options opt;
    opt.max_dim = 12;
    opt.progress = [&](double) {
      ++calls;
      return !cancel;
    };
    contour_spectrum_builder b(vol, opt);
    while (std::string(b.phase()) != where)
      ASSERT_FALSE(b.step(64));
    ASSERT_FALSE(b.step(64));
    const int before = calls;
    cancel = true;
    EXPECT_THROW(b.step(64), contour_spectrum_error);
    EXPECT_EQ(calls, before + 1);
    EXPECT_TRUE(b.done());
    EXPECT_STREQ(b.phase(), "done");
    const double frozen = b.progress();
    EXPECT_GT(frozen, 0.0);
    EXPECT_LT(frozen, 1.0);
    // Every later step rethrows the same error, without calling back.
    try {
      b.step(64);
      ADD_FAILURE() << "a cancelled builder stepped";
    } catch (const contour_spectrum_error &e) {
      EXPECT_NE(std::string(e.what()).find("cancelled"), std::string::npos) << e.what();
    }
    EXPECT_EQ(calls, before + 1);
    EXPECT_THROW(b.take(), contour_spectrum_error);
    EXPECT_EQ(b.progress(), frozen);
  }
}

TEST_F(ContourSpectrumTest, BuilderDestroyedMidway) {
  // Destroying a builder in any phase releases everything (the round-2 harness also runs this
  // under an allocation count and ASan); the voxels are then shared with their owner only.
  const volume vol = noise_field(ctx, dimension(24, 20, 18), Float, 5);
  const long owners = vol.data_as_shared_array().use_count();
  contour_spectrum_options opt;
  opt.max_dim = 15;
  for (const char *where : {"domain", "resampling", "integrating"}) {
    SCOPED_TRACE(where);
    auto b = std::make_unique<contour_spectrum_builder>(vol, opt);
    EXPECT_GT(vol.data_as_shared_array().use_count(), owners); // the builder's shallow copy
    while (std::string(b->phase()) != where)
      ASSERT_FALSE(b->step(100));
    ASSERT_FALSE(b->step(100));
    b.reset();
    EXPECT_EQ(vol.data_as_shared_array().use_count(), owners);
  }
  // A finished builder no longer holds the volume, even before its spectrum is taken.
  contour_spectrum_builder b(vol, opt);
  while (!b.step(5000)) {
  }
  EXPECT_EQ(vol.data_as_shared_array().use_count(), owners);
  const uint64 total = b.work_done();
  EXPECT_EQ(b.take().size(), 256u);
  // Nor does one in its final pass (the last `samples` units), which it may be destroyed in.
  contour_spectrum_builder last(vol, opt);
  last.step(static_cast<std::size_t>(total - 100));
  EXPECT_STREQ(last.phase(), "integrating");
  EXPECT_FALSE(last.done());
  EXPECT_EQ(vol.data_as_shared_array().use_count(), owners);
}

TEST_F(ContourSpectrumTest, BuilderInputErrorsThrowOnTheFirstStep) {
  volume flat(ctx, dimension(8, 8, 1), Float);
  contour_spectrum_builder b(flat); // constructing never throws
  EXPECT_FALSE(b.done());
  EXPECT_THROW(b.step(10), contour_spectrum_error);
  EXPECT_TRUE(b.done());
  EXPECT_EQ(b.progress(), 0.0);
  EXPECT_THROW(b.step(10), contour_spectrum_error);
  EXPECT_THROW(b.take(), contour_spectrum_error);

  const volume ok = ramp_field(ctx);
  contour_spectrum_options opt;
  opt.samples = 1;
  contour_spectrum_builder few(ok, opt);
  EXPECT_THROW(few.step(0), contour_spectrum_error);

  // A non-finite domain is only known once the scan is done.
  volume inf = ramp_field(ctx);
  inf.min(-std::numeric_limits<double>::infinity());
  contour_spectrum_builder unbounded(inf);
  EXPECT_FALSE(unbounded.step(100)); // scanning for the maximum
  bool threw = false;
  try {
    while (!unbounded.step(100)) {
    }
  } catch (const contour_spectrum_error &) {
    threw = true;
  }
  EXPECT_TRUE(threw);
  EXPECT_TRUE(unbounded.done());
}

TEST_F(ContourSpectrumTest, BuilderTakeOnce) {
  const volume vol = smooth_field(ctx, dimension(9, 8, 7));
  contour_spectrum_builder b(vol);
  EXPECT_THROW(b.take(), contour_spectrum_error); // not started
  b.step(10);
  EXPECT_THROW(b.take(), contour_spectrum_error); // not ready
  EXPECT_TRUE(b.step(0));
  expect_identical(b.take(), compute_contour_spectrum(vol));
  EXPECT_THROW(b.take(), contour_spectrum_error); // taken
  EXPECT_TRUE(b.step(0));
  EXPECT_EQ(b.progress(), 1.0);
}

TEST_F(ContourSpectrumTest, MovedBuildersKeepWorking) {
  const volume vol = noise_field(ctx, dimension(14, 12, 10), Float, 5);
  contour_spectrum_options opt;
  opt.max_dim = 9;
  const contour_spectrum ref = compute_contour_spectrum(vol, opt);

  // Moved in each phase, the computation carries on where it was.
  contour_spectrum_builder a(vol, opt);
  while (std::string(a.phase()) != "resampling")
    a.step(101);
  a.step(101);
  const uint64 work = a.work_done();
  const double progress = a.progress();
  contour_spectrum_builder b(std::move(a));
  EXPECT_EQ(b.work_done(), work);
  EXPECT_EQ(b.progress(), progress);
  EXPECT_STREQ(b.phase(), "resampling");
  while (std::string(b.phase()) != "integrating")
    b.step(101);
  b.step(101);
  contour_spectrum_builder c(vol);
  c.step(10);
  c = std::move(b); // discards c's own computation
  while (!c.step(333)) {
  }
  expect_identical(c.take(), ref);

  // A moved-from builder is empty, and a builder assigned to it works.
  for (const contour_spectrum_builder *empty : {&a, &b}) {
    EXPECT_TRUE(empty->done());
    EXPECT_STREQ(empty->phase(), "done");
    EXPECT_EQ(empty->progress(), 1.0);
    EXPECT_EQ(empty->work_done(), 0u);
  }
  EXPECT_THROW(a.step(10), contour_spectrum_error);
  EXPECT_THROW(a.take(), contour_spectrum_error);
  a = contour_spectrum_builder(vol, opt);
  EXPECT_FALSE(a.done());
  while (!a.step(4096)) {
  }
  expect_identical(a.take(), ref);
}

TEST_F(ContourSpectrumTest, BuildersOnOneVolumeAcrossThreads) {
  const volume vol = integer_field(ctx, 20, UShort, 5.0, 3.0);
  const contour_spectrum serial = compute_contour_spectrum(vol);
  contour_spectrum r1, r2;
  const auto run = [&vol](contour_spectrum &out, std::size_t budget) {
    contour_spectrum_builder b(vol);
    while (!b.step(budget)) {
    }
    out = b.take();
  };
  std::thread t1(run, std::ref(r1), std::size_t(33));
  std::thread t2(run, std::ref(r2), std::size_t(4096));
  t1.join();
  t2.join();
  expect_identical(r1, serial);
  expect_identical(r2, serial);

  // One builder may move between threads between steps.
  contour_spectrum_builder b(vol);
  while (!b.done()) {
    std::thread t([&b] { b.step(std::size_t(1) << 19); });
    t.join();
  }
  expect_identical(b.take(), serial);
  EXPECT_FALSE(vol.minIsSet()); // nothing wrote the lazy min/max cache
  EXPECT_FALSE(vol.maxIsSet());
}

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::volume_filter (cvc/volume/volume_filter.h) and volume.filter in the DSL
// (cvc/volume/volume_filter_intrinsics.h; docs/roadmap/DATA_FROM_STATE_TREE.md, decision 2): the
// stepper's bilateral filter matches VolMagick's voxels::bilateralFilter voxel for voxel and never
// writes its input; steps are bounded by their budget; a constant volume comes back as it is;
// bad inputs throw; the three DSL forms on the app's scheduler, cooperative and threaded, and a bad
// parameter raising at the call; install_volume_filter_intrinsics is one app's alone. Runs natively
// and under node (the ports lane).
//
// V0 (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4) adds contrast enhancement, anisotropic
// diffusion and GDTV, checked against the VolumeRover port plan's §1.7 matrix:
//   T1  each equals its VolMagick one-shot (voxels::contrastEnhancement, ::anisotropicDiffusion,
//       ::gdtvFilter) byte for byte, on float, unsigned char, unsigned short and double volumes and
//       on volumes one voxel thin along each axis;
//   T5  parameters out of range, NaN, an unknown filter and empty or null input throw or raise;
//       a constant volume and a NaN voxel go through;
//   T6  two threads with different filters give what each gives alone, in either order; the
//       intrinsics are one app's alone;
//   T7  step budgets 1, 64, seeded random and 0 give the same bytes; progress rises within [0, 1]
//       and the phases are named; a cancel in each phase leaves no result; the slowest step at the
//       default budget is printed;
//   T8  every field round-trips through the state tree, with its properties;
//   T10 the new filters in the DSL's three forms, cooperative and threaded, and job.cancel.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/param_table.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_filter.h>
#include <cvc/volume/volume_filter_intrinsics.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

// nx x ny x nz floats: a ramp with an edge down x = nx / 2 and a ripple on it, so the bilateral
// filter both smooths (the ripple) and keeps (the edge).
cvc::volume edged(cvc::app &app, int nx, int ny, int nz) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), cvc::Float);
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i)
        v(i, j, k, (i < nx / 2 ? 0.0 : 400.0) + 3.0 * j + 7.0 * std::sin(1.7 * i + 2.3 * k));
  return v;
}

std::vector<float> floats(const cvc::volume &v) {
  std::vector<float> out(v.XDim() * v.YDim() * v.ZDim());
  std::memcpy(out.data(), v.data_ptr(), out.size() * sizeof(float));
  return out;
}

TEST(VolumeFilter, BilateralMatchesVolMagickVoxelForVoxel) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(edged(app, 11, 9, 7));
  const std::vector<float> before = floats(*in);
  for (const int radius : {1, 2}) {
    cvc::volume_filter_params p;
    p.radius = radius;
    p.radiometric_sigma = 60.0;
    p.spatial_sigma = 1.2;
    auto job = cvc::volume_filter_job(in, p);
    while (!job->step(37)) {
    }
    ASSERT_FALSE(job->failed()) << job->error();
    const std::shared_ptr<const cvc::volume> out = job->take_result();
    ASSERT_TRUE(out);
    EXPECT_NE(out.get(), in.get());
    EXPECT_NE(out->data_ptr(), in->data_ptr()) << "the result has voxels of its own";

    cvc::volume legacy(*in);
    legacy.voxels::copy(); // a deep copy, as the filter writes in place
    legacy.bilateralFilter(p.radiometric_sigma, p.spatial_sigma, static_cast<unsigned>(radius));
    EXPECT_EQ(floats(*out), floats(legacy)) << "radius " << radius;
    EXPECT_EQ(out->boundingBox(), in->boundingBox());
    EXPECT_DOUBLE_EQ(out->min(), legacy.min());
    EXPECT_DOUBLE_EQ(out->max(), legacy.max());
  }
  EXPECT_EQ(floats(*in), before) << "the input, which other nodes show, is never written";
}

TEST(VolumeFilter, StepsAreBoundedByTheirBudget) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(edged(app, 10, 6, 4));
  cvc::volume_filter_stepper s(in, cvc::volume_filter_params{});
  EXPECT_STREQ(s.phase(), "bilateral");
  EXPECT_FALSE(s.step(1)); // one row, however small the budget
  EXPECT_DOUBLE_EQ(s.progress(), 1.0 / 24.0);
  EXPECT_FALSE(s.step(30)); // three rows of ten voxels
  EXPECT_DOUBLE_EQ(s.progress(), 4.0 / 24.0);
  EXPECT_THROW(s.take(), std::logic_error);
  EXPECT_TRUE(s.step(0)); // the rest
  EXPECT_TRUE(s.done());
  EXPECT_DOUBLE_EQ(s.progress(), 1.0);
  EXPECT_TRUE(s.take());
}

TEST(VolumeFilter, AConstantVolumeComesBackAsItIs) {
  cvc::app app;
  cvc::volume flat(app, cvc::dimension(4, 4, 4), cvc::Float);
  flat.fill(5.0);
  const cvc::volume out = cvc::filter_volume(flat, cvc::volume_filter_params{});
  EXPECT_DOUBLE_EQ(out.min(), 5.0);
  EXPECT_DOUBLE_EQ(out.max(), 5.0);
}

TEST(VolumeFilter, BadInputsThrow) {
  cvc::app app;
  EXPECT_THROW(cvc::volume_filter_stepper(nullptr, cvc::volume_filter_params{}),
               std::invalid_argument);
  cvc::volume_filter_params p;
  p.radius = 17;
  EXPECT_THROW(
      cvc::volume_filter_stepper(std::make_shared<const cvc::volume>(edged(app, 4, 4, 4)), p),
      std::invalid_argument);
}

// ---- contrast, anisotropic and GDTV against VolMagick ------------------------------------------

// nx x ny x nz voxels of type t: two regions, a slope and seeded noise (no global generator), the
// values inside what every voxel type holds (0 to 250).
cvc::volume noisy(cvc::app &app, int nx, int ny, int nz, cvc::data_type t, unsigned seed = 7) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), t);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> noise(-12.0, 12.0);
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        const double base = (i + j < (nx + ny) / 2 ? 60.0 : 170.0) + 2.0 * k + 1.5 * j;
        v(i, j, k, std::clamp(base + noise(rng), 0.0, 250.0));
      }
  return v;
}

std::size_t byte_size(const cvc::volume &v) {
  return static_cast<std::size_t>(v.XDim() * v.YDim() * v.ZDim() * v.voxelSize());
}

// The voxels' bytes, so NaN and -0 compare exactly too.
bool same_bytes(const cvc::volume &a, const cvc::volume &b) {
  return a.voxelType() == b.voxelType() && a.voxel_dimensions() == b.voxel_dimensions() &&
         std::memcmp(a.data_ptr(), b.data_ptr(), byte_size(a)) == 0;
}

// The first voxel where two volumes differ, for a failure's message.
std::string first_difference(const cvc::volume &a, const cvc::volume &b) {
  if (a.voxel_dimensions() != b.voxel_dimensions())
    return "different dimensions";
  for (std::uint64_t k = 0; k < a.ZDim(); ++k)
    for (std::uint64_t j = 0; j < a.YDim(); ++j)
      for (std::uint64_t i = 0; i < a.XDim(); ++i)
        if (a(i, j, k) != b(i, j, k)) {
          char s[160];
          std::snprintf(s, sizeof s, "at (%d, %d, %d): %.17g vs %.17g", int(i), int(j), int(k),
                        a(i, j, k), b(i, j, k));
          return s;
        }
  return "only in bytes (NaN payloads or -0)";
}

// What VolumeRover2 ran for the same parameters: the one-shot on a deep copy.
cvc::volume legacy_filter(const cvc::volume &in, const cvc::volume_filter_params &p) {
  cvc::volume v(in);
  v.voxels::copy();
  switch (p.filter) {
  case cvc::volume_filter_kind::bilateral:
    v.bilateralFilter(p.radiometric_sigma, p.spatial_sigma, static_cast<unsigned>(p.radius));
    break;
  case cvc::volume_filter_kind::contrast:
    v.contrastEnhancement(p.contrast_resistor);
    break;
  case cvc::volume_filter_kind::anisotropic:
    v.anisotropicDiffusion(static_cast<unsigned>(p.anisotropic_iterations));
    break;
  case cvc::volume_filter_kind::gdtv: // GDTVFilterDialog: gdtvFilter(2 - exp, lambda, n, nhood - 1)
    v.gdtvFilter(2.0 - p.gdtv_exponent, p.gdtv_lambda, static_cast<unsigned>(p.gdtv_iterations),
                 p.gdtv_neighbours == cvc::volume_gdtv_neighbours::six ? 0u : 1u);
    break;
  }
  return v;
}

// The stepper run with a budget schedule: a fixed budget, or (budget < 0) a seeded random budget
// per step between 1 and 300.
std::shared_ptr<const cvc::volume> stepped(std::shared_ptr<const cvc::volume> in,
                                           const cvc::volume_filter_params &p, long budget,
                                           std::vector<std::string> *phases = nullptr) {
  cvc::volume_filter_stepper s(std::move(in), p);
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> pick(1, 300);
  double last = 0.0;
  for (;;) {
    const std::size_t b =
        budget < 0 ? static_cast<std::size_t>(pick(rng)) : static_cast<std::size_t>(budget);
    const bool done = s.step(b);
    const double pr = s.progress();
    EXPECT_GE(pr, last) << "progress never falls";
    EXPECT_LE(pr, 1.0);
    last = pr;
    if (phases)
      phases->push_back(s.phase());
    if (done)
      break;
  }
  EXPECT_DOUBLE_EQ(s.progress(), 1.0);
  return s.take();
}

cvc::volume_filter_params kind(cvc::volume_filter_kind k) {
  cvc::volume_filter_params p;
  p.filter = k;
  return p;
}

struct parity_case {
  const char *what;
  cvc::volume_filter_params p;
};

std::vector<parity_case> parity_cases() {
  std::vector<parity_case> cases;
  using K = cvc::volume_filter_kind;
  cases.push_back({"contrast 0.95", kind(K::contrast)});
  cvc::volume_filter_params c = kind(K::contrast);
  c.contrast_resistor = 0.5;
  cases.push_back({"contrast 0.5", c});
  cvc::volume_filter_params a = kind(K::anisotropic);
  a.anisotropic_iterations = 3;
  cases.push_back({"anisotropic 3", a});
  a.anisotropic_iterations = 1;
  cases.push_back({"anisotropic 1", a});
  cases.push_back({"gdtv defaults", kind(K::gdtv)});
  cvc::volume_filter_params g = kind(K::gdtv);
  g.gdtv_exponent = 1.4;
  g.gdtv_lambda = 0.05;
  g.gdtv_iterations = 2;
  cases.push_back({"gdtv 1.4 0.05 x2", g});
  g.gdtv_neighbours = cvc::volume_gdtv_neighbours::twenty_six;
  cases.push_back({"gdtv 26 1.4 0.05 x2", g});
  g.gdtv_exponent = 2.0;
  g.gdtv_iterations = 1;
  cases.push_back({"gdtv 26 linear", g});
  return cases;
}

TEST(VolumeFilterKinds, EachMatchesItsVolMagickOneShotByteForByte) {
  cvc::app app;
  for (const cvc::data_type t : {cvc::Float, cvc::UChar, cvc::UShort, cvc::Double}) {
    const auto in = std::make_shared<const cvc::volume>(noisy(app, 13, 11, 9, t));
    const std::vector<unsigned char> before(static_cast<const unsigned char *>(in->data_ptr()),
                                            static_cast<const unsigned char *>(in->data_ptr()) +
                                                byte_size(*in));
    for (const parity_case &c : parity_cases()) {
      SCOPED_TRACE(std::string(c.what) + ", voxel type " + std::to_string(int(t)));
      const std::shared_ptr<const cvc::volume> out = stepped(in, c.p, 64);
      ASSERT_TRUE(out);
      EXPECT_NE(out->data_ptr(), in->data_ptr()) << "the result has voxels of its own";
      const cvc::volume legacy = legacy_filter(*in, c.p);
      EXPECT_TRUE(same_bytes(*out, legacy)) << first_difference(*out, legacy);
      EXPECT_EQ(out->boundingBox(), in->boundingBox());
      EXPECT_DOUBLE_EQ(out->min(), legacy.min());
      EXPECT_DOUBLE_EQ(out->max(), legacy.max());
    }
    EXPECT_EQ(0, std::memcmp(in->data_ptr(), before.data(), before.size()))
        << "the input, which other nodes show, is never written";
  }
}

TEST(VolumeFilterKinds, ThinVolumesMatchToo) {
  cvc::app app;
  // One voxel thin along each axis, and a single voxel: the edge cases of every sweep.
  const int dims[][3] = {{1, 7, 5}, {7, 1, 5}, {7, 5, 1}, {1, 1, 6}, {1, 1, 1}, {2, 2, 2}};
  for (const auto &d : dims)
    for (const parity_case &c : parity_cases()) {
      SCOPED_TRACE(std::string(c.what) + " on " + std::to_string(d[0]) + "x" +
                   std::to_string(d[1]) + "x" + std::to_string(d[2]));
      const auto in = std::make_shared<const cvc::volume>(noisy(app, d[0], d[1], d[2], cvc::Float));
      const std::shared_ptr<const cvc::volume> out = stepped(in, c.p, 1);
      const cvc::volume legacy = legacy_filter(*in, c.p);
      EXPECT_TRUE(same_bytes(*out, legacy)) << first_difference(*out, legacy);
    }
}

TEST(VolumeFilterKinds, EveryStepScheduleGivesTheSameBytes) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 10, 9, 7, cvc::Float, 3));
  const std::set<std::string> named = {
      "bilateral",      "contrast: mapping",    "contrast: up",
      "contrast: down", "contrast: stretching", "contrast: restoring",
      "anisotropic",    "gdtv: gradient",       "gdtv: filtering"};
  std::vector<cvc::volume_filter_params> ps = {kind(cvc::volume_filter_kind::bilateral)};
  for (const parity_case &c : parity_cases())
    ps.push_back(c.p);
  for (const cvc::volume_filter_params &p : ps) {
    SCOPED_TRACE("filter " + std::to_string(int(p.filter)));
    std::vector<std::string> phases;
    const auto at0 = stepped(in, p, 0);
    for (const long b : {1L, 64L, -1L}) {
      const auto out = stepped(in, p, b, &phases);
      EXPECT_TRUE(same_bytes(*out, *at0)) << "budget " << b << ": " << first_difference(*out, *at0);
    }
    for (const std::string &ph : phases)
      EXPECT_TRUE(named.count(ph)) << "unnamed phase '" << ph << "'";
  }
}

TEST(VolumeFilterKinds, StepsAreWholeRowsAndPhasesAdvance) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 10, 6, 4, cvc::Float));
  {
    cvc::volume_filter_stepper s(in, kind(cvc::volume_filter_kind::contrast));
    EXPECT_STREQ(s.phase(), "contrast: mapping");
    std::set<std::string> seen;
    while (!s.step(1))
      seen.insert(s.phase());
    for (const char *ph :
         {"contrast: up", "contrast: down", "contrast: stretching", "contrast: restoring"})
      EXPECT_TRUE(seen.count(ph)) << ph;
  }
  {
    cvc::volume_filter_params p = kind(cvc::volume_filter_kind::anisotropic);
    p.anisotropic_iterations = 2;
    cvc::volume_filter_stepper s(in, p);
    EXPECT_STREQ(s.phase(), "anisotropic");
    EXPECT_FALSE(s.step(1)); // one row, however small the budget
    EXPECT_DOUBLE_EQ(s.progress(), 1.0 / 48.0);
    EXPECT_FALSE(s.step(30)); // three rows of ten
    EXPECT_DOUBLE_EQ(s.progress(), 4.0 / 48.0);
    EXPECT_THROW(s.take(), std::logic_error);
    EXPECT_TRUE(s.step(0));
    EXPECT_TRUE(s.take());
    EXPECT_THROW(s.take(), std::logic_error) << "the result is handed out once";
  }
  {
    cvc::volume_filter_stepper s(in, kind(cvc::volume_filter_kind::gdtv));
    EXPECT_STREQ(s.phase(), "gdtv: gradient");
    EXPECT_FALSE(s.step(23 * 10)); // all of the gradient's 24 rows but the last
    EXPECT_STREQ(s.phase(), "gdtv: gradient");
    EXPECT_FALSE(s.step(1));
    EXPECT_STREQ(s.phase(), "gdtv: filtering");
    EXPECT_DOUBLE_EQ(s.progress(), 0.5);
    EXPECT_TRUE(s.step(0));
    EXPECT_NE(s.status().find("gdtv: 10 x 6 x 4 voxels"), std::string::npos) << s.status();
  }
}

TEST(VolumeFilterKinds, ACancelInEachPhaseLeavesNoResult) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 8, 7, 6, cvc::Float));
  std::vector<cvc::volume_filter_params> ps;
  for (const parity_case &c : parity_cases())
    ps.push_back(c.p);
  ps.push_back(kind(cvc::volume_filter_kind::bilateral));
  for (const cvc::volume_filter_params &p : ps) {
    // Which phases this filter goes through, then a cancel at the first step of each.
    std::vector<std::string> order;
    {
      cvc::volume_filter_stepper s(in, p);
      do {
        if (order.empty() || order.back() != s.phase())
          order.push_back(s.phase());
      } while (!s.step(1));
    }
    for (const std::string &ph : order) {
      SCOPED_TRACE("filter " + std::to_string(int(p.filter)) + ", cancelled in " + ph);
      auto job = cvc::volume_filter_job(in, p);
      while (std::string(job->phase()) != ph && !job->done())
        job->step(1);
      job->cancel();
      EXPECT_TRUE(job->step(1));
      EXPECT_TRUE(job->cancelled());
      EXPECT_FALSE(job->has_result());
      EXPECT_THROW(job->take_result(), std::exception);
    }
  }
}

TEST(VolumeFilterKinds, BadParametersAndInputsThrow) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 5, 5, 5, cvc::Float));
  using K = cvc::volume_filter_kind;
  const auto throws = [&](cvc::volume_filter_params p, const char *field) {
    try {
      cvc::volume_filter_stepper s(in, p);
      ADD_FAILURE() << field << ": no exception";
    } catch (const std::invalid_argument &e) {
      EXPECT_NE(std::string(e.what()).find(field), std::string::npos) << e.what();
    }
  };
  cvc::volume_filter_params p = kind(K::contrast);
  p.contrast_resistor = 1.5;
  throws(p, "contrast.resistor");
  p.contrast_resistor = std::nan("");
  throws(p, "contrast.resistor");
  p = kind(K::anisotropic);
  p.anisotropic_iterations = 0;
  throws(p, "anisotropic.iterations");
  p = kind(K::gdtv);
  p.gdtv_exponent = 2.5;
  throws(p, "gdtv.exponent");
  p = kind(K::gdtv);
  p.gdtv_lambda = 0.0; // filtGDTV divides by the weights' sum, which is lambda on a flat patch
  throws(p, "gdtv.lambda");
  p = kind(K::gdtv);
  p.gdtv_iterations = 1001;
  throws(p, "gdtv.iterations");
  p = kind(K::gdtv);
  p.gdtv_neighbours = static_cast<cvc::volume_gdtv_neighbours>(9);
  throws(p, "gdtv.neighbours");
  p = kind(K::gdtv);
  p.filter = static_cast<K>(9);
  throws(p, "filter");

  for (const K k : {K::contrast, K::anisotropic, K::gdtv}) {
    EXPECT_THROW(cvc::volume_filter_stepper(nullptr, kind(k)), std::invalid_argument);
    EXPECT_THROW(
        cvc::volume_filter_stepper(
            std::make_shared<const cvc::volume>(app, cvc::dimension(0, 0, 0), cvc::Float), kind(k)),
        std::invalid_argument);
  }
}

TEST(VolumeFilterKinds, ConstantVolumesAndNaNVoxelsGoThrough) {
  cvc::app app;
  for (const parity_case &c : parity_cases()) {
    SCOPED_TRACE(c.what);
    cvc::volume flat(app, cvc::dimension(5, 4, 3), cvc::Float);
    flat.fill(42.0);
    const cvc::volume out = cvc::filter_volume(flat, c.p);
    const cvc::volume legacy = legacy_filter(flat, c.p);
    EXPECT_TRUE(same_bytes(out, legacy)) << first_difference(out, legacy);

    cvc::volume nan = noisy(app, 6, 5, 4, cvc::Float);
    nan(2, 2, 2, std::nan(""));
    nan.unsetMinMax();
    EXPECT_NO_THROW(cvc::filter_volume(nan, c.p)); // no crash, whatever NaN spreads to
  }
}

// T6: two filters on two threads give what each gives alone, and running one before the other
// changes neither.
TEST(VolumeFilterKinds, TwoThreadsAndEitherOrderGiveTheSameResults) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 12, 10, 8, cvc::Float));
  cvc::volume_filter_params a = kind(cvc::volume_filter_kind::contrast);
  cvc::volume_filter_params b = kind(cvc::volume_filter_kind::gdtv);
  b.gdtv_neighbours = cvc::volume_gdtv_neighbours::twenty_six;
  b.gdtv_iterations = 3;
  const auto alone_a = stepped(in, a, 0);
  const auto alone_b = stepped(in, b, 0);
  const auto run = [&in](const cvc::volume_filter_params &p) {
    auto job = cvc::volume_filter_job(in, p);
    while (!job->step(97)) {
    }
    return job->take_result();
  };
  std::shared_ptr<const cvc::volume> ta, tb;
  std::thread x([&] { ta = run(a); });
  std::thread y([&] { tb = run(b); });
  x.join();
  y.join();
  ASSERT_TRUE(ta && tb);
  EXPECT_TRUE(same_bytes(*ta, *alone_a));
  EXPECT_TRUE(same_bytes(*tb, *alone_b));
  const auto b_first = run(b);
  const auto a_after = run(a);
  EXPECT_TRUE(same_bytes(*b_first, *alone_b));
  EXPECT_TRUE(same_bytes(*a_after, *alone_a));
}

// T7: the slowest step at the default budget (the DSL's and the component's), recorded for each
// filter on a 48^3 volume. Printed, not bounded: CI's runners and sanitizers vary too much.
TEST(VolumeFilterKinds, TheSlowestStepIsRecorded) {
  cvc::app app;
  const auto in = std::make_shared<const cvc::volume>(noisy(app, 48, 48, 48, cvc::Float));
  std::vector<cvc::volume_filter_params> ps = {
      kind(cvc::volume_filter_kind::bilateral), kind(cvc::volume_filter_kind::contrast),
      kind(cvc::volume_filter_kind::anisotropic), kind(cvc::volume_filter_kind::gdtv)};
  cvc::volume_filter_params g26 = kind(cvc::volume_filter_kind::gdtv);
  g26.gdtv_neighbours = cvc::volume_gdtv_neighbours::twenty_six;
  ps.push_back(g26);
  for (const cvc::volume_filter_params &p : ps) {
    auto job = cvc::volume_filter_job(in, p);
    while (!job->step(65536)) {
    }
    ASSERT_FALSE(job->failed()) << job->error();
    std::printf("volume_filter %d: %zu steps, slowest %.2f ms, all %.1f ms (48^3, budget 65536)\n",
                int(p.filter), job->steps(), job->max_step_ms(), job->work_ms());
  }
}

// T8: every field to the state tree and back, its defaults seeded and its properties written.
TEST(VolumeFilterKinds, TheParametersRoundTripThroughTheStateTree) {
  cvc::app app;
  cvc::state &node = app.root()("filters");
  cvc::volume_filter_params p;
  p.filter = cvc::volume_filter_kind::gdtv;
  p.contrast_resistor = 0.25;
  p.anisotropic_iterations = 7;
  p.gdtv_exponent = 1.25;
  p.gdtv_lambda = 3e-3;
  p.gdtv_iterations = 4;
  p.gdtv_neighbours = cvc::volume_gdtv_neighbours::twenty_six;
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("filter").value(), "gdtv");
  EXPECT_EQ(node("contrast")("resistor").value(), "0.25");
  EXPECT_EQ(node("gdtv")("neighbours").value(), "twenty_six");
  const cvc::param_result<cvc::volume_filter_params> back =
      cvc::params_from_state<cvc::volume_filter_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.filter, p.filter);
  EXPECT_EQ(back.value.contrast_resistor, p.contrast_resistor);
  EXPECT_EQ(back.value.anisotropic_iterations, p.anisotropic_iterations);
  EXPECT_EQ(back.value.gdtv_exponent, p.gdtv_exponent);
  EXPECT_EQ(back.value.gdtv_lambda, p.gdtv_lambda);
  EXPECT_EQ(back.value.gdtv_iterations, p.gdtv_iterations);
  EXPECT_EQ(back.value.gdtv_neighbours, p.gdtv_neighbours);

  // An empty node is seeded with VolumeRover2's defaults; a value that does not parse is reported.
  cvc::state &fresh = app.root()("fresh");
  const auto seeded = cvc::params_from_state<cvc::volume_filter_params>(fresh);
  EXPECT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("anisotropic")("iterations").value(), "20");
  EXPECT_EQ(fresh("contrast")("resistor").value(), "0.95");
  EXPECT_EQ(fresh("gdtv")("exponent").value(), "1");
  EXPECT_EQ(fresh("gdtv")("lambda").value(), "1e-04"); // the shortest round-trip form
  EXPECT_EQ(fresh("gdtv")("iterations").value(), "1");
  EXPECT_EQ(fresh("gdtv")("neighbours").value(), "six");
  EXPECT_EQ(fresh("radiometric_sigma").value(), "200");
  EXPECT_EQ(fresh("filter").value(), "bilateral");
  EXPECT_NE(fresh("filter").property("choices").find("anisotropic"), std::string::npos);
  EXPECT_EQ(fresh("gdtv")("exponent").property("max"), "2");
  fresh("gdtv")("iterations").value(std::string("lots"));
  EXPECT_FALSE(cvc::params_from_state<cvc::volume_filter_params>(fresh).ok());
}

// ---- volume.filter in the DSL -------------------------------------------------------------------

class VolumeFilterIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
protected:
  void SetUp() override {
    se::async_scheduler &sched = app.exec_scheduler();
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    cvc::register_job_op(env, ictx, app, cvc::volume_filter_op(GetParam()));
    cvc::register_job_intrinsics(env, app);
    vol = std::make_shared<const cvc::volume>(edged(app, 12, 10, 8));
    app.root()("data.head.volume").data(boost::any(vol)); // as a data: entry holds it
    cvc::volume_filter_params p;
    p.radius = 1;
    expected = cvc::filter_volume(*vol, p);
  }

  int run(const std::string &program) {
    se::execute_options eo;
    eo.env = env;
    return app.exec_scheduler().execute(program, eo);
  }
  template <class Pred> bool pump_until(Pred pred, double seconds = 120.0) {
    const auto t0 = steady::now();
    se::async_scheduler &sched = app.exec_scheduler();
    while (!pred()) {
      if (std::chrono::duration<double>(steady::now() - t0).count() > seconds)
        return false;
      sched.drain_ingress();
      sched.wake_awaiting();
      sched.sync_step();
      sched.sync_run(200000, 0.1);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
  }
  bool finished(int pid) {
    const auto info = app.exec_scheduler().get_process_info(pid);
    return !info || se::is_finished(info->status);
  }
  std::string key(const std::string &k) { return app.root()(k).value(); }
  std::string exit_error(int pid) {
    const auto info = app.exec_scheduler().get_process_info(pid);
    return info ? info->exit_error.value_or(std::string()) : std::string("(reaped)");
  }
  // The volume a data object at `path` holds (stored with state-data-set, or a -start's result).
  std::shared_ptr<const cvc::volume> volume_at(const std::string &path) {
    const boost::any a = app.root()(path).data();
    const se::value_t *v = boost::any_cast<se::value_t>(&a);
    if (!v)
      return nullptr;
    const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v);
    if (!obj || !*obj)
      return nullptr;
    const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&(*obj)->payload);
    return p ? *p : nullptr;
  }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
  std::shared_ptr<const cvc::volume> vol;
  cvc::volume expected{app};
};

TEST_P(VolumeFilterIntrinsics, TheWaitingFormReturnsTheFilteredVolume) {
  const int pid = run("(state-data-set \"r.out\" (volume.filter \"data.head.volume\" "
                      "(dict \"radius\" 1)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  const auto out = volume_at("r.out");
  ASSERT_TRUE(out);
  EXPECT_EQ(floats(*out), floats(expected));
  EXPECT_NE(out->data_ptr(), vol->data_ptr());
}

TEST_P(VolumeFilterIntrinsics, TheAsyncFormAndParameters) {
  const int pid = run("(let ((f (volume.filter-async \"data.head.volume\" (dict \"radius\" 1))))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (state-data-set \"r.out\" (await f)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  const auto out = volume_at("r.out");
  ASSERT_TRUE(out);
  EXPECT_EQ(floats(*out), floats(expected));
  // A bad parameter raises at the call, naming it; so does an input that is no volume.
  const int bad = run("(volume.filter \"data.head.volume\" (dict \"radius\" 40))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("radius"), std::string::npos) << exit_error(bad);
  app.root()("doc.text").value(std::string("not a volume"));
  const int none = run("(volume.filter \"doc.text\")");
  ASSERT_TRUE(pump_until([&] { return finished(none); }));
  EXPECT_NE(exit_error(none), "");
  const int defaults =
      run("(state-set \"r.radius\" (get-attr (params.defaults \"volume.filter\") \"radius\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.radius"), "2");
}

TEST_P(VolumeFilterIntrinsics, TheStartFormAndStoringTheResultWhereTheViewsLook) {
  const int caller = run("(state-set \"r.pid\" (volume.filter-start \"data.head.volume\" "
                         "(dict \"radius\" 1) \"filtering\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_FALSE(key("r.pid").empty());
  ASSERT_TRUE(pump_until([&] { return !app.root()("filtering.result").data().empty(); }))
      << "status: " << key("filtering.status");
  ASSERT_TRUE(pump_until([&] { return key("filtering.busy") == "0"; }));
  // The program stores it at the dataset's key: what a volren and a volume_grid there follow.
  const int store =
      run("(state-data-set \"data.head.volume\" (state-data-get \"filtering.result\"))");
  ASSERT_TRUE(pump_until([&] { return finished(store); }));
  EXPECT_EQ(exit_error(store), "");
  const auto out = volume_at("data.head.volume");
  ASSERT_TRUE(out);
  EXPECT_EQ(floats(*out), floats(expected));
}

TEST_P(VolumeFilterIntrinsics, TheNewFiltersInEveryFormAndACancel) {
  using K = cvc::volume_filter_kind;
  const std::pair<const char *, K> kinds[] = {
      {"contrast", K::contrast}, {"anisotropic", K::anisotropic}, {"gdtv", K::gdtv}};
  for (const auto &[name, k] : kinds) {
    SCOPED_TRACE(name);
    cvc::volume_filter_params p;
    p.filter = k;
    p.anisotropic_iterations = 2;
    const cvc::volume want = cvc::filter_volume(*vol, p);
    const std::string f =
        std::string("(dict \"filter\" \"") + name + "\" \"anisotropic.iterations\" 2)";
    // The waiting form.
    const int wait = run("(state-data-set \"r." + std::string(name) +
                         "\" (volume.filter \"data.head.volume\" " + f + "))");
    ASSERT_TRUE(pump_until([&] { return finished(wait); }));
    EXPECT_EQ(exit_error(wait), "");
    auto out = volume_at("r." + std::string(name));
    ASSERT_TRUE(out);
    EXPECT_TRUE(same_bytes(*out, want)) << first_difference(*out, want);
    // The -start form, its result at the target, as the component reads it.
    const std::string target = std::string("started_") + name;
    const int start =
        run("(volume.filter-start \"data.head.volume\" " + f + " \"" + target + "\")");
    ASSERT_TRUE(pump_until([&] { return finished(start); }, 5.0)) << "-start returns at once";
    ASSERT_TRUE(pump_until([&] {
      return key(target + ".busy") == "0" && !app.root()(target + ".result").data().empty();
    })) << key(target + ".status");
    out = volume_at(target + ".result");
    ASSERT_TRUE(out);
    EXPECT_TRUE(same_bytes(*out, want));
  }
  // The table names the new fields, with VolumeRover2's defaults.
  const int defaults = run("(state-set \"r.iter\" (get-attr (params.defaults \"volume.filter\") "
                           "\"anisotropic.iterations\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.iter"), "20");
  // An unknown filter raises at the call, naming the choices.
  const int bad = run("(volume.filter \"data.head.volume\" (dict \"filter\" \"sharpen\"))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("filter"), std::string::npos) << exit_error(bad);
  // A long diffusion, stopped part-way by job.cancel: no result, status cancelled.
  auto big = std::make_shared<const cvc::volume>(noisy(app, 40, 40, 40, cvc::Float));
  app.root()("data.big.volume").data(boost::any(big));
  const int slow = run("(state-set \"r.slow\" (volume.filter-start \"data.big.volume\" "
                       "(dict \"filter\" \"anisotropic\" \"anisotropic.iterations\" 1000) "
                       "\"slow\"))");
  ASSERT_TRUE(pump_until([&] { return finished(slow); }, 5.0));
  ASSERT_FALSE(key("r.slow").empty());
  const int cancel = run("(job.cancel (int (state-get \"slow.pid\")))");
  ASSERT_TRUE(pump_until([&] { return finished(cancel); }));
  EXPECT_EQ(exit_error(cancel), "");
  ASSERT_TRUE(pump_until([&] { return key("slow.busy") == "0"; })) << key("slow.status");
  EXPECT_EQ(key("slow.status"), "cancelled");
  EXPECT_TRUE(app.root()("slow.result").data().empty()) << "a cancelled job leaves no result";
}

INSTANTIATE_TEST_SUITE_P(Modes, VolumeFilterIntrinsics,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded),
                         [](const auto &info) {
                           return info.param == cvc::sliced_threading::cooperative ? "cooperative"
                                                                                   : "threaded";
                         });

TEST(VolumeFilterIntrinsicsApps, InstalledOnOneAppAlone) {
  cvc::app a, b;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } ga{a}, gb{b};
  cvc::install_volume_filter_intrinsics(a);
  const std::string script =
      "(state-set \"r.radius\" (get-attr (params.defaults \"volume.filter\") \"radius\"))";
  std::vector<std::string> errors_a, errors_b;
  EXPECT_TRUE(cvc::ariadne::run_init(a, "", script, &errors_a))
      << (errors_a.empty() ? "" : errors_a.front());
  EXPECT_EQ(a.root()("r.radius").value(), "2");
  EXPECT_FALSE(cvc::ariadne::run_init(b, "", script, &errors_b))
      << "an app that did not install it has no volume.filter";
  EXPECT_TRUE(b.root()("r.radius").value().empty());
}

} // namespace

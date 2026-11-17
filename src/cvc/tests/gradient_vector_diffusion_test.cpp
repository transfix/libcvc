/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::gradient_vector_diffusion and cvc::critical_point_at
// (cvc/volume/gradient_vector_diffusion.h, docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5 step
// 3): T2 a voxel's update by hand and the scheme's fixed points, T3 the critical kinds of known
// fields, T4 invariants, T5 bad input, T6 threads and order, T7 resumable, T8 the parameter table.
// Its legacy parity (T1, SegMed's gvfflow) is in critical_points_test.cpp, beside the critical
// points'.

#include "resumable_harness.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <cvc/volume/gradient_vector_diffusion_params.h>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

cvc::vector_field random_field(int nx, int ny, int nz, unsigned seed, double zeros = 0.0) {
  cvc::vector_field f(nx, ny, nz);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  std::uniform_real_distribution<double> p(0.0, 1.0);
  for (std::size_t v = 0; v < f.size(); ++v) {
    if (p(rng) < zeros) {
      f.x[v] = f.y[v] = f.z[v] = 0.0f;
    } else {
      f.x[v] = u(rng);
      f.y[v] = u(rng);
      f.z[v] = u(rng);
    }
  }
  return f;
}

bool same_field(const cvc::vector_field &a, const cvc::vector_field &b) {
  return a.nx == b.nx && a.ny == b.ny && a.nz == b.nz && a.x == b.x && a.y == b.y && a.z == b.z;
}

double max_diff(const cvc::vector_field &a, const cvc::vector_field &b) {
  double d = 0;
  for (std::size_t v = 0; v < a.size(); ++v)
    d = std::max({d, double(std::fabs(a.x[v] - b.x[v])), double(std::fabs(a.y[v] - b.y[v])),
                  double(std::fabs(a.z[v] - b.z[v]))});
  return d;
}

cvc::gradient_vector_diffusion_params
raw(int iterations, double k = 2.0, cvc::diffusion_weights w = cvc::diffusion_weights::current) {
  cvc::gradient_vector_diffusion_params p;
  p.iterations = iterations;
  p.k = k;
  p.weights = w;
  p.normalize = false;
  return p;
}

// ---- T2: units --------------------------------------------------------------------------------

TEST(GradientVectorDiffusion, OneVoxelByHand) {
  // Eq. 3 on the centre of a 3x3x3 field, in double: u' = u + sum g_n (u_n - u), the g_n
  // exp(k (cos a - 1)) divided by their sum.
  const cvc::vector_field f = random_field(3, 3, 3, 4);
  const cvc::vector_field r = cvc::gradient_vector_diffusion(f, raw(1));
  const std::size_t c = f.index(1, 1, 1);
  const std::size_t nb[6] = {f.index(2, 1, 1), f.index(0, 1, 1), f.index(1, 2, 1),
                             f.index(1, 0, 1), f.index(1, 1, 2), f.index(1, 1, 0)};
  auto len = [&](std::size_t v) {
    return std::sqrt(double(f.x[v]) * f.x[v] + double(f.y[v]) * f.y[v] + double(f.z[v]) * f.z[v]);
  };
  double g[6], sum = 0;
  for (int n = 0; n < 6; ++n) {
    const double cosa =
        (double(f.x[c]) * f.x[nb[n]] + double(f.y[c]) * f.y[nb[n]] + double(f.z[c]) * f.z[nb[n]]) /
        (len(c) * len(nb[n]));
    g[n] = std::exp(2.0 * (cosa - 1.0));
    sum += g[n];
  }
  double want[3] = {f.x[c], f.y[c], f.z[c]};
  const std::vector<float> *comp[3] = {&f.x, &f.y, &f.z};
  for (int a = 0; a < 3; ++a)
    for (int n = 0; n < 6; ++n)
      want[a] += g[n] / sum * ((*comp[a])[nb[n]] - (*comp[a])[c]);
  EXPECT_NEAR(r.x[c], want[0], 1e-5);
  EXPECT_NEAR(r.y[c], want[1], 1e-5);
  EXPECT_NEAR(r.z[c], want[2], 1e-5);
}

TEST(GradientVectorDiffusion, FixedPointsAndAverages) {
  // A zero field stays zero, normalised or not.
  cvc::vector_field zero(5, 4, 3);
  EXPECT_TRUE(same_field(cvc::gradient_vector_diffusion(zero), zero));
  // A uniform field is a fixed point (every weight equal); normalised, every vector has length 1.
  cvc::vector_field uni(6, 5, 4);
  std::fill(uni.x.begin(), uni.x.end(), 0.6f);
  std::fill(uni.y.begin(), uni.y.end(), -0.8f);
  std::fill(uni.z.begin(), uni.z.end(), 0.0f);
  EXPECT_LE(max_diff(cvc::gradient_vector_diffusion(uni, raw(5)), uni), 1e-6);
  const cvc::vector_field un = cvc::gradient_vector_diffusion(uni);
  EXPECT_NEAR(un.x[0], 0.6f, 1e-6);
  EXPECT_NEAR(un.y[0], -0.8f, 1e-6);
  // k = 0: every nonzero neighbour weighs the same, so a voxel takes its neighbours' mean.
  const cvc::vector_field f = random_field(4, 4, 4, 9);
  const cvc::vector_field m = cvc::gradient_vector_diffusion(f, raw(1, 0.0));
  const std::size_t c = f.index(1, 2, 1);
  const float mean = (f.x[f.index(2, 2, 1)] + f.x[f.index(0, 2, 1)] + f.x[f.index(1, 3, 1)] +
                      f.x[f.index(1, 1, 1)] + f.x[f.index(1, 2, 2)] + f.x[f.index(1, 2, 0)]) /
                     6.0f;
  EXPECT_NEAR(m.x[c], mean, 1e-6);
  // A voxel whose weighing vector is zero takes the plain mean too.
  cvc::vector_field h = random_field(3, 3, 3, 2);
  const std::size_t hc = h.index(1, 1, 1);
  h.x[hc] = h.y[hc] = h.z[hc] = 0.0f;
  const cvc::vector_field hm = cvc::gradient_vector_diffusion(h, raw(1));
  EXPECT_NEAR(hm.y[hc],
              (h.y[h.index(2, 1, 1)] + h.y[h.index(0, 1, 1)] + h.y[h.index(1, 2, 1)] +
               h.y[h.index(1, 0, 1)] + h.y[h.index(1, 1, 2)] + h.y[h.index(1, 1, 0)]) /
                  6.0f,
              1e-6);
  // No iterations and no normalisation: the field itself.
  const cvc::vector_field n0 = cvc::gradient_vector_diffusion(f, raw(0));
  EXPECT_TRUE(same_field(n0, f));
}

TEST(GradientVectorDiffusion, TheWeighingFieldMatters) {
  // After one iteration the two weighings agree (the field being diffused is the initial one);
  // after more they part.
  const cvc::vector_field f = random_field(8, 7, 6, 3);
  EXPECT_TRUE(
      same_field(cvc::gradient_vector_diffusion(f, raw(1)),
                 cvc::gradient_vector_diffusion(f, raw(1, 2.0, cvc::diffusion_weights::initial))));
  EXPECT_GT(
      max_diff(cvc::gradient_vector_diffusion(f, raw(3)),
               cvc::gradient_vector_diffusion(f, raw(3, 2.0, cvc::diffusion_weights::initial))),
      1e-4);
}

// ---- T3: known fields -------------------------------------------------------------------------

cvc::vector_field radial(int n, float sign) {
  cvc::vector_field f(n, n, n);
  const float c = (n - 1) / 2.0f;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const std::size_t v = f.index(i, j, k);
        f.x[v] = sign * (i - c);
        f.y[v] = sign * (j - c);
        f.z[v] = sign * (k - c);
      }
  return f;
}

TEST(GradientVectorDiffusion, CriticalKinds) {
  // A radial field's centre is a source (all point away: a minimum) or a sink (all point in: a
  // maximum). So is the rest of the 3x3x3 block around it, by the 26-neighbour test: the centre's
  // zero vector points nowhere, and the others point past them. Two voxels out, none is.
  const cvc::vector_field out = radial(7, 1.0f), in = radial(7, -1.0f);
  EXPECT_EQ(cvc::critical_point_at(out, 3, 3, 3), cvc::critical_kind::minimum);
  EXPECT_EQ(cvc::critical_point_at(in, 3, 3, 3), cvc::critical_kind::maximum);
  EXPECT_EQ(cvc::critical_point_at(out, 2, 3, 4), cvc::critical_kind::minimum);
  EXPECT_EQ(cvc::critical_point_at(in, 4, 4, 2), cvc::critical_kind::maximum);
  EXPECT_EQ(cvc::critical_point_at(out, 1, 3, 3), cvc::critical_kind::none);
  EXPECT_EQ(cvc::critical_point_at(in, 5, 5, 1), cvc::critical_kind::none);
  // A saddle: out along x, in along y and z.
  cvc::vector_field s = radial(5, 1.0f);
  for (std::size_t v = 0; v < s.size(); ++v) {
    s.y[v] = -s.y[v];
    s.z[v] = -s.z[v];
  }
  EXPECT_EQ(cvc::critical_point_at(s, 2, 2, 2), cvc::critical_kind::none);
  // A zero field is flat, nowhere critical; a lone vector with zero neighbours counts as a
  // minimum, as SegMed counted it (no neighbour points towards it).
  cvc::vector_field z(3, 3, 3);
  EXPECT_EQ(cvc::critical_point_at(z, 1, 1, 1), cvc::critical_kind::none);
  z.x[z.index(1, 1, 1)] = 1.0f;
  EXPECT_EQ(cvc::critical_point_at(z, 1, 1, 1), cvc::critical_kind::minimum);
  // At the volume's corner only the neighbours inside count.
  EXPECT_EQ(cvc::critical_point_at(radial(4, -1.0f), 0, 0, 0), cvc::critical_kind::minimum);
  // Diffusion keeps a radial field's centre what it was.
  for (const auto w : {cvc::diffusion_weights::current, cvc::diffusion_weights::initial}) {
    cvc::gradient_vector_diffusion_params p;
    p.weights = w;
    EXPECT_EQ(cvc::critical_point_at(cvc::gradient_vector_diffusion(radial(7, 1.0f), p), 3, 3, 3),
              cvc::critical_kind::minimum);
    EXPECT_EQ(cvc::critical_point_at(cvc::gradient_vector_diffusion(radial(7, -1.0f), p), 3, 3, 3),
              cvc::critical_kind::maximum);
  }
}

// ---- T4: invariants ---------------------------------------------------------------------------

TEST(GradientVectorDiffusion, Invariants) {
  for (unsigned seed = 1; seed <= 5; ++seed) {
    const cvc::vector_field f = random_field(9, 8, 7, seed, 0.1);
    const cvc::vector_field r = cvc::gradient_vector_diffusion(f);
    // Normalised: the longest vector has length 1.
    double longest = 0;
    for (std::size_t v = 0; v < r.size(); ++v)
      longest = std::max(longest, std::sqrt(double(r.x[v]) * r.x[v] + double(r.y[v]) * r.y[v] +
                                            double(r.z[v]) * r.z[v]));
    EXPECT_NEAR(longest, 1.0, 1e-6) << seed;
    // Mirrored in x (the field flipped and its x components negated), the result mirrors.
    cvc::vector_field m(f.nx, f.ny, f.nz);
    for (int k = 0; k < f.nz; ++k)
      for (int j = 0; j < f.ny; ++j)
        for (int i = 0; i < f.nx; ++i) {
          const std::size_t a = f.index(i, j, k), b = f.index(f.nx - 1 - i, j, k);
          m.x[b] = -f.x[a];
          m.y[b] = f.y[a];
          m.z[b] = f.z[a];
        }
    const cvc::vector_field rm = cvc::gradient_vector_diffusion(m);
    double worst = 0;
    for (int k = 0; k < f.nz; ++k)
      for (int j = 0; j < f.ny; ++j)
        for (int i = 0; i < f.nx; ++i) {
          const std::size_t a = r.index(i, j, k), b = r.index(r.nx - 1 - i, j, k);
          worst =
              std::max({worst, double(std::fabs(rm.x[b] + r.x[a])),
                        double(std::fabs(rm.y[b] - r.y[a])), double(std::fabs(rm.z[b] - r.z[a]))});
        }
    EXPECT_LE(worst, 1e-5) << seed;
    // Scaling the input scales nothing: the angles and the normalisation do not see it.
    cvc::vector_field s = f;
    for (std::size_t v = 0; v < s.size(); ++v) {
      s.x[v] *= 4.0f;
      s.y[v] *= 4.0f;
      s.z[v] *= 4.0f;
    }
    EXPECT_LE(max_diff(cvc::gradient_vector_diffusion(s), r), 1e-5) << seed;
  }
}

// ---- T5: bad input ----------------------------------------------------------------------------

template <class F> std::string message_of(F f) {
  try {
    f();
  } catch (const cvc::gradient_vector_diffusion_error &e) {
    return e.what();
  } catch (const std::exception &e) {
    return std::string("WRONG TYPE: ") + e.what();
  }
  return "no exception";
}

TEST(GradientVectorDiffusion, BadInputThrowsWithAUsefulMessage) {
  cvc::vector_field f = random_field(3, 3, 3, 1);
  f.y.pop_back();
  EXPECT_NE(message_of([&] { cvc::gradient_vector_diffusion(f); }).find("not the 3x3x3 grid's 27"),
            std::string::npos);
  cvc::vector_field nan = random_field(3, 4, 5, 1);
  nan.z[nan.index(2, 1, 3)] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_NE(
      message_of([&] { cvc::gradient_vector_diffusion(nan); }).find("(2, 1, 3) is not finite"),
      std::string::npos);
  EXPECT_NE(message_of([&] {
              cvc::gradient_vector_diffusion(random_field(2, 2, 2, 1), raw(-1));
            }).find("iterations"),
            std::string::npos);
  EXPECT_NE(message_of([&] {
              cvc::gradient_vector_diffusion(random_field(2, 2, 2, 1),
                                             raw(1, std::numeric_limits<double>::infinity()));
            }).find("k"),
            std::string::npos);
  // An empty grid is no error: nothing to diffuse.
  EXPECT_EQ(cvc::gradient_vector_diffusion(cvc::vector_field()).size(), 0u);
}

// ---- T6: no singletons ------------------------------------------------------------------------

TEST(GradientVectorDiffusion, CallsAreIndependent) {
  const cvc::vector_field a = random_field(16, 12, 10, 21), b = random_field(10, 14, 9, 22, 0.2);
  const auto pa = raw(6), pb = raw(3, 0.5, cvc::diffusion_weights::initial);
  const cvc::vector_field ra = cvc::gradient_vector_diffusion(a, pa);
  const cvc::vector_field rb = cvc::gradient_vector_diffusion(b, pb);
  EXPECT_TRUE(same_field(cvc::gradient_vector_diffusion(b, pb), rb));
  EXPECT_TRUE(same_field(cvc::gradient_vector_diffusion(a, pa), ra));
  cvc::vector_field ta, tb;
  std::thread t1([&] { ta = cvc::gradient_vector_diffusion(a, pa); });
  std::thread t2([&] { tb = cvc::gradient_vector_diffusion(b, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same_field(ta, ra));
  EXPECT_TRUE(same_field(tb, rb));
}

// ---- T7: resumable ----------------------------------------------------------------------------

TEST(GradientVectorDiffusion, StepperIsResumable) {
  const cvc::vector_field f = random_field(20, 18, 16, 5, 0.05);
  cvc::gradient_vector_diffusion_params p;
  p.iterations = 4;
  cvc::test::t7_subject<cvc::gradient_vector_diffusion_stepper, cvc::vector_field> s;
  s.name = "gradient_vector_diffusion 20x18x16";
  s.make = [&] { return std::make_unique<cvc::gradient_vector_diffusion_stepper>(f, p); };
  s.take = [](cvc::gradient_vector_diffusion_stepper &st) { return st.take(); };
  s.equal = same_field;
  s.blocking = [&] { return cvc::gradient_vector_diffusion(f, p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "diffusing", "normalising"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST(GradientVectorDiffusion, UnitsAreVoxelUpdates) {
  // A unit per voxel checked, per voxel per iteration, and two per voxel normalised.
  const cvc::vector_field f = random_field(10, 9, 8, 6);
  cvc::gradient_vector_diffusion_params p;
  p.iterations = 3;
  cvc::gradient_vector_diffusion_stepper st(f, p);
  std::size_t steps = 0;
  while (!st.step(1))
    ++steps;
  EXPECT_EQ(st.units(), f.size() * (1 + 3 + 2));
  EXPECT_EQ(steps + 1, st.units());
}

TEST(GradientVectorDiffusion, ACancelledStepperGivesNothing) {
  auto job = cvc::gradient_vector_diffusion_job(random_field(16, 16, 16, 7));
  job->step(100);
  EXPECT_FALSE(job->done());
  job->cancel();
  EXPECT_TRUE(job->step(100));
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  cvc::vector_field bad = random_field(2, 2, 2, 1);
  bad.x.resize(3);
  auto failed = cvc::gradient_vector_diffusion_job(bad);
  EXPECT_TRUE(failed->step(0));
  EXPECT_TRUE(failed->failed());
  EXPECT_THROW((void)failed->result(), cvc::gradient_vector_diffusion_error);
}

// ---- T8: the parameter table ------------------------------------------------------------------

TEST(GradientVectorDiffusion, ParameterTable) {
  cvc::app a;
  cvc::gradient_vector_diffusion_params p;
  p.iterations = 3;
  p.k = 0.75;
  p.weights = cvc::diffusion_weights::initial;
  p.normalize = false;
  EXPECT_TRUE(cvc::params_validate(p).empty());
  cvc::state &node = a.root()("doc.gvd");
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("weights").value(), "initial");
  EXPECT_EQ(node("weights").property("choices"), "current,initial");
  const auto back = cvc::params_from_state<cvc::gradient_vector_diffusion_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.iterations, 3);
  EXPECT_EQ(back.value.k, 0.75);
  EXPECT_EQ(back.value.weights, cvc::diffusion_weights::initial);
  EXPECT_EQ(back.value.normalize, false);
  const cvc::gradient_vector_diffusion_params d; // SegMed's numbers, the paper's weights
  EXPECT_EQ(d.iterations, 10);
  EXPECT_EQ(d.k, 2.0);
  EXPECT_EQ(d.weights, cvc::diffusion_weights::current);
  EXPECT_TRUE(d.normalize);
  p.k = -1;
  EXPECT_FALSE(cvc::params_validate(p).empty());
}

} // namespace

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::result_job<R> (cvc/core/result_job.h) and the node keys of a job (cvc/core/job_keys.h), the
// VolumeRover ports' shared job pieces (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.9).
// No port exists yet, so the T7 harness (resumable_harness.h) proves them on libcvc's own steppers:
// contour_tree_builder, resize_stepper and (with libigl) fem::stepped_solve. Small fake steppers
// cover what those cannot reach: a stepper that throws, one with its own status() and cancel(), a
// cancel from another thread. Runs natively and under node (the ports lane).

#include "resumable_harness.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state.h>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_jobs.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#ifdef CVC_ENABLE_LIBIGL
#include <cvc/geometry/fem.h>
#include <cvc/geometry/geometry.h>
#endif

namespace {

using cvc::result_job;

// --- fake steppers ------------------------------------------------------------------------------

// Counts `n` units in two named phases; its result is the sum 0 + 1 + ... + n-1. Throws when the
// count reaches `throw_at` (if set).
struct counter {
  std::uint64_t n = 1000, i = 0, sum = 0;
  std::uint64_t throw_at = ~std::uint64_t(0);
  bool step(std::size_t budget) {
    const std::uint64_t end = budget == 0 ? n : std::min<std::uint64_t>(n, i + budget);
    for (; i < end; ++i) {
      if (i == throw_at)
        throw std::invalid_argument("boom at " + std::to_string(i));
      sum += i;
    }
    return i >= n;
  }
  double progress() const { return n ? static_cast<double>(i) / static_cast<double>(n) : 1.0; }
  const char *phase() const { return i < n / 2 ? "first half" : "second half"; }
  std::uint64_t take() { return sum; }
};

// The same with no phase(), and a status() and a cancel() of its own.
struct chatty {
  std::uint64_t n = 100, i = 0;
  std::atomic<int> *cancels = nullptr;
  bool step(std::size_t budget) {
    i = std::min<std::uint64_t>(n, i + (budget == 0 ? n : budget));
    return i >= n;
  }
  double progress() const { return static_cast<double>(i) / static_cast<double>(n); }
  std::string status() const { return "counted " + std::to_string(i); }
  void cancel() {
    if (cancels)
      ++*cancels;
  }
  int take() { return static_cast<int>(i); }
};

TEST(ResultJob, StepsToItsResult) {
  std::unique_ptr<result_job<std::uint64_t>> job = cvc::make_result_job(counter{});
  EXPECT_FALSE(job->done());
  EXPECT_FALSE(job->has_result());
  EXPECT_THROW((void)job->result(), cvc::result_job_error) << "not done yet";
  double last = 0.0;
  int steps = 0;
  while (!job->step(7)) {
    ++steps;
    EXPECT_GE(job->progress(), last);
    last = job->progress();
    EXPECT_LT(job->progress(), 1.0);
    const std::string ph = job->phase();
    EXPECT_TRUE(ph == "first half" || ph == "second half") << ph;
    EXPECT_EQ(job->status().rfind(ph + " ", 0), 0u) << job->status();
    EXPECT_EQ(job->status().back(), '%');
  }
  EXPECT_GT(steps, 100);
  EXPECT_TRUE(job->done());
  EXPECT_FALSE(job->cancelled());
  EXPECT_FALSE(job->failed());
  EXPECT_STREQ(job->phase(), "done");
  EXPECT_EQ(job->status(), "done");
  EXPECT_EQ(job->progress(), 1.0);
  EXPECT_EQ(job->result(), 999u * 1000u / 2u);
  EXPECT_EQ(job->steps(), static_cast<std::size_t>(steps + 1));
  EXPECT_GE(job->work_ms(), 0.0);
  EXPECT_GE(job->work_ms(), job->max_step_ms());
  EXPECT_EQ(job->take_result(), 999u * 1000u / 2u);
  EXPECT_THROW((void)job->take_result(), cvc::result_job_error) << "taken once";
  EXPECT_TRUE(job->step(5)) << "a finished job stays finished";
}

TEST(ResultJob, BudgetZeroRunsToTheEndInOneStep) {
  auto job = cvc::make_result_job(counter{});
  EXPECT_TRUE(job->step(0));
  EXPECT_EQ(job->steps(), 1u);
  EXPECT_EQ(job->result(), 999u * 1000u / 2u);
}

TEST(ResultJob, AThrowingStepFailsTheJob) {
  counter c;
  c.throw_at = 600;
  auto job = cvc::make_result_job(std::move(c));
  while (!job->step(64)) {
  }
  EXPECT_TRUE(job->done());
  EXPECT_TRUE(job->failed());
  EXPECT_FALSE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  EXPECT_STREQ(job->phase(), "failed");
  EXPECT_EQ(job->error(), "boom at 600");
  EXPECT_EQ(job->status(), "failed: boom at 600");
  EXPECT_THROW((void)job->result(), std::invalid_argument) << "the stepper's own exception";
  EXPECT_TRUE(job->step(1));
}

TEST(ResultJob, CancelBeforeAndDuringSteps) {
  {
    auto job = cvc::make_result_job(counter{});
    job->cancel();
    EXPECT_TRUE(job->done());
    EXPECT_TRUE(job->cancelled());
    EXPECT_STREQ(job->phase(), "cancelled");
    EXPECT_TRUE(job->step(10));
    EXPECT_EQ(job->status(), "cancelled");
    EXPECT_THROW((void)job->result(), cvc::result_job_error);
  }
  {
    auto job = cvc::make_result_job(counter{});
    job->step(10);
    job->cancel();
    job->cancel(); // twice is once
    EXPECT_TRUE(job->step(10));
    EXPECT_TRUE(job->cancelled());
    EXPECT_FALSE(job->has_result());
  }
  {
    // A cancel after the result is ready keeps the result: the job was not cancelled.
    auto job = cvc::make_result_job(counter{});
    job->step(0);
    job->cancel();
    EXPECT_FALSE(job->cancelled());
    EXPECT_TRUE(job->has_result());
  }
}

TEST(ResultJob, CancelFromAnotherThreadEndsItAtTheNextStep) {
  counter c;
  c.n = std::uint64_t(1) << 40; // never finishes by itself
  std::shared_ptr<result_job<std::uint64_t>> job(cvc::make_result_job(std::move(c)));
  std::atomic<bool> started{false};
  std::thread t([&] {
    while (!job->step(1000))
      started = true;
  });
  while (!started)
    std::this_thread::yield();
  EXPECT_GT(job->progress(), 0.0); // read while it steps
  job->cancel();
  t.join();
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
}

TEST(ResultJob, AStepperOwnStatusAndCancelAreUsed) {
  std::atomic<int> cancels{0};
  chatty c;
  c.cancels = &cancels;
  auto job = cvc::make_result_job(std::move(c));
  job->step(10);
  EXPECT_STREQ(job->phase(), "running") << "a stepper with no phase()";
  EXPECT_EQ(job->status(), "counted 10");
  job->cancel();
  EXPECT_EQ(cancels.load(), 1) << "the stepper's own cancel(), for a stop within a step";
  job->step(1);
  job->cancel();
  EXPECT_EQ(cancels.load(), 1) << "once, and never after the stepper is gone";
}

TEST(ResultJob, ATakerMakesTheResult) {
  auto job = cvc::make_result_job<std::string>(std::make_unique<counter>(),
                                               [](counter &c) { return std::to_string(c.sum); });
  job->step(0);
  EXPECT_EQ(job->result(), "499500");
}

// --- the real steppers --------------------------------------------------------------------------

cvc::volume float_volume(cvc::app &a, std::uint64_t nx, std::uint64_t ny, std::uint64_t nz,
                         const std::vector<float> &v) {
  return cvc::volume(a, reinterpret_cast<const unsigned char *>(v.data()),
                     cvc::dimension(nx, ny, nz), cvc::Float);
}

// Seeded noise: a contour tree with many nodes.
std::vector<float> noise(std::size_t n, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> d(0.0f, 1.0f);
  std::vector<float> v(n);
  for (float &x : v)
    x = d(rng);
  return v;
}

// A few Gaussian blobs: a small tree, smooth data.
std::vector<float> blobs(std::size_t nx, std::size_t ny, std::size_t nz) {
  std::vector<float> v(nx * ny * nz);
  const double c[3][3] = {{0.3, 0.3, 0.3}, {0.7, 0.6, 0.4}, {0.4, 0.75, 0.7}};
  for (std::size_t k = 0; k < nz; ++k)
    for (std::size_t j = 0; j < ny; ++j)
      for (std::size_t i = 0; i < nx; ++i) {
        const double x = double(i) / nx, y = double(j) / ny, z = double(k) / nz;
        double f = 0.0;
        for (const auto &b : c)
          f += std::exp(-30.0 * ((x - b[0]) * (x - b[0]) + (y - b[1]) * (y - b[1]) +
                                 (z - b[2]) * (z - b[2])));
        v[i + nx * (j + ny * k)] = static_cast<float>(f);
      }
  return v;
}

bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

bool same_tree(const cvc::contour_tree &a, const cvc::contour_tree &b) {
  if (a.dims != b.dims || !same_bits(a.value_min, b.value_min) ||
      !same_bits(a.value_max, b.value_max) || a.nodes.size() != b.nodes.size() ||
      a.arcs.size() != b.arcs.size())
    return false;
  for (std::size_t i = 0; i < a.nodes.size(); ++i)
    if (a.nodes[i].voxel != b.nodes[i].voxel || !same_bits(a.nodes[i].value, b.nodes[i].value) ||
        a.nodes[i].kind != b.nodes[i].kind)
      return false;
  for (std::size_t i = 0; i < a.arcs.size(); ++i)
    if (a.arcs[i].hi != b.arcs[i].hi || a.arcs[i].lo != b.arcs[i].lo)
      return false;
  return true;
}

cvc::test::t7_subject<cvc::contour_tree_builder, cvc::contour_tree>
tree_subject(const std::string &name, const cvc::volume &vol, cvc::contour_tree_options opt = {}) {
  cvc::test::t7_subject<cvc::contour_tree_builder, cvc::contour_tree> s;
  s.name = name;
  s.make = [vol, opt] { return std::make_unique<cvc::contour_tree_builder>(vol, opt); };
  s.take = [](cvc::contour_tree_builder &b) { return b.take(); };
  s.equal = same_tree;
  s.blocking = [vol, opt] { return cvc::compute_contour_tree(vol, opt); };
  return s;
}

TEST(ResultJobT7, ContourTreeOfNoise) {
  cvc::app a;
  // About 46k voxels: some tens of ms, so the cooperative run spans several 4 ms slices.
  const cvc::volume vol = float_volume(a, 40, 36, 32, noise(40 * 36 * 32, 7));
  const cvc::test::t7_report r = cvc::test::run_t7(tree_subject("contour tree, noise", vol));
  // Every phase the builder documents, in order.
  const std::vector<std::string> want{"preparing", "sorting", "join tree", "split tree", "merging"};
  EXPECT_EQ(r.phases, want);
  EXPECT_GT(r.steps_at_budget_1, 15u * 40 * 36 * 32 / 2) << "about 15 units a voxel";
  // A step of 4096 units costs a few ms at most (contour_tree.h measured 3.5-5 ms on a loaded
  // desktop); a second means the budget was not honoured. Sanitizer builds stay far under it.
  EXPECT_LT(r.slowest_step_ms, 1000.0);
  EXPECT_GT(r.frames, 2u);
}

TEST(ResultJobT7, ContourTreeOfBlobsResampledAndQuantized) {
  cvc::app a;
  const cvc::volume vol = float_volume(a, 33, 28, 24, blobs(33, 28, 24));
  cvc::contour_tree_options opt;
  opt.max_dim = 20;
  opt.quantize_u8 = true;
  cvc::test::t7_options o;
  o.budgets = {1, 64, 4097, 0};
  cvc::test::run_t7(tree_subject("contour tree, blobs, resampled", vol, opt), o);
}

TEST(ResultJobT7, ResizeStepper) {
  cvc::app a;
  const cvc::volume vol = float_volume(a, 17, 13, 11, noise(17 * 13 * 11, 3));
  const cvc::dimension target(33, 20, 9);
  cvc::test::t7_subject<cvc::resize_stepper, cvc::volume> s;
  s.name = "resize 17x13x11 -> 33x20x9";
  s.make = [vol, target] { return std::make_unique<cvc::resize_stepper>(vol, target); };
  s.take = [](cvc::resize_stepper &r) { return r.take(); };
  s.equal = [](const cvc::volume &x, const cvc::volume &y) {
    return x.voxel_dimensions() == y.voxel_dimensions() && x.voxelType() == y.voxelType() &&
           std::memcmp(x.data_ptr(), y.data_ptr(), x.voxel_dimensions().size() * x.voxelSize()) ==
               0;
  };
  s.blocking = [vol, target] {
    cvc::volume v(vol);
    v.resize(target);
    return v;
  };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  EXPECT_EQ(r.phases, std::vector<std::string>{}) << "resize_stepper names no phase";
}

TEST(ResultJobT7, VolumeJobsWrapTheirSteppers) {
  cvc::app a;
  const cvc::volume vol = float_volume(a, 12, 12, 12, blobs(12, 12, 12));
  auto ct = cvc::contour_tree_job(vol);
  while (!ct->step(256)) {
  }
  EXPECT_TRUE(same_tree(ct->result(), cvc::compute_contour_tree(vol)));
  auto rs = cvc::resize_job(vol, cvc::dimension(5, 6, 7));
  EXPECT_TRUE(rs->step(0));
  EXPECT_EQ(rs->result().voxel_dimensions(), cvc::dimension(5, 6, 7));
}

#ifdef CVC_ENABLE_LIBIGL
cvc::geometry cube_tets(int n) {
  cvc::geometry g;
  const auto id = [n](int i, int j, int k) {
    return cvc::index_t(i + (n + 1) * (j + (n + 1) * k));
  };
  for (int k = 0; k <= n; ++k)
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i)
        g.points().push_back({{double(i) / n, double(j) / n, double(k) / n}});
  const int perms[6][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}, {0, 2, 1}, {2, 1, 0}, {1, 0, 2}};
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        for (int p = 0; p < 6; ++p) {
          int c[3] = {i, j, k};
          cvc::geometry::tet_t t;
          t[0] = id(c[0], c[1], c[2]);
          for (int s = 0; s < 3; ++s) {
            ++c[perms[p][s]];
            t[s + 1] = id(c[0], c[1], c[2]);
          }
          if (p >= 3)
            std::swap(t[2], t[3]);
          g.tets().push_back(t);
        }
  return g;
}

TEST(ResultJobT7, FemSteppedPoisson) {
  auto cube = std::make_shared<const cvc::geometry>(cube_tets(6));
  cvc::fem::dirichlet bc;
  bc.vertices = cvc::fem::boundary_vertices(*cube);
  for (cvc::index_t v : bc.vertices) {
    const auto &p = cube->const_points()[v];
    bc.values.push_back(std::exp(p[0] + 0.5 * p[1]) * std::cos(p[2]));
  }
  const std::vector<double> f(cube->num_points(), 1.0);
  cvc::test::t7_subject<cvc::fem::stepped_solve, std::vector<double>> s;
  s.name = "fem poisson 6^3";
  s.make = [cube, f, bc] { return cvc::fem::stepped_solve::poisson(cube, f, bc); };
  s.take = [](cvc::fem::stepped_solve &st) { return st.field(); };
  s.equal = [](const std::vector<double> &x, const std::vector<double> &y) {
    return x.size() == y.size() && std::memcmp(x.data(), y.data(), x.size() * sizeof(double)) == 0;
  };
  // No blocking reference: the one-shot solve_poisson factors with Eigen's own LDL^T, so it agrees
  // to rounding (fem_test), and the stepper at budget 0 is the reference instead.
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  EXPECT_FALSE(r.phases.empty());
  const std::vector<double> one_shot = cvc::fem::solve_poisson(*cube, f, bc);
  auto job = cvc::make_result_job<std::vector<double>>(
      cvc::fem::stepped_solve::poisson(cube, f, bc),
      [](cvc::fem::stepped_solve &st) { return st.field(); });
  job->step(0);
  ASSERT_EQ(job->result().size(), one_shot.size());
  for (std::size_t i = 0; i < one_shot.size(); ++i)
    EXPECT_NEAR(job->result()[i], one_shot[i], 1e-9 * (1.0 + std::fabs(one_shot[i])));
}
#endif

// --- the node keys ------------------------------------------------------------------------------

class JobKeys : public ::testing::TestWithParam<cvc::sliced_threading> {
protected:
  std::string key(const std::string &k) { return app.root()("doc.job")(k).value(); }
  void pump_until_idle(int pid) { ASSERT_TRUE(cvc::wait_sliced_job(app, pid, 120.0)); }
  cvc::app app;
};

TEST_P(JobKeys, PublishedFromStartToResult) {
  const cvc::volume vol = float_volume(app, 48, 48, 48, noise(48 * 48 * 48, 11));
  std::shared_ptr<result_job<cvc::contour_tree>> job(cvc::contour_tree_job(vol));
  auto keys = std::make_shared<cvc::job_keys>(app.root(), "doc.job");
  keys->idle();
  EXPECT_EQ(key("busy"), "0");
  cvc::sliced_task_options o;
  o.kind = "test";
  o.threading = GetParam();
  int progress_notes = 0;
  o.on_progress = [&](const cvc::sliced_job &) { ++progress_notes; };
  std::size_t nodes = 0;
  const int pid =
      cvc::schedule_published_job(app, job, o, keys, "building...", [&](cvc::sliced_job &j) {
        nodes = static_cast<result_job<cvc::contour_tree> &>(j).result().nodes.size();
      });
  ASSERT_GE(pid, 0);
  EXPECT_EQ(key("busy"), "1");
  EXPECT_EQ(key("stoppable"), "1");
  EXPECT_EQ(key("progress"), "0");
  EXPECT_EQ(key("status"), "building...");
  EXPECT_EQ(key("phase"), "");
  EXPECT_EQ(key("pid"), std::to_string(pid));
  int last = 0;
  bool saw_phase = false;
  while (cvc::sliced_job_running(app, pid)) {
    cvc::test::detail::pump_frame(app);
    const int p = std::stoi(key("progress"));
    EXPECT_GE(p, last) << "progress never falls";
    last = p;
    const std::string ph = key("phase");
    if (!ph.empty()) {
      saw_phase = true;
      EXPECT_EQ(ph.back(), '%') << ph;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  EXPECT_GT(nodes, 0u) << "on_result took the result";
  EXPECT_EQ(nodes, cvc::compute_contour_tree(vol).nodes.size());
  EXPECT_EQ(key("busy"), "0");
  EXPECT_EQ(key("stoppable"), "0");
  EXPECT_EQ(key("progress"), "100");
  EXPECT_EQ(key("phase"), "");
  EXPECT_EQ(key("status"), "done");
  EXPECT_FALSE(key("stats.work_ms").empty());
  if (GetParam() == cvc::sliced_threading::cooperative) {
    EXPECT_GT(progress_notes, 0);
    EXPECT_TRUE(saw_phase) << "a 48^3 tree takes more than one 4 ms slice";
  }
}

TEST_P(JobKeys, AStopPublishesCancelledAndNoResult) {
  counter c;
  c.n = std::uint64_t(1) << 40;
  std::shared_ptr<result_job<std::uint64_t>> job(cvc::make_result_job(std::move(c)));
  auto keys = std::make_shared<cvc::job_keys>(app.root(), "doc.job");
  cvc::sliced_task_options o;
  o.threading = GetParam();
  bool took = false;
  const int pid = cvc::schedule_published_job(app, job, o, keys, "counting...",
                                              [&](cvc::sliced_job &) { took = true; });
  ASSERT_GE(pid, 0);
  for (int i = 0; i < 5; ++i)
    cvc::test::detail::pump_frame(app);
  EXPECT_TRUE(cvc::stop_sliced_job(app, pid));
  pump_until_idle(pid);
  EXPECT_FALSE(took);
  EXPECT_EQ(key("status"), "cancelled");
  EXPECT_EQ(key("busy"), "0");
  EXPECT_EQ(key("stoppable"), "0");
  EXPECT_FALSE(cvc::stop_sliced_job(app, pid)) << "gone";
  EXPECT_FALSE(cvc::stop_sliced_job(app, 987654)) << "never was";
}

TEST_P(JobKeys, AFailurePublishesItsMessage) {
  counter c;
  c.n = 100000;
  c.throw_at = 50000;
  std::shared_ptr<result_job<std::uint64_t>> job(cvc::make_result_job(std::move(c)));
  auto keys = std::make_shared<cvc::job_keys>(app.root(), "doc.job");
  cvc::sliced_task_options o;
  o.threading = GetParam();
  bool took = false;
  const int pid = cvc::schedule_published_job(app, job, o, keys, "counting...",
                                              [&](cvc::sliced_job &) { took = true; });
  pump_until_idle(pid);
  EXPECT_FALSE(took) << "a failed job has no result to take";
  EXPECT_EQ(key("status"), "failed: boom at 50000");
  EXPECT_EQ(key("busy"), "0");
}

INSTANTIATE_TEST_SUITE_P(Modes, JobKeys,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded),
                         [](const ::testing::TestParamInfo<cvc::sliced_threading> &i) {
                           return i.param == cvc::sliced_threading::cooperative
                                      ? std::string("Cooperative")
                                      : std::string("Threaded");
                         });

TEST(JobKeysUnit, PathsAndPhaseText) {
  cvc::app a;
  cvc::job_keys under(a.root(), "a.b");
  EXPECT_EQ(under.path("busy"), "a.b.busy");
  cvc::job_keys top(a.root(), "");
  EXPECT_EQ(top.path("busy"), "busy");
  EXPECT_EQ(cvc::job_phase_text("sorting", 42), "sorting 42%");
  EXPECT_EQ(cvc::job_phase_text(nullptr, 42), "");
  EXPECT_EQ(cvc::job_phase_text("", 42), "");
  under.started("go");
  under.progress(150, "x");
  EXPECT_EQ(a.root()("a.b.progress").value(), "100") << "clamped";
  under.stopping();
  EXPECT_EQ(a.root()("a.b.status").value(), "stopping...");
  cvc::job_keys untimed(a.root(), "c", {}, false);
  untimed.started("go");
  untimed.finished("ok");
  EXPECT_TRUE(a.root()("c.stats.work_ms").value().empty()) << "a node that times its own work";
}

} // namespace

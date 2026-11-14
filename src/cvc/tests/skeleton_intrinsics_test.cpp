/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// skeleton.compute (cvc/geometry/skeleton_intrinsics.h), the Skeletonization port in the DSL:
// the VolumeRover ports' T10 rows (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §1.7).
// The three forms on the app's scheduler, cooperative and threaded; a bad parameter raising at
// the call; job.cancel and a kill part-way through; params.defaults; and an Ariadne document
// starting it with -start from an action under the default limits, its progress keys rising while
// the frame loop runs. Runs natively and under node (the ports lane).

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_intrinsics.h>
#include <cvc/geometry/tet_complex.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

// A capsule's surface along x, as a geometry of points (the skeleton is one strand).
cvc::geometry capsule(cvc::app &app, int nring, int nalong) {
  const double pi = 3.14159265358979323846, length = 8.0, r = 1.0;
  cvc::geometry g(app);
  auto add = [&](double x, double y, double z) {
    g.points().push_back(cvc::geometry::point_t{{x, y, z}});
  };
  for (int i = 0; i <= nalong; ++i)
    for (int k = 0; k < nring; ++k) {
      const double a = 2 * pi * (k + 0.37 * (i % 3)) / nring;
      add(length * i / nalong, r * std::cos(a), r * std::sin(a));
    }
  const int ncap = nring / 4;
  for (int side = 0; side < 2; ++side)
    for (int i = 1; i <= ncap; ++i) {
      const double t = (pi / 2) * i / (ncap + 0.5);
      const int nr = std::max(1, int(std::lround(nring * std::cos(t))));
      for (int k = 0; k < nr; ++k) {
        const double a = 2 * pi * (k + 0.21 * i) / nr;
        add(side == 0 ? -r * std::sin(t) : length + r * std::sin(t), r * std::cos(t) * std::cos(a),
            r * std::cos(t) * std::sin(a));
      }
    }
  return g;
}

class SkeletonIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
protected:
  void SetUp() override {
    if (!cvc::tet_complex_available())
      GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
    se::async_scheduler &sched = app.exec_scheduler();
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    cvc::register_job_op(env, ictx, app, cvc::skeleton_compute_op(GetParam()));
    cvc::register_job_intrinsics(env, app);
    geom = capsule(app, 20, 40);
    app.root()("doc.points").data(boost::any(geom));
    expected = cvc::skeletonize(geom);
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

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
  cvc::geometry geom;
  cvc::skeleton expected;
};

TEST_P(SkeletonIntrinsics, TheWaitingFormReturnsTheSkeleton) {
  const int pid = run("(let ((s (skeleton.compute \"doc.points\")))"
                      "  (state-set \"r.polylines\" (get-attr s \"polylines\"))"
                      "  (state-set \"r.clusters\" (get-attr s \"clusters\"))"
                      "  (state-data-set \"r.strands\" (get-attr s \"strands\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.polylines"), std::to_string(expected.polylines));
  EXPECT_EQ(key("r.clusters"), std::to_string(expected.clusters));
  const boost::any a = app.root()("r.strands").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&a);
  ASSERT_NE(v, nullptr);
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v);
  ASSERT_TRUE(obj && *obj);
  EXPECT_EQ((*obj)->type_name, "cvc::geometry");
  const cvc::geometry *strands = boost::any_cast<cvc::geometry>(&(*obj)->payload);
  ASSERT_NE(strands, nullptr);
  EXPECT_EQ(strands->num_lines(), expected.strands.num_lines());
}

TEST_P(SkeletonIntrinsics, TheAsyncFormAndParameters) {
  const int pid =
      run("(let ((f (skeleton.compute-async \"doc.points\""
          "                                (dict \"sheet_count\" 1 \"helix_window\" #t))))"
          "  (state-set \"r.between\" \"ran\")"
          "  (state-set \"r.polylines\" (get-attr (await f) \"polylines\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.polylines"), std::to_string(expected.polylines));
  // A bad parameter raises at the call, naming it.
  const int bad = run("(skeleton.compute \"doc.points\" (dict \"threshold\" 5))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("threshold"), std::string::npos) << exit_error(bad);
  // The table, through params.defaults.
  const int defaults =
      run("(state-set \"r.sc\" (get-attr (params.defaults \"skeleton.compute\") \"sheet_count\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.sc"), "2");
}

TEST_P(SkeletonIntrinsics, TheStartFormPublishesTheKeysAndTheResult) {
  const int caller = run("(state-set \"r.pid\" (skeleton.compute-start \"doc.points\" \"job\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_FALSE(key("r.pid").empty());
  // Once done the status is the stepper's summary, "done: 1 strands, 0 sheets (0 clusters)".
  ASSERT_TRUE(pump_until([&] { return key("job.status").rfind("done", 0) == 0; }))
      << "status: " << key("job.status") << ", busy " << key("job.busy");
  EXPECT_EQ(key("job.busy"), "0");
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result lands at job.result";
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "polylines")->v), expected.polylines);
}

TEST_P(SkeletonIntrinsics, JobCancelAndKillStopIt) {
  // A bigger cloud, so the job is still running when the cancel arrives.
  app.root()("doc.big").data(boost::any(capsule(app, 48, 160)));
  const int caller = run("(state-set \"r.pid\" (skeleton.compute-start \"doc.big\" \"big\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  EXPECT_EQ(key("r.cancelled"), "true");
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_TRUE(app.root()("big.result").data().empty());
  // Killing the waiting process cancels its job: the scheduler ends up with no task running.
  const int waiting = run("(skeleton.compute \"doc.big\")");
  for (int i = 0; i < 10; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_FALSE(finished(waiting));
  app.exec_scheduler().kill(waiting);
  EXPECT_TRUE(pump_until([&] { return finished(waiting); }));
}

INSTANTIATE_TEST_SUITE_P(Modes, SkeletonIntrinsics,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded),
                         [](const auto &info) {
                           return info.param == cvc::sliced_threading::cooperative ? "cooperative"
                                                                                   : "threaded";
                         });

// ---- an Ariadne document starts it from an action, under the default limits ----------------

struct ClickBackend : cvc::ariadne::Backend {
  std::string click;
  cvc::ariadne::Capabilities capabilities() const override {
    cvc::ariadne::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const cvc::ariadne::Size &, float,
                    bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const cvc::ariadne::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *) override {}
  void text_value(const char *, const std::string &) override {}
  void separator() override {}
  bool button(const char *l) override {
    if (click == l) {
      click.clear();
      return true;
    }
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  cvc::ariadne::BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  cvc::ariadne::BoolEdit checkbox(const char *, bool) override { return {}; }
  cvc::ariadne::IntEdit slider_int(const char *, const char *, int, int, int) override {
    return {};
  }
  cvc::ariadne::DoubleEdit slider_double(const char *, const char *, double, double, double,
                                         const char *) override {
    return {};
  }
  cvc::ariadne::IndexEdit combo(const char *, int, const std::vector<std::string> &) override {
    return {};
  }
  cvc::ariadne::StringEdit input_text(const char *, const char *, const std::string &,
                                      bool) override {
    return {};
  }
};

TEST(SkeletonIntrinsicsAriadne, AnActionStartsItUnderTheDefaultLimits) {
  if (!cvc::tet_complex_available())
    GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::add_skeleton_intrinsics(app);
  // Big enough to span many frames and several 100 ms progress notes.
  const cvc::geometry pts = capsule(app, 40, 120);
  app.root()("doc.points").data(boost::any(pts));
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_time").value().empty());
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_steps").value().empty());

  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button(
      "Go", "(state-set \"r.pid\" (skeleton.compute-start \"doc.points\" \"job\"))")}));
  be.click = "Go";
  std::set<std::string> progress_seen;
  std::size_t frames = 0, frames_busy = 0;
  const auto t0 = steady::now();
  while (std::chrono::duration<double>(steady::now() - t0).count() < 300.0) {
    rt.render();
    rt.drain();
    ++frames;
    const std::string busy = app.root()("job.busy").value();
    if (busy == "1") {
      ++frames_busy;
      progress_seen.insert(app.root()("job.progress").value());
    }
    if (!app.root()("r.pid").value().empty() && busy == "0")
      break;
  }
  ASSERT_FALSE(app.root()("r.pid").value().empty()) << "the action never ran its -start";
  EXPECT_EQ(app.root()("job.status").value().rfind("done", 0), 0u)
      << app.root()("job.status").value();
  EXPECT_GT(frames_busy, 2u) << "the frames kept coming while the job ran";
  EXPECT_GE(progress_seen.size(), 2u) << "progress rose while the frame loop ran";
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result landed at job.result";
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "polylines")->v), cvc::skeletonize(pts).polylines);
  std::cout << "[ T10 ] skeleton.compute-start from an action: " << frames << " frames, "
            << frames_busy << " while busy, " << progress_seen.size() << " progress values\n";
}

} // namespace

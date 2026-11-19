/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tiler.tile (cvc/geometry/tiler_intrinsics.h), the ContourTiler port in the DSL: the VolumeRover
// ports' T10 rows (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §1.7). The three forms on
// the app's scheduler, cooperative and threaded; a stack as a data object or a state path; a bad
// parameter raising at the call; job.cancel and a kill part-way through; params.defaults; and an
// Ariadne document starting it with -start from an action under the default limits, its keys
// under `<node>.tiler` rising while the frame loop runs. Runs natively and under node (the ports
// lane).

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/geometry/contour_intrinsics.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_intrinsics.h>
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

#ifndef CVC_WITH_PLANAR
#define SKIP_WITHOUT_PLANAR() GTEST_SKIP() << "libcvc was built without cvc::planar"
#else
#define SKIP_WITHOUT_PLANAR()                                                                      \
  do {                                                                                             \
  } while (0)
#endif

constexpr double pi = 3.14159265358979323846;

// A coordinate on a 2^-8 grid, unchanged by the tiler's snapping.
double q8(double v) { return std::round(v * 256) / 256; }

// A wobbly ellipse, counterclockwise.
std::vector<double> blob(double cx, double cy, double a, double b, double phase, int n) {
  std::vector<double> xy;
  for (int i = 0; i < n; ++i) {
    const double t = 2 * pi * i / n;
    const double r = 1 + 0.05 * std::cos(3 * t + phase);
    xy.push_back(q8(cx + a * r * std::cos(t)));
    xy.push_back(q8(cy + b * r * std::sin(t)));
  }
  return xy;
}

// A tube of `slices` sections, `n` vertices each.
cvc::contour_stack tube(int slices, int n) {
  cvc::contour_stack s;
  for (int k = 0; k < slices; ++k)
    s.add(k, "tube", blob(8 + 0.125 * k, 8, 3 + 0.125 * (k % 2), 2.5, 0.4 * k, n));
  return s;
}

const cvc::geometry *geometry_in(const se::value_t &v) {
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v.v);
  if (!obj || !*obj || (*obj)->type_name != "cvc::geometry")
    return nullptr;
  return boost::any_cast<cvc::geometry>(&(*obj)->payload);
}

class TilerIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
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
    cvc::register_job_op(env, ictx, app, cvc::tiler_tile_op(GetParam()));
    cvc::register_job_intrinsics(env, app);
    stack = tube(6, 24);
    app.root()("doc.contours").data(boost::any(stack));
#ifdef CVC_WITH_PLANAR
    expected = cvc::tile_contours(stack);
#endif
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
  const cvc::geometry *stored_geometry(const std::string &path) {
    stored = app.root()(path).data();
    const se::value_t *v = boost::any_cast<se::value_t>(&stored);
    return v ? geometry_in(*v) : nullptr;
  }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
  cvc::contour_stack stack;
  cvc::tile_result expected;
  boost::any stored;
};

TEST_P(TilerIntrinsics, TheWaitingFormReturnsTheSurfaces) {
  SKIP_WITHOUT_PLANAR();
  const int pid =
      run("(let ((r (tiler.tile \"doc.contours\")))"
          "  (state-set \"r.warnings\" (length (get-attr r \"warnings\")))"
          "  (state-data-set \"r.tube\" (get-attr (get-attr r \"surfaces\") \"tube\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.warnings"), "0");
  const cvc::geometry *g = stored_geometry("r.tube");
  ASSERT_NE(g, nullptr) << "the surface is a cvc::geometry data object";
  ASSERT_EQ(expected.components.count("tube"), 1u);
  EXPECT_EQ(g->num_tris(), expected.components.at("tube").num_tris());
  EXPECT_EQ(g->num_points(), expected.components.at("tube").num_points());
  EXPECT_GT(g->num_tris(), 0u);
}

TEST_P(TilerIntrinsics, AStackDataObjectAndParameters) {
  SKIP_WITHOUT_PLANAR();
  // A stack as a data object (as contours.* and state-data-get hand them out), tiled without its
  // end caps.
  app.root()("doc.object").data(boost::any(cvc::contour_stack_value(stack)));
  const int pid = run("(let ((f (tiler.tile-async (state-data-get \"doc.object\")"
                      "                           (dict \"cap_ends\" 0))))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (state-data-set \"r.open\" (get-attr (get-attr (await f) \"surfaces\")"
                      "                                       \"tube\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  cvc::tile_params open;
  open.cap_ends = false;
  const cvc::tile_result r = cvc::tile_contours(stack, open);
  const cvc::geometry *g = stored_geometry("r.open");
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->num_tris(), r.components.at("tube").num_tris());
  EXPECT_LT(g->num_tris(), expected.components.at("tube").num_tris()) << "no caps";
  // A bad parameter raises at the call, naming it.
  const int bad = run("(tiler.tile \"doc.contours\" (dict \"curation_delta\" -1))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("curation_delta"), std::string::npos) << exit_error(bad);
  // So does a curation_delta the slices are too close for (the tiler's own check, in the job).
  const int close = run("(tiler.tile \"doc.contours\" (dict \"curation_delta\" 2))");
  ASSERT_TRUE(pump_until([&] { return finished(close); }));
  EXPECT_NE(exit_error(close).find("curation_delta"), std::string::npos) << exit_error(close);
  // Something that is not a stack.
  const int wrong = run("(tiler.tile \"doc.nothing\")");
  ASSERT_TRUE(pump_until([&] { return finished(wrong); }));
  EXPECT_NE(exit_error(wrong).find("tiler.tile"), std::string::npos) << exit_error(wrong);
  // The table, through params.defaults.
  const int defaults =
      run("(state-set \"r.ce\" (get-attr (params.defaults \"tiler.tile\") \"cap_ends\"))"
          "(state-set \"r.zf\" (get-attr (params.defaults \"tiler.tile\") \"z_first\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.ce"), "true");
  EXPECT_EQ(key("r.zf"), "-1");
}

TEST_P(TilerIntrinsics, TheStartFormPublishesTheKeysUnderTheNode) {
  SKIP_WITHOUT_PLANAR();
  const int caller =
      run("(state-set \"r.pid\" (tiler.tile-start \"doc.contours\" \"doc.lab.tiler\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_FALSE(key("r.pid").empty());
  // Once done the status is the stepper's summary, "done: 1 surfaces, N triangles, 0 warnings".
  ASSERT_TRUE(pump_until([&] { return key("doc.lab.tiler.status").rfind("done", 0) == 0; }))
      << "status: " << key("doc.lab.tiler.status") << ", busy " << key("doc.lab.tiler.busy");
  EXPECT_NE(key("doc.lab.tiler.status").find("1 surfaces"), std::string::npos);
  EXPECT_EQ(key("doc.lab.tiler.busy"), "0");
  EXPECT_EQ(key("doc.lab.tiler.progress"), "100");
  const boost::any r = app.root()("doc.lab.tiler.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result lands at doc.lab.tiler.result";
  const se::value_t *surfaces = se::field(*v, "surfaces");
  ASSERT_NE(surfaces, nullptr);
  const se::value_t *tube_surface = se::field(*surfaces, "tube");
  ASSERT_NE(tube_surface, nullptr);
  const cvc::geometry *g = geometry_in(*tube_surface);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->num_tris(), expected.components.at("tube").num_tris());
}

TEST_P(TilerIntrinsics, JobCancelAndKillStopIt) {
  SKIP_WITHOUT_PLANAR();
  // A longer, denser tube, so the job is still running when the cancel arrives.
  app.root()("doc.big").data(boost::any(tube(120, 160)));
  const int caller = run("(state-set \"r.pid\" (tiler.tile-start \"doc.big\" \"big\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  EXPECT_EQ(key("r.cancelled"), "true");
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_TRUE(app.root()("big.result").data().empty());
  // Killing the waiting process cancels its job: the scheduler ends up with no task running.
  const int waiting = run("(tiler.tile \"doc.big\")");
  for (int i = 0; i < 10; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_FALSE(finished(waiting));
  app.exec_scheduler().kill(waiting);
  EXPECT_TRUE(pump_until([&] { return finished(waiting); }));
}

INSTANTIATE_TEST_SUITE_P(Modes, TilerIntrinsics,
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

TEST(TilerIntrinsicsAriadne, AnActionStartsItUnderTheDefaultLimits) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::add_tiler_intrinsics(app);
  // Big enough to span many frames and several 100 ms progress notes.
  const cvc::contour_stack s = tube(60, 192);
  app.root()("doc.lab.contours").data(boost::any(s));
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_time").value().empty());
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_steps").value().empty());

  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button(
      "Tile", "(state-set \"r.pid\" (tiler.tile-start \"doc.lab.contours\" \"doc.lab.tiler\"))")}));
  be.click = "Tile";
  std::set<std::string> progress_seen;
  std::size_t frames = 0, frames_busy = 0;
  const auto t0 = steady::now();
  // 109-111 s in a Debug job, past 300 s on a loaded runner: ctest gives the suite 900 s too.
  while (std::chrono::duration<double>(steady::now() - t0).count() < 900.0) {
    rt.render();
    rt.drain();
    ++frames;
    const std::string busy = app.root()("doc.lab.tiler.busy").value();
    if (busy == "1") {
      ++frames_busy;
      progress_seen.insert(app.root()("doc.lab.tiler.progress").value());
    }
    if (!app.root()("r.pid").value().empty() && busy == "0")
      break;
  }
  ASSERT_FALSE(app.root()("r.pid").value().empty()) << "the action never ran its -start";
  EXPECT_EQ(app.root()("doc.lab.tiler.status").value().rfind("done", 0), 0u)
      << app.root()("doc.lab.tiler.status").value();
  EXPECT_GT(frames_busy, 2u) << "the frames kept coming while the job ran";
  EXPECT_GE(progress_seen.size(), 2u) << "progress rose while the frame loop ran";
  const boost::any r = app.root()("doc.lab.tiler.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result landed at doc.lab.tiler.result";
  const se::value_t *surfaces = se::field(*v, "surfaces");
  ASSERT_NE(surfaces, nullptr);
  const se::value_t *tube_surface = se::field(*surfaces, "tube");
  ASSERT_NE(tube_surface, nullptr);
  const cvc::geometry *surface = geometry_in(*tube_surface);
  ASSERT_NE(surface, nullptr);
  EXPECT_EQ(surface->num_tris(), cvc::tile_contours(s).components.at("tube").num_tris());
  std::cout << "[ T10 ] tiler.tile-start from an action: " << frames << " frames, " << frames_busy
            << " while busy, " << progress_seen.size() << " progress values\n";
}

} // namespace

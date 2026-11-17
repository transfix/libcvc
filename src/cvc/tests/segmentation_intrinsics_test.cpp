/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// segment.seeded (cvc/volume/segmentation_intrinsics.h), the GenSeg port in the DSL: the
// VolumeRover ports' T10 rows (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §1.7). The
// three forms on the app's scheduler, cooperative and threaded; a bad parameter raising at the
// call; job.cancel and a kill part-way through; params.defaults; and an Ariadne document starting
// it with -start from an action under the default limits, its progress keys rising while the frame
// loop runs. Runs natively and under node (the ports lane).

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/point_class_intrinsics.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_intrinsics.h>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

// n^3 voxels: two Gaussian bumps along x over a gentle ramp, so every voxel is above t_low and the
// final march visits them all (which makes the job last).
cvc::volume bumps(cvc::app &app, int n) {
  cvc::volume v(app, cvc::dimension(n, n, n), cvc::Float);
  float *d = reinterpret_cast<float *>(v.data_ptr());
  const double s = n / 6.0;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double c = n / 2.0, a = n / 4.0, b = 3 * n / 4.0;
        const double r2 = (j - c) * (j - c) + (k - c) * (k - c);
        const double f = std::exp(-((i - a) * (i - a) + r2) / (2 * s * s)) +
                         std::exp(-((i - b) * (i - b) + r2) / (2 * s * s));
        d[(std::size_t(k) * n + j) * n + i] = float(f + 0.5 + 0.001 * i);
      }
  return v;
}

cvc::point_classes two_seeds(int n) {
  cvc::point_classes pc;
  pc.add("left", n / 4, n / 2, n / 2);
  pc.add("right", 3 * n / 4, n / 2, n / 2);
  return pc;
}

cvc::seeded_segmentation_params test_params() {
  cvc::seeded_segmentation_params p;
  p.t_low = 0.1; // data units: every voxel passes
  p.t_high = 10.0;
  p.gamma = -0.02; // gentle edges, so the fronts cross the whole volume
  return p;
}

const char *params_dict = "(dict \"t_low\" 0.1 \"t_high\" 10.0 \"gamma\" -0.02)";

class SegmentationIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
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
    cvc::register_job_op(env, ictx, app, cvc::segment_seeded_op(GetParam()));
    cvc::register_job_op(env, ictx, app, cvc::critical_point_seeds_op(GetParam()));
    cvc::register_job_intrinsics(env, app);
    cvc::register_point_class_intrinsics(env, ictx, app);
    vol.emplace(bumps(app, 20));
    app.root()("doc.volume").data(boost::any(*vol));
    app.root()("doc.seeds").data(boost::any(two_seeds(20)));
    expected.emplace(cvc::segment_seeded(*vol, two_seeds(20), test_params()));
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
  // The payload of the data object at `path` (stored there with state-data-set).
  template <class T> std::shared_ptr<const T> object_at(const std::string &path) {
    const boost::any a = app.root()(path).data();
    const se::value_t *v = boost::any_cast<se::value_t>(&a);
    if (!v)
      return nullptr;
    const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v);
    if (!obj || !*obj)
      return nullptr;
    const auto *p = boost::any_cast<std::shared_ptr<const T>>(&(*obj)->payload);
    return p ? *p : nullptr;
  }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
  std::optional<cvc::volume> vol;
  std::optional<cvc::seeded_segmentation> expected;
};

TEST_P(SegmentationIntrinsics, TheWaitingFormReturnsTheSegmentation) {
  const int pid =
      run(std::string("(let ((s (segment.seeded \"doc.volume\" \"doc.seeds\" ") + params_dict +
          ")))"
          "  (state-set \"r.names\" (str (get-attr s \"class_names\")))"
          "  (state-set \"r.left\" (nth (get-attr s \"class_voxels\") 0))"
          "  (state-set \"r.border\" (get-attr (point-classes.info (get-attr s \"border\"))"
          "                                    \"points\"))"
          "  (state-data-set \"r.labels\" (get-attr s \"labels\"))"
          "  (state-data-set \"r.class1\" (nth (get-attr s \"classes\") 1)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_NE(key("r.names").find("left"), std::string::npos) << key("r.names");
  EXPECT_EQ(key("r.left"), std::to_string(expected->class_voxels[0]));
  EXPECT_EQ(key("r.border"), std::to_string(expected->border.size()));
  const auto labels = object_at<cvc::volume>("r.labels");
  ASSERT_TRUE(labels);
  EXPECT_EQ(labels->voxelType(), cvc::UChar);
  EXPECT_EQ(std::memcmp(labels->data_ptr(), expected->labels.data_ptr(), 20 * 20 * 20), 0);
  const auto class1 = object_at<cvc::volume>("r.class1");
  ASSERT_TRUE(class1);
  EXPECT_EQ(class1->voxelType(), cvc::Float);
}

TEST_P(SegmentationIntrinsics, TheAsyncFormAndParameters) {
  const int pid = run(std::string("(let ((f (segment.seeded-async \"doc.volume\" \"doc.seeds\" ") +
                      params_dict +
                      ")))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (state-set \"r.right\" (nth (get-attr (await f) \"class_voxels\") 1)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.right"), std::to_string(expected->class_voxels[1]));
  // A bad parameter raises at the call, naming it; so does a bad input.
  const int bad = run("(segment.seeded \"doc.volume\" \"doc.seeds\" (dict \"gamma\" 0.5))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("gamma"), std::string::npos) << exit_error(bad);
  const int none = run("(segment.seeded \"doc.volume\" (point-classes.remove nil \"x\" 0 0 0))");
  ASSERT_TRUE(pump_until([&] { return finished(none); }));
  EXPECT_NE(exit_error(none), "");
  // The table, through params.defaults.
  const int defaults =
      run("(state-set \"r.gamma\" (get-attr (params.defaults \"segment.seeded\") \"gamma\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.gamma"), "-0.12");
}

TEST_P(SegmentationIntrinsics, TheStartFormPublishesTheKeysAndTheResult) {
  const int caller = run(std::string("(state-set \"r.pid\" (segment.seeded-start \"doc.volume\" "
                                     "\"doc.seeds\" ") +
                         params_dict + " \"job\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_FALSE(key("r.pid").empty());
  ASSERT_TRUE(pump_until([&] { return !app.root()("job.result").data().empty(); }))
      << "status: " << key("job.status") << ", busy " << key("job.busy");
  ASSERT_TRUE(pump_until([&] { return key("job.busy") == "0"; }));
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result lands at job.result";
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "automatic_seeds")->v),
            std::int64_t(expected->automatic_seeds));
}

TEST_P(SegmentationIntrinsics, JobCancelAndKillStopIt) {
  // A bigger volume, so the job is still running when the cancel arrives.
  app.root()("doc.big").data(boost::any(bumps(app, 128)));
  app.root()("doc.big_seeds").data(boost::any(two_seeds(128)));
  const int caller = run(std::string("(state-set \"r.pid\" (segment.seeded-start \"doc.big\" "
                                     "\"doc.big_seeds\" ") +
                         params_dict + " \"big\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  EXPECT_EQ(key("r.cancelled"), "true");
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_TRUE(app.root()("big.result").data().empty());
  // Killing the waiting process cancels its job: the scheduler ends up with no task running.
  const int waiting =
      run(std::string("(segment.seeded \"doc.big\" \"doc.big_seeds\" ") + params_dict + ")");
  for (int i = 0; i < 10; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_FALSE(finished(waiting)) << exit_error(waiting);
  app.exec_scheduler().kill(waiting);
  EXPECT_TRUE(pump_until([&] { return finished(waiting); }));
}

// ---- segment.critical-points: the paper's seeds -----------------------------------------------

TEST_P(SegmentationIntrinsics, CriticalPointsInTheThreeForms) {
  const cvc::point_classes want = cvc::critical_point_seeds(*vol);
  ASSERT_EQ(want.classes.size(), 2u);
  // The waiting form: a point_classes data object, feature then background.
  const int pid =
      run("(let ((pc (segment.critical-points \"doc.volume\")))"
          "  (state-set \"r.names\" (str (get-attr (point-classes.info pc) \"classes\")))"
          "  (state-set \"r.feature\" (length (point-classes.points pc \"feature\")))"
          "  (state-data-set \"r.seeds\" pc))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_NE(key("r.names").find("feature"), std::string::npos) << key("r.names");
  EXPECT_NE(key("r.names").find("background"), std::string::npos) << key("r.names");
  EXPECT_EQ(key("r.feature"), std::to_string(want.classes[0].size()));
  const auto got = object_at<cvc::point_classes>("r.seeds");
  ASSERT_TRUE(got);
  EXPECT_TRUE(*got == want);
  // The async form, with parameters (the §4 threshold moving every maximum to the background).
  const int apid =
      run("(let ((f (segment.critical-points-async \"doc.volume\" "
          "             (dict \"cp_feature_threshold\" 100.0))))"
          "  (state-set \"r.between\" \"ran\")"
          "  (state-set \"r.f2\" (length (point-classes.points (await f) \"feature\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(apid); }));
  EXPECT_EQ(exit_error(apid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.f2"), "0");
  // The start form publishes the node keys and lands the result at target.result.
  const int caller = run("(state-set \"r.cpid\" (segment.critical-points-start \"doc.volume\" nil "
                         "\"cpjob\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_TRUE(pump_until([&] { return !app.root()("cpjob.result").data().empty(); }))
      << "status: " << key("cpjob.status");
  // A bad parameter raises at the call, naming it.
  const int bad = run("(segment.critical-points \"doc.volume\" (dict \"cp_k\" -1))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("cp_k"), std::string::npos) << exit_error(bad);
}

TEST_P(SegmentationIntrinsics, SegmentSeededWithCriticalPointsNeedsNoClasses) {
  cvc::seeded_segmentation_params p = test_params();
  p.automatic = cvc::seed_mode::critical_points;
  const cvc::seeded_segmentation want = cvc::segment_seeded(*vol, cvc::point_classes{}, p);
  const int pid =
      run(std::string("(let ((s (segment.seeded \"doc.volume\" "
                      "             (point-classes.remove nil \"x\" 0 0 0) "
                      "             (dict \"t_low\" 0.1 \"t_high\" 10.0 \"gamma\" "
                      "                   -0.02 \"automatic\" \"critical-points\"))))") +
          "  (state-set \"r.names\" (str (get-attr s \"class_names\")))"
          "  (state-set \"r.feature\" (nth (get-attr s \"class_voxels\") 0)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_NE(key("r.names").find("background"), std::string::npos) << key("r.names");
  EXPECT_EQ(key("r.feature"), std::to_string(want.class_voxels.at(0)));
}

INSTANTIATE_TEST_SUITE_P(Modes, SegmentationIntrinsics,
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

TEST(SegmentationIntrinsicsAriadne, AnActionStartsItUnderTheDefaultLimits) {
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::install_segmentation_intrinsics(app);
  // Big enough to span many frames and several of a threaded job's chunks (about 250 ms each),
  // each ending in a progress note.
  app.root()("doc.volume").data(boost::any(bumps(app, 160)));
  app.root()("doc.seeds").data(boost::any(two_seeds(160)));
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_time").value().empty());
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_steps").value().empty());

  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button(
      "Go",
      std::string("(state-set \"r.pid\" (segment.seeded-start \"doc.volume\" \"doc.seeds\" ") +
          params_dict + " \"job\"))")}));
  be.click = "Go";
  std::set<std::string> progress_seen;
  std::size_t frames = 0, frames_busy = 0;
  const auto t0 = steady::now();
  // 93-101 s in a Debug job, three times that on a loaded runner: ctest gives the suite 900 s too.
  while (std::chrono::duration<double>(steady::now() - t0).count() < 900.0) {
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
  EXPECT_GT(frames_busy, 2u) << "the frames kept coming while the job ran";
  EXPECT_GE(progress_seen.size(), 2u) << "progress rose while the frame loop ran";
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result landed at job.result, status "
                        << app.root()("job.status").value();
  // Every voxel passes t_low and the fronts cross the volume: the two classes label it all.
  const se::value_t *counts = se::field(*v, "class_voxels");
  ASSERT_NE(counts, nullptr);
  const auto &list = *std::get<se::list_ptr>(counts->v);
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(std::get<std::int64_t>(list[0].v) + std::get<std::int64_t>(list[1].v),
            std::int64_t(160) * 160 * 160);
  std::cout << "[ T10 ] segment.seeded-start from an action: " << frames << " frames, "
            << frames_busy << " while busy, " << progress_seen.size() << " progress values\n";
}

} // namespace

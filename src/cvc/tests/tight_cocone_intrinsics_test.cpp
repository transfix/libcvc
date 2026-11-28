/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cocone.tight (cvc/geometry/tight_cocone_intrinsics.h), the TightCocone port in the DSL, and its
// component (cvc://components/mesh/tight_cocone.ari): the VolumeRover ports' T10 and T13 rows
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §1.7). The three forms on the app's
// scheduler, cooperative and threaded; a bad parameter raising at the call; job.cancel and a kill
// part-way through; params.defaults; installed on one app alone (T6); an Ariadne action starting it
// with -start under the default limits, its progress keys rising while the frame loop runs; the
// component loading clean; and the component run headless over a dataset the app made, its Run
// button clicked and the surface put back in the dataset. Runs natively and under node (the ports
// lane).

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tet_complex.h>
#include <cvc/geometry/tight_cocone.h>
#include <cvc/geometry/tight_cocone_intrinsics.h>
#include <cvc/geometry/tight_cocone_params.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

namespace {

namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

// n points on a sphere of radius r (a Fibonacci spiral, a little off its lattice).
cvc::geometry sphere(int n, double r = 1.0) {
  cvc::geometry g;
  const double golden = pi * (3.0 - std::sqrt(5.0));
  for (int i = 0; i < n; ++i) {
    const double z = 1.0 - 2.0 * (i + 0.5) / n, s = std::sqrt(1.0 - z * z);
    const double a = golden * i + 0.1 * std::sin(7.0 * i);
    g.points().push_back(cvc::geometry::point_t{{r * s * std::cos(a), r * s * std::sin(a), r * z}});
  }
  return g;
}

class TightCoconeIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
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
    cvc::register_job_op(env, ictx, app, cvc::tight_cocone_op(GetParam()));
    cvc::register_job_intrinsics(env, app);
    geom = sphere(600);
    app.root()("doc.points").data(boost::any(geom));
    expected = cvc::tight_cocone(geom);
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
  cvc::tight_cocone_result expected;
};

TEST_P(TightCoconeIntrinsics, TheWaitingFormReturnsTheSurface) {
  const int pid = run("(let ((s (cocone.tight \"doc.points\")))"
                      "  (state-set \"r.triangles\" (get-attr s \"triangles\"))"
                      "  (state-set \"r.points\" (get-attr s \"surface_points\"))"
                      "  (state-set \"r.inside\" (get-attr s \"inside_cells\"))"
                      "  (state-data-set \"r.surface\" (get-attr s \"surface\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.triangles"), std::to_string(expected.surface.num_tris()));
  EXPECT_EQ(key("r.points"), std::to_string(expected.surface.num_points()));
  EXPECT_EQ(key("r.inside"), std::to_string(expected.inside_cells));
  const boost::any a = app.root()("r.surface").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&a);
  ASSERT_NE(v, nullptr);
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v);
  ASSERT_TRUE(obj && *obj);
  EXPECT_EQ((*obj)->type_name, "cvc::geometry");
  // As a dataset holds a geometry, so data-put and the scene take it as it is.
  const auto *surface = boost::any_cast<std::shared_ptr<const cvc::geometry>>(&(*obj)->payload);
  ASSERT_TRUE(surface && *surface);
  EXPECT_EQ((*surface)->const_tris(), expected.surface.const_tris());
  EXPECT_EQ((*surface)->const_points(), expected.surface.const_points());
}

TEST_P(TightCoconeIntrinsics, TheAsyncFormAndParameters) {
  cvc::tight_cocone_params p;
  p.cocone_phi = 0.3;
  const cvc::tight_cocone_result narrow = cvc::tight_cocone(geom, p);
  const int pid = run("(let ((f (cocone.tight-async \"doc.points\" (dict \"cocone_phi\" 0.3))))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (state-set \"r.triangles\" (get-attr (await f) \"triangles\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.triangles"), std::to_string(narrow.surface.num_tris()));
  // Parameters as the text a state key holds read the same (what the component passes).
  const int text = run("(state-set \"r.text\" (get-attr (cocone.tight \"doc.points\""
                       "  (dict \"robust\" \"0\" \"cocone_phi\" \"0.3\")) \"triangles\"))");
  ASSERT_TRUE(pump_until([&] { return finished(text); }));
  EXPECT_EQ(exit_error(text), "");
  EXPECT_EQ(key("r.text"), std::to_string(narrow.surface.num_tris()));
  // A bad parameter raises at the call, naming it.
  const int bad = run("(cocone.tight \"doc.points\" (dict \"cocone_phi\" 5))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("cocone_phi"), std::string::npos) << exit_error(bad);
  // The table, through params.defaults.
  const int defaults = run(
      "(state-set \"r.robust\" (if (get-attr (params.defaults \"cocone.tight\") \"robust\") 1 0))");
  ASSERT_TRUE(pump_until([&] { return finished(defaults); }));
  EXPECT_EQ(key("r.robust"), "0");
}

TEST_P(TightCoconeIntrinsics, TheStartFormPublishesTheKeysAndTheResult) {
  const int caller = run("(state-set \"r.pid\" (cocone.tight-start \"doc.points\" \"job\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0)) << "-start returns at once";
  EXPECT_EQ(exit_error(caller), "");
  ASSERT_FALSE(key("r.pid").empty());
  ASSERT_TRUE(pump_until([&] { return key("job.status").rfind("done", 0) == 0; }))
      << "status: " << key("job.status") << ", busy " << key("job.busy");
  EXPECT_EQ(key("job.busy"), "0");
  EXPECT_EQ(key("job.pid"), key("r.pid"));
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result lands at job.result";
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "triangles")->v),
            std::int64_t(expected.surface.num_tris()));
}

TEST_P(TightCoconeIntrinsics, JobCancelAndKillStopIt) {
  // A bigger cloud, so the job is still running when the cancel arrives.
  app.root()("doc.big").data(boost::any(sphere(20000)));
  const int caller = run("(state-set \"r.pid\" (cocone.tight-start \"doc.big\" \"big\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  EXPECT_EQ(key("r.cancelled"), "true");
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_TRUE(app.root()("big.result").data().empty());
  // Killing the waiting process cancels its job: the scheduler ends up with no task running.
  const int waiting = run("(cocone.tight \"doc.big\")");
  for (int i = 0; i < 10; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_FALSE(finished(waiting));
  app.exec_scheduler().kill(waiting);
  EXPECT_TRUE(pump_until([&] { return finished(waiting); }));
}

INSTANTIATE_TEST_SUITE_P(Modes, TightCoconeIntrinsics,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded),
                         [](const auto &info) {
                           return info.param == cvc::sliced_threading::cooperative ? "cooperative"
                                                                                   : "threaded";
                         });

// ---- T6: installed on one app alone ---------------------------------------------------------

TEST(TightCoconeIntrinsicsApps, InstalledOnOneAppAlone) {
  cvc::app a, b;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } ga{a}, gb{b};
  cvc::install_tight_cocone_intrinsics(a);
  const std::string script = "(state-set \"r.phi\" (get-attr (params.defaults \"cocone.tight\") "
                             "\"cocone_phi\"))";
  std::vector<std::string> errors_a, errors_b;
  EXPECT_TRUE(cvc::ariadne::run_init(a, "", script, &errors_a))
      << (errors_a.empty() ? "" : errors_a.front());
  EXPECT_NE(a.root()("r.phi").value().rfind("0.3926990816", 0), std::string::npos)
      << a.root()("r.phi").value();
  EXPECT_FALSE(cvc::ariadne::run_init(b, "", script, &errors_b))
      << "an app that did not install it has no cocone.tight";
  EXPECT_TRUE(b.root()("r.phi").value().empty());
}

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

TEST(TightCoconeIntrinsicsAriadne, AnActionStartsItUnderTheDefaultLimits) {
  if (!cvc::tet_complex_available())
    GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::install_tight_cocone_intrinsics(app);
  // Big enough to span many frames and several 100 ms progress notes.
  const cvc::geometry pts = sphere(8000, 10.0);
  app.root()("doc.points").data(boost::any(pts));
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_time").value().empty());
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_steps").value().empty());

  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button(
      "Go", "(state-set \"r.pid\" (cocone.tight-start \"doc.points\" \"job\"))")}));
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
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "triangles")->v),
            std::int64_t(cvc::tight_cocone(pts).surface.num_tris()));
  std::cout << "[ T10 ] cocone.tight-start from an action: " << frames << " frames, "
            << frames_busy << " while busy, " << progress_seen.size() << " progress values\n";
}

// ---- the component --------------------------------------------------------------------------

namespace fs = std::filesystem;

fs::path ariadne_dir() { return fs::path(CVC_REPO_DIR) / "src" / "cvc" / "ariadne"; }

bool have_yaml() {
  cvc::app probe;
  return cvc::ariadne::load_string(probe, "meta: { min_libcvc: \"0.0.0\" }\n").ok;
}

// Until CC1's recursive ShippedComponentsLoadClean covers the mesh/ collection, its own check:
// the unit is a window and loads with no warning, from this tree's copy.
TEST(TightCoconeComponent, LoadsClean) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const fs::path file = ariadne_dir() / "components" / "mesh" / "tight_cocone.ari";
  if (!fs::is_regular_file(file))
    GTEST_SKIP() << "component sources not found at " << file.string();
  cvc::app app;
  cvc::ariadne::register_cvc_uri_handler(app, {ariadne_dir().string()});
  const cvc::ariadne::UriResult src =
      cvc::ariadne::resolve(app, "cvc://components/mesh/tight_cocone.ari");
  std::error_code ec;
  ASSERT_TRUE(src.ok) << src.error;
  EXPECT_TRUE(fs::equivalent(fs::path(src.canonical), file, ec)) << src.canonical;
  for (const char *args : {"", "\n    args: { source: scan, item: geometry, result: scan, job: "
                               "cocone.scan, title: Scan surface, id: scan_surface }"}) {
    const cvc::ariadne::LoadResult r = cvc::ariadne::load_string(
        app, std::string("meta: { min_libcvc: \"0.0.0\" }\n"
                         "import: cvc://components/mesh/tight_cocone.ari\n"
                         "windows:\n  - include: tight_cocone") +
                 args + "\n");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(r.schema_issues.empty()) << r.schema_issues.front().text();
    for (const std::string &w : r.warnings)
      ADD_FAILURE() << w;
    ASSERT_FALSE(r.root.children.empty());
    EXPECT_EQ(r.root.children.front().kind, cvc::ariadne::Kind::Window);
  }
}

// The component headless (an AppRuntime and a backend that clicks), over a dataset the app made:
// the points come from the state tree, Run starts the job, and on_tick's tight-cocone-step puts
// the surface in the dataset as its output `surface` and the counts under the job's info.*.
TEST(TightCoconeComponent, RunsOnTheAppsPointsAndPutsTheSurfaceBack) {
  if (!cvc::tet_complex_available())
    GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const fs::path doc = fs::path(CVC_REPO_DIR) / "src" / "cvc" / "tests" / "data" / "tight_cocone" /
                       "component_run.ari";
  if (!fs::is_regular_file(doc))
    GTEST_SKIP() << "test document not found at " << doc.string();
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::install_tight_cocone_intrinsics(app);
  cvc::ariadne::register_data_generator(
      app, "tight_cocone_test_sphere", [](cvc::ariadne::DataDecode &d) {
        d.out("geometry").value(std::string("geometry"));
        d.out("geometry").data(boost::any(std::shared_ptr<const cvc::geometry>(
            std::make_shared<const cvc::geometry>(sphere(500)))));
        return std::vector<cvc::archive::file>();
      });
  const cvc::tight_cocone_result expected = cvc::tight_cocone(sphere(500));

  ClickBackend be;
  cvc::ariadne::AppRuntime art(app, "ui.tc");
  art.set_backend(be);
  art.add_component_path(ariadne_dir().string());
  const std::vector<std::string> warnings = art.load(doc.string());
  for (const std::string &w : warnings)
    ADD_FAILURE() << "load: " << w;
  auto at = [&](const std::string &k) { return app.root()("ui.tc." + k).value(); };
  // The dataset's own node (data.points is a link to it: reading through the link with
  // operator() would make a node in its place).
  auto dataset = [&](const std::string &k) {
    const std::string store = art.data_store()->resolve("ui.tc.data.points");
    cvc::state *s =
        store.empty() ? nullptr : app.root().findDescendant(store + cvc::state::SEPARATOR + k);
    return s ? s->value() : std::string();
  };
  const auto t0 = steady::now();
  bool clicked = false;
  while (std::chrono::duration<double>(steady::now() - t0).count() < 120.0) {
    if (!clicked && dataset("status") == "ready") {
      be.click = "Run";
      clicked = true;
    }
    art.runtime().render();
    art.drain();
    if (!at("tc.info.triangles").empty())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  for (const std::string &w : art.take_warnings())
    ADD_FAILURE() << "runtime: " << w;
  ASSERT_TRUE(clicked) << "the points never got ready: '" << dataset("status") << "' "
                       << dataset("error");
  EXPECT_EQ(at("tc.status").rfind("done", 0), 0u) << at("tc.status");
  // The window's keys hold the table's defaults, exactly (its widgets' def: values are the
  // table's, so a widget drawn before on_tick seeds them writes the same).
  {
    const auto seeded =
        cvc::params_from_state<cvc::tight_cocone_params>(app.root()("ui.tc.tc.params"));
    ASSERT_TRUE(seeded.ok()) << seeded.message();
    const cvc::tight_cocone_params defaults;
    cvc::for_each_param<cvc::tight_cocone_params>([&](const auto &f) {
      EXPECT_EQ(seeded.value.*(f.member), defaults.*(f.member))
          << f.name << " = " << at(std::string("tc.params.") + f.name);
    });
  }
  EXPECT_EQ(at("tc.info.triangles"), std::to_string(expected.surface.num_tris()));
  EXPECT_EQ(at("tc.info.surface_points"), std::to_string(expected.surface.num_points()));
  EXPECT_EQ(at("tc.info.points"), "500");
  EXPECT_EQ(at("tc.info.warnings"), "");
  // The surface is on the tree, in the dataset, where a `source: { data: points, item: surface }`
  // node draws it; the points the component read are still there beside it.
  const std::string store = art.data_store()->resolve("ui.tc.data.points");
  ASSERT_FALSE(store.empty());
  cvc::state *out = app.root().findDescendant(store + cvc::state::SEPARATOR + "surface");
  ASSERT_NE(out, nullptr);
  const boost::any put = out->data();
  const auto *surface = boost::any_cast<std::shared_ptr<const cvc::geometry>>(&put);
  ASSERT_TRUE(surface && *surface);
  EXPECT_EQ((*surface)->const_tris(), expected.surface.const_tris());
  cvc::state *in = app.root().findDescendant(store + cvc::state::SEPARATOR + "geometry");
  ASSERT_NE(in, nullptr);
  EXPECT_FALSE(in->data().empty());
}

} // namespace

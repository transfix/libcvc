/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// grid-contours.* (cvc/volume/volume_grid_contour_intrinsics.h), a volume grid's contours in the
// DSL: the VolumeRover ports' T10 rows for P10 (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.6, §1.7). grid-contours.tile's three forms on the app's scheduler, cooperative and threaded,
// with job.cancel part-way through; info, stack, from-stack and add-curve; params.defaults; bad
// arguments raising at the call; and an Ariadne document starting a tile with -start from an
// action under the default limits. Runs natively and under node (the ports lane).

#include <chrono>
#include <cmath>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid_contour_intrinsics.h>
#include <cvc/volume/volume_grid_contours.h>
#include <gtest/gtest.h>
#include <memory>
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

double q8(double v) { return std::round(v * 256) / 256; }

cvc::grid_contours tube(int slices, int n) {
  cvc::grid_contours c;
  c.frame.dims = {64, 64, 64};
  cvc::grid_contour_object o;
  o.name = "tube";
  for (int k = 0; k < slices; ++k) {
    cvc::grid_curve cv;
    cv.index = 4 + k;
    for (int i = 0; i < n; ++i) {
      const double t = 2 * pi * i / n;
      cv.ru.push_back(q8(20 + 6 * std::cos(t) * (1 + 0.05 * std::cos(3 * t + k))));
      cv.ru.push_back(q8(24 + 5 * std::sin(t)));
    }
    cv.closed = true;
    o.curves.push_back(std::move(cv));
  }
  c.objects.push_back(std::move(o));
  return c;
}

const cvc::geometry *geometry_in(const se::value_t &v) {
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v.v);
  if (!obj || !*obj || (*obj)->type_name != "cvc::geometry")
    return nullptr;
  return boost::any_cast<cvc::geometry>(&(*obj)->payload);
}

template <class T> std::shared_ptr<const T> payload_of(const boost::any &a) {
  const se::value_t *v = boost::any_cast<se::value_t>(&a);
  if (!v)
    return nullptr;
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v);
  if (!obj || !*obj)
    return nullptr;
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&(*obj)->payload))
    return *p;
  return nullptr;
}

class GridContourIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
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
    cvc::register_grid_contour_intrinsics(env, ictx, app, GetParam());
    curves = tube(6, 24);
    app.root()("doc.grid.curves")
        .data(boost::any(std::make_shared<const cvc::grid_contours>(curves)));
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
  bool ran(const std::string &program) {
    const int pid = run(program);
    return pump_until([&] { return finished(pid); }) && exit_error(pid).empty();
  }
  std::string ran_error(const std::string &program) {
    const int pid = run(program);
    pump_until([&] { return finished(pid); });
    return exit_error(pid);
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
  cvc::grid_contours curves;
  boost::any stored;
};

TEST_P(GridContourIntrinsics, TheWaitingFormReturnsTheSurfaces) {
  SKIP_WITHOUT_PLANAR();
  ASSERT_TRUE(ran("(let ((r (grid-contours.tile \"doc.grid.curves\")))"
                  "  (state-set \"r.warnings\" (length (get-attr r \"warnings\")))"
                  "  (state-data-set \"r.tube\" (get-attr (get-attr r \"surfaces\") \"tube\")))"));
  EXPECT_EQ(key("r.warnings"), "0");
  stored = app.root()("r.tube").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&stored);
  ASSERT_NE(v, nullptr);
  const cvc::geometry *g = geometry_in(*v);
  ASSERT_NE(g, nullptr) << "the surface is a cvc::geometry data object";
  EXPECT_EQ(g->num_tris(), cvc::tile_grid_contours(curves).surfaces.at("tube").num_tris());
  // A bad parameter, and something that is not curves, raise naming them.
  EXPECT_NE(ran_error("(grid-contours.tile \"doc.grid.curves\" (dict \"curation_delta\" -1))")
                .find("curation_delta"),
            std::string::npos);
  EXPECT_NE(ran_error("(grid-contours.tile \"doc.nothing\")").find("grid-contours.tile"),
            std::string::npos);
  ASSERT_TRUE(ran("(state-set \"r.ce\" (get-attr (params.defaults \"grid-contours.tile\") "
                  "\"cap_ends\"))"
                  "(state-set \"r.ps\" (get-attr (params.defaults \"grid-contours.display\") "
                  "\"point_size\"))"
                  "(state-set \"r.sa\" (get-attr (params.defaults \"grid-contours.object\") "
                  "\"samples\"))"));
  EXPECT_EQ(key("r.ce"), "true");
  EXPECT_EQ(key("r.ps"), "0.6");
  EXPECT_EQ(key("r.sa"), "5");
}

TEST_P(GridContourIntrinsics, TheStartFormPublishesTheKeysAndJobCancelStopsIt) {
  SKIP_WITHOUT_PLANAR();
  ASSERT_TRUE(ran("(state-set \"r.pid\" (grid-contours.tile-start \"doc.grid.curves\" "
                  "\"doc.grid.tile\"))"));
  ASSERT_TRUE(pump_until([&] { return key("doc.grid.tile.status").rfind("tiled", 0) == 0; }))
      << "status: " << key("doc.grid.tile.status");
  EXPECT_EQ(key("doc.grid.tile.busy"), "0");
  EXPECT_EQ(key("doc.grid.tile.progress"), "100");
  EXPECT_NE(app.root()("doc.grid.tile.result").data().type(), typeid(void));
  // A big one, cancelled part-way.
  app.root()("doc.big").data(boost::any(std::make_shared<const cvc::grid_contours>(tube(60, 160))));
  ASSERT_TRUE(ran("(state-set \"r.pid\" (grid-contours.tile-start \"doc.big\" \"big\"))"));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  ASSERT_TRUE(ran("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))"));
  EXPECT_EQ(key("r.cancelled"), "true");
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_TRUE(app.root()("big.result").data().empty());
}

TEST_P(GridContourIntrinsics, InfoStackFromStackAndAddCurve) {
  ASSERT_TRUE(ran("(let ((i (grid-contours.info \"doc.grid.curves\")))"
                  "  (state-set \"r.curves\" (get-attr i \"curves\"))"
                  "  (state-set \"r.points\" (get-attr i \"points\"))"
                  "  (state-set \"r.name\" (get-attr (car (get-attr i \"objects\")) \"name\"))"
                  "  (state-set \"r.dims\" (car (get-attr (get-attr i \"frame\") \"dims\"))))"));
  EXPECT_EQ(key("r.curves"), "6");
  EXPECT_EQ(key("r.points"), "144");
  EXPECT_EQ(key("r.name"), "tube");
  EXPECT_EQ(key("r.dims"), "64");
  // The stack of the z planes, and back.
  ASSERT_TRUE(ran("(state-data-set \"r.stack\" (grid-contours.stack \"doc.grid.curves\"))"
                  "(state-data-set \"r.back\" (grid-contours.from-stack \"r.stack\" "
                  "\"doc.grid.curves\"))"
                  "(state-data-set \"r.side\" (grid-contours.stack \"doc.grid.curves\" \"x\"))"));
  const std::shared_ptr<const cvc::contour_stack> s =
      payload_of<cvc::contour_stack>(app.root()("r.stack").data());
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, cvc::grid_contours_stack(curves));
  const std::shared_ptr<const cvc::grid_contours> back =
      payload_of<cvc::grid_contours>(app.root()("r.back").data());
  ASSERT_TRUE(back);
  EXPECT_EQ(*back, curves);
  EXPECT_EQ(payload_of<cvc::contour_stack>(app.root()("r.side").data())->polygon_count(), 0u);
  // A frame from a volume.
  auto vol = std::make_shared<const cvc::volume>(app, cvc::dimension(8, 9, 10), cvc::UChar);
  app.root()("doc.vol").data(boost::any(vol));
  ASSERT_TRUE(ran("(state-data-set \"r.onvol\" (grid-contours.from-stack \"r.stack\" "
                  "\"doc.vol\"))"));
  EXPECT_EQ(payload_of<cvc::grid_contours>(app.root()("r.onvol").data())->frame.dims,
            (std::array<int, 3>{8, 9, 10}));
  // Curves built from nothing, a pair list and a flat list.
  ASSERT_TRUE(ran("(state-data-set \"r.drawn\""
                  "  (grid-contours.add-curve"
                  "    (grid-contours.add-curve nil \"axon\" \"z\" 3 (list (list 0 0) (list 4 0) "
                  "(list 2 3)))"
                  "    \"axon\" \"y\" 5 (list 1 1 2 2) #f))"));
  const std::shared_ptr<const cvc::grid_contours> drawn =
      payload_of<cvc::grid_contours>(app.root()("r.drawn").data());
  ASSERT_TRUE(drawn);
  ASSERT_EQ(drawn->objects.size(), 1u);
  ASSERT_EQ(drawn->objects[0].curves.size(), 2u);
  EXPECT_TRUE(drawn->objects[0].curves[0].closed);
  EXPECT_EQ(drawn->objects[0].curves[0].ru, (std::vector<double>{0, 0, 4, 0, 2, 3}));
  EXPECT_EQ(drawn->objects[0].curves[1].axis, cvc::slice_axis::y);
  EXPECT_FALSE(drawn->objects[0].curves[1].closed);
  // Bad arguments raise at the call.
  EXPECT_NE(ran_error("(grid-contours.add-curve nil \"a\" \"z\" 3 (list 0 0 1 1))")
                .find("fewer than three"),
            std::string::npos);
  EXPECT_NE(
      ran_error("(grid-contours.add-curve nil \"a\" \"w\" 3 (list 0 0 1 1 2 2))").find("axis 'w'"),
      std::string::npos);
  EXPECT_NE(ran_error("(grid-contours.add-curve nil \"a\" \"z\" 3 (list 0 0 1))").find("odd"),
            std::string::npos);
  EXPECT_NE(ran_error("(grid-contours.info)").find("TypeError"), std::string::npos);
  EXPECT_NE(ran_error("(grid-contours.stack \"doc.vol\")").find("grid-contours.stack"),
            std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(Modes, GridContourIntrinsics,
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

TEST(GridContourIntrinsicsAriadne, AnActionStartsATileUnderTheDefaultLimits) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  struct guard {
    cvc::app &app;
    ~guard() { cvc::ariadne::clear_action_intrinsics(app); }
  } g{app};
  cvc::install_grid_contour_intrinsics(app);
  const cvc::grid_contours c = tube(40, 128);
  app.root()("doc.grid.curves").data(boost::any(std::make_shared<const cvc::grid_contours>(c)));
  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button(
      "Tile",
      "(state-set \"r.pid\" (grid-contours.tile-start \"doc.grid.curves\" \"doc.grid.tile\"))")}));
  be.click = "Tile";
  std::size_t frames = 0, frames_busy = 0;
  const auto t0 = steady::now();
  while (std::chrono::duration<double>(steady::now() - t0).count() < 900.0) {
    rt.render();
    rt.drain();
    ++frames;
    const std::string busy = app.root()("doc.grid.tile.busy").value();
    frames_busy += busy == "1";
    if (!app.root()("r.pid").value().empty() && busy == "0")
      break;
  }
  ASSERT_FALSE(app.root()("r.pid").value().empty()) << "the action never ran its -start";
  EXPECT_EQ(app.root()("doc.grid.tile.status").value().rfind("tiled", 0), 0u)
      << app.root()("doc.grid.tile.status").value();
  EXPECT_GT(frames_busy, 2u) << "the frames kept coming while the job ran";
}

} // namespace

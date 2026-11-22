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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
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
  EXPECT_THROW(cvc::volume_filter_stepper(std::make_shared<const cvc::volume>(edged(app, 4, 4, 4)),
                                          p),
               std::invalid_argument);
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

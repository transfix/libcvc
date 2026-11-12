/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The generic intrinsic forms of a port's operation (cvc/core/job_intrinsics.h): (<op> ...),
// (<op>-async ...), (<op>-start ... target) and (job.cancel pid), the VolumeRover ports' T10 rows
// for the shared piece (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, §1.7). No port exists
// yet, so the op is the contour tree (contour_tree_builder as a result_job, its options' table),
// with fake steppers for failures and for jobs that never end by themselves. Each form runs
// cooperatively and threaded; a kill of the waiting process and of the task's own process, and a
// job.cancel part-way, each stop the job. One Ariadne document starts it with -start from a
// button's on: action under the default action limits, and the progress keys rise while the frames
// run. Runs natively and under node (the ports lane).

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/contour_tree_params.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_jobs.h>
#include <gtest/gtest.h>
#include <iostream>
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

// A stepper that counts for ever (or to `n`), counting itself alive, and throws at `throw_at`.
struct endless {
  std::shared_ptr<std::atomic<int>> live;
  std::uint64_t n = ~std::uint64_t(0), i = 0, throw_at = ~std::uint64_t(0);
  endless(std::shared_ptr<std::atomic<int>> l, std::uint64_t n_, std::uint64_t t)
      : live(std::move(l)), n(n_), throw_at(t) {
    ++*live;
  }
  ~endless() { --*live; }
  endless(const endless &) = delete;
  bool step(std::size_t budget) {
    const std::uint64_t end = budget == 0 ? n : std::min<std::uint64_t>(n, i + budget);
    for (; i < end; ++i)
      if (i == throw_at)
        throw std::invalid_argument("boom");
    return i >= n;
  }
  double progress() const { return n == ~std::uint64_t(0) ? 0.0 : double(i) / double(n); }
  const char *phase() const { return "counting"; }
  std::int64_t take() { return static_cast<std::int64_t>(i); }
};

std::vector<float> noise(std::size_t n, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> d(0.0f, 1.0f);
  std::vector<float> v(n);
  for (float &x : v)
    x = d(rng);
  return v;
}

// The ops under test, for an app: the contour tree, and the endless counter.
std::vector<cvc::job_op> test_ops(cvc::sliced_threading mode,
                                  std::shared_ptr<std::atomic<int>> live) {
  std::vector<cvc::job_op> ops;
  cvc::job_op tree = cvc::make_job_op<cvc::contour_tree_options, cvc::contour_tree>(
      "volume.contour-tree", 1,
      [](const cvc::job_call &c, const cvc::contour_tree_options &o) {
        return cvc::contour_tree_job(*c.input_as<cvc::volume>(0), o);
      },
      [](cvc::contour_tree &&t) {
        auto obj = std::make_shared<se::data_object>();
        const auto nodes = static_cast<std::int64_t>(t.nodes.size());
        const auto arcs = static_cast<std::int64_t>(t.arcs.size());
        obj->payload = std::make_shared<const cvc::contour_tree>(std::move(t));
        obj->type_name = "cvc::contour_tree";
        return se::make_dict({{"nodes", se::value_t(nodes)},
                              {"arcs", se::value_t(arcs)},
                              {"tree", se::value_t(obj)}});
      },
      [](const cvc::job_call &c, const cvc::contour_tree_options &) {
        const se::value_t &in = c.input(0);
        const std::string *p = std::get_if<std::string>(&in.v);
        return p ? *p : std::string("a volume");
      });
  tree.threading = mode;
  ops.push_back(tree);
  // (test.count n [throw_at]) as an op with no parameter table: its inputs are numbers, read in
  // make.
  cvc::job_op count;
  count.name = "test.count";
  count.inputs = 1;
  count.threading = mode;
  count.make = [live](const cvc::job_call &c) {
    const std::int64_t *n = std::get_if<std::int64_t>(&c.input(0).v);
    if (!n)
      throw std::invalid_argument(c.where(0) + " is a count");
    std::uint64_t throw_at = ~std::uint64_t(0);
    if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&c.params().v))
      for (const auto &kv : **d)
        if (kv.first == "throw_at")
          throw_at = static_cast<std::uint64_t>(std::get<std::int64_t>(kv.second.v));
    const std::uint64_t total = *n < 0 ? ~std::uint64_t(0) : static_cast<std::uint64_t>(*n);
    std::shared_ptr<cvc::result_job<std::int64_t>> job(
        cvc::make_result_job(std::make_unique<endless>(live, total, throw_at)));
    cvc::job_start js;
    js.job = job;
    js.result = [job] { return se::value_t(job->take_result()); };
    js.subject = "count";
    return js;
  };
  ops.push_back(count);
  return ops;
}

// Runs programs on the app's own scheduler with the ops registered, as a host would.
class JobIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
protected:
  void SetUp() override {
    live = std::make_shared<std::atomic<int>>(0);
    se::async_scheduler &sched = app.exec_scheduler();
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    for (const cvc::job_op &op : test_ops(GetParam(), live))
      cvc::register_job_op(env, ictx, app, op);
    cvc::register_job_intrinsics(env, app);
    const std::vector<float> data = noise(20 * 20 * 20, 5);
    vol = std::make_unique<cvc::volume>(app, reinterpret_cast<const unsigned char *>(data.data()),
                                        cvc::dimension(20, 20, 20), cvc::Float);
    app.root()("doc.vol").data(boost::any(*vol));
    expected = cvc::compute_contour_tree(*vol);
  }

  int run(const std::string &program, const std::string &owner = std::string()) {
    se::execute_options eo;
    eo.env = env;
    eo.owner = owner;
    return app.exec_scheduler().execute(program, eo);
  }
  // Pump frames until `pred` holds (or `seconds` pass).
  template <class Pred> bool pump_until(Pred pred, double seconds = 60.0) {
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
  std::shared_ptr<std::atomic<int>> live;
  std::unique_ptr<cvc::volume> vol;
  cvc::contour_tree expected;
};

TEST_P(JobIntrinsics, TheWaitingFormReturnsTheResult) {
  const int pid = run("(let ((r (volume.contour-tree \"doc.vol\")))"
                      "  (state-set \"r.nodes\" (get-attr r \"nodes\"))"
                      "  (state-set \"r.arcs\" (get-attr r \"arcs\"))"
                      "  (state-data-set \"r.tree\" (get-attr r \"tree\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); })) << "the waiting call never resumed";
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.nodes"), std::to_string(expected.nodes.size()));
  EXPECT_EQ(key("r.arcs"), std::to_string(expected.arcs.size()));
  // The result's data object, stored again by the program (as the DSL value holding it).
  const boost::any tree = app.root()("r.tree").data();
  const se::value_t *dv = boost::any_cast<se::value_t>(&tree);
  ASSERT_NE(dv, nullptr);
  const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&dv->v);
  ASSERT_TRUE(obj && *obj);
  EXPECT_EQ((*obj)->type_name, "cvc::contour_tree");
  const auto *t = boost::any_cast<std::shared_ptr<const cvc::contour_tree>>(&(*obj)->payload);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ((*t)->nodes.size(), expected.nodes.size());
  // An input may be a data object a program stored at a path, or the data object itself.
  const int again = run("(state-data-set \"doc.copy\" (state-data-get \"doc.vol\"))"
                        "(state-set \"r.copy\" (get-attr (volume.contour-tree \"doc.copy\") "
                        "                               \"nodes\"))"
                        "(state-set \"r.obj\" (get-attr (volume.contour-tree"
                        "                               (state-data-get \"doc.vol\")) \"nodes\"))");
  ASSERT_TRUE(pump_until([&] { return finished(again); }));
  EXPECT_EQ(exit_error(again), "");
  EXPECT_EQ(key("r.copy"), std::to_string(expected.nodes.size()));
  EXPECT_EQ(key("r.obj"), std::to_string(expected.nodes.size()));
}

TEST_P(JobIntrinsics, TheAsyncFormReturnsAFuture) {
  cvc::contour_tree_options o;
  o.max_dim = 8;
  const cvc::contour_tree coarse = cvc::compute_contour_tree(*vol, o);
  const int pid = run("(let ((f (volume.contour-tree-async \"doc.vol\" (dict \"max_dim\" 8))))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (state-set \"r.nodes\" (get-attr (await f) \"nodes\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.nodes"), std::to_string(coarse.nodes.size()));
}

TEST_P(JobIntrinsics, TheStartFormPublishesTheKeysAndTheResult) {
  const int caller = run("(state-set \"r.pid\" (volume.contour-tree-start \"doc.vol\" \"job\"))");
  // The call returns at once: the calling process ends in its first frame.
  ASSERT_TRUE(pump_until([&] { return finished(caller); }, 5.0));
  EXPECT_EQ(exit_error(caller), "");
  const std::string pid = key("r.pid");
  ASSERT_FALSE(pid.empty());
  EXPECT_EQ(key("job.pid"), pid);
  EXPECT_TRUE(key("job.busy") == "1" || key("job.status") == "done");
  ASSERT_TRUE(pump_until([&] { return key("job.busy") == "0"; }));
  EXPECT_EQ(key("job.status"), "done");
  EXPECT_EQ(key("job.progress"), "100");
  EXPECT_EQ(key("job.stoppable"), "0");
  EXPECT_EQ(key("job.phase"), "");
  EXPECT_FALSE(key("job.stats.work_ms").empty());
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result on the data channel";
  const se::value_t *nodes = se::field(*v, "nodes");
  ASSERT_NE(nodes, nullptr);
  EXPECT_EQ(std::get<std::int64_t>(nodes->v), static_cast<std::int64_t>(expected.nodes.size()));
  // A program reads it back.
  const int reader =
      run("(state-set \"r.read\" (get-attr (state-data-get \"job.result\") \"arcs\"))");
  ASSERT_TRUE(pump_until([&] { return finished(reader); }));
  EXPECT_EQ(key("r.read"), std::to_string(expected.arcs.size()));
}

TEST_P(JobIntrinsics, BadParametersAndInputsRaiseAtTheCall) {
  const int pid =
      run("(begin"
          "  (state-set \"r.params\" (try (volume.contour-tree \"doc.vol\" (dict \"max_dims\" 8))"
          "                             (catch (ValueError e) (get-attr e \"message\"))))"
          "  (state-set \"r.missing\" (try (volume.contour-tree \"doc.nothing\")"
          "                              (catch (ValueError e) (get-attr e \"message\"))))"
          "  (state-set \"r.arity\" (try (volume.contour-tree)"
          "                            (catch (TypeError e) (get-attr e \"message\"))))"
          "  (state-set \"r.start\" (try (volume.contour-tree-start \"doc.vol\" 5)"
          "                            (catch (TypeError e) (get-attr e \"message\")))))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_NE(key("r.params").find("unknown parameter 'max_dims'; valid: max_dim, quantize_u8"),
            std::string::npos)
      << key("r.params");
  EXPECT_NE(key("r.missing").find("no state node at 'doc.nothing'"), std::string::npos)
      << key("r.missing");
  EXPECT_NE(key("r.arity").find("takes 1 input and an optional dict of parameters"),
            std::string::npos)
      << key("r.arity");
  EXPECT_NE(key("r.start").find("the target path"), std::string::npos) << key("r.start");
}

TEST_P(JobIntrinsics, AFailureRaisesAtTheCallAndShowsInTheTarget) {
  const int pid = run("(begin"
                      "  (state-set \"r.err\" (try (test.count 100000 (dict \"throw_at\" 70000))"
                      "                          (catch (ValueError e) (get-attr e \"message\"))))"
                      "  (test.count-start 100000 (dict \"throw_at\" 50) \"bad\"))");
  ASSERT_TRUE(pump_until([&] { return finished(pid) && key("bad.busy") == "0"; }));
  EXPECT_EQ(key("r.err"), "boom");
  EXPECT_EQ(key("bad.status"), "failed: boom");
  EXPECT_TRUE(app.root()("bad.result").data().empty()) << "no result for a failed job";
}

TEST_P(JobIntrinsics, JobCancelStopsAStartedTask) {
  const int caller = run("(state-set \"r.pid\" (test.count-start -1 \"forever\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return live->load() == 1; })); // the stepper exists
  for (int i = 0; i < 5; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_EQ(key("forever.busy"), "1");
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))"
                            "(state-set \"r.unknown\" (job.cancel 999999))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  EXPECT_EQ(key("r.cancelled"), "true");
  EXPECT_EQ(key("r.unknown"), "false");
  ASSERT_TRUE(pump_until([&] { return key("forever.busy") == "0"; }));
  EXPECT_EQ(key("forever.status"), "cancelled");
  EXPECT_TRUE(app.root()("forever.result").data().empty());
  ASSERT_TRUE(pump_until([&] { return live->load() == 0; })) << "the stepper outlived the cancel";
}

TEST_P(JobIntrinsics, KillingTheWaitingProcessCancelsItsJob) {
  const int pid = run("(test.count -1)");
  ASSERT_TRUE(pump_until([&] { return live->load() == 1; }));
  for (int i = 0; i < 5; ++i)
    pump_until([] { return false; }, 0.002);
  EXPECT_FALSE(finished(pid)) << "it waits for a job that never ends";
  app.exec_scheduler().kill(pid);
  ASSERT_TRUE(pump_until([&] { return live->load() == 0; })) << "the job outlived its caller";
}

TEST_P(JobIntrinsics, KillingTheTasksProcessCancelsItsJob) {
  const int caller = run("(state-set \"r.pid\" (test.count-start -1 \"t\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return live->load() == 1; }));
  const int task = std::stoi(key("r.pid"));
  const auto info = app.exec_scheduler().get_process_info(task);
  ASSERT_TRUE(info.has_value());
  const std::string mode =
      GetParam() == cvc::sliced_threading::cooperative ? "cooperative" : "threaded";
  EXPECT_EQ(info->name, "test.count/" + mode + " count") << "how (ps) shows it";
  app.exec_scheduler().kill(task);
  ASSERT_TRUE(pump_until([&] { return live->load() == 0; }));
}

TEST_P(JobIntrinsics, ATaskCarriesItsCallersOwnerTag) {
  const int caller =
      run("(state-set \"r.pid\" (test.count-start -1 \"owned\"))", "doc:closing-test");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return live->load() == 1; }));
  // Closing the document kills what it owns, its task included.
  app.exec_scheduler().kill_owner("doc:closing-test");
  ASSERT_TRUE(pump_until([&] { return live->load() == 0; }));
}

TEST_P(JobIntrinsics, ParamsIntrinsicsSeeTheOpsTable) {
  const int pid = run("(state-set \"r.d\" (get-attr (params.defaults \"volume.contour-tree\")"
                      "                           \"quantize_u8\"))"
                      "(state-set \"r.c\" (params.check \"volume.contour-tree\""
                      "                                 (dict \"max_dim\" -1)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.d"), "false");
  EXPECT_NE(key("r.c").find("max_dim: -1 is out of range"), std::string::npos) << key("r.c");
}

INSTANTIATE_TEST_SUITE_P(Modes, JobIntrinsics,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded),
                         [](const ::testing::TestParamInfo<cvc::sliced_threading> &i) {
                           return i.param == cvc::sliced_threading::cooperative
                                      ? std::string("Cooperative")
                                      : std::string("Threaded");
                         });

// --- T10 in Ariadne: -start from an on: action under the default limits -----------------------

// A headless backend: draws nothing, and reports a click on `click` once.
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

// Registered on the app (its own providers since #1137); cleared on the way out.
struct ProviderGuard {
  cvc::app &app;
  ~ProviderGuard() { cvc::ariadne::clear_action_intrinsics(app); }
};

TEST(JobIntrinsicsAriadne, AnActionStartsAJobUnderTheDefaultLimits) {
  cvc::app app;
  ProviderGuard guard{app};
  auto live = std::make_shared<std::atomic<int>>(0);
  // Cooperative, as single-threaded wasm runs it: the job's slices share the frames with the UI.
  const std::vector<cvc::job_op> ops = test_ops(cvc::sliced_threading::cooperative, live);
  cvc::ariadne::register_action_intrinsics(
      app, [&app, ops](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        for (const cvc::job_op &op : ops)
          cvc::register_job_op(env, ictx, app, op);
        cvc::register_job_intrinsics(env, app);
      });
  // A volume big enough to span many frames and several 100 ms progress notes (some 0.3 s of work
  // natively, a few ms of it a frame).
  const std::vector<float> data = noise(96 * 96 * 96, 9);
  cvc::volume vol(app, reinterpret_cast<const unsigned char *>(data.data()),
                  cvc::dimension(96, 96, 96), cvc::Float);
  app.root()("doc.vol").data(boost::any(vol));
  // The action limits are left at their defaults (0.1 s, 5000 steps): nothing raises them.
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_time").value().empty());
  EXPECT_TRUE(app.root()("ariadne.limits.action_max_steps").value().empty());

  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group(
      {cvc::ariadne::button("Go", "(state-set \"r.pid\" (volume.contour-tree-start \"doc.vol\" "
                                  "\"job\"))")}));
  be.click = "Go";
  std::set<std::string> progress_seen;
  std::size_t frames = 0, frames_busy = 0;
  const auto t0 = steady::now();
  while (std::chrono::duration<double>(steady::now() - t0).count() < 120.0) {
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
  EXPECT_EQ(app.root()("job.status").value(), "done");
  EXPECT_GT(frames_busy, 2u) << "the frames kept coming while the job ran";
  EXPECT_GE(progress_seen.size(), 2u) << "progress rose while the frame loop ran";
  const boost::any r = app.root()("job.result").data();
  const se::value_t *v = boost::any_cast<se::value_t>(&r);
  ASSERT_NE(v, nullptr) << "the result landed at job.result";
  EXPECT_EQ(std::get<std::int64_t>(se::field(*v, "nodes")->v),
            static_cast<std::int64_t>(cvc::compute_contour_tree(vol).nodes.size()));
  std::cout << "[ T10 ] -start from an on: action: " << frames << " frames, " << frames_busy
            << " while busy, " << progress_seen.size() << " progress values\n";
}

} // namespace

// cvc::async_lane: the persistent FIFO worker lane, and the state_exec glue that posts lane results
// to channels a resident (chrooted) DSL process msg-recv's on, driven by pump_exec_frame.
// Synthetic workloads only.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <future>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;
using cvc::async_lane;
using cvc::lane_mode;
using namespace std::chrono_literals;

namespace {

// Poll `done` until it holds or `timeout` passes.
template <class F> bool wait_until(F done, std::chrono::milliseconds timeout = 5000ms) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline)
      return false;
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

se::value_t err_value(const std::string &e) { return se::value_t("err: " + e); }

// Aborts the test binary with `what` unless destroyed within `timeout`. Used where a regression
// would spin forever ON THE TEST THREAD, which no gtest assertion can interrupt, so it fails fast
// instead of hanging CI until the ctest timeout.
class hang_watchdog {
public:
  explicit hang_watchdog(std::string what, std::chrono::milliseconds timeout = 10000ms)
      : what_(std::move(what)) {
    thread_ = std::thread([this, timeout] {
      std::unique_lock<std::mutex> lk(m_);
      if (!cv_.wait_for(lk, timeout, [this] { return disarmed_; })) {
        std::fprintf(stderr, "hang_watchdog: %s\n", what_.c_str());
        std::abort();
      }
    });
  }
  ~hang_watchdog() {
    {
      std::lock_guard<std::mutex> lk(m_);
      disarmed_ = true;
    }
    cv_.notify_one();
    thread_.join();
  }
  hang_watchdog(const hang_watchdog &) = delete;
  hang_watchdog &operator=(const hang_watchdog &) = delete;

private:
  std::string what_;
  std::mutex m_;
  std::condition_variable cv_;
  bool disarmed_ = false;
  std::thread thread_;
};

} // namespace

// ---------------------------------------------------------------------------
// async_lane on its own
// ---------------------------------------------------------------------------

TEST(AsyncLane, RunsJobsInFifoOrderOnOnePersistentThread) {
  // Written only by the lane's jobs; read after stop(), whose join orders the two.
  std::vector<int> order;
  std::vector<std::thread::id> ids;
  async_lane lane("fifo");
  EXPECT_EQ(lane.name(), "fifo");
  EXPECT_EQ(lane.mode(), lane_mode::threaded);
  EXPECT_TRUE(lane.threaded());
  EXPECT_FALSE(lane.stopped());
  for (int i = 0; i < 500; ++i)
    ASSERT_TRUE(lane.submit([&order, &ids, i] {
      order.push_back(i);
      ids.push_back(std::this_thread::get_id());
    }));
  lane.stop();
  ASSERT_EQ(order.size(), 500u);
  for (int i = 0; i < 500; ++i)
    ASSERT_EQ(order[i], i) << "lane jobs ran out of submission order";
  for (const auto &id : ids)
    ASSERT_EQ(id, ids.front()) << "a lane runs every job on its one persistent thread";
  EXPECT_NE(ids.front(), std::this_thread::get_id());
  EXPECT_EQ(lane.completed(), 500u);
  EXPECT_EQ(lane.failed(), 0u);
  EXPECT_EQ(lane.in_flight(), 0u);
  EXPECT_EQ(lane.pending(), 0u);
}

TEST(AsyncLane, ConcurrentProducersKeepPerProducerOrder) {
  constexpr int kProducers = 4;
  constexpr int kPerProducer = 250;
  std::vector<std::pair<int, int>> seen; // lane thread only
  async_lane lane("mpsc");
  std::vector<std::thread> producers;
  for (int p = 0; p < kProducers; ++p)
    producers.emplace_back([&lane, &seen, p] {
      for (int i = 0; i < kPerProducer; ++i)
        lane.submit([&seen, p, i] { seen.emplace_back(p, i); });
    });
  for (auto &t : producers)
    t.join();
  lane.stop();
  ASSERT_EQ(seen.size(), static_cast<std::size_t>(kProducers * kPerProducer));
  std::vector<int> next(kProducers, 0);
  for (const auto &[p, i] : seen) {
    ASSERT_EQ(i, next[p]) << "producer " << p << " jobs reordered";
    ++next[p];
  }
}

TEST(AsyncLane, StopDrainsEveryQueuedJobThenJoinsAndRefusesNewOnes) {
  std::atomic<int> ran{0};
  std::promise<void> gate;
  std::shared_future<void> gate_open = gate.get_future().share();
  async_lane lane("drain");
  // The first job holds the worker, so the other 20 are still queued when stop() begins.
  ASSERT_TRUE(lane.submit([gate_open, &ran] {
    gate_open.wait();
    ++ran;
  }));
  for (int i = 0; i < 20; ++i)
    ASSERT_TRUE(lane.submit([&ran] { ++ran; }));
  EXPECT_EQ(lane.in_flight(), 21u);
  EXPECT_GE(lane.pending(), 20u);
  EXPECT_FALSE(lane.submit(async_lane::job{})) << "an empty job is refused";

  std::thread opener([&gate] {
    std::this_thread::sleep_for(20ms);
    gate.set_value();
  });
  lane.stop(); // returns only once all 21 have run and the worker has exited
  opener.join();
  EXPECT_EQ(ran.load(), 21);
  EXPECT_TRUE(lane.stopped());
  EXPECT_EQ(lane.in_flight(), 0u);
  EXPECT_EQ(lane.completed(), 21u);

  EXPECT_FALSE(lane.submit([&ran] { ++ran; })) << "submit after stop() must be refused";
  lane.stop(); // idempotent
  EXPECT_EQ(ran.load(), 21);
}

TEST(AsyncLane, DestructorDrainsAndJoins) {
  std::atomic<int> ran{0};
  {
    async_lane lane("dtor");
    for (int i = 0; i < 50; ++i)
      lane.submit([&ran] {
        std::this_thread::sleep_for(100us);
        ++ran;
      });
  }
  EXPECT_EQ(ran.load(), 50);
}

TEST(AsyncLane, ThrowingJobGoesToOnErrorAndTheLaneKeepsRunning) {
  std::mutex m;
  std::vector<std::string> errors;
  std::vector<std::thread::id> handler_threads;
  std::atomic<int> after{0};
  async_lane lane("err");
  lane.set_on_error([&](const std::string &msg) {
    std::lock_guard<std::mutex> lk(m);
    errors.push_back(msg);
    handler_threads.push_back(std::this_thread::get_id());
  });
  lane.submit([] { throw std::runtime_error("boom"); });
  lane.submit([] { throw 42; });
  lane.submit([&after] { ++after; });
  lane.stop();
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0], "boom");
  EXPECT_EQ(errors[1], "unknown error");
  for (const auto &id : handler_threads)
    EXPECT_NE(id, std::this_thread::get_id()) << "the handler runs on the lane's thread";
  EXPECT_EQ(after.load(), 1) << "a throwing job must not stop the lane";
  EXPECT_EQ(lane.failed(), 2u);
  EXPECT_EQ(lane.completed(), 3u);
  EXPECT_EQ(lane.last_error(), "unknown error");
}

TEST(AsyncLane, ThrowingOrMissingErrorHandlerIsContained) {
  std::atomic<int> after{0};
  async_lane lane("err2");
  EXPECT_EQ(lane.last_error(), "");
  lane.submit([] { throw std::runtime_error("no handler yet"); }); // recorded only
  lane.submit([&lane] {
    lane.set_on_error([](const std::string &) { throw std::logic_error("handler threw"); });
  });
  lane.submit([] { throw std::runtime_error("x"); });
  lane.submit([&after] { ++after; });
  lane.stop();
  EXPECT_EQ(after.load(), 1);
  EXPECT_EQ(lane.failed(), 2u);
  EXPECT_EQ(lane.last_error(), "x");
}

TEST(AsyncLane, LastJobMsTimesTheMostRecentJob) {
  async_lane lane("timing");
  lane.submit([] { std::this_thread::sleep_for(5ms); });
  lane.stop();
  EXPECT_GE(lane.last_job_ms(), 4.0);
}

TEST(AsyncLane, StopFromInsideOwnJobMarksStoppingWithoutSelfJoin) {
  std::atomic<int> drained{0};
  std::atomic<int> refused_ran{0};
  std::promise<void> gate;
  std::shared_future<void> gate_open = gate.get_future().share();
  std::promise<void> stopped_inside;
  std::future<void> stopped_inside_f = stopped_inside.get_future();
  async_lane lane("self");
  lane.submit([&, gate_open] {
    gate_open.wait();
    lane.stop(); // must not deadlock joining its own thread
    stopped_inside.set_value();
  });
  lane.submit([&drained] { ++drained; }); // queued before the stop: still drained
  gate.set_value();
  stopped_inside_f.wait();
  EXPECT_TRUE(lane.stopped());
  EXPECT_FALSE(lane.submit([&refused_ran] { ++refused_ran; }));
  lane.stop(); // from this thread: completes the join
  EXPECT_EQ(drained.load(), 1);
  EXPECT_EQ(refused_ran.load(), 0);
}

TEST(AsyncLane, DeferredModeRunsJobsOnThePumpThreadOneHopPerCall) {
  std::vector<int> order;
  std::vector<std::thread::id> ids;
  std::size_t nested = 99;
  std::vector<std::string> errors;
  async_lane lane("deferred", lane_mode::deferred);
  EXPECT_EQ(lane.mode(), lane_mode::deferred);
  EXPECT_FALSE(lane.threaded());
  lane.set_on_error([&errors](const std::string &e) { errors.push_back(e); });

  for (int i = 0; i < 3; ++i)
    lane.submit([&, i] {
      order.push_back(i);
      ids.push_back(std::this_thread::get_id());
      if (i == 0) // submitted mid-drain: runs on the NEXT call, not this one
        lane.submit([&order] { order.push_back(100); });
    });
  EXPECT_TRUE(order.empty()) << "a deferred lane runs nothing until the pump drains it";
  EXPECT_EQ(lane.pending(), 3u);
  EXPECT_EQ(lane.in_flight(), 3u);
  EXPECT_EQ(lane.run_deferred(), 3u);
  EXPECT_EQ(order, (std::vector<int>{0, 1, 2}));
  for (const auto &id : ids)
    EXPECT_EQ(id, std::this_thread::get_id());
  EXPECT_EQ(lane.pending(), 1u);
  EXPECT_EQ(lane.run_deferred(), 1u);
  EXPECT_EQ(order.back(), 100);

  // A per-call budget, a re-entrant drain (a no-op), and a throwing job on the pump thread.
  for (int i = 0; i < 4; ++i)
    lane.submit([] {});
  EXPECT_EQ(lane.run_deferred(2), 2u);
  EXPECT_EQ(lane.pending(), 2u);
  lane.submit([&] { nested = lane.run_deferred(); });
  lane.submit([] { throw std::runtime_error("pump-side throw"); });
  EXPECT_EQ(lane.run_deferred(), 4u);
  EXPECT_EQ(nested, 0u);
  EXPECT_EQ(errors, (std::vector<std::string>{"pump-side throw"}));
  EXPECT_EQ(lane.run_deferred(), 0u);

  // stop() from inside a deferred job only marks the lane; the jobs behind it still drain.
  int after_stop = 0;
  lane.submit([&lane] { lane.stop(); });
  lane.submit([&after_stop] { ++after_stop; });
  EXPECT_EQ(lane.run_deferred(), 2u);
  EXPECT_EQ(after_stop, 1);
  EXPECT_TRUE(lane.stopped());
  EXPECT_FALSE(lane.submit([] {}));
}

TEST(AsyncLane, DeferredStopRunsTheRestOnTheCaller) {
  int ran = 0;
  async_lane lane("deferred-stop", lane_mode::deferred);
  for (int i = 0; i < 5; ++i)
    lane.submit([&ran] { ++ran; });
  lane.stop();
  EXPECT_EQ(ran, 5);
  EXPECT_EQ(lane.in_flight(), 0u);
  async_lane threaded("threaded");
  EXPECT_EQ(threaded.run_deferred(), 0u) << "run_deferred is a no-op on a threaded lane";
}

// run_job releases a job's captures BEFORE the job counts as finished, so an owner that sees
// in_flight() == 0 knows the lane holds none of its objects any more. The capture's deleter runs on
// the thread that releases it and records the counters at that moment.
TEST(AsyncLane, JobCapturesAreReleasedBeforeTheJobCountsAsDone) {
  for (const lane_mode mode : {lane_mode::threaded, lane_mode::deferred}) {
    SCOPED_TRACE(mode == lane_mode::threaded ? "threaded" : "deferred");
    async_lane lane("release", mode);
    std::atomic<std::size_t> in_flight_at_release{99};
    std::atomic<std::uint64_t> completed_at_release{99};
    auto capture = std::shared_ptr<int>(new int(7), [&](int *p) {
      in_flight_at_release = lane.in_flight();
      completed_at_release = lane.completed();
      delete p;
    });
    const std::weak_ptr<int> watch = capture;
    ASSERT_TRUE(lane.submit([capture = std::move(capture)] { (void)*capture; }));
    if (mode == lane_mode::deferred)
      EXPECT_EQ(lane.run_deferred(), 1u);
    ASSERT_TRUE(wait_until([&] { return lane.in_flight() == 0; }));
    EXPECT_TRUE(watch.expired()) << "the lane still held the job's capture at in_flight() == 0";
    EXPECT_EQ(in_flight_at_release.load(), 1u) << "the capture outlived the job's in_flight count";
    EXPECT_EQ(completed_at_release.load(), 0u) << "the capture outlived the job's completion";
  }
}

// stop() on a deferred lane whose drain is further up THIS thread's stack -- B's job drains lane C,
// and C's job stops B -- must only mark B stopping, exactly like a stop() from B's own job. Waiting
// for B's drain to end would spin forever (this thread owns it) and hang a single-threaded wasm.
TEST(AsyncLane, StopFromANestedDrainOfAnotherLaneOnlyMarksTheLane) {
  hang_watchdog watchdog("a deferred stop() from a nested drain of another lane spun forever");
  std::vector<std::string> order;
  std::size_t b_reentered = 99;
  async_lane b("b", lane_mode::deferred);
  async_lane c("c", lane_mode::deferred);
  c.submit([&] {
    b_reentered = b.run_deferred(); // B is draining further up: a no-op, not a re-entry
    order.push_back("c: stop b");
    b.stop();
    order.push_back("c: stopped b");
  });
  b.submit([&] {
    order.push_back("b1");
    b.submit([&] { order.push_back("b-late"); }); // beyond this drain's budget: waits
    EXPECT_EQ(c.run_deferred(), 1u);
  });
  b.submit([&] { order.push_back("b2"); }); // queued before the stop: runs in this drain
  EXPECT_EQ(b.run_deferred(), 2u);
  EXPECT_EQ(order, (std::vector<std::string>{"b1", "c: stop b", "c: stopped b", "b2"}));
  EXPECT_EQ(b_reentered, 0u);
  EXPECT_EQ(c.failed(), 0u) << c.last_error();
  EXPECT_TRUE(b.stopped());
  EXPECT_FALSE(b.submit([] {}));
  EXPECT_EQ(b.pending(), 1u);
  b.stop(); // from outside any drain: runs the rest on the caller
  EXPECT_EQ(order.back(), "b-late");
  EXPECT_EQ(b.in_flight(), 0u);
}

// The threaded form of the same nesting: a threaded lane's job drains a deferred lane whose job
// stops the threaded lane. That stop() runs on the threaded lane's own worker, so it must only mark
// the lane; joining would throw resource_deadlock_would_occur into the deferred lane's job.
TEST(AsyncLane, StopFromAnotherLanesJobOnTheWorkerDoesNotSelfJoin) {
  std::promise<void> gate;
  std::shared_future<void> gate_open = gate.get_future().share();
  std::promise<void> drained;
  std::future<void> drained_f = drained.get_future();
  std::atomic<int> after{0};
  async_lane d("d", lane_mode::deferred);
  async_lane t("t"); // destroyed first: its job drains d
  d.submit([&t] { t.stop(); });
  ASSERT_TRUE(t.submit([&, gate_open] {
    gate_open.wait(); // hold the worker until the job below is queued too
    d.run_deferred();
    drained.set_value();
  }));
  ASSERT_TRUE(t.submit([&after] { ++after; })); // queued before the stop: still drained
  gate.set_value();
  drained_f.wait();
  EXPECT_TRUE(t.stopped());
  EXPECT_EQ(d.failed(), 0u) << d.last_error();
  t.stop(); // from this thread: completes the join
  EXPECT_EQ(after.load(), 1);
}

// ---------------------------------------------------------------------------
// state_exec integration
// ---------------------------------------------------------------------------

// A state_exec host wired the way a demo wires one: one async_scheduler pumped by THIS thread (the
// render/main thread) with pump_exec_frame, and processes whose intrinsics are chrooted under a
// document root.
class LaneTaskTest : public ::testing::Test {
protected:
  cvc::app app;
  se::async_scheduler sched;

  struct proc_ctx {
    se::memory_tracker tracker;
    se::intrinsics_context ictx;
    se::environment_ptr env;
  };
  // Must outlive the pids: the registered intrinsics capture &ictx.
  std::vector<std::unique_ptr<proc_ctx>> ctxs;

  // A fresh environment with the DSL intrinsics chrooted to `root` ("" = the whole tree).
  proc_ctx &make_ctx(const std::string &root) {
    auto c = std::make_unique<proc_ctx>();
    c->ictx.sched = &sched;
    c->ictx.tracker = &c->tracker;
    c->ictx.proc = se::make_process();
    se::apply_chroot(c->ictx, cvc::state::instance(app), root);
    c->env = se::builtins::make_default_environment();
    se::register_intrinsics(c->env, &c->ictx);
    ctxs.push_back(std::move(c));
    return *ctxs.back();
  }

  int spawn(proc_ctx &c, const std::string &script) {
    se::execute_options o;
    o.env = c.env;
    o.root_path = c.ictx.root_path;
    o.owner = "lane-test";
    return sched.execute(script, o);
  }

  se::process_status status(int pid) const {
    const auto info = sched.get_process_info(pid);
    return info ? info->status : se::process_status::killed;
  }

  // Pump frames (one pump_exec_frame each, ~1 ms apart) until `done` holds or `timeout` passes.
  template <class F>
  bool pump_until(F done, std::initializer_list<async_lane *> lanes = {},
                  std::chrono::milliseconds timeout = 5000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      cvc::pump_exec_frame(sched, lanes);
      if (done())
        return true;
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      std::this_thread::sleep_for(1ms);
    }
  }

  // Pop the single value a lane task posted to `key` (no receiver, so it waits in pending).
  std::optional<se::value_t> take(const std::string &key) {
    EXPECT_EQ(sched.pending_message_count(key), 1u) << "exactly one value per task on " << key;
    return sched.pop_pending_message(key);
  }

  void TearDown() override { sched.kill_owner("lane-test"); }
};

TEST_F(LaneTaskTest, ParkAndFutureResumeTheCallerWithTheLaneResult) {
  std::mutex m;
  std::vector<std::thread::id> work_threads;
  async_lane lane("work");
  proc_ctx &c = make_ctx("app");
  const std::string root = c.ictx.root_path;
  auto make_work = [&](std::int64_t n) {
    return [&, n] {
      {
        std::lock_guard<std::mutex> lk(m);
        work_threads.push_back(std::this_thread::get_id());
      }
      return se::value_t(n * 2);
    };
  };
  se::builtins::register_fn(
      c.env, "lane-double", [&, root](std::span<const se::value_t> args) -> se::value_t {
        return cvc::park_on_lane_task(lane, sched, root, "app",
                                      make_work(std::get<std::int64_t>(args[0].v)), err_value);
      });
  se::builtins::register_fn(
      c.env, "lane-double-async", [&, root](std::span<const se::value_t> args) -> se::value_t {
        return cvc::future_lane_task(lane, sched, root, "app",
                                     make_work(std::get<std::int64_t>(args[0].v)), err_value);
      });
  const int pid = spawn(c, "(+ (lane-double 21) (await (lane-double-async 10)))");
  ASSERT_TRUE(pump_until([&] { return status(pid) == se::process_status::terminated; }))
      << "the parked caller never resumed";
  const auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::int64_t>(result->v), 62); // 21*2 (transparent park) + 10*2 (future)
  std::lock_guard<std::mutex> lk(m);
  ASSERT_EQ(work_threads.size(), 2u);
  for (const auto &id : work_threads)
    EXPECT_NE(id, std::this_thread::get_id()) << "lane work must run off the pump thread";
}

TEST_F(LaneTaskTest, ProcessChrootedToAppReceivesCrossThreadHashPosts) {
  std::vector<std::int64_t> got;  // written by apply-built, which runs on the pump (this thread)
  std::vector<std::string> plain; // likewise, for the non-'#' channel
  async_lane geom("geom");
  async_lane sim("sim"); // destroyed first: its jobs submit to geom
  proc_ctx &c = make_ctx("app");
  proc_ctx &c2 = make_ctx("app");
  se::builtins::register_fn(c.env, "apply-built", [&got](std::span<const se::value_t> args) {
    got.push_back(std::get<std::int64_t>(args[0].v));
    return se::value_t();
  });
  se::builtins::register_fn(c2.env, "apply-plain", [&plain](std::span<const se::value_t> args) {
    plain.push_back(std::get<std::string>(args[0].v));
    return se::value_t();
  });
  const int pid = spawn(c, "(while t (apply-built (msg-recv \"app#built.tracks\")))");
  const int pid2 = spawn(c2, "(while t (apply-plain (msg-recv \"built\")))");
  sched.sync_run(1000, 1.0);
  ASSERT_EQ(status(pid), se::process_status::waiting);
  ASSERT_EQ(status(pid2), se::process_status::waiting);

  // The sim lane hands each generation straight to the geom lane (no main-thread hop), whose job
  // posts to a FIXED '#' channel; a '#' channel survives the "app" chroot verbatim.
  for (int gen = 1; gen <= 50; ++gen)
    ASSERT_TRUE(sim.submit([this, &geom, gen] {
      const std::string key = cvc::post_lane_task(
          geom, sched, "app", "app#built.tracks",
          [gen] { return se::value_t(static_cast<std::int64_t>(gen)); }, err_value);
      EXPECT_EQ(key, "app#built.tracks");
    }));
  // A non-'#' channel is scoped exactly as the chrooted (msg-recv "built") resolves it.
  const std::string scoped = cvc::post_lane_task(
      geom, sched, "app", "built", [] { return se::value_t("scoped"); }, err_value);
  EXPECT_EQ(scoped, "app.channels.built");

  ASSERT_TRUE(pump_until([&] { return got.size() == 50 && plain.size() == 1; }))
      << "got " << got.size() << " of 50 '#' posts, " << plain.size() << " of 1 scoped post";
  for (std::size_t i = 0; i < got.size(); ++i)
    EXPECT_EQ(got[i], static_cast<std::int64_t>(i + 1)) << "lane results reordered end to end";
  EXPECT_EQ(plain.front(), "scoped");
  // Still resident: parked for the next generation.
  EXPECT_EQ(status(pid), se::process_status::waiting);
  EXPECT_EQ(status(pid2), se::process_status::waiting);
}

TEST_F(LaneTaskTest, EveryTaskPostsExactlyOneValueOnEveryErrorPath) {
  async_lane lane("errs");
  const auto ok = [] { return se::value_t(static_cast<std::int64_t>(5)); };
  const auto throws_std = []() -> se::value_t { throw std::runtime_error("bad input"); };
  const auto throws_other = []() -> se::value_t { throw 7; };
  const auto throwing_on_error = [](const std::string &) -> se::value_t {
    throw std::logic_error("on_error threw");
  };
  const std::string k_ok = cvc::post_lane_task(lane, sched, "", "t#ok", ok, err_value);
  const std::string k_std = cvc::post_lane_task(lane, sched, "", "t#std", throws_std, err_value);
  const std::string k_other =
      cvc::post_lane_task(lane, sched, "", "t#other", throws_other, err_value);
  const std::string k_bad_handler =
      cvc::post_lane_task(lane, sched, "", "t#bad-handler", throws_std, throwing_on_error);
  const std::string k_no_handler =
      cvc::post_lane_task(lane, sched, "", "t#no-handler", throws_std, nullptr);
  lane.stop(); // every value is in the scheduler's ingress now
  const std::string k_stopped = cvc::post_lane_task(lane, sched, "", "t#stopped", ok, err_value);
  const std::string k_stopped_bare =
      cvc::post_lane_task(lane, sched, "", "t#stopped2", ok, nullptr);
  EXPECT_EQ(lane.failed(), 0u) << "task errors are handled inside the job, not by the lane";

  sched.drain_ingress(); // no receivers: each value lands in its channel's pending FIFO
  auto as_string = [](const std::optional<se::value_t> &v) {
    return v ? std::get<std::string>(v->v) : std::string("<none>");
  };
  const auto v_ok = take(k_ok);
  ASSERT_TRUE(v_ok.has_value());
  EXPECT_EQ(std::get<std::int64_t>(v_ok->v), 5);
  EXPECT_EQ(as_string(take(k_std)), "err: bad input");
  EXPECT_EQ(as_string(take(k_other)), "err: unknown error");
  EXPECT_EQ(as_string(take(k_bad_handler)), "async_lane 'errs': bad input");
  EXPECT_EQ(as_string(take(k_no_handler)), "async_lane 'errs': bad input");
  EXPECT_EQ(as_string(take(k_stopped)), "err: async_lane 'errs' is stopped");
  EXPECT_EQ(as_string(take(k_stopped_bare)), "async_lane 'errs' is stopped");
  EXPECT_EQ(sched.total_pending_messages(), 0u);
}

TEST_F(LaneTaskTest, LaunchReturnsAUniqueHashChannelPerTask) {
  async_lane lane("sim");
  const auto one = [] { return se::value_t(static_cast<std::int64_t>(1)); };
  const std::string a = cvc::launch_lane_task(lane, sched, "app", "app", one, err_value);
  const std::string b = cvc::launch_lane_task(lane, sched, "app", "app", one, err_value);
  EXPECT_NE(a, b);
  EXPECT_EQ(a.rfind("app#sim.", 0), 0u) << a;
  EXPECT_EQ(b.rfind("app#sim.", 0), 0u) << b;
  lane.stop();
  sched.drain_ingress();
  EXPECT_TRUE(take(a).has_value());
  EXPECT_TRUE(take(b).has_value());
}

// pump_exec_frame must re-ready (await ...) frame-yielders once per frame: a resident that ticks
// and then yields a frame advances exactly one tick per pump, and never without one.
TEST_F(LaneTaskTest, PumpAdvancesAnAwaitFrameYielderOncePerFrame) {
  int ticks = 0; // written by tick, which runs on the pump (this thread)
  proc_ctx &c = make_ctx("app");
  se::builtins::register_fn(c.env, "tick", [&ticks](std::span<const se::value_t>) {
    ++ticks;
    return se::value_t();
  });
  const int pid = spawn(c, "(while t (begin (tick) (await 0)))");
  for (int frame = 1; frame <= 5; ++frame) {
    // A generous time budget, so only the frame boundary limits the run.
    cvc::pump_exec_frame(sched, {}, 20000, 1.0);
    ASSERT_EQ(ticks, frame) << "the frame-yielder did not advance exactly once on frame " << frame;
    ASSERT_EQ(status(pid), se::process_status::waiting);
  }
  sched.sync_run(1000, 1.0); // no frame boundary: nothing advances
  EXPECT_EQ(ticks, 5);
}

// Regression (plan critique: "live sim stalls"): a resident (while t (sleep (msg-recv
// (sim-launch)))) is the ONLY process. Once it parks on the lane result nothing is runnable, so
// sync_run() alone never drains the lane's post, and after each (sleep) only step() wakes it.
// pump_exec_frame (drain_ingress + wake_awaiting + an unconditional sync_step + sync_run) must keep
// it cycling every frame.
TEST_F(LaneTaskTest, SleeperIsWokenByLaneCompletionWhenNothingElseIsRunnable) {
  std::atomic<int> batches{0};
  async_lane lane("sim");
  proc_ctx &c = make_ctx("app");
  const std::string root = c.ictx.root_path;
  se::builtins::register_fn(c.env, "sim-launch",
                            [&, root](std::span<const se::value_t>) -> se::value_t {
                              return se::value_t(cvc::launch_lane_task(
                                  lane, sched, root, "app",
                                  [&batches] {
                                    ++batches;
                                    return se::value_t(0.001); // seconds until the next batch
                                  },
                                  err_value));
                            });
  const int pid = spawn(c, "(while t (sleep (msg-recv (sim-launch))))");
  sched.sync_run(1000, 1.0); // runs until it parks on the first lane result
  ASSERT_EQ(status(pid), se::process_status::waiting);
  ASSERT_FALSE(sched.has_runnable()) << "precondition: the sleeper is the only process";
  ASSERT_TRUE(wait_until([&] { return batches.load() == 1 && lane.in_flight() == 0; }));

  ASSERT_TRUE(pump_until([&] { return batches.load() >= 10; }))
      << "the resident stalled after " << batches.load() << " lane batches";
  EXPECT_NE(status(pid), se::process_status::killed) << sched.get_exit_error(pid).value_or("");
}

// The same resident on a DEFERRED lane: the no-thread (single-threaded wasm) path, where the lane's
// work runs on the pump thread inside pump_exec_frame.
TEST_F(LaneTaskTest, DeferredLaneDrivesTheSameResidentFromThePump) {
  int batches = 0;
  std::thread::id work_thread;
  async_lane lane("sim", lane_mode::deferred);
  proc_ctx &c = make_ctx("app");
  const std::string root = c.ictx.root_path;
  se::builtins::register_fn(c.env, "sim-launch",
                            [&, root](std::span<const se::value_t>) -> se::value_t {
                              return se::value_t(cvc::launch_lane_task(
                                  lane, sched, root, "app",
                                  [&] {
                                    ++batches;
                                    work_thread = std::this_thread::get_id();
                                    return se::value_t(0.001);
                                  },
                                  err_value));
                            });
  const int pid = spawn(c, "(while t (sleep (msg-recv (sim-launch))))");
  sched.sync_run(1000, 1.0);
  ASSERT_EQ(status(pid), se::process_status::waiting);
  EXPECT_EQ(batches, 0) << "a deferred lane runs nothing until the pump";
  EXPECT_EQ(cvc::pump_exec_frame(sched, {&lane}), 1u);
  EXPECT_EQ(batches, 1);
  ASSERT_TRUE(pump_until([&] { return batches >= 10; }, {&lane}))
      << "the resident stalled after " << batches << " deferred batches";
  EXPECT_EQ(work_thread, std::this_thread::get_id());
}

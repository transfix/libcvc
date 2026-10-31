/// @file state_exec_async_calls_test.cpp
/// @brief Long-running native calls (docs/STATE_EXEC_EXCEPTIONS.md section 3): async functions,
/// the await_call frame, futures, errors at the call site, executors, cancellation, progress and
/// what a call is charged (AS-1..33), in both evaluators and on both schedulers where they apply.
/// Case IDs are the spec's (section 6.6). The Python cases (AS-4, AS-16, AS-32) are in
/// bindings/pycvc/test_pycvc_state_exec_async.py; the Ariadne reachability half of AS-17 is in
/// ariadne_runtime_test.cpp.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/core/async_task.h>
#include <cvc/core/thread_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <future>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;
using namespace std::chrono_literals;

namespace {

using clock_type = std::chrono::steady_clock;

double seconds_since(clock_type::time_point t) {
  return std::chrono::duration<double>(clock_type::now() - t).count();
}

enum engine : unsigned {
  recursive = 1,  // evaluator
  stackless = 2,  // stackless_evaluator
  sync_sched = 4, // scheduler: kernels run inline
  async_sched = 8 // async_scheduler: kernels on the app's background pool
};
constexpr unsigned kSchedulers = sync_sched | async_sched;
constexpr unsigned kAll = recursive | stackless | sync_sched | async_sched;

const char *engine_name(unsigned e) {
  switch (e) {
  case recursive:
    return "recursive";
  case stackless:
    return "stackless";
  case sync_sched:
    return "scheduler";
  default:
    return "async_scheduler";
  }
}

std::string show(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  return to_string(v);
}

int64_t int_arg(std::span<const value_t> args, std::size_t i, int64_t dflt) {
  if (i < args.size())
    if (const auto *n = std::get_if<int64_t>(&args[i].v))
      return *n;
  return dflt;
}

// What the kernels saw, for the cases that check it.
struct probe {
  std::atomic<bool> started{false};
  std::atomic<bool> cancelled_seen{false};
  std::atomic<int> ended{0};
  std::atomic<bool> late_finished{false};
  std::atomic<bool> a_done{false};
};

// Sleep `ms` in 1 ms naps, checking cancelled() (and so recording the CPU time) between them.
void nap(async_call &call, int64_t ms, probe &p) {
  p.started = true;
  const auto end = clock_type::now() + std::chrono::milliseconds(ms);
  while (clock_type::now() < end) {
    if (call.cancelled()) {
      p.cancelled_seen = true;
      throw std::runtime_error(call.function_name() + ": cancelled");
    }
    std::this_thread::sleep_for(1ms);
  }
}

// Use `ms` of this thread's CPU time, checking cancelled() as it goes when `call` is given.
void spin(async_call *call, int64_t ms, probe &p) {
  const double cpu0 = thread_cpu_seconds();
  volatile uint64_t sink = 0;
  while (thread_cpu_seconds() - cpu0 < 1e-3 * double(ms)) {
    for (int i = 0; i < 20000; ++i)
      sink = sink + uint64_t(i);
    if (call && call->cancelled()) {
      p.cancelled_seen = true;
      throw std::runtime_error(call->function_name() + ": cancelled");
    }
  }
}

// The test's async functions, on `host`.
void register_kernels(const environment_ptr &env, async_host &host, cvc::app &app, probe &p,
                      async_options opts = {}) {
  // (slow-op x [ms]): 2x after ms (default 50), checking cancelled().
  register_async_fn(
      env, host, "slow-op",
      [&p](std::span<const value_t> a) -> async_kernel {
        const int64_t x = int_arg(a, 0, 0), ms = int_arg(a, 1, 50);
        return [&p, x, ms](async_call &call) -> value_t {
          nap(call, ms, p);
          ++p.ended;
          return value_t(x * 2);
        };
      },
      opts);
  // (nap-blind ms): sleeps without checking cancelled(); returns ms.
  register_async_fn(
      env, host, "nap-blind",
      [&p](std::span<const value_t> a) -> async_kernel {
        const int64_t ms = int_arg(a, 0, 50);
        return [&p, ms](async_call &) -> value_t {
          std::this_thread::sleep_for(std::chrono::milliseconds(ms));
          p.late_finished = true;
          return value_t(ms);
        };
      },
      opts);
  // (busy ms): ms of CPU time, checking cancelled().
  register_async_fn(
      env, host, "busy",
      [&p](std::span<const value_t> a) -> async_kernel {
        const int64_t ms = int_arg(a, 0, 50);
        return [&p, ms](async_call &call) -> value_t {
          spin(&call, ms, p);
          return value_t(ms);
        };
      },
      opts);
  // (throw-invalid): a kernel throwing std::invalid_argument.
  register_async_fn(
      env, host, "throw-invalid",
      [](std::span<const value_t>) -> async_kernel {
        return [](async_call &) -> value_t { throw std::invalid_argument("bad argument"); };
      },
      opts);
  // (a ms): 1 after ms, then marks a_done; (b): throws ValueError at once.
  register_async_fn(
      env, host, "a",
      [&p](std::span<const value_t> args) -> async_kernel {
        const int64_t ms = int_arg(args, 0, 100);
        return [&p, ms](async_call &call) -> value_t {
          nap(call, ms, p);
          p.a_done = true;
          return value_t(1);
        };
      },
      opts);
  register_async_fn(
      env, host, "b",
      [](std::span<const value_t>) -> async_kernel {
        return [](async_call &) -> value_t { throw std::invalid_argument("b failed"); };
      },
      opts);
  // (progress-op n): posts 1..n to progress-op.progress, returns n.
  register_async_fn(
      env, host, "progress-op",
      [](std::span<const value_t> a) -> async_kernel {
        const int64_t n = int_arg(a, 0, 1);
        return [n](async_call &call) -> value_t {
          for (int64_t i = 1; i <= n; ++i)
            call.post(value_t(i));
          return value_t(n);
        };
      },
      opts);
  // (sum-later list ms): the sum of the list's numbers, after ms.
  register_async_fn(
      env, host, "sum-later",
      [&p](std::span<const value_t> a) -> async_kernel {
        value_t list = a.empty() ? value_t{} : a[0];
        const int64_t ms = int_arg(a, 1, 50);
        return [&p, list, ms](async_call &call) -> value_t {
          nap(call, ms, p);
          int64_t sum = 0;
          if (const auto *lp = std::get_if<list_ptr>(&list.v))
            for (const value_t &v : **lp)
              sum += std::get<int64_t>(v.v);
          return value_t(sum);
        };
      },
      opts);
  // (fanout n ms): n chunks on app.computePool(), each ms of CPU time.
  register_async_fn(
      env, host, "fanout",
      [&app, &p](std::span<const value_t> a) -> async_kernel {
        const int64_t n = int_arg(a, 0, 4), ms = int_arg(a, 1, 100);
        return [&app, &p, n, ms](async_call &) -> value_t {
          app.computePool().parallel_for(
              static_cast<int>(n), [&](int) { spin(nullptr, ms, p); }, static_cast<int>(n));
          return value_t(n);
        };
      },
      opts);
  // (alloc-op): reports 100 KiB, then 4 MiB, through eval_alloc_check.
  register_async_fn(
      env, host, "alloc-op",
      [](std::span<const value_t>) -> async_kernel {
        return [](async_call &) -> value_t {
          eval_alloc_check(100 * 1024, "alloc-op");
          eval_alloc_check(4 * 1024 * 1024, "alloc-op");
          return value_t(0);
        };
      },
      opts);
  // (fanout-alloc): chunks on the compute pool's workers (not the kernel's thread) report 4 MiB.
  register_async_fn(
      env, host, "fanout-alloc",
      [&app](std::span<const value_t>) -> async_kernel {
        return [&app](async_call &) -> value_t {
          const std::thread::id kernel_thread = std::this_thread::get_id();
          std::atomic<int> on_workers{0};
          app.computePool().parallel_for(
              8,
              [&](int) {
                std::this_thread::sleep_for(10ms);
                if (std::this_thread::get_id() != kernel_thread) {
                  ++on_workers;
                  eval_alloc_check(4 * 1024 * 1024, "fanout-alloc chunk");
                }
              },
              8);
          return value_t(static_cast<int64_t>(on_workers.load()));
        };
      },
      opts);
}

// A cvc::app whose background pool can be stopped as ~app first stops it, with the app still
// there, so that what a host does with the pool stopped can be checked; and whose pool can be made
// unable to create a thread, as when thread resources are exhausted (a throw from the pool's test
// seam counts as the creation failing).
struct stoppable_app : cvc::app {
  std::atomic<bool> fail_threads{false};
  stoppable_app() {
    _poolWorkerSpawnHook = [this] {
      if (fail_threads.load())
        throw boost::thread_resource_error();
    };
  }
  void stop_pool() { stopThreadPool(); }
};

// One engine's world: the app (its background pool and compute pool), a scheduler, the host
// and an environment with the kernels and intrinsics.
struct world {
  unsigned e;
  stoppable_app app;
  cvc::state *tree = nullptr;
  probe p;
  std::unique_ptr<scheduler> sync;
  std::unique_ptr<async_scheduler> async;
  std::unique_ptr<async_host> host;
  intrinsics_context ictx;
  process_ptr ctx_proc = make_process();
  environment_ptr env = builtins::make_default_environment();

  explicit world(unsigned engine, async_options opts = {}) : e(engine) {
    tree = &cvc::state::instance(app);
    app.computePool();
    if (e == sync_sched) {
      sync = std::make_unique<scheduler>();
      sync->set_watch_root(tree);
      host = std::make_unique<async_host>(sync.get());
      ictx.sched = sync.get();
    } else if (e == async_sched) {
      async = std::make_unique<async_scheduler>();
      async->set_watch_root(tree);
      host = std::make_unique<async_host>(*async, &app);
      ictx.sched = async.get();
    } else {
      host = std::make_unique<async_host>(nullptr);
    }
    if (ictx.sched) {
      ictx.root = tree;
      ctx_proc->pid = 0;
      ictx.proc = ctx_proc;
      register_intrinsics(env, &ictx);
    }
    register_kernels(env, *host, app, p, opts);
    // (set-rate "r"): state_exec.defaults.limits.async_steps_per_cpu_ms, with no load_settings().
    builtins::register_fn(env, "set-rate", [this](std::span<const value_t> a) -> value_t {
      (*tree)("state_exec.defaults.limits.async_steps_per_cpu_ms").value(show(a[0]));
      return nil_value;
    });
    // (my-async-steps): the steps the calling process's async calls were charged so far.
    builtins::register_fn(env, "my-async-steps", [this](std::span<const value_t>) -> value_t {
      const auto info = ictx.sched->get_process_info(ictx.sched->current_pid());
      return value_t(static_cast<int64_t>(info ? info->async_steps : 0));
    });
  }
  ~world() {
    if (host)
      host->shutdown(); // before the scheduler it posts to
  }

  int submit(const std::string &script, execute_options opts = {}) {
    opts.env = env;
    return sync ? sync->execute(script, opts) : async->execute(script, opts);
  }

  // Drive the scheduler until `done` (or `secs` pass). Returns whether `done` held.
  template <class Done> bool pump(Done done, double secs = 15.0) {
    if (!sync && !async)
      return done(); // a bare evaluator: nothing to drive
    const auto start = clock_type::now();
    while (seconds_since(start) < secs) {
      if (sync) {
        sync->run(100000, 0.02);
      } else {
        async->drain_ingress();
        async->sync_run(100000, 0.02);
      }
      if (done())
        return true;
      std::this_thread::sleep_for(1ms);
    }
    return done();
  }

  process_info info(int pid) const {
    return *(sync ? sync->get_process_info(pid) : async->get_process_info(pid));
  }
  bool finished(int pid) const { return is_finished(info(pid).status); }
  bool waiting(int pid) const { return info(pid).status == process_status::waiting; }
  int run_to_end(int pid) {
    EXPECT_TRUE(pump([&] { return finished(pid); })) << "process " << pid << " didn't end";
    return pid;
  }
  std::optional<value_t> result(int pid) const {
    return sync ? sync->get_result(pid) : async->get_result(pid);
  }
  std::optional<value_t> exit_exception(int pid) const {
    return sync ? sync->get_exit_exception(pid) : async->get_exit_exception(pid);
  }
};

// What a program came to.
struct outcome {
  bool raised = false;
  std::string value, cls, message;
  value_t exc;
};

outcome record(const value_t &exc) {
  outcome o;
  o.raised = true;
  o.exc = exc;
  o.cls = class_name(class_of_instance(exc));
  o.message = exception_message(exc);
  return o;
}

// Run `script` in `w` to its end.
outcome run(world &w, const std::string &script, execute_options opts = {}) {
  try {
    switch (w.e) {
    case recursive: {
      evaluator ev(w.env);
      return {false, show(ev.evaluate_script(script))};
    }
    case stackless: {
      stackless_evaluator ev(w.env);
      return {false, show(ev.evaluate_script(script))};
    }
    default: {
      const int pid = w.run_to_end(w.submit(script, opts));
      if (const auto exc = w.exit_exception(pid))
        return record(*exc);
      const auto r = w.result(pid);
      outcome o;
      o.value = r ? show(*r) : "<" + std::string(to_string(w.info(pid).status)) + ">";
      return o;
    }
    }
  } catch (const std::exception &ex) {
    return record(exception_from(ex));
  }
}

template <class Body> void for_engines(unsigned engines, Body body) {
  for (unsigned e : {recursive, stackless, sync_sched, async_sched}) {
    if (!(engines & e))
      continue;
    SCOPED_TRACE(engine_name(e));
    body(e);
  }
}

} // namespace

// ===========================================================================
// Calling (AS-1, 2, 5, 12, 14)
// ===========================================================================

TEST(AsyncCalls, AS1_OtherProcessesStepWhileOneIsParked) {
  world w(async_sched);
  const auto start = clock_type::now();
  const int a = w.submit("(slow-op 2)");
  const int b = w.submit("(slow-op 3)");
  const int c = w.submit("(begin (set i 0) (while (< i 2000) (set i (+ i 1))) i)");
  bool c_ran_while_parked = false;
  uint64_t c_steps = 0;
  ASSERT_TRUE(w.pump([&] {
    const uint64_t steps = w.info(c).step_count;
    if (w.waiting(a) && w.waiting(b) && steps > c_steps)
      c_ran_while_parked = true;
    c_steps = steps;
    return w.finished(a) && w.finished(b) && w.finished(c);
  }));
  EXPECT_EQ(show(*w.result(a)), "4");
  EXPECT_EQ(show(*w.result(b)), "6");
  EXPECT_EQ(show(*w.result(c)), "2000");
  EXPECT_TRUE(c_ran_while_parked);
  EXPECT_LT(seconds_since(start), 5.0);
}

TEST(AsyncCalls, AS2_TheWholeProgramParks) {
  world w(async_sched);
  const int pid = w.submit("(slow-op 21)");
  bool parked = false;
  ASSERT_TRUE(w.pump([&] {
    parked = parked || w.waiting(pid);
    return w.finished(pid);
  }));
  EXPECT_TRUE(parked);
  EXPECT_EQ(show(*w.result(pid)), "42");
}

TEST(AsyncCalls, AS5_AwaitAFuture) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    EXPECT_EQ(run(w, "(await (slow-op-async 2))").value, "4");
    EXPECT_EQ(run(w, "(begin (set f (slow-op-async 5)) (list (await f) (await f)))").value,
              "(10 10)"); // a future can be awaited again
  });
}

TEST(AsyncCalls, AS12_InlineGivesTheSameValuesAndExceptions) {
  for_engines(kAll, [](unsigned e) {
    world w(e);
    EXPECT_EQ(run(w, "(slow-op 2)").value, "4");
    EXPECT_EQ(run(w, "(+ 1 (slow-op 2 5))").value, "5");
    const outcome o = run(w, "(throw-invalid)");
    ASSERT_TRUE(o.raised);
    EXPECT_EQ(o.cls, "ValueError");
    EXPECT_EQ(show(*field(o.exc, "origin")), "async");
  });
}

TEST(AsyncCalls, AS14_TheKernelGetsACopyOfAListArgument) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    EXPECT_EQ(run(w, "(begin (set l (list 1 2 3)) (set f (sum-later-async l 50)) (append l 4) "
                     "(await f))")
                  .value,
              "6");
  });
}

// ===========================================================================
// Errors (AS-3, 6, 7, 8)
// ===========================================================================

TEST(AsyncCalls, AS3_AKernelsErrorRaisesAtTheCall) {
  for_engines(kAll, [](unsigned e) {
    world w(e);
    EXPECT_EQ(run(w, "(try (throw-invalid) (catch (ValueError e) (list (get-attr e \"origin\") "
                     "(get-attr e \"function\") (get-attr e \"message\"))))")
                  .value,
              "(\"async\" \"throw-invalid\" \"bad argument\")");
  });
}

TEST(AsyncCalls, AS6_AwaitTimesOutAndTheFutureStillFinishes) {
  world w(async_sched);
  EXPECT_EQ(run(w, "(begin (set f (slow-op-async 2 200)) (set r (try (await f 0.01) (catch "
                   "(TimeoutError e) \"timeout\"))) (list r (await f)))")
                .value,
            "(\"timeout\" 4)");
  EXPECT_FALSE(w.p.cancelled_seen); // the work went on
}

TEST(AsyncCalls, AS7_AwaitAllRaisesTheFirstFailureOnceAllFinish) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    builtins::register_fn(w.env, "a-done", [&w](std::span<const value_t>) -> value_t {
      return value_t(w.p.a_done.load());
    });
    EXPECT_EQ(run(w, "(try (await-all (list (a-async 100) (b-async))) (catch (ValueError e) (list "
                     "(get-attr e \"message\") (a-done))))")
                  .value,
              "(\"b failed\" #t)");
    EXPECT_EQ(run(w, "(await-all (list (slow-op-async 1) 7 (slow-op-async 2)))").value, "(2 7 4)");
  });
}

namespace {
// (chan-future "c"): a channel future on c; (deliver "c" v): v to c's receivers, or c's queue.
void register_channel_fns(world &w) {
  builtins::register_fn(w.env, "chan-future", [](std::span<const value_t> a) -> value_t {
    return make_future(show(a[0]));
  });
  builtins::register_fn(w.env, "deliver", [&w](std::span<const value_t> a) -> value_t {
    return value_t(static_cast<int64_t>(w.ictx.sched->deliver_to_receivers(show(a[0]), a[1])));
  });
}
} // namespace

TEST(AsyncCalls, AwaitAllTakesChannelFutures) {
  // 3.3: with async calls' futures and plain values, in list order whatever order the messages
  // come in; a message sent before it waits is taken.
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    register_channel_fns(w);
    const int a =
        w.submit("(await-all (list (chan-future \"c1\") (slow-op-async 3 50) (chan-future "
                 "\"c2\") 9))");
    ASSERT_TRUE(w.pump([&] { return w.waiting(a); }));
    w.run_to_end(
        w.submit("(begin (deliver \"c2\" \"two\") (sleep 0.05) (deliver \"c1\" \"one\"))"));
    w.run_to_end(a);
    ASSERT_TRUE(w.result(a));
    EXPECT_EQ(show(*w.result(a)), "(\"one\" 6 \"two\" 9)");
    EXPECT_EQ(run(w, "(begin (deliver \"c3\" 1) (deliver \"c4\" 2) (await-all (list (chan-future "
                     "\"c4\") (chan-future \"c3\"))))")
                  .value,
              "(2 1)");
    // A message that is itself a channel future is the value, not waited for in turn.
    EXPECT_EQ(run(w, "(begin (deliver \"c5\" (chan-future \"c6\")) (get-attr (car (await-all (list "
                     "(chan-future \"c5\")))) \"__future__\"))")
                  .value,
              "c6");
  });
}

TEST(AsyncCalls, AwaitAllKeepsTheMessagesItHasInASnapshot) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    register_channel_fns(w);
    const int a = w.submit("(await-all (list (chan-future \"s1\") 5 (chan-future \"s2\")))");
    ASSERT_TRUE(w.pump([&] { return w.waiting(a); }));
    // s1's message arrives; it waits on s2 by the time the sender ends.
    w.run_to_end(w.submit("(begin (deliver \"s1\" \"one\") (sleep 0.05))"));
    ASSERT_TRUE(w.waiting(a));
    cvc::state &snap = (*w.tree)("__await_all_snapshot__");
    ASSERT_TRUE(w.sync ? w.sync->snapshot(a, snap) : w.async->snapshot(a, snap));
    const int r = w.sync ? w.sync->restore(snap, w.env) : w.async->restore(snap, w.env);
    ASSERT_TRUE(w.pump([&] { return w.waiting(r); })) << "the restored process didn't wait again";
    w.run_to_end(w.submit("(deliver \"s2\" \"two\")"));
    for (const int pid : {a, r}) {
      w.run_to_end(pid);
      ASSERT_TRUE(w.result(pid)) << pid;
      EXPECT_EQ(show(*w.result(pid)), "(\"one\" 5 \"two\")") << pid;
    }
  });
}

TEST(AsyncCalls, AS8_ARegistrationTimeoutRaisesTimeoutError) {
  for_engines(kAll, [](unsigned e) {
    async_options opts;
    opts.timeout = 0.05;
    world w(e, opts);
    const auto start = clock_type::now();
    const outcome o = run(w, "(try (slow-op 1 1000) (catch (TimeoutError e) (get-attr e "
                             "\"function\")))");
    EXPECT_EQ(o.value, "slow-op") << o.cls << ": " << o.message;
    EXPECT_LT(seconds_since(start), 0.9);
    // The kernel sees cancelled() (at once inline; on its thread soon after a threaded timeout).
    EXPECT_TRUE(w.pump([&] { return w.p.cancelled_seen.load(); }, 2.0) || w.p.cancelled_seen);
  });
}

// ===========================================================================
// Cancellation and lifetime (AS-9, 10, 11)
// ===========================================================================

TEST(AsyncCalls, AS9_KillingAParkedProcessCancelsItsCall) {
  world w(async_sched);
  const int pid = w.submit("(slow-op 1 5000)");
  ASSERT_TRUE(w.pump([&] { return w.waiting(pid) && w.p.started; }));
  EXPECT_EQ(w.async->async_calls_in_flight(), 1u);
  ASSERT_TRUE(w.async->kill(pid));
  EXPECT_EQ(w.async->async_calls_in_flight(), 0u);
  EXPECT_TRUE(w.pump([&] { return w.p.cancelled_seen.load(); }, 3.0));
  w.host->wait_idle();
  w.async->drain_ingress(); // the late completion is dropped
  EXPECT_EQ(w.info(pid).status, process_status::killed);
  EXPECT_EQ(w.info(pid).exit_error.value_or(""), "killed_by_user");
  EXPECT_FALSE(w.result(pid));
}

// The cancel a reap makes first (pycvc's Exec makes it before its control mutex) leaves a live
// process's calls alone: dropped, its call would leave the process parked on it for good.
TEST(AsyncCalls, CancelBeforeAReapLeavesALiveProcessesCalls) {
  world w(async_sched);
  const int pid = w.submit("(slow-op 3 50)");
  ASSERT_TRUE(w.pump([&] { return w.waiting(pid) && w.p.started; }));
  w.async->cancel_async_calls_of(pid);
  EXPECT_EQ(w.async->async_calls_in_flight(), 1u);
  ASSERT_TRUE(w.pump([&] { return w.result(pid).has_value(); }));
  EXPECT_EQ(to_string(*w.result(pid)), "6");
}

TEST(AsyncCalls, AS10_HostTeardownWaitsForItsKernels) {
  auto w = std::make_unique<world>(async_sched);
  const int pid = w->submit("(slow-op 1 5000)");
  ASSERT_TRUE(w->pump([&] { return w->waiting(pid) && w->p.started; }));
  const auto start = clock_type::now();
  w->host->shutdown(); // cancels the call, waits for its kernel
  EXPECT_EQ(w->host->in_flight(), 0u);
  EXPECT_TRUE(w->p.cancelled_seen);
  EXPECT_LT(seconds_since(start), 2.0);
  // A call made after the shutdown raises.
  const int late = w->submit("(try (slow-op 1) (catch (RuntimeError e) \"closed\"))");
  ASSERT_TRUE(w->pump([&] { return w->finished(late); }));
  EXPECT_EQ(show(*w->result(late)), "closed");
  w.reset(); // the scheduler goes after the host, with nothing in flight
}

TEST(AsyncCalls, AS11_ABlockedKernelDoesntDelayAComputeFanOut) {
  world w(async_sched);
  const int pid = w.submit("(nap-blind 400)");
  ASSERT_TRUE(w.pump([&] { return w.waiting(pid); }));
  std::this_thread::sleep_for(20ms); // the kernel is asleep on its background thread
  double fanout = 0.0;
  std::thread other([&] {
    const auto start = clock_type::now();
    w.app.computePool().parallel_for(64, [](int) { std::this_thread::sleep_for(1ms); });
    fanout = seconds_since(start);
  });
  other.join();
  EXPECT_LT(fanout, 0.3) << "the fan-out waited for the sleeping kernel";
  w.run_to_end(pid);
  EXPECT_EQ(show(*w.result(pid)), "400");
}

// ===========================================================================
// Progress, snapshots, where it can't park (AS-13, 15, 17)
// ===========================================================================

TEST(AsyncCalls, AS13_ProgressReachesTheProgressChannel) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    EXPECT_EQ(run(w, "(begin (progress-op 2) (list (msg-recv \"progress-op.progress\") (msg-recv "
                     "\"progress-op.progress\")))")
                  .value,
              "(1 2)");
  });
}

TEST(AsyncCalls, AS15_ARestoredCallCantResume) {
  world w(async_sched);
  const int pid =
      w.submit("(try (slow-op 2 300) (catch (RuntimeError e) (get-attr e \"message\")))");
  ASSERT_TRUE(w.pump([&] { return w.waiting(pid); }));
  cvc::state &snap = (*w.tree)("__as15_snapshot__");
  ASSERT_TRUE(w.async->snapshot(pid, snap));
  const int restored = w.async->restore(snap, w.env);
  w.run_to_end(restored);
  const auto r = w.result(restored);
  ASSERT_TRUE(r);
  EXPECT_NE(show(*r).find("can't resume"), std::string::npos) << show(*r);
  w.run_to_end(pid); // the original resumes as usual
  EXPECT_EQ(show(*w.result(pid)), "4");
}

TEST(AsyncCalls, AS17_WhereParkingIsForbiddenACallRaises) {
  for_engines(stackless | async_sched, [](unsigned e) {
    world w(e);
    stackless_evaluator ev(w.env);
    park_forbidden_scope no_parking;
    try {
      ev.evaluate_script("(slow-op 1)");
      ADD_FAILURE() << "the call didn't raise";
    } catch (const dsl_exception &ex) {
      EXPECT_EQ(ex.class_name(), "RuntimeError");
      EXPECT_NE(std::string(ex.what()).find("slow-op"), std::string::npos);
    }
    EXPECT_EQ(w.p.ended, 0); // nothing ran
  });
}

TEST(AsyncCalls, AKernelCantUseTheEvaluatorOrAScheduler) {
  // 3.7: a kernel runs off the scheduler's timeline. The evaluators and the schedulers' entry
  // points refuse it, on its thread and in its fan-out's chunks, and the error raises at the call.
  for_engines(kAll, [](unsigned e) {
    world w(e);
    // (eval-in-kernel stackless?): evaluates (+ 1 2) in the kernel.
    register_async_fn(w.env, *w.host, "eval-in-kernel",
                      [](std::span<const value_t> a) -> async_kernel {
                        const bool use_stackless = int_arg(a, 0, 0) != 0;
                        return [use_stackless](async_call &) -> value_t {
                          const environment_ptr env = builtins::make_default_environment();
                          if (use_stackless)
                            return stackless_evaluator(env).evaluate_script("(+ 1 2)");
                          return evaluator(env).evaluate_script("(+ 1 2)");
                        };
                      });
    // (sched-in-kernel): kills a process that doesn't exist, from the kernel.
    scheduler_base *sched = w.ictx.sched;
    register_async_fn(
        w.env, *w.host, "sched-in-kernel", [sched](std::span<const value_t>) -> async_kernel {
          return [sched](async_call &) -> value_t { return value_t(sched && sched->kill(123456)); };
        });
    // (chunks-marked): how many of 8 fan-out chunks run marked as the kernel's.
    register_async_fn(w.env, *w.host, "chunks-marked",
                      [&w](std::span<const value_t>) -> async_kernel {
                        return [&w](async_call &) -> value_t {
                          std::atomic<int> marked{0};
                          w.app.computePool().parallel_for(
                              8,
                              [&](int) {
                                std::this_thread::sleep_for(2ms);
                                if (in_async_kernel())
                                  ++marked;
                              },
                              8);
                          return value_t(static_cast<int64_t>(marked.load()));
                        };
                      });
    builtins::register_fn(w.env, "in-kernel?", [](std::span<const value_t>) -> value_t {
      return value_t(in_async_kernel());
    });
    for (const char *which : {"0", "1"}) {
      SCOPED_TRACE(which);
      const outcome o = run(w, std::string("(try (eval-in-kernel ") + which +
                                   ") (catch (NativeError e) (list (get-attr e \"origin\") "
                                   "(get-attr e \"message\"))))");
      EXPECT_NE(o.value.find("\"async\""), std::string::npos) << o.value << o.cls << o.message;
      EXPECT_NE(o.value.find("must not use the evaluator"), std::string::npos) << o.value;
    }
    if (w.ictx.sched) {
      const outcome o =
          run(w, "(try (sched-in-kernel) (catch (NativeError e) (get-attr e \"message\")))");
      EXPECT_NE(o.value.find("kill: an async function's kernel"), std::string::npos)
          << o.value << o.cls << o.message;
    }
    EXPECT_EQ(run(w, "(list (chunks-marked) (in-kernel?))").value, "(8 #f)");
    EXPECT_FALSE(in_async_kernel());
  });
}

// ===========================================================================
// Budgets (AS-18..33)
// ===========================================================================

TEST(AsyncCalls, AS18_AS26_ACpuBoundCallsCharges) {
  // AS-18 on the async scheduler, AS-26 inline on the sync scheduler: the same charges.
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    const int pid = w.run_to_end(w.submit("(busy 200)"));
    const process_info i = w.info(pid);
    EXPECT_EQ(i.async_calls, 1u);
    EXPECT_GE(i.async_cpu_time, 0.19);
    EXPECT_LT(i.async_cpu_time, 0.4);
    EXPECT_GE(i.async_time, i.async_cpu_time - 0.02);
    const double at_default = 200.0 * kDefaultAsyncStepsPerCpuMs; // 0.2 CPU s at the default
    EXPECT_GE(double(i.async_steps), 0.9 * at_default);
    EXPECT_LT(double(i.async_steps), 2.0 * at_default);
    EXPECT_GE(i.step_count, i.async_steps); // they count in step_count
    EXPECT_GE(i.elapsed_time, i.async_time - 1e-3);
  });
}

TEST(AsyncCalls, AS19_RunningOutOfTimeInACallKills) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    execute_options opts;
    opts.max_time = 0.1;
    const auto start = clock_type::now();
    const int pid = w.run_to_end(w.submit("(try (busy 1000) (catch (e) \"caught\"))", opts));
    EXPECT_LT(seconds_since(start), 0.8);
    const process_info i = w.info(pid);
    EXPECT_EQ(i.status, process_status::killed);
    EXPECT_EQ(i.exit_error.value_or(""), "time_limit_exceeded");
    EXPECT_FALSE(w.result(pid)); // the catch didn't run
    EXPECT_TRUE(w.pump([&] { return w.p.cancelled_seen.load(); }, 2.0));
  });
}

TEST(AsyncCalls, AS20_ACallsStepsCountAgainstMaxSteps) {
  for_engines(kSchedulers, [](unsigned e) {
    {
      world w(e);
      execute_options opts;
      opts.max_steps = 1000;
      const int pid = w.run_to_end(w.submit("(busy 50)", opts));
      EXPECT_EQ(w.info(pid).status, process_status::killed);
      EXPECT_EQ(w.info(pid).exit_error.value_or(""), "max_steps_exceeded");
    }
    {
      world w(e);
      (*w.tree)("state_exec.defaults.limits.async_steps_per_cpu_ms").value("0");
      execute_options opts;
      opts.max_steps = 1000;
      const int pid = w.run_to_end(w.submit("(busy 50)", opts));
      EXPECT_EQ(w.info(pid).status, process_status::terminated);
      EXPECT_EQ(w.info(pid).async_steps, 1u); // each call counts its one step
    }
  });
}

TEST(AsyncCalls, AS21_AS29_ASleepingCallIsChargedItsRunTimeButNoCpu) {
  // AS-29 is this case on Windows: the thread CPU clock there reads CPU time, not wall time.
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    const int pid = w.run_to_end(w.submit("(slow-op 1 200)"));
    const process_info i = w.info(pid);
    EXPECT_GE(i.async_time, 0.19);
    EXPECT_GE(i.elapsed_time, 0.19); // charged to max_time's first half
    EXPECT_LT(i.async_cpu_time, 0.03);
    EXPECT_GE(i.async_steps, 1u);
    EXPECT_LT(double(i.async_steps), 30.0 * kDefaultAsyncStepsPerCpuMs); // under 30 CPU ms
  });
}

// An I/O call (async_options::io, the network verbs): its wall time and CPU are the host's, so it
// is charged its one step, and a budget that a plain call of the same work runs out of holds.
TEST(AsyncCalls, AnIoCallIsChargedItsOneStep) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    async_options io;
    io.io = true;
    register_async_fn(
        w.env, *w.host, "io-busy",
        [&w](std::span<const value_t> a) -> async_kernel {
          const int64_t ms = int_arg(a, 0, 50);
          return [&w, ms](async_call &call) -> value_t {
            spin(&call, ms, w.p);
            return value_t(ms);
          };
        },
        io);
    register_async_fn(
        w.env, *w.host, "io-nap",
        [](std::span<const value_t> a) -> async_kernel {
          const int64_t ms = int_arg(a, 0, 50);
          return [ms](async_call &) -> value_t {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            return value_t(ms);
          };
        },
        io);
    execute_options opts;
    opts.max_steps = 1000; // (busy 50) runs out of these (AS-20)
    opts.max_time = 0.1;   // and (slow-op 1 200) out of this
    const int pid = w.run_to_end(w.submit("(+ (io-busy 50) (io-nap 200))", opts));
    const process_info i = w.info(pid);
    EXPECT_EQ(i.status, process_status::terminated) << i.exit_error.value_or("");
    ASSERT_TRUE(w.result(pid));
    EXPECT_EQ(show(*w.result(pid)), "250");
    EXPECT_EQ(i.async_calls, 2u);
    EXPECT_EQ(i.async_steps, 2u); // one step each
    EXPECT_EQ(i.async_time, 0.0);
    EXPECT_EQ(i.async_cpu_time, 0.0);
  });
}

TEST(AsyncCalls, AS22_AFanOutsCpuTimeIsCharged) {
  if (std::thread::hardware_concurrency() < 4)
    GTEST_SKIP() << "needs 4 cores";
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    const int pid = w.run_to_end(w.submit("(fanout 4 100)"));
    const process_info i = w.info(pid);
    EXPECT_GE(i.async_cpu_time, 0.3);
    EXPECT_GT(i.async_cpu_time, i.async_time);
  });
}

TEST(AsyncCalls, AS23_TimeInTheQueueIsntCharged) {
  world w(async_sched);
  w.app.setThreadPoolSize(1);
  const int first = w.submit("(slow-op 1 200)");
  ASSERT_TRUE(w.pump([&] { return w.waiting(first); }));
  const int second = w.submit("(slow-op 1 50)");
  w.run_to_end(first);
  w.run_to_end(second);
  EXPECT_GE(w.info(first).async_time, 0.19);
  EXPECT_LT(w.info(second).async_time, 0.15); // its own run, not its wait for the thread
  EXPECT_GE(w.info(second).async_time, 0.045);
}

TEST(AsyncCalls, AS24_FuturesInFlightAreEachCharged) {
  world w(async_sched);
  const auto start = clock_type::now();
  const int pid = w.run_to_end(
      w.submit("(begin (set a (slow-op-async 1 100)) (set b (slow-op-async 1 100)) (+ (await a) "
               "(await b)))"));
  const double wall = seconds_since(start);
  EXPECT_EQ(show(*w.result(pid)), "4");
  EXPECT_EQ(w.info(pid).async_calls, 2u);
  EXPECT_GE(w.info(pid).async_time, 0.19);
  EXPECT_LT(wall, w.info(pid).async_time + 0.05); // more run time than the wall clock showed
}

TEST(AsyncCalls, AS25_ACallThatNeverChecksIsStoppedByItsRunTime) {
  world w(async_sched);
  execute_options opts;
  opts.max_time = 0.1;
  const auto start = clock_type::now();
  const int pid = w.submit("(nap-blind 600)", opts);
  ASSERT_TRUE(w.pump([&] { return w.finished(pid); }));
  EXPECT_LT(seconds_since(start), 0.5);
  EXPECT_EQ(w.info(pid).status, process_status::killed);
  EXPECT_EQ(w.info(pid).exit_error.value_or(""), "time_limit_exceeded");
  w.host->wait_idle(); // its late result is dropped
  w.async->drain_ingress();
  EXPECT_TRUE(w.p.late_finished);
  EXPECT_FALSE(w.result(pid));
}

TEST(AsyncCalls, AS27_TheStepRateIsReadAtEveryCharge) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    const outcome o = run(w, "(begin (busy 50) (set s1 (my-async-steps)) (set-rate 0) (busy 50) "
                             "(list s1 (- (my-async-steps) s1)))");
    ASSERT_FALSE(o.raised) << o.cls << ": " << o.message;
    const value_t v = parse(o.value);
    const auto &l = *std::get<list_ptr>(v.v);
    const int64_t first = std::get<int64_t>(l[0].v), second = std::get<int64_t>(l[1].v);
    const double at_default = 50.0 * kDefaultAsyncStepsPerCpuMs; // 50 CPU ms at the default
    EXPECT_GE(double(first), 0.8 * at_default);
    EXPECT_LT(double(first), 2.0 * at_default);
    EXPECT_EQ(second, 1);
  });
}

TEST(AsyncCalls, AS28_TheSchedulersRateWinsAndTheProcessesWinsOverIt) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    if (w.sync)
      w.sync->set_id("t");
    else
      w.async->set_id("t");
    (*w.tree)("state_exec.defaults.limits.async_steps_per_cpu_ms").value("10");
    (*w.tree)("state_exec.schedulers.t.limits.async_steps_per_cpu_ms").value("100");
    const int pid = w.run_to_end(w.submit("(busy 50)"));
    EXPECT_GE(w.info(pid).async_steps, 4000u);
    EXPECT_LT(w.info(pid).async_steps, 10000u);
    // Published as the effective value, under the scheduler's own subtree.
    EXPECT_EQ((*w.tree)("state_exec.schedulers.t.effective.async_steps_per_cpu_ms").value(), "100");
    execute_options opts;
    opts.async_steps_per_cpu_ms = 1.0;
    const int own = w.run_to_end(w.submit("(busy 50)", opts));
    EXPECT_GE(w.info(own).async_steps, 40u);
    EXPECT_LT(w.info(own).async_steps, 100u);
    // A decimal rate: 0.5 is a step every 2 CPU ms.
    (*w.tree)("state_exec.schedulers.t.limits.async_steps_per_cpu_ms").value("0.5");
    const int half = w.run_to_end(w.submit("(busy 50)"));
    EXPECT_GE(w.info(half).async_steps, 20u);
    EXPECT_LT(w.info(half).async_steps, 50u);
  });
}

TEST(AsyncCalls, AS30_AKernelRunsUnderTheProcessesAllocationLimit) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    execute_options opts;
    opts.max_memory = 1024 * 1024;
    const int pid =
        w.submit("(try (alloc-op) (catch (MemoryError e) (get-attr e \"origin\")))", opts);
    // What the kernel reported before the refusal reaches the process's heap accounting; a
    // measurement may take it right after, so watch for it as the process runs.
    uint64_t reported = 0, measurements = 0;
    ASSERT_TRUE(w.pump([&] {
      const process_info i = w.info(pid);
      reported = std::max<uint64_t>(reported, i.heap_alloc_bytes);
      measurements = std::max<uint64_t>(measurements, i.heap_measurements);
      return w.finished(pid);
    }));
    EXPECT_EQ(show(*w.result(pid)), "async");
    EXPECT_TRUE(reported >= 100 * 1024 || measurements > 0)
        << "reported " << reported << ", measurements " << measurements;
  });
}

TEST(AsyncCalls, AS31_TheLimitReachesTheKernelsFanOut) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    execute_options opts;
    opts.max_memory = 1024 * 1024;
    EXPECT_EQ(run(w, "(try (fanout-alloc) (catch (MemoryError e) \"refused\"))", opts).value,
              "refused");
    // Without a budget the chunks' 4 MiB passes (the default limit is 16 MiB).
    world unlimited(e);
    EXPECT_NE(run(unlimited, "(fanout-alloc)").value, "0"); // chunks ran on workers
  });
}

TEST(AsyncCalls, AS33_TheCountersSurviveASnapshot) {
  for_engines(kSchedulers, [](unsigned e) {
    world w(e);
    const int pid = w.submit("(begin (slow-op 1 100) (msg-recv \"never\"))");
    // The call is done (charged in full) and the process waits in msg-recv.
    ASSERT_TRUE(w.pump([&] {
      return w.p.ended == 1 && w.waiting(pid) &&
             (!w.async || w.async->async_calls_in_flight() == 0);
    }));
    EXPECT_GE(w.info(pid).async_time, 0.09);
    cvc::state &snap = (*w.tree)("__as33_snapshot__");
    ASSERT_TRUE(w.sync ? w.sync->snapshot(pid, snap) : w.async->snapshot(pid, snap));
    const int restored = w.sync ? w.sync->restore(snap, w.env) : w.async->restore(snap, w.env);
    const process_info before = w.info(pid), after = w.info(restored);
    EXPECT_EQ(after.async_calls, before.async_calls);
    EXPECT_DOUBLE_EQ(after.async_time, before.async_time);
    EXPECT_DOUBLE_EQ(after.async_cpu_time, before.async_cpu_time);
    EXPECT_EQ(after.async_steps, before.async_steps);
    EXPECT_EQ(after.step_count, before.step_count);
    EXPECT_NEAR(after.elapsed_time, before.elapsed_time, 1e-6);
  });
}

// ===========================================================================
// What a step costs (3.8: the default of limits.async_steps_per_cpu_ms)
// ===========================================================================

// Whether the compiler optimized this build, which is what the default rate is set for. Not
// NDEBUG, which a Debug build can define too (the mesher's did, for all of src/cvc). Nor a
// sanitizer's build, whose steps cost several times more.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define CVC_TEST_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define CVC_TEST_SANITIZED 1
#endif
#if (defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))) &&                          \
    !defined(CVC_TEST_SANITIZED)
constexpr bool kOptimizedBuild = true;
#else
constexpr bool kOptimizedBuild = false;
#endif

// Measure the CPU time a scheduler step takes on representative programs, the basis of the
// default rate: an async call is charged the steps its CPU time would have bought. It runs the
// same number of steps of each, reports the steps per CPU millisecond, and, in an optimized
// build (the default is set for one), checks that the default stays within a factor of four of
// what the cheapest and the dearest of them cost. An unoptimized build's steps cost 20 to 40
// times more.
TEST(AsyncCalls, StepCostSetsTheDefaultRate) {
  constexpr uint64_t kSteps = 100000; // each program runs longer than this
  const char *programs[] = {
      // arithmetic, set and a loop
      "(begin (set i 0) (while (< i 300000) (set i (+ i 1))) i)",
      // DSL function calls
      "(begin (defun f (x) (+ x 1)) (set i 0) (while (< i 100000) (set i (f i))) i)",
      // building and reading a list, and a dict
      "(begin (set l (list)) (set i 0) (while (< i 50000) (begin (append l i) (set i (+ i 1)))) "
      "(set d (dict \"n\" 0)) (for x l (set-attr d \"n\" (+ (get-attr d \"n\") x))) (get-attr d "
      "\"n\"))",
  };
  double lowest = 1e18, highest = 0.0;
  for (unsigned e : {sync_sched, async_sched}) {
    for (const char *program : programs) {
      world w(e);
      if (w.async)
        w.async->set_data_defaults(async_scheduler::host_data_defaults());
      const int pid = w.submit(program);
      const double cpu0 = thread_cpu_seconds();
      if (w.sync)
        w.sync->run(kSteps);
      else
        w.async->sync_run(kSteps);
      const double cpu_ms = 1e3 * (thread_cpu_seconds() - cpu0);
      const uint64_t steps = w.info(pid).step_count;
      EXPECT_FALSE(w.finished(pid)) << "the program ended before the measurement did";
      const double rate = double(steps) / cpu_ms;
      std::printf("STEPCOST %s %.0f steps per CPU ms (%llu steps, %.1f CPU ms): %.50s\n",
                  engine_name(e), rate, (unsigned long long)steps, cpu_ms, program);
      lowest = std::min(lowest, rate);
      highest = std::max(highest, rate);
    }
  }
  if (!kOptimizedBuild) {
    std::printf("STEPCOST an unoptimized build: the default (%g) is set for an optimized one\n",
                kDefaultAsyncStepsPerCpuMs);
    return;
  }
  EXPECT_GE(kDefaultAsyncStepsPerCpuMs, lowest / 4.0);
  EXPECT_LE(kDefaultAsyncStepsPerCpuMs, highest * 4.0);
}

// ===========================================================================
// The thread CPU clock (3.8; AS-29's premise)
// ===========================================================================

TEST(AsyncCalls, ThreadCpuClockReadsCpuTimeNotWallTime) {
  const double cpu0 = thread_cpu_seconds();
  std::this_thread::sleep_for(100ms);
  EXPECT_LT(thread_cpu_seconds() - cpu0, 0.02); // asleep: almost no CPU time
  probe p;
  const double cpu1 = thread_cpu_seconds();
  spin(nullptr, 50, p);
  EXPECT_GE(thread_cpu_seconds() - cpu1, 0.049);
}

// ===========================================================================
// The background executor: persistent workers, nothing retained
// ===========================================================================

// More calls in flight than the app's background pool has workers: every call completes,
// promptly, on no more threads than the pool's size. (The pool used to start a thread per call,
// give the calls queued while it was full one thread key, and stall 5 s per key collision before
// hanging for good: docs/STATE_EXEC_BENCH.md, "The background pool stalls, then hangs".)
TEST(AsyncCalls, MoreCallsThanPoolWorkersAllCompleteOnThePoolsWorkers) {
  world w(async_sched);
  w.app.setThreadPoolSize(2);
  std::mutex m;
  std::set<std::thread::id> threads;
  register_async_fn(w.env, *w.host, "where-op", [&](std::span<const value_t> a) -> async_kernel {
    const value_t x = a.empty() ? value_t{} : a[0];
    return [&, x](async_call &) -> value_t {
      {
        std::lock_guard<std::mutex> lk(m);
        threads.insert(std::this_thread::get_id());
      }
      std::this_thread::sleep_for(1ms);
      return x;
    };
  });
  const auto start = clock_type::now();
  std::vector<int> pids;
  for (int i = 0; i < 40; ++i)
    pids.push_back(w.submit("(where-op " + std::to_string(i) + ")"));
  ASSERT_TRUE(w.pump([&] {
    return std::all_of(pids.begin(), pids.end(), [&](int pid) { return w.finished(pid); });
  }));
  for (int i = 0; i < 40; ++i)
    EXPECT_EQ(show(*w.result(pids[i])), std::to_string(i));
  EXPECT_LT(seconds_since(start), 5.0);
  EXPECT_LE(threads.size(), 2u);
  EXPECT_EQ(w.app.threads().size(), 0u); // pooled kernels have no thread-map entries
  w.host->wait_idle();
  EXPECT_EQ(w.host->in_flight(), 0u);
}

// Once a call is over, nothing keeps it or its kernel: the kernel's captures go on its thread
// before the host's count drops, and the call itself (its result) once the scheduler and the
// program are done with it. (Each pooled thread used to hold its task, and with it the call and
// its result, until the app went.)
TEST(AsyncCalls, AFinishedCallKeepsNothingAlive) {
  world w(async_sched);
  std::vector<std::weak_ptr<int>> captures;
  std::vector<std::weak_ptr<async_call>> calls;
  std::mutex m;
  register_async_fn(w.env, *w.host, "keep-op", [&](std::span<const value_t>) -> async_kernel {
    auto sentinel = std::make_shared<int>(1);
    {
      std::lock_guard<std::mutex> lk(m);
      captures.push_back(sentinel);
    }
    return [&, sentinel](async_call &call) -> value_t {
      std::lock_guard<std::mutex> lk(m);
      calls.push_back(std::static_pointer_cast<async_call>(call.shared_from_this()));
      return value_t(std::string(4096, 'x'));
    };
  });
  for (int i = 0; i < 3; ++i) {
    const int pid = w.run_to_end(w.submit("(length (keep-op))"));
    EXPECT_EQ(show(*w.result(pid)), "4096");
  }
  const int fut = w.run_to_end(w.submit("(length (await (keep-op-async)))"));
  EXPECT_EQ(show(*w.result(fut)), "4096");
  w.host->wait_idle();
  w.async->drain_ingress();
  ASSERT_EQ(captures.size(), 4u);
  ASSERT_EQ(calls.size(), 4u);
  for (const auto &c : captures)
    EXPECT_TRUE(c.expired()) << "a kernel's captures outlived its call";
  for (const auto &c : calls)
    EXPECT_TRUE(c.expired()) << "a finished call is still held";
}

// A call whose kernel returns within its timeout keeps its value however long freeing what the
// kernel captured takes: the call is settled when the kernel ends, and the kernel freed after.
TEST(AsyncCalls, FreeingAKernelsCapturesIsntCountedAgainstItsTimeout) {
  world w(async_sched);
  struct slow_to_free {
    ~slow_to_free() { std::this_thread::sleep_for(400ms); }
  };
  async_options opts;
  opts.timeout = 0.2;
  register_async_fn(
      w.env, *w.host, "slow-free-op",
      [](std::span<const value_t>) -> async_kernel {
        auto held = std::make_shared<slow_to_free>();
        return [held](async_call &) -> value_t { return value_t(static_cast<int64_t>(7)); };
      },
      opts);
  const int pid =
      w.run_to_end(w.submit("(try (slow-free-op) (catch (TimeoutError e) \"timeout\"))"));
  ASSERT_TRUE(w.result(pid));
  EXPECT_EQ(show(*w.result(pid)), "7");
  w.host->wait_idle();
}

// The host forgets finished calls in batches, but every call still in flight is cancelled at
// shutdown, after any number of finished ones.
TEST(AsyncCalls, ShutdownCancelsTheCallInFlightAfterManyFinished) {
  auto w = std::make_unique<world>(async_sched);
  const int quick = w->run_to_end(
      w->submit("(begin (set i 0) (while (< i 100) (begin (slow-op i 0) (set i (+ i 1)))) i)"));
  EXPECT_EQ(show(*w->result(quick)), "100");
  w->p.started = false;
  const int pid = w->submit("(slow-op 1 5000)");
  ASSERT_TRUE(w->pump([&] { return w->waiting(pid) && w->p.started; }));
  const auto start = clock_type::now();
  w->host->shutdown();
  EXPECT_EQ(w->host->in_flight(), 0u);
  EXPECT_TRUE(w->p.cancelled_seen);
  EXPECT_LT(seconds_since(start), 2.0);
  w.reset();
}

// ===========================================================================
// The app's background pool stops under a host (~app stops it first)
// ===========================================================================

namespace {

// Drive `w` until `done`, at most `secs`; the world is leaked, not destroyed, if a call is still
// in flight then: its host's shutdown() would wait for that call for good.
template <class Done> bool pump_or_leak(std::unique_ptr<world> &w, Done done, double secs = 5.0) {
  const bool ok = w->pump(done, secs);
  if (w->host->in_flight() != 0) {
    ADD_FAILURE() << w->host->in_flight() << " call(s) never left the host";
    (void)w.release();
  }
  return ok;
}

} // namespace

// Once the app's background pool has stopped, a kernel submitted to it is dropped, and its call
// says so: it raises RuntimeError at the call and leaves the host, so the process isn't left
// parked and shutdown() doesn't wait for it for good. (The drop used to be silent: the call was
// never settled and the host's count of kernels in flight never went down.)
TEST(AsyncCalls, ACallMadeOnceTheAppsPoolHasStoppedRaisesAndLeavesTheHost) {
  auto w = std::make_unique<world>(async_sched);
  w->app.stop_pool();
  const int pid = w->submit("(slow-op 1 10)");
  const bool ended = pump_or_leak(w, [&] { return w->finished(pid); });
  if (!w)
    return;
  ASSERT_TRUE(ended) << "the call never ended";
  const auto exc = w->exit_exception(pid);
  ASSERT_TRUE(exc);
  const outcome o = record(*exc);
  EXPECT_EQ(o.cls, "RuntimeError");
  EXPECT_NE(o.message.find("slow-op: not run: the app's background pool has stopped"),
            std::string::npos)
      << o.message;
  EXPECT_FALSE(w->p.started);
  const auto start = clock_type::now();
  w->host->shutdown();
  EXPECT_LT(seconds_since(start), 1.0);
}

// A kernel still queued when the pool stops (~app drops the queue) is dropped the same way: its
// call raises RuntimeError, naming the stopped pool as the cause, and leaves the host. The kernel
// already running finishes.
TEST(AsyncCalls, ACallStillQueuedWhenTheAppsPoolStopsRaisesAndLeavesTheHost) {
  auto w = std::make_unique<world>(async_sched);
  w->app.setThreadPoolSize(1);
  const int first = w->submit("(nap-blind 300)"); // holds the pool's one worker
  ASSERT_TRUE(w->pump([&] { return w->waiting(first) && w->app.getActiveThreadCount() == 1; }));
  const int second = w->submit("(slow-op 2 10)");
  ASSERT_TRUE(w->pump([&] { return w->waiting(second) && w->app.getPendingThreadCount() == 1; }));
  w->app.stop_pool();
  const bool ended = pump_or_leak(w, [&] { return w->finished(first) && w->finished(second); });
  if (!w)
    return;
  ASSERT_TRUE(ended) << "a call never ended";
  ASSERT_TRUE(w->result(first));
  EXPECT_EQ(show(*w->result(first)), "300");
  const auto exc = w->exit_exception(second);
  ASSERT_TRUE(exc);
  const outcome o = record(*exc);
  EXPECT_EQ(o.cls, "RuntimeError");
  EXPECT_NE(o.message.find("slow-op: not run: the app's background pool has stopped"),
            std::string::npos)
      << o.message;
  EXPECT_FALSE(w->p.started);
}

// A kernel for a lane that has stopped is refused, and its call says so: it raises RuntimeError
// naming the lane (its name in the host, not the lane thread's), and leaves the host.
TEST(AsyncCalls, ACallToAStoppedLaneRaisesAndLeavesTheHost) {
  cvc::async_lane lane("work-thread"); // outlives the world, as a host's lanes must
  async_options opts;
  opts.where = executor::on_lane("work");
  auto w = std::make_unique<world>(async_sched, opts);
  w->host->add_lane("work", lane);
  lane.stop();
  const int pid = w->submit("(slow-op 1 10)");
  const bool ended = pump_or_leak(w, [&] { return w->finished(pid); });
  if (!w)
    return;
  ASSERT_TRUE(ended) << "the call never ended";
  const auto exc = w->exit_exception(pid);
  ASSERT_TRUE(exc);
  const outcome o = record(*exc);
  EXPECT_EQ(o.cls, "RuntimeError");
  EXPECT_NE(o.message.find("slow-op: lane 'work' is stopped"), std::string::npos) << o.message;
  EXPECT_FALSE(w->p.started);
  EXPECT_EQ(lane.completed(), 0u);
  const auto start = clock_type::now();
  w->host->shutdown();
  EXPECT_LT(seconds_since(start), 1.0);
}

// launch_pool_task posts exactly one value, also for a task the app drops: submitted once the
// pool has stopped, or still queued when it stops. The parked process resumes with on_error's
// value instead of waiting for good.
TEST(AsyncCalls, APoolTaskTheAppDropsStillPostsOnErrorsValue) {
  for (int queued = 0; queued < 2; ++queued) {
    SCOPED_TRACE(queued ? "queued" : "submitted after the stop");
    auto w = std::make_unique<world>(async_sched);
    std::atomic<bool> ran(false);
    async_scheduler &sched = *w->async;
    builtins::register_fn(w->env, "pool-double", [&](std::span<const value_t> a) -> value_t {
      const int64_t n = int_arg(a, 0, 0);
      return cvc::park_on_pool_task(
          w->app, sched, sched.current_process()->root_path, "pool-double.reply",
          [n, &ran] {
            ran = true;
            return value_t(n * 2);
          },
          [](const std::string &e) { return value_t("err: " + e); });
    });
    int first = -1;
    if (queued) {
      w->app.setThreadPoolSize(1);
      first = w->submit("(nap-blind 300)"); // holds the pool's one worker
      ASSERT_TRUE(w->pump([&] { return w->waiting(first) && w->app.getActiveThreadCount() == 1; }));
    } else {
      w->app.stop_pool();
    }
    const int pid = w->submit("(begin (pool-double 21))"); // a top-level call can't park
    if (queued) {
      ASSERT_TRUE(w->pump([&] { return w->app.getPendingThreadCount() == 1; }));
      w->app.stop_pool();
    }
    ASSERT_TRUE(w->pump([&] { return w->finished(pid); }, 5.0)) << "the process never resumed";
    if (const auto exc = w->exit_exception(pid))
      FAIL() << "raised " << record(*exc).cls << ": " << record(*exc).message;
    ASSERT_TRUE(w->result(pid));
    EXPECT_EQ(show(*w->result(pid)), "err: not run: the app's background pool has stopped");
    EXPECT_FALSE(ran.load());
    if (queued)
      w->run_to_end(first);
  }
}

// A call that no background worker can be started for (thread resources exhausted before the
// pool's first worker) raises RuntimeError at the call, saying so, and leaves the host: the
// process isn't left parked, and shutdown() doesn't wait for it. (The kernel used to wait in the
// pool's queue for a later submission to start a worker, and the host's shutdown, ~app's first
// step, waited for it for good.)
TEST(AsyncCalls, ACallNoWorkerCanBeStartedForRaisesAndLeavesTheHost) {
  auto w = std::make_unique<world>(async_sched);
  w->app.fail_threads = true;
  const int pid = w->submit("(slow-op 1 10)");
  const bool ended = pump_or_leak(w, [&] { return w->finished(pid); });
  if (!w)
    return;
  ASSERT_TRUE(ended) << "the call never ended";
  const auto exc = w->exit_exception(pid);
  ASSERT_TRUE(exc);
  const outcome o = record(*exc);
  EXPECT_EQ(o.cls, "RuntimeError");
  EXPECT_NE(o.message.find("slow-op: not run: no background worker could be started"),
            std::string::npos)
      << o.message;
  EXPECT_FALSE(w->p.started);
  EXPECT_EQ(w->app.getPendingThreadCount(), 0u);
  const auto start = clock_type::now();
  w->host->shutdown();
  EXPECT_LT(seconds_since(start), 1.0);
}

// launch_pool_task posts on_error's value at once when no background worker can be started for
// its task (thread resources exhausted before the pool's first worker): the parked process
// resumes with it rather than waiting for a later submission to start a worker.
TEST(AsyncCalls, APoolTaskNoWorkerCanBeStartedForPostsOnErrorsValue) {
  auto w = std::make_unique<world>(async_sched);
  std::atomic<bool> ran(false);
  async_scheduler &sched = *w->async;
  builtins::register_fn(w->env, "pool-double", [&](std::span<const value_t> a) -> value_t {
    const int64_t n = int_arg(a, 0, 0);
    return cvc::park_on_pool_task(
        w->app, sched, sched.current_process()->root_path, "pool-double.reply",
        [n, &ran] {
          ran = true;
          return value_t(n * 2);
        },
        [](const std::string &e) { return value_t("err: " + e); });
  });
  w->app.fail_threads = true;
  const int pid = w->submit("(begin (pool-double 21))"); // a top-level call can't park
  const bool resumed = w->pump([&] { return w->finished(pid); }, 5.0);
  if (!resumed)
    w->app.stop_pool(); // the queued task posts its drop while the scheduler is still there
  ASSERT_TRUE(resumed) << "the process never resumed";
  if (const auto exc = w->exit_exception(pid))
    FAIL() << "raised " << record(*exc).cls << ": " << record(*exc).message;
  ASSERT_TRUE(w->result(pid));
  EXPECT_EQ(show(*w->result(pid)), "err: not run: no background worker could be started");
  EXPECT_FALSE(ran.load());
  EXPECT_EQ(w->app.getPendingThreadCount(), 0u);
}

namespace {

// Hold the pool's one slot with a keyed task on a thread of its own, so that no worker is started,
// until `release`, an interrupt (~app's) or 20 s (so that a test that fails ends).
void hold_the_pool(stoppable_app &a, std::atomic<bool> &release) {
  a.setThreadPoolSize(1);
  a.startThreadPooled(
      "hold",
      [&release] {
        const auto until = clock_type::now() + 20s;
        while (!release.load() && clock_type::now() < until)
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // an interruption point
      },
      cvc::PRIORITY_NORMAL, /*wait=*/true);
}

// End the keyed task holding the slot with the pool unable to create a thread: the worker its
// thread asks for on the way out can't be created.
void release_the_pool_failing(stoppable_app &a, std::atomic<bool> &release) {
  a.fail_threads = true;
  release = true;
  if (cvc::thread_ptr hold = a.threads("hold"))
    hold->join();
}

// So that a test that fails ends: a worker that can start runs what is still queued.
void start_a_worker(stoppable_app &a) {
  a.fail_threads = false;
  a.submitPooled([] {});
}

} // namespace

// An async call can be queued with no worker running or being created: keyed tasks, on threads of
// their own, hold every slot of a pool that has never started a worker. When one of them ends
// and the worker its thread asks for can't be created (thread resources exhausted), the
// kernel isn't left queued for a later submission to start a worker: the pool drops it unrun, and
// its call raises RuntimeError ("not run: no background worker could be started") and leaves the
// host. A host bound to the app that shuts down before ~app (as pycvc's Exec's does) isn't held
// up. (The kernel used to stay queued: the process stayed parked, and the host's shutdown waited
// for the kernel until something started a worker.)
TEST(AsyncCalls, ACallLeftQueuedOnAPoolWithNoWorkerRaisesAndLeavesTheHost) {
  std::atomic<bool> release(false); // before the world: the keyed task reads it until ~app
  auto w = std::make_unique<world>(async_sched);
  hold_the_pool(w->app, release);
  const int pid = w->submit("(slow-op 1 10)");
  ASSERT_TRUE(w->pump([&] { return w->waiting(pid) && w->app.getPendingThreadCount() == 1; }, 5.0))
      << "the call's kernel wasn't queued";
  release_the_pool_failing(w->app, release);
  EXPECT_EQ(w->app.getPendingThreadCount(), 0u)
      << "the kernel waits for a worker nothing will start";
  // The host shuts down before the app goes, and the kernel doesn't hold it up.
  auto shut = std::async(std::launch::async, [&w] { w->host->shutdown(); });
  const bool hung = shut.wait_for(4s) == std::future_status::timeout;
  if (hung)
    start_a_worker(w->app); // the kernel runs, sees its call cancelled, and the shutdown ends
  EXPECT_FALSE(hung) << "the host's shutdown waits for a kernel no worker will run";
  ASSERT_EQ(shut.wait_for(10s), std::future_status::ready);
  const bool ended = pump_or_leak(w, [&] { return w->finished(pid); });
  if (!w)
    return;
  ASSERT_TRUE(ended) << "the call never ended";
  const auto exc = w->exit_exception(pid);
  ASSERT_TRUE(exc);
  const outcome o = record(*exc);
  EXPECT_EQ(o.cls, "RuntimeError");
  EXPECT_NE(o.message.find("slow-op: not run: no background worker could be started"),
            std::string::npos)
      << o.message;
  EXPECT_FALSE(w->p.started);
}

// launch_pool_task's task, left queued on a pool with no worker the same way, posts on_error's
// value: the parked process resumes with it rather than waiting for a later submission to start a
// worker.
TEST(AsyncCalls, APoolTaskLeftQueuedOnAPoolWithNoWorkerPostsOnErrorsValue) {
  std::atomic<bool> release(false), ran(false); // before the world, which the tasks outlive
  auto w = std::make_unique<world>(async_sched);
  async_scheduler &sched = *w->async;
  builtins::register_fn(w->env, "pool-double", [&](std::span<const value_t> a) -> value_t {
    const int64_t n = int_arg(a, 0, 0);
    return cvc::park_on_pool_task(
        w->app, sched, sched.current_process()->root_path, "pool-double.reply",
        [n, &ran] {
          ran = true;
          return value_t(n * 2);
        },
        [](const std::string &e) { return value_t("err: " + e); });
  });
  hold_the_pool(w->app, release);
  const int pid = w->submit("(begin (pool-double 21))"); // a top-level call can't park
  ASSERT_TRUE(w->pump([&] { return w->app.getPendingThreadCount() == 1; }, 5.0))
      << "the task wasn't queued";
  release_the_pool_failing(w->app, release);
  EXPECT_EQ(w->app.getPendingThreadCount(), 0u) << "the task waits for a worker nothing will start";
  const bool resumed = w->pump([&] { return w->finished(pid); }, 5.0);
  if (!resumed) {
    start_a_worker(w->app); // the task runs and posts its value
    w->pump([&] { return w->finished(pid); }, 5.0);
  }
  ASSERT_TRUE(resumed) << "the process never resumed";
  if (const auto exc = w->exit_exception(pid))
    FAIL() << "raised " << record(*exc).cls << ": " << record(*exc).message;
  ASSERT_TRUE(w->result(pid));
  EXPECT_EQ(show(*w->result(pid)), "err: not run: no background worker could be started");
  EXPECT_FALSE(ran.load());
}

// An async call queued behind a keyed thread that holds the only slot of a pool with no worker,
// and still runs when ~app begins, doesn't hold ~app for good. ~app cancels its own host's calls
// and waits for their kernels before it closes the pool, and interrupts the keyed thread only after
// that, but a pool with no worker can't run a queued kernel: ~app then closes the pool first,
// which drops the kernel and ends its call. (~app used to wait in the host's shutdown for a kernel
// nothing would run.)
TEST(AsyncCalls, DestroyingTheAppEndsACallQueuedOnAPoolWithNoWorker) {
  std::atomic<bool> started(false), release(false); // before the app, which the tasks outlive
  auto a = std::make_unique<stoppable_app>();
  environment_ptr env = builtins::make_default_environment();
  async_scheduler &sched = a->exec_scheduler();
  register_async_fn(env, a->exec_async_host(), "op",
                    [&started](std::span<const value_t>) -> async_kernel {
                      return [&started](async_call &) -> value_t {
                        started = true;
                        return value_t(int64_t(2));
                      };
                    });
  hold_the_pool(*a, release);
  execute_options opts;
  opts.env = env;
  const int pid = sched.execute(std::string("(op 1)"), opts);
  bool queued = false;
  for (const auto start = clock_type::now(); !queued && seconds_since(start) < 5.0;) {
    sched.drain_ingress();
    sched.sync_run(100000, 0.02);
    queued = a->getPendingThreadCount() == 1;
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(queued) << "the call's kernel wasn't queued";
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  EXPECT_EQ(a->getActiveThreadCount(), 1u) << "the keyed task holds the slot";
  stoppable_app *raw = a.get();
  auto destroyed = std::async(std::launch::async, [&a] { a.reset(); });
  const bool hung = destroyed.wait_for(4s) == std::future_status::timeout;
  EXPECT_FALSE(hung) << "~app waits in its host's shutdown for a kernel no worker will run";
  if (hung) {
    // So that the test ends: the pool is still open (~app is in the host's shutdown), and the
    // keyed task's end starts a worker, which runs the kernel; its call is cancelled.
    raw->fail_threads = false;
    release = true;
  }
  EXPECT_EQ(destroyed.wait_for(10s), std::future_status::ready);
  EXPECT_FALSE(started.load());
}

// ===========================================================================
// Work that runs by itself (docs/STATE_EXEC_EXCEPTIONS.md 3.12): an async_work's start, which the
// work completes later (a browser fetch's callback), and the kernel where the call can't park.
// ===========================================================================

namespace {

// The calls started by (self-op x) and what became of them: a test answers them (answer), as a
// browser's event loop answers a fetch, between drains.
struct started_probe {
  std::vector<std::shared_ptr<async_call>> pending; // started, not yet answered
  std::vector<int64_t> args;                        // their x, in order
  int starts = 0, kernels = 0, stopped = 0;
};

// (self-op x): 2x, started (the work answers later) where the call can park, else the kernel.
// (self-op-throws): a start that throws.
void register_started(world &w, started_probe &sp, async_options opts = {}) {
  register_async_fn(
      w.env, *w.host, "self-op",
      [&sp](std::span<const value_t> a) -> async_work {
        const int64_t x = int_arg(a, 0, 0);
        async_work work;
        work.kernel = [&sp, x](async_call &) -> value_t {
          ++sp.kernels;
          return value_t(x * 2);
        };
        work.start = [&sp, x](const std::shared_ptr<async_call> &call) {
          ++sp.starts;
          sp.pending.push_back(call);
          sp.args.push_back(x);
          call->on_cancel([&sp] { ++sp.stopped; });
        };
        return work;
      },
      opts);
  register_async_fn(
      w.env, *w.host, "self-op-throws",
      [](std::span<const value_t>) -> async_work {
        async_work work;
        work.start = [](const std::shared_ptr<async_call> &) {
          throw std::invalid_argument("self-op-throws: no transport");
        };
        return work;
      },
      opts);
}

// Answer the oldest pending call, as its work would: with 2x, or by raising `error`.
void answer(world &w, started_probe &sp, const char *error = nullptr) {
  ASSERT_FALSE(sp.pending.empty());
  std::shared_ptr<async_call> call = sp.pending.front();
  const int64_t x = sp.args.front();
  sp.pending.erase(sp.pending.begin());
  sp.args.erase(sp.args.begin());
  const bool settled = error ? call->settle({}, std::make_exception_ptr(std::runtime_error(error)))
                             : call->settle(value_t(x * 2), nullptr);
  if (settled)
    w.host->finish(call);
}

} // namespace

TEST(AsyncCalls, AStartedCallParksUntilItsWorkAnswers) {
  world w(async_sched);
  started_probe sp;
  register_started(w, sp);
  const int a = w.submit("(self-op 21)");
  const int c = w.submit("(begin (set i 0) (while (< i 2000) (set i (+ i 1))) i)");
  // Nothing answers: the caller parks, the other process runs to its end, no kernel runs.
  ASSERT_TRUE(w.pump([&] { return w.waiting(a) && w.finished(c); }));
  EXPECT_EQ(sp.starts, 1);
  EXPECT_EQ(sp.kernels, 0);
  EXPECT_EQ(w.async->async_calls_in_flight(), 1u);
  EXPECT_EQ(w.host->in_flight(), 0u) << "no executor runs it";
  for (int i = 0; i < 20; ++i) // more drains change nothing
    w.pump([] { return false; }, 0.005);
  EXPECT_TRUE(w.waiting(a));
  answer(w, sp);
  w.run_to_end(a);
  EXPECT_EQ(show(*w.result(a)), "42");
  EXPECT_EQ(w.async->async_calls_in_flight(), 0u);
  EXPECT_EQ(sp.stopped, 0) << "an answered call isn't stopped";
}

TEST(AsyncCalls, AStartedCallsFutureAwaitsWithATimeoutAsTheWorkGoesOn) {
  world w(async_sched);
  started_probe sp;
  register_started(w, sp);
  // cypkg's loop: poll the future in slices, counting the slices that time out, until it is done.
  const int pid = w.submit("(begin (set f (self-op-async 5)) (set slices 0) (set r 0)"
                           "  (while (= r 0)"
                           "    (set got (try (await f 0.01) (catch (TimeoutError e) 0)))"
                           "    (if (= got 0) (set slices (+ slices 1)) (set r got)))"
                           "  (list r (> slices 2) (await f)))");
  ASSERT_TRUE(w.pump([&] { return !sp.pending.empty(); }));
  w.pump([] { return false; }, 0.1); // several slices time out meanwhile
  EXPECT_FALSE(w.finished(pid));
  answer(w, sp);
  w.run_to_end(pid);
  ASSERT_TRUE(w.result(pid)) << (w.exit_exception(pid) ? exception_message(*w.exit_exception(pid))
                                                       : std::string());
  EXPECT_EQ(show(*w.result(pid)), "(10 #t 10)");
}

TEST(AsyncCalls, AStartedCallsErrorRaisesAtTheCall) {
  world w(async_sched);
  started_probe sp;
  register_started(w, sp);
  const int pid =
      w.submit("(try (self-op 1) (catch (RuntimeError e) (list (get-attr e \"function\") "
               "(get-attr e \"origin\") (get-attr e \"message\"))))");
  ASSERT_TRUE(w.pump([&] { return !sp.pending.empty(); }));
  answer(w, sp, "the network is down");
  w.run_to_end(pid);
  EXPECT_EQ(show(*w.result(pid)), "(\"self-op\" \"async\" \"the network is down\")");

  // A start that throws ends its call with that exception.
  const outcome o =
      run(w, "(try (self-op-throws) (catch (ValueError e) (get-attr e \"message\")))");
  EXPECT_EQ(o.value, "self-op-throws: no transport") << o.cls << ": " << o.message;
}

TEST(AsyncCalls, KillingAProcessStopsItsStartedCall) {
  world w(async_sched);
  started_probe sp;
  register_started(w, sp);
  const int pid = w.submit("(self-op 1)");
  ASSERT_TRUE(w.pump([&] { return w.waiting(pid) && !sp.pending.empty(); }));
  ASSERT_TRUE(w.async->kill(pid));
  EXPECT_EQ(sp.stopped, 1) << "the kill runs the call's cancel hook";
  EXPECT_EQ(w.async->async_calls_in_flight(), 0u);
  EXPECT_TRUE(sp.pending.front()->cancel_requested());
  w.async->drain_ingress();
  EXPECT_EQ(w.info(pid).status, process_status::killed);
  // A hook set on a cancelled call runs at once.
  int late = 0;
  sp.pending.front()->on_cancel([&late] { ++late; });
  EXPECT_EQ(late, 1);
}

TEST(AsyncCalls, ARegistrationTimeoutStopsAStartedCall) {
  async_options opts;
  opts.timeout = 0.05;
  world w(async_sched);
  started_probe sp;
  register_started(w, sp, opts);
  const auto start = clock_type::now();
  const outcome o = run(w, "(try (self-op 1) (catch (TimeoutError e) (get-attr e \"function\")))");
  EXPECT_EQ(o.value, "self-op") << o.cls << ": " << o.message;
  EXPECT_LT(seconds_since(start), 2.0);
  EXPECT_EQ(sp.stopped, 1);
}

TEST(AsyncCalls, HostShutdownStopsAStartedCall) {
  world w(async_sched);
  started_probe sp;
  register_started(w, sp);
  const int pid = w.submit("(self-op 1)");
  ASSERT_TRUE(w.pump([&] { return !sp.pending.empty(); }));
  w.host->shutdown();
  EXPECT_EQ(sp.stopped, 1);
}

TEST(AsyncCalls, WhereAStartedCallCantParkItsKernelRuns) {
  // No parking scheduler (the evaluators, the sync scheduler): the work's kernel, in the step.
  for_engines(recursive | stackless | sync_sched, [](unsigned e) {
    world w(e);
    started_probe sp;
    register_started(w, sp);
    EXPECT_EQ(run(w, "(self-op 4)").value, "8");
    EXPECT_EQ(sp.starts, 0);
    EXPECT_EQ(sp.kernels, 1);
  });
}

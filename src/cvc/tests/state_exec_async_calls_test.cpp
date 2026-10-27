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

// One engine's world: the app (its background pool and compute pool), a scheduler, the host
// and an environment with the kernels and intrinsics.
struct world {
  unsigned e;
  cvc::app app;
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
    EXPECT_GE(i.async_steps, 360000u); // 0.2 CPU s at the default 2000 steps per CPU ms
    EXPECT_LT(i.async_steps, 800000u);
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
    EXPECT_LT(i.async_steps, 60000u);
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
    EXPECT_GE(first, 80000); // 50 CPU ms at the default 2000 steps per CPU ms
    EXPECT_LT(first, 200000);
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
// NDEBUG: the mesher defines it for every build type. Nor a sanitizer's build, whose steps cost
// several times more.
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

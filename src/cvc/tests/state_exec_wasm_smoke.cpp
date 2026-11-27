// state_exec_wasm_smoke: with-lock where there are no threads (the Emscripten build without
// pthreads that ci.yml's ari-wasm job makes; locks plan L1.12, L3.6). A plain main(), no gtest: on
// the sync scheduler and on the async scheduler, a section's value, then a holder parked inside a
// section and a waiter granted once the holder lets go, then an empty lock-query. Then the same on
// an async scheduler attached to the app's lock manager as pycvc's Exec attaches
// (threaded_host_attach()): where there are no threads, auto mode makes that manager single and the
// attach is accepted. Then the app's exec_pool at threads 0, the only size without threads
// (threaded execution's Stage 1a): its pump_all pumps two groups inline, on this thread. Then a
// group attached to an app's pool at threads 0 (Stage 2a's continuous mode), which only its
// attaching thread can pump, driven by pump_host and wait_for_work until its program ends. It
// prints STATE_EXEC_WASM_OK and returns 0, or prints what failed and returns 1. Built only with
// CVC_BUILD_STATE_EXEC_WASM_SMOKE (it runs natively too, where threads are compiled in: there the
// single manager refuses that attach, naming state_exec.locks.threading, and the sections run on a
// same-thread attach; the pools are asked for 0 workers, so they have none there either).

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace se = cvc::state_exec;

namespace {

struct world {
  cvc::app app;
  std::unique_ptr<se::scheduler> sync;
  std::unique_ptr<se::async_scheduler> async;
  std::deque<se::intrinsics_context> contexts; // one per program, at stable addresses

  explicit world(bool use_sync) {
    if (use_sync)
      sync = std::make_unique<se::scheduler>();
    else
      async = std::make_unique<se::async_scheduler>();
  }
  se::scheduler_base &base() { return sync ? static_cast<se::scheduler_base &>(*sync) : *async; }

  int submit(const std::string &src) {
    se::intrinsics_context &ctx = contexts.emplace_back();
    ctx.sched = &base();
    ctx.root = &cvc::state::instance(app);
    ctx.proc = se::make_process();
    se::environment_ptr env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ctx);
    se::execute_options opts;
    opts.env = env;
    const int pid = base().execute(src, opts);
    ctx.pid = pid;
    ctx.proc->pid = pid;
    return pid;
  }

  // Step until `done()` or a bounded number of steps (there is no other thread to wait for).
  template <class Pred> bool step_until(Pred done) {
    for (int i = 0; i < 200000 && !done(); ++i) {
      if (sync) {
        sync->step();
      } else {
        async->drain_ingress();
        async->sync_step();
      }
    }
    return done();
  }

  se::process_status status(int pid) { return base().get_process_info(pid)->status; }
  std::string result(int pid) {
    const auto r = sync ? sync->get_result(pid) : async->get_result(pid);
    return r ? se::to_string(*r) : std::string("<no result>");
  }
};

bool expect(bool ok, const char *kind, const std::string &what) {
  if (!ok)
    std::printf("state_exec_wasm_smoke: %s: %s\n", kind, what.c_str());
  return ok;
}

// A section's value; a holder parked in a section, and a waiter granted when it lets go; then an
// empty lock-query.
bool sections(world &w, const char *kind) {
  bool ok = true;
  const int a = w.submit(R"((with-lock "k" (+ 1 2)))");
  ok &= expect(w.step_until([&] { return se::is_finished(w.status(a)); }), kind,
               "the uncontended section did not finish");
  ok &= expect(w.result(a) == "3", kind, "the section's value is " + w.result(a));

  const int holder = w.submit(R"((with-lock "k" (msg-recv "go")))");
  ok &= expect(w.step_until([&] { return w.status(holder) == se::process_status::waiting; }), kind,
               "the holder did not park");
  const int waiter = w.submit(R"((with-lock "k" 7))");
  ok &= expect(w.step_until([&] { return w.status(waiter) == se::process_status::waiting; }), kind,
               "the waiter did not wait");
  w.submit(R"((msg-send "go" "x"))");
  ok &= expect(w.step_until([&] {
    return se::is_finished(w.status(holder)) && se::is_finished(w.status(waiter));
  }),
               kind, "the holder and the waiter did not finish");
  ok &= expect(w.result(waiter) == "7", kind, "the waiter's value is " + w.result(waiter));

  const int query = w.submit(R"((lock-query))");
  ok &= expect(w.step_until([&] { return se::is_finished(w.status(query)); }), kind,
               "lock-query did not finish");
  ok &= expect(w.result(query) == R"({"held": (), "waiting": ()})", kind,
               "lock-query left " + w.result(query));
  return ok;
}

bool run(bool use_sync) {
  world w(use_sync);
  return sections(w, use_sync ? "sync" : "async");
}

// The app's lock manager, attached as pycvc's Exec attaches (locks plan L3.6; gap G-24).
bool run_on_app_manager() {
  const char *kind = "app manager";
  bool ok = true;
#if defined(__EMSCRIPTEN__) || defined(__wasi__)
  // ari-wasm's build has no pthreads, and wasip1 none at all (publish-cvcgl-wasm.yml's
  // libcvc-wasi job runs this under wasmtime): the checks below are the no-threads ones only then.
  ok &= expect(!se::threads_compiled_in, kind, "threads_compiled_in is true (pthreads?)");
#endif
  world w(false);
  const std::shared_ptr<se::lock_manager> mgr = w.app.lock_manager();
  ok &= expect(mgr->threading() == se::lock_threading::single, kind,
               "app::lock_manager()'s auto mode is not single");
  try {
    w.async->set_lock_manager(mgr, se::threaded_host_attach());
    ok &= expect(!se::threads_compiled_in, kind, "the single manager took a cross-thread attach");
  } catch (const std::logic_error &e) {
    // Threads are compiled in (a native run): the single manager refuses the attach, naming the
    // setting that allows it. The sections then run attached on this thread.
    const std::string what = e.what();
    ok &= expect(se::threads_compiled_in, kind, "the attach was refused: " + what);
    ok &= expect(what.find("state_exec.locks.threading") != std::string::npos, kind,
                 "the refusal does not name state_exec.locks.threading: " + what);
    w.async->set_lock_manager(mgr);
  }
  ok &= sections(w, kind);
  // The sections ran on the app's manager: the waiter queued there, and nothing is left.
  const se::lock_stats s = mgr->stats();
  ok &= expect(s.waits == 1, kind, "the app manager queued " + std::to_string(s.waits) + " waits");
  ok &= expect(s.holds == 0 && s.queued == 0, kind, "the app manager is not empty");
  return ok;
}

// The app's exec_pool at threads 0 (S1.15): two groups on the app, each with a program, pumped
// by pump_all rounds of 1000 steps until both have ended, every pump on this thread.
bool run_exec_pool() {
  const char *kind = "exec_pool";
  bool ok = true;
  cvc::app app;
  cvc::exec_pool &pool = app.exec_pool(cvc::pool_options{});
  ok &= expect(pool.threads() == 0, kind,
               "the pool has " + std::to_string(pool.threads()) + " workers, not 0");
  se::exec_group::options oa, ob;
  oa.name = "wasm.a";
  oa.app = &app;
  ob.name = "wasm.b";
  ob.app = &app;
  se::exec_group a(std::move(oa)), b(std::move(ob));
  const int pa = a.execute(
      std::string("(begin (set s 0) (set i 0) (while (< i 100) (begin (set s (+ s i)) (set i (+ i "
                  "1)))) s)"));
  const int pb = b.execute(std::string("(* 6 7)"));
  const auto ended = [](se::exec_group &g, int pid) {
    const auto info = g.sched().get_process_info(pid);
    return info && se::is_finished(info->status);
  };
  const auto result = [](se::exec_group &g, int pid) {
    const auto r = g.sched().get_result(pid);
    return r ? se::to_string(*r) : std::string("<no result>");
  };
  const std::uint64_t worker_pumps = pool.stats().worker_pumps;
  se::exec_group *const groups[] = {&a, &b};
  int rounds = 0;
  for (; rounds < 1000 && !(ended(a, pa) && ended(b, pb)); ++rounds)
    pool.pump_all(groups, se::pump_budget{1000, std::numeric_limits<double>::infinity()},
                  std::chrono::steady_clock::time_point::max());
  ok &= expect(ended(a, pa) && ended(b, pb), kind, "the groups' programs did not end");
  ok &= expect(rounds > 1, kind, "the loop ended in one 1000-step pump");
  ok &= expect(result(a, pa) == "4950", kind, "group a returned " + result(a, pa));
  ok &= expect(result(b, pb) == "42", kind, "group b returned " + result(b, pb));
  ok &= expect(pool.stats().worker_pumps == worker_pumps, kind, "a worker pumped a group");
  ok &= expect(pool.stats().pumps >= std::uint64_t(rounds) + 1, kind,
               "the pool counted " + std::to_string(pool.stats().pumps) + " pumps in " +
                   std::to_string(rounds) + " rounds");
  return ok;
}

// The app's exec_pool at threads 0 in continuous mode (S2.md section 5; exec_pool.h): a group
// attached by this thread is one only this thread pumps (no worker exists here: threads_compiled_in
// is false, and the pool has none natively either), in pump_host, with wait_for_work while nothing
// is queued, in pumps of 200 steps until its program ends. It is read only under its token
// (with_group), every pump ran on this thread (host_pumps, no worker pump), and it detaches.
bool run_continuous() {
  const char *kind = "continuous";
  bool ok = true;
  cvc::app app;
  cvc::pool_options po;
  po.pump_steps = 200;
  po.pump_seconds = std::numeric_limits<double>::infinity();
  cvc::exec_pool &pool = app.exec_pool(po);
  ok &= expect(pool.threads() == 0, kind,
               "the pool has " + std::to_string(pool.threads()) + " workers, not 0");
  se::exec_group::options o;
  o.name = "wasm.c";
  o.app = &app;
  se::exec_group g(std::move(o));
  const int pid = g.execute(
      std::string("(begin (set s 0) (set i 0) (while (< i 100) (begin (set s (+ s i)) (set i (+ i "
                  "1)))) s)"));
  // Detached however this ends, before the group goes.
  struct attachment {
    cvc::exec_pool &pool;
    se::exec_group &g;
    attachment(cvc::exec_pool &p, se::exec_group &group) : pool(p), g(group) { pool.attach(g); }
    ~attachment() {
      if (pool.attached(g))
        pool.detach(g);
    }
  };
  const cvc::pool_stats s0 = pool.stats();
  bool done = false;
  std::string got;
  {
    const attachment attached(pool, g);
    for (int turn = 0; turn < 10000 && !done; ++turn) {
      if (!pool.pump_host(std::numeric_limits<double>::infinity()))
        pool.wait_for_work(0.01);
      pool.with_group(g, [&] {
        const auto info = g.sched().get_process_info(pid);
        done = info && se::is_finished(info->status);
      });
    }
    pool.with_group(g, [&] {
      const auto r = g.sched().get_result(pid);
      got = r ? se::to_string(*r) : std::string("<no result>");
    });
    pool.detach(g);
    ok &= expect(!pool.attached(g), kind, "the group is still attached after detach");
  }
  const cvc::pool_stats s1 = pool.stats();
  const std::uint64_t pumps = s1.pumps - s0.pumps, here = s1.host_pumps - s0.host_pumps;
  ok &= expect(done, kind, "the attached group's program did not end");
  ok &= expect(got == "4950", kind, "the attached group returned " + got);
  ok &= expect(s1.worker_pumps == s0.worker_pumps, kind, "a worker pumped the group");
  ok &= expect(here >= 2, kind,
               std::to_string(here) + " pumps on this thread, not several of 200 steps");
  ok &= expect(here == pumps, kind,
               std::to_string(pumps - here) + " of " + std::to_string(pumps) +
                   " pumps ran off this thread");
  return ok;
}

} // namespace

int main() {
  try {
    const bool ok =
        run(true) && run(false) && run_on_app_manager() && run_exec_pool() && run_continuous();
    if (!ok)
      return 1;
    std::printf("STATE_EXEC_WASM_OK\n");
    return 0;
  } catch (const std::exception &e) {
    std::printf("state_exec_wasm_smoke: %s\n", e.what());
    return 1;
  }
}

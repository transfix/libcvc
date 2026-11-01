// state_exec_wasm_smoke: with-lock where there are no threads (the Emscripten build without
// pthreads that ci.yml's ari-wasm job makes; locks plan L1.12, L3.6). A plain main(), no gtest: on
// the sync scheduler and on the async scheduler, a section's value, then a holder parked inside a
// section and a waiter granted once the holder lets go, then an empty lock-query. Then the same on
// an async scheduler attached to the app's lock manager as pycvc's Exec attaches
// (threaded_host_attach()): where there are no threads, auto mode makes that manager single and the
// attach is accepted. It prints STATE_EXEC_WASM_OK and returns 0, or prints what failed and
// returns 1. Built only with CVC_BUILD_STATE_EXEC_WASM_SMOKE (it runs natively too, where threads
// are compiled in: there the single manager refuses that attach, naming
// state_exec.locks.threading, and the sections run on a same-thread attach).

#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <exception>
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
#ifdef __EMSCRIPTEN__
  // ari-wasm's build has no pthreads: the checks below are the no-threads ones only then.
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

} // namespace

int main() {
  try {
    const bool ok = run(true) && run(false) && run_on_app_manager();
    if (!ok)
      return 1;
    std::printf("STATE_EXEC_WASM_OK\n");
    return 0;
  } catch (const std::exception &e) {
    std::printf("state_exec_wasm_smoke: %s\n", e.what());
    return 1;
  }
}

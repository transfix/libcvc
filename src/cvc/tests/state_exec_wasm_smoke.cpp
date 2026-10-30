// state_exec_wasm_smoke: with-lock where there are no threads (the Emscripten build without
// pthreads that ci.yml's ari-wasm job makes; locks plan L1.12). A plain main(), no gtest: on the
// sync scheduler and on the async scheduler, a section's value, then a holder parked inside a
// section and a waiter granted once the holder lets go, then an empty lock-query. It prints
// STATE_EXEC_WASM_OK and returns 0, or prints what failed and returns 1. Built only with
// CVC_BUILD_STATE_EXEC_WASM_SMOKE (it runs natively too).

#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <exception>
#include <memory>
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

bool run(bool use_sync) {
  const char *kind = use_sync ? "sync" : "async";
  world w(use_sync);
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

} // namespace

int main() {
  try {
    const bool ok = run(true) && run(false);
    if (!ok)
      return 1;
    std::printf("STATE_EXEC_WASM_OK\n");
    return 0;
  } catch (const std::exception &e) {
    std::printf("state_exec_wasm_smoke: %s\n", e.what());
    return 1;
  }
}

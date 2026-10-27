// cvc::async_task — offload a blocking kernel to the compute pool and resume a parked state_exec
// process when it finishes.  See inc/cvc/core/async_task.h for the contract and the two rules.

#include <atomic>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/async_task.h>
#include <cvc/state/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/state/state_exec/intrinsics.h> // resolve_channel_key, make_future, park_on_channel
#include <exception>
#include <utility>

namespace cvc {

namespace se = state_exec;

namespace {

// launch_pool_task's task, shared by the copies of its pool task: it posts exactly one value to
// `done`, the work's or on_error's, also when the pool destroys it unrun (it was submitted, or
// still queued, once ~app had stopped the pool).
struct pool_task_post {
  se::async_scheduler &sched;
  pool_task_work work;
  pool_task_on_error on_error;
  std::string done;
  bool ran = false;

  pool_task_post(se::async_scheduler &s, pool_task_work w, pool_task_on_error e, std::string d)
      : sched(s), work(std::move(w)), on_error(std::move(e)), done(std::move(d)) {}
  pool_task_post(const pool_task_post &) = delete;
  pool_task_post &operator=(const pool_task_post &) = delete;

  void run() {
    ran = true;
    se::value_t payload;
    try {
      payload = work();
    } catch (const std::exception &e) {
      payload = on_error(e.what());
    } catch (...) {
      payload = on_error("unknown error");
    }
    sched.post_message(done, payload);
  }

  ~pool_task_post() {
    if (ran)
      return;
    const std::string msg = "not run: the app's background pool has stopped";
    try {
      se::value_t payload;
      try {
        payload = on_error(msg);
      } catch (...) {
        payload = se::value_t(msg); // on_error failed too: the waiter still gets a value
      }
      sched.post_message(done, payload);
    } catch (...) {
    }
  }
};

} // namespace

std::string launch_pool_task(app &a, se::async_scheduler &sched, const std::string &root,
                             const std::string &chan_prefix, pool_task_work work,
                             pool_task_on_error on_error) {
  // A UNIQUE reply channel per call so two in-flight tasks never collide on the single recv_path a
  // process has.  The '#' makes it policy-exempt + identity under resolve_channel_key, so it
  // survives channel scoping verbatim (the same trick the http/fetch launchers use).
  static std::atomic<std::uint64_t> seq{0};
  const std::string chan =
      chan_prefix + "#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  const std::string done = se::resolve_channel_key(root, chan);
  // submitPooled returns immediately; the task runs on one of the app's background-pool workers
  // (no thread of its own; it used to go through compute_async(1, ...), which also built the
  // compute pool for a fan-out of one).  post_message is the thread-safe ingress `sched` drains on
  // its own driver thread, so the worker never touches the scheduler's internals — and it posts to
  // the SAME scheduler the caller drives, so the parked process is resumed there.  ALWAYS post
  // exactly one value (on_error on a throw, or when the app drops the task unrun as it goes away)
  // so a waiter can't hang.
  auto task = std::make_shared<pool_task_post>(sched, std::move(work), std::move(on_error), done);
  // The pool task owns it: refused, it posts from its destructor before submitPooled returns.
  a.submitPooled([task = std::move(task)] { task->run(); });
  return done;
}

se::value_t park_on_pool_task(app &a, se::async_scheduler &sched, const std::string &root,
                              const std::string &chan_prefix, pool_task_work work,
                              pool_task_on_error on_error) {
  const std::string done =
      launch_pool_task(a, sched, root, chan_prefix, std::move(work), std::move(on_error));
  return se::park_on_channel(&sched, sched.current_process().get(), sched.current_pid(), done);
}

se::value_t future_pool_task(app &a, se::async_scheduler &sched, const std::string &root,
                             const std::string &chan_prefix, pool_task_work work,
                             pool_task_on_error on_error) {
  return se::make_future(
      launch_pool_task(a, sched, root, chan_prefix, std::move(work), std::move(on_error)));
}

} // namespace cvc

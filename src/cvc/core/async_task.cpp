// cvc::async_task — offload a blocking kernel to the compute pool and resume a parked state_exec
// process when it finishes.  See inc/cvc/core/async_task.h for the contract and the two rules.

#include <cvc/core/async_task.h>

#ifdef CVC_STATE_EXEC

#include <atomic>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/intrinsics.h> // resolve_channel_key, make_future, park_on_channel
#include <exception>
#include <utility>

namespace cvc {

namespace se = state_exec;

std::string launch_pool_task(app &a, const std::string &root, const std::string &chan_prefix,
                             pool_task_work work, pool_task_on_error on_error) {
  // A UNIQUE reply channel per call so two in-flight tasks never collide on the single recv_path a
  // process has.  The '#' makes it policy-exempt + identity under resolve_channel_key, so it
  // survives channel scoping verbatim (the same trick the http/fetch launchers use).
  static std::atomic<std::uint64_t> seq{0};
  const std::string chan =
      chan_prefix + "#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  const std::string done = se::resolve_channel_key(root, chan);
  // compute_async returns immediately; the completion lambda runs on a pool worker.  post_message
  // is the thread-safe ingress the scheduler drains on its own thread, so the worker never touches
  // the scheduler's internals.  ALWAYS post exactly one value (on_error on a throw) so a waiter
  // can't hang.
  a.compute_async(
      1, [](int) {},
      [&a, work = std::move(work), on_error = std::move(on_error), done] {
        se::value_t payload;
        try {
          payload = work();
        } catch (const std::exception &e) {
          payload = on_error(e.what());
        } catch (...) {
          payload = on_error("unknown error");
        }
        a.exec_scheduler().post_message(done, payload);
      });
  return done;
}

se::value_t park_on_pool_task(app &a, const std::string &root, const std::string &chan_prefix,
                              pool_task_work work, pool_task_on_error on_error) {
  const std::string done =
      launch_pool_task(a, root, chan_prefix, std::move(work), std::move(on_error));
  se::async_scheduler &sched = a.exec_scheduler();
  return se::park_on_channel(&sched, sched.current_process().get(), sched.current_pid(), done);
}

se::value_t future_pool_task(app &a, const std::string &root, const std::string &chan_prefix,
                             pool_task_work work, pool_task_on_error on_error) {
  return se::make_future(
      launch_pool_task(a, root, chan_prefix, std::move(work), std::move(on_error)));
}

} // namespace cvc

#endif // CVC_STATE_EXEC

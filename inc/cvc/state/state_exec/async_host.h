/**
 * @file async_host.h
 * @brief Registering long-running native calls (docs/STATE_EXEC_EXCEPTIONS.md section 3).
 *
 * An async_host is what a host provides for async functions: the executors kernels run on
 * (the app's background task pool, named lanes, or inline), the scheduler whose processes park
 * while they run, and the calls in flight, so that teardown cancels them and waits for their
 * kernels. app::exec_async_host() is the one for the app's exec scheduler (Ariadne); pycvc's
 * Exec builds one for its private scheduler.
 *
 *   se::register_async_fn(env, app.exec_async_host(), "volume-blur",
 *     [](std::span<const se::value_t> args) -> se::async_kernel {
 *       auto in = marshal_in(args); // the arguments are private copies already
 *       return [in](se::async_call &call) {
 *         return marshal_out(run_blur(in, [&] { return call.cancelled(); }));
 *       };
 *     });
 *
 * (volume-blur v 3) then parks the calling process and resumes it with the kernel's value, or
 * raises the kernel's exception at the call; (volume-blur-async v 3) returns a future for
 * (await f [seconds]) or (await-all (list f ...)).
 */
#ifndef CVC_STATE_EXEC_ASYNC_HOST_H
#define CVC_STATE_EXEC_ASYNC_HOST_H

#include <cstddef>
#include <cvc/state/state_exec/async_call.h>
#include <cvc/state/state_exec/types.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace cvc {
class app;
class async_lane;
} // namespace cvc

namespace cvc::state_exec {

class async_scheduler;
class scheduler_base;

/// The executors, scheduler and in-flight calls of a host's async functions.
class async_host {
public:
  /// A host whose kernels run inline, in the step that calls them, charged to the process
  /// `sched` is stepping, if any (with none, a bare evaluator's: its steps only).
  explicit async_host(scheduler_base *sched = nullptr);
  /// A host for `sched`, whose calls park their process while the kernel runs on `app`'s
  /// background task pool or on a lane (add_lane). Without an app (null), background kernels
  /// run inline.
  async_host(async_scheduler &sched, cvc::app *app);
  /// shutdown(). Destroy the host before the scheduler it posts to.
  ~async_host();

  async_host(const async_host &) = delete;
  async_host &operator=(const async_host &) = delete;

  /// Make `lane` the executor of functions registered with executor::on_lane(name). It must
  /// outlive the host's calls.
  void add_lane(const std::string &name, cvc::async_lane &lane);

  /// Cancel every call in flight and wait until its kernel has ended (a queued one ends at
  /// once). Calls made afterwards raise RuntimeError. Idempotent.
  void shutdown();
  /// Wait, without cancelling anything, until no kernel is queued or running.
  void wait_idle();
  /// Kernels queued or running.
  std::size_t in_flight() const;

  /// The scheduler whose processes the calls are charged to (null: none).
  scheduler_base *scheduler() const { return sched_; }
  /// The scheduler calls park on (null: kernels run inline).
  async_scheduler *parking_scheduler() const { return async_sched_; }

  /// Call `kernel` as `name`, from the native function the evaluator is applying. Transparent
  /// (`future` false): park the calling process until the kernel is done and resume it with
  /// the value, or raise the kernel's exception at the call. Future: return a future at once.
  /// Where the process can't park, or the executor is inline, the kernel runs here, now.
  /// Raises RuntimeError where parking is forbidden (park_forbidden_scope).
  value_t call(const std::string &name, async_kernel kernel, const async_options &opts,
               bool future);

  /// A call whose outcome another part of the host supplies, by settle() and finish()
  /// (pycvc's coroutines, run by the Exec's own event loop): park the calling process on it
  /// (or, `future`, return a future). Its run time isn't charged (it runs on no executor
  /// thread of the host's). Throws RuntimeError where the process can't park.
  std::shared_ptr<async_call> begin_external(const std::string &name, bool future, value_t &result);
  /// Post the outcome of an external call that has been settled; thread-safe.
  void finish(const std::shared_ptr<async_call> &call);

private:
  struct shared_state;

  value_t run_inline(const std::shared_ptr<async_call> &call, const async_kernel &kernel,
                     process *proc, bool future);
  // On the executor thread: run the kernel, settle the call, post its completion.
  static void run_kernel(std::shared_ptr<async_call> call, std::shared_ptr<async_kernel> holder,
                         std::shared_ptr<shared_state> state) noexcept;
  void submit(const std::shared_ptr<async_call> &call, async_kernel kernel,
              const async_options &opts);
  bool runs_threaded(const async_options &opts) const;
  void check_open(const std::string &name) const;

  std::shared_ptr<shared_state> state_;
  scheduler_base *sched_ = nullptr;
  async_scheduler *async_sched_ = nullptr;
  cvc::app *app_ = nullptr;
  std::unordered_map<std::string, cvc::async_lane *> lanes_;
};

/// Register `name` in `env` as an async function (docs/STATE_EXEC_EXCEPTIONS.md 3.2):
/// (name args...) copies its arguments' DSL lists and dicts (as deep-data work, counted against
/// the process's budget), runs `marshal` on them on the scheduler thread, and calls the kernel
/// it returns through `host` (async_host::call). With opts.future_variant, `<name>-async` too,
/// which returns a future. A kernel that throws raises at the call site: the exception mapped
/// by its C++ type, with origin "async" and `function` set; a timeout raises TimeoutError.
/// `host` must outlive every call of the functions.
void register_async_fn(environment_ptr env, async_host &host, const std::string &name,
                       async_marshal marshal, async_options opts = {});

/// A future of `call`: {"__future__": its channel, "__async_call__": a handle to it}, which
/// (await f) reads directly.
value_t make_async_future(const std::shared_ptr<async_call> &call);

/// The async call a future holds, or null (not an async call's future, or one decoded from a
/// snapshot: its call can't resume).
std::shared_ptr<detail::awaitable> async_call_of_future(const value_t &future);

/// Whether `v` is a future that held an async call (its "__async_call__" key), live or not.
bool is_async_future(const value_t &v);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_HOST_H

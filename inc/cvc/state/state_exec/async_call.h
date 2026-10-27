/**
 * @file async_call.h
 * @brief Long-running native calls (docs/STATE_EXEC_EXCEPTIONS.md section 3): the handle a
 *        kernel runs with, where it runs, and what a call is charged.
 *
 * An async function (register_async_fn, async_host.h) is a native function whose work, its
 * kernel, runs on a host executor instead of in the evaluator step that called it. The calling
 * process parks at the call while the kernel runs and resumes with its value, or raises its
 * exception there. The kernel gets an async_call: it checks cancelled() at convenient points
 * (which also records the CPU time it has used, for the process's budget) and may post()
 * progress values to the program.
 */
#ifndef CVC_STATE_EXEC_ASYNC_CALL_H
#define CVC_STATE_EXEC_ASYNC_CALL_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cvc/core/thread_pool.h> // fanout_context: a kernel's parallel_for is charged to it
#include <cvc/core/types.h>       // thread_priority
#include <cvc/state/state_exec/stackless_evaluator.h> // detail::awaitable
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>

namespace cvc::state_exec {

class async_call;
class async_host;
class async_scheduler;
class scheduler_base;
struct process;

/// An async function's work. It runs on an executor thread (or inline, see executor_kind),
/// with the arguments its marshal step copied. It returns the call's value or throws its error;
/// it must not touch the state tree, the evaluator or any DSL value the program holds, and its
/// value must be fresh (docs/STATE_EXEC_EXCEPTIONS.md 3.7). Report large allocations with
/// eval_alloc_check(): the process's allocation limit holds on the kernel's thread and in its
/// computePool().parallel_for chunks, and what it reports counts toward the process's heap.
using async_kernel = std::function<value_t(async_call &call)>;

/// An async function's marshal step: runs on the scheduler's thread, in the calling step, with
/// the call's arguments, and returns the kernel to run. The arguments are already private copies
/// (DSL lists and dicts are copied before it runs, data_objects shared), so the kernel may keep
/// them; a marshal that throws raises at the call like any native function.
using async_marshal = std::function<async_kernel(std::span<const value_t> args)>;

/// Where an async function's kernel runs (docs/STATE_EXEC_EXCEPTIONS.md 3.5).
enum class executor_kind {
  /// The app's background task pool (app::submitPooled: its persistent workers), in the order
  /// of its priorities, FIFO within one: for every kernel, CPU-bound or blocking. A host without
  /// an app runs it inline.
  background,
  /// A named async_lane the host was given (async_host::add_lane): work that must stay on one
  /// thread.
  lane,
  /// On the thread that calls the function, in its step, which blocks meanwhile. Also what any
  /// call does whose process can't park: on the sync scheduler, a bare evaluator, a host with
  /// no executor.
  inline_call,
};

/// An executor: its kind and, for a lane, the lane's name.
struct executor {
  executor_kind kind = executor_kind::background;
  std::string lane;

  static executor background() { return {}; }
  static executor on_lane(std::string name) { return {executor_kind::lane, std::move(name)}; }
  static executor inline_call() { return {executor_kind::inline_call, {}}; }
};

/// How an async function is registered.
struct async_options {
  executor where;                             ///< the background pool by default
  thread_priority priority = PRIORITY_NORMAL; ///< its place in the background pool's queue
  /// Seconds from the call; past it the call raises TimeoutError at the call site, the kernel
  /// sees cancelled(), and its result is dropped.
  std::optional<double> timeout;
  bool future_variant = true; ///< also register "<name>-async", which returns a future
};

/// One call of an async function: the kernel's handle, and what the host and the scheduler keep
/// about the call while it runs.
class async_call : public detail::awaitable, public std::enable_shared_from_this<async_call> {
public:
  using clock = std::chrono::steady_clock;

  // ---- for kernels ----

  /// The function's name.
  const std::string &function_name() const override { return name_; }

  /// Whether the kernel should stop: its process was killed or reaped (its budget ran out
  /// while the call ran included), the call timed out, its owner group was reaped, or the host
  /// is being torn down. A kernel that stops early throws (or returns); its result is dropped.
  /// Each call on the kernel's own thread also records the CPU time the thread has used, which
  /// the host charges the process as the call runs.
  bool cancelled();

  /// Send a progress value to the channel `<name>.progress` under the process's root, which
  /// the program reads with (msg-recv ...). Nothing waits for it, and a full queue drops it.
  /// The value must be fresh, like the kernel's result.
  void post(const value_t &progress);

  /// Time the kernel spent waiting for something the host provides (a Python kernel waiting to
  /// take the GIL): not charged as run time (docs/STATE_EXEC_EXCEPTIONS.md 3.8).
  void add_wait(double seconds);

  // ---- awaitable: what an await_call frame or a future reads ----

  bool finished() const override { return finished_.load(std::memory_order_acquire); }
  bool wait(std::optional<double> seconds) override;
  /// The kernel's value, or throws its exception as the DSL exception it raises at the call
  /// site: mapped by its C++ type (exception_from), with origin "async" and `function` set, and
  /// the C++ exception as the dsl_exception's original() (a Python exception re-raises).
  value_t outcome() override;

  // ---- the host's and the schedulers' (async_host.cpp, the schedulers) ----

  async_call(std::string name, int pid);
  ~async_call() override;

  int pid() const { return pid_; }
  /// The channel a process waiting for the call waits on (its recv_path).
  const std::string &channel() const { return channel_; }
  bool threaded() const { return async_sched_ != nullptr; }
  bool started() const { return start_ns_.load(std::memory_order_acquire) != 0; }

  /// Cancel: cancelled() is true from now on.
  void cancel() { cancel_.store(true, std::memory_order_release); }
  bool cancel_requested() const { return cancel_.load(std::memory_order_acquire); }
  /// Whether the call's timeout has passed at `now`.
  bool past_timeout(clock::time_point now) const;

  /// Settle the call: its outcome is the kernel's value or `error`. Only the first settling
  /// counts (a timeout or a cancellation may come first); returns whether this one did.
  bool settle(value_t value, std::exception_ptr error);
  /// Settle the call as timed out (TimeoutError), unless it is settled already.
  bool settle_timeout();

  /// What the scheduler has charged the process for the call so far (charge_async_call).
  struct charges {
    double run = 0.0;          // run time
    double past_half = 0.0;    // run time spent while the process was past max_time / 2
    double cpu_to_time = 0.0;  // CPU time charged to cpu_time_charged
    double cpu_reported = 0.0; // CPU time added to async_cpu_time
    double cpu_stepped = 0.0;  // CPU time converted to steps
    double step_fraction = 0.0;
    uint64_t steps = 0;
    bool closed = false; // the final charge is made
  };
  charges charged;

  /// The kernel's run time so far: from when it started on its executor thread to when it
  /// ended (or `now`), less what it spent waiting (add_wait). 0 while queued.
  double run_seconds(clock::time_point now) const;
  /// The CPU time recorded so far: the kernel thread's, as of its last meter (or its end), and
  /// its fan-out chunks'.
  double cpu_seconds() const;
  /// Run time since the kernel thread's CPU time was last recorded: a kernel that never calls
  /// cancelled() is charged that as CPU time until it ends (docs/STATE_EXEC_EXCEPTIONS.md 3.8).
  double unmetered_seconds(clock::time_point now) const;
  /// Bytes the kernel and its fan-out chunks reported allocating (eval_alloc_check, ...).
  uint64_t alloc_bytes() const { return alloc_bytes_.load(std::memory_order_acquire); }
  /// Whether the kernel has ended (its run time and CPU time are final).
  bool kernel_ended() const { return end_ns_.load(std::memory_order_acquire) != 0; }

private:
  friend class async_host;
  friend class async_scheduler;

  // The fan-out context the kernel's computePool().parallel_for chunks run under: charged to
  // the call, under its allocation limit.
  struct chunk_context final : cvc::fanout_context {
    async_call *call = nullptr;
    void run_chunks(const std::function<void()> &chunks) noexcept override;
  };

  void begin_kernel();       // on the kernel's thread, as it starts
  void end_kernel();         // on the kernel's thread, as it ends
  void meter();              // record the kernel thread's CPU time so far
  bool over_budget_inline(); // an inline call: whether its process's budget is spent

  std::string name_;
  int pid_;
  std::string channel_; // the reply channel (threaded calls)
  // "<name>.progress" resolved under the calling process's root: built on the first post()
  // (most kernels never post), from the root path kept here (the process may be gone by then).
  bool has_progress_ = false; // a process to post to (not a bare evaluator)
  std::string progress_root_;
  std::once_flag progress_once_;
  std::string progress_channel_;
  scheduler_base *sched_ = nullptr;        // where progress goes (inline: on this thread)
  async_scheduler *async_sched_ = nullptr; // threaded: where the completion is posted
  std::optional<double> timeout_;
  clock::time_point created_ = clock::now();
  std::optional<std::size_t> alloc_limit_; // the calling step's (eval_alloc_limit())
  chunk_context chunks_;

  // An external call (async_host::begin_external): the host's own loop settles it, so nothing
  // may block waiting for it.
  bool external_ = false;
  // Its index in its host's list of calls to cancel at shutdown (guarded by the host's lock).
  static constexpr std::size_t kNoSlot = static_cast<std::size_t>(-1);
  std::size_t host_slot_ = kNoSlot;
  // Inline calls: the process running it, for cancelled() to check its budget, and the step
  // rate that check uses (read on first need).
  process *inline_proc_ = nullptr;
  std::optional<double> inline_rate_;

  std::atomic<bool> cancel_{false};
  std::thread::id kernel_thread_;
  double cpu0_ = 0.0; // the kernel thread's CPU clock when the kernel started
  // Steady-clock and CPU nanoseconds, written on the kernel's threads, read by the scheduler.
  std::atomic<int64_t> start_ns_{0};
  std::atomic<int64_t> end_ns_{0};
  std::atomic<int64_t> metered_ns_{0}; // when the kernel thread's CPU time was last recorded
  std::atomic<int64_t> cpu_ns_{0};     // the kernel thread's CPU time then
  std::atomic<int64_t> fanout_cpu_ns_{0};
  std::atomic<int64_t> wait_ns_{0};
  std::atomic<uint64_t> alloc_bytes_{0};

  mutable std::mutex m_;
  std::condition_variable settled_cv_;
  bool settled_ = false;
  bool timed_out_ = false;
  value_t value_;
  std::exception_ptr error_;
  std::optional<value_t> exception_; // error_ as a DSL exception, made once
  std::atomic<bool> finished_{false};
};

namespace detail {

/// Charge `proc` for `call` up to `now`: its run time (to accumulated_time, async_time), its CPU
/// time (async_cpu_time, and cpu_time_charged by the share of its run time past max_time's
/// halfway point, the unmetered tail counted as CPU until it ends), and its steps at `rate` per
/// CPU millisecond (step_kind::async, async_steps). `final`: the kernel has ended or the call
/// is done for; the true CPU time replaces the estimate, and a call charged no step is charged
/// one. `inline_step`: the call ran inside the process's current step.
void charge_async_call(process &proc, async_call &call, double rate, bool final,
                       async_call::clock::time_point now, bool inline_step = false);

} // namespace detail

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_CALL_H

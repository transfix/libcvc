/**
 * @file async_lane.h
 * @brief A persistent, named, FIFO worker "lane" plus the state_exec glue that lets a resident DSL
 *        process wait on a lane's results without ever blocking the thread that pumps it.
 *
 * WHAT A LANE IS
 *   One long-lived worker thread draining a FIFO of std::function<void()> jobs, in submission
 *   order, one at a time. It is the persistent counterpart of the per-call compute-pool offload in
 *   async_task.h: launch_pool_task starts a fresh pool thread per call (fine for an occasional HTTP
 *   request or volume filter, ruinous for a per-frame sim tick, and an on-demand Worker spawn per
 *   call on wasm). A lane is created once, so a per-tick job is a queue push plus a wake, and
 *   because a lane runs ONE job at a time, state a lane's jobs own (a sim world, a geometry diff
 *   baseline) is touched by one thread with no locking.
 *
 *     cvc::async_lane sim_lane("sim");
 *     sim_lane.submit([&world] { world.step(1); });  // any thread; FIFO
 *
 *   Contract:
 *   - submit() is thread-safe and FIFO per submitting thread (one global FIFO, one worker).
 *   - A job that throws never terminates the process and never kills the lane: the exception
 *     message goes to the on_error handler (set_on_error), is counted in failed(), and the next job
 *     runs. The handler runs on the thread that ran the job.
 *   - stop() refuses new jobs (submit() then returns false), RUNS every job already queued, then
 *     joins the worker. It is idempotent, and ~async_lane() calls it, so a lane declared AFTER the
 *     objects its jobs capture is drained and joined before those objects die.
 *   - Never call stop() (or destroy a lane) from the browser main thread on wasm: the join blocks.
 *     A wasm app normally never stops its lanes (EXIT_RUNTIME=0).
 *
 * BUILDS WITHOUT THREADS
 *   Under Emscripten without -pthread (__EMSCRIPTEN__ && !__EMSCRIPTEN_PTHREADS__) starting a
 *   std::thread aborts, so every lane is forced to lane_mode::deferred: jobs queue and run on the
 *   PUMP thread, in FIFO order, when the driver calls run_deferred() (pump_exec_frame does that for
 *   the lanes passed to it). Results then arrive one frame after the submit, exactly as with a
 *   threaded lane, and nothing else in the caller changes. lane_mode::deferred can also be chosen
 *   natively, e.g. for a "--sim-async=0" parity run where the lane's work must stay on main.
 *
 * STATE_EXEC INTEGRATION (launch_lane_task / park_on_lane_task / future_lane_task / post_lane_task)
 *   These mirror launch_pool_task / park_on_pool_task / future_pool_task (async_task.h) with the
 *   compute-pool offload replaced by lane.submit(). The work's value, or on_error(message) if it
 *   throws, is posted with the THREAD-SAFE async_scheduler::post_message to a channel resolved by
 *   state_exec::resolve_channel_key(root, channel) -- the key a process chrooted to `root`
 *   resolves the same channel to in (msg-recv ...). Every path posts exactly one value, so a waiter
 *   never hangs: a throwing on_error posts a fallback error string, and a stopped lane posts
 *   on_error("... is stopped") at once instead of dropping the task.
 *   Use '#' channels (e.g. "app#built.tracks") for anything a lane posts: a '#' channel is
 *   the identity under chroot scoping and exempt from the section 12 channel policy, so the
 *   poster and a scoped receiver always agree on the key. A resident process can then wait on
 *   lane results forever:
 *
 *     register_fn(env, "sim-launch", [&](auto) {      // runs on the pump thread
 *       return se::value_t(cvc::launch_lane_task(sim_lane, sched, ictx.root_path, "app",
 *                                                [&] { return run_ticks(); }, on_err));
 *     });                                             // -> "app#sim.<n>"
 *     sched.execute("(while t (apply-sim (msg-recv (sim-launch))))", opts);
 *
 *   The `work` rules of async_task.h apply unchanged: it runs OFF the scheduler thread, so marshal
 *   inputs in before the submit and the result out; never touch the DSL evaluator or the state tree
 *   from it. Every scheduler a lane posts to, and every object its jobs capture, must outlive the
 *   lane's queued jobs: declare the lane after them (it is destroyed, so drained, first) or stop()
 *   it before they die.
 *
 * THE PUMP (drivers MUST call sync_step every frame)
 *   A lane completion only reaches the scheduler's thread-safe ingress queue. The thread that
 *   drives the scheduler must, EVERY frame:
 *
 *     sched.wake_awaiting(); // re-ready (await ...) frame-yielders, once per frame
 *     sched.sync_step();     // UNCONDITIONAL: drains the ingress (delivering lane posts, readying
 *                            // their receivers), then wakes (sleep ...) processes that are due
 *     sched.sync_run(steps, secs);
 *
 *   sync_run() steps only while has_runnable() is true, and the ingress is drained and sleepers
 *   woken only inside step(), so a pump without the sync_step() call never delivers a lane post
 *   to, or wakes, a process parked while nothing else is runnable: a
 *   (while t (sleep (msg-recv (sim-launch)))) loop stops after its first sleep. (A driver that
 *   skips sync_step() must call sched.drain_ingress() itself before sync_run.)
 *   pump_exec_frame() is this pump, plus run_deferred() on the lanes passed to it.
 *
 * BUILD FLAG
 *   None. state_exec is always built (the CVC_STATE_EXEC option was removed in #493), so the lane
 *   and these helpers are always available. The lane class itself has no scheduler dependency.
 */
#ifndef CVC_CORE_ASYNC_LANE_H
#define CVC_CORE_ASYNC_LANE_H

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cvc/core/async_task.h> // pool_task_work / pool_task_on_error
#include <cvc/core/state_exec/types.h>
#include <deque>
#include <functional>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

namespace cvc {

namespace state_exec {
class async_scheduler;
}

/// True when this build can start threads: every native build, and Emscripten built with -pthread.
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
inline constexpr bool async_lane_has_threads = false;
#else
inline constexpr bool async_lane_has_threads = true;
#endif

/// How an async_lane runs its jobs.
enum class lane_mode {
  /// One persistent worker thread (the default). Forced to `deferred` when !async_lane_has_threads.
  threaded,
  /// No thread: jobs queue and run, FIFO, on whichever thread calls run_deferred() -- the pump.
  deferred,
};

/// A persistent named worker lane: one thread, one FIFO, one job at a time. See the file comment.
class async_lane {
public:
  using job = std::function<void()>;
  /// Receives the message of a job that threw (std::exception::what(), or "unknown error").
  using error_handler = std::function<void(const std::string &)>;

  /// Start the lane. `name` labels the worker thread (Linux/macOS) and the lane's error strings.
  /// Throws std::system_error if the worker thread cannot be started.
  explicit async_lane(std::string name, lane_mode mode = lane_mode::threaded);
  /// stop(): drain then join.
  ~async_lane();

  async_lane(const async_lane &) = delete;
  async_lane &operator=(const async_lane &) = delete;
  async_lane(async_lane &&) = delete;
  async_lane &operator=(async_lane &&) = delete;

  /// Queue `j` (FIFO) from any thread. Returns false, discarding `j`, when `j` is empty or once
  /// stop() has begun.
  bool submit(job j);

  /// Refuse new jobs, run every job already queued, then join the worker (a deferred lane runs them
  /// on the caller). Idempotent and thread-safe. Called from one of the lane's own jobs -- or from
  /// another lane's job drained inside one of them -- it only marks the lane stopping (a thread
  /// cannot join itself, nor wait out a drain it is running); the destructor or a later stop() from
  /// outside the lane's jobs completes the join or the drain.
  void stop();

  /// Deferred lanes: run up to `max_jobs` of the jobs queued at entry, in FIFO order, on the
  /// calling (pump) thread, and return how many ran. A job submitted while this runs waits for the
  /// next call, so one call is one hop, as with a threaded lane. Re-entrant calls (from inside a
  /// job) and threaded lanes return 0.
  // (max)() so a <windows.h> max macro cannot expand here.
  std::size_t run_deferred(std::size_t max_jobs = (std::numeric_limits<std::size_t>::max)());

  /// Install the handler for a job that throws (replaces any previous one; thread-safe). A handler
  /// that itself throws is ignored. Without a handler the error is only recorded (failed(),
  /// last_error()): a lane never prints, because on wasm a Worker's stdio is proxied synchronously
  /// to the main thread.
  void set_on_error(error_handler h);

  const std::string &name() const { return name_; }
  /// The effective mode (`deferred` when this build has no threads, whatever was requested).
  lane_mode mode() const { return mode_; }
  bool threaded() const { return mode_ == lane_mode::threaded; }
  /// stop() has begun: submit() refuses new jobs.
  bool stopped() const;
  /// Jobs queued but not started.
  std::size_t pending() const;
  /// Jobs queued or running (a job counts until it returns).
  std::size_t in_flight() const { return in_flight_.load(std::memory_order_acquire); }
  /// Jobs that have finished, including the ones that threw.
  std::uint64_t completed() const { return completed_.load(std::memory_order_acquire); }
  /// Jobs that threw.
  std::uint64_t failed() const { return failed_.load(std::memory_order_acquire); }
  /// Message of the most recent job that threw ("" if none has).
  std::string last_error() const;
  /// Wall time of the most recently finished job, in milliseconds.
  double last_job_ms() const { return last_job_ms_.load(std::memory_order_acquire); }

private:
  void worker_loop();
  void run_job(job &j);
  void report_error(const std::string &msg);
  bool pop_front(job &out);

  const std::string name_;
  lane_mode mode_;

  mutable std::mutex mtx_; // guards queue_ and stopping_
  std::condition_variable cv_;
  std::deque<job> queue_;
  bool stopping_ = false;

  std::mutex join_mtx_; // serializes concurrent stop() joins
  std::thread worker_;
  std::atomic<bool> draining_{false}; // a deferred drain is running (re-entrancy/serial guard)

  mutable std::mutex err_mtx_; // guards on_error_ and last_error_
  error_handler on_error_;
  std::string last_error_;

  std::atomic<std::size_t> in_flight_{0};
  std::atomic<std::uint64_t> completed_{0};
  std::atomic<std::uint64_t> failed_{0};
  std::atomic<double> last_job_ms_{0.0};
};

/// The lane work kernel (same shape as the pool kernel): runs on the lane, returns the DSL value.
using lane_task_work = pool_task_work;
/// Builds the value to deliver when `work` throws, from the exception message.
using lane_task_on_error = pool_task_on_error;

/// Run `work` on `lane` and post its value -- or on_error(message) if it throws -- on `sched` to
/// the FIXED channel `channel`, resolved for chroot `root` with resolve_channel_key (the key a
/// process chrooted to `root` gets for (msg-recv channel); a '#' channel is used verbatim). For a
/// resident receiver such as (while t (apply-x (msg-recv "app#built.x"))). Callable from any
/// thread, including from inside another lane's job. Returns the resolved key.
std::string post_lane_task(async_lane &lane, state_exec::async_scheduler &sched,
                           const std::string &root, const std::string &channel, lane_task_work work,
                           lane_task_on_error on_error);

/// As post_lane_task, to a UNIQUE '#' reply channel "<chan_prefix>#<lane name>.<n>" (so two
/// in-flight tasks never share the single recv_path a process has), and return that key: the
/// native-fn result a DSL caller hands to (msg-recv ...). Returns immediately.
std::string launch_lane_task(async_lane &lane, state_exec::async_scheduler &sched,
                             const std::string &root, const std::string &chan_prefix,
                             lane_task_work work, lane_task_on_error on_error);

/// Transparent form: launch the task and SELF-PARK the current process on `sched`; it resumes with
/// the result threaded into the enclosing expression. Call only from a native_fn running under
/// `sched` (sched.current_pid()/current_process() valid), and not as a top-level call (wrap it,
/// e.g. in (begin ...)): park_on_channel throws for a call with no enclosing frame.
state_exec::value_t park_on_lane_task(async_lane &lane, state_exec::async_scheduler &sched,
                                      const std::string &root, const std::string &chan_prefix,
                                      lane_task_work work, lane_task_on_error on_error);

/// Future form: launch the task and return a {"__future__": chan} handle to (await ...) or
/// (msg-recv ...) on `sched`.
state_exec::value_t future_lane_task(async_lane &lane, state_exec::async_scheduler &sched,
                                     const std::string &root, const std::string &chan_prefix,
                                     lane_task_work work, lane_task_on_error on_error);

/// One frame of the scheduler pump, in the required order: run_deferred() on each of `lanes` (a
/// no-op for threaded lanes), then sched.wake_awaiting(), one UNCONDITIONAL sched.sync_step()
/// (drains the ingress and wakes due sleepers even when nothing is runnable), and
/// sched.sync_run(max_steps, max_time). Call once per frame on the scheduler's thread. Returns the
/// number of deferred lane jobs run. The defaults are a render-loop budget (20000 steps, 3 ms).
std::size_t pump_exec_frame(state_exec::async_scheduler &sched,
                            std::initializer_list<async_lane *> lanes = {},
                            std::uint64_t max_steps = 20000, double max_time = 0.003);

} // namespace cvc

#endif // CVC_CORE_ASYNC_LANE_H

/**
 * @file async_scheduler.h
 * @brief Coroutine-enabled scheduler using async_stackless_evaluator.
 *
 * Extends the synchronous scheduler with task<T> coroutine integration,
 * enabling cooperative multitasking via co_await.  Each run_one() call
 * advances a single process by one time slice and returns a task<T>.
 */
#ifndef CVC_STATE_EXEC_ASYNC_SCHEDULER_H
#define CVC_STATE_EXEC_ASYNC_SCHEDULER_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/state/state_exec/async_stackless_evaluator.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exec_router.h> // group_ref, route_entry (S2b)
#include <cvc/state/state_exec/heap.h> // cycle_registry
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/task.h>
#include <cvc/state/state_exec/types.h>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

class async_call;
struct random_stream; // stdlib.h

/// Async process scheduler using C++20 coroutines.
///
/// Same semantics as the sync scheduler. step() and run() are coroutines for a coroutine
/// executor to await; a step never suspends inside (a process's step runs to its end on the
/// calling thread), so sync_step() and sync_run() step without making a coroutine frame.
///
/// Implements scheduler_base — the intrinsic-facing surface — so the DSL
/// intrinsics (spawn/sleep/msg-send/msg-recv/self/ps/kill/pause/resume + the
/// state-watch registry) drive it identically to the sync scheduler.
class async_scheduler : public scheduler_base {
public:
  explicit async_scheduler(scheduling_policy policy = scheduling_policy::round_robin);
  /// A group's scheduler (docs/STATE_EXEC_THREADED.md 4.2): its global environment is
  /// environment::extend(base). A program's `set` binds in its process's own run env, and nothing
  /// a program writes reaches `base` (which may be frozen: the shared builtins).
  async_scheduler(scheduling_policy policy, environment_ptr base);
  /// Destroys the lock home first (protocol 5.7), before any other member goes; then a posted
  /// command never applied gets done(false).
  ~async_scheduler() override;

  // --- Locks (docs/STATE_EXEC_LOCKS.md) ---

  /// Use `mgr` for this scheduler's locks: the home is made and attached now, with `attach`
  /// (lock_attach::cross_thread when this scheduler may be pumped on another thread than the
  /// manager's other homes; a single-mode manager refuses it). Throws std::logic_error when a
  /// process of this scheduler holds or waits for a lock, or when `mgr` refuses the attach; the
  /// scheduler then keeps the home and manager it had.
  void set_lock_manager(std::shared_ptr<lock_manager> mgr, lock_attach attach = {});
  detail::lock_home *locks() override;
  /// Whether the scheduler has its lock home yet (made by set_lock_manager, or by locks() on first
  /// use); locks() makes one when it doesn't.
  bool has_lock_home() const noexcept { return home_ != nullptr; }
  /// The processes parked on a with-lock request: the process table's count (0: none, and
  /// nothing to look for).
  std::size_t lock_waiting_count() const noexcept { return procs_.lock_waiting_count(); }
  /// Clear `out`, then append the pid of every process parked on a with-lock request, in pid
  /// order. Walks the processes only while lock_waiting_count() is not zero.
  void lock_waiting_pids(std::vector<int> &out) const;
  bool check_tables() const override { return procs_.check(); }
  /// Close the lock home's port, if it has a home (lock_port::close, idempotent): no post lands
  /// after it. A group's teardown does it first, before its kills (td 4.2, S1-Q1).
  void close_lock_port();

  // --- Groups (docs/STATE_EXEC_THREADED.md 4.1, 4.5, 4.7, 4.10; Stage 1) ---

  /// Use `r` (null: none) for this scheduler's tracked objects: its evaluation context has it, and
  /// its due check and its exit notes name it, so they reach `r` whichever thread drives it (a
  /// host's kill(pid) between pumps included). The owner keeps `r` alive, and collects and frees
  /// it after this scheduler and its environments are gone. Unset, its evaluator's own registry is
  /// used (made at the first use, collected and freed with the scheduler).
  void set_cycle_registry(cycle_registry *r);
  /// The registry set_cycle_registry() gave, or null.
  cycle_registry *registry() const noexcept { return registry_; }
  /// Use `s` (null: none) for math.random, in its evaluation context. The owner keeps it alive.
  void set_random_stream(random_stream *s);
  /// The context its processes' steps run in (exec_context): its registry and random stream, and
  /// during a step the step's deadline and allocation guard. An exec_group's pump sets the group
  /// in it.
  exec_context &evaluation_context() { return evaluator_.context(); }
  /// Draw new pids from `source` (its fetch_add(1)) instead of this scheduler's own counter, so the
  /// schedulers sharing it never repeat a pid (td 4.5, B32): the app pool's groups and the app
  /// scheduler. `source` must be above every pid this scheduler has issued (process_table's
  /// insert rule).
  void set_pid_source(std::shared_ptr<std::atomic<int>> source);

  /// RAII: the calling thread is this scheduler's pump owner while the scope lives (S1-Q4). The
  /// rule is "no owner, or me": it takes an unset owner, nests in a scope of the same thread, and
  /// restores what it found. Another thread's owner makes the constructor throw
  /// std::runtime_error(kPumpedElsewhereError). A driver holds one while it drives: an exec_group's
  /// pump (the group's token, taken per pump by the pool), pycvc's Exec::run until it quiesces,
  /// the Python pump wrappers per call.
  class pump_owner_scope {
  public:
    explicit pump_owner_scope(async_scheduler &s);
    ~pump_owner_scope();
    pump_owner_scope(const pump_owner_scope &) = delete;
    pump_owner_scope &operator=(const pump_owner_scope &) = delete;

  private:
    async_scheduler &s_;
    std::thread::id outer_;
  };
  /// Any thread (one relaxed load): whether a thread other than the caller holds the pump owner.
  bool pumped_elsewhere() const noexcept;

  // --- Stage 2: the continuous pool (S2a; docs/STATE_EXEC_THREADED.md 4.4, 4.9) ---

  /// Install `fn(ctx)` as the notifier of this scheduler's wakeup, which its lock home's port
  /// shares (detail::wakeup::set_notifier, S2-Q2). While one is installed, every post calls it,
  /// whatever idle_waiting_ says: post_message, post_async_completion and post_control under the
  /// ingress mutex, a lock post under the port's mu_ (lock_port::post, unchanged); and
  /// wait_for_work() must not be called (asserted). nullptr clears it. Any thread; returns once no
  /// call of the old notifier runs. exec_pool::attach installs one and detach clears it;
  /// ExecPool.run_all installs one per pooled group for its batch (S2-Q6).
  void set_wake_notifier(detail::wake_notifier fn, void *ctx);
  /// Whether a notifier is installed (any thread).
  bool has_wake_notifier() const noexcept;

  /// What the last run loop left (S2-Q4), computed at its end on the pump owner's thread while a
  /// wake notifier is installed (a pool attached the group, or a host batch waits on it:
  /// ExecPool.run_all, the bench's fork-join async point), the only readers; otherwise it is left
  /// as it was, so a scheduler with no notifier runs master's loop exactly (R2b S-m2: the timer
  /// peek pops stale heap entries). exec_group::pump copies it into pump_result. Pump owner only.
  struct run_report {
    /// A process was ready when the run loop ended (its step or time budget ran out first).
    bool runnable = false;
    /// The earliest time the scheduler has work of its own that no post announces: its earliest
    /// timer (a sleeper's wake, a timed receive's deadline, a lock wait's deadline), an async
    /// call's registration timeout or next charge (the 1 ms beat of a process with a step or time
    /// limit), its cycle collection falling due, and now + 1 ms while a process waits for the
    /// next frame (yield_frame, which a pump's wake_awaiting readies); time_point::max(): none.
    std::chrono::steady_clock::time_point next_deadline =
        std::chrono::steady_clock::time_point::max();
    /// next_deadline is a sleeper's or a timed wait's, to be kept precisely (detail::wakeup's
    /// precise wait, S2-Q3); every other deadline is coarse.
    bool precise = false;
    /// process_table::best_runnable() when the run loop ended.
    std::optional<int> best_runnable;
    /// A process waits for the next frame (yield_frame) when the run loop ended.
    bool awaiting_frame = false;
  };
  const run_report &last_run_report() const noexcept { return run_report_; }

  /// The long-step monitor (S2a.7, S2-Q11): a step that takes longer than the threshold (at
  /// slice_steps above 1, a run's 16-step window), one that throws included, is counted
  /// (long_steps()) and reported to the sink with the call-site name of the native it ended in
  /// ("native" on the higher-order path, "(no native)" when it applied none), at most once per name
  /// per minute. The threshold is the tree's state_exec.long_step_ms when it is set, else the
  /// default set here; 0: off. A pool with workers gives the groups it pumps a default of 1 ms.
  /// With no sink, caught steps are only counted. Pump owner, or before the first pump.
  using long_step_sink = std::function<void(const std::string &native, double seconds)>;
  void set_long_step_sink(long_step_sink sink);
  void set_long_step_default(double seconds);
  /// The steps (or windows) the monitor caught since the scheduler was made (any thread).
  std::uint64_t long_steps() const noexcept { return long_steps_.load(std::memory_order_relaxed); }

  // --- Stage 2: the router and process control across groups (S2b; td 4.5, 4.6; S2-Q7..S2-Q10;
  // the rules are exec_router.h's) ---

  /// Join the app's routing as the group `self` names: exec_group's constructor (an app's group)
  /// and app::exec_scheduler() (group 0). From then on messaging on a key homed in another group
  /// goes through `router` while it is active, the pids this scheduler issues are written to its
  /// directory at their creation and erased at their reap, and each run loop publishes the
  /// processes' rows. Once, before the first process. A scheduler never given one (a plain pycvc
  /// Exec, a bare scheduler, a group with no app) never routes, writes or publishes anything.
  void set_router(exec_router *router, group_ref self);
  exec_router *router() const noexcept { return router_; }
  /// The handle set_router gave (empty with none).
  const group_ref &route_self() const noexcept { return route_self_; }
  /// The teardown's step (S2b.4; exec_router.h, "Teardown and rehoming"): unregister this group's
  /// prefixes, close its handle, then send its queued router entries, its remote-waiter records and
  /// the pending messages of the keys it homed to their new homes. The token holder, after the
  /// lock port's close and before the kills. Nothing without a router.
  void leave_router();
  /// Any thread (group_ref::post, which keeps the scheduler alive): push a router entry into the
  /// ordered ingress, as post_message's posts are pushed (a wait or the notifier is woken). False
  /// when the scheduler is being destroyed, or `e` is a send (not uncapped) and the ingress holds
  /// ingress_cap() capped entries (counted in ingress_counters().ingress_refused).
  bool post_route(detail::route_entry e);
  /// Per-pid delivery (td 4.6's deliver; S2b.1): when `pid` waits on `key` with recv_seq `seq`,
  /// give it `msg` as deliver_to_receivers gives a waiter its message, ready it, and return true;
  /// else false (stale). Scheduler thread.
  bool deliver_to(int pid, std::uint64_t seq, const std::string &key, const value_t &msg);

  /// The ingress cap (S2-Q10): the capped entries (send entries and host post_message posts) the
  /// ingress may hold undrained; 0: none. The tree's state_exec.groups.ingress_cap, else 65536; a
  /// value set here wins over the tree's. Scheduler thread.
  void set_ingress_cap(std::size_t cap);
  std::size_t ingress_cap() const noexcept { return ingress_cap_; }
  /// The ingress's counters since the scheduler was made (td 4.17, S2-Q10), any thread.
  struct ingress_stats {
    std::uint64_t ingress_refused = 0; ///< capped entries refused at the cap
    std::uint64_t requeued = 0;        ///< messages a requeue gave back to this home
    std::uint64_t stale_dropped =
        0; ///< stale deliveries dropped (a waiter already had the message)
    std::uint64_t pending_dropped = 0; ///< messages a full pending FIFO dropped
    std::uint64_t control_refused = 0; ///< commands refused (never applied, or one that threw)
    std::uint64_t throw_dropped = 0;   ///< entries a drain's throw left unapplied, not put back
    std::uint64_t destroy_dropped = 0; ///< messages and router entries the destructor dropped
  };
  ingress_stats ingress_counters() const noexcept;

  /// What the last run loop published (S2-Q9), any thread. Zero before the first publish, and
  /// always for a scheduler with no router.
  struct published_summary {
    bool runnable = false;
    std::chrono::steady_clock::time_point next_deadline =
        std::chrono::steady_clock::time_point::max();
    std::optional<int> best_runnable;
    int processes = 0;           ///< its live processes (not ended ones awaiting their reap)
    std::uint64_t publishes = 0; ///< run loops that published so far
  };
  published_summary published() const noexcept;
  /// `pid`'s published row, any thread; nullopt for a pid the last publish did not have.
  std::optional<published_process> published_info(int pid) const;
  /// Every published row, in pid order, any thread (S2c's cross-group ps filters them by owner).
  std::vector<published_process> published_rows() const;

  // --- Process submission ---

  int execute(const std::string &script, const execute_options &opts = {}) override;

  int execute(const value_t &expr, const execute_options &opts = {}) override;

  // --- Stepping (coroutines) ---

  /// Execute one step as a coroutine (the same step as sync_step()).
  task<int> step();

  /// Run until all done or limits. `max_steps` counts the steps the run's processes were charged,
  /// not the picks: a pick's evaluator step is one, and what that step charged on top of it is
  /// counted too (add_steps: a script engine's slice, an inline async call's run time), so work
  /// done inside a step can't stretch a run. An iteration that picks nothing counts one.
  task<std::unordered_map<int, value_t>> run(std::optional<uint64_t> max_steps = std::nullopt,
                                             std::optional<double> max_time = std::nullopt);

  /// Blocking wrappers: the same steps, with no coroutine frame.
  int sync_step();
  std::unordered_map<int, value_t> sync_run(std::optional<uint64_t> max_steps = std::nullopt,
                                            std::optional<double> max_time = std::nullopt);
  /// sync_run() without collecting the results: for a host that pumps the scheduler and reads
  /// the results it needs (get_result) or none, so a run doesn't cost a map entry for every
  /// finished process not yet reaped. Returns the steps it took, counted as run()'s `max_steps`.
  uint64_t sync_run_steps(std::optional<uint64_t> max_steps = std::nullopt,
                          std::optional<double> max_time = std::nullopt);

  /// Stage S (docs/STATE_EXEC_THREADED.md 4.4): a pick runs up to `steps` steps of its process, a
  /// run, on one step's guards, clock reading and accounting. 1, the default, is today's loop
  /// exactly. A run ends early at a park or any other status change, a post from another thread
  /// (a message, a completion, a control, a lock post), a process made ready at a better priority
  /// level or the runner's own level getting worse, a pending signal or watch event, a stop,
  /// max_steps or the message limits, the run's step budget, or, at a
  /// clock check after its steps 2, 4, 8 and 16 and then every 16, past `seconds`, the process's
  /// time, the pump's time budget (charged as the steps since its last check), a due timer or a
  /// due charge of an async call in flight (its timeout, or the 1 ms charge beat of a process with
  /// limits). A process with max_step_time or max_memory keeps one step a pick (a per-step guard;
  /// heap measured step by step). `seconds` applies only to a run loop with a time budget, so a
  /// run on steps alone (a deterministic pump) ends on steps only; infinite never ends a run, and
  /// 0 or less means 1 ms. Set here, the values win over the tree's state_exec.slice_steps and
  /// state_exec.slice_seconds, which win over CVC_EXEC_SLICE_STEPS, read when the scheduler is
  /// made. set_slice is set_slice_steps(steps) and set_slice_seconds(seconds). Scheduler thread.
  void set_slice(std::uint64_t steps, double seconds = 1e-3);
  /// One half of set_slice (0 steps means 1): the tree's other setting still applies, as for an
  /// exec_group given only one of its options. Each wins over its tree setting from then on.
  void set_slice_steps(std::uint64_t steps);
  void set_slice_seconds(double seconds);
  std::uint64_t slice_steps() const noexcept { return slice_steps_; }
  double slice_seconds() const noexcept { return slice_seconds_; }

  /// Any thread: end the run loop at its next step boundary (td 4.12). A stop() with no run
  /// active is discarded by the next run, which starts by clearing it.
  void stop();
  /// Any thread: is the run loop active? Once this is true, a stop() ends that run.
  bool is_running() const { return running_.load(std::memory_order_acquire); }

  // --- Process control ---

  bool pause(int pid) override;
  bool resume(int pid) override;
  bool kill(int pid) override;
  int fork(int pid) override;

  /// Kill every live process tagged with `owner` — an Ariadne Runtime reaping its
  /// process group on teardown. Returns the number killed. Parked processes
  /// (sleep/msg-recv/await) are killed too, so nothing lingers after the document
  /// that owned them closes. `owner` == "" reaps the unowned group.
  /// The live processes are taken first, so one a kill's slots submit is not killed; every kill
  /// is tried, and the first that threw is rethrown after the rest (S1.4).
  int kill_owner(const std::string &owner);
  /// kill(pid) with `reason` as the process's exit error (get_exit_error): a pool failing a group
  /// kills its processes with "internal error" (S1.6). False for an unknown or finished pid.
  bool kill(int pid, const std::string &reason);

  /// Cooperative sleep: put a process into `waiting`; woken by the pump
  /// (wake_sleeping_processes, called at the top of step()) once the deadline
  /// passes.  Identical semantics to the sync scheduler.
  bool sleep(int pid, double seconds) override;

  /// (await expr): park the process (`waiting` + awaiting_frame) until the next frame
  /// boundary; re-readied by wake_awaiting(). Returns false for an unknown/non-runnable pid.
  bool yield_frame(int pid) override;
  /// Re-ready every process parked by yield_frame — call ONCE per frame (the Ariadne pump
  /// does, at the top of a drain, NOT inside step()/sync_run, so an await crosses exactly one
  /// frame). Returns the number re-readied.
  int wake_awaiting();

  // --- Inter-process messaging (msg-send/msg-recv) ---
  bool receive_message(int pid, const std::string &path) override;
  bool receive_message_until(int pid, const std::string &path, double seconds) override;
  int deliver_to_receivers(const std::string &path, const value_t &msg) override;
  std::optional<value_t> pop_pending_message(const std::string &path) override;
  std::size_t pending_message_count(const std::string &path) const override;
  std::size_t total_pending_messages() const;
  /// Max messages queued per path when no receiver waits (0 = unlimited).
  std::size_t max_pending_messages = 1024;

  /// THREAD-SAFE ingress: post a message from ANY thread (e.g. an app.computePool()
  /// worker that finished the async work a `(msg-recv "path")` action is parked on).
  /// The message is queued under a lock and delivered on the scheduler's own thread at
  /// the next step (drain_ingress → deliver_to_receivers), so no scheduler state is
  /// mutated off-thread. deliver_to_receivers itself is NOT thread-safe — external
  /// threads must use this, only the scheduler thread may call deliver_to_receivers.
  /// Stage 2 (S2b, S2-Q10): it counts against the ingress cap, and returns false, posting nothing,
  /// once the cap is reached (or the scheduler is being destroyed); and on a key the router homes
  /// in another group it forwards the message there as a send (the home's cap then applies).
  bool post_message(const std::string &path, const value_t &msg);

  /// Block the calling (scheduler) thread until there is work for the next drain_ingress() and
  /// run: a post_message() or a settled async call (from any thread), a post to the lock port
  /// (another home's wake or relevel for one of its processes), a sleeper's or a timed
  /// wait's deadline, an async call's registration timeout, the next charge of a call whose
  /// process has a step or time limit (every millisecond while one runs: the charge may find
  /// the limit reached, docs/STATE_EXEC_EXCEPTIONS.md 3.8), or `deadline`, whichever comes
  /// first. Returns at once when a process is ready or such work is already waiting. True when
  /// there is work; false when `deadline` passed with none. Processes parked until the next
  /// frame (yield_frame) are the driver's to wake (wake_awaiting): they don't end the wait (see
  /// awaiting_frame_count()). A call's timeout and charges count only while its process still
  /// runs: a call whose process ended without awaiting it (an abandoned future) is no drain's
  /// to time out or charge, so it doesn't end the wait, and a NaN or infinite timeout never
  /// does. A sleeper's or a timed wait's deadline is kept to well under a millisecond, which on
  /// Windows costs up to ~0.6 ms of spinning at its end, at above-normal priority for a thread
  /// at normal (detail::wakeup); the other deadlines may be met up to ~0.5 ms late, burning no
  /// CPU. The wait never ends before a deadline on the steady clock.
  bool wait_for_work(std::chrono::steady_clock::time_point deadline);
  /// wait_for_work() for at most `seconds` (infinity: no deadline of its own).
  bool wait_for_work(double seconds);
  /// Processes parked until the next frame (yield_frame), which the next wake_awaiting() readies.
  std::size_t awaiting_frame_count() const { return procs_.awaiting_count(); }

  /// Apply any queued post_message() deliveries NOW, on the calling (scheduler) thread.
  /// step() calls this, but a driver must ALSO call it before a run whose has_runnable()
  /// gate would otherwise be false — a delivery can READY a parked receiver, and
  /// sync_run/run skip stepping (so never drain the ingress) when nothing is runnable yet.
  /// It also takes the async calls that finished, and charges the ones still running
  /// (docs/STATE_EXEC_EXCEPTIONS.md 3.8): a driver that calls it often (every frame) keeps
  /// their processes' budgets current. And it wakes the sleepers and timed waits that are due.
  /// Before all of it, it applies the lock port's posts (protocol 5.9): a lock waiter granted on
  /// another thread becomes ready here.
  void drain_ingress();

  // --- Async calls (docs/STATE_EXEC_EXCEPTIONS.md 3; for async_host) ---

  /// Track `call`, made by the process stepping now, as in flight: drain_ingress() charges its
  /// process as it runs, times it out, and cancels it if its process is killed or reaped.
  void track_async_call(std::shared_ptr<async_call> call);
  /// THREAD-SAFE: `call` has settled. The next drain_ingress() makes its final charge and
  /// resumes the processes waiting for it. A call this scheduler no longer tracks is dropped.
  void post_async_completion(std::shared_ptr<async_call> call);
  /// Async calls tracked and not yet finished.
  std::size_t async_calls_in_flight() const { return async_calls_.size(); }
  /// Tests (AR-3b): the post_message() and post_async_completion() calls this scheduler has
  /// taken, from any thread.
  std::uint64_t ingress_posts_for_test() const noexcept {
    return ingress_posts_.load(std::memory_order_relaxed);
  }

  /// The steps a CPU millisecond of `proc`'s async calls is charged: its own
  /// execute_options::async_steps_per_cpu_ms, else limits.async_steps_per_cpu_ms from the tree
  /// (async_steps_per_cpu_ms_from_state). The tree's value is read again only once something
  /// under the watch root changed since it was last read, so a change applies to the next charge
  /// while a quiet tree costs no read.

  double async_steps_per_cpu_ms(const process &proc) override;

  /// A unique identifier for this scheduler, the key under `state_exec.schedulers.<id>` its
  /// live settings are read from (limits.async_steps_per_cpu_ms) and the effective values
  /// published to (`effective.*`). Empty: the defaults only, nothing published.
  void set_id(const std::string &id) {
    id_ = id;
    rate_valid_ = false; // the id decides which limits.async_steps_per_cpu_ms applies
  }
  const std::string &id() const { return id_; }

  /// Adopt a process decoded from a snapshot (decode_process): it gets a new pid and is ready,
  /// unless it had ended. Its counters and budget so far are kept. A call it was waiting for
  /// can't resume: the call raises RuntimeError when the process runs. Returns the pid.
  int restore(process_ptr proc);
  /// Snapshot process `pid` into `node` (encode_process). False for an unknown pid.
  bool snapshot(int pid, cvc::state &node) const;
  /// Restore a snapshot() (decode_process, rebind_natives with `natives`, then
  /// restore(process_ptr)). Pass the environment the process ran with (execute_options::env) or
  /// null for none. Returns the new pid.
  int restore(cvc::state &node, environment_ptr natives = nullptr);

  /// S2b (S2-Q8): the local half as scheduler_base's, plus the directory's: a pid in another group
  /// of the caller's (non-empty) owner is posted there.
  bool control_as(const std::string &caller_owner, int pid, control_cmd::kind what) override;

  bool set_priority(int pid, int priority) override;
  std::optional<priority_info> get_priority(int pid) const override;
  priority_change set_priority_checked(int caller_pid, int pid, int priority) override;
  std::optional<priority_info> get_priority_as(int caller_pid, int pid) const override;
  bool set_priority_floor(int pid, int floor) override;
  void note_priority_clamped() override;
  /// Thread-safe: `cmd` for process `pid`, applied between steps on the scheduler's thread by the
  /// next drain (a step's, or drain_ingress()'s), as post_message's posts are; a driver blocked
  /// in wait_for_work() wakes. Commands are applied one at a time in the order they were posted
  /// with the messages and router entries (the ordered ingress, S2b.1), before the drain's settled
  /// calls (S0.7's throw rule is kept). kill,
  /// pause, resume, signal and set_priority are the direct calls' host paths, unchecked (kill's
  /// reason is killed_by_user; signal "SIGKILL" kills); an unknown pid is found, and refused,
  /// then. Each one's `done`, when set, gets what its call returned (a kill that throws has still
  /// killed its process: true); one that throws otherwise, or is never applied, gets false. A
  /// query runs its function, and must not throw. True once queued. A done that the destructor
  /// calls (with false) may post again: that post returns false, and the destructor refuses it in
  /// turn, never applying it. Any other post to a scheduler being destroyed is undefined: a host
  /// must not post to a scheduler it is destroying, except from a done.
  bool post_control(int pid, control_cmd cmd);
  /// Thread-safe: whether a posted command waits for a drain (a posted message is no command).
  bool has_pending_control() const noexcept {
    return controls_waiting_.load(std::memory_order_acquire) != 0;
  }
  /// Thread-safe (S2b): whether anything posted waits for a drain: a message, a command, a router
  /// entry or a settled call. A driver that pumps a scheduler only while it has live processes
  /// (group 0's) pumps it for this too, so a router entry homed there is never stranded (S2.md
  /// section 0, row 22).
  bool has_pending_ingress() const noexcept {
    return ingress_pending_.load(std::memory_order_acquire);
  }
  bool set_max_steps(int pid, uint64_t max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, uint64_t bytes);
  bool set_max_messages(int pid, uint64_t count);
  bool set_max_message_bytes(int pid, uint64_t bytes);

  // --- Signal handling ---

  bool send_signal(int pid, const std::string &signal);

  // --- Process info ---

  std::vector<process_info> list_processes() const override;
  std::optional<process_info> get_process_info(int pid) const override;
  std::optional<value_t> get_result(int pid) const;
  /// The failure reason of a killed process (e.g. the message of a native_fn that threw, or a
  /// resource-limit tag), or nullopt if the pid is unknown or did not fail. A driver reports this
  /// so a caught exception is not flattened to a generic "killed".
  std::optional<std::string> get_exit_error(int pid) const;
  /// A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6), or nullopt.
  std::optional<value_t> get_exit_exception(int pid) const;
  /// Throw a failed process's exception as a dsl_exception (with the C++ exception it was made
  /// from, if any, as its original()); returns for any other process.
  void raise_exit_exception(int pid) const;
  /// Erase a FINISHED (terminated or killed) process and its side-table entries from the scheduler,
  /// freeing its env/state/captured values. Returns false (no-op) for an unknown pid or a live one
  /// — never drops a runnable/waiting process. A one-shot driver (pycvc.Exec.run) reaps its pid
  /// after collecting the result so a long-lived scheduler does not accumulate dead processes.
  /// Its async calls still in flight (futures it never awaited) are cancelled. With `released`, the
  /// erased process is handed out there instead of destroyed, so a caller that reaps under a lock
  /// frees it, and whatever its captured values run as they go (a Python finalizer), after the
  /// unlock (pycvc's Exec, docs/STATE_EXEC_LOCKS.md 7.4). A process reaped while it steps is erased
  /// once its step is over, and `released` is left empty.
  bool reap(int pid, process_ptr *released = nullptr);
  /// Cancel a finished `pid`'s async calls still in flight, as reap() does first; what they ran so
  /// far stays charged. The charge reads the async step rate, which a scheduler with an id
  /// publishes to the tree, whose slots run then. A caller that reaps under a lock cancels first,
  /// before taking it, and the reap then finds nothing left to cancel. A live process's calls are
  /// left alone: dropped, they would leave it waiting on them for good (its kill cancels them).
  void cancel_async_calls_of(int pid);
  std::unordered_map<int, value_t> get_results() const;
  scheduler_stats get_stats() const override;

  // --- Accessors ---

  scheduling_policy policy() const override { return policy_; }
  const memory_tracker &mem_tracker() const { return mem_tracker_; }
  int process_count() const { return static_cast<int>(procs_.size()); }
  bool has_runnable() const;

  /// PID / process pointer of the process currently being stepped (-1 / nullptr
  /// if none).  Set around evaluator_.step() so intrinsics can query `self`.
  int current_pid() const override { return current_pid_; }
  const process_ptr &current_process() const override {
    return current_proc_ ? *current_proc_ : no_process_;
  }

  void queue_watch_event(int pid, process::watch_event evt);

  /// The 5-argument form with a null root (the scheduler's watch root).
  void register_watch_handler(int pid, int watch_id, value_t handler,
                              const std::string &path) override;
  /// Resolves `path` under `root` when it is non-null (a chrooted context's root, which the entry
  /// pins; td 4.8). The watch is read only after a write under its own root.
  void register_watch_handler(int pid, int watch_id, value_t handler, const std::string &path,
                              boost::shared_ptr<cvc::state> root) override;
  void unregister_watch_handler(int pid, int watch_id) override;

  /// The root for watch polling, settings and the async step rate: a host's call (app, pycvc),
  /// which keeps the root alive. A step makes only the pinned form below (td 4.8).
  void set_watch_root(cvc::state *root) override;
  /// The same for a root a program's step gives (state-watch's): kept by `pin` while it is the
  /// watch root (scheduler_base).
  void set_watch_root(cvc::state *root, boost::shared_ptr<cvc::state> pin) override;
  /// The root set_watch_root() gave, or null.
  cvc::state *watch_root() const override { return watch_root_; }

  /// The watch entries poll_watches() has looked up so far, one each time it polls a watch,
  /// whether or not its node exists. A watch is polled only after a write under the root it
  /// resolves under (td 4.8: per-root gating).
  uint64_t watch_polls() const noexcept { return watch_polls_; }

  /// This scheduler's built-in parser settings: what a submitted script is parsed with when the
  /// watch root's `state_exec.defaults.parser.*` keys don't say otherwise. parse_options{} (parse
  /// at submit) unless the host sets it.
  void set_parser_defaults(const parse_options &defaults) {
    parser_defaults_ = defaults;
    submit_valid_ = false;
  }
  const parse_options &parser_defaults() const { return parser_defaults_; }

  /// The parser defaults libcvc's own hosts give their async schedulers, so a script behaves the
  /// same in each: app::exec_scheduler() (Ariadne) and pycvc's Exec. Scripts are read inside
  /// their process, 1024 tokens per step (docs/STATE_EXEC_GRAMMAR.md, "Reading inside the
  /// scheduler"). Every script the repo carries fits in one slice (the largest is 159 tokens;
  /// .ari scripts are at most 24), so a normal action pays one extra step of its 5000. A slice
  /// costs about 80 us at the parser's measured ~80 ns per token, so a pathologically large
  /// script is spread over steps instead of stalling a drain.
  static parse_options host_parser_defaults() {
    parse_options defaults;
    defaults.step_tokens = 1024;
    return defaults;
  }

  /// This scheduler's built-in deep-data settings: what a submitted process runs with when the
  /// watch root's `state_exec.defaults.data.*` keys don't say otherwise. data_options{} (no
  /// slicing) unless the host sets it.
  void set_data_defaults(const data_options &defaults) {
    data_defaults_ = defaults;
    submit_valid_ = false;
  }
  const data_options &data_defaults() const { return data_defaults_; }

  /// The deep-data settings a process submitted now gets: the watch root's keys over
  /// data_defaults(). Read on every submit, like the parser settings.
  data_options data_settings() const {
    return watch_root_ ? data_options_from_state(*watch_root_, {}, data_defaults_) : data_defaults_;
  }

  /// The deep-data defaults libcvc's own hosts give their async schedulers (app::exec_scheduler()
  /// and pycvc's Exec), next to host_parser_defaults(): deep-data work runs 1024 nodes per step
  /// (docs/STATE_EXEC_GRAMMAR.md, "Deep data on the executor"). At the measured ~45 ns a node
  /// to free, ~70 to print, ~250 to compare and ~380 to copy, a slice costs 0.05-0.4 ms, so a huge
  /// value is spread over steps instead of stalling a drain, while the data of a normal program
  /// (far under 1024 nodes a step) costs it no extra steps.
  static data_options host_data_defaults() {
    data_options defaults;
    defaults.step_nodes = 1024;
    return defaults;
  }

private:
  friend class async_call; // async_call::outcome() finishes a call before the program sees it

  /// What this scheduler gives its lock home (design 8.4).
  struct lock_ops_impl final : detail::lock_sched_ops {
    explicit lock_ops_impl(async_scheduler &sched) : s(sched) {}
    async_scheduler &s;
    bool park_lock(process &p) override;
    void ready_lock_waiter(process &p) override;
    void relevel(process &p) override;
    std::size_t lock_expired_count() const override;
    void take_lock_expired(std::vector<process *> &out) override;
    process *find_process(int pid) override;
    void for_each_process(const std::function<void(process &)> &fn) override;
  };

  scheduling_policy policy_;
  std::string id_;
  double published_rate_ = -1.0; // the effective.async_steps_per_cpu_ms last published
  int next_pid_ = 1;
  int rr_index_ = 0;
  /// The run loop's flags, which stop() and is_running() touch from any thread (td B17), as the
  /// sync scheduler's: a run clears stop_requested_ before it publishes running_, and reads it with
  /// acquire, pairing with stop()'s release.
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  static_assert(std::atomic<bool>::is_always_lock_free, "stop() must not take a lock");
  uint64_t total_steps_ = 0;
  uint64_t step_charge_ = 0;      // the steps the last pick charged its process (run_loop's)
  uint64_t priority_clamped_ = 0; // spawns clamped to the spawner's floor (note_priority_clamped)

  int current_pid_ = -1;
  uint32_t cycle_check_ = 0; // steps, for checking every 64th whether a cycle collection is due
  const process_ptr *current_proc_ = nullptr; // the stepping process's entry in procs_
  int reap_after_step_ = -1; // reaped while it stepped: erased once the step is over
  // Stage S: the steps a pick may run of its process, and the time a run may take (set_slice).
  uint64_t slice_steps_ = 1;
  // The default (a tree slice_steps deleted): CVC_EXEC_SLICE_STEPS as the scheduler was made, or 1.
  uint64_t slice_steps_default_ = 1;
  double slice_seconds_ = 1e-3;
  bool slice_steps_set_ = false;       // set_slice_steps was called: the tree's doesn't apply
  bool slice_seconds_set_ = false;     // set_slice_seconds was called: the tree's doesn't apply
  uint64_t slice_seen_ = ~uint64_t{0}; // the settings version the tree's slice was last read at
  uint64_t run_steps_ = 1;             // the steps the last pick ran (execute_process_step)
  // The last run's steps after its last clock reading, which the run loop's next check of its time
  // budget stands for (time_budget::spent(now, checks)); 1 after a pick of one step.
  uint64_t run_unchecked_ = 1;
  time_budget *run_budget_ = nullptr; // the run loop's time budget, while it runs

  // The scheduled evaluator IS the async_stackless_evaluator (roadmap §8.12). Its step never
  // suspends, so the scheduler calls its step_now() directly (no coroutine frame per step).
  async_stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;
  cvc::state *watch_root_ = nullptr;
  /// What keeps watch_root_ alive when a step gave it (set_watch_root's pin); null for a host's.
  /// Declared before what connects to the root, so it goes after them.
  boost::shared_ptr<cvc::state> watch_root_pin_;
  parse_options parser_defaults_;
  data_options data_defaults_;

  /// Every process by pid (finished ones until reaped), its run queue, timers and waiters.
  detail::process_table procs_;

  lock_ops_impl lock_ops_{*this};
  /// Made by locks() or set_lock_manager(); null until a with-lock needs it.
  std::unique_ptr<detail::lock_home> home_;
  std::vector<process *> lock_expired_; // reused by drain's lock expiries
  /// An expiry pass (at `now`) threw: the lock waiters it had not timed out get a deadline due
  /// `now`, back on the timer heap, and lock_expired_ is emptied.
  void keep_lock_expiries(std::chrono::steady_clock::time_point now);

  /// A steady_clock reading the next step may start from (see scheduler.h).
  using clock = std::chrono::steady_clock;
  clock::time_point chain_now_{};
  bool chain_valid_ = false;
  double chain_cpu_ = -1.0;
  clock::time_point now_for_step();

  /// Counts tree changes under watch_root_ while a watch resolves under it or async calls are in
  /// flight: poll_watches() reads those watched nodes, and async_steps_per_cpu_ms() the rate,
  /// only once the count moved. A watch with a root of its own is counted in watch_roots_
  /// instead (td 4.8).
  detail::tree_changes tree_changes_;
  uint64_t watch_seen_ = 0;
  detail::watch_roots watch_roots_;
  uint64_t watch_polls_ = 0; // watch entries polled (watch_polls())
  uint64_t rate_seen_ = 0;
  bool rate_valid_ = false;
  double rate_ = kDefaultAsyncStepsPerCpuMs;
  void track_tree_changes();
  double tree_rate();

  /// What a submission reads from the tree (the allocation limit, the heap check, the parser
  /// and deep-data settings), read again only when settings_changes_ moved.
  struct submit_settings {
    uint64_t max_alloc_bytes = kDefaultMaxAllocBytes;
    std::size_t heap_check_steps = 0;
    parse_options parse;
    std::size_t data_step_nodes = 0;
  };
  detail::settings_changes settings_changes_;
  submit_settings submit_;
  uint64_t submit_seen_ = 0;
  bool submit_valid_ = false;
  const submit_settings &settings_for_submit();

  std::vector<process *> woken_; // reused by the waiter wakes
  std::vector<int> charged_;     // the pids charge_async_calls charged (reused)

  const process_ptr no_process_; // null: current_process() while none is stepped

  /// Per-path FIFO for messages sent when no receiver is waiting (mirrors sync scheduler). A deque:
  /// a requeue goes to its front (S2b.4).
  std::unordered_map<std::string, std::deque<value_t>> pending_messages_;

  /// The ordered ingress (S2b.1; exec_router.h): post_message's messages, post_control's commands
  /// and router entries in one queue, applied in post order by a drain, after the lock port's posts
  /// and before the settled calls. Guarded by ingress_mutex_.
  struct posted_message {
    posted_message(std::string k, value_t v) : key(std::move(k)), value(std::move(v)) {}
    std::string key;
    value_t value;
  };
  struct posted_control {
    posted_control(int p, control_cmd c) : pid(p), cmd(std::move(c)) {}
    int pid;
    control_cmd cmd;
  };
  using ingress_entry = std::variant<posted_message, posted_control, detail::route_entry>;
  std::mutex ingress_mutex_;
  std::vector<ingress_entry> ingress_;
  /// The capped entries in ingress_ (host messages and sends not uncapped), against ingress_cap_.
  std::size_t ingress_capped_ = 0;
  std::vector<std::shared_ptr<async_call>> completions_; // settled calls, guarded likewise
  /// Set (under ingress_mutex_) while ingress_ or completions_ holds something, so a drain finds
  /// an empty ingress without the lock.
  std::atomic<bool> ingress_pending_{false};
  /// post_message() and post_async_completion() calls taken (ingress_posts_for_test()).
  std::atomic<std::uint64_t> ingress_posts_{0};
  /// The commands posted and not yet applied or refused, queued or in a batch a drain is
  /// applying: has_pending_control(). A command stops counting once its call is made, before its
  /// done runs, or as it is refused. destroying_ (under ingress_mutex_): the destructor took the
  /// queue, and a post (a done's) is queued for it to refuse.
  std::atomic<std::size_t> controls_waiting_{0};
  bool destroying_ = false;
  /// A wait_for_work() is (about to be) blocked: a post notifies wakeup_.
  std::atomic<bool> idle_waiting_{false};
  /// What wait_for_work() blocks on. Made at construction and shared with the lock home's port,
  /// whose posts notify it whatever idle_waiting_ says (protocol 2.3).
  std::shared_ptr<detail::wakeup> wakeup_;
  // Notify a wait_for_work() in progress (any thread, after ingress_pending_ is set).
  void notify_work();

  /// The async calls in flight (scheduler thread only).
  std::vector<std::shared_ptr<async_call>> async_calls_;
  // A settled call: its final charge, then wake who waits for it.
  void finish_async_call(const std::shared_ptr<async_call> &call);
  // Charge the calls still running, time them out, kill the processes past their budget.
  void charge_async_calls(clock::time_point now);
  // When a drain next has work for the calls in flight (a timeout, a charge that may hit a
  // limit), or nullopt (wait_for_work).
  std::optional<clock::time_point> next_call_charge(clock::time_point now) const;
  // charge_async_calls from the `i`th call on, over a copy of the list (a timeout changes it),
  // at the tree's rate `tree` as read before the walk.
  void charge_async_calls_from(std::size_t i, clock::time_point now, std::optional<double> tree);
  // The tree's rate for the walk, read before it when a started call needs it (else nullopt).
  std::optional<double> tree_rate_for_calls();
  // The rate a call is charged at in the walk: its process's own, else `tree`; nullopt when the
  // call isn't started, or needs the tree's and it wasn't read.
  static std::optional<double> walk_rate(const process &proc, const async_call &call,
                                         std::optional<double> tree);
  // Check the processes the walk charged against their limits.
  void check_charged(clock::time_point now);
  // Cancel and drop the calls of `pid` (killed or reaped).
  void cancel_async_calls(int pid);
  // Resume the processes waiting on `channel` in the await_call phase.
  void wake_async_waiters(const std::string &channel);

  const process_ptr *select_process();
  // One step of the run loop (sync_step() and step() start a fresh clock chain).
  int step_once();
  // What a step does before picking a process: the lock port's posts, the ingress, the calls,
  // the due timers.
  void drain(bool in_step);
  // Apply one post_control command (scheduler thread), then tell its done what the call returned.
  void apply_control(int pid, control_cmd &cmd, bool &counted_out);
  // Apply one entry of the ordered ingress (drain); `counted_out` as apply_control's.
  void apply_entry(ingress_entry &e, bool &counted_out);
  // The entry that threw in a drain (the throw rule, exec_router.h): not put back, and counted; a
  // command refused unless its call returned; a deliver that patched nothing goes home as a
  // requeue.
  void entry_threw(ingress_entry &e, bool counted_out) noexcept;
  // A drain that threw: what it had not applied (batch[m..], finished[f..]) goes back to the front
  // of the ingress, ahead of what was posted since (S0.7).
  void requeue_ingress(std::vector<ingress_entry> &batch, std::size_t m,
                       std::vector<std::shared_ptr<async_call>> &finished, std::size_t f) noexcept;
  // The destructor's end of an entry never applied: a command refused, a message dropped and
  // counted, a router entry bounced to its home (bounce_route).
  void refuse_entry(ingress_entry &e) noexcept;
  // Push a message or router entry into the ingress under ingress_mutex_: false when it is being
  // destroyed, or `capped` and the cap is reached (counted).
  bool push_capped(ingress_entry &&e, bool capped);
  uint64_t run_loop(std::optional<uint64_t> max_steps, std::optional<double> max_time);
  // The run loop's report (run_report_), at its end, while a wake notifier is installed.
  void fill_run_report();
  // The tree's state_exec.slice_steps and slice_seconds, read at a run's start when a setting
  // under state_exec may have changed (settings_changes), each unless its own setter
  // (set_slice_steps, set_slice_seconds, or set_slice for both) was called.
  void refresh_slice();
  // A pick that runs up to min(slice_steps_, budget) steps of its process: step_once, with a run
  // in place of one step. step_once is today's loop, and run_loop takes it at slice_steps_ 1.
  int step_run(uint64_t budget);
  // One step, or with max_run above 1 a run of up to max_run steps on that step's guards, clock
  // reading and accounting (continue_run). run_steps_ is set to the steps it ran.
  void execute_process_step(const process_ptr &pp, uint64_t max_run = 1);
  // The rest of a run after its first step, counting each step in run_steps_ as it begins (a step
  // that throws counts): returns whether the evaluation is done.
  // count0 is the process's step count before the run: the run ends once it has charged max_run.
  // charge_cpu and cpu0 are the run's CPU charging, as execute_process_step set it at its start:
  // max_time's second half ends the run at its clock reads.
  bool continue_run(process &proc, uint64_t max_run, clock::time_point start, uint64_t count0,
                    bool charge_cpu, double cpu0);
  void handle_signal(process &proc);
  void handle_watch_event(process &proc);
  void restore_from_signal(process &proc);
  void poll_watches();
  void check_limits(process &proc);
  void check_limits(process &proc, clock::time_point now);
  void terminate_process(process &proc, value_t result);
  void kill_process(process &proc, const std::string &reason);
  // End a process `failed` with the exception it didn't catch (docs/STATE_EXEC_EXCEPTIONS.md 2.6).
  void fail_process(process &proc, value_t exc, std::exception_ptr original);
  /// After a process ended: note it for the cycle collector and, unless it is the one stepping
  /// now, release its evaluation state (process::release_state).
  void release_if_idle(process &proc);
  process_info make_info(const process &proc) const;

  /// Stage 1: the registry and stream the owner set (null: the thread's own), the pid source
  /// (null: next_pid_), and the pump owner, the thread driving this scheduler (none:
  /// std::thread::id()).
  cycle_registry *registry_ = nullptr;
  random_stream *random_ = nullptr;
  std::shared_ptr<std::atomic<int>> pid_source_;
  std::atomic<std::thread::id> pump_owner_{};

  /// Stage 2 (S2a): the last run loop's report (last_run_report), and the long-step monitor's
  /// sink, default threshold and count.
  run_report run_report_;
  long_step_sink long_step_sink_;
  double long_step_default_ = 0.0;
  std::atomic<std::uint64_t> long_steps_{0};
  /// CVC_EXEC_FUZZ's yields at the ingress posts (S2-Q15), read as the scheduler is made; null
  /// when it is off.
  std::unique_ptr<detail::lock_fuzz> ingress_fuzz_;

  /// Stage 2 (S2b): the app's router and this scheduler's handle (set_router; null and empty:
  /// none), the ingress cap and whether a host set it, and the ingress's counters
  /// (ingress_counters()). The ordered queue itself, its home records (remote waiters, tracked
  /// broadcasts) and the cache of homes are S2b.1's and S2b.4's, beside the members they replace.
  exec_router *router_ = nullptr;
  group_ref route_self_;
  std::size_t ingress_cap_ = 65536;
  bool ingress_cap_set_ = false;
  struct ingress_counter_set {
    std::atomic<std::uint64_t> ingress_refused{0}, requeued{0}, stale_dropped{0},
        pending_dropped{0}, control_refused{0}, throw_dropped{0}, destroy_dropped{0};
  };
  ingress_counter_set ingress_counts_;
  /// Channel homes (S2b.4; exec_router.h's protocol, as amended in S2b.0a), scheduler thread. As a
  /// home: the remote waiters of its keys, in subscription order (a std::map: its teardown
  /// forwards them in key order, DT-4), and its tracked broadcasts by id (unique in the app). As a
  /// receiver: its processes' waits on keys homed elsewhere, by pid, with the home each subscribed
  /// at. The park count recv_seq is drawn from (a requeue's next local waiter is the earliest), the
  /// cache of homes (capped, checked against the router's epoch) with whether this group registered
  /// a prefix (an island keeps the keys no other prefix covers), and whether the deliver being
  /// applied patched its frame.
  struct remote_waiter {
    group_ref from;
    int pid = -1;
    std::uint64_t seq = 0;
    std::optional<clock::time_point> deadline;
  };
  std::map<std::string, std::deque<remote_waiter>> remote_waiters_;
  struct tracked_broadcast {
    std::string key;
    value_t value;
    std::size_t outstanding = 0; // copies not yet answered (taken or requeue)
    bool taken = false;
  };
  std::unordered_map<std::uint64_t, tracked_broadcast> tracked_;
  std::uint64_t broadcast_seq_ = 0; // ids with no router (never routed: tests' bare schedulers)
  struct remote_wait {
    std::string key;
    std::uint64_t seq = 0;
    group_ref home;
    bool timed = false; // a timed wait, which a timer pass may end (sweep_remote_waits)
  };
  std::unordered_map<int, remote_wait> remote_waits_;
  std::size_t remote_timed_waits_ = 0; // the timed ones among them
  /// Erase `it` from remote_waits_, keeping the timed count.
  void forget_remote_wait(std::unordered_map<int, remote_wait>::iterator it);
  std::uint64_t recv_clock_ = 0;
  mutable std::unordered_map<std::string, group_ref> home_cache_;
  mutable std::uint64_t home_cache_epoch_ = 0;
  mutable bool home_cache_prefixed_ = false; // this group registered a prefix, as of that epoch
  bool deliver_patched_ = false;
  /// `key`'s home through the cache (router set; not a '#' key).
  group_ref cached_home(const std::string &key) const;
  /// Whether `key` is homed in another group (router on, not '#'; an island's keys no other
  /// group's prefix covers stay its own), which `home` gets. Scheduler thread.
  bool homed_elsewhere(const std::string &key, group_ref *home) const;
  /// Post `e` to `home`, any thread. A closed handle is resolved again (home_of), waiting while
  /// it is still that closed group (exec_router.cpp). `posted_to` gets the home that took it. 1:
  /// taken; 0: refused by the home (its cap); -1: no home took it.
  int post_routed(group_ref home, detail::route_entry e, group_ref *posted_to = nullptr) const;
  /// A message sent at its home (a local msg-send, a host message, a send entry): every waiter,
  /// local at once and remote by a deliver, else the back of the pending FIFO (past
  /// max_pending_messages when `uncapped`: a rehomed message). The waiters reached.
  int deliver_home(const std::string &key, const value_t &msg, bool uncapped);
  /// The remote waiters of `key` at this home get `msg` (deliver entries; broadcast tracking);
  /// nonzero when the message was handed on.
  std::size_t deliver_remote(const std::string &key, const value_t &msg, bool local_took);
  /// A requeue at its home: the next waiter (the earliest local waiter, else the earliest record),
  /// else the front of the pending FIFO, past its cap.
  void give_next(const std::string &key, value_t msg);
  /// A stale copy back at its home (a requeue entry, or a deliver a closed receiver refused):
  /// tracked broadcasts by `broadcast`, else give_next.
  void apply_requeue(const std::string &key, value_t msg, std::uint64_t broadcast);
  /// A tracked broadcast's id (exec_router::new_broadcast_id).
  std::uint64_t new_broadcast_id();
  /// receive_message's park (recv_seq moves), and its subscribe at a remote home.
  bool park_receiver(int pid, const std::string &path);
  void subscribe_if_remote(process &proc, const std::string &path);
  /// `pid`'s remote wait ended (a timeout, a kill, a reap, a local delivery): unsubscribe.
  void end_remote_wait(int pid) noexcept;
  /// After timed waits expired: unsubscribe the remote waits that no longer wait.
  void sweep_remote_waits() noexcept;
  void apply_route(detail::route_entry &e);
  void home_subscribe(detail::route_entry &e);
  void home_unsubscribe(const detail::route_entry &e);
  void home_taken(const detail::route_entry &e);
  void home_adopt(detail::route_entry &e);
  void receiver_deliver(detail::route_entry &e);
  /// A deliver that threw before patching its frame (the throw rule): stale, and its wait
  /// subscribed again with the same seq.
  void deliver_threw(detail::route_entry &&e) noexcept;
  /// A router entry this scheduler will not apply (its teardown's queue, its destructor) sent on
  /// to the key's new home (exec_router::new_home_of), as the teardown rule says, or dropped and
  /// counted.
  void bounce_route(detail::route_entry &&e) noexcept;
  /// Published info (S2-Q9; S2b.3): the summary's atomics, the rows under their mutex, the
  /// generation they were built at, and when their counters were last refreshed. Written at the
  /// run loop's end (publish_info), read by any thread.
  std::atomic<bool> pub_runnable_{false};
  std::atomic<std::int64_t> pub_deadline_{std::numeric_limits<std::int64_t>::max()};
  std::atomic<int> pub_best_{std::numeric_limits<int>::min()}; // min(): none
  std::atomic<int> pub_processes_{0};
  std::atomic<std::uint64_t> pub_count_{0};
  mutable std::mutex pub_mu_;
  std::vector<published_process> pub_rows_; // in pid order, under pub_mu_
  std::uint64_t pub_generation_ = ~std::uint64_t{0};
  clock::time_point pub_refreshed_{};
  /// Publish at the run loop's end (S2-Q9): the summary always; the rows when the table's
  /// generation moved, their counters at most every 100 ms. Pump owner; nothing without a router.
  void publish_info();
  /// The long-step monitor (S2a.7): the threshold in force in seconds (0: off), the tree's
  /// state_exec.long_step_ms in seconds while it is set (else the default applies), when the step
  /// or the run's window being timed began, and when each native was last reported.
  double long_step_seconds_ = 0.0;
  std::optional<double> long_step_tree_;
  clock::time_point long_window_{};
  std::unordered_map<std::string, clock::time_point> long_step_reported_;
  /// The step or window that began at long_window_ ends `now`: counted and reported if it was
  /// long (async_scheduler.cpp); the next one begins `now`.
  void time_long_step(const process &proc, clock::time_point now) noexcept;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_SCHEDULER_H

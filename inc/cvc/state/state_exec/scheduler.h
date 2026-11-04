/**
 * @file scheduler.h
 * @brief Round-robin process scheduler with resource enforcement.
 *
 * Manages a set of processes, executing them in round-robin order with
 * configurable time-slice granularity.  Enforces per-process step limits,
 * timeouts, and memory limits.  Supports spawn, fork, kill, pause/resume,
 * and inter-process messaging.
 */
#ifndef CVC_STATE_EXEC_SCHEDULER_H
#define CVC_STATE_EXEC_SCHEDULER_H

#include <atomic>
#include <chrono>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

// scheduling_policy, execute_options, process_info, scheduler_stats now live in
// scheduler_base.h (shared by the sync scheduler and async_scheduler).

namespace detail {

/// The roots a scheduler's watches resolve under, other than its watch root: a chrooted
/// context's (watch_handler_entry::root; td 4.8, S0.5). Each distinct root gets its own change
/// counter on that node's signals, shared by the entries with that root and dropped with the last
/// of them, so a watch is read only after a write under its own root. Scheduler thread only.
class watch_roots {
public:
  /// The entries may have changed: a watch registered or removed, or the watch root moved.
  void invalidate() noexcept { stale_ = true; }
  /// Bring the counters in line with the entries of `watchers` when they may have changed (after
  /// invalidate(), or once the number of watching processes changed: one ended). `watch_root` is
  /// the scheduler's watch root: an entry without a root of its own, or with that one, counts
  /// toward uses_main() instead. True when it connected or dropped a counter.
  bool sync(const std::vector<process *> &watchers, const cvc::state *watch_root);
  /// Whether an entry resolves under the scheduler's watch root (as of the last sync()).
  bool uses_main() const noexcept { return uses_main_; }
  bool empty() const noexcept { return slots_.empty(); }
  /// Read every counter: true when any moved since the last poll() (a new counter has).
  bool poll();
  /// Whether `root`'s counter moved at the last poll().
  bool moved(const cvc::state *root) const;
  /// Drop every counter (nothing watches).
  void clear();

private:
  struct slot {
    boost::shared_ptr<cvc::state> root; // the entries pin it too; this keeps its signals
    tree_changes changes;
    uint64_t seen = 0;
    bool moved = false;
    bool used = false;
  };
  std::unordered_map<const cvc::state *, std::unique_ptr<slot>> slots_;
  bool stale_ = true;
  bool uses_main_ = false;
  std::size_t watchers_ = 0;
};

} // namespace detail

/// Synchronous process scheduler.
///
/// Manages multiple processes backed by stackless_evaluator.  Uses
/// scheduling policies to select which process(es) to step.  Enforces
/// per-process resource limits (max_steps, max_time, max_memory,
/// max_messages) at each step boundary.
///
/// Implements scheduler_base — the intrinsic-facing surface the DSL calls
/// through intrinsics_context.sched.
class scheduler : public scheduler_base {
public:
  explicit scheduler(scheduling_policy policy = scheduling_policy::round_robin);
  /// Destroys the lock home first (protocol 5.7), before any other member goes; then a posted
  /// command never applied gets done(false).
  ~scheduler() override;

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

  // --- Process submission ---

  /// Parse and execute a script, returning the assigned PID.
  int execute(const std::string &script, const execute_options &opts = {});

  /// Execute a pre-parsed expression.
  int execute(const value_t &expr, const execute_options &opts = {});

  // --- Stepping ---

  /// Execute one step on the next eligible process.
  /// Returns number of steps executed (0 or 1).
  int step();

  /// Run until all processes complete or global limits reached.
  /// Returns map of pid → exit_code for terminated processes.
  ///
  /// `max_steps` counts the loop's iterations, the idle ones (nothing ready) included, and
  /// `max_time` the run's own time (a time_budget). While nothing is ready and processes sleep
  /// (or wait for a message with a time limit), run() blocks until the earliest of their
  /// deadlines instead of spinning, burning no CPU, and charges the wait to both budgets as the
  /// spin it replaces spent them: to `max_time` as run time, to `max_steps` at 100 ns an idle
  /// iteration. So it returns at once when the step budget would run out before the deadline
  /// (a frame's run(N) pump), waits when it would not (a bounded run to completion), and
  /// returns when the time budget runs out first. While only processes waiting for a message
  /// with no time limit are left, nothing can wake them in a synchronous run: run() returns at
  /// once when `max_steps` bounds it, else waits out `max_time`, else waits until stop(). A post
  /// to the scheduler's lock port (a grant for a lock waiter, made on another home's thread) or a
  /// post_control ends any such wait, and run() steps again (design 8.7; td 4.12).
  std::unordered_map<int, value_t> run(std::optional<uint64_t> max_steps = std::nullopt,
                                       std::optional<double> max_time = std::nullopt);

  /// run() without collecting the results: for a host that pumps the scheduler and reads the
  /// results it needs (get_result) or none, so a run doesn't cost a map entry for every
  /// finished process not yet reaped. Returns the process steps it took.
  uint64_t run_steps(std::optional<uint64_t> max_steps = std::nullopt,
                     std::optional<double> max_time = std::nullopt);

  /// Any thread: end the run loop at its next check, an idle wait included (td 4.12). Not
  /// async-signal-safe: it notifies the idle wait, which takes a lock on POSIX. A stop() with no
  /// run active is discarded by the next run, which starts by clearing it.
  void stop();

  /// Any thread: is the run loop active? Once this is true, a stop() ends that run.
  bool is_running() const { return running_.load(std::memory_order_acquire); }

  /// Erase a finished (terminated, killed or failed) process, freeing what it kept (its
  /// result, error and statistics). False for an unknown pid or a live one. Until then a
  /// finished process stays readable (get_result, get_process_info, get_results).
  bool reap(int pid);

  // --- Process control ---

  bool pause(int pid);
  bool resume(int pid);
  bool kill(int pid);

  /// Put a process to sleep for the given number of seconds.
  /// The process enters `waiting` status and is automatically woken
  /// by the scheduler loop once the deadline expires.
  bool sleep(int pid, double seconds) override;

  /// No-op on the synchronous scheduler (no frame boundary): `(await …)` degrades to
  /// identity. Returns false (nothing parked).
  bool yield_frame(int pid) override;

  /// Put a process into `waiting` status until a message arrives at
  /// the given state-tree path.  Returns false if the PID is invalid
  /// or the process is not in a runnable state.
  bool receive_message(int pid, const std::string &path);
  bool receive_message_until(int pid, const std::string &path, double seconds) override;

  /// Deliver a message dict to all processes waiting on `path`.
  /// Each matching process has the message pushed to its inbox,
  /// its evaluator result set to the message, and its status
  /// changed to `ready`.  Returns the number of processes woken.
  ///
  /// When no process is currently waiting on `path`, the message
  /// is stored in a per-path FIFO queue so that a future
  /// `msg-recv` call can retrieve it without blocking.  The queue
  /// is bounded by `max_pending_messages` (default 1024); excess
  /// messages are silently dropped.
  int deliver_to_receivers(const std::string &path, const value_t &msg);

  /// Pop one queued message for `path`, or nullopt if the queue
  /// is empty.  Called by `msg-recv` before suspending.
  std::optional<value_t> pop_pending_message(const std::string &path);

  /// Number of queued (undelivered) messages for `path`.
  std::size_t pending_message_count(const std::string &path) const;

  /// Total queued messages across all paths.
  std::size_t total_pending_messages() const;

  /// Maximum messages that can be queued per path (0 = unlimited).
  std::size_t max_pending_messages = 1024;

  /// Fork a running/ready process.  Returns child PID or -1 on failure.
  int fork(int pid);

  bool set_priority(int pid, int priority) override;
  std::optional<priority_info> get_priority(int pid) const override;
  priority_change set_priority_checked(int caller_pid, int pid, int priority) override;
  std::optional<priority_info> get_priority_as(int caller_pid, int pid) const override;
  bool set_priority_floor(int pid, int floor) override;
  void note_priority_clamped() override;
  /// Thread-safe: `cmd` for process `pid`, applied between steps on the scheduler's thread, at
  /// the top of the next step (step(), or run()'s next iteration). A post ends an idle run()'s
  /// wait, and run() applies what is posted before it tests has_runnable(), so a resume of the
  /// only process, or a kill of the last one, takes effect in a run with nothing else to do.
  /// Commands are applied one at a time, first in, first out. kill, pause, resume, signal and
  /// set_priority are the direct calls' host paths, unchecked (kill's reason is killed_by_user;
  /// signal "SIGKILL" kills); an unknown pid is found, and refused, then. Each one's `done`, when
  /// set, gets what its call returned; one never applied gets false from the destructor. A query
  /// runs its function, and must not throw. True once queued.
  bool post_control(int pid, control_cmd cmd);
  /// Thread-safe: whether a posted command waits to be applied (the CLI's serve loop steps for
  /// one even with nothing runnable).
  bool has_pending_control() const noexcept {
    return control_pending_.load(std::memory_order_acquire);
  }
  bool set_max_steps(int pid, uint64_t max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, uint64_t bytes);
  bool set_max_messages(int pid, uint64_t count);
  bool set_max_message_bytes(int pid, uint64_t bytes);

  // --- Signal handling ---

  /// Send a signal to a process.  SIGKILL kills immediately.
  bool send_signal(int pid, const std::string &signal);

  // --- Process info ---

  std::vector<process_info> list_processes() const;
  std::optional<process_info> get_process_info(int pid) const;
  std::optional<value_t> get_result(int pid) const;
  /// Why a process ended, when it didn't terminate normally: the kill reason, or a failed
  /// process's "Class: message". nullopt for an unknown pid or one still running or terminated.
  std::optional<std::string> get_exit_error(int pid) const;
  /// A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6), or nullopt.
  std::optional<value_t> get_exit_exception(int pid) const;
  /// Throw a failed process's exception as a dsl_exception (with the C++ exception it was made
  /// from, if any, as its original()); returns for any other process.
  void raise_exit_exception(int pid) const;
  std::unordered_map<int, value_t> get_results() const;
  scheduler_stats get_stats() const;

  // --- Accessors ---

  scheduling_policy policy() const override { return policy_; }
  const memory_tracker &mem_tracker() const { return mem_tracker_; }
  int process_count() const { return static_cast<int>(procs_.size()); }

  /// Check if any process is still runnable.
  bool has_runnable() const;

  /// PID of the process currently being stepped (-1 if none).
  int current_pid() const { return current_pid_; }

  /// Process pointer of the process currently being stepped (nullptr if none).
  const process_ptr &current_process() const {
    return current_proc_ ? *current_proc_ : no_process();
  }

  /// Queue a watch event for a process by PID.
  void queue_watch_event(int pid, process::watch_event evt);

  /// Register a watch handler with its watched path on the scheduler's process: the 5-argument
  /// form with a null root (the scheduler's watch root).
  void register_watch_handler(int pid, int watch_id, value_t handler, const std::string &path);
  /// The same, resolving `path` under `root` when it is non-null (a chrooted context's root, which
  /// the entry pins; td 4.8). The watch is read only after a write under its own root.
  void register_watch_handler(int pid, int watch_id, value_t handler, const std::string &path,
                              boost::shared_ptr<cvc::state> root) override;

  /// Remove a watch handler from the scheduler's process object.
  void unregister_watch_handler(int pid, int watch_id);

  /// Set the state-tree root for watch polling and settings. A host call (app, pycvc, the CLI),
  /// never made from a step: a chrooted state-watch keeps its own root instead (td 4.8).
  void set_watch_root(cvc::state *root);
  /// The same for a root a program's step gives (state-watch's): kept by `pin` while it is the
  /// watch root (scheduler_base).
  void set_watch_root(cvc::state *root, boost::shared_ptr<cvc::state> pin) override;
  /// The root set_watch_root() gave, or null.
  cvc::state *watch_root() const override { return watch_root_; }

  /// The watched nodes poll_watches() has read so far, one for each watch it reads. A watch is
  /// read only after a write under the root it resolves under (td 4.8: per-root gating).
  uint64_t watch_polls() const noexcept { return watch_polls_; }

  /// Set a unique identifier for this scheduler instance.
  /// Used as the key under `state_exec.schedulers.<id>` in the state tree.
  void set_id(const std::string &id) {
    id_ = id;
    rate_valid_ = false; // the id decides which limits.async_steps_per_cpu_ms applies
  }
  const std::string &id() const { return id_; }

  /// Load settings from the state tree.
  ///
  /// Resolution order (per setting):
  ///   1. Per-scheduler: `state_exec.schedulers.<id>.<key>` (if id set)
  ///   2. Global default: `state_exec.defaults.<key>`
  ///   3. Hardcoded fallback
  ///
  /// After resolution, effective values are published back to the
  /// per-scheduler subtree so they are visible in the state tree.
  /// Requires set_watch_root() and (for per-scheduler) set_id() first.
  ///
  /// Settings: `max_pending_messages`, the parser's `parser.*`
  /// (parse_options_from_state) and the deep-data `data.*`
  /// (data_options_from_state), which apply to every process submitted
  /// afterwards.
  void load_settings();

  /// The parser settings execute() parses scripts with (defaults until
  /// load_settings() runs).
  const parse_options &parser_settings() const { return parse_options_; }

  /// The deep-data settings execute() gives processes (defaults until
  /// load_settings() runs).
  const data_options &data_settings() const { return data_options_; }

  /// Well-known state tree paths for scheduler configuration.
  static constexpr const char *settings_root = "state_exec";
  static constexpr const char *defaults_prefix = "state_exec.defaults";
  static constexpr const char *sched_prefix = "state_exec.schedulers";

  /// The effective limits.max_alloc_bytes (kDefaultMaxAllocBytes until load_settings() reads
  /// the tree); each process submitted afterwards takes it as its max_alloc_bytes.
  uint64_t max_alloc_bytes() const { return max_alloc_bytes_; }

  /// The steps a CPU millisecond of `proc`'s async calls is charged: its own
  /// execute_options::async_steps_per_cpu_ms, else limits.async_steps_per_cpu_ms read from the
  /// tree now (no load_settings() needed: async_steps_per_cpu_ms_from_state), else the default.
  /// The tree's value is read again only once something under its `state_exec` node changed
  /// since the last read, so a change applies to the next call while a quiet tree costs no read.
  /// The sync scheduler runs a call's kernel inline, in the step that makes it.
  double async_steps_per_cpu_ms(const process &proc) override;

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

private:
  std::string id_;
  uint64_t max_alloc_bytes_ = kDefaultMaxAllocBytes;
  double published_rate_ = -1.0; // the effective.async_steps_per_cpu_ms last published
  scheduling_policy policy_;
  parse_options parse_options_;
  data_options data_options_;
  heap_options heap_options_;
  uint32_t cycle_check_ = 0; // steps, for checking every 64th whether a cycle collection is due
  int next_pid_ = 1;
  int rr_index_ = 0;
  /// run()'s flags, which stop() and is_running() touch from any thread (td B17). A run clears
  /// stop_requested_ before it publishes running_, so a stop() made once is_running() is true is
  /// never lost. No data rides on them: the loop reads stop_requested_ relaxed.
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  static_assert(std::atomic<bool>::is_always_lock_free, "stop() must not take a lock");
  uint64_t total_steps_ = 0;
  uint64_t priority_clamped_ = 0; // spawns clamped to the spawner's floor (note_priority_clamped)
  int current_pid_ = -1;
  const process_ptr *current_proc_ = nullptr; // the stepping process's entry in procs_
  int reap_after_step_ = -1; // reaped while it stepped: erased once the step is over

  stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;
  cvc::state *watch_root_ = nullptr;
  /// What keeps watch_root_ alive when a step gave it (set_watch_root's pin); null for a host's.
  /// Declared before what connects to the root, so it goes after them.
  boost::shared_ptr<cvc::state> watch_root_pin_;

  /// Every process by pid (finished ones until reaped), its run queue, timers and waiters.
  detail::process_table procs_;

  /// A steady_clock reading the next step may start from: the end of the step before, when
  /// nothing ran between the two but the scheduler's own bookkeeping (within one run()).
  using clock = std::chrono::steady_clock;
  clock::time_point chain_now_{};
  bool chain_valid_ = false;
  double chain_cpu_ = -1.0; // the thread's CPU time then, when it was read (else < 0)
  clock::time_point now_for_step();

  /// Counts tree changes under watch_root_ while a watch resolves under it: poll_watches() reads
  /// those watched nodes only when the count moved. A watch with a root of its own is counted in
  /// watch_roots_ instead (td 4.8).
  detail::tree_changes tree_changes_;
  uint64_t watch_seen_ = 0;
  detail::watch_roots watch_roots_;
  uint64_t watch_polls_ = 0; // watched nodes read (watch_polls())
  /// limits.async_steps_per_cpu_ms as last read from the tree, and the settings version then.
  detail::settings_changes settings_changes_;
  uint64_t rate_seen_ = 0;
  bool rate_valid_ = false;
  double rate_ = kDefaultAsyncStepsPerCpuMs;
  /// run()'s idle wait, made at construction (td 4.12). The lock home shares it with its port, so
  /// a post ends the wait (protocol 2.3), and stop() notifies it.
  std::shared_ptr<detail::wakeup> idle_;
  std::vector<process *> woken_; // reused by deliver_to_receivers

  /// Per-path FIFO queue for messages sent when no receiver is waiting.
  std::unordered_map<std::string, std::queue<value_t>> pending_messages_;

  static const process_ptr &no_process();

  /// Select the next process to run based on scheduling policy.
  const process_ptr *select_process();

  /// One step of the run loop (step() starts a fresh clock chain). It first applies the lock
  /// port's posts, when there are any (protocol 5.9).
  int step_once();
  /// run()'s wait while nothing is ready, charged to its budgets (`steps`, the loop's
  /// iterations so far, and `budget`, running): false to end the run. A post to the lock port or
  /// to post_control ends it (true: the run's loop applies the post).
  bool idle_wait(std::optional<uint64_t> max_steps, uint64_t &steps, time_budget *budget);

  /// Execute one step on a specific process and check resource limits.
  void execute_process_step(const process_ptr &pp);

  /// Handle pending signals for a process.
  void handle_signal(process &proc);

  /// Handle pending watch events for a process.
  void handle_watch_event(process &proc);

  /// Restore process state after signal/watch handler completes.
  void restore_from_signal(process &proc);

  /// Poll all watch_handlers entries for value changes, queuing events.
  void poll_watches();

  /// Check resource limits after a step.  May kill the process.
  void check_limits(process &proc);
  /// check_limits with the steady_clock reading the step ended at.
  void check_limits(process &proc, clock::time_point now);

  /// Mark a process as terminated with a result.
  void terminate_process(process &proc, value_t result);

  /// Mark a process as killed with an error.
  void kill_process(process &proc, const std::string &reason);

  /// Mark a process as failed with the exception it didn't catch.
  void fail_process(process &proc, value_t exc, std::exception_ptr original);

  /// After a process ended: note it for the cycle collector and, unless it is the one stepping
  /// now, release its evaluation state (process::release_state).
  void release_if_idle(process &proc);

  /// Build process_info from a process.
  process_info make_info(const process &proc) const;

  /// What this scheduler gives its lock home (design 8.4).
  struct lock_ops_impl final : detail::lock_sched_ops {
    explicit lock_ops_impl(scheduler &sched) : s(sched) {}
    scheduler &s;
    bool park_lock(process &p) override;
    void ready_lock_waiter(process &p) override;
    void relevel(process &p) override;
    std::size_t lock_expired_count() const override;
    void take_lock_expired(std::vector<process *> &out) override;
    process *find_process(int pid) override;
    void for_each_process(const std::function<void(process &)> &fn) override;
  };
  lock_ops_impl lock_ops_{*this};
  /// Made by locks() or set_lock_manager(); null until a with-lock needs it.
  std::unique_ptr<detail::lock_home> home_;
  std::vector<process *> lock_expired_; // reused by step_once's lock expiries
  /// An expiry pass (at `now`) threw: the lock waiters it had not timed out get a deadline due
  /// `now`, back on the timer heap, and lock_expired_ is emptied.
  void keep_lock_expiries(clock::time_point now);

  /// post_control's commands, from any thread: guarded by control_mutex_, applied at the top of
  /// step_once and of run()'s loop. control_pending_ is set (under the mutex) while there are
  /// any, so a step finds none without the lock.
  std::mutex control_mutex_;
  std::vector<std::pair<int, control_cmd>> controls_;
  std::atomic<bool> control_pending_{false};
  /// Apply the queued commands in order. One that throws gets done(false), and the rest go back
  /// to the front of the queue before the exception propagates.
  void apply_controls();
  void apply_control(int pid, control_cmd &cmd);
};

/// limits.max_alloc_bytes from the state tree under `root`:
/// `state_exec.schedulers.<scheduler_id>.limits.max_alloc_bytes` (when scheduler_id is not
/// empty), then `state_exec.defaults.limits.max_alloc_bytes`, then `base`. A missing or
/// unparsable value is skipped. 0 means no limit of its own; the process's max_memory still
/// bounds each allocation.
uint64_t max_alloc_bytes_from_state(cvc::state &root, const std::string &scheduler_id,
                                    uint64_t base = kDefaultMaxAllocBytes);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_SCHEDULER_H

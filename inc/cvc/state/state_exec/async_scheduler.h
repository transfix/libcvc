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
#include <cvc/state/state_exec/async_stackless_evaluator.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/heap.h> // cycle_registry
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/task.h>
#include <cvc/state/state_exec/types.h>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
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
  void post_message(const std::string &path, const value_t &msg);

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

  bool set_priority(int pid, int priority) override;
  std::optional<priority_info> get_priority(int pid) const override;
  priority_change set_priority_checked(int caller_pid, int pid, int priority) override;
  std::optional<priority_info> get_priority_as(int caller_pid, int pid) const override;
  bool set_priority_floor(int pid, int floor) override;
  void note_priority_clamped() override;
  /// Thread-safe: `cmd` for process `pid`, applied between steps on the scheduler's thread by the
  /// next drain (a step's, or drain_ingress()'s), as post_message's posts are; a driver blocked
  /// in wait_for_work() wakes. Commands are applied one at a time, first in, first out, after the
  /// drain's messages and completions, whatever the order they were all posted in (S0.7). kill,
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

  /// Per-path FIFO for messages sent when no receiver is waiting (mirrors sync scheduler).
  std::unordered_map<std::string, std::queue<value_t>> pending_messages_;

  /// Thread-safe ingress for cross-thread post_message(): guarded by ingress_mutex_, drained
  /// on the scheduler thread by drain_ingress() (public) and applied via deliver_to_receivers.
  std::mutex ingress_mutex_;
  std::vector<std::pair<std::string, value_t>> ingress_;
  std::vector<std::shared_ptr<async_call>> completions_; // settled calls, guarded likewise
  std::vector<std::pair<int, control_cmd>> controls_;    // post_control's, guarded likewise
  /// Set (under ingress_mutex_) while ingress_, completions_ or controls_ holds something, so a
  /// drain finds an empty ingress without the lock.
  std::atomic<bool> ingress_pending_{false};
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
  // A drain that threw: what it had not applied (batch[m..], finished[f..], controls[c..]) goes
  // back to the front of the ingress, ahead of what was posted since (S0.7).
  void requeue_ingress(std::vector<std::pair<std::string, value_t>> &batch, std::size_t m,
                       std::vector<std::shared_ptr<async_call>> &finished, std::size_t f,
                       std::vector<std::pair<int, control_cmd>> &controls, std::size_t c) noexcept;
  uint64_t run_loop(std::optional<uint64_t> max_steps, std::optional<double> max_time);
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
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_SCHEDULER_H

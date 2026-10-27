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
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

class async_call;

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

  // --- Process submission ---

  int execute(const std::string &script, const execute_options &opts = {}) override;

  int execute(const value_t &expr, const execute_options &opts = {}) override;

  // --- Stepping (coroutines) ---

  /// Execute one step as a coroutine (the same step as sync_step()).
  task<int> step();

  /// Run until all done or limits.
  task<std::unordered_map<int, value_t>> run(std::optional<uint64_t> max_steps = std::nullopt,
                                             std::optional<double> max_time = std::nullopt);

  /// Blocking wrappers: the same steps, with no coroutine frame.
  int sync_step();
  std::unordered_map<int, value_t> sync_run(std::optional<uint64_t> max_steps = std::nullopt,
                                            std::optional<double> max_time = std::nullopt);
  /// sync_run() without collecting the results: for a host that pumps the scheduler and reads
  /// the results it needs (get_result) or none, so a run doesn't cost a map entry for every
  /// finished process not yet reaped. Returns the steps it took.
  uint64_t sync_run_steps(std::optional<uint64_t> max_steps = std::nullopt,
                          std::optional<double> max_time = std::nullopt);

  void stop();
  bool is_running() const { return running_; }

  // --- Process control ---

  bool pause(int pid) override;
  bool resume(int pid) override;
  bool kill(int pid) override;
  int fork(int pid) override;

  /// Kill every live process tagged with `owner` — an Ariadne Runtime reaping its
  /// process group on teardown. Returns the number killed. Parked processes
  /// (sleep/msg-recv/await) are killed too, so nothing lingers after the document
  /// that owned them closes. `owner` == "" reaps the unowned group.
  int kill_owner(const std::string &owner);

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
  /// run: a post_message() or a settled async call (from any thread), a sleeper's or a timed
  /// wait's deadline, an async call's registration timeout, the next charge of a call whose
  /// process has a step or time limit (every millisecond while one runs: the charge may find
  /// the limit reached, docs/STATE_EXEC_EXCEPTIONS.md 3.8), or `deadline`, whichever comes
  /// first. Returns at once when a process is ready or such work is already waiting. True when
  /// there is work; false when `deadline` passed with none. Processes parked until the next
  /// frame (yield_frame) are the driver's to wake (wake_awaiting): they don't end the wait (see
  /// awaiting_frame_count()). A sleeper's deadline is kept to well under a millisecond, which
  /// on Windows costs up to ~0.6 ms of spinning at its end (detail::wakeup); the other
  /// deadlines may be met up to ~0.5 ms late, burning no CPU.
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

  bool set_priority(int pid, int priority);
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
  /// Its async calls still in flight (futures it never awaited) are cancelled.
  bool reap(int pid);
  std::unordered_map<int, value_t> get_results() const;
  scheduler_stats get_stats() const override;

  // --- Accessors ---

  scheduling_policy policy() const { return policy_; }
  const memory_tracker &mem_tracker() const { return mem_tracker_; }
  int process_count() const { return static_cast<int>(procs_.size()); }
  bool has_runnable() const;

  /// PID / process pointer of the process currently being stepped (-1 / nullptr
  /// if none).  Set around evaluator_.step() so intrinsics can query `self`.
  int current_pid() const override { return current_pid_; }
  const process_ptr &current_process() const override {
    return current_proc_ ? *current_proc_ : no_process();
  }

  void queue_watch_event(int pid, process::watch_event evt);

  void register_watch_handler(int pid, int watch_id, value_t handler,
                              const std::string &path) override;
  void unregister_watch_handler(int pid, int watch_id) override;

  void set_watch_root(cvc::state *root) override;

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

  scheduling_policy policy_;
  std::string id_;
  double published_rate_ = -1.0; // the effective.async_steps_per_cpu_ms last published
  int next_pid_ = 1;
  int rr_index_ = 0;
  bool running_ = false;
  bool stop_requested_ = false;
  uint64_t total_steps_ = 0;

  int current_pid_ = -1;
  uint32_t cycle_check_ = 0; // steps, for checking every 64th whether a cycle collection is due
  const process_ptr *current_proc_ = nullptr; // the stepping process's entry in procs_
  int reap_after_step_ = -1; // reaped while it stepped: erased once the step is over

  // The scheduled evaluator IS the async_stackless_evaluator (roadmap §8.12). Its step never
  // suspends, so the scheduler calls its step_now() directly (no coroutine frame per step).
  async_stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;
  cvc::state *watch_root_ = nullptr;
  parse_options parser_defaults_;
  data_options data_defaults_;

  /// Every process by pid (finished ones until reaped), its run queue, timers and waiters.
  detail::process_table procs_;

  /// A steady_clock reading the next step may start from (see scheduler.h).
  using clock = std::chrono::steady_clock;
  clock::time_point chain_now_{};
  bool chain_valid_ = false;
  double chain_cpu_ = -1.0;
  clock::time_point now_for_step();

  /// Counts tree changes under watch_root_ while processes have watches or async calls are in
  /// flight: poll_watches() reads the watched nodes, and async_steps_per_cpu_ms() the rate, only
  /// once the count moved.
  detail::tree_changes tree_changes_;
  uint64_t watch_seen_ = 0;
  uint64_t rate_seen_ = 0;
  bool rate_valid_ = false;
  double rate_ = kDefaultAsyncStepsPerCpuMs;
  uint64_t rate_publications_ = 0; // tree writes publishing the effective rate
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

  struct charged_process {
    process *proc;
    int pid;
  };
  std::vector<process *> woken_;         // reused by the waiter wakes
  std::vector<charged_process> charged_; // reused by charge_async_calls

  static const process_ptr &no_process();

  /// Per-path FIFO for messages sent when no receiver is waiting (mirrors sync scheduler).
  std::unordered_map<std::string, std::queue<value_t>> pending_messages_;

  /// Thread-safe ingress for cross-thread post_message(): guarded by ingress_mutex_, drained
  /// on the scheduler thread by drain_ingress() (public) and applied via deliver_to_receivers.
  std::mutex ingress_mutex_;
  std::vector<std::pair<std::string, value_t>> ingress_;
  std::vector<std::shared_ptr<async_call>> completions_; // settled calls, guarded likewise
  /// Set (under ingress_mutex_) while ingress_ or completions_ holds something, so a drain finds
  /// an empty ingress without the lock.
  std::atomic<bool> ingress_pending_{false};
  /// A wait_for_work() is (about to be) blocked: a post notifies wakeup_.
  std::atomic<bool> idle_waiting_{false};
  std::unique_ptr<detail::wakeup> wakeup_;
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
  // charge_async_calls from the `i`th call on, over a copy of the list (a timeout changes it).
  void charge_async_calls_from(std::size_t i, clock::time_point now);
  // Cancel and drop the calls of `pid` (killed or reaped).
  void cancel_async_calls(int pid);
  // Resume the processes waiting on `channel` in the await_call phase.
  void wake_async_waiters(const std::string &channel);

  const process_ptr *select_process();
  // One step of the run loop (sync_step() and step() start a fresh clock chain).
  int step_once();
  // What a step does before picking a process: the ingress, the calls, the due timers.
  void drain(bool in_step);
  uint64_t run_loop(std::optional<uint64_t> max_steps, std::optional<double> max_time);
  void execute_process_step(const process_ptr &pp);
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
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_SCHEDULER_H

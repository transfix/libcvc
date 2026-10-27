#include <algorithm>
#include <chrono>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_call.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/scheduler.h>         // max_alloc_bytes_from_state
#include <cvc/state/state_exec/state_value_codec.h> // snapshot / restore

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

async_scheduler::async_scheduler(scheduling_policy policy)
    : policy_(policy), evaluator_(builtins::make_default_environment()),
      procs_(policy != scheduling_policy::round_robin),
      wakeup_(std::make_unique<detail::wakeup>()) {}

const process_ptr &async_scheduler::no_process() {
  static const process_ptr none;
  return none;
}

// ---------------------------------------------------------------------------
// Process submission (same as sync scheduler)
// ---------------------------------------------------------------------------

int async_scheduler::execute(const std::string &script, const execute_options &opts) {
  detail::check_not_in_async_kernel("async_scheduler::execute");
  int pid = next_pid_++;
  process_ptr pp = make_process(); // built in place: no move of its tables
  process &proc = *pp;
  proc.pid = pid;
  proc.name = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
  proc.status = process_status::ready;
  proc.priority = opts.priority;
  proc.uid = opts.uid;
  proc.gid = opts.gid;
  proc.root_path = opts.root_path;
  proc.owner = opts.owner;
  proc.max_steps = opts.max_steps;
  proc.max_time = opts.max_time;
  proc.max_step_time = opts.max_step_time;
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  // No load_settings() here: read the allocation limit from the tree on every submission.
  proc.max_alloc_bytes =
      watch_root_ ? max_alloc_bytes_from_state(*watch_root_, {}) : kDefaultMaxAllocBytes;
  proc.heap_check_steps =
      (watch_root_ ? heap_options_from_state(*watch_root_) : heap_options{}).check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  // No load_settings() here: read the parser settings from the tree on every submission, over
  // this scheduler's own defaults (set_parser_defaults).
  const parse_options popts =
      watch_root_ ? parse_options_from_state(*watch_root_, {}, parser_defaults_) : parser_defaults_;
  proc.state = evaluator_.create_state(script, opts.env, popts);
  proc.state.data_step_nodes = data_settings().step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

int async_scheduler::execute(const value_t &expr, const execute_options &opts) {
  detail::check_not_in_async_kernel("async_scheduler::execute");
  int pid = next_pid_++;
  process_ptr pp = make_process(); // built in place: no move of its tables
  process &proc = *pp;
  proc.pid = pid;
  proc.name = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
  proc.status = process_status::ready;
  proc.priority = opts.priority;
  proc.uid = opts.uid;
  proc.gid = opts.gid;
  proc.root_path = opts.root_path;
  proc.owner = opts.owner;
  proc.max_steps = opts.max_steps;
  proc.max_time = opts.max_time;
  proc.max_step_time = opts.max_step_time;
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  // No load_settings() here: read the allocation limit from the tree on every submission.
  proc.max_alloc_bytes =
      watch_root_ ? max_alloc_bytes_from_state(*watch_root_, {}) : kDefaultMaxAllocBytes;
  proc.heap_check_steps =
      (watch_root_ ? heap_options_from_state(*watch_root_) : heap_options{}).check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(expr, opts.env);
  proc.state.data_step_nodes = data_settings().step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

// ---------------------------------------------------------------------------
// Process selection (same logic as sync)
// ---------------------------------------------------------------------------

const process_ptr *async_scheduler::select_process() {
  if (policy_ == scheduling_policy::priority)
    return procs_.pick_first();
  return procs_.pick_round_robin(rr_index_);
}

// ---------------------------------------------------------------------------
// Step execution (same logic as sync)
// ---------------------------------------------------------------------------

async_scheduler::clock::time_point async_scheduler::now_for_step() {
  if (!chain_valid_) {
    chain_now_ = clock::now();
    chain_cpu_ = -1.0;
    chain_valid_ = true;
  }
  return chain_now_;
}

void async_scheduler::execute_process_step(const process_ptr &pp) {
  process &proc = *pp;
  // Between steps: free the reference cycles programs leaked, once enough piled up (heap.h).
  // Checked every 64 steps: the check reads thread-local state.
  if ((++cycle_check_ & 63) == 0 && cycle_collection_due()) {
    collect_cycles();
    chain_valid_ = false;
  }

  // Track which process is currently executing so intrinsics can query it (self/ps).
  current_pid_ = proc.pid;
  current_proc_ = &pp;
  struct guard {
    int &pid;
    const process_ptr *&pp;
    ~guard() {
      pid = -1;
      pp = nullptr;
    }
  } reset_guard{current_pid_, current_proc_};
  // Once this step is over, a process it finished lets go of what it evaluated with.
  struct release_guard {
    process &p;
    ~release_guard() {
      if (is_finished(p.status))
        p.release_state();
    }
  } release_dead{proc};

  // A process still reading its script (parser.step_tokens) takes signals and watch events once
  // the script is read: their handlers run on the evaluation stack, which doesn't exist yet. One
  // in the middle of deep-data work (data.step_nodes) takes them once that is done: the work's
  // call frame waits on top of the stack.
  const bool can_interrupt = !proc.in_signal_handler && !proc.in_watch_handler &&
                             !proc.state.reading && !proc.state.data_work;

  if (!proc.pending_signals.empty() && can_interrupt) {
    chain_valid_ = false;
    handle_signal(proc);
    return;
  }

  if (!proc.pending_watch_events.empty() && can_interrupt) {
    chain_valid_ = false;
    handle_watch_event(proc);
    return;
  }

  // One clock reading starts the step and serves every time budget of it (see scheduler.cpp).
  procs_.set_status(proc, process_status::running);
  const clock::time_point step_start = now_for_step();
  const double chained_cpu = chain_cpu_;
  chain_valid_ = false; // until this step ends plainly
  proc.last_run_start = step_start;
  // Async calls that run inline in this step are charged as calls (3.8), not as the step.
  proc.step_async_wall = 0.0;
  proc.step_async_cpu = 0.0;

  // Arm the cooperative deadline for this step so an uncapped nested evaluator or a
  // long-running builtin aborts at the process's remaining time budget too — the same
  // guarantee the sync scheduler has (scheduler.cpp). max_time <= 0 means unlimited.
  std::optional<double> step_budget = proc.time_left(step_start);
  // §13.8 per-STEP cap: bound a SINGLE step's time too (fresh each step, never accumulates),
  // so a resident with total max_time == 0 still can't HANG the host's drain on an in-step loop. A
  // healthy step is far under the cap; a per-node DSL loop yields per step and is bounded by the
  // pump's own per-drain cap instead. Take the tighter of the two.
  if (proc.max_step_time > 0.0)
    step_budget = step_budget ? std::min(*step_budget, proc.max_step_time) : proc.max_step_time;
  eval_deadline_guard step_deadline(step_budget, step_start);
  // Bound what one native call in this step may allocate (see scheduler.cpp).
  // Armed only when it changes something (see scheduler.cpp).
  std::optional<eval_alloc_guard> step_alloc;
  if (proc.max_memory > 0 || proc.max_alloc_bytes != kDefaultMaxAllocBytes)
    step_alloc.emplace(proc.alloc_limit(
        proc.max_memory > 0 ? mem_tracker_.current_bytes(proc.pid) + proc.heap_bytes : 0));
  const bool charge_cpu = proc.charges_cpu_time(step_start);
  const double cpu0 = charge_cpu ? (chained_cpu >= 0.0 ? chained_cpu : thread_cpu_seconds()) : 0.0;
  // One async_stackless_evaluator step, run to its end here: the step never suspends inside
  // (its coroutine form only yields after it), so it is called directly, with no coroutine
  // frame per step.
  bool done = false;
  try {
    done = evaluator_.step_now(proc.state);
  } catch (const std::exception &e) {
    // A signal or watch handler's uncaught exception is raised in the program where the handler
    // interrupted it (docs/STATE_EXEC_EXCEPTIONS.md 2.4), for the program's own try to catch.
    bool raised_in_program = false;
    if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.uncaught) {
      value_t exc = std::move(*proc.state.uncaught);
      proc.state.uncaught.reset();
      restore_from_signal(proc);
      evaluator_.raise(proc.state, std::move(exc));
      raised_in_program = !proc.state.uncaught;
    }
    if (!raised_in_program && proc.state.uncaught) {
      // An exception the program didn't catch ends it `failed`, with the exception kept for the
      // host (docs/STATE_EXEC_EXCEPTIONS.md 2.6).
      proc.accumulated_time +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - proc.last_run_start)
              .count();
      const auto *thrown = dynamic_cast<const dsl_exception *>(&e);
      fail_process(proc, *proc.state.uncaught, thrown ? thrown->original() : nullptr);
      return;
    }
    if (!raised_in_program) {
      // A runtime error thrown by a step — a throwing intrinsic (e.g. a channel-policy refusal)
      // or a bad-arg builtin — must NOT propagate out of the pump: it would escape sync_run() and
      // crash the host's per-frame drain loop. Mark the process killed with the reason
      // (mirroring the sync scheduler's runtime-error handling); a caller sweep reports a killed
      // process fail-safe. This is the async lane's analogue of the sync evaluator's
      // re-throw-to-caller, made drain-safe.
      proc.accumulated_time +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - proc.last_run_start)
              .count();
      // The step's deadline (max_time, or the per-step cap) cutting a long loop inside the step
      // is the process running out of time, reported as check_limits() reports it between steps.
      const bool out_of_time =
          dynamic_cast<const evaluation_timeout *>(&e) != nullptr && step_deadline.expired();
      kill_process(proc, out_of_time ? "time_limit_exceeded" : e.what());
      return;
    }
  }

  const clock::time_point now = clock::now();
  // A parking intrinsic (sleep / yield-frame / receive_message) already closed out this running
  // slice into accumulated_time and flipped status to `waiting`; accrue here only while still
  // running, so a parking step's slice is not double-counted (mirrors process::elapsed_time()'s own
  // status guard — previously the unconditional add booked the pre-park slice twice).
  if (proc.status == process_status::running)
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  double cpu1 = -1.0;
  if (charge_cpu) {
    cpu1 = thread_cpu_seconds();
    proc.charge_cpu_time(std::max(0.0, cpu1 - cpu0 - proc.step_async_cpu));
  }
  // Charge the step's time to what it was spent on (eval, read or data).
  proc.state.stats.add_step_time(std::chrono::duration<double>(now - step_start).count() -
                                 proc.step_async_wall);
  // Heap accounting: take a finished measurement, start the next when due (heap.h).
  if (proc.max_memory > 0 || proc.state.heap_measured)
    heap_after_step(proc, step_alloc ? step_alloc->allocated() : 0,
                    mem_tracker_.current_bytes(proc.pid));

  if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.done) {
    restore_from_signal(proc);
    procs_.set_status(proc, process_status::ready);
    check_limits(proc, now);
    return;
  }

  check_limits(proc, now);
  if (proc.status == process_status::killed)
    return;

  if (done) {
    terminate_process(proc, proc.state.result); // runs on_complete: not the next step's time
  } else {
    if (proc.status == process_status::running) {
      // Only reset to ready if the intrinsic didn't change status (sleep/msg-recv set waiting).
      procs_.set_status(proc, process_status::ready);
    }
    chain_now_ = now;
    chain_cpu_ = cpu1;
    chain_valid_ = true;
  }
}

// ---------------------------------------------------------------------------
// Stepping
// ---------------------------------------------------------------------------

void async_scheduler::drain(bool in_step) {
  // Cross-thread posts and settled calls, when there are any: the flag spares an empty ingress
  // the lock.
  if (ingress_pending_.load(std::memory_order_acquire)) {
    std::vector<std::pair<std::string, value_t>> batch;
    std::vector<std::shared_ptr<async_call>> finished;
    {
      std::lock_guard<std::mutex> lock(ingress_mutex_);
      batch.swap(ingress_);
      finished.swap(completions_);
      ingress_pending_.store(false, std::memory_order_relaxed);
    }
    // On the scheduler thread now: deliver_to_receivers mutates process state, which is safe
    // here (no other thread touches the scheduler). A message with no waiter falls into the
    // per-path pending FIFO, exactly as an on-thread msg-send would.
    for (auto &[path, msg] : batch)
      deliver_to_receivers(path, msg);
    for (const std::shared_ptr<async_call> &call : finished)
      finish_async_call(call);
    chain_valid_ = false;
  }
  if (!async_calls_.empty()) {
    charge_async_calls(in_step ? now_for_step() : clock::now());
    chain_valid_ = false;
  }
  // Sleepers and timed waits ((await f seconds)) that are due wake, on the heap's deadlines.
  if (procs_.timed_count() > 0) {
    const clock::time_point now = in_step ? now_for_step() : clock::now();
    if (procs_.timer_due(now))
      procs_.wake_due(now);
  }
}

int async_scheduler::step_once() {
  detail::check_not_in_async_kernel("async_scheduler::step");
  // Apply any cross-thread post_message() deliveries on THIS (scheduler) thread, wake sleeping
  // processes whose deadline has passed, then poll watched paths: once a step.
  drain(true);
  poll_watches();
  const process_ptr *pp = select_process();
  if (!pp)
    return 0;
  execute_process_step(*pp);
  ++total_steps_;
  if (reap_after_step_ >= 0) {
    procs_.erase(reap_after_step_);
    reap_after_step_ = -1;
  }
  return 1;
}

task<int> async_scheduler::step() {
  chain_valid_ = false;
  co_return step_once();
}

std::unordered_map<int, value_t> async_scheduler::run_loop(std::optional<uint64_t> max_steps,
                                                           std::optional<double> max_time) {
  running_ = true;
  stop_requested_ = false;
  chain_valid_ = false;
  uint64_t steps = 0;
  // max_time is a time_budget: the run's own time, not time its thread spent descheduled (a
  // per-frame pump on a loaded machine still finishes a quick action within its frame).
  std::optional<time_budget> budget;
  if (max_time) {
    budget.emplace(*max_time);
    budget->start();
  }

  while (!stop_requested_ && has_runnable()) {
    if (max_steps && steps >= *max_steps)
      break;
    if (budget && budget->spent(now_for_step()))
      break;
    step_once();
    ++steps;
  }

  running_ = false;
  chain_valid_ = false;
  return get_results();
}

task<std::unordered_map<int, value_t>> async_scheduler::run(std::optional<uint64_t> max_steps,
                                                            std::optional<double> max_time) {
  co_return run_loop(max_steps, max_time);
}

int async_scheduler::sync_step() {
  chain_valid_ = false;
  return step_once();
}

std::unordered_map<int, value_t> async_scheduler::sync_run(std::optional<uint64_t> max_steps,
                                                           std::optional<double> max_time) {
  return run_loop(max_steps, max_time);
}

void async_scheduler::stop() { stop_requested_ = true; }

bool async_scheduler::has_runnable() const { return procs_.runnable_count() > 0; }

bool async_scheduler::wait_for_work(clock::time_point deadline) {
  detail::check_not_in_async_kernel("async_scheduler::wait_for_work");
  for (;;) {
    if (procs_.runnable_count() > 0 || ingress_pending_.load(std::memory_order_acquire))
      return true;
    const std::optional<clock::time_point> timer = procs_.next_timer();
    const clock::time_point now = clock::now();
    if (timer && *timer <= now)
      return true;
    if (now >= deadline)
      return false;
    const clock::time_point until = timer ? std::min(*timer, deadline) : deadline;
    // Announce the wait, then look at the ingress once more: a post either lands before this
    // look (seen here) or sees idle_waiting_ and notifies (both sides are seq_cst).
    idle_waiting_.store(true, std::memory_order_seq_cst);
    if (!ingress_pending_.load(std::memory_order_seq_cst))
      wakeup_->wait_until(until);
    idle_waiting_.store(false, std::memory_order_relaxed);
  }
}

bool async_scheduler::wait_for_work(double seconds) {
  return wait_for_work(clock::now() + std::chrono::duration_cast<clock::duration>(
                                          std::chrono::duration<double>(std::max(0.0, seconds))));
}

void async_scheduler::notify_work() {
  if (idle_waiting_.load(std::memory_order_seq_cst))
    wakeup_->notify();
}

// ---------------------------------------------------------------------------
// Cooperative sleep + inter-process messaging (parity with the sync scheduler;
// bodies mirror scheduler.cpp so the intrinsics behave identically on either).
// ---------------------------------------------------------------------------

bool async_scheduler::sleep(int pid, double seconds) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  const auto now = std::chrono::steady_clock::now();
  if (proc.status == process_status::running)
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  proc.wake_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(seconds));
  procs_.set_status(proc, process_status::waiting); // a timer for wake_time
  return true;
}

bool async_scheduler::yield_frame(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.awaiting_frame = true;
  procs_.set_status(proc, process_status::waiting); // among the frame waiters
  return true;
}

int async_scheduler::wake_awaiting() {
  std::vector<process *> list;
  list.swap(woken_);
  procs_.take_awaiting(list);
  for (process *proc : list) {
    proc->awaiting_frame = false;
    procs_.set_status(*proc, process_status::ready);
  }
  const int woken = static_cast<int>(list.size());
  list.clear();
  woken_.swap(list);
  return woken;
}

bool async_scheduler::receive_message(int pid, const std::string &path) {
  detail::check_not_in_async_kernel("async_scheduler::receive_message");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.recv_path = path;
  proc.recv_deadline.reset();
  procs_.set_status(proc, process_status::waiting); // indexed under its channel
  return true;
}

bool async_scheduler::receive_message_until(int pid, const std::string &path, double seconds) {
  if (!receive_message(pid, path))
    return false;
  process &proc = *procs_.find(pid);
  proc.recv_deadline = std::chrono::steady_clock::now() +
                       std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                           std::chrono::duration<double>(std::max(0.0, seconds)));
  procs_.update(proc); // a timer for the deadline
  return true;
}

int async_scheduler::deliver_to_receivers(const std::string &path, const value_t &msg) {
  detail::check_not_in_async_kernel("async_scheduler::deliver_to_receivers");
  // The processes waiting on `path`, from the receiver index (no scan).
  std::vector<process *> list;
  list.swap(woken_);
  procs_.take_receivers(path, list);
  for (process *proc : list) {
    proc->recv_path.reset();
    proc->recv_deadline.reset();
    proc->state.result = msg;
    // A call parked in the await_call phase takes the message; otherwise patch the nil
    // placeholder that msg-recv left in the parent frame.
    if (!deliver_await(proc->state, msg) && !proc->state.stack.empty() &&
        !proc->state.stack.back().results.empty())
      proc->state.stack.back().results.back() = msg;
    procs_.set_status(*proc, process_status::ready);
  }
  const int woken = static_cast<int>(list.size());
  list.clear();
  woken_.swap(list);
  // If no process was waiting, queue for later msg-recv calls.
  if (woken == 0) {
    auto &q = pending_messages_[path];
    if (max_pending_messages == 0 || q.size() < max_pending_messages)
      q.push(msg);
  }
  return woken;
}

std::optional<value_t> async_scheduler::pop_pending_message(const std::string &path) {
  auto it = pending_messages_.find(path);
  if (it == pending_messages_.end() || it->second.empty())
    return std::nullopt;
  auto msg = std::move(it->second.front());
  it->second.pop();
  if (it->second.empty())
    pending_messages_.erase(it);
  return msg;
}

std::size_t async_scheduler::pending_message_count(const std::string &path) const {
  auto it = pending_messages_.find(path);
  if (it == pending_messages_.end())
    return 0;
  return it->second.size();
}

std::size_t async_scheduler::total_pending_messages() const {
  std::size_t total = 0;
  for (const auto &[_, q] : pending_messages_)
    total += q.size();
  return total;
}

void async_scheduler::post_message(const std::string &path, const value_t &msg) {
  // Callable from any thread — the ONLY thread-safe entry point into messaging. Just enqueue
  // under the lock; delivery happens on the scheduler thread in drain_ingress(). A driver
  // blocked in wait_for_work() wakes.
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    ingress_.emplace_back(path, msg);
    ingress_pending_.store(true, std::memory_order_seq_cst);
  }
  notify_work();
}

void async_scheduler::drain_ingress() {
  detail::check_not_in_async_kernel("async_scheduler::drain_ingress");
  // What a step does first: deliver the posts, finish the settled calls, charge the ones still
  // running, and wake the sleepers and timed waits that are due ((await f seconds)), so a driver
  // that only drains while nothing is runnable still sees them.
  chain_valid_ = false;
  drain(false);
}

// ---------------------------------------------------------------------------
// Async calls (docs/STATE_EXEC_EXCEPTIONS.md 3)
// ---------------------------------------------------------------------------

void async_scheduler::track_async_call(std::shared_ptr<async_call> call) {
  async_calls_.push_back(std::move(call));
}

void async_scheduler::post_async_completion(std::shared_ptr<async_call> call) {
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    completions_.push_back(std::move(call));
    ingress_pending_.store(true, std::memory_order_seq_cst);
  }
  notify_work();
}

void async_scheduler::wake_async_waiters(const std::string &channel) {
  std::vector<process *> list;
  list.swap(woken_);
  procs_.take_receivers(channel, list);
  for (process *proc : list) {
    proc->recv_path.reset();
    proc->recv_deadline.reset();
    deliver_await(proc->state, nil_value); // the frame reads the call's outcome
    procs_.set_status(*proc, process_status::ready);
  }
  list.clear();
  woken_.swap(list);
}

void async_scheduler::finish_async_call(const std::shared_ptr<async_call> &call) {
  const auto it = std::find(async_calls_.begin(), async_calls_.end(), call);
  if (it == async_calls_.end())
    return; // dropped already: its process was killed or reaped, or it was finished
  async_calls_.erase(it);
  process *proc = procs_.find(call->pid());
  if (proc && !is_finished(proc->status)) {
    // The rest of the call is charged before its process resumes (3.8): a call that used up
    // the budget kills the process instead.
    detail::charge_async_call(*proc, *call, async_steps_per_cpu_ms(*proc), true,
                              std::chrono::steady_clock::now());
    if (proc->max_memory > 0)
      proc->heap_alloc_bytes += call->alloc_bytes();
    check_limits(*proc);
  } else {
    call->charged.closed = true;
  }
  wake_async_waiters(call->channel());
}

void async_scheduler::charge_async_calls(clock::time_point now) {
  // Every started call is charged its run and CPU time so far (3.8), then the processes it
  // charged are checked against their limits. Walked in place: only a call's own timeout
  // changes the list meanwhile, and from there on the rest goes over a copy.
  charged_.clear();
  for (std::size_t i = 0; i < async_calls_.size(); ++i) {
    async_call &call = *async_calls_[i];
    process *proc = procs_.find(call.pid());
    if (!proc || is_finished(proc->status))
      continue;
    if (!call.finished() && call.past_timeout(now)) {
      charge_async_calls_from(i, now);
      return;
    }
    if (!call.started())
      continue; // queued: waiting for a thread isn't charged
    detail::charge_async_call(*proc, call, async_steps_per_cpu_ms(*proc), false, now);
    if (std::find(charged_.begin(), charged_.end(), proc) == charged_.end())
      charged_.push_back(proc);
  }
  // At a limit the process is killed (2.5), and its calls cancelled.
  for (process *proc : charged_)
    if (!is_finished(proc->status))
      check_limits(*proc);
}

void async_scheduler::charge_async_calls_from(std::size_t first, clock::time_point now) {
  // A copy: a kill or a timeout below changes the list.
  const std::vector<std::shared_ptr<async_call>> calls(async_calls_.begin() + first,
                                                       async_calls_.end());
  for (const std::shared_ptr<async_call> &call : calls) {
    process *found = procs_.find(call->pid());
    if (!found || is_finished(found->status))
      continue;
    process &proc = *found;
    if (!call->finished() && call->past_timeout(now) && call->settle_timeout()) {
      // Its own timeout: it raises TimeoutError at the call, and its kernel is told to stop.
      call->cancel();
      finish_async_call(call);
      continue;
    }
    if (!call->started())
      continue; // queued: waiting for a thread isn't charged
    detail::charge_async_call(proc, *call, async_steps_per_cpu_ms(proc), false, now);
    if (std::find(charged_.begin(), charged_.end(), &proc) == charged_.end())
      charged_.push_back(&proc);
  }
  // At a limit the process is killed (2.5), and its calls cancelled.
  for (process *proc : charged_)
    if (!is_finished(proc->status))
      check_limits(*proc);
}

void async_scheduler::cancel_async_calls(int pid) {
  if (async_calls_.empty())
    return;
  const auto pit = procs_.all().find(pid);
  const auto now = std::chrono::steady_clock::now();
  for (auto it = async_calls_.begin(); it != async_calls_.end();) {
    if ((*it)->pid() != pid) {
      ++it;
      continue;
    }
    std::shared_ptr<async_call> call = *it;
    it = async_calls_.erase(it);
    call->cancel();
    // What it ran so far stays charged; its result, when it comes, is dropped.
    if (pit != procs_.all().end())
      detail::charge_async_call(*pit->second, *call, async_steps_per_cpu_ms(*pit->second), true,
                                now);
  }
}

double async_scheduler::async_steps_per_cpu_ms(const process &proc) {
  if (proc.async_steps_per_cpu_ms)
    return *proc.async_steps_per_cpu_ms;
  if (!watch_root_)
    return kDefaultAsyncStepsPerCpuMs;
  return tree_rate();
}

double async_scheduler::tree_rate() {
  // Read live (3.8): a change applies to the next charge. The tree is read again only when
  // something under the watch root changed since the last read (tree_changes), so the calls in
  // flight don't walk it at every charge.
  track_tree_changes();
  const bool counted = tree_changes_.root() == watch_root_; // else read every time
  const uint64_t seen = tree_changes_.count(); // before the read: a change during it reads again
  if (counted && rate_valid_ && seen == rate_seen_)
    return rate_;
  const double rate = async_steps_per_cpu_ms_from_state(*watch_root_, id_);
  if (!id_.empty() && rate != published_rate_) {
    publish_async_steps_per_cpu_ms(*watch_root_, id_, rate);
    published_rate_ = rate;
  }
  rate_seen_ = seen;
  rate_valid_ = counted;
  rate_ = rate;
  return rate;
}

void async_scheduler::track_tree_changes() {
  // Counted while something reads the tree at every step: watches, or the rate of the calls in
  // flight. Otherwise nothing is connected, and a write under the root pays nothing for it.
  const bool needed = watch_root_ && (!procs_.watchers().empty() || !async_calls_.empty());
  cvc::state *const want = needed ? watch_root_ : nullptr;
  if (tree_changes_.root() == want)
    return;
  tree_changes_.watch(want);
  rate_valid_ = false;
  watch_seen_ = tree_changes_.count() - 1; // poll the watches once: changes before weren't counted
}

void async_scheduler::set_watch_root(cvc::state *root) {
  watch_root_ = root;
  track_tree_changes();
}

int async_scheduler::restore(process_ptr proc) {
  const int pid = next_pid_++;
  proc->pid = pid;
  if (!is_finished(proc->status))
    proc->status = process_status::ready;
  proc->wake_time.reset();
  proc->recv_path.reset();
  proc->recv_deadline.reset();
  proc->awaiting_frame = false;
  proc->create_time = std::chrono::steady_clock::now();
  procs_.insert(std::move(proc));
  return pid;
}

void async_scheduler::queue_watch_event(int pid, process::watch_event evt) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  it->second->pending_watch_events.push_back(std::move(evt));
}

void async_scheduler::register_watch_handler(int pid, int watch_id, value_t handler,
                                             const std::string &path) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  std::string initial;
  if (watch_root_) {
    auto *node = watch_root_->findDescendant(path);
    if (node)
      initial = node->value();
  }
  it->second->watch_handlers[watch_id] = {std::move(handler), path, initial};
  procs_.update(*it->second);
}

void async_scheduler::unregister_watch_handler(int pid, int watch_id) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  it->second->watch_handlers.erase(watch_id);
  procs_.update(*it->second);
}

// ---------------------------------------------------------------------------
// Signal handling (same as sync)
// ---------------------------------------------------------------------------

void async_scheduler::handle_signal(process &proc) {
  if (proc.pending_signals.empty())
    return;

  std::string sig = proc.pending_signals.front();
  proc.pending_signals.erase(proc.pending_signals.begin());

  auto it = proc.signal_handlers.find(sig);
  if (it == proc.signal_handlers.end())
    return;

  proc.saved_stack = proc.state.stack;
  proc.saved_result = proc.state.result;
  proc.in_signal_handler = true;

  auto handler_env = environment::extend(proc.state.global_env);
  handler_env->set("__signal__", value_t(sig));

  proc.state.stack.clear();
  proc.state.stack.push_back({.expr = it->second, .env = handler_env});
  proc.state.done = false;
}

void async_scheduler::restore_from_signal(process &proc) {
  proc.in_signal_handler = false;
  proc.in_watch_handler = false;
  if (!proc.saved_stack.empty()) {
    proc.state.stack = std::move(proc.saved_stack);
    proc.state.result = std::move(proc.saved_result);
    proc.state.done = false;
    proc.saved_stack.clear();
  } else {
    proc.state.done = true;
  }
}

void async_scheduler::poll_watches() {
  // Only the processes with watches, and only when the tree under the watch root changed since
  // the last poll (tree_changes). A value the poll finds changed queues a watch event.
  if (!watch_root_)
    return;
  track_tree_changes();
  if (procs_.watchers().empty())
    return;
  const uint64_t seen = tree_changes_.count();
  if (seen == watch_seen_)
    return;
  watch_seen_ = seen;
  chain_valid_ = false; // the reads below are no step's time
  for (process *proc : procs_.watchers()) {
    for (auto &[wid, entry] : proc->watch_handlers) {
      auto *node = watch_root_->findDescendant(entry.path);
      if (!node)
        continue;
      std::string cur = node->value();
      if (cur != entry.last_value) {
        entry.last_value = cur;
        proc->pending_watch_events.push_back({wid});
      }
    }
  }
}

void async_scheduler::handle_watch_event(process &proc) {
  if (proc.pending_watch_events.empty())
    return;

  auto evt = std::move(proc.pending_watch_events.front());
  proc.pending_watch_events.erase(proc.pending_watch_events.begin());

  // Look up the handler+path from the process's watch_handlers map
  auto hit = proc.watch_handlers.find(evt.watch_id);
  if (hit == proc.watch_handlers.end())
    return; // Watch was removed; discard stale event

  auto &entry = hit->second;

  // The handler must be a closure
  auto *cp = std::get_if<closure_ptr>(&entry.handler.v);
  if (!cp || !*cp)
    return;
  auto &cls = *cp;

  proc.saved_stack = proc.state.stack;
  proc.saved_result = proc.state.result;
  proc.in_watch_handler = true;

  // Directly apply the closure: create env extending the closure's
  // captured env with params bound.
  auto local = environment::extend(cls->env_snapshot);
  if (cls->params.size() > 0)
    local->set(cls->params[0].name, value_t(entry.path));
  if (cls->params.size() > 1)
    local->set(cls->params[1].name, value_t(std::string()));

  proc.state.stack.clear();
  if (cls->body.size() == 1) {
    proc.state.stack.push_back({.expr = cls->body[0], .env = local});
  } else {
    std::vector<value_t> begin_exprs;
    begin_exprs.reserve(1 + cls->body.size());
    begin_exprs.push_back(value_t{symbol{"begin"}});
    begin_exprs.insert(begin_exprs.end(), cls->body.begin(), cls->body.end());
    proc.state.stack.push_back({.expr = make_list(std::move(begin_exprs)), .env = local});
  }
  proc.state.done = false;
}

bool async_scheduler::send_signal(int pid, const std::string &signal) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (is_finished(proc.status))
    return false;

  if (signal == "SIGKILL") {
    kill_process(proc, "SIGKILL");
    return true;
  }

  proc.pending_signals.push_back(signal);
  return true;
}

// ---------------------------------------------------------------------------
// Resource limits (same as sync)
// ---------------------------------------------------------------------------

void async_scheduler::check_limits(process &proc) { check_limits(proc, clock::now()); }

void async_scheduler::check_limits(process &proc, clock::time_point now) {
  if (proc.max_steps > 0 && proc.step_count() >= proc.max_steps) {
    kill_process(proc, "max_steps_exceeded");
    return;
  }
  if (proc.over_time_limit(now)) {
    kill_process(proc, "time_limit_exceeded");
    return;
  }
  // max_memory: state-tree writes plus the measured heap share (heap.h)
  if (proc.max_memory > 0 &&
      mem_tracker_.current_bytes(proc.pid) + proc.heap_bytes >= proc.max_memory) {
    kill_process(proc, "memory_limit_exceeded");
    return;
  }
  if (proc.max_messages > 0 && proc.message_count >= proc.max_messages) {
    kill_process(proc, "message_limit_exceeded");
    return;
  }
  if (proc.max_message_bytes > 0 && proc.message_bytes >= proc.max_message_bytes) {
    kill_process(proc, "message_bytes_exceeded");
    return;
  }
}

// ---------------------------------------------------------------------------
// Process lifecycle (same as sync)
// ---------------------------------------------------------------------------

void async_scheduler::terminate_process(process &proc, value_t result) {
  procs_.set_status(proc, process_status::terminated);
  proc.exit_code = std::move(result);
  proc.state.stats.mark_complete();
  if (proc.on_complete) {
    proc.on_complete(proc.exit_code);
  }
  release_if_idle(proc);
}

void async_scheduler::kill_process(process &proc, const std::string &reason) {
  cancel_async_calls(proc.pid); // its kernels see cancelled(), and their results are dropped
  procs_.set_status(proc, process_status::killed);
  proc.exit_error = reason;
  proc.state.stats.mark_complete();
  release_if_idle(proc);
}

void async_scheduler::fail_process(process &proc, value_t exc, std::exception_ptr original) {
  procs_.set_status(proc, process_status::failed);
  proc.exit_error = exception_summary(exc);
  proc.exit_exception = std::move(exc);
  proc.exit_original = std::move(original);
  proc.state.stats.mark_complete();
  release_if_idle(proc);
}

void async_scheduler::release_if_idle(process &proc) {
  note_process_exit(); // what it leaked is garbage now (heap.h)
  // A process stepping right now (one that killed itself) is released once its step is over.
  if (proc.pid != current_pid_)
    proc.release_state();
}

bool async_scheduler::pause(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  procs_.set_status(proc, process_status::paused);
  return true;
}

bool async_scheduler::resume(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::paused)
    return false;
  procs_.set_status(proc, process_status::ready);
  return true;
}

bool async_scheduler::kill(int pid) {
  detail::check_not_in_async_kernel("async_scheduler::kill");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (is_finished(proc.status))
    return false;
  kill_process(proc, "killed_by_user");
  return true;
}

int async_scheduler::kill_owner(const std::string &owner) {
  int killed = 0;
  for (auto &[pid, proc] : procs_.all()) {
    if (proc->owner == owner && !is_finished(proc->status)) {
      kill_process(*proc, "owner_reaped");
      ++killed;
    }
  }
  return killed;
}

bool async_scheduler::snapshot(int pid, cvc::state &node) const {
  const auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  encode_process(node, *it->second);
  return true;
}

int async_scheduler::restore(cvc::state &node, environment_ptr natives) {
  process_ptr proc = decode_process(node);
  rebind_natives(proc->state, natives);
  return restore(std::move(proc));
}

int async_scheduler::fork(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return -1;
  auto &parent = *it->second;
  if (parent.status != process_status::ready && parent.status != process_status::running)
    return -1;
  // Deep-data work in flight (a function collections.map calls, say) is not copied: a child
  // could not finish the call its stack waits on. A heap measurement waits on no frame, so a
  // child without it is whole, unless the measurement runs on top of such work.
  if (waits_on_data_work(parent.state))
    return -1;

  int child_pid = next_pid_++;
  auto child = make_process();
  child->pid = child_pid;
  child->name = parent.name + "-fork";
  child->status = process_status::ready;
  child->priority = parent.priority;
  child->uid = parent.uid;
  child->gid = parent.gid;
  child->root_path = parent.root_path;
  child->max_steps = parent.max_steps;
  child->max_time = parent.max_time;
  child->max_step_time = parent.max_step_time;
  child->max_memory = parent.max_memory;
  child->max_alloc_bytes = parent.max_alloc_bytes;
  child->heap_check_steps = parent.heap_check_steps;
  child->max_messages = parent.max_messages;
  child->max_message_bytes = parent.max_message_bytes;
  child->signal_handlers = parent.signal_handlers;
  child->create_time = std::chrono::steady_clock::now();
  child->parent_pid = pid;

  child->state.root_expr = parent.state.root_expr;
  child->state.result = parent.state.result;
  child->state.done = parent.state.done;
  child->state.stack = parent.state.stack;
  child->state.user_macros = parent.state.user_macros;
  child->state.data_step_nodes = parent.state.data_step_nodes;
  auto child_env = std::make_shared<environment>();
  if (parent.state.global_env) {
    child_env->bindings = parent.state.global_env->bindings;
    child_env->outer = parent.state.global_env->outer;
  }
  child->state.global_env = child_env;
  child->state.stats.start();

  parent.state.result = value_t(static_cast<int64_t>(child_pid));
  child->state.result = value_t(static_cast<int64_t>(0));

  procs_.insert(std::move(child));
  return child_pid;
}

// ---------------------------------------------------------------------------
// Process control setters
// ---------------------------------------------------------------------------

bool async_scheduler::set_priority(int pid, int priority) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->priority = priority;
  procs_.update(*it->second); // its run-queue level, under the priority policies
  return true;
}

bool async_scheduler::set_max_steps(int pid, uint64_t max_steps) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_steps = max_steps;
  return true;
}

bool async_scheduler::set_max_time(int pid, double seconds) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_time = seconds;
  return true;
}

bool async_scheduler::set_max_memory(int pid, uint64_t bytes) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_memory = bytes;
  return true;
}

bool async_scheduler::set_max_messages(int pid, uint64_t count) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_messages = count;
  return true;
}

bool async_scheduler::set_max_message_bytes(int pid, uint64_t bytes) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_message_bytes = bytes;
  return true;
}

// ---------------------------------------------------------------------------
// Process info / query (same as sync)
// ---------------------------------------------------------------------------

process_info async_scheduler::make_info(const process &proc) const {
  process_info info;
  info.pid = proc.pid;
  info.name = proc.name;
  info.status = proc.status;
  info.priority = proc.priority;
  info.uid = proc.uid;
  info.gid = proc.gid;
  info.step_count = proc.step_count();
  fill_budget_usage(info, proc.state.stats);
  info.elapsed_time = proc.elapsed_time();
  info.current_memory = mem_tracker_.current_bytes(proc.pid);
  info.peak_memory = mem_tracker_.peak_bytes(proc.pid);
  info.heap_memory = proc.heap_bytes;
  info.heap_measurements = proc.heap_measurements;
  fill_async_usage(info, proc);
  info.max_memory = proc.max_memory;
  info.max_time = proc.max_time;
  info.message_count = proc.message_count;
  info.max_messages = proc.max_messages;
  info.message_bytes = proc.message_bytes;
  info.max_message_bytes = proc.max_message_bytes;
  info.parent_pid = proc.parent_pid;
  info.exit_error = proc.exit_error;
  info.exit_exception = proc.exit_exception;
  return info;
}

std::vector<process_info> async_scheduler::list_processes() const {
  std::vector<process_info> result;
  result.reserve(procs_.size());
  for (const auto &[pid, proc] : procs_.all()) {
    result.push_back(make_info(*proc));
  }
  std::sort(result.begin(), result.end(),
            [](const process_info &a, const process_info &b) { return a.pid < b.pid; });
  return result;
}

std::optional<process_info> async_scheduler::get_process_info(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return make_info(*it->second);
}

std::optional<value_t> async_scheduler::get_result(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  if (it->second->status != process_status::terminated)
    return std::nullopt;
  return it->second->exit_code;
}

std::optional<std::string> async_scheduler::get_exit_error(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return it->second->exit_error;
}

std::optional<value_t> async_scheduler::get_exit_exception(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return it->second->exit_exception;
}

void async_scheduler::raise_exit_exception(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end() || !it->second->exit_exception)
    return;
  throw dsl_exception(*it->second->exit_exception, it->second->exit_original);
}

bool async_scheduler::reap(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  const process_status st = it->second->status;
  if (!is_finished(st))
    return false; // never drop a still-live (ready/running/waiting/paused) process
  cancel_async_calls(pid);
  // Its env, state, watch_handlers and captured values (including any Python holders) all live on
  // the process object, so erasing it frees them. The process stepping now (its on_complete
  // reaping it) is erased once its step is over.
  if (pid == current_pid_)
    reap_after_step_ = pid;
  else
    procs_.erase(pid);
  return true;
}

std::unordered_map<int, value_t> async_scheduler::get_results() const {
  std::unordered_map<int, value_t> results;
  for (const auto &[pid, proc] : procs_.all()) {
    if (proc->status == process_status::terminated) {
      results[pid] = proc->exit_code;
    }
  }
  return results;
}

scheduler_stats async_scheduler::get_stats() const {
  scheduler_stats s;
  s.total_processes = static_cast<int>(procs_.size());
  s.total_steps = total_steps_;
  for (const auto &[pid, proc] : procs_.all()) {
    switch (proc->status) {
    case process_status::running:
      ++s.running;
      break;
    case process_status::ready:
      ++s.ready;
      break;
    case process_status::paused:
      ++s.paused;
      break;
    case process_status::terminated:
      ++s.terminated;
      break;
    case process_status::killed:
      ++s.killed;
      break;
    case process_status::failed:
      ++s.failed;
      break;
    default:
      break;
    }
  }
  return s;
}

} // namespace cvc::state_exec

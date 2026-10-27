#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/state_value_codec.h> // snapshot / restore
#include <stdexcept>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

scheduler::scheduler(scheduling_policy policy)
    : policy_(policy), evaluator_(builtins::make_default_environment()),
      procs_(policy != scheduling_policy::round_robin) {}

const process_ptr &scheduler::no_process() {
  static const process_ptr none;
  return none;
}

// ---------------------------------------------------------------------------
// Settings — state-tree-backed configuration
// ---------------------------------------------------------------------------

// Helper: read a size_t from a state node, returning nullopt on missing/bad.
static std::optional<std::size_t> read_size_setting(cvc::state *root, const std::string &path) {
  auto *node = root->findDescendant(path);
  if (!node)
    return std::nullopt;
  std::string v = node->value();
  if (v.empty())
    return std::nullopt;
  try {
    return static_cast<std::size_t>(std::stoul(v));
  } catch (...) {
    return std::nullopt;
  }
}

void scheduler::load_settings() {
  if (!watch_root_)
    return;

  // --- max_pending_messages ---
  constexpr std::size_t fallback_max_pending = 1024;
  std::size_t effective = fallback_max_pending;

  // Global default
  if (auto v =
          read_size_setting(watch_root_, std::string(defaults_prefix) + ".max_pending_messages"))
    effective = *v;

  // Per-scheduler override
  if (!id_.empty()) {
    std::string per = std::string(sched_prefix) + "." + id_ + ".max_pending_messages";
    if (auto v = read_size_setting(watch_root_, per))
      effective = *v;
  }

  max_pending_messages = effective;

  // --- parser.* and data.* (same resolution order; see parse_options_from_state) ---
  parse_options_ = parse_options_from_state(*watch_root_, id_);
  data_options_ = data_options_from_state(*watch_root_, id_);
  // --- limits.max_alloc_bytes and limits.heap_check_steps (same resolution order) ---
  max_alloc_bytes_ = max_alloc_bytes_from_state(*watch_root_, id_);
  heap_options_ = heap_options_from_state(*watch_root_, id_);

  // Publish effective values to per-scheduler subtree
  if (!id_.empty()) {
    std::string base = std::string(sched_prefix) + "." + id_;
    (*watch_root_)(base + ".max_pending_messages").value(std::to_string(max_pending_messages));
    (*watch_root_)(base + ".parser.max_depth").value(std::to_string(parse_options_.max_depth));
    (*watch_root_)(base + ".parser.step_tokens").value(std::to_string(parse_options_.step_tokens));
    (*watch_root_)(base + ".data.step_nodes").value(std::to_string(data_options_.step_nodes));
    (*watch_root_)(base + ".limits.max_alloc_bytes").value(std::to_string(max_alloc_bytes_));
    (*watch_root_)(base + ".limits.heap_check_steps")
        .value(std::to_string(heap_options_.check_steps));
    // Read live at every charge; this publishes what it is now.
    const double rate = async_steps_per_cpu_ms_from_state(*watch_root_, id_);
    publish_async_steps_per_cpu_ms(*watch_root_, id_, rate);
    published_rate_ = rate;
    (*watch_root_)(base + ".policy")
        .value(policy_ == scheduling_policy::round_robin ? "round_robin"
               : policy_ == scheduling_policy::priority  ? "priority"
                                                         : "priority_rr");
  }
}

// A non-negative decimal number ("1000", "0.5", ".25"), read without the C locale's help; nullopt
// for anything else.
static std::optional<double> read_rate_setting(cvc::state *root, const std::string &path) {
  auto *node = root->findDescendant(path);
  if (!node)
    return std::nullopt;
  const std::string v = node->value();
  double whole = 0.0, scale = 1.0;
  bool digits = false, point = false;
  for (const char c : v) {
    if (c == '.' && !point) {
      point = true;
    } else if (c >= '0' && c <= '9') {
      digits = true;
      if (point) {
        scale /= 10.0;
        whole += scale * (c - '0');
      } else {
        whole = whole * 10.0 + (c - '0');
      }
    } else {
      return std::nullopt;
    }
  }
  return digits ? std::optional<double>(whole) : std::nullopt;
}

double async_steps_per_cpu_ms_from_state(cvc::state &root, const std::string &scheduler_id) {
  constexpr const char *suffix = ".limits.async_steps_per_cpu_ms";
  if (!scheduler_id.empty())
    if (auto v = read_rate_setting(&root, std::string(scheduler::sched_prefix) + "." +
                                              scheduler_id + suffix))
      return *v;
  if (auto v = read_rate_setting(&root, std::string(scheduler::defaults_prefix) + suffix))
    return *v;
  return kDefaultAsyncStepsPerCpuMs;
}

void publish_async_steps_per_cpu_ms(cvc::state &root, const std::string &scheduler_id,
                                    double rate) {
  if (scheduler_id.empty())
    return;
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.17g", rate);
  root(std::string(scheduler::sched_prefix) + "." + scheduler_id +
       ".effective.async_steps_per_cpu_ms")
      .value(buf);
}

double scheduler::async_steps_per_cpu_ms(const process &proc) {
  if (proc.async_steps_per_cpu_ms)
    return *proc.async_steps_per_cpu_ms;
  if (!watch_root_)
    return kDefaultAsyncStepsPerCpuMs;
  // Read live (3.8): a change applies to the next charge. The tree is read again only once
  // something under its state_exec node changed (settings_changes), so an inline call doesn't
  // walk the tree when nothing did.
  const uint64_t seen = settings_changes_.version(watch_root_); // before the read: see below
  if (rate_valid_ && seen == rate_seen_)
    return rate_;
  const double rate = async_steps_per_cpu_ms_from_state(*watch_root_, id_);
  if (!id_.empty() && rate != published_rate_) {
    publish_async_steps_per_cpu_ms(*watch_root_, id_, rate); // moves the version: read once more
    published_rate_ = rate;
  }
  // The version from before the read: a change during it (another thread's write, or the
  // publication above) reads the tree again next time.
  rate_seen_ = seen;
  rate_valid_ = true;
  rate_ = rate;
  return rate;
}

uint64_t max_alloc_bytes_from_state(cvc::state &root, const std::string &scheduler_id,
                                    uint64_t base) {
  constexpr const char *suffix = ".limits.max_alloc_bytes";
  uint64_t effective = base;
  if (auto v = read_size_setting(&root, std::string(scheduler::defaults_prefix) + suffix))
    effective = *v;
  if (!scheduler_id.empty())
    if (auto v = read_size_setting(&root, std::string(scheduler::sched_prefix) + "." +
                                              scheduler_id + suffix))
      effective = *v;
  return effective;
}

// ---------------------------------------------------------------------------
// Process submission
// ---------------------------------------------------------------------------

int scheduler::execute(const std::string &script, const execute_options &opts) {
  detail::check_not_in_async_kernel("scheduler::execute");
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
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  proc.max_alloc_bytes = max_alloc_bytes_;
  proc.heap_check_steps = heap_options_.check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(script, opts.env, parse_options_);
  proc.state.data_step_nodes = data_options_.step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

int scheduler::execute(const value_t &expr, const execute_options &opts) {
  detail::check_not_in_async_kernel("scheduler::execute");
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
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  proc.max_alloc_bytes = max_alloc_bytes_;
  proc.heap_check_steps = heap_options_.check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(expr, opts.env);
  proc.state.data_step_nodes = data_options_.step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

// ---------------------------------------------------------------------------
// Process selection
// ---------------------------------------------------------------------------

// The pick is what it always was: the runnable processes in pid order, indexed by rr_index_
// (round robin), the lowest pid of the best priority (priority), or rr_index_ among the best
// priority's (priority_rr); the run queue finds it without scanning (process_table).
const process_ptr *scheduler::select_process() {
  if (policy_ == scheduling_policy::priority)
    return procs_.pick_first();
  return procs_.pick_round_robin(rr_index_);
}

// ---------------------------------------------------------------------------
// Step execution
// ---------------------------------------------------------------------------

scheduler::clock::time_point scheduler::now_for_step() {
  if (!chain_valid_) {
    chain_now_ = clock::now();
    chain_cpu_ = -1.0;
    chain_valid_ = true;
  }
  return chain_now_;
}

void scheduler::execute_process_step(const process_ptr &pp) {
  process &proc = *pp;
  // Between steps: free the reference cycles programs leaked, once enough piled up (heap.h).
  // Checked every 64 steps: the check reads thread-local state.
  if ((++cycle_check_ & 63) == 0 && cycle_collection_due()) {
    collect_cycles();
    chain_valid_ = false;
  }

  // Track which process is currently executing so intrinsics can query it.
  current_pid_ = proc.pid;
  current_proc_ = &pp;
  // Ensure current_pid_ is reset even on early returns.
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

  // 1. Handle pending signals first
  if (!proc.pending_signals.empty() && can_interrupt) {
    chain_valid_ = false; // it copies the stack: not the next step's time
    handle_signal(proc);
    return; // Signal setup counts as a step
  }

  // 1b. Handle pending watch events
  if (!proc.pending_watch_events.empty() && can_interrupt) {
    chain_valid_ = false;
    handle_watch_event(proc);
    return;
  }

  // 2. Mark as running and track timing. Inside run() a step starts from the reading the step
  // before it ended at, when nothing ran in between but the loop picking this process: one clock
  // read a step. Every time budget of the step is computed from that one reading.
  procs_.set_status(proc, process_status::running);
  const clock::time_point start = now_for_step();
  const double chained_cpu = chain_cpu_;
  chain_valid_ = false; // until this step ends plainly
  proc.last_run_start = start;
  // Async calls run inline in the step (3.5) and are charged as calls (3.8), not as the step.
  proc.step_async_wall = 0.0;
  proc.step_async_cpu = 0.0;

  // 3. Execute one evaluation step, bounded by the process's remaining time budget.
  // A single step may enter an uncapped nested evaluator (a defclass method body) or a
  // long-running builtin (collect, a generator drive) whose INTERNAL loop the scheduler's
  // between-steps max_time check (checked only after step() returns) cannot see. Arming the
  // cooperative deadline for this step lets that inner work abort at the budget too — the
  // same guarantee the read-lane's run() already has. max_time <= 0 means unlimited (nullopt).
  eval_deadline_guard step_deadline(proc.time_left(start), start);
  // Bound what one native call in this step may allocate: limits.max_alloc_bytes, tightened to
  // what is left of max_memory (its state-tree writes plus its measured heap share).
  // With no budget and the default cap, an armed guard would change nothing (the default applies
  // unarmed), so it is armed only when it does: it costs thread-local writes every step.
  std::optional<eval_alloc_guard> step_alloc;
  if (proc.max_memory > 0 || proc.max_alloc_bytes != kDefaultMaxAllocBytes)
    step_alloc.emplace(proc.alloc_limit(
        proc.max_memory > 0 ? mem_tracker_.current_bytes(proc.pid) + proc.heap_bytes : 0));
  const bool charge_cpu = proc.charges_cpu_time(start);
  const double cpu0 = charge_cpu ? (chained_cpu >= 0.0 ? chained_cpu : thread_cpu_seconds()) : 0.0;
  bool done = false;
  try {
    done = evaluator_.step(proc.state);
  } catch (const evaluation_timeout &) {
    // The step's deadline cut a long loop inside it (a data job, a generator, a stdlib call):
    // the process ran out of time, as the between-steps check below would find. A deadline
    // armed outside this scheduler is its caller's, so that one propagates like other errors.
    if (!step_deadline.expired())
      throw;
    proc.accumulated_time +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - proc.last_run_start)
            .count();
    kill_process(proc, "time_limit_exceeded");
    return;
  } catch (const std::exception &e) {
    // A signal or watch handler's uncaught exception is raised in the program where the handler
    // interrupted it (docs/STATE_EXEC_EXCEPTIONS.md 2.4), for the program's own try to catch.
    bool caught_in_program = false;
    if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.uncaught) {
      value_t exc = std::move(*proc.state.uncaught);
      proc.state.uncaught.reset();
      restore_from_signal(proc);
      evaluator_.raise(proc.state, std::move(exc));
      caught_in_program = !proc.state.uncaught;
    }
    if (!caught_in_program) {
      // An exception the program didn't catch ends it `failed`, with the exception kept for the
      // host (docs/STATE_EXEC_EXCEPTIONS.md 2.6); run() goes on with the other processes. An
      // error that isn't the program's (a syntax error found while reading it) propagates.
      if (!proc.state.uncaught)
        throw;
      proc.accumulated_time +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - proc.last_run_start)
              .count();
      const auto *thrown = dynamic_cast<const dsl_exception *>(&e);
      fail_process(proc, *proc.state.uncaught, thrown ? thrown->original() : nullptr);
      return;
    }
    done = proc.state.done;
  }

  // 4. Accumulate running time, and charge it to what the step was spent on
  const clock::time_point now = clock::now();
  const double step_seconds = std::chrono::duration<double>(now - start).count();
  proc.accumulated_time += step_seconds;
  proc.state.stats.add_step_time(step_seconds - proc.step_async_wall);
  double cpu1 = -1.0;
  if (charge_cpu) {
    cpu1 = thread_cpu_seconds();
    proc.charge_cpu_time(std::max(0.0, cpu1 - cpu0 - proc.step_async_cpu));
  }
  // Heap accounting: take a finished measurement, start the next when due (heap.h).
  if (proc.max_memory > 0 || proc.state.heap_measured)
    heap_after_step(proc, step_alloc ? step_alloc->allocated() : 0,
                    mem_tracker_.current_bytes(proc.pid));

  // 5. Check if signal/watch handler just finished
  if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.done) {
    restore_from_signal(proc);
    // Don't check done — we restored original evaluation
    procs_.set_status(proc, process_status::ready);
    check_limits(proc, now);
    return;
  }

  // 6. Check resource limits
  check_limits(proc, now);
  if (proc.status == process_status::killed)
    return;

  // 7. Check completion
  if (done) {
    terminate_process(proc, proc.state.result); // runs on_complete: not the next step's time
  } else {
    if (proc.status == process_status::running) {
      // Only reset to ready if the intrinsic didn't change the status
      // (e.g. sleep or msg-recv set it to waiting).
      procs_.set_status(proc, process_status::ready);
    }
    chain_now_ = now;
    chain_cpu_ = cpu1;
    chain_valid_ = true;
  }
}

int scheduler::step() {
  chain_valid_ = false; // called from outside a run: what ran since the last step is no step's
  return step_once();
}

int scheduler::step_once() {
  detail::check_not_in_async_kernel("scheduler::step");
  // Wake sleeping processes (and timed waits) whose deadline has passed.
  if (procs_.timed_count() > 0) {
    const clock::time_point now = now_for_step();
    if (procs_.timer_due(now) && procs_.wake_due(now) > 0)
      chain_valid_ = false; // the wakes are no step's time
  }

  // Check for value changes on watched state paths.
  poll_watches();

  const process_ptr *pp = select_process();
  if (!pp)
    return 0;
  // A process its own on_complete reaped goes once its step is over, however the step ends.
  struct reap_guard {
    detail::process_table &procs;
    int &pid;
    ~reap_guard() {
      if (pid >= 0) {
        procs.erase(pid);
        pid = -1;
      }
    }
  } reap_deferred{procs_, reap_after_step_};
  execute_process_step(*pp);
  ++total_steps_;
  return 1;
}

std::unordered_map<int, value_t> scheduler::run(std::optional<uint64_t> max_steps,
                                                std::optional<double> max_time) {
  run_steps(max_steps, max_time);
  return get_results();
}

uint64_t scheduler::run_steps(std::optional<uint64_t> max_steps, std::optional<double> max_time) {
  running_ = true;
  stop_requested_ = false;
  chain_valid_ = false;
  uint64_t steps = 0; // the loop's iterations, idle ones included (max_steps counts them)
  uint64_t taken = 0; // the process steps among them
  // max_time is a time_budget: the run's own time, not time its thread spent descheduled.
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
    const int stepped = step_once();
    ++steps;
    if (stepped != 0) {
      ++taken;
      continue;
    }
    if (!idle_wait(max_steps, steps, budget ? &*budget : nullptr))
      break;
  }

  running_ = false;
  chain_valid_ = false;
  return taken;
}

bool scheduler::idle_wait(std::optional<uint64_t> max_steps, uint64_t &steps, time_budget *budget) {
  // Nothing is ready: only sleepers and processes waiting for messages are left (has_runnable).
  // run() used to spin here, each idle iteration one of max_steps, until a deadline passed or
  // its budgets ran out. It waits instead, burning no CPU, and keeps both budgets as the spin
  // spent them: the wait is charged to max_time as run time (time_budget::charge_wait), and to
  // max_steps at kIdleStepSeconds an iteration, about what a spinning one took. So a run whose
  // step budget would have run out before the next deadline (a host pumping run(N) for a
  // frame) returns at once, as the spin did a few microseconds later, and one with steps to
  // spare (a bounded run to completion) waits for the sleeper, as the spin did.
  constexpr double kIdleStepSeconds = 100e-9;
  chain_valid_ = false;
  const clock::time_point now = clock::now();
  const std::optional<clock::time_point> next = procs_.next_timer();
  const uint64_t steps_left = max_steps ? *max_steps - std::min(steps, *max_steps) : 0;
  clock::time_point until = next ? *next : clock::time_point::max();
  if (budget) {
    budget->stop(); // what is left of it, as of now
    until = std::min(until, detail::time_after(now, budget->remaining()));
  }
  // The step budget runs out first (or only message waiters without a time limit are left,
  // whom nothing in a synchronous run can wake): what spinning through it would come to.
  if (max_steps && (!next || detail::time_after(now, static_cast<double>(steps_left) *
                                                         kIdleStepSeconds) < until)) {
    if (budget)
      budget->start(now);
    return false;
  }
  if (!idle_)
    idle_ = std::make_unique<detail::wakeup>();
  // stop() may come meanwhile (from another thread, a signal handler): checked every 10 ms. A
  // sleeper's deadline is waited for precisely, the budget's end need not be.
  constexpr auto kStopCheck = std::chrono::milliseconds(10);
  const bool for_timer = next && *next == until;
  for (clock::time_point t = now; !stop_requested_ && t < until; t = clock::now()) {
    const clock::time_point slice = until - t > kStopCheck ? t + kStopCheck : until;
    idle_->wait_until(slice, for_timer && slice == until);
  }
  const clock::time_point end = clock::now();
  const double waited = std::chrono::duration<double>(end - now).count();
  if (budget) {
    budget->charge_wait(waited);
    budget->start(end); // the loop's next check finds it spent if the wait used it up
  }
  if (max_steps && steps_left > 1)
    steps += std::min(static_cast<uint64_t>(waited / kIdleStepSeconds), steps_left - 1);
  return next.has_value(); // only message waiters: the budget ran out or stop() came
}

void scheduler::stop() { stop_requested_ = true; }

bool scheduler::has_runnable() const {
  // Ready or running, or waiting with a wake_time (a sleeper becomes runnable at its deadline),
  // or waiting for a message (it becomes runnable when one arrives).
  return procs_.runnable_count() > 0 || procs_.sleeping_count() > 0 || procs_.receiving_count() > 0;
}

void scheduler::queue_watch_event(int pid, process::watch_event evt) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  it->second->pending_watch_events.push_back(std::move(evt));
}

void scheduler::register_watch_handler(int pid, int watch_id, value_t handler,
                                       const std::string &path) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  // Capture current value so the first change can be detected by polling.
  std::string initial;
  if (watch_root_) {
    auto *node = watch_root_->findDescendant(path);
    if (node)
      initial = node->value();
  }
  it->second->watch_handlers[watch_id] = {std::move(handler), path, initial};
  procs_.update(*it->second);
}

void scheduler::unregister_watch_handler(int pid, int watch_id) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  it->second->watch_handlers.erase(watch_id);
  procs_.update(*it->second);
}

void scheduler::set_watch_root(cvc::state *root) {
  if (root != watch_root_ && tree_changes_.root())
    tree_changes_.watch(root); // counted afresh (a new root counts as a change)
  if (root != watch_root_)
    rate_valid_ = false;
  watch_root_ = root;
}

// ---------------------------------------------------------------------------
// Watch polling — replaces boost::signals2 for cross-platform reliability
// ---------------------------------------------------------------------------

void scheduler::poll_watches() {
  // Only the processes with watches, and only when the tree under the watch root changed since
  // the last poll (a write anywhere under it counts: tree_changes). A value the poll finds
  // changed queues a watch event, as it always did.
  if (!watch_root_)
    return;
  if (procs_.watchers().empty()) {
    if (tree_changes_.root()) {
      tree_changes_.watch(nullptr); // no slot on the tree while nothing watches it
      chain_valid_ = false;
    }
    return;
  }
  if (tree_changes_.root() != watch_root_) {
    tree_changes_.watch(watch_root_);
    watch_seen_ = tree_changes_.count() - 1; // poll once now: changes before it weren't counted
    chain_valid_ = false;                    // connecting is no step's time
  }
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

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

void scheduler::handle_signal(process &proc) {
  if (proc.pending_signals.empty())
    return;

  std::string sig = proc.pending_signals.front();
  proc.pending_signals.erase(proc.pending_signals.begin());

  auto it = proc.signal_handlers.find(sig);
  if (it == proc.signal_handlers.end())
    return; // No handler — ignored

  // Save current evaluation state (full stack + result)
  proc.saved_stack = proc.state.stack;
  proc.saved_result = proc.state.result;
  proc.in_signal_handler = true;

  // Set up handler evaluation
  auto handler_env = environment::extend(proc.state.global_env);
  handler_env->set("__signal__", value_t(sig));

  // Replace stack with handler invocation: (handler)
  proc.state.stack.clear();
  proc.state.stack.push_back({.expr = it->second, .env = handler_env});
  proc.state.done = false;
}

void scheduler::restore_from_signal(process &proc) {
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

void scheduler::handle_watch_event(process &proc) {
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

  // Save current evaluation state (full stack + result)
  proc.saved_stack = proc.state.stack;
  proc.saved_result = proc.state.result;
  proc.in_watch_handler = true;

  // Directly apply the closure: create env extending the closure's
  // captured env with params bound.  This avoids the full
  // symbol-lookup-and-call path through the evaluator, working around
  // a SEGFAULT observed on macOS / Apple Clang.
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

bool scheduler::send_signal(int pid, const std::string &signal) {
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
// Resource limit checking
// ---------------------------------------------------------------------------

void scheduler::check_limits(process &proc) { check_limits(proc, clock::now()); }

void scheduler::check_limits(process &proc, clock::time_point now) {
  // max_steps
  if (proc.max_steps > 0 && proc.step_count() >= proc.max_steps) {
    kill_process(proc, "max_steps_exceeded");
    return;
  }
  // max_time
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
  // max_messages
  if (proc.max_messages > 0 && proc.message_count >= proc.max_messages) {
    kill_process(proc, "message_limit_exceeded");
    return;
  }
  // max_message_bytes
  if (proc.max_message_bytes > 0 && proc.message_bytes >= proc.max_message_bytes) {
    kill_process(proc, "message_bytes_exceeded");
    return;
  }
}

// ---------------------------------------------------------------------------
// Process lifecycle
// ---------------------------------------------------------------------------

void scheduler::terminate_process(process &proc, value_t result) {
  procs_.set_status(proc, process_status::terminated);
  proc.exit_code = std::move(result);
  proc.state.stats.mark_complete();
  if (proc.on_complete) {
    proc.on_complete(proc.exit_code);
  }
  release_if_idle(proc);
}

void scheduler::kill_process(process &proc, const std::string &reason) {
  procs_.set_status(proc, process_status::killed);
  proc.exit_error = reason;
  proc.state.stats.mark_complete();
  release_if_idle(proc);
}

void scheduler::fail_process(process &proc, value_t exc, std::exception_ptr original) {
  procs_.set_status(proc, process_status::failed);
  proc.exit_error = exception_summary(exc);
  proc.exit_exception = std::move(exc);
  proc.exit_original = std::move(original);
  proc.state.stats.mark_complete();
  release_if_idle(proc);
}

void scheduler::release_if_idle(process &proc) {
  note_process_exit(); // what it leaked is garbage now (heap.h)
  // A process stepping right now (one that killed itself) is released once its step is over.
  if (proc.pid != current_pid_)
    proc.release_state();
}

bool scheduler::pause(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  // If running, accumulate time up to now
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  procs_.set_status(proc, process_status::paused);
  return true;
}

bool scheduler::resume(int pid) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::paused)
    return false;
  procs_.set_status(proc, process_status::ready);
  return true;
}

bool scheduler::sleep(int pid, double seconds) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  const auto now = std::chrono::steady_clock::now();
  if (proc.status == process_status::running)
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  proc.wake_time = detail::time_after(now, seconds); // saturating: a huge sleep doesn't wrap
  procs_.set_status(proc, process_status::waiting);  // a timer for wake_time
  return true;
}

bool scheduler::yield_frame(int /*pid*/) {
  // No frame boundary in the synchronous scheduler — (await …) is identity here (the intrinsic
  // already returned its value; not parking lets the process simply continue).
  return false;
}

bool scheduler::receive_message(int pid, const std::string &path) {
  detail::check_not_in_async_kernel("scheduler::receive_message");
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

bool scheduler::receive_message_until(int pid, const std::string &path, double seconds) {
  if (!receive_message(pid, path))
    return false;
  process &proc = *procs_.find(pid);
  proc.recv_deadline = detail::time_after(std::chrono::steady_clock::now(), seconds);
  procs_.update(proc); // a timer for the deadline
  return true;
}

int scheduler::deliver_to_receivers(const std::string &path, const value_t &msg) {
  detail::check_not_in_async_kernel("scheduler::deliver_to_receivers");
  // The processes waiting on `path`, from the receiver index (no scan).
  std::vector<process *> woken_list;
  woken_list.swap(woken_);
  procs_.take_receivers(path, woken_list);
  for (process *proc : woken_list) {
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
  const int woken = static_cast<int>(woken_list.size());
  woken_list.clear();
  woken_.swap(woken_list);
  // If no process was waiting, queue for later msg-recv calls.
  if (woken == 0) {
    auto &q = pending_messages_[path];
    if (max_pending_messages == 0 || q.size() < max_pending_messages)
      q.push(msg);
  }
  return woken;
}

std::optional<value_t> scheduler::pop_pending_message(const std::string &path) {
  auto it = pending_messages_.find(path);
  if (it == pending_messages_.end() || it->second.empty())
    return std::nullopt;
  auto msg = std::move(it->second.front());
  it->second.pop();
  if (it->second.empty())
    pending_messages_.erase(it);
  return msg;
}

std::size_t scheduler::pending_message_count(const std::string &path) const {
  auto it = pending_messages_.find(path);
  if (it == pending_messages_.end())
    return 0;
  return it->second.size();
}

std::size_t scheduler::total_pending_messages() const {
  std::size_t total = 0;
  for (const auto &[_, q] : pending_messages_)
    total += q.size();
  return total;
}

bool scheduler::kill(int pid) {
  detail::check_not_in_async_kernel("scheduler::kill");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (is_finished(proc.status))
    return false;
  kill_process(proc, "killed_by_user");
  return true;
}

int scheduler::restore(process_ptr proc) {
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

bool scheduler::snapshot(int pid, cvc::state &node) const {
  const auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  encode_process(node, *it->second);
  return true;
}

int scheduler::restore(cvc::state &node, environment_ptr natives) {
  process_ptr proc = decode_process(node);
  rebind_natives(proc->state, natives);
  return restore(std::move(proc));
}

int scheduler::fork(int pid) {
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
  child->max_memory = parent.max_memory;
  child->max_alloc_bytes = parent.max_alloc_bytes;
  child->heap_check_steps = parent.heap_check_steps;
  child->max_messages = parent.max_messages;
  child->max_message_bytes = parent.max_message_bytes;
  child->signal_handlers = parent.signal_handlers;
  child->create_time = std::chrono::steady_clock::now();
  child->parent_pid = pid;

  // Deep copy evaluator state
  child->state.root_expr = parent.state.root_expr;
  child->state.result = parent.state.result;
  child->state.done = parent.state.done;
  child->state.stack = parent.state.stack; // Copy stack
  child->state.user_macros = parent.state.user_macros;
  child->state.data_step_nodes = parent.state.data_step_nodes;
  // Create fresh env as a copy of parent's bindings
  auto child_env = std::make_shared<environment>();
  if (parent.state.global_env) {
    child_env->bindings = parent.state.global_env->bindings;
    child_env->outer = parent.state.global_env->outer;
  }
  child->state.global_env = child_env;
  child->state.stats.start();

  // Child result = 0, parent result = child_pid
  // (set on caller's evaluator result so next step picks it up)
  parent.state.result = value_t(static_cast<int64_t>(child_pid));
  child->state.result = value_t(static_cast<int64_t>(0));

  procs_.insert(std::move(child));
  return child_pid;
}

// ---------------------------------------------------------------------------
// Process control setters
// ---------------------------------------------------------------------------

bool scheduler::set_priority(int pid, int priority) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->priority = priority;
  procs_.update(*it->second); // its run-queue level, under the priority policies
  return true;
}

bool scheduler::set_max_steps(int pid, uint64_t max_steps) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_steps = max_steps;
  return true;
}

bool scheduler::set_max_time(int pid, double seconds) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_time = seconds;
  return true;
}

bool scheduler::set_max_memory(int pid, uint64_t bytes) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_memory = bytes;
  return true;
}

bool scheduler::set_max_messages(int pid, uint64_t count) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_messages = count;
  return true;
}

bool scheduler::set_max_message_bytes(int pid, uint64_t bytes) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->max_message_bytes = bytes;
  return true;
}

// ---------------------------------------------------------------------------
// Process info / query
// ---------------------------------------------------------------------------

process_info scheduler::make_info(const process &proc) const {
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

std::vector<process_info> scheduler::list_processes() const {
  std::vector<process_info> result;
  result.reserve(procs_.size());
  for (const auto &[pid, proc] : procs_.all()) {
    result.push_back(make_info(*proc));
  }
  // Sort by PID for deterministic ordering
  std::sort(result.begin(), result.end(),
            [](const process_info &a, const process_info &b) { return a.pid < b.pid; });
  return result;
}

std::optional<process_info> scheduler::get_process_info(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return make_info(*it->second);
}

std::optional<value_t> scheduler::get_result(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  if (it->second->status != process_status::terminated)
    return std::nullopt;
  return it->second->exit_code;
}

std::optional<std::string> scheduler::get_exit_error(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return it->second->exit_error;
}

std::optional<value_t> scheduler::get_exit_exception(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return std::nullopt;
  return it->second->exit_exception;
}

void scheduler::raise_exit_exception(int pid) const {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end() || !it->second->exit_exception)
    return;
  throw dsl_exception(*it->second->exit_exception, it->second->exit_original);
}

bool scheduler::reap(int pid) {
  detail::check_not_in_async_kernel("scheduler::reap");
  if (pid == current_pid_) {
    // The process stepping now (its on_complete reaping it): erased once its step is over.
    const process *p = procs_.find(pid);
    if (!p || !is_finished(p->status))
      return false;
    reap_after_step_ = pid;
    return true;
  }
  return procs_.erase(pid);
}

std::unordered_map<int, value_t> scheduler::get_results() const {
  std::unordered_map<int, value_t> results;
  for (const auto &[pid, proc] : procs_.all()) {
    if (proc->status == process_status::terminated) {
      results[pid] = proc->exit_code;
    }
  }
  return results;
}

scheduler_stats scheduler::get_stats() const {
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

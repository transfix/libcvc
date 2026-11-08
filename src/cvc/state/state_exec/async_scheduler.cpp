#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_call.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/priority.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/scheduler.h>         // max_alloc_bytes_from_state
#include <cvc/state/state_exec/state_value_codec.h> // snapshot / restore
#include <cvc/state/state_exec/stdlib.h>            // random_stream_scope
#include <exception>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <thread>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

async_scheduler::async_scheduler(scheduling_policy policy)
    : policy_(policy), evaluator_(builtins::make_default_environment()),
      procs_(policy != scheduling_policy::round_robin),
      wakeup_(std::make_shared<detail::wakeup>()) {}

// A group's scheduler (S1.3): its global environment is its own scope over `base`. A program's
// `set` binds in its process's own run env (step_set_value's env->set), so nothing a program writes
// reaches `base`, which may be frozen and shared (the builtins).
async_scheduler::async_scheduler(scheduling_policy policy, environment_ptr base)
    : policy_(policy), evaluator_(environment::extend(std::move(base))),
      procs_(policy != scheduling_policy::round_robin),
      wakeup_(std::make_shared<detail::wakeup>()) {}

void async_scheduler::set_cycle_registry(cycle_registry *r) { registry_ = r; }

void async_scheduler::set_random_stream(random_stream *s) { random_ = s; }

void async_scheduler::set_pid_source(std::shared_ptr<std::atomic<int>> source) {
  if (source) {
    // Above every pid issued here (process_table's insert rule): the own counter's, and every pid
    // in the table, which a source set before may have issued.
    int highest = pid_source_ ? 0 : next_pid_ - 1;
    for (const auto &[pid, p] : procs_.all())
      highest = std::max(highest, pid);
    const int first = source->load(std::memory_order_relaxed);
    if (first <= highest)
      throw std::invalid_argument("set_pid_source: the source's next pid " + std::to_string(first) +
                                  " is not above pid " + std::to_string(highest));
  } else if (pid_source_) {
    // Back to the own counter, above whatever the source issued.
    next_pid_ = std::max(next_pid_, pid_source_->load(std::memory_order_relaxed));
  }
  pid_source_ = std::move(source);
}

async_scheduler::pump_owner_scope::pump_owner_scope(async_scheduler &s) : s_(s), outer_() {
  // "No owner, or me" (S1-Q4): taken from unset with acquire, the handoff edge td 4.1 asks for.
  const std::thread::id me = std::this_thread::get_id();
  std::thread::id seen{};
  while (!s_.pump_owner_.compare_exchange_weak(seen, me, std::memory_order_acquire,
                                               std::memory_order_relaxed)) {
    if (seen == me) { // nested in a scope of this thread
      outer_ = me;
      return;
    }
    if (seen != std::thread::id{})
      throw std::runtime_error(kPumpedElsewhereError);
  }
}

async_scheduler::pump_owner_scope::~pump_owner_scope() {
  s_.pump_owner_.store(outer_, std::memory_order_release);
}

bool async_scheduler::pumped_elsewhere() const noexcept {
  const std::thread::id owner = pump_owner_.load(std::memory_order_relaxed);
  return owner != std::thread::id{} && owner != std::this_thread::get_id();
}

namespace {
// S1.1: a scheduler given a registry or a random stream binds what it was given on each entry
// that can run DSL or track, one store and one restore per call (never per step); a bare one binds
// nothing, as before. What it was not given stays as the caller bound it.
class entry_binding {
public:
  entry_binding(cycle_registry *r, random_stream *s) {
    if (r)
      registry_.emplace(r);
    if (s)
      random_.emplace(s);
  }

private:
  std::optional<cycle_registry_scope> registry_;
  std::optional<random_stream_scope> random_;
};

// S1-Q4 (td 4.1): debug builds, or CVC_EXEC_VERIFY=1 (read once, as the library loads), check the
// pump owner's rule in the public mutators. A call while another thread pumps is a host's race:
// it aborts.
#ifdef NDEBUG
const bool kVerifyPumpOwner = [] {
  const char *v = std::getenv("CVC_EXEC_VERIFY");
  return v && *v && *v != '0';
}();
#else
constexpr bool kVerifyPumpOwner = true;
#endif

void verify_pump_owner(const async_scheduler &s, const char *what) {
  if (kVerifyPumpOwner && s.pumped_elsewhere()) {
    std::fprintf(stderr, "%s: %s\n", what, kPumpedElsewhereError);
    std::abort();
  }
}

// The next pid: from the shared source when there is one (td 4.5, B32), else the own counter.
int draw_pid(const std::shared_ptr<std::atomic<int>> &source, int &next) {
  return source ? source->fetch_add(1, std::memory_order_relaxed) : next++;
}

// What a posted command never applied, or whose call threw, tells its done: false (S0.0's
// contract; a query has no done). done must not throw.
void refuse_control(control_cmd &cmd) noexcept {
  if (cmd.done && cmd.what != control_cmd::kind::query)
    cmd.done(false);
}

// The start-pinned lookups' hold on a node its host keeps alive (the watch root): a non-owning
// alias, so a lookup takes no reference on it (td 4.8).
boost::shared_ptr<cvc::state> host_held(cvc::state *node) {
  return boost::shared_ptr<cvc::state>(boost::shared_ptr<cvc::state>(), node);
}
} // namespace

async_scheduler::~async_scheduler() {
  home_.reset();
  // Commands posted and never applied: their done gets false, each counted out first. Taken under
  // the lock, so a post that returned before the destruction began is seen. A done that posts
  // again (only a done may post to a scheduler being destroyed) is queued for refusal, never
  // applied, and refused by the next round here: the refusals iterate, and never nest, however
  // often a done posts again. A done that posts again every time it is refused never ends this.
  std::vector<std::pair<int, control_cmd>> left;
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    destroying_ = true;
    left.swap(controls_);
  }
  while (!left.empty()) {
    for (auto &entry : left) {
      controls_waiting_.fetch_sub(1, std::memory_order_release);
      refuse_control(entry.second);
    }
    left.clear();
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    left.swap(controls_);
  }
}

void async_scheduler::set_lock_manager(std::shared_ptr<lock_manager> mgr, lock_attach attach) {
  if (home_)
    for (const auto &[pid, p] : procs_.all())
      if (p->locks && (!p->locks->holds.empty() || p->locks->ticket))
        throw std::logic_error("set_lock_manager: process " + std::to_string(pid) +
                               " holds or waits for a lock");
  // The port shares wakeup_, so a post ends a wait_for_work() (protocol 2.3). The new home is
  // attached before the old one goes, so a refused attach (lock_attach) leaves the scheduler its
  // home and manager. Both are attached meanwhile, which a manager allows; the old one is
  // destroyed here, before any other member (protocol 5.7).
  auto home =
      std::make_unique<detail::lock_home>(std::move(mgr), lock_ops_, policy_, wakeup_, attach);
  home_.swap(home);
  home.reset();
}

detail::lock_home *async_scheduler::locks() {
  if (!home_)
    home_ = std::make_unique<detail::lock_home>(std::make_shared<lock_manager>(), lock_ops_,
                                                policy_, wakeup_);
  return home_.get();
}

void async_scheduler::close_lock_port() {
  // Never through locks(), which would make a private home to close.
  if (has_lock_home())
    home_->port()->close();
}

void async_scheduler::keep_lock_expiries(clock::time_point now) {
  // As scheduler::keep_lock_expiries: no raw pointer outlives a pass that threw; each lock waiter
  // it had not yet timed out gets a deadline due `now`, back on the timer heap, so the next timer
  // pass expires it.
  for (process *p : lock_expired_) {
    if (p->status != process_status::waiting || !p->lock_waiting || p->lock_deadline)
      continue;
    p->lock_deadline = now;
    try {
      procs_.update(*p);
    } catch (...) { // the pass's own exception is the one to report
    }
  }
  lock_expired_.clear();
}

void async_scheduler::lock_waiting_pids(std::vector<int> &out) const {
  out.clear();
  std::size_t left = procs_.lock_waiting_count();
  if (left == 0)
    return;
  // The table counts the waiters (its entry's lock_waiting) but keeps no list of them: the walk
  // stops once it has found them all.
  for (const auto &[pid, p] : procs_.all()) {
    if (!p->table.lock_waiting)
      continue;
    out.push_back(pid);
    if (--left == 0)
      break;
  }
  std::sort(out.begin(), out.end());
}

bool async_scheduler::lock_ops_impl::park_lock(process &p) {
  if (p.status != process_status::ready && p.status != process_status::running)
    return false;
  if (p.status == process_status::running) // close the running slice, as receive_message does
    p.accumulated_time +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - p.last_run_start).count();
  s.procs_.set_status(p, process_status::waiting); // a lock waiter; its deadline a timer
  return true;
}

void async_scheduler::lock_ops_impl::ready_lock_waiter(process &p) {
  s.procs_.set_status(p, process_status::ready);
}

void async_scheduler::lock_ops_impl::relevel(process &p) { s.procs_.update(p); }

std::size_t async_scheduler::lock_ops_impl::lock_expired_count() const {
  return s.procs_.lock_expired_count();
}

void async_scheduler::lock_ops_impl::take_lock_expired(std::vector<process *> &out) {
  s.procs_.take_lock_expired(out);
}

process *async_scheduler::lock_ops_impl::find_process(int pid) { return s.procs_.find(pid); }

void async_scheduler::lock_ops_impl::for_each_process(const std::function<void(process &)> &fn) {
  for (const auto &[pid, p] : s.procs_.all())
    fn(*p);
}

const process_ptr &async_scheduler::no_process() {
  static const process_ptr none;
  return none;
}

// ---------------------------------------------------------------------------
// Process submission (same as sync scheduler)
// ---------------------------------------------------------------------------

int async_scheduler::execute(const std::string &script, const execute_options &opts) {
  detail::check_not_in_async_kernel("async_scheduler::execute");
  verify_pump_owner(*this, "async_scheduler::execute");
  int pid = draw_pid(pid_source_, next_pid_);
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
  // The floor (design 7.2): a host's, explicit, or the submit priority; and what the process
  // keeps alive (a spawned child's context, td 4.15).
  proc.priority_floor = opts.priority_floor.value_or(opts.priority);
  proc.priority_floor_explicit = opts.priority_floor.has_value() && opts.priority_floor_explicit;
  proc.parent_pid = opts.parent_pid;
  proc.keep_alive = opts.keep_alive;
  proc.max_steps = opts.max_steps;
  proc.max_time = opts.max_time;
  proc.max_step_time = opts.max_step_time;
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  // No load_settings() here: the tree's settings as of this submission (submit_settings).
  const submit_settings &set = settings_for_submit();
  proc.max_alloc_bytes = set.max_alloc_bytes;
  proc.heap_check_steps = set.heap_check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(script, opts.env, set.parse);
  proc.state.data_step_nodes = set.data_step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

int async_scheduler::execute(const value_t &expr, const execute_options &opts) {
  detail::check_not_in_async_kernel("async_scheduler::execute");
  verify_pump_owner(*this, "async_scheduler::execute");
  int pid = draw_pid(pid_source_, next_pid_);
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
  // The floor (design 7.2): a host's, explicit, or the submit priority; and what the process
  // keeps alive (a spawned child's context, td 4.15).
  proc.priority_floor = opts.priority_floor.value_or(opts.priority);
  proc.priority_floor_explicit = opts.priority_floor.has_value() && opts.priority_floor_explicit;
  proc.parent_pid = opts.parent_pid;
  proc.keep_alive = opts.keep_alive;
  proc.max_steps = opts.max_steps;
  proc.max_time = opts.max_time;
  proc.max_step_time = opts.max_step_time;
  proc.max_memory = opts.max_memory;
  proc.max_messages = opts.max_messages;
  proc.max_message_bytes = opts.max_message_bytes;
  if (opts.async_steps_per_cpu_ms && *opts.async_steps_per_cpu_ms >= 0.0)
    proc.async_steps_per_cpu_ms = opts.async_steps_per_cpu_ms;
  // No load_settings() here: the tree's settings as of this submission (submit_settings).
  const submit_settings &set = settings_for_submit();
  proc.max_alloc_bytes = set.max_alloc_bytes;
  proc.heap_check_steps = set.heap_check_steps;
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(expr, opts.env);
  proc.state.data_step_nodes = set.data_step_nodes;
  proc.state.stats.start();
  procs_.insert(std::move(pp));
  return pid;
}

const async_scheduler::submit_settings &async_scheduler::settings_for_submit() {
  // Read from the tree on every submission, as before, but only when a setting under its
  // state_exec node may have changed since the last read (settings_changes): a write there, or
  // the node coming or going. A change still applies to the next submission.
  if (!watch_root_) {
    submit_ = submit_settings{kDefaultMaxAllocBytes, heap_options{}.check_steps, parser_defaults_,
                              data_defaults_.step_nodes};
    submit_valid_ = false;
    return submit_;
  }
  const uint64_t seen = settings_changes_.version(watch_root_); // before the reads
  if (submit_valid_ && seen == submit_seen_)
    return submit_;
  submit_.max_alloc_bytes = max_alloc_bytes_from_state(*watch_root_, {});
  submit_.heap_check_steps = heap_options_from_state(*watch_root_).check_steps;
  submit_.parse = parse_options_from_state(*watch_root_, {}, parser_defaults_);
  submit_.data_step_nodes = data_options_from_state(*watch_root_, {}, data_defaults_).step_nodes;
  submit_seen_ = seen;
  submit_valid_ = true;
  return submit_;
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
  // Checked every 64 steps: the check reads the registry's counters. A scheduler given a registry
  // names it (S1.1), else the thread's own is used.
  if ((++cycle_check_ & 63) == 0 &&
      (registry_ ? cycle_collection_due(*registry_) : cycle_collection_due())) {
    if (registry_)
      collect_cycles(*registry_);
    else
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
  if (home_)
    home_->store_progress(now); // the home's last_progress, read by blocking_sinks (design 8.4)
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
  // Posts from other homes' threads (wakes, relevels), first (protocol 5.9). The drain clears
  // the port's pending flag, so wait_for_work, which returns while it is set, never spins.
  if (home_ && home_->port_pending()) {
    home_->drain_port();
    chain_valid_ = false; // the posts are no step's time
  }
  // Cross-thread posts and settled calls, when there are any: the flag spares an empty ingress
  // the lock.
  if (ingress_pending_.load(std::memory_order_acquire)) {
    std::vector<std::pair<std::string, value_t>> batch;
    std::vector<std::shared_ptr<async_call>> finished;
    std::vector<std::pair<int, control_cmd>> controls;
    {
      std::lock_guard<std::mutex> lock(ingress_mutex_);
      batch.swap(ingress_);
      finished.swap(completions_);
      controls.swap(controls_);
      ingress_pending_.store(false, std::memory_order_relaxed);
      // The batch's commands stay counted as waiting, each until it is applied or refused.
    }
    chain_valid_ = false;
    // On the scheduler thread now: deliver_to_receivers mutates process state, which is safe
    // here (no other thread touches the scheduler). A message with no waiter falls into the
    // per-path pending FIFO, exactly as an on-thread msg-send would. Then the settled calls, then
    // the commands posted from other threads (post_control), each in the order they came. One at
    // a time: a call's finish or a kill can throw (a tree write's slots), and what was not yet
    // applied then goes back to the ingress before the exception propagates (S0.7).
    std::size_t m = 0, f = 0, c = 0;
    bool counted_out = false; // the command being applied no longer counts as waiting
    try {
      while (m < batch.size()) {
        auto &[path, msg] = batch[m++];
        deliver_to_receivers(path, msg);
      }
      while (f < finished.size())
        finish_async_call(finished[f++]);
      while (c < controls.size()) {
        auto &[pid, cmd] = controls[c++];
        counted_out = false;
        apply_control(pid, cmd, counted_out);
      }
    } catch (...) {
      if (c > 0) { // a command threw: it is refused, unless its done was called already
        if (!counted_out)
          controls_waiting_.fetch_sub(1, std::memory_order_release);
        refuse_control(controls[c - 1].second);
      }
      requeue_ingress(batch, m, finished, f, controls, c);
      throw;
    }
  }
  if (!async_calls_.empty()) {
    charge_async_calls(in_step ? now_for_step() : clock::now());
    chain_valid_ = false;
  }
  // Sleepers and timed waits ((await f seconds)) that are due wake, on the heap's deadlines; lock
  // waits whose deadline passed time out.
  if (procs_.timed_count() > 0) {
    const clock::time_point now = in_step ? now_for_step() : clock::now();
    if (procs_.timer_due(now)) {
      try {
        if (procs_.wake_due(now, &lock_expired_) > 0) {
          chain_valid_ = false; // the wakes are no step's time
          if (!lock_expired_.empty() && home_)
            home_->wait_expired(lock_expired_);
        }
      } catch (...) {
        keep_lock_expiries(now);
        throw;
      }
      lock_expired_.clear();
    }
  }
}

int async_scheduler::step_once() {
  // A step drains the ingress first (the check is drain_ingress's, as it always was: a step
  // used to begin by calling it).
  detail::check_not_in_async_kernel("async_scheduler::drain_ingress");
  // Apply any cross-thread post_message() deliveries on THIS (scheduler) thread, wake sleeping
  // processes whose deadline has passed, then poll watched paths: once a step.
  drain(true);
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

task<int> async_scheduler::step() {
  const entry_binding bound(registry_, random_);
  chain_valid_ = false;
  co_return step_once();
}

uint64_t async_scheduler::run_loop(std::optional<uint64_t> max_steps,
                                   std::optional<double> max_time) {
  const entry_binding bound(registry_, random_);
  // As the sync scheduler's run (td B17): a stop() from before this run is discarded, and one
  // made once is_running() reads true is seen.
  stop_requested_.store(false, std::memory_order_relaxed);
  running_.store(true, std::memory_order_release);
  chain_valid_ = false;
  // However the run ends (a step's exception too: a throwing on_complete), it is over.
  struct run_end {
    std::atomic<bool> &running;
    bool &chain_valid;
    ~run_end() {
      running.store(false, std::memory_order_release);
      chain_valid = false;
    }
  } end_of_run{running_, chain_valid_};
  uint64_t steps = 0;
  // max_time is a time_budget: the run's own time, not time its thread spent descheduled (a
  // per-frame pump on a loaded machine still finishes a quick action within its frame).
  std::optional<time_budget> budget;
  if (max_time) {
    budget.emplace(*max_time);
    budget->start();
  }

  uint64_t taken = 0;
  while (!stop_requested_.load(std::memory_order_relaxed) && has_runnable()) {
    if (max_steps && steps >= *max_steps)
      break;
    if (budget && budget->spent(now_for_step()))
      break;
    taken += static_cast<uint64_t>(step_once());
    ++steps;
  }
  // The idle collection (S1.1): a group's registry is collected only by its own steps, and an idle
  // group takes none, so a run that leaves nothing runnable collects when one is due. The due
  // rule's 1 s throttle after an exit is enough: a fork-join pool pumps an idle group every frame,
  // and an unthrottled rule would walk the whole registry each frame an Ariadne action ends.
  if (registry_ && !has_runnable() && cycle_collection_due(*registry_))
    collect_cycles(*registry_);
  return taken;
}

task<std::unordered_map<int, value_t>> async_scheduler::run(std::optional<uint64_t> max_steps,
                                                            std::optional<double> max_time) {
  run_loop(max_steps, max_time);
  co_return get_results();
}

int async_scheduler::sync_step() {
  const entry_binding bound(registry_, random_);
  chain_valid_ = false;
  return step_once();
}

std::unordered_map<int, value_t> async_scheduler::sync_run(std::optional<uint64_t> max_steps,
                                                           std::optional<double> max_time) {
  run_loop(max_steps, max_time);
  return get_results();
}

uint64_t async_scheduler::sync_run_steps(std::optional<uint64_t> max_steps,
                                         std::optional<double> max_time) {
  return run_loop(max_steps, max_time);
}

void async_scheduler::stop() {
  // Any thread. No notify: the run loop never waits (wait_for_work is the driver's).
  stop_requested_.store(true, std::memory_order_release);
}

bool async_scheduler::has_runnable() const { return procs_.runnable_count() > 0; }

std::optional<async_scheduler::clock::time_point>
async_scheduler::next_call_charge(clock::time_point now) const {
  // When the next drain has work for the calls in flight: the earliest registration timeout
  // (async_call::past_timeout), and, while a call runs for a process with a step or time limit,
  // the next charge (3.8), which may find the limit reached: a kernel that never checks
  // cancelled() is charged its run time as CPU time, so only a drain stops it. A driver that
  // only drains while nothing is runnable keeps budgets and timeouts current this way.
  // A call whose process ended (an abandoned future) or is gone is no drain's to charge or time
  // out (charge_async_calls passes it over), so it must not end the wait either: the wait would
  // return at once, again and again, until its kernel ended.
  constexpr auto kChargeBeat = std::chrono::milliseconds(1);
  std::optional<clock::time_point> next;
  for (const std::shared_ptr<async_call> &call : async_calls_) {
    if (call->finished())
      continue;
    const process *proc = procs_.find(call->pid());
    if (!proc || is_finished(proc->status))
      continue;
    // When past_timeout holds, none for a timeout it never reaches (NaN, infinite).
    if (const std::optional<clock::time_point> timeout = call->timeout_point())
      next = std::min(next.value_or(clock::time_point::max()), *timeout);
    if (proc->max_time > 0.0 || proc->max_steps > 0)
      next = std::min(next.value_or(clock::time_point::max()), now + kChargeBeat);
  }
  return next;
}

bool async_scheduler::wait_for_work(clock::time_point deadline) {
  detail::check_not_in_async_kernel("async_scheduler::wait_for_work");
  // The calls' next charge or timeout, from when the wait began (the beat must not move on with
  // each wake-up of the loop below).
  const std::optional<clock::time_point> charge =
      async_calls_.empty() ? std::nullopt : next_call_charge(clock::now());
  for (;;) {
    if (procs_.runnable_count() > 0 || ingress_pending_.load(std::memory_order_acquire) ||
        (home_ && home_->port_pending()))
      return true;
    const std::optional<clock::time_point> timer = procs_.next_timer();
    const clock::time_point now = clock::now();
    if (timer && *timer <= now)
      return true;
    if (charge && *charge <= now)
      return true;
    if (now >= deadline)
      return false;
    clock::time_point until = timer ? std::min(*timer, deadline) : deadline;
    if (charge)
      until = std::min(until, *charge);
    // A sleeper's or timed wait's deadline is kept precisely; the others need not be.
    const bool precise = timer && *timer == until;
    // Announce the wait, then look at the ingress once more: a post either lands before this
    // look (seen here) or sees idle_waiting_ and notifies (both sides are seq_cst). A lock post
    // notifies whatever idle_waiting_ says (protocol 2.3): one this look misses ends the wait.
    idle_waiting_.store(true, std::memory_order_seq_cst);
    if (!ingress_pending_.load(std::memory_order_seq_cst) && !(home_ && home_->port_pending()))
      wakeup_->wait_until(until, precise);
    idle_waiting_.store(false, std::memory_order_relaxed);
  }
}

bool async_scheduler::wait_for_work(double seconds) {
  return wait_for_work(detail::time_after(clock::now(), seconds));
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
  proc.wake_time = detail::time_after(now, seconds); // saturating: a huge sleep doesn't wrap
  procs_.set_status(proc, process_status::waiting);  // a timer for wake_time
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
  const entry_binding bound(registry_, random_);
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
  proc.recv_deadline = detail::time_after(std::chrono::steady_clock::now(), seconds);
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
  // The wait is notified under the lock: once the poster lets go of it, it touches the scheduler
  // no more, so a scheduler thread that drains this post may destroy the scheduler.
  std::lock_guard<std::mutex> lock(ingress_mutex_);
  ingress_.emplace_back(path, msg);
  ingress_pending_.store(true, std::memory_order_seq_cst);
  notify_work();
}

void async_scheduler::drain_ingress() {
  detail::check_not_in_async_kernel("async_scheduler::drain_ingress");
  const entry_binding bound(registry_, random_);
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
  call->owner_serial_ = serial(); // what the foreign-future check compares (S1-Q2)
  async_calls_.push_back(std::move(call));
}

void async_scheduler::post_async_completion(std::shared_ptr<async_call> call) {
  std::lock_guard<std::mutex> lock(ingress_mutex_); // notified under it: see post_message
  completions_.push_back(std::move(call));
  ingress_pending_.store(true, std::memory_order_seq_cst);
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
  // Finished once (td 4.9, S0.9): the first finish or cancel claims the call, and a later one (its
  // completion after a cancel or a timeout, or after an outcome() that finished it first) charges
  // nothing more. The processes waiting on its channel are woken either way: one that awaited
  // the call since (a fork child, once its parent was killed) would otherwise wait for good.
  if (call->closed_.exchange(true, std::memory_order_acq_rel)) {
    wake_async_waiters(call->channel());
    return;
  }
  const auto it = std::find(async_calls_.begin(), async_calls_.end(), call);
  if (it == async_calls_.end()) {
    wake_async_waiters(call->channel()); // not tracked here: nothing to charge
    return;
  }
  async_calls_.erase(it);
  // Claimed and off the list: nothing finishes it again, so its waiters are woken even when what
  // follows throws (the rate's publication runs the tree's slots; a kill at a limit can throw).
  try {
    process *proc = procs_.find(call->pid());
    double rate = 0.0;
    if (proc && !is_finished(proc->status)) {
      // Read before charging: publishing the rate writes the tree, whose slots may kill or reap.
      rate = async_steps_per_cpu_ms(*proc);
      proc = procs_.find(call->pid());
    }
    if (proc && !is_finished(proc->status)) {
      // The rest of the call is charged before its process resumes (3.8): a call that used up
      // the budget kills the process instead.
      const clock::time_point now = clock::now();
      detail::charge_async_call(*proc, *call, rate, true, now);
      if (proc->max_memory > 0)
        proc->heap_alloc_bytes += call->alloc_bytes();
      check_limits(*proc, now);
    } else {
      call->charged.closed = true;
    }
  } catch (...) {
    wake_async_waiters(call->channel());
    throw;
  }
  wake_async_waiters(call->channel());
}

void async_scheduler::charge_async_calls(clock::time_point now) {
  // Every started call is charged its run and CPU time so far (3.8), then the processes it
  // charged are checked against their limits, all at the one reading `now` (they are waiting:
  // their elapsed time doesn't depend on it). The tree's rate is read once, before the walk:
  // reading it may publish it, a write to the tree whose slots may kill or reap, and none may
  // run while the list is walked. So the walk is in place: only a call's own timeout changes
  // the list then (its settling may kill), and from the first one on, the rest goes over a copy.
  const std::optional<double> tree = tree_rate_for_calls();
  charged_.clear();
  for (std::size_t i = 0; i < async_calls_.size(); ++i) {
    async_call &call = *async_calls_[i];
    const int pid = call.pid();
    process *proc = procs_.find(pid);
    if (!proc || is_finished(proc->status))
      continue;
    if (!call.finished() && call.past_timeout(now)) {
      charge_async_calls_from(i, now, tree);
      return;
    }
    const std::optional<double> rate = walk_rate(*proc, call, tree);
    if (!rate)
      continue;
    detail::charge_async_call(*proc, call, *rate, false, now);
    if (std::find(charged_.begin(), charged_.end(), pid) == charged_.end())
      charged_.push_back(pid);
  }
  check_charged(now);
}

void async_scheduler::charge_async_calls_from(std::size_t first, clock::time_point now,
                                              std::optional<double> tree) {
  // A copy: a timeout settles its call, whose final charge may kill its process, and settling
  // or killing may publish the rate, a write to the tree whose slots may change the list or
  // reap. Processes are found again by pid after anything that may have reaped one.
  const std::vector<std::shared_ptr<async_call>> calls(
      async_calls_.begin() + static_cast<std::ptrdiff_t>(first), async_calls_.end());
  for (const std::shared_ptr<async_call> &call : calls) {
    const int pid = call->pid();
    process *proc = procs_.find(pid);
    if (!proc || is_finished(proc->status))
      continue;
    if (!call->finished() && call->past_timeout(now) && call->settle_timeout()) {
      // Its own timeout: it raises TimeoutError at the call, and its kernel is told to stop. It
      // is finished, and its waiters woken, even when its cancel hook throws (against its
      // contract), which is reported after.
      try {
        call->cancel();
      } catch (...) {
        finish_async_call(call);
        throw;
      }
      finish_async_call(call);
      continue;
    }
    const std::optional<double> rate = walk_rate(*proc, *call, tree);
    if (!rate)
      continue;
    detail::charge_async_call(*proc, *call, *rate, false, now);
    if (std::find(charged_.begin(), charged_.end(), pid) == charged_.end())
      charged_.push_back(pid);
  }
  check_charged(now);
}

std::optional<double> async_scheduler::tree_rate_for_calls() {
  // The rate the calls without one of their own are charged at, when a started call of a live
  // process needs it: read (and published) now, before the walk.
  if (!watch_root_)
    return kDefaultAsyncStepsPerCpuMs;
  for (const std::shared_ptr<async_call> &call : async_calls_) {
    if (!call->started())
      continue;
    const process *proc = procs_.find(call->pid());
    if (proc && !is_finished(proc->status) && !proc->async_steps_per_cpu_ms)
      return tree_rate();
  }
  return std::nullopt;
}

std::optional<double> async_scheduler::walk_rate(const process &proc, const async_call &call,
                                                 std::optional<double> tree) {
  if (!call.started())
    return std::nullopt; // queued: waiting for a thread isn't charged
  if (proc.async_steps_per_cpu_ms)
    return *proc.async_steps_per_cpu_ms;
  return tree; // nullopt: it started after the read, and is charged from the next drain on
}

void async_scheduler::check_charged(clock::time_point now) {
  // At a limit the process is killed (2.5), and its calls cancelled. Found again by pid, and
  // walked by index: cancelling may publish the rate, a write to the tree whose slots may reap.
  for (std::size_t k = 0; k < charged_.size(); ++k) {
    process *proc = procs_.find(charged_[k]);
    if (proc && !is_finished(proc->status))
      check_limits(*proc, now);
  }
}

void async_scheduler::cancel_async_calls(int pid) {
  const auto of_pid = [pid](const std::shared_ptr<async_call> &c) { return c->pid() == pid; };
  const std::size_t n =
      static_cast<std::size_t>(std::count_if(async_calls_.begin(), async_calls_.end(), of_pid));
  if (n == 0)
    return;
  // Out of the list first, then the rate: reading it may publish it, a write to the tree whose
  // slots may do anything, kill this process too (that kill finds no calls left to cancel, so it
  // doesn't read the rate again and recurse), and none of them runs mid-walk.
  // Each is claimed as it leaves the list (td 4.9, S0.9), so its completion, when it comes,
  // charges nothing more. And it is settled, with the error every await of it raises from now on
  // (another process's: a fork child's), however it awaits: parked in await or await-all, after
  // the cancel, or blocked in a generator's body. Settled before its cancel hook runs, so the
  // kernel's own settling, once told to stop, is ignored whenever it comes, work that never
  // settles once cancelled (async_start) is finished all the same, and a hook that throws
  // (against its contract) skips neither the settle nor the wake: it is reported once they are
  // done.
  std::vector<std::shared_ptr<async_call>> calls;
  calls.reserve(n); // so recording a claimed call can't fail
  std::exception_ptr hook_error;
  for (auto it = async_calls_.begin(); it != async_calls_.end();) {
    if (!of_pid(*it)) {
      ++it;
      continue;
    }
    std::shared_ptr<async_call> call = std::move(*it);
    it = async_calls_.erase(it);
    if (!call->closed_.exchange(true, std::memory_order_acq_rel)) {
      calls.push_back(call);
      const std::string &name = call->function_name();
      call->settle({},
                   std::make_exception_ptr(dsl_error(
                       "RuntimeError", name + ": cancelled: the process that made the call ended",
                       "async", {{"function", value_t(name)}})));
    }
    try {
      call->cancel();
    } catch (...) {
      if (!hook_error)
        hook_error = std::current_exception();
    }
  }
  // Another process may be waiting for one of them: a fork child awaiting the same future. It is
  // woken, and reads the call's outcome, the error above (S0.9). Woken before the rate is read:
  // a slot that throws when the rate is published must not leave it waiting for a call nothing
  // finishes again.
  for (const std::shared_ptr<async_call> &call : calls)
    wake_async_waiters(call->channel());
  process *proc = procs_.find(pid);
  const double rate = proc ? async_steps_per_cpu_ms(*proc) : 0.0;
  proc = procs_.find(pid);
  if (proc) {
    // What they ran so far stays charged; their results, when they come, are dropped.
    const auto now = std::chrono::steady_clock::now();
    for (const std::shared_ptr<async_call> &call : calls)
      detail::charge_async_call(*proc, *call, rate, true, now);
  }
  if (hook_error)
    std::rethrow_exception(hook_error);
}

void async_scheduler::cancel_async_calls_of(int pid) {
  const process *proc = procs_.find(pid);
  if (proc && !is_finished(proc->status))
    return; // live: its calls wake it when they finish, or its kill cancels them
  cancel_async_calls(pid);
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
    publish_async_steps_per_cpu_ms(*watch_root_, id_, rate); // a tree write: its slots run now
    published_rate_ = rate;
  }
  rate_seen_ = seen;
  rate_valid_ = counted;
  rate_ = rate;
  return rate;
}

void async_scheduler::track_tree_changes() {
  // Counted while something reads the tree at every step: watches that resolve under the watch
  // root (one with a root of its own is counted in watch_roots_), or the rate of the calls in
  // flight. Otherwise nothing is connected, and a write under the root pays nothing for it.
  const bool needed = watch_root_ && ((!procs_.watchers().empty() && watch_roots_.uses_main()) ||
                                      !async_calls_.empty());
  cvc::state *const want = needed ? watch_root_ : nullptr;
  if (tree_changes_.root() == want)
    return;
  tree_changes_.watch(want);
  chain_valid_ = false; // connecting is no step's time
  rate_valid_ = false;
  watch_seen_ = tree_changes_.count() - 1; // poll the watches once: changes before weren't counted
}

void async_scheduler::set_watch_root(cvc::state *root) { set_watch_root(root, nullptr); }

void async_scheduler::set_watch_root(cvc::state *root, boost::shared_ptr<cvc::state> pin) {
  verify_pump_owner(*this, "async_scheduler::set_watch_root");
  if (root != watch_root_) {
    rate_valid_ = false;
    submit_valid_ = false;
    settings_changes_.clear(); // the node it holds is the old root's
    watch_roots_.invalidate(); // an entry whose root this is now counts under it
  }
  watch_root_ = root;
  track_tree_changes();
  // A host keeps its root alive; one a step gave is kept here (R0-mid). The old hold goes last,
  // once nothing is connected to what it held.
  watch_root_pin_.swap(pin);
}

int async_scheduler::restore(process_ptr proc) {
  verify_pump_owner(*this, "async_scheduler::restore");
  const entry_binding bound(registry_, random_);
  // A stack that holds a lock (a lock_body with its hold, or a lock_enter woken with a grant)
  // can't be restored: no snapshot makes one, and its hold is no restored process's (protocol
  // 5.11; LW-12).
  for (const eval_frame &f : proc->state.stack) {
    const bool body = f.phase == eval_phase::lock_body;
    const bool enter = f.phase == eval_phase::lock_enter;
    if (!(body || enter) || f.extra_vals.size() < 7)
      continue;
    const std::size_t slot = body ? 6 : 3; // lock_frame_slot::hold / wait_or_outcome
    if (std::holds_alternative<int64_t>(f.extra_vals[slot].v))
      throw std::invalid_argument("restore: the process's stack holds a lock");
  }
  const int pid = draw_pid(pid_source_, next_pid_);
  proc->pid = pid;
  if (!is_finished(proc->status))
    proc->status = process_status::ready;
  proc->wake_time.reset();
  proc->recv_path.reset();
  proc->recv_deadline.reset();
  proc->awaiting_frame = false;
  // A restored process is a new one: no lock of the one it was made from comes with it.
  proc->locks.reset();
  proc->lock_waiting = false;
  proc->lock_deadline.reset();
  proc->lock_expired_pending = false;
  proc->inherited_priority = INT_MAX;
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
  register_watch_handler(pid, watch_id, std::move(handler), path, nullptr);
}

void async_scheduler::register_watch_handler(int pid, int watch_id, value_t handler,
                                             const std::string &path,
                                             boost::shared_ptr<cvc::state> root) {
  verify_pump_owner(*this, "async_scheduler::register_watch_handler");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  // The path resolves under the watch's own root when it has one (td 4.8), else under the watch
  // root.
  std::string initial;
  // Start-pinned: the entry holds its root, the host the watch root.
  if (const boost::shared_ptr<cvc::state> under = root ? root : host_held(watch_root_))
    if (const boost::shared_ptr<cvc::state> node = under->findDescendantShared(under, path))
      initial = node->value();
  it->second->watch_handlers[watch_id] = {std::move(handler), path, initial, std::move(root)};
  procs_.update(*it->second);
  watch_roots_.invalidate();
}

void async_scheduler::unregister_watch_handler(int pid, int watch_id) {
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return;
  it->second->watch_handlers.erase(watch_id);
  procs_.update(*it->second);
  watch_roots_.invalidate();
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
  // Only the processes with watches, and only the watches under a root whose tree changed since
  // the last poll (tree_changes). A watch resolves under its own root (a chrooted context's) or
  // the watch root, and each root is counted on its own (td 4.8, S0.5). A value the poll finds
  // changed queues a watch event.
  const std::vector<process *> &watchers = procs_.watchers();
  if (watchers.empty()) {
    if (!watch_roots_.empty()) {
      watch_roots_.clear(); // no slot on the tree while nothing watches it
      chain_valid_ = false;
    }
  } else if (watch_roots_.sync(watchers, watch_root_)) {
    chain_valid_ = false; // connecting is no step's time
  }
  if (watch_root_)
    track_tree_changes();
  if (watchers.empty())
    return;
  bool main_moved = false;
  if (watch_root_ && watch_roots_.uses_main()) {
    const uint64_t seen = tree_changes_.count();
    main_moved = seen != watch_seen_;
    watch_seen_ = seen;
  }
  const bool roots_moved = watch_roots_.poll();
  if (!main_moved && !roots_moved)
    return;
  chain_valid_ = false; // the reads below are no step's time
  const boost::shared_ptr<cvc::state> held_root = host_held(watch_root_);
  for (process *proc : watchers) {
    for (auto &[wid, entry] : proc->watch_handlers) {
      cvc::state *const root = entry.root ? entry.root.get() : watch_root_;
      if (!root || !(root == watch_root_ ? main_moved : watch_roots_.moved(root)))
        continue;
      ++watch_polls_;
      // Pinned while it is read: another thread may delete it meanwhile (td 4.8). The lookup is
      // start-pinned: the entry holds its own root, the host the watch root, so it takes no
      // reference on the root.
      const boost::shared_ptr<cvc::state> &start = entry.root ? entry.root : held_root;
      const boost::shared_ptr<cvc::state> node = start->findDescendantShared(start, entry.path);
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
  verify_pump_owner(*this, "async_scheduler::send_signal");
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
  // Its locks go before on_complete runs (design 5.6); a hold left now is a leak, counted.
  if (const detail::lock_test_hooks *hooks = detail::lock_test_hooks_get();
      hooks && hooks->before_terminate_end)
    hooks->before_terminate_end(proc);
  if (proc.locks && home_)
    home_->process_ended(proc, false);
  proc.exit_code = std::move(result);
  proc.state.stats.mark_complete();
  if (proc.on_complete) {
    proc.on_complete(proc.exit_code);
  }
  release_if_idle(proc);
}

void async_scheduler::kill_process(process &proc, const std::string &reason) {
  const int pid = proc.pid;
  // Its kernels see cancelled(), and their results are dropped. Cancelling can throw (a cancel
  // hook, against its contract; the rate's publication, which runs the tree's slots): the process
  // is killed all the same, and the exception reported after (R0-final), as a kill whose locks'
  // release throws leaves its process killed. It must not run on with its calls cancelled.
  std::exception_ptr cancel_error;
  try {
    cancel_async_calls(pid);
  } catch (...) {
    cancel_error = std::current_exception();
  }
  // Cancelling may publish the rate, a write to the tree whose slots may have ended (or reaped)
  // the process meanwhile: that stands.
  process *const p = procs_.find(pid);
  if (p && !is_finished(p->status)) {
    procs_.set_status(*p, process_status::killed);
    if (p->locks && home_)
      home_->process_ended(*p, true); // a kill or a budget: the sections it was in are abandoned
    p->exit_error = reason;
    p->state.stats.mark_complete();
    release_if_idle(*p);
  }
  if (cancel_error)
    std::rethrow_exception(cancel_error);
}

void async_scheduler::fail_process(process &proc, value_t exc, std::exception_ptr original) {
  procs_.set_status(proc, process_status::failed);
  if (proc.locks && home_)
    home_->process_ended(proc, true);
  proc.exit_error = exception_summary(exc);
  proc.exit_exception = std::move(exc);
  proc.exit_original = std::move(original);
  proc.state.stats.mark_complete();
  release_if_idle(proc);
}

void async_scheduler::release_if_idle(process &proc) {
  // What it leaked is garbage now (heap.h): noted in this scheduler's registry when it has one
  // (S1.1), so a host's kill(pid) between pumps notes the group's, not the host thread's.
  if (registry_)
    note_process_exit(*registry_);
  else
    note_process_exit();
  // A process stepping right now (one that killed itself) is released once its step is over.
  if (proc.pid != current_pid_)
    proc.release_state();
}

bool async_scheduler::pause(int pid) {
  verify_pump_owner(*this, "async_scheduler::pause");
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
  verify_pump_owner(*this, "async_scheduler::resume");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::paused)
    return false;
  procs_.set_status(proc, process_status::ready);
  return true;
}

bool async_scheduler::kill(int pid) { return kill(pid, "killed_by_user"); }

bool async_scheduler::kill(int pid, const std::string &reason) {
  detail::check_not_in_async_kernel("async_scheduler::kill");
  verify_pump_owner(*this, "async_scheduler::kill");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  auto &proc = *it->second;
  if (is_finished(proc.status))
    return false;
  kill_process(proc, reason);
  return true;
}

int async_scheduler::kill_owner(const std::string &owner) {
  verify_pump_owner(*this, "async_scheduler::kill_owner");
  // The owner's live processes first, then each by its pid (S1.4; reap is the model): a kill can
  // publish the step rate, whose tree slots may reap or submit processes, so the loop never ranges
  // over the table itself. One ended or reaped meanwhile is passed over, and one submitted
  // meanwhile is not killed. Every kill is tried; the first that threw is rethrown after the rest,
  // its process killed all the same (kill_process).
  std::vector<int> pids;
  for (const auto &[pid, proc] : procs_.all())
    if (proc->owner == owner && !is_finished(proc->status))
      pids.push_back(pid);
  int killed = 0;
  std::exception_ptr first_error;
  for (const int pid : pids) {
    process *const p = procs_.find(pid);
    if (!p || is_finished(p->status))
      continue;
    try {
      kill_process(*p, "owner_reaped");
    } catch (...) {
      if (!first_error)
        first_error = std::current_exception();
    }
    ++killed;
  }
  if (first_error)
    std::rethrow_exception(first_error);
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
  verify_pump_owner(*this, "async_scheduler::restore");
  const entry_binding bound(registry_, random_); // the decode tracks class scopes
  process_ptr proc = decode_process(node);
  rebind_natives(proc->state, natives);
  return restore(std::move(proc));
}

int async_scheduler::fork(int pid) {
  verify_pump_owner(*this, "async_scheduler::fork");
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
  // A holder can't fork: the child would run inside sections it doesn't hold (design 5.11).
  if (parent.locks && !parent.locks->holds.empty())
    return -1;

  int child_pid = draw_pid(pid_source_, next_pid_);
  auto child = make_process();
  child->pid = child_pid;
  child->name = parent.name + "-fork";
  child->status = process_status::ready;
  child->priority = parent.priority;
  child->uid = parent.uid;
  child->gid = parent.gid;
  child->root_path = parent.root_path;
  child->owner = parent.owner; // td B28: a host reaps the child with its parent (kill_owner)
  child->priority_floor = parent.priority_floor; // design 7.2: fork copies the floor
  child->priority_floor_explicit = parent.priority_floor_explicit;
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
  child->keep_alive = parent.keep_alive; // it shares the parent's context (td 4.15)

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
  detail::fork_scopes(child->state, parent.state.global_env, child_env);
  // Forked from inside its own step (the DSL's (fork), td B28): the top frame is the call that
  // applies fork, which the parent finishes with the child's pid. The child's copy is finished
  // here with 0, as hand_up finishes a call, so the child goes on after the call instead of
  // calling fork again.
  if (pid == current_pid() && !child->state.stack.empty()) {
    std::vector<eval_frame> &stack = child->state.stack;
    stack.pop_back();
    if (!stack.empty()) {
      stack.back().results.push_back(value_t(static_cast<int64_t>(0)));
    } else {
      child->state.result = value_t(static_cast<int64_t>(0));
      child->state.done = true;
    }
  }
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
  verify_pump_owner(*this, "async_scheduler::set_priority");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  it->second->priority = priority;
  procs_.update(*it->second); // its run-queue level, under the priority policies
  if (it->second->locks && home_)
    home_->set_base(*it->second); // the manager's mirror of the base (protocol 5.8, I8)
  return true;
}

// ---------------------------------------------------------------------------
// Priorities (docs/STATE_EXEC_LOCKS.md 7.4; the sync scheduler's, on the same table)
// ---------------------------------------------------------------------------

namespace {
// The effective priority get-priority and inspect report: the one the pick uses, which under
// round_robin is the base (design 7.1).
int reported_effective(const process &p, scheduling_policy policy) {
  return policy == scheduling_policy::round_robin ? p.priority : p.effective_priority();
}

// The descent walk's lookup (design 7.2): a live process of `procs`.
std::function<const process *(int)> live_in(const detail::process_table &procs) {
  return [&procs](int pid) -> const process * {
    const process *p = procs.find(pid);
    return p && !is_finished(p->status) ? p : nullptr;
  };
}
} // namespace

std::optional<priority_info> async_scheduler::get_priority(int pid) const {
  const process *p = procs_.find(pid);
  if (!p)
    return std::nullopt;
  return priority_info{p->priority, reported_effective(*p, policy_), detail::floor_of(*p), policy_,
                       p->priority_floor_explicit};
}

priority_change async_scheduler::set_priority_checked(int caller_pid, int pid, int priority) {
  verify_pump_owner(*this, "async_scheduler::set_priority_checked");
  const process *caller = procs_.find(caller_pid);
  const process *target = procs_.find(pid);
  if (!caller || !target)
    return priority_change::unknown;
  const priority_change r =
      detail::check_priority_change(*caller, *target, priority, live_in(procs_));
  if (r == priority_change::applied)
    set_priority(pid, priority);
  return r;
}

std::optional<priority_info> async_scheduler::get_priority_as(int caller_pid, int pid) const {
  if (caller_pid != pid) { // a caller's own pid is always visible to it
    const process *caller = procs_.find(caller_pid);
    const process *target = procs_.find(pid);
    if (!caller || !target || !detail::visible_to(*caller, *target, live_in(procs_)))
      return std::nullopt;
  }
  return get_priority(pid);
}

bool async_scheduler::set_priority_floor(int pid, int floor) {
  verify_pump_owner(*this, "async_scheduler::set_priority_floor");
  process *p = procs_.find(pid);
  if (!p)
    return false;
  p->priority_floor = floor;
  p->priority_floor_explicit = true;
  return true;
}

void async_scheduler::note_priority_clamped() {
  // The count is the record: state_exec has no log sink to tell once (gap G-5).
  ++priority_clamped_;
}

bool async_scheduler::post_control(int pid, control_cmd cmd) {
  // Any thread. Queued with the ingress and applied by the next drain, on the scheduler thread
  // (apply_control): the posting thread may not read procs_, so an unknown pid is found, and
  // refused, there. Notified under the lock, as post_message is.
  std::lock_guard<std::mutex> lock(ingress_mutex_);
  controls_.emplace_back(pid, std::move(cmd));
  controls_waiting_.fetch_add(1, std::memory_order_release);
  if (destroying_)
    return false; // a done the destructor calls posted this: the destructor refuses it next
  ingress_pending_.store(true, std::memory_order_seq_cst);
  notify_work();
  return true;
}

void async_scheduler::apply_control(int pid, control_cmd &cmd, bool &counted_out) {
  // The command stops counting as waiting once its call is made, before its done hears what the
  // call returned (a query has no done); `counted_out` tells the batch it was.
  const auto settle = [this, &cmd, &counted_out](bool result) {
    controls_waiting_.fetch_sub(1, std::memory_order_release);
    counted_out = true;
    if (cmd.done && cmd.what != control_cmd::kind::query) {
      // Taken from the command first: a done that throws (against its contract) is not called
      // again by the refusal of the command that threw.
      const std::function<void(bool)> done = std::move(cmd.done);
      cmd.done = nullptr;
      done(result);
    }
  };
  bool applied = false;
  try {
    switch (cmd.what) {
    case control_cmd::kind::query:
      if (cmd.query)
        cmd.query(*this);
      break;
    case control_cmd::kind::set_priority:
      applied = set_priority(pid, cmd.value); // the host path, unchecked (design 7.4)
      break;
    case control_cmd::kind::kill:
      applied = kill(pid); // killed_by_user
      break;
    case control_cmd::kind::pause:
      applied = pause(pid);
      break;
    case control_cmd::kind::resume:
      applied = resume(pid);
      break;
    case control_cmd::kind::signal:
      applied = send_signal(pid, cmd.signal); // "SIGKILL" kills
      break;
    }
  } catch (...) {
    // A kill (or a SIGKILL) that throws has still killed its process (kill_process; R0-final), and
    // its done hears that. Any other command that throws is refused, done(false), by the batch.
    if (cmd.what == control_cmd::kind::kill || cmd.what == control_cmd::kind::signal) {
      const process *const p = procs_.find(pid);
      if (p && p->status == process_status::killed)
        settle(true);
    }
    throw;
  }
  settle(applied);
}

void async_scheduler::requeue_ingress(std::vector<std::pair<std::string, value_t>> &batch,
                                      std::size_t m,
                                      std::vector<std::shared_ptr<async_call>> &finished,
                                      std::size_t f,
                                      std::vector<std::pair<int, control_cmd>> &controls,
                                      std::size_t c) noexcept {
  // The entry that threw is not put back: a delivery or a finish may have half happened, and the
  // command was refused.
  try {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    ingress_pending_.store(true, std::memory_order_seq_cst); // the next drain looks again
    // An insert that fails (no memory) leaves its queue as it was.
    ingress_.insert(ingress_.begin(), std::make_move_iterator(batch.begin() + m),
                    std::make_move_iterator(batch.end()));
    completions_.insert(completions_.begin(), std::make_move_iterator(finished.begin() + f),
                        std::make_move_iterator(finished.end()));
    f = finished.size();
    if (c < controls.size()) { // back in the queue: still waiting
      controls_.insert(controls_.begin(), std::make_move_iterator(controls.begin() + c),
                       std::make_move_iterator(controls.end()));
      c = controls.size();
    }
  } catch (...) {
    // No memory to queue them again. A settled call left behind still wakes who waits for it: the
    // waiter's outcome() finishes the call. The commands left are refused rather than lost.
    for (; f < finished.size(); ++f) {
      try {
        wake_async_waiters(finished[f]->channel());
      } catch (...) {
      }
    }
    for (; c < controls.size(); ++c) {
      controls_waiting_.fetch_sub(1, std::memory_order_release);
      refuse_control(controls[c].second);
    }
  }
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
  info.locks_held = proc.locks ? proc.locks->holds.size() : 0;
  info.effective_priority = reported_effective(proc, policy_);
  info.priority_floor = detail::floor_of(proc);
  if (proc.status == process_status::waiting) {
    if (proc.lock_waiting)
      info.waiting_on = "lock /" + detail::lock_home::waiting_key(proc);
    else if (proc.recv_path)
      info.waiting_on = "msg " + *proc.recv_path;
    else if (proc.wake_time)
      info.waiting_on = std::string("sleep");
    else if (proc.awaiting_frame)
      info.waiting_on = std::string("frame");
  }
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

bool async_scheduler::reap(int pid, process_ptr *released) {
  verify_pump_owner(*this, "async_scheduler::reap");
  auto it = procs_.all().find(pid);
  if (it == procs_.all().end())
    return false;
  const process_status st = it->second->status;
  if (!is_finished(st))
    return false; // never drop a still-live (ready/running/waiting/paused) process
  // Held across the cancel, whose rate read may publish to the tree: its slots may change the
  // process table (reap this process, add others), so `it` is not read after it.
  process_ptr proc = it->second;
  cancel_async_calls(pid);
  // Its env, state, watch_handlers and captured values (including any Python holders) all live on
  // the process object, so erasing it frees them, or `released` takes them to free later. The
  // process stepping now (its on_complete reaping it) is erased once its step is over.
  if (pid == current_pid_) {
    reap_after_step_ = pid;
  } else {
    procs_.erase(pid);
    if (released)
      *released = std::move(proc);
  }
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
  s.priority_clamped = priority_clamped_;
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

#include <algorithm>
#include <chrono>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h>
#include <cvc/core/state_exec/builtins.h>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

async_scheduler::async_scheduler(scheduling_policy policy)
    : policy_(policy), evaluator_(builtins::make_default_environment()) {}

// ---------------------------------------------------------------------------
// Process submission (same as sync scheduler)
// ---------------------------------------------------------------------------

int async_scheduler::execute(const std::string &script, const execute_options &opts) {
  int pid = next_pid_++;
  process proc;
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
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(script, opts.env);
  proc.state.stats.start();
  processes_.emplace(pid, std::make_shared<process>(std::move(proc)));
  return pid;
}

int async_scheduler::execute(const value_t &expr, const execute_options &opts) {
  int pid = next_pid_++;
  process proc;
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
  proc.signal_handlers = opts.signal_handlers;
  proc.on_complete = opts.on_complete;
  proc.create_time = std::chrono::steady_clock::now();
  proc.state = evaluator_.create_state(expr, opts.env);
  proc.state.stats.start();
  processes_.emplace(pid, std::make_shared<process>(std::move(proc)));
  return pid;
}

// ---------------------------------------------------------------------------
// Process selection (same logic as sync)
// ---------------------------------------------------------------------------

process_ptr async_scheduler::select_process() {
  std::vector<process *> runnable;
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::ready || proc->status == process_status::running) {
      runnable.push_back(proc.get());
    }
  }
  if (runnable.empty())
    return nullptr;

  std::sort(runnable.begin(), runnable.end(),
            [](const process *a, const process *b) { return a->pid < b->pid; });

  switch (policy_) {
  case scheduling_policy::round_robin: {
    rr_index_ = rr_index_ % static_cast<int>(runnable.size());
    auto *p = runnable[rr_index_];
    rr_index_ = (rr_index_ + 1) % static_cast<int>(runnable.size());
    return processes_.at(p->pid);
  }
  case scheduling_policy::priority: {
    std::sort(runnable.begin(), runnable.end(),
              [](const process *a, const process *b) { return a->priority < b->priority; });
    return processes_.at(runnable[0]->pid);
  }
  case scheduling_policy::priority_rr: {
    std::sort(runnable.begin(), runnable.end(), [](const process *a, const process *b) {
      if (a->priority != b->priority)
        return a->priority < b->priority;
      return a->pid < b->pid;
    });
    int best = runnable[0]->priority;
    std::vector<process *> same_prio;
    for (auto *p : runnable) {
      if (p->priority == best)
        same_prio.push_back(p);
    }
    rr_index_ = rr_index_ % static_cast<int>(same_prio.size());
    auto *p = same_prio[rr_index_];
    rr_index_ = (rr_index_ + 1) % static_cast<int>(same_prio.size());
    return processes_.at(p->pid);
  }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Step execution (same logic as sync)
// ---------------------------------------------------------------------------

void async_scheduler::execute_process_step(process &proc) {
  // Track which process is currently executing so intrinsics can query it (self/ps).
  current_pid_ = proc.pid;
  current_proc_ = processes_[proc.pid];
  struct guard {
    int &pid;
    process_ptr &pp;
    ~guard() {
      pid = -1;
      pp.reset();
    }
  } reset_guard{current_pid_, current_proc_};

  if (!proc.pending_signals.empty() && !proc.in_signal_handler && !proc.in_watch_handler) {
    handle_signal(proc);
    return;
  }

  if (!proc.pending_watch_events.empty() && !proc.in_signal_handler && !proc.in_watch_handler) {
    handle_watch_event(proc);
    return;
  }

  proc.status = process_status::running;
  proc.last_run_start = std::chrono::steady_clock::now();

  // Arm the cooperative deadline for this step so an uncapped nested evaluator or a
  // long-running builtin aborts at the process's remaining time budget too — the same
  // guarantee the sync scheduler has (scheduler.cpp). max_time <= 0 means unlimited.
  std::optional<double> step_budget;
  if (proc.max_time > 0.0) {
    const double remaining = proc.max_time - proc.elapsed_time();
    step_budget = remaining > 0.0 ? remaining : 0.0;
  }
  eval_deadline_guard step_deadline(step_budget);
  // Drive one async_stackless_evaluator step to completion at the leaf. The evaluator's step()
  // is a coroutine that runs one inner stackless step then yields a suspend_point; sync_wait
  // consumes that yield here. (Propagating the suspend_point up into step()/run() as a true
  // co_await chain is only meaningful when a coroutine executor drives the scheduler for
  // intra-run interleaving — not the case for the synchronous sync_step/sync_run + per-frame
  // pump paths — and multi-level co_await + suspend across sync_wait is not robust here.)
  bool done = evaluator_.step(proc.state).sync_wait();

  auto now = std::chrono::steady_clock::now();
  proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();

  if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.done) {
    restore_from_signal(proc);
    proc.status = process_status::ready;
    check_limits(proc);
    return;
  }

  check_limits(proc);
  if (proc.status == process_status::killed)
    return;

  if (done) {
    terminate_process(proc, proc.state.result);
  } else if (proc.status == process_status::running) {
    // Only reset to ready if the intrinsic didn't change status (sleep/msg-recv set waiting).
    proc.status = process_status::ready;
  }
}

// ---------------------------------------------------------------------------
// Coroutine stepping
// ---------------------------------------------------------------------------

task<int> async_scheduler::step() {
  // Apply any cross-thread post_message() deliveries on THIS (scheduler) thread, then wake
  // sleeping processes whose deadline has passed, then poll watched paths.
  drain_ingress();
  wake_sleeping_processes();
  poll_watches();
  auto proc = select_process();
  if (!proc)
    co_return 0;
  execute_process_step(*proc);
  ++total_steps_;
  co_return 1;
}

task<std::unordered_map<int, value_t>> async_scheduler::run(std::optional<uint64_t> max_steps,
                                                            std::optional<double> max_time) {
  running_ = true;
  stop_requested_ = false;
  uint64_t steps = 0;
  auto start = std::chrono::steady_clock::now();

  while (!stop_requested_ && has_runnable()) {
    if (max_steps && steps >= *max_steps)
      break;
    if (max_time) {
      auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= *max_time)
        break;
    }
    co_await step();
    ++steps;
  }

  running_ = false;
  co_return get_results();
}

int async_scheduler::sync_step() { return step().sync_wait(); }

std::unordered_map<int, value_t> async_scheduler::sync_run(std::optional<uint64_t> max_steps,
                                                           std::optional<double> max_time) {
  return run(max_steps, max_time).sync_wait();
}

void async_scheduler::stop() { stop_requested_ = true; }

bool async_scheduler::has_runnable() const {
  for (const auto &[pid, proc] : processes_) {
    if (proc->status == process_status::ready || proc->status == process_status::running) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Cooperative sleep + inter-process messaging (parity with the sync scheduler;
// bodies mirror scheduler.cpp so the intrinsics behave identically on either).
// ---------------------------------------------------------------------------

void async_scheduler::wake_sleeping_processes() {
  auto now = std::chrono::steady_clock::now();
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::waiting && proc->wake_time && now >= *proc->wake_time) {
      proc->wake_time.reset();
      proc->status = process_status::ready;
    }
  }
}

bool async_scheduler::sleep(int pid, double seconds) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.wake_time = std::chrono::steady_clock::now() +
                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(seconds));
  proc.status = process_status::waiting;
  return true;
}

bool async_scheduler::receive_message(int pid, const std::string &path) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.recv_path = path;
  proc.status = process_status::waiting;
  return true;
}

int async_scheduler::deliver_to_receivers(const std::string &path, const value_t &msg) {
  int woken = 0;
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::waiting && proc->recv_path && *proc->recv_path == path) {
      proc->recv_path.reset();
      proc->state.result = msg;
      // Patch the nil placeholder that msg-recv left in the parent frame.
      if (!proc->state.stack.empty() && !proc->state.stack.back().results.empty())
        proc->state.stack.back().results.back() = msg;
      proc->status = process_status::ready;
      ++woken;
    }
  }
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
  // under the lock; delivery happens on the scheduler thread in drain_ingress().
  std::lock_guard<std::mutex> lock(ingress_mutex_);
  ingress_.emplace_back(path, msg);
}

void async_scheduler::drain_ingress() {
  std::vector<std::pair<std::string, value_t>> batch;
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    if (ingress_.empty())
      return;
    batch.swap(ingress_);
  }
  // On the scheduler thread now: deliver_to_receivers mutates process state, which is safe here
  // (no other thread touches the scheduler). A message with no waiter falls into the per-path
  // pending FIFO, exactly as an on-thread msg-send would.
  for (auto &[path, msg] : batch)
    deliver_to_receivers(path, msg);
}

void async_scheduler::queue_watch_event(int pid, process::watch_event evt) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  it->second->pending_watch_events.push_back(std::move(evt));
}

void async_scheduler::register_watch_handler(int pid, int watch_id, value_t handler,
                                             const std::string &path) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  std::string initial;
  if (watch_root_) {
    auto *node = watch_root_->findDescendant(path);
    if (node)
      initial = node->value();
  }
  it->second->watch_handlers[watch_id] = {std::move(handler), path, initial};
}

void async_scheduler::unregister_watch_handler(int pid, int watch_id) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  it->second->watch_handlers.erase(watch_id);
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
  if (!watch_root_)
    return;
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::terminated || proc->status == process_status::killed)
      continue;
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
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status == process_status::terminated || proc.status == process_status::killed)
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

void async_scheduler::check_limits(process &proc) {
  if (proc.max_steps > 0 && proc.step_count() >= proc.max_steps) {
    kill_process(proc, "max_steps_exceeded");
    return;
  }
  if (proc.max_time > 0.0 && proc.elapsed_time() >= proc.max_time) {
    kill_process(proc, "time_limit_exceeded");
    return;
  }
  if (proc.max_memory > 0 && mem_tracker_.current_bytes(proc.pid) >= proc.max_memory) {
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
  proc.status = process_status::terminated;
  proc.exit_code = std::move(result);
  proc.state.stats.mark_complete();
  if (proc.on_complete) {
    proc.on_complete(proc.exit_code);
  }
}

void async_scheduler::kill_process(process &proc, const std::string &reason) {
  proc.status = process_status::killed;
  proc.exit_error = reason;
  proc.state.stats.mark_complete();
}

bool async_scheduler::pause(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.status = process_status::paused;
  return true;
}

bool async_scheduler::resume(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::paused)
    return false;
  proc.status = process_status::ready;
  return true;
}

bool async_scheduler::kill(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status == process_status::terminated || proc.status == process_status::killed)
    return false;
  kill_process(proc, "killed_by_user");
  return true;
}

int async_scheduler::kill_owner(const std::string &owner) {
  int killed = 0;
  for (auto &[pid, proc] : processes_) {
    if (proc->owner == owner && proc->status != process_status::terminated &&
        proc->status != process_status::killed) {
      kill_process(*proc, "owner_reaped");
      ++killed;
    }
  }
  return killed;
}

int async_scheduler::fork(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return -1;
  auto &parent = *it->second;
  if (parent.status != process_status::ready && parent.status != process_status::running)
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
  auto child_env = std::make_shared<environment>();
  if (parent.state.global_env) {
    child_env->bindings = parent.state.global_env->bindings;
    child_env->outer = parent.state.global_env->outer;
  }
  child->state.global_env = child_env;
  child->state.stats.start();

  parent.state.result = value_t(static_cast<int64_t>(child_pid));
  child->state.result = value_t(static_cast<int64_t>(0));

  processes_.emplace(child_pid, std::move(child));
  return child_pid;
}

// ---------------------------------------------------------------------------
// Process control setters
// ---------------------------------------------------------------------------

bool async_scheduler::set_priority(int pid, int priority) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->priority = priority;
  return true;
}

bool async_scheduler::set_max_steps(int pid, uint64_t max_steps) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_steps = max_steps;
  return true;
}

bool async_scheduler::set_max_time(int pid, double seconds) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_time = seconds;
  return true;
}

bool async_scheduler::set_max_memory(int pid, uint64_t bytes) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_memory = bytes;
  return true;
}

bool async_scheduler::set_max_messages(int pid, uint64_t count) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_messages = count;
  return true;
}

bool async_scheduler::set_max_message_bytes(int pid, uint64_t bytes) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
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
  info.elapsed_time = proc.elapsed_time();
  info.current_memory = mem_tracker_.current_bytes(proc.pid);
  info.peak_memory = mem_tracker_.peak_bytes(proc.pid);
  info.max_memory = proc.max_memory;
  info.max_time = proc.max_time;
  info.message_count = proc.message_count;
  info.max_messages = proc.max_messages;
  info.message_bytes = proc.message_bytes;
  info.max_message_bytes = proc.max_message_bytes;
  info.parent_pid = proc.parent_pid;
  return info;
}

std::vector<process_info> async_scheduler::list_processes() const {
  std::vector<process_info> result;
  result.reserve(processes_.size());
  for (const auto &[pid, proc] : processes_) {
    result.push_back(make_info(*proc));
  }
  std::sort(result.begin(), result.end(),
            [](const process_info &a, const process_info &b) { return a.pid < b.pid; });
  return result;
}

std::optional<process_info> async_scheduler::get_process_info(int pid) const {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return std::nullopt;
  return make_info(*it->second);
}

std::optional<value_t> async_scheduler::get_result(int pid) const {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return std::nullopt;
  if (it->second->status != process_status::terminated)
    return std::nullopt;
  return it->second->exit_code;
}

std::unordered_map<int, value_t> async_scheduler::get_results() const {
  std::unordered_map<int, value_t> results;
  for (const auto &[pid, proc] : processes_) {
    if (proc->status == process_status::terminated) {
      results[pid] = proc->exit_code;
    }
  }
  return results;
}

scheduler_stats async_scheduler::get_stats() const {
  scheduler_stats s;
  s.total_processes = static_cast<int>(processes_.size());
  s.total_steps = total_steps_;
  for (const auto &[pid, proc] : processes_) {
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
    default:
      break;
    }
  }
  return s;
}

} // namespace cvc::state_exec

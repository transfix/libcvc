#include <algorithm>
#include <chrono>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/scheduler.h>
#include <stdexcept>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

scheduler::scheduler(scheduling_policy policy)
    : policy_(policy), evaluator_(builtins::make_default_environment()) {}

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

  // Publish effective values to per-scheduler subtree
  if (!id_.empty()) {
    std::string base = std::string(sched_prefix) + "." + id_;
    (*watch_root_)(base + ".max_pending_messages").value(std::to_string(max_pending_messages));
    (*watch_root_)(base + ".policy")
        .value(policy_ == scheduling_policy::round_robin ? "round_robin"
               : policy_ == scheduling_policy::priority  ? "priority"
                                                         : "priority_rr");
  }
}

// ---------------------------------------------------------------------------
// Process submission
// ---------------------------------------------------------------------------

int scheduler::execute(const std::string &script, const execute_options &opts) {
  int pid = next_pid_++;
  process proc;
  proc.pid = pid;
  proc.name = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
  proc.status = process_status::ready;
  proc.priority = opts.priority;
  proc.uid = opts.uid;
  proc.gid = opts.gid;
  proc.root_path = opts.root_path;
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

int scheduler::execute(const value_t &expr, const execute_options &opts) {
  int pid = next_pid_++;
  process proc;
  proc.pid = pid;
  proc.name = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
  proc.status = process_status::ready;
  proc.priority = opts.priority;
  proc.uid = opts.uid;
  proc.gid = opts.gid;
  proc.root_path = opts.root_path;
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
// Process selection
// ---------------------------------------------------------------------------

process_ptr scheduler::select_process() {
  std::vector<process *> runnable;
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::ready || proc->status == process_status::running) {
      runnable.push_back(proc.get());
    }
  }
  if (runnable.empty())
    return nullptr;

  // Sort by PID for deterministic ordering within policy
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
    // Lower priority value = higher priority (like Unix nice)
    std::sort(runnable.begin(), runnable.end(),
              [](const process *a, const process *b) { return a->priority < b->priority; });
    return processes_.at(runnable[0]->pid);
  }
  case scheduling_policy::priority_rr: {
    // Sort by priority, then round-robin within highest priority
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
// Step execution
// ---------------------------------------------------------------------------

void scheduler::execute_process_step(process &proc) {
  // Track which process is currently executing so intrinsics can query it.
  current_pid_ = proc.pid;
  current_proc_ = processes_[proc.pid];
  // Ensure current_pid_ is reset even on early returns.
  struct guard {
    int &pid;
    process_ptr &pp;
    ~guard() {
      pid = -1;
      pp.reset();
    }
  } reset_guard{current_pid_, current_proc_};

  // 1. Handle pending signals first
  if (!proc.pending_signals.empty() && !proc.in_signal_handler && !proc.in_watch_handler) {
    handle_signal(proc);
    return; // Signal setup counts as a step
  }

  // 1b. Handle pending watch events
  if (!proc.pending_watch_events.empty() && !proc.in_signal_handler && !proc.in_watch_handler) {
    handle_watch_event(proc);
    return;
  }

  // 2. Mark as running and track timing
  proc.status = process_status::running;
  proc.last_run_start = std::chrono::steady_clock::now();

  // 3. Execute one evaluation step
  bool done = evaluator_.step(proc.state);

  // 4. Accumulate running time
  auto now = std::chrono::steady_clock::now();
  proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();

  // 5. Check if signal/watch handler just finished
  if ((proc.in_signal_handler || proc.in_watch_handler) && proc.state.done) {
    restore_from_signal(proc);
    // Don't check done — we restored original evaluation
    proc.status = process_status::ready;
    check_limits(proc);
    return;
  }

  // 6. Check resource limits
  check_limits(proc);
  if (proc.status == process_status::killed)
    return;

  // 7. Check completion
  if (done) {
    terminate_process(proc, proc.state.result);
  } else if (proc.status == process_status::running) {
    // Only reset to ready if the intrinsic didn't change the status
    // (e.g. sleep or msg-recv set it to waiting).
    proc.status = process_status::ready;
  }
}

int scheduler::step() {
  // Wake sleeping processes whose deadline has passed.
  wake_sleeping_processes();

  // Check for value changes on watched state paths.
  poll_watches();

  auto proc = select_process();
  if (!proc)
    return 0;
  execute_process_step(*proc);
  ++total_steps_;
  return 1;
}

std::unordered_map<int, value_t> scheduler::run(std::optional<uint64_t> max_steps,
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
    step();
    ++steps;
  }

  running_ = false;
  return get_results();
}

void scheduler::stop() { stop_requested_ = true; }

bool scheduler::has_runnable() const {
  for (const auto &[pid, proc] : processes_) {
    if (proc->status == process_status::ready || proc->status == process_status::running)
      return true;
    // A sleeping process will become runnable when its deadline expires.
    if (proc->status == process_status::waiting && proc->wake_time)
      return true;
    // A message-waiting process will become runnable when a message arrives.
    if (proc->status == process_status::waiting && proc->recv_path)
      return true;
  }
  return false;
}

void scheduler::queue_watch_event(int pid, process::watch_event evt) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  it->second->pending_watch_events.push_back(std::move(evt));
}

void scheduler::register_watch_handler(int pid, int watch_id, value_t handler,
                                       const std::string &path) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  // Capture current value so the first change can be detected by polling.
  std::string initial;
  if (watch_root_) {
    auto *node = watch_root_->findDescendant(path);
    if (node)
      initial = node->value();
  }
  it->second->watch_handlers[watch_id] = {std::move(handler), path, initial};
}

void scheduler::unregister_watch_handler(int pid, int watch_id) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return;
  it->second->watch_handlers.erase(watch_id);
}

// ---------------------------------------------------------------------------
// Watch polling — replaces boost::signals2 for cross-platform reliability
// ---------------------------------------------------------------------------

void scheduler::poll_watches() {
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

// ---------------------------------------------------------------------------
// Sleep / wake
// ---------------------------------------------------------------------------

void scheduler::wake_sleeping_processes() {
  auto now = std::chrono::steady_clock::now();
  for (auto &[pid, proc] : processes_) {
    if (proc->status == process_status::waiting && proc->wake_time && now >= *proc->wake_time) {
      proc->wake_time.reset();
      proc->status = process_status::ready;
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
// Resource limit checking
// ---------------------------------------------------------------------------

void scheduler::check_limits(process &proc) {
  // max_steps
  if (proc.max_steps > 0 && proc.step_count() >= proc.max_steps) {
    kill_process(proc, "max_steps_exceeded");
    return;
  }
  // max_time
  if (proc.max_time > 0.0 && proc.elapsed_time() >= proc.max_time) {
    kill_process(proc, "time_limit_exceeded");
    return;
  }
  // max_memory
  if (proc.max_memory > 0 && mem_tracker_.current_bytes(proc.pid) >= proc.max_memory) {
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
  proc.status = process_status::terminated;
  proc.exit_code = std::move(result);
  proc.state.stats.mark_complete();
  if (proc.on_complete) {
    proc.on_complete(proc.exit_code);
  }
}

void scheduler::kill_process(process &proc, const std::string &reason) {
  proc.status = process_status::killed;
  proc.exit_error = reason;
  proc.state.stats.mark_complete();
}

bool scheduler::pause(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::ready && proc.status != process_status::running)
    return false;
  // If running, accumulate time up to now
  if (proc.status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time += std::chrono::duration<double>(now - proc.last_run_start).count();
  }
  proc.status = process_status::paused;
  return true;
}

bool scheduler::resume(int pid) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status != process_status::paused)
    return false;
  proc.status = process_status::ready;
  return true;
}

bool scheduler::sleep(int pid, double seconds) {
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

bool scheduler::receive_message(int pid, const std::string &path) {
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

int scheduler::deliver_to_receivers(const std::string &path, const value_t &msg) {
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
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  auto &proc = *it->second;
  if (proc.status == process_status::terminated || proc.status == process_status::killed)
    return false;
  kill_process(proc, "killed_by_user");
  return true;
}

int scheduler::fork(int pid) {
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

  // Deep copy evaluator state
  child->state.root_expr = parent.state.root_expr;
  child->state.result = parent.state.result;
  child->state.done = parent.state.done;
  child->state.stack = parent.state.stack; // Copy stack
  child->state.user_macros = parent.state.user_macros;
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

  processes_.emplace(child_pid, std::move(child));
  return child_pid;
}

// ---------------------------------------------------------------------------
// Process control setters
// ---------------------------------------------------------------------------

bool scheduler::set_priority(int pid, int priority) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->priority = priority;
  return true;
}

bool scheduler::set_max_steps(int pid, uint64_t max_steps) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_steps = max_steps;
  return true;
}

bool scheduler::set_max_time(int pid, double seconds) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_time = seconds;
  return true;
}

bool scheduler::set_max_memory(int pid, uint64_t bytes) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_memory = bytes;
  return true;
}

bool scheduler::set_max_messages(int pid, uint64_t count) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return false;
  it->second->max_messages = count;
  return true;
}

bool scheduler::set_max_message_bytes(int pid, uint64_t bytes) {
  auto it = processes_.find(pid);
  if (it == processes_.end())
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

std::vector<process_info> scheduler::list_processes() const {
  std::vector<process_info> result;
  result.reserve(processes_.size());
  for (const auto &[pid, proc] : processes_) {
    result.push_back(make_info(*proc));
  }
  // Sort by PID for deterministic ordering
  std::sort(result.begin(), result.end(),
            [](const process_info &a, const process_info &b) { return a.pid < b.pid; });
  return result;
}

std::optional<process_info> scheduler::get_process_info(int pid) const {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return std::nullopt;
  return make_info(*it->second);
}

std::optional<value_t> scheduler::get_result(int pid) const {
  auto it = processes_.find(pid);
  if (it == processes_.end())
    return std::nullopt;
  if (it->second->status != process_status::terminated)
    return std::nullopt;
  return it->second->exit_code;
}

std::unordered_map<int, value_t> scheduler::get_results() const {
  std::unordered_map<int, value_t> results;
  for (const auto &[pid, proc] : processes_) {
    if (proc->status == process_status::terminated) {
      results[pid] = proc->exit_code;
    }
  }
  return results;
}

scheduler_stats scheduler::get_stats() const {
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

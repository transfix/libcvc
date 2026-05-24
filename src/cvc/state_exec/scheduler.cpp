#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/builtins.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

scheduler::scheduler(scheduling_policy policy)
    : policy_(policy),
      evaluator_(builtins::make_default_environment()) {}

// ---------------------------------------------------------------------------
// Process submission
// ---------------------------------------------------------------------------

int scheduler::execute(const std::string& script, const execute_options& opts) {
    int pid = next_pid_++;
    process proc;
    proc.pid           = pid;
    proc.name          = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
    proc.status        = process_status::ready;
    proc.priority      = opts.priority;
    proc.uid           = opts.uid;
    proc.gid           = opts.gid;
    proc.max_steps     = opts.max_steps;
    proc.max_time      = opts.max_time;
    proc.max_memory    = opts.max_memory;
    proc.max_messages  = opts.max_messages;
    proc.signal_handlers = opts.signal_handlers;
    proc.on_complete   = opts.on_complete;
    proc.create_time   = std::chrono::steady_clock::now();
    proc.state         = evaluator_.create_state(script, opts.env);
    proc.state.stats.start();
    processes_.emplace(pid, std::move(proc));
    return pid;
}

int scheduler::execute(const value_t& expr, const execute_options& opts) {
    int pid = next_pid_++;
    process proc;
    proc.pid           = pid;
    proc.name          = opts.name.empty() ? ("proc-" + std::to_string(pid)) : opts.name;
    proc.status        = process_status::ready;
    proc.priority      = opts.priority;
    proc.uid           = opts.uid;
    proc.gid           = opts.gid;
    proc.max_steps     = opts.max_steps;
    proc.max_time      = opts.max_time;
    proc.max_memory    = opts.max_memory;
    proc.max_messages  = opts.max_messages;
    proc.signal_handlers = opts.signal_handlers;
    proc.on_complete   = opts.on_complete;
    proc.create_time   = std::chrono::steady_clock::now();
    proc.state         = evaluator_.create_state(expr, opts.env);
    proc.state.stats.start();
    processes_.emplace(pid, std::move(proc));
    return pid;
}

// ---------------------------------------------------------------------------
// Process selection
// ---------------------------------------------------------------------------

process* scheduler::select_process() {
    std::vector<process*> runnable;
    for (auto& [pid, proc] : processes_) {
        if (proc.status == process_status::ready ||
            proc.status == process_status::running) {
            runnable.push_back(&proc);
        }
    }
    if (runnable.empty()) return nullptr;

    // Sort by PID for deterministic ordering within policy
    std::sort(runnable.begin(), runnable.end(),
              [](const process* a, const process* b) {
                  return a->pid < b->pid;
              });

    switch (policy_) {
        case scheduling_policy::round_robin: {
            rr_index_ = rr_index_ % static_cast<int>(runnable.size());
            auto* p = runnable[rr_index_];
            rr_index_ = (rr_index_ + 1) % static_cast<int>(runnable.size());
            return p;
        }
        case scheduling_policy::priority: {
            // Lower priority value = higher priority (like Unix nice)
            std::sort(runnable.begin(), runnable.end(),
                      [](const process* a, const process* b) {
                          return a->priority < b->priority;
                      });
            return runnable[0];
        }
        case scheduling_policy::priority_rr: {
            // Sort by priority, then round-robin within highest priority
            std::sort(runnable.begin(), runnable.end(),
                      [](const process* a, const process* b) {
                          if (a->priority != b->priority)
                              return a->priority < b->priority;
                          return a->pid < b->pid;
                      });
            int best = runnable[0]->priority;
            std::vector<process*> same_prio;
            for (auto* p : runnable) {
                if (p->priority == best) same_prio.push_back(p);
            }
            rr_index_ = rr_index_ % static_cast<int>(same_prio.size());
            auto* p = same_prio[rr_index_];
            rr_index_ = (rr_index_ + 1) % static_cast<int>(same_prio.size());
            return p;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Step execution
// ---------------------------------------------------------------------------

void scheduler::execute_process_step(process& proc) {
    // 1. Handle pending signals first
    if (!proc.pending_signals.empty() && !proc.in_signal_handler) {
        handle_signal(proc);
        return;  // Signal setup counts as a step
    }

    // 2. Mark as running and track timing
    proc.status = process_status::running;
    proc.last_run_start = std::chrono::steady_clock::now();

    // 3. Execute one evaluation step
    bool done = evaluator_.step(proc.state);

    // 4. Accumulate running time
    auto now = std::chrono::steady_clock::now();
    proc.accumulated_time +=
        std::chrono::duration<double>(now - proc.last_run_start).count();

    // 5. Check if signal handler just finished
    if (proc.in_signal_handler && proc.state.done) {
        restore_from_signal(proc);
        // Don't check done — we restored original evaluation
        proc.status = process_status::ready;
        check_limits(proc);
        return;
    }

    // 6. Check resource limits
    check_limits(proc);
    if (proc.status == process_status::killed) return;

    // 7. Check completion
    if (done) {
        terminate_process(proc, proc.state.result);
    } else {
        proc.status = process_status::ready;
    }
}

int scheduler::step() {
    process* proc = select_process();
    if (!proc) return 0;
    execute_process_step(*proc);
    ++total_steps_;
    return 1;
}

std::unordered_map<int, value_t> scheduler::run(
    std::optional<uint64_t> max_steps,
    std::optional<double>   max_time)
{
    running_ = true;
    stop_requested_ = false;
    uint64_t steps = 0;
    auto start = std::chrono::steady_clock::now();

    while (!stop_requested_ && has_runnable()) {
        if (max_steps && steps >= *max_steps) break;
        if (max_time) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= *max_time) break;
        }
        step();
        ++steps;
    }

    running_ = false;
    return get_results();
}

void scheduler::stop() {
    stop_requested_ = true;
}

bool scheduler::has_runnable() const {
    for (const auto& [pid, proc] : processes_) {
        if (proc.status == process_status::ready ||
            proc.status == process_status::running) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

void scheduler::handle_signal(process& proc) {
    if (proc.pending_signals.empty()) return;

    std::string sig = proc.pending_signals.front();
    proc.pending_signals.erase(proc.pending_signals.begin());

    auto it = proc.signal_handlers.find(sig);
    if (it == proc.signal_handlers.end()) return;  // No handler — ignored

    // Save current evaluation state
    if (!proc.state.stack.empty()) {
        proc.saved_frame = proc.state.stack.back();
    }
    proc.in_signal_handler = true;

    // Set up handler evaluation
    auto handler_env = environment::extend(proc.state.global_env);
    handler_env->set("__signal__", value_t(sig));

    // Replace stack with handler invocation: (handler)
    proc.state.stack.clear();
    proc.state.stack.push_back({.expr = it->second, .env = handler_env});
    proc.state.done = false;
}

void scheduler::restore_from_signal(process& proc) {
    proc.in_signal_handler = false;
    if (proc.saved_frame) {
        proc.state.stack.clear();
        proc.state.stack.push_back(*proc.saved_frame);
        proc.state.done = false;
        proc.saved_frame.reset();
    } else {
        proc.state.done = true;
    }
}

bool scheduler::send_signal(int pid, const std::string& signal) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    auto& proc = it->second;
    if (proc.status == process_status::terminated ||
        proc.status == process_status::killed) return false;

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

void scheduler::check_limits(process& proc) {
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
    if (proc.max_memory > 0 &&
        mem_tracker_.current_bytes(proc.pid) >= proc.max_memory) {
        kill_process(proc, "memory_limit_exceeded");
        return;
    }
    // max_messages
    if (proc.max_messages > 0 && proc.message_count >= proc.max_messages) {
        kill_process(proc, "message_limit_exceeded");
        return;
    }
}

// ---------------------------------------------------------------------------
// Process lifecycle
// ---------------------------------------------------------------------------

void scheduler::terminate_process(process& proc, value_t result) {
    proc.status    = process_status::terminated;
    proc.exit_code = std::move(result);
    proc.state.stats.mark_complete();
    if (proc.on_complete) {
        proc.on_complete(proc.exit_code);
    }
}

void scheduler::kill_process(process& proc, const std::string& reason) {
    proc.status     = process_status::killed;
    proc.exit_error = reason;
    proc.state.stats.mark_complete();
}

bool scheduler::pause(int pid) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    auto& proc = it->second;
    if (proc.status != process_status::ready &&
        proc.status != process_status::running) return false;
    // If running, accumulate time up to now
    if (proc.status == process_status::running) {
        auto now = std::chrono::steady_clock::now();
        proc.accumulated_time +=
            std::chrono::duration<double>(now - proc.last_run_start).count();
    }
    proc.status = process_status::paused;
    return true;
}

bool scheduler::resume(int pid) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    auto& proc = it->second;
    if (proc.status != process_status::paused) return false;
    proc.status = process_status::ready;
    return true;
}

bool scheduler::kill(int pid) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    auto& proc = it->second;
    if (proc.status == process_status::terminated ||
        proc.status == process_status::killed) return false;
    kill_process(proc, "killed_by_user");
    return true;
}

int scheduler::fork(int pid) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return -1;
    auto& parent = it->second;
    if (parent.status != process_status::ready &&
        parent.status != process_status::running) return -1;

    int child_pid = next_pid_++;
    process child;
    child.pid             = child_pid;
    child.name            = parent.name + "-fork";
    child.status          = process_status::ready;
    child.priority        = parent.priority;
    child.uid             = parent.uid;
    child.gid             = parent.gid;
    child.max_steps       = parent.max_steps;
    child.max_time        = parent.max_time;
    child.max_memory      = parent.max_memory;
    child.max_messages    = parent.max_messages;
    child.signal_handlers = parent.signal_handlers;
    child.create_time     = std::chrono::steady_clock::now();
    child.parent_pid      = pid;

    // Deep copy evaluator state
    child.state.root_expr  = parent.state.root_expr;
    child.state.result     = parent.state.result;
    child.state.done       = parent.state.done;
    child.state.stack      = parent.state.stack;  // Copy stack
    child.state.user_macros = parent.state.user_macros;
    // Create fresh env as a copy of parent's bindings
    auto child_env = std::make_shared<environment>();
    if (parent.state.global_env) {
        child_env->bindings = parent.state.global_env->bindings;
        child_env->outer    = parent.state.global_env->outer;
    }
    child.state.global_env = child_env;
    child.state.stats.start();

    // Child result = 0, parent result = child_pid
    // (set on caller's evaluator result so next step picks it up)
    parent.state.result = value_t(static_cast<int64_t>(child_pid));
    child.state.result  = value_t(static_cast<int64_t>(0));

    processes_.emplace(child_pid, std::move(child));
    return child_pid;
}

// ---------------------------------------------------------------------------
// Process control setters
// ---------------------------------------------------------------------------

bool scheduler::set_priority(int pid, int priority) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    it->second.priority = priority;
    return true;
}

bool scheduler::set_max_steps(int pid, uint64_t max_steps) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    it->second.max_steps = max_steps;
    return true;
}

bool scheduler::set_max_time(int pid, double seconds) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    it->second.max_time = seconds;
    return true;
}

bool scheduler::set_max_memory(int pid, uint64_t bytes) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    it->second.max_memory = bytes;
    return true;
}

bool scheduler::set_max_messages(int pid, uint64_t count) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return false;
    it->second.max_messages = count;
    return true;
}

// ---------------------------------------------------------------------------
// Process info / query
// ---------------------------------------------------------------------------

process_info scheduler::make_info(const process& proc) const {
    process_info info;
    info.pid            = proc.pid;
    info.name           = proc.name;
    info.status         = proc.status;
    info.priority       = proc.priority;
    info.uid            = proc.uid;
    info.gid            = proc.gid;
    info.step_count     = proc.step_count();
    info.elapsed_time   = proc.elapsed_time();
    info.current_memory = mem_tracker_.current_bytes(proc.pid);
    info.peak_memory    = mem_tracker_.peak_bytes(proc.pid);
    info.max_memory     = proc.max_memory;
    info.max_time       = proc.max_time;
    info.message_count  = proc.message_count;
    info.max_messages   = proc.max_messages;
    info.parent_pid     = proc.parent_pid;
    return info;
}

std::vector<process_info> scheduler::list_processes() const {
    std::vector<process_info> result;
    result.reserve(processes_.size());
    for (const auto& [pid, proc] : processes_) {
        result.push_back(make_info(proc));
    }
    // Sort by PID for deterministic ordering
    std::sort(result.begin(), result.end(),
              [](const process_info& a, const process_info& b) {
                  return a.pid < b.pid;
              });
    return result;
}

std::optional<process_info> scheduler::get_process_info(int pid) const {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return std::nullopt;
    return make_info(it->second);
}

std::optional<value_t> scheduler::get_result(int pid) const {
    auto it = processes_.find(pid);
    if (it == processes_.end()) return std::nullopt;
    if (it->second.status != process_status::terminated) return std::nullopt;
    return it->second.exit_code;
}

std::unordered_map<int, value_t> scheduler::get_results() const {
    std::unordered_map<int, value_t> results;
    for (const auto& [pid, proc] : processes_) {
        if (proc.status == process_status::terminated) {
            results[pid] = proc.exit_code;
        }
    }
    return results;
}

scheduler_stats scheduler::get_stats() const {
    scheduler_stats s;
    s.total_processes = static_cast<int>(processes_.size());
    s.total_steps = total_steps_;
    for (const auto& [pid, proc] : processes_) {
        switch (proc.status) {
            case process_status::running:    ++s.running; break;
            case process_status::ready:      ++s.ready; break;
            case process_status::paused:     ++s.paused; break;
            case process_status::terminated: ++s.terminated; break;
            case process_status::killed:     ++s.killed; break;
            default: break;
        }
    }
    return s;
}

} // namespace cvc::state_exec

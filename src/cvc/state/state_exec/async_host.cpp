// Long-running native calls (docs/STATE_EXEC_EXCEPTIONS.md section 3): async_call, async_host and
// register_async_fn. See async_call.h and async_host.h.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h> // resolve_channel_key
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <stdexcept>
#include <utility>

namespace cvc::state_exec {

namespace {

using clock = async_call::clock;

int64_t steady_ns(clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

int64_t to_ns(double seconds) { return static_cast<int64_t>(std::llround(seconds * 1e9)); }

constexpr const char *kFutureKey = "__future__";
constexpr const char *kCallKey = "__async_call__";

// The `function` field of an async function's errors. Built only on the way to a throw: a call
// that succeeds makes no vector.
std::vector<std::pair<std::string, value_t>> function_fields(const std::string &name) {
  return {{"function", value_t(name)}};
}

// A call's reply channel: '#' makes it the identity under channel scoping, and exempt from the
// channel policy.
std::string reply_channel(const std::string &name, const char *kind) {
  static std::atomic<std::uint64_t> seq{0};
  return name + kind + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
}

// The handle a future holds, if it is an async call's: its second entry.
const value_t *future_handle(const value_t &v) {
  const dict_ptr *dp = std::get_if<dict_ptr>(&v.v);
  if (!dp || !*dp || (*dp)->size() != 2)
    return nullptr;
  const auto &entries = **dp;
  if (entries[0].first != kFutureKey || entries[1].first != kCallKey)
    return nullptr;
  return &entries[1].second;
}

} // namespace

// ---------------------------------------------------------------------------
// async_call
// ---------------------------------------------------------------------------

async_call::async_call(std::string name, int pid) : name_(std::move(name)), pid_(pid) {
  chunks_.call = this;
}

async_call::~async_call() = default;

bool async_call::cancelled() {
  if (std::this_thread::get_id() == kernel_thread_) {
    meter();
    if (inline_proc_ && !cancel_requested() && over_budget_inline())
      cancel(); // the scheduler kills the process once the step is over
  }
  if (cancel_requested())
    return true;
  if (timeout_ && past_timeout(clock::now())) {
    cancel();
    return true;
  }
  return false;
}

void async_call::post(const value_t &progress) {
  if (progress_channel_.empty())
    return; // no process to post to (a bare evaluator)
  if (async_sched_) {
    async_sched_->post_message(progress_channel_, progress);
    return;
  }
  // Inline: the kernel runs in its process's step, on the scheduler's thread, and the delivery is
  // the host's, not the kernel's.
  if (sched_ && std::this_thread::get_id() == kernel_thread_) {
    const detail::async_kernel_scope host(false);
    sched_->deliver_to_receivers(progress_channel_, progress);
  }
}

void async_call::add_wait(double seconds) {
  if (seconds > 0.0)
    wait_ns_.fetch_add(to_ns(seconds), std::memory_order_acq_rel);
}

bool async_call::wait(std::optional<double> seconds) {
  std::unique_lock<std::mutex> lk(m_);
  if (external_ && !settled_)
    // Its host settles it from the loop that drives the scheduler: blocking here would wait
    // forever.
    throw dsl_error("RuntimeError", name_ + ": can't wait for its result here", "async",
                    function_fields(name_));
  if (!seconds) {
    settled_cv_.wait(lk, [this] { return settled_; });
    return true;
  }
  return settled_cv_.wait_for(lk, std::chrono::duration<double>(std::max(0.0, *seconds)),
                              [this] { return settled_; });
}

value_t async_call::outcome() {
  // The rest of the call is charged before the program sees its outcome (3.8): a call whose
  // completion the scheduler hasn't taken yet is finished now.
  if (async_sched_ && !charged.closed)
    async_sched_->finish_async_call(shared_from_this());
  if (timed_out_) {
    std::string limit = "its timeout";
    if (timeout_) {
      char buf[32];
      std::snprintf(buf, sizeof buf, "%g", *timeout_);
      limit = std::string(buf) + " s";
    }
    throw dsl_error("TimeoutError", name_ + ": no result within " + limit, "async",
                    function_fields(name_));
  }
  if (!error_)
    return value_;
  if (!exception_) {
    // Mapped by the C++ type, as for a native function (exceptions.h), from where it came.
    value_t exc;
    try {
      std::rethrow_exception(error_);
    } catch (const std::exception &e) {
      exc = exception_from(e);
    } catch (...) {
      exc = make_exception(exception_class("NativeError"), name_ + ": unknown error", {}, "async");
    }
    set_field(exc, "origin", value_t(std::string("async")));
    set_field(exc, "function", value_t(name_));
    exception_ = std::move(exc);
  }
  throw dsl_exception(*exception_, error_);
}

bool async_call::past_timeout(clock::time_point now) const {
  return timeout_ && std::chrono::duration<double>(now - created_).count() >= *timeout_;
}

bool async_call::settle(value_t value, std::exception_ptr error) {
  {
    std::lock_guard<std::mutex> lk(m_);
    if (settled_)
      return false;
    settled_ = true;
    value_ = std::move(value);
    error_ = std::move(error);
  }
  finished_.store(true, std::memory_order_release);
  settled_cv_.notify_all();
  return true;
}

bool async_call::settle_timeout() {
  {
    std::lock_guard<std::mutex> lk(m_);
    if (settled_)
      return false;
    settled_ = true;
    timed_out_ = true;
  }
  finished_.store(true, std::memory_order_release);
  settled_cv_.notify_all();
  return true;
}

double async_call::run_seconds(clock::time_point now) const {
  const int64_t start = start_ns_.load(std::memory_order_acquire);
  if (start == 0)
    return 0.0;
  const int64_t end = end_ns_.load(std::memory_order_acquire);
  const int64_t ran =
      (end != 0 ? end : steady_ns(now)) - start - wait_ns_.load(std::memory_order_acquire);
  return ran > 0 ? 1e-9 * double(ran) : 0.0;
}

double async_call::cpu_seconds() const {
  return 1e-9 * double(cpu_ns_.load(std::memory_order_acquire) +
                       fanout_cpu_ns_.load(std::memory_order_acquire));
}

double async_call::unmetered_seconds(clock::time_point now) const {
  if (start_ns_.load(std::memory_order_acquire) == 0 || kernel_ended())
    return 0.0;
  const int64_t since = steady_ns(now) - metered_ns_.load(std::memory_order_acquire);
  return since > 0 ? 1e-9 * double(since) : 0.0;
}

void async_call::chunk_context::run_chunks(const std::function<void()> &chunks) noexcept {
  const double cpu0 = thread_cpu_seconds();
  {
    const detail::async_kernel_scope kernel(true);
    // The process's allocation limit holds in the kernel's chunks too (3.8).
    eval_alloc_guard guard(call->alloc_limit_);
    chunks();
    call->alloc_bytes_.fetch_add(guard.allocated(), std::memory_order_acq_rel);
  }
  call->fanout_cpu_ns_.fetch_add(to_ns(thread_cpu_seconds() - cpu0), std::memory_order_acq_rel);
}

void async_call::begin_kernel() {
  kernel_thread_ = std::this_thread::get_id();
  cpu0_ = thread_cpu_seconds();
  const int64_t now = steady_ns(clock::now());
  metered_ns_.store(now, std::memory_order_release);
  start_ns_.store(now, std::memory_order_release);
}

void async_call::end_kernel() {
  meter();
  end_ns_.store(steady_ns(clock::now()), std::memory_order_release);
}

void async_call::meter() {
  cpu_ns_.store(to_ns(thread_cpu_seconds() - cpu0_), std::memory_order_release);
  metered_ns_.store(steady_ns(clock::now()), std::memory_order_release);
}

// An inline call runs in its process's step, which the scheduler can't charge until it ends, so
// cancelled() checks what the call would be charged now against the process's budget.
bool async_call::over_budget_inline() {
  const process &p = *inline_proc_;
  const double run = run_seconds(clock::now());
  const double cpu = cpu_seconds();
  if (p.max_steps > 0) {
    // The step rate, read once per call and only for a step budget (it is a state-tree read
    // where the process doesn't set it). This runs on the stepping thread: the kernel's.
    if (!inline_rate_)
      inline_rate_ = sched_->async_steps_per_cpu_ms(p);
    if (p.step_count() + static_cast<uint64_t>(cpu * 1000.0 * *inline_rate_) >= p.max_steps)
      return true;
  }
  if (p.max_time > 0.0) {
    const double half = 0.5 * p.max_time;
    // Where the process was when the call began: its time so far (the step's slice, which
    // holds this call), less this call's run so far.
    const double before = p.elapsed_time() - run;
    const double past = std::min(run, std::max(0.0, before + run - half));
    const double cpu_part = run > 0.0 ? cpu * past / run : 0.0;
    if (before + run >= half && p.cpu_time_charged + cpu_part >= half)
      return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Charging (3.8)
// ---------------------------------------------------------------------------

void detail::charge_async_call(process &proc, async_call &call, double rate, bool final,
                               async_call::clock::time_point now, bool inline_step) {
  async_call::charges &ch = call.charged;
  if (ch.closed)
    return;
  const double run = call.run_seconds(now);
  const double cpu = call.cpu_seconds();
  // While the kernel runs, the time since it last recorded its CPU time counts as CPU time, so a
  // kernel that never checks cancelled() is stopped by its run time; its true CPU time replaces
  // that once it ends.
  const double est_cpu = final ? cpu : cpu + call.unmetered_seconds(now);

  // Run time: the first half of max_time (accumulated_time). The part of it spent while the
  // process was past the halfway point decides how much of the CPU time is the second half's.
  const double dr = run - ch.run;
  if (dr > 0.0) {
    if (proc.max_time > 0.0) {
      // An inline call's run time is inside the step's slice, which elapsed_time() counts and
      // the step adds to accumulated_time.
      const double before = proc.elapsed_time() - (inline_step ? run : 0.0);
      const double half = 0.5 * proc.max_time;
      ch.past_half += before >= half ? dr : std::max(0.0, before + dr - half);
    }
    if (inline_step)
      proc.step_async_wall += dr;
    else
      proc.accumulated_time += dr;
    proc.async_time += dr;
    ch.run = run;
  }

  // CPU time: charged in full (no per-step cap), as the share of the run past halfway.
  const double to_time = run > 0.0 ? est_cpu * std::min(1.0, ch.past_half / run) : 0.0;
  proc.cpu_time_charged += to_time - ch.cpu_to_time;
  ch.cpu_to_time = to_time;
  proc.async_cpu_time += cpu - ch.cpu_reported;
  ch.cpu_reported = cpu;

  // Steps: the CPU time recorded since the last charge, at the rate in force now.
  if (cpu > ch.cpu_stepped) {
    ch.step_fraction += (cpu - ch.cpu_stepped) * 1000.0 * std::max(0.0, rate);
    ch.cpu_stepped = cpu;
  }
  uint64_t steps = static_cast<uint64_t>(ch.step_fraction);
  ch.step_fraction -= static_cast<double>(steps);
  if (final && ch.steps + steps == 0)
    steps = 1; // every call counts at least one step
  if (steps > 0) {
    proc.state.stats.add_steps(step_kind::async, steps);
    proc.async_steps += steps;
    ch.steps += steps;
  }
  if (final)
    ch.closed = true;
}

// ---------------------------------------------------------------------------
// async_host
// ---------------------------------------------------------------------------

struct async_host::shared_state {
  mutable std::mutex m;
  std::condition_variable idle;
  std::size_t running = 0;                      // kernels queued or running
  std::atomic<bool> stopped{false};             // written under m; read without it by check_open
  std::vector<std::weak_ptr<async_call>> calls; // to cancel at shutdown
  std::size_t prune_at = 64; // prune expired entries when calls reaches this size

  void add(const std::shared_ptr<async_call> &call, bool runs) {
    std::lock_guard<std::mutex> lk(m);
    if (runs)
      ++running;
    // Expired entries go in batches, when the list has doubled since the last prune: amortized
    // O(1) per call instead of a scan of every live call on every call.
    if (calls.size() >= prune_at) {
      calls.erase(std::remove_if(calls.begin(), calls.end(),
                                 [](const std::weak_ptr<async_call> &w) { return w.expired(); }),
                  calls.end());
      prune_at = std::max<std::size_t>(64, 2 * calls.size());
    }
    calls.push_back(call);
  }
  void end() {
    {
      std::lock_guard<std::mutex> lk(m);
      --running;
    }
    idle.notify_all();
  }
};

async_host::async_host(scheduler_base *sched)
    : state_(std::make_shared<shared_state>()), sched_(sched) {}

async_host::async_host(async_scheduler &sched, cvc::app *app)
    : state_(std::make_shared<shared_state>()), sched_(&sched), async_sched_(&sched), app_(app) {}

async_host::~async_host() { shutdown(); }

void async_host::add_lane(const std::string &name, cvc::async_lane &lane) { lanes_[name] = &lane; }

void async_host::shutdown() {
  std::unique_lock<std::mutex> lk(state_->m);
  state_->stopped.store(true, std::memory_order_release);
  for (const std::weak_ptr<async_call> &w : state_->calls)
    if (std::shared_ptr<async_call> call = w.lock())
      call->cancel();
  state_->idle.wait(lk, [this] { return state_->running == 0; });
  state_->calls.clear();
}

void async_host::wait_idle() {
  std::unique_lock<std::mutex> lk(state_->m);
  state_->idle.wait(lk, [this] { return state_->running == 0; });
}

std::size_t async_host::in_flight() const {
  std::lock_guard<std::mutex> lk(state_->m);
  return state_->running;
}

void async_host::check_open(const std::string &name) const {
  if (state_->stopped.load(std::memory_order_acquire))
    throw dsl_error("RuntimeError", name + ": its host has shut down", "async",
                    function_fields(name));
}

bool async_host::runs_threaded(const async_options &opts) const {
  switch (opts.where.kind) {
  case executor_kind::background:
    return app_ != nullptr && cvc::async_lane_has_threads;
  case executor_kind::lane:
    return true;
  case executor_kind::inline_call:
    return false;
  }
  return false;
}

value_t async_host::call(const std::string &name, async_kernel kernel, const async_options &opts,
                         bool future) {
  if (parking_forbidden())
    throw dsl_error("RuntimeError",
                    name + ": an async function can't be called here (this context can't wait "
                           "for it)",
                    "async", function_fields(name));
  check_open(name);
  if (opts.where.kind == executor_kind::lane && !lanes_.count(opts.where.lane))
    throw dsl_error("RuntimeError", name + ": its host has no lane named '" + opts.where.lane + "'",
                    "async", function_fields(name));

  process *proc = sched_ && sched_->current_process() ? sched_->current_process().get() : nullptr;
  auto call = std::make_shared<async_call>(name, proc ? proc->pid : -1);
  call->timeout_ = opts.timeout;
  call->alloc_limit_ = eval_alloc_limit(); // the calling step's
  call->sched_ = sched_;
  if (proc)
    call->progress_channel_ = resolve_channel_key(proc->root_path, name + ".progress");

  // On an executor only where the process can be charged and resumed: a step of the parking
  // scheduler, and, for a transparent call, at a point that can park.
  const bool threaded = proc && async_sched_ && async_sched_->current_pid() == proc->pid &&
                        runs_threaded(opts) && (future || can_await());
  // The reply channel: what a parked process waits on, and a future's identity. A transparent
  // inline call has neither.
  if (threaded || future)
    call->channel_ = reply_channel(name, ".async#");
  if (!threaded)
    return run_inline(call, kernel, proc, future);

  call->async_sched_ = async_sched_;
  ++proc->async_calls;
  async_sched_->track_async_call(call);
  submit(call, std::move(kernel), opts);
  if (future)
    return make_async_future(call);
  if (!async_sched_->receive_message(proc->pid, call->channel_)) {
    call->cancel();
    throw dsl_error("RuntimeError", name + ": can't park process " + std::to_string(proc->pid),
                    "async", function_fields(name));
  }
  request_await({call->channel_, name, detail::make_awaitable_handle(call)});
  return nil_value;
}

value_t async_host::run_inline(const std::shared_ptr<async_call> &call, const async_kernel &kernel,
                               process *proc, bool future) {
  if (proc) {
    ++proc->async_calls;
    call->inline_proc_ = proc;
    // The step rate is read when it is needed: by cancelled() for a step budget, and for the
    // charge below (a change made meanwhile applies to that charge).
  }
  value_t value;
  std::exception_ptr error;
  call->begin_kernel();
  {
    // The kernel runs under the step's allocation guard (its reports count as the step's); its
    // fan-out chunks under the call's.
    cvc::fanout_context_scope fanout(&call->chunks_);
    const detail::async_kernel_scope in_kernel(true);
    try {
      value = kernel(*call);
    } catch (...) {
      error = std::current_exception();
    }
  }
  call->end_kernel();
  call->inline_proc_ = nullptr;
  const clock::time_point now = clock::now();
  if (call->past_timeout(now))
    call->settle_timeout();
  else
    call->settle(std::move(value), std::move(error));

  if (proc) {
    // Charged as a call, not as part of the step (3.8): the step's own charge leaves it out.
    proc->step_async_cpu += 1e-9 * double(call->cpu_ns_.load(std::memory_order_acquire));
    detail::charge_async_call(*proc, *call, sched_->async_steps_per_cpu_ms(*proc), true, now, true);
    if (proc->max_memory > 0) // the kernel's own reports went to the step's guard
      proc->heap_alloc_bytes += call->alloc_bytes();
    // A call that used up the budget: the scheduler kills the process when the step is over
    // (2.5), so its outcome, a value or an exception the program could catch, is dropped.
    if ((proc->max_steps > 0 && proc->step_count() >= proc->max_steps) || proc->over_time_limit())
      return nil_value;
  } else if (detail::data_step_scope *data = detail::data_step_scope::current()) {
    // A bare evaluator: its steps are all there is to charge.
    const double steps = call->cpu_seconds() * 1000.0 * kDefaultAsyncStepsPerCpuMs;
    data->stats().add_steps(step_kind::async, std::max<uint64_t>(1, static_cast<uint64_t>(steps)));
  }
  if (future)
    return make_async_future(call);
  return call->outcome();
}

void async_host::run_kernel(const std::shared_ptr<async_call> &call, async_kernel &kernel,
                            shared_state &state) noexcept {
  {
    value_t value;
    std::exception_ptr error;
    if (call->cancel_requested()) {
      error = std::make_exception_ptr(dsl_error(
          "RuntimeError", call->function_name() + ": cancelled before it started", "async"));
    } else {
      call->begin_kernel();
      {
        cvc::fanout_context_scope fanout(&call->chunks_);
        const detail::async_kernel_scope in_kernel(true);
        eval_alloc_guard guard(call->alloc_limit_);
        try {
          value = kernel(*call);
        } catch (...) {
          error = std::current_exception();
        }
        call->alloc_bytes_.fetch_add(guard.allocated(), std::memory_order_acq_rel);
      }
      call->end_kernel();
    }
    // The kernel and what it captured go here, on its thread, before the host's count goes down
    // (a host waiting for its calls to end may then free what they referenced).
    kernel = nullptr;
    // Past its timeout, the call raises TimeoutError whatever the kernel did once it saw
    // cancelled(), as when the scheduler times it out first.
    const bool settled = call->past_timeout(clock::now())
                             ? call->settle_timeout()
                             : call->settle(std::move(value), std::move(error));
    if (settled)
      call->async_sched_->post_async_completion(call);
  }
  state.end();
}

void async_host::submit(const std::shared_ptr<async_call> &call, async_kernel kernel,
                        const async_options &opts) {
  std::shared_ptr<shared_state> state = state_;
  state->add(call, true);
  // The executor runs, then destroys, this task; nothing else holds the kernel. (Moved and then
  // cleared: a moved-from std::function may keep a copy of a small callable.)
  std::function<void()> task = [call, kernel = std::move(kernel), state]() mutable {
    run_kernel(call, kernel, *state);
  };
  kernel = nullptr;
  try {
    if (opts.where.kind == executor_kind::lane) {
      if (!lanes_.at(opts.where.lane)->submit(std::move(task)))
        throw std::runtime_error(call->function_name() + ": lane '" + opts.where.lane +
                                 "' is stopped");
    } else {
      // An anonymous task on the background pool's persistent workers (no thread, no key).
      app_->submitPooled(std::move(task), opts.priority);
    }
  } catch (...) {
    // Not queued: the call fails now.
    if (call->settle({}, std::current_exception()))
      async_sched_->post_async_completion(call);
    state->end();
  }
}

std::shared_ptr<async_call> async_host::begin_external(const std::string &name, bool future,
                                                       value_t &result) {
  if (parking_forbidden())
    throw dsl_error("RuntimeError",
                    name + ": an async function can't be called here (this context can't wait "
                           "for it)",
                    "async", function_fields(name));
  check_open(name);
  process *proc = sched_ && sched_->current_process() ? sched_->current_process().get() : nullptr;
  if (!proc || !async_sched_ || async_sched_->current_pid() != proc->pid ||
      (!future && !can_await()))
    throw dsl_error("RuntimeError", name + ": can't wait for its result here", "async",
                    function_fields(name));
  auto call = std::make_shared<async_call>(name, proc->pid);
  call->channel_ = reply_channel(name, ".external#");
  call->sched_ = sched_;
  call->async_sched_ = async_sched_;
  call->external_ = true;
  call->progress_channel_ = resolve_channel_key(proc->root_path, name + ".progress");
  ++proc->async_calls;
  state_->add(call, false);
  async_sched_->track_async_call(call);
  if (future) {
    result = make_async_future(call);
  } else {
    if (!async_sched_->receive_message(proc->pid, call->channel_))
      throw dsl_error("RuntimeError", name + ": can't park process " + std::to_string(proc->pid),
                      "async", function_fields(name));
    request_await({call->channel_, name, detail::make_awaitable_handle(call)});
    result = nil_value;
  }
  return call;
}

void async_host::finish(const std::shared_ptr<async_call> &call) {
  if (call && call->async_sched_)
    call->async_sched_->post_async_completion(call);
}

// ---------------------------------------------------------------------------
// register_async_fn and futures
// ---------------------------------------------------------------------------

void register_async_fn(environment_ptr env, async_host &host, const std::string &name,
                       async_marshal marshal, async_options opts) {
  auto shared = std::make_shared<const async_marshal>(std::move(marshal));
  const auto make = [&host, name, shared, opts](bool future) -> native_fn {
    return [&host, name, shared, opts, future](std::span<const value_t> args) -> value_t {
      // With its arguments copied (3.7), marshal them and make the call.
      auto start = [&host, name, shared, opts, future](value_t copied) -> value_t {
        const std::vector<value_t> &argv = *std::get<list_ptr>(copied.v);
        async_kernel kernel = (*shared)(std::span<const value_t>(argv));
        if (!kernel)
          throw dsl_error("TypeError", name + ": its marshal step returned no kernel", "async",
                          {{"function", value_t(name)}});
        return host.call(name, std::move(kernel), opts, future);
      };
      value_t argv = make_list(std::vector<value_t>(args.begin(), args.end()));
      if (std::none_of(args.begin(), args.end(), [](const value_t &a) { return is_compound(a); }))
        return start(std::move(argv));
      // The program may change a list or dict it passed while the kernel runs: the kernel gets
      // copies, made as deep-data work, counted against the process's budget.
      return run_data_job(make_copy_job(std::move(argv)), std::move(start));
    };
  };
  builtins::register_fn(env, name, make(false));
  if (opts.future_variant)
    builtins::register_fn(env, name + "-async", make(true));
}

value_t make_async_future(const std::shared_ptr<async_call> &call) {
  return make_dict(
      {{kFutureKey, value_t(call->channel())}, {kCallKey, detail::make_awaitable_handle(call)}});
}

std::shared_ptr<detail::awaitable> async_call_of_future(const value_t &future) {
  const value_t *handle = future_handle(future);
  return handle ? detail::awaitable_of(*handle) : nullptr;
}

bool is_async_future(const value_t &v) { return future_handle(v) != nullptr; }

} // namespace cvc::state_exec

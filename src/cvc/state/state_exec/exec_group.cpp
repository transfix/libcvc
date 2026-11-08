// An execution group (exec_group.h; docs/STATE_EXEC_THREADED.md 4.1, 4.2, 4.7, 4.9; S1.md S1.5,
// and S1.8's host_proxied wrapping).

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/stdlib.h>
#include <exception>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <utility>

namespace cvc::state_exec {

namespace {

thread_local exec_group *t_current = nullptr;

std::atomic<std::uint64_t> g_group_seq{0};

// The groups alive per app (detail::live_groups).
std::mutex g_live_mutex;
std::unordered_map<const cvc::app *, std::size_t> &live_by_app() {
  static auto *m = new std::unordered_map<const cvc::app *, std::size_t>; // outlives every group
  return *m;
}

// Log a teardown or construction note through the group's app (interruption held off: this runs
// in a noexcept destructor), else stderr. Never throws.
void note(cvc::app *app, unsigned level, const std::string &text) noexcept {
  try {
    if (app) {
      boost::this_thread::disable_interruption hold;
      app->log(level, text);
    } else {
      std::cerr << text << '\n';
    }
  } catch (...) {
  }
}

std::string what_of(std::exception_ptr e) {
  try {
    std::rethrow_exception(e);
  } catch (const std::exception &x) {
    return x.what();
  } catch (...) {
    return "an exception of unknown type";
  }
}

// A copy of `v`'s lists and dicts, keeping their sharing and cycles; everything else shared (an
// AST's atoms, td B12). No native recursion: a deep AST costs heap, not stack.
value_t copy_lists_and_dicts(const value_t &v) {
  std::unordered_map<const void *, value_t> made;   // source container -> its copy
  std::vector<std::pair<value_t, value_t>> pending; // (source, copy) containers left to fill
  const auto copy_of = [&](const value_t &src) -> value_t {
    const void *key = nullptr;
    value_t out;
    if (const list_ptr *l = std::get_if<list_ptr>(&src.v); l && *l) {
      key = l->get();
      if (auto it = made.find(key); it != made.end())
        return it->second;
      out = make_list(std::vector<value_t>((*l)->size()));
    } else if (const dict_ptr *d = std::get_if<dict_ptr>(&src.v); d && *d) {
      key = d->get();
      if (auto it = made.find(key); it != made.end())
        return it->second;
      out = make_dict(std::vector<std::pair<std::string, value_t>>((*d)->size()));
    } else {
      return src;
    }
    made.emplace(key, out);
    pending.emplace_back(src, out);
    return out;
  };
  const value_t root = copy_of(v);
  while (!pending.empty()) {
    const auto [src, dst] = std::move(pending.back());
    pending.pop_back();
    if (const list_ptr *l = std::get_if<list_ptr>(&src.v)) {
      std::vector<value_t> &out = *std::get<list_ptr>(dst.v);
      for (std::size_t i = 0; i < (*l)->size(); ++i)
        out[i] = copy_of((**l)[i]);
    } else {
      const dict_ptr &in = std::get<dict_ptr>(src.v);
      auto &out = *std::get<dict_ptr>(dst.v);
      for (std::size_t i = 0; i < in->size(); ++i) {
        out[i].first = (*in)[i].first;
        out[i].second = copy_of((*in)[i].second);
      }
    }
  }
  return root;
}

} // namespace

std::size_t detail::live_groups(const cvc::app *app) noexcept {
  std::lock_guard<std::mutex> lock(g_live_mutex);
  const auto it = live_by_app().find(app);
  return it == live_by_app().end() ? 0 : it->second;
}

struct exec_group::impl {
  // Declared first, so it goes last: collected and freed after the host, the scheduler and its
  // environments are gone (S1.5).
  cycle_registry_ptr registry = make_cycle_registry();
  random_stream random;
  options opts;
  std::uint64_t seq = 0;
  std::unique_ptr<async_scheduler> sched;
  std::once_flag host_once;
  std::unique_ptr<async_host> host; // declared after the scheduler it posts to, so it goes first
  std::atomic<bool> failed{false};
  detail::group_pool_link link;
  // An inline on_complete threw in this pump (as today, it ends the pump): pump() rethrows it
  // rather than failing the group.
  bool callback_threw = false;
};

exec_group::exec_group(options o) : impl_(std::make_unique<impl>()) {
  impl &m = *impl_;
  m.opts = std::move(o);
  m.seq = g_group_seq.fetch_add(1, std::memory_order_relaxed) + 1;
  m.link.seq = m.seq;
  m.link.random = &m.random;
  cvc::app *const app = m.opts.app;
  cvc::state *const tree = app ? &app->root() : m.opts.watch_root;
  // As app::exec_scheduler() builds the app's (S1.5): the policy, fixed now (the home takes it at
  // attach), from state_exec.policy.
  bool unknown = false;
  const scheduling_policy policy =
      tree ? policy_from_state(*tree, scheduling_policy::round_robin, &unknown)
           : scheduling_policy::round_robin;
  if (unknown)
    note(app, 1,
         "exec_group '" + m.opts.name + "': state_exec.policy: unknown value; using " +
             "round_robin");
  // Its global env extends the base: the host's, else the one frozen builtins scope (td 4.2,
  // S1.3), so a group builds no default environment of its own.
  environment_ptr base = m.opts.base ? m.opts.base : frozen_builtins();
  m.sched = std::make_unique<async_scheduler>(policy, std::move(base));
  if (cvc::state *root = m.opts.watch_root ? m.opts.watch_root : tree)
    m.sched->set_watch_root(root);
  m.sched->set_parser_defaults(async_scheduler::host_parser_defaults());
  m.sched->set_data_defaults(async_scheduler::host_data_defaults());
  // Its own registry and math.random stream, which its scheduler binds (S1.1, S1.2); a fixed seed
  // when given (a deterministic pool seeds an unseeded stream from the name, S1.6).
  if (m.opts.seed)
    m.random.seed = *m.opts.seed;
  m.sched->set_cycle_registry(m.registry.get());
  m.sched->set_random_stream(&m.random);
  if (app) {
    // The app's lock manager, in its own mode, as the app scheduler attaches (S1-Q1): with-lock
    // here excludes the same keys app-wide.
    const std::shared_ptr<lock_manager> locks = app->lock_manager();
    const bool shared = locks->threading() == lock_threading::shared;
    m.sched->set_lock_manager(locks, lock_attach{shared});
  }
  std::lock_guard<std::mutex> lock(g_live_mutex);
  ++live_by_app()[app];
}

exec_group::~exec_group() {
  impl &m = *impl_;
  cvc::app *const app = m.opts.app;
  // Not from a host_proxied body or an on_complete callback, nor while a pump_all holds it (S1.6).
  assert(!cvc::detail::in_pool_callback() && "an exec_group destroyed inside a pump_all");
  assert(m.link.call == nullptr && "an exec_group destroyed while a pump_all holds it");
  const auto step = [&](const char *what, auto &&fn) noexcept {
    try {
      fn();
    } catch (...) {
      note(app, 1,
           "exec_group '" + m.opts.name + "' teardown: " + what + ": " +
               what_of(std::current_exception()));
    }
  };
  // The token, waiting for a pump in progress (no pool holds it outside a pump_all).
  std::optional<async_scheduler::pump_owner_scope> token;
  for (;;) {
    try {
      token.emplace(*m.sched);
      break;
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
  // Then, in td 4.2's order: no lock post lands from here on; the test hook (GL-2); the kills,
  // which settle the processes' calls as cancelled; the host, waiting for its kernels; the
  // host-call queue's entries, dropped unrun; the reap.
  step("closing the lock port", [&] { m.sched->close_lock_port(); });
  if (m.opts.teardown_hook_for_test)
    step("the teardown hook", [&] { m.opts.teardown_hook_for_test(*this); });
  step("listing its processes", [&] {
    for (const process_info &p : m.sched->list_processes())
      if (!is_finished(p.status))
        step("a kill", [&] { m.sched->kill(p.pid); });
  });
  if (m.host)
    step("shutting its async host down", [&] { m.host->shutdown(); });
  cvc::detail::purge_host_calls(*this);
  step("listing its processes", [&] {
    for (const process_info &p : m.sched->list_processes())
      step("a reap", [&] { m.sched->reap(p.pid); });
  });
  token.reset();
  {
    std::lock_guard<std::mutex> lock(g_live_mutex);
    auto it = live_by_app().find(app);
    if (it != live_by_app().end() && --it->second == 0)
      live_by_app().erase(it);
  }
  // The members go now: the host, the scheduler and its envs, and last the registry, whose
  // deleter collects it under its own binding.
}

async_scheduler &exec_group::sched() { return *impl_->sched; }

async_host &exec_group::host() {
  std::call_once(impl_->host_once, [this] {
    impl_->host = std::make_unique<async_host>(*impl_->sched, impl_->opts.app);
  });
  return *impl_->host;
}

pump_result exec_group::pump(const pump_budget &budget) {
  impl &m = *impl_;
  // The token: the scheduler's pump owner, from unset (S1.4). Held by another thread here is a
  // bug of the pool or the host (two pumps at once): asserted, and refused with the S1-Q4 text.
  std::optional<async_scheduler::pump_owner_scope> token;
  try {
    token.emplace(*m.sched);
  } catch (...) {
    assert(false && "exec_group::pump: another thread holds the group's token");
    throw;
  }
  struct current_scope {
    exec_group *outer;
    explicit current_scope(exec_group *g) : outer(t_current) { t_current = g; }
    ~current_scope() { t_current = outer; }
  } bind(this);
  pump_result r;
  m.callback_threw = false;
  try {
    if (m.opts.pump_hook_for_test)
      m.opts.pump_hook_for_test(*this);
    async_scheduler &s = *m.sched;
    s.drain_ingress();
    r.woken = s.wake_awaiting();
    const std::optional<double> seconds = std::isfinite(budget.seconds)
                                              ? std::optional<double>(std::max(0.0, budget.seconds))
                                              : std::nullopt;
    r.steps = s.sync_run_steps(budget.steps, seconds);
  } catch (...) {
    if (m.callback_threw) {
      // An on_complete callback run inline threw: the host's exception, as today.
      m.callback_threw = false;
      throw;
    }
    m.failed.store(true, std::memory_order_release);
    m.link.failure = what_of(std::current_exception());
    r.failed = true;
  }
  return r;
}

bool exec_group::wants_pump() const {
  async_scheduler &s = *impl_->sched;
  // A runnable process, or (wait_for_work's checks, with a deadline already passed) pending
  // ingress, lock posts, a timer due or a call to charge.
  return s.has_runnable() || s.wait_for_work(std::chrono::steady_clock::time_point::min());
}

namespace {

// on_complete for a process of `m`'s group (S1.6, td 4.9): queued for the pump_all's caller when a
// worker pumps the group, else run inline, as today.
std::function<void(value_t)> group_on_complete(bool *threw, std::function<void(value_t)> cb) {
  return [threw, cb = std::move(cb)](value_t result) {
    if (cvc::detail::defer_on_complete([cb, result] { cb(result); }))
      return;
    try {
      cb(std::move(result));
    } catch (...) {
      *threw = true;
      throw;
    }
  };
}

} // namespace

int exec_group::execute(const std::string &script, const execute_options &opts) {
  if (!opts.on_complete)
    return impl_->sched->execute(script, opts);
  execute_options o = opts;
  o.on_complete = group_on_complete(&impl_->callback_threw, std::move(o.on_complete));
  return impl_->sched->execute(script, o);
}

int exec_group::execute(const value_t &expr, const execute_options &opts) {
  impl &m = *impl_;
  // td B12: a quoted literal is the AST's own node, so the group gets its own copy of every list
  // and dict. The caller's reference is always there (use_count() is never below 1 here), and a
  // nested literal another group's process kept outlives the top's sharing (AS-1): copying always
  // is what keeps two groups, and the host's AST, from sharing one.
  const value_t given = copy_lists_and_dicts(expr);
  if (!opts.on_complete)
    return m.sched->execute(given, opts);
  execute_options o = opts;
  o.on_complete = group_on_complete(&m.callback_threw, std::move(o.on_complete));
  return m.sched->execute(given, o);
}

void exec_group::install(const environment_ptr &env, const std::string &name, native_fn fn,
                         provider_affinity kind) {
  // A host_only native is not installed in an `any` group: a call is a NameError, never a race
  // (td 4.9). The host_proxied wrapping comes with the host-call queue (S1.8).
  if (impl_->opts.affinity == group_affinity::any && kind == provider_affinity::host_only)
    return;
  builtins::register_fn(env, name, std::move(fn));
}

bool exec_group::failed() const { return impl_->failed.load(std::memory_order_acquire); }

cycle_registry *exec_group::registry() const { return impl_->registry.get(); }

exec_group *exec_group::current() { return t_current; }

const exec_group::options &exec_group::opts() const { return impl_->opts; }

detail::group_pool_link &exec_group::pool_link() noexcept { return impl_->link; }

} // namespace cvc::state_exec

// An execution group (exec_group.h; docs/STATE_EXEC_THREADED.md 4.1, 4.2, 4.7, 4.9; S1.md S1.5,
// and S1.8's host_proxied wrapping).

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
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

// What the groups built with an app keep on it: how many were made (each one's seq, its creation
// order in the app), how many are alive (detail::live_groups), and whether the note that the app's
// lock manager is single was logged (S1-Q1, once per app).
struct app_groups {
  static constexpr const char *app_slot = "cvc.state_exec.exec_groups";
  std::atomic<std::uint64_t> made{0};
  std::atomic<std::size_t> live{0};
  std::atomic<bool> single_noted{false};
};

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

std::size_t detail::live_groups(cvc::app &app) noexcept {
  return app.slot<app_groups>().live.load(std::memory_order_acquire);
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
  app_groups *of_app = nullptr; // its app's counts (null: no app)
  // An inline on_complete threw in this pump (as today, it ends the pump): pump() rethrows it
  // rather than failing the group.
  bool callback_threw = false;
};

exec_group::exec_group(options o) : impl_(std::make_unique<impl>()) {
  impl &m = *impl_;
  m.opts = std::move(o);
  cvc::app *const app = m.opts.app;
  app_groups *const of_app = app ? &app->slot<app_groups>() : nullptr;
  // Its creation order among the app's groups; a group with no app has none (0), and a pool keeps
  // such groups in the order it is given them.
  m.of_app = of_app;
  m.seq = of_app ? of_app->made.fetch_add(1, std::memory_order_relaxed) + 1 : 0;
  m.link.seq = m.seq;
  m.link.random = &m.random;
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
  // Its global env extends the base: the host's, else the app's frozen builtins scope (td 4.2,
  // S1.3), so a group of an app builds no default environment of its own. A group with no app
  // builds one, frozen as the app's is.
  environment_ptr base = m.opts.base;
  if (!base && app) {
    base = frozen_builtins(*app);
  } else if (!base) {
    base = builtins::make_default_environment();
    base->frozen = true;
  }
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
  // Stage S: the group's runs, each setting it gives; the one it doesn't give stays the tree's
  // (else CVC_EXEC_SLICE_STEPS and 1 ms).
  if (m.opts.slice_steps)
    m.sched->set_slice_steps(*m.opts.slice_steps);
  if (m.opts.slice_seconds)
    m.sched->set_slice_seconds(*m.opts.slice_seconds);
  if (app) {
    // Pids no other group or the app scheduler repeats (S1.4).
    m.sched->set_pid_source(app->exec_pid_source());
    // The app's lock manager, in its own mode, as the app scheduler attaches (S1-Q1): with-lock
    // here excludes the same keys app-wide.
    const bool pool_built = app->exec_pool_probe() != nullptr;
    const std::shared_ptr<lock_manager> locks = app->lock_manager();
    const bool shared = locks->threading() == lock_threading::shared;
    m.sched->set_lock_manager(locks, lock_attach{shared});
    // A single manager's tables have one thread: every pool pumps this group on its caller only.
    m.link.caller_only = !shared;
    if (!shared && !pool_built) {
      const bool first = !of_app->single_noted.exchange(true, std::memory_order_relaxed);
      if (first)
        note(app, 2,
             "exec_group '" + m.opts.name +
                 "': this app's lock manager is single, made before any " +
                 "exec_pool asked for workers: its groups are pumped on the caller of pump_all " +
                 "(S1-Q1; give app::exec_pool(opts) first, or set state_exec.threads)");
    }
  }
  if (of_app)
    of_app->live.fetch_add(1, std::memory_order_release);
}

exec_group::~exec_group() {
  impl &m = *impl_;
  cvc::app *const app = m.opts.app;
  // Never from inside its own pump (an inline native or callback of it): its token would nest,
  // and the teardown would free the scheduler under its run loop. That aborts, in every build.
  if (t_current == this) {
    std::fputs("exec_group: destroyed from inside its own pump\n", stderr);
    std::fflush(stderr);
    std::abort();
  }
  // Nor while a pump_all holds it (a host body or callback of that call, or another thread): the
  // call would go on using it after the teardown (N3). That aborts, in every build, too.
  if (m.link.call.load(std::memory_order_relaxed) != nullptr) {
    std::fputs("exec_group: destroyed while a pump_all holds it\n", stderr);
    std::fflush(stderr);
    std::abort();
  }
  // Not from a host_proxied body or an on_complete callback at all (S1.6; a debug assertion).
  assert(!cvc::detail::in_pool_callback() && "an exec_group destroyed inside a pump_all");
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
  if (m.of_app) // alive: a group never outlives its app (~app asserts it)
    m.of_app->live.fetch_sub(1, std::memory_order_release);
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
  // A runnable process, a process waiting for the next frame (the pump's wake_awaiting readies
  // it), or (wait_for_work's checks, with a deadline already passed) pending ingress, lock posts,
  // a timer due or a call to charge.
  return s.has_runnable() || s.awaiting_frame_count() > 0 ||
         s.wait_for_work(std::chrono::steady_clock::time_point::min());
}

namespace {

// on_complete for a process of `m`'s group (S1.6, td 4.9): queued for the pump_all's caller when a
// threaded pump_all pumps the group (on a worker or on its caller), else run inline, as today.
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
  impl &m = *impl_;
  if (!opts.on_complete && (!opts.owner.empty() || m.opts.owner.empty()))
    return m.sched->execute(script, opts);
  execute_options o = opts;
  if (o.owner.empty())
    o.owner = m.opts.owner; // the group's owner by default (options::owner)
  if (o.on_complete)
    o.on_complete = group_on_complete(&m.callback_threw, std::move(o.on_complete));
  return m.sched->execute(script, o);
}

int exec_group::execute(const value_t &expr, const execute_options &opts) {
  impl &m = *impl_;
  // td B12: a quoted literal is the AST's own node, so the group gets its own copy of every list
  // and dict. The caller's reference is always there (use_count() is never below 1 here), and a
  // nested literal another group's process kept outlives the top's sharing (AS-1): copying always
  // is what keeps two groups, and the host's AST, from sharing one.
  const value_t given = copy_lists_and_dicts(expr);
  if (!opts.on_complete && (!opts.owner.empty() || m.opts.owner.empty()))
    return m.sched->execute(given, opts);
  execute_options o = opts;
  if (o.owner.empty())
    o.owner = m.opts.owner;
  if (o.on_complete)
    o.on_complete = group_on_complete(&m.callback_threw, std::move(o.on_complete));
  return m.sched->execute(given, o);
}

namespace {

// A host_proxied native's body, run on the host thread (S1.8, td 4.9): `fn` on private copies of
// its arguments.
struct proxied_body {
  native_fn fn;
  std::vector<value_t> args;
  value_t run(std::exception_ptr &error) const {
    try {
      return fn(std::span<const value_t>(args));
    } catch (...) {
      error = std::current_exception();
      return nil_value();
    }
  }
};

// Where the calling thread is a worker of `pool`: park the calling process on the body, queued
// for the pump_all's caller, and resume it with the body's value at the group's next pump; at a
// point that cannot park (a generator body), block this worker until the caller has run it.
value_t proxy_on_worker(cvc::exec_pool &pool, exec_group &group, const std::string &name,
                        proxied_body body) {
  if (can_await()) {
    value_t result;
    async_host &host = group.host();
    const std::shared_ptr<async_call> call = host.begin_external(name, false, result);
    pool.run_on_host([&host, call, body = std::move(body)] {
      std::exception_ptr error;
      value_t value = body.run(error);
      call->settle(std::move(value), std::move(error));
      host.finish(call);
    });
    return result;
  }
  struct rendezvous {
    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    value_t value;
    std::exception_ptr error;
  };
  auto meet = std::make_shared<rendezvous>();
  cvc::detail::run_on_host_waiting(pool, [meet, body = std::move(body)] {
    std::exception_ptr error;
    value_t value = body.run(error);
    std::lock_guard<std::mutex> lock(meet->m);
    meet->value = std::move(value);
    meet->error = std::move(error);
    meet->done = true;
    meet->cv.notify_all();
  });
  std::unique_lock<std::mutex> lock(meet->m);
  meet->cv.wait(lock, [&] { return meet->done; });
  if (meet->error)
    std::rethrow_exception(meet->error);
  return std::move(meet->value);
}

} // namespace

void exec_group::install(const environment_ptr &env, const std::string &name, native_fn fn,
                         provider_affinity kind) {
  // In a host-affinity group every kind is installed as it is (td 4.9).
  if (impl_->opts.affinity == group_affinity::host || kind == provider_affinity::any) {
    builtins::register_fn(env, name, std::move(fn));
    return;
  }
  if (kind == provider_affinity::host_only)
    return; // not installed: a call is a NameError, never a race
  // host_proxied: inline where the caller of the pump_all (or no pump_all) runs the step, as
  // today, at threads 0 included; through the host-call queue on a worker.
  auto shared_fn = std::make_shared<const native_fn>(std::move(fn));
  exec_group *const group = this;
  builtins::register_fn(
      env, name, [group, name, shared_fn](std::span<const value_t> args) -> value_t {
        cvc::exec_pool *pool = cvc::detail::worker_pool();
        if (!pool)
          return (*shared_fn)(args);
        // The body reads its arguments on the host thread while this worker steps the group's
        // other processes: it gets copies, made as deep-data work, as an async function's do.
        auto start = [group, name, shared_fn](value_t copied) -> value_t {
          const std::vector<value_t> &argv = *std::get<list_ptr>(copied.v);
          cvc::exec_pool *pool = cvc::detail::worker_pool(); // the copy may end on the caller
          if (!pool)
            return (*shared_fn)(std::span<const value_t>(argv));
          return proxy_on_worker(*pool, *group, name, proxied_body{*shared_fn, argv});
        };
        value_t argv = make_list(std::vector<value_t>(args.begin(), args.end()));
        if (std::none_of(args.begin(), args.end(), [](const value_t &a) { return is_compound(a); }))
          return start(std::move(argv));
        return run_data_job(make_copy_job(std::move(argv), copy_mode::share, "host_proxied"),
                            std::move(start));
      });
}

bool exec_group::failed() const { return impl_->failed.load(std::memory_order_acquire); }

cycle_registry *exec_group::registry() const { return impl_->registry.get(); }

exec_group *exec_group::current() { return t_current; }

const exec_group::options &exec_group::opts() const { return impl_->opts; }

detail::group_pool_link &exec_group::pool_link() noexcept { return impl_->link; }

} // namespace cvc::state_exec

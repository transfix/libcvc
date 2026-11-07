#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <mutex>
#include <utility>

namespace cvc::state_exec {

// Stage 1 interfaces: a stub group. It builds a plain scheduler (with the base env when one is
// given) and pumps it in pump()'s order, binding current(); it has no registry, random stream,
// token, pid source, lock attach or teardown yet. S1.5 is the group.

namespace {
thread_local exec_group *t_current = nullptr;
} // namespace

struct exec_group::impl {
  explicit impl(options o)
      : opts(std::move(o)),
        sched(opts.base
                  ? std::make_unique<async_scheduler>(scheduling_policy::round_robin, opts.base)
                  : std::make_unique<async_scheduler>(scheduling_policy::round_robin)) {}

  options opts;
  std::unique_ptr<async_scheduler> sched;
  std::once_flag host_once;
  std::unique_ptr<async_host> host; // declared after the scheduler it posts to, so it goes first
  bool failed = false;
};

exec_group::exec_group(options opts) : impl_(std::make_unique<impl>(std::move(opts))) {
  if (impl_->opts.watch_root)
    impl_->sched->set_watch_root(impl_->opts.watch_root);
}

exec_group::~exec_group() = default;

async_scheduler &exec_group::sched() { return *impl_->sched; }

async_host &exec_group::host() {
  std::call_once(impl_->host_once, [this] {
    impl_->host = std::make_unique<async_host>(*impl_->sched, impl_->opts.app);
  });
  return *impl_->host;
}

pump_result exec_group::pump(const pump_budget &budget) {
  struct current_scope {
    exec_group *outer;
    explicit current_scope(exec_group *g) : outer(t_current) { t_current = g; }
    ~current_scope() { t_current = outer; }
  } bind(this);
  if (impl_->opts.pump_hook_for_test)
    impl_->opts.pump_hook_for_test(*this);
  pump_result r;
  async_scheduler &s = *impl_->sched;
  s.drain_ingress();
  r.woken = s.wake_awaiting();
  r.steps = s.sync_run_steps(budget.steps, budget.seconds);
  return r;
}

bool exec_group::wants_pump() const {
  return true; // the stub always pumps; S1.5 asks the scheduler
}

int exec_group::execute(const std::string &script, const execute_options &opts) {
  return impl_->sched->execute(script, opts);
}

int exec_group::execute(const value_t &expr, const execute_options &opts) {
  return impl_->sched->execute(expr, opts);
}

// S1.0 stub: binds every kind as it is (a host-affinity group's rule); P's S1.8 wrapping does the
// rest.
void exec_group::install(const environment_ptr &env, const std::string &name, native_fn fn,
                         provider_affinity) {
  builtins::register_fn(env, name, std::move(fn));
}

bool exec_group::failed() const { return impl_->failed; }

cycle_registry *exec_group::registry() const { return nullptr; }

exec_group *exec_group::current() { return t_current; }

const exec_group::options &exec_group::opts() const { return impl_->opts; }

} // namespace cvc::state_exec

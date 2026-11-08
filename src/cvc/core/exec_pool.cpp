#include <atomic>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state_exec/exec_group.h>
#include <mutex>

namespace cvc {

// Stage 1 interfaces: a stub pool with no workers. pump_all pumps every group on the caller, in the
// order given, as at 0 threads; the deadline, the failure handling, the helper wake and the
// host-call queue are S1.6's.

namespace {
std::atomic<std::uint64_t> g_worker_pumps{0};
// The pool whose pump_all the calling thread is in, or null.
thread_local const exec_pool *t_pump_all = nullptr;
} // namespace

struct exec_pool::impl {
  pool_options opts;
  mutable std::mutex m;
  pool_stats stats; // guarded by m
};

exec_pool::exec_pool(const pool_options &opts) : impl_(std::make_unique<impl>()) {
  impl_->opts = opts;
}

exec_pool::~exec_pool() = default;

void exec_pool::pump_all(std::span<state_exec::exec_group *const> groups,
                         const state_exec::pump_budget &budget,
                         std::chrono::steady_clock::time_point) {
  struct in_pump_all_scope {
    const exec_pool *outer;
    explicit in_pump_all_scope(const exec_pool *p) : outer(t_pump_all) { t_pump_all = p; }
    ~in_pump_all_scope() { t_pump_all = outer; }
  } bind(this);
  for (state_exec::exec_group *g : groups) {
    if (!g)
      continue;
    g->pump(budget);
    std::lock_guard<std::mutex> lock(impl_->m);
    ++impl_->stats.pumps;
  }
}

unsigned exec_pool::threads() const { return 0; }

pool_stats exec_pool::stats() const {
  std::lock_guard<std::mutex> lock(impl_->m);
  return impl_->stats;
}

std::vector<state_exec::exec_group *> exec_pool::failed_groups() const { return {}; }

void exec_pool::run_on_host(std::function<void()> fn) {
  if (fn)
    fn(); // nothing runs on a worker yet: every caller is the host
}

bool exec_pool::in_pump_all() const { return t_pump_all == this; }

void exec_pool::set_last_pump_for_test(state_exec::exec_group &, double) {}

std::uint64_t exec_pool_worker_pumps_total() {
  return g_worker_pumps.load(std::memory_order_relaxed);
}

namespace detail {

// The stub pool has no worker: nothing is queued, deferred or purged.
exec_pool *worker_pool() noexcept { return nullptr; }

bool defer_on_complete(std::function<void()>) { return false; }

bool in_pool_callback() noexcept { return false; }

void purge_host_calls(state_exec::exec_group &) noexcept {}

} // namespace detail

} // namespace cvc

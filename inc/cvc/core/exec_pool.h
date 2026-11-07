#ifndef CVC_CORE_EXEC_POOL_H
#define CVC_CORE_EXEC_POOL_H

// The pool that pumps execution groups (docs/STATE_EXEC_THREADED.md 4.1, 4.4, 4.10, 4.14, 4.17;
// Stage 1: fork-join). app::exec_pool() owns one per app. A pump_all() pumps each group it is
// given once, on persistent workers and on the calling thread, and returns when all are done:
// outside a pump_all no worker touches any group, so host code (sweeps, get_process_info, reap,
// kill_owner) runs between them as it does today. The pool owns no group (exec_group.h).

#include <chrono>
#include <cstdint>
#include <cvc/core/cpu_placement.h>
#include <cvc/core/thread_class.h>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace cvc {

namespace state_exec {
class exec_group;
struct pump_budget;
} // namespace state_exec

struct pool_options {
  // threads: auto.
  static constexpr unsigned auto_threads = static_cast<unsigned>(-1);
  // Persistent workers besides the calling thread. 0: every pump runs on the calling thread, as
  // today, and the only mode in a build without threads. auto_threads: the CPUs in the highest
  // efficiency class minus one with cpu_set::performance, hardware_concurrency() - 1 with
  // cpu_set::all. Any count is clamped to hardware_concurrency() - 1 and the affinity mask, and
  // to 0 in a build without threads (state_exec::threads_compiled_in) or for an app whose lock
  // manager is single (S1-Q1); each clamp is logged once, and stats() reports the result.
  unsigned threads = 0;
  // Deterministic mode (td 4.14): every group pumped on the caller in creation order, slices fixed
  // in steps, the deadline and the seconds budgets ignored, random streams seeded from the groups'
  // names.
  bool deterministic = false;
  // The adaptive helper wake (td 4.4): when the groups' last pumps total less than this many
  // seconds, the caller pumps them all and wakes no worker.
  double inline_below = 100e-6;
  cpu_set placement = cpu_set::performance;
  // Declared for the locks track's L4; not applied in Stage 1.
  cvc::thread_class worker_class = cvc::thread_class::normal;
};

// A pool's counters since it was built.
struct pool_stats {
  std::uint64_t pumps = 0;        // group pumps, on any thread
  std::uint64_t helper_wakes = 0; // workers woken to help a pump_all
  std::uint64_t skipped = 0;      // groups a pump_all did not start before its deadline
  std::uint64_t worker_pumps = 0; // group pumps that ran on a worker
  unsigned effective_threads = 0; // the workers running: threads()
};

class exec_pool {
public:
  explicit exec_pool(const pool_options &opts);
  // Stops the workers after their current pumps. Every group's owner has destroyed it before.
  ~exec_pool();
  exec_pool(const exec_pool &) = delete;
  exec_pool &operator=(const exec_pool &) = delete;

  // Pump each of `groups` once (exec_group::pump) and return when all are done (S1.6):
  // - The caller pumps the host-affinity groups first, then helps with the rest. Workers and the
  //   caller take groups from one list, longest last pump first; at 0 threads in creation order.
  // - A group starts only while `deadline` has not passed, with the smaller of budget.seconds
  //   and the time left. A group not started counts in stats().skipped and goes first next call.
  // - A group belongs to one pump_all at a time. Calls from several threads share the workers,
  //   each with its own list, host-call queue and accounting (S1-Q11).
  // - A C++ exception escaping a group's pump fails that group (failed_groups()) and kills its
  //   processes with "internal error"; the others finish. No exception leaves a worker.
  // - The on_complete callbacks of a worker-pumped group run on the caller after the join, in
  //   completion order per group; if any throws, the rest still run and the first throw is
  //   rethrown here. A group pumped on the caller runs them inline, as today.
  // - It returns only with its host-call queue empty (run_on_host).
  // Not from a host_proxied body or an on_complete callback (a debug assertion).
  void pump_all(std::span<state_exec::exec_group *const> groups,
                const state_exec::pump_budget &budget,
                std::chrono::steady_clock::time_point deadline);

  // The workers running, after the clamps (pool_options::threads). The caller pumps too.
  unsigned threads() const;
  pool_stats stats() const;
  // The groups the last pump_all this thread called marked failed.
  std::vector<state_exec::exec_group *> failed_groups() const;

  // The host-call queue (S1.6, td 4.9): run `fn` on the thread that called the pump_all pumping
  // the calling thread's group. It is queued on that call's queue, which its caller runs while it
  // waits at the join, or it runs inline here when the calling thread is that caller or no
  // pump_all is pumping it (0 threads included), so a caller never waits for itself. It returns
  // once `fn` is queued or has run; a host_proxied wrapper that cannot park (a generator body)
  // waits for `fn` itself. The rules:
  // - pump_all returns only with its queue empty;
  // - a group's teardown purges the group's entries without running them (its kills, before,
  //   have settled their calls as cancelled);
  // - `fn` must not call pump_all or destroy a group (a debug assertion);
  // - the host thread never blocks without servicing its queue.
  void run_on_host(std::function<void()> fn);

  // Whether the calling thread is inside one of this pool's pump_all calls: as its caller (where
  // host_proxied bodies and on_complete callbacks run) or as a worker pumping one of its groups.
  bool in_pump_all() const;

  // Tests (GR-9): take `seconds` as `group`'s last pump time, for the helper wake and the order.
  void set_last_pump_for_test(state_exec::exec_group &group, double seconds);

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

// Group pumps that ran on a worker of any pool, process-wide (AR-1).
std::uint64_t exec_pool_worker_pumps_total();

} // namespace cvc

#endif // CVC_CORE_EXEC_POOL_H

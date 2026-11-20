#ifndef CVC_CORE_EXEC_POOL_H
#define CVC_CORE_EXEC_POOL_H

// The pool that pumps execution groups (docs/STATE_EXEC_THREADED.md 4.1, 4.4, 4.10, 4.14, 4.17).
// app::exec_pool() owns one per app. The pool owns no group (exec_group.h). Two modes:
// - Fork-join (Stage 1): a pump_all() pumps each group it is given once, on persistent workers and
//   on the calling thread, and returns when all are done: outside a pump_all no worker touches
//   any group, so host code (sweeps, get_process_info, reap, kill_owner) runs between them as it
//   does today.
// - Continuous (Stage 2, S2a): an attach()ed group is pumped whenever it has work, without a
//   frame barrier; its host touches it under its token (with_group), and services its host queue
//   in pump_host and wait_for_work. With nothing attached, the pool is Stage 1's.

#include <chrono>
#include <cstdint>
#include <cvc/core/cpu_placement.h>
#include <cvc/core/thread_class.h>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace cvc {

class app;
class exec_pool;

namespace state_exec {
class exec_group;
struct pump_budget;
} // namespace state_exec

namespace detail {
bool in_pool_callback(const cvc::app *app) noexcept; // below
} // namespace detail

struct pool_options {
  // threads: auto.
  static constexpr unsigned auto_threads = static_cast<unsigned>(-1);
  // Persistent workers besides the calling thread. 0: every pump runs on the calling thread, as
  // today, and the only mode in a build without threads. auto_threads: the CPUs in the highest
  // efficiency class minus one with cpu_set::performance, hardware_concurrency() - 1 with
  // cpu_set::all. Any count is clamped to hardware_concurrency() - 1 and the affinity mask, and
  // to 0 in a build without threads (state_exec::threads_compiled_in); each clamp is noted once
  // (take_notes), and stats() reports the result. S1-Q1: a group attached to a single-mode lock
  // manager is pumped on the caller only, by any pool, logged once (and asserted in debug builds
  // when a pool with workers is given one); app::exec_pool(opts) builds an inline pool for an app
  // whose manager is single, which an exec_group built before it makes so in a C++ host without
  // state_exec.threads (logged once). A single manager means one thread drives that app: every
  // pump_all of its groups, group 0's drivers and any direct pump come from one host thread, which
  // no pool can check for the host.
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
  // Stage 2 (S2a; S2-Q12, S2-Q17): one continuous pump's budget, as a pump_budget: at most
  // pump_steps steps and pump_seconds of the group's own time (deterministic mode: steps only).
  std::uint64_t pump_steps = 20000;
  double pump_seconds = 2e-3;
  // Virtual time weighted by each group's best_runnable (td 4.4, S2-Q13): declared off; S2c
  // builds it, and turns it on once prio.groups_weighted passes.
  bool priority_weights = false;
  // Give each pump to a thread other than the one that pumped the group last, as
  // CVC_EXEC_FORCE_MOVE=1 does (either turns it on), in both modes: the bench's mt.groups.move
  // compares the two in one rep.
  bool force_move = false;
};

// A pool's counters since it was built.
struct pool_stats {
  std::uint64_t pumps = 0;        // group pumps, on any thread
  std::uint64_t helper_wakes = 0; // workers woken to help a pump_all
  std::uint64_t skipped = 0;      // groups a pump_all did not start before its deadline
  std::uint64_t worker_pumps = 0; // group pumps that ran on a worker
  unsigned effective_threads = 0; // the workers running: threads()
  // Stage 2 (continuous mode, S2a):
  std::uint64_t wakes = 0;           // groups queued by wake(g) (its `scheduled` transition)
  std::uint64_t timer_wakes = 0;     // groups queued because the deadline they published came
  std::uint64_t requeues = 0;        // releases that queued a group again (runnable, notified)
  std::uint64_t parks = 0;           // times a worker parked on the eventcount
  std::uint64_t empty_wakes = 0;     // worker wakeups that found no group to pump (PL-4)
  std::uint64_t moves = 0;           // continuous pumps on another thread than the group's last
  std::uint64_t host_pumps = 0;      // continuous pumps on an attaching thread (pump_host)
  std::uint64_t long_host_waits = 0; // waits of a worker over 10 ms for a host body (S2-Q5)
  unsigned attached = 0;             // groups attached now
  unsigned active_workers = 0;       // workers pumping now, less those blocked over 10 ms (S2-Q11)
};

class exec_pool {
public:
  explicit exec_pool(const pool_options &opts);
  // Stops the workers after their current pumps. Every group's owner has destroyed it before.
  ~exec_pool();
  exec_pool(const exec_pool &) = delete;
  exec_pool &operator=(const exec_pool &) = delete;

  // Pump each of `groups` once (exec_group::pump) and return when all are done (S1.6):
  // - The caller pumps the host-affinity groups (and those on a single lock manager) first, then
  //   helps with the rest. Workers and the caller take groups from one list, longest last pump
  //   first; at 0 threads in creation order.
  // - A group starts only while `deadline` has not passed, with the smaller of budget.seconds
  //   and the time left. A group not started counts in stats().skipped and goes first next call,
  //   the one skipped longest first. host_first (exec_pump_frame's group 0) counts as skipped
  //   when it starts past the deadline, and is ordered with the caller's skipped groups: so of k
  //   caller-pumped units that each fill a frame, each starts at least once in k frames, and
  //   neither a busy group 0 nor busy frame groups starve the other.
  // - A group belongs to one pump_all at a time. Calls from several threads share the workers,
  //   each with its own list, host-call queue and accounting (S1-Q11).
  // - A C++ exception escaping a group's pump fails that group and kills its processes with
  //   "internal error"; the others finish. No exception leaves a worker. It returns the groups it
  //   failed (raw pointers, to compare with, not to use once their owners may have destroyed
  //   them); when it rethrows a host exception (below) they are failed() all the same.
  // - In a threaded pump_all (workers, not deterministic) every group's on_complete callbacks
  //   (exec_group::execute's) run on the caller after the join, whichever thread pumped it, in
  //   completion order per group, with the group's registry and stream bound; if any throws, the
  //   rest still run and the first throw is rethrown here. At 0 threads and in deterministic mode
  //   they run inline in the pump, as today. host_first's own callbacks run inline in it.
  // - It returns only with its host-call queue empty (run_on_host).
  // - Nothing of it is kept per thread: two pools (two apps') pumped by one thread, in turn or
  //   one inside a group of the other's, each see only their own calls and groups.
  // Not from a host_proxied body or an on_complete callback of one of this pool's calls (a debug
  // assertion).
  std::vector<state_exec::exec_group *> pump_all(std::span<state_exec::exec_group *const> groups,
                                                 const state_exec::pump_budget &budget,
                                                 std::chrono::steady_clock::time_point deadline);

  // The workers running, after the clamps (pool_options::threads). The caller pumps too.
  unsigned threads() const;
  pool_stats stats() const;

  // The host-call queue (S1.6, td 4.9): run `fn` on the thread that called the pump_all pumping
  // `group`, when the calling thread is the worker pumping it. It is queued on that call's queue,
  // which its caller runs while it waits at the join, or it runs inline here when the calling
  // thread is that caller or no pump_all of this pool is pumping the group on it (0 threads
  // included), so a caller never waits for itself. It returns
  // once `fn` is queued or has run; a host_proxied wrapper that cannot park (a generator body)
  // waits for `fn` itself. The rules:
  // - pump_all returns only with its queue empty;
  // - a group's teardown purges the group's entries without running them (its kills, before,
  //   have settled their calls as cancelled);
  // - `fn` must not call pump_all or destroy a group (a debug assertion);
  // - the host thread never blocks without servicing its queue;
  // - the caller runs a queued `fn` once no other thread pumps the group: the group's pump has
  //   ended, or its worker waits for a body it queued (run_on_host_waiting); `fn` runs with the
  //   group's registry and stream bound. A parked process so resumes at its group's next pump,
  //   never within the pump that parked it;
  // - one group's bodies run in the order they were queued; bodies of different groups run as
  //   they become runnable, not in queue order across groups;
  // - "the host thread" is the caller of that pump_all: two callers run their own queues at once.
  // Stage 2 (S2-Q5): for an attached group the queue is its attaching thread's, serviced in that
  // thread's pump_host, wait_for_work, with_group and detach; a body runs inline when the calling
  // thread is the attaching thread itself (pumping a caller-pumped group). A body queued by a
  // worker takes the group's token on the attaching thread (with_group's rule), so it never runs
  // beside a pump; a waiting one (run_on_host_waiting) runs under its blocked worker's token.
  void run_on_host(state_exec::exec_group &group, std::function<void()> fn);

  // Whether the calling thread is inside one of this pool's pump_all calls: as its caller (where
  // its natives in group 0, host_proxied bodies and on_complete callbacks run) or as a worker
  // pumping one of its groups. Read from the pool's calls (their callers' and pumpers' thread
  // ids), not kept per thread: another pool's pump_all on this thread is not this one's.
  bool in_pump_all() const;

  // Tests (GR-9): take `seconds` as `group`'s last pump time, for the helper wake and the order.
  void set_last_pump_for_test(state_exec::exec_group &group, double seconds);

  // pump_all with `host_first` run on the caller before anything else, as a host-affinity group's
  // pump would be: app::exec_pump_frame() pumps group 0 (the app scheduler, which is no
  // exec_group) with it, overlapping the workers. Its throw is rethrown after the join, as an
  // on_complete callback's is.
  std::vector<state_exec::exec_group *> pump_all(std::span<state_exec::exec_group *const> groups,
                                                 const state_exec::pump_budget &budget,
                                                 std::chrono::steady_clock::time_point deadline,
                                                 const std::function<void()> &host_first);

  // --- Stage 2: continuous mode (S2a; td 4.4, 4.9, 4.12, 4.14; S2.md S2-Q2..S2-Q6) ---
  //
  // An attached group is pumped whenever it has work, with no frame barrier: a post to it (a
  // message, a completion, a control, a lock post) calls its scheduler's notifier, wake(g), which
  // queues it; a worker pops it, pumps it once on the pool's budget (pump_steps, pump_seconds),
  // queues it again while its pump says it has work, else files the deadline the pump published
  // in the pool's timer heap (group_pool_link in exec_group.h has the state word and its frozen
  // transitions). Workers take the ready queue by virtual time (pump time consumed; a woken
  // group's clamped to the queue's minimum less one pump, S2-Q13), spin briefly, then park on the
  // pool's eventcount until a wake or the earliest deadline (S2-Q3).
  //
  // The attaching thread (the one that called attach) is "the host thread" of what it attached
  // (S2-Q5). It alone pumps its caller-pumped groups: a host-affinity group, one on a single-mode
  // lock manager, and every group of a pool with no workers or in deterministic mode (td 4.14;
  // deterministic mode pumps them in creation order, on steps only). It runs their host-queue
  // entries (host_proxied bodies, deferred on_complete callbacks) and finishes their failures, in
  // pump_host, wait_for_work, with_group and detach, and never blocks without servicing that
  // queue. It detaches what it attached before it exits (unchecked: nothing is kept per thread),
  // and ~exec_pool asserts that nothing is attached.

  // Attach `group`: the calling thread becomes its attaching thread, its scheduler's notifier is
  // installed, and it is woken once, so work posted before is seen. Refused (std::logic_error)
  // when it is attached already, to any pool, or a pump_all holds it. pump_all refuses it from
  // then on (an assertion; std::logic_error in release builds) until it is detached. It may be
  // destroyed attached, on its attaching thread: its teardown detaches it first (S2-Q4).
  void attach(state_exec::exec_group &group);
  // Detach `group` (S2-Q4's order, without the kills): no worker pumps it once this returns, its
  // notifier is cleared, its queued host bodies and callbacks have run on this thread, and it is
  // an unattached group again, which pump_all takes. Waits for a pump in progress, servicing this
  // thread's host queue. On its attaching thread (a debug assertion). Nothing for a group that is
  // not attached to this pool.
  void detach(state_exec::exec_group &group);
  // On an attaching thread: run its host-queue entries, then pump its queued caller-pumped
  // groups, each once on the pool's budget as a worker would, until none is queued or
  // `max_seconds` passed (infinity: no limit; the entries run whatever it is). True when it ran an
  // entry or pumped a group. Never from a host body, a callback or a pump.
  bool pump_host(double max_seconds);
  // On an attaching thread: block until it has work for pump_host (a caller-pumped group queued
  // or falling due, a host-queue entry, a failed group to take) or `seconds` passed (infinity: no
  // limit), running its host-queue entries meanwhile. True when there is work (an entry it ran
  // counts); false when the time passed with none.
  bool wait_for_work(double seconds);
  // Run `fn` on this thread under `group`'s token (S2-Q4). For an attached group: wait for a pump
  // in progress (servicing this thread's host queue), keep workers off it while `fn` runs (one
  // that pops it meanwhile leaves it notified), and queue it again afterwards if one did. For a
  // group not attached: take the token as async_scheduler::pump_owner_scope does (throwing when
  // another thread holds it). For touching the group's scheduler (submit, read, kill, reap), not
  // for pumping it. An exception from `fn` propagates once the token is released.
  void with_group(state_exec::exec_group &group, const std::function<void()> &fn);
  // On an attaching thread: its attached groups whose pump failed (a C++ exception escaped it;
  // their processes were killed with "internal error"), each returned once, in the order they
  // failed, detached by this call. Logged through their apps, as pump_all logs a failure.
  std::vector<state_exec::exec_group *> take_failed();
  // Whether `group` is attached to this pool (any thread; a snapshot).
  bool attached(const state_exec::exec_group &group) const;

  // The options the pool was built with, before its clamps (threads() is what they came to).
  const pool_options &options() const;
  // Whether the pool runs in deterministic mode: its options' flag, or CVC_EXEC_DETERMINISTIC=1
  // when it was built.
  bool deterministic() const;
  // What building the pool clamped, and the workers that failed to start, one line each, for its
  // owner to log once (app::exec_pool() does); taken, so the next call returns none.
  std::vector<std::string> take_notes();

private:
  friend bool detail::in_pool_callback(const cvc::app *app) noexcept;

  struct impl;
  std::unique_ptr<impl> impl_;
};

// What an exec_group uses of the pool it is pumped by (exec_group.cpp); not for hosts. Each takes
// the group (or the app) it is about: a pump_all holding a group points the group at its own
// entry (pool_link().call), so nothing is kept per thread.
namespace detail {
// The pool whose worker the calling thread is, while it pumps `group` for a pump_all or as an
// attached group (Stage 2); else null. A host_proxied native of `group` queues its body there
// (run_on_host), and runs it inline anywhere else.
exec_pool *worker_pool(state_exec::exec_group &group) noexcept;
// While a threaded pump_all pumps `group` on the calling thread (a worker, or its caller): queue
// `callback` (an on_complete call of the group's) for that call's caller, which runs it after the
// join, and return true. While a worker pumps `group` attached (Stage 2, S2-Q5): queue it on the
// group's attaching thread's host queue, and return true. Anywhere else (threads 0,
// deterministic mode, no pump_all, the attaching thread pumping it, another thread) return false:
// run it inline.
bool defer_on_complete(state_exec::exec_group &group, std::function<void()> callback);
// Whether the calling thread runs a queued host_proxied body or a deferred on_complete callback
// of a pump_all of `app`'s pool (app::exec_pool(), if built), where destroying a group is refused
// (a debug assertion). False for a null app.
bool in_pool_callback(const cvc::app *app) noexcept;
// A group's teardown (S1.5): drop the group's entries from the host-call queue of the pump_all it
// belongs to, unrun, waiting for one its caller is running now. Nothing when it belongs to none
// (always, in practice: ~exec_group aborts while a pump_all holds its group). Stage 2: an
// attached group's entries on its attaching thread's queue too, which detach_end leaves there.
void purge_host_calls(state_exec::exec_group &group) noexcept;
// Stage 2 (S2a): an attached group's teardown (S2-Q4; ~exec_group), steps 1 and 2: set detaching,
// then wait for a pump in progress (and a host_held of another thread) to end, servicing the
// calling thread's host queue meanwhile. False, doing nothing, when the group is not attached.
bool detach_begin(state_exec::exec_group &group) noexcept;
// Step 4, once the teardown holds the token and has closed the lock port: the group leaves the
// ready queue, the next slots, its attaching thread's ready and failed lists and the timer heap,
// its notifier is cleared (waiting for a wake in progress) and its attachment dropped. Its
// host-queue entries stay for purge_host_calls, after the kills. Nothing when it is not attached.
void detach_end(state_exec::exec_group &group) noexcept;
// run_on_host for a worker pumping `group` that then waits for `fn` to have run (a host_proxied
// call that cannot park): the caller runs it as soon as it services the queue, while the worker
// is blocked.
void run_on_host_waiting(exec_pool &pool, state_exec::exec_group &group, std::function<void()> fn);
} // namespace detail

} // namespace cvc

#endif // CVC_CORE_EXEC_POOL_H

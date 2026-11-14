// The app's pool of execution groups, fork-join (exec_pool.h; docs/STATE_EXEC_THREADED.md 4.1,
// 4.4, 4.9, 4.10, 4.14, 4.17; S1.md S1.6 and S1-Q11).
//
// A pump_all call is a call_state on its caller's stack: its groups, the shared LPT list of the
// `any` groups not yet started, its host-call queue and its accounting. The pool's mutex guards
// every call and the groups' pool records; taking a group from the list under it is the handoff
// edge, and each group's token (its scheduler's pump owner) is taken by exec_group::pump itself.
// Workers are shared by every call; a call's caller services only its own queue.

#include <algorithm>
#include <atomic>
#include <boost/thread/thread.hpp>
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/cpu_placement.h>
#include <cvc/core/exec_pool.h>
#include <cvc/core/thread_class.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/stdlib.h>
#include <deque>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace cvc {

namespace se = cvc::state_exec;
using clock_type = std::chrono::steady_clock;

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v && v[0] == '1' && v[1] == '\0';
}

// The knobs (td 4.13): read when a pool is built, so a test sets one for the pool it builds.
bool force_move_knob() { return env_flag("CVC_EXEC_FORCE_MOVE"); }
bool deterministic_knob() { return env_flag("CVC_EXEC_DETERMINISTIC"); }

// A stable hash of a group's name, its random stream's seed in deterministic mode (FNV-1a).
std::uint64_t name_seed(const std::string &name) {
  std::uint64_t h = 14695981039346656037ull;
  for (unsigned char c : name) {
    h ^= c;
    h *= 1099511628211ull;
  }
  return h;
}

// One group of a pump_all.
struct entry {
  se::exec_group *group = nullptr;
  bool host = false;               // the caller pumps it (host affinity, single lock)
  std::uint64_t skipped_since = 0; // skipped from that call on: starts first
  bool running = false;            // being pumped now (under the pool's mutex)
  bool blocked = false; // its worker waits for a host body it queued (under the pool's mutex)
  std::vector<std::function<void()>> deferred; // its on_complete calls, for after the join
  std::uint64_t finished = 0;                  // when its pump ended (the callbacks' order)
};

struct host_call {
  entry *e;   // whose group's worker queued it (a teardown purges by its group)
  bool waits; // the worker waits for it (run_on_host_waiting): run at once
  std::function<void()> fn;
};

struct call_state;

// A worker's thread, while it pumps a group for a call.
struct worker_ctx {
  exec_pool *pool;
  call_state *call;
  entry *e;
};

thread_local worker_ctx *t_worker = nullptr;
thread_local call_state *t_caller = nullptr; // the pump_all this thread is the caller of
thread_local entry *t_defer = nullptr;       // the entry whose on_complete calls are deferred
thread_local bool t_callback = false;        // in a queued host body or a deferred on_complete

// The last pump_all's failed groups of each pool this thread called, by the pool's life token (an
// owner-equal weak_ptr names one pool for as long as the entry holds it, even once the pool is
// gone and another is made where it was); a list goes when the next call fails none, or once its
// pool is gone (failed_groups()).
thread_local std::vector<std::pair<std::weak_ptr<void>, std::vector<se::exec_group *>>> t_failed;

bool same_owner(const std::weak_ptr<void> &a, const std::shared_ptr<void> &b) {
  return !a.owner_before(b) && !b.owner_before(a);
}

struct callback_scope {
  bool outer;
  callback_scope() : outer(t_callback) { t_callback = true; }
  ~callback_scope() { t_callback = outer; }
};

// The pool's state (exec_pool::impl is this): its workers, the calls in progress, its counters.
struct pool_state {
  exec_pool *self = nullptr;
  std::shared_ptr<void> life = std::make_shared<int>(0); // t_failed's key for this pool
  pool_options opts;
  unsigned threads = 0; // workers started
  bool deterministic = false;
  bool force_move = false;

  std::mutex m;
  std::condition_variable work_cv;  // workers: a call wants a helper, or stop
  std::condition_variable purge_cv; // a host body a caller ran has ended
  std::vector<call_state *> calls;  // the pump_all calls in progress
  std::size_t next_call = 0;        // where workers start looking among the calls (round robin)
  bool stopping = false;
  pool_stats stats;
  double last_host_first = 0.0; // the last host_first's time (exec_pump_frame's group 0)
  // host_first started past the deadline from that call on (0: it did not), so it took no time:
  // it starts first next call, as a skipped group does (N1).
  std::uint64_t host_first_skipped_since = 0;
  bool single_noted = false; // a single-manager group was kept on the caller (logged once)
  std::vector<std::string> notes;
  std::vector<std::unique_ptr<boost::thread>> workers;
  // Numbers this pool's pump_all calls: a skipped group's (or host_first's) stamp is the call that
  // first skipped it, and the oldest stamp starts first (N1, S-M1). An app's groups are pumped by
  // its own pool, so the stamps they carry are this counter's.
  std::atomic<std::uint64_t> call_seq{0};

  void worker_main(int index, bool p_core, bool place);
};

struct call_state {
  pool_state *pool = nullptr;
  std::uint64_t seq = 0; // pool_state::call_seq
  se::pump_budget budget{0, 0.0};
  clock_type::time_point deadline;
  bool timed = true;     // the deadline and the seconds budgets apply (not deterministic)
  bool threaded = false; // workers may pump its groups: every on_complete waits for the join
  std::thread::id caller;
  std::vector<entry> entries;
  std::vector<std::size_t> list; // the `any` entries in LPT order; [front, back) not started
  std::size_t front = 0, back = 0;
  std::size_t running = 0;         // groups being pumped
  std::size_t wakes_left = 0;      // helpers it may still wake
  std::size_t helpers_pending = 0; // helpers woken (counted) that no worker has answered yet
  std::deque<host_call> queue;
  se::exec_group *running_body = nullptr; // whose host body its caller runs now
  std::condition_variable cv;             // its caller: a group done, a host call queued
  std::uint64_t finish_seq = 0;
  std::vector<se::exec_group *> failed;
  se::exec_group *single_note = nullptr; // a single-manager group to log, once per pool
  std::exception_ptr error; // the first host exception (on_complete, host_first, a host body)

  std::size_t remaining() const { return back - front; }
};

// What one pump did, for the bookkeeping done under the pool's mutex.
struct pumped {
  double seconds = 0.0;
  bool failed = false;
  std::exception_ptr error;
};

double seconds_left(const call_state &c, clock_type::time_point now) {
  if (c.deadline == clock_type::time_point::max())
    return std::numeric_limits<double>::infinity();
  return std::max(0.0, std::chrono::duration<double>(c.deadline - now).count());
}

// After a pump a C++ exception escaped (GR-12): kill the group's processes, on the pumping thread,
// holding its token again. Never throws.
void kill_failed_group(se::exec_group &g) noexcept {
  try {
    se::async_scheduler &s = g.sched();
    se::async_scheduler::pump_owner_scope token(s);
    for (const se::process_info &p : s.list_processes())
      if (!se::is_finished(p.status)) {
        try {
          s.kill(p.pid, "internal error");
        } catch (...) {
        }
      }
  } catch (...) {
  }
}

// Pump `e` once on this thread (no lock held): the frame's budget, the token in pump(). Never
// throws: an exception out of pump() (an inline on_complete's) is returned.
pumped pump_entry(call_state &c, entry &e, bool on_worker, exec_pool *pool) noexcept {
  pumped out;
  worker_ctx ctx{pool, &c, &e};
  worker_ctx *const outer = t_worker;
  entry *const outer_defer = t_defer;
  t_worker = on_worker ? &ctx : nullptr;
  t_defer = on_worker || c.threaded ? &e : nullptr;
  const clock_type::time_point start = clock_type::now();
  try {
    se::pump_budget budget = c.budget;
    if (c.timed)
      budget.seconds = std::min(budget.seconds, seconds_left(c, start));
    else
      budget.seconds = std::numeric_limits<double>::infinity(); // deterministic: steps only
    out.failed = e.group->pump(budget).failed;
  } catch (...) {
    out.error = std::current_exception();
  }
  t_worker = outer;
  t_defer = outer_defer;
  out.seconds = std::chrono::duration<double>(clock_type::now() - start).count();
  if (out.failed)
    kill_failed_group(*e.group);
  return out;
}

// Under the pool's mutex: the next group `thread` may start from `c`'s list (P-core threads from
// the front, E-core ones from the back), or none. Past the deadline the rest are skipped.
std::optional<std::size_t> take_group(pool_state &p, call_state &c, int thread, bool from_back) {
  if (c.remaining() == 0)
    return std::nullopt;
  if (c.timed && clock_type::now() >= c.deadline) {
    for (std::size_t i = c.front; i < c.back; ++i) {
      se::detail::group_pool_link &link = c.entries[c.list[i]].group->pool_link();
      link.skipped = true;
      if (!link.skipped_since)
        link.skipped_since = c.seq;
    }
    p.stats.skipped += c.remaining();
    c.front = c.back;
    // The call may be complete now: its caller may be waiting for a worker (CVC_EXEC_FORCE_MOVE
    // refuses it the groups it pumped last), so tell it.
    c.cv.notify_all();
    return std::nullopt;
  }
  if (p.force_move) {
    // A different thread than last time (CVC_EXEC_FORCE_MOVE): the first such group from this
    // thread's end, swapped to it.
    std::optional<std::size_t> at;
    for (std::size_t k = 0; k < c.remaining() && !at; ++k) {
      const std::size_t i = from_back ? c.back - 1 - k : c.front + k;
      if (c.entries[c.list[i]].group->pool_link().last_thread != thread)
        at = i;
    }
    if (!at)
      return std::nullopt;
    std::swap(c.list[*at], c.list[from_back ? c.back - 1 : c.front]);
  }
  const std::size_t idx = from_back ? c.list[--c.back] : c.list[c.front++];
  c.entries[idx].group->pool_link().skipped = false;
  c.entries[idx].group->pool_link().skipped_since = 0;
  c.entries[idx].running = true;
  ++c.running;
  return idx;
}

// Under the pool's mutex: record `e`'s pump on `thread`, and tell the caller. Nothing here
// allocates (c.failed is reserved), so it does not throw.
void finish_group(pool_state &p, call_state &c, entry &e, int thread, const pumped &r,
                  bool on_worker) noexcept {
  se::detail::group_pool_link &link = e.group->pool_link();
  link.last_pump_seconds = r.seconds;
  link.last_thread = thread;
  ++p.stats.pumps;
  if (on_worker)
    ++p.stats.worker_pumps;
  e.finished = ++c.finish_seq;
  e.running = false;
  if (r.failed && c.failed.size() < c.failed.capacity())
    c.failed.push_back(e.group);
  if (r.error && !c.error)
    c.error = r.error;
  --c.running;
  c.cv.notify_all();
}

// Under the pool's mutex: wake one more helper for `c`. A wake is counted when it is made, so
// stats().helper_wakes does not depend on which thread gets to the work first (GR-9). Each wake
// is answered on its own call (helpers_pending is per call), by whichever idle worker comes.
void wake_helper(pool_state &p, call_state &c) {
  --c.wakes_left;
  ++c.helpers_pending;
  ++p.stats.helper_wakes;
  p.work_cv.notify_one();
}

// Under the pool's mutex: a call with a helper wake pending that `thread` can serve, looking from
// the next call in turn, so concurrent calls are served round robin (S1-Q11).
call_state *call_wanting_help(pool_state &p, int thread) {
  const std::size_t n = p.calls.size();
  for (std::size_t k = 0; k < n; ++k) {
    call_state *c = p.calls[(p.next_call + k) % n];
    if (c->helpers_pending == 0 || c->remaining() == 0)
      continue;
    bool can = !p.force_move;
    for (std::size_t i = c->front; i < c->back && !can; ++i)
      can = c->entries[c->list[i]].group->pool_link().last_thread != thread;
    if (can) {
      p.next_call = (p.next_call + k + 1) % n;
      return c;
    }
  }
  return nullptr;
}

void pool_state::worker_main(int index, bool p_core, bool place) {
  // No exception leaves a worker (td 4.17): a pump's is caught in pump_entry, and pump_entry and
  // finish_group do not throw, so a group taken is always finished; anything else ends the worker.
  try {
    detail::apply_process_class_here();
    se::collect_cycles_at_thread_exit(); // its own registry should stay empty (S1-Q3)
    if (place)
      place_current_thread(opts.placement, static_cast<unsigned>(index - 1));
  } catch (...) {
  }
  const bool from_back = !p_core && opts.placement == cpu_set::all; // td 4.4: E-cores from the back
  try {
    std::unique_lock<std::mutex> lk(m);
    for (;;) {
      work_cv.wait(lk, [&] { return stopping || call_wanting_help(*this, index) != nullptr; });
      if (stopping)
        break;
      call_state *c = call_wanting_help(*this, index);
      --c->helpers_pending;
      // Help while the call has work; the woken helper wakes the next while work remains (td 4.4).
      while (const std::optional<std::size_t> idx = take_group(*this, *c, index, from_back)) {
        if (c->remaining() > 0 && c->wakes_left > 0)
          wake_helper(*this, *c);
        entry &e = c->entries[*idx];
        lk.unlock();
        const pumped r = pump_entry(*c, e, true, self);
        lk.lock();
        finish_group(*this, *c, e, index, r, true);
      }
    }
  } catch (...) {
  }
  // A group's objects go to its registry, which its scheduler binds: none is tracked here.
  assert(se::tracked_objects() == 0 && "a pool worker tracked objects in its own registry");
}

// Run `fn` (a queued host body or a deferred on_complete) for `group` on the caller, with the
// group's registry and stream bound (no other thread pumps the group meanwhile), and keep its
// first exception in `c`.
void run_for_group(call_state &c, se::exec_group &group, std::function<void()> &fn,
                   std::unique_lock<std::mutex> *lk) noexcept {
  callback_scope in_callback;
  try {
    se::cycle_registry_scope registry(group.registry());
    se::random_stream_scope stream(group.pool_link().random);
    fn();
  } catch (...) {
    if (lk)
      lk->lock();
    if (!c.error)
      c.error = std::current_exception();
    if (lk)
      lk->unlock();
  }
  try {
    fn = nullptr; // what it captured goes here, outside the lock
  } catch (...) {
  }
}

// Under the pool's mutex: the next queued host body the caller may run. A group's bodies run in
// the order they were queued; one runs once no other thread pumps its group: the group's pump has
// ended (its process resumes at its next pump), or its worker is blocked on a body it queued (a
// generator body's call, run_on_host_waiting). Bodies of different groups run as they become
// runnable, not in queue order across groups.
std::optional<std::size_t> runnable_body(const call_state &c) {
  for (std::size_t i = 0; i < c.queue.size(); ++i) {
    const entry *e = c.queue[i].e;
    bool earlier = false;
    for (std::size_t j = 0; j < i && !earlier; ++j)
      earlier = c.queue[j].e == e;
    if (!earlier && (!e->running || e->blocked))
      return i;
  }
  return std::nullopt;
}

} // namespace

struct exec_pool::impl : pool_state {};

exec_pool::exec_pool(const pool_options &opts) : impl_(std::make_unique<impl>()) {
  impl &p = *impl_;
  p.self = this;
  p.opts = opts;
  p.deterministic = opts.deterministic || deterministic_knob();
  p.force_move = force_move_knob();
  const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
  unsigned want = opts.threads;
  if (want == pool_options::auto_threads)
    want = opts.placement == cpu_set::performance ? performance_core_count() - 1 : hw - 1;
  // The clamps (td 4.14, S1.6), each noted once for the owner to log.
  const auto clamp = [&](unsigned limit, const std::string &why) {
    if (want > limit) {
      p.notes.push_back("exec_pool: " + std::to_string(want) + " workers clamped to " +
                        std::to_string(limit) + " (" + why + ")");
      want = limit;
    }
  };
  clamp(hw - 1, "hardware_concurrency() - 1");
  if (const std::size_t mask = logical_cpus().size(); mask > 0)
    clamp(static_cast<unsigned>(mask - 1), "the process's affinity mask, less the caller");
  if (!se::threads_compiled_in)
    clamp(0, "a build without threads");
  if (want == 0 || !se::threads_compiled_in)
    return;
  // Workers on the placement's CPUs in order (one per physical core first, the performance cores
  // first), each on its own while there are CPUs for it; one beyond them floats.
  const std::vector<logical_cpu> cpus = placement_cpus(opts.placement);
  unsigned best = 0;
  for (const logical_cpu &cpu : cpus)
    best = std::max(best, cpu.efficiency);
  p.workers.reserve(want); // no push_back throws with a thread started
  for (unsigned i = 0; i < want; ++i) {
    const bool place = i < cpus.size();
    const bool p_core = !place || cpus[i].efficiency == best;
    const int index = static_cast<int>(i) + 1; // the caller is thread 0
    try {
      boost::thread::attributes attrs;
      attrs.set_stack_size(8u << 20); // td 4.1: room for the native-stack guard's reserve
      p.workers.push_back(std::make_unique<boost::thread>(
          attrs, [ps = &p, index, p_core, place] { ps->worker_main(index, p_core, place); }));
    } catch (const std::exception &e) {
      // The pool runs with the workers that started; the caller always pumps (td 4.17).
      p.notes.push_back("exec_pool: worker " + std::to_string(index) +
                        " failed to start: " + e.what());
    }
  }
  p.threads = static_cast<unsigned>(p.workers.size());
}

exec_pool::~exec_pool() {
  impl &p = *impl_;
  {
    std::lock_guard<std::mutex> lock(p.m);
    assert(p.calls.empty() && "exec_pool destroyed during a pump_all");
    p.stopping = true; // workers stop after their current pumps
  }
  p.work_cv.notify_all();
  boost::this_thread::disable_interruption hold; // join is an interruption point
  for (std::unique_ptr<boost::thread> &w : p.workers)
    if (w->joinable())
      w->join();
  std::erase_if(t_failed, [&](const auto &f) { return same_owner(f.first, p.life); });
}

void exec_pool::pump_all(std::span<state_exec::exec_group *const> groups,
                         const state_exec::pump_budget &budget,
                         std::chrono::steady_clock::time_point deadline) {
  pump_all(groups, budget, deadline, std::function<void()>());
}

void exec_pool::pump_all(std::span<state_exec::exec_group *const> groups,
                         const state_exec::pump_budget &budget,
                         std::chrono::steady_clock::time_point deadline,
                         const std::function<void()> &host_first) {
  impl &p = *impl_;
  assert(!t_callback && "pump_all from a host_proxied body or an on_complete callback");
  call_state c;
  c.pool = &p;
  c.seq = p.call_seq.fetch_add(1, std::memory_order_relaxed) + 1;
  c.budget = budget;
  c.deadline = deadline;
  c.timed = !p.deterministic;
  c.threaded = p.threads > 0 && !p.deterministic;
  c.caller = std::this_thread::get_id();
  c.entries.reserve(groups.size());
  c.failed.reserve(groups.size()); // finish_group never allocates
  {
    // A group belongs to one pump_all at a time (S1-Q11): asserted; a release build skips it.
    for (se::exec_group *g : groups) {
      if (!g)
        continue;
      se::detail::group_pool_link &link = g->pool_link();
      std::lock_guard<std::mutex> lock(link.membership);
      if (void *held = link.call.load(std::memory_order_relaxed)) {
        assert(held == &c && "an exec_group given to two pump_all calls at once");
        (void)held;
        continue; // given twice in this call, or held by another
      }
      // S1-Q1: a group on a single-mode lock manager is pumped on the caller only, whichever
      // pool this is; a pool with workers given one is a host's mistake (a debug assertion).
      if (link.caller_only && c.threaded) {
        assert(false && "a group on a single-mode lock manager given to a pool with workers");
        if (!c.single_note)
          c.single_note = g; // logged once per pool, decided under the pool's mutex below
      }
      link.call.store(&c, std::memory_order_relaxed);
      entry e;
      e.group = g;
      e.host = g->opts().affinity == se::group_affinity::host || link.caller_only;
      e.skipped_since = link.skipped ? std::max<std::uint64_t>(link.skipped_since, 1) : 0;
      c.entries.push_back(std::move(e));
    }
  }
  struct membership_end {
    call_state &c;
    ~membership_end() {
      for (entry &e : c.entries) {
        se::detail::group_pool_link &link = e.group->pool_link();
        std::lock_guard<std::mutex> lock(link.membership);
        link.call.store(nullptr, std::memory_order_relaxed);
      }
    }
  } leave{c};
  if (p.deterministic)
    for (entry &e : c.entries) {
      // Seeded from its name, unless it has a seed or drew already (td 4.4).
      se::random_stream *r = e.group->pool_link().random;
      if (r && !r->seeded && !r->seed)
        r->seed = name_seed(e.group->opts().name);
    }
  call_state *const outer_caller = t_caller;
  t_caller = &c;
  struct caller_end {
    call_state *outer;
    ~caller_end() { t_caller = outer; }
  } restore{outer_caller};

  const auto by_creation = [&](std::size_t a, std::size_t b) {
    return c.entries[a].group->pool_link().seq < c.entries[b].group->pool_link().seq;
  };
  const auto run_host_first = [&] {
    if (!host_first)
      return;
    const clock_type::time_point start = clock_type::now();
    // Started past the deadline it takes no time (exec_pump_frame's group 0 gets what the frame
    // has left): then it is stamped as skipped and starts first next call (N1).
    const bool late = c.timed && start >= c.deadline;
    try {
      host_first();
    } catch (...) {
      if (!c.error)
        c.error = std::current_exception();
    }
    std::lock_guard<std::mutex> lock(p.m);
    p.last_host_first = std::chrono::duration<double>(clock_type::now() - start).count();
    if (!late)
      p.host_first_skipped_since = 0;
    else if (!p.host_first_skipped_since)
      p.host_first_skipped_since = c.seq;
    ++p.stats.pumps;
  };
  // Pump entry `i` on the caller, if the deadline allows it (else it is skipped).
  const auto caller_pumps = [&](std::size_t i) {
    entry &e = c.entries[i];
    {
      std::lock_guard<std::mutex> lock(p.m);
      se::detail::group_pool_link &link = e.group->pool_link();
      if (c.timed && clock_type::now() >= c.deadline) {
        link.skipped = true;
        if (!link.skipped_since)
          link.skipped_since = c.seq;
        ++p.stats.skipped;
        return;
      }
      link.skipped = false;
      link.skipped_since = 0;
      e.running = true;
      ++c.running;
    }
    const pumped r = pump_entry(c, e, false, this);
    std::lock_guard<std::mutex> lock(p.m);
    finish_group(p, c, e, 0, r, false);
  };
  // What the caller pumps itself: `mine` (in creation order) and host_first. Those skipped before
  // go first, the one skipped longest first (host_first first on a tie), then host_first, then
  // the rest: so with k of them that each fill a frame, each starts at least once in k frames,
  // and neither a busy group 0 nor busy frame groups keep the other from starting (S-M1, N1).
  // Deterministic mode skips nothing: host_first, then the rest.
  const auto caller_runs = [&](const std::vector<std::size_t> &mine) {
    std::uint64_t first_since = 0;
    if (host_first && c.timed) {
      std::lock_guard<std::mutex> lock(p.m);
      first_since = p.host_first_skipped_since;
    }
    struct unit {
      std::uint64_t since;
      bool is_host_first;
      std::size_t i;
    };
    std::vector<unit> skipped;
    std::vector<std::size_t> rest;
    if (first_since)
      skipped.push_back({first_since, true, 0});
    for (std::size_t i : mine) {
      if (c.timed && c.entries[i].skipped_since)
        skipped.push_back({c.entries[i].skipped_since, false, i});
      else
        rest.push_back(i);
    }
    std::stable_sort(skipped.begin(), skipped.end(), [](const unit &a, const unit &b) {
      return a.since != b.since ? a.since < b.since : a.is_host_first && !b.is_host_first;
    });
    for (const unit &u : skipped) {
      if (u.is_host_first)
        run_host_first();
      else
        caller_pumps(u.i);
    }
    if (!first_since)
      run_host_first();
    for (std::size_t i : rest)
      caller_pumps(i);
  };

  std::vector<std::size_t> order(c.entries.size());
  for (std::size_t i = 0; i < order.size(); ++i)
    order[i] = i;

  if (!c.threaded) {
    // Everything on the caller, in creation order (td 4.14: at threads 0 the results are
    // deterministic mode's), the skipped ones and host_first as caller_runs orders them.
    std::sort(order.begin(), order.end(), by_creation);
    caller_runs(order);
  } else {
    // The host-affinity groups on the caller, in creation order; the rest on the shared list,
    // longest last pump first (LPT, td 4.4), skipped ones first.
    std::vector<std::size_t> host;
    double total = 0.0;
    {
      std::lock_guard<std::mutex> lock(p.m);
      if (host_first)
        total += p.last_host_first;
      for (std::size_t i : order) {
        total += c.entries[i].group->pool_link().last_pump_seconds;
        (c.entries[i].host ? host : c.list).push_back(i);
      }
    }
    std::sort(host.begin(), host.end(), by_creation);
    std::sort(c.list.begin(), c.list.end(), [&](std::size_t a, std::size_t b) {
      const entry &ea = c.entries[a], &eb = c.entries[b];
      if (ea.skipped_since != eb.skipped_since)
        return ea.skipped_since && (!eb.skipped_since || ea.skipped_since < eb.skipped_since);
      const double la = ea.group->pool_link().last_pump_seconds;
      const double lb = eb.group->pool_link().last_pump_seconds;
      if (la != lb)
        return la > lb;
      return ea.group->pool_link().seq < eb.group->pool_link().seq;
    });
    c.back = c.list.size();
    // The adaptive helper wake (td 4.4): none when the groups' last pumps total under
    // inline_below; else one per group beyond the caller's first, up to the pool's size.
    std::size_t wakes = 0;
    if (!c.list.empty()) {
      const std::size_t caller_takes = host.empty() && !host_first ? 1 : 0;
      if (p.force_move)
        wakes = std::min<std::size_t>(c.list.size(), p.threads);
      else if (total >= p.opts.inline_below)
        wakes = std::min<std::size_t>(c.list.size() - caller_takes, p.threads);
    }
    {
      std::lock_guard<std::mutex> lock(p.m);
      p.calls.push_back(&c);
      c.wakes_left = wakes;
      if (p.force_move) {
        // Every group to a thread other than its last: every helper at once.
        c.helpers_pending = wakes;
        c.wakes_left = 0;
        p.stats.helper_wakes += wakes;
        p.work_cv.notify_all();
      } else if (wakes > 0) {
        wake_helper(p, c);
      }
    }
    struct call_end {
      pool_state &p;
      call_state &c;
      ~call_end() {
        std::lock_guard<std::mutex> lock(p.m);
        p.calls.erase(std::find(p.calls.begin(), p.calls.end(), &c));
      }
    } unregister{p, c};
    // The join (td 4.9: the host thread never blocks without servicing the queue). `drain`: take
    // no more groups, only run the queue until every group taken has finished; an exception out
    // of the loop drains before it unwinds, since workers hold `c` until their groups finish.
    const auto join = [&](bool drain) {
      std::unique_lock<std::mutex> lk(p.m);
      if (drain)
        c.front = c.back;
      for (;;) {
        if (const std::optional<std::size_t> at = runnable_body(c)) {
          host_call hc = std::move(c.queue[*at]);
          c.queue.erase(c.queue.begin() + static_cast<std::ptrdiff_t>(*at));
          c.running_body = hc.e->group;
          lk.unlock();
          run_for_group(c, *hc.e->group, hc.fn, &lk);
          lk.lock();
          if (hc.waits)
            hc.e->blocked = false; // its worker resumes now
          c.running_body = nullptr;
          p.purge_cv.notify_all();
          continue;
        }
        if (!drain) {
          if (const std::optional<std::size_t> idx = take_group(p, c, 0, false)) {
            entry &e = c.entries[*idx];
            lk.unlock();
            const pumped r = pump_entry(c, e, false, this);
            lk.lock();
            finish_group(p, c, e, 0, r, false);
            continue;
          }
        }
        if (c.running == 0 && c.remaining() == 0 && c.queue.empty())
          break;
        if (!drain && p.force_move && c.running == 0 && c.remaining() > 0) {
          // Every group left was the caller's last: hand them to the workers.
          c.helpers_pending = std::max<std::size_t>(c.helpers_pending, p.threads);
          p.work_cv.notify_all();
        }
        // With groups not started (only under CVC_EXEC_FORCE_MOVE: the caller takes any other),
        // wait no longer than the deadline, then skip them here on time; a busy worker might reach
        // the call much later, or never wake this caller. In slices of at most a second: a far
        // deadline (time_point::max()) is not handed to wait_until, where some libraries overflow.
        if (!drain && c.timed && c.remaining() > 0)
          c.cv.wait_for(lk, std::min<clock_type::duration>(c.deadline - clock_type::now(),
                                                           std::chrono::seconds(1)));
        else
          c.cv.wait(lk);
      }
    };
    try {
      // The caller's own: the host-affinity groups and host_first, and, with no helper woken
      // (the caller is then the only pumper of this call), the shared list's groups too, so
      // their skips are ordered with host_first's as at threads 0. Helpers take the list beside
      // the caller otherwise, skipped groups first.
      std::vector<std::size_t> mine = host;
      if (wakes == 0) {
        std::lock_guard<std::mutex> lock(p.m);
        mine.insert(mine.end(), c.list.begin() + static_cast<std::ptrdiff_t>(c.front),
                    c.list.begin() + static_cast<std::ptrdiff_t>(c.back));
        c.front = c.back;
      }
      std::sort(mine.begin(), mine.end(), by_creation);
      caller_runs(mine);
      join(false);
    } catch (...) {
      join(true);
      throw;
    }
  }

  // After the join, on the caller: the on_complete calls deferred during it (every group's in a
  // threaded call), group by group in the order their pumps ended, each in completion order, with
  // the group's registry and stream bound; the first throw is rethrown once all ran (S1.6).
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return c.entries[a].finished < c.entries[b].finished;
  });
  for (std::size_t i : order) {
    for (std::function<void()> &cb : c.entries[i].deferred)
      run_for_group(c, *c.entries[i].group, cb, nullptr);
    c.entries[i].deferred.clear();
  }
  // A failed group is logged through its app, here on the host thread; and, once per pool, a
  // group kept on the caller because its lock manager is single (S1-Q1).
  const auto log = [](se::exec_group &g, const std::string &text) {
    if (cvc::app *a = g.opts().app) {
      try {
        boost::this_thread::disable_interruption hold;
        a->log(1, text);
      } catch (...) {
      }
    }
  };
  for (se::exec_group *g : c.failed)
    log(*g, "exec_pool: group '" + g->opts().name + "' failed: a C++ exception escaped its pump (" +
                g->pool_link().failure + "); its processes were killed");
  if (c.single_note) {
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(p.m);
      first = !std::exchange(p.single_noted, true);
    }
    if (first)
      log(*c.single_note, "exec_pool: group '" + c.single_note->opts().name +
                              "' is on a single-mode lock manager (state_exec.locks.threading): "
                              "it is pumped on the caller of pump_all only (S1-Q1)");
  }
  {
    std::erase_if(t_failed, [](const auto &f) { return f.first.expired(); }); // pools gone
    auto it = std::find_if(t_failed.begin(), t_failed.end(),
                           [&](const auto &f) { return same_owner(f.first, p.life); });
    if (c.failed.empty()) {
      if (it != t_failed.end())
        t_failed.erase(it);
    } else if (it != t_failed.end()) {
      it->second = c.failed;
    } else {
      t_failed.emplace_back(p.life, c.failed);
    }
  }
  if (c.error)
    std::rethrow_exception(c.error);
}

unsigned exec_pool::threads() const { return impl_->threads; }

pool_stats exec_pool::stats() const {
  std::lock_guard<std::mutex> lock(impl_->m);
  pool_stats s = impl_->stats;
  s.effective_threads = impl_->threads;
  return s;
}

std::vector<state_exec::exec_group *> exec_pool::failed_groups() const {
  for (const auto &[life, groups] : t_failed)
    if (same_owner(life, impl_->life))
      return groups;
  return {};
}

namespace {

void queue_host_call(pool_state &p, worker_ctx &w, std::function<void()> fn, bool waits) {
  std::lock_guard<std::mutex> lock(p.m);
  if (waits)
    w.e->blocked = true; // the worker waits for this body: its group is not being pumped
  w.call->queue.push_back({w.e, waits, std::move(fn)});
  w.call->cv.notify_all();
}

} // namespace

void exec_pool::run_on_host(std::function<void()> fn) {
  if (!fn)
    return;
  worker_ctx *const w = t_worker;
  if (!w || w->pool != this) {
    fn(); // the caller of the pump_all, or no pump_all: a caller never waits for itself
    return;
  }
  queue_host_call(*impl_, *w, std::move(fn), false);
}

bool exec_pool::in_pump_all() const {
  return (t_caller && t_caller->pool == impl_.get()) || (t_worker && t_worker->pool == this);
}

void exec_pool::set_last_pump_for_test(state_exec::exec_group &group, double seconds) {
  std::lock_guard<std::mutex> lock(impl_->m);
  group.pool_link().last_pump_seconds = seconds;
}

const pool_options &exec_pool::options() const { return impl_->opts; }

bool exec_pool::deterministic() const { return impl_->deterministic; }

std::vector<std::string> exec_pool::take_notes() {
  std::lock_guard<std::mutex> lock(impl_->m);
  return std::exchange(impl_->notes, {});
}

namespace detail {

exec_pool *worker_pool() noexcept { return t_worker ? t_worker->pool : nullptr; }

bool defer_on_complete(std::function<void()> callback) {
  if (!t_defer)
    return false;
  t_defer->deferred.push_back(std::move(callback)); // this thread's alone until the join
  return true;
}

bool in_pool_callback() noexcept { return t_callback; }

void purge_host_calls(state_exec::exec_group &group) noexcept {
  try {
    // Held across the wait: the call cannot end (its caller clears this group's membership under
    // it) while this waits on it.
    std::lock_guard<std::mutex> membership(group.pool_link().membership);
    auto *c = static_cast<call_state *>(group.pool_link().call.load(std::memory_order_relaxed));
    if (!c)
      return;
    std::deque<host_call> dropped; // destroyed outside the pool's lock
    std::unique_lock<std::mutex> lk(c->pool->m);
    for (auto it = c->queue.begin(); it != c->queue.end();)
      if (it->e->group == &group) {
        dropped.push_back(std::move(*it));
        it = c->queue.erase(it);
      } else {
        ++it;
      }
    // A body its caller runs now is waited for. (Never this thread's own: a group of a pump_all
    // is not destroyed from that call's bodies or callbacks, ~exec_group aborts first, N3.)
    c->pool->purge_cv.wait(lk, [&] { return c->running_body != &group; });
    lk.unlock();
  } catch (...) {
  }
}

void run_on_host_waiting(exec_pool &pool, std::function<void()> fn) {
  worker_ctx *const w = t_worker;
  if (!w || w->pool != &pool) {
    fn();
    return;
  }
  queue_host_call(*static_cast<pool_state *>(w->call->pool), *w, std::move(fn), true);
}

} // namespace detail

} // namespace cvc

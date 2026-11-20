// The app's pool of execution groups, fork-join (exec_pool.h; docs/STATE_EXEC_THREADED.md 4.1,
// 4.4, 4.9, 4.10, 4.14, 4.17; S1.md S1.6 and S1-Q11).
//
// A pump_all call is a call_state on its caller's stack: its groups, the shared LPT list of the
// `any` groups not yet started, its host-call queue and its accounting. The pool's mutex guards
// every call and the groups' pool records; taking a group from the list under it is the handoff
// edge, and each group's token (its scheduler's pump owner) is taken by exec_group::pump itself.
// Workers are shared by every call; a call's caller services only its own queue.
//
// Continuous mode (Stage 2, S2a; td 4.4, 4.9, 4.14; S2.md S2-Q2..S2-Q6, S2-Q13, S2-Q15): an
// attached group has an attachment record (the notifier's ctx, link.attached), and the same
// mutex (Q in group_pool_link's transitions) guards the ready queue, the workers' next slots, the
// timer heaps, the attaching threads' records (their host queues, ready and failed lists) and the
// facts kept beside the state word. Workers serve both modes: a pump_all wanting help first, then
// the ready queue; idle, they wait on the condition variable while no group is attached for them,
// else spin and park on the eventcount (an epoch and a wakeup each).

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
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/stdlib.h>
#include <deque>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <immintrin.h> // _mm_pause
#endif

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

struct call_state;

// A thread id another thread may read while it is written (the entry's pumper). Movable only for
// building the entries, before any other thread sees them.
struct atomic_thread_id {
  std::atomic<std::thread::id> v{};
  atomic_thread_id() = default;
  atomic_thread_id(atomic_thread_id &&o) noexcept : v(o.v.load(std::memory_order_relaxed)) {}
};

// One group of a pump_all. While the call holds the group, the group's pool_link().call points at
// its entry, so code that has the group (its natives, its on_complete, its teardown) finds the
// call and whether the calling thread pumps it now, with no per-thread state.
struct entry {
  call_state *call = nullptr;
  se::exec_group *group = nullptr;
  atomic_thread_id pumper;         // the thread pumping it now (pump_entry), else none
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

// --- Stage 2: continuous mode ---

struct pool_state;
struct host_thread;
struct attachment;

// A thread waiting for an attached group's pump, or another thread's hold of its token, to end
// (with_group, detach, a teardown): linked on the group's attachment, notified at the release.
struct release_waiter {
  se::detail::wakeup *wake = nullptr;
  attachment *on = nullptr; // null once a detach let it go (the record is freed)
  std::thread::id who;      // the waiting thread
  bool wants_hold = false;  // it waits to hold the group (with_group), not only for a pump's end
  release_waiter *next = nullptr;
};

// An attached group: the notifier's ctx and link.attached. Its fields are the pool's mutex's
// (Q), but `pumper`, which only the thread pumping it compares with itself.
struct attachment {
  pool_state *pool = nullptr;
  se::exec_group *group = nullptr;
  se::detail::group_pool_link *link = nullptr;
  host_thread *host = nullptr; // its attaching thread's record
  bool caller_pumped = false;  // pumped by its attaching thread only (td 4.14, S2-Q4)
  bool long_step_set = false;  // the long-step monitor's default given (S2-Q11)
  std::uint64_t seq = 0;       // creation order: the deterministic rotation, and ties
  // The facts beside the state word (exec_group.h's transitions):
  bool queued = false;    // in the ready queue, a next slot or its host's ready list
  bool host_held = false; // a host thread holds its token (with_group, a host body)
  std::thread::id holder; // that thread
  bool deferred = false;  // popped while host_held: queued again at the hold's end
  bool handed = false;    // host_held was set for `holder` by a release, not yet taken up
  bool failed = false;    // on its host's failed list
  std::thread::id detacher; // the thread finishing its detach, once detaching is set
  bool blocked = false;   // its worker waits for a host body it queued (run_on_host_waiting)
  clock_type::time_point blocked_since{};
  clock_type::time_point wait_noted{}; // when a long host wait was last logged (once a minute)
  std::size_t entries = 0;             // its entries on its host's queue
  double vtime = 0.0;                  // pump seconds consumed (S2-Q13's clamp at a wake)
  std::uint64_t generation = 0;        // a timer entry of an older generation is stale
  std::size_t filed = 0;               // its entries in a timer heap, stale ones included
  bool on_worker = false;              // pumped now by a worker, not its attaching thread
  release_waiter *waiters = nullptr;
  std::atomic<std::thread::id> pumper{}; // the thread pumping it now
};

// One entry of an attaching thread's host queue (S2-Q5): a host_proxied body or a deferred
// on_complete callback of one of its worker-pumped groups.
struct host_entry {
  attachment *a = nullptr;
  bool waits = false; // its worker blocks until it ran (run_on_host_waiting)
  std::function<void()> fn;
};

// A deadline a pump published, filed once its group went idle (the release's CAS to 0).
struct timer_entry {
  clock_type::time_point at;
  bool precise = false;
  attachment *a = nullptr;
  std::uint64_t generation = 0;
};
struct timer_later {
  bool operator()(const timer_entry &x, const timer_entry &y) const { return x.at > y.at; }
};

// A thread that attached groups (S2-Q5), found by its id (nothing is kept per thread): its host
// queue, its caller-pumped groups' ready list and timers, its failed groups, and its wakeup.
struct host_thread {
  std::thread::id id;
  se::detail::wakeup wake;
  std::deque<host_entry> queue;
  std::vector<attachment *> ready;
  std::vector<attachment *> failed;
  std::vector<timer_entry> timers; // a heap, earliest first
  unsigned attached = 0;
  bool in_callback = false;   // it runs a host entry now (in_pool_callback)
  double vmin = 0.0;          // its ready list's virtual-time floor
  std::uint64_t last_seq = 0; // the deterministic rotation's last group
};

// A worker's continuous-mode state.
struct worker_rec {
  int index = 0;
  bool p_core = true;
  std::thread::id id; // set as it starts: which thread a wake comes from
  se::detail::wakeup wake;
  bool parked = false;           // on the eventcount (in pool_state::parked)
  bool signalled = false;        // a waker took it off the parked list and owes it a notify
  bool rearm = false;            // woken to wait for the timer heap's new earliest deadline
  bool stale = false;            // an owed notify outlived its wait: the next wait returns at once
  attachment *current = nullptr; // the group it pumps now
  attachment *next = nullptr;    // its next slot (Go's runnext)
  int next_streak = 0;           // next-slot pumps in a row (Tokio's valve: 3)
};

// The pool's state (exec_pool::impl is this): its workers, the calls in progress, its counters.
struct pool_state {
  exec_pool *self = nullptr;
  pool_options opts;
  unsigned threads = 0; // workers started
  bool deterministic = false;
  bool force_move = false;

  std::mutex m;
  std::condition_variable work_cv;  // workers: a call wants a helper, or stop
  std::condition_variable purge_cv; // a host body a caller ran has ended
  // The threaded pump_all calls in progress, until each has run its deferred callbacks.
  std::vector<call_state *> calls;
  // The threads in a pump_all of this pool now, as its caller, threaded or not (in_pump_all).
  std::vector<std::thread::id> callers;
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

  // Stage 2 (continuous mode), under m:
  std::vector<std::unique_ptr<worker_rec>> wrecs; // one per worker, made before they start
  std::unordered_map<const attachment *, std::unique_ptr<attachment>> attachments;
  std::vector<std::unique_ptr<host_thread>> hosts;
  std::vector<attachment *> ready;  // the workers' ready queue: a heap by virtual time
  std::vector<timer_entry> timers;  // the workers' groups' deadlines: a heap, earliest first
  std::vector<worker_rec *> parked; // workers parked on the eventcount
  unsigned attached_any = 0;        // attached groups the workers pump
  unsigned cv_waiters = 0;          // workers waiting on work_cv
  unsigned spinning = 0;            // workers in the eventcount's spin, watching the epoch
  unsigned owed = 0;                // workers taken off the parked list, not yet back under Q
  worker_rec *keeper = nullptr;     // the parked worker that waits for the timer heap's top
  clock_type::time_point keeper_until{};
  double vmin = 0.0;              // the ready queue's virtual-time floor: the last group popped
  std::uint64_t attach_order = 0; // orders the groups that have no creation order (no app)
  // Bumped whenever work is queued: a spinning worker watches it without the mutex (S2-Q3).
  std::atomic<std::uint64_t> epoch{0};
  // CVC_EXEC_FUZZ, read when the pool is built (S2-Q15): the lock manager's point helper.
  se::detail::lock_fuzz fuzz;

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
  // Its caller runs a queued host body or a deferred on_complete now (in_pool_callback).
  std::atomic<bool> in_callback{false};
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
// throws: an exception out of pump() (an inline on_complete's) is returned. While it pumps, `e`
// names this thread its pumper: the group's natives and callbacks find it through the group.
// The first time a pool with workers pumps a group (a pump_all's or continuously; a group no pool
// pumped before), its long-step monitor's default is 1 ms (S2-Q11; the tree's
// state_exec.long_step_ms wins), set holding its token.
void give_long_step_default(se::exec_group &g) {
  se::async_scheduler::pump_owner_scope token(g.sched());
  g.sched().set_long_step_default(1e-3);
}

pumped pump_entry(call_state &c, entry &e) noexcept {
  pumped out;
  e.pumper.v.store(std::this_thread::get_id(), std::memory_order_relaxed);
  const clock_type::time_point start = clock_type::now();
  try {
    if (c.pool->threads > 0 && e.group->pool_link().last_thread < 0)
      give_long_step_default(*e.group);
    se::pump_budget budget = c.budget;
    if (c.timed)
      budget.seconds = std::min(budget.seconds, seconds_left(c, start));
    else
      budget.seconds = std::numeric_limits<double>::infinity(); // deterministic: steps only
    out.failed = e.group->pump(budget).failed;
  } catch (...) {
    out.error = std::current_exception();
  }
  e.pumper.v.store(std::thread::id(), std::memory_order_relaxed);
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

// Under the pool's mutex: take a parked worker off the eventcount for work, a P-core one first,
// the timer keeper last, and one that did not pump `avoid` last when it is given
// (CVC_EXEC_FORCE_MOVE); it is owed a notify. Null when none is parked.
worker_rec *unpark_one(pool_state &p, const attachment *avoid = nullptr) {
  worker_rec *best = nullptr;
  int best_score = -1;
  for (worker_rec *w : p.parked) {
    const int score = (w->p_core ? 4 : 0) + (w != p.keeper ? 2 : 0) +
                      (!avoid || avoid->link->last_thread != w->index ? 1 : 0);
    if (score > best_score) {
      best = w;
      best_score = score;
    }
  }
  if (!best)
    return nullptr;
  p.parked.erase(std::find(p.parked.begin(), p.parked.end(), best));
  best->parked = false;
  best->signalled = true;
  ++p.owed;
  return best;
}

// Under the pool's mutex: tell one idle worker there is work (a pump_all wanting help). Workers
// wait on work_cv while no group is attached for them (Stage 1's wait, unchanged), else park on
// the eventcount (S2-Q3); with no worker parked this is Stage 1's notify_one.
void notify_idle_worker(pool_state &p) {
  p.epoch.fetch_add(1, std::memory_order_release);
  if (p.cv_waiters > 0 || p.parked.empty())
    p.work_cv.notify_one();
  else if (worker_rec *w = unpark_one(p))
    w->wake.notify();
}

// Under the pool's mutex: tell every idle worker (CVC_EXEC_FORCE_MOVE's helpers, the stop).
void notify_all_workers(pool_state &p) {
  p.epoch.fetch_add(1, std::memory_order_release);
  p.work_cv.notify_all();
  while (worker_rec *w = unpark_one(p))
    w->wake.notify();
}

// Under the pool's mutex: wake one more helper for `c`. A wake is counted when it is made, so
// stats().helper_wakes does not depend on which thread gets to the work first (GR-9). Each wake
// is answered on its own call (helpers_pending is per call), by whichever idle worker comes.
void wake_helper(pool_state &p, call_state &c) {
  --c.wakes_left;
  ++c.helpers_pending;
  ++p.stats.helper_wakes;
  notify_idle_worker(p);
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

// --- Stage 2: the state word, the ready queue, wake(g) and the release (S2a.2) ---

using word_t = std::uint32_t;
constexpr word_t kScheduled = se::detail::group_pool_link::scheduled;
constexpr word_t kNotified = se::detail::group_pool_link::notified;
constexpr word_t kPumping = se::detail::group_pool_link::pumping;
constexpr word_t kDetaching = se::detail::group_pool_link::detaching;

// CVC_EXEC_FUZZ's pool points (S2-Q15), drawn through the lock manager's helper as points of
// their own (lock_fuzz_at has a fixed underlying type, so these are valid values of it).
enum class pool_point : std::uint8_t {
  wake_or = 16,  // wake(g): after the fetch_or, before Q
  pop_to_pump,   // between a pop (pumping set) and the pump
  release_cas,   // the release, under Q, before its CAS
  park_commit,   // the eventcount, between the prepare and the commit
  hold_to_token, // a host hold, between setting host_held and taking the token
};
void fuzz_point(pool_state &p, pool_point at) noexcept {
  se::detail::lock_fuzz_point(&p.fuzz, static_cast<se::detail::lock_fuzz_at>(at));
}

std::atomic<word_t> &word(attachment &a) { return a.link->state; }

// The ready order: the least virtual time first, then creation order.
struct vtime_later {
  bool operator()(const attachment *x, const attachment *y) const {
    return x->vtime != y->vtime ? x->vtime > y->vtime : x->seq > y->seq;
  }
};

// The clamp's slack, one pump (S2-Q13); 2 ms when pumps have no time limit.
double pump_slack(const pool_state &p) {
  const double s = p.opts.pump_seconds;
  return std::isfinite(s) && s > 0.0 ? s : 2e-3;
}

// Under Q: the worker whose thread `id` is, or null (no thread_local: compared with the ids the
// workers recorded as they started).
worker_rec *worker_of(pool_state &p, std::thread::id id) {
  for (const std::unique_ptr<worker_rec> &w : p.wrecs)
    if (w->id == id)
      return w.get();
  return nullptr;
}

// Under Q: the attaching thread `id`'s record, or null.
host_thread *host_of(pool_state &p, std::thread::id id) {
  for (const std::unique_ptr<host_thread> &h : p.hosts)
    if (h->id == id)
      return h.get();
  return nullptr;
}

// Under Q: `group`'s attachment when it is attached to `p`, else null. Its pointer is compared
// with p's records, never followed first (another pool's may be gone).
attachment *attachment_in(pool_state &p, se::exec_group &group) {
  auto *a = static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire));
  return a && p.attachments.count(a) ? a : nullptr;
}

// Under Q: onto the ready queue, whose room was reserved at attach (it never allocates).
void push_ready(pool_state &p, attachment &a) {
  p.ready.push_back(&a);
  std::push_heap(p.ready.begin(), p.ready.end(), vtime_later{});
}

// Who to tell once something is queued: a parked worker, one waiting on work_cv, or an
// attaching thread.
struct wake_target {
  worker_rec *worker = nullptr;
  bool cv = false;
  host_thread *host = nullptr;
  void notify(pool_state &p) const {
    if (worker)
      worker->wake.notify();
    else if (cv)
      p.work_cv.notify_one();
    else if (host)
      host->wake.notify();
  }
};

// Under Q: an idle worker for `a`, just queued for the workers, the epoch moved (one that did not
// pump it last, under CVC_EXEC_FORCE_MOVE). None while a worker spins: it sees the epoch and pops
// it (Tokio's searching workers), sparing a parked worker's wake; under CVC_EXEC_FORCE_MOVE only
// when the spinner cannot be the thread that pumped `a` last (`from_releaser`: its releasing worker
// queued it, and that worker is not spinning).
wake_target idle_worker_for(pool_state &p, const attachment *a, bool from_releaser = false) {
  wake_target t;
  if (p.spinning > 0 && (!p.force_move || from_releaser))
    return t;
  if (p.force_move && a && p.cv_waiters > 0 &&
      std::none_of(p.parked.begin(), p.parked.end(),
                   [&](const worker_rec *w) { return w->index != a->link->last_thread; })) {
    t.cv = true; // every parked worker pumped `a` last: one waiting on work_cv takes it
    return t;
  }
  if (worker_rec *w = unpark_one(p, p.force_move ? a : nullptr))
    t.worker = w;
  else if (p.cv_waiters > 0)
    t.cv = true;
  return t;
}

// Under Q: queue `a`, which just took its `scheduled` transition (a wake, a due timer, a hold's
// end), its virtual time clamped to at least the queue's floor less one pump (S2-Q13), its filed
// deadline made stale. A caller-pumped group goes onto its attaching thread's ready list. A
// worker group woken by worker `from` while `from` pumps another group goes into `from`'s next
// slot (Go's runnext, td 4.4), whose occupant goes to the queue; else onto the ready queue. With
// `tell` false no idle worker is taken for it (the calling worker pumps it next).
wake_target queue_woken(pool_state &p, attachment &a, worker_rec *from, bool tell = true) {
  ++a.generation;
  a.queued = true;
  wake_target t;
  if (a.caller_pumped) {
    host_thread &h = *a.host;
    a.vtime = std::max(a.vtime, h.vmin - pump_slack(p));
    h.ready.push_back(&a);
    t.host = &h;
    return t;
  }
  a.vtime = std::max(a.vtime, p.vmin - pump_slack(p));
  p.epoch.fetch_add(1, std::memory_order_release);
  if (from && !p.force_move && from->current && from->current != &a) {
    attachment *displaced = std::exchange(from->next, &a);
    if (!displaced)
      return t; // its waker takes it after its pump
    push_ready(p, *displaced);
    return idle_worker_for(p, displaced);
  }
  push_ready(p, a);
  return tell ? idle_worker_for(p, &a) : t;
}

// wake(g), the notifier (exec_group.h's transitions; S2-Q2): on the posting thread, under its
// lock and the notifier lock. It takes only Q and notifies one wakeup, outside Q; it never
// allocates (every list it pushes onto has room reserved at attach) and never throws.
void wake(attachment &a) noexcept {
  pool_state &p = *a.pool;
  const word_t old = word(a).fetch_or(kScheduled | kNotified, std::memory_order_acq_rel);
  if (old & (kScheduled | kDetaching))
    return;
  fuzz_point(p, pool_point::wake_or);
  wake_target t;
  {
    std::lock_guard<std::mutex> lock(p.m);
    if (word(a).load(std::memory_order_relaxed) & kDetaching)
      return;
    t = queue_woken(p, a, a.caller_pumped ? nullptr : worker_of(p, std::this_thread::get_id()));
    ++p.stats.wakes;
  }
  t.notify(p);
}

void wake_notifier(void *ctx) noexcept { wake(*static_cast<attachment *>(ctx)); }

// Under Q: wake(g)'s transition, for a due timer or a host hold's end; the caller notifies `t` (a
// worker taken off the parked list is owed it).
bool wake_locked(pool_state &p, attachment &a, wake_target &t, bool tell = true) {
  const word_t old = word(a).fetch_or(kScheduled | kNotified, std::memory_order_acq_rel);
  if (old & (kScheduled | kDetaching))
    return false;
  t = queue_woken(p, a, nullptr, tell);
  return true;
}

// Under Q: queue `a` again, still scheduled (a release that found work, a hold's end). The
// releasing worker pops next, unless a pump_all wants its help first; then, as when it is forced
// elsewhere (CVC_EXEC_FORCE_MOVE) or for a hold's end, an idle worker (or the attaching thread)
// is told.
wake_target requeue(pool_state &p, attachment &a, const worker_rec *releaser) {
  ++p.stats.requeues;
  a.queued = true;
  wake_target t;
  if (a.caller_pumped) {
    a.host->ready.push_back(&a);
    t.host = a.host;
    return t;
  }
  push_ready(p, a);
  if (!releaser || !p.calls.empty() || (p.force_move && p.threads > 1)) {
    // Work for another worker: the epoch moves (a spinning or parking worker looks again). A
    // requeue its releaser takes next moves nothing, so idle workers park beside a busy group.
    p.epoch.fetch_add(1, std::memory_order_release);
    t = idle_worker_for(p, &a, releaser != nullptr);
  }
  return t;
}

// Under Q: pop's checks on `a`, just taken from a queue: dropped while detaching; left notified
// and deferred while a host holds its token; else it is pumping now, by the calling thread (pop
// and pumping are one step under Q). True when claimed.
bool claim(attachment &a) {
  const word_t w = word(a).load(std::memory_order_acquire);
  if (w & kDetaching)
    return false;
  if (a.host_held) {
    word(a).fetch_or(kNotified, std::memory_order_acq_rel);
    a.deferred = true;
    return false;
  }
  word(a).fetch_or(kPumping, std::memory_order_acq_rel);
  a.pumper.store(std::this_thread::get_id(), std::memory_order_relaxed);
  return true;
}

// Under Q: the next group of the ready queue for worker `w`, the least virtual time first; under
// CVC_EXEC_FORCE_MOVE (with another worker to take it) the first one `w` did not pump last, found
// by popping at most 4 it did pump last and pushing them back (O(log n) each; a scan only past
// them), and `skipped` says one was left for another worker. Never allocates.
attachment *take_ready(pool_state &p, const worker_rec &w, bool &skipped) {
  const bool moving = p.force_move && p.threads > 1;
  attachment *left[4];
  std::size_t n = 0;
  attachment *got = nullptr;
  while (!p.ready.empty()) {
    std::pop_heap(p.ready.begin(), p.ready.end(), vtime_later{});
    attachment *a = p.ready.back();
    p.ready.pop_back();
    if (!moving || a->link->last_thread != w.index) {
      got = a;
      break;
    }
    left[n++] = a;
    if (n == 4) {
      for (std::size_t i = 0; i < p.ready.size() && !got; ++i)
        if (p.ready[i]->link->last_thread != w.index) {
          got = p.ready[i];
          p.ready.erase(p.ready.begin() + static_cast<std::ptrdiff_t>(i));
          std::make_heap(p.ready.begin(), p.ready.end(), vtime_later{});
        }
      break;
    }
  }
  for (std::size_t i = 0; i < n; ++i)
    push_ready(p, *left[i]); // back where they were: the room is theirs
  skipped = n > 0;
  return got;
}

// Under Q: another worker's next slot, for a worker that found nothing else, so a slot whose
// owner pumps long is not left waiting.
attachment *steal_next(pool_state &p, const worker_rec &w) {
  const bool moving = p.force_move && p.threads > 1;
  for (const std::unique_ptr<worker_rec> &o : p.wrecs)
    if (o.get() != &w && o->next && (!moving || o->next->link->last_thread != w.index))
      return std::exchange(o->next, nullptr);
  return nullptr;
}

// Under Q: drop the stale entries on top of a timer heap (their groups were woken since). Every
// entry's attachment is alive: a detach removes its entries.
void drop_stale(std::vector<timer_entry> &heap) {
  while (!heap.empty() && heap.front().generation != heap.front().a->generation) {
    --heap.front().a->filed;
    std::pop_heap(heap.begin(), heap.end(), timer_later{});
    heap.pop_back();
  }
}

// Under Q: queue the groups whose filed deadline has come (td 4.4's timers; S2-Q4's due timer),
// as wake(g) would (stats().timer_wakes). With `by_worker`, an idle worker is told of each but the
// first, which the calling worker pumps.
void fire_due(pool_state &p, std::vector<timer_entry> &heap, clock_type::time_point now,
              bool by_worker) {
  unsigned fired = 0;
  for (;;) {
    drop_stale(heap);
    if (heap.empty() || heap.front().at > now)
      return;
    attachment &a = *heap.front().a;
    --a.filed;
    std::pop_heap(heap.begin(), heap.end(), timer_later{});
    heap.pop_back();
    wake_target t;
    if (!wake_locked(p, a, t, !by_worker || fired > 0))
      continue;
    ++p.stats.timer_wakes;
    ++fired;
    t.notify(p); // every worker unparked is notified; an attaching thread's own heap: itself
  }
}

// Under Q, after the release's CAS to 0 (and only then): file the deadline the pump published.
void file_deadline(pool_state &p, attachment &a, clock_type::time_point at, bool precise) {
  std::vector<timer_entry> &heap = a.caller_pumped ? a.host->timers : p.timers;
  ++a.generation;
  heap.push_back({at, precise, &a, a.generation});
  ++a.filed;
  std::push_heap(heap.begin(), heap.end(), timer_later{});
}

// Under Q: pop for worker `w` (exec_group.h's pop): the due timers first, then its next slot
// (unless it took it 3 times in a row: Tokio's valve sends it to the queue), the ready queue,
// another worker's slot. Null when nothing is there for it.
attachment *pop_for_worker(pool_state &p, worker_rec &w, bool &skipped) {
  fire_due(p, p.timers, clock_type::now(), true);
  for (;;) {
    attachment *a = nullptr;
    if (w.next && w.next_streak < 3) {
      a = std::exchange(w.next, nullptr);
      ++w.next_streak;
    } else {
      if (w.next)
        push_ready(p, *std::exchange(w.next, nullptr));
      w.next_streak = 0;
      a = take_ready(p, w, skipped);
      if (!a)
        a = steal_next(p, w);
    }
    if (!a)
      return nullptr;
    a->queued = false;
    if (!claim(*a))
      continue;
    p.vmin = std::max(p.vmin, a->vtime);
    a->on_worker = true;
    w.current = a;
    return a;
  }
}

// Under Q: notify the threads waiting for `a`'s pump or hold to end.
void notify_waiters(attachment &a) {
  for (release_waiter *r = a.waiters; r; r = r->next)
    r->wake->notify();
}

// Under Q, as a release queues `a` again: when a host waits to hold it (a with_group, else its
// attaching thread with host entries for it), set host_held for that host now, on its behalf, as
// its own look would once pumping cleared; else the worker that queues it pops it again in the
// same critical section, and a host never finds it free. The pop that follows leaves it notified
// and deferred (exec_group.h's pop), and the hold's end queues it again.
void hand_hold_over(attachment &a) {
  std::thread::id to;
  for (release_waiter *r = a.waiters; r && to == std::thread::id(); r = r->next)
    if (r->wants_hold)
      to = r->who;
  if (to == std::thread::id() && a.entries > 0)
    to = a.host->id;
  if (to == std::thread::id())
    return;
  a.host_held = true;
  a.holder = to;
  a.handed = true;
}

// Under Q, as detaching is set: a hold handed over and not taken up goes (its host then finds
// detaching set, exec_group.h's host transition).
void revoke_handed_hold(attachment &a) {
  if (!a.handed)
    return;
  a.handed = false;
  a.host_held = false;
  a.deferred = false;
  notify_waiters(a);
}

// What one continuous pump did.
struct pump_outcome {
  se::pump_result r;
  double seconds = 0.0;
  std::exception_ptr error; // a host exception out of the pump (an inline on_complete's)
};

// Pump `a` once on the calling thread, which claimed it (no lock held; exec_group.h's pump):
// clear notified, then the pool's budget (deterministic mode: steps only), the token taken in
// pump(). A failed pump's processes are killed here, holding the token. Never throws.
pump_outcome pump_attached(pool_state &p, attachment &a) noexcept {
  pump_outcome out;
  fuzz_point(p, pool_point::pop_to_pump);
  word(a).fetch_and(~kNotified, std::memory_order_acq_rel);
  se::exec_group &g = *a.group;
  const clock_type::time_point start = clock_type::now();
  try {
    if (p.threads > 0 && !a.long_step_set) { // the pumping thread's alone while it pumps
      a.long_step_set = true;
      give_long_step_default(g);
    }
    const double seconds =
        p.deterministic ? std::numeric_limits<double>::infinity() : p.opts.pump_seconds;
    out.r = g.pump(se::pump_budget{p.opts.pump_steps, seconds});
  } catch (...) {
    out.error = std::current_exception();
  }
  out.seconds = std::chrono::duration<double>(clock_type::now() - start).count();
  if (out.r.failed)
    kill_failed_group(g);
  return out;
}

// Under Q: the release (exec_group.h's transitions) by the thread that pumped `a`: a worker `w`,
// or its attaching thread (null; thread index 0). It reads only `out` and the word: a failed pump
// takes the failure row; work left (runnable, a deadline due, or a host exception that cut the
// pump short) queues it again; else the CAS from exactly `scheduled` to 0, and only if that
// succeeds the deadline is filed; a post since (notified) queues it again.
void release(pool_state &p, attachment &a, const pump_outcome &out, worker_rec *w) noexcept {
  const int thread = w ? w->index : 0;
  word(a).fetch_and(~kPumping, std::memory_order_acq_rel);
  a.pumper.store(std::thread::id(), std::memory_order_relaxed);
  a.on_worker = false;
  if (w)
    w->current = nullptr;
  se::detail::group_pool_link &link = *a.link;
  ++p.stats.pumps;
  ++(w ? p.stats.worker_pumps : p.stats.host_pumps);
  if (link.last_thread >= 0 && link.last_thread != thread)
    ++p.stats.moves;
  link.last_thread = thread;
  link.last_pump_seconds = out.seconds;
  a.vtime += out.seconds;
  notify_waiters(a);
  if (a.entries > 0)
    a.host->wake.notify(); // its host entries may run now
  if (word(a).load(std::memory_order_acquire) & kDetaching)
    return; // its detach or teardown waits for this
  if (out.r.failed) {
    word(a).fetch_or(kDetaching, std::memory_order_acq_rel);
    a.detacher = a.host->id; // take_failed (or the teardown) finishes it there
    a.failed = true;
    a.host->failed.push_back(&a); // room reserved at attach
    a.host->wake.notify();
    return;
  }
  const clock_type::time_point now = clock_type::now();
  if (!out.error && !out.r.runnable && out.r.next_deadline > now) {
    fuzz_point(p, pool_point::release_cas);
    word_t expected = kScheduled;
    if (word(a).compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
      if (out.r.next_deadline == clock_type::time_point::max())
        return;
      try {
        file_deadline(p, a, out.r.next_deadline, out.r.deadline_precise);
      } catch (...) {
        wake_target t; // no room for its deadline: queue it again, as a wake would
        if (wake_locked(p, a, t))
          t.notify(p);
      }
      return;
    }
  }
  hand_hold_over(a);
  const wake_target t = requeue(p, a, w);
  if (w)
    t.notify(p);
}

// --- Stage 2: parking (S2a.4; S2-Q3) ---

// Under Q: when the workers' timer heap has a top no parked worker waits for (no keeper, or the
// keeper waits for a later time), make one re-arm its wait: one worker waits for the timers, so
// a lone sleeper wakes one worker once (PL-4).
void hand_off_timers(pool_state &p) {
  drop_stale(p.timers);
  if (p.timers.empty() || (p.keeper && p.keeper_until <= p.timers.front().at))
    return;
  worker_rec *k = p.keeper && p.keeper->parked ? p.keeper : nullptr;
  if (k) {
    p.parked.erase(std::find(p.parked.begin(), p.parked.end(), k));
    k->parked = false;
    k->signalled = true;
    ++p.owed;
  } else {
    k = unpark_one(p);
  }
  if (!k)
    return; // every worker is busy: they look at the heap at each pop
  k->rearm = true;
  p.keeper = k;
  p.keeper_until = p.timers.front().at;
  k->wake.notify();
}

void cpu_relax() noexcept {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
  _mm_pause();
#elif defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#endif
}

void pool_state::worker_main(int index, bool p_core, bool place) {
  // No exception leaves a worker (td 4.17): a pump's is caught in pump_entry (pump_attached), and
  // they and finish_group (release) do not throw, so a group taken is always finished; anything
  // else ends the worker.
  try {
    detail::apply_process_class_here();
    if (place)
      place_current_thread(opts.placement, static_cast<unsigned>(index - 1));
  } catch (...) {
  }
  const bool from_back = !p_core && opts.placement == cpu_set::all; // td 4.4: E-cores from the back
  worker_rec &w = *wrecs[static_cast<std::size_t>(index - 1)];
  try {
    std::unique_lock<std::mutex> lk(m);
    w.id = std::this_thread::get_id();
    bool woke = false; // back from a park nothing explains: an empty wake if it finds nothing
    bool spun = false; // spun since it last found work
    for (;;) {
      if (stopping)
        break;
      if (call_state *c = call_wanting_help(*this, index)) {
        woke = spun = false;
        --c->helpers_pending;
        // Help while the call has work; the woken helper wakes the next while work remains (td
        // 4.4).
        while (const std::optional<std::size_t> idx = take_group(*this, *c, index, from_back)) {
          if (c->remaining() > 0 && c->wakes_left > 0)
            wake_helper(*this, *c);
          entry &e = c->entries[*idx];
          lk.unlock();
          const pumped r = pump_entry(*c, e);
          lk.lock();
          finish_group(*this, *c, e, index, r, true);
        }
        continue;
      }
      if (attached_any == 0 && ready.empty() && !w.next) {
        // No group attached for the workers: Stage 1's wait, for a pump_all or the stop.
        woke = spun = false;
        ++cv_waiters;
        work_cv.wait(lk);
        --cv_waiters;
        continue;
      }
      // Continuous mode (S2-Q4): pop, pump, release.
      bool skipped = false;
      if (attachment *a = pop_for_worker(*this, w, skipped)) {
        woke = spun = false;
        hand_off_timers(*this);
        if (!ready.empty()) // more work queued: another idle worker (none while one spins)
          idle_worker_for(*this, nullptr).notify(*this);
        lk.unlock();
        const pump_outcome out = pump_attached(*this, *a);
        lk.lock();
        release(*this, *a, out, &w);
        continue;
      }
      // CVC_EXEC_FORCE_MOVE left groups this worker pumped last: unless a worker is already on its
      // way (woken, or spinning), wake one for them (a timer this worker fired, say).
      if (skipped && !ready.empty() && owed == 0 && spinning == 0)
        idle_worker_for(*this, nullptr).notify(*this);
      if (std::exchange(woke, false))
        ++stats.empty_wakes;
      if (!spun) {
        // S2-Q3: spin about 20 us watching the epoch, without the mutex, before parking.
        spun = true;
        const std::uint64_t seen = epoch.load(std::memory_order_acquire);
        ++spinning; // a waker spares a parked worker meanwhile, moving the epoch
        lk.unlock();
        const clock_type::time_point until = clock_type::now() + std::chrono::microseconds(20);
        while (epoch.load(std::memory_order_acquire) == seen && clock_type::now() < until)
          cpu_relax();
        lk.lock();
        --spinning;
        continue;
      }
      spun = false;
      // The eventcount (S2-Q3): prepare under Q (registered as parked, the queue and the heap
      // looked at above under the same lock, so a wake either was seen or will notify this
      // worker), then commit: wait until a notify or the timer heap's top, if it keeps it.
      w.parked = true;
      parked.push_back(&w); // room reserved: one per worker
      ++stats.parks;
      clock_type::time_point until = clock_type::time_point::max();
      bool precise = false;
      drop_stale(timers);
      if (!timers.empty() && (!keeper || keeper == &w || timers.front().at < keeper_until)) {
        keeper = &w;
        keeper_until = until = timers.front().at;
        precise = timers.front().precise; // a sleeper's or a timed wait's: kept precisely
      }
      const std::uint64_t seen = epoch.load(std::memory_order_acquire);
      lk.unlock();
      fuzz_point(*this, pool_point::park_commit);
      bool waited = false, notified = false;
      if (epoch.load(std::memory_order_acquire) == seen) {
        waited = true;
        notified = w.wake.wait_until(until, precise);
      }
      lk.lock();
      if (w.parked) {
        parked.erase(std::find(parked.begin(), parked.end(), &w));
        w.parked = false;
      }
      if (keeper == &w)
        keeper = nullptr;
      bool explained = std::exchange(w.rearm, false);
      spun = explained; // a re-armed wait parks again at once
      if (w.signalled) {
        // A waker took it off the parked list: its notify was consumed now, or lands later.
        w.signalled = false;
        --owed;
        w.stale = !notified;
      } else if (notified && w.stale) {
        w.stale = false; // an earlier waker's late notify
        explained = true;
      }
      woke = waited && !explained;
    }
  } catch (...) {
  }
}

// Run `fn` (a queued host body or a deferred on_complete) for `group` on the caller (no other
// thread pumps the group meanwhile; a host body runs in a context with the group's registry and
// stream), and keep its first exception in `c`.
void run_for_group(call_state &c, se::exec_group &group, std::function<void()> &fn,
                   std::unique_lock<std::mutex> *lk) noexcept {
  struct in_callback_scope {
    std::atomic<bool> &flag;
    bool outer;
    explicit in_callback_scope(std::atomic<bool> &f) : flag(f), outer(f.exchange(true)) {}
    ~in_callback_scope() { flag.store(outer); }
  } in_callback(c.in_callback);
  (void)group;
  try {
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

// The entry of the pump_all holding `group` now, or null.
entry *entry_of(se::exec_group &group) noexcept {
  return static_cast<entry *>(group.pool_link().call.load(std::memory_order_relaxed));
}

// The entry of `group` when the calling thread is a worker pumping it now (not the caller of its
// pump_all, which never waits for itself), or null.
entry *pumped_on_this_worker(se::exec_group &group) noexcept {
  entry *e = entry_of(group);
  const std::thread::id self = std::this_thread::get_id();
  if (!e || e->pumper.v.load(std::memory_order_relaxed) != self || e->call->caller == self)
    return nullptr;
  return e;
}

// Under the pool's mutex: whether the calling thread is the caller of one of `p`'s threaded calls
// and runs a host body or a deferred callback of it now.
bool caller_in_callback_locked(const pool_state &p) {
  const std::thread::id self = std::this_thread::get_id();
  for (const call_state *c : p.calls)
    if (c->caller == self && c->in_callback.load(std::memory_order_relaxed))
      return true;
  return false;
}

void queue_host_call(call_state &c, entry &e, std::function<void()> fn, bool waits) {
  std::lock_guard<std::mutex> lock(c.pool->m);
  if (waits)
    e.blocked = true; // the worker waits for this body: its group is not being pumped
  c.queue.push_back({&e, waits, std::move(fn)});
  c.cv.notify_all();
}

// --- Stage 2: the attaching threads (S2a.4; S2-Q4, S2-Q5) ---

// Log `text` through `g`'s app (interruption held off), as pump_all logs a failure.
void log_for(se::exec_group &g, const std::string &text) noexcept {
  if (cvc::app *a = g.opts().app) {
    try {
      boost::this_thread::disable_interruption hold;
      a->log(1, text);
    } catch (...) {
    }
  }
}

// The attachment of `group` when the calling thread is a worker pumping it now (so the record
// lives while it is used), else null.
attachment *attached_on_this_worker(se::exec_group &group) noexcept {
  auto *a = static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire));
  if (!a || a->pumper.load(std::memory_order_relaxed) != std::this_thread::get_id() ||
      !a->on_worker)
    return nullptr;
  return a;
}

// A worker pumping `a`: queue `fn` on its attaching thread's host queue (S2-Q5). `waits`: the
// worker then blocks until it ran (run_on_host_waiting), its group blocked meanwhile.
void queue_host_entry(attachment &a, std::function<void()> fn, bool waits) {
  pool_state &p = *a.pool;
  {
    std::lock_guard<std::mutex> lock(p.m);
    a.host->queue.push_back({&a, waits, std::move(fn)});
    ++a.entries;
    if (waits) {
      a.blocked = true;
      a.blocked_since = clock_type::now();
    }
  }
  a.host->wake.notify();
}

// Under Q, for the scope's life (made and destroyed under Q): the calling thread waits on `wake`
// for `a`'s pump or hold to end. A detach that frees `a` orphans the link first.
struct waiter_link {
  release_waiter node;
  waiter_link(attachment &a, se::detail::wakeup &wake, bool wants_hold) {
    node.wake = &wake;
    node.on = &a;
    node.who = std::this_thread::get_id();
    node.wants_hold = wants_hold;
    node.next = a.waiters;
    a.waiters = &node;
  }
  ~waiter_link() {
    if (!node.on)
      return;
    release_waiter **at = &node.on->waiters;
    while (*at != &node)
      at = &(*at)->next;
    *at = node.next;
  }
  waiter_link(const waiter_link &) = delete;
  waiter_link &operator=(const waiter_link &) = delete;
};

// Under Q: the next entry of `h`'s queue that may run now, and whether it takes its group's token
// (exec_group.h's host transition). A waiting body (run_on_host_waiting) runs at once under its
// blocked worker's token, taking nothing. Any other entry takes the group's token, so it waits
// while the group pumps (its worker blocked or not), is held by another thread (a hold handed to
// this thread is taken up), or is being detached (detach runs it, a teardown drops it); one
// group's such entries run in their order.
std::optional<std::size_t> runnable_entry(const host_thread &h, bool &needs_token) {
  for (std::size_t i = 0; i < h.queue.size(); ++i) {
    const host_entry &e = h.queue[i];
    const attachment *a = e.a;
    if (e.waits) {
      if (a->blocked) {
        needs_token = false;
        return i;
      }
      continue;
    }
    bool earlier = false;
    for (std::size_t j = 0; j < i && !earlier; ++j)
      earlier = h.queue[j].a == a && !h.queue[j].waits;
    if (earlier)
      continue;
    if (a->link->state.load(std::memory_order_acquire) & (kPumping | kDetaching))
      continue;
    if (a->host_held && !(a->handed && a->holder == h.id))
      continue;
    needs_token = true;
    return i;
  }
  return std::nullopt;
}

// Under Q (`lk` locked on entry and return): run one entry of `h`'s queue that may run now, on
// the calling thread (h's), keeping the first exception in `err`; false when none may. A body
// binds its group's registry and stream itself (proxied_body), a callback runs as run_for_group
// runs one. A wait of its worker over 10 ms counts in long_host_waits, logged once a minute.
bool service_once(pool_state &p, host_thread &h, std::unique_lock<std::mutex> &lk,
                  std::exception_ptr &err) {
  bool needs_token = false;
  const std::optional<std::size_t> at = runnable_entry(h, needs_token);
  if (!at)
    return false;
  host_entry e = std::move(h.queue[*at]);
  h.queue.erase(h.queue.begin() + static_cast<std::ptrdiff_t>(*at));
  attachment &a = *e.a;
  --a.entries;
  if (needs_token) {
    if (a.handed) {
      a.handed = false; // a release set it for this thread
    } else {
      a.host_held = true;
      a.holder = std::this_thread::get_id();
    }
  }
  const clock_type::time_point blocked_since = a.blocked_since;
  const bool outer = std::exchange(h.in_callback, true);
  lk.unlock();
  bool ran = true;
  {
    std::optional<se::async_scheduler::pump_owner_scope> token;
    if (needs_token) {
      fuzz_point(p, pool_point::hold_to_token);
      try {
        token.emplace(a.group->sched());
      } catch (...) {
        ran = false; // a thread outside the pool holds the token: left queued, and reported
        if (!err)
          err = std::current_exception();
      }
    }
    if (ran) {
      try {
        e.fn();
      } catch (...) {
        if (!err)
          err = std::current_exception();
      }
      try {
        e.fn = nullptr; // what it captured goes here, outside the lock, under the token
      } catch (...) {
      }
    }
  }
  const double waited =
      ran && e.waits ? std::chrono::duration<double>(clock_type::now() - blocked_since).count()
                     : 0.0;
  lk.lock();
  h.in_callback = outer;
  bool note = false;
  if (!ran) {
    h.queue.push_front(std::move(e));
    ++a.entries;
  } else if (e.waits) {
    a.blocked = false; // its worker resumes now
    if (waited > 0.010) {
      ++p.stats.long_host_waits;
      const clock_type::time_point now = clock_type::now();
      note =
          a.wait_noted == clock_type::time_point{} || now - a.wait_noted >= std::chrono::minutes(1);
      if (note)
        a.wait_noted = now;
    }
  }
  if (needs_token) {
    a.host_held = false;
    notify_waiters(a);
    wake_target t;
    if (a.deferred) { // a worker popped it meanwhile (S2-Q4): queue it again
      a.deferred = false;
      t = requeue(p, a, nullptr);
    } else if (wake_locked(p, a, t)) { // what ran may have resumed a process, posting nothing
      ++p.stats.wakes;
    }
    t.notify(p);
  }
  if (note) {
    se::exec_group &g = *a.group;
    lk.unlock();
    log_for(g, "exec_pool: a worker of group '" + g.opts().name + "' waited " +
                   std::to_string(waited * 1e3) +
                   " ms for a host body: its attaching thread did not service its queue (S2-Q5)");
    lk.lock();
  }
  return ran;
}

// Under Q: run what `h`'s queue holds that may run now; true when it ran any.
bool service(pool_state &p, host_thread &h, std::unique_lock<std::mutex> &lk,
             std::exception_ptr &err) {
  bool ran = false;
  while (service_once(p, h, lk, err))
    ran = true;
  return ran;
}

// Under Q (`lk` locked on entry and return): one round of a host thread's wait on `a` (td 4.9's
// rule: it services its host queue meanwhile, so a worker blocked on one of its bodies goes on):
// run one of the calling thread's host entries, else wait on its wakeup until a release, a hold's
// end, a detach's end or an entry for it.
void wait_round(pool_state &p, attachment &a, std::unique_lock<std::mutex> &lk,
                std::exception_ptr &err, std::optional<se::detail::wakeup> &own, bool wants_hold) {
  host_thread *h = host_of(p, std::this_thread::get_id());
  if (h && service_once(p, *h, lk, err))
    return;
  se::detail::wakeup *wk = h ? &h->wake : nullptr;
  if (!wk) { // a thread with no record waits on one of its own
    if (!own)
      own.emplace();
    wk = &*own;
  }
  waiter_link link(a, *wk, wants_hold);
  lk.unlock();
  wk->wait_until(clock_type::time_point::max());
  lk.lock();
}

// Under Q (`lk` locked on entry and return): wait until `a` is neither pumping nor held by
// another thread (detach's step 2), in wait_rounds. Returns early once `a` is detached (its record
// gone: the caller looks again).
void wait_until_free(pool_state &p, attachment &a, std::unique_lock<std::mutex> &lk,
                     std::exception_ptr &err) {
  const std::thread::id me = std::this_thread::get_id();
  std::optional<se::detail::wakeup> own;
  while (p.attachments.count(&a) &&
         ((word(a).load(std::memory_order_acquire) & kPumping) || (a.host_held && a.holder != me)))
    wait_round(p, a, lk, err, own, false);
}

// Under Q: the attaching thread `h`'s next caller-pumped group (exec_group.h's pop), claimed: in
// deterministic mode the rotation in creation order (td 4.4), else the least virtual time.
attachment *pop_for_host(pool_state &p, host_thread &h) {
  for (;;) {
    if (h.ready.empty())
      return nullptr;
    std::size_t best = 0;
    for (std::size_t i = 1; i < h.ready.size(); ++i) {
      const attachment *x = h.ready[i], *b = h.ready[best];
      if (p.deterministic) {
        // The first in creation order after the last one pumped, wrapping around.
        const bool x_after = x->seq > h.last_seq, b_after = b->seq > h.last_seq;
        if (x_after != b_after ? x_after : x->seq < b->seq)
          best = i;
      } else if (vtime_later{}(b, x)) {
        best = i;
      }
    }
    attachment *a = h.ready[best];
    h.ready.erase(h.ready.begin() + static_cast<std::ptrdiff_t>(best));
    a->queued = false;
    if (!claim(*a))
      continue;
    a->on_worker = false;
    h.vmin = std::max(h.vmin, a->vtime);
    h.last_seq = a->seq;
    return a;
  }
}

// Room for `n` elements in `v`, grown geometrically (attach reserves one more each time).
template <typename V> void ensure_room(V &v, std::size_t n) {
  if (v.capacity() < n)
    v.reserve(std::max(n, 2 * v.capacity()));
}

// Remove every element equal to `v` from a vector (Q held).
template <typename T> void erase_value(std::vector<T *> &v, const T *x) {
  v.erase(std::remove(v.begin(), v.end(), x), v.end());
}

// Step 4 of a detach or a teardown (exec_group.h), the caller holding the group's token: `a`
// leaves the ready queue, the next slots, its host's ready and failed lists and the timer heaps,
// and its host-queue entries move to `out`; then, outside Q, its notifier is cleared (waiting for
// a wake in progress); then the threads waiting on it are let go, the attachment is dropped, the
// word goes to 0 and the record is freed. Never throws.
void finish_detach(pool_state &p, attachment &a, std::deque<host_entry> &out) noexcept {
  se::exec_group &g = *a.group;
  se::detail::group_pool_link &link = *a.link;
  std::unique_ptr<attachment> rec;
  {
    std::lock_guard<std::mutex> lock(p.m);
    host_thread &h = *a.host;
    // Each list is looked at only when `a` can be on it, so a detach costs little with many
    // groups attached.
    if (a.queued) {
      if (std::find(p.ready.begin(), p.ready.end(), &a) != p.ready.end()) {
        erase_value(p.ready, &a);
        std::make_heap(p.ready.begin(), p.ready.end(), vtime_later{});
      }
      for (const std::unique_ptr<worker_rec> &w : p.wrecs)
        if (w->next == &a)
          w->next = nullptr;
      erase_value(h.ready, &a);
    }
    if (a.failed)
      erase_value(h.failed, &a);
    if (a.filed > 0)
      for (std::vector<timer_entry> *heap : {&p.timers, &h.timers}) {
        heap->erase(std::remove_if(heap->begin(), heap->end(),
                                   [&](const timer_entry &t) { return t.a == &a; }),
                    heap->end());
        std::make_heap(heap->begin(), heap->end(), timer_later{});
      }
    for (auto it = h.queue.begin(); a.entries > 0 && it != h.queue.end();) {
      if (it->a == &a) {
        --a.entries;
        try {
          out.push_back(std::move(*it));
        } catch (...) { // no room to keep it: dropped unrun here
        }
        it = h.queue.erase(it);
      } else {
        ++it;
      }
    }
    a.entries = 0;
  }
  // Outside Q (S2-Q2), waiting for a wake in progress (it finds detaching and queues nothing);
  // only then is the attachment dropped and the word stored 0, so no wake comes between.
  try {
    g.sched().set_wake_notifier(nullptr, nullptr);
  } catch (...) {
  }
  {
    std::lock_guard<std::mutex> lock(p.m);
    for (release_waiter *r = a.waiters; r; r = r->next) {
      r->on = nullptr;
      r->wake->notify();
    }
    a.waiters = nullptr;
    --a.host->attached;
    if (!a.caller_pumped)
      --p.attached_any;
    --p.stats.attached;
    link.attached.store(nullptr, std::memory_order_release);
    link.state.store(0, std::memory_order_release);
    auto it = p.attachments.find(&a);
    rec = std::move(it->second);
    p.attachments.erase(it);
  }
}

// Run a detached group's host entries on the calling thread, which holds its token (detach's step
// 5, take_failed), in their order, as in_pool_callback's host entries.
void run_entries(pool_state &p, std::deque<host_entry> &entries, std::exception_ptr &err) {
  host_thread *h = nullptr;
  bool outer = false;
  {
    std::lock_guard<std::mutex> lock(p.m);
    h = host_of(p, std::this_thread::get_id());
    if (h)
      outer = std::exchange(h->in_callback, true);
  }
  for (host_entry &e : entries) {
    try {
      e.fn();
    } catch (...) {
      if (!err)
        err = std::current_exception();
    }
    try {
      e.fn = nullptr;
    } catch (...) {
    }
  }
  entries.clear();
  if (h) {
    std::lock_guard<std::mutex> lock(p.m);
    h->in_callback = outer;
  }
}

// Under Q: free the calling thread's record once it has nothing attached or queued (only its own
// thread frees it: a thread waiting on its wakeup is that thread).
void release_host_if_idle(pool_state &p) {
  const std::thread::id me = std::this_thread::get_id();
  for (auto it = p.hosts.begin(); it != p.hosts.end(); ++it)
    if ((*it)->id == me) {
      host_thread &h = **it;
      if (h.attached == 0 && h.queue.empty() && h.failed.empty() && !h.in_callback)
        p.hosts.erase(it);
      return;
    }
}

// Take `g`'s token into `slot` on the calling thread, waiting while a thread outside the pool
// holds it (as the teardown waits); the pool's own pumps of it are over when this is called.
void take_token(se::exec_group &g, std::optional<se::async_scheduler::pump_owner_scope> &slot) {
  for (;;) {
    try {
      slot.emplace(g.sched());
      return;
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}

} // namespace

struct exec_pool::impl : pool_state {};

exec_pool::exec_pool(const pool_options &opts) : impl_(std::make_unique<impl>()) {
  impl &p = *impl_;
  p.self = this;
  p.opts = opts;
  p.deterministic = opts.deterministic || deterministic_knob();
  p.force_move = opts.force_move || force_move_knob();
  // CVC_EXEC_FUZZ, read once now (S2-Q15): the pool's protocol points yield as its seed says.
  p.fuzz.seed.store(se::detail::lock_fuzz_seed_from_environment(), std::memory_order_relaxed);
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
  // Each worker's continuous-mode record first (its wakeup, its next slot), so none is made
  // while workers run; the parked list has room for all.
  p.wrecs.reserve(want);
  for (unsigned i = 0; i < want; ++i) {
    auto w = std::make_unique<worker_rec>();
    w->index = static_cast<int>(i) + 1;
    w->p_core = i >= cpus.size() || cpus[i].efficiency == best;
    p.wrecs.push_back(std::move(w));
  }
  p.parked.reserve(want);
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
    // S2-Q5: the attaching threads detach what they attached first.
    assert(p.stats.attached == 0 && "exec_pool destroyed with a group attached");
    p.stopping = true; // workers stop after their current pumps
    notify_all_workers(p);
  }
  boost::this_thread::disable_interruption hold; // join is an interruption point
  for (std::unique_ptr<boost::thread> &w : p.workers)
    if (w->joinable())
      w->join();
}

std::vector<state_exec::exec_group *>
exec_pool::pump_all(std::span<state_exec::exec_group *const> groups,
                    const state_exec::pump_budget &budget,
                    std::chrono::steady_clock::time_point deadline) {
  return pump_all(groups, budget, deadline, std::function<void()>());
}

std::vector<state_exec::exec_group *> exec_pool::pump_all(
    std::span<state_exec::exec_group *const> groups, const state_exec::pump_budget &budget,
    std::chrono::steady_clock::time_point deadline, const std::function<void()> &host_first) {
  impl &p = *impl_;
#ifndef NDEBUG
  {
    std::lock_guard<std::mutex> lock(p.m);
    assert(!caller_in_callback_locked(p) &&
           "pump_all from a host_proxied body or an on_complete callback");
  }
#endif
  // This thread is this pool's caller until pump_all returns, its deferred callbacks run
  // (in_pump_all).
  struct caller_listing {
    pool_state &p;
    explicit caller_listing(pool_state &pool) : p(pool) {
      std::lock_guard<std::mutex> lock(p.m);
      p.callers.push_back(std::this_thread::get_id());
    }
    ~caller_listing() {
      std::lock_guard<std::mutex> lock(p.m);
      p.callers.erase(std::find(p.callers.begin(), p.callers.end(), std::this_thread::get_id()));
    }
    caller_listing(const caller_listing &) = delete;
    caller_listing &operator=(const caller_listing &) = delete;
  } listed_caller(p);
  // S2-Q4: an attached group belongs to the continuous pool, never to a pump_all (an assertion,
  // and std::logic_error in release builds), refused before any group is taken.
  for (se::exec_group *g : groups)
    if (g && g->pool_link().attached.load(std::memory_order_acquire) != nullptr) {
      assert(false && "an attached exec_group given to pump_all");
      throw std::logic_error("exec_pool::pump_all: group '" + g->opts().name +
                             "' is attached to a pool (detach it first; S2-Q4)");
    }
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
      entry e;
      e.call = &c;
      e.group = g;
      e.host = g->opts().affinity == se::group_affinity::host || link.caller_only;
      e.skipped_since = link.skipped ? std::max<std::uint64_t>(link.skipped_since, 1) : 0;
      c.entries.push_back(std::move(e)); // reserved: the entries never move from here on
      link.call.store(&c.entries.back(), std::memory_order_relaxed);
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
  // A threaded call is listed in p.calls (workers help it; in_pool_callback reads it) until its
  // deferred callbacks have run.
  struct call_end {
    pool_state &p;
    call_state &c;
    bool listed = false;
    ~call_end() {
      if (!listed)
        return;
      std::lock_guard<std::mutex> lock(p.m);
      p.calls.erase(std::find(p.calls.begin(), p.calls.end(), &c));
    }
  } unregister{p, c};

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
    const pumped r = pump_entry(c, e);
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
      unregister.listed = true;
      c.wakes_left = wakes;
      if (p.force_move) {
        // Every group to a thread other than its last: every helper at once.
        c.helpers_pending = wakes;
        c.wakes_left = 0;
        p.stats.helper_wakes += wakes;
        notify_all_workers(p);
      } else if (wakes > 0) {
        wake_helper(p, c);
      }
    }
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
            const pumped r = pump_entry(c, e);
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
          notify_all_workers(p);
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
  if (c.error)
    std::rethrow_exception(c.error);
  return std::move(c.failed);
}

unsigned exec_pool::threads() const { return impl_->threads; }

bool exec_pool::in_pump_all() const {
  const std::thread::id self = std::this_thread::get_id();
  std::lock_guard<std::mutex> lock(impl_->m);
  if (std::find(impl_->callers.begin(), impl_->callers.end(), self) != impl_->callers.end())
    return true;
  // A worker pumps only a threaded call's groups, and a call is listed once its entries are built.
  for (const call_state *c : impl_->calls)
    for (const entry &e : c->entries)
      if (e.pumper.v.load(std::memory_order_relaxed) == self)
        return true;
  return false;
}

pool_stats exec_pool::stats() const {
  std::lock_guard<std::mutex> lock(impl_->m);
  pool_stats s = impl_->stats;
  s.effective_threads = impl_->threads;
  // S2-Q11's active count: workers inside a pump, less those blocked over 10 ms on a host body.
  const clock_type::time_point now = clock_type::now();
  for (const std::unique_ptr<worker_rec> &w : impl_->wrecs)
    if (const attachment *a = w->current)
      if (!a->blocked || now - a->blocked_since <= std::chrono::milliseconds(10))
        ++s.active_workers;
  s.fuzz_points = impl_->fuzz.hits.load(std::memory_order_relaxed);
  return s;
}

void exec_pool::run_on_host(state_exec::exec_group &group, std::function<void()> fn) {
  if (!fn)
    return;
  entry *const e = pumped_on_this_worker(group);
  if (e && e->call->pool == impl_.get()) {
    queue_host_call(*e->call, *e, std::move(fn), false);
    return;
  }
  // Stage 2 (S2-Q5): a worker pumping an attached group queues it for its attaching thread.
  if (attachment *a = e ? nullptr : attached_on_this_worker(group); a && a->pool == impl_.get()) {
    queue_host_entry(*a, std::move(fn), false);
    return;
  }
  fn(); // the caller of the pump_all, or no pump_all: a caller never waits for itself
}

void exec_pool::set_last_pump_for_test(state_exec::exec_group &group, double seconds) {
  std::lock_guard<std::mutex> lock(impl_->m);
  group.pool_link().last_pump_seconds = seconds;
}

// --- Stage 2: continuous mode (S2a; exec_group.h's transitions) ---

void exec_pool::attach(state_exec::exec_group &group) {
  impl &p = *impl_;
  se::detail::group_pool_link &link = group.pool_link();
  attachment *a = nullptr;
  {
    std::lock_guard<std::mutex> membership(link.membership);
    if (link.attached.load(std::memory_order_acquire) != nullptr)
      throw std::logic_error("exec_pool::attach: group '" + group.opts().name +
                             "' is attached already");
    if (link.call.load(std::memory_order_relaxed) != nullptr)
      throw std::logic_error("exec_pool::attach: group '" + group.opts().name +
                             "' is held by a pump_all");
    std::lock_guard<std::mutex> lock(p.m);
    const std::thread::id me = std::this_thread::get_id();
    host_thread *h = host_of(p, me);
    std::unique_ptr<host_thread> made;
    if (!h) {
      made = std::make_unique<host_thread>();
      made->id = me;
      h = made.get();
    }
    auto rec = std::make_unique<attachment>();
    a = rec.get();
    a->pool = &p;
    a->group = &group;
    a->link = &link;
    a->host = h;
    // Pumped by this thread only (S2-Q4, td 4.14): a host-affinity group, one on a single-mode
    // lock manager, and every group of a pool with no workers or in deterministic mode.
    a->caller_pumped = p.threads == 0 || p.deterministic || !se::threads_compiled_in ||
                       group.opts().affinity == se::group_affinity::host || link.caller_only;
    a->seq = link.seq ? link.seq : (std::uint64_t(1) << 62) + p.attach_order++;
    // Room on every list a wake or a release pushes it onto, so wake(g) never allocates.
    ensure_room(h->ready, h->attached + 1);
    ensure_room(h->failed, h->attached + 1);
    if (!a->caller_pumped)
      ensure_room(p.ready, p.attached_any + 1);
    ensure_room(p.hosts, p.hosts.size() + 1);
    p.attachments.emplace(a, std::move(rec));
    if (made)
      p.hosts.push_back(std::move(made)); // reserved: does not throw
    ++h->attached;
    if (!a->caller_pumped)
      ++p.attached_any;
    ++p.stats.attached;
    if (p.deterministic) {
      // Seeded from its name, unless it has a seed or drew already (td 4.4), as pump_all does.
      se::random_stream *r = link.random;
      if (r && !r->seeded && !r->seed)
        r->seed = name_seed(group.opts().name);
    }
    // Attach takes the `scheduled` transition itself: until it queues the group below, a post's
    // wake finds it scheduled and queues nothing, so no worker pumps it meanwhile.
    link.state.store(kScheduled | kNotified, std::memory_order_relaxed);
    link.attached.store(a, std::memory_order_release);
  }
  // Outside Q (S2-Q2: the pool installs no notifier holding it).
  se::async_scheduler &s = group.sched();
  s.set_wake_notifier(&wake_notifier, a);
  // A lock post racing that install may have read "no notifier" under its port's mutex and
  // signalled the bare wakeup, its pending flag stored relaxed: taking that mutex once now makes
  // the post's flag visible to the first pump, and every later post sees the notifier.
  if (s.has_lock_home())
    (void)s.locks()->port()->closed();
  // Then queued once, so work posted before is seen.
  wake_target t;
  {
    std::lock_guard<std::mutex> lock(p.m);
    t = queue_woken(p, *a, nullptr);
    ++p.stats.wakes;
  }
  t.notify(p);
}

void exec_pool::detach(state_exec::exec_group &group) {
  impl &p = *impl_;
  std::exception_ptr err;
  attachment *a = nullptr;
  {
    std::unique_lock<std::mutex> lk(p.m);
    a = attachment_in(p, group);
    if (!a)
      return;
    assert(a->host->id == std::this_thread::get_id() &&
           "exec_pool::detach off its attaching thread");
    word(*a).fetch_or(kDetaching, std::memory_order_acq_rel); // (1)
    a->detacher = std::this_thread::get_id();
    revoke_handed_hold(*a);
    wait_until_free(p, *a, lk, err); // (2)
    if (!p.attachments.count(a))
      return;
  }
  {
    std::optional<se::async_scheduler::pump_owner_scope> token;
    take_token(group, token); // (3)
    std::deque<host_entry> entries;
    finish_detach(p, *a, entries); // (4)
    run_entries(p, entries, err);  // (5): its queued bodies and callbacks run here
  }
  {
    std::lock_guard<std::mutex> lock(p.m);
    release_host_if_idle(p);
  }
  if (err)
    std::rethrow_exception(err);
}

bool exec_pool::pump_host(double max_seconds) {
  impl &p = *impl_;
  const clock_type::time_point deadline =
      se::detail::time_after(clock_type::now(), std::isnan(max_seconds) ? 0.0 : max_seconds);
  std::exception_ptr err;
  bool did = false;
  {
    std::unique_lock<std::mutex> lk(p.m);
    host_thread *h = host_of(p, std::this_thread::get_id());
    if (!h)
      return false;
    did = service(p, *h, lk, err);
    for (;;) {
      fire_due(p, h->timers, clock_type::now(), false);
      attachment *a = pop_for_host(p, *h);
      if (!a)
        break;
      lk.unlock();
      const pump_outcome out = pump_attached(p, *a);
      lk.lock();
      release(p, *a, out, nullptr);
      if (out.error && !err)
        err = out.error;
      did = true;
      service(p, *h, lk, err);
      if (clock_type::now() >= deadline)
        break;
    }
  }
  if (err)
    std::rethrow_exception(err);
  return did;
}

bool exec_pool::wait_for_work(double seconds) {
  impl &p = *impl_;
  const clock_type::time_point deadline =
      se::detail::time_after(clock_type::now(), std::isnan(seconds) ? 0.0 : seconds);
  std::exception_ptr err;
  bool work = false;
  {
    std::unique_lock<std::mutex> lk(p.m);
    host_thread *h = host_of(p, std::this_thread::get_id());
    if (!h)
      return false; // nothing attached here: no work can come for pump_host
    for (;;) {
      if (service(p, *h, lk, err)) {
        work = true;
        break;
      }
      const clock_type::time_point now = clock_type::now();
      fire_due(p, h->timers, now, false);
      if (!h->ready.empty() || !h->failed.empty()) {
        work = true;
        break;
      }
      if (now >= deadline)
        break;
      // Until a notify (a group queued, an entry, a failure, a pump's end with entries waiting)
      // or the earliest deadline of its caller-pumped groups, kept precisely when a sleeper's or
      // a timed wait's (S2-Q3). Every push notifies after it, so none is missed between the look
      // above and the wait.
      clock_type::time_point until = deadline;
      bool precise = false;
      drop_stale(h->timers);
      if (!h->timers.empty() && h->timers.front().at < until) {
        until = h->timers.front().at;
        precise = h->timers.front().precise;
      }
      lk.unlock();
      h->wake.wait_until(until, precise);
      lk.lock();
    }
  }
  if (err)
    std::rethrow_exception(err);
  return work;
}

void exec_pool::with_group(state_exec::exec_group &group, const std::function<void()> &fn) {
  impl &p = *impl_;
  const std::thread::id me = std::this_thread::get_id();
  std::exception_ptr err;
  attachment *held = nullptr;
  {
    std::unique_lock<std::mutex> lk(p.m);
    std::optional<se::detail::wakeup> own;
    for (;;) { // exec_group.h's host transition, looked at again after each wait
      attachment *a = attachment_in(p, group);
      if (!a)
        break; // not attached (any more): as a group not attached
      if (a->host_held && a->holder == me && a->handed) {
        a->handed = false; // a release set it for this thread: held now
        held = a;
        break;
      }
      // Inside its own pump, or a hold of this thread's: the token nests, as unattached.
      if ((a->host_held && a->holder == me) || a->pumper.load(std::memory_order_relaxed) == me)
        break;
      const word_t w = word(*a).load(std::memory_order_acquire);
      if (w & kDetaching) {
        if (a->detacher == me)
          break; // on the thread finishing the detach: as unattached, at once
        wait_round(p, *a, lk, err, own, false); // until the detach is over
        continue;
      }
      if ((w & kPumping) || a->host_held) {
        wait_round(p, *a, lk, err, own, true);
        continue;
      }
      // Workers stay off it now: one that pops it leaves it notified and deferred (S2-Q4).
      a->host_held = true;
      a->holder = me;
      held = a;
      break;
    }
  }
  if (!held) {
    se::async_scheduler::pump_owner_scope token(group.sched());
    if (fn)
      fn();
    if (err)
      std::rethrow_exception(err);
    return;
  }
  fuzz_point(p, pool_point::hold_to_token);
  std::exception_ptr fn_error;
  try {
    se::async_scheduler::pump_owner_scope token(group.sched());
    try {
      if (fn)
        fn();
    } catch (...) {
      fn_error = std::current_exception();
    }
  } catch (...) {
    fn_error = std::current_exception(); // the token: held outside the pool
  }
  {
    // The release: queue it again if a pop was deferred, else wake's transition, since fn may have
    // submitted, killed or resumed, which posts nothing. If fn detached it, its record is gone
    // and left alone (a record made since is never this thread's hold); the group is not touched.
    std::lock_guard<std::mutex> lock(p.m);
    if (p.attachments.count(held) && held->host_held && held->holder == me) {
      held->host_held = false;
      notify_waiters(*held);
      wake_target t;
      if (held->deferred) {
        held->deferred = false;
        t = requeue(p, *held, nullptr);
      } else if (wake_locked(p, *held, t)) {
        ++p.stats.wakes;
      }
      t.notify(p);
    }
  }
  if (fn_error)
    std::rethrow_exception(fn_error);
  if (err)
    std::rethrow_exception(err);
}

std::vector<state_exec::exec_group *> exec_pool::take_failed() {
  impl &p = *impl_;
  std::vector<attachment *> list;
  {
    std::lock_guard<std::mutex> lock(p.m);
    host_thread *h = host_of(p, std::this_thread::get_id());
    if (!h || h->failed.empty())
      return {};
    std::vector<attachment *> fresh;
    fresh.reserve(h->failed.capacity());
    list.swap(h->failed);
    h->failed.swap(fresh);
    for (attachment *a : list)
      a->failed = false;
  }
  std::vector<se::exec_group *> out;
  out.reserve(list.size());
  for (attachment *a : list) {
    se::exec_group &g = *a->group;
    std::exception_ptr err;
    {
      std::unique_lock<std::mutex> lk(p.m);
      wait_until_free(p, *a, lk, err); // a with_group of another thread
      if (!p.attachments.count(a))
        continue;
    }
    {
      std::optional<se::async_scheduler::pump_owner_scope> token;
      take_token(g, token);
      std::deque<host_entry> entries;
      finish_detach(p, *a, entries);
      run_entries(p, entries, err);
    }
    out.push_back(&g);
    // Logged here, on its attaching thread, as pump_all logs a failure (S1.6); an entry's throw
    // is logged too, since the failed groups are this call's result.
    log_for(g, "exec_pool: group '" + g.opts().name +
                   "' failed: a C++ exception escaped its pump (" + g.pool_link().failure +
                   "); its processes were killed");
    if (err) {
      try {
        std::rethrow_exception(err);
      } catch (const std::exception &x) {
        log_for(g, "exec_pool: group '" + g.opts().name + "': a host entry threw: " + x.what());
      } catch (...) {
        log_for(g, "exec_pool: group '" + g.opts().name + "': a host entry threw");
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(p.m);
    release_host_if_idle(p);
  }
  return out;
}

bool exec_pool::attached(const state_exec::exec_group &group) const {
  std::lock_guard<std::mutex> lock(impl_->m);
  return attachment_in(*impl_, const_cast<se::exec_group &>(group)) != nullptr;
}

const pool_options &exec_pool::options() const { return impl_->opts; }

bool exec_pool::deterministic() const { return impl_->deterministic; }

std::vector<std::string> exec_pool::take_notes() {
  std::lock_guard<std::mutex> lock(impl_->m);
  return std::exchange(impl_->notes, {});
}

namespace detail {

exec_pool *worker_pool(state_exec::exec_group &group) noexcept {
  if (const entry *e = pumped_on_this_worker(group))
    return e->call->pool->self;
  if (const attachment *a = attached_on_this_worker(group)) // Stage 2: attached, on a worker
    return a->pool->self;
  return nullptr;
}

bool defer_on_complete(state_exec::exec_group &group, std::function<void()> callback) {
  if (entry *const e = entry_of(group)) {
    if (!e->call->threaded ||
        e->pumper.v.load(std::memory_order_relaxed) != std::this_thread::get_id())
      return false;
    e->deferred.push_back(std::move(callback)); // this thread pumps it: its alone until the join
    return true;
  }
  // Stage 2 (S2-Q5): a worker pumping an attached group queues it for its attaching thread, which
  // runs it under the group's token; its attaching thread pumping it runs it inline.
  if (attachment *a = attached_on_this_worker(group)) {
    queue_host_entry(*a, std::move(callback), false);
    return true;
  }
  return false;
}

bool in_pool_callback(const cvc::app *app) noexcept {
  const exec_pool *const pool = app ? app->exec_pool_probe() : nullptr;
  if (!pool)
    return false;
  try {
    std::lock_guard<std::mutex> lock(pool->impl_->m);
    if (caller_in_callback_locked(*pool->impl_))
      return true;
    // Stage 2: an attaching thread running one of its host entries.
    const std::thread::id self = std::this_thread::get_id();
    for (const std::unique_ptr<host_thread> &h : pool->impl_->hosts)
      if (h->id == self && h->in_callback)
        return true;
    return false;
  } catch (...) {
    return false;
  }
}

void purge_host_calls(state_exec::exec_group &group) noexcept {
  try {
    // Stage 2: an attached group's entries on its attaching thread's queue (none once detach_end
    // ran: it takes them, below), unrun.
    if (auto *a =
            static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire))) {
      std::deque<host_entry> dropped; // destroyed outside the pool's lock
      std::lock_guard<std::mutex> lock(a->pool->m);
      std::deque<host_entry> &q = a->host->queue;
      for (auto it = q.begin(); it != q.end();)
        if (it->a == a) {
          dropped.push_back(std::move(*it));
          it = q.erase(it);
        } else {
          ++it;
        }
      a->entries = 0;
    }
    // Held across the wait: the call cannot end (its caller clears this group's membership under
    // it) while this waits on it.
    std::lock_guard<std::mutex> membership(group.pool_link().membership);
    const entry *const held = entry_of(group);
    if (!held)
      return;
    call_state *const c = held->call;
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

bool detach_begin(state_exec::exec_group &group) noexcept {
  // The teardown's thread is the only one that detaches the group: its record lives meanwhile.
  auto *a = static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire));
  if (!a)
    return false;
  pool_state &p = *a->pool;
  std::exception_ptr err;
  try {
    std::unique_lock<std::mutex> lk(p.m);
    word(*a).fetch_or(kDetaching, std::memory_order_acq_rel); // (1)
    a->detacher = std::this_thread::get_id();
    revoke_handed_hold(*a);
    wait_until_free(p, *a, lk, err); // (2)
  } catch (...) {
  }
  if (err) {
    try {
      std::rethrow_exception(err);
    } catch (const std::exception &x) {
      log_for(group, "exec_pool: group '" + group.opts().name +
                         "' teardown: a host entry threw: " + x.what());
    } catch (...) {
    }
  }
  return true;
}

bool host_held_here(state_exec::exec_group &group) noexcept {
  // Only its attaching thread frees the record (detach, take_failed, the teardown), so on that
  // thread, or on one holding it, the record lives while this reads it under the pool's mutex.
  auto *a = static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire));
  if (!a)
    return false;
  try {
    std::lock_guard<std::mutex> lock(a->pool->m);
    return a->pool->attachments.count(a) && a->host_held && a->holder == std::this_thread::get_id();
  } catch (...) {
    return false;
  }
}

void detach_end(state_exec::exec_group &group) noexcept {
  auto *a = static_cast<attachment *>(group.pool_link().attached.load(std::memory_order_acquire));
  if (!a)
    return;
  pool_state &p = *a->pool;
  try {
    // detach_begin came first (S2-Q4), so nothing pumps it; were it skipped, a pump that slipped
    // in finds the token held and ends at once.
    std::unique_lock<std::mutex> lk(p.m);
    word(*a).fetch_or(kDetaching, std::memory_order_acq_rel);
    a->detacher = std::this_thread::get_id();
    revoke_handed_hold(*a);
    std::exception_ptr err;
    wait_until_free(p, *a, lk, err);
    if (!p.attachments.count(a))
      return;
  } catch (...) {
  }
  // Its host entries are dropped unrun here, under the teardown's token, rather than left for
  // purge_host_calls: once the attachment is dropped nothing leads from the group to its pool.
  // The kills that follow settle their calls as cancelled, as they would have.
  std::deque<host_entry> dropped;
  finish_detach(p, *a, dropped);
  try {
    dropped.clear();
    std::lock_guard<std::mutex> lock(p.m);
    release_host_if_idle(p);
  } catch (...) {
  }
}

void run_on_host_waiting(exec_pool &pool, state_exec::exec_group &group, std::function<void()> fn) {
  entry *const e = pumped_on_this_worker(group);
  if (e && e->call->pool->self == &pool) {
    queue_host_call(*e->call, *e, std::move(fn), true);
    return;
  }
  // Stage 2 (S2-Q5): its attaching thread runs it under this blocked worker's token.
  if (attachment *a = e ? nullptr : attached_on_this_worker(group); a && a->pool->self == &pool) {
    queue_host_entry(*a, std::move(fn), true);
    return;
  }
  fn();
}

} // namespace detail

} // namespace cvc

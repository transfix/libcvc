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

std::atomic<std::uint64_t> g_worker_pumps{0};

// Guards every group's pool_link().call: which pump_all a group belongs to (S1-Q11), read by a
// group's teardown to purge its host calls. Taken before a pool's mutex, never after it.
std::mutex g_membership;

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

std::string what_of(std::exception_ptr e) {
  try {
    std::rethrow_exception(e);
  } catch (const std::exception &x) {
    return x.what();
  } catch (...) {
    return "an exception of unknown type";
  }
}

// One group of a pump_all.
struct entry {
  se::exec_group *group = nullptr;
  bool host = false;                           // affinity host: the caller pumps it
  std::vector<std::function<void()>> deferred; // on_complete calls its worker pump queued
  std::uint64_t finished = 0;                  // when its pump ended (the callbacks' order)
};

struct host_call {
  se::exec_group *group; // whose worker queued it (a teardown purges by it)
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
thread_local bool t_callback = false;        // in a queued host body or a deferred on_complete

struct callback_scope {
  bool outer;
  callback_scope() : outer(t_callback) { t_callback = true; }
  ~callback_scope() { t_callback = outer; }
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
  std::vector<call_state *> calls;  // the pump_all calls in progress
  bool stopping = false;
  pool_stats stats;
  double last_host_first = 0.0; // the last host_first's time (exec_pump_frame's group 0)
  std::vector<std::string> notes;
  std::vector<std::unique_ptr<boost::thread>> workers;

  // The last pump_all's failed groups, per calling thread (failed_groups()).
  std::mutex failed_m;
  std::unordered_map<std::thread::id, std::vector<se::exec_group *>> failed_by_thread;

  void worker_main(int index, bool p_core, bool place);
};

struct call_state {
  pool_state *pool = nullptr;
  se::pump_budget budget{0, 0.0};
  clock_type::time_point deadline;
  bool timed = true; // the deadline and the seconds budgets apply (not deterministic)
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

// Pump `e` once on this thread (no lock held): the frame's budget, the token in pump().
pumped pump_entry(call_state &c, entry &e, bool on_worker, exec_pool *pool) {
  se::pump_budget budget = c.budget;
  const clock_type::time_point start = clock_type::now();
  if (c.timed)
    budget.seconds = std::min(budget.seconds, seconds_left(c, start));
  else
    budget.seconds = std::numeric_limits<double>::infinity(); // deterministic: steps only
  worker_ctx ctx{pool, &c, &e};
  worker_ctx *const outer = t_worker;
  t_worker = on_worker ? &ctx : nullptr;
  pumped out;
  try {
    out.failed = e.group->pump(budget).failed;
  } catch (...) {
    out.error = std::current_exception(); // an inline on_complete's throw
  }
  t_worker = outer;
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
    for (std::size_t i = c.front; i < c.back; ++i)
      c.entries[c.list[i]].group->pool_link().skipped = true;
    p.stats.skipped += c.remaining();
    c.front = c.back;
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
  ++c.running;
  return idx;
}

// Under the pool's mutex: record `e`'s pump on `thread`, and tell the caller.
void finish_group(pool_state &p, call_state &c, entry &e, int thread, const pumped &r,
                  bool on_worker) {
  se::detail::group_pool_link &link = e.group->pool_link();
  link.last_pump_seconds = r.seconds;
  link.last_thread = thread;
  ++p.stats.pumps;
  if (on_worker) {
    ++p.stats.worker_pumps;
    g_worker_pumps.fetch_add(1, std::memory_order_relaxed);
  }
  e.finished = ++c.finish_seq;
  if (r.failed)
    c.failed.push_back(e.group);
  if (r.error && !c.error)
    c.error = r.error;
  --c.running;
  c.cv.notify_all();
}

// Under the pool's mutex: wake one more helper for `c`. A wake is counted when it is made, so
// stats().helper_wakes does not depend on which thread gets to the work first (GR-9).
void wake_helper(pool_state &p, call_state &c) {
  --c.wakes_left;
  ++c.helpers_pending;
  ++p.stats.helper_wakes;
  p.work_cv.notify_one();
}

// Under the pool's mutex: a call that wants a helper `thread` can serve.
call_state *call_wanting_help(pool_state &p, int thread) {
  for (call_state *c : p.calls) {
    if (c->helpers_pending == 0 || c->remaining() == 0)
      continue;
    if (!p.force_move)
      return c;
    for (std::size_t i = c->front; i < c->back; ++i)
      if (c->entries[c->list[i]].group->pool_link().last_thread != thread)
        return c;
  }
  return nullptr;
}

void pool_state::worker_main(int index, bool p_core, bool place) {
  detail::apply_process_class_here();
  se::collect_cycles_at_thread_exit(); // its own registry should stay empty (S1-Q3)
  if (place)
    place_current_thread(opts.placement, static_cast<unsigned>(index - 1));
  const bool from_back = !p_core && opts.placement == cpu_set::all; // td 4.4: E-cores from the back
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
  lk.unlock();
  // A group's objects go to its registry, which its scheduler binds: none is tracked here.
  assert(se::tracked_objects() == 0 && "a pool worker tracked objects in its own registry");
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
  // Workers on the placement's CPUs in order (performance cores first), each on its own while
  // there are CPUs for it; one beyond them floats.
  const std::vector<logical_cpu> cpus = placement_cpus(opts.placement);
  const unsigned best = cpus.empty() ? 0 : cpus.front().efficiency;
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
  c.budget = budget;
  c.deadline = deadline;
  c.timed = !p.deterministic;
  c.caller = std::this_thread::get_id();
  {
    // A group belongs to one pump_all at a time (S1-Q11): asserted; a release build skips it.
    std::lock_guard<std::mutex> lock(g_membership);
    for (se::exec_group *g : groups) {
      if (!g)
        continue;
      se::detail::group_pool_link &link = g->pool_link();
      if (link.call) {
        assert(link.call == &c && "an exec_group given to two pump_all calls at once");
        continue; // given twice in this call, or held by another
      }
      link.call = &c;
      c.entries.push_back({g, g->opts().affinity == se::group_affinity::host, {}, 0});
    }
  }
  struct membership_end {
    call_state &c;
    ~membership_end() {
      std::lock_guard<std::mutex> lock(g_membership);
      for (entry &e : c.entries)
        e.group->pool_link().call = nullptr;
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
    const se::detail::group_pool_link &la = c.entries[a].group->pool_link();
    const se::detail::group_pool_link &lb = c.entries[b].group->pool_link();
    if (la.skipped != lb.skipped && !p.deterministic)
      return la.skipped; // skipped last time: first now
    return la.seq < lb.seq;
  };
  const auto run_host_first = [&] {
    if (!host_first)
      return;
    const clock_type::time_point start = clock_type::now();
    try {
      host_first();
    } catch (...) {
      if (!c.error)
        c.error = std::current_exception();
    }
    std::lock_guard<std::mutex> lock(p.m);
    p.last_host_first = std::chrono::duration<double>(clock_type::now() - start).count();
    ++p.stats.pumps;
  };
  // Pump entry `i` on the caller.
  const auto caller_pumps = [&](std::size_t i) {
    entry &e = c.entries[i];
    const pumped r = pump_entry(c, e, false, this);
    std::lock_guard<std::mutex> lock(p.m);
    finish_group(p, c, e, 0, r, false);
  };
  // A group not started before the deadline (the caller's own).
  const auto skip_on_caller = [&](std::size_t i) {
    std::lock_guard<std::mutex> lock(p.m);
    c.entries[i].group->pool_link().skipped = true;
    ++p.stats.skipped;
  };
  const auto started_in_time = [&](std::size_t i) {
    if (c.timed && clock_type::now() >= c.deadline) {
      skip_on_caller(i);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(p.m);
      c.entries[i].group->pool_link().skipped = false;
      ++c.running;
    }
    return true;
  };

  std::vector<std::size_t> order(c.entries.size());
  for (std::size_t i = 0; i < order.size(); ++i)
    order[i] = i;

  if (p.threads == 0 || p.deterministic) {
    // Everything on the caller, in creation order (td 4.14: at threads 0 the results are
    // deterministic mode's), host_first first.
    std::sort(order.begin(), order.end(), by_creation);
    run_host_first();
    for (std::size_t i : order)
      if (started_in_time(i))
        caller_pumps(i);
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
      const se::detail::group_pool_link &la = c.entries[a].group->pool_link();
      const se::detail::group_pool_link &lb = c.entries[b].group->pool_link();
      if (la.skipped != lb.skipped)
        return la.skipped;
      if (la.last_pump_seconds != lb.last_pump_seconds)
        return la.last_pump_seconds > lb.last_pump_seconds;
      return la.seq < lb.seq;
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
    run_host_first();
    for (std::size_t i : host)
      if (started_in_time(i))
        caller_pumps(i);
    // Help, and service the host-call queue, until every group is done and the queue is empty
    // (td 4.9: the host thread never blocks without servicing it).
    std::unique_lock<std::mutex> lk(p.m);
    for (;;) {
      if (!c.queue.empty()) {
        host_call hc = std::move(c.queue.front());
        c.queue.pop_front();
        c.running_body = hc.group;
        lk.unlock();
        {
          callback_scope in_body;
          try {
            hc.fn();
          } catch (...) {
            lk.lock();
            if (!c.error)
              c.error = std::current_exception();
            lk.unlock();
          }
          hc.fn = nullptr; // what it captured goes here, outside the lock
        }
        lk.lock();
        c.running_body = nullptr;
        p.purge_cv.notify_all();
        continue;
      }
      if (const std::optional<std::size_t> idx = take_group(p, c, 0, false)) {
        entry &e = c.entries[*idx];
        lk.unlock();
        const pumped r = pump_entry(c, e, false, this);
        lk.lock();
        finish_group(p, c, e, 0, r, false);
        continue;
      }
      if (c.running == 0 && c.remaining() == 0)
        break;
      if (p.force_move && c.running == 0 && c.remaining() > 0) {
        // Every group left was the caller's last: hand them to the workers.
        c.helpers_pending = std::max<std::size_t>(c.helpers_pending, p.threads);
        p.work_cv.notify_all();
      }
      c.cv.wait(lk);
    }
  }

  // After the join, on the caller: the on_complete calls worker pumps queued, group by group in
  // the order their pumps ended, each in completion order; the first throw is rethrown once all
  // ran (S1.6).
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return c.entries[a].finished < c.entries[b].finished;
  });
  for (std::size_t i : order) {
    for (std::function<void()> &cb : c.entries[i].deferred) {
      callback_scope in_callback;
      try {
        cb();
      } catch (...) {
        if (!c.error)
          c.error = std::current_exception();
      }
    }
    c.entries[i].deferred.clear();
  }
  // A failed group is logged through its app, here on the host thread.
  for (se::exec_group *g : c.failed)
    if (cvc::app *a = g->opts().app) {
      try {
        boost::this_thread::disable_interruption hold;
        a->log(1, "exec_pool: group '" + g->opts().name + "' failed: a C++ exception escaped its " +
                      "pump (" + g->pool_link().failure + "); its processes were killed");
      } catch (...) {
      }
    }
  {
    std::lock_guard<std::mutex> lock(p.failed_m);
    p.failed_by_thread[c.caller] = c.failed;
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
  std::lock_guard<std::mutex> lock(impl_->failed_m);
  const auto it = impl_->failed_by_thread.find(std::this_thread::get_id());
  return it == impl_->failed_by_thread.end() ? std::vector<state_exec::exec_group *>() : it->second;
}

void exec_pool::run_on_host(std::function<void()> fn) {
  if (!fn)
    return;
  worker_ctx *const w = t_worker;
  if (!w || w->pool != this) {
    fn(); // the caller of the pump_all, or no pump_all: a caller never waits for itself
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->m);
  w->call->queue.push_back({w->e->group, std::move(fn)});
  w->call->cv.notify_all();
}

bool exec_pool::in_pump_all() const {
  return (t_caller && t_caller->pool == impl_.get()) || (t_worker && t_worker->pool == this);
}

void exec_pool::set_last_pump_for_test(state_exec::exec_group &group, double seconds) {
  std::lock_guard<std::mutex> lock(impl_->m);
  group.pool_link().last_pump_seconds = seconds;
}

const pool_options &exec_pool::options() const { return impl_->opts; }

std::vector<std::string> exec_pool::take_notes() {
  std::lock_guard<std::mutex> lock(impl_->m);
  return std::exchange(impl_->notes, {});
}

std::uint64_t exec_pool_worker_pumps_total() {
  return g_worker_pumps.load(std::memory_order_relaxed);
}

namespace detail {

exec_pool *worker_pool() noexcept { return t_worker ? t_worker->pool : nullptr; }

bool defer_on_complete(std::function<void()> callback) {
  if (!t_worker)
    return false;
  t_worker->e->deferred.push_back(std::move(callback)); // this worker's alone until the join
  return true;
}

bool in_pool_callback() noexcept { return t_callback; }

void purge_host_calls(state_exec::exec_group &group) noexcept {
  try {
    std::lock_guard<std::mutex> membership(g_membership);
    auto *c = static_cast<call_state *>(group.pool_link().call);
    if (!c)
      return;
    std::deque<host_call> dropped; // destroyed outside the pool's lock
    std::unique_lock<std::mutex> lk(c->pool->m);
    for (auto it = c->queue.begin(); it != c->queue.end();)
      if (it->group == &group) {
        dropped.push_back(std::move(*it));
        it = c->queue.erase(it);
      } else {
        ++it;
      }
    c->pool->purge_cv.wait(lk, [&] { return c->running_body != &group; });
    lk.unlock();
  } catch (...) {
  }
}

} // namespace detail

} // namespace cvc

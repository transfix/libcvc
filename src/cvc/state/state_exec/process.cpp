#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif
#include <algorithm>
#include <atomic>
#include <boost/signals2.hpp>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <mutex>
#include <thread>

namespace cvc::state_exec {

const char *to_string(process_status s) {
  switch (s) {
  case process_status::ready:
    return "ready";
  case process_status::running:
    return "running";
  case process_status::paused:
    return "paused";
  case process_status::waiting:
    return "waiting";
  case process_status::terminated:
    return "terminated";
  case process_status::killed:
    return "killed";
  case process_status::failed:
    return "failed";
  }
  return "unknown";
}

process_status parse_process_status(const std::string &s) {
  if (s == "ready")
    return process_status::ready;
  if (s == "running")
    return process_status::running;
  if (s == "paused")
    return process_status::paused;
  if (s == "waiting")
    return process_status::waiting;
  if (s == "terminated")
    return process_status::terminated;
  if (s == "killed")
    return process_status::killed;
  if (s == "failed")
    return process_status::failed;
  return process_status::ready;
}

double process::elapsed_time() const {
  if (status != process_status::running)
    return accumulated_time;
  return elapsed_time(std::chrono::steady_clock::now());
}

double process::elapsed_time(std::chrono::steady_clock::time_point now) const {
  double t = accumulated_time;
  if (status == process_status::running)
    t += std::chrono::duration<double>(now - last_run_start).count();
  return t;
}

std::optional<double> process::time_left() const {
  if (max_time <= 0.0)
    return std::nullopt;
  const double left =
      charges_cpu_time() ? 0.5 * max_time - cpu_time_charged : max_time - elapsed_time();
  return std::max(0.0, left);
}

std::optional<double> process::time_left(std::chrono::steady_clock::time_point now) const {
  if (max_time <= 0.0)
    return std::nullopt;
  const double left =
      charges_cpu_time(now) ? 0.5 * max_time - cpu_time_charged : max_time - elapsed_time(now);
  return std::max(0.0, left);
}

void process::release_state() {
  evaluation_stats stats = std::move(state.stats);
  state = evaluator_state{};
  state.stats = std::move(stats);
  state.done = true;
  saved_stack.clear();
  saved_result = value_t{};
  signal_handlers.clear();
  watch_handlers.clear();
  inbox = {};
  heap_measuring = false;
}

bool process::heap_measure_due(uint64_t state_bytes) const {
  if (max_memory == 0 || heap_measuring)
    return false;
  if (heap_alloc_bytes >= heap_remeasure_bytes(static_cast<std::size_t>(heap_bytes)))
    return true;
  if (state_bytes + heap_bytes + heap_alloc_bytes >= max_memory)
    return heap_alloc_bytes > 0; // what natives allocated since may have crossed the budget
  return heap_check_steps > 0 && heap_steps >= heap_check_steps;
}

std::optional<std::size_t> process::alloc_limit(std::size_t used_bytes) const {
  std::optional<std::size_t> limit;
  if (max_alloc_bytes > 0)
    limit = static_cast<std::size_t>(std::min<uint64_t>(max_alloc_bytes, SIZE_MAX));
  if (max_memory > 0) {
    const uint64_t left = max_memory > used_bytes ? max_memory - used_bytes : 0;
    const std::size_t left_bytes = static_cast<std::size_t>(std::min<uint64_t>(left, SIZE_MAX));
    limit = limit ? std::min(*limit, left_bytes) : left_bytes;
  }
  return limit;
}

// ---------------------------------------------------------------------------
// detail::process_table
// ---------------------------------------------------------------------------

namespace detail {

std::chrono::steady_clock::time_point time_after(std::chrono::steady_clock::time_point t,
                                                 double seconds) {
  using clock = std::chrono::steady_clock;
  if (!(seconds > 0.0)) // also NaN
    return t;
  // The seconds left before the clock's last time point, as a double: anything at or past it
  // saturates (a cast of a double too big for the clock's integer ticks is undefined).
  const double room = std::chrono::duration<double>(clock::time_point::max() - t).count();
  if (seconds >= room)
    return clock::time_point::max();
  const clock::duration d =
      std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(seconds));
  return d >= clock::time_point::max() - t ? clock::time_point::max() : t + d;
}

bool seconds_passed(std::chrono::steady_clock::time_point from,
                    std::chrono::steady_clock::time_point now, double seconds) {
  return std::chrono::duration<double>(now - from).count() >= seconds;
}

std::optional<std::chrono::steady_clock::time_point>
first_time_passed(std::chrono::steady_clock::time_point from, double seconds) {
  using clock = std::chrono::steady_clock;
  if (std::isnan(seconds))
    return std::nullopt;
  if (seconds_passed(from, from, seconds))
    return from; // zero or negative
  // time_after truncates seconds * ticks-per-second, rounded as a double, to whole ticks, and
  // seconds_passed rounds the ticks elapsed to a double: for a long time (~1e7 s and up) both
  // round over several ticks, and the first point may be before time_after's or a few ticks
  // after it. seconds_passed only goes from false to true as `now` grows, so bracket the point
  // in steps that double from there, then halve the bracket.
  clock::time_point lo = from; // seconds_passed fails here
  clock::time_point hi = time_after(from, seconds);
  clock::duration step(1);
  if (seconds_passed(from, hi, seconds)) {
    for (;;) {
      const clock::time_point t = hi - from > step ? hi - step : from;
      if (!seconds_passed(from, t, seconds)) {
        lo = t;
        break;
      }
      hi = t;
      if (step < clock::duration::max() / 2)
        step *= 2;
    }
  } else {
    lo = hi;
    for (;;) {
      if (lo == clock::time_point::max())
        return std::nullopt; // infinite, or beyond the clock
      const clock::time_point t =
          step < clock::time_point::max() - lo ? lo + step : clock::time_point::max();
      if (seconds_passed(from, t, seconds)) {
        hi = t;
        break;
      }
      lo = t;
      if (step < clock::duration::max() / 2)
        step *= 2;
    }
  }
  while (hi - lo > clock::duration(1)) {
    const clock::time_point mid = lo + (hi - lo) / 2;
    (seconds_passed(from, mid, seconds) ? hi : lo) = mid;
  }
  return hi;
}

namespace {
bool runnable_status(process_status s) {
  return s == process_status::ready || s == process_status::running;
}
// The deadline a waiting process's timer is for: its wake_time, else a timed wait's deadline,
// else a lock wait's.
std::optional<std::chrono::steady_clock::time_point> deadline_of(const process &p) {
  if (p.status != process_status::waiting)
    return std::nullopt;
  if (p.wake_time)
    return p.wake_time;
  if (p.recv_path && p.recv_deadline)
    return p.recv_deadline;
  if (p.lock_waiting && p.lock_deadline)
    return p.lock_deadline;
  return std::nullopt;
}
struct timer_later {
  template <class T> bool operator()(const T &a, const T &b) const { return a.at > b.at; }
};
} // namespace

process_table::process_table(bool by_priority) : by_priority_(by_priority) {
  if (!by_priority_)
    levels_.push_back(level{});
}

const process_ptr &process_table::insert(process_ptr p) {
  process &proc = *p;
  proc.table = process::table_entry{};
  const auto [it, inserted] = map_.insert_or_assign(proc.pid, std::move(p));
  (void)inserted;
  if (!is_finished(proc.status))
    assign_slot(proc, &it->second);
  update(proc);
  return it->second;
}

bool process_table::erase(int pid) {
  const auto it = map_.find(pid);
  if (it == map_.end() || !is_finished(it->second->status))
    return false;
  update(*it->second); // out of every index (it already is, unless a field changed since)
  map_.erase(it);
  return true;
}

void process_table::update(process &p) {
  process::table_entry &e = p.table;
  const bool finished = is_finished(p.status);

  // Run queue.
  if (finished) {
    if (e.slot != none)
      release_slot(p);
  } else {
    const bool want = runnable_status(p.status);
    const int key = level_key(p);
    if (e.queued && (!want || e.level != key))
      dequeue(p);
    if (want && !e.queued && e.slot != none)
      enqueue(p, key);
  }

  const bool waiting = p.status == process_status::waiting;

  // Sleepers.
  const bool sleeping = waiting && p.wake_time.has_value();
  if (sleeping != e.sleeping) {
    e.sleeping = sleeping;
    sleeping ? ++sleeping_ : --sleeping_;
  }

  // Lock waiters, and lock expiries flagged for take_lock_expired.
  const bool lock_waiting = waiting && p.lock_waiting;
  if (lock_waiting != e.lock_waiting) {
    e.lock_waiting = lock_waiting;
    lock_waiting ? ++lock_waiting_ : --lock_waiting_;
  }
  const bool lock_expired = !finished && p.lock_expired_pending;
  if (lock_expired != e.lock_expired) {
    e.lock_expired = lock_expired;
    lock_expired ? ++lock_expired_ : --lock_expired_;
  }

  // Receivers by channel.
  const bool receiving = waiting && p.recv_path.has_value();
  if (e.recv_bucket != none && (!receiving || buckets_[e.recv_bucket].path != *p.recv_path))
    remove_receiver(p);
  if (receiving && e.recv_bucket == none)
    add_receiver(p);

  // Frame waiters.
  const bool awaiting = waiting && p.awaiting_frame;
  if (awaiting && e.await_pos == none) {
    e.await_pos = static_cast<std::uint32_t>(awaiting_.size());
    awaiting_.push_back(&p);
  } else if (!awaiting && e.await_pos != none) {
    process *last = awaiting_.back();
    awaiting_[e.await_pos] = last;
    last->table.await_pos = e.await_pos;
    awaiting_.pop_back();
    e.await_pos = none;
  }

  // Timers.
  const std::optional<clock::time_point> deadline = deadline_of(p);
  if (!deadline) {
    if (e.timer) {
      e.timer = false;
      --timed_;
    }
  } else if (!e.timer || e.timer_at != *deadline) {
    if (!e.timer)
      ++timed_;
    e.timer = true;
    e.timer_at = *deadline;
    push_timer(*deadline, p.pid);
  }

  // Watchers.
  const bool watching = !finished && !p.watch_handlers.empty();
  if (watching && e.watch_pos == none) {
    e.watch_pos = static_cast<std::uint32_t>(watchers_.size());
    watchers_.push_back(&p);
  } else if (!watching && e.watch_pos != none) {
    process *last = watchers_.back();
    watchers_[e.watch_pos] = last;
    last->table.watch_pos = e.watch_pos;
    watchers_.pop_back();
    e.watch_pos = none;
  }
}

// --- Run queue -------------------------------------------------------------

process_table::level *process_table::level_for(int key, bool create) {
  auto it = std::lower_bound(levels_.begin(), levels_.end(), key,
                             [](const level &l, int k) { return l.key < k; });
  if (it != levels_.end() && it->key == key)
    return &*it;
  if (!create)
    return nullptr;
  level l;
  l.key = key;
  l.fen.assign(static_cast<std::size_t>(cap_) + 1, 0);
  return &*levels_.insert(it, std::move(l));
}

process_table::level *process_table::first_nonempty() {
  for (level &l : levels_)
    if (l.count > 0)
      return &l;
  return nullptr;
}

std::uint32_t process_table::kth(const level &l, std::uint32_t k) const {
  // Fenwick descent: the slot holding the (k+1)-th queued process of the level.
  std::uint32_t pos = 0;
  std::int32_t rem = static_cast<std::int32_t>(k) + 1;
  for (std::uint32_t step = cap_; step != 0; step >>= 1) {
    const std::uint32_t next = pos + step;
    if (next <= cap_ && l.fen[next] < rem) {
      pos = next;
      rem -= l.fen[next];
    }
  }
  return pos; // 1-based pos + 1, so 0-based pos
}

void process_table::fen_add(level &l, std::uint32_t slot, std::int32_t delta) {
  for (std::uint32_t i = slot + 1; i <= cap_; i += i & (~i + 1))
    l.fen[i] += delta;
}

void process_table::assign_slot(process &p, const process_ptr *pp) {
  if (slots_.size() >= cap_)
    regrow();
  p.table.slot = static_cast<std::uint32_t>(slots_.size());
  slots_.push_back(pp);
}

void process_table::release_slot(process &p) {
  if (p.table.queued)
    dequeue(p);
  slots_[p.table.slot] = nullptr;
  ++dead_;
  p.table.slot = none;
}

void process_table::regrow() {
  // Drop the ended processes' slots, keeping the order, and size the trees for twice what is
  // left, so a regrow comes after at least as many slots as it moves (amortized O(1) a slot).
  std::uint32_t live = 0;
  for (const process_ptr *pp : slots_)
    if (pp) {
      (*pp)->table.slot = live;
      slots_[live++] = pp;
    }
  slots_.resize(live);
  dead_ = 0;
  std::uint32_t cap = 16;
  while (cap < 2 * (live + 1))
    cap <<= 1;
  cap_ = cap;
  // Empty levels go (the priorities nothing runs at now); the one level of round robin stays.
  if (by_priority_)
    levels_.erase(
        std::remove_if(levels_.begin(), levels_.end(), [](const level &l) { return l.count == 0; }),
        levels_.end());
  for (level &l : levels_)
    l.fen.assign(static_cast<std::size_t>(cap_) + 1, 0);
  for (std::uint32_t s = 0; s < live; ++s) {
    const process &q = **slots_[s];
    if (q.table.queued)
      level_for(q.table.level, false)->fen[s + 1] = 1;
  }
  for (level &l : levels_)
    for (std::uint32_t i = 1; i <= cap_; ++i) {
      const std::uint32_t j = i + (i & (~i + 1));
      if (j <= cap_)
        l.fen[j] += l.fen[i];
    }
}

void process_table::enqueue(process &p, int key) {
  level &l = *level_for(key, true);
  fen_add(l, p.table.slot, 1);
  ++l.count;
  ++queued_;
  p.table.queued = true;
  p.table.level = key;
}

void process_table::dequeue(process &p) {
  level &l = *level_for(p.table.level, false);
  fen_add(l, p.table.slot, -1);
  --l.count;
  --queued_;
  p.table.queued = false;
}

const process_ptr *process_table::pick_round_robin(int &rr_index) {
  level *l = first_nonempty();
  if (!l)
    return nullptr;
  const int n = static_cast<int>(l->count);
  if (n == 1) {
    // One runnable process (a lone program): when it is the one picked last, it is picked again
    // without the descent. The index comes out as below (any index mod 1 is 0).
    rr_index = 0;
    if (last_pick_ < slots_.size() && slots_[last_pick_]) {
      const process::table_entry &e = (*slots_[last_pick_])->table;
      if (e.queued && e.level == l->key)
        return slots_[last_pick_];
    }
    last_pick_ = kth(*l, 0);
    return slots_[last_pick_];
  }
  const int k = rr_index >= 0 && rr_index < n ? rr_index : rr_index % n;
  rr_index = k + 1 < n ? k + 1 : 0;
  last_pick_ = kth(*l, static_cast<std::uint32_t>(k));
  return slots_[last_pick_];
}

const process_ptr *process_table::pick_first() {
  level *l = first_nonempty();
  return l ? slots_[kth(*l, 0)] : nullptr;
}

// --- Receivers -------------------------------------------------------------

void process_table::add_receiver(process &p) {
  const auto [it, inserted] = bucket_of_.try_emplace(*p.recv_path, none);
  if (inserted) {
    if (free_buckets_.empty()) {
      it->second = static_cast<std::uint32_t>(buckets_.size());
      buckets_.emplace_back();
    } else {
      it->second = free_buckets_.back();
      free_buckets_.pop_back();
    }
    buckets_[it->second].path = *p.recv_path;
  }
  bucket &b = buckets_[it->second];
  p.table.recv_bucket = it->second;
  p.table.recv_pos = static_cast<std::uint32_t>(b.procs.size());
  b.procs.push_back(&p);
  ++receiving_;
}

void process_table::remove_receiver(process &p) {
  const std::uint32_t bi = p.table.recv_bucket;
  bucket &b = buckets_[bi];
  process *last = b.procs.back();
  b.procs[p.table.recv_pos] = last;
  last->table.recv_pos = p.table.recv_pos;
  b.procs.pop_back();
  p.table.recv_bucket = none;
  p.table.recv_pos = none;
  --receiving_;
  if (b.procs.empty())
    free_bucket(bi);
}

void process_table::free_bucket(std::uint32_t bi) {
  bucket &b = buckets_[bi];
  bucket_of_.erase(b.path);
  b.path.clear();
  free_buckets_.push_back(bi);
}

void process_table::take_receivers(const std::string &path, std::vector<process *> &out) {
  const auto it = bucket_of_.find(path);
  if (it == bucket_of_.end())
    return;
  const std::uint32_t bi = it->second;
  bucket &b = buckets_[bi];
  for (process *p : b.procs) {
    p->table.recv_bucket = none;
    p->table.recv_pos = none;
    out.push_back(p);
  }
  receiving_ -= b.procs.size();
  b.procs.clear(); // keeps its capacity for the bucket's next use
  free_bucket(bi);
}

void process_table::take_awaiting(std::vector<process *> &out) {
  for (process *p : awaiting_) {
    p->table.await_pos = none;
    out.push_back(p);
  }
  awaiting_.clear();
}

// --- Timers ----------------------------------------------------------------

void process_table::push_timer(clock::time_point at, int pid) {
  // Stale entries (a wait that ended before its deadline) are dropped as they surface; if they
  // pile up (many waits that end early), the heap is rebuilt from the live ones.
  if (heap_.size() >= 64 && heap_.size() > 2 * timed_) {
    heap_.erase(std::remove_if(heap_.begin(), heap_.end(),
                               [this](const timer_entry &t) { return !timer_live(t); }),
                heap_.end());
    std::make_heap(heap_.begin(), heap_.end(), timer_later{});
  }
  heap_.push_back(timer_entry{at, pid});
  std::push_heap(heap_.begin(), heap_.end(), timer_later{});
}

bool process_table::timer_live(const timer_entry &t) const {
  const process *p = find(t.pid);
  return p && p->table.timer && p->table.timer_at == t.at;
}

std::optional<process_table::clock::time_point> process_table::next_timer() {
  while (!heap_.empty()) {
    if (timer_live(heap_.front()))
      return heap_.front().at;
    std::pop_heap(heap_.begin(), heap_.end(), timer_later{});
    heap_.pop_back();
  }
  return std::nullopt;
}

std::size_t process_table::wake_due(clock::time_point now, std::vector<process *> *lock_expired) {
  std::size_t woken = 0;
  while (!heap_.empty() && heap_.front().at <= now) {
    const timer_entry t = heap_.front();
    std::pop_heap(heap_.begin(), heap_.end(), timer_later{});
    heap_.pop_back();
    if (!timer_live(t))
      continue;
    process &p = *find(t.pid);
    p.table.timer = false; // this entry is spent; update() pushes another if one is needed
    --timed_;
    if (p.status == process_status::waiting && p.wake_time && now >= *p.wake_time) {
      p.wake_time.reset();
      set_status(p, process_status::ready);
      ++woken;
    } else if (p.status == process_status::waiting && p.recv_path && p.recv_deadline &&
               now >= *p.recv_deadline) {
      // A timed wait ((await f seconds)) that ran out: the waiting call raises TimeoutError.
      p.recv_path.reset();
      p.recv_deadline.reset();
      expire_await(p.state);
      set_status(p, process_status::ready);
      ++woken;
    } else if (p.status == process_status::waiting && p.lock_waiting && p.lock_deadline &&
               now >= *p.lock_deadline) {
      // A lock wait that ran out: its home cancels the ticket and wakes the process
      // (lock_home::wait_expired). The deadline is spent, so update() pushes no timer again.
      p.lock_deadline.reset();
      if (lock_expired)
        lock_expired->push_back(&p);
      else
        p.lock_expired_pending = true;
      update(p);
      ++woken;
    } else {
      update(p); // not due after all: keep its timer
    }
  }
  return woken;
}

void process_table::take_lock_expired(std::vector<process *> &out) {
  for (const auto &[pid, pp] : map_) {
    process &p = *pp;
    if (!p.lock_expired_pending)
      continue;
    p.lock_expired_pending = false;
    update(p);
    if (!is_finished(p.status))
      out.push_back(&p);
  }
}

// --- Invariants ------------------------------------------------------------

bool process_table::check() const {
  std::size_t queued = 0, sleeping = 0, receiving = 0, timed = 0, awaiting = 0, watching = 0;
  std::size_t lock_waiting = 0, lock_expired = 0;
  int last_pid = INT32_MIN;
  for (std::uint32_t s = 0; s < slots_.size(); ++s) {
    if (!slots_[s])
      continue;
    const process &p = **slots_[s];
    if (p.table.slot != s || p.pid <= last_pid || is_finished(p.status))
      return false;
    last_pid = p.pid;
  }
  for (const auto &[pid, pp] : map_) {
    const process &p = *pp;
    const process::table_entry &e = p.table;
    const bool finished = is_finished(p.status);
    if (finished != (e.slot == none))
      return false;
    if (!finished &&
        (e.slot >= slots_.size() || slots_[e.slot] == nullptr || slots_[e.slot]->get() != &p))
      return false;
    const bool want_q = !finished && runnable_status(p.status);
    if (e.queued != want_q || (want_q && e.level != level_key(p)))
      return false;
    queued += e.queued;
    const bool waiting = p.status == process_status::waiting;
    if (e.lock_waiting != (waiting && p.lock_waiting))
      return false;
    lock_waiting += e.lock_waiting;
    // A flagged lock expiry is a lock waiter's, and is counted while the process lives.
    if (p.lock_expired_pending && !finished && !(waiting && p.lock_waiting))
      return false;
    if (e.lock_expired != (!finished && p.lock_expired_pending))
      return false;
    lock_expired += e.lock_expired;
    if (e.sleeping != (waiting && p.wake_time.has_value()))
      return false;
    sleeping += e.sleeping;
    const bool recv = waiting && p.recv_path.has_value();
    if (recv != (e.recv_bucket != none))
      return false;
    if (recv) {
      const auto it = bucket_of_.find(*p.recv_path);
      if (it == bucket_of_.end() || it->second != e.recv_bucket ||
          buckets_[e.recv_bucket].procs[e.recv_pos] != &p)
        return false;
      ++receiving;
    }
    const bool aw = waiting && p.awaiting_frame;
    if (aw != (e.await_pos != none) || (aw && awaiting_[e.await_pos] != &p))
      return false;
    awaiting += aw;
    const std::optional<clock::time_point> d = deadline_of(p);
    if (d.has_value() != e.timer || (d && e.timer_at != *d))
      return false;
    if (d) {
      bool found = false;
      for (const timer_entry &t : heap_)
        found = found || (t.pid == p.pid && t.at == *d);
      if (!found)
        return false;
      ++timed;
    }
    const bool w = !finished && !p.watch_handlers.empty();
    if (w != (e.watch_pos != none) || (w && watchers_[e.watch_pos] != &p))
      return false;
    watching += w;
  }
  if (queued != queued_ || sleeping != sleeping_ || receiving != receiving_ || timed != timed_ ||
      awaiting != awaiting_.size() || watching != watchers_.size() ||
      lock_waiting != lock_waiting_ || lock_expired != lock_expired_)
    return false;
  // The trees: each level's count, and its prefix sums against the slots.
  std::size_t level_total = 0;
  for (const level &l : levels_) {
    std::uint32_t n = 0;
    for (std::uint32_t s = 0; s < slots_.size(); ++s)
      if (slots_[s] && (*slots_[s])->table.queued && (*slots_[s])->table.level == l.key)
        if (kth(l, n++) != s)
          return false;
    if (n != l.count)
      return false;
    level_total += n;
  }
  return level_total == queued_;
}

} // namespace detail

// ---------------------------------------------------------------------------
// detail::wakeup
// ---------------------------------------------------------------------------

namespace detail {

#if defined(_WIN32)
struct wakeup::impl {
  HANDLE event = nullptr; // auto-reset: a wait it satisfies consumes the notification
  HANDLE timer = nullptr; // a synchronization timer, high-resolution where there is one
};

wakeup::wakeup() : impl_(std::make_unique<impl>()) {
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
  impl_->event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  impl_->timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                        TIMER_ALL_ACCESS);
  if (!impl_->timer) // before Windows 10 1803: a timer of the system's resolution
    impl_->timer = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);
}

wakeup::~wakeup() {
  if (impl_->timer)
    CloseHandle(impl_->timer);
  if (impl_->event)
    CloseHandle(impl_->event);
}

void wakeup::notify() {
  if (impl_->event)
    SetEvent(impl_->event);
}

bool wakeup::wait_until(clock::time_point t, bool precise) {
  if (!impl_->event) {
    std::this_thread::sleep_until(t);
    return false;
  }
  // The high-resolution timer fires on a ~0.5 ms grid. A precise wait sets it that much early
  // and spins through the last stretch on the CPU (still watching the event): it ends on time,
  // as the spinning it replaces did, for at most that much CPU. It spins rather than yields: a
  // yield on a loaded machine can give the core away for a whole scheduler quantum, and the
  // wake comes that late. Any other wait just takes the timer.
  constexpr long long kSpinNs = 600000;
  const long long early = precise ? kSpinNs : 0;
  // A precise wait on a thread at normal priority runs above it, so on a machine whose cores are
  // all busy the thread gets one back when its timer fires instead of a scheduler quantum later
  // (a spinning thread never gave its core up). It blocks or spins for at most the wait. A thread
  // its host gave another priority keeps it: a lowered one stays below its host's other threads.
  // Back to normal after, unless the priority changed meanwhile (the host's change stands).
  struct priority_raise {
    HANDLE thread = GetCurrentThread();
    bool raised = false;
    explicit priority_raise(bool on) {
      raised = on && GetThreadPriority(thread) == THREAD_PRIORITY_NORMAL &&
               SetThreadPriority(thread, THREAD_PRIORITY_ABOVE_NORMAL);
    }
    ~priority_raise() {
      if (raised && GetThreadPriority(thread) == THREAD_PRIORITY_ABOVE_NORMAL)
        SetThreadPriority(thread, THREAD_PRIORITY_NORMAL);
    }
  } raise(precise);
  for (;;) {
    const clock::time_point now = clock::now();
    if (now >= t)
      return WaitForSingleObject(impl_->event, 0) == WAIT_OBJECT_0;
    const long long ns =
        t == clock::time_point::max()
            ? LLONG_MAX
            : std::chrono::duration_cast<std::chrono::nanoseconds>(t - now).count();
    if (impl_->timer && ns <= early) {
      if (WaitForSingleObject(impl_->event, 0) == WAIT_OBJECT_0)
        return true;
      for (int i = 0; i < 64; ++i)
        YieldProcessor(); // a pause: the core stays ours
      continue;
    }
    if (impl_->timer && ns < LLONG_MAX / 2) {
      LARGE_INTEGER due;
      due.QuadPart = -((ns - early + 99) / 100); // relative, in 100 ns units
      if (SetWaitableTimer(impl_->timer, &due, 0, nullptr, nullptr, FALSE)) {
        const HANDLE handles[2] = {impl_->event, impl_->timer};
        if (WaitForMultipleObjects(2, handles, FALSE, INFINITE) == WAIT_OBJECT_0)
          return true;
        // The timer: the loop reads the clock again, so a timer that fired early never ends a
        // wait before `t` (it waits the rest), and a precise wait spins its last stretch.
        continue;
      }
    }
    const DWORD ms = ns >= static_cast<long long>(INFINITE - 1) * 1000000
                         ? INFINITE - 1
                         : static_cast<DWORD>((ns + 999999) / 1000000);
    if (WaitForSingleObject(impl_->event, ms) == WAIT_OBJECT_0)
      return true;
  }
}
#else
struct wakeup::impl {
  std::mutex mu;
  std::condition_variable cv;
  bool signaled = false;
};

wakeup::wakeup() : impl_(std::make_unique<impl>()) {}
wakeup::~wakeup() = default;

void wakeup::notify() {
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    impl_->signaled = true;
  }
  impl_->cv.notify_one();
}

bool wakeup::wait_until(clock::time_point t, bool /*precise*/) {
  // A far deadline (time_point::max(): none) is waited for an hour at a time, so no standard
  // library converts it to its own clock with an overflow. The clock is read again after each
  // wait: none ends before `t` unless notified.
  std::unique_lock<std::mutex> lock(impl_->mu);
  for (;;) {
    if (impl_->signaled) {
      impl_->signaled = false;
      return true;
    }
    const clock::time_point now = clock::now();
    if (now >= t)
      return false;
    impl_->cv.wait_until(lock, t - now > std::chrono::hours(1) ? now + std::chrono::hours(1) : t);
  }
}
#endif

// ---------------------------------------------------------------------------
// detail::tree_changes
// ---------------------------------------------------------------------------

struct tree_changes::impl {
  // Shared with the slots: a write on another thread may still be running one when this goes.
  std::shared_ptr<std::atomic<std::uint64_t>> count =
      std::make_shared<std::atomic<std::uint64_t>>(0);
  boost::signals2::scoped_connection child, value;
};

tree_changes::tree_changes() : impl_(std::make_unique<impl>()) {}
tree_changes::~tree_changes() = default;

void tree_changes::watch(cvc::state *root) {
  if (root == root_ && (!root || attached()))
    return;
  impl_->child.disconnect();
  impl_->value.disconnect();
  root_ = root;
  // A new root is a change: what a reader saw counts for the old one.
  impl_->count->fetch_add(1, std::memory_order_acq_rel);
  if (!root)
    return;
  std::shared_ptr<std::atomic<std::uint64_t>> c = impl_->count;
  impl_->child = root->childChanged.connect(
      [c](const std::string &) { c->fetch_add(1, std::memory_order_release); });
  impl_->value = root->valueChanged.connect([c]() { c->fetch_add(1, std::memory_order_release); });
}

bool tree_changes::attached() const { return impl_->child.connected(); }

std::uint64_t tree_changes::count() const { return impl_->count->load(std::memory_order_acquire); }

// ---------------------------------------------------------------------------
// detail::settings_changes
// ---------------------------------------------------------------------------

std::uint64_t settings_changes::version(cvc::state *root) {
  // The node the settings live under (scheduler::settings_root); a different node than the one
  // counted, or none, counts as a change (tree_changes::watch).
  cvc::state *const node = root ? root->findDescendant("state_exec") : nullptr;
  changes_.watch(node);
  return changes_.count();
}

} // namespace detail

} // namespace cvc::state_exec

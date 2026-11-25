// Stage 2's process control across groups (docs/STATE_EXEC_THREADED.md 4.5; S2.md S2b,
// S2-Q7..S2-Q9): the pid directory, the owner rule's cross-group half, and published info
// (exec_router.h has the rules).
#include <algorithm>
#include <cassert>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/detail/priority.h> // floor_of
#include <cvc/state/state_exec/exec_router.h>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace cvc::state_exec {

// --- exec_directory (S2b.2; td 4.5: 64 shards, each a mutex and a map) ---

namespace {
constexpr std::size_t kDirectoryShards = 64;
std::size_t shard_of(int pid) noexcept {
  return static_cast<std::size_t>(static_cast<unsigned>(pid)) % kDirectoryShards;
}
} // namespace

struct exec_directory::shard {
  mutable std::mutex mu;
  std::unordered_map<int, directory_entry> entries;
};

exec_directory::exec_directory() : shards_(std::make_unique<shard[]>(kDirectoryShards)) {}

exec_directory::~exec_directory() = default;

void exec_directory::add(int pid, const group_ref &group, const std::string &owner) {
  shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  // Pids are unique in an app and an entry goes at its reap, so one already here is a live pid
  // issued twice (a pid source below a live pid, which set_pid_source refuses).
  assert(s.entries.find(pid) == s.entries.end() && "exec_directory: add replaces a live entry");
  s.entries.insert_or_assign(pid, directory_entry{pid, group, owner});
}

void exec_directory::remove(int pid) noexcept {
  shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  s.entries.erase(pid);
}

std::optional<directory_entry> exec_directory::find(int pid) const {
  const shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  const auto it = s.entries.find(pid);
  if (it == s.entries.end())
    return std::nullopt;
  return it->second;
}

std::vector<group_ref> exec_directory::groups_of(const std::string &owner) const {
  // Each group once, in group id order (creation order; group 0 first), whatever order the
  // shards' maps hold their pids in: pool.kill_owner posts in that order.
  std::vector<group_ref> out;
  for (std::size_t i = 0; i < kDirectoryShards; ++i) {
    const shard &s = shards_[i];
    std::lock_guard<std::mutex> lock(s.mu);
    for (const auto &[pid, e] : s.entries)
      if (e.owner == owner && e.group && std::find(out.begin(), out.end(), e.group) == out.end())
        out.push_back(e.group);
  }
  std::stable_sort(out.begin(), out.end(),
                   [](const group_ref &a, const group_ref &b) { return a.id() < b.id(); });
  return out;
}

std::size_t exec_directory::size() const {
  std::size_t n = 0;
  for (std::size_t i = 0; i < kDirectoryShards; ++i) {
    std::lock_guard<std::mutex> lock(shards_[i].mu);
    n += shards_[i].entries.size();
  }
  return n;
}

// --- The router's control entry points (S2b.5) and info (S2b.3) ---

bool exec_router::post_control(int pid, control_cmd cmd) {
  // The directory's mutex is a leaf: the entry is copied out before anything is posted.
  const std::optional<directory_entry> e = directory_.find(pid);
  return e && e->group.post_control(pid, std::move(cmd));
}

std::optional<published_process> exec_router::info(int pid) const {
  const std::optional<directory_entry> e = directory_.find(pid);
  if (!e)
    return std::nullopt;
  std::optional<published_process> row;
  e->group.visit([&](async_scheduler &s) { row = s.published_info(pid); });
  return row;
}

// --- async_scheduler: the owner rule (S2b.5) ---

bool async_scheduler::control_as(const std::string &caller_owner, int pid, control_cmd::kind what) {
  if (what != control_cmd::kind::kill && what != control_cmd::kind::pause &&
      what != control_cmd::kind::resume)
    return false;
  // The local half (scheduler_base's default, read from the table rather than a process_info): a
  // caller with an owner reaches only its owner's processes; one without, every process.
  if (const process *p = procs_.find(pid)) {
    if (!caller_owner.empty() && p->owner != caller_owner)
      return false;
    if (what == control_cmd::kind::kill)
      return kill(pid);
    return what == control_cmd::kind::pause ? pause(pid) : resume(pid);
  }
  // The directory's half (S2-Q8): a pid in another group of the caller's own, non-empty, owner is
  // posted there, and "requested" is #t. Owner "" has no rights across groups (S2-Q7).
  if (caller_owner.empty() || !router_)
    return false;
  const std::optional<directory_entry> e = router_->directory().find(pid);
  if (!e || e->owner != caller_owner || e->group == route_self_)
    return false;
  control_cmd cmd;
  cmd.what = what;
  return e->group.post_control(pid, std::move(cmd));
}

// --- async_scheduler: published info (S2b.3; S2-Q9) ---

namespace {
constexpr auto kCounterRefresh = std::chrono::milliseconds(100); // S2-Q9's counter lag

const published_process *row_of(const std::vector<published_process> &rows, int pid) {
  const auto it = std::lower_bound(rows.begin(), rows.end(), pid,
                                   [](const published_process &r, int p) { return r.pid < p; });
  return it != rows.end() && it->pid == pid ? &*it : nullptr;
}

bool holds_locks(const process &p) { return p.locks && !p.locks->holds.empty(); }

// `shown` is `key` as a row shows it: absolute, with a leading '/'.
bool same_key(const std::string &shown, const std::string &key) {
  return shown.size() == key.size() + 1 && shown[0] == '/' &&
         shown.compare(1, key.size(), key) == 0;
}

// Whether a process's lock holds differ from its row's. A hold taken or dropped without a wait
// changes no status, so it does not move the table's generation, and lock_home's paths are
// frozen (L1's speed work): the publisher compares the mirrors with the rows instead. Only the
// holders' keys are compared; a row that showed holds of a process that now has none is found by
// count (every holder's row matched, so the counts differ).
bool holds_moved(const detail::process_table &procs, const std::vector<published_process> &rows) {
  std::size_t holders = 0;
  for (const auto &[pid, p] : procs.all()) {
    if (!holds_locks(*p))
      continue;
    ++holders;
    const published_process *row = row_of(rows, pid);
    if (!row)
      return true;
    std::size_t i = 0;
    for (const detail::mhold &h : p->locks->holds)
      for (const detail::lock_node *n : h.keys) {
        if (i >= row->locks_held.size() || !same_key(row->locks_held[i], n->key))
          return true;
        ++i;
      }
    if (i != row->locks_held.size())
      return true;
  }
  const auto shown = static_cast<std::size_t>(std::count_if(
      rows.begin(), rows.end(), [](const published_process &r) { return !r.locks_held.empty(); }));
  return shown != holders;
}
} // namespace

void async_scheduler::publish_info() {
  if (!router_)
    return;
  // The summary: the run report when this run loop computed it (a wake notifier is installed),
  // else the table's own.
  bool runnable = false;
  std::optional<int> best;
  clock::time_point next = clock::time_point::max();
  if (wakeup_->has_notifier()) {
    runnable = run_report_.runnable;
    best = run_report_.best_runnable;
    next = run_report_.next_deadline;
  } else {
    runnable = has_runnable();
    best = procs_.best_runnable();
    if (procs_.timed_count() > 0)
      next = procs_.next_timer().value_or(clock::time_point::max());
  }

  // The rows: rebuilt when the generation moved (or a hold did), else their counters refreshed at
  // most every 100 ms. The end of the run loop's last step is "now" when there was one.
  const clock::time_point now = chain_valid_ ? chain_now_ : clock::now();
  const std::uint64_t generation = procs_.generation();
  const bool rebuild = generation != pub_generation_ || (home_ && holds_moved(procs_, pub_rows_));
  try {
    if (rebuild) {
      std::vector<published_process> rows;
      rows.reserve(procs_.size());
      int live = 0;
      for (const auto &[pid, pp] : procs_.all()) {
        const process &p = *pp;
        if (!is_finished(p.status))
          ++live;
        published_process &r = rows.emplace_back();
        r.pid = p.pid;
        r.name = p.name;
        r.owner = p.owner;
        r.uid = p.uid;
        r.scope = process_scope(p.root_path, p.owner_root);
        r.parent_pid = p.parent_pid;
        r.status = p.status;
        r.base_priority = p.priority;
        // As get-priority and inspect report it: under round_robin the base (design 7.1).
        r.effective_priority =
            policy_ == scheduling_policy::round_robin ? p.priority : p.effective_priority();
        r.floor = detail::floor_of(p);
        if (holds_locks(p))
          for (const detail::mhold &h : p.locks->holds)
            for (const detail::lock_node *n : h.keys)
              r.locks_held.push_back("/" + n->key);
        // process_info::waiting_on, as make_info reads it.
        if (p.status == process_status::waiting) {
          if (p.lock_waiting)
            r.waits_on = "lock /" + detail::lock_home::waiting_key(p);
          else if (p.recv_path)
            r.waits_on = "msg " + *p.recv_path;
          else if (p.wake_time)
            r.waits_on = "sleep";
          else if (p.awaiting_frame)
            r.waits_on = "frame";
        }
        r.steps = p.step_count();
        r.run_seconds = p.elapsed_time(now);
        r.memory = mem_tracker_.current_bytes(p.pid);
      }
      std::sort(
          rows.begin(), rows.end(),
          [](const published_process &a, const published_process &b) { return a.pid < b.pid; });
      {
        std::lock_guard<std::mutex> lock(pub_mu_);
        pub_rows_.swap(rows); // the old rows go outside the lock
      }
      pub_generation_ = generation;
      pub_refreshed_ = now;
      pub_processes_.store(live, std::memory_order_relaxed);
    } else if (!pub_rows_.empty() && now - pub_refreshed_ >= kCounterRefresh) {
      std::lock_guard<std::mutex> lock(pub_mu_);
      for (published_process &r : pub_rows_)
        if (const process *p = procs_.find(r.pid)) {
          r.steps = p->step_count();
          r.run_seconds = p->elapsed_time(now);
          r.memory = mem_tracker_.current_bytes(r.pid);
        }
      pub_refreshed_ = now;
    }
  } catch (...) {
    // No memory for the rows: the old ones stay, and the next run loop rebuilds them.
    pub_generation_ = ~std::uint64_t{0};
  }

  pub_runnable_.store(runnable, std::memory_order_relaxed);
  pub_deadline_.store(next.time_since_epoch().count(), std::memory_order_relaxed);
  pub_best_.store(best.value_or(std::numeric_limits<int>::min()), std::memory_order_relaxed);
  pub_count_.fetch_add(1, std::memory_order_release); // last: a reader that sees it sees the rest
}

async_scheduler::published_summary async_scheduler::published() const noexcept {
  published_summary s;
  s.publishes = pub_count_.load(std::memory_order_acquire);
  if (s.publishes == 0)
    return s;
  s.runnable = pub_runnable_.load(std::memory_order_relaxed);
  s.next_deadline =
      clock::time_point(clock::duration(pub_deadline_.load(std::memory_order_relaxed)));
  const int best = pub_best_.load(std::memory_order_relaxed);
  if (best != std::numeric_limits<int>::min())
    s.best_runnable = best;
  s.processes = pub_processes_.load(std::memory_order_relaxed);
  return s;
}

std::optional<published_process> async_scheduler::published_info(int pid) const {
  std::lock_guard<std::mutex> lock(pub_mu_);
  if (const published_process *r = row_of(pub_rows_, pid))
    return *r;
  return std::nullopt;
}

std::vector<published_process> async_scheduler::published_rows() const {
  std::lock_guard<std::mutex> lock(pub_mu_);
  return pub_rows_;
}

} // namespace cvc::state_exec

/**
 * @file process_table.h
 * @brief The schedulers' process table: every process by pid, and the indexes a step uses.
 *
 * Both schedulers keep their processes here. Besides the pid map (which keeps an ended process
 * until it is reaped, so its result stays readable), the table keeps what a step needs without
 * scanning every process:
 *
 * - a run queue: the ready and running processes, in pid order within a level (one level, or one
 *   per priority for the priority policies), with order statistics (a Fenwick tree over the
 *   processes' slots), so the k-th of them is found in O(log n). That is what the round-robin
 *   pick needs to stay exactly what it was: the pid-sorted runnable list indexed by rr_index;
 * - a timer heap of the sleepers' wake_time and the timed waits' recv_deadline;
 * - the processes waiting for a message, by channel (recv_path);
 * - the processes parked until the next frame (awaiting_frame), and those with state watches;
 * - counts of the sleepers and receivers (the sync scheduler's has_runnable()).
 *
 * The scheduler changes a process's status, priority and wait fields itself, then calls update()
 * (set_status() for a status change), which brings the indexes in line with the fields. An ended
 * process leaves every index; it stays in the map until erase(). Scheduler-thread only, like the
 * schedulers.
 */
#ifndef CVC_STATE_EXEC_PROCESS_TABLE_H
#define CVC_STATE_EXEC_PROCESS_TABLE_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/process.h>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec::detail {

/// `t` plus `seconds`, saturating: an infinite or huge `seconds` gives the clock's last time
/// point (a plain duration_cast of one overflows), a negative or NaN one gives `t`.
std::chrono::steady_clock::time_point time_after(std::chrono::steady_clock::time_point t,
                                                 double seconds);

class process_table {
public:
  using clock = std::chrono::steady_clock;
  using map_type = std::unordered_map<int, process_ptr>;

  /// `by_priority`: a run-queue level per priority (the priority policies); otherwise one level.
  explicit process_table(bool by_priority = false);
  process_table(const process_table &) = delete;
  process_table &operator=(const process_table &) = delete;

  // --- The processes by pid, ended ones included ---

  const map_type &all() const { return map_; }
  std::size_t size() const { return map_.size(); }
  process *find(int pid) const {
    const auto it = map_.find(pid);
    return it == map_.end() ? nullptr : it->second.get();
  }

  /// Add `p` and index it. A live process's pid must be higher than every pid added before it
  /// (the run queue's slots are in pid order). Its table entry starts afresh.
  const process_ptr &insert(process_ptr p);
  /// Drop an ended process. False (nothing done) for an unknown pid or a live one.
  bool erase(int pid);

  /// Bring the indexes in line with `p`'s status, priority, wake_time, recv_path,
  /// recv_deadline, awaiting_frame and watch_handlers. Call it after changing any of them.
  void update(process &p);
  /// Set `p`'s status. Between ready and running no index changes.
  void set_status(process &p, process_status s) {
    const bool was = p.status == process_status::ready || p.status == process_status::running;
    p.status = s;
    if (!was || (s != process_status::ready && s != process_status::running))
      update(p);
  }

  // --- The run queue: the ready and running processes ---

  std::size_t runnable_count() const { return queued_; }
  /// The round-robin pick (round_robin, and priority_rr within the best priority): the
  /// `rr_index`-th (mod the count) of the runnable processes in pid order; rr_index becomes the
  /// next index. nullptr when none is runnable.
  const process_ptr *pick_round_robin(int &rr_index);
  /// The priority pick: the lowest pid of the best (lowest) priority. nullptr when none.
  const process_ptr *pick_first();

  // --- Counts and lists ---

  /// Waiting with a wake_time / waiting with a recv_path.
  std::size_t sleeping_count() const { return sleeping_; }
  std::size_t receiving_count() const { return receiving_; }
  /// Live processes with a deadline: sleepers and timed waits.
  std::size_t timed_count() const { return timed_; }
  /// Waiting for the next frame (awaiting_frame).
  std::size_t awaiting_count() const { return awaiting_.size(); }
  /// Live processes with state watches.
  const std::vector<process *> &watchers() const { return watchers_; }

  // --- Timers ---

  /// The earliest live deadline (wake_time, or recv_deadline of a timed wait), or nullopt.
  std::optional<clock::time_point> next_timer();
  /// Whether some timer may be due at `now` (a lower bound: an entry may be stale).
  bool timer_due(clock::time_point now) const { return !heap_.empty() && heap_.front().at <= now; }
  /// Wake what is due at `now`: a sleeper becomes ready, a timed wait expires
  /// (expire_await: its call raises TimeoutError) and becomes ready. Returns how many woke.
  std::size_t wake_due(clock::time_point now);

  // --- Waiters ---

  /// Move the processes waiting on `path` to `out` (appended), out of the receiver index. The
  /// caller resets their recv_path and readies them (set_status), which updates the rest.
  void take_receivers(const std::string &path, std::vector<process *> &out);
  /// Move the frame waiters to `out` likewise; the caller clears awaiting_frame and readies them.
  void take_awaiting(std::vector<process *> &out);

  /// Whether every index agrees with the processes' fields (O(n); for tests).
  bool check() const;

private:
  static constexpr std::uint32_t none = process::table_entry::none;
  struct level {
    int key = 0;
    std::uint32_t count = 0;
    std::vector<std::int32_t> fen; // Fenwick tree over the slots, 1-based, size cap_ + 1
  };
  struct timer_entry {
    clock::time_point at;
    int pid;
  };
  struct bucket {
    std::string path;
    std::vector<process *> procs;
  };

  level *level_for(int key, bool create);
  level *first_nonempty();
  std::uint32_t kth(const level &l, std::uint32_t k) const;
  void fen_add(level &l, std::uint32_t slot, std::int32_t delta);
  void assign_slot(process &p, const process_ptr *pp);
  void release_slot(process &p);
  void regrow();
  void enqueue(process &p, int key);
  void dequeue(process &p);
  void add_receiver(process &p);
  void remove_receiver(process &p);
  void free_bucket(std::uint32_t b);
  void push_timer(clock::time_point at, int pid);
  bool timer_live(const timer_entry &t) const;

  bool by_priority_;
  map_type map_;

  // Run queue: slots in pid order (null once a process ended), and a Fenwick tree per level.
  std::vector<const process_ptr *> slots_;
  std::uint32_t cap_ = 0;
  std::uint32_t dead_ = 0;
  std::vector<level> levels_; // sorted by key
  std::size_t queued_ = 0;
  std::uint32_t last_pick_ = none; // the slot pick_round_robin returned last (a hint)

  std::size_t sleeping_ = 0;
  std::size_t receiving_ = 0;
  std::size_t timed_ = 0;

  std::vector<timer_entry> heap_; // min-heap on `at`; entries go stale, checked when popped

  std::vector<bucket> buckets_;
  std::vector<std::uint32_t> free_buckets_;
  std::unordered_map<std::string, std::uint32_t> bucket_of_;

  std::vector<process *> awaiting_;
  std::vector<process *> watchers_;
};

/// A wakeup a thread can block on until another thread notifies it or a time comes, the time
/// kept to well under a millisecond (Windows: a high-resolution waitable timer where there is
/// one, since a timed wait on a condition variable there lasts whole ~15.6 ms timer ticks).
class wakeup {
public:
  using clock = std::chrono::steady_clock;
  wakeup();
  ~wakeup();
  wakeup(const wakeup &) = delete;
  wakeup &operator=(const wakeup &) = delete;
  /// Any thread: wake a wait_until() in progress, or the next one.
  void notify();
  /// Block until notify() (true; the notification is consumed) or `t` (false). A `precise`
  /// wait (a sleeper's deadline) ends on time: on Windows, whose high-resolution timer fires on
  /// a ~0.5 ms grid, its timer is set early and the last stretch (at most ~0.6 ms) is spent
  /// spinning on the CPU, watching for a notify(). Any other wait may end up to that grid late
  /// and burns no CPU.
  bool wait_until(clock::time_point t, bool precise = false);

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

/// Counts the changes under a state tree's root (its childChanged and valueChanged signals,
/// which a write anywhere under it fires, on whichever thread writes). Two equal counts mean
/// nothing changed in between. Connected only while a scheduler needs it: every write under the
/// root pays for the slot.
class tree_changes {
public:
  tree_changes();
  ~tree_changes();
  tree_changes(const tree_changes &) = delete;
  tree_changes &operator=(const tree_changes &) = delete;
  /// Count the changes under `root` from now on (nullptr: stop counting). Watching the root
  /// it already watches does nothing, unless that node went away meanwhile (see attached()).
  void watch(cvc::state *root);
  cvc::state *root() const { return root_; }
  /// Whether the slots are still connected to root()'s signals: false once that node is gone
  /// (its signals destroyed), even if another node now has its address.
  bool attached() const;
  std::uint64_t count() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
  cvc::state *root_ = nullptr;
};

/// The schedulers' settings live under the `state_exec` node of their tree (state_exec.defaults.*
/// and state_exec.schedulers.<id>.*). version() is a number that moves whenever one of them may
/// have changed: a write under that node (its change signals), or the node coming, going or
/// being replaced. A scheduler re-reads its settings only when it moved, so a submission or a
/// charge doesn't walk the tree for each setting, and writes elsewhere in the tree pay nothing.
/// It costs one lookup of the `state_exec` node per call. Like the watches, it relies on writes
/// firing the tree's signals: one that doesn't (state::reset with fireCallbacks=false, a node
/// operator() creates without a value) is seen at the next write under the node that does.
class settings_changes {
public:
  std::uint64_t version(cvc::state *root);

private:
  tree_changes changes_;
};

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_PROCESS_TABLE_H

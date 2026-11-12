/**
 * @file detail/lock_home.h
 * @brief A scheduler's lock home: protocol section 5 (docs/STATE_EXEC_LOCKS_PROTOCOL.md), the
 *        home actions between a scheduler, its processes' evaluators and the lock manager.
 *
 * Internal. One home per scheduler, made when the scheduler first needs one (design 8.4). It owns
 * the private state of protocol 2.2 for its processes (process::locks and the lock wait fields),
 * a key cache (design 8.4), its port (protocol 2.3), and the calls into the manager's table. It
 * runs on the thread that pumps its scheduler (protocol 0: for a pycvc Exec with no run() active,
 * whichever thread holds its control mutex); debug builds assert that no two threads run one
 * home's actions at once. Nothing here is thread-safe but lock_port, whose post() any thread
 * calls, the per-step reads port_pending() and store_progress(), and blocking_sinks().
 */
#ifndef CVC_STATE_EXEC_DETAIL_LOCK_HOME_H
#define CVC_STATE_EXEC_DETAIL_LOCK_HOME_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc::state_exec {

struct process;
struct eval_frame;
struct evaluator_state;
struct intrinsics_context;

namespace detail {

using lock_clock = std::chrono::steady_clock;

struct home_request;
class lock_home;
class wakeup; // process_table.h

// ---------------------------------------------------------------------------------------------
// The port (protocol 2.3)
// ---------------------------------------------------------------------------------------------

struct lock_fuzz; // below

/// A home's port: the posts that operations on other homes' threads make for its processes. Held
/// by its home and by every notification captured for it, so a poster keeps it (and the wakeup
/// it shares with its scheduler) alive while the scheduler is destroyed. Its mutex is its own,
/// never a scheduler's: post, close and drain take it, and nothing but the wakeup's notify() is
/// called under it.
class lock_port {
public:
  /// `wake`: the scheduler's wakeup, notified by every post (null: none to notify). `fuzz`: its
  /// manager's fuzzing of the protocol points (null: none).
  explicit lock_port(std::shared_ptr<wakeup> wake, std::shared_ptr<lock_fuzz> fuzz = nullptr);
  ~lock_port();
  lock_port(const lock_port &) = delete;
  lock_port &operator=(const lock_port &) = delete;

  /// Any thread: append `x`, set pending and notify the wakeup, all under the port's mutex; false
  /// (and nothing appended) when the port is closed. The closed test, the append and the signal
  /// are one critical section, so a post either lands before close() or is dropped.
  bool post(const lock_post &x);
  /// The home's thread, as the home is destroyed (protocol 5.7): no post lands after it.
  void close();
  bool closed() const;
  /// The home's thread: swap the queue into `out` (cleared first, so its capacity is reused) and
  /// clear pending, under the port's mutex.
  void drain(std::vector<lock_post> &out);
  /// Whether a post landed since the last drain. A relaxed load: a post racing it is seen at the
  /// next test, and its notify() ends any wait begun meanwhile.
  bool pending() const noexcept { return pending_.load(std::memory_order_relaxed); }

private:
  mutable std::mutex mu_;
  std::vector<lock_post> queue_;
  bool closed_ = false;
  std::atomic<bool> pending_{false};
  std::shared_ptr<wakeup> wake_;
  std::shared_ptr<lock_fuzz> fuzz_;
};

/// The protocol points where CVC_EXEC_FUZZ=<seed> yields (phase L3.7, LT-4; gap G-27).
enum class lock_fuzz_at : std::uint8_t {
  after_op,          // after a manager operation, before its notes are delivered
  post_enter,        // lock_port::post, before its critical section
  post_exit,         // lock_port::post, after its critical section
  drain_between,     // lock_home::drain_port, between two posts
  close_then_detach, // ~lock_home, between the port's close and detach_home
};
/// A lock manager's fuzzing of its protocol points (lock_manager::set_fuzz_seed): its seed (0:
/// off), how many points were reached with a seed in effect, and the stream its yields are drawn
/// from, which every thread at the manager's points advances. The manager's own, shared with its
/// homes' ports: nothing is shared between managers.
struct lock_fuzz {
  std::atomic<std::uint64_t> seed{0};
  std::atomic<std::uint64_t> hits{0};
  std::atomic<std::uint64_t> stream{0};
};
/// CVC_EXEC_FUZZ=<seed>, read now (a manager reads it as it is made): decimal or 0x hex; any other
/// non-empty value is hashed, so it turns the yields on too. Unset, empty or 0: 0 (off).
std::uint64_t lock_fuzz_seed_from_environment();
/// At a protocol point, with `fuzz`'s seed set: the thread runs on, yields, or sometimes pauses
/// 0-50 us (yielding until the time has passed), as the stream says. Off (or null): one relaxed
/// load.
void lock_fuzz_point(lock_fuzz *fuzz, lock_fuzz_at where) noexcept;

// ---------------------------------------------------------------------------------------------
// The lock frames (design 8.5)
// ---------------------------------------------------------------------------------------------

/// The extra_vals layout of a lock frame (eval_phase::lock_enter and lock_body): the lock form's
/// operations; the keys as written (busy path only: the fast path leaves nil, gap G-34); the
/// :else form; the wait's handle, then its outcome; the flags; the :as name; the hold id (or nil);
/// then the body's forms.
enum class lock_frame_slot : std::size_t {
  ops = 0,
  keys = 1,
  else_form = 2,
  wait_or_outcome = 3,
  flags = 4,
  as_name = 5,
  hold = 6,
  body = 7
};

/// The data_object type of the with-lock binding (a lock_ops). The codec never writes its payload.
inline constexpr const char *kLockOpsType = "lock_ops";
/// The data_object type of a lock wait's handle (gap G-1). The codec writes it as nil.
inline constexpr const char *kLockWaitType = "lock_wait";

/// What the with-lock binding holds: the intrinsics context of the environment it was registered
/// in (its scheduler, chroot, policies), and whatever must stay alive with it (pycvc's copy keeps
/// its Python owner, LK-23).
struct lock_ops {
  intrinsics_context *ctx = nullptr;
  std::shared_ptr<void> keep;
};

/// The lock_ops a value holds (one variant-index test, then the type name), or null.
const lock_ops *lock_ops_of(const value_t &v);
/// The with-lock binding for `ctx`; `keep` stays alive with it.
value_t make_lock_ops(intrinsics_context *ctx, std::shared_ptr<void> keep = nullptr);

/// The lock-enter token (design 8.5): what the evaluator hands the one call that applies
/// %lock-wait for a lock form (lock_wait_call's `token`), so the wait can reach that form's
/// lock_enter frame; a direct call of %lock-wait by a program has none (LW-2). It never outlives
/// that call.
struct lock_enter_token {
  evaluator_state *state = nullptr;
  std::size_t frame = 0; // the lock_enter frame's index
  int pid = -1;          // the process applying it
};

/// A lock key as a program wrote it, made absolute (design 4.1): split on '.', each segment
/// trimmed, empty ones dropped; an empty key, or a '#' in it, is a ValueError. A leading '/' is
/// app-global, allowed where a '/' channel is (else PermissionError; gap G-15); any other key is
/// under the chroot (ctx.root_path).
std::string resolve_lock_key(const intrinsics_context &ctx, std::string_view written,
                             const char *who);
/// The canonical request for KEY (a string, or a list of strings) as written, each key found in
/// `home`'s key cache or resolved (resolve_lock_key). Call home.begin_request() first.
void build_request(home_request &req, lock_home &home, const intrinsics_context &ctx,
                   const value_t &key, lock_mode mode);
/// %lock-wait's body (protocol 5.2): KEYS as written, MODE ("exclusive" or "shared"), TIMEOUT
/// (nil, or seconds). LockError where the process cannot wait, or with no lock-enter token.
value_t lock_wait_call(const lock_ops &ops, std::span<const value_t> args,
                       const lock_enter_token *token);
/// Bind with-lock, locks-held, lock-query and %lock-wait, and set-priority and get-priority, in
/// `env` (register_intrinsics does).
void register_lock_intrinsics(environment_ptr env, intrinsics_context *ctx);

// ---------------------------------------------------------------------------------------------
// The private state of protocol 2.2
// ---------------------------------------------------------------------------------------------

/// A hold in a process's mirror (protocol 2.2's MHold): a completed grant and how many sections
/// (the granting one and the covered ones nested in it) use it.
struct mhold {
  std::uint64_t id = 0;
  small_vector<lock_node *, 4> keys; // canonical; the hold keeps the nodes alive
  lock_mode mode = lock_mode::exclusive;
  std::uint32_t count = 1;
  std::uint64_t gen = 0;
  bool abandoned = false;
  bool in_handler = false; // made by a signal or watch handler (the handler rule, design 5.7)
};

/// A process's lock state (process::locks): its mirror and its ticket.
struct process_locks {
  std::vector<mhold> holds; // in acquisition order; reserved before a grant, never shrunk
  std::optional<std::uint64_t> ticket; // p.lock_ticket: set while parked, or woken not yet stepped
  lock_mode ticket_mode = lock_mode::exclusive;
  small_vector<std::string, 2> ticket_keys; // the ticket's keys, absolute, canonical
  bool ticket_in_handler = false;
  std::string ticket_root;              // the requester's chroot, for the keys an outcome shows
  std::optional<double> ticket_timeout; // for a TimeoutError's message
};

/// What a scheduler gives its home (design 8.4). Each is called on the scheduler's thread.
struct lock_sched_ops {
  virtual ~lock_sched_ops() = default;
  /// Park `p` (stepping now) as a lock waiter: close its running slice, status waiting, its
  /// deadline (if any) a timer. False when it can't be parked.
  virtual bool park_lock(process &p) = 0;
  /// A lock waiter's wait is over: ready it.
  virtual void ready_lock_waiter(process &p) = 0;
  /// Its inherited priority changed: re-level it in the run queue (protocol 5.9).
  virtual void relevel(process &p) = 0;
  /// The process table's flagged lock expiries (wake_due with no vector).
  virtual std::size_t lock_expired_count() const = 0;
  virtual void take_lock_expired(std::vector<process *> &out) = 0;
  /// A live process of this scheduler by pid, or null.
  virtual process *find_process(int pid) = 0;
  /// Every process of this scheduler, ended ones included.
  virtual void for_each_process(const std::function<void(process &)> &fn) = 0;
};

// ---------------------------------------------------------------------------------------------
// The home
// ---------------------------------------------------------------------------------------------

/// A request as a home takes it: the mode and the canonical keys (protocol 1), each with the
/// key-cache entry it was found under (`scope`, `written`), its node when the cache had it, and
/// the absolute key otherwise. Built with lock_home::lookup and canonicalize().
struct home_request {
  struct key {
    std::uint32_t scope = 0;
    std::string_view written;  // the key as the program wrote it (valid during the request)
    lock_node *node = nullptr; // a cache hit
    std::string absolute;      // a miss: the absolute canonical key
    std::string_view text() const { return node ? std::string_view(node->key) : absolute; }
  };
  lock_mode mode = lock_mode::exclusive;
  small_vector<key, 4> keys;
};

/// Sort `r`'s keys by their absolute keys and drop every key that an earlier one covers (an equal
/// key or an ancestor): protocol 1's canonical request.
void canonicalize(home_request &r);

/// What try_enter (protocol 5.1 steps 1-4) came to.
struct enter_result {
  enum class kind : std::uint8_t { granted, covered, busy };
  kind what = kind::busy;
  std::uint64_t hold = 0; // granted, covered: the hold (covered: the covering one)
  std::uint64_t gen = 0;
  bool abandoned = false; // granted only (a covered request reports false, I11)
};

/// The outcome a lock_enter frame keeps (protocol 2.2, 5.3): a granted hold id, or the
/// exception to raise (TimeoutError, DeadlockError).
struct frame_outcome {
  std::optional<std::uint64_t> hold;
  value_t error;
};

class lock_home {
public:
  /// Attach a home to `mgr` for a scheduler (`ops`) of `policy`: the home applies priorities
  /// (protocol 2.1's prio) exactly when the policy is priority or priority_rr (design 12). The
  /// home makes its port, sharing `wake` (the scheduler's wakeup; null: posts notify nothing), and
  /// its progress cell, and passes both and `attach` to the manager (protocol 4.4).
  lock_home(std::shared_ptr<lock_manager> mgr, lock_sched_ops &ops, scheduling_policy policy,
            std::shared_ptr<wakeup> wake = nullptr, lock_attach attach = {});
  /// Protocol 5.7: close the port, detach (every process of the home ended with abandon), deliver,
  /// discard the port's posts (lock_stats::posts_discarded), drop the port, then clear every
  /// process's lock fields. Runs before any other scheduler member is destroyed; never throws.
  ~lock_home();
  lock_home(const lock_home &) = delete;
  lock_home &operator=(const lock_home &) = delete;

  home_id id() const { return id_; }
  /// Whether the home applies priorities (its scheduler's policy is not round_robin).
  bool applies_priorities() const { return prio_; }
  lock_manager &manager() { return *mgr_; }
  const std::shared_ptr<lock_manager> &manager_ptr() const { return mgr_; }

  // -- building a request ----------------------------------------------------------------------
  /// Start a request: entries the request looks up are pinned until the next one starts.
  void begin_request();
  /// The interned id of a chroot (a request's scope).
  std::uint32_t scope_id(std::string_view root_path);
  /// The cached node of key `written` in `scope`, or null (a miss). Pins the entry.
  lock_node *lookup(std::uint32_t scope, std::string_view written);

  // -- protocol 5
  // ----------------------------------------------------------------------------------
  /// 5.1 steps 1-4 for `p` (stepping now). Steps 1-3 read only the mirror: the handler rule and
  /// the upgrade refusal raise LockError (reason "handler", "upgrade"; keys shown under `root`);
  /// a covered request bumps the covering hold's count. Step 4 tries the manager. Nothing after
  /// a grant can throw: the mirror's slot is reserved first.
  enter_result try_enter(process &p, home_request &req, std::string_view root);
  /// 5.2: block the request. Queued: `p` is parked (lock_waiting, its deadline, park_lock) and
  /// nullopt is returned. Granted or failed at once: the outcome, collected (5.9).
  std::optional<frame_outcome> wait(process &p, home_request &req, std::optional<double> timeout,
                                    std::string_view root);
  /// 5.3: the body of granted hold `hold` begins.
  void start_body(process &p, std::uint64_t hold);
  /// 5.4: a lock frame of `p` ends (its body completed, or an unwind passes it). `hold` is the
  /// lock_body's hold, or a lock_enter's granted hold; `lock_enter` withdraws a ticket still
  /// queued (the defensive path). noexcept.
  void release_frame(process &p, std::optional<std::uint64_t> hold, bool lock_enter) noexcept;
  /// 5.6: `p` ended (right after its status changed). `abandon`: a kill, a budget, a failure.
  void process_ended(process &p, bool abandon);
  /// 5.5: the lock waits whose deadlines passed (wake_due's vector, plus the flagged ones).
  void wait_expired(std::vector<process *> &expired);
  /// 5.8: `p`'s base priority changed (p.priority).
  void set_base(process &p);
  /// 4.14: apply notifications: this home's at once, another home's posted to its port
  /// (lock_stats::posts when it lands, posts_dropped when the port is closed).
  void deliver(const lock_notes &notes);
  /// 5.9: apply one post to this home's process: a wake (stale ones ignored), or a relevel, which
  /// reads the manager's current value and re-levels the process only when it changed.
  void apply_post(const lock_post &post);

  // -- the scheduler's per-step calls (protocol 5.9, design 8.4) --------------------------------
  /// Whether the port holds posts: the scheduler then calls drain_port() between steps, and its
  /// waits (wait_for_work, idle_wait) end.
  bool port_pending() const noexcept { return port_->pending(); }
  /// Apply the port's posts in FIFO order (5.9).
  void drain_port();
  /// The end-of-step clock reading of a step of this home's scheduler (a relaxed store; read only
  /// by the manager's blocking_sinks).
  void store_progress(lock_clock::time_point now) noexcept {
    progress_->store(now.time_since_epoch().count(), std::memory_order_relaxed);
  }
  const std::shared_ptr<lock_port> &port() const { return port_; }

  /// The manager's blocking_sinks for this home's `pid` (pycvc's idle test, design 9.2): each
  /// sink with its home's last progress, in lock_clock ticks.
  std::vector<std::pair<lock_pid, std::int64_t>> blocking_sinks(int pid) const;
  /// The same for several of this home's pids at once (pycvc's idle pass over its lock waits):
  /// one search under one M.mu hold; a sink that is one of `pids` is left out.
  std::vector<std::pair<lock_pid, std::int64_t>> blocking_sinks(std::span<const int> pids) const;

  // -- reads
  // ---------------------------------------------------------------------------------------
  /// Whether `p` holds a lock (its mirror is not empty).
  static bool holds_locks(const process &p);
  /// `abs` written as a program under chroot `root` would write it (protocol 5.10): relative when
  /// strictly under it, else with a leading '/'.
  static std::string show(std::string_view abs, std::string_view root);
  /// `p`'s held keys, absolute with a leading '/', comma-separated (a snapshot's refusal).
  static std::string describe_holds(const process &p);
  /// The first key of `p`'s ticket, absolute, for process_info::waiting_on; "" without one.
  static std::string waiting_key(const process &p);

  // -- checks
  // ---------------------------------------------------------------------------------------
  /// The home-side clauses of protocol section 6 (I2, I3, I4 home clauses, I6 second half, I8's
  /// base and quiescent clauses) over every process of the home, and the key cache against the
  /// manager's references. Throws lock_violation. For tests, at quiescent points.
  void check_quiescent() const;
  /// The key cache's capacity and how many entries it holds.
  std::size_t cache_capacity() const { return entries_.size(); }
  std::size_t cache_size() const { return index_.size(); }

private:
  // The key cache (design 8.4): a fixed array of entries (it never grows, so an entry's key
  // stays where it is) indexed by (scope, key as written). Each live entry holds one key-cache
  // reference of this home on its node (protocol 4.15). Eviction is CLOCK, skipping the entries
  // of the request being built (pinned), so a request never evicts its own keys.
  struct entry {
    std::uint32_t scope = 0;
    std::string written;
    lock_node *node = nullptr; // null while reserved for a miss, or free
    std::uint64_t pinned = 0;  // the request that last used it (pin_epoch_)
    bool used = false;         // CLOCK's reference bit
    bool live = false;         // in index_
  };
  struct cache_key {
    std::uint32_t scope = 0;
    std::string_view written; // into entries_[slot].written
  };
  struct cache_hash {
    std::size_t operator()(const cache_key &k) const noexcept;
  };
  /// The home token (debug builds; L3.1): held by a thread for the length of a home action, and
  /// re-entered by that thread when the action nests another (a delivery applying a post).
  class action;
  struct cache_eq {
    bool operator()(const cache_key &a, const cache_key &b) const noexcept {
      return a.scope == b.scope && a.written == b.written;
    }
  };

  lock_pid pid_of(const process &p) const;
  static rec_info info_of(const process &p);
  static process_locks &locks_of(process &p);
  /// The table request for `req`, and the cache slots its misses need: evictions go into
  /// `delta`; each miss that gets a slot asks the table for a reference (key_ref::ref).
  void build_table_request(const home_request &req, lock_request &treq, cache_delta &delta);
  /// After the table call: the reserved slots take the nodes the table filled in.
  void commit_misses(const lock_request &treq) noexcept;
  /// The table call threw: the reserved slots go back.
  void abandon_misses() noexcept;
  /// An entry to evict for a miss of `req`: never one of `req`'s own (pinned, reserved, or naming
  /// one of its keys' nodes through another spelling), so refs.drop never names a key of the
  /// request (protocol 4.15).
  std::optional<std::size_t> victim(const home_request &req);
  void free_entry(std::size_t slot);
  /// 5.9's collect: resume the ticket and keep the outcome; a grant joins the mirror.
  frame_outcome collect(process &p, std::uint64_t tid);
  /// 4.14 after an operation: its notes delivered, with the fuzz point between (L3.7) when there
  /// are any.
  void deliver_op(const lock_notes &notes);
  /// 5.9's apply_wake: a ticket's wait is over (granted, cancelled or failed).
  void apply_wake(int pid, std::uint64_t tid);
  void apply_relevel(int pid);
  value_t outcome_error(const lock_outcome &out, const process_locks &pl) const;

  std::shared_ptr<lock_manager> mgr_;
  lock_table &table_;
  lock_sched_ops &ops_;
  home_id id_ = 0;
  bool prio_ = false; // protocol 2.1's Home.prio
  std::shared_ptr<lock_port> port_;                     // Home.port
  std::shared_ptr<std::atomic<std::int64_t>> progress_; // Home.progress (lock_clock ticks)
  std::vector<lock_post> drained_;                      // drain_port's buffer, reused
  // The home token (debug builds only; present in every build so the layout never depends on
  // NDEBUG): the thread running this home's actions, and how deeply they nest on it.
  mutable std::atomic<std::thread::id> token_{};
  mutable std::uint32_t token_depth_ = 0;

  std::vector<entry> entries_;
  std::unordered_map<cache_key, std::size_t, cache_hash, cache_eq> index_;
  std::vector<std::size_t> free_;
  std::size_t hand_ = 0;        // CLOCK's hand
  std::uint64_t pin_epoch_ = 1; // the current request's pin
  std::map<std::string, std::uint32_t, std::less<>> scopes_;
  // The misses of the request being made: (index into its keys, slot), for commit_misses.
  small_vector<std::pair<std::size_t, std::size_t>, 4> misses_;
};

/// The lock frame's outcome as a value (protocol 2.2): the granted hold id, or the exception.
value_t outcome_value(const frame_outcome &o);
/// The outcome a lock_enter frame's slot holds, if any (nil before its wait is collected).
std::optional<frame_outcome> outcome_of(const value_t &slot);

} // namespace detail
} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_DETAIL_LOCK_HOME_H

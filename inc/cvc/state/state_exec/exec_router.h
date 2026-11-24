/**
 * @file exec_router.h
 * @brief Stage 2's router and pid directory (docs/STATE_EXEC_THREADED.md 4.5, 4.6; S2.md S2b): the
 * group each channel key is homed at, the group each pid is in, and the handle a group is posted
 * to by. One of each per app (app::exec_router()), beside its pid source.
 *
 * The rules S2b.0 freezes (S2.md S2-Q7..S2-Q10; change them only through the lead), as amended from
 * the router model's findings (tools/locks_model/router.py; S2b.0a):
 *
 * - **Homes.** A resolved channel key's home is the group that registered the longest prefix
 *   covering it (the key itself, or the prefix followed by '.'), else group 0, the app scheduler,
 *   which registers "" when app::exec_scheduler() builds it (a key homed at an unbuilt group 0
 *   builds it). A '#' key is never routed: it stays in the group that resolves it. Registering a
 *   prefix moves no waiter or message already parked on a key under it.
 * - **Islands.** A group that registered no prefix (a Stage 1 host's own group: has_prefix() is
 *   false) keeps every key no other group's prefix covers in its own scheduler, router on or off,
 *   as today; only a key under another group's prefix is routed from it. So a plain group's own
 *   messaging never changes home as the router turns on and off. Two plain groups exchange messages
 *   only on keys a group with a prefix homes (give one a prefix: channel_prefixes). When a prefix's
 *   home goes, its keys come home to group 0 for groups with prefixes and to each island itself;
 *   an island's waits parked at the dying home are rehomed to group 0 and served there, while its
 *   later waits on those keys are its own (a documented gap). An island homes nothing for another
 *   group, so its teardown forwards none of its own keys' messages: they die with it, counted, as a
 *   Stage 1 group's do.
 * - **When the router is on** (active()): while a group other than group 0 has a registered
 *   prefix, or a remote-waiter record exists at some home. Off, no scheduler consults it (RT-1):
 *   messaging is today's, one scheduler at a time.
 * - **The ordered ingress** (async_scheduler, S2b.1). post_message's messages, post_control's
 *   commands and router entries share one queue, applied in post order; the lock port's posts come
 *   before it in a drain, and settled async calls after it. A drain that throws puts back what it
 *   had not applied, ahead of later posts (S0.7); the entry that threw is not put back (it may
 *   have half happened) and is counted (throw_dropped): a command still gets done(false), and only
 *   a deliver that did not patch its frame goes back to its home as a stale one does (a requeue,
 *   or dropped when requeue_if_stale is false), with a subscribe for its wait (the same seq): the
 *   process still waits, and its record was used.
 * - **The protocol** (td 4.6, route_entry's kinds). msg-send in S on a key homed at H != S posts
 *   `send` to H; H applies it as a local msg-send there: each current waiter gets it, a local one
 *   at once, a remote one by a `deliver` to its group (one envelope copy per receiving group);
 *   with none, it joins H's pending FIFO. msg-recv in R on a key homed at H != R parks the process
 *   with a new recv_seq and posts `subscribe`; H answers from its pending FIFO with a `deliver`,
 *   or records a remote waiter. R applies a `deliver` by pid and seq (async_scheduler::deliver_to);
 *   a stale one (the process no longer waits with that seq) goes back to H as a `requeue`, which H
 *   gives to the next waiter (the earliest local waiter, else the earliest record), else puts at
 *   the front of the FIFO, past its cap. A deliver that the receiving group's closed handle refuses
 *   is stale at once: H applies it as that requeue (or drops it when requeue_if_stale is false). A
 *   timed-out, killed or reaped remote wait posts `unsubscribe`.
 * - **Broadcasts.** A send that a local waiter at H took sends its remote copies with
 *   requeue_if_stale false (a stale one is dropped, counted). One that reached no local waiter and
 *   one remote waiter is untracked (broadcast 0, a stale copy requeued). One that reached no local
 *   waiter and k >= 2 remote waiters is tracked at H by an id unique in the app
 *   (exec_router::new_broadcast_id): each copy answers `taken` or `requeue`; if every copy comes
 *   back stale H applies it once as a requeue, and a stale copy of one already taken is dropped. A
 *   home torn down with copies outstanding forwards its tracker (`adopt`), which the new home takes
 *   over, so the copies' late answers find it. So nothing is lost and nothing is delivered twice. A
 *   requeue or taken naming an id its home does not track is applied as untracked.
 * - **The cap** (state_exec.groups.ingress_cap, default 65536, 0: none). Only `send` entries and a
 *   host's post_message count; past the cap they are refused (msg-send's {status "error", reason
 *   "ingress full"}; post_message returns false). Controls, deliver, requeue, subscribe,
 *   unsubscribe, taken, adopt, uncapped sends (rehomed messages), completions and lock posts never
 *   count. An uncapped send that finds no waiter joins the pending FIFO past max_pending_messages:
 *   a message a home accepted is never dropped by its move.
 * - **Teardown and rehoming.** A group's teardown, holding its token after the lock port's close
 *   and before the kills (async_scheduler::leave_router): (1) closes its handle: a post through it
 *   is refused from then on, and its poster re-resolves the home (home_of) and posts again, waiting
 *   (yielding) while the home is still this group; (2) sends what it holds to the keys' new homes,
 *   resolved without it or any other closed group (exec_router::new_home_of), in the order it took
 *   them: the pending messages of the keys it homed (uncapped sends, FIFO order), its remote-waiter
 *   records (subscribes with the same seq), its broadcast trackers (adopt), then its queued
 *   messages and router entries in queue order (a host post of a key it homed and a send as
 *   uncapped sends; a deliver as a stale one: a requeue to its home, or dropped when
 *   requeue_if_stale is false; the rest as they are); a '#' key's message dies with it, counted;
 *   (3) only then unregisters its prefixes, which ends the posters' wait. The forward waits on
 *   nothing, so the wait is bounded. Its kills then post unsubscribes for its processes' remote
 *   waits. Group 0's teardown (~app) drops, and counts, whatever is still posted to it.
 * - **Host posts.** async_scheduler::post_message on a key homed elsewhere (router on) forwards
 *   it to the home as a send. exec_pool::post_message posts to the key's home.
 * - **The owner rule** (S2-Q7, S2-Q8; scheduler_base::control_as). Locally, a caller with owner ""
 *   (an unowned process, a host) sees and controls every process, as today; one with owner X only
 *   X's: kill, pause, resume, ps and inspect. Across groups only a caller with a non-empty owner
 *   reaches a pid the directory gives that owner: kill, pause and resume are posted there and give
 *   #t ("requested"); anything else gives #f, as an unknown pid does.
 * - **The directory** is written by the scheduler that issues a pid, at execute, fork and restore
 *   (Stage 3: move), and at reap; a pid is in it from its creation until its reap, so a process
 *   made since its group's last publish is found.
 * - **Published info** (S2-Q9): each scheduler with a router publishes at the end of every run
 *   loop, whichever driver ran it; the rows are rebuilt when the process table's generation moved
 *   (create, reap, status, priority, lock holds), and steps, seconds and memory refresh at most
 *   every 100 ms.
 * - **Lock order:** a group_ref's lock, then the target's ingress mutex, then its wake notifier's
 *   lock, then the pool's ready-queue mutex. The router's and the directory's own mutexes are
 *   leaves: nothing is posted, and no scheduler is built, while either is held.
 */
#ifndef CVC_STATE_EXEC_EXEC_ROUTER_H
#define CVC_STATE_EXEC_EXEC_ROUTER_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/state/state_exec/scheduler_base.h> // control_cmd, published_process
#include <cvc/state/state_exec/types.h>          // value_t
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class app;
}

namespace cvc::state_exec {

class async_scheduler;
class exec_group;

namespace detail {
struct route_target; // exec_router.cpp: the scheduler a group_ref posts to, under its lock
struct route_entry;  // below
} // namespace detail

/// A group as the router and the directory name it: the app's group 0 (app::exec_scheduler()) or
/// an exec_group. Any thread may copy one and post through it. Its group's teardown closes it
/// before the scheduler goes: from then on a post through it returns false, never touching the
/// freed scheduler (a post holds the handle's lock while it pushes, and the close waits for it).
/// Empty (default-constructed): no group, and a post returns false.
class group_ref {
public:
  group_ref() = default;
  /// A handle for `sched`: group 0 when `group` is null. Its owner makes it once, gives it to the
  /// scheduler (async_scheduler::set_router), and closes it in its teardown.
  static group_ref make(async_scheduler &sched, exec_group *group);
  /// Refuse every post from now on, waiting for one in progress. Idempotent; noexcept.
  void close() const noexcept;
  /// Closed, or empty.
  bool closed() const noexcept;
  explicit operator bool() const noexcept { return t_ != nullptr; }
  bool is_group0() const noexcept;
  /// The exec_group, or null for group 0 (and an empty handle). For identity and logs: nothing is
  /// called through it.
  exec_group *group() const noexcept;
  /// Its group's creation order in its app (group 0: 0): deterministic orders, logs.
  std::uint64_t id() const noexcept;
  bool operator==(const group_ref &o) const noexcept { return t_ == o.t_; }
  bool operator!=(const group_ref &o) const noexcept { return t_ != o.t_; }

  /// Push `e` into the group's ordered ingress (async_scheduler::post_route), any thread: false
  /// when the handle is closed, or the group refused it (a capped send at its cap). Never waits
  /// for a pump.
  bool post(detail::route_entry e) const;
  /// async_scheduler::post_message through the handle (a host post: it counts against the cap):
  /// false once closed, or refused.
  bool post_message(const std::string &key, const value_t &value) const;
  /// async_scheduler::post_control through the handle: false once closed, and then `cmd`'s done
  /// is never called (the caller knows from the result).
  bool post_control(int pid, control_cmd cmd) const;
  /// While the handle is open, run `fn` with its scheduler, which stays alive for the call, and
  /// return true; false once closed. For the scheduler's any-thread members only (published_info,
  /// ingress_counters, ...): `fn` must not post through this handle, nor block on the group.
  bool visit(const std::function<void(async_scheduler &)> &fn) const;

private:
  std::shared_ptr<detail::route_target> t_;
};

namespace detail {
/// A router entry (td 4.6, S2-Q10), posted to a group's ordered ingress and applied in its drain,
/// in post order with its messages and commands. `key` is always the resolved channel key.
struct route_entry {
  enum class kind : std::uint8_t {
    /// key, value: a msg-send (or a host's post) for the key's home H, applied there as a local
    /// msg-send; counts against H's cap unless `uncapped`.
    send,
    /// key, from, pid, seq, deadline: pid, in group `from`, waits on key with that seq; H answers
    /// with a deliver from its pending FIFO, or records a remote waiter.
    subscribe,
    /// key, from, pid, seq: that wait ended (a timeout, a kill); H drops its record. A record
    /// already used means a deliver is in flight: it comes back as a requeue if stale.
    unsubscribe,
    /// key, from (the home), pid, seq, value, broadcast, requeue_if_stale: for pid if it still
    /// waits on key with that seq (deliver_to); else stale: a requeue to `from` (or dropped,
    /// counted, when requeue_if_stale is false).
    deliver,
    /// key, value, broadcast: a stale delivery back at its home: the next waiter, else the front
    /// of the pending FIFO, past max_pending_messages; never capped.
    requeue,
    /// key, broadcast: a tracked broadcast copy reached its waiter.
    taken,
    /// key, value, broadcast, outstanding, taken: a tracker its home's teardown forwards (S2b.0a);
    /// the new home tracks the broadcast from there, so its copies' late answers find it.
    adopt,
  };
  kind what = kind::send;
  std::string key;
  value_t value;
  group_ref from;
  int pid = -1;
  std::uint64_t seq = 0;
  /// subscribe: the timed wait's deadline (H skips a record past it), or none.
  std::optional<std::chrono::steady_clock::time_point> deadline;
  /// deliver, requeue, taken: a tracked broadcast's id at its home; 0: untracked.
  std::uint64_t broadcast = 0;
  /// deliver: a stale copy goes back as a requeue; false when a local waiter at the home took the
  /// message, so a stale copy is dropped.
  bool requeue_if_stale = true;
  /// send: never refused, never counted, and past max_pending_messages when it finds no waiter (a
  /// rehomed message).
  bool uncapped = false;
  /// adopt: the copies that have not answered yet, and whether one was taken.
  std::uint32_t outstanding = 0;
  bool taken = false;
};
} // namespace detail

/// One pid's directory entry: its group and owner (td 4.5).
struct directory_entry {
  int pid = -1;
  group_ref group;
  std::string owner;
};

/// The pid directory (td 4.5, S2-Q8): the group each process of the app is in, from its creation
/// until its reap, with its owner. 64 shards, each a mutex and a map. Any thread.
class exec_directory {
public:
  exec_directory();
  ~exec_directory();
  exec_directory(const exec_directory &) = delete;
  exec_directory &operator=(const exec_directory &) = delete;

  /// The scheduler that issued `pid` (execute, fork, restore; Stage 3: move). Replaces an entry
  /// for the same pid (a debug build asserts there is none).
  void add(int pid, const group_ref &group, const std::string &owner);
  /// At reap. Nothing for an unknown pid.
  void remove(int pid) noexcept;
  std::optional<directory_entry> find(int pid) const;
  /// The groups with a process of `owner` (each once, by group id): pool.kill_owner's targets.
  std::vector<group_ref> groups_of(const std::string &owner) const;
  std::size_t size() const;

private:
  struct shard;
  std::unique_ptr<shard[]> shards_;
};

/// The app's router (td 4.6, S2-Q10): channel homes by registered prefix, and the host entry points
/// the pool forwards to. Any thread, unless a member says otherwise.
class exec_router {
public:
  explicit exec_router(cvc::app *app);
  ~exec_router();
  exec_router(const exec_router &) = delete;
  exec_router &operator=(const exec_router &) = delete;

  /// `group` is the home of the keys under `prefix` ("" is group 0's, the default home). Group 0
  /// registers "" when it is built; Ariadne a document group its prefix, a pooled Exec its chroot
  /// (exec_group::options::channel_prefixes, registered by the group's constructor). The epoch
  /// moves.
  void register_prefix(const group_ref &group, std::string prefix);
  /// Drop every prefix `group` registered (its teardown, async_scheduler::leave_router). The epoch
  /// moves.
  void unregister(const group_ref &group);
  /// The key's home: the longest registered prefix covering it, else group 0 (built now if it is
  /// not yet). Uncached: each scheduler caches its lookups, a capped map checked against epoch().
  /// Counted in consulted(). Not for a '#' key (never routed).
  group_ref home_of(std::string_view key) const;
  /// home_of, with every closed group's prefixes left out: where a teardown forwards what it holds
  /// (exec_router.h, "Teardown and rehoming"). Not counted in consulted().
  group_ref new_home_of(std::string_view key) const;
  /// The group whose registered prefix (not group 0's "") covers `key`, the longest; empty when
  /// none does. Never builds group 0: an island's lookup (its keys no prefix covers are its own).
  /// Counted in consulted().
  group_ref prefix_home_of(std::string_view key) const;
  /// Whether `group` registered a prefix (group 0's "" counts): a group that did not is an island
  /// (the header's rule), whose keys no other group's prefix covers stay its own.
  bool has_prefix(const group_ref &group) const;
  /// A tracked broadcast's id, unique in the app (never 0).
  std::uint64_t new_broadcast_id() noexcept;
  /// Whether the router is on (the header's rule): two atomic loads.
  bool active() const noexcept;
  /// Moves at every registration and unregistration: a scheduler's cache of homes is stale once it
  /// differs.
  std::uint64_t epoch() const noexcept;
  /// home_of() calls so far (RT-1: none with one group).
  std::uint64_t consulted() const noexcept;
  /// A home recorded a remote waiter (+1), or dropped or used one (-1): the router stays on while
  /// any exists.
  void note_remote_waiters(int delta) noexcept;

  exec_directory &directory() noexcept { return directory_; }
  const exec_directory &directory() const noexcept { return directory_; }
  cvc::app *app() const noexcept { return app_; }

  // --- The host entry points (exec_pool's, S2b) ---

  /// Post `value` on resolved key `key` to its home as a host's post (exec_pool::post_message):
  /// group 0's post_message while the router is off. False when the home refused it (its cap).
  bool post_message(const std::string &key, const value_t &value);
  /// Post `cmd` to the group the directory has `pid` in: true when it took it (its done is then
  /// called once, there); false for a pid not in the directory or a closed group (done not called).
  bool post_control(int pid, control_cmd cmd);
  /// `pid`'s row as its group last published it; nullopt for a pid not in the directory or not
  /// published since its creation.
  std::optional<published_process> info(int pid) const;

private:
  struct impl;
  cvc::app *const app_;
  exec_directory directory_;
  std::unique_ptr<impl> impl_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EXEC_ROUTER_H

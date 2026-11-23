/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_STATE_SYNC_ADAPTER_H__
#define __CVC_STATE_SYNC_ADAPTER_H__

#include <atomic>
#include <boost/signals2/connection.hpp>
#include <chrono>
#include <cvc/core/namespace.h>
#include <cvc/state/state_change_journal.h>
#include <cvc/state/state_codec_registry.h>
#include <cvc/state/state_subscription_router.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

class app;
class state;
class state_data_hydrator;
class hybrid_clock;

// ----------------
// cvc::state_sync_adapter
// ----------------
// Purpose:
//   Bridge between a local cvc::state tree and the distributed-state
//   pipeline. Observes value/data changes on a state subtree, records
//   them in a state_change_journal, and dispatches notifications to
//   path-prefix subscribers via a state_subscription_router. Also
//   accepts inbound mutations from remote peers and applies them to
//   the local tree under a re-entrancy guard so that they do not
//   re-enter the journal.
//
// Threading:
//   Thread-safe. All public methods may be called concurrently.
//   Signal callbacks from boost::signals2 are dispatched on the
//   mutating thread; journal/router operations are guarded by an
//   internal mutex distinct from those owned by the journal/router.
//   The slots are not safe for writers on other threads, though
//   (STATE_EXEC_THREADED.md 4.8): they re-walk the tree from the root
//   with raw, creating lookups, and read `_attached` unlocked. In
//   threaded Stage 1 only host-thread writers reach the subtree it
//   observes (`conf.*`: host_proxied natives and host code). Pinned
//   walks and an atomic `_attached` are the follow-up before a
//   worker-pumped group may write there.
//
// Scope:
//   This unit deliberately knows nothing about the wire protocol or
//   the network transport. Phase 2 wires it to a codec + blob store,
//   phase 3 to gRPC.
//
// Deletes:
//   A node taken out of the observed subtree (state::removeChild: a (state-delete), a
//   (state-set-tree) dropping the children its value does not name, a list row going, a world
//   letting an entity go) is journaled as ONE delete_subtree at its path, stamped with the clock's
//   time (set_clock), so a peer's copy loses it as a write would change it. A remote delete is
//   applied, never journaled. state::reset(false) drops children without telling anyone, so it
//   replicates nothing.
//
//   Each delete this adapter applies, local or remote, leaves a tombstone: its path and time.
//   buried(m) says a remote write at or below one is older than the delete, so its writer made it
//   before it saw the delete, and it must not bring the subtree back
//   (state_cluster_shard::ingest_remote drops it). A write made after its writer saw the delete is
//   later on any clock that merged the delete's time, so it makes the node again, as meant.
//
class state_sync_adapter {
public:
  // Callback invoked when a local mutation is observed and journaled.
  // Receives a const reference to the freshly appended mutation. The
  // adapter holds no lock when this fires.
  using on_local_mutation_func = std::function<void(const state_mutation &)>;

  // Callback invoked when a remote mutation has been applied to the
  // local tree (used for fanning out to other peers / logging).
  using on_remote_applied_func = std::function<void(const state_mutation &)>;

  // Construct an adapter rooted at `root_path` in the state tree of
  // app `ctx`. An empty `root_path` means the entire tree. The
  // adapter does NOT take ownership of the app/state; the caller
  // must outlive the adapter.
  state_sync_adapter(app &ctx, std::string root_path, std::string local_node_id);

  ~state_sync_adapter();

  state_sync_adapter(const state_sync_adapter &) = delete;
  state_sync_adapter &operator=(const state_sync_adapter &) = delete;

  // Attach observers to the current subtree. Idempotent; calling
  // twice is a no-op for already-attached paths. New child nodes
  // discovered after attach() are wired up lazily, when their
  // ancestors' childChanged first names them. That comes after the
  // write that made the node, so a node first seen through a local
  // write has its content (value, and data() with a codec) journaled
  // then: the first write to a fresh node replicates too.
  void attach();

  // Detach all observers. After this, local mutations are no longer
  // journaled and remote mutations are no longer applied. Safe to
  // call from any thread.
  void detach();

  bool is_attached() const noexcept;

  // Accessors for the underlying components.
  state_change_journal &journal() noexcept { return _journal; }
  const state_change_journal &journal() const noexcept { return _journal; }

  state_subscription_router &router() noexcept { return _router; }
  const state_subscription_router &router() const noexcept { return _router; }

  // Configure notification callbacks. Replace any previous setting.
  void set_on_local_mutation(on_local_mutation_func cb);
  void set_on_remote_applied(on_remote_applied_func cb);

  // The codec registry used to (de)serialize node data() payloads for replication. Not owned;
  // typically the owning shard's registry. When null, data() changes are not replicated (value()
  // and child structure still are). Set before attach().
  void set_codec_registry(const state_codec_registry *codecs) noexcept { _codecs = codecs; }

  // The hydrator used to resolve blob-ref data() payloads that were offloaded to the blob store
  // because they exceeded the inline threshold. Not owned; typically the owning session's hydrator.
  // When null, a set_data mutation carrying a blob_ref is skipped (value()/structure still flow).
  // Set before attach(). An inline set_data payload never consults the hydrator.
  void set_data_hydrator(state_data_hydrator *hydrator) noexcept { _hydrator = hydrator; }

  // The clock that times local deletes (and their tombstones): typically the owning shard's. Not
  // owned. Without one a local delete goes out untimed (the shard stamps it as it sends it) and
  // leaves no tombstone. Set before attach().
  void set_clock(hybrid_clock *clock) noexcept { _clock = clock; }

  // Apply a mutation received from a remote peer. Marks the calling
  // thread as "applying remote" for the duration so that the
  // resulting state signal does NOT loop back into the journal.
  // Returns true if the mutation was applied successfully.
  bool apply_remote(const state_mutation &m);

  // Whether the remote mutation `m` is older than a delete at or above its path that this adapter
  // applied (see Deletes above): a timed write, touch or metadata change whose hlc_time is at most
  // the tombstone's. A delete, a delegation op and an untimed mutation (hlc_time 0) never are.
  bool buried(const state_mutation &m) const;

  // The tombstones, oldest first: each the delete that left it (path, origin, sequence, time).
  std::vector<state_mutation> tombstones() const;

  // How long a tombstone lasts, in the clock's time (default 10 min), and how many there may be
  // (default 4096: past that the oldest go). A delete takes the older tombstones below it with it.
  void set_tombstone_limits(std::size_t max_entries, std::chrono::milliseconds ttl);

  // Test/diagnostic helpers.
  std::size_t observed_paths() const noexcept;
  std::uint64_t local_mutation_count() const noexcept;
  std::uint64_t remote_mutation_count() const noexcept;

  // Phase 8 slice 4d: subscription forwarding through transparent
  // links. Returns subscriptions whose path_prefix covers `path`,
  // PLUS subscriptions registered at any link-side path that
  // transparently shadows `path` (or an ancestor of `path`).
  // Results are deduplicated by id. When no transparent link
  // aliases `path`, this is equivalent to router().subscriptions_for(path).
  std::vector<state_subscription> subscriptions_for_path(const std::string &path) const;

  // Count of subscriptions that were matched only via a
  // transparent-link alias (i.e., not by direct router lookup).
  // Cumulative across all subscriptions_for_path calls (a local mutation does not look its
  // subscriptions up yet: that walks the whole tree, and nothing dispatches to them).
  std::uint64_t forwarded_through_link_count() const noexcept;

  // Suppression guard: while held on a given thread, local state
  // changes on that thread to this adapter's app are NOT journaled, by
  // this adapter or another on the same app. Another app's changes are
  // (each app keeps its own suppressed threads). Useful for bulk loads
  // and tests. Re-entrant.
  class suppression_scope {
  public:
    explicit suppression_scope(state_sync_adapter &a);
    ~suppression_scope();
    suppression_scope(const suppression_scope &) = delete;
    suppression_scope &operator=(const suppression_scope &) = delete;

  private:
    state_sync_adapter &_adapter;
  };

private:
  struct connection_set {
    boost::signals2::connection value_changed;
    boost::signals2::connection child_changed;
    boost::signals2::connection data_changed;
    boost::signals2::connection comment_changed;
    boost::signals2::connection hidden_changed;
    boost::signals2::connection read_only_changed;
    boost::signals2::connection property_changed;

    void disconnect_all() {
      value_changed.disconnect();
      child_changed.disconnect();
      data_changed.disconnect();
      comment_changed.disconnect();
      hidden_changed.disconnect();
      read_only_changed.disconnect();
      property_changed.disconnect();
    }
  };

  // Attach observers to `s` whose full path is `full_path`.
  void attach_node(state &s, const std::string &full_path);
  // Disconnect and forget the observers at and below `full_path` (a removed subtree). With _mutex.
  // Returns whether there were any: only the first of the removal's childChanged slots (it fires
  // on every observed ancestor) finds them. A node made on the way to a deeper write has no slot
  // of its own, but the deeper node does.
  bool drop_subtree(const std::string &full_path);

  // Journal + dispatch the local removal of the node at `path`, and leave its tombstone.
  void journal_delete(const std::string &path);
  // Leave a tombstone for the delete `m` (timed: hlc_time != 0).
  void bury(const state_mutation &m);

  // Connect all per-node change signals (value/data/comment/hidden/readOnly/child) for `node`,
  // whose full dotted path is `full_path`, and return the resulting connection set. Does not touch
  // _connections; the caller emplaces the result. Each slot re-resolves the node by path (never
  // captures `node`), early-returns while applying a remote mutation, and journals + dispatches the
  // corresponding mutation.
  connection_set connect_node(state &node, const std::string &full_path);

  // Journal + dispatch a local change to the node at `path`: its value, or its data() payload
  // (skipped without a codec registry or a codec for its type).
  void journal_value(const std::string &path);
  void journal_data(const std::string &path);

  // Compose a full dot-path for a given child of `parent_full_path`.
  static std::string join_path(const std::string &parent_full_path, const std::string &child);

  // Fire on_local_mutation_func and router dispatch outside any lock.
  void dispatch_local(const state_mutation &m);

  // True if the calling thread is currently applying a remote
  // mutation to this app (or holds a suppression_scope on it).
  bool current_thread_suppressed() const noexcept;

  // The threads applying a remote mutation to the app, or holding a suppression_scope on it, with
  // their depths: the app's own (app::slot), shared by every adapter on the app.
  struct suppression;
  struct suppression_guard;

  app &_ctx;
  suppression &_suppression;
  std::string _root_path; // empty => whole tree
  std::string _local_node_id;

  state_change_journal _journal;
  state_subscription_router _router;
  const state_codec_registry *_codecs =
      nullptr; // not owned; data() replication codecs (may be null)
  state_data_hydrator *_hydrator =
      nullptr; // not owned; resolves blob-ref data() payloads (may be null)
  hybrid_clock *_clock = nullptr; // not owned; times local deletes (may be null)

  // The tombstones (see Deletes), defined in the .cpp over file-local element types, which keeps
  // their containers' template code out of the cvc.dll export table.
  struct tombstone_table;
  std::unique_ptr<tombstone_table> _tombstones;

  mutable std::mutex _mutex;
  // Written under _mutex, but the slots read it unlocked on the writing thread: a race once a
  // writer is not the host thread (STATE_EXEC_THREADED.md 4.8's follow-up makes it atomic).
  bool _attached;
  std::unordered_map<std::string, connection_set> _connections;

  on_local_mutation_func _on_local;
  on_remote_applied_func _on_remote;

  std::atomic<std::uint64_t> _local_count;
  std::atomic<std::uint64_t> _remote_count;
  mutable std::atomic<std::uint64_t> _forwarded_through_link_count{0};
};

} // namespace cvc

#endif // __CVC_STATE_SYNC_ADAPTER_H__

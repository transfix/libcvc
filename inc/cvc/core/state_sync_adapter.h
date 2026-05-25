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
#include <cvc/core/namespace.h>
#include <cvc/core/state_change_journal.h>
#include <cvc/core/state_subscription_router.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

class app;
class state;

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
//
// Scope:
//   This unit deliberately knows nothing about the wire protocol or
//   the network transport. Phase 2 wires it to a codec + blob store,
//   phase 3 to gRPC.
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
  // discovered after attach() are wired up lazily.
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

  // Apply a mutation received from a remote peer. Marks the calling
  // thread as "applying remote" for the duration so that the
  // resulting state signal does NOT loop back into the journal.
  // Returns true if the mutation was applied successfully.
  bool apply_remote(const state_mutation &m);

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
  // Cumulative across all dispatch_local / subscriptions_for_path calls.
  std::uint64_t forwarded_through_link_count() const noexcept;

  // Suppression guard: while held on a given thread, local state
  // changes on that thread are NOT journaled. Useful for bulk loads
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
  };

  // Attach observers to `s` whose full path is `full_path`.
  void attach_node(state &s, const std::string &full_path);

  // Compose a full dot-path for a given child of `parent_full_path`.
  static std::string join_path(const std::string &parent_full_path, const std::string &child);

  // Fire on_local_mutation_func and router dispatch outside any lock.
  void dispatch_local(const state_mutation &m);

  // True if the calling thread is currently applying a remote
  // mutation (or holds a suppression_scope).
  bool current_thread_suppressed() const noexcept;

  app &_ctx;
  std::string _root_path; // empty => whole tree
  std::string _local_node_id;

  state_change_journal _journal;
  state_subscription_router _router;

  mutable std::mutex _mutex;
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

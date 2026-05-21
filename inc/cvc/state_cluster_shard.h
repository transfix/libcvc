/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_CLUSTER_SHARD_H__
#define __CVC_STATE_CLUSTER_SHARD_H__

#include <cvc/namespace.h>
#include <cvc/state_authority_map.h>
#include <cvc/state_change_journal.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_codec_registry.h>
#include <cvc/state_replica.h>
#include <cvc/state_subscription_router.h>
#include <cvc/state_sync_adapter.h>
#include <cvc/state_write_policy.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

class app;

// ----------------
// cvc::state_cluster_shard
// ----------------
// Purpose:
//   A single ownership unit binding all per-tree distributed-state
//   components: change journal + subscription router (owned via the
//   adapter), sync adapter, replica metadata, authority map, and
//   codec registry. One shard represents one logical replicated
//   state tree from the point of view of a single node.
//
//   A multi-tenant process can host many shards (one per cluster it
//   participates in). Cross-shard routing is the transport layer's
//   responsibility.
//
// Lifecycle:
//   1. Construct with (app, cluster_id, local_node_id, root_path).
//      The shard creates a state_sync_adapter bound to the given
//      app/root, plus its own replica/authority/codecs.
//   2. attach() to start observing the subtree.
//   3. ingest_remote(mutation) for each inbound replicated change.
//      The shard checks loop-detection (replica.seen) and (when
//      enabled) authority, advances the vector clock, records the
//      seen-set entry, then applies via the adapter.
//   4. drain_local() returns local-origin mutations from the
//      journal that have not yet been published to peers, advancing
//      the publish cursor.
//
// Threading:
//   Thread-safe. ingest_remote / drain_local may be called
//   concurrently. Component accessors return stable references.
//
class state_cluster_shard {
public:
  struct ingest_result {
    bool applied = false;
    bool duplicate = false;     // already in replica seen-set
    bool rejected = false;      // failed authority check
    std::string reject_reason;  // populated when rejected
  };

  state_cluster_shard(app &ctx, std::string cluster_id,
                      std::string local_node_id,
                      std::string root_path = std::string());

  ~state_cluster_shard();

  state_cluster_shard(const state_cluster_shard &) = delete;
  state_cluster_shard &operator=(const state_cluster_shard &) = delete;

  const std::string &cluster_id() const noexcept { return _cluster_id; }
  const std::string &local_node_id() const noexcept {
    return _local_node_id;
  }
  const std::string &root_path() const noexcept { return _root_path; }

  // Component accessors. The shard owns all of these for its lifetime.
  state_sync_adapter &adapter() noexcept { return *_adapter; }
  state_change_journal &journal() noexcept { return _adapter->journal(); }
  state_subscription_router &router() noexcept {
    return _adapter->router();
  }
  state_replica &replica() noexcept { return *_replica; }
  state_authority_map &authority() noexcept {
    return _delegation->authority();
  }
  state_delegation_manager &delegation() noexcept { return *_delegation; }
  state_codec_registry &codecs() noexcept { return *_codecs; }
  state_message_bus &message_bus() noexcept { return *_message_bus; }
  state_write_policy &write_policy() noexcept { return *_write_policy; }

  // Wire up adapter observers and start journaling local changes.
  void attach();

  // Stop journaling local changes and applying remote changes.
  void detach();

  bool is_attached() const noexcept;

  // Apply a remote-origin mutation. The shard checks the seen-set
  // for loop detection; if seen, returns {applied=false,
  // duplicate=true}. It then records the mutation in the seen-set,
  // advances the vector clock for origin_node_id, applies via the
  // adapter, and updates the peer's last_applied_sequence.
  ingest_result ingest_remote(const state_mutation &m);

  // Admit a remote-origin out-of-band message into this shard's
  // local message bus. Performs (origin_node_id, message_id) dedup;
  // returns true if newly admitted (subscribers were invoked),
  // false if duplicate or if the cluster_id does not match this
  // shard. Messages are NOT journaled and do NOT advance any
  // vector clock or replica seen-set.
  bool ingest_remote_message(const state_message &m);

  // Return local-origin mutations (origin_node_id == local) that
  // have been journaled but not yet published to peers, advancing
  // the publish cursor. If max_count == 0, returns everything
  // pending.
  std::vector<state_mutation> drain_local(std::size_t max_count = 0);

  // Last drained local sequence (0 if none drained yet).
  std::uint64_t published_cursor() const;

  // Rewind the publish cursor; the next drain_local will re-emit
  // local mutations whose journal sequence is > `sequence`.
  void rewind_publish_cursor(std::uint64_t sequence);

  // Strictness toggle: when true, ingest_remote rejects mutations
  // whose paths resolve to an authority entry whose cluster_id
  // differs from this shard's cluster_id. Default false.
  void set_enforce_authority(bool enforce) noexcept;
  bool enforce_authority() const noexcept;

  // Phase 5: write-policy enforcement. When true, ingest_remote
  // consults this shard's write_policy(). A mutation whose
  // origin_node_id is not permitted to write the path is rejected
  // with reason "write policy ...". Default false.
  void set_enforce_write_policy(bool enforce) noexcept;
  bool enforce_write_policy() const noexcept;

  // Phase 5: deterministic last-writer-wins conflict resolution.
  // When true, ingest_remote tracks the most recently applied
  // mutation per path. If a new mutation is concurrent with the
  // current one and loses the (origin_node_id, sequence)
  // tie-breaker, the apply is skipped and counted as a conflict
  // loss. Default false.
  void set_resolve_conflicts(bool resolve) noexcept;
  bool resolve_conflicts() const noexcept;

  // Phase 5: counters for distributed observability. These are
  // cumulative since the shard was constructed; readers see the
  // current value at point of call.
  std::uint64_t total_remote_applied() const noexcept {
    return _ctr_remote_applied.load();
  }
  std::uint64_t total_remote_duplicates() const noexcept {
    return _ctr_remote_duplicates.load();
  }
  std::uint64_t total_remote_rejected() const noexcept {
    return _ctr_remote_rejected.load();
  }
  std::uint64_t total_conflicts_detected() const noexcept {
    return _ctr_conflicts_detected.load();
  }
  std::uint64_t total_conflicts_lost() const noexcept {
    return _ctr_conflicts_lost.load();
  }

  // Phase 6: subtree delegation. When true, ingest_remote consults
  // the delegation manager. A mutation whose path resolves to a
  // foreign cluster is rejected with reason "path delegated to
  // cluster <id>"; an expired lease is rejected with reason
  // "delegation lease expired ...". Default false.
  void set_enforce_delegation(bool enforce) noexcept;
  bool enforce_delegation() const noexcept;

  // Phase 6: classify a path against the delegation manager. This
  // is a thin wrapper around delegation().route(path) and is the
  // preferred entry point for clients that want to know whether a
  // local write should be applied here, forwarded, or held until a
  // lease is renewed.
  state_delegation_manager::route_decision
  route_path(const std::string &path) const {
    return _delegation->route(path);
  }

  std::uint64_t total_delegation_routed() const noexcept {
    return _ctr_delegation_routed.load();
  }
  std::uint64_t total_delegation_expired() const noexcept {
    return _ctr_delegation_expired.load();
  }

private:
  std::string _cluster_id;
  std::string _local_node_id;
  std::string _root_path;

  std::unique_ptr<state_sync_adapter> _adapter;
  std::unique_ptr<state_replica> _replica;
  std::unique_ptr<state_delegation_manager> _delegation;
  std::unique_ptr<state_codec_registry> _codecs;
  std::unique_ptr<state_message_bus> _message_bus;
  std::unique_ptr<state_write_policy> _write_policy;

  mutable std::mutex _mutex;
  std::uint64_t _publish_cursor = 0; // last drained local sequence
  bool _enforce_authority = false;
  bool _enforce_write_policy = false;
  bool _enforce_delegation = false;
  bool _resolve_conflicts = false;
  std::unordered_map<std::string, state_mutation> _last_path_mutation;

  std::atomic<std::uint64_t> _ctr_remote_applied{0};
  std::atomic<std::uint64_t> _ctr_remote_duplicates{0};
  std::atomic<std::uint64_t> _ctr_remote_rejected{0};
  std::atomic<std::uint64_t> _ctr_conflicts_detected{0};
  std::atomic<std::uint64_t> _ctr_conflicts_lost{0};
  std::atomic<std::uint64_t> _ctr_delegation_routed{0};
  std::atomic<std::uint64_t> _ctr_delegation_expired{0};
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_CLUSTER_SHARD_H__

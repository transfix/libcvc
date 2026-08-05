/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_CLUSTER_SHARD_H__
#define __CVC_STATE_CLUSTER_SHARD_H__

#include <atomic>
#include <cvc/core/namespace.h>
#include <cvc/core/state_authority_map.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/core/state_change_journal.h>
#include <cvc/core/state_codec_registry.h>
#include <cvc/core/state_delegation_manager.h>
#include <cvc/core/state_hash_partition.h>
#include <cvc/core/state_hybrid_time.h>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_replica.h>
#include <cvc/core/state_subscription_router.h>
#include <cvc/core/state_sync_adapter.h>
#include <cvc/core/state_write_policy.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

class app;
class state_transport;

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
    bool duplicate = false;    // already in replica seen-set
    bool rejected = false;     // failed authority check
    std::string reject_reason; // populated when rejected
  };

  state_cluster_shard(app &ctx, std::string cluster_id, std::string local_node_id,
                      std::string root_path = std::string());

  ~state_cluster_shard();

  state_cluster_shard(const state_cluster_shard &) = delete;
  state_cluster_shard &operator=(const state_cluster_shard &) = delete;

  const std::string &cluster_id() const noexcept { return _cluster_id; }
  const std::string &local_node_id() const noexcept { return _local_node_id; }
  const std::string &root_path() const noexcept { return _root_path; }

  // Component accessors. The shard owns all of these for its lifetime.
  state_sync_adapter &adapter() noexcept { return *_adapter; }
  state_change_journal &journal() noexcept { return _adapter->journal(); }
  state_subscription_router &router() noexcept { return _adapter->router(); }
  state_replica &replica() noexcept { return *_replica; }
  state_authority_map &authority() noexcept { return _delegation->authority(); }
  state_delegation_manager &delegation() noexcept { return *_delegation; }
  state_codec_registry &codecs() noexcept { return *_codecs; }
  state_message_bus &message_bus() noexcept { return *_message_bus; }
  state_write_policy &write_policy() noexcept { return *_write_policy; }
  hybrid_clock &clock() noexcept { return _clock; }
  state_hash_partition &partition() noexcept { return _partition; }
  const state_hash_partition &partition() const noexcept { return _partition; }

  // Blob store and inline payload threshold. When both are set,
  // drain_local() offloads mutation values larger than the threshold
  // to the blob store and replaces the payload with a blob_ref.
  void set_blob_store(state_blob_store *store) noexcept;
  state_blob_store *blob_store() const noexcept;
  void set_max_inline_payload_bytes(std::uint32_t bytes) noexcept;
  std::uint32_t max_inline_payload_bytes() const noexcept;

  // When true and the partition map is non-empty, ingest_remote()
  // rejects mutations whose path hashes to a different node_id.
  void set_enforce_partition(bool enforce) noexcept;
  bool enforce_partition() const noexcept;

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

  // Return local-origin mutations journaled after `after_sequence`,
  // stamped for the wire exactly as drain_local() stamps them, but
  // WITHOUT advancing the publish cursor.
  //
  // Transports use this to backfill a peer that connects mid-stream.
  // drain_local() advances the cursor whether or not the transport
  // had anywhere to send the mutations, so any write made while no
  // peer was connected is otherwise invisible to a peer that
  // connects later. Replaying is safe to repeat: the receiving
  // shard's replica seen-set drops (origin_node_id, sequence) pairs
  // it has already applied.
  std::vector<state_mutation> replay_local(std::uint64_t after_sequence = 0);

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
  std::uint64_t total_remote_applied() const noexcept { return _ctr_remote_applied.load(); }
  std::uint64_t total_remote_duplicates() const noexcept { return _ctr_remote_duplicates.load(); }
  std::uint64_t total_remote_rejected() const noexcept { return _ctr_remote_rejected.load(); }
  std::uint64_t total_conflicts_detected() const noexcept { return _ctr_conflicts_detected.load(); }
  std::uint64_t total_conflicts_lost() const noexcept { return _ctr_conflicts_lost.load(); }

  // Per-path conflict detail record.
  struct conflict_entry {
    std::string path;
    std::string winner_node_id;
    std::uint64_t winner_sequence = 0;
    std::string loser_node_id;
    std::uint64_t loser_sequence = 0;
  };

  // Return the most recent conflicts (up to `max_entries`, default 64).
  // The ring buffer is only populated when resolve_conflicts is true.
  std::vector<conflict_entry> recent_conflicts(std::size_t max_entries = 64) const;

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
  state_delegation_manager::route_decision route_path(const std::string &path) const {
    return _delegation->route(path);
  }

  std::uint64_t total_delegation_routed() const noexcept { return _ctr_delegation_routed.load(); }
  std::uint64_t total_delegation_expired() const noexcept { return _ctr_delegation_expired.load(); }

  // Phase 6: publish a delegation as a control-plane mutation. The
  // delegation is applied to this shard's local delegation manager
  // immediately, then journaled with op=delegate_subtree so the
  // next drain_local() emits it for peers. Receivers that ingest
  // this mutation install the same delegation in their own
  // delegation manager. `lease_duration_ns == 0` means infinite
  // lease.
  void publish_delegation(const std::string &path_prefix, const std::string &cluster_id,
                          const std::string &endpoint = std::string(),
                          std::uint64_t lease_duration_ns = 0);

  // Phase 6: publish a revocation. Removes the delegation locally
  // and journals an op=revoke_delegation mutation so peers do the
  // same.
  void publish_revocation(const std::string &path_prefix);

  // Phase 6: control-plane counters.
  std::uint64_t total_delegations_applied() const noexcept {
    return _ctr_delegations_applied.load();
  }
  std::uint64_t total_revocations_applied() const noexcept {
    return _ctr_revocations_applied.load();
  }

  // -------- Inbound interest filter (lazy-load enforcement) --------
  //
  // A cluster may carry more state than any single client can
  // afford to mirror. This filter is the receiver-side guard that
  // turns "client mirrors everything pushed at it" into "client
  // only materializes paths it has explicitly registered interest
  // in".
  //
  // Semantics:
  //   * add_interest(prefix) installs a path-prefix the local node
  //     is willing to receive. Prefix matching is dot-segment
  //     aware: "scene" matches "scene" and "scene.foo" but NOT
  //     "scenery". Empty prefix ("") matches everything.
  //   * When enforce_interest() is true, ingest_remote and
  //     ingest_remote_message reject mutations / messages whose
  //     path is not covered by any registered prefix. The
  //     mutation is recorded as filtered_out, the seen-set is
  //     NOT advanced (so a later interest-add can replay).
  //   * Default: enforce=false, interests empty. This preserves
  //     the previous "mirror everything" behavior so existing
  //     deployments are unaffected until they opt in.
  //   * Enabling enforce_interest with an empty interest set
  //     means "reject all inbound" \u2014 safe-by-default for
  //     deployments that want to whitelist explicitly.
  //
  // Threading: thread-safe.
  void add_interest(std::string path_prefix);
  bool remove_interest(const std::string &path_prefix);
  void clear_interests();
  std::vector<std::string> interests() const;
  bool path_is_of_interest(const std::string &path) const;

  void set_enforce_interest(bool enforce) noexcept;
  bool enforce_interest() const noexcept;

  std::uint64_t total_remote_filtered_out() const noexcept {
    return _ctr_remote_filtered_out.load();
  }

  // Snapshot: walk the local state tree under `path_prefix` and
  // return a vector of entries suitable for initial-sync.
  struct snapshot_entry {
    std::string path;
    std::string string_value;
    std::string comment;
    bool hidden = false;
    bool read_only = false;
    std::string type_name;
    std::string origin_node_id;
    std::uint64_t sequence = 0;
  };
  std::vector<snapshot_entry> snapshot(const std::string &path_prefix = std::string()) const;

  // -------- Phase 8 slice 2: cluster-agnostic message routing --------
  //
  // The shard owns the bridge from "I want to send a message at
  // path P" to "the right cluster, the local bus, and the wire".
  // Callers (notably cvc::state::sendMessage) never name a
  // cluster_id: the shard derives the owning cluster from its
  // authority map (longest-prefix) and falls back to this shard's
  // own cluster_id when there is no entry.
  //
  // Threading: thread-safe.

  struct send_message_result {
    enum class status_kind {
      delivered,       // routing succeeded; see counts for fan-out
      no_transport,    // owner is remote but no transport is set
      duplicate_local, // local bus reported a dedup hit
    };
    status_kind status = status_kind::delivered;
    std::string owner_cluster_id;    // resolved owner of the path
    bool owner_is_local = true;      // owner matches this shard
    std::size_t local_admitted = 0;  // 1 if local bus admitted it
    std::size_t peers_delivered = 0; // peer shards that admitted
    std::size_t peers_targeted = 0;  // peer streams attempted
  };

  // Send an out-of-band message. The caller does NOT need to
  // stamp m.cluster_id; this method does so based on the
  // authority map. If m.origin_node_id is empty, the shard's
  // local_node_id is used. A non-empty m.message_id is left
  // untouched; an empty one stays empty (dedup-bypass).
  send_message_result send_message(state_message m);

  // Optional back-pointer to the transport used for cross-shard
  // fan-out. When unset, send_message() still delivers to the
  // local message bus but cannot reach peers.
  void set_transport(state_transport *t) noexcept;
  state_transport *transport() const noexcept;

  // Per-app default shard registry. attach() installs this shard
  // as the default for its app context if no default is already
  // installed; detach() removes it iff it is the current default.
  // This is the lookup cvc::state::sendMessage uses to find a
  // shard without the caller naming a cluster.
  static state_cluster_shard *default_for(const app &ctx) noexcept;

  // Explicit install/uninstall in addition to attach()/detach().
  // Useful for tests that need to swap the default shard for an
  // app context without going through attach/detach plumbing.
  void install_as_default();
  void uninstall_as_default();

private:
  // Shared body of drain_local()/replay_local(): replay the journal
  // past `after_sequence`, keep local-origin mutations, and stamp
  // each one for the wire (cluster id, HLC time, blob offload).
  std::vector<state_mutation> collect_local(std::uint64_t after_sequence, std::size_t max_count);

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

  // Ring buffer of recent conflict entries.
  static constexpr std::size_t kMaxConflictRing = 128;
  std::vector<conflict_entry> _conflict_ring;
  std::size_t _conflict_ring_pos = 0;
  std::atomic<std::uint64_t> _ctr_delegation_routed{0};
  std::atomic<std::uint64_t> _ctr_delegation_expired{0};
  std::atomic<std::uint64_t> _ctr_delegations_applied{0};
  std::atomic<std::uint64_t> _ctr_revocations_applied{0};
  std::atomic<std::uint64_t> _ctr_remote_filtered_out{0};
  std::atomic<std::uint64_t> _ctr_partition_rejected{0};

  // Inbound interest filter.
  std::vector<std::string> _interests;
  bool _enforce_interest = false;

  // Phase 8 slice 2.
  state_transport *_transport = nullptr;
  app *_app_ctx = nullptr; // captured at construction for default_for()
  hybrid_clock _clock;
  state_hash_partition _partition;
  bool _enforce_partition = false;

  state_blob_store *_blob_store = nullptr;
  std::uint32_t _max_inline_payload_bytes = 0; // 0 = disabled
};

} // namespace cvc

#endif // __CVC_STATE_CLUSTER_SHARD_H__

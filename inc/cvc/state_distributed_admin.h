/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DISTRIBUTED_ADMIN_H__
#define __CVC_STATE_DISTRIBUTED_ADMIN_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace CVC_NAMESPACE {

class state;
class state_blob_store;
class state_cluster_shard;
class state_message_bus;
class state_peer_registry;

// ----------------
// cvc::state_distributed_admin
// ----------------
// Phase 6 (Performance & Production Hardening).
//
// Operator-facing inspection and maintenance facade for the
// distributed-state stack. The admin does not own any of the
// subsystems it observes; callers attach pointers to live
// instances. Detached subsystems are silently skipped in reports.
//
// The report struct is a plain data snapshot (cheap to copy, safe
// to serialize). to_text() renders it in a stable line-oriented
// format suitable for logs and CLI tooling. gc_blobs() removes
// blobs from an attached blob store that are not present in the
// caller-supplied live set.
//
// Threading:
//   snapshot() and to_text() are safe to call concurrently with
//   normal traffic on the attached subsystems (each underlying
//   accessor is thread-safe). attach_*() and detach_*() must not
//   race with snapshot() on the same admin instance.
//
class state_distributed_admin {
public:
  struct shard_report {
    bool attached = false;
    std::string cluster_id;
    std::string node_id;
    bool enforce_authority = false;
    bool enforce_write_policy = false;
    bool enforce_delegation = false;
    bool resolve_conflicts = false;
    std::uint64_t total_remote_applied = 0;
    std::uint64_t total_remote_duplicates = 0;
    std::uint64_t total_remote_rejected = 0;
    std::uint64_t total_conflicts_detected = 0;
    std::uint64_t total_conflicts_lost = 0;
    std::uint64_t total_delegation_routed = 0;
    std::uint64_t total_delegation_expired = 0;
  };

  struct delegation_entry {
    std::string prefix;
    std::string cluster_id;
    std::string endpoint;
    std::uint64_t expires_at_ns = 0; // 0 == never
  };

  struct peer_entry {
    std::string node_id;
    std::string cluster_id;
    std::string endpoint;
    std::vector<std::string> subscriptions;
    std::uint64_t last_seen_ns = 0;
    std::uint64_t mutations_delivered = 0;
    std::uint64_t messages_delivered = 0;
    std::uint64_t deliveries_filtered = 0;
  };

  struct bus_report {
    bool attached = false;
    std::uint64_t total_admitted = 0;
    std::uint64_t total_duplicates = 0;
    std::uint64_t total_dispatched = 0;
    std::uint64_t total_dropped = 0;
  };

  struct blob_report {
    bool attached = false;
    std::size_t count = 0;
    std::uint64_t bytes_stored = 0;
  };

  struct report {
    shard_report shard;
    std::vector<delegation_entry> delegations;
    std::vector<peer_entry> peers;
    bus_report bus;
    blob_report blobs;
  };

  struct gc_result {
    std::size_t scanned = 0; // candidates considered
    std::size_t removed = 0; // blobs erased
    std::uint64_t bytes_freed = 0;
  };

  state_distributed_admin() noexcept = default;

  state_distributed_admin(const state_distributed_admin &) = delete;
  state_distributed_admin &operator=(const state_distributed_admin &) = delete;

  // Subsystem attachment. Pass nullptr to detach. Lifetime of the
  // pointed-to objects is the caller's responsibility.
  void attach_shard(state_cluster_shard *shard) noexcept;
  void attach_peer_registry(state_peer_registry *peers) noexcept;
  void attach_blob_store(state_blob_store *blobs) noexcept;
  void attach_message_bus(state_message_bus *bus) noexcept;

  state_cluster_shard *shard() const noexcept { return _shard; }
  state_peer_registry *peer_registry() const noexcept { return _peers; }
  state_blob_store *blob_store() const noexcept { return _blobs; }
  state_message_bus *message_bus() const noexcept { return _bus; }

  // Capture a coherent snapshot of all attached subsystems. Each
  // sub-snapshot is taken independently; callers should not assume
  // cross-subsystem atomicity.
  report snapshot() const;

  // Human-readable line-oriented rendering of `r`. Every line ends
  // with '\n'. Counters are printed even when zero so diffs are
  // stable. Detached subsystems are listed as "<name>: detached".
  static std::string to_text(const report &r);

  // Convenience: snapshot() + to_text().
  std::string to_text() const { return to_text(snapshot()); }

  // Erase every blob in the attached blob store whose digest is NOT
  // in `live_digests`. Returns counts and bytes freed. If no blob
  // store is attached, returns a zero result.
  //
  // The caller is responsible for assembling the live set; the admin
  // does not enumerate references on its own (a true cross-subsystem
  // walk requires the caller's tree topology). Snapshotting the
  // store and erasing happen under the store's own locks; concurrent
  // put()s of digests not in `live_digests` may be erased
  // immediately after creation. Callers that need stronger
  // guarantees should quiesce writers first.
  gc_result gc_blobs(const std::unordered_set<std::string> &live_digests);

  // Phase 8 slice 3: link-graph static analyzer.
  //
  // Walk the subtree rooted at `root` and enumerate every closed
  // cycle in the link graph. The link graph contains one vertex
  // per link node (identified by its absolute path) and one edge
  // per (source -> normalized linkTarget) when the target is also
  // a link node in the same tree. Links pointing to non-link
  // terminal nodes or to missing paths are ignored — they cannot
  // close a cycle on their own.
  //
  // A self-loop (link node whose target normalizes to its own
  // absolute path) is reported as a single-element cycle. Other
  // cycles are reported as the SCC's vertices ordered to start at
  // the lexicographically smallest path for stability.
  //
  // This is a pure read-only analysis: no nodes are created and
  // no traversal mutates the tree.
  struct link_cycles_result {
    std::vector<std::vector<std::string>> cycles;
    std::size_t link_nodes_scanned = 0;
  };
  static link_cycles_result link_cycles(state &root);

  // -------- Phase 8 slice 4c: transparent link index --------
  //
  // Walk the subtree rooted at `root` and enumerate every link
  // node whose mode is transparent. The resulting index pairs
  // (link_path, target_path) so callers can compute equivalent
  // paths under transparent aliasing. Opaque links are NOT
  // indexed: by definition they do not shadow their target's
  // contents and a subscriber under an opaque link wants the
  // link itself, not the target.
  //
  // This is a pure read-only analysis: no nodes are created and
  // no traversal mutates the tree.
  struct transparent_link {
    std::string link_path;   // absolute path of the link node
    std::string target_path; // canonical link target (empty == root)
  };

  struct transparent_link_index_result {
    std::vector<transparent_link> links;
    std::size_t link_nodes_scanned = 0;
  };

  static transparent_link_index_result transparent_link_index(state &root);

  // Given a path that just changed on the target side, return the
  // equivalent link-side paths via every transparent link in the
  // subtree rooted at `root`. For each transparent link
  // (link_path, target_path) such that `target_path` equals
  // `path` or `path` starts with target_path + ".", emits
  // `link_path` concatenated with the relative remainder.
  //
  // Chains of transparent links (a -> b -> c, both transparent)
  // are followed to a fixed point up to `hop_budget` hops, so a
  // query on path "c" returns both "b" and "a". Cycles terminate
  // naturally because already-emitted aliases are not re-expanded.
  // Self-aliases (where the link node sits at its own target
  // path) are NOT emitted to avoid trivial redundancy.
  //
  // Used by slice 4d to fan out target-side mutations to
  // link-side subscribers via the existing subscription router.
  static std::vector<std::string> transparent_link_aliases(state &root, const std::string &path,
                                                           std::size_t hop_budget = 64);

  // -------- Phase 7: force resync + stale peer GC --------
  //
  // Force re-synchronization for a specific peer. This publishes
  // a full state snapshot via the attached transport so the peer
  // can catch up from its current position. Returns the number of
  // mutations published. If no shard or transport is attached,
  // returns 0.
  struct resync_result {
    std::size_t mutations_sent = 0;
    std::size_t bytes_sent = 0;
  };
  resync_result force_resync(const std::string &peer_node_id);

  // Remove peers from the attached peer registry whose
  // last_seen_ns is older than `stale_threshold_ns` ago (measured
  // from `now_ns`). Returns the node IDs that were removed.
  std::vector<std::string> gc_stale_peers(std::uint64_t now_ns, std::uint64_t stale_threshold_ns);

private:
  state_cluster_shard *_shard = nullptr;
  state_peer_registry *_peers = nullptr;
  state_blob_store *_blobs = nullptr;
  state_message_bus *_bus = nullptr;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_DISTRIBUTED_ADMIN_H__

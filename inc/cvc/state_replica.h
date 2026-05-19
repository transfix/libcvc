/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_REPLICA_H__
#define __CVC_STATE_REPLICA_H__

#include <cvc/namespace.h>
#include <cvc/state_change_journal.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_replica
// ----------------
// Purpose:
//   Tracks per-peer replication metadata for a single shared state
//   tree: cluster membership, last applied sequence per peer, and a
//   vector clock used to order concurrent mutations and detect
//   causal relationships.
//
// Vector clock semantics:
//   The clock maps node_id -> latest observed sequence number from
//   that node. Local mutations advance the local node's component;
//   applying a remote mutation advances the remote node's component
//   if its sequence is greater than the currently recorded value.
//
//   Two clocks a and b compare as:
//     - a == b: every component matches
//     - a <  b: every component of a is <= the matching component
//                of b AND at least one strictly less
//     - a >  b: symmetric
//     - concurrent: otherwise
//
// Conflict ordering:
//   For two concurrent mutations on the same path the deterministic
//   winner is the one with the lexicographically greater
//   (origin_node_id, sequence) pair. This produces a total order
//   that all peers agree on without coordination.
//
// Threading:
//   Thread-safe. All public methods may be called concurrently.
//
class state_replica {
public:
  enum class clock_compare { equal, less_than, greater_than, concurrent };

  struct peer_info {
    std::string node_id;
    std::uint64_t last_applied_sequence = 0;
    bool alive = true;
  };

  explicit state_replica(std::string local_node_id);

  const std::string &local_node_id() const noexcept { return _local_node_id; }

  // ---- Cluster membership ----
  void add_peer(const std::string &node_id);
  bool remove_peer(const std::string &node_id);
  void mark_alive(const std::string &node_id, bool alive);
  bool has_peer(const std::string &node_id) const;
  std::vector<peer_info> peers() const;
  std::size_t peer_count() const;

  // ---- Per-peer last applied sequence ----
  // Returns the previous value.
  std::uint64_t set_last_applied(const std::string &node_id,
                                 std::uint64_t sequence);
  std::uint64_t last_applied(const std::string &node_id) const;

  // ---- Vector clock ----
  void observe_local(std::uint64_t local_sequence);
  void observe_remote(const std::string &node_id, std::uint64_t sequence);
  std::unordered_map<std::string, std::uint64_t> clock_snapshot() const;
  std::uint64_t clock_component(const std::string &node_id) const;

  static clock_compare
  compare_clocks(const std::unordered_map<std::string, std::uint64_t> &a,
                 const std::unordered_map<std::string, std::uint64_t> &b);

  // ---- Conflict resolution ----
  // Returns true if `incoming` should win over `current` for the
  // same path. The tie-breaker is (origin_node_id, sequence)
  // lexicographic on origin_node_id then numeric on sequence.
  static bool should_replace(const state_mutation &current,
                             const state_mutation &incoming);

  // ---- Loop detection ----
  // Returns true if a mutation with the given (origin_node_id,
  // sequence) has already been observed by this replica (either
  // because we originated it or because we already applied it from
  // a peer). Updates the internal seen-set when `record` is true.
  bool seen(const std::string &origin_node_id,
            std::uint64_t sequence,
            bool record);

  // Number of distinct (origin, seq) pairs currently tracked in the
  // seen-set.
  std::size_t seen_size() const;

  // Empty the seen-set. Useful for tests; production code typically
  // truncates it lazily by sequence horizon (future work).
  void clear_seen();

private:
  std::string _local_node_id;

  mutable std::mutex _mutex;
  std::unordered_map<std::string, peer_info> _peers;
  std::unordered_map<std::string, std::uint64_t> _clock;
  // seen: origin_node_id -> set of sequence numbers. Stored as a
  // sorted vector for cache locality; sequences are typically
  // monotonically increasing so insertion at the back is the
  // common case.
  std::unordered_map<std::string, std::vector<std::uint64_t>> _seen;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_REPLICA_H__

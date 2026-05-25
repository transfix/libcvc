/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TELEMETRY_AGGREGATOR_H__
#define __CVC_STATE_TELEMETRY_AGGREGATOR_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_node_telemetry.h>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

struct state_message;
class state_message_bus;

// ----------------
// cvc::routing_feedback_policy
// ----------------
// Thresholds for telemetry-driven slow-peer isolation.
//
struct routing_feedback_policy {
  // Isolate a peer whose delivery latency p99 exceeds this (ns).
  // 0 = disabled.
  std::uint64_t latency_p99_threshold_ns = 0;

  // Isolate a peer whose outbox drop count exceeds this per sample.
  // 0 = disabled.
  std::uint64_t outbox_drop_threshold = 0;

  // Minimum number of samples before a peer can be isolated
  // (prevents acting on startup noise).
  std::uint64_t min_samples = 3;
};

// ----------------
// cvc::cluster_telemetry_summary
// ----------------
// Aggregated view of the cluster derived from per-node snapshots.
//
struct cluster_telemetry_summary {
  std::uint64_t timestamp_ns = 0;
  std::size_t node_count = 0;
  std::size_t stale_count = 0; // nodes not heard from recently

  // Aggregate counters (sum across nodes).
  std::uint64_t total_mutations_published = 0;
  std::uint64_t total_mutations_applied = 0;
  std::uint64_t total_messages_admitted = 0;
  std::uint64_t total_bytes_sent = 0;
  std::uint64_t total_bytes_received = 0;
  std::uint64_t total_blobs_stored = 0;
  std::uint64_t total_blob_bytes = 0;
  std::uint64_t total_path_count = 0;
  std::uint64_t total_link_count = 0;

  // Worst-case latencies across all nodes.
  std::uint64_t max_enqueue_p99 = 0;
  std::uint64_t max_delivery_p99 = 0;

  // Aggregate rates (sum across nodes).
  double cluster_mutation_publish_rate = 0.0;
  double cluster_mutation_apply_rate = 0.0;
  double cluster_message_admit_rate = 0.0;
};

// ----------------
// cvc::state_telemetry_aggregator
// ----------------
// Phase 9: cluster-level telemetry aggregator.
//
// Consumes telemetry_snapshot messages from the OOB bus and
// maintains a latest-snapshot-per-node map. On demand, produces
// a cluster_telemetry_summary by aggregating across all known
// nodes.
//
// Multiple aggregators may run in the same cluster; results are
// eventually consistent. Stale detection: a node whose latest
// snapshot is older than 3 × publish_interval is marked stale.
//
// Typical usage:
//   state_telemetry_aggregator agg("alpha");
//   agg.attach_bus(bus); // auto-subscribes to __telemetry.*
//   // ... on demand:
//   auto summary = agg.summarize();
//   for (auto& [id, snap] : agg.peer_snapshots()) { ... }
//
class state_telemetry_aggregator {
public:
  // `stale_threshold_ns` defaults to 3 × 1 s = 3 s.
  explicit state_telemetry_aggregator(std::string cluster_id,
                                      std::uint64_t stale_threshold_ns = 3'000'000'000ULL);

  ~state_telemetry_aggregator();

  state_telemetry_aggregator(const state_telemetry_aggregator &) = delete;
  state_telemetry_aggregator &operator=(const state_telemetry_aggregator &) = delete;

  // Attach to a message bus and subscribe to __telemetry.* messages.
  // Returns true if subscription was installed successfully.
  bool attach_bus(state_message_bus &bus);

  // Detach from the bus (unsubscribe).
  void detach_bus();

  // Ingest a telemetry snapshot directly (e.g. from local node
  // or from a deserialized message). Thread-safe.
  void ingest(const telemetry_snapshot &snap);

  // Produce a cluster-wide summary from all known peer snapshots.
  cluster_telemetry_summary summarize() const;

  // Return a copy of all known peer snapshots.
  std::unordered_map<std::string, telemetry_snapshot> peer_snapshots() const;

  // Number of known peers (including stale).
  std::size_t peer_count() const;

  // Number of peers whose latest snapshot is older than the stale
  // threshold.
  std::size_t stale_count() const;

  // Mark a peer as no longer tracked (e.g. after explicit removal).
  bool remove_peer(const std::string &node_id);

  // Clear all peer snapshots.
  void clear();

  // Human-readable cluster summary text.
  std::string to_text() const;

  // Phase 9 routing feedback: evaluate peers against the policy
  // and return lists of node_ids that should be isolated or
  // released. The caller is responsible for mapping node_ids to
  // transport-level peer handles and calling mark_peer_slow /
  // clear_peer_slow.
  struct routing_feedback_result {
    std::vector<std::string> isolate; // node_ids to mark slow
    std::vector<std::string> release; // node_ids to clear slow
  };
  routing_feedback_result evaluate_routing_feedback(const routing_feedback_policy &policy) const;

  const std::string &cluster_id() const noexcept { return _cluster_id; }
  std::uint64_t stale_threshold_ns() const noexcept { return _stale_threshold_ns; }

private:
  void on_message(const state_message &msg);

  std::string _cluster_id;
  std::uint64_t _stale_threshold_ns;

  state_message_bus *_bus = nullptr;
  std::uint64_t _sub_id = 0;

  mutable std::mutex _mu;
  std::unordered_map<std::string, telemetry_snapshot> _peers;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TELEMETRY_AGGREGATOR_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_NODE_TELEMETRY_H__
#define __CVC_STATE_NODE_TELEMETRY_H__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/namespace.h>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

class app;
class state_cluster_shard;
class state_message_bus;
class state_transport;

// ----------------
// cvc::ewma
// ----------------
// Exponentially-weighted moving average with configurable
// half-life. Thread-safe via double-wide CAS on 64-bit platforms.
//
struct ewma {
  // Construct with a half-life in nanoseconds. The default 1 s
  // means that a sample taken 1 second after the previous one
  // moves the EWMA halfway toward the new value.
  explicit ewma(std::uint64_t half_life_ns = 1'000'000'000ULL) noexcept;

  // Record a new sample at `now_ns` (monotonic nanoseconds).
  void update(double sample, std::uint64_t now_ns) noexcept;

  // Current smoothed value.
  double value() const noexcept;

  // Reset to zero.
  void reset() noexcept;

  std::uint64_t half_life_ns() const noexcept { return _half_life_ns; }

private:
  std::uint64_t _half_life_ns;
  mutable std::mutex _mu;
  double _value = 0.0;
  std::uint64_t _last_ns = 0;
  bool _initialized = false;
};

// ----------------
// cvc::latency_histogram
// ----------------
// Fixed-bucket histogram for latency values in nanoseconds.
// Bucket boundaries are powers of two from 2^10 (1 µs) to
// 2^30 (1.07 s), plus an overflow bucket.
//
// Provides O(1) record and O(bucket_count) percentile queries.
//
struct latency_histogram {
  // 21 buckets: [0, 1µs), [1µs, 2µs), [2µs, 4µs), ...
  // [512ms, 1.07s), [1.07s, ∞)
  static constexpr std::size_t BUCKET_COUNT = 22;

  void record(std::uint64_t latency_ns) noexcept;
  std::uint64_t count() const noexcept;

  // Returns the bucket boundary that the given percentile (0.0–1.0)
  // falls into, or 0 if no samples have been recorded.
  std::uint64_t percentile(double p) const noexcept;

  // p50 / p90 / p99 convenience.
  std::uint64_t p50() const noexcept { return percentile(0.50); }
  std::uint64_t p90() const noexcept { return percentile(0.90); }
  std::uint64_t p99() const noexcept { return percentile(0.99); }

  void reset() noexcept;

  // Raw bucket access for serialization.
  std::uint64_t bucket(std::size_t i) const noexcept;

private:
  std::atomic<std::uint64_t> _buckets[BUCKET_COUNT] = {};
  static std::size_t bucket_index(std::uint64_t ns) noexcept;
};

// ----------------
// cvc::telemetry_snapshot
// ----------------
// POD snapshot of a single node's telemetry at a point in time.
// Cheap to copy and safe to serialize.
//
struct telemetry_snapshot {
  std::string node_id;
  std::string cluster_id;
  std::uint64_t timestamp_ns = 0; // monotonic clock

  // --- Counters (cumulative) ---
  std::uint64_t mutations_published = 0;
  std::uint64_t mutations_applied = 0;
  std::uint64_t mutations_duplicates = 0;
  std::uint64_t mutations_rejected = 0;
  std::uint64_t mutations_conflicts = 0;
  std::uint64_t messages_admitted = 0;
  std::uint64_t messages_duplicates = 0;
  std::uint64_t messages_dispatched = 0;
  std::uint64_t messages_dropped = 0;
  std::uint64_t bytes_sent = 0;
  std::uint64_t bytes_received = 0;
  std::uint64_t blobs_stored = 0;
  std::uint64_t blob_bytes = 0;
  std::uint64_t delegations_routed = 0;
  std::uint64_t delegations_expired = 0;
  std::uint64_t quarantined_mutations = 0;
  std::uint64_t quarantined_messages = 0;
  std::uint64_t auto_isolations = 0;

  // --- Rates (EWMA, per second) ---
  double mutation_publish_rate = 0.0;
  double mutation_apply_rate = 0.0;
  double message_admit_rate = 0.0;

  // --- Queue depths ---
  std::uint64_t outbox_depth = 0;
  std::uint64_t outbox_drops = 0;

  // --- Tree shape ---
  std::uint64_t path_count = 0;
  std::uint64_t link_count = 0;

  // --- Latency percentiles (ns) ---
  std::uint64_t enqueue_p50 = 0;
  std::uint64_t enqueue_p90 = 0;
  std::uint64_t enqueue_p99 = 0;
  std::uint64_t delivery_p50 = 0;
  std::uint64_t delivery_p90 = 0;
  std::uint64_t delivery_p99 = 0;

  // --- Peer count ---
  std::uint64_t peer_count = 0;
  std::uint64_t slow_peer_count = 0;
};

// ----------------
// cvc::state_node_telemetry
// ----------------
// Phase 9: per-node telemetry sampler.
//
// Collects counters, EWMAs, and latency histograms from attached
// subsystems (shard, transport, message bus, blob store) and
// produces a telemetry_snapshot on demand. Also supports periodic
// publishing of snapshots onto the OOB message bus for distribution
// to other cluster nodes.
//
// Typical usage:
//   state_node_telemetry tel(a, "node-1", "alpha");
//   tel.attach_shard(shard);
//   tel.attach_transport(transport);
//   tel.attach_message_bus(bus);
//   // ... on a timer:
//   tel.sample();
//   tel.publish_delta();
//   // ... for local inspection:
//   auto snap = tel.snapshot();
//
// Threading:
//   sample() is not thread-safe with itself (call from one thread
//   or one timer). snapshot() is safe concurrent with sample().
//   record_*_latency() are lock-free (atomic histogram buckets).
//
class state_node_telemetry {
public:
  state_node_telemetry(app &ctx, std::string node_id, std::string cluster_id);
  ~state_node_telemetry();

  state_node_telemetry(const state_node_telemetry &) = delete;
  state_node_telemetry &operator=(const state_node_telemetry &) = delete;

  // Subsystem attachment (non-owning pointers).
  void attach_shard(state_cluster_shard *s) noexcept;
  void attach_transport(state_transport *t) noexcept;
  void attach_message_bus(state_message_bus *b) noexcept;

  // Capture counters from attached subsystems and update EWMAs.
  // Call this periodically (e.g. once per second).
  void sample();

  // Return a snapshot of the latest sampled state.
  telemetry_snapshot snapshot() const;

  // Record a latency measurement. Callers in the hot path should
  // bracket their operation with steady_clock and call these.
  void record_enqueue_latency(std::uint64_t ns) noexcept;
  void record_delivery_latency(std::uint64_t ns) noexcept;

  // Serialize the latest snapshot into a state_message suitable
  // for publishing on the OOB bus. Uses JSON for debug builds,
  // a compact binary format for release.
  static constexpr const char *MIME_TELEMETRY = "application/x-cvc-telemetry+json";
  static constexpr const char *TOPIC_PREFIX = "__telemetry.";

  // Publish the latest snapshot as an OOB message on the attached
  // bus (if any). Returns true if the message was admitted.
  bool publish_snapshot();

  // Serialize / deserialize a snapshot to/from JSON.
  static std::string serialize_json(const telemetry_snapshot &snap);
  static bool deserialize_json(const std::string &json, telemetry_snapshot &out);

  const std::string &node_id() const noexcept { return _node_id; }
  const std::string &cluster_id() const noexcept { return _cluster_id; }

private:
  app &_ctx;
  std::string _node_id;
  std::string _cluster_id;

  state_cluster_shard *_shard = nullptr;
  state_transport *_transport = nullptr;
  state_message_bus *_bus = nullptr;

  // Rate EWMAs (1 s half-life).
  ewma _mutation_publish_rate;
  ewma _mutation_apply_rate;
  ewma _message_admit_rate;

  // Latency histograms.
  latency_histogram _enqueue_hist;
  latency_histogram _delivery_hist;

  // Previous counter values for rate computation.
  std::uint64_t _prev_mutations_published = 0;
  std::uint64_t _prev_mutations_applied = 0;
  std::uint64_t _prev_messages_admitted = 0;
  std::uint64_t _prev_sample_ns = 0;
  std::uint64_t _publish_seq = 0;

  mutable std::mutex _snap_mu;
  telemetry_snapshot _latest;
};

} // namespace cvc

#endif // __CVC_STATE_NODE_TELEMETRY_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DISTRIBUTED_METRICS_H__
#define __CVC_STATE_DISTRIBUTED_METRICS_H__

#include <cvc/namespace.h>

#include <cstdint>
#include <string>

namespace CVC_NAMESPACE {

class app;
class state_cluster_shard;
class state_transport;

// ----------------
// cvc::state_distributed_metrics
// ----------------
// Phase 5.
//
// Publishes per-shard and per-transport observability counters into
// the local cvc::state tree under the path
//   __system.distributed.<cluster_id>.<key>
// where <key> is the metric name. Counters are written verbatim;
// readers can subscribe to "__system.distributed" to track the
// distributed subsystem health.
//
// The metrics published per shard:
//   remote.applied            (uint64) total mutations applied
//   remote.duplicates         (uint64) total duplicate mutations dropped
//   remote.rejected           (uint64) total rejected mutations
//   conflicts.detected        (uint64) total path conflicts detected
//   conflicts.lost            (uint64) total mutations dropped on conflict
//
// Per transport (when provided):
//   transport.published       (uint64) total publish() calls
//   transport.delivered       (uint64) total successful peer deliveries
//   transport.peers           (uint64) current peer count in registry
//   transport.filtered        (uint64) total deliveries filtered by peer subs
//
// All publishes are best-effort. The implementation never throws.
//
struct state_distributed_metrics {
  // Snapshot the current shard counters and write them under
  //   __system.distributed.<cluster_id>.<key>
  // Returns the number of distinct keys written (5 on success).
  static std::size_t publish_shard(app &ctx,
                                   const state_cluster_shard &shard);

  // Snapshot the current transport counters and write them under
  //   __system.distributed.<cluster_id>.transport.<key>
  // The transport's peer registry is also queried for total peers.
  static std::size_t publish_transport_inproc(app &ctx,
                                              const std::string &cluster_id,
                                              const state_transport &t,
                                              std::uint64_t published,
                                              std::uint64_t delivered);

  // Generic helper: write a single uint64 under
  //   __system.distributed.<cluster_id>.<key>
  static void write_u64(app &ctx,
                        const std::string &cluster_id,
                        const std::string &key,
                        std::uint64_t value);
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_DISTRIBUTED_METRICS_H__

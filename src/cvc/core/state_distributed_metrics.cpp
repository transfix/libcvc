/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_distributed_metrics.h>
#include <cvc/state_transport.h>
#include <exception>

namespace cvc {

void state_distributed_metrics::write_u64(app &ctx, const std::string &cluster_id,
                                          const std::string &key, std::uint64_t value) {
  try {
    std::string path = "__system.distributed.";
    path += cluster_id.empty() ? "_unset_" : cluster_id;
    path += '.';
    path += key;
    state::instance(ctx)(path).value<std::uint64_t>(value);
  } catch (const std::exception &) {
    // Metrics are best-effort.
  } catch (...) {
  }
}

std::size_t state_distributed_metrics::publish_shard(app &ctx, const state_cluster_shard &shard) {
  const std::string &cid = shard.cluster_id();
  write_u64(ctx, cid, "remote.applied", shard.total_remote_applied());
  write_u64(ctx, cid, "remote.duplicates", shard.total_remote_duplicates());
  write_u64(ctx, cid, "remote.rejected", shard.total_remote_rejected());
  write_u64(ctx, cid, "conflicts.detected", shard.total_conflicts_detected());
  write_u64(ctx, cid, "conflicts.lost", shard.total_conflicts_lost());
  return 5;
}

std::size_t state_distributed_metrics::publish_transport_inproc(app &ctx,
                                                                const std::string &cluster_id,
                                                                const state_transport &t,
                                                                std::uint64_t published,
                                                                std::uint64_t delivered) {
  write_u64(ctx, cluster_id, "transport.published", published);
  write_u64(ctx, cluster_id, "transport.delivered", delivered);
  write_u64(ctx, cluster_id, "transport.peers", static_cast<std::uint64_t>(t.peers().size()));
  return 3;
}

std::size_t state_distributed_metrics::publish_conflicts(app &ctx,
                                                         const state_cluster_shard &shard) {
  auto entries = shard.recent_conflicts(32);
  const std::string &cid = shard.cluster_id();
  std::size_t n = 0;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    auto prefix = "conflicts.recent." + std::to_string(i);
    try {
      std::string base = "__system.distributed.";
      base += cid.empty() ? "_unset_" : cid;
      base += '.';
      base += prefix;
      state::instance(ctx)(base + ".path").value(entries[i].path);
      state::instance(ctx)(base + ".winner_node").value(entries[i].winner_node_id);
      state::instance(ctx)(base + ".winner_seq").value<std::uint64_t>(entries[i].winner_sequence);
      state::instance(ctx)(base + ".loser_node").value(entries[i].loser_node_id);
      state::instance(ctx)(base + ".loser_seq").value<std::uint64_t>(entries[i].loser_sequence);
      ++n;
    } catch (...) {
    }
  }
  return n;
}

} // namespace cvc

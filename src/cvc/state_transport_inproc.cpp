/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_cluster_shard.h>
#include <cvc/state_transport_inproc.h>

#include <algorithm>

namespace CVC_NAMESPACE {

void state_transport_inproc::register_shard(state_cluster_shard *shard) {
  if (shard == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(_mutex);
  if (std::find(_shards.begin(), _shards.end(), shard) == _shards.end()) {
    _shards.push_back(shard);
  }
}

void state_transport_inproc::unregister_shard(state_cluster_shard *shard) {
  if (shard == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(_mutex);
  _shards.erase(std::remove(_shards.begin(), _shards.end(), shard),
                _shards.end());
}

state_transport::publish_stats
state_transport_inproc::publish(const state_mutation &m) {
  publish_stats stats{};

  // Snapshot peers under the lock so ingest happens unlocked
  // (ingest_remote may take the shard's own mutex; we don't want
  // to hold the transport mutex across it).
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr) {
        continue;
      }
      if (s->cluster_id() != m.cluster_id) {
        continue;
      }
      if (s->local_node_id() == m.origin_node_id) {
        // Skip the originator: origin_node_id matches that shard's id.
        continue;
      }
      peers.push_back(s);
    }
  }

  _published.fetch_add(1, std::memory_order_relaxed);

  for (auto *peer : peers) {
    auto r = peer->ingest_remote(m);
    if (r.applied) {
      ++stats.delivered;
      _delivered.fetch_add(1, std::memory_order_relaxed);
    } else if (r.duplicate) {
      ++stats.delivered;
      ++stats.duplicates;
    } else if (r.rejected) {
      ++stats.rejected;
    }
  }

  return stats;
}

state_transport::publish_message_stats
state_transport_inproc::publish_message(const state_message &m) {
  publish_message_stats stats{};
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr)
        continue;
      if (!m.cluster_id.empty() && s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      peers.push_back(s);
    }
  }

  _msg_published.fetch_add(1, std::memory_order_relaxed);

  for (auto *peer : peers) {
    if (peer->ingest_remote_message(m)) {
      ++stats.delivered;
      _msg_delivered.fetch_add(1, std::memory_order_relaxed);
    } else {
      ++stats.duplicates;
    }
  }
  return stats;
}

std::size_t state_transport_inproc::pump_shard(state_cluster_shard &shard) {
  auto pending = shard.drain_local();
  for (const auto &m : pending) {
    publish(m);
  }
  return pending.size();
}

std::size_t state_transport_inproc::pump_all() {
  std::vector<state_cluster_shard *> snapshot;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    snapshot = _shards;
  }
  std::size_t total = 0;
  for (auto *s : snapshot) {
    if (s != nullptr) {
      total += pump_shard(*s);
    }
  }
  return total;
}

void state_transport_inproc::flush() {
  // Synchronous transport; nothing pending after publish() returns.
}

std::size_t state_transport_inproc::shard_count() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _shards.size();
}

} // namespace CVC_NAMESPACE

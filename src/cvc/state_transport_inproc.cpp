/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_transport_inproc.h>

namespace CVC_NAMESPACE {

state_transport_inproc::~state_transport_inproc() {
  // Close any installed outbox queues so blocked publishers wake.
  std::lock_guard<std::mutex> lock(_mutex);
  for (auto &kv : _outboxes) {
    if (kv.second && kv.second->queue) {
      kv.second->queue->close();
    }
  }
}

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
  _shards.erase(std::remove(_shards.begin(), _shards.end(), shard), _shards.end());
  _slow_peers.erase(shard);
  auto it = _outboxes.find(shard);
  if (it != _outboxes.end()) {
    if (it->second && it->second->queue) {
      it->second->queue->close();
    }
    _outboxes.erase(it);
  }
}

state_transport_inproc::peer_outbox *
state_transport_inproc::find_outbox_locked(state_cluster_shard *peer) {
  auto it = _outboxes.find(peer);
  if (it == _outboxes.end() || !it->second)
    return nullptr;
  return it->second.get();
}

state_transport::publish_stats state_transport_inproc::publish(const state_mutation &m) {
  publish_stats stats{};

  // Snapshot peers under the lock so ingest happens unlocked
  // (ingest_remote may take the shard's own mutex; we don't want
  // to hold the transport mutex across it).
  std::vector<state_cluster_shard *> peers;
  std::uint64_t quarantined_skipped = 0;
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
      if (_slow_peers.count(s) != 0) {
        ++quarantined_skipped;
        continue;
      }
      peers.push_back(s);
    }
  }

  if (quarantined_skipped != 0) {
    _quarantined_mutations.fetch_add(quarantined_skipped, std::memory_order_relaxed);
  }
  _published.fetch_add(1, std::memory_order_relaxed);

  for (auto *peer : peers) {
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    auto r = peer->ingest_remote(m);
    if (r.applied) {
      ++stats.delivered;
      _delivered.fetch_add(1, std::memory_order_relaxed);
      _peers.note_mutation_delivered(peer->local_node_id());
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
  // Snapshot peers and per-peer outbox decisions under the lock.
  struct peer_dest {
    state_cluster_shard *peer;
    state_bounded_queue<state_message> *queue; // null = synchronous
  };
  std::vector<peer_dest> peers;
  std::uint64_t quarantined_skipped = 0;
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
      if (_slow_peers.count(s) != 0) {
        ++quarantined_skipped;
        continue;
      }
      auto *ob = find_outbox_locked(s);
      peers.push_back({s, ob ? ob->queue.get() : nullptr});
    }
  }

  if (quarantined_skipped != 0) {
    _quarantined_messages.fetch_add(quarantined_skipped, std::memory_order_relaxed);
  }
  _msg_published.fetch_add(1, std::memory_order_relaxed);

  for (auto &dest : peers) {
    auto *peer = dest.peer;
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    if (dest.queue != nullptr) {
      // Outbox path: stage; do not invoke ingest_remote_message
      // here. Drop_oldest evicts silently; drop_newest rejects.
      const std::uint64_t before_dn = dest.queue->total_dropped_newest();
      const std::uint64_t before_do = dest.queue->total_dropped_oldest();
      bool admitted = dest.queue->push(m);
      const std::uint64_t after_dn = dest.queue->total_dropped_newest();
      const std::uint64_t after_do = dest.queue->total_dropped_oldest();
      if (after_dn > before_dn) {
        _outbox_dropped_newest.fetch_add(after_dn - before_dn, std::memory_order_relaxed);
      }
      if (after_do > before_do) {
        _outbox_dropped_oldest.fetch_add(after_do - before_do, std::memory_order_relaxed);
      }
      if (admitted) {
        _outbox_admitted.fetch_add(1, std::memory_order_relaxed);
        ++stats.delivered; // queued counts as in-flight delivered
      } else {
        ++stats.duplicates; // overflow drop, surface as non-delivered
      }
      // Auto-isolation: if the cumulative drops have reached the
      // configured threshold, mark this peer slow so subsequent
      // publishes skip it. We do this here (after admit/drop) so
      // that the threshold reflects observed pressure, not just
      // configured capacity.
      const std::uint64_t threshold = _auto_isolation_threshold.load(std::memory_order_relaxed);
      if (threshold != 0 && (after_dn + after_do) >= threshold) {
        bool newly_slow = false;
        {
          std::lock_guard<std::mutex> lock(_mutex);
          newly_slow = _slow_peers.insert(peer).second;
        }
        if (newly_slow) {
          _auto_isolations.fetch_add(1, std::memory_order_relaxed);
        }
      }
      continue;
    }
    if (peer->ingest_remote_message(m)) {
      ++stats.delivered;
      _msg_delivered.fetch_add(1, std::memory_order_relaxed);
      _peers.note_message_delivered(peer->local_node_id());
    } else {
      ++stats.duplicates;
    }
  }
  return stats;
}

void state_transport_inproc::set_peer_message_outbox(state_cluster_shard *peer,
                                                     std::size_t capacity, outbox_policy policy) {
  if (peer == nullptr)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  if (capacity == 0) {
    auto it = _outboxes.find(peer);
    if (it != _outboxes.end()) {
      if (it->second && it->second->queue)
        it->second->queue->close();
      _outboxes.erase(it);
    }
    return;
  }
  auto ob = std::make_unique<peer_outbox>();
  ob->policy = policy;
  ob->queue = std::make_unique<state_bounded_queue<state_message>>(capacity, policy);
  // Replacing an existing outbox: close the old one to release any
  // blocked producers.
  auto &slot = _outboxes[peer];
  if (slot && slot->queue)
    slot->queue->close();
  slot = std::move(ob);
}

void state_transport_inproc::clear_peer_message_outbox(state_cluster_shard *peer) {
  if (peer == nullptr)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  auto it = _outboxes.find(peer);
  if (it == _outboxes.end())
    return;
  if (it->second && it->second->queue)
    it->second->queue->close();
  _outboxes.erase(it);
}

std::size_t state_transport_inproc::peer_message_outbox_size(state_cluster_shard *peer) const {
  if (peer == nullptr)
    return 0;
  std::lock_guard<std::mutex> lock(_mutex);
  auto it = _outboxes.find(peer);
  if (it == _outboxes.end() || !it->second || !it->second->queue)
    return 0;
  return it->second->queue->size();
}

std::size_t state_transport_inproc::deliver_message_outbox(state_cluster_shard *peer,
                                                           std::size_t max) {
  if (peer == nullptr)
    return 0;
  // Pull a snapshot of the queue pointer under the lock; the queue
  // itself is internally synchronized.
  state_bounded_queue<state_message> *q = nullptr;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _outboxes.find(peer);
    if (it == _outboxes.end() || !it->second || !it->second->queue)
      return 0;
    q = it->second->queue.get();
  }
  std::size_t delivered = 0;
  std::size_t budget = (max == 0) ? std::size_t(-1) : max;
  state_message m;
  while (budget > 0 && q->try_pop(m)) {
    --budget;
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    if (peer->ingest_remote_message(m)) {
      ++delivered;
      _msg_delivered.fetch_add(1, std::memory_order_relaxed);
      _peers.note_message_delivered(peer->local_node_id());
    }
  }
  return delivered;
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

void state_transport_inproc::mark_peer_slow(state_cluster_shard *peer) {
  if (peer == nullptr)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _slow_peers.insert(peer);
}

void state_transport_inproc::clear_peer_slow(state_cluster_shard *peer) {
  if (peer == nullptr)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _slow_peers.erase(peer);
}

bool state_transport_inproc::is_peer_slow(state_cluster_shard *peer) const {
  if (peer == nullptr)
    return false;
  std::lock_guard<std::mutex> lock(_mutex);
  return _slow_peers.count(peer) != 0;
}

std::vector<state_cluster_shard *> state_transport_inproc::slow_peers() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return std::vector<state_cluster_shard *>(_slow_peers.begin(), _slow_peers.end());
}

void state_transport_inproc::set_auto_isolation_drop_threshold(std::uint64_t threshold) noexcept {
  _auto_isolation_threshold.store(threshold, std::memory_order_relaxed);
}

// ---------- chunk fetch ----------

bool state_transport_inproc::fetch_chunk(const std::string &digest, chunk_callback on_chunk) {
  if (!_blob_store || digest.empty())
    return false;
  std::vector<unsigned char> bytes;
  if (!_blob_store->get(digest, bytes))
    return false;
  if (on_chunk)
    on_chunk(digest, bytes);
  return true;
}

std::size_t state_transport_inproc::fetch_chunks(const std::vector<std::string> &digests,
                                                 chunk_callback on_chunk) {
  if (!_blob_store)
    return 0;
  std::size_t n = 0;
  for (const auto &d : digests) {
    std::vector<unsigned char> bytes;
    if (_blob_store->get(d, bytes)) {
      if (on_chunk)
        on_chunk(d, bytes);
      ++n;
    }
  }
  return n;
}

} // namespace CVC_NAMESPACE

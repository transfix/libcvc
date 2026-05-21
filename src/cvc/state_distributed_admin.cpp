/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_distributed_admin.h>

#include <cvc/state_authority_map.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_peer_registry.h>

#include <sstream>
#include <vector>

namespace CVC_NAMESPACE {

void state_distributed_admin::attach_shard(
    state_cluster_shard *shard) noexcept {
  _shard = shard;
}

void state_distributed_admin::attach_peer_registry(
    state_peer_registry *peers) noexcept {
  _peers = peers;
}

void state_distributed_admin::attach_blob_store(
    state_blob_store *blobs) noexcept {
  _blobs = blobs;
}

void state_distributed_admin::attach_message_bus(
    state_message_bus *bus) noexcept {
  _bus = bus;
}

state_distributed_admin::report state_distributed_admin::snapshot() const {
  report r;

  if (_shard) {
    r.shard.attached = true;
    r.shard.cluster_id = _shard->cluster_id();
    r.shard.node_id = _shard->local_node_id();
    r.shard.enforce_authority = _shard->enforce_authority();
    r.shard.enforce_write_policy = _shard->enforce_write_policy();
    r.shard.enforce_delegation = _shard->enforce_delegation();
    r.shard.resolve_conflicts = _shard->resolve_conflicts();
    r.shard.total_remote_applied = _shard->total_remote_applied();
    r.shard.total_remote_duplicates = _shard->total_remote_duplicates();
    r.shard.total_remote_rejected = _shard->total_remote_rejected();
    r.shard.total_conflicts_detected = _shard->total_conflicts_detected();
    r.shard.total_conflicts_lost = _shard->total_conflicts_lost();
    r.shard.total_delegation_routed = _shard->total_delegation_routed();
    r.shard.total_delegation_expired = _shard->total_delegation_expired();

    auto snap = _shard->delegation().authority().snapshot();
    r.delegations.reserve(snap.size());
    for (auto &kv : snap) {
      delegation_entry e;
      e.prefix = kv.first;
      e.cluster_id = kv.second.cluster_id;
      e.endpoint = kv.second.endpoint;
      e.expires_at_ns = kv.second.expires_at_ns;
      r.delegations.push_back(std::move(e));
    }
  }

  if (_peers) {
    auto plist = _peers->snapshot();
    r.peers.reserve(plist.size());
    for (auto &p : plist) {
      peer_entry e;
      e.node_id = p.node_id;
      e.cluster_id = p.cluster_id;
      e.endpoint = p.endpoint;
      e.subscriptions = p.subscriptions;
      e.last_seen_ns = p.last_seen_ns;
      e.mutations_delivered = p.mutations_delivered;
      e.messages_delivered = p.messages_delivered;
      e.deliveries_filtered = p.deliveries_filtered;
      r.peers.push_back(std::move(e));
    }
  }

  if (_bus) {
    r.bus.attached = true;
    r.bus.total_admitted = _bus->total_admitted();
    r.bus.total_duplicates = _bus->total_duplicates();
    r.bus.total_dispatched = _bus->total_dispatched();
    r.bus.total_dropped = _bus->total_dropped();
  }

  if (_blobs) {
    r.blobs.attached = true;
    r.blobs.count = _blobs->size();
    r.blobs.bytes_stored = _blobs->bytes_stored();
  }

  return r;
}

std::string state_distributed_admin::to_text(const report &r) {
  std::ostringstream os;
  os << "[shard]\n";
  if (!r.shard.attached) {
    os << "  detached\n";
  } else {
    os << "  cluster_id=" << r.shard.cluster_id << "\n"
       << "  node_id=" << r.shard.node_id << "\n"
       << "  enforce_authority=" << (r.shard.enforce_authority ? 1 : 0)
       << "\n"
       << "  enforce_write_policy=" << (r.shard.enforce_write_policy ? 1 : 0)
       << "\n"
       << "  enforce_delegation=" << (r.shard.enforce_delegation ? 1 : 0)
       << "\n"
       << "  resolve_conflicts=" << (r.shard.resolve_conflicts ? 1 : 0)
       << "\n"
       << "  remote_applied=" << r.shard.total_remote_applied << "\n"
       << "  remote_duplicates=" << r.shard.total_remote_duplicates << "\n"
       << "  remote_rejected=" << r.shard.total_remote_rejected << "\n"
       << "  conflicts_detected=" << r.shard.total_conflicts_detected << "\n"
       << "  conflicts_lost=" << r.shard.total_conflicts_lost << "\n"
       << "  delegation_routed=" << r.shard.total_delegation_routed << "\n"
       << "  delegation_expired=" << r.shard.total_delegation_expired
       << "\n";
  }

  os << "[delegations] count=" << r.delegations.size() << "\n";
  for (auto &d : r.delegations) {
    os << "  prefix='" << d.prefix << "' cluster=" << d.cluster_id
       << " endpoint=" << d.endpoint << " expires_at_ns=" << d.expires_at_ns
       << "\n";
  }

  os << "[peers] count=" << r.peers.size() << "\n";
  for (auto &p : r.peers) {
    os << "  node=" << p.node_id << " cluster=" << p.cluster_id
       << " endpoint=" << p.endpoint << " subs=" << p.subscriptions.size()
       << " last_seen_ns=" << p.last_seen_ns
       << " mut_delivered=" << p.mutations_delivered
       << " msg_delivered=" << p.messages_delivered
       << " filtered=" << p.deliveries_filtered << "\n";
  }

  os << "[bus]\n";
  if (!r.bus.attached) {
    os << "  detached\n";
  } else {
    os << "  admitted=" << r.bus.total_admitted << "\n"
       << "  duplicates=" << r.bus.total_duplicates << "\n"
       << "  dispatched=" << r.bus.total_dispatched << "\n"
       << "  dropped=" << r.bus.total_dropped << "\n";
  }

  os << "[blobs]\n";
  if (!r.blobs.attached) {
    os << "  detached\n";
  } else {
    os << "  count=" << r.blobs.count << "\n"
       << "  bytes_stored=" << r.blobs.bytes_stored << "\n";
  }

  return os.str();
}

state_distributed_admin::gc_result state_distributed_admin::gc_blobs(
    const std::unordered_set<std::string> &live_digests) {
  gc_result g;
  if (!_blobs)
    return g;

  std::vector<std::string> all = _blobs->digests();
  g.scanned = all.size();
  for (const auto &d : all) {
    if (live_digests.find(d) != live_digests.end())
      continue;
    // Read size before erase so we can report bytes_freed accurately.
    std::vector<unsigned char> bytes;
    std::uint64_t sz = 0;
    if (_blobs->get(d, bytes))
      sz = static_cast<std::uint64_t>(bytes.size());
    if (_blobs->erase(d)) {
      ++g.removed;
      g.bytes_freed += sz;
    }
  }
  return g;
}

} // namespace CVC_NAMESPACE

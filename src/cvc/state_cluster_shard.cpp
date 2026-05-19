/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_cluster_shard.h>

#include <cvc/app.h>

#include <utility>

namespace CVC_NAMESPACE {

state_cluster_shard::state_cluster_shard(app &ctx, std::string cluster_id,
                                         std::string local_node_id,
                                         std::string root_path)
    : _cluster_id(std::move(cluster_id)),
      _local_node_id(std::move(local_node_id)),
      _root_path(std::move(root_path)) {
  _adapter = std::make_unique<state_sync_adapter>(ctx, _root_path,
                                                  _local_node_id);
  _replica = std::make_unique<state_replica>(_local_node_id);
  _authority = std::make_unique<state_authority_map>();
  _codecs = std::make_unique<state_codec_registry>();
  _codecs->register_builtin_codecs();
}

state_cluster_shard::~state_cluster_shard() {
  if (_adapter && _adapter->is_attached())
    _adapter->detach();
}

void state_cluster_shard::attach() { _adapter->attach(); }

void state_cluster_shard::detach() { _adapter->detach(); }

bool state_cluster_shard::is_attached() const noexcept {
  return _adapter->is_attached();
}

void state_cluster_shard::set_enforce_authority(bool enforce) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _enforce_authority = enforce;
}

bool state_cluster_shard::enforce_authority() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _enforce_authority;
}

state_cluster_shard::ingest_result
state_cluster_shard::ingest_remote(const state_mutation &m) {
  ingest_result r;

  // Loop detection: have we already applied this exact (origin,seq)?
  if (_replica->seen(m.origin_node_id, m.sequence, /*record*/ false)) {
    r.duplicate = true;
    return r;
  }

  // Optional authority enforcement: a path that resolves to an
  // authority entry whose cluster_id differs from this shard's
  // cluster_id is rejected.
  bool enforce;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    enforce = _enforce_authority;
  }
  if (enforce) {
    auto auth = _authority->resolve(m.path);
    if (auth.valid && !auth.cluster_id.empty() &&
        auth.cluster_id != _cluster_id) {
      r.rejected = true;
      r.reject_reason = "path '" + m.path + "' is owned by cluster '" +
                        auth.cluster_id + "', not '" + _cluster_id + "'";
      return r;
    }
  }

  // Record before applying so a re-entrant signal cannot double-apply.
  (void)_replica->seen(m.origin_node_id, m.sequence, /*record*/ true);
  _replica->observe_remote(m.origin_node_id, m.sequence);

  // Apply. apply_remote marks the calling thread as remote so the
  // resulting state signals do not loop back into the journal.
  bool ok = _adapter->apply_remote(m);
  r.applied = ok;
  if (ok)
    _replica->set_last_applied(m.origin_node_id, m.sequence);
  return r;
}

std::vector<state_mutation>
state_cluster_shard::drain_local(std::size_t max_count) {
  std::uint64_t cursor;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    cursor = _publish_cursor;
  }

  std::vector<state_mutation> pending = _adapter->journal().replay_after(cursor);
  std::vector<state_mutation> out;
  out.reserve(pending.size());
  for (auto &m : pending) {
    if (m.origin_node_id != _local_node_id)
      continue;
    out.push_back(std::move(m));
    if (max_count != 0 && out.size() >= max_count)
      break;
  }

  if (!out.empty()) {
    std::lock_guard<std::mutex> lk(_mutex);
    if (out.back().sequence > _publish_cursor)
      _publish_cursor = out.back().sequence;
    _replica->observe_local(_publish_cursor);
  }
  return out;
}

std::uint64_t state_cluster_shard::published_cursor() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _publish_cursor;
}

void state_cluster_shard::rewind_publish_cursor(std::uint64_t sequence) {
  std::lock_guard<std::mutex> lk(_mutex);
  _publish_cursor = sequence;
}

} // namespace CVC_NAMESPACE

/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_cluster_shard.h>

#include <cvc/app.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <utility>

namespace CVC_NAMESPACE {

namespace {

// Wire format for op=delegate_subtree / revoke_delegation:
//   path         = path_prefix
//   string_value = target cluster_id    (empty for revoke)
//   type_name    = endpoint hint        (empty when not used)
//   payload      = inline 8-byte LE u64 lease_duration_ns
//                  (absent or zero-length = infinite lease;
//                   irrelevant for revoke)
//
// The receiver applies lease_duration_ns relative to its own
// clock so per-node skew does not corrupt the lease horizon.

std::vector<unsigned char> encode_lease_duration(std::uint64_t ns) {
  std::vector<unsigned char> out(sizeof(std::uint64_t));
  for (std::size_t i = 0; i < sizeof(std::uint64_t); ++i) {
    out[i] = static_cast<unsigned char>((ns >> (i * 8)) & 0xff);
  }
  return out;
}

std::uint64_t decode_lease_duration(const std::vector<unsigned char> &bytes) {
  if (bytes.size() < sizeof(std::uint64_t))
    return 0;
  std::uint64_t ns = 0;
  for (std::size_t i = 0; i < sizeof(std::uint64_t); ++i) {
    ns |= static_cast<std::uint64_t>(bytes[i]) << (i * 8);
  }
  return ns;
}

bool is_delegation_op(state_mutation_op op) {
  return op == state_mutation_op::delegate_subtree ||
         op == state_mutation_op::revoke_delegation;
}

} // namespace

state_cluster_shard::state_cluster_shard(app &ctx, std::string cluster_id,
                                         std::string local_node_id,
                                         std::string root_path)
    : _cluster_id(std::move(cluster_id)),
      _local_node_id(std::move(local_node_id)),
      _root_path(std::move(root_path)) {
  _adapter = std::make_unique<state_sync_adapter>(ctx, _root_path,
                                                  _local_node_id);
  _replica = std::make_unique<state_replica>(_local_node_id);
  _delegation = std::make_unique<state_delegation_manager>(_cluster_id);
  _codecs = std::make_unique<state_codec_registry>();
  _codecs->register_builtin_codecs();
  _message_bus = std::make_unique<state_message_bus>();
  _write_policy = std::make_unique<state_write_policy>();
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

void state_cluster_shard::set_enforce_write_policy(bool enforce) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _enforce_write_policy = enforce;
}

bool state_cluster_shard::enforce_write_policy() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _enforce_write_policy;
}

void state_cluster_shard::set_resolve_conflicts(bool resolve) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _resolve_conflicts = resolve;
}

bool state_cluster_shard::resolve_conflicts() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _resolve_conflicts;
}

void state_cluster_shard::set_enforce_delegation(bool enforce) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _enforce_delegation = enforce;
}

bool state_cluster_shard::enforce_delegation() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _enforce_delegation;
}

bool state_cluster_shard::ingest_remote_message(const state_message &m) {
  if (!m.cluster_id.empty() && m.cluster_id != _cluster_id)
    return false;
  return _message_bus->admit(m);
}

state_cluster_shard::ingest_result
state_cluster_shard::ingest_remote(const state_mutation &m) {
  ingest_result r;

  // Loop detection: have we already applied this exact (origin,seq)?
  if (_replica->seen(m.origin_node_id, m.sequence, /*record*/ false)) {
    r.duplicate = true;
    _ctr_remote_duplicates.fetch_add(1, std::memory_order_relaxed);
    return r;
  }

  // Phase 6 control-plane fast path. Delegation mutations are
  // metadata about the delegation system itself; they bypass the
  // authority/delegation/write-policy/conflict gates that govern
  // ordinary value writes. Loop detection still applies.
  if (is_delegation_op(m.op)) {
    (void)_replica->seen(m.origin_node_id, m.sequence, /*record*/ true);
    _replica->observe_remote(m.origin_node_id, m.sequence);
    if (m.op == state_mutation_op::delegate_subtree) {
      _delegation->delegate(m.path, m.string_value, m.type_name,
                            decode_lease_duration(m.payload.inline_bytes));
      _ctr_delegations_applied.fetch_add(1, std::memory_order_relaxed);
    } else {
      (void)_delegation->revoke(m.path);
      _ctr_revocations_applied.fetch_add(1, std::memory_order_relaxed);
    }
    _replica->set_last_applied(m.origin_node_id, m.sequence);
    _ctr_remote_applied.fetch_add(1, std::memory_order_relaxed);
    r.applied = true;
    return r;
  }

  // Optional authority enforcement: a path that resolves to an
  // authority entry whose cluster_id differs from this shard's
  // cluster_id is rejected.
  bool enforce_auth;
  bool enforce_policy;
  bool enforce_deleg;
  bool resolve_conf;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    enforce_auth = _enforce_authority;
    enforce_policy = _enforce_write_policy;
    enforce_deleg = _enforce_delegation;
    resolve_conf = _resolve_conflicts;
  }
  if (enforce_auth) {
    auto auth = _delegation->authority().resolve(m.path);
    if (auth.valid && !auth.cluster_id.empty() &&
        auth.cluster_id != _cluster_id) {
      r.rejected = true;
      r.reject_reason = "path '" + m.path + "' is owned by cluster '" +
                        auth.cluster_id + "', not '" + _cluster_id + "'";
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
  }
  if (enforce_deleg) {
    auto rd = _delegation->route(m.path);
    if (rd.kind == state_delegation_manager::route_kind::remote) {
      r.rejected = true;
      r.reject_reason = "path '" + m.path + "' delegated to cluster '" +
                        rd.cluster_id + "' (prefix '" + rd.matched_prefix +
                        "')";
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      _ctr_delegation_routed.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
    if (rd.kind == state_delegation_manager::route_kind::expired) {
      r.rejected = true;
      r.reject_reason = "delegation lease expired for prefix '" +
                        rd.matched_prefix + "' (cluster '" + rd.cluster_id +
                        "')";
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      _ctr_delegation_expired.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
  }
  if (enforce_policy) {
    auto d = _write_policy->authorize(m.path, m.origin_node_id);
    if (!d.allowed) {
      r.rejected = true;
      r.reject_reason = d.reject_reason;
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
  }

  // Conflict resolution: when enabled, compare against the last
  // mutation applied at this path. If incoming loses the
  // deterministic tie-breaker we still record it as seen so we
  // never reprocess it, but skip the apply.
  bool conflict_lost = false;
  if (resolve_conf) {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _last_path_mutation.find(m.path);
    if (it != _last_path_mutation.end() &&
        it->second.origin_node_id != m.origin_node_id) {
      _ctr_conflicts_detected.fetch_add(1, std::memory_order_relaxed);
      if (!state_replica::should_replace(it->second, m)) {
        conflict_lost = true;
      }
    }
  }

  // Record before applying so a re-entrant signal cannot double-apply.
  (void)_replica->seen(m.origin_node_id, m.sequence, /*record*/ true);
  _replica->observe_remote(m.origin_node_id, m.sequence);

  if (conflict_lost) {
    _ctr_conflicts_lost.fetch_add(1, std::memory_order_relaxed);
    // Mark as seen but do not apply.
    return r;
  }

  // Apply. apply_remote marks the calling thread as remote so the
  // resulting state signals do not loop back into the journal.
  bool ok = _adapter->apply_remote(m);
  r.applied = ok;
  if (ok) {
    _replica->set_last_applied(m.origin_node_id, m.sequence);
    _ctr_remote_applied.fetch_add(1, std::memory_order_relaxed);
    if (resolve_conf) {
      std::lock_guard<std::mutex> lk(_mutex);
      _last_path_mutation[m.path] = m;
    }
  }
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
    // Stamp the shard's cluster identity. The adapter/journal does
    // not know about clusters; that's a shard-level fact carried on
    // the wire so peers can route by cluster_id.
    if (m.cluster_id.empty())
      m.cluster_id = _cluster_id;
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

void state_cluster_shard::publish_delegation(
    const std::string &path_prefix, const std::string &cluster_id,
    const std::string &endpoint, std::uint64_t lease_duration_ns) {
  // Apply locally first so the local view is consistent before any
  // peer sees it.
  _delegation->delegate(path_prefix, cluster_id, endpoint, lease_duration_ns);
  _ctr_delegations_applied.fetch_add(1, std::memory_order_relaxed);

  // Journal the control-plane mutation so drain_local picks it up
  // for the transport. Stamp origin/cluster so the journal does
  // not default-fill them with the adapter's local-node label.
  state_mutation m;
  m.cluster_id = _cluster_id;
  m.origin_node_id = _local_node_id;
  m.path = path_prefix;
  m.op = state_mutation_op::delegate_subtree;
  m.string_value = cluster_id;
  m.type_name = endpoint;
  m.payload =
      state_payload::inline_data(encode_lease_duration(lease_duration_ns));
  auto stored = _adapter->journal().append(m);
  // Mark as seen on this shard so a round-trip through the
  // transport cannot re-apply our own delegation.
  (void)_replica->seen(stored.origin_node_id, stored.sequence,
                       /*record*/ true);
  _replica->observe_local(stored.sequence);
}

void state_cluster_shard::publish_revocation(const std::string &path_prefix) {
  (void)_delegation->revoke(path_prefix);
  _ctr_revocations_applied.fetch_add(1, std::memory_order_relaxed);

  state_mutation m;
  m.cluster_id = _cluster_id;
  m.origin_node_id = _local_node_id;
  m.path = path_prefix;
  m.op = state_mutation_op::revoke_delegation;
  auto stored = _adapter->journal().append(m);
  (void)_replica->seen(stored.origin_node_id, stored.sequence,
                       /*record*/ true);
  _replica->observe_local(stored.sequence);
}

} // namespace CVC_NAMESPACE

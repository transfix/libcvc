/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_transport.h>
#include <cvc/core/state_volume_codec.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace cvc {

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
  return op == state_mutation_op::delegate_subtree || op == state_mutation_op::revoke_delegation;
}

// Per-app default-shard registry for Phase 8 slice 2. Keyed by
// app pointer; value is the shard most recently installed as the
// default for that app. Cleared on uninstall_as_default().
std::mutex &default_registry_mutex() {
  static std::mutex m;
  return m;
}

std::unordered_map<const app *, state_cluster_shard *> &default_registry() {
  static std::unordered_map<const app *, state_cluster_shard *> r;
  return r;
}

} // namespace

state_cluster_shard::state_cluster_shard(app &ctx, std::string cluster_id,
                                         std::string local_node_id, std::string root_path)
    : _cluster_id(std::move(cluster_id)), _local_node_id(std::move(local_node_id)),
      _root_path(std::move(root_path)), _app_ctx(&ctx) {
  if (!state::isValidStateName(_cluster_id))
    throw std::invalid_argument("cluster_id '" + _cluster_id + "' violates C identifier rules");
  if (!state::isValidStateName(_local_node_id))
    throw std::invalid_argument("local_node_id '" + _local_node_id +
                                "' violates C identifier rules");
  _adapter = std::make_unique<state_sync_adapter>(ctx, _root_path, _local_node_id);
  _replica = std::make_unique<state_replica>(_local_node_id);
  _delegation = std::make_unique<state_delegation_manager>(_cluster_id);
  _codecs = std::make_unique<state_codec_registry>();
  _codecs->register_builtin_codecs();
  register_volume_geometry_codecs(*_codecs);
  _message_bus = std::make_unique<state_message_bus>();
  _write_policy = std::make_unique<state_write_policy>();
}

state_cluster_shard::~state_cluster_shard() {
  uninstall_as_default();
  if (_adapter && _adapter->is_attached())
    _adapter->detach();
}

void state_cluster_shard::attach() {
  _adapter->attach();
  install_as_default();
}

void state_cluster_shard::detach() {
  uninstall_as_default();
  _adapter->detach();
}

bool state_cluster_shard::is_attached() const noexcept { return _adapter->is_attached(); }

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

std::vector<state_cluster_shard::conflict_entry>
state_cluster_shard::recent_conflicts(std::size_t max_entries) const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::size_t n = std::min(max_entries, _conflict_ring.size());
  std::vector<conflict_entry> out;
  out.reserve(n);
  // Walk backwards from the write cursor to get most-recent-first.
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t idx = (_conflict_ring_pos + _conflict_ring.size() - 1 - i) % _conflict_ring.size();
    out.push_back(_conflict_ring[idx]);
  }
  return out;
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
  if (!path_is_of_interest(m.path)) {
    _ctr_remote_filtered_out.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return _message_bus->admit(m);
}

state_cluster_shard::ingest_result state_cluster_shard::ingest_remote(const state_mutation &m) {
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

  // Inbound interest filter: when enabled, drop any mutation whose
  // path is not covered by a registered interest prefix. The
  // seen-set is intentionally NOT advanced so that adding the
  // interest later allows a re-publish to land.
  if (!path_is_of_interest(m.path)) {
    r.rejected = true;
    r.reject_reason = "path '" + m.path + "' outside local interest set";
    _ctr_remote_filtered_out.fetch_add(1, std::memory_order_relaxed);
    return r;
  }

  // Hash-partition enforcement: when enabled, reject mutations
  // whose path hashes to a node other than this one. This
  // prevents a shard from accumulating data it should not own.
  bool enforce_part;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    enforce_part = _enforce_partition;
  }
  if (enforce_part && _partition.size() > 0) {
    std::string owner = _partition.owner_of(m.path);
    if (!owner.empty() && owner != _local_node_id) {
      r.rejected = true;
      r.reject_reason =
          "path '" + m.path + "' partitioned to node '" + owner + "', not '" + _local_node_id + "'";
      _ctr_partition_rejected.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
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
    if (auth.valid && !auth.cluster_id.empty() && auth.cluster_id != _cluster_id) {
      r.rejected = true;
      r.reject_reason = "path '" + m.path + "' is owned by cluster '" + auth.cluster_id +
                        "', not '" + _cluster_id + "'";
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
  }
  if (enforce_deleg) {
    auto rd = _delegation->route(m.path);
    if (rd.kind == state_delegation_manager::route_kind::remote) {
      r.rejected = true;
      r.reject_reason = "path '" + m.path + "' delegated to cluster '" + rd.cluster_id +
                        "' (prefix '" + rd.matched_prefix + "')";
      _ctr_remote_rejected.fetch_add(1, std::memory_order_relaxed);
      _ctr_delegation_routed.fetch_add(1, std::memory_order_relaxed);
      return r;
    }
    if (rd.kind == state_delegation_manager::route_kind::expired) {
      r.rejected = true;
      r.reject_reason = "delegation lease expired for prefix '" + rd.matched_prefix +
                        "' (cluster '" + rd.cluster_id + "')";
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
    if (it != _last_path_mutation.end() && it->second.origin_node_id != m.origin_node_id) {
      _ctr_conflicts_detected.fetch_add(1, std::memory_order_relaxed);
      if (!state_replica::should_replace(it->second, m)) {
        conflict_lost = true;
        // Record conflict detail in ring buffer.
        if (_conflict_ring.size() < kMaxConflictRing)
          _conflict_ring.push_back({});
        auto &e = _conflict_ring[_conflict_ring_pos % kMaxConflictRing];
        e.path = m.path;
        e.winner_node_id = it->second.origin_node_id;
        e.winner_sequence = it->second.sequence;
        e.loser_node_id = m.origin_node_id;
        e.loser_sequence = m.sequence;
        _conflict_ring_pos = (_conflict_ring_pos + 1) % kMaxConflictRing;
      }
    }
  }

  // Record before applying so a re-entrant signal cannot double-apply.
  (void)_replica->seen(m.origin_node_id, m.sequence, /*record*/ true);
  _replica->observe_remote(m.origin_node_id, m.sequence);

  // Merge the remote HLC timestamp into our local clock so our
  // next outbound mutation will be causally after this one.
  if (m.hlc_time != 0)
    _clock.update(hybrid_time::from_packed(m.hlc_time));

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

std::vector<state_mutation> state_cluster_shard::drain_local(std::size_t max_count) {
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
    // Stamp HLC time on outbound mutations so receivers can merge
    // causal ordering via their own hybrid_clock::update().
    if (m.hlc_time == 0)
      m.hlc_time = _clock.now().packed();
    // Offload large values to the blob store when a threshold is set.
    if (_blob_store && _max_inline_payload_bytes > 0 && m.op == state_mutation_op::set_value &&
        m.payload.kind == state_payload_kind::none &&
        m.string_value.size() > _max_inline_payload_bytes) {
      std::vector<unsigned char> bytes(m.string_value.begin(), m.string_value.end());
      state_blob_ref ref = _blob_store->put(bytes, m.type_name);
      m.string_value.clear();
      m.payload = state_payload::blob_ref(ref);
    }
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

void state_cluster_shard::publish_delegation(const std::string &path_prefix,
                                             const std::string &cluster_id,
                                             const std::string &endpoint,
                                             std::uint64_t lease_duration_ns) {
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
  m.payload = state_payload::inline_data(encode_lease_duration(lease_duration_ns));
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

// ---- Inbound interest filter ----

namespace {

// Normalize a path-prefix: trim leading/trailing dots. Empty stays empty.
std::string normalize_interest_prefix(std::string p) {
  while (!p.empty() && p.front() == '.')
    p.erase(p.begin());
  while (!p.empty() && p.back() == '.')
    p.pop_back();
  return p;
}

// Dot-segment-aware prefix match. Empty prefix matches every path.
bool path_matches_prefix(const std::string &path, const std::string &pref) {
  if (pref.empty())
    return true;
  if (path.size() < pref.size())
    return false;
  if (path.compare(0, pref.size(), pref) != 0)
    return false;
  return path.size() == pref.size() || path[pref.size()] == '.';
}

} // namespace

void state_cluster_shard::add_interest(std::string path_prefix) {
  std::string p = normalize_interest_prefix(std::move(path_prefix));
  std::lock_guard<std::mutex> lk(_mutex);
  for (const auto &existing : _interests) {
    if (existing == p)
      return;
  }
  _interests.push_back(std::move(p));
}

bool state_cluster_shard::remove_interest(const std::string &path_prefix) {
  std::string p = normalize_interest_prefix(path_prefix);
  std::lock_guard<std::mutex> lk(_mutex);
  for (auto it = _interests.begin(); it != _interests.end(); ++it) {
    if (*it == p) {
      _interests.erase(it);
      return true;
    }
  }
  return false;
}

void state_cluster_shard::clear_interests() {
  std::lock_guard<std::mutex> lk(_mutex);
  _interests.clear();
}

std::vector<std::string> state_cluster_shard::interests() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _interests;
}

bool state_cluster_shard::path_is_of_interest(const std::string &path) const {
  std::lock_guard<std::mutex> lk(_mutex);
  if (!_enforce_interest)
    return true;
  for (const auto &pref : _interests) {
    if (path_matches_prefix(path, pref))
      return true;
  }
  return false;
}

void state_cluster_shard::set_enforce_interest(bool enforce) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _enforce_interest = enforce;
}

bool state_cluster_shard::enforce_interest() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _enforce_interest;
}

void state_cluster_shard::set_enforce_partition(bool enforce) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _enforce_partition = enforce;
}

bool state_cluster_shard::enforce_partition() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _enforce_partition;
}

void state_cluster_shard::set_blob_store(state_blob_store *store) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _blob_store = store;
}

state_blob_store *state_cluster_shard::blob_store() const noexcept { return _blob_store; }

void state_cluster_shard::set_max_inline_payload_bytes(std::uint32_t bytes) noexcept {
  _max_inline_payload_bytes = bytes;
}

std::uint32_t state_cluster_shard::max_inline_payload_bytes() const noexcept {
  return _max_inline_payload_bytes;
}

void state_cluster_shard::set_transport(state_transport *t) noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  _transport = t;
}

state_transport *state_cluster_shard::transport() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _transport;
}

state_cluster_shard *state_cluster_shard::default_for(const app &ctx) noexcept {
  std::lock_guard<std::mutex> lk(default_registry_mutex());
  auto it = default_registry().find(&ctx);
  if (it == default_registry().end())
    return nullptr;
  return it->second;
}

void state_cluster_shard::install_as_default() {
  if (_app_ctx == nullptr)
    return;
  std::lock_guard<std::mutex> lk(default_registry_mutex());
  auto &slot = default_registry()[_app_ctx];
  // First-writer wins: do not displace an existing default.
  if (slot == nullptr)
    slot = this;
}

void state_cluster_shard::uninstall_as_default() {
  if (_app_ctx == nullptr)
    return;
  std::lock_guard<std::mutex> lk(default_registry_mutex());
  auto it = default_registry().find(_app_ctx);
  if (it != default_registry().end() && it->second == this)
    default_registry().erase(it);
}

state_cluster_shard::send_message_result state_cluster_shard::send_message(state_message m) {
  send_message_result r;

  // Resolve the owning cluster from the authority map. The
  // caller's m.cluster_id (if any) is ignored on purpose: this
  // method is the cluster-agnostic entry point.
  auto auth = _delegation->authority().resolve(m.path);
  if (auth.valid && !auth.cluster_id.empty()) {
    r.owner_cluster_id = auth.cluster_id;
  } else {
    r.owner_cluster_id = _cluster_id;
  }
  r.owner_is_local = (r.owner_cluster_id == _cluster_id);

  // Stamp the message.
  m.cluster_id = r.owner_cluster_id;
  if (m.origin_node_id.empty())
    m.origin_node_id = _local_node_id;
  if (m.message_id.empty()) {
    static std::atomic<std::uint64_t> seq{0};
    m.message_id = std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  }

  // Local delivery: only when we own the path. Cross-cluster
  // messages must not be admitted into our own local bus; they
  // belong to a foreign cluster's subscribers.
  if (r.owner_is_local) {
    bool admitted = _message_bus->admit(m);
    if (admitted) {
      r.local_admitted = 1;
    } else {
      // m.origin_node_id+m.message_id pair was seen before.
      r.status = send_message_result::status_kind::duplicate_local;
    }
  }

  // Wire fan-out: hand to the configured transport. The
  // transport is responsible for routing to peer shards (same
  // cluster on inproc) or across cluster boundaries (grpc).
  state_transport *t;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    t = _transport;
  }
  if (t == nullptr) {
    if (!r.owner_is_local)
      r.status = send_message_result::status_kind::no_transport;
    return r;
  }
  auto ps = t->publish_message(m);
  r.peers_delivered = ps.delivered;
  r.peers_targeted = ps.peers;
  return r;
}

std::vector<state_cluster_shard::snapshot_entry>
state_cluster_shard::snapshot(const std::string &path_prefix) const {
  std::vector<snapshot_entry> result;
  if (!_app_ctx)
    return result;

  // Start at root (or prefix).
  auto &root = state::instance(*_app_ctx)(path_prefix);
  root.traverse([&](std::string child_path) {
    auto &node = state::instance(*_app_ctx)(child_path);
    snapshot_entry e;
    e.path = child_path;
    e.string_value = node.value();
    e.comment = node.comment();
    e.hidden = node.hidden();
    e.read_only = node.readOnly();
    result.push_back(std::move(e));
  });
  return result;
}

} // namespace cvc

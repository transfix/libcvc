/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_node_telemetry.h>
#include <cvc/state_peer_registry.h>
#include <cvc/state_transport.h>
#include <cvc/state_transport_inproc.h>
#include <sstream>

namespace CVC_NAMESPACE {

// ---------------------------------------------------------------
// ewma
// ---------------------------------------------------------------

ewma::ewma(std::uint64_t half_life_ns) noexcept : _half_life_ns(half_life_ns) {}

void ewma::update(double sample, std::uint64_t now_ns) noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  if (!_initialized) {
    _value = sample;
    _last_ns = now_ns;
    _initialized = true;
    return;
  }
  if (now_ns <= _last_ns) {
    _value = sample;
    return;
  }
  double dt = static_cast<double>(now_ns - _last_ns);
  double alpha = 1.0 - std::exp(-dt * 0.693147180559945 / static_cast<double>(_half_life_ns));
  _value += alpha * (sample - _value);
  _last_ns = now_ns;
}

double ewma::value() const noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  return _value;
}

void ewma::reset() noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  _value = 0.0;
  _last_ns = 0;
  _initialized = false;
}

// ---------------------------------------------------------------
// latency_histogram
// ---------------------------------------------------------------

// Bucket boundaries: [0, 1024), [1024, 2048), [2048, 4096), ...
// The last bucket catches everything >= 2^30 (~1.07 s).
std::size_t latency_histogram::bucket_index(std::uint64_t ns) noexcept {
  if (ns < 1024)
    return 0;
  // __builtin_clzll gives the number of leading zeros for a 64-bit
  // unsigned. For ns >= 1024 (2^10), the bucket index is
  // floor(log2(ns)) - 9, clamped to BUCKET_COUNT - 1.
  unsigned leading = static_cast<unsigned>(__builtin_clzll(ns));
  unsigned log2_floor = 63u - leading;
  std::size_t idx = static_cast<std::size_t>(log2_floor - 9);
  return idx < BUCKET_COUNT - 1 ? idx : BUCKET_COUNT - 1;
}

void latency_histogram::record(std::uint64_t latency_ns) noexcept {
  _buckets[bucket_index(latency_ns)].fetch_add(1, std::memory_order_relaxed);
}

std::uint64_t latency_histogram::count() const noexcept {
  std::uint64_t total = 0;
  for (std::size_t i = 0; i < BUCKET_COUNT; ++i)
    total += _buckets[i].load(std::memory_order_relaxed);
  return total;
}

std::uint64_t latency_histogram::percentile(double p) const noexcept {
  std::uint64_t total = count();
  if (total == 0)
    return 0;
  std::uint64_t target = static_cast<std::uint64_t>(static_cast<double>(total) * p);
  if (target >= total)
    target = total - 1;
  std::uint64_t cumulative = 0;
  for (std::size_t i = 0; i < BUCKET_COUNT; ++i) {
    cumulative += _buckets[i].load(std::memory_order_relaxed);
    if (cumulative > target) {
      // Return the upper boundary of this bucket.
      if (i == 0)
        return 1024ULL;
      if (i >= BUCKET_COUNT - 1)
        return 1ULL << 30;
      return 1ULL << (static_cast<unsigned>(i) + 10);
    }
  }
  return 1ULL << 30;
}

void latency_histogram::reset() noexcept {
  for (std::size_t i = 0; i < BUCKET_COUNT; ++i)
    _buckets[i].store(0, std::memory_order_relaxed);
}

std::uint64_t latency_histogram::bucket(std::size_t i) const noexcept {
  if (i >= BUCKET_COUNT)
    return 0;
  return _buckets[i].load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------
// state_node_telemetry
// ---------------------------------------------------------------

static std::uint64_t now_ns() {
  using namespace std::chrono;
  return static_cast<std::uint64_t>(
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

state_node_telemetry::state_node_telemetry(app &ctx, std::string node_id, std::string cluster_id)
    : _ctx(ctx), _node_id(std::move(node_id)), _cluster_id(std::move(cluster_id)) {}

state_node_telemetry::~state_node_telemetry() = default;

void state_node_telemetry::attach_shard(state_cluster_shard *s) noexcept { _shard = s; }
void state_node_telemetry::attach_transport(state_transport *t) noexcept { _transport = t; }
void state_node_telemetry::attach_message_bus(state_message_bus *b) noexcept { _bus = b; }

void state_node_telemetry::record_enqueue_latency(std::uint64_t ns) noexcept {
  _enqueue_hist.record(ns);
}

void state_node_telemetry::record_delivery_latency(std::uint64_t ns) noexcept {
  _delivery_hist.record(ns);
}

void state_node_telemetry::sample() {
  auto ts = now_ns();
  telemetry_snapshot snap;
  snap.node_id = _node_id;
  snap.cluster_id = _cluster_id;
  snap.timestamp_ns = ts;

  // Shard counters.
  if (_shard) {
    snap.mutations_applied = _shard->total_remote_applied();
    snap.mutations_duplicates = _shard->total_remote_duplicates();
    snap.mutations_rejected = _shard->total_remote_rejected();
    snap.mutations_conflicts = _shard->total_conflicts_detected();
    snap.delegations_routed = _shard->total_delegation_routed();
    snap.delegations_expired = _shard->total_delegation_expired();
  }

  // Transport counters.
  if (_transport) {
    // Try to downcast to inproc for detailed counters.
    auto *inproc = dynamic_cast<state_transport_inproc *>(_transport);
    if (inproc) {
      snap.mutations_published = inproc->total_published();
      snap.bytes_sent = inproc->total_published(); // proxy: count-based
      snap.bytes_received = inproc->total_delivered();
      snap.quarantined_mutations = inproc->total_quarantined_mutations();
      snap.quarantined_messages = inproc->total_quarantined_messages();
      snap.auto_isolations = inproc->total_auto_isolations();
      snap.outbox_drops =
          inproc->total_outbox_dropped_newest() + inproc->total_outbox_dropped_oldest();
      snap.slow_peer_count = static_cast<std::uint64_t>(inproc->slow_peers().size());
    }
    snap.peer_count = static_cast<std::uint64_t>(_transport->peers().size());
  }

  // Message bus counters.
  if (_bus) {
    snap.messages_admitted = _bus->total_admitted();
    snap.messages_duplicates = _bus->total_duplicates();
    snap.messages_dispatched = _bus->total_dispatched();
    snap.messages_dropped = _bus->total_dropped();
  }

  // Tree shape — count children of root.
  try {
    auto &root = state::instance(_ctx);
    snap.path_count = static_cast<std::uint64_t>(root.numChildren());
  } catch (...) {
  }

  // Rate EWMAs.
  if (_prev_sample_ns > 0 && ts > _prev_sample_ns) {
    double dt_s = static_cast<double>(ts - _prev_sample_ns) / 1e9;
    if (dt_s > 0.0) {
      double pub_delta = static_cast<double>(snap.mutations_published - _prev_mutations_published);
      _mutation_publish_rate.update(pub_delta / dt_s, ts);

      double app_delta = static_cast<double>(snap.mutations_applied - _prev_mutations_applied);
      _mutation_apply_rate.update(app_delta / dt_s, ts);

      double msg_delta = static_cast<double>(snap.messages_admitted - _prev_messages_admitted);
      _message_admit_rate.update(msg_delta / dt_s, ts);
    }
  }
  _prev_mutations_published = snap.mutations_published;
  _prev_mutations_applied = snap.mutations_applied;
  _prev_messages_admitted = snap.messages_admitted;
  _prev_sample_ns = ts;

  snap.mutation_publish_rate = _mutation_publish_rate.value();
  snap.mutation_apply_rate = _mutation_apply_rate.value();
  snap.message_admit_rate = _message_admit_rate.value();

  // Latency histograms.
  snap.enqueue_p50 = _enqueue_hist.p50();
  snap.enqueue_p90 = _enqueue_hist.p90();
  snap.enqueue_p99 = _enqueue_hist.p99();
  snap.delivery_p50 = _delivery_hist.p50();
  snap.delivery_p90 = _delivery_hist.p90();
  snap.delivery_p99 = _delivery_hist.p99();

  {
    std::lock_guard<std::mutex> lk(_snap_mu);
    _latest = snap;
  }
}

telemetry_snapshot state_node_telemetry::snapshot() const {
  std::lock_guard<std::mutex> lk(_snap_mu);
  return _latest;
}

// ---------------------------------------------------------------
// JSON serialization — lightweight, no external dependency
// ---------------------------------------------------------------

static void json_kv(std::ostringstream &os, const char *key, std::uint64_t val, bool &first) {
  if (!first)
    os << ',';
  os << '"' << key << '"' << ':' << val;
  first = false;
}

static void json_kv(std::ostringstream &os, const char *key, double val, bool &first) {
  if (!first)
    os << ',';
  os << '"' << key << '"' << ':' << val;
  first = false;
}

static void json_kv(std::ostringstream &os, const char *key, const std::string &val, bool &first) {
  if (!first)
    os << ',';
  os << '"' << key << "\":\"" << val << '"';
  first = false;
}

std::string state_node_telemetry::serialize_json(const telemetry_snapshot &s) {
  std::ostringstream os;
  os << '{';
  bool f = true;
  json_kv(os, "node_id", s.node_id, f);
  json_kv(os, "cluster_id", s.cluster_id, f);
  json_kv(os, "timestamp_ns", s.timestamp_ns, f);
  json_kv(os, "mutations_published", s.mutations_published, f);
  json_kv(os, "mutations_applied", s.mutations_applied, f);
  json_kv(os, "mutations_duplicates", s.mutations_duplicates, f);
  json_kv(os, "mutations_rejected", s.mutations_rejected, f);
  json_kv(os, "mutations_conflicts", s.mutations_conflicts, f);
  json_kv(os, "messages_admitted", s.messages_admitted, f);
  json_kv(os, "messages_duplicates", s.messages_duplicates, f);
  json_kv(os, "messages_dispatched", s.messages_dispatched, f);
  json_kv(os, "messages_dropped", s.messages_dropped, f);
  json_kv(os, "bytes_sent", s.bytes_sent, f);
  json_kv(os, "bytes_received", s.bytes_received, f);
  json_kv(os, "blobs_stored", s.blobs_stored, f);
  json_kv(os, "blob_bytes", s.blob_bytes, f);
  json_kv(os, "delegations_routed", s.delegations_routed, f);
  json_kv(os, "delegations_expired", s.delegations_expired, f);
  json_kv(os, "quarantined_mutations", s.quarantined_mutations, f);
  json_kv(os, "quarantined_messages", s.quarantined_messages, f);
  json_kv(os, "auto_isolations", s.auto_isolations, f);
  json_kv(os, "mutation_publish_rate", s.mutation_publish_rate, f);
  json_kv(os, "mutation_apply_rate", s.mutation_apply_rate, f);
  json_kv(os, "message_admit_rate", s.message_admit_rate, f);
  json_kv(os, "outbox_depth", s.outbox_depth, f);
  json_kv(os, "outbox_drops", s.outbox_drops, f);
  json_kv(os, "path_count", s.path_count, f);
  json_kv(os, "link_count", s.link_count, f);
  json_kv(os, "enqueue_p50", s.enqueue_p50, f);
  json_kv(os, "enqueue_p90", s.enqueue_p90, f);
  json_kv(os, "enqueue_p99", s.enqueue_p99, f);
  json_kv(os, "delivery_p50", s.delivery_p50, f);
  json_kv(os, "delivery_p90", s.delivery_p90, f);
  json_kv(os, "delivery_p99", s.delivery_p99, f);
  json_kv(os, "peer_count", s.peer_count, f);
  json_kv(os, "slow_peer_count", s.slow_peer_count, f);
  os << '}';
  return os.str();
}

// Minimal JSON parser for telemetry snapshots — extract key:value
// pairs from a flat JSON object.
static bool extract_u64(const std::string &json, const char *key, std::uint64_t &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && (json[pos] == ' ' || json[pos] == '"'))
    ++pos;
  char *end = nullptr;
  out = std::strtoull(json.c_str() + pos, &end, 10);
  return end != json.c_str() + pos;
}

static bool extract_double(const std::string &json, const char *key, double &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && json[pos] == ' ')
    ++pos;
  char *end = nullptr;
  out = std::strtod(json.c_str() + pos, &end);
  return end != json.c_str() + pos;
}

static bool extract_string(const std::string &json, const char *key, std::string &out) {
  std::string needle = std::string("\"") + key + "\":\"";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  auto end = json.find('"', pos);
  if (end == std::string::npos)
    return false;
  out = json.substr(pos, end - pos);
  return true;
}

bool state_node_telemetry::deserialize_json(const std::string &json, telemetry_snapshot &s) {
  if (json.empty() || json[0] != '{')
    return false;

  extract_string(json, "node_id", s.node_id);
  extract_string(json, "cluster_id", s.cluster_id);
  extract_u64(json, "timestamp_ns", s.timestamp_ns);
  extract_u64(json, "mutations_published", s.mutations_published);
  extract_u64(json, "mutations_applied", s.mutations_applied);
  extract_u64(json, "mutations_duplicates", s.mutations_duplicates);
  extract_u64(json, "mutations_rejected", s.mutations_rejected);
  extract_u64(json, "mutations_conflicts", s.mutations_conflicts);
  extract_u64(json, "messages_admitted", s.messages_admitted);
  extract_u64(json, "messages_duplicates", s.messages_duplicates);
  extract_u64(json, "messages_dispatched", s.messages_dispatched);
  extract_u64(json, "messages_dropped", s.messages_dropped);
  extract_u64(json, "bytes_sent", s.bytes_sent);
  extract_u64(json, "bytes_received", s.bytes_received);
  extract_u64(json, "blobs_stored", s.blobs_stored);
  extract_u64(json, "blob_bytes", s.blob_bytes);
  extract_u64(json, "delegations_routed", s.delegations_routed);
  extract_u64(json, "delegations_expired", s.delegations_expired);
  extract_u64(json, "quarantined_mutations", s.quarantined_mutations);
  extract_u64(json, "quarantined_messages", s.quarantined_messages);
  extract_u64(json, "auto_isolations", s.auto_isolations);
  extract_double(json, "mutation_publish_rate", s.mutation_publish_rate);
  extract_double(json, "mutation_apply_rate", s.mutation_apply_rate);
  extract_double(json, "message_admit_rate", s.message_admit_rate);
  extract_u64(json, "outbox_depth", s.outbox_depth);
  extract_u64(json, "outbox_drops", s.outbox_drops);
  extract_u64(json, "path_count", s.path_count);
  extract_u64(json, "link_count", s.link_count);
  extract_u64(json, "enqueue_p50", s.enqueue_p50);
  extract_u64(json, "enqueue_p90", s.enqueue_p90);
  extract_u64(json, "enqueue_p99", s.enqueue_p99);
  extract_u64(json, "delivery_p50", s.delivery_p50);
  extract_u64(json, "delivery_p90", s.delivery_p90);
  extract_u64(json, "delivery_p99", s.delivery_p99);
  extract_u64(json, "peer_count", s.peer_count);
  extract_u64(json, "slow_peer_count", s.slow_peer_count);
  return !s.node_id.empty();
}

bool state_node_telemetry::publish_snapshot() {
  if (!_bus)
    return false;
  auto snap = snapshot();
  std::string json = serialize_json(snap);
  std::string path = std::string(TOPIC_PREFIX) + _cluster_id + "." + _node_id;
  auto msg = state_message::make_text(path, json, MIME_TELEMETRY);
  msg.cluster_id = _cluster_id;
  msg.origin_node_id = _node_id;
  msg.message_id = "tel-" + std::to_string(++_publish_seq);
  msg.ttl_hops = 4;
  return _bus->admit(msg);
}

} // namespace CVC_NAMESPACE

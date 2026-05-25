/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_node_telemetry.h>
#include <cvc/core/state_telemetry_aggregator.h>
#include <sstream>

namespace cvc {

static std::uint64_t agg_now_ns() {
  using namespace std::chrono;
  return static_cast<std::uint64_t>(
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

state_telemetry_aggregator::state_telemetry_aggregator(std::string cluster_id,
                                                       std::uint64_t stale_threshold_ns)
    : _cluster_id(std::move(cluster_id)), _stale_threshold_ns(stale_threshold_ns) {}

state_telemetry_aggregator::~state_telemetry_aggregator() { detach_bus(); }

bool state_telemetry_aggregator::attach_bus(state_message_bus &bus) {
  detach_bus();
  _bus = &bus;
  std::string prefix = std::string(state_node_telemetry::TOPIC_PREFIX) + _cluster_id;
  _sub_id = bus.subscribe(prefix, [this](const state_message &msg) { on_message(msg); });
  return _sub_id != 0;
}

void state_telemetry_aggregator::detach_bus() {
  if (_bus && _sub_id) {
    _bus->unsubscribe(_sub_id);
    _sub_id = 0;
  }
  _bus = nullptr;
}

void state_telemetry_aggregator::on_message(const state_message &msg) {
  if (msg.content_type != state_node_telemetry::MIME_TELEMETRY &&
      msg.effective_content_type() != state_node_telemetry::MIME_TELEMETRY)
    return;

  telemetry_snapshot snap;
  if (!state_node_telemetry::deserialize_json(msg.string_value, snap))
    return;

  ingest(snap);
}

void state_telemetry_aggregator::ingest(const telemetry_snapshot &snap) {
  std::lock_guard<std::mutex> lk(_mu);
  _peers[snap.node_id] = snap;
}

cluster_telemetry_summary state_telemetry_aggregator::summarize() const {
  std::lock_guard<std::mutex> lk(_mu);
  cluster_telemetry_summary sum;
  sum.timestamp_ns = agg_now_ns();
  sum.node_count = _peers.size();

  for (auto &[id, snap] : _peers) {
    if (sum.timestamp_ns > snap.timestamp_ns &&
        (sum.timestamp_ns - snap.timestamp_ns) > _stale_threshold_ns) {
      ++sum.stale_count;
    }
    sum.total_mutations_published += snap.mutations_published;
    sum.total_mutations_applied += snap.mutations_applied;
    sum.total_messages_admitted += snap.messages_admitted;
    sum.total_bytes_sent += snap.bytes_sent;
    sum.total_bytes_received += snap.bytes_received;
    sum.total_blobs_stored += snap.blobs_stored;
    sum.total_blob_bytes += snap.blob_bytes;
    sum.total_path_count += snap.path_count;
    sum.total_link_count += snap.link_count;
    sum.max_enqueue_p99 = std::max(sum.max_enqueue_p99, snap.enqueue_p99);
    sum.max_delivery_p99 = std::max(sum.max_delivery_p99, snap.delivery_p99);
    sum.cluster_mutation_publish_rate += snap.mutation_publish_rate;
    sum.cluster_mutation_apply_rate += snap.mutation_apply_rate;
    sum.cluster_message_admit_rate += snap.message_admit_rate;
  }
  return sum;
}

std::unordered_map<std::string, telemetry_snapshot>
state_telemetry_aggregator::peer_snapshots() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers;
}

std::size_t state_telemetry_aggregator::peer_count() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers.size();
}

std::size_t state_telemetry_aggregator::stale_count() const {
  std::lock_guard<std::mutex> lk(_mu);
  auto now = agg_now_ns();
  std::size_t count = 0;
  for (auto &[id, snap] : _peers) {
    if (now > snap.timestamp_ns && (now - snap.timestamp_ns) > _stale_threshold_ns)
      ++count;
  }
  return count;
}

bool state_telemetry_aggregator::remove_peer(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers.erase(node_id) > 0;
}

void state_telemetry_aggregator::clear() {
  std::lock_guard<std::mutex> lk(_mu);
  _peers.clear();
}

std::string state_telemetry_aggregator::to_text() const {
  auto sum = summarize();
  std::lock_guard<std::mutex> lk(_mu);

  std::ostringstream os;
  os << "[telemetry]\n";
  os << "  cluster_id: " << _cluster_id << "\n";
  os << "  nodes: " << sum.node_count << " (" << sum.stale_count << " stale)\n";
  os << "  mutations_published: " << sum.total_mutations_published << "\n";
  os << "  mutations_applied: " << sum.total_mutations_applied << "\n";
  os << "  messages_admitted: " << sum.total_messages_admitted << "\n";
  os << "  bytes_sent: " << sum.total_bytes_sent << "\n";
  os << "  bytes_received: " << sum.total_bytes_received << "\n";
  os << "  total_paths: " << sum.total_path_count << "\n";
  os << "  total_links: " << sum.total_link_count << "\n";
  os << "  max_enqueue_p99: " << sum.max_enqueue_p99 << " ns\n";
  os << "  max_delivery_p99: " << sum.max_delivery_p99 << " ns\n";
  os << "  cluster_mutation_rate: " << sum.cluster_mutation_publish_rate << " /s\n";
  os << "  cluster_apply_rate: " << sum.cluster_mutation_apply_rate << " /s\n";
  os << "  cluster_message_rate: " << sum.cluster_message_admit_rate << " /s\n";

  for (auto &[id, snap] : _peers) {
    os << "  [peer " << id << "]\n";
    os << "    cluster: " << snap.cluster_id << "\n";
    os << "    mutations: pub=" << snap.mutations_published << " app=" << snap.mutations_applied
       << " dup=" << snap.mutations_duplicates << " rej=" << snap.mutations_rejected << "\n";
    os << "    messages: adm=" << snap.messages_admitted << " dup=" << snap.messages_duplicates
       << " drop=" << snap.messages_dropped << "\n";
    os << "    rates: pub=" << snap.mutation_publish_rate << "/s"
       << " app=" << snap.mutation_apply_rate << "/s" << " msg=" << snap.message_admit_rate
       << "/s\n";
    os << "    latency: enq_p99=" << snap.enqueue_p99 << "ns" << " del_p99=" << snap.delivery_p99
       << "ns\n";
    os << "    peers: " << snap.peer_count << " (slow=" << snap.slow_peer_count << ")\n";
  }
  return os.str();
}

state_telemetry_aggregator::routing_feedback_result
state_telemetry_aggregator::evaluate_routing_feedback(const routing_feedback_policy &policy) const {
  routing_feedback_result result;
  std::lock_guard<std::mutex> lk(_mu);

  for (auto &[id, snap] : _peers) {
    bool should_isolate = false;

    if (policy.latency_p99_threshold_ns > 0 &&
        snap.delivery_p99 > policy.latency_p99_threshold_ns) {
      should_isolate = true;
    }

    if (policy.outbox_drop_threshold > 0 && snap.outbox_drops > policy.outbox_drop_threshold) {
      should_isolate = true;
    }

    if (should_isolate) {
      result.isolate.push_back(id);
    } else {
      // Only suggest releasing if the peer was potentially isolated.
      // Callers decide whether the peer is actually marked slow.
      result.release.push_back(id);
    }
  }
  return result;
}

} // namespace cvc

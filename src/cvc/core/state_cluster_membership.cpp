/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cvc/core/state_cluster_membership.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_message.h>
#include <cvc/core/state_peer_registry.h>
#include <cvc/core/state_replica.h>
#include <cvc/core/state_transport.h>
#include <sstream>

namespace cvc {

namespace {

constexpr const char *MEMBERSHIP_PREFIX = "__system.membership.";
constexpr const char *HB_CONTENT_TYPE = "application/x-cvc-heartbeat";

std::string heartbeat_path(const std::string &cluster_id, const std::string &node_id) {
  std::string p = MEMBERSHIP_PREFIX;
  p += cluster_id;
  p += '.';
  p += node_id;
  return p;
}

std::uint64_t steady_now_ns() {
  auto tp = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count());
}

} // namespace

// ---------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------

state_cluster_membership::state_cluster_membership(std::string cluster_id,
                                                   std::string local_node_id)
    : _cluster_id(std::move(cluster_id)), _local_node_id(std::move(local_node_id)) {}

state_cluster_membership::~state_cluster_membership() { stop(); }

// ---------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------

void state_cluster_membership::set_shard(state_cluster_shard *shard) noexcept { _shard = shard; }

void state_cluster_membership::set_transport(state_transport *transport) noexcept {
  _transport = transport;
}

void state_cluster_membership::set_peer_registry(state_peer_registry *peers) noexcept {
  _peer_registry = peers;
}

void state_cluster_membership::set_config(config cfg) noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  _config = cfg;
}

state_cluster_membership::config state_cluster_membership::current_config() const noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  return _config;
}

// ---------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------

std::size_t state_cluster_membership::add_callback(event_callback cb) {
  std::lock_guard<std::mutex> lk(_cb_mu);
  auto id = _next_cb_id++;
  _callbacks.emplace(id, std::move(cb));
  return id;
}

bool state_cluster_membership::remove_callback(std::size_t id) {
  std::lock_guard<std::mutex> lk(_cb_mu);
  return _callbacks.erase(id) > 0;
}

void state_cluster_membership::fire_event(const membership_event &ev) {
  std::lock_guard<std::mutex> lk(_cb_mu);
  for (auto &[id, cb] : _callbacks) {
    cb(ev);
  }
}

// ---------------------------------------------------------------
// Clock
// ---------------------------------------------------------------

void state_cluster_membership::set_clock(clock_fn fn) {
  std::lock_guard<std::mutex> lk(_mu);
  _clock_fn = std::move(fn);
}

std::uint64_t state_cluster_membership::now_ns() const {
  std::lock_guard<std::mutex> lk(_mu);
  if (_clock_fn)
    return _clock_fn();
  return steady_now_ns();
}

// ---------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------

void state_cluster_membership::start() {
  if (_running.exchange(true))
    return; // already running
  _stop_requested.store(false);
  _tick_thread = std::thread(&state_cluster_membership::tick_loop, this);
}

void state_cluster_membership::stop() {
  if (!_running.load())
    return;
  _stop_requested.store(true);
  if (_tick_thread.joinable())
    _tick_thread.join();
  _running.store(false);
}

// ---------------------------------------------------------------
// Heartbeat emission
// ---------------------------------------------------------------

void state_cluster_membership::emit_heartbeat(std::uint64_t ts_ns) {
  // Encode timestamp as string payload. The heartbeat path encodes
  // the sender's identity; the payload carries the endpoint (if
  // known) so receivers can discover how to connect back.
  std::string path = heartbeat_path(_cluster_id, _local_node_id);

  // Build a small text payload with timestamp + endpoint info.
  std::ostringstream payload;
  payload << ts_ns;

  auto msg = state_message::make_text(path, payload.str(), HB_CONTENT_TYPE);
  msg.cluster_id = _cluster_id;
  msg.origin_node_id = _local_node_id;
  // Empty message_id: heartbeats are fire-and-forget, not deduped.
  // Callers receive the latest one; older ones are irrelevant.
  msg.ttl_hops = 1;

  if (_shard) {
    _shard->send_message(std::move(msg));
  } else if (_transport) {
    _transport->publish_message(msg);
  }

  _ctr_hb_sent.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------
// Inbound heartbeat processing
// ---------------------------------------------------------------

void state_cluster_membership::on_heartbeat(const std::string &node_id,
                                            const std::string &cluster_id,
                                            const std::string &endpoint,
                                            std::uint64_t timestamp_ns) {
  if (node_id == _local_node_id)
    return; // ignore own heartbeats
  if (cluster_id != _cluster_id)
    return; // wrong cluster

  _ctr_hb_recv.fetch_add(1, std::memory_order_relaxed);

  bool is_new = false;
  {
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _peers.find(node_id);
    if (it == _peers.end()) {
      // New peer discovered via heartbeat.
      peer_status ps;
      ps.node_id = node_id;
      ps.cluster_id = cluster_id;
      ps.endpoint = endpoint;
      ps.state = peer_state::alive;
      ps.last_heartbeat_ns = timestamp_ns;
      ps.state_changed_ns = timestamp_ns;
      _peers.emplace(node_id, ps);
      is_new = true;
    } else {
      // Existing peer: refresh heartbeat time and restore to alive
      // if it was suspect or dead.
      auto &ps = it->second;
      ps.last_heartbeat_ns = timestamp_ns;
      if (!endpoint.empty())
        ps.endpoint = endpoint;
      if (ps.state != peer_state::alive) {
        ps.state = peer_state::alive;
        ps.state_changed_ns = timestamp_ns;
      }
    }
  }

  // Update the peer_registry liveness timestamp.
  if (_peer_registry) {
    if (is_new) {
      _peer_registry->add_peer(node_id, cluster_id, endpoint);
    }
    _peer_registry->note_seen(node_id, timestamp_ns);
  }

  // Update the replica if available.
  if (_shard) {
    auto &replica = _shard->replica();
    if (!replica.has_peer(node_id))
      replica.add_peer(node_id);
    if (is_new || true) // always mark alive on heartbeat
      replica.mark_alive(node_id, true);
  }

  if (is_new) {
    _ctr_join.fetch_add(1, std::memory_order_relaxed);
    fire_event({event_kind::peer_joined, node_id, cluster_id, timestamp_ns});
  }
}

// ---------------------------------------------------------------
// Manual peer registration (e.g. seeds)
// ---------------------------------------------------------------

void state_cluster_membership::register_peer(const std::string &node_id,
                                             const std::string &cluster_id,
                                             const std::string &endpoint) {
  auto ts = now_ns();
  {
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _peers.find(node_id);
    if (it != _peers.end())
      return; // already known

    peer_status ps;
    ps.node_id = node_id;
    ps.cluster_id = cluster_id;
    ps.endpoint = endpoint;
    ps.state = peer_state::alive;
    ps.last_heartbeat_ns = ts;
    ps.state_changed_ns = ts;
    _peers.emplace(node_id, ps);
  }

  if (_peer_registry)
    _peer_registry->add_peer(node_id, cluster_id, endpoint);

  if (_shard) {
    auto &replica = _shard->replica();
    if (!replica.has_peer(node_id))
      replica.add_peer(node_id);
  }

  _ctr_join.fetch_add(1, std::memory_order_relaxed);
  fire_event({event_kind::peer_joined, node_id, cluster_id, ts});
}

// ---------------------------------------------------------------
// Peer scanning / failure detection
// ---------------------------------------------------------------

void state_cluster_membership::scan_peers(std::uint64_t ts_ns) {
  config cfg;
  {
    std::lock_guard<std::mutex> lk(_mu);
    cfg = _config;
  }

  auto suspect_ns = static_cast<std::uint64_t>(cfg.suspect_timeout_ms) * 1'000'000ULL;
  auto dead_ns = static_cast<std::uint64_t>(cfg.dead_timeout_ms) * 1'000'000ULL;
  auto evict_ns = static_cast<std::uint64_t>(cfg.evict_timeout_ms) * 1'000'000ULL;

  std::vector<membership_event> events;
  std::vector<std::string> to_evict;

  {
    std::lock_guard<std::mutex> lk(_mu);
    for (auto &[nid, ps] : _peers) {
      auto age = (ts_ns > ps.last_heartbeat_ns) ? (ts_ns - ps.last_heartbeat_ns) : 0ULL;

      if (ps.state == peer_state::alive && age >= suspect_ns) {
        ps.state = peer_state::suspect;
        ps.state_changed_ns = ts_ns;
        _ctr_suspect.fetch_add(1, std::memory_order_relaxed);
        events.push_back({event_kind::peer_suspect, nid, ps.cluster_id, ts_ns});
      }

      if (ps.state == peer_state::suspect && age >= dead_ns) {
        ps.state = peer_state::dead;
        ps.state_changed_ns = ts_ns;
        _ctr_dead.fetch_add(1, std::memory_order_relaxed);
        events.push_back({event_kind::peer_dead, nid, ps.cluster_id, ts_ns});
      }

      if (ps.state == peer_state::dead && age >= evict_ns) {
        to_evict.push_back(nid);
      }
    }

    for (auto &nid : to_evict) {
      auto it = _peers.find(nid);
      std::string cid;
      if (it != _peers.end()) {
        cid = it->second.cluster_id;
        _peers.erase(it);
      }
      _ctr_evict.fetch_add(1, std::memory_order_relaxed);
      events.push_back({event_kind::peer_evicted, nid, cid, ts_ns});
    }
  }

  // Evict from peer_registry and replica outside the lock.
  for (auto &nid : to_evict) {
    if (_peer_registry)
      _peer_registry->remove_peer(nid);
    if (_shard) {
      _shard->replica().mark_alive(nid, false);
      _shard->replica().remove_peer(nid);
    }
  }

  // Fire events outside the lock.
  for (auto &ev : events)
    fire_event(ev);
}

// ---------------------------------------------------------------
// Tick loop
// ---------------------------------------------------------------

void state_cluster_membership::tick_loop() {
  while (!_stop_requested.load()) {
    auto ts = now_ns();
    emit_heartbeat(ts);
    scan_peers(ts);

    std::uint32_t interval_ms;
    {
      std::lock_guard<std::mutex> lk(_mu);
      interval_ms = _config.heartbeat_interval_ms;
    }

    // Sleep in small increments so stop() doesn't block for the
    // full interval.
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval_ms);
    while (!_stop_requested.load() && std::chrono::steady_clock::now() < end)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// ---------------------------------------------------------------
// Snapshot / query
// ---------------------------------------------------------------

std::vector<state_cluster_membership::peer_status> state_cluster_membership::peer_snapshot() const {
  std::lock_guard<std::mutex> lk(_mu);
  std::vector<peer_status> out;
  out.reserve(_peers.size());
  for (auto &[nid, ps] : _peers)
    out.push_back(ps);
  return out;
}

bool state_cluster_membership::get_peer(const std::string &node_id, peer_status &out) const {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return false;
  out = it->second;
  return true;
}

} // namespace cvc

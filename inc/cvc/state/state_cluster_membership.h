/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_CLUSTER_MEMBERSHIP_H__
#define __CVC_STATE_CLUSTER_MEMBERSHIP_H__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace cvc {

class state_cluster_shard;
class state_peer_registry;
class state_transport;

// ----------------
// cvc::state_cluster_membership
// ----------------
// Phase 10: automatic cluster membership lifecycle.
//
// Builds on the manual peer_registry + replica primitives by adding
// heartbeat emission, failure detection, and automatic peer
// eviction. The component runs a background tick loop that:
//
//   1. Emits a heartbeat out-of-band message at
//      __system.membership.<cluster_id>.<local_node_id> so other
//      nodes know this process is alive.
//
//   2. Scans the peer_registry for stale peers. Peers transition
//      through three states based on time since last heartbeat:
//        alive   -> suspect  (after suspect_timeout)
//        suspect -> dead     (after dead_timeout)
//        dead    -> evicted  (removed from peer_registry + replica)
//
//   3. Fires event callbacks so other components can react to
//      membership changes (e.g. rebalance shard authority, update
//      UI, log warnings).
//
// Inbound heartbeats are processed by subscribing to the message
// bus for the __system.membership prefix. When a heartbeat arrives,
// the membership manager updates peer_registry::note_seen() and
// auto-registers unknown peers.
//
// Threading:
//   All public methods are thread-safe. The tick loop runs in a
//   dedicated thread started by start() and joined by stop().
//
class state_cluster_membership {
public:
  // Peer lifecycle state as observed by this node's failure
  // detector. This is distinct from the transport-level connection
  // state; a peer can be reachable at the transport level but stop
  // sending heartbeats, or vice versa.
  enum class peer_state { alive, suspect, dead };

  struct peer_status {
    std::string node_id;
    std::string cluster_id;
    std::string endpoint;
    peer_state state = peer_state::alive;
    std::uint64_t last_heartbeat_ns = 0;
    std::uint64_t state_changed_ns = 0;
  };

  // Membership lifecycle events.
  enum class event_kind { peer_joined, peer_suspect, peer_dead, peer_evicted };

  struct membership_event {
    event_kind kind;
    std::string node_id;
    std::string cluster_id;
    std::uint64_t timestamp_ns = 0;
  };

  // Callback type for membership events. Fired from the tick loop
  // thread; handlers should be non-blocking.
  using event_callback = std::function<void(const membership_event &)>;

  struct config {
    // How often to emit heartbeats and scan for stale peers.
    std::uint32_t heartbeat_interval_ms = 1000;

    // After this duration without a heartbeat a peer becomes
    // suspect. Default: 3x heartbeat interval.
    std::uint32_t suspect_timeout_ms = 3000;

    // After this duration without a heartbeat a suspect peer is
    // declared dead. Default: 5x heartbeat interval.
    std::uint32_t dead_timeout_ms = 5000;

    // After this duration a dead peer is evicted from the registry.
    // Default: 10x heartbeat interval.
    std::uint32_t evict_timeout_ms = 10000;
  };

  // Construct with the cluster's identity. Does not start the tick
  // loop; call start() after wiring up the shard and transport.
  state_cluster_membership(std::string cluster_id, std::string local_node_id);

  ~state_cluster_membership();

  state_cluster_membership(const state_cluster_membership &) = delete;
  state_cluster_membership &operator=(const state_cluster_membership &) = delete;

  const std::string &cluster_id() const noexcept { return _cluster_id; }
  const std::string &local_node_id() const noexcept { return _local_node_id; }

  // Wire up dependencies. Must be called before start().
  void set_shard(state_cluster_shard *shard) noexcept;
  void set_transport(state_transport *transport) noexcept;
  void set_peer_registry(state_peer_registry *peers) noexcept;

  // Install an event callback. May be called before or after
  // start(). Multiple callbacks are supported; each receives every
  // event. Returns an id that can be passed to remove_callback().
  std::size_t add_callback(event_callback cb);
  bool remove_callback(std::size_t id);

  void set_config(config cfg) noexcept;
  config current_config() const noexcept;

  // Start the background tick loop.
  void start();

  // Stop the tick loop and join the thread.
  void stop();

  bool is_running() const noexcept { return _running.load(); }

  // Process an inbound heartbeat. Normally called from the message
  // bus subscriber, but may be called directly for testing.
  void on_heartbeat(const std::string &node_id, const std::string &cluster_id,
                    const std::string &endpoint, std::uint64_t timestamp_ns);

  // Current view of all tracked peers. Does not include the local
  // node.
  std::vector<peer_status> peer_snapshot() const;

  // Look up a single peer. Returns nullptr-like empty optional
  // if unknown.
  bool get_peer(const std::string &node_id, peer_status &out) const;

  // Manually register a peer (e.g. from seed configuration).
  void register_peer(const std::string &node_id, const std::string &cluster_id,
                     const std::string &endpoint = std::string());

  // Counters for observability.
  std::uint64_t total_heartbeats_sent() const noexcept { return _ctr_hb_sent.load(); }
  std::uint64_t total_heartbeats_received() const noexcept { return _ctr_hb_recv.load(); }
  std::uint64_t total_peers_suspected() const noexcept { return _ctr_suspect.load(); }
  std::uint64_t total_peers_declared_dead() const noexcept { return _ctr_dead.load(); }
  std::uint64_t total_peers_evicted() const noexcept { return _ctr_evict.load(); }
  std::uint64_t total_peers_joined() const noexcept { return _ctr_join.load(); }

  // Clock injection for deterministic testing. When set, the tick
  // loop and on_heartbeat use this instead of steady_clock.
  using clock_fn = std::function<std::uint64_t()>;
  void set_clock(clock_fn fn);

private:
  void tick_loop();
  void emit_heartbeat(std::uint64_t now_ns);
  void scan_peers(std::uint64_t now_ns);
  void fire_event(const membership_event &ev);

  std::uint64_t now_ns() const;

  std::string _cluster_id;
  std::string _local_node_id;

  state_cluster_shard *_shard = nullptr;
  state_transport *_transport = nullptr;
  state_peer_registry *_peer_registry = nullptr;

  config _config;

  mutable std::mutex _mu;
  std::unordered_map<std::string, peer_status> _peers;

  std::mutex _cb_mu;
  std::size_t _next_cb_id = 1;
  std::unordered_map<std::size_t, event_callback> _callbacks;

  clock_fn _clock_fn;

  std::atomic<bool> _running{false};
  std::atomic<bool> _stop_requested{false};
  std::thread _tick_thread;

  std::atomic<std::uint64_t> _ctr_hb_sent{0};
  std::atomic<std::uint64_t> _ctr_hb_recv{0};
  std::atomic<std::uint64_t> _ctr_suspect{0};
  std::atomic<std::uint64_t> _ctr_dead{0};
  std::atomic<std::uint64_t> _ctr_evict{0};
  std::atomic<std::uint64_t> _ctr_join{0};
};

} // namespace cvc

#endif // __CVC_STATE_CLUSTER_MEMBERSHIP_H__

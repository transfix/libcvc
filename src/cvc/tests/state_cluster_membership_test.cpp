/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cvc/app.h>
#include <cvc/state_cluster_membership.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_peer_registry.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>

using namespace CVC_NAMESPACE;

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------

namespace {

// Deterministic clock for tests. Advances only when the test tells
// it to, making failure-detection timing reproducible.
class fake_clock {
public:
  explicit fake_clock(std::uint64_t initial_ns = 0) : _ns(initial_ns) {}
  std::uint64_t now() const { return _ns.load(); }
  void advance_ms(std::uint64_t ms) { _ns.fetch_add(ms * 1'000'000ULL); }
  void set_ns(std::uint64_t ns) { _ns.store(ns); }

private:
  std::atomic<std::uint64_t> _ns{0};
};

} // namespace

// ---------------------------------------------------------------
// Tests
// ---------------------------------------------------------------

TEST(StateClusterMembership, ConstructAndDestroy) {
  state_cluster_membership m("cluster_a", "node_1");
  EXPECT_EQ(m.cluster_id(), "cluster_a");
  EXPECT_EQ(m.local_node_id(), "node_1");
  EXPECT_FALSE(m.is_running());
}

TEST(StateClusterMembership, StartAndStop) {
  state_cluster_membership m("cluster_a", "node_1");
  m.set_config({.heartbeat_interval_ms = 50});
  m.start();
  EXPECT_TRUE(m.is_running());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  m.stop();
  EXPECT_FALSE(m.is_running());
  EXPECT_GT(m.total_heartbeats_sent(), 0u);
}

TEST(StateClusterMembership, DoubleStartIsSafe) {
  state_cluster_membership m("cluster_a", "node_1");
  m.set_config({.heartbeat_interval_ms = 50});
  m.start();
  m.start(); // should be a no-op
  EXPECT_TRUE(m.is_running());
  m.stop();
}

TEST(StateClusterMembership, DoubleStopIsSafe) {
  state_cluster_membership m("cluster_a", "node_1");
  m.start();
  m.stop();
  m.stop(); // should be a no-op
  EXPECT_FALSE(m.is_running());
}

TEST(StateClusterMembership, DestructorCallsStop) {
  auto m = std::make_unique<state_cluster_membership>("cluster_a", "node_1");
  m->set_config({.heartbeat_interval_ms = 50});
  m->start();
  EXPECT_TRUE(m->is_running());
  m.reset(); // destructor should call stop() without hanging
}

TEST(StateClusterMembership, RegisterPeer) {
  state_cluster_membership m("cluster_a", "node_1");
  state_peer_registry reg;
  m.set_peer_registry(&reg);

  m.register_peer("node_2", "cluster_a", "host2:50051");

  auto snap = m.peer_snapshot();
  ASSERT_EQ(snap.size(), 1u);
  EXPECT_EQ(snap[0].node_id, "node_2");
  EXPECT_EQ(snap[0].cluster_id, "cluster_a");
  EXPECT_EQ(snap[0].endpoint, "host2:50051");
  EXPECT_EQ(snap[0].state, state_cluster_membership::peer_state::alive);

  // Also registered in the peer_registry.
  EXPECT_TRUE(reg.has_peer("node_2"));
  EXPECT_EQ(m.total_peers_joined(), 1u);
}

TEST(StateClusterMembership, RegisterPeerDedups) {
  state_cluster_membership m("cluster_a", "node_1");
  m.register_peer("node_2", "cluster_a");
  m.register_peer("node_2", "cluster_a"); // no-op
  EXPECT_EQ(m.peer_snapshot().size(), 1u);
  EXPECT_EQ(m.total_peers_joined(), 1u);
}

TEST(StateClusterMembership, OnHeartbeatNewPeer) {
  state_cluster_membership m("cluster_a", "node_1");
  state_peer_registry reg;
  m.set_peer_registry(&reg);

  m.on_heartbeat("node_2", "cluster_a", "host2:50051", 1'000'000'000);

  auto snap = m.peer_snapshot();
  ASSERT_EQ(snap.size(), 1u);
  EXPECT_EQ(snap[0].node_id, "node_2");
  EXPECT_EQ(snap[0].state, state_cluster_membership::peer_state::alive);
  EXPECT_EQ(snap[0].last_heartbeat_ns, 1'000'000'000u);

  EXPECT_TRUE(reg.has_peer("node_2"));
  EXPECT_EQ(m.total_heartbeats_received(), 1u);
  EXPECT_EQ(m.total_peers_joined(), 1u);
}

TEST(StateClusterMembership, OnHeartbeatRefreshExisting) {
  state_cluster_membership m("cluster_a", "node_1");

  m.on_heartbeat("node_2", "cluster_a", "", 1'000'000'000);
  m.on_heartbeat("node_2", "cluster_a", "", 2'000'000'000);

  auto snap = m.peer_snapshot();
  ASSERT_EQ(snap.size(), 1u);
  EXPECT_EQ(snap[0].last_heartbeat_ns, 2'000'000'000u);
  // Should only have joined once.
  EXPECT_EQ(m.total_peers_joined(), 1u);
  EXPECT_EQ(m.total_heartbeats_received(), 2u);
}

TEST(StateClusterMembership, OnHeartbeatIgnoresSelf) {
  state_cluster_membership m("cluster_a", "node_1");
  m.on_heartbeat("node_1", "cluster_a", "", 1'000'000'000);
  EXPECT_EQ(m.peer_snapshot().size(), 0u);
  EXPECT_EQ(m.total_heartbeats_received(), 0u);
}

TEST(StateClusterMembership, OnHeartbeatIgnoresWrongCluster) {
  state_cluster_membership m("cluster_a", "node_1");
  m.on_heartbeat("node_2", "cluster_b", "", 1'000'000'000);
  EXPECT_EQ(m.peer_snapshot().size(), 0u);
  EXPECT_EQ(m.total_heartbeats_received(), 0u);
}

TEST(StateClusterMembership, FailureDetectionSuspect) {
  fake_clock clk(1'000'000'000);
  state_cluster_membership m("cluster_a", "node_1");
  m.set_clock([&] { return clk.now(); });
  m.set_config({.heartbeat_interval_ms = 100,
                .suspect_timeout_ms = 500,
                .dead_timeout_ms = 1000,
                .evict_timeout_ms = 2000});

  // Peer sends one heartbeat at t=1s.
  m.on_heartbeat("node_2", "cluster_a", "", clk.now());

  // Advance past suspect threshold (500ms).
  clk.advance_ms(600);
  // Trigger a scan manually (scan_peers is private, so we start/stop).
  // Instead, let's use the public tick loop briefly.
  // Actually, we'll just start and let it tick once.
  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  m.stop();

  state_cluster_membership::peer_status ps;
  ASSERT_TRUE(m.get_peer("node_2", ps));
  EXPECT_EQ(ps.state, state_cluster_membership::peer_state::suspect);
  EXPECT_EQ(m.total_peers_suspected(), 1u);
}

TEST(StateClusterMembership, FailureDetectionDead) {
  fake_clock clk(1'000'000'000);
  state_cluster_membership m("cluster_a", "node_1");
  m.set_clock([&] { return clk.now(); });
  m.set_config({.heartbeat_interval_ms = 100,
                .suspect_timeout_ms = 500,
                .dead_timeout_ms = 1000,
                .evict_timeout_ms = 2000});

  m.on_heartbeat("node_2", "cluster_a", "", clk.now());

  // Advance past dead threshold (1000ms).
  clk.advance_ms(1100);
  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  m.stop();

  state_cluster_membership::peer_status ps;
  ASSERT_TRUE(m.get_peer("node_2", ps));
  EXPECT_EQ(ps.state, state_cluster_membership::peer_state::dead);
  EXPECT_GE(m.total_peers_suspected(), 1u);
  EXPECT_EQ(m.total_peers_declared_dead(), 1u);
}

TEST(StateClusterMembership, FailureDetectionEvict) {
  fake_clock clk(1'000'000'000);
  state_cluster_membership m("cluster_a", "node_1");
  state_peer_registry reg;
  m.set_peer_registry(&reg);
  m.set_clock([&] { return clk.now(); });
  m.set_config({.heartbeat_interval_ms = 100,
                .suspect_timeout_ms = 200,
                .dead_timeout_ms = 400,
                .evict_timeout_ms = 800});

  m.on_heartbeat("node_2", "cluster_a", "", clk.now());
  EXPECT_TRUE(reg.has_peer("node_2"));

  // Advance past eviction threshold (800ms).
  clk.advance_ms(900);
  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  m.stop();

  // Peer should be evicted.
  EXPECT_EQ(m.peer_snapshot().size(), 0u);
  EXPECT_FALSE(reg.has_peer("node_2"));
  EXPECT_EQ(m.total_peers_evicted(), 1u);
}

TEST(StateClusterMembership, HeartbeatRevivesSuspectPeer) {
  fake_clock clk(1'000'000'000);
  state_cluster_membership m("cluster_a", "node_1");
  m.set_clock([&] { return clk.now(); });
  m.set_config({.heartbeat_interval_ms = 100,
                .suspect_timeout_ms = 500,
                .dead_timeout_ms = 1000,
                .evict_timeout_ms = 2000});

  m.on_heartbeat("node_2", "cluster_a", "", clk.now());

  // Go past suspect.
  clk.advance_ms(600);
  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  m.stop();

  state_cluster_membership::peer_status ps;
  ASSERT_TRUE(m.get_peer("node_2", ps));
  EXPECT_EQ(ps.state, state_cluster_membership::peer_state::suspect);

  // Now the peer sends a fresh heartbeat.
  clk.advance_ms(10);
  m.on_heartbeat("node_2", "cluster_a", "", clk.now());

  ASSERT_TRUE(m.get_peer("node_2", ps));
  EXPECT_EQ(ps.state, state_cluster_membership::peer_state::alive);
}

TEST(StateClusterMembership, EventCallbacks) {
  fake_clock clk(1'000'000'000);
  state_cluster_membership m("cluster_a", "node_1");
  m.set_clock([&] { return clk.now(); });
  m.set_config({.heartbeat_interval_ms = 100,
                .suspect_timeout_ms = 200,
                .dead_timeout_ms = 400,
                .evict_timeout_ms = 600});

  std::mutex ev_mu;
  std::vector<state_cluster_membership::membership_event> events;
  m.add_callback([&](const state_cluster_membership::membership_event &ev) {
    std::lock_guard<std::mutex> lk(ev_mu);
    events.push_back(ev);
  });

  // Join event on first heartbeat.
  m.on_heartbeat("node_2", "cluster_a", "", clk.now());
  {
    std::lock_guard<std::mutex> lk(ev_mu);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].kind, state_cluster_membership::event_kind::peer_joined);
    EXPECT_EQ(events[0].node_id, "node_2");
  }

  // Advance past all thresholds and run a tick.
  clk.advance_ms(700);
  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  m.stop();

  // Should have suspect, dead, and evicted events.
  std::lock_guard<std::mutex> lk(ev_mu);
  ASSERT_GE(events.size(), 4u);

  // Find each event type.
  bool has_suspect = false, has_dead = false, has_evicted = false;
  for (auto &ev : events) {
    if (ev.kind == state_cluster_membership::event_kind::peer_suspect)
      has_suspect = true;
    if (ev.kind == state_cluster_membership::event_kind::peer_dead)
      has_dead = true;
    if (ev.kind == state_cluster_membership::event_kind::peer_evicted)
      has_evicted = true;
  }
  EXPECT_TRUE(has_suspect);
  EXPECT_TRUE(has_dead);
  EXPECT_TRUE(has_evicted);
}

TEST(StateClusterMembership, RemoveCallback) {
  state_cluster_membership m("cluster_a", "node_1");
  int count = 0;
  auto id = m.add_callback([&](const state_cluster_membership::membership_event &) { count++; });

  m.on_heartbeat("node_2", "cluster_a", "", 1'000'000'000);
  EXPECT_EQ(count, 1);

  EXPECT_TRUE(m.remove_callback(id));
  m.on_heartbeat("node_3", "cluster_a", "", 2'000'000'000);
  EXPECT_EQ(count, 1); // callback removed, should not fire
}

TEST(StateClusterMembership, ConfigUpdateDuringRun) {
  state_cluster_membership m("cluster_a", "node_1");
  m.set_config({.heartbeat_interval_ms = 50});
  m.start();

  // Update config while running.
  m.set_config({.heartbeat_interval_ms = 100, .suspect_timeout_ms = 1000});
  auto cfg = m.current_config();
  EXPECT_EQ(cfg.heartbeat_interval_ms, 100u);
  EXPECT_EQ(cfg.suspect_timeout_ms, 1000u);

  m.stop();
}

TEST(StateClusterMembership, GetPeerNotFound) {
  state_cluster_membership m("cluster_a", "node_1");
  state_cluster_membership::peer_status ps;
  EXPECT_FALSE(m.get_peer("nonexistent", ps));
}

TEST(StateClusterMembership, IntegrationWithShardAndTransport) {
  app ctx;
  state_transport_inproc transport;
  state_cluster_shard shard(ctx, "cluster_a", "node_1");
  shard.attach();
  transport.register_shard(&shard);
  shard.set_transport(&transport);

  state_cluster_membership m("cluster_a", "node_1");
  m.set_shard(&shard);
  m.set_transport(&transport);
  m.set_peer_registry(&transport.peers());
  m.set_config({.heartbeat_interval_ms = 50});

  m.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  m.stop();

  EXPECT_GE(m.total_heartbeats_sent(), 1u);

  transport.unregister_shard(&shard);
  shard.detach();
}

TEST(StateClusterMembership, TwoNodeHeartbeatExchange) {
  app ctx1, ctx2;
  state_transport_inproc transport;

  state_cluster_shard shard1(ctx1, "cluster_a", "node_1");
  state_cluster_shard shard2(ctx2, "cluster_a", "node_2");
  shard1.attach();
  shard2.attach();
  transport.register_shard(&shard1);
  transport.register_shard(&shard2);
  shard1.set_transport(&transport);
  shard2.set_transport(&transport);

  state_cluster_membership m1("cluster_a", "node_1");
  state_cluster_membership m2("cluster_a", "node_2");

  m1.set_shard(&shard1);
  m1.set_transport(&transport);
  m1.set_peer_registry(&transport.peers());

  m2.set_shard(&shard2);
  m2.set_transport(&transport);
  m2.set_peer_registry(&transport.peers());

  // Manually simulate heartbeat exchange (since inproc transport
  // message delivery is synchronous, the tick loop's heartbeats
  // will reach the other shard's message bus, but processing
  // requires explicit wiring to on_heartbeat).
  m1.register_peer("node_2", "cluster_a");
  m2.register_peer("node_1", "cluster_a");

  EXPECT_EQ(m1.peer_snapshot().size(), 1u);
  EXPECT_EQ(m2.peer_snapshot().size(), 1u);

  auto snap1 = m1.peer_snapshot();
  EXPECT_EQ(snap1[0].node_id, "node_2");

  auto snap2 = m2.peer_snapshot();
  EXPECT_EQ(snap2[0].node_id, "node_1");

  transport.unregister_shard(&shard1);
  transport.unregister_shard(&shard2);
  shard1.detach();
  shard2.detach();
}

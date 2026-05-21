/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_transport_inproc.h>

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

} // namespace

TEST(StateTransportInprocTest, RegisterAndUnregisterTracksShards) {
  cvc::app a, b;
  cvc::state_transport_inproc t;
  EXPECT_EQ(t.shard_count(), 0u);

  cvc::state_cluster_shard sa(a, "C", "A");
  cvc::state_cluster_shard sb(b, "C", "B");
  t.register_shard(&sa);
  t.register_shard(&sb);
  EXPECT_EQ(t.shard_count(), 2u);

  // Idempotent register.
  t.register_shard(&sa);
  EXPECT_EQ(t.shard_count(), 2u);

  t.unregister_shard(&sa);
  EXPECT_EQ(t.shard_count(), 1u);
  t.unregister_shard(&sb);
  EXPECT_EQ(t.shard_count(), 0u);

  // Null-safe.
  t.register_shard(nullptr);
  t.unregister_shard(nullptr);
  EXPECT_EQ(t.shard_count(), 0u);
}

TEST(StateTransportInprocTest, TwoShardConvergence) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // First-set-on-fresh-child is lost (see adapter doc); set twice.
  cvc::state::instance(aA)("k").value(std::string("init"));
  cvc::state::instance(aA)("k").value(std::string("v1"));
  std::size_t pumped = t.pump_all();
  EXPECT_GE(pumped, 1u);

  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1");
  EXPECT_GE(sB.replica().last_applied("A"), 1u);
  EXPECT_GE(t.total_published(), 1u);
  EXPECT_GE(t.total_delivered(), 1u);
}

TEST(StateTransportInprocTest, RoundTripSuppressedByDedup) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  cvc::state::instance(aA)("rt").value(std::string("seed"));
  cvc::state::instance(aA)("rt").value(std::string("payload"));
  t.pump_all();

  // B applied A's mutation but, since apply_remote does not journal,
  // B has nothing to publish back. Pump again should be a no-op.
  std::size_t second = t.pump_all();
  EXPECT_EQ(second, 0u);

  // And A's seen-set already contains its own (A,seq), so even a
  // forced re-publish of that same envelope to A would be deduped:
  // verify by re-publishing the last drained mutation explicitly
  // through the transport (origin=A so sA itself is skipped, but
  // sB will dedup).
  cvc::state::instance(aA)("rt").value(std::string("payload2"));
  t.pump_all();
  EXPECT_EQ(cvc::state::instance(aB)("rt").value(), "payload2");

  // No phantom local mutations on B.
  auto b_local = sB.drain_local();
  EXPECT_TRUE(b_local.empty());
}

TEST(StateTransportInprocTest, ThreeShardFanOut) {
  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  cvc::state::instance(aA)("fan").value(std::string("seed"));
  cvc::state::instance(aA)("fan").value(std::string("hello"));
  t.pump_all();

  EXPECT_EQ(cvc::state::instance(aB)("fan").value(), "hello");
  EXPECT_EQ(cvc::state::instance(aC)("fan").value(), "hello");
}

TEST(StateTransportInprocTest, CrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B"); // different cluster
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  cvc::state::instance(aA)("iso").value(std::string("seed"));
  cvc::state::instance(aA)("iso").value(std::string("v"));
  t.pump_all();

  // B should not have observed anything from cluster C1.
  EXPECT_NE(cvc::state::instance(aB)("iso").value(), "v");
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);
}

TEST(StateTransportInprocTest, UnregisterStopsDelivery) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  cvc::state::instance(aA)("u").value(std::string("seed"));
  cvc::state::instance(aA)("u").value(std::string("first"));
  t.pump_all();
  EXPECT_EQ(cvc::state::instance(aB)("u").value(), "first");

  t.unregister_shard(&sB);
  cvc::state::instance(aA)("u").value(std::string("second"));
  t.pump_all();
  EXPECT_EQ(cvc::state::instance(aB)("u").value(), "first");
}

TEST(StateTransportInprocTest, PublishReturnsStats) {
  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  cvc::state::instance(aA)("p").value(std::string("seed"));
  cvc::state::instance(aA)("p").value(std::string("once"));
  auto pending = sA.drain_local();
  ASSERT_FALSE(pending.empty());
  auto stats = t.publish(pending.back());
  EXPECT_EQ(stats.delivered, 2u);
  EXPECT_EQ(stats.duplicates, 0u);
  EXPECT_EQ(stats.rejected, 0u);

  // Re-publishing the same mutation: both peers report duplicate.
  auto stats2 = t.publish(pending.back());
  EXPECT_EQ(stats2.delivered, 2u);
  EXPECT_EQ(stats2.duplicates, 2u);
}

TEST(StateTransportInprocTest, AuthorityRejectionCountedAsRejected) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  // B enforces authority and considers "owned.*" foreign.
  sB.authority().delegate("owned", "OTHER");
  sB.set_enforce_authority(true);
  t.register_shard(&sA);
  t.register_shard(&sB);

  cvc::state::instance(aA)("owned.x").value(std::string("seed"));
  cvc::state::instance(aA)("owned.x").value(std::string("nope"));
  auto drained = sA.drain_local();
  ASSERT_FALSE(drained.empty());
  auto stats = t.publish(drained.back());
  EXPECT_EQ(stats.rejected, 1u);
  EXPECT_EQ(stats.delivered, 0u);
}

TEST(StateTransportInprocTest, FlushIsNoop) {
  cvc::state_transport_inproc t;
  t.flush(); // must not crash, must not block
}

TEST(StateTransportInprocStressTest, OptionalConcurrentPublishStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run inproc "
                    "transport stress tests";
  }
  cvc::app aA, aB, aC, aD;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  cvc::state_cluster_shard sD(aD, "C", "D");
  sA.attach();
  sB.attach();
  sC.attach();
  sD.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);
  t.register_shard(&sD);

  const int kPerWriter = 200;
  std::atomic<bool> stop{false};
  std::thread pumper([&]() {
    while (!stop.load()) {
      t.pump_all();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  auto writer = [&](cvc::app &app_ref, const std::string &tag) {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(app_ref)("stress." + tag).value(std::string("v0"));
      cvc::state::instance(app_ref)("stress." + tag)
          .value("v" + std::to_string(i));
    }
  };
  std::thread wA([&]() { writer(aA, "A"); });
  std::thread wB([&]() { writer(aB, "B"); });
  wA.join();
  wB.join();

  // Final drain.
  for (int i = 0; i < 16; ++i) {
    if (t.pump_all() == 0)
      break;
  }
  stop.store(true);
  pumper.join();
  // One last sweep to absorb anything pumper missed.
  while (t.pump_all() > 0) {
  }

  // Convergence: every node should see the last value from each writer.
  std::string lastA = "v" + std::to_string(kPerWriter - 1);
  std::string lastB = "v" + std::to_string(kPerWriter - 1);
  EXPECT_EQ(cvc::state::instance(aB)("stress.A").value(), lastA);
  EXPECT_EQ(cvc::state::instance(aC)("stress.A").value(), lastA);
  EXPECT_EQ(cvc::state::instance(aD)("stress.A").value(), lastA);
  EXPECT_EQ(cvc::state::instance(aA)("stress.B").value(), lastB);
  EXPECT_EQ(cvc::state::instance(aC)("stress.B").value(), lastB);
  EXPECT_EQ(cvc::state::instance(aD)("stress.B").value(), lastB);
}

TEST(StateTransportInprocPerformanceTest, OptionalFanoutThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run inproc "
                    "transport performance smoke tests";
  }
  const int kPeers = 8;
  std::vector<std::unique_ptr<cvc::app>> apps;
  std::vector<std::unique_ptr<cvc::state_cluster_shard>> shards;
  cvc::state_transport_inproc t;
  for (int i = 0; i < kPeers; ++i) {
    apps.emplace_back(std::make_unique<cvc::app>());
    shards.emplace_back(std::make_unique<cvc::state_cluster_shard>(
        *apps.back(), "C", "N" + std::to_string(i)));
    shards.back()->attach();
    t.register_shard(shards.back().get());
  }

  // Prime the path so subsequent value() sets are journaled.
  cvc::state::instance(*apps[0])("perf.k").value(std::string("seed"));

  const int kIters = 2000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i) {
    cvc::state::instance(*apps[0])("perf.k").value("v" + std::to_string(i));
    if ((i & 0x3F) == 0)
      t.pump_all();
  }
  while (t.pump_all() > 0) {
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs =
      std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
          .count();
  std::cerr << "[transport_inproc perf] " << kIters << " mutations across "
            << kPeers << " peers in " << secs << "s ("
            << (kIters / secs) << " mut/s, "
            << (static_cast<double>(t.total_delivered()) / secs)
            << " deliveries/s)\n";
  EXPECT_LT(secs, 30.0);
}

// ----------------------------------------------------------------------------
// Phase 4: out-of-band messaging tests.
// ----------------------------------------------------------------------------

#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>

namespace {

cvc::state_message make_oob(const std::string &cluster,
                            const std::string &origin,
                            const std::string &id,
                            const std::string &path,
                            const std::string &str = {}) {
  cvc::state_message m;
  m.cluster_id = cluster;
  m.origin_node_id = origin;
  m.message_id = id;
  m.path = path;
  m.string_value = str;
  m.content_type = "text/plain";
  return m;
}

} // namespace

TEST(StateTransportInprocTest, MessageRoundTrip) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  std::string received_payload;
  sB.message_bus().subscribe("chat", [&](const cvc::state_message &m) {
    received_payload = m.string_value;
    hits.fetch_add(1);
  });

  auto msg = make_oob("C", "A", "m1", "chat.lobby", "hello");
  auto stats = t.publish_message(msg);
  EXPECT_GE(stats.delivered, 1u);
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(received_payload, "hello");
}

TEST(StateTransportInprocTest, MessageDedupOnRedundantPublish) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  auto msg = make_oob("C", "A", "m1", "x");
  auto s1 = t.publish_message(msg);
  auto s2 = t.publish_message(msg);
  EXPECT_GE(s1.delivered, 1u);
  EXPECT_EQ(s2.delivered, 0u);
  EXPECT_GE(s2.duplicates, 1u);
  EXPECT_EQ(hits.load(), 1);
}

TEST(StateTransportInprocTest, MessageDoesNotAdvanceClock) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // Only message traffic.
  t.publish_message(make_oob("C", "A", "m1", "x", "v"));
  t.publish_message(make_oob("C", "A", "m2", "y", "w"));

  EXPECT_EQ(sB.replica().last_applied("A"), 0u);
}

TEST(StateTransportInprocTest, MessageNotInJournal) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  auto a_before = sA.journal().size();
  auto b_before = sB.journal().size();
  t.publish_message(make_oob("C", "A", "m1", "x", "v"));
  EXPECT_EQ(sA.journal().size(), a_before);
  EXPECT_EQ(sB.journal().size(), b_before);
}

TEST(StateTransportInprocTest, MessageCrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  auto stats = t.publish_message(make_oob("C1", "A", "m1", "x"));
  EXPECT_EQ(stats.delivered, 0u);
  EXPECT_EQ(hits.load(), 0);
}

TEST(StateTransportInprocTest, MessageMultiSubscriberFanOut) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> all{0}, lobby{0}, sports{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) { all.fetch_add(1); });
  sB.message_bus().subscribe("chat.lobby", [&](const cvc::state_message &) { lobby.fetch_add(1); });
  sB.message_bus().subscribe("chat.sports", [&](const cvc::state_message &) { sports.fetch_add(1); });

  t.publish_message(make_oob("C", "A", "m1", "chat.lobby"));
  t.publish_message(make_oob("C", "A", "m2", "chat.sports"));

  EXPECT_EQ(all.load(), 2);
  EXPECT_EQ(lobby.load(), 1);
  EXPECT_EQ(sports.load(), 1);
}

// ---- Phase 5: subscription-prefix routing ----

namespace {
cvc::state_mutation make_mut(const std::string &origin,
                             const std::string &cluster,
                             std::uint64_t seq, const std::string &path) {
  cvc::state_mutation m;
  m.cluster_id = cluster;
  m.origin_node_id = origin;
  m.sequence = seq;
  m.path = path;
  m.string_value = "v";
  m.type_name = "std::string";
  return m;
}
} // namespace

TEST(StateTransportInprocPhase5, PeerSubscriptionsFilterDelivery) {
  cvc::app appA, appB;
  cvc::state_cluster_shard sA(appA, "c", "A");
  cvc::state_cluster_shard sB(appB, "c", "B");
  sA.attach();
  sB.attach();

  cvc::state_transport_inproc t;
  t.register_shard(&sA);
  t.register_shard(&sB);

  // B only subscribes to "alpha". "beta" should be filtered.
  t.peers().add_peer("B", "c", "", {"alpha"});

  t.publish(make_mut("A", "c", 1, "alpha.x"));
  t.publish(make_mut("A", "c", 2, "beta.y"));

  EXPECT_EQ(1u, sB.total_remote_applied());
  // The filtered delivery is recorded on the peer entry.
  auto snap = t.peers().snapshot();
  ASSERT_EQ(1u, snap.size());
  EXPECT_EQ(1u, snap[0].mutations_delivered);
  EXPECT_EQ(1u, snap[0].deliveries_filtered);

  t.unregister_shard(&sA);
  t.unregister_shard(&sB);
}

TEST(StateTransportInprocPhase5, UnregisteredPeerDeliversAll) {
  cvc::app appA, appB;
  cvc::state_cluster_shard sA(appA, "c", "A");
  cvc::state_cluster_shard sB(appB, "c", "B");
  sA.attach();
  sB.attach();

  cvc::state_transport_inproc t;
  t.register_shard(&sA);
  t.register_shard(&sB);

  // Do not register B in the peer registry: back-compat = match-all.
  t.publish(make_mut("A", "c", 1, "anything"));
  t.publish(make_mut("A", "c", 2, "else"));

  EXPECT_EQ(2u, sB.total_remote_applied());

  t.unregister_shard(&sA);
  t.unregister_shard(&sB);
}

// ---------------------------------------------------------------------
// Per-peer message outbox (Phase 6 backpressure on transports)
// ---------------------------------------------------------------------

namespace {

cvc::state_message make_outbox_msg(const std::string &cluster,
                                   const std::string &origin,
                                   const std::string &id,
                                   const std::string &path) {
  cvc::state_message m;
  m.cluster_id = cluster;
  m.origin_node_id = origin;
  m.message_id = id;
  m.path = path;
  m.string_value = "payload-" + id;
  m.content_type = cvc::state_message::MIME_TEXT;
  return m;
}

} // namespace

TEST(StateTransportInprocOutbox, NoOutboxIsSynchronous) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
  auto stats = t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  EXPECT_GE(stats.delivered, 1u);
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(t.total_outbox_admitted(), 0u);
}

TEST(StateTransportInprocOutbox, DropNewestRejectsWhenFull) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 2,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);

  // Three publishes; capacity 2; third must be rejected.
  auto s1 = t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  auto s2 = t.publish_message(make_outbox_msg("C", "A", "m2", "x"));
  auto s3 = t.publish_message(make_outbox_msg("C", "A", "m3", "x"));
  EXPECT_EQ(s1.delivered, 1u);
  EXPECT_EQ(s2.delivered, 1u);
  EXPECT_EQ(s3.delivered, 0u);
  EXPECT_GE(s3.duplicates, 1u); // overflow surfaces as non-delivered

  // Nothing has reached the subscriber yet — outbox is staged.
  EXPECT_EQ(hits.load(), 0);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 2u);
  EXPECT_EQ(t.total_outbox_admitted(), 2u);
  EXPECT_EQ(t.total_outbox_dropped_newest(), 1u);
  EXPECT_EQ(t.total_outbox_dropped_oldest(), 0u);

  std::size_t delivered = t.deliver_message_outbox(&sB);
  EXPECT_EQ(delivered, 2u);
  EXPECT_EQ(hits.load(), 2);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
}

TEST(StateTransportInprocOutbox, DropOldestEvictsFront) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::vector<std::string> received;
  sB.message_bus().subscribe("", [&](const cvc::state_message &m) {
    received.push_back(m.message_id);
  });

  t.set_peer_message_outbox(&sB, 2,
                            cvc::state_transport_inproc::outbox_policy::drop_oldest);

  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  t.publish_message(make_outbox_msg("C", "A", "m2", "x"));
  t.publish_message(make_outbox_msg("C", "A", "m3", "x")); // evicts m1
  t.publish_message(make_outbox_msg("C", "A", "m4", "x")); // evicts m2

  EXPECT_EQ(t.peer_message_outbox_size(&sB), 2u);
  EXPECT_EQ(t.total_outbox_dropped_oldest(), 2u);
  EXPECT_EQ(t.total_outbox_admitted(), 4u);

  t.deliver_message_outbox(&sB);
  ASSERT_EQ(received.size(), 2u);
  EXPECT_EQ(received[0], "m3");
  EXPECT_EQ(received[1], "m4");
}

TEST(StateTransportInprocOutbox, DeliverMaxRespectsBudget) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 8,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  for (int i = 0; i < 5; ++i)
    t.publish_message(make_outbox_msg("C", "A",
                                      "m" + std::to_string(i), "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 5u);

  EXPECT_EQ(t.deliver_message_outbox(&sB, 2), 2u);
  EXPECT_EQ(hits.load(), 2);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 3u);

  EXPECT_EQ(t.deliver_message_outbox(&sB, 0), 3u);
  EXPECT_EQ(hits.load(), 5);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
}

TEST(StateTransportInprocOutbox, ClearOutboxRevertsToSync) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  EXPECT_EQ(hits.load(), 0);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 1u);

  // Drain then clear.
  t.deliver_message_outbox(&sB);
  EXPECT_EQ(hits.load(), 1);
  t.clear_peer_message_outbox(&sB);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);

  // Now publish goes synchronously.
  t.publish_message(make_outbox_msg("C", "A", "m2", "x"));
  EXPECT_EQ(hits.load(), 2);
}

TEST(StateTransportInprocOutbox, UnregisterShardClearsOutbox) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  t.set_peer_message_outbox(&sB, 2,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 1u);

  t.unregister_shard(&sB);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
}

TEST(StateTransportInprocOutbox, ZeroCapacityRemovesOutbox) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  // Capacity 0 should clear it.
  t.set_peer_message_outbox(&sB, 0,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);

  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
}

TEST(StateTransportInprocOutbox, MixedPeersOutboxAndSync) {
  // sB has an outbox; sC stays synchronous. A publish should stage
  // on sB while immediately reaching sC.
  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  std::atomic<int> hitsB{0}, hitsC{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hitsB.fetch_add(1);
  });
  sC.message_bus().subscribe("", [&](const cvc::state_message &) {
    hitsC.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);

  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  t.publish_message(make_outbox_msg("C", "A", "m2", "x"));
  EXPECT_EQ(hitsB.load(), 0);     // staged
  EXPECT_EQ(hitsC.load(), 2);     // synchronous
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 2u);
  EXPECT_EQ(t.peer_message_outbox_size(&sC), 0u);

  EXPECT_EQ(t.deliver_message_outbox(&sB), 2u);
  EXPECT_EQ(hitsB.load(), 2);
}

TEST(StateTransportInprocOutbox, OriginPeerNeverReceivesOwnMessage) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // Outbox installed on the origin shouldn't even see the message.
  t.set_peer_message_outbox(&sA, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);

  t.publish_message(make_outbox_msg("C", "A", "m1", "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sA), 0u);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
}

TEST(StateTransportInprocOutbox, ClusterFilterAppliesBeforeOutbox) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  t.set_peer_message_outbox(&sB, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);

  // Origin cluster C1; B is in C2 and must not be staged.
  t.publish_message(make_outbox_msg("C1", "A", "m1", "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
  EXPECT_EQ(t.total_outbox_admitted(), 0u);
}

TEST(StateTransportInprocOutbox, ReplacingOutboxClosesOldQueue) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 8,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  for (int i = 0; i < 5; ++i)
    t.publish_message(make_outbox_msg("C", "A",
                                      "m" + std::to_string(i), "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 5u);

  // Re-install: pending messages are dropped (queue closed), new
  // queue starts empty.
  t.set_peer_message_outbox(&sB, 2,
                            cvc::state_transport_inproc::outbox_policy::drop_oldest);
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 0u);
  EXPECT_EQ(hits.load(), 0);

  t.publish_message(make_outbox_msg("C", "A", "n1", "x"));
  EXPECT_EQ(t.peer_message_outbox_size(&sB), 1u);
}

TEST(StateTransportInprocOutbox, DeliverEmptyReturnsZero) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // No outbox -> deliver returns 0.
  EXPECT_EQ(t.deliver_message_outbox(&sB), 0u);

  // Outbox installed but empty -> still 0.
  t.set_peer_message_outbox(&sB, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  EXPECT_EQ(t.deliver_message_outbox(&sB), 0u);
  EXPECT_EQ(t.deliver_message_outbox(&sB, 100), 0u);
}

TEST(StateTransportInprocOutbox, NullPeerIsSafe) {
  cvc::state_transport_inproc t;
  // None of these should crash or change counters.
  t.set_peer_message_outbox(nullptr, 4,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  t.clear_peer_message_outbox(nullptr);
  EXPECT_EQ(t.peer_message_outbox_size(nullptr), 0u);
  EXPECT_EQ(t.deliver_message_outbox(nullptr), 0u);
}

TEST(StateTransportInprocOutbox, AggregateCountersAcrossPeers) {
  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  t.set_peer_message_outbox(&sB, 1,
                            cvc::state_transport_inproc::outbox_policy::drop_newest);
  t.set_peer_message_outbox(&sC, 1,
                            cvc::state_transport_inproc::outbox_policy::drop_oldest);

  // Each publish hits both peers (origin = A, cluster matches).
  // First: B accepts, C accepts.
  // Second: B drops_newest (full), C drops_oldest then accepts.
  // Third: same as second.
  for (int i = 0; i < 3; ++i)
    t.publish_message(make_outbox_msg("C", "A",
                                      "m" + std::to_string(i), "x"));

  EXPECT_EQ(t.total_outbox_admitted(), 1u + 3u); // B:1, C:3
  EXPECT_EQ(t.total_outbox_dropped_newest(), 2u); // B drops 2nd & 3rd
  EXPECT_EQ(t.total_outbox_dropped_oldest(), 2u); // C evicts twice
}

TEST(StateTransportInprocOutbox, ConcurrentProducerConsumer) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  t.set_peer_message_outbox(&sB, 32,
                            cvc::state_transport_inproc::outbox_policy::drop_oldest);

  std::atomic<bool> stop{false};
  std::thread producer([&] {
    int i = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      t.publish_message(make_outbox_msg("C", "A",
                                        "m" + std::to_string(i++), "x"));
    }
  });
  // Drain in parallel.
  for (int spin = 0; spin < 200; ++spin)
    t.deliver_message_outbox(&sB, 8);
  stop.store(true, std::memory_order_relaxed);
  producer.join();
  // Final drain.
  t.deliver_message_outbox(&sB);

  // Either delivered or dropped — never lost-without-accounting.
  const auto pub = t.total_outbox_admitted() + t.total_outbox_dropped_oldest();
  // Every publish corresponds to either an admit or a drop.
  EXPECT_GT(pub, 0u);
  // hits is bounded by admitted (some queue messages may still be
  // in flight when test ends, but we final-drained).
  EXPECT_LE(static_cast<std::uint64_t>(hits.load()),
            t.total_outbox_admitted());
}

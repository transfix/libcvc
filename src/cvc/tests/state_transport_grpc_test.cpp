/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_transport_grpc.h>

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

bool wait_connected(cvc::state_transport_grpc &a,
                    cvc::state_transport_grpc &b,
                    std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (a.connection_count() >= 1 && b.connection_count() >= 1)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

TEST(StateTransportGrpcTest, StartStopBindsEphemeralPort) {
  cvc::state_transport_grpc t;
  ASSERT_NO_THROW(t.start("127.0.0.1:0", "A", "C"));
  EXPECT_FALSE(t.listen_address().empty());
  EXPECT_NE(t.listen_address(), "127.0.0.1:0");
  t.stop();
}

TEST(StateTransportGrpcTest, ConnectFailsWhenPeerMissing) {
  cvc::state_transport_grpc t;
  t.start("127.0.0.1:0", "A", "C");
  // Connect to a port that should be unbound.
  EXPECT_FALSE(t.connect_to_peer("127.0.0.1:1",
                                 std::chrono::milliseconds(150)));
  t.stop();
}

TEST(StateTransportGrpcTest, TwoEndpointConvergence) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("k").value(std::string("init"));
  cvc::state::instance(aA)("k").value(std::string("v1"));

  std::size_t pumped = tA.pump_all();
  EXPECT_GE(pumped, 1u);
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));

  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1");
  EXPECT_GE(sB.replica().last_applied("A"), 1u);

  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, RoundTripSuppressedByDedup) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("rt").value(std::string("seed"));
  cvc::state::instance(aA)("rt").value(std::string("payload"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));

  std::size_t b_pumped = tB.pump_all();
  EXPECT_EQ(b_pumped, 0u);

  cvc::state::instance(aA)("rt").value(std::string("payload2"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(2, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("rt").value(), "payload2");
  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, CrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C1");
  tB.start("127.0.0.1:0", "B", "C2");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("iso").value(std::string("seed"));
  cvc::state::instance(aA)("iso").value(std::string("v"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(500));

  EXPECT_NE(cvc::state::instance(aB)("iso").value(), "v");
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);
  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, ThreeEndpointFanOut) {
  cvc::app aA, aB, aC;
  cvc::state_transport_grpc tA, tB, tC;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tA.connect_to_peer(tC.listen_address(),
                                 std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  cvc::state::instance(aA)("fan").value(std::string("seed"));
  cvc::state::instance(aA)("fan").value(std::string("hello"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  tC.wait_for_received(1, std::chrono::milliseconds(2000));

  EXPECT_EQ(cvc::state::instance(aB)("fan").value(), "hello");
  EXPECT_EQ(cvc::state::instance(aC)("fan").value(), "hello");
  tA.stop();
  tB.stop();
  tC.stop();
}

TEST(StateTransportGrpcTest, BlobPayloadRoundTrip) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state_mutation m;
  m.cluster_id = "C";
  m.tree_id = "default";
  m.origin_node_id = "A";
  m.sequence = 100;
  m.mutation_id = "A:100";
  m.path = "blob.path";
  m.op = cvc::state_mutation_op::set_value;
  m.type_name = "std::string";
  m.string_value = "blob-name";
  m.payload.kind = cvc::state_payload_kind::blob;
  m.payload.blob.digest = "deadbeefcafebabe";
  m.payload.blob.size_bytes = 4096;
  m.payload.blob.codec = "raw";
  m.latest_value_only = false;

  tA.publish(m);
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));

  EXPECT_GE(tB.total_received_frames(), 1u);
  EXPECT_EQ(sB.replica().last_applied("A"), 100u);
  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, DisconnectAfterStop) {
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  tA.stop();
  tB.stop();
  SUCCEED();
}

TEST(StateTransportGrpcStressTest, OptionalConcurrentPublishStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run grpc "
                    "transport stress tests";
  }
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  const int kPerWriter = 200;
  std::atomic<bool> stop{false};
  std::thread pumper([&]() {
    while (!stop.load()) {
      tA.pump_all();
      tB.pump_all();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  std::thread wA([&]() {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(aA)("stress.A").value(std::string("v0"));
      cvc::state::instance(aA)("stress.A")
          .value("v" + std::to_string(i));
    }
  });
  std::thread wB([&]() {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(aB)("stress.B").value(std::string("v0"));
      cvc::state::instance(aB)("stress.B")
          .value("v" + std::to_string(i));
    }
  });
  wA.join();
  wB.join();

  for (int i = 0; i < 32; ++i) {
    if (tA.pump_all() == 0 && tB.pump_all() == 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  stop.store(true);
  pumper.join();
  while (tA.pump_all() > 0 || tB.pump_all() > 0) {
  }
  // Wait for receive convergence: poll the observable state until it
  // matches the final writers' values, with a generous timeout.
  std::string lastA = "v" + std::to_string(kPerWriter - 1);
  std::string lastB = "v" + std::to_string(kPerWriter - 1);
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cvc::state::instance(aB)("stress.A").value() == lastA &&
        cvc::state::instance(aA)("stress.B").value() == lastB)
      break;
    tA.pump_all();
    tB.pump_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(cvc::state::instance(aB)("stress.A").value(), lastA);
  EXPECT_EQ(cvc::state::instance(aA)("stress.B").value(), lastB);
  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcPerformanceTest, OptionalRoundTripThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run grpc "
                    "transport perf tests";
  }
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("perf.k").value(std::string("seed"));

  const int kIters = 2000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i) {
    cvc::state::instance(aA)("perf.k").value("v" + std::to_string(i));
    if ((i & 0x3F) == 0)
      tA.pump_all();
  }
  while (tA.pump_all() > 0) {
  }
  tA.flush();
  tB.wait_for_received(static_cast<std::uint64_t>(kIters / 64),
                       std::chrono::milliseconds(5000));
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(
                    elapsed)
                    .count();
  std::cerr << "[transport_grpc perf] " << kIters
            << " mutations across gRPC in " << secs << "s ("
            << (kIters / secs) << " mut/s, sent="
            << tA.total_sent_frames() << ", recv="
            << tB.total_received_frames() << ")\n";
  EXPECT_LT(secs, 30.0);
  tA.stop();
  tB.stop();
}

// ----------------------------------------------------------------------------
// Phase 4: out-of-band messaging tests.
// ----------------------------------------------------------------------------

#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>

namespace {

cvc::state_message make_oob_grpc(const std::string &cluster,
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

TEST(StateTransportGrpcTest, MessageRoundTripCrossPeer) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  std::string payload;
  sB.message_bus().subscribe("chat", [&](const cvc::state_message &m) {
    payload = m.string_value;
    hits.fetch_add(1);
  });

  tA.publish_message(make_oob_grpc("C", "A", "m1", "chat.lobby", "hi"));
  EXPECT_TRUE(tB.wait_for_received_messages(1, std::chrono::milliseconds(2000)));
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(payload, "hi");
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);

  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, MessageDedupAcrossRedundantPublish) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  auto m = make_oob_grpc("C", "A", "m1", "x", "v");
  tA.publish_message(m);
  tA.publish_message(m);
  tA.publish_message(m);

  EXPECT_TRUE(tB.wait_for_received_messages(1, std::chrono::milliseconds(2000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(hits.load(), 1);

  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, MessageCrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C1");
  tB.start("127.0.0.1:0", "B", "C2");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) {
    hits.fetch_add(1);
  });

  tA.publish_message(make_oob_grpc("C1", "A", "m1", "x", "v"));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(hits.load(), 0);

  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, MessageDoesNotAdvanceJournalOrClock) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(),
                                 std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  auto a_jb = sA.journal().size();
  auto b_jb = sB.journal().size();

  tA.publish_message(make_oob_grpc("C", "A", "m1", "x", "v"));
  tA.publish_message(make_oob_grpc("C", "A", "m2", "y", "w"));
  EXPECT_TRUE(tB.wait_for_received_messages(2, std::chrono::milliseconds(2000)));

  EXPECT_EQ(sA.journal().size(), a_jb);
  EXPECT_EQ(sB.journal().size(), b_jb);
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);

  tA.stop();
  tB.stop();
}

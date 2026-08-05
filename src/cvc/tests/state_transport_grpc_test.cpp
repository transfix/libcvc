/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_transport_grpc.h>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

bool wait_connected(cvc::state_transport_grpc &a, cvc::state_transport_grpc &b,
                    std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (a.connection_count() >= 1 && b.connection_count() >= 1)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// Spin until `pred` holds or timeout. Receive-side assertions need
// this rather than wait_for_received(): that counter is incremented
// by the reader thread when a frame is decoded, which is before
// on_inbound_mutation has ingested it, so it can be satisfied a
// moment before the mutation is visible in the tree or the replica.
template <typename Pred> bool wait_until(Pred pred, std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
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
  EXPECT_FALSE(t.connect_to_peer("127.0.0.1:1", std::chrono::milliseconds(150)));
  t.stop();
}

// A writes and pumps while nobody is connected. pump_shard() drains
// the journal and advances the publish cursor regardless of whether
// there was anywhere to send, so without backfill-on-connect those
// mutations are gone: a peer that connects afterwards would never
// see them, however long it pumps.
//
// B dials A here, so the backfill goes out from A's server-side
// stream handler.
TEST(StateTransportGrpcTest, LateJoinerReceivesWritesMadeBeforeItConnected) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("late.k").value(std::string("seed"));
  cvc::state::instance(aA)("late.k").value(std::string("v1"));

  // The drop: drained with zero connections, cursor advanced anyway.
  ASSERT_EQ(tA.connection_count(), 0u);
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  EXPECT_GT(sA.published_cursor(), 0u);

  // B joins only now, and A never writes again.
  ASSERT_TRUE(tB.connect_to_peer(tA.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("late.k").value() == "v1"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("late.k").value(), "v1");
  EXPECT_GT(tA.total_backfilled(), 0u);

  tA.stop();
  tB.stop();
}

// Same, but A dials B, so the backfill goes out from A's client-side
// connect_to_peer() path instead. gRPC establishes streams in two
// places and both have to replay.
TEST(StateTransportGrpcTest, LateJoinerBackfilledWhenLocalNodeDials) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("dial.k").value(std::string("seed"));
  cvc::state::instance(aA)("dial.k").value(std::string("v1"));
  ASSERT_EQ(tA.connection_count(), 0u);
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();

  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("dial.k").value() == "v1"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("dial.k").value(), "v1");
  EXPECT_GT(tA.total_backfilled(), 0u);

  tA.stop();
  tB.stop();
}

// Backfill must reach the peer ahead of any concurrently published
// mutation. The receiving replica's seen-set is exact membership,
// not a high-water mark, so if a live frame overtook the replay the
// older values would be applied last and clobber the newer one.
//
// B dials A, so A replays on its server handler thread while A's
// main thread keeps publishing — that concurrency is the whole
// point, and dialling from A instead would complete the backfill
// inside connect_to_peer() before any live write could race it.
// A writes strictly increasing v0..vN, so B's view must never move
// backwards.
TEST(StateTransportGrpcTest, BackfillPrecedesConcurrentLiveTraffic) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  auto suffix = [](const std::string &v) -> long {
    if (v.size() < 2 || v[0] != 'v')
      return -1;
    try {
      return std::stol(v.substr(1));
    } catch (...) {
      return -1;
    }
  };

  cvc::state::instance(aA)("ord.k").value(std::string("seed"));
  for (int i = 0; i < 64; ++i)
    cvc::state::instance(aA)("ord.k").value("v" + std::to_string(i));
  tA.pump_all(); // Dropped: no peers yet. Journal keeps all 64.
  tA.flush();

  // Watch B for a value that goes backwards while the race is on.
  std::atomic<bool> stop_watch{false};
  std::atomic<long> regressions{0};
  std::thread watcher([&]() {
    long seen = -1;
    while (!stop_watch.load()) {
      long cur = suffix(cvc::state::instance(aB)("ord.k").value());
      if (cur >= 0) {
        if (cur < seen)
          regressions.fetch_add(1);
        else
          seen = cur;
      }
    }
  });

  // Publish continuously; B connects into the middle of it.
  std::atomic<bool> stop_writer{false};
  std::atomic<int> last_written{63};
  std::thread writer([&]() {
    for (int i = 64; !stop_writer.load(); ++i) {
      cvc::state::instance(aA)("ord.k").value("v" + std::to_string(i));
      tA.pump_all();
      tA.flush();
      last_written.store(i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  ASSERT_TRUE(tB.connect_to_peer(tA.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  stop_writer.store(true);
  writer.join();

  std::string want = "v" + std::to_string(last_written.load());
  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("ord.k").value() == want; },
                         std::chrono::milliseconds(3000)));
  stop_watch.store(true);
  watcher.join();

  EXPECT_EQ(regressions.load(), 0)
      << "B's value moved backwards: backfill lost the race with live traffic";
  EXPECT_EQ(cvc::state::instance(aB)("ord.k").value(), want);
  EXPECT_GT(tA.total_backfilled(), 0u);

  tA.stop();
  tB.stop();
}

// The escape hatch: with backfill disabled the transport keeps its
// old behaviour, so the late joiner sees nothing.
TEST(StateTransportGrpcTest, BackfillCanBeDisabled) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.set_backfill_on_connect(false);
  EXPECT_FALSE(tA.backfill_on_connect());
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("nobf.k").value(std::string("seed"));
  cvc::state::instance(aA)("nobf.k").value(std::string("v1"));
  tA.pump_all();
  tA.flush();

  ASSERT_TRUE(tB.connect_to_peer(tA.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  EXPECT_EQ(tA.total_backfilled(), 0u);
  EXPECT_NE(cvc::state::instance(aB)("nobf.k").value(), "v1");

  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcTest, TwoEndpointConvergence) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tA.connect_to_peer(tC.listen_address(), std::chrono::milliseconds(2000)));

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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
      cvc::state::instance(aA)("stress.A").value("v" + std::to_string(i));
    }
  });
  std::thread wB([&]() {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(aB)("stress.B").value(std::string("v0"));
      cvc::state::instance(aB)("stress.B").value("v" + std::to_string(i));
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
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  tB.wait_for_received(static_cast<std::uint64_t>(kIters / 64), std::chrono::milliseconds(5000));
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  std::cerr << "[transport_grpc perf] " << kIters << " mutations across gRPC in " << secs << "s ("
            << (kIters / secs) << " mut/s, sent=" << tA.total_sent_frames()
            << ", recv=" << tB.total_received_frames() << ")\n";
  EXPECT_LT(secs, 30.0);
  tA.stop();
  tB.stop();
}

// ----------------------------------------------------------------------------
// Phase 4: out-of-band messaging tests.
// ----------------------------------------------------------------------------

#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>

namespace {

cvc::state_message make_oob_grpc(const std::string &cluster, const std::string &origin,
                                 const std::string &id, const std::string &path,
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) { hits.fetch_add(1); });

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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) { hits.fetch_add(1); });

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
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
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

// ---- Phase 5: bearer-token authentication ----

TEST(StateTransportGrpcPhase5, BearerTokenAcceptsValidPeer) {
  cvc::state_transport_grpc tA, tB;
  cvc::state_transport_grpc::auth_config auth;
  auth.expected_token = "secret-123";
  auth.outbound_token = "secret-123";
  tA.set_auth_config(auth);
  tB.set_auth_config(auth);

  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tB.connect_to_peer(tA.listen_address()));

  cvc::app aA, aB;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state_mutation m;
  m.cluster_id = "C";
  m.origin_node_id = "B";
  m.sequence = 1;
  m.path = "k";
  m.string_value = "v";
  m.type_name = "std::string";
  tB.publish(m);
  EXPECT_TRUE(tA.wait_for_received(1, std::chrono::milliseconds(2000)));
  tA.stop();
  tB.stop();
}

TEST(StateTransportGrpcPhase5, BearerTokenRejectsMissingMetadata) {
  cvc::state_transport_grpc tA, tB;
  cvc::state_transport_grpc::auth_config server_auth;
  server_auth.expected_token = "secret-456";
  tA.set_auth_config(server_auth);
  // tB has no outbound_token; server should reject.

  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  // Connect may succeed at the channel layer; the bidi RPC will be
  // rejected. We don't strictly need to assert connect_to_peer's
  // return — the receive count must stay at zero.
  (void)tB.connect_to_peer(tA.listen_address());

  cvc::app aA, aB;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state_mutation m;
  m.cluster_id = "C";
  m.origin_node_id = "B";
  m.sequence = 1;
  m.path = "k";
  m.string_value = "v";
  m.type_name = "std::string";
  tB.publish(m);

  // Give the server a brief chance to (not) deliver.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(0u, tA.total_received_mutations());

  tA.stop();
  tB.stop();
}

// ---- Phase 5: TLS handshake (opt-in) ----
//
// Generates a self-signed certificate at runtime via /usr/bin/openssl.
// Skipped unless CVC_DISTRIBUTED_STATE_TLS_TESTS=1 and openssl is
// available.

namespace {

bool tls_tests_enabled() {
  const char *e = std::getenv("CVC_DISTRIBUTED_STATE_TLS_TESTS");
  return e != nullptr && std::string(e) == "1";
}

bool generate_self_signed(std::string &cert_pem, std::string &key_pem) {
  char tmpl[] = "/tmp/cvc_tls_XXXXXX";
  if (!mkdtemp(tmpl))
    return false;
  std::string dir = tmpl;
  std::string cmd = "openssl req -x509 -newkey rsa:2048 -keyout " + dir + "/k.pem -out " + dir +
                    "/c.pem -days 1 -nodes -subj '/CN=localhost' "
                    "-addext 'subjectAltName=DNS:localhost,IP:127.0.0.1' "
                    ">/dev/null 2>&1";
  if (std::system(cmd.c_str()) != 0)
    return false;
  auto slurp = [](const std::string &p, std::string &out) -> bool {
    std::ifstream f(p);
    if (!f)
      return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return !out.empty();
  };
  bool ok = slurp(dir + "/c.pem", cert_pem) && slurp(dir + "/k.pem", key_pem);
  std::system(("rm -rf " + dir).c_str());
  return ok;
}

} // namespace

TEST(StateTransportGrpcPhase5, TlsHandshakeRoundTrip) {
  if (!tls_tests_enabled()) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_TLS_TESTS=1 to run TLS tests";
  }
  std::string cert, key;
  if (!generate_self_signed(cert, key)) {
    GTEST_SKIP() << "openssl not available; cannot generate self-signed cert";
  }

  cvc::state_transport_grpc tA, tB;
  cvc::state_transport_grpc::tls_config tls;
  tls.server_cert_pem = cert;
  tls.server_key_pem = key;
  tls.root_ca_pem = cert; // self-signed: cert == its own CA
  tls.require_client_auth = false;

  tA.set_tls_config(tls);
  tB.set_tls_config(tls);

  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  // gRPC TLS validates the server name against the certificate. Our
  // SAN includes localhost; gRPC parses host:port from the target.
  std::string target = tA.listen_address();
  // Replace 127.0.0.1 with localhost so SAN check matches.
  auto colon = target.find_last_of(':');
  std::string port = target.substr(colon + 1);
  ASSERT_TRUE(tB.connect_to_peer("localhost:" + port));

  cvc::app aA, aB;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state_mutation m;
  m.cluster_id = "C";
  m.origin_node_id = "B";
  m.sequence = 1;
  m.path = "k.tls";
  m.string_value = "v";
  m.type_name = "std::string";
  tB.publish(m);
  EXPECT_TRUE(tA.wait_for_received(1, std::chrono::milliseconds(3000)));

  tA.stop();
  tB.stop();
}

// ---------------------------------------------------------------
// Reconnect / resilience tests for the gRPC transport.
// ---------------------------------------------------------------

TEST(StateTransportGrpcReconnectTest, StopRestartReconnect) {
  cvc::app aA, aB;

  auto tA = std::make_unique<cvc::state_transport_grpc>();
  cvc::state_transport_grpc tB;
  tA->start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  auto addrB = tB.listen_address();
  ASSERT_TRUE(tA->connect_to_peer(addrB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(*tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA->register_shard(&sA);
  tB.register_shard(&sB);

  // Phase 1: replicate a value.
  cvc::state::instance(aA)("k").value(std::string("seed"));
  cvc::state::instance(aA)("k").value(std::string("v1"));
  tA->pump_all();
  tA->flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1");

  // Phase 2: stop A's transport (simulate restart).
  tA->unregister_shard(&sA);
  tA->stop();
  tA.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Phase 3: restart A, reconnect to B.
  tA = std::make_unique<cvc::state_transport_grpc>();
  tA->start("127.0.0.1:0", "A", "C");
  ASSERT_TRUE(tA->connect_to_peer(addrB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(*tA, tB, std::chrono::milliseconds(2000)));
  tA->register_shard(&sA);

  // Phase 4: replicate after reconnect.
  cvc::state::instance(aA)("k").value(std::string("v2_after_reconnect"));
  tA->pump_all();
  tA->flush();
  tB.wait_for_received(2, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v2_after_reconnect");

  tA->stop();
  tB.stop();
}

TEST(StateTransportGrpcReconnectTest, PeerDisconnectNoHang) {
  cvc::app aA, aB;

  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("x").value(std::string("seed"));
  cvc::state::instance(aA)("x").value(std::string("v1"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("x").value(), "v1");

  // B goes away.
  tB.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // A can still pump without crashing.
  cvc::state::instance(aA)("x").value(std::string("v2"));
  EXPECT_NO_THROW(tA.pump_all());
  EXPECT_NO_THROW(tA.flush());

  tA.stop();
}

TEST(StateTransportGrpcReconnectTest, ConnectionCountTracking) {
  cvc::state_transport_grpc tA, tB;
  EXPECT_EQ(tA.connection_count(), 0u);

  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  EXPECT_EQ(tA.connection_count(), 0u);

  ASSERT_TRUE(tA.connect_to_peer(tB.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  EXPECT_GE(tA.connection_count(), 1u);
  EXPECT_GE(tB.connection_count(), 1u);

  tB.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_NO_THROW(tA.connection_count());

  tA.stop();
}

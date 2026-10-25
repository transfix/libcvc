/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <atomic>
#include <boost/any.hpp>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/media/media_frame.h>
#include <cvc/media/media_router.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_peer_registry.h>
#include <cvc/state/state_transport.h>
#include <cvc/state/state_transport_grpc.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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

namespace {
struct GrpcSnapDesc {
  int w = 0, h = 0;
};
std::vector<unsigned char> enc_grpc_snap(const boost::any &v) {
  const GrpcSnapDesc d = boost::any_cast<GrpcSnapDesc>(v);
  std::vector<unsigned char> b(sizeof(GrpcSnapDesc));
  std::memcpy(b.data(), &d, sizeof(GrpcSnapDesc));
  return b;
}
boost::any dec_grpc_snap(const std::vector<unsigned char> &b) {
  GrpcSnapDesc d{};
  if (b.size() == sizeof(GrpcSnapDesc))
    std::memcpy(&d, b.data(), sizeof(GrpcSnapDesc));
  return boost::any(d);
}
} // namespace

// A node's data() (not just value() + metadata) now crosses in a gRPC snapshot:
// B requests a snapshot from A and the entry carries the codec-encoded data().
// (Closes the PR4-deferred gap where the gRPC SnapshotEntry dropped data().)
TEST(StateTransportGrpcTest, SnapshotCarriesData) {
  cvc::app aA, aB;
  aA.registerDataType<GrpcSnapDesc>("grpc.snap.desc");
  aB.registerDataType<GrpcSnapDesc>("grpc.snap.desc");

  cvc::state_transport_grpc tA, tB;
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.codecs().register_codec("grpc.snap.desc", enc_grpc_snap, dec_grpc_snap);
  sB.codecs().register_codec("grpc.snap.desc", enc_grpc_snap, dec_grpc_snap);
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  // A holds a node with value + data(); snapshot reads the tree directly, so a
  // single set suffices (no live-replication first-write quirk here).
  cvc::state::instance(aA)("snap.cam").value(std::string("live"));
  cvc::state::instance(aA)("snap.cam").data(boost::any(GrpcSnapDesc{1920, 1080}));

  ASSERT_TRUE(tB.connect_to_peer(tA.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  std::mutex mu;
  std::vector<cvc::state_transport::snapshot_entry> got;
  tB.request_snapshot("C", "",
                      [&](const std::vector<cvc::state_transport::snapshot_entry> &entries, bool) {
                        std::lock_guard<std::mutex> lk(mu);
                        for (const auto &e : entries)
                          got.push_back(e);
                      });

  ASSERT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        for (const auto &e : got)
          if (e.path == "snap.cam" && !e.data_type_name.empty())
            return true;
        return false;
      },
      std::chrono::milliseconds(3000)));

  std::lock_guard<std::mutex> lk(mu);
  const cvc::state_transport::snapshot_entry *cam = nullptr;
  for (const auto &e : got)
    if (e.path == "snap.cam")
      cam = &e;
  ASSERT_NE(cam, nullptr);
  EXPECT_EQ(cam->string_value, "live");
  EXPECT_EQ(cam->data_type_name, "grpc.snap.desc");
  ASSERT_EQ(cam->data_payload.kind, cvc::state_payload_kind::inline_bytes);
  const GrpcSnapDesc d = boost::any_cast<GrpcSnapDesc>(
      sB.codecs().decode("grpc.snap.desc", cam->data_payload.inline_bytes));
  EXPECT_EQ(d.w, 1920);
  EXPECT_EQ(d.h, 1080);

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

// A relay learns when a peer's stream ends, with the peer's node id (the WS transport's contract),
// so it can retract a departed native client from the room; its own shutdown reports nothing.
TEST(StateTransportGrpcTest, ConnectionClosedReportsThePeersNodeId) {
  cvc::state_transport_grpc relay, client;
  std::mutex mu;
  std::vector<std::string> closed;
  relay.set_on_connection_closed([&](const std::string &node_id) {
    std::lock_guard<std::mutex> lk(mu);
    closed.push_back(node_id);
  });
  relay.start("127.0.0.1:0", "relay", "C");
  client.start("127.0.0.1:0", "alice", "C");
  ASSERT_TRUE(client.connect_to_peer(relay.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(client, relay, std::chrono::milliseconds(2000)));
  // Leave only once the relay has READ alice's Hello: a cancelled stream drops frames the relay has
  // not read yet, and a stream that never identified itself is (by contract) not reported.
  const auto hello_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (;;) {
    const auto links = relay.link_reports();
    if (!links.empty() && links[0].remote_node_id == "alice")
      break;
    ASSERT_LT(std::chrono::steady_clock::now(), hello_deadline) << "the relay never saw alice";
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  client.stop(); // alice leaves
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(mu);
      if (!closed.empty())
        break;
    }
    ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "no connection-closed report";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  relay.stop();
  std::lock_guard<std::mutex> lk(mu);
  ASSERT_EQ(closed.size(), 1u) << "reported once, and not again on the relay's own stop()";
  EXPECT_EQ(closed[0], "alice");
}

// The link probe: both ends Ping, the other end Pongs, and each measures the round trip on its own
// clock. Each side reports one link with its peer's node id, traffic both ways, and an RTT -- the
// same report a relay gets for a client and a server gets for a peer (delegation) server.
TEST(StateTransportGrpcTest, LinkProbeMeasuresRoundTripAtBothEnds) {
  cvc::state_transport_grpc relay, client;
  relay.set_ping_interval(std::chrono::milliseconds(30));
  client.set_ping_interval(std::chrono::milliseconds(30));
  relay.start("127.0.0.1:0", "relay", "C");
  client.start("127.0.0.1:0", "alice", "C");
  ASSERT_TRUE(client.connect_to_peer(relay.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(client, relay, std::chrono::milliseconds(2000)));
  const auto measured = [](const cvc::state_transport_grpc &t) {
    const auto links = t.link_reports();
    return links.size() == 1 && links[0].stats.pongs_received >= 2 && links[0].stats.rtt_ms >= 0.0;
  };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!(measured(relay) && measured(client))) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "no round trip measured";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto server_end = relay.link_reports().at(0);
  const auto client_end = client.link_reports().at(0);
  EXPECT_EQ(server_end.transport, "grpc");
  EXPECT_TRUE(server_end.accepted);
  EXPECT_EQ(server_end.remote_node_id, "alice");
  EXPECT_FALSE(client_end.accepted);
  EXPECT_EQ(client_end.remote_node_id, "relay");
  for (const auto &r : {server_end, client_end}) {
    EXPECT_GT(r.stats.bytes_in, 0u);
    EXPECT_GT(r.stats.bytes_out, 0u);
    EXPECT_LT(r.stats.rtt_ms, 1000.0) << "loopback round trip";
  }
  client.stop();
  relay.stop();
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

#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>

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

// mkdtemp(3) is POSIX-only; create a unique temp dir portably so this file
// compiles on MSVC (the TLS tests stay gated off on Windows via the env check).
std::string make_temp_dir(const char *prefix) {
  std::error_code ec;
  std::filesystem::path base = std::filesystem::temp_directory_path(ec);
  if (ec)
    return {};
  std::random_device rd;
  for (int i = 0; i < 100; ++i) {
    std::filesystem::path cand = base / (std::string(prefix) + std::to_string(rd()));
    if (std::filesystem::create_directory(cand, ec) && !ec)
      return cand.string();
  }
  return {};
}

bool generate_self_signed(std::string &cert_pem, std::string &key_pem) {
  std::string dir = make_temp_dir("cvc_tls_");
  if (dir.empty())
    return false;
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

// Star topology: a RELAY server R that two clients A and B each dial, but A and B do NOT dial each
// other. With set_relay(true), a write on A reaches B THROUGH R — the foundation of a hosted,
// NAT-friendly conference (clients only ever connect to the server).
TEST(StateTransportGrpcTest, RelayForwardsBetweenStarClients) {
  cvc::app aA, aB;
  cvc::state_transport_grpc tR, tA, tB;
  tR.start("127.0.0.1:0", "R", "C"); // the relay has no shard — it only forwards
  tR.set_relay(true);
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  // A and B each dial ONLY the relay (never each other).
  ASSERT_TRUE(tA.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));

  // connect_to_peer returns once the CLIENT stream is open, which can be before the relay has
  // admitted the connection server-side. The relay forwards a frame only to the connections that
  // are present (alive + writable) WHEN THE FRAME ARRIVES, and nothing re-pumps A afterward, so if
  // A's single write/pump below raced ahead of B's admission at R the one forwarded batch would be
  // lost and B would never converge. Gate on R having admitted BOTH connections first: a server
  // connection binds its writer (can_write()) before it is counted, so connection_count() == 2
  // means both A and B are live forward targets. Bounded wait -> deterministic, not a fixed sleep.
  {
    auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (tR.connection_count() < 2 && std::chrono::steady_clock::now() < connect_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(tR.connection_count(), 2u)
        << "relay R should have admitted both A and B before A writes";
  }

  // A writes (set twice: first-set-on-fresh-child is not observed) and pumps -> A -> R -> B.
  cvc::state::instance(aA)("k").value(std::string("init"));
  cvc::state::instance(aA)("k").value(std::string("v1"));
  EXPECT_GT(tA.pump_all(), 0u);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (cvc::state::instance(aB)("k").value() != "v1" &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1")
      << "the relay should forward A's write to B even though they never connected directly";

  tA.stop();
  tB.stop();
  tR.stop();
  sA.detach();
  sB.detach();
}

// ----------------------------------------------------------------------------
// Per-peer delivery on a relay: connection identity, the relay interest gate, registry-filtered
// forwards and snapshots, targeted snapshot pushes, and the media egress gate.
// ----------------------------------------------------------------------------

namespace {

// Wait until `t` has a live link identified (its Hello read) as each of `ids`.
bool wait_identified(const cvc::state_transport_grpc &t, const std::vector<std::string> &ids,
                     std::chrono::milliseconds to) {
  return wait_until(
      [&] {
        const auto links = t.link_reports();
        return std::all_of(ids.begin(), ids.end(), [&](const std::string &id) {
          return std::any_of(links.begin(), links.end(), [&](const cvc::state_link_report &l) {
            return l.remote_node_id == id;
          });
        });
      },
      to);
}

// The value at `path`, or "<absent>" when the node does not exist (looked up without creating it).
std::string leaf(cvc::app &a, const std::string &path) {
  cvc::state *n = cvc::state::instance(a).findDescendant(path);
  return n ? n->value() : std::string("<absent>");
}

// Write `v` at a fresh `path` so that it is journaled: the first set on a new child is not
// observed, so it is set twice (as in RelayForwardsBetweenStarClients).
void write_fresh(cvc::app &a, const std::string &path, const std::string &v) {
  cvc::state::instance(a)(path).value(std::string("init"));
  cvc::state::instance(a)(path).value(v);
}

cvc::state_mutation value_mutation(const std::string &origin, std::uint64_t seq,
                                   const std::string &path, const std::string &value) {
  cvc::state_mutation m;
  m.cluster_id = "C";
  m.tree_id = "default";
  m.origin_node_id = origin;
  m.sequence = seq;
  m.mutation_id = origin + ":" + std::to_string(seq);
  m.path = path;
  m.op = cvc::state_mutation_op::set_value;
  m.type_name = "std::string";
  m.string_value = value;
  return m;
}

cvc::state_transport::snapshot_entry leaf_entry(const std::string &path, const std::string &value) {
  cvc::state_transport::snapshot_entry e;
  e.path = path;
  e.string_value = value;
  e.type_name = "std::string";
  return e;
}

// Denies one path prefix (and everything under it); delivers the rest.
class deny_prefix_filter final : public cvc::state_delivery_filter {
public:
  explicit deny_prefix_filter(std::string prefix) : _prefix(std::move(prefix)) {}
  bool deliver(const std::string &path) const noexcept override {
    return !cvc::state_peer_registry::prefix_matches(_prefix, path);
  }

private:
  std::string _prefix;
};

std::uint64_t filtered_count(const cvc::state_transport_grpc &t, const std::string &node_id) {
  for (const auto &p : t.peers().snapshot())
    if (p.node_id == node_id)
      return p.deliveries_filtered;
  return 0;
}

} // namespace

// A connection's peer is unknown until a Hello names it. A client whose Hello carries no node id
// stays unknown: its link reports no id and the registry cannot filter it, so it gets everything.
// A client whose Hello names it reports that name, and its registry entry filters publish() from
// then on.
TEST(StateTransportGrpcTest, PeerIdUnknownUntilHello) {
  cvc::app aN, aB;
  cvc::state_transport_grpc tR, tN, tB;
  tR.start("127.0.0.1:0", "R", "C");
  tN.start("127.0.0.1:0"); // no node id: its Hello names nobody
  tB.start("127.0.0.1:0", "B", "C");
  tR.peers().add_peer("B", "C", "", {"room"});

  cvc::state_cluster_shard sN(aN, "C", "N");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sN.attach();
  sB.attach();
  tN.register_shard(&sN);
  tB.register_shard(&sB);

  ASSERT_TRUE(tN.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_until([&] { return tR.connection_count() == 2; }, std::chrono::seconds(3)));
  ASSERT_TRUE(wait_identified(tR, {"B"}, std::chrono::seconds(3)));
  {
    const auto links = tR.link_reports();
    ASSERT_EQ(links.size(), 2u);
    EXPECT_EQ(
        std::count_if(links.begin(), links.end(),
                      [](const cvc::state_link_report &l) { return l.remote_node_id.empty(); }),
        1)
        << "the unnamed client's link reports no peer id";
  }

  // other.k is outside B's subscription; room.k (published after it) is inside.
  tR.publish(value_mutation("R", 1, "other.k", "o"));
  tR.publish(value_mutation("R", 2, "room.k", "r"));

  EXPECT_TRUE(wait_until([&] { return leaf(aB, "room.k") == "r"; }, std::chrono::seconds(3)));
  EXPECT_TRUE(wait_until([&] { return leaf(aN, "room.k") == "r" && leaf(aN, "other.k") == "o"; },
                         std::chrono::seconds(3)))
      << "an unknown peer gets everything";
  EXPECT_EQ(leaf(aB, "other.k"), "<absent>") << "B's subscription filters what precedes room.k";
  EXPECT_GE(filtered_count(tR, "B"), 1u);

  tN.stop();
  tB.stop();
  tR.stop();
}

// With the interest gate on, a relay whose shard enforces "conf" withholds a client's ui.docs.*
// write from the other clients (no shard of its own would take it) while conf.* still flows; with
// the gate off it forwards both, as before.
TEST(StateTransportGrpcTest, RelayInterestGateWithholdsOutOfInterestPaths) {
  cvc::app aR, aA, aB;
  cvc::state_transport_grpc tR, tA, tB;
  tR.start("127.0.0.1:0", "R", "C");
  tR.set_relay(true);
  tR.set_relay_requires_interest(true);
  EXPECT_TRUE(tR.relay_requires_interest());
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sR(aR, "C", "R");
  sR.add_interest("conf");
  sR.set_enforce_interest(true);
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sR.attach();
  sA.attach();
  sB.attach();
  tR.register_shard(&sR);
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tA.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_until([&] { return tR.connection_count() == 2; }, std::chrono::seconds(3)));

  // ui.docs.x goes first on A's stream, so once conf.rooms.r.y reaches B the relay has already
  // decided ui.docs.x.
  write_fresh(aA, "ui.docs.x", "u1");
  write_fresh(aA, "conf.rooms.r.y", "c1");
  EXPECT_GT(tA.pump_all(), 0u);
  EXPECT_TRUE(
      wait_until([&] { return leaf(aB, "conf.rooms.r.y") == "c1"; }, std::chrono::seconds(3)));
  EXPECT_EQ(leaf(aB, "ui.docs.x"), "<absent>") << "the relay withholds an out-of-interest write";
  EXPECT_EQ(leaf(aR, "conf.rooms.r.y"), "c1") << "ingest is unchanged";
  const std::uint64_t withheld = tR.total_relay_withheld();
  EXPECT_GE(withheld, 1u);

  tR.set_relay_requires_interest(false);
  cvc::state::instance(aA)("ui.docs.x").value(std::string("u2"));
  cvc::state::instance(aA)("conf.rooms.r.y").value(std::string("c2"));
  EXPECT_GT(tA.pump_all(), 0u);
  EXPECT_TRUE(wait_until(
      [&] { return leaf(aB, "ui.docs.x") == "u2" && leaf(aB, "conf.rooms.r.y") == "c2"; },
      std::chrono::seconds(3)))
      << "with the gate off the relay forwards everything";
  EXPECT_EQ(tR.total_relay_withheld(), withheld);

  tA.stop();
  tB.stop();
  tR.stop();
}

// The gate keeps the ingest exemption for delegation ops: a delegation over an out-of-interest
// subtree still reaches the other client, while a plain write under it is withheld.
TEST(StateTransportGrpcTest, RelayInterestGateForwardsDelegationOps) {
  cvc::app aR, aA, aB;
  cvc::state_transport_grpc tR, tA, tB;
  tR.start("127.0.0.1:0", "R", "C");
  tR.set_relay(true);
  tR.set_relay_requires_interest(true);
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sR(aR, "C", "R");
  sR.add_interest("conf");
  sR.set_enforce_interest(true);
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sR.attach();
  sA.attach();
  sB.attach();
  tR.register_shard(&sR);
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tA.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_until([&] { return tR.connection_count() == 2; }, std::chrono::seconds(3)));

  // A plain write under ui.docs is withheld (conf.rooms.r.y, written after it, marks the point by
  // which the relay has decided it)...
  write_fresh(aA, "ui.docs.x", "u1");
  write_fresh(aA, "conf.rooms.r.y", "c1");
  EXPECT_GT(tA.pump_all(), 0u);
  EXPECT_TRUE(
      wait_until([&] { return leaf(aB, "conf.rooms.r.y") == "c1"; }, std::chrono::seconds(3)));
  EXPECT_EQ(leaf(aB, "ui.docs.x"), "<absent>");
  const std::uint64_t withheld = tR.total_relay_withheld();
  EXPECT_GE(withheld, 1u);

  // ...but a delegation of that subtree is forwarded.
  sA.publish_delegation("ui.docs", "C2");
  cvc::state::instance(aA)("conf.rooms.r.y").value(std::string("c2"));
  EXPECT_GT(tA.pump_all(), 0u);
  EXPECT_TRUE(
      wait_until([&] { return leaf(aB, "conf.rooms.r.y") == "c2"; }, std::chrono::seconds(3)));
  EXPECT_EQ(sB.total_delegations_applied(), 1u) << "the delegation op is forwarded";
  EXPECT_EQ(tR.total_relay_withheld(), withheld);

  tA.stop();
  tB.stop();
  tR.stop();
}

// A relay forward consults each target's registry entry: B's delivery filter denies x.secret, so
// B gets only x.pub, while C (no entry) gets both.
TEST(StateTransportGrpcTest, RelayForwardHonorsPeerFilter) {
  cvc::app aA, aB, aC;
  cvc::state_transport_grpc tR, tA, tB, tC;
  tR.start("127.0.0.1:0", "R", "C");
  tR.set_relay(true);
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  ASSERT_TRUE(tA.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"A", "B", "C"}, std::chrono::seconds(3)));
  tR.peers().add_peer("B", "C");
  ASSERT_TRUE(tR.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.secret")));

  write_fresh(aA, "x.secret", "s1");
  write_fresh(aA, "x.pub", "p1");
  EXPECT_GT(tA.pump_all(), 0u);

  EXPECT_TRUE(wait_until([&] { return leaf(aB, "x.pub") == "p1"; }, std::chrono::seconds(3)));
  EXPECT_TRUE(wait_until([&] { return leaf(aC, "x.pub") == "p1" && leaf(aC, "x.secret") == "s1"; },
                         std::chrono::seconds(3)));
  EXPECT_EQ(leaf(aB, "x.secret"), "<absent>") << "B's filter denies x.secret on the forward";
  EXPECT_GE(filtered_count(tR, "B"), 1u);

  tA.stop();
  tB.stop();
  tC.stop();
  tR.stop();
}

// The snapshot responder answers with what the requester's registry entry lets it see.
TEST(StateTransportGrpcTest, SnapshotRequestFilteredByRequester) {
  cvc::app aR;
  cvc::state_transport_grpc tR, tB, tC;
  tR.set_backfill_on_connect(false);
  tR.start("127.0.0.1:0", "R", "C");
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");

  cvc::state_cluster_shard sR(aR, "C", "R");
  sR.attach();
  tR.register_shard(&sR);
  // The snapshot reads the tree directly, so a single set suffices.
  cvc::state::instance(aR)("x.secret").value(std::string("s"));
  cvc::state::instance(aR)("x.pub").value(std::string("p"));

  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"B", "C"}, std::chrono::seconds(3)));
  tR.peers().add_peer("B", "C");
  ASSERT_TRUE(tR.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.secret")));

  const auto paths_from = [](cvc::state_transport_grpc &t) {
    std::mutex mu;
    std::vector<std::string> paths;
    const bool ok = t.request_snapshot(
        "C", "x", [&](const std::vector<cvc::state_transport::snapshot_entry> &es, bool) {
          std::lock_guard<std::mutex> lk(mu);
          for (const auto &e : es)
            paths.push_back(e.path);
        });
    EXPECT_TRUE(ok);
    std::lock_guard<std::mutex> lk(mu);
    return paths;
  };
  const auto has = [](const std::vector<std::string> &v, const char *p) {
    return std::find(v.begin(), v.end(), p) != v.end();
  };

  const auto gotB = paths_from(tB);
  EXPECT_TRUE(has(gotB, "x.pub"));
  EXPECT_FALSE(has(gotB, "x.secret")) << "B's filter applies to its snapshot";
  const auto gotC = paths_from(tC);
  EXPECT_TRUE(has(gotC, "x.pub"));
  EXPECT_TRUE(has(gotC, "x.secret")) << "an unfiltered requester gets everything";

  tB.stop();
  tC.stop();
  tR.stop();
}

// push_snapshot_to writes only to the connection identified as the named peer.
TEST(StateTransportGrpcTest, PushSnapshotToReachesOnlyNamedPeer) {
  cvc::app aB, aC;
  cvc::state_transport_grpc tR, tB, tC;
  tR.start("127.0.0.1:0", "R", "C");
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");

  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sB.attach();
  sC.attach();
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"B", "C"}, std::chrono::seconds(3)));

  const std::vector<cvc::state_transport::snapshot_entry> forB = {leaf_entry("pushed.k", "v")};
  EXPECT_EQ(tR.push_snapshot_to("B", forB), 1u);
  EXPECT_EQ(tR.push_snapshot_to("nobody", forB), 0u) << "an unknown peer gets nothing";
  EXPECT_EQ(tR.push_snapshot_to("B", {}), 0u) << "an empty push writes nothing";
  // C's own push, sent after B's, marks the point by which anything for C has landed.
  EXPECT_EQ(tR.push_snapshot_to("C", {leaf_entry("pushed.c", "c")}), 1u);

  EXPECT_TRUE(wait_until([&] { return leaf(aB, "pushed.k") == "v"; }, std::chrono::seconds(3)));
  EXPECT_TRUE(wait_until([&] { return leaf(aC, "pushed.c") == "c"; }, std::chrono::seconds(3)));
  EXPECT_EQ(leaf(aC, "pushed.k"), "<absent>");
  EXPECT_EQ(leaf(aB, "pushed.c"), "<absent>");

  tB.stop();
  tC.stop();
  tR.stop();
}

// A push and the publishes that follow it on the same thread keep their order on the wire: a
// snapshot entry bypasses the receiver's conflict gate, so a push overtaken by the newer live write
// would leave the old value behind.
TEST(StateTransportGrpcTest, PushThenPublishKeepsOrder) {
  cvc::app aB;
  cvc::state_transport_grpc tR, tB;
  tR.start("127.0.0.1:0", "R", "C");
  tB.start("127.0.0.1:0", "B", "C");

  cvc::state_cluster_shard sB(aB, "C", "B");
  sB.attach();
  tB.register_shard(&sB);

  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"B"}, std::chrono::seconds(3)));

  ASSERT_EQ(tR.push_snapshot_to("B", {leaf_entry("room.p", "v1")}), 1u);
  tR.publish(value_mutation("R", 1, "room.p", "v2"));
  tR.publish(value_mutation("R", 2, "room.done", "1"));

  EXPECT_TRUE(wait_until([&] { return leaf(aB, "room.done") == "1"; }, std::chrono::seconds(3)));
  EXPECT_EQ(leaf(aB, "room.p"), "v2");

  tB.stop();
  tR.stop();
}

// The media gate on the relay's fan-out: a receiver it denies is skipped while the others still get
// the frame, and a node's own uplink is never gated.
TEST(StateTransportGrpcTest, MediaGateOnGrpc) {
  cvc::state_transport_grpc tR, tA, tB, tC;
  tR.start("127.0.0.1:0", "R", "C");
  tR.set_media_stall_drop(std::chrono::milliseconds(40)); // healthy loopback writes never trip it
  // Declared after tR so it destructs first, clearing tR's media sink while tR is still alive.
  cvc::media_router mr;
  mr.add_endpoint(&tR);
  tA.start("127.0.0.1:0", "A", "C");
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");

  std::mutex mu;
  std::vector<std::int64_t> seqB, seqC;
  std::atomic<int> gotA{0};
  tA.set_on_media([&](const cvc::media_frame &) { gotA.fetch_add(1); });
  tB.set_on_media([&](const cvc::media_frame &f) {
    std::lock_guard<std::mutex> lk(mu);
    seqB.push_back(f.seq);
  });
  tC.set_on_media([&](const cvc::media_frame &f) {
    std::lock_guard<std::mutex> lk(mu);
    seqC.push_back(f.seq);
  });

  ASSERT_TRUE(tA.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"A", "B", "C"}, std::chrono::seconds(3)));

  tR.set_media_gate([](const cvc::media_frame &, const std::string &rx) { return rx != "B"; });
  // A gate that denies everything on A's own transport: its uplink must ignore it.
  tA.set_media_gate([](const cvc::media_frame &, const std::string &) { return false; });

  cvc::media_frame m;
  m.origin_node_id = "A";
  m.stream_id = "camera";
  m.codec = "mjpeg";
  m.width = 64;
  m.height = 48;
  m.seq = 1;
  m.payload = {0xff, 0xd8, 0x01, 0xff, 0xd9};
  tA.publish_media_frame(m);
  EXPECT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !seqC.empty();
      },
      std::chrono::seconds(3)));
  EXPECT_EQ(tR.total_media_gated(), 1u) << "B is the only receiver denied";

  // Clearing the gate restores full fan-out. B's stream is FIFO, so once seq 2 arrives a seq 1
  // sent to B would already be there.
  tR.set_media_gate({});
  m.seq = 2;
  tA.publish_media_frame(m);
  EXPECT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return seqB.size() == 1 && seqC.size() == 2;
      },
      std::chrono::seconds(3)));
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(seqB, std::vector<std::int64_t>{2});
  }
  EXPECT_EQ(tA.total_media_sent(), 2u) << "the uplink is never gated";
  EXPECT_EQ(tR.total_media_gated(), 1u);
  EXPECT_EQ(tR.total_media_stall_dropped(), 0u);
  EXPECT_EQ(gotA.load(), 0) << "the origin is still skipped";

  tA.stop();
  tB.stop();
  tC.stop();
  tR.stop();
}

TEST(StateTransportGrpcTest, MediaWriteStalled) {
  using T = cvc::state_transport_grpc;
  constexpr std::uint64_t kMs = 1'000'000;
  constexpr std::uint64_t now = 10'000 * kMs;
  EXPECT_FALSE(T::media_write_stalled(0, now, 40 * kMs)) << "no write in flight";
  EXPECT_FALSE(T::media_write_stalled(now - 39 * kMs, now, 40 * kMs));
  EXPECT_TRUE(T::media_write_stalled(now - 41 * kMs, now, 40 * kMs));
  EXPECT_FALSE(T::media_write_stalled(now - 41 * kMs, now, 0)) << "0 turns the drop off";
}

// A client shard that enforces interest takes from an unsolicited (relay admit) snapshot only what
// it would take live; a shard that does not enforce still takes everything.
TEST(StateTransportGrpcTest, ClientSnapshotIngestHonorsEnforcedInterest) {
  cvc::app aR, aB, aC;
  cvc::state_transport_grpc tR, tB, tC;
  tR.set_backfill_on_connect(false); // the admit snapshot alone carries the relay's state
  tR.start("127.0.0.1:0", "R", "C");
  tR.set_relay(true);
  tB.start("127.0.0.1:0", "B", "C");
  tC.start("127.0.0.1:0", "C", "C");

  cvc::state_cluster_shard sR(aR, "C", "R");
  sR.attach();
  tR.register_shard(&sR);
  cvc::state::instance(aR)("conf.x").value(std::string("c"));
  cvc::state::instance(aR)("ui.y").value(std::string("u"));

  cvc::state_cluster_shard sB(aB, "C", "B");
  sB.add_interest("conf");
  sB.set_enforce_interest(true);
  cvc::state_cluster_shard sC(aC, "C", "C");
  sB.attach();
  sC.attach();
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  ASSERT_TRUE(tB.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(tR.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tR, {"B", "C"}, std::chrono::seconds(3)));
  // The relay reads a Hello only after the admit snapshot is written, so a push now lands behind
  // it: once the push is applied, so is the admit snapshot.
  ASSERT_EQ(tR.push_snapshot_to("B", {leaf_entry("conf.done", "1")}), 1u);
  ASSERT_EQ(tR.push_snapshot_to("C", {leaf_entry("conf.done", "1")}), 1u);
  EXPECT_TRUE(
      wait_until([&] { return leaf(aB, "conf.done") == "1" && leaf(aC, "conf.done") == "1"; },
                 std::chrono::seconds(3)));

  EXPECT_EQ(leaf(aB, "conf.x"), "c");
  EXPECT_EQ(leaf(aB, "ui.y"), "<absent>") << "outside B's enforced interest";
  EXPECT_EQ(leaf(aC, "conf.x"), "c");
  EXPECT_EQ(leaf(aC, "ui.y"), "u");

  tB.stop();
  tC.stop();
  tR.stop();
}

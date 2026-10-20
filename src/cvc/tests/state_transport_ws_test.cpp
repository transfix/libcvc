/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Mirrors state_transport_grpc_test for the WebSocket backend: two transports over ws://127.0.0.1:0
// replicate a room tree, a late joiner converges via backfill, and a relay bridges two pure clients
// (the browser topology — clients that dial without a local acceptor, as a wasm peer will).

#include <atomic>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/media/media_router.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <cvc/state/state_transport_ws.h>
#if defined(CVC_ENABLE_GRPC)
#include <cvc/state/state_transport_grpc.h>
#endif
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

bool wait_connected(cvc::state_transport_ws &a, cvc::state_transport_ws &b,
                    std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (a.connection_count() >= 1 && b.connection_count() >= 1)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// Spin until `pred` holds or timeout. Receive-side assertions need this rather than
// wait_for_received(): that counter is bumped when a frame is decoded, a moment before
// on_inbound_mutation has ingested it into the tree/replica.
template <typename Pred> bool wait_until(Pred pred, std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

std::string ws(const std::string &hostport) { return "ws://" + hostport; }

} // namespace

TEST(StateTransportWsTest, StartStopBindsEphemeralPort) {
  cvc::state_transport_ws t;
  ASSERT_NO_THROW(t.start("127.0.0.1:0", "A", "C"));
  EXPECT_FALSE(t.listen_address().empty());
  EXPECT_NE(t.listen_address(), "127.0.0.1:0");
  t.stop();
}

TEST(StateTransportWsTest, ConnectFailsWhenPeerMissing) {
  cvc::state_transport_ws t;
  // A pure client needs no prior start(); dialing an unbound port must fail fast.
  EXPECT_FALSE(t.connect_to_peer(ws("127.0.0.1:1"), std::chrono::milliseconds(300)));
  t.stop();
}

// A server, B client. B dials A, then A writes — the mutation crosses live over the one WS stream.
TEST(StateTransportWsTest, LiveMutationCrossesServerToClient) {
  cvc::app aA, aB;
  cvc::state_transport_ws tA, tB;
  tA.start("127.0.0.1:0", "A", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tB.connect_to_peer(ws(tA.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state::instance(aA)("live.k").value(std::string("seed"));
  cvc::state::instance(aA)("live.k").value(std::string("v1"));
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("live.k").value() == "v1"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("live.k").value(), "v1");

  tA.stop();
  tB.stop();
}

// A writes and pumps with nobody connected (the cursor advances regardless). B joins afterward and
// must still converge — backfill-on-connect replays A's journaled local mutations to the new peer.
TEST(StateTransportWsTest, LateJoinerReceivesWritesMadeBeforeItConnected) {
  cvc::app aA, aB;
  cvc::state_transport_ws tA, tB;
  tA.start("127.0.0.1:0", "A", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("late.k").value(std::string("seed"));
  cvc::state::instance(aA)("late.k").value(std::string("v1"));

  ASSERT_EQ(tA.connection_count(), 0u);
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();

  // B dials A only now; the backfill goes out from A's server-side accept path. A never writes
  // again.
  ASSERT_TRUE(tB.connect_to_peer(ws(tA.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("late.k").value() == "v1"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("late.k").value(), "v1");
  EXPECT_GT(tA.total_backfilled(), 0u);

  tA.stop();
  tB.stop();
}

// The conference topology: a relay server S bridges two PURE clients (neither calls start(), as a
// browser peer cannot). A's mutation reaches B only because S re-sends it to every other
// connection.
TEST(StateTransportWsTest, RelayBridgesTwoClients) {
  cvc::app aA, aB;
  cvc::state_transport_ws S, tA, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true); // without this a leaf frame would die at the server

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  // Both clients must be on S before A writes, or the relayed frame has nowhere to land (S holds no
  // journal of its own to backfill B with).
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 2; }, std::chrono::milliseconds(2000)));

  cvc::state::instance(aA)("relay.k").value(std::string("seed"));
  cvc::state::instance(aA)("relay.k").value(std::string("hello"));
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("relay.k").value() == "hello"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("relay.k").value(), "hello");

  tA.stop();
  tB.stop();
  S.stop();
}

// Media plane: a client's MediaFrame reaches the OTHER client through the relay's fan-out (SFU),
// carrying its origin + codec + size + payload intact, and the publisher does NOT receive its own
// frame back (the relay skips the origin connection). Media never enters a shard.
TEST(StateTransportWsTest, RelayFansMediaFrameToOtherClient) {
  cvc::app aA, aB;
  cvc::state_transport_ws S, tA, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  // Media now fans through a media_router (set_relay governs the control plane only). Declared after
  // S so it destructs first, clearing S's media sink while S is still alive.
  cvc::media_router mr;
  mr.add_endpoint(&S);

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::mutex mu;
  std::vector<cvc::ws_media_frame> gotB;
  std::atomic<int> gotA{0};
  tB.set_on_media([&](const cvc::ws_media_frame &m) {
    std::lock_guard<std::mutex> lk(mu);
    gotB.push_back(m);
  });
  tA.set_on_media([&](const cvc::ws_media_frame &) { gotA.fetch_add(1); });

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 2; }, std::chrono::milliseconds(2000)));

  cvc::ws_media_frame m;
  m.origin_node_id = "A";
  m.stream_id = "camera";
  m.codec = "mjpeg";
  m.width = 640;
  m.height = 480;
  m.seq = 7;
  m.pts_seconds = 1.5;
  m.payload = {0xff, 0xd8, 0x01, 0x02, 0x03, 0xff, 0xd9}; // a tiny JPEG-ish blob
  tA.publish_media_frame(m);

  ASSERT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !gotB.empty();
      },
      std::chrono::milliseconds(3000)));
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(gotB.size(), 1u);
    EXPECT_EQ(gotB[0].origin_node_id, "A");
    EXPECT_EQ(gotB[0].stream_id, "camera");
    EXPECT_EQ(gotB[0].codec, "mjpeg");
    EXPECT_EQ(gotB[0].width, 640u);
    EXPECT_EQ(gotB[0].height, 480u);
    EXPECT_EQ(gotB[0].seq, 7);
    EXPECT_EQ(gotB[0].payload, m.payload);
  }
  // The publisher must not receive its own frame back.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(gotA.load(), 0);
  EXPECT_EQ(tB.total_media_received(), 1u);
  EXPECT_EQ(tA.total_media_sent(), 1u);

  tA.stop();
  tB.stop();
  S.stop();
}

// Ghost reaping: a client whose tab navigated/reloaded away stops streaming and its socket may not
// close cleanly. reap_idle shuts the silent connection down after the idle timeout, which fires the
// on-close retraction (so the roster drops the ghost) and then sweeps the connection.
TEST(StateTransportWsTest, RelayReapsIdleConnection) {
  cvc::app aA;
  cvc::state_transport_ws S, tA;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  std::mutex mu;
  std::vector<std::string> reaped;
  S.set_on_connection_closed([&](const std::string &nid) {
    std::lock_guard<std::mutex> lk(mu);
    reaped.push_back(nid);
  });
  cvc::state_cluster_shard sA(aA, "C", "A");
  sA.attach();
  tA.register_shard(&sA);
  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 1; }, std::chrono::milliseconds(2000)));

  // tA sends nothing further, so S's connection goes idle (only the Hello set its last-rx). Let it
  // age past the timeout, then reap: the stale connection is shut down and retracted ("A").
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  S.reap_idle(std::chrono::milliseconds(20));
  EXPECT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !reaped.empty();
      },
      std::chrono::milliseconds(2000)));
  // A second pass sweeps the finished reader (join + erase), so connections don't accumulate.
  EXPECT_TRUE(wait_until(
      [&] {
        S.reap_idle(std::chrono::milliseconds(20));
        return S.connection_count() == 0;
      },
      std::chrono::milliseconds(2000)));
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_FALSE(reaped.empty());
    EXPECT_EQ(reaped[0], "A"); // tA's Hello node id (adopted from its shard)
  }
  tA.stop();
  S.stop();
}

// Late-joiner bootstrap: a relay that holds the room tree in its own shard pushes a full snapshot to
// each newly admitted connection, so a client that joins AFTER others are present still sees them
// (live set_relay forwarding only carries writes made after the client connects). This is what lets
// a browser that refreshes / opens late see everyone already in the conference room.
TEST(StateTransportWsTest, RelayBackfillsLateJoinerWithExistingState) {
  cvc::app aS, aA, aB;
  cvc::state_transport_ws S, tA, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S"); // the relay aggregates the room tree here
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sA.attach();
  sB.attach();
  S.register_shard(&sS);
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  // A joins first and writes; its forwarded mutation lands in the relay's shard.
  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 1; }, std::chrono::milliseconds(2000)));
  cvc::state::instance(aA)("room.a.name").value(std::string("seed"));
  cvc::state::instance(aA)("room.a.name").value(std::string("alice"));
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  ASSERT_TRUE(wait_until([&] { return cvc::state::instance(aS)("room.a.name").value() == "alice"; },
                         std::chrono::milliseconds(3000)));

  // B joins LATE and never writes; A never writes again. B must still see A, which can only come
  // from the relay's snapshot push on connect.
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("room.a.name").value() == "alice"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aB)("room.a.name").value(), "alice");

  tA.stop();
  tB.stop();
  S.stop();
}

#if defined(CVC_ENABLE_GRPC)
// The WS<->gRPC bridge (the wasm-interop payoff): a relay runs a gRPC acceptor AND a WebSocket
// acceptor over the SAME shard, bridged both ways. A native WS client and a native gRPC client each
// reach the other — exactly what a browser (WS) client and a native (gRPC) client will do in one
// room, which is the whole point of carrying the identical Frame wire on both transports.
TEST(StateTransportWsTest, BridgeWsAndGrpcClientsShareARoom) {
  cvc::app aRelay, aWs, aGrpc;

  // Relay: both backends over one shared shard, each relaying, bridged to each other.
  cvc::state_transport_grpc G;
  cvc::state_transport_ws W;
  G.start("127.0.0.1:0", "relay", "C");
  W.start("127.0.0.1:0", "relay", "C");
  G.set_relay(true);
  W.set_relay(true);
  G.set_bridge(&W);
  W.set_bridge(&G);
  cvc::state_cluster_shard sR(aRelay, "C", "relay");
  sR.attach();
  G.register_shard(&sR);
  W.register_shard(&sR);

  // A WebSocket client (dials with no local acceptor — the browser topology).
  cvc::state_transport_ws wc;
  cvc::state_cluster_shard sWs(aWs, "C", "wsclient");
  sWs.attach();
  wc.register_shard(&sWs);
  ASSERT_TRUE(wc.connect_to_peer(ws(W.listen_address()), std::chrono::milliseconds(2000)));

  // A gRPC client.
  cvc::state_transport_grpc gc;
  gc.start("127.0.0.1:0", "grpcclient", "C");
  cvc::state_cluster_shard sGrpc(aGrpc, "C", "grpcclient");
  sGrpc.attach();
  gc.register_shard(&sGrpc);
  ASSERT_TRUE(gc.connect_to_peer(G.listen_address(), std::chrono::milliseconds(2000)));

  ASSERT_TRUE(wait_until([&] { return G.connection_count() >= 1 && W.connection_count() >= 1; },
                         std::chrono::milliseconds(2000)));

  // WS client writes -> reaches the gRPC client through the bridge.
  cvc::state::instance(aWs)("bridge.ws.k").value(std::string("seed"));
  cvc::state::instance(aWs)("bridge.ws.k").value(std::string("from-ws"));
  wc.pump_all();
  wc.flush();
  EXPECT_TRUE(
      wait_until([&] { return cvc::state::instance(aGrpc)("bridge.ws.k").value() == "from-ws"; },
                 std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aGrpc)("bridge.ws.k").value(), "from-ws");

  // gRPC client writes -> reaches the WS client through the bridge.
  cvc::state::instance(aGrpc)("bridge.grpc.k").value(std::string("seed"));
  cvc::state::instance(aGrpc)("bridge.grpc.k").value(std::string("from-grpc"));
  gc.pump_all();
  gc.flush();
  EXPECT_TRUE(
      wait_until([&] { return cvc::state::instance(aWs)("bridge.grpc.k").value() == "from-grpc"; },
                 std::chrono::milliseconds(3000)));
  EXPECT_EQ(cvc::state::instance(aWs)("bridge.grpc.k").value(), "from-grpc");

  wc.stop();
  gc.stop();
  W.stop();
  G.stop();
}
#endif // CVC_ENABLE_GRPC

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

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <cvc/state/state_transport_ws.h>
#if defined(CVC_ENABLE_GRPC)
#include <cvc/state/state_transport_grpc.h>
#endif
#include <gtest/gtest.h>
#include <string>
#include <thread>

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

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
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
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
#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <map>
#include <memory>
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

// Spin until `t` reports at least `n` links whose peer has identified itself (sent its Hello). A
// registry rule or a push keyed by node id only applies to a connection once this holds.
bool wait_identified(const cvc::state_transport &t, std::size_t n, std::chrono::milliseconds to) {
  return wait_until(
      [&] {
        std::size_t k = 0;
        for (const auto &r : t.link_reports())
          if (!r.remote_node_id.empty())
            ++k;
        return k >= n;
      },
      to);
}

// Bytes this end has written on each link, keyed by the peer's node id.
std::map<std::string, std::uint64_t> bytes_out_by_peer(const cvc::state_transport &t) {
  std::map<std::string, std::uint64_t> out;
  for (const auto &r : t.link_reports())
    out[r.remote_node_id] = r.stats.bytes_out;
  return out;
}

std::string value_at(cvc::app &a, const std::string &path) {
  return cvc::state::instance(a)(path).value();
}

// The first set on a fresh child is not observed, so seed it before the value that must replicate.
void write_value(cvc::app &a, const std::string &path, const std::string &v) {
  cvc::state::instance(a)(path).value(std::string("seed"));
  cvc::state::instance(a)(path).value(v);
}

cvc::state_transport::snapshot_entry entry(const std::string &path, const std::string &v) {
  cvc::state_transport::snapshot_entry e;
  e.path = path;
  e.string_value = v;
  return e;
}

// A delivery filter that denies one subtree.
class deny_prefix_filter final : public cvc::state_delivery_filter {
public:
  explicit deny_prefix_filter(std::string prefix) : _prefix(std::move(prefix)) {}
  bool deliver(const std::string &path) const noexcept override {
    return !cvc::state_peer_registry::prefix_matches(_prefix, path);
  }

private:
  std::string _prefix;
};

std::uint64_t deliveries_filtered(const cvc::state_transport &t, const std::string &node_id) {
  for (const auto &p : t.peers().snapshot())
    if (p.node_id == node_id)
      return p.deliveries_filtered;
  return 0;
}

// Protobuf wire bytes of one length-delimited field (payloads under 128 bytes).
std::string pb_field(int field, const std::string &bytes) {
  std::string out;
  out.push_back(static_cast<char>((field << 3) | 2));
  out.push_back(static_cast<char>(bytes.size()));
  out += bytes;
  return out;
}

// A raw cvc.transport.v1.Frame carrying a Hello (Frame.hello = 1; Hello.node_id = 1,
// Hello.cluster_id = 2), and one carrying an empty Ping (Frame.ping = 10).
std::string hello_frame(const std::string &node_id, const std::string &cluster_id) {
  return pb_field(1, pb_field(1, node_id) + pb_field(2, cluster_id));
}
std::string ping_frame() { return pb_field(10, std::string()); }

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
  // Media now fans through a media_router (set_relay governs the control plane only). Declared
  // after S so it destructs first, clearing S's media sink while S is still alive.
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
  // No link probe here: a client that answers Pings is NOT idle (see QuietClientAnsweringPings...).
  S.set_ping_interval(std::chrono::milliseconds(0));
  tA.set_ping_interval(std::chrono::milliseconds(0));
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

// The link probe: both ends Ping, the other end Pongs, and each measures the round trip on its own
// clock. Each side reports one link with its peer's node id, traffic both ways, and an RTT.
TEST(StateTransportWsTest, LinkProbeMeasuresRoundTripAtBothEnds) {
  cvc::app aA;
  cvc::state_transport_ws S, tA;
  S.set_ping_interval(std::chrono::milliseconds(30));
  tA.set_ping_interval(std::chrono::milliseconds(30));
  S.start("127.0.0.1:0", "S", "C");
  cvc::state_cluster_shard sA(aA, "C", "A");
  sA.attach();
  tA.register_shard(&sA);
  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  const auto measured = [](const cvc::state_transport_ws &t) {
    const auto links = t.link_reports();
    return links.size() == 1 && links[0].stats.pongs_received >= 2 && links[0].stats.rtt_ms >= 0.0;
  };
  ASSERT_TRUE(
      wait_until([&] { return measured(S) && measured(tA); }, std::chrono::milliseconds(3000)));
  const auto server = S.link_reports().at(0);
  const auto client = tA.link_reports().at(0);
  EXPECT_EQ(server.transport, "ws");
  EXPECT_TRUE(server.accepted);
  EXPECT_EQ(server.remote_node_id, "A");
  EXPECT_FALSE(client.accepted);
  EXPECT_EQ(client.remote_node_id, "S");
  for (const auto &r : {server, client}) {
    EXPECT_GT(r.stats.bytes_in, 0u);
    EXPECT_GT(r.stats.bytes_out, 0u);
    EXPECT_GE(r.stats.rtt_min_ms, 0.0);
    EXPECT_LT(r.stats.rtt_ms, 1000.0) << "loopback round trip";
    EXPECT_GE(r.age_s, 0.0);
  }
  tA.stop();
  S.stop();
}

// A client that sends nothing of its own but still answers the relay's Pings is alive, so the idle
// reaper keeps it -- the backgrounded-browser-tab case (its WebSocket onmessage still fires and
// answers, while its timers are throttled). A client that cannot answer is still reaped
// (RelayReapsIdleConnection).
TEST(StateTransportWsTest, QuietClientAnsweringPingsIsNotReaped) {
  cvc::app aA;
  cvc::state_transport_ws S, tA;
  S.set_ping_interval(std::chrono::milliseconds(20));
  tA.set_ping_interval(std::chrono::milliseconds(0)); // the client never initiates anything
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  std::atomic<int> reaped{0};
  S.set_on_connection_closed([&](const std::string &) { reaped.fetch_add(1); });
  cvc::state_cluster_shard sA(aA, "C", "A");
  sA.attach();
  tA.register_shard(&sA);
  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 1; }, std::chrono::milliseconds(2000)));
  for (int i = 0; i < 20; ++i) { // ~600 ms of reaping with a 150 ms idle limit
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    S.reap_idle(std::chrono::milliseconds(150));
  }
  EXPECT_EQ(reaped.load(), 0) << "a client answering Pings is not a ghost";
  EXPECT_EQ(S.connection_count(), 1u);
  tA.stop();
  S.stop();
}

// A peer that stops reading (a frozen browser tab, a dead network) must not hang the relay: a write
// to it blocks once its TCP buffers fill, holding the connection's write lock, so without the write
// watchdog every later writer to it -- the relay's main loop included -- waits forever (seen in
// production). The watchdog closes a connection whose write has been blocked > 3 s: the write
// returns and the connection is gone.
TEST(StateTransportWsTest, WriteToAPeerThatStopsReadingIsAbortedNotHung) {
  namespace beast = boost::beast;
  namespace asio = boost::asio;
  cvc::state_transport_ws S;
  S.set_ping_interval(std::chrono::milliseconds(0)); // the watchdog still runs; no probe traffic
  S.start("127.0.0.1:0", "S", "C");
  const std::string addr = S.listen_address();
  const std::string port = addr.substr(addr.rfind(':') + 1);

  // A raw WebSocket peer: completes the handshake, then never reads.
  asio::io_context ioc;
  asio::ip::tcp::resolver resolver(ioc);
  asio::ip::tcp::socket sock(ioc);
  asio::connect(sock, resolver.resolve("127.0.0.1", port));
  beast::websocket::stream<asio::ip::tcp::socket> peer(std::move(sock));
  peer.handshake("127.0.0.1", "/");
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() == 1; }, std::chrono::milliseconds(2000)));

  std::atomic<bool> returned{false};
  std::thread writer([&] {
    cvc::ws_media_frame m;
    m.origin_node_id = "S";
    m.stream_id = "camera";
    m.codec = "mjpeg";
    m.payload.assign(std::size_t(32) << 20, 0x5a); // far beyond any socket buffer
    S.publish_media_frame(m);
    S.publish_media_frame(m);
    returned.store(true);
  });
  EXPECT_TRUE(wait_until([&] { return returned.load(); }, std::chrono::milliseconds(10000)))
      << "the write to a non-reading peer never returned (the relay would be hung)";
  EXPECT_TRUE(
      wait_until([&] { return S.connection_count() == 0; }, std::chrono::milliseconds(5000)))
      << "the stalled connection was not closed";
  writer.join();
  beast::error_code ec;
  peer.next_layer().close(ec);
  S.stop();
}

// Late-joiner bootstrap: a relay that holds the room tree in its own shard pushes a full snapshot
// to each newly admitted connection, so a client that joins AFTER others are present still sees
// them (live set_relay forwarding only carries writes made after the client connects). This is what
// lets a browser that refreshes / opens late see everyone already in the conference room.
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

// A connection's identity is the node id of its FIRST Hello: unknown before it, and a later Hello
// naming someone else changes nothing -- neither the link report nor which connection a push
// addressed to that id reaches.
TEST(StateTransportWsTest, SecondHelloIsIgnored) {
  namespace beast = boost::beast;
  namespace asio = boost::asio;
  cvc::state_transport_ws S;
  S.set_ping_interval(std::chrono::milliseconds(0)); // the raw peer's frames are the only traffic
  S.start("127.0.0.1:0", "S", "C");
  const std::string addr = S.listen_address();
  const std::string port = addr.substr(addr.rfind(':') + 1);

  asio::io_context ioc;
  asio::ip::tcp::resolver resolver(ioc);
  asio::ip::tcp::socket sock(ioc);
  asio::connect(sock, resolver.resolve("127.0.0.1", port));
  beast::websocket::stream<asio::ip::tcp::socket> peer(std::move(sock));
  peer.handshake("127.0.0.1", "/");
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() == 1; }, std::chrono::milliseconds(2000)));
  ASSERT_EQ(S.link_reports().size(), 1u);
  EXPECT_EQ(S.link_reports()[0].remote_node_id, "") << "no identity before the Hello";
  EXPECT_EQ(S.push_snapshot_to("first", {entry("k", "v")}), 0u);

  peer.binary(true);
  peer.write(asio::buffer(hello_frame("first", "C")));
  peer.write(asio::buffer(hello_frame("second", "C")));
  peer.write(asio::buffer(ping_frame()));
  // One reader parses a connection's frames in order and counts each as it starts on it, so a
  // third inbound frame means the second Hello has been fully handled.
  ASSERT_TRUE(wait_until(
      [&] {
        const auto links = S.link_reports();
        return links.size() == 1 && links[0].stats.frames_in >= 3;
      },
      std::chrono::milliseconds(2000)));
  const auto links = S.link_reports();
  ASSERT_EQ(links.size(), 1u);
  EXPECT_EQ(links[0].remote_node_id, "first");
  EXPECT_EQ(S.push_snapshot_to("second", {entry("k", "v")}), 0u);
  EXPECT_EQ(S.push_snapshot_to("first", {entry("k", "v")}), 1u);

  beast::error_code ec;
  peer.next_layer().close(ec);
  S.stop();
}

// The opt-in relay interest gate: a frame no local shard admits is neither re-fanned nor bridged.
// The relay's shard takes only "conf" (as a conference relay's does), so A's ui.docs.* write stays
// at the relay while its conf.* write reaches B; with the gate off the same write is forwarded.
TEST(StateTransportWsTest, RelayInterestGateWithholdsOutOfInterestPaths) {
  cvc::app aS, aA, aB;
  cvc::state_transport_ws S, tA, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  EXPECT_FALSE(S.relay_requires_interest()) << "off by default";
  S.set_relay_requires_interest(true);
  EXPECT_TRUE(S.relay_requires_interest());

  cvc::state_cluster_shard sS(aS, "C", "S");
  sS.add_interest("conf");
  sS.set_enforce_interest(true);
  // B mirrors everything, so whatever it lacks was withheld by the relay, not refused by B.
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sA.attach();
  sB.attach();
  S.register_shard(&sS);
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 2; }, std::chrono::milliseconds(2000)));

  write_value(aA, "ui.docs.x", "echo");
  write_value(aA, "conf.rooms.r.y", "joined");
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  // A's frames reach the relay, and B, in order over one connection each: once conf.* has landed,
  // a forwarded ui.docs.* would already have landed before it.
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "conf.rooms.r.y") == "joined"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aB, "ui.docs.x"), "");
  EXPECT_GE(S.total_relay_withheld(), 1u);
  EXPECT_EQ(value_at(aS, "conf.rooms.r.y"), "joined") << "ingest is unchanged";

  S.set_relay_requires_interest(false);
  const std::uint64_t withheld = S.total_relay_withheld();
  cvc::state::instance(aA)("ui.docs.x").value(std::string("echo2"));
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  EXPECT_TRUE(wait_until([&] { return value_at(aB, "ui.docs.x") == "echo2"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(S.total_relay_withheld(), withheld);

  tA.stop();
  tB.stop();
  S.stop();
}

// The gate keeps ingest's delegation exemption: a delegate op on a path outside the relay's
// interest still reaches the other clients, while an ordinary write to that subtree does not.
TEST(StateTransportWsTest, RelayInterestGateForwardsDelegationOps) {
  cvc::app aS, aA, aB;
  cvc::state_transport_ws S, tA, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  S.set_relay_requires_interest(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  sS.add_interest("conf");
  sS.set_enforce_interest(true);
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sA.attach();
  sB.attach();
  S.register_shard(&sS);
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() >= 2; }, std::chrono::milliseconds(2000)));

  write_value(aA, "ui.docs.x", "echo");
  sA.publish_delegation("ui.docs.owned", "other-cluster", "", /*lease=*/0);
  write_value(aA, "conf.k", "after");
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "conf.k") == "after"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(sB.total_delegations_applied(), 1u) << "the delegation op was forwarded";
  EXPECT_EQ(sS.total_delegations_applied(), 1u);
  EXPECT_EQ(value_at(aB, "ui.docs.x"), "") << "the ordinary write was withheld";
  EXPECT_GE(S.total_relay_withheld(), 1u);

  tA.stop();
  tB.stop();
  S.stop();
}

// The relay's re-fan honours each target's registry entry: B's delivery filter denies x.secret, so
// B receives only x.pub, while C (unregistered) receives both.
TEST(StateTransportWsTest, RelayForwardHonorsPeerFilter) {
  cvc::app aA, aB, aC;
  cvc::state_transport_ws S, tA, tB, tC;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true); // no shard: a pure forwarding relay

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C1");
  sA.attach();
  sB.attach();
  sC.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 3, std::chrono::milliseconds(2000)));
  S.peers().add_peer("B", "C");
  ASSERT_TRUE(S.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.secret")));

  write_value(aA, "x.secret", "s");
  write_value(aA, "x.pub", "p");
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  ASSERT_TRUE(wait_until(
      [&] {
        return value_at(aC, "x.secret") == "s" && value_at(aC, "x.pub") == "p" &&
               value_at(aB, "x.pub") == "p";
      },
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aB, "x.secret"), "");
  EXPECT_GE(deliveries_filtered(S, "B"), 1u);

  tA.stop();
  tB.stop();
  tC.stop();
  S.stop();
}

// The periodic broadcast is filtered per connection. B's filter denies room.secret, so its frame
// omits it; C (unregistered) and D (registered with no rule) are unfiltered and get the one shared
// frame -- byte-for-byte the same, and larger than B's.
TEST(StateTransportWsTest, BroadcastSnapshotIsPerConnectionFiltered) {
  cvc::app aS, aB, aC, aD;
  cvc::state_transport_ws S, tB, tC, tD;
  // No link probe at either end: the broadcast is the only traffic the byte counts below see.
  for (auto *t : {&S, &tB, &tC, &tD})
    t->set_ping_interval(std::chrono::milliseconds(0));
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C1");
  cvc::state_cluster_shard sD(aD, "C", "D");
  sS.attach();
  sB.attach();
  sC.attach();
  sD.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tC.register_shard(&sC);
  tD.register_shard(&sD);

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tD.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 3, std::chrono::milliseconds(2000)));
  S.peers().add_peer("B", "C");
  ASSERT_TRUE(S.peers().set_filter("B", std::make_shared<deny_prefix_filter>("room.secret")));
  S.peers().add_peer("D", "C");

  // Relay-local writes made after everyone identified, and never pumped: only the broadcast can
  // carry them.
  write_value(aS, "room.secret", "s");
  write_value(aS, "room.pub", "p");
  // A connection's admit frames go out under its write lock, which a broadcast write waits for, so
  // after one broadcast every admit is counted and the next broadcast's bytes stand alone.
  EXPECT_EQ(S.broadcast_snapshot(), 3u);
  const auto before = bytes_out_by_peer(S);
  EXPECT_EQ(S.broadcast_snapshot(), 3u);
  const auto after = bytes_out_by_peer(S);
  // A push queued behind the broadcast on B's connection: once it lands, B has applied the whole
  // broadcast frame.
  ASSERT_EQ(S.push_snapshot_to("B", {entry("room.sentinel", "1")}), 1u);
  ASSERT_TRUE(wait_until(
      [&] {
        return value_at(aB, "room.sentinel") == "1" && value_at(aC, "room.secret") == "s" &&
               value_at(aC, "room.pub") == "p" && value_at(aD, "room.secret") == "s" &&
               value_at(aD, "room.pub") == "p";
      },
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aB, "room.pub"), "p");
  EXPECT_EQ(value_at(aB, "room.secret"), "");

  const std::uint64_t sent_b = after.at("B") - before.at("B");
  const std::uint64_t sent_c = after.at("C1") - before.at("C1");
  const std::uint64_t sent_d = after.at("D") - before.at("D");
  EXPECT_GT(sent_c, 0u);
  EXPECT_EQ(sent_c, sent_d) << "unfiltered connections share one frame";
  EXPECT_LT(sent_b, sent_c) << "the filtered connection's frame omits the denied entry";

  tB.stop();
  tC.stop();
  tD.stop();
  S.stop();
}

// push_snapshot_to writes only to the connection that identified as the named node, regardless of
// the registry (the caller decided), and reports how many connections it wrote.
TEST(StateTransportWsTest, PushSnapshotToReachesOnlyNamedPeer) {
  cvc::app aB, aC;
  cvc::state_transport_ws S, tB, tC;
  S.start("127.0.0.1:0", "S", "C");

  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C1");
  sB.attach();
  sC.attach();
  tB.register_shard(&sB);
  tC.register_shard(&sC);
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 2, std::chrono::milliseconds(2000)));
  // B's registry entry would deny push.* to the live stream; a push bypasses it.
  S.peers().add_peer("B", "C", std::string(), {"other"});

  EXPECT_EQ(S.push_snapshot_to("nobody", {entry("push.k", "v")}), 0u);
  EXPECT_EQ(S.push_snapshot_to("B", {}), 0u);
  EXPECT_EQ(S.push_snapshot_to("B", {entry("push.k", "for-b")}), 1u);
  EXPECT_EQ(S.push_snapshot_to("C1", {entry("push.c", "for-c")}), 1u);
  ASSERT_TRUE(wait_until(
      [&] { return value_at(aB, "push.k") == "for-b" && value_at(aC, "push.c") == "for-c"; },
      std::chrono::milliseconds(3000)));
  // C's push was written after B's, on the same thread: had B's frame also gone to C, C would have
  // applied it first.
  EXPECT_EQ(value_at(aC, "push.k"), "");
  EXPECT_EQ(value_at(aB, "push.c"), "");

  tB.stop();
  tC.stop();
  S.stop();
}

// A push and the publishes after it share the connection's FIFO, so a pushed value never lands on
// top of a newer live write -- and a push after the last live write wins.
TEST(StateTransportWsTest, PushThenPublishKeepsOrder) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.start("127.0.0.1:0", "S", "C");
  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(2000)));

  write_value(aS, "p", "live-start");
  S.pump_all();
  for (int i = 0; i < 20; ++i) {
    const std::string n = std::to_string(i);
    ASSERT_EQ(S.push_snapshot_to("B", {entry("p", "push-" + n)}), 1u);
    cvc::state::instance(aS)("p").value("live-" + n);
    EXPECT_GT(S.pump_all(), 0u);
  }
  ASSERT_TRUE(
      wait_until([&] { return value_at(aB, "p") == "live-19"; }, std::chrono::milliseconds(3000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(value_at(aB, "p"), "live-19") << "no pushed value arrived after the newer live write";

  ASSERT_EQ(S.push_snapshot_to("B", {entry("p", "push-last")}), 1u);
  EXPECT_TRUE(wait_until([&] { return value_at(aB, "p") == "push-last"; },
                         std::chrono::milliseconds(3000)));

  tB.stop();
  S.stop();
}

// The media egress gate is asked once per identified receiver of a fanned frame: the receiver it
// denies is skipped, the others still receive, the origin is never asked, and the relay's own
// uplink (publish_media_frame) is never gated. Clearing the gate restores the full fan-out.
TEST(StateTransportWsTest, RelayMediaGateSkipsDeniedReceiver) {
  std::mutex mu;
  std::vector<std::string> asked;
  std::vector<std::int64_t> gotB;
  std::atomic<int> gotC{0};
  cvc::app aA, aB, aC;
  cvc::state_transport_ws S, tA, tB, tC;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::media_router mr;
  mr.add_endpoint(&S);
  S.set_media_gate([&](const cvc::media_frame &, const std::string &receiver) {
    std::lock_guard<std::mutex> lk(mu);
    asked.push_back(receiver);
    return receiver != "B";
  });

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C1");
  sA.attach();
  sB.attach();
  sC.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);
  tB.set_on_media([&](const cvc::ws_media_frame &m) {
    std::lock_guard<std::mutex> lk(mu);
    gotB.push_back(m.seq);
  });
  tC.set_on_media([&](const cvc::ws_media_frame &) { gotC.fetch_add(1); });

  ASSERT_TRUE(tA.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 3, std::chrono::milliseconds(2000)));

  cvc::ws_media_frame m;
  m.origin_node_id = "A";
  m.stream_id = "camera";
  m.codec = "mjpeg";
  m.seq = 1;
  m.payload = {0xff, 0xd8, 0x01, 0xff, 0xd9};
  tA.publish_media_frame(m);
  ASSERT_TRUE(wait_until([&] { return gotC.load() >= 1; }, std::chrono::milliseconds(3000)));
  // Wait for the gate's decision on every receiver (C's delivery does not order B's).
  ASSERT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return asked.size() >= 2;
      },
      std::chrono::milliseconds(3000)));
  // Counted just after the gate returns, so wait for it rather than racing the fanning thread.
  EXPECT_TRUE(
      wait_until([&] { return S.total_media_gated() == 1; }, std::chrono::milliseconds(3000)));

  cvc::ws_media_frame own = m;
  own.origin_node_id = "S";
  own.seq = 2;
  S.publish_media_frame(own);
  ASSERT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !gotB.empty();
      },
      std::chrono::milliseconds(3000)));
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(gotB, std::vector<std::int64_t>{2}) << "B gets the relay's own frame, not A's";
    std::vector<std::string> sorted = asked;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(sorted, (std::vector<std::string>{"B", "C1"})) << "once per receiver, never A";
  }

  S.set_media_gate({});
  m.seq = 3;
  tA.publish_media_frame(m);
  EXPECT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return gotB.size() == 2 && gotB[1] == 3;
      },
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(S.total_media_gated(), 1u);

  tA.stop();
  tB.stop();
  tC.stop();
  S.stop();
}

// A client whose shard enforces an interest set applies only the snapshot entries inside it, as
// live ingest does: the relay's whole-tree admit snapshot (and a push) no longer plants ui.docs.*
// nodes in a client that takes only conf.*. A client that does not enforce still takes them all.
TEST(StateTransportWsTest, ClientSnapshotIngestHonorsEnforcedInterest) {
  cvc::app aS, aB, aC;
  cvc::state_transport_ws S, tB, tC;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);                // the admit snapshot
  S.set_backfill_on_connect(false); // ... and nothing else carries the relay-local writes

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C1");
  sB.add_interest("conf");
  sB.set_enforce_interest(true);
  sS.attach();
  sB.attach();
  sC.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  write_value(aS, "ui.docs.b", "relay-ui");
  write_value(aS, "conf.a", "relay-conf");

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 2, std::chrono::milliseconds(2000)));
  // Queued behind the admit snapshot on B's connection; its last entry landing means B has
  // processed both frames.
  ASSERT_EQ(S.push_snapshot_to("B", {entry("ui.docs.c", "pushed-ui"), entry("conf.z", "z")}), 1u);
  ASSERT_TRUE(
      wait_until([&] { return value_at(aB, "conf.z") == "z"; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aB, "conf.a"), "relay-conf");
  EXPECT_EQ(value_at(aB, "ui.docs.b"), "");
  EXPECT_EQ(value_at(aB, "ui.docs.c"), "");

  EXPECT_TRUE(wait_until(
      [&] {
        return value_at(aC, "ui.docs.b") == "relay-ui" && value_at(aC, "conf.a") == "relay-conf";
      },
      std::chrono::milliseconds(3000)));

  tB.stop();
  tC.stop();
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

// The interest gate covers the bridge leg in both directions: with the relay's shard taking only
// "conf", a client's ui.docs.* write is neither re-fanned nor bridged to the other protocol, while
// its conf.* write crosses.
TEST(StateTransportWsTest, BridgeLegHonoursGate) {
  cvc::app aRelay, aWs, aGrpc;
  cvc::state_transport_grpc G;
  cvc::state_transport_ws W;
  G.start("127.0.0.1:0", "relay", "C");
  W.start("127.0.0.1:0", "relay", "C");
  G.set_relay(true);
  W.set_relay(true);
  G.set_relay_requires_interest(true);
  W.set_relay_requires_interest(true);
  G.set_bridge(&W);
  W.set_bridge(&G);
  cvc::state_cluster_shard sR(aRelay, "C", "relay");
  sR.add_interest("conf");
  sR.set_enforce_interest(true);
  sR.attach();
  G.register_shard(&sR);
  W.register_shard(&sR);

  // Neither client enforces interest, so what one lacks was withheld by the relay.
  cvc::state_transport_ws wc;
  cvc::state_cluster_shard sWs(aWs, "C", "wsclient");
  sWs.attach();
  wc.register_shard(&sWs);
  ASSERT_TRUE(wc.connect_to_peer(ws(W.listen_address()), std::chrono::milliseconds(2000)));
  cvc::state_transport_grpc gc;
  gc.start("127.0.0.1:0", "grpcclient", "C");
  cvc::state_cluster_shard sGrpc(aGrpc, "C", "grpcclient");
  sGrpc.attach();
  gc.register_shard(&sGrpc);
  ASSERT_TRUE(gc.connect_to_peer(G.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_until([&] { return G.connection_count() >= 1 && W.connection_count() >= 1; },
                         std::chrono::milliseconds(2000)));

  // WS -> gRPC. The WS reader withholds or bridges each frame in order, onto one gRPC stream.
  write_value(aWs, "ui.docs.w", "from-ws");
  write_value(aWs, "conf.w", "from-ws");
  wc.pump_all();
  wc.flush();
  ASSERT_TRUE(wait_until([&] { return value_at(aGrpc, "conf.w") == "from-ws"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aGrpc, "ui.docs.w"), "");
  EXPECT_GE(W.total_relay_withheld(), 1u);

  // gRPC -> WS.
  write_value(aGrpc, "ui.docs.g", "from-grpc");
  write_value(aGrpc, "conf.g", "from-grpc");
  gc.pump_all();
  gc.flush();
  ASSERT_TRUE(wait_until([&] { return value_at(aWs, "conf.g") == "from-grpc"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aWs, "ui.docs.g"), "");
  EXPECT_GE(G.total_relay_withheld(), 1u);

  wc.stop();
  gc.stop();
  W.stop();
  G.stop();
}

// The bridge leg goes through the destination transport's publish(), so it honours THAT
// transport's registry: the WS client's filter denies x.secret, which a gRPC client writes.
TEST(StateTransportWsTest, BridgeLegHonorsDestinationRegistry) {
  cvc::app aRelay, aWs, aGrpc;
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

  cvc::state_transport_ws wc;
  cvc::state_cluster_shard sWs(aWs, "C", "wsclient");
  sWs.attach();
  wc.register_shard(&sWs);
  ASSERT_TRUE(wc.connect_to_peer(ws(W.listen_address()), std::chrono::milliseconds(2000)));
  cvc::state_transport_grpc gc;
  gc.start("127.0.0.1:0", "grpcclient", "C");
  cvc::state_cluster_shard sGrpc(aGrpc, "C", "grpcclient");
  sGrpc.attach();
  gc.register_shard(&sGrpc);
  ASSERT_TRUE(gc.connect_to_peer(G.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return G.connection_count() >= 1; }, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(W, 1, std::chrono::milliseconds(2000)));
  W.peers().add_peer("wsclient", "C");
  ASSERT_TRUE(W.peers().set_filter("wsclient", std::make_shared<deny_prefix_filter>("x.secret")));

  write_value(aGrpc, "x.secret", "s");
  write_value(aGrpc, "x.pub", "p");
  gc.pump_all();
  gc.flush();
  ASSERT_TRUE(
      wait_until([&] { return value_at(aWs, "x.pub") == "p"; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aWs, "x.secret"), "");
  EXPECT_EQ(value_at(aRelay, "x.secret"), "s") << "the relay itself still ingested it";
  EXPECT_GE(deliveries_filtered(W, "wsclient"), 1u);

  wc.stop();
  gc.stop();
  W.stop();
  G.stop();
}
#endif // CVC_ENABLE_GRPC

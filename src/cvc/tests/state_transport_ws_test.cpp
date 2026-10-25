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
#include <cvc/state/state_device_key.h>
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

// Protobuf wire helpers for a raw client (no generated code in the tests): a length-delimited
// field, and the first length-delimited field FIELD of MSG ("" when absent).
std::string pb_field(unsigned field, const std::string &bytes) {
  std::string out;
  const auto varint = [&out](std::uint64_t v) {
    for (; v >= 0x80; v >>= 7)
      out.push_back(static_cast<char>((v & 0x7f) | 0x80));
    out.push_back(static_cast<char>(v));
  };
  varint((field << 3) | 2);
  varint(bytes.size());
  return out + bytes;
}
std::string pb_get(const std::string &msg, unsigned field) {
  std::size_t i = 0;
  const auto varint = [&]() {
    std::uint64_t v = 0;
    for (int shift = 0; i < msg.size() && shift < 64; shift += 7) {
      const auto b = static_cast<unsigned char>(msg[i++]);
      v |= static_cast<std::uint64_t>(b & 0x7f) << shift;
      if (!(b & 0x80))
        break;
    }
    return v;
  };
  while (i < msg.size()) {
    const std::uint64_t key = varint();
    switch (key & 7) {
    case 0:
      (void)varint();
      break;
    case 1:
      i += 8;
      break;
    case 5:
      i += 4;
      break;
    case 2: {
      const std::uint64_t n = varint();
      if (n > msg.size() - (std::min)(i, msg.size()))
        return {};
      if ((key >> 3) == field)
        return msg.substr(i, n);
      i += n;
      break;
    }
    default:
      return {};
    }
  }
  return {};
}

// A WebSocket client speaking raw frames to ADDR (host:port), so a test can send what a
// state_transport_ws client never would, in any order.
class raw_ws_client {
public:
  explicit raw_ws_client(const std::string &addr) : _resolver(_ioc), _ws(_ioc) {
    const std::size_t colon = addr.rfind(':');
    boost::asio::connect(_ws.next_layer(),
                         _resolver.resolve(addr.substr(0, colon), addr.substr(colon + 1)));
    _ws.handshake(addr, "/");
    _ws.binary(true);
  }
  // The next binary frame.
  std::string read() {
    boost::beast::flat_buffer buf;
    _ws.read(buf);
    return boost::beast::buffers_to_string(buf.data());
  }
  void write(const std::string &frame) { _ws.write(boost::asio::buffer(frame)); }

private:
  boost::asio::io_context _ioc;
  boost::asio::ip::tcp::resolver _resolver;
  boost::beast::websocket::stream<boost::asio::ip::tcp::socket> _ws;
};

// Frame{hello: Hello{node_id, cluster_id, device_id}} and Frame{device_proof: DeviceProof}.
std::string hello_frame(const std::string &node, const std::string &cluster,
                        const std::string &device) {
  return pb_field(1, pb_field(1, node) + pb_field(2, cluster) + pb_field(3, device));
}
std::string proof_frame(const std::string &key, const std::string &signature) {
  return pb_field(12, pb_field(1, key) + pb_field(2, signature));
}

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

// A value mutation as `origin` wrote it (publish() sends it as is, whoever the sender is).
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
// omits it; C (unregistered) and D (registered with no rule) are unfiltered and hold the same, so
// they get the same frame -- byte-for-byte, and larger than B's.
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
  // A connection's admit frames go out under its write lock, which flush() waits for, so the
  // broadcast's bytes stand alone.
  S.flush();
  const auto before = bytes_out_by_peer(S);
  EXPECT_EQ(S.broadcast_snapshot(), 3u);
  const auto after = bytes_out_by_peer(S);
  EXPECT_EQ(S.broadcast_snapshot(), 0u) << "every peer already holds what it is delivered";
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

// The broadcast sends diffs, and must still converge a client that lost frames. Each way to lose
// one, and what heals it:
//   1. a change no live frame carried (a missed re-fan): the next diff carries it, and only it;
//   2. a live frame that reached the client, then a change back to the value the relay last
//      broadcast that the client missed: the live frame made the relay forget what the client held
//      there, so the diff re-sends it even though the tree matches the last broadcast again;
//   3. a frame the client dropped itself (no shard registered when it arrived): the relay wrote it,
//      so no diff re-sends it, and only the client's full-refresh request heals it;
//   4. the client's own write, overridden on the relay without a live frame: its write made the
//      relay forget what it held there, so the diff sends the tree's value back.
TEST(StateTransportWsTest, BroadcastDiffConvergesAClientThatDroppedFrames) {
  using ms = std::chrono::milliseconds;
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  for (auto *t : {&S, &tB})
    t->set_ping_interval(ms(0)); // only the frames below cross the link
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  write_value(aS, "room.k", "v0");
  EXPECT_GT(S.pump_all(), 0u); // no connection yet: later pumps carry only later writes
  const auto set = [](cvc::app &a, const std::string &v) {
    cvc::state::instance(a)("room.k").value(v);
  };

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), ms(2000)));
  ASSERT_TRUE(wait_identified(S, 1, ms(2000)));
  S.flush(); // B's admit snapshot is out
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v0"; }, ms(3000)));
  EXPECT_EQ(S.broadcast_snapshot(), 0u) << "B holds what its admit snapshot carried";

  // 1.
  set(aS, "v1"); // never pumped
  std::uint64_t e0 = S.total_broadcast_entries();
  EXPECT_EQ(S.broadcast_snapshot(), 1u);
  EXPECT_EQ(S.total_broadcast_entries() - e0, 1u) << "only the changed entry";
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v1"; }, ms(3000)));
  EXPECT_EQ(S.broadcast_snapshot(), 0u);

  // 2.
  set(aS, "v2");
  EXPECT_GT(S.pump_all(), 0u);
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v2"; }, ms(3000)));
  set(aS, "v1"); // never pumped: the tree is back to what the last broadcast carried
  EXPECT_EQ(S.broadcast_snapshot(), 1u);
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v1"; }, ms(3000)));

  // 3.
  tB.unregister_shard(&sB);
  set(aS, "v3"); // never pumped
  const std::uint64_t r0 = tB.total_received_frames();
  EXPECT_EQ(S.broadcast_snapshot(), 1u);
  // B reads its frames in order on one thread, so once the push behind the diff has arrived, B is
  // done with the diff -- which it dropped, having no shard.
  ASSERT_EQ(S.push_snapshot_to("B", {entry("room.sentinel", "1")}), 1u);
  ASSERT_TRUE(wait_until([&] { return tB.total_received_frames() >= r0 + 2; }, ms(3000)));
  tB.register_shard(&sB);
  EXPECT_EQ(S.broadcast_snapshot(), 0u) << "the relay believes B holds v3";
  EXPECT_EQ(value_at(aB, "room.k"), "v1");
  ASSERT_EQ(tB.request_full_refresh(), 1u);
  e0 = S.total_broadcast_entries();
  ASSERT_TRUE(wait_until([&] { return S.broadcast_snapshot() == 1u; }, ms(3000)));
  EXPECT_GE(S.total_broadcast_entries() - e0, 2u) << "the whole tree: room and room.k";
  ASSERT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v3"; }, ms(3000)));
  EXPECT_EQ(S.broadcast_snapshot(), 0u) << "back to diffs";

  // 4.
  set(aB, "b4");
  EXPECT_GT(tB.pump_all(), 0u);
  ASSERT_TRUE(wait_until([&] { return value_at(aS, "room.k") == "b4"; }, ms(3000)));
  set(aS, "v3"); // never pumped: the relay overrides B's write with what it last sent B
  EXPECT_EQ(S.broadcast_snapshot(), 1u);
  EXPECT_TRUE(wait_until([&] { return value_at(aB, "room.k") == "v3"; }, ms(3000)));

  tB.stop();
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

// The admission hold (as StateTransportGrpcTest.AdmissionHoldFiltersTheBootstrap): an accepted
// connection gets the relay's Hello and nothing else until release_admissions() finds a rule for
// its peer; then its admit snapshot goes through the rule and holds what was written meanwhile.
TEST(StateTransportWsTest, AdmissionHoldFiltersTheBootstrap) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.set_backfill_on_connect(false);
  S.set_admission_hold(std::chrono::seconds(30));
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  write_value(aS, "x.secret", "s");
  write_value(aS, "x.pub", "p");

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(2000)));
  EXPECT_EQ(S.total_admissions_held(), 1u) << "held before it was visible";
  EXPECT_EQ(S.release_admissions(), 0u) << "no rule for B yet";
  EXPECT_EQ(S.push_snapshot_to("B", {entry("x.pushed", "1")}), 0u);
  write_value(aS, "x.late", "l");
  S.pump_all(); // x.late goes out live, to no one
  S.broadcast_snapshot();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(value_at(aB, "x.pub"), "") << "a held peer gets no bootstrap or re-broadcast";
  EXPECT_EQ(value_at(aB, "x.late"), "") << "...and no live write";
  EXPECT_EQ(value_at(aB, "x.pushed"), "") << "...and no push";

  S.peers().add_peer("B", "C");
  ASSERT_TRUE(S.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.secret")));
  EXPECT_EQ(S.release_admissions(), 1u);
  EXPECT_EQ(S.release_admissions(), 0u) << "released once";
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.pub") == "p" && value_at(aB, "x.late") == "l"; },
                 std::chrono::milliseconds(3000)))
      << "the bootstrap carries what was written during the hold";
  EXPECT_EQ(value_at(aB, "x.secret"), "") << "the bootstrap goes through B's rule";
  EXPECT_EQ(S.total_admission_timeouts(), 0u);

  cvc::state::instance(aS)("x.late").value(std::string("l2"));
  S.pump_all();
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.late") == "l2"; }, std::chrono::milliseconds(3000)))
      << "live from the release on";

  // The release recorded only what B's rule let through as B's starting point for the broadcast
  // diff, so once the rule allows x.secret the next broadcast sends it.
  ASSERT_TRUE(S.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.none")));
  S.broadcast_snapshot();
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.secret") == "s"; }, std::chrono::milliseconds(3000)))
      << "an entry the release withheld is not taken as held";

  tB.stop();
  S.stop();
}

// With backfill on (a relay that keeps its journal), the release replays the journal through the
// peer's rule too, not only the snapshot.
TEST(StateTransportWsTest, AdmissionHoldFiltersTheBackfillToo) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.set_admission_hold(std::chrono::seconds(30));
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  write_value(aS, "x.secret", "s");
  write_value(aS, "x.pub", "p");

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(2000)));
  S.peers().add_peer("B", "C");
  ASSERT_TRUE(S.peers().set_filter("B", std::make_shared<deny_prefix_filter>("x.secret")));
  const std::uint64_t backfilled = S.total_backfilled();
  EXPECT_EQ(S.release_admissions(), 1u);
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.pub") == "p"; }, std::chrono::milliseconds(3000)));
  EXPECT_GT(S.total_backfilled(), backfilled) << "the journal was replayed";
  EXPECT_EQ(value_at(aB, "x.secret"), "") << "...through B's rule";

  tB.stop();
  S.stop();
}

// A peer with no rule is bootstrapped, unfiltered, when the hold runs out (counted as a timeout).
TEST(StateTransportWsTest, AdmissionHoldTimesOutUnfiltered) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.set_backfill_on_connect(false);
  S.set_admission_hold(std::chrono::milliseconds(500));
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  write_value(aS, "x.secret", "s");

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(2000)));
  EXPECT_TRUE(
      wait_until([&] { return S.release_admissions() == 1u; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(S.total_admission_timeouts(), 1u);
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.secret") == "s"; }, std::chrono::milliseconds(3000)))
      << "no rule: the old, unfiltered bootstrap";

  tB.stop();
  S.stop();
}

// A relay with a room token (auth_config) refuses the upgrade of a client that does not offer it,
// before any frame, so such a client is sent nothing; one that offers it is admitted and
// bootstrapped. The device id its Hello carried shows in the relay's link report.
TEST(StateTransportWsTest, RoomTokenIsCheckedAtTheHandshake) {
  cvc::app aS, aB;
  cvc::state_transport_ws S;
  S.set_auth_config({"room-token-0123456789", ""});
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S");
  sS.attach();
  S.register_shard(&sS);
  write_value(aS, "x.secret", "s");

  {
    cvc::state_transport_ws none;
    EXPECT_FALSE(none.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)))
        << "no token";
  }
  {
    cvc::state_transport_ws wrong;
    wrong.set_auth_config({"", "room-token-9876543210"});
    EXPECT_FALSE(wrong.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)))
        << "a wrong token";
  }
  EXPECT_EQ(S.total_auth_rejected(), 2u);
  EXPECT_EQ(S.connection_count(), 0u) << "neither was admitted";

  cvc::state_transport_ws tB;
  cvc::state_cluster_shard sB(aB, "C", "B");
  sB.attach();
  tB.register_shard(&sB);
  tB.set_auth_config({"", "room-token-0123456789"});
  tB.set_device_id("0123456789abcdef0123456789abcdef");
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(3000)));
  const auto links = S.link_reports();
  ASSERT_EQ(links.size(), 1u);
  EXPECT_EQ(links[0].remote_device_id, "0123456789abcdef0123456789abcdef");
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.secret") == "s"; }, std::chrono::milliseconds(3000)))
      << "admitted, and bootstrapped";
  EXPECT_EQ(S.total_auth_rejected(), 2u);

  // A browser requires the reply to name one of the subprotocols it offered: a raw handshake with
  // the browser's offer gets cvc.v1 back. (Beast's own client does not check, so this would pass
  // tB's connect even if the reply were wrong.)
  {
    namespace beast = boost::beast;
    boost::asio::io_context ioc;
    boost::asio::ip::tcp::resolver resolver(ioc);
    const std::string addr = S.listen_address();
    const std::size_t colon = addr.rfind(':');
    beast::websocket::stream<boost::asio::ip::tcp::socket> raw(ioc);
    boost::asio::connect(raw.next_layer(),
                         resolver.resolve(addr.substr(0, colon), addr.substr(colon + 1)));
    raw.set_option(
        beast::websocket::stream_base::decorator([](beast::websocket::request_type &req) {
          req.set(beast::http::field::sec_websocket_protocol,
                  "cvc.v1,cvc-token.room-token-0123456789");
        }));
    beast::websocket::response_type res;
    beast::error_code ec;
    raw.handshake(res, addr, "/", ec);
    ASSERT_FALSE(ec) << ec.message();
    EXPECT_EQ(std::string(res[beast::http::field::sec_websocket_protocol]), "cvc.v1");
    raw.next_layer().close(ec);
  }

  tB.stop();
  S.stop();
}

// A client that offers a room token to an open relay (no expected_token) is admitted as before.
TEST(StateTransportWsTest, ARoomTokenOfferedToAnOpenRelayIsHarmless) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  write_value(aS, "x.k", "v");
  tB.set_auth_config({"", "room-token-0123456789"});
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.k") == "v"; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(S.total_auth_rejected(), 0u);
  tB.stop();
  S.stop();
}

// Each accepted connection does its handshake on its own thread: a client that connects and sends
// nothing does not hold up the next join.
TEST(StateTransportWsTest, AnIdleHandshakeDoesNotBlockOtherJoins) {
  cvc::app aS, aB;
  cvc::state_transport_ws S, tB;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sS.attach();
  sB.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);

  boost::asio::io_context ioc;
  boost::asio::ip::tcp::socket idle(ioc);
  const std::string addr = S.listen_address();
  const std::size_t colon = addr.rfind(':');
  boost::asio::ip::tcp::resolver resolver(ioc);
  boost::asio::connect(idle, resolver.resolve(addr.substr(0, colon), addr.substr(colon + 1)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // accepted, and silent

  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(tB.connect_to_peer(ws(addr), std::chrono::milliseconds(2000)));
  EXPECT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(2000)));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(2));

  boost::system::error_code ec;
  idle.close(ec);
  tB.stop();
  S.stop(); // returns although a handshake was pending
}

// The link report says which link holds its node id at the inbound gate (a relay writes a
// participant's per-link leaves from that one only), and carries only a valid device id.
TEST(StateTransportWsTest, LinkReportsShowTheHolderAndAValidDeviceId) {
  cvc::app aS, aB, aB2, aC;
  cvc::state_transport_ws S, tB, tB2, tC;
  S.set_inbound_gate(std::make_shared<cvc::state_inbound_gate>());
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sB2(aB2, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  for (cvc::state_cluster_shard *s : {&sS, &sB, &sB2, &sC})
    s->attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tB2.register_shard(&sB2);
  tC.register_shard(&sC);
  tB.set_device_id("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
  tB2.set_device_id("dddddddddddddddddddddddddddddddd");
  tC.set_device_id(std::string(65, 'c')); // too long: ignored
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 1, std::chrono::milliseconds(3000)));
  ASSERT_TRUE(tB2.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tC.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 3, std::chrono::milliseconds(3000)));

  int holders_of_b = 0;
  for (const cvc::state_link_report &l : S.link_reports()) {
    if (l.remote_node_id == "B" && l.holds_id) {
      ++holders_of_b;
      EXPECT_EQ(l.remote_device_id, "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
          << "the first claim holds B";
    }
    if (l.remote_node_id == "B" && !l.holds_id)
      EXPECT_EQ(l.remote_device_id, "dddddddddddddddddddddddddddddddd");
    if (l.remote_node_id == "C")
      EXPECT_EQ(l.remote_device_id, "") << "an oversized device id is ignored";
  }
  EXPECT_EQ(holders_of_b, 1);

  tB.stop();
  tB2.stop();
  tC.stop();
  S.stop();
}

// Device keys (state_device_key.h): a relay with a pin store challenges each client in its Hello.
// A client with a key and a device id proves it and is verified, pinning its key; one with an id
// and no key, or a key and no id, is not; one showing a pinned id under another key is refused.
// A relay without pins challenges nobody, so nothing is sent to refuse.
TEST(StateTransportWsTest, DeviceProofsPinOnFirstUse) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  const std::string dev_b = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
  cvc::app aS, aB, aN, aK, aI;
  cvc::state_transport_ws S, tB, tN, tK, tI;
  S.set_device_pins(std::make_shared<cvc::state_device_pins>());
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S"), sB(aB, "C", "B"), sN(aN, "C", "N"), sK(aK, "C", "K"),
      sI(aI, "C", "I");
  for (cvc::state_cluster_shard *s : {&sS, &sB, &sN, &sK, &sI})
    s->attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tN.register_shard(&sN);
  tK.register_shard(&sK);
  tI.register_shard(&sI);
  const std::string scope = ws(S.listen_address());
  tB.set_device_id(dev_b);
  tB.set_device_signer(cvc::state_device_signer::derive(std::string(32, 'b'), scope));
  tN.set_device_id("nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn");
  tK.set_device_signer(cvc::state_device_signer::derive(std::string(32, 'k'), scope));
  tI.set_device_id(dev_b);
  tI.set_device_signer(cvc::state_device_signer::derive(std::string(32, 'i'), scope));
  for (cvc::state_transport_ws *t : {&tB, &tN, &tK})
    ASSERT_TRUE(t->connect_to_peer(scope, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return S.total_devices_verified() == 1; }, std::chrono::milliseconds(3000)))
      << "B proves its key";
  ASSERT_TRUE(tI.connect_to_peer(scope, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_until([&] { return S.total_device_proofs_refused() == 1; },
                         std::chrono::milliseconds(3000)))
      << "I shows B's id under another key";
  ASSERT_TRUE(wait_identified(S, 4, std::chrono::milliseconds(3000)));
  std::map<std::string, bool> verified;
  for (const cvc::state_link_report &l : S.link_reports())
    verified[l.remote_node_id] = l.device_verified;
  EXPECT_TRUE(verified["B"]);
  EXPECT_FALSE(verified["N"]) << "an id without a key";
  EXPECT_FALSE(verified["K"]) << "a key without an id";
  EXPECT_FALSE(verified["I"]) << "another key for a pinned id";
  EXPECT_EQ(S.total_devices_verified(), 1u);
  EXPECT_EQ(S.total_device_proofs_refused(), 1u);
  for (const cvc::state_link_report &l : tB.link_reports())
    EXPECT_FALSE(l.device_verified) << "a client verifies no relay";

  cvc::app aP;
  cvc::state_transport_ws P; // no pins
  P.start("127.0.0.1:0", "P", "C");
  cvc::state_cluster_shard sP(aP, "C", "P");
  sP.attach();
  P.register_shard(&sP);
  cvc::app aB2;
  cvc::state_transport_ws tB2;
  cvc::state_cluster_shard sB2(aB2, "C", "B2");
  sB2.attach();
  tB2.register_shard(&sB2);
  tB2.set_device_id(dev_b);
  tB2.set_device_signer(cvc::state_device_signer::derive(std::string(32, 'b'), scope));
  ASSERT_TRUE(tB2.connect_to_peer(ws(P.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(P, 1, std::chrono::milliseconds(3000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(P.total_devices_verified(), 0u);
  EXPECT_EQ(P.total_device_proofs_refused(), 0u) << "unchallenged, B2 sent no proof";

  for (cvc::state_transport_ws *t : {&tB, &tN, &tK, &tI, &tB2})
    t->stop();
  P.stop();
  S.stop();
}

// The order a relay must not depend on: a client's reader can answer the relay's challenge before
// the client's own Hello is written. A proof that comes first is held and checked when the Hello
// names the device. A proof answers one challenge only, and one for another device id is refused.
TEST(StateTransportWsTest, AProofBeforeTheHelloIsCheckedWhenItComes) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  cvc::app aS;
  cvc::state_transport_ws S;
  S.set_device_pins(std::make_shared<cvc::state_device_pins>());
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::state_cluster_shard sS(aS, "C", "S");
  sS.attach();
  S.register_shard(&sS);
  const auto key = cvc::state_device_signer::derive(std::string(32, 'r'), "raw");
  ASSERT_NE(key, nullptr);
  const std::string dev = "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr";

  raw_ws_client R(S.listen_address());
  const std::string nonce = pb_get(pb_get(R.read(), 1), 4); // the relay's Hello comes first
  ASSERT_EQ(nonce.size(), cvc::kDeviceNonceBytes);
  const std::string proof =
      proof_frame(key->public_key(), key->sign(cvc::device_proof_message(nonce, "S", "R", dev)));
  R.write(proof);
  R.write(hello_frame("R", "C", dev));
  ASSERT_TRUE(
      wait_until([&] { return S.total_devices_verified() == 1; }, std::chrono::milliseconds(3000)))
      << "the early proof is checked once the Hello comes";
  EXPECT_EQ(S.total_device_proofs_refused(), 0u);
  R.write(proof); // the same proof again
  EXPECT_TRUE(wait_until([&] { return S.total_device_proofs_refused() == 1; },
                         std::chrono::milliseconds(3000)))
      << "a challenge is answered once";
  bool verified = false;
  for (const cvc::state_link_report &l : S.link_reports())
    if (l.remote_node_id == "R")
      verified = l.device_verified;
  EXPECT_TRUE(verified) << "and the replay does not undo the proof";

  raw_ws_client Q(S.listen_address());
  const std::string nonce_q = pb_get(pb_get(Q.read(), 1), 4);
  ASSERT_EQ(nonce_q.size(), cvc::kDeviceNonceBytes);
  Q.write(hello_frame("Q", "C", "qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq"));
  Q.write(proof_frame(key->public_key(), key->sign(cvc::device_proof_message(
                                             nonce_q, "S", "Q", "another-device-id"))));
  EXPECT_TRUE(wait_until([&] { return S.total_device_proofs_refused() == 2; },
                         std::chrono::milliseconds(3000)))
      << "a proof signed for another device id";
  EXPECT_EQ(S.total_devices_verified(), 1u);
  S.stop();
}

// The inbound gate's rule in these tests: a peer may write only below x.<its node id>.
class own_prefix_gate final : public cvc::state_inbound_gate {
public:
  bool admit(const std::string &node_id, const cvc::state_mutation &m) const override {
    return cvc::state_peer_registry::prefix_matches("x." + node_id, m.path);
  }
  bool admit(const std::string &node_id, const cvc::state_message &m) const override {
    return cvc::state_peer_registry::prefix_matches("x." + node_id, m.path);
  }
};

// Connection-bound identity (as
// StateTransportGrpcTest.InboundGateTakesOnlyWhatAPeerMayWriteAsItself): a write to another peer's
// path, a mutation or media frame naming another origin, and a snapshot pushed at the relay are
// dropped -- not applied, not forwarded -- and counted.
TEST(StateTransportWsTest, InboundGateTakesOnlyWhatAPeerMayWriteAsItself) {
  cvc::app aS, aB, aD;
  cvc::state_transport_ws S, tB, tD;
  S.set_inbound_gate(std::make_shared<own_prefix_gate>());
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  std::atomic<int> media_as_b{0}, media_as_d{0};
  tD.set_on_media([&](const cvc::ws_media_frame &f) {
    (f.origin_node_id == "B" ? media_as_b : media_as_d).fetch_add(1);
  });
  // The relay fans media to its other connections (a media_router does this in arisrv).
  S.set_on_media([&](const cvc::ws_media_frame &f) { S.publish_media_frame(f); });

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sD(aD, "C", "D");
  sS.attach();
  sB.attach();
  sD.attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tD.register_shard(&sD);

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tD.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 2, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(tB, 1, std::chrono::milliseconds(2000)));

  cvc::ws_media_frame m;
  m.stream_id = "camera";
  m.codec = "mjpeg";
  m.width = 64;
  m.height = 48;
  m.payload = {0xff, 0xd8, 0x01, 0xff, 0xd9};
  m.origin_node_id = "D";
  m.seq = 1;
  tB.publish_media_frame(m);                              // as D
  tB.publish(value_mutation("B", 1, "x.D.k", "b"));       // D's path
  tB.publish(value_mutation("D", 2, "x.D.k", "spoofed")); // as D
  cvc::state_message msg;                                 // a message to D's path
  msg.cluster_id = "C";
  msg.origin_node_id = "B";
  msg.message_id = "B:1";
  msg.path = "x.D.inbox";
  msg.content_type = cvc::state_message::MIME_TEXT;
  msg.string_value = "hi";
  tB.publish_message(msg);
  EXPECT_EQ(tB.push_snapshot_to("S", {entry("x.B.pushed", "1")}), 1u);
  m.origin_node_id = "B";
  m.seq = 2;
  tB.publish_media_frame(m);                          // its own
  tB.publish(value_mutation("B", 3, "x.B.k", "own")); // its own, after the rest
  EXPECT_TRUE(
      wait_until([&] { return value_at(aD, "x.B.k") == "own"; }, std::chrono::milliseconds(3000)))
      << "its own path is taken and forwarded";
  EXPECT_TRUE(wait_until([&] { return media_as_b.load() == 1; }, std::chrono::milliseconds(3000)))
      << "...and so is its own media";
  EXPECT_EQ(value_at(aS, "x.B.k"), "own");
  EXPECT_EQ(value_at(aS, "x.D.k"), "") << "another peer's path, or as another peer";
  EXPECT_EQ(value_at(aD, "x.D.k"), "") << "...is not forwarded either";
  EXPECT_EQ(value_at(aS, "x.B.pushed"), "") << "a client's snapshot is not applied";
  EXPECT_EQ(media_as_d.load(), 0) << "media as another peer is not fanned";
  EXPECT_EQ(S.total_inbound_rejected(), 5u);

  tB.stop();
  tD.stop();
  S.stop();
}

// One live connection at a time holds a node id (as StateTransportGrpcTest.
// InboundGateBindsANodeIdToOneStream): a second connection claiming B is not B until the first
// closes, and only the holder's close is B leaving.
TEST(StateTransportWsTest, InboundGateBindsANodeIdToOneConnection) {
  cvc::app aS, aB, aB2, aB3;
  cvc::state_transport_ws S, tB, tB2, tB3;
  auto gate = std::make_shared<own_prefix_gate>();
  S.set_inbound_gate(gate);
  std::mutex mu;
  std::vector<std::string> closed;
  S.set_on_connection_closed([&](const std::string &id) {
    std::lock_guard<std::mutex> lk(mu);
    closed.push_back(id);
  });
  const auto closed_now = [&] {
    std::lock_guard<std::mutex> lk(mu);
    return closed;
  };
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sB2(aB2, "C", "B");
  cvc::state_cluster_shard sB3(aB3, "C", "B");
  for (cvc::state_cluster_shard *s : {&sS, &sB, &sB2, &sB3})
    s->attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tB2.register_shard(&sB2);
  tB3.register_shard(&sB3);

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return gate->holder("B") != nullptr; }, std::chrono::milliseconds(3000)))
      << "B is bound at its Hello";
  ASSERT_TRUE(tB2.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 2, std::chrono::milliseconds(3000)));

  tB2.publish(value_mutation("B", 20, "x.B.second", "2"));
  tB.publish(value_mutation("B", 10, "x.B.first", "1"));
  EXPECT_TRUE(wait_until([&] { return value_at(aS, "x.B.first") == "1"; },
                         std::chrono::milliseconds(3000)));
  EXPECT_TRUE(
      wait_until([&] { return S.total_inbound_rejected() == 1; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(value_at(aS, "x.B.second"), "") << "a second connection claiming B is not B";

  tB2.stop();
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() == 1; }, std::chrono::milliseconds(3000)));
  tB.stop();
  EXPECT_TRUE(
      wait_until([&] { return gate->holder("B") == nullptr; }, std::chrono::milliseconds(3000)))
      << "freed when its holder closes";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(closed_now(), std::vector<std::string>{"B"}) << "only the holder's close is B leaving";

  ASSERT_TRUE(tB3.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  tB3.publish(value_mutation("B", 30, "x.B.third", "3"));
  EXPECT_TRUE(
      wait_until([&] { return value_at(aS, "x.B.third") == "3"; }, std::chrono::milliseconds(3000)))
      << "a free node id is claimed again";

  tB3.stop();
  S.stop();
}

// A client naming the relay's own node id never speaks (as StateTransportGrpcTest.
// InboundGateRefusesTheRelaysOwnId).
TEST(StateTransportWsTest, InboundGateRefusesTheRelaysOwnId) {
  cvc::app aS, aE, aB;
  cvc::state_transport_ws S, tE, tB;
  auto gate = std::make_shared<own_prefix_gate>();
  S.set_inbound_gate(gate);
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sE(aE, "C", "S"); // claims the relay's id
  cvc::state_cluster_shard sB(aB, "C", "B");
  for (cvc::state_cluster_shard *s : {&sS, &sE, &sB})
    s->attach();
  S.register_shard(&sS);
  tE.register_shard(&sE);
  tB.register_shard(&sB);

  ASSERT_TRUE(tE.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 2, std::chrono::milliseconds(3000)));
  tE.publish(value_mutation("S", 1000, "x.S.k", "forged"));
  EXPECT_TRUE(
      wait_until([&] { return S.total_inbound_rejected() == 1; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(gate->holder("S"), nullptr) << "the relay's id is never bound";

  S.publish(value_mutation("S", 1000, "x.S.real", "1"));
  EXPECT_TRUE(
      wait_until([&] { return value_at(aB, "x.S.real") == "1"; }, std::chrono::milliseconds(3000)))
      << "nothing went out in the relay's name";
  EXPECT_EQ(value_at(aB, "x.S.k"), "");

  tE.stop();
  tB.stop();
  S.stop();
}

// A silent holder's id goes to the next claim (as StateTransportGrpcTest.
// InboundGateHandsASilentHoldersIdOver).
TEST(StateTransportWsTest, InboundGateHandsASilentHoldersIdOver) {
  cvc::app aS, aB, aB2;
  cvc::state_transport_ws S, tB, tB2;
  auto gate = std::make_shared<own_prefix_gate>();
  gate->set_takeover_after(std::chrono::milliseconds(1000));
  S.set_inbound_gate(gate);
  // No probes: a holder that sends nothing is silent.
  S.set_ping_interval(std::chrono::minutes(10));
  tB.set_ping_interval(std::chrono::minutes(10));
  std::mutex mu;
  std::vector<std::string> closed;
  S.set_on_connection_closed([&](const std::string &id) {
    std::lock_guard<std::mutex> lk(mu);
    closed.push_back(id);
  });
  const auto closed_now = [&] {
    std::lock_guard<std::mutex> lk(mu);
    return closed;
  };
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);

  cvc::state_cluster_shard sS(aS, "C", "S");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sB2(aB2, "C", "B");
  for (cvc::state_cluster_shard *s : {&sS, &sB, &sB2})
    s->attach();
  S.register_shard(&sS);
  tB.register_shard(&sB);
  tB2.register_shard(&sB2);

  ASSERT_TRUE(tB.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return gate->holder("B") != nullptr; }, std::chrono::milliseconds(3000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // B is silent past the takeover
  ASSERT_TRUE(tB2.connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  EXPECT_TRUE(
      wait_until([&] { return gate->total_takeovers() == 1; }, std::chrono::milliseconds(3000)))
      << "the second claim takes the silent holder's id";

  tB2.publish(value_mutation("B", 20, "x.B.new", "2"));
  EXPECT_TRUE(
      wait_until([&] { return value_at(aS, "x.B.new") == "2"; }, std::chrono::milliseconds(3000)));
  tB.publish(value_mutation("B", 10, "x.B.old", "1"));
  EXPECT_TRUE(
      wait_until([&] { return S.total_inbound_rejected() == 1; }, std::chrono::milliseconds(3000)))
      << "the displaced connection speaks no more";
  EXPECT_EQ(value_at(aS, "x.B.old"), "");

  tB.stop();
  ASSERT_TRUE(
      wait_until([&] { return S.connection_count() == 1; }, std::chrono::milliseconds(3000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(closed_now().empty()) << "the displaced connection's close is not B leaving";
  tB2.stop();
  EXPECT_TRUE(wait_until([&] { return closed_now() == std::vector<std::string>{"B"}; },
                         std::chrono::milliseconds(3000)))
      << "the holder's close is";

  S.stop();
}

#if defined(CVC_ENABLE_GRPC)
// The WS<->gRPC bridge (the wasm-interop payoff): a relay runs a gRPC acceptor AND a WebSocket
// acceptor over the SAME shard, bridged both ways. A native WS client and a native gRPC client each
// reach the other — exactly what a browser (WS) client and a native (gRPC) client will do in one
// room, which is the whole point of carrying the identical Frame wire on both transports.
// One inbound gate on a relay's two transports: a pid held over gRPC cannot be claimed over WS
// until its gRPC stream closes.
TEST(StateTransportWsTest, InboundGateSpansBothTransports) {
  cvc::app aR, aG, aW;
  cvc::state_transport_grpc G, gB;
  cvc::state_transport_ws W, wB;
  auto gate = std::make_shared<own_prefix_gate>();
  G.set_inbound_gate(gate);
  W.set_inbound_gate(gate);
  G.start("127.0.0.1:0", "relay", "C");
  W.start("127.0.0.1:0", "relay", "C");
  G.set_relay(true);
  W.set_relay(true);
  cvc::state_cluster_shard sR(aR, "C", "relay");
  sR.attach();
  G.register_shard(&sR);
  W.register_shard(&sR);

  cvc::state_cluster_shard sG(aG, "C", "B");
  cvc::state_cluster_shard sW(aW, "C", "B");
  sG.attach();
  sW.attach();
  gB.start("127.0.0.1:0", "B", "C");
  gB.register_shard(&sG);
  wB.register_shard(&sW);
  ASSERT_TRUE(gB.connect_to_peer(G.listen_address(), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
      wait_until([&] { return gate->holder("B") != nullptr; }, std::chrono::milliseconds(3000)));
  ASSERT_TRUE(wB.connect_to_peer(ws(W.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(W, 1, std::chrono::milliseconds(3000)));

  wB.publish(value_mutation("B", 20, "x.B.ws", "1"));
  EXPECT_TRUE(
      wait_until([&] { return W.total_inbound_rejected() == 1; }, std::chrono::milliseconds(3000)))
      << "B is held over gRPC";
  EXPECT_EQ(value_at(aR, "x.B.ws"), "");

  gB.stop();
  ASSERT_TRUE(
      wait_until([&] { return gate->holder("B") == nullptr; }, std::chrono::milliseconds(3000)));
  wB.publish(value_mutation("B", 21, "x.B.ws2", "2"));
  EXPECT_TRUE(
      wait_until([&] { return value_at(aR, "x.B.ws2") == "2"; }, std::chrono::milliseconds(3000)))
      << "free again once the gRPC stream closed";

  wB.stop();
  G.unregister_shard(&sR);
  W.unregister_shard(&sR);
  W.stop();
  G.stop();
}

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

// The media egress shaper is asked after the gate, once per identified receiver: one gets a
// variant (its own smaller copy, encoded once whatever number of receivers share it), one gets
// nothing, one the frame as sent; the relay's own uplink is never shaped, and each delivery and
// drop counts on that receiver's link (state_link_stats media counters).
TEST(StateTransportWsTest, RelayMediaShaperSendsEachReceiverItsShape) {
  std::mutex mu;
  std::vector<cvc::ws_media_frame> gotB, gotD;
  std::atomic<int> gotC{0};
  cvc::app aA, aB, aC, aD;
  cvc::state_transport_ws S, tA, tB, tC, tD;
  S.start("127.0.0.1:0", "S", "C");
  S.set_relay(true);
  cvc::media_router mr;
  mr.add_endpoint(&S);
  auto small = std::make_shared<cvc::media_frame>();
  small->origin_node_id = "A";
  small->stream_id = "camera";
  small->codec = "mjpeg";
  small->width = 80;
  small->height = 60;
  small->seq = 1;
  small->payload = {0xff, 0xd8, 0xff, 0xd9};
  S.set_media_shaper([&](const cvc::media_frame &, const std::string &receiver) {
    cvc::media_shape s;
    if (receiver == "B")
      s.variant = small;
    else if (receiver == "C1")
      s.drop = true;
    return s;
  });

  cvc::state_cluster_shard sA(aA, "C", "A"), sB(aB, "C", "B"), sC(aC, "C", "C1"), sD(aD, "C", "D");
  for (cvc::state_cluster_shard *s : {&sA, &sB, &sC, &sD})
    s->attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);
  tD.register_shard(&sD);
  tB.set_on_media([&](const cvc::ws_media_frame &m) {
    std::lock_guard<std::mutex> lk(mu);
    gotB.push_back(m);
  });
  tC.set_on_media([&](const cvc::ws_media_frame &) { gotC.fetch_add(1); });
  tD.set_on_media([&](const cvc::ws_media_frame &m) {
    std::lock_guard<std::mutex> lk(mu);
    gotD.push_back(m);
  });
  for (cvc::state_transport_ws *t : {&tA, &tB, &tC, &tD})
    ASSERT_TRUE(t->connect_to_peer(ws(S.listen_address()), std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_identified(S, 4, std::chrono::milliseconds(2000)));

  cvc::ws_media_frame m;
  m.origin_node_id = "A";
  m.stream_id = "camera";
  m.codec = "mjpeg";
  m.width = 320;
  m.height = 240;
  m.seq = 1;
  m.payload.assign(2000, 0x5a);
  tA.publish_media_frame(m);
  ASSERT_TRUE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !gotB.empty() && !gotD.empty();
      },
      std::chrono::milliseconds(3000)));
  ASSERT_TRUE(wait_until([&] { return S.total_media_shape_dropped() == 1; },
                         std::chrono::milliseconds(3000)));
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(gotB[0].width, 80u) << "B gets its variant";
    EXPECT_EQ(gotB[0].payload.size(), 4u);
    EXPECT_EQ(gotD[0].width, 320u) << "D gets the frame as sent";
    EXPECT_EQ(gotD[0].payload.size(), 2000u);
  }
  EXPECT_EQ(gotC.load(), 0) << "C gets nothing";
  EXPECT_EQ(S.total_media_shaped(), 1u);
  std::map<std::string, cvc::state_link_stats::counters> by;
  for (const cvc::state_link_report &l : S.link_reports())
    by[l.remote_node_id] = l.stats;
  EXPECT_EQ(by["B"].media_sent, 1u);
  EXPECT_EQ(by["D"].media_sent, 1u);
  EXPECT_EQ(by["C1"].media_sent, 0u) << "a shaped-out frame is not a delivery";

  for (cvc::state_transport_ws *t : {&tA, &tB, &tC, &tD})
    t->stop();
  S.stop();
}

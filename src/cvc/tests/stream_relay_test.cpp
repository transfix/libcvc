/*
  Copyright 2026 The University of Texas at Austin

  Tests for cvc::ariadne::stream::stream_relay — the conference SFU fan-out core
  (Phase 0). One ingest per publisher, N egress to subscribers, over the gRPC
  StreamTransport. Built only under CVC_ENABLE_GRPC.
*/

#include <chrono>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/core/app.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace cs = cvc::ariadne::stream;

namespace {

cs::format_desc rgba4x4() {
  cs::format_desc f;
  f.kind = cs::frame_kind::video_raw;
  f.codec = "rgba8";
  f.w = 4;
  f.h = 4;
  f.stride = 16;
  f.bytes = 64; // 4*4*4
  return f;
}

} // namespace

// relay() registers a proxy per id, is idempotent per id, and counts live
// streams — no network needed (the transport is never started).
TEST(StreamRelay, Registration) {
  cvc::app app;
  cvc::stream_transport_grpc transport;
  cs::stream_relay relay(app, transport);
  const cs::format_desc fmt = rgba4x4();

  cs::stream_params p0;
  p0.id = "cam0";
  p0.format = fmt;
  const std::string t0 = relay.relay(p0);
  ASSERT_FALSE(t0.empty());

  cs::stream_params p1;
  p1.id = "cam1";
  p1.format = fmt;
  const std::string t1 = relay.relay(p1);
  ASSERT_FALSE(t1.empty());
  EXPECT_NE(t0, t1);
  EXPECT_EQ(relay.stream_count(), 2u);

  // Idempotent: relaying the same id again returns the same token, no new proxy.
  cs::stream_params p0again;
  p0again.id = "cam0";
  p0again.format = fmt;
  EXPECT_EQ(relay.relay(p0again), t0);
  EXPECT_EQ(relay.stream_count(), 2u);

  // An unsized format is rejected.
  cs::stream_params bad;
  bad.id = "bad";
  EXPECT_TRUE(relay.relay(bad).empty());
  EXPECT_EQ(relay.stream_count(), 2u);

  relay.stop();
}

// Two participants both publish id "camera" under their OWN scope (root_path = the pid). The relay
// must key each proxy by its (root_path, id) IDENTITY — the token — not by the id alone, or the
// second participant collides onto the first and is silently dropped. This is the exact conference
// SFU bug: with id-only keying the server relayed just 2 of 4 streams (A's), and every peer's pull
// of B's media had no egress. No network needed.
TEST(StreamRelay, ScopedSameIdNoCollision) {
  cvc::app app;
  cvc::stream_transport_grpc transport;
  cs::stream_relay relay(app, transport);
  const cs::format_desc fmt = rgba4x4();

  cs::stream_params a;
  a.id = "camera";
  a.root_path = "A";
  a.format = fmt;
  const std::string ta = relay.relay(a);
  ASSERT_FALSE(ta.empty());

  cs::stream_params b;
  b.id = "camera";   // SAME id as A...
  b.root_path = "B"; // ...but a different scope (the other participant)
  b.format = fmt;
  const std::string tb = relay.relay(b);
  ASSERT_FALSE(tb.empty());

  EXPECT_NE(ta, tb) << "same id in different scopes must yield distinct tokens";
  EXPECT_EQ(relay.stream_count(), 2u) << "both scoped streams must each get their own proxy";

  // Idempotent per (scope, id): re-relaying A's camera returns A's token and adds no proxy.
  cs::stream_params aAgain;
  aAgain.id = "camera";
  aAgain.root_path = "A";
  aAgain.format = fmt;
  EXPECT_EQ(relay.relay(aAgain), ta);
  EXPECT_EQ(relay.stream_count(), 2u);

  relay.stop();
}

// The SFU fan-out: N publishers each serve a stream to the relay; M subscribers
// each pull all N; every one of the N*M paths delivers that publisher's frames
// (verified by a per-publisher payload signature). Over loopback gRPC.
TEST(StreamRelay, FanOutNxM) {
  const int N = 2; // publishers
  const int M = 3; // subscribers
  const cs::format_desc fmt = rgba4x4();

  // ── Relay: a started StreamTransport server + the relay (built BEFORE start) ──
  cvc::app relayApp;
  cvc::stream_transport_grpc relayTransport;
  cs::stream_relay relay(relayApp, relayTransport);
  relayTransport.start("127.0.0.1:0");
  ASSERT_TRUE(relayTransport.is_running());
  const std::string relayEndpoint = relayTransport.listen_address();
  ASSERT_FALSE(relayEndpoint.empty());

  // Register a proxy per publisher, each sized for M pulling subscribers.
  std::vector<std::string> tokens;
  for (int i = 0; i < N; ++i) {
    cs::stream_params p;
    p.id = "pub" + std::to_string(i);
    p.format = fmt;
    p.expected_subscribers = M; // the relay proxy's pool must cover M egress subs
    p.subscriber_depth = 8;
    const std::string tok = relay.relay(p);
    ASSERT_FALSE(tok.empty()) << "relay.relay failed for publisher " << i;
    tokens.push_back(tok);
  }
  ASSERT_EQ(relay.stream_count(), static_cast<std::size_t>(N));

  // ── Publishers: own app + client transport + bridge; serve the stream in. ──
  std::vector<std::unique_ptr<cvc::app>> pubApps;
  std::vector<std::unique_ptr<cvc::stream_transport_grpc>> pubTransports;
  std::vector<std::unique_ptr<cs::stream_bridge>> pubBridges;
  std::vector<std::unique_ptr<cs::stream>> pubStreams;
  for (int i = 0; i < N; ++i) {
    pubApps.emplace_back(new cvc::app());
    pubTransports.emplace_back(new cvc::stream_transport_grpc());
    pubBridges.emplace_back(new cs::stream_bridge(*pubApps[i], *pubTransports[i]));
    cs::stream_params p;
    p.id = "pub" + std::to_string(i);
    p.format = fmt;
    std::unique_ptr<cs::stream> s = cs::stream::open(*pubApps[i], p);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->token(), tokens[i]); // same (root,id) → token lines up with the relay proxy
    ASSERT_TRUE(pubBridges[i]->serve(*s, relayEndpoint));
    pubStreams.push_back(std::move(s));
  }

  // ── Subscribers: own app + client transport + bridge; pull EVERY publisher. ──
  std::vector<std::unique_ptr<cvc::app>> subApps;
  std::vector<std::unique_ptr<cvc::stream_transport_grpc>> subTransports;
  std::vector<std::unique_ptr<cs::stream_bridge>> subBridges;
  std::vector<std::vector<std::unique_ptr<cs::stream>>> subProxies(M);
  std::vector<std::vector<std::shared_ptr<cs::subscription>>> subSubs(M);
  for (int m = 0; m < M; ++m) {
    subApps.emplace_back(new cvc::app());
    subTransports.emplace_back(new cvc::stream_transport_grpc());
    subBridges.emplace_back(new cs::stream_bridge(*subApps[m], *subTransports[m]));
    for (int i = 0; i < N; ++i) {
      cs::stream_params p;
      p.id = "pub" + std::to_string(i);
      p.format = fmt;
      p.subscriber_depth = 8;
      std::unique_ptr<cs::stream> proxy = cs::stream::open(*subApps[m], p);
      ASSERT_NE(proxy, nullptr);
      std::shared_ptr<cs::subscription> sub = proxy->channel().subscribe(cs::deliver_mode::ring, 8);
      ASSERT_NE(sub, nullptr);
      ASSERT_TRUE(subBridges[m]->pull_from(relayEndpoint, *proxy));
      subProxies[m].push_back(std::move(proxy));
      subSubs[m].push_back(sub);
    }
  }

  // Produce continuously (so an async egress subscription that isn't ready yet
  // just misses early frames and catches later ones), draining until every
  // N*M path has delivered a frame carrying its publisher's signature byte.
  std::vector<std::vector<bool>> got(M, std::vector<bool>(N, false));
  int remaining = M * N;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  int seq = 0;
  while (remaining > 0 && std::chrono::steady_clock::now() < deadline) {
    for (int i = 0; i < N; ++i) {
      auto buf = std::make_shared<std::vector<std::uint8_t>>(64, static_cast<std::uint8_t>(i + 1));
      pubStreams[i]->channel().publish_external(buf->data(), buf->size(), 0.01 * seq, buf);
    }
    ++seq;
    for (int m = 0; m < M; ++m) {
      for (int i = 0; i < N; ++i) {
        if (got[m][i])
          continue;
        cs::frame_ptr f;
        if (subSubs[m][i]->try_pop(f) && f && f->size == 64 &&
            f->data[0] == static_cast<std::uint8_t>(i + 1)) {
          got[m][i] = true;
          --remaining;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  for (int m = 0; m < M; ++m)
    for (int i = 0; i < N; ++i)
      EXPECT_TRUE(got[m][i]) << "subscriber " << m << " never received publisher " << i;
  EXPECT_EQ(remaining, 0);
  EXPECT_GE(relay.frames_forwarded(), static_cast<std::uint64_t>(M * N));

  // Teardown: publishers stop pushing; subscribers stop pulling; the relay stops
  // its bridge (closes egress subscriptions); THEN the transports stop; THEN the
  // streams drop. close() TryCancel()s, so a subscriber leaving while the relay
  // is still serving unblocks cleanly regardless of order.
  for (auto &b : pubBridges)
    b->stop();
  for (auto &b : subBridges)
    b->stop();
  relay.stop();
  for (auto &t : pubTransports)
    t->stop();
  for (auto &t : subTransports)
    t->stop();
  relayTransport.stop();
}

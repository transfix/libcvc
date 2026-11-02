/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The conference's fast lane: the datagram codecs (fast_lane.h, room_lane.h), the pose stream's
// acked mode that the pose lane runs (pose_stream.h) and room_pose_relay's routing onto the lane,
// and -- in a build with CVC_ENABLE_RTC -- real WebRTC lanes between a relay and clients on
// loopback (fast_lane_rtc.h).

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cvc/ariadne/conference/room_lane.h>
#include <cvc/ariadne/conference/room_pose_relay.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/pose_stream.h>
#include <cvc/net/fast_lane.h>
#include <cvc/net/fast_lane_rtc.h>
#include <cvc/state/state_transport.h>
#include <gtest/gtest.h>
#include <initializer_list>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace ca = cvc::ariadne;
namespace ra = cvc::ariadne::conference;

cvc::physics::body_state at(double x, double y) {
  cvc::physics::body_state s;
  s.position = {x, y, 0.0};
  s.linear_velocity = {1.5, 0.0, 0.0};
  s.active = true;
  return s;
}

std::vector<ca::pose_subject> subjects(std::initializer_list<std::pair<const char *, double>> xs) {
  std::vector<ca::pose_subject> out;
  for (const auto &[pid, x] : xs)
    out.push_back(ca::pose_subject{pid, at(x, 0.0), true});
  return out;
}

double x_of(const std::vector<ca::pose_sample> &samples, const std::string &pid) {
  for (const ca::pose_sample &s : samples)
    if (s.pid == pid)
      return s.state.position.x;
  return -1.0;
}

} // namespace

TEST(FastLaneCodecTest, MediaRoundTrips) {
  cvc::media_frame f;
  f.origin_node_id = "alice";
  f.stream_id = "camera";
  f.codec = "mjpeg";
  f.width = 320;
  f.height = 240;
  f.seq = 77;
  f.pts_seconds = 12.5;
  f.payload = {1, 2, 3, 250};
  const std::vector<std::uint8_t> d = cvc::encode_lane_media(f);
  ASSERT_FALSE(d.empty());
  EXPECT_EQ(cvc::lane_kind_of(d.data(), d.size()),
            static_cast<std::uint8_t>(cvc::lane_kind::media));
  cvc::media_frame g;
  ASSERT_TRUE(cvc::decode_lane_media(d.data(), d.size(), g));
  EXPECT_EQ(g.origin_node_id, "alice");
  EXPECT_EQ(g.stream_id, "camera");
  EXPECT_EQ(g.codec, "mjpeg");
  EXPECT_EQ(g.width, 320u);
  EXPECT_EQ(g.height, 240u);
  EXPECT_EQ(g.seq, 77);
  EXPECT_DOUBLE_EQ(g.pts_seconds, 12.5);
  EXPECT_EQ(g.payload, f.payload);
  // A truncated header is refused; so is a frame past the lane's largest datagram.
  EXPECT_FALSE(cvc::decode_lane_media(d.data(), 10, g));
  f.payload.assign(cvc::kLaneMaxDatagram, 0);
  EXPECT_TRUE(cvc::encode_lane_media(f).empty());
}

TEST(FastLaneCodecTest, ProbesAndAcksRoundTrip) {
  const auto ping = cvc::encode_lane_probe(cvc::lane_kind::ping, 9, -123456789);
  cvc::lane_kind k{};
  std::uint64_t seq = 0;
  std::int64_t t = 0;
  ASSERT_TRUE(cvc::decode_lane_probe(ping.data(), ping.size(), k, seq, t));
  EXPECT_EQ(k, cvc::lane_kind::ping);
  EXPECT_EQ(seq, 9u);
  EXPECT_EQ(t, -123456789);
  const auto ack = ra::encode_lane_pose_ack(4242);
  std::uint32_t s = 0;
  ASSERT_TRUE(ra::decode_lane_pose_ack(ack.data(), ack.size(), s));
  EXPECT_EQ(s, 4242u);
  EXPECT_FALSE(cvc::decode_lane_probe(ack.data(), ack.size(), k, seq, t));
}

// The pose lane's datagram is the reliable transports' PoseSnapshot message behind the kind byte.
TEST(FastLaneCodecTest, PoseSnapshotDatagramRoundTrips) {
  ca::pose_stream_encoder enc;
  const cvc::pose_snapshot snap = enc.encode(120, subjects({{"alice", 1.25}, {"bob", -3.0}}), 60);
  const std::vector<std::uint8_t> d = ra::encode_lane_pose_snapshot(snap);
#if !defined(CVC_ENABLE_WS) && !defined(CVC_ENABLE_GRPC)
  EXPECT_TRUE(d.empty()); // no PoseSnapshot message in this build: no lane carries poses
#else
  ASSERT_FALSE(d.empty());
  EXPECT_EQ(cvc::lane_kind_of(d.data(), d.size()), static_cast<std::uint8_t>(cvc::lane_kind::pose));
  cvc::pose_snapshot back;
  ASSERT_TRUE(ra::decode_lane_pose_snapshot(d.data(), d.size(), back));
  EXPECT_EQ(back.seq, snap.seq);
  EXPECT_EQ(back.base_seq, 0u);
  EXPECT_EQ(back.server_tick, 120u);
  ASSERT_EQ(back.entries.size(), 2u);
  EXPECT_EQ(back.entries[0].pid, "alice");
  EXPECT_EQ(back.entries[0].x, 1250);
  const auto ack = ra::encode_lane_pose_ack(back.seq); // another kind is not a snapshot
  EXPECT_FALSE(ra::decode_lane_pose_snapshot(ack.data(), ack.size(), back));
#endif
}

// ACKED encoder + multi-base decoder: snapshots in flight all differ from the last acknowledged
// one, so any of them decodes on its own, in any order, and a lost one costs nothing.
TEST(PoseStreamAckedTest, ALostSnapshotIsSupersededNotWaitedFor) {
  ca::pose_stream_encoder enc;
  enc.set_acked(true);
  ca::pose_stream_decoder dec;
  dec.set_bases(ra::kLanePoseBases);
  std::vector<ca::pose_sample> out;
  // 1: the keyframe, delivered and acked.
  const cvc::pose_snapshot s1 = enc.encode(10, subjects({{"alice", 1.0}, {"bob", 5.0}}), 60);
  EXPECT_EQ(s1.base_seq, 0u);
  ASSERT_TRUE(dec.decode(s1, out));
  EXPECT_TRUE(enc.ack(s1.seq));
  // 2 is lost; 3 differs from 1 (the acked base), not from 2.
  const cvc::pose_snapshot s2 = enc.encode(12, subjects({{"alice", 2.0}, {"bob", 5.0}}), 60);
  const cvc::pose_snapshot s3 = enc.encode(14, subjects({{"alice", 3.0}, {"bob", 5.0}}), 60);
  EXPECT_EQ(s2.base_seq, s1.seq);
  EXPECT_EQ(s3.base_seq, s1.seq);
  EXPECT_NE(s2.seq, s3.seq);
  out.clear();
  ASSERT_TRUE(dec.decode(s3, out));
  EXPECT_NEAR(x_of(out, "alice"), 3.0, 1e-3);
  EXPECT_TRUE(enc.ack(s3.seq));
  // 2 arrives late: it decodes (its base is kept) but gives no samples (older than 3).
  out.clear();
  EXPECT_TRUE(dec.decode(s2, out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(enc.ack(s2.seq)); // never moves the base back
  // 4 differs from 3, the newest acked.
  const cvc::pose_snapshot s4 = enc.encode(16, subjects({{"alice", 4.0}, {"bob", 5.0}}), 60);
  EXPECT_EQ(s4.base_seq, s3.seq);
  out.clear();
  ASSERT_TRUE(dec.decode(s4, out));
  EXPECT_NEAR(x_of(out, "alice"), 4.0, 1e-3);
  EXPECT_EQ(dec.gaps(), 0u);
}

// A subject that moved once and stopped is resent until a snapshot carrying it is acked; then the
// stream goes quiet (only heartbeats).
TEST(PoseStreamAckedTest, AChangeIsResentUntilAcknowledged) {
  ca::pose_stream_encoder enc;
  enc.set_acked(true);
  const cvc::pose_snapshot s1 = enc.encode(10, subjects({{"alice", 1.0}}), 1000);
  EXPECT_TRUE(enc.ack(s1.seq));
  const cvc::pose_snapshot s2 = enc.encode(12, subjects({{"alice", 2.0}}), 1000);
  EXPECT_EQ(s2.entries.size(), 1u);
  // s2 lost; still standing at 2.0: the difference goes out again.
  const cvc::pose_snapshot s3 = enc.encode(14, subjects({{"alice", 2.0}}), 1000);
  EXPECT_EQ(s3.entries.size(), 1u);
  EXPECT_TRUE(enc.ack(s3.seq));
  const cvc::pose_snapshot s4 = enc.encode(16, subjects({{"alice", 2.0}}), 1000);
  EXPECT_TRUE(s4.entries.empty());
  EXPECT_TRUE(s4.gone.empty());
}

// A snapshot whose base the client no longer has is refused (a gap) until a keyframe; a reset
// keyframe restarts both ends.
TEST(PoseStreamAckedTest, AnUnknownBaseIsAGapUntilTheKeyframe) {
  ca::pose_stream_encoder enc;
  enc.set_acked(true);
  ca::pose_stream_decoder dec;
  dec.set_bases(4);
  std::vector<ca::pose_sample> out;
  const cvc::pose_snapshot s1 = enc.encode(10, subjects({{"alice", 1.0}}), 60);
  EXPECT_TRUE(enc.ack(s1.seq)); // acked, but the client never decoded it (a lost-ack race)
  const cvc::pose_snapshot s2 = enc.encode(12, subjects({{"alice", 2.0}}), 60);
  EXPECT_FALSE(dec.decode(s2, out));
  EXPECT_EQ(dec.gaps(), 1u);
  enc.reset(); // the relay's periodic keyframe
  const cvc::pose_snapshot k = enc.encode(14, subjects({{"alice", 3.0}}), 60);
  EXPECT_EQ(k.base_seq, 0u);
  ASSERT_TRUE(dec.decode(k, out));
  EXPECT_NEAR(x_of(out, "alice"), 3.0, 1e-3);
  EXPECT_FALSE(enc.ack(s2.seq)); // from before the restart
  EXPECT_TRUE(enc.ack(k.seq));
}

// Until a keyframe's ack comes back, every snapshot is a keyframe too (base 0), and the relay may
// take the ack of any of them for the base: the client keeps them all. A NEW encoder (seqs that go
// back) is a different stream, and the client forgets the old one's bases.
TEST(PoseStreamAckedTest, AKeyframeKeepsTheBasesItsAcksMayName) {
  ca::pose_stream_encoder enc;
  enc.set_acked(true);
  ca::pose_stream_decoder dec;
  dec.set_bases(ra::kLanePoseBases);
  std::vector<ca::pose_sample> out;
  const cvc::pose_snapshot k1 = enc.encode(10, subjects({{"alice", 1.0}}), 60);
  const cvc::pose_snapshot k2 = enc.encode(12, subjects({{"alice", 1.5}}), 60);
  EXPECT_EQ(k1.base_seq, 0u);
  EXPECT_EQ(k2.base_seq, 0u); // no ack yet
  ASSERT_TRUE(dec.decode(k1, out));
  ASSERT_TRUE(dec.decode(k2, out));
  EXPECT_TRUE(enc.ack(k1.seq)); // k2's ack is lost
  const cvc::pose_snapshot s3 = enc.encode(14, subjects({{"alice", 2.0}}), 60);
  EXPECT_EQ(s3.base_seq, k1.seq);
  out.clear();
  ASSERT_TRUE(dec.decode(s3, out));
  EXPECT_NEAR(x_of(out, "alice"), 2.0, 1e-3);
  EXPECT_EQ(dec.gaps(), 0u);

  ca::pose_stream_encoder fresh; // the relay forgot this viewer and met it again
  fresh.set_acked(true);
  const cvc::pose_snapshot f1 = fresh.encode(20, subjects({{"alice", 10.0}}), 60);
  EXPECT_LE(f1.seq, s3.seq);
  ASSERT_TRUE(dec.decode(f1, out));
  EXPECT_TRUE(fresh.ack(f1.seq));
  const cvc::pose_snapshot f2 = fresh.encode(22, subjects({{"alice", 11.0}}), 60);
  EXPECT_EQ(f2.base_seq, f1.seq);
  out.clear();
  ASSERT_TRUE(dec.decode(f2, out));
  EXPECT_NEAR(x_of(out, "alice"), 11.0, 1e-3) << "against the new stream's base, not the old seq";
}

// The reliable stream is unchanged: commit-on-write, and base_seq must be the last seq.
TEST(PoseStreamAckedTest, TheReliableStreamKeepsItsRules) {
  ca::pose_stream_encoder enc;
  ca::pose_stream_decoder dec;
  std::vector<ca::pose_sample> out;
  const cvc::pose_snapshot s1 = enc.encode(10, subjects({{"alice", 1.0}}), 60);
  enc.commit();
  ASSERT_TRUE(dec.decode(s1, out));
  const cvc::pose_snapshot s2 = enc.encode(12, subjects({{"alice", 2.0}}), 60);
  enc.commit();
  const cvc::pose_snapshot s3 = enc.encode(14, subjects({{"alice", 3.0}}), 60);
  EXPECT_EQ(s3.base_seq, s2.seq);
  EXPECT_FALSE(dec.decode(s3, out)); // s2 never applied
  EXPECT_FALSE(enc.ack(s2.seq));     // acks mean nothing to a reliable stream
}

namespace {

class capture_transport : public cvc::state_transport {
public:
  std::set<std::string> connected;
  std::vector<std::pair<std::string, cvc::pose_snapshot>> sent;
  void register_shard(cvc::state_cluster_shard *) override {}
  void unregister_shard(cvc::state_cluster_shard *) override {}
  publish_stats publish(const cvc::state_mutation &) override { return {}; }
  std::size_t pump_shard(cvc::state_cluster_shard &) override { return 0; }
  std::size_t pump_all() override { return 0; }
  void flush() override {}
  std::size_t push_pose_snapshot_to(const std::string &id, const cvc::pose_snapshot &s) override {
    if (!connected.count(id))
      return 0;
    sent.emplace_back(id, s);
    return 1;
  }
};

class fake_pose_lane : public ra::pose_snapshot_lane {
public:
  std::set<std::string> open;
  std::vector<std::pair<std::string, cvc::pose_snapshot>> sent;
  bool carries(const std::string &pid) const override { return open.count(pid) > 0; }
  bool send(const std::string &pid, const cvc::pose_snapshot &s) override {
    sent.emplace_back(pid, s);
    return true;
  }
};

double dist(const cvc::physics::vec3 &a, const cvc::physics::vec3 &b) {
  const cvc::physics::vec3 d = a - b;
  return std::sqrt(cvc::physics::dot(d, d));
}

} // namespace

// room_pose_relay sends a viewer whose pose lane is open its stream there, acked: every third
// snapshot is lost and none is waited for, the client stays within a millimetre of the server, and
// the viewer's connection carries none of them meanwhile. Opening and closing the lane each restart
// the stream with a keyframe on the new path.
TEST(RoomPoseRelayLaneTest, ALaneViewerGetsAnAckedStreamThere) {
  cvc::ariadne::holon server(cvc::ariadne::holon_role::server);
  server.add_participant("a", false, at(0.0, 0.0));
  server.add_participant("b", false, at(2.0, 0.0));
  server.set_intent("b", cvc::ariadne::avatar_intent{cvc::physics::vec3{3.0, 0.0, 0.0}});
  capture_transport tp;
  tp.connected = {"a", "b"};
  fake_pose_lane lane;
  ra::room_pose_relay relay;
  relay.add_transport(&tp);
  relay.set_lane(&lane);
  relay.set_rtt("a", 20.0);
  relay.set_rtt("b", 20.0);
  const double dt = server.clock().fixed_dt();
  ca::pose_stream_decoder tcp_dec, lane_dec;
  lane_dec.set_bases(ra::kLanePoseBases);
  std::uint64_t on_lane = 0, lost = 0, tcp_after = 0;
  bool lane_key = false, tcp_key = false;
  for (std::uint64_t step = 0; step < 60; ++step) {
    if (step == 10)
      lane.open.insert("a");
    if (step == 50)
      lane.open.erase("a");
    server.advance(2 * dt);
    tp.sent.clear();
    lane.sent.clear();
    relay.step(server, step, nullptr);
    for (const auto &[viewer, snap] : tp.sent) {
      if (viewer != "a")
        continue;
      EXPECT_FALSE(step >= 10 && step < 50) << "a's lane is open at step " << step;
      if (step >= 50 && tcp_after++ == 0)
        tcp_key = snap.base_seq == 0;
      std::vector<ca::pose_sample> out;
      EXPECT_TRUE(tcp_dec.decode(snap, out)) << "step " << step;
    }
    for (const auto &[viewer, snap] : lane.sent) {
      ASSERT_EQ(viewer, "a");
      if (on_lane++ == 0)
        lane_key = snap.base_seq == 0;
      if (on_lane % 3 == 2) {
        ++lost;
        continue;
      }
      std::vector<ca::pose_sample> out;
      ASSERT_TRUE(lane_dec.decode(snap, out)) << "step " << step;
      for (const ca::pose_sample &s : out)
        EXPECT_LE(dist(s.state.position, server.state(s.pid).position), 0.001)
            << s.pid << " at step " << step;
      relay.acknowledge("a", snap.seq);
    }
  }
  EXPECT_TRUE(lane_key) << "the lane's first snapshot is a keyframe";
  EXPECT_TRUE(tcp_key) << "and so is the connection's, once the lane closes";
  EXPECT_EQ(on_lane, 40u) << "60 Hz while the lane was open";
  EXPECT_GT(lost, 10u);
  EXPECT_GT(tcp_after, 0u);
  EXPECT_EQ(lane_dec.gaps(), 0u);
  const ra::room_pose_relay::stats st = relay.take_stats();
  EXPECT_EQ(st.lane, on_lane);
  EXPECT_GT(st.acks, 0u);
}

// ---------------- a real lane (CVC_ENABLE_RTC) ----------------

namespace {

template <class F> bool wait_for(F &&f, std::chrono::milliseconds limit) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!f()) {
    if (std::chrono::steady_clock::now() > end)
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

std::uint16_t test_port() {
  // Spread concurrent ctest processes over a range (each test binds its own port).
  return static_cast<std::uint16_t>(
      27800 + (std::chrono::steady_clock::now().time_since_epoch().count() / 1000) % 150);
}

} // namespace

TEST(FastLaneRtcTest, ALaneOpensAndCarriesDatagramsBothWays) {
  if (!cvc::fast_lane_rtc_available())
    GTEST_SKIP() << "built without CVC_ENABLE_RTC";
  cvc::fast_lane_server_config cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = test_port();
  cfg.probe_interval = std::chrono::milliseconds(100);
  cvc::fast_lane_server server(cfg);
  std::mutex mu;
  std::vector<std::vector<std::uint8_t>> at_server, at_client;
  server.set_on_datagram(cvc::lane_kind::pose_ack,
                         [&](const std::string &pid, const std::uint8_t *d, std::size_t n) {
                           std::lock_guard<std::mutex> lk(mu);
                           EXPECT_EQ(pid, "alice");
                           at_server.emplace_back(d, d + n);
                         });
  cvc::fast_lane_client *client_ptr = nullptr;
  cvc::fast_lane_client client([&](const std::string &offer) {
    server.offer("alice", offer, [&](const std::string &answer) { client_ptr->on_answer(answer); });
  });
  client_ptr = &client;
  client.set_on_datagram([&](const std::uint8_t *d, std::size_t n) {
    std::lock_guard<std::mutex> lk(mu);
    at_client.emplace_back(d, d + n);
  });
  client.start();
  ASSERT_TRUE(wait_for([&] { return client.open() && server.lane_open("alice"); },
                       std::chrono::seconds(10)));
  const auto ack = ra::encode_lane_pose_ack(5);
  ASSERT_TRUE(client.send(ack.data(), ack.size()));
  cvc::media_frame mf;
  mf.origin_node_id = "bob";
  mf.stream_id = "mic";
  mf.payload.assign(80, 3);
  const auto pose = cvc::encode_lane_media(mf); // any datagram the relay sends
  ASSERT_TRUE(server.send_datagram("alice", pose.data(), pose.size()));
  EXPECT_FALSE(server.send_datagram("nobody", pose.data(), pose.size()));
  ASSERT_TRUE(wait_for(
      [&] {
        std::lock_guard<std::mutex> lk(mu);
        return !at_server.empty() && !at_client.empty();
      },
      std::chrono::seconds(5)));
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(at_server[0], ack);
    EXPECT_EQ(at_client[0], pose);
  }
  // The server's pings come back as pongs: the lane has a round trip.
  ASSERT_TRUE(wait_for(
      [&] {
        const auto rs = server.link_reports();
        return rs.size() == 1 && rs[0].stats.rtt_ms >= 0.0;
      },
      std::chrono::seconds(5)));
  EXPECT_EQ(server.link_reports()[0].transport, "webrtc");
  EXPECT_EQ(server.link_reports()[0].remote_node_id, "alice");
  client.close();
  EXPECT_TRUE(wait_for([&] { return !server.lane_open("alice"); }, std::chrono::seconds(10)));
}

// A relay's pose and media lanes are two servers on ONE UDP port (libjuice's mux tells their ICE
// sessions apart), each with its own peer connection to the same client: both open, and each
// carries only its own datagrams.
TEST(FastLaneRtcTest, TwoServersShareOnePort) {
  if (!cvc::fast_lane_rtc_available())
    GTEST_SKIP() << "built without CVC_ENABLE_RTC";
  cvc::fast_lane_server_config cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = static_cast<std::uint16_t>(test_port() + 600);
  cvc::fast_lane_server pose_server(cfg), media_server(cfg);
  std::atomic<int> at_pose{0}, at_media{0};
  pose_server.set_on_datagram(
      cvc::lane_kind::pose_ack,
      [&](const std::string &, const std::uint8_t *, std::size_t) { ++at_pose; });
  media_server.set_on_datagram(
      cvc::lane_kind::pose_ack,
      [&](const std::string &, const std::uint8_t *, std::size_t) { ++at_media; });
  cvc::fast_lane_client *pp = nullptr, *mp = nullptr;
  cvc::fast_lane_client pose_client([&](const std::string &offer) {
    pose_server.offer("alice", offer, [&](const std::string &a) { pp->on_answer(a); });
  });
  cvc::fast_lane_client media_client([&](const std::string &offer) {
    media_server.offer("alice", offer, [&](const std::string &a) { mp->on_answer(a); });
  });
  pp = &pose_client;
  mp = &media_client;
  pose_client.start();
  media_client.start();
  ASSERT_TRUE(wait_for(
      [&] {
        return pose_client.open() && media_client.open() && pose_server.lane_open("alice") &&
               media_server.lane_open("alice");
      },
      std::chrono::seconds(10)));
  const auto ack = ra::encode_lane_pose_ack(1);
  ASSERT_TRUE(pose_client.send(ack.data(), ack.size()));
  ASSERT_TRUE(wait_for([&] { return at_pose.load() == 1; }, std::chrono::seconds(5)));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(at_media.load(), 0);
}

// A lane that goes quiet is closed by the relay, so its viewer falls back to the connection
// (the generation moves, and room_aoi_relay re-installs that viewer's filter).
TEST(FastLaneRtcTest, ASilentLaneIsClosed) {
  if (!cvc::fast_lane_rtc_available())
    GTEST_SKIP() << "built without CVC_ENABLE_RTC";
  cvc::fast_lane_server_config cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = static_cast<std::uint16_t>(test_port() + 400);
  cfg.probe_interval = std::chrono::milliseconds(50);
  cfg.idle_timeout = std::chrono::milliseconds(1); // nothing answers that fast
  cvc::fast_lane_server server(cfg);
  cvc::fast_lane_client *self = nullptr;
  cvc::fast_lane_client client([&](const std::string &offer) {
    server.offer("alice", offer, [&](const std::string &answer) { self->on_answer(answer); });
  });
  self = &client;
  std::atomic<bool> opened{false};
  client.set_on_state([&](bool open) { opened = opened || open; });
  client.start();
  ASSERT_TRUE(wait_for([&] { return opened.load(); }, std::chrono::seconds(10)));
  ASSERT_TRUE(
      wait_for([&] { return server.counters().idle_closed >= 1; }, std::chrono::seconds(5)));
  EXPECT_FALSE(server.lane_open("alice"));
}

TEST(FastLaneRtcTest, MediaFansThroughTheGateAndNeverBackToItsOrigin) {
  if (!cvc::fast_lane_rtc_available())
    GTEST_SKIP() << "built without CVC_ENABLE_RTC";
  cvc::fast_lane_server_config cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = static_cast<std::uint16_t>(test_port() + 200);
  cvc::fast_lane_server server(cfg);
  std::atomic<int> ingested{0};
  server.set_media_sink([&](const cvc::media_frame &f, const void *src) {
    EXPECT_EQ(f.origin_node_id, "alice");
    EXPECT_NE(src, nullptr);
    ++ingested;
  });
  server.set_media_gate(
      [](const cvc::media_frame &, const std::string &receiver) { return receiver != "carol"; });
  struct peer {
    std::unique_ptr<cvc::fast_lane_client> lane;
    std::atomic<int> media{0};
  };
  std::map<std::string, peer> peers;
  for (const char *pid : {"alice", "bob", "carol"}) {
    peer &p = peers[pid];
    p.lane = std::make_unique<cvc::fast_lane_client>([&server, pid, &p](const std::string &o) {
      server.offer(pid, o, [&p](const std::string &a) { p.lane->on_answer(a); });
    });
    p.lane->set_on_datagram([&p](const std::uint8_t *d, std::size_t n) {
      cvc::media_frame f;
      if (cvc::decode_lane_media(d, n, f))
        ++p.media;
    });
    p.lane->start();
  }
  ASSERT_TRUE(wait_for(
      [&] {
        for (auto &kv : peers)
          if (!kv.second.lane->open() || !server.lane_open(kv.first))
            return false;
        return true;
      },
      std::chrono::seconds(10)));
  cvc::media_frame f;
  f.origin_node_id = "alice";
  f.stream_id = "mic";
  f.codec = "opus";
  f.payload.assign(120, 7);
  const auto d = cvc::encode_lane_media(f);
  ASSERT_TRUE(peers["alice"].lane->send(d.data(), d.size()));
  ASSERT_TRUE(wait_for([&] { return ingested.load() == 1; }, std::chrono::seconds(5)));
  server.fan_media(f, nullptr); // as the router would, for a frame from another endpoint
  ASSERT_TRUE(wait_for([&] { return peers["bob"].media.load() == 1; }, std::chrono::seconds(5)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(peers["alice"].media.load(), 0); // its own frame
  EXPECT_EQ(peers["carol"].media.load(), 0); // gated
  EXPECT_EQ(server.counters().media_gated, 1u);
  // A lane may not speak as another participant.
  f.origin_node_id = "bob";
  const auto forged = cvc::encode_lane_media(f);
  peers["alice"].lane->send(forged.data(), forged.size());
  ASSERT_TRUE(wait_for([&] { return server.counters().refused >= 1; }, std::chrono::seconds(5)));
  EXPECT_EQ(ingested.load(), 1);
}

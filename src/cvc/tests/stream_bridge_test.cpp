/*
  Copyright 2026 The University of Texas at Austin

  The cross-node streaming capstone (Phase 10, PR9): a stream produced on one
  app's node is consumed on another, with frame bytes carried over the gRPC
  StreamTransport. Built only under CVC_ENABLE_GRPC.
*/

#include <chrono>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/app.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <gtest/gtest.h>
#include <memory>
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

// Frames published into a stream on node A arrive, byte-identical and in order,
// in a local proxy stream on node B — carried over the gRPC data plane.
TEST(StreamBridge, FramesCrossNodeToNode) {
  cvc::app aA, aB;
  const cs::format_desc fmt = rgba4x4();

  // ── Consumer B: a StreamTransport server, a bridge (constructed BEFORE the
  // transport starts so its router is installed), and a local proxy stream. ──
  cvc::stream_transport_grpc transportB;
  cs::stream_bridge bridgeB(aB, transportB);
  transportB.start("127.0.0.1:0");
  ASSERT_TRUE(transportB.is_running());

  cs::stream_params pB;
  pB.id = "cam0";
  pB.format = fmt;
  pB.subscriber_depth = 8; // size the proxy's pool for the depth-8 observer below
  std::unique_ptr<cs::stream> proxyB = cs::stream::open(aB, pB);
  ASSERT_NE(proxyB, nullptr);
  std::shared_ptr<cs::subscription> subB = proxyB->channel().subscribe(cs::deliver_mode::ring, 8);
  ASSERT_NE(subB, nullptr);
  bridgeB.receive_into(proxyB->token(), *proxyB);

  // ── Producer A: a local stream and a client-only transport + bridge. ──
  cvc::stream_transport_grpc transportA;
  cs::stream_bridge bridgeA(aA, transportA);
  cs::stream_params pA;
  pA.id = "cam0";
  pA.format = fmt;
  std::unique_ptr<cs::stream> streamA = cs::stream::open(aA, pA);
  ASSERT_NE(streamA, nullptr);

  // Wire A → B: push A's frames to B's StreamTransport endpoint.
  ASSERT_TRUE(
      bridgeA.serve(*streamA, transportB.listen_address(), std::chrono::milliseconds(3000)));

  // Produce N frames into A.
  const int N = 5;
  std::vector<std::vector<unsigned char>> sent;
  for (int i = 0; i < N; ++i) {
    std::vector<unsigned char> payload(64);
    for (int j = 0; j < 64; ++j)
      payload[j] = static_cast<unsigned char>((i * 64 + j) & 0xff);
    auto buf = std::make_shared<std::vector<unsigned char>>(payload);
    streamA->channel().publish_external(buf->data(), buf->size(), 0.1 * i, buf);
    sent.push_back(std::move(payload));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Consume them from B's proxy (poll with a timeout — the path is async).
  std::vector<std::vector<unsigned char>> got;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (static_cast<int>(got.size()) < N && std::chrono::steady_clock::now() < deadline) {
    cs::frame_ptr f;
    if (subB->try_pop(f) && f)
      got.emplace_back(f->data, f->data + f->size);
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_EQ(static_cast<int>(got.size()), N) << "not all frames crossed to the consumer";
  for (int i = 0; i < N; ++i)
    EXPECT_EQ(got[i], sent[i]) << "frame " << i << " differs across the node boundary";

  EXPECT_GE(bridgeA.frames_pushed(), static_cast<std::uint64_t>(N));
  EXPECT_GE(bridgeB.frames_injected(), static_cast<std::uint64_t>(N));

  // Teardown order: stop the bridges (join pushers) before dropping streams.
  bridgeA.stop();
  bridgeB.stop();
  streamA.reset();
  proxyB.reset();
  transportA.stop();
  transportB.stop();
}

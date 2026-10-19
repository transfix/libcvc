/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Conference room MEDIA (roadmap §4b), mesh P2P: a participant publishes an AUDIO stream into a
// room and another participant subscribes to it; the audio frames cross the node boundary over the
// merged gRPC stream_bridge, routed entirely by the replicated room-tree descriptor (no manual
// endpoint wiring). Grpc-only (CMake gates it with the other stream_bridge tests). The state plane
// (the room tree) rides an inproc transport (pump_all), the media plane a real gRPC transport on
// loopback — exactly the split a two-process conference uses.

#include <chrono>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_media_intrinsics.h>
#include <cvc/ariadne/stream/audio_sources.h> // audio_f32_bytes
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/app.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vector>

namespace cs = cvc::ariadne::stream;
namespace conf = cvc::ariadne::conference;

TEST(RoomMedia, AudioPublishSubscribeCrossesNodes) {
  cvc::app aA, aB;

  // ── State plane: the room tree, replicated A<->B over an inproc transport (pump_all). ──
  cvc::state_transport_inproc state_tp;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  state_tp.register_shard(&sA);
  state_tp.register_shard(&sB);

  // ── Media plane: A runs a gRPC StreamTransport server (frames land here), B is a client. The
  // bridge is constructed before the transport starts so its open-handler is installed. ──
  cvc::stream_transport_grpc transportA;
  cs::stream_bridge bridgeA(aA, transportA);
  transportA.start("127.0.0.1:0");
  ASSERT_TRUE(transportA.is_running());
  const std::string endpointA = transportA.listen_address();

  cvc::stream_transport_grpc transportB;
  cs::stream_bridge bridgeB(aB, transportB);

  // ── A joins and opens its mic-shaped audio stream (f32 48k mono), then PUBLISHES it. ──
  conf::room_join(aA, "standup", "A", "Alice");
  cs::stream_params pA;
  pA.id = "audio"; // a single path segment (no '.'); scope makes the token unique per participant
  pA.root_path = "A"; // = the publisher's pid -> token "A.streams.audio"
  pA.format.kind = cs::frame_kind::audio_pcm;
  pA.format.codec = "f32";
  pA.format.sample_rate = 48000;
  pA.format.channels = 1;
  pA.format.bytes = cs::audio_f32_bytes(1024, 1);
  pA.subscriber_depth = 8;
  pA.expected_subscribers = 2;
  std::unique_ptr<cs::stream> streamA = cs::stream::open(aA, pA);
  ASSERT_NE(streamA, nullptr);
  ASSERT_TRUE(conf::room_publish(aA, bridgeA, endpointA, "standup", "A", "audio", *streamA));

  // Replicate the room tree (participant + stream descriptor) to B.
  state_tp.pump_all();

  // ── B joins and SUBSCRIBES to A's audio purely from the replicated descriptor. ──
  conf::room_join(aB, "standup", "B", "Bob");
  std::unique_ptr<cs::stream> proxyB = conf::room_subscribe(aB, bridgeB, "standup", "A", "audio");
  ASSERT_NE(proxyB, nullptr) << "subscribe should resolve A's descriptor and pull it";
  EXPECT_EQ(proxyB->token(), streamA->token()) << "proxy shares the publisher's token for routing";

  std::shared_ptr<cs::subscription> subB = proxyB->channel().subscribe(cs::deliver_mode::ring, 8);
  ASSERT_NE(subB, nullptr);

  // The server-push subscription is created asynchronously on A's server thread when B's pull
  // handshake arrives; wait for it so no early frame is missed.
  auto sub_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (streamA->channel().subscriber_count() == 0 &&
         std::chrono::steady_clock::now() < sub_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_GE(streamA->channel().subscriber_count(), 1u) << "the subscribe never pulled";

  // ── Produce N audio chunks into A; they should arrive byte-identical at B's proxy. ──
  const int N = 5;
  const std::size_t chunk = cs::audio_f32_bytes(1024, 1);
  std::vector<std::vector<unsigned char>> sent;
  for (int i = 0; i < N; ++i) {
    auto buf = std::make_shared<std::vector<unsigned char>>(chunk);
    for (std::size_t j = 0; j < chunk; ++j)
      (*buf)[j] = static_cast<unsigned char>((i * 7 + j) & 0xff);
    streamA->channel().publish_external(buf->data(), buf->size(), 0.02 * i, buf);
    sent.push_back(*buf);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::vector<std::vector<unsigned char>> got;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (static_cast<int>(got.size()) < N && std::chrono::steady_clock::now() < deadline) {
    cs::frame_ptr f;
    if (subB->try_pop(f) && f)
      got.emplace_back(f->data, f->data + f->size);
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_EQ(static_cast<int>(got.size()), N) << "published audio did not cross the node boundary";
  for (int i = 0; i < N; ++i)
    EXPECT_EQ(got[i], sent[i]) << "audio chunk " << i << " differs across nodes";

  // Teardown: stop the bridges (close the server-push subscription + join the puller) BEFORE the
  // transports and before the proxy/stream they feed.
  bridgeA.stop();
  bridgeB.stop();
  streamA.reset();
  proxyB.reset();
  transportA.stop();
  transportB.stop();
  state_tp.unregister_shard(&sA);
  state_tp.unregister_shard(&sB);
  sA.detach();
  sB.detach();
}

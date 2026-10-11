/*
  Copyright 2026 The University of Texas at Austin

  Tests for cvc::stream_transport_grpc — the gRPC streaming DATA plane
  (Phase 10, PR7). A dedicated StreamTransport server on its own port, and a
  client that opens a bidi StreamChannel and round-trips frames.
*/

#include <atomic>
#include <chrono>
#include <cvc/state/state_stream_transport_grpc.h>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using cvc::stream_transport_grpc;

namespace {

std::vector<unsigned char> bytes(std::initializer_list<int> v) {
  std::vector<unsigned char> b;
  for (int x : v)
    b.push_back(static_cast<unsigned char>(x));
  return b;
}

} // namespace

// A frame written by a client is delivered to the server's sink AND echoed back
// on the same channel — the skeleton round-trip, over a dedicated listener.
TEST(StreamTransportGrpc, RoundTripFrame) {
  stream_transport_grpc server;
  std::mutex mu;
  std::vector<stream_transport_grpc::frame> sink;
  server.set_on_frame([&](const stream_transport_grpc::frame &f) {
    std::lock_guard<std::mutex> lk(mu);
    sink.push_back(f);
  });
  server.start("127.0.0.1:0"); // OS picks a free port
  ASSERT_TRUE(server.is_running());
  const std::string addr = server.listen_address();
  ASSERT_FALSE(addr.empty());

  stream_transport_grpc client; // a client need not run its own server
  std::unique_ptr<stream_transport_grpc::channel> ch =
      client.open_channel(addr, std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);

  stream_transport_grpc::frame f;
  f.stream_id = "streams.cam0";
  f.seq = 42;
  f.pts_seconds = 1.5;
  f.flags = stream_transport_grpc::flag_keyframe;
  f.payload = bytes({1, 2, 3, 4, 5});
  ASSERT_TRUE(ch->send(f));

  // The server echoes the frame back; recv() returns it byte-identical.
  stream_transport_grpc::frame echo;
  ASSERT_TRUE(ch->recv(echo));
  EXPECT_EQ(echo.stream_id, "streams.cam0");
  EXPECT_EQ(echo.seq, 42);
  EXPECT_DOUBLE_EQ(echo.pts_seconds, 1.5);
  EXPECT_EQ(echo.flags, static_cast<std::uint32_t>(stream_transport_grpc::flag_keyframe));
  EXPECT_EQ(echo.payload, bytes({1, 2, 3, 4, 5}));

  // The server's sink saw it too (invoked before the echo, on a server thread).
  bool got = false;
  for (int i = 0; i < 400 && !got; ++i) {
    {
      std::lock_guard<std::mutex> lk(mu);
      got = !sink.empty();
    }
    if (!got)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(got) << "server sink never received the frame";
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(sink.front().stream_id, "streams.cam0");
    EXPECT_EQ(sink.front().seq, 42);
    EXPECT_EQ(sink.front().payload, bytes({1, 2, 3, 4, 5}));
  }
  EXPECT_GE(server.frames_received(), 1u);
  EXPECT_GE(server.frames_echoed(), 1u);

  ch->close();
  server.stop();
  EXPECT_FALSE(server.is_running());
}

// A single frame larger than the ~4 MiB gRPC default must round-trip, proving
// the raised message-size cap on both the server and the client channel.
TEST(StreamTransportGrpc, LargePayloadRoundTrip) {
  stream_transport_grpc server;
  server.start("127.0.0.1:0");
  const std::string addr = server.listen_address();
  ASSERT_FALSE(addr.empty());

  stream_transport_grpc client;
  auto ch = client.open_channel(addr, std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);

  stream_transport_grpc::frame f;
  f.stream_id = "streams.big";
  f.seq = 7;
  f.payload.resize(5u * 1024u * 1024u); // 5 MiB > 4 MiB gRPC default
  for (std::size_t i = 0; i < f.payload.size(); ++i)
    f.payload[i] = static_cast<unsigned char>(i & 0xff);
  ASSERT_TRUE(ch->send(f));

  stream_transport_grpc::frame echo;
  ASSERT_TRUE(ch->recv(echo));
  ASSERT_EQ(echo.payload.size(), f.payload.size());
  EXPECT_EQ(echo.payload, f.payload);
  EXPECT_EQ(echo.stream_id, "streams.big");
  EXPECT_EQ(echo.seq, 7);

  ch->close();
  server.stop();
}

// A frame larger than the configured chunk size is split into multiple
// StreamFrames on the wire and reassembled byte-identical on the peer.
TEST(StreamTransportGrpc, ChunkedFrameRoundTrip) {
  stream_transport_grpc server;
  std::mutex mu;
  std::vector<stream_transport_grpc::frame> sink;
  server.set_on_frame([&](const stream_transport_grpc::frame &f) {
    std::lock_guard<std::mutex> lk(mu);
    sink.push_back(f);
  });
  server.start("127.0.0.1:0");
  const std::string addr = server.listen_address();
  ASSERT_FALSE(addr.empty());

  stream_transport_grpc client;
  client.set_chunk_bytes(4096); // force a multi-chunk split
  EXPECT_EQ(client.chunk_bytes(), 4096u);
  auto ch = client.open_channel(addr, std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);

  stream_transport_grpc::frame f;
  f.stream_id = "streams.cam0";
  f.seq = 100;
  f.pts_seconds = 2.0;
  f.payload.resize(40000); // ~10 chunks at 4096 B
  for (std::size_t i = 0; i < f.payload.size(); ++i)
    f.payload[i] = static_cast<unsigned char>((i * 31u + 7u) & 0xff);
  ASSERT_TRUE(ch->send(f));

  // recv() reassembles the echoed chunks back into the whole frame.
  stream_transport_grpc::frame echo;
  ASSERT_TRUE(ch->recv(echo));
  EXPECT_EQ(echo.stream_id, "streams.cam0");
  EXPECT_EQ(echo.seq, 100);
  EXPECT_DOUBLE_EQ(echo.pts_seconds, 2.0);
  ASSERT_EQ(echo.payload.size(), f.payload.size());
  EXPECT_EQ(echo.payload, f.payload);

  // The server delivered exactly ONE reassembled frame to the sink.
  bool got = false;
  for (int i = 0; i < 400 && !got; ++i) {
    {
      std::lock_guard<std::mutex> lk(mu);
      got = !sink.empty();
    }
    if (!got)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(got);
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(sink.size(), 1u) << "server must see exactly one reassembled frame";
    EXPECT_EQ(sink.front().payload, f.payload);
  }
  EXPECT_EQ(server.frames_received(), 1u);
  EXPECT_GT(server.frames_echoed(), server.frames_received())
      << "a split frame must produce more wire chunks than whole frames";

  ch->close();
  server.stop();
}

// Opening a channel to an address with no StreamTransport server fails cleanly
// (null) within the timeout rather than hanging or crashing.
TEST(StreamTransportGrpc, OpenChannelToDeadAddressReturnsNull) {
  stream_transport_grpc client;
  auto ch = client.open_channel("127.0.0.1:1", std::chrono::milliseconds(500));
  EXPECT_EQ(ch, nullptr);
}

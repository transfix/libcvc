/*
  Copyright 2026 The University of Texas at Austin

  Tests for cvc::stream_transport_grpc — the gRPC streaming DATA plane
  (Phase 10, PR7). A dedicated StreamTransport server on its own port, and a
  client that opens a bidi StreamChannel and round-trips frames.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/state/state_stream_transport_grpc.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <mutex>
#include <random>
#include <sstream>
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

// Mint a throwaway self-signed cert (CN/SAN localhost) via openssl, same as the
// control-plane TLS test. Returns false if openssl is unavailable.
bool generate_self_signed(std::string &cert_pem, std::string &key_pem) {
  std::string dir = make_temp_dir("cvc_stream_tls_");
  if (dir.empty())
    return false;
  std::string cmd = "openssl req -x509 -newkey rsa:2048 -keyout " + dir + "/k.pem -out " + dir +
                    "/c.pem -days 1 -nodes -subj '/CN=localhost' "
                    "-addext 'subjectAltName=DNS:localhost,IP:127.0.0.1' >/dev/null 2>&1";
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

// A frame round-trips over a TLS-encrypted StreamChannel. Opt-in (needs openssl
// + CVC_DISTRIBUTED_STATE_TLS_TESTS=1), mirroring the control-plane TLS test.
TEST(StreamTransportGrpc, TlsRoundTrip) {
  if (!tls_tests_enabled())
    GTEST_SKIP() << "set CVC_DISTRIBUTED_STATE_TLS_TESTS=1 to run TLS tests";
  std::string cert, key;
  if (!generate_self_signed(cert, key))
    GTEST_SKIP() << "openssl not available; cannot generate a self-signed cert";

  stream_transport_grpc::tls_config tls;
  tls.server_cert_pem = cert;
  tls.server_key_pem = key;
  tls.root_ca_pem = cert; // self-signed: the cert is its own CA

  stream_transport_grpc server;
  server.set_tls_config(tls);
  server.start("127.0.0.1:0");
  ASSERT_TRUE(server.is_running());
  const std::string addr = server.listen_address();
  const auto colon = addr.find_last_of(':');
  ASSERT_NE(colon, std::string::npos);
  // Connect by name so the cert's localhost SAN validates.
  const std::string target = "localhost:" + addr.substr(colon + 1);

  stream_transport_grpc client;
  client.set_tls_config(tls); // root_ca for the client side
  std::unique_ptr<stream_transport_grpc::channel> ch =
      client.open_channel(target, std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);

  stream_transport_grpc::frame f;
  f.stream_id = "streams.tls";
  f.seq = 7;
  f.payload = bytes({9, 8, 7, 6});
  ASSERT_TRUE(ch->send(f));
  stream_transport_grpc::frame echo;
  ASSERT_TRUE(ch->recv(echo));
  EXPECT_EQ(echo.stream_id, "streams.tls");
  EXPECT_EQ(echo.payload, bytes({9, 8, 7, 6}));

  ch->close();
  server.stop();
}

// Server-push: a client opens a channel with a flag_open frame naming a stream,
// and the server's open handler pushes that stream's frames back (the
// conventional producer-serves-a-port model).
TEST(StreamTransportGrpc, ServerPush) {
  stream_transport_grpc server;
  server.set_open_handler([](const std::string &sid) -> stream_transport_grpc::frame_producer {
    if (sid != "streams.push")
      return {}; // decline unknown streams
    auto counter = std::make_shared<int>(0);
    return [counter](stream_transport_grpc::frame &out) -> bool {
      if (*counter >= 3)
        return false; // exhausted after 3 frames
      out.stream_id = "streams.push";
      out.seq = *counter;
      out.payload = bytes({*counter, *counter + 10});
      ++(*counter);
      return true;
    };
  });
  server.start("127.0.0.1:0");
  ASSERT_TRUE(server.is_running());

  stream_transport_grpc client;
  std::unique_ptr<stream_transport_grpc::channel> ch =
      client.open_channel(server.listen_address(), std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);

  // Open-handshake frame: name the stream we want pushed.
  stream_transport_grpc::frame open;
  open.stream_id = "streams.push";
  open.flags = stream_transport_grpc::flag_open;
  ASSERT_TRUE(ch->send(open));

  std::vector<stream_transport_grpc::frame> got;
  for (int i = 0; i < 3; ++i) {
    stream_transport_grpc::frame f;
    if (!ch->recv(f))
      break;
    got.push_back(f);
  }
  ASSERT_EQ(got.size(), 3u) << "server did not push all frames";
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(got[i].stream_id, "streams.push");
    EXPECT_EQ(got[i].seq, i);
    EXPECT_EQ(got[i].payload, bytes({i, i + 10}));
  }

  ch->close();
  server.stop();
}

// close() must unblock a recv() parked on a LONG-LIVED server stream that never
// finishes on its own. Regression: close() did WritesDone()+Finish(), and
// Finish() blocks until the server completes — which an idle push stream never
// does — so close() (and the parked recv()) hung forever. Now close() TryCancel()s.
TEST(StreamTransportGrpc, CloseUnblocksBlockedRecv) {
  auto done = std::make_shared<std::atomic<bool>>(false);
  stream_transport_grpc server;
  server.set_open_handler([done](const std::string &) -> stream_transport_grpc::frame_producer {
    return [done](stream_transport_grpc::frame &) -> bool {
      // Idle producer: yields no frame until teardown, so the server stream
      // stays open and the client's recv() blocks in Read().
      while (!done->load())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return false; // exhausted → lets the server finish for a clean stop()
    };
  });
  server.start("127.0.0.1:0");
  ASSERT_TRUE(server.is_running());

  stream_transport_grpc client;
  std::unique_ptr<stream_transport_grpc::channel> ch =
      client.open_channel(server.listen_address(), std::chrono::milliseconds(3000));
  ASSERT_NE(ch, nullptr);
  stream_transport_grpc::frame open;
  open.stream_id = "streams.idle";
  open.flags = stream_transport_grpc::flag_open;
  ASSERT_TRUE(ch->send(open));

  std::atomic<bool> recv_returned{false};
  std::thread reader([&]() {
    stream_transport_grpc::frame f;
    ch->recv(f); // parks in Read() until close() cancels the RPC
    recv_returned.store(true);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(150)); // let recv() park
  ch->close();   // must return promptly (regression: hung in Finish())
  reader.join(); // must not hang (recv() returned false on cancel)
  EXPECT_TRUE(recv_returned.load());

  *done = true; // release the server producer for a clean shutdown
  server.stop();
}

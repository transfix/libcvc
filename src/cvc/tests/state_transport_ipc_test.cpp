/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_transport_ipc.h>
#include <fcntl.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

std::string make_socket_path(const std::string &label) {
  auto pid = static_cast<long long>(::getpid());
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  auto dir = std::filesystem::temp_directory_path();
  auto p =
      dir / ("cvc_ipc_" + std::to_string(pid) + "_" + std::to_string(now) + "_" + label + ".sock");
  return p.string();
}

// Spin until both endpoints have at least one connection or timeout.
bool wait_connected(cvc::state_transport_ipc &a, cvc::state_transport_ipc &b,
                    std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (a.connection_count() >= 1 && b.connection_count() >= 1)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// Spin until `pred` holds or timeout. Receive-side assertions need
// this rather than wait_for_received(): that counter is incremented
// by the reader thread when a frame is decoded, which is before
// dispatch_inbound has ingested it, so it can be satisfied a moment
// before the mutation is visible in the tree or the replica.
template <typename Pred> bool wait_until(Pred pred, std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

} // namespace

TEST(StateTransportIpcTest, StartStopBindsAndUnlinks) {
  cvc::state_transport_ipc t;
  auto path = make_socket_path("startstop");
  ASSERT_NO_THROW(t.start(path, "A", "C"));
  EXPECT_TRUE(std::filesystem::exists(path));
  t.stop();
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(StateTransportIpcTest, ConnectFailsWhenPeerMissing) {
  cvc::state_transport_ipc t;
  auto path = make_socket_path("noserver");
  EXPECT_FALSE(t.connect_to_peer(path, std::chrono::milliseconds(50)));
}

TEST(StateTransportIpcTest, TwoEndpointConvergence) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_conv");
  auto pB = make_socket_path("B_conv");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  // First-set-on-fresh-child is lost (adapter quirk). Set twice.
  cvc::state::instance(aA)("k").value(std::string("init"));
  cvc::state::instance(aA)("k").value(std::string("v1"));

  std::size_t pumped = tA.pump_all();
  EXPECT_GE(pumped, 1u);
  tA.flush();
  // Receive is async on B.
  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("k").value() == "v1"; },
                         std::chrono::milliseconds(2000)));

  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1");
  EXPECT_GE(sB.replica().last_applied("A"), 1u);

  // Stop transports before shards leave scope; otherwise reader
  // threads may still call ingest_remote on a destroyed shard.
  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, RoundTripSuppressedByDedup) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_rt");
  auto pB = make_socket_path("B_rt");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("rt").value(std::string("seed"));
  cvc::state::instance(aA)("rt").value(std::string("payload"));
  tA.pump_all();
  tA.flush();
  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("rt").value() == "payload"; },
                         std::chrono::milliseconds(2000)));

  // B applied A's mutation but does not re-journal it. Pumping B
  // should be a no-op so nothing flows back.
  std::size_t b_pumped = tB.pump_all();
  EXPECT_EQ(b_pumped, 0u);

  cvc::state::instance(aA)("rt").value(std::string("payload2"));
  tA.pump_all();
  tA.flush();
  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("rt").value() == "payload2"; },
                         std::chrono::milliseconds(2000)));
  EXPECT_EQ(cvc::state::instance(aB)("rt").value(), "payload2");
  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, CrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_iso");
  auto pB = make_socket_path("B_iso");
  tA.start(pA, "A", "C1");
  tB.start(pB, "B", "C2");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("iso").value(std::string("seed"));
  cvc::state::instance(aA)("iso").value(std::string("v"));
  tA.pump_all();
  tA.flush();
  // Frame still arrives but B's shard is in a different cluster.
  tB.wait_for_received(1, std::chrono::milliseconds(500));

  EXPECT_NE(cvc::state::instance(aB)("iso").value(), "v");
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);
  tA.stop();
  tB.stop();
}

// A writes and pumps while nobody is connected. pump_shard() drains
// the journal and advances the publish cursor regardless of whether
// there was anywhere to send, so without backfill-on-connect those
// mutations are gone: a peer that connects afterwards would never
// see them, however long it pumps.
TEST(StateTransportIpcTest, LateJoinerReceivesWritesMadeBeforeItConnected) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_late");
  auto pB = make_socket_path("B_late");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("late.k").value(std::string("seed"));
  cvc::state::instance(aA)("late.k").value(std::string("v1"));

  // The drop: drained with zero connections, cursor advanced anyway.
  ASSERT_EQ(tA.connection_count(), 0u);
  EXPECT_GT(tA.pump_all(), 0u);
  tA.flush();
  EXPECT_GT(sA.published_cursor(), 0u);

  // B joins only now, and A never writes again.
  ASSERT_TRUE(tB.connect_to_peer(pA, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  EXPECT_TRUE(wait_until([&] { return cvc::state::instance(aB)("late.k").value() == "v1"; },
                         std::chrono::milliseconds(2000)));
  EXPECT_EQ(cvc::state::instance(aB)("late.k").value(), "v1");
  EXPECT_GT(tA.total_backfilled(), 0u);
  tA.stop();
  tB.stop();
}

// Backfill must reach the peer ahead of any concurrently published
// mutation. The receiving replica's seen-set is exact membership, not
// a high-water mark, so if a live frame overtook the replay the older
// values would be applied last and clobber the newer one.
//
// Reads frames off a raw socket rather than using a second transport,
// because the property under test is wire order: mutation sequences
// must arrive monotonically even while another thread publishes.
TEST(StateTransportIpcTest, BackfillPrecedesConcurrentLiveTraffic) {
  cvc::app aA;
  cvc::state_transport_ipc tA;
  auto pA = make_socket_path("A_order");
  tA.start(pA, "A", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  sA.attach();
  tA.register_shard(&sA);

  cvc::state::instance(aA)("ord.k").value(std::string("seed"));
  for (int i = 0; i < 64; ++i)
    cvc::state::instance(aA)("ord.k").value("v" + std::to_string(i));
  tA.pump_all(); // Dropped: no peers yet. Journal keeps all 64.
  tA.flush();

  // Raw peer: connect, then keep publishing so live frames race the
  // backfill on this very connection.
  int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, pA.c_str(), sizeof(addr.sun_path) - 1);
  ASSERT_EQ(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);

  // Drain the socket on its own thread throughout. publish() writes
  // synchronously, so a peer that stops reading would wedge the
  // writer once the socket buffer filled.
  ASSERT_EQ(::fcntl(fd, F_SETFL, O_NONBLOCK), 0);
  std::vector<unsigned char> buf;
  std::mutex buf_mu;
  std::atomic<bool> stop_reader{false};
  std::thread drainer([&]() {
    while (!stop_reader.load()) {
      unsigned char tmp[65536];
      ssize_t r = ::recv(fd, tmp, sizeof(tmp), 0);
      if (r > 0) {
        std::lock_guard<std::mutex> lk(buf_mu);
        buf.insert(buf.end(), tmp, tmp + r);
        continue;
      }
      if (r == 0)
        return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  for (int i = 64; i < 128; ++i) {
    cvc::state::instance(aA)("ord.k").value("v" + std::to_string(i));
    tA.pump_all();
    tA.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  stop_reader.store(true);
  drainer.join();
  ::close(fd);
  std::lock_guard<std::mutex> buf_lk(buf_mu);

  // Walk the frame stream: u32 magic, u16 version, u16 type, u32 len.
  auto le32 = [&](std::size_t o) {
    return static_cast<std::uint32_t>(buf[o]) | (static_cast<std::uint32_t>(buf[o + 1]) << 8) |
           (static_cast<std::uint32_t>(buf[o + 2]) << 16) |
           (static_cast<std::uint32_t>(buf[o + 3]) << 24);
  };
  std::size_t off = 0;
  int frames = 0, mutations = 0;
  std::uint64_t prev_seq = 0;
  bool first_frame_is_hello = false;
  while (off + 12 <= buf.size()) {
    std::uint16_t mtype = static_cast<std::uint16_t>(buf[off + 4 + 2] | (buf[off + 4 + 3] << 8));
    std::uint32_t len = le32(off + 8);
    if (off + 12 + len > buf.size())
      break;
    if (frames == 0)
      first_frame_is_hello = (mtype == 1);
    if (mtype == 2) { // MUTATION
      // Payload: cluster_id, tree_id, origin_node_id (all u32-len
      // prefixed strings), then u64 sequence.
      std::size_t p = off + 12;
      for (int f = 0; f < 3; ++f)
        p += 4 + le32(p);
      std::uint64_t seq = 0;
      for (int b = 0; b < 8; ++b)
        seq |= static_cast<std::uint64_t>(buf[p + b]) << (8 * b);
      // Non-decreasing, not strictly increasing: a mutation caught by
      // both the backfill snapshot and a live publish legitimately
      // goes out twice, and the receiver's seen-set drops the repeat.
      // Only a sequence going *backwards* means the replay lost the
      // race and would overwrite a newer value with an older one.
      EXPECT_GE(seq, prev_seq) << "mutation sequence went backwards at frame " << frames << " ("
                               << seq << " after " << prev_seq
                               << "): backfill lost the race with live traffic";
      prev_seq = seq;
      ++mutations;
    }
    off += 12 + len;
    ++frames;
  }
  EXPECT_TRUE(first_frame_is_hello);
  EXPECT_GE(mutations, 64) << "expected the whole journal to be backfilled";
  tA.stop();
}

// Convergence check on the same race, through a real peer: whatever
// the interleaving, B must end up holding A's latest value.
TEST(StateTransportIpcTest, BackfillDoesNotClobberConcurrentWrites) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_race");
  auto pB = make_socket_path("B_race");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("race.k").value(std::string("seed"));
  for (int i = 0; i < 32; ++i)
    cvc::state::instance(aA)("race.k").value("v" + std::to_string(i));
  tA.pump_all(); // Dropped: no peers yet.
  tA.flush();

  // Keep publishing while B connects, so a live frame races the
  // backfill on the same connection.
  std::atomic<bool> stop_writer{false};
  std::atomic<int> last_written{-1};
  std::thread writer([&]() {
    for (int i = 32; !stop_writer.load(); ++i) {
      cvc::state::instance(aA)("race.k").value("v" + std::to_string(i));
      last_written.store(i);
      tA.pump_all();
      tA.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  ASSERT_TRUE(tB.connect_to_peer(pA, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop_writer.store(true);
  writer.join();

  // Drain anything still queued, then let it settle.
  tA.pump_all();
  tA.flush();
  std::string want = "v" + std::to_string(last_written.load());
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cvc::state::instance(aB)("race.k").value() == want)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  // B must converge on the newest value, not on a replayed older one.
  EXPECT_EQ(cvc::state::instance(aB)("race.k").value(), want);
  tA.stop();
  tB.stop();
}

// The escape hatch: with backfill disabled the transport keeps its
// old behaviour, so the late joiner sees nothing.
TEST(StateTransportIpcTest, BackfillCanBeDisabled) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_nobf");
  auto pB = make_socket_path("B_nobf");
  tA.set_backfill_on_connect(false);
  EXPECT_FALSE(tA.backfill_on_connect());
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("nobf.k").value(std::string("seed"));
  cvc::state::instance(aA)("nobf.k").value(std::string("v1"));
  tA.pump_all();
  tA.flush();

  ASSERT_TRUE(tB.connect_to_peer(pA, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(tA.total_backfilled(), 0u);
  EXPECT_NE(cvc::state::instance(aB)("nobf.k").value(), "v1");
  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, ThreeEndpointFanOut) {
  cvc::app aA, aB, aC;
  cvc::state_transport_ipc tA, tB, tC;
  auto pA = make_socket_path("A_fan");
  auto pB = make_socket_path("B_fan");
  auto pC = make_socket_path("C_fan");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  tC.start(pC, "C", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(tA.connect_to_peer(pC, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);
  tC.register_shard(&sC);

  cvc::state::instance(aA)("fan").value(std::string("seed"));
  cvc::state::instance(aA)("fan").value(std::string("hello"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  tC.wait_for_received(1, std::chrono::milliseconds(2000));

  EXPECT_EQ(cvc::state::instance(aB)("fan").value(), "hello");
  EXPECT_EQ(cvc::state::instance(aC)("fan").value(), "hello");
  tA.stop();
  tB.stop();
  tC.stop();
}

TEST(StateTransportIpcTest, BlobPayloadRoundTrip) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_blob");
  auto pB = make_socket_path("B_blob");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  // Hand-craft a mutation with a blob_ref payload and publish it
  // directly through the transport to verify the blob branch of the
  // wire codec.
  cvc::state_mutation m;
  m.cluster_id = "C";
  m.tree_id = "default";
  m.origin_node_id = "A";
  m.sequence = 100;
  m.mutation_id = "A:100";
  m.path = "blob.path";
  m.op = cvc::state_mutation_op::set_value;
  m.type_name = "std::string";
  m.string_value = "blob-name";
  m.payload.kind = cvc::state_payload_kind::blob;
  m.payload.blob.digest = "deadbeefcafebabe";
  m.payload.blob.size_bytes = 4096;
  m.payload.blob.codec = "raw";
  m.latest_value_only = false;

  tA.publish(m);
  tA.flush();
  EXPECT_TRUE(wait_until([&] { return sB.replica().last_applied("A") == 100u; },
                         std::chrono::milliseconds(2000)));

  EXPECT_GE(tB.total_received_frames(), 1u);
  // sB.replica should record A,100 as last_applied (blob fetch may
  // not happen in this minimal env; mutation envelope is enough).
  EXPECT_EQ(sB.replica().last_applied("A"), 100u);
  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, DisconnectAfterStop) {
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_stop");
  auto pB = make_socket_path("B_stop");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  tA.stop();
  // tB's reader should observe close and connection_count drops to 0
  // eventually; but with our reader thread held for join until tB
  // also stops, just verify tB.stop() returns cleanly.
  tB.stop();
  SUCCEED();
}

TEST(StateTransportIpcStressTest, OptionalConcurrentPublishStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run ipc "
                    "transport stress tests";
  }
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_str");
  auto pB = make_socket_path("B_str");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  const int kPerWriter = 200;
  std::atomic<bool> stop{false};
  std::thread pumper([&]() {
    while (!stop.load()) {
      tA.pump_all();
      tB.pump_all();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  std::thread wA([&]() {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(aA)("stress.A").value(std::string("v0"));
      cvc::state::instance(aA)("stress.A").value("v" + std::to_string(i));
    }
  });
  std::thread wB([&]() {
    for (int i = 0; i < kPerWriter; ++i) {
      cvc::state::instance(aB)("stress.B").value(std::string("v0"));
      cvc::state::instance(aB)("stress.B").value("v" + std::to_string(i));
    }
  });
  wA.join();
  wB.join();

  for (int i = 0; i < 32; ++i) {
    if (tA.pump_all() == 0 && tB.pump_all() == 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  stop.store(true);
  pumper.join();
  while (tA.pump_all() > 0 || tB.pump_all() > 0) {
  }
  // Wait for receive convergence.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::string lastA = "v" + std::to_string(kPerWriter - 1);
  std::string lastB = "v" + std::to_string(kPerWriter - 1);
  EXPECT_EQ(cvc::state::instance(aB)("stress.A").value(), lastA);
  EXPECT_EQ(cvc::state::instance(aA)("stress.B").value(), lastB);
  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcPerformanceTest, OptionalRoundTripThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run ipc "
                    "transport perf tests";
  }
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_perf");
  auto pB = make_socket_path("B_perf");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("perf.k").value(std::string("seed"));

  const int kIters = 2000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i) {
    cvc::state::instance(aA)("perf.k").value("v" + std::to_string(i));
    if ((i & 0x3F) == 0)
      tA.pump_all();
  }
  while (tA.pump_all() > 0) {
  }
  tA.flush();
  tB.wait_for_received(static_cast<std::uint64_t>(kIters / 64), std::chrono::milliseconds(5000));
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  std::cerr << "[transport_ipc perf] " << kIters << " mutations across UDS in " << secs << "s ("
            << (kIters / secs) << " mut/s, sent=" << tA.total_sent_frames()
            << ", recv=" << tB.total_received_frames() << ")\n";
  EXPECT_LT(secs, 30.0);
  tA.stop();
  tB.stop();
}

// ----------------------------------------------------------------------------
// Phase 4: out-of-band messaging tests.
// ----------------------------------------------------------------------------

#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>

namespace {

cvc::state_message make_oob_ipc(const std::string &cluster, const std::string &origin,
                                const std::string &id, const std::string &path,
                                const std::string &str = {}) {
  cvc::state_message m;
  m.cluster_id = cluster;
  m.origin_node_id = origin;
  m.message_id = id;
  m.path = path;
  m.string_value = str;
  m.content_type = "text/plain";
  return m;
}

} // namespace

TEST(StateTransportIpcTest, MessageRoundTripCrossPeer) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_msg");
  auto pB = make_socket_path("B_msg");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  std::string payload;
  sB.message_bus().subscribe("chat", [&](const cvc::state_message &m) {
    payload = m.string_value;
    hits.fetch_add(1);
  });

  auto stats = tA.publish_message(make_oob_ipc("C", "A", "m1", "chat.lobby", "hi"));
  (void)stats;
  EXPECT_TRUE(tB.wait_for_received_messages(1, std::chrono::milliseconds(2000)));
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(payload, "hi");

  // Journal not advanced; clocks not advanced.
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);

  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, MessageDedupAcrossMultiPath) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_dup");
  auto pB = make_socket_path("B_dup");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) { hits.fetch_add(1); });

  auto m = make_oob_ipc("C", "A", "m1", "x", "v");
  tA.publish_message(m);
  tA.publish_message(m);
  tA.publish_message(m);

  EXPECT_TRUE(tB.wait_for_received_messages(1, std::chrono::milliseconds(2000)));
  // Allow a short settling window for any duplicate frames in flight.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(hits.load(), 1);

  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, MessageCrossClusterIsolation) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_iso");
  auto pB = make_socket_path("B_iso");
  tA.start(pA, "A", "C1");
  tB.start(pB, "B", "C2");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C1", "A");
  cvc::state_cluster_shard sB(aB, "C2", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  std::atomic<int> hits{0};
  sB.message_bus().subscribe("", [&](const cvc::state_message &) { hits.fetch_add(1); });

  tA.publish_message(make_oob_ipc("C1", "A", "m1", "x", "v"));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(hits.load(), 0);

  tA.stop();
  tB.stop();
}

TEST(StateTransportIpcTest, MessageDoesNotAdvanceJournalOrClock) {
  cvc::app aA, aB;
  cvc::state_transport_ipc tA, tB;
  auto pA = make_socket_path("A_noj");
  auto pB = make_socket_path("B_noj");
  tA.start(pA, "A", "C");
  tB.start(pB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  auto a_jb = sA.journal().size();
  auto b_jb = sB.journal().size();

  tA.publish_message(make_oob_ipc("C", "A", "m1", "x", "v"));
  tA.publish_message(make_oob_ipc("C", "A", "m2", "y", "w"));
  EXPECT_TRUE(tB.wait_for_received_messages(2, std::chrono::milliseconds(2000)));

  EXPECT_EQ(sA.journal().size(), a_jb);
  EXPECT_EQ(sB.journal().size(), b_jb);
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);

  tA.stop();
  tB.stop();
}

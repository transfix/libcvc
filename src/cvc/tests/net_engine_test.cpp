// cvc::net's connection engine (inc/cvc/net/net_engine.h, docs/roadmap/ARIADNE_NET_ENGINE.md N1,
// N3): framing in its three modes and their switches, the order of sends and broadcasts, groups,
// backpressure, the inbound limits, the counters, TLS (handshakes on 1.2 and 1.3, a refused
// version, reloads and the files' watch, the keyprint, detection on a plain port, backpressure and
// broadcasts over TLS; with OpenSSL, CVC_NET_TEST_TLS), and a fan-out benchmark (one message to N
// loopback connections; CVC_NET_FANOUT_N sets N, 6000 by default, and CVC_NET_FANOUT_THREADS the
// engine's I/O threads, 2 by default).
//
// The clients are plain Asio sockets on the test's side (TLS ones run OpenSSL over them); the
// engine runs its own threads.

#include <algorithm>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/net/net_engine.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#if CVC_NET_TEST_TLS
#include <filesystem>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
#endif

#if !defined(_WIN32)
#include <sys/resource.h>
#endif
#if defined(__APPLE__)
#include <sys/sysctl.h>
#include <sys/syslimits.h>
#endif

#if defined(__SANITIZE_THREAD__)
#define CVC_NET_TEST_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define CVC_NET_TEST_TSAN 1
#endif
#endif

namespace {

namespace asio = boost::asio;
using tcp = asio::ip::tcp;
using namespace std::chrono_literals;
using cvc::net::close_reason;
using cvc::net::conn_id;
using cvc::net::framing;
using cvc::net::listener_options;
using cvc::net::net_batch;
using cvc::net::net_engine;
using cvc::net::net_event;

// A blocking client with reads that give up after a timeout.
struct client {
  asio::io_context ctx;
  tcp::socket s{ctx};

  explicit client(std::uint16_t port, int receive_buffer = 0) {
    s.open(tcp::v4());
    if (receive_buffer)
      s.set_option(asio::socket_base::receive_buffer_size(receive_buffer));
    s.connect({asio::ip::make_address("127.0.0.1"), port});
    s.set_option(tcp::no_delay(true));
    s.non_blocking(true);
  }

  void write(const std::string &bytes) {
    std::size_t done = 0;
    while (done < bytes.size()) {
      boost::system::error_code ec;
      done += s.write_some(asio::buffer(bytes.data() + done, bytes.size() - done), ec);
      if (ec == asio::error::would_block)
        std::this_thread::sleep_for(1ms);
      else if (ec)
        return;
    }
  }

  // Read until `n` bytes came, the peer closed or `timeout` passed.
  std::string read(std::size_t n, std::chrono::milliseconds timeout = 5000ms) {
    std::string out;
    char buf[65536];
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (out.size() < n && std::chrono::steady_clock::now() < deadline) {
      boost::system::error_code ec;
      const std::size_t got =
          s.read_some(asio::buffer(buf, std::min(sizeof buf, n - out.size())), ec);
      if (ec == asio::error::would_block) {
        std::this_thread::sleep_for(1ms);
        continue;
      }
      if (ec)
        break;
      out.append(buf, got);
    }
    return out;
  }

  // Whether the peer ends the connection (end of stream or a reset) within `timeout`.
  bool closed(std::chrono::milliseconds timeout = 5000ms) {
    char buf[4096];
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      boost::system::error_code ec;
      s.read_some(asio::buffer(buf), ec);
      if (ec == asio::error::would_block) {
        std::this_thread::sleep_for(1ms);
        continue;
      }
      if (ec)
        return true;
    }
    return false;
  }
};

// What the program has seen, with the frames copied out of their batches.
struct seen {
  std::vector<conn_id> accepted;
  std::map<conn_id, cvc::net::conn_info> info;
  std::map<conn_id, std::vector<std::string>> frames;
  std::map<conn_id, close_reason> closed;
  std::map<conn_id, int> writable;
  std::vector<std::string> errors;
  std::vector<std::string> reloads; // tls_reloaded's keyprints
  std::size_t batches = 0;
};

struct program {
  net_engine &engine;
  seen s;
  std::function<void(const net_event &, const std::string &)> on_frame; // each frame, as it comes

  explicit program(net_engine &e) : engine(e) {}

  void absorb(const net_batch &b) {
    ++s.batches;
    for (const net_event &ev : b.events) {
      switch (ev.kind) {
      case net_event::type::accepted:
        s.accepted.push_back(ev.conn);
        s.info[ev.conn] = ev.info;
        break;
      case net_event::type::frames:
        for (std::string_view f : ev.frames) {
          s.frames[ev.conn].emplace_back(f);
          if (on_frame)
            on_frame(ev, s.frames[ev.conn].back());
        }
        break;
      case net_event::type::closed:
        s.closed[ev.conn] = ev.reason;
        break;
      case net_event::type::writable:
        ++s.writable[ev.conn];
        break;
      case net_event::type::error:
        s.errors.push_back(ev.message);
        break;
      case net_event::type::tls_reloaded:
        s.reloads.push_back(ev.message);
        break;
      }
    }
  }

  template <class Pred> bool until(Pred pred, std::chrono::milliseconds timeout = 5000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    net_batch b;
    while (!pred()) {
      if (std::chrono::steady_clock::now() > deadline)
        return false;
      if (engine.wait(b, 10ms))
        absorb(b);
    }
    return true;
  }

  // Take what comes for `d`.
  void idle(std::chrono::milliseconds d) {
    const auto end = std::chrono::steady_clock::now() + d;
    net_batch b;
    while (std::chrono::steady_clock::now() < end)
      if (engine.wait(b, 5ms))
        absorb(b);
  }

  conn_id accept_one(std::size_t index = 0) {
    EXPECT_TRUE(until([&] { return s.accepted.size() > index; }));
    return s.accepted.size() > index ? s.accepted[index] : 0;
  }

  std::vector<std::string> frames_of(conn_id c) { return s.frames[c]; }

  // The connection of a client (two workers post their accepts in either order).
  conn_id id_of(const client &c) {
    const std::uint16_t port = c.s.local_endpoint().port();
    conn_id found = 0;
    until([&] {
      for (const auto &[id, info] : s.info)
        if (info.peer_port == port)
          found = id;
      return found != 0;
    });
    return found;
  }
};

listener_options local(framing f, std::size_t max_frame = 65536) {
  listener_options o;
  o.address = "127.0.0.1";
  o.port = 0;
  o.name = "test";
  o.frames = std::move(f);
  o.max_frame = max_frame;
  return o;
}

std::string be(std::size_t n, int bytes) {
  std::string out;
  for (int i = bytes - 1; i >= 0; --i)
    out.push_back(static_cast<char>((n >> (8 * i)) & 0xff));
  return out;
}

// ── framing ──────────────────────────────────────────────────────────────────────────────────

TEST(NetEngineFraming, DelimiterFramesSplitAcrossReadsAndManyInOneRead) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::delimited('|')));
  ASSERT_NE(l, 0u);
  client c(e.port(l));
  const conn_id id = p.accept_one();
  c.write("ab");
  std::this_thread::sleep_for(30ms);
  c.write("|cd");
  std::this_thread::sleep_for(30ms);
  c.write("|e|f|g|");
  ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == 5; }));
  EXPECT_EQ(p.frames_of(id), (std::vector<std::string>{"ab|", "cd|", "e|", "f|", "g|"}));
}

TEST(NetEngineFraming, LengthPrefixesOfOneTwoAndFourBytes) {
  for (int prefix : {1, 2, 4}) {
    SCOPED_TRACE(prefix);
    net_engine e;
    program p(e);
    const auto l = e.listen(local(framing::length_prefixed(static_cast<std::uint8_t>(prefix))));
    ASSERT_NE(l, 0u);
    client c(e.port(l));
    const conn_id id = p.accept_one();
    const std::vector<std::string> payloads = {"", "x", std::string(200, 'y'), "tail"};
    std::string wire;
    for (const auto &pl : payloads)
      wire += be(pl.size(), prefix) + pl;
    // The first half a byte at a time (a prefix split across reads), the rest at once.
    const std::size_t half = wire.size() / 2;
    for (std::size_t i = 0; i < half; ++i) {
      c.write(wire.substr(i, 1));
      if (i % 50 == 0)
        std::this_thread::sleep_for(2ms);
    }
    c.write(wire.substr(half));
    ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == payloads.size(); }));
    EXPECT_EQ(p.frames_of(id), payloads);
  }
}

TEST(NetEngineFraming, RawFramesAreReadsOfAtMostMaxFrame) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw(), 4096));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  std::string data;
  for (int i = 0; i < 100000; ++i)
    data.push_back(static_cast<char>('a' + i % 26));
  c.write(data);
  std::string got;
  ASSERT_TRUE(p.until([&] {
    got.clear();
    for (const auto &f : p.s.frames[id])
      got += f;
    return got.size() >= data.size();
  }));
  EXPECT_EQ(got, data);
  for (const auto &f : p.s.frames[id])
    EXPECT_LE(f.size(), 4096u);
}

TEST(NetEngineFraming, AFrameOfMaxFrameIsKeptAndOneByteMoreCloses) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::delimited('|'), 8));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  c.write("1234567|");  // 8 bytes with the delimiter: kept
  c.write("123456789"); // past 8 with no delimiter yet: closed before the delimiter comes
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::frame_too_long);
  EXPECT_EQ(p.frames_of(id), (std::vector<std::string>{"1234567|"}));
  EXPECT_TRUE(c.closed());
}

TEST(NetEngineFraming, AnOversizedFramesSplitAcrossReadsCloses) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::delimited('\n'), 1000));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  for (int i = 0; i < 20; ++i) {
    c.write(std::string(100, 'z'));
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::frame_too_long);
  EXPECT_TRUE(p.frames_of(id).empty());
}

TEST(NetEngineFraming, ALengthPastMaxFrameClosesAtItsPrefix) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::length_prefixed(2), 100));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  c.write(be(200, 2)); // no payload follows: the prefix alone closes it
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::frame_too_long);
}

// NMDC's client-to-client transfer: "$ADCSND ... n|", then n raw bytes, then commands again, all
// in one write. A pause after the $ADCSND frame lets the program set raw(n) with nothing split
// ahead; raw(n) pauses again after its n bytes for the switch back.
TEST(NetEngineFraming, APausedConnectionSwitchesFramingExactlyAtTheFrame) {
  net_engine e;
  program p(e);
  framing f = framing::delimited('|');
  f.pause_after = {"$ADCSND"};
  const auto l = e.listen(local(f));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  const std::string blob = "bin|ary|\x00|data"; // delimiters inside, which raw must not split on
  std::string wire = "$Hello|$ADCSND file x 0 " + std::to_string(blob.size()) + "|";
  wire += blob;
  wire += "$Next|$Last|";
  p.on_frame = [&](const net_event &ev, const std::string &frame) {
    if (frame.rfind("$ADCSND", 0) == 0)
      e.set_framing(ev.conn, framing::raw(blob.size()));
  };
  c.write(wire);
  std::string raw;
  ASSERT_TRUE(p.until([&] {
    raw.clear();
    const auto &fs = p.s.frames[id];
    for (std::size_t i = 2; i < fs.size(); ++i)
      raw += fs[i];
    return raw.size() >= blob.size();
  }));
  EXPECT_EQ(raw, blob);
  p.idle(50ms); // paused: no further frames until the program says
  const std::size_t before = p.s.frames[id].size();
  e.set_framing(id, framing::delimited('|'));
  ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == before + 2; }));
  const auto fs = p.frames_of(id);
  EXPECT_EQ(fs[0], "$Hello|");
  EXPECT_EQ(fs[fs.size() - 2], "$Next|");
  EXPECT_EQ(fs.back(), "$Last|");
}

TEST(NetEngineFraming, SetFramingAppliesToTheNextUnsplitByte) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::delimited('\n')));
  client c(e.port(l));
  const conn_id id = p.accept_one();
  c.write("BINARY\n");
  ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == 1; }));
  e.set_framing(id, framing::length_prefixed(4));
  e.send(id, "ok\n");
  ASSERT_EQ(c.read(3), "ok\n"); // the switch is applied: the reply follows it
  c.write(be(3, 4) + "a\nb" + be(1, 4) + "c");
  ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == 3; }));
  EXPECT_EQ(p.frames_of(id), (std::vector<std::string>{"BINARY\n", "a\nb", "c"}));
}

// ── output: order, groups ────────────────────────────────────────────────────────────────────

TEST(NetEngineOutput, SendsAndBroadcastsKeepTheirOrderOnEveryConnection) {
  net_engine e;
  program p(e);
  ASSERT_GE(e.io_threads(), 1u);
  const auto l = e.listen(local(framing::delimited('|')));
  client a(e.port(l)), b(e.port(l));
  const conn_id ia = p.id_of(a), ib = p.id_of(b);
  e.join(ia, "g");
  e.join(ib, "g");
  std::string want_a, want_b;
  for (int i = 0; i < 3000; ++i) {
    const std::string m = std::to_string(i) + "|";
    switch (i % 3) {
    case 0:
      e.send(ia, "a" + m);
      want_a += "a" + m;
      break;
    case 1:
      e.broadcast("g", "g" + m);
      want_a += "g" + m;
      want_b += "g" + m;
      break;
    default:
      e.send(ib, "b" + m);
      want_b += "b" + m;
      break;
    }
  }
  EXPECT_EQ(a.read(want_a.size()), want_a);
  EXPECT_EQ(b.read(want_b.size()), want_b);
}

TEST(NetEngineOutput, AJoinIssuedBeforeABroadcastIncludesTheConnection) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw()));
  std::vector<std::unique_ptr<client>> cs;
  for (int i = 0; i < 20; ++i)
    cs.push_back(std::make_unique<client>(e.port(l)));
  std::vector<conn_id> ids;
  for (const auto &c : cs)
    ids.push_back(p.id_of(*c));
  for (std::size_t i = 0; i < cs.size(); ++i) {
    e.join(ids[i], "room"); // each join, then at once a broadcast
    e.broadcast("room", "m" + std::to_string(i) + ";");
  }
  // The i-th client joined before broadcast i: it has broadcasts i..19.
  for (std::size_t i = 0; i < cs.size(); ++i) {
    std::string want;
    for (std::size_t k = i; k < cs.size(); ++k)
      want += "m" + std::to_string(k) + ";";
    EXPECT_EQ(cs[i]->read(want.size()), want) << i;
  }
}

TEST(NetEngineGroups, ExcludeLeaveAndCloseLeaveEveryGroup) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw()));
  client c1(e.port(l));
  const conn_id i1 = p.accept_one(0);
  client c2(e.port(l));
  const conn_id i2 = p.accept_one(1);
  client c3(e.port(l));
  const conn_id i3 = p.accept_one(2);
  for (conn_id id : {i1, i2, i3}) {
    e.join(id, "all");
    e.join(id, "all"); // twice is once
  }
  e.join(i3, "ops");
  e.broadcast("all", "x", i1);
  EXPECT_EQ(c2.read(1), "x");
  EXPECT_EQ(c3.read(1), "x");
  e.leave(i2, "all");
  e.broadcast("all", "y");
  EXPECT_EQ(c1.read(1), "y");
  EXPECT_EQ(c3.read(1), "y");
  e.broadcast("ops", "o");
  EXPECT_EQ(c3.read(1), "o");
  e.close(i3, false);
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(i3) > 0; }));
  const auto before = e.stats().engine.broadcast_receivers;
  e.broadcast("all", "z");
  e.broadcast("ops", "nobody");
  EXPECT_EQ(c1.read(1), "z");
  EXPECT_EQ(c2.read(1, 100ms), ""); // it left
  ASSERT_TRUE(p.until([&] { return e.stats().engine.broadcast_receivers == before + 1; }));
  p.idle(50ms);
  EXPECT_EQ(e.stats().engine.broadcast_receivers, before + 1);
  EXPECT_EQ(p.s.closed[i3], close_reason::program);
}

TEST(NetEngineOutput, SendManyReachesEachOnceAndBroadcastsAnUnknownGroupReachNobody) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw()));
  client c1(e.port(l)), c2(e.port(l)), c3(e.port(l));
  const conn_id i1 = p.id_of(c1), i3 = p.id_of(c3);
  ASSERT_NE(p.id_of(c2), 0u);
  e.send_many({i1, i3, 0, 999999}, "hi");
  e.broadcast("nobody", "never");
  EXPECT_EQ(c1.read(2), "hi");
  EXPECT_EQ(c3.read(2), "hi");
  EXPECT_EQ(c2.read(1, 100ms), "");
}

// ── backpressure, closes ─────────────────────────────────────────────────────────────────────

TEST(NetEngineBackpressure, ASlowReaderClosesPastMaxQueued) {
  net_engine e;
  program p(e);
  auto o = local(framing::raw());
  o.max_queued = 256 * 1024;
  const auto l = e.listen(o);
  client c(e.port(l), 4096); // never reads
  const conn_id id = p.accept_one();
  const auto chunk = std::make_shared<const std::string>(64 * 1024, 'q');
  for (int i = 0; i < 400 && !p.s.closed.count(id); ++i) {
    e.send(id, chunk);
    if (i % 16 == 0)
      p.idle(2ms);
  }
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::slow_reader);
  const auto st = e.stats();
  EXPECT_EQ(st.engine.slow_reader(), 1u);
  EXPECT_EQ(st.engine.queued_bytes, 0u);
  EXPECT_GT(st.engine.max_queued_bytes, 0u);
  EXPECT_LE(st.engine.max_queued_bytes, o.max_queued);
  EXPECT_EQ(st.listeners.at(0).counters.slow_reader(), 1u);
}

TEST(NetEngineBackpressure, WritableComesOnceTheQueueDrainsToTheMark) {
  net_engine e;
  program p(e);
  auto o = local(framing::raw());
  o.max_queued = 256 * 1024 * 1024;
  const auto l = e.listen(o);
  client c(e.port(l), 4096);
  const conn_id id = p.accept_one();
  const auto chunk = std::make_shared<const std::string>(1024 * 1024, 'w');
  std::size_t sent = 0;
  for (int i = 0; i < 64 && e.stats().engine.queued_bytes == 0; ++i) {
    e.send(id, chunk);
    sent += chunk->size();
    p.idle(20ms);
  }
  ASSERT_GT(e.stats().engine.queued_bytes, 0u) << "the socket never filled";
  e.want_writable(id, 0);
  p.idle(30ms);
  EXPECT_EQ(p.s.writable[id], 0); // still queued: nobody reads
  EXPECT_EQ(c.read(sent, 20000ms).size(), sent);
  ASSERT_TRUE(p.until([&] { return p.s.writable[id] == 1; }));
  e.want_writable(id, 0); // already drained: at once
  ASSERT_TRUE(p.until([&] { return p.s.writable[id] == 2; }));
}

TEST(NetEngineClose, AFlushingCloseSendsWhatIsQueuedThenEnds) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw()));
  client c(e.port(l), 4096);
  const conn_id id = p.accept_one();
  const auto chunk = std::make_shared<const std::string>(64 * 1024, 'f');
  for (int i = 0; i < 64; ++i)
    e.send(id, chunk);
  e.close(id, true);
  e.send(id, "after the close: dropped");
  c.write("unread input|"); // must not turn the close into a reset
  const std::string got = c.read(64 * chunk->size() + 100, 20000ms);
  EXPECT_EQ(got.size(), 64 * chunk->size());
  EXPECT_TRUE(c.closed());
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::program);
}

TEST(NetEngineClose, AClosingWithoutFlushEndsAtOnceAndPeerClosesAreSaid) {
  net_engine e;
  program p(e);
  const auto l = e.listen(local(framing::raw()));
  client c1(e.port(l));
  const conn_id i1 = p.accept_one(0);
  client c2(e.port(l));
  const conn_id i2 = p.accept_one(1);
  e.close(i1, false);
  EXPECT_TRUE(c1.closed());
  boost::system::error_code ec;
  c2.s.close(ec);
  ASSERT_TRUE(p.until([&] { return p.s.closed.size() == 2; }));
  EXPECT_EQ(p.s.closed[i1], close_reason::program);
  EXPECT_EQ(p.s.closed[i2], close_reason::peer_closed);
  const auto st = e.stats();
  EXPECT_EQ(st.engine.open, 0u);
  EXPECT_EQ(st.engine.accepted, 2u);
}

// ── limits ───────────────────────────────────────────────────────────────────────────────────

TEST(NetEngineLimits, TheFirstFrameMustComeInTime) {
  net_engine e;
  program p(e);
  auto o = local(framing::delimited('|'));
  o.limits.first_frame_timeout = 250ms;
  const auto l = e.listen(o);
  client quiet(e.port(l));
  const conn_id iq = p.accept_one(0);
  client prompt(e.port(l));
  const conn_id ip = p.accept_one(1);
  prompt.write("$Key|");
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(iq) > 0; }));
  EXPECT_EQ(p.s.closed[iq], close_reason::first_frame_timeout);
  p.idle(300ms);
  EXPECT_EQ(p.s.closed.count(ip), 0u);
  EXPECT_TRUE(quiet.closed());
}

TEST(NetEngineLimits, BytesAndFramesPerWindow) {
  net_engine e;
  program p(e);
  auto ob = local(framing::delimited('|'));
  ob.limits.bytes_per_window = 1000;
  ob.limits.window = 10000ms;
  auto of = local(framing::delimited('|'));
  of.limits.frames_per_window = 5;
  of.limits.window = 10000ms;
  const auto lb = e.listen(ob), lf = e.listen(of);
  client cb(e.port(lb));
  const conn_id ib = p.accept_one(0);
  client cf(e.port(lf));
  const conn_id iff = p.accept_one(1);
  cb.write(std::string(600, 'b') + "|");
  std::this_thread::sleep_for(20ms);
  cb.write(std::string(600, 'b') + "|");
  std::string ten;
  for (int i = 0; i < 10; ++i)
    ten += "f|";
  cf.write(ten);
  ASSERT_TRUE(p.until([&] { return p.s.closed.size() == 2; }));
  EXPECT_EQ(p.s.closed[ib], close_reason::bytes_limit);
  EXPECT_EQ(p.s.closed[iff], close_reason::frames_limit);
  EXPECT_LE(p.s.frames[iff].size(), 5u);
  EXPECT_LE(p.s.frames[ib].size(), 1u);
}

// ── the engine ───────────────────────────────────────────────────────────────────────────────

TEST(NetEngine, CountersIdsAndInfo) {
  net_engine e;
  program p(e);
  auto o = local(framing::delimited('|'));
  o.name = "nmdc";
  const auto l = e.listen(o);
  client c1(e.port(l)), c2(e.port(l));
  const conn_id a = p.id_of(c1), b = p.id_of(c2);
  EXPECT_NE(a, 0u);
  EXPECT_NE(a, b);
  EXPECT_EQ(p.s.info[a].peer_address, "127.0.0.1");
  EXPECT_EQ(p.s.info[a].local_port, e.port(l));
  EXPECT_EQ(p.s.info[a].listener, l);
  c1.write("one|two|");
  c2.write("three|");
  ASSERT_TRUE(p.until([&] { return p.s.frames[a].size() + p.s.frames[b].size() == 3; }));
  e.join(a, "g");
  e.join(b, "g");
  e.broadcast("g", "hello");
  EXPECT_EQ(c1.read(5), "hello");
  EXPECT_EQ(c2.read(5), "hello");
  p.idle(20ms);
  const auto st = e.stats();
  EXPECT_EQ(st.engine.accepted, 2u);
  EXPECT_EQ(st.engine.open, 2u);
  EXPECT_EQ(st.engine.frames_in, 3u);
  EXPECT_EQ(st.engine.bytes_in, 14u);
  EXPECT_EQ(st.engine.broadcasts, 1u);
  EXPECT_EQ(st.engine.broadcast_receivers, 2u);
  EXPECT_EQ(st.engine.messages_out, 2u);
  EXPECT_EQ(st.engine.bytes_out, 10u);
  ASSERT_EQ(st.listeners.size(), 1u);
  EXPECT_EQ(st.listeners[0].name, "nmdc");
  EXPECT_EQ(st.listeners[0].counters.frames_in, 3u);
  EXPECT_EQ(st.threads.size(), e.io_threads());
  std::uint64_t busy = 0, open = 0;
  for (const auto &t : st.threads) {
    busy += t.busy_ns;
    open += t.connections;
  }
  EXPECT_GT(busy, 0u);
  EXPECT_EQ(open, 2u);
}

// One group with members from two listeners: each listener counts the receivers it gave, and only
// the engine counts the broadcasts.
TEST(NetEngine, EachListenerCountsItsOwnBroadcastReceivers) {
  net_engine e;
  program p(e);
  auto oa = local(framing::raw());
  oa.name = "hub";
  auto ob = local(framing::raw());
  ob.name = "admin";
  const auto la = e.listen(oa), lb = e.listen(ob);
  ASSERT_NE(la, 0u);
  ASSERT_NE(lb, 0u);
  client a1(e.port(la)), a2(e.port(la)), b1(e.port(lb));
  const conn_id ia1 = p.id_of(a1), ia2 = p.id_of(a2), ib1 = p.id_of(b1);
  for (conn_id id : {ia1, ia2, ib1})
    e.join(id, "all");
  e.broadcast("all", "x");      // a1, a2, b1
  e.broadcast("all", "y", ia1); // a2, b1
  EXPECT_EQ(a1.read(1), "x");
  EXPECT_EQ(a2.read(2), "xy");
  EXPECT_EQ(b1.read(2), "xy");
  ASSERT_TRUE(p.until([&] { return e.stats().engine.broadcast_receivers == 5; }));
  const auto st = e.stats();
  EXPECT_EQ(st.engine.broadcasts, 2u);
  ASSERT_EQ(st.listeners.size(), 2u);
  EXPECT_EQ(st.listeners[0].id, la);
  EXPECT_EQ(st.listeners[0].name, "hub");
  EXPECT_EQ(st.listeners[0].counters.broadcast_receivers, 3u);
  EXPECT_EQ(st.listeners[0].counters.messages_out, 3u);
  EXPECT_EQ(st.listeners[0].counters.broadcasts, 0u);
  EXPECT_EQ(st.listeners[1].id, lb);
  EXPECT_EQ(st.listeners[1].name, "admin");
  EXPECT_EQ(st.listeners[1].counters.broadcast_receivers, 2u);
  EXPECT_EQ(st.listeners[1].counters.messages_out, 2u);
  EXPECT_EQ(st.listeners[1].counters.broadcasts, 0u);
}

TEST(NetEngine, ABadAddressOrABusyPortFailsToListen) {
  net_engine e;
  std::string why;
  auto o = local(framing::raw());
  o.address = "not-an-address";
  EXPECT_EQ(e.listen(o, &why), 0u);
  EXPECT_FALSE(why.empty());
  const auto l = e.listen(local(framing::raw()));
  ASSERT_NE(l, 0u);
  auto again = local(framing::raw());
  again.port = e.port(l);
  why.clear();
  // A second bind of the same port: refused where the system refuses it (SO_REUSEADDR does not
  // share a listening port on Linux or macOS).
#if !defined(_WIN32)
  EXPECT_EQ(e.listen(again, &why), 0u);
  EXPECT_FALSE(why.empty());
#endif
}

TEST(NetEngine, StopClosesEveryConnectionAndLaterCommandsDoNothing) {
  auto e = std::make_unique<net_engine>();
  program p(*e);
  const auto l = e->listen(local(framing::raw()));
  client c1(e->port(l)), c2(e->port(l));
  ASSERT_TRUE(p.until([&] { return p.s.accepted.size() == 2; }));
  std::atomic<int> notified{0};
  e->set_notify([&] { ++notified; });
  e->stop();
  e->send(p.s.accepted[0], "late");
  net_batch b;
  while (e->poll(b))
    p.absorb(b);
  EXPECT_EQ(p.s.closed.size(), 2u);
  for (const auto &[id, why] : p.s.closed)
    EXPECT_EQ(why, close_reason::engine_stopped);
  EXPECT_GE(notified.load(), 1);
  EXPECT_FALSE(e->wait(b, 10ms)); // stopped and empty: no wait
  EXPECT_TRUE(c1.closed());
  e.reset();
  EXPECT_TRUE(c2.closed());
}

TEST(NetEngine, ABatchKeepsItsFramesAliveUntilItIsCleared) {
  net_engine e;
  const auto l = e.listen(local(framing::delimited('\n')));
  client c(e.port(l));
  net_batch keep;
  std::vector<std::string_view> views;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  c.write("first\nsecond\n");
  net_batch b;
  while (views.size() < 2 && std::chrono::steady_clock::now() < deadline) {
    if (!e.wait(b, 10ms))
      continue;
    for (const auto &ev : b.events)
      for (auto f : ev.frames)
        views.push_back(f);
    if (!views.empty()) {
      keep.events.insert(keep.events.end(), b.events.begin(), b.events.end());
      keep.keep.insert(keep.keep.end(), b.keep.begin(), b.keep.end());
    }
  }
  b.clear();
  c.write("more\nand more\n"); // new reads must not land in the kept buffers
  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(views.size(), 2u);
  EXPECT_EQ(views[0], "first\n");
  EXPECT_EQ(views[1], "second\n");
}

// ── TLS (N3) ─────────────────────────────────────────────────────────────────────────────────

#if CVC_NET_TEST_TLS

std::string hex_of(const unsigned char *p, std::size_t n) {
  static const char digits[] = "0123456789abcdef";
  std::string out;
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(digits[p[i] >> 4]);
    out.push_back(digits[p[i] & 15]);
  }
  return out;
}

// RFC 4648 base32 of a hex string, unpadded: the keyprint's digits.
std::string base32_of_hex(const std::string &h) {
  static const char digits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
  std::string bits;
  for (char c : h) {
    const int v = std::stoi(std::string(1, c), nullptr, 16);
    for (int b = 3; b >= 0; --b)
      bits.push_back((v >> b) & 1 ? '1' : '0');
  }
  while (bits.size() % 5)
    bits.push_back('0');
  std::string out;
  for (std::size_t i = 0; i < bits.size(); i += 5)
    out.push_back(digits[std::stoi(bits.substr(i, 5), nullptr, 2)]);
  return out;
}

std::string cert_hex(X509 *x) {
  unsigned char md[32];
  unsigned int len = 0;
  X509_digest(x, EVP_sha256(), md, &len);
  return hex_of(md, len);
}

std::string tls_dir() { return CVC_NET_TEST_TLS_DIR; }

// A directory of the test's own files, removed with it.
struct temp_dir {
  std::filesystem::path path;
  temp_dir() {
    static std::atomic<int> n{0};
    path = std::filesystem::temp_directory_path() /
           ("cvc_net_tls_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
            std::to_string(n++));
    std::filesystem::create_directories(path);
  }
  ~temp_dir() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
  std::string file(const std::string &name) const { return (path / name).string(); }
};

// A self-signed P-256 certificate and its key, as PEM strings, and its DER's SHA-256 in hex.
struct test_cert {
  std::string cert_pem, key_pem, sha256;
};

test_cert make_cert(const std::string &cn) {
  static std::atomic<long> serial{1};
  test_cert out;
  EVP_PKEY *key = nullptr;
  EVP_PKEY_CTX *kc = EVP_PKEY_CTX_new_id(EVP_PKEY_EC, nullptr);
  EVP_PKEY_keygen_init(kc);
  EVP_PKEY_CTX_set_ec_paramgen_curve_nid(kc, NID_X9_62_prime256v1);
  EVP_PKEY_keygen(kc, &key);
  EVP_PKEY_CTX_free(kc);
  X509 *x = X509_new();
  X509_set_version(x, 2);
  ASN1_INTEGER_set(X509_get_serialNumber(x), serial++);
  X509_gmtime_adj(X509_getm_notBefore(x), -3600);
  X509_gmtime_adj(X509_getm_notAfter(x), 30L * 86400);
  X509_set_pubkey(x, key);
  X509_NAME *name = X509_get_subject_name(x);
  X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                             reinterpret_cast<const unsigned char *>(cn.c_str()), -1, -1, 0);
  X509_set_issuer_name(x, name);
  X509_sign(x, key, EVP_sha256());
  auto pem = [](auto write) {
    BIO *b = BIO_new(BIO_s_mem());
    write(b);
    char *data = nullptr;
    const long n = BIO_get_mem_data(b, &data);
    std::string s(data, static_cast<std::size_t>(n));
    BIO_free(b);
    return s;
  };
  out.cert_pem = pem([&](BIO *b) { PEM_write_bio_X509(b, x); });
  out.key_pem =
      pem([&](BIO *b) { PEM_write_bio_PrivateKey(b, key, nullptr, nullptr, 0, nullptr, nullptr); });
  out.sha256 = cert_hex(x);
  X509_free(x);
  EVP_PKEY_free(key);
  return out;
}

// Replace a file at once (a new one renamed over it), its time moved on, so a watch sees it
// change even where file times are coarse.
void write_file(const std::string &path, const std::string &bytes) {
  static int step = 0;
  const std::string next = path + ".new";
  std::FILE *f = std::fopen(next.c_str(), "wb");
  ASSERT_NE(f, nullptr) << next;
  std::fwrite(bytes.data(), 1, bytes.size(), f);
  std::fclose(f);
  std::filesystem::last_write_time(next, std::filesystem::file_time_type::clock::now() +
                                             std::chrono::seconds(++step * 2));
  // Windows refuses to replace a file while the watch has it open: try again.
  std::error_code ec;
  for (int i = 0; i < 100; ++i) {
    std::filesystem::rename(next, path, ec);
    if (!ec)
      break;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_FALSE(ec) << path << ": " << ec.message();
}

// A certificate's two files, one after the other.
void put_cert(const std::string &cert, const std::string &key, const test_cert &c) {
  write_file(cert, c.cert_pem);
  write_file(key, c.key_pem);
}

// A TLS client over the blocking-with-timeouts client's socket: OpenSSL on its non-blocking
// descriptor, polled.
struct tls_client_options {
  int min_version = 0, max_version = 0; // TLS1_2_VERSION, ...; 0: OpenSSL's
  std::string sni;
  std::string cert_pem, key_pem; // a client certificate
};

struct tls_client {
  using options = tls_client_options;
  client c;
  SSL_CTX *ctx = nullptr;
  SSL *ssl = nullptr;
  std::string error;

  explicit tls_client(std::uint16_t port, const options &o = {}, int receive_buffer = 0)
      : c(port, receive_buffer) {
    ctx = SSL_CTX_new(TLS_client_method());
    if (o.min_version)
      SSL_CTX_set_min_proto_version(ctx, o.min_version);
    if (o.max_version)
      SSL_CTX_set_max_proto_version(ctx, o.max_version);
    if (!o.cert_pem.empty()) {
      BIO *b = BIO_new_mem_buf(o.cert_pem.data(), static_cast<int>(o.cert_pem.size()));
      X509 *x = PEM_read_bio_X509(b, nullptr, nullptr, nullptr);
      BIO_free(b);
      b = BIO_new_mem_buf(o.key_pem.data(), static_cast<int>(o.key_pem.size()));
      EVP_PKEY *k = PEM_read_bio_PrivateKey(b, nullptr, nullptr, nullptr);
      BIO_free(b);
      SSL_CTX_use_certificate(ctx, x);
      SSL_CTX_use_PrivateKey(ctx, k);
      X509_free(x);
      EVP_PKEY_free(k);
    }
    SSL_CTX_set_verify(ctx, SSL_VERIFY_NONE, nullptr); // the tests read the server's digest
    ssl = SSL_new(ctx);
    SSL_set_fd(ssl, static_cast<int>(c.s.native_handle()));
    if (!o.sni.empty())
      SSL_set_tlsext_host_name(ssl, o.sni.c_str());
    SSL_set_connect_state(ssl);
  }
  ~tls_client() {
    SSL_free(ssl);
    SSL_CTX_free(ctx);
  }

  // Whether the call wants to be tried again (and wait a little); sets `error` when not.
  bool again(int r) {
    const int e = SSL_get_error(ssl, r);
    if (e == SSL_ERROR_WANT_READ || e == SSL_ERROR_WANT_WRITE) {
      std::this_thread::sleep_for(1ms);
      return true;
    }
    const unsigned long code = ERR_get_error();
    const char *why = code ? ERR_reason_error_string(code) : nullptr;
    error = why ? why : "ssl error " + std::to_string(e);
    ERR_clear_error();
    return false;
  }

  bool handshake(std::chrono::milliseconds timeout = 5000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const int r = SSL_do_handshake(ssl);
      if (r == 1)
        return true;
      if (!again(r))
        return false;
    }
    error = "timed out";
    return false;
  }

  void write(const std::string &bytes) {
    std::size_t done = 0;
    while (done < bytes.size()) {
      const int r = SSL_write(ssl, bytes.data() + done, static_cast<int>(bytes.size() - done));
      if (r > 0)
        done += static_cast<std::size_t>(r);
      else if (!again(r))
        return;
    }
  }

  std::string read(std::size_t n, std::chrono::milliseconds timeout = 5000ms) {
    std::string out;
    std::vector<char> buf(65536);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (out.size() < n && std::chrono::steady_clock::now() < deadline) {
      const int r =
          SSL_read(ssl, buf.data(), static_cast<int>(std::min(buf.size(), n - out.size())));
      if (r > 0)
        out.append(buf.data(), static_cast<std::size_t>(r));
      else if (!again(r))
        break;
    }
    return out;
  }

  // Whether the server ends the session (close_notify, end of stream or a reset) in time.
  bool closed(std::chrono::milliseconds timeout = 5000ms) {
    char buf[4096];
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const int r = SSL_read(ssl, buf, sizeof buf);
      if (r <= 0 && !again(r))
        return true;
    }
    return false;
  }

  std::string version() const { return SSL_get_version(ssl); }
  std::string server_sha256() const {
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
    X509 *x = SSL_get1_peer_certificate(ssl);
#else
    X509 *x = SSL_get_peer_certificate(ssl);
#endif
    if (!x)
      return {};
    std::string h = cert_hex(x);
    X509_free(x);
    return h;
  }
};

// src/arisrv/tests/tls/localhost.pem's keyprint. Its SHA-256 is what
//   openssl x509 -in localhost.pem -outform DER | openssl dgst -sha256
// prints, 42081986f376739721e6594713fd38f141b987c2d063b3eef32edd137e92c899, and these digits are
// Python's base64.b32encode of it, unpadded.
const char *kLocalhostKeyprint = "SHA256/IIEBTBXTOZZZOIPGLFDRH7JY6FA3TB6C2BR3H3XTF3ORG7USZCMQ";

listener_options tls_local(framing f, const std::string &cert, const std::string &key) {
  listener_options o = local(std::move(f));
  o.tls.certificate_file = cert;
  o.tls.key_file = key;
  return o;
}

listener_options localhost_tls(framing f) {
  return tls_local(std::move(f), tls_dir() + "/localhost.pem", tls_dir() + "/localhost.key");
}

TEST(NetEngineTls, HandshakesOnTls12And13WithTheirVersionCipherSniAndPeerCertificate) {
  ASSERT_TRUE(net_engine::tls_available());
  net_engine e;
  program p(e);
  std::string err;
  const auto l = e.listen(localhost_tls(framing::delimited('|')), &err);
  ASSERT_NE(l, 0u) << err;
  EXPECT_EQ(e.tls_keyprint(l), kLocalhostKeyprint);

  tls_client::options o12;
  o12.max_version = TLS1_2_VERSION;
  o12.sni = "hub.example";
  tls_client a(e.port(l), o12);
  ASSERT_TRUE(a.handshake()) << a.error;
  EXPECT_EQ(a.version(), "TLSv1.2");
  EXPECT_EQ(a.server_sha256(), "42081986f376739721e6594713fd38f141b987c2d063b3eef32edd137e92c899");
  a.write("$Key a|$ValidateNick a|");
  const conn_id ia = p.id_of(a.c);
  ASSERT_TRUE(p.until([&] { return p.s.frames[ia].size() == 2; }));
  EXPECT_EQ(p.frames_of(ia), (std::vector<std::string>{"$Key a|", "$ValidateNick a|"}));
  const auto &info_a = p.s.info[ia];
  EXPECT_TRUE(info_a.tls());
  EXPECT_EQ(info_a.tls_version, "TLSv1.2");
  EXPECT_FALSE(info_a.tls_cipher.empty());
  EXPECT_EQ(info_a.tls_sni, "hub.example");
  EXPECT_TRUE(info_a.tls_peer_sha256.empty()); // it sent no certificate
  e.send(ia, "$Hello a|");
  EXPECT_EQ(a.read(9), "$Hello a|");

  const test_cert mine = make_cert("client");
  tls_client::options o13;
  o13.min_version = TLS1_3_VERSION;
  o13.cert_pem = mine.cert_pem;
  o13.key_pem = mine.key_pem;
  tls_client b(e.port(l), o13);
  ASSERT_TRUE(b.handshake()) << b.error;
  EXPECT_EQ(b.version(), "TLSv1.3");
  const conn_id ib = p.id_of(b.c);
  const auto &info_b = p.s.info[ib];
  EXPECT_EQ(info_b.tls_version, "TLSv1.3");
  EXPECT_EQ(info_b.tls_cipher.rfind("TLS_", 0), 0u) << info_b.tls_cipher;
  EXPECT_TRUE(info_b.tls_sni.empty());
  EXPECT_EQ(info_b.tls_peer_sha256, mine.sha256);
  EXPECT_EQ(info_b.tls_peer_keyprint, "SHA256/" + base32_of_hex(mine.sha256));

  // Many small sends (packed into records) and one large one (records straight from its buffer),
  // in order.
  std::string want;
  for (int i = 0; i < 500; ++i) {
    const std::string line = "<hub> line " + std::to_string(i) + "|";
    e.send(ib, line);
    want += line;
  }
  std::string big(1 << 20, '\0');
  for (std::size_t i = 0; i < big.size(); ++i)
    big[i] = static_cast<char>('a' + (i * 7919) % 26);
  e.send(ib, big);
  want += big;
  EXPECT_EQ(b.read(want.size(), 20000ms), want);
  b.write("$MyINFO b|");
  ASSERT_TRUE(p.until([&] { return p.s.frames[ib].size() == 1; }));

  const auto st = e.stats();
  EXPECT_EQ(st.engine.tls_accepted, 2u);
  EXPECT_EQ(st.listeners.at(0).counters.tls_accepted, 2u);
  EXPECT_GT(st.engine.bytes_out, want.size()); // the records' overhead
}

TEST(NetEngineTls, AVersionBelowTheMinimumAndAStalledHandshakeCloseUnseen) {
  net_engine e;
  program p(e);
  auto o = localhost_tls(framing::delimited('|'));
  o.tls.min_version = cvc::net::tls_protocol::tls1_3;
  o.tls.handshake_timeout = 300ms;
  const auto l = e.listen(o);
  ASSERT_NE(l, 0u);

  tls_client::options o12;
  o12.max_version = TLS1_2_VERSION;
  tls_client old(e.port(l), o12);
  EXPECT_FALSE(old.handshake());
  EXPECT_FALSE(old.error.empty());

  client stalled(e.port(l)); // connects and says nothing
  EXPECT_TRUE(stalled.closed(5000ms));

  client plain(e.port(l)); // plain bytes on a TLS port
  plain.write("$Supports NoGetINFO|");
  EXPECT_TRUE(plain.closed(5000ms));

  const auto refused = [&] {
    return e.stats().engine.closed[static_cast<std::size_t>(close_reason::tls_handshake)];
  };
  ASSERT_TRUE(p.until([&] { return refused() == 3; }));
  p.idle(50ms);
  EXPECT_TRUE(p.s.accepted.empty()); // the program never heard of them
  EXPECT_TRUE(p.s.closed.empty());
  const auto st = e.stats();
  EXPECT_EQ(st.engine.accepted, 3u);
  EXPECT_EQ(st.engine.open, 0u);
  EXPECT_EQ(st.engine.tls_accepted, 0u);
}

TEST(NetEngineTls, TheKeyprintIsTheCertificatesDerSha256InBase32) {
  net_engine e;
  const auto l = e.listen(localhost_tls(framing::raw()));
  ASSERT_NE(l, 0u);
  const std::string kp = e.tls_keyprint(l);
  EXPECT_EQ(kp, kLocalhostKeyprint);
  EXPECT_EQ(kp.size(), 7u + 52u);

  // The DER read here and hashed with EVP_Digest, apart from the engine's X509_digest.
  BIO *b = BIO_new_file((tls_dir() + "/localhost.pem").c_str(), "r");
  ASSERT_NE(b, nullptr);
  X509 *x = PEM_read_bio_X509(b, nullptr, nullptr, nullptr);
  BIO_free(b);
  ASSERT_NE(x, nullptr);
  unsigned char *der = nullptr;
  const int der_len = i2d_X509(x, &der);
  X509_free(x);
  ASSERT_GT(der_len, 0);
  unsigned char md[32];
  unsigned int md_len = 0;
  EVP_Digest(der, static_cast<std::size_t>(der_len), md, &md_len, EVP_sha256(), nullptr);
  OPENSSL_free(der);
  EXPECT_EQ(kp, "SHA256/" + base32_of_hex(hex_of(md, md_len)));

#if !defined(_WIN32)
  // And the openssl command, where there is one.
  const std::string cmd = "openssl x509 -in '" + tls_dir() +
                          "/localhost.pem' -outform DER 2>/dev/null | openssl dgst -sha256 -r "
                          "2>/dev/null";
  if (std::FILE *f = popen(cmd.c_str(), "r")) {
    char line[256] = {};
    const bool got = std::fgets(line, sizeof line, f) != nullptr;
    pclose(f);
    const std::string h = got ? std::string(line).substr(0, 64) : std::string();
    if (h.size() == 64 && h.find_first_not_of("0123456789abcdef") == std::string::npos)
      EXPECT_EQ(kp, "SHA256/" + base32_of_hex(h));
  }
#endif

  const auto plain = e.listen(local(framing::raw()));
  EXPECT_TRUE(e.tls_keyprint(plain).empty());
  EXPECT_TRUE(e.tls_keyprint(9999).empty());
}

TEST(NetEngineTls, AReloadServesNewConnectionsAndKeepsOpenOnes) {
  temp_dir dir;
  const std::string cert = dir.file("hub.pem"), key = dir.file("hub.key");
  const test_cert ca = make_cert("a"), cb = make_cert("b");
  put_cert(cert, key, ca);
  net_engine e;
  program p(e);
  const auto l = e.listen(tls_local(framing::delimited('|'), cert, key));
  ASSERT_NE(l, 0u);
  EXPECT_EQ(e.tls_keyprint(l), "SHA256/" + base32_of_hex(ca.sha256));

  tls_client first(e.port(l));
  ASSERT_TRUE(first.handshake()) << first.error;
  EXPECT_EQ(first.server_sha256(), ca.sha256);
  const conn_id i1 = p.id_of(first.c);

  put_cert(cert, key, cb);
  std::string err;
  ASSERT_TRUE(e.reload_tls(l, &err)) << err;
  EXPECT_EQ(e.tls_keyprint(l), "SHA256/" + base32_of_hex(cb.sha256));
  ASSERT_TRUE(p.until([&] { return p.s.reloads.size() == 1; }));
  EXPECT_EQ(p.s.reloads[0], "SHA256/" + base32_of_hex(cb.sha256));

  tls_client second(e.port(l));
  ASSERT_TRUE(second.handshake()) << second.error;
  EXPECT_EQ(second.server_sha256(), cb.sha256);

  // The first connection kept its session, both ways.
  e.send(i1, "still here|");
  EXPECT_EQ(first.read(11), "still here|");
  first.write("me too|");
  ASSERT_TRUE(p.until([&] { return p.s.frames[i1].size() == 1; }));
  EXPECT_FALSE(p.s.closed.count(i1));

  // A file that does not load leaves the certificate in use.
  write_file(cert, "not a certificate");
  EXPECT_FALSE(e.reload_tls(l, &err));
  EXPECT_NE(err.find(cert), std::string::npos) << err;
  EXPECT_EQ(e.tls_keyprint(l), "SHA256/" + base32_of_hex(cb.sha256));
  tls_client third(e.port(l));
  ASSERT_TRUE(third.handshake()) << third.error;
  EXPECT_EQ(third.server_sha256(), cb.sha256);

  // New options: other files.
  const std::string cert2 = dir.file("other.pem"), key2 = dir.file("other.key");
  put_cert(cert2, key2, ca);
  cvc::net::tls_options other;
  other.certificate_file = cert2;
  other.key_file = key2;
  ASSERT_TRUE(e.reload_tls(l, other, &err)) << err;
  tls_client fourth(e.port(l));
  ASSERT_TRUE(fourth.handshake()) << fourth.error;
  EXPECT_EQ(fourth.server_sha256(), ca.sha256);

  const auto plain = e.listen(local(framing::raw()));
  EXPECT_FALSE(e.reload_tls(plain, &err));
  EXPECT_FALSE(e.reload_tls(12345, &err));
}

TEST(NetEngineTls, AWatchedListenerReloadsWhenItsFilesChange) {
  temp_dir dir;
  const std::string cert = dir.file("hub.pem"), key = dir.file("hub.key");
  const test_cert ca = make_cert("a"), cb = make_cert("b"), cc = make_cert("c");
  put_cert(cert, key, ca);
  net_engine e;
  program p(e);
  auto o = tls_local(framing::delimited('|'), cert, key);
  o.tls.watch = 50ms;
  const auto l = e.listen(o);
  ASSERT_NE(l, 0u);
  tls_client first(e.port(l));
  ASSERT_TRUE(first.handshake()) << first.error;
  const conn_id i1 = p.id_of(first.c);

  put_cert(cert, key, cb);
  ASSERT_TRUE(p.until([&] { return p.s.reloads.size() == 1; }));
  EXPECT_EQ(p.s.reloads[0], "SHA256/" + base32_of_hex(cb.sha256));
  tls_client second(e.port(l));
  ASSERT_TRUE(second.handshake()) << second.error;
  EXPECT_EQ(second.server_sha256(), cb.sha256);

  // A broken file is said once (not every tick), and the certificate in use stays.
  write_file(cert, "-----BEGIN CERTIFICATE-----\ntruncated");
  ASSERT_TRUE(p.until([&] { return p.s.errors.size() == 1; }));
  EXPECT_NE(p.s.errors[0].find("TLS reload"), std::string::npos) << p.s.errors[0];
  p.idle(300ms);
  EXPECT_EQ(p.s.errors.size(), 1u);
  EXPECT_EQ(e.tls_keyprint(l), "SHA256/" + base32_of_hex(cb.sha256));

  put_cert(cert, key, cc);
  ASSERT_TRUE(p.until([&] { return p.s.reloads.size() == 2; }));
  EXPECT_EQ(e.tls_keyprint(l), "SHA256/" + base32_of_hex(cc.sha256));
  e.send(i1, "kept|");
  EXPECT_EQ(first.read(5), "kept|");
}

// A plain port that also speaks TLS, for a protocol whose server speaks first: each connection is
// greeted once the engine has decided its layer.
TEST(NetEngineTlsDetect, TlsPlainSlowAndSilentClientsOnOnePort) {
  net_engine e;
  program p(e);
  auto o = localhost_tls(framing::delimited('|'));
  o.detect_tls = 400ms;
  const auto l = e.listen(o);
  ASSERT_NE(l, 0u);
  std::set<conn_id> greeted;
  const auto greet = [&] {
    for (conn_id id : p.s.accepted)
      if (greeted.insert(id).second)
        e.send(id, "$Lock EXTENDEDPROTOCOL|");
  };
  const auto accepted = [&](const client &c) {
    const conn_id id = p.id_of(c);
    greet();
    return id;
  };
  const std::string lock = "$Lock EXTENDEDPROTOCOL|";

  // TLS at once: greeted over TLS, its `accepted` already knowing it.
  {
    tls_client t(e.port(l));
    ASSERT_TRUE(t.handshake()) << t.error;
    const conn_id id = accepted(t.c);
    EXPECT_EQ(p.s.info[id].tls_version.substr(0, 5), "TLSv1");
    EXPECT_EQ(t.read(lock.size()), lock);
  }
  // A plain client that speaks first: plain at once, its input framed.
  {
    client c(e.port(l));
    c.write("$Supports NoHello|");
    const conn_id id = accepted(c);
    EXPECT_FALSE(p.s.info[id].tls());
    ASSERT_TRUE(p.until([&] { return p.s.frames[id].size() == 1; }));
    EXPECT_EQ(p.frames_of(id)[0], "$Supports NoHello|");
    EXPECT_EQ(c.read(lock.size()), lock);
  }
  // A slow TLS client, within the wait.
  {
    tls_client t(e.port(l));
    std::this_thread::sleep_for(150ms);
    ASSERT_TRUE(t.handshake()) << t.error;
    const conn_id id = accepted(t.c);
    EXPECT_TRUE(p.s.info[id].tls());
    EXPECT_EQ(t.read(lock.size()), lock);
  }
  // A silent plain client, waiting for the server: plain once the wait is over.
  {
    const auto start = std::chrono::steady_clock::now();
    client c(e.port(l));
    const conn_id id = accepted(c);
    EXPECT_GE(std::chrono::steady_clock::now() - start, 380ms);
    EXPECT_FALSE(p.s.info[id].tls());
    EXPECT_EQ(c.read(lock.size()), lock);
  }
  // A TLS client later than the wait has been decided plain, and greeted in plain: its
  // handshake fails on the greeting.
  {
    tls_client t(e.port(l));
    const conn_id id = accepted(t.c);
    EXPECT_FALSE(p.s.info[id].tls());
    std::this_thread::sleep_for(50ms);
    EXPECT_FALSE(t.handshake(3000ms));
  }
  const auto st = e.stats();
  EXPECT_EQ(st.engine.tls_accepted, 2u);
  EXPECT_EQ(st.engine.accepted, 5u);
}

TEST(NetEngineTlsBackpressure, ASlowTlsReaderClosesPastMaxQueued) {
  net_engine e;
  program p(e);
  auto o = localhost_tls(framing::raw());
  o.max_queued = 256 * 1024;
  const auto l = e.listen(o);
  tls_client c(e.port(l), {}, 4096);
  ASSERT_TRUE(c.handshake()) << c.error; // then never reads
  const conn_id id = p.id_of(c.c);
  const auto chunk = std::make_shared<const std::string>(64 * 1024, 'q');
  for (int i = 0; i < 400 && !p.s.closed.count(id); ++i) {
    e.send(id, chunk);
    if (i % 16 == 0)
      p.idle(2ms);
  }
  ASSERT_TRUE(p.until([&] { return p.s.closed.count(id) > 0; }));
  EXPECT_EQ(p.s.closed[id], close_reason::slow_reader);
  const auto st = e.stats();
  EXPECT_EQ(st.engine.slow_reader(), 1u);
  EXPECT_EQ(st.engine.queued_bytes, 0u);
  EXPECT_LE(st.engine.max_queued_bytes, o.max_queued);
}

TEST(NetEngineTlsBackpressure, WritableComesOnceATlsQueueDrainsToTheMark) {
  net_engine e;
  program p(e);
  auto o = localhost_tls(framing::raw());
  o.max_queued = 256 * 1024 * 1024;
  const auto l = e.listen(o);
  tls_client c(e.port(l), {}, 4096);
  ASSERT_TRUE(c.handshake()) << c.error;
  const conn_id id = p.id_of(c.c);
  const auto chunk = std::make_shared<const std::string>(1024 * 1024, 'w');
  std::size_t sent = 0;
  for (int i = 0; i < 64 && e.stats().engine.queued_bytes == 0; ++i) {
    e.send(id, chunk);
    sent += chunk->size();
    p.idle(20ms);
  }
  ASSERT_GT(e.stats().engine.queued_bytes, 0u) << "the socket never filled";
  e.want_writable(id, 0);
  p.idle(30ms);
  EXPECT_EQ(p.s.writable[id], 0);
  EXPECT_EQ(c.read(sent, 20000ms), std::string(sent, 'w'));
  ASSERT_TRUE(p.until([&] { return p.s.writable[id] == 1; }));
}

TEST(NetEngineTls, ABroadcastReachesAGroupOfTlsAndPlainConnections) {
  net_engine e(cvc::net::engine_options{3});
  program p(e);
  const auto plain_l = e.listen(local(framing::delimited('|')));
  auto o = localhost_tls(framing::delimited('|'));
  o.detect_tls = 300ms;
  const auto both_l = e.listen(o);
  ASSERT_NE(both_l, 0u);
  const auto tls_l = e.listen(localhost_tls(framing::delimited('|')));
  ASSERT_NE(tls_l, 0u);

  std::vector<std::unique_ptr<client>> plains;
  std::vector<std::unique_ptr<tls_client>> tlss;
  for (int i = 0; i < 3; ++i)
    plains.push_back(std::make_unique<client>(e.port(plain_l)));
  for (int i = 0; i < 2; ++i) { // plain on the detecting port: they speak first
    plains.push_back(std::make_unique<client>(e.port(both_l)));
    plains.back()->write("$Hi|");
  }
  for (int i = 0; i < 3; ++i) {
    tlss.push_back(std::make_unique<tls_client>(e.port(i % 2 ? both_l : tls_l)));
    ASSERT_TRUE(tlss.back()->handshake()) << tlss.back()->error;
  }
  std::vector<conn_id> ids;
  for (auto &c : plains)
    ids.push_back(p.id_of(*c));
  for (auto &t : tlss)
    ids.push_back(p.id_of(t->c));
  for (conn_id id : ids)
    e.join(id, "all");
  const conn_id excluded = ids[1];
  std::string want;
  for (int i = 0; i < 200; ++i) {
    const std::string line =
        i == 100 ? std::string(40000, 'x') + "|" : "<hub> broadcast " + std::to_string(i) + "|";
    e.broadcast("all", line, i % 2 ? 0 : excluded);
    want += line;
  }
  std::string want_excluded;
  for (int i = 0; i < 200; ++i)
    if (i % 2)
      want_excluded += "<hub> broadcast " + std::to_string(i) + "|";
  for (std::size_t i = 0; i < plains.size(); ++i) {
    const std::string &w = ids[i] == excluded ? want_excluded : want;
    EXPECT_EQ(plains[i]->read(w.size(), 20000ms), w) << "plain " << i;
  }
  for (std::size_t i = 0; i < tlss.size(); ++i)
    EXPECT_EQ(tlss[i]->read(want.size(), 20000ms), want) << "tls " << i;
  const auto st = e.stats();
  EXPECT_EQ(st.engine.broadcasts, 200u);
  EXPECT_EQ(st.engine.broadcast_receivers, 200u * ids.size() - 100u);
  EXPECT_EQ(st.engine.tls_accepted, 3u);

  // A flushing close sends what is queued, then close_notify.
  for (std::size_t i = 0; i < tlss.size(); ++i) {
    const conn_id id = ids[plains.size() + i];
    e.send(id, "$ForceMove elsewhere|");
    e.close(id, true);
  }
  for (auto &t : tlss) {
    EXPECT_EQ(t->read(21), "$ForceMove elsewhere|");
    EXPECT_TRUE(t->closed());
  }
  ASSERT_TRUE(p.until([&] { return p.s.closed.size() == tlss.size(); }));
  for (const auto &[id, why] : p.s.closed)
    EXPECT_EQ(why, close_reason::program);
}

#endif // CVC_NET_TEST_TLS

// ── fan-out benchmark ────────────────────────────────────────────────────────────────────────

std::size_t fanout_n() {
  if (const char *v = std::getenv("CVC_NET_FANOUT_N"))
    return static_cast<std::size_t>(std::max(1, std::atoi(v)));
#if defined(CVC_NET_TEST_TSAN)
  return 300;
#else
  return 6000;
#endif
}

// Room for 2n descriptors (each connection's two ends), or the n that fits.
std::size_t fit_descriptors(std::size_t n) {
#if !defined(_WIN32)
  rlimit r{};
  if (getrlimit(RLIMIT_NOFILE, &r) == 0) {
    const rlim_t want = static_cast<rlim_t>(2 * n + 256);
    if (r.rlim_cur < want) {
      const rlim_t had = r.rlim_cur;
      r.rlim_cur = std::min(want, r.rlim_max);
      if (setrlimit(RLIMIT_NOFILE, &r) != 0) { // macOS refuses more than OPEN_MAX
        r.rlim_cur = std::max<rlim_t>(had, std::min<rlim_t>(10240, r.rlim_max));
        setrlimit(RLIMIT_NOFILE, &r);
      }
      getrlimit(RLIMIT_NOFILE, &r);
    }
#if defined(__APPLE__)
    // macOS takes a larger limit and still refuses descriptors past OPEN_MAX and
    // kern.maxfilesperproc (a macos-latest run failed its 5115th connect at 10240).
    rlim_t ceiling = OPEN_MAX;
    int per_proc = 0;
    std::size_t len = sizeof per_proc;
    if (sysctlbyname("kern.maxfilesperproc", &per_proc, &len, nullptr, 0) == 0 && per_proc > 0)
      ceiling = std::min<rlim_t>(ceiling, static_cast<rlim_t>(per_proc));
    r.rlim_cur = std::min(r.rlim_cur, ceiling);
#endif
    if (r.rlim_cur < want)
      n = r.rlim_cur > 512 ? static_cast<std::size_t>((r.rlim_cur - 256) / 2) : 128;
  }
#endif
  return n;
}

double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : v[v.size() / 2];
}

TEST(NetEngineFanOut, OneMessageToManyLoopbackReceivers) {
  const std::size_t want = fanout_n();
  const std::size_t n = fit_descriptors(want);
  if (n != want)
    std::printf("[fan-out] descriptor limit: %zu receivers instead of %zu\n", n, want);
  cvc::net::engine_options eo;
  if (const char *v = std::getenv("CVC_NET_FANOUT_THREADS"))
    eo.io_threads = static_cast<unsigned>(std::max(1, std::atoi(v)));
  net_engine e(eo);
  program p(e);
  auto o = local(framing::delimited('|'));
  o.address = "127.0.0.1";
  const auto l = e.listen(o);
  ASSERT_NE(l, 0u);

  // The receivers: one io_context of the test's, two threads, a read of one message per round.
  asio::io_context rctx;
  auto guard = asio::make_work_guard(rctx);
  std::vector<std::thread> readers;
  std::vector<std::unique_ptr<tcp::socket>> socks;
  socks.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    auto s = std::make_unique<tcp::socket>(rctx);
    boost::system::error_code ec;
    s->connect({asio::ip::make_address("127.0.0.1"), e.port(l)}, ec);
    ASSERT_FALSE(ec) << "connect " << i << ": " << ec.message();
    socks.push_back(std::move(s));
    if (i % 512 == 0)
      p.idle(0ms);
  }
  ASSERT_TRUE(p.until([&] { return p.s.accepted.size() == n; }, 60000ms));
  for (conn_id id : p.s.accepted)
    e.join(id, "all");
  for (int t = 0; t < 2; ++t)
    readers.emplace_back([&] { rctx.run(); });

  const std::string msg = "<sublevels> " + std::string(100, 'm') + "|"; // a chat line
  std::vector<std::string> bufs(n);
  std::string expect;
  std::atomic<std::size_t> done{0};
  std::atomic<std::size_t> bad{0};
  auto arm = [&](int burst) {
    expect.clear();
    for (int k = 0; k < burst; ++k)
      expect += msg;
    done = 0;
    for (std::size_t i = 0; i < n; ++i) {
      bufs[i].assign(expect.size(), '\0');
      asio::async_read(*socks[i], asio::buffer(bufs[i]),
                       [&, i](const boost::system::error_code &ec, std::size_t) {
                         if (ec || bufs[i] != expect)
                           ++bad;
                         ++done;
                       });
    }
  };
  auto busy_now = [&] {
    std::uint64_t ns = 0;
    for (const auto &t : e.stats().threads)
      ns += t.busy_ns;
    return ns;
  };

#if defined(CVC_NET_TEST_TSAN)
  const int rounds = 3;
#else
  const int rounds = 21;
#endif
  std::uint64_t sent = 0;
  // `burst` broadcasts issued back to back a round: the I/O threads gather what reaches a
  // connection in one turn into one write.
  auto measure = [&](int burst) {
    std::vector<double> call_us, busy_us, per_message_us, wall_ms;
    for (int r = 0; r < rounds + 1; ++r) { // round 0 warms up
      arm(burst);
      p.idle(5ms);
      const auto before = busy_now();
      const auto t0 = std::chrono::steady_clock::now();
      for (int k = 0; k < burst; ++k)
        e.broadcast("all", msg);
      const auto t1 = std::chrono::steady_clock::now();
      sent += static_cast<std::uint64_t>(burst) * n;
      const auto deadline = t1 + 60s;
      while (done.load() < n && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
      const auto t2 = std::chrono::steady_clock::now();
      ASSERT_EQ(done.load(), n) << "round " << r;
      p.idle(5ms); // the turn's last accounting
      if (r == 0)
        continue;
      const double busy = static_cast<double>(busy_now() - before) / 1000.0;
      call_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count() / burst);
      busy_us.push_back(busy);
      per_message_us.push_back(busy / static_cast<double>(n) / burst);
      wall_ms.push_back(std::chrono::duration<double, std::milli>(t2 - t0).count());
    }
    std::printf("[fan-out] burst=%d: broadcast() call median %.1f us; I/O threads' busy time "
                "median %.2f ms in all, %.3f us per receiver per message; all received (the "
                "test's readers included) median %.2f ms\n",
                burst, median(call_us), median(busy_us) / 1000.0, median(per_message_us),
                median(wall_ms));
    if (burst == 1)
      RecordProperty("us_per_receiver_x1000", static_cast<int>(median(per_message_us) * 1000));
  };
  std::printf("[fan-out] receivers=%zu io_threads=%u message=%zu B rounds=%d\n", n, e.io_threads(),
              msg.size(), rounds);
  measure(1);
  measure(10);
  EXPECT_EQ(bad.load(), 0u);
  EXPECT_EQ(e.stats().engine.broadcast_receivers, sent);
  RecordProperty("receivers", static_cast<int>(n));

  guard.reset();
  for (auto &s : socks) {
    boost::system::error_code ec;
    s->close(ec);
  }
  rctx.stop();
  for (auto &t : readers)
    t.join();
}

#if CVC_NET_TEST_TLS
// What TLS adds to a broadcast: one 113-byte line to a group of TLS receivers and to a plain group
// of the same size on the same engine, rounds interleaved; the I/O threads' busy time a receiver
// for each (CVC_NET_TLS_FANOUT_N receivers a group, 500 by default; CVC_NET_FANOUT_THREADS).
TEST(NetEngineTlsFanOut, OneMessageToTlsAndPlainReceivers) {
#if defined(CVC_NET_TEST_TSAN)
  std::size_t want = 40;
  const int rounds = 3;
#else
  std::size_t want = 500;
  const int rounds = 11;
#endif
  if (const char *v = std::getenv("CVC_NET_TLS_FANOUT_N"))
    want = static_cast<std::size_t>(std::max(1, std::atoi(v)));
  const std::size_t n = std::max<std::size_t>(1, fit_descriptors(2 * want) / 2);
  cvc::net::engine_options eo;
  if (const char *v = std::getenv("CVC_NET_FANOUT_THREADS"))
    eo.io_threads = static_cast<unsigned>(std::max(1, std::atoi(v)));
  net_engine e(eo);
  program p(e);
  const auto plain_l = e.listen(local(framing::delimited('|')));
  const auto tls_l = e.listen(localhost_tls(framing::delimited('|')));
  ASSERT_NE(tls_l, 0u);
  std::vector<std::unique_ptr<client>> plains;
  std::vector<std::unique_ptr<tls_client>> tlss;
  for (std::size_t i = 0; i < n; ++i) {
    plains.push_back(std::make_unique<client>(e.port(plain_l)));
    tlss.push_back(std::make_unique<tls_client>(e.port(tls_l)));
    ASSERT_TRUE(tlss.back()->handshake()) << i << ": " << tlss.back()->error;
    if (i % 64 == 0)
      p.idle(0ms);
  }
  ASSERT_TRUE(p.until([&] { return p.s.accepted.size() == 2 * n; }, 60000ms));
  for (conn_id id : p.s.accepted)
    e.join(id, p.s.info[id].tls() ? "tls" : "plain");

  const std::string msg = "<sublevels> " + std::string(100, 'm') + "|";
  auto busy_now = [&] {
    std::uint64_t ns = 0;
    for (const auto &t : e.stats().threads)
      ns += t.busy_ns;
    return ns;
  };
  std::uint64_t reached = 0;
  // One round: `burst` broadcasts to `group`; the busy time a receiver a message, in us.
  auto round = [&](const char *group, int burst) {
    p.idle(2ms);
    const auto before = busy_now();
    for (int k = 0; k < burst; ++k)
      e.broadcast(group, msg);
    reached += static_cast<std::uint64_t>(burst) * n;
    EXPECT_TRUE(p.until(
        [&] {
          const auto st = e.stats();
          return st.engine.broadcast_receivers == reached && st.engine.queued_bytes == 0;
        },
        60000ms));
    p.idle(5ms); // the turn's last accounting
    const double us = static_cast<double>(busy_now() - before) / 1000.0;
    const std::string want_bytes = [&] {
      std::string s;
      for (int k = 0; k < burst; ++k)
        s += msg;
      return s;
    }();
    std::size_t bad = 0;
    if (std::strcmp(group, "tls") == 0) {
      for (auto &t : tlss)
        bad += t->read(want_bytes.size(), 20000ms) != want_bytes;
    } else {
      for (auto &c : plains)
        bad += c->read(want_bytes.size(), 20000ms) != want_bytes;
    }
    EXPECT_EQ(bad, 0u) << group;
    return us / static_cast<double>(n) / burst;
  };
  for (int burst : {1, 10}) {
    std::vector<double> tls_us, plain_us;
    for (int r = 0; r < rounds + 1; ++r) { // round 0 warms up
      const double t = round("tls", burst), pl = round("plain", burst);
      if (r > 0) {
        tls_us.push_back(t);
        plain_us.push_back(pl);
      }
    }
    std::printf("[tls fan-out] receivers=%zu a group, io_threads=%u, burst=%d: I/O busy time "
                "median %.3f us per TLS receiver per message, %.3f us per plain one\n",
                n, e.io_threads(), burst, median(tls_us), median(plain_us));
    if (burst == 1) {
      RecordProperty("tls_us_per_receiver_x1000", static_cast<int>(median(tls_us) * 1000));
      RecordProperty("plain_us_per_receiver_x1000", static_cast<int>(median(plain_us) * 1000));
    }
  }
  RecordProperty("receivers", static_cast<int>(n));
}
#endif // CVC_NET_TEST_TLS

} // namespace

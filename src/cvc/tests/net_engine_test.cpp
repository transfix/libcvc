// cvc::net's connection engine (inc/cvc/net/net_engine.h, docs/roadmap/ARIADNE_NET_ENGINE.md N1):
// framing in its three modes and their switches, the order of sends and broadcasts, groups,
// backpressure, the inbound limits, the counters, and a fan-out benchmark (one message to N
// loopback connections; CVC_NET_FANOUT_N sets N, 6000 by default, and CVC_NET_FANOUT_THREADS the
// engine's I/O threads, 2 by default).
//
// The clients are plain Asio sockets on the test's side; the engine runs its own threads.

#include <algorithm>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/net/net_engine.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

} // namespace

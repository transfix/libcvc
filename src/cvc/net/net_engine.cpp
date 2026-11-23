// cvc::net — the native connection engine (inc/cvc/net/net_engine.h,
// docs/roadmap/ARIADNE_NET_ENGINE.md N1).
//
// Each I/O thread is a `worker`: an io_context, the connections pinned to it, its share of every
// group (a group's members on that thread), and its commands. A command for one connection goes
// to that connection's worker; a broadcast goes to every worker, each of which walks its own
// members. Commands are queued under one mutex, so every worker sees them in the order they were
// issued.
//
// Every handler on a worker is a `turn`. During a turn the worker only queues: output goes onto
// connections' queues (and the connection onto the dirty list), events onto the worker's own list,
// closed connections onto the reap list. The turn's end writes each dirty connection once,
// gathering whatever queued in the turn into one write, then reaps, then posts the events to the
// program under one lock.
//
// Sockets are non-blocking. Reads wait for readiness and then read into the worker's buffer; the
// bytes that hold whole frames are copied once into a refcounted buffer the frames point into, so
// an idle connection holds no read buffer. Writes go straight to the socket and wait for
// readiness only when it is full.
//
// On Windows the read wait is already IOCP's: Asio waits for a stream socket's input with a
// zero-byte overlapped WSARecv. Its wait for room to write goes through a select() thread beside
// IOCP, though, which is slow with thousands of sockets, so there a full socket gets the rest of
// its output as an overlapped write (async_write_some) instead.

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <cvc/net/net_engine.h>
#include <deque>
#include <iterator>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#if !defined(__EMSCRIPTEN__)
#include <boost/asio.hpp>
#endif

namespace cvc {
namespace net {

const char *to_string(close_reason reason) {
  switch (reason) {
  case close_reason::program:
    return "program";
  case close_reason::peer_closed:
    return "peer_closed";
  case close_reason::error:
    return "error";
  case close_reason::frame_too_long:
    return "frame_too_long";
  case close_reason::slow_reader:
    return "slow_reader";
  case close_reason::bytes_limit:
    return "bytes_limit";
  case close_reason::frames_limit:
    return "frames_limit";
  case close_reason::first_frame_timeout:
    return "first_frame_timeout";
  case close_reason::engine_stopped:
    return "engine_stopped";
  }
  return "unknown";
}

#if defined(__EMSCRIPTEN__)

struct net_engine::impl {};
net_engine::net_engine(engine_options) : m_(std::make_unique<impl>()) {}
net_engine::~net_engine() = default;
bool net_engine::available() { return false; }
unsigned net_engine::io_threads() const { return 0; }
listener_id net_engine::listen(const listener_options &, std::string *error) {
  if (error)
    *error = "a browser page cannot listen";
  return 0;
}
std::uint16_t net_engine::port(listener_id) const { return 0; }
void net_engine::stop_listening(listener_id) {}
void net_engine::send(conn_id, std::string_view) {}
void net_engine::send(conn_id, payload) {}
void net_engine::send_many(const std::vector<conn_id> &, std::string_view) {}
void net_engine::broadcast(const std::string &, std::string_view, conn_id) {}
void net_engine::broadcast(const std::string &, payload, conn_id) {}
void net_engine::close(conn_id, bool) {}
void net_engine::join(conn_id, const std::string &) {}
void net_engine::leave(conn_id, const std::string &) {}
void net_engine::set_framing(conn_id, framing) {}
void net_engine::resume(conn_id) {}
void net_engine::want_writable(conn_id, std::size_t) {}
bool net_engine::wait(net_batch &out, std::chrono::milliseconds) {
  out.clear();
  return false;
}
bool net_engine::poll(net_batch &out) {
  out.clear();
  return false;
}
void net_engine::set_notify(std::function<void()>) {}
net_stats net_engine::stats() const { return {}; }
void net_engine::stop() {}

#else

// Writes that find the socket full go on as overlapped writes on Windows (see the top).
// -DCVC_NET_ASYNC_WRITES=1 takes that path elsewhere too, to test it.
#if !defined(CVC_NET_ASYNC_WRITES)
#if defined(_WIN32)
#define CVC_NET_ASYNC_WRITES 1
#else
#define CVC_NET_ASYNC_WRITES 0
#endif
#endif

namespace asio = boost::asio;
using tcp = asio::ip::tcp;
using steady = std::chrono::steady_clock;

namespace {

constexpr std::size_t kGatherBuffers = 64; // Asio passes at most 64 buffers to one writev
constexpr std::size_t kGatherBytes = 256 * 1024;
constexpr int kReadsPerTurn = 4; // then other connections get the thread
constexpr std::chrono::milliseconds kAcceptRetry{100};

using counter = std::atomic<std::uint64_t>;

// One thread's counters, for the engine or for one listener. Only the thread that owns the block
// writes it; stats() sums the blocks.
struct alignas(64) counter_block {
  counter open{0}, accepted{0};
  std::array<counter, kCloseReasons> closed{};
  counter bytes_in{0}, frames_in{0}, bytes_out{0}, messages_out{0};
  counter broadcast_receivers{0};
  counter queued_bytes{0}, max_queued_bytes{0};
  counter busy_ns{0}, handlers{0}, connections{0}; // the engine's blocks only

  void add_to(net_counters &c) const {
    c.open += open.load(std::memory_order_relaxed);
    c.accepted += accepted.load(std::memory_order_relaxed);
    for (std::size_t i = 0; i < kCloseReasons; ++i)
      c.closed[i] += closed[i].load(std::memory_order_relaxed);
    c.bytes_in += bytes_in.load(std::memory_order_relaxed);
    c.frames_in += frames_in.load(std::memory_order_relaxed);
    c.bytes_out += bytes_out.load(std::memory_order_relaxed);
    c.messages_out += messages_out.load(std::memory_order_relaxed);
    c.broadcast_receivers += broadcast_receivers.load(std::memory_order_relaxed);
    c.queued_bytes += queued_bytes.load(std::memory_order_relaxed);
    c.max_queued_bytes =
        std::max(c.max_queued_bytes, max_queued_bytes.load(std::memory_order_relaxed));
  }
};

inline void bump(counter &c, std::uint64_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }
inline void drop(counter &c, std::uint64_t n = 1) { c.fetch_sub(n, std::memory_order_relaxed); }

using chunk_ptr = std::shared_ptr<const std::string>;

struct command {
  enum class op : std::uint8_t {
    send,
    send_many,
    broadcast,
    close,
    join,
    leave,
    set_framing,
    resume,
    want_writable,
  };
  op kind = op::send;
  bool flush = false;
  conn_id conn = 0; // broadcast: the excluded connection
  std::vector<conn_id> conns;
  std::string group;
  net_engine::payload data;
  framing frames;
  std::size_t mark = 0;
};

} // namespace

struct worker;
struct lingerer;
struct connection;

struct listener_state {
  listener_id id = 0;
  std::string name;
  listener_options options;
  tcp::acceptor acceptor;
  asio::steady_timer retry;
  std::uint16_t port = 0;
  bool stopped = false;                    // on the acceptor's thread
  std::unique_ptr<counter_block[]> blocks; // one a worker

  listener_state(asio::io_context &ctx, unsigned workers)
      : acceptor(ctx), retry(ctx), blocks(new counter_block[workers]) {}
};

struct net_engine::impl {
  engine_options options;
  std::vector<std::unique_ptr<worker>> workers;

  // Commands: every worker's queue, under one lock, so all of them see one order.
  std::mutex cmd_mu;
  bool stopped = false;
  std::atomic<bool> stopping{false};
  std::atomic<conn_id> next_conn{0};
  counter broadcasts{0};

  // The listeners (their acceptors run on worker 0).
  mutable std::mutex lst_mu;
  std::unordered_map<listener_id, std::shared_ptr<listener_state>> listeners;
  listener_id next_listener = 0;

  // The program's queue.
  std::mutex ev_mu;
  std::condition_variable ev_cv;
  std::vector<net_event> events;
  std::vector<std::shared_ptr<const void>> keep;
  bool ev_closed = false;
  std::shared_ptr<std::function<void()>> notify;

  worker &worker_of(conn_id id);
  void push(worker &w, command c); // cmd_mu held
  void post_events(std::vector<net_event> &evs, std::vector<std::shared_ptr<const void>> &k);
  void start_accept(const std::shared_ptr<listener_state> &l);
  void adopt(worker &w, tcp::socket socket, const std::shared_ptr<listener_state> &l, conn_id id);
  ~impl();
};

struct worker {
  net_engine::impl &engine;
  unsigned index;
  asio::io_context ctx{1};
  std::optional<asio::executor_work_guard<asio::io_context::executor_type>> guard;
  std::thread thread;

  // Under impl::cmd_mu.
  std::vector<command> commands;
  bool drain_posted = false;

  // The thread's own.
  std::unordered_map<conn_id, std::shared_ptr<connection>> conns;
  std::unordered_map<std::string, std::unordered_set<connection *>> groups;
  std::vector<connection *> dirty;
  std::vector<std::shared_ptr<connection>> reap;
  std::unordered_map<lingerer *, std::shared_ptr<lingerer>> lingering;
  std::vector<net_event> out_events;
  std::vector<std::shared_ptr<const void>> out_keep;
  std::vector<char> scratch;
  std::vector<asio::const_buffer> gather;
  std::vector<command> running; // the commands being applied
  counter_block counters;

  worker(net_engine::impl &e, unsigned i, std::size_t read_buffer)
      : engine(e), index(i), scratch(read_buffer) {
    gather.reserve(kGatherBuffers);
  }

  void drain();
  void apply(command &c);
  void end_turn();
  net_event &event_for(conn_id id, listener_id listener);
};

// A closed connection's socket, its output sent and its side ended, read until the peer ends its
// side or the deadline passes. The program has heard of the close already.
struct lingerer : std::enable_shared_from_this<lingerer> {
  worker &w;
  tcp::socket sock;
  asio::steady_timer timer;
  char buf[1024];

  lingerer(worker &wk, tcp::socket s) : w(wk), sock(std::move(s)), timer(wk.ctx) {}

  static void start(worker &w, tcp::socket s, steady::time_point deadline);
  void read() {
    sock.async_read_some(asio::buffer(buf), [self = shared_from_this()](
                                                const boost::system::error_code &ec, std::size_t) {
      if (ec)
        self->end();
      else
        self->read();
    });
  }
  void end();
};

// A handler's run on a worker: its time, then the turn's writes, reaps and events.
struct turn {
  worker &w;
  steady::time_point start = steady::now();
  explicit turn(worker &wk) : w(wk) {}
  ~turn() {
    w.end_turn();
    bump(w.counters.busy_ns,
         static_cast<std::uint64_t>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(steady::now() - start).count()));
    bump(w.counters.handlers);
  }
  turn(const turn &) = delete;
  turn &operator=(const turn &) = delete;
};

struct connection : std::enable_shared_from_this<connection> {
  enum class phase : std::uint8_t { open, closing, closed };

  worker &w;
  conn_id id;
  std::shared_ptr<listener_state> listener;
  counter_block &lst; // the listener's block on this worker
  tcp::socket sock;
  framing frames;
  std::size_t max_frame, max_queued;
  inbound_limits limits;

  phase state = phase::open;
  bool reading = false, writing = false, paused = false, peer_eof = false;
  bool got_frame = false, is_dirty = false, want_w = false;
  std::size_t low_mark = 0;
  std::uint64_t raw_left = 0;

  // Input not yet split (a frame's start, or what a pause held back).
  std::string pending;
  // The window the inbound limits count in.
  steady::time_point window_start;
  std::uint64_t window_bytes = 0, window_frames = 0;

  std::deque<net_engine::payload> out;
  std::size_t front_off = 0; // bytes of out.front() already written
  std::size_t queued = 0;
#if CVC_NET_ASYNC_WRITES
  // The overlapped write's payloads, kept until its handler runs (a close clears `out` first).
  std::vector<net_engine::payload> in_flight;
#endif

  asio::steady_timer timer; // the first frame's deadline, or a flushing close's
  unsigned timer_gen = 0;
  std::vector<std::string> groups;
  close_reason reason = close_reason::program;
  std::string message;

  connection(worker &wk, conn_id i, const std::shared_ptr<listener_state> &l, tcp::socket s)
      : w(wk), id(i), listener(l), lst(l->blocks[wk.index]), sock(std::move(s)),
        frames(l->options.frames), max_frame(std::max<std::size_t>(1, l->options.max_frame)),
        max_queued(l->options.max_queued), limits(l->options.limits), timer(wk.ctx) {
    set_framing(frames);
  }

  void count(counter counter_block::*f, std::uint64_t n = 1) {
    bump(w.counters.*f, n);
    bump(lst.*f, n);
  }
  void uncount(counter counter_block::*f, std::uint64_t n) {
    drop(w.counters.*f, n);
    drop(lst.*f, n);
  }

  void set_framing(const framing &f) {
    frames = f;
    if (frames.kind == framing::mode::length && frames.prefix != 1 && frames.prefix != 2 &&
        frames.prefix != 4)
      frames.prefix = 4;
    raw_left = frames.kind == framing::mode::raw ? frames.raw_bytes : 0;
  }

  // ── timers ────────────────────────────────────────────────────────────────────────────────────
  void arm_timer(std::chrono::milliseconds after, close_reason why) {
    const unsigned gen = ++timer_gen;
    timer.expires_after(after);
    timer.async_wait([self = shared_from_this(), gen, why](const boost::system::error_code &ec) {
      if (ec || gen != self->timer_gen)
        return;
      turn t(self->w);
      if (why == close_reason::first_frame_timeout) {
        if (self->state == phase::open && !self->got_frame)
          self->fail(why);
      } else if (self->state == phase::closing) {
        self->fail(close_reason::program, "the flush timed out");
      }
    });
  }
  void cancel_timer() {
    ++timer_gen;
    timer.cancel();
  }

  // ── input ─────────────────────────────────────────────────────────────────────────────────────
  // Wait for input. On Windows Asio waits with a zero-byte overlapped WSARecv, which goes through
  // IOCP and holds no buffer.
  void start_read() {
    if (reading || state == phase::closed || peer_eof || (state == phase::open && paused))
      return;
    reading = true;
    sock.async_wait(tcp::socket::wait_read,
                    [self = shared_from_this()](const boost::system::error_code &ec) {
                      turn t(self->w);
                      self->reading = false;
                      self->on_readable(ec, t.start);
                    });
  }

  void on_readable(const boost::system::error_code &wait_ec, steady::time_point now) {
    if (state == phase::closed)
      return;
    if (wait_ec) {
      fail(close_reason::error, wait_ec.message());
      return;
    }
    for (int i = 0; i < kReadsPerTurn; ++i) {
      boost::system::error_code ec;
      const std::size_t n = sock.read_some(asio::buffer(w.scratch), ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again)
        break;
      if (ec == asio::error::eof) {
        peer_eof = true;
        if (state == phase::open)
          fail(close_reason::peer_closed);
        return; // closing: the output still drains
      }
      if (ec) {
        fail(close_reason::error, ec.message());
        return;
      }
      if (state != phase::open)
        continue; // closing: what comes in is dropped
      count(&counter_block::bytes_in, n);
      if (limits.bytes_per_window || limits.frames_per_window) {
        if (now - window_start >= limits.window) {
          window_start = now;
          window_bytes = window_frames = 0;
        }
        window_bytes += n;
        if (limits.bytes_per_window && window_bytes > limits.bytes_per_window) {
          fail(close_reason::bytes_limit);
          return;
        }
      }
      consume(w.scratch.data(), n);
      if (state != phase::open || paused)
        return;
      if (n < w.scratch.size())
        break; // most likely all there was
    }
    start_read();
  }

  // Bytes just read: kept as `pending` while they complete no frame, else split.
  void consume(const char *data, std::size_t n) {
    std::size_t scan = 0;
    switch (frames.kind) {
    case framing::mode::delimiter:
      if (!std::memchr(data, frames.delimiter, n)) {
        if (pending.size() + n >= max_frame) {
          fail(close_reason::frame_too_long);
          return;
        }
        pending.append(data, n);
        return;
      }
      scan = pending.size(); // `pending` holds no delimiter
      break;
    case framing::mode::length: {
      const std::size_t have = pending.size() + n;
      if (have < frames.prefix) {
        pending.append(data, n);
        return;
      }
      std::uint64_t len = 0;
      for (std::size_t i = 0; i < frames.prefix; ++i) {
        const unsigned char b =
            static_cast<unsigned char>(i < pending.size() ? pending[i] : data[i - pending.size()]);
        len = (len << 8) | b;
      }
      if (len > max_frame) {
        fail(close_reason::frame_too_long);
        return;
      }
      if (have < frames.prefix + len) {
        pending.append(data, n);
        return;
      }
      break;
    }
    case framing::mode::raw:
      break;
    }
    auto chunk = std::make_shared<std::string>();
    if (pending.empty()) {
      chunk->assign(data, n);
    } else {
      chunk->reserve(pending.size() + n);
      chunk->append(pending);
      chunk->append(data, n);
      pending.clear();
    }
    split(std::move(chunk), scan);
  }

  // Split a buffer into frames from its start; `scan`: where a delimiter may first be. What is
  // left (an unfinished frame, or what follows a pause) becomes `pending`.
  void split(std::shared_ptr<std::string> buf, std::size_t scan) {
    chunk_ptr chunk = std::move(buf);
    const char *base = chunk->data();
    const std::size_t end = chunk->size();
    std::size_t pos = 0;
    bool kept = false;
    while (pos < end && state == phase::open && !paused) {
      std::size_t off = pos, len = 0, next = 0;
      if (frames.kind == framing::mode::delimiter) {
        const std::size_t from = std::max(pos, scan);
        const void *q = std::memchr(base + from, frames.delimiter, end - from);
        if (!q)
          break;
        len = static_cast<const char *>(q) - (base + pos) + 1;
        if (len > max_frame) {
          fail(close_reason::frame_too_long);
          return;
        }
        next = pos + len;
      } else if (frames.kind == framing::mode::length) {
        if (end - pos < frames.prefix)
          break;
        std::uint64_t n = 0;
        for (std::size_t i = 0; i < frames.prefix; ++i)
          n = (n << 8) | static_cast<unsigned char>(base[pos + i]);
        if (n > max_frame) {
          fail(close_reason::frame_too_long);
          return;
        }
        if (end - pos - frames.prefix < n)
          break;
        off = pos + frames.prefix;
        len = static_cast<std::size_t>(n);
        next = off + len;
      } else {
        len = std::min(end - pos, max_frame);
        if (frames.raw_bytes)
          len = static_cast<std::size_t>(std::min<std::uint64_t>(len, raw_left));
        next = pos + len;
      }
      if (!emit(chunk, base + off, len, kept))
        return;
      pos = next;
      if (frames.kind == framing::mode::raw && frames.raw_bytes) {
        raw_left -= len;
        if (raw_left == 0)
          paused = true;
      }
    }
    if (state != phase::open)
      return;
    pending.assign(base + pos, end - pos);
    if (!paused && frames.kind == framing::mode::delimiter && pending.size() >= max_frame)
      fail(close_reason::frame_too_long);
  }

  // One frame to the program, unless a limit closes the connection first.
  bool emit(const chunk_ptr &chunk, const char *data, std::size_t len, bool &kept) {
    if (limits.frames_per_window && ++window_frames > limits.frames_per_window) {
      fail(close_reason::frames_limit);
      return false;
    }
    if (!got_frame) {
      got_frame = true;
      if (limits.first_frame_timeout.count() > 0)
        cancel_timer();
    }
    count(&counter_block::frames_in);
    net_event &ev = w.event_for(id, listener->id);
    ev.frames.emplace_back(data, len);
    if (!kept) {
      w.out_keep.push_back(chunk);
      kept = true;
    }
    for (const std::string &p : frames.pause_after) {
      if (len >= p.size() && std::memcmp(data, p.data(), p.size()) == 0) {
        paused = true;
        break;
      }
    }
    return true;
  }

  // A new framing (or the old one) from the next unsplit byte.
  void unpause(const framing *f) {
    if (state != phase::open)
      return;
    if (f)
      set_framing(*f);
    else if (frames.kind == framing::mode::raw && frames.raw_bytes)
      raw_left = frames.raw_bytes;
    paused = false;
    if (!pending.empty()) {
      auto chunk = std::make_shared<std::string>(std::move(pending));
      pending.clear();
      split(std::move(chunk), 0);
    }
    if (state == phase::open && !paused)
      start_read();
  }

  // ── output ────────────────────────────────────────────────────────────────────────────────────
  void enqueue(const net_engine::payload &p) {
    if (state != phase::open || p->empty())
      return;
    if (queued + p->size() > max_queued) {
      fail(close_reason::slow_reader);
      return;
    }
    out.push_back(p);
    queued += p->size();
    count(&counter_block::queued_bytes, p->size());
    count(&counter_block::messages_out);
    if (queued > w.counters.max_queued_bytes.load(std::memory_order_relaxed))
      w.counters.max_queued_bytes.store(queued, std::memory_order_relaxed);
    if (queued > lst.max_queued_bytes.load(std::memory_order_relaxed))
      lst.max_queued_bytes.store(queued, std::memory_order_relaxed);
    mark_dirty();
  }

  void mark_dirty() {
    if (!is_dirty && !writing) {
      is_dirty = true;
      w.dirty.push_back(this);
    }
  }

  // The front of the queue as one gathered write, in w.gather; its size in bytes.
  std::size_t gather_out() {
    w.gather.clear();
    std::size_t bytes = 0;
    std::size_t skip = front_off;
    for (const auto &p : out) {
      if (w.gather.size() == kGatherBuffers || bytes >= kGatherBytes)
        break;
      w.gather.emplace_back(p->data() + skip, p->size() - skip);
      bytes += p->size() - skip;
      skip = 0;
    }
    return bytes;
  }

  // Write what is queued until the socket is full; then wait for room.
  void write_now() {
    is_dirty = false;
    while (state != phase::closed && !writing && !out.empty()) {
      const std::size_t bytes = gather_out();
      boost::system::error_code ec;
      const std::size_t n = sock.write_some(w.gather, ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again) {
        wait_writable();
        return;
      }
      if (ec) {
        fail(close_reason::error, ec.message());
        return;
      }
      wrote(n);
      if (n < bytes) {
        wait_writable(); // the socket is full
        return;
      }
    }
    if (state == phase::closing && out.empty())
      linger();
  }

  void wrote(std::size_t n) {
    count(&counter_block::bytes_out, n);
    uncount(&counter_block::queued_bytes, n);
    queued -= n;
    while (n) {
      const std::size_t left = out.front()->size() - front_off;
      if (n < left) {
        front_off += n;
        break;
      }
      n -= left;
      front_off = 0;
      out.pop_front();
    }
    if (want_w && queued <= low_mark)
      post_writable();
  }

  void post_writable() {
    want_w = false;
    w.out_events.emplace_back();
    net_event &ev = w.out_events.back();
    ev.kind = net_event::type::writable;
    ev.conn = id;
    ev.listener = listener->id;
  }

#if CVC_NET_ASYNC_WRITES
  // The rest as an overlapped write: Asio copies the buffer list, and `in_flight` keeps the bytes.
  void wait_writable() {
    writing = true;
    gather_out();
    for (std::size_t i = 0; i < w.gather.size(); ++i)
      in_flight.push_back(out[i]);
    sock.async_write_some(
        w.gather, [self = shared_from_this()](const boost::system::error_code &ec, std::size_t n) {
          turn t(self->w);
          self->writing = false;
          self->in_flight.clear();
          if (self->state == phase::closed)
            return;
          if (ec) {
            self->fail(close_reason::error, ec.message());
            return;
          }
          self->wrote(n);
          self->write_now();
        });
  }
#else
  void wait_writable() {
    writing = true;
    sock.async_wait(tcp::socket::wait_write,
                    [self = shared_from_this()](const boost::system::error_code &ec) {
                      turn t(self->w);
                      self->writing = false;
                      if (self->state == phase::closed)
                        return;
                      if (ec) {
                        self->fail(close_reason::error, ec.message());
                        return;
                      }
                      self->write_now();
                    });
  }
#endif

  // ── closing ───────────────────────────────────────────────────────────────────────────────────
  void close(bool flush, std::chrono::milliseconds flush_timeout) {
    if (state != phase::open)
      return;
    if (!flush) {
      fail(close_reason::program);
      return;
    }
    state = phase::closing;
    pending.clear();
    arm_timer(flush_timeout, close_reason::program);
    if (out.empty() && !writing)
      linger();
    else
      start_read(); // drop what comes while the output drains
  }

  // The output is out: end our side and tell the program. The socket goes to a lingerer that
  // reads until the peer ends its side too (within the flush timeout), so input the peer sent that
  // nobody read does not turn the close into a reset that loses our output.
  void linger() {
    if (!peer_eof) {
      boost::system::error_code ec;
      sock.shutdown(tcp::socket::shutdown_send, ec);
      if (!ec)
        lingerer::start(w, std::move(sock), timer.expiry());
    }
    fail(close_reason::program);
  }

  void fail(close_reason why, std::string why_text = std::string()) {
    if (state == phase::closed)
      return;
    state = phase::closed;
    reason = why;
    message = std::move(why_text);
    boost::system::error_code ec;
    sock.close(ec);
    cancel_timer();
    if (queued)
      uncount(&counter_block::queued_bytes, queued);
    queued = 0;
    out.clear();
    pending.clear();
    w.reap.push_back(shared_from_this());
  }
};

void lingerer::start(worker &w, tcp::socket s, steady::time_point deadline) {
  auto l = std::make_shared<lingerer>(w, std::move(s));
  w.lingering.emplace(l.get(), l);
  l->timer.expires_at(deadline);
  l->timer.async_wait([l](const boost::system::error_code &ec) {
    if (!ec)
      l->end();
  });
  l->read();
}

void lingerer::end() {
  boost::system::error_code ec;
  sock.close(ec);
  timer.cancel();
  w.lingering.erase(this); // may destroy it: the handler running holds a reference
}

// ── worker ────────────────────────────────────────────────────────────────────────────────────

net_event &worker::event_for(conn_id id, listener_id listener) {
  if (!out_events.empty()) {
    net_event &last = out_events.back();
    if (last.kind == net_event::type::frames && last.conn == id)
      return last;
  }
  out_events.emplace_back();
  net_event &ev = out_events.back();
  ev.kind = net_event::type::frames;
  ev.conn = id;
  ev.listener = listener;
  return ev;
}

void worker::drain() {
  turn t(*this);
  {
    std::lock_guard<std::mutex> lock(engine.cmd_mu);
    running.swap(commands);
    drain_posted = false;
  }
  for (command &c : running)
    apply(c);
  running.clear();
}

void worker::apply(command &c) {
  if (c.kind == command::op::broadcast) {
    auto g = groups.find(c.group);
    if (g == groups.end())
      return;
    std::uint64_t reached = 0;
    for (connection *conn : g->second) {
      if (conn->id == c.conn || conn->state != connection::phase::open)
        continue;
      conn->enqueue(c.data);
      bump(conn->lst.broadcast_receivers);
      ++reached;
    }
    bump(counters.broadcast_receivers, reached);
    return;
  }
  if (c.kind == command::op::send_many) {
    for (conn_id id : c.conns) {
      auto it = conns.find(id);
      if (it != conns.end())
        it->second->enqueue(c.data);
    }
    return;
  }
  auto it = conns.find(c.conn);
  if (it == conns.end())
    return;
  connection &conn = *it->second;
  switch (c.kind) {
  case command::op::send:
    conn.enqueue(c.data);
    break;
  case command::op::close:
    conn.close(c.flush, conn.listener->options.flush_timeout);
    break;
  case command::op::join:
    if (conn.state == connection::phase::open &&
        std::find(conn.groups.begin(), conn.groups.end(), c.group) == conn.groups.end()) {
      groups[c.group].insert(&conn);
      conn.groups.push_back(std::move(c.group));
    }
    break;
  case command::op::leave: {
    auto g = std::find(conn.groups.begin(), conn.groups.end(), c.group);
    if (g != conn.groups.end()) {
      conn.groups.erase(g);
      auto members = groups.find(c.group);
      if (members != groups.end()) {
        members->second.erase(&conn);
        if (members->second.empty())
          groups.erase(members);
      }
    }
    break;
  }
  case command::op::set_framing:
    conn.unpause(&c.frames);
    break;
  case command::op::resume:
    if (conn.paused)
      conn.unpause(nullptr);
    break;
  case command::op::want_writable:
    if (conn.state != connection::phase::open)
      break;
    conn.want_w = true;
    conn.low_mark = c.mark;
    if (conn.queued <= c.mark)
      conn.post_writable();
    break;
  default:
    break;
  }
}

void worker::end_turn() {
  // Writes first: a connection on the dirty list is alive until the reap below.
  for (std::size_t i = 0; i < dirty.size(); ++i)
    dirty[i]->write_now();
  dirty.clear();
  for (auto &conn : reap) {
    for (const std::string &name : conn->groups) {
      auto g = groups.find(name);
      if (g != groups.end()) {
        g->second.erase(conn.get());
        if (g->second.empty())
          groups.erase(g);
      }
    }
    conn->groups.clear();
    conns.erase(conn->id);
    conn->uncount(&counter_block::open, 1);
    drop(counters.connections);
    bump(counters.closed[static_cast<std::size_t>(conn->reason)]);
    bump(conn->lst.closed[static_cast<std::size_t>(conn->reason)]);
    out_events.emplace_back();
    net_event &ev = out_events.back();
    ev.kind = net_event::type::closed;
    ev.conn = conn->id;
    ev.listener = conn->listener->id;
    ev.reason = conn->reason;
    ev.message = conn->message;
  }
  reap.clear();
  if (!out_events.empty())
    engine.post_events(out_events, out_keep);
}

// ── engine ────────────────────────────────────────────────────────────────────────────────────

worker &net_engine::impl::worker_of(conn_id id) { return *workers[(id - 1) % workers.size()]; }

void net_engine::impl::push(worker &w, command c) {
  w.commands.push_back(std::move(c));
  if (!w.drain_posted) {
    w.drain_posted = true;
    asio::post(w.ctx, [&w] { w.drain(); });
  }
}

void net_engine::impl::post_events(std::vector<net_event> &evs,
                                   std::vector<std::shared_ptr<const void>> &k) {
  std::shared_ptr<std::function<void()>> wake;
  {
    std::lock_guard<std::mutex> lock(ev_mu);
    const bool was_empty = events.empty();
    if (was_empty) {
      events.swap(evs);
      keep.swap(k);
    } else {
      std::move(evs.begin(), evs.end(), std::back_inserter(events));
      std::move(k.begin(), k.end(), std::back_inserter(keep));
    }
    if (was_empty)
      wake = notify;
  }
  evs.clear();
  k.clear();
  ev_cv.notify_one();
  if (wake && *wake)
    (*wake)();
}

void net_engine::impl::start_accept(const std::shared_ptr<listener_state> &l) {
  if (l->stopped || stopping.load())
    return;
  const conn_id id = next_conn.fetch_add(1) + 1;
  worker &target = worker_of(id);
  l->acceptor.async_accept(
      target.ctx, [this, l, id, &target](const boost::system::error_code &ec, tcp::socket s) {
        if (ec == asio::error::operation_aborted || l->stopped)
          return;
        if (!ec) {
          asio::post(target.ctx, [this, &target, l, id, sock = std::move(s)]() mutable {
            adopt(target, std::move(sock), l, id);
          });
          start_accept(l);
          return;
        }
        // Out of descriptors and the like: say so, and try again shortly.
        worker &w0 = *workers[0];
        {
          turn t(w0);
          w0.out_events.emplace_back();
          net_event &ev = w0.out_events.back();
          ev.kind = net_event::type::error;
          ev.listener = l->id;
          ev.message = ec.message();
        }
        l->retry.expires_after(kAcceptRetry);
        l->retry.async_wait([this, l](const boost::system::error_code &e) {
          if (!e)
            start_accept(l);
        });
      });
}

void net_engine::impl::adopt(worker &w, tcp::socket socket,
                             const std::shared_ptr<listener_state> &l, conn_id id) {
  turn t(w);
  boost::system::error_code ec;
  if (stopping.load()) {
    socket.close(ec);
    return;
  }
  const tcp::endpoint peer = socket.remote_endpoint(ec);
  if (ec) { // gone already: the program never hears of it
    socket.close(ec);
    return;
  }
  const tcp::endpoint local = socket.local_endpoint(ec);
  socket.non_blocking(true, ec);
  if (l->options.no_delay)
    socket.set_option(tcp::no_delay(true), ec);
  auto conn = std::make_shared<connection>(w, id, l, std::move(socket));
  conn->window_start = t.start;
  w.conns.emplace(id, conn);
  conn->count(&counter_block::accepted);
  conn->count(&counter_block::open);
  bump(w.counters.connections);

  w.out_events.emplace_back();
  net_event &ev = w.out_events.back();
  ev.kind = net_event::type::accepted;
  ev.conn = id;
  ev.listener = l->id;
  ev.info.id = id;
  ev.info.listener = l->id;
  ev.info.peer_address = peer.address().to_string();
  ev.info.peer_port = peer.port();
  ev.info.local_address = local.address().to_string();
  ev.info.local_port = local.port();

  if (l->options.limits.first_frame_timeout.count() > 0)
    conn->arm_timer(l->options.limits.first_frame_timeout, close_reason::first_frame_timeout);
  conn->start_read();
}

net_engine::impl::~impl() = default;

net_engine::net_engine(engine_options options) : m_(std::make_unique<impl>()) {
  const unsigned cores = std::max(1u, std::thread::hardware_concurrency());
  options.io_threads = std::clamp(options.io_threads, 1u, cores);
  options.read_buffer = std::max<std::size_t>(options.read_buffer, 512);
  m_->options = options;
  for (unsigned i = 0; i < options.io_threads; ++i)
    m_->workers.push_back(std::make_unique<worker>(*m_, i, options.read_buffer));
  for (auto &w : m_->workers) {
    w->guard.emplace(w->ctx.get_executor());
    w->thread = std::thread([wk = w.get()] { wk->ctx.run(); });
  }
}

net_engine::~net_engine() {
  stop();
  {
    std::lock_guard<std::mutex> lock(m_->lst_mu);
    m_->listeners.clear(); // their acceptors before worker 0's io_context
  }
}

bool net_engine::available() { return true; }
unsigned net_engine::io_threads() const { return m_->options.io_threads; }

listener_id net_engine::listen(const listener_options &options, std::string *error) {
  auto fail = [&](const std::string &why) -> listener_id {
    if (error)
      *error = why;
    return 0;
  };
  if (m_->stopping.load())
    return fail("the engine has stopped");
  boost::system::error_code ec;
  const asio::ip::address addr = asio::ip::make_address(options.address, ec);
  if (ec)
    return fail("not an address: " + options.address);
  auto l = std::make_shared<listener_state>(m_->workers[0]->ctx,
                                            static_cast<unsigned>(m_->workers.size()));
  l->options = options;
  l->name = options.name;
  const tcp::endpoint ep(addr, options.port);
  l->acceptor.open(ep.protocol(), ec);
  if (ec)
    return fail(ec.message());
  l->acceptor.set_option(tcp::acceptor::reuse_address(true), ec);
  if (addr.is_v6() && addr.is_unspecified())
    l->acceptor.set_option(asio::ip::v6_only(false), ec); // both families where it may
  l->acceptor.bind(ep, ec);
  if (ec)
    return fail("cannot bind " + options.address + ":" + std::to_string(options.port) + ": " +
                ec.message());
  l->acceptor.listen(std::max(1, options.backlog), ec);
  if (ec)
    return fail(ec.message());
  l->port = l->acceptor.local_endpoint(ec).port();
  {
    std::lock_guard<std::mutex> lock(m_->lst_mu);
    l->id = ++m_->next_listener;
    m_->listeners.emplace(l->id, l);
  }
  asio::post(m_->workers[0]->ctx, [this, l] { m_->start_accept(l); });
  return l->id;
}

std::uint16_t net_engine::port(listener_id listener) const {
  std::lock_guard<std::mutex> lock(m_->lst_mu);
  auto it = m_->listeners.find(listener);
  return it == m_->listeners.end() ? 0 : it->second->port;
}

void net_engine::stop_listening(listener_id listener) {
  std::shared_ptr<listener_state> l;
  {
    std::lock_guard<std::mutex> lock(m_->lst_mu);
    auto it = m_->listeners.find(listener);
    if (it == m_->listeners.end())
      return;
    l = it->second;
  }
  asio::post(m_->workers[0]->ctx, [l] {
    l->stopped = true;
    boost::system::error_code ec;
    l->acceptor.close(ec);
    l->retry.cancel();
  });
}

namespace {
net_engine::payload copy_of(std::string_view bytes) {
  return std::make_shared<const std::string>(bytes);
}
} // namespace

void net_engine::send(conn_id conn, std::string_view bytes) { send(conn, copy_of(bytes)); }

void net_engine::send(conn_id conn, payload bytes) {
  if (!conn || !bytes)
    return;
  command c;
  c.kind = command::op::send;
  c.conn = conn;
  c.data = std::move(bytes);
  std::lock_guard<std::mutex> lock(m_->cmd_mu);
  if (!m_->stopped)
    m_->push(m_->worker_of(conn), std::move(c));
}

void net_engine::send_many(const std::vector<conn_id> &conns, std::string_view bytes) {
  const payload data = copy_of(bytes);
  std::vector<std::vector<conn_id>> by_worker(m_->workers.size());
  for (conn_id id : conns)
    if (id)
      by_worker[(id - 1) % by_worker.size()].push_back(id);
  std::lock_guard<std::mutex> lock(m_->cmd_mu);
  if (m_->stopped)
    return;
  for (std::size_t i = 0; i < by_worker.size(); ++i) {
    if (by_worker[i].empty())
      continue;
    command c;
    c.kind = command::op::send_many;
    c.conns = std::move(by_worker[i]);
    c.data = data;
    m_->push(*m_->workers[i], std::move(c));
  }
}

void net_engine::broadcast(const std::string &group, std::string_view bytes, conn_id exclude) {
  broadcast(group, copy_of(bytes), exclude);
}

void net_engine::broadcast(const std::string &group, payload bytes, conn_id exclude) {
  if (!bytes)
    return;
  bump(m_->broadcasts);
  std::lock_guard<std::mutex> lock(m_->cmd_mu);
  if (m_->stopped)
    return;
  for (auto &w : m_->workers) {
    command c;
    c.kind = command::op::broadcast;
    c.conn = exclude;
    c.group = group;
    c.data = bytes;
    m_->push(*w, std::move(c));
  }
}

namespace {
void push_one(net_engine::impl &m, command c) {
  if (!c.conn)
    return;
  std::lock_guard<std::mutex> lock(m.cmd_mu);
  if (!m.stopped)
    m.push(m.worker_of(c.conn), std::move(c));
}
} // namespace

void net_engine::close(conn_id conn, bool flush) {
  command c;
  c.kind = command::op::close;
  c.conn = conn;
  c.flush = flush;
  push_one(*m_, std::move(c));
}

void net_engine::join(conn_id conn, const std::string &group) {
  command c;
  c.kind = command::op::join;
  c.conn = conn;
  c.group = group;
  push_one(*m_, std::move(c));
}

void net_engine::leave(conn_id conn, const std::string &group) {
  command c;
  c.kind = command::op::leave;
  c.conn = conn;
  c.group = group;
  push_one(*m_, std::move(c));
}

void net_engine::set_framing(conn_id conn, framing frames) {
  command c;
  c.kind = command::op::set_framing;
  c.conn = conn;
  c.frames = std::move(frames);
  push_one(*m_, std::move(c));
}

void net_engine::resume(conn_id conn) {
  command c;
  c.kind = command::op::resume;
  c.conn = conn;
  push_one(*m_, std::move(c));
}

void net_engine::want_writable(conn_id conn, std::size_t low_mark) {
  command c;
  c.kind = command::op::want_writable;
  c.conn = conn;
  c.mark = low_mark;
  push_one(*m_, std::move(c));
}

bool net_engine::wait(net_batch &out, std::chrono::milliseconds timeout) {
  out.clear();
  std::unique_lock<std::mutex> lock(m_->ev_mu);
  m_->ev_cv.wait_for(lock, timeout, [&] { return !m_->events.empty() || m_->ev_closed; });
  out.events.swap(m_->events);
  out.keep.swap(m_->keep);
  return !out.events.empty();
}

bool net_engine::poll(net_batch &out) {
  out.clear();
  std::lock_guard<std::mutex> lock(m_->ev_mu);
  out.events.swap(m_->events);
  out.keep.swap(m_->keep);
  return !out.events.empty();
}

void net_engine::set_notify(std::function<void()> notify) {
  auto fn = notify ? std::make_shared<std::function<void()>>(std::move(notify)) : nullptr;
  std::lock_guard<std::mutex> lock(m_->ev_mu);
  m_->notify = std::move(fn);
}

net_stats net_engine::stats() const {
  net_stats s;
  for (const auto &w : m_->workers) {
    w->counters.add_to(s.engine);
    net_stats::thread_stats t;
    t.busy_ns = w->counters.busy_ns.load(std::memory_order_relaxed);
    t.handlers = w->counters.handlers.load(std::memory_order_relaxed);
    t.connections = w->counters.connections.load(std::memory_order_relaxed);
    s.threads.push_back(t);
  }
  s.engine.broadcasts = m_->broadcasts.load(std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(m_->lst_mu);
  for (const auto &[id, l] : m_->listeners) {
    net_stats::listener_stats ls;
    ls.id = id;
    ls.name = l->name;
    ls.port = l->port;
    for (std::size_t i = 0; i < m_->workers.size(); ++i)
      l->blocks[i].add_to(ls.counters);
    s.listeners.push_back(std::move(ls));
  }
  std::sort(s.listeners.begin(), s.listeners.end(),
            [](const auto &a, const auto &b) { return a.id < b.id; });
  return s;
}

void net_engine::stop() {
  {
    std::lock_guard<std::mutex> lock(m_->cmd_mu);
    if (m_->stopped)
      return;
    m_->stopped = true;
    m_->stopping.store(true);
  }
  std::vector<std::shared_ptr<listener_state>> ls;
  {
    std::lock_guard<std::mutex> lock(m_->lst_mu);
    for (auto &[id, l] : m_->listeners)
      ls.push_back(l);
  }
  asio::post(m_->workers[0]->ctx, [ls] {
    for (auto &l : ls) {
      l->stopped = true;
      boost::system::error_code ec;
      l->acceptor.close(ec);
      l->retry.cancel();
    }
  });
  for (auto &w : m_->workers) {
    asio::post(w->ctx, [wk = w.get()] {
      turn t(*wk);
      std::vector<std::shared_ptr<connection>> all;
      for (auto &[id, c] : wk->conns)
        all.push_back(c);
      for (auto &c : all)
        c->fail(close_reason::engine_stopped);
      std::vector<std::shared_ptr<lingerer>> ls;
      for (auto &[p, l] : wk->lingering)
        ls.push_back(l);
      for (auto &l : ls)
        l->end();
    });
    w->guard.reset();
  }
  for (auto &w : m_->workers)
    if (w->thread.joinable())
      w->thread.join();
  {
    std::lock_guard<std::mutex> lock(m_->ev_mu);
    m_->ev_closed = true;
  }
  m_->ev_cv.notify_all();
}

#endif // !__EMSCRIPTEN__

} // namespace net
} // namespace cvc

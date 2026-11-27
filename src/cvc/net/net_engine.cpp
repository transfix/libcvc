// cvc::net — the native connection engine (inc/cvc/net/net_engine.h,
// docs/roadmap/ARIADNE_NET_ENGINE.md N1, N3).
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
//
// TLS (N3) keeps that model. A TLS connection's OpenSSL session reads and writes through a BIO of
// our own over two strings: the records read from the socket go into `in`, and OpenSSL's records
// come out in `out`, which the connection writes as it writes plain output. Its queue holds the
// program's bytes as on a plain connection (one buffer for every receiver of a broadcast); they
// are encrypted as they are written, small ones packed into one record. Asio's ssl::stream would
// need a read buffer a connection and its own async operations, so only its ssl::context (the
// certificate, the key, the options) is used.

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

#if !defined(__EMSCRIPTEN__) && !defined(__wasi__)
#include <boost/asio.hpp>
#endif

#if !defined(CVC_NET_HAVE_TLS) || defined(__EMSCRIPTEN__)
#undef CVC_NET_HAVE_TLS
#define CVC_NET_HAVE_TLS 0
#endif
#if CVC_NET_HAVE_TLS
#include <boost/asio/ssl/context.hpp>
#include <climits>
#include <filesystem>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
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
  case close_reason::tls_handshake:
    return "tls_handshake";
  }
  return "unknown";
}

// No engine in a browser page, nor under wasip1 (no sockets of its own, no Boost.Asio).
#if defined(__EMSCRIPTEN__) || defined(__wasi__)

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
bool net_engine::tls_available() { return false; }
bool net_engine::reload_tls(listener_id, std::string *error) {
  if (error)
    *error = "a browser page cannot listen";
  return false;
}
bool net_engine::reload_tls(listener_id, const tls_options &, std::string *error) {
  return reload_tls(0, error);
}
std::string net_engine::tls_keyprint(listener_id) const { return {}; }
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
constexpr std::size_t kTlsRecord = 16384;   // the most one TLS record holds
constexpr std::size_t kTlsPass = 64 * 1024; // the output one write's turn encrypts
constexpr std::size_t kTlsKeep = 4096;      // a TLS buffer past this is freed once it empties

using counter = std::atomic<std::uint64_t>;

// One thread's counters, for the engine or for one listener. Only the thread that owns the block
// writes it; stats() sums the blocks.
struct alignas(64) counter_block {
  counter open{0}, accepted{0};
  std::array<counter, kCloseReasons> closed{};
  counter bytes_in{0}, frames_in{0}, bytes_out{0}, messages_out{0};
  counter broadcast_receivers{0};
  counter queued_bytes{0}, max_queued_bytes{0}, tls_accepted{0};
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
    c.tls_accepted += tls_accepted.load(std::memory_order_relaxed);
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

// ── TLS ───────────────────────────────────────────────────────────────────────────────────────

// RFC 4648's base32, unpadded (ADC's keyprints and TTHs).
std::string base32(const unsigned char *p, std::size_t n) {
  static const char digits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
  std::string out;
  out.reserve((n * 8 + 4) / 5);
  unsigned buf = 0;
  int bits = 0;
  for (std::size_t i = 0; i < n; ++i) {
    buf = (buf << 8) | p[i];
    bits += 8;
    while (bits >= 5) {
      out.push_back(digits[(buf >> (bits - 5)) & 31]);
      bits -= 5;
    }
  }
  if (bits > 0)
    out.push_back(digits[(buf << (5 - bits)) & 31]);
  return out;
}

std::string hex(const unsigned char *p, std::size_t n) {
  static const char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(n * 2);
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(digits[p[i] >> 4]);
    out.push_back(digits[p[i] & 15]);
  }
  return out;
}

#if CVC_NET_HAVE_TLS

namespace ssl = asio::ssl;
namespace fs = std::filesystem;

// The two files' modification times, as the watch compares them.
using file_times = std::pair<fs::file_time_type, fs::file_time_type>;

// OpenSSL's reason for the last failure on this thread (and the queue cleared), or `otherwise`.
std::string ssl_reason(const char *otherwise) {
  const unsigned long code = ERR_get_error();
  ERR_clear_error();
  if (code == 0)
    return otherwise;
  if (const char *r = ERR_reason_error_string(code))
    return r;
  char buf[256];
  ERR_error_string_n(code, buf, sizeof buf);
  return buf;
}

// The SHA-256 of a certificate's DER; false when OpenSSL cannot make it.
bool cert_sha256(X509 *cert, unsigned char (&md)[32]) {
  unsigned int len = 0;
  return X509_digest(cert, EVP_sha256(), md, &len) == 1 && len == 32;
}

// A listener's certificate and key, loaded: what new connections start their sessions from. Each
// connection holds the one it started from, so its BIO method outlives the session's BIO.
struct tls_context {
  ssl::context ctx{ssl::context::tls_server};
  BIO_METHOD *bio = nullptr; // tls_session's BIO (no process-wide method: libcvc keeps no globals)
  tls_options options;
  std::string keyprint;
  file_times times; // the files' modification times just before they were read

  tls_context() = default;
  tls_context(const tls_context &) = delete;
  tls_context &operator=(const tls_context &) = delete;
  ~tls_context() {
    if (bio)
      BIO_meth_free(bio);
  }
};

BIO_METHOD *new_bio_method();

file_times times_of(const tls_options &o) {
  std::error_code ec;
  file_times t;
  t.first = fs::last_write_time(o.certificate_file, ec);
  if (ec)
    t.first = fs::file_time_type::min();
  t.second = fs::last_write_time(o.key_file.empty() ? o.certificate_file : o.key_file, ec);
  if (ec)
    t.second = fs::file_time_type::min();
  return t;
}

// Load `o`'s files into a new context; nullptr with `why` when they do not load.
std::shared_ptr<tls_context> load_tls(const tls_options &o, std::string &why) {
  if (!o.enabled()) {
    why = "no certificate file";
    return nullptr;
  }
  auto t = std::make_shared<tls_context>();
  t->options = o;
  t->times = times_of(o);
  t->bio = new_bio_method();
  if (!t->bio) {
    why = ssl_reason("out of memory");
    return nullptr;
  }
  SSL_CTX *c = t->ctx.native_handle();
  boost::system::error_code ec;
  t->ctx.set_options(ssl::context::default_workarounds | ssl::context::no_sslv2 |
                         ssl::context::no_sslv3 | ssl::context::single_dh_use,
                     ec);
  long opts = SSL_OP_NO_COMPRESSION;
#if defined(SSL_OP_NO_RENEGOTIATION)
  opts |= SSL_OP_NO_RENEGOTIATION;
#endif
#if defined(SSL_OP_IGNORE_UNEXPECTED_EOF)
  opts |= SSL_OP_IGNORE_UNEXPECTED_EOF; // a peer that ends without close_notify just closed
#endif
  SSL_CTX_set_options(c, opts);
  SSL_CTX_set_min_proto_version(c, o.min_version == tls_protocol::tls1_3 ? TLS1_3_VERSION
                                                                         : TLS1_2_VERSION);
  // An idle session holds no record buffers; resumption is by ticket, with no server-side cache
  // of a session a connection.
  SSL_CTX_set_mode(c, SSL_MODE_RELEASE_BUFFERS);
  SSL_CTX_set_session_cache_mode(c, SSL_SESS_CACHE_OFF);
  static const unsigned char sid[] = "cvc.net";
  SSL_CTX_set_session_id_context(c, sid, sizeof sid - 1);
  ERR_clear_error();
  if (!o.ciphers.empty() && SSL_CTX_set_cipher_list(c, o.ciphers.c_str()) != 1) {
    why = "the TLS 1.2 ciphers \"" + o.ciphers + "\": " + ssl_reason("none is known");
    return nullptr;
  }
  if (!o.ciphersuites.empty() && SSL_CTX_set_ciphersuites(c, o.ciphersuites.c_str()) != 1) {
    why = "the TLS 1.3 ciphersuites \"" + o.ciphersuites + "\": " + ssl_reason("none is known");
    return nullptr;
  }
  t->ctx.use_certificate_chain_file(o.certificate_file, ec);
  if (ec) {
    why = "the certificate " + o.certificate_file + ": " + ec.message();
    return nullptr;
  }
  const std::string &key = o.key_file.empty() ? o.certificate_file : o.key_file;
  t->ctx.use_private_key_file(key, ssl::context::pem, ec);
  if (ec) {
    why = "the key " + key + ": " + ec.message();
    return nullptr;
  }
  if (SSL_CTX_check_private_key(c) != 1) {
    why = "the key " + key + " is not the certificate's: " + ssl_reason("they do not match");
    return nullptr;
  }
  if (o.request_peer_certificate)
    SSL_CTX_set_verify(c, SSL_VERIFY_PEER | SSL_VERIFY_CLIENT_ONCE,
                       [](int, X509_STORE_CTX *) { return 1; }); // the program judges it
  unsigned char md[32];
  X509 *cert = SSL_CTX_get0_certificate(c);
  if (!cert || !cert_sha256(cert, md)) {
    why = "the certificate " + o.certificate_file + ": " + ssl_reason("no digest");
    return nullptr;
  }
  t->keyprint = "SHA256/" + base32(md, sizeof md);
  ERR_clear_error();
  return t;
}

// One connection's OpenSSL session, over a BIO of `in` (records read, not yet taken) and `out`
// (records to write). Started as a server here; N4's client connections start it as a client
// (SSL_set_connect_state, with SNI, verification and a pin).
struct tls_session {
  SSL *ssl = nullptr;
  std::string in;
  std::size_t in_off = 0;
  std::string out;
  std::size_t out_off = 0;
#if CVC_NET_ASYNC_WRITES
  std::string flight; // an overlapped write's records
#endif
  bool shut = false; // close_notify made

  tls_session() = default;
  tls_session(const tls_session &) = delete;
  tls_session &operator=(const tls_session &) = delete;
  ~tls_session() {
    if (ssl)
      SSL_free(ssl); // and the BIO
  }

  static std::unique_ptr<tls_session> server(tls_context &t, std::string &why) {
    auto s = std::make_unique<tls_session>();
    ERR_clear_error();
    s->ssl = SSL_new(t.ctx.native_handle());
    BIO *b = s->ssl ? BIO_new(t.bio) : nullptr;
    if (!b) {
      why = ssl_reason("out of memory");
      return nullptr;
    }
    BIO_set_data(b, s.get());
    BIO_set_init(b, 1);
    SSL_set_bio(s->ssl, b, b); // one reference, the session's
    SSL_set_accept_state(s->ssl);
    return s;
  }

  bool has_out() const { return out_off < out.size(); }
  void sent(std::size_t n) {
    out_off += n;
    if (out_off == out.size()) {
      out.clear();
      out_off = 0;
      if (out.capacity() > kTlsKeep)
        std::string().swap(out);
    }
  }

  // 1: done; 0: it needs more input; -1: it failed (`why`).
  int handshake(std::string &why) {
    ERR_clear_error();
    const int r = SSL_do_handshake(ssl);
    if (r == 1)
      return 1;
    const int e = SSL_get_error(ssl, r);
    if (e == SSL_ERROR_WANT_READ)
      return 0;
    why = ssl_reason("the handshake failed");
    return -1;
  }

  // The bytes read (>0); 0: none until more input; -1: the peer's close_notify; -2: a failure.
  long read(char *buf, std::size_t len, std::string &why) {
    ERR_clear_error();
    const int r = SSL_read(ssl, buf, static_cast<int>(std::min<std::size_t>(len, INT_MAX)));
    if (r > 0)
      return r;
    const int e = SSL_get_error(ssl, r);
    if (e == SSL_ERROR_WANT_READ)
      return 0;
    if (e == SSL_ERROR_ZERO_RETURN)
      return -1;
    why = ssl_reason("a TLS read failed");
    return -2;
  }

  // Encrypt `n` bytes into `out` (the BIO takes all of it).
  bool write(const char *data, std::size_t n, std::string &why) {
    ERR_clear_error();
    if (SSL_write(ssl, data, static_cast<int>(n)) == static_cast<int>(n))
      return true;
    why = ssl_reason("a TLS write failed");
    return false;
  }

  void shutdown() {
    ERR_clear_error();
    SSL_shutdown(ssl); // our close_notify into `out`; the peer's is not waited for
    ERR_clear_error();
    shut = true;
  }

  void describe(conn_info &info) const {
    info.tls_version = SSL_get_version(ssl);
    if (const char *c = SSL_get_cipher_name(ssl))
      info.tls_cipher = c;
    if (const char *sni = SSL_get_servername(ssl, TLSEXT_NAMETYPE_host_name))
      info.tls_sni = sni;
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
    X509 *peer = SSL_get1_peer_certificate(ssl);
#else
    X509 *peer = SSL_get_peer_certificate(ssl);
#endif
    if (peer) {
      unsigned char md[32];
      if (cert_sha256(peer, md)) {
        info.tls_peer_sha256 = hex(md, sizeof md);
        info.tls_peer_keyprint = "SHA256/" + base32(md, sizeof md);
      }
      X509_free(peer);
    }
  }

  // The BIO.
  static int bio_write(BIO *b, const char *data, int len) {
    BIO_clear_retry_flags(b);
    if (len <= 0)
      return 0;
    static_cast<tls_session *>(BIO_get_data(b))->out.append(data, static_cast<std::size_t>(len));
    return len;
  }
  static int bio_read(BIO *b, char *data, int len) {
    BIO_clear_retry_flags(b);
    auto *s = static_cast<tls_session *>(BIO_get_data(b));
    const std::size_t have = s->in.size() - s->in_off;
    if (have == 0) {
      BIO_set_retry_read(b);
      return -1;
    }
    const std::size_t n = std::min<std::size_t>(have, static_cast<std::size_t>(std::max(len, 0)));
    std::memcpy(data, s->in.data() + s->in_off, n);
    s->in_off += n;
    if (s->in_off == s->in.size()) {
      s->in.clear();
      s->in_off = 0;
      if (s->in.capacity() > kTlsKeep)
        std::string().swap(s->in);
    }
    return static_cast<int>(n);
  }
  static long bio_ctrl(BIO *b, int cmd, long, void *) {
    switch (cmd) {
    case BIO_CTRL_FLUSH:
      return 1;
    case BIO_CTRL_PENDING: {
      auto *s = static_cast<tls_session *>(BIO_get_data(b));
      return static_cast<long>(s->in.size() - s->in_off);
    }
    default:
      return 0;
    }
  }
};

BIO_METHOD *new_bio_method() {
  BIO_METHOD *bm = BIO_meth_new(BIO_TYPE_SOURCE_SINK, "cvc.net");
  if (bm) {
    BIO_meth_set_write(bm, &tls_session::bio_write);
    BIO_meth_set_read(bm, &tls_session::bio_read);
    BIO_meth_set_ctrl(bm, &tls_session::bio_ctrl);
  }
  return bm;
}

#else // !CVC_NET_HAVE_TLS: no listener has TLS, so no connection makes a session.

struct tls_context {
  tls_options options;
  std::string keyprint;
};
std::shared_ptr<tls_context> load_tls(const tls_options &, std::string &why) {
  why = "this build of libcvc has no TLS (OpenSSL's libssl)";
  return nullptr;
}
struct tls_session {
  std::string out;
  std::size_t out_off = 0;
#if CVC_NET_ASYNC_WRITES
  std::string flight;
#endif
  bool shut = false;
  static std::unique_ptr<tls_session> server(tls_context &, std::string &why) {
    why = "no TLS";
    return nullptr;
  }
  bool has_out() const { return false; }
  void sent(std::size_t) {}
  int handshake(std::string &) { return -1; }
  long read(char *, std::size_t, std::string &) { return -2; }
  bool write(const char *, std::size_t, std::string &) { return false; }
  void shutdown() { shut = true; }
  void describe(conn_info &) const {}
  std::string in;
};

#endif // CVC_NET_HAVE_TLS

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

  // TLS: what new connections start from, replaced by a reload.
  mutable std::mutex tls_mu;
  std::shared_ptr<tls_context> tls; // under tls_mu
  asio::steady_timer watch;         // the files' watch, on the acceptor's thread
#if CVC_NET_HAVE_TLS
  file_times watch_seen{};  // the times the watch's last tick saw
  file_times watch_tried{}; // the times a watched reload last tried (and failed) to load
#endif

  listener_state(asio::io_context &ctx, unsigned workers)
      : acceptor(ctx), retry(ctx), blocks(new counter_block[workers]), watch(ctx) {}

  std::shared_ptr<tls_context> tls_now() const {
    std::lock_guard<std::mutex> lock(tls_mu);
    return tls;
  }
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
  // A listener's event from any thread (a TLS reload's).
  void post_listener_event(net_event::type kind, listener_id listener, std::string message);
  bool reload(listener_id id, const tls_options *tls, std::string *error);
  void arm_watch(const std::shared_ptr<listener_state> &l); // on worker 0
  void watch_tick(const std::shared_ptr<listener_state> &l);
  std::shared_ptr<listener_state> find_listener(listener_id id) const;
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
  std::vector<char> tls_pack; // small sends packed into one record
  std::vector<asio::const_buffer> gather;
  std::vector<command> running; // the commands being applied
  counter_block counters;

  worker(net_engine::impl &e, unsigned i, std::size_t read_buffer)
      : engine(e), index(i), scratch(read_buffer), tls_pack(kTlsRecord) {
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
  // What it speaks. `detecting` waits for the first byte (listener_options::detect_tls);
  // `handshaking` and `tls` have a session.
  enum class layer : std::uint8_t { plain, detecting, handshaking, tls };
  enum class timer_for : std::uint8_t { first_frame, flush, detect, handshake };

  worker &w;
  conn_id id;
  std::shared_ptr<listener_state> listener;
  counter_block &lst; // the listener's block on this worker
  tcp::socket sock;
  framing frames;
  std::size_t max_frame, max_queued;
  inbound_limits limits;
  conn_info info; // posted with `accepted`

  layer speaks = layer::plain;
  std::shared_ptr<tls_context> tls_ctx; // the listener's TLS when this was accepted
  std::unique_ptr<tls_session> tls;

  phase state = phase::open;
  bool reading = false, writing = false, paused = false, peer_eof = false;
  bool got_frame = false, is_dirty = false, want_w = false;
  bool announced = false; // `accepted` posted: the program knows it, and hears of its close
  std::size_t low_mark = 0;
  std::uint64_t raw_left = 0;

  // Input not yet split (a frame's start, or what a pause held back).
  std::string pending;
  // The window the inbound limits count in.
  steady::time_point window_start;
  std::uint64_t window_bytes = 0, window_frames = 0;

  std::deque<net_engine::payload> out;
  std::size_t front_off = 0; // bytes of out.front() already written (or encrypted)
  std::size_t queued = 0;
#if CVC_NET_ASYNC_WRITES
  // The overlapped write's payloads, kept until its handler runs (a close clears `out` first).
  std::vector<net_engine::payload> in_flight;
#endif

  // One deadline at a time: the detection's, then the handshake's, then the first frame's, or a
  // flushing close's.
  asio::steady_timer timer;
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

  // ── the start: detection, the handshake, `accepted` ──────────────────────────────────────────
  // Tell the program (its first sends then go out on the layer decided).
  void announce() {
    announced = true;
    w.out_events.emplace_back();
    net_event &ev = w.out_events.back();
    ev.kind = net_event::type::accepted;
    ev.conn = id;
    ev.listener = listener->id;
    ev.info = info;
    if (limits.first_frame_timeout.count() > 0 && !got_frame)
      arm_timer(limits.first_frame_timeout, timer_for::first_frame);
    if (!out.empty())
      mark_dirty();
  }

  bool begin_tls() {
    std::string why;
    tls = tls_session::server(*tls_ctx, why);
    if (!tls) {
      fail(close_reason::tls_handshake, why);
      return false;
    }
    speaks = layer::handshaking;
    if (tls_ctx->options.handshake_timeout.count() > 0)
      arm_timer(tls_ctx->options.handshake_timeout, timer_for::handshake);
    else
      cancel_timer();
    return true;
  }

  void established() {
    speaks = layer::tls;
    cancel_timer();
    tls->describe(info);
    count(&counter_block::tls_accepted);
    announce();
  }

  // ── timers ────────────────────────────────────────────────────────────────────────────────────
  void arm_timer(std::chrono::milliseconds after, timer_for why) {
    const unsigned gen = ++timer_gen;
    timer.expires_after(after);
    timer.async_wait([self = shared_from_this(), gen, why](const boost::system::error_code &ec) {
      if (ec || gen != self->timer_gen)
        return;
      turn t(self->w);
      self->on_timer(why);
    });
  }
  void cancel_timer() {
    ++timer_gen;
    timer.cancel();
  }
  void on_timer(timer_for why) {
    switch (why) {
    case timer_for::first_frame:
      if (state == phase::open && !got_frame)
        fail(close_reason::first_frame_timeout);
      break;
    case timer_for::flush:
      if (state == phase::closing)
        fail(close_reason::program, "the flush timed out");
      break;
    case timer_for::detect: // nothing came: a plain client waiting for the server
      if (state == phase::open && speaks == layer::detecting) {
        speaks = layer::plain;
        announce();
      }
      break;
    case timer_for::handshake:
      if (state == phase::open && speaks == layer::handshaking)
        fail(close_reason::tls_handshake, "the handshake timed out");
      break;
    }
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
      if (!input(w.scratch.data(), n, now))
        return;
      if (n < w.scratch.size())
        break; // most likely all there was
    }
    start_read();
  }

  // Bytes read from the socket; false once the connection stops reading (closed or paused).
  bool input(const char *data, std::size_t n, steady::time_point now) {
    if (speaks == layer::detecting) {
      cancel_timer();
      if (static_cast<unsigned char>(data[0]) == 0x16) { // a TLS handshake record
        if (!begin_tls())
          return false;
      } else {
        speaks = layer::plain;
        announce();
      }
    }
    if (tls) {
      tls->in.append(data, n);
      return tls_pump(now);
    }
    return plain_input(data, n, now);
  }

  // The program's bytes: counted, limited, split.
  bool plain_input(const char *data, std::size_t n, steady::time_point now) {
    count(&counter_block::bytes_in, n);
    if (limits.bytes_per_window || limits.frames_per_window) {
      if (now - window_start >= limits.window) {
        window_start = now;
        window_bytes = window_frames = 0;
      }
      window_bytes += n;
      if (limits.bytes_per_window && window_bytes > limits.bytes_per_window) {
        fail(close_reason::bytes_limit);
        return false;
      }
    }
    consume(data, n);
    return state == phase::open && !paused;
  }

  // Run the handshake on what has come, then decrypt it into the program's bytes (into the
  // worker's read buffer, free again once its records are in `tls->in`).
  bool tls_pump(steady::time_point now) {
    std::string why;
    if (speaks == layer::handshaking) {
      const int r = tls->handshake(why);
      if (tls->has_out())
        mark_dirty(); // its records
      if (r < 0) {
        fail(close_reason::tls_handshake, why);
        return false;
      }
      if (r == 0)
        return true;
      established();
    }
    while (state == phase::open && !paused) {
      const long n = tls->read(w.scratch.data(), w.scratch.size(), why);
      if (n > 0) {
        plain_input(w.scratch.data(), static_cast<std::size_t>(n), now);
        continue;
      }
      if (n == -1)
        fail(close_reason::peer_closed);
      else if (n < -1)
        fail(close_reason::error, why);
      break;
    }
    if (state != phase::closed && tls->has_out())
      mark_dirty(); // a reply OpenSSL made while reading (a key update's)
    return state == phase::open && !paused;
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
    // TLS: records the pause left in the session (read, not yet decrypted) come next.
    if (state == phase::open && !paused && speaks == layer::tls)
      tls_pump(steady::now());
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
    if (tls) {
      write_tls();
      return;
    }
    if (speaks == layer::detecting)
      return; // nothing goes out before the layer is decided
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

  // TLS: the session's records first; then, once the handshake is done, the queue encrypted a
  // pass at a time; on a flushing close, close_notify last.
  void write_tls() {
    while (state != phase::closed && !writing) {
      if (tls->has_out()) {
        const std::size_t left = tls->out.size() - tls->out_off;
        boost::system::error_code ec;
        const std::size_t n =
            sock.write_some(asio::buffer(tls->out.data() + tls->out_off, left), ec);
        if (ec == asio::error::would_block || ec == asio::error::try_again) {
          wait_writable();
          return;
        }
        if (ec) {
          fail(close_reason::error, ec.message());
          return;
        }
        count(&counter_block::bytes_out, n);
        tls->sent(n);
        if (n < left) {
          wait_writable();
          return;
        }
        continue;
      }
      if (speaks != layer::tls)
        break; // the handshake's records only, until it is done
      if (!out.empty()) {
        if (!encrypt())
          return;
        continue;
      }
      if (state == phase::closing && !tls->shut) {
        tls->shutdown();
        continue;
      }
      break;
    }
    if (state == phase::closing && !writing && out.empty() && !tls->has_out() &&
        (tls->shut || speaks != layer::tls))
      linger();
  }

  // Encrypt up to kTlsPass bytes of the queue into records: small sends packed into one record,
  // a large one straight from its buffer.
  bool encrypt() {
    std::size_t budget = kTlsPass;
    std::string why;
    while (!out.empty() && budget) {
      const net_engine::payload &front = out.front();
      const std::size_t left = front->size() - front_off;
      const char *from = nullptr;
      std::size_t n = 0;
      if (left >= w.tls_pack.size()) {
        from = front->data() + front_off;
        n = std::min(left, budget);
      } else {
        const std::size_t cap = std::min(w.tls_pack.size(), budget);
        std::size_t skip = front_off;
        for (const auto &p : out) {
          const std::size_t take = std::min(cap - n, p->size() - skip);
          std::memcpy(w.tls_pack.data() + n, p->data() + skip, take);
          n += take;
          skip = 0;
          if (n == cap)
            break;
        }
        from = w.tls_pack.data();
      }
      if (!tls->write(from, n, why)) {
        fail(close_reason::error, why);
        return false;
      }
      taken(n);
      budget -= n;
    }
    return true;
  }

  void wrote(std::size_t n) {
    count(&counter_block::bytes_out, n);
    taken(n);
  }

  // `n` bytes of the queue are out of it (written, or encrypted into records).
  void taken(std::size_t n) {
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
  // The rest as an overlapped write: Asio copies the buffer list, and `in_flight` keeps the bytes
  // (on TLS, `tls->flight` the records, while OpenSSL may add more to `tls->out`).
  void wait_writable() {
    writing = true;
    if (tls) {
      tls->flight.assign(tls->out, tls->out_off, std::string::npos);
      tls->out.clear();
      tls->out_off = 0;
      sock.async_write_some(
          asio::buffer(tls->flight),
          [self = shared_from_this()](const boost::system::error_code &ec, std::size_t n) {
            turn t(self->w);
            self->writing = false;
            if (self->state == phase::closed)
              return;
            if (ec) {
              self->fail(close_reason::error, ec.message());
              return;
            }
            self->count(&counter_block::bytes_out, n);
            self->tls->out.insert(0, self->tls->flight, n, std::string::npos);
            self->tls->flight.clear();
            self->write_now();
          });
      return;
    }
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
    arm_timer(flush_timeout, timer_for::flush);
    if (tls) {
      mark_dirty(); // the queue, then close_notify, then the lingerer
      start_read();
    } else if (out.empty() && !writing) {
      linger();
    } else {
      start_read(); // drop what comes while the output drains
    }
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
    if (!conn->announced)
      continue; // the program never heard of it (a failed handshake, a client gone meanwhile)
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

  conn_info &info = conn->info;
  info.id = id;
  info.listener = l->id;
  info.peer_address = peer.address().to_string();
  info.peer_port = peer.port();
  info.local_address = local.address().to_string();
  info.local_port = local.port();

  // `accepted` waits for TLS's handshake, or for the detection's first byte.
  conn->tls_ctx = l->tls_now();
  if (!conn->tls_ctx) {
    conn->announce();
  } else if (l->options.detect_tls.count() > 0) {
    conn->speaks = connection::layer::detecting;
    conn->arm_timer(l->options.detect_tls, connection::timer_for::detect);
  } else if (!conn->begin_tls()) {
    return;
  }
  conn->start_read();
}

void net_engine::impl::post_listener_event(net_event::type kind, listener_id listener,
                                           std::string message) {
  std::vector<net_event> evs(1);
  evs[0].kind = kind;
  evs[0].listener = listener;
  evs[0].message = std::move(message);
  std::vector<std::shared_ptr<const void>> k;
  post_events(evs, k);
}

std::shared_ptr<listener_state> net_engine::impl::find_listener(listener_id id) const {
  std::lock_guard<std::mutex> lock(lst_mu);
  auto it = listeners.find(id);
  return it == listeners.end() ? nullptr : it->second;
}

bool net_engine::impl::reload(listener_id id, const tls_options *tls, std::string *error) {
  auto fail = [&](const std::string &why) {
    if (error)
      *error = why;
    return false;
  };
  const std::shared_ptr<listener_state> l = find_listener(id);
  if (!l)
    return fail("no listener " + std::to_string(id));
  const std::shared_ptr<tls_context> now = l->tls_now();
  if (!now)
    return fail("listener " + std::to_string(id) + " has no TLS");
  std::string why;
  std::shared_ptr<tls_context> next = load_tls(tls ? *tls : now->options, why);
  if (!next)
    return fail(why);
  const std::string keyprint = next->keyprint;
  {
    std::lock_guard<std::mutex> lock(l->tls_mu);
    l->tls = std::move(next);
  }
  post_listener_event(net_event::type::tls_reloaded, id, keyprint);
  if (tls) // the watch's interval may have changed
    asio::post(workers[0]->ctx, [this, l] { arm_watch(l); });
  return true;
}

void net_engine::impl::arm_watch(const std::shared_ptr<listener_state> &l) {
  l->watch.cancel();
  const std::shared_ptr<tls_context> now = l->tls_now();
  if (l->stopped || stopping.load() || !now || now->options.watch.count() <= 0)
    return;
  l->watch.expires_after(now->options.watch);
  l->watch.async_wait([this, l](const boost::system::error_code &ec) {
    if (!ec)
      watch_tick(l);
  });
}

// A file changed since its context was loaded (and since the last try that failed), and both have
// kept their times for a tick: load them. The wait lets a certificate and its key be replaced one
// after the other, and a write end, before they are read.
void net_engine::impl::watch_tick(const std::shared_ptr<listener_state> &l) {
#if CVC_NET_HAVE_TLS
  const std::shared_ptr<tls_context> now = l->tls_now();
  if (!now || l->stopped)
    return;
  const file_times times = times_of(now->options);
  const bool settled = times == l->watch_seen;
  l->watch_seen = times;
  if (settled && times != now->times && times != l->watch_tried) {
    std::string why;
    std::shared_ptr<tls_context> next = load_tls(now->options, why);
    if (next) {
      l->watch_tried = {};
      const std::string keyprint = next->keyprint;
      {
        std::lock_guard<std::mutex> lock(l->tls_mu);
        // A reload_tls() meanwhile wins: this one read the files it replaced.
        if (l->tls == now)
          l->tls = std::move(next);
      }
      post_listener_event(net_event::type::tls_reloaded, l->id, keyprint);
    } else {
      l->watch_tried = times;
      post_listener_event(net_event::type::error, l->id, "TLS reload: " + why);
    }
  }
#endif
  arm_watch(l);
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
  if (options.tls.enabled()) {
    std::string why;
    l->tls = load_tls(options.tls, why);
    if (!l->tls)
      return fail(why);
  } else if (options.detect_tls.count() > 0) {
    return fail("detect_tls needs a certificate (tls.certificate_file)");
  }
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
  asio::post(m_->workers[0]->ctx, [this, l] {
    m_->start_accept(l);
    m_->arm_watch(l);
  });
  return l->id;
}

#if CVC_NET_HAVE_TLS
bool net_engine::tls_available() { return true; }
#else
bool net_engine::tls_available() { return false; }
#endif

bool net_engine::reload_tls(listener_id listener, std::string *error) {
  return m_->reload(listener, nullptr, error);
}

bool net_engine::reload_tls(listener_id listener, const tls_options &tls, std::string *error) {
  return m_->reload(listener, &tls, error);
}

std::string net_engine::tls_keyprint(listener_id listener) const {
  const std::shared_ptr<listener_state> l = m_->find_listener(listener);
  const std::shared_ptr<tls_context> t = l ? l->tls_now() : nullptr;
  return t ? t->keyprint : std::string();
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
    l->watch.cancel();
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
      l->watch.cancel();
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

#endif // !__EMSCRIPTEN__ && !__wasi__

} // namespace net
} // namespace cvc

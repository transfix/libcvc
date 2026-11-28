#ifndef CVC_NET_NET_ENGINE_H
#define CVC_NET_NET_ENGINE_H

// cvc::net — the native connection engine (docs/roadmap/ARIADNE_NET_ENGINE.md, parts N1 and N3).
//
// A caller-owned engine that holds raw TCP and TLS connections on I/O threads of its own: it
// accepts them, splits their input into frames, queues their output with a bound, and sends one
// buffer to a group of connections without a copy per receiver. It knows no protocol.
//
//   - Threads. `io_threads` Asio io_contexts, each run by one thread. A connection lives on one of
//     them for its whole life (its id says which), so nothing inside a connection takes a lock.
//   - Input. The I/O threads post events (accepted, frames, closed, writable, error) to one queue.
//     The program takes everything queued as a batch (wait() or poll()); a batch's frames point
//     into buffers the batch keeps alive until it is cleared or destroyed. Events of one
//     connection keep their order.
//   - Output. send(), send_many() and broadcast() copy the bytes once into a refcounted buffer,
//     and each receiver's queue holds a reference. Commands apply in the order they were issued:
//     for one connection, bytes go out in the order of the sends, broadcasts and closes that
//     reached it, and a join() issued before a broadcast() includes the connection. Every call
//     only queues a command and returns.
//   - Backpressure. A connection whose queued output passes `max_queued` closes (slow_reader);
//     nothing is dropped from the middle of a stream. want_writable() asks for one `writable`
//     event once the queue drains to a low mark.
//   - TLS (N3). A listener with a certificate and key speaks TLS (OpenSSL, 1.2 and 1.3), or, with
//     `detect_tls`, both TLS and plain on one port. The handshake (and the detection) happens
//     before `accepted` is posted, so the program's first send goes out on the right layer. Output
//     stays one refcounted buffer a message; each TLS connection encrypts its own copy as it
//     writes. reload_tls() takes new files for new connections; open ones keep theirs.
//
// The pycvc binding is N2 (bindings/pycvc/pycvc_net.i). Client connections, with client TLS, and
// UDP are N4. Not available in a browser build: listen() fails there.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace net {

// A connection's id: never 0 and never reused within an engine's life.
using conn_id = std::uint64_t;
using listener_id = std::uint64_t;

// How a connection's input is split into frames (§2.2).
struct framing {
  enum class mode : std::uint8_t {
    delimiter, // the bytes up to and including `delimiter`
    length,    // a `prefix`-byte (1, 2 or 4) big-endian length, then that many bytes; the frame
               // is those bytes, without the prefix
    raw,       // whatever one read returned (at most max_frame bytes)
  };
  mode kind = mode::raw;
  std::uint8_t delimiter = '\n';
  std::uint8_t prefix = 4;
  // raw only: deliver this many bytes (0: no end), then pause as pause_after does.
  std::uint64_t raw_bytes = 0;
  // After a frame that starts with one of these, the engine splits no further input of the
  // connection, and stops reading it, until the program's next set_framing() (or resume()). A
  // protocol whose input changes framing after a known frame (NMDC's $ADCSND before a file's
  // bytes) sets the new framing there with no race against bytes read ahead.
  std::vector<std::string> pause_after;

  static framing delimited(std::uint8_t byte) {
    framing f;
    f.kind = mode::delimiter;
    f.delimiter = byte;
    return f;
  }
  static framing length_prefixed(std::uint8_t prefix_bytes) {
    framing f;
    f.kind = mode::length;
    f.prefix = prefix_bytes;
    return f;
  }
  static framing raw(std::uint64_t bytes = 0) {
    framing f;
    f.kind = mode::raw;
    f.raw_bytes = bytes;
    return f;
  }
};

// Limits the engine enforces before the program sees a connection's input (§2.3). 0: none.
struct inbound_limits {
  std::uint64_t bytes_per_window = 0;
  std::uint64_t frames_per_window = 0;
  std::chrono::milliseconds window{1000};
  // From the accept to the first complete frame.
  std::chrono::milliseconds first_frame_timeout{0};
};

// TLS versions, as a minimum (§2.5). The maximum is the highest the OpenSSL in use has.
enum class tls_protocol : std::uint8_t { tls1_2, tls1_3 };

// A listener's TLS (§2.5, N3). A certificate file makes the listener speak TLS.
struct tls_options {
  std::string certificate_file; // PEM: the certificate, then any intermediates. Empty: no TLS.
  std::string key_file;         // PEM: its private key (unencrypted)
  tls_protocol min_version = tls_protocol::tls1_2;
  std::string ciphers;      // TLS 1.2's cipher list, in OpenSSL's syntax; empty: OpenSSL's default
  std::string ciphersuites; // TLS 1.3's; empty: OpenSSL's default
  // Ask each client for a certificate. Any certificate is accepted, and so is none: its SHA-256
  // goes in conn_info for the program to judge (DC clients' certificates are self-signed).
  bool request_peer_certificate = true;
  // A handshake not done by then closes the connection (close_reason::tls_handshake).
  std::chrono::milliseconds handshake_timeout{10000};
  // >0: check the two files' modification times this often, and reload (reload_tls) when either
  // has changed and both have then kept their times for one more check (so a certificate and its
  // key can be replaced one after the other). A failed reload posts the listener's `error` once
  // and keeps the certificate in use; the next change tries again.
  std::chrono::milliseconds watch{0};

  bool enabled() const { return !certificate_file.empty(); }
};

struct listener_options {
  std::string address = "0.0.0.0"; // an IPv4 or IPv6 literal ("::" listens on both where the
                                   // system allows it)
  std::uint16_t port = 0;          // 0: one the system picks (port() says which)
  std::string name;                // for stats and events
  framing frames;                  // each accepted connection's first framing
  int backlog = 1024;
  std::size_t max_frame = 65536;            // a longer frame closes with frame_too_long
  std::size_t max_queued = 4 * 1024 * 1024; // queued output past this closes with slow_reader
  inbound_limits limits;
  bool no_delay = true; // TCP_NODELAY on accepted connections
  // How long close(conn, flush) waits for the queue to drain before it closes anyway.
  std::chrono::milliseconds flush_timeout{10000};
  tls_options tls; // a certificate file: TLS on this listener
  // With `tls`: >0 makes this a plain port that also speaks TLS. The engine waits this long for
  // the client's first byte: a TLS record (0x16) starts TLS, anything else, or nothing in time,
  // stays plain. Either way `accepted` comes after the decision, so a protocol whose server speaks
  // first (NMDC's $Lock) greets a waiting plain client once the wait is over.
  std::chrono::milliseconds detect_tls{0};
  // The registry tie-in (§2.6) adds register_clients.
};

enum class close_reason : std::uint8_t {
  program,             // close() or stop_listening's owner closed it
  peer_closed,         // the peer ended its side
  error,               // a read or write failed (the event's `message` says how)
  frame_too_long,      // a frame past max_frame
  slow_reader,         // queued output past max_queued
  bytes_limit,         // inbound_limits::bytes_per_window
  frames_limit,        // inbound_limits::frames_per_window
  first_frame_timeout, // inbound_limits::first_frame_timeout
  engine_stopped,      // stop() or the engine's destruction
  // The TLS handshake failed (the event's `message` says how) or timed out. Only counted: the
  // program never heard of the connection, so no `closed` event is posted for it.
  tls_handshake,
};
constexpr std::size_t kCloseReasons = 10;
const char *to_string(close_reason reason);

struct conn_info {
  conn_id id = 0;
  listener_id listener = 0;
  std::string peer_address;
  std::uint16_t peer_port = 0;
  std::string local_address;
  std::uint16_t local_port = 0;
  // TLS, as the handshake left it; all empty on a plain connection.
  std::string tls_version; // "TLSv1.2", "TLSv1.3"
  std::string tls_cipher; // OpenSSL's name: "TLS_AES_256_GCM_SHA384", "ECDHE-RSA-AES128-GCM-SHA256"
  std::string tls_sni;    // the server name the client asked for, if it did
  // The client's certificate, if it sent one: the SHA-256 of its DER, as 64 lowercase hex digits,
  // and the same as a keyprint ("SHA256/" and 52 base32 digits; tls_keyprint below).
  std::string tls_peer_sha256;
  std::string tls_peer_keyprint;

  bool tls() const { return !tls_version.empty(); }
};

struct net_event {
  enum class type : std::uint8_t {
    accepted,     // `info`
    frames,       // `frames`, in arrival order; one event may hold several reads' frames
    closed,       // `reason`, and `message` for close_reason::error
    writable,     // the queue drained to the mark want_writable() asked for
    error,        // a listener's: `message` (an accept failed, and the listener keeps trying; or a
                  // watched TLS reload failed, and the certificate in use stays)
    tls_reloaded, // a listener's TLS was reloaded: `message` is its new keyprint
  };
  type kind = type::frames;
  conn_id conn = 0;
  listener_id listener = 0;
  conn_info info;
  std::vector<std::string_view> frames; // valid while the batch holding the event is
  close_reason reason = close_reason::program;
  std::string message;
};

// What wait() and poll() hand over: every event queued since the last batch. The frames' bytes
// live as long as the batch (or until clear()).
struct net_batch {
  std::vector<net_event> events;
  std::vector<std::shared_ptr<const void>> keep; // the buffers the frames point into
  void clear() {
    events.clear();
    keep.clear();
  }
  bool empty() const { return events.empty(); }
};

// One block of counters (§2.10): the engine's, or one listener's.
struct net_counters {
  std::uint64_t open = 0;
  std::uint64_t accepted = 0; // TCP accepts, before any TLS handshake or detection
  std::array<std::uint64_t, kCloseReasons> closed{}; // by close_reason
  std::uint64_t bytes_in = 0; // the program's bytes (on TLS, what it decrypted to)
  std::uint64_t frames_in = 0;
  std::uint64_t bytes_out = 0;    // written to sockets (on TLS, the records)
  std::uint64_t messages_out = 0; // buffers queued to connections (one a receiver)
  // broadcast() calls: the engine's only (a listener's block reads 0), since one broadcast's
  // receivers can come from several listeners.
  std::uint64_t broadcasts = 0;
  std::uint64_t broadcast_receivers = 0; // the receivers broadcasts reached (a listener's: its own)
  std::uint64_t queued_bytes = 0;        // output queued now (before TLS: the program's bytes)
  std::uint64_t max_queued_bytes = 0;    // the most one connection has had queued
  std::uint64_t tls_accepted = 0;        // connections whose TLS handshake completed
  std::uint64_t slow_reader() const {
    return closed[static_cast<std::size_t>(close_reason::slow_reader)];
  }
};

struct net_stats {
  net_counters engine;
  struct listener_stats {
    listener_id id = 0;
    std::string name;
    std::uint16_t port = 0;
    net_counters counters;
  };
  std::vector<listener_stats> listeners;
  struct thread_stats {
    std::uint64_t busy_ns = 0; // time in the engine's handlers
    std::uint64_t handlers = 0;
    std::uint64_t connections = 0; // open on this thread
  };
  std::vector<thread_stats> threads;
};

struct engine_options {
  unsigned io_threads = 2; // at least 1, at most the core count
  // Each I/O thread's read buffer; one read takes at most this much.
  std::size_t read_buffer = 64 * 1024;
};

class net_engine {
public:
  explicit net_engine(engine_options options = {});
  ~net_engine(); // stop()
  net_engine(const net_engine &) = delete;
  net_engine &operator=(const net_engine &) = delete;

  // Whether this build can listen at all (false in a browser).
  static bool available();
  unsigned io_threads() const;

  // Bind and listen; 0, with `error` saying why, when it cannot. Accepting starts at once.
  listener_id listen(const listener_options &options, std::string *error = nullptr);
  std::uint16_t port(listener_id listener) const; // the bound port (0: no such listener)
  // Stop accepting; its open connections stay.
  void stop_listening(listener_id listener);

  // ── TLS (§2.5, N3) ─────────────────────────────────────────────────────────────────────────────
  // Whether this build has TLS (OpenSSL's libssl); without it a listener with TLS fails to listen.
  static bool tls_available();
  // Load the listener's certificate and key files again, or `tls`'s (which replace its TLS
  // options), for the connections it accepts from now on; open connections keep what they had.
  // On success a `tls_reloaded` event follows. False, with `error` saying why, when the listener
  // has no TLS or the files do not load; the certificate in use then stays.
  bool reload_tls(listener_id listener, std::string *error = nullptr);
  bool reload_tls(listener_id listener, const tls_options &tls, std::string *error = nullptr);
  // The listener's certificate as a keyprint, [ADC-EXT] KEYP's form: "SHA256/" and the unpadded
  // base32 of the SHA-256 of its DER. Empty without TLS.
  std::string tls_keyprint(listener_id listener) const;

  // ── commands (any thread; each queues and returns) ──────────────────────────────────────────
  using payload = std::shared_ptr<const std::string>;
  void send(conn_id conn, std::string_view bytes);
  void send(conn_id conn, payload bytes);
  void send_many(const std::vector<conn_id> &conns, std::string_view bytes);
  void broadcast(const std::string &group, std::string_view bytes, conn_id exclude = 0);
  void broadcast(const std::string &group, payload bytes, conn_id exclude = 0);
  // flush: send what is queued first (at most flush_timeout), then close.
  void close(conn_id conn, bool flush = true);
  void join(conn_id conn, const std::string &group);
  void leave(conn_id conn, const std::string &group);
  // The connection's framing from the next byte not yet split; also ends a pause.
  void set_framing(conn_id conn, framing frames);
  // End a pause (framing::pause_after, framing::raw(n)) keeping the framing.
  void resume(conn_id conn);
  // One `writable` event once the connection's queued output is at most `low_mark` bytes (at
  // once if it is already).
  void want_writable(conn_id conn, std::size_t low_mark);

  // ── input ────────────────────────────────────────────────────────────────────────────────────
  // Replace `out` with every event queued, waiting at most `timeout` for one. False when nothing
  // came: the timeout passed, or the engine has stopped and its queue is empty.
  bool wait(net_batch &out, std::chrono::milliseconds timeout);
  // The same without waiting.
  bool poll(net_batch &out);
  // Called on an I/O thread whenever the queue goes from empty to not, for a program whose loop
  // wakes on its own signal (N2's waker). Set it before listening. It must not call the engine.
  void set_notify(std::function<void()> notify);

  net_stats stats() const;

  // Close every connection (closed: engine_stopped) and listener, and join the I/O threads. The
  // queue keeps what was posted for the program to take; later commands do nothing.
  void stop();

  struct impl;

private:
  std::unique_ptr<impl> m_;
};

} // namespace net
} // namespace cvc

#endif // CVC_NET_NET_ENGINE_H

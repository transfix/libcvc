/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// WebSocket backend of cvc::state_transport. See inc/cvc/state/state_transport_ws.h for the design
// and docs/roadmap/WS_STATE_TRANSPORT.md for the plan. This transport carries the SAME protobuf
// envelope as state_transport_grpc (cvc.transport.v1.Frame), here as WebSocket BINARY frames, so a
// relay running both a gRPC and a WS acceptor can bridge wasm + native clients into one room.
//
// Needs the generated Frame MESSAGE (protobuf), not the gRPC service, so this TU does NOT include
// grpcpp and compiles with CVC_ENABLE_GRPC off (the wasm conference builds grpc-off). The native WS
// is Boost.Beast (header-only); under __EMSCRIPTEN__ the client would use the browser WebSocket
// (emscripten/websocket.h) — scaffolded here, implemented with the wasm build (Phase 2).

#include "proto/state_transport.pb.h"

#include <algorithm>
#include <cstdio>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_ws.h>
#include <stdexcept>
#include <thread>
#include <utility>

#ifndef __EMSCRIPTEN__
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#ifdef __linux__
#include <linux/sockios.h> // SIOCOUTQ — unsent bytes in a socket's send buffer (send_backlog)
#include <sys/ioctl.h>
#endif
#else
#include <cstdint>
#include <cstdlib> // std::getenv — CVC_WS_BROADCAST_DEBUG diagnostic gate
#include <cstring>
#include <emscripten/websocket.h>
#endif

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

// Match the gRPC/IPC frame ceiling so all three wire paths agree (a large set_data payload can be
// tens of MiB; the default websocket read cap is far smaller).
constexpr std::size_t kMaxWsMessageBytes = 64ull * 1024 * 1024;

// ── Frame codec (duplicated from state_transport_grpc.cpp; the plan's step 1 extracts this into a
// shared state_frame_codec TU — kept byte-identical so the two transports' wire bytes match) ──

pb::MutationOp encode_op(state_mutation_op op) {
  switch (op) {
  case state_mutation_op::set_value:
    return pb::MUTATION_OP_SET_VALUE;
  case state_mutation_op::set_data:
    return pb::MUTATION_OP_SET_DATA;
  case state_mutation_op::set_comment:
    return pb::MUTATION_OP_SET_COMMENT;
  case state_mutation_op::set_hidden:
    return pb::MUTATION_OP_SET_HIDDEN;
  case state_mutation_op::set_read_only:
    return pb::MUTATION_OP_SET_READ_ONLY;
  case state_mutation_op::touch:
    return pb::MUTATION_OP_TOUCH;
  case state_mutation_op::reset_node:
    return pb::MUTATION_OP_RESET_NODE;
  case state_mutation_op::delete_subtree:
    return pb::MUTATION_OP_DELETE_SUBTREE;
  case state_mutation_op::delegate_subtree:
    return pb::MUTATION_OP_DELEGATE_SUBTREE;
  case state_mutation_op::revoke_delegation:
    return pb::MUTATION_OP_REVOKE_DELEGATION;
  case state_mutation_op::set_property:
    return pb::MUTATION_OP_SET_PROPERTY;
  }
  return pb::MUTATION_OP_UNSPECIFIED;
}

state_mutation_op decode_op(pb::MutationOp op) {
  switch (op) {
  case pb::MUTATION_OP_SET_VALUE:
    return state_mutation_op::set_value;
  case pb::MUTATION_OP_SET_DATA:
    return state_mutation_op::set_data;
  case pb::MUTATION_OP_SET_COMMENT:
    return state_mutation_op::set_comment;
  case pb::MUTATION_OP_SET_HIDDEN:
    return state_mutation_op::set_hidden;
  case pb::MUTATION_OP_SET_READ_ONLY:
    return state_mutation_op::set_read_only;
  case pb::MUTATION_OP_TOUCH:
    return state_mutation_op::touch;
  case pb::MUTATION_OP_RESET_NODE:
    return state_mutation_op::reset_node;
  case pb::MUTATION_OP_DELETE_SUBTREE:
    return state_mutation_op::delete_subtree;
  case pb::MUTATION_OP_DELEGATE_SUBTREE:
    return state_mutation_op::delegate_subtree;
  case pb::MUTATION_OP_REVOKE_DELEGATION:
    return state_mutation_op::revoke_delegation;
  case pb::MUTATION_OP_SET_PROPERTY:
    return state_mutation_op::set_property;
  default:
    return state_mutation_op::set_value;
  }
}

void set_pb_payload(pb::Payload *p, const state_payload &pl) {
  switch (pl.kind) {
  case state_payload_kind::none:
    p->set_none(true);
    break;
  case state_payload_kind::inline_bytes:
    p->set_inline_bytes(std::string(pl.inline_bytes.begin(), pl.inline_bytes.end()));
    break;
  case state_payload_kind::blob: {
    auto *b = p->mutable_blob();
    b->set_digest(pl.blob.digest);
    b->set_size_bytes(pl.blob.size_bytes);
    b->set_codec(pl.blob.codec);
    break;
  }
  }
}

void read_pb_payload(const pb::Payload &p, state_payload &pl) {
  switch (p.kind_case()) {
  case pb::Payload::kInlineBytes: {
    const std::string &s = p.inline_bytes();
    pl.kind = state_payload_kind::inline_bytes;
    pl.inline_bytes.assign(s.begin(), s.end());
    break;
  }
  case pb::Payload::kBlob:
    pl.kind = state_payload_kind::blob;
    pl.blob.digest = p.blob().digest();
    pl.blob.size_bytes = p.blob().size_bytes();
    pl.blob.codec = p.blob().codec();
    break;
  case pb::Payload::kNone:
  case pb::Payload::KIND_NOT_SET:
    pl.kind = state_payload_kind::none;
    break;
  }
}

void encode_mutation(const state_mutation &m, pb::Mutation *out) {
  out->set_cluster_id(m.cluster_id);
  out->set_tree_id(m.tree_id);
  out->set_origin_node_id(m.origin_node_id);
  out->set_sequence(m.sequence);
  out->set_mutation_id(m.mutation_id);
  out->set_path(m.path);
  out->set_op(encode_op(m.op));
  out->set_type_name(m.type_name);
  out->set_string_value(m.string_value);
  out->set_latest_value_only(m.latest_value_only);
  out->set_hlc_time(m.hlc_time);
  set_pb_payload(out->mutable_payload(), m.payload);
}

state_mutation decode_mutation(const pb::Mutation &in) {
  state_mutation m;
  m.cluster_id = in.cluster_id();
  m.tree_id = in.tree_id();
  m.origin_node_id = in.origin_node_id();
  m.sequence = in.sequence();
  m.mutation_id = in.mutation_id();
  m.path = in.path();
  m.op = decode_op(in.op());
  m.type_name = in.type_name();
  m.string_value = in.string_value();
  m.latest_value_only = in.latest_value_only();
  m.hlc_time = in.hlc_time();
  if (in.has_payload())
    read_pb_payload(in.payload(), m.payload);
  return m;
}

void encode_message(const state_message &m, pb::Message *out) {
  out->set_cluster_id(m.cluster_id);
  out->set_origin_node_id(m.origin_node_id);
  out->set_message_id(m.message_id);
  out->set_path(m.path);
  out->set_ttl_hops(m.ttl_hops);
  out->set_content_type(m.content_type);
  out->set_string_value(m.string_value);
  out->set_bytes_payload(std::string(m.bytes.begin(), m.bytes.end()));
}

state_message decode_message(const pb::Message &in) {
  state_message m;
  m.cluster_id = in.cluster_id();
  m.origin_node_id = in.origin_node_id();
  m.message_id = in.message_id();
  m.path = in.path();
  m.ttl_hops = in.ttl_hops();
  m.content_type = in.content_type();
  m.string_value = in.string_value();
  const std::string &b = in.bytes_payload();
  m.bytes.assign(b.begin(), b.end());
  return m;
}

} // namespace

#ifndef __EMSCRIPTEN__
namespace {
namespace beast = boost::beast;
namespace websocket = boost::beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
} // namespace
#endif

// ── connection: one bidirectional WebSocket stream. One reader thread drains it; writes serialize
// on write_mu so the publish() fan-out and the per-connection Hello/backfill interleave correctly
// (one outstanding read + one outstanding write is the only concurrency Beast's stream permits). ──
class state_transport_ws::connection {
public:
  std::mutex write_mu;
  std::atomic<bool> alive{true};
  std::string remote_node_id;
  std::string remote_cluster_id;
  // Steady-clock ns of the last inbound frame. A live conference client streams media continuously,
  // so a peer that navigated away (its socket not cleanly closed) stops updating this — the relay's
  // reaper uses it to drop the ghost. 0 until the first frame; set to "now" when a connection opens
  // so a brand-new, not-yet-speaking client is not reaped.
  std::atomic<std::int64_t> last_rx_ns{0};

#ifndef __EMSCRIPTEN__
  std::unique_ptr<websocket::stream<tcp::socket>> ws;
  std::thread reader;
  std::atomic<bool> reader_done{false}; // the reader loop has returned: safe to join + erase

  bool can_write() const { return ws != nullptr; }

  // Must hold write_mu.
  bool write_locked(const pb::Frame &f) {
    if (!alive.load() || !ws)
      return false;
    std::string bytes;
    if (!f.SerializeToString(&bytes))
      return false;
    beast::error_code ec;
    ws->binary(true);
    ws->write(net::buffer(bytes), ec);
    if (ec) {
      alive.store(false);
      return false;
    }
    return true;
  }
#else
  // Browser WebSocket: the handle, a back-pointer to the owning transport (for the C callbacks),
  // and an `open` flag — the browser cannot send until onopen has fired. There is no reader thread;
  // the onmessage callback drives deliver_inbound().
  EMSCRIPTEN_WEBSOCKET_T em_socket = -1;
  state_transport_ws *owner = nullptr;
  bool open = false;

  bool can_write() const { return em_socket >= 0 && open; }

  bool write_locked(const pb::Frame &f) {
    if (!alive.load() || em_socket < 0 || !open)
      return false;
    std::string bytes;
    if (!f.SerializeToString(&bytes))
      return false;
    return emscripten_websocket_send_binary(em_socket, const_cast<char *>(bytes.data()),
                                            static_cast<uint32_t>(bytes.size())) >= 0;
  }
#endif

  bool write(const pb::Frame &f) {
    std::lock_guard<std::mutex> lk(write_mu);
    return write_locked(f);
  }

  // Bytes still unsent in this connection's OS send buffer (Linux only; 0 elsewhere). A large value
  // means the peer is not draining fast enough and the next write would BLOCK. The fan paths check
  // this before writing and skip a backed-up connection — dropping a loss-tolerant media frame, or
  // deferring the periodic roster re-broadcast to the next tick — instead of blocking the whole
  // relay (and every other client's roster) on one slow peer. The continuous video flood otherwise
  // fills a slow client's buffer, the synchronous ws->write() blocks, and the broadcast loop stalls
  // for all.
  std::size_t send_backlog() {
#if !defined(__EMSCRIPTEN__) && defined(__linux__)
    if (!ws)
      return 0;
    int unsent = 0;
    const int fd = ws->next_layer().native_handle();
    if (fd >= 0 && ::ioctl(fd, SIOCOUTQ, &unsent) == 0 && unsent > 0)
      return static_cast<std::size_t>(unsent);
#endif
    return 0;
  }
};

#ifndef __EMSCRIPTEN__
struct state_transport_ws::impl {
  net::io_context ioc;
  std::unique_ptr<tcp::acceptor> acceptor;
  std::thread accept_thread;
  std::string listen_addr_resolved;
};
#else
struct state_transport_ws::impl {
  std::string listen_addr_resolved;
};
#endif

state_transport_ws::state_transport_ws() : _impl(new impl()) {}

state_transport_ws::~state_transport_ws() { stop(); }

// ── small url/addr parsing helpers ──
namespace {

// "host:port" -> (host, port); a missing port defaults to `dflt`.
void split_host_port(const std::string &hp, std::string &host, std::string &port,
                     const char *dflt) {
  auto colon = hp.find_last_of(':');
  if (colon == std::string::npos) {
    host = hp;
    port = dflt;
  } else {
    host = hp.substr(0, colon);
    port = hp.substr(colon + 1);
  }
}

// "ws://host:port/path", "host:port/path", or "host:port" -> host, port, target. Returns false for
// wss:// (TLS is Phase 2) or an empty host.
bool parse_ws_url(const std::string &url, std::string &host, std::string &port,
                  std::string &target) {
  std::string s = url;
  if (s.rfind("wss://", 0) == 0)
    return false; // TLS not in the MVP
  if (s.rfind("ws://", 0) == 0)
    s = s.substr(5);
  auto slash = s.find('/');
  std::string hostport = (slash == std::string::npos) ? s : s.substr(0, slash);
  target = (slash == std::string::npos) ? std::string("/") : s.substr(slash);
  split_host_port(hostport, host, port, "80");
  return !host.empty();
}

} // namespace

#ifndef __EMSCRIPTEN__

void state_transport_ws::start(const std::string &listen_addr, const std::string &node_id,
                               const std::string &cluster_id) {
  if (_running.load())
    throw std::runtime_error("state_transport_ws::start: already running");

  _node_id = node_id;
  _cluster_id = cluster_id;

  std::string host, port;
  split_host_port(listen_addr, host, port, "0");

  beast::error_code ec;
  tcp::resolver resolver(_impl->ioc);
  auto results = resolver.resolve(host, port, ec);
  if (ec || results.empty())
    throw std::runtime_error("state_transport_ws::start: resolve failed for '" + listen_addr +
                             "': " + ec.message());
  tcp::endpoint ep = results.begin()->endpoint();

  _impl->acceptor = std::make_unique<tcp::acceptor>(_impl->ioc);
  _impl->acceptor->open(ep.protocol(), ec);
  if (ec)
    throw std::runtime_error("state_transport_ws::start: open failed: " + ec.message());
  _impl->acceptor->set_option(net::socket_base::reuse_address(true), ec);
  _impl->acceptor->bind(ep, ec);
  if (ec)
    throw std::runtime_error("state_transport_ws::start: bind failed for '" + listen_addr +
                             "': " + ec.message());
  _impl->acceptor->listen(net::socket_base::max_listen_connections, ec);
  if (ec)
    throw std::runtime_error("state_transport_ws::start: listen failed: " + ec.message());

  auto local = _impl->acceptor->local_endpoint(ec);
  _impl->listen_addr_resolved = host + ":" + std::to_string(local.port());

  _running.store(true);
  _impl->accept_thread = std::thread([this]() { accept_loop(); });
}

// Server accept loop: synchronous accept → WebSocket handshake → admit + spawn reader. Breaks when
// stop() closes the acceptor (accept() then errors).
void state_transport_ws::accept_loop() {
  for (;;) {
    tcp::socket sock(_impl->ioc);
    beast::error_code ec;
    _impl->acceptor->accept(sock, ec);
    if (ec)
      break; // acceptor closed by stop(), or a fatal accept error
    if (!_running.load())
      break; // woken by stop()'s loopback self-connect

    auto conn = std::make_shared<connection>();
    conn->ws = std::make_unique<websocket::stream<tcp::socket>>(std::move(sock));
    conn->ws->read_message_max(kMaxWsMessageBytes);
    conn->ws->accept(ec); // server-side WebSocket handshake
    if (ec)
      continue;

    // Spawn the reader before admit (admit publishes the connection to stop(), which joins the
    // reader — so the thread must already be assigned), matching state_transport_grpc.
    conn->reader = std::thread([this, conn]() { read_loop(conn); });
    admit_connection(conn);
  }
}

bool state_transport_ws::connect_to_peer(const std::string &url,
                                         std::chrono::milliseconds timeout) {
  // Unlike gRPC (which requires a prior start()), a WS client may dial without a local acceptor —
  // that is the whole point for a browser peer. The io_context is always present in _impl.
  (void)timeout; // the sync connect fails fast on refusal; a deadline timer is Phase 2.

  std::string host, port, target;
  if (!parse_ws_url(url, host, port, target))
    return false;

  auto conn = std::make_shared<connection>();
  beast::error_code ec;
  try {
    tcp::resolver resolver(_impl->ioc);
    auto results = resolver.resolve(host, port, ec);
    if (ec || results.empty())
      return false;
    tcp::socket sock(_impl->ioc);
    net::connect(sock, results, ec);
    if (ec)
      return false;
    conn->ws = std::make_unique<websocket::stream<tcp::socket>>(std::move(sock));
    conn->ws->read_message_max(kMaxWsMessageBytes);
    conn->ws->handshake(host, target.empty() ? std::string("/") : target, ec);
    if (ec)
      return false;
  } catch (const std::exception &) {
    return false;
  }

  conn->reader = std::thread([this, conn]() { read_loop(conn); });
  _running.store(true); // so stop() tears the client connection down
  admit_connection(conn);
  return true;
}

// Per-connection reader: parse each inbound binary frame into a pb::Frame and dispatch. Shared by
// the server-accepted and client-dialed connections.
void state_transport_ws::read_loop(std::shared_ptr<connection> conn) {
  beast::flat_buffer buffer;
  for (;;) {
    if (!conn->alive.load())
      break;
    beast::error_code ec;
    buffer.clear();
    conn->ws->read(buffer, ec);
    if (ec)
      break; // peer closed, or socket closed by stop()
    std::string bytes = beast::buffers_to_string(buffer.data());
    deliver_inbound(conn.get(), bytes.data(), bytes.size());
  }
  conn->alive.store(false);
  // A peer closed this connection (not a relay shutdown): notify the owner so a relay can retract
  // the departed participant from the room. _running is cleared by stop(), so this fires only on a
  // real peer drop, and only when the peer identified itself (sent a Hello).
  if (_running.load() && _on_conn_closed && !conn->remote_node_id.empty())
    _on_conn_closed(conn->remote_node_id);
  conn->reader_done.store(true, std::memory_order_release); // reap_idle may now join + erase this
}

void state_transport_ws::stop() {
  if (!_running.exchange(false))
    return;

  // Wake the accept loop. A synchronous acceptor.accept() already blocked in another thread is NOT
  // interrupted by closing the acceptor, so make a throwaway loopback connection: accept() returns
  // it, the loop sees _running == false and breaks. Then close the acceptor.
  if (_impl->acceptor && _impl->acceptor->is_open()) {
    beast::error_code ec;
    auto ep = _impl->acceptor->local_endpoint(ec);
    if (!ec) {
      try {
        tcp::socket waker(_impl->ioc);
        waker.connect(tcp::endpoint(net::ip::make_address("127.0.0.1"), ep.port()), ec);
        waker.close(ec);
      } catch (const std::exception &) {
      }
    }
    _impl->acceptor->close(ec);
  }

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
    _conns.clear();
  }
  // Interrupt each reader's blocking read(): shutdown() wakes a recv() blocked in another thread
  // (closing the fd alone does not), then close.
  for (auto &c : conns) {
    c->alive.store(false);
    if (c->ws) {
      beast::error_code ec;
      c->ws->next_layer().shutdown(tcp::socket::shutdown_both, ec);
      c->ws->next_layer().close(ec);
    }
  }

  if (_impl->accept_thread.joinable())
    _impl->accept_thread.join();
  for (auto &c : conns)
    if (c->reader.joinable())
      c->reader.join();

  _impl->acceptor.reset();
}

#else // __EMSCRIPTEN__ — the browser WebSocket client.

namespace {
// The browser fires these on the event loop; userData is the connection* (kept alive by _conns),
// which carries a back-pointer to its owning transport. A browser is a client only, so there is no
// accept path and no reader thread — onmessage is the reader.
bool em_ws_on_open(int, const EmscriptenWebSocketOpenEvent *, void *ud) {
  auto *c = static_cast<state_transport_ws::connection *>(ud);
  c->owner->on_connection_open(c);
  return true;
}
bool em_ws_on_message(int, const EmscriptenWebSocketMessageEvent *e, void *ud) {
  auto *c = static_cast<state_transport_ws::connection *>(ud);
  if (!e->isText) // frames are binary protobuf
    c->owner->deliver_inbound(c, e->data, e->numBytes);
  return true;
}
bool em_ws_on_close(int, const EmscriptenWebSocketCloseEvent *, void *ud) {
  auto *c = static_cast<state_transport_ws::connection *>(ud);
  c->open = false;
  c->alive.store(false);
  return true;
}
bool em_ws_on_error(int, const EmscriptenWebSocketErrorEvent *, void *ud) {
  auto *c = static_cast<state_transport_ws::connection *>(ud);
  c->alive.store(false);
  return true;
}
} // namespace

void state_transport_ws::start(const std::string &, const std::string &, const std::string &) {
  throw std::runtime_error("state_transport_ws::start: a server acceptor is not supported under "
                           "wasm (browser = client)");
}

void state_transport_ws::accept_loop() {}
void state_transport_ws::read_loop(std::shared_ptr<connection>) {}

bool state_transport_ws::connect_to_peer(const std::string &url,
                                         std::chrono::milliseconds timeout) {
  (void)timeout; // the browser cannot block for the handshake; onopen drives the rest
  if (!emscripten_websocket_is_supported())
    return false;

  EmscriptenWebSocketCreateAttributes attrs;
  emscripten_websocket_init_create_attributes(&attrs);
  attrs.url = url.c_str();
  attrs.createOnMainThread = true;
  EMSCRIPTEN_WEBSOCKET_T sock = emscripten_websocket_new(&attrs);
  if (sock < 0)
    return false;

  auto conn = std::make_shared<connection>();
  conn->em_socket = sock;
  conn->owner = this;
  // Register now so the connection is visible; Hello + backfill are deferred to on_connection_open
  // (the socket cannot send until it is open). userData is conn.get(), kept alive by _conns.
  register_connection(conn);
  _running.store(true);
  emscripten_websocket_set_onopen_callback(sock, conn.get(), em_ws_on_open);
  emscripten_websocket_set_onmessage_callback(sock, conn.get(), em_ws_on_message);
  emscripten_websocket_set_onclose_callback(sock, conn.get(), em_ws_on_close);
  emscripten_websocket_set_onerror_callback(sock, conn.get(), em_ws_on_error);
  return true; // initiated; "connected" is when onopen fires (connection_count counts open sockets)
}

// Deferred second half of admit_connection for a browser socket: once onopen fires the socket can
// send, so mark it open and push Hello + the backfill under its write lock.
void state_transport_ws::on_connection_open(connection *c) {
  std::vector<state_cluster_shard *> shards;
  if (_backfill_on_connect.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  std::lock_guard<std::mutex> wlk(c->write_mu);
  c->open = true;
  send_hello_locked(*c);
  if (!shards.empty())
    send_backfill_locked(*c, shards);
}

void state_transport_ws::stop() {
  if (!_running.exchange(false))
    return;
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
    _conns.clear();
  }
  for (auto &c : conns) {
    c->alive.store(false);
    c->open = false;
    if (c->em_socket >= 0) {
      emscripten_websocket_close(c->em_socket, 1000, "bye");
      emscripten_websocket_delete(c->em_socket);
      c->em_socket = -1;
    }
  }
}

#endif // __EMSCRIPTEN__

std::string state_transport_ws::listen_address() const {
  return _impl ? _impl->listen_addr_resolved : std::string();
}

// Steady-clock nanoseconds (the reaper's monotonic clock).
static std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Parse one inbound Frame (serialized protobuf bytes) and dispatch it. Shared by the native reader
// loop and the browser onmessage callback. MVP handles Hello / Mutation / Message; chunk / snapshot
// / heartbeat are Phase 2 (the base-class defaults cover their request paths).
void state_transport_ws::deliver_inbound(connection *c, const void *data, std::size_t len) {
  increment_recv_frames();
  c->last_rx_ns.store(steady_ns(), std::memory_order_relaxed); // liveness for the reaper
  pb::Frame in;
  if (!in.ParseFromArray(data, static_cast<int>(len)))
    return;
  if (in.has_hello()) {
    c->remote_node_id = in.hello().node_id();
    c->remote_cluster_id = in.hello().cluster_id();
  } else if (in.has_mutation()) {
    on_inbound_mutation(decode_mutation(in.mutation()), c);
    _recv_mutations.fetch_add(1, std::memory_order_relaxed);
    { std::lock_guard<std::mutex> lk(_recv_mu); }
    _recv_cv.notify_all();
  } else if (in.has_message()) {
    on_inbound_message(decode_message(in.message()), c);
    _recv_messages.fetch_add(1, std::memory_order_relaxed);
  } else if (in.has_snapshot_response()) {
    // A relay pushed its full current state (request_id 0) so this late-joining client sees the
    // peers already in the room. Apply each entry as an initial-sync bootstrap. The conference uses
    // a single shard/cluster, so applying to every registered shard is correct here; the entries do
    // not carry a cluster id.
    const auto &sr = in.snapshot_response();
    std::vector<state_cluster_shard *> shards;
    {
      std::lock_guard<std::mutex> lk(_shards_mu);
      shards = _shards;
    }
    for (const auto &e : sr.entries()) {
      state_transport::snapshot_entry se;
      se.path = e.path();
      se.string_value = e.string_value();
      se.comment = e.comment();
      se.hidden = e.hidden();
      se.read_only = e.read_only();
      for (const auto &p : e.properties())
        se.properties[p.first] = p.second;
      se.type_name = e.type_name();
      se.origin_node_id = e.origin_node_id();
      se.sequence = e.sequence();
      se.data_type_name = e.data_type_name();
      if (e.has_data_payload())
        read_pb_payload(e.data_payload(), se.data_payload);
      for (auto *s : shards) {
        if (s == nullptr)
          continue;
        // One bootstrap entry the local tree rejects must not take down the whole client: apply
        // each defensively so the rest of the roster still loads. A relay snapshot carries its
        // whole shard, which includes per-client read-only UI state the joiner already owns (e.g.
        // the ariadne runtime's ui.docs.* argv) — those are EXPECTED to be rejected, so skip them
        // quietly and only surface genuinely unexpected failures.
        try {
          s->ingest_snapshot_entry(se);
        } catch (const std::exception &ex) {
          if (std::string(ex.what()).find("read-only") == std::string::npos)
            std::fprintf(stderr, "cvc: ws snapshot ingest skipped path '%s': %s\n", se.path.c_str(),
                         ex.what());
        } catch (...) {
          std::fprintf(stderr, "cvc: ws snapshot ingest skipped path '%s' (non-std)\n",
                       se.path.c_str());
        }
      }
    }
  } else if (in.has_media()) {
    // Media frame (camera/mic) on the control channel. NOT shard state and NEVER on the control
    // bridge: decode to the canonical media_frame and hand it to the media sink, tagged with the
    // source connection as the opaque token. On a relay the sink is a media_router (it fans via
    // fan_media, skipping this origin, and across to other protocols); on a leaf (browser) it is
    // the set_on_media subscribe sugar.
    _media_recv.fetch_add(1, std::memory_order_relaxed);
    if (_media_sink) {
      const auto &mf = in.media();
      media_frame out;
      out.origin_node_id = mf.origin_node_id();
      out.stream_id = mf.stream_id();
      out.codec = mf.codec();
      out.width = mf.width();
      out.height = mf.height();
      out.seq = mf.seq();
      out.pts_seconds = mf.pts_seconds();
      out.sample_rate = mf.sample_rate();
      out.channels = mf.channels();
      const std::string &p = mf.payload();
      out.payload.assign(p.begin(), p.end());
      _media_sink(out, /*src_conn=*/c);
    }
  }
}

void state_transport_ws::register_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  if (std::find(_shards.begin(), _shards.end(), shard) == _shards.end())
    _shards.push_back(shard);
  // A CLIENT dials with connect_to_peer() and never start()s, so it has no node id for its Hello —
  // the relay would then see an empty id on close and could not retract the departed participant.
  // Adopt the first registered shard's identity so the Hello carries this participant's node id.
  // start() still wins (it sets _node_id before any shard registers), so this only fills the gap
  // for a pure client; registration is sequenced before connect_to_peer(), so the Hello sees it.
  if (_node_id.empty())
    _node_id = shard->local_node_id();
  if (_cluster_id.empty())
    _cluster_id = shard->cluster_id();
}

void state_transport_ws::unregister_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  _shards.erase(std::remove(_shards.begin(), _shards.end(), shard), _shards.end());
}

state_transport::publish_stats state_transport_ws::publish(const state_mutation &m) {
  publish_stats stats{};

  // Local fan-out to same-cluster peer shards in this process (excluding the originator).
  std::vector<state_cluster_shard *> local_peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    local_peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (!s)
        continue;
      if (s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      local_peers.push_back(s);
    }
  }
  for (auto *peer : local_peers) {
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    auto r = peer->ingest_remote(m);
    if (r.applied) {
      ++stats.delivered;
      _delivered.fetch_add(1, std::memory_order_relaxed);
      _peers.note_mutation_delivered(peer->local_node_id());
    } else if (r.duplicate) {
      ++stats.delivered;
      ++stats.duplicates;
    } else if (r.rejected) {
      ++stats.rejected;
    }
  }

  // Wire fan-out: encode once, write to every connection.
  pb::Frame frame;
  encode_mutation(m, frame.mutable_mutation());

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    // Don't block the caller (the arisrv main loop drains local writes here) on a backed-up peer —
    // it would stall the loop and its periodic broadcast for everyone. The ~1s broadcast
    // re-delivers.
    if (c->send_backlog() > 2 * 1024 * 1024)
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.delivered;
      if (!c->remote_node_id.empty())
        _peers.note_mutation_delivered(c->remote_node_id);
    }
  }

  _published.fetch_add(1, std::memory_order_relaxed);
  return stats;
}

state_transport::publish_message_stats state_transport_ws::publish_message(const state_message &m) {
  publish_message_stats stats{};

  std::vector<state_cluster_shard *> local_peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    local_peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (!s)
        continue;
      if (!m.cluster_id.empty() && s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      local_peers.push_back(s);
    }
  }
  for (auto *peer : local_peers) {
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    if (peer->ingest_remote_message(m)) {
      ++stats.delivered;
      _peers.note_message_delivered(peer->local_node_id());
    } else {
      ++stats.duplicates;
    }
  }

  pb::Frame frame;
  encode_message(m, frame.mutable_message());

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.peers;
      if (!c->remote_node_id.empty())
        _peers.note_message_delivered(c->remote_node_id);
    }
  }
  return stats;
}

std::size_t state_transport_ws::write_media(const media_frame &m, const void *exclude) {
  pb::Frame frame;
  pb::MediaFrame *mf = frame.mutable_media();
  mf->set_origin_node_id(m.origin_node_id);
  mf->set_stream_id(m.stream_id);
  mf->set_codec(m.codec);
  mf->set_width(m.width);
  mf->set_height(m.height);
  mf->set_seq(m.seq);
  mf->set_pts_seconds(m.pts_seconds);
  mf->set_sample_rate(m.sample_rate);
  mf->set_channels(m.channels);
  mf->set_payload(m.payload.data(), static_cast<int>(m.payload.size()));

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  // Media is unreplicated, un-bridged, and not subject to the peer path-filter (it carries no
  // path). `exclude` (a connection*) skips the origin peer when a relay fans; nullptr sends to all.
  std::size_t n = 0;
  for (auto &c : conns) {
    if (!c || c.get() == exclude || !c->alive.load() || !c->can_write())
      continue;
    // Media is loss-tolerant: if this peer's send buffer is backed up (it can't keep pace with the
    // video), DROP the frame rather than let the synchronous write block the relay. A LOW threshold
    // is deliberate — it keeps each peer's buffer small so the small, must-deliver STATE writes
    // (roster join/leave + the periodic re-broadcast below) are never crowded out or deferred.
    // Dropping video to a slow peer is invisible; a stalled roster makes clients unable to SEE each
    // other at all.
    if (c->send_backlog() > 32 * 1024)
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
}

// A browser client has one connection (the relay), so publish_media_frame pushes its camera/mic UP;
// a relay publishing locally (uncommon) would send to all its connections.
void state_transport_ws::publish_media_frame(const ws_media_frame &m) {
  _media_sent.fetch_add(write_media(m, nullptr), std::memory_order_relaxed);
}

// Router egress: fan one peer's frame to every OTHER connection (the origin is `exclude`).
void state_transport_ws::fan_media(const media_frame &f, const void *exclude) {
  _media_fwd.fetch_add(write_media(f, exclude), std::memory_order_relaxed);
}

void state_transport_ws::set_media_sink(media_sink_fn sink) { _media_sink = std::move(sink); }

void state_transport_ws::set_on_media(media_frame_fn cb) {
  if (!cb) {
    _media_sink = {};
    return;
  }
  _media_sink = [cb = std::move(cb)](const media_frame &f, const void * /*src_conn*/) { cb(f); };
}

std::size_t state_transport_ws::pump_shard(state_cluster_shard &shard) {
  auto pending = shard.drain_local();
  for (const auto &m : pending)
    publish(m);
  return pending.size();
}

std::size_t state_transport_ws::pump_all() {
  std::vector<state_cluster_shard *> snap;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    snap = _shards;
  }
  std::size_t total = 0;
  for (auto *s : snap)
    if (s)
      total += pump_shard(*s);
  return total;
}

void state_transport_ws::flush() {
  // Acquire+release each connection's write mutex: once we can take it, any in-flight write on that
  // connection has completed (writes are synchronous under write_mu).
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c)
      continue;
    std::lock_guard<std::mutex> wlk(c->write_mu);
    (void)wlk;
  }
}

void state_transport_ws::on_inbound_mutation(const state_mutation &m, connection *origin) {
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (!s)
        continue;
      if (s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      peers.push_back(s);
    }
  }
  for (auto *peer : peers) {
    auto r = peer->ingest_remote(m);
    if (r.applied)
      _delivered.fetch_add(1, std::memory_order_relaxed);
  }
  // Relay (star topology): re-send to every OTHER connection so clients that only reach this server
  // converge with each other. origin is skipped (no echo); origin_node_id is preserved so the
  // receiver's dedup makes any round-trip a no-op.
  if (_relay.load(std::memory_order_relaxed)) {
    pb::Frame frame;
    encode_mutation(m, frame.mutable_mutation());
    std::vector<std::shared_ptr<connection>> conns;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      conns = _conns;
    }
    for (auto &c : conns) {
      if (!c || c.get() == origin || !c->alive.load() || !c->can_write())
        continue;
      if (c->send_backlog() > 2 * 1024 * 1024)
        continue; // don't block the relay on a backed-up peer; the ~1s roster broadcast backstops
                  // it
      if (c->write(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Bridge to a sibling transport (the relay's other backend) so its connections get this mutation.
  if (_bridge)
    _bridge->publish(m);
}

void state_transport_ws::on_inbound_message(const state_message &m, connection *origin) {
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (!s)
        continue;
      if (!m.cluster_id.empty() && s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      peers.push_back(s);
    }
  }
  for (auto *peer : peers)
    (void)peer->ingest_remote_message(m);
  if (_relay.load(std::memory_order_relaxed)) {
    pb::Frame frame;
    encode_message(m, frame.mutable_message());
    std::vector<std::shared_ptr<connection>> conns;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      conns = _conns;
    }
    for (auto &c : conns) {
      if (!c || c.get() == origin || !c->alive.load() || !c->can_write())
        continue;
      if (c->send_backlog() > 2 * 1024 * 1024)
        continue; // don't block the relay on a backed-up peer; the ~1s roster broadcast backstops
                  // it
      if (c->write(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Bridge to a sibling transport (the relay's other backend) so its connections get this message.
  if (_bridge)
    _bridge->publish_message(m);
}

void state_transport_ws::register_connection(std::shared_ptr<connection> conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.push_back(std::move(conn));
}

void state_transport_ws::admit_connection(const std::shared_ptr<connection> &conn) {
  conn->last_rx_ns.store(steady_ns(),
                         std::memory_order_relaxed); // don't reap a just-admitted client
  // Snapshot the shard list before taking the write mutex (every other path releases _shards_mu
  // before touching a connection's write mutex — keep that order to avoid a lock cycle).
  const bool backfill = _backfill_on_connect.load(std::memory_order_acquire);
  const bool relay = _relay.load(std::memory_order_relaxed);
  std::vector<state_cluster_shard *> shards;
  if (backfill || relay) {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  // Hold write_mu across publication so a concurrent publish() cannot slip a newer mutation ahead
  // of the replayed older ones (the receiver's seen-set only suppresses exact repeats).
  std::lock_guard<std::mutex> wlk(conn->write_mu);
  register_connection(conn);
  send_hello_locked(*conn);
  if (backfill && !shards.empty())
    send_backfill_locked(*conn, shards);
  // A relay pushes its full current state so a late-joining client sees peers already in the room
  // (the live set_relay forwarding only carries writes that happen AFTER this client connects).
  if (relay && !shards.empty())
    send_snapshot_locked(*conn, shards);
}

void state_transport_ws::send_hello_locked(connection &c) {
  pb::Frame f;
  auto *h = f.mutable_hello();
  h->set_node_id(_node_id);
  h->set_cluster_id(_cluster_id);
  c.write_locked(f);
}

void state_transport_ws::send_backfill_locked(connection &c,
                                              const std::vector<state_cluster_shard *> &shards) {
  // The peer's Hello has not arrived yet, so send everything and let the receiver drop what does
  // not match its cluster or its interest set, exactly as for live traffic.
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    for (const auto &m : s->replay_local()) {
      pb::Frame f;
      encode_mutation(m, f.mutable_mutation());
      if (!c.write_locked(f))
        return; // stream died mid-backfill; the reader will reap it
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      _backfilled.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void state_transport_ws::send_snapshot_locked(connection &c,
                                              const std::vector<state_cluster_shard *> &shards) {
  // A relay holds the whole room tree (every client's forwarded writes land in its shard), but
  // those entries are REMOTE-origin, so send_backfill_locked's replay_local() yields nothing. Push
  // the full current snapshot instead, as an unsolicited SnapshotResponse (request_id 0), so a late
  // joiner bootstraps the peers already present. The receiver applies each entry via
  // ingest_snapshot_entry (a bulk initial load, no conflict gate).
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    const auto snap = s->snapshot();
    if (snap.empty())
      continue;
    pb::Frame f;
    auto *rsp = f.mutable_snapshot_response();
    rsp->set_request_id(0);
    rsp->set_final(true);
    for (const auto &e : snap) {
      auto *pe = rsp->add_entries();
      pe->set_path(e.path);
      pe->set_string_value(e.string_value);
      pe->set_comment(e.comment);
      pe->set_hidden(e.hidden);
      pe->set_read_only(e.read_only);
      for (const auto &p : e.properties)
        (*pe->mutable_properties())[p.first] = p.second;
      pe->set_type_name(e.type_name);
      pe->set_origin_node_id(e.origin_node_id);
      pe->set_sequence(e.sequence);
      pe->set_data_type_name(e.data_type_name);
      if (e.data_payload.kind != state_payload_kind::none)
        set_pb_payload(pe->mutable_data_payload(), e.data_payload);
    }
    if (!c.write_locked(f))
      return; // stream died mid-snapshot; the reader will reap it
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

std::size_t state_transport_ws::broadcast_snapshot() {
  // Build each shard's snapshot frame ONCE, then write it to every live connection. The receivers
  // apply it idempotently (ingest_snapshot_entry is a bulk load), so re-sending the same
  // roster/scene each interval is a no-op for a client that is already converged and a recovery for
  // one that missed a live re-fan. Uses connection::write() (locks per connection) — do NOT hold a
  // connection write lock here. Snapshotting the shard reads the tree under its own lock.
  std::vector<state_cluster_shard *> shards;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  std::vector<pb::Frame> frames;
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    const auto snap = s->snapshot();
    if (snap.empty())
      continue;
    pb::Frame f;
    auto *rsp = f.mutable_snapshot_response();
    rsp->set_request_id(0);
    rsp->set_final(true);
    for (const auto &e : snap) {
      auto *pe = rsp->add_entries();
      pe->set_path(e.path);
      pe->set_string_value(e.string_value);
      pe->set_comment(e.comment);
      pe->set_hidden(e.hidden);
      pe->set_read_only(e.read_only);
      for (const auto &p : e.properties)
        (*pe->mutable_properties())[p.first] = p.second;
      pe->set_type_name(e.type_name);
      pe->set_origin_node_id(e.origin_node_id);
      pe->set_sequence(e.sequence);
      pe->set_data_type_name(e.data_type_name);
      if (e.data_payload.kind != state_payload_kind::none)
        set_pb_payload(pe->mutable_data_payload(), e.data_payload);
    }
    frames.push_back(std::move(f));
  }
  if (frames.empty())
    return 0;
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  // Diagnostic (off unless CVC_WS_BROADCAST_DEBUG is set): per-tick summary of how many peers the
  // roster broadcast reached vs. skipped, and the worst backlog observed. This is the decisive
  // signal for "clients can't see each other" — a skipped peer is one the periodic roster never
  // reaches.
  static const bool dbg = std::getenv("CVC_WS_BROADCAST_DEBUG") != nullptr;
  std::size_t wrote = 0, skipped = 0, max_backlog = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    const std::size_t bl = c->send_backlog();
    if (bl > max_backlog)
      max_backlog = bl;
    // Skip a peer whose send buffer is badly backed up: writing the (multi-entry) snapshot would
    // block the whole broadcast loop on that one slow peer, starving every other client. This is
    // periodic
    // (~1s), so a skipped peer simply catches up on the next tick once its buffer drains. The media
    // drop keeps a peer's buffer from filling with video, so at 2 MB this only trips for a peer
    // whose socket is effectively dead (not draining state at all) — exactly when we must not
    // block.
    if (bl > 2 * 1024 * 1024) {
      ++skipped;
      if (dbg)
        std::fprintf(stderr, "[ws-bcast] SKIP node='%s' backlog=%zuKB\n", c->remote_node_id.c_str(),
                     bl / 1024);
      continue;
    }
    bool ok = true;
    for (const auto &f : frames)
      ok = c->write(f) && ok;
    if (ok) {
      ++wrote;
      _sent_frames.fetch_add(frames.size(), std::memory_order_relaxed);
    }
  }
  if (dbg)
    std::fprintf(stderr, "[ws-bcast] wrote=%zu skipped=%zu maxbacklog=%zuKB conns=%zu frames=%zu\n",
                 wrote, skipped, max_backlog / 1024, conns.size(), frames.size());
  return wrote;
}

void state_transport_ws::unregister_connection(connection *conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.erase(
      std::remove_if(_conns.begin(), _conns.end(),
                     [conn](const std::shared_ptr<connection> &c) { return c.get() == conn; }),
      _conns.end());
}

std::size_t state_transport_ws::reap_idle(std::chrono::milliseconds timeout) {
  std::size_t reaped = 0;
#ifndef __EMSCRIPTEN__
  const std::int64_t now = steady_ns();
  const std::int64_t to_ns = static_cast<std::int64_t>(timeout.count()) * 1000000;
  std::vector<std::shared_ptr<connection>> snap;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    snap = _conns;
  }
  // 1) Shut down connections gone silent past the timeout. A conference client streams media
  // continuously, so a peer that navigated/reloaded away (its socket not cleanly closed) stops
  // sending; shutdown() wakes its reader's blocked read(), which fires _on_conn_closed (retracting
  // the ghost) and sets reader_done. Same mechanism stop() uses to interrupt a blocked read.
  for (auto &c : snap) {
    if (!c || !c->alive.load() || !c->ws)
      continue;
    const std::int64_t last = c->last_rx_ns.load(std::memory_order_relaxed);
    if (last != 0 && (now - last) > to_ns) {
      std::fprintf(stderr, "[ws] reaping idle connection node='%s' idle=%.1fs\n",
                   c->remote_node_id.c_str(), static_cast<double>(now - last) / 1.0e9);
      beast::error_code ec;
      c->ws->next_layer().shutdown(tcp::socket::shutdown_both, ec);
      c->ws->next_layer().close(ec);
      c->alive.store(false);
    }
  }
  // 2) Sweep connections whose reader has returned (dropped peer OR just-reaped): join + erase, so
  // _conns and the finished reader threads do not accumulate across a session's reconnects.
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (auto it = _conns.begin(); it != _conns.end();) {
      connection *c = it->get();
      if (c && c->reader_done.load(std::memory_order_acquire)) {
        if (c->reader.joinable())
          c->reader.join();
        it = _conns.erase(it);
        ++reaped;
      } else {
        ++it;
      }
    }
  }
#else
  (void)timeout;
#endif
  return reaped;
}

std::size_t state_transport_ws::shard_count() const {
  std::lock_guard<std::mutex> lk(_shards_mu);
  return _shards.size();
}

std::size_t state_transport_ws::connection_count() const {
  std::lock_guard<std::mutex> lk(_conns_mu);
  std::size_t n = 0;
  for (auto &c : _conns)
    if (c && c->alive.load())
      ++n;
  return n;
}

std::uint64_t state_transport_ws::wait_for_received(std::uint64_t target,
                                                    std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lk(_recv_mu);
  _recv_cv.wait_for(lk, timeout, [&]() { return _recv_mutations.load() >= target; });
  return _recv_mutations.load();
}

} // namespace cvc

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
#else
#include <cstdint>
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

#ifndef __EMSCRIPTEN__
  std::unique_ptr<websocket::stream<tcp::socket>> ws;
  std::thread reader;

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

// Parse one inbound Frame (serialized protobuf bytes) and dispatch it. Shared by the native reader
// loop and the browser onmessage callback. MVP handles Hello / Mutation / Message; chunk / snapshot
// / heartbeat are Phase 2 (the base-class defaults cover their request paths).
void state_transport_ws::deliver_inbound(connection *c, const void *data, std::size_t len) {
  increment_recv_frames();
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
  }
}

void state_transport_ws::register_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  if (std::find(_shards.begin(), _shards.end(), shard) == _shards.end())
    _shards.push_back(shard);
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
  // Snapshot the shard list before taking the write mutex (every other path releases _shards_mu
  // before touching a connection's write mutex — keep that order to avoid a lock cycle).
  std::vector<state_cluster_shard *> shards;
  if (_backfill_on_connect.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  // Hold write_mu across publication so a concurrent publish() cannot slip a newer mutation ahead
  // of the replayed older ones (the receiver's seen-set only suppresses exact repeats).
  std::lock_guard<std::mutex> wlk(conn->write_mu);
  register_connection(conn);
  send_hello_locked(*conn);
  if (!shards.empty())
    send_backfill_locked(*conn, shards);
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

void state_transport_ws::unregister_connection(connection *conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.erase(
      std::remove_if(_conns.begin(), _conns.end(),
                     [conn](const std::shared_ptr<connection> &c) { return c.get() == conn; }),
      _conns.end());
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

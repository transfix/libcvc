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
#include <cctype>
#include <cstdio>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_device_key.h>
#include <cvc/state/state_identity.h> // constant_time_equal
#include <cvc/state/state_transport_ws.h>
#include <future>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#ifndef __EMSCRIPTEN__
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
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

// Append one entry to `rsp`. The admit snapshot, the periodic broadcast and the targeted push all
// encode through here, so their entry bytes agree.
void add_snapshot_entry(pb::SnapshotResponse *rsp, const state_transport::snapshot_entry &e) {
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

// Encode `snap` into `rsp` as one unsolicited, final snapshot (request_id 0). With a `view`, only
// the entries it delivers are written (the receiving peer's rule); nullptr writes them all.
void fill_snapshot_response(pb::SnapshotResponse *rsp,
                            const std::vector<state_transport::snapshot_entry> &snap,
                            const state_peer_registry::delivery_view *view = nullptr) {
  rsp->set_request_id(0);
  rsp->set_final(true);
  for (const auto &e : snap)
    if (view == nullptr || view->deliver(e.path))
      add_snapshot_entry(rsp, e);
}

// FNV-1a 64 over length-prefixed fields: everything an entry carries except its path (the key it
// is remembered under), so two digests match only when a receiver would end up with the same node.
class entry_digest {
public:
  explicit entry_digest(const state_transport::snapshot_entry &e) {
    str(e.string_value);
    str(e.comment);
    u64((e.hidden ? 1u : 0u) | (e.read_only ? 2u : 0u));
    u64(e.properties.size());
    for (const auto &p : e.properties) {
      str(p.first);
      str(p.second);
    }
    str(e.type_name);
    str(e.origin_node_id);
    u64(e.sequence);
    str(e.data_type_name);
    const state_payload &pl = e.data_payload;
    u64(static_cast<std::uint64_t>(pl.kind));
    if (pl.kind == state_payload_kind::inline_bytes) {
      u64(pl.inline_bytes.size());
      bytes(pl.inline_bytes.data(), pl.inline_bytes.size());
    } else if (pl.kind == state_payload_kind::blob) {
      str(pl.blob.digest);
      u64(pl.blob.size_bytes);
      str(pl.blob.codec);
    }
  }
  std::uint64_t value() const noexcept { return _h; }

private:
  void bytes(const void *data, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < n; ++i) {
      _h ^= b[i];
      _h *= 1099511628211ull;
    }
  }
  void u64(std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      const unsigned char b = static_cast<unsigned char>(v >> (8 * i));
      bytes(&b, 1);
    }
  }
  void str(const std::string &s) {
    u64(s.size());
    bytes(s.data(), s.size());
  }
  std::uint64_t _h = 14695981039346656037ull;
};

// What one connection holds of a tree path, as far as the relay knows: the digest of the content
// the last snapshot frame carried for it, and the broadcast round that last delivered the path.
// File-local, so the per-connection map stays out of a DLL's export table.
struct held_entry {
  std::uint64_t digest = 0;
  std::uint64_t round = 0;
};

// A mutation that can change nodes below its path, not just the node at it.
bool touches_subtree(state_mutation_op op) {
  return op == state_mutation_op::reset_node || op == state_mutation_op::delete_subtree;
}

} // namespace

namespace {

// The subprotocol a cvc WebSocket peer speaks, and the prefix of the one that carries a room token
// (state_transport_ws::auth_config).
constexpr const char *kWsProtocol = "cvc.v1";
constexpr const char *kTokenProtocol = "cvc-token.";

// The Sec-WebSocket-Protocol offer of a client holding room token TOKEN. No spaces: the browser's
// WebSocket API takes the list split on commas alone.
std::string protocol_offer(const std::string &token) {
  return std::string(kWsProtocol) + "," + kTokenProtocol + token;
}

} // namespace

#ifndef __EMSCRIPTEN__
namespace {
namespace beast = boost::beast;
namespace websocket = boost::beast::websocket;
namespace http = boost::beast::http;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

// Whether the Sec-WebSocket-Protocol list OFFERED names WANT (each entry trimmed, all of them
// compared in constant time).
bool offers_protocol(const std::string &offered, const std::string &want) {
  bool found = false;
  std::size_t start = 0;
  while (start <= offered.size()) {
    std::size_t end = offered.find(',', start);
    if (end == std::string::npos)
      end = offered.size();
    std::size_t b = start, e = end;
    while (b < e && std::isspace(static_cast<unsigned char>(offered[b])))
      ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(offered[e - 1])))
      --e;
    found |= constant_time_equal(offered.substr(b, e - b), want);
    start = end + 1;
  }
  return found;
}

// Answer an upgrade request with 401 and close: the room token was missing or wrong.
void refuse_upgrade(tcp::socket &sock, const http::request<http::string_body> &req) {
  http::response<http::string_body> res{http::status::unauthorized, req.version()};
  res.set(http::field::content_type, "text/plain");
  res.keep_alive(false);
  res.body() = "room token required\n";
  res.prepare_payload();
  beast::error_code ec;
  http::write(sock, res, ec);
  sock.shutdown(tcp::socket::shutdown_both, ec);
  sock.close(ec);
}

// How long one write may block before its connection is closed (see
// connection::write_started_ns).
constexpr std::int64_t kWriteStallNs = 3'000'000'000;
// How long an accepted connection may take to finish its WebSocket handshake, and how many may be
// in one at once (each holds a thread until it finishes or is closed).
constexpr std::int64_t kHandshakeNs = 10'000'000'000;
constexpr std::size_t kMaxHandshakes = 64;

// Make a write blocked on `s` fail after kWriteStallNs, so it releases the connection's write lock.
// Elsewhere the probe thread's watchdog does that with shutdown(), which wakes a send() blocked in
// another thread; Winsock's shutdown() does not, so on Windows the socket times the send out itself
// (SO_SNDTIMEO), and the write fails with WSAETIMEDOUT. A timed-out socket is unusable afterwards,
// which is fine: the failed write marks the connection dead.
void bound_blocking_writes(tcp::socket &s) {
#ifdef _WIN32
  const DWORD ms = static_cast<DWORD>(kWriteStallNs / 1'000'000);
  ::setsockopt(s.native_handle(), SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&ms),
               sizeof(ms));
#else
  (void)s;
#endif
}
} // namespace
#endif

// ── connection: one bidirectional WebSocket stream. One reader thread drains it; writes serialize
// on write_mu so the publish() fan-out and the per-connection Hello/backfill interleave correctly
// (one outstanding read + one outstanding write is the only concurrency Beast's stream permits). ──
class state_transport_ws::connection {
public:
  std::mutex write_mu;
  std::atomic<bool> alive{true};
  // The peer's identity from its FIRST Hello. The connection's reader (the one thread that parses
  // its inbound frames) writes both strings once and then sets `identified` (release); a later
  // Hello is ignored, so they never change after that. Every other thread reads through peer_id().
  std::string remote_node_id;
  std::string remote_cluster_id;
  std::string remote_device_id; // the device id of that Hello ("" when none), written the same way
  std::atomic<bool> identified{false};
  // The peer's node id, or nullptr before its Hello (or when that Hello named no node). nullptr
  // means "unknown", which delivers everything, as an unregistered peer does.
  const std::string *peer_id() const noexcept {
    if (!identified.load(std::memory_order_acquire) || remote_node_id.empty())
      return nullptr;
    return &remote_node_id;
  }
  // The inbound gate (state_transport::set_inbound_gate) this ACCEPTED connection is held to,
  // copied before its reader starts; null for a dialed connection or an ungated relay. `bound`: it
  // held its node id at its last check, and `displaced`: a later claim took it over since (set by
  // the gate). Used only on this connection's reader thread, apart from `displaced`.
  std::shared_ptr<state_inbound_gate> gate;
  bool bound = false;
  std::atomic<bool> displaced{false};
  // Its Hello named one of the relay's own ids (the transport's or a shard's): never a client's.
  bool local_id = false;
  // Device proof (state_device_key.h), on an ACCEPTED connection:
  //   - device_nonce: the challenge the relay's Hello carried, taken by the first proof checked
  //     (guarded by proof_mu, as the Hello may go out on another thread);
  //   - pending_*: a proof that came before the client's own Hello (its reader can answer ours
  //     before its Hello is written), checked once that Hello names the device; reader only;
  //   - device_verified: the proof checked out against the device id's pin.
  std::mutex proof_mu;
  std::string device_nonce;
  bool proof_pending = false;
  std::string pending_key, pending_signature;
  std::atomic<bool> device_verified{false};
  // Whether this connection may speak now: always without a gate; with one, once it has identified
  // and holds its node id (a duplicate claim retries here until the holder closes or falls silent).
  bool speaks() {
    if (!gate)
      return true;
    if (local_id)
      return false;
    if (bound && displaced.load(std::memory_order_relaxed))
      bound = false;
    if (!bound) {
      const std::string *pid = peer_id();
      displaced.store(false, std::memory_order_relaxed);
      bound = pid != nullptr && gate->bind(*pid, this, &last_rx_ns, &displaced);
    }
    return bound;
  }
  // As speaks(), naming its own node id as `origin`.
  bool speaks_as(const std::string &origin) {
    return speaks() && (!gate || origin == remote_node_id);
  }
  // Give the node id up at close. True when this close is that peer leaving: always without a
  // gate; with one, when it held the id until now.
  bool unbind() {
    bound = false;
    if (!gate)
      return true;
    return identified.load() && gate->release(remote_node_id, this);
  }
  // Steady-clock ns of the last inbound frame. A live conference client streams media continuously,
  // so a peer that navigated away (its socket not cleanly closed) stops updating this — the relay's
  // reaper uses it to drop the ghost. 0 until the first frame; set to "now" when a connection opens
  // so a brand-new, not-yet-speaking client is not reaped.
  std::atomic<std::int64_t> last_rx_ns{0};
  // Link quality (state_link_stats): traffic both ways + the Ping/Pong round trip. `accepted` marks
  // the server end; opened_ns (steady-clock) is the report's age base, set at registration.
  state_link_stats link;
  bool accepted = false;
  std::int64_t opened_ns = 0;
  // Steady-clock ns when the write in progress started; 0 when none is. A peer that stops reading
  // (a frozen tab, a dead network) makes a write block once its TCP buffer is full -- while holding
  // write_mu, so every other writer to it (the relay's main loop included) would wait forever. The
  // probe thread's watchdog closes a connection whose write has been blocked too long, which fails
  // the write and frees the lock (state_transport_ws::ping_loop); on Windows the socket's own send
  // timeout fails it (bound_blocking_writes).
  std::atomic<std::int64_t> write_started_ns{0};
  // Steady-clock ns when an accepted connection's handshake began; 0 once it is over (the probe
  // thread's handshake deadline, state_transport_ws::ping_loop).
  std::atomic<std::int64_t> handshake_started_ns{0};
  // Waiting for its bootstrap (the admission hold): write(), try_write(), write_mutation(),
  // write_entries() and broadcast_snapshot() send it nothing until release_locked clears it. Set
  // and cleared only under write_mu, so a write that saw it set happened before the release took
  // its snapshot, and so is in it. The admission's own frames go through write_locked.
  std::atomic<bool> admission_held{false};
  // What this peer holds, path by path, for broadcast_snapshot's diff (see held_entry): a snapshot
  // frame records what it carried; any other frame touching the path, written to the peer or
  // received from it, forgets it. Guarded by held_mu, which is never held across a socket write (a
  // reader thread forgets its peer's own writes, and must not wait behind a blocked write). A
  // WRITER also holds write_mu from its record/forget through its write, so the record always
  // describes the frames in the order the peer receives them. Lock order: write_mu, then held_mu.
  std::mutex held_mu;
  std::unordered_map<std::string, held_entry> held;
  // Set when the peer sends a SnapshotRequest: its next broadcast carries everything it is
  // delivered, not just the diff.
  std::atomic<bool> full_refresh{false};

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
    write_started_ns.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count(),
                           std::memory_order_relaxed);
    ws->write(net::buffer(bytes), ec);
    write_started_ns.store(0, std::memory_order_relaxed);
    if (ec) {
      alive.store(false);
      return false;
    }
    link.note_tx(bytes.size());
    return true;
  }

  // Unblock any read or write in progress and mark the connection dead (the write watchdog). On
  // Windows shutdown() does not wake a blocked send(); bound_blocking_writes times that one out.
  void abort_io() {
    alive.store(false);
    if (ws) {
      beast::error_code ec;
      ws->next_layer().shutdown(tcp::socket::shutdown_both, ec);
    }
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
    if (emscripten_websocket_send_binary(em_socket, const_cast<char *>(bytes.data()),
                                         static_cast<uint32_t>(bytes.size())) < 0)
      return false;
    link.note_tx(bytes.size());
    return true;
  }
#endif

  bool write(const pb::Frame &f) {
    std::lock_guard<std::mutex> lk(write_mu);
    return !admission_held.load(std::memory_order_relaxed) && write_locked(f);
  }

  // Write only if no other write holds the connection right now (the link probe never waits).
  bool try_write(const pb::Frame &f) {
    std::unique_lock<std::mutex> lk(write_mu, std::try_to_lock);
    return lk.owns_lock() && !admission_held.load(std::memory_order_relaxed) && write_locked(f);
  }

  // The peer's copy of `m.path` (and, for a subtree op, of everything below it) is no longer
  // known, so the next broadcast sends it whatever the tree holds there.
  void forget(const state_mutation &m) {
    std::lock_guard<std::mutex> lk(held_mu);
    if (!touches_subtree(m.op)) {
      held.erase(m.path);
      return;
    }
    for (auto it = held.begin(); it != held.end();)
      it = state_peer_registry::prefix_matches(m.path, it->first) ? held.erase(it) : std::next(it);
  }

  // Write a frame carrying the live mutation `m`, forgetting what the peer held at its path.
  bool write_mutation(const pb::Frame &f, const state_mutation &m) {
    std::lock_guard<std::mutex> lk(write_mu);
    if (admission_held.load(std::memory_order_relaxed))
      return false;
    forget(m);
    return write_locked(f);
  }

  // Write a snapshot frame carrying `entries` outside a broadcast (a targeted push), forgetting
  // each entry's path.
  bool write_entries(const pb::Frame &f,
                     const std::vector<state_transport::snapshot_entry> &entries) {
    std::lock_guard<std::mutex> lk(write_mu);
    if (admission_held.load(std::memory_order_relaxed))
      return false;
    {
      std::lock_guard<std::mutex> hk(held_mu);
      for (const auto &e : entries)
        held.erase(e.path);
    }
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
#elif defined(__EMSCRIPTEN__)
    // The browser's own queue (WebSocket.bufferedAmount): a send never blocks there, so without
    // this a slow uplink only grows the queue, and every frame behind it -- the link probe too --
    // waits longer.
    size_t buffered = 0;
    if (em_socket >= 0 &&
        emscripten_websocket_get_buffered_amount(em_socket, &buffered) == EMSCRIPTEN_RESULT_SUCCESS)
      return buffered;
#endif
    return 0;
  }
};

#ifndef __EMSCRIPTEN__
struct state_transport_ws::impl {
  net::io_context ioc;
  std::unique_ptr<tcp::acceptor> acceptor;
  std::thread accept_thread;
  std::thread ping_thread; // link probe (ping_loop), joined by stop()
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
  if (!_ping_started.exchange(true))
    _impl->ping_thread = std::thread([this]() { ping_loop(); });
}

static std::int64_t steady_ns(); // defined with the reaper below

// Server accept loop: synchronous accept, then a thread per connection for its handshake, admission
// and reads (serve_accepted). Breaks when stop() closes the acceptor (accept() then errors).
void state_transport_ws::accept_loop() {
  for (;;) {
    tcp::socket sock(_impl->ioc);
    beast::error_code ec;
    _impl->acceptor->accept(sock, ec);
    if (ec)
      break; // acceptor closed by stop(), or a fatal accept error
    if (!_running.load())
      break; // woken by stop()'s loopback self-connect

    // The handshake runs on the connection's own thread, so a client that connects and never sends
    // its upgrade request holds up no other join. At most kMaxHandshakes wait at once; the probe
    // thread closes one that outlasts kHandshakeNs.
    reap_handshakes();
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      if (_handshaking.size() >= kMaxHandshakes) {
        sock.shutdown(tcp::socket::shutdown_both, ec);
        sock.close(ec);
        continue;
      }
    }
    auto conn = std::make_shared<connection>();
    conn->accepted = true;
    {
      std::lock_guard<std::mutex> lk(_inbound_gate_mu);
      conn->gate = _inbound_gate;
    }
    conn->ws = std::make_unique<websocket::stream<tcp::socket>>(std::move(sock));
    conn->ws->read_message_max(kMaxWsMessageBytes);
    bound_blocking_writes(conn->ws->next_layer());
    conn->handshake_started_ns.store(steady_ns(), std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      _handshaking.push_back(conn);
    }
    // The thread waits until its handle is assigned: once admitted, the connection is visible to
    // stop() and reap_idle(), which join it.
    auto assigned = std::make_shared<std::promise<void>>();
    std::shared_future<void> go = assigned->get_future().share();
    conn->reader = std::thread([this, conn, go]() {
      go.wait();
      serve_accepted(conn);
    });
    assigned->set_value();
  }
}

bool state_transport_ws::handshake_accepted(connection &c) {
  // The upgrade request first: a relay with a room token checks it there, before the connection
  // is admitted, and a client that offered subprotocols must be answered one (a browser drops the
  // connection otherwise).
  beast::error_code ec;
  beast::flat_buffer hbuf;
  http::request<http::string_body> req;
  http::read(c.ws->next_layer(), hbuf, req, ec);
  if (ec || !websocket::is_upgrade(req))
    return false;
  const std::string offered(req[http::field::sec_websocket_protocol]);
  if (!_auth.expected_token.empty() &&
      !offers_protocol(offered, kTokenProtocol + _auth.expected_token)) {
    _auth_rejected.fetch_add(1, std::memory_order_relaxed);
    refuse_upgrade(c.ws->next_layer(), req);
    return false;
  }
  if (offers_protocol(offered, kWsProtocol))
    c.ws->set_option(websocket::stream_base::decorator([](websocket::response_type &res) {
      res.set(http::field::sec_websocket_protocol, kWsProtocol);
    }));
  c.ws->accept(req, ec); // server-side WebSocket handshake
  return !ec;
}

void state_transport_ws::serve_accepted(std::shared_ptr<connection> conn) {
  const bool ok = handshake_accepted(*conn) && _running.load();
  conn->handshake_started_ns.store(0, std::memory_order_relaxed);
  if (!ok) {
    // Left in _handshaking for the accept loop (or stop()) to join.
    conn->alive.store(false);
    conn->reader_done.store(true, std::memory_order_release);
    return;
  }
  admit_connection(conn); // now in _conns, then out of _handshaking: never in neither
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    _handshaking.erase(std::remove(_handshaking.begin(), _handshaking.end(), conn),
                       _handshaking.end());
  }
  read_loop(conn);
}

void state_transport_ws::reap_handshakes() {
  std::vector<std::shared_ptr<connection>> done;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (auto it = _handshaking.begin(); it != _handshaking.end();) {
      if ((*it)->reader_done.load(std::memory_order_acquire)) {
        done.push_back(std::move(*it));
        it = _handshaking.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (auto &c : done)
    if (c->reader.joinable())
      c->reader.join();
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
    bound_blocking_writes(conn->ws->next_layer());
    if (!_auth.outbound_token.empty())
      conn->ws->set_option(websocket::stream_base::decorator(
          [offer = protocol_offer(_auth.outbound_token)](websocket::request_type &req) {
            req.set(http::field::sec_websocket_protocol, offer);
          }));
    websocket::response_type res;
    conn->ws->handshake(res, host, target.empty() ? std::string("/") : target, ec);
    if (ec) {
      if (res.result() == http::status::unauthorized)
        std::fprintf(stderr, "cvc: ws: %s refused the connection: room token missing or wrong\n",
                     url.c_str());
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }

  conn->reader = std::thread([this, conn]() { read_loop(conn); });
  _running.store(true); // so stop() tears the client connection down
  admit_connection(conn);
  if (!_ping_started.exchange(true))
    _impl->ping_thread = std::thread([this]() { ping_loop(); });
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
  // real peer drop, and only when the peer identified itself (sent a Hello). Under an inbound gate,
  // only the connection holding that node id leaves as it: another claiming it is not that peer.
  const bool left = conn->unbind();
  if (const std::string *pid = conn->peer_id(); pid && left && _running.load() && _on_conn_closed)
    _on_conn_closed(*pid);
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
    // Connections still in their handshake: one admitted meanwhile is in both lists, which is fine
    // (a second shutdown is harmless, and a joined thread is no longer joinable).
    conns.insert(conns.end(), _handshaking.begin(), _handshaking.end());
    _handshaking.clear();
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
  if (_impl->ping_thread.joinable())
    _impl->ping_thread.join();
  _ping_started.store(false);
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
  // With a room token, offer it as a subprotocol (auth_config); otherwise offer none, so an older
  // relay that answers no subprotocol still accepts the browser.
  const std::string offer =
      _auth.outbound_token.empty() ? std::string() : protocol_offer(_auth.outbound_token);
  if (!offer.empty())
    attrs.protocols = offer.c_str();
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
    send_backfill_locked(*c, shards, nullptr);
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
  c->link.note_rx(len);
  pb::Frame in;
  if (!in.ParseFromArray(data, static_cast<int>(len)))
    return;
  // Link probe: answer a Ping at once (in a browser this runs in the WebSocket onmessage event,
  // which a backgrounded tab still receives), and measure a Pong against our own clock.
  if (in.has_ping()) {
    pb::Frame f;
    f.mutable_pong()->set_seq(in.ping().seq());
    f.mutable_pong()->set_sent_ns(in.ping().sent_ns());
    if (c->write(f))
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (in.has_pong()) {
    c->link.note_rtt(steady_ns() - in.pong().sent_ns());
    return;
  }
  if (in.has_hello()) {
    // Write-once identity: only the first Hello counts (see connection::identified). This thread
    // is the only writer, so the relaxed check cannot race another Hello.
    if (!c->identified.load(std::memory_order_relaxed)) {
      c->remote_node_id = in.hello().node_id();
      c->remote_cluster_id = in.hello().cluster_id();
      // An id that is not a valid_device_id (oversized, say) is ignored, not stored.
      c->remote_device_id =
          valid_device_id(in.hello().device_id()) ? in.hello().device_id() : std::string();
      c->identified.store(true, std::memory_order_release);
      if (c->gate) {
        c->local_id = is_local_id(c->remote_node_id);
        (void)c->speaks(); // claim the node id now
      }
      // A relay that verifies device keys challenges a client it accepted; answer it.
      if (!c->accepted && !in.hello().nonce().empty())
        answer_device_challenge(*c, in.hello().nonce(), in.hello().node_id());
      // A proof that came first can be checked now that the Hello names the device.
      if (c->proof_pending) {
        c->proof_pending = false;
        const std::string key = std::move(c->pending_key);
        const std::string signature = std::move(c->pending_signature);
        on_device_proof(*c, key, signature);
      }
    }
  } else if (in.has_device_proof()) {
    on_device_proof(*c, in.device_proof().public_key(), in.device_proof().signature());
  } else if (in.has_mutation()) {
    on_inbound_mutation(decode_mutation(in.mutation()), c);
    _recv_mutations.fetch_add(1, std::memory_order_relaxed);
    { std::lock_guard<std::mutex> lk(_recv_mu); }
    _recv_cv.notify_all();
  } else if (in.has_message()) {
    on_inbound_message(decode_message(in.message()), c);
    _recv_messages.fetch_add(1, std::memory_order_relaxed);
  } else if (in.has_snapshot_request()) {
    // The peer asks for a full refresh (request_full_refresh): its next broadcast carries the whole
    // tree it is delivered. Answered there, on the relay's own loop, not on this reader thread.
    // Under a gate, only a peer that speaks as itself is answered.
    if (!c->speaks()) {
      _inbound_rejected.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    c->full_refresh.store(true, std::memory_order_relaxed);
  } else if (in.has_snapshot_response()) {
    // A relay pushed its full current state (request_id 0) so this late-joining client sees the
    // peers already in the room. Apply each entry as an initial-sync bootstrap. The conference uses
    // a single shard/cluster, so applying to every registered shard is correct here; the entries do
    // not carry a cluster id. A gated relay takes none from its clients (state_inbound_gate).
    if (c->gate) {
      _inbound_rejected.fetch_add(1, std::memory_order_relaxed);
      return;
    }
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
        // Honour the shard's enforced interest set as live ingest does: a relay snapshot carries
        // its whole tree, which would otherwise plant nodes this client never asked for. A shard
        // that does not enforce interest takes every entry, as before.
        if (!s->path_is_of_interest(se.path))
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
    if (!c->speaks_as(in.media().origin_node_id())) {
      _inbound_rejected.fetch_add(1, std::memory_order_relaxed); // media as another peer
      return;
    }
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
    const std::string *pid = c->peer_id();
    if (pid && !_peers.should_deliver(*pid, m.path)) {
      _peers.note_delivery_filtered(*pid);
      continue;
    }
    // Don't block the caller (the arisrv main loop drains local writes here) on a backed-up peer —
    // it would stall the loop and its periodic broadcast for everyone. The ~1s broadcast
    // re-delivers.
    if (c->send_backlog() > 2 * 1024 * 1024)
      continue;
    if (c->write_mutation(frame, m)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.delivered;
      if (pid)
        _peers.note_mutation_delivered(*pid);
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
    const std::string *pid = c->peer_id();
    if (pid && !_peers.should_deliver(*pid, m.path)) {
      _peers.note_delivery_filtered(*pid);
      continue;
    }
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.peers;
      if (pid)
        _peers.note_message_delivered(*pid);
    }
  }
  return stats;
}

std::size_t state_transport_ws::write_media(const media_frame &m, const void *exclude, bool gated) {
  // One copy of the gate per frame: a concurrent set_media_gate swaps the slot, never the object
  // this fan is calling, and no lock is held across the socket writes below.
  std::shared_ptr<const media_gate_fn> gate;
  std::shared_ptr<const media_shape_fn> shaper;
  if (gated) {
    std::lock_guard<std::mutex> lk(_media_gate_mu);
    gate = _media_gate;
    shaper = _media_shaper;
  }

  const auto encode = [](const media_frame &src, pb::Frame &out) {
    pb::MediaFrame *mf = out.mutable_media();
    mf->set_origin_node_id(src.origin_node_id);
    mf->set_stream_id(src.stream_id);
    mf->set_codec(src.codec);
    mf->set_width(src.width);
    mf->set_height(src.height);
    mf->set_seq(src.seq);
    mf->set_pts_seconds(src.pts_seconds);
    mf->set_sample_rate(src.sample_rate);
    mf->set_channels(src.channels);
    mf->set_payload(src.payload.data(), static_cast<int>(src.payload.size()));
  };
  pb::Frame frame;
  encode(m, frame);
  // The shaper's variants, each encoded once for every receiver it serves.
  std::vector<std::pair<const media_frame *, pb::Frame>> variants;
  const auto variant_frame = [&](const media_frame *v) -> const pb::Frame & {
    for (const auto &e : variants)
      if (e.first == v)
        return e.second;
    variants.emplace_back(v, pb::Frame());
    encode(*v, variants.back().second);
    return variants.back().second;
  };

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  // Media is unreplicated, un-bridged, and not subject to the peer path-filter (it carries no
  // path). `exclude` (a connection*) skips the origin peer when a relay fans; nullptr sends to all.
  const std::size_t backlog_drop = _media_backlog_drop.load(std::memory_order_relaxed);
  std::size_t n = 0;
  for (auto &c : conns) {
    // A held connection (the admission hold) gets no media yet; skipping it here also keeps the
    // gate from spending a thumbnail on it.
    if (!c || c.get() == exclude || !c->alive.load() || !c->can_write() ||
        c->admission_held.load(std::memory_order_relaxed))
      continue;
    // Media is loss-tolerant: if this peer's send buffer is backed up (it can't keep pace with the
    // video), DROP the frame rather than let the synchronous write block the relay. A LOW threshold
    // (32 KB unless set_media_backlog_drop says otherwise) is deliberate — it keeps each peer's
    // buffer small so the small, must-deliver STATE writes (roster join/leave + the periodic
    // re-broadcast below) are never crowded out or deferred. Dropping video to a slow peer is
    // invisible; a stalled roster makes clients unable to SEE each other at all.
    // Audio (a frame with a sample rate) is small and the first thing a listener misses, so it
    // has four times video's allowance: a congested link sheds video first.
    const std::size_t limit = m.sample_rate > 0 ? backlog_drop * 4 : backlog_drop;
    if (backlog_drop != 0 && c->send_backlog() > limit) {
      _media_backlog_dropped.fetch_add(1, std::memory_order_relaxed);
      c->link.note_media_dropped();
      continue;
    }
    // Per-receiver egress gate, asked last so a frame the backlog drop skips never counts as sent
    // in the gate's own bookkeeping (e.g. a thumbnail cadence). An unidentified receiver is
    // ungated: the gate has no id to decide on, and "unknown" delivers everywhere else too.
    if (gate) {
      const std::string *pid = c->peer_id();
      if (pid && !(*gate)(m, *pid)) {
        _media_gated.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
    }
    // Then its shape: the frame as sent, a smaller variant, or nothing.
    const pb::Frame *out = &frame;
    if (shaper) {
      const std::string *pid = c->peer_id();
      if (pid) {
        const media_shape s = (*shaper)(m, *pid);
        if (s.drop) {
          _media_shape_dropped.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        if (s.variant) {
          out = &variant_frame(s.variant.get());
          _media_shaped.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
    if (c->write(*out)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      c->link.note_media_sent();
      ++n;
    }
  }
  return n;
}

// A browser client has one connection (the relay), so publish_media_frame pushes its camera/mic UP;
// a relay publishing locally (uncommon) would send to all its connections. A node's own uplink is
// never gated.
void state_transport_ws::publish_media_frame(const ws_media_frame &m) {
  _media_sent.fetch_add(write_media(m, nullptr, /*gated=*/false), std::memory_order_relaxed);
}

// Router egress: fan one peer's frame to every OTHER connection (the origin is `exclude`), each
// receiver subject to the media gate.
void state_transport_ws::fan_media(const media_frame &f, const void *exclude) {
  _media_fwd.fetch_add(write_media(f, exclude, /*gated=*/true), std::memory_order_relaxed);
}

void state_transport_ws::set_media_sink(media_sink_fn sink) { _media_sink = std::move(sink); }

void state_transport_ws::set_media_gate(media_gate_fn gate) {
  std::shared_ptr<const media_gate_fn> next;
  if (gate)
    next = std::make_shared<const media_gate_fn>(std::move(gate));
  {
    std::lock_guard<std::mutex> lk(_media_gate_mu);
    _media_gate.swap(next);
  }
  // `next` now holds the previous gate; it is released here, outside the lock (a fan still using
  // its own copy keeps it alive until that fan finishes).
}

void state_transport_ws::set_media_shaper(media_shape_fn shaper) {
  std::shared_ptr<const media_shape_fn> next;
  if (shaper)
    next = std::make_shared<const media_shape_fn>(std::move(shaper));
  {
    std::lock_guard<std::mutex> lk(_media_gate_mu);
    _media_shaper.swap(next);
  }
}

void state_transport_ws::set_media_backlog_drop(std::size_t bytes) noexcept {
  _media_backlog_drop.store(bytes, std::memory_order_relaxed);
}

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
#ifdef __EMSCRIPTEN__
  send_pings_if_due(); // no ping thread in a browser: the frame pump drives the probe
#endif
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
  // A gated relay takes from a client only what it may write as itself (state_inbound_gate). What
  // it holds there is now unknown, so the next broadcast sends it the tree's value.
  if (origin != nullptr && origin->gate &&
      !(origin->speaks_as(m.origin_node_id) && origin->gate->admit(origin->remote_node_id, m))) {
    _inbound_rejected.fetch_add(1, std::memory_order_relaxed);
    origin->forget(m);
    return;
  }
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
  // The origin wrote this path itself, so what it holds there is no longer what it was last sent:
  // the next broadcast re-sends the tree's value, which also corrects a write this relay rejected.
  if (origin != nullptr)
    origin->forget(m);
  const bool relay = _relay.load(std::memory_order_relaxed);
  if (!relay && _bridge == nullptr)
    return; // a leaf: applied locally, never re-sent
  // Interest gate (opt-in): re-fan only what some local same-cluster shard admits -- its interest
  // set covers the path, or it is a delegation op, which ingest always takes. When every client
  // enforces the relay's interest, a withheld frame is one no client would apply. Asked after
  // ingest_remote has returned, so admits() takes only the shard's leaf lock. With no matching
  // shard (a pure forwarding relay) everything is forwarded.
  if (_relay_requires_interest.load(std::memory_order_relaxed) && !peers.empty() &&
      std::none_of(peers.begin(), peers.end(),
                   [&m](const state_cluster_shard *s) { return s->admits(m); })) {
    _relay_withheld.fetch_add(1, std::memory_order_relaxed);
    return; // neither the relay loop nor the bridge
  }
  // Relay (star topology): re-send to every OTHER connection so clients that only reach this server
  // converge with each other. origin is skipped (no echo); origin_node_id is preserved so the
  // receiver's dedup makes any round-trip a no-op. Each target's registry entry is honoured.
  if (relay) {
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
      if (const std::string *pid = c->peer_id(); pid && !_peers.should_deliver(*pid, m.path)) {
        _peers.note_delivery_filtered(*pid);
        continue;
      }
      if (c->send_backlog() > 2 * 1024 * 1024)
        continue; // don't block the relay on a backed-up peer; the ~1s roster broadcast backstops
                  // it
      if (c->write_mutation(frame, m))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Bridge to a sibling transport (the relay's other backend) so its connections get this mutation.
  if (_bridge)
    _bridge->publish(m);
}

void state_transport_ws::on_inbound_message(const state_message &m, connection *origin) {
  if (origin != nullptr && origin->gate &&
      !(origin->speaks_as(m.origin_node_id) && origin->gate->admit(origin->remote_node_id, m))) {
    _inbound_rejected.fetch_add(1, std::memory_order_relaxed);
    return;
  }
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
  const bool relay = _relay.load(std::memory_order_relaxed);
  if (!relay && _bridge == nullptr)
    return;
  // Interest gate (opt-in), as in on_inbound_mutation.
  if (_relay_requires_interest.load(std::memory_order_relaxed) && !peers.empty() &&
      std::none_of(peers.begin(), peers.end(),
                   [&m](const state_cluster_shard *s) { return s->admits_message(m); })) {
    _relay_withheld.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (relay) {
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
      if (const std::string *pid = c->peer_id(); pid && !_peers.should_deliver(*pid, m.path)) {
        _peers.note_delivery_filtered(*pid);
        continue;
      }
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

std::vector<state_link_report> state_transport_ws::link_reports() const {
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  const std::int64_t now = steady_ns();
  std::vector<state_link_report> out;
  for (const auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    state_link_report r;
    r.transport = "ws";
    if (const std::string *pid = c->peer_id()) {
      r.remote_node_id = *pid;
      r.remote_device_id = c->remote_device_id;
      r.holds_id = !c->gate || c->gate->holder(*pid) == c.get();
      r.device_verified = c->device_verified.load(std::memory_order_relaxed);
    }
    r.accepted = c->accepted;
    r.age_s = static_cast<double>(now - c->opened_ns) * 1e-9;
    r.idle_s = static_cast<double>(now - c->last_rx_ns.load()) * 1e-9;
    r.stats = c->link.read();
    out.push_back(std::move(r));
  }
  return out;
}

void state_transport_ws::send_pings_if_due() {
  const auto interval = ping_interval();
  if (interval.count() <= 0)
    return;
  const std::int64_t now = steady_ns();
  std::int64_t due = _next_ping_ns.load(std::memory_order_relaxed);
  if (now < due ||
      !_next_ping_ns.compare_exchange_strong(
          due, now + std::chrono::duration_cast<std::chrono::nanoseconds>(interval).count()))
    return;
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    // Skip a connection that cannot send yet, and one whose send buffer is badly backed up (the
    // write would block the prober behind it; its RTT would only measure the backlog anyway).
    if (!c || !c->alive.load() || !c->can_write() || c->send_backlog() > (64u << 10))
      continue;
    pb::Frame f;
    f.mutable_ping()->set_seq(c->link.next_ping_seq());
    f.mutable_ping()->set_sent_ns(steady_ns());
    if (c->try_write(f))
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_ws::ping_loop() {
#ifndef __EMSCRIPTEN__
  while (_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_pings_if_due();
    std::vector<std::shared_ptr<connection>> conns, handshaking;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      conns = _conns;
      handshaking = _handshaking;
    }
    const std::int64_t now = steady_ns();
    // A handshake that has not finished in kHandshakeNs is a client that connected and never sent
    // (or never finished) its upgrade request: close it, which fails its read and ends its thread.
    for (auto &c : handshaking) {
      const std::int64_t started = c->handshake_started_ns.load(std::memory_order_relaxed);
      if (started != 0 && now - started > kHandshakeNs)
        c->abort_io();
    }
    for (auto &c : conns) {
      const std::int64_t started = c ? c->write_started_ns.load(std::memory_order_relaxed) : 0;
      if (started != 0 && now - started > kWriteStallNs && c->alive.load()) {
        const std::string *pid = c->peer_id();
        std::fprintf(stderr, "[ws] write to '%s' blocked > 3 s (peer not reading): closing\n",
                     pid ? pid->c_str() : "");
        c->abort_io();
      }
    }
  }
#endif
}

void state_transport_ws::register_connection(std::shared_ptr<connection> conn) {
  if (conn->opened_ns == 0)
    conn->opened_ns = steady_ns();
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
  // The admission hold: an accepted relay connection gets its bootstrap, and live traffic, only
  // once release_admissions() finds it ready, so the snapshot can go through the peer's rule. It is
  // held before it is visible, and the Hello below goes through write_locked, which the hold does
  // not stop.
  const bool hold =
      relay && conn->accepted && _admission_hold_ns.load(std::memory_order_relaxed) > 0;
  if (hold) {
    conn->admission_held.store(true, std::memory_order_relaxed);
    _admissions_held.fetch_add(1, std::memory_order_relaxed);
  }
  register_connection(conn);
  send_hello_locked(*conn);
  if (hold)
    return;
  if (backfill && !shards.empty())
    send_backfill_locked(*conn, shards, nullptr);
  // A relay pushes its full current state so a late-joining client sees peers already in the room
  // (the live set_relay forwarding only carries writes that happen AFTER this client connects).
  if (relay && !shards.empty())
    send_snapshot_locked(*conn, shards, nullptr);
}

void state_transport_ws::set_admission_hold(std::chrono::milliseconds max_hold) {
  // A day at most, so the nanosecond count cannot overflow (milliseconds::max() means "long").
  const auto capped = (std::min)(max_hold, std::chrono::milliseconds(std::chrono::hours(24)));
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(capped).count();
  _admission_hold_ns.store(ns > 0 ? static_cast<std::int64_t>(ns) : 0, std::memory_order_relaxed);
}

void state_transport_ws::set_inbound_gate(std::shared_ptr<state_inbound_gate> gate) {
  std::lock_guard<std::mutex> lk(_inbound_gate_mu);
  _inbound_gate = std::move(gate);
}

bool state_transport_ws::is_local_id(const std::string &id) const {
  if (id == _node_id)
    return true;
  std::lock_guard<std::mutex> lk(_shards_mu);
  return std::any_of(_shards.begin(), _shards.end(), [&id](const state_cluster_shard *s) {
    return s != nullptr && s->local_node_id() == id;
  });
}

std::size_t state_transport_ws::release_admissions() {
  std::vector<std::shared_ptr<connection>> waiting;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (const auto &c : _conns)
      // A closed one stays listed until the reaper sweeps it: never bootstrap it.
      if (c && c->alive.load() && c->admission_held.load(std::memory_order_relaxed))
        waiting.push_back(c);
  }
  if (waiting.empty())
    return 0;
  std::vector<state_cluster_shard *> shards;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  const std::int64_t hold = _admission_hold_ns.load(std::memory_order_relaxed);
  const std::int64_t now = steady_ns();
  std::size_t n = 0;
  for (const auto &c : waiting) {
    // The rule is read before the write lock is taken (no path holds the registry's lock and a
    // connection's write lock together).
    state_peer_registry::delivery_view view;
    const std::string *pid = c->peer_id();
    if (pid)
      view = _peers.view_of(*pid);
    const bool ruled = view.filtered();
    if (!ruled && hold > 0 && now - c->opened_ns < hold)
      continue; // no rule for the peer yet, and the hold has time left
    std::lock_guard<std::mutex> wlk(c->write_mu);
    if (!release_locked(*c, shards, ruled ? &view : nullptr))
      continue;
    if (!ruled)
      _admission_timeouts.fetch_add(1, std::memory_order_relaxed);
    ++n;
  }
  return n;
}

bool state_transport_ws::release_locked(connection &c,
                                        const std::vector<state_cluster_shard *> &shards,
                                        const state_peer_registry::delivery_view *view) {
  if (!c.admission_held.exchange(false, std::memory_order_relaxed))
    return false;
  if (_backfill_on_connect.load(std::memory_order_acquire) && !shards.empty())
    send_backfill_locked(c, shards, view);
  if (_relay.load(std::memory_order_relaxed) && !shards.empty())
    send_snapshot_locked(c, shards, view);
  return true;
}

void state_transport_ws::send_hello_locked(connection &c) {
  pb::Frame f;
  auto *h = f.mutable_hello();
  h->set_node_id(_node_id);
  h->set_cluster_id(_cluster_id);
  if (!_device_id.empty())
    h->set_device_id(_device_id);
  // A relay with a pin store challenges each client it accepts to prove its device key.
  if (c.accepted && _device_pins && device_keys_available()) {
    std::lock_guard<std::mutex> lk(c.proof_mu);
    c.device_nonce = device_random_bytes(kDeviceNonceBytes);
    h->set_nonce(c.device_nonce);
  }
  c.write_locked(f);
}

void state_transport_ws::answer_device_challenge(connection &c, const std::string &nonce,
                                                 const std::string &relay_node_id) {
  // Sign only a nonce of a sane size: the proof is for one relay's challenge, nothing larger.
  if (!_device_signer || _device_id.empty() || nonce.size() > kDeviceNonceMaxBytes)
    return;
  pb::Frame f;
  auto *p = f.mutable_device_proof();
  p->set_public_key(_device_signer->public_key());
  p->set_signature(
      _device_signer->sign(device_proof_message(nonce, relay_node_id, _node_id, _device_id)));
  if (p->signature().empty())
    return;
  if (c.write(f))
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
}

void state_transport_ws::on_device_proof(connection &c, const std::string &public_key,
                                         const std::string &signature) {
  if (c.accepted && !c.identified.load(std::memory_order_relaxed) && !c.proof_pending) {
    c.proof_pending = true; // checked when the Hello comes (deliver_inbound)
    c.pending_key = public_key;
    c.pending_signature = signature;
    return;
  }
  std::string nonce;
  {
    std::lock_guard<std::mutex> lk(c.proof_mu);
    nonce.swap(c.device_nonce); // one proof per challenge
  }
  const std::string *pid = c.peer_id();
  if (nonce.empty() || pid == nullptr || c.local_id || c.remote_device_id.empty() ||
      !_device_pins) {
    _device_proofs_refused.fetch_add(1, std::memory_order_relaxed); // unasked, or anonymous
    return;
  }
  const std::string message = device_proof_message(nonce, _node_id, *pid, c.remote_device_id);
  if (!verify_device_proof(public_key, message, signature)) {
    _device_proofs_refused.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "cvc: ws device proof from %s did not verify\n", pid->c_str());
    return;
  }
  const state_device_pins::verdict v = _device_pins->check(c.remote_device_id, public_key);
  if (v != state_device_pins::verdict::pinned && v != state_device_pins::verdict::matched) {
    _device_proofs_refused.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr,
                 "cvc: ws device of %s was not pinned (another key, or a full store): unverified\n",
                 pid->c_str());
    return;
  }
  c.device_verified.store(true, std::memory_order_relaxed);
  _devices_verified.fetch_add(1, std::memory_order_relaxed);
}

void state_transport_ws::send_backfill_locked(connection &c,
                                              const std::vector<state_cluster_shard *> &shards,
                                              const state_peer_registry::delivery_view *view) {
  // At admission the peer's Hello has not arrived yet, so send everything and let the receiver
  // drop what does not match its cluster or its interest set, exactly as for live traffic. A
  // release from the admission hold passes the peer's rule as `view`.
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    for (const auto &m : s->replay_local()) {
      if (view != nullptr && !view->deliver(m.path))
        continue;
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
                                              const std::vector<state_cluster_shard *> &shards,
                                              const state_peer_registry::delivery_view *view) {
  // A relay holds the whole room tree (every client's forwarded writes land in its shard), but
  // those entries are REMOTE-origin, so send_backfill_locked's replay_local() yields nothing. Push
  // the full current snapshot instead, as an unsolicited SnapshotResponse (request_id 0), so a late
  // joiner bootstraps the peers already present. The receiver applies each entry via
  // ingest_snapshot_entry (a bulk initial load, no conflict gate). Without the admission hold it
  // goes out before the peer's Hello, so there is no identity to look a registry entry up by and
  // it is unfiltered; with it, release_admissions passes the peer's rule as `view`. What it
  // carried is recorded as the peer's starting point, so the first broadcast is a diff.
  const std::uint64_t round = _broadcast_round.load(std::memory_order_relaxed);
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    const auto snap = s->snapshot();
    if (snap.empty())
      continue;
    pb::Frame f;
    fill_snapshot_response(f.mutable_snapshot_response(), snap, view);
    if (!c.write_locked(f))
      return; // stream died mid-snapshot; the reader will reap it
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
    // Only what the view let through: an entry it withheld must still go out once the rule allows.
    std::lock_guard<std::mutex> hk(c.held_mu);
    for (const auto &e : snap)
      if (view == nullptr || view->deliver(e.path))
        c.held[e.path] = held_entry{entry_digest(e).value(), round};
  }
}

std::size_t state_transport_ws::broadcast_snapshot() {
  // Snapshot each shard ONCE, then bring every live connection up to date with it. A connection is
  // sent only the entries it does not already hold (connection::held): those whose content differs
  // from what its last snapshot frame carried, and those forgotten since because another frame
  // touched them. So a path that changed and went out live is sent once more -- the relay cannot
  // tell whether the live frame was applied -- and an unchanged tree sends nothing. The receivers
  // apply each entry idempotently (ingest_snapshot_entry is a bulk load), which makes this a
  // recovery for a client that missed a live re-fan. Uses each connection's write lock for the
  // comparison and the write together -- do NOT hold one here. Snapshotting the shard reads the
  // tree under its own lock.
  std::vector<state_cluster_shard *> shards;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  struct shard_snapshot {
    std::vector<snapshot_entry> entries;
    std::vector<std::uint64_t> digests; // entry_digest of each entry, shared by every connection
  };
  std::vector<shard_snapshot> snaps;
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    shard_snapshot ss;
    ss.entries = s->snapshot();
    if (ss.entries.empty())
      continue;
    ss.digests.reserve(ss.entries.size());
    for (const auto &e : ss.entries)
      ss.digests.push_back(entry_digest(e).value());
    snaps.push_back(std::move(ss));
  }
  if (snaps.empty())
    return 0;
  const std::uint64_t round = _broadcast_round.fetch_add(1, std::memory_order_relaxed) + 1;
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
  std::size_t wrote = 0, skipped = 0, max_backlog = 0, filtered = 0, entries = 0, refreshed = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    const std::string *pid = c->peer_id();
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
        std::fprintf(stderr, "[ws-bcast] SKIP node='%s' backlog=%zuKB\n", pid ? pid->c_str() : "",
                     bl / 1024);
      continue;
    }
    // One registry lookup per connection; the view then answers every entry without the lock.
    state_peer_registry::delivery_view view;
    if (pid)
      view = _peers.view_of(*pid);
    if (view.filtered())
      ++filtered;
    // Under write_mu from the comparison through the write: a live frame for one of these paths
    // lands wholly before (and is compared against) or wholly after (and forgets) this frame.
    std::lock_guard<std::mutex> wlk(c->write_mu);
    // A connection the admission hold still holds gets its first state from its release.
    if (c->admission_held.load(std::memory_order_relaxed))
      continue;
    std::vector<pb::Frame> frames; // one per shard with anything for this peer
    std::size_t n = 0;
    {
      std::lock_guard<std::mutex> hk(c->held_mu);
      if (c->full_refresh.exchange(false, std::memory_order_relaxed)) {
        c->held.clear();
        ++refreshed;
      }
      for (const auto &ss : snaps) {
        pb::Frame f;
        auto *rsp = f.mutable_snapshot_response();
        rsp->set_request_id(0);
        rsp->set_final(true);
        for (std::size_t i = 0; i < ss.entries.size(); ++i) {
          const snapshot_entry &e = ss.entries[i];
          if (!view.deliver(e.path))
            continue;
          // Never a peer's own value back to it: the peer is its source, and our copy can be
          // older than what it holds (state_inbound_gate::sources).
          if (c->gate && pid && c->gate->sources(*pid, e.path))
            continue;
          // Keyed by path alone, as the receiver applies every entry to every shard it has.
          auto [it, fresh] = c->held.try_emplace(e.path);
          it->second.round = round;
          if (!fresh && it->second.digest == ss.digests[i])
            continue; // the peer holds this already
          it->second.digest = ss.digests[i];
          add_snapshot_entry(rsp, e);
        }
        if (rsp->entries_size() > 0) {
          n += static_cast<std::size_t>(rsp->entries_size());
          frames.push_back(std::move(f));
        }
      }
      // A path this round did not deliver left the tree or the peer's view: drop it, so it is sent
      // again if it comes back.
      for (auto it = c->held.begin(); it != c->held.end();)
        it = it->second.round == round ? std::next(it) : c->held.erase(it);
    }
    bool ok = true;
    for (const auto &f : frames) {
      if (!c->write_locked(f)) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      // The stream died (write_locked marked it dead); hold nothing for it.
      std::lock_guard<std::mutex> hk(c->held_mu);
      c->held.clear();
      continue;
    }
    if (!frames.empty()) {
      ++wrote;
      entries += n;
      _sent_frames.fetch_add(frames.size(), std::memory_order_relaxed);
      _broadcast_entries.fetch_add(n, std::memory_order_relaxed);
    }
  }
  if (dbg)
    std::fprintf(stderr,
                 "[ws-bcast] wrote=%zu skipped=%zu filtered=%zu refreshed=%zu entries=%zu "
                 "maxbacklog=%zuKB conns=%zu shards=%zu\n",
                 wrote, skipped, filtered, refreshed, entries, max_backlog / 1024, conns.size(),
                 snaps.size());
  return wrote;
}

std::size_t state_transport_ws::request_full_refresh() {
  pb::Frame f;
  f.mutable_snapshot_request()->set_cluster_id(_cluster_id);
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  std::size_t n = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    if (c->write(f)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
}

std::size_t state_transport_ws::push_snapshot_to(const std::string &node_id,
                                                 const std::vector<snapshot_entry> &entries) {
  if (node_id.empty() || entries.empty())
    return 0;
  std::vector<std::shared_ptr<connection>> targets;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (const auto &c : _conns) {
      if (!c || !c->alive.load() || !c->can_write())
        continue;
      if (const std::string *pid = c->peer_id(); pid && *pid == node_id)
        targets.push_back(c);
    }
  }
  if (targets.empty())
    return 0; // the peer is on another transport (or gone): nothing to encode
  pb::Frame f;
  fill_snapshot_response(f.mutable_snapshot_response(), entries);
  std::size_t n = 0;
  for (auto &c : targets) {
    // Never block the caller (the relay's main loop) on a peer that is not draining; the caller's
    // next push or the periodic broadcast covers what this one would have carried.
    if (c->send_backlog() > 2 * 1024 * 1024)
      continue;
    // The caller chose these entries, so they need not match the tree: forget the peer's copy of
    // each path, and a broadcast that delivers one sends the tree's value there.
    if (c->write_entries(f, entries)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
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
      const std::string *pid = c->peer_id();
      std::fprintf(stderr, "[ws] reaping idle connection node='%s' idle=%.1fs\n",
                   pid ? pid->c_str() : "", static_cast<double>(now - last) / 1.0e9);
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
  // 3) And the threads of handshakes that ended unadmitted.
  reap_handshakes();
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

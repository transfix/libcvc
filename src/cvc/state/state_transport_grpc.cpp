/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "proto/state_transport.grpc.pb.h"
#include "proto/state_transport.pb.h"

#include <algorithm>
#include <chrono>
#include <cstdio> // the write watchdog's report
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message_bus.h>
#include <cvc/state/state_transport_grpc.h>
#include <functional>
#include <grpcpp/grpcpp.h>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

// Steady-clock nanoseconds: the link probe's clock (a Ping carries it; the Pong echoes it back to
// the same process, so no cross-host clock sync is involved) and the link report's age / idle base.
inline std::int64_t link_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Largest gRPC message we accept/send, in bytes. The gRPC default receive cap is ~4 MiB, too small
// for a large binary message payload (a 1080p RGBA frame is ~8 MiB); match the IPC transport's
// 64 MiB frame cap so both wire paths agree on the ceiling.
constexpr int kMaxGrpcMessageBytes = 64 * 1024 * 1024;

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

// Shared state_payload <-> pb::Payload mapping, used by both Mutation framing
// and the snapshot entry's data() payload.
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

cvc::media_frame decode_media(const pb::MediaFrame &in) {
  cvc::media_frame m;
  m.origin_node_id = in.origin_node_id();
  m.stream_id = in.stream_id();
  m.codec = in.codec();
  m.width = in.width();
  m.height = in.height();
  m.seq = in.seq();
  m.pts_seconds = in.pts_seconds();
  m.sample_rate = in.sample_rate();
  m.channels = in.channels();
  const std::string &p = in.payload();
  m.payload.assign(p.begin(), p.end());
  return m;
}

// Encode `entries` into `rsp`, skipping each one `view` (when given) would not deliver. The single
// SnapshotEntry encoding behind the admit snapshot, the request responder and the targeted push.
// Returns the number of entries encoded.
std::size_t fill_snapshot_response(pb::SnapshotResponse *rsp,
                                   const std::vector<state_transport::snapshot_entry> &entries,
                                   const state_peer_registry::delivery_view *view) {
  std::size_t n = 0;
  for (const auto &e : entries) {
    if (view && !view->deliver(e.path))
      continue;
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
    ++n;
  }
  return n;
}

// The inverse of fill_snapshot_response, for a received SnapshotResponse.
std::vector<state_transport::snapshot_entry> read_snapshot_entries(const pb::SnapshotResponse &sr) {
  std::vector<state_transport::snapshot_entry> entries;
  entries.reserve(sr.entries_size());
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
    entries.push_back(std::move(se));
  }
  return entries;
}

// The registry's verdict on sending `path` to a connection whose peer is `pid`; nullptr (a peer
// that has not identified yet) gets everything. A refusal is counted against the peer.
bool registry_allows(state_peer_registry &peers, const std::string *pid, const std::string &path) {
  if (pid == nullptr || peers.should_deliver(*pid, path))
    return true;
  peers.note_delivery_filtered(*pid);
  return false;
}

} // namespace

// Connection abstracts a single bidirectional stream. Server-side
// streams run inside a gRPC handler thread; client-side streams own
// a reader thread that we manage. Writes go through write() under
// write_mu so that the publish() fan-out and the per-stream Hello
// path serialize on the same lock.
//
// write_locked() exists for admit_connection(), which has to keep
// write_mu held across several frames; write_raw is the underlying
// stream write and must only be called with write_mu held. It is a
// timed mutex so a media frame can give up on a receiver whose stream
// stays busy (the stall drop, state_transport_grpc::write_media).
class state_transport_grpc::connection {
public:
  std::timed_mutex write_mu;
  std::function<bool(const pb::Frame &)> write_raw;
  std::atomic<bool> alive{true};
  // The peer's identity, from the first Hello that names one. Written once, on this connection's
  // reader thread, before `identified` is released; a later Hello is ignored, so neither string
  // changes after that. Any other thread reads them only through peer_id().
  std::string remote_node_id;
  std::string remote_cluster_id;
  std::atomic<bool> identified{false};

  // The peer's node id once its Hello has arrived; nullptr before (an unknown peer, which the
  // registry delivers everything to).
  const std::string *peer_id() const noexcept {
    return identified.load(std::memory_order_acquire) ? &remote_node_id : nullptr;
  }

  // Reader thread only. A Hello without a node id identifies nothing.
  void on_hello(const pb::Hello &h) {
    if (identified.load(std::memory_order_relaxed) || h.node_id().empty())
      return;
    remote_node_id = h.node_id();
    remote_cluster_id = h.cluster_id();
    identified.store(true, std::memory_order_release);
  }

  // Link quality (state_link_stats): traffic both ways + the Ping/Pong round trip. `accepted` marks
  // the server end of the stream; opened/last-rx are steady-clock ns for the report's age / idle.
  state_link_stats link;
  bool accepted = false;
  std::int64_t opened_ns = 0;
  std::atomic<std::int64_t> last_rx_ns{0};
  // Steady-clock ns when the write in progress started; 0 when none is. A peer that stops reading
  // makes a stream Write block on flow control while holding write_mu, so every other writer to it
  // would wait forever; the probe thread's watchdog cancels a stream whose write has been blocked
  // too long (state_transport_grpc::ping_loop). `cancel` cancels the stream; the server handler
  // clears it (under cancel_mu) before its ServerContext goes away.
  std::atomic<std::int64_t> write_started_ns{0};
  std::mutex cancel_mu;
  std::function<void()> cancel;
  // Waiting for its bootstrap (the admission hold): every write fails until release_locked clears
  // it. Set and cleared only under write_mu, so a write that saw it set happened before the release
  // took its snapshot, and so is in it.
  std::atomic<bool> held{false};

  bool can_write() const { return static_cast<bool>(write_raw); }

  bool write_locked(const pb::Frame &f) {
    if (!alive.load() || !write_raw || held.load(std::memory_order_relaxed))
      return false;
    const std::size_t bytes = f.ByteSizeLong();
    write_started_ns.store(link_now_ns(), std::memory_order_relaxed);
    const bool ok = write_raw(f);
    write_started_ns.store(0, std::memory_order_relaxed);
    if (!ok)
      return false;
    link.note_tx(bytes);
    return true;
  }

  // Write only if no other write holds the stream right now (the link probe never waits).
  bool try_write(const pb::Frame &f) {
    std::unique_lock<std::timed_mutex> lk(write_mu, std::try_to_lock);
    return lk.owns_lock() && write_locked(f);
  }

  // Cancel the stream: a blocked Write fails and frees write_mu (the write watchdog).
  void abort_io() {
    alive.store(false);
    std::lock_guard<std::mutex> lk(cancel_mu);
    if (cancel)
      cancel();
  }

  bool write(const pb::Frame &f) {
    std::lock_guard<std::timed_mutex> lk(write_mu);
    return write_locked(f);
  }

  // Client-side only.
  std::shared_ptr<grpc::ClientContext> client_ctx;
  std::shared_ptr<grpc::ClientReaderWriter<pb::Frame, pb::Frame>> client_stream;
  std::unique_ptr<pb::StateTransport::Stub> client_stub;
  std::shared_ptr<grpc::Channel> client_channel;
  std::thread client_reader;
};

namespace {

class StateTransportServiceImpl final : public pb::StateTransport::Service {
public:
  explicit StateTransportServiceImpl(state_transport_grpc *owner) : _owner(owner) {}

  grpc::Status Channel(grpc::ServerContext *ctx,
                       grpc::ServerReaderWriter<pb::Frame, pb::Frame> *stream) override {
    // Phase 5: bearer-token check.
    const std::string &expected = _owner->auth().expected_token;
    if (!expected.empty()) {
      const auto &md = ctx->client_metadata();
      auto it = md.find("authorization");
      bool ok = false;
      if (it != md.end()) {
        std::string got(it->second.data(), it->second.size());
        std::string want = "Bearer " + expected;
        if (got == want)
          ok = true;
      }
      if (!ok)
        return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "missing or invalid bearer token");
    }

    auto conn = std::make_shared<state_transport_grpc::connection>();
    conn->accepted = true;
    conn->opened_ns = link_now_ns();
    conn->last_rx_ns.store(conn->opened_ns);
    conn->cancel = [ctx] { ctx->TryCancel(); };
    auto *raw_stream = stream;
    conn->write_raw = [raw_stream](const pb::Frame &f) -> bool { return raw_stream->Write(f); };

    // Registers the connection and sends Hello followed by the
    // journal backfill, all under this connection's write mutex.
    _owner->admit_connection(conn);

    pb::Frame in;
    while (!ctx->IsCancelled() && stream->Read(&in)) {
      _owner->increment_recv_frames();
      if (_owner->on_link_frame(*conn, in))
        continue;
      if (in.has_hello()) {
        conn->on_hello(in.hello());
      } else if (in.has_mutation()) {
        _owner->on_inbound_mutation(decode_mutation(in.mutation()), conn.get());
        _owner->increment_recv_mutations();
      } else if (in.has_message()) {
        _owner->on_inbound_message(decode_message(in.message()), conn.get());
        _owner->increment_recv_messages();
      } else if (in.has_media()) {
        _owner->on_inbound_media(decode_media(in.media()), conn.get());
      } else if (in.has_chunk_request()) {
        _owner->on_inbound_chunk_request(conn.get(), in.chunk_request().digest(),
                                         in.chunk_request().request_id());
      } else if (in.has_chunk_response()) {
        const auto &cr = in.chunk_response();
        std::string data_str = cr.data();
        std::vector<unsigned char> data_vec(data_str.begin(), data_str.end());
        _owner->on_inbound_chunk_response(cr.request_id(), cr.found(), std::move(data_vec));
      } else if (in.has_snapshot_request()) {
        _owner->on_inbound_snapshot_request(conn.get(), in.snapshot_request().cluster_id(),
                                            in.snapshot_request().path_prefix(),
                                            in.snapshot_request().request_id());
      } else if (in.has_snapshot_response()) {
        const auto &sr = in.snapshot_response();
        _owner->on_inbound_snapshot_response(sr.request_id(), read_snapshot_entries(sr),
                                             sr.final());
      } else if (in.has_heartbeat()) {
        _owner->on_inbound_heartbeat(in.heartbeat().node_id(), in.heartbeat().cluster_id());
      }
    }

    conn->alive.store(false);
    {
      std::lock_guard<std::mutex> lk(conn->cancel_mu);
      conn->cancel = nullptr; // ctx dies with this handler
    }
    // So does STREAM, which other threads write to (publish, the relay's forwards, the link
    // probe): fail any Write still blocked on it, then wait out the one in progress. Every write
    // runs under write_mu and checks `alive` there, so none starts after this.
    ctx->TryCancel();
    { std::lock_guard<std::timed_mutex> wl(conn->write_mu); }
    _owner->unregister_connection(conn.get());
    const std::string *pid = conn->peer_id();
    _owner->notify_connection_closed(pid ? *pid : std::string());
    return grpc::Status::OK;
  }

private:
  state_transport_grpc *_owner;
};

} // namespace

struct state_transport_grpc::impl {
  std::string listen_addr_resolved;
  std::unique_ptr<StateTransportServiceImpl> service;
  std::unique_ptr<grpc::Server> server;
};

state_transport_grpc::state_transport_grpc() : _impl(new impl()) {}

state_transport_grpc::~state_transport_grpc() { stop(); }

void state_transport_grpc::set_tls_config(tls_config cfg) {
  if (_running.load())
    throw std::runtime_error("state_transport_grpc::set_tls_config: already running");
  _tls = std::move(cfg);
  _tls_set = true;
}

void state_transport_grpc::set_auth_config(auth_config cfg) {
  if (_running.load())
    throw std::runtime_error("state_transport_grpc::set_auth_config: already running");
  _auth = std::move(cfg);
}

void state_transport_grpc::start(const std::string &listen_addr, const std::string &node_id,
                                 const std::string &cluster_id) {
  if (_running.load())
    throw std::runtime_error("state_transport_grpc::start: already running");

  _node_id = node_id;
  _cluster_id = cluster_id;

  _impl->service.reset(new StateTransportServiceImpl(this));

  grpc::ServerBuilder builder;
  int bound_port = 0;
  std::shared_ptr<grpc::ServerCredentials> server_creds;
  if (_tls_set && !_tls.server_cert_pem.empty() && !_tls.server_key_pem.empty()) {
    grpc::SslServerCredentialsOptions opts(
        _tls.require_client_auth ? GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY
                                 : GRPC_SSL_DONT_REQUEST_CLIENT_CERTIFICATE);
    grpc::SslServerCredentialsOptions::PemKeyCertPair pkcp{_tls.server_key_pem,
                                                           _tls.server_cert_pem};
    opts.pem_key_cert_pairs.push_back(pkcp);
    if (!_tls.root_ca_pem.empty())
      opts.pem_root_certs = _tls.root_ca_pem;
    server_creds = grpc::SslServerCredentials(opts);
  } else {
    server_creds = grpc::InsecureServerCredentials();
  }
  // Raise the message-size limits from the ~4 MiB gRPC default so large binary payloads round-trip
  // (see kMaxGrpcMessageBytes); the client channel below sets the matching caps.
  builder.SetMaxReceiveMessageSize(kMaxGrpcMessageBytes);
  builder.SetMaxSendMessageSize(kMaxGrpcMessageBytes);
  builder.AddListeningPort(listen_addr, server_creds, &bound_port);
  builder.RegisterService(_impl->service.get());
  _impl->server = builder.BuildAndStart();
  if (!_impl->server)
    throw std::runtime_error("state_transport_grpc::start: BuildAndStart failed");

  auto colon = listen_addr.find_last_of(':');
  std::string host = (colon == std::string::npos) ? listen_addr : listen_addr.substr(0, colon);
  if (bound_port > 0)
    _impl->listen_addr_resolved = host + ":" + std::to_string(bound_port);
  else
    _impl->listen_addr_resolved = listen_addr;

  _running.store(true);

  // Start heartbeat sender if configured.
  if (_heartbeat_interval.count() > 0) {
    _heartbeat_thread = std::thread([this]() { heartbeat_loop(); });
  }
  _ping_thread = std::thread([this]() { ping_loop(); });
}

void state_transport_grpc::set_heartbeat_interval(std::chrono::milliseconds interval) noexcept {
  _heartbeat_interval = interval;
}

std::chrono::milliseconds state_transport_grpc::heartbeat_interval() const noexcept {
  return _heartbeat_interval;
}

std::string state_transport_grpc::listen_address() const {
  return _impl ? _impl->listen_addr_resolved : std::string();
}

bool state_transport_grpc::connect_to_peer(const std::string &target,
                                           std::chrono::milliseconds timeout) {
  if (!_running.load())
    return false;

  std::shared_ptr<grpc::ChannelCredentials> chan_creds;
  if (_tls_set && !_tls.root_ca_pem.empty()) {
    grpc::SslCredentialsOptions opts;
    opts.pem_root_certs = _tls.root_ca_pem;
    if (!_tls.server_cert_pem.empty() && !_tls.server_key_pem.empty() && _tls.require_client_auth) {
      opts.pem_cert_chain = _tls.server_cert_pem;
      opts.pem_private_key = _tls.server_key_pem;
    }
    chan_creds = grpc::SslCredentials(opts);
  } else {
    chan_creds = grpc::InsecureChannelCredentials();
  }
  grpc::ChannelArguments chan_args;
  chan_args.SetMaxReceiveMessageSize(kMaxGrpcMessageBytes);
  chan_args.SetMaxSendMessageSize(kMaxGrpcMessageBytes);
  auto channel = grpc::CreateCustomChannel(target, chan_creds, chan_args);
  auto deadline = std::chrono::system_clock::now() + timeout;
  if (!channel->WaitForConnected(deadline))
    return false;

  auto stub = pb::StateTransport::NewStub(channel);
  auto ctx = std::make_shared<grpc::ClientContext>();
  if (!_auth.outbound_token.empty())
    ctx->AddMetadata("authorization", "Bearer " + _auth.outbound_token);
  std::shared_ptr<grpc::ClientReaderWriter<pb::Frame, pb::Frame>> stream(
      stub->Channel(ctx.get()).release());
  if (!stream)
    return false;

  auto conn = std::make_shared<connection>();
  conn->client_ctx = ctx;
  conn->client_stream = stream;
  conn->client_stub = std::move(stub);
  conn->client_channel = channel;

  conn->write_raw = [stream](const pb::Frame &f) -> bool { return stream->Write(f); };

  // Spawn the reader before publishing the connection. admit_connection()
  // makes it visible to stop(), which joins client_reader — so the thread
  // has to already be assigned or stop() could miss it and leave a
  // joinable std::thread to destruct.
  conn->opened_ns = link_now_ns();
  conn->last_rx_ns.store(conn->opened_ns);
  conn->cancel = [ctx = conn->client_ctx] {
    if (ctx)
      ctx->TryCancel();
  };
  conn->client_reader = std::thread([this, conn]() {
    pb::Frame in;
    while (conn->alive.load() && conn->client_stream->Read(&in)) {
      increment_recv_frames();
      if (on_link_frame(*conn, in))
        continue;
      if (in.has_hello()) {
        conn->on_hello(in.hello());
      } else if (in.has_mutation()) {
        on_inbound_mutation(decode_mutation(in.mutation()), conn.get());
        increment_recv_mutations();
      } else if (in.has_message()) {
        on_inbound_message(decode_message(in.message()), conn.get());
        increment_recv_messages();
      } else if (in.has_media()) {
        on_inbound_media(decode_media(in.media()), conn.get());
      } else if (in.has_chunk_request()) {
        on_inbound_chunk_request(conn.get(), in.chunk_request().digest(),
                                 in.chunk_request().request_id());
      } else if (in.has_chunk_response()) {
        const auto &cr = in.chunk_response();
        std::string data_str = cr.data();
        std::vector<unsigned char> data_vec(data_str.begin(), data_str.end());
        on_inbound_chunk_response(cr.request_id(), cr.found(), std::move(data_vec));
      } else if (in.has_snapshot_request()) {
        on_inbound_snapshot_request(conn.get(), in.snapshot_request().cluster_id(),
                                    in.snapshot_request().path_prefix(),
                                    in.snapshot_request().request_id());
      } else if (in.has_snapshot_response()) {
        const auto &sr = in.snapshot_response();
        on_inbound_snapshot_response(sr.request_id(), read_snapshot_entries(sr), sr.final());
      } else if (in.has_heartbeat()) {
        on_inbound_heartbeat(in.heartbeat().node_id(), in.heartbeat().cluster_id());
      }
    }
    conn->alive.store(false);
  });

  // Registers the connection and sends Hello followed by the journal
  // backfill, all under this connection's write mutex.
  admit_connection(conn);
  return true;
}

void state_transport_grpc::notify_connection_closed(const std::string &node_id) {
  // stop() clears _running before it cancels the streams, so a shutdown never reports departures.
  if (_running.load() && _on_conn_closed && !node_id.empty())
    _on_conn_closed(node_id);
}

void state_transport_grpc::stop() {
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
    if (c->client_ctx)
      c->client_ctx->TryCancel();
  }

  if (_impl && _impl->server) {
    _impl->server->Shutdown(std::chrono::system_clock::now());
    _impl->server->Wait();
    _impl->server.reset();
  }

  if (_heartbeat_thread.joinable())
    _heartbeat_thread.join();
  if (_ping_thread.joinable())
    _ping_thread.join();

  for (auto &c : conns) {
    if (c->client_reader.joinable())
      c->client_reader.join();
  }

  if (_impl)
    _impl->service.reset();
}

void state_transport_grpc::register_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  if (std::find(_shards.begin(), _shards.end(), shard) == _shards.end())
    _shards.push_back(shard);
}

void state_transport_grpc::unregister_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  _shards.erase(std::remove(_shards.begin(), _shards.end(), shard), _shards.end());
}

state_transport::publish_stats state_transport_grpc::publish(const state_mutation &m) {
  publish_stats stats{};

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
    if (!registry_allows(_peers, pid, m.path))
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.delivered;
      if (pid)
        _peers.note_mutation_delivered(*pid);
    }
  }

  _published.fetch_add(1, std::memory_order_relaxed);
  return stats;
}

std::size_t state_transport_grpc::pump_shard(state_cluster_shard &shard) {
  auto pending = shard.drain_local();
  for (const auto &m : pending)
    publish(m);
  return pending.size();
}

std::size_t state_transport_grpc::pump_all() {
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

void state_transport_grpc::flush() {
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c)
      continue;
    std::lock_guard<std::timed_mutex> wlk(c->write_mu);
    (void)wlk;
  }
}

state_transport::publish_message_stats
state_transport_grpc::publish_message(const state_message &m) {
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
    } else
      ++stats.duplicates;
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
    if (!registry_allows(_peers, pid, m.path))
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.peers;
      if (pid)
        _peers.note_message_delivered(*pid);
    }
  }
  return stats;
}

void state_transport_grpc::on_inbound_mutation(const state_mutation &m, connection *origin) {
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
  const bool relay = _relay.load(std::memory_order_relaxed);
  if (!relay && !_bridge)
    return;
  // The interest gate withholds the frame from the relay fan AND the bridge alike.
  if (!relay_admits(m, peers)) {
    _relay_withheld.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // Relay (star topology): re-send this mutation to every OTHER connection so clients that only
  // reach this server still converge with each other. The origin connection is skipped (no echo);
  // origin_node_id is preserved so duplicate suppression / vector clocks still work end-to-end.
  // Each target's registry entry applies, as it does to publish().
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
      if (!registry_allows(_peers, c->peer_id(), m.path))
        continue;
      if (c->write(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Bridge to a sibling transport (e.g. the relay's WebSocket side) so its connections get this
  // mutation — the WS<->gRPC bridge that puts browser (wasm) and native clients in one room. The
  // sibling's publish() applies its own registry.
  if (_bridge)
    _bridge->publish(m);
}

void state_transport_grpc::on_inbound_message(const state_message &m, connection *origin) {
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
  if (!relay && !_bridge)
    return;
  if (!relay_admits(m, peers)) {
    _relay_withheld.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // Relay (star topology): forward the message to every OTHER connection (see on_inbound_mutation).
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
      if (!registry_allows(_peers, c->peer_id(), m.path))
        continue;
      if (c->write(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Bridge to a sibling transport (the relay's WebSocket side) so its connections get this message.
  if (_bridge)
    _bridge->publish_message(m);
}

bool state_transport_grpc::relay_admits(const state_mutation &m,
                                        const std::vector<state_cluster_shard *> &shards) const {
  if (!_relay_requires_interest.load(std::memory_order_relaxed) || shards.empty())
    return true;
  return std::any_of(shards.begin(), shards.end(),
                     [&m](const state_cluster_shard *s) { return s->admits(m); });
}

bool state_transport_grpc::relay_admits(const state_message &m,
                                        const std::vector<state_cluster_shard *> &shards) const {
  if (!_relay_requires_interest.load(std::memory_order_relaxed) || shards.empty())
    return true;
  return std::any_of(shards.begin(), shards.end(),
                     [&m](const state_cluster_shard *s) { return s->admits_message(m); });
}

std::size_t state_transport_grpc::write_media(const media_frame &m, const void *exclude,
                                              bool gated) {
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
  // One copy of the gate for the whole frame: a concurrent set_media_gate swaps the slot, never
  // the function this loop is calling.
  std::shared_ptr<const media_gate_fn> gate;
  if (gated) {
    std::lock_guard<std::mutex> lk(_media_gate_mu);
    gate = _media_gate;
  }
  const std::uint64_t stall_ns = _media_stall_ns.load(std::memory_order_relaxed);
  // Media is unreplicated, un-bridged (it crosses protocols via the media_router), and carries no
  // path. `exclude` (a connection*) skips the origin peer when a relay fans; nullptr sends to all.
  std::size_t n = 0;
  for (auto &c : conns) {
    // A held connection (the admission hold) gets no media yet; skipping it here, unlocked, also
    // keeps the gate from spending a thumbnail on it.
    if (!c || c.get() == exclude || !c->alive.load() || !c->can_write() ||
        c->held.load(std::memory_order_relaxed))
      continue;
    std::unique_lock<std::timed_mutex> lk(c->write_mu, std::defer_lock);
    if (stall_ns != 0) {
      // A write that has already run past the window: drop at once. Otherwise wait for the stream
      // at most the window, so this (publisher's reader) thread never queues behind a slow
      // receiver for longer -- a write that starts and then blocks on flow control would hold
      // every frame behind it until the 3 s watchdog.
      const auto started =
          static_cast<std::uint64_t>(c->write_started_ns.load(std::memory_order_relaxed));
      if ((started != 0 &&
           media_write_stalled(started, static_cast<std::uint64_t>(link_now_ns()), stall_ns)) ||
          !lk.try_lock_for(std::chrono::nanoseconds(stall_ns))) {
        _media_stall_dropped.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
    } else {
      lk.lock();
    }
    // The gate once the stream is ours, so a frame the stall drop discards never uses up a
    // thumbnail slot.
    if (gate) {
      const std::string *pid = c->peer_id();
      if (pid && !(*gate)(m, *pid)) {
        _media_gated.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
    }
    if (c->write_locked(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
}

void state_transport_grpc::publish_media_frame(const media_frame &m) {
  _media_sent.fetch_add(write_media(m, nullptr, /*gated=*/false), std::memory_order_relaxed);
}

void state_transport_grpc::fan_media(const media_frame &f, const void *exclude) {
  _media_fwd.fetch_add(write_media(f, exclude, /*gated=*/true), std::memory_order_relaxed);
}

void state_transport_grpc::set_media_gate(media_gate_fn gate) {
  std::shared_ptr<const media_gate_fn> next;
  if (gate)
    next = std::make_shared<const media_gate_fn>(std::move(gate));
  {
    std::lock_guard<std::mutex> lk(_media_gate_mu);
    _media_gate.swap(next);
  }
  // `next` now holds the previous gate; it is released here, outside the lock (a reader thread
  // mid-frame keeps its own copy alive).
}

void state_transport_grpc::set_media_stall_drop(std::chrono::milliseconds ms) noexcept {
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ms).count();
  _media_stall_ns.store(ns > 0 ? static_cast<std::uint64_t>(ns) : 0, std::memory_order_relaxed);
}

bool state_transport_grpc::media_write_stalled(std::uint64_t write_started_ns, std::uint64_t now_ns,
                                               std::uint64_t stall_ns) noexcept {
  return stall_ns != 0 && write_started_ns != 0 && now_ns > write_started_ns &&
         now_ns - write_started_ns > stall_ns;
}

void state_transport_grpc::set_media_sink(media_sink_fn sink) { _media_sink = std::move(sink); }

void state_transport_grpc::set_on_media(media_frame_fn cb) {
  if (!cb) {
    _media_sink = {};
    return;
  }
  _media_sink = [cb = std::move(cb)](const media_frame &f, const void * /*src_conn*/) { cb(f); };
}

void state_transport_grpc::on_inbound_media(const media_frame &m, connection *origin) {
  _media_recv.fetch_add(1, std::memory_order_relaxed);
  if (_media_sink)
    _media_sink(m, /*src_conn=*/origin);
}

void state_transport_grpc::register_connection(std::shared_ptr<connection> conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.push_back(std::move(conn));
}

void state_transport_grpc::admit_connection(const std::shared_ptr<connection> &conn) {
  // Snapshot the shard list before taking the write mutex: every
  // other path releases _shards_mu before touching a connection's
  // write mutex, and keeping that order here avoids introducing a
  // lock cycle.
  const bool backfill = _backfill_on_connect.load(std::memory_order_acquire);
  const bool relay = _relay.load(std::memory_order_relaxed);
  std::vector<state_cluster_shard *> shards;
  if (backfill || relay) {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }

  // Hold the write mutex across publication of the connection so a
  // concurrent publish() blocks until Hello and the backfill are on
  // the wire. If a live mutation went out first, the older values
  // replayed behind it would land on the peer afterwards and clobber
  // the newer one (the seen-set only suppresses exact repeats).
  std::lock_guard<std::timed_mutex> wlk(conn->write_mu);
  register_connection(conn);
  send_hello_locked(*conn);
  // The admission hold: an accepted relay stream gets its bootstrap, and live traffic, only once
  // release_admissions() finds it ready, so the snapshot can go through the peer's delivery rule.
  if (relay && conn->accepted && _admission_hold_ns.load(std::memory_order_relaxed) > 0) {
    conn->held.store(true, std::memory_order_relaxed);
    _admissions_held.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (backfill && !shards.empty())
    send_backfill_locked(*conn, shards, nullptr);
  // A relay pushes its full current state so a LATE-joining client sees peers already in the room
  // (the live set_relay forwarding only carries writes that happen AFTER this client connects, and
  // the relay's room tree is all remote-origin so backfill's replay_local yields nothing). Mirrors
  // the WS relay's send_snapshot_locked — without it a native gRPC client joining a populated room
  // sees no one.
  if (relay && !shards.empty())
    send_snapshot_locked(*conn, shards, nullptr);
}

void state_transport_grpc::set_admission_hold(std::chrono::milliseconds max_hold) {
  // A day at most, so the nanosecond count cannot overflow (milliseconds::max() means "long").
  const auto capped = (std::min)(max_hold, std::chrono::milliseconds(std::chrono::hours(24)));
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(capped).count();
  _admission_hold_ns.store(ns > 0 ? static_cast<std::int64_t>(ns) : 0, std::memory_order_relaxed);
}

std::size_t state_transport_grpc::release_admissions() {
  std::vector<std::shared_ptr<connection>> held;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (const auto &c : _conns)
      if (c && c->alive.load() && c->held.load(std::memory_order_relaxed))
        held.push_back(c);
  }
  if (held.empty())
    return 0;
  std::vector<state_cluster_shard *> shards;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    shards = _shards;
  }
  const std::int64_t hold = _admission_hold_ns.load(std::memory_order_relaxed);
  const std::int64_t now = link_now_ns();
  std::size_t n = 0;
  for (const auto &c : held) {
    // The rule is read before the write lock is taken (no path holds the registry's lock and a
    // connection's write lock together).
    state_peer_registry::delivery_view view;
    const std::string *pid = c->peer_id();
    if (pid)
      view = _peers.view_of(*pid);
    const bool ruled = view.filtered();
    if (!ruled && hold > 0 && now - c->opened_ns < hold)
      continue; // no rule for the peer yet, and the hold has time left
    std::lock_guard<std::timed_mutex> wlk(c->write_mu);
    if (!release_locked(*c, shards, ruled ? &view : nullptr))
      continue;
    if (!ruled)
      _admission_timeouts.fetch_add(1, std::memory_order_relaxed);
    ++n;
  }
  return n;
}

bool state_transport_grpc::release_locked(connection &c,
                                          const std::vector<state_cluster_shard *> &shards,
                                          const state_peer_registry::delivery_view *view) {
  if (!c.held.exchange(false, std::memory_order_relaxed))
    return false;
  if (_backfill_on_connect.load(std::memory_order_acquire) && !shards.empty())
    send_backfill_locked(c, shards, view);
  if (_relay.load(std::memory_order_relaxed) && !shards.empty())
    send_snapshot_locked(c, shards, view);
  return true;
}

void state_transport_grpc::send_hello_locked(connection &c) {
  pb::Frame f;
  auto *h = f.mutable_hello();
  h->set_node_id(_node_id);
  h->set_cluster_id(_cluster_id);
  c.write_locked(f);
}

void state_transport_grpc::send_backfill_locked(connection &c,
                                                const std::vector<state_cluster_shard *> &shards,
                                                const state_peer_registry::delivery_view *view) {
  // At admission the peer's Hello has not arrived yet, so there is no
  // remote node id to filter on; send everything and let the receiver
  // drop what does not match its cluster (on_inbound_mutation) or its
  // interest set (ingest_remote), exactly as it would for live traffic.
  // A release from the admission hold passes the peer's rule as `view`.
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    for (const auto &m : s->replay_local()) {
      if (view != nullptr && !view->deliver(m.path))
        continue;
      pb::Frame f;
      encode_mutation(m, f.mutable_mutation());
      if (!c.write_locked(f))
        return; // Stream died mid-backfill; the reader will reap it.
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      _backfilled.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void state_transport_grpc::send_snapshot_locked(connection &c,
                                                const std::vector<state_cluster_shard *> &shards,
                                                const state_peer_registry::delivery_view *view) {
  // Relay-only: push the full CURRENT snapshot of each shard as an unsolicited SnapshotResponse
  // (request_id 0), so a late joiner bootstraps the peers already present. replay_local() cannot do
  // this on a relay because the room tree it holds is all remote-origin (forwarded from clients).
  // The receiver applies each entry via on_inbound_snapshot_response's no-waiter path
  // (ingest_snapshot_ entry, a bulk initial load). Mirrors
  // state_transport_ws::send_snapshot_locked.
  //
  // Without the admission hold this goes out before the peer's Hello, so there is no identity to
  // look up in the registry and it is unfiltered: the receiver keeps only what its own interest
  // takes. With it, release_admissions passes the peer's rule as `view`.
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
    fill_snapshot_response(rsp, snap, view);
    if (!c.write_locked(f))
      return; // stream died mid-snapshot; the reader will reap it
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_grpc::unregister_connection(connection *conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.erase(
      std::remove_if(_conns.begin(), _conns.end(),
                     [conn](const std::shared_ptr<connection> &c) { return c.get() == conn; }),
      _conns.end());
}

std::size_t state_transport_grpc::shard_count() const {
  std::lock_guard<std::mutex> lk(_shards_mu);
  return _shards.size();
}

std::vector<state_link_report> state_transport_grpc::link_reports() const {
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  const std::int64_t now = link_now_ns();
  std::vector<state_link_report> out;
  for (const auto &c : conns) {
    if (!c || !c->alive.load())
      continue;
    state_link_report r;
    r.transport = "grpc";
    if (const std::string *pid = c->peer_id())
      r.remote_node_id = *pid;
    r.accepted = c->accepted;
    r.age_s = static_cast<double>(now - c->opened_ns) * 1e-9;
    r.idle_s = static_cast<double>(now - c->last_rx_ns.load()) * 1e-9;
    r.stats = c->link.read();
    out.push_back(std::move(r));
  }
  return out;
}

bool state_transport_grpc::on_link_frame(connection &c, const pb::Frame &in) {
  c.link.note_rx(in.ByteSizeLong());
  c.last_rx_ns.store(link_now_ns(), std::memory_order_relaxed);
  if (in.has_ping()) {
    pb::Frame f;
    f.mutable_pong()->set_seq(in.ping().seq());
    f.mutable_pong()->set_sent_ns(in.ping().sent_ns());
    if (c.write(f))
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
    return true;
  }
  if (in.has_pong()) {
    c.link.note_rtt(link_now_ns() - in.pong().sent_ns());
    return true;
  }
  return false;
}

void state_transport_grpc::send_pings() {
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    pb::Frame f;
    f.mutable_ping()->set_seq(c->link.next_ping_seq());
    f.mutable_ping()->set_sent_ns(link_now_ns());
    if (c->try_write(f))
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_grpc::ping_loop() {
  // How long one write may block before its stream is cancelled (connection::write_started_ns).
  constexpr std::int64_t kWriteStallNs = 3'000'000'000;
  auto next = std::chrono::steady_clock::now();
  while (_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::vector<std::shared_ptr<connection>> conns;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      conns = _conns;
    }
    const std::int64_t now = link_now_ns();
    for (auto &c : conns) {
      const std::int64_t started = c ? c->write_started_ns.load(std::memory_order_relaxed) : 0;
      if (started != 0 && now - started > kWriteStallNs && c->alive.load()) {
        const std::string *pid = c->peer_id();
        std::fprintf(stderr, "[grpc] write to '%s' blocked > 3 s (peer not reading): closing\n",
                     pid ? pid->c_str() : "");
        c->abort_io();
      }
    }
    const auto interval = ping_interval();
    if (interval.count() <= 0 || std::chrono::steady_clock::now() < next)
      continue;
    next = std::chrono::steady_clock::now() + interval;
    send_pings();
  }
}

std::size_t state_transport_grpc::connection_count() const {
  std::lock_guard<std::mutex> lk(_conns_mu);
  std::size_t n = 0;
  for (auto &c : _conns)
    if (c && c->alive.load())
      ++n;
  return n;
}

std::uint64_t state_transport_grpc::wait_for_received(std::uint64_t target,
                                                      std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (_recv_mutations.load() < target) {
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return _recv_mutations.load();
}

std::uint64_t state_transport_grpc::wait_for_received_messages(std::uint64_t target,
                                                               std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (_recv_messages.load() < target) {
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return _recv_messages.load();
}

void state_transport_grpc::on_inbound_chunk_request(connection *conn, const std::string &digest,
                                                    std::uint64_t request_id) {
  pb::Frame f;
  auto *rsp = f.mutable_chunk_response();
  rsp->set_digest(digest);
  rsp->set_request_id(request_id);
  if (_blob_store) {
    std::vector<unsigned char> bytes;
    if (_blob_store->get(digest, bytes)) {
      rsp->set_found(true);
      rsp->set_data(std::string(bytes.begin(), bytes.end()));
    } else {
      rsp->set_found(false);
    }
  } else {
    rsp->set_found(false);
  }
  if (conn && conn->alive.load() && conn->can_write()) {
    conn->write(f);
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_grpc::on_inbound_chunk_response(std::uint64_t request_id, bool found,
                                                     std::vector<unsigned char> data) {
  {
    std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
    auto it = _chunk_waiters.find(request_id);
    if (it != _chunk_waiters.end()) {
      it->second->found = found;
      it->second->data = std::move(data);
      it->second->done = true;
    }
  }
  _chunk_waiters_cv.notify_all();
}

bool state_transport_grpc::fetch_chunk(const std::string &digest, chunk_callback on_chunk) {
  if (digest.empty())
    return false;

  // Try local blob store first.
  if (_blob_store) {
    std::vector<unsigned char> bytes;
    if (_blob_store->get(digest, bytes)) {
      if (on_chunk)
        on_chunk(digest, bytes);
      return true;
    }
  }

  // Ask connected peers.
  std::uint64_t req_id = _next_chunk_req_id.fetch_add(1, std::memory_order_relaxed);
  auto waiter = std::make_shared<chunk_waiter>();
  {
    std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters[req_id] = waiter;
  }

  pb::Frame frame;
  auto *req = frame.mutable_chunk_request();
  req->set_digest(digest);
  req->set_request_id(req_id);

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  std::size_t sent = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++sent;
    }
  }

  bool result = false;
  if (sent > 0) {
    std::unique_lock<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters_cv.wait_for(lk, std::chrono::seconds(10), [&]() { return waiter->done; });
    if (waiter->done && waiter->found) {
      if (on_chunk)
        on_chunk(digest, waiter->data);
      result = true;
    }
  }

  {
    std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters.erase(req_id);
  }
  return result;
}

void state_transport_grpc::on_inbound_snapshot_request(connection *conn,
                                                       const std::string &cluster_id,
                                                       const std::string &path_prefix,
                                                       std::uint64_t request_id) {
  // Gather entries from all local shards matching the cluster.
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    for (auto *s : _shards)
      if (s && s->cluster_id() == cluster_id)
        peers.push_back(s);
  }

  std::vector<snapshot_entry> all;
  for (auto *peer : peers) {
    auto snap = peer->snapshot(path_prefix);
    all.insert(all.end(), std::make_move_iterator(snap.begin()),
               std::make_move_iterator(snap.end()));
  }

  // The requester sees what its registry entry lets its live traffic see (a delivery policy, not
  // access control). A requester that has not identified gets everything.
  state_peer_registry::delivery_view view;
  if (conn != nullptr)
    if (const std::string *pid = conn->peer_id())
      view = _peers.view_of(*pid);

  // Build response frame.
  pb::Frame f;
  auto *rsp = f.mutable_snapshot_response();
  rsp->set_request_id(request_id);
  rsp->set_final(true);
  fill_snapshot_response(rsp, all, view.filtered() ? &view : nullptr);

  if (conn && conn->alive.load() && conn->can_write()) {
    conn->write(f);
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_grpc::on_inbound_snapshot_response(std::uint64_t request_id,
                                                        const std::vector<snapshot_entry> &entries,
                                                        bool final) {
  bool matched = false;
  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    auto it = _snap_waiters.find(request_id);
    if (it != _snap_waiters.end()) {
      matched = true;
      for (const auto &e : entries)
        it->second->entries.push_back(e);
      if (final)
        it->second->done = true;
    }
  }
  if (final)
    _snap_waiters_cv.notify_all();
  // An UNSOLICITED snapshot (no pending request — a relay's request_id 0 push to a late joiner, see
  // send_snapshot_locked) is applied to the local shards as an initial-sync bootstrap, mirroring
  // the WS relay path, so a native client joining a populated room sees the peers already present.
  // Applied defensively: a relay snapshot carries its whole shard, including per-client read-only
  // ariadne ui.docs.* state the joiner already owns, which the local tree is EXPECTED to reject.
  // A shard that enforces interest takes from a snapshot only what it would take live (a no-op
  // for a shard that does not enforce).
  if (!matched && !entries.empty()) {
    std::vector<state_cluster_shard *> shards;
    {
      std::lock_guard<std::mutex> lk(_shards_mu);
      shards = _shards;
    }
    for (const auto &e : entries)
      for (auto *s : shards) {
        if (s == nullptr || !s->path_is_of_interest(e.path))
          continue;
        try {
          s->ingest_snapshot_entry(e);
        } catch (const std::exception &ex) {
          if (std::string(ex.what()).find("read-only") == std::string::npos)
            std::fprintf(stderr, "cvc: grpc snapshot ingest skipped path '%s': %s\n",
                         e.path.c_str(), ex.what());
        } catch (...) {
          std::fprintf(stderr, "cvc: grpc snapshot ingest skipped path '%s' (non-std)\n",
                       e.path.c_str());
        }
      }
  }
}

void state_transport_grpc::on_inbound_heartbeat(const std::string &node_id,
                                                const std::string & /*cluster_id*/) {
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  _peers.note_seen(node_id, static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()));
}

bool state_transport_grpc::request_snapshot(const std::string &cluster_id,
                                            const std::string &path_prefix,
                                            snapshot_callback on_entries) {
  std::uint64_t req_id = _next_snap_req_id.fetch_add(1, std::memory_order_relaxed);
  auto waiter = std::make_shared<snap_waiter>();
  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters[req_id] = waiter;
  }

  pb::Frame frame;
  auto *req = frame.mutable_snapshot_request();
  req->set_cluster_id(cluster_id);
  req->set_path_prefix(path_prefix);
  req->set_request_id(req_id);

  // Send to first alive connection.
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  std::size_t sent = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load() || !c->can_write())
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++sent;
      break; // only need one peer
    }
  }

  bool result = false;
  if (sent > 0) {
    std::unique_lock<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters_cv.wait_for(lk, std::chrono::seconds(30), [&]() { return waiter->done; });
    if (waiter->done) {
      if (on_entries)
        on_entries(waiter->entries, true);
      result = true;
    }
  }

  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters.erase(req_id);
  }
  return result;
}

std::size_t state_transport_grpc::push_snapshot_to(const std::string &node_id,
                                                   const std::vector<snapshot_entry> &entries) {
  if (entries.empty())
    return 0;
  std::vector<std::shared_ptr<connection>> targets;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    for (const auto &c : _conns) {
      if (!c || !c->alive.load() || !c->can_write())
        continue;
      const std::string *pid = c->peer_id();
      if (pid && *pid == node_id)
        targets.push_back(c);
    }
  }
  if (targets.empty())
    return 0;

  // The same frame shape as the admit snapshot, so the client applies it through the same
  // unsolicited-snapshot path. Unfiltered: the caller chose `entries` for this peer.
  pb::Frame f;
  auto *rsp = f.mutable_snapshot_response();
  rsp->set_request_id(0);
  rsp->set_final(true);
  fill_snapshot_response(rsp, entries, /*view=*/nullptr);

  // A synchronous write per connection: a publish() on this thread afterwards lands behind it.
  std::size_t n = 0;
  for (auto &c : targets) {
    if (c->write(f)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
}

void state_transport_grpc::heartbeat_loop() {
  while (_running.load()) {
    auto sleep_end = std::chrono::steady_clock::now() + _heartbeat_interval;
    while (_running.load() && std::chrono::steady_clock::now() < sleep_end)
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!_running.load())
      break;

    pb::Frame frame;
    auto *hb = frame.mutable_heartbeat();
    hb->set_node_id(_node_id);
    hb->set_cluster_id(_cluster_id);
    hb->set_timestamp_ns(
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count()));

    std::vector<std::shared_ptr<connection>> conns;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      conns = _conns;
    }
    for (auto &c : conns) {
      if (!c || !c->alive.load() || !c->can_write())
        continue;
      if (c->write(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

} // namespace cvc

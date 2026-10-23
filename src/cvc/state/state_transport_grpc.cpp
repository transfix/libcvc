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
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message_bus.h>
#include <cvc/state/state_transport_grpc.h>
#include <functional>
#include <grpcpp/grpcpp.h>
#include <stdexcept>
#include <utility>

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

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

} // namespace

// Connection abstracts a single bidirectional stream. Server-side
// streams run inside a gRPC handler thread; client-side streams own
// a reader thread that we manage. Writes go through write() under
// write_mu so that the publish() fan-out and the per-stream Hello
// path serialize on the same lock.
//
// write_locked() exists for admit_connection(), which has to keep
// write_mu held across several frames; write_raw is the underlying
// stream write and must only be called with write_mu held.
class state_transport_grpc::connection {
public:
  std::mutex write_mu;
  std::function<bool(const pb::Frame &)> write_raw;
  std::atomic<bool> alive{true};
  std::string remote_node_id;
  std::string remote_cluster_id;

  bool can_write() const { return static_cast<bool>(write_raw); }

  bool write_locked(const pb::Frame &f) {
    if (!alive.load() || !write_raw)
      return false;
    return write_raw(f);
  }

  bool write(const pb::Frame &f) {
    std::lock_guard<std::mutex> lk(write_mu);
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
    auto *raw_stream = stream;
    conn->write_raw = [raw_stream](const pb::Frame &f) -> bool { return raw_stream->Write(f); };

    // Registers the connection and sends Hello followed by the
    // journal backfill, all under this connection's write mutex.
    _owner->admit_connection(conn);

    pb::Frame in;
    while (!ctx->IsCancelled() && stream->Read(&in)) {
      _owner->increment_recv_frames();
      if (in.has_hello()) {
        conn->remote_node_id = in.hello().node_id();
        conn->remote_cluster_id = in.hello().cluster_id();
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
        _owner->on_inbound_snapshot_response(sr.request_id(), entries, sr.final());
      } else if (in.has_heartbeat()) {
        _owner->on_inbound_heartbeat(in.heartbeat().node_id(), in.heartbeat().cluster_id());
      }
    }

    conn->alive.store(false);
    _owner->unregister_connection(conn.get());
    _owner->notify_connection_closed(conn->remote_node_id);
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
  conn->client_reader = std::thread([this, conn]() {
    pb::Frame in;
    while (conn->alive.load() && conn->client_stream->Read(&in)) {
      increment_recv_frames();
      if (in.has_hello()) {
        conn->remote_node_id = in.hello().node_id();
        conn->remote_cluster_id = in.hello().cluster_id();
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
        std::vector<snapshot_entry> entries;
        entries.reserve(sr.entries_size());
        for (const auto &e : sr.entries()) {
          snapshot_entry se;
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
        on_inbound_snapshot_response(sr.request_id(), entries, sr.final());
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
    std::lock_guard<std::mutex> wlk(c->write_mu);
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
  // Relay (star topology): re-send this mutation to every OTHER connection so clients that only
  // reach this server still converge with each other. The origin connection is skipped (no echo);
  // origin_node_id is preserved so duplicate suppression / vector clocks still work end-to-end.
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
  // Bridge to a sibling transport (e.g. the relay's WebSocket side) so its connections get this
  // mutation — the WS<->gRPC bridge that puts browser (wasm) and native clients in one room.
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
  // Relay (star topology): forward the message to every OTHER connection (see on_inbound_mutation).
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
  // Bridge to a sibling transport (the relay's WebSocket side) so its connections get this message.
  if (_bridge)
    _bridge->publish_message(m);
}

std::size_t state_transport_grpc::write_media(const media_frame &m, const void *exclude) {
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
  // Media is unreplicated, un-bridged (it crosses protocols via the media_router), and carries no
  // path. `exclude` (a connection*) skips the origin peer when a relay fans; nullptr sends to all.
  std::size_t n = 0;
  for (auto &c : conns) {
    if (!c || c.get() == exclude || !c->alive.load() || !c->can_write())
      continue;
    if (c->write(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++n;
    }
  }
  return n;
}

void state_transport_grpc::publish_media_frame(const media_frame &m) {
  _media_sent.fetch_add(write_media(m, nullptr), std::memory_order_relaxed);
}

void state_transport_grpc::fan_media(const media_frame &f, const void *exclude) {
  _media_fwd.fetch_add(write_media(f, exclude), std::memory_order_relaxed);
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
  std::lock_guard<std::mutex> wlk(conn->write_mu);
  register_connection(conn);
  send_hello_locked(*conn);
  if (backfill && !shards.empty())
    send_backfill_locked(*conn, shards);
  // A relay pushes its full current state so a LATE-joining client sees peers already in the room
  // (the live set_relay forwarding only carries writes that happen AFTER this client connects, and
  // the relay's room tree is all remote-origin so backfill's replay_local yields nothing). Mirrors
  // the WS relay's send_snapshot_locked — without it a native gRPC client joining a populated room
  // sees no one.
  if (relay && !shards.empty())
    send_snapshot_locked(*conn, shards);
}

void state_transport_grpc::send_hello_locked(connection &c) {
  pb::Frame f;
  auto *h = f.mutable_hello();
  h->set_node_id(_node_id);
  h->set_cluster_id(_cluster_id);
  c.write_locked(f);
}

void state_transport_grpc::send_backfill_locked(connection &c,
                                                const std::vector<state_cluster_shard *> &shards) {
  // The peer's Hello has not arrived yet, so there is no remote node
  // id to filter on; send everything and let the receiver drop what
  // does not match its cluster (on_inbound_mutation) or its interest
  // set (ingest_remote), exactly as it would for live traffic.
  for (auto *s : shards) {
    if (s == nullptr)
      continue;
    for (const auto &m : s->replay_local()) {
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
                                                const std::vector<state_cluster_shard *> &shards) {
  // Relay-only: push the full CURRENT snapshot of each shard as an unsolicited SnapshotResponse
  // (request_id 0), so a late joiner bootstraps the peers already present. replay_local() cannot do
  // this on a relay because the room tree it holds is all remote-origin (forwarded from clients).
  // The receiver applies each entry via on_inbound_snapshot_response's no-waiter path
  // (ingest_snapshot_ entry, a bulk initial load). Mirrors
  // state_transport_ws::send_snapshot_locked.
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
    for (auto &se : snap) {
      snapshot_entry te;
      te.path = std::move(se.path);
      te.string_value = std::move(se.string_value);
      te.comment = std::move(se.comment);
      te.hidden = se.hidden;
      te.read_only = se.read_only;
      te.properties = std::move(se.properties);
      te.type_name = std::move(se.type_name);
      te.origin_node_id = std::move(se.origin_node_id);
      te.sequence = se.sequence;
      te.data_type_name = std::move(se.data_type_name);
      te.data_payload = std::move(se.data_payload);
      all.push_back(std::move(te));
    }
  }

  // Build response frame.
  pb::Frame f;
  auto *rsp = f.mutable_snapshot_response();
  rsp->set_request_id(request_id);
  rsp->set_final(true);
  for (const auto &e : all) {
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
  if (!matched && !entries.empty()) {
    std::vector<state_cluster_shard *> shards;
    {
      std::lock_guard<std::mutex> lk(_shards_mu);
      shards = _shards;
    }
    for (const auto &e : entries)
      for (auto *s : shards) {
        if (s == nullptr)
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

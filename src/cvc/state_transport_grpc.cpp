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
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_transport_grpc.h>
#include <functional>
#include <grpcpp/grpcpp.h>
#include <stdexcept>
#include <utility>

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

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
  default:
    return state_mutation_op::set_value;
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

  pb::Payload *p = out->mutable_payload();
  switch (m.payload.kind) {
  case state_payload_kind::none:
    p->set_none(true);
    break;
  case state_payload_kind::inline_bytes:
    p->set_inline_bytes(std::string(m.payload.inline_bytes.begin(), m.payload.inline_bytes.end()));
    break;
  case state_payload_kind::blob: {
    auto *b = p->mutable_blob();
    b->set_digest(m.payload.blob.digest);
    b->set_size_bytes(m.payload.blob.size_bytes);
    b->set_codec(m.payload.blob.codec);
    break;
  }
  }
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
  if (in.has_payload()) {
    const auto &p = in.payload();
    switch (p.kind_case()) {
    case pb::Payload::kNone:
      m.payload.kind = state_payload_kind::none;
      break;
    case pb::Payload::kInlineBytes: {
      const std::string &s = p.inline_bytes();
      m.payload.kind = state_payload_kind::inline_bytes;
      m.payload.inline_bytes.assign(s.begin(), s.end());
      break;
    }
    case pb::Payload::kBlob:
      m.payload.kind = state_payload_kind::blob;
      m.payload.blob.digest = p.blob().digest();
      m.payload.blob.size_bytes = p.blob().size_bytes();
      m.payload.blob.codec = p.blob().codec();
      break;
    case pb::Payload::KIND_NOT_SET:
      m.payload.kind = state_payload_kind::none;
      break;
    }
  }
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

// Connection abstracts a single bidirectional stream. Server-side
// streams run inside a gRPC handler thread; client-side streams own
// a reader thread that we manage. Writes go through write_fn under
// write_mu so that the publish() fan-out and the per-stream Hello
// path serialize on the same lock.
class state_transport_grpc::connection {
public:
  std::mutex write_mu;
  std::function<bool(const pb::Frame &)> write_fn;
  std::atomic<bool> alive{true};
  std::string remote_node_id;
  std::string remote_cluster_id;

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
    auto *write_mu = &conn->write_mu;
    std::weak_ptr<state_transport_grpc::connection> wconn = conn;
    conn->write_fn = [raw_stream, write_mu, wconn](const pb::Frame &f) -> bool {
      auto sc = wconn.lock();
      if (!sc)
        return false;
      std::lock_guard<std::mutex> lk(*write_mu);
      if (!sc->alive.load())
        return false;
      return raw_stream->Write(f);
    };

    _owner->register_connection(conn);

    // Send our Hello first.
    {
      pb::Frame f;
      auto *h = f.mutable_hello();
      h->set_node_id(_owner->local_node_id());
      h->set_cluster_id(_owner->local_cluster_id());
      conn->write_fn(f);
    }

    pb::Frame in;
    while (!ctx->IsCancelled() && stream->Read(&in)) {
      _owner->increment_recv_frames();
      if (in.has_hello()) {
        conn->remote_node_id = in.hello().node_id();
        conn->remote_cluster_id = in.hello().cluster_id();
      } else if (in.has_mutation()) {
        _owner->on_inbound_mutation(decode_mutation(in.mutation()));
        _owner->increment_recv_mutations();
      } else if (in.has_message()) {
        _owner->on_inbound_message(decode_message(in.message()));
        _owner->increment_recv_messages();
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
          se.type_name = e.type_name();
          se.origin_node_id = e.origin_node_id();
          se.sequence = e.sequence();
          entries.push_back(std::move(se));
        }
        _owner->on_inbound_snapshot_response(sr.request_id(), entries, sr.final());
      } else if (in.has_heartbeat()) {
        _owner->on_inbound_heartbeat(in.heartbeat().node_id(), in.heartbeat().cluster_id());
      }
    }

    conn->alive.store(false);
    _owner->unregister_connection(conn.get());
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
  auto channel = grpc::CreateChannel(target, chan_creds);
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

  auto *write_mu = &conn->write_mu;
  std::weak_ptr<connection> wconn = conn;
  conn->write_fn = [stream, write_mu, wconn](const pb::Frame &f) -> bool {
    auto sc = wconn.lock();
    if (!sc)
      return false;
    std::lock_guard<std::mutex> lk(*write_mu);
    if (!sc->alive.load())
      return false;
    return stream->Write(f);
  };

  // Send our Hello.
  {
    pb::Frame f;
    auto *h = f.mutable_hello();
    h->set_node_id(_node_id);
    h->set_cluster_id(_cluster_id);
    conn->write_fn(f);
  }

  // Spawn reader.
  conn->client_reader = std::thread([this, conn]() {
    pb::Frame in;
    while (conn->alive.load() && conn->client_stream->Read(&in)) {
      increment_recv_frames();
      if (in.has_hello()) {
        conn->remote_node_id = in.hello().node_id();
        conn->remote_cluster_id = in.hello().cluster_id();
      } else if (in.has_mutation()) {
        on_inbound_mutation(decode_mutation(in.mutation()));
        increment_recv_mutations();
      } else if (in.has_message()) {
        on_inbound_message(decode_message(in.message()));
        increment_recv_messages();
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
          se.type_name = e.type_name();
          se.origin_node_id = e.origin_node_id();
          se.sequence = e.sequence();
          entries.push_back(std::move(se));
        }
        on_inbound_snapshot_response(sr.request_id(), entries, sr.final());
      } else if (in.has_heartbeat()) {
        on_inbound_heartbeat(in.heartbeat().node_id(), in.heartbeat().cluster_id());
      }
    }
    conn->alive.store(false);
  });

  register_connection(conn);
  return true;
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
    if (!c || !c->alive.load() || !c->write_fn)
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    if (c->write_fn(frame)) {
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
    if (!c || !c->alive.load() || !c->write_fn)
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    if (c->write_fn(frame)) {
      _sent_frames.fetch_add(1, std::memory_order_relaxed);
      ++stats.peers;
      if (!c->remote_node_id.empty())
        _peers.note_message_delivered(c->remote_node_id);
    }
  }
  return stats;
}

void state_transport_grpc::on_inbound_mutation(const state_mutation &m) {
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
}

void state_transport_grpc::on_inbound_message(const state_message &m) {
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
}

void state_transport_grpc::register_connection(std::shared_ptr<connection> conn) {
  std::lock_guard<std::mutex> lk(_conns_mu);
  _conns.push_back(std::move(conn));
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
  if (conn && conn->alive.load() && conn->write_fn) {
    conn->write_fn(f);
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
    if (!c || !c->alive.load() || !c->write_fn)
      continue;
    if (c->write_fn(frame)) {
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
      te.type_name = std::move(se.type_name);
      te.origin_node_id = std::move(se.origin_node_id);
      te.sequence = se.sequence;
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
    pe->set_type_name(e.type_name);
    pe->set_origin_node_id(e.origin_node_id);
    pe->set_sequence(e.sequence);
  }

  if (conn && conn->alive.load() && conn->write_fn) {
    conn->write_fn(f);
    _sent_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

void state_transport_grpc::on_inbound_snapshot_response(std::uint64_t request_id,
                                                        const std::vector<snapshot_entry> &entries,
                                                        bool final) {
  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    auto it = _snap_waiters.find(request_id);
    if (it != _snap_waiters.end()) {
      for (const auto &e : entries)
        it->second->entries.push_back(e);
      if (final)
        it->second->done = true;
    }
  }
  if (final)
    _snap_waiters_cv.notify_all();
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
    if (!c || !c->alive.load() || !c->write_fn)
      continue;
    if (c->write_fn(frame)) {
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
      if (!c || !c->alive.load() || !c->write_fn)
        continue;
      if (c->write_fn(frame))
        _sent_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "proto/state_transport.grpc.pb.h"
#include "proto/state_transport.pb.h"

#include <cvc/state/state_stream_transport_grpc.h>
#include <grpcpp/grpcpp.h>
#include <stdexcept>
#include <string>
#include <utility>

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

// Matches the control-plane cap so a single large frame round-trips above the
// ~4 MiB gRPC default (PR8 chunks frames larger than this).
constexpr int kStreamMaxMessageBytes = 64 * 1024 * 1024;

stream_transport_grpc::frame from_pb(const pb::StreamFrame &m) {
  stream_transport_grpc::frame f;
  f.stream_id = m.stream_id();
  f.seq = m.seq();
  f.pts_seconds = m.pts_seconds();
  f.flags = m.flags();
  const std::string &p = m.payload();
  f.payload.assign(p.begin(), p.end());
  return f;
}

void to_pb(const stream_transport_grpc::frame &f, pb::StreamFrame &m) {
  m.set_stream_id(f.stream_id);
  m.set_seq(f.seq);
  m.set_pts_seconds(f.pts_seconds);
  m.set_flags(f.flags);
  m.set_payload(std::string(f.payload.begin(), f.payload.end()));
}

// Server side of StreamTransport. Each inbound StreamChannel runs here on its
// own gRPC thread: deliver every frame to the sink, then echo it back so a
// client can observe a round-trip. PR8/PR9 replace the echo with real
// producer→consumer fan-out and negotiation.
class StreamTransportServiceImpl final : public pb::StreamTransport::Service {
public:
  StreamTransportServiceImpl(stream_transport_grpc::on_frame_func on_frame,
                             std::atomic<std::uint64_t> *recv, std::atomic<std::uint64_t> *echoed)
      : _on_frame(std::move(on_frame)), _recv(recv), _echoed(echoed) {}

  grpc::Status
  StreamChannel(grpc::ServerContext *ctx,
                grpc::ServerReaderWriter<pb::StreamFrame, pb::StreamFrame> *stream) override {
    pb::StreamFrame in;
    while (!ctx->IsCancelled() && stream->Read(&in)) {
      _recv->fetch_add(1, std::memory_order_relaxed);
      if (_on_frame)
        _on_frame(from_pb(in));
      if (stream->Write(in))
        _echoed->fetch_add(1, std::memory_order_relaxed);
    }
    return grpc::Status::OK;
  }

private:
  stream_transport_grpc::on_frame_func _on_frame;
  std::atomic<std::uint64_t> *_recv;
  std::atomic<std::uint64_t> *_echoed;
};

// Client side of an open StreamChannel.
class client_channel final : public stream_transport_grpc::channel {
public:
  client_channel(std::shared_ptr<grpc::Channel> chan,
                 std::unique_ptr<pb::StreamTransport::Stub> stub,
                 std::shared_ptr<grpc::ClientContext> ctx,
                 std::unique_ptr<grpc::ClientReaderWriter<pb::StreamFrame, pb::StreamFrame>> stream)
      : _chan(std::move(chan)), _stub(std::move(stub)), _ctx(std::move(ctx)),
        _stream(std::move(stream)) {}

  ~client_channel() override { close(); }

  bool send(const stream_transport_grpc::frame &f) override {
    if (!_stream || _closed)
      return false;
    pb::StreamFrame m;
    to_pb(f, m);
    return _stream->Write(m);
  }

  bool recv(stream_transport_grpc::frame &out) override {
    if (!_stream || _closed)
      return false;
    pb::StreamFrame m;
    if (!_stream->Read(&m))
      return false;
    out = from_pb(m);
    return true;
  }

  void close() override {
    if (_closed)
      return;
    _closed = true;
    if (_stream) {
      _stream->WritesDone();
      _stream->Finish();
    }
  }

private:
  std::shared_ptr<grpc::Channel> _chan;
  std::unique_ptr<pb::StreamTransport::Stub> _stub;
  std::shared_ptr<grpc::ClientContext> _ctx;
  std::unique_ptr<grpc::ClientReaderWriter<pb::StreamFrame, pb::StreamFrame>> _stream;
  bool _closed = false;
};

} // namespace

struct stream_transport_grpc::impl {
  std::string listen_addr_resolved;
  on_frame_func on_frame;
  std::unique_ptr<StreamTransportServiceImpl> service;
  std::unique_ptr<grpc::Server> server;
};

stream_transport_grpc::stream_transport_grpc() : _impl(new impl()) {}

stream_transport_grpc::~stream_transport_grpc() { stop(); }

std::size_t stream_transport_grpc::max_message_bytes() noexcept {
  return static_cast<std::size_t>(kStreamMaxMessageBytes);
}

void stream_transport_grpc::set_on_frame(on_frame_func cb) { _impl->on_frame = std::move(cb); }

void stream_transport_grpc::start(const std::string &listen_addr) {
  if (_running.load())
    throw std::runtime_error("stream_transport_grpc::start: already running");

  _impl->service.reset(
      new StreamTransportServiceImpl(_impl->on_frame, &_frames_received, &_frames_echoed));

  grpc::ServerBuilder builder;
  int bound_port = 0;
  builder.SetMaxReceiveMessageSize(kStreamMaxMessageBytes);
  builder.SetMaxSendMessageSize(kStreamMaxMessageBytes);
  builder.AddListeningPort(listen_addr, grpc::InsecureServerCredentials(), &bound_port);
  builder.RegisterService(_impl->service.get());
  _impl->server = builder.BuildAndStart();
  if (!_impl->server)
    throw std::runtime_error("stream_transport_grpc::start: BuildAndStart failed");

  auto colon = listen_addr.find_last_of(':');
  std::string host = (colon == std::string::npos) ? listen_addr : listen_addr.substr(0, colon);
  _impl->listen_addr_resolved =
      (bound_port > 0) ? host + ":" + std::to_string(bound_port) : listen_addr;

  _running.store(true);
}

void stream_transport_grpc::stop() {
  if (!_running.exchange(false))
    return;
  if (_impl->server) {
    _impl->server->Shutdown();
    _impl->server->Wait();
    _impl->server.reset();
  }
  _impl->service.reset();
}

std::string stream_transport_grpc::listen_address() const {
  return _impl ? _impl->listen_addr_resolved : std::string();
}

std::unique_ptr<stream_transport_grpc::channel>
stream_transport_grpc::open_channel(const std::string &target, std::chrono::milliseconds timeout) {
  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(kStreamMaxMessageBytes);
  args.SetMaxSendMessageSize(kStreamMaxMessageBytes);
  auto chan = grpc::CreateCustomChannel(target, grpc::InsecureChannelCredentials(), args);
  auto deadline = std::chrono::system_clock::now() + timeout;
  if (!chan->WaitForConnected(deadline))
    return nullptr;

  auto stub = pb::StreamTransport::NewStub(chan);
  auto ctx = std::make_shared<grpc::ClientContext>();
  auto stream = stub->StreamChannel(ctx.get());
  if (!stream)
    return nullptr;

  return std::unique_ptr<channel>(
      new client_channel(chan, std::move(stub), ctx, std::move(stream)));
}

} // namespace cvc

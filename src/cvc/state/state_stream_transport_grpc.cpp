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

#include <algorithm>
#include <cstddef>
#include <cvc/state/state_stream_transport_grpc.h>
#include <grpcpp/grpcpp.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace {

namespace pb = ::cvc::transport::v1;

// Matches the control-plane cap so a single large frame round-trips above the
// ~4 MiB gRPC default (PR8 chunks frames larger than this).
constexpr int kStreamMaxMessageBytes = 64 * 1024 * 1024;

// Split a frame into `chunk_bytes`-sized StreamFrames and write them in index
// order over an ordered gRPC stream. chunk_bytes == 0 sends the whole payload
// as one message. Every chunk repeats (stream_id, seq, pts, flags) so each is
// self-describing. Returns false if a Write fails (the stream is down).
template <class Writer>
bool write_chunked(Writer *w, const stream_transport_grpc::frame &f, std::size_t chunk_bytes) {
  const std::size_t total = f.payload.size();
  const std::size_t step = (chunk_bytes == 0) ? (total == 0 ? std::size_t{1} : total) : chunk_bytes;
  const std::uint32_t count =
      (total == 0) ? 1u : static_cast<std::uint32_t>((total + step - 1) / step);
  for (std::uint32_t i = 0; i < count; ++i) {
    pb::StreamFrame m;
    m.set_stream_id(f.stream_id);
    m.set_seq(f.seq);
    m.set_pts_seconds(f.pts_seconds);
    m.set_flags(f.flags);
    m.set_chunk_index(i);
    m.set_chunk_count(count);
    const std::size_t off = static_cast<std::size_t>(i) * step;
    const std::size_t len = (off < total) ? std::min(step, total - off) : 0;
    if (len > 0)
      m.set_payload(std::string(reinterpret_cast<const char *>(f.payload.data() + off), len));
    if (!w->Write(m))
      return false;
  }
  return true;
}

// Reassembles chunked frames arriving on one ordered stream. offer() returns
// true and fills `out` when a chunk completes a frame. In-order delivery is
// assumed (the gRPC stream is ordered and write_chunked emits chunks in index
// order); a gap, an out-of-order index, or an unexpected (stream_id, seq) drops
// the in-progress frame rather than mis-stitching it.
class frame_reassembler {
public:
  bool offer(const pb::StreamFrame &c, stream_transport_grpc::frame &out) {
    const std::uint32_t count = (c.chunk_count() == 0) ? 1u : c.chunk_count();
    const std::uint32_t idx = c.chunk_index();
    if (idx == 0 || !_active || c.stream_id() != _sid || c.seq() != _seq) {
      if (idx != 0) {
        _active = false; // a non-initial chunk with no matching in-progress frame
        return false;
      }
      _active = true; // (re)start on the first chunk
      _sid = c.stream_id();
      _seq = c.seq();
      _pts = c.pts_seconds();
      _flags = c.flags();
      _count = count;
      _next = 0;
      _buf.clear();
    }
    if (idx != _next) { // gap / out of order
      _active = false;
      return false;
    }
    const std::string &p = c.payload();
    _buf.insert(_buf.end(), p.begin(), p.end());
    ++_next;
    if (_next >= _count) {
      out.stream_id = _sid;
      out.seq = _seq;
      out.pts_seconds = _pts;
      out.flags = _flags;
      out.payload = _buf;
      _active = false;
      return true;
    }
    return false;
  }

private:
  bool _active = false;
  std::string _sid;
  std::int64_t _seq = 0;
  double _pts = 0.0;
  std::uint32_t _flags = 0;
  std::uint32_t _count = 1;
  std::uint32_t _next = 0;
  std::vector<unsigned char> _buf;
};

// Server side of StreamTransport. Each inbound StreamChannel runs here on its
// own gRPC thread: deliver every frame to the sink, then echo it back so a
// client can observe a round-trip. PR8/PR9 replace the echo with real
// producer→consumer fan-out and negotiation.
class StreamTransportServiceImpl final : public pb::StreamTransport::Service {
public:
  StreamTransportServiceImpl(stream_transport_grpc::on_frame_func on_frame,
                             stream_transport_grpc::open_handler_func open_handler,
                             std::atomic<std::uint64_t> *recv, std::atomic<std::uint64_t> *echoed,
                             bool echo, std::size_t chunk_bytes)
      : _on_frame(std::move(on_frame)), _open_handler(std::move(open_handler)), _recv(recv),
        _echoed(echoed), _echo(echo), _chunk_bytes(chunk_bytes) {}

  grpc::Status
  StreamChannel(grpc::ServerContext *ctx,
                grpc::ServerReaderWriter<pb::StreamFrame, pb::StreamFrame> *stream) override {
    frame_reassembler reasm;
    pb::StreamFrame in;
    bool first = true;
    while (!ctx->IsCancelled() && stream->Read(&in)) {
      if (first) {
        first = false;
        // Server-push: a flag_open first frame switches this channel to pushing
        // the named stream's frames (the conventional producer-serves-a-port
        // model). The open frame carries no data of its own.
        if (_open_handler && (in.flags() & stream_transport_grpc::flag_open)) {
          stream_transport_grpc::frame_producer producer = _open_handler(in.stream_id());
          if (producer) {
            for (;;) {
              if (ctx->IsCancelled())
                break;
              stream_transport_grpc::frame f; // fresh each push
              if (!producer(f))
                break; // stream exhausted
              if (!write_chunked(stream, f, _chunk_bytes))
                break; // client disconnected
            }
            return grpc::Status::OK;
          }
        }
      }
      // Sink path (non-push clients): reassemble into whole frames, optionally
      // echoing the raw chunk back (the PR7/8 round-trip shape; OFF for the
      // bridge's one-way push, where an unread echo would deadlock the sender).
      if (_echo && stream->Write(in))
        _echoed->fetch_add(1, std::memory_order_relaxed);
      stream_transport_grpc::frame whole;
      if (reasm.offer(in, whole)) {
        _recv->fetch_add(1, std::memory_order_relaxed);
        if (_on_frame)
          _on_frame(whole);
      }
    }
    return grpc::Status::OK;
  }

private:
  stream_transport_grpc::on_frame_func _on_frame;
  stream_transport_grpc::open_handler_func _open_handler;
  std::atomic<std::uint64_t> *_recv;
  std::atomic<std::uint64_t> *_echoed;
  bool _echo;
  std::size_t _chunk_bytes;
};

// Client side of an open StreamChannel.
class client_channel final : public stream_transport_grpc::channel {
public:
  client_channel(std::shared_ptr<grpc::Channel> chan,
                 std::unique_ptr<pb::StreamTransport::Stub> stub,
                 std::shared_ptr<grpc::ClientContext> ctx,
                 std::unique_ptr<grpc::ClientReaderWriter<pb::StreamFrame, pb::StreamFrame>> stream,
                 std::size_t chunk_bytes)
      : _chan(std::move(chan)), _stub(std::move(stub)), _ctx(std::move(ctx)),
        _stream(std::move(stream)), _chunk_bytes(chunk_bytes) {}

  ~client_channel() override { close(); }

  bool send(const stream_transport_grpc::frame &f) override {
    if (!_stream || _closed)
      return false;
    return write_chunked(_stream.get(), f, _chunk_bytes);
  }

  bool recv(stream_transport_grpc::frame &out) override {
    if (!_stream || _closed)
      return false;
    pb::StreamFrame m;
    // Read chunks until one completes a whole frame (or the stream ends).
    while (_stream->Read(&m)) {
      if (_reasm.offer(m, out))
        return true;
    }
    return false;
  }

  void close() override {
    if (_closed)
      return;
    _closed = true;
    // Abort the RPC. TryCancel() is thread-safe and unblocks a reader thread
    // parked in recv()'s Read() on a LONG-LIVED server stream — one that never
    // finishes on its own (e.g. a serve_port push that is idle in its producer).
    // WritesDone()+Finish() do NOT: Finish() blocks until the server finishes
    // (which such a stream never does) and may not run concurrently with a
    // reader's Read(). After TryCancel the stream/context clean up on
    // destruction, so no Finish() is needed here.
    if (_ctx)
      _ctx->TryCancel();
  }

private:
  std::shared_ptr<grpc::Channel> _chan;
  std::unique_ptr<pb::StreamTransport::Stub> _stub;
  std::shared_ptr<grpc::ClientContext> _ctx;
  std::unique_ptr<grpc::ClientReaderWriter<pb::StreamFrame, pb::StreamFrame>> _stream;
  std::size_t _chunk_bytes = 0;
  frame_reassembler _reasm;
  bool _closed = false;
};

} // namespace

struct stream_transport_grpc::impl {
  std::string listen_addr_resolved;
  on_frame_func on_frame;
  open_handler_func open_handler;
  std::unique_ptr<StreamTransportServiceImpl> service;
  std::unique_ptr<grpc::Server> server;
};

stream_transport_grpc::stream_transport_grpc() : _impl(new impl()) {}

stream_transport_grpc::~stream_transport_grpc() { stop(); }

std::size_t stream_transport_grpc::max_message_bytes() noexcept {
  return static_cast<std::size_t>(kStreamMaxMessageBytes);
}

void stream_transport_grpc::set_chunk_bytes(std::size_t bytes) noexcept {
  _chunk_bytes.store(bytes, std::memory_order_relaxed);
}

std::size_t stream_transport_grpc::chunk_bytes() const noexcept {
  return _chunk_bytes.load(std::memory_order_relaxed);
}

void stream_transport_grpc::set_server_echo(bool on) noexcept {
  _server_echo.store(on, std::memory_order_relaxed);
}

void stream_transport_grpc::set_tls_config(tls_config cfg) {
  if (_running.load())
    throw std::runtime_error("stream_transport_grpc::set_tls_config: already running");
  _tls = std::move(cfg);
  _tls_set = true;
}

void stream_transport_grpc::set_on_frame(on_frame_func cb) { _impl->on_frame = std::move(cb); }

void stream_transport_grpc::set_open_handler(open_handler_func h) {
  _impl->open_handler = std::move(h);
}

void stream_transport_grpc::start(const std::string &listen_addr) {
  if (_running.load())
    throw std::runtime_error("stream_transport_grpc::start: already running");

  _impl->service.reset(new StreamTransportServiceImpl(_impl->on_frame, _impl->open_handler,
                                                      &_frames_received, &_frames_echoed,
                                                      _server_echo.load(), _chunk_bytes.load()));

  grpc::ServerBuilder builder;
  int bound_port = 0;
  builder.SetMaxReceiveMessageSize(kStreamMaxMessageBytes);
  builder.SetMaxSendMessageSize(kStreamMaxMessageBytes);
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
  auto chan = grpc::CreateCustomChannel(target, chan_creds, args);
  auto deadline = std::chrono::system_clock::now() + timeout;
  if (!chan->WaitForConnected(deadline))
    return nullptr;

  auto stub = pb::StreamTransport::NewStub(chan);
  auto ctx = std::make_shared<grpc::ClientContext>();
  auto stream = stub->StreamChannel(ctx.get());
  if (!stream)
    return nullptr;

  return std::unique_ptr<channel>(
      new client_channel(chan, std::move(stub), ctx, std::move(stream), _chunk_bytes.load()));
}

} // namespace cvc

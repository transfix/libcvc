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

#ifndef __CVC_STATE_STREAM_TRANSPORT_GRPC_H__
#define __CVC_STATE_STREAM_TRANSPORT_GRPC_H__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

// ----------------
// cvc::stream_transport_grpc  (Phase 10, PR7 — the streaming DATA plane)
// ----------------
// Purpose:
//   The gRPC transport for stream FRAME bytes, separate from the control-plane
//   state_transport_grpc. It runs its OWN gRPC server on its OWN port/listener
//   (the StreamTransport service, proto/state_transport.proto) so that
//   high-bandwidth frame traffic cannot head-of-line-block control traffic on
//   the StateTransport server. One bidirectional StreamChannel per
//   consumer↔producer pair.
//
//   This layer is transport-level: it moves opaque `frame` records (routing +
//   timing metadata + payload bytes). Mapping a cvc::ariadne::stream::frame to
//   and from this envelope, and the discover→negotiate→open sequence, live a
//   layer above (PR9). PR7 is the skeleton: start a server, open a channel to a
//   peer, and round-trip frames. Per-frame chunking of oversized payloads is
//   PR8; today a `frame` carries one whole frame.
//
// Threading:
//   start()/stop() are not thread-safe against each other. Once running, the
//   server dispatches each inbound StreamChannel on its own gRPC thread and
//   invokes the on_frame callback there; the callback must be thread-safe. A
//   `channel` is driven from one thread at a time (its send()/recv() are not
//   mutually thread-safe); distinct channels are independent.
//
// Availability:
//   Only built when CVC_ENABLE_GRPC is on (like state_transport_grpc). The
//   header is plain (no gRPC types) so it may be included unconditionally, but
//   the symbols link only in a gRPC-enabled build.
//
class stream_transport_grpc {
public:
  // Transport-level frame: routing + timing + bytes. `flags` bit0 = keyframe,
  // bit1 = open handshake (the first frame a consumer sends to name its stream).
  struct frame {
    std::string stream_id;
    std::int64_t seq = 0;
    double pts_seconds = 0.0;
    std::uint32_t flags = 0;
    std::vector<unsigned char> payload;
  };

  enum frame_flag : std::uint32_t { flag_keyframe = 1u << 0, flag_open = 1u << 1 };

  // Invoked on a server gRPC thread for every frame received on any inbound
  // StreamChannel. Must be thread-safe.
  using on_frame_func = std::function<void(const frame &)>;

  // A client-side open StreamChannel. send() writes a frame to the peer; recv()
  // blocks for the next frame from the peer (false when the stream ends).
  // Destroying or close()ing the channel half-closes and finishes the RPC.
  class channel {
  public:
    virtual ~channel() = default;
    virtual bool send(const frame &f) = 0;
    virtual bool recv(frame &out) = 0;
    virtual void close() = 0;
  };

  stream_transport_grpc();
  ~stream_transport_grpc();

  stream_transport_grpc(const stream_transport_grpc &) = delete;
  stream_transport_grpc &operator=(const stream_transport_grpc &) = delete;

  // Server-side sink for inbound frames. Set before start(). The skeleton
  // server also echoes each inbound frame back on its channel so a client can
  // observe a round-trip; the echo is independent of this callback.
  void set_on_frame(on_frame_func cb);

  // Start the dedicated StreamTransport server listening on `listen_addr`
  // (host:port; port 0 picks a free port, read back via listen_address()).
  void start(const std::string &listen_addr);
  void stop();
  bool is_running() const noexcept { return _running.load(); }

  // The resolved listen address (with the OS-chosen port substituted when the
  // request used port 0). Empty before start().
  std::string listen_address() const;

  // Client: open a StreamChannel to a peer's StreamTransport server. Returns
  // null if the channel cannot connect within `timeout`.
  std::unique_ptr<channel> open_channel(const std::string &target,
                                        std::chrono::milliseconds timeout);

  // Diagnostics (cumulative since start()). frames_received counts WHOLE frames
  // reassembled and delivered to the sink; frames_echoed counts wire messages
  // (chunks) echoed back — so frames_echoed > frames_received means frames were
  // split into multiple chunks.
  std::uint64_t frames_received() const noexcept { return _frames_received.load(); }
  std::uint64_t frames_echoed() const noexcept { return _frames_echoed.load(); }

  // Max gRPC message size this transport negotiates (server + client), so even a
  // single chunk round-trips comfortably above the ~4 MiB gRPC default.
  static std::size_t max_message_bytes() noexcept;

  // Stream-tuned per-chunk payload cap (PR8), independent of the gRPC message
  // cap: a frame whose payload exceeds this is split into that-many-byte chunks
  // on send() and reassembled by (stream_id, seq) on the peer. This bounds the
  // size of any single wire message for frame traffic without touching the
  // control plane's 64 MiB limit. Applies to channels opened AFTER it is set
  // (open_channel captures the current value). 0 disables splitting (one
  // message per frame). Default: 1 MiB.
  void set_chunk_bytes(std::size_t bytes) noexcept;
  std::size_t chunk_bytes() const noexcept;

private:
  struct impl;
  std::unique_ptr<impl> _impl;

  std::atomic<bool> _running{false};
  std::atomic<std::uint64_t> _frames_received{0};
  std::atomic<std::uint64_t> _frames_echoed{0};
  std::atomic<std::size_t> _chunk_bytes{1u << 20}; // 1 MiB
};

} // namespace cvc

#endif // __CVC_STATE_STREAM_TRANSPORT_GRPC_H__

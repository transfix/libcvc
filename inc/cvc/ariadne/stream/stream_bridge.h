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

#ifndef __CVC_ARIADNE_STREAM_STREAM_BRIDGE_H__
#define __CVC_ARIADNE_STREAM_STREAM_BRIDGE_H__

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cvc {

class app;
class stream_transport_grpc; // cvc::state (data plane); built under CVC_ENABLE_GRPC

namespace ariadne {
namespace stream {

class stream;

// ----------------
// cvc::ariadne::stream::stream_bridge  (Phase 10, PR9 — the cross-node capstone)
// ----------------
// Ties the local stream subsystem to the gRPC data plane so a stream produced
// on one node is consumed on another — the moment frames actually cross.
//
//   PRODUCER  serve(local, consumer_endpoint): opens a StreamChannel to the
//     consumer's StreamTransport and pushes `local`'s frames — a pusher
//     subscribes to local.channel() (ring) and send()s each frame (chunked by
//     the transport) until the stream closes or the consumer disconnects.
//   CONSUMER  receive_into(stream_id, proxy): routes inbound frames for
//     `stream_id` into `proxy`, a local stream opened with the stream's format
//     (from the replicated descriptor). Frames arrive via publish_external, so
//     local consumers of `proxy` observe them like any local stream.
//
// This bridge maps a cvc::ariadne::stream::frame to/from the transport's
// envelope: the payload is the frame bytes; seq and pts ride along; the FORMAT
// is not re-sent per frame — the consumer already has it from the descriptor.
//
// Roles vs the original plan: to reuse the PR7/PR8 transport unchanged (client
// send + server sink + chunking), the CONSUMER runs the gRPC server and the
// PRODUCER is the client that pushes. The endpoint the producer connects to is
// the consumer's; wiring it via a sendMessage offer/answer handshake (so a
// consumer that merely discovers the descriptor is auto-served) is the thin
// next layer on top of this bridge. The conventional producer-serves-a-port
// model is a later refinement that needs a transport server-push path.
//
// Threading:
//   serve() spawns one pusher thread per call; stop()/the destructor close the
//   subscriptions (unblocking pop()) and join them. The consumer router runs on
//   the transport's server threads and is guarded by an internal mutex.
//
// Availability:
//   Uses cvc::stream_transport_grpc, so it is built only under CVC_ENABLE_GRPC.
//
class stream_bridge {
public:
  // `transport` must outlive the bridge. Construct the bridge BEFORE starting
  // `transport` on the consumer side: the constructor installs the frame router
  // as the transport's on_frame sink (the transport captures that sink at
  // start()). receive_into() then updates routes dynamically.
  stream_bridge(app &ctx, stream_transport_grpc &transport);
  ~stream_bridge();

  stream_bridge(const stream_bridge &) = delete;
  stream_bridge &operator=(const stream_bridge &) = delete;

  // PRODUCER: push `local`'s frames to the consumer's StreamTransport at
  // `consumer_endpoint`. Returns false if the channel cannot connect in time.
  bool serve(stream &local, const std::string &consumer_endpoint,
             std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  // CONSUMER: route inbound frames tagged `stream_id` into `proxy`. `proxy` must
  // outlive the bridge (or until stop()). Safe to call after the transport has
  // started.
  void receive_into(const std::string &stream_id, stream &proxy);

  // Stop pushing and routing: close subscriptions, join pusher threads, close
  // channels, clear routes. Idempotent; also run by the destructor.
  void stop();

  // Diagnostics.
  std::uint64_t frames_pushed() const noexcept { return _frames_pushed.load(); }
  std::uint64_t frames_injected() const noexcept { return _router ? _router->injected.load() : 0; }

private:
  struct serving; // one producer→consumer push (channel + subscription + thread)

  // Consumer routing state. Held via shared_ptr and co-owned by the transport's
  // on_frame sink lambda, so a frame delivered on a transport server thread
  // after the bridge is destroyed routes against a still-live object (stop()
  // clears routes, so it simply finds nothing) rather than a dangling `this`.
  struct router_state {
    std::mutex mu;
    std::map<std::string, stream *> routes; // stream_id -> local proxy
    std::atomic<std::uint64_t> injected{0};
  };

  app &_ctx;
  stream_transport_grpc &_transport;
  std::shared_ptr<router_state> _router;

  std::mutex _mu;                                  // guards _servings + _stopped
  std::vector<std::unique_ptr<serving>> _servings; // producer pushers
  bool _stopped = false;

  std::atomic<std::uint64_t> _frames_pushed{0};
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_BRIDGE_H__

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
class subscription;
struct stream_params;

// ----------------
// cvc::ariadne::stream::stream_bridge  (Phase 10, PR9 — the cross-node capstone)
// ----------------
// Ties the local stream subsystem to the gRPC data plane so a stream produced
// on one node is consumed on another — the moment frames actually cross. Two
// transport roles are supported; a given stream uses one or the other.
//
//   ── Consumer-serves-a-port (client push) ──
//   PRODUCER  serve(local, consumer_endpoint): opens a StreamChannel to the
//     consumer's StreamTransport and pushes `local`'s frames — a pusher
//     subscribes to local.channel() (ring) and send()s each frame (chunked by
//     the transport) until the stream closes or the consumer disconnects.
//   CONSUMER  receive_into(stream_id, proxy): routes inbound frames for
//     `stream_id` into `proxy`, a local stream opened with the stream's format
//     (from the replicated descriptor). Frames arrive via publish_external, so
//     local consumers of `proxy` observe them like any local stream.
//
//   ── Producer-serves-a-port (server push, the conventional model) ──
//   PRODUCER  serve_port(local): publishes `local` on THIS bridge's (server)
//     StreamTransport. When a consumer opens a channel naming `local`'s id, the
//     transport's open-handler subscribes to local.channel() (ring) and pushes
//     each frame back down that channel until the stream closes or the consumer
//     disconnects. One started server serves any number of serve_port streams.
//   CONSUMER  pull_from(producer_endpoint, proxy): opens a StreamChannel to the
//     producer's StreamTransport, names the stream (proxy's id) with an open
//     handshake, and injects the frames the producer pushes back into `proxy`.
//
// This bridge maps a cvc::ariadne::stream::frame to/from the transport's
// envelope: the payload is the frame bytes; seq and pts ride along; the FORMAT
// is not re-sent per frame — the consumer already has it from the descriptor.
//
// Roles: serve()/receive_into() reuse the PR7/PR8 transport unchanged (client
// send + server sink), so the CONSUMER runs the gRPC server and the PRODUCER
// pushes to it; auto_serve()/request_remote() wire that endpoint via a
// sendMessage offer/answer handshake. serve_port()/pull_from() use the PR-follow-up
// server-push path instead, so the PRODUCER runs the server and consumers pull
// — the conventional model, useful when one producer serves many consumers from
// a stable port.
//
// Threading:
//   serve() spawns one pusher thread per call and pull_from() one puller thread;
//   stop()/the destructor unblock them (closing the subscription for a pusher,
//   closing the channel for a puller) and join them. serve_port() adds no bridge
//   thread — its pushes run on the transport's own server threads, draining
//   subscriptions that stop() closes. The consumer router and the served-stream
//   table run on transport server threads and are each guarded by a mutex.
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

  // PRODUCER (server push): publish `local` on THIS bridge's StreamTransport,
  // which must be a started server. A consumer that opens a channel naming
  // `local`'s id (see pull_from) is then pushed `local`'s frames on the
  // transport's own server thread. Registers a route only; the push starts when
  // a consumer connects and ends when `local` closes or the consumer drops.
  // `local` must outlive the bridge (or until stop()). Idempotent per id.
  void serve_port(stream &local);

  // CONSUMER: route inbound frames tagged `stream_id` into `proxy`. `proxy` must
  // outlive the bridge (or until stop()). Safe to call after the transport has
  // started.
  void receive_into(const std::string &stream_id, stream &proxy);

  // CONSUMER (server push): open a StreamChannel to a producer's StreamTransport
  // at `producer_endpoint`, name the stream (proxy's id) with an open handshake,
  // and inject the frames the producer pushes back into `proxy`. The counterpart
  // to a producer's serve_port(). `proxy` must outlive the bridge (or until
  // stop()). Returns false if the channel cannot connect in time.
  bool pull_from(const std::string &producer_endpoint, stream &proxy,
                 std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  // The content_type marking a stream-subscription offer message (an out-of-band
  // state message whose string_value is the consumer's StreamTransport endpoint).
  static const char *const kOfferContentType;

  // PRODUCER (auto-negotiation): offer `local` for cross-node consumers without
  // naming them. Subscribes to the app's default-shard message bus at
  // local.token(); when a consumer sends an offer there (see request_remote),
  // serve()s `local` to that consumer's endpoint. `local` must outlive the
  // bridge (or until stop()). Requires a default shard installed for the app.
  void auto_serve(stream &local,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  // CONSUMER (auto-negotiation): open a local proxy stream for a remote stream
  // described by `params` (its id/root/format, from the discovered descriptor),
  // route inbound frames into it, and sendMessage an offer carrying THIS bridge's
  // transport endpoint to the stream's path so the producer starts pushing.
  // Returns the proxy (local consumers subscribe to it), or null on failure. The
  // bridge's transport must be a started server (its listen_address() is offered).
  std::unique_ptr<stream> request_remote(const stream_params &params);

  // Stop pushing and routing: close subscriptions, join pusher threads, close
  // channels, clear routes. Idempotent; also run by the destructor.
  void stop();

  // Diagnostics. frames_pushed counts frames this bridge sent to a peer, whether
  // via serve() (client push) or serve_port() (server push); frames_injected
  // counts frames delivered into a local proxy, via receive_into() or pull_from().
  std::uint64_t frames_pushed() const noexcept {
    return _frames_pushed.load() + (_served ? _served->pushed.load() : 0);
  }
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

  // Producer-side server-push state. Co-owned (shared_ptr) by the transport's
  // open-handler lambda so a consumer that connects after the bridge is
  // destroyed routes against a still-live object (stopped==true → declines)
  // rather than a dangling `this`. `subs` holds every per-consumer subscription
  // the open-handler created, so stop() can close them to unblock the server
  // threads draining them.
  struct served_state {
    std::mutex mu;
    std::map<std::string, stream *> served;          // stream_id -> local stream
    std::vector<std::shared_ptr<subscription>> subs; // per-consumer drains to close
    std::atomic<std::uint64_t> pushed{0};
    bool stopped = false;
  };

  app &_ctx;
  stream_transport_grpc &_transport;
  std::shared_ptr<router_state> _router;
  std::shared_ptr<served_state> _served;

  std::mutex _mu;                                   // guards _servings + _stopped + _offer_unsubs
  std::vector<std::unique_ptr<serving>> _servings;  // producer pushers
  std::vector<std::function<void()>> _offer_unsubs; // auto_serve bus unsubscribes
  bool _stopped = false;

  std::atomic<std::uint64_t> _frames_pushed{0};
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_BRIDGE_H__

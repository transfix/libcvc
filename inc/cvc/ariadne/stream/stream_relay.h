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

#ifndef __CVC_ARIADNE_STREAM_STREAM_RELAY_H__
#define __CVC_ARIADNE_STREAM_STREAM_RELAY_H__

#include <cstddef>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace cvc {

class app;
class stream_transport_grpc; // cvc::state (data plane); built under CVC_ENABLE_GRPC

namespace ariadne {
namespace stream {

class stream;
struct stream_params;
class stream_bridge;

// ----------------
// cvc::ariadne::stream::stream_relay  (conference Phase 0 — the SFU fan-out core)
// ----------------
// A Selective Forwarding Unit: ingest each publisher's stream ONCE into a
// per-publisher proxy, then fan it out to any number of subscribers, over the
// gRPC StreamTransport. It packages the proven stream_bridge pattern — a proxy
// filled by receive_into() and re-offered by serve_port() — with lifetime and
// unique-id management, so a room controller can drive it per participant.
//
// Topology (clients connect OUT to the relay; the hub forwards):
//   * PUBLISHER  serve()s its stream to the relay's transport. The frames carry
//     the stream's token as their id; the relay routes them into proxy[token].
//   * SUBSCRIBER pull_from()s the relay's transport naming that same token; the
//     relay pushes proxy[token]'s frames back. One ingest, N egress.
// relay() opens each proxy with the publisher's id, so the proxy's token equals
// the publisher's token and publisher/subscriber names line up with no extra
// negotiation.
//
// The relay forwards OPAQUE frames: it never decodes media. That is what lets a
// later end-to-end-encryption layer (SFrame / insertable streams) keep the
// payload private to the clients — the relay routes on the frame envelope only.
//
// Scale: this is the full-fan-out core (every publisher to every subscriber),
// which is the MVP shape. Per-subscriber selection (simulcast/active-speaker)
// and broadcast fan-out trees are later layers on top of this primitive.
//
// Threading / lifetime: the relay owns one stream_bridge over the caller's
// (started) transport and one proxy stream per relayed id. Fan-out runs on the
// transport's own server threads (serve_port). stop()/the destructor stop the
// bridge BEFORE the proxies are dropped, so no server thread races teardown.
// Call stop() before stopping the transport. Construct the relay BEFORE
// start()ing the transport (the bridge installs the transport's router and
// open-handler, captured at start()).
//
// Availability: uses cvc::stream_transport_grpc, so built only under
// CVC_ENABLE_GRPC.
class stream_relay {
public:
  // `transport` is the relay's StreamTransport server; it must outlive the
  // relay and be start()ed AFTER the relay is constructed.
  stream_relay(app &ctx, stream_transport_grpc &transport);
  ~stream_relay();

  stream_relay(const stream_relay &) = delete;
  stream_relay &operator=(const stream_relay &) = delete;

  // Make a stream relayable: open a per-publisher ingest proxy sized by
  // `params` (set params.expected_subscribers to the max fan-out), route the
  // publisher's inbound frames into it, and offer it to subscribers. Returns
  // the proxy's token (what publishers serve to and subscribers pull by), or an
  // empty string on failure (bad params, or the relay is stopped). Idempotent
  // per stream identity — the (root_path, id) pair, i.e. the token — so same-id
  // streams in different scopes (every participant's "camera"/"audio") get their
  // own proxy; a repeat call for a live identity returns its existing token.
  std::string relay(const stream_params &params);

  // Stop everything: stop the bridge (close serve_port subscriptions, join),
  // then drop all proxies. Idempotent; also run by the destructor. Call BEFORE
  // stopping the transport.
  //
  // NOTE: per-stream teardown (a participant leaving mid-call) is a follow-up —
  // it needs a keyed remove-route/unserve on stream_bridge so a single proxy
  // can be retired without a dangling pointer. Phase 0 is relay() + stop().
  void stop();

  // Diagnostics.
  std::size_t stream_count() const;       // live relayed streams
  std::uint64_t frames_forwarded() const; // frames pushed to subscribers

private:
  app &_ctx;
  stream_transport_grpc &_transport;
  std::unique_ptr<stream_bridge> _bridge; // receive_into routes + serve_port offers
  mutable std::mutex _mu;
  // token (registry_key = <root_path>.streams.<id>) -> ingest/egress proxy
  std::map<std::string, std::unique_ptr<stream>> _proxies;
  bool _stopped = false;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_RELAY_H__

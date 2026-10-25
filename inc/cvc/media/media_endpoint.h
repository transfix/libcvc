/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_ENDPOINT_H__
#define __CVC_MEDIA_ENDPOINT_H__

#include <cvc/media/media_frame.h>
#include <functional>
#include <memory>
#include <string>

namespace cvc {

// What a receiver gets of one frame (media_endpoint::set_media_shaper).
struct media_shape {
  bool drop = false;                          // nothing
  std::shared_ptr<const media_frame> variant; // this instead (null: the frame as sent)
};

// cvc::media_endpoint — the PLUG-IN SURFACE for a media transport. A transport (WebSocket, gRPC,
// and whatever protocol we add next: WebRTC, QUIC, ...) implements this so a cvc::media_router can
// fan the generic cvc::media_frame between protocols without knowing any wire format. The transport
// does the two translations: inbound wire frame -> media_frame (handed to the sink), and
// media_frame
// -> outbound wire frame (fan_media).
//
// ECHO SUPPRESSION is by an OPAQUE per-endpoint connection token, never by the (forgeable)
// origin_node_id. When a frame arrives, the endpoint hands the sink an opaque `src_conn` that
// identifies the connection it came in on. The router passes that same token back as `exclude` ONLY
// to the endpoint that produced it, so the origin peer is skipped while every other peer — on this
// endpoint and on all other endpoints — receives the frame. The token is meaningful only to the
// endpoint that minted it; the router treats it as a void* cookie and never dereferences it.
//
// A single-connection LEAF client (e.g. a browser) also implements this: fan_media pushes its own
// frame up its one connection (no peers to skip), and the sink delivers peers' frames the relay
// sent down. Such a client needs no router; it uses the convenience publish/subscribe sugar its
// transport layers on top (publish_media_frame / set_on_media).
class media_endpoint {
public:
  virtual ~media_endpoint() = default;

  // Egress: deliver `f` to every peer connection of this endpoint EXCEPT the one identified by
  // `exclude` (an opaque token this endpoint previously handed its sink; nullptr = all
  // connections). Thread-safe; a no-op with no connections. Must not block indefinitely.
  virtual void fan_media(const media_frame &f, const void *exclude = nullptr) = 0;

  // Ingress: set the sink invoked (on a reader thread) for each MediaFrame received from a peer.
  // The second argument is the opaque source-connection token for `exclude`. Must be thread-safe
  // and must not block. Setting it replaces any previous sink; pass {} to clear.
  using media_sink_fn = std::function<void(const media_frame &f, const void *src_conn)>;
  virtual void set_media_sink(media_sink_fn sink) = 0;

  // A short, stable protocol tag for diagnostics and routing policy: "ws", "grpc", "webrtc", ...
  virtual const char *media_protocol() const noexcept = 0;

  // Per-receiver EGRESS gate: fan_media asks it once per receiving connection, with the node_id
  // that connection identified as (Hello), and skips the connection when it returns false. A
  // connection that has not identified yet always receives. Asked on the publisher's reader
  // thread, so it must be thread-safe and non-blocking (no tree walk, no lock held across a
  // socket write). It never gates a node's OWN uplink (publish_media_frame). Setting it replaces
  // any previous gate; {} (the default) is today's full fan-out. Default: ignored (an endpoint
  // without per-receiver egress, e.g. a test fake).
  using media_gate_fn =
      std::function<bool(const media_frame &f, const std::string &receiver_node_id)>;
  virtual void set_media_gate(media_gate_fn gate) { (void)gate; }

  // Per-receiver egress SHAPE, asked after the gate admits a frame: send the frame as is, a
  // variant of it (the same media, smaller -- a relay's lower video rung, media_rungs.h) or
  // nothing (media_shape). Same thread rules as the gate. One variant may serve many receivers: an
  // endpoint encodes each variant once per frame. Setting it replaces any previous shaper; {} (the
  // default) sends every frame as is. Default: ignored.
  using media_shape_fn =
      std::function<media_shape(const media_frame &f, const std::string &receiver_node_id)>;
  virtual void set_media_shaper(media_shape_fn shaper) { (void)shaper; }
};

} // namespace cvc

#endif // __CVC_MEDIA_ENDPOINT_H__

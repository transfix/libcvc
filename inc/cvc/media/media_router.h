/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_ROUTER_H__
#define __CVC_MEDIA_ROUTER_H__

#include <atomic>
#include <cstdint>
#include <cvc/media/media_endpoint.h>
#include <mutex>
#include <vector>

namespace cvc {

// cvc::media_router — the HUB that makes media interop protocol-agnostic. A relay registers one
// media_endpoint per protocol it speaks (WebSocket for browsers, gRPC for native, ...); a frame
// ingested on ANY endpoint is fanned to the egress of EVERY endpoint, so a browser (WS) and a
// native client (gRPC) in the same room see and hear each other through a single canonical path.
//
// It carries only the generic cvc::media_frame and knows nothing of any wire format — all protocol
// specifics live in the endpoints. Adding a new protocol is implementing cvc::media_endpoint and
// calling add_endpoint(); the router needs no change. (This is the media counterpart to the state
// transports' control-plane set_bridge, generalized from a 1:1 cross-pointer to an N-way hub.)
//
// ECHO SUPPRESSION uses the endpoint's opaque source-connection token (see media_endpoint): the
// origin peer is skipped on its own endpoint, everyone else receives the frame. There is no reliance
// on origin_node_id, and no per-protocol special-casing.
//
// Lifetime: every registered endpoint must outlive the router (or be removed while quiescent).
// remove_endpoint() clears that endpoint's sink before dropping it, so no in-flight frame routes
// through a removed endpoint.
class media_router {
public:
  media_router() = default;
  ~media_router();

  media_router(const media_router &) = delete;
  media_router &operator=(const media_router &) = delete;

  // Register `ep` and wire its ingress sink to this router. Idempotent for a pointer already added.
  void add_endpoint(media_endpoint *ep);

  // Unregister `ep`: clear its sink (so it stops feeding the router) and drop it from the fan set.
  void remove_endpoint(media_endpoint *ep);

  std::size_t endpoint_count() const;
  std::uint64_t total_routed() const noexcept { return _routed.load(std::memory_order_relaxed); }

private:
  // Called (on an endpoint's reader thread) for each frame `src` ingested; fans to every endpoint,
  // excluding the origin connection on `src` itself.
  void ingest(const media_frame &f, media_endpoint *src, const void *src_conn);

  mutable std::mutex _mu;
  std::vector<media_endpoint *> _eps;
  std::atomic<std::uint64_t> _routed{0};
};

} // namespace cvc

#endif // __CVC_MEDIA_ROUTER_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/media/media_router.h>

namespace cvc {

media_router::~media_router() {
  // Clear every endpoint's sink so none feeds a router that is going away.
  std::vector<media_endpoint *> eps;
  {
    std::lock_guard<std::mutex> lk(_mu);
    eps.swap(_eps);
  }
  for (media_endpoint *ep : eps)
    if (ep)
      ep->set_media_sink({});
}

void media_router::add_endpoint(media_endpoint *ep) {
  if (!ep)
    return;
  {
    std::lock_guard<std::mutex> lk(_mu);
    if (std::find(_eps.begin(), _eps.end(), ep) != _eps.end())
      return; // already registered
    _eps.push_back(ep);
  }
  // Wire ingress: every frame this endpoint receives is ingested here, tagged with its endpoint and
  // the opaque source-connection token so ingest() can skip the origin.
  ep->set_media_sink(
      [this, ep](const media_frame &f, const void *src_conn) { ingest(f, ep, src_conn); });
}

void media_router::remove_endpoint(media_endpoint *ep) {
  if (!ep)
    return;
  bool was_registered = false;
  {
    std::lock_guard<std::mutex> lk(_mu);
    auto it = std::find(_eps.begin(), _eps.end(), ep);
    if (it != _eps.end()) {
      _eps.erase(it);
      was_registered = true;
    }
  }
  // Stop it feeding the router. Done outside the lock; the sink is the endpoint's own, not ours.
  if (was_registered)
    ep->set_media_sink({});
}

std::size_t media_router::endpoint_count() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _eps.size();
}

void media_router::ingest(const media_frame &f, media_endpoint *src, const void *src_conn) {
  _routed.fetch_add(1, std::memory_order_relaxed);
  // Snapshot the (small, rarely-changing) endpoint set, then fan OUTSIDE the lock: fan_media does
  // socket writes, and holding _mu across them would serialize all media and risk lock-ordering
  // against an endpoint's own connection locks. Endpoints must outlive the router (see header), so a
  // snapshotted pointer stays valid for the duration of this call.
  std::vector<media_endpoint *> eps;
  {
    std::lock_guard<std::mutex> lk(_mu);
    eps = _eps;
  }
  for (media_endpoint *ep : eps) {
    if (!ep)
      continue;
    // On the origin endpoint, skip the connection the frame arrived on; every other endpoint sends
    // to all of its connections (the origin peer is not among them).
    ep->fan_media(f, ep == src ? src_conn : nullptr);
  }
}

} // namespace cvc

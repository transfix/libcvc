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

#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream/stream_relay.h>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace stream {

stream_relay::stream_relay(app &ctx, stream_transport_grpc &transport)
    : _ctx(ctx), _transport(transport), _bridge(new stream_bridge(ctx, transport)) {}

stream_relay::~stream_relay() { stop(); }

std::string stream_relay::relay(const stream_params &params) {
  std::lock_guard<std::mutex> lk(_mu);
  if (_stopped)
    return std::string();
  // Idempotent per stream IDENTITY — the canonical (root_path, id) key, which IS the token. The id
  // ALONE is not unique: a conference scopes each participant's streams under its own root_path
  // (the pid), so every participant publishes id "camera"/"audio" and only the scope tells them
  // apart. Keying on id alone collides them, and the idempotent early-return below would then drop
  // every participant's stream but the first (stream_count would read 2, never 2*N).
  const std::string key = stream::registry_key(params.root_path, params.id);
  auto existing = _proxies.find(key);
  if (existing != _proxies.end())
    return existing->second->token();

  std::unique_ptr<stream> proxy = stream::open(_ctx, params);
  if (!proxy)
    return std::string(); // unsized format, empty id, or a live dup under (root,id)

  const std::string token = proxy->token(); // == key == registry_key(root_path, id)
  // Ingest: a publisher serve()s this token to us; route those frames in.
  _bridge->receive_into(token, *proxy);
  // Egress: offer the proxy so each subscriber can pull it by the same token.
  _bridge->serve_port(*proxy);
  _proxies.emplace(key, std::move(proxy));
  return token;
}

void stream_relay::stop() {
  std::vector<std::unique_ptr<stream>> proxies; // keep proxies alive past bridge teardown
  {
    std::lock_guard<std::mutex> lk(_mu);
    if (_stopped)
      return;
    _stopped = true;
    for (auto &kv : _proxies)
      proxies.push_back(std::move(kv.second));
    _proxies.clear();
  }
  // Stop the bridge FIRST: it closes the serve_port subscriptions (unblocking
  // the transport server threads draining them), joins, and clears its route /
  // served tables — so after this the bridge holds no pointer into any proxy.
  if (_bridge)
    _bridge->stop();
  // Only now drop the proxies (the moved-from unique_ptrs still named the same
  // stream objects, which the bridge referenced until stop() cleared them).
  proxies.clear();
}

std::size_t stream_relay::stream_count() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _proxies.size();
}

std::uint64_t stream_relay::frames_forwarded() const {
  // frames_pushed() counts serve_port egress to subscribers (plus any serve()).
  return _bridge ? _bridge->frames_pushed() : 0;
}

} // namespace stream
} // namespace ariadne
} // namespace cvc

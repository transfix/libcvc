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

#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace stream {

// One active producer→consumer push: the client channel, the subscription it
// drains, and the pusher thread.
struct stream_bridge::serving {
  std::unique_ptr<cvc::stream_transport_grpc::channel> channel;
  std::shared_ptr<subscription> sub;
  std::thread pusher;
};

stream_bridge::stream_bridge(app &ctx, stream_transport_grpc &transport)
    : _ctx(ctx), _transport(transport), _router(std::make_shared<router_state>()) {
  // A bridge does one-way push (producer send → consumer sink); it never reads
  // echoes. Disable the server echo so an unread echo cannot fill flow control
  // and deadlock a producer's push. (Both must be set before the transport
  // start()s, which the bridge-before-start contract guarantees.)
  _transport.set_server_echo(false);
  // Install the frame router as the transport's sink. The lambda co-owns the
  // router_state (shared_ptr), so it stays valid even if a frame arrives after
  // the bridge is destroyed. Must be set before the transport start()s, which
  // is why the bridge is constructed first (see the header).
  std::shared_ptr<router_state> r = _router;
  _transport.set_on_frame([r](const cvc::stream_transport_grpc::frame &wf) {
    stream *proxy = nullptr;
    {
      std::lock_guard<std::mutex> lk(r->mu);
      auto it = r->routes.find(wf.stream_id);
      if (it != r->routes.end())
        proxy = it->second;
    }
    if (!proxy)
      return;
    // Copy the chunk-reassembled bytes into a keepalive the frame borrows for
    // its lifetime (publish_external treats `data` as immutable + owned by
    // `keepalive`).
    auto buf = std::make_shared<std::vector<std::uint8_t>>(wf.payload.begin(), wf.payload.end());
    proxy->channel().publish_external(buf->data(), buf->size(), wf.pts_seconds, buf);
    r->injected.fetch_add(1, std::memory_order_relaxed);
  });
}

stream_bridge::~stream_bridge() { stop(); }

void stream_bridge::receive_into(const std::string &stream_id, stream &proxy) {
  std::lock_guard<std::mutex> lk(_router->mu);
  _router->routes[stream_id] = &proxy;
}

bool stream_bridge::serve(stream &local, const std::string &consumer_endpoint,
                          std::chrono::milliseconds timeout) {
  std::unique_ptr<cvc::stream_transport_grpc::channel> ch =
      _transport.open_channel(consumer_endpoint, timeout);
  if (!ch)
    return false;

  // Ring subscription: pop() blocks for each new frame and returns false when
  // the stream closes, which ends the pusher cleanly. Depth 3 matches the
  // default stream pool so serve() works on a stream opened with defaults; a
  // full ring drops oldest (drop-at-source) rather than stalling the producer.
  std::shared_ptr<subscription> sub = local.channel().subscribe(deliver_mode::ring, 3);
  if (!sub) {
    ch->close();
    return false;
  }

  const std::string stream_id = local.token();
  auto s = std::unique_ptr<serving>(new serving());
  cvc::stream_transport_grpc::channel *raw_ch = ch.get();
  subscription *raw_sub = sub.get();
  s->channel = std::move(ch);
  s->sub = sub;
  s->pusher = std::thread([this, raw_ch, raw_sub, stream_id]() {
    frame_ptr f;
    while (raw_sub->pop(f)) {
      if (!f)
        continue;
      cvc::stream_transport_grpc::frame wf;
      wf.stream_id = stream_id;
      wf.seq = f->seq;
      wf.pts_seconds = f->pts_seconds;
      wf.payload.assign(f->data, f->data + f->size);
      if (!raw_ch->send(wf))
        break; // consumer disconnected
      _frames_pushed.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::lock_guard<std::mutex> lk(_mu);
  _servings.push_back(std::move(s));
  return true;
}

void stream_bridge::stop() {
  std::vector<std::unique_ptr<serving>> servings;
  {
    std::lock_guard<std::mutex> lk(_mu);
    if (_stopped)
      return;
    _stopped = true;
    servings.swap(_servings);
  }
  // Stop routing first (post-stop deliveries find no proxy).
  {
    std::lock_guard<std::mutex> lk(_router->mu);
    _router->routes.clear();
  }
  // Close each subscription to unblock its pusher's pop(), join, then close the
  // channel (the joined thread no longer touches it).
  for (auto &s : servings) {
    if (s->sub)
      s->sub->close();
    if (s->pusher.joinable())
      s->pusher.join();
    if (s->channel)
      s->channel->close();
  }
}

} // namespace stream
} // namespace ariadne
} // namespace cvc

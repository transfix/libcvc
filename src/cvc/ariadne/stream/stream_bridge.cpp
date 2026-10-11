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
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace stream {

// One active bridge-owned transfer: the client channel plus the worker thread
// that drives it. For a producer push (serve()), `sub` is the local
// subscription the worker drains and `reader` is false; the worker is unblocked
// at teardown by closing `sub`. For a consumer pull (pull_from()), `sub` is null
// and `reader` is true; the worker blocks in the channel's recv() and is
// unblocked by closing the channel. (serve_port() uses no serving — its pushes
// run on transport server threads; see served_state.)
struct stream_bridge::serving {
  std::unique_ptr<cvc::stream_transport_grpc::channel> channel;
  std::shared_ptr<subscription> sub;
  std::thread worker;
  bool reader = false;
};

stream_bridge::stream_bridge(app &ctx, stream_transport_grpc &transport)
    : _ctx(ctx), _transport(transport), _router(std::make_shared<router_state>()),
      _served(std::make_shared<served_state>()) {
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

  // Install the server-push open-handler (the serve_port side). It is invoked on
  // a transport server thread when a consumer opens a channel naming a stream;
  // it returns a frame_producer that drains that stream's subscription. Captures
  // only the shared served_state (never `this`), so a late connect after the
  // bridge is destroyed finds stopped==true and declines. Must be set before the
  // transport start()s — same bridge-before-start contract as the sink above.
  std::shared_ptr<served_state> sv = _served;
  _transport.set_open_handler(
      [sv](const std::string &sid) -> cvc::stream_transport_grpc::frame_producer {
        stream *local = nullptr;
        {
          std::lock_guard<std::mutex> lk(sv->mu);
          if (sv->stopped)
            return {};
          auto it = sv->served.find(sid);
          if (it != sv->served.end())
            local = it->second;
        }
        if (!local)
          return {}; // not served here → decline (transport falls back to sink)
        // One ring subscription per consumer; depth 3 matches the default pool
        // and drops oldest under backpressure (drop-at-source) rather than
        // stalling the producer.
        std::shared_ptr<subscription> sub = local->channel().subscribe(deliver_mode::ring, 3);
        if (!sub)
          return {};
        {
          std::lock_guard<std::mutex> lk(sv->mu);
          if (sv->stopped) {
            sub->close(); // raced with stop(): don't start a push that won't be joined
            return {};
          }
          sv->subs.push_back(sub);
        }
        return [sv, sub, sid](cvc::stream_transport_grpc::frame &out) -> bool {
          frame_ptr f;
          while (sub->pop(f)) { // blocks; false when the stream (or sub) closes
            if (!f)
              continue;
            out.stream_id = sid;
            out.seq = f->seq;
            out.pts_seconds = f->pts_seconds;
            out.payload.assign(f->data, f->data + f->size);
            sv->pushed.fetch_add(1, std::memory_order_relaxed);
            return true;
          }
          return false; // exhausted → server ends this channel
        };
      });
}

stream_bridge::~stream_bridge() { stop(); }

void stream_bridge::receive_into(const std::string &stream_id, stream &proxy) {
  std::lock_guard<std::mutex> lk(_router->mu);
  _router->routes[stream_id] = &proxy;
}

const char *const stream_bridge::kOfferContentType = "application/x-cvc-stream-offer";

void stream_bridge::auto_serve(stream &local, std::chrono::milliseconds timeout) {
  state_cluster_shard *shard = state_cluster_shard::default_for(_ctx);
  if (!shard)
    return; // no default shard installed → no bus to hear offers on
  const std::string token = local.token();
  state_message_bus &bus = shard->message_bus();
  // On an offer at this stream's path, connect to the consumer and push.
  // NB: the bus dispatches under its own lock, so serve()'s connect runs there
  // synchronously — fine for a reachable consumer; a slow/unreachable one would
  // briefly stall offer dispatch (a future refinement can offload the connect).
  state_message_bus::subscription_id id =
      bus.subscribe(token, [this, &local, timeout](const state_message &m) {
        if (m.content_type != kOfferContentType)
          return;
        const std::string endpoint = m.string_value;
        if (!endpoint.empty())
          serve(local, endpoint, timeout);
      });
  std::lock_guard<std::mutex> lk(_mu);
  _offer_unsubs.push_back([&bus, id]() { bus.unsubscribe(id); });
}

std::unique_ptr<stream> stream_bridge::request_remote(const stream_params &params) {
  std::unique_ptr<stream> proxy = stream::open(_ctx, params);
  if (!proxy)
    return nullptr;
  receive_into(proxy->token(), *proxy);
  // Offer this bridge's transport endpoint at the stream's path; the producer's
  // auto_serve() subscriber picks it up and starts pushing frames here.
  state::instance(_ctx)(proxy->token())
      .sendMessage(_transport.listen_address(), kOfferContentType, /*hop_budget=*/64,
                   /*binary=*/false);
  return proxy;
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
  s->worker = std::thread([this, raw_ch, raw_sub, stream_id]() {
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

void stream_bridge::serve_port(stream &local) {
  // Register the stream; the open-handler installed in the ctor does the actual
  // push when a consumer connects naming this id. No bridge thread is spawned.
  std::lock_guard<std::mutex> lk(_served->mu);
  if (_served->stopped)
    return;
  _served->served[local.token()] = &local;
}

bool stream_bridge::pull_from(const std::string &producer_endpoint, stream &proxy,
                              std::chrono::milliseconds timeout) {
  std::unique_ptr<cvc::stream_transport_grpc::channel> ch =
      _transport.open_channel(producer_endpoint, timeout);
  if (!ch)
    return false;

  // Open handshake: name the stream we want pushed. The producer's open-handler
  // (serve_port) matches this id and pushes that stream's frames back on this
  // same channel. The open frame carries no payload of its own.
  cvc::stream_transport_grpc::frame open;
  open.stream_id = proxy.token();
  open.flags = cvc::stream_transport_grpc::flag_open;
  if (!ch->send(open)) {
    ch->close();
    return false;
  }

  auto s = std::unique_ptr<serving>(new serving());
  s->reader = true;
  cvc::stream_transport_grpc::channel *raw_ch = ch.get();
  s->channel = std::move(ch);
  stream *proxy_ptr = &proxy;
  // Puller thread: drain pushed frames and inject them into the proxy, exactly as
  // the consumer router does for the client-push path. recv() returns false when
  // the producer ends the stream or stop() closes the channel.
  s->worker = std::thread([this, raw_ch, proxy_ptr]() {
    cvc::stream_transport_grpc::frame wf;
    while (raw_ch->recv(wf)) {
      auto buf = std::make_shared<std::vector<std::uint8_t>>(wf.payload.begin(), wf.payload.end());
      proxy_ptr->channel().publish_external(buf->data(), buf->size(), wf.pts_seconds, buf);
      _router->injected.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::lock_guard<std::mutex> lk(_mu);
  _servings.push_back(std::move(s));
  return true;
}

void stream_bridge::stop() {
  std::vector<std::unique_ptr<serving>> servings;
  std::vector<std::function<void()>> offer_unsubs;
  {
    std::lock_guard<std::mutex> lk(_mu);
    if (_stopped)
      return;
    _stopped = true;
    servings.swap(_servings);
    offer_unsubs.swap(_offer_unsubs);
  }
  // Stop hearing new offers first (no new servings appear during teardown).
  for (auto &unsub : offer_unsubs)
    unsub();
  // Stop routing first (post-stop deliveries find no proxy).
  {
    std::lock_guard<std::mutex> lk(_router->mu);
    _router->routes.clear();
  }
  // Stop server-push: decline new connects and close every per-consumer
  // subscription so the transport server threads draining them unblock and end
  // their channels. Those threads belong to the transport and are joined by its
  // own stop() (call bridge.stop() before transport.stop()).
  {
    std::lock_guard<std::mutex> lk(_served->mu);
    _served->stopped = true;
    _served->served.clear();
    for (auto &sub : _served->subs)
      if (sub)
        sub->close();
    _served->subs.clear();
  }
  // Join bridge-owned workers. A pusher (serve()) is unblocked by closing its
  // subscription, then its channel is closed after the join (the joined thread
  // no longer touches it). A puller (pull_from()) blocks in recv(), so its
  // channel must be closed BEFORE the join to unblock it.
  for (auto &s : servings) {
    if (s->reader) {
      if (s->channel)
        s->channel->close();
      if (s->worker.joinable())
        s->worker.join();
    } else {
      if (s->sub)
        s->sub->close();
      if (s->worker.joinable())
        s->worker.join();
      if (s->channel)
        s->channel->close();
    }
  }
}

} // namespace stream
} // namespace ariadne
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <boost/any.hpp>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/intrinsics.h>      // resolve_channel_key (§12 channel scoping)
#include <cvc/stream/stream.h>
#include <cvc/stream/stream_registry.h>

namespace cvc {
namespace stream {

namespace {
std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
} // namespace

stream::stream(app &ctx, std::string id) : ctx_(ctx), id_(std::move(id)) {}

std::string stream::registry_key(const std::string &root_path, const std::string &id) {
  // Canonical: <root_path>.streams.<id>, normalized (root_path empty => streams.<id>).
  std::string p = root_path.empty() ? std::string() : root_path + state::SEPARATOR;
  p += "streams";
  p += state::SEPARATOR;
  p += id;
  return state::normalize_path(p);
}

stream::~stream() {
  close();
  // Release the descriptor pins/handles. The channel's dtor close()s again
  // (idempotent). The /streams/<id> node stays in the tree, marked "closed".
  stats_node_ = state::handle();
  descriptor_ = state::handle();
  descriptor_pins_.clear();
}

std::unique_ptr<stream> stream::open(app &ctx, const stream_params &p) {
  const std::size_t slab_bytes = p.format.frame_bytes();
  if (slab_bytes == 0 || p.id.empty())
    return nullptr; // unsized format or missing id

  std::unique_ptr<stream> s(new stream(ctx, p.id));
  s->scope_ = p.root_path;
  // §4.1 scoping. The registry key / descriptor path is the canonical
  // <root_path>.streams.<id>. The seq/evt event channels are scoped through the
  // SAME chroot resolver a (msg-recv ...) in this document uses, computed ONCE
  // here on the owner thread; the producer then posts these pre-resolved keys, so
  // producer and a same-scope receiver resolve to the identical string (and a
  // bottom-up `channels:` grant can bridge them across documents).
  s->token_ = registry_key(p.root_path, p.id);
  s->seq_channel_ = cvc::state_exec::resolve_channel_key(p.root_path, "streams." + p.id + ".seq");
  s->evt_channel_ = cvc::state_exec::resolve_channel_key(p.root_path, "streams." + p.id + ".evt");
  s->heartbeat_ns_ = p.heartbeat_hz > 0.0 ? static_cast<std::int64_t>(1.0e9 / p.heartbeat_hz) : 0;

  std::size_t slab_count =
      p.expected_subscribers * (p.subscriber_depth + p.in_flight_per_subscriber) +
      p.producer_working_set;
  if (slab_count == 0)
    slab_count = 1;
  s->pool_ = frame_pool::create(slab_bytes, slab_count);
  s->channel_ = std::make_unique<stream_channel>(
      ctx, p.id, p.format, s->pool_, p.producer_working_set, p.in_flight_per_subscriber);

  if (!stream_registry::for_app(ctx).install(s->token_, s->channel_.get()))
    return nullptr; // token already live

  // Publish the descriptor node at the canonical scoped path <root_path>.streams.<id>
  // (== token_) on the owner thread. sharedChild with the whole dotted path pins
  // EVERY ancestor hop (root_path's segments + streams + id), because value()/data()
  // walk _parent via fullName()/childChanged (H1); a leaf-only pin would leave a
  // chrooted stream's ancestors exposed to a concurrent sweep.
  try {
    state &root = state::instance(ctx);
    state::state_ptr node = root.sharedChild(s->token_, &s->descriptor_pins_);
    node->data(boost::any(p.format));
    node->value("live");
    state::state_ptr stats = node->sharedChild("stats", &s->descriptor_pins_);
    s->descriptor_ = state::handle(node);
    s->stats_node_ = state::handle(stats);
  } catch (...) {
    stream_registry::for_app(ctx).uninstall(s->token_, s->channel_.get());
    return nullptr;
  }

  s->update_stats();
  s->live_ = true; // fully constructed: the "closed" teardown is now this stream's to post
  s->post_lifecycle("live");
  return s;
}

void stream::post_seq(std::int64_t seq) {
  pending_seq_.store(seq, std::memory_order_relaxed);
  if (heartbeat_ns_ <= 0) {
    ctx_.exec_scheduler().post_message(seq_channel_,
                                       cvc::state_exec::value_t(static_cast<int64_t>(seq)));
    return;
  }
  const std::int64_t now = steady_now_ns();
  std::int64_t last = last_post_ns_.load(std::memory_order_relaxed);
  if (now - last < heartbeat_ns_)
    return; // throttle: coalesce; the payload carries the latest seq
  if (!last_post_ns_.compare_exchange_strong(last, now, std::memory_order_relaxed))
    return; // another poster won this window
  ctx_.exec_scheduler().post_message(
      seq_channel_,
      cvc::state_exec::value_t(static_cast<int64_t>(pending_seq_.load(std::memory_order_relaxed))));
}

void stream::post_lifecycle(const char *lifecycle) {
  ctx_.exec_scheduler().post_message(evt_channel_,
                                     cvc::state_exec::value_t(std::string(lifecycle)));
}

void stream::update_stats() {
  if (!stats_node_)
    return;
  stream_stats st;
  // Use the CHANNEL's count (= seq_), which includes both pool and external
  // (publish_external) publishes; pool_->total_published() would undercount a
  // producer that aliases its own buffers via publish_external.
  st.published = channel_ ? channel_->total_published() : 0;
  st.borrow_fail = pool_ ? pool_->total_borrow_fail() : 0;
  st.slabs_in_use = pool_ ? pool_->in_use() : 0;
  st.subscribers = channel_ ? channel_->subscriber_count() : 0;
  st.last_seq = channel_ ? channel_->last_seq() : -1;
  try {
    stats_node_->data(boost::any(st));
  } catch (...) {
    // best-effort telemetry
  }
}

void stream::close() {
  if (closed_.exchange(true))
    return;
  if (channel_)
    channel_->close(); // unblock parked ring consumers
  stream_registry::for_app(ctx_).uninstall(token_, channel_ ? channel_.get() : nullptr);
  // Only a stream that actually went live owns the "closed" transition on the
  // (id-shared) evt/seq channels + descriptor. A failed open() tears down
  // silently so it cannot disturb an already-live stream of the same id.
  if (!live_)
    return;
  // Flush the final seq the heartbeat throttle may have withheld, so a receiver
  // sees the last frame index before the "closed" event.
  if (pending_seq_.load(std::memory_order_relaxed) >= 0)
    ctx_.exec_scheduler().post_message(
        seq_channel_, cvc::state_exec::value_t(
                          static_cast<int64_t>(pending_seq_.load(std::memory_order_relaxed))));
  if (descriptor_) {
    try {
      descriptor_->value("closed");
    } catch (...) {
    }
  }
  post_lifecycle("closed");
}

} // namespace stream
} // namespace cvc

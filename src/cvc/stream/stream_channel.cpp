/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/stream/stream_channel.h>

namespace cvc {
namespace stream {

// --------------------------------------------------------------------------
// subscription
// --------------------------------------------------------------------------

subscription::subscription(deliver_mode m, std::size_t depth) : mode_(m), depth_(depth) {
  if (mode_ == deliver_mode::ring) {
    ring_ = std::make_unique<state_bounded_queue<frame_ptr>>(
        depth == 0 ? 1 : depth, state_bounded_queue<frame_ptr>::overflow_policy::drop_oldest);
  }
}

void subscription::deliver(const frame_ptr &f) {
  if (mode_ == deliver_mode::ring) {
    if (ring_)
      ring_->push(f); // drop_oldest: never blocks, bumps total_dropped_oldest() on overflow
    return;
  }
  // latest register: publish the freshest frame with one atomic swap. Count
  // every supersession of a non-null slot as an overwrite (telemetry: an upper
  // bound on frames this freshest-wins subscriber skipped — read state is not
  // tracked, so a frame the consumer already sampled still counts).
  frame_ptr prev = std::atomic_exchange(&latest_, f);
  if (prev)
    overwrites_.fetch_add(1, std::memory_order_relaxed);
}

frame_ptr subscription::latest() const { return std::atomic_load(&latest_); }

bool subscription::pop(frame_ptr &out) {
  if (mode_ == deliver_mode::ring && ring_)
    return ring_->pop(out);
  return false;
}

bool subscription::try_pop(frame_ptr &out) {
  if (mode_ == deliver_mode::ring && ring_)
    return ring_->try_pop(out);
  return false;
}

std::uint64_t subscription::dropped() const {
  if (mode_ == deliver_mode::ring)
    return ring_ ? ring_->total_dropped_oldest() : 0;
  return overwrites_.load(std::memory_order_relaxed);
}

void subscription::close() {
  if (ring_)
    ring_->close(); // unblocks a parked pop()
  // latest() never blocks, so a latest-mode subscription needs no wake.
}

// --------------------------------------------------------------------------
// stream_channel
// --------------------------------------------------------------------------

stream_channel::stream_channel(app &ctx, std::string id, format_desc fmt,
                               std::shared_ptr<frame_pool> pool, std::size_t producer_working_set,
                               std::size_t in_flight_per_subscriber)
    : ctx_(ctx), id_(std::move(id)), fmt_(std::move(fmt)), pool_(std::move(pool)),
      producer_working_set_(producer_working_set),
      in_flight_per_subscriber_(in_flight_per_subscriber) {}

stream_channel::~stream_channel() {
  close(); // idempotent; wakes any parked ring consumer even on a bare-destruct path
}

std::size_t stream_channel::slots_for(deliver_mode m, std::size_t depth) const noexcept {
  // A ring pins up to `depth` queued + `in_flight` being drained; a latest
  // register pins 1 current + `in_flight` being rendered.
  const std::size_t held = (m == deliver_mode::ring) ? depth : 1;
  return held + in_flight_per_subscriber_;
}

std::shared_ptr<subscription> stream_channel::subscribe(deliver_mode m, std::size_t depth) {
  std::lock_guard<std::mutex> lk(subs_mu_);
  const std::size_t sc = pool_->slab_count();
  const std::size_t avail = sc > producer_working_set_ ? sc - producer_working_set_ : 0;
  const std::size_t need = slots_for(m, depth);
  if (committed_slots_ + need > avail) {
    // Refuse rather than under-provision: admitting this subscriber would let a
    // slow consumer pin enough slabs to starve the producer (§3.2 invariant).
    return nullptr;
  }
  auto s = std::make_shared<subscription>(m, depth);
  committed_slots_ += need;
  subs_.push_back(s);
  return s;
}

void stream_channel::unsubscribe(const std::shared_ptr<subscription> &s) {
  if (!s)
    return;
  std::shared_ptr<subscription> found;
  {
    std::lock_guard<std::mutex> lk(subs_mu_);
    for (auto it = subs_.begin(); it != subs_.end(); ++it) {
      if (*it == s) {
        committed_slots_ -= slots_for((*it)->mode(), (*it)->depth());
        found = *it;
        subs_.erase(it);
        break;
      }
    }
  }
  if (found)
    found->close(); // outside the lock: only touches the subscription's own state
}

void stream_channel::fan_out(const frame_ptr &f) {
  // Snapshot the subscription handles under the lock, then deliver OFF the lock:
  // the shared_ptr copies keep a concurrently-unsubscribing subscription alive
  // through delivery, and delivery never blocks on subs_mu_.
  std::vector<std::shared_ptr<subscription>> snapshot;
  {
    std::lock_guard<std::mutex> lk(subs_mu_);
    snapshot = subs_;
  }
  for (const auto &s : snapshot)
    s->deliver(f);
}

std::int64_t stream_channel::publish(const frame_pool::lease &l, std::size_t used, double pts) {
  const std::int64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
  frame_ptr f = pool_->publish(l, used, seq, pts, fmt_);
  fan_out(f);
  return seq;
}

std::int64_t stream_channel::publish_external(const std::uint8_t *data, std::size_t size,
                                              double pts, std::shared_ptr<void> keepalive) {
  const std::int64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
  // Build a frame that borrows the caller's storage directly (no pool slab). The
  // keepalive co-owns `data` for the frame's whole life, so a subscriber that
  // still holds the frame_ptr keeps the caller's buffer alive.
  auto f = std::make_shared<frame>();
  f->data = data;
  f->size = size;
  f->seq = seq;
  f->pts_seconds = pts;
  f->format = fmt_;
  f->keepalive = std::move(keepalive);
  fan_out(frame_ptr(std::move(f)));
  return seq;
}

std::size_t stream_channel::subscriber_count() const {
  std::lock_guard<std::mutex> lk(subs_mu_);
  return subs_.size();
}

void stream_channel::close() {
  if (closed_.exchange(true))
    return;
  std::vector<std::shared_ptr<subscription>> snapshot;
  {
    std::lock_guard<std::mutex> lk(subs_mu_);
    snapshot = subs_;
  }
  for (const auto &s : snapshot)
    s->close();
}

} // namespace stream
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_STREAM_CHANNEL_H__
#define __CVC_ARIADNE_STREAM_STREAM_CHANNEL_H__

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/frame_pool.h>
#include <cvc/core/state_bounded_queue.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// stream_channel — the per-stream hub that fans one producer's frames out to N
// independent subscribers (roadmap STATE_BINARY_STREAMING.md §3.3).
//
// Delivery is per-subscriber and bounded, so a slow consumer cannot stall the
// producer or grow memory. There are two delivery modes, and they use DIFFERENT
// primitives because state_bounded_queue has no non-destructive read:
//   - deliver_mode::latest (video display): a lock-free single-slot register
//     holding the freshest frame. latest() is NON-destructive, so a renderer
//     re-samples the current frame every rendered frame; superseded frames are
//     simply overwritten.
//   - deliver_mode::ring (audio/sensor): a state_bounded_queue<frame_ptr> with
//     overflow_policy::drop_oldest, drained in order by pop()/try_pop().
//
// Because a frame_ptr is a shared_ptr, every deliver/push/drop/pop is a refcount
// op, never a byte copy.

namespace cvc {

class app;

namespace ariadne {
namespace stream {

enum class deliver_mode { latest, ring };

// One consumer's view of a stream. Held via shared_ptr so a mid-teardown
// unsubscribe cannot free it under a concurrent publish() fan-out.
class subscription {
public:
  subscription(deliver_mode m, std::size_t depth);

  // Producer side: hand this subscriber the newest frame (refcount bump only).
  void deliver(const frame_ptr &f);

  // deliver_mode::latest — the freshest frame, or null before the first
  // delivery. Non-destructive: repeated calls return the same frame until a
  // newer one is delivered.
  frame_ptr latest() const;

  // deliver_mode::ring — drain in FIFO order. pop() blocks until an item is
  // available or the subscription is closed (then returns false); try_pop() is
  // non-blocking. Both return false for a latest-mode subscription.
  bool pop(frame_ptr &out);
  bool try_pop(frame_ptr &out);

  // Frames this subscriber missed: ring = oldest-dropped; latest = superseded.
  std::uint64_t dropped() const;

  deliver_mode mode() const noexcept { return mode_; }
  std::size_t depth() const noexcept { return depth_; }

  // Unblock a parked pop() and reject further deliveries' side effects.
  void close();

private:
  const deliver_mode mode_;
  const std::size_t depth_;
  std::shared_ptr<const frame> latest_; // mode::latest; published via std::atomic_store
  std::unique_ptr<state_bounded_queue<frame_ptr>> ring_; // mode::ring
  std::atomic<std::uint64_t> overwrites_{0};
};

class stream_channel {
public:
  // `producer_working_set` and `in_flight_per_subscriber` are the slots reserved
  // (against the pool) for the producer's own working set and for the one frame
  // a consumer holds while draining/rendering; subscribe() enforces the §3.2
  // invariant against pool->slab_count() using them.
  stream_channel(app &ctx, std::string id, format_desc fmt, std::shared_ptr<frame_pool> pool,
                 std::size_t producer_working_set = 2, std::size_t in_flight_per_subscriber = 1);
  ~stream_channel();

  stream_channel(const stream_channel &) = delete;
  stream_channel &operator=(const stream_channel &) = delete;

  // Add a subscriber. Returns null (and logs) when the fixed pool cannot cover
  // another subscriber of this depth without violating the §3.2 invariant —
  // Phase-1 policy is refuse-rather-than-under-provision (grow-on-subscribe is a
  // deferred follow-up).
  std::shared_ptr<subscription> subscribe(deliver_mode m, std::size_t depth = 3);
  void unsubscribe(const std::shared_ptr<subscription> &s);

  // Producer thread: stamp the next seq, freeze `used` bytes of the filled slab
  // into a frame, and fan it out to every subscriber. Returns the stamped seq.
  std::int64_t publish(const frame_pool::lease &l, std::size_t used, double pts);

  // Producer thread: publish a frame that ALIASES caller-owned storage with no
  // pool slab. `keepalive` must own `data` (treated as read-only and immutable
  // after this call) for at least as long as any subscriber holds the frame;
  // `size` is the valid bytes at `data` (typically format().frame_bytes()). Use
  // this for a producer that already owns its buffers — e.g. a numpy array or a
  // decoder's output — so the stream borrows them zero-copy instead of copying
  // into a pool slab. Stamps the next seq (shared with the pool path, so seq is
  // monotonic across both producer paths) and returns it. Thread-safe for a
  // single producer thread; the caller must not mutate `data` after publishing.
  std::int64_t publish_external(const std::uint8_t *data, std::size_t size, double pts,
                                std::shared_ptr<void> keepalive);

  frame_pool &pool() noexcept { return *pool_; }
  const std::shared_ptr<frame_pool> &pool_ptr() const noexcept { return pool_; }
  const std::string &id() const noexcept { return id_; }
  const format_desc &format() const noexcept { return fmt_; }
  std::size_t subscriber_count() const;
  // The most recently stamped seq, or -1 before the first publish().
  std::int64_t last_seq() const noexcept { return seq_.load(std::memory_order_relaxed) - 1; }
  std::uint64_t total_published() const noexcept {
    return static_cast<std::uint64_t>(seq_.load(std::memory_order_relaxed));
  }

  // Idempotent. Closes every subscription (unblocking parked ring consumers).
  void close();

private:
  // Slots a subscription of (mode, depth) consumes against the pool.
  std::size_t slots_for(deliver_mode m, std::size_t depth) const noexcept;
  // Snapshot the subscriber set under the lock, then deliver off the lock (the
  // shared_ptr copies keep a concurrently-unsubscribing subscription alive).
  void fan_out(const frame_ptr &f);

  app &ctx_;
  const std::string id_;
  const format_desc fmt_;
  const std::shared_ptr<frame_pool> pool_;
  const std::size_t producer_working_set_;
  const std::size_t in_flight_per_subscriber_;
  std::atomic<std::int64_t> seq_{0};
  mutable std::mutex subs_mu_;
  std::vector<std::shared_ptr<subscription>> subs_;
  std::size_t committed_slots_ = 0; // running sum of admitted subscriptions' slots
  std::atomic<bool> closed_{false};
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_CHANNEL_H__

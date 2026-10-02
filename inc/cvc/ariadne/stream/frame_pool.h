/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_FRAME_POOL_H__
#define __CVC_ARIADNE_STREAM_FRAME_POOL_H__

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

// frame_pool — a fixed set of reusable byte slabs for zero-steady-state-alloc
// streaming (roadmap STATE_BINARY_STREAMING.md §3.2).
//
// The producer acquire()s a free slab, fills it, and publish()es it into an
// immutable frame_ptr whose keepalive returns the slab to the free-list when
// the LAST consumer drops it. In steady state there is zero malloc and zero
// copy on the borrow path.
//
// Sizing invariant (§3.2, LOAD-BEARING): the pool must be provisioned to
//   Sum over subscribers of (queue depth + frames in flight) + producer working set
// A pool that is too small re-couples producer and consumers. This class does
// NOT hide that under a stall: acquire() is NON-BLOCKING and returns nullopt
// when every slab is pinned, so the producer DROPS AT SOURCE (bumping
// total_borrow_fail()) instead of blocking. stream::open sizes the pool to the
// invariant and stream_channel refuses a subscription that would exceed it.
//
// Lifetime: the pool is held via shared_ptr (create()); each published frame's
// keepalive captures shared_from_this(), so the pool outlives every frame it
// minted — a channel can close and drop the pool while consumers still hold
// frames, with no use-after-free. The pool holds only FREE slab indices (never
// frame_ptrs), so there is no ownership cycle.

namespace cvc {
namespace ariadne {
namespace stream {

class frame_pool : public std::enable_shared_from_this<frame_pool> {
public:
  // A checked-out slab: writable storage the producer fills before publish().
  struct lease {
    std::size_t idx = 0;
    std::uint8_t *data = nullptr;
    std::size_t cap = 0;
  };

  // slab_count is clamped to >= 1 and slab_bytes to >= 1 so a degenerate size
  // never yields a zero-capacity buffer.
  static std::shared_ptr<frame_pool> create(std::size_t slab_bytes, std::size_t slab_count);
  ~frame_pool();

  frame_pool(const frame_pool &) = delete;
  frame_pool &operator=(const frame_pool &) = delete;

  // Non-blocking, noexcept. Returns a free slab, or nullopt when all slabs are
  // pinned (borrow-fail => the producer must drop this frame at source, NEVER
  // block). Bumps total_borrow_fail() on nullopt.
  std::optional<lease> acquire() noexcept;

  // Freeze the filled slab (`used` bytes) into an immutable frame_ptr. The
  // frame's keepalive returns the slab to the free-list on the LAST consumer
  // drop. `used` is clamped to the slab capacity.
  frame_ptr publish(const lease &l, std::size_t used, std::int64_t seq, double pts,
                    const format_desc &fmt);

  // Return a slab to the free-list WITHOUT publishing it (producer decided not
  // to emit this frame after acquiring). Safe to call at most once per lease.
  void discard(const lease &l) noexcept;

  std::size_t slab_bytes() const noexcept { return slab_bytes_; }
  std::size_t slab_count() const noexcept { return slab_count_; }
  std::size_t in_use() const noexcept; // pinned (checked-out or borrowed) slabs

  std::uint64_t total_borrow_fail() const noexcept {
    return borrow_fail_.load(std::memory_order_relaxed);
  }
  std::uint64_t total_published() const noexcept {
    return published_.load(std::memory_order_relaxed);
  }

private:
  frame_pool(std::size_t slab_bytes, std::size_t slab_count);
  void release(std::size_t idx) noexcept; // called from the keepalive deleter

  const std::size_t slab_bytes_;
  const std::size_t slab_count_;
  mutable std::mutex mu_;
  // Stable backing storage: `slabs_` is sized once and never reallocated, and
  // each inner buffer is fixed-capacity, so a lease's data pointer stays valid
  // for the slab's whole life.
  std::vector<std::vector<std::uint8_t>> slabs_;
  std::vector<std::size_t> free_; // free-list of slab indices
  std::atomic<std::uint64_t> borrow_fail_{0};
  std::atomic<std::uint64_t> published_{0};
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_FRAME_POOL_H__

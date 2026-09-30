/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/stream/frame_pool.h>

namespace cvc {
namespace stream {

std::shared_ptr<frame_pool> frame_pool::create(std::size_t slab_bytes, std::size_t slab_count) {
  // Cannot std::make_shared through a private ctor; new + shared_ptr keeps the
  // ctor private while still enabling shared_from_this().
  return std::shared_ptr<frame_pool>(new frame_pool(slab_bytes, slab_count));
}

frame_pool::frame_pool(std::size_t slab_bytes, std::size_t slab_count)
    : slab_bytes_(slab_bytes == 0 ? 1 : slab_bytes), slab_count_(slab_count == 0 ? 1 : slab_count) {
  slabs_.resize(slab_count_);
  free_.reserve(slab_count_);
  for (std::size_t i = 0; i < slab_count_; ++i) {
    slabs_[i].resize(slab_bytes_); // fixed-capacity, never reallocated afterwards
    free_.push_back(i);
  }
}

std::optional<frame_pool::lease> frame_pool::acquire() noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  if (free_.empty()) {
    // Every slab is pinned: drop-at-source, never block. The producer must skip
    // this frame; the counter surfaces under-provisioning against the invariant.
    borrow_fail_.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }
  const std::size_t idx = free_.back();
  free_.pop_back();
  lease l;
  l.idx = idx;
  l.data = slabs_[idx].data();
  l.cap = slab_bytes_;
  return l;
}

frame_ptr frame_pool::publish(const lease &l, std::size_t used, std::int64_t seq, double pts,
                              const format_desc &fmt) {
  const std::size_t n = used > slab_bytes_ ? slab_bytes_ : used;
  auto self = shared_from_this();
  // The keepalive owns the slab: when the last frame_ptr borrowing it drops,
  // the deleter returns idx to the free-list. The captured `self` keeps the
  // pool alive at least as long as any frame it minted.
  std::shared_ptr<void> keep(static_cast<void *>(l.data),
                             [self, idx = l.idx](void *) noexcept { self->release(idx); });
  auto f = std::make_shared<frame>();
  f->data = l.data;
  f->size = n;
  f->seq = seq;
  f->pts_seconds = pts;
  f->format = fmt;
  f->keepalive = std::move(keep);
  published_.fetch_add(1, std::memory_order_relaxed);
  return frame_ptr(std::move(f));
}

void frame_pool::discard(const lease &l) noexcept { release(l.idx); }

void frame_pool::release(std::size_t idx) noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  free_.push_back(idx);
}

std::size_t frame_pool::in_use() const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  return slab_count_ - free_.size();
}

} // namespace stream
} // namespace cvc

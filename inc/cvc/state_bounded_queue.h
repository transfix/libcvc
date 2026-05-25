/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_BOUNDED_QUEUE_H__
#define __CVC_STATE_BOUNDED_QUEUE_H__

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cvc/namespace.h>
#include <deque>
#include <mutex>
#include <utility>

namespace cvc {

// ----------------
// cvc::state_bounded_queue<T>
// ----------------
// Phase 6 (Performance & Production Hardening).
//
// Thread-safe bounded queue with selectable overflow policy. The
// distributed-state transports use it as a per-peer outbound buffer
// so a slow consumer cannot stall the producer (drop-newest), lose
// the freshest update (drop-oldest), or evade rate limits without
// the producer noticing (block).
//
// Policies:
//   drop_newest  - if the queue is full when push() is called, the
//                  incoming item is rejected. push() returns false
//                  and total_dropped_newest() bumps. Use when older
//                  pending items still matter (FIFO ordering).
//
//   drop_oldest  - if full, the front item is evicted and the new
//                  item is appended. push() returns true and
//                  total_dropped_oldest() bumps. Use when only the
//                  freshest update matters (state snapshots, latest-
//                  value-only paths).
//
//   block        - push() blocks until space is available or the
//                  caller-supplied timeout elapses. On timeout,
//                  push_for() returns false and
//                  total_blocked_timeouts() bumps. push() with no
//                  timeout blocks indefinitely.
//
// Threading:
//   All public methods are safe to call concurrently. pop() may
//   block waiting for an item; close() unblocks all waiters and
//   makes subsequent push() calls fail.
//
// Closure:
//   close() drains no items (callers may still drain via pop()
//   until the queue is empty); it just signals "no more pushes
//   coming". After close() the queue rejects pushes and pop()
//   returns false once empty.
//
template <class T> class state_bounded_queue {
public:
  enum class overflow_policy { drop_newest, drop_oldest, block };

  explicit state_bounded_queue(std::size_t capacity,
                               overflow_policy policy = overflow_policy::drop_newest)
      : _capacity(capacity == 0 ? 1 : capacity), _policy(policy) {}

  state_bounded_queue(const state_bounded_queue &) = delete;
  state_bounded_queue &operator=(const state_bounded_queue &) = delete;

  // Try to push. Semantics depend on the policy. When the queue is
  // closed, returns false and bumps total_rejected_closed().
  bool push(T value) {
    std::unique_lock<std::mutex> lk(_mu);
    if (_closed) {
      _rejected_closed.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    if (_q.size() >= _capacity) {
      switch (_policy) {
      case overflow_policy::drop_newest:
        _dropped_newest.fetch_add(1, std::memory_order_relaxed);
        return false;
      case overflow_policy::drop_oldest:
        _q.pop_front();
        _dropped_oldest.fetch_add(1, std::memory_order_relaxed);
        break;
      case overflow_policy::block:
        _not_full.wait(lk, [this]() { return _closed || _q.size() < _capacity; });
        if (_closed) {
          _rejected_closed.fetch_add(1, std::memory_order_relaxed);
          return false;
        }
        break;
      }
    }
    _q.push_back(std::move(value));
    _admitted.fetch_add(1, std::memory_order_relaxed);
    _not_empty.notify_one();
    return true;
  }

  // Push with a wait timeout. Only meaningful for the block policy;
  // for the drop_* policies it behaves identically to push() (no
  // wait happens). Returns false on timeout or close.
  template <class Rep, class Period>
  bool push_for(T value, const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock<std::mutex> lk(_mu);
    if (_closed) {
      _rejected_closed.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    if (_q.size() >= _capacity) {
      switch (_policy) {
      case overflow_policy::drop_newest:
        _dropped_newest.fetch_add(1, std::memory_order_relaxed);
        return false;
      case overflow_policy::drop_oldest:
        _q.pop_front();
        _dropped_oldest.fetch_add(1, std::memory_order_relaxed);
        break;
      case overflow_policy::block:
        if (!_not_full.wait_for(lk, timeout,
                                [this]() { return _closed || _q.size() < _capacity; })) {
          _blocked_timeouts.fetch_add(1, std::memory_order_relaxed);
          return false;
        }
        if (_closed) {
          _rejected_closed.fetch_add(1, std::memory_order_relaxed);
          return false;
        }
        break;
      }
    }
    _q.push_back(std::move(value));
    _admitted.fetch_add(1, std::memory_order_relaxed);
    _not_empty.notify_one();
    return true;
  }

  // Block until an item is available or the queue is closed and
  // empty. Returns true with `out` populated, false when the queue
  // is closed and drained.
  bool pop(T &out) {
    std::unique_lock<std::mutex> lk(_mu);
    _not_empty.wait(lk, [this]() { return !_q.empty() || _closed; });
    if (_q.empty())
      return false;
    out = std::move(_q.front());
    _q.pop_front();
    _popped.fetch_add(1, std::memory_order_relaxed);
    _not_full.notify_one();
    return true;
  }

  // Non-blocking pop. Returns false if the queue is empty.
  bool try_pop(T &out) {
    std::lock_guard<std::mutex> lk(_mu);
    if (_q.empty())
      return false;
    out = std::move(_q.front());
    _q.pop_front();
    _popped.fetch_add(1, std::memory_order_relaxed);
    _not_full.notify_one();
    return true;
  }

  void close() {
    {
      std::lock_guard<std::mutex> lk(_mu);
      _closed = true;
    }
    _not_empty.notify_all();
    _not_full.notify_all();
  }

  bool closed() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _closed;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _q.size();
  }

  std::size_t capacity() const noexcept { return _capacity; }
  overflow_policy policy() const noexcept { return _policy; }

  std::uint64_t total_admitted() const noexcept {
    return _admitted.load(std::memory_order_relaxed);
  }
  std::uint64_t total_popped() const noexcept { return _popped.load(std::memory_order_relaxed); }
  std::uint64_t total_dropped_newest() const noexcept {
    return _dropped_newest.load(std::memory_order_relaxed);
  }
  std::uint64_t total_dropped_oldest() const noexcept {
    return _dropped_oldest.load(std::memory_order_relaxed);
  }
  std::uint64_t total_blocked_timeouts() const noexcept {
    return _blocked_timeouts.load(std::memory_order_relaxed);
  }
  std::uint64_t total_rejected_closed() const noexcept {
    return _rejected_closed.load(std::memory_order_relaxed);
  }

private:
  mutable std::mutex _mu;
  std::condition_variable _not_full;
  std::condition_variable _not_empty;
  std::deque<T> _q;
  const std::size_t _capacity;
  const overflow_policy _policy;
  bool _closed = false;
  std::atomic<std::uint64_t> _admitted{0};
  std::atomic<std::uint64_t> _popped{0};
  std::atomic<std::uint64_t> _dropped_newest{0};
  std::atomic<std::uint64_t> _dropped_oldest{0};
  std::atomic<std::uint64_t> _blocked_timeouts{0};
  std::atomic<std::uint64_t> _rejected_closed{0};
};

} // namespace cvc

#endif // __CVC_STATE_BOUNDED_QUEUE_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_HYBRID_TIME_H__
#define __CVC_STATE_HYBRID_TIME_H__

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <mutex>

namespace cvc {

// ----------------
// cvc::hybrid_time
// ----------------
// Hybrid Logical Clock (HLC) timestamp combining physical wall
// time with a logical counter to provide a monotonically
// increasing, causally consistent ordering across nodes.
//
// Encoding: 48 bits of wall_ms (milliseconds since epoch) in the
// upper bits, 16 bits of logical counter in the lower bits.
// Packed into a single uint64_t for wire efficiency.
//
// Causal ordering:
//   send/local event: now() advances the clock.
//   receive: update(remote_hlc) merges the remote timestamp,
//            ensuring this node's clock never goes backwards.
//
struct hybrid_time {
  std::uint64_t wall_ms = 0; // milliseconds since Unix epoch
  std::uint16_t logical = 0; // logical counter

  // Pack into a 64-bit value: upper 48 bits wall, lower 16 bits logical.
  std::uint64_t packed() const noexcept {
    return (wall_ms << 16) | static_cast<std::uint64_t>(logical);
  }

  // Unpack from a 64-bit value.
  static hybrid_time from_packed(std::uint64_t v) noexcept {
    return {v >> 16, static_cast<std::uint16_t>(v & 0xFFFF)};
  }

  bool operator<(const hybrid_time &o) const noexcept { return packed() < o.packed(); }
  bool operator>(const hybrid_time &o) const noexcept { return packed() > o.packed(); }
  bool operator==(const hybrid_time &o) const noexcept { return packed() == o.packed(); }
  bool operator!=(const hybrid_time &o) const noexcept { return packed() != o.packed(); }
  bool operator<=(const hybrid_time &o) const noexcept { return packed() <= o.packed(); }
  bool operator>=(const hybrid_time &o) const noexcept { return packed() >= o.packed(); }

  // Returns true if this time is non-zero (has been assigned).
  explicit operator bool() const noexcept { return packed() != 0; }
};

// ----------------
// cvc::hybrid_clock
// ----------------
// Thread-safe HLC clock. Each node in the cluster keeps one
// instance. Call now() before sending a mutation, and update()
// when receiving a remote mutation's timestamp.
//
class hybrid_clock {
public:
  // Issue a new timestamp for a local event.
  hybrid_time now() noexcept {
    std::lock_guard<std::mutex> lk(_mu);
    auto phys = wall_ms();
    if (phys > _last.wall_ms) {
      _last.wall_ms = phys;
      _last.logical = 0;
    } else {
      ++_last.logical;
    }
    return _last;
  }

  // Merge a remote timestamp (receive event).
  hybrid_time update(hybrid_time remote) noexcept {
    std::lock_guard<std::mutex> lk(_mu);
    auto phys = wall_ms();
    if (phys > _last.wall_ms && phys > remote.wall_ms) {
      _last.wall_ms = phys;
      _last.logical = 0;
    } else if (_last.wall_ms == remote.wall_ms) {
      _last.logical = std::max(_last.logical, remote.logical) + 1;
    } else if (_last.wall_ms > remote.wall_ms) {
      ++_last.logical;
    } else {
      _last.wall_ms = remote.wall_ms;
      _last.logical = remote.logical + 1;
    }
    return _last;
  }

  // Current clock value without advancing.
  hybrid_time current() const noexcept {
    std::lock_guard<std::mutex> lk(_mu);
    return _last;
  }

private:
  static std::uint64_t wall_ms() noexcept {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
  }

  mutable std::mutex _mu;
  hybrid_time _last{};
};

} // namespace cvc

#endif // __CVC_STATE_HYBRID_TIME_H__

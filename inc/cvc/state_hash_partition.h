/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_HASH_PARTITION_H__
#define __CVC_STATE_HASH_PARTITION_H__

#include <algorithm>
#include <cstdint>
#include <cvc/namespace.h>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_hash_partition
// ----------------
// Consistent-hash partition map for routing state paths to nodes
// within a cluster. Each node owns a contiguous range on the hash
// ring [range_begin, range_end). A path's owner is found by
// hashing the path and locating the first range whose end exceeds
// the hash value (sorted ring lookup).
//
// Thread-safe: all methods are mutex-guarded.
//
class state_hash_partition {
public:
  // A contiguous slice of the hash ring.
  struct range {
    std::string node_id;
    std::uint32_t range_begin = 0;
    std::uint32_t range_end = 0; // exclusive
  };

  state_hash_partition() = default;

  // Assign ownership of [begin, end) to `node_id`. Overlapping
  // ranges are not checked — the caller must ensure consistency.
  void assign(const std::string &node_id, std::uint32_t begin, std::uint32_t end) {
    std::lock_guard<std::mutex> lk(_mu);
    _ranges.push_back({node_id, begin, end});
    std::sort(_ranges.begin(), _ranges.end(),
              [](const range &a, const range &b) { return a.range_begin < b.range_begin; });
  }

  // Uniformly partition the full 32-bit hash space among the given
  // node IDs. Replaces any existing assignment.
  void assign_uniform(const std::vector<std::string> &node_ids) {
    std::lock_guard<std::mutex> lk(_mu);
    _ranges.clear();
    if (node_ids.empty())
      return;
    std::uint64_t total = static_cast<std::uint64_t>(UINT32_MAX) + 1;
    std::uint64_t slice = total / node_ids.size();
    std::uint32_t lo = 0;
    for (std::size_t i = 0; i < node_ids.size(); ++i) {
      std::uint32_t hi =
          (i + 1 == node_ids.size()) ? UINT32_MAX : static_cast<std::uint32_t>(lo + slice - 1);
      _ranges.push_back({node_ids[i], lo, hi + 1});
      lo = hi + 1;
    }
  }

  // Resolve the owner node for a given path. Returns empty string
  // if no range covers the hash.
  std::string owner_of(const std::string &path) const {
    std::uint32_t h = hash(path);
    std::lock_guard<std::mutex> lk(_mu);
    for (const auto &r : _ranges) {
      // range_end == 0 means the range wraps the full 32-bit space
      // (UINT32_MAX + 1 overflowed to 0). Treat as "covers everything
      // from range_begin onward".
      if (r.range_end == 0) {
        if (h >= r.range_begin)
          return r.node_id;
      } else {
        if (h >= r.range_begin && h < r.range_end)
          return r.node_id;
      }
    }
    return {};
  }

  // Check if `node_id` owns the given path.
  bool owns(const std::string &node_id, const std::string &path) const {
    return owner_of(path) == node_id;
  }

  // Current snapshot of all ranges.
  std::vector<range> snapshot() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _ranges;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _ranges.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lk(_mu);
    _ranges.clear();
  }

  // FNV-1a hash (32-bit) of a path string.
  static std::uint32_t hash(const std::string &path) noexcept {
    std::uint32_t h = 2166136261u;
    for (unsigned char c : path) {
      h ^= c;
      h *= 16777619u;
    }
    return h;
  }

private:
  mutable std::mutex _mu;
  std::vector<range> _ranges;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_HASH_PARTITION_H__

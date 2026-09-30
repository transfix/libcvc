/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/stream/stream_registry.h>
#include <memory>

namespace cvc {
namespace stream {

namespace {

// Per-app registries, owned here for the life of the process (mirrors the
// per-app singleton pattern of state_cluster_shard::default_for). A handful of
// long-lived apps, so the map stays tiny; unique_ptr frees them at exit.
std::mutex &registries_mutex() {
  static std::mutex m;
  return m;
}

std::unordered_map<const app *, std::unique_ptr<stream_registry>> &registries() {
  static std::unordered_map<const app *, std::unique_ptr<stream_registry>> m;
  return m;
}

} // namespace

stream_registry &stream_registry::for_app(const app &ctx) {
  std::lock_guard<std::mutex> lk(registries_mutex());
  auto &slot = registries()[&ctx];
  if (!slot)
    slot.reset(new stream_registry());
  return *slot;
}

bool stream_registry::install(const std::string &token, stream_channel *ch) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = by_token_.find(token);
  if (it != by_token_.end())
    return false; // first-writer-wins
  by_token_.emplace(token, ch);
  return true;
}

void stream_registry::uninstall(const std::string &token, stream_channel *ch) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = by_token_.find(token);
  if (it != by_token_.end() && it->second == ch)
    by_token_.erase(it);
}

stream_channel *stream_registry::lookup(const std::string &token) const {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = by_token_.find(token);
  return it == by_token_.end() ? nullptr : it->second;
}

std::size_t stream_registry::size() const {
  std::lock_guard<std::mutex> lk(mu_);
  return by_token_.size();
}

} // namespace stream
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/core/app.h>
#include <memory>
#include <mutex>

namespace cvc {
namespace ariadne {
namespace stream {

namespace {

// An app's registry, held in its slot: it goes with the app, so a later app (even one made at the
// same address) starts with none of its tokens.
struct registry_slot {
  static constexpr const char *app_slot = "cvc.ariadne.stream-registry";
  std::mutex m;
  std::unique_ptr<stream_registry> registry;
};

} // namespace

stream_registry &stream_registry::for_app(app &ctx) {
  registry_slot &s = ctx.slot<registry_slot>();
  std::lock_guard<std::mutex> lk(s.m);
  if (!s.registry)
    s.registry.reset(new stream_registry());
  return *s.registry;
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
} // namespace ariadne
} // namespace cvc

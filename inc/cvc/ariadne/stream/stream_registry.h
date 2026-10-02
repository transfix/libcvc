/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_STREAM_REGISTRY_H__
#define __CVC_ARIADNE_STREAM_STREAM_REGISTRY_H__

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>

// stream_registry — a process-local, per-app side-table mapping a stream's
// handle token to its live stream_channel (roadmap STATE_BINARY_STREAMING.md §4).
//
// The /streams/<id> descriptor node in cvc::state carries the replicated
// metadata; the live channel object is NEVER serialized. A consumer resolves a
// token from the descriptor to the in-process channel through this table.
//
// The mapped pointer is raw and NON-OWNING: it is valid only while the channel
// is alive. The channel's owner installs the token on announce and uninstalls
// it BEFORE the channel is destroyed (erase-before-destruct), so lookup() can
// never hand back a dangling pointer. Install is first-writer-wins so the
// negotiating->live handshake for a token cannot be hijacked.

namespace cvc {

class app;

namespace ariadne {
namespace stream {

class stream_channel;

class stream_registry {
public:
  // The per-app registry (created on first use, lives for the process).
  static stream_registry &for_app(const app &ctx);

  stream_registry(const stream_registry &) = delete;
  stream_registry &operator=(const stream_registry &) = delete;

  // Returns false if `token` is already live (first-writer-wins).
  bool install(const std::string &token, stream_channel *ch);
  // Erases `token` only if it currently maps to `ch` (safe against a token that
  // was already reinstalled by a successor).
  void uninstall(const std::string &token, stream_channel *ch);
  // The live channel for `token`, or nullptr.
  stream_channel *lookup(const std::string &token) const;
  std::size_t size() const;

private:
  stream_registry() = default;
  mutable std::mutex mu_;
  std::unordered_map<std::string, stream_channel *> by_token_;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_REGISTRY_H__

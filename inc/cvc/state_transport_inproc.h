/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_INPROC_H__
#define __CVC_STATE_TRANSPORT_INPROC_H__

#include <cvc/namespace.h>
#include <cvc/state_transport.h>

#include <atomic>
#include <mutex>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_transport_inproc
// ----------------
// In-memory transport. Delivery is synchronous: publish() iterates
// over registered shards in the same cluster (excluding origin)
// and calls ingest_remote on each. Useful for unit tests and any
// scenario where a single process hosts multiple shards.
//
class state_transport_inproc final : public state_transport {
public:
  state_transport_inproc() = default;
  ~state_transport_inproc() override = default;

  state_transport_inproc(const state_transport_inproc &) = delete;
  state_transport_inproc &operator=(const state_transport_inproc &) = delete;

  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;

  publish_stats publish(const state_mutation &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;

  // Diagnostics.
  std::size_t shard_count() const;
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }

private:
  mutable std::mutex _mutex;
  std::vector<state_cluster_shard *> _shards;
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _delivered{0};
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TRANSPORT_INPROC_H__

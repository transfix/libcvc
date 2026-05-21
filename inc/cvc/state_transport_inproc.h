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
#include <cvc/state_bounded_queue.h>
#include <cvc/state_transport.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_transport_inproc
// ----------------
// In-memory transport. Delivery is synchronous by default:
// publish() iterates over registered shards in the same cluster
// (excluding origin) and calls ingest_remote on each.
//
// Per-peer message outbox (Phase 6 backpressure):
//   set_peer_message_outbox(peer, capacity, policy) installs a
//   bounded queue between publish_message() and the peer's
//   ingest_remote_message(). When configured, publish_message()
//   pushes into the outbox and returns immediately; the message is
//   actually delivered when deliver_message_outbox(peer) is called.
//   Without a configured outbox, publish_message() retains its
//   original synchronous semantics.
//
//   This lets tests model a slow consumer and observe drop_newest /
//   drop_oldest / block-timeout behavior. It is also the building
//   block that the IPC and gRPC transports will share for slow-peer
//   isolation.
//
class state_transport_inproc final : public state_transport {
public:
  using outbox_policy = state_bounded_queue<state_message>::overflow_policy;

  state_transport_inproc() = default;
  ~state_transport_inproc() override;

  state_transport_inproc(const state_transport_inproc &) = delete;
  state_transport_inproc &operator=(const state_transport_inproc &) = delete;

  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;

  publish_stats publish(const state_mutation &m) override;
  publish_message_stats publish_message(const state_message &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;

  // Install a bounded message outbox for `peer`. `capacity` is the
  // maximum number of pending messages; `policy` selects the
  // overflow behavior. Replaces any prior configuration. Pass
  // capacity=0 (or call clear_peer_message_outbox) to revert to
  // synchronous delivery.
  void set_peer_message_outbox(state_cluster_shard *peer,
                               std::size_t capacity,
                               outbox_policy policy = outbox_policy::drop_newest);

  // Remove an installed outbox; subsequent publish_message() calls
  // for this peer go back to synchronous delivery.
  void clear_peer_message_outbox(state_cluster_shard *peer);

  // Number of messages currently pending in the peer's outbox. 0
  // if no outbox is installed.
  std::size_t peer_message_outbox_size(state_cluster_shard *peer) const;

  // Drain up to `max` messages from the peer's outbox by calling
  // ingest_remote_message on the peer. Returns the number of
  // messages actually delivered (excludes ones the peer treated as
  // duplicates or filtered). max=0 means "drain until empty".
  std::size_t deliver_message_outbox(state_cluster_shard *peer,
                                     std::size_t max = 0);

  // Aggregated outbox counters across all installed outboxes.
  std::uint64_t total_outbox_admitted() const noexcept {
    return _outbox_admitted.load();
  }
  std::uint64_t total_outbox_dropped_newest() const noexcept {
    return _outbox_dropped_newest.load();
  }
  std::uint64_t total_outbox_dropped_oldest() const noexcept {
    return _outbox_dropped_oldest.load();
  }
  std::uint64_t total_outbox_blocked_timeouts() const noexcept {
    return _outbox_blocked_timeouts.load();
  }

  // Diagnostics.
  std::size_t shard_count() const;
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }
  std::uint64_t total_messages_published() const noexcept {
    return _msg_published.load();
  }
  std::uint64_t total_messages_delivered() const noexcept {
    return _msg_delivered.load();
  }

private:
  struct peer_outbox {
    std::unique_ptr<state_bounded_queue<state_message>> queue;
    outbox_policy policy = outbox_policy::drop_newest;
  };

  // Returns a pointer to the peer's outbox if one is installed,
  // else nullptr. Caller must hold _mutex when reading the map.
  peer_outbox *find_outbox_locked(state_cluster_shard *peer);

  mutable std::mutex _mutex;
  std::vector<state_cluster_shard *> _shards;
  std::unordered_map<state_cluster_shard *, std::unique_ptr<peer_outbox>>
      _outboxes;
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _delivered{0};
  std::atomic<std::uint64_t> _msg_published{0};
  std::atomic<std::uint64_t> _msg_delivered{0};
  std::atomic<std::uint64_t> _outbox_admitted{0};
  std::atomic<std::uint64_t> _outbox_dropped_newest{0};
  std::atomic<std::uint64_t> _outbox_dropped_oldest{0};
  std::atomic<std::uint64_t> _outbox_blocked_timeouts{0};
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TRANSPORT_INPROC_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_INPROC_H__
#define __CVC_STATE_TRANSPORT_INPROC_H__

#include <atomic>
#include <cvc/namespace.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_bounded_queue.h>
#include <cvc/state_transport.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

  // Chunk fetch: look for the digest in each registered shard's
  // associated blob store (if the shard exposes one). For inproc,
  // this is a synchronous local lookup.
  bool fetch_chunk(const std::string &digest, chunk_callback on_chunk) override;
  std::size_t fetch_chunks(const std::vector<std::string> &digests,
                           chunk_callback on_chunk) override;

  // Install a bounded message outbox for `peer`. `capacity` is the
  // maximum number of pending messages; `policy` selects the
  // overflow behavior. Replaces any prior configuration. Pass
  // capacity=0 (or call clear_peer_message_outbox) to revert to
  // synchronous delivery.
  void set_peer_message_outbox(state_cluster_shard *peer, std::size_t capacity,
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
  std::size_t deliver_message_outbox(state_cluster_shard *peer, std::size_t max = 0);

  // Aggregated outbox counters across all installed outboxes.
  std::uint64_t total_outbox_admitted() const noexcept { return _outbox_admitted.load(); }
  std::uint64_t total_outbox_dropped_newest() const noexcept {
    return _outbox_dropped_newest.load();
  }
  std::uint64_t total_outbox_dropped_oldest() const noexcept {
    return _outbox_dropped_oldest.load();
  }
  std::uint64_t total_outbox_blocked_timeouts() const noexcept {
    return _outbox_blocked_timeouts.load();
  }

  // ----------------
  // Slow-peer isolation (Phase 6 backpressure, bullet 4)
  // ----------------
  // Mark `peer` as slow: subsequent publish()/publish_message()
  // calls skip this peer entirely (no outbox enqueue, no
  // synchronous ingest) so a stalled consumer cannot back up
  // healthy peers. Skipped deliveries are counted under
  // total_quarantined_*. The peer remains skipped until
  // clear_peer_slow() is called or the auto-isolation threshold
  // releases it (it does not, by design — release is manual).
  //
  // Quarantined peers continue to accept replays via
  // deliver_message_outbox(); operators are expected to drain or
  // discard the outbox before clearing the slow flag.
  void mark_peer_slow(state_cluster_shard *peer);
  void clear_peer_slow(state_cluster_shard *peer);
  bool is_peer_slow(state_cluster_shard *peer) const;
  std::vector<state_cluster_shard *> slow_peers() const;

  // When > 0, an outbox whose cumulative drops (drop_newest +
  // drop_oldest, since outbox install) reach this value is
  // auto-marked slow on the publish_message() call that caused
  // the threshold to be crossed. Default 0 = disabled (manual
  // mark only). Setting to 0 disables auto-isolation but does not
  // clear peers already marked slow.
  void set_auto_isolation_drop_threshold(std::uint64_t threshold) noexcept;
  std::uint64_t auto_isolation_drop_threshold() const noexcept {
    return _auto_isolation_threshold.load();
  }

  std::uint64_t total_quarantined_messages() const noexcept { return _quarantined_messages.load(); }
  std::uint64_t total_quarantined_mutations() const noexcept {
    return _quarantined_mutations.load();
  }
  std::uint64_t total_auto_isolations() const noexcept { return _auto_isolations.load(); }

  // Diagnostics.
  std::size_t shard_count() const;
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }
  std::uint64_t total_messages_published() const noexcept { return _msg_published.load(); }
  std::uint64_t total_messages_delivered() const noexcept { return _msg_delivered.load(); }

  // Set a blob store that fetch_chunk() searches when looking for
  // chunks requested by peers. Without this, fetch_chunk returns
  // false (no blobs available).
  void set_blob_store(state_blob_store *store) noexcept { _blob_store = store; }
  state_blob_store *blob_store() const noexcept { return _blob_store; }

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
  std::unordered_map<state_cluster_shard *, std::unique_ptr<peer_outbox>> _outboxes;
  std::unordered_set<state_cluster_shard *> _slow_peers;
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _delivered{0};
  std::atomic<std::uint64_t> _msg_published{0};
  std::atomic<std::uint64_t> _msg_delivered{0};
  std::atomic<std::uint64_t> _outbox_admitted{0};
  std::atomic<std::uint64_t> _outbox_dropped_newest{0};
  std::atomic<std::uint64_t> _outbox_dropped_oldest{0};
  std::atomic<std::uint64_t> _outbox_blocked_timeouts{0};
  std::atomic<std::uint64_t> _quarantined_messages{0};
  std::atomic<std::uint64_t> _quarantined_mutations{0};
  std::atomic<std::uint64_t> _auto_isolations{0};
  std::atomic<std::uint64_t> _auto_isolation_threshold{0};
  state_blob_store *_blob_store = nullptr;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TRANSPORT_INPROC_H__

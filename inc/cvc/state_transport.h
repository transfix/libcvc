/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_H__
#define __CVC_STATE_TRANSPORT_H__

#include <cvc/namespace.h>
#include <cvc/state_change_journal.h>
#include <cvc/state_message.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace CVC_NAMESPACE {

class state_cluster_shard;

// ----------------
// cvc::state_transport
// ----------------
// Purpose:
//   Abstract carrier between cluster shards. Concrete transports
//   (inproc, ipc, grpc) implement the same operations:
//     register_shard / unregister_shard
//     publish(mutation)
//     pump_shard(shard) -> drain that shard's local journal and
//                          publish the results
//     pump_all()        -> pump every registered shard
//     flush()           -> block until pending deliveries complete
//
//   Loop suppression is the receiving shard's responsibility: the
//   transport delivers every mutation to every peer shard sharing
//   the same cluster_id (excluding the originator); the shard's
//   replica seen-set drops duplicates.
//
//   Same-cluster routing only. Cross-cluster delegation is handled
//   above this layer by the authority map.
//
// Threading:
//   Implementations must be safe to call from multiple threads.
//
class state_transport {
public:
  struct publish_stats {
    std::size_t delivered = 0;     // peer shards that accepted (applied or duplicate)
    std::size_t duplicates = 0;    // peer shards that reported duplicate
    std::size_t rejected = 0;      // peer shards that rejected (authority)
  };

  // Statistics returned by publish_message().
  struct publish_message_stats {
    std::size_t delivered = 0;   // local shards that admitted the message
    std::size_t duplicates = 0;  // local shards that reported duplicate
    std::size_t peers = 0;       // remote peer streams the message was sent on
  };

  virtual ~state_transport() = default;

  // Register a shard so it can both publish and receive.
  virtual void register_shard(state_cluster_shard *shard) = 0;

  // Unregister; safe to call from the shard's destructor.
  virtual void unregister_shard(state_cluster_shard *shard) = 0;

  // Best-effort publish to other shards in the same cluster_id.
  virtual publish_stats publish(const state_mutation &m) = 0;

  // Best-effort publish of an out-of-band message. Default is a
  // no-op so transports that have not yet implemented the message
  // path remain compilable; concrete transports should override.
  // The same loop-suppression convention as publish() applies:
  // origin shards are skipped on local fan-out, and the receiving
  // shard's message bus dedups by (origin_node_id, message_id).
  virtual publish_message_stats publish_message(const state_message &m) {
    (void)m;
    return {};
  }

  // Convenience: drain the shard's local-origin journal entries and
  // publish each one. Returns the number of mutations published.
  virtual std::size_t pump_shard(state_cluster_shard &shard) = 0;

  // Pump every registered shard until none has pending local work
  // (one pass; caller may loop until 0 returned for full quiescence).
  virtual std::size_t pump_all() = 0;

  // Block until any in-flight asynchronous deliveries complete.
  // No-op for synchronous transports.
  virtual void flush() = 0;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TRANSPORT_H__

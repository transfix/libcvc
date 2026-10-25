/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_H__
#define __CVC_STATE_TRANSPORT_H__

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <cvc/state/state_change_journal.h>
#include <cvc/state/state_link_stats.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_peer_registry.h>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace cvc {

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
    std::size_t delivered = 0;  // peer shards that accepted (applied or duplicate)
    std::size_t duplicates = 0; // peer shards that reported duplicate
    std::size_t rejected = 0;   // peer shards that rejected (authority)
  };

  // Statistics returned by publish_message().
  struct publish_message_stats {
    std::size_t delivered = 0;  // local shards that admitted the message
    std::size_t duplicates = 0; // local shards that reported duplicate
    std::size_t peers = 0;      // remote peer streams the message was sent on
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

  // Phase 5: per-peer subscription routing. Each concrete transport
  // consults this registry before fanning out a publish() or
  // publish_message() to a peer; if the peer is registered with a
  // non-empty subscription set and the path is not covered, the
  // delivery is filtered (counted via
  // state_peer_registry::note_delivery_filtered).
  // Peers not present in the registry receive everything
  // (back-compat default).
  state_peer_registry &peers() noexcept { return _peers; }
  const state_peer_registry &peers() const noexcept { return _peers; }

  // -------- Link quality (Ping/Pong probes, per-connection traffic) --------
  //
  // One report per live connection: bytes and frames both ways, and the round-trip time the
  // transport's Ping/Pong probe measures (state_link_stats). A relay sees one link per client; a
  // client sees its link to the relay; a server also sees its links to other servers. Transports
  // without connections report none (the default).
  virtual std::vector<state_link_report> link_reports() const { return {}; }

  // How often each connection is pinged (default 2 s; 0 turns the probe off). The probe also keeps
  // a quiet-but-connected peer from looking idle to a relay's reaper.
  void set_ping_interval(std::chrono::milliseconds interval) noexcept {
    _ping_interval_ms.store(interval.count(), std::memory_order_relaxed);
  }
  std::chrono::milliseconds ping_interval() const noexcept {
    return std::chrono::milliseconds(_ping_interval_ms.load(std::memory_order_relaxed));
  }

  // -------- Chunk fetch (volume/geometry streaming) --------
  //
  // Callback-based chunk retrieval from remote peers. The
  // transport asks registered remote peers for the chunk; the
  // first peer that has it delivers the bytes to `on_chunk`.
  //
  // The default implementation is a no-op (returns false) so
  // existing transports continue to compile until they implement
  // the path. Concrete transports that support multi-host blob
  // transfer override this.
  //
  // `on_chunk(digest, bytes)` is called at most once; if no peer
  // has the chunk the call returns false.
  using chunk_callback =
      std::function<void(const std::string &digest, const std::vector<unsigned char> &bytes)>;

  virtual bool fetch_chunk(const std::string &digest, chunk_callback on_chunk) {
    (void)digest;
    (void)on_chunk;
    return false;
  }

  // Batch variant: fetch all chunks whose digests are in
  // `digests`. `on_chunk` is called once per successfully
  // fetched chunk. Returns the number of chunks fetched.
  virtual std::size_t fetch_chunks(const std::vector<std::string> &digests,
                                   chunk_callback on_chunk) {
    std::size_t n = 0;
    for (const auto &d : digests)
      if (fetch_chunk(d, on_chunk))
        ++n;
    return n;
  }

  // -------- Initial-sync snapshot protocol --------
  //
  // A snapshot entry represents one node's state.
  struct snapshot_entry {
    std::string path;
    std::string string_value;
    std::string comment;
    bool hidden = false;
    bool read_only = false;
    std::map<std::string, std::string> properties; // state::properties()
    std::string type_name;
    std::string origin_node_id;
    std::uint64_t sequence = 0;
    // data() payload (Phase 10, PR4): the node's data() encoded exactly as the live set_data path
    // (codec-encoded, offloaded to a blob_ref when large). Empty data_type_name = the node has no
    // replicable data(). Lets a late joiner reconstruct data(), not just value() + metadata.
    std::string data_type_name;
    state_payload data_payload;
  };

  // Callback invoked with batches of snapshot entries. `final` is
  // true on the last invocation.
  using snapshot_callback =
      std::function<void(const std::vector<snapshot_entry> &entries, bool final)>;

  // Request a snapshot of the remote peer's state tree rooted at
  // `path_prefix` (empty = whole tree). The entries are delivered
  // asynchronously via `on_entries`.
  //
  // Default returns false (not supported).
  virtual bool request_snapshot(const std::string &cluster_id, const std::string &path_prefix,
                                snapshot_callback on_entries) {
    (void)cluster_id;
    (void)path_prefix;
    (void)on_entries;
    return false;
  }

  // Targeted push: write `entries` as ONE unsolicited, final snapshot frame (request_id 0) to
  // every live connection whose peer identified (Hello) as `node_id`, bypassing the peer
  // registry -- the caller already decided what this peer gets. Clients apply an unsolicited
  // snapshot like a reply. A relay uses it to backfill or refresh exactly the leaves a delivery
  // filter withholds from the live stream. Returns the number of connections written; 0 (before
  // encoding anything) when no connection has that id.
  //
  // Called on the same thread as publish(), so per-connection FIFO orders a push ahead of the
  // publishes that follow it. Default: not supported (returns 0).
  virtual std::size_t push_snapshot_to(const std::string &node_id,
                                       const std::vector<snapshot_entry> &entries) {
    (void)node_id;
    (void)entries;
    return 0;
  }

  // -------- Transport bridge (wasm interop) --------
  //
  // Link this transport to a sibling of a DIFFERENT backend — e.g. a gRPC transport and a
  // WebSocket transport running in ONE relay over the same shards. A mutation/message RECEIVED
  // from a connection is, after the normal same-transport relay, also handed to the bridged peer,
  // which fans it out to ITS connections. That is how a browser (WebSocket) client and a native
  // (gRPC) client end up in one room: each backend only knows its own connections, and the bridge
  // carries frames across the two connection sets.
  //
  // The forward uses the peer's publish()/publish_message() (concrete transports call
  // bridge()->publish(m) from on_inbound_*). Those do NOT re-enter on_inbound, so no relay loop
  // forms; and because origin_node_id is preserved, the receiving shard's dedup makes the redundant
  // apply a no-op. Null (the default) = not bridged — a leaf client applies locally and never
  // forwards.
  void set_bridge(state_transport *peer) noexcept { _bridge = peer; }
  state_transport *bridge() const noexcept { return _bridge; }

protected:
  state_peer_registry _peers;
  state_transport *_bridge = nullptr;
  std::atomic<std::int64_t> _ping_interval_ms{2000};
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_H__

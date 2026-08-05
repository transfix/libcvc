/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_IPC_H__
#define __CVC_STATE_TRANSPORT_IPC_H__

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cvc/core/namespace.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/core/state_transport.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_transport_ipc
// ----------------
// Same-host transport over Unix domain sockets. Each instance binds
// one UDS listener (server role) and may dial any number of peer
// listeners (client role). Connections are full-duplex; a single
// socket carries traffic in both directions.
//
// Wire envelope (little-endian):
//   magic    u32 = 0x43564354 ("CVCT")
//   version  u16 = 1
//   msg_type u16 (1 = HELLO, 2 = MUTATION)
//   length   u32 (payload bytes that follow)
//   payload  length bytes
//
// HELLO payload:
//   string node_id
//   string cluster_id
//
// MUTATION payload: full state_mutation (see state_transport_ipc.cpp
// for the encoder). All strings are u32-length-prefixed.
//
// Receiver dispatch mirrors state_transport_inproc: every registered
// shard whose cluster_id matches the mutation and whose node id is
// not the origin gets ingest_remote called on it. Loop suppression
// is the shard replica's seen-set.
//
// Threading:
//   start(): binds listener, spawns one acceptor thread.
//   connect_to_peer(): blocking connect, spawns one reader thread.
//   publish(): serializes once, then writes synchronously to each
//              connection under its write mutex. flush() is a no-op
//              because writes are synchronous on success.
//
// Catch-up:
//   publish() is fire-and-forget to whichever connections exist at
//   that instant, while pump_shard() advances the shard's publish
//   cursor either way. A mutation pumped before any peer connected
//   is therefore never retried. To keep late joiners convergent,
//   every new connection is sent the registered shards' journaled
//   local mutations before it sees live traffic — see
//   set_backfill_on_connect(). Two nodes that each dial the other
//   hold two connections and so exchange the backfill twice; the
//   receiving replica's seen-set makes the repeat a no-op.
//
// Lifetime:
//   Reader threads call ingest_remote on registered shards. Callers
//   MUST call stop() (or unregister the shard) before destroying any
//   registered state_cluster_shard, otherwise an in-flight inbound
//   frame can dispatch to a destroyed shard. The destructor calls
//   stop() automatically, so simply ensuring the transport outlives
//   its shards (declare it earlier in scope) is enough.
//
class state_transport_ipc final : public state_transport {
public:
  struct connection;

  state_transport_ipc();
  ~state_transport_ipc() override;

  state_transport_ipc(const state_transport_ipc &) = delete;
  state_transport_ipc &operator=(const state_transport_ipc &) = delete;

  // Bind a listening UDS at `path` (unlinked first if it exists)
  // and start the acceptor thread. Throws std::runtime_error on
  // failure. `node_id` and `cluster_id` populate the HELLO frame
  // sent to peers on connect; they are advisory and used only for
  // diagnostics. Empty values are allowed.
  void start(const std::string &path, const std::string &node_id = std::string(),
             const std::string &cluster_id = std::string());

  // Connect to a peer's listening UDS. Sends HELLO. Returns true on
  // success. Spawns a reader thread for the connection.
  bool connect_to_peer(const std::string &path,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

  // Stop the acceptor, close all connections, join threads, and
  // unlink the listener path.
  void stop();

  // Blob store for chunk fetch servicing.
  void set_blob_store(state_blob_store *store) noexcept { _blob_store = store; }
  state_blob_store *blob_store() const noexcept { return _blob_store; }

  // Backfill on connect (default true). When enabled, a newly
  // established connection is sent every local-origin mutation each
  // registered shard has journaled, before any live traffic, so a
  // peer that connects after local writes still converges. Without
  // it a mutation pumped while no peer was connected is dropped by
  // publish() and never resent — see pump_shard().
  //
  // Turn it off only when peers are known to obtain their initial
  // view another way (request_snapshot(), or a peer that is always
  // connected before the first write); the cost is one extra frame
  // per journaled mutation per connect.
  void set_backfill_on_connect(bool enable) noexcept { _backfill_on_connect = enable; }
  bool backfill_on_connect() const noexcept { return _backfill_on_connect; }

  // state_transport interface.
  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;
  publish_stats publish(const state_mutation &m) override;
  publish_message_stats publish_message(const state_message &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;
  bool fetch_chunk(const std::string &digest, chunk_callback on_chunk) override;
  bool request_snapshot(const std::string &cluster_id, const std::string &path_prefix,
                        snapshot_callback on_entries) override;

  // Diagnostics.
  std::size_t shard_count() const;
  std::size_t connection_count() const;
  std::uint64_t total_backfilled() const noexcept { return _backfilled.load(); }
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_sent_frames() const noexcept { return _sent_frames.load(); }
  std::uint64_t total_received_frames() const noexcept { return _recv_frames.load(); }
  std::uint64_t total_received_mutations() const noexcept { return _recv_mutations.load(); }
  std::uint64_t total_received_messages() const noexcept { return _recv_messages.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }

  // Wait until at least `target` MUTATION frames have been received
  // (HELLO frames are not counted) or `timeout` elapses. Used by
  // tests to bridge the asynchronous receive path. Returns the
  // current total_received_mutations() value.
  std::uint64_t wait_for_received(std::uint64_t target, std::chrono::milliseconds timeout);

  // Wait until at least `target` OOB MESSAGE frames have been
  // received or `timeout` elapses.
  std::uint64_t wait_for_received_messages(std::uint64_t target, std::chrono::milliseconds timeout);

  // Test hook: called from reader threads when a MUTATION is
  // dispatched. Only used internally by connection::reader_loop.
  void dispatch_inbound(const state_mutation &m);

  // Test hook: called from reader threads when an OOB message is
  // dispatched.
  void dispatch_inbound_message(const state_message &m);

private:
  void accept_loop();
  void reader_loop(std::shared_ptr<connection> conn);
  // Register `conn`, then send HELLO and (if enabled) the backfill,
  // all while holding conn->write_mu so that a publish() racing on
  // another thread cannot slip a newer mutation in ahead of the
  // older ones being replayed.
  void admit_connection(const std::shared_ptr<connection> &conn);
  void send_hello_locked(connection &c);
  void send_backfill_locked(connection &c, const std::vector<state_cluster_shard *> &shards);
  bool write_frame_locked(connection &c, std::uint16_t msg_type,
                          const std::vector<unsigned char> &payload);

  std::string _listen_path;
  std::string _node_id;
  std::string _cluster_id;
  int _listen_fd = -1;
  std::atomic<bool> _running{false};
  std::thread _accept_thread;

  mutable std::mutex _shards_mu;
  std::vector<state_cluster_shard *> _shards;

  mutable std::mutex _conns_mu;
  std::vector<std::shared_ptr<connection>> _conns;

  state_blob_store *_blob_store = nullptr;
  std::atomic<bool> _backfill_on_connect{true};

  std::atomic<std::uint64_t> _backfilled{0};
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _recv_messages{0};
  std::atomic<std::uint64_t> _delivered{0};

  // Chunk fetch request/response tracking.
  std::atomic<std::uint64_t> _next_chunk_req_id{1};
  mutable std::mutex _chunk_waiters_mu;
  std::condition_variable _chunk_waiters_cv;
  struct chunk_waiter {
    bool done = false;
    bool found = false;
    std::vector<unsigned char> data;
  };
  std::unordered_map<std::uint64_t, std::shared_ptr<chunk_waiter>> _chunk_waiters;

  // Snapshot request/response tracking.
  std::atomic<std::uint64_t> _next_snap_req_id{1};
  mutable std::mutex _snap_waiters_mu;
  std::condition_variable _snap_waiters_cv;
  struct snap_waiter {
    bool done = false;
    std::vector<snapshot_entry> entries;
  };
  std::unordered_map<std::uint64_t, std::shared_ptr<snap_waiter>> _snap_waiters;
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_IPC_H__

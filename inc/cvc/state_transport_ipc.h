/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_IPC_H__
#define __CVC_STATE_TRANSPORT_IPC_H__

#include <cvc/namespace.h>
#include <cvc/state_transport.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace CVC_NAMESPACE {

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
  void start(const std::string &path,
             const std::string &node_id = std::string(),
             const std::string &cluster_id = std::string());

  // Connect to a peer's listening UDS. Sends HELLO. Returns true on
  // success. Spawns a reader thread for the connection.
  bool connect_to_peer(const std::string &path,
                       std::chrono::milliseconds timeout =
                           std::chrono::milliseconds(2000));

  // Stop the acceptor, close all connections, join threads, and
  // unlink the listener path.
  void stop();

  // state_transport interface.
  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;
  publish_stats publish(const state_mutation &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;

  // Diagnostics.
  std::size_t shard_count() const;
  std::size_t connection_count() const;
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_sent_frames() const noexcept { return _sent_frames.load(); }
  std::uint64_t total_received_frames() const noexcept {
    return _recv_frames.load();
  }
  std::uint64_t total_received_mutations() const noexcept {
    return _recv_mutations.load();
  }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }

  // Wait until at least `target` MUTATION frames have been received
  // (HELLO frames are not counted) or `timeout` elapses. Used by
  // tests to bridge the asynchronous receive path. Returns the
  // current total_received_mutations() value.
  std::uint64_t wait_for_received(std::uint64_t target,
                                  std::chrono::milliseconds timeout);

  // Test hook: called from reader threads when a MUTATION is
  // dispatched. Only used internally by connection::reader_loop.
  void dispatch_inbound(const state_mutation &m);

private:
  void accept_loop();
  void reader_loop(std::shared_ptr<connection> conn);
  void send_hello(connection &c);
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

  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _delivered{0};
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_TRANSPORT_IPC_H__

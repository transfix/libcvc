/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_GRPC_H__
#define __CVC_STATE_TRANSPORT_GRPC_H__

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
// cvc::state_transport_grpc
// ----------------
// Cross-host transport using gRPC bidirectional streaming. Each
// instance can act as both server (acceptor) and client (dialer);
// peers establish one long-lived stream per pairing and push Frame
// messages in either direction. See proto/state_transport.proto for
// the wire schema.
//
// Built only when libcvc is configured with -DCVC_ENABLE_GRPC=ON.
// Requires Protobuf and gRPC (provided by libcvc-deps v1.1.0 or by
// system packages libprotobuf-dev / libgrpc++-dev / protobuf-
// compiler-grpc).
//
// Threading:
//   start(): builds and starts a grpc::Server bound to the given
//   address. The server runs in gRPC's own threadpool; one server
//   handler thread runs per accepted stream.
//   connect_to_peer(): creates a client channel and bidirectional
//   stream, sends Hello, spawns one reader thread that drains the
//   client side of the stream.
//   publish(): serializes the mutation once, then under each per-
//   stream write mutex calls Write(frame). flush() is a no-op
//   because writes are synchronous on success.
//   stop(): server->Shutdown(), TryCancel on every client context,
//   join client reader threads. Server handler threads are joined
//   inside Shutdown.
//
// Catch-up:
//   publish() is fire-and-forget to whichever streams exist at that
//   instant, while pump_shard() advances the shard's publish cursor
//   either way. A mutation pumped before any peer connected is
//   therefore never retried. To keep late joiners convergent, every
//   new stream is sent the registered shards' journaled local
//   mutations before it sees live traffic — see
//   set_backfill_on_connect(). Two nodes that each dial the other
//   hold two streams and so exchange the backfill twice; the
//   receiving replica's seen-set makes the repeat a no-op.
//
// Lifetime:
//   Same constraint as state_transport_ipc: callers MUST stop() (or
//   destroy) the transport before destroying any registered
//   state_cluster_shard, otherwise an in-flight inbound frame can
//   dispatch to a freed shard.
//
class state_transport_grpc final : public state_transport {
public:
  // Phase 5: optional transport-level TLS configuration. When set
  // before start() / connect_to_peer(), the server uses
  // SslServerCredentials and the client uses SslCredentials. PEM
  // bytes are passed verbatim to gRPC. An empty server_cert_pem +
  // server_key_pem disables server-side TLS even if the struct is
  // installed. require_client_auth=true switches the server to
  // mutual TLS (client certificate must chain to root_ca_pem).
  struct tls_config {
    std::string server_cert_pem;
    std::string server_key_pem;
    std::string root_ca_pem;
    bool require_client_auth = false;
  };

  // Phase 5: optional bearer-token authentication. When
  // expected_token is set, the server rejects any incoming
  // Channel() RPC whose "authorization" metadata does not equal
  // "Bearer <expected_token>". When outbound_token is set, the
  // client stamps the same metadata on its ClientContext.
  struct auth_config {
    std::string expected_token;
    std::string outbound_token;
  };

  state_transport_grpc();
  ~state_transport_grpc() override;

  state_transport_grpc(const state_transport_grpc &) = delete;
  state_transport_grpc &operator=(const state_transport_grpc &) = delete;

  // Phase 5: must be called BEFORE start() / connect_to_peer().
  void set_tls_config(tls_config cfg);
  void set_auth_config(auth_config cfg);

  // Bind a gRPC server listening at `listen_addr`. The address is in
  // gRPC's URI form ("host:port"), e.g. "127.0.0.1:0" for an
  // ephemeral port. Throws std::runtime_error on failure. node_id /
  // cluster_id populate the Hello frame sent on each new stream and
  // are advisory.
  void start(const std::string &listen_addr, const std::string &node_id = std::string(),
             const std::string &cluster_id = std::string());

  // The actual listen address, with the bound port resolved (useful
  // when the caller passed "host:0").
  std::string listen_address() const;

  // Dial `target` ("host:port") and open a bidirectional stream.
  // Sends Hello on success. Returns true on success; spawns a
  // client-side reader thread.
  bool connect_to_peer(const std::string &target,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

  // Shut down server, cancel all client streams, join reader threads.
  void stop();

  // Heartbeat interval. When non-zero, a background thread sends
  // Heartbeat frames to all connections at this cadence. Default 0
  // (disabled).
  void set_heartbeat_interval(std::chrono::milliseconds interval) noexcept;
  std::chrono::milliseconds heartbeat_interval() const noexcept;

  // Blob store for servicing inbound chunk requests and for local
  // lookup before sending outbound requests.
  void set_blob_store(state_blob_store *store) noexcept { _blob_store = store; }
  state_blob_store *blob_store() const noexcept { return _blob_store; }

  // Backfill on connect (default true). When enabled, a newly
  // established stream is sent every local-origin mutation each
  // registered shard has journaled, before any live traffic, so a
  // peer that connects after local writes still converges. Without
  // it a mutation pumped while no peer was connected is dropped by
  // publish() and never resent -- see pump_shard().
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
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_sent_frames() const noexcept { return _sent_frames.load(); }
  std::uint64_t total_received_frames() const noexcept { return _recv_frames.load(); }
  std::uint64_t total_received_mutations() const noexcept { return _recv_mutations.load(); }
  std::uint64_t total_received_messages() const noexcept { return _recv_messages.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }
  std::uint64_t total_backfilled() const noexcept { return _backfilled.load(); }

  // Wait until at least `target` MUTATION frames have been received
  // (Hello frames are not counted) or `timeout` elapses.
  std::uint64_t wait_for_received(std::uint64_t target, std::chrono::milliseconds timeout);

  // Wait until at least `target` OOB message frames have been
  // received or `timeout` elapses.
  std::uint64_t wait_for_received_messages(std::uint64_t target, std::chrono::milliseconds timeout);

  // Internal API used by the server-side RPC handler and the client
  // reader thread. Public so the impl translation unit (which is the
  // only one that knows about the generated proto types) can call it.
  // Not part of the stable user-facing API.
  class connection;
  const std::string &local_node_id() const noexcept { return _node_id; }
  const std::string &local_cluster_id() const noexcept { return _cluster_id; }
  const auth_config &auth() const noexcept { return _auth; }
  void on_inbound_mutation(const state_mutation &m);
  void on_inbound_message(const state_message &m);
  // Chunk fetch request/response dispatch (called from connection
  // reader threads when a ChunkRequest/ChunkResponse frame arrives).
  void on_inbound_chunk_request(connection *conn, const std::string &digest,
                                std::uint64_t request_id);
  void on_inbound_chunk_response(std::uint64_t request_id, bool found,
                                 std::vector<unsigned char> data);
  void on_inbound_snapshot_request(connection *conn, const std::string &cluster_id,
                                   const std::string &path_prefix, std::uint64_t request_id);
  void on_inbound_snapshot_response(std::uint64_t request_id,
                                    const std::vector<snapshot_entry> &entries, bool final);
  void on_inbound_heartbeat(const std::string &node_id, const std::string &cluster_id);
  void register_connection(std::shared_ptr<connection> conn);
  // Register `conn`, then send Hello and (if enabled) the backfill,
  // all while holding conn->write_mu so that a publish() racing on
  // another thread cannot slip a newer mutation in ahead of the
  // older ones being replayed.
  void admit_connection(const std::shared_ptr<connection> &conn);
  void unregister_connection(connection *conn);
  void increment_recv_frames() noexcept { _recv_frames.fetch_add(1, std::memory_order_relaxed); }
  void increment_recv_mutations() noexcept {
    _recv_mutations.fetch_add(1, std::memory_order_relaxed);
  }
  void increment_recv_messages() noexcept {
    _recv_messages.fetch_add(1, std::memory_order_relaxed);
  }

private:
  void send_hello_locked(connection &c);
  void send_backfill_locked(connection &c, const std::vector<state_cluster_shard *> &shards);

  struct impl;
  std::unique_ptr<impl> _impl;

  std::string _node_id;
  std::string _cluster_id;
  std::atomic<bool> _running{false};

  tls_config _tls;
  bool _tls_set = false;
  auth_config _auth;

  mutable std::mutex _shards_mu;
  std::vector<state_cluster_shard *> _shards;

  mutable std::mutex _conns_mu;
  std::vector<std::shared_ptr<connection>> _conns;

  std::atomic<bool> _backfill_on_connect{true};

  std::atomic<std::uint64_t> _backfilled{0};
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _recv_messages{0};
  std::atomic<std::uint64_t> _delivered{0};

  state_blob_store *_blob_store = nullptr;

  // Chunk fetch waiter infrastructure.
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

  // Heartbeat sender.
  std::chrono::milliseconds _heartbeat_interval{0};
  std::thread _heartbeat_thread;
  void heartbeat_loop();
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_GRPC_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_DISTRIBUTED_STATE_SESSION_H__
#define __CVC_DISTRIBUTED_STATE_SESSION_H__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_compression_registry.h>
#include <cvc/state_data_hydrator.h>
#include <cvc/state_distributed_admin.h>
#include <cvc/state_transport.h>
#include <cvc/state_transport_inproc.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace CVC_NAMESPACE {

class app;

// ----------------
// Sync modes for per-path mount points.
// ----------------
enum class sync_mode {
  read_only,     // mirror remote writes, reject local writes
  read_write,    // bidirectional replication
  authoritative, // this node is the authority for the prefix
  delegated      // prefix is delegated to a foreign cluster
};

// ----------------
// A single mount point: path prefix + sync mode.
// ----------------
struct distributed_state_mount {
  std::string path;
  sync_mode mode = sync_mode::read_write;
};

// ----------------
// Target for delegation: remote cluster id + endpoint.
// ----------------
struct delegation_target {
  std::string cluster_id;
  std::string endpoint;
  std::uint64_t lease_duration_ns = 0; // 0 = infinite
};

// ----------------
// Transport selection.
// ----------------
enum class transport_kind {
  inproc, // in-process (testing / single-process multi-tree)
  ipc,    // unix domain sockets (same-host multi-process)
  grpc    // gRPC (networked)
};

// ----------------
// Configuration for a distributed state session.
// ----------------
struct distributed_state_config {
  std::string cluster_id;
  std::string node_id;
  std::string root_path; // subtree to replicate ("" = whole tree)

  transport_kind transport = transport_kind::inproc;
  std::string listen_address;     // for ipc/grpc: socket path or host:port
  std::vector<std::string> seeds; // peer endpoints to connect to
  std::vector<distributed_state_mount> mounts;

  bool enforce_authority = false;
  bool enforce_write_policy = false;
  bool enforce_delegation = false;
  bool resolve_conflicts = false;
  bool enforce_interest = false;

  // TLS / auth (gRPC only — ignored for inproc / ipc).
  std::string tls_server_cert_pem;
  std::string tls_server_key_pem;
  std::string tls_root_ca_pem;
  bool tls_require_client_auth = false;
  bool require_tls = false; // when true, session throws if TLS certs are missing
  std::string auth_expected_token;
  std::string auth_outbound_token;

  // Tuning.
  std::uint32_t max_inline_payload_bytes = 65536;
  std::string blob_store_path;   // empty = memory-only blob store
  bool snapshot_on_join = false; // request full snapshot from first seed on join

  std::uint32_t pump_interval_ms = 10; // background pump loop interval (0 = no pump thread)
};

// ----------------
// Snapshot of current replica health.
// ----------------
struct replica_status {
  bool running = false;
  std::size_t peer_count = 0;
  std::uint64_t local_sequence = 0;
  std::uint64_t pump_cycles = 0;
  std::uint64_t pending_hydrations = 0;
};

// ----------------
// cvc::distributed_state_session
// ----------------
// Purpose:
//   Convenience wrapper that assembles and wires together all the
//   distributed state components (shard, transport, blob store,
//   compression, admin) from a single config struct. Provides a
//   high-level API for joining a cluster, managing sync paths, and
//   delegating subtrees.
//
// Lifecycle:
//   1. join(app, config) creates the session, starts the transport,
//      connects to seeds, and begins background pumping.
//   2. sync_path() / delegate() / undelegate() adjust replication.
//   3. stop() tears everything down in the correct order.
//   4. Destructor calls stop() if not already called.
//
// Threading:
//   All public methods are thread-safe.
//
class distributed_state_session {
public:
  ~distributed_state_session();

  distributed_state_session(const distributed_state_session &) = delete;
  distributed_state_session &operator=(const distributed_state_session &) = delete;

  // Create and start a session.
  static std::shared_ptr<distributed_state_session> join(app &ctx,
                                                         const distributed_state_config &config);

  // Graceful shutdown: detach shard, stop transport, join pump thread.
  void stop();

  // Whether the session has been stopped.
  bool is_running() const noexcept { return _running.load(std::memory_order_acquire); }

  // Add a sync path at runtime.
  void sync_path(const std::string &path, sync_mode mode = sync_mode::read_write);

  // Delegate a subtree to a remote cluster.
  void delegate(const std::string &path, const delegation_target &target);

  // Revoke a delegation.
  void undelegate(const std::string &path);

  // Access underlying components for advanced usage.
  state_cluster_shard &shard() noexcept { return *_shard; }
  state_transport &transport() noexcept { return *_transport; }
  state_blob_store &blob_store() noexcept { return *_blob_store; }
  state_distributed_admin &admin() noexcept { return *_admin; }
  state_data_hydrator &hydrator() noexcept { return *_hydrator; }

  // Wait until the blob at `path` has been hydrated, or timeout
  // expires. Returns the hydration status.
  state_data_hydrator::hydration_status
  wait_for_data(const std::string &path,
                std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

  // Get a snapshot of the replica health.
  replica_status status() const;

  // Diagnostics.
  const std::string &cluster_id() const noexcept { return _config.cluster_id; }
  const std::string &node_id() const noexcept { return _config.node_id; }
  std::uint64_t pump_cycles() const noexcept {
    return _pump_cycles.load(std::memory_order_relaxed);
  }

private:
  explicit distributed_state_session(const distributed_state_config &config);

  void pump_loop();

  distributed_state_config _config;

  std::unique_ptr<state_cluster_shard> _shard;
  std::unique_ptr<state_transport> _transport;
  std::unique_ptr<memory_state_blob_store> _blob_store;
  std::unique_ptr<state_distributed_admin> _admin;
  std::unique_ptr<state_data_hydrator> _hydrator;

  app *_app_ctx = nullptr;

  std::thread _pump_thread;
  std::atomic<bool> _running{false};
  std::atomic<std::uint64_t> _pump_cycles{0};
};

} // namespace CVC_NAMESPACE

#endif // __CVC_DISTRIBUTED_STATE_SESSION_H__

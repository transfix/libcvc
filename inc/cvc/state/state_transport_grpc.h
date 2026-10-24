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
#include <cvc/media/media_endpoint.h> // media_frame + the media_router plug-in surface
#include <cvc/state/state_blob_store.h>
#include <cvc/state/state_transport.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace transport {
namespace v1 {
class Frame; // the generated envelope (proto/state_transport.proto), for on_link_frame
} // namespace v1
} // namespace transport

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
class state_transport_grpc final : public state_transport, public media_endpoint {
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

  // RELAY mode (star topology): when enabled, a mutation or message frame RECEIVED from one
  // connection is also forwarded to every OTHER connection, so clients that only connect to this
  // (server) node still see each other's room-tree writes. Off by default — a leaf node applies a
  // received frame to its local shards and never re-sends it. Enable only on a relay/server; a
  // forwarded frame keeps its origin_node_id, and the origin connection is skipped (no echo). Each
  // target's peer registry entry applies to the forward exactly as it does to publish().
  void set_relay(bool enable) noexcept { _relay.store(enable, std::memory_order_relaxed); }
  bool relay() const noexcept { return _relay.load(std::memory_order_relaxed); }

  // Relay interest gate (off by default). When on, a frame received from a connection is re-sent
  // (to the other connections AND across the bridge) only if some local same-cluster shard would
  // take it: state_cluster_shard::admits() for a mutation, so delegation ops always pass, and
  // admits_message() for a message. A relay without such a shard forwards everything, as before.
  // Ingest into the local shards is unchanged; each withheld frame is counted. Meant for a relay
  // whose clients all enforce the relay's interest, where a withheld frame is one no client would
  // apply. Thread-safe; a flip applies to the frames received after it.
  void set_relay_requires_interest(bool on) noexcept {
    _relay_requires_interest.store(on, std::memory_order_relaxed);
  }
  bool relay_requires_interest() const noexcept {
    return _relay_requires_interest.load(std::memory_order_relaxed);
  }
  std::uint64_t total_relay_withheld() const noexcept { return _relay_withheld.load(); }

  // Invoked (on a server RPC thread) when a peer's stream ENDS -- it closed or dropped its
  // connection -- with that peer's node id (from its Hello). The same contract as
  // state_transport_ws::set_on_connection_closed, so a relay retracts a departed native client from
  // the room exactly as it does a closed browser tab. Not called during stop(), nor for a stream
  // that never sent a Hello. The handler must be thread-safe.
  using connection_closed_fn = std::function<void(const std::string &node_id)>;
  void set_on_connection_closed(connection_closed_fn cb) { _on_conn_closed = std::move(cb); }

  // ── Media plane (carried as the Frame envelope's MediaFrame, same as the WS transport) ──
  // publish_media_frame sends a frame to this transport's connections; a relay's media_router fans
  // each received frame back out via fan_media(). set_on_media is the leaf subscribe sugar. Media
  // is NOT replicated, NOT journaled, and never crosses the control bridge (it crosses protocols
  // through the media_router instead). Thread-safe; a no-op with no connections.
  void publish_media_frame(const media_frame &m);
  using media_frame_fn = std::function<void(const media_frame &)>;
  void set_on_media(media_frame_fn cb);

  // ── media_endpoint (the media_router plug-in surface) ──
  void fan_media(const media_frame &f, const void *exclude = nullptr) override;
  void set_media_sink(media_sink_fn sink) override;
  const char *media_protocol() const noexcept override { return "grpc"; }
  // Per-receiver egress gate for fan_media (see media_endpoint::set_media_gate). The gate is copied
  // once per frame and asked once per identified receiver; a receiver that has not sent its Hello
  // always gets the frame, and publish_media_frame is never gated. {} restores full fan-out.
  void set_media_gate(media_gate_fn gate) override;

  // Stall drop (0 = off, the default): skip a receiver whose write in progress started more than
  // `ms` ago instead of queueing this frame behind it. A healthy buffered write takes microseconds,
  // so one blocked that long means the peer's flow-control window is full; dropping the frame
  // keeps the publisher's reader thread (and every other receiver) moving. Media is loss-tolerant;
  // the existing 3 s write watchdog still closes a peer that stays stuck. Applies to every media
  // write, own uplink included (the counterpart of the WS transport's backlog drop).
  void set_media_stall_drop(std::chrono::milliseconds ms) noexcept;
  // True when a write is in flight (`write_started_ns` != 0) and has run longer than `stall_ns`;
  // `stall_ns` 0 never stalls. Steady-clock nanoseconds, the base of the connection's write clock.
  static bool media_write_stalled(std::uint64_t write_started_ns, std::uint64_t now_ns,
                                  std::uint64_t stall_ns) noexcept;

  std::uint64_t total_media_sent() const noexcept { return _media_sent.load(); }
  std::uint64_t total_media_received() const noexcept { return _media_recv.load(); }
  std::uint64_t total_media_forwarded() const noexcept { return _media_fwd.load(); }
  std::uint64_t total_media_gated() const noexcept { return _media_gated.load(); }
  std::uint64_t total_media_stall_dropped() const noexcept { return _media_stall_dropped.load(); }

  // state_transport interface.
  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;
  publish_stats publish(const state_mutation &m) override;
  publish_message_stats publish_message(const state_message &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;
  bool fetch_chunk(const std::string &digest, chunk_callback on_chunk) override;
  // The answering side filters its reply by the requester's peer registry entry (the rule its live
  // traffic gets; a requester that has not sent its Hello gets everything).
  bool request_snapshot(const std::string &cluster_id, const std::string &path_prefix,
                        snapshot_callback on_entries) override;
  // Writes `entries` as one unsolicited final SnapshotResponse (request_id 0) to each live
  // connection whose Hello named `node_id`, unfiltered by the peer registry. Returns the number of
  // connections written: 0, with nothing encoded, when none matches or `entries` is empty. A write
  // to a peer that stopped reading is bounded by the write watchdog.
  std::size_t push_snapshot_to(const std::string &node_id,
                               const std::vector<snapshot_entry> &entries) override;

  // Diagnostics.
  std::size_t shard_count() const;
  std::size_t connection_count() const;
  // One report per live connection (bytes/frames both ways, Ping/Pong round-trip time).
  std::vector<state_link_report> link_reports() const override;
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
  void on_inbound_mutation(const state_mutation &m, connection *origin = nullptr);
  void on_inbound_message(const state_message &m, connection *origin = nullptr);
  // A MediaFrame arrived from `origin`: hand the decoded media_frame to the media sink tagged with
  // `origin` as the opaque source token (a relay's media_router fans it, skipping `origin`).
  void on_inbound_media(const media_frame &m, connection *origin = nullptr);
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
  // A server-side stream ended: fire the connection-closed handler for its peer (see
  // set_on_connection_closed).
  void notify_connection_closed(const std::string &node_id);
  // Note an inbound frame on `c` (link counters + liveness), and answer / measure a link probe.
  // Returns true when `in` was a Ping or Pong (fully handled: never dispatched further).
  bool on_link_frame(connection &c, const ::cvc::transport::v1::Frame &in);
  void increment_recv_frames() noexcept { _recv_frames.fetch_add(1, std::memory_order_relaxed); }
  void increment_recv_mutations() noexcept {
    _recv_mutations.fetch_add(1, std::memory_order_relaxed);
  }
  void increment_recv_messages() noexcept {
    _recv_messages.fetch_add(1, std::memory_order_relaxed);
  }

private:
  // Encode `m` to a Frame and write it to every live connection except `exclude` (a connection*;
  // nullptr = all), consulting the media gate when `gated`. Returns the number written. Backs
  // publish_media_frame (ungated) and fan_media (gated).
  std::size_t write_media(const media_frame &m, const void *exclude, bool gated);
  // The relay interest gate for one received frame, given the local same-cluster shards that just
  // ingested it (see set_relay_requires_interest).
  bool relay_admits(const state_mutation &m,
                    const std::vector<state_cluster_shard *> &shards) const;
  bool relay_admits(const state_message &m, const std::vector<state_cluster_shard *> &shards) const;
  void send_hello_locked(connection &c);
  void send_backfill_locked(connection &c, const std::vector<state_cluster_shard *> &shards);
  // Relay-only: push the full CURRENT snapshot of each shard to a newly admitted connection, so a
  // late joiner sees peers already present (the room tree is all remote-origin, so backfill carries
  // nothing).
  void send_snapshot_locked(connection &c, const std::vector<state_cluster_shard *> &shards);

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
  std::atomic<bool> _relay{false};
  std::atomic<bool> _relay_requires_interest{false};
  std::atomic<std::uint64_t> _relay_withheld{0};
  connection_closed_fn _on_conn_closed; // set before start(); read on RPC threads

  std::atomic<std::uint64_t> _backfilled{0};
  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _recv_messages{0};
  std::atomic<std::uint64_t> _delivered{0};

  media_sink_fn _media_sink; // media_router ingress (relay) OR set_on_media leaf sugar
  std::atomic<std::uint64_t> _media_sent{0};
  std::atomic<std::uint64_t> _media_recv{0};
  std::atomic<std::uint64_t> _media_fwd{0};
  // The media gate, swapped whole under _media_gate_mu and copied out once per frame: a reader
  // thread keeps calling its own copy while another thread replaces the slot.
  mutable std::mutex _media_gate_mu;
  std::shared_ptr<const media_gate_fn> _media_gate;
  std::atomic<std::uint64_t> _media_gated{0};
  std::atomic<std::uint64_t> _media_stall_ns{0};
  std::atomic<std::uint64_t> _media_stall_dropped{0};

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

  // Link probe: every ping_interval(), send each live connection a Ping (state_transport.proto);
  // the peer's Pong gives the round trip. Runs on its own thread from start() to stop().
  std::thread _ping_thread;
  void ping_loop();
  void send_pings();
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_GRPC_H__

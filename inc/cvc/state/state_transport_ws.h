/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_TRANSPORT_WS_H__
#define __CVC_STATE_TRANSPORT_WS_H__

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cvc/media/media_endpoint.h> // media_frame + the media_router plug-in surface
#include <cvc/state/state_transport.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

// The media frame carried over this transport is the GENERIC, transport-neutral cvc::media_frame
// (inc/cvc/media/media_frame.h). ws_media_frame is kept as an alias for existing callers. Its wire
// encoding is proto/state_transport.proto's MediaFrame, shared with the gRPC transport (both carry
// the Frame envelope). NOT shard state: a relay fans it across the media_router; a leaf delivers it
// to its on_media sink; it never touches a shard or the WS<->gRPC CONTROL bridge.
using ws_media_frame = media_frame;

// ----------------
// cvc::state_transport_ws
// ----------------
// A WebSocket backend of the state_transport interface, the SIBLING of
// state_transport_grpc. It exists so a BROWSER (wasm) client can join the same
// cvc::state cluster as native clients: browsers cannot open gRPC streams or
// listen for connections, but they can open a WebSocket to a reachable server.
//
// SAME WIRE AS gRPC. Both transports exchange the identical protobuf envelope
// cvc.transport.v1.Frame (proto/state_transport.proto: oneof Hello / Mutation /
// Message / Chunk* / Snapshot* / Heartbeat), here carried as WebSocket BINARY
// frames instead of a gRPC bidi stream. That is the whole point of the design:
// because the frame bytes are identical, a relay running BOTH a gRPC acceptor
// and a WS acceptor can forward a frame received on one onto the other, so wasm
// and native clients see and hear each other in one room (the user's interop
// requirement). See the "relay/bridge" note on set_relay() below.
//
// TOPOLOGY. A browser is a CLIENT only (SFU-only, roadmap §"Browser = SFU-only"):
// it connect_to_peer()s the server's ws:// URL and never start()s a server. A
// native node can do either. So start() is native-only (Boost.Beast acceptor);
// connect_to_peer() has two client impls selected at compile time — Boost.Beast
// on native, the browser WebSocket (emscripten/websocket.h) under __EMSCRIPTEN__.
//
// BUILD. Needs the generated Frame message (protobuf; NOT the gRPC service), so
// it does not require CVC_ENABLE_GRPC — the proto's MESSAGES are generated for
// this target even in a grpc-off / wasm build (the gRPC *service* stubs are what
// grpc-only). Boost.Beast (header-only, part of Boost) provides the native WS.
//
// SCOPE (MVP, this first cut): server + client, Hello + Mutation + Message
// frames, the pure-virtual state_transport interface (register_shard / publish /
// pump_* / flush) + publish_message, relay mode, and backfill-on-connect — i.e.
// enough to replicate a room tree + room messages between a WS client and the
// server, and (with relay) across to gRPC clients. DEFERRED to Phase 2, mirroring
// state_transport_grpc: request_snapshot (the conference late-joiner bootstrap),
// fetch_chunk (blob data plane), heartbeat, TLS (wss://), and bearer auth.
//
// Lifetime: like the other transports, stop() (or destroy) the transport BEFORE
// destroying any registered state_cluster_shard, or an in-flight inbound frame
// can dispatch to a freed shard.
//
class state_transport_ws final : public state_transport, public media_endpoint {
public:
  state_transport_ws();
  ~state_transport_ws() override;

  state_transport_ws(const state_transport_ws &) = delete;
  state_transport_ws &operator=(const state_transport_ws &) = delete;

  // ── Server (native only) ──
  // Bind a WebSocket acceptor at `listen_addr` ("host:port", e.g. "127.0.0.1:0"
  // for an ephemeral port). Each accepted connection speaks Frame-over-WS-binary.
  // node_id / cluster_id populate the Hello sent on each new connection (advisory).
  // Throws std::runtime_error on bind failure. No-op / unsupported under wasm.
  void start(const std::string &listen_addr, const std::string &node_id = std::string(),
             const std::string &cluster_id = std::string());

  // The bound listen address with the port resolved (when "host:0" was passed).
  std::string listen_address() const;

  // ── Client ──
  // Dial a WebSocket URL ("ws://host:port/path"; "/" path is fine). Opens the
  // connection, sends Hello, and starts reading inbound frames on a reader
  // (native: a thread; wasm: the browser event loop). Returns true once the
  // handshake completes within `timeout`.
  bool connect_to_peer(const std::string &url,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(4000));

  // Stop the acceptor, close every connection, join reader threads (native).
  void stop();

  // ── Relay / bridge (star topology) ──
  // When enabled, a Mutation/Message frame RECEIVED from one connection is also
  // forwarded to every OTHER connection (the origin is skipped; the frame keeps
  // its origin_node_id, so the receiver's dedup makes any round-trip a no-op).
  // The conference relay turns this ON and also runs a gRPC transport in the same
  // process over the SAME shards; a small bridge (Phase 2) relays frames between
  // the two transports' connection sets so WS and gRPC clients share the room.
  // Off by default — a leaf client applies a received frame locally and never
  // re-sends it.
  void set_relay(bool enable) noexcept { _relay.store(enable, std::memory_order_relaxed); }
  bool relay() const noexcept { return _relay.load(std::memory_order_relaxed); }

  // Invoked (on a server reader thread) when a peer CLOSES its connection, with that peer's node id
  // (from its Hello). A relay uses this to retract the departed participant from the room so the
  // roster does not accumulate ghosts of closed browser tabs. Not called during stop(), nor for a
  // connection that never sent a Hello. The handler must be thread-safe.
  using connection_closed_fn = std::function<void(const std::string &node_id)>;
  void set_on_connection_closed(connection_closed_fn cb) { _on_conn_closed = std::move(cb); }

  // Relay maintenance: shut down any connection that has sent NO frame for `timeout` (so its peer's
  // on-close retraction fires), and join+erase finished reader threads. A conference client streams
  // media continuously, so this drops a peer whose tab navigated/reloaded away WITHOUT a clean
  // WebSocket close — the usual TCP close is never seen for such a ghost. Call periodically on the
  // server's main loop. No-op under wasm. Returns how many connections were erased this call.
  std::size_t reap_idle(std::chrono::milliseconds timeout);

  // ── Media plane (bring-up: media over this WS control channel) ──
  // publish_media_frame sends a frame to this transport's connections: a browser CLIENT has one
  // connection (the relay), so it pushes its camera/mic UP; a relay's media_router fans each received
  // frame back out via fan_media(). The frame is NOT journaled and never crosses the WS<->gRPC
  // CONTROL bridge (media crosses protocols through the media_router instead). Thread-safe; a no-op
  // with no connections.
  void publish_media_frame(const ws_media_frame &m);

  // Leaf (browser) subscribe sugar: route each peer's received frame into a decode proxy / playback.
  // Layered over set_media_sink (the source-connection token is ignored). Must be thread-safe.
  using media_frame_fn = std::function<void(const ws_media_frame &)>;
  void set_on_media(media_frame_fn cb);

  // ── media_endpoint (the media_router plug-in surface) ──
  // fan_media writes `f` to every connection except `exclude` (a connection* this transport handed
  // its sink; nullptr = all). set_media_sink installs the router's ingress (see media_endpoint).
  void fan_media(const media_frame &f, const void *exclude = nullptr) override;
  void set_media_sink(media_sink_fn sink) override;
  const char *media_protocol() const noexcept override { return "ws"; }

  std::uint64_t total_media_sent() const noexcept { return _media_sent.load(); }
  std::uint64_t total_media_received() const noexcept { return _media_recv.load(); }
  std::uint64_t total_media_forwarded() const noexcept { return _media_fwd.load(); }

  // Backfill on connect (default true): replay each registered shard's journaled
  // local-origin mutations to a newly admitted connection BEFORE any live traffic,
  // so a peer that connects after local writes still converges (same rationale as
  // state_transport_grpc::set_backfill_on_connect). The receiver's seen-set makes
  // a repeat a no-op when both sides backfill.
  void set_backfill_on_connect(bool enable) noexcept { _backfill_on_connect = enable; }
  bool backfill_on_connect() const noexcept { return _backfill_on_connect; }

  // ── state_transport interface ──
  void register_shard(state_cluster_shard *shard) override;
  void unregister_shard(state_cluster_shard *shard) override;
  publish_stats publish(const state_mutation &m) override;
  publish_message_stats publish_message(const state_message &m) override;
  std::size_t pump_shard(state_cluster_shard &shard) override;
  std::size_t pump_all() override;
  void flush() override;
  // fetch_chunk() / request_snapshot() use the base defaults for now (Phase 2).

  // ── Diagnostics ──
  std::size_t shard_count() const;
  std::size_t connection_count() const;
  std::uint64_t total_published() const noexcept { return _published.load(); }
  std::uint64_t total_sent_frames() const noexcept { return _sent_frames.load(); }
  std::uint64_t total_received_frames() const noexcept { return _recv_frames.load(); }
  std::uint64_t total_received_mutations() const noexcept { return _recv_mutations.load(); }
  std::uint64_t total_received_messages() const noexcept { return _recv_messages.load(); }
  std::uint64_t total_delivered() const noexcept { return _delivered.load(); }
  std::uint64_t total_backfilled() const noexcept { return _backfilled.load(); }

  // Wait until at least `target` MUTATION frames have been received (Hello frames
  // are not counted) or `timeout` elapses; returns the count reached.
  std::uint64_t wait_for_received(std::uint64_t target, std::chrono::milliseconds timeout);

  // ── Internal API (called by the per-connection reader) ──
  // Public so the impl TU (the only one that knows the generated proto + Beast
  // types) can call back in. Not a stable user-facing API.
  class connection;
  const std::string &local_node_id() const noexcept { return _node_id; }
  const std::string &local_cluster_id() const noexcept { return _cluster_id; }
  void on_inbound_mutation(const state_mutation &m, connection *origin = nullptr);
  void on_inbound_message(const state_message &m, connection *origin = nullptr);
  void register_connection(std::shared_ptr<connection> conn);
  // Register `conn`, then (holding conn's write lock) send Hello and, if enabled,
  // the backfill, so a publish() racing on another thread cannot interleave a
  // newer mutation ahead of the replayed older ones.
  void admit_connection(const std::shared_ptr<connection> &conn);
  void unregister_connection(connection *conn);
  void increment_recv_frames() noexcept { _recv_frames.fetch_add(1, std::memory_order_relaxed); }
  // Parse one inbound Frame (serialized protobuf bytes) off `c` and dispatch it (Hello / Mutation /
  // Message). Shared by the native reader loop and the wasm onmessage callback.
  void deliver_inbound(connection *c, const void *data, std::size_t len);
  // (wasm) Once the browser socket's onopen fires it can send, so push Hello + the backfill — the
  // deferred second half of admit_connection for a connection that cannot send until it is open.
  void on_connection_open(connection *c);

private:
  // Encode `m` to a Frame and write it to every live connection except `exclude` (a connection*;
  // nullptr = all). Returns the number written. Backs both publish_media_frame and fan_media.
  std::size_t write_media(const media_frame &m, const void *exclude);
  void send_hello_locked(connection &c);
  void send_backfill_locked(connection &c, const std::vector<state_cluster_shard *> &shards);
  // Relay-only: push the full CURRENT snapshot of each shard to a newly admitted connection, so a
  // late joiner sees peers already present. replay_local() cannot do this on a relay because the
  // room tree it holds is all remote-origin (forwarded from clients), not local.
  void send_snapshot_locked(connection &c, const std::vector<state_cluster_shard *> &shards);

  // Server accept loop (native) and the per-connection reader loop, both run on their own threads.
  // Defined in the impl TU (it owns the proto parse + the Beast read/write); no-op under wasm.
  void accept_loop();
  void read_loop(std::shared_ptr<connection> conn);

  struct impl;
  std::unique_ptr<impl> _impl;

  std::string _node_id;
  std::string _cluster_id;
  std::atomic<bool> _running{false};

  mutable std::mutex _shards_mu;
  std::vector<state_cluster_shard *> _shards;

  mutable std::mutex _conns_mu;
  std::vector<std::shared_ptr<connection>> _conns;

  std::atomic<bool> _backfill_on_connect{true};
  std::atomic<bool> _relay{false};
  connection_closed_fn _on_conn_closed; // set by a relay; invoked on a peer close (see above)
  media_sink_fn _media_sink; // media_router ingress (relay) OR set_on_media leaf sugar; see above

  std::atomic<std::uint64_t> _media_sent{0};  // MediaFrames this transport sent (publish_media_frame)
  std::atomic<std::uint64_t> _media_recv{0};  // MediaFrames received from peers
  std::atomic<std::uint64_t> _media_fwd{0};   // MediaFrames a relay fanned to other connections

  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _recv_messages{0};
  std::atomic<std::uint64_t> _delivered{0};
  std::atomic<std::uint64_t> _backfilled{0};

  // wait_for_received infrastructure (received MUTATION frames).
  mutable std::mutex _recv_mu;
  std::condition_variable _recv_cv;
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_WS_H__

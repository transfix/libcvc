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
  // re-sends it. Each forward honours the target peer's registry entry
  // (subscriptions + delivery filter), exactly as publish() does.
  void set_relay(bool enable) noexcept { _relay.store(enable, std::memory_order_relaxed); }
  bool relay() const noexcept { return _relay.load(std::memory_order_relaxed); }

  // Relay interest gate (opt-in; default off). When on, a Mutation/Message received from a
  // connection is re-fanned -- to the other connections AND across the bridge -- only when some
  // registered same-cluster shard would admit it (state_cluster_shard::admits / admits_message:
  // the path is of interest, and a delegation op always passes). A relay whose clients all
  // enforce the relay's own interest set thereby stops forwarding frames no client would apply.
  // A relay with no matching shard still forwards everything, and ingest is unchanged. Each
  // withheld frame is counted in total_relay_withheld().
  void set_relay_requires_interest(bool on) noexcept {
    _relay_requires_interest.store(on, std::memory_order_relaxed);
  }
  bool relay_requires_interest() const noexcept {
    return _relay_requires_interest.load(std::memory_order_relaxed);
  }
  std::uint64_t total_relay_withheld() const noexcept { return _relay_withheld.load(); }

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

  // Relay reliability: bring EVERY connection up to date with the current shard snapshot (not just
  // a new one, the way admit does). A client-to-client live re-fan is single-shot with no retry, so
  // a join that races an already-connected peer's socket can be missed and never recovers; calling
  // this periodically on the server's main loop lets every client re-converge on the room/scene
  // tree within one interval (idempotent bulk apply). Returns how many connections it wrote to: a
  // connection that already holds everything it is delivered gets no frame and is not counted.
  //
  // It sends DIFFS. Per connection, the transport remembers what the last snapshot frame (the admit
  // snapshot or a broadcast) carried for each path, and forgets a path whenever any other frame
  // touching it is written to that peer (a live mutation, a push) or arrives from it (the peer's
  // own write). A broadcast then carries only the entries whose current content differs from what
  // the peer was last sent, plus the forgotten ones, so an unchanged tree costs nothing and a
  // missed live re-fan is still healed on the next call. A peer gets the whole tree it is delivered
  // only on a full refresh: once, when it asks (request_full_refresh) -- for state it dropped on
  // its own side, which the relay cannot see.
  //
  // Each connection's frame honours the peer registry: it holds only the entries its
  // state_peer_registry::delivery_view passes. Shard snapshots are taken once per call.
  std::size_t broadcast_snapshot();

  // Client side of the full refresh: ask every connected relay to send this node the whole tree it
  // is delivered on its next broadcast_snapshot, not just the diff. For a client that may have
  // dropped state the relay believes it holds -- frames that arrived while no shard was registered,
  // a shard re-registered or given a wider interest set, a local write that was never published.
  // Sends a SnapshotRequest (the whole tree; a path prefix is not honoured yet). Returns the
  // connections written.
  std::size_t request_full_refresh();

  // ── Media plane (bring-up: media over this WS control channel) ──
  // publish_media_frame sends a frame to this transport's connections: a browser CLIENT has one
  // connection (the relay), so it pushes its camera/mic UP; a relay's media_router fans each
  // received frame back out via fan_media(). The frame is NOT journaled and never crosses the
  // WS<->gRPC CONTROL bridge (media crosses protocols through the media_router instead).
  // Thread-safe; a no-op with no connections.
  void publish_media_frame(const ws_media_frame &m);

  // Leaf (browser) subscribe sugar: route each peer's received frame into a decode proxy /
  // playback. Layered over set_media_sink (the source-connection token is ignored). Must be
  // thread-safe.
  using media_frame_fn = std::function<void(const ws_media_frame &)>;
  void set_on_media(media_frame_fn cb);

  // ── media_endpoint (the media_router plug-in surface) ──
  // fan_media writes `f` to every connection except `exclude` (a connection* this transport handed
  // its sink; nullptr = all). set_media_sink installs the router's ingress (see media_endpoint).
  void fan_media(const media_frame &f, const void *exclude = nullptr) override;
  void set_media_sink(media_sink_fn sink) override;
  const char *media_protocol() const noexcept override { return "ws"; }
  // Per-receiver egress gate (see media_endpoint::set_media_gate). fan_media asks it once per
  // identified receiving connection; a connection that has not sent its Hello always receives,
  // and publish_media_frame (this node's own uplink) is never gated. Thread-safe; the gate is
  // swapped whole and a fan already in progress finishes with the gate it started with.
  void set_media_gate(media_gate_fn gate) override;

  // Backlog drop (kDefaultMediaBacklogDrop; 0 = off): skip a connection whose OS send buffer
  // already holds more than `bytes` unsent, rather than let the synchronous write block behind a
  // peer that cannot keep pace. Media is loss-tolerant, and a small threshold keeps a slow peer's
  // buffer from crowding out the state writes queued behind its video. Applies to every media
  // write, own uplink included (the counterpart of state_transport_grpc::set_media_stall_drop).
  // Linux only: the backlog is read with SIOCOUTQ and is 0 elsewhere. The gate is asked after
  // this check, so a dropped frame uses no thumbnail. Thread-safe.
  void set_media_backlog_drop(std::size_t bytes) noexcept;
  static constexpr std::size_t kDefaultMediaBacklogDrop = 32 * 1024;

  std::uint64_t total_media_sent() const noexcept { return _media_sent.load(); }
  std::uint64_t total_media_received() const noexcept { return _media_recv.load(); }
  std::uint64_t total_media_forwarded() const noexcept { return _media_fwd.load(); }
  // Receiver deliveries the media gate denied (one per frame per denied connection).
  std::uint64_t total_media_gated() const noexcept { return _media_gated.load(); }
  // Receiver deliveries the backlog drop skipped (one per frame per backed-up connection).
  std::uint64_t total_media_backlog_dropped() const noexcept {
    return _media_backlog_dropped.load();
  }

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

  // One unsolicited, final snapshot frame of exactly `entries` to each live connection identified
  // as `node_id` (see state_transport::push_snapshot_to); the registry is not consulted. A
  // connection with more than 2 MB of unsent backlog is skipped -- the caller's next push or the
  // periodic broadcast_snapshot heals it. Returns the connections written; 0, with nothing
  // encoded, when none matches or `entries` is empty.
  std::size_t push_snapshot_to(const std::string &node_id,
                               const std::vector<snapshot_entry> &entries) override;
  // See state_transport::set_admission_hold. Applies to the connections this relay accepts.
  void set_admission_hold(std::chrono::milliseconds max_hold) override;
  std::size_t release_admissions() override;
  // Connections whose bootstrap the hold delayed, and those of them it released without a delivery
  // rule for the peer (the hold ran out, or was turned off).
  std::uint64_t total_admissions_held() const noexcept { return _admissions_held.load(); }
  std::uint64_t total_admission_timeouts() const noexcept { return _admission_timeouts.load(); }
  // See state_transport::set_inbound_gate. Applies to the connections this relay accepts
  // afterwards.
  void set_inbound_gate(std::shared_ptr<state_inbound_gate> gate) override;
  std::uint64_t total_inbound_rejected() const override { return _inbound_rejected.load(); }

  // Bearer-token authentication (a relay's room token, ARIADNE_CONFERENCE_ROADMAP §5f), as
  // state_transport_grpc::auth_config. A browser cannot set an Authorization header, so the token
  // travels in the handshake's Sec-WebSocket-Protocol: a client with outbound_token offers
  // "cvc.v1, cvc-token.<token>", and a server with expected_token refuses the upgrade (HTTP 401,
  // counted by total_auth_rejected()) unless it is offered, before any frame, so a peer without it
  // is sent nothing. A server answers cvc.v1 to a client that offers it, as a browser requires.
  // Tokens must be valid_room_token (state_identity.h). Set before start() / connect_to_peer().
  struct auth_config {
    std::string expected_token;
    std::string outbound_token;
  };
  void set_auth_config(auth_config cfg) { _auth = std::move(cfg); }
  std::uint64_t total_auth_rejected() const noexcept { return _auth_rejected.load(); }
  std::uint64_t total_devices_verified() const override { return _devices_verified.load(); }
  std::uint64_t total_device_proofs_refused() const override {
    return _device_proofs_refused.load();
  }

  // ── Diagnostics ──
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
  // Snapshot entries broadcast_snapshot has written, summed over connections.
  std::uint64_t total_broadcast_entries() const noexcept { return _broadcast_entries.load(); }

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
  // newer mutation ahead of the replayed older ones. Under the admission hold an
  // accepted relay connection gets only the Hello here (release_admissions).
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
  // nullptr = all). With `gated`, the media gate (when one is set) is asked per identified receiver
  // first. Returns the number written. Backs publish_media_frame (ungated) and fan_media (gated).
  std::size_t write_media(const media_frame &m, const void *exclude, bool gated);
  void send_hello_locked(connection &c);
  // Whether `id` is this transport's own node id or a registered shard's (state_inbound_gate: a
  // client never speaks as the relay).
  bool is_local_id(const std::string &id) const;
  // A client answering a relay's Hello nonce (DeviceProof), and a relay checking one.
  void answer_device_challenge(connection &c, const std::string &nonce,
                               const std::string &relay_node_id);
  void on_device_proof(connection &c, const std::string &public_key, const std::string &signature);
  // Replay each shard's journaled local writes (backfill on connect); with `view`, only the ones
  // that rule delivers.
  void send_backfill_locked(connection &c, const std::vector<state_cluster_shard *> &shards,
                            const state_peer_registry::delivery_view *view);
  // Relay-only: push the full CURRENT snapshot of each shard to a newly admitted connection, so a
  // late joiner sees peers already present. replay_local() cannot do this on a relay because the
  // room tree it holds is all remote-origin (forwarded from clients), not local. With `view`, only
  // what that rule delivers.
  void send_snapshot_locked(connection &c, const std::vector<state_cluster_shard *> &shards,
                            const state_peer_registry::delivery_view *view);
  // Bootstrap a held connection (backfill, then the snapshot through `view`) and let live traffic
  // reach it. False when it was no longer held.
  bool release_locked(connection &c, const std::vector<state_cluster_shard *> &shards,
                      const state_peer_registry::delivery_view *view);

  // Server accept loop (native) and the per-connection reader loop, both run on their own threads.
  // Defined in the impl TU (it owns the proto parse + the Beast read/write); no-op under wasm.
  void accept_loop();
  void read_loop(std::shared_ptr<connection> conn);
  // An accepted connection's own thread: its handshake (where a room token is checked), then, once
  // admitted, its reader -- so a client slow to send its upgrade request holds up no one else.
  void serve_accepted(std::shared_ptr<connection> conn);
  bool handshake_accepted(connection &c);
  // Join and drop the threads of handshakes that ended unadmitted.
  void reap_handshakes();

  // Link probe: Ping every live connection once ping_interval() has passed since the last round.
  // Native: a thread started with the first start() / connect_to_peer(). Browser (no threads):
  // pump_all() calls it every frame. A Ping that arrives is answered at once in deliver_inbound --
  // in a browser that is the WebSocket onmessage event, which keeps firing in a backgrounded tab.
  void send_pings_if_due();
  void ping_loop();
  std::atomic<std::int64_t> _next_ping_ns{0};
  std::atomic<bool> _ping_started{false};

  struct impl;
  std::unique_ptr<impl> _impl;

  std::string _node_id;
  std::string _cluster_id;
  std::atomic<bool> _running{false};

  mutable std::mutex _shards_mu;
  std::vector<state_cluster_shard *> _shards;

  mutable std::mutex _conns_mu;
  std::vector<std::shared_ptr<connection>> _conns;
  // Accepted connections still in their handshake, each on its own thread (guarded by _conns_mu).
  // The probe thread closes one whose handshake outlasts kHandshakeNs; stop() closes them all.
  std::vector<std::shared_ptr<connection>> _handshaking;

  std::atomic<bool> _backfill_on_connect{true};
  std::atomic<bool> _relay{false};
  std::atomic<bool> _relay_requires_interest{false};
  std::atomic<std::uint64_t> _relay_withheld{0}; // inbound frames the interest gate kept local
  connection_closed_fn _on_conn_closed; // set by a relay; invoked on a peer close (see above)
  media_sink_fn _media_sink; // media_router ingress (relay) OR set_on_media leaf sugar; see above

  // The media egress gate: replaced whole under _media_gate_mu, and copied once per fanned frame so
  // the per-receiver loop (and its socket writes) runs without the lock. Null = no gate.
  mutable std::mutex _media_gate_mu;
  std::shared_ptr<const media_gate_fn> _media_gate;

  std::atomic<std::uint64_t> _media_sent{
      0}; // MediaFrames this transport sent (publish_media_frame)
  std::atomic<std::uint64_t> _media_recv{0};  // MediaFrames received from peers
  std::atomic<std::uint64_t> _media_fwd{0};   // MediaFrames a relay fanned to other connections
  std::atomic<std::uint64_t> _media_gated{0}; // receiver deliveries the media gate denied
  // Receiver deliveries the backlog drop skipped, and its threshold in bytes (0 = off).
  std::atomic<std::uint64_t> _media_backlog_dropped{0};
  std::atomic<std::size_t> _media_backlog_drop{kDefaultMediaBacklogDrop};
  std::atomic<std::int64_t> _admission_hold_ns{0}; // 0 = admit at once
  std::atomic<std::uint64_t> _admissions_held{0};
  std::atomic<std::uint64_t> _admission_timeouts{0};
  // The inbound gate new accepted connections copy (each keeps its own, so a reader never locks
  // this).
  mutable std::mutex _inbound_gate_mu;
  std::shared_ptr<state_inbound_gate> _inbound_gate;
  std::atomic<std::uint64_t> _inbound_rejected{0};
  auth_config _auth; // set before start() / connect_to_peer()
  std::atomic<std::uint64_t> _auth_rejected{0};
  std::atomic<std::uint64_t> _devices_verified{0};
  std::atomic<std::uint64_t> _device_proofs_refused{0};

  std::atomic<std::uint64_t> _published{0};
  std::atomic<std::uint64_t> _sent_frames{0};
  std::atomic<std::uint64_t> _recv_frames{0};
  std::atomic<std::uint64_t> _recv_mutations{0};
  std::atomic<std::uint64_t> _recv_messages{0};
  std::atomic<std::uint64_t> _delivered{0};
  std::atomic<std::uint64_t> _backfilled{0};
  std::atomic<std::uint64_t> _broadcast_entries{0};
  // broadcast_snapshot's call count: a connection's remembered paths carry the last call that
  // delivered them, so a path the peer no longer gets (or that left the tree) is dropped.
  std::atomic<std::uint64_t> _broadcast_round{0};

  // wait_for_received infrastructure (received MUTATION frames).
  mutable std::mutex _recv_mu;
  std::condition_variable _recv_cv;
};

} // namespace cvc

#endif // __CVC_STATE_TRANSPORT_WS_H__

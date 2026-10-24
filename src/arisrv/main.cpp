/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// arisrv — the headless Ariadne conference RELAY. Clients connect ONLY to this server, not
// to each other, so a conference works across home NAT. It relays:
//   - the ROOM-TREE control plane — state_transport_grpc::set_relay forwards room-tree updates
//     (roster + stream descriptors) between all clients, star-topology;
//   - the MEDIA plane — a stream_relay watches the room tree and fans out each published stream
//     (ingest the publisher's pushed frames, serve them to every subscriber).
// Core libcvc + gRPC only — NO cvcGL/VTK/GL — so it runs on a plain headless VM.
//
//   arisrv --room standup --state-listen 0.0.0.0:7700 --media-listen 0.0.0.0:7701

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib> // std::getenv (CVC_ARISRV_ROOM_DEBUG)
#include <cstring>
#include <cvc/ariadne/conference/room_intrinsics.h>       // room_leave (retract a departed peer)
#include <cvc/ariadne/conference/room_media_intrinsics.h> // room_relay_tick
#include <cvc/ariadne/conference/room_scene.h> // seed_default_scene (the shared 3D scene)
#include <cvc/ariadne/holon.h>                            // server-authoritative avatar holon (H1)
#include <cvc/ariadne/holon_sync.h> // server_pump: intent -> holon -> authoritative pose
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/core/app.h>
#include <cvc/media/media_router.h> // protocol-agnostic media hub (WS + gRPC + future protocols)
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <cvc/state/state_transport_grpc.h>
#if CVC_ENABLE_WS
#include <cvc/state/state_transport_ws.h> // browser (wasm) clients join over WebSocket, bridged to gRPC
#endif
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#if defined(__linux__) && defined(__GLIBC__)
#include <execinfo.h> // backtrace -- a fatal signal logs the relay's stack
#include <unistd.h>   // write, STDERR_FILENO
#endif

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

#if defined(__linux__) && defined(__GLIBC__)
// A relay runs unattended, so a crash must explain itself in its log: on SIGSEGV / SIGABRT (glibc's
// "malloc(): ... corrupted" aborts) / SIGBUS / SIGFPE / SIGILL, write the stack to stderr, then
// re-raise with the default action so the process still dies (and exits non-zero). Only
// async-signal-safe calls: backtrace_symbols_fd writes straight to the fd without allocating, and
// install_crash_trace() calls backtrace() once up front so its lazy libgcc load never happens
// inside the handler on a corrupted heap.
void on_fatal(int sig) {
  static const char head[] = "\n[arisrv] FATAL signal; stack:\n";
  (void)!write(STDERR_FILENO, head, sizeof(head) - 1);
  void *frames[64];
  const int n = backtrace(frames, 64);
  backtrace_symbols_fd(frames, n, STDERR_FILENO);
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}
void install_crash_trace() {
  void *warm[1];
  (void)backtrace(warm, 1);
  for (int sig : {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL})
    std::signal(sig, on_fatal);
}
#else
void install_crash_trace() {}
#endif
} // namespace

int main(int argc, char **argv) {
  std::string room = "standup", nodeId = "server";
  std::string stateListen = "0.0.0.0:7700", mediaListen = "0.0.0.0:7701";
#if CVC_ENABLE_WS
  std::string wsListen = "0.0.0.0:7702"; // WebSocket control plane for browser (wasm) clients
#endif
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
    if (!std::strcmp(a, "--room"))
      room = next("standup");
    else if (!std::strcmp(a, "--node-id"))
      nodeId = next("server");
    else if (!std::strcmp(a, "--state-listen"))
      stateListen = next(stateListen.c_str());
    else if (!std::strcmp(a, "--media-listen"))
      mediaListen = next(mediaListen.c_str());
#if CVC_ENABLE_WS
    else if (!std::strcmp(a, "--ws-listen"))
      wsListen = next(wsListen.c_str());
#endif
  }

  cvc::app app;

  // Presence: when a client's connection ends -- a native (gRPC) client exits, or a browser (WS)
  // tab closes -- retract it from the room so the roster and the avatar holon do not accumulate
  // ghosts. Both transports report the departed node id the same way, on their own threads, so the
  // id is only queued here; the main loop below applies room_leave (where every other room-tree
  // write happens) and the retraction rides the normal pump + relay forwarding to the others.
  std::mutex departed_mu;
  std::vector<std::string> departed;
  const auto on_departed = [&](const std::string &node_id) {
    std::lock_guard<std::mutex> lk(departed_mu);
    departed.push_back(node_id);
  };

  // Room-tree (state) plane: serve + RELAY (forward frames between clients). The shard ingests the
  // room tree into app.root() so the media relay below can watch it.
  cvc::state_transport_grpc state_tp;
  state_tp.set_on_connection_closed(on_departed);
  state_tp.start(stateListen, nodeId, "conf");
  state_tp.set_relay(true);
  cvc::state_cluster_shard shard(app, "conf", nodeId);
  // The relay forwards ONLY the shared room tree (conf.rooms.*). A client also writes per-document
  // UI state (ui.docs.*, e.g. conf.self — its display name) on the same shard; without this filter
  // the relay would ingest and fan that to every other client, so they would all show the same name
  // (clients "collide on the name"). Dropping non-"conf" paths at ingest keeps each client's UI
  // local and keeps the late-joiner snapshot to just the room tree.
  shard.add_interest("conf");
  shard.set_enforce_interest(true);
  shard.attach();
  shard.install_as_default();
  state_tp.register_shard(&shard);

#if CVC_ENABLE_WS
  // WebSocket control plane + the WS<->gRPC bridge, so BROWSER (wasm) clients join the SAME room as
  // native gRPC clients: each backend relays within itself, and the bridge carries a frame received
  // on one onto the other's connections (same shard, so both back-fill new peers). Live roster /
  // stream-descriptor mutations cross both ways now; request_snapshot over WS for a late joiner's
  // full bootstrap is Phase 2 (a browser joining a populated room sees peers that join AFTER it).
  cvc::state_transport_ws ws_tp;
  ws_tp.set_on_connection_closed(on_departed); // a closed browser tab leaves like a native client
  ws_tp.start(wsListen, nodeId, "conf");
  ws_tp.set_relay(true);
  ws_tp.register_shard(&shard);
  state_tp.set_bridge(&ws_tp);
  ws_tp.set_bridge(&state_tp);
#endif

  // Media plane: a stream_relay. Constructed BEFORE media_tp.start() (it installs the frame
  // router).
  cvc::stream_transport_grpc media_tp;
  cvc::ariadne::stream::stream_relay relay(app, media_tp);
  media_tp.start(mediaListen);

  // Media hub: a protocol-agnostic router fans each received media_frame between every transport
  // endpoint, so a browser (WS) and a native client (gRPC) in one room see/hear each other through
  // ONE canonical path — a new protocol is just another endpoint. (The WS<->gRPC control bridge
  // above carries the ROOM TREE between transports; media crosses HERE instead — media is never on
  // the control bridge.) Declared AFTER the transports so it destructs FIRST, clearing their media
  // sinks while they are still alive; the explicit stop()s below join readers before that.
  cvc::media_router mrouter;
  mrouter.add_endpoint(&state_tp); // native gRPC clients (state plane carries MediaFrame too)
#if CVC_ENABLE_WS
  mrouter.add_endpoint(&ws_tp); // browser (wasm) WebSocket clients
#endif

  install_crash_trace();
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::printf("[arisrv] relay up | room='%s' node='%s' | state=%s | media=%s\n", room.c_str(),
              nodeId.c_str(), state_tp.listen_address().c_str(), media_tp.listen_address().c_str());
#if CVC_ENABLE_WS
  std::printf("[arisrv] websocket (browser) clients: ws=%s (bridged to the gRPC room)\n",
              ws_tp.listen_address().c_str());
#endif
  std::fflush(stdout);

  std::set<std::string> relayed;
  std::size_t last = 0;
  int ticks = 0;
  int mediaticks = 0;           // cadence for the media (router) heartbeat
  std::uint64_t lastRouted = 0; // last-logged media_router routed count
#if CVC_ENABLE_WS
  int reapticks = 0; // cadence for reaping ghost (navigated-away) connections
#endif
  // The server-authoritative avatar holon for this room (H1): each tick it reads
  // every participant's proposed intent from the replicated tree, integrates it on
  // its own clock, and writes the authoritative pose back (fanned to clients by the
  // drain_local block below). The relay loop paces ~5 Hz, so the holon catches up
  // several quanta per tick and clients interpolate between the ~5 Hz pose samples;
  // a smoother authoritative rate is a later tuning (tick the holon on its own).
  cvc::ariadne::holon server_holon(cvc::ariadne::holon_role::server);
  const char *room_debug_env = std::getenv("CVC_ARISRV_ROOM_DEBUG");
  const bool room_debug =
      room_debug_env && *room_debug_env && std::strcmp(room_debug_env, "0") != 0;
  int room_debug_ticks = 0;
  // The shared 3D scene (H1 slice 5, roadmap §5g): the relay is its sole writer and seeds the MVP
  // environment (ground + a reference box under a sun) once. The writes are local shard mutations,
  // so the first pass of the loop below fans them to the clients, and late joiners get them in the
  // snapshot.
  if (cvc::ariadne::conference::seed_default_scene(app, room)) {
    const auto rev = cvc::ariadne::conference::scene_revision(app, room);
    std::printf("[arisrv] seeded the shared scene for room '%s' (rev %llu)\n", room.c_str(),
                static_cast<unsigned long long>(rev));
    std::fflush(stdout);
  }
  while (!g_stop.load()) {
#if CVC_ENABLE_WS
    // Reap ghosts (~every second): a tab that navigated or reloaded away without a clean WebSocket
    // close stops streaming media; reap_idle shuts such a connection down after ~10 s so its
    // on-close retraction fires and the roster does not accumulate ghosts. A live client streams
    // continuously.
    if (++reapticks % 5 == 0) {
      ws_tp.reap_idle(std::chrono::seconds(10));
      // Re-broadcast the room/scene snapshot to every client (~1s). A client-to-client live re-fan
      // is single-shot with no retry, so a join that races an already-connected peer's socket is
      // otherwise lost for good (an established client never sees a later joiner). This idempotent
      // re-sync makes every client re-converge within ~1s — and is the reliable push the
      // server-authoritative room+scene tree rides on.
      ws_tp.broadcast_snapshot();
    }
#endif
    // Apply any queued departures (a native client exited or a browser tab closed) before pumping,
    // so each retraction goes out to the remaining clients this tick. The departed participant's
    // proposed intent is zeroed too: the server holon drops them now (server_pump follows the
    // roster), and a client that rejoins under the same id must not resume a walk it proposed
    // before it left.
    {
      std::vector<std::string> gone;
      {
        std::lock_guard<std::mutex> lk(departed_mu);
        gone.swap(departed);
      }
      for (const std::string &nid : gone) {
        cvc::ariadne::conference::room_leave(app, room, nid);
        cvc::ariadne::write_intent(app, room, nid, cvc::ariadne::avatar_intent{});
        std::printf("[arisrv] participant left: %s\n", nid.c_str());
      }
      if (!gone.empty())
        std::fflush(stdout);
    }
    cvc::ariadne::conference::room_relay_tick(app, room, relay, relayed);
    // Advance the authoritative avatar holon and write each participant's pose. The
    // pose writes are local shard mutations, fanned to clients by the drain below.
    cvc::ariadne::server_pump(server_holon, app, room);
    // CVC_ARISRV_ROOM_DEBUG=1: every ~5 s, log the room as the server holds it -- each present
    // participant's authoritative position and its proposed intent -- so replication across the
    // gRPC and WebSocket planes can be checked from the relay without a client.
    if (room_debug && ++room_debug_ticks % 25 == 0) {
      const auto roster = cvc::ariadne::conference::room_roster(app, room);
      std::printf("[arisrv] room '%s': %zu present, holon tick %llu\n", room.c_str(), roster.size(),
                  static_cast<unsigned long long>(server_holon.tick()));
      for (const auto &p : roster) {
        cvc::physics::body_state s;
        std::uint64_t t = 0;
        cvc::ariadne::avatar_intent in;
        const bool has_pose = cvc::ariadne::read_pose(app, room, p.pid, s, t);
        const bool has_intent = cvc::ariadne::read_intent(app, room, p.pid, in);
        std::printf("[arisrv]   %-16s pos %s (%.2f, %.2f) intent %s (%.2f, %.2f)\n", p.pid.c_str(),
                    has_pose ? "" : "none", s.position.x, s.position.y, has_intent ? "" : "none",
                    in.move.x, in.move.y);
      }
      std::fflush(stdout);
    }
#if CVC_ENABLE_WS
    // Fan the relay's OWN room-tree writes — the room_leave retractions queued above and
    // room_relay_tick's per-stream "ready" markers — out to EVERY client on BOTH transports. The
    // shard's publish cursor is shared by the two transports, so a plain state_tp.pump_all() would
    // drain it and the WebSocket side would send nothing (browser rosters would never drop a closed
    // tab). So drain the local journal ONCE and publish each mutation to both transports: publish()
    // writes only to its own connections and never crosses the bridge, so each client gets it
    // exactly once with no echo. drain_local yields only our local writes, never the clients'
    // replicated mutations (those ride set_relay forwarding), so this cannot loop.
    {
      auto pending = shard.drain_local();
      for (const auto &m : pending) {
        state_tp.publish(m);
        ws_tp.publish(m);
      }
    }
#else
    // Publish the relay's own room-tree writes (per-stream "ready" markers from room_relay_tick) to
    // the connected clients. drain_local pumps only our local writes, never the clients' replicated
    // mutations (those ride set_relay forwarding), so this cannot echo.
    state_tp.pump_all();
#endif
    const std::size_t n = relay.stream_count();
    const bool heartbeat = (n > 0 && ++ticks % 25 == 0); // ~every 5s while gRPC media is live
    if (n != last || heartbeat) {
      last = n;
      std::printf("[arisrv] relaying %zu stream(s); %llu frames forwarded\n", n,
                  static_cast<unsigned long long>(relay.frames_forwarded()));
      std::fflush(stdout);
    }
    // Media heartbeat (~every 5s while anyone is connected). Conference media rides the control
    // channels (WS MediaFrame + gRPC MediaFrame), fanned by the media_router — NOT in the gRPC
    // `relay` stream stats above. Report the router total plus per-plane conns/recv/fanned so a
    // quiet relay is distinguishable from a stuck one, and so WS<->gRPC interop is visible (a frame
    // received on one plane is fanned on BOTH).
    if (++mediaticks % 25 == 0) {
      const std::uint64_t routed = mrouter.total_routed();
      std::size_t wsc = 0;
      std::uint64_t wsr = 0, wsf = 0;
#if CVC_ENABLE_WS
      wsc = ws_tp.connection_count();
      wsr = ws_tp.total_media_received();
      wsf = ws_tp.total_media_forwarded();
#endif
      const std::size_t gc = state_tp.connection_count();
      const std::uint64_t gr = state_tp.total_media_received();
      const std::uint64_t gf = state_tp.total_media_forwarded();
      if (wsc + gc > 0 || routed != lastRouted) {
        lastRouted = routed;
        std::printf("[arisrv] media: routed %llu | ws %zu conn recv %llu fan %llu | grpc %zu conn "
                    "recv %llu "
                    "fan %llu\n",
                    static_cast<unsigned long long>(routed), wsc,
                    static_cast<unsigned long long>(wsr), static_cast<unsigned long long>(wsf), gc,
                    static_cast<unsigned long long>(gr), static_cast<unsigned long long>(gf));
        std::fflush(stdout);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::printf("[arisrv] shutting down (relayed %zu stream(s), %llu frames forwarded)\n",
              relay.stream_count(), static_cast<unsigned long long>(relay.frames_forwarded()));
  relay.stop();
  media_tp.stop();
#if CVC_ENABLE_WS
  // Drop the bridge cross-pointers before stopping, so an in-flight inbound frame cannot forward
  // onto a transport that is being torn down.
  state_tp.set_bridge(nullptr);
  ws_tp.set_bridge(nullptr);
  ws_tp.unregister_shard(&shard);
  ws_tp.stop();
#endif
  state_tp.unregister_shard(&shard);
  state_tp.stop();
  shard.detach();
  return 0;
}

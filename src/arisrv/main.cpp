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
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/conference/room_intrinsics.h>       // room_leave (retract a departed peer)
#include <cvc/ariadne/conference/room_media_intrinsics.h> // room_relay_tick
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/core/app.h>
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

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
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

  // Room-tree (state) plane: serve + RELAY (forward frames between clients). The shard ingests the
  // room tree into app.root() so the media relay below can watch it.
  cvc::state_transport_grpc state_tp;
  state_tp.start(stateListen, nodeId, "conf");
  state_tp.set_relay(true);
  cvc::state_cluster_shard shard(app, "conf", nodeId);
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
  ws_tp.start(wsListen, nodeId, "conf");
  ws_tp.set_relay(true);
  ws_tp.register_shard(&shard);
  state_tp.set_bridge(&ws_tp);
  ws_tp.set_bridge(&state_tp);

  // Presence: when a browser (WS) client closes its tab, retract it from the room so the roster does
  // not accumulate ghosts of closed tabs. The close fires on a WS reader thread, so just queue the
  // departed node id here and apply room_leave on the main loop below (where every other room-tree
  // write happens), then the retraction rides the normal pump + relay forwarding to the others.
  std::mutex departed_mu;
  std::vector<std::string> departed;
  ws_tp.set_on_connection_closed([&](const std::string &node_id) {
    std::lock_guard<std::mutex> lk(departed_mu);
    departed.push_back(node_id);
  });
#endif

  // Media plane: a stream_relay. Constructed BEFORE media_tp.start() (it installs the frame
  // router).
  cvc::stream_transport_grpc media_tp;
  cvc::ariadne::stream::stream_relay relay(app, media_tp);
  media_tp.start(mediaListen);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::printf("[arisrv] relay up | room='%s' node='%s' | state=%s | media=%s\n",
              room.c_str(), nodeId.c_str(), state_tp.listen_address().c_str(),
              media_tp.listen_address().c_str());
#if CVC_ENABLE_WS
  std::printf("[arisrv] websocket (browser) clients: ws=%s (bridged to the gRPC room)\n",
              ws_tp.listen_address().c_str());
#endif
  std::fflush(stdout);

  std::set<std::string> relayed;
  std::size_t last = 0;
  int ticks = 0;
  while (!g_stop.load()) {
#if CVC_ENABLE_WS
    // Apply any queued departures (a browser tab closed) before pumping, so each retraction goes out
    // to the remaining clients this tick.
    {
      std::vector<std::string> gone;
      {
        std::lock_guard<std::mutex> lk(departed_mu);
        gone.swap(departed);
      }
      for (const std::string &nid : gone) {
        cvc::ariadne::conference::room_leave(app, room, nid);
        std::printf("[arisrv] participant left: %s\n", nid.c_str());
      }
      if (!gone.empty())
        std::fflush(stdout);
    }
#endif
    cvc::ariadne::conference::room_relay_tick(app, room, relay, relayed);
#if CVC_ENABLE_WS
    // Fan the relay's OWN room-tree writes — the room_leave retractions queued above and
    // room_relay_tick's per-stream "ready" markers — out to EVERY client on BOTH transports. The
    // shard's publish cursor is shared by the two transports, so a plain state_tp.pump_all() would
    // drain it and the WebSocket side would send nothing (browser rosters would never drop a closed
    // tab). So drain the local journal ONCE and publish each mutation to both transports: publish()
    // writes only to its own connections and never crosses the bridge, so each client gets it exactly
    // once with no echo. drain_local yields only our local writes, never the clients' replicated
    // mutations (those ride set_relay forwarding), so this cannot loop.
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
    const bool heartbeat = (n > 0 && ++ticks % 25 == 0); // ~every 5s while media is live
    if (n != last || heartbeat) {
      last = n;
      std::printf("[arisrv] relaying %zu stream(s); %llu frames forwarded\n", n,
                  static_cast<unsigned long long>(relay.frames_forwarded()));
      std::fflush(stdout);
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

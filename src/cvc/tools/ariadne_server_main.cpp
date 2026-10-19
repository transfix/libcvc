/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// ariadne-server — the headless Ariadne conference RELAY. Clients connect ONLY to this server, not
// to each other, so a conference works across home NAT. It relays:
//   - the ROOM-TREE control plane — state_transport_grpc::set_relay forwards room-tree updates
//     (roster + stream descriptors) between all clients, star-topology;
//   - the MEDIA plane — a stream_relay watches the room tree and fans out each published stream
//     (ingest the publisher's pushed frames, serve them to every subscriber).
// Core libcvc + gRPC only — NO cvcGL/VTK/GL — so it runs on a plain headless VM.
//
//   ariadne-server --room standup --state-listen 0.0.0.0:7700 --media-listen 0.0.0.0:7701

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/conference/room_media_intrinsics.h> // room_relay_tick
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/core/app.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <cvc/state/state_transport_grpc.h>
#include <set>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
} // namespace

int main(int argc, char **argv) {
  std::string room = "standup", nodeId = "server";
  std::string stateListen = "0.0.0.0:7700", mediaListen = "0.0.0.0:7701";
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

  // Media plane: a stream_relay. Constructed BEFORE media_tp.start() (it installs the frame
  // router).
  cvc::stream_transport_grpc media_tp;
  cvc::ariadne::stream::stream_relay relay(app, media_tp);
  media_tp.start(mediaListen);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::printf("[ariadne-server] relay up | room='%s' node='%s' | state=%s | media=%s\n",
              room.c_str(), nodeId.c_str(), state_tp.listen_address().c_str(),
              media_tp.listen_address().c_str());
  std::fflush(stdout);

  std::set<std::string> relayed;
  std::size_t last = 0;
  int ticks = 0;
  while (!g_stop.load()) {
    cvc::ariadne::conference::room_relay_tick(app, room, relay, relayed);
    // Publish the relay's own room-tree writes (per-stream "ready" markers from room_relay_tick) to
    // the connected clients. collect_local pumps only our local writes, never the clients'
    // replicated mutations (those ride set_relay forwarding), so this cannot echo.
    state_tp.pump_all();
    const std::size_t n = relay.stream_count();
    const bool heartbeat = (n > 0 && ++ticks % 25 == 0); // ~every 5s while media is live
    if (n != last || heartbeat) {
      last = n;
      std::printf("[ariadne-server] relaying %zu stream(s); %llu frames forwarded\n", n,
                  static_cast<unsigned long long>(relay.frames_forwarded()));
      std::fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::printf("[ariadne-server] shutting down (relayed %zu stream(s), %llu frames forwarded)\n",
              relay.stream_count(), static_cast<unsigned long long>(relay.frames_forwarded()));
  relay.stop();
  media_tp.stop();
  state_tp.unregister_shard(&shard);
  state_tp.stop();
  shard.detach();
  return 0;
}

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
// The room tree, and the camera/mic frames the state transports carry (the media_router's fan-out),
// go through the room's AREA OF INTEREST (room_aoi_relay): how much of each participant every
// other one receives, by their distance in the room. The stream_relay plane (--media-listen:
// room-publish / room-subscribe streams) is not gated by it.
// Core libcvc + gRPC only — NO cvcGL/VTK/GL — so it runs on a plain headless VM.
//
//   arisrv --room standup --state-listen 0.0.0.0:7700 --media-listen 0.0.0.0:7701

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib> // std::getenv (CVC_ARISRV_ROOM_DEBUG, CVC_ARISRV_S7, CVC_ARISRV_AOI)
#include <cstring>
#include <cvc/ariadne/conference/room_aoi.h>        // aoi_params, the presets, write_aoi_params
#include <cvc/ariadne/conference/room_aoi_relay.h>  // room_aoi_relay, relay_room_step
#include <cvc/ariadne/conference/room_intrinsics.h> // room_leave (retract a departed peer)
#include <cvc/ariadne/conference/room_media_intrinsics.h> // room_relay_tick
#include <cvc/ariadne/conference/room_net.h>   // per-participant + room link-quality stats
#include <cvc/ariadne/conference/room_scene.h> // seed_default_scene (the shared 3D scene)
#include <cvc/ariadne/holon.h>                 // server-authoritative avatar holon (H1)
#include <cvc/ariadne/holon_sync.h>            // server_pump: intent -> holon -> authoritative pose
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/core/app.h>
#include <cvc/media/media_router.h> // protocol-agnostic media hub (WS + gRPC + future protocols)
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_stream_transport_grpc.h>
#include <cvc/state/state_transport_grpc.h>
#if CVC_ENABLE_WS
#include <cvc/state/state_transport_ws.h> // browser (wasm) clients join over WebSocket, bridged to gRPC
#endif
#include <initializer_list>
#include <memory>
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

// Whether V is one of OK; otherwise a usage error for FLAG on stderr.
bool check_choice(const char *flag, const std::string &v, std::initializer_list<const char *> ok) {
  std::string all;
  for (const char *o : ok) {
    if (v == o)
      return true;
    if (!all.empty())
      all += '|';
    all += o;
  }
  std::fprintf(stderr, "[arisrv] %s expects %s, got '%s'\n", flag, all.c_str(), v.c_str());
  return false;
}

// A whole non-negative decimal integer, or a usage error naming FLAG.
bool check_count(const char *flag, const std::string &v, long &out) {
  char *end = nullptr;
  const long n = v.empty() ? -1 : std::strtol(v.c_str(), &end, 10);
  if (n < 0 || !end || *end != '\0') {
    std::fprintf(stderr, "[arisrv] %s expects a non-negative integer, got '%s'\n", flag, v.c_str());
    return false;
  }
  out = n;
  return true;
}
} // namespace

int main(int argc, char **argv) {
  std::string room = "standup", nodeId = "server";
  std::string stateListen = "0.0.0.0:7700", mediaListen = "0.0.0.0:7701";
#if CVC_ENABLE_WS
  std::string wsListen = "0.0.0.0:7702"; // WebSocket control plane for browser (wasm) clients
#endif
  // Spatial interest (S7). Each mechanism has its own switch and is on here, though the library
  // leaves every one of them off by default; --s7 off (or CVC_ARISRV_S7=0) turns them all off at
  // once, which is exactly the relay as it was before them. The scene choice is not one of them.
  //   --aoi off|conference|hall  the area-of-interest preset (or CVC_ARISRV_AOI). Default
  //                              conference, under which the 20 m room is unchanged; hall with
  //                              --scene large
  //   --aoi-video-max K          cameras each receiver gets at full rate, nearest first; the
  //                              rest of its inner peers as thumbnails (default 0 = all)
  //   --relay-gate on|off        forward a client's frame only when the room-tree shard takes it
  //   --relay-journal trim|keep  trim: a joiner bootstraps from the admit snapshot instead of a
  //                              journal backfill, and the journal is erased as it is published
  //   --media-stall-ms N         gRPC: skip a receiver whose media write has been stuck N ms
  //                              (default 40; 0 = off)
  //   --scene default|large      the seeded scene (large: a 100 m plane of LOD meshes)
  const char *s7_env = std::getenv("CVC_ARISRV_S7");
  const char *aoi_env = std::getenv("CVC_ARISRV_AOI");
  // CVC_ARISRV_S7: 0/off or 1/on (unset: on).
  std::string s7Arg = !s7_env || !*s7_env         ? "on"
                      : !std::strcmp(s7_env, "0") ? "off"
                      : !std::strcmp(s7_env, "1") ? "on"
                                                  : s7_env;
  std::string aoiMode = (aoi_env && *aoi_env) ? aoi_env : ""; // empty: the scene's default
  std::string relayGateArg = "on", relayJournal = "trim", scene = "default";
  std::string aoiVideoMaxArg = "0", mediaStallArg = "40";
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
    else if (!std::strcmp(a, "--s7"))
      s7Arg = next("on");
    else if (!std::strcmp(a, "--aoi"))
      aoiMode = next("conference");
    else if (!std::strcmp(a, "--aoi-video-max"))
      aoiVideoMaxArg = next("");
    else if (!std::strcmp(a, "--relay-gate"))
      relayGateArg = next("on");
    else if (!std::strcmp(a, "--relay-journal"))
      relayJournal = next("trim");
    else if (!std::strcmp(a, "--media-stall-ms"))
      mediaStallArg = next("");
    else if (!std::strcmp(a, "--scene"))
      scene = next("default");
  }
  if (aoiMode.empty())
    aoiMode = scene == "large" ? "hall" : "conference";
  if (!check_choice("--s7", s7Arg, {"on", "off"}) ||
      !check_choice("--aoi", aoiMode, {"off", "conference", "hall"}) ||
      !check_choice("--relay-gate", relayGateArg, {"on", "off"}) ||
      !check_choice("--relay-journal", relayJournal, {"trim", "keep"}) ||
      !check_choice("--scene", scene, {"default", "large"}))
    return 2;
  long aoiVideoMaxN = 0, mediaStallMs = 40;
  if (!check_count("--aoi-video-max", aoiVideoMaxArg, aoiVideoMaxN) ||
      !check_count("--media-stall-ms", mediaStallArg, mediaStallMs))
    return 2;
  const int aoiVideoMax = static_cast<int>((std::min)(aoiVideoMaxN, 1000000L));
  const bool s7 = s7Arg == "on";
  if (!s7)
    aoiMode = "off";
  const bool relayGate = s7 && relayGateArg == "on";
  const bool trimJournal = s7 && relayJournal == "trim";
  const long stallMs = s7 ? mediaStallMs : 0L;

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
  // The S7 switches (each one's library default when off), set before the first client connects.
  // The gate withholds a forwarded frame the shard below would not take; every client enforces the
  // same "conf" interest, so none of them would have applied it. Without backfill a joiner takes
  // the relay's own writes from the admit snapshot, which holds every one of them, so the journal
  // can be trimmed as it is published (relay_room_step).
  state_tp.set_relay_requires_interest(relayGate);
  state_tp.set_backfill_on_connect(!trimJournal);
  state_tp.set_media_stall_drop(std::chrono::milliseconds(stallMs));
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
  ws_tp.set_on_connection_closed(on_departed);  // a closed browser tab leaves like a native client
  ws_tp.set_relay_requires_interest(relayGate); // as on the gRPC side, both legs of the bridge
  ws_tp.set_backfill_on_connect(!trimJournal);
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

  // The room's area of interest: every room step it places the participants from the server holon
  // below and, per receiver, filters the room tree each transport sends (other participants'
  // intents, the poses of peers out of reach), pushes those poses at a reduced cadence instead,
  // and gates the media fan-out (thumbnails for a middle peer's camera, nothing from an outer
  // one). Declared after the transports and the router, so it is gone before they are; it is also
  // reset explicitly at shutdown, while they still run.
  cvc::ariadne::conference::aoi_params aoiParams;
  std::unique_ptr<cvc::ariadne::conference::room_aoi_relay> aoi_relay;
  if (aoiMode != "off") {
    aoiParams = aoiMode == "hall" ? cvc::ariadne::conference::hall_aoi_params()
                                  : cvc::ariadne::conference::conference_aoi_params();
    aoiParams.video_full_max = aoiVideoMax;
    aoi_relay =
        std::make_unique<cvc::ariadne::conference::room_aoi_relay>(app, shard, room, aoiParams);
    aoi_relay->add_transport(&state_tp);
    aoi_relay->add_media_endpoint(&state_tp);
#if CVC_ENABLE_WS
    aoi_relay->add_transport(&ws_tp);
    aoi_relay->add_media_endpoint(&ws_tp);
#endif
  }

  install_crash_trace();
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::printf("[arisrv] relay up | room='%s' node='%s' | state=%s | media=%s\n", room.c_str(),
              nodeId.c_str(), state_tp.listen_address().c_str(), media_tp.listen_address().c_str());
#if CVC_ENABLE_WS
  std::printf("[arisrv] websocket (browser) clients: ws=%s (bridged to the gRPC room)\n",
              ws_tp.listen_address().c_str());
#endif
  std::printf("[arisrv] S7 %s | aoi %s (full-rate cameras: %d, 0 = all) | relay gate %s | journal "
              "%s | grpc media stall drop %ld ms | scene %s\n",
              s7 ? "on" : "off (the pre-S7 relay)", aoiMode.c_str(), aoiVideoMax,
              relayGate ? "on" : "off", trimJournal ? "trim (no backfill)" : "keep (backfill)",
              stallMs, scene.c_str());
  std::fflush(stdout);

  std::set<std::string> relayed;
  std::size_t last = 0;
  int ticks = 0;
  int mediaticks = 0;           // cadence for the media (router) heartbeat
  std::uint64_t lastRouted = 0; // last-logged media_router routed count
#if CVC_ENABLE_WS
  int reapticks = 0; // cadence for reaping ghost (navigated-away) connections
#endif
  // The server-authoritative avatar holon for this room (H1): each room step it reads
  // every participant's proposed intent from the replicated tree, integrates it on
  // its own clock (a stamped intent at the tick the client applied it), and writes
  // the authoritative pose back -- what changed, plus a ~1 s heartbeat -- fanned to
  // clients by room_step. The relay loop paces ~5 Hz; the room steps
  // kRoomSubsteps times per pass (~20 Hz), so a pose sample is at most ~50 ms old
  // and an intent waits at most that long before the server integrates it.
  cvc::ariadne::holon server_holon(cvc::ariadne::holon_role::server,
                                   cvc::ariadne::room_holon_config());
  std::printf("[arisrv] avatar physics: %s\n",
              server_holon.physics() ? "Jolt rigid bodies" : "kinematic (no Jolt in this build)");
  constexpr int kRoomSubsteps = 4;
  constexpr auto kRoomStep = std::chrono::milliseconds(50); // x kRoomSubsteps = the loop's 200 ms
  // Fan the relay's OWN room-tree writes -- the room_leave retractions, room_relay_tick's
  // per-stream "ready" markers, the holon's poses -- out to EVERY client on BOTH transports.
  // The shard's publish cursor is shared by the two transports, so a plain state_tp.pump_all()
  // would drain it and the WebSocket side would send nothing (browser rosters would never drop a
  // closed tab). So relay_room_step drains the local journal ONCE and publishes each mutation to
  // every plane: publish() writes only to its own connections and never crosses the bridge, so
  // each client gets it exactly once with no echo. drain_local yields only our local writes, never
  // the clients' replicated mutations (those ride set_relay forwarding), so this cannot loop. (With
  // one plane that is state_tp.pump_all().) The AOI's pose pushes go out first, so on every
  // connection they precede the step's live writes; with --relay-journal trim the published
  // journal is erased after. With no AOI and no trim it is that drain and publish alone.
#if CVC_ENABLE_WS
  const std::vector<cvc::state_transport *> planes{&state_tp, &ws_tp};
#else
  const std::vector<cvc::state_transport *> planes{&state_tp};
#endif
  const auto room_start = std::chrono::steady_clock::now();
  const auto room_step = [&] {
    const double now_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - room_start).count();
    cvc::ariadne::conference::relay_room_step(shard, planes, aoi_relay.get(), server_holon, now_s,
                                              trimJournal);
  };
  const char *room_debug_env = std::getenv("CVC_ARISRV_ROOM_DEBUG");
  const bool room_debug =
      room_debug_env && *room_debug_env && std::strcmp(room_debug_env, "0") != 0;
  int room_debug_ticks = 0;
  // Link stats (~1 s): every connection on both planes reports its traffic and Ping/Pong round
  // trip (state_transport::link_reports); the rate meter turns the counters into rates. The relay
  // publishes its view of each participant (net.relay.*) and its totals (server.net.*) into the
  // room tree, and logs them: every link each ~5 s with CVC_ARISRV_ROOM_DEBUG, a totals line each
  // minute otherwise. Links that are not participants (peer servers, delegation) are logged too.
  cvc::state_link_rate_meter link_meter;
  int net_ticks = 0;
  const auto net_start = std::chrono::steady_clock::now();
  // The shared 3D scene (H1 slice 5, roadmap §5g): the relay is its sole writer and seeds the MVP
  // environment (ground + a reference box under a sun) once -- or, with --scene large, a 100 m
  // plane with a grid of meshes to walk between (what the hall preset and mesh LOD are for). The
  // writes are local shard mutations, so the first pass of the loop below fans them to the
  // clients, and late joiners get them in the snapshot.
  const bool seeded = scene == "large" ? cvc::ariadne::conference::seed_large_demo_scene(app, room)
                                       : cvc::ariadne::conference::seed_default_scene(app, room);
  if (seeded) {
    const auto rev = cvc::ariadne::conference::scene_revision(app, room);
    std::printf("[arisrv] seeded the shared %s scene for room '%s' (rev %llu)\n", scene.c_str(),
                room.c_str(), static_cast<unsigned long long>(rev));
    std::fflush(stdout);
  }
  // The AOI's parameters, once, under conf.rooms.<room>.aoi: clients derive their audio gain from
  // them. With --aoi off (and --s7 off) they are written disabled, so every peer plays at full
  // gain -- also on a client that lived through an earlier relay's AOI and kept its leaves.
  {
    cvc::ariadne::conference::aoi_params published = aoiParams;
    published.enabled = aoi_relay != nullptr;
    cvc::ariadne::conference::write_aoi_params(app, room, published);
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
      // server-authoritative room+scene tree rides on. It sends each client only what it does not
      // already hold (a diff), and the whole tree only to a client that asks for a full refresh.
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
    if (++net_ticks % 5 == 0) {
      std::vector<cvc::state_link_report> links = state_tp.link_reports();
#if CVC_ENABLE_WS
      for (cvc::state_link_report &r : ws_tp.link_reports())
        links.push_back(std::move(r));
#endif
      const double now_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - net_start).count();
      const auto rates = link_meter.update(links, now_s);
      cvc::ariadne::conference::server_net total;
      std::set<std::string> present;
      for (const auto &p : cvc::ariadne::conference::room_roster(app, room))
        present.insert(p.pid);
      for (const cvc::state_link_report &r : links) {
        const auto it = rates.find(cvc::state_link_rate_meter::key_of(r));
        const double in_kbps = it == rates.end() ? 0.0 : it->second.in_bps * 8.0 / 1000.0;
        const double out_kbps = it == rates.end() ? 0.0 : it->second.out_bps * 8.0 / 1000.0;
        total.in_kbps += in_kbps;
        total.out_kbps += out_kbps;
        ++total.links;
        if (present.count(r.remote_node_id))
          cvc::ariadne::conference::write_relay_net(app, room, r.remote_node_id,
                                                    {r.stats.rtt_ms, in_kbps, out_kbps});
      }
      cvc::ariadne::conference::write_server_net(app, room, total);
      const bool detail = room_debug && net_ticks % 25 == 0;
      if (detail || net_ticks % 300 == 0) {
        std::printf("[arisrv] net: %d link(s) | in %.1f kb/s | out %.1f kb/s\n", total.links,
                    total.in_kbps, total.out_kbps);
        if (detail)
          for (const cvc::state_link_report &r : links) {
            const auto it = rates.find(cvc::state_link_rate_meter::key_of(r));
            std::printf("[arisrv]   %-4s %-16s %-7s rtt %6.1f ms (min %.1f) | in %7.1f kb/s | out "
                        "%7.1f kb/s | idle %.1fs\n",
                        r.transport.c_str(), r.remote_node_id.c_str(),
                        r.accepted ? "client" : "peer", r.stats.rtt_ms, r.stats.rtt_min_ms,
                        it == rates.end() ? 0.0 : it->second.in_bps * 8.0 / 1000.0,
                        it == rates.end() ? 0.0 : it->second.out_bps * 8.0 / 1000.0, r.idle_s);
          }
        std::fflush(stdout);
      }
    }
    if (room_debug && ++room_debug_ticks % 25 == 0) {
      const auto roster = cvc::ariadne::conference::room_roster(app, room);
      std::printf("[arisrv] room '%s': %zu present, holon tick %llu, %llu intent(s) vetoed\n",
                  room.c_str(), roster.size(), static_cast<unsigned long long>(server_holon.tick()),
                  static_cast<unsigned long long>(server_holon.vetoes()));
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
      // The AOI over the same ~5 s: the pairs in each tier now, the pose pushes and the media
      // frames the gate passed whole, as thumbnails or not at all, and the room-tree deliveries
      // the registries filtered; then the transports' own S7 counters (cumulative) and the
      // relay journal's size (about 0 while it is trimmed).
      unsigned long long ws_withheld = 0, ws_gated = 0;
#if CVC_ENABLE_WS
      ws_withheld = static_cast<unsigned long long>(ws_tp.total_relay_withheld());
      ws_gated = static_cast<unsigned long long>(ws_tp.total_media_gated());
#endif
      cvc::ariadne::conference::room_aoi_relay::stats st;
      if (aoi_relay)
        st = aoi_relay->take_stats();
      std::printf(
          "[arisrv] aoi %s: pairs inner %zu middle %zu outer %zu | pushes %llu (%llu entries) | "
          "media full %llu thumb %llu dropped %llu | state filtered %llu | relay withheld grpc "
          "%llu ws %llu | media gated grpc %llu ws %llu, stalled %llu | journal %zu\n",
          aoiMode.c_str(), st.inner, st.middle, st.outer,
          static_cast<unsigned long long>(st.pushes),
          static_cast<unsigned long long>(st.push_entries),
          static_cast<unsigned long long>(st.media_full),
          static_cast<unsigned long long>(st.media_thumb),
          static_cast<unsigned long long>(st.media_dropped),
          static_cast<unsigned long long>(st.state_filtered),
          static_cast<unsigned long long>(state_tp.total_relay_withheld()), ws_withheld,
          static_cast<unsigned long long>(state_tp.total_media_gated()), ws_gated,
          static_cast<unsigned long long>(state_tp.total_media_stall_dropped()),
          shard.journal().size());
      std::fflush(stdout);
    }
    room_step();
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
    // The rest of this pass's room steps (the first ran above): advance the holon and fan its
    // poses between the loop's slower housekeeping.
    for (int i = 1; i < kRoomSubsteps && !g_stop.load(); ++i) {
      std::this_thread::sleep_for(kRoomStep);
      cvc::ariadne::server_pump(server_holon, app, room);
      room_step();
    }
    std::this_thread::sleep_for(kRoomStep);
  }

  std::printf("[arisrv] shutting down (relayed %zu stream(s), %llu frames forwarded)\n",
              relay.stream_count(), static_cast<unsigned long long>(relay.frames_forwarded()));
  // The AOI first, while the transports still run: it clears the media gates and the registry
  // filters it installed.
  aoi_relay.reset();
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

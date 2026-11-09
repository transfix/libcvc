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
// Core libcvc + gRPC only — NO cvcGL/VTK/GL — so it runs on a plain headless VM. With --doc it
// also runs a server document: room rules, bots and checks in the DSL, and the namespaces it
// publishes to every client (cvc/ariadne/server_document.h).
//
//   arisrv --room standup --state-listen 0.0.0.0:7700 --media-listen 0.0.0.0:7701

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib> // std::getenv (CVC_ARISRV_ROOM_DEBUG, CVC_ARISRV_S7, CVC_ARISRV_AOI)
#include <cstring>
#include <cvc/ariadne/conference/room_aoi.h>        // aoi_params, the presets, write_aoi_params
#include <cvc/ariadne/conference/room_aoi_relay.h>  // room_aoi_relay, relay_room_step
#include <cvc/ariadne/conference/room_avatar.h>     // seed_avatar_catalog (what avatars may wear)
#include <cvc/ariadne/conference/room_chat.h>       // room chat and direct messages
#include <cvc/ariadne/conference/room_intrinsics.h> // room_leave (retract a departed peer)
#include <cvc/ariadne/conference/room_lane.h>       // the pose lane + lane signaling
#include <cvc/ariadne/conference/room_media_intrinsics.h> // room_relay_tick
#include <cvc/ariadne/conference/room_net.h>        // per-participant + room link-quality stats
#include <cvc/ariadne/conference/room_pose_relay.h> // room_pose_relay (avatar pose snapshots)
#include <cvc/ariadne/conference/room_rules.h>      // the world's rules (write_room_rules)
#include <cvc/ariadne/conference/room_rungs.h>      // each viewer's video rung, adapted to its link
#include <cvc/ariadne/conference/room_scene.h>      // the room's world (write_room_world)
#include <cvc/ariadne/conference/room_write_gate.h> // bound writes: a client writes only itself
#include <cvc/ariadne/holon.h>                      // server-authoritative avatar holon (H1)
#include <cvc/ariadne/holon_sync.h>      // server_pump: intent -> holon -> authoritative pose
#include <cvc/ariadne/host_cli.h>        // --set KEY=VALUE, as ari and arigl take it
#include <cvc/ariadne/loader.h>          // load_file: the stock world documents
#include <cvc/ariadne/server_document.h> // --doc: the server document and what it publishes
#include <cvc/ariadne/stream/stream_relay.h>
#include <cvc/ariadne/uri.h>         // cvc://worlds/... (register_cvc_uri_handler)
#include <cvc/ariadne/world_rules.h> // the world: block (world_rules_from)
#include <cvc/core/app.h>
#include <cvc/media/media_router.h> // protocol-agnostic media hub (WS + gRPC + future protocols)
#include <cvc/net/fast_lane_rtc.h>  // the unreliable lane (WebRTC data channels)
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_device_key.h> // device-key pins (trust on first use)
#include <cvc/state/state_identity.h>   // valid_room_token
#include <cvc/state/state_stream_transport_grpc.h>
#include <cvc/state/state_transport_grpc.h>
#if CVC_ENABLE_WS
#include <cvc/state/state_transport_ws.h> // browser (wasm) clients join over WebSocket, bridged to gRPC
#endif
#include <fstream>
#include <initializer_list>
#include <map>
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
  //   --media-delay-ms N         gRPC: skip a receiver whose stream has queued N ms past its base
  //                              round trip (video; poses at 2N, audio at 4N; default 100; 0 = off)
  //   --admission-hold-ms N      hold a joiner's bootstrap until the area of interest has its
  //                              filter, so its admit snapshot is filtered too; at most N ms, then
  //                              unfiltered (default 2000; 0 = off; only with an area of interest)
  //   --scene default|large      the world the room starts in: cvc://worlds/default.ari (the 20 m
  //                              room) or cvc://worlds/large.ari (a 100 m plane of LOD meshes).
  //                              A --doc whose document has a scene: is the world instead
  // Debugging (repeatable, in order):
  //   --set KEY=VALUE            write VALUE to the state key KEY (from the app's root, e.g.
  //                              conf.rooms.main.topic) once the room's world and policies are
  //                              written, so it overrides them, and before a server document's
  //                              init:; a conf.* key reaches every client
  // Bound writes (not an S7 switch, so --s7 off leaves it on):
  //   --bind-writes on|off       each client connection speaks only as the pid of its Hello, held
  //                              by one live connection at a time, and writes only its own
  //                              participant subtree minus the relay's keys (room_write_gate).
  //                              Default on; off takes any client's writes, as before
  // Room token (§5f; not an S7 switch):
  //   --room-token-file PATH     admit only clients that present the token in PATH (its first
  //                              line), or CVC_ARISRV_ROOM_TOKEN: gRPC as a bearer token, WebSocket
  //                              as a cvc-token.<token> subprotocol (state_identity.h has the
  //                              rules). A client without it is refused before it is sent
  //                              anything. Unset: an open room, as before
  // Device keys (§5f; not an S7 switch). The relay always checks them: its Hello challenges each
  // client, a client answers with its per-relay device key (state_device_key.h), and the first
  // key a device id shows is pinned to it. A participant's device tag is written only for a
  // device that proved its pinned key.
  //   --device-pins PATH         keep the pins in PATH (one "<device id> <key>" line each, added
  //                              as devices first show), so they outlive a restart. Unset: pins
  //                              are kept in memory, for this run only
  // Per-viewer video (§5j; not an S7 switch). The relay sends each viewer every camera at a rung --
  // full (as sent), half or quarter (re-encoded smaller here, media_rungs.h), thumb (quarter at
  // 2 fps) or off -- adapted to that viewer's link (room_rung_controller). Audio is always sent.
  //   --adapt on|off             adapt each viewer's rung to its link (default on); off: every
  //                              viewer at --video-max
  //   --video-max RUNG           the best rung any viewer gets (default full)
  //   --video-floor RUNG         the lowest adaptation goes (default thumb; off allows audio only)
  //   --viewer-kbps-max N        a viewer's budget, kbit/s: above it, it steps down (default 0 =
  //                              none)
  // The fast lanes (docs/roadmap/CONFERENCE_UNRELIABLE_TRANSPORT.md; needs CVC_ENABLE_RTC). A
  // client may open unreliable, unordered WebRTC data channels to the relay, offered over its
  // reliable connection: a pose lane, which carries its pose snapshots (room_pose_relay's stream,
  // acknowledged so a lost one is never waited for), and a media lane. A lost packet is superseded
  // by the next instead of holding anything up. Off unless --lane-listen.
  //   --lane-listen ADDR:PORT    the one UDP port every lane shares (e.g. 0.0.0.0:7703)
  //   --lane-advertise IP[,IP]   the address(es) clients reach that port at, put in each answer
  //                              (the public IP of a NAT that forwards the port); default: only
  //                              the host's own addresses
  //   --lane-media on|off        a media lane beside each pose lane (default off: media stays on
  //                              the reliable connections and poses alone ride the lanes, which
  //                              measured best under loss -- SCTP's congestion control holds
  //                              video longer than TCP's; the design doc's section 7)
  // Avatars (ARIADNE_CONFERENCE_ROADMAP.md §5k). The world sets the policy every avatar moves
  // under, the same for everyone (its world: block, ARIADNE_WORLDS.md WD2), and the relay publishes
  // it so clients predict under it too. These override the world's:
  //   --avatar-energy on|off     an energy budget for every avatar's movement; the stock worlds':
  //                              a push of at most 840 N, 1.2 kW at most, a 30 kJ reserve refilled
  //                              at 300 W, and carrying the body along costs 0.2 m g |v| (on)
  //   --avatar-empty rest|stop   when an avatar's reserve runs out: rest leaves resting power (it
  //                              slows to what that sustains); stop holds it until a quarter has
  //                              refilled (the stock worlds': rest)
  // The server document (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §6.1, §6.5.3; not an S7 switch):
  //   --doc PATH                 run the .ari document at PATH on the relay, headless: its init:
  //                              once the room is up, its on_tick: every room step (60 Hz), under
  //                              the DSL's step and time caps (ariadne.limits.*, which --set
  //                              reaches). It binds under ui.docs.arisrv, which no client is sent,
  //                              and reads the room through `room` (read-only). Its server: block's
  //                              publish: list names the namespaces every client is sent; only the
  //                              document writes them, so it needs --bind-writes on. (quit [code])
  //                              stops the relay with that exit status; requires: [relay] holds
  //                              only there. --set is applied before the seeds and the document
  //                              (WD0 in ARIADNE_WORLDS.md moves it last). Unset: no document
  const char *s7_env = std::getenv("CVC_ARISRV_S7");
  const char *aoi_env = std::getenv("CVC_ARISRV_AOI");
  // CVC_ARISRV_S7: 0/off or 1/on (unset: on).
  std::string s7Arg = !s7_env || !*s7_env         ? "on"
                      : !std::strcmp(s7_env, "0") ? "off"
                      : !std::strcmp(s7_env, "1") ? "on"
                                                  : s7_env;
  std::string aoiMode = (aoi_env && *aoi_env) ? aoi_env : ""; // empty: the scene's default
  std::string relayGateArg = "on", relayJournal = "trim", scene = "default", bindWritesArg = "on";
  std::string roomTokenFile;
  bool roomTokenFlag = false; // --room-token-file was given (even with no path)
  std::string devicePinsFile;
  bool devicePinsFlag = false; // --device-pins was given (even with no path)
  std::string adaptArg = "on", videoMaxArg = "full", videoFloorArg = "thumb", viewerKbpsArg = "0";
  std::string aoiVideoMaxArg = "0", mediaStallArg = "40", mediaDelayArg = "100",
              admissionHoldArg = "2000";
  std::string laneListen, laneAdvertise, laneMediaArg = "off";
  std::string avatarEnergyArg, avatarEmptyArg; // "": the world's
  bool sceneFlag = false; // --scene was given (a --doc with a scene: is the world instead)
  std::string docPath;
  bool docFlag = false;                                       // --doc was given (even with no path)
  std::vector<std::pair<std::string, std::string>> stateSets; // --set KEY=VALUE
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
    else if (!std::strcmp(a, "--media-delay-ms"))
      mediaDelayArg = next("");
    else if (!std::strcmp(a, "--admission-hold-ms"))
      admissionHoldArg = next("");
    else if (!std::strcmp(a, "--scene")) {
      scene = next("default");
      sceneFlag = true;
    } else if (!std::strcmp(a, "--bind-writes"))
      bindWritesArg = next("on");
    else if (!std::strcmp(a, "--room-token-file")) {
      roomTokenFlag = true;
      roomTokenFile = next("");
    } else if (!std::strcmp(a, "--device-pins")) {
      devicePinsFlag = true;
      devicePinsFile = next("");
    } else if (!std::strcmp(a, "--adapt"))
      adaptArg = next("on");
    else if (!std::strcmp(a, "--video-max"))
      videoMaxArg = next("");
    else if (!std::strcmp(a, "--video-floor"))
      videoFloorArg = next("");
    else if (!std::strcmp(a, "--viewer-kbps-max"))
      viewerKbpsArg = next("");
    else if (!std::strcmp(a, "--lane-listen"))
      laneListen = next("");
    else if (!std::strcmp(a, "--lane-advertise"))
      laneAdvertise = next("");
    else if (!std::strcmp(a, "--lane-media"))
      laneMediaArg = next("on");
    else if (!std::strcmp(a, "--avatar-energy"))
      avatarEnergyArg = next("on");
    else if (!std::strcmp(a, "--avatar-empty"))
      avatarEmptyArg = next("rest");
    else if (!std::strcmp(a, "--doc")) {
      docFlag = true;
      docPath = next("");
    } else if (!std::strcmp(a, "--set")) {
      std::pair<std::string, std::string> kv;
      const char *v = next("");
      if (!cvc::ariadne::parse_state_set(v, kv)) {
        std::fprintf(stderr, "[arisrv] --set expects KEY=VALUE, got '%s'\n", v);
        return 2;
      }
      stateSets.push_back(std::move(kv));
    }
  }
  if (aoiMode.empty())
    aoiMode = scene == "large" ? "hall" : "conference";
  if (!check_choice("--s7", s7Arg, {"on", "off"}) ||
      !check_choice("--aoi", aoiMode, {"off", "conference", "hall"}) ||
      !check_choice("--relay-gate", relayGateArg, {"on", "off"}) ||
      !check_choice("--relay-journal", relayJournal, {"trim", "keep"}) ||
      !check_choice("--scene", scene, {"default", "large"}) ||
      !check_choice("--bind-writes", bindWritesArg, {"on", "off"}) ||
      !check_choice("--adapt", adaptArg, {"on", "off"}) ||
      !check_choice("--video-max", videoMaxArg, {"full", "half", "quarter", "thumb", "off"}) ||
      !check_choice("--video-floor", videoFloorArg, {"full", "half", "quarter", "thumb", "off"}) ||
      !check_choice("--lane-media", laneMediaArg, {"on", "off"}) ||
      (!avatarEnergyArg.empty() &&
       !check_choice("--avatar-energy", avatarEnergyArg, {"on", "off"})) ||
      (!avatarEmptyArg.empty() &&
       !check_choice("--avatar-empty", avatarEmptyArg, {"rest", "stop"})))
    return 2;
  long aoiVideoMaxN = 0, mediaStallMs = 40, mediaDelayMs = 100, admissionHoldMs = 2000;
  if (!check_count("--aoi-video-max", aoiVideoMaxArg, aoiVideoMaxN) ||
      !check_count("--media-stall-ms", mediaStallArg, mediaStallMs) ||
      !check_count("--media-delay-ms", mediaDelayArg, mediaDelayMs) ||
      !check_count("--admission-hold-ms", admissionHoldArg, admissionHoldMs))
    return 2;
  long viewerKbps = 0;
  if (!check_count("--viewer-kbps-max", viewerKbpsArg, viewerKbps))
    return 2;
  cvc::ariadne::conference::rung_params rungs;
  rungs.adapt = adaptArg == "on";
  (void)cvc::parse_video_rung(videoMaxArg, rungs.max_rung);
  (void)cvc::parse_video_rung(videoFloorArg, rungs.floor_rung);
  rungs.viewer_kbps_max = static_cast<double>(viewerKbps);
  // Each viewer's rung (the controller, once a second) and the camera frames shaped to it (the
  // shaper, on the transports' reader threads). Declared before the transports, so they outlive
  // every fan that calls the shaper.
  cvc::ariadne::conference::room_rung_controller rung_ctl(rungs);
  cvc::fast_lane_server_config laneCfg;
  if (!laneListen.empty()) {
    const std::size_t colon = laneListen.rfind(':');
    long port = 0;
    if (colon == std::string::npos ||
        !check_count("--lane-listen (port)", laneListen.substr(colon + 1), port) || port < 1 ||
        port > 65535)
      return 2;
    if (!cvc::fast_lane_rtc_available()) {
      std::fprintf(stderr, "arisrv: --lane-listen: this build has no WebRTC (CVC_ENABLE_RTC)\n");
      return 2;
    }
    laneCfg.bind_address = laneListen.substr(0, colon);
    laneCfg.port = static_cast<std::uint16_t>(port);
    for (std::size_t at = 0; at < laneAdvertise.size();) {
      const std::size_t comma = laneAdvertise.find(',', at);
      const std::string ip =
          laneAdvertise.substr(at, comma == std::string::npos ? comma : comma - at);
      if (!ip.empty())
        laneCfg.advertise.push_back(ip);
      at = comma == std::string::npos ? laneAdvertise.size() : comma + 1;
    }
  }
  cvc::media_rung_shaper shaper;
  // Set once, before the loop, when the lane is on: a viewer whose lane is open gets its media
  // there, so the reliable endpoints send it none.
  std::atomic<cvc::fast_lane_server *> laneOf{nullptr};
  const auto shape = [&shaper, &laneOf](const cvc::media_frame &f, const std::string &viewer) {
    if (cvc::fast_lane_server *l = laneOf.load(); l && l->lane_open(viewer))
      return cvc::media_shape{true, nullptr};
    return shaper.shape(f, viewer);
  };
  const auto lane_shape = [&shaper](const cvc::media_frame &f, const std::string &viewer) {
    return shaper.shape(f, viewer);
  };
  const int aoiVideoMax = static_cast<int>((std::min)(aoiVideoMaxN, 1000000L));
  const bool s7 = s7Arg == "on";
  if (!s7)
    aoiMode = "off";
  const bool relayGate = s7 && relayGateArg == "on";
  const bool trimJournal = s7 && relayJournal == "trim";
  const long stallMs = s7 ? mediaStallMs : 0L;
  // The hold waits for a filter only the area of interest installs.
  const long holdMs = aoiMode != "off" ? (std::min)(admissionHoldMs, 600000L) : 0L;
  // One gate for both transports, so a pid held on one cannot be claimed on the other.
  std::shared_ptr<cvc::state_inbound_gate> bindGate;
  if (bindWritesArg == "on")
    bindGate = std::make_shared<cvc::ariadne::conference::room_write_gate>(room);
  // The room token: from the file (its first line, trimmed), else the environment. Not taken from
  // the command line, which other users of the host can read. A token asked for must be one: a
  // flag with no path, an unreadable or blank file, or a set but empty variable stops arisrv here
  // instead of opening the room.
  std::string roomToken;
  bool tokenAsked = false;
  if (roomTokenFlag) {
    tokenAsked = true;
    std::ifstream in(roomTokenFile);
    if (roomTokenFile.empty() || !in || !std::getline(in, roomToken)) {
      std::fprintf(stderr, "arisrv: --room-token-file: cannot read '%s'\n", roomTokenFile.c_str());
      return 2;
    }
    while (!roomToken.empty() && std::isspace(static_cast<unsigned char>(roomToken.back())))
      roomToken.pop_back();
  } else if (const char *t = std::getenv("CVC_ARISRV_ROOM_TOKEN")) {
    tokenAsked = true;
    roomToken = t;
  }
  if (tokenAsked && !cvc::valid_room_token(roomToken)) {
    std::fprintf(stderr, "arisrv: the room token must be 16 to 256 characters from "
                         "[A-Za-z0-9._~-]\n");
    return 2;
  }
  // The device-key pins, one store for both transports (a device id pinned on one is checked on
  // the other). A pin file asked for must be one arisrv can add to.
  if (devicePinsFlag) {
    std::ofstream probe(devicePinsFile, std::ios::app);
    if (devicePinsFile.empty() || !probe) {
      std::fprintf(stderr, "arisrv: --device-pins: cannot write '%s'\n", devicePinsFile.c_str());
      return 2;
    }
  }
  const auto devicePins = std::make_shared<cvc::state_device_pins>(devicePinsFile);

  cvc::app app;

  // The server document's server: block, read now, before anything listens: the shard below is
  // scoped by it, and a document that does not parse stops arisrv here. It runs once the room is
  // up (below the seeding).
  std::unique_ptr<cvc::ariadne::ServerDocument> serverDoc;
  if (docFlag) {
    if (docPath.empty()) {
      std::fprintf(stderr, "arisrv: --doc expects a path\n");
      return 2;
    }
    serverDoc = std::make_unique<cvc::ariadne::ServerDocument>(app, room);
    std::vector<std::string> warnings;
    if (const std::string error = serverDoc->read(docPath, &warnings); !error.empty()) {
      std::fprintf(stderr, "arisrv: --doc: %s\n", error.c_str());
      return 2;
    }
    for (const std::string &w : warnings)
      std::fprintf(stderr, "arisrv: --doc: %s\n", w.c_str());
    // A published namespace is the server's alone because bound writes refuse every client write
    // outside its own participant. Without them a client could write it.
    if (!serverDoc->block().publish.empty() && bindWritesArg != "on") {
      std::fprintf(stderr,
                   "arisrv: --doc: %s publishes namespaces, which needs bound writes "
                   "(--bind-writes on): without them a client could write there too\n",
                   docPath.c_str());
      return 2;
    }
  }

  // The room's world (ARIADNE_WORLDS.md WD1), chosen now so that a bad one stops the relay before
  // it listens; it is published below, where the room is set up. A --doc whose document's scene:
  // has nodes or lights is the world. Otherwise it is the stock world --scene names,
  // cvc://worlds/<name>.ari, or that world's C++ twin when the file can't be found or read.
  //
  // The world's rules (WD2): the avatars' archetypes, policy and controllers from its world: block,
  // gravity from its scene's physics:. A --doc that has a world: block sets them even when the
  // scene is a stock world's; otherwise they are the world document's (the C++ twin's are
  // default_world_rules()). --avatar-energy and --avatar-empty override its energy policy.
  cvc::ariadne::register_cvc_uri_handler();
  cvc::ariadne::register_world_block();
  namespace rconf = cvc::ariadne::conference;
  rconf::room_world world;
  std::string worldFrom;
  std::vector<std::string> worldWarnings;
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  std::string rulesFrom;
  const bool docWorld =
      serverDoc && (!serverDoc->scene().nodes.empty() || !serverDoc->scene().lights.empty());
  if (serverDoc && (docWorld || serverDoc->world())) {
    if (const std::string e = cvc::ariadne::world_rules_from(serverDoc->scene(), serverDoc->world(),
                                                             rules, &worldWarnings);
        !e.empty()) {
      std::fprintf(stderr, "arisrv: --doc: %s: %s\n", docPath.c_str(), e.c_str());
      return 2;
    }
    rulesFrom = docPath;
  }
  if (docWorld) {
    if (sceneFlag) {
      std::fprintf(stderr,
                   "arisrv: --scene %s and --doc %s both name the room's world (the document has "
                   "a scene:); drop one\n",
                   scene.c_str(), docPath.c_str());
      return 2;
    }
    world = rconf::room_world_from_scene(serverDoc->scene(), &worldWarnings);
    worldFrom = docPath;
  } else {
    worldFrom = "cvc://worlds/" + scene + ".ari";
    const cvc::ariadne::UriResult found = cvc::ariadne::resolve(worldFrom);
    cvc::ariadne::LoadResult lr;
    if (found.ok)
      lr = cvc::ariadne::load_file(found.canonical);
    if (found.ok && lr.ok) {
      world = rconf::room_world_from_scene(lr.scene, &worldWarnings);
      for (const std::string &w : lr.warnings)
        worldWarnings.push_back(w);
      if (rulesFrom.empty()) {
        // A stock world whose rules do not hold is a libcvc bug: say so and run its twin's.
        if (const std::string e = cvc::ariadne::world_rules_from(
                lr.scene, cvc::ariadne::find_world_block(lr), rules, &worldWarnings);
            !e.empty()) {
          worldWarnings.push_back(worldFrom + ": " + e + "; running the default rules instead");
          rules = cvc::ariadne::default_world_rules();
          rulesFrom = worldFrom + " (the default rules)";
        } else {
          rulesFrom = worldFrom;
        }
      }
    } else {
      worldWarnings.push_back(worldFrom + ": " + (found.ok ? lr.error : found.error) +
                              "; seeding its C++ twin instead");
      world = scene == "large" ? rconf::large_world() : rconf::default_world();
      worldFrom += " (its C++ twin)";
      if (rulesFrom.empty())
        rulesFrom = worldFrom; // default_world_rules(), as rules already holds
    }
  }
  if (!avatarEnergyArg.empty())
    rules.policy.energy.enabled = avatarEnergyArg == "on";
  if (!avatarEmptyArg.empty())
    rules.policy.energy.empty = avatarEmptyArg == "stop"
                                    ? cvc::ariadne::avatar_energy_policy::when_empty::stop
                                    : cvc::ariadne::avatar_energy_policy::when_empty::rest;

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
  // can be trimmed as it is published (relay_room_step). The admission hold keeps that snapshot
  // until the area of interest has the joiner's filter, so it is filtered like the live stream.
  state_tp.set_relay_requires_interest(relayGate);
  state_tp.set_backfill_on_connect(!trimJournal);
  state_tp.set_media_stall_drop(std::chrono::milliseconds(stallMs));
  state_tp.set_media_delay_drop(std::chrono::milliseconds(mediaDelayMs));
  state_tp.set_admission_hold(std::chrono::milliseconds(holdMs));
  state_tp.set_inbound_gate(bindGate);
  state_tp.set_auth_config({roomToken, std::string()});
  state_tp.set_device_pins(devicePins);
  state_tp.set_media_shaper(shape);
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
  // And it SENDS only the room tree, plus the namespaces a server document publishes: the relay's
  // tree also holds that document's own state (ui.docs.arisrv), which no client is sent. Clients
  // took nothing else before either, since each one's interest takes only conf.
  if (serverDoc)
    serverDoc->scope(shard);
  else
    shard.add_publish_prefix("conf");
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
  ws_tp.set_admission_hold(std::chrono::milliseconds(holdMs));
  ws_tp.set_inbound_gate(bindGate);
  ws_tp.set_auth_config({roomToken, std::string()});
  ws_tp.set_device_pins(devicePins);
  ws_tp.set_media_shaper(shape);
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

  // Room chat and direct messages (room_chat.h): the relay keeps the history and pushes each client
  // what it may read. A DM is pushed only to the connections holding its two pids, which takes
  // bound writes: without them any connection could claim a pid and read its DMs, so they are off.
  cvc::ariadne::conference::chat_limits chatLimits;
  chatLimits.dms = bindGate != nullptr;
  auto chat =
      std::make_unique<cvc::ariadne::conference::room_chat_relay>(app, shard, room, chatLimits);
  chat->add_transport(&state_tp);
#if CVC_ENABLE_WS
  chat->add_transport(&ws_tp);
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
  std::printf("[arisrv] S7 %s | aoi %s (full-rate cameras: %d, 0 = all) | relay gate %s | journal "
              "%s | grpc media stall drop %ld ms, delay drop %ld ms | scene %s | bound writes %s | "
              "room token %s\n",
              s7 ? "on" : "off (the pre-S7 relay)", aoiMode.c_str(), aoiVideoMax,
              relayGate ? "on" : "off", trimJournal ? "trim (no backfill)" : "keep (backfill)",
              stallMs, mediaDelayMs, scene.c_str(),
              bindGate ? "on" : "off (any client's writes are taken)",
              roomToken.empty() ? "off (an open room)" : "on");
  std::printf("[arisrv] chat: room%s | history in memory: %zu room messages, %zu per "
              "conversation\n",
              chatLimits.dms ? " and direct messages" : " only (direct messages need bound writes)",
              chatLimits.room_keep, chatLimits.dm_keep);
  if (!cvc::device_keys_available())
    std::printf("[arisrv] device keys: unavailable (no OpenSSL in this build): no device tags\n");
  else if (devicePins->path().empty())
    std::printf("[arisrv] device keys: checked; pins in memory (this run only)\n");
  else
    std::printf("[arisrv] device keys: checked; %zu pin(s) in %s\n", devicePins->size(),
                devicePins->path().c_str());
  const auto kbps_text = [&] {
    return rungs.viewer_kbps_max > 0.0 ? std::to_string(viewerKbps) + " kb/s" : std::string("none");
  };
  std::printf("[arisrv] video per viewer: %s | best %s | floor %s | budget %s | transcode %s\n",
              rungs.adapt ? "adapted to each link" : "fixed", cvc::video_rung_name(rungs.max_rung),
              cvc::video_rung_name(rungs.floor_rung), kbps_text().c_str(),
              cvc::jpeg_transcode_available() ? "libjpeg-turbo"
                                              : "unavailable (smaller rungs send frames as is)");
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
  // every participant's proposed intent from the replicated tree and integrates it on
  // its own clock (a stamped intent at the tick the client applied it). The relay loop
  // paces ~5 Hz; the room steps kRoomSubsteps times per pass (60 Hz), so an intent waits
  // at most ~17 ms before the server integrates it. Each step the pose relay sends every
  // viewer that is due its snapshot of the poses it sees (room_pose_relay: 60, 30 or 20 Hz
  // by its round trip); the tree's copy of the poses is written every kTreePoseSteps steps
  // (1 Hz), for late joiners and the log -- and an avatar standing still is re-stamped there only
  // every kTreePoseHeartbeatS, since the stream's own heartbeat is what keeps clients' clocks.
  cvc::ariadne::holon server_holon(cvc::ariadne::holon_role::server,
                                   cvc::ariadne::room_holon_config());
  std::printf("[arisrv] avatar physics: %s\n",
              server_holon.physics() ? "Jolt rigid bodies" : "kinematic (no Jolt in this build)");
  constexpr int kRoomSubsteps = 12;
  constexpr auto kRoomStep = std::chrono::microseconds(16667); // x kRoomSubsteps = the 200 ms pass
  constexpr std::uint64_t kTreePoseSteps = 60;
  constexpr double kTreePoseHeartbeatS = 5.0;
  cvc::ariadne::conference::room_pose_relay::config pose_cfg;
  pose_cfg.heartbeat_ticks =
      static_cast<std::uint64_t>(std::llround(1.0 / server_holon.clock().fixed_dt()));
  pose_cfg.push_interval_ticks = cvc::ariadne::conference::aoi_pose_push_interval_ticks(
      aoiParams, 1.0 / server_holon.clock().fixed_dt());
  pose_cfg.keyframe_ticks = 2 * pose_cfg.heartbeat_ticks;
  cvc::ariadne::conference::room_pose_relay pose_relay(pose_cfg);
  pose_relay.add_transport(&state_tp);
#if CVC_ENABLE_WS
  pose_relay.add_transport(&ws_tp);
#endif
  std::uint64_t room_steps = 0;
  const auto pump_room = [&] {
    cvc::ariadne::server_pump(server_holon, app, room, -1.0, room_steps % kTreePoseSteps == 0,
                              kTreePoseHeartbeatS);
  };
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
  // The fast lanes, two per participant on the one UDP port: the pose lane (room_pose_relay's
  // snapshots, from an acked stream) and the media lane (an endpoint of the media hub like the
  // others: the area of interest's gate and the rung shaper apply to it too). Apart, so poses never
  // queue behind video in one SCTP congestion window.
  std::unique_ptr<cvc::fast_lane_server> lanes, mediaLanes;
  std::unique_ptr<cvc::ariadne::conference::room_lane_relay> laneRelay;
  if (!laneListen.empty()) {
    cvc::fast_lane_server_config poseCfg = laneCfg, mediaCfg = laneCfg;
    poseCfg.transport_name = "webrtc/pose";
    mediaCfg.transport_name = "webrtc/media";
    lanes = std::make_unique<cvc::fast_lane_server>(poseCfg);
    if (laneMediaArg == "on") {
      mediaLanes = std::make_unique<cvc::fast_lane_server>(mediaCfg);
      mediaLanes->set_media_shaper(lane_shape);
      mrouter.add_endpoint(mediaLanes.get());
      if (aoi_relay)
        aoi_relay->add_media_endpoint(mediaLanes.get());
      laneOf.store(mediaLanes.get());
    }
    laneRelay = std::make_unique<cvc::ariadne::conference::room_lane_relay>(
        shard, planes, room, *lanes, mediaLanes.get(), pose_relay);
    pose_relay.set_lane(laneRelay.get()); // a viewer with an open pose lane gets its stream there
    std::string adv;
    for (const std::string &a : laneCfg.advertise)
      adv += (adv.empty() ? "" : ",") + a;
    std::printf("[arisrv] fast lanes: udp %s:%u | advertised %s | poses %s | media %s\n",
                laneCfg.bind_address.c_str(), static_cast<unsigned>(laneCfg.port),
                adv.empty() ? "(host addresses)" : adv.c_str(),
                "on the pose lane (acked snapshots)",
                mediaLanes ? "on the lane" : "on the reliable connection (--lane-media off)");
    std::fflush(stdout);
  }
  const auto room_start = std::chrono::steady_clock::now();
  const auto room_step = [&] {
    const double now_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - room_start).count();
    // Chat first: its pushes go out now, and the room head it writes rides this step's drain, so
    // a client never sees the head before the message.
    chat->step(now_s);
    cvc::ariadne::conference::relay_room_step(shard, planes, aoi_relay.get(), server_holon, now_s,
                                              trimJournal);
    const std::shared_ptr<const cvc::ariadne::conference::aoi_snapshot> tiers =
        aoi_relay ? aoi_relay->snapshot() : nullptr;
    pose_relay.step(server_holon, room_steps++, tiers.get());
  };
  const char *room_debug_env = std::getenv("CVC_ARISRV_ROOM_DEBUG");
  const bool room_debug =
      room_debug_env && *room_debug_env && std::strcmp(room_debug_env, "0") != 0;
  int room_debug_ticks = 0;
  std::uint64_t lastRefused = 0; // client frames the bound-writes gate refused, at the last log
  std::uint64_t lastUnauthorized = 0;  // connections refused for the room token, at the last log
  std::uint64_t lastProofsRefused = 0; // device proofs refused (another key, a bad signature)
  // Link stats (~1 s): every connection on both planes reports its traffic and Ping/Pong round
  // trip (state_transport::link_reports). Every net_publish_period_s (room_net.h) the rate meter
  // turns the counters into rates over that window; the relay publishes its view of each
  // participant (net.relay.*) and its totals (server.net.*) into the room tree, and logs them:
  // every link each time with CVC_ARISRV_ROOM_DEBUG, a totals line each minute otherwise. Links
  // that are not participants (peer servers, delegation) are logged too.
  cvc::state_link_rate_meter link_meter;
  int net_ticks = 0;
  // The meters' window in 200 ms passes: a whole number of the ~1 s link-stat steps.
  const long netPublishSteps = std::lround(cvc::ariadne::conference::net_publish_period_s);
  const int netPublishTicks = 5 * static_cast<int>(std::max(1L, netPublishSteps));
  const auto net_start = std::chrono::steady_clock::now();
  // The shared 3D scene (H1 slice 5, roadmap §5g), from the world chosen above (WD1): the relay is
  // its sole writer, and every client builds the room from it, its rig and shadows included (the
  // look is the world's, Q1). One revision for the whole scene. The writes are local shard
  // mutations, so the first pass of the loop below fans them to the clients, and late joiners get
  // them in the snapshot.
  rconf::write_room_world(app, room, world);
  {
    const std::size_t lights = world.env.lights.size();
    std::printf(
        "[arisrv] world: %s for room '%s': %zu objects, %zu light%s, shadows %s (rev %llu)\n",
        worldFrom.c_str(), room.c_str(), world.objects.size(), lights, lights == 1 ? "" : "s",
        world.env.shadows ? "on" : "off",
        static_cast<unsigned long long>(rconf::scene_revision(app, room)));
    for (const std::string &w : worldWarnings)
      std::printf("[arisrv] world: %s\n", w.c_str());
    std::fflush(stdout);
  }
  // What an avatar may wear (room_avatar.h): the relay supplies the catalog, each participant
  // picks from it, and every client draws the pick the same way, for itself as for everyone else.
  if (cvc::ariadne::conference::seed_avatar_catalog(app, room)) {
    const auto cat = cvc::ariadne::conference::read_avatar_catalog(app, room);
    std::printf("[arisrv] seeded the avatar catalog for room '%s' (%zu colours, %zu accessories)\n",
                room.c_str(), cat.colors.size(), cat.accessories.size());
    std::fflush(stdout);
  }
  // And the rules every avatar moves under (WD2, §5k): the world's physics, archetypes, policy
  // and controllers, four sections in force from the start (activation tick 0) and stamped with
  // this run's epoch, so a client that outlived an earlier relay takes them afresh. Every holon,
  // the relay's own and each client's, reads them from the tree (holon_sync).
  {
    rconf::write_room_rules(app, room, rules);
    const cvc::ariadne::avatar_archetype &a = rules.avatar();
    const cvc::ariadne::avatar_energy_policy &e = rules.policy.energy;
    const cvc::physics::vec3 &g = rules.physics.gravity;
    std::printf("[arisrv] rules: %s: avatars are %s (%.0f kg, %.2f x %.2f x %.2f m) on %s, at most "
                "%.1f m/s, gravity %.2f m/s^2; energy %s\n",
                rulesFrom.c_str(), a.id.c_str(), a.body.mass, a.body.width, a.body.width,
                a.body.height, rules.controller().id.c_str(), rules.policy.max_speed,
                std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z),
                !e.enabled ? "off (only the push limit)"
                : e.empty == cvc::ariadne::avatar_energy_policy::when_empty::stop
                    ? "on; an empty reserve stops"
                    : "on; an empty reserve rests");
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
  // --set (WD0): after the world and the room's policies, so it overrides them, and before the
  // server document, so its ariadne.limits.* bound the document from init: on. Before WD0 it came
  // first, so a --set of scene.rev stopped the world from being seeded and an object's leaves
  // were overwritten by the seed. The first pass of the loop below fans it out with the rest.
  cvc::ariadne::apply_state_sets(app, "", stateSets);
  // The server document, now that the room is up: its init: runs here, after the world, the
  // room's policies and --set (so it can extend or override them), and its other programs on the
  // room steps below. A load that fails stops the relay before it serves a step.
  bool docFailed = false;
  if (serverDoc) {
    serverDoc->runtime().set_warning_listener([](const std::string &w) {
      std::printf("[arisrv] server document: %s\n", w.c_str());
      std::fflush(stdout);
    });
    try {
      for (const std::string &w : serverDoc->load(docPath))
        std::printf("[arisrv] server document: %s\n", w.c_str());
      std::string published;
      for (const std::string &ns : serverDoc->block().publish)
        published += (published.empty() ? "" : ", ") + ns;
      std::printf("[arisrv] server document: %s | binds under %s, the room read-only at `room` | "
                  "publishes %s\n",
                  docPath.c_str(), serverDoc->prefix().c_str(),
                  published.empty() ? "nothing" : (published + " (to every client)").c_str());
    } catch (const std::exception &e) {
      std::fprintf(stderr, "arisrv: --doc: %s\n", e.what());
      docFailed = true;
      g_stop.store(true);
    }
    std::fflush(stdout);
  }
  // The server document's programs, each room step and ahead of the holon, so what they write
  // rides that step's fan-out. They read the holon's world clock (sim.tick, sim.time). (quit
  // [code]) stops the loop.
  const auto doc_step = [&] {
    if (!serverDoc || docFailed)
      return;
    serverDoc->step(server_holon.tick(), server_holon.clock().t());
    if (serverDoc->should_close() && !g_stop.load()) {
      std::printf("[arisrv] the server document asked to stop (quit %d)\n", serverDoc->exit_code());
      std::fflush(stdout);
      g_stop.store(true);
    }
  };
  // The room steps on a fixed 50 ms grid (sleep_until, not sleep_for after each step's work), so
  // pose samples leave evenly spaced: a client shows each peer a fixed delay behind its newest
  // sample, and every late one is a stall it has to cover. A loop that falls a step behind (a
  // stall) restarts the grid rather than running the missed steps back to back.
  auto next_room_step = std::chrono::steady_clock::now();
  const auto wait_room_step = [&]() {
    next_room_step += kRoomStep;
    const auto now = std::chrono::steady_clock::now();
    if (next_room_step + kRoomStep < now)
      next_room_step = now;
    std::this_thread::sleep_until(next_room_step);
  };
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
        chat->participant_left(nid);
        cvc::ariadne::write_intent(app, room, nid, cvc::ariadne::avatar_intent{});
        shaper.forget_viewer(nid);
        shaper.forget_publisher(nid);
        if (laneRelay)
          laneRelay->forget(nid);
        std::printf("[arisrv] participant left: %s\n", nid.c_str());
      }
      if (!gone.empty())
        std::fflush(stdout);
    }
    cvc::ariadne::conference::room_relay_tick(app, room, relay, relayed);
    // Advance the authoritative avatar holon and write each participant's pose, then fan them out
    // at once: the pose writes are local shard mutations, and the housekeeping below would
    // otherwise make this step's samples later than the other three's.
    doc_step();
    pump_room();
    room_step();
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
      const bool publish = net_ticks % netPublishTicks == 0;
      std::map<std::string, cvc::state_link_rate_meter::rate> rates;
      if (publish)
        rates = link_meter.update(links, now_s);
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
        // A participant's per-link leaves come from the link holding its pid only (bound writes):
        // another connection claiming it is not that participant.
        if (!r.holds_id || !present.count(r.remote_node_id))
          continue;
        if (publish)
          cvc::ariadne::conference::write_relay_net(app, room, r.remote_node_id,
                                                    {r.stats.rtt_ms, in_kbps, out_kbps});
        if (r.stats.rtt_ms >= 0.0) // its pose snapshot rate
          pose_relay.set_rtt(r.remote_node_id, r.stats.rtt_ms);
      }
      // A viewer whose poses ride its pose lane gets its snapshot rate from that lane's round trip.
      if (lanes)
        for (const cvc::state_link_report &r : lanes->link_reports())
          if (r.stats.rtt_ms >= 0.0 && present.count(r.remote_node_id))
            pose_relay.set_rtt(r.remote_node_id, r.stats.rtt_ms);
      cvc::ariadne::conference::write_link_devices(app, room, links, present);
      // Each viewer's video rung for its link, to the shaper and to the room (its net.relay.video,
      // which its client shows).
      // A viewer's video rung follows the link that carries its media: its lane's report, when it
      // has one, comes first (the controller takes the first report per participant).
      std::vector<cvc::state_link_report> mediaLinks =
          mediaLanes ? mediaLanes->link_reports() : std::vector<cvc::state_link_report>{};
      mediaLinks.insert(mediaLinks.end(), links.begin(), links.end());
      for (const auto &[pid, s] : rung_ctl.step(mediaLinks, now_s)) {
        shaper.set_rung(pid, s.rung);
        if (present.count(pid))
          cvc::ariadne::conference::write_relay_video_rung(app, room, pid,
                                                           cvc::video_rung_name(s.rung));
        if (s.changed) {
          std::printf("[arisrv] video: %s now %s (queue %.0f ms, %.0f%% dropped, %.0f kb/s)\n",
                      pid.c_str(), cvc::video_rung_name(s.rung), s.queue_ms, 100.0 * s.drop,
                      s.out_kbps);
          std::fflush(stdout);
        }
      }
      if (publish)
        cvc::ariadne::conference::write_server_net(app, room, total);
      // A client writing outside its own participant, or as another peer, is refused: say so (at
      // most every ~5 s) whenever the count has grown.
      if (net_ticks % 25 == 0) {
        std::uint64_t refused = state_tp.total_inbound_rejected();
#if CVC_ENABLE_WS
        refused += ws_tp.total_inbound_rejected();
#endif
        if (refused > lastRefused) {
          std::printf("[arisrv] bound writes: refused %llu client frame(s) (%llu in all): writes "
                      "outside the sender's own participant, or as another peer\n",
                      static_cast<unsigned long long>(refused - lastRefused),
                      static_cast<unsigned long long>(refused));
          std::fflush(stdout);
          lastRefused = refused;
        }
        // And a connection refused for a missing or wrong room token.
        std::uint64_t unauthorized = state_tp.total_auth_rejected();
#if CVC_ENABLE_WS
        unauthorized += ws_tp.total_auth_rejected();
#endif
        if (unauthorized > lastUnauthorized) {
          std::printf("[arisrv] room token: refused %llu connection(s) (%llu in all)\n",
                      static_cast<unsigned long long>(unauthorized - lastUnauthorized),
                      static_cast<unsigned long long>(unauthorized));
          std::fflush(stdout);
          lastUnauthorized = unauthorized;
        }
        // And a device whose proof did not check out against its pin: it stays in the room, with
        // no device tag.
        std::uint64_t proofsRefused = state_tp.total_device_proofs_refused();
#if CVC_ENABLE_WS
        proofsRefused += ws_tp.total_device_proofs_refused();
#endif
        if (proofsRefused > lastProofsRefused) {
          std::printf("[arisrv] device keys: refused %llu proof(s) (%llu in all): no device tag\n",
                      static_cast<unsigned long long>(proofsRefused - lastProofsRefused),
                      static_cast<unsigned long long>(proofsRefused));
          std::fflush(stdout);
          lastProofsRefused = proofsRefused;
        }
      }
      const bool detail = room_debug && publish;
      if (detail || (publish && net_ticks % 300 < netPublishTicks)) {
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
        if (detail && lanes) {
          cvc::fast_lane_server::totals t = lanes->counters();
          const cvc::fast_lane_server::totals m =
              mediaLanes ? mediaLanes->counters() : cvc::fast_lane_server::totals{};
          t.offers += m.offers;
          t.opened += m.opened;
          t.datagrams_in += m.datagrams_in;
          t.datagrams_out += m.datagrams_out;
          t.dropped_full += m.dropped_full;
          t.media_in = m.media_in;
          t.media_out = m.media_out;
          std::printf("[arisrv]   lanes: %llu offer(s), %llu opened | datagrams in %llu out %llu, "
                      "%llu dropped at a full buffer | media in %llu out %llu\n",
                      static_cast<unsigned long long>(t.offers),
                      static_cast<unsigned long long>(t.opened),
                      static_cast<unsigned long long>(t.datagrams_in),
                      static_cast<unsigned long long>(t.datagrams_out),
                      static_cast<unsigned long long>(t.dropped_full),
                      static_cast<unsigned long long>(t.media_in),
                      static_cast<unsigned long long>(t.media_out));
          std::vector<cvc::state_link_report> laneLinks = lanes->link_reports();
          laneLinks.insert(laneLinks.end(), mediaLinks.begin(), mediaLinks.end());
          for (const cvc::state_link_report &r : laneLinks)
            if (r.transport.rfind("webrtc", 0) == 0)
              std::printf("[arisrv]   %-12s %-16s rtt %6.1f ms (min %.1f) | in %llu B | out %llu "
                          "B | idle %.1fs\n",
                          r.transport.c_str(), r.remote_node_id.c_str(), r.stats.rtt_ms,
                          r.stats.rtt_min_ms, static_cast<unsigned long long>(r.stats.bytes_in),
                          static_cast<unsigned long long>(r.stats.bytes_out), r.idle_s);
        }
        if (detail)
          std::printf(
              "[arisrv]   video shaping: %llu variant(s) made, %llu delivered, %llu KB saved\n",
              static_cast<unsigned long long>(shaper.total_transcoded()),
              static_cast<unsigned long long>(shaper.total_shaped()),
              static_cast<unsigned long long>(shaper.total_bytes_saved() / 1024));
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
        // Facings in degrees about +Z: the pose's (from its orientation) and the one asked for.
        constexpr double kDeg = 180.0 / 3.14159265358979323846;
        std::printf("[arisrv]   %-16s pos %s (%.2f, %.2f) facing %.1f intent %s (%.2f, %.2f) "
                    "yaw %.1f\n",
                    p.pid.c_str(), has_pose ? "" : "none", s.position.x, s.position.y,
                    2.0 * std::atan2(s.rotation.z, s.rotation.w) * kDeg, has_intent ? "" : "none",
                    in.move.x, in.move.y, in.yaw * kDeg);
      }
      // The AOI over the same ~5 s: the pairs in each tier now, the pose pushes and the media
      // frames the gate passed whole, as thumbnails or not at all, and the room-tree deliveries
      // the registries filtered; then the transports' own S7 counters (cumulative) and the
      // relay journal's size (about 0 while it is trimmed).
      unsigned long long ws_withheld = 0, ws_gated = 0, ws_backlog = 0;
      // Admissions the hold delayed, and those it released without a filter (timed out).
      unsigned long long held = state_tp.total_admissions_held();
      unsigned long long held_timeouts = state_tp.total_admission_timeouts();
#if CVC_ENABLE_WS
      ws_withheld = static_cast<unsigned long long>(ws_tp.total_relay_withheld());
      ws_gated = static_cast<unsigned long long>(ws_tp.total_media_gated());
      ws_backlog = static_cast<unsigned long long>(ws_tp.total_media_backlog_dropped());
      held += ws_tp.total_admissions_held();
      held_timeouts += ws_tp.total_admission_timeouts();
#endif
      cvc::ariadne::conference::room_aoi_relay::stats st;
      if (aoi_relay)
        st = aoi_relay->take_stats();
      std::printf(
          "[arisrv] aoi %s: pairs inner %zu middle %zu outer %zu | pushes %llu (%llu entries) | "
          "media full %llu thumb %llu dropped %llu | state filtered %llu | relay withheld grpc "
          "%llu ws %llu | media gated grpc %llu ws %llu, stalled %llu, backlog %llu | "
          "admissions held %llu (timed out %llu) | journal %zu\n",
          aoiMode.c_str(), st.inner, st.middle, st.outer,
          static_cast<unsigned long long>(st.pushes),
          static_cast<unsigned long long>(st.push_entries),
          static_cast<unsigned long long>(st.media_full),
          static_cast<unsigned long long>(st.media_thumb),
          static_cast<unsigned long long>(st.media_dropped),
          static_cast<unsigned long long>(st.state_filtered),
          static_cast<unsigned long long>(state_tp.total_relay_withheld()), ws_withheld,
          static_cast<unsigned long long>(state_tp.total_media_gated()), ws_gated,
          static_cast<unsigned long long>(state_tp.total_media_stall_dropped()), ws_backlog, held,
          held_timeouts, shard.journal().size());
      const auto cs = chat->take_stats();
      std::printf(
          "[arisrv] chat: %llu post(s) (%llu direct) | %llu page(s) | %llu refused | %llu "
          "dropped | %llu push(es)\n",
          static_cast<unsigned long long>(cs.posts), static_cast<unsigned long long>(cs.dms),
          static_cast<unsigned long long>(cs.pages), static_cast<unsigned long long>(cs.refused),
          static_cast<unsigned long long>(cs.dropped), static_cast<unsigned long long>(cs.pushes));
      const auto ps = pose_relay.take_stats();
      std::printf(
          "[arisrv] poses: %llu snapshot(s) (%llu on pose lanes, %llu acks), %llu entries, "
          "%llu unsent |",
          static_cast<unsigned long long>(ps.snapshots), static_cast<unsigned long long>(ps.lane),
          static_cast<unsigned long long>(ps.acks), static_cast<unsigned long long>(ps.entries),
          static_cast<unsigned long long>(ps.unsent));
      for (const auto &p : cvc::ariadne::conference::room_roster(app, room))
        std::printf(" %s %u Hz", p.pid.c_str(),
                    60u / std::max(1u, pose_relay.interval_steps(p.pid)));
      std::printf("\n");
      std::fflush(stdout);
    }
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
      wait_room_step();
      doc_step();
      pump_room();
      room_step();
    }
    wait_room_step();
  }

  std::printf("[arisrv] shutting down (relayed %zu stream(s), %llu frames forwarded)\n",
              relay.stream_count(), static_cast<unsigned long long>(relay.frames_forwarded()));
  // The AOI first, while the transports still run: it clears the media gates and the registry
  // filters it installed. Chat too: it unsubscribes from the shard's message bus.
  aoi_relay.reset();
  chat.reset();
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
  if (docFailed)
    return 1;
  return serverDoc && serverDoc->should_close() ? serverDoc->exit_code() : 0;
}

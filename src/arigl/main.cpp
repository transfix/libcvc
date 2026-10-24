/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// arigl — the GL Ariadne runtime: run a .ari document over cvcGL (VTK + ImGui). `ari` is the
// GL-free core runtime and `arisrv` the headless server; arigl is the one that draws a 3D scene.
//
//   arigl run      [flags] doc.ari [document args...]
//   arigl snapshot [flags] doc.ari [document args...]
//   arigl check    [flags] doc.ari
//
// By default the document runs in a real window with a GL context: its widgets are ImGui over the
// VTK scene. With --terminal it runs in the terminal instead: the widgets go through libcvc's
// terminal backends (FTXUI, or --backend tvision) and the scene is rendered offscreen and drawn
// behind them (cvc::gl::ariadne::TerminalScene).
//
// The flags are the ones every Ariadne runtime takes (host_cli.h) plus --terminal, and they come
// BEFORE the document path. Everything after it belongs to the document (doc_args.h): it reads it
// as argv, or declares an args: block (`arigl run doc.ari --help` then shows its options).
//
// run ends on a "quit" action (a menu item with `on: quit`, or a program calling (quit)), when the
// window closes, or on Ctrl-C in the terminal. OS signals (SIGTERM, SIGHUP, SIGINT, SIGQUIT,
// SIGUSR1/2) go to the document's on_signal: in both modes; one it does not handle ends the run
// with exit status 128 + the signal number.
// snapshot renders N frames, paced like run so a live stream delivers into them (a window's scene
// clock stays fixed, so its frames are reproducible): in a window, offscreen at --width x --height
// pixels (default 1024 x 768), writing the last one to --png FILE; with --terminal, the last frame
// as text (--ansi keeps its colours), plus the rendered scene as --png if given.
// check loads the document as run would, offscreen, and prints its warnings.

#include <algorithm>
#include <cctype> // std::isalnum / std::isdigit — sanitize the conference node id
#include <chrono>
#include <cstdio>
#include <cstdlib> // std::getenv — the conference relay / room / name come from the environment
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/host_info.h>
#include <cvc/ariadne/host_signals.h>
#include <cvc/ariadne/loader.h> // run_init — seed conf.room / conf.self before the first tick
#include <cvc/ariadne/stream/stream_view.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/core/app.h>
#include <random> // std::random_device — the conference node id's per-launch suffix
#include <sstream> // arigl info: the GL report, line by line
#if CVC_ENABLE_WS || CVC_ENABLE_GRPC
// Conference mode: a state transport joins a hosted room, chosen by the relay URL scheme —
// ws://|wss:// (WebSocket, what browsers dial; the host page sets wss://<page-origin>/…) or grpc://
// (native gRPC). register_conference supplies the room verbs and the shard/transport are wired here
// in the host; both transports carry the canonical media_frame and interop via the relay's router.
#include <cvc/ariadne/holon.h>                 // the client replica of the room holon (avatars, H1)
#include <cvc/ariadne/holon_sync.h>            // client_pump / propose_intent over the room tree
#include <cvc/ariadne/stream/frame.h>          // frame_ptr — forward the local camera/mic streams
#include <cvc/ariadne/stream/stream.h>         // stream::open — per-peer decode proxies
#include <cvc/ariadne/stream/stream_channel.h> // subscribe / publish_external
#include <cvc/ariadne/stream/stream_registry.h> // look up the local camera/mic streams by token
#include <cvc/core/clock_driver.h>        // one-advance-per-frame fan-out over the world_clock
#include <cvc/core/world_clock.h>         // app.world_clock().t() — the common media timestamp
#include <cvc/gl/ariadne/room_view.h>     // the room scene + avatar markers in the 3D view
#include <cvc/gl/capture/audio_source.h>  // list_audio_capture_devices — mic picker
#include <cvc/gl/capture/camera_source.h> // list_cameras — camera picker
#include <cvc/gl/playback/audio_output.h> // open_audio_output — play each peer's mic
#include <cvc/media/media_frame.h>        // cvc::media_frame — the canonical media frame
#include <cvc/state/state.h>              // cvc::state::instance — conference media state keys
#include <cvc/state/state_cluster_shard.h>
#if CVC_ENABLE_WS
#include <cvc/state/state_transport_ws.h>
#endif
#if CVC_ENABLE_GRPC
#include <cvc/state/state_transport_grpc.h>
#endif
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#endif
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h> // single-thread wasm: flush the publisher at frame cadence
#include <emscripten.h>             // emscripten_sleep — yield the asyncify loop to the browser
#endif
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/SdlInput.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/TerminalScene.h>
#include <cvc/gl/ariadne/bundles.h>
#include <cvc/gl/ariadne/scene_panels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_view_widget.h> // jpeg_frame_decoder
#include <cvc/image/image.h>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <vtkRenderWindow.h> // arigl info: the GL it renders with

namespace {

namespace ari = cvc::ariadne;
namespace gla = cvc::gl::ariadne;

const char *const kDefaultPrefix = "ui.docs.ari"; // the same root ari uses, in both modes

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage: arigl run|snapshot|check [flags] doc.ari [document args...]\n"
               "       arigl info [--terminal] [--json] [--functions] [--backend B]\n"
               "\n"
               "flags (before the document):\n"
               "  --terminal             run in the terminal, the scene drawn behind the widgets\n"
               "%s"
               "\n"
               "A window by default (ImGui over the VTK scene). snapshot renders offscreen: write\n"
               "it with --png FILE, or with --terminal print it as text. Everything after the\n"
               "document path is the document's own: it reads it as argv, or declares an args:\n"
               "block (then `arigl run doc.ari --help` shows its options).\n"
               "\n"
               "info prints what this host provides, in a window or with --terminal: the GL it\n"
               "renders with, the terminal (size, colours, kitty / Sixel), what libcvc was built\n"
               "with, the capabilities a document's requires: and (has? ...) see, and how many\n"
               "functions programs can call (--functions lists them; --json prints JSON).\n",
               ari::host_flags_help().c_str());
  return code;
}

// Load the document. Returns -1 to carry on, else the exit status: 0 after printing the
// document's --help, 2 for arguments it does not accept, 1 for a failed load.
template <class Runtime> int load_document(Runtime &rt, const ari::HostOptions &o) {
  std::vector<std::string> warnings;
  try {
    warnings = rt.load(o.doc);
  } catch (const ari::document_args_error &e) {
    if (e.help) {
      std::fputs(e.usage.c_str(), stdout);
      return 0;
    }
    std::fprintf(stderr, "arigl: %s: %s\n", o.doc.c_str(), e.what());
    const std::string note = ari::misplaced_flag_note("arigl", o.doc_args);
    if (!note.empty())
      std::fprintf(stderr, "%s\n", note.c_str());
    std::fprintf(stderr, "\n%s", e.usage.c_str());
    return 2;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "arigl: %s\n", e.what());
    return 1;
  }
  for (const std::string &w : warnings)
    std::fprintf(stderr, "arigl: warning: %s\n", w.c_str());
  return -1;
}

void print_warnings(std::vector<std::string> warnings) {
  for (const std::string &w : warnings)
    std::fprintf(stderr, "arigl: warning: %s\n", w.c_str());
}

void note_misplaced(const ari::AppRuntime &rt, const ari::HostOptions &o) {
  if (rt.has_args_block())
    return;
  const std::string note = ari::misplaced_flag_note("arigl", o.doc_args);
  if (!note.empty())
    std::fprintf(stderr, "%s\n", note.c_str());
}

// --graphics MODE -> how a terminal draws images (host_cli has checked MODE).
ari::FtxuiBackend::Options::ImageColor image_mode(const std::string &s) {
  using Mode = ari::FtxuiBackend::Options::ImageColor;
  if (s == "kitty")
    return Mode::Kitty;
  if (s == "sixel")
    return Mode::Sixel;
  if (s == "color")
    return Mode::Color;
  if (s == "mono")
    return Mode::Mono;
  return Mode::Auto;
}

// --image NAME=FILE: publish each picture for the document's image: widgets. False on a bad file.
template <class Backend> bool publish_images(Backend &backend, const ari::HostOptions &o) {
  for (const auto &[name, file] : o.images) {
    try {
      backend.set_image(name, cvc::image::load(file));
    } catch (const std::exception &e) {
      std::fprintf(stderr, "arigl: --image %s: %s\n", file.c_str(), e.what());
      return false;
    }
  }
  return true;
}

// A snapshot's frames run at the interactive loop's rate (--fps, else 20 a second), so a live
// stream's producer has time to deliver frames into them. The window's scene clock stays fixed.
std::chrono::duration<double> snapshot_period(const ari::HostOptions &o) {
  return std::chrono::duration<double>(1.0 / (o.fps > 0 ? o.fps : 20));
}

// The SDL-free cvc::gl::InputEvent, as the runtime's cvc::ariadne::InputEvent.
ari::InputEvent to_ari(const cvc::gl::InputEvent &e) {
  ari::InputEvent a;
  using GK = cvc::gl::InputEvent::Kind;
  using AK = ari::InputEvent::Kind;
  switch (e.kind) {
  case GK::KeyDown:
    a.kind = AK::KeyDown;
    break;
  case GK::KeyUp:
    a.kind = AK::KeyUp;
    break;
  case GK::MouseMove:
    a.kind = AK::MouseMove;
    break;
  case GK::MouseButtonDown:
    a.kind = AK::MouseButtonDown;
    break;
  case GK::MouseButtonUp:
    a.kind = AK::MouseButtonUp;
    break;
  case GK::MouseWheel:
    a.kind = AK::MouseWheel;
    break;
  }
  a.key = e.key;
  a.mods = e.mods;
  a.repeat = e.repeat;
  a.x = e.x;
  a.y = e.y;
  a.dx = e.dx;
  a.dy = e.dy;
  a.button = e.button;
  a.clicks = e.clicks;
  return a;
}

// ── conference mode (optional) ───────────────────────────────────────────────────────────────────
// Enabled when CVC_ARIADNE_RELAY is set in the environment: the browser host page sets it to
// wss://<page-origin>/cyprty-ws (so the deployed client auto-connects to its own server), native
// tests set it to ws://host:port. register_conference supplies the room verbs (room-join/roster);
// the shard + WebSocket transport are owned here and pumped each frame. A plain local .ari run just
// leaves CVC_ARIADNE_RELAY unset and never touches any of this.
struct ConferenceSession {
#if CVC_ENABLE_WS || CVC_ENABLE_GRPC
  // shard is declared BEFORE the transport so the transport (whose destructor stops its reader) is
  // torn down first.
  std::unique_ptr<cvc::state_cluster_shard> shard;
#if CVC_ENABLE_WS
  std::unique_ptr<cvc::state_transport_ws> ws_; // dialed for a ws://|wss:// relay (browsers)
#endif
#if CVC_ENABLE_GRPC
  std::unique_ptr<cvc::state_transport_grpc> grpc_; // dialed for a grpc:// relay (native)
#endif
  // The chosen transport, held via the base for the shared ops (register_shard / pump_all). The
  // media send and stop go through the concrete via these wrappers (publish_media_frame / stop are
  // not on the base interface). Exactly one of ws_/grpc_ is set; tp_ points at it.
  cvc::state_transport *tp_ = nullptr;
  std::function<void(const cvc::media_frame &)> send_media_;
  std::function<void()> stop_;
  bool active = false;

  // ── media plane (browser camera + mic over the WS control channel) ──
  cvc::app *app_ = nullptr;
  std::string pid_; // this participant's node id (== the room-join pid; stamps our outgoing frames)
  std::string
      prefix_;       // the document chroot prefix: host state access must prepend it so it reaches
                     // the SAME keys the .ari's (state-get/-set) do (which are prefix-relative)
  std::string room_; // the joined room id (conf.rooms.<room_>)

  // ── the room in 3D (H1 slice 6): the shared scene + avatars, third-person controls ──
  // A client replica of the room holon, kept in step with the replicated room tree by client_pump
  // each frame: our avatar predicted from our own intent, peers walked between the relay's pose
  // samples. room_view draws the server's scene + one marker per participant into the window's
  // SceneGraph; the camera orbits our avatar. Made by attach_view (window mode only); room_view_
  // is declared after holon_ and torn down first (stop()).
  std::unique_ptr<cvc::ariadne::holon> holon_;
  std::unique_ptr<gla::room_view> room_view_;
  cvc::gl::CameraController *cam_ = nullptr;
  cvc::ariadne::avatar_intent last_intent_; // the last intent proposed (propose only on change)
  bool intent_proposed_ = false;            // proposed once yet (the first frame always proposes)
  std::vector<std::string> drawn_pids_;     // avatars drawn at the last log (log on change)
  bool framed_ = false;                     // camera framed on our avatar once
  double orbit_center_[3] = {0.0, 0.0, 0.0};
  static constexpr double kWalkSpeed = 2.0; // m/s
  static constexpr double kEyeHeight = 1.2; // orbit centre above the avatar's feet (m)

  // Publish: forward the local camera (latest) and mic (ring) streams. The .ari puts their tokens
  // in "cam_token" / "mic_token". We re-stamp each outgoing frame with a COMMON host clock (one
  // reading per pump) so the receiver can A/V-sync them — the sources' own pts are on unrelated
  // clocks (SDL camera vs SDL audio vs synthetic), not comparable across streams.
  struct publisher {
    std::string key;   // state key holding the stream token
    std::string kind;  // "camera" | "mic"
    bool ring = false; // audio = ring (forward EVERY chunk in order); video = latest (freshest)
    std::string token; // current stream token
    std::shared_ptr<cvc::ariadne::stream::subscription> sub;
    std::int64_t last_seq = -1;
  };
  std::vector<publisher> pubs_;

  // Consume: peers' frames arrive on the ws reader -> inbox_; pump() routes them. Per peer we keep
  // a video decode proxy + a small pts-ordered buffer of video frames (so the display can lag to
  // the audio clock), and a queue of decoded audio chunks awaiting the mixer (below).
  std::mutex media_mu_;
  std::vector<cvc::media_frame> inbox_;
  struct vframe {
    double pts = 0.0;
    std::shared_ptr<std::vector<std::uint8_t>> bytes;
  };
  struct peer_media {
    std::unique_ptr<cvc::ariadne::stream::stream>
        video_proxy;         // what the peer's stream_view renders
    std::deque<vframe> vbuf; // buffered video frames awaiting their audio-synced display time
    double shown_pts = -1.0; // pts of the frame currently injected into video_proxy
    std::deque<std::vector<float>> achunks; // decoded f32 chunks awaiting the mixer
    double last_audio_pts = -1.0;           // world-clock stamp of this peer's last audio chunk
    bool has_audio = false; // once true, video display follows this peer's audio clock
    // Per-peer LOCAL controls (this client's mix only — never sent to the peer, never change their
    // capture). Driven by the user-list window; applied in mix_audio / sync_and_show.
    float volume = 1.0f;      // playback gain in our mix (slider; 1 = unity)
    bool audio_muted = false; // muted in our mix (we skip their audio)
    bool video_muted = false; // hidden locally (we stop advancing their video)
    double talk_until = 0.0; // wall time until which this peer counts as "talking" (name highlight)
  };
  std::map<std::string, peer_media> peers_; // keyed by origin pid
  double self_talk_until_ = 0.0;            // wall time until which WE count as "talking"
  double now_wall_ = 0.0;                   // wall-clock seconds, refreshed once per pump
  std::vector<std::string> cam_devices_, mic_devices_; // cached device names (index = device index)
  bool devices_enumerated_ = false;                    // devices listed yet (deferred past startup)

  // ALL peers' audio is summed into ONE output: SDL/WebAudio multi-stream behaviour in wasm is
  // uncertain and browsers cap AudioContexts, so one mixed sink works for any number of talkers.
  // mix_audio() pops one chunk from each peer per mixed chunk and sums them (silence for an empty
  // peer). mix_frames_ is the interleaved sample count of a chunk (all mics are
  // 48k/1ch/1024-frame).
  std::unique_ptr<cvc::gl::playback::audio_output> mix_out_;
  int mix_rate_ = 0, mix_ch_ = 0;
  std::size_t mix_frames_ = 0;

  // Voice activity: RMS of an interleaved f32 chunk (~0..1), smoothed + thresholded for the
  // "talking" name highlight.
  static double chunk_rms(const float *s, std::size_t n) {
    double sumsq = 0.0;
    for (std::size_t i = 0; i < n; ++i)
      sumsq += static_cast<double>(s[i]) * static_cast<double>(s[i]);
    return n ? std::sqrt(sumsq / static_cast<double>(n)) : 0.0;
  }
  static constexpr double kTalkThreshold = 0.02; // RMS above this = "talking"
  static constexpr double kTalkHold = 0.60;      // keep "talking" lit this long (s) after the last
                                            // loud chunk — bridges brief pauses between words and
                                            // jittery delivery (wall time, so frame-rate indep.)

  // The state key carrying a peer's video-proxy token, so the .ari can bind that peer's
  // stream_view.
  static std::string media_token_key(const std::string &origin, const std::string &kind) {
    return "media." + origin + "." + kind;
  }

  // The per-frame clock driver: advances the per-app cvc::world_clock from wall time ONCE per frame
  // (the conference .ari declares no `clock:` block, so nothing else drives this app's clock) and
  // fans the conference's per-frame steps out as ordered observers. driver_->frame().t is the ONE
  // media timestamp per frame — world_clock seconds, tick*fixed_dt (drift-free, 1/120 s quanta;
  // sub-frame for media) — stamped on BOTH our camera and mic frames so a receiver can align them
  // (their sources' own pts are on unrelated clocks). Built in setup() once the app is known; null
  // for a plain local document (no relay), where pump() no-ops. install_observers() defines the
  // step order below.
  std::unique_ptr<cvc::clock_driver> driver_;
  cvc::clock_driver::handle dev_enum_h_; // handle of the self-removing device-enum one-shot

  // The order each conference step fires in, each frame (ascending). Gaps leave room to slot a
  // future step between these without renumbering.
  enum : int {
    kOrderPumpAll = 10,   // drain/flush the state transport (replicated room tree) — first
    kOrderRoom = 15,      // the room in 3D: keys -> intent, client holon, scene + avatars, camera
    kOrderWall = 20,      // refresh now_wall_ for the wall-time talk-hold
    kOrderControls = 30,  // read the user-list window's per-peer local controls
    kOrderDevEnum = 40,   // lazily enumerate camera/mic devices (one-shot)
    kOrderApplyDev = 50,  // map the device combos' picks to cam/mic.want_device
    kOrderForward = 60,   // forward our camera+mic, each stamped with frame.t
    kOrderDrain = 70,     // route peers' inbound frames into per-peer buffers
    kOrderMix = 80,       // sum peers' queued audio into the single mixed output
    kOrderShow = 90,      // advance each peer's video to its audio clock
    kOrderActivity = 100, // decay + publish talk activity for the roster — last
  };

  // Call AFTER the GL bundles and BEFORE load(): register the room verbs, seed conf.room/conf.self,
  // and dial the relay. Returns false (a plain local document) when no relay is configured.
  bool setup(cvc::app &app, const std::string &prefix, cvc::gl::ImGuiBackend &backend) {
    const char *relay = std::getenv("CVC_ARIADNE_RELAY");
    if (!relay || !*relay)
      return false;
    const char *roomEnv = std::getenv("CVC_ARIADNE_ROOM");
    const char *nameEnv = std::getenv("CVC_ARIADNE_NAME");
    const std::string room = (roomEnv && *roomEnv) ? roomEnv : "standup";
    const std::string self = (nameEnv && *nameEnv) ? nameEnv : "guest";
    // The shard's local_node_id identifies this participant in the replicated room tree and must be
    // a C identifier (letters/digits/underscore, no leading digit). The DISPLAY name (conf.self,
    // shown in the roster) may be any text — e.g. a browser's random "guest-ab12" — and room-join
    // already carries it separately as the "name" attr, so derive a safe node id here rather than
    // passing the display name straight through (a hyphen in it otherwise throws).
    //
    // The id also gets a random per-launch suffix. Replication drops a write whose (origin node id,
    // sequence) it has already seen, so a client that restarts under the SAME id (a reloaded tab,
    // a relaunched native client, two people who both kept the name "guest") would have every
    // write ignored until the relay restarts: it would join a room nobody can see it in. A fresh
    // id per session makes each launch a new participant; the name in the roster is unchanged.
    std::string nodeId;
    for (char c : self)
      nodeId += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    if (nodeId.empty() || std::isdigit(static_cast<unsigned char>(nodeId.front())))
      nodeId.insert(0, "n_");
    {
      std::random_device rd;
      const char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
      nodeId += '_';
      for (int i = 0; i < 6; ++i)
        nodeId += digits[rd() % 36];
    }
    gla::register_conference(app, backend); // room-join/roster + streaming + stream_view
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", nodeId);
    // Replicate ONLY the shared room tree (conf.rooms.*), never this client's per-document UI state
    // (ui.docs.*). Without this the shard's adapter observes from the root, so a plain (state-set)
    // the .ari makes — above all conf.self, the display name bound to the Name field — would
    // replicate to every peer and they would all show the same name (two clients "collide on the
    // name"). The interest filter is dot-segment aware: "conf" matches conf.rooms.* but not
    // ui.docs.ari.conf.self.
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach(); // also installs this as the default shard; room-join derives the pid from it
    app_ = &app;
    pid_ = nodeId;
    prefix_ = prefix;
    room_ = room;
    // Drive the world clock from wall time via the clock driver: live mode so advance() banks wall
    // time. The driver owns the single advance of this app's clock and fires the conference's
    // per-frame steps in order (install_observers); nothing else advances it.
    app.world_clock().set_mode(cvc::world_clock::mode::live);
    driver_ = std::make_unique<cvc::clock_driver>(app.world_clock());
    install_observers();
    // The streams we forward: the camera (latest — freshest frame) and the mic (ring — every chunk
    // in order, since dropped audio is a glitch). The .ari publishes their tokens in
    // cam_token/mic_token.
    pubs_ = {{"cam_token", "camera", /*ring=*/false, {}, {}, -1},
             {"mic_token", "mic", /*ring=*/true, {}, {}, -1}};
    // Peers' media arrives on the transport's reader thread: just queue it here (do not touch the
    // stream registry / app tree off the main thread), and route it in pump() on the main thread.
    auto on_media = [this](const cvc::media_frame &m) {
      std::lock_guard<std::mutex> lk(media_mu_);
      inbox_.push_back(m);
    };
    // Pick the transport by the relay URL scheme: grpc:// dials the native gRPC state plane;
    // anything else (ws://|wss://) dials the WebSocket plane the browser uses. Both carry the
    // canonical media_frame and interop through the relay's media_router, so a native and a browser
    // client share one room's audio/video. One transport is owned here; tp_ is the base handle for
    // the shared ops and send_media_/stop_ wrap the concrete's publish/stop.
    const std::string relayUrl(relay);
    const bool want_grpc = (relayUrl.rfind("grpc://", 0) == 0);
    bool ok = false;
    if (want_grpc) {
#if CVC_ENABLE_GRPC
      grpc_ = std::make_unique<cvc::state_transport_grpc>();
      grpc_->start("127.0.0.1:0", nodeId,
                   "conf"); // gRPC must start() (ephemeral port) even to dial
      grpc_->register_shard(shard.get());
      grpc_->set_on_media(on_media);
      tp_ = grpc_.get();
      send_media_ = [p = grpc_.get()](const cvc::media_frame &m) { p->publish_media_frame(m); };
      stop_ = [p = grpc_.get()] { p->stop(); };
      ok = grpc_->connect_to_peer(relayUrl.substr(7), std::chrono::milliseconds(8000));
#else
      std::fprintf(stderr, "arigl: conference: relay is grpc:// but this build has no gRPC "
                           "(CVC_ENABLE_GRPC=OFF); running the document locally.\n");
      return false;
#endif
    } else {
#if CVC_ENABLE_WS
      ws_ = std::make_unique<cvc::state_transport_ws>();
      ws_->register_shard(shard.get());
      ws_->set_on_media(on_media);
      tp_ = ws_.get();
      send_media_ = [p = ws_.get()](const cvc::media_frame &m) { p->publish_media_frame(m); };
      stop_ = [p = ws_.get()] { p->stop(); };
      ok = ws_->connect_to_peer(relayUrl, std::chrono::milliseconds(8000));
#else
      std::fprintf(stderr, "arigl: conference: relay is ws:// but this build has no WebSocket "
                           "(CVC_ENABLE_WS=OFF); running the document locally.\n");
      return false;
#endif
    }
    std::fprintf(stderr, "arigl: conference: %s %s as '%s' (room '%s')\n",
                 ok ? "connected to" : "could NOT reach relay", relay, self.c_str(), room.c_str());
    // ?synthetic=1 (the host page sets CVC_ARIADNE_SYNTHETIC_CAM) drives the self-cam from a
    // hardware-free MJPEG test pattern instead of the real camera, so the conference — peer video
    // included — can be exercised in a browser with no camera.
    const char *synthEnv = std::getenv("CVC_ARIADNE_SYNTHETIC_CAM");
    const std::string synth = (synthEnv && *synthEnv && std::string(synthEnv) != "0") ? "1" : "0";
    std::vector<std::string> ce;
    ari::run_init(
        app, prefix,
        "(state-set \"conf.room\" \"" + room + "\") (state-set \"conf.self\" \"" + self +
            "\") (state-set \"cam_synthetic\" \"" + synth + "\") (state-set \"self.pid\" \"" +
            nodeId +
            // Seed peer.count to a valid integer BEFORE the first render, so the roster's
            // `repeat: (state-get "peer.count")` never evaluates an empty node on frame 0
            // (which warns "did not yield an integer" until the first on_tick sets it).
            "\") (state-set \"peer.count\" \"0\")"
            // Seed the device-selection fields to EMPTY STRINGS. state-get on an UNSET node
            // returns nil, not "", so the real-camera on_tick guard (= cam.want_device "")
            // was false before the host first populated it, running the device-switch body
            // and evaluating (int nil) — which THROWS and permanently kills the on_tick
            // resident, so the roster never builds (synthetic skips that block, so it was
            // only ever broken for real cameras). With "" seeded the guard is correct.
            " (state-set \"cam.want_device\" \"\") (state-set \"mic.want_device\" \"\")"
            " (state-set \"cam.cur_device\" \"\") (state-set \"mic.cur_device\" \"\")",
        &ce);
    for (const std::string &e : ce)
      std::fprintf(stderr, "arigl: conference init: %s\n", e.c_str());
    // NOTE: device enumeration is deferred to pump() (see devices_enumerated_), NOT done here —
    // on single-threaded wasm, SDL camera/audio device queries before the Asyncify event loop runs
    // block/stall and hang startup.
    active = true;
    return true;
  }
  // Window mode: show the room in 3D in `view` — the server's shared scene and an avatar per
  // participant — with third-person controls: WASD / arrow keys walk our avatar relative to the
  // camera's heading, and `cam` (Orbit) keeps orbiting it. A no-op for a document with no relay.
  void attach_view(cvc::gl::SceneRenderer &view, cvc::gl::CameraController &cam) {
    if (!active || !driver_ || !app_)
      return;
    holon_ = std::make_unique<cvc::ariadne::holon>(cvc::ariadne::holon_role::client);
    room_view_ =
        std::make_unique<gla::room_view>(view, *app_, room_, pid_, view.scene().getStatePrefix());
    cam_ = &cam;
    driver_->add_frame_observer(kOrderRoom,
                                [this](const cvc::clock_driver::clock_frame &f) { step_room(f); });
  }

  void step_room(const cvc::clock_driver::clock_frame &f) {
    if (!holon_ || !room_view_ || !cam_)
      return;
    // Keys -> intent, only in Orbit: in Fly the same keys move the camera instead. The camera's
    // held set is dropped while the UI has the keyboard, so typing a name never walks.
    cvc::ariadne::avatar_intent in;
    if (cam_->mode() == cvc::gl::CameraController::Mode::Orbit) {
      double eye[3], focal[3], up[3];
      cam_->getPose(eye, focal, up);
      const auto held = [this](const char *a, const char *b) {
        return cam_->keyHeld(a) || cam_->keyHeld(b);
      };
      in = gla::intent_from_keys(held("w", "Up"), held("s", "Down"), held("a", "Left"),
                                 held("d", "Right"), eye, focal, kWalkSpeed);
    }
    // Propose on the first frame even when standing still, so the tree never holds a walk an
    // earlier session under this id proposed; after that, only on change (a replicated write).
    if (!intent_proposed_ || in.move.x != last_intent_.move.x || in.move.y != last_intent_.move.y ||
        in.move.z != last_intent_.move.z) {
      cvc::ariadne::propose_intent(*holon_, *app_, room_, pid_, in);
      last_intent_ = in;
      intent_proposed_ = true;
    }
    // Re-assert it locally every frame: our avatar joins the holon only once our room-join is in
    // the tree, so an intent proposed before then must still reach the prediction.
    holon_->set_intent(pid_, last_intent_);
    cvc::ariadne::client_pump(*holon_, *app_, room_, pid_, f.wall_dt);
    room_view_->update(*holon_, f.wall_dt);
    // Log who is drawn whenever that changes (the browser page shows stderr in its footer and the
    // console): the quickest check that a client sees the room the relay holds.
    const std::vector<std::string> drawn = room_view_->avatar_pids();
    if (drawn != drawn_pids_) {
      std::string who;
      for (const std::string &pid : drawn)
        who += (who.empty() ? "" : ", ") + pid;
      std::fprintf(stderr, "arigl: room: %zu avatar(s): %s\n", drawn.size(), who.c_str());
      drawn_pids_ = drawn;
    }
    for (const std::string &w : room_view_->take_warnings())
      std::fprintf(stderr, "arigl: room: %s\n", w.c_str());
    // Third person: frame our avatar once, then keep the orbit centre on it.
    double p[3];
    if (room_view_->self_position(p)) {
      if (!framed_) {
        // Frame the room around you, not just you: arriving at a party, you want to see who's here.
        cam_->frameBounds(p[0] - 8.0, p[1] - 8.0, p[2], p[0] + 8.0, p[1] + 8.0, p[2] + 2.0);
        framed_ = true;
      }
      const double c[3] = {p[0], p[1], p[2] + kEyeHeight};
      if (std::fabs(c[0] - orbit_center_[0]) + std::fabs(c[1] - orbit_center_[1]) +
              std::fabs(c[2] - orbit_center_[2]) >
          1e-4) {
        cam_->setOrbitCenter(c[0], c[1], c[2]);
        orbit_center_[0] = c[0], orbit_center_[1] = c[1], orbit_center_[2] = c[2];
      }
    }
  }

  // One advance per frame. The driver advances the per-app world_clock from wall time (the single
  // media stamp, frame().t) and fires the conference's per-frame steps as the ordered observers
  // install_observers() registered. Called from the input pump (run mode) and directly per frame
  // (snapshot mode); a plain local document has no driver and no-ops.
  void pump() {
    if (!tp_ || !driver_)
      return;
    driver_->advance(); // measured wall delta; fires kOrderPumpAll .. kOrderActivity in order
  }

  // Register the per-frame steps as ordered observers on the driver, once, in setup(). They fire
  // in ascending kOrder* each advance(), all BEFORE the document's on:tick (conf.pump runs before
  // app_rt_.drain()), preserving the deliberate one-frame lag read_peer_controls relies on.
  void install_observers() {
    using cf = cvc::clock_driver::clock_frame;
    // Drive the state transport first: flush replicated room-tree writes and service queued reads
    // so room-join/roster are in place before anything this frame reads them.
    driver_->add_frame_observer(kOrderPumpAll, [this](const cf &) { tp_->pump_all(); });
    // Wall-clock seconds for the talk-hold — deliberately wall time, not world time, so the hold is
    // frame-rate independent and does not freeze when the sim clock is paused or scaled.
    driver_->add_frame_observer(kOrderWall, [this](const cf &) {
      now_wall_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                      .count();
    });
    // Pull the user-list window's per-peer volume/mute (local only, never sent).
    driver_->add_frame_observer(kOrderControls, [this](const cf &) { read_peer_controls(); });
    // Enumerate the camera/mic devices for the pickers LAZILY — once the camera stream is open, so
    // the SDL audio/camera subsystems are already initialized (enumeration is then a cheap refcount
    // bump) and the Asyncify event loop is running. Doing this at setup instead STALLS startup on
    // single-threaded wasm (SDL device queries before the loop runs block / can't resolve). A
    // self-removing one-shot: it checks every frame until the camera token appears, enumerates
    // once, then unregisters so the per-frame check stops.
    dev_enum_h_ = driver_->add_frame_observer(kOrderDevEnum, [this](const cf &) {
      if (!devices_enumerated_ && app_ &&
          !cvc::state::instance(*app_)(prefix_ + ".cam_token").value().empty()) {
        enumerate_devices();
        devices_enumerated_ = true;
        driver_->remove(dev_enum_h_);
      }
    });
    // Map the device combos' picks to cam/mic.want_device for the .ari to re-capture against.
    driver_->add_frame_observer(kOrderApplyDev, [this](const cf &) { apply_device_selection(); });
    // Forward our camera (latest) + mic (ring), each re-stamped with THIS frame's single
    // world-clock reading (frame.t) so a receiver can A/V-align them; also measures our own mic
    // level.
    driver_->add_frame_observer(kOrderForward, [this](const cf &f) { forward_streams(f.t); });
    // Route peers' inbound frames (queued by the reader thread) into per-peer buffers; also
    // measures each peer's mic level.
    driver_->add_frame_observer(kOrderDrain, [this](const cf &) { drain_inbox(); });
    // Sum all peers' queued audio into the single mixed output (per-peer gain, skip muted).
    driver_->add_frame_observer(kOrderMix, [this](const cf &) { mix_audio(); });
    // Advance each peer's video to its audio clock (skip locally video-muted).
    driver_->add_frame_observer(kOrderShow, [this](const cf &) { sync_and_show(); });
    // Decay talk levels + publish media.<pid>.talking for the name highlight.
    driver_->add_frame_observer(kOrderActivity, [this](const cf &) { publish_activity(); });
  }

  // Pull the per-peer LOCAL controls the user-list window writes — keyed by roster INDEX, which the
  // .ari maps to a pid in peer.<i>.pid — and apply them to peers_. Reads last frame's on_tick
  // values (conf.pump runs before the document tick), a negligible one-frame lag. LOCAL only: never
  // sent.
  void read_peer_controls() {
    if (!app_)
      return;
    auto &st = cvc::state::instance(*app_);
    auto get = [&](const std::string &k) { return st(prefix_ + "." + k).value(); };
    int count = 0;
    try {
      count = std::stoi(get("peer.count"));
    } catch (...) {
      count = 0;
    }
    for (int i = 0; i < count; ++i) {
      const std::string base = "peer." + std::to_string(i);
      const std::string pid = get(base + ".pid");
      if (pid.empty() || pid == pid_)
        continue;
      auto it = peers_.find(pid);
      if (it == peers_.end())
        continue;
      peer_media &pm = it->second;
      const std::string vol = get(base + ".vol");
      if (!vol.empty()) {
        try {
          pm.volume = std::max(0.0f, std::stof(vol));
        } catch (...) {
        }
      }
      // aon/von are "on" checkboxes (default on): muted only when explicitly "0".
      pm.audio_muted = (get(base + ".aon") == "0");
      pm.video_muted = (get(base + ".von") == "0");
    }
  }

  // Enumerate the camera + mic devices and publish their names (newline-joined) to state so the
  // own-camera window's combos can list them (the .ari splits on "\n"); default each selection to
  // the current device (0). Call on the SDL main thread (setup / pump). Browser labels may be
  // generic until camera/mic permission is granted; switching still works by index.
  void enumerate_devices() {
    if (!app_)
      return;
    cam_devices_ = cvc::gl::capture::list_cameras();
    mic_devices_ = cvc::gl::capture::list_audio_capture_devices();
    auto &st = cvc::state::instance(*app_);
    auto join = [](const std::vector<std::string> &v) {
      std::string s;
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (i)
          s += '\n';
        s += v[i];
      }
      return s;
    };
    st(prefix_ + ".cam.devices").value(join(cam_devices_));
    st(prefix_ + ".mic.devices").value(join(mic_devices_));
    if (st(prefix_ + ".cam.sel").value().empty() && !cam_devices_.empty())
      st(prefix_ + ".cam.sel").value(cam_devices_[0]);
    if (st(prefix_ + ".mic.sel").value().empty() && !mic_devices_.empty())
      st(prefix_ + ".mic.sel").value(mic_devices_[0]);
  }

  // Map the combo's selected device NAME (cam.sel / mic.sel) to its index and publish it as
  // cam.want_device / mic.want_device; the .ari re-captures when that index differs from the one it
  // currently has open (cam.cur_device / mic.cur_device).
  void apply_device_selection() {
    if (!app_)
      return;
    auto &st = cvc::state::instance(*app_);
    auto want = [&](const std::vector<std::string> &devs, const char *selkey, const char *wantkey) {
      const std::string sel = st(prefix_ + "." + selkey).value();
      if (sel.empty())
        return;
      for (std::size_t i = 0; i < devs.size(); ++i)
        if (devs[i] == sel) {
          st(prefix_ + "." + wantkey).value(std::to_string(i));
          return;
        }
    };
    want(cam_devices_, "cam.sel", "cam.want_device");
    want(mic_devices_, "mic.sel", "mic.want_device");
  }

  // Publish each participant's boolean "talking" to media.<pid>.talking (self included) — true
  // while its talk-hold has not expired; the .ari copies it into peer.<i>.talking for the name
  // highlight.
  void publish_activity() {
    if (!app_)
      return;
    auto &st = cvc::state::instance(*app_);
    auto set_talk = [&](const std::string &pid, double until) {
      st(prefix_ + "." + media_token_key(pid, "talking")).value(until > now_wall_ ? "1" : "0");
    };
    set_talk(pid_, self_talk_until_);
    for (auto &kv : peers_)
      set_talk(kv.first, kv.second.talk_until);
  }

  void stop() {
    room_view_.reset(); // remove the room's nodes while the SceneGraph is still alive
    peers_.clear();     // close decode proxies before the transport/shard go away
    mix_out_.reset();
    for (publisher &p : pubs_)
      p.sub.reset();
    if (stop_)
      stop_(); // joins the transport reader thread (idempotent: _running.exchange(false))
  }

  // Join the transport reader BEFORE any member is destroyed. stop() runs in the destructor body,
  // ahead of reverse-order member destruction, so the reader (which the transport destructor would
  // otherwise join only AFTER media_mu_/inbox_ are already gone — they are declared after
  // ws_/grpc_) can no longer fire on_media into a destroyed mutex/vector. This covers every
  // teardown path: snapshot/check/info and a document-load failure return without the run loop's
  // explicit stop(), and an exception unwinding the run loop skips it too. stop() is idempotent, so
  // the run path's explicit call followed by this one is safe. (Single-threaded wasm has no reader
  // thread.)
  ~ConferenceSession() { stop(); }

  // Mix: pop one chunk from each peer that has audio queued, sum them (silence for an empty peer),
  // clamp, and play to the single output. Produce mixed chunks while the output is below ~100 ms so
  // it stays fed without running far ahead; stop when no peer has a chunk (output drains to
  // silence).
  void mix_audio() {
    if (!mix_out_ || mix_frames_ == 0 || mix_rate_ <= 0 || mix_ch_ <= 0)
      return;
    const double bps = static_cast<double>(mix_rate_) * static_cast<double>(mix_ch_) * 4.0;
    std::vector<float> mixed;
    for (int guard = 0; guard < 64; ++guard) { // bound work per pump
      if (bps > 0.0 && static_cast<double>(mix_out_->queued_bytes()) / bps >= 0.10)
        break;
      mixed.assign(mix_frames_, 0.0f);
      bool any = false;
      for (auto &kv : peers_) {
        peer_media &pm = kv.second;
        if (pm.achunks.empty())
          continue;
        const std::vector<float> &c = pm.achunks.front();
        if (!pm.audio_muted && pm.volume > 0.0f) { // locally muted -> consume but do not mix
          const std::size_t n = std::min(c.size(), mixed.size());
          const float g = pm.volume; // per-peer local volume (user slider)
          for (std::size_t i = 0; i < n; ++i)
            mixed[i] += c[i] * g;
        }
        pm.achunks
            .pop_front(); // always consume, so a muted peer stays in lockstep (queue doesn't grow)
        any = true;
      }
      if (!any)
        break;
      for (float &v : mixed)
        v = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v); // clamp when several talk at once
      mix_out_->play(mixed.data(), mixed.size() * sizeof(float));
    }
  }

  // Forward our local streams (camera: freshest frame; mic: every chunk in order) to the relay,
  // which fans them to the peers. EVERY outgoing frame is stamped with the common world-clock time
  // so a receiver can A/V-sync them — the sources' own pts (SDL camera vs SDL audio vs synthetic)
  // are not comparable across streams.
  void forward_streams(double stamp) {
    if (!app_)
      return;
    auto &reg = cvc::ariadne::stream::stream_registry::for_app(*app_);
    for (publisher &p : pubs_) {
      const std::string token = cvc::state::instance(*app_)(prefix_ + "." + p.key).value();
      if (token.empty())
        continue;
      if (token != p.token) { // the .ari (re)opened the stream — rebind
        p.token = token;
        p.sub.reset();
        p.last_seq = -1;
      }
      if (!p.sub) {
        cvc::ariadne::stream::stream_channel *ch = reg.lookup(p.token);
        if (!ch)
          continue; // not open yet
        p.sub = ch->subscribe(p.ring ? cvc::ariadne::stream::deliver_mode::ring
                                     : cvc::ariadne::stream::deliver_mode::latest,
                              p.ring ? 8 : 1);
        if (!p.sub)
          continue;
      }
      if (p.ring) { // audio: forward every new chunk in order
        cvc::ariadne::stream::frame_ptr f;
        while (p.sub->try_pop(f))
          if (f)
            forward_frame(p, *f, stamp);
      } else { // video: forward the freshest frame if it is new
        cvc::ariadne::stream::frame_ptr f = p.sub->latest();
        if (f && f->seq > p.last_seq)
          forward_frame(p, *f, stamp);
      }
    }
  }

  void forward_frame(publisher &p, const cvc::ariadne::stream::frame &f, double stamp) {
    // Self voice activity: measure our own forwarded mic so OUR name lights up in the list too.
    if (p.kind == "mic" && f.format.codec == "f32" && f.data && f.size >= sizeof(float) &&
        chunk_rms(reinterpret_cast<const float *>(f.data), f.size / sizeof(float)) > kTalkThreshold)
      self_talk_until_ = now_wall_ + kTalkHold;
    cvc::media_frame mf;
    mf.origin_node_id = pid_;
    mf.stream_id = p.kind;
    mf.codec = f.format.codec;
    mf.width = static_cast<std::uint32_t>(f.format.w);
    mf.height = static_cast<std::uint32_t>(f.format.h);
    mf.sample_rate = static_cast<std::uint32_t>(f.format.sample_rate);
    mf.channels = static_cast<std::uint32_t>(f.format.channels);
    mf.seq = f.seq;
    mf.pts_seconds = stamp; // the COMMON clock, not the source pts
    mf.payload.assign(f.data, f.data + f.size);
    send_media_(mf);
    p.last_seq = f.seq;
  }

  // Route each queued peer frame: mic -> a per-peer audio output (played immediately; its queue
  // depth is the A/V clock); camera -> a pts-ordered buffer + a lazily-opened decode proxy the tile
  // renders.
  void drain_inbox() {
    std::vector<cvc::media_frame> batch;
    {
      std::lock_guard<std::mutex> lk(media_mu_);
      batch.swap(inbox_);
    }
    for (cvc::media_frame &m : batch) {
      if (m.origin_node_id.empty() || m.origin_node_id == pid_)
        continue; // never play/show ourselves as a peer
      peer_media &pm = peers_[m.origin_node_id];
      if (m.stream_id == "mic") {
        if (!m.payload.empty() && (m.payload.size() % sizeof(float)) == 0) {
          const float *s = reinterpret_cast<const float *>(m.payload.data());
          const std::size_t n = m.payload.size() / sizeof(float);
          // Voice activity FIRST — a loud chunk holds the peer "talking" for kTalkHold seconds, for
          // the name highlight. Independent of the audio output below (which can fail to open with
          // no device), so "talking" works even where playback does not.
          if (chunk_rms(s, n) > kTalkThreshold)
            pm.talk_until = now_wall_ + kTalkHold;
          // Open the ONE mixed output lazily; a failed open (no audio device) just leaves mix_out_
          // null and the mixer no-ops — talk + roster are unaffected.
          if (!mix_out_ && m.sample_rate > 0 && m.channels > 0) {
            try {
              cvc::gl::playback::audio_output_spec spec;
              spec.sample_rate = static_cast<int>(m.sample_rate);
              spec.channels = static_cast<int>(m.channels);
              cvc::gl::playback::audio_output_open opened =
                  cvc::gl::playback::open_audio_output(spec);
              mix_out_ = std::move(opened.sink);
              mix_rate_ = opened.sample_rate;
              mix_ch_ = opened.channels;
            } catch (...) {
              mix_out_.reset();
            }
          }
          // Queue the chunk for the mixer (do NOT play per-peer — mix_audio() sums all peers).
          // Bound the backlog so a stuck mixer can't grow memory unbounded.
          pm.achunks.emplace_back(s, s + n);
          if (mix_frames_ == 0)
            mix_frames_ = n;
          while (pm.achunks.size() > 16)
            pm.achunks.pop_front();
        }
        pm.last_audio_pts = m.pts_seconds;
        pm.has_audio = true;
      } else { // "camera" (video)
        if (!pm.video_proxy && m.width > 0 && m.height > 0) {
          cvc::ariadne::stream::stream_params p;
          p.id = "peer_" + m.origin_node_id + "_camera"; // origin is already a safe node id
          p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
          p.format.codec = m.codec.empty() ? std::string("mjpeg") : m.codec;
          p.format.w = static_cast<int>(m.width);
          p.format.h = static_cast<int>(m.height);
          p.format.stride = static_cast<int>(m.width) * 4; // descriptor only; bytes alias on inject
          p.expected_subscribers = 1;
          pm.video_proxy = cvc::ariadne::stream::stream::open(*app_, p);
          if (pm.video_proxy)
            cvc::state::instance (*app_)(prefix_ + "." +
                                         media_token_key(m.origin_node_id, "camera"))
                .value(pm.video_proxy->token());
        }
        if (pm.video_proxy && m.width > 0 && m.height > 0) {
          vframe vf;
          vf.pts = m.pts_seconds;
          vf.bytes = std::make_shared<std::vector<std::uint8_t>>(std::move(m.payload));
          pm.vbuf.push_back(std::move(vf));
          while (pm.vbuf.size() > 60) // never buffer more than ~1 s (bounds a stalled/absent clock)
            pm.vbuf.pop_front();
        }
      }
    }
  }

  // A/V sync: display the newest buffered video frame the peer's audio has reached. The audio
  // currently at the speaker has pts (last-enqueued pts - queued seconds); both streams were
  // stamped with the sender's world clock, so showing the frame at that pts lines the lips to the
  // sound. With no audio from a peer the target is +inf, so the freshest frame shows immediately
  // (no added delay).
  void sync_and_show() {
    for (auto &kv : peers_) {
      peer_media &pm = kv.second;
      if (!pm.video_proxy)
        continue;
      if (pm.video_muted) { // locally hidden: stop advancing (the tile freezes); drop the backlog
        pm.vbuf.clear();
        continue;
      }
      // This peer's audio latency = what sits in its mixer queue + what is queued in the shared
      // output. Show the video frame whose pts the audio has reached: last_audio_pts minus that.
      double target = 1e18;
      if (pm.has_audio && mix_out_ && mix_rate_ > 0 && mix_ch_ > 0 && mix_frames_ > 0) {
        const double bps = static_cast<double>(mix_rate_) * static_cast<double>(mix_ch_) * 4.0;
        const double out_s = bps > 0.0 ? static_cast<double>(mix_out_->queued_bytes()) / bps : 0.0;
        const double chunk_s = static_cast<double>(mix_frames_) / static_cast<double>(mix_ch_) /
                               static_cast<double>(mix_rate_);
        const double peer_s = static_cast<double>(pm.achunks.size()) * chunk_s;
        target = pm.last_audio_pts - out_s - peer_s;
      }
      std::shared_ptr<std::vector<std::uint8_t>> show;
      double show_pts = pm.shown_pts;
      while (!pm.vbuf.empty() &&
             pm.vbuf.front().pts <= target) { // drop all due frames, keep the last
        show = pm.vbuf.front().bytes;
        show_pts = pm.vbuf.front().pts;
        pm.vbuf.pop_front();
      }
      if (show && show_pts > pm.shown_pts) {
        pm.video_proxy->channel().publish_external(show->data(), show->size(), show_pts, show);
        pm.shown_pts = show_pts;
      }
    }
  }
#else
  bool active = false;
  bool setup(cvc::app &, const std::string &, cvc::gl::ImGuiBackend &) {
    if (const char *r = std::getenv("CVC_ARIADNE_RELAY"); r && *r)
      std::fprintf(stderr, "arigl: CVC_ARIADNE_RELAY is set but this build has no conference "
                           "transport (CVC_ENABLE_WS=OFF and CVC_ENABLE_GRPC=OFF); running the "
                           "document locally.\n");
    return false;
  }
  void attach_view(cvc::gl::SceneRenderer &, cvc::gl::CameraController &) {}
  void pump() {}
  void stop() {}
#endif
};

// ── a real window (ImGui over the VTK scene) ────────────────────────────────────────────────────

bool has_flag(const ari::HostOptions &o, const char *flag) {
  return std::find(o.extra_flags.begin(), o.extra_flags.end(), flag) != o.extra_flags.end();
}

// The GL a SceneRenderer renders with: its vendor, renderer and version lines, from one offscreen
// render. False where there is no usable OpenGL (checked first, so a headless box says so rather
// than failing in VTK).
bool describe_gl(cvc::gl::SceneRenderer &view, std::string &out) {
  vtkRenderWindow *w = view.renderWindow();
  if (!w || !w->SupportsOpenGL())
    return false;
  w->Render();
  const char *caps = w->ReportCapabilities();
  if (!caps)
    return true;
  std::istringstream lines(caps);
  for (std::string line; std::getline(lines, line);)
    if (line.rfind("OpenGL vendor", 0) == 0 || line.rfind("OpenGL renderer", 0) == 0 ||
        line.rfind("OpenGL version", 0) == 0)
      out += line + "\n";
  return true;
}

int print_info(ari::HostInfo info, const ari::HostOptions &o) {
  if (!o.doc.empty()) {
    std::fprintf(stderr,
                 "arigl: info takes no document (arigl check doc.ari reports what one needs)\n");
    return 2;
  }
  std::fputs(ari::format_host_info(info, has_flag(o, "--json"), has_flag(o, "--functions")).c_str(),
             stdout);
  return 0;
}

int run_window(const ari::HostOptions &o) {
  const bool offscreen = o.command != "run"; // snapshot and check never open a window
  if (o.command == "snapshot" && o.png.empty()) {
    std::fprintf(stderr, "arigl: a window snapshot needs --png FILE (or use --terminal)\n");
    return 2;
  }
  const int width = o.width ? o.width : 1024, height = o.height ? o.height : 768;

  cvc::app app;
  cvc::gl::SceneGraph sg(app, o.prefix.empty() ? kDefaultPrefix : o.prefix);
  cvc::gl::SceneRenderer view(sg, width, height, offscreen, "arigl");
  view.setBackground(0.09, 0.10, 0.12);
  cvc::gl::CameraController cam(view);
  cam.setMode(cvc::gl::CameraController::Mode::Orbit);
  cvc::gl::TouchGestures touch(view, cam); // Tab orbit / fly, drag to look
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true);

  gla::AriRuntime rt(view, cam, ui);
  gla::register_gl_basics(app);
  gla::register_streaming(app);
  gla::register_window_widgets(rt.backend(), app);
  if (!publish_images(rt.backend(), o))
    return 1;
  // Conference mode (no-op unless CVC_ARIADNE_RELAY is set): register the room verbs and dial the
  // relay (WebSocket for ws://|wss://, gRPC for grpc://), so the document (conference_roster.ari)
  // joins a hosted room.
  ConferenceSession conf;
  // A conference-setup failure must not take down the runtime — fall back to running the document
  // locally (no relay) and say why, rather than letting the exception abort main.
  try {
    if (conf.setup(app, sg.getStatePrefix(), rt.backend()))
      conf.attach_view(view, cam); // the room in 3D behind the conference windows
  } catch (const std::exception &e) {
    std::fprintf(stderr, "arigl: conference setup failed (%s); running the document locally\n",
                 e.what());
  }
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); });
  rt.app_runtime().set_document_args(o.doc_args, "arigl " + o.command + " " + o.doc,
                                     /*validate=*/o.command != "check");
  if (o.command == "info") {
    ari::HostInfo info = ari::gather_host_info(rt.app_runtime().runtime(), "arigl");
    info.backend = "imgui";
    info.gl = describe_gl(view, info.gl_info);
    return print_info(std::move(info), o);
  }
  if (const int status = load_document(rt, o); status >= 0)
    return status;
  note_misplaced(rt.app_runtime(), o);
  // The rich scene panels (scene_panel / stage_lighting_panel / scene_menu), for a document that
  // uses them rather than the declarative components. They need the realized lighting rig.
  if (const gla::RealizedScene *realized = rt.realized()) {
    cvc::gl::StageLighting *rig = realized->rigs.empty() ? nullptr : realized->rigs.front().get();
    gla::register_scene_panels(rt.backend(), sg, rig);
  }
  if (o.command == "check")
    return 0;

  if (o.command == "snapshot") {
    rt.set_fixed_clock(true); // reproducible: the scene clock advances 1/30 s a frame
    const int frames = std::max(1, o.frames);
    const auto period = snapshot_period(o);
    for (int f = 0; f < frames; ++f) {
      const auto start = std::chrono::steady_clock::now();
      conf.pump(); // publish this frame's room-tree + media to the relay (run mode does this via
                   // the input pump, set below; a snapshot of a live conference must pump too)
      rt.frame(1.0 / 30.0);
      if (f + 1 == frames)
        view.writePNG(o.png.c_str()); // renders and writes the last frame
      else
        rt.render();
      std::this_thread::sleep_until(start + period);
    }
    print_warnings(rt.take_warnings());
    return 0;
  }

  // Input the VTK interactor does not carry: touch navigation, SDL events (a wasm canvas), and the
  // OS signals trapped below. Fed before the drain, so the document sees them this frame.
  cvc::gl::SdlInput sdl;
  sdl.init();
  std::string exit_signal;
  rt.set_input_pump([&] {
    conf.pump(); // push this frame's room-tree writes (room-join + roster) to the relay
    touch.update();
    for (const cvc::gl::InputEvent &e : sdl.poll())
      rt.post_input(to_ari(e));
    for (std::string sig = ari::take_trapped_signal(); !sig.empty();
         sig = ari::take_trapped_signal())
      if (!rt.post_signal(sig, "os")) { // no on_signal: the host's default is to stop
        exit_signal = sig;
        rt.request_close();
      }
  });
  ari::install_signal_traps();
  view.setFrameYield(cvc::gl::SceneRenderer::FrameYield::App);
  const double fps = o.fps > 0 ? o.fps : 120.0;
#ifndef __EMSCRIPTEN__
  const auto period = std::chrono::duration<double>(1.0 / fps);
#endif
  while (!rt.should_close()) {
#ifdef __EMSCRIPTEN__
    // Browser: one frame, then yield to the event loop (Asyncify). No wall-clock pacing — the
    // browser throttles requestAnimationFrame; FrameYield::App above keeps VTK from yielding twice.
    rt.step(1.0 / fps);
    print_warnings(rt.take_warnings());
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush(); // single-thread wasm: no worker drains publishes, so do it per frame
#endif
    emscripten_sleep(0);
#else
    const auto start = std::chrono::steady_clock::now();
    rt.step(1.0 / fps);
    print_warnings(rt.take_warnings());
    std::this_thread::sleep_until(start + period);
#endif
  }
  conf.stop(); // join the transport reader before the shard (owned by conf) is destroyed
  ari::restore_signal_traps();
  cam.detach();
  if (!exit_signal.empty())
    return 128 + ari::signal_number(exit_signal);
  return 0;
}

// ── the terminal (FTXUI / tvision widgets, the scene rendered behind them) ──────────────────────

// TerminalScene plus the scene-node customs check a GL host makes before realizing.
class CheckedTerminalScene : public gla::TerminalScene {
public:
  using gla::TerminalScene::TerminalScene;
  bool verify(const ari::LoadResult &loaded, std::vector<std::string> *errors) override {
    return gla::verify_scene_customs(loaded, errors);
  }
};

int run_terminal(const ari::HostOptions &o) {
  if (!ari::FtxuiBackend::available()) {
    std::fprintf(stderr, "arigl: libcvc was built without FTXUI (no terminal backend)\n");
    return 1;
  }
  // snapshot / check always lay out with FTXUI; tvision only changes the interactive chrome.
  const bool tvision = o.backend == "tvision" && o.command == "run";
  if (tvision && !ari::TvisionBackend::available()) {
    std::fprintf(stderr, "arigl: libcvc was built without tvision (try --backend ftxui)\n");
    return 1;
  }

  cvc::app app;
  // Lifetime order: the backend outlives the scene adapter, which outlives the AppRuntime.
  ari::FtxuiBackend::Options look;
  look.image_color = image_mode(o.graphics);
  std::unique_ptr<ari::FtxuiBackend> backend = tvision ? std::make_unique<ari::TvisionBackend>(look)
                                                       : std::make_unique<ari::FtxuiBackend>(look);
  if (!publish_images(*backend, o))
    return 1;
  // stream_view in the terminal: live frames as the backend's images, JPEG ones decoded by cvcGL.
  const auto streams = ari::register_stream_view_widget(*backend, app);
  streams->set_decoder("mjpeg", gla::jpeg_frame_decoder());
  streams->set_decoder("jpeg", gla::jpeg_frame_decoder());
  // One state root for the document AND the scene graph, as in a window (AriRuntime uses the
  // scene's prefix): node realizers seed their state (forest.*, light.*, ...) under the scene's
  // prefix, so a different one leaves the document's panels reading nothing.
  const std::string prefix = o.prefix.empty() ? kDefaultPrefix : o.prefix;
  CheckedTerminalScene scene(app, *backend, /*supersample=*/2, prefix);
  ari::AppRuntime rt(app, prefix, &scene);
  rt.set_backend(*backend);
  gla::register_gl_basics(app);
  gla::register_streaming(app);
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); });
  rt.set_document_args(o.doc_args, "arigl " + o.command + " --terminal " + o.doc,
                       /*validate=*/o.command != "check");
  // Ask the terminal what it can draw before the load, so requires: / (has? "term.kitty") see it
  // (the interactive loop then does not ask again). tvision draws half-blocks only.
  ari::HostInfo term;
  ari::describe_terminal(term);
  ari::TermGraphics graphics;
  const bool probe = (o.command == "run" || o.command == "info") && o.backend != "tvision" &&
                     term.tty &&
                     (look.image_color == ari::FtxuiBackend::Options::ImageColor::Auto ||
                      look.image_color == ari::FtxuiBackend::Options::ImageColor::Kitty ||
                      look.image_color == ari::FtxuiBackend::Options::ImageColor::Sixel);
  if (probe) {
    graphics = ari::gfx::probe_terminal();
    if (o.cell_w > 0) {
      graphics.cell_w = o.cell_w;
      graphics.cell_h = o.cell_h;
    }
    backend->set_terminal_graphics(graphics);
  }
  if (o.command == "info") {
    ari::HostInfo info = ari::gather_host_info(rt.runtime(), "arigl --terminal");
    info.backend = o.backend == "tvision" ? "tvision" : "ftxui";
    info.gl = true; // the scene behind the widgets renders offscreen
    info.gl_info = "offscreen, at the cells' size (the scene behind the widgets)\n";
    if (probe) {
      info.graphics = graphics;
      info.probed = true;
    }
    return print_info(std::move(info), o);
  }
  if (const int status = load_document(rt, o); status >= 0)
    return status;
  note_misplaced(rt, o);
  if (o.command == "check")
    return 0;

  if (o.command == "snapshot") {
    const int width = std::max(10, o.width ? o.width : 100);
    const int height = std::max(4, o.height ? o.height : 30);
    using Mode = ari::FtxuiBackend::Options::ImageColor;
    const bool pixels =
        o.ansi && (look.image_color == Mode::Kitty || look.image_color == Mode::Sixel);
    if (pixels) { // the cell size images are drawn at: the terminal's, or --cell-size
      ari::TermGraphics g = ari::gfx::probe_terminal();
      if (o.cell_w > 0) {
        g.cell_w = o.cell_w;
        g.cell_h = o.cell_h;
      }
      backend->set_terminal_graphics(g);
    }
    backend->set_screen_size(width, height);
    std::string frame;
    const auto period = snapshot_period(o);
    for (int f = 0; f < std::max(2, o.frames); ++f) { // the first drain realizes; then a frame
      const auto start = std::chrono::steady_clock::now();
      rt.drain(); // services and renders the scene, publishing the frame
      frame = backend->render_to_string(rt.runtime(), width, height, o.ansi);
      std::this_thread::sleep_until(start + period);
    }
    if (pixels) { // Sixel draws at screen positions, so the frame starts at the top-left
      const int rows = static_cast<int>(std::count(frame.begin(), frame.end(), '\n')) + 1;
      frame = "\x1b[H\x1b[2J" + frame + backend->take_graphics() + "\x1b[" +
              std::to_string(rows + 1) + ";1H";
    }
    std::fputs(frame.c_str(), stdout);
    if (!o.png.empty())
      scene.write_png(o.png);
    print_warnings(rt.take_warnings());
    return 0;
  }

  ari::FtxuiBackend::RunOptions run;
  if (o.fps > 0)
    run.fps = o.fps;
  run.mouse = o.mouse;
  run.on_start = ari::install_signal_traps;
  run.on_stop = ari::restore_signal_traps;
  run.take_signal = ari::take_trapped_signal;
  run.background_input = [&](const ari::TermEvent &e) { return scene.handle_input(e); };
  ari::FtxuiBackend::RunExit how;
  if (tvision)
    how = static_cast<ari::TvisionBackend &>(*backend).run(rt, run);
  else
    how = backend->run(rt, run);
  print_warnings(rt.take_warnings());
  if (how.from_os) // an OS signal the document didn't handle: report it the way a shell expects
    return 128 + ari::signal_number(how.signal);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  const ari::HostOptions o =
      ari::parse_host_args(argc, argv, {"--terminal", "--json", "--functions"});
  if (o.help)
    return usage(0);
  if (!o.error.empty()) {
    std::fprintf(stderr, "arigl: %s\n", o.error.c_str());
    return usage(2);
  }
  if (o.command != "run" && o.command != "snapshot" && o.command != "check" &&
      o.command != "info") {
    std::fprintf(stderr, "arigl: unknown command '%s'\n", o.command.c_str());
    return usage(2);
  }
  if (!o.backend.empty() && o.backend != "ftxui" && o.backend != "tvision") {
    std::fprintf(stderr, "arigl: --backend is ftxui or tvision, not '%s'\n", o.backend.c_str());
    return 2;
  }
  const bool terminal = has_flag(o, "--terminal");
  if (o.command != "info" && (has_flag(o, "--json") || has_flag(o, "--functions"))) {
    std::fprintf(stderr, "arigl: --json and --functions are for arigl info\n");
    return usage(2);
  }
  if (!terminal && (!o.backend.empty() || !o.graphics.empty() || o.cell_w > 0)) {
    std::fprintf(stderr, "arigl: --backend, --graphics and --cell-size are for a terminal; add "
                         "--terminal\n");
    return 2;
  }
  if (!ari::AppRuntime::have_yaml()) {
    std::fprintf(stderr, "arigl: libcvc was built without yaml-cpp, so it cannot load a .ari\n");
    return 1;
  }
  return terminal ? run_terminal(o) : run_window(o);
}

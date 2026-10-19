// ariadne_conference — the conference host bundle (roadmap §8): the ONLY C++ a conference .ari
// needs. It extends the stock cvcGL Ariadne runner (cf. ariadne_hello / ariadne_stream) with the
// five stream/media registrations, then runs a document that drives the whole media pipeline from
// the DSL program lane — no C++ opens a stream here.
//
//   register_stream_intrinsics   (core)  (stream-open/info/close)
//   register_gl_stream_intrinsics (cvcGL) (gl-bind-stream)
//   register_media_intrinsics    (cvcGL) (stream-capture-camera / -audio / stream-play-audio)
//   register_stream_node_type    (cvcGL) `type: stream`      — video-on-quad scene node
//   register_stream_view_widget  (cvcGL) `type: stream_view` — video-in-an-ImGui-window widget
//
// With these, a pure-.ari document (conference_media.ari) can, from an on_tick/action lane,
// (stream-open) a camera and a mic, attach producers with (stream-capture-camera)/(-audio), render
// the video in a reactive `stream_view` widget, and (stream-play-audio) the mic. Hardware-free by
// default: synthetic video + a 440 Hz tone, so it runs and verifies HEADLESS. A real camera/mic/
// speakers need a build with CVC_ENABLE_SDL (the .ari passes device >= 0).
//
//   ariadne_conference conference_media.ari                                  # navigable window
//   ariadne_conference conference_media.ari --offscreen --png conf.png       # one captured frame
//   ariadne_conference conference_media.ari --frames 32                      # headless, no PNG

#include <cstdio>
#include <cstdlib>
#include <cstring>
#ifndef _WIN32
#include <unistd.h> // gethostname (default --name to the host name; Windows reads COMPUTERNAME)
#endif
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // register_room_intrinsics (room-join/roster/leave)
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/stream/muted_source.h> // --camera: mute gate for the host-owned camera stream
#include <cvc/ariadne/stream/stream.h> // --camera: open the video stream sized to the device
#include <cvc/ariadne/stream/stream_registry.h> // post-run verification: lookup the opened tokens
#include <cvc/ariadne/stream_intrinsics.h> // register_stream_intrinsics (stream-open/info/close)
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/media_verbs.h> // register_media_intrinsics (capture-camera/-audio, play-audio)
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_node.h>        // register_stream_node_type (type: stream)
#include <cvc/gl/ariadne/stream_verbs.h>       // register_gl_stream_intrinsics (gl-bind-stream)
#include <cvc/gl/ariadne/stream_view_widget.h> // register_stream_view_widget (type: stream_view)
#include <cvc/volume/bounding_box.h>
#include <memory>
#include <string>
#include <vector>

#if CVC_ENABLE_SDL
#include <cvc/gl/capture/camera_source.h> // --camera: a real SDL3 camera as the self-view source
#include <cvc/gl/capture/jpeg_encode_source.h> // compress camera video to JPEG for the network
#endif

#if CVC_ENABLE_GRPC
#include <chrono>
#include <cvc/ariadne/conference/room_media_intrinsics.h> // register_room_media_intrinsics
#include <cvc/ariadne/stream/stream_bridge.h>             // the media bridge
#include <cvc/state/state_cluster_shard.h>                // the room-tree cluster shard
#include <cvc/state/state_stream_transport_grpc.h>        // stream_transport_grpc (media plane)
#include <cvc/state/state_transport_grpc.h>               // state_transport_grpc (room-tree plane)
#endif

using cvc::gl::CameraController;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;
namespace sx = cvc::ariadne::stream;

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false, useCamera = false, useMic = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  // Clustering (live multi-instance conference, roadmap §4b/§7). Giving --node-id turns on the
  // cross-process room cluster: this instance serves its room-tree state at --state-listen and (if
  // --state-peer is given) dials that peer; media rides a gRPC bridge at --media-listen (ephemeral
  // is fine — the resolved address is advertised through the room tree). --peer-id tells the demo
  // .ari whose streams to subscribe to. Without --node-id the host is the single-node demo.
  std::string nodeId, roomId = "standup", peerId;
  std::string stateListen, statePeer, mediaListen = "127.0.0.1:0";
  std::string mediaServer; // --media-server <addr>: SFU mode — push/pull media through this relay
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--png")) {
      png = next("");
      offscreen = true;
    } else if (!std::strcmp(a, "--camera"))
      useCamera = true;
    else if (!std::strcmp(a, "--mic"))
      useMic = true;
    else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (!std::strcmp(a, "--node-id") || !std::strcmp(a, "--name"))
      nodeId = next(""); // this participant's display name + cluster id (unique per participant)
    else if (!std::strcmp(a, "--room"))
      roomId = next("standup");
    else if (!std::strcmp(a, "--peer-id"))
      peerId = next(""); // deprecated/ignored: N-party discovers all peers from the room roster
    else if (!std::strcmp(a, "--state-listen"))
      stateListen = next("");
    else if (!std::strcmp(a, "--state-peer"))
      statePeer = next("");
    else if (!std::strcmp(a, "--media-listen"))
      mediaListen = next("127.0.0.1:0");
    else if (!std::strcmp(a, "--media-server"))
      mediaServer = next("");
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  // Joining a room (a peer to dial or a state addr to serve) with no explicit --name defaults the
  // display name to this host's name — so a friend just runs with --state-peer/--media-server and
  // is discovered by everyone else through the replicated roster (no --peer-id needed).
  if (nodeId.empty() && (!statePeer.empty() || !stateListen.empty())) {
#ifdef _WIN32
    // COMPUTERNAME is reliably set on Windows (unlike POSIX $HOSTNAME); avoids pulling in <windows.h>
    // (and its macro collisions) just for GetComputerName.
    const char *hn = std::getenv("COMPUTERNAME");
    nodeId = (hn && hn[0]) ? std::string(hn) : std::string("guest");
#else
    char hn[256] = {0};
    nodeId = (gethostname(hn, sizeof(hn) - 1) == 0 && hn[0]) ? std::string(hn) : std::string("guest");
#endif
  }
  const bool clustered = !nodeId.empty();
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 16; // let the producers deliver and the texture upload before the capture

  if (docPath.empty()) {
    std::printf("[ariadne_conference] FATAL: give a .ari document (e.g. conference_media.ari)\n");
    return 1;
  }

  cvc::app app;
  // When clustered, scope this instance's document (and thus its self-stream tokens
  // <scope>.streams.<id>) by the node id, so two participants that both open a stream called
  // "camera" get DISTINCT tokens and a subscriber's proxy never collides with its own stream.
  SceneGraph sg(app, clustered ? nodeId : std::string("conference"));
  SceneRenderer view(sg, width, height, offscreen, "main");
  view.setBackground(0.06, 0.07, 0.09);
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true); // the stream_view window is the point — keep it in captures too

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  bool quit = false;
  rt.on("quit", [&] { quit = true; });

  // ── The conference host bundle (§8): the registrations, once, before the scene loads ──
  // These are the only C++ a conference .ari needs; everything else is DSL.
  ari::register_stream_intrinsics(app);              // (stream-open/info/close) — core
  ari::conference::register_room_intrinsics(app);    // (room-join/roster/leave) — core, §4b
  cvc::gl::ariadne::register_gl_stream_intrinsics(); // (gl-bind-stream)
  cvc::gl::ariadne::register_media_intrinsics(); // (stream-capture-camera / -audio / play-audio)
  cvc::gl::ariadne::register_stream_node_type(); // type: stream (video-on-quad scene node)
  cvc::gl::ariadne::register_stream_view_widget(backend, app); // type: stream_view (ImGui window)

  // ── Clustering (§4b/§7): join a cross-process room cluster so two instances can see/hear each
  // other. Owned at main scope so they outlive the documents; explicitly stopped at teardown. ──
#if CVC_ENABLE_GRPC
  std::unique_ptr<cvc::state_transport_grpc> state_tp;
  std::unique_ptr<cvc::state_cluster_shard> shard;
  std::unique_ptr<cvc::stream_transport_grpc> media_tp;
  std::unique_ptr<sx::stream_bridge> bridge;
  std::string media_endpoint; // this node's media-bridge address (advertised in the room tree)
  bool cam_published = false; // live camera is published once, after the .ari joins the room
  if (clustered) {
    if (stateListen.empty()) {
      std::printf("[ariadne_conference] FATAL: --node-id needs --state-listen <addr>\n");
      return 1;
    }
    // Room-tree (state) plane: serve this node's shard + optionally dial a peer. One bidirectional
    // stream carries replication both ways, and backfill-on-connect converges a late joiner.
    state_tp = std::make_unique<cvc::state_transport_grpc>();
    state_tp->start(stateListen, nodeId, "conf");
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", nodeId);
    shard->attach();
    shard->install_as_default(); // room-join/-publish derive the pid from the default shard's node
                                 // id
    state_tp->register_shard(shard.get());
    if (!statePeer.empty()) {
      if (state_tp->connect_to_peer(statePeer, std::chrono::milliseconds(4000))) {
        // Bootstrap the CURRENT room tree from the server. The journal backfill on connect carries
        // only new mutations, so a late joiner would miss participants who joined before it — a
        // snapshot carries the whole tree (every participant + stream descriptor), ingested into
        // our shard, so the roster and the tiles converge for the 3rd, 4th, ... person to join.
        state_tp->request_snapshot(
            "conf", "",
            [&shard](const std::vector<cvc::state_transport::snapshot_entry> &entries, bool) {
              if (shard)
                for (const auto &e : entries)
                  shard->ingest_snapshot_entry(e);
            });
      } else {
        std::printf("[ariadne_conference] WARN: could not dial state peer %s (still serving)\n",
                    statePeer.c_str());
      }
    }
    // Media plane: a gRPC stream bridge. room-publish serve_port()s on it; room-subscribe pulls a
    // peer's stream from the endpoint the peer advertised in the room tree.
    media_tp = std::make_unique<cvc::stream_transport_grpc>();
    bridge = std::make_unique<sx::stream_bridge>(app, *media_tp);
    media_tp->start(mediaListen);
    media_endpoint = media_tp->listen_address();
    // SFU when --media-server is set (push/pull media through the relay); mesh otherwise.
    ari::conference::register_room_media_intrinsics(app, *bridge, media_endpoint, mediaServer);
    // Tell the demo .ari who it is (scoped to this document's prefix = nodeId).
    std::vector<std::string> ce;
    ari::run_init(app, sg.getStatePrefix(),
                  "(state-set \"conf.room\" \"" + roomId + "\") (state-set \"conf.self\" \"" +
                      nodeId + "\")" + (useMic ? " (state-set \"mic_live\" \"1\")" : ""),
                  &ce);
    std::printf("[ariadne_conference] clustered: node='%s' room='%s' | state=%s dial=%s | media=%s\n",
                nodeId.c_str(), roomId.c_str(), state_tp->listen_address().c_str(),
                statePeer.empty() ? "(none)" : statePeer.c_str(), media_endpoint.c_str());
  }
#else
  if (clustered)
    std::printf(
        "[ariadne_conference] --node-id needs a CVC_ENABLE_GRPC build; running single-node.\n");
#endif

  // Load the document, realize its scene. NOTE: no stream is opened here — the .ari opens and
  // captures from its on_tick program lane (stream-open needs the document scope, which the init:
  // lane lacks), and a reactive stream_view widget binds to the token each frame as it goes live.
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::printf("[ariadne_conference] load failed: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("[ariadne_conference]   %s\n", w.c_str());
  std::vector<std::string> init_errs;
  ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs);
  for (const std::string &e : init_errs)
    std::printf("[ariadne_conference] init: %s\n", e.c_str());

  // ── --camera: open a REAL camera in C++ (roadmap §8 live path) ──
  // A camera dictates its own resolution, so the DSL cannot pre-size a stream to it (the stream w/h
  // MUST equal the device's, as the source converts pixel format but does not rescale). So the host
  // opens the self-view camera here, sizes the video stream to the device, and sets cam_live=1 +
  // cam_token in state so the .ari SKIPS its synthetic open and binds the stream_view widget to
  // this token. Audio stays DSL-driven (the 440 Hz tone). Opened on this (the SDL main) thread per
  // the camera_source.h thread contract; closed below before teardown. Hardware-free without
  // --camera (or without SDL / a device): the .ari opens its own synthetic self-view, exactly as
  // before.
  std::unique_ptr<sx::stream> live_cam;
#if CVC_ENABLE_SDL
  if (useCamera) {
    cvc::gl::capture::camera_open camo = cvc::gl::capture::open_camera(0);
    if (camo.source) {
      sx::stream_params sp;
      // Clustered: the "camera" kind (token <node>.streams.camera) the peer subscribes to and the
      // host publishes below. Single-node: "selfcam", the id conference_media.ari binds its widget
      // to. Same scope the DSL opens under (the document prefix = node id when clustered).
      sp.id = clustered ? "camera" : "selfcam";
      sp.root_path = sg.getStatePrefix();
      // Compress camera video to JPEG ("mjpeg") so it fits a real internet uplink: raw rgba8 is
      // ~1.9 MB/frame (unusable over home broadband), JPEG is ~10-50x smaller. The byte slab is
      // sized for the worst-case JPEG; the stream_view decodes on the subscriber.
      sp.format.kind = sx::frame_kind::video_raw;
      sp.format.codec = "mjpeg";
      sp.format.w = camo.width;
      sp.format.h = camo.height;
      sp.format.stride = camo.width * 4; // decoded geometry (informational for mjpeg)
      sp.format.bytes = cvc::gl::capture::jpeg_stream_slab(camo.width, camo.height);
      sp.expected_subscribers = 3; // the stream_view widget (+ headroom); cf. ariadne_stream
      live_cam = sx::stream::open(app, sp);
      if (live_cam) {
        // Mute gate (UX cam toggle): the host camera is C++-owned (not in the document stream table,
        // so the DSL stream-set-muted can't reach it) — wrap it in a muted_source bound to the
        // stream's mute_flag(), then drive that flag from the render loop by polling the .ari's
        // cam_on. Wrapped BEFORE the JPEG encoder so a muted frame is black rgba -> a black JPEG.
        auto gated =
            std::make_unique<sx::muted_source>(std::move(camo.source), live_cam->mute_flag());
        auto enc = std::make_unique<cvc::gl::capture::jpeg_encode_source>(std::move(gated),
                                                                          camo.width, camo.height);
        live_cam->start_producer(std::move(enc), camo.fps > 0 ? camo.fps : 30.0);
        std::vector<std::string> ce;
        ari::run_init(app, sg.getStatePrefix(),
                      "(state-set \"cam_live\" \"1\") (state-set \"cam_token\" \"" +
                          live_cam->token() + "\")",
                      &ce);
        for (const std::string &e : ce)
          std::printf("[ariadne_conference] camera init: %s\n", e.c_str());
        std::printf("[ariadne_conference] live camera: %s %dx%d -> %s\n", camo.name.c_str(),
                    camo.width, camo.height, live_cam->token().c_str());
      } else {
        std::printf(
            "[ariadne_conference] --camera: stream::open failed; the .ari uses synthetic.\n");
      }
    } else {
      std::printf("[ariadne_conference] --camera: no camera opened; the .ari uses synthetic.\n");
    }
  }
#else
  (void)useCamera;
#endif

  rt.set_tick_program(lr.on_tick_script);
  rt.set_root(std::move(lr.root));

  cvc::gl::ariadne::RealizedScene realized;
  if (lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &sw);
    for (const std::string &w : sw)
      std::printf("[ariadne_conference]   %s\n", w.c_str());
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  }

  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain(); // runs the on_tick program lane: (stream-open) + (stream-capture-*) +
                // (stream-play-audio) + (when clustered) (room-join/-publish/-subscribe)
    // UX cam toggle (host camera path): the .ari's button flips cam_on, but the host-owned camera
    // stream is not in the document table, so the DSL stream-set-muted can't reach it — apply the
    // mute here instead by polling cam_on (prefix-scoped like every DSL key). Default ON; muted only
    // when cam_on is explicitly "0". Runs on the render/owner thread, same as set_muted expects.
    if (live_cam) {
      const std::string cop = sg.getStatePrefix().empty()
                                  ? std::string("cam_on")
                                  : sg.getStatePrefix() + cvc::state::SEPARATOR + "cam_on";
      cvc::state *n = app.root().findDescendant(cop);
      live_cam->set_muted(n && n->value() == "0");
    }
#if CVC_ENABLE_GRPC
    if (state_tp)
      state_tp->pump_all(); // publish this frame's room-tree writes to the peer
    // Publish the live camera ONCE, after the .ari's first tick has joined the room (room_publish
    // needs our participant subtree to exist). The host owns the camera stream — it alone knows the
    // device resolution needed to size it — so it publishes here rather than from the DSL; the .ari
    // skips its synthetic camera when cam_live is set.
    if (live_cam && clustered && bridge && !cam_published) {
      for (const ari::conference::room_participant &p : ari::conference::room_roster(app, roomId))
        if (p.pid == nodeId) {
          ari::conference::room_publish(app, *bridge, media_endpoint, roomId, nodeId, "camera",
                                        *live_cam, mediaServer);
          cam_published = true;
          std::printf("[ariadne_conference] published live camera (%s/camera)\n", nodeId.c_str());
          break;
        }
    }
#endif
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), capturing ? dt : -1.0);
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 30.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("[ariadne_conference] captured %ld frame(s)%s%s\n", frames,
                png.empty() ? "" : " -> ", png.c_str());
  } else {
    std::puts("[ariadne_conference] running — close the window or Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render();
    }
  }

  if (live_cam)
    live_cam->close(); // join the camera producer before teardown (render-thread contract)

#if CVC_ENABLE_GRPC
  // Stop the media pullers + transports BEFORE the Runtime tears down (it closes the pulled
  // proxies), so no puller feeds a proxy as it closes; then the room-tree cluster.
  if (bridge)
    bridge->stop();
  if (media_tp)
    media_tp->stop();
  if (state_tp) {
    if (shard)
      state_tp->unregister_shard(shard.get());
    state_tp->stop();
  }
  if (shard)
    shard->detach();
  if (clustered) {
    // Report what converged: the replicated roster, and how many OTHER participants' cameras this
    // node pulled (N-party — every non-self participant should have a subscribed camera proxy).
    std::printf("[ariadne_conference] roster:");
    for (const ari::conference::room_participant &p : ari::conference::room_roster(app, roomId))
      std::printf(" %s(%s)", p.pid.c_str(), p.name.c_str());
    std::printf("\n");
    auto &reg = sx::stream_registry::for_app(app);
    int peers = 0, subscribed = 0;
    for (const ari::conference::room_participant &p : ari::conference::room_roster(app, roomId)) {
      if (p.pid == nodeId)
        continue;
      ++peers;
      const bool live = reg.lookup(p.pid + ".streams.camera") != nullptr;
      std::printf("[ariadne_conference] peer %s camera %s\n", p.pid.c_str(),
                  live ? "SUBSCRIBED" : "(not yet)");
      subscribed += live ? 1 : 0;
    }
    std::printf("[ariadne_conference] node '%s': subscribed %d/%d peers\n", nodeId.c_str(),
                subscribed, peers);
    return (peers > 0 && subscribed == peers) ? 0 : 3;
  }
#endif

  // ── Verify the single-node demo opened its two streams (the host opened none) ──
  // The DSL opens streams under <document-root>.streams.<id>; the single-node document root is the
  // SceneGraph state prefix ("conference"), so the tokens are conference.streams.<id>.
  auto &reg = sx::stream_registry::for_app(app);
  int opened = 0;
  for (const char *tok : {"conference.streams.selfcam", "conference.streams.selfmic"}) {
    const bool live = reg.lookup(tok) != nullptr;
    std::printf("[ariadne_conference] stream %-16s %s\n", tok, live ? "LIVE" : "(absent)");
    opened += live ? 1 : 0;
  }
  std::printf("[ariadne_conference] %d/2 streams opened from the .ari program lane\n", opened);
  return opened == 2 ? 0 : 2;
}

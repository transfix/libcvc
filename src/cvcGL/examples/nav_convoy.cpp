// nav_convoy — a navigation demo whose ENTIRE UI is the reusable Ariadne .ari component library,
// with only the genuinely-host parts in C++: a streamed "agents" scene node (register_scene_node_
// type + a per-frame geometry rebuild), a couple of capability verbs (sim.restart / sim.step /
// camera.recenter), and live readouts published to `stats.*` state keys. It is the library-based
// counterpart to the hand-written nav_city_drive.cpp — the scene, the control panels, and the
// display toggles are all DECLARED in nav_convoy.ari + cvc://components/*.ari, no bespoke panel
// code.
//
//   nav_convoy nav_convoy.ari --component-path <repo>/src/cvc/ariadne
//   nav_convoy nav_convoy.ari --offscreen --png convoy.png --component-path …

#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/bind.h> // read_bool_or / read_or / resolve_bind — read the sim.* control keys
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/volume/bounding_box.h>
#include <string>
#include <thread>
#include <vector>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;

namespace {

constexpr double kRing = 90.0; // world radius the convoy circles within

// Build `count` triangular vehicle markers at time `t`, each driving around a ring at its own
// radius and speed — a small, recognizable streamed convoy. Rebuilt each frame from the sim clock.
cvc::geometry build_agents(int count, double t) {
  cvc::geometry g;
  for (int i = 0; i < count; ++i) {
    const double phase = (2.0 * M_PI * i) / (count > 0 ? count : 1);
    const double speed = 0.35 + 0.04 * (i % 5);
    const double ang = phase + t * speed;
    const double r = kRing * (0.45 + 0.55 * ((i % 4) / 3.0)); // spread across a few lanes
    const double cx = r * std::cos(ang), cy = r * std::sin(ang);
    const double hx = -std::sin(ang), hy = std::cos(ang); // heading = the tangent
    const double px = -hy, py = hx;                       // perpendicular
    const double L = 4.0, W = 1.8, z = 0.5;               // marker size + hover height
    const unsigned base = 3u * static_cast<unsigned>(i);
    g.points().push_back({cx + hx * L, cy + hy * L, z});                               // nose
    g.points().push_back({cx - hx * L * 0.5 + px * W, cy - hy * L * 0.5 + py * W, z}); // left tail
    g.points().push_back({cx - hx * L * 0.5 - px * W, cy - hy * L * 0.5 - py * W, z}); // right tail
    for (int k = 0; k < 3; ++k)
      g.normals().push_back({0, 0, 1});
    g.tris().push_back({base, base + 1u, base + 2u});
  }
  return g;
}

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--png")) {
      png = next("");
      offscreen = true;
    } else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 30; // let the convoy move + shadows bake before the capture
  if (docPath.empty()) {
    std::fprintf(stderr,
                 "usage: nav_convoy <doc.ari> [--offscreen --png P] [--component-path DIR]\n");
    return 2;
  }

  cvc::app app;
  cvc::ariadne::register_cvc_uri_handler(componentPaths); // the cvc:// component library path
  SceneGraph sg(app, "convoy");
  SceneRenderer view(sg, width, height, offscreen, "main");

  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(!capturing);

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);

  const std::string prefix = sg.getStatePrefix();
  int agentCount = 24;

  // The genuinely-host scene piece: a streamed "agents" node. The realizer builds the initial
  // convoy from props; the host loop rebuilds its geometry each tick. `visible: show.agents` in the
  // .ari binds its visibility declaratively (the Display toggles drive it — no code here).
  cvc::gl::ariadne::register_scene_node_type(
      "agents", [&agentCount](SceneGraph &g, const ari::SceneNode &n, cvc::gl::GraphicsNode *parent,
                              cvc::gl::ariadne::RealizedScene &, std::vector<std::string> *) {
        agentCount = static_cast<int>(n.props.num("count", 24.0));
        cvc::geometry geom = build_agents(agentCount, 0.0);
        std::shared_ptr<GeometryNode> node =
            parent ? parent->createChild<GeometryNode>(n.id, geom)
                   : std::dynamic_pointer_cast<GeometryNode>(g.addGraphics(n.id, geom));
        if (node) {
          node->setUseSingleColor(true);
          node->setColor(0.95, 0.75, 0.22);
          node->setAmbient(0.35);
          node->setDiffuse(0.85);
        }
        return std::static_pointer_cast<cvc::gl::GraphicsNode>(node);
      });

  // Capability verbs (the host seam a bare-name on: routes to). sim.step advances one tick while
  // paused; sim.restart resets the clock; camera.recenter reframes the scene.
  double simT = 0.0;
  bool stepOnce = false;
  cvc::bounding_box sceneBounds;
  rt.on("sim.restart", [&] { simT = 0.0; });
  rt.on("sim.step", [&] { stepOnce = true; });
  rt.on("camera.recenter", [&] {
    if (!sceneBounds.isNull())
      cam.frameBounds(sceneBounds.minx, sceneBounds.miny, sceneBounds.minz, sceneBounds.maxx,
                      sceneBounds.maxy, sceneBounds.maxz);
  });
  rt.on("quit", [] {});

  cvc::gl::ariadne::RealizedScene realized;
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::fprintf(stderr, "nav_convoy: %s\n", lr.error.c_str());
    return 1;
  }
  std::printf("nav_convoy: loaded %s\n", docPath.c_str());
  for (const std::string &w : lr.warnings)
    std::printf("nav_convoy:   %s\n", w.c_str());
  std::vector<std::string> initErrs;
  ari::run_init(app, prefix, lr.init_script, &initErrs);
  for (const std::string &e : initErrs)
    std::printf("nav_convoy:   %s\n", e.c_str());
  std::vector<std::string> customErrs;
  const bool customsOk = cvc::gl::ariadne::verify_scene_customs(lr, &customErrs);
  for (const std::string &e : customErrs)
    std::printf("nav_convoy:   %s\n", e.c_str());
  if (customsOk && lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, prefix, &sw);
    for (const std::string &w : sw)
      std::printf("nav_convoy:   %s\n", w.c_str());
    sceneBounds = sg.computeGraphicsBounds();
    if (!sceneBounds.isNull())
      cam.frameBounds(sceneBounds.minx, sceneBounds.miny, sceneBounds.minz, sceneBounds.maxx,
                      sceneBounds.maxy, sceneBounds.maxz);
  }
  rt.set_root(std::move(lr.root));

  // The live "agents" geometry node (host-updated) and the sim-control keys the .ari binds.
  auto agents = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("agents"));
  const std::string kPaused = ari::resolve_bind(prefix, "sim.paused");
  const std::string kSpeed = ari::resolve_bind(prefix, "sim.speed");
  const std::string kStatAgents = ari::resolve_bind(prefix, "stats.agents");
  const std::string kStatFps = ari::resolve_bind(prefix, "stats.fps");
  ari::write<int>(app, kStatAgents,
                  agentCount); // published for a `text: bind stats.agents` readout

  auto tick = [&](double wall_dt) {
    view.processUIEvents();
    touch.update();
    cam.update(wall_dt);
    rt.drain();
    // Advance the convoy unless paused (Step nudges one frame while paused). Speed scales sim time.
    const bool paused = ari::read_bool_or(app, kPaused, 0) != 0;
    const double speed = ari::read_or<double>(app, kSpeed, 1.0);
    if (!paused || stepOnce) {
      simT += wall_dt * (speed > 0 ? speed : 0.0);
      stepOnce = false;
      if (agents)
        agents->setGeometry(build_agents(agentCount, simT));
    }
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    ari::write<int>(app, kStatFps, static_cast<int>(wall_dt > 0 ? 1.0 / wall_dt : 0));
    for (const std::string &w : rt.take_reactive_warnings())
      std::fprintf(stderr, "%s\n", w.c_str());
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      tick(1.0 / 30.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("nav_convoy: captured %ld frame(s)%s%s\n", frames, png.empty() ? "" : " -> ",
                png.c_str());
  } else {
    const auto t0 = std::chrono::steady_clock::now();
    double last = 0.0;
    while (!view.windowClosed()) {
      const double now =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const double dt = now - last;
      last = now;
      tick(dt > 0 ? dt : 1.0 / 120.0);
      view.render();
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
  }
  cam.detach();
  return 0;
}

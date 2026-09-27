// nav_sim — a REAL cvc::nav simulation driven through the reusable Ariadne .ari component UI.
// The scene + every control window are DECLARED (nav_sim.ari + cvc://components/*.ari); the host
// runs an actual cvc::nav::sim_world, steps it on the WORLD CLOCK (cvc::world_clock, fixed-dt
// quanta scaled by the sim.speed knob — never a raw wall_dt), renders the agents at their
// WORLD-METRE poses (sim_world::snapshot, the scene coordinate space — no ad-hoc rescale), applies
// the nav.knobs.* sliders each frame via sim_world::set_live_knobs, and publishes nav.stats.* for
// the readouts. The library-based counterpart to nav_city_drive.cpp — Cut 1 of the
// Ariadne<->cvc::nav bridge.
//
//   nav_sim nav_sim.ari --component-path <repo>/src/cvc/ariadne

#define _USE_MATH_DEFINES
#include "nav_common.h" // navdemo::add_border — a bordered synthetic occupancy

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/core/world_clock.h> // the sim advances on the world clock, not wall time
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/volume/bounding_box.h>
#include <memory>
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
namespace nav = cvc::nav;

namespace {

constexpr int kGrid = 128;      // occupancy grid
constexpr double kHalf = 100.0; // world half-extent in metres (the scene coord space)

// A synthetic bordered occupancy with a few interior blocks, so the agents have something to route
// around. 0 = free, 1 = occupied; row-major rows*cols.
std::vector<std::uint8_t> make_occupancy() {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(kGrid) * kGrid, 0);
  navdemo::add_border(occ.data(), kGrid, kGrid);
  const auto block = [&](int r0, int r1, int c0, int c1) {
    for (int r = r0; r < r1; ++r)
      for (int c = c0; c < c1; ++c)
        occ[static_cast<std::size_t>(r) * kGrid + c] = 1;
  };
  block(40, 56, 40, 56);
  block(40, 56, 72, 88);
  block(72, 88, 56, 72);
  return occ;
}

// The vehicle + world config. veh_params default to 0 (a bundle would supply them); pick physical
// values for the trained metric (scale = 0.05, "1 m = 20 world units",
// nav_city_drive.cpp:436,1128).
nav::sim_world::config make_config() {
  nav::sim_world::config cfg;
  cfg.rows = cfg.cols = kGrid;
  cfg.min_x = -kHalf;
  cfg.max_x = kHalf;
  cfg.min_y = -kHalf;
  cfg.max_y = kHalf;
  cfg.cx = 0.0;
  cfg.cy = 0.0;
  cfg.scale = 0.05; // the trained coef_mlp world metric
  cfg.veh.rr = 0.02f;
  cfg.veh.d_hat = 0.06f;
  cfg.veh.dt = 0.06f; // the sim quantum -> the world-clock fixed_dt below
  cfg.veh.vmax = 0.9f;
  cfg.reach_tol = 0.8f;
  cfg.freeze_sense = true; // a static known map (Cut 1: no fog divergence)
  cfg.sep_radius = 0.06f;  // steer around peers instead of through them
  cfg.sep_gain = 0.03f;
  return cfg;
}

// Build triangular agent markers at the snapshot poses (WORLD METRES = the scene coords, placed
// directly) pointing along heading. Rebuilt each frame from the sim.
cvc::geometry build_agents(const std::vector<float> &pos, const std::vector<float> &heading, int n,
                           double marker) {
  cvc::geometry g;
  for (int i = 0; i < n; ++i) {
    const double x = pos[2 * i], y = pos[2 * i + 1], th = heading[i];
    const double hx = std::cos(th), hy = std::sin(th); // heading
    const double px = -hy, py = hx;                    // perpendicular
    const double L = marker, W = 0.5 * marker, z = 0.4 * marker;
    const unsigned b = 3u * static_cast<unsigned>(i);
    g.points().push_back({x + hx * L, y + hy * L, z});
    g.points().push_back({x - hx * L * 0.5 + px * W, y - hy * L * 0.5 + py * W, z});
    g.points().push_back({x - hx * L * 0.5 - px * W, y - hy * L * 0.5 - py * W, z});
    for (int k = 0; k < 3; ++k)
      g.normals().push_back({0, 0, 1});
    g.tris().push_back({b, b + 1u, b + 2u});
  }
  return g;
}

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768, agents = 48;
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
    else if (!std::strcmp(a, "--agents"))
      agents = std::atoi(next("48"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 90; // let the agents move + shadows bake
  if (docPath.empty()) {
    std::fprintf(stderr, "usage: nav_sim <doc.ari> [--offscreen --png P] [--component-path DIR]\n");
    return 2;
  }

  cvc::app app;
  ari::register_cvc_uri_handler(componentPaths);
  SceneGraph sg(app, "nav");
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

  // Build the real cvc::nav sim. default_biased() is the hand-tuned net that runs with no trained
  // weights file (robust for a synthetic scene); a trained coef_mlp would replace it.
  const nav::sim_world::config cfg = make_config();
  const std::vector<std::uint8_t> occ = make_occupancy();
  std::unique_ptr<nav::sim_world> world;
  auto rebuild = [&](int n) {
    world = std::make_unique<nav::sim_world>(
        nav::sim_world::from_occupancy(cfg, occ.data(), nav::coef_mlp::default_biased(), n,
                                       /*seed=*/7, nav::sim_world::belief_mode::shared, 1));
    world->set_thread_pool(&app.computePool());
  };
  rebuild(agents);
  const int N = world->size();
  const double markerM = (cfg.max_x - cfg.min_x) / 80.0; // marker size in world metres

  // The world clock: one quantum == the sim's dt; sim.speed scales world time; paused banks
  // nothing.
  cvc::world_clock::config cc;
  cc.fixed_dt = cfg.veh.dt;
  cvc::world_clock clock(cc);

  // The `agents` scene node (host-built geometry; visibility bound to show.agents in the .ari).
  cvc::gl::ariadne::register_scene_node_type(
      "agents", [&](SceneGraph &g, const ari::SceneNode &n, cvc::gl::GraphicsNode *,
                    cvc::gl::ariadne::RealizedScene &, std::vector<std::string> *) {
        std::vector<float> pos(2 * N), head(N), spd(N);
        std::vector<int> mode(N);
        std::vector<std::uint8_t> reached(N);
        world->snapshot(pos.data(), head.data(), spd.data(), mode.data(), reached.data());
        cvc::geometry geom = build_agents(pos, head, N, markerM);
        auto node = std::dynamic_pointer_cast<GeometryNode>(g.addGraphics(n.id, geom));
        if (node) {
          node->setUseSingleColor(true);
          node->setColor(0.95, 0.75, 0.22);
          node->setAmbient(0.35);
          node->setDiffuse(0.85);
        }
        return std::static_pointer_cast<cvc::gl::GraphicsNode>(node);
      });

  bool stepOnce = false, rebuildReq = false;
  cvc::bounding_box sceneBounds;
  rt.on("sim.restart", [&] { rebuildReq = true; });
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
    std::fprintf(stderr, "nav_sim: %s\n", lr.error.c_str());
    return 1;
  }
  std::printf("nav_sim: loaded %s (%d agents)\n", docPath.c_str(), N);
  for (const std::string &w : lr.warnings)
    std::printf("nav_sim:   %s\n", w.c_str());
  std::vector<std::string> initErrs;
  ari::run_init(app, prefix, lr.init_script, &initErrs);
  for (const std::string &e : initErrs)
    std::printf("nav_sim:   %s\n", e.c_str());
  std::vector<std::string> customErrs;
  const bool customsOk = cvc::gl::ariadne::verify_scene_customs(lr, &customErrs);
  for (const std::string &e : customErrs)
    std::printf("nav_sim:   %s\n", e.c_str());
  if (customsOk && lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, prefix, &sw);
    for (const std::string &w : sw)
      std::printf("nav_sim:   %s\n", w.c_str());
    sceneBounds = sg.computeGraphicsBounds();
    if (!sceneBounds.isNull())
      cam.frameBounds(sceneBounds.minx, sceneBounds.miny, sceneBounds.minz, sceneBounds.maxx,
                      sceneBounds.maxy, sceneBounds.maxz);
  }
  rt.set_root(std::move(lr.root));
  auto agentsNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("agents"));

  const std::string kPaused = ari::resolve_bind(prefix, "sim.paused");
  const std::string kSpeed = ari::resolve_bind(prefix, "sim.speed");
  // The live-knob keys (a subset for Cut 1; nav_settings.ari binds them).
  const std::string kRange = ari::resolve_bind(prefix, "nav.knobs.range_m");
  const std::string kSepGain = ari::resolve_bind(prefix, "nav.knobs.sep_gain");
  const std::string kStatArrived = ari::resolve_bind(prefix, "stats.arrived");
  const std::string kStatAgents = ari::resolve_bind(prefix, "stats.agents");
  const std::string kStatTick = ari::resolve_bind(prefix, "stats.tick");
  ari::write<int>(app, kStatAgents, N);

  nav::sim_world::config live = cfg; // the mirror we push into the world each frame

  auto tick = [&](double wall_dt) {
    view.processUIEvents();
    touch.update();
    cam.update(wall_dt);
    rt.drain();
    if (rebuildReq) {
      rebuildReq = false;
      rebuild(N);
      clock.reset();
      if (agentsNode)
        agentsNode->setGeometry(build_agents(std::vector<float>(2 * N), std::vector<float>(N), N,
                                             markerM)); // re-seed (positions refresh next frame)
    }
    // Apply the live knobs (nav_settings.ari sliders) to the running world each frame.
    live.range_m = ari::read_or<double>(app, kRange, cfg.range_m);
    live.sep_gain = static_cast<float>(ari::read_or<double>(app, kSepGain, cfg.sep_gain));
    world->set_live_knobs(live);
    // Advance the sim on the WORLD CLOCK: sim.speed scales world time; paused banks nothing; a Step
    // yields exactly one quantum.
    const bool paused = ari::read_bool_or(app, kPaused, 0) != 0;
    clock.set_scale(ari::read_or<double>(app, kSpeed, 1.0));
    clock.set_mode(paused ? cvc::world_clock::mode::paused : cvc::world_clock::mode::live);
    int steps = clock.advance(wall_dt).steps;
    if (stepOnce) {
      stepOnce = false;
      steps += static_cast<int>(clock.step_once().steps);
    }
    for (int s = 0; s < steps; ++s)
      world->step();
    // Render the agents at their world-metre poses.
    if (agentsNode) {
      std::vector<float> pos(2 * N), head(N), spd(N);
      std::vector<int> mode(N);
      std::vector<std::uint8_t> reached(N);
      world->snapshot(pos.data(), head.data(), spd.data(), mode.data(), reached.data());
      agentsNode->setGeometry(build_agents(pos, head, N, markerM));
      int arrived = 0;
      for (int i = 0; i < N; ++i)
        arrived += reached[i] ? 1 : 0;
      ari::write<int>(app, kStatArrived, arrived);
      ari::write<int>(app, kStatTick, static_cast<int>(world->tick()));
    }
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
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
    std::printf("nav_sim: captured %ld frame(s)%s%s (tick %ld)\n", frames,
                png.empty() ? "" : " -> ", png.c_str(), world ? world->tick() : 0);
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

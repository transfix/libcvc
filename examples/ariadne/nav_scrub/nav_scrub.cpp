// nav_scrub — watch a policy learn, in a window or a browser page: cvc::nav's checkpoint scrubber
// (cvc/nav/nav_scrub.h, docs/NAV_LAB.md) on a C++ Ariadne host. With no history it trains one
// first: cvc::nav's own coef_trainer, a few dozen steps on the scenario's own map in its frame, a
// checkpoint every few steps. Then every checkpoint is deployed on the built-in blocks scenario,
// headless and in this process, and nav_scrub.ari shows checkpoint k beside step 0: the deployed
// metric against the step, the training curve, the table, the Stats and Ledger windows at the play
// head, and the two runs' ghosts. The views are the library's (cvc::nav::scrub_view): this file
// trains, draws the world and the ghosts, and wires the events. pycvc_gl.nav_lab scrub is the
// Python twin.
//
//   nav_scrub nav_scrub.ari --component-path <repo>/src/cvc/ariadne
//   nav_scrub nav_scrub.ari --history run/history.jsonl          # a history of your own
//   nav_scrub nav_scrub.ari --offscreen --frames 60 --png scrub.png --component-path ...
//
// In the gallery (the wasm build) the document and the component library are in the page's file
// system, and the training and the deploys run in the page, so nothing is fetched.

#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
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
#include <cvc/nav/coef_train.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/nav_scrub.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_trace.h>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;
namespace nav = cvc::nav;
namespace fs = std::filesystem;

namespace {

// An axis-aligned box from (x0, y0, z0) to (x1, y1, z1), appended to g.
void add_box(cvc::geometry &g, double x0, double y0, double x1, double y1, double z0, double z1) {
  const unsigned b = static_cast<unsigned>(g.points().size());
  const double xs[2] = {x0, x1}, ys[2] = {y0, y1}, zs[2] = {z0, z1};
  for (int k = 0; k < 8; ++k)
    g.points().push_back({xs[k & 1], ys[(k >> 1) & 1], zs[(k >> 2) & 1]});
  for (int k = 0; k < 8; ++k)
    g.normals().push_back({0, 0, 1});
  const unsigned f[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                             {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
  for (const auto &t : f)
    g.tris().push_back({b + t[0], b + t[1], b + t[2]});
}

// A body (length x width x height) at a pose: its rear axle at (x, y), heading h, wheelbase wb.
void add_body(cvc::geometry &g, const nav::ghost_pose &p, double height) {
  const double c = std::cos(p.heading), s = std::sin(p.heading);
  const double cx = p.x + 0.5 * p.wheelbase * c, cy = p.y + 0.5 * p.wheelbase * s;
  const double hl = 0.5 * p.length, hw = 0.5 * p.width;
  const unsigned b = static_cast<unsigned>(g.points().size());
  for (int k = 0; k < 8; ++k) {
    const double u = (k & 1) ? hl : -hl, v = (k & 2) ? hw : -hw, z = (k & 4) ? height : 0.2;
    g.points().push_back({cx + u * c - v * s, cy + u * s + v * c, z});
    g.normals().push_back({0, 0, 1});
  }
  const unsigned f[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                             {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
  for (const auto &t : f)
    g.tris().push_back({b + t[0], b + t[1], b + t[2]});
}

// The scenario's walls: each grid row's runs of blocked cells, as boxes.
cvc::geometry walls(const nav::sim_world &w, double height) {
  cvc::geometry g;
  const nav::sim_world::config &c = w.current_config();
  const double dx = (c.max_x - c.min_x) / (w.cols() - 1), dy = (c.max_y - c.min_y) / (w.rows() - 1);
  const std::uint8_t *t = w.truth();
  for (int r = 0; r < w.rows(); ++r)
    for (int col = 0; col < w.cols();) {
      if (!t[static_cast<std::size_t>(r) * w.cols() + col]) {
        ++col;
        continue;
      }
      const int c0 = col;
      while (col < w.cols() && t[static_cast<std::size_t>(r) * w.cols() + col])
        ++col;
      add_box(g, c.min_x + (c0 - 0.5) * dx, c.min_y + (r - 0.5) * dy, c.min_x + (col - 0.5) * dx,
              c.min_y + (r + 0.5) * dy, 0.0, height);
    }
  return g;
}

// A short training run of cvc::nav's own trainer on the map it is deployed on (the scenario's
// world, in its frame), its history and checkpoints under `dir`.
std::string train(const std::string &dir, int steps, int every, const nav::sim_world &w) {
  fs::create_directories(dir);
  nav::train_config cfg;
  cfg.n = 32;
  cfg.hidden = 32;
  cfg.window = 4;
  cfg.horizon = 16;
  cfg.steps = steps;
  cfg.lr = 3e-3f;
  cfg.checkpoint_every = every;
  cfg.history_path = dir + "/history.jsonl";
  cfg.checkpoint_prefix = dir + "/coef";
  const nav::sim_world::config &c = w.current_config();
  const nav::training_scene scene =
      nav::occupancy_scene(w.truth(), w.rows(), w.cols(), c.min_x, c.min_y, c.max_x, c.max_y,
                           c.scale, c.veh.rr, c.veh.d_hat, c.veh.dt, c.veh.vmax);
  nav::coef_trainer tr(cfg, 7);
  tr.train(scene);
  return cfg.history_path;
}

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png, history;
  bool offscreen = false;
  long frames = 0;
  int width = 1280, height = 800, agents = 4, steps = 60, every = 6;
  long ticks = 500;
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
      width = std::atoi(next("1280"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("800"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (!std::strcmp(a, "--history"))
      history = next("");
    else if (!std::strcmp(a, "--agents"))
      agents = std::atoi(next("4"));
    else if (!std::strcmp(a, "--ticks"))
      ticks = std::atol(next("500"));
    else if (!std::strcmp(a, "--steps"))
      steps = std::atoi(next("60"));
    else if (!std::strcmp(a, "--every"))
      every = std::atoi(next("6"));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
#ifdef __EMSCRIPTEN__
  if (docPath.empty())
    docPath = "/nav_scrub/nav_scrub.ari"; // preloaded with the page
  if (componentPaths.empty())
    componentPaths.push_back("/"); // cvc://components/... is /components/...
#endif
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 30;
  if (docPath.empty()) {
    std::fprintf(stderr, "usage: nav_scrub <nav_scrub.ari> [--history H] [--agents N] [--ticks T]"
                         " [--offscreen --png P] [--component-path DIR]\n");
    return 2;
  }

  // the scenario, and the checkpoints: a history of the user's, or one trained now on its map
  const nav::scrub_scenario scenario = nav::default_scenario(agents, 0, ticks);
  nav::policy_library lib;
  const nav::sim_world world = nav::world_from_trace(scenario.world, lib);
  if (history.empty()) {
    const std::string dir = (fs::temp_directory_path() / "nav_scrub_run").string();
    std::printf("nav_scrub: training %d steps on the blocks (a checkpoint every %d) into %s\n",
                steps, every, dir.c_str());
    history = train(dir, steps, every, world);
  }
  nav::training_history h = nav::read_training_history(history);
  if (h.checkpoints.empty()) {
    std::fprintf(stderr, "nav_scrub: %s names no checkpoint (a .cvcnav) that is there\n",
                 history.c_str());
    return 2;
  }
  nav::scrub_options so;
  so.say = [](const std::string &line) { std::printf("nav_scrub: %s\n", line.c_str()); };
  std::vector<nav::scrub_entry> entries = nav::scrub(h, scenario, so);
  nav::scrub_view scrub(h, entries, -1, scenario.name + ", " + std::to_string(ticks) + " ticks");

  cvc::app app;
  ari::register_cvc_uri_handler(app, componentPaths);
  SceneGraph sg(app, "navscrub");
  SceneRenderer view(sg, width, height, offscreen, "main");
  view.setFrameYield(SceneRenderer::FrameYield::App);
  view.setBackground(0.70, 0.78, 0.86);
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  const std::string prefix = sg.getStatePrefix();

  rt.on("scrub.prev", [&] { scrub.prev(); });
  rt.on("scrub.next", [&] { scrub.next(); });
  rt.on("scrub.pick_step", [&] { scrub.pick_step(app, prefix); });
  rt.on("sim.step", [&] { scrub.replay().head().step(); });
  rt.on("sim.restart", [&] { scrub.replay().head().restart(app, prefix); });
  rt.on("arms.seek", [&] { scrub.replay().head().seek(app, prefix); });
  rt.on("arms.retarget", [] {});
  rt.on("quit", [] {});

  ari::LoadResult lr = ari::load_file(app, docPath.c_str());
  if (!lr.ok) {
    std::fprintf(stderr, "nav_scrub: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("nav_scrub:   %s\n", w.c_str());
  std::vector<std::string> initErrs;
  ari::run_init(app, prefix, lr.init_script, &initErrs);
  for (const std::string &e : initErrs)
    std::printf("nav_scrub:   %s\n", e.c_str());
  rt.set_root(std::move(lr.root));

  // the world: the ground, the walls, and each arm's ghosts and goal posts
  const nav::sim_world::config &wc = world.current_config();
  cvc::geometry ground;
  add_box(ground, wc.min_x, wc.min_y, wc.max_x, wc.max_y, -0.2, 0.0);
  auto g0 = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("ground", ground));
  g0->setUseSingleColor(true);
  g0->setColor(0.55, 0.60, 0.50);
  auto wn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("walls", walls(world, 2.5)));
  wn->setUseSingleColor(true);
  wn->setColor(0.82, 0.80, 0.76);
  const double slots[2][3] = {{0x2A / 255.0, 0x78 / 255.0, 0xD6 / 255.0},
                              {0xEB / 255.0, 0x68 / 255.0, 0x34 / 255.0}};
  std::shared_ptr<GeometryNode> ghosts[2];
  for (int j = 0; j < 2; ++j) {
    cvc::geometry empty;
    add_box(empty, 0, 0, 0.01, 0.01, 0, 0.01);
    ghosts[j] =
        std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("arm" + std::to_string(j), empty));
    ghosts[j]->setUseSingleColor(true);
    ghosts[j]->setColor(slots[j][0], slots[j][1], slots[j][2]);
    ghosts[j]->setOpacity(0.75);
  }
  cvc::geometry posts;
  std::vector<float> goals(2 * world.size());
  world.goals_world(goals.data());
  for (int i = 0; i < world.size(); ++i)
    add_box(posts, goals[2 * i] - 0.4, goals[2 * i + 1] - 0.4, goals[2 * i] + 0.4,
            goals[2 * i + 1] + 0.4, 0.0, 6.0);
  auto pn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("goals", posts));
  pn->setUseSingleColor(true);
  pn->setColor(0.95, 0.80, 0.10);
  cam.frameBounds(wc.min_x, wc.min_y, 0.0, wc.max_x, wc.max_y, 30.0);

  auto tick = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain();
    const nav::scrub_view::frame f = scrub.update(app, prefix, dt);
    for (int j = 0; j < 2 && j < static_cast<int>(f.replay.poses.size()); ++j) {
      cvc::geometry g;
      for (const nav::ghost_pose &p : f.replay.poses[static_cast<std::size_t>(j)])
        add_body(g, p, 1.6 + 0.4 * j);
      ghosts[j]->setGeometry(g);
    }
    for (const std::string &w : rt.take_reactive_warnings())
      std::fprintf(stderr, "%s\n", w.c_str());
  };

  if (capturing) {
    for (long fr = 0; fr < frames; ++fr) {
      tick(1.0 / 30.0);
      if (fr + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("nav_scrub: captured %ld frame(s)%s%s\n", frames, png.empty() ? "" : " -> ",
                png.c_str());
    return 0;
  }
  const auto t0 = std::chrono::steady_clock::now();
  double last = 0.0;
  while (!view.windowClosed()) {
    const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double dt = now - last;
    last = now;
    tick(dt > 0 ? dt : 1.0 / 120.0);
    view.render();
#ifdef __EMSCRIPTEN__
    emscripten_sleep(0); // yield to the browser (Asyncify)
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(8));
#endif
  }
  cam.detach();
  return 0;
}

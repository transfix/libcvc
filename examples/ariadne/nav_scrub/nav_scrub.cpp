// nav_scrub — watch a policy learn, in a window or a browser page: cvc::nav's checkpoint scrubber
// (cvc/nav/nav_scrub.h, docs/NAV_SCRUB.md) on a C++ Ariadne host. With no history it trains one
// first: cvc::nav's own coef_trainer, a few dozen steps on the scenario's own map in its frame, a
// checkpoint every few steps. Then every checkpoint is deployed on the scenario, headless and in
// this process, on each plant (--plant, more than once: by default the Jolt Humvee, which collides
// with the buildings, and nav's kinematic bicycle, which drives through them, for comparison; the
// Plant combo switches), and nav_scrub.ari shows checkpoint k beside step 0: the deployed metric
// against the step, the training curve, the table, the Stats and Ledger windows at the play head,
// and the two runs in the world. The views are the library's (cvc::nav::scrub_view): this file
// trains, draws the two runs, and wires the events. pycvc_gl.nav_lab scrub is the Python twin.
//
// The world is the document's (its scene's world nodes; --world picks one): south Austin from its
// bundle, a generated world or a flat square, run by the world runtime (cvc::ariadne::SceneWorld),
// whose grid the scenario is on (cvc::nav::grid_scenario: agents 12 m apart on free ground, in
// metres) and whose geometry the Humvee collides with; or the built-in blocks. Each run's vehicles
// are the Humvee at its true size as a world vehicle draws it (in parts where vehicle-humvee has
// them, its wheels rolling by the distance it has driven and its front pair turned by its logged
// steer; else the one-piece model; else a box), posed on the ground; checkpoint k's opaque in
// orange, step 0's translucent in blue, each with a ring under it and its trail behind it. Every
// pose is the run's own: where two cars overlap, they are there in both runs (step 0 and k start
// alike).
//
//   nav_scrub nav_scrub.ari --component-path <repo>/src/cvc/ariadne --bundle <austin_south>
//   nav_scrub nav_scrub.ari --net-allow https://cvcpkg.org        # Austin from cvcpkg.org
//   nav_scrub nav_scrub.ari --world generated                     # or flat, or blocks
//   nav_scrub nav_scrub.ari --history run/history.jsonl          # a history of your own
//   nav_scrub nav_scrub.ari --plant humvee                        # the Humvee alone
//   nav_scrub nav_scrub.ari --offscreen --frames 60 --png scrub.png --component-path ...
//   nav_scrub nav_scrub.ari --no-ui --at 12 ...                    # the world alone at 12 s
//   nav_scrub nav_scrub.ari --look 0,-60,40,0,0,0 ...                # the camera's eye and target
//   nav_scrub nav_scrub.ari --camera follow ...                     # behind checkpoint k's vehicle
//
// Without --bundle the city is a local install's (CVC_NAV_BUNDLE, or share/cvc-scenes/austin_south
// under CVC_PREFIX, CVCPKG_PREFIX or CONDA_PREFIX), else cvcpkg.org's with --net-allow, else the
// generated world. In the gallery (the wasm build) the document, the component library and the
// Austin bundle are in the page's file system (/bundle), and the training and the deploys run in
// the page, so nothing is fetched.

#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/world_vehicles.h>
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
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/nav_scrub.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/run_log.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/nav/world_grid.h>
#include <cvc/net/net_policy.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/scrub_plant.h>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;
namespace gla = cvc::gl::ariadne;
namespace nav = cvc::nav;
namespace fs = std::filesystem;

namespace {

// The two runs' colours (Ariadne chart: slots c0 and c1, as the Stats and Ledger windows have
// them): step 0 blue, checkpoint k orange. Step 0 is drawn translucent, k opaque.
constexpr float kRunColour[2][3] = {{0x2A / 255.0f, 0x78 / 255.0f, 0xD6 / 255.0f},
                                    {0xEB / 255.0f, 0x68 / 255.0f, 0x34 / 255.0f}};
constexpr double kRunOpacity[2] = {0.38, 1.0};
constexpr double kTrailWidth[2] = {0.6, 0.9}; // m
// Each run's ring round its vehicles (m): step 0's outside k's, so where the two runs' vehicles are
// at one pose (both start there) both rings show.
constexpr double kRing[2][2] = {{3.6, 4.2}, {2.8, 3.4}};
constexpr long kTrailEvery = 4; // a trail is redrawn when its run moves this many ticks

// Let the browser run between the steps of a wait (Asyncify), or sleep a little.
void yield_a_moment() {
#ifdef __EMSCRIPTEN__
  emscripten_sleep(0);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
#endif
}

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

// The scenario's walls: each grid row's runs of blocked cells, as boxes (the blocks, which have
// no meshes of their own).
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

// The ground's height at (x, y): bilinear between g's samples, 0 without a grid or heights.
double ground_at(const nav::world_grid *g, double x, double y) {
  if (!g || g->height.empty() || g->rows < 2 || g->cols < 2)
    return 0.0;
  const double cell = g->cell();
  const double u = std::clamp((x - g->min_x) / cell, 0.0, g->cols - 1.0);
  const double v = std::clamp((y - g->min_y) / cell, 0.0, g->rows - 1.0);
  const int c0 = std::min(static_cast<int>(u), g->cols - 2),
            r0 = std::min(static_cast<int>(v), g->rows - 2);
  const double fu = u - c0, fv = v - r0;
  const auto h = [g](int r, int c) {
    return static_cast<double>(g->height[static_cast<std::size_t>(r) * g->cols + c]);
  };
  return (1 - fv) * ((1 - fu) * h(r0, c0) + fu * h(r0, c0 + 1)) +
         fv * ((1 - fu) * h(r0 + 1, c0) + fu * h(r0 + 1, c0 + 1));
}

// A flat strip `width` m wide along `pts`, `lift` m over the ground under each point.
cvc::geometry strip(const std::vector<std::array<double, 2>> &pts, double width,
                    const nav::world_grid *g, double lift) {
  cvc::geometry out;
  if (pts.size() < 2)
    return out;
  for (std::size_t j = 0; j < pts.size(); ++j) {
    const std::array<double, 2> &a = pts[j > 0 ? j - 1 : 0],
                                &b = pts[std::min(j + 1, pts.size() - 1)];
    double dx = b[0] - a[0], dy = b[1] - a[1];
    const double n = std::hypot(dx, dy);
    if (n > 1e-9)
      dx /= n, dy /= n;
    else
      dx = 1.0, dy = 0.0;
    const double z = ground_at(g, pts[j][0], pts[j][1]) + lift, w = 0.5 * width;
    out.points().push_back({pts[j][0] - dy * w, pts[j][1] + dx * w, z});
    out.points().push_back({pts[j][0] + dy * w, pts[j][1] - dx * w, z});
    out.normals().push_back({0, 0, 1});
    out.normals().push_back({0, 0, 1});
    if (j > 0) {
      const unsigned q = static_cast<unsigned>(2 * j);
      out.tris().push_back({q - 2, q - 1, q});
      out.tris().push_back({q - 1, q + 1, q});
    }
  }
  return out;
}

// A ring from r0 to r1 m round the origin, z m up.
cvc::geometry ring(double r0, double r1, double z) {
  cvc::geometry g;
  constexpr int n = 40;
  for (int k = 0; k < n; ++k) {
    const double a = 2.0 * M_PI * k / n, c = std::cos(a), s = std::sin(a);
    g.points().push_back({r0 * c, r0 * s, z});
    g.points().push_back({r1 * c, r1 * s, z});
    g.normals().push_back({0, 0, 1});
    g.normals().push_back({0, 0, 1});
    const unsigned i = static_cast<unsigned>(2 * k), j = static_cast<unsigned>(2 * ((k + 1) % n));
    g.tris().push_back({i, i + 1, j});
    g.tris().push_back({i + 1, j + 1, j});
  }
  return g;
}

// A pose's matrix (row-major): turned `heading` about z, at (x, y, z).
void pose_matrix(double x, double y, double z, double heading, double m[16]) {
  const double c = std::cos(heading), s = std::sin(heading);
  const double v[16] = {c, -s, 0, x, s, c, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
  std::copy(v, v + 16, m);
}

// n and everything under it at opacity `o`.
void set_opacity(GraphicsNode &n, double o) {
  if (auto *g = dynamic_cast<GeometryNode *>(&n))
    g->setOpacity(o);
  for (const std::shared_ptr<GraphicsNode> &c : n.getGraphicsChildren())
    if (c)
      set_opacity(*c, o);
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

// A local Austin bundle (a directory with terrain.json): `flag`, $CVC_NAV_BUNDLE, the page's
// /bundle, or an install's share/cvc-scenes/austin_south. "" when there is none.
std::string local_bundle(const std::string &flag) {
  std::error_code ec;
  const auto is_bundle = [&](const fs::path &p) {
    return !p.empty() && fs::is_regular_file(p / "terrain.json", ec);
  };
  if (!flag.empty())
    return flag;
  std::vector<fs::path> dirs;
  if (const char *b = std::getenv("CVC_NAV_BUNDLE"); b && *b)
    dirs.emplace_back(b);
  dirs.emplace_back("/bundle");
  for (const char *v : {"CVC_PREFIX", "CVCPKG_PREFIX", "CONDA_PREFIX"})
    if (const char *e = std::getenv(v); e && *e)
      dirs.push_back(fs::path(e) / "share" / "cvc-scenes" / "austin_south");
  for (const fs::path &d : dirs)
    if (is_bundle(d))
      return fs::absolute(d, ec).string();
  return "";
}

// The vehicles of one run: each a group (the vehicle's look, about its chassis box's centre) and
// a ring under it, posed each frame; and its trails, redrawn as it moves on.
struct run_cars {
  std::vector<std::string> nodes, rings, trails;
  std::vector<long> trail_tick; // the tick each trail was drawn to (-1: draw it)
  // A parted look's wheels, each car's (none: a one-piece model or a box), and the distance each
  // car has driven by each tick (its logged speed, summed), which rolls them.
  std::vector<std::vector<gla::drawn_wheel>> wheels;
  std::vector<std::vector<double>> driven;
};

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png, history, worldName = "city", bundleFlag, cameraFlag;
  bool offscreen = false, windows = true;
  long frames = 0;
  int width = 1280, height = 800, agents = 4, steps = 60, every = 6;
  unsigned seed = 0;
  long ticks = 600;
  double cell = 2.0, at = 0.0;
  std::vector<std::string> componentPaths, plants, netAllow;
  std::vector<double> look6; // --look ex,ey,ez,tx,ty,tz: the camera's eye and target (m)
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
    else if (!std::strcmp(a, "--seed"))
      seed = static_cast<unsigned>(std::atol(next("0")));
    else if (!std::strcmp(a, "--ticks"))
      ticks = std::atol(next("600"));
    else if (!std::strcmp(a, "--steps"))
      steps = std::atoi(next("60"));
    else if (!std::strcmp(a, "--every"))
      every = std::atoi(next("6"));
    else if (!std::strcmp(a, "--plant"))
      plants.push_back(next("humvee"));
    else if (!std::strcmp(a, "--world"))
      worldName = next("city");
    else if (!std::strcmp(a, "--bundle"))
      bundleFlag = next("");
    else if (!std::strcmp(a, "--net-allow"))
      netAllow.push_back(next(""));
    else if (!std::strcmp(a, "--camera"))
      cameraFlag = next("orbit");
    else if (!std::strcmp(a, "--at"))
      at = std::atof(next("0"));
    else if (!std::strcmp(a, "--look")) {
      std::string v = next("");
      for (char &ch : v)
        if (ch == ',')
          ch = ' ';
      std::istringstream in(v);
      double d;
      while (in >> d)
        look6.push_back(d);
    } else if (!std::strcmp(a, "--no-ui"))
      windows = false;
    else if (!std::strcmp(a, "--cell"))
      cell = std::atof(next("2"));
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
    std::fprintf(stderr, "usage: nav_scrub <nav_scrub.ari> [--world city|generated|flat|blocks]"
                         " [--bundle DIR] [--net-allow ORIGIN] [--history H] [--agents N]"
                         " [--seed S] [--ticks T] [--plant P ...] [--offscreen --png P]"
                         " [--component-path DIR]\n");
    return 2;
  }
  if (worldName != "city" && worldName != "generated" && worldName != "flat" &&
      worldName != "blocks") {
    std::fprintf(stderr, "nav_scrub: --world %s: one of city, generated, flat, blocks\n",
                 worldName.c_str());
    return 2;
  }
  if (plants.empty()) { // the Humvee, and the bicycle to compare (the Humvee alone without Jolt)
    plants.push_back(cvc::physics::scrub_default_plant());
    if (plants.front() != cvc::nav::kScrubKinematic)
      plants.push_back(cvc::nav::kScrubKinematic);
  }
  for (const std::string &p : plants)
    try {
      cvc::physics::plant_host::check(p);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "nav_scrub: --plant %s: %s\n", p.c_str(), e.what());
      return 2;
    }

  cvc::app app;
  ari::register_cvc_uri_handler(app, componentPaths);
  gla::register_world_node_types(app);
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
  const auto say = [](const std::string &line) { std::printf("nav_scrub: %s\n", line.c_str()); };
  if (!cameraFlag.empty()) // the Episode window's Camera: follow or orbit
    nav::panel_set(app, prefix + ".view.camera", cameraFlag);

  // The world: the document's world node for --world, run by the world runtime, or the blocks.
  const std::string bundle = local_bundle(bundleFlag); // the city's
  if (worldName == "city") {
    if (bundle.empty() && netAllow.empty()) {
      say("no Austin bundle here (--bundle DIR, CVC_NAV_BUNDLE, or --net-allow "
          "https://cvcpkg.org to fetch it): the generated world instead");
      worldName = "generated";
    }
  }
  gla::RealizedScene realized;
  std::shared_ptr<const nav::world_grid> grid;
  std::shared_ptr<const cvc::world::world_geometry> geometry;
  std::string worldNode, worldLabel = worldName;
  if (worldName != "blocks") {
    std::vector<ari::SceneNode> kept; // the world on's nodes, and the ones in every world
    for (ari::SceneNode &n : lr.scene.nodes) {
      if (n.world == worldName && worldNode.empty())
        worldNode = n.id;
      if (n.world.empty() || n.world == worldName)
        kept.push_back(std::move(n));
    }
    lr.scene.nodes = std::move(kept);
    lr.scene.world.source = worldName;
    auto store = ari::DataStore::of(app);
    if (!netAllow.empty()) {
      auto policy = std::make_shared<cvc::net::NetPolicy>();
      for (const std::string &o : netAllow) {
        std::string why;
        if (const std::optional<cvc::net::Origin> origin = cvc::net::parse_origin(o, &why))
          policy->allow(*origin);
        else
          say("--net-allow " + o + ": " + why);
      }
      store->set_network(policy);
    }
    const std::string base = fs::absolute(fs::path(docPath)).parent_path().string();
    for (ari::DataSpec d : lr.data) {
      if (d.name == "city" && worldName != "city")
        continue; // Austin is not fetched for another world
      if (d.name == "city" && !bundle.empty()) {
        d.uri = bundle; // the bundle's directory is the package's share/cvc-scenes/austin_south
        d.path.clear();
      }
      d.base = base;
      store->bind(prefix, d);
    }
    std::vector<std::string> customErrs;
    if (!gla::verify_scene_customs(app, lr, &customErrs))
      for (const std::string &e : customErrs)
        say(e);
    std::vector<std::string> sw;
    realized = gla::realize_scene(sg, lr.scene, prefix, &sw);
    for (const std::string &w : sw)
      say("  " + w);
    if (!realized.world || worldNode.empty()) {
      say("the document has no world '" + worldName + "': the blocks instead");
      worldName = "blocks";
    } else {
      say("waiting for the " + worldName + " world (" + worldNode + ")");
      const auto t0 = std::chrono::steady_clock::now();
      std::string last;
      for (;;) {
        ari::sync_scene_visibility(app, realized.visibility);
        gla::tick_scene(realized, view.renderer(), 1.0 / 30.0);
        std::vector<std::string> gw;
        // a generated world is in once its job is over (world.busy 0): its drawing goes in over
        // a few frames after its dataset
        const bool busy =
            worldName == "generated" && nav::panel_get(app, prefix + ".world.busy") == "1";
        grid = busy ? nullptr : realized.world->grid(worldNode, cell, 0.0, &gw);
        if (grid)
          break;
        const std::string status = nav::panel_get(app, prefix + ".data.city.status");
        if (worldName == "city" && status != last && !status.empty())
          say("  Austin: " + (last = status));
        if (worldName == "city" && status == "error") {
          say("  Austin: " + nav::panel_get(app, prefix + ".data.city.error"));
          break;
        }
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 600.0)
          break;
        yield_a_moment();
      }
      if (!grid) {
        say("the " + worldName + " world did not come up: the blocks instead");
        worldName = "blocks";
      } else {
        geometry = realized.world->world_geometry(worldNode);
        if (worldName == "city")
          worldLabel = "austin";
        say(worldLabel + ": a " + std::to_string(grid->cols) + " x " + std::to_string(grid->rows) +
            " grid of " + nav::panel_number(grid->cell(), 1) + " m cells over (" +
            nav::panel_number(grid->min_x, 0) + ", " + nav::panel_number(grid->min_y, 0) + ")-(" +
            nav::panel_number(grid->max_x, 0) + ", " + nav::panel_number(grid->max_y, 0) + "), " +
            std::to_string(grid->occupied_count()) + " samples blocked" +
            (geometry ? "; the plant collides with its geometry" : ""));
      }
    }
  }

  // the scenario, and the checkpoints: a history of the user's, or one trained now on its map
  nav::scrub_scenario scenario = worldName == "blocks"
                                     ? nav::default_scenario(agents, seed, ticks)
                                     : nav::grid_scenario(*grid, agents, seed, ticks, worldLabel);
  nav::policy_library lib;
  const nav::sim_world world = nav::world_from_trace(scenario.world, lib);
  for (std::size_t i = 0; i < scenario.world.agents.size(); ++i) {
    const auto &a = scenario.world.agents[i];
    const nav::sim_world::config &c = world.current_config();
    const double x = a.start[0] / c.scale + c.cx, y = a.start[1] / c.scale + c.cy;
    const double gx = a.goal[0] / c.scale + c.cx, gy = a.goal[1] / c.scale + c.cy;
    say("agent " + std::to_string(i) + ": (" + nav::panel_number(x, 1) + ", " +
        nav::panel_number(y, 1) + ", ground " + nav::panel_number(ground_at(grid.get(), x, y), 1) +
        ") to (" + nav::panel_number(gx, 1) + ", " + nav::panel_number(gy, 1) + "), " +
        nav::panel_number(std::hypot(gx - x, gy - y), 1) + " m");
  }
  if (history.empty()) {
    const std::string dir =
        (fs::temp_directory_path() / "nav_scrub_run" / (worldLabel + "-" + std::to_string(seed)))
            .string();
    std::printf("nav_scrub: training %d steps on %s (a checkpoint every %d) into %s\n", steps,
                worldLabel.c_str(), every, dir.c_str());
    history = train(dir, steps, every, world);
  }
  nav::training_history h = nav::read_training_history(history);
  if (h.checkpoints.empty()) {
    std::fprintf(stderr, "nav_scrub: %s names no checkpoint (a .cvcnav) that is there\n",
                 history.c_str());
    return 2;
  }
  nav::scrub_options so;
  so.say = say;
  so.plant = geometry ? cvc::physics::scrub_plant(geometry) : cvc::physics::scrub_plant();
  std::vector<nav::scrub_plant_runs> runs;
  for (const std::string &p : plants) {
    scenario.plant = p;
    runs.push_back({p, nav::scrub(h, scenario, so)});
  }
  nav::scrub_view scrub(h, std::move(runs), -1,
                        scenario.name + ", " + std::to_string(ticks) + " ticks");

  rt.on("scrub.prev", [&] { scrub.prev(); });
  rt.on("scrub.next", [&] { scrub.next(); });
  rt.on("scrub.pick_step", [&] { scrub.pick_step(app, prefix); });
  rt.on("sim.step", [&] { scrub.replay().head().step(); });
  rt.on("sim.restart", [&] { scrub.replay().head().restart(app, prefix); });
  rt.on("arms.seek", [&] { scrub.replay().head().seek(app, prefix); });
  rt.on("arms.retarget", [] {});
  rt.on("quit", [] {});
  if (windows)
    rt.set_root(std::move(lr.root));

  // The blocks' ground and walls, which have no meshes of their own.
  const nav::sim_world::config &wc = world.current_config();
  if (worldName == "blocks") {
    cvc::geometry ground;
    add_box(ground, wc.min_x, wc.min_y, wc.max_x, wc.max_y, -0.2, 0.0);
    auto g0 = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("ground", ground));
    g0->setUseSingleColor(true);
    g0->setColor(0.55, 0.60, 0.50);
    auto wn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("walls", walls(world, 2.5)));
    wn->setUseSingleColor(true);
    wn->setColor(0.82, 0.80, 0.76);
  }
  const nav::world_grid *ground = grid.get();

  // Each goal a post on the ground.
  std::vector<float> goals(2 * world.size());
  world.goals_world(goals.data());
  {
    cvc::geometry posts;
    for (int i = 0; i < world.size(); ++i) {
      const double gx = goals[2 * i], gy = goals[2 * i + 1], gz = ground_at(ground, gx, gy);
      add_box(posts, gx - 0.6, gy - 0.6, gx + 0.6, gy + 0.6, gz, gz + 10.0);
    }
    auto pn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("goals", posts));
    pn->setUseSingleColor(true);
    pn->setColor(0.95, 0.80, 0.10);
  }

  // The runs' vehicles: the Humvee at its true size, as a world vehicle's default look is
  // (vehicle_look_named: in parts where they are installed, else Humvee.glb, else a box); each over
  // a ring of its run's colour.
  gla::vehicle_shape shape; // a Humvee's chassis box, as the catalog publishes it
  double stand = 0.9;       // its chassis box's centre over the ground
  try {
    stand = ari::stand_height(cvc::physics::plant_host::preset("humvee"));
  } catch (const std::exception &) {
  }
  const ari::vehicle_look look = ari::vehicle_look_named("humvee");
  say("the vehicles: " + (look.parted()         ? "the Humvee in parts, " + look.chassis
                          : look.kind == "file" ? look.path
                                                : std::string("boxes (no Humvee.glb)")));
  std::shared_ptr<gla::vehicle_model_cache> models = gla::make_vehicle_model_cache();
  run_cars cars[2];
  std::vector<gla::drawn_wheel> built_wheels; // the car build_car last drew's
  const auto build_car = [&](int j, int i) {
    const std::string node = "run" + std::to_string(j) + "_car" + std::to_string(i);
    const float dark[3] = {0.5f * kRunColour[j][0], 0.5f * kRunColour[j][1],
                           0.5f * kRunColour[j][2]};
    std::vector<std::string> w;
    const auto realize_look = [&](const ari::vehicle_look &l) {
      ari::Scene one;
      one.nodes.push_back(gla::vehicle_scene_node(node, l, shape, kRunColour[j], dark));
      gla::realize_scene(sg, one, prefix, &w);
    };
    realize_look(look);
    bool model = false;
    built_wheels.clear();
    if (look.kind == "file") {
      std::shared_ptr<GraphicsNode> group = sg.getGraphics(node);
      model = group &&
              (look.parted() ? gla::attach_parted_model(app, *models, *group, node, look, shape,
                                                        -stand, kRunColour[j], &built_wheels, &w)
                             : gla::attach_vehicle_model(app, *models, *group, node + "_mesh", look,
                                                         shape, -stand, kRunColour[j], &w));
      if (!model) {
        sg.removeGraphics(node);
        ari::vehicle_look box;
        box.kind = "box";
        realize_look(box);
      }
    }
    for (const std::string &m : w)
      say("  " + m);
    if (std::shared_ptr<GraphicsNode> group = sg.getGraphics(node)) {
      set_opacity(*group, kRunOpacity[j]);
      group->setCastsShadow(j == 1);
    }
    return node;
  };
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < world.size(); ++i) {
      const std::string node = build_car(j, i);
      cars[j].nodes.push_back(node);
      cars[j].wheels.push_back(built_wheels);
      const std::string r = node + "_ring";
      auto rn = std::dynamic_pointer_cast<GeometryNode>(
          sg.addGraphics(r, ring(kRing[j][0], kRing[j][1], 0.12)));
      rn->setUseSingleColor(true);
      rn->setColor(kRunColour[j][0], kRunColour[j][1], kRunColour[j][2]);
      rn->setOpacity(j == 1 ? 1.0 : 0.6);
      rn->setCastsShadow(false);
      cars[j].rings.push_back(r);
      const std::string t = node + "_trail";
      cars[j].trails.push_back(t);
      cars[j].trail_tick.push_back(-1);
    }

  // The camera: the overview, on the agents from the south-south-west, 55 degrees up, their
  // starts and goals and 25 m round them in view (or --look's); or, with the Episode window's
  // Camera on Follow the arm (view.camera "follow"), behind the Stats window's Follow arm
  // (view.arm; checkpoint k's by default), its vehicle the Ledger shows (ledger.vehicle).
  const auto overview = [&] {
    cam.setMode(CameraController::Mode::Orbit);
    if (look6.size() == 6) {
      cam.lookAt(&look6[0], &look6[3]);
      return;
    }
    std::vector<float> starts(2 * world.size()); // the normalized starts, in metres
    const float *on = world.positions_normalized();
    for (int i = 0; i < 2 * world.size(); ++i)
      starts[i] = static_cast<float>(on[i] / wc.scale + (i % 2 ? wc.cy : wc.cx));
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    for (int i = 0; i < world.size(); ++i)
      for (const float *p : {&starts[2 * i], &goals[2 * i]}) {
        x0 = std::min(x0, double(p[0])), x1 = std::max(x1, double(p[0]));
        y0 = std::min(y0, double(p[1])), y1 = std::max(y1, double(p[1]));
      }
    const double target[3] = {0.5 * (x0 + x1), 0.5 * (y0 + y1),
                              ground_at(ground, 0.5 * (x0 + x1), 0.5 * (y0 + y1))};
    const double span = std::max(x1 - x0, y1 - y0) + 50.0;
    const double d = 0.62 * span / std::tan(15.0 * M_PI / 180.0); // a 30 degree view
    const double el = 55.0 * M_PI / 180.0, az = -110.0 * M_PI / 180.0;
    const double eye[3] = {target[0] + d * std::cos(el) * std::cos(az),
                           target[1] + d * std::cos(el) * std::sin(az),
                           target[2] + d * std::sin(el)};
    cam.frameBounds(x0 - 25.0, y0 - 25.0, target[2], x1 + 25.0, y1 + 25.0, target[2] + 30.0);
    cam.lookAt(eye, target);
  };
  overview();
  std::string cameraMode = "orbit";

  const nav::run_log *shownLog = nullptr;
  auto tick = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain();
    if (realized.world) { // the world on drawn (its nodes' visible: follow it), its data drawn
      ari::sync_scene_visibility(app, realized.visibility);
      gla::tick_scene(realized, view.renderer(), dt);
    }
    if (const std::string mode = nav::panel_get(app, prefix + ".view.camera", "orbit");
        mode != cameraMode) {
      cameraMode = mode;
      if (mode == "follow") { // high behind the vehicle, looking down at it over the street
        const std::string t = prefix + ".viewers.main.camera.track.";
        for (const auto &[k, v] : {std::pair<const char *, double>{"back", 5.0 * shape.length},
                                   {"height", 7.0 * shape.length},
                                   {"look_ahead", shape.length},
                                   {"look_up", 0.0},
                                   {"cam_tau", 0.15},
                                   {"pos_tau", 0.1}})
          nav::panel_set(app, t + k, nav::panel_number(v, 3));
        cam.setMode(CameraController::Mode::Track);
        cam.resetTracking();
      } else {
        overview();
      }
    }
    nav::scrub_view::frame f = scrub.update(app, prefix, dt);
    if (at > 0.0) { // --at: the play head there, once the first frame has made it
      scrub.replay().head().set(at);
      at = 0.0;
      f = scrub.update(app, prefix, 0.0);
    }
    const nav::run_log &log = scrub.replay().log();
    if (&log != shownLog) { // another checkpoint or plant: every trail again
      shownLog = &log;
      for (int j = 0; j < 2; ++j) {
        run_cars &c = cars[j];
        std::fill(c.trail_tick.begin(), c.trail_tick.end(), -1);
        c.driven.assign(c.nodes.size(), {});
        for (std::size_t i = 0; i < c.nodes.size(); ++i)
          if (!c.wheels[i].empty() && j < log.arms() && static_cast<int>(i) < log.agents())
            c.driven[i] = log.cumulative(j, static_cast<int>(i), "speed");
      }
    }
    // the follow camera's vehicle: the Stats window's Follow arm (checkpoint k's by default), the
    // Ledger's agent
    const int followArm = cameraMode == "follow"
                              ? nav::arm_index(log, nav::panel_get(app, prefix + ".view.arm"), 1)
                              : -1;
    const int followAgent = std::atoi(nav::panel_get(app, prefix + ".ledger.vehicle", "0").c_str());
    for (int j = 0; j < 2 && j < static_cast<int>(f.replay.poses.size()); ++j) {
      const std::vector<nav::ghost_pose> &poses = f.replay.poses[static_cast<std::size_t>(j)];
      for (std::size_t i = 0; i < poses.size() && i < cars[j].nodes.size(); ++i) {
        const nav::ghost_pose &p = poses[i];
        const double c = std::cos(p.heading), s = std::sin(p.heading);
        const double wb = p.wheelbase > 0.0 ? p.wheelbase : shape.wheelbase;
        const double cx = p.x + 0.5 * wb * c, cy = p.y + 0.5 * wb * s; // the axles' middle
        const double gz = ground_at(ground, cx, cy);
        if (j == followArm && static_cast<int>(i) == followAgent)
          cam.feedTrackTarget(cx, cy, gz);
        double m[16];
        pose_matrix(cx, cy, gz + stand, p.heading, m);
        if (std::shared_ptr<GraphicsNode> g = sg.getGraphics(cars[j].nodes[i]))
          g->setPoseMatrix(m);
        // A parted look's wheels: rolled by the distance driven to the play head over their
        // radius, the front pair turned by the logged steer (parted_wheel_poses, as a world
        // vehicle's).
        if (const std::vector<gla::drawn_wheel> &dw = cars[j].wheels[i]; !dw.empty()) {
          const std::vector<double> &sum = cars[j].driven[i];
          const std::vector<double> &ts = log.times();
          double metres = 0.0, steer = 0.0;
          if (!sum.empty() &&
              sum.size() == ts.size()) { // between the ticks either side, as its pose
            const long k = std::clamp<long>(f.replay.k, 0, static_cast<long>(sum.size()) - 1);
            const auto k0 = static_cast<std::size_t>(k), k1 = std::min(k0 + 1, sum.size() - 1);
            const double u =
                k1 > k0 ? std::clamp((f.replay.t - ts[k0]) / (ts[k1] - ts[k0]), 0.0, 1.0) : 0.0;
            metres = (sum[k0] + u * (sum[k1] - sum[k0])) * log.dt();
            steer = log.at(k, j, static_cast<int>(i), "tel.steer");
          }
          const double r = shape.wheel_radius > 0.0 ? shape.wheel_radius : 0.45;
          std::vector<ari::entity_wheel> at(dw.size());
          for (std::size_t q = 0; q < dw.size(); ++q) {
            at[q].x = dw[q].x;
            at[q].y = dw[q].y;
            at[q].z = dw[q].rest_z;
            at[q].steered = dw[q].steers;
            at[q].steer = dw[q].steers && std::isfinite(steer) ? steer : 0.0;
            at[q].spin = std::isfinite(metres) ? std::remainder(metres / r, 2.0 * M_PI) : 0.0;
            at[q].radius = r;
          }
          const std::vector<std::array<double, 16>> wm = gla::parted_wheel_poses(dw, at);
          for (std::size_t q = 0; q < wm.size(); ++q)
            if (const std::shared_ptr<GraphicsNode> w = dw[q].node.lock())
              w->setPoseMatrix(wm[q].data());
        }
        pose_matrix(cx, cy, gz, p.heading, m);
        if (std::shared_ptr<GraphicsNode> g = sg.getGraphics(cars[j].rings[i]))
          g->setPoseMatrix(m);
        // its trail, up to the play head, redrawn every few ticks it moves on (or back)
        long &drawn = cars[j].trail_tick[i];
        if (drawn >= 0 && f.replay.k >= drawn && f.replay.k - drawn < kTrailEvery)
          continue;
        drawn = f.replay.k;
        std::vector<std::array<double, 2>> pts; // a point each 0.25 m it moved (none standing)
        const auto add = [&pts](double x, double y) {
          if (pts.empty() || std::hypot(x - pts.back()[0], y - pts.back()[1]) >= 0.25)
            pts.push_back({x, y});
        };
        for (long k = 0; k <= f.replay.k && k < log.ticks(); ++k)
          add(log.at(k, j, static_cast<int>(i), "x"), log.at(k, j, static_cast<int>(i), "y"));
        add(p.x, p.y);
        if (sg.getGraphics(cars[j].trails[i]))
          sg.removeGraphics(cars[j].trails[i]);
        if (pts.size() >= 2) {
          auto tn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(
              cars[j].trails[i], strip(pts, kTrailWidth[j], ground, 0.15 + 0.05 * j)));
          tn->setUseSingleColor(true);
          tn->setColor(kRunColour[j][0], kRunColour[j][1], kRunColour[j][2]);
          tn->setOpacity(j == 1 ? 1.0 : 0.6);
          tn->setCastsShadow(false);
        }
      }
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

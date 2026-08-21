// nav_finale — the GRL-SNAM "finale" in 3-D on the real Austin bundle: 8 vehicles that
// hold the KNOWN city map (freeze_sense — no fog in the finale) drive from the west edge
// to a rendezvous staging line (Act 1), then split into pursuit packs chasing 4 orbiting
// targets (Act 2) — all on cvc::nav's sim_world, no Python, no libtorch. This is the
// "global A* spine + learned local control" architecture on a real city: grid_nav::astar
// (string-pulled via simplify) over a lightly-inflated occupancy gives each vehicle a
// coarse route; between waypoints the SAME reactive stack drives — sample phi -> coef_mlp
// (alpha,beta,gamma) -> carrot bicycle — supplying the true wall clearance the coarse A*
// deliberately omits. retarget() fires ONLY when a waypoint index changes (retargeting
// every frame would reset the carrot FSM's escape state and the vehicle would never move).
// map_id[i]=i gives per-vehicle belief planes, but with no sensing they stay identical
// copies of the known map (structure for future fog, behaviourally inert here). The city
// occupancy is rasterized in C++ straight from the bundle's buildings.glb
// (occupancy_from_model, the C++ building_occupancy). A 2-D picture-in-picture minimap in
// the corner shows every agent's position and the line to the target it is chasing.
//
// The bundle (terrain.json + buildings.glb) is passed at runtime and never vendored:
//   nav_finale --bundle /path/to/scenes/austin_south --capture fly --offscreen \
//              --frames 600 --out /tmp/finale
//   ffmpeg -framerate 30 -i /tmp/finale/frame_%05d.png -c:v libx264 -pix_fmt yuv420p finale.mp4
// With no --bundle it falls back to a synthetic city so the scenario still runs.

#include "nav_common.h"

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/image/image.h>
#include <cvc/model/model_file_io.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/grid_nav.h> // astar / simplify / inflate
#include <cvc/nav/sim_world.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::CameraController;

namespace {
const double PI = std::acos(-1.0);

// Scan terrain.json for the world bounds (min_x/min_y/max_x/max_y). Tiny targeted
// parse — we only need the 4 numbers, not the height grid.
bool read_bounds(const std::string &path, navdemo::Bounds &b) {
  std::ifstream f(path);
  if (!f)
    return false;
  const std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  auto num = [&](const char *key, double &v) -> bool {
    const auto k = s.find(key);
    if (k == std::string::npos)
      return false;
    const auto colon = s.find(':', k);
    if (colon == std::string::npos)
      return false;
    v = std::atof(s.c_str() + colon + 1);
    return true;
  };
  return num("\"min_x\"", b.min_x) && num("\"min_y\"", b.min_y) && num("\"max_x\"", b.max_x) &&
         num("\"max_y\"", b.max_y);
}

// Load a raw uint8 .npy (Python's cached occupancy) for IoU validation; returns
// empty on any mismatch. Assumes '|u1' C-order, square.
std::vector<std::uint8_t> load_npy_u8(const std::string &path, int &side) {
  side = 0;
  std::ifstream f(path, std::ios::binary);
  if (!f)
    return {};
  char magic[6] = {0};
  f.read(magic, 6);
  if (std::string(magic, 6) != std::string("\x93NUMPY", 6))
    return {};
  f.seekg(8);
  unsigned char lo = f.get(), hi = f.get();
  const int hlen = lo | (hi << 8);
  f.seekg(10 + hlen);
  std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(f)),
                                 std::istreambuf_iterator<char>());
  const int s = static_cast<int>(std::lround(std::sqrt(static_cast<double>(data.size()))));
  if (static_cast<std::size_t>(s) * s != data.size())
    return {};
  side = s;
  return data;
}

// Load a mesh (e.g. Humvee.glb) as a canonical vehicle instance template: length ->
// +x (forward), width -> +y, height -> +z, centred in XY and resting on z=0, scaled
// so the longest side is `target_len` metres. Robust to Y-up vs Z-up source (the
// height axis is taken as the smallest extent). false if it can't be read.
bool load_vehicle_template(const std::string &path, double target_len, std::vector<double> &verts,
                           std::vector<std::uint32_t> &tris) {
  if (!std::filesystem::exists(path))
    return false;
  cvc::model m = cvc::read_model(path);
  cvc::geometry g = m.merged();
  if (g.num_tris() == 0)
    return false;
  const auto &pts = g.points();
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &v : pts)
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], v[k]);
      hi[k] = std::max(hi[k], v[k]);
    }
  const double ext[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
  int up = 0; // height = smallest extent
  for (int k = 1; k < 3; ++k)
    if (ext[k] < ext[up])
      up = k;
  int fwd = (up + 1) % 3, side = (up + 2) % 3; // forward = longer of the remaining two
  if (ext[side] > ext[fwd])
    std::swap(fwd, side);
  const double s = ext[fwd] > 1e-9 ? target_len / ext[fwd] : 1.0;
  const double cf = 0.5 * (lo[fwd] + hi[fwd]), cs = 0.5 * (lo[side] + hi[side]);
  verts.clear();
  verts.reserve(pts.size() * 3);
  for (const auto &v : pts) {
    verts.push_back((v[fwd] - cf) * s);    // -> +x forward
    verts.push_back((v[side] - cs) * s);   // -> +y side
    verts.push_back((v[up] - lo[up]) * s); // -> +z up, resting on 0
  }
  tris.clear();
  for (const auto &t : g.tris())
    for (int k = 0; k < 3; ++k)
      tris.push_back(static_cast<std::uint32_t>(t[k]));
  return true;
}

// One agent's A* route: world waypoints + a cursor.
struct Route {
  std::vector<std::array<double, 2>> wp;
  std::size_t idx = 0;
};

// Plan an A* route (string-pulled) over `planOcc` from world (sx,sy) to (gx,gy),
// returned as world waypoints ending at the true goal. Straight-to-goal if blocked.
Route plan_route(const std::vector<std::uint8_t> &planOcc, int rows, int cols,
                 const navdemo::Bounds &b, double sx, double sy, double gx, double gy) {
  auto w2c = [&](double x, double y, int &r, int &c) {
    c = static_cast<int>(std::lround((x - b.min_x) / (b.max_x - b.min_x) * (cols - 1)));
    r = static_cast<int>(std::lround((y - b.min_y) / (b.max_y - b.min_y) * (rows - 1)));
    c = std::max(0, std::min(cols - 1, c));
    r = std::max(0, std::min(rows - 1, r));
  };
  int sr, sc, gr, gc;
  w2c(sx, sy, sr, sc);
  w2c(gx, gy, gr, gc);
  Route rt;
  const auto path = cvc::nav::astar(planOcc.data(), rows, cols, sr, sc, gr, gc);
  if (path.size() >= 4) {
    const auto sp = cvc::nav::simplify(planOcc.data(), rows, cols, path.data(),
                                       static_cast<int>(path.size() / 2));
    for (std::size_t i = 0; i + 1 < sp.size(); i += 2)
      rt.wp.push_back({b.min_x + static_cast<double>(sp[i + 1]) / (cols - 1) * (b.max_x - b.min_x),
                       b.min_y + static_cast<double>(sp[i]) / (rows - 1) * (b.max_y - b.min_y)});
  }
  rt.wp.push_back({gx, gy}); // always end at the true goal
  return rt;
}
// A small downward-pointing pyramid (apex at the origin, square base up at +h)
// — the pursuit-target marker. A silhouette deliberately distinct from every
// vehicle glyph, so a hovering target can't be mistaken for a ninth agent.
cvc::geometry target_marker(double half, double h, const double rgb[3]) {
  cvc::geometry g;
  auto add = [&](double x, double y, double z) {
    g.points().push_back({x, y, z});
    g.colors().push_back({rgb[0], rgb[1], rgb[2]});
  };
  add(0, 0, 0); // apex, pointing down at the street
  add(-half, -half, h);
  add(half, -half, h);
  add(half, half, h);
  add(-half, half, h);
  using I = cvc::geometry::index_t;
  auto tri = [&](int a, int b, int c) {
    g.tris().push_back({static_cast<I>(a), static_cast<I>(b), static_cast<I>(c)});
  };
  tri(0, 1, 2);
  tri(0, 2, 3);
  tri(0, 3, 4);
  tri(0, 4, 1);
  tri(1, 3, 2); // base cap
  tri(1, 4, 3);
  return g;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  std::string bundle, vehicle, capture = "fly", out = "frames", png;
  int width = 1280, height = 720, nx = 384;
  long frames = 0;
  double fps = 30.0, hz = 60.0;
  bool offscreen = false, no_shadows = false, no_minimap = false;

  po::options_description desc("nav_finale — the 8-vehicle Austin finale in cvcGL");
  desc.add_options()("help,h",
                     "show this help")("bundle", po::value<std::string>(&bundle),
                                       "Austin bundle dir (terrain.json + buildings.glb)")(
      "grid", po::value<int>(&nx)->default_value(384), "occupancy resolution")(
      "vehicle", po::value<std::string>(&vehicle),
      "vehicle model .glb (default: <bundle>/../../shared/Humvee.glb; else an arrow)")(
      "offscreen", po::bool_switch(&offscreen))("no-shadows", po::bool_switch(&no_shadows))(
      "no-minimap", po::bool_switch(&no_minimap),
      "hide the 2-D PiP minimap")("frames", po::value<long>(&frames)->default_value(0))(
      "fps", po::value<double>(&fps)->default_value(30.0))(
      "hz", po::value<double>(&hz)->default_value(60.0))(
      "capture", po::value<std::string>(&capture)->default_value("none"),
      "none (interactive window) | fly | orbit (offscreen PNG capture)")(
      "width", po::value<int>(&width)->default_value(1280))(
      "height", po::value<int>(&height)->default_value(720))(
      "out", po::value<std::string>(&out)->default_value("frames"))("png",
                                                                    po::value<std::string>(&png));
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  const bool capturing = (capture != "none");
  if (capturing) {
    offscreen = true;
    if (frames <= 0)
      frames = 600; // a capture must end — 0 would render offscreen forever, looking hung
    std::filesystem::create_directories(out);
    std::printf("nav_finale: capturing %ld frames (%s, offscreen) -> %s/frame_*.png\n", frames,
                capture.c_str(), out.c_str());
  }
  const bool minimap = !no_minimap;

  // 1. Occupancy + a mesh to render. Real bundle if given; else a synthetic city.
  navdemo::Bounds bounds;
  int ny = nx;
  std::vector<std::uint8_t> occ;
  cvc::geometry cityMesh;
  bool haveMesh = false;
  bool usedBundle = false;

  if (!bundle.empty()) {
    const std::string tj = bundle + "/terrain.json", glb = bundle + "/buildings.glb";
    if (read_bounds(tj, bounds) && std::filesystem::exists(glb)) {
      std::printf("nav_finale: loading bundle %s  bounds [%.0f,%.0f]..[%.0f,%.0f]\n",
                  bundle.c_str(), bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
      cvc::model m = cvc::read_model(glb);
      cityMesh = m.merged();
      haveMesh = cityMesh.num_tris() > 0;
      {
        const auto &p = cityMesh.points();
        double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
        for (const auto &v : p)
          for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], v[k]);
            hi[k] = std::max(hi[k], v[k]);
          }
        std::printf("nav_finale: buildings.glb -> %llu meshes, %llu tris, bbox "
                    "x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]\n",
                    (unsigned long long)m.num_meshes(), (unsigned long long)cityMesh.num_tris(),
                    lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
      }
      // Raw building footprints for the sim (the reactive d_hat gives clearance); the
      // A* planOcc below adds a light inflation of its own.
      occ = navdemo::occupancy_from_model(cityMesh, bounds, nx, ny, /*inflate=*/0);
      usedBundle = true;
      // Validate against Python's cached occupancy if present (IoU).
      char npy[1024];
      std::snprintf(npy, sizeof npy, "%s/buildings.glb.occ_%dx%d_i0.npy", bundle.c_str(), nx, ny);
      int side = 0;
      auto ref = load_npy_u8(npy, side);
      if (side == nx) {
        long inter = 0, uni = 0;
        for (std::size_t i = 0; i < occ.size(); ++i) {
          const bool a = occ[i] != 0, b = ref[i] != 0;
          inter += (a && b);
          uni += (a || b);
        }
        // A rough cross-check only: our grid-node (n-1) sampling (which matches
        // sim_world's cell convention, so the occupancy aligns with the rendered
        // mesh) differs by a sub-cell shift from Python's pixel-render occupancy.
        std::printf("nav_finale: occupancy cross-check IoU vs Python cache = %.2f (sampling-"
                    "convention diff; internally consistent with the mesh)\n",
                    uni ? double(inter) / uni : 0.0);
      }
    } else {
      std::printf("nav_finale: bundle at %s incomplete; falling back to synthetic city\n",
                  bundle.c_str());
    }
  }
  if (occ.empty()) { // synthetic fallback
    cvc::nav::training_scene ts = cvc::nav::city_scene(std::min(nx, 128));
    nx = ts.cols;
    ny = ts.rows;
    occ = ts.occ;
    bounds = {ts.min_x, ts.min_y, ts.max_x, ts.max_y};
  }
  navdemo::add_border(occ.data(), ny, nx);
  const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);

  // 2. Eight vehicles. Act 1: enter from the west edge (spread in y), rendezvous on
  //    an eastern staging line. Act 2: pursue 4 moving targets (2 chasers each).
  const int N = 8, NT = 4;
  // Use the SAME world->normalized scale as the working city_scene demo (0.05, i.e.
  // 1 normalized unit = 20 m) so the vehicle's internal constants (wheelbase L,
  // a_max, ...) — which are normalized and tuned at that scale — stay physical. A
  // custom scale leaves those constants mis-sized and the drive governor clamps to 0.
  const double sc = 0.05;
  auto norm = [&](double wx, double wy, float *o2) {
    o2[0] = static_cast<float>((wx - 0.5 * (bounds.min_x + bounds.max_x)) * sc);
    o2[1] = static_cast<float>((wy - 0.5 * (bounds.min_y + bounds.max_y)) * sc);
  };
  const double westX = bounds.min_x + 0.10 * (bounds.max_x - bounds.min_x);
  const double stageX = bounds.min_x + 0.72 * (bounds.max_x - bounds.min_x);
  std::vector<float> o(2 * N), goal(2 * N), color(3 * N);
  for (int i = 0; i < N; ++i) {
    const double y = bounds.min_y + (0.22 + 0.56 * i / (N - 1)) * (bounds.max_y - bounds.min_y);
    norm(westX, y, &o[2 * i]);
    norm(stageX, y, &goal[2 * i]);
    const double hue = static_cast<double>(i) / N;
    color[3 * i] = static_cast<float>(0.5 + 0.5 * std::cos(2 * PI * hue));
    color[3 * i + 1] = static_cast<float>(0.5 + 0.5 * std::cos(2 * PI * hue + 2.1));
    color[3 * i + 2] = static_cast<float>(0.5 + 0.5 * std::cos(2 * PI * hue + 4.2));
  }

  cvc::nav::sim_world::config cfg;
  cfg.rows = ny;
  cfg.cols = nx;
  cfg.min_x = bounds.min_x;
  cfg.min_y = bounds.min_y;
  cfg.max_x = bounds.max_x;
  cfg.max_y = bounds.max_y;
  cfg.cx = 0.5 * (bounds.min_x + bounds.max_x);
  cfg.cy = 0.5 * (bounds.min_y + bounds.max_y);
  cfg.scale = sc;
  // The city_scene demo's proven vehicle params (normalized; at scale 0.05 that is
  // rr ~3 m, barrier ~7 m, arrive ~16 m). reach_tol (0.8 = 16 m) stays well under the
  // route-follower's waypoint-advance radius (arr = 0.03*span m) so a vehicle never
  // parks at an intermediate waypoint.
  cfg.veh.rr = 0.15f;
  cfg.veh.d_hat = 0.35f;
  cfg.veh.dt = 0.06f;
  cfg.veh.vmax = 0.9f;
  cfg.freeze_sense = true; // the vehicles know the city map (prior == truth)
  cfg.range_m = 200.0;
  cfg.n_rays = 200;
  cfg.reach_tol = 0.8f;

  std::vector<int> map_id(N);
  for (int i = 0; i < N; ++i)
    map_id[i] = i; // private belief per vehicle
  cvc::nav::sim_world world(cfg, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(),
                            o.data(), goal.data(), color.data(), N, map_id.data(), N);

  // 3. Scene.
  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "finale");

  if (haveMesh) {
    auto b = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("buildings", cityMesh));
    if (b) {
      b->setUseSingleColor(true);
      b->setColor(0.62, 0.62, 0.66);
      b->setAmbient(0.4);
      b->setDiffuse(0.8);
    }
  } else {
    const double wall_rgb[3] = {0.58, 0.58, 0.64};
    sg.addGraphics("buildings",
                   navdemo::occupancy_to_walls(occ.data(), ny, nx, bounds, 0.05 * span, wall_rgb));
  }
  const double ground_rgb[3] = {0.18, 0.21, 0.25};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", navdemo::ground_quad(bounds, 0.0, ground_rgb)));
  // Drape the bundle's real satellite aerial over the ground (the ground_quad's UVs
  // map [min,max] -> [0,1]; setTexture handles the top-left/bottom-left V flip).
  cvc::image sat;
  bool haveSat = false;
  if (usedBundle) {
    const std::string satpath = bundle + "/satellite.png";
    if (std::filesystem::exists(satpath)) {
      sat = cvc::read_image(satpath);
      haveSat = sat.width() > 0 && sat.height() > 0;
      if (haveSat)
        std::printf("nav_finale: satellite %dx%d draped on the ground\n", sat.width(),
                    sat.height());
    }
  }
  if (groundNode) {
    if (haveSat) {
      groundNode->setUseSingleColor(true);
      groundNode->setColor(1.0, 1.0, 1.0);
      groundNode->setAmbient(0.85); // let the imagery read; keep it flat + bright
      groundNode->setDiffuse(0.5);
      groundNode->setTexture(sat, /*zeroCopy=*/false);
    } else {
      groundNode->setAmbient(0.6);
      groundNode->setDiffuse(0.5);
    }
  }

  navdemo::AgentGlyphs glyphs;
  cvc::geometry agentGeom;
  std::vector<double> hv;
  std::vector<std::uint32_t> hvt;
  std::string vpath = vehicle;
  if (vpath.empty() && usedBundle)
    vpath = bundle + "/../../shared/Humvee.glb"; // the bundle's sibling shared/ dir
  const bool haveVehicle =
      !vpath.empty() && load_vehicle_template(vpath, 0.02 * span, hv, hvt); // ~60 m marker
  if (haveVehicle) {
    std::printf("nav_finale: vehicle model %s -> %zu verts / %zu tris per agent\n", vpath.c_str(),
                hv.size() / 3, hvt.size() / 3);
    agentGeom = glyphs.build_template(app, N, color.data(), hv, hvt, 0.002 * span);
  } else {
    agentGeom = glyphs.build(app, N, color.data(), 0.02 * span, 0.006 * span);
  }
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("agents", agentGeom));
  if (agentNode) {
    agentNode->setUseSingleColor(false);
    agentNode->setAmbient(0.7);
    agentNode->setDiffuse(0.8);
  }

  // Pursuit-target markers (Act 2): a downward pyramid hovering over the streets,
  // labelled T1..T4, in the hue of a vehicle that chases it. Without these, Act 2
  // read as "the vehicles got lost" — the thing being chased was never drawn.
  const double tgtHover = 0.05 * span, tgtHalf = 0.022 * span;
  std::vector<std::shared_ptr<GeometryNode>> targetNodes(NT);
  for (int k = 0; k < NT; ++k) {
    const double rgb[3] = {color[3 * k], color[3 * k + 1], color[3 * k + 2]};
    char nm[24];
    std::snprintf(nm, sizeof nm, "target_%d", k);
    auto node = std::dynamic_pointer_cast<GeometryNode>(
        sg.addGraphics(nm, target_marker(tgtHalf, 1.6 * tgtHalf, rgb)));
    if (node) {
      node->setUseSingleColor(false);
      node->setAmbient(0.85);
      node->setDiffuse(0.6);
      char lb[8];
      std::snprintf(lb, sizeof lb, "T%d", k + 1);
      node->setLabelText(lb);
      node->setShowLabel(true);
      node->setVisible(false); // targets exist only in Act 2
      node->setPosition(cfg.cx, cfg.cy, tgtHover);
    }
    targetNodes[k] = node;
  }

  sg.addDirectionalLight(-40, 58, 1.0, 0.96, 0.88, 1.1);
  sg.addDirectionalLight(150, 32, 0.5, 0.58, 0.72, 0.45);

  SceneRenderer view(sg, width, height, offscreen, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 2 : 4);
  }
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.20, 0.26, 0.36);
  view.renderer()->SetBackground2(0.55, 0.66, 0.82);

  CameraController cam(view);
  cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, 0.05 * span);
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  // Precompute the minimap base: occupancy as a grey top-down raster.
  const int MM = 180; // minimap pixels
  cvc::image mmbase(MM, MM, cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
  {
    unsigned char *p = mmbase.data();
    const unsigned char *sd = haveSat ? sat.data() : nullptr;
    const int sw = haveSat ? sat.width() : 0, sh = haveSat ? sat.height() : 0,
              schn = haveSat ? sat.channels() : 0;
    for (int r = 0; r < MM; ++r)
      for (int c = 0; c < MM; ++c) {
        const long i = (static_cast<long>(r) * MM + c) * 3;
        if (haveSat) { // downsample the satellite (north-up: image row 0 = north = top),
                       // dimmed so the route lines + vehicle dots read on top of it
          const long si =
              (static_cast<long>(r * (sh - 1) / (MM - 1)) * sw + c * (sw - 1) / (MM - 1)) * schn;
          p[i] = static_cast<unsigned char>(0.4 * sd[si]);
          p[i + 1] = static_cast<unsigned char>(0.4 * sd[si + 1]);
          p[i + 2] = static_cast<unsigned char>(0.4 * sd[si + 2]);
        } else { // occupancy grey (north-up: minimap top -> occ row ny-1 = max_y)
          const int oc = c * (nx - 1) / (MM - 1), orr = (MM - 1 - r) * (ny - 1) / (MM - 1);
          const unsigned char v = occ[static_cast<std::size_t>(orr) * nx + oc] ? 90 : 32;
          p[i] = v;
          p[i + 1] = v + 6;
          p[i + 2] = v + 12;
        }
      }
  }

  std::printf("nav_finale: %d vehicles (%s), %dx%d occupancy, minimap=%s\n", N,
              usedBundle ? "real Austin" : "synthetic city", nx, ny, minimap ? "on" : "off");

  // 4. Run: sim on the render thread (only 8 agents). Act split at 45% of the run.
  const long total = frames > 0 ? frames : 600;
  const long act2 = static_cast<long>(0.45 * total);
  const int sub = std::max(1, static_cast<int>(hz / std::max(1.0, fps)));
  std::vector<float> pos(2 * N), head(N), spd(N);
  std::vector<int> md(N);
  std::vector<std::uint8_t> rch(N);
  std::vector<float> tgt(2 * NT); // current target world positions (Act 2)
  double eye[3], focal[3];
  long frame = 0;
  bool targetsShown = false;
  const auto tw0 = std::chrono::steady_clock::now(); // interactive wall clock
  double lastT = 0.0;

  // A* route spine: plan over a lightly-inflated occupancy (too much clearance seals
  // off dense downtown so A* can't reach a goal there); the reactive drive supplies
  // the real wall clearance while following the waypoints.
  std::vector<std::uint8_t> planOcc = cvc::nav::inflate(occ.data(), ny, nx, 1);
  std::vector<Route> routes(N);
  std::vector<std::size_t> curWp(N, static_cast<std::size_t>(-1)); // last waypoint retargeted to
  world.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data()); // initial poses
  for (int i = 0; i < N; ++i) {
    const double gx = goal[2 * i] / sc + cfg.cx, gy = goal[2 * i + 1] / sc + cfg.cy;
    routes[i] = plan_route(planOcc, ny, nx, bounds, pos[2 * i], pos[2 * i + 1], gx, gy);
  }

  while (!view.windowClosed()) {
    double t, dt;
    if (capturing) {
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - tw0).count();
      dt = t - lastT;
      lastT = t;
    }
    const bool act2now = frame >= act2;

    // Act 2: 4 targets orbit the map centre; each pair of vehicles chases one, with
    // its A* route replanned to the moving target periodically.
    if (act2now) {
      for (int k = 0; k < NT; ++k) {
        const double a = 2 * PI * k / NT + 0.25 * (t - act2 / fps);
        const double rad = 0.30 * span;
        tgt[2 * k] = static_cast<float>(cfg.cx + rad * std::cos(a));
        tgt[2 * k + 1] = static_cast<float>(cfg.cy + rad * std::sin(a));
      }
      if (!targetsShown) { // Act 2 opens: reveal the hovering target markers
        for (auto &n : targetNodes)
          if (n)
            n->setVisible(true);
        targetsShown = true;
      }
      for (int k = 0; k < NT; ++k)
        if (targetNodes[k])
          targetNodes[k]->setPosition(tgt[2 * k], tgt[2 * k + 1], tgtHover);
      if ((frame - act2) % 15 == 0)
        for (int i = 0; i < N; ++i) {
          routes[i] = plan_route(planOcc, ny, nx, bounds, pos[2 * i], pos[2 * i + 1],
                                 tgt[2 * (i % NT)], tgt[2 * (i % NT) + 1]);
          curWp[i] = static_cast<std::size_t>(-1); // force a retarget onto the new route
        }
    }

    // Follow the route: aim each vehicle at its current waypoint, advancing on arrival.
    // Retarget ONLY when the waypoint changes — retargeting every frame resets the
    // carrot FSM's escape/tracking state and the vehicle never gets moving.
    for (int i = 0; i < N; ++i) {
      Route &rt = routes[i];
      const double arr = 0.03 * span;
      while (rt.idx + 1 < rt.wp.size()) {
        const double dx = rt.wp[rt.idx][0] - pos[2 * i], dy = rt.wp[rt.idx][1] - pos[2 * i + 1];
        if (dx * dx + dy * dy < arr * arr)
          ++rt.idx;
        else
          break;
      }
      if (rt.idx != curWp[i]) {
        float w2[2];
        norm(rt.wp[rt.idx][0], rt.wp[rt.idx][1], w2);
        world.retarget(i, w2[0], w2[1]);
        curWp[i] = rt.idx;
      }
    }

    for (int s = 0; s < sub; ++s)
      world.step();
    world.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data());
    if (std::getenv("NAV_DBG") && frame % 20 == 0)
      std::printf("[dbg] f%ld act2=%d a0=(%.0f,%.0f)->wp%zu/%zu a4=(%.0f,%.0f) spd0=%.3f\n", frame,
                  static_cast<int>(act2now), pos[0], pos[1], routes[0].idx, routes[0].wp.size(),
                  pos[8], pos[9], spd[0]);
    const auto &xyz = glyphs.pack(pos.data(), head.data());
    if (agentNode)
      agentNode->updateVertices(xyz);

    if (capturing) {
      // Scripted capture camera: a slow high fly that drifts across the map.
      const double az = 0.4 + 0.25 * std::sin(0.12 * t);
      navdemo::orbit_camera(bounds, 0.05 * span, az, 42.0 * PI / 180.0, 1.9, eye, focal);
      view.setCamera(eye[0], eye[1], eye[2], focal[0], focal[1], focal[2], 0, 0, 1, 30);
    } else {
      // Interactive: the CameraController owns the camera (Tab orbit/fly, WASD +
      // mouse). The scripted path above must never stomp it — that killed all
      // mouse input in v1.
      cam.update(dt);
    }

    if (capturing) {
      view.render();
      std::vector<unsigned char> rgb = view.frameRGB(); // bottom-up RGB
      const int W = view.frameWidth(), H = view.frameHeight();
      cvc::image frameImg(W, H, cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
      unsigned char *fp = frameImg.data();
      for (int r = 0; r < H; ++r) // flip bottom-up -> top-down
        std::copy(&rgb[(static_cast<long>(H - 1 - r) * W) * 3],
                  &rgb[(static_cast<long>(H - r) * W) * 3], &fp[static_cast<long>(r) * W * 3]);

      if (minimap) {
        // All compositing goes through nav_common's bounds-safe blit/plot, so an
        // oversized minimap or an off-map agent can never overflow the frame buffer.
        const int pad = 12, x0 = W - MM - pad, y0 = pad;
        navdemo::blit_clamped(fp, W, H, mmbase.data(), MM, MM, 3, x0, y0);
        auto plot = [&](double wx, double wy, unsigned char R, unsigned char G, unsigned char B,
                        int rad) {
          const int c =
              static_cast<int>((wx - bounds.min_x) / (bounds.max_x - bounds.min_x) * (MM - 1));
          const int rr =
              static_cast<int>((bounds.max_y - wy) / (bounds.max_y - bounds.min_y) * (MM - 1));
          navdemo::plot_disc(fp, W, H, x0 + c, y0 + rr, rad, R, G, B);
        };
        for (int i = 0; i < N; ++i) {
          const double wx = pos[2 * i] / sc + cfg.cx, wy = pos[2 * i + 1] / sc + cfg.cy;
          { // predicted path = the remaining A* route waypoints
            const Route &rt = routes[i];
            double ax = wx, ay = wy;
            for (std::size_t w = rt.idx; w < rt.wp.size(); ++w) {
              const double qx = rt.wp[w][0], qy = rt.wp[w][1];
              for (double u = 0; u <= 1.0; u += 0.04)
                plot(ax + (qx - ax) * u, ay + (qy - ay) * u, 245, 235, 90, 1);
              ax = qx;
              ay = qy;
            }
          }
          plot(wx, wy, 12, 12, 12, 4); // dark halo so the dot reads on the aerial
          plot(wx, wy, static_cast<unsigned char>(color[3 * i] * 255),
               static_cast<unsigned char>(color[3 * i + 1] * 255),
               static_cast<unsigned char>(color[3 * i + 2] * 255), 3);
        }
        if (act2now)
          for (int k = 0; k < NT; ++k)
            plot(tgt[2 * k], tgt[2 * k + 1], 255, 60, 60, 3);
      }
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", out.c_str(), frame);
      cvc::write_image(frameImg, path);
    } else {
      view.processUIEvents();
      view.render();
    }

    ++frame;
    if (frames > 0 && frame >= frames)
      break;
    if (frames == 0 && !capturing && view.windowClosed())
      break;
    if (frames == 0 && capturing && frame >= total)
      break;
  }

  cam.detach();
  if (!png.empty())
    view.writePNG(png);
  std::printf("nav_finale: done (%ld frames)\n", frame);
  return 0;
}

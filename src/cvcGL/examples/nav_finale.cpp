// nav_finale — the GRL-SNAM "finale" in 3-D on the real Austin bundle: 8 vehicles
// enter blind from the west, rendezvous on a staging line (Act 1), then split into
// pursuit packs that chase 4 moving targets (Act 2) — all on cvc::nav's sim_world,
// no Python, no libtorch. The city occupancy is rasterized in C++ straight from the
// bundle's buildings.glb (occupancy_from_model, the C++ building_occupancy). A 2-D
// picture-in-picture minimap in the corner shows every agent's position and the line
// to the target it is currently chasing (its predicted heading).
//
// The bundle (terrain.json + buildings.glb) is passed at runtime and never vendored:
//   nav_finale --bundle /path/to/scenes/austin_south --capture fly --offscreen \
//              --frames 600 --out /tmp/finale
//   ffmpeg -framerate 30 -i /tmp/finale/frame_%05d.png -c:v libx264 -pix_fmt yuv420p finale.mp4
// With no --bundle it falls back to a synthetic city so the scenario still runs.

#include "nav_common.h"

#include <algorithm>
#include <boost/program_options.hpp>
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
} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  std::string bundle, capture = "fly", out = "frames", png;
  int width = 1280, height = 720, nx = 384;
  long frames = 0;
  double fps = 30.0, hz = 60.0;
  bool offscreen = false, no_shadows = false, no_minimap = false;

  po::options_description desc("nav_finale — the 8-vehicle Austin finale in cvcGL");
  desc.add_options()("help,h",
                     "show this help")("bundle", po::value<std::string>(&bundle),
                                       "Austin bundle dir (terrain.json + buildings.glb)")(
      "grid", po::value<int>(&nx)->default_value(384), "occupancy resolution")(
      "offscreen", po::bool_switch(&offscreen))("no-shadows", po::bool_switch(&no_shadows))(
      "no-minimap", po::bool_switch(&no_minimap),
      "hide the 2-D PiP minimap")("frames", po::value<long>(&frames)->default_value(0))(
      "fps", po::value<double>(&fps)->default_value(30.0))(
      "hz", po::value<double>(&hz)->default_value(60.0))(
      "capture", po::value<std::string>(&capture)->default_value("fly"),
      "none | fly | orbit")("width", po::value<int>(&width)->default_value(1280))(
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
    std::filesystem::create_directories(out);
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
      // inflate 1: fills sub-cell undersampling gaps + gives agents wall clearance.
      occ = navdemo::occupancy_from_model(cityMesh, bounds, nx, ny, /*inflate=*/1);
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
  const double sc = 1.0 / (0.5 * span); // world -> normalized (centered)
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
  cfg.veh.rr = 0.03f;
  cfg.veh.d_hat = 0.08f;
  cfg.veh.dt = 0.06f;
  cfg.veh.vmax = 0.9f;
  cfg.freeze_sense = false; // fog: each vehicle builds its own map
  cfg.sense_every = 3;
  cfg.range_m = 0.12 / sc; // ~12% of the map
  cfg.n_rays = 200;
  cfg.reach_tol = 1.2f;

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
  if (groundNode) {
    groundNode->setAmbient(0.6);
    groundNode->setDiffuse(0.5);
  }

  navdemo::AgentGlyphs glyphs;
  const double gsz = 0.012 * span;
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("agents", glyphs.build(app, N, color.data(), gsz, 0.02 * span)));
  if (agentNode) {
    agentNode->setUseSingleColor(false);
    agentNode->setAmbient(0.7);
    agentNode->setDiffuse(0.8);
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
  const int MM = 168; // minimap pixels
  cvc::image mmbase(MM, MM, cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
  {
    unsigned char *p = mmbase.data();
    for (int r = 0; r < MM; ++r)
      for (int c = 0; c < MM; ++c) {
        // north-up: minimap top row -> occ row ny-1 (max_y), matching the agent plot.
        const int oc = c * (nx - 1) / (MM - 1), orr = (MM - 1 - r) * (ny - 1) / (MM - 1);
        const bool wall = occ[static_cast<std::size_t>(orr) * nx + oc] != 0;
        unsigned char v = wall ? 90 : 32;
        const long i = (static_cast<long>(r) * MM + c) * 3;
        p[i] = v;
        p[i + 1] = v + 6;
        p[i + 2] = v + 12;
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

  while (!view.windowClosed()) {
    const double t = frame / fps;
    const bool act2now = frame >= act2;

    // Act 2: 4 targets orbit the map centre; each pair of vehicles chases one.
    if (act2now) {
      for (int k = 0; k < NT; ++k) {
        const double a = 2 * PI * k / NT + 0.25 * (t - act2 / fps);
        const double rad = 0.30 * span;
        tgt[2 * k] = static_cast<float>(cfg.cx + rad * std::cos(a));
        tgt[2 * k + 1] = static_cast<float>(cfg.cy + rad * std::sin(a));
      }
      for (int i = 0; i < N; ++i) {
        float g2[2];
        norm(tgt[2 * (i % NT)], tgt[2 * (i % NT) + 1], g2);
        world.retarget(i, g2[0], g2[1]);
      }
    }

    for (int s = 0; s < sub; ++s)
      world.step();
    world.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data());
    const auto &xyz = glyphs.pack(pos.data(), head.data());
    if (agentNode)
      agentNode->updateVertices(xyz);

    // Continuous camera: a slow high fly that drifts across the map.
    const double az = 0.4 + 0.25 * std::sin(0.12 * t);
    navdemo::orbit_camera(bounds, 0.05 * span, az, 42.0 * PI / 180.0, 1.9, eye, focal);
    view.setCamera(eye[0], eye[1], eye[2], focal[0], focal[1], focal[2], 0, 0, 1, 30);

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
        const int pad = 12, x0 = W - MM - pad, y0 = pad;
        const unsigned char *mp = mmbase.data();
        for (int r = 0; r < MM; ++r)
          for (int c = 0; c < MM; ++c) {
            const long d = (static_cast<long>(y0 + r) * W + (x0 + c)) * 3;
            const long sI = (static_cast<long>(r) * MM + c) * 3;
            for (int ch = 0; ch < 3; ++ch)
              fp[d + ch] = mp[sI + ch];
          }
        auto plot = [&](double wx, double wy, unsigned char R, unsigned char G, unsigned char B,
                        int rad) {
          const int c =
              static_cast<int>((wx - bounds.min_x) / (bounds.max_x - bounds.min_x) * (MM - 1));
          const int rr =
              static_cast<int>((bounds.max_y - wy) / (bounds.max_y - bounds.min_y) * (MM - 1));
          for (int dy = -rad; dy <= rad; ++dy)
            for (int dx = -rad; dx <= rad; ++dx) {
              const int px = x0 + c + dx, py = y0 + rr + dy;
              if (px < 0 || py < 0 || px >= W || py >= H)
                continue;
              const long d = (static_cast<long>(py) * W + px) * 3;
              fp[d] = R;
              fp[d + 1] = G;
              fp[d + 2] = B;
            }
        };
        for (int i = 0; i < N; ++i) {
          const double wx = pos[2 * i] / sc + cfg.cx, wy = pos[2 * i + 1] / sc + cfg.cy;
          if (act2now) { // predicted-path line to the chased target
            const double gx = tgt[2 * (i % NT)], gy = tgt[2 * (i % NT) + 1];
            for (double u = 0; u <= 1.0; u += 0.05)
              plot(wx + (gx - wx) * u, wy + (gy - wy) * u, 230, 220, 90, 0);
          }
          plot(wx, wy, static_cast<unsigned char>(color[3 * i] * 255),
               static_cast<unsigned char>(color[3 * i + 1] * 255),
               static_cast<unsigned char>(color[3 * i + 2] * 255), 2);
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

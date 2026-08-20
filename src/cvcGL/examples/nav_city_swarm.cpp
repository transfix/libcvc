// nav_city_swarm — a cvcGL demo of the cvc::nav reactive swarm: N vehicles navigate
// a procedural "city" (the same city_scene the trainer uses), rendered with no Python
// and no libtorch. The whole swarm runs on a sim_thread off the render thread; agents
// are ONE merged glyph mesh streamed via GeometryNode::updateVertices, so it stays
// smooth into the thousands. Belief is shared / grouped / private (agents coloured by
// their belief group). Interactive window, or offscreen capture -> PNG frames -> mp4.
//
//   nav_city_swarm --agents 1500 --belief grouped --capture orbit --offscreen \
//                  --frames 300 --out /tmp/city && \
//   ffmpeg -framerate 30 -i /tmp/city/frame_%05d.png -c:v libx264 -pix_fmt yuv420p city.mp4

#include "nav_common.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::CameraController;

namespace {
const double PI = std::acos(-1.0);

// HSV (h in [0,1), s, v) -> RGB, for per-group agent colours.
void hsv2rgb(double h, double s, double v, float out[3]) {
  const double i = std::floor(h * 6.0);
  const double f = h * 6.0 - i;
  const double p = v * (1.0 - s), q = v * (1.0 - f * s), t = v * (1.0 - (1.0 - f) * s);
  double r = v, g = v, b = v;
  switch (static_cast<int>(i) % 6) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  default:
    r = v;
    g = p;
    b = q;
    break;
  }
  out[0] = static_cast<float>(r);
  out[1] = static_cast<float>(g);
  out[2] = static_cast<float>(b);
}
} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int agents = 512, grid = 96, width = 1280, height = 720;
  long frames = 0;
  double fps = 30.0, hz = 60.0;
  std::string belief = "shared", capture = "orbit", out = "frames", png;
  bool offscreen = false, fog = false, no_shadows = false, ortho = false;

  po::options_description desc("nav_city_swarm — cvc::nav swarm in a cvcGL city");
  desc.add_options()("help,h", "show this help")(
      "agents", po::value<int>(&agents)->default_value(512), "number of vehicles")(
      "grid", po::value<int>(&grid)->default_value(96), "city occupancy resolution")(
      "belief", po::value<std::string>(&belief)->default_value("shared"),
      "shared | grouped | private")("fog", po::bool_switch(&fog),
                                    "enable sensing so belief diverges (default: static map)")(
      "offscreen", po::bool_switch(&offscreen), "render offscreen (forced by --capture)")(
      "ortho", po::bool_switch(&ortho), "top-down 2-D orthographic view (matplotlib-style)")(
      "no-shadows", po::bool_switch(&no_shadows),
      "disable shadow map")("frames", po::value<long>(&frames)->default_value(0),
                            "stop after N frames (0 = until closed)")(
      "fps", po::value<double>(&fps)->default_value(30.0),
      "capture frame rate")("hz", po::value<double>(&hz)->default_value(60.0), "sim tick rate")(
      "capture", po::value<std::string>(&capture)->default_value("orbit"),
      "none | orbit | fly")("width", po::value<int>(&width)->default_value(1280))(
      "height", po::value<int>(&height)->default_value(720))(
      "out", po::value<std::string>(&out)->default_value("frames"),
      "PNG frame directory")("png", po::value<std::string>(&png), "write a single final PNG here");
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

  // Ortho 2-D reads cleanest flat: force shadows off and a straight top-down camera.
  if (ortho)
    no_shadows = true;

  // 1. Occupancy from the pure-C++ "city" scene (same map the trainer uses). Wall the
  //    border so reactive agents can't drive off the open edge (city_scene has none).
  cvc::nav::training_scene ts = cvc::nav::city_scene(grid);
  navdemo::add_border(ts.occ.data(), ts.rows, ts.cols);
  const navdemo::Bounds bounds{ts.min_x, ts.min_y, ts.max_x, ts.max_y};
  const double span = ts.max_x - ts.min_x;
  const double wall_h = 0.06 * span; // blocky buildings

  cvc::nav::sim_world::config cfg;
  cfg.rows = ts.rows;
  cfg.cols = ts.cols;
  cfg.min_x = ts.min_x;
  cfg.min_y = ts.min_y;
  cfg.max_x = ts.max_x;
  cfg.max_y = ts.max_y;
  cfg.cx = ts.cx;
  cfg.cy = ts.cy;
  cfg.scale = ts.scale;
  cfg.veh.rr = ts.rr;
  cfg.veh.d_hat = ts.d_hat;
  cfg.veh.dt = ts.dt;
  cfg.veh.vmax = ts.vmax;
  cfg.freeze_sense = !fog; // static known map by default (the thousands-of-agents path)
  cfg.reach_tol = 0.8f;

  auto mode = cvc::nav::sim_world::belief_mode::shared;
  int clusters = 1;
  if (belief == "private")
    mode = cvc::nav::sim_world::belief_mode::private_belief;
  else if (belief == "grouped") {
    mode = cvc::nav::sim_world::belief_mode::clustered;
    clusters = std::max(2, agents / 64);
  }

  cvc::nav::sim_world world = cvc::nav::sim_world::from_occupancy(
      cfg, ts.occ.data(), cvc::nav::coef_mlp::default_biased(), agents, /*seed=*/7, mode, clusters);
  const int N = world.size();

  // 2. Per-agent glyph colours: by belief group (grouped/private) so the grouping is
  //    visible; shared -> a rainbow by index.
  const int *grp = world.agent_planes();
  const int M = world.planes();
  std::vector<float> color(3 * N);
  for (int i = 0; i < N; ++i) {
    const double hue = (M > 1) ? static_cast<double>(grp[i]) / M : static_cast<double>(i) / N;
    hsv2rgb(hue, 0.62, 0.96, &color[3 * i]);
  }

  // 3. Scene.
  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "city");

  const double wall_rgb[3] = {0.58, 0.58, 0.64};
  cvc::geometry walls =
      navdemo::occupancy_to_walls(ts.occ.data(), ts.rows, ts.cols, bounds, wall_h, wall_rgb);
  sg.addGraphics("walls", walls);

  const double ground_rgb[3] = {0.20, 0.23, 0.27};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", navdemo::ground_quad(bounds, 0.0, ground_rgb)));
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
    groundNode->setAmbient(0.65); // soften the shadow-map boundary on the big flat ground
    groundNode->setDiffuse(0.50);
  }

  navdemo::AgentGlyphs glyphs;
  const double gsz = 0.02 * span;
  cvc::geometry agentGeom = glyphs.build(app, N, color.data(), gsz, 0.6);
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("agents", agentGeom));
  if (agentNode) {
    agentNode->setUseSingleColor(false); // per-vertex group colours
    agentNode->setAmbient(0.55);
    agentNode->setDiffuse(0.85);
  }

  sg.addDirectionalLight(-42, 58, 1.0, 0.96, 0.86, 1.15);  // warm key
  sg.addDirectionalLight(140, 30, 0.55, 0.62, 0.78, 0.50); // cool fill

  SceneRenderer view(sg, width, height, offscreen, "main");
  // Shadows must be enabled AFTER the renderer exists (they attach to its passes).
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 1 : 3);
  }
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.28, 0.40, 0.58);
  view.renderer()->SetBackground2(0.66, 0.78, 0.92);

  CameraController cam(view);
  cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, wall_h);
  if (ortho)
    navdemo::set_ortho_topdown(view, bounds, 4.0); // fixed 2-D top-down map
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("nav_city_swarm: %d agents, belief=%s (M=%d), grid=%d, %s, shadows=%s\n", N,
              belief.c_str(), M, grid, fog ? "fog" : "static-map", shadows ? "on" : "off");

  // 4. Run: sim off-thread; render loop streams poses into the merged glyph mesh.
  cvc::nav::sim_thread sim(world, hz);
  sim.start();

  const auto t0 = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  double eye[3], focal[3];
  while (!view.windowClosed()) {
    double t, dt;
    if (capturing) {
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      dt = t - last;
      last = t;
    }
    view.processUIEvents();

    if (auto snap = sim.read()) {
      if (snap->n == N) {
        const auto &xyz = glyphs.pack(snap->pos.data(), snap->heading.data());
        if (agentNode)
          agentNode->updateVertices(xyz);
      }
    }

    if (ortho) {
      // fixed top-down map — camera set once, nothing to move
    } else if (capturing && capture == "fly") {
      const double az = 0.6 + 0.15 * t;
      const double el = (34.0 - 12.0 * std::sin(0.25 * t)) * PI / 180.0;
      navdemo::orbit_camera(bounds, wall_h * 0.5, az, el, 1.4, eye, focal);
      view.setCamera(eye[0], eye[1], eye[2], focal[0], focal[1], focal[2], 0, 0, 1, 32);
    } else if (capturing) { // orbit
      const double az = 0.7 + 0.30 * t;
      navdemo::orbit_camera(bounds, wall_h * 0.5, az, 36.0 * PI / 180.0, 1.75, eye, focal);
      view.setCamera(eye[0], eye[1], eye[2], focal[0], focal[1], focal[2], 0, 0, 1, 30);
    } else {
      cam.update(dt);
    }

    if (capturing) {
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", out.c_str(), frame);
      view.writePNG(path);
    } else {
      view.render();
    }

    ++frame;
    if (frames > 0 && frame >= frames)
      break;
  }

  sim.stop();
  cam.detach();
  if (!png.empty())
    view.writePNG(png);
  std::printf("nav_city_swarm: done (%ld frames, sim ticks=%ld)\n", frame, sim.ticks());
  return 0;
}

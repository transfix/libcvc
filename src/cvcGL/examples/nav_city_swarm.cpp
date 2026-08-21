// nav_city_swarm — a cvcGL demo of the cvc::nav reactive swarm: N vehicles cross a
// procedural "city" (the same city_scene the trainer learns on) toward per-agent goals
// with NO global plan — pure local reaction, no Python, no libtorch. Each agent, each
// tick: sample the signed-distance clearance field phi -> a tiny CoefMLP (coef_mlp.h,
// 5->64->64->3 SiLU) emits (alpha,beta,gamma) = wall-barrier / goal-spring / damping ->
// a kinematic bicycle steers toward a "carrot" on the goal bearing, deflected by the
// barrier, with a bug-style wall-follow escape on a stall (the carrot FSM, drive.h).
// The whole swarm runs on a sim_thread off the render thread; agents are ONE merged
// glyph mesh streamed via GeometryNode::updateVertices, so it stays smooth into the
// thousands. --belief shared|grouped|private picks how many log-odds belief planes the
// fleet keeps (agents coloured by plane), but the modes only DIVERGE under --fog: with
// sensing off every plane equals the known map, so grouped/private just recolour the
// same trajectories. Interactive window, or offscreen capture -> PNG frames -> mp4.
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
#include <cvc/gl/ScreenTextHud.h>
#include <cvc/image/image.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h> // SceneGraph.h only forward-declares it
#include <emscripten.h>
#endif
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

// Fleet fog coverage on the ground: the union over ALL belief planes of what the
// fleet has seen. Three tiers — never-seen near-black, remembered dim, currently
// in some agent's view lit. With --belief grouped/private the coverage visibly
// grows per group: "coverage shared, knowledge private" becomes a picture.
void fill_fleet_fog(unsigned char *px, const cvc::nav::sim_world &w, int rows, int cols) {
  const int M = w.planes();
  for (long i = 0; i < static_cast<long>(rows) * cols; ++i) {
    bool vis = false, seen = false;
    for (int m = 0; m < M && !vis; ++m) {
      vis = vis || w.last_visible(m)[i];
      seen = seen || w.ever_seen(m)[i];
    }
    if (!vis)
      for (int m = 0; m < M && !seen; ++m)
        seen = w.ever_seen(m)[i] != 0;
    unsigned char *p = px + 4 * i;
    if (vis) {
      p[0] = 62;
      p[1] = 68;
      p[2] = 78;
    } else if (seen) {
      p[0] = 40;
      p[1] = 44;
      p[2] = 50;
    } else {
      p[0] = 20;
      p[1] = 22;
      p[2] = 26;
    }
    p[3] = 255;
  }
}
} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int agents = 512, grid = 96, width = 1280, height = 720;
  long frames = 0;
  double mouseSens = 0.25, moveSpeed = 0.0; // camera feel (0 move speed = auto from bounds)
  double fps = 30.0, hz = 60.0;
  std::string belief = "shared", capture = "orbit", out = "frames", png;
  bool offscreen = false, fog = false, no_fog = false, no_shadows = false, ortho = false;

  po::options_description desc("nav_city_swarm — cvc::nav swarm in a cvcGL city");
  desc.add_options()("help,h", "show this help")(
      "agents", po::value<int>(&agents)->default_value(800), "number of vehicles")(
      "grid", po::value<int>(&grid)->default_value(96), "city occupancy resolution")(
      "belief", po::value<std::string>(&belief)->default_value("grouped"),
      "shared | grouped | private")("fog", po::bool_switch(&fog),
                                    "(legacy no-op; fog is on by default)")(
      "no-fog", po::bool_switch(&no_fog),
      "static known map, no sensing — the scale-benchmark path (try --agents 1500)")(
      "offscreen", po::bool_switch(&offscreen), "render offscreen (forced by --capture)")(
      "ortho", po::bool_switch(&ortho), "top-down 2-D orthographic view (matplotlib-style)")(
      "no-shadows", po::bool_switch(&no_shadows),
      "disable shadow map")("frames", po::value<long>(&frames)->default_value(0),
                            "stop after N frames (0 = until closed)")(
      "fps", po::value<double>(&fps)->default_value(30.0),
      "capture frame rate")("hz", po::value<double>(&hz)->default_value(60.0), "sim tick rate")(
      "capture", po::value<std::string>(&capture)->default_value("none"),
      "none (interactive window) | orbit | fly (offscreen PNG capture)")(
      "mouse-sensitivity", po::value<double>(&mouseSens)->default_value(0.25),
      "look speed, degrees per pixel of mouse motion")(
      "move-speed", po::value<double>(&moveSpeed)->default_value(0.0),
      "fly speed in world units/s (0 = auto from scene bounds)")(
      "width", po::value<int>(&width)->default_value(1280))(
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
    if (frames <= 0)
      frames = 600; // a capture must end — 0 would render offscreen forever, looking hung
    std::filesystem::create_directories(out);
    std::printf("nav_city_swarm: capturing %ld frames (%s, offscreen) -> %s/frame_*.png\n", frames,
                capture.c_str(), out.c_str());
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
  const bool fogOn = !no_fog; // fog by default: belief modes only MEAN something with sensing
  (void)fog;                  // legacy switch, absorbed by the fog-on default
  cfg.freeze_sense = !fogOn;
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
      navdemo::occupancy_to_walls(ts.occ.data(), ts.rows, ts.cols, bounds, wall_h, wall_rgb,
                                  /*vary=*/0.45);
  sg.addGraphics("walls", walls);

  const double ground_rgb[3] = {0.20, 0.23, 0.27};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", navdemo::ground_quad(bounds, 0.0, ground_rgb)));
  cvc::image fogTex(grid, grid, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    groundNode->setAmbient(0.65); // soften the shadow-map boundary on the big flat ground
    groundNode->setDiffuse(0.50);
    if (fogOn) { // the ground IS the fleet's fog coverage
      fill_fleet_fog(fogTex.data(), world, ts.rows, ts.cols);
      groundNode->setColor(1, 1, 1);
      groundNode->setTexture(fogTex, /*zeroCopy=*/true);
    } else {
      groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
    }
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

  // Every agent's DESTINATION, rendered: one merged mesh of hovering pyramids in
  // each agent's hue (goals_world), packed once — N wandering triangles become N
  // journeys. Without this the learned controller is indistinguishable from boids.
  // An ARRIVED agent's pyramid sinks away, so clutter falls as progress accrues.
  navdemo::AgentGlyphs goalGlyphs;
  std::shared_ptr<GeometryNode> goalsNode;
  std::vector<double> goalXyz;
  {
    const double gh = 0.006 * span, ght = 1.7 * gh;
    const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                    ght, gh, gh, ght, -gh, gh,  ght};
    const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 1, 3, 2, 1, 4, 3};
    cvc::geometry goalGeom =
        goalGlyphs.build_template(app, N, color.data(), pv, pt, /*z=*/0.012 * span);
    goalsNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("goals", goalGeom));
    std::vector<float> gw(static_cast<std::size_t>(2) * N), zeroHead(N, 0.0f);
    world.goals_world(gw.data());
    goalXyz = goalGlyphs.pack(gw.data(), zeroHead.data()); // keep: arrived goals sink away
    if (goalsNode) {
      goalsNode->setUseSingleColor(false);
      goalsNode->setAmbient(0.85);
      goalsNode->setDiffuse(0.55);
      goalsNode->updateVertices(goalXyz);
    }
  }

  // Breadcrumb TRAILS: one merged LINES mesh, K segments per agent in a ring
  // buffer, streamed every 3rd frame — the emergent street-flow accumulates on
  // screen instead of evaporating every frame. Trail colour = agent hue, dimmed.
  const int TRAIL_K = 12;
  std::shared_ptr<GeometryNode> trailNode;
  std::vector<double> trailXyz;
  std::vector<float> trailLast(static_cast<std::size_t>(2) * N);
  std::vector<int> trailWr(N, 0);
  {
    std::vector<float> sp0(static_cast<std::size_t>(2) * N);
    world.snapshot(sp0.data(), nullptr, nullptr, nullptr, nullptr); // start poses
    trailLast = sp0;
    cvc::geometry tg;
    trailXyz.resize(static_cast<std::size_t>(3) * 2 * TRAIL_K * N);
    for (int i = 0; i < N; ++i)
      for (int k = 0; k < TRAIL_K; ++k)
        for (int e = 0; e < 2; ++e) {
          const std::size_t v = (static_cast<std::size_t>(i) * TRAIL_K + k) * 2 + e;
          tg.points().push_back({sp0[2 * i], sp0[2 * i + 1], 0.35});
          tg.colors().push_back(
              {0.45 * color[3 * i], 0.45 * color[3 * i + 1], 0.45 * color[3 * i + 2]});
          trailXyz[3 * v] = sp0[2 * i];
          trailXyz[3 * v + 1] = sp0[2 * i + 1];
          trailXyz[3 * v + 2] = 0.35;
          if (e == 1)
            tg.lines().push_back({static_cast<cvc::geometry::index_t>(v - 1),
                                  static_cast<cvc::geometry::index_t>(v)});
        }
    trailNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("trails", tg));
    if (trailNode) {
      trailNode->setRenderMode(GeometryRenderMode::LINES);
      trailNode->setUseSingleColor(false);
      trailNode->setLineWidth(1.5);
      trailNode->setAmbient(1.0);
      trailNode->setDiffuse(0.0);
      trailNode->setDepthOffset(1.0);
    }
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

  cvc::gl::ScreenTextHud status(view); // corner status line — claims go on screen
  status.setCentered(false);
  status.setPosition(0.015, 0.95);
  status.setFontSize(13);
  status.setColor(0.75, 0.80, 0.86);

  CameraController cam(view);
  cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, wall_h);
  cam.setMouseSensitivity(mouseSens); // --mouse-sensitivity / state settings.mouse_sensitivity
  if (moveSpeed > 0.0)
    cam.setMoveSpeed(moveSpeed);
  if (ortho)
    navdemo::set_ortho_topdown(view, bounds, 4.0); // fixed 2-D top-down map
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("nav_city_swarm: %d agents, belief=%s (M=%d), grid=%d, %s, shadows=%s\n", N,
              belief.c_str(), M, grid, fogOn ? "fog" : "static-map", shadows ? "on" : "off");

  // 4. Run: sim off-thread; render loop streams poses into the merged glyph mesh.
#ifndef __EMSCRIPTEN__
  cvc::nav::sim_thread sim(world, hz);
  sim.start();
#else
  // Single-threaded browser build: no sim_thread worker (GitHub Pages can't send
  // the COOP/COEP headers a pthreads wasm build needs). Step the world inline each
  // frame and read its world-metre snapshot directly — the same call sim_thread makes.
  std::vector<float> emPos(static_cast<std::size_t>(N) * 2), emHead(N);
  std::vector<int> emMd(N);
  std::vector<std::uint8_t> emRch(N);
  (void)hz;
#endif

  std::vector<unsigned char> rgbBuf; // agent state-restyle scratch
  long arrived = 0;
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

    // Trails: append a ring-buffer segment per agent that moved (every 3rd frame,
    // one buffer upload for the whole fleet).
    auto trail_update = [&](const float *p) {
      if (!trailNode)
        return;
      for (int i = 0; i < N; ++i) {
        const float mdx = p[2 * i] - trailLast[2 * i], mdy = p[2 * i + 1] - trailLast[2 * i + 1];
        const float m2 = mdx * mdx + mdy * mdy;
        if (m2 < 1.0f)
          continue;
        if (m2 > 9.0f) {               // too far since the last sample — a chord would cut a
          trailLast[2 * i] = p[2 * i]; // corner THROUGH a building; restart instead
          trailLast[2 * i + 1] = p[2 * i + 1];
          continue;
        }
        const int k = trailWr[i];
        const std::size_t v = (static_cast<std::size_t>(i) * TRAIL_K + k) * 2;
        trailXyz[3 * v] = trailLast[2 * i];
        trailXyz[3 * v + 1] = trailLast[2 * i + 1];
        trailXyz[3 * v + 2] = 0.35;
        trailXyz[3 * (v + 1)] = p[2 * i];
        trailXyz[3 * (v + 1) + 1] = p[2 * i + 1];
        trailXyz[3 * (v + 1) + 2] = 0.35;
        trailLast[2 * i] = p[2 * i];
        trailLast[2 * i + 1] = p[2 * i + 1];
        trailWr[i] = (k + 1) % TRAIL_K;
      }
      trailNode->updateVertices(trailXyz);
    };

    // Fleet fog coverage repaint (sense cadence, not per frame).
    if (fogOn && groundNode && frame % 15 == 0) {
      fill_fleet_fog(fogTex.data(), world, ts.rows, ts.cols);
      groundNode->texture_modified();
    }

    // State restyle (every 2nd frame, one buffer upload): cruising = agent hue,
    // wall-follow escape = white flash, arrived = calm green. Stalls and
    // successes stop looking identical to "the demo broke".
    auto restyle = [&](const int *mode, const std::uint8_t *reach) {
      long arr = 0;
      for (int i = 0; i < N; ++i)
        arr += reach[i] ? 1 : 0;
      arrived = arr;
      if (goalsNode && !goalXyz.empty() && frame % 5 == 0) {
        const int GV = goalGlyphs.point_count() / N;
        bool changed = false;
        for (int i = 0; i < N; ++i)
          if (reach[i]) {
            double *p = goalXyz.data() + static_cast<std::size_t>(3) * GV * i;
            if (p[2] > -1.0) { // not yet sunk
              for (int v = 0; v < GV; ++v)
                p[3 * v + 2] = -5.0;
              changed = true;
            }
          }
        if (changed)
          goalsNode->updateVertices(goalXyz);
      }
      if (!agentNode || (frame % 2))
        return;
      const int V = glyphs.point_count() / N;
      rgbBuf.resize(static_cast<std::size_t>(3) * N * V);
      for (int i = 0; i < N; ++i) {
        unsigned char r, g, b;
        if (reach[i]) {
          r = 110;
          g = 190;
          b = 130;
        } else if (mode[i] == 1) {
          r = 255;
          g = 255;
          b = 255;
        } else {
          r = static_cast<unsigned char>(color[3 * i] * 255);
          g = static_cast<unsigned char>(color[3 * i + 1] * 255);
          b = static_cast<unsigned char>(color[3 * i + 2] * 255);
        }
        unsigned char *p = rgbBuf.data() + static_cast<std::size_t>(3) * V * i;
        for (int v = 0; v < V; ++v) {
          p[3 * v] = r;
          p[3 * v + 1] = g;
          p[3 * v + 2] = b;
        }
      }
      agentNode->updateColors(rgbBuf);
    };

#ifndef __EMSCRIPTEN__
    if (auto snap = sim.read()) {
      if (snap->n == N) {
        const auto &xyz = glyphs.pack(snap->pos.data(), snap->heading.data());
        if (agentNode)
          agentNode->updateVertices(xyz);
        restyle(snap->mode.data(), snap->reached.data());
        trail_update(snap->pos.data());
      }
    }
#else
    world.step(0);
    world.snapshot(emPos.data(), emHead.data(), nullptr, emMd.data(), emRch.data());
    {
      const auto &xyz = glyphs.pack(emPos.data(), emHead.data());
      if (agentNode)
        agentNode->updateVertices(xyz);
      restyle(emMd.data(), emRch.data());
      trail_update(emPos.data());
    }
#endif

    if (frame % 15 == 0) { // HUD status line (throttled)
      char st[200];
      std::snprintf(st, sizeof st,
                    "%d agents · belief %s (M=%d) · learned CoefMLP · fog %s · arrivals %ld/%d", N,
                    belief.c_str(), M, fogOn ? "on" : "off", arrived, N);
      status.setText(st);
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

#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush(); // no worker thread — drain publishes at frame cadence
#endif
    emscripten_sleep(0); // yield to the browser event loop (Asyncify)
#endif
    ++frame;
    if (frames > 0 && frame >= frames)
      break;
  }

#ifndef __EMSCRIPTEN__
  sim.stop();
#endif
  cam.detach();
  if (!png.empty())
    view.writePNG(png);
#ifndef __EMSCRIPTEN__
  std::printf("nav_city_swarm: done (%ld frames, sim ticks=%ld)\n", frame, sim.ticks());
#endif
  return 0;
}

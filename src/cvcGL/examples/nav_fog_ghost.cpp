// nav_fog_ghost — the fog-of-war "ghost" story from GRL-SNAM, in 3-D. One vehicle
// carries a STALE belief map with a phantom wall across its path that reality lacks.
// The phantom is a zero-clearance trough in the agent's signed-distance field, so the
// reactive drive's wall-barrier term (alpha) deflects its carrot AROUND empty space and
// it sets off detouring. As it advances, its sensor ray-casts the TRUE map into its
// belief: the phantom cells accrue "free" evidence, the occupied bit flips, the SDF is
// rebuilt WITHOUT the ghost (the rebuild IS the replan), the barrier vanishes, and the
// drive straightens through. The ground is textured with the agent's own planning field
// (phi, the SDF clearance field — NOT the raw log-odds belief grid), so you watch the
// ghost trough dissolve as the sensor sweeps it. This is the reactive CORE only: one
// agent, fixed goal, driven by the SDF gradient alone — GRL-SNAM's ghost also layers a
// BeliefRoutePlanner (A*) over a log-odds grid; here the field gradient carries the
// detour. Belief-vs-truth, rendered with no Python, no libtorch (cvc::nav + cvcGL).
//
//   nav_fog_ghost --capture orbit --offscreen --frames 400 --out /tmp/ghost && \
//   ffmpeg -framerate 30 -i /tmp/ghost/frame_%05d.png -c:v libx264 -pix_fmt yuv420p ghost.mp4

#include "nav_common.h"

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/image/image.h>
#include <cvc/nav/coef_mlp.h>
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

// Map a planning-field clearance phi (SDF, normalized units) to an RGBA "belief"
// colour: believed-occupied / near a wall -> hot red-orange (opaque), open space ->
// cool blue (faint), so a phantom wall reads as a red band that fades as it clears.
void phi_to_rgba(float phi, float clip, unsigned char out[4]) {
  const float t = std::min(std::max(phi, 0.0f), clip) / clip;  // 0 = wall, 1 = wide open
  const float r = 1.0f - t;                                    // red at walls
  const float b = t;                                           // blue in the open
  const float g = 0.55f * (1.0f - std::fabs(2.0f * t - 1.0f)); // green ridge mid-range
  out[0] = static_cast<unsigned char>(r * 255);
  out[1] = static_cast<unsigned char>(g * 255);
  out[2] = static_cast<unsigned char>(b * 255);
  out[3] = static_cast<unsigned char>((0.55f + 0.4f * (1.0f - t)) * 255); // walls opaque
}

// Refill the belief texture from the agent's planning field (plane 0 = phi). The
// image is cols x rows; pixel (c, r) <- phi[r*cols + c]. setTexture already flips V.
// `clip` is the phi value mapped to fully-open (blue) — the SDF's range is scene-
// dependent (a small few, not tens), so it is measured once from the initial field.
void fill_belief(unsigned char *px, const float *field, int rows, int cols, float clip) {
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const float phi = field[static_cast<long>(r) * cols + c]; // plane 0
      phi_to_rgba(phi, clip, px + (static_cast<long>(r) * cols + c) * 4);
    }
}

// A LINES ring (circle) of `radius` at height z, colour rgb — the sensor footprint.
cvc::geometry sensor_ring(double radius, double z, const double rgb[3], int seg = 72) {
  cvc::geometry g;
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &lines = g.lines();
  for (int i = 0; i < seg; ++i) {
    const double a = 2.0 * PI * i / seg;
    pts.push_back({radius * std::cos(a), radius * std::sin(a), z});
    cols.push_back({rgb[0], rgb[1], rgb[2]});
    lines.push_back({static_cast<cvc::geometry::index_t>(i),
                     static_cast<cvc::geometry::index_t>((i + 1) % seg)});
  }
  return g;
}
} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int grid = 120, width = 1280, height = 720;
  long frames = 0;
  double fps = 30.0, hz = 60.0;
  std::string capture = "orbit", out = "frames", png;
  bool offscreen = false, no_shadows = false, ortho = false;

  po::options_description desc("nav_fog_ghost — the fog-of-war ghost story in cvcGL");
  desc.add_options()("help,h", "show this help")("grid", po::value<int>(&grid)->default_value(120),
                                                 "occupancy resolution")(
      "offscreen", po::bool_switch(&offscreen))(
      "ortho", po::bool_switch(&ortho), "top-down 2-D orthographic view (matplotlib-style)")(
      "no-shadows", po::bool_switch(&no_shadows))("frames",
                                                  po::value<long>(&frames)->default_value(0))(
      "fps", po::value<double>(&fps)->default_value(30.0))(
      "hz", po::value<double>(&hz)->default_value(60.0),
      "sim tick rate")("capture", po::value<std::string>(&capture)->default_value("none"),
                       "none (interactive window) | orbit | fly (offscreen PNG capture)")(
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
    std::printf("nav_fog_ghost: capturing %ld frames (%s, offscreen) -> %s/frame_*.png\n", frames,
                capture.c_str(), out.c_str());
  }
  if (ortho)
    no_shadows = true;

  // 1. The ghost scenario. Bounds +/-100, scale 0.05 (the city meta). TRUTH is an
  //    open field inside a border; the stale PRIOR adds a phantom vertical wall the
  //    agent must discover isn't there.
  const int R = grid, C = grid;
  const double HALF = 100.0, SCALE = 0.05;
  const navdemo::Bounds bounds{-HALF, -HALF, HALF, HALF};
  std::vector<std::uint8_t> truth(static_cast<std::size_t>(R) * C, 0), prior;
  auto border = [&](std::vector<std::uint8_t> &m) {
    for (int r = 0; r < R; ++r)
      for (int c = 0; c < C; ++c)
        if (r == 0 || c == 0 || r == R - 1 || c == C - 1)
          m[static_cast<std::size_t>(r) * C + c] = 1;
  };
  border(truth);
  prior = truth; // phantom wall lives only in the belief
  // size_t loop counter + index math: no signed-overflow UB for the optimizer to
  // exploit, so -Waggressive-loop-optimizations stays quiet (grid is a runtime value).
  const std::size_t wall_lo = static_cast<std::size_t>(R) / 4;
  const std::size_t wall_hi = static_cast<std::size_t>(R) * 3 / 4;
  for (std::size_t r = wall_lo; r < wall_hi; ++r)
    prior[r * C + C / 2] = 1; // vertical bar at mid-column

  cvc::nav::sim_world::config cfg;
  cfg.rows = R;
  cfg.cols = C;
  cfg.min_x = -HALF;
  cfg.min_y = -HALF;
  cfg.max_x = HALF;
  cfg.max_y = HALF;
  cfg.cx = 0;
  cfg.cy = 0;
  cfg.scale = SCALE;
  cfg.veh.rr = 0.15f;
  cfg.veh.d_hat = 0.35f;
  cfg.veh.dt = 0.06f;
  cfg.veh.vmax = 0.9f;
  cfg.freeze_sense = false; // FOG: sense + rebuild belief each sense tick
  cfg.sense_every = 2;
  cfg.range_m = 55.0;
  cfg.n_rays = 240;
  cfg.reach_tol = 1.0f;

  // Single agent: start left, goal right — the straight line crosses the phantom bar.
  const float o[2] = {static_cast<float>(-80.0 * SCALE), static_cast<float>(-6.0 * SCALE)};
  const float goal[2] = {static_cast<float>(80.0 * SCALE), static_cast<float>(6.0 * SCALE)};
  const float color[3] = {0.98f, 0.85f, 0.20f}; // the hero vehicle: gold

  cvc::nav::sim_world world(cfg, truth.data(), prior.data(), cvc::nav::coef_mlp::default_biased(),
                            o, goal, color, 1);

  // 2. Scene.
  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "ghost");

  const double wall_rgb[3] = {0.42, 0.44, 0.50};
  sg.addGraphics("border", navdemo::occupancy_to_walls(truth.data(), R, C, bounds, 6.0, wall_rgb));

  // Ground carries the live belief heatmap.
  const double ground_rgb[3] = {1, 1, 1};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", navdemo::ground_quad(bounds, 0.0, ground_rgb)));
  cvc::image belief(C, R, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *bpx = belief.data(); // own the buffer, then alias it via setTexture
  // Measure the SDF's open-space extent once to scale the colourmap (phi maxes out
  // at a small few in normalized units, scene-dependent).
  float belief_clip = 1.5f;
  {
    const float *f = world.field_data();
    for (long i = 0; i < static_cast<long>(R) * C; ++i)
      belief_clip = std::max(belief_clip, f[i]);
  }
  fill_belief(bpx, world.field_data(), R, C, belief_clip);
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    groundNode->setColor(1, 1, 1);
    groundNode->setAmbient(0.9); // the texture IS the story; keep it bright + flat
    groundNode->setDiffuse(0.3);
    groundNode->setTexture(belief, /*zeroCopy=*/true);
  }

  // The agent + its sensor ring.
  navdemo::AgentGlyphs glyph;
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("agent", glyph.build(app, 1, color, 7.0, 1.2)));
  if (agentNode) {
    agentNode->setUseSingleColor(false);
    agentNode->setAmbient(0.7);
    agentNode->setDiffuse(0.8);
  }
  const double ring_rgb[3] = {0.98, 0.9, 0.4};
  auto ringNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("sensor", sensor_ring(cfg.range_m, 0.8, ring_rgb)));
  if (ringNode) {
    ringNode->setRenderMode(GeometryRenderMode::LINES);
    ringNode->setUseSingleColor(true);
    ringNode->setColor(ring_rgb[0], ring_rgb[1], ring_rgb[2]);
    ringNode->setLineWidth(2.0);
  }

  sg.addDirectionalLight(-38, 60, 1.0, 0.97, 0.9, 1.1);
  sg.addDirectionalLight(150, 34, 0.5, 0.58, 0.72, 0.45);

  SceneRenderer view(sg, width, height, offscreen, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(1024);
    sg.setShadowUpdateInterval(capturing ? 1 : 4);
  }
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.12, 0.14, 0.18);
  view.renderer()->SetBackground2(0.30, 0.36, 0.46);

  CameraController cam(view);
  cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, 12.0);
  if (ortho)
    navdemo::set_ortho_topdown(view, bounds, 4.0);
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("nav_fog_ghost: 1 agent, %dx%d fog belief, phantom wall in the prior\n", R, C);

  // 3. Run the sim on the render thread (single agent — cheap) so field_data() and
  //    the pose stay in lock-step with the belief texture we upload.
  const auto t0 = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  double eye[3], focal[3];
  std::vector<float> pos(2), head(1), spd(1);
  std::vector<int> md(1);
  std::vector<std::uint8_t> rch(1);
  bool reached_note = false;
  const int sub = std::max(1, static_cast<int>(hz / std::max(1.0, fps)));

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

    for (int s = 0; s < sub; ++s)
      world.step();

    world.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data());
    const auto &xyz = glyph.pack(pos.data(), head.data());
    if (agentNode)
      agentNode->updateVertices(xyz);
    if (ringNode)
      ringNode->setPosition(pos[0], pos[1], 0.0);
    // Belief evolves as the agent senses — re-upload the heatmap in place.
    fill_belief(bpx, world.field_data(), R, C, belief_clip);
    if (groundNode)
      groundNode->texture_modified();

    if (ortho) {
      // fixed top-down map — camera set once
    } else if (capturing) {
      const double az = -PI / 2 + 0.16 * std::sin(0.2 * t); // gentle look, mostly from the south
      navdemo::orbit_camera(bounds, 6.0, az, 52.0 * PI / 180.0, 2.7, eye, focal);
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
#ifndef __EMSCRIPTEN__
    if (capturing && rch[0] && frame > 30) // reached — hold a beat, then end the capture
      break;
    if (frames > 0 && frame >= frames)
      break;
    if (rch[0] && !reached_note) { // interactive: announce, keep the window open
      std::printf("nav_fog_ghost: goal reached — ghost wall dissolved. Window stays open "
                  "(close it to exit).\n");
      reached_note = true;
    }
#endif
    // Browser build loops forever (the agent parks at the goal); the camera stays live.
  }

  cam.detach();
  if (!png.empty())
    view.writePNG(png);
  std::printf("nav_fog_ghost: done (%ld frames, reached=%d)\n", frame, static_cast<int>(rch[0]));
  return 0;
}

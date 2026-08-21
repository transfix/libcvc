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
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
#include <cvc/gl/TouchGestures.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
#include <cvc/image/image.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h> // SceneGraph.h only forward-declares it
#include <emscripten.h>
#endif
#include <filesystem>
#ifdef CVC_ENABLE_IMGUI
#include <unistd.h>
#endif
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::CameraController;

namespace {
const double PI = std::acos(-1.0);

// The GROUND IS THE FOG — the honest epistemic texture, replacing the old phi
// rainbow (which coloured the whole floor with meaningless halos). Three fog
// tiers per cell: never-seen near-black, remembered dim, currently-visible lit.
// Belief paints on top as an LED grid: WALL red where belief & truth, GHOST
// amber where belief & ~truth (the phantom — a wall the agent believes that
// reality lacks). Cells are 3x3 texels with a 1px dark gutter so belief reads
// as discretized KNOWLEDGE, not world. Pixel (3c+sx, 3r+sy) <- cell (r, c);
// setTexture's V-flip handles orientation.
constexpr int kCellPx = 3;
void fill_epistemic(unsigned char *px, const cvc::nav::sim_world &w, int rows, int cols) {
  const std::uint8_t *seen = w.ever_seen(0), *vis = w.last_visible(0);
  const std::uint8_t *bel = w.belief_occ(0), *tru = w.truth();
  const int W = kCellPx * cols;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const long i = static_cast<long>(r) * cols + c;
      // Fog tier
      unsigned char t[3] = {14, 15, 18}; // never seen: near-black
      if (vis[i]) {
        t[0] = 51;
        t[1] = 56;
        t[2] = 66; // in view right now: lit
      } else if (seen[i]) {
        t[0] = 33;
        t[1] = 36;
        t[2] = 41; // remembered: dim
      }
      // Belief LED
      const bool occ = bel[i] != 0;
      const bool ghost = occ && !tru[i];
      const unsigned char led[3] = {static_cast<unsigned char>(ghost ? 242 : 217),
                                    static_cast<unsigned char>(ghost ? 184 : 64),
                                    static_cast<unsigned char>(ghost ? 64 : 56)};
      for (int sy = 0; sy < kCellPx; ++sy)
        for (int sx = 0; sx < kCellPx; ++sx) {
          unsigned char *p = px + (static_cast<long>(kCellPx * r + sy) * W + kCellPx * c + sx) * 4;
          const bool gutter = (sx == kCellPx - 1) || (sy == kCellPx - 1);
          const unsigned char *col = (occ && !gutter) ? led : t;
          p[0] = col[0];
          p[1] = col[1];
          p[2] = col[2];
          p[3] = 255;
        }
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
  double mouseSens = 0.25, moveSpeed = 0.0; // camera feel (0 move speed = auto from bounds)
  double fps = 30.0, hz = 60.0, speed = 0.5;
  std::string capture = "orbit", out = "frames", png, viewMode = "map", scenario = "ghost";
  int traffic_n = 5;
  double sensorRange = 35.0;
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
      "sim tick rate")("speed", po::value<double>(&speed)->default_value(0.5),
                       "world speed as a multiple of real time (story pace)")(
      "scenario", po::value<std::string>(&scenario)->default_value("ghost"),
      "ghost (stale map lies) | dynamic (world changes mid-run) | traffic (ghost + cross traffic)")(
      "range", po::value<double>(&sensorRange)->default_value(35.0),
      "sensor range in metres")("traffic", po::value<int>(&traffic_n)->default_value(5),
                                "number of cross-traffic vehicles in the traffic scenario")(
      "view", po::value<std::string>(&viewMode)->default_value("map"),
      "map (top-down 2-D, the honest baseline) | 3d (perspective orbit)")(
      "capture", po::value<std::string>(&capture)->default_value("none"),
      "none (interactive window) | orbit | fly (offscreen PNG capture)")(
      "mouse-sensitivity", po::value<double>(&mouseSens)->default_value(0.25),
      "look speed, degrees per pixel of mouse motion")(
      "move-speed", po::value<double>(&moveSpeed)->default_value(0.0),
      "fly speed in world units/s (0 = auto from scene bounds)")(
      "width", po::value<int>(&width)->default_value(1280))(
      "height", po::value<int>(&height)->default_value(720))(
      "out", po::value<std::string>(&out)->default_value("frames"))("png",
                                                                    po::value<std::string>(&png));
  bool no_ui = false;
  desc.add_options()("no-ui", po::bool_switch(&no_ui),
                     "hide the ImGui overlay (default: hidden while capturing)");
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
  // The fixed top-down map is the honest DEFAULT view for the story (the 2-D
  // demos' framing); --view 3d opts into the perspective orbit. Captures keep
  // their scripted 3-D camera unless --ortho forces the map.
  if (!ortho && !capturing)
    ortho = (viewMode == "map");
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

  // THREE limited-belief scenarios, all the same machinery, different lies:
  //
  //   ghost   — the map claims a wall that reality lacks. The agent detours
  //             around nothing, senses the space is clear, and drives through.
  //   dynamic — the map is RIGHT at first, then the world CHANGES: a blockage
  //             appears mid-run (stamped into the live planning surface with
  //             sim_world::add_obstacle) and the agent must notice and re-route.
  //   traffic — the ghost lie, but with other vehicles crossing the same space,
  //             so belief-vs-truth plays out against moving obstacles.
  //
  // The lie always lives in the BELIEF (prior_occ), never in `truth` — collision
  // is always scored against truth, which is what keeps the demo honest.
  const std::size_t wall_lo = static_cast<std::size_t>(R) / 4;
  const std::size_t wall_hi = static_cast<std::size_t>(R) * 3 / 4;
  auto phantom_bar = [&](std::vector<std::uint8_t> &m) {
    for (std::size_t r = wall_lo; r < wall_hi; ++r)
      m[r * C + C / 2] = 1; // vertical bar at mid-column
  };
  prior = truth;
  if (scenario != "dynamic") // dynamic starts with an HONEST map
    phantom_bar(prior);

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
  cfg.range_m = sensorRange; // reads as a SENSOR, not a hoop spanning half the map
  cfg.n_rays = 240;
  cfg.reach_tol = 1.0f;

  // Agent 0 is always the HERO (gold, left -> right through the lie). `traffic`
  // adds cross-traffic that shares the hero's belief plane, so the peers are part
  // of the world the hero is reasoning about.
  const int NA = (scenario == "traffic") ? traffic_n + 1 : 1;
  std::vector<float> o(2 * NA), goal(2 * NA), color(3 * NA);
  o[0] = static_cast<float>(-80.0 * SCALE);
  o[1] = static_cast<float>(-6.0 * SCALE);
  goal[0] = static_cast<float>(80.0 * SCALE);
  goal[1] = static_cast<float>(6.0 * SCALE);
  color[0] = 0.98f;
  color[1] = 0.85f;
  color[2] = 0.20f; // hero: gold
  for (int i = 1; i < NA; ++i) {
    // Cross traffic: alternate top->bottom and bottom->top on staggered columns,
    // so they sweep THROUGH the hero's corridor rather than trailing it.
    const double frac = static_cast<double>(i) / (NA > 1 ? NA : 1);
    const double x = -60.0 + 120.0 * frac;
    const bool down = (i % 2) == 0;
    o[2 * i] = static_cast<float>(x * SCALE);
    o[2 * i + 1] = static_cast<float>((down ? 78.0 : -78.0) * SCALE);
    goal[2 * i] = static_cast<float>(x * SCALE);
    goal[2 * i + 1] = static_cast<float>((down ? -78.0 : 78.0) * SCALE);
    color[3 * i] = 0.35f;
    color[3 * i + 1] = 0.62f + 0.3f * static_cast<float>(frac);
    color[3 * i + 2] = 0.95f; // traffic: cool blue
  }

  cvc::nav::sim_world world(cfg, truth.data(), prior.data(), cvc::nav::coef_mlp::default_biased(),
                            o.data(), goal.data(), color.data(), NA);

  // 2. Scene.
  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "ghost");

  const double wall_rgb[3] = {0.42, 0.44, 0.50};
  sg.addGraphics("border", navdemo::occupancy_to_walls(truth.data(), R, C, bounds, 6.0, wall_rgb));

  // Ground carries the live EPISTEMIC texture: fog tiers + the belief LED grid.
  const double ground_rgb[3] = {1, 1, 1};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", navdemo::ground_quad(bounds, 0.0, ground_rgb)));
  cvc::image belief(kCellPx * C, kCellPx * R, cvc::image::pixel_format::RGBA,
                    cvc::image::data_type::u8);
  unsigned char *bpx = belief.data(); // own the buffer, then alias it via setTexture
  fill_epistemic(bpx, world, R, C);
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    groundNode->setColor(1, 1, 1);
    groundNode->setAmbient(1.0); // the texture IS the story; flat, never shaded
    groundNode->setDiffuse(0.0);
    groundNode->setTexture(belief, /*zeroCopy=*/true);
  }

  // The GHOST itself, standing in the world: a translucent amber wall extruded
  // from the phantom cells (belief & ~truth). It ERODES cell by cell as the
  // sensor clears them — the premise finally has a 3-D body, and its dissolution
  // is the story beat.
  const double ghost_rgb[3] = {0.95, 0.72, 0.25};
  auto ghost_cells = [&]() {
    std::vector<std::uint8_t> g(static_cast<std::size_t>(R) * C, 0);
    const std::uint8_t *bel = world.belief_occ(0), *tru = world.truth();
    long n = 0;
    for (long i = 0; i < static_cast<long>(R) * C; ++i)
      if (bel[i] && !tru[i]) {
        g[i] = 1;
        ++n;
      }
    return std::make_pair(g, n);
  };
  auto [ghostOcc, ghostCount] = ghost_cells();
  const long initialGhost = ghostCount;
  auto ghostNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(
      "ghost", navdemo::occupancy_to_walls(ghostOcc.data(), R, C, bounds, 6.0, ghost_rgb)));
  if (ghostNode) {
    ghostNode->setUseSingleColor(false);
    ghostNode->setAmbient(0.9);
    ghostNode->setDiffuse(0.2);
    ghostNode->setOpacity(0.45); // visibly a wall, visibly not solid
  }
  int lastFv = world.field_version();
  long lastPaint = -1000;

  // The agent + its sensor ring.
  navdemo::AgentGlyphs glyph;
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("agent", glyph.build(app, NA, color.data(), 7.0, 1.2)));
  if (agentNode) {
    agentNode->setUseSingleColor(false);
    agentNode->setAmbient(0.7);
    agentNode->setDiffuse(0.8);
  }
  const double ring_rgb[3] = {0.30, 0.72, 0.95}; // FOV cyan (the 2-D demos' palette)
  auto ringNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("sensor", sensor_ring(cfg.range_m, 0.8, ring_rgb)));
  if (ringNode) {
    ringNode->setRenderMode(GeometryRenderMode::LINES);
    ringNode->setUseSingleColor(true);
    ringNode->setColor(ring_rgb[0], ring_rgb[1], ring_rgb[2]);
    ringNode->setLineWidth(2.0);
    ringNode->setRenderLinesAsTubes(true);
    ringNode->setAmbient(1.0); // flat overlay colour — never shaded to black
    ringNode->setDiffuse(0.0);
  }

  // Intent + history made visible (the 2-D demos' core vocabulary): an orange
  // goal pyramid + green start pad, a thick blue PLAN line (agent -> carrot ->
  // goal — the carrot IS the live plan, and it visibly bends around the phantom),
  // and a growing yellow TRACK trail of where the agent has actually driven.
  const double goal_rgb[3] = {1.00, 0.45, 0.20}, start_rgb[3] = {0.35, 0.85, 0.45};
  const double route_rgb[3] = {0.35, 0.65, 1.00}, track_rgb[3] = {0.98, 0.85, 0.30};
  const double gx = 80.0, gy = 6.0, sx = -80.0, sy = -6.0; // world (set above, normalized)
  {
    auto goalNode = std::dynamic_pointer_cast<GeometryNode>(
        sg.addGraphics("goal", navdemo::pyramid_marker(4.5, 7.0, goal_rgb)));
    if (goalNode) {
      goalNode->setUseSingleColor(false);
      goalNode->setAmbient(0.85);
      goalNode->setDiffuse(0.6);
      goalNode->setLabelText("GOAL");
      goalNode->setShowLabel(true);
      goalNode->setPosition(gx, gy, 9.0); // hovering, apex pointing at the spot
    }
    auto startNode = std::dynamic_pointer_cast<GeometryNode>(
        sg.addGraphics("start", navdemo::disc_marker(3.5, 0.25, start_rgb)));
    if (startNode) {
      startNode->setUseSingleColor(false);
      startNode->setAmbient(0.9);
      startNode->setDiffuse(0.3);
      startNode->setPosition(sx, sy, 0.0);
    }
  }
  // Plan line: 2 preallocated segments (agent->carrot, carrot->goal), streamed.
  auto make_lines = [&](const char *name, int segs, const double rgb[3], double width, double x0,
                        double y0, double z0) {
    cvc::geometry g;
    for (int s = 0; s < segs; ++s) {
      for (int e = 0; e < 2; ++e) {
        g.points().push_back({x0, y0, z0});
        g.colors().push_back({rgb[0], rgb[1], rgb[2]});
      }
      g.lines().push_back({static_cast<cvc::geometry::index_t>(2 * s),
                           static_cast<cvc::geometry::index_t>(2 * s + 1)});
    }
    auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(name, g));
    if (node) {
      node->setRenderMode(GeometryRenderMode::LINES);
      node->setUseSingleColor(true);
      node->setColor(rgb[0], rgb[1], rgb[2]);
      node->setLineWidth(width);
      node->setRenderLinesAsTubes(true);
      node->setDepthOffset(2.0); // decal-style: never z-fight the ground texture
      node->setAmbient(1.0);     // flat overlay colour — never shaded to black
      node->setDiffuse(0.0);
    }
    return node;
  };
  auto planNode = make_lines("plan", 2, route_rgb, 5.0, sx, sy, 0.7);
  const int TRAIL_SEGS = 600;
  auto trailNode = make_lines("track", TRAIL_SEGS, track_rgb, 3.0, sx, sy, 0.55);
  std::vector<double> planXyz(static_cast<std::size_t>(3) * 4, 0.0);
  std::vector<double> trailXyz(static_cast<std::size_t>(3) * 2 * TRAIL_SEGS, 0.0);
  for (int s = 0; s < 2 * TRAIL_SEGS; ++s) { // all degenerate at the start pad
    trailXyz[3 * s] = sx;
    trailXyz[3 * s + 1] = sy;
    trailXyz[3 * s + 2] = 0.55;
  }
  int trailLen = 0; // segments used so far
  float trailLast[2] = {static_cast<float>(sx), static_cast<float>(sy)};

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
  cam.setMouseSensitivity(mouseSens); // --mouse-sensitivity / state settings.mouse_sensitivity
  if (moveSpeed > 0.0)
    cam.setMoveSpeed(moveSpeed);
  if (ortho)
    navdemo::set_ortho_topdown(view, bounds, 4.0, capturing ? nullptr : &cam);

  // On-screen story: a lower-third caption band + a corner status line. Every
  // claim goes on screen, not stdout.
  cvc::gl::ScreenTextHud caption(view);
  caption.setFontSize(19);
  cvc::gl::ScreenTextHud status(view);
  status.setCentered(false);
  status.setPosition(0.015, 0.95);
  status.setFontSize(13);
  status.setColor(0.72, 0.78, 0.85);
  {
    char st[160];
    std::snprintf(st, sizeof st,
                  "fog-of-war ghost · cvc::nav reactive drive (learned CoefMLP) · belief %dx%d · "
                  "speed %.2gx",
                  R, C, speed);
    status.setText(st);
  }
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("nav_fog_ghost: scenario=%s, %d vehicle%s, %dx%d belief, %s\n", scenario.c_str(), NA,
              NA == 1 ? "" : "s", R, C,
              scenario == "dynamic" ? "honest map that changes mid-run"
                                    : "phantom wall in the prior");

  // 3. Run the sim on the render thread (single agent — cheap) so field_data() and
  //    the pose stay in lock-step with the belief texture we upload.
  const auto t0 = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  double eye[3], focal[3];
  std::vector<float> pos(2 * NA), head(NA), spd(NA);
  std::vector<int> md(NA);
  std::vector<std::uint8_t> rch(NA);
  bool reached_note = false;
  // ---------------- ImGui control panel -------------------------------------
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  // Phones have no mouse: pinch = zoom, two-finger drag = pan/turn.
  cvc::gl::TouchGestures touch(view, cam);
  // A capture is a deliverable: no control panel in the frames unless asked.
  ui.setVisible(!no_ui && !capturing);
  bool uiPaused = false, uiRestart = false, ui2D = ortho, uiCaptions = true;
  std::string uiScenario = scenario;
  int uiTraffic = traffic_n;
  double uiSpeed = speed, uiRange = cfg.range_m;
#ifdef CVC_ENABLE_IMGUI
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Scenario")) {
        const char *sc[] = {"ghost", "dynamic", "traffic"};
        for (const char *n : sc)
          if (ImGui::MenuItem(n, nullptr, uiScenario == n)) {
            uiScenario = n;
            uiRestart = true;
          }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Sim")) {
        ImGui::MenuItem("Paused", nullptr, &uiPaused);
        if (ImGui::MenuItem("Restart"))
          uiRestart = true;
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("2-D map", nullptr, &ui2D);
        ImGui::MenuItem("Captions", nullptr, &uiCaptions);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Camera")) {
        namespace u = cvc::gl::ui;
        const std::string cp = sg.getStatePrefix() + ".viewers.main.camera.settings.";
        u::SliderDouble(app, "Look sens", cp + "mouse_sensitivity", 0.02, 2.0, 0.25);
        u::SliderDouble(app, "Move speed", cp + "move_speed", 1.0, 400.0, 40.0);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Belief controls");
    ImGui::Text("scenario: %s", uiScenario.c_str());
    ImGui::Text("world t %.1fs | %s", world.tick() * cfg.veh.dt, rch[0] ? "ARRIVED" : "en route");
    ImGui::Separator();
    {
      float f = static_cast<float>(uiSpeed);
      if (ImGui::SliderFloat("world speed", &f, 0.1f, 3.0f, "%.2fx"))
        uiSpeed = f;
      float r = static_cast<float>(uiRange);
      if (ImGui::SliderFloat("sensor range", &r, 10.0f, 90.0f, "%.0f m"))
        uiRange = r; // applies on restart
      ImGui::SliderInt("traffic", &uiTraffic, 1, 16);
    }
    if (ImGui::Button("Apply / Restart", ImVec2(-1, 0)))
      uiRestart = true;
    ImGui::TextDisabled("scenario / range / traffic need a restart");
    ImGui::End();
  });
#endif

  std::vector<std::string> restartArgs;
  navdemo::SimPacer pacer;
  std::vector<float> cwv(2 * NA, 0.0f); // live carrots (world); [0] = hero
  double ghostGoneT = -1.0;             // world time the last phantom cell cleared
  bool blockDropped = false;
  double blockT = -1.0;

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
    touch.update(); // apply any pinch/two-finger gesture from this frame

#ifdef CVC_ENABLE_IMGUI
    // ---- apply UI actions ---------------------------------------------------
    speed = uiSpeed; // world pace: live
    caption.setVisible(uiCaptions);
    if (ui2D != ortho) { // 2-D map <-> 3-D perspective, live
      ortho = ui2D;
      if (ortho)
        navdemo::set_ortho_topdown(view, bounds, 4.0, &cam);
      else {
        cam.setMode(CameraController::Mode::Orbit);
        cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, 12.0);
      }
    }
    if (uiRestart) {
      // The scenario decides the agent COUNT, the prior map and the whole scene
      // graph, so a scenario change is a fresh process rather than a surgical
      // rebuild — the honest way to guarantee no stale belief/mesh survives.
      uiRestart = false;
      restartArgs = {"--scenario", uiScenario,          "--traffic", std::to_string(uiTraffic),
                     "--view",     ui2D ? "map" : "3d", "--speed",   std::to_string(uiSpeed)};
      // sensor range is a config field; carry it too.
      restartArgs.push_back("--range");
      restartArgs.push_back(std::to_string(uiRange));
      break; // leave the loop; main() re-execs below
    }
    if (uiPaused) {
      view.render();
      continue; // frozen world, live camera + UI
    }
#endif

    // World time runs at `speed` x real time on ANY display rate (fixed-dt
    // accumulator); a capture uses the same pacer with a synthetic 1/fps wall
    // clock, so captured world pacing matches the live window deterministically.
    const int nticks = pacer.ticks(dt, cfg.veh.dt, speed);
    for (int s = 0; s < nticks; ++s)
      world.step();

    world.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data());
    const auto &xyz = glyph.pack(pos.data(), head.data());
    if (agentNode)
      agentNode->updateVertices(xyz);
    if (ringNode)
      ringNode->setPosition(pos[0], pos[1], 0.0);

    // The PLAN, live: agent -> carrot -> goal. The carrot is the FSM's steering
    // target — the blue line visibly bows around the phantom while the agent
    // believes in it, then snaps straight when the belief clears.
    world.carrots_world(cwv.data());
    if (planNode) {
      planXyz[0] = pos[0];
      planXyz[1] = pos[1];
      planXyz[2] = 0.7;
      planXyz[3] = planXyz[6] = cwv[0];
      planXyz[4] = planXyz[7] = cwv[1];
      planXyz[5] = planXyz[8] = 0.7;
      planXyz[9] = gx;
      planXyz[10] = gy;
      planXyz[11] = 0.7;
      planNode->updateVertices(planXyz);
    }
    // The TRACK, growing: append a trail segment every 3rd frame once the agent
    // has actually moved; the yellow history stays behind the vehicle over the
    // blue intent, exactly the 2-D demos' plan-vs-track contrast.
    if (trailNode && frame % 3 == 0 && trailLen < TRAIL_SEGS) {
      const float mdx = pos[0] - trailLast[0], mdy = pos[1] - trailLast[1];
      if (mdx * mdx + mdy * mdy > 0.35f) {
        const std::size_t b = static_cast<std::size_t>(6) * trailLen;
        trailXyz[b] = trailLast[0];
        trailXyz[b + 1] = trailLast[1];
        trailXyz[b + 2] = 0.55;
        trailXyz[b + 3] = pos[0];
        trailXyz[b + 4] = pos[1];
        trailXyz[b + 5] = 0.55;
        trailLast[0] = pos[0];
        trailLast[1] = pos[1];
        ++trailLen;
        trailNode->updateVertices(trailXyz);
      }
    }

    const double worldT = world.tick() * cfg.veh.dt;

    // `dynamic`: the world CHANGES under the agent. A few seconds in, a blockage is
    // stamped across the corridor ahead of it (sim_world::add_obstacle writes the
    // live dynamic layer, which is composited into the planning surface on the
    // next SENSE tick — so the agent only "knows" once it senses, exactly like a
    // real discovery). The stale-map lie and this are the two halves of limited
    // belief: believing something that isn't there, and not yet knowing
    // something that is.
    if (scenario == "dynamic" && !blockDropped && worldT > 3.5) {
      // add_obstacle stamps a CENTRED RADIUS over the cell rect, so it grows
      // well beyond the literal rect — keep the rect thin and short or it swamps
      // the map. It lands in the belief (truth is immutable from here), so the
      // renderer draws it in GHOST amber: honest, since from the agent's side a
      // new blockage and a believed wall are the same evidence.
      const int rmid = static_cast<int>((wall_lo + wall_hi) / 2);
      const int r0 = rmid - C / 12, r1 = rmid + C / 12;
      const int c0 = C / 2, c1 = C / 2;
      world.add_obstacle(r0, r1, c0, c1);
      blockDropped = true;
      blockT = worldT;
    }

    // Repaint the fog/belief ground only when a sense tick actually happened —
    // per-frame smoothing of belief would be a lie (and wasted work).
    if (world.tick() - lastPaint >= cfg.sense_every) {
      fill_epistemic(bpx, world, R, C);
      if (groundNode)
        groundNode->texture_modified();
      lastPaint = world.tick();
    }

    // Ghost erosion: on a belief flip (field-version bump), rebuild the amber
    // wall from the surviving phantom cells; hide it when the last one clears.
    // The visible pop at each rebuild IS the story beat.
    if (world.field_version() != lastFv) {
      lastFv = world.field_version();
      auto [g2, n2] = ghost_cells();
      if (n2 != ghostCount && ghostNode) {
        ghostCount = n2;
        // The discovery beat fires when the corridor BREAKS (half the phantom
        // eroded) — the stubs beyond sensor range may honestly never clear.
        if (ghostGoneT < 0.0 && n2 < initialGhost / 2)
          ghostGoneT = worldT;
        if (n2 == 0)
          ghostNode->setVisible(false);
        else
          ghostNode->setGeometry(
              navdemo::occupancy_to_walls(g2.data(), R, C, bounds, 6.0, ghost_rgb));
        ghostOcc = std::move(g2);
      }
    }

    // The caption arc — scenario-specific, driven by world time + events.
    if (scenario == "dynamic") {
      if (rch[0])
        caption.setText("Re-routed around a blockage that appeared after it set off.");
      else if (blockDropped && worldT - blockT < 5.0)
        caption.setText("THE WORLD CHANGED: the way ahead just closed.");
      else if (blockDropped)
        caption.setText("");
      else
        caption.setText("The map is honest — for now.");
    } else if (rch[0])
      caption.setText("Straight to the goal — through a wall that was never there.");
    else if (ghostGoneT >= 0.0 && worldT - ghostGoneT < 4.0)
      caption.setText("SENSOR: nothing there. Replanned.");
    else if (ghostGoneT >= 0.0)
      caption.setText("");
    else if (worldT > 5.0)
      caption.setText("Belief is all it has — so it detours.");
    else
      caption.setText("MAP: wall ahead. TRUTH: open field.");

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

#ifdef CVC_ENABLE_IMGUI
  if (!restartArgs.empty()) {
    // Relaunch with the UI's settings. Everything above (window, scene graph,
    // belief planes, glyph meshes) is torn down by leaving main, so the new
    // scenario starts from a genuinely clean slate — no stale belief, no
    // mismatched mesh sizes.
    std::vector<char *> av;
    av.push_back(argv[0]);
    for (auto &a : restartArgs)
      av.push_back(const_cast<char *>(a.c_str()));
    av.push_back(nullptr);
    std::fflush(nullptr);
    execv("/proc/self/exe", av.data());
    std::perror("nav_fog_ghost: restart exec failed"); // falls through to exit
  }
#endif
  return 0;
}

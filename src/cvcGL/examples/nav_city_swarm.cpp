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
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
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
  bool fogOn = !no_fog; // fog by default: belief modes only MEAN something with sensing
  (void)fog;            // legacy switch, absorbed by the fog-on default
  cfg.freeze_sense = !fogOn;
  cfg.reach_tol = 0.8f;
  // Inter-agent separation: without it the vehicles pass straight through each
  // other. A reactive carrot nudge away from peers within ~3 vehicle radii makes
  // the swarm steer AROUND itself. Live-tunable below; sep_gain 0 turns it off.
  cfg.sep_radius = 3.0f * static_cast<float>(ts.rr);
  cfg.sep_gain = 1.5f * static_cast<float>(ts.rr);

  // The sim lives behind a pointer so the UI can REBUILD it: agent count and
  // belief mode are baked into sim_world at construction, so "restart with 2000
  // agents / private belief" means constructing a new world, not mutating one.
  unsigned simSeed = 7;
  auto build_world = [&](int nAgents, const std::string &beliefMode, bool fogOnNow, unsigned seed) {
    auto m = cvc::nav::sim_world::belief_mode::shared;
    int k = 1;
    if (beliefMode == "private")
      m = cvc::nav::sim_world::belief_mode::private_belief;
    else if (beliefMode == "grouped") {
      m = cvc::nav::sim_world::belief_mode::clustered;
      k = std::max(2, nAgents / 64);
    }
    cfg.freeze_sense = !fogOnNow;
    return std::make_unique<cvc::nav::sim_world>(cvc::nav::sim_world::from_occupancy(
        cfg, ts.occ.data(), cvc::nav::coef_mlp::default_biased(), nAgents, seed, m, k));
  };
  auto worldPtr = build_world(agents, belief, !no_fog, simSeed);
  cvc::nav::sim_world &world = *worldPtr;
  int N = world.size();

  // 2. Per-agent glyph colours: by belief group (grouped/private) so the grouping is
  //    visible; shared -> a rainbow by index.
  const int *grp = world.agent_planes();
  int M = world.planes();
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
    // A tall, slender BEACON per goal: apex on the ground (points at the exact
    // spot) widening up to a small cap ABOVE the rooftops (buildings are
    // wall_h = 0.06*span tall). The old 0.006*span pyramid capped at 0.022*span
    // sat entirely below the skyline, so targets were invisible in the city.
    const double gh = 0.010 * span; // cap half-width
    const double ght = 2.2 * wall_h; // beacon height (~0.13*span): clears rooftops
    const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                    ght, gh, gh, ght, -gh, gh,  ght};
    const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 1, 3, 2, 1, 4, 3};
    cvc::geometry goalGeom =
        goalGlyphs.build_template(app, N, color.data(), pv, pt, /*z=*/0.0); // apex on the ground
    goalsNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("goals", goalGeom));
    std::vector<float> gw(static_cast<std::size_t>(2) * N), zeroHead(N, 0.0f);
    world.goals_world(gw.data());
    goalXyz = goalGlyphs.pack(gw.data(), zeroHead.data()); // keep: arrived goals sink away
    if (goalsNode) {
      goalsNode->setUseSingleColor(false);
      goalsNode->setAmbient(1.0); // emissive: the beacon glows in its agent's hue
      goalsNode->setDiffuse(0.35);
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

  // Aimed STAGE RIG instead of two scene-spanning directional lights. A
  // directional light's shadow map is baked over the whole scene bbox (ground +
  // building height + air), so texels are wasted and vehicle shadows smear onto
  // rooftops; a rig of aimed spots bakes a perspective map that lands texels on
  // the city — the fix for the swarm demo's shadow artifacts, and the same live
  // StageLightingPanel UI as lsystem_forest (see navdemo::make_stage_rig).
  auto rig = navdemo::make_stage_rig(sg, bounds, wall_h);

  SceneRenderer view(sg, width, height, offscreen, "main");
  // Shadows must be enabled AFTER the renderer exists (they attach to its passes).
  bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 1 : 3);
  }
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.28, 0.40, 0.58);
  view.renderer()->SetBackground2(0.66, 0.78, 0.92);

  cvc::gl::ScreenTextHud status(view, "status"); // corner status line — claims go on screen
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
    navdemo::set_ortho_topdown(view, bounds, 4.0,
                               capturing ? nullptr : &cam); // fixed 2-D top-down map
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("nav_city_swarm: %d agents, belief=%s (M=%d), grid=%d, %s, shadows=%s\n", N,
              belief.c_str(), M, grid, fogOn ? "fog" : "static-map", shadows ? "on" : "off");

  // 4. Run: sim off-thread; render loop streams poses into the merged glyph mesh.
#if CVC_NAV_DEMO_SIM_WORKER
  auto sim = std::make_unique<cvc::nav::sim_thread>(world, hz);
  sim->start();
#else
  // Non-threaded browser build: no worker available, because a wasm thread needs
  // SharedArrayBuffer and the browser only grants that on a cross-origin-isolated
  // page (GitHub Pages cannot send COOP/COEP, so the gallery build lands here).
  // Step the world inline each frame and read its world-metre snapshot directly —
  // the same call sim_thread makes. Build with -DCVC_WASM_PTHREADS=ON, and serve
  // the page cross-origin isolated, to get the worker branch above instead.
  std::vector<float> emPos(static_cast<std::size_t>(N) * 2), emHead(N);
  bool simPausedEm = false;
  std::vector<int> emMd(N);
  std::vector<std::uint8_t> emRch(N);
  (void)hz;
#endif

  std::vector<unsigned char> rgbBuf; // agent state-restyle scratch
  long arrived = 0;

  // ---------------- ImGui control panel -------------------------------------
  // Everything the demo can do, live: parameters, view mode, camera feel, and a
  // real RESTART (agent count and belief mode are baked into sim_world at
  // construction, so changing them rebuilds the world and the N-sized meshes).
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  // Phones have no mouse: pinch = zoom, two-finger drag = pan/turn.
  cvc::gl::TouchGestures touch(view, cam);
  // A capture is a deliverable: no control panel in the frames unless asked.
  ui.setVisible(!no_ui && !capturing);
  bool uiPaused = false, uiStep = false, uiRestart = false, ui2D = ortho;
  int uiAgents = N;
  std::string uiBelief = belief;
  bool uiFog = fogOn, uiShadows = shadows, uiTrails = true, uiGoals = true;
  double uiHz = hz;
  float uiTrailW = 1.5f;                              // live path (trail) line width
  bool uiSeparate = true;                             // inter-agent avoidance on/off (live)
  float uiSepGain = 1.5f * static_cast<float>(ts.rr); // avoidance strength (live)
  bool uiLighting = false;                            // the StageLightingPanel window
#ifdef CVC_ENABLE_IMGUI
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Sim")) {
        ImGui::MenuItem("Paused", "Space", &uiPaused);
        if (ImGui::MenuItem("Step one frame", nullptr, false, uiPaused))
          uiStep = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Restart"))
          uiRestart = true;
        if (ImGui::MenuItem("Restart (new seed)")) {
          ++simSeed;
          uiRestart = true;
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("2-D map", nullptr, &ui2D);
        ImGui::MenuItem("Trails", nullptr, &uiTrails);
        ImGui::MenuItem("Goals", nullptr, &uiGoals);
        ImGui::MenuItem("Shadows", nullptr, &uiShadows);
        ImGui::MenuItem("Stage lighting", nullptr, &uiLighting);
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
    ImGui::Begin("Swarm controls");
    ImGui::Text("%d agents | %ld arrived | %.0f fps", N, arrived, ImGui::GetIO().Framerate);
    ImGui::Separator();
    ImGui::SliderInt("agents", &uiAgents, 32, 4000);
    const char *modes[] = {"shared", "grouped", "private"};
    int bi = (uiBelief == "private") ? 2 : (uiBelief == "grouped" ? 1 : 0);
    if (ImGui::Combo("belief", &bi, modes, 3))
      uiBelief = modes[bi];
    ImGui::Checkbox("fog (sensing)", &uiFog);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip(
          "Off = every belief plane equals the known map, so belief modes only recolour.");
    {
      float hzf = static_cast<float>(uiHz);
      if (ImGui::SliderFloat("sim Hz", &hzf, 10.0f, 240.0f, "%.0f"))
        uiHz = hzf;
    }
    // ---- Display (all live, no restart) -------------------------------------
    ImGui::Separator();
    ImGui::TextDisabled("Display (live)");
    ImGui::Checkbox("Paths", &uiTrails); // the driven breadcrumb trails
    ImGui::SameLine();
    ImGui::Checkbox("Targets", &uiGoals);
    ImGui::SameLine();
    ImGui::Checkbox("Shadows", &uiShadows);
    ImGui::SliderFloat("path width", &uiTrailW, 0.5f, 6.0f, "%.1f");
    ImGui::Checkbox("Avoid others", &uiSeparate); // inter-agent separation (live)
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Steer around neighbours instead of driving through them.");
    if (uiSeparate) {
      float g = uiSepGain / static_cast<float>(ts.rr); // shown as multiples of a vehicle radius
      if (ImGui::SliderFloat("avoid strength", &g, 0.0f, 5.0f, "%.1f x rr"))
        uiSepGain = g * static_cast<float>(ts.rr);
    }
    ImGui::Separator();
    if (ImGui::Button("Apply / Restart", ImVec2(-1, 0)))
      uiRestart = true;
    ImGui::TextDisabled("agents / belief / fog need a restart");
    ImGui::End();

    // The library lighting panel — the same control surface as lsystem_forest,
    // driving this scene's StageLighting rig (key/fill/back/wash, shadows,
    // gizmos to debug the shadow cones).
    cvc::gl::ui::StageLightingPanel(*rig, &uiLighting);
  });
#endif

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
    touch.update(); // apply any pinch/two-finger gesture from this frame

#ifdef CVC_ENABLE_IMGUI
    // ---- apply UI actions ---------------------------------------------------
    if (uiShadows != shadows) { // live toggles
      shadows = sg.setShadowsEnabled(uiShadows) && uiShadows;
      uiShadows = shadows;
    }
    if (trailNode) {
      trailNode->setVisible(uiTrails);
      trailNode->setLineWidth(uiTrailW);
    }
    if (goalsNode)
      goalsNode->setVisible(uiGoals);
    // Inter-agent avoidance: live-tuned each frame (0 gain = off, vehicles pass
    // through each other again).
    world.set_separation(uiSeparate ? cfg.sep_radius : 0.0f, uiSeparate ? uiSepGain : 0.0f);
    if (ui2D != ortho) { // 2-D map <-> 3-D perspective, live
      ortho = ui2D;
      if (ortho)
        navdemo::set_ortho_topdown(view, bounds, 4.0, &cam);
      else {
        cam.setMode(CameraController::Mode::Orbit);
        cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, wall_h);
      }
    }
    if (uiRestart) {
      // Agent count / belief mode / fog are constructor-time properties of
      // sim_world, so a restart rebuilds the world and every N-sized mesh.
      uiRestart = false;
#if CVC_NAV_DEMO_SIM_WORKER
      sim->stop();
      sim.reset(); // join before the world it borrows changes underneath it
#endif
      fogOn = uiFog;
      belief = uiBelief;
      *worldPtr = std::move(*build_world(uiAgents, uiBelief, uiFog, simSeed));
      N = world.size();
      grp = world.agent_planes();
      M = world.planes();
      color.assign(3 * N, 0.0f);
      for (int i = 0; i < N; ++i) {
        const double hue = (M > 1) ? static_cast<double>(grp[i]) / M : static_cast<double>(i) / N;
        hsv2rgb(hue, 0.62, 0.96, &color[3 * i]);
      }
      if (agentNode)
        agentNode->setGeometry(glyphs.build(app, N, color.data(), gsz, 0.6));
      { // goals + trails are N-sized too
        std::vector<float> gw(static_cast<std::size_t>(2) * N), zeroHead(N, 0.0f);
        world.goals_world(gw.data());
        const double gh = 0.010 * span, ght = 2.2 * wall_h; // beacon (matches initial build)
        const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                        ght, gh, gh, ght, -gh, gh,  ght};
        const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4,
                                               0, 4, 1, 1, 3, 2, 1, 4, 3};
        if (goalsNode) {
          goalsNode->setGeometry(
              goalGlyphs.build_template(app, N, color.data(), pv, pt, 0.0));
          goalXyz = goalGlyphs.pack(gw.data(), zeroHead.data());
          goalsNode->updateVertices(goalXyz);
        }
        trailLast.assign(static_cast<std::size_t>(2) * N, 0.0f);
        world.snapshot(trailLast.data(), nullptr, nullptr, nullptr, nullptr);
        trailWr.assign(N, 0);
        trailXyz.assign(static_cast<std::size_t>(3) * 2 * TRAIL_K * N, 0.0);
        cvc::geometry tg;
        for (int i = 0; i < N; ++i)
          for (int k = 0; k < TRAIL_K; ++k)
            for (int e = 0; e < 2; ++e) {
              const std::size_t v = (static_cast<std::size_t>(i) * TRAIL_K + k) * 2 + e;
              tg.points().push_back({trailLast[2 * i], trailLast[2 * i + 1], 0.35});
              tg.colors().push_back(
                  {0.45 * color[3 * i], 0.45 * color[3 * i + 1], 0.45 * color[3 * i + 2]});
              trailXyz[3 * v] = trailLast[2 * i];
              trailXyz[3 * v + 1] = trailLast[2 * i + 1];
              trailXyz[3 * v + 2] = 0.35;
              if (e == 1)
                tg.lines().push_back({static_cast<cvc::geometry::index_t>(v - 1),
                                      static_cast<cvc::geometry::index_t>(v)});
            }
        if (trailNode)
          trailNode->setGeometry(tg);
      }
      arrived = 0;
      frame = 0;
#if CVC_NAV_DEMO_SIM_WORKER
      sim = std::make_unique<cvc::nav::sim_thread>(world, uiHz);
      sim->start();
#endif
      std::printf("nav_city_swarm: restarted (%d agents, belief=%s, fog=%s, seed=%u)\n", N,
                  belief.c_str(), fogOn ? "on" : "off", simSeed);
    }
#if CVC_NAV_DEMO_SIM_WORKER
    if (sim)
      sim->set_paused(uiPaused && !uiStep);
#else
    simPausedEm = uiPaused && !uiStep; // browser build steps the world inline
#endif
    uiStep = false;
#endif

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

#if CVC_NAV_DEMO_SIM_WORKER
    if (auto snap = sim->read()) {
      if (snap->n == N) {
        const auto &xyz = glyphs.pack(snap->pos.data(), snap->heading.data());
        if (agentNode)
          agentNode->updateVertices(xyz);
        restyle(snap->mode.data(), snap->reached.data());
        trail_update(snap->pos.data());
      }
    }
#else
    if (!simPausedEm)
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

#if CVC_NAV_DEMO_SIM_WORKER
  if (sim)
    sim->stop();
#endif
  cam.detach();
  if (!png.empty())
    view.writePNG(png);
#if CVC_NAV_DEMO_SIM_WORKER
  std::printf("nav_city_swarm: done (%ld frames, sim ticks=%ld)\n", frame, sim ? sim->ticks() : 0L);
#endif
  return 0;
}

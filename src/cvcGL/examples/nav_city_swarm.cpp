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

#include <algorithm> // std::nth_element / clamp / min / max for --lod partition
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib> // std::getenv (CVC_NAV_BUNDLE)
#include <cvc/core/app.h>
#include <cvc/core/state.h> // write Track params (back/height/look_ahead) to camera state
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
#include <cvc/lod/select.h> // agent distance-LOD selection (--lod)
#include <future>
#include <utility> // std::pair (lod score list)
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
#include <vtkActor2D.h>         // skip 2D HUD/text when mirroring to PiP
#include <vtkActorCollection.h> // enumerate main-renderer props to mirror onto PiP
#include <vtkCamera.h>          // PiP ortho camera
#include <vtkCellArray.h>       // build/read polydata for LOD decimation
#include <vtkIdList.h>          //   "
#include <vtkLight.h>           // PiP needs its own lights (main renderer's are aimed at chase)
#include <vtkMatrix4x4.h>       // follow-cam probe transform
#include <vtkNew.h>
#include <vtkPoints.h>   //   "
#include <vtkPolyData.h> //   "
#include <vtkPropCollection.h>
#include <vtkQuadricClustering.h> // fast far-LOD for the big buildings mesh
#include <vtkQuadricDecimation.h> // bake decimated Humvee LOD rungs
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

// Force the discrete GPU on hybrid (Optimus / PowerXpress) laptops. Without this
// an OpenGL app defaults to the integrated GPU -- measured here: the whole demo
// ran on the Intel Iris Xe while the RTX 3050 Ti sat at 2%. These exported
// symbols are read by the NVIDIA/AMD drivers at process start and must live in
// the executable module at global scope. (Verified: the chase cam went 336 ->
// 93 ms/frame once it landed on the discrete GPU.)
#ifdef _WIN32
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

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
    // setTexture takes a TOP-left-origin image and flips the TCoords' V to reach
    // VTK's bottom-left convention (GeometryNode.h). The nav grid is the other
    // way up -- row 0 is min_y, as occupancy_to_walls and plan_route both read
    // it -- so writing grid row r to image row r handed it a BOTTOM-left image
    // and the flip mirrored the fog about the map's mid-line. Everything else on
    // screen is placed from world coordinates and stayed put, which is what made
    // it hard to see: the coverage was plausible, just on the wrong side. The
    // satellite drapes correctly on this same node precisely because a PNG
    // already IS top-left. Same defect and fix as nav_fog_ghost's
    // fill_epistemic (#295).
    const long r = i / cols, c = i % cols;
    unsigned char *p = px + 4 * ((rows - 1 - r) * cols + c);
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

// Decimate a flat triangle mesh to roughly (1 - reduction) of its triangles via
// VTK's quadric-error metric, which preserves the silhouette — the right tool
// for keeping a Humvee recognizable at a fraction of its 8k tris. Output is a
// fresh flat vert/tri mesh; UVs/normals are dropped (the coarse rungs are
// flat-shaded a vehicle olive, and group identity rides the flag above each
// agent). Returns false if decimation collapsed the mesh.
bool decimate_mesh(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                   double reduction, std::vector<double> &outVerts,
                   std::vector<std::uint32_t> &outTris) {
  const vtkIdType nv = static_cast<vtkIdType>(verts.size() / 3);
  vtkNew<vtkPoints> pts;
  pts->SetNumberOfPoints(nv);
  for (vtkIdType i = 0; i < nv; ++i)
    pts->SetPoint(i, verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
  vtkNew<vtkCellArray> cells;
  for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
    const vtkIdType tri[3] = {static_cast<vtkIdType>(tris[t]), static_cast<vtkIdType>(tris[t + 1]),
                              static_cast<vtkIdType>(tris[t + 2])};
    cells->InsertNextCell(3, tri);
  }
  vtkNew<vtkPolyData> pd;
  pd->SetPoints(pts);
  pd->SetPolys(cells);
  vtkNew<vtkQuadricDecimation> dec;
  dec->SetInputData(pd);
  dec->SetTargetReduction(reduction);
  dec->Update();
  vtkPolyData *out = dec->GetOutput();
  outVerts.clear();
  outTris.clear();
  if (!out || !out->GetPoints() || out->GetNumberOfPolys() < 4)
    return false;
  vtkPoints *op = out->GetPoints();
  const vtkIdType on = op->GetNumberOfPoints();
  outVerts.resize(static_cast<std::size_t>(on) * 3);
  for (vtkIdType i = 0; i < on; ++i) {
    double p[3];
    op->GetPoint(i, p);
    outVerts[3 * i] = p[0];
    outVerts[3 * i + 1] = p[1];
    outVerts[3 * i + 2] = p[2];
  }
  vtkCellArray *polys = out->GetPolys();
  polys->InitTraversal();
  vtkNew<vtkIdList> ids;
  while (polys->GetNextCell(ids))
    if (ids->GetNumberOfIds() == 3)
      for (int j = 0; j < 3; ++j)
        outTris.push_back(static_cast<std::uint32_t>(ids->GetId(j)));
  return !outTris.empty();
}

// Fast far-LOD for a LARGE mesh (the ~1M-tri buildings). vtkQuadricDecimation is
// priority-queue based and far too slow for a million triangles at load;
// vtkQuadricClustering bins vertices into a spatial grid and collapses each cell in
// a single linear pass, which is what we want for a distant city silhouette. The
// grid divisions set the coarseness. Colours are NOT carried through here — the
// caller re-tints the output from the satellite, which is exact rather than the
// clustering's averaged scalars. Returns false if the result collapsed.
bool decimate_clustered(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                        int divX, int divY, int divZ, std::vector<double> &outVerts,
                        std::vector<std::uint32_t> &outTris) {
  const vtkIdType nv = static_cast<vtkIdType>(verts.size() / 3);
  vtkNew<vtkPoints> pts;
  pts->SetNumberOfPoints(nv);
  for (vtkIdType i = 0; i < nv; ++i)
    pts->SetPoint(i, verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
  vtkNew<vtkCellArray> cells;
  for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
    const vtkIdType tri[3] = {static_cast<vtkIdType>(tris[t]), static_cast<vtkIdType>(tris[t + 1]),
                              static_cast<vtkIdType>(tris[t + 2])};
    cells->InsertNextCell(3, tri);
  }
  vtkNew<vtkPolyData> pd;
  pd->SetPoints(pts);
  pd->SetPolys(cells);
  vtkNew<vtkQuadricClustering> qc;
  qc->SetInputData(pd);
  qc->SetNumberOfDivisions(divX, divY, divZ);
  qc->CopyCellDataOff();
  qc->Update();
  vtkPolyData *out = qc->GetOutput();
  outVerts.clear();
  outTris.clear();
  if (!out || !out->GetPoints() || out->GetNumberOfPolys() < 4)
    return false;
  vtkPoints *op = out->GetPoints();
  const vtkIdType on = op->GetNumberOfPoints();
  outVerts.resize(static_cast<std::size_t>(on) * 3);
  for (vtkIdType i = 0; i < on; ++i) {
    double p[3];
    op->GetPoint(i, p);
    outVerts[3 * i] = p[0];
    outVerts[3 * i + 1] = p[1];
    outVerts[3 * i + 2] = p[2];
  }
  vtkCellArray *polys = out->GetPolys();
  polys->InitTraversal();
  vtkNew<vtkIdList> ids;
  while (polys->GetNextCell(ids))
    if (ids->GetNumberOfIds() == 3)
      for (int j = 0; j < 3; ++j)
        outTris.push_back(static_cast<std::uint32_t>(ids->GetId(j)));
  return !outTris.empty();
}
} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int agents = 512, grid = 96, width = 1280, height = 720;
  long frames = 0;
  double mouseSens = 0.25, moveSpeed = 0.0; // camera feel (0 move speed = auto from bounds)
  double fps = 30.0, hz = 60.0;
  std::string belief = "shared", capture = "orbit", out = "frames", png, bundle, vehicle;
  bool offscreen = false, fog = false, no_fog = false, no_shadows = false, ortho = false,
       lite = false, lod = false, noLod = false, buildingLod = false, noPip = false;
  int followInit = -1; // agent index to follow at start (-1 = free camera)
  int nearCap = 10;    // --lod: full-detail (rung 0) Humvee budget; coarser rungs fill the rest

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
      "disable shadow map")("lite", po::bool_switch(&lite),
                            "prefer the bundle's low-poly buildings_flat.glb "
                            "(4x fewer tris, faster shadow pass)")(
      "lod", po::bool_switch(&lod),
      "force agent distance-LOD on (it is ON by default; kept for compatibility). LOD "
      "is a 4-rung chain of quadric-decimated Humvees picked per-agent via cvc::lod")(
      "no-lod", po::bool_switch(&noLod),
      "disable agent distance-LOD (perf control: every agent drawn at full Humvee detail)")(
      "building-lod", po::bool_switch(&buildingLod),
      "swap the buildings mesh to a coarse clustered skyline from altitude. OFF by "
      "default: on a discrete GPU the full mesh renders free and looks far better; "
      "this helps only when the buildings are GPU-bound (weak/integrated GPU)")(
      "near-cap", po::value<int>(&nearCap)->default_value(10),
      "full-detail (rung 0) Humvee budget under --lod; coarser decimated rungs fill the rest")(
      "no-pip", po::bool_switch(&noPip),
      "disable the picture-in-picture minimap (perf diagnostic: it re-renders the "
      "whole scene from a top-down camera every frame)")("frames",
                                                         po::value<long>(&frames)->default_value(0),
                                                         "stop after N frames (0 = until closed)")(
      "fps", po::value<double>(&fps)->default_value(30.0),
      "capture frame rate")("hz", po::value<double>(&hz)->default_value(60.0), "sim tick rate")(
      "capture", po::value<std::string>(&capture)->default_value("none"),
      "none (interactive window) | orbit | fly | follow (chase --follow N)")(
      "mouse-sensitivity", po::value<double>(&mouseSens)->default_value(0.25),
      "look speed, degrees per pixel of mouse motion")(
      "move-speed", po::value<double>(&moveSpeed)->default_value(0.0),
      "fly speed in world units/s (0 = auto from scene bounds)")(
      "width", po::value<int>(&width)->default_value(1280))(
      "height", po::value<int>(&height)->default_value(720))(
      "out", po::value<std::string>(&out)->default_value("frames"),
      "PNG frame directory")("png", po::value<std::string>(&png), "write a single final PNG here")(
      "bundle", po::value<std::string>(&bundle),
      "city bundle dir (terrain.json + buildings.glb) for a REAL, much larger city (e.g. "
      "Austin); else $CVC_NAV_BUNDLE; else the synthetic city_scene")(
      "vehicle", po::value<std::string>(&vehicle),
      "vehicle model .glb per agent (default: <bundle>/../../shared/Humvee.glb; else a flat "
      "arrow)")("follow", po::value<int>(&followInit)->default_value(-1),
                "agent index to CHASE from start with the cinematic Track camera (-1 = free)");
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

  // 1. The city occupancy. Default: the pure-C++ "city_scene" the trainer learns on
  //    (bordered so reactive agents can't drive off the open edge). A --bundle (or
  //    $CVC_NAV_BUNDLE) swaps in a REAL, much larger city loaded at runtime
  //    (terrain.json + buildings.glb, e.g. Austin at +/-1500 -- 30x larger). The
  //    controller transfers because we keep scale 0.05, the trained metric, so the
  //    coef_mlp's normalized vehicle constants stay physical (see nav_finale).
  cvc::nav::training_scene ts = cvc::nav::city_scene(grid); // vehicle params + synthetic fallback
  std::vector<std::uint8_t> occ = ts.occ;
  navdemo::Bounds bounds{ts.min_x, ts.min_y, ts.max_x, ts.max_y};
  int rows = ts.rows, cols = ts.cols;
  double scale = ts.scale;
  cvc::geometry cityMesh;
  bool haveMesh = false;
  navdemo::Terrain terrain; // real elevation from bundle's terrain.json (optional)
  if (bundle.empty())
    if (const char *envB = std::getenv("CVC_NAV_BUNDLE"))
      bundle = envB;
#ifdef __EMSCRIPTEN__
  // Emscripten build: CMake --preload-file stashes the Austin bundle at /bundle
  // in the virtual filesystem. Autodetect it so the browser demo picks it up
  // with no user-visible env / argv machinery.
  if (bundle.empty() && std::filesystem::exists("/bundle/terrain.json"))
    bundle = "/bundle";
#else
  // Native fallback probes: without --bundle / $CVC_NAV_BUNDLE, look for an
  // Austin bundle in the places one would expect a cvcpkg-installed prefix or
  // an in-tree dev checkout to keep it, so the demo isn't just a black screen
  // when a user forgets the flag. First hit wins; the chosen path is logged.
  if (bundle.empty()) {
    for (const char *p :
         {"deps/share/cvc-scenes/austin_south", "../deps/share/cvc-scenes/austin_south",
          "share/cvc-scenes/austin_south", "platoon-sim/scene_viewer/exports/scenes/austin_south",
          "../platoon-sim/scene_viewer/exports/scenes/austin_south", "scenes/austin_south",
          "../scenes/austin_south"}) {
      if (std::filesystem::exists(std::string(p) + "/terrain.json")) {
        bundle = p;
        std::printf("nav_city_swarm: no --bundle given, autodetected %s\n", bundle.c_str());
        break;
      }
    }
    if (bundle.empty())
      std::fprintf(stderr,
                   "nav_city_swarm: no --bundle given and no known local Austin bundle found; "
                   "falling back to the synthetic city (no terrain, no satellite, no buildings). "
                   "Pass --bundle <dir> to load one.\n");
  }
#endif
  if (!bundle.empty()) {
    navdemo::Bounds bb;
    std::vector<std::uint8_t> bocc;
    bool loaded = false;
    try {
      loaded = navdemo::load_city_bundle(bundle, grid, grid, bb, bocc, &cityMesh, &terrain,
                                         /*prefer_flat=*/lite);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "nav_city_swarm: bundle load threw: %s -> using synthetic city\n",
                   e.what());
      loaded = false;
    } catch (...) {
      std::fprintf(stderr, "nav_city_swarm: bundle load raised unknown -> using synthetic city\n");
      loaded = false;
    }
    if (loaded) {
      bounds = bb;
      rows = grid;
      cols = grid;
      scale = 0.05; // the trained world metric; keeps the coef_mlp constants physical
      haveMesh = cityMesh.num_tris() > 0;
      if (!bocc.empty()) {
        occ = std::move(bocc);
      } else {
        // Buildings mesh failed to load (e.g. wasm assimp glTF quirk) but terrain
        // + satellite are still real — keep the existing (synthetic) occupancy
        // rescaled to the real bounds so the swarm has some walls to avoid.
        // occ retains its ts.occ contents from before the bundle load.
        std::fprintf(
            stderr,
            "  no buildings mesh — using synthetic occupancy over the real Austin bounds\n");
      }
      std::printf("nav_city_swarm: bundle %s  bounds [%.0f,%.0f]..[%.0f,%.0f]  %llu tris%s\n",
                  bundle.c_str(), bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y,
                  (unsigned long long)cityMesh.num_tris(),
                  terrain.empty() ? ""
                                  : (std::string(" (terrain ") + std::to_string(terrain.rows) +
                                     "x" + std::to_string(terrain.cols) + ")")
                                        .c_str());
    } else {
      std::printf("nav_city_swarm: bundle '%s' incomplete; using the synthetic city\n",
                  bundle.c_str());
    }
  }
  navdemo::add_border(occ.data(), rows, cols);
  const double span = bounds.max_x - bounds.min_x;
  const double wall_h =
      0.06 * span; // blocky fallback buildings (real mesh used when a bundle loads)

  cvc::nav::sim_world::config cfg;
  cfg.rows = rows;
  cfg.cols = cols;
  cfg.min_x = bounds.min_x;
  cfg.min_y = bounds.min_y;
  cfg.max_x = bounds.max_x;
  cfg.max_y = bounds.max_y;
  cfg.cx = bounds.cx();
  cfg.cy = bounds.cy();
  cfg.scale = scale;
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
  // Start every vehicle on one edge of the map and put every goal on the opposite
  // edge — a deliberate cross-map migration instead of a random scatter that
  // reads as a tangled mess.
  cfg.spawn_layout = cvc::nav::sim_world::config::spawn::opposed;

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
        cfg, occ.data(), cvc::nav::coef_mlp::default_biased(), nAgents, seed, m, k));
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
  const double ground_rgb[3] = {0.20, 0.23, 0.27};

  // ---- The bundle's aerial photo: the demo's headline ground AND building tint.
  // Load it whenever the bundle ships one (previously gated on fog-off, which hid
  // it by default). Kept at function scope: setTexture's zeroCopy path borrows
  // these pixels for the ground's lifetime.
  cvc::image satTex;
  bool haveSat = false;
  if (!bundle.empty()) {
    const std::string sp = bundle + "/satellite.png";
    if (std::filesystem::exists(sp)) {
      try {
        satTex = cvc::read_image(sp).converted(cvc::image::pixel_format::RGBA,
                                               cvc::image::data_type::u8);
        haveSat = satTex.width() > 0 && satTex.height() > 0;
        if (haveSat)
          std::printf("nav_city_swarm: satellite %s (%dx%d)\n", sp.c_str(), satTex.width(),
                      satTex.height());
      } catch (const std::exception &e) { // a missing decoder must not kill the demo
        std::fprintf(stderr, "nav_city_swarm: satellite load failed (%s)\n", e.what());
      }
    }
  }
  // Sample the aerial photo at a world (x, y) -> RGB in [0,1]. Top-left-origin
  // image over [min,max] on each axis; V flipped to match the ground UVs.
  auto sat_rgb = [&](double x, double y, double &r, double &g, double &b) {
    const int W = satTex.width(), H = satTex.height();
    double u = (x - bounds.min_x) / (bounds.max_x - bounds.min_x);
    double v = (y - bounds.min_y) / (bounds.max_y - bounds.min_y);
    u = std::min(std::max(u, 0.0), 1.0);
    v = std::min(std::max(v, 0.0), 1.0);
    const int px = std::min(W - 1, static_cast<int>(u * W));
    const int py = std::min(H - 1, static_cast<int>((1.0 - v) * H));
    const unsigned char *p = satTex.data() + (static_cast<std::size_t>(py) * W + px) * 4;
    r = p[0] / 255.0;
    g = p[1] / 255.0;
    b = p[2] / 255.0;
  };

  // Building LOD: the full mesh up close, a fast clustered far mesh from altitude.
  // Both live in the main scene; the render loop toggles their visibility by camera
  // height so distant/fly-over views draw the cheap silhouette and close-ups the
  // full detail. The PiP mirrors whichever is visible, so it gets the cheap mesh
  // from altitude for free.
  std::shared_ptr<GeometryNode> fullBuildings, farBuildings;
  if (haveMesh) {
    // Real building geometry from buildings.glb (true heights + shapes). glb bakes
    // bases at z=0, so lift every vertex by terrain.sample() to sit them on the
    // ground; and when we have the satellite, tint each vertex by the aerial photo
    // at its footprint so the city reads as real Austin instead of a flat grey
    // block model. One sample per vertex, once at load.
    auto &pts = cityMesh.points();
    if (!terrain.empty())
      for (auto &p : pts)
        p[2] += terrain.sample(p[0], p[1]);
    if (haveSat) {
      auto &cols = cityMesh.colors();
      cols.resize(pts.size());
      for (std::size_t i = 0; i < pts.size(); ++i) {
        double r, g, b;
        sat_rgb(pts[i][0], pts[i][1], r, g, b);
        // Bias toward the photo but keep a grey floor so shaded facades still read.
        cols[i] = {0.30 + 0.70 * r, 0.30 + 0.70 * g, 0.32 + 0.68 * b};
      }
    }
    auto wnode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("buildings", cityMesh));
    if (wnode) {
      wnode->setUseSingleColor(!haveSat); // per-vertex satellite tint when we have it
      if (!haveSat)
        wnode->setColor(wall_rgb[0], wall_rgb[1], wall_rgb[2]);
      wnode->setAmbient(0.35);
      wnode->setDiffuse(0.80);
      wnode->setSpecular(0.10);
    }
    fullBuildings = wnode;

    // Far-LOD twin: cluster the ~1M-tri mesh down to a coarse skyline for distant
    // views, re-tinted from the satellite (exact, not the clustering's averaged
    // scalars) so it never reverts to the grey block model. cityMesh is already
    // terrain-lifted here, so the far mesh sits on the ground too. Hidden until the
    // loop raises the camera. Opt-in (--building-lod): the full mesh is GPU-cheap
    // and looks better, so the default demo keeps it.
    if (buildingLod) {
      const auto &fp = cityMesh.points();
      const auto &ft = cityMesh.tris();
      std::vector<double> fv(fp.size() * 3);
      for (std::size_t i = 0; i < fp.size(); ++i) {
        fv[3 * i] = fp[i][0];
        fv[3 * i + 1] = fp[i][1];
        fv[3 * i + 2] = fp[i][2];
      }
      std::vector<std::uint32_t> fi(ft.size() * 3);
      for (std::size_t i = 0; i < ft.size(); ++i) {
        fi[3 * i] = static_cast<std::uint32_t>(ft[i][0]);
        fi[3 * i + 1] = static_cast<std::uint32_t>(ft[i][1]);
        fi[3 * i + 2] = static_cast<std::uint32_t>(ft[i][2]);
      }
      std::vector<double> dv;
      std::vector<std::uint32_t> di;
      // ~110x110 footprint cells, 40 height bins over the city bbox: fine enough to
      // keep individual building footprints instead of melting them into blobs,
      // still a fraction of the full tri count.
      if (decimate_clustered(fv, fi, 110, 110, 40, dv, di) && di.size() >= 12) {
        cvc::geometry farMesh;
        auto &dp = farMesh.points();
        dp.resize(dv.size() / 3);
        for (std::size_t i = 0; i < dp.size(); ++i)
          dp[i] = {dv[3 * i], dv[3 * i + 1], dv[3 * i + 2]};
        auto &dt = farMesh.tris();
        dt.resize(di.size() / 3);
        for (std::size_t i = 0; i < dt.size(); ++i)
          dt[i] = {di[3 * i], di[3 * i + 1], di[3 * i + 2]};
        if (haveSat) {
          auto &dc = farMesh.colors();
          dc.resize(dp.size());
          for (std::size_t i = 0; i < dp.size(); ++i) {
            double r, g, b;
            sat_rgb(dp[i][0], dp[i][1], r, g, b);
            dc[i] = {0.30 + 0.70 * r, 0.30 + 0.70 * g, 0.32 + 0.68 * b};
          }
        }
        farBuildings =
            std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("buildings_far", farMesh));
        if (farBuildings) {
          farBuildings->setUseSingleColor(!haveSat);
          if (!haveSat)
            farBuildings->setColor(wall_rgb[0], wall_rgb[1], wall_rgb[2]);
          farBuildings->setAmbient(0.35);
          farBuildings->setDiffuse(0.80);
          farBuildings->setSpecular(0.10);
          if (farBuildings->prop())
            farBuildings->prop()->SetVisibility(0); // shown only from altitude
        }
        std::printf("nav_city_swarm: building LOD — far mesh %zu tris (from %llu full)\n",
                    dt.size(), (unsigned long long)cityMesh.num_tris());
        std::fflush(stdout);
      }
    }
  } else {
    sg.addGraphics("walls", navdemo::occupancy_to_walls(occ.data(), rows, cols, bounds, wall_h,
                                                        wall_rgb, /*vary=*/0.45));
  }

  // Real elevation from terrain.json when we have a bundle — a tessellated
  // heightmap so the buildings sit on the ground and the vehicles drive over
  // hills instead of clipping straight through them. Flat ground_quad fallback
  // when no terrain (synthetic city_scene, or a bundle missing the grid).
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", terrain.empty() ? navdemo::ground_quad(bounds, 0.0, ground_rgb)
                                               : navdemo::terrain_mesh(terrain, ground_rgb)));
  cvc::image fogTex(grid, grid, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    if (haveSat) { // the ground IS the aerial photo (default when the bundle ships one)
      groundNode->setColor(1, 1, 1);
      groundNode->setTexture(satTex, /*zeroCopy=*/true);
      groundNode->setAmbient(0.85); // photo already carries its own shading
      groundNode->setDiffuse(0.35);
    } else if (fogOn) { // fallback: the ground IS the fleet's fog coverage
      fill_fleet_fog(fogTex.data(), world, rows, cols);
      groundNode->setColor(1, 1, 1);
      groundNode->setTexture(fogTex, /*zeroCopy=*/true);
      groundNode->setAmbient(0.65); // soften the shadow-map boundary on the big flat ground
      groundNode->setDiffuse(0.50);
    } else {
      groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
      groundNode->setAmbient(0.65);
      groundNode->setDiffuse(0.50);
    }
  }

  navdemo::AgentGlyphs glyphs;
  // Vehicle size derived from real Humvee dimensions (M1097 length ~ 4.72 m).
  // Bundled cities (e.g. Austin) use metre-scale world units — terrain.json's
  // extents are in local metres — so gsz = kHumveeLenM directly. Synthetic
  // city_scene lives on a normalized ±100 world with scale = 0.05; the trained
  // vehicle radius rr = 0.15 world units is the same physical size the coef_mlp
  // learned around, so the synthetic proxy uses ~2 * rr for a similar footprint.
  //
  // (Sanity: at ±100 span for city_scene, 2*rr = 0.3 world units feels small,
  // but that's the WORLD the controller was trained on. This footprint keeps
  // the visual proportion the same as the trained hitbox.)
  constexpr double kHumveeLenM = 4.72; // M1097 length, metres
  // A real Humvee model per agent instead of a flat arrow, when one is available
  // (--vehicle, or <bundle>/../../shared/Humvee.glb). Instanced + streamed exactly
  // like the arrow: build_template -> pack -> updateVertices.
  std::vector<double> vverts;
  std::vector<std::uint32_t> vtris;
  std::vector<float> vuvs; // model UVs (empty when the glTF has no texcoords)
  cvc::image vtexture;     // base-color texture (empty when the model has none)
  bool haveVehicle = false;
  if (vehicle.empty() && !bundle.empty()) {
    // Try inside the bundle first (a scene that ships its own vehicle), then the
    // sibling shared/ convention (vehicle-humvee cvcpkg + platoon-sim layout).
    const std::string candIn = bundle + "/Humvee.glb";
    const std::string candSib = bundle + "/../../shared/Humvee.glb";
    if (std::filesystem::exists(candIn))
      vehicle = candIn;
    else if (std::filesystem::exists(candSib))
      vehicle = candSib;
  }
  // Bundles use metre-scale world units; synthetic uses normalized units.
  const bool worldIsMetres = !bundle.empty();
  const double gsz = worldIsMetres ? kHumveeLenM : (2.0 * static_cast<double>(cfg.veh.rr));
  if (!vehicle.empty())
    haveVehicle = navdemo::load_vehicle_template(vehicle, gsz, vverts, vtris, &vuvs, &vtexture);
  const bool haveVehicleTexture = haveVehicle && !vuvs.empty() && !vtexture.empty();
  if (haveVehicle)
    std::printf("nav_city_swarm: vehicle model %s (%zu tris, gsz=%.2f world-units%s%s)\n",
                vehicle.c_str(), vtris.size() / 3, gsz,
                worldIsMetres ? " ~ real 4.72 m Humvee" : " (normalized city)",
                haveVehicleTexture ? " + baked texture" : "");
  // Build the per-agent instanced mesh (Humvee if loaded, else the flat arrow).
  // When the model has UVs + a texture we pass UVs through so setTexture() below
  // drapes the real Humvee texture; identity moves to the flag.
  auto build_agents = [&]() {
    if (!haveVehicle)
      return glyphs.build(app, N, color.data(), gsz, 0.6);
    return glyphs.build_template(app, N, color.data(), vverts, vtris, /*z=*/0.0,
                                 haveVehicleTexture ? &vuvs : nullptr);
  };
  cvc::geometry agentGeom = build_agents();
  auto agentNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("agents", agentGeom));
  if (agentNode) {
    agentNode->setUseSingleColor(false); // per-vertex group colours
    // Shaded, not flat: low ambient + strong diffuse so the stage rig gives the
    // vehicles real form (the flat arrow read as a paper cut-out).
    agentNode->setAmbient(0.32);
    agentNode->setDiffuse(0.9);
    agentNode->setSpecular(0.2);
    agentNode->setSpecularPower(18.0);
    // Real baked Humvee texture when the glTF ships one — the group identity
    // moves to the flag above the vehicle. VTK's texture path takes precedence
    // over per-vertex colour scalars once setTexture is applied.
    if (haveVehicleTexture)
      agentNode->setTexture(vtexture, /*zeroCopy=*/false);
  }

  // --- Agent distance LOD (--lod): a real decimated-mesh CHAIN ---------------
  // The Austin demo is bottlenecked on the per-frame agent-body pack (N x an
  // 8k-tri Humvee, transformed and deep-copied through updateVertices every
  // frame). Rather than a binary full-Humvee/arrow snap, bake a CHAIN of
  // decimated Humvee rungs (VTK quadric decimation, silhouette-preserving) from
  // full detail down to a coarse ~4%-tri silhouette, and pick a rung per agent
  // each frame by on-screen size via cvc::lod. Each rung is capped, and because
  // most agents land on a cheap rung the TOTAL packed cost is lower than one
  // near-node of full Humvees — while the vehicle stays a recognizable Humvee at
  // every distance. Rung 0 keeps the baked texture; coarser rungs carry the
  // per-agent group colour (re-gathered each frame). Only meaningful when a real
  // vehicle mesh loaded; opt-in so the default path is untouched.
  // LOD is on by default now (the demo is agent-bound); --no-lod is the perf
  // control, --lod is a redundant force-on kept for older invocations.
  const bool useLod = (lod || !noLod) && haveVehicle;
  struct LodRung {
    navdemo::AgentGlyphs glyphs;
    std::shared_ptr<GeometryNode> node;
    int cap = 0;
    int vpi = 0;                    // verts per instance (for the colour re-gather)
    bool textured = false;          // rung 0 only
    std::vector<unsigned char> rgb; // per-frame colour scratch (coloured rungs)
  };
  std::vector<LodRung> rungs;
  std::vector<unsigned char> agentColorU8; // per-agent rgb for the colour re-gather
  if (useLod) {
    agentNode->setVisible(false); // replaced by the rung nodes
    agentColorU8.resize(color.size());
    for (std::size_t i = 0; i < color.size(); ++i)
      agentColorU8[i] =
          static_cast<unsigned char>(std::lround(std::clamp(color[i], 0.0f, 1.0f) * 255.0f));
    // (target reduction, cap) per rung, finest first. The last rung's cap is N so
    // it always absorbs the remainder. nearCap tunes the full-detail budget.
    struct Spec {
      double reduction;
      int cap;
    };
    const std::vector<Spec> specs = {
        {0.00, std::max(1, std::min(N, nearCap))}, // full Humvee, textured (close-ups only)
        {0.62, std::min(N, 28)},                   // ~38% tris
        {0.88, std::min(N, 72)},                   // ~12% tris
        {0.985, N},                                // ~1.5% tris — the far majority, cheap
    };
    rungs.resize(specs.size());
    for (std::size_t r = 0; r < specs.size(); ++r) {
      std::vector<double> rv = vverts;
      std::vector<std::uint32_t> rt = vtris;
      if (specs[r].reduction > 0.0 && !decimate_mesh(vverts, vtris, specs[r].reduction, rv, rt)) {
        rv = vverts; // decimation collapsed the mesh: fall back to full detail
        rt = vtris;
      }
      LodRung &rung = rungs[r];
      rung.cap = std::max(1, specs[r].cap);
      rung.textured = (r == 0) && haveVehicleTexture;
      cvc::geometry g = rung.glyphs.build_template(app, rung.cap, color.data(), rv, rt, /*z=*/0.0,
                                                   rung.textured ? &vuvs : nullptr);
      rung.vpi = rung.glyphs.verts_per_instance();
      rung.node = std::dynamic_pointer_cast<GeometryNode>(
          sg.addGraphics("agents_lod_" + std::to_string(r), g));
      if (rung.node) {
        rung.node->setUseSingleColor(false);
        rung.node->setAmbient(0.34);
        rung.node->setDiffuse(0.9);
        rung.node->setSpecular(0.18);
        rung.node->setSpecularPower(18.0);
        if (rung.textured)
          rung.node->setTexture(vtexture, /*zeroCopy=*/false);
        else
          rung.rgb.assign(static_cast<std::size_t>(rung.cap) * rung.vpi * 3, 180);
      }
      std::printf("nav_city_swarm: LOD rung %zu: %zu tris x cap %d%s\n", r, rt.size() / 3, rung.cap,
                  rung.textured ? " (textured)" : "");
    }
    std::fflush(stdout);
  }

  // Per-agent COLOR FLAG: a thin pole + rectangular flag flying above the vehicle,
  // instanced + streamed like the body. This keeps identity legible when the body
  // wears its real Humvee texture instead of a group tint, and reads from far away
  // over the city — same trick as the goal beacons, at vehicle scale.
  navdemo::AgentGlyphs flagGlyphs;
  std::shared_ptr<GeometryNode> flagNode;
  {
    // Local frame: forward +x, up +z. Sizes are anchored to the VEHICLE (not the
    // city) so the flag is proportional in chase-cam close-ups AND still legible
    // from a wide orbit — enough headroom to clear vehicle geometry, but nothing
    // that dwarfs the Humvee when the camera closes in. gsz IS the Humvee length
    // in world units (see kHumveeLenM), so all these dimensions read like real
    // metres given scale = 0.05 (1 m = 20 world units).
    const double ph = gsz * 1.5;  // pole tall enough to clear roof + a bit
    const double pr = gsz * 0.03; // pole thickness ~0.15 m (visible from orbit)
    const double fw = gsz * 0.7;  // flag panel width ~ 3.3 m at Humvee scale
    const double fh = gsz * 0.4;  // flag panel height ~ 1.9 m at Humvee scale
    // pole (thin square column) + flag panel (double-sided quad), all in the -y half
    // so the flag flutters to the LEFT of the direction of travel.
    std::vector<double> fv = {
        // pole (8 verts)
        -pr,
        -pr,
        0.0,
        pr,
        -pr,
        0.0,
        pr,
        pr,
        0.0,
        -pr,
        pr,
        0.0,
        -pr,
        -pr,
        ph,
        pr,
        -pr,
        ph,
        pr,
        pr,
        ph,
        -pr,
        pr,
        ph,
        // flag panel (4 verts): a rectangle from the pole out to -y=-fw, at top
        0.0,
        0.0,
        ph - fh,
        0.0,
        -fw,
        ph - fh,
        0.0,
        -fw,
        ph,
        0.0,
        0.0,
        ph,
    };
    std::vector<std::uint32_t> ft = {
        // pole box (12 tris)
        0,
        2,
        1,
        0,
        3,
        2,
        4,
        5,
        6,
        4,
        6,
        7,
        0,
        1,
        5,
        0,
        5,
        4,
        1,
        2,
        6,
        1,
        6,
        5,
        2,
        3,
        7,
        2,
        7,
        6,
        3,
        0,
        4,
        3,
        4,
        7,
        // flag panel (2 tris, both winds — double-sided by drawing twice)
        8,
        9,
        10,
        8,
        10,
        11,
        8,
        10,
        9,
        8,
        11,
        10,
    };
    (void)fh;
    (void)fw; // clang tidy: used only through fv above
    cvc::geometry flagGeom = flagGlyphs.build_template(app, N, color.data(), fv, ft, /*z=*/0.0);
    flagNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("flags", flagGeom));
    if (flagNode) {
      flagNode->setUseSingleColor(false);
      flagNode->setAmbient(0.6); // stay legible in shadow, still shaded
      flagNode->setDiffuse(0.7);
    }
  }

  // PiP OVERLAY DOTS: a big flat disc per agent in the agent's hue, sized so it
  // reads as a visible pixel on the top-down ortho map (~1/40 of the city span).
  // Uses AgentGlyphs (identical streaming model as the body/flags: build once,
  // pack per frame, updateVertices) — only its scale and its renderer set differ.
  // Attached ONLY to the PiP renderer (removed from main) so it doesn't pollute
  // the chase view.
  navdemo::AgentGlyphs pipDotGlyphs;
  std::shared_ptr<GeometryNode> pipDotNode;
  {
    // Flat regular pentagon in the XY plane, ~40x the Humvee footprint so it
    // shows at the ortho scale (3km map / 720px window ~ 4 m/px, so a 200 m
    // marker is ~50 px — a clickable pixel target).
    const double rr = std::max(gsz * 20.0, (bounds.max_x - bounds.min_x) * 0.008);
    std::vector<double> pv;
    std::vector<std::uint32_t> pt;
    pv.reserve(5 * 3);
    for (int k = 0; k < 5; ++k) {
      const double a = k * (2.0 * PI / 5.0) - PI / 2.0; // pointer up
      pv.push_back(rr * std::cos(a));
      pv.push_back(rr * std::sin(a));
      pv.push_back(0.0);
    }
    pt = {0, 1, 2, 0, 2, 3, 0, 3, 4}; // triangle fan
    cvc::geometry dotGeom = pipDotGlyphs.build_template(app, N, color.data(), pv, pt, /*z=*/0.5);
    pipDotNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("pip_dots", dotGeom));
    if (pipDotNode) {
      pipDotNode->setUseSingleColor(false);
      pipDotNode->setAmbient(1.0); // flat, no shadow tint — reads as a UI marker
      pipDotNode->setDiffuse(0.0);
    }
  }

  // Follow-cam PROBE: an invisible one-triangle node whose transform is set to
  // the selected agent's world pose each tick. CameraController::Mode::Track
  // resolves its target by node name -> node->getWorldTransform() (origin), so
  // this proxy is what the cinematic follow camera reads. The probe is
  // graphically invisible; only its transform matters.
  cvc::geometry probeGeom(app);
  {
    auto &pts = probeGeom.points();
    auto &tris = probeGeom.tris();
    pts.push_back({0, 0, 0});
    pts.push_back({0.01, 0, 0});
    pts.push_back({0, 0.01, 0});
    tris.push_back({0, 1, 2});
  }
  auto probeNode =
      std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("follow_probe", probeGeom));
  vtkNew<vtkMatrix4x4> probeXform;
  probeXform->Identity();
  if (probeNode)
    probeNode->setVisible(false); // pure transform holder for the follow camera
  int followAgent = followInit;   // -1 = free camera, else agent index we're chasing
  auto set_probe_at = [&](double x, double y, double z, double heading) {
    // Rotation about +z, translation to (x,y,z). We only need translation for the
    // Track target lookup, but keeping heading in the transform makes the camera
    // sit BEHIND the vehicle nose (Track uses the local -x axis as "behind").
    const double c = std::cos(heading), s = std::sin(heading);
    probeXform->SetElement(0, 0, c);
    probeXform->SetElement(0, 1, -s);
    probeXform->SetElement(0, 3, x);
    probeXform->SetElement(1, 0, s);
    probeXform->SetElement(1, 1, c);
    probeXform->SetElement(1, 3, y);
    probeXform->SetElement(2, 3, z);
    if (probeNode)
      probeNode->setTransform(probeXform);
  };

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
    // Size relative to the city, but CAPPED to a physical multiple of the vehicle
    // so a huge real city (Austin, +/-1500) isn't a forest of 400 giant spikes.
    const double phys = static_cast<double>(cfg.veh.rr) / scale; // vehicle radius, metres
    const double gh = std::min(0.006 * span, 2.5 * phys);        // cap half-width
    const double ght = std::min(1.2 * wall_h, 22.0 * phys);      // beacon height (clears rooftops)
    const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                    ght, gh, gh, ght, -gh, gh,  ght};
    const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 1, 3, 2, 1, 4, 3};
    cvc::geometry goalGeom =
        goalGlyphs.build_template(app, N, color.data(), pv, pt, /*z=*/0.0); // apex on the ground
    goalsNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("goals", goalGeom));
    std::vector<float> gw(static_cast<std::size_t>(2) * N), zeroHead(N, 0.0f);
    world.goals_world(gw.data());
    // Sit each beacon's apex on the ground: lift it by the terrain height at the
    // goal (as the buildings and agents are), or with real elevation the goals
    // sink through the floor. Goals are static, so this is sampled once here.
    std::vector<double> goalZoff(N, 0.0);
    if (!terrain.empty())
      for (int i = 0; i < N; ++i)
        goalZoff[i] = terrain.sample(gw[2 * i], gw[2 * i + 1]);
    goalXyz = goalGlyphs.pack_z(gw.data(), zeroHead.data(), goalZoff.data());
    if (goalsNode) {
      goalsNode->setUseSingleColor(false);
      // Shaded, not a flat glow: enough ambient to stay legible in shadow, but
      // real diffuse so the beacon has form under the stage rig.
      goalsNode->setAmbient(0.5);
      goalsNode->setDiffuse(0.75);
      goalsNode->setSpecular(0.15);
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

  // FOLLOWED-VEHICLE PATH: a dedicated LINES node showing ONLY the chased
  // agent's ring buffer, in a bright accent (yellow) at 3x line width so it
  // stands out against the dim fleet trails. Same TRAIL_K slots, drawn on top
  // via a larger depth offset. Only visible while chaseOn — hidden otherwise.
  std::shared_ptr<GeometryNode> pathNode;
  std::vector<double> pathXyz(static_cast<std::size_t>(3) * 2 * TRAIL_K, 0.0);
  {
    cvc::geometry pg;
    for (int k = 0; k < TRAIL_K; ++k)
      for (int e = 0; e < 2; ++e) {
        pg.points().push_back({0.0, 0.0, 0.35});
        pg.colors().push_back({1.0, 0.85, 0.10}); // saturated yellow
        if (e == 1)
          pg.lines().push_back({static_cast<cvc::geometry::index_t>(2 * k),
                                static_cast<cvc::geometry::index_t>(2 * k + 1)});
      }
    pathNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("chase_path", pg));
    if (pathNode) {
      pathNode->setRenderMode(GeometryRenderMode::LINES);
      pathNode->setUseSingleColor(false);
      pathNode->setLineWidth(4.0); // 2.7x the fleet trails
      pathNode->setAmbient(1.0);
      pathNode->setDiffuse(0.0);
      pathNode->setDepthOffset(2.0); // above the fleet trails
      pathNode->setVisible(false);   // shown only while chase cam is on
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
  cam.setScene(&sg); // Track mode resolves its target node in this scene
  cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, wall_h);
  cam.setMouseSensitivity(mouseSens); // --mouse-sensitivity / state settings.mouse_sensitivity
  if (moveSpeed > 0.0)
    cam.setMoveSpeed(moveSpeed);
  // Track (chase) mode is tuned by state keys under viewers.main.camera.track.*
  // (see CameraController.h); its DEFAULTS are back=55 / height=40 world units,
  // right for the ±100 training world. On a metre-scale bundle (Austin ±1500)
  // those trail distances are microscopic against the vehicle and the camera
  // degenerates. Set trail parameters PROPORTIONAL TO THE VEHICLE (gsz metres),
  // so the chase-cam works the same on any city.
  auto configure_track_params = [&]() {
    const std::string tp = sg.getStatePrefix() + ".viewers.main.camera.track.";
    auto st = [&](const std::string &k, double v) {
      try {
        cvc::state::instance(app)(tp + k).value(v);
      } catch (...) {
      }
    };
    st("back", gsz * 2.5);       // ~12 m behind a 4.7 m Humvee (intimate over-the-shoulder)
    st("height", gsz * 1.2);     // ~5.7 m above
    st("look_ahead", gsz * 1.2); // aim slightly ahead of the vehicle
    st("look_up", gsz * 0.4);    // aim a touch above ground plane
    st("min_speed", 0.5);        // fall back to a static trail below 0.5 wu/s
    // Faster catch-up: the DEFAULT cam_tau = 0.55 is a filmic slow ease that
    // takes ~50 frames to close on the target from a wide framed pose. Tighten
    // so the chase snaps into place within a second (~0.08 s tau -> ~30 frames
    // to converge at 30 fps EMA).
    st("cam_tau", 0.08);
    st("pos_tau", 0.06);
    // Read back via the READ overload (value<T>() returns T; the writer overload
    // takes const T& and clobbers).
    double back_check = -1.0;
    try {
      back_check = cvc::state::instance(app)(tp + "back").value<double>();
    } catch (const std::exception &e) {
      std::printf("nav_city_swarm: read-back threw: %s\n", e.what());
    }
    std::printf("nav_city_swarm: Track configured (gsz=%.1f) at %s (back read=%.2f)\n", gsz,
                tp.c_str(), back_check);
  };
  // --follow N: cinematic Track camera. Seed the vtkCamera at the chase pose on
  // frame 0 so it's already framed behind the vehicle (instead of easing in for
  // ~50 frames from the default wide orbit). Grab the initial agent pose from
  // the world and both position the probe and seed the camera to match.
  if (followInit >= 0 && followInit < N) {
    std::vector<float> ip(static_cast<std::size_t>(2) * N), ih(N);
    world.snapshot(ip.data(), ih.data(), nullptr, nullptr, nullptr);
    const double ax = ip[2 * followInit], ay = ip[2 * followInit + 1];
    const double ah = ih[followInit];
    const double az = terrain.empty() ? 0.0 : terrain.sample(ax, ay);
    configure_track_params();
    set_probe_at(ax, ay, az, ah);
    const double ch = std::cos(ah), sh = std::sin(ah);
    const double back = gsz * 2.5, height = gsz * 1.2, la = gsz * 1.2;
    const double ex = ax - ch * back, ey = ay - sh * back, ez = az + height;
    const double fx = ax + ch * la, fy = ay + sh * la, fz = az + gsz * 0.4;
    view.setCamera(ex, ey, ez, fx, fy, fz, 0, 0, 1, 45.0);
    cam.setTrackTarget("follow_probe");
    cam.setMode(CameraController::Mode::Track);
  }
  if (ortho)
    navdemo::set_ortho_topdown(view, bounds, 4.0,
                               capturing ? nullptr : &cam); // fixed 2-D top-down map
  sg.setDiagnosticChromeVisible(false);
  sg.processEvents();

  // Picture-in-picture: an orthographic top-down overlay in the bottom-right
  // corner. Uses a SECOND vtkRenderer on the same render window, sharing the
  // main renderer's actors via AddViewProp (each vtkProp can live in multiple
  // renderers — VTK handles that fine). Layer 1 draws over layer 0. Click a
  // vehicle in the PiP to CHASE it (handled from the ImGui draw callback below).
  vtkSmartPointer<vtkRenderer> pipRenderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkCamera> pipCam = vtkSmartPointer<vtkCamera>::New();
  {
    const double halfH = 0.5 * (bounds.max_y - bounds.min_y);
    const double halfW = 0.5 * (bounds.max_x - bounds.min_x);
    const double cx = bounds.cx(), cy = bounds.cy();
    pipCam->SetParallelProjection(true);
    pipCam->SetParallelScale(std::max(halfH, halfW) * 1.05);
    pipCam->SetPosition(cx, cy, wall_h * 5.0 + 500.0); // straight down
    pipCam->SetFocalPoint(cx, cy, 0.0);
    pipCam->SetViewUp(0.0, 1.0, 0.0);
    pipCam->SetClippingRange(1.0, wall_h * 10.0 + 5000.0);
    pipRenderer->SetActiveCamera(pipCam);
    pipRenderer->SetViewport(0.72, 0.0, 1.0, 0.28); // bottom-right corner
    pipRenderer->SetBackground(0.02, 0.03, 0.05);
    pipRenderer->SetLayer(1);
    // Own lighting so the top-down view is evenly lit regardless of the main
    // scene's stage rig (which is aimed at the chase framing).
    pipRenderer->AutomaticLightCreationOff();
    pipRenderer->RemoveAllLights();
    {
      // Dim, ambient-only headlight so the satellite ground doesn't blow out
      // and vehicle glyphs stay legible against it.
      auto hl = vtkSmartPointer<vtkLight>::New();
      hl->SetLightTypeToHeadlight();
      hl->SetIntensity(0.55);
      pipRenderer->AddLight(hl);
    }
    // Mirror the main renderer's 3-D props onto the PiP. Each vtkProp3D can live
    // in multiple renderers — VTK handles that fine. SKIP 2-D actors (HUD text,
    // FPS overlay): those are viewport-relative and would double-draw on top of
    // the PiP looking like a bug.
    vtkRenderer *main = view.renderer();
    vtkPropCollection *props = main->GetViewProps();
    props->InitTraversal();
    while (vtkProp *p = props->GetNextProp())
      if (!vtkActor2D::SafeDownCast(p))
        pipRenderer->AddViewProp(p);
    // The pip_dots overlay is PiP-only: remove from main, keep on PiP.
    if (pipDotNode) {
      vtkProp *dotsProp = pipDotNode->prop();
      if (dotsProp) {
        main->RemoveViewProp(dotsProp);
        pipRenderer->AddViewProp(dotsProp);
      }
    }
    if (!noPip) { // --no-pip skips activation so the PiP never renders (perf diagnostic)
      view.renderWindow()->SetNumberOfLayers(2);
      view.renderWindow()->AddRenderer(pipRenderer);
    }
  }
  // Click-in-PiP → follow. We record the last click and, if it lands over the
  // PiP viewport, ortho-unproject it to a world (x,y), then pick the nearest
  // agent's position and set followAgent.
  auto pip_click_to_follow = [&](double mouseX, double mouseY) {
    // Convert normalized display coords -> the PiP viewport local coords -> world.
    // Viewport is [0.72, 0.0, 1.0, 0.28]; mouseX/mouseY are 0..1 across the window.
    if (mouseX < 0.72 || mouseX > 1.0 || mouseY < 0.0 || mouseY > 0.28)
      return false;
    const double u = (mouseX - 0.72) / (1.0 - 0.72); // 0..1 within PiP
    const double v = mouseY / 0.28;                  // 0..1 within PiP (top->bottom)
    // Ortho unproject: viewport center = (cx,cy), extent = parallelScale * aspect.
    int *vs = view.renderWindow()->GetSize(); // {width, height} — VTK owns this buffer
    const int pw = static_cast<int>((1.0 - 0.72) * vs[0]);
    const int ph = static_cast<int>(0.28 * vs[1]);
    const double aspect = ph > 0 ? static_cast<double>(pw) / ph : 1.0;
    const double sy = pipCam->GetParallelScale();
    const double sx = sy * aspect;
    const double wx = bounds.cx() + (u - 0.5) * 2.0 * sx;
    const double wy = bounds.cy() - (v - 0.5) * 2.0 * sy; // v grows downward in screen coords
    // Nearest-agent lookup using the current snapshot (emPos is refreshed above).
    // Cheap linear scan — for a few hundred agents, negligible.
    std::vector<float> ep(static_cast<std::size_t>(2) * N);
    world.snapshot(ep.data(), nullptr, nullptr, nullptr, nullptr);
    int best = -1;
    double bestD2 = 1e30;
    for (int i = 0; i < N; ++i) {
      const double dx = ep[2 * i] - wx, dy = ep[2 * i + 1] - wy;
      const double d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = i;
      }
    }
    if (best >= 0) {
      followAgent = best;
      configure_track_params();
      cam.setTrackTarget("follow_probe");
      cam.setMode(CameraController::Mode::Track);
      std::printf("nav_city_swarm: PiP click -> follow agent %d (dist=%.1f)\n", best,
                  std::sqrt(bestD2));
    }
    return true;
  };

  std::printf("nav_city_swarm: %d agents, belief=%s (M=%d), grid=%d, %s, shadows=%s\n", N,
              belief.c_str(), M, grid, fogOn ? "fog" : "static-map", shadows ? "on" : "off");

  // Per-frame terrain elevation at each agent's (x,y). Populated once per
  // snapshot, then reused by every pack_z() below (body, flags, pip dots).
  // Shared across the worker AND inline sim branches, so declared BEFORE the
  // #if. NULL-equivalent when terrain is empty — pack_z() then leaves z alone.
  std::vector<double> zoff(N, 0.0);
  auto sample_zoff = [&](const float *p) {
    if (terrain.empty()) {
      std::fill(zoff.begin(), zoff.end(), 0.0);
    } else {
      for (int i = 0; i < N; ++i)
        zoff[i] = terrain.sample(p[2 * i], p[2 * i + 1]);
    }
  };
  const double *zoff_ptr = &zoff[0];

  // 4. Run: sim off-thread; render loop streams poses into the merged glyph mesh.
  // Hand the sim the app's shared fork-join pool so each tick's kernels reuse
  // persistent workers instead of spawning threads per call — and, sized to leave
  // the render thread a core, stop the sim from starving it.
  world.set_thread_pool(&app.computePool());
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
  bool uiFog = fogOn, uiTrails = true, uiGoals = true, uiFlags = true;
  double uiHz = hz;
  float uiTrailW = 1.5f;                              // live path (trail) line width
  bool uiSeparate = true;                             // inter-agent avoidance on/off (live)
  float uiSepGain = 1.5f * static_cast<float>(ts.rr); // avoidance strength (live)
  bool uiScene = false;                               // the library's ScenePanel window
  bool uiLighting = false;                            // the StageLightingPanel window
#ifdef CVC_ENABLE_IMGUI
  // imgui exports no symbols, so this .exe can link its own static copy with its
  // own null GImGui. Adopt the overlay's context so the raw ImGui::* calls below
  // and the cvc::gl::ui::* widgets inside libcvc share one.
  ImGui::SetCurrentContext(ui.imguiContext());
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
        ImGui::MenuItem("Flags", nullptr, &uiFlags);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Scene")) {
        // The library's menu: shadows, shadow-map resolution, bake interval and
        // the diagnostic chrome, all read back from the scene rather than from
        // demo-side mirrors that can drift out of agreement with it.
        cvc::gl::ui::SceneMenuItems(sg, &uiScene, &uiLighting);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Camera")) {
        // The library's menu, off the controller's own state path — this block
        // used to splice ".viewers.main.camera.settings." as a literal in three
        // demos, and it now also carries invert-pitch and drag-pans, which no
        // demo exposed.
        cvc::gl::ui::CameraMenuItems(cam, 400.0, 40.0);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }

    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Swarm controls");
    ImGui::Text("%d agents | %ld arrived | %.0f fps", N, arrived, ImGui::GetIO().Framerate);
    { // periodic fps to stdout so benchmarking doesn't need a screenshot of the HUD
      static int fpsTick = 0;
      if ((++fpsTick % 120) == 30) {
        std::printf("nav_city_swarm: %.1f fps | %d agents\n", ImGui::GetIO().Framerate, N);
        std::fflush(stdout); // stdout is block-buffered when redirected; flush so a
                             // killed benchmark run still yields its fps samples
      }
    }
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
    ImGui::Checkbox("Flags", &uiFlags);
    // Cinematic chase cam (harvested from grl-snam's ChaseCamera). Explicit
    // Checkbox on/off — no more "-1 == off" magic. When on, an index typable
    // in the InputInt AND draggable on the slider picks the agent to CHASE;
    // unchecking snaps the camera back to Orbit and re-frames the whole city.
    static bool chaseOn = false;
    // Keep chaseOn in sync with followAgent for external mutators (e.g. the
    // PiP click-to-follow, which sets followAgent directly).
    if (followAgent >= 0)
      chaseOn = true;
    else if (followAgent < 0)
      chaseOn = false;
    if (ImGui::Checkbox("Chase cam", &chaseOn)) {
      if (chaseOn) {
        if (followAgent < 0)
          followAgent = 0;
        configure_track_params();
        cam.setTrackTarget("follow_probe");
        cam.setMode(CameraController::Mode::Track);
      } else {
        followAgent = -1;
        cam.setMode(CameraController::Mode::Orbit);
        cam.frameBounds(bounds.min_x, bounds.min_y, 0.0, bounds.max_x, bounds.max_y, wall_h);
      }
    }
    if (chaseOn) {
      ImGui::SameLine();
      ImGui::PushItemWidth(60);
      const int followBefore = followAgent;
      ImGui::InputInt("##chaseidx", &followAgent, 1, 10);
      ImGui::PopItemWidth();
      ImGui::SameLine();
      ImGui::PushItemWidth(-90);
      ImGui::SliderInt("##chaseslider", &followAgent, 0, N - 1);
      ImGui::PopItemWidth();
      if (followAgent < 0)
        followAgent = 0;
      if (followAgent >= N)
        followAgent = N - 1;
      if (followAgent != followBefore)
        configure_track_params();
    }
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
    cvc::gl::ui::ScenePanel(sg, &uiScene);
    cvc::gl::ui::StageLightingPanel(*rig, &uiLighting);
  });
#endif

  // Agent-body pack, shared by the worker and inline sim branches. Non-LOD: the
  // original single-node pack. LOD: partition agents by projected size (cvc::lod)
  // into per-rung index buckets (finest rung gets the biggest on-screen agents,
  // bounded by its cap; the rest cascade down), so the vertex upload scales with
  // the near budget, not the whole swarm.
  std::vector<std::pair<float, int>> lodScore;
  std::vector<std::vector<std::uint32_t>> rungIdx(rungs.size());
  if (useLod) {
    lodScore.reserve(N);
    for (auto &ri : rungIdx)
      ri.reserve(N);
  }
  const double agentR = 0.5 * gsz; // vehicle bounding radius ~ half its length
  auto pack_bodies = [&](const float *pos, const float *head, const double *zo) {
    if (!useLod) {
      if (agentNode)
        agentNode->updateVertices(glyphs.pack_z(pos, head, zo));
      return;
    }
    // Eye + fov from the live camera (last render). The camera moves slowly, so
    // a one-frame lag in the partition is invisible.
    cvc::lod::view_params vp = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
    vp.viewport_h_px = height;
    // Six frustum planes for off-screen agent culling. cvc::vis owns the proper
    // culler, but the deps prefix here predates that module, so the demo inlines
    // a minimal test: normalize each plane (so d is a true distance) and cull an
    // agent only when its bounding sphere is fully behind some plane. All six
    // planes are tested, so plane ORDER is irrelevant (no near/far trap).
    double fp[6][4];
    bool haveFr = false;
    if (vtkRenderer *r = view.renderer()) {
      if (vtkCamera *cam3 = r->GetActiveCamera()) {
        cam3->GetPosition(vp.eye);
        vp.tan_half_fov = std::tan(0.5 * cam3->GetViewAngle() * PI / 180.0);
        double p24[24];
        cam3->GetFrustumPlanes(r->GetTiledAspectRatio(), p24);
        for (int pl = 0; pl < 6; ++pl) {
          const double *P = p24 + 4 * pl;
          const double len = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
          const double inv = len > 0.0 ? 1.0 / len : 1.0;
          for (int k = 0; k < 4; ++k)
            fp[pl][k] = P[k] * inv;
        }
        haveFr = true;
      }
    }
    auto in_frustum = [&](const double c[3]) {
      for (int pl = 0; pl < 6; ++pl)
        if (fp[pl][0] * c[0] + fp[pl][1] * c[1] + fp[pl][2] * c[2] + fp[pl][3] < -agentR)
          return false; // fully behind this plane
      return true;
    };
    // Score each IN-FRUSTUM agent by projected radius; off-screen agents are
    // dropped entirely (not packed into either node). This is the chase-cam win:
    // a close camera sees a fraction of the swarm, so most agents cost nothing.
    // The largest on-screen agents go near (full Humvee), the rest far (arrow).
    lodScore.clear();
    for (int i = 0; i < N; ++i) {
      const double c[3] = {pos[2 * i], pos[2 * i + 1], zo ? zo[i] : 0.0};
      if (haveFr && !in_frustum(c))
        continue; // off-screen: culled
      const double d = cvc::lod::bound_distance_m(c, agentR, vp);
      lodScore.push_back({static_cast<float>(cvc::lod::screen_radius_px(agentR, d, vp)), i});
    }
    // Biggest on-screen agents take the finest rung (bounded by its cap); the
    // rest cascade down to coarser rungs. A full sort is cheap for N ~ a few k.
    std::sort(lodScore.begin(), lodScore.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    const std::size_t vis = lodScore.size();
    std::size_t s = 0;
    for (std::size_t r = 0; r < rungs.size(); ++r) {
      LodRung &rung = rungs[r];
      std::vector<std::uint32_t> &idx = rungIdx[r];
      idx.clear();
      const bool last = (r + 1 == rungs.size());
      const std::size_t take =
          last ? (vis - s) : std::min(static_cast<std::size_t>(rung.cap), vis - s);
      for (std::size_t k = 0; k < take; ++k, ++s)
        idx.push_back(static_cast<std::uint32_t>(lodScore[s].second));
      if (!rung.node)
        continue;
      rung.node->updateVertices(
          rung.glyphs.pack_lod(pos, head, zo, idx.data(), static_cast<int>(idx.size())));
      if (!rung.textured) {
        // Re-gather this rung's colours by slot so group identity survives the
        // partition (cheap: count * vpi uchar triples, no transform).
        const int cnt = static_cast<int>(idx.size());
        for (int j = 0; j < cnt; ++j) {
          const int a = idx[j];
          for (int k = 0; k < rung.vpi; ++k) {
            const std::size_t o = (static_cast<std::size_t>(j) * rung.vpi + k) * 3;
            rung.rgb[o] = agentColorU8[3 * a];
            rung.rgb[o + 1] = agentColorU8[3 * a + 1];
            rung.rgb[o + 2] = agentColorU8[3 * a + 2];
          }
        }
        rung.node->updateColors(rung.rgb);
      }
    }
  };

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
    // Click in the PiP corner -> chase whichever agent is nearest the click.
    if (!capturing) {
      ImGuiIO &io = ImGui::GetIO();
      if (ImGui::IsMouseClicked(0) && !io.WantCaptureMouse) {
        const double mx = io.MousePos.x / std::max(io.DisplaySize.x, 1.0f);
        const double my = io.MousePos.y / std::max(io.DisplaySize.y, 1.0f);
        // PiP viewport spans [0.72,0]..[1.0,0.28] but VTK's y=0 is at the
        // BOTTOM and ImGui's y=0 is at the TOP — flip.
        const double myVtk = 1.0 - my;
        if (mx >= 0.72 && myVtk <= 0.28)
          pip_click_to_follow(mx, 0.28 - myVtk);
      }
    }
    // ---- apply UI actions ---------------------------------------------------
    if (trailNode) {
      trailNode->setVisible(uiTrails);
      trailNode->setLineWidth(uiTrailW);
    }
    // Highlighted path for the chased vehicle: extract that agent's slice from
    // the shared trailXyz ring buffer (which trail_update writes each frame)
    // and upload just those K segments to the dedicated pathNode.
    if (pathNode) {
      const bool showPath = (followAgent >= 0 && followAgent < N && uiTrails);
      pathNode->setVisible(showPath);
      if (showPath) {
        const std::size_t base = static_cast<std::size_t>(followAgent) * TRAIL_K * 2 * 3;
        for (std::size_t j = 0; j < static_cast<std::size_t>(TRAIL_K) * 2 * 3; ++j)
          pathXyz[j] = trailXyz[base + j];
        pathNode->updateVertices(pathXyz);
      }
    }
    if (goalsNode)
      goalsNode->setVisible(uiGoals);
    if (flagNode)
      flagNode->setVisible(uiFlags);
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
    // Async world rebuild: kick off build_world on a helper thread the moment
    // uiRestart flips, keep rendering the CURRENT world's snapshots while it
    // runs, then hot-swap on the frame the future is ready. Old flow blocked
    // the render loop for the entire construction (grid + SDF + agent scatter
    // = 300-800 ms on Austin), which is what the "restart flicker" reported.
    // The swap itself still stops the sim, moves the world, and rebuilds the
    // N-sized meshes when N/M changed — that lands in a few frames rather
    // than half a second.
    static std::future<std::unique_ptr<cvc::nav::sim_world>> pendingWorld;
    if (uiRestart && !pendingWorld.valid()) {
      uiRestart = false;
      const int reqAgents = uiAgents;
      const std::string reqBelief = uiBelief;
      const bool reqFog = uiFog;
      const std::uint32_t reqSeed = simSeed;
      pendingWorld = std::async(
          std::launch::async, [=]() { return build_world(reqAgents, reqBelief, reqFog, reqSeed); });
      std::printf("nav_city_swarm: rebuilding world (%d agents, %s, fog=%s, seed=%u) async\n",
                  reqAgents, reqBelief.c_str(), reqFog ? "on" : "off", reqSeed);
    }
    if (pendingWorld.valid() &&
        pendingWorld.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto newWorld = pendingWorld.get();
#if CVC_NAV_DEMO_SIM_WORKER
      sim->stop();
      sim.reset(); // join before the world it borrows changes underneath it
#endif
      const int prevN = N;
      const int prevM = M;
      belief = uiBelief;
      fogOn = uiFog;
      *worldPtr = std::move(*newWorld);
      world.set_thread_pool(&app.computePool()); // move-assign cleared it; re-inject
      N = world.size();
      grp = world.agent_planes();
      M = world.planes();
      color.assign(3 * N, 0.0f);
      for (int i = 0; i < N; ++i) {
        const double hue = (M > 1) ? static_cast<double>(grp[i]) / M : static_cast<double>(i) / N;
        hsv2rgb(hue, 0.62, 0.96, &color[3 * i]);
      }
      const bool rebuild_meshes = (N != prevN) || (M != prevM);
      if (rebuild_meshes && agentNode)
        agentNode->setGeometry(build_agents());
      { // goals + trails are N-sized too
        std::vector<float> gw(static_cast<std::size_t>(2) * N), zeroHead(N, 0.0f);
        world.goals_world(gw.data());
        const double phys = static_cast<double>(cfg.veh.rr) / scale; // matches initial build
        const double gh = std::min(0.006 * span, 2.5 * phys);
        const double ght = std::min(1.2 * wall_h, 22.0 * phys);
        const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                        ght, gh, gh, ght, -gh, gh,  ght};
        const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4,
                                               0, 4, 1, 1, 3, 2, 1, 4, 3};
        if (goalsNode) {
          if (rebuild_meshes)
            goalsNode->setGeometry(goalGlyphs.build_template(app, N, color.data(), pv, pt, 0.0));
          std::vector<double> goalZoff(N, 0.0); // sit beacons on the terrain (see initial build)
          if (!terrain.empty())
            for (int i = 0; i < N; ++i)
              goalZoff[i] = terrain.sample(gw[2 * i], gw[2 * i + 1]);
          goalXyz = goalGlyphs.pack_z(gw.data(), zeroHead.data(), goalZoff.data());
          goalsNode->updateVertices(goalXyz);
        }
        trailLast.assign(static_cast<std::size_t>(2) * N, 0.0f);
        world.snapshot(trailLast.data(), nullptr, nullptr, nullptr, nullptr);
        trailWr.assign(N, 0);
        trailXyz.assign(static_cast<std::size_t>(3) * 2 * TRAIL_K * N, 0.0);
        if (rebuild_meshes) {
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
        } else {
          // Same N: seed trailXyz from current positions; frame loop's
          // trail_update overwrites entries as agents move without a mesh swap.
          for (int i = 0; i < N; ++i)
            for (int k = 0; k < TRAIL_K; ++k)
              for (int e = 0; e < 2; ++e) {
                const std::size_t v = (static_cast<std::size_t>(i) * TRAIL_K + k) * 2 + e;
                trailXyz[3 * v] = trailLast[2 * i];
                trailXyz[3 * v + 1] = trailLast[2 * i + 1];
                trailXyz[3 * v + 2] = 0.35;
              }
          if (trailNode)
            trailNode->updateVertices(trailXyz);
        }
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

    // Fleet fog coverage repaint (sense cadence, not per frame). Skipped when the
    // ground is the satellite photo — the fog belief still shows in the minimap.
    if (fogOn && !haveSat && groundNode && frame % 15 == 0) {
      fill_fleet_fog(fogTex.data(), world, rows, cols);
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
      // Under --lod the body recolour is skipped: agentNode is hidden and the
      // near/far LOD nodes carry their own colour (Humvee texture + far-arrow
      // group tint), so this full-swarm updateColors would be pure waste and
      // would undo the pack win. (v1 trade-off: LOD bodies don't show the
      // reached/mode recolour; the flags and the belief minimap still do.)
      if (!agentNode || useLod || (frame % 2))
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
        sample_zoff(snap->pos.data()); // one bilinear terrain sample per agent
        pack_bodies(snap->pos.data(), snap->heading.data(), zoff_ptr);
        if (flagNode) // flags share the same pose stream, different template
          flagNode->updateVertices(
              flagGlyphs.pack_z(snap->pos.data(), snap->heading.data(), zoff_ptr));
        if (pipDotNode) // PiP-only marker dots, same pose stream (ortho — z irrelevant)
          pipDotNode->updateVertices(pipDotGlyphs.pack(snap->pos.data(), snap->heading.data()));
        if (followAgent >= 0 && followAgent < N) {
          const float *p = snap->pos.data() + 2 * followAgent;
          set_probe_at(p[0], p[1], zoff[followAgent], snap->heading[followAgent]);
        }
        restyle(snap->mode.data(), snap->reached.data());
        trail_update(snap->pos.data());
      }
    }
#else
    if (!simPausedEm)
      world.step(0);
    world.snapshot(emPos.data(), emHead.data(), nullptr, emMd.data(), emRch.data());
    {
      sample_zoff(emPos.data());
      pack_bodies(emPos.data(), emHead.data(), zoff_ptr);
      if (flagNode)
        flagNode->updateVertices(flagGlyphs.pack_z(emPos.data(), emHead.data(), zoff_ptr));
      if (pipDotNode)
        pipDotNode->updateVertices(pipDotGlyphs.pack(emPos.data(), emHead.data()));
      if (followAgent >= 0 && followAgent < N)
        set_probe_at(emPos[2 * followAgent], emPos[2 * followAgent + 1], zoff[followAgent],
                     emHead[followAgent]);
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
    } else if (capturing && capture == "follow") {
      // Cinematic Track camera CHASES the selected agent. cam.update(dt) advances
      // the smoothed pose (position -> heading two-stage ease + critically-damped).
      cam.update(dt);
      if (frame < 3 || frame % 15 == 0) {
        auto wt = probeNode ? probeNode->getWorldTransform() : nullptr;
        double px = wt ? wt->GetElement(0, 3) : 0.0;
        double py = wt ? wt->GetElement(1, 3) : 0.0;
        std::printf(
            "nav_city_swarm: frame %ld  mode=%d  probe=(%.1f,%.1f)  followAgent=%d  dt=%.3f\n",
            frame, (int)cam.mode(), px, py, followAgent, dt);
      }
    } else if (capturing) { // orbit
      const double az = 0.7 + 0.30 * t;
      navdemo::orbit_camera(bounds, wall_h * 0.5, az, 36.0 * PI / 180.0, 1.75, eye, focal);
      view.setCamera(eye[0], eye[1], eye[2], focal[0], focal[1], focal[2], 0, 0, 1, 30);
    } else {
      cam.update(dt);
    }

    // Building LOD swap: draw the full mesh up close, the coarse clustered skyline
    // once the camera pulls back far enough that per-building detail is sub-pixel.
    // Keyed on camera-to-focal distance (chase ~tens of m, overview ~thousands),
    // with a hysteresis band so it doesn't flip-flap at the boundary. Toggling the
    // vtkProp visibility flag is cheap and the PiP (which shares these props)
    // follows automatically.
    if (farBuildings && fullBuildings) {
      if (vtkRenderer *r = view.renderer()) {
        if (vtkCamera *ac = r->GetActiveCamera()) {
          double e[3], f[3];
          ac->GetPosition(e);
          ac->GetFocalPoint(f);
          const double dx = e[0] - f[0], dy = e[1] - f[1], dz = e[2] - f[2];
          const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          const double span2 = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
          static bool farOn = false;
          if (!farOn && dist > 0.30 * span2)
            farOn = true;
          else if (farOn && dist < 0.22 * span2)
            farOn = false;
          if (auto *pf = fullBuildings->prop())
            pf->SetVisibility(farOn ? 0 : 1);
          if (auto *pr = farBuildings->prop())
            pr->SetVisibility(farOn ? 1 : 0);
        }
      }
    }

    if (capturing) {
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", out.c_str(), frame);
      view.writePNG(path);
    } else {
      // Optional render-cost profiling (CVC_RENDER_PROFILE=1): time view.render()
      // (main + shadow + PiP submission) vs the rest of the frame, averaged every
      // 120 frames. Pair with --no-pip to attribute the PiP's share.
      static const bool kRProf = [] {
        const char *e = std::getenv("CVC_RENDER_PROFILE");
        return e && e[0] && e[0] != '0';
      }();
      if (kRProf) {
        using rclk = std::chrono::steady_clock;
        static double accRender = 0.0;
        static long rframes = 0;
        static rclk::time_point prev{};
        const auto t0 = rclk::now();
        view.render();
        const auto t1 = rclk::now();
        accRender += std::chrono::duration<double, std::milli>(t1 - t0).count();
        double frameMs = 0.0;
        if (prev.time_since_epoch().count() != 0) // full frame = render-end to render-end
          frameMs = std::chrono::duration<double, std::milli>(t1 - prev).count();
        prev = t1;
        static double accFrame = 0.0;
        accFrame += frameMs;
        if (++rframes >= 120) {
          const double n = static_cast<double>(rframes);
          std::fprintf(stderr,
                       "CVC_RENDER_PROFILE pip=%s | avg over %ld frames: view.render()=%.1f ms | "
                       "frame=%.1f ms (%.1f fps)\n",
                       noPip ? "off" : "on", rframes, accRender / n, accFrame / n,
                       accFrame > 0 ? 1000.0 * n / accFrame : 0.0);
          std::fflush(stderr);
          accRender = 0.0;
          accFrame = 0.0;
          rframes = 0;
        }
      } else {
        view.render();
      }
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

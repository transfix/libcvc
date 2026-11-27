/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ariadne_swarm -- `type: nav_swarm` and `type: swarm_layer` (swarm_nodes.h, ARIADNE.md §5.4
// "Swarms"), loaded from a document and rendered offscreen, on the synthetic city:
//
//   build    the swarm builds on the synthetic city when it names none, and publishes its stats;
//   layers   each layer makes its meshes, a merged mesh for the whole swarm (a vertex count a whole
//            number of vehicles), and the city layer draws the synthetic city's blocks and ground;
//   stream   the vehicles move: the agents' and flags' points change from frame to frame;
//   lod      a mesh model makes four rungs, the coarser welded and decimated (fewer points a
//            vehicle than the model), and an overview draws every vehicle on the coarsest;
//   humvee   `look: humvee` is the look a world vehicle's is: with vehicle-humvee's parts installed
//            (a prefix of .glb files, CVC_PREFIX) its wheels are in the template, in a tyre's dark
//            on every rung, the coarsest too (each colour role decimated on its own);
//   follow   the follow layer is a node a track camera finds by id, at the followed vehicle;
//            the path layer shows only while one is followed;
//   keys     a layer's visible: follows its key; a target's keys move its beacons;
//   warn     a layer outside a nav_swarm node, and an unknown layer, warn and draw nothing;
//   ghost    an arena with a stale map's phantom wall (the fog-ghost story): the city layer draws
//            its border and the ground, the ghosts layer the phantom as a wall, worn away as the
//            vehicle's sensor clears it; plan, route and sensor stream with the vehicle.
//
// Skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1. Synthetic data only.

#include "streaming_test_util.h"

#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/swarm_nodes.h>
#include <cvc/model/model_file_io.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>

using namespace cvcgl_test;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::gl::ariadne;

namespace {

const std::string kPrefix = "doc";
constexpr int W = 320, H = 240;

std::string key(const std::string &k) { return kPrefix + "." + k; }

// A box 4 x 2 x 1.5 with a lower, shorter hood at +x: a model the vehicle template orients.
void write_model(const std::filesystem::path &file) {
  std::ofstream o(file);
  const auto box = [&](double x0, double y0, double z0, double x1, double y1, double z1, int base) {
    const double v[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                            {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
    for (const auto &p : v)
      o << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
    const int f[12][3] = {{1, 3, 2}, {1, 4, 3}, {5, 6, 7}, {5, 7, 8}, {1, 2, 6}, {1, 6, 5},
                          {2, 3, 7}, {2, 7, 6}, {3, 4, 8}, {3, 8, 7}, {4, 1, 5}, {4, 5, 8}};
    for (const auto &t : f)
      o << "f " << base + t[0] << " " << base + t[1] << " " << base + t[2] << "\n";
  };
  box(-2.0, -1.0, 0.0, 0.8, 1.0, 1.5, 0); // the cabin
  box(0.8, -0.9, 0.0, 2.0, 0.9, 0.8, 8);  // the hood
}

// vehicle-humvee's parted Humvee as cvcpkg installs it, under `prefix`: share/cvc-scenes/shared's
// Humvee.glb and humvee/{chassis.glb, wheel.glb, rig.json} (the model above as the chassis and the
// one piece, a 0.9 m wheel). False where this build writes no .glb.
bool write_humvee_prefix(cvc::app &app, const std::filesystem::path &obj,
                         const std::filesystem::path &prefix) {
  namespace fs = std::filesystem;
  if (!cvc::can_write_model_to_memory(".glb") || !cvc::can_read_model_from_memory(".glb"))
    return false;
  const fs::path shared = prefix / "share" / "cvc-scenes" / "shared";
  std::error_code ec;
  fs::create_directories(shared / "humvee", ec);
  const fs::path wheel_obj = prefix / "wheel.obj";
  {
    std::ofstream o(wheel_obj);
    for (int k = 0; k < 8; ++k)
      o << "v " << (k & 1 ? 0.45 : -0.45) << " " << (k & 2 ? 0.15 : -0.15) << " "
        << (k & 4 ? 0.45 : -0.45) << "\n";
    const int f[12][3] = {{1, 3, 4}, {1, 4, 2}, {5, 6, 8}, {5, 8, 7}, {1, 2, 6}, {1, 6, 5},
                          {3, 7, 8}, {3, 8, 4}, {1, 5, 7}, {1, 7, 3}, {2, 4, 8}, {2, 8, 6}};
    for (const auto &t : f)
      o << "f " << t[0] << " " << t[1] << " " << t[2] << "\n";
  }
  try {
    const cvc::model car = cvc::read_model(app, obj.string());
    const cvc::model wheel = cvc::read_model(app, wheel_obj.string());
    std::ofstream(shared / "Humvee.glb", std::ios::binary)
        << cvc::write_model_to_memory(app, car, ".glb");
    std::ofstream(shared / "humvee" / "chassis.glb", std::ios::binary)
        << cvc::write_model_to_memory(app, car, ".glb");
    std::ofstream(shared / "humvee" / "wheel.glb", std::ios::binary)
        << cvc::write_model_to_memory(app, wheel, ".glb");
  } catch (const std::exception &e) {
    std::printf("  the humvee prefix: %s\n", e.what());
    return false;
  }
  std::ofstream(shared / "humvee" / "rig.json")
      << R"({ "radius": 0.45, "hubs": [[1.4, 0.95, 0.45], [1.4, -0.95, 0.45], [-1.4, 0.95, 0.45],
  [-1.4, -0.95, 0.45]] })";
  return true;
}

// An environment variable set for the test (unset when empty).
void set_env(const char *name, const std::string &value) {
#ifdef _WIN32
  _putenv_s(name, value.c_str());
#else
  if (value.empty())
    unsetenv(name);
  else
    setenv(name, value.c_str(), 1);
#endif
}

// How many of a mesh's points are a tyre's dark (vehicle_view's 18, 18, 20).
std::size_t tyre_points(const std::shared_ptr<GeometryNode> &g) {
  std::size_t n = 0;
  if (g && g->getGeometry())
    for (const auto &c : g->getGeometry()->const_colors())
      if (std::fabs(c[0] - 18.0 / 255.0) < 1e-3 && std::fabs(c[2] - 20.0 / 255.0) < 1e-3)
        ++n;
  return n;
}

std::string doc_text(const std::string &model, bool humvee) {
  return R"(meta: { min_libcvc: "0.0.0" }
scene:
  clock: { scale: 3.6 }
  main_view: { hide: [minimap] }
  nodes:
    - node: swarm
      type: nav_swarm
      agents: 48
      grid: 40
      targets: 4
      belief: grouped
      fog: true
      policy: default
      children:
        - { node: swarm_city, type: swarm_layer, layer: city }
        - { node: swarm_agents, type: swarm_layer, layer: agents, look: arrow }
        - { node: swarm_models, type: swarm_layer, layer: agents, look: ")" +
         model + R"(", lod: true }
)" +
         (humvee
              ? "        - { node: swarm_humvee, type: swarm_layer, layer: agents, look: humvee }\n"
              : "") +
         R"(        - { node: swarm_flags, type: swarm_layer, layer: flags, visible: show.flags }
        - { node: swarm_goals, type: swarm_layer, layer: goals }
        - { node: swarm_trails, type: swarm_layer, layer: trails, every: 1 }
        - { node: swarm_path, type: swarm_layer, layer: path }
        - { node: swarm_belief, type: swarm_layer, layer: belief, material: { opacity: 0.55 } }
        - { node: swarm_dots, type: swarm_layer, layer: dots, tags: [minimap] }
        - { node: swarm_targets, type: swarm_layer, layer: targets, tags: [minimap] }
        - { node: chase, type: swarm_layer, layer: follow }
        - { node: swarm_bogus, type: swarm_layer, layer: bogus }
    - { node: stray, type: swarm_layer, layer: flags }
)";
}

// The fog-ghost arena: one vehicle on a belief route through a phantom wall its map shows.
const char *kGhostDoc = R"(meta: { min_libcvc: "0.0.0" }
scene:
  clock: { scale: 3.6 }
  nodes:
    - node: swarm
      type: nav_swarm
      keys: ghost
      grid: 120
      belief: shared
      fog: true
      policy: default
      reach: 0.2
      sense_ticks: 2
      range: 35
      route: belief
      arena: { size: 200, phantoms: [[-0.5, -50, 0.5, 50]], wall_height: 6 }
      vehicles:
        - { start: [-80, -6], goal: [80, 6], color: [0.98, 0.85, 0.20] }
      children:
        - { node: g_city, type: swarm_layer, layer: city, belief: true }
        - { node: g_ghosts, type: swarm_layer, layer: ghosts }
        - { node: g_plan, type: swarm_layer, layer: plan }
        - { node: g_route, type: swarm_layer, layer: route }
        - { node: g_sensor, type: swarm_layer, layer: sensor }
)";

std::shared_ptr<GeometryNode> mesh_of(SceneGraph &sg, const std::string &layer,
                                      const std::string &suffix) {
  std::shared_ptr<GraphicsNode> swarm = sg.getGraphics("swarm");
  std::shared_ptr<GraphicsNode> l = swarm ? swarm->findChildByName(layer) : nullptr;
  std::shared_ptr<GraphicsNode> m = l ? l->findChildByName(layer + suffix) : nullptr;
  return std::dynamic_pointer_cast<GeometryNode>(m);
}

std::size_t points_of(const std::shared_ptr<GeometryNode> &g) {
  return g && g->getGeometry() ? g->getGeometry()->num_points() : 0;
}

// The points the node draws now (streamed ones included: updateVertices writes the polydata).
std::vector<double> drawn_xy(const std::shared_ptr<GeometryNode> &g) {
  std::vector<double> xy;
  vtkActor *actor = g ? vtkActor::SafeDownCast(g->prop()) : nullptr;
  vtkPolyData *pd = actor && actor->GetMapper()
                        ? vtkPolyData::SafeDownCast(actor->GetMapper()->GetInputAsDataSet())
                        : nullptr;
  if (!pd || !pd->GetPoints())
    return xy;
  for (vtkIdType i = 0; i < pd->GetNumberOfPoints(); ++i) {
    double p[3];
    pd->GetPoint(i, p);
    xy.push_back(p[0]);
    xy.push_back(p[1]);
  }
  return xy;
}

double moved(const std::vector<double> &a, const std::vector<double> &b) {
  double d = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i)
    d += std::fabs(a[i] - b[i]);
  return d;
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  std::printf("cvcgl_ariadne_swarm\n");
  if (!renderAvailable(app))
    return finish("cvcgl_ariadne_swarm");
  ari::register_swarm_node_types(app);

  std::random_device rd;
  const std::filesystem::path dir = std::filesystem::temp_directory_path() /
                                    ("cvcgl-swarm-" + std::to_string(rd()) + std::to_string(rd()));
  std::filesystem::create_directories(dir);
  const std::filesystem::path model = dir / "car.obj";
  write_model(model);
  // The Humvee in parts where the swarm looks for it, and nowhere else.
  const bool humvee = write_humvee_prefix(app, model, dir / "prefix");
  if (humvee) {
    set_env("CVC_PREFIX", (dir / "prefix").string());
    for (const char *v : {"CVCPKG_PREFIX", "CONDA_PREFIX", "CVC_NAV_BUNDLE"})
      set_env(v, "");
  } else {
    std::printf("  humvee: skipped (no .glb writer in this build)\n");
  }

  cvc::ariadne::LoadResult doc =
      cvc::ariadne::load_string(app, doc_text(model.generic_string(), humvee));
  if (!check(doc.ok, "the document loads", doc.error))
    return finish("cvcgl_ariadne_swarm");

  SceneGraph sg(app, "swarm");
  sg.setDiagnosticChromeVisible(false);
  std::vector<std::string> warnings;
  ari::RealizedScene rs = ari::realize_scene(sg, doc.scene, kPrefix, &warnings);
  bool warned_bogus = false, warned_stray = false;
  for (const std::string &w : warnings) {
    warned_bogus = warned_bogus || w.find("no layer 'bogus'") != std::string::npos;
    warned_stray = warned_stray || w.find("under a nav_swarm node") != std::string::npos;
  }
  check(warned_bogus, "warn: an unknown layer says so");
  check(warned_stray, "warn: a layer outside a nav_swarm node says so");

  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  const auto frame = [&]() {
    cvc::ariadne::sync_scene_visibility(app, rs.visibility);
    ari::tick_scene(rs, sr.renderer(), 1.0 / 30.0);
    sr.render();
  };
  // The build is off the frame's thread: frames until it is in.
  for (int i = 0; i < 600 && cvc::ariadne::read_or<int>(app, key("swarm.stats.ready"), 0) != 1; ++i)
    frame();
  if (!check(cvc::ariadne::read_or<int>(app, key("swarm.stats.ready"), 0) == 1,
             "build: the swarm is in"))
    return finish("cvcgl_ariadne_swarm");
  frame();
  check(cvc::ariadne::read_or<int>(app, key("swarm.stats.agents"), 0) == 48,
        "build: stats.agents is 48");
  check(cvc::ariadne::read_or<std::string>(app, key("swarm.stats.city"), "") ==
            "the synthetic city",
        "build: no city: the synthetic one");
  check(cvc::ariadne::read_or<int>(app, key("swarm.stats.planes"), 0) == 2,
        "build: grouped belief, 48 agents: 2 planes");

  // --- layers -------------------------------------------------------------------------------
  const auto arrows = mesh_of(sg, "swarm_agents", "_body");
  const auto flags = mesh_of(sg, "swarm_flags", "_mesh");
  const auto goals = mesh_of(sg, "swarm_goals", "_mesh");
  const auto trails = mesh_of(sg, "swarm_trails", "_lines");
  const auto path = mesh_of(sg, "swarm_path", "_lines");
  const auto dots = mesh_of(sg, "swarm_dots", "_mesh");
  check(points_of(arrows) == 48u * 3u, "agents: an arrow (3 points) a vehicle");
  check(points_of(flags) == 48u * 12u, "flags: a pole and a flag (12 points) a vehicle");
  check(points_of(goals) == 48u * 5u, "goals: a beacon (5 points) a vehicle");
  check(points_of(trails) == 48u * 12u * 2u, "trails: 12 segments a vehicle");
  check(points_of(path) == 12u * 2u, "path: one vehicle's 12 segments");
  check(points_of(dots) == 48u * 5u, "dots: a pentagon a vehicle");
  check(points_of(mesh_of(sg, "swarm_city", "_walls")) > 0 &&
            points_of(mesh_of(sg, "swarm_city", "_ground")) == 4,
        "city: the synthetic city's blocks and ground");
  for (int i = 0; i < 6; ++i)
    frame();
  check(points_of(mesh_of(sg, "swarm_targets", "_mesh")) == 4u * 2u * 25u,
        "targets: a disc and its ring (25 points each) a target");

  // --- lod ----------------------------------------------------------------------------------
  {
    std::size_t rungs = 0, coarse = 0;
    for (int k = 0; k < 4; ++k)
      if (const auto g = mesh_of(sg, "swarm_models", "_lod" + std::to_string(k)); g) {
        ++rungs;
        if (k == 3)
          coarse = points_of(g);
      }
    check(rungs == 4, "lod: a model makes four rungs");
    check(coarse > 0 && coarse % 48 == 0 && coarse / 48 <= 16,
          "lod: the coarsest rung is welded (at most the model's 16 corners a vehicle)",
          std::to_string(coarse));
  }

  // --- humvee -------------------------------------------------------------------------------
  if (humvee) {
    const auto fine = mesh_of(sg, "swarm_humvee", "_lod0");
    const auto coarse = mesh_of(sg, "swarm_humvee", "_lod3");
    check(fine && coarse, "humvee: `look: humvee` makes its rungs");
    const std::size_t fine_tyre = tyre_points(fine), coarse_tyre = tyre_points(coarse);
    std::printf("  humvee: the finest rung %zu points (%zu a tyre's), the coarsest %zu (%zu)\n",
                points_of(fine), fine_tyre, points_of(coarse), coarse_tyre);
    check(fine_tyre > 0 && fine_tyre < points_of(fine),
          "humvee: in parts, its wheels a tyre's dark and its chassis its colour");
    check(coarse_tyre > 0 && coarse_tyre % 48 == 0 && points_of(coarse) % 48 == 0,
          "humvee: the coarsest rung keeps every vehicle's wheels dark");
  }

  // --- stream -------------------------------------------------------------------------------
  {
    const std::vector<double> first = drawn_xy(arrows), flags0 = drawn_xy(flags);
    for (int i = 0; i < 30; ++i)
      frame();
    check(moved(first, drawn_xy(arrows)) > 0.0, "stream: the vehicles move from frame to frame");
    check(moved(flags0, drawn_xy(flags)) > 0.0, "stream: their flags go with them");
  }

  // --- follow -------------------------------------------------------------------------------
  {
    check(path && !path->isVisible(), "follow: the path is hidden while none is followed");
    cvc::ariadne::write(app, key("swarm.follow"), 3);
    for (int i = 0; i < 3; ++i)
      frame();
    std::shared_ptr<GraphicsNode> chase = sg.getGraphics("chase");
    check(static_cast<bool>(chase), "follow: a track camera finds the follow node by its id");
    const double fx = cvc::ariadne::read_or<double>(app, key("swarm.follow_pose.x"), 1e9);
    const double fy = cvc::ariadne::read_or<double>(app, key("swarm.follow_pose.y"), 1e9);
    if (chase) {
      vtkSmartPointer<vtkMatrix4x4> m = chase->getWorldTransform();
      check(m && std::fabs(m->GetElement(0, 3) - fx) < 1e-3 &&
                std::fabs(m->GetElement(1, 3) - fy) < 1e-3,
            "follow: the node sits at the followed vehicle");
    }
    check(path && path->isVisible(), "follow: the path shows while one is followed");
  }

  // --- keys ---------------------------------------------------------------------------------
  {
    cvc::ariadne::write(app, key("show.flags"), 0);
    frame();
    std::shared_ptr<GraphicsNode> fl = sg.getGraphics("swarm")->findChildByName("swarm_flags");
    check(fl && !fl->isVisible(), "keys: the flags follow show.flags");
    const std::vector<double> before = drawn_xy(goals);
    cvc::ariadne::write(app, key("swarm.targets.0.x"), 1.0);
    cvc::ariadne::write(app, key("swarm.targets.0.y"), 2.0);
    frame();
    frame();
    check(moved(before, drawn_xy(goals)) > 0.0, "keys: a target moved takes its beacons with it");
  }

  // --- ghost --------------------------------------------------------------------------------
  {
    cvc::app gapp;
    ari::register_swarm_node_types(gapp);
    cvc::ariadne::LoadResult gdoc = cvc::ariadne::load_string(gapp, kGhostDoc);
    if (!check(gdoc.ok, "ghost: the document loads", gdoc.error))
      return finish("cvcgl_ariadne_swarm");
    SceneGraph gsg(gapp, "ghost");
    gsg.setDiagnosticChromeVisible(false);
    std::vector<std::string> gw;
    ari::RealizedScene grs = ari::realize_scene(gsg, gdoc.scene, kPrefix, &gw);
    SceneRenderer gsr(gsg, W, H, /*offscreen=*/true);
    const auto gframe = [&]() {
      ari::tick_scene(grs, gsr.renderer(), 1.0 / 30.0);
      gsr.render();
    };
    const auto gnum = [&](const char *k) {
      return cvc::ariadne::read_or<int>(gapp, key(std::string("ghost.") + k), -1);
    };
    for (int i = 0; i < 600 && gnum("stats.ready") != 1; ++i)
      gframe();
    gframe();
    check(gnum("stats.ready") == 1, "ghost: the swarm is in on its arena");
    check(cvc::ariadne::read_or<std::string>(gapp, key("ghost.stats.city"), "") == "the arena",
          "ghost: stats.city names the arena");
    check(gnum("stats.ghosts0") == 60, "ghost: the phantom is 60 cells of the map",
          std::to_string(gnum("stats.ghosts0")));
    check(points_of(mesh_of(gsg, "g_city", "_walls")) > 0 &&
              points_of(mesh_of(gsg, "g_city", "_ground")) == 4,
          "ghost: the city layer draws the arena's border and its ground");
    const auto ghosts = mesh_of(gsg, "g_ghosts", "_mesh");
    const std::size_t ghost0 = points_of(ghosts);
    check(ghost0 > 0, "ghost: the ghosts layer stands the phantom up as a wall");
    vtkActor *ga = ghosts ? vtkActor::SafeDownCast(ghosts->prop()) : nullptr;
    check(ga && std::fabs(ga->GetProperty()->GetOpacity() - 0.45) < 1e-6,
          "ghost: see-through (0.45)");
    check(points_of(mesh_of(gsg, "g_plan", "_lines")) == 4u,
          "ghost: the plan is two segments a vehicle");
    check(points_of(mesh_of(gsg, "g_sensor", "_lines")) == 72u * 2u,
          "ghost: the sensor ring is 72 segments a vehicle");
    check(points_of(mesh_of(gsg, "g_route", "_lines")) >= 2u,
          "ghost: the route round the phantom is drawn");
    const std::vector<double> plan0 = drawn_xy(mesh_of(gsg, "g_plan", "_lines"));
    const std::vector<double> ring0 = drawn_xy(mesh_of(gsg, "g_sensor", "_lines"));
    bool worn = false;
    for (int i = 0; i < 900 && !worn; ++i) {
      gframe();
      worn = gnum("stats.ghosts") >= 0 && gnum("stats.ghosts") <= 45; // a quarter of it
    }
    check(worn, "ghost: the sensor wears the phantom away", std::to_string(gnum("stats.ghosts")));
    const auto ghosts_now = mesh_of(gsg, "g_ghosts", "_mesh");
    check(ghosts_now && (!ghosts_now->isVisible() || points_of(ghosts_now) < ghost0),
          "ghost: the wall the ghosts layer draws wears away with it");
    check(moved(plan0, drawn_xy(mesh_of(gsg, "g_plan", "_lines"))) > 0.0,
          "ghost: the plan streams with the vehicle");
    check(moved(ring0, drawn_xy(mesh_of(gsg, "g_sensor", "_lines"))) > 0.0,
          "ghost: the sensor ring goes with it");
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  return finish("cvcgl_ariadne_swarm");
}

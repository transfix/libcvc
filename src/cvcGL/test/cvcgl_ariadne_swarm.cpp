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
//   follow   the follow layer is a node a track camera finds by id, at the followed vehicle;
//            the path layer shows only while one is followed;
//   keys     a layer's visible: follows its key; a target's keys move its beacons;
//   warn     a layer outside a nav_swarm node, and an unknown layer, warn and draw nothing.
//
// Skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1. Synthetic data only.

#include "streaming_test_util.h"

#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/swarm_nodes.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>

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

std::string doc_text(const std::string &model) {
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
        - { node: swarm_flags, type: swarm_layer, layer: flags, visible: show.flags }
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

  cvc::ariadne::LoadResult doc = cvc::ariadne::load_string(app, doc_text(model.generic_string()));
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

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  return finish("cvcgl_ariadne_swarm");
}

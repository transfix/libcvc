/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// boat_node.cpp — the `boat` scene node drawn (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D5, D6).
// The scene's world runs the boat (cvc::ariadne::BoatRuntime, scene_boat.h: in its physics world,
// afloat on its water, from its keys or by nav); this draws its hull (add_hull_node: white
// topsides, a stripe, the wet bottom dark) at the boat's pose, bound to its hull body once the boat
// is in the world, and hidden until then; or, with a model (its `model` prop, or the tactical
// preset's TacticalBoat), that model fitted to the boat's length overall, its keel on the hull's
// bottom. The water node keeps its water out of the hull (BoatRuntime::cutout).

#include <algorithm>
#include <array>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_boat.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_vehicles.h> // find_vehicle_model
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h> // add_hull_node, attach_vehicle_model
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/gl/physics_scene_binding.h>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// The model boat `n` is drawn as: its `model` prop (a file, or a name found where the vehicle
// models are, `<name>.glb` too; none: its hull), else for the tactical preset TacticalBoat. ""
// draws the hull: none asked for, or the tactical preset's model not installed (quietly; a `model`
// that is not found warns).
std::string boat_model_path(const ari::SceneNode &n, const std::string &preset,
                            std::vector<std::string> *warnings) {
  namespace fs = std::filesystem;
  const bool asked = n.props.has("model");
  const std::string name =
      asked ? n.props.str("model") : (preset == "tactical" ? "TacticalBoat" : std::string());
  if (name.empty() || name == "none")
    return std::string();
  std::error_code ec;
  if (fs::is_regular_file(fs::path(name), ec))
    return name;
  std::string path = ari::find_vehicle_model(name);
  if (path.empty() && fs::path(name).extension().empty())
    path = ari::find_vehicle_model(name + ".glb");
  if (path.empty() && asked && warnings)
    warnings->push_back("ari: boat '" + n.id + "': no model '" + name + "'; drawn as its hull");
  return path;
}

// The fitted model under `group`: the boat's length overall (its hull's when it gives none),
// about the hull body's origin, its keel at the hull's bottom. False when there is no hull or the
// model cannot be read (a warning says why).
bool attach_boat_model(SceneGraph &sg, RealizedScene &out, GraphicsNode &group,
                       const std::string &id, const std::string &path,
                       const cvc::physics::dyn_boat_desc &desc, const float color[3],
                       std::vector<std::string> *warnings) {
  const cvc::physics::boat_desc &boat = desc.boat;
  if (!boat.hull.hull || boat.hull.hull->vertices.empty())
    return false;
  double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300, z0 = 1e300, z1 = -1e300;
  for (const cvc::physics::vec3 &v : boat.hull.hull->vertices) {
    x0 = std::min(x0, v.x);
    x1 = std::max(x1, v.x);
    y0 = std::min(y0, v.y);
    y1 = std::max(y1, v.y);
    z0 = std::min(z0, v.z);
    z1 = std::max(z1, v.z);
  }
  if (!out.vehicle_models)
    out.vehicle_models = make_vehicle_model_cache();
  ari::vehicle_look look;
  look.kind = "file";
  look.path = path;
  look.fit = true;
  vehicle_shape shape;
  shape.length = desc.length_overall > 0.0 ? desc.length_overall : x1 - x0;
  shape.width = y1 - y0;
  shape.height = z1 - z0;
  return attach_vehicle_model(sg.appContext(), *out.vehicle_models, group, id, look, shape, z0,
                              color, warnings);
}

std::shared_ptr<GraphicsNode> realize_boat(SceneGraph &sg, const ari::SceneNode &n,
                                           GraphicsNode *parent, RealizedScene &out,
                                           std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> group = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const ari::SceneBoatSpec spec = ari::scene_boat_spec(n, identity, sg.getStatePrefix(), warnings);
  float topsides[3] = {0.93f, 0.93f, 0.90f};
  if (n.has_material)
    std::copy(n.color, n.color + 3, topsides);
  static const float stripe[3] = {0.10f, 0.24f, 0.56f};
  const std::string model = boat_model_path(n, spec.preset, warnings);
  if (model.empty() ||
      !attach_boat_model(sg, out, *group, n.id + "_model", model, spec.desc, topsides, warnings))
    add_hull_node(sg, n.id + "_hull", group.get(), spec.desc.boat, topsides, stripe);
  group->setVisible(false); // until its boat is in the world
  // Bound to the boat's hull body once there is one (and again when a restart makes another).
  const std::weak_ptr<GraphicsNode> weak = group;
  auto bound = std::make_shared<cvc::physics::body_id>();
  const std::string id = n.id;
  out.scene_ticks.push_back([weak, id, bound](vtkRenderer *, RealizedScene &scene,
                                              const cvc::world_clock::step_result &) {
    const std::shared_ptr<GraphicsNode> node = weak.lock();
    if (!node || !scene.world || !scene.physics_binding)
      return;
    const ari::BoatRuntime *b = scene.world->boat(id);
    const cvc::physics::body_id hull = b ? b->hull() : cvc::physics::body_id{};
    if (hull != *bound) {
      if (bound->valid() && scene.physics_binding->bound(*bound))
        scene.physics_binding->unbind(*bound);
      if (hull.valid())
        try {
          scene.physics_binding->bind(hull, node);
        } catch (const std::exception &) {
        }
      *bound = hull;
    }
    node->setVisible(hull.valid());
  });
  return group;
}

} // namespace

void register_boat_node_types(cvc::app &app) {
  register_scene_node_type(app, "boat", realize_boat);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

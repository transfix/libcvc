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
// is in the world, and hidden until then; or, with a model (its `model` prop: a file, a model's
// name, or a file in a document data: entry's package, fetched as the document loads; or the
// tactical preset's TacticalBoat), that model fitted to the boat's length overall, its keel on the
// hull's bottom, in place of the hull once it is ready. The water node keeps its water out of the
// hull (BoatRuntime::cutout).

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
#include <cvc/gl/ariadne/vehicle_view.h> // add_hull_node, attach_vehicle_model, boat_fit_of
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/gl/physics_scene_binding.h>
#include <exception>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// The look boat `n` is drawn as: its `model` prop (node_model_look: `{ data: NAME, file: FILE }`,
// a file, a model's name, or none), else for the tactical preset TacticalBoat (a local install). A
// look of kind "" draws the hull: none asked for, or the tactical preset's model not installed
// (quietly; a `model` that is not found warns).
ari::vehicle_look boat_look(const ari::SceneNode &n, const std::string &preset,
                            const std::vector<ari::SceneData> &data,
                            std::vector<std::string> *warnings) {
  if (const ari::Value *m = n.props.find("model"))
    return node_model_look(*m, data, "ari: boat '" + n.id + "'", "its hull", warnings);
  ari::vehicle_look look;
  look.fit = true;
  if (preset != "tactical")
    return look;
  std::string path = ari::find_vehicle_model("TacticalBoat");
  if (path.empty())
    path = ari::find_vehicle_model("TacticalBoat.glb");
  if (!path.empty()) {
    look.kind = "file";
    look.path = path;
  }
  return look;
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
  // Its model, when it has one and it is ready; else its hull, and while the model's package
  // loads, the model goes in in its place once it has (the scene tick below).
  if (!out.vehicle_models)
    out.vehicle_models = make_vehicle_model_cache();
  const ari::vehicle_look look = boat_look(n, spec.preset, out.data, warnings);
  const boat_fit fit = boat_fit_of(spec.desc);
  const std::array<float, 3> color{topsides[0], topsides[1], topsides[2]};
  vehicle_model_status status = vehicle_model_status::failed;
  if (!look.kind.empty() && fit.ok)
    status =
        vehicle_model_state(sg.appContext(), *out.vehicle_models, look, fit.shape.length, warnings);
  const std::string model_id = n.id + "_model", hull_id = n.id + "_hull";
  if (status != vehicle_model_status::ready ||
      !attach_vehicle_model(sg.appContext(), *out.vehicle_models, *group, model_id, look, fit.shape,
                            fit.keel, color.data(), warnings))
    add_hull_node(sg, hull_id, group.get(), spec.desc.boat, topsides, stripe);
  if (status == vehicle_model_status::pending) {
    const std::weak_ptr<GraphicsNode> held = group;
    auto done = std::make_shared<bool>(false);
    out.scene_ticks.push_back(
        [held, done, look, fit, color, model_id, hull_id](vtkRenderer *, RealizedScene &scene,
                                                          const cvc::world_clock::step_result &) {
          const std::shared_ptr<GraphicsNode> g = held.lock();
          if (*done || !g || !scene.sg || !scene.vehicle_models)
            return;
          cvc::app &app = scene.sg->appContext();
          const vehicle_model_status now =
              vehicle_model_state(app, *scene.vehicle_models, look, fit.shape.length);
          if (now == vehicle_model_status::pending)
            return;
          *done = true; // failed: the hull stays
          if (now == vehicle_model_status::ready &&
              attach_vehicle_model(app, *scene.vehicle_models, *g, model_id, look, fit.shape,
                                   fit.keel, color.data()))
            if (std::shared_ptr<GraphicsNode> hull = g->findChildByName(hull_id))
              g->removeGraphicsChild(hull);
        });
  }
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

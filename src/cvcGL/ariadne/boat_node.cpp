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
// is in the world, and hidden until then. The water node keeps its water out of the hull.

#include <array>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_boat.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h> // add_hull_node
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/gl/physics_scene_binding.h>
#include <exception>
#include <memory>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

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

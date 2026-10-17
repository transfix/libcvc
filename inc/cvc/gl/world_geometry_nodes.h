/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::gl::add_world_geometry — the world's geometry as scene-graph nodes: the
// same height fields and meshes the physics world collides with
// (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3).
//
//   const auto city = cvc::world::load_city_bundle(dir);
//   const auto nodes = cvc::gl::add_world_geometry(sg, city.geometry);
//
// One GeometryNode per item with role_render, named prefix + item name. The node
// shares the item's arrays (cvc::geometry is copy-on-write). A geometry with no
// vertex colours of its own is coloured by its surfaces: each vertex takes the
// mean albedo of the faces around it, so a terrain shows its land cover and a
// building its shell. A mesh item's placement becomes the node's pose.
//
// Call it on the scene's owner thread, like SceneGraph::addGraphics.

#ifndef __CVC_GL_WORLD_GEOMETRY_NODES_H__
#define __CVC_GL_WORLD_GEOMETRY_NODES_H__

#include <cvc/world/world_geometry.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {

class GraphicsNode;
class SceneGraph;

std::vector<std::shared_ptr<GraphicsNode>>
add_world_geometry(SceneGraph &sg, const cvc::world::world_geometry &g,
                   const std::string &prefix = std::string());

// The colours add_world_geometry gives a geometry without its own: per vertex, the
// mean albedo of its faces' surfaces (`surface` for faces without one).
cvc::geometry::colors_t surface_colors(const cvc::geometry &geom,
                                       const cvc::world::surface_registry &surfaces,
                                       std::uint16_t surface);

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_WORLD_GEOMETRY_NODES_H__

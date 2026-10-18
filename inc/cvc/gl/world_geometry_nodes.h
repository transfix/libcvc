/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::gl::add_world_geometry — the world's geometry as scene-graph nodes: the
// same height fields and meshes the physics world collides with, and the same
// volumes its fields push with (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3 and
// G6).
//
//   const auto city = cvc::world::load_city_bundle(dir);
//   const auto nodes = cvc::gl::add_world_geometry(sg, city.geometry);
//
// One GeometryNode per item with role_render, named prefix + item name. The node
// shares the item's arrays (cvc::geometry is copy-on-write). A geometry with no
// vertex colours of its own is coloured by its surfaces: each vertex takes the
// mean albedo of the faces around it, so a terrain shows its land cover and a
// building its shell. A mesh item's placement becomes the node's pose. After the
// geometry nodes, one VolumeNode per volume with role_render, named prefix +
// volume name: it shares the volume's voxels and sits at the volume's bounding
// box. The nodes come back in that order: items, then volumes.
//
// `parent` is the node that stands for the world's frame: the nodes are made its
// children (top-level nodes without one), so a transform on it (a render origin,
// a scale) moves the whole world. Give the same node to
// cvc::gl::physics_scene_binding::set_frame and the bodies share that frame.
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
                   const std::string &prefix = std::string(),
                   const std::shared_ptr<GraphicsNode> &parent = nullptr);

// The colours add_world_geometry gives a geometry without its own: per vertex, the
// mean albedo of its faces' surfaces (`surface` for faces without one).
cvc::geometry::colors_t surface_colors(const cvc::geometry &geom,
                                       const cvc::world::surface_registry &surfaces,
                                       std::uint16_t surface);

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_WORLD_GEOMETRY_NODES_H__

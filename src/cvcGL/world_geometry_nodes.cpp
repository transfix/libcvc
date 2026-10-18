/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/world_geometry_nodes.h>
#include <cvc/physics/pose_tracker.h>

namespace cvc {
namespace gl {

cvc::geometry::colors_t surface_colors(const cvc::geometry &geom,
                                       const cvc::world::surface_registry &surfaces,
                                       std::uint16_t surface) {
  const auto &pts = geom.const_points();
  cvc::geometry::colors_t colors(pts.size(), cvc::geometry::color_t{{0.0, 0.0, 0.0}});
  std::vector<int> faces(pts.size(), 0);
  auto add = [&](cvc::index_t v, std::uint16_t s) {
    if (v >= pts.size())
      return;
    const float *a = surfaces[s < surfaces.size() ? s : surface].albedo;
    colors[v][0] += a[0];
    colors[v][1] += a[1];
    colors[v][2] += a[2];
    ++faces[v];
  };
  const auto &tris = geom.const_tris();
  for (std::size_t i = 0; i < tris.size(); ++i) {
    const std::uint16_t s = geom.has_face_materials() ? geom.face_material(i) : surface;
    for (const cvc::index_t v : tris[i])
      add(v, s);
  }
  const auto &quads = geom.const_quads();
  for (std::size_t i = 0; i < quads.size(); ++i) {
    const std::uint16_t s =
        geom.has_face_materials() ? geom.face_material(tris.size() + i) : surface;
    for (const cvc::index_t v : quads[i])
      add(v, s);
  }
  const float *fallback = surfaces[surface].albedo;
  for (std::size_t v = 0; v < colors.size(); ++v) {
    if (faces[v] == 0) {
      colors[v] = cvc::geometry::color_t{{fallback[0], fallback[1], fallback[2]}};
      continue;
    }
    for (int k = 0; k < 3; ++k)
      colors[v][k] /= faces[v];
  }
  return colors;
}

std::vector<std::shared_ptr<GraphicsNode>>
add_world_geometry(SceneGraph &sg, const cvc::world::world_geometry &g, const std::string &prefix,
                   const std::shared_ptr<GraphicsNode> &parent) {
  std::vector<std::shared_ptr<GraphicsNode>> nodes;
  for (const cvc::world::geometry_item &item : g.items()) {
    if (!item.has(cvc::world::role_render))
      continue;
    // A plain copy shares the item's arrays; only colours added below detach.
    cvc::geometry geom = *item.geometry;
    if (geom.const_colors().size() != geom.num_points())
      geom.colors() = surface_colors(geom, g.surfaces(), item.surface);
    std::shared_ptr<GraphicsNode> node =
        parent ? std::static_pointer_cast<GraphicsNode>(
                     parent->createChild<GeometryNode>(prefix + item.name, geom))
               : sg.addGraphics(prefix + item.name, geom);
    const bool placed =
        !item.height_field() && (item.position[0] != 0.0 || item.position[1] != 0.0 ||
                                 item.position[2] != 0.0 || item.rotation[0] != 1.0);
    if (node && placed) {
      double m[16];
      cvc::physics::pose_matrix(
          {{item.position[0], item.position[1], item.position[2]},
           {item.rotation[0], item.rotation[1], item.rotation[2], item.rotation[3]}},
          m);
      node->setPoseMatrix(m);
    }
    nodes.push_back(node);
  }
  for (const cvc::world::volume_item &v : g.volumes()) {
    if (!v.has(cvc::world::role_render))
      continue;
    // VolumeNode keeps a copy that shares the volume's voxels; the volume is placed
    // by its own bounding box, in world metres.
    const cvc::volume &vol = *v.field.volume;
    nodes.push_back(
        parent ? std::static_pointer_cast<GraphicsNode>(
                     parent->createChild<VolumeNode>(prefix + v.name, vol))
               : std::static_pointer_cast<GraphicsNode>(sg.addGraphics(prefix + v.name, vol)));
  }
  return nodes;
}

} // namespace gl
} // namespace cvc

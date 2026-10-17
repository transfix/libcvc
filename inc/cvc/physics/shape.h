/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics shapes. Primitives (sphere, box, convex hull) are the engine's own;
// meshes and terrain are the world's one representation, cvc::geometry and
// cvc::height_field (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md), shared with the
// renderer and the navigator rather than copied into physics structs. Every shape
// is in body-local coordinates except terrain, which is placed by its own grid.

#ifndef __CVC_PHYSICS_SHAPE_H__
#define __CVC_PHYSICS_SHAPE_H__

#include <cstdint>
#include <cvc/physics/types.h>
#include <memory>
#include <variant>
#include <vector>

namespace cvc {

class geometry;
class height_field;

namespace physics {

struct sphere {
  double radius = 0.5;
};

struct box {
  vec3 half_extents{0.5, 0.5, 0.5};
};

// The convex hull of a point cloud, in body-local coordinates.
struct convex_hull {
  std::vector<vec3> points;
};

// A static triangle mesh (buildings, props that never move): a cvc::geometry's
// triangles, and its quads split in two the way cvc::geometry::tri_surface splits
// them, in body-local coordinates.
//
// Materials: each face's surface material (cvc::geometry::face_materials) is
// looked up in `materials`, so materials[surface id] is this world's material id;
// with `materials` empty the surface id is used as the world's material id
// directly. A geometry without face materials takes the body's material.
struct mesh {
  std::shared_ptr<const cvc::geometry> geometry;
  std::vector<material_id> materials;
};

// Terrain: a cvc::height_field, the same object the renderer draws and the
// navigator samples. Static, and placed by its own grid: the body's position and
// rotation are ignored, so the terrain lines up with the raster it came from.
//
// The engine collides with the height field's own triangles (the same diagonal
// split cvc::height_field::height() describes), so a body rests where
// field->height(x, y) says. A hole (NaN) has no collision anywhere it touches.
//
// Materials: each cell's surface id (cvc::height_field::cell_materials) is looked
// up in `materials` as for a mesh; a field without cell materials takes the
// body's material. A terrain may use at most 256 distinct world materials.
//
// The engine stores heights compressed. `max_error` is the largest height error
// accepted; a backend that cannot meet it refuses the shape.
struct terrain {
  std::shared_ptr<const cvc::height_field> field;
  std::vector<material_id> materials;
  double max_error = 0.005;
};

using shape = std::variant<sphere, box, convex_hull, mesh, terrain>;

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_SHAPE_H__

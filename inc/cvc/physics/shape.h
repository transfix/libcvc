/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics shapes. Every shape is in body-local coordinates except the height
// field, whose placement is its own origin and cell size.

#ifndef __CVC_PHYSICS_SHAPE_H__
#define __CVC_PHYSICS_SHAPE_H__

#include <cmath>
#include <cstdint>
#include <cvc/physics/types.h>
#include <variant>
#include <vector>

namespace cvc {
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

// A triangle mesh, for static bodies only (buildings). Body-local coordinates.
struct triangle_mesh {
  std::vector<vec3> vertices;
  std::vector<std::uint32_t> indices;          // three per triangle
  std::vector<material_id> triangle_materials; // one per triangle; empty = the body's material
};

// Terrain over a regular grid: rows run along +y, columns along +x.
//
// Sample (row, col) is at origin + (col * dx, row * dy, heights[row * cols + col]).
// A NaN height is a hole: no collision anywhere it touches. Each cell (the quad
// between four samples) carries a material: cell_materials[row * (cols - 1) + col]
// indexes `materials`, a palette of at most 256 world materials. With no cell
// materials every cell is palette entry 0; with no palette, palette entry 0 is
// the body's material.
//
// The engine stores heights compressed. `max_error` is the largest height error
// accepted; a backend that cannot meet it refuses the shape. Jolt also splits each
// cell into two triangles, so its surface differs from bilinear_height() by the
// terrain's curvature within a cell (at most |h''| * cell^2 / 8).
//
// Height fields are static. The body's position and rotation are ignored: the
// field is placed by its origin, so it lines up with the raster it came from
// (cvc::world::raster_out, terrain.json).
struct heightfield {
  int rows = 0, cols = 0;
  double dx = 1.0, dy = 1.0;
  vec3 origin;
  std::vector<float> heights;
  std::vector<std::uint8_t> cell_materials;
  std::vector<material_id> materials;
  double max_error = 0.005;
};

using shape = std::variant<sphere, box, convex_hull, triangle_mesh, heightfield>;

// The height field's surface at world (x, y), interpolated bilinearly, the way
// cvc::nav samples its rasters. NaN outside the grid or next to a hole.
inline double bilinear_height(const heightfield &h, double x, double y) {
  if (h.rows < 2 || h.cols < 2 || h.heights.size() != static_cast<size_t>(h.rows) * h.cols)
    return std::nan("");
  const double u = (x - h.origin.x) / h.dx, v = (y - h.origin.y) / h.dy;
  if (!(u >= 0.0 && v >= 0.0 && u <= h.cols - 1 && v <= h.rows - 1))
    return std::nan("");
  int c = static_cast<int>(std::floor(u)), r = static_cast<int>(std::floor(v));
  if (c > h.cols - 2)
    c = h.cols - 2;
  if (r > h.rows - 2)
    r = h.rows - 2;
  const double fu = u - c, fv = v - r;
  const auto at = [&](int rr, int cc) { return static_cast<double>(h.heights[rr * h.cols + cc]); };
  const double z = (1 - fu) * (1 - fv) * at(r, c) + fu * (1 - fv) * at(r, c + 1) +
                   (1 - fu) * fv * at(r + 1, c) + fu * fv * at(r + 1, c + 1);
  return h.origin.z + z;
}

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_SHAPE_H__

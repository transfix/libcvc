/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// immersion.h — how deep a body sits in the water, column by column
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md B5 and C2): the immersed depth zeta(x, y) of a body
// over the cells of a grid window, from its shape, its pose and the water's surface. It is what a
// wake solver (cvc::ocean::ewave, the GPU's EWave) takes as a body's source,
// dh = -(zeta(t) - zeta(t - dt)).
//
//   cvc::physics::immersion_body crate(desc);            // its hull if it has one, else its shape
//   cvc::physics::immersion_water sea{surface.get(), t}; // or immersion_water::of(field, t)
//   cvc::physics::immersion_grid g;                      // the wake window's cells
//   g.nx = g.ny = 256; g.dx = 0.5; g.origin_x = x0; g.origin_y = y0;
//   std::vector<double> zeta(g.count(), 0.0);
//   add_immersion(crate, pose.position, pose.rotation, sea, g, zeta);
//
// What is underwater is what a fluid field immerses (field.h, hull.h): the same shape (a box, a
// sphere or the convex hull of a point set, or the body's hull_mesh where it has one, as the field
// step pushes on the hull in place of the cells), the same pose (body_state::position is the
// shape's origin), and the same water: the surface's height at (x, y) at the field time, capped by
// the field box's top, and only inside the box. The field step samples the body with its cells (or
// the hull's triangles); this samples it with vertical lines, exactly. So for a box at rest on
// still water the two agree exactly, and in general to the cells' resolution: the immersed volume
// sum(zeta) dx^2 is the field's field_load::immersed_volume (physics_immersion_test measures it).
// A client with no physics world (a renderer, arisrv from a replicated pose) gets the same zeta.
//
// A cell's zeta is the mean, over samples x samples sub-columns spread evenly across the cell
// (the cell dx wide, centred on its point origin + (i dx, j dx), as cvc::ocean::ewave places its
// cells), of the length of each sub-column inside the body and below the water's top over the
// cell's point. The water's top is read once per cell, the shape once per sub-column: zeta is the
// body's depth averaged over the cell's area, so it moves smoothly as an edge crosses the cell and
// its volume converges to the immersed volume as the cells shrink.
//
// Standard library only, like the rest of cvc/physics. Deterministic, and the same bits on every
// platform for the same inputs: IEEE arithmetic and sqrt, no libm transcendentals, built without
// FMA contraction (the surface's own determinism is its own: fluid_surface.h).

#ifndef __CVC_PHYSICS_IMMERSION_H__
#define __CVC_PHYSICS_IMMERSION_H__

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/physics/body.h>
#include <cvc/physics/field.h>
#include <cvc/physics/fluid_surface.h>
#include <cvc/physics/hull.h>
#include <cvc/physics/shape.h>
#include <cvc/physics/types.h>
#include <limits>
#include <vector>

namespace cvc {
namespace physics {

// The water a body is immersed in: the free surface's height at time t, under a ceiling, inside a
// box. With no surface the water's top is the ceiling (a field box's flat top).
struct immersion_water {
  const fluid_surface *surface = nullptr;
  double time = 0.0; // s: the world's field time (world::field_time)
  // The fluid field's box: water only for lo.x <= x <= hi.x and lo.y <= y <= hi.y, from lo.z up to
  // min(hi.z, the surface's height). The default is unbounded.
  vec3 lo{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()};
  vec3 hi{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()};

  // A fluid field's water at time t: its surface and its volume's bounding box, as the field step
  // reads them. Throws std::invalid_argument for a field with no volume. The surface is borrowed:
  // the field must outlive the result.
  static immersion_water of(const field &f, double t);
};

// A grid window: nx x ny cells of side dx, cell (i, j)'s point at origin + (i dx, j dx), stored
// row-major (row j), as cvc::ocean::ewave and the GPU's EWave lay out theirs.
struct immersion_grid {
  int nx = 0, ny = 0;
  double dx = 0.5;
  double origin_x = 0.0, origin_y = 0.0;
  int samples = 4; // sub-columns per side in each cell (1: the cell's point alone)
  std::size_t count() const noexcept { return std::size_t(nx) * std::size_t(ny); }
};

// A body's outline as the water sees it, prepared once.
class immersion_body {
public:
  // A box, a sphere or a convex hull (its faces are found here, once: the hull of the points).
  // Throws std::invalid_argument for a mesh or terrain (static: never immersed), a value that is
  // not finite, or a hull of fewer than 4 points or with no volume.
  explicit immersion_body(const shape &s);
  // A hull's triangles (hull.h): closed, wound counter-clockwise seen from outside. A vertical
  // line's inside is read from its crossings (it enters where a face looks down and leaves where
  // one looks up), so the mesh need not be convex. Throws std::invalid_argument for an index out
  // of range or a vertex that is not finite.
  explicit immersion_body(const hull_mesh &h);
  // What a fluid field pushes on for this body: its hull where it has one, else its shape.
  static immersion_body of(const body_desc &d);

  enum class kind : std::uint8_t { box, sphere, convex, mesh };
  kind outline() const noexcept { return _kind; }

  // The length (m) of the vertical line through world (x, y) inside the body at this pose and
  // below z = top (0 where it misses the body or lies above top). The rotation is a unit
  // quaternion, as body_state carries.
  double column(const vec3 &position, const quat &rotation, double x, double y, double top) const;
  // The world box around the body at this pose: lo and hi in x, y and z.
  void bounds(const vec3 &position, const quat &rotation, vec3 &lo, vec3 &hi) const;

  // The convex hull's faces, n . p <= offset in the body's frame (n a unit vector), (n, offset)
  // per face; empty for the other outlines.
  const std::vector<std::array<double, 4>> &planes() const noexcept { return _planes; }

  // A 64-bit FNV-1a digest of the outline.
  std::uint64_t digest() const;

private:
  friend double add_immersion(const immersion_body &, const vec3 &, const quat &,
                              const immersion_water &, const immersion_grid &,
                              std::vector<double> &);

  kind _kind = kind::box;
  vec3 _half;                                           // box
  double _radius = 0.0;                                 // sphere
  std::vector<std::array<double, 4>> _planes;           // convex: (n, offset)
  std::vector<vec3> _points;                            // convex: the points; mesh: the vertices
  std::vector<std::array<std::uint32_t, 3>> _triangles; // mesh
};

// Adds the body's zeta (m) over the grid into `zeta` (nx * ny values, row-major), so several
// bodies sum, and returns the volume it added, sum(zeta) dx^2 (m^3). Only the cells under the
// body's box are visited. Throws std::invalid_argument for a grid that is empty, a dx not above 0,
// samples outside 1..64, a zeta of the wrong size or a pose that is not finite.
double add_immersion(const immersion_body &body, const vec3 &position, const quat &rotation,
                     const immersion_water &water, const immersion_grid &grid,
                     std::vector<double> &zeta);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_IMMERSION_H__

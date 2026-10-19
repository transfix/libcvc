/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics fields: regions of the world that push on the bodies inside them,
// such as water a box floats in, snow a wheel sinks into, or wind a ball drifts
// with (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G6).
//
//   cvc::physics::field lake;
//   lake.kind = cvc::physics::field_kind::fluid;
//   lake.volume = water;               // a cvc::volume of kg/m^3: what VolumeNode draws
//   const auto id = w.add_field(lake);
//
// A field is cvc::volume data in world metres: the same volume a
// cvc::gl::VolumeNode draws and a cvc::world::world_geometry holds. It acts inside
// its volume's bounding box only, and its values are sampled trilinearly there
// (corner samples, as cvc::volume::interpolate reads them). The world reads the
// values when the field is added or set, so a volume changed afterwards takes
// effect at the next set_field.
//
// How a field pushes. A dynamic body is cut into sample cells when it is added:
// the interior of its sphere, box or convex hull, about 8 cells along its longest
// side. Meshes and terrain are static and never pushed. Before every step, for
// each field and each awake dynamic body, each cell's world box is clipped to the
// field's box; the immersed part, a fraction f of the cell's volume V_c, is sampled
// at its centre:
//
//   fluid   buoyancy  -rho g f V_c       rho the volume's density, g the world's gravity
//           drag      1/2 rho C_d a |u| u  u the fluid's velocity minus the cell's
//   medium  support   k f V_c, upward    k the volume's bearing modulus
//   flow    drag      as for a fluid     no buoyancy: air's is negligible
//   every   damping   c f V_c u          c the field's damping
//
// a is f V_c / V times the body's area across u: pi r^2 for a sphere, the
// projected face area for a box, the area of the sphere of equal volume for a
// hull. A body wholly inside a uniform flow therefore feels the textbook
// 1/2 rho C_d A |u| u. Each cell's force acts at its centre, so torques follow: a
// tilted slab rights itself, and a spinning ball is slowed. Drag and damping are
// integrated implicitly in their own strength (scaled by (1 - e^-x) / x, x the
// body's drag rate times fixed_dt), so a light body in a dense fluid never
// overshoots the fluid's velocity. "Upward" is against gravity (+z in a world
// without gravity).
//
// Damping is the linear resistance quadratic drag lacks at low speed. For a medium
// it is the main resistance to sinking in. For water it stands for the energy a
// floating body loses making waves: without it a body bobbing at a few cm/s feels
// almost no drag and bobs for a long time. A heave damped near critically needs c
// about 2 sqrt(rho g A m) / V_sub (A the waterplane area, m the mass, V_sub the
// volume under water); 1e4 N s/m^4 suits a body of a metre or so.
//
// A medium supports elastically. A flat footprint A sunk d metres into a medium of
// modulus k carries k A d (a Winkler foundation, Bekker's n = 1), and the medium
// pushes the body back out as it rises. A medium that keeps ruts is later work.
//
// Sleeping bodies are not pushed: the forces were in balance when the body fell
// asleep. add_field and set_field wake the bodies inside the field's box.
//
// Standard library only, like the rest of cvc/physics: cvc::volume is declared,
// not included.

#ifndef __CVC_PHYSICS_FIELD_H__
#define __CVC_PHYSICS_FIELD_H__

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/physics/types.h>
#include <memory>
#include <vector>

namespace cvc {

class volume;

namespace physics {

enum class field_kind : std::uint8_t {
  fluid,  // water, oil: buoyancy and drag
  medium, // snow, mud, sand: support as a body sinks in, and damping
  flow,   // moving air: drag only
};

struct field {
  field_kind kind = field_kind::fluid;
  // The field's region and strength, in world metres. The field acts inside this
  // volume's bounding box only. For a fluid or a flow the values are mass density
  // (kg/m^3). For a medium they are the bearing modulus k (N/m^3): the support per
  // cubic metre of a body sunk into it.
  std::shared_ptr<const cvc::volume> volume;
  // The stuff's velocity (m/s) along world x, y and z, one volume per component on
  // a grid of its own. Give all three or none; with none, the stuff is still.
  // Outside a component's bounding box that component is 0.
  std::array<std::shared_ptr<const cvc::volume>, 3> velocity;
  double drag_coefficient = 1.0; // fluid, flow: C_d
  double damping = 0.0;          // every kind: c, N s/m^4
  // medium: the friction a vehicle's tire finds in it (snow ~0.3, mud ~0.4), in place of the
  // ground's; 0 keeps the ground's. A vehicle's wheels also sink into a medium and ride on it
  // (vehicle.h).
  double traction = 0.0;
  // medium: whether it keeps the ruts wheels press into it (plastic snow, mud). A wheel then
  // leaves its sinkage behind it, a rut map at rut_cell (m) resolution over the field's box; a
  // wheel in a rut sits at least as deep, and the compaction resistance counts only snow
  // compacted further (b k (z^2 - rut^2) / 2), so a second pass in the tracks sinks no deeper and
  // rolls with less drag. The ruts are world state: saved, restored and hashed. set_field
  // starts the medium fresh.
  bool ruts = false;
  double rut_cell = 0.05;
};

// A medium's ruts (world::field_ruts): the depth (m) the surface has been pressed down, per cell
// of rut_cell over the field's box in x and y, row-major from (x0, y0), row 0 at min y.
struct rut_map {
  double x0 = 0.0, y0 = 0.0, cell = 0.0;
  int cols = 0, rows = 0;
  std::vector<float> depth;
  // The depth at (x, y), or 0 outside the map.
  double at(double x, double y) const {
    if (cols <= 0 || rows <= 0 || !(cell > 0.0))
      return 0.0;
    const double fc = (x - x0) / cell, fr = (y - y0) / cell;
    if (!(fc >= 0.0 && fr >= 0.0 && fc < cols && fr < rows))
      return 0.0;
    return depth[static_cast<std::size_t>(static_cast<int>(fr)) * cols + static_cast<int>(fc)];
  }
};

// A field handle. Never reused: a removed field's id stays invalid.
struct field_id {
  std::uint32_t value = 0xffffffffu;
  bool valid() const { return value != 0xffffffffu; }
  friend bool operator==(field_id a, field_id b) { return a.value == b.value; }
  friend bool operator!=(field_id a, field_id b) { return a.value != b.value; }
  friend bool operator<(field_id a, field_id b) { return a.value < b.value; }
};

// What the fields did to one body in the last step.
struct field_load {
  vec3 force;                   // N, the sum over the body's cells
  vec3 torque;                  // N m, about the body's centre of mass
  double immersed_volume = 0.0; // m^3, summed over the fields
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_H__

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// interp.h — interpret a derived word into geometry (Z-up, metres).
//
// One walker handles both the turtle alphabet (plants/rocks) and the scope
// alphabet (buildings): [ / ] push/pop the whole state, which is a scope
// (position + orthonormal frame + size vector), and a turtle state is a scope
// where the frame's heading is the growth direction. This is the roadmap §5.1
// "scope is a strict superset of a turtle state" made concrete.
//
// Angles in the WORD are already radians? No — the ruleset stores degrees; the
// interpreter converts here, ONCE, so the demo's radians-as-degrees bug is not
// inherited.
//
// Material semantics stay abstract: every primitive carries a generic `role`
// (trunk/foliage/rock/concrete/…). cvc::world maps role -> its surface_registry
// class; cvc::lsys never knows the concrete class ids or the DBG RF vocabulary.

#ifndef CVC_LSYS_INTERP_H
#define CVC_LSYS_INTERP_H

#include <cstdint>
#include <cvc/lsys/grammar.h>
#include <cvc/lsys/module.h>
#include <vector>

namespace cvc {
namespace lsys {

struct vec3 {
  double x = 0, y = 0, z = 0;
};

// Generic material roles. cvc::world maps these to concrete surface classes and
// (on the DBG side) to RF material names. NOT DBG-specific.
enum class role : std::uint16_t {
  trunk = 0,     // woody stem  -> wood
  branch,        // thin woody  -> wood
  foliage,       // leaves/canopy -> foliage
  rock,          // stone       -> rock
  ground,        // soil/dirt   -> soil
  water,         // -> water
  wall_concrete, // reinforced_concrete
  wall_brick,    // brick
  wall_drywall,  // drywall
  glass,         // glass
  metal,         // metal
  wood_solid,    // wood (planks, poles) -> wood
  unknown,
  _count
};

struct segment {         // tapered cylinder (a woody stem)
  vec3 a, b;             // endpoints (metres)
  double r0 = 0, r1 = 0; // radii at a, b
  role rl = role::trunk;
  std::uint8_t level = 0;
};

struct leaf { // a foliage marker
  vec3 pos;
  vec3 dir;
  double size = 0;
  role rl = role::foliage;
  std::uint8_t level = 0;
};

struct obox { // an oriented box (building part / rock chunk)
  vec3 center;
  vec3 axis[3]; // orthonormal columns
  vec3 half;    // half-extents along axis[0..2]
  role rl = role::wall_concrete;
  std::uint8_t level = 0;
};

struct paint2d { // an explicit ground-plane material stamp
  enum shape : std::uint8_t { disc, capsule } sh = disc;
  vec3 a, b; // disc: a=center; capsule: a..b centreline
  double radius = 0, feather = 0;
  role rl = role::ground;
};

struct structure {
  std::vector<segment> segments;
  std::vector<leaf> leaves;
  std::vector<obox> boxes;
  std::vector<paint2d> paints;
  vec3 lo, hi; // axis-aligned bounds of all geometry (metres)

  bool empty() const {
    return segments.empty() && leaves.empty() && boxes.empty() && paints.empty();
  }
};

struct interp_options {
  double leaf_size = 1.0; // default "L" size when it carries no param
  role default_leaf = role::foliage;
  role default_stem = role::trunk;
  role default_solid = role::wall_concrete;
  // The scope's initial size vector (metres) for building/rock roots.
  vec3 scope_size{1.0, 1.0, 1.0};
};

// Interpret a word into geometry. Deterministic and GL-free.
structure interpret(const ruleset &rs, const word &w, const interp_options &opt = interp_options{});

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_INTERP_H

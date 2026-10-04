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

// forest.h — the lsystem_forest demo's TREE generators, promoted into the library.
//
// Two species, ported VERBATIM in shape from src/cvcGL/examples/lsystem_forest.cpp so the
// trees look exactly as they do in the demo (this is the look the author wants kept — it is
// NOT the generic parametric pine_monopodial recipe):
//
//   * pine    — a conifer grown by a string-grammar turtle (TREE_RULES, commands F [ ] L R T
//               + a digit that recurses into a child rule). Its MODULE HIERARCHY is retained so
//               the wind cascade can re-pose only the swayer modules each frame (route C); that
//               is why the pine does NOT go through the generic derive()/interpret() path, whose
//               flat `structure` has no per-module frame to rotate.
//   * branchy — a compact recursive broadleaf splitter (a tapered trunk that forks into child
//               limbs, tips coloured as foliage).
//
// Output is real mesh: a `wood` cvc::geometry (triangles + per-vertex colour) shared by both
// species, and a `needle` cvc::geometry (LINES) for the pine foliage. This is the one place the
// otherwise mesh-free lsys library emits cvc::geometry directly — the demo's tessellation moved
// in, not re-derived.
//
// DETERMINISM (roadmap §5.2): unlike the predecessor's sequential std::mt19937 (whose draws
// reshuffle when you add a tree or reject one below the waterline), every random value here goes
// through the hashed RNG (rng.h) keyed on a STABLE element id (the placement attempt index, or a
// derivation path_id), so adding trees or moving the shoreline cannot move a tree that was
// already there. The tree SHAPE for a given set of parameters is deterministic.

#ifndef CVC_LSYS_FOREST_H
#define CVC_LSYS_FOREST_H

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include <cvc/lsys/interp.h> // cvc::lsys::vec3

namespace cvc {
class geometry; // output mesh type (core, GL-free); forward-declared to keep this header light

namespace lsys {

// Which species a forest scatters.
enum class species_mix {
  mix,     // alternate pine / branchy (the demo default)
  pine,    // all conifers
  branchy, // all broadleaf
};

// Parameters for a scattered forest. Geometry units are metres, Z-up, centred on the origin.
struct forest_params {
  std::uint64_t seed = 1337;
  int count = 60;            // number of trees to PLANT (attempts are bounded at count*6)
  double span = 100.0;       // scatter half-extent (a disc of this radius in world XY)
  double sea_level = 0.5;    // plant only where height_at(x,y) > sea_level + 0.5 (dry land)
  species_mix species = species_mix::mix;
  // branchy knobs
  double scale = 1.0;    // overall branchy scale (multiplies length + radius)
  double length = 6.0;   // branchy trunk length (before scale)
  double radius = 0.7;   // branchy trunk radius (before scale)
  int levels = 4;        // branchy recursion depth
  int branches = 3;      // branchy child limbs per node
  // pine knob
  double pine_scale = 1.35; // pine trunk-size multiplier
};

// Per-pine wind re-pose state (route C): the module-frame vertices + their offsets into the
// merged buffers, so repose_forest() can rewrite only each pine's vertices each frame. Opaque to
// callers except through repose_forest(); populated by grow_forest() when `wind` is requested.
struct forest_wind {
  struct mod_rec {
    int parent = -1;
    std::array<double, 16> hang{}; // the module's local frame (row-major 4x4)
    bool swayer = false;           // is this module in the sway cascade (low levels)?
    std::vector<vec3> local_wood;  // module-frame wood vertices
    int w_off = 0;                 // first wood vertex's offset in the merged buffer
    std::vector<vec3> local_needle;
    int n_off = 0;
  };
  struct tree {
    std::vector<mod_rec> mods;
    double phase = 0.0; // sway phase offset
    double sway = 0.0;  // sway amplitude
  };
  std::vector<tree> trees;
};

struct forest_result {
  int planted = 0; // trees actually placed
  int pines = 0;   // of which conifers
};

// A terrain height sampler: returns the ground height at world (x, y). Planting rejects any
// (x, y) whose height is at or below sea_level + 0.5. A null-equivalent (always-0) sampler plants
// on a flat plane at z = 0.
using height_fn = std::function<double(double x, double y)>;

// Grow a scattered forest into `wood` (triangles + per-vertex colour) and `needle` (LINES, pine
// foliage). When `wind` is non-null it is filled with each pine's re-pose records for
// repose_forest(). Appends to the meshes (does not clear them). Deterministic in `p.seed`.
forest_result grow_forest(const forest_params &p, const height_fn &height_at, cvc::geometry &wood,
                          cvc::geometry &needle, forest_wind *wind = nullptr);

// Re-pose every pine for wind time `t` (seconds) into the flat merged vertex buffers, scaling the
// global sway by `wind_scale`. The buffers must be the merged wood/needle meshes' current vertex
// arrays (3 doubles per vertex); grow_forest filled the bind pose, and this overwrites the swayer
// modules' vertices. The caller uploads them via GeometryNode::updateVertices. A no-op when the
// forest has no pines.
void repose_forest(const forest_wind &w, double t, double wind_scale, std::vector<double> &wood_buf,
                   std::vector<double> &needle_buf);

// ── single-tree generators (building blocks; also the seam for DSL intrinsics) ───────────────
// Grow ONE pine at base (bx,by,bz) with trunk scale `size` and grammar depth `maturity` (1..5),
// appending to the shared `wood`/`needle` meshes. When `out_tree` is non-null its wind re-pose
// records are filled. `phase`/`sway` seed the returned tree's sway (ignored when out_tree null).
void grow_pine(cvc::geometry &wood, cvc::geometry &needle, double bx, double by, double bz,
               double size, int maturity, forest_wind::tree *out_tree = nullptr, double phase = 0.0,
               double sway = 0.0);

// Grow ONE branchy broadleaf at base (bx,by,bz) growing up +Z, appending to `wood`. `seed`
// keys the hashed branch jitter (stable per sub-branch via a derivation path id).
void grow_branchy(cvc::geometry &wood, std::uint64_t seed, double bx, double by, double bz,
                  double length, double radius, int levels, int branches);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_FOREST_H

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

// scene_water.h — the water a `water` scene node puts into the scene's physics world, with or
// without a renderer, and the coast both it and the drawn water read
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §11.2, O15).
//
// A water node with `body:` is one fluid field (cvc::physics::field) over its water: the ground's
// extent, and `outer` metres past it when the ground's whole rim is under water (a shore-edged
// world, an island), from below the deepest ground to above the waves. Its surface is the sea the
// node draws (cvc::ocean::sea_surface read through cvc::physics::ocean_surface), so a body floats
// on the waves the renderer shows:
//   - the wind sea's three cascades (lsystem_coast's tiles and seed, h0 from build_h0), each fading
//     out as the still water shoals (over half its band's peak wavelength, at most `shore_fade`),
//     their sideways moves kept under 0.8 of the way to the shore;
//   - the swell's shore layer (cvc::ocean::shore_layer): refracting, shoaling, breaking on the
//     beaches that face it and reflecting off steep shores, when `swell_height` is above 0.
// The drawn node (cvcGL's water_node.cpp) and SceneWorld::from_scene (no renderer) both run a
// WaterRuntime; the drawn node draws the same coast (scene_water_shore) and cascade fades
// (scene_water_cascades).
//
// The coast is a shore grid (cvc::world::water_bodies: still-water depth, shore distance and
// normal, slope) over the ground's samples, at most 1025 a side (every few samples on a larger
// ground). When the rim is sea and there is a swell, the grid goes one deep-water swell wavelength
// past the rim (at most 512 m), the ground there the rim's, so the swell enters as a plane wave
// before it meets the coast. Past the grid both the GPU and the CPU read its edge.
//
// On the beach the swell runs up as the swash (shore_layer::swash): the drawn node draws it on the
// ground node's meshes, and the physics sea carries it, so a body on the sand is lifted by the film
// drawn there. `swash: 0` leaves it out of both.

#ifndef __CVC_ARIADNE_SCENE_WATER_H__
#define __CVC_ARIADNE_SCENE_WATER_H__

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
class height_field;
namespace ocean {
struct shore_grid;
class shore_layer;
class sea_surface;
} // namespace ocean
namespace physics {
struct field;
}
namespace ariadne {

struct SceneNode;
struct SceneColliders;

// What a water node states, its props seeding the live keys under its root (water.*, or its
// `keys:`): the level, the wind, the waves' chop and height, the swell, how they fade at the
// shore, and how far the sea goes past a rim under water.
struct SceneWaterSpec {
  double level = 0.0;
  double wind_speed = 9.0;    // m/s
  double wind_dir = 30.0;     // degrees: the way the waves travel, from +x toward +y
  double fetch = 120000.0;    // m
  double chop = 1.15;         // the cascades' sideways moves (OceanFFT::chop)
  double wave_amp = 1.0;      // a height scale (the cascades'; the swell keeps its true size)
  double shore_fade = 3.0;    // m: the most depth a cascade fades over toward the shore
  double swell_height = 0.0;  // m, deep-water significant height (0: no swell)
  double swell_period = 10.0; // s
  double swell_dir = 200.0;   // degrees: the way it travels, from +x toward +y
  double outer = 6000.0;      // m past a rim under water
  bool swash = true;          // the swash on the beach (the `swash` prop)
};

// The node's spec: its props, then its live keys (seeded from the props, as the drawn node does).
SceneWaterSpec scene_water_spec(cvc::app &app, const SceneNode &n, const std::string &bind_prefix);

// Whether the ground's rim is under `level` all round (98 % of its samples, so a stray rock at the
// edge does not cut the sea off): then the sea goes on past it.
bool water_rim_is_sea(const cvc::height_field &ground, double level);

// The coast the waves feel: the shore grid, the ground's slope on it (rise over run, for a wake's
// beaches), and the swell's shore layer (null without a swell).
struct SceneWaterShore {
  std::shared_ptr<const cvc::ocean::shore_grid> grid;
  std::vector<float> slope;
  std::shared_ptr<const cvc::ocean::shore_layer> layer;
  double build_ms = 0.0; // how long it took (the grid and the layer)
};

// The coast of `ground` for spec s (null when the ground is under 2 x 2 samples or its cells are
// not square). Tens to a few hundred milliseconds; the callers keep it until its inputs change.
std::shared_ptr<const SceneWaterShore> scene_water_shore(const cvc::height_field &ground,
                                                         const SceneWaterSpec &s);
// What scene_water_shore depends on besides the ground's heights.
std::vector<double> scene_water_shore_signature(const SceneWaterSpec &s);

// The wind sea's cascades, as both seas build them: their tiles (m, long to short), their fades
// toward the shore (m of still-water depth, half the band's peak wavelength, at most shore_fade)
// and the wavenumbers their velocity decays with below the level.
struct SceneWaterCascades {
  std::vector<double> tiles, fade_m, decay_k;
  std::uint64_t seed = 0;
};
SceneWaterCascades scene_water_cascades(const SceneWaterSpec &s);

// A water node's physics: its colliders (one fluid field), rebuilt when its keys (the wind, the
// chop, the wave height, the swell) change or its ground is regenerated. The scene's world swaps
// the field at its next sync, as it swaps a generated world's bodies.
class WaterRuntime {
public:
  // `ground` gives the ground the water fills, now and after a regenerate (null: none yet).
  WaterRuntime(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
               std::function<std::shared_ptr<const cvc::height_field>()> ground);
  ~WaterRuntime();
  WaterRuntime(const WaterRuntime &) = delete;
  WaterRuntime &operator=(const WaterRuntime &) = delete;

  const std::string &node() const;
  const std::shared_ptr<SceneColliders> &colliders() const;
  // Read the keys and the ground; rebuild the field when either changed. True when it was.
  bool tick();

  // As last built (null before the first build): the coast, the sea a body feels, and the field
  // (for a wake: cvc::physics::immersion_water::of(field, t)).
  std::shared_ptr<const SceneWaterShore> shore() const;
  std::shared_ptr<const cvc::ocean::sea_surface> sea() const;
  std::shared_ptr<const cvc::physics::field> field() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WATER_H__

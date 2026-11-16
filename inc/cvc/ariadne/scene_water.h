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
// without a renderer (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §11.2, O15).
//
// A water node with `body:` is one fluid field (cvc::physics::field) over its water: the ground's
// extent, and `outer` metres past it when the ground's whole rim is under water (a shore-edged
// world, an island), from below the deepest ground to above the waves. Its surface is the sea the
// node draws: the wind sea's three cascades (lsystem_coast's tiles and seed, h0 from
// cvc::ocean::build_h0) evolved on the CPU by cvc::ocean::sea_surface and read through
// cvc::physics::ocean_surface, so a body floats on the waves the renderer shows. The drawn node
// (cvcGL's water_node.cpp) and SceneWorld::from_scene (no renderer) both run a WaterRuntime.
//
// The physics sea is the open sea's: its waves do not yet fade toward the shore as the drawn ones
// do (that needs a shore grid), so a body in the last few metres of a beach rides waves the
// picture has calmed.

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
namespace ariadne {

struct SceneNode;
struct SceneColliders;

// What a water node states, its props seeding the live keys under its root (water.*, or its
// `keys:`): the level, the wind, the waves' chop and height, and how far the sea goes past a rim
// under water.
struct SceneWaterSpec {
  double level = 0.0;
  double wind_speed = 9.0; // m/s
  double wind_dir = 30.0;  // degrees: the way the waves travel, from +x toward +y
  double fetch = 120000.0; // m
  double chop = 1.15;      // the cascades' sideways moves (OceanFFT::chop)
  double wave_amp = 1.0;   // a height scale
  double outer = 6000.0;   // m past a rim under water
};

// The node's spec: its props, then its live keys (seeded from the props, as the drawn node does).
SceneWaterSpec scene_water_spec(cvc::app &app, const SceneNode &n, const std::string &bind_prefix);

// Whether the ground's rim is under `level` all round (98 % of its samples, so a stray rock at the
// edge does not cut the sea off): then the sea goes on past it.
bool water_rim_is_sea(const cvc::height_field &ground, double level);

// A water node's physics: its colliders (one fluid field), rebuilt when its keys (the wind, the
// chop, the wave height) change or its ground is regenerated. The scene's world swaps the field
// at its next sync, as it swaps a generated world's bodies.
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

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WATER_H__

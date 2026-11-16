/*
  Copyright 2026 The University of Texas at Austin

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

// scene_wind.h — the scene's wind, as a node reads it: one place for every node the wind moves
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E3, E5; O13 builds it for the water, O14 points it at
// one scene wind).
//
//   SceneWind w = scene_wind(app, node, prefix, kWaterWindKeys, {9.0, 30.0, 0.25});
//   double g = cvc::algo::wind_gust(w.state(), x, y, t);   // the gusts at a point
//
// A wind is its mean speed, the way it blows and its gustiness; the gusts riding on it are
// cvc::algo::wind_gust, one deterministic function of (x, y, t) that the water (its ripples and
// roughness), the clouds' drift, the trees and the air physics can all sample alike. Today the
// water node is the only reader, and its wind is its own keys (water.wind_speed, water.wind_dir,
// water.gustiness: kWaterWindKeys). O14's one scene wind (`wind.speed`, `wind.dir`,
// `wind.gustiness`) changes this function, not its callers: every node that reads its wind here
// then follows the scene's.
//
// Directions: `dir` is the way the wind blows in degrees from +x toward +y (the water's
// convention). cloud_sky's `wind_az` is a compass azimuth (0 toward +y, 90 toward +x); azimuth()
// converts.

#ifndef __CVC_ARIADNE_SCENE_WIND_H__
#define __CVC_ARIADNE_SCENE_WIND_H__

#include <cvc/algo/wind_field.h>
#include <string>

namespace cvc {
class app;
namespace ariadne {

struct SceneNode;

struct SceneWind {
  double speed = 9.0;      // m/s, the mean wind (U10)
  double dir = 30.0;       // degrees: the way it blows, from +x toward +y
  double gustiness = 0.25; // 0: steady (ABYSSAL's default 0.25)

  cvc::algo::wind_state state() const;
  double azimuth() const { return 90.0 - dir; } // compass degrees, the way it blows
};

// The state keys (under a node's root, honouring its `keys:`) and props a node's wind is read from.
struct SceneWindKeys {
  const char *speed, *dir, *gustiness;                // keys
  const char *speed_prop, *dir_prop, *gustiness_prop; // props that seed them
};
// The water node's: water.wind_speed, water.wind_dir and water.gustiness, seeded from its
// wind_speed, wind_dir and gustiness props.
inline constexpr SceneWindKeys kWaterWindKeys{
    "water.wind_speed", "water.wind_dir", "water.gustiness", "wind_speed", "wind_dir", "gustiness"};

// The wind node n reads: its keys, seeded from its props (or `defaults`), read now.
SceneWind scene_wind(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                     const SceneWindKeys &keys, const SceneWind &defaults = {});

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WIND_H__

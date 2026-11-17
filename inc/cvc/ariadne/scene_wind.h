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

// scene_wind.h — the scene's one wind: what every node the wind moves reads
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E3, E5; O13, O14).
//
//   scene:
//     wind: { speed: 9, dir: 30, gustiness: 0.25 }   # seeds wind.speed, wind.dir, wind.gustiness
//
//   SceneWind w = scene_wind(app, node, prefix, kWaterWindKeys, {9.0, 30.0, 0.25});
//   double g = cvc::algo::wind_gust(w.state(), x, y, t);   // the gusts at a point
//
// A wind is its mean speed, the way it blows and its gustiness; the gusts riding on it are
// cvc::algo::wind_gust, one deterministic function of (x, y, t) that the water (its ripples and
// roughness), the clouds' drift, the trees' sway and the physics' air (cvc::physics::wind_flow)
// all sample alike.
//
// The scene wind. A document's `scene: wind:` seeds three live keys under the scene's state
// root: wind.speed (m/s), wind.dir (degrees) and wind.gustiness (seed_scene_wind, which the
// renderer and the headless SceneWorld both run). While wind.speed is set, the scene has a wind,
// and every reader follows it (scene_wind_set): change wind.dir and the waves, the cloud deck's
// drift, the trees' sway and the push on every body in the air turn together.
//
// A node's own wind. Without a scene wind each node keeps its own keys: the water its
// water.wind_speed / water.wind_dir / water.gustiness (kWaterWindKeys), cloud_sky its
// sky.wind_az, a forest its symmetric sway. With one, those keys are left alone and unused: they
// are what the node falls back to if the scene's wind keys are cleared (the water's panel's wind
// sliders then drive it again).
//
// Directions: `dir` is the way the wind blows in degrees from +x toward +y (the water's
// convention). cloud_sky's heading `wind_az` is the one whose drift is (sin az, -cos az): az =
// dir + 90 (azimuth()).

#ifndef __CVC_ARIADNE_SCENE_WIND_H__
#define __CVC_ARIADNE_SCENE_WIND_H__

#include <cvc/algo/wind_field.h>
#include <memory>
#include <optional>
#include <string>

namespace cvc {
class app;
namespace physics {
class wind_flow;
}
namespace ariadne {

struct SceneNode;
struct Scene;

struct SceneWind {
  double speed = 9.0;      // m/s, the mean wind (U10)
  double dir = 30.0;       // degrees: the way it blows, from +x toward +y
  double gustiness = 0.25; // 0: steady (ABYSSAL's default 0.25)

  cvc::algo::wind_state state() const;
  // cloud_sky's heading for this wind (its drift is (sin az, -cos az)): dir + 90 degrees.
  double azimuth() const { return dir + 90.0; }
  bool operator==(const SceneWind &) const = default;
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

// Seed the scene wind's keys (wind.speed, wind.dir, wind.gustiness under bind_prefix) from the
// document's `scene: wind:`, when it has one; keys already set keep their values.
void seed_scene_wind(cvc::app &app, const Scene &scene, const std::string &bind_prefix);

// The scene's wind, read now, when it has one (wind.speed is set under bind_prefix); a missing
// dir or gustiness reads 0 and the default 0.25.
std::optional<SceneWind> scene_wind_set(cvc::app &app, const std::string &bind_prefix);

// The wind node n reads: the scene's when it has one; else its own keys, seeded from its props
// (or `defaults`), read now.
SceneWind scene_wind(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                     const SceneWindKeys &keys, const SceneWind &defaults = {});

// The physics' air for wind w over base level base_z (m): cvc::physics::wind_flow.
std::shared_ptr<const cvc::physics::wind_flow> scene_wind_flow(const SceneWind &w,
                                                               double base_z = 0.0);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WIND_H__

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
//
// With `wake_push: 1` (the water.wake_push key; off by default) the field has a wake
// (cvc::physics::field_wake, OCEAN-AND-WEATHER-ROADMAP.md C5 and D6): the waves its floating
// bodies make are stepped inside the physics step, on a window of `wake_push_cells` cells of
// `wake_push_cell` metres (64 of 1 m) where it holds the most floating bodies (a boat counting as
// four), and push the other bodies, so boats feel each other's wakes and the debris' rings. It is
// physics: the same on every host and at every frame rate, in the world's state and its hash. The
// drawn wake (the node's eWave window) is separate and unchanged: it draws every body's waves
// around the camera's target.

//
// The wind on the water (O13; OCEAN-AND-WEATHER-ROADMAP.md E2-E4, E6), the same in both seas:
//   - The wind is read through scene_wind.h (water.wind_speed, water.wind_dir, water.gustiness),
//     the one place O14's scene wind will drive.
//   - The sea lags it (E4): SceneWaterSeaClock follows the wind with `sea_lag` seconds' time
//     constant and steps along it by keyframes, each crossfading from the last over `sea_fade`
//     seconds (cvc::ocean::sea_lag). A keyframe builds the cascades' h0 for its wind and, when its
//     direction moved, the fetch map. The physics sea mixes the two h0 by the crossfade's weight
//     at the clock's time (sea_surface's forcing); the drawn node mixes the same two on the CPU
//     and uploads them, but only while a crossfade runs.
//   - Fetch and lee (E2, `lee`): a coarse fetch map over the shore grid (at most 256 samples a
//     side), each sample's fetch capped by its body's open extent (a lake's or a pond's) or the
//     open `fetch` for the sea. Each cascade is scaled by sqrt(F / F_full), clamped, F_full the
//     fetch its band is fully grown at.
//   - Gusts (E3, `gustiness`): the ripple cascade's weight follows the scene's gust field, and the
//     drawn water's roughness too (cat's paws).
//   - Currents (E6, `current` m/s RMS): a curl-noise flow carries the two short cascades and the
//     water's velocity, so a floating body drifts as the small waves drawn do. water_bodies has no
//     stream velocities yet (hydrology, W1), so the current is the curl noise alone.

#ifndef __CVC_ARIADNE_SCENE_WATER_H__
#define __CVC_ARIADNE_SCENE_WATER_H__

#include <cstdint>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_lag.h>
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
struct fetch_field;
struct sea_forcing;
struct sea_cascade;
} // namespace ocean
namespace physics {
struct field;
}
namespace ariadne {

struct SceneNode;
struct SceneColliders;

// What a water node states, its props seeding the live keys under its root (water.*, or its
// `keys:`): the level, the wind, the waves' chop and height, the swell, how they fade at the
// shore, and how far the sea goes past a rim under water; and the wind's effects (O13).
struct SceneWaterSpec {
  double level = 0.0;
  double wind_speed = 9.0;    // m/s (the wind, read through scene_wind.h)
  double wind_dir = 30.0;     // degrees: the way the waves travel, from +x toward +y
  double gustiness = 0.25;    // the gusts' strength (E3)
  double fetch = 120000.0;    // m: the open sea's fetch
  double chop = 1.15;         // the cascades' sideways moves (OceanFFT::chop)
  double wave_amp = 1.0;      // a height scale (the cascades'; the swell keeps its true size)
  double shore_fade = 3.0;    // m: the most depth a cascade fades over toward the shore
  double swell_height = 0.0;  // m, deep-water significant height (0: no swell)
  double swell_period = 10.0; // s
  double swell_dir = 200.0;   // degrees: the way it travels, from +x toward +y
  double outer = 6000.0;      // m past a rim under water
  bool swash = true;          // the swash on the beach (the `swash` prop)
  bool lee = true;            // E2: fetch and lee (0: every cascade at full height)
  double sea_lag = 20.0;      // E4: s, the sea's wind follows the wind with this time constant
  double sea_fade = 4.0;      // E4: s, a new keyframe's sea crossfades in over this
  double current = 0.0;       // E6: m/s, the current's RMS speed (0: none)
  // C5 (D6): the waves the floating bodies make push the others (cvc::physics::field_wake): the
  // `wake_push` key (0 / 1, off by default) and the window's props, `wake_push_cells` (a power of
  // two, 8 .. 128), `wake_push_cell` (m) and `wake_push_depth` (m; 0: deep water).
  bool wake_push = false;
  int wake_push_cells = 64;
  double wake_push_cell = 1.0;
  double wake_push_depth = 0.0;
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
  // Per sample of the grid: the most fetch its body allows (a lake's or a pond's open extent,
  // water_body::fetch_m), or -1 (the sea's: the open fetch).
  std::vector<float> fetch_cap;
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

// The sea as the wind has grown it (O13): the last two keyframes of the lagged wind, their
// cascades' h0 and fetch maps, and the crossfade between them. Both seas read one.
struct SceneWaterSea {
  cvc::ocean::sea_wind from, to;             // the keyframes' winds
  double t0 = 0.0, t1 = 0.0;                 // the crossfade, clock seconds
  std::vector<cvc::ocean::h0_field> h0;      // per cascade: the new keyframe's
  std::vector<cvc::ocean::h0_field> h0_from; // and the old's (empty: no crossfade yet)
  std::shared_ptr<const cvc::ocean::fetch_field> fetch, fetch_from; // null: no lee
  std::vector<double> full, full_from; // per cascade: the fetch it is grown at (E2)
  std::uint64_t keyframe = 0;          // counts keyframes (and rebuilt fetch maps)
  double build_ms = 0.0;               // the last keyframe's h0 and fetch maps

  // The new keyframe's share at clock time t (0 -> 1 over the crossfade).
  double weight(double t) const;
  bool fading(double t) const { return t >= t0 && t < t1; }
};

// Steps a water node's sea along its lagged wind (cvc::ocean::sea_lag). advance() with the spec
// and coast now and the clock's time: true when the sea changed (a keyframe, or a new coast or lee
// switch that rebuilt the fetch maps).
class SceneWaterSeaClock {
public:
  SceneWaterSeaClock();
  ~SceneWaterSeaClock();
  bool advance(double t, const SceneWaterSpec &s,
               const std::shared_ptr<const SceneWaterShore> &shore);
  std::shared_ptr<const SceneWaterSea> sea() const; // null before the first advance

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The sea_surface forcing for spec s and sea k: the crossfade, the fetch maps, the scene's wind
// (the gusts) and the current.
std::shared_ptr<const cvc::ocean::sea_forcing> scene_water_forcing(const SceneWaterSpec &s,
                                                                   const SceneWaterSea &k);
// The cascades a sea_surface takes for spec s and sea k: h0 (and h0_from), the fades and decays,
// each band's full fetch, the ripples' gust weight, the short cascades carried by the current.
std::vector<cvc::ocean::sea_cascade> scene_water_sea_cascades(const SceneWaterSpec &s,
                                                              const SceneWaterSea &k);
// The cascades' gust weights (1 for the ripples) and whether each is carried by the current: what
// both seas use.
constexpr double kWaterGustWeight[3] = {0.0, 0.0, 1.0};
constexpr bool kWaterAdvect[3] = {false, true, true};

// A water node's physics: its colliders (one fluid field), rebuilt when its keys (the chop, the
// wave height, the swell, the gusts, the current) change, the sea takes a new keyframe of its
// lagged wind, or its ground is regenerated. The scene's world swaps the field at its next sync,
// as it swaps a generated world's bodies.
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
  // The still-water level (m) the field was last built for (before the first build, its spec's).
  double level() const;
  // The sea's keyframes (O13), which the drawn node draws too.
  std::shared_ptr<const SceneWaterSea> keyframes() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WATER_H__

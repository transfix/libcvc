/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// scene_boat.h — a `boat` scene node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D6): a boat in the
// scene's physics world, afloat on its water, driven from its keys (a helm) or by the navigator to
// a goal, with or without a renderer.
//
//   - node: launch
//     type: boat
//     water: water          # the water node it floats on (default: the scene's first)
//     preset: launch        # launch | jonboat (cvc::physics::dyn_boat_desc)
//     mass: 900             # optional, over the preset's
//     max_speed: 3.19       # optional
//     transform: { position: [10, 165, 0], rotation: [0, 0, 45] }   # x, y and the heading
//     drive: keys           # keys: its throttle and rudder | nav: the navigator, to its goal
//     goal: [40, 190]       # nav's goal (world metres)
//     keys: launch          # its keys' root (default: its id)
//
// It is a cvc::physics::dyn_boat_plant with one boat in the scene's world, which the world steps
// (step_world off), and a cvc::nav::sim_world of its own over the water's ground (its grid with the
// water's depth: land and the water too shallow for it blocked). The boat goes in when the water's
// grid is ready and has water under its position: at its draft, upright, heading where its rotation
// points. Its z is the water's, not the transform's.
//
// Keys, under its root (seeded from the props):
//   drive               keys | nav
//   throttle, rudder    the helm in [-1, 1] (+ rudder turns left), under drive: keys
//   goal                "x,y", under drive: nav
// and telemetry: x, y, z, heading (degrees from +x), speed (m/s through the water), thrust (N),
// reached (1 within reach of nav's goal), placed (1 in the world), status.
//
// A scene world (SceneWorld::bind_boats) runs one per node; a renderer draws the node's hull at the
// boat's pose (cvcGL's boat node) and keeps the water out of it.

#ifndef __CVC_ARIADNE_SCENE_BOAT_H__
#define __CVC_ARIADNE_SCENE_BOAT_H__

#include <array>
#include <cvc/physics/dyn_boat_plant.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {
struct world_grid;
}
namespace ariadne {

struct SceneNode;

// What a boat node states (its props and world matrix).
struct SceneBoatSpec {
  std::string node;  // its id
  std::string keys;  // its keys' root, resolved under the bind prefix
  std::string water; // the water node it floats on ("": the scene's first)
  std::string preset = "launch";
  physics::dyn_boat_desc desc;
  double x = 0.0, y = 0.0, heading = 0.0; // where it starts (heading in rad)
  std::string drive = "keys";             // keys | nav
  bool has_goal = false;
  double goal_x = 0.0, goal_y = 0.0;
};
// The spec of node `n` at world matrix `world` (row-major), its keys resolved under
// `bind_prefix`. An unknown preset warns and takes the launch.
SceneBoatSpec scene_boat_spec(const SceneNode &n, const std::array<double, 16> &world,
                              const std::string &bind_prefix,
                              std::vector<std::string> *warnings = nullptr);

class BoatRuntime {
public:
  // `app` may be null (no keys).
  BoatRuntime(cvc::app *app, SceneBoatSpec spec);
  ~BoatRuntime();
  BoatRuntime(const BoatRuntime &) = delete;
  BoatRuntime &operator=(const BoatRuntime &) = delete;

  const SceneBoatSpec &spec() const;
  // Put the boat in `world` on `grid` (its water's ground, with the water layer): true when it is
  // in (or already was). False and why when it cannot be yet: no water there, too shallow, a grid
  // without water. It does not try again with the same world and grid until either changes.
  bool place(physics::world &world, std::shared_ptr<const nav::world_grid> grid,
             std::string *why = nullptr);
  bool placed() const;
  // The world was made afresh (SceneWorld::restart): the boat went with the old one. The next
  // place() puts it in again.
  void world_gone();

  // Its keys: the drive, the helm or the goal, from the next step.
  void read_keys();
  // Around each world step: the navigator's tick every k steps, then the plant's step.
  void before_step();
  void after_step();
  void publish();

  physics::boat_id boat() const; // invalid until placed
  physics::body_id hull() const;
  physics::boat_state state() const; // throws cvc::physics_error until placed
  // How deep it draws (its keel or its propeller), and its hull's half length, half beam, and
  // its sheer's and bottom's heights over its origin (m): what a renderer keeps water out of.
  double draft() const;
  // Its hull's body description (its collision shape and hull_mesh), the same object for its life:
  // what a renderer immerses in a wake window.
  std::shared_ptr<const physics::body_desc> hull_desc() const;
  double half_length() const;
  double half_beam() const;
  double sheer() const;
  double bottom() const;
  const std::string &status() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_BOAT_H__

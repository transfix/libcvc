/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::plant_host: the plant a sim_world drives on, chosen by name. Either nav's
// kinematic bicycle, or physics vehicles in a world the host builds and owns. An application
// keeps one per sim_world and picks the plant in code or in the state tree:
//
//   cvc::physics::plant_host host;           // a flat ground, 1 m per nav world unit
//   host.use(sim, "humvee");                 // sim's agents drive Humvees on Jolt
//   host.use(sim, "kinematic");              // back to nav's bicycle
//   host.pull(app, "demo", sim);             // or live: <prefix>.nav_plant.vehicle
//
// The name is "kinematic" (or empty), a vehicle_desc preset ("humvee", "roboracer"), or a
// comma-separated list of presets: agent i drives the i-th, cycled. Without a Jolt backend in
// the build, a preset falls back to the kinematic bicycle, and fallback() says so.
//
// "dynamic" puts the agents on the presets' dynamic single-track model instead (dyn_bike_plant:
// tires, load transfer, actuator lag and its port-Hamiltonian energy books), and "dynamic4w" on
// the four-wheel model with suspension (dyn_4w_plant). "dynamic" or "dynamic4w" alone is the
// Humvee's; "dynamic:<presets>" and "dynamic4w:<presets>" name them ("dynamic:roboracer",
// "dynamic4w:humvee,roboracer"). They drive on options::plant.ground_height (else the flat
// ground_z) with options::ground_mu's grip: the single track takes the slope along its heading,
// the four wheels each the ground under it. With Jolt in the build their chassis are bodies in a
// physics world built as below, so they collide with the geometry, with each other and with
// anything added to physics(); without Jolt they drive the same ground and collide with nothing.
//
// The world is rebuilt on every switch. It holds a flat ground at options::ground_z, or, after
// set_geometry, a cvc::world's static collide items (add_world_geometry). The navigator's
// occupancy grid is not added: nav keeps its agents off what it sees, and the vehicles collide
// with each other and with the geometry given. The vehicles plan with their own limits and
// size (nav_plant_options::plan_with_vehicle).
//
// Lifetime: the host owns the world and the nav_plant. It must outlive sim's use of them, or
// switch sim back to "kinematic" first. Call use() and pull() from the thread that steps sim.

#ifndef __CVC_PHYSICS_PLANT_HOST_H__
#define __CVC_PHYSICS_PLANT_HOST_H__

#include <cvc/physics/dyn_4w_plant.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {
class sim_world;
}
namespace world {
class world_geometry;
}
namespace physics {

struct plant_host_options {
  world_config world;      // the physics world (Jolt, cross-platform determinism, 0.01 s)
  nav_plant_options plant; // latency, margins, the plan; metres_per_world_unit is nav's frame
  // The flat ground, used when no geometry is set: its top's height (m), its grip and rolling
  // resistance, and its half extent (m).
  double ground_z = 0.0;
  double ground_mu = 0.9;
  double ground_rolling_resistance = 0.015;
  double ground_half_size = 2000.0;
};

class plant_host {
public:
  explicit plant_host(plant_host_options options = {});
  ~plant_host();
  plant_host(const plant_host &) = delete;
  plant_host &operator=(const plant_host &) = delete;

  // Whether this build has the Jolt backend; the presets use() accepts (each also as
  // "dynamic:<preset>").
  static bool available();
  static std::vector<std::string> presets();
  // Whether a name asks for the dynamic single-track plant ("dynamic" or "dynamic:..."), or the
  // four-wheel one ("dynamic4w" or "dynamic4w:...").
  static bool is_dynamic(const std::string &vehicle);
  static bool is_dynamic4w(const std::string &vehicle);
  // A preset by name (as in use()); unknown names throw cvc::physics_error.
  static vehicle_desc preset(const std::string &name);
  // Throws cvc::physics_error unless use() accepts `vehicle`: for checking a name up front.
  static void check(const std::string &vehicle);

  // The next world's static colliders: a cvc::world's collide items, in place of the flat
  // ground, with nav material ids per surface id (add_world_geometry's). Borrowed until the
  // next call; null goes back to the flat ground. Takes effect at the next use().
  void set_geometry(const cvc::world::world_geometry *g, std::vector<int> nav_materials = {});

  // Put sim's agents on `vehicle`, at their poses, at rest, in a new world. An unknown name
  // throws and changes nothing. Returns what they drive now: `vehicle`, or "kinematic" when it
  // asked for vehicles and this build has no Jolt.
  std::string use(cvc::nav::sim_world &sim, const std::string &vehicle);

  const std::string &vehicle() const { return _asked; }   // what use() was last asked for
  const std::string &driving() const { return _driving; } // what the agents drive
  bool fallback() const { return _asked != _driving; }
  nav_plant *plant() { return _plant.get(); } // null on the bicycle and on the dynamic plants
  // The physics world: null on the bicycle, and on the dynamic plants in a build without Jolt.
  world *physics() { return _world.get(); }
  dyn_bike_plant *dynamic() { return _dyn.get(); }  // null unless the agents drive "dynamic"
  dyn_4w_plant *dynamic4w() { return _dyn4.get(); } // null unless they drive "dynamic4w"
  // Whichever plant the agents drive on (null on the bicycle).
  cvc::nav::vehicle_plant *active();
  const plant_host_options &options() const { return _o; }

  // The state tree:
  //   <prefix>.nav_plant.vehicle     knob: what to drive; a change switches the plant
  //   <prefix>.nav_plant.driving     telemetry: what the agents drive (after any fallback)
  //   <prefix>.nav_plant.available   build: 1 when this build has Jolt
  // and, on vehicles, the plant's own keys (nav_plant::pull). An unset vehicle is seeded with
  // the current one; an unknown name is ignored and the plant kept.
  void pull(cvc::app &ctx, const std::string &prefix, cvc::nav::sim_world &sim);

private:
  // A new world holding the geometry (or the flat ground), with `cfg`; `ground` gets the bodies
  // that are the ground (the terrain, or the flat ground's box).
  std::unique_ptr<world> build_world(const world_config &cfg, std::vector<body_id> &ground) const;
  // Detach sim and drop every plant and the world.
  void drop(cvc::nav::sim_world &sim);

  plant_host_options _o;
  const cvc::world::world_geometry *_geometry = nullptr;
  std::vector<int> _nav_materials;
  std::unique_ptr<world> _world;
  std::unique_ptr<nav_plant> _plant;
  std::unique_ptr<dyn_bike_plant> _dyn;
  std::unique_ptr<dyn_4w_plant> _dyn4;
  std::string _asked = "kinematic", _driving = "kinematic";
  std::set<std::string> _described;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_PLANT_HOST_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::world_binding — a physics world configured from, and reported to,
// an app's state tree.
//
// Everything configurable about a world lives in the tree under one prefix
// (default "physics"), so a scene file, a UI, a Python script or a remote peer
// sets it and every one of them sees the same values. The world itself stays
// free of the tree (like world_clock and world_clock_binding): the binding reads
// the tree into the world on pull() and writes the world's telemetry on push(),
// Nothing here is global: a binding belongs to one app and one world.
//
// Thread-safe: every call locks the binding (and the world locks itself), so a
// UI thread may pull() a change while the simulation thread calls step(); the
// state tree is the cross-thread channel either way.
//
//   auto cfg = cvc::physics::world_binding::config_from_state(app);   // seeds defaults
//   cvc::physics::world w(cfg);
//   cvc::physics::world_binding bind(app, w);
//   ... per frame: bind.follow(app.world_clock());                    // pull, step, push
//
// Keys, relative to the prefix. A key that is unset is seeded with the world's
// value (the knob idiom: whatever set it first, a scene or a script, wins). An
// unreadable value is ignored, never thrown into a frame.
//
//   built with the world (config_from_state reads them; changing one later sets
//   restart_required instead of touching the running world):
//     backend                  string  null | jolt
//     fixed_dt                 double  seconds per step
//     threads                  int     worker threads in a step; 0 = none
//     determinism              string  same_binary | cross_platform
//     max_bodies, max_body_pairs, max_contact_constraints   int
//     origin.x, origin.y, origin.z                          double, metres
//
//   applied by every pull():
//     gravity.x, gravity.y, gravity.z      double  m/s^2
//     collision_steps, velocity_steps, position_steps     int
//     penetration_slop                     double  metres
//     friction_combine, restitution_combine
//                                          string  product | min | max | geometric_mean | average
//     contacts                             string  none | added_removed | all
//     materials.<name>.friction            double  one child per material; a new child adds
//     materials.<name>.restitution         double  a material, a changed value updates it from
//     materials.<name>.rolling_resistance  double  the next step. "default" is material 0.
//     materials.<name>.nav_material        int     Names must not contain '.'.
//     publish_every                        int     step()/follow() push every N steps (1)
//     hash_every                           int     push() publishes state_hash every N
//                                                  steps; 0 = never (it digests the whole
//                                                  state, so it is off by default)
//
//   written by push():
//     tick, time, step_ms, bodies, active_bodies, contact_events, saturated
//     state_hash               string  16 hex digits (see hash_every)
//     version                  string  the backend and its build options
//     restart_required         bool    a build-time key differs from the running world

#ifndef __CVC_PHYSICS_WORLD_BINDING_H__
#define __CVC_PHYSICS_WORLD_BINDING_H__

#include <cstdint>
#include <cvc/physics/world.h>
#include <map>
#include <mutex>
#include <string>

namespace cvc {

class app;
class world_clock;

namespace physics {

class world_binding {
public:
  // A world_config read from the tree under `prefix`, seeding every unset key
  // with the default. Materials are not part of it: the binding adds them.
  static world_config config_from_state(cvc::app &ctx, const std::string &prefix = "physics");

  world_binding(cvc::app &ctx, physics::world &w, std::string prefix = "physics");

  // Tree -> world: the runtime settings, then the material table.
  void pull();
  // World -> tree: the telemetry keys.
  void push();
  // pull(), one step, push() (every publish_every steps).
  step_report step();
  // pull(), world().follow(clock), push() if any step ran and one is due.
  std::uint64_t follow(const cvc::world_clock &clock);

  // The id of a material declared under <prefix>.materials (as of the last pull).
  // Throws cvc::physics_error for a name the tree does not declare.
  material_id material(const std::string &name) const;

  // The absolute state path of a key, e.g. key("gravity.z") = "physics.gravity.z".
  std::string key(const std::string &name) const;

  physics::world &world() { return _w; }
  const std::string &prefix() const { return _prefix; }

private:
  struct material_slot {
    material_id id = 0;
    physics::material applied;
  };

  void seed();
  void pull_locked();
  void push_locked();
  void pull_materials();

  mutable std::mutex _mutex;
  cvc::app &_ctx;
  physics::world &_w;
  std::string _prefix;
  bool _seeded = false;
  int _publish_every = 1;
  int _hash_every = 0;
  bool _restart_required = false;
  step_report _last;
  std::map<std::string, material_slot> _materials;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_WORLD_BINDING_H__

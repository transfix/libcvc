/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The world the dynamic plants drive in (dyn_bike_plant, dyn_4w_plant): the ground under their
// wheels, and the physics world their chassis collide in. Internal to libcvc.
//
// The ground is nav_plant_options' (ground_height, else ground_z): heights in world metres, the
// same function an application spawns Jolt vehicles on. The dynamic models own their wheels and
// the ground under them; the physics world owns everything else a vehicle can hit.
//
// Collisions (dyn_chassis). Each vehicle is a dynamic box in the world: its chassis box
// (vehicle_desc::half_extents) at the model's ride height and attitude. Every substep, before the
// world's one step, put() places each box at the pose its vehicle starts the substep in, moving
// with the twist its model reached. The world's step carries the boxes along and resolves their
// contacts. A box that touched a body that is not the ground (a building, a wall, another
// vehicle, a prop) is touched(), and moved() reads where the world left it: the plant writes that
// pose and twist back into its model and books the energy it changed as the contact's. A box that
// touched nothing leaves its model's step alone, bit for bit.

#ifndef __CVC_PHYSICS_DYN_WORLD_H__
#define __CVC_PHYSICS_DYN_WORLD_H__

#include <cstdint>
#include <cvc/physics/dyn_driver.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <set>
#include <vector>

namespace cvc {
namespace physics {
namespace dyn {

// The ground's height (m) at a world point: options.ground_height, else options.ground_z.
double ground_at(const nav_plant_options &o, double x, double y);
// The sine of the ground's slope along heading psi at (x, y), + uphill: central differences over
// `span` metres.
double grade_at(const nav_plant_options &o, double x, double y, double psi, double span);

// Rz(psi) Ry(th) Rx(phi): the heading, then the pitch (+ lowers the nose), then the roll (+ lifts
// the left side), as the dynamic models measure them.
quat attitude(double psi, double th, double phi);
vec3 rotate(const quat &q, const vec3 &v);
double yaw_of(const quat &q);
double wrap(double a); // into (-pi, pi]

// The Jolt plant's driver (vehicle.cpp, drive()) for a dynamic model: a step's drive force per unit
// mass, for nav's acceleration `want` along the heading at speed vx, on a slope whose sine along
// the heading is `grade` (+ uphill), for a vehicle d whose drive lags drive_lag (s). Substep h (s).
double drive(dyn_driver &driver, const vehicle_desc &d, double want, double vx, double grade,
             double gravity, double drive_lag, double h);
// After the substep (and any contact): the speed it reached, which is what the driver measures.
void measured(dyn_driver &driver, double vx0, double vx1, double h);

// The chassis boxes of one plant's vehicles in a physics world.
class dyn_chassis {
public:
  dyn_chassis() = default;
  ~dyn_chassis();
  dyn_chassis(const dyn_chassis &) = delete;
  dyn_chassis &operator=(const dyn_chassis &) = delete;

  // Drop the boxes, then work in `w` (null: no collisions), where `ground` are the bodies the
  // wheels drive on: touching them is the tires', not a collision, and the boxes' surface has this
  // friction (combined with what they touch by the world's rule). Sets w's contact reporting to
  // every step's (contact_reporting::all), since a chassis pressed against a wall stays in contact.
  void reset(world *w, std::vector<body_id> ground, double friction);
  bool on() const { return _w != nullptr; }
  world *physics() const { return _w; }

  // A box for the next vehicle, `d`'s chassis box at this pose. Returns its index.
  int add(const vehicle_desc &d, const vec3 &position, const quat &rotation);
  // Before the world's step: box i at this pose (the substep's start), moving with v (m/s, at the
  // box's centre) and spin w (rad/s), world axes.
  void put(int i, const vec3 &position, const quat &rotation, const vec3 &v, const vec3 &w);
  // The world's step, then which boxes touched something that is not the ground.
  void step();
  bool touched(int i) const { return _touched.at(static_cast<std::size_t>(i)) != 0; }
  // Of that, for the plant ledger: the other box that box i touched deepest in the step (its
  // index; -1 none) and how deep (m), and the contacts it began with static geometry (a body
  // without mass that is not the ground: a building, a wall).
  int touched_box(int i) const { return _other.at(static_cast<std::size_t>(i)); }
  double touched_box_depth(int i) const { return _other_depth.at(static_cast<std::size_t>(i)); }
  int static_contacts(int i) const { return _statics.at(static_cast<std::size_t>(i)); }
  // Box i where the world's step left it.
  body_state moved(int i) const;
  body_id box(int i) const { return _boxes.at(static_cast<std::size_t>(i)); }
  int size() const { return static_cast<int>(_boxes.size()); }

private:
  void clear();
  world *_w = nullptr;
  material_id _material = 0;
  std::vector<body_id> _boxes;
  std::set<std::uint32_t> _ground;
  std::vector<std::uint8_t> _touched;
  std::vector<int> _other, _statics;
  std::vector<double> _other_depth;
};

// Box i's contacts in c's last step, added to a tick's: the contacts it began with static
// geometry, and the other box it touched deepest (index and depth; -1 none).
void count_contacts(const dyn_chassis &c, int i, int &statics, int &touched, double &depth);

// A body at `rotation` as the plant ledger takes its attitude (plant_sample's grade, roll and
// up_z: its forward, left and up axes' world z), as nav_plant measures it.
void attitude_of(const quat &rotation, cvc::nav::plant_sample &s);

} // namespace dyn
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_WORLD_H__

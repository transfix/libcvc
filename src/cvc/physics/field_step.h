/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The field step: the forces cvc::physics fields put on bodies, which world.cpp
// applies before each engine step (field.h has the model). Private: not
// installed. It runs in the facade, in double precision and in ascending field and
// body id order, so a run stays a function of its inputs; the backend sees one
// force and one torque per pushed body.

#ifndef __CVC_PHYSICS_FIELD_STEP_H__
#define __CVC_PHYSICS_FIELD_STEP_H__

#include "backend_impl.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// A copy of a cvc::volume's values, sampled trilinearly in world metres.
struct field_grid {
  std::vector<double> values; // x fastest, as cvc::voxels stores them
  std::uint64_t n[3] = {0, 0, 0};
  double lo[3] = {0.0, 0.0, 0.0}, hi[3] = {0.0, 0.0, 0.0};
  double span[3] = {0.0, 0.0, 0.0}; // 0 along an axis with one sample

  bool contains(const double p[3]) const;
  // The value at p, which the caller has checked lies in the box.
  double at(const double p[3]) const;
};

struct compiled_field {
  field desc; // as given: it holds the volumes
  field_grid scalar;
  bool moving = false;
  field_grid velocity[3];
  rut_map ruts;              // a medium with desc.ruts: what wheels have pressed into it
  std::size_t rut_cells = 0; // cells with a rut
  double rut_deepest = 0.0;  // m
};

// A body's hull (hull.h) as the fluids see it, built when the body is added.
struct field_hull {
  std::shared_ptr<const hull_mesh> desc; // null: the body has no hull
  std::uint64_t digest = 0;              // of the description, for save() and state_hash()
  std::vector<vec3> normal;              // per triangle, body-local unit outward normals
  std::vector<double> wetted;            // per triangle, the last step's submerged area (m^2)
  double length = 0.0;                   // m, for the Reynolds number
  double slam_length = 0.0;              // m, the cube root of the hull's volume
  double inertia = 0.0;                  // m^2 per kg, a low estimate of the least moment
};

// A dynamic body as the fields see it: its sample cells and what its drag area
// needs, built when the body is added.
struct field_body {
  enum class outline : std::uint8_t { sphere, box, other };
  std::vector<vec3> cells; // centres, body-local about the body's origin
  vec3 cell;               // a cell's size along the body's axes
  vec3 lo, hi;             // the cells' body-local bounds
  double cell_volume = 0.0, volume = 0.0, mass = 0.0;
  vec3 center_of_mass; // body-local
  outline shape = outline::other;
  double radius = 0.0;  // sphere
  vec3 half;            // box
  bool vehicle = false; // a vehicle's chassis: no flow drag (vehicle.h's aero is its drag)
  field_hull hull;      // a fluid pushes on its triangles in place of the cells
  field_load load;      // the last step's
};

// The water at a point (field_set::water_at), for a boat's propeller and rudder.
struct water_sample {
  bool wet = false;     // under the fluid's top and above its bottom, at a density > 0
  double depth = 0.0;   // m below the top: the free surface's height, or the box's top
  double density = 0.0; // kg/m^3
  vec3 velocity;        // m/s, the water's: the surface's, or the velocity volumes'
};

// Worker threads for the fields' free-surface reads (world_config::field_threads). run(n, fn)
// hands the indices 0 .. n - 1 out to the workers and the calling thread from a shared counter
// and returns once every one has run; fn must not depend on which thread runs an index. Without
// workers (or in a build without threads) it runs them in turn on the calling thread. The first
// exception an index throws is rethrown from run().
class field_workers {
public:
  explicit field_workers(int threads);
  ~field_workers();
  field_workers(const field_workers &) = delete;
  field_workers &operator=(const field_workers &) = delete;
  int threads() const { return _wanted; }
  void run(std::size_t n, const std::function<void(std::size_t)> &fn);

private:
  void loop();
  void work();

  int _wanted = 0;
  std::vector<std::thread> _threads;
  std::mutex _m;
  std::condition_variable _go, _done;
  const std::function<void(std::size_t)> *_fn = nullptr;
  std::size_t _n = 0;
  std::atomic<std::size_t> _next{0};
  std::uint64_t _generation = 0;
  int _busy = 0;
  bool _stop = false;
  std::exception_ptr _error;
};

class field_set {
public:
  // Throw cvc::physics_error for a field world::add_field refuses.
  field_id add(world_backend &be, const field &f);
  // Keeps the ruts when the new field lays them on the same grid.
  void set(world_backend &be, field_id id, const field &f);
  field get(field_id id) const;
  void remove(field_id id);
  std::size_t size() const { return _fields.size(); }

  // A body world::add_body has checked and the backend has added. Only dynamic
  // spheres, boxes and convex hulls are recorded.
  void body_added(const world_backend &be, body_id id, const body_desc &d);
  void body_removed(body_id id) { _bodies.erase(id.value); }
  // A new mass for a dynamic body (world::set_body).
  void set_mass(body_id id, double mass) {
    const auto it = _bodies.find(id.value);
    if (it != _bodies.end() && mass > 0.0)
      it->second.mass = mass;
  }

  // A vehicle's chassis (body_added first): its centre of mass, and no flow drag.
  void mark_vehicle(body_id id, const vec3 &center_of_mass);

  // Push every awake body the fields reach, for the step about to run.
  // `t` is the world's field time, at which free surfaces are read.
  void apply(world_backend &be, const world_config &cfg, double t);
  field_load load(body_id id) const;

  // A dynamic body's centre of mass, body-local (the origin for one the fields do not record).
  vec3 center_of_mass(body_id id) const {
    const auto it = _bodies.find(id.value);
    return it == _bodies.end() ? vec3{} : it->second.center_of_mass;
  }
  // The water at `p` at field time `t`: the first fluid field, in id order, whose box holds `p`
  // in x and y and reaches below it. False if none does; `out` is then dry and still.
  bool water_at(const vec3 &p, double t, water_sample &out) const;

  // The air at `p` for a vehicle's aero: a flow field's density and wind where one covers
  // `p` (the first, in id order), true; else false, `rho` and `wind` untouched.
  bool air_at(const vec3 &p, double &rho, vec3 &wind) const;
  // The medium over a wheel's ground contact `p`: the deepest medium field above it along
  // +z, its bearing modulus at mid-depth and its traction. False if none lies over `p`.
  // Read-only: safe from the engine's worker threads during a step.
  bool medium_under(const vec3 &p, wheel_medium &out) const;
  // The height of the tallest medium field (0 with none).
  double medium_reach() const;

  // Ruts (field::ruts). Press a wheel's footprint into the medium `field`: the
  // rectangle `length` along the unit direction (dx, dy) by `width` across, centred on
  // (x, y), keeping at each cell the deeper of the rut and `depth`. With `behind`, only
  // the half behind the centre (a rolling wheel leaves its rut behind it).
  void press(std::uint32_t field, double x, double y, double dx, double dy, double length,
             double width, double depth, bool behind);
  // The rut at (x, y) in `field`; 0 without ruts there.
  double rut_at(std::uint32_t field, double x, double y) const;
  rut_map ruts(field_id id) const;
  bool has_ruts() const;
  // Every field, for world::snapshot.
  void snapshot(std::vector<field_snapshot> &out) const;
  // The rut maps' depths, for world::save / restore (the same fields, in id order).
  void save_ruts(std::string &out) const;
  void restore_ruts(const std::string &in, std::size_t &pos);
  // Free surfaces (field::surface). Whether any fluid has one; each surface field's id and
  // digest, in id order, for world::save; and the check restore makes that this world's
  // surfaces are the state's (throws cvc::physics_error otherwise).
  bool has_surfaces() const;
  void save_surfaces(std::string &out) const;
  void check_surfaces(const std::string &in, std::size_t &pos) const;
  // Hulls (body_desc::hull). Whether any body has one; each hull body's id, the hull's digest
  // and its triangles' last wetted areas, in id order, for world::save; and restoring them
  // (throws cvc::physics_error if this world's hulls differ from the state's).
  bool has_hulls() const;
  void save_hulls(std::string &out) const;
  void restore_hulls(const std::string &in, std::size_t &pos);

private:
  void wake_in(world_backend &be, const field_grid &box);

  std::map<std::uint32_t, compiled_field> _fields;
  std::map<std::uint32_t, field_body> _bodies;
  std::uint32_t _next = 0;
  std::unique_ptr<field_workers> _workers; // world_config::field_threads, made on first use
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_STEP_H__

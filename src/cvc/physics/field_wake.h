/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// field_wake.h — a fluid's wake (field::wake, field.h; OCEAN-AND-WEATHER-ROADMAP.md C5, D6): the
// waves its floating bodies make, stepped inside the physics step, and what each body reads of the
// others' waves. Private: not installed. field_step.cpp runs one per fluid field with a wake.
//
// One window of n x n cells of dx on the world's grid of whole cells, cell (i, j) at
// ((ci + i) dx, (cj + j) dx), and one wave state in it per body that makes waves (its "window"
// here): h, phi and dh/dt per cell, stepped by cvc::ocean::ewave's exact linear step (no walls)
// one after another through one ewave (its coefficients shared). A body's source is its immersed
// columns' change, -(zeta(t) - zeta(t - T)) smoothed [1 2 1] / 4 along x and y (it keeps the
// volume), added to its own state. Body b reads the sum of the states less its own.
//
// Deterministic: the bodies are taken in id order, the states summed in id order, ewave and
// add_immersion give the same bits on every platform, and the module is built without FMA
// contraction (the physics exact-math list). The reads are const and may run on several threads.

#ifndef __CVC_PHYSICS_FIELD_WAKE_H__
#define __CVC_PHYSICS_FIELD_WAKE_H__

#include <cstdint>
#include <cvc/ocean/ewave.h>
#include <cvc/physics/field.h>
#include <cvc/physics/immersion.h>
#include <cvc/physics/types.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// A body the wake may hear, as the field step hands it over (in id order).
struct wake_body {
  std::uint32_t id = 0;
  const immersion_body *outline = nullptr; // null: it makes no waves (its outline would not cut)
  vec3 position;
  quat rotation;
  // Its world box.
  double lo[3] = {0.0, 0.0, 0.0}, hi[3] = {0.0, 0.0, 0.0};
  bool hull = false;     // a boat: the window follows these first
  bool active = false;   // awake
  bool in_water = false; // its box reaches below the water's top over its position
};

// The others' waves at a point, for one body.
struct wake_sample {
  double h = 0.0;            // m
  double u = 0.0, v = 0.0;   // m/s: grad phi, the surface's horizontal velocity
  double w = 0.0;            // m/s: dh/dt
  double gx = 0.0, gy = 0.0; // grad h
};

class fluid_wake {
public:
  // Throws cvc::physics_error for parameters out of range.
  fluid_wake(const field_wake &p, double gravity);

  const field_wake &params() const { return _p; }
  // The world's gravity (m/s^2) the waves run under: a new value remakes the engine, the waves
  // kept.
  void set_gravity(double gravity);

  // Before the reads of every physics step (fixed_dt s): every whole number of steps nearest the
  // period, steps the states by that time, moves the window to the bodies, adds each body's
  // source and works out what the bodies read. `water` is the field's water now (its surface at
  // the field time, its box). Appends to `wake_up` the sleeping bodies the others' waves reach.
  void advance(const std::vector<wake_body> &bodies, const immersion_water &water, double fixed_dt,
               std::vector<std::uint32_t> &wake_up);

  // The state of body `id` (its own waves), or -1 when it has none.
  int own(std::uint32_t id) const;
  // The waves at (x, y) less state `own`'s (-1: all of them); zero outside the window.
  void sample(int own, double x, double y, wake_sample &out) const;
  bool placed() const { return _placed; }

  wake_info info() const;

  // World state (world::save / restore): the window's place, its step count and the states.
  void save(std::string &out) const;
  // Throws cvc::physics_error for a state that does not fit this wake.
  void restore(const std::string &in, std::size_t &pos);

private:
  struct state {
    std::uint32_t body = 0;
    bool source = true;            // its body still makes waves here
    std::vector<double> h, phi, w; // n^2 each
    std::vector<double> zeta;      // its body's columns at the last wave step
    std::vector<double> f;         // what it gives a reader: 6 per cell (h, u, v, w, gx, gy)
  };
  void derive(state &s) const;
  void sum();
  void shift(int di, int dj);

  field_wake _p;
  int _n = 0;
  double _dx = 1.0;
  double _gravity = 0.0;
  std::unique_ptr<ocean::ewave> _engine;
  bool _placed = false;
  std::int64_t _ci = 0, _cj = 0; // cell (0, 0) is world cell (ci, cj)
  std::uint64_t _count = 0;      // physics steps seen
  std::map<std::uint32_t, state> _states;
  std::vector<std::uint32_t> _ids;     // the states' bodies in id order (index = own())
  std::vector<const state *> _byIndex; // and the states themselves
  std::vector<double> _total;          // 6 per cell: the sum of every state's f
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_WAKE_H__

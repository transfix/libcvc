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
// ((ci + i) dx, (cj + j) dx), and one wave state in it per body that makes waves: h, phi and dh/dt
// per cell, stepped by cvc::ocean::ewave's exact linear step (no walls) one after another through
// one ewave (its coefficients shared). With a shore (field_wake::shore) the window is masked from
// it at every place it takes, and the ewave runs its no-flux step (walls and beaches; one or more
// whole substeps a wave step, so it carries no remainder from one state to the next). A body's
// source is its immersed columns' change, -(zeta(t) - zeta(t - T)) smoothed [1 2 1] / 4 along x
// and y (it keeps the volume) and times the mask, added to its own state; the columns are cut
// against what it floats in, the sea and the others' waves. Body b reads the sum of the states
// less its own.
//
// Depth: each state also keeps its motion (grad phi, dh/dt and grad h) at `layers` depths below
// the top, cell / 2 doubling (layer_depth), each wavenumber decayed on its own as linear theory
// has it (cosh and sinh over the window's depth, e^{-k d} in deep water), from its fields extended
// past the walls by the ewave's ghosts (their mirror images, so the walls stay no-flux). A read at
// depth d interpolates between the two layers round d, and past the deepest decays as the window's
// longest wave would. The layers reach as deep as the bodies in the window do (1.5 times their
// deepest point below the top, at most kMaxLayers); how many is world state.
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
#include <functional>
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
  bool hull = false;     // a boat: weighs more where the window goes
  bool active = false;   // awake
  bool over = false;     // its box overlaps the field's
  bool in_water = false; // and reaches below the water's top over its position (advance sets it)
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
  // At most this many states whose bodies make no more waves are kept fading beside the sources.
  static constexpr int kMaxFading = 4;
  // At most this many depths below the top each state keeps its motion at.
  static constexpr int kMaxLayers = 5;

  // Throws cvc::physics_error for parameters out of range.
  fluid_wake(const field_wake &p, double gravity);

  const field_wake &params() const { return _p; }
  // The world's gravity (m/s^2) the waves run under: a new value remakes the engine, the waves
  // kept.
  void set_gravity(double gravity);

  // Every physics step (fixed_dt s), first: counts it, and says whether the waves step at it (every
  // whole number of steps nearest the period, at least one), and by how long (`dt`).
  bool begin_step(double fixed_dt, double &dt);
  // At a step where they do, before the reads: steps the states by `dt`, moves the window to the
  // bodies, adds each awake body's source and works out what the bodies read. The field's box is
  // lo .. hi; `sea` gives its surface held at the field time t (asked for only when the step has
  // work to do: none when every body sleeps and no wave is left). Sets each body's in_water, and
  // appends to `wake_up` the sleeping bodies the others' waves reach.
  void advance(std::vector<wake_body> &bodies, const vec3 &lo, const vec3 &hi,
               const std::function<const fluid_surface *()> &sea, double t, double dt,
               std::vector<std::uint32_t> &wake_up);

  // The state of body `id` (its own waves), or -1 when it has none.
  int own(std::uint32_t id) const;
  // The waves at (x, y) less state `own`'s (-1: all of them), at the top; zero outside the window.
  void sample(int own, double x, double y, wake_sample &out) const;
  // Their height alone (sample's h).
  double height(int own, double x, double y) const;
  // Their motion (sample's u, v, w, gx and gy; h is left 0) at `depth` metres below the top (at
  // most 0: the top's, sample's own).
  void motion(int own, double x, double y, double depth, wake_sample &out) const;
  bool placed() const { return _placed; }
  // Layer l's depth below the top (l in 1 .. kMaxLayers): cell / 2 times 2^(l - 1), at most the
  // window's water depth.
  double layer_depth(int l) const;
  // The depths kept now.
  int layers() const { return _layers; }

  wake_info info() const;

  // World state (world::save / restore): the window's place, its step count, the bodies outside it,
  // the depths kept and the states.
  void save(std::string &out) const;
  // A saved wake read whole before anything changes (parse throws cvc::physics_error for a state
  // that does not fit this wake), then taken (commit, which cannot fail): a failed read leaves the
  // wake as it was. `version` is the section's (1: from before shores and depths, none kept).
  struct saved;
  std::shared_ptr<saved> parse(const std::string &in, std::size_t &pos, int version = 2) const;
  void commit(saved &s);

private:
  struct state {
    std::uint32_t body = 0;
    // Its body makes waves here: it holds one of the `sources` slots. A body that sleeps, leaves
    // the window or leaves the world gives its slot up (its waves fade on), and takes one again,
    // if one is free, when it is awake in the window again.
    bool source = true;
    std::vector<double> h, phi, w; // n^2 each
    std::vector<double> zeta;      // its body's columns at the last wave step
    std::vector<double> f;         // what it gives a reader: 6 per cell (h, u, v, w, gx, gy)
    std::vector<double> deep;      // and below the top: per layer, 5 per cell (u, v, w, gx, gy)
  };
  void derive(state &s) const;
  // s.deep for the depths kept (_layers), from its fields extended past the walls.
  void deepen(state &s);
  void sum();
  void shift(int di, int dj);
  // The window's cell for a world coordinate (false when it is not finite or absurdly far).
  bool cell_of(double v, std::int64_t &out) const;
  // The engine for the gravity, the wave step and the window's mask now.
  void remake();
  // The mask at the window's place now, given to the engine (with a shore).
  void remask();
  // Layer l's per-mode decay, sideways (grad phi, grad h) and up (dh/dt), with the inverse FFT's
  // 1 / n^2 folded in: made when first needed.
  const std::vector<double> &decay(int l, bool up);

  field_wake _p;
  int _n = 0;
  double _dx = 1.0;
  double _gravity = 0.0;
  double _T = 0.0; // the wave step the engine's substep divides (with a shore; 0 until known)
  std::unique_ptr<ocean::ewave> _engine;
  ocean::fft_plan _fft;
  bool _placed = false;
  std::int64_t _ci = 0, _cj = 0; // cell (0, 0) is world cell (ci, cj)
  std::uint64_t _count = 0;      // physics steps seen
  std::map<std::uint32_t, state> _states;
  std::vector<std::uint32_t> _ids;     // the states' bodies in id order (index = own())
  std::vector<const state *> _byIndex; // and the states themselves
  std::vector<double> _total;          // 6 per cell: the sum of every state's f
  std::vector<double> _deepTotal;      // per layer, 5 per cell: the sum of every state's deep
  int _outside = 0; // bodies in the water but not in the window, at the last wave step
  int _layers = 0;  // the depths below the top each state keeps its motion at
  std::vector<double> _mask; // n^2 at the window's place (with a shore and once placed): 1 water
  std::vector<double> _decay[2][kMaxLayers]; // decay(l, up), per mode
  std::vector<double> _work[2];              // complex work fields for deepen
};

struct fluid_wake::saved {
  bool placed = false;
  std::int64_t ci = 0, cj = 0;
  std::uint64_t count = 0;
  int outside = 0;
  int layers = 0;
  std::map<std::uint32_t, state> states;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_WAKE_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_OCEAN_WAKESOURCE_H__
#define __CVC_GL_OCEAN_WAKESOURCE_H__

#include <cvc/ocean/ewave.h>
#include <cvc/physics/immersion.h>
#include <vector>

// ---------------------------------------------------------------------------
// WakeSource — bodies as an eWave window's sources (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md
// C2, O9): each step, each body's columns of immersed depth zeta over the window's cells
// (cvc::physics::add_immersion, the code physics immerses with), and the height source
// dh = -(zeta(t) - zeta(t - dt)), splatted with a smooth kernel, for EWave::addHeights (or
// cvc::ocean::ewave::add_heights).
// ---------------------------------------------------------------------------
//   WakeSource src(window.params());
//   src.begin();                                   // each step
//   for (body : bodies) src.add(outline, pose.position, pose.rotation, water);
//   if (src.finish(dh)) window.addHeights(dh);
//   window.step(dt);
//
// The source conserves volume: what a body pushes under the surface rises around it, so a
// dropped crate makes a ring, and a moving hull pushes water up at its bow and lets it down at
// its stern (with exact dispersion the Kelvin pattern follows).
//
// The kernel is [1 2 1] / 4 along x and then y over the cells: it keeps the volume, takes the
// footprint's sharpest edges (shorter than two cells, which the window cannot carry) out of the
// source, and spreads a small body over a few cells.
//
// When the window moves (EWave::shift), shift() moves the last zeta with it. The cells that
// enter take their first zeta as their last, so a body already under the strip that enters does
// not make a source as it appears. The first finish() makes none either: it only records zeta.
//
// Depends on cvc::ocean (for ewave_params) and cvc::physics (immersion.h), both standard library;
// float-free and GPU-free, so it is unit-testable anywhere.
class WakeSource {
public:
  // The window's cells (n, dx, origin) and the sub-columns per cell side (immersion_grid).
  explicit WakeSource(const cvc::ocean::ewave_params &window, int samples = 4);

  const cvc::physics::immersion_grid &grid() const { return m_grid; }

  // The window moved by (di, dj) cells (EWave::shift's arguments).
  void shift(int di, int dj);

  // Forgets the last zeta, so the next finish() only records (after a pause, when the bodies may
  // have moved while no source was made).
  void reset() { m_haveLast = false; }

  // Starts a step: zeta is cleared.
  void begin();
  // Adds one body's zeta at this pose in this water; returns its immersed volume in the window
  // (m^3).
  double add(const cvc::physics::immersion_body &body, const cvc::physics::vec3 &position,
             const cvc::physics::quat &rotation, const cvc::physics::immersion_water &water);
  // Ends the step: dh (n x n, row-major) = -(zeta - the last zeta), smoothed, and zeta becomes the
  // last. False, with dh untouched, when the source is zero everywhere (nothing to add).
  bool finish(std::vector<double> &dh);

  // This step's zeta (m), n x n, row-major.
  const std::vector<double> &zeta() const { return m_zeta; }
  // The volume the bodies added have under the water in the window this step (m^3).
  double volume() const { return m_volume; }

private:
  cvc::physics::immersion_grid m_grid;
  std::vector<double> m_zeta, m_last, m_work, m_row;
  bool m_haveLast = false;
  double m_volume = 0.0;
};

#endif // __CVC_GL_OCEAN_WAKESOURCE_H__

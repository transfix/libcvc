/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav's plant seam (docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md, P4).
//
// A sim_world integrates its agents with the fused kinematic bicycle unless a plant is set
// (sim_world::set_plant). With a plant, the drive computes each agent's command, the bicycle
// angle delta_cmd and the acceleration a_cmd, and the plant integrates: nav reads the agent's
// pose and speed back from it every tick. nav knows only this interface; the physics plant
// (cvc::physics::nav_plant) lives in cvc::physics and converts between nav's normalized frame
// and SI there, through world_units with nav::frame_of(cfg).
//
// Everything here is in nav's normalized frame: normalized = (world - centre) * S, with S the
// sim_world's config.scale. Angles are radians, times seconds.

#ifndef __CVC_NAV_VEHICLE_PLANT_H__
#define __CVC_NAV_VEHICLE_PLANT_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/plant_types.h>

namespace cvc {
namespace nav {

// The plant interface. A sim_world calls attach() from set_plant(), then every tick: state()
// for each agent (the tick's start), command() for each, advance() once, and state() again.
class vehicle_plant {
public:
  virtual ~vehicle_plant() = default;
  virtual plant_kind kind() const = 0;

  // Bind to a sim_world's n agents, starting at `start` ([n], normalized), in its frame, at
  // its tick of dt seconds. Called by sim_world::set_plant.
  virtual void attach(int n, const plant_state *start, const cvc::world_units::normalized_frame &f,
                      double dt) = 0;
  virtual plant_state state(int i) const = 0;
  virtual plant_limits limits(int i) const = 0;
  // Agent i's command for the next tick: the bicycle angle (rad, + left) and the acceleration
  // along the heading (normalized / s^2; < 0 slows a forward-moving agent).
  virtual void command(int i, float steer, float accel) = 0;
  // Run the plant through one tick.
  virtual void advance() = 0;

  // The substeps of the kinematic reference the drift is measured against: the plant's own
  // steps a tick, doc 31 D14's nsub = k.
  virtual int reference_substeps() const { return 1; }
  // The tick's drift, which sim_world measures and hands over to bin and publish.
  virtual void report_drift(int i, const drift_sample &s) { (void)i, (void)s; }
  // For nav_stats: the energy agent i drew last tick (J), and the contacts its body started with
  // static geometry last tick. A plant that does not know reports 0.
  virtual double energy_step(int i) const { return (void)i, 0.0; }
  virtual int static_contacts_step(int i) const { return (void)i, 0; }
  // Whether energy_step() measures anything (nav_stats then carries energy_J and
  // energy_per_km).
  virtual bool tracks_energy() const { return false; }
  // For nav_stats' plant ledger: agent i as of the end of the last tick (plant_sample, SI). A
  // plant that does not measure it reports the default sample and tracks_samples() false.
  virtual plant_sample sample(int i) const { return (void)i, plant_sample{}; }
  virtual bool tracks_samples() const { return false; }
  // How the plant's measured state (plant_state::tire_use, load_transfer) modulates the drive
  // (H4); sim_world reads it each tick. Off (no gain) from a plant that does not measure it.
  virtual plant_modulation modulation() const { return {}; }
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_VEHICLE_PLANT_H__

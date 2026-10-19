/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics as cvc::nav's plant (P4, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md).
//
//   cvc::physics::world w(cfg);
//   ... the ground, buildings ...
//   cvc::physics::nav_plant plant(w, {cvc::physics::vehicle_desc::humvee()});
//   sim.set_plant(&plant); // nav's agents now drive Humvees
//   for (;;) sim.step();   // each tick: nav's command, six physics steps, the readback
//
// Agent i of the sim_world drives vehicle i, a vehicle_desc from the list (cycled), added at
// attach() with its rear axle on the agent's start and its heading. This is the one place nav's
// normalized frame meets SI: positions, speeds and accelerations cross through world_units with
// the sim_world's frame (nav::frame_of(cfg)).
//
// The readback (doc 31 D14) is the rear axle's position (the rearmost unsteered axle, which nav's
// bicycle tracks), the body's heading, and the signed forward speed at the rear axle: the
// centre of mass's velocity carried to the rear axle by the body's rotation, along the body's
// forward axis.
//
// The command goes to world::drive: the bicycle angle as it is, and the acceleration in the
// direction the agent is to move at the tick's end (reverse when that is backwards).

#ifndef __CVC_PHYSICS_NAV_PLANT_H__
#define __CVC_PHYSICS_NAV_PLANT_H__

#include <cstdint>
#include <cvc/core/world_units.h>
#include <cvc/nav/terrain.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/world.h>
#include <set>
#include <string>
#include <vector>

namespace cvc {

class app;

namespace physics {

struct nav_plant_options {
  // The ground under the agents' starts (world metres), and how high above it the chassis
  // origin is added: < 0 = just above the wheels' full reach (their suspension extended).
  double ground_z = 0.0;
  double spawn_height = -1.0;
  // advance() steps the world dt / fixed_dt times (a whole number, or attach() throws). false:
  // the world steps itself on the clock nav follows (world::follow), and advance() only reads.
  bool step_world = true;
  // The vehicle's response lag beyond the tick it holds a command (s): the driver model, the
  // pedals and the engine. The stopping governor takes it off the distance to a wall.
  double latency = 0.1;
  // The share of the vehicle's max_decel the stopping governor plans with.
  double brake_margin = 0.8;
  // Nav's world frame in metres (world_units::config::metres_per_world_unit).
  double metres_per_world_unit = 1.0;
  // The drift summary's lateral-acceleration bins: their upper edges in g, ascending. A last,
  // open bin holds the rest.
  std::vector<double> drift_bins_g{0.1, 0.2, 0.3, 0.5};
  // The kinematic reference's substeps a tick; 0: the physics steps a tick (D14's nsub = k).
  int drift_nsub = 0;
  // How the measured tire use and load transfer modulate the drive (H4); off by default.
  cvc::nav::plant_modulation modulation;
  // The navigator plans with each vehicle's own envelope and size (limits()):
  //   - its acceleration, accel_margin x max_accel;
  //   - its top speed, max_speed, but no more than nav's vmax (a speed limit);
  //   - its cornering, lateral_g in g, but at most tip_margin of its static stability factor
  //     (half its track over its centre of gravity's height) in g;
  //   - its size: the chassis box and the wheels, from the rear axle.
  // false: nav's own a_max, vmax, a_lat_max and footprint (the steering, braking and latency
  // limits come from the vehicle either way).
  bool plan_with_vehicle = true;
  double accel_margin = 0.8;
  double lateral_g = 0.5;
  double tip_margin = 0.8;
};

class nav_plant final : public cvc::nav::vehicle_plant {
public:
  // `descs` must not be empty; agent i drives descs[i % descs.size()].
  nav_plant(world &w, std::vector<vehicle_desc> descs, nav_plant_options options = {});
  // Leaves its vehicles in the world (the world may already be gone).
  ~nav_plant() override = default;
  nav_plant(const nav_plant &) = delete;
  nav_plant &operator=(const nav_plant &) = delete;

  // ── cvc::nav::vehicle_plant ──
  cvc::nav::plant_kind kind() const override { return cvc::nav::plant_kind::jolt; }
  // Adds one vehicle per agent (removing any of an earlier attach) at rest.
  void attach(int n, const cvc::nav::plant_state *start,
              const cvc::world_units::normalized_frame &frame, double dt) override;
  cvc::nav::plant_state state(int i) const override;
  cvc::nav::plant_limits limits(int i) const override;
  void command(int i, float steer, float accel) override;
  void advance() override;
  void report_drift(int i, const cvc::nav::drift_sample &s) override;
  int reference_substeps() const override;
  double energy_step(int i) const override;
  int static_contacts_step(int i) const override;
  bool tracks_energy() const override { return true; }
  cvc::nav::plant_modulation modulation() const override { return _o.modulation; }

  // The terrain limits of a vehicle, for sim_world::set_terrain (P5): its steepest climb, the
  // lesser of what its tires hold on `mu` and what its first gear (with a converter's stall
  // multiplication) pulls at its mass; and its static stability factor, the track over twice the
  // centre of gravity's height above the ground.
  static cvc::nav::terrain_config terrain_config_of(const vehicle_desc &d, double mu = 0.9,
                                                    double gravity = 9.80665);

  // ── the vehicles ──
  int size() const { return static_cast<int>(_agents.size()); }
  vehicle_id vehicle(int i) const { return _agents.at(i).vehicle; }
  const vehicle_desc &desc(int i) const { return _agents.at(i).desc; }
  world &physics_world() { return _w; }

  // ── the drift, in SI ──
  struct drift_si {
    double position_m = 0.0;
    double heading_rad = 0.0;
    double speed_mps = 0.0;
    double lateral_g = 0.0;      // the bicycle's lateral acceleration, in g
    double stop_speed_mps = 0.0; // the stopping speed the tick's command respected
  };
  // Agent i's last tick.
  const drift_si &last_drift(int i) const { return _agents.at(i).drift; }
  // Every tick of every agent, binned by lateral g.
  struct drift_bin {
    double lo_g = 0.0, hi_g = 0.0; // hi_g is +inf for the last bin
    std::uint64_t ticks = 0;
    double position_rms_m = 0.0, position_max_m = 0.0;
    double heading_rms_rad = 0.0, heading_max_rad = 0.0;
    double speed_rms_mps = 0.0, speed_max_mps = 0.0;
  };
  std::vector<drift_bin> drift_summary() const;
  void clear_drift();

  // ── the cost ──
  // advance()'s wall time: the mean per tick, and per vehicle per tick (microseconds).
  double advance_us() const;
  double advance_us_per_vehicle() const;

  // ── the settings ──
  const nav_plant_options &options() const { return _o; }
  // The live ones; an invalid value throws cvc::physics_error and changes nothing.
  void set_latency(double s);          // >= 0
  void set_brake_margin(double share); // (0, 1]
  void set_drift_nsub(int n);          // >= 0
  void set_plan_with_vehicle(bool on);
  void set_accel_margin(double share); // (0, 1]
  void set_lateral_g(double g);        // (0, 2]
  void set_tip_margin(double share);   // (0, 1]
  // Onsets >= 0, gains >= 0.
  void set_modulation(const cvc::nav::plant_modulation &m);

  // ── the state tree ──
  // The live settings, read by pull() and seeded with the plant's values when unset (an
  // invalid value is ignored):
  //   <prefix>.nav_plant.{latency, brake_margin, drift_nsub}
  //   <prefix>.nav_plant.plan.{with_vehicle, accel_margin, lateral_g, tip_margin}
  //   <prefix>.nav_plant.modulation.{tire_onset, tire_gain, tip_onset, tip_gain}
  // and the rest, which are fixed, written by publish():
  //   <prefix>.nav_plant.settings.{step_world, ground_z, spawn_height, metres_per_world_unit,
  //     physics_steps, drift_bins_g}
  void pull(cvc::app &ctx, const std::string &prefix);
  // Agent i's nav keys beside the physics binding's vehicles.<name>.* (world_binding:
  // publish_vehicle(names[i], vehicle(i))), and the drift summary and cost:
  //   <prefix>.vehicles.<names[i]>.nav.{steer_command, accel_command}   rad, m/s^2
  //   <prefix>.vehicles.<names[i]>.nav.{tire_use, load_transfer, pitch_rad, roll_rad}
  //   <prefix>.vehicles.<names[i]>.nav.drift.{position_m, heading_rad, speed_mps, lateral_g,
  //     stop_speed_mps}
  //   <prefix>.nav_plant.drift.<b>.{lo_g, hi_g, ticks, position_rms_m, position_max_m,
  //     heading_rms_rad, heading_max_rad, speed_rms_mps, speed_max_mps}
  //   <prefix>.nav_plant.{vehicles, advance_us, advance_us_per_vehicle}
  // A name may be empty to leave that agent out. Every key says what it is, as world_binding's
  // do: a comment and the properties access, unit and (on a knob) a recommended min and max.
  void publish(cvc::app &ctx, const std::string &prefix,
               const std::vector<std::string> &names) const;

private:
  struct agent {
    vehicle_desc desc;
    vehicle_id vehicle;
    body_id chassis;
    double rear_x = 0.0;        // the rear axle, body x (m)
    double drawn = 0.0;         // energy drawn so far (J)
    double energy_step = 0.0;   // last tick (J)
    int contacts_step = 0;      // last tick
    double steer_command = 0.0; // rad
    double accel_command = 0.0; // m/s^2 along the heading
    drift_si drift;
  };
  struct bin_sums {
    std::uint64_t ticks = 0;
    double pos2 = 0.0, pos_max = 0.0, head2 = 0.0, head_max = 0.0, speed2 = 0.0, speed_max = 0.0;
  };

  double drawn(const agent &a) const;
  void count_contacts();

  world &_w;
  std::vector<vehicle_desc> _descs;
  nav_plant_options _o;
  cvc::world_units _units;
  cvc::world_units::normalized_frame _frame;
  double _dt = 0.0;
  int _steps = 0; // physics steps per tick
  std::vector<agent> _agents;
  std::vector<bin_sums> _bins;
  std::uint64_t _ticks = 0;
  double _advance_us = 0.0; // summed
  // The keys described so far (prefix-qualified), so each is described once.
  mutable std::set<std::string> _described;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_NAV_PLANT_H__

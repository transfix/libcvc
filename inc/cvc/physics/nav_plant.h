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
#include <cvc/nav/dyn_4w.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/terrain.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/world.h>
#include <functional>
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
  // The ground's height under a start (world metres, at the rear axle's x, y in metres), for a
  // terrain that is not flat; empty: ground_z everywhere.
  std::function<double(double x, double y)> ground_height;
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
  // Agent i's contact with the other agents' vehicles over the last tick: the other vehicle it
  // touched (its agent index; -1 none; the one it touched deepest), and the deepest penetration
  // (m). From the engine's contact events between two chassis, begun or persisting, so a vehicle
  // resting against another counts every tick it does.
  int vehicle_contact_step(int i) const;
  double vehicle_contact_depth(int i) const;
  // Agent i's chassis against a wall over the last tick: a static or kinematic body met with the
  // contact's normal within 45 degrees of horizontal (a building's side, a cell box, a kerb),
  // from when the contact begins until it ends (a vehicle at rest against a wall sleeps, and its
  // contacts persist without the engine reporting them), and the deepest such penetration (m).
  // The ground under a vehicle on its side or roof meets it from below, so it is not a wall.
  bool wall_contact_step(int i) const;
  double wall_contact_depth(int i) const;
  bool tracks_energy() const override { return true; }
  // The plant ledger's sample of agent i at the end of the last tick (at attach(), the vehicle
  // at rest and no command), from the vehicle's state then. The fields it shares with
  // plant_log's planar_sample are computed as plant_log computes them: the centre of mass's
  // velocity in the heading's frame, grade and roll the sines of the body's pitch and roll
  // (fwd.z and left.z), a_lat / a_lon the change of its horizontal velocity over the tick
  // across the mean heading. ltr classifies a wheel by its contact point's side of the centre of
  // mass along the body's left axis.
  cvc::nav::plant_sample sample(int i) const override;
  bool tracks_samples() const override { return true; }
  cvc::nav::plant_modulation modulation() const override { return _o.modulation; }

  // The terrain limits of a vehicle, for sim_world::set_terrain (P5): its steepest climb, the
  // lesser of what its tires hold on `mu` and what its first gear (with a converter's stall
  // multiplication) pulls at its mass; and its static stability factor, the track over twice the
  // centre of gravity's height above the ground.
  static cvc::nav::terrain_config terrain_config_of(const vehicle_desc &d, double mu = 0.9,
                                                    double gravity = 9.80665);

  // A vehicle's dynamic single-track parameters (nav/dyn_bike.h, P8a), in SI, from its spec
  // sheet, so both plants read one vehicle_desc:
  //   - mass; yaw inertia of the chassis box, m (hx^2 + hy^2) / 3;
  //   - the centre of gravity's distances to the front (steered) and rear axles, and its height
  //     as terrain_config_of has it;
  //   - Pacejka C from the tires' sliding fraction (sin(C pi / 2) is tire_sliding) and B fitted to
  //     how Jolt's tires corner. Jolt's lateral friction ramps linearly to its peak at
  //     tire_peak_slip_angle, a cornering stiffness of 1 / alpha_peak per unit load, and a Pacejka
  //     tire peaking at the same angle starts 4.5x steeper (B C = C tan(pi / 2C) / alpha_peak).
  //     So B C = k / alpha_peak with k = kJoltTireStiffness (1.12), fitted to the one-tick
  //     residual of Jolt runs of the Humvee and the RoboRacer (libcvc #749's fit_dyn_bike;
  //     held-out twist residual about 11x lower than with the 4.5x tire). `front_stiffness`
  //     scales the front's B; Jolt's tires have no load sensitivity or steering compliance, and
  //     the fit puts front and rear alike (1);
  //   - the drive split from which axles are driven, the brake split from the axles' brake
  //     torques; drag 1/2 rho C_d A and the rolling resistance;
  //   - the steering's rate; `drive_lag` for the force, < 0 for the vehicle's own: a direct drive
  //     (one gear, no torque converter) answers within the tick (0), a geared drive through a
  //     converter takes 0.1 s (nav_plant_options::latency's default);
  //   - for a direct drive, no drag or rolling resistance: the driver model's integral trim tracks
  //     the commanded acceleration and makes up for them, so the commanded acceleration is the
  //     achieved one (the RoboRacer's fit: lag 0, resistance x0; the Humvee's keeps both).
  static constexpr double kJoltTireStiffness = 1.12;
  static cvc::nav::dyn_bike_si dyn_bike_si_of(const vehicle_desc &d, double mu = 0.9,
                                              double front_stiffness = 1.0, double drive_lag = -1.0,
                                              double gravity = 9.80665);

  // The same vehicle in a normalized frame where its wheelbase is `wheelbase`: S =
  // wheelbase / d.wheelbase(). This is how a trainer whose scene has no metres
  // (coef_train's city, veh_L = 0.035) gets a physical vehicle for rollout_kind::dynamic:
  // dyn_bike_params_at(vehicle_desc::roboracer(), 0.035) reads the scene at about 0.108
  // normalized units per metre, where vmax 0.9 is 8.3 m/s.
  static cvc::nav::diff::dyn_bike_params dyn_bike_params_at(const vehicle_desc &d, double wheelbase,
                                                            double mu = 0.9);

  // The same vehicle on four wheels (nav/dyn_4w.h, P8d): dyn_bike_si_of's numbers, with its
  // defaults (front_stiffness 1, drive_lag < 0 for the vehicle's own), plus
  //   - the half tracks of the front (steered) and rear axles;
  //   - the chassis box's roll and pitch inertias, m (hy^2 + hz^2) / 3 and m (hx^2 + hz^2) / 3;
  //   - each corner's spring from its axle's suspension frequency f and the corner's static
  //     load m_c (k = m_c (2 pi f)^2), and its damper from the damping ratio z (c = 2 z
  //     sqrt(k m_c)). The anti-roll bars are left out.
  static cvc::nav::dyn4_si dyn4_si_of(const vehicle_desc &d, double front_stiffness = 1.0,
                                      double drive_lag = -1.0, double gravity = 9.80665);
  // dyn4_si_of in a normalized frame where its wheelbase is `wheelbase`, as dyn_bike_params_at.
  static cvc::nav::diff::dyn4_params dyn4_params_at(const vehicle_desc &d, double wheelbase);

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
    int touched = -1;           // the vehicle it touched deepest last tick (agent index); -1 none
    double touch_depth = 0.0;   // that contact's penetration, m
    bool wall = false;          // a wall contact began or persisted last tick
    double wall_depth = 0.0;    // the deepest of those contacts' penetrations last tick, m
    struct wall_touch {
      std::uint32_t body = 0, sub_wall = 0, sub_chassis = 0; // the contact's (removed by them)
      double depth = 0.0;                                    // its last penetration, m
    };
    std::vector<wall_touch> walls; // its wall contacts begun and not yet removed
    double steer_command = 0.0; // rad
    double accel_command = 0.0; // m/s^2 along the heading
    drift_si drift;
    cvc::nav::plant_sample sample; // as of the last tick (sample())
    double psi = 0.0;              // the heading then, rad
    double vwx = 0.0, vwy = 0.0;   // the centre of mass's world velocity then, m/s
  };
  struct bin_sums {
    std::uint64_t ticks = 0;
    double pos2 = 0.0, pos_max = 0.0, head2 = 0.0, head_max = 0.0, speed2 = 0.0, speed_max = 0.0;
  };

  double drawn(const agent &a, const vehicle_energy &e) const;
  // Agent a's plant_sample from its state `vs`; `tick`: over a tick of _dt since the last one
  // (the accelerations), else at rest (attach).
  void measure(agent &a, const physics::vehicle_state &vs, bool tick) const;
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

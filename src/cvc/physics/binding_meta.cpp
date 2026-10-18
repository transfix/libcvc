/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "binding_meta.h"

#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <limits>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr double N = std::numeric_limits<double>::quiet_NaN();

// The world's own keys, under the prefix.
const key_meta kWorld[] = {
    // built with the world
    {"backend", "build", nullptr, N, N, "The physics backend: null or jolt."},
    {"fixed_dt", "build", "s", 0.001, 0.05,
     "Seconds per physics step; 0.01 is 100 Hz, the rate the vehicle presets are tuned at."},
    {"threads", "build", "", 0, 64, "Worker threads in a step; 0 runs it on the calling thread."},
    {"determinism", "build", nullptr, N, N,
     "same_binary (one build reproduces a run) or cross_platform (every target does)."},
    {"max_bodies", "build", "", 16, 1e6, "The most bodies the world holds."},
    {"max_body_pairs", "build", "", 16, 1e7, "The most body pairs the broad phase tracks."},
    {"max_contact_constraints", "build", "", 16, 1e7,
     "The most contact constraints a step solves; saturated reports when it fills."},
    {"origin.x", "build", "m", N, N,
     "The floating origin, x: world metres the engine's are offset by."},
    {"origin.y", "build", "m", N, N, "The floating origin, y."},
    {"origin.z", "build", "m", N, N, "The floating origin, z."},
    // applied by every pull()
    {"gravity.x", "knob", "m/s^2", -50, 50, "Gravity, x."},
    {"gravity.y", "knob", "m/s^2", -50, 50, "Gravity, y."},
    {"gravity.z", "knob", "m/s^2", -50, 50, "Gravity, z (-9.80665 on Earth; z is up)."},
    {"collision_steps", "knob", "", 1, 32,
     "Engine sub-steps a step. A vehicle's drivetrain runs once per sub-step: 8 keeps the "
     "Humvee's energy error near 1%, 32 the RoboRacer's."},
    {"velocity_steps", "knob", "", 1, 50, "Velocity solver iterations a step."},
    {"position_steps", "knob", "", 1, 20, "Position solver iterations a step."},
    {"penetration_slop", "knob", "m", 0, 0.05,
     "How far bodies may overlap before the solver pushes them apart."},
    {"friction_combine", "knob", nullptr, N, N,
     "How two surfaces' friction combine: product, min, max, geometric_mean or average."},
    {"restitution_combine", "knob", nullptr, N, N,
     "How two surfaces' restitution combine: product, min, max, geometric_mean or average."},
    {"contacts", "knob", nullptr, N, N,
     "Which contacts a step reports: none, added_removed, or all (persisting ones too)."},
    // how often push() publishes
    {"publish_every", "knob", "", 1, 1000,
     "step() and follow() push every N steps (with fixed_dt 0.01, 10 is 10 Hz)."},
    {"snapshot_every", "knob", "", 0, 1000,
     "The snapshot data at most every push, every N steps; 0 never."},
    {"snapshot_json_every", "knob", "", 0, 1000,
     "The snapshot as JSON every N steps; 0 never (JSON costs the most)."},
    {"hash_every", "knob", "", 0, 1000,
     "state_hash every N steps; 0 never (it digests the whole state)."},
    {"publish_thread", "knob", "", 0, 1,
     "1: a background thread writes what a push publishes, so a step pays only for the snapshot."},
    // written by push()
    {"tick", "telemetry", "", N, N, "Steps the world has taken."},
    {"time", "telemetry", "s", N, N, "Simulated seconds."},
    {"step_ms", "telemetry", "ms", N, N, "Wall-clock milliseconds the last step took."},
    {"bodies", "telemetry", "", N, N, "Bodies in the world."},
    {"active_bodies", "telemetry", "", N, N, "Bodies awake."},
    {"contact_events", "telemetry", "", N, N, "Contact events the last step reported."},
    {"saturated", "telemetry", "", N, N,
     "1 when an engine buffer filled and contacts were dropped: raise the max_* settings."},
    {"state_hash", "telemetry", nullptr, N, N,
     "A digest of the whole simulation state (hash_every)."},
    {"version", "telemetry", nullptr, N, N, "The backend and its build options."},
    {"restart_required", "telemetry", "", N, N,
     "1 when a build setting differs from the running world."},
    {"snapshot", "telemetry", nullptr, N, N,
     "Every body, field, vehicle and contact at one tick, as data (world_snapshot)."},
    {"snapshot_json", "telemetry", nullptr, N, N,
     "The snapshot as JSON, without the static bodies."},
    {"snapshot_json_static", "telemetry", nullptr, N, N,
     "The static bodies as JSON, rewritten only when one changes."},
    {"publish_dropped", "telemetry", "", N, N,
     "Publications the publish thread skipped because a newer one replaced them."},
};

const key_meta kMaterial[] = {
    {"friction", "knob", "1", 0, 2,
     "Coulomb friction: the tire's peak grip and the contacts' friction (0.9 dry road, 0.5 wet, "
     "0.1 ice)."},
    {"restitution", "knob", "1", 0, 1, "Bounce: 0 none, 1 elastic."},
    {"rolling_resistance", "knob", "1", 0, 0.3,
     "C_rr for a vehicle on it (0.015 a hard road, 0.05 dirt, 0.1 or more sand)."},
    {"nav_material", "knob", "", -1, 255,
     "The cvc::nav material id this surface carries; -1 none."},
};

const key_meta kBody[] = {
    {"id", "telemetry", "", N, N, "The body's id."},
    {"motion", "telemetry", nullptr, N, N, "static, kinematic or dynamic."},
    {"position.x", "telemetry", "m", N, N, "The body's origin, x."},
    {"position.y", "telemetry", "m", N, N, "The body's origin, y."},
    {"position.z", "telemetry", "m", N, N, "The body's origin, z."},
    {"rotation.w", "telemetry", "1", N, N, "Orientation quaternion, w."},
    {"rotation.x", "telemetry", "1", N, N, "Orientation quaternion, x."},
    {"rotation.y", "telemetry", "1", N, N, "Orientation quaternion, y."},
    {"rotation.z", "telemetry", "1", N, N, "Orientation quaternion, z."},
    {"velocity.x", "telemetry", "m/s", N, N, "Linear velocity, x."},
    {"velocity.y", "telemetry", "m/s", N, N, "Linear velocity, y."},
    {"velocity.z", "telemetry", "m/s", N, N, "Linear velocity, z."},
    {"angular_velocity.x", "telemetry", "rad/s", N, N, "Angular velocity, x."},
    {"angular_velocity.y", "telemetry", "rad/s", N, N, "Angular velocity, y."},
    {"angular_velocity.z", "telemetry", "rad/s", N, N, "Angular velocity, z."},
    {"active", "telemetry", "", N, N, "1 while awake."},
    {"field_force.x", "telemetry", "N", N, N, "The fields' force on it (buoyancy, drag), x."},
    {"field_force.y", "telemetry", "N", N, N, "The fields' force on it, y."},
    {"field_force.z", "telemetry", "N", N, N, "The fields' force on it, z."},
    {"immersed_volume", "telemetry", "m^3", N, N, "Its volume inside a fluid field."},
    {"material", "knob", nullptr, N, N, "The name of its material (materials.<name>)."},
    {"mass", "knob", "kg", 0.001, 1e7, "A dynamic body's mass."},
    {"linear_damping", "knob", "1/s", 0, 1, "Damping of its linear velocity."},
    {"angular_damping", "knob", "1/s", 0, 1, "Damping of its angular velocity."},
    {"allow_sleeping", "knob", "", 0, 1, "1 lets it sleep when it comes to rest."},
};

const key_meta kVehicle[] = {
    // telemetry
    {"chassis", "telemetry", "", N, N, "The chassis body's id."},
    {"speed", "telemetry", "m/s", N, N, "Speed along the chassis's forward axis (< 0 backing)."},
    {"steer", "telemetry", "rad", N, N, "The steering actuator's bicycle angle now (+ left)."},
    {"steer_command", "telemetry", "rad", N, N, "The angle the actuator slews toward."},
    {"throttle", "telemetry", "1", N, N, "The throttle this step, 0 to 1."},
    {"brake", "telemetry", "1", N, N, "The brake this step, 0 to 1."},
    {"hand_brake", "telemetry", "1", N, N, "The hand brake this step, 0 to 1."},
    {"gear", "telemetry", "", N, N, "-1 reverse, 0 neutral, 1 and up forward."},
    {"engine_rpm", "telemetry", "rpm", N, N, "The engine's (motor's) speed."},
    {"accel_trim", "telemetry", "m/s^2", N, N,
     "The driver model's integral trim: what its feed-forward was missing."},
    {"position.x", "telemetry", "m", N, N, "The chassis origin, x."},
    {"position.y", "telemetry", "m", N, N, "The chassis origin, y."},
    {"position.z", "telemetry", "m", N, N, "The chassis origin, z."},
    {"rotation.w", "telemetry", "1", N, N, "Chassis orientation quaternion, w."},
    {"rotation.x", "telemetry", "1", N, N, "Chassis orientation quaternion, x."},
    {"rotation.y", "telemetry", "1", N, N, "Chassis orientation quaternion, y."},
    {"rotation.z", "telemetry", "1", N, N, "Chassis orientation quaternion, z."},
    {"velocity.x", "telemetry", "m/s", N, N, "Chassis velocity, x."},
    {"velocity.y", "telemetry", "m/s", N, N, "Chassis velocity, y."},
    {"velocity.z", "telemetry", "m/s", N, N, "Chassis velocity, z."},
    {"angular_velocity.x", "telemetry", "rad/s", N, N, "Chassis angular velocity, x."},
    {"angular_velocity.y", "telemetry", "rad/s", N, N, "Chassis angular velocity, y."},
    {"angular_velocity.z", "telemetry", "rad/s", N, N, "Chassis angular velocity, z (yaw rate)."},
    {"aero_force.x", "telemetry", "N", N, N, "The aerodynamic drag applied, x."},
    {"aero_force.y", "telemetry", "N", N, N, "The aerodynamic drag applied, y."},
    {"aero_force.z", "telemetry", "N", N, N, "The aerodynamic drag applied, z."},
    {"air_velocity.x", "telemetry", "m/s", N, N, "The air's velocity at the chassis (a wind), x."},
    {"air_velocity.y", "telemetry", "m/s", N, N, "The air's velocity at the chassis, y."},
    {"air_velocity.z", "telemetry", "m/s", N, N, "The air's velocity at the chassis, z."},
    // the energy ledger (P3), joules since the vehicle was added
    {"energy.P_W", "telemetry", "W", N, N, "The power the engine supplied in the last step."},
    {"energy.supplied_J", "telemetry", "J", N, N, "The engine's (motor's) shaft work."},
    {"energy.idle_J", "telemetry", "J", N, N,
     "What holding the engine at idle speed added (< 0 when the rev limit took some)."},
    {"energy.brake_J", "telemetry", "J", N, N, "Dissipated by the brakes."},
    {"energy.tire_J", "telemetry", "J", N, N, "Dissipated by tire slip."},
    {"energy.aero_J", "telemetry", "J", N, N, "Dissipated by aerodynamic drag."},
    {"energy.roll_J", "telemetry", "J", N, N, "Dissipated by rolling resistance."},
    {"energy.medium_J", "telemetry", "J", N, N, "Dissipated compacting snow, mud or sand."},
    {"energy.drivetrain_J", "telemetry", "J", N, N,
     "Dissipated in the drivetrain: the gears, the converter's slip, engine braking, damping."},
    {"energy.drivetrain_split_J", "telemetry", "J", N, N,
     "What Jolt's split drivetrain step adds: a numerical error, which shrinks with the step."},
    {"energy.suspension_J", "telemetry", "J", N, N, "Dissipated by the dampers."},
    {"energy.chassis_damping_J", "telemetry", "J", N, N,
     "Dissipated by the chassis's own damping."},
    {"energy.fields_J", "telemetry", "J", N, N, "Work the fields did on the chassis."},
    {"energy.stored_J", "telemetry", "J", N, N,
     "The change in kinetic, potential and spring energy."},
    {"energy.residual_J", "telemetry", "J", N, N,
     "What the balance leaves: the simulation's unexplained energy error."},
    {"energy.regen_J", "telemetry", "J", N, N, "Braking energy returned to a battery."},
    {"energy.source_J", "telemetry", "J", N, N,
     "Drawn from the battery (electrical) or the fuel (chemical)."},
    {"energy.soc", "telemetry", "1", N, N, "A battery's state of charge, 0 to 1."},
    {"energy.fuel_kg", "telemetry", "kg", N, N, "Fuel burnt."},
    {"energy.fuel_left_kg", "telemetry", "kg", N, N, "Fuel left in the tank."},
    {"energy.distance_m", "telemetry", "m", N, N, "Distance the chassis travelled."},
    {"energy.Wh_per_km", "telemetry", "Wh/km", N, N,
     "Source energy per kilometre (shaft work without a source)."},
    // the navigator's command, through the driver model
    {"command.steer", "command", "rad", -1.5, 1.5, "The bicycle angle to steer to (+ left)."},
    {"command.accel", "command", "m/s^2", -15, 15,
     "The acceleration wanted along the direction of travel; < 0 slows."},
    {"command.reverse", "command", "", 0, 1, "1 drives backwards."},
    // pedals, past the driver model
    {"controls.steer", "command", "rad", -1.5, 1.5,
     "The bicycle angle (still through the actuator)."},
    {"controls.throttle", "command", "1", 0, 1, "The throttle, 0 to 1."},
    {"controls.brake", "command", "1", 0, 1, "The brake, 0 to 1."},
    {"controls.hand_brake", "command", "1", 0, 1, "The hand brake, 0 to 1."},
    {"controls.reverse", "command", "", 0, 1, "1 selects reverse."},
    {"controls.neutral", "command", "", 0, 1, "1 selects neutral."},
    // tuning (world::set_vehicle)
    {"tuning.max_steer", "knob", "rad", 0.05, 1.2, "The steering lock: the largest bicycle angle."},
    {"tuning.max_steer_rate", "knob", "rad/s", 0.05, 10,
     "How fast the steering actuator slews (0.35 a Humvee, 3 a RoboRacer)."},
    {"tuning.max_accel", "knob", "m/s^2", 0.1, 15,
     "The driver model's full-throttle acceleration, its feed-forward scale."},
    {"tuning.max_decel", "knob", "m/s^2", 0.1, 15,
     "The driver model's full-brake deceleration, its feed-forward scale."},
    {"tuning.drag_coefficient", "knob", "1", 0, 2, "The aerodynamic drag coefficient, C_d."},
    {"tuning.frontal_area", "knob", "m^2", 0, 20, "The frontal area the drag acts on, A."},
    {"tuning.air_density", "knob", "kg/m^3", 0, 1.5,
     "The air's density where no flow field covers it."},
    {"tuning.rolling_resistance", "knob", "1", 0, 0.3,
     "C_rr where the ground's material has none."},
    {"tuning.engine_max_torque", "knob", "N m", 0.01, 5000,
     "The engine's (motor's) peak torque, which the torque curve scales."},
    {"tuning.engine_min_rpm", "knob", "rpm", 0, 2000, "Idle speed (0 for a motor)."},
    {"tuning.engine_max_rpm", "knob", "rpm", 100, 40000,
     "The governor's or speed limit's top speed."},
    {"tuning.shift_up_rpm", "knob", "rpm", 100, 40000, "Upshift at full throttle."},
    {"tuning.shift_down_rpm", "knob", "rpm", 0, 40000, "Downshift at full throttle."},
    {"tuning.shift_up_rpm_light", "knob", "rpm", 0, 40000,
     "Upshift with the throttle shut; the automatic interpolates with the throttle. 0 = the "
     "full-throttle point."},
    {"tuning.shift_down_rpm_light", "knob", "rpm", 0, 40000,
     "Downshift with the throttle shut. 0 = the full-throttle point."},
    {"tuning.clutch_strength", "knob", "N m s/rad", 0.1, 1000,
     "The engine-gearbox coupling per rad/s of slip (a converter's when converter_k > 0)."},
    {"tuning.converter_k", "knob", "rpm/sqrt(N m)", 0, 300,
     "The torque converter's K-factor: it carries at most (rpm / K)^2. 0 = no converter."},
    {"tuning.converter_stall", "knob", "1", 1, 3.5,
     "The converter's torque multiplication with the gearbox input still."},
    {"tuning.converter_coupling", "knob", "1", 0.5, 1,
     "The speed ratio where the converter stops multiplying."},
    {"tuning.driveline_efficiency", "knob", "1", 0.5, 1,
     "The share of the engine's torque the gears deliver to the wheels."},
    {"tuning.engine_friction_curve", "knob", "rpm:N m", N, N,
     "The engine's motoring friction against its speed, as rpm:N m points (\"700:78, 2040:87, "
     "3400:128\"); \"none\" for none. It brakes only with the throttle shut."},
};

const key_meta kWheel[] = {
    {"steer", "telemetry", "rad", N, N, "The wheel's steer angle (+ left)."},
    {"angular_velocity", "telemetry", "rad/s", N, N, "Its spin (+ rolling forward)."},
    {"suspension_length", "telemetry", "m", N, N, "The suspension's length from its attachment."},
    {"contact", "telemetry", "", N, N, "1 while it touches the ground."},
    {"contact_point.x", "telemetry", "m", N, N, "Where it touches, x."},
    {"contact_point.y", "telemetry", "m", N, N, "Where it touches, y."},
    {"contact_point.z", "telemetry", "m", N, N,
     "Where it touches, z (on a medium, its compacted surface)."},
    {"load", "telemetry", "N", N, N, "The suspension's push on the ground."},
    {"longitudinal_slip", "telemetry", "1", N, N, "Slip ratio."},
    {"lateral_slip", "telemetry", "rad", N, N, "Slip angle."},
    {"friction", "telemetry", "1", N, N,
     "The ground's mu the tire used (its material or a medium)."},
    {"rolling_resistance", "telemetry", "N", N, N, "Rolling resistance against the wheel."},
    {"medium_depth", "telemetry", "m", N, N, "Medium (snow, mud, sand) over the ground under it."},
    {"sinkage", "telemetry", "m", N, N, "How far it has sunk below the medium's fresh surface."},
    {"rut", "telemetry", "m", N, N, "The rut it found, before this pass."},
    {"medium_resistance", "telemetry", "N", N, N, "The medium's compaction resistance."},
};

const key_meta kAxle[] = {
    {"max_brake_torque", "knob", "N m", 0, 20000, "Each wheel's brake torque at full brake."},
    {"max_hand_brake_torque", "knob", "N m", 0, 20000, "Each wheel's hand brake torque."},
    {"suspension_frequency", "knob", "Hz", 0.5, 10, "The suspension's natural frequency."},
    {"suspension_damping", "knob", "1", 0, 2, "The suspension's damping ratio (1 is critical)."},
    {"anti_roll_stiffness", "knob", "N/m", 0, 1e6,
     "The anti-roll bar's stiffness (only on an axle added with one)."},
};

const key_meta kField[] = {
    {"kind", "telemetry", nullptr, N, N, "fluid, flow, wind or medium."},
    {"ruts.cells", "telemetry", "", N, N, "Rut cells pressed so far."},
    {"ruts.deepest", "telemetry", "m", N, N, "The deepest rut."},
    {"drag_coefficient", "knob", "1", 0, 2, "A fluid's or flow's drag coefficient, C_d."},
    {"damping", "knob", "N s/m^4", 0, 1e5, "The field's damping coefficient."},
    {"traction", "knob", "1", 0, 2, "A medium's own tire grip, mu; 0 = the ground's."},
};

std::string join(const std::string &base, const char *key) {
  return base.empty() ? std::string(key) : base + "." + key;
}

template <std::size_t K>
void describe_all(cvc::app &ctx, const std::string &base, const key_meta (&t)[K]) {
  for (const key_meta &m : t)
    describe(ctx, join(base, m.key), m);
}

} // namespace

void describe(cvc::app &ctx, const std::string &path, const key_meta &m) {
  cvc::describe_state_key(ctx, path, m);
}

void describe_world(cvc::app &ctx, const std::string &base) { describe_all(ctx, base, kWorld); }
void describe_material(cvc::app &ctx, const std::string &base) {
  describe_all(ctx, base, kMaterial);
}
void describe_body(cvc::app &ctx, const std::string &base) { describe_all(ctx, base, kBody); }
void describe_field(cvc::app &ctx, const std::string &base) { describe_all(ctx, base, kField); }

void describe_vehicle(cvc::app &ctx, const std::string &base, int wheels, int axles) {
  describe_all(ctx, base, kVehicle);
  for (int i = 0; i < wheels; ++i)
    describe_all(ctx, base + ".wheels." + std::to_string(i), kWheel);
  for (int i = 0; i < axles; ++i)
    describe_all(ctx, base + ".tuning.axles." + std::to_string(i), kAxle);
}

} // namespace detail
} // namespace physics
} // namespace cvc

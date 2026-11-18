/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::mujoco_robot: one articulated robot simulated by MuJoCo, standing on a
// cvc::height_field (docs/MUJOCO.md). MuJoCo owns the robot; the ground is the same height
// field the renderer, nav and the Jolt world read.
//
//   cvc::physics::mujoco_robot_options o;
//   o.model = menagerie + "/unitree_go2/scene.xml";
//   o.terrain = field;                          // in place of the scene's "floor" plane
//   cvc::physics::mujoco_robot go2(o);          // at the "home" keyframe, its servos holding it
//   go2.apply_wrench(go2.body("base"), {0, 60, 0}); // a push, held until changed
//   go2.step(50);
//   go2.clear_wrenches();
//   go2.set_wind(scene_air, clock_t0);          // the scene's wind on every geom (xfrc_applied)
//   cvc::physics::mujoco_energy e = go2.energy(); // MuJoCo's own (mjENBL_ENERGY)
//
// Build: CVC_WITH_MUJOCO, off by default, with MuJoCo from cvcpkg (`cvcpkg install mujoco
// mujoco-menagerie`). Without it, available() is false and the constructor throws
// cvc::unsupported_exception. MuJoCo (Apache-2.0) is linked as its own library and never
// compiled into libcvc; this header names none of its types.
//
// Units and frame: SI, z up, the libcvc world's frame, which is MuJoCo's world frame unmoved.
// The generalized coordinates are MuJoCo's: qpos (nq) and qvel (nv). A free joint's qpos is its
// position and unit quaternion (w, x, y, z); its qvel is the linear velocity (world axes) and
// then the angular velocity (body axes).
//
// State: everything lives on the object (the model, the data, the books). Nothing here installs
// MuJoCo callbacks (mju_user_*) or registers plugins, so two robots share nothing and two threads
// may each step their own. MuJoCo's own fatal errors still go to MuJoCo's handler.
//
// Determinism: MuJoCo documents it for one binary. Two robots built from the same options and
// given the same commands step to the same bits in one build; across builds and platforms they
// need not (the Jolt world's cross-platform mode has no counterpart here). Reading the robot
// (energy(), body_position(), ...) never changes its trajectory.

#ifndef __CVC_PHYSICS_MUJOCO_ROBOT_H__
#define __CVC_PHYSICS_MUJOCO_ROBOT_H__

#include <cstdint>
#include <cvc/physics/types.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class height_field;
namespace physics {
class flow_velocity; // flow_velocity.h: the wind
namespace detail {
class mujoco_engine; // src/cvc/physics/mujoco_engine.h
}

// How an actuator reads its command.
enum class actuator_mode : std::uint8_t {
  torque,   // the actuator's force (N m on a hinge), clamped to its force range
  position, // a joint target (rad): MuJoCo's servo, kp (target - q) - kd qdot, its force clamped to
            // the same range and the target to the joint's range
};

struct mujoco_robot_options {
  // The MJCF file, e.g. <prefix>/share/mujoco-menagerie/unitree_go2/scene.xml.
  std::string model;
  // The ground: replaces the geom named `floor` with this height field (a MuJoCo hfield, which
  // splits each cell along the same diagonal as height_field::height(), so the robot stands where
  // field->height(x, y) says). The floor's contact parameters carry over. Null keeps the model's
  // floor. A field with holes (NaN) is refused: MuJoCo's height fields have none.
  std::shared_ptr<const cvc::height_field> terrain;
  std::string floor = "floor";
  double timestep = 0.0; // s; 0 keeps the model's (the Go2's is 0.002)
  // MuJoCo's energies (mjENBL_ENERGY), for energy() and books().
  bool energy = true;
  // The start: the keyframe's joint angles (empty: the model's qpos0), with the floating base at
  // (x, y) heading yaw (rad, from +x). Its height is the keyframe's, which is measured from z = 0,
  // the floor the model was written for: here it is taken from the ground at (x, y), along the
  // ground's normal there when align_to_terrain (the base tilted with it), else straight up. The
  // base is then raised until no geom reaches into the terrain. A robot without a free joint
  // stays where its model puts it.
  std::string keyframe = "home";
  double x = 0.0, y = 0.0, yaw = 0.0;
  bool align_to_terrain = true;
  // Every actuator starts in `mode`: in position mode its target is its joint's start angle, in
  // torque mode its force is 0. The servos' gains, N m / rad and N m s / rad.
  actuator_mode mode = actuator_mode::position;
  double kp = 60.0, kd = 2.0;
};

// One contact over the last step.
struct mujoco_contact {
  int geom[2] = {-1, -1};
  int body[2] = {-1, -1}; // body 0 is the world, which holds the terrain
  vec3 position;          // world
  vec3 normal;            // unit, from geom[0] toward geom[1]
  double distance = 0.0;  // < 0: the geoms overlap (m)
  vec3 force;             // on body[1] from body[0], world axes (N); repulsive along +normal
  vec3 torque;            // the torsional and rolling part (condim 4 and 6), N m, world axes
};

// MuJoCo's energies of the current state (mjENBL_ENERGY), J.
struct mujoco_energy {
  double potential = 0.0; // gravity and springs
  double kinetic = 0.0;
  double total() const { return potential + kinetic; }
};

// The work each kind of force did on the robot since it was built (or since the restored
// snapshot was saved), J. A step of h adds h f . (v0 + v1) / 2: the generalized force f that
// acted over the step, against the mean of the velocities it began and ended with. Joint damping
// is booked as MuJoCo's Euler step applies it, implicitly (-D v1). The integrator's own error is
// what is left over, residual(), which shrinks with h. Booked for the Euler and implicit
// integrators; under RK4 the forces read back are the last stage's, so the books are rough.
struct mujoco_books {
  double actuators = 0.0;      // the actuators
  double external = 0.0;       // apply_wrench's forces, and the wind's (set_wind)
  double contacts = 0.0;       // contact forces (soft contacts give some energy back)
  double joint_friction = 0.0; // dry friction in the joints (frictionloss)
  double joint_limits = 0.0;   // joint and tendon limits
  double joint_damping = 0.0;  // viscous damping in the joints and tendons
  double other = 0.0;          // equality constraints and the other passive forces
  double energy_start = 0.0;   // E when the books began
  double energy_now = 0.0;     // E of the current state
  double work() const {
    return actuators + external + contacts + joint_friction + joint_limits + joint_damping + other;
  }
  double residual() const { return energy_now - energy_start - work(); }
};

// Everything a robot needs to go back to a moment: MuJoCo's integration state (time, qpos, qvel,
// controls, applied forces, the solver's warm start), the actuators' modes and gains, and the
// books. Opaque: give it to restore() on the robot that saved it.
struct mujoco_snapshot {
  std::vector<double> state;
  std::vector<std::uint8_t> modes;
  std::vector<double> kp, kd;
  mujoco_books books;
};

class mujoco_robot {
public:
  // Whether this build has MuJoCo, and which: e.g. "MuJoCo 4.0.0" ("" without it).
  static bool available();
  static std::string version();

  // Loads options.model, puts the terrain in, places the robot. Throws cvc::physics_error when
  // the model does not load or compile, the floor is missing, the keyframe is unknown, or an
  // actuator is not a single-joint one; cvc::unsupported_exception without MuJoCo.
  explicit mujoco_robot(const mujoco_robot_options &options);
  ~mujoco_robot();
  mujoco_robot(mujoco_robot &&) noexcept;
  mujoco_robot &operator=(mujoco_robot &&) noexcept;
  mujoco_robot(const mujoco_robot &) = delete;
  mujoco_robot &operator=(const mujoco_robot &) = delete;

  // --- the model ---------------------------------------------------------------------------
  int nq() const;
  int nv() const;
  int actuators() const;
  int bodies() const;
  int geoms() const;
  // By name; -1 when there is none.
  int actuator(const std::string &name) const;
  int body(const std::string &name) const;
  int geom(const std::string &name) const;
  std::string actuator_name(int i) const;
  std::string body_name(int b) const;
  std::string geom_name(int g) const;
  int geom_body(int g) const;
  // The terrain's geom (-1 when the model's floor was kept).
  int terrain_geom() const;
  // Actuator i's joint (MuJoCo's id) and where its angle is in qpos and its speed in qvel.
  int actuator_joint(int i) const;
  int actuator_qpos(int i) const;
  int actuator_qvel(int i) const;
  // Actuator i's force range (N m), from the model's forcerange, else a motor's ctrlrange.
  double force_min(int i) const;
  double force_max(int i) const;
  double mass() const;     // kg, every body
  double timestep() const; // s
  double time() const;     // s since the start

  // --- commands ------------------------------------------------------------------------------
  // Switch actuator i (or all) to `m`; its command restarts: a position actuator holds its joint
  // where it is, a torque actuator applies 0.
  void set_mode(int i, actuator_mode m);
  void set_mode(actuator_mode m);
  actuator_mode mode(int i) const;
  // Actuator i's servo gains (used in position mode).
  void set_gains(int i, double kp, double kd);
  double kp(int i) const;
  double kd(int i) const;
  // Actuator i's command (a target in position mode, a force in torque mode), or all of them.
  void command(int i, double u);
  void command(const std::vector<double> &u);
  std::vector<double> commands() const;
  // Every position actuator's target becomes its joint's angle now.
  void hold();

  // --- pushes --------------------------------------------------------------------------------
  // A force and torque on body b (world axes, at its centre of mass), held until changed or
  // cleared; a second call on b replaces the first.
  void apply_wrench(int b, const vec3 &force, const vec3 &torque = {});
  // A force at a world point on body b: the same as its force and moment about b's centre.
  void apply_force_at(int b, const vec3 &force, const vec3 &point);
  void clear_wrenches();

  // --- the wind ------------------------------------------------------------------------------
  // The air the robot stands in (flow_velocity.h: the scene's wind_flow, its storms' gust fronts
  // in it (cvc::ariadne::StormWindFlow), or any flow), read at each step's start at time() +
  // time_offset (a scene passes its clock's time at the robot's start, so the robot reads the
  // gusts the world's bodies read). Each collision geom of a moving body takes the drag
  // 1/2 rho C_d A |u| u: u is the air at its bounding box's centre (the flow's reference velocity
  // at its body's centre of mass times the flow's height profile at the box's centre, as the
  // world's field step reads it) less the velocity of that centre, and A is the box's area across
  // u. The forces and their moments about each body's centre of mass go into MuJoCo's
  // xfrc_applied for that step, beside apply_wrench's (which they neither replace nor show in),
  // and into the books as external work. Null: still air, the default (the robot steps as
  // without this). Throws cvc::physics_error for a non-finite or negative C_d or a density <= 0.
  void set_wind(std::shared_ptr<const flow_velocity> air, double time_offset = 0.0,
                double drag_coefficient = 1.0, double density = 1.225);
  // The air's force on the robot over the last step, summed over its bodies (N, world axes), and
  // its torque about the world's origin (N m); 0 in still air and before the first step.
  vec3 wind_force() const;
  vec3 wind_torque() const;

  // --- stepping ------------------------------------------------------------------------------
  // One fixed step of timestep() (n of them). Throws cvc::physics_error when MuJoCo found the
  // state unusable (NaN or a runaway acceleration) and reset it.
  void step();
  void step(int n);

  // --- the state -----------------------------------------------------------------------------
  std::vector<double> qpos() const;
  std::vector<double> qvel() const;
  // The floating base (the first free joint): position, rotation, linear velocity (world) and
  // angular velocity (world). Without a free joint, body 1's pose and a zero velocity.
  vec3 base_position() const;
  quat base_rotation() const;
  vec3 base_velocity() const;
  vec3 base_spin() const;
  // The base's up axis's world z: 1 upright, 0 on its side.
  double base_up_z() const;
  // Body b's centre (its frame's origin) and rotation now.
  vec3 body_position(int b) const;
  quat body_rotation(int b) const;

  // --- the last step -------------------------------------------------------------------------
  // What each actuator applied (N m), and the contacts that acted: empty before the first step
  // and after restore().
  std::vector<double> actuator_forces() const;
  std::vector<mujoco_contact> contacts() const;

  // --- energy --------------------------------------------------------------------------------
  // MuJoCo's own energies of the current state, and the work books. Throw std::logic_error when
  // the robot was built without energy.
  mujoco_energy energy() const;
  mujoco_books books() const;

  // --- snapshots -----------------------------------------------------------------------------
  mujoco_snapshot save() const;
  void restore(const mujoco_snapshot &s);
  // Back to how the robot was built: the start's state, modes, gains and books (a gym's reset).
  void reset();

private:
  std::unique_ptr<detail::mujoco_engine> _e;
};

// A standing trial, as the tests and `cvc mujoco stand` run it: a robot on a plane that rises
// along +x at slope_deg, settled for `settle` seconds under its servos, then pushed on its base
// with `push` (N, world axes) for push_duration, and run to `duration`. Everything is measured on
// the base.
struct mujoco_trial_options {
  std::string model;
  double slope_deg = 0.0;
  double yaw = 0.0;           // the robot's heading (rad): 0 faces uphill
  double half_extent = 4.0;   // the terrain's half size (m), centred on the robot
  double cell = 0.05;         // its spacing (m)
  double timestep = 0.0;      // 0: the model's
  double kp = 60.0, kd = 2.0; // the servos
  double settle = 1.0;        // s before the push
  vec3 push;                  // N on the base, world axes
  double push_duration = 0.1; // s
  double duration = 4.0;      // s in all
  std::string base = "base";
  // The geoms allowed to touch the terrain; any other touching it is a fall. The Go2's feet.
  std::vector<std::string> feet = {"FL", "FR", "RL", "RR"};
};

struct mujoco_trial_result {
  int steps = 0;
  // At the push (the end of settling) and at the end: the base's height over the terrain under
  // it (m), its up axis's world z, and its speed (m/s).
  double settled_height = 0.0, end_height = 0.0;
  double settled_up_z = 0.0, end_up_z = 0.0;
  double settled_speed = 0.0, end_speed = 0.0;
  // After the push: the base's lowest height and up z, and how far it moved over the ground from
  // where it was at the push (m), at most and at the end.
  double min_height = 0.0, min_up_z = 0.0;
  double max_displacement = 0.0, end_displacement = 0.0;
  // Steps on which a geom that is not a foot touched the terrain.
  int body_contact_steps = 0;
  // Fell: the base tipped past 60 degrees (up z < 0.5) or a geom that is not a foot touched the
  // terrain. Recovered: did not fall, and ends within 2 cm of the settled height, 0.02 of the
  // settled up z, and below 0.05 m/s.
  bool fell = false;
  bool recovered = false;
  mujoco_books books;
};

mujoco_trial_result run_mujoco_trial(const mujoco_trial_options &options);

// For `cvc mujoco`: a robot's model in words (MuJoCo's version, sizes, mass, step, actuators with
// their joints and ranges, bodies), and a trial's result as one JSON object.
std::string describe(const mujoco_robot &robot);
std::string to_json(const mujoco_trial_result &result);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_MUJOCO_ROBOT_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The interface between cvc::physics::mujoco_robot and MuJoCo. Private: not installed. Standard
// library only (and the std-only cvc/physics headers), because the MuJoCo side compiles as its own
// target, cvc_physics_mujoco, without libcvc's other include paths (src/cvc/CMakeLists.txt).
//
// mujoco_robot validates arguments and reads the height field; the engine owns mjModel and
// mjData. The engine reports errors as std exceptions; mujoco_robot turns them into cvc ones.

#ifndef __CVC_PHYSICS_MUJOCO_ENGINE_H__
#define __CVC_PHYSICS_MUJOCO_ENGINE_H__

#include <cvc/physics/mujoco_robot.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// A height field as the engine takes it: sample (r, c) at (x0 + c dx, y0 + r dy), heights
// row-major, row 0 first, no holes.
struct mujoco_grid {
  int rows = 0, cols = 0;
  double x0 = 0.0, y0 = 0.0, dx = 1.0, dy = 1.0;
  std::vector<float> heights;
};

// Where the robot starts: the ground's point under (x, y) and its upward unit normal there.
struct mujoco_ground {
  vec3 point;
  vec3 normal{0.0, 0.0, 1.0};
};

class mujoco_engine {
public:
  virtual ~mujoco_engine() = default;

  virtual int nq() const = 0;
  virtual int nv() const = 0;
  virtual int actuators() const = 0;
  virtual int bodies() const = 0;
  virtual int geoms() const = 0;
  virtual int actuator(const std::string &name) const = 0;
  virtual int body(const std::string &name) const = 0;
  virtual int geom(const std::string &name) const = 0;
  virtual std::string actuator_name(int i) const = 0;
  virtual std::string body_name(int b) const = 0;
  virtual std::string geom_name(int g) const = 0;
  virtual int geom_body(int g) const = 0;
  virtual int terrain_geom() const = 0;
  virtual int actuator_joint(int i) const = 0;
  virtual int actuator_qpos(int i) const = 0;
  virtual int actuator_qvel(int i) const = 0;
  virtual double force_min(int i) const = 0;
  virtual double force_max(int i) const = 0;
  virtual double mass() const = 0;
  virtual double timestep() const = 0;
  virtual double time() const = 0;

  virtual void set_mode(int i, actuator_mode m) = 0;
  virtual actuator_mode mode(int i) const = 0;
  virtual void set_gains(int i, double kp, double kd) = 0;
  virtual double kp(int i) const = 0;
  virtual double kd(int i) const = 0;
  virtual void command(int i, double u) = 0;
  virtual std::vector<double> commands() const = 0;
  virtual void hold() = 0;

  virtual void apply_wrench(int b, const vec3 &force, const vec3 &torque) = 0;
  virtual void clear_wrenches() = 0;
  virtual vec3 body_com(int b) = 0;

  virtual void step() = 0;

  virtual std::vector<double> qpos() const = 0;
  virtual std::vector<double> qvel() const = 0;
  virtual vec3 base_position() = 0;
  virtual quat base_rotation() = 0;
  virtual vec3 base_velocity() = 0;
  virtual vec3 base_spin() = 0;
  virtual vec3 body_position(int b) = 0;
  virtual quat body_rotation(int b) = 0;

  virtual std::vector<double> actuator_forces() const = 0;
  virtual std::vector<mujoco_contact> contacts() const = 0;

  virtual bool energy_on() const = 0;
  virtual mujoco_energy energy() = 0;
  virtual mujoco_books books() = 0;

  virtual mujoco_snapshot save() const = 0;
  virtual void restore(const mujoco_snapshot &s) = 0;
};

// Builds the robot: options' model, with `grid` (null: keep the model's floor) in place of
// options.floor, started over `ground`. options.terrain is not read.
std::unique_ptr<mujoco_engine> make_mujoco_engine(const mujoco_robot_options &options,
                                                  const mujoco_grid *grid,
                                                  const mujoco_ground &ground);
std::string mujoco_version();

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_MUJOCO_ENGINE_H__

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::mujoco_robot (physics/mujoco_robot.h): the Unitree Go2 from cvcpkg's
// mujoco-menagerie, on cvc::height_field terrain. Without CVC_WITH_MUJOCO only the first test runs
// (the constructor refuses); the rest skip.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/core/exception.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/mujoco_robot.h>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::physics;

namespace {

const double kPi = std::acos(-1.0);

std::string go2_scene() { return std::string(CVC_MUJOCO_MENAGERIE_DIR) + "/unitree_go2/scene.xml"; }

#define NEED_MUJOCO()                                                                              \
  do {                                                                                             \
    if (!mujoco_robot::available())                                                                \
      GTEST_SKIP() << "built without MuJoCo (CVC_WITH_MUJOCO)";                                    \
  } while (0)

// A plane rising along +x at slope_deg, centred on the origin.
std::shared_ptr<cvc::height_field> plane(double slope_deg, double half = 3.0, double cell = 0.05) {
  const int n = static_cast<int>(std::lround(2.0 * half / cell)) + 1;
  const double rise = std::tan(slope_deg * kPi / 180.0);
  std::vector<float> h(static_cast<std::size_t>(n) * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h[static_cast<std::size_t>(r) * n + c] = static_cast<float>(rise * (-half + c * cell));
  return std::make_shared<cvc::height_field>(n, n, -half, -half, cell, cell, std::move(h));
}

mujoco_robot_options go2_on(std::shared_ptr<const cvc::height_field> field) {
  mujoco_robot_options o;
  o.model = go2_scene();
  o.terrain = std::move(field);
  return o;
}

// Bitwise equality of two double vectors.
bool same_bits(const std::vector<double> &a, const std::vector<double> &b) {
  return a.size() == b.size() &&
         (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0);
}

// A scripted run: the servos swing the legs (a squat at 1.5 Hz), a push on the base from 0.4 s
// to 0.5 s, and `readout` reads the robot every step. Returns qpos and qvel at the end.
std::vector<double> scripted(mujoco_robot &r, int steps, bool readout) {
  const std::vector<double> stand = r.commands();
  const int base = r.body("base");
  for (int k = 0; k < steps; ++k) {
    const double t = r.time();
    std::vector<double> u = stand;
    for (int i = 0; i < r.actuators(); ++i)
      if (i % 3 != 0) // thighs and calves
        u[static_cast<std::size_t>(i)] +=
            0.25 * std::sin(2.0 * kPi * 1.5 * t) * (i % 3 == 1 ? 1 : -2);
    r.command(u);
    if (k == 200)
      r.apply_wrench(base, {0.0, 40.0, 0.0});
    if (k == 250)
      r.clear_wrenches();
    r.step();
    if (readout) {
      (void)r.energy();
      (void)r.books();
      (void)r.body_position(base);
      (void)r.contacts();
    }
  }
  std::vector<double> out = r.qpos();
  const std::vector<double> v = r.qvel();
  out.insert(out.end(), v.begin(), v.end());
  return out;
}

} // namespace

TEST(MujocoRobot, AvailabilityMatchesTheBuild) {
  mujoco_robot_options o = go2_on(plane(0.0));
  if (!mujoco_robot::available()) {
    EXPECT_EQ(mujoco_robot::version(), "");
    EXPECT_THROW(mujoco_robot r(o), cvc::unsupported_exception);
    return;
  }
  EXPECT_EQ(mujoco_robot::version().rfind("MuJoCo 4.", 0), 0u) << mujoco_robot::version();
}

TEST(MujocoRobot, LoadsTheGo2OnAHeightField) {
  NEED_MUJOCO();
  mujoco_robot r(go2_on(plane(0.0)));
  EXPECT_EQ(r.nq(), 19); // a free base (7) and 12 hinges
  EXPECT_EQ(r.nv(), 18);
  ASSERT_EQ(r.actuators(), 12);
  EXPECT_GE(r.terrain_geom(), 0);
  EXPECT_EQ(r.geom("floor"), -1); // replaced
  EXPECT_EQ(r.geom_body(r.terrain_geom()), 0);
  EXPECT_NEAR(r.mass(), 6.921 + 4 * (0.678 + 1.152 + 0.241352), 1e-9);
  EXPECT_DOUBLE_EQ(r.timestep(), 0.002);
  const char *names[] = {"FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
                         "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"};
  const std::vector<double> u = r.commands();
  for (int i = 0; i < 12; ++i) {
    EXPECT_EQ(r.actuator_name(i), names[i]);
    EXPECT_EQ(r.actuator(names[i]), i);
    EXPECT_EQ(r.mode(i), actuator_mode::position);
    const double limit = i % 3 == 2 ? 45.43 : 23.7; // the knees are stronger
    EXPECT_DOUBLE_EQ(r.force_min(i), -limit);
    EXPECT_DOUBLE_EQ(r.force_max(i), limit);
    // In position mode the servos start on the keyframe's joint angles.
    const double home = i % 3 == 0 ? 0.0 : (i % 3 == 1 ? 0.9 : -1.8);
    EXPECT_DOUBLE_EQ(u[static_cast<std::size_t>(i)], home);
    EXPECT_DOUBLE_EQ(r.qpos()[static_cast<std::size_t>(r.actuator_qpos(i))], home);
  }
  EXPECT_TRUE(r.contacts().empty()); // nothing has stepped
  EXPECT_DOUBLE_EQ(r.time(), 0.0);
  EXPECT_NEAR(r.base_up_z(), 1.0, 1e-12);
}

// MuJoCo's height field is the cvc::height_field's surface: a small ball placed over the middle
// of either triangle of bumpy cells touches the terrain at the distance the height field's own
// triangle gives. The other diagonal would be centimetres off on these cells.
TEST(MujocoRobot, TerrainIsTheHeightFieldsTriangles) {
  NEED_MUJOCO();
  const std::string xml = ::testing::TempDir() + "/cvc_mujoco_ball.xml";
  {
    std::ofstream f(xml);
    f << "<mujoco><option timestep='0.002'/><worldbody>"
         "<geom name='floor' type='plane' size='0 0 0.05' margin='0.01'/>"
         "<body name='ball' pos='0 0 0.01'><freejoint/>"
         "<geom name='ball' type='sphere' size='0.01' mass='0.1' margin='0.01'/></body>"
         "</worldbody><keyframe><key name='home' qpos='0 0 0.01 1 0 0 0'/></keyframe></mujoco>";
  }
  const int n = 9;
  const double cell = 0.5;
  std::vector<float> h(n * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h[static_cast<std::size_t>(r * n + c)] = static_cast<float>(
          0.15 * std::sin(1.3 * c + 0.4) * std::cos(0.9 * r) + 0.05 * ((r * 7 + c * 3) % 5));
  auto field = std::make_shared<cvc::height_field>(n, n, -2.0, -2.0, cell, cell, std::move(h));
  int checked = 0;
  for (int r = 1; r < n - 2; r += 2)
    for (int c = 1; c < n - 2; c += 2)
      for (int lower = 0; lower < 2; ++lower) {
        // The centroid of the cell's lower-right (u > v) or upper-left triangle.
        const double u = lower ? 2.0 / 3.0 : 1.0 / 3.0, v = lower ? 1.0 / 3.0 : 2.0 / 3.0;
        mujoco_robot_options o;
        o.model = xml;
        o.terrain = field;
        o.x = -2.0 + (c + u) * cell;
        o.y = -2.0 + (r + v) * cell;
        mujoco_robot ball(o);
        const vec3 p = ball.base_position();
        ball.step();
        double dist = 1e9;
        for (const mujoco_contact &k : ball.contacts())
          if (k.geom[0] == ball.terrain_geom() || k.geom[1] == ball.terrain_geom())
            dist = std::min(dist, k.distance);
        const cvc::vector_t nn = field->normal(p.x, p.y);
        const vec3 nrm{nn[0], nn[1], nn[2]};
        const double ours = dot(p - vec3{o.x, o.y, field->height(o.x, o.y)}, nrm) - 0.01;
        EXPECT_NEAR(dist, ours, 1e-5) << "cell (" << r << ", " << c << ") lower " << lower;
        ++checked;
      }
  EXPECT_EQ(checked, 18);
  std::remove(xml.c_str());
}

TEST(MujocoRobot, Go2StandsOnFlatGround) {
  NEED_MUJOCO();
  mujoco_trial_options t;
  t.model = go2_scene();
  t.duration = 3.0;
  const mujoco_trial_result res = run_mujoco_trial(t);
  std::printf("flat: height %.4f -> %.4f, up_z %.6f, speed %.2e, moved %.2e m, residual %.3e J\n",
              res.settled_height, res.end_height, res.end_up_z, res.end_speed, res.end_displacement,
              res.books.residual());
  EXPECT_FALSE(res.fell);
  EXPECT_TRUE(res.recovered);
  EXPECT_EQ(res.body_contact_steps, 0);
  EXPECT_GT(res.end_height, 0.22);
  EXPECT_LT(res.end_height, 0.32);
  EXPECT_GT(res.end_up_z, 0.999);
  EXPECT_LT(res.end_speed, 1e-3);
  EXPECT_LT(res.end_displacement, 5e-3);

  // At rest the terrain carries the robot's weight, through the feet only.
  mujoco_robot r(go2_on(plane(0.0)));
  r.step(1000);
  double up = 0.0;
  for (const mujoco_contact &c : r.contacts()) {
    const bool t0 = c.geom[0] == r.terrain_geom();
    ASSERT_TRUE(t0 || c.geom[1] == r.terrain_geom());
    const std::string foot = r.geom_name(t0 ? c.geom[1] : c.geom[0]);
    EXPECT_TRUE(foot == "FL" || foot == "FR" || foot == "RL" || foot == "RR") << foot;
    up += t0 ? c.force.z : -c.force.z; // the terrain's push on the robot
  }
  EXPECT_NEAR(up, r.mass() * 9.81, 0.01 * r.mass() * 9.81);
}

// Up and down a 10 degree slope and across it: the servos hold the stance and the feet hold the
// ground.
TEST(MujocoRobot, Go2StandsOnSlopes) {
  NEED_MUJOCO();
  struct pose {
    double slope, yaw;
  };
  for (const pose p : {pose{10.0, 0.0}, pose{-10.0, 0.0}, pose{10.0, kPi / 2}}) {
    mujoco_trial_options t;
    t.model = go2_scene();
    t.slope_deg = p.slope;
    t.yaw = p.yaw;
    t.settle = 1.0;
    t.duration = 3.0;
    const mujoco_trial_result res = run_mujoco_trial(t);
    std::printf("slope %+.0f yaw %.2f: height %.4f -> %.4f, up_z %.6f, speed %.2e, moved %.2e m\n",
                p.slope, p.yaw, res.settled_height, res.end_height, res.end_up_z, res.end_speed,
                res.end_displacement);
    EXPECT_FALSE(res.fell) << p.slope << " " << p.yaw;
    EXPECT_TRUE(res.recovered) << p.slope << " " << p.yaw;
    // The base keeps the slope's tilt (the servos give a little under the load).
    EXPECT_NEAR(res.end_up_z, std::cos(p.slope * kPi / 180.0), 0.015);
    EXPECT_LT(res.end_speed, 1e-3);
    EXPECT_LT(res.end_displacement, 5e-3); // over the last two seconds
  }
}

// A push perturbs the stance measurably, and the robot comes back; a hard one knocks it over. The
// pushes are sideways on the base for 0.1 s. Measured on Linux x86_64 (MuJoCo 4.0.0): it recovers
// from 150 N (15 N s) and falls at 200 N, so 60 N and 300 N sit well inside either side.
TEST(MujocoRobot, Go2RecoversFromAPushOrFalls) {
  NEED_MUJOCO();
  mujoco_trial_options t;
  t.model = go2_scene();
  t.settle = 1.0;
  t.push_duration = 0.1;
  t.duration = 4.0;
  auto pushed = [&](double f) {
    t.push = {0.0, f, 0.0};
    const mujoco_trial_result res = run_mujoco_trial(t);
    std::printf("push %.0f N: moved up to %.4f m (%.4f at the end), min up_z %.4f, min height "
                "%.4f, fell %d, recovered %d\n",
                f, res.max_displacement, res.end_displacement, res.min_up_z, res.min_height,
                res.fell, res.recovered);
    return res;
  };

  const mujoco_trial_result soft = pushed(60.0);
  EXPECT_FALSE(soft.fell);
  EXPECT_TRUE(soft.recovered);
  EXPECT_EQ(soft.body_contact_steps, 0);
  EXPECT_GT(soft.max_displacement, 0.01); // it moved
  EXPECT_LT(soft.min_up_z, 0.9995);       // and rocked
  EXPECT_LT(soft.end_displacement, soft.max_displacement);

  const mujoco_trial_result hard = pushed(300.0);
  EXPECT_TRUE(hard.fell);
  EXPECT_FALSE(hard.recovered);
  EXPECT_LT(hard.min_up_z, 0.5);
  EXPECT_GT(hard.body_contact_steps, 0);
  EXPECT_GT(hard.max_displacement, 0.2);
}

// With energy on, MuJoCo's energy change matches the work the books add up: the actuators', the
// push's, and the contacts' and joints' losses. What is left is the integrator's error, which
// halves with the step.
TEST(MujocoRobot, EnergyChangeMatchesTheWorkDone) {
  NEED_MUJOCO();
  double residual[2] = {0.0, 0.0};
  int k = 0;
  for (double dt : {0.002, 0.001}) {
    mujoco_trial_options t;
    t.model = go2_scene();
    t.timestep = dt;
    t.settle = 1.0;
    t.push = {0.0, 60.0, 0.0};
    t.duration = 3.0;
    const mujoco_trial_result res = run_mujoco_trial(t);
    const mujoco_books &b = res.books;
    const double gross = std::fabs(b.actuators) + std::fabs(b.external) + std::fabs(b.contacts) +
                         std::fabs(b.joint_friction) + std::fabs(b.joint_limits) +
                         std::fabs(b.joint_damping) + std::fabs(b.other);
    std::printf("dt %.4f: dE %.5f = actuators %.5f + push %.5f + contacts %.5f + joint friction "
                "%.5f + limits %.5f + damping %.5f + other %.5f, residual %.3e J (%.1e of %.3f J "
                "gross)\n",
                dt, b.energy_now - b.energy_start, b.actuators, b.external, b.contacts,
                b.joint_friction, b.joint_limits, b.joint_damping, b.other, b.residual(),
                std::fabs(b.residual()) / gross, gross);
    // The stated tolerance: the books close to 2e-5 of the gross work (measured: 5e-6 at 2 ms).
    EXPECT_LT(std::fabs(b.residual()), 2e-5 * gross) << dt;
    EXPECT_GT(gross, 1.0);
    EXPECT_GT(b.external, 0.0); // the push moved the robot its way
    EXPECT_LT(b.contacts, 0.0); // and the rest dissipated
    EXPECT_LT(b.joint_friction, 0.0);
    EXPECT_LT(b.joint_damping, 0.0);
    EXPECT_EQ(b.joint_limits, 0.0); // no joint reached its stop
    EXPECT_EQ(b.other, 0.0);        // the Go2 has no other passive force
    residual[k++] = b.residual();
  }
  // The residual is the Euler step's own error: first order, so it halves with the step.
  const double ratio = residual[1] / residual[0];
  EXPECT_GT(ratio, 0.3);
  EXPECT_LT(ratio, 0.7);
}

// MuJoCo's determinism within one binary: two robots built alike and given the same commands step
// to the same bits; reading one every step changes nothing; a restored snapshot replays exactly.
TEST(MujocoRobot, TwoRunsInOneBinaryAreIdentical) {
  NEED_MUJOCO();
  mujoco_robot a(go2_on(plane(5.0)));
  mujoco_robot b(go2_on(plane(5.0)));
  const std::vector<double> qa = scripted(a, 1500, false);
  const std::vector<double> qb = scripted(b, 1500, true);
  EXPECT_TRUE(same_bits(qa, qb));
  EXPECT_EQ(a.books().residual(), b.books().residual());

  mujoco_robot c(go2_on(plane(5.0)));
  scripted(c, 300, false);
  const mujoco_snapshot s = c.save();
  const std::vector<double> first = scripted(c, 700, false);
  const mujoco_books first_books = c.books();
  c.restore(s);
  EXPECT_TRUE(c.contacts().empty());
  const std::vector<double> again = scripted(c, 700, false);
  EXPECT_TRUE(same_bits(first, again));
  EXPECT_EQ(first_books.work(), c.books().work());
}

TEST(MujocoRobot, TorqueModeAndForceLimits) {
  NEED_MUJOCO();
  mujoco_robot r(go2_on(plane(0.0)));
  r.set_mode(actuator_mode::torque);
  for (int i = 0; i < r.actuators(); ++i) {
    EXPECT_EQ(r.mode(i), actuator_mode::torque);
    EXPECT_EQ(r.commands()[static_cast<std::size_t>(i)], 0.0);
  }
  std::vector<double> u(12, 100.0); // past every limit
  r.command(u);
  r.step();
  const std::vector<double> f = r.actuator_forces();
  ASSERT_EQ(f.size(), 12u);
  for (int i = 0; i < 12; ++i)
    EXPECT_DOUBLE_EQ(f[static_cast<std::size_t>(i)], r.force_max(i));

  // Back in position mode with a far target, the servo saturates at the same limit.
  r.set_mode(1, actuator_mode::position);
  r.set_mode(0, actuator_mode::torque);
  r.command(0, 0.0);
  r.command(1, r.qpos()[static_cast<std::size_t>(r.actuator_qpos(1))] + 2.0);
  r.step();
  EXPECT_DOUBLE_EQ(r.actuator_forces()[1], r.force_max(1));
  EXPECT_DOUBLE_EQ(r.actuator_forces()[0], 0.0);
}

TEST(MujocoRobot, RefusesWhatItCannotBuild) {
  NEED_MUJOCO();
  {
    mujoco_robot_options o = go2_on(plane(0.0));
    o.model = "/nonexistent/scene.xml";
    EXPECT_THROW(mujoco_robot r(o), cvc::physics_error);
  }
  {
    mujoco_robot_options o = go2_on(plane(0.0));
    o.floor = "no_such_geom";
    EXPECT_THROW(mujoco_robot r(o), cvc::physics_error);
  }
  {
    mujoco_robot_options o = go2_on(plane(0.0));
    o.keyframe = "no_such_key";
    EXPECT_THROW(mujoco_robot r(o), cvc::physics_error);
  }
  {
    mujoco_robot_options o = go2_on(plane(0.0));
    o.x = 50.0; // off the 6 m field
    EXPECT_THROW(mujoco_robot r(o), cvc::physics_error);
  }
  {
    auto f = std::make_shared<cvc::height_field>(*plane(0.0));
    f->set_height(3, 3, std::nanf(""));
    EXPECT_THROW(mujoco_robot r(go2_on(f)), cvc::physics_error);
  }
  mujoco_robot r(go2_on(plane(0.0)));
  EXPECT_THROW(r.command(12, 0.0), cvc::physics_error);
  EXPECT_THROW(r.command(std::vector<double>(11, 0.0)), cvc::physics_error);
  EXPECT_THROW(r.apply_wrench(0, {1.0, 0.0, 0.0}), cvc::physics_error);
  mujoco_robot_options off = go2_on(plane(0.0));
  off.energy = false;
  mujoco_robot quiet(off);
  EXPECT_THROW(quiet.energy(), std::logic_error);
}

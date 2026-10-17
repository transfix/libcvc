/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::pose_tracker and world::states: the two samples a view draws
// between, the fraction it draws at, and the batch read underneath.

#include <atomic>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/pose_tracker.h>
#include <cvc/physics/world.h>
#include <gtest/gtest.h>
#include <thread>
#include <vector>

using namespace cvc::physics;

constexpr double kPi = 3.14159265358979323846;

namespace {

world_config null_config(double fixed_dt = 0.01) {
  world_config c;
  c.backend = backend::null;
  c.fixed_dt = fixed_dt;
  return c;
}

cvc::world_clock clock_at(double fixed_dt = 0.01) {
  cvc::world_clock::config cc;
  cc.fixed_dt = fixed_dt;
  return cvc::world_clock(cc);
}

// A kinematic box at x0 moving at v m/s along x: the null backend integrates it
// exactly, so its pose at every tick is known.
body_id mover(world &w, double x0, double v) {
  body_desc d;
  d.shape = box{{0.1, 0.1, 0.1}};
  d.motion = motion_type::kinematic;
  d.position = {x0, 0.0, 0.0};
  d.linear_velocity = {v, 0.0, 0.0};
  return w.add_body(d);
}

quat about_z(double radians) { return {std::cos(radians / 2), 0.0, 0.0, std::sin(radians / 2)}; }

double yaw(const quat &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

} // namespace

TEST(PhysicsPose, InterpolateIsExactAtTheEnds) {
  const pose a{{1, 2, 3}, about_z(0.2)};
  const pose b{{5, 6, 7}, about_z(0.9)};
  const pose p0 = interpolate(a, b, 0.0), p1 = interpolate(a, b, 1.0);
  EXPECT_EQ(p0.position.x, 1.0);
  EXPECT_EQ(p0.rotation.z, a.rotation.z);
  EXPECT_EQ(p1.position.z, 7.0);
  EXPECT_EQ(p1.rotation.w, b.rotation.w);
  // Out of range and NaN clamp to the ends rather than extrapolating.
  EXPECT_EQ(interpolate(a, b, -0.5).position.x, 1.0);
  EXPECT_EQ(interpolate(a, b, 1.5).position.x, 5.0);
  EXPECT_EQ(interpolate(a, b, std::nan("")).position.x, 1.0);
}

TEST(PhysicsPose, InterpolateTakesTheShorterArc) {
  const pose a{{0, 0, 0}, about_z(0.0)};
  const pose b{{2, 0, 0}, about_z(1.0)};
  const pose mid = interpolate(a, b, 0.5);
  EXPECT_DOUBLE_EQ(mid.position.x, 1.0);
  EXPECT_NEAR(yaw(mid.rotation), 0.5, 1e-12);
  // -q is the same rotation as q; the blend must not swing the long way round.
  pose b_neg = b;
  b_neg.rotation = {-b.rotation.w, -b.rotation.x, -b.rotation.y, -b.rotation.z};
  EXPECT_NEAR(yaw(interpolate(a, b_neg, 0.5).rotation), 0.5, 1e-12);
  const quat &r = interpolate(a, b, 0.3).rotation;
  EXPECT_NEAR(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z, 1.0, 1e-12);
}

TEST(PhysicsPose, PoseMatrixIsRowMajorRotationThenTranslation) {
  const pose p{{1, 2, 3}, about_z(kPi / 2)};
  double m[16];
  pose_matrix(p, m);
  // The x axis turns to y; the translation sits in the last column.
  const double x[4] = {1, 0, 0, 1};
  double y[4] = {0, 0, 0, 0};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      y[r] += m[r * 4 + c] * x[c];
  EXPECT_NEAR(y[0], 1.0, 1e-12);
  EXPECT_NEAR(y[1], 3.0, 1e-12);
  EXPECT_NEAR(y[2], 3.0, 1e-12);
  EXPECT_EQ(m[12], 0.0);
  EXPECT_EQ(m[15], 1.0);
}

TEST(PhysicsPose, StatesReadsABatchAtOneTick) {
  world w(null_config());
  const body_id a = mover(w, 0.0, 1.0);
  const body_id b = mover(w, 5.0, -1.0);
  const body_id gone = mover(w, 9.0, 0.0);
  w.remove_body(gone);
  w.step(3);
  const state_batch s = w.states({a, gone, b, body_id{}});
  EXPECT_EQ(s.tick, 3u);
  ASSERT_EQ(s.states.size(), 4u);
  ASSERT_TRUE(s.states[0].has_value());
  EXPECT_NEAR(s.states[0]->position.x, 0.03, 1e-12);
  EXPECT_FALSE(s.states[1].has_value()); // removed: empty, not an exception
  ASSERT_TRUE(s.states[2].has_value());
  EXPECT_NEAR(s.states[2]->position.x, 4.97, 1e-12);
  EXPECT_FALSE(s.states[3].has_value()); // never valid
}

TEST(PhysicsPose, FollowKeepsTheLastTwoTicks) {
  world w(null_config());
  auto clock = clock_at();
  const body_id a = mover(w, 0.0, 1.0);
  pose_tracker t(w);
  t.track(a);
  EXPECT_EQ(t.follow(clock), 0u); // anchors and captures
  ASSERT_TRUE(t.sample_one(a, 0.5).has_value());
  EXPECT_EQ(t.sample_one(a, 0.5)->position.x, 0.0); // one sample: drawn as it is

  clock.advance_ticks(5);
  EXPECT_EQ(t.follow(clock), 5u);
  EXPECT_EQ(t.tick(), 5u);
  // Between tick 4 (x = 0.04) and tick 5 (x = 0.05), not between 0 and 5.
  EXPECT_NEAR(t.sample_one(a, 0.0)->position.x, 0.04, 1e-12);
  EXPECT_NEAR(t.sample_one(a, 0.5)->position.x, 0.045, 1e-12);
  EXPECT_NEAR(t.sample_one(a, 1.0)->position.x, 0.05, 1e-12);

  clock.advance_ticks(1);
  EXPECT_EQ(t.follow(clock), 1u);
  EXPECT_NEAR(t.sample_one(a, 0.0)->position.x, 0.05, 1e-12);
  EXPECT_EQ(t.follow(clock), 0u); // nothing due: the samples stay
  EXPECT_NEAR(t.sample_one(a, 1.0)->position.x, 0.06, 1e-12);
}

TEST(PhysicsPose, AlphaCountsQuantaForASlowerWorld) {
  world w(null_config(0.04)); // one step every four clock quanta
  auto clock = clock_at(0.01);
  pose_tracker t(w);
  t.follow(clock);
  clock.advance_ticks(8);
  EXPECT_EQ(t.follow(clock), 2u);
  EXPECT_DOUBLE_EQ(t.alpha(clock, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(t.alpha(clock, 0.5), 0.125);
  clock.advance_ticks(2); // halfway to the next step
  EXPECT_EQ(t.follow(clock), 0u);
  EXPECT_DOUBLE_EQ(t.alpha(clock, 0.0), 0.5);
  EXPECT_LT(t.alpha(clock, 0.999999), 1.0);

  world odd(null_config(0.015)); // not a whole number of quanta
  pose_tracker t2(odd);
  EXPECT_THROW(t2.follow(clock), cvc::unsupported_exception);
  EXPECT_THROW(t2.alpha(clock, 0.0), cvc::unsupported_exception);
}

TEST(PhysicsPose, SnapAfterATeleportDrawsTheNewPlace) {
  world w(null_config());
  const body_id a = mover(w, 0.0, 1.0);
  pose_tracker t(w);
  t.track(a);
  w.step();
  t.capture();
  w.step();
  t.capture();
  EXPECT_NEAR(t.sample_one(a, 0.5)->position.x, 0.015, 1e-12);

  body_state s = w.state(a);
  s.position = {100.0, 0.0, 0.0};
  w.set_state(a, s);
  t.snap(a); // before the step that follows, as the header says
  w.step();
  t.capture();
  EXPECT_NEAR(t.sample_one(a, 0.5)->position.x, 100.01, 1e-12) << "swept from the old place";
  w.step();
  t.capture();
  EXPECT_NEAR(t.sample_one(a, 0.5)->position.x, 100.015, 1e-12) << "interpolates again";
}

TEST(PhysicsPose, RemovedAndSkippedBodies) {
  world w(null_config());
  const body_id a = mover(w, 0.0, 1.0);
  const body_id b = mover(w, 1.0, 1.0);
  pose_tracker t(w);
  t.track(a);
  t.track(b);
  t.track(a); // twice is once
  EXPECT_EQ(t.size(), 2u);
  EXPECT_THROW(t.track(body_id{}), cvc::physics_error);
  t.capture();
  w.remove_body(a);
  w.step();
  t.capture();
  EXPECT_EQ(t.size(), 1u);
  EXPECT_FALSE(t.tracking(a));
  const auto all = t.sample_all(0.5);
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all[0].id, b);

  // A gap of more than one step between samples draws the newest, never a lerp
  // across the gap.
  w.step(3);
  t.capture();
  EXPECT_NEAR(t.sample_one(b, 0.0)->position.x, 1.04, 1e-12);
  // A second capture at the same tick keeps the pair; a world restored to an
  // earlier tick starts again from one sample.
  const auto saved = w.save();
  w.step();
  t.capture();
  t.capture();
  EXPECT_NEAR(t.sample_one(b, 0.0)->position.x, 1.04, 1e-12);
  w.restore(saved);
  t.capture();
  EXPECT_NEAR(t.sample_one(b, 0.5)->position.x, 1.04, 1e-12);
  t.untrack(b);
  EXPECT_FALSE(t.sample_one(b, 0.5).has_value());
}

TEST(PhysicsPose, SamplersNeverSeeHalfACapture) {
  world w(null_config());
  auto clock = clock_at();
  std::vector<body_id> ids;
  for (int i = 0; i < 16; ++i)
    ids.push_back(mover(w, 0.0, 1.0)); // all in lockstep
  pose_tracker t(w);
  for (const body_id id : ids)
    t.track(id);
  t.follow(clock);
  std::atomic<bool> done{false};
  std::atomic<int> torn{0}, samples{0};
  std::thread viewer([&] {
    std::vector<pose_tracker::sample> out;
    while (!done.load()) {
      t.sample_all(0.5, out);
      for (const auto &s : out)
        if (s.pose.position.x != out.front().pose.position.x)
          torn.fetch_add(1);
      samples.fetch_add(1);
    }
  });
  for (int f = 0; f < 2000; ++f) {
    clock.advance_ticks(1 + f % 3);
    t.follow(clock);
  }
  done.store(true);
  viewer.join();
  EXPECT_EQ(torn.load(), 0);
  EXPECT_GT(samples.load(), 0);
  EXPECT_EQ(w.tick(), t.tick());
}

TEST(PhysicsPose, JoltBoxFallsBetweenItsLastTwoStates) {
  if (!backend_available(backend::jolt))
    GTEST_SKIP() << "this libcvc was built without Jolt";
  world_config c;
  c.backend = backend::jolt;
  world w(c);
  auto clock = clock_at();
  body_desc d;
  d.shape = box{{0.2, 0.2, 0.2}};
  d.position = {0.0, 0.0, 10.0};
  const body_id a = w.add_body(d);
  pose_tracker t(w);
  t.track(a);
  t.follow(clock);
  clock.advance_ticks(10);
  t.follow(clock);
  const double z0 = t.sample_one(a, 0.0)->position.z;
  const double z1 = t.sample_one(a, 1.0)->position.z;
  EXPECT_NEAR(z1, w.state(a).position.z, 1e-12) << "the newest sample is the world's state";
  EXPECT_GT(z0, z1) << "falling";
  const double zm = t.sample_one(a, 0.5)->position.z;
  EXPECT_NEAR(zm, 0.5 * (z0 + z1), 1e-12);
}

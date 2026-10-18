/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// ackermann_controller's members other than its constructor and destructor, built with RTTI
// (jolt_vehicle_controller.cpp says why they are not there): the per-wheel Ackermann steering
// and the energy ledger's measurements. Its arithmetic is the controller's, with the target's
// floating-point flags; nothing here needs a Jolt type's typeinfo (no typeid, dynamic_cast or
// exception of one), and the vtable stays in jolt_vehicle_controller.cpp.
//
// measured_post_collide and measured_friction are Jolt 5.6.0's
// WheeledVehicleController::PostCollide and SolveLongitudinalAndLateralConstraints (MIT
// licensed, Copyright 2021 Jorrit Rouwe), with the energy ledger's measurements added between
// their steps and nothing else changed.

#include "jolt_vehicle_controller.h"

#include <Jolt/Math/DynMatrix.h>
#include <Jolt/Math/GaussianElimination.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <cfloat>
#include <cmath>

namespace cvc {
namespace physics {
namespace detail {

namespace {

// The chassis's kinetic energy, linear and rotational, in double from Jolt's float state.
double chassis_kinetic(const JPH::Body &b, double *rotational = nullptr) {
  const JPH::MotionProperties *mp = b.GetMotionProperties();
  const JPH::Vec3 v = b.GetLinearVelocity();
  const double inv_m = mp->GetInverseMass();
  double ke = 0.0;
  if (inv_m > 0.0)
    ke = 0.5 *
         (double(v.GetX()) * v.GetX() + double(v.GetY()) * v.GetY() + double(v.GetZ()) * v.GetZ()) /
         inv_m;
  // The angular velocity in the principal axes, where the inertia is diagonal.
  const JPH::Vec3 wl =
      (b.GetRotation() * mp->GetInertiaRotation()).Conjugated() * b.GetAngularVelocity();
  const JPH::Vec3 inv_i = mp->GetInverseInertiaDiagonal();
  double rot = 0.0;
  for (int k = 0; k < 3; ++k)
    if (inv_i[k] > 0.0f)
      rot += 0.5 * double(wl[k]) * wl[k] / inv_i[k];
  if (rotational)
    *rotational = rot;
  return ke + rot;
}

double wheel_kinetic(const JPH::Wheel &w) {
  const double om = w.GetAngularVelocity();
  return 0.5 * static_cast<const JPH::WheelSettingsWV *>(w.GetSettings())->mInertia * om * om;
}

double dot3(const double *a, const double *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

} // namespace

double ackermann_controller::friction_torque(double rpm) const {
  const std::vector<std::pair<double, double>> &c = engine_friction;
  if (c.empty())
    return 0.0;
  if (rpm <= c.front().first)
    return c.front().second;
  for (std::size_t i = 1; i < c.size(); ++i)
    if (rpm <= c[i].first)
      return c[i - 1].second + (c[i].second - c[i - 1].second) * (rpm - c[i - 1].first) /
                                   (c[i].first - c[i - 1].first);
  return c.back().second;
}

void ackermann_controller::PreCollide(float dt, JPH::PhysicsSystem &ps) {
  JPH::WheeledVehicleController::PreCollide(dt, ps);
  if (!steer)
    return;
  JPH::Wheels &wheels = mConstraint.GetWheels();
  for (size_t i = 0; i < wheels.size() && i < steer->size(); ++i)
    wheels[i]->SetSteerAngle((*steer)[i]);
}

void ackermann_controller::PostCollide(float dt, JPH::PhysicsSystem &ps) {
  if (!tally || tally->omega.size() != mConstraint.GetWheels().size() ||
      tally->next_compression.size() != mConstraint.GetWheels().size()) {
    JPH::WheeledVehicleController::PostCollide(dt, ps);
    return;
  }
  sample(dt);
  measured_post_collide(dt);
  tally->lateral_warm_start = true;
}

bool ackermann_controller::SolveLongitudinalAndLateralConstraints(float dt) {
  if (!tally || tally->omega.size() != mConstraint.GetWheels().size())
    return JPH::WheeledVehicleController::SolveLongitudinalAndLateralConstraints(dt);
  return measured_friction(dt);
}

// The vehicle at the start of this sub-step's PostCollide: the wheels' collision tests have
// just run on the current pose, so the suspension lengths match it. Closes the interval since the
// last sample.
void ackermann_controller::sample(float dt) {
  energy_tally &t = *tally;
  const JPH::Body &b = *mConstraint.GetVehicleBody();
  const JPH::MotionProperties *mp = b.GetMotionProperties();
  const JPH::Wheels &wheels = mConstraint.GetWheels();

  double rot = 0.0;
  double kinetic = chassis_kinetic(b, &rot);
  for (const JPH::Wheel *w : wheels)
    kinetic += wheel_kinetic(*w);
  {
    const double we = mEngine.GetAngularVelocity();
    kinetic += 0.5 * double(mEngine.mInertia) * we * we;
  }
  const double inv_m = mp->GetInverseMass();
  const JPH::RVec3 c = b.GetCenterOfMassPosition();
  const double com[3] = {double(c.GetX()) + t.origin[0], double(c.GetY()) + t.origin[1],
                         double(c.GetZ()) + t.origin[2]};
  const double potential = inv_m > 0.0 ? -dot3(t.gravity, com) / inv_m : 0.0;

  // The suspension springs and the anti-roll bars, as VehicleConstraint::SetupVelocityConstraint
  // sets them: a spring of stiffness m_eff w^2 / cos(alpha) on the compression past the rest
  // length, and a damper of 2 m_eff zeta w / cos(alpha).
  const JPH::RMat44 transform = b.GetWorldTransform();
  double spring = 0.0;
  std::size_t i = 0;
  for (const JPH::Wheel *w : wheels) {
    const JPH::WheelSettings *s = w->GetSettings();
    t.next_compression[i] = -1.0;
    t.next_damping[i] = 0.0;
    if (w->HasContact() && s->mSuspensionMaxLength > s->mSuspensionMinLength) {
      const JPH::Vec3 fp = s->mPosition + 0.5f *
                                              (s->mSuspensionMinLength + s->mSuspensionMaxLength) *
                                              s->mSuspensionDirection;
      const JPH::Vec3 fp_x_neg_up = fp.Cross(-mConstraint.GetLocalUp());
      const double m_eff =
          1.0 /
          (inv_m + fp_x_neg_up.Dot(mp->GetLocalSpaceInverseInertia().Multiply3x3(fp_x_neg_up)));
      const double omega = 2.0 * JPH::JPH_PI * s->mSuspensionSpring.mFrequency;
      const JPH::Vec3 dir = transform.Multiply3x3(s->mSuspensionDirection);
      const double cos_a = std::max(0.1, double(dir.Dot(-w->GetContactNormal())));
      const double k = m_eff * omega * omega / cos_a;
      const double x = std::max(0.0, double(s->mSuspensionMaxLength) + s->mSuspensionPreloadLength -
                                         w->GetSuspensionLength());
      spring += 0.5 * k * x * x;
      t.next_compression[i] = x;
      t.next_damping[i] = 2.0 * m_eff * s->mSuspensionSpring.mDamping * omega / cos_a;
    }
    ++i;
  }
  for (std::size_t a = 0; a < anti_roll.size() && 2 * a + 1 < wheels.size(); ++a) {
    const JPH::Wheel *l = wheels[2 * a], *r = wheels[2 * a + 1];
    if (anti_roll[a] > 0.0 && l->HasContact() && r->HasContact()) {
      const double d = double(r->GetSuspensionLength()) - l->GetSuspensionLength();
      spring += 0.5 * anti_roll[a] * d * d;
    }
  }

  const JPH::Vec3 v = b.GetLinearVelocity(), om = b.GetAngularVelocity();
  const double vel[3] = {v.GetX(), v.GetY(), v.GetZ()};
  const double ang[3] = {om.GetX(), om.GetY(), om.GetZ()};
  if (t.sampled) {
    const double h = t.h;
    t.stored += (kinetic + potential + spring) - (t.kinetic + t.potential + t.spring);
    // The loads' work: each acted through the interval at the centre of mass.
    const double vm[3] = {0.5 * (t.v[0] + vel[0]), 0.5 * (t.v[1] + vel[1]),
                          0.5 * (t.v[2] + vel[2])};
    const double wm[3] = {0.5 * (t.w[0] + ang[0]), 0.5 * (t.w[1] + ang[1]),
                          0.5 * (t.w[2] + ang[2])};
    for (int k = 0; k < 4; ++k)
      t.work[k] += (dot3(t.interval_loads[k], vm) + dot3(t.interval_loads[k] + 3, wm)) * h;
    // The dampers, on the compression rate through the interval, for a wheel touching
    // the ground at both ends of it.
    for (std::size_t j = 0; j < t.compression.size(); ++j)
      if (t.compression[j] >= 0.0 && t.next_compression[j] >= 0.0 && h > 0.0) {
        const double rate = (t.next_compression[j] - t.compression[j]) / h;
        t.suspension += 0.5 * (t.damping[j] + t.next_damping[j]) * rate * rate * h;
      }
    t.distance += std::sqrt(dot3(vm, vm)) * h;
    // The chassis body's own damping, applied as its velocities were integrated:
    // v *= max(0, 1 - c h).
    const double kl = std::max(0.0, 1.0 - double(mp->GetLinearDamping()) * h);
    const double ka = std::max(0.0, 1.0 - double(mp->GetAngularDamping()) * h);
    const double lin_prev = inv_m > 0.0 ? 0.5 * dot3(t.v, t.v) / inv_m : 0.0;
    t.chassis_damping += lin_prev * (1.0 - kl * kl) + t.rot_kinetic * (1.0 - ka * ka);
  }
  for (std::size_t j = 0; j < t.compression.size(); ++j) {
    t.compression[j] = t.next_compression[j];
    t.damping[j] = t.next_damping[j];
  }
  t.kinetic = kinetic;
  t.potential = potential;
  t.spring = spring;
  t.rot_kinetic = rot;
  for (int k = 0; k < 3; ++k) {
    t.v[k] = vel[k];
    t.w[k] = ang[k];
  }
  for (int k = 0; k < 4; ++k)
    for (int j = 0; j < 6; ++j)
      t.interval_loads[k][j] = t.loads[k][j];
  t.h = dt;
  t.sampled = true;
}

void ackermann_controller::measured_post_collide(float inDeltaTime) {
  using namespace JPH;
  energy_tally &t = *tally;

  // Remember old RPM so we can detect if we're increasing or decreasing
  float old_engine_rpm = mEngine.GetCurrentRPM();

  Wheels &wheels = mConstraint.GetWheels();

  // [ledger] the wheels' speeds before their damping
  for (uint wheel_index = 0, num_wheels = (uint)wheels.size(); wheel_index < num_wheels;
       ++wheel_index)
    t.omega[wheel_index] = wheels[wheel_index]->GetAngularVelocity();

  // Update wheel angle, do this before applying torque to the wheels (as friction will slow them
  // down again)
  for (uint wheel_index = 0, num_wheels = (uint)wheels.size(); wheel_index < num_wheels;
       ++wheel_index) {
    WheelWV *w = static_cast<WheelWV *>(wheels[wheel_index]);
    w->Update(wheel_index, inDeltaTime, mConstraint);
  }

  // [ledger] the wheels' damping; then t.omega holds the damped speeds
  for (uint wheel_index = 0, num_wheels = (uint)wheels.size(); wheel_index < num_wheels;
       ++wheel_index) {
    const double inertia =
        static_cast<const WheelSettingsWV *>(wheels[wheel_index]->GetSettings())->mInertia;
    const double w0 = t.omega[wheel_index], w1 = wheels[wheel_index]->GetAngularVelocity();
    t.drivetrain += 0.5 * inertia * (w0 * w0 - w1 * w1);
    t.omega[wheel_index] = wheels[wheel_index]->GetAngularVelocity();
  }

  // In auto transmission mode, don't accelerate the engine when switching gears
  float forward_input = abs(mForwardInput);
  if (mTransmission.mMode == ETransmissionMode::Auto)
    forward_input *= mTransmission.GetClutchFriction();

  // [ledger] the engine's damping: the speed it leaves, as ApplyDamping computes it, and
  // what the idle-speed clamp then adds (or the rev limit removes)
  const double engine_inertia = mEngine.mInertia;
  const double engine_omega0 = mEngine.GetAngularVelocity();
  const float free_rpm =
      mEngine.GetCurrentRPM() * max(0.0f, 1.0f - mEngine.mAngularDamping * inDeltaTime);

  // Apply engine damping
  mEngine.ApplyDamping(inDeltaTime);

  // [ledger]
  const double engine_damped = mEngine.GetAngularVelocity();
  {
    const double before = engine_omega0;
    const double free = free_rpm >= mEngine.mMinRPM && free_rpm <= mEngine.mMaxRPM
                            ? engine_damped
                            : free_rpm / VehicleEngine::cAngularVelocityToRPM;
    t.drivetrain += 0.5 * engine_inertia * (before * before - free * free);
    t.idle += 0.5 * engine_inertia * (engine_damped * engine_damped - free * free);
  }

  // Calculate engine torque
  float engine_torque = mEngine.GetTorque(forward_input);

  // Define a struct that contains information about driven differentials (i.e. that have wheels
  // connected)
  struct DrivenDifferential {
    const VehicleDifferentialSettings *mDifferential;
    float mAngularVelocity;
    float mClutchToDifferentialTorqueRatio;
    float mTempTorqueFactor;
  };

  // Collect driven differentials and their speeds
  Array<DrivenDifferential> driven_differentials;
  driven_differentials.reserve(mDifferentials.size());
  float differential_omega_min = FLT_MAX, differential_omega_max = 0.0f;
  for (const VehicleDifferentialSettings &d : mDifferentials) {
    float avg_omega = 0.0f;
    int avg_omega_denom = 0;
    int indices[] = {d.mLeftWheel, d.mRightWheel};
    for (int idx : indices)
      if (idx != -1) {
        avg_omega += wheels[idx]->GetAngularVelocity();
        avg_omega_denom++;
      }

    if (avg_omega_denom > 0) {
      avg_omega = abs(avg_omega * d.mDifferentialRatio /
                      float(avg_omega_denom)); // ignoring that the differentials may be rotating in
                                               // different directions
      driven_differentials.push_back({&d, avg_omega, d.mEngineTorqueRatio, 0});

      // Remember min and max velocity
      differential_omega_min = min(differential_omega_min, avg_omega);
      differential_omega_max = max(differential_omega_max, avg_omega);
    }
  }

  if (mDifferentialLimitedSlipRatio < FLT_MAX // Limited slip differential needs to be turned on
      && differential_omega_max > differential_omega_min) // There needs to be a velocity difference
  {
    // Calculate factor based on relative speed of a differential
    float sum_factor = 0.0f;
    for (DrivenDifferential &d : driven_differentials) {
      // Differential with max velocity gets factor 0, differential with min velocity 1
      d.mTempTorqueFactor = (differential_omega_max - d.mAngularVelocity) /
                            (differential_omega_max - differential_omega_min);
      sum_factor += d.mTempTorqueFactor;
    }

    // Normalize the result
    for (DrivenDifferential &d : driven_differentials)
      d.mTempTorqueFactor /= sum_factor;

    // Prevent div by zero
    differential_omega_min = max(1.0e-3f, differential_omega_min);
    differential_omega_max = max(1.0e-3f, differential_omega_max);

    // Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels
    // are turning at mDifferentialLimitedSlipRatio
    float alpha = min((differential_omega_max / differential_omega_min - 1.0f) /
                          (mDifferentialLimitedSlipRatio - 1.0f),
                      1.0f);
    JPH_ASSERT(alpha >= 0.0f);
    float one_min_alpha = 1.0f - alpha;

    // Update torque ratio for all differentials
    for (DrivenDifferential &d : driven_differentials)
      d.mClutchToDifferentialTorqueRatio =
          one_min_alpha * d.mClutchToDifferentialTorqueRatio + alpha * d.mTempTorqueFactor;
  }

#ifdef JPH_ENABLE_ASSERTS
  // Assert the values add up to 1
  float sum_torque_factors = 0.0f;
  for (DrivenDifferential &d : driven_differentials)
    sum_torque_factors += d.mClutchToDifferentialTorqueRatio;
  JPH_ASSERT(abs(sum_torque_factors - 1.0f) < 1.0e-6f);
#endif // JPH_ENABLE_ASSERTS

  // Define a struct that collects information about the wheels that connect to the engine
  struct DrivenWheel {
    WheelWV *mWheel;
    float mClutchToWheelRatio;
    float mClutchToWheelTorqueRatio;
    float mEstimatedAngularImpulse;
  };
  Array<DrivenWheel> driven_wheels;
  driven_wheels.reserve(wheels.size());

  // Collect driven wheels
  float transmission_ratio = mTransmission.GetCurrentRatio();
  for (const DrivenDifferential &dd : driven_differentials) {
    VehicleDifferentialSettings d = *dd.mDifferential;

    WheelWV *wl = d.mLeftWheel != -1 ? static_cast<WheelWV *>(wheels[d.mLeftWheel]) : nullptr;
    WheelWV *wr = d.mRightWheel != -1 ? static_cast<WheelWV *>(wheels[d.mRightWheel]) : nullptr;

    float clutch_to_wheel_ratio = transmission_ratio * d.mDifferentialRatio;

    if (wl != nullptr && wr != nullptr) {
      // Calculate torque ratio
      float ratio_l, ratio_r;
      d.CalculateTorqueRatio(wl->GetAngularVelocity(), wr->GetAngularVelocity(), ratio_l, ratio_r);

      // Add both wheels
      driven_wheels.push_back(
          {wl, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio * ratio_l, 0.0f});
      driven_wheels.push_back(
          {wr, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio * ratio_r, 0.0f});
    } else if (wl != nullptr) {
      // Only left wheel, all power to left
      driven_wheels.push_back(
          {wl, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio, 0.0f});
    } else if (wr != nullptr) {
      // Only right wheel, all power to right
      driven_wheels.push_back(
          {wr, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio, 0.0f});
    }
  }

  // [ledger] the engine's speed the solve leaves, before Jolt clamps it to the idle speed;
  // and, over the driven wheels, the clutch's work and the solve's own damping (see below)
  double engine_solved = engine_damped;
  double clutch_work = 0.0, wheel_damping = 0.0;

  bool solved = false;
  if (!driven_wheels.empty()) {
    // (Jolt's derivation of this implicit engine / clutch / wheel solve is in
    // WheeledVehicleController.cpp.)

    // Dimension of matrix is N + 1
    int n = (int)driven_wheels.size() + 1;

    // Last column of w is for the engine angular velocity
    int engine = n - 1;

    // Define a and b
    DynMatrix a(n, n);
    DynMatrix b(n, 1);

    // Get number of driven wheels as a float
    float num_driven_wheels_float = float(driven_wheels.size());

    // Angular velocity of engine
    float w_engine = mEngine.GetAngularVelocity();

    // Calculate the total strength of the clutch
    float clutch_strength = transmission_ratio != 0.0f
                                ? mTransmission.GetClutchFriction() * mTransmission.mClutchStrength
                                : 0.0f;

    // dt / Ie
    float dt_div_ie = inDeltaTime / mEngine.mInertia;

    // Calculate scale factor for impulses based on previous delta time
    float impulse_scale = mPreviousDeltaTime > 0.0f ? inDeltaTime / mPreviousDeltaTime : 0.0f;

    // Iterate the rows for the wheels
    for (int i = 0; i < (int)driven_wheels.size(); ++i) {
      DrivenWheel &w_i = driven_wheels[i];
      const WheelSettingsWV *settings = w_i.mWheel->GetSettings();

      // Get wheel inertia
      float inertia = settings->mInertia;

      // S * R(i)
      float s_r = clutch_strength * w_i.mClutchToWheelRatio;

      // dt * S * R(i) * F(i) / Iw
      float dt_s_r_f_div_iw = inDeltaTime * s_r * w_i.mClutchToWheelTorqueRatio / inertia;

      // Fill in the columns of a for wheel j
      for (int j = 0; j < (int)driven_wheels.size(); ++j) {
        const DrivenWheel &w_j = driven_wheels[j];
        a(i, j) = dt_s_r_f_div_iw * w_j.mClutchToWheelRatio / num_driven_wheels_float;
      }

      // Add ww(i, t+dt)
      a(i, i) += 1.0f;

      // Add the column for the engine
      a(i, engine) = -dt_s_r_f_div_iw;

      // Calculate external angular impulse operating on the wheel: TW(i) * dt
      float dt_tw = 0.0f;

      // Combine brake with hand brake torque
      float brake_torque =
          mBrakeInput * settings->mMaxBrakeTorque + mHandBrakeInput * settings->mMaxHandBrakeTorque;
      if (brake_torque > 0.0f) {
        // We're braking
        // Calculate brake angular impulse
        float sign;
        if (w_i.mWheel->GetAngularVelocity() != 0.0f)
          sign = Sign(w_i.mWheel->GetAngularVelocity());
        else
          sign = Sign(mTransmission.GetCurrentRatio()); // When wheels have locked up use the
                                                        // transmission ratio to determine the sign
        dt_tw = sign * inDeltaTime * brake_torque;
      }

      if (w_i.mWheel->HasContact()) {
        // We have wheel contact with the floor
        // Note that we don't know the torque due to the ground contact yet, so we use the impulse
        // applied from the last frame to estimate it Wheel torque TW = force * radius = lambda / dt
        // * radius
        dt_tw += impulse_scale * w_i.mWheel->GetLongitudinalLambda() * settings->mRadius;
      }

      w_i.mEstimatedAngularImpulse = dt_tw;

      // Fill in the constant b = ww(i,t)+(dt*TW(i))/Iw(i)
      b(i, 0) = w_i.mWheel->GetAngularVelocity() - dt_tw / inertia;

      // To avoid looping over the wheels again, we also fill in the wheel columns of the engine row
      // here
      a(engine, i) = -dt_div_ie * s_r / num_driven_wheels_float;
    }

    // Finalize the engine row
    a(engine, engine) = (1.0f + dt_div_ie * clutch_strength);
    b(engine, 0) = w_engine + dt_div_ie * engine_torque;

    // Solve the linear equation
    if (GaussianElimination(a, b)) {
      // Update the angular velocities for the wheels
      for (int i = 0; i < (int)driven_wheels.size(); ++i) {
        DrivenWheel &w_i = driven_wheels[i];
        const WheelSettingsWV *settings = w_i.mWheel->GetSettings();

        // Get solved wheel angular velocity
        float angular_velocity = b(i, 0);

        // [ledger]
        const double predicted = angular_velocity, before = w_i.mWheel->GetAngularVelocity();

        // We estimated TW and applied it in the equation above, but we haven't actually applied
        // this torque yet so we undo it here. It will be applied when we solve the actual braking /
        // the constraints with the floor.
        angular_velocity += w_i.mEstimatedAngularImpulse / settings->mInertia;

        // Update angular velocity
        w_i.mWheel->SetAngularVelocity(angular_velocity);

        // [ledger] The solve is implicit Euler: every torque at the end-of-step speed, the
        // one it predicted with the brake and ground torques it estimated. The wheel's
        // change is the clutch's angular impulse dL; at the predicted speed it is the
        // clutch's work, and dL^2 / 2I the step's own damping. Undoing the estimate leaves the
        // wheel dL tau / I more (tau, the estimate's angular impulse): energy the split step
        // makes, the ledger's drivetrain_split.
        {
          const double inertia = settings->mInertia;
          const double impulse = inertia * (double(angular_velocity) - before);
          clutch_work += impulse * predicted;
          wheel_damping += 0.5 * impulse * impulse / inertia;
          const double made = impulse * w_i.mEstimatedAngularImpulse / inertia;
          t.split += made;
          if (mBrakeInput * settings->mMaxBrakeTorque +
                  mHandBrakeInput * settings->mMaxHandBrakeTorque >
              0.0f)
            t.split_braked += made; // the brakes will take it back
        }
      }

      // Update the engine RPM
      mEngine.SetCurrentRPM(b(engine, 0) * VehicleEngine::cAngularVelocityToRPM);

      // [ledger]
      engine_solved = b(engine, 0);
      const float solved_rpm = b(engine, 0) * VehicleEngine::cAngularVelocityToRPM;
      if (solved_rpm >= mEngine.mMinRPM && solved_rpm <= mEngine.mMaxRPM)
        engine_solved = mEngine.GetAngularVelocity(); // not clamped

      // The speeds have been solved
      solved = true;
    } else {
      JPH_ASSERT(false, "New engine/wheel speeds could not be calculated!");
    }
  }

  if (!solved) {
    // [ledger] as ApplyTorque computes it
    const float free_rpm_torque = mEngine.GetCurrentRPM() + VehicleEngine::cAngularVelocityToRPM *
                                                                engine_torque * inDeltaTime /
                                                                mEngine.mInertia;

    // Engine not connected to wheels, apply all torque to engine rotation
    mEngine.ApplyTorque(engine_torque, inDeltaTime);

    // [ledger]
    engine_solved = free_rpm_torque >= mEngine.mMinRPM && free_rpm_torque <= mEngine.mMaxRPM
                        ? double(mEngine.GetAngularVelocity())
                        : free_rpm_torque / VehicleEngine::cAngularVelocityToRPM;
  }

  // [ledger] The engine row is I (w1 - w0) = dt (T - tc), at the end speed w1: the torque's
  // work T dt w1 is what the engine supplies, dt tc w1 what leaves it through the clutch, and
  // I (w1 - w0)^2 / 2 the step's own damping. Of the clutch's work, what did not reach the
  // wheels it lost in slip: dt tc (w1 - R w_wheel) >= 0, engine braking included. What the
  // idle-speed clamp then adds (or the rev limit removes) is the idle's.
  {
    const double clutch_impulse =
        double(engine_torque) * inDeltaTime - engine_inertia * (engine_solved - engine_damped);
    const double torque_work = double(engine_torque) * inDeltaTime * engine_solved;
    const double engine_now = mEngine.GetAngularVelocity();
    // The engine's own work is Jolt's over the driveline's efficiency; the gears lose the
    // difference before the clutch's work is counted.
    const double engine_work = torque_work / driveline_efficiency;
    t.engine += engine_work;
    t.supplied += engine_work;
    t.drivetrain += engine_work - torque_work;
    // Fuelled while the driver asks for torque (through a shift too) and at idle; cut on
    // the overrun.
    if (mForwardInput != 0.0f || mEngine.GetCurrentRPM() <= 1.1f * mEngine.mMinRPM)
      t.indicated +=
          engine_work +
          (friction_torque(engine_damped * VehicleEngine::cAngularVelocityToRPM) * engine_damped +
           accessory_power) *
              inDeltaTime;
    t.drivetrain +=
        0.5 * engine_inertia * (engine_solved - engine_damped) * (engine_solved - engine_damped) +
        (clutch_impulse * engine_solved - clutch_work) + wheel_damping;
    t.idle += 0.5 * engine_inertia * (engine_now * engine_now - engine_solved * engine_solved);
  }

  // Calculate if any of the wheels are slipping, this is used to prevent gear switching
  bool wheels_slipping = false;
  for (const DrivenWheel &w : driven_wheels)
    wheels_slipping |= w.mClutchToWheelTorqueRatio > 0.0f &&
                       (!w.mWheel->HasContact() || w.mWheel->mLongitudinalSlip > 0.1f);

  // Only allow shifting up when we're not slipping and we're increasing our RPM.
  // After a jump, we have a very high engine RPM but once we hit the ground the RPM should be
  // decreasing and we don't want to shift up during that time.
  bool can_shift_up = !wheels_slipping && mEngine.GetCurrentRPM() >= old_engine_rpm;

  // Update transmission
  mTransmission.Update(inDeltaTime, mEngine.GetCurrentRPM(), mForwardInput, can_shift_up);

  // Braking
  for (Wheel *w_base : wheels) {
    WheelWV *w = static_cast<WheelWV *>(w_base);
    const WheelSettingsWV *settings = w->GetSettings();

    // [ledger] what the brakes take from the wheel
    const double before = wheel_kinetic(*w);

    // Combine brake with hand brake torque
    float brake_torque =
        mBrakeInput * settings->mMaxBrakeTorque + mHandBrakeInput * settings->mMaxHandBrakeTorque;
    if (brake_torque > 0.0f) {
      // Calculate how much torque is needed to stop the wheels from rotating in this time step
      float brake_torque_to_lock_wheels =
          abs(w->GetAngularVelocity()) * settings->mInertia / inDeltaTime;
      if (brake_torque > brake_torque_to_lock_wheels) {
        // Wheels are locked
        w->SetAngularVelocity(0.0f);
        w->mBrakeImpulse =
            (brake_torque - brake_torque_to_lock_wheels) * inDeltaTime / settings->mRadius;
      } else {
        // Slow down the wheels
        w->ApplyTorque(-Sign(w->GetAngularVelocity()) * brake_torque, inDeltaTime);
        w->mBrakeImpulse = 0.0f;
      }
    } else {
      // Not braking
      w->mBrakeImpulse = 0.0f;
    }

    // [ledger]
    t.brake += before - wheel_kinetic(*w);
  }

  // Remember previous delta time so we can scale the impulses correctly
  mPreviousDeltaTime = inDeltaTime;
}

bool ackermann_controller::measured_friction(float inDeltaTime) {
  using namespace JPH;
  energy_tally &t = *tally;
  const Body &body = *mConstraint.GetVehicleBody();

  // [ledger] The lateral constraint was warm started with last sub-step's impulse before this
  // first solve: its work, J along the contact's lateral axis at the contact point, undone from
  // the velocities it left.
  if (t.lateral_warm_start) {
    t.lateral_warm_start = false;
    const MotionProperties *mp = body.GetMotionProperties();
    const double now = chassis_kinetic(body);
    for (const Wheel *w : mConstraint.GetWheels()) {
      const float lambda = w->GetLateralLambda();
      if (!w->HasContact() || lambda == 0.0f)
        continue;
      const Vec3 impulse = lambda * w->GetContactLateral();
      const Vec3 r = Vec3(w->GetContactPosition() - body.GetCenterOfMassPosition());
      const Vec3 v = body.GetLinearVelocity() - mp->GetInverseMass() * impulse;
      const Vec3 om =
          body.GetAngularVelocity() - body.GetInverseInertia().Multiply3x3(r.Cross(impulse));
      const Vec3 wl = (body.GetRotation() * mp->GetInertiaRotation()).Conjugated() * om;
      const Vec3 inv_i = mp->GetInverseInertiaDiagonal();
      double before = 0.5 *
                      (double(v.GetX()) * v.GetX() + double(v.GetY()) * v.GetY() +
                       double(v.GetZ()) * v.GetZ()) /
                      mp->GetInverseMass();
      for (int k = 0; k < 3; ++k)
        if (inv_i[k] > 0.0f)
          before += 0.5 * double(wl[k]) * wl[k] / inv_i[k];
      t.tire += before - now;
    }
  }

  bool impulse = false;

  float *max_lateral_friction_impulse =
      (float *)JPH_STACK_ALLOC(mConstraint.GetWheels().size() * sizeof(float));

  uint wheel_index = 0;
  for (Wheel *w_base : mConstraint.GetWheels()) {
    if (w_base->HasContact()) {
      WheelWV *w = static_cast<WheelWV *>(w_base);
      const WheelSettingsWV *settings = w->GetSettings();

      // [ledger]
      const double before = chassis_kinetic(body) + wheel_kinetic(*w);

      // Calculate max impulse that we can apply on the ground
      float max_longitudinal_friction_impulse;
      mTireMaxImpulseCallback(wheel_index, max_longitudinal_friction_impulse,
                              max_lateral_friction_impulse[wheel_index], w->GetSuspensionLambda(),
                              w->mCombinedLongitudinalFriction, w->mCombinedLateralFriction,
                              w->mLongitudinalSlip, w->mLateralSlip, inDeltaTime);

      // Calculate relative velocity between wheel contact point and floor in longitudinal direction
      Vec3 relative_velocity =
          mConstraint.GetVehicleBody()->GetPointVelocity(w->GetContactPosition()) -
          w->GetContactPointVelocity();
      float relative_longitudinal_velocity = relative_velocity.Dot(w->GetContactLongitudinal());

      // Calculate brake force to apply
      float min_longitudinal_impulse, max_longitudinal_impulse;
      if (w->mBrakeImpulse != 0.0f) {
        // Limit brake force by max tire friction
        float brake_impulse = min(w->mBrakeImpulse, max_longitudinal_friction_impulse);

        // Check which direction the brakes should be applied (we don't want to apply an impulse
        // that would accelerate the vehicle)
        if (relative_longitudinal_velocity >= 0.0f) {
          min_longitudinal_impulse = -brake_impulse;
          max_longitudinal_impulse = 0.0f;
        } else {
          min_longitudinal_impulse = 0.0f;
          max_longitudinal_impulse = brake_impulse;
        }

        // Longitudinal impulse, note that we assume that once the wheels are locked that the brakes
        // have more than enough torque to keep the wheels locked so we exclude any rotation deltas
        impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse,
                                                      max_longitudinal_impulse);

        // [ledger] a locked wheel's sliding is the brakes'
        t.brake += before - (chassis_kinetic(body) + wheel_kinetic(*w));
      } else {
        // Assume we want to apply an angular impulse that makes the delta velocity between wheel
        // and ground zero in one time step, calculate the amount of linear impulse needed to do
        // that
        float desired_angular_velocity = relative_longitudinal_velocity / settings->mRadius;
        float linear_impulse = (w->GetAngularVelocity() - desired_angular_velocity) *
                               settings->mInertia / settings->mRadius;

        // Limit the impulse by max tire friction
        float prev_lambda = w->GetLongitudinalLambda();
        min_longitudinal_impulse = max_longitudinal_impulse =
            Clamp(prev_lambda + linear_impulse, -max_longitudinal_friction_impulse,
                  max_longitudinal_friction_impulse);

        // Longitudinal impulse
        impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse,
                                                      max_longitudinal_impulse);

        // Update the angular velocity of the wheels according to the lambda that was applied
        w->SetAngularVelocity(w->GetAngularVelocity() - (w->GetLongitudinalLambda() - prev_lambda) *
                                                            settings->mRadius / settings->mInertia);

        // [ledger]
        t.tire += before - (chassis_kinetic(body) + wheel_kinetic(*w));
      }
    }
    ++wheel_index;
  }

  wheel_index = 0;
  for (Wheel *w_base : mConstraint.GetWheels()) {
    if (w_base->HasContact()) {
      WheelWV *w = static_cast<WheelWV *>(w_base);

      // [ledger]
      const double before = chassis_kinetic(body);

      // Lateral friction
      float max_lateral_impulse = max_lateral_friction_impulse[wheel_index];
      impulse |=
          w->SolveLateralConstraintPart(mConstraint, -max_lateral_impulse, max_lateral_impulse);

      // [ledger]
      t.tire += before - chassis_kinetic(body);
    }
    ++wheel_index;
  }

  return impulse;
}

} // namespace detail
} // namespace physics
} // namespace cvc

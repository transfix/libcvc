/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Jolt rollout (physics/jolt_rollout.h, rollout_kind::jolt; docs/NAV_TRAINING.md, "Training
// on Jolt"): the Jolt Humvee runs the training rollout's forward and the four-wheel
// port-Hamiltonian model, calibrated to Jolt's energy ledger, gives its gradient.
//  1. the vehicle reads back as the model's state, at rest where it was put;
//  2. a batch replays bit for bit from save(), which the finite differences below rely on;
//  3. the scene's walls are in the world, one copy per agent, and a vehicle driven into one
//     reports the contact;
//  4. calibrate_dyn4 closes the model's books against Jolt's ledger on held-out runs;
//  5. the gradient agrees with central differences of the loss on Jolt;
//  6. training on Jolt lowers the loss on Jolt.
// The Humvee drives the trainer's city at about 0.106 normalized units per metre (veh_L 0.35 for
// its 3.30 m wheelbase), with a Humvee's acceleration and cornering.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn4_ledger.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/jolt_rollout.h>
#include <cvc/physics/nav_plant.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <vector>

using cvc::nav::coef_trainer;
using cvc::nav::train_config;
using cvc::nav::training_scene;
using cvc::physics::jolt_rollout;
using cvc::physics::vehicle_desc;

namespace {

#define REQUIRE_JOLT()                                                                             \
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt))                               \
  GTEST_SKIP() << "this libcvc was built without Jolt"

train_config humvee_cfg(int n) {
  train_config c;
  c.n = n;
  c.hidden = 16;
  c.veh_L = 0.35f;
  c.veh_a_max = 0.3f;     // 2.8 m/s^2
  c.veh_a_lat_max = 0.5f; // 4.7 m/s^2
  c.rollout = cvc::nav::rollout_kind::jolt;
  c.dyn4 = cvc::physics::nav_plant::dyn4_params_at(vehicle_desc::humvee(), c.veh_L);
  c.plant = std::make_shared<jolt_rollout>(vehicle_desc::humvee(), c.veh_L);
  return c;
}

// The batch train() resets the plant to: starts and goals from `seed`, aimed at the goals.
void reset_batch(const training_scene &sc, const train_config &cfg, unsigned seed,
                 std::vector<float> &o, std::vector<float> &goal) {
  const int n = cfg.n;
  o.resize(2 * n);
  goal.resize(2 * n);
  sc.sample_starts_goals(n, seed, o.data(), goal.data(), cfg.goal_min, cfg.goal_max,
                         cvc::nav::start_clearance_of(cfg));
  std::vector<float> psi(n);
  for (int i = 0; i < n; ++i)
    psi[i] = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]);
  cvc::nav::rollout_batch b;
  b.scene = &sc;
  b.veh = cvc::nav::bike_veh_of(sc, cfg);
  b.model = cfg.dyn4;
  b.price = cfg.dyn4_price;
  b.mu = cfg.ground_mu;
  b.n = n;
  b.o = o.data();
  b.psi = psi.data();
  cfg.plant->reset(b);
}

// cfg's model calibrated on Jolt runs (the fit's seed), with its price.
cvc::nav::dyn4_calibration calibrate(const training_scene &sc, train_config &cfg, int n, int steps,
                                     unsigned seed) {
  const std::vector<cvc::nav::dyn4_ledger_tick> ticks =
      cvc::nav::record_plant_ticks(sc, cfg, *cfg.plant, n, steps, seed);
  const cvc::nav::dyn4_calibration cal =
      cvc::nav::calibrate_dyn4(cfg.dyn4, cvc::nav::bike_veh_of(sc, cfg), cfg.ground_mu, ticks);
  cfg.dyn4 = cvc::nav::calibrated(cfg.dyn4, cal);
  cfg.dyn4_price = cal.price;
  return cal;
}

void print_books(const char *what, const cvc::nav::dyn4_books &b) {
  std::printf("  %-22s %5llu ticks (%llu contact, %llu rest): port %+.3f tire %+.3f drag %+.3f "
              "rolling %+.3f damper %+.3f dissipated %+.3f stored %+.3f drawn %+.3f (r2 %.3f), "
              "twist %.4f\n",
              what, static_cast<unsigned long long>(b.ticks),
              static_cast<unsigned long long>(b.skipped_contact),
              static_cast<unsigned long long>(b.skipped_rest), b.port.error(), b.tire.error(),
              b.drag.error(), b.rolling.error(), b.damper.error(), b.dissipated.error(),
              b.stored.error(), b.drawn.error(), b.drawn_r2, b.twist_rms);
}

} // namespace

// The Humvee put on a start reads back there, at rest: the rear axle on the start, the heading,
// no speed, its heave and attitude from where it settled near zero, the model's actuator at rest.
// Several of the city's starts are close enough to a block that the body is turned to fit.
TEST(JoltRollout, ReadsTheVehicleBackAtRest) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(12);
  std::vector<float> o, goal;
  reset_batch(sc, cfg, 5, o, goal);
  auto &jolt = static_cast<jolt_rollout &>(*cfg.plant);
  EXPECT_NEAR(jolt.units_per_metre(), 0.35 / 3.30, 1e-7);
  EXPECT_EQ(jolt.size(), 12);
  for (int i = 0; i < 12; ++i) {
    const cvc::nav::diff::dyn4_state<float> s = jolt.state(i);
    // Aimed at its goal, turned if its body would stand on a wall there.
    const float psi = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]) +
                      static_cast<float>(jolt.start_turn(i));
    std::printf("  agent %d: rear axle off its start by (%.2e, %.2e), heading %+.2e, vx %.2e, z "
                "%.2e, roll %.2e, pitch %.2e\n",
                i, s.x - o[2 * i], s.y - o[2 * i + 1], s.psi - psi, s.vx, s.z, s.phi, s.th);
    const float cm = static_cast<float>(jolt.units_per_metre() * 0.02); // 2 cm, normalized
    EXPECT_NEAR(s.x, o[2 * i], cm);
    EXPECT_NEAR(s.y, o[2 * i + 1], cm);
    EXPECT_NEAR(s.psi, psi, 2e-3f);
    EXPECT_NEAR(s.vx, 0.0f, cm);
    EXPECT_NEAR(s.z, 0.0f, cm);
    EXPECT_NEAR(s.phi, 0.0f, 0.01f);
    EXPECT_NEAR(s.th, 0.0f, 0.01f);
    EXPECT_EQ(s.a, 0.0f);
    const cvc::nav::plant_energy_lines l = jolt.lines(i);
    EXPECT_EQ(l.drawn, 0.0) << "no step yet";
  }
}

// A batch restored (jolt_rollout::save / restore) repeats every state and ledger line.
TEST(JoltRollout, ReplaysABatchBitForBit) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(6);
  std::vector<float> o, goal;
  reset_batch(sc, cfg, 9, o, goal);
  cvc::nav::rollout_plant &p = *cfg.plant;
  const int n = cfg.n;
  std::vector<float> d(n), a(n);
  const auto drive = [&](int t) {
    for (int i = 0; i < n; ++i) {
      d[i] = 0.3f * std::sin(0.1f * static_cast<float>(t + 5 * i));
      a[i] = 0.2f - 0.004f * static_cast<float>(t % 50);
    }
    p.step(d.data(), a.data());
  };
  for (int t = 0; t < 10; ++t)
    drive(t);
  const std::vector<std::uint8_t> saved = p.save();
  const auto run = [&] {
    std::vector<float> out;
    for (int t = 10; t < 35; ++t) {
      drive(t);
      for (int i = 0; i < n; ++i) {
        const cvc::nav::diff::dyn4_state<float> s = p.state(i);
        for (int k = 0; k < cvc::nav::diff::dyn4_state<float>::size; ++k)
          out.push_back(s[k]);
        const cvc::nav::plant_energy_lines l = p.lines(i);
        out.push_back(static_cast<float>(l.drawn));
        out.push_back(static_cast<float>(l.port));
        out.push_back(static_cast<float>(l.tire));
      }
    }
    return out;
  };
  const std::vector<float> first = run();
  p.restore(saved);
  const std::vector<float> again = run();
  ASSERT_EQ(first.size(), again.size());
  for (std::size_t k = 0; k < first.size(); ++k)
    ASSERT_EQ(first[k], again[k]) << "value " << k;
  EXPECT_GT(std::fabs(first[3]), 0.0f) << "the vehicles should move";
}

// Every agent drives its own copy of the scene, walls included: an agent driven straight at a
// block from beside it reports the contact, and no agent touches another's vehicle.
TEST(JoltRollout, EachAgentDrivesItsOwnCopyOfTheWalls) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(4);
  // A free cell (about a metre) with room around it and a wall 7 cells on toward +x.
  const float k = 0.35f / 3.30f;
  const auto occupied = [&](int r, int c) {
    return r < 0 || c < 0 || r >= sc.rows || c >= sc.cols ||
           sc.occ[static_cast<std::size_t>(r) * sc.cols + c] != 0;
  };
  int start = -1;
  for (int cell : sc.free_cells) {
    const int r = cell / sc.cols, c = cell % sc.cols;
    bool room = c + 7 < sc.cols && occupied(r, c + 7);
    for (int dr = -3; dr <= 3 && room; ++dr)
      for (int dc = -3; dc <= 5 && room; ++dc)
        room = !occupied(r + dr, c + dc);
    if (room) {
      start = cell;
      break;
    }
  }
  ASSERT_GE(start, 0);
  const int r = start / sc.cols, c = start % sc.cols;
  const float x = static_cast<float>(
      (sc.min_x + static_cast<double>(c) / (sc.cols - 1) * (sc.max_x - sc.min_x) - sc.cx) *
      sc.scale);
  const float y = static_cast<float>(
      (sc.min_y + static_cast<double>(r) / (sc.rows - 1) * (sc.max_y - sc.min_y) - sc.cy) *
      sc.scale);
  std::vector<float> o(2 * cfg.n), psi(cfg.n, 0.0f);
  for (int i = 0; i < cfg.n; ++i) {
    o[2 * i] = x;
    o[2 * i + 1] = y;
  }
  cvc::nav::rollout_batch b;
  b.scene = &sc;
  b.veh = cvc::nav::bike_veh_of(sc, cfg);
  b.model = cfg.dyn4;
  b.n = cfg.n;
  b.o = o.data();
  b.psi = psi.data();
  auto &jolt = static_cast<jolt_rollout &>(*cfg.plant);
  jolt.reset(b);
  EXPECT_GT(jolt.wall_boxes(), 0);
  std::vector<float> d(cfg.n, 0.0f), a(cfg.n, 2.0f * k); // 2 m/s^2 straight ahead
  int first_contact = -1;
  for (int t = 0; t < 150 && first_contact < 0; ++t) {
    jolt.step(d.data(), a.data());
    for (int i = 0; i < cfg.n; ++i) {
      EXPECT_LT(jolt.plant().vehicle_contact_step(i), 0) << "agent " << i << " met another";
      if (jolt.lines(i).contact && first_contact < 0)
        first_contact = t;
    }
  }
  std::printf("  %d wall boxes a copy; the agents reach the wall at tick %d\n", jolt.wall_boxes(),
              first_contact);
  EXPECT_GE(first_contact, 0) << "the wall should stop them";
  // All four copies are the same scene: the agents are where the first is.
  const cvc::nav::diff::dyn4_state<float> s0 = jolt.state(0);
  for (int i = 1; i < cfg.n; ++i)
    EXPECT_NEAR(jolt.state(i).x, s0.x, 1e-3f);
}

// The model's books against Jolt's ledger, before and after calibrate_dyn4, on held-out runs
// (other starts and goals) under the untrained policy. Calibrated, the dissipation and what the
// source paid close; the port carries the slip loss the model has no wheel spin for.
TEST(JoltRollout, CalibrationClosesTheBooksOnHeldOutRuns) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(16);
  const cvc::nav::diff::dyn4_params nominal = cfg.dyn4;
  const cvc::nav::diff::bike_veh v = cvc::nav::bike_veh_of(sc, cfg);
  const cvc::nav::dyn4_calibration cal = calibrate(sc, cfg, 16, 200, 1);
  const std::vector<cvc::nav::dyn4_ledger_tick> held =
      cvc::nav::record_plant_ticks(sc, cfg, *cfg.plant, 16, 200, 2);
  const cvc::nav::dyn4_books before =
      cvc::nav::dyn4_books_of(nominal, v, cfg.ground_mu, cvc::nav::dyn4_calibration{}, held);
  const cvc::nav::dyn4_books after = cvc::nav::dyn4_books_of(nominal, v, cfg.ground_mu, cal, held);
  std::printf("  drive %.3f brake %.3f comp %.3f lag %.3f s | tire x%.3f drag x%.3f rolling x%.3f "
              "damper x%.3f | books tire x%.3f lon %.3e | price %.3f per port J, idle %.0f W\n",
              cal.drive_gain, cal.brake_gain, cal.resist_comp, cal.tau_drive, cal.tire, cal.drag,
              cal.rolling, cal.damper, cal.tire_lat, cal.tire_lon, cal.price.drive,
              cal.price.idle / (v.L / 3.30 * v.L / 3.30));
  print_books("fit, before", cal.before);
  print_books("fit, after", cal.after);
  print_books("held out, before", before);
  print_books("held out, after", after);
  ASSERT_GT(after.ticks, 1000u);
  EXPECT_LT(std::fabs(after.dissipated.error()), 0.15);
  EXPECT_LT(std::fabs(after.drawn.error()), 0.10);
  EXPECT_GT(after.drawn_r2, 0.5);
  EXPECT_LE(after.twist_rms, before.twist_rms) << "calibrated, the model's step is no worse";
  EXPECT_LT(std::fabs(after.dissipated.error()), std::fabs(before.dissipated.error()));
}

// The gradient on Jolt: the model's at Jolt's states against central differences of the loss on
// Jolt itself, the plant restored between the two sides. The batch first drives 20 steps under
// the policy, so the window of 30 (1.8 s) starts from vehicles under way: from rest a short
// window barely moves them, and the loss's change under a small step in the parameters is below
// what Jolt's own float arithmetic (the copies sit hundreds of metres from its origin) leaves in
// it. Checked along each of the policy's three output biases, which shift alpha, beta and gamma
// for every agent and step (the coefficients the policy controls), and along the gradient. The
// forward is Jolt's, so the model's gradient is an estimate of Jolt's: each must agree in sign
// and within a third in size. The uncalibrated model's is printed beside it.
TEST(JoltRollout, GradientAgreesWithJoltFiniteDifferences) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(32);
  const cvc::nav::diff::dyn4_params nominal = cfg.dyn4;
  calibrate(sc, cfg, 16, 150, 1);
  const int n = cfg.n, warm = 20, window = 30;
  coef_trainer tr(cfg, 1);
  const int P = tr.num_params();
  std::vector<float> o, goal, dummy(2 * n, 0.0f), g, g_nominal;
  reset_batch(sc, cfg, 21, o, goal);
  tr.loss_and_grad(sc, dummy.data(), dummy.data(), goal.data(), n, warm, nullptr);
  const std::vector<std::uint8_t> start = cfg.plant->save();
  const double L0 = tr.loss_and_grad(sc, dummy.data(), dummy.data(), goal.data(), n, window, &g);
  {
    train_config c = cfg;
    c.dyn4 = nominal;
    coef_trainer t(c, 1);
    cfg.plant->restore(start);
    t.loss_and_grad(sc, dummy.data(), dummy.data(), goal.data(), n, window, &g_nominal);
  }
  const std::vector<float> base = tr.params();
  const auto loss_at = [&](const std::vector<float> &p) {
    cfg.plant->restore(start);
    tr.set_params(p);
    return tr.loss_and_grad(sc, dummy.data(), dummy.data(), goal.data(), n, window, nullptr);
  };
  const auto fd_along = [&](const std::vector<double> &u, double eps) {
    std::vector<float> pp(P), pm(P);
    for (int i = 0; i < P; ++i) {
      pp[i] = base[i] + static_cast<float>(eps * u[i]);
      pm[i] = base[i] - static_cast<float>(eps * u[i]);
    }
    return (loss_at(pp) - loss_at(pm)) / (2.0 * eps);
  };
  const auto rel = [](double a, double b) {
    return std::fabs(a - b) / (std::fabs(a) + std::fabs(b));
  };
  std::printf("[jolt-gradcheck] loss %.4f over %d agents x %d steps after %d\n", L0, n, window,
              warm);
  const char *names[3] = {"alpha", "beta", "gamma"};
  for (int k = 0; k < 3; ++k) {
    std::vector<double> u(P, 0.0);
    u[P - 3 + k] = 1.0;
    const double fd = fd_along(u, 0.05);
    const double an = g[P - 3 + k];
    std::printf("  d/d(%s bias): model %+.4e (uncalibrated %+.4e)  Jolt %+.4e  rel %.3f\n",
                names[k], an, g_nominal[P - 3 + k], fd, rel(an, fd));
    EXPECT_EQ(an > 0.0, fd > 0.0) << names[k]
                                  << ": the model's gradient and Jolt's disagree in sign";
    EXPECT_LT(rel(an, fd), 0.35) << names[k];
  }
  double gnorm = 0.0;
  for (int i = 0; i < P; ++i)
    gnorm += static_cast<double>(g[i]) * g[i];
  gnorm = std::sqrt(gnorm);
  std::vector<double> ug(P);
  for (int i = 0; i < P; ++i)
    ug[i] = g[i] / gnorm;
  const double dd = fd_along(ug, 0.05);
  cfg.plant->restore(start);
  tr.set_params(base);
  std::printf("  along the gradient: model |g| %.4e  Jolt %.4e  ratio %.3f\n", gnorm, dd,
              dd / gnorm);
  EXPECT_GT(dd, 0.0) << "the model's gradient does not point uphill on Jolt";
  EXPECT_LT(rel(dd, gnorm), 0.35);
}

// Training on Jolt: 25 outer steps on one batch (train() with steps 1, the same seed, Adam's
// state kept between calls), a horizon of 60 steps (3.6 s, long enough for the Humvees to get
// under way: see the gradcheck) in windows of 10, lower the horizon loss on that batch on Jolt.
TEST(JoltRollout, TrainingLowersTheLossOnJolt) {
  REQUIRE_JOLT();
  const training_scene sc = cvc::nav::city_scene(96);
  train_config cfg = humvee_cfg(16);
  calibrate(sc, cfg, 16, 150, 1);
  cfg.horizon = 60;
  cfg.window = 10;
  cfg.steps = 1;
  cfg.seed = 31;
  cfg.lr = 5e-4f;
  cfg.cosine_lr = false;
  coef_trainer tr(cfg, 2);
  std::vector<float> o, goal, dummy(2 * cfg.n, 0.0f);
  const auto horizon_loss = [&] {
    reset_batch(sc, cfg, cfg.seed, o, goal);
    return tr.loss_and_grad(sc, dummy.data(), dummy.data(), goal.data(), cfg.n, cfg.horizon,
                            nullptr);
  };
  const double before = horizon_loss();
  for (int step = 0; step < 25; ++step)
    tr.train(sc);
  const double after = horizon_loss();
  const auto &jolt = static_cast<const jolt_rollout &>(*cfg.plant);
  std::printf("[jolt-train] horizon loss before %.4f after %.4f (%.0f us per Humvee step)\n",
              before, after, jolt.step_us_per_vehicle());
  EXPECT_TRUE(std::isfinite(after));
  EXPECT_LT(after, before) << "training on Jolt did not lower the loss on Jolt";
}

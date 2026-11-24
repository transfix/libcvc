/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH drives a sim_world's vehicles: part 2b of C5 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.5), the
// twin of GRL-SNAM's cars controller (grl_snam.ariadne.coph_cars, T4). It lives outside sim_world,
// as route_follower does (D10):
//
//   cvc::nav::coph_driver drive(sim, cfg);
//   drive.attach(0);
//   drive.attach(1);
//   for (;;) {
//     drive.apply(); // before each step
//     sim.step();
//   }
//
// Each attached vehicle carries a car's learner (coph_agent with no network: CooperativeLearner on
// Learner, routing with the profile's own planner; GRL-SNAM's G13), all of them in one coph_fleet
// over a coph_channel. Each apply():
//   - Every ticks_per_step() ticks (CoPH's dt 0.12 sqrt(s), over the sim's veh.dt, rounded, at
//     least 1; D6) is a CoPH step. Each vehicle's state is read in CoPH's frame (Froude scaling by
//     s about the origin, D4: q = (p - origin) / s, v = speed (cos h, sin h) / sqrt(s)); its goal
//     is the sim's, or the point local_goal along the straight line to it when that is farther,
//     slid on once the vehicle is within half of local_goal of it or when the sim's goal moves
//     (D12). It senses CoPH's 21 lattice points of the risk (the material plane's r~ where the
//     sim has one, else base_risk) and the occupied cells within the scaled sensor radius as disks
//     (visible_obstacles: on its plane's last-visible cells, or on the truth under freeze_sense).
//     The fleet's boundary follows: the held step completes with the mean force the vehicle took
//     (D5), the vehicle is done when the sim says it reached its goal, packets go out on the
//     update cadence, and each vehicle updates and acts: the force it holds to the next step. A
//     vehicle that is done runs again once the sim gives it a new goal (coph_fleet::resume, as
//     GRL-SNAM's cars controller does), its learner keeping what it learned.
//   - Every tick, each vehicle not done allocates its held force (coph_allocate, §3.3.2: the
//     acceleration along the heading and the steer, with its wheelbase, the steer lock and its
//     acceleration limit) and drives with it through sim_world::override_command. A vehicle that
//     is done brakes: steer 0, acceleration -clamp(speed / max(dt, 0.25), -a_max, a_max).
//
// Each vehicle's ledger is in CoPH's frame, per unit mass, over its CoPH steps: the work of the
// force it took along its own displacement, its own kinetic change, the heat CoPH's plant books
// from the same start under that force at the learner's damping (C2's closed form), and their
// residual, which measures how far the vehicle's motion is from a damped point mass; the
// unallocated work e . dq of the force it did not take; the learners' shaping jumps; the risk
// cost dt 4 r; and the allocation residual sqrt(sum |e|^2 / sum |u|^2) over the ticks.
//
// Without a coph_driver, or with one and no vehicle attached, every run is byte-unchanged.
//
// coph_driver.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_DRIVER_H
#define CVC_NAV_COPH_DRIVER_H

#include <cvc/nav/coph_allocation.h>
#include <cvc/nav/coph_fleet.h>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace cvc {
namespace nav {

class sim_world;

struct coph_driver_config {
  coph_config cfg;                // every vehicle's learner (the profile; steps: the run's length)
  coph_communication_config comm; // every vehicle's, and the byte budget
  std::vector<int> senders;       // the attached vehicles (by attachment order) that export; empty:
                                  // all of them
  double froude = 1.0;            // s (D4): CoPH's lengths are s world units, its times sqrt(s)
  std::optional<coph_vec2> origin;       // CoPH's origin in world units; unset: the map's centre
  double local_goal = 30.0;              // world units (D12)
  double speed_floor = coph_speed_floor; // world units / s (§3.3.2)
  double base_risk = 0.02;               // the risk read where the sim has no material layer
  std::uint32_t seed = 0;                // the readings' uid prefix and the bootstrap draws
  // The sensor noise of reading j of a vehicle (its attachment index) at a CoPH step; none when
  // unset.
  std::function<double(int vehicle, long long step, int j)> noise;
};

class coph_driver {
public:
  // Throws std::invalid_argument when the configuration is outside the demos' profile, froude or
  // local_goal is not positive, or the sim's veh.dt is not.
  coph_driver(sim_world &sim, coph_driver_config cfg,
              std::shared_ptr<coph_channel> channel = nullptr);

  // Agent i drives by CoPH from the next apply() on. Attach before the first apply(); each once.
  void attach(int i);
  const std::vector<int> &attached() const { return attached_; }

  // Before each sim.step().
  void apply();

  int ticks_per_step() const { return k_; }
  long long steps() const { return steps_; }               // CoPH steps taken
  const coph_fleet *fleet() const { return fleet_.get(); } // null before the first apply()

  // A vehicle's account (its attachment index), as above.
  struct ledger {
    coph_energy energy;
    double risk_cost = 0;
    double e2 = 0, u2 = 0; // sum |e|^2 dt and sum |u|^2 dt over the ticks, dt in CoPH's frame
    double max_step_residual = 0;
    double allocation_residual() const;
  };
  const ledger &account(int vehicle) const { return ledgers_.at(vehicle); }
  // Whether the vehicle is at its goal: it brakes until the sim gives it a new one.
  bool done(int vehicle) const;

private:
  struct car {
    int agent = -1;
    coph_vec2 goal{};  // the sim's, world units
    coph_vec2 local{}; // the local goal, world units
    coph_vec2 u{};     // the held force
    coph_vec2 used_sum{}, e_sum{};
    int held = 0;
    std::optional<coph_state> x_prev;
    bool done = false;
  };
  struct pose {
    coph_vec2 p{};
    double heading = 0, speed = 0;
    bool reached = false;
  };
  std::vector<pose> read() const;
  coph_state state(const pose &p) const;
  coph_vec2 to_coph(const coph_vec2 &world) const;
  coph_vec2 to_world(const coph_vec2 &q) const;
  coph_vec2 local_goal(const coph_vec2 &p, const coph_vec2 &goal) const;
  double risk_at(int agent, const coph_vec2 &world) const;
  std::vector<coph_shape> obstacles_near(int agent, const coph_vec2 &world) const;
  void coph_step(const std::vector<pose> &poses);

  sim_world &sim_;
  coph_driver_config cfg_;
  std::shared_ptr<coph_channel> channel_;
  coph_vec2 origin_{};
  int k_ = 1;
  long long ticks_ = 0, steps_ = 0;
  std::vector<int> attached_;
  std::vector<car> cars_;
  std::vector<ledger> ledgers_;
  std::unique_ptr<coph_fleet> fleet_;
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_DRIVER_H

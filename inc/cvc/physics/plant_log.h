/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::plant_log: what a nav_plant's vehicles did, tick by tick, as planar rigid bodies
// in SI, for fitting a plant model to the physics; and the dynamic single-track model's one-tick
// residual against them, beside the kinematic bicycle's.
//
//   plant_log log(sim, plant);   // sim drives `plant`
//   log.start();                 // the agents before the first step
//   for (...) { sim.step(); log.record(); }
//   const auto res = one_tick_residuals(log.runs(), dt, desc, mu, nsub);
//   const flight_set f = cut_flights(log.runs(), 250, 250);
//   write_npy("flights.npy", f.data, {f.flights, f.rows, kPlanarWidth});
//
// A sample is the vehicle's centre of mass and heading on SE(2), its body twist there, the
// steering it has, and the command nav held over the tick that ended at the sample (the
// zero-order hold a plant-learning dataset wants: row k + 1's input acted from t_k to t_k+1).
// Everything is in the physics world's frame (metres, radians, seconds).
//
// The residual is the vehicle-physics plan's per-tick drift (D14) with diff::dyn_plant beside
// the kinematic bicycle: each tick both models start from the plant's own state at the tick's
// start, take the command it held for `nsub` substeps of dt / nsub, and are compared with where
// the plant was at the tick's end, binned by the lateral acceleration the plant measured over
// that tick. The dynamic model starts from the measured steering and its own drive actuator
// (which runs on the commands alone).

#ifndef __CVC_PHYSICS_PLANT_LOG_H__
#define __CVC_PHYSICS_PLANT_LOG_H__

#include <cstddef>
#include <cstdint>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/dyn_power.h>
#include <cvc/physics/vehicle.h>
#include <string>
#include <vector>

namespace cvc {
namespace nav {
class sim_world;
}
namespace physics {

class nav_plant;

// One vehicle at one tick. cvc::nav::plant_sample (nav/plant_types.h, nav_stats' plant ledger)
// shares vx, vy, r, steer, grade, roll, steer_cmd, accel_cmd, a_lat, a_lon, contacts, wheels and
// wheels_down with it, by name and meaning; nav_plant computes them as this log does.
struct planar_sample {
  double t = 0.0;                    // s since start()
  double x = 0.0, y = 0.0;           // the centre of mass
  double psi = 0.0;                  // heading, continuous along the run (unwrapped)
  double vx = 0.0, vy = 0.0;         // the centre of mass's velocity in the body frame
  double r = 0.0;                    // yaw rate
  double rear_x = 0.0, rear_y = 0.0; // the rear axle (the point nav tracks)
  double steer = 0.0;                // the road-wheel (bicycle) angle the vehicle has
  double grade = 0.0;                // sin of the slope along the heading, + uphill (the pitch)
  double roll = 0.0;                 // sin of the body's roll, + left side up
  double steer_cmd = 0.0;            // nav's command over the tick that ended here; 0 at start()
  double accel_cmd = 0.0;            // m/s^2 along the body's x, as nav sent it
  double drive = 0.0; // accel_cmd within what the vehicle can do: -max_decel..max_accel
  double vmax = 0.0;  // the top speed nav plans this vehicle with (m/s)
  double a_lat = 0.0, a_lon = 0.0; // the centre of mass's acceleration over that tick, body axes
  int contacts = 0;                // static contacts during that tick
  int wheels = 0, wheels_down = 0; // the vehicle's wheels, and how many touch the ground
  bool grounded = true;            // at most one wheel off the ground (a planar body)
  // J the vehicle drew over that tick (nav_plant::energy_step: from its fuel or battery, net of
  // regeneration, or its shaft work without a source); 0 at start()
  double energy = 0.0;
  // J its tires lost to slip over that tick (the ledger's `tire`); 0 at start()
  double tire = 0.0;
};

// Records every agent of `sim`, which drives `plant`. Both must outlive the log.
class plant_log {
public:
  plant_log(const cvc::nav::sim_world &sim, nav_plant &plant);
  void start();  // before the first step: the agents' state, no command yet
  void record(); // after each sim.step(): the state reached and the command held
  int agents() const { return static_cast<int>(_runs.size()); }
  double dt() const { return _dt; }
  const std::vector<std::vector<planar_sample>> &runs() const { return _runs; }

private:
  planar_sample sample(int i) const;
  const cvc::nav::sim_world &_sim;
  nav_plant &_plant;
  double _dt = 0.0, _accel_si = 1.0;
  std::vector<double> _vmax;
  std::vector<double> _tire; // each agent's ledger `tire` at its last sample
  std::vector<std::vector<planar_sample>> _runs;
};

// ── the model's residual ────────────────────────────────────────────────────────────

struct residual_bin {
  double lo_g = 0.0, hi_g = 0.0; // |lateral acceleration| / g; the last bin is open (hi infinite)
  std::uint64_t ticks = 0;
  double position_rms_m = 0.0, position_max_m = 0.0; // the rear axle
  double heading_rms_rad = 0.0, heading_max_rad = 0.0;
  double speed_rms_mps = 0.0, speed_max_mps = 0.0; // vx
  double lateral_rms_mps = 0.0;                    // vy at the centre of mass (dynamic only)
  double yaw_rate_rms = 0.0;                       // r (dynamic only)
};

struct plant_residuals {
  std::vector<residual_bin> dynamic, kinematic;
  // Ticks left out, by why (the first that applies): a static contact; a wheel off the ground at
  // either end; an impact (a horizontal acceleration past kImpactG, more than tires give: another
  // vehicle); the vehicle at rest at both ends.
  std::uint64_t skipped_contact = 0, skipped_airborne = 0, skipped_impact = 0, skipped_rest = 0;
  std::uint64_t skipped() const {
    return skipped_contact + skipped_airborne + skipped_impact + skipped_rest;
  }
};

// The horizontal acceleration (g) past which a tick is an impact, not tire forces.
constexpr double kImpactG = 1.5;

// The one-tick residuals of diff::dyn_plant (the vehicle from dyn_bike_si_of(d, mu), in SI) and
// of the kinematic bicycle (wheelbase d.wheelbase(), the commanded steer applied at once, its speed
// within [-vmax / 4, vmax] as sim_world's reference has it) over every tick of `runs`, both given
// the command as nav sent it. `bins_g` are the bins' upper edges, ascending. The ticks
// plant_residuals lists are skipped (at rest: |vx| < 0.2 m/s at both ends).
plant_residuals one_tick_residuals(const std::vector<std::vector<planar_sample>> &runs, double dt,
                                   const vehicle_desc &d, double mu = 0.9, int nsub = 1,
                                   const std::vector<double> &bins_g = {0.1, 0.2, 0.3, 0.5});

// The same with the dynamic model built from `si` (SI) and the kinematic bicycle's wheelbase and
// steering lock given: for a vehicle other than dyn_bike_si_of's, such as a fitted one.
plant_residuals one_tick_residuals(const std::vector<std::vector<planar_sample>> &runs, double dt,
                                   const cvc::nav::dyn_bike_si &si, double wheelbase,
                                   double max_steer, int nsub = 1,
                                   const std::vector<double> &bins_g = {0.1, 0.2, 0.3, 0.5});

// ── fitting the dynamic model to the plant ───────────────────────────────────────────

// The dynamic model's mean squared one-tick residual in the body twist over the ticks
// one_tick_residuals keeps: (vx error)^2 + (vy error)^2 + (L x yaw-rate error)^2, in (m/s)^2,
// L its wheelbase. `ticks`, when given, receives how many ticks it averaged.
double dyn_bike_cost(const std::vector<std::vector<planar_sample>> &runs, double dt,
                     const cvc::nav::dyn_bike_si &si, double max_steer, int nsub = 1,
                     std::uint64_t *ticks = nullptr);

// dyn_bike_si_of(d, mu) with the knobs a spec sheet does not give fitted to the plant by
// coordinate descent on dyn_bike_cost: the drive and steering lags (s), and factors on the tires'
// stiffness B (front, rear), on the yaw inertia, and on drag and rolling resistance together (a
// driver model that tracks the commanded acceleration makes up for them: 0). dyn_bike_si_of's own
// values are unchanged: this measures what the plant wants. Grids: lags 0-0.3 s, the resistance
// factor 0-1, the others 0.1-2.1.
struct dyn_bike_fit {
  cvc::nav::dyn_bike_si si;                  // the fitted vehicle
  double cost_default = 0.0, cost_fit = 0.0; // dyn_bike_cost before and after
  std::uint64_t ticks = 0;
  double drive_lag = 0.0, steer_lag = 0.0;
  double tire_b_front_factor = 1.0, tire_b_rear_factor = 1.0, yaw_inertia_factor = 1.0;
  double resistance_factor = 1.0;
};
dyn_bike_fit fit_dyn_bike(const std::vector<std::vector<planar_sample>> &runs, double dt,
                          const vehicle_desc &d, double mu = 0.9, int nsub = 1, int sweeps = 2);

// ── fitting the power model to the ledger (P8c) ──────────────────────────────────────

// One run's energy: what the plant's ledger measured (planar_sample::energy summed) and what the
// power model gives over the same ticks.
struct power_run {
  double measured_J = 0.0, model_J = 0.0;
  double distance_m = 0.0; // the centre of mass's path
  double seconds = 0.0;
};

struct power_fit {
  cvc::nav::dyn_power_si si; // the model (evaluate_power: the one given)
  // The tire term's calibration (fit_power, when the runs carry the ledger's `tire`): the ledger's
  // tire J per J of diff::dyn_tire_power, lateral and longitudinal, and the r^2 of the ticks' tire
  // energy it gives. si.tire_lat and si.tire_lon are these times the source's price of a tire J.
  double tire_cal_lat = 0.0, tire_cal_lon = 0.0, tire_r2 = 0.0;
  std::uint64_t ticks = 0;
  double r2 = 0.0;          // over the ticks' energies
  double total_error = 0.0; // (sum model - sum measured) / sum measured, over every run
  // The worst run's |model - measured| / measured, over runs that drew at least 1% of the mean.
  double worst_run_error = 0.0;
  std::vector<power_run> runs;
};

// Fits nav::dyn_power_si (detail/dyn_power.h) to the ledger in `runs` by non-negative least
// squares on the ticks' energies (k_neg's sign held <= 0). When the runs carry the ledger's tire
// term (planar_sample::tire), the tire term is fitted first: the dynamic vehicle's own tire
// dissipation at each tick's state (diff::dyn_tire_power, from `dyn`'s tires), lateral and
// longitudinal, scaled to the ledger's `tire` by non-negative least squares. Its sum then joins
// the energy fit as one more column, whose coefficient is what the source pays per tire J. The
// model reads, each tick, the drive force per unit mass the dynamic vehicle `dyn` (nullptr:
// nav_plant::dyn_bike_si_of(d)) needs to make the motion the plant made (the measured acceleration
// less dyn's drag, rolling resistance and grade), and the speed vx the tick ended at: the trainer's
// rollout carries the same two (dyn_state::a, vx). Every tick counts, contacts and impacts
// included: the run totals are the ledger's own.
power_fit fit_power(const std::vector<std::vector<planar_sample>> &runs, double dt,
                    const vehicle_desc &d, const cvc::nav::dyn_bike_si *dyn = nullptr);

// Runs logged at one tick dt (s), for fitting several scenes whose ticks differ at once.
struct power_runs {
  std::vector<std::vector<planar_sample>> runs;
  double dt = 0.0;
};
power_fit fit_power(const std::vector<power_runs> &sets, const vehicle_desc &d,
                    const cvc::nav::dyn_bike_si *dyn = nullptr);

// The same report for a model already fitted, such as on held-out runs.
power_fit evaluate_power(const std::vector<std::vector<planar_sample>> &runs, double dt,
                         const vehicle_desc &d, const cvc::nav::dyn_power_si &si,
                         const cvc::nav::dyn_bike_si *dyn = nullptr);

// ── equal-length flights for a plant-learning dataset ──────────────────────────────

// A row: the pose, the body twist and the two inputs.
//   0 x, 1 y, 2 psi, 3 vx, 4 vy, 5 r, 6 the drive input (drive), 7 the steering input.
// Row k's inputs acted from t_k-1 to t_k (row 0's from before the flight; unused by a model).
constexpr int kPlanarWidth = 8;

enum class steer_input {
  actual, // the mean of the road-wheel angle at the interval's two ends
  command // nav's command, held over the interval
};

struct flight_set {
  std::vector<double> data; // [flights * rows * kPlanarWidth], row-major
  std::size_t flights = 0, rows = 0;
  std::vector<int> agent, first; // each flight's agent and first sample
  std::size_t dropped = 0;       // windows left out: a contact, an impact, a wheel off the
                                 // ground, or barely moving
  double peak_lateral_g = 0.0;   // the largest |a_lat| / g in the kept flights
};

// Every window of `rows` samples, `stride` apart, of every run. A window is kept when it has no
// static contact, no impact and no wheel off the ground, and the vehicle moves faster than
// `min_speed` (m/s) somewhere in it. The drive input is planar_sample::drive.
flight_set cut_flights(const std::vector<std::vector<planar_sample>> &runs, std::size_t rows,
                       std::size_t stride, double min_speed = 1.0,
                       steer_input steer = steer_input::actual);

// A little-endian float64 .npy file (format 1.0, C order). Throws std::runtime_error.
void write_npy(const std::string &path, const std::vector<double> &data,
               const std::vector<std::size_t> &shape);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_PLANT_LOG_H__

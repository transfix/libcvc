/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::calibrate_dyn4: the four-wheel model's books calibrated to a vehicle simulation's
// energy ledger (docs/NAV_TRAINING.md, "Training on Jolt"), so the model the Jolt rollout
// differentiates moves energy into motion as the simulation does.
//
//   std::vector<cvc::nav::dyn4_ledger_tick> ticks =
//       cvc::nav::record_plant_ticks(scene, cfg, plant, 64, 200, seed); // coef_train.h
//   const cvc::nav::dyn4_calibration cal = cvc::nav::calibrate_dyn4(cfg.dyn4, veh, 0.9f, ticks);
//   cfg.dyn4 = cvc::nav::calibrated(cfg.dyn4, cal);
//   cfg.dyn4_price = cal.price;
//
// Each tick, the model's power lines (diff::dyn4_ph_lines) are evaluated at the simulation's own
// states: the trapezoid between the tick's two ends, on flat ground of friction mu, the start with
// the actuators' outputs the tick applied (as dyn4_plant_books has it). They are set against the
// simulation's ledger over the tick (plant_energy_lines), line by line, and the model's one-tick
// step from the simulation's state against the state the simulation reached:
//   drag, rolling, damper   their coefficients (drag, c_rr, both axles' c), which the lines are
//            linear in: each the ratio of the simulation's total to the model's;
//   drive, tire   the drive's answer to the command (dyn4_params::drive_gain, brake_gain,
//            resist_comp) and its lag tau_drive, what turns the command into motion, and the
//            tires' stiffness B on both axles, how they answer slip: together (the drive's force
//            takes from what the friction ellipse leaves the tires), by least squares
//            (Gauss-Newton, each step halved until it lowers the cost) on the one-tick twist (vx,
//            vy and the yaw rate times the wheelbase), the model's actuator re-run on the commands
//            with each try. The tires' slip loss is then set against the ledger's tire line, which
//            the model's lateral slip alone does not account for: it has no wheel spin. The line is
//            split by non-negative least squares between that loss (tire_lat) and the
//            longitudinal slip a wheel's force costs (dyn4_slip_proxy: sum_i Fx_i^2 u_i / (mu_i
//            Fz_i), tire_lon); the books carry both, and the longitudinal loss, behind the contact
//            patch, on the port as well. Neither changes the model's dynamics;
//   drawn    the price (dyn4_energy_price): the source's energy per unit of the model's positive
//            port power and its idle power, by non-negative least squares on what the source paid.
// The fits repeat over `rounds`, as none is independent of the others. A tick with a contact, or
// at rest at both ends, is left out. dyn4_books_of reports the books on other ticks, held out.

#ifndef __CVC_NAV_DYN4_LEDGER_H__
#define __CVC_NAV_DYN4_LEDGER_H__

#include <cstdint>
#include <cvc/nav/rollout_plant.h>
#include <vector>

namespace cvc {
namespace nav {

// One tick of one vehicle: the states it started and ended at (rollout_plant::state), the
// command it held, and its ledger over the tick. `first`: the first tick after a reset (the
// model's actuator starts from rest there); ticks of one vehicle are in order.
struct dyn4_ledger_tick {
  diff::dyn4_state<float> s0, s1;
  float delta_cmd = 0.0f, a_cmd = 0.0f;
  plant_energy_lines lines;
  int agent = 0;
  bool first = false;
};

// A line's totals over the ticks, the simulation's and the model's.
struct dyn4_line_totals {
  double plant = 0.0, model = 0.0;
  // (model - plant) / |plant|; 0 when both are 0.
  double error() const;
};

// The model's books against the simulation's, over the ticks a calibration keeps.
struct dyn4_books {
  std::uint64_t ticks = 0;
  std::uint64_t skipped_contact = 0, skipped_rest = 0;
  dyn4_line_totals port, tire, drag, rolling, damper;
  dyn4_line_totals dissipated; // tire + drag + rolling + damper
  dyn4_line_totals stored;     // the simulation's stored change against the model's H
  dyn4_line_totals drawn;      // what the source paid against dyn4_step_energy at the price
  double drawn_r2 = 0.0;       // over the ticks' drawn energy
  // The model's one-tick residual: its step from the simulation's state under the tick's command
  // against the state the simulation reached, the rms of (vx, vy, wheelbase x r) error (the
  // frame's speed).
  double twist_rms = 0.0;
};

struct dyn4_calibration {
  // The model's drive (dyn4_params' own drive_gain, brake_gain and resist_comp, set, and its lag:
  // < 0 the model's own), and factors on its tires' B, drag, c_rr and dampers. The defaults leave
  // a model as it is.
  float drive_gain = 1.0f, brake_gain = 1.0f, resist_comp = 0.0f, tau_drive = -1.0f;
  double tire = 1.0, drag = 1.0, rolling = 1.0, damper = 1.0;
  float c_f = 0.0f, c_r = 0.0f; // the dampers' rates, the model's times `damper` (0: the model's)
  // The books' tire line: tire_lat x the model's lateral slip loss + tire_lon x dyn4_slip_proxy.
  double tire_lat = 1.0, tire_lon = 0.0;
  dyn4_energy_price price;
  dyn4_books before, after; // on the ticks it was fitted to: the given model (no price), and the
                            // calibrated one
};

// Fits the model p to the ticks; see the header comment. Throws std::invalid_argument when no
// tick is left to fit.
dyn4_calibration calibrate_dyn4(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                                const std::vector<dyn4_ledger_tick> &ticks, int rounds = 4);
// p with the calibration's drive, factors and damper rates applied.
diff::dyn4_params calibrated(const diff::dyn4_params &p, const dyn4_calibration &c);
// The books of calibrated(p, c) at c.price on the ticks, the actuator re-run on their commands.
// A default dyn4_calibration gives p's own books.
dyn4_books dyn4_books_of(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                         const dyn4_calibration &c, const std::vector<dyn4_ledger_tick> &ticks);

// The model's lines over one step from s0 to s1: the trapezoid of dyn4_ph_lines between them (s0
// with s1's actuators, which the step applied), its H's change as `stored`, and the rest of its
// books (lever, the integrator's error) as `residual`. drawn is left 0, contact false.
plant_energy_lines dyn4_tick_lines(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                                   const diff::dyn4_state<float> &s0,
                                   const diff::dyn4_state<float> &s1);
// The longitudinal slip a wheel's force costs, up to the tire's stiffness: sum_i Fx_i^2 u_i /
// (mu_i Fz_i), a power, at the state.
double dyn4_slip_proxy(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                       const diff::dyn4_state<double> &s);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DYN4_LEDGER_H__

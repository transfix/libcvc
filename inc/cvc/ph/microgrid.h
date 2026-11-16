/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph microgrids: DC networks of distributed generation units (DGUs) and RL lines, built on
// cvc::ph composition (compose.h), as in the electrical studies of C-PHAST (Shubham Bhardwaj and
// Chandrajit Bajaj).
//
// A DGU is a canonical_law with n = 1:
//   - state x = (q, phi): its capacitor's charge and its filter inductor's flux;
//   - efforts: the bus voltage v = dH/dq and the inductor current i = dH/dphi = phi / L;
//   - loss: the filter resistance R;
//   - inputs u = (I_in, V_src): the current injected at its bus (minus its load) and its source
//     voltage, so dq/dt gains I_in and dphi/dt gains V_src;
//   - one coupling port, its bus voltage.
//
// A line is a linear_law with state phi_e (current phi_e / L_e, loss R_e) and one port, its
// current. A network pairs bus voltages with line currents through its incidence matrix (-1 at a
// line's sending bus, +1 at its receiving bus), so dq_bus += A i_line and dphi_line -= A^T v_bus.
//
// The readout (composite::readout) is each block's efforts: (v, i) per DGU, then i_e per line.
// storage_from_efforts (compose.h) maps a physical state back to charges and fluxes.

#ifndef __CVC_PH_MICROGRID_H__
#define __CVC_PH_MICROGRID_H__

#include <cvc/ph/compose.h>
#include <string>
#include <vector>

namespace cvc {
namespace ph {

// A DGU with a linear capacitor C (F), filter inductance L (H) and resistance R (ohm).
law_ptr linear_dgu(double C, double L, double R,
                   conservative_scheme scheme = conservative_scheme::leapfrog);
// A DGU whose capacitor's charge is q = sum_j charge[j] v^j (an odd increasing polynomial; see
// canonical_spec::capacitor); conserved by implicit midpoint.
law_ptr nonlinear_dgu(vecd charge, double L, double R);
// The electrical studies' reference DGU: q = 2.2 (v + v^3 / 3) C, L = 1.8 H, R = 1.2 ohm.
law_ptr reference_dgu();
// An RL line: inductance L (H), resistance R (ohm).
law_ptr rl_line(double R, double L);

// A line from bus `from` (sending) to bus `to` (receiving).
struct line_spec {
  int from = 0, to = 0;
  double R = 0.0, L = 1.0;
};

// A network: blocks dgu0..dgu{n-1}, then line0..line{m-1}, wired by the lines' incidence.
struct microgrid {
  composite system;
  std::vector<int> buses, lines; // block indices
};
microgrid make_microgrid(const std::vector<law_ptr> &dgus, const std::vector<line_spec> &lines);
// The same DGU law at every bus.
microgrid make_microgrid(law_ptr dgu, int buses, const std::vector<line_spec> &lines);

// The physical state (v, i per DGU, then i_e per line), in the readout's order.
// storage_from_efforts (compose.h) maps it back to charges and fluxes.
vecd physical_state(const microgrid &g, const vecd &x);

// Errors between two trajectories of the same network (row 0, the shared start, left out): the
// RMSE over every future sample of the bus voltages, the DGU currents and the line currents, and
// the error in the largest absolute line current over the rollout.
struct grid_errors {
  double voltage_rmse = 0, dgu_current_rmse = 0, line_current_rmse = 0;
  double peak_line_current_a = 0, peak_line_current_b = 0, peak_line_current_error = 0;
};
grid_errors compare_trajectories(const microgrid &g, const std::vector<vecd> &a,
                                 const std::vector<vecd> &b);
// The largest absolute line current over a trajectory (row 0 left out).
double peak_line_current(const microgrid &g, const std::vector<vecd> &x);

} // namespace ph
} // namespace cvc

#endif

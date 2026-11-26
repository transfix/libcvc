/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph mechanics: C-PHAST's mechanical components (Shubham Bhardwaj and Chandrajit Bajaj;
// docs/PH_COMPOSE.md, "Mechanics"; Phase 5 of docs/roadmap/PH-COMPOSE-ROADMAP.md).
//
// A mechanical block is a canonical_law (compose.h): x = (q, p), H = V(q) + 1/2 p . M(q)^-1 p,
// R = diag(0, D(q)). Its ports are momentum ports, B = [0; B_p(q)]: identity columns for directly
// applied generalized forces (the arm's joints), or an attachment point's (dr/dq)^T (a foot,
// serial_chain). Algorithm 2 steps such a system with
//
//   transition_opts o;
//   o.order = split_order::inputs_outside; // input -> coupling -> local -> coupling -> input
//   o.coupling = coupling_scheme::cayley;  // each coupling half-step keeps its kinetic energy
//
// This file holds the pieces around that:
//
//   - the planar arm of the count-transfer study (Appendix K.1): the simulator that makes its data
//     (phast's PlanarRobotArm), the known per-joint law, the dense count-transfer coupling
//     (Eq. 208) and its fit;
//   - the state from positions alone (Appendix A.3): finite-difference velocities (Eq. 65) and
//     their canonicalization (p = M qdot), or a fit of the window's start through the model;
//     then the takeover rollout (Eq. 67);
//   - the Unitree Go2's legs as blocks, with their feet as attachment ports (the Menagerie's
//     go2.xml), and compliant contact on a port (contact_law, connect_damper).
//
// Every function here is a pure function of its arguments; nothing is process-global.

#ifndef __CVC_PH_MECHANICAL_H__
#define __CVC_PH_MECHANICAL_H__

#include <cstdint>
#include <cvc/ph/compose.h>
#include <string>
#include <vector>

namespace cvc {
namespace ph {

// ── the planar arm (Appendix K.1) ───────────────────────────────────────────────────────────────

// phast's planar arm in its interconnection setting (generate_robot_arm_interconnection_data):
// homogeneous joints of inertia `inertia` and viscous damping `damping`, the separable gravity
// V_i = gravity (1 - cos q_i), joint limits by a soft clamp, and the hidden skew coupling
// u_coup = -C_true qdot with C_true = coupling x chain / sqrt(n) (chain: +1 above the diagonal, -1
// below). Inputs are per-joint sinusoids tau_i = torque_scale sin(2 pi f_i t + phi_i), with f_i
// = torque_freq (0.5 + U(0, 1)) and phi_i ~ U(0, 2 pi); the start is init_scale N(0, 1) for every
// angle and velocity.
struct arm_params {
  double dt = 0.02;
  double gravity = 9.81;
  double inertia = 1.0, damping = 0.1;
  double coupling = 1.0;
  bool normalize = true; // C_true / sqrt(n) (phast's inv_sqrt_n); false: no normalization
  double limit = 2.5, limit_beta = 10.0;
  double torque_scale = 2.0, torque_freq = 0.5, init_scale = 0.5;
};

// C_true (n x n, row-major).
vecd arm_true_coupling(int n, const arm_params &p);

// One step of the simulator: q and qdot (n each) in place, tau (n) held. phast's PlanarRobotArm
// .step: a Cayley half-step of the coupling (qdot <- (I + c A)^-1 (I - c A) qdot, A = C / inertia,
// c = dt / 4), semi-implicit Euler (qdot += dt (tau - damping qdot - gravity sin q) / inertia,
// q += dt qdot), the closing Cayley half-step, then the soft clamp of q and qdot <- (q_new - q) /
// dt. That last assignment supersedes the closing half-step, as in phast, so the coupling acts for
// half of each step: the data carry C_true / 2, and that is what a fit recovers.
void arm_true_step(int n, const arm_params &p, const vecd &C, double *q, double *qdot,
                   const double *tau);

// A trajectory of `steps` + 1 states: q[k] and qdot[k] (n each) at step k; tau[k] (n) is held over
// step k (steps rows).
struct arm_trajectory {
  std::vector<vecd> q, qdot, tau;
};

std::vector<arm_trajectory> arm_dataset(int n, int count, int steps, std::uint64_t seed,
                                        const arm_params &p = {});

// The KNOWN per-joint law of Table 10: a canonical 1-DOF block with mass `inertia`, damping
// `damping`, the potential gravity (1 - cos q), one momentum port and one momentum input (the
// joint torque), stepped by leapfrog.
law_ptr arm_joint(const arm_params &p = {});

// The homogeneous count-transfer coupling (Eq. 208): C_hat(s) = ((U - U^T) restricted to the
// first s blocks) / sqrt(s), for U smax x smax (row-major) and s <= smax.
vecd count_coupling(const vecd &U, int smax, int s);

// n joints "j0" ... "j<n-1>" of one law, their port 0 paired by the skew C_hat (n x n; empty: no
// coupling).
composite arm_system(law_ptr joint, int n, const vecd &C_hat = {});

// Algorithm 2 for the arm: one step of p.dt, inputs outside, Cayley coupling.
transition_opts arm_transition(const arm_params &p = {});

// ── the state from positions (Appendix A.3) ─────────────────────────────────────────────────────

// Each canonical block's configuration coordinates (its first n state indices), in block order:
// what a joint encoder measures.
std::vector<int> position_channels(const composite &s);

// p = M(q) qdot block by block (the diagonal-mass canonicalizer); positions and velocities are in
// position_channels' order. Blocks with no configuration coordinates take a zero state.
vecd canonicalize(const composite &s, const vecd &q, const vecd &qdot);

enum class observer_kind {
  // Eq. 65 without the learned residual: qdot at the last frame is (q_t - q_{t-1}) / dt, then
  // canonicalize. phast adds a temporal convolutional network's correction (Eq. 66), which is not
  // ported; window_fit takes its place.
  finite_difference,
  // The window's start fit through the frozen transition to every observed frame (fit_history in
  // state space, query.h), then run to the last frame: the model's own estimate, no training.
  window_fit,
};

struct observer_options {
  observer_kind kind = observer_kind::window_fit;
  // window_fit's prior: the start's positions about the first frame, its momenta about zero.
  double position_scale = 0.05, momentum_scale = 3.0;
  double noise_floor = 0.002; // fit_history's weight floor
  int max_evals = 40;
};

// The latent state at the last of positions.size() frames (rows of position_channels values),
// with inputs[k] (input_dim) held between frames k and k + 1 (positions.size() - 1 rows).
vecd observe(const composite &s, const transition_opts &o, const std::vector<vecd> &positions,
             const std::vector<vecd> &inputs, const observer_options &obs = {});

// The takeover rollout (Eq. 67): from x, the positions after each of inputs.size() steps.
std::vector<vecd> takeover(const composite &s, const transition_opts &o, const vecd &x,
                           const std::vector<vecd> &inputs);

// ── the count-transfer study (Section 4.4, Table 10) ────────────────────────────────────────────

struct takeover_options {
  int context = 5;  // observed frames (L_h)
  int horizon = 50; // predicted steps (N_pred)
  int windows = 1;  // windows per trajectory, back to back from frame 0
  observer_options observer;
};

// The joint-position MSE of takeover predictions over every window of the trajectories (all of
// one joint count, the system's).
double takeover_mse(const composite &s, const transition_opts &o,
                    const std::vector<arm_trajectory> &data, const takeover_options &t);

// Fits the coupling of an arm of n_train joints to its trajectories: the skew entries of
// C_hat(n_train), by Levenberg-Marquardt on the takeover residuals (central differences). The
// window starts are observed with the current coupling and held through each fit, `rounds` times
// over. U is smax x smax with only its first n_train rows and columns set, so count_coupling(U,
// smax, s) re-instantiates the coupling at any s <= smax: entries beyond the training block are
// zero (they get no data), and the trained ones scale by sqrt(n_train / s).
struct coupling_fit_options {
  takeover_options takeover;
  int smax = 8;
  int rounds = 2;
  int max_evals = 30; // residual evaluations per round, besides the Jacobian's
  double jacobian_step = 1e-4;
  double ftol = 1e-10, xtol = 1e-10;
};

struct coupling_fit_result {
  vecd U; // smax x smax
  int smax = 0;
  double mse_start = 0; // the takeover MSE with no coupling
  double mse = 0;       // and with the fitted one
  int evals = 0;        // residual evaluations, besides the Jacobian's
};

coupling_fit_result fit_arm_coupling(law_ptr joint, int n_train,
                                     const std::vector<arm_trajectory> &data,
                                     const transition_opts &o, const coupling_fit_options &f = {});

// ── the Go2's legs ──────────────────────────────────────────────────────────────────────────────

enum class go2_leg { front_left, front_right, rear_left, rear_right };
const char *to_string(go2_leg leg);       // "FL", "FR", "RL", "RR", the Menagerie's prefixes
go2_leg go2_leg_of(const std::string &s); // throws std::invalid_argument

// The foot of a leg in the base frame: hip (abduction about x), thigh and calf (about y), from the
// Menagerie's go2.xml (unitree_go2, its "foot" geom's centre). Ports carry x, y and z.
serial_chain go2_foot(go2_leg leg);

// The diagonal of the leg's joint-space inertia at q (hip, thigh, calf; kg m^2), from the bodies'
// inertials in go2.xml and the joints' armature, the base held fixed.
vecd go2_leg_inertia(go2_leg leg, const vecd &q);

// A leg as one block: n = 3, mass the inertia's diagonal at q_ref (the home keyframe's 0, 0.9,
// -1.8 by default), the joints' viscous damping (go2.xml's 2 N m s / rad), no potential of its own
// (gravity and contact arrive through ports and inputs). Ports: the foot's x, y, z (force in,
// velocity out). Inputs: the three joint torques, then a force at the foot (x, y, z: the foot's
// load, say). Leapfrog.
struct go2_leg_options {
  vecd q_ref = {0.0, 0.9, -1.8};
  double damping = 2.0;
};
law_ptr go2_leg_law(go2_leg leg, const go2_leg_options &o = {});

// ── contact ─────────────────────────────────────────────────────────────────────────────────────

// Compliant contact on a normal port (an attachment point's z): adds a contact_law block of the
// stiffness (its state the compression, s = ground height - foot height while touching), pairs it
// with port `port` of block `block` so ds/dt = -(the port's velocity), and puts a Hunt-Crossley
// damper of coefficient `damping` x [s]_+ on that port. Returns the contact block. The contact's
// state is the caller's to set at the start.
int attach_contact(composite &s, int block, int port, double stiffness, double damping,
                   std::string name = {});

} // namespace ph
} // namespace cvc

#endif

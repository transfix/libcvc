/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's plant, potential and command, in C++: C2 of docs/roadmap/NAV-COPH-ROADMAP.md
// (§4.2), the twin of GRL-SNAM's grl_snam.coph.model:
//
//   coph_flow, coph_balance     the exact flow of q' = v, v' = u - d v under a force held over dt,
//                               and its energy balance (model.flow, model.balance).
//   coph_risk, coph_potential   the risk field and the potential V (model.risk, model.potential),
//   coph_grad_potential         and V's gradient in q, analytic where the Python uses autograd.
//   coph_shaping_parameters     progress shaping's alpha and its bound (model.shaping_parameters).
//   coph_command                -grad V - 1.4 v, scaled to the force limit (model.command).
//   coph_energy                 one agent's running energy account.
//
// The plant's closed forms are ill-conditioned at small damping: dt - I cancels to about d dt^2 /
// 2, and the heat sums terms of size |u|^2 / d. They reproduce the Python only with exp and expm1
// correctly rounded, as torch's are, and with the operations in the Python's order, uncontracted.
// coph_exp and coph_expm1 are therefore correctly rounded (std::expm1 is not: MSVC's misses at most
// of GRL-SNAM's plant cases), and coph_model.cpp is built with -ffp-contract=off. The potential
// uses cvc::algo::det's functions, which give the same bits on every platform.
//
// The templates are on the scalar so the same code can run on duals for replay and the damping
// Newton step (C3). They are defined in coph_model.cpp, which instantiates them, so they are
// compiled under its floating-point flags; this PR instantiates double.
//
// Only the demos' profile is implemented (D7): no local residual (the Wendland correction; the
// demos never set Stage.local_residual), and the shaping is 'legacy' or 'progress'.
//
// GRL-SNAM's golden (grl_snam.tools.coph_golden, committed as src/cvc/tests/data/coph_golden.json)
// pins the plant (G2), the potential, gradient and command (G3) and the allocation (G11,
// coph_allocation.h) at the FLOAT64 tier, rtol 1e-12 and atol 1e-12; its plant_rules and
// potential_rules texts state the formulas.

#ifndef CVC_NAV_COPH_MODEL_H
#define CVC_NAV_COPH_MODEL_H

#include <array>
#include <cvc/nav/coph_config.h>
#include <cvc/nav/coph_ledger.h>
#include <optional>
#include <span>

namespace cvc {
namespace nav {

// e^x and e^x - 1, correctly rounded (to nearest) wherever the result is a normal double: they are
// evaluated in double-double and rounded once, so they can miss only where e^x lies within about
// 2^-100 of a rounding boundary. Standard IEEE operations only, so the same bits on every
// platform.
double coph_exp(double x);
double coph_expm1(double x);

template <class T> using coph_vec2_of = std::array<T, 2>;
template <class T> using coph_state_of = std::array<T, 4>; // x, y, vx, vy

// model.flow: the state after dt under the force u held, with damping d > 0:
//   a = exp(-d dt), I = -expm1(-d dt) / d,
//   q_after = (q + I v) + ((dt - I) u) / d, v_after = a v + I u.
template <class T>
coph_state_of<T> coph_flow(const coph_state_of<T> &x, const coph_vec2_of<T> &u, const T &damping,
                           double dt);

// model.balance over the same step. residual = (kinetic_change - work) + heat: zero but for
// round-off, which at tiny damping is not small (the heat cancels).
struct coph_balance_terms {
  double work = 0;           // u . (q_after - q)
  double heat = 0;           // the energy the damping took
  double kinetic_change = 0; // (|v_after|^2 - |v|^2) / 2
  double residual = 0;
};
coph_balance_terms coph_balance(const coph_state &x, const coph_vec2 &u, double damping, double dt);

// What the potential reads of a stage: its RBF field (centres, weights, width) and its obstacles.
// The field and the geometry are frozen while the policy is evaluated.
struct coph_field_view {
  std::span<const coph_vec2> centers;
  std::span<const double> weights;
  double width = 0.8; // rbf_width
  std::span<const coph_shape> obstacles;
};

// The stage's field, with the configuration's rbf_width, and the obstacles the caller chooses: the
// routed controller passes the whole geometry store (navigation.py), the learner's own command and
// replay the stage's obstacles (learner.py). Views; the stage and the obstacles must outlive it.
coph_field_view coph_field_of(const coph_stage &stage, const coph_config &cfg,
                              std::span<const coph_shape> obstacles);

// Progress shaping's parameters (model.shaping_parameters). alpha and bound depend on neither q nor
// the policy, so a stage computes them once:
//   bound = policy_bounds[1] (exp(-0.5) sum |w| / (4 width) + sum_obstacles softplus(6 (r + 0.47))
//   / 3), alpha = min(1, 2 eta k ell / bound), 1 when bound = 0.
struct coph_shaping {
  double k = 1.0, ell = 0.5, eta = 0.5; // shaping_k, shaping_ell, shaping_eta
  double alpha = 1.0;
  double bound = 0.0;
};
coph_shaping coph_shaping_parameters(const coph_field_view &field, const coph_config &cfg);

// The shaping for cfg's shaping_mode: the parameters for 'progress', nullopt (legacy) otherwise.
std::optional<coph_shaping> coph_shaping_for(const coph_field_view &field, const coph_config &cfg);

// model.risk: softplus(sum_k w_k exp(-0.5 |(q - c_k) / width|^2)) / 4, with torch's softplus
// (linear above 20).
template <class T> T coph_risk(const coph_vec2_of<T> &q, const coph_field_view &field);

// model.potential, with s = |q - target|^2 and
//   shaping = policy[0] risk(q) + policy[1] sum_obstacles softplus(6 (0.35 - clearance))^2 / 36,
//   clearance = the obstacle's surface distance - 0.12 (the agent's radius):
//   legacy (progress = nullopt):  V = 0.5 s + shaping,
//   progress:                     V = 0.5 k s + ((alpha s) / (s + ell^2)) shaping.
template <class T>
T coph_potential(const coph_vec2_of<T> &q, const coph_vec2 &target, const coph_vec2_of<T> &policy,
                 const coph_field_view &field, const std::optional<coph_shaping> &progress);

// dV/dq, analytic. Not defined where V is not smooth: at a slab's segment ends or on its core, or
// at a disk's centre.
template <class T>
coph_vec2_of<T> coph_grad_potential(const coph_vec2_of<T> &q, const coph_vec2 &target,
                                    const coph_vec2_of<T> &policy, const coph_field_view &field,
                                    const std::optional<coph_shaping> &progress);

// model.command: proposed = -grad V - 1.4 v; applied = proposed min(1, limit / max(|proposed|,
// 1e-12)).
template <class T> struct coph_command_of {
  coph_vec2_of<T> applied;
  coph_vec2_of<T> proposed;
};
template <class T>
coph_command_of<T> coph_command(const coph_state_of<T> &x, const coph_vec2 &target,
                                const coph_vec2_of<T> &policy, const coph_field_view &field,
                                const std::optional<coph_shaping> &progress, double limit);

extern template coph_state_of<double> coph_flow<double>(const coph_state_of<double> &,
                                                        const coph_vec2_of<double> &,
                                                        const double &, double);
extern template double coph_risk<double>(const coph_vec2_of<double> &, const coph_field_view &);
extern template double coph_potential<double>(const coph_vec2_of<double> &, const coph_vec2 &,
                                              const coph_vec2_of<double> &, const coph_field_view &,
                                              const std::optional<coph_shaping> &);
extern template coph_vec2_of<double>
coph_grad_potential<double>(const coph_vec2_of<double> &, const coph_vec2 &,
                            const coph_vec2_of<double> &, const coph_field_view &,
                            const std::optional<coph_shaping> &);
extern template coph_command_of<double>
coph_command<double>(const coph_state_of<double> &, const coph_vec2 &, const coph_vec2_of<double> &,
                     const coph_field_view &, const std::optional<coph_shaping> &, double);

// One agent's energy account (§4.2): the plant's balance summed step by step, the shaping jumps
// the learner's commits make (V_new(x) - V_old(x), C3), and the work of the force the car did not
// take (the allocation's e, §3.3.2). The plant_ledger items that show it arrive with the fleet
// driver, which attaches CoPH to a vehicle (C5, D11).
struct coph_energy {
  double work = 0;
  double heat = 0;
  double kinetic_change = 0;
  double residual = 0;         // the sum of each step's residual
  double shaping_jumps = 0;    // the sum of V_new(x) - V_old(x) at each commit
  double unallocated_work = 0; // the sum of e . (q_after - q): e is held over the step, as u is
  long long steps = 0;

  void add(const coph_balance_terms &b);
  void add_shaping_jump(double jump) { shaping_jumps += jump; }
  void add_unallocated(const coph_vec2 &e, const coph_vec2 &q_before, const coph_vec2 &q_after);
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_MODEL_H

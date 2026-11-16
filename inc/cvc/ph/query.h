/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph queries: compare proposed changes to a system before making them, after C-PHAST
// (Shubham Bhardwaj and Chandrajit Bajaj; docs/roadmap/PH-COMPOSE-ROADMAP.md, Phase 3).
//
// A query has four steps, with every component law fixed:
//
//   1. Estimate the actual system's present state from its measured history (fit_history): fit
//      the physical state at the start of the window, through the frozen transition, to the
//      measured channels (voltages only, say), and run it forward to the query time.
//   2. Give each candidate (a rewiring, an added component, other inputs) its start
//      (assign_state, compose.h): kept components keep their estimated state, added ones take
//      supplied states.
//   3. Roll each candidate out and read its consequences per step (consequences): the readout,
//      the stored energy, the loss and external powers, and the step's work, loss and residual.
//   4. Summarize each candidate as an action card (make_card: peak current, voltage range,
//      accumulated loss ...) and rank them with error bounds (rank_candidates).
//
// The ranking is certified only where the bounds allow it. A candidate is admitted when every limit
// holds after its error bound is added, g + dg <= 0; a is ordered before b only when a's largest
// possible score is below b's smallest, J(a) + dJ(a) < J(b) - dJ(b). An inconclusive test gives
// no verdict, and an empty admitted set gives none either (abstain). The bounds are the caller's:
// sampled_bounds gives sampled ones (an ensemble of starting states and laws plus the
// transition's own error against a fine reference), which are evidence, not a proof.

#ifndef __CVC_PH_QUERY_H__
#define __CVC_PH_QUERY_H__

#include <cstdint>
#include <cvc/ph/compose.h>
#include <string>
#include <vector>

namespace cvc {
namespace ph {

// ── 1. the state from a measured history ────────────────────────────────────────────────────────

// A measured history of `steps` integration steps (steps = inputs.size()).
struct history {
  std::vector<vecd> inputs;      // inputs[k] (input_dim) held over step k
  std::vector<int> sample_steps; // the step (0 .. steps) of each sample, increasing
  std::vector<int> channels;     // the readout indices measured (composite::readout's order)
  std::vector<vecd> samples;     // one row of channels.size() values per sample step
};

struct history_fit_options {
  transition_opts step; // the frozen transition the window is run through
  vecd prior_mean,
      prior_scale;    // the window start's readout (efforts): mean and scale, state_dim each
  double noise = 0.0; // the measurement noise's standard deviation
  double noise_floor = 0.002;  // each measured channel is weighted by 1 / max(noise, noise_floor)
  int max_evals = 20;          // residual evaluations, besides the Jacobian's
  double jacobian_step = 1e-4; // central differences in the prior-scaled coordinates
  double ftol = 1e-8, xtol = 1e-8, gtol = 1e-8;
  double rank_rtol = 1e-8; // singular values above rank_rtol x the largest count toward the rank
};

// The fit minimizes, over xi (state_dim prior-scaled efforts at the window start, readout =
// prior_mean + prior_scale xi),
//
//   sum over samples |(readout_channels(x_k(xi)) - sample) / max(noise, floor)|^2 + |xi|^2,
//
// where x_0(xi) = storage_from_efforts(prior_mean + prior_scale xi) and x_{k+1} = step(x_k,
// inputs[k]), by Levenberg-Marquardt from xi = 0 with a central-difference Jacobian.
struct history_fit_result {
  vecd xi;
  vecd x_start, x_now;  // the window start's state, and the state after every step (the query time)
  bool success = false; // stopped on a tolerance (ftol, xtol or gtol), not the budget
  double optimality = 0; // |J^T r|_inf at the end
  int rank = 0;          // the weighted data Jacobian's numerical rank (the prior's rows left out)
  int evals = 0;         // residual evaluations
  double cost = 0;       // 1/2 |r|^2 at the end
  // The Laplace approximation of xi's posterior, (J^T J)^-1 at the end (state_dim x state_dim,
  // row-major; empty when J^T J is singular): the spread posterior_states draws from.
  vecd xi_covariance;
  // A decision admits the estimate only when the fit succeeded, is stationary and the data
  // determine every coordinate: success, optimality <= max_optimality, rank == full_rank.
  bool admitted(int full_rank, double max_optimality = 1e-4) const {
    return success && optimality <= max_optimality && rank == full_rank;
  }
};
history_fit_result fit_history(const composite &s, const history &h, const history_fit_options &o);

// States at the query time drawn from the fit's posterior: xi ~ N(xi_hat, sigmas^2 covariance),
// each mapped to a start (storage_from_efforts) and run through the history's inputs. Draws whose
// start no state reaches are skipped. Needs xi_covariance.
std::vector<vecd> posterior_states(const composite &s, const history &h,
                                   const history_fit_options &o, const history_fit_result &fit,
                                   int count, std::uint64_t seed, double sigmas = 1.0);

// The readouts at the history's sample steps, from a start state (its forward model).
std::vector<vecd> history_readouts(const composite &s, const history &h,
                                   const transition_opts &step, const vecd &x_start);

// ── 3. consequences ─────────────────────────────────────────────────────────────────────────────

// One step's consequences, read at its end state x+ with the step's input u held.
struct consequence {
  vecd readout;              // every block's efforts at x+
  double stored = 0;         // H(x+)
  double loss_power = 0;     // e . R e at x+
  double external_power = 0; // e . G u at x+
  double work = 0;           // the step's external work (trapezoid)
  double loss = 0;           // the step's dissipated energy (trapezoid)
  double residual = 0;       // dH - work + loss
};

// The consequences of a rollout's steps (r.x.size() - 1 of them). u holds one input row held over
// every step, or one row per step, as rollout takes it.
std::vector<consequence> consequences(const composite &s, const rollout_result &r, const vecd &u);

// ── 4. action cards and the ranking ─────────────────────────────────────────────────────────────

enum class summary {
  max_abs,      // the largest |value| of the channels over the steps
  min,          // the smallest value of the channels
  max,          // the largest value of the channels
  final_value,  // the channels' last values' largest |value|
  total_loss,   // the sum of the steps' dissipated energy
  total_work,   // the sum of the steps' external work
  max_residual, // the largest |step residual|
};

// One line of an action card: a summary over some readout channels (ignored for the energy lines).
struct card_entry {
  std::string name;
  summary kind = summary::max_abs;
  std::vector<int> channels;
};

struct action_card {
  std::string candidate;
  std::vector<std::string> names;
  vecd values;                                 // one per entry, in the spec's order
  double value(const std::string &name) const; // throws std::out_of_range for an unknown entry
};

action_card make_card(const std::string &candidate, const std::vector<consequence> &c,
                      const std::vector<card_entry> &spec);

// A limit on one card entry: value <= bound (upper) or value >= bound.
struct card_limit {
  std::string entry;
  double bound = 0;
  bool upper = true;
};

// The score J = sum_k weight_k * value_k / scale_k over the named card entries (a scale of 0 is 1).
struct card_objective {
  std::vector<std::string> entries;
  vecd weights, scales;
};

// The error bound of every card entry for one candidate (one per card entry, in its order).
struct card_bounds {
  vecd delta;
};

enum class feasibility { admitted, rejected, unresolved };
enum class verdict { certified, inconclusive, abstain };

struct ranking {
  std::vector<feasibility> feasible; // per candidate: every limit holds with its bound (admitted),
                                     // one fails with it (rejected), or neither
  vecd score, score_bound;           // J and dJ per candidate
  // order[a][b] = +1 when a is certified better (J(a) + dJ(a) < J(b) - dJ(b)), -1 when b is,
  // 0 when the bounds overlap.
  std::vector<std::vector<int>> order;
  int best = -1; // the admitted candidate with the lowest score, -1 if none
  // certified: best is certified better than every other candidate not rejected (admitted or
  // unresolved); inconclusive: some bound overlaps; abstain: no candidate is admitted.
  verdict outcome = verdict::abstain;
  double suboptimality = 0; // the best's physical cost is within this of the admitted set's
                            // best: 2 max dJ over the admitted candidates
};

ranking rank_candidates(const std::vector<action_card> &cards,
                        const std::vector<card_bounds> &bounds,
                        const std::vector<card_limit> &limits, const card_objective &objective);

// ── sampled error bounds ────────────────────────────────────────────────────────────────────────

// Sampled bounds for one candidate's card: the largest |card - nominal card| over
//   - every start in `starts` (e.g. draws around the estimated state), rolled out on `s`;
//   - every alternative law set in `alternatives` (the same wiring with perturbed laws) from x0;
//   - the transition itself against RK4 at dt / refine (the numerical error), from x0;
// summed, entry by entry. They hold for what was sampled; they prove nothing about what was not.
card_bounds sampled_bounds(const composite &s, const transition_opts &o, const vecd &x0,
                           const vecd &u, int steps, const std::vector<card_entry> &spec,
                           const std::vector<vecd> &starts,
                           const std::vector<const composite *> &alternatives = {},
                           int refine = 16);

const char *to_string(verdict v);
const char *to_string(feasibility f);

} // namespace ph
} // namespace cvc

#endif

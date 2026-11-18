/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph queries (inc/cvc/ph/query.h): estimating a network's state from its bus voltages alone,
// the consequences and action cards of candidate futures, and the certified ranking.
//
// The network is the electrical studies' four-DGU chain with their reference DGU laws. The truth is
// RK4 at dt / 16 and the model is the split step; the laws are known, so the gaps are estimation
// and integration error. The history is one second: 21 voltage samples, one every 5 steps of 0.01
// s.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <cvc/ph/microgrid.h>
#include <cvc/ph/query.h>
#include <cvc/ph/rng.h>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

using namespace cvc::ph;

std::string sci(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3e", v);
  return b;
}

double uniform(rng &g, double lo, double hi) { return lo + (hi - lo) * g.uniform(); }

const std::vector<line_spec> kChain = {{0, 1, 0.3, 0.8}, {1, 2, 0.4, 0.8}, {2, 3, 0.5, 0.8}};

vecd sample_physical(rng &g, int buses, int lines) {
  vecd e;
  for (int i = 0; i < buses; ++i) {
    e.push_back(uniform(g, 0.5, 1.3));
    e.push_back(uniform(g, -0.25, 0.35));
  }
  for (int k = 0; k < lines; ++k)
    e.push_back(uniform(g, -0.1, 0.1));
  return e;
}

vecd sample_inputs(rng &g, int buses) {
  vecd u;
  for (int i = 0; i < buses; ++i) {
    u.push_back(-uniform(g, 0.05, 0.2));
    u.push_back(uniform(g, 0.9, 1.2));
  }
  return u;
}

// A true history: RK4 at dt / 16 from x_true, voltages sampled every 5 steps over `steps`, with
// Gaussian noise of `noise` V. Returns the history and the true state at the query time.
struct truth {
  history h;
  vecd x_start, x_now;
};

truth make_history(const microgrid &m, const vecd &x_true, const vecd &u, int steps, double noise,
                   rng &g) {
  truth t;
  t.x_start = x_true;
  t.h.inputs.assign(steps, u);
  t.h.channels = voltage_channels(m);
  transition_opts fine;
  fine.scheme = step_scheme::rk4;
  fine.dt = 0.01 / 16;
  vecd x = x_true, e(m.system.state_dim());
  for (int k = 0; k <= steps; ++k) {
    if (k % 5 == 0) {
      m.system.readout(x.data(), e.data());
      vecd row;
      for (int c : t.h.channels)
        row.push_back(e[c] + noise * g.normal());
      t.h.sample_steps.push_back(k);
      t.h.samples.push_back(row);
    }
    if (k < steps)
      for (int j = 0; j < 16; ++j)
        step(m.system, fine, u.data(), x.data());
  }
  t.x_now = x;
  return t;
}

history_fit_options fit_options(const microgrid &m, double noise) {
  history_fit_options o;
  microgrid_prior(m, o.prior_mean, o.prior_scale);
  o.noise = noise;
  return o;
}

double max_err(const microgrid &m, const vecd &a, const vecd &b, const std::vector<int> &channels) {
  vecd ea(m.system.state_dim()), eb(ea.size());
  m.system.readout(a.data(), ea.data());
  m.system.readout(b.data(), eb.data());
  double d = 0;
  for (int c : channels)
    d = std::max(d, std::fabs(ea[c] - eb[c]));
  return d;
}

// ── 1. the state from bus voltages ─────────────────────────────────────────────────────────────

// The forecast from an estimated state against the truth from the true one, `steps` steps ahead.
grid_errors forecast_errors(const microgrid &m, const vecd &x_hat, const vecd &x_true,
                            const vecd &u, int steps) {
  transition_opts o;
  transition_opts fine;
  fine.scheme = step_scheme::rk4;
  fine.dt = 0.01 / 16;
  const rollout_result a = rollout(m.system, o, x_hat, u, steps),
                       b = rollout(m.system, fine, x_true, u, steps * 16);
  std::vector<vecd> truth;
  for (size_t k = 0; k < b.x.size(); k += 16)
    truth.push_back(b.x[k]);
  return compare_trajectories(m, a.x, truth);
}

TEST(PhQuery, VoltagesAloneRecoverTheChainsState) {
  // As the electrical studies' voltage-only protocol: one second of bus voltages, the state fitted
  // and run forward, then forecasts of 0.5 s and 2 s scored against the truth (the paper's Table 8
  // reports, for its numerical history estimate on clean voltages, 0.36 mV / 26 mA / 25 mA at
  // 0.5 s on five-DGU networks).
  const microgrid m = make_microgrid(reference_dgu(), 4, kChain);
  const int episodes = 6;
  double v05 = 0, i05 = 0, l05 = 0, v2 = 0, i2 = 0, l2 = 0;
  for (int ep = 0; ep < episodes; ++ep) {
    rng g(300 + ep);
    const vecd x_true = storage_from_efforts(m.system, sample_physical(g, 4, 3)).x;
    const vecd u = sample_inputs(g, 4);
    const truth t = make_history(m, x_true, u, 100, 0.0, g);
    const history_fit_result f = fit_history(m.system, t.h, fit_options(m, 0.0));
    EXPECT_TRUE(f.success) << "episode " << ep;
    EXPECT_TRUE(f.admitted(m.system.state_dim())) << "episode " << ep;
    EXPECT_LE(f.evals, 20);
    EXPECT_EQ(f.xi_covariance.size(), size_t(m.system.state_dim()) * m.system.state_dim());
    const grid_errors a = forecast_errors(m, f.x_now, t.x_now, u, 50),
                      b = forecast_errors(m, f.x_now, t.x_now, u, 200);
    v05 += a.voltage_rmse / episodes;
    i05 += a.dgu_current_rmse / episodes;
    l05 += a.line_current_rmse / episodes;
    v2 += b.voltage_rmse / episodes;
    i2 += b.dgu_current_rmse / episodes;
    l2 += b.line_current_rmse / episodes;
  }
  std::cout << "PhQuery fit, clean voltages, forecast RMSE at 0.5 s: " << 1e3 * v05 << " mV, "
            << 1e3 * i05 << " mA DGU, " << 1e3 * l05 << " mA line; at 2 s: " << 1e3 * v2 << " mV, "
            << 1e3 * i2 << " mA, " << 1e3 * l2 << " mA\n";
  RecordProperty("voltage_rmse_05_V", sci(v05));
  RecordProperty("dgu_current_rmse_05_A", sci(i05));
  RecordProperty("line_current_rmse_05_A", sci(l05));
  // The voltages are pinned; the hidden currents are known to tens of mA, as in the paper.
  EXPECT_LT(v05, 2e-3);
  EXPECT_LT(i05, 6e-2);
  EXPECT_LT(l05, 6e-2);
  EXPECT_LT(v2, 5e-3);
}

TEST(PhQuery, NoisyVoltagesStillGiveAFiniteAdmittedEstimate) {
  const microgrid m = make_microgrid(reference_dgu(), 4, kChain);
  rng g(41);
  const vecd x_true = storage_from_efforts(m.system, sample_physical(g, 4, 3)).x;
  const vecd u = sample_inputs(g, 4);
  const truth t = make_history(m, x_true, u, 100, 0.02, g);
  const history_fit_result f = fit_history(m.system, t.h, fit_options(m, 0.02));
  for (double v : f.x_now)
    ASSERT_TRUE(std::isfinite(v));
  EXPECT_EQ(f.rank, m.system.state_dim());
  // Noise of 20 mV leaves the voltages within a few times that.
  EXPECT_LT(max_err(m, f.x_now, t.x_now, voltage_channels(m)), 0.1);
}

TEST(PhQuery, ASymmetricCycleHidesACirculatingCurrent) {
  // Known linear laws, every bus voltage observed: a tree determines its state, a cycle whose lines
  // all decay at the same rate does not (a current circulating round it leaves every voltage the
  // same), so its data Jacobian loses one rank and the estimate is not admitted.
  const law_ptr dgu = linear_dgu(2.2, 1.8, 1.2);
  const microgrid tree = make_microgrid(dgu, 3, {{0, 1, 0.4, 0.8}, {1, 2, 0.4, 0.8}});
  const microgrid ring =
      make_microgrid(dgu, 3, {{0, 1, 0.4, 0.8}, {1, 2, 0.4, 0.8}, {2, 0, 0.4, 0.8}});
  rng g(8);
  for (const microgrid *m : {&tree, &ring}) {
    const int lines = static_cast<int>(m->lines.size());
    const vecd x_true = storage_from_efforts(m->system, sample_physical(g, 3, lines)).x;
    const truth t = make_history(*m, x_true, sample_inputs(g, 3), 100, 0.0, g);
    const history_fit_result f = fit_history(m->system, t.h, fit_options(*m, 0.0));
    const int full = m->system.state_dim();
    if (m == &tree) {
      EXPECT_EQ(f.rank, full);
      EXPECT_TRUE(f.admitted(full));
    } else {
      EXPECT_EQ(f.rank, full - 1);
      EXPECT_FALSE(f.admitted(full));
    }
  }
}

TEST(PhQuery, AHistoryIsChecked) {
  const microgrid m = make_microgrid(reference_dgu(), 2, {{0, 1, 0.3, 0.8}});
  history h;
  h.inputs.assign(10, vecd(4, 0.0));
  h.channels = {0};
  h.sample_steps = {0, 11};
  h.samples = {{1.0}, {1.0}};
  EXPECT_THROW(fit_history(m.system, h, fit_options(m, 0.0)), std::invalid_argument);
  h.sample_steps = {0, 5};
  h.channels = {99};
  EXPECT_THROW(fit_history(m.system, h, fit_options(m, 0.0)), std::invalid_argument);
}

// ── 3. consequences ────────────────────────────────────────────────────────────────────────────

TEST(PhQuery, ConsequencesCarryTheStepsBooks) {
  const microgrid m = make_microgrid(reference_dgu(), 4, kChain);
  rng g(5);
  const vecd x0 = storage_from_efforts(m.system, sample_physical(g, 4, 3)).x,
             u = sample_inputs(g, 4);
  transition_opts o;
  const rollout_result r = rollout(m.system, o, x0, u, 50);
  const std::vector<consequence> c = consequences(m.system, r, u);
  ASSERT_EQ(c.size(), 50u);
  for (size_t k = 0; k < c.size(); ++k) {
    EXPECT_EQ(c[k].stored, m.system.energy(r.x[k + 1].data()));
    EXPECT_EQ(c[k].work, r.ledger[k].supplied);
    EXPECT_EQ(c[k].loss, r.ledger[k].dissipated);
    EXPECT_EQ(c[k].residual, r.ledger[k].residual);
    EXPECT_GE(c[k].loss_power, 0.0);
  }
  // The card's energy lines are those books summed.
  const action_card card =
      make_card("a", c, {{"loss", summary::total_loss, {}}, {"work", summary::total_work, {}}});
  double loss = 0, work = 0;
  for (const books &b : r.ledger) {
    loss += b.dissipated;
    work += b.supplied;
  }
  EXPECT_DOUBLE_EQ(card.value("loss"), loss);
  EXPECT_DOUBLE_EQ(card.value("work"), work);
  EXPECT_THROW(card.value("nothing"), std::out_of_range);
}

// ── 4. the ranking ─────────────────────────────────────────────────────────────────────────────

action_card card_of(const std::string &name, double peak, double loss) {
  action_card c;
  c.candidate = name;
  c.names = {"peak", "loss"};
  c.values = {peak, loss};
  return c;
}

TEST(PhQuery, TheRankingAdmitsOrdersAndAbstainsByItsBounds) {
  const std::vector<card_limit> limits = {{"peak", 1.0, true}};
  const card_objective obj{{"loss"}, {1.0}, {}};
  // a: safely under the limit and cheapest; b: dearer; c: over the limit even with its bound.
  std::vector<action_card> cards = {card_of("a", 0.5, 1.0), card_of("b", 0.6, 2.0),
                                    card_of("c", 1.5, 0.1)};
  ranking r = rank_candidates(cards, {{{0.1, 0.1}}, {{0.1, 0.1}}, {{0.1, 0.1}}}, limits, obj);
  EXPECT_EQ(r.feasible[0], feasibility::admitted);
  EXPECT_EQ(r.feasible[2], feasibility::rejected);
  EXPECT_EQ(r.best, 0);
  EXPECT_EQ(r.outcome, verdict::certified);
  EXPECT_EQ(r.order[0][1], 1);
  EXPECT_EQ(r.order[1][0], -1);
  EXPECT_DOUBLE_EQ(r.suboptimality, 0.2);
  // Wider bounds overlap: the same best, no certificate.
  r = rank_candidates(cards, {{{0.1, 0.6}}, {{0.1, 0.6}}, {{0.1, 0.6}}}, limits, obj);
  EXPECT_EQ(r.best, 0);
  EXPECT_EQ(r.outcome, verdict::inconclusive);
  EXPECT_EQ(r.order[0][1], 0);
  // A bound that crosses the limit leaves a candidate unresolved, and an unresolved rival blocks
  // the certificate.
  cards[2] = card_of("c", 0.95, 0.1);
  r = rank_candidates(cards, {{{0.1, 0.1}}, {{0.1, 0.1}}, {{0.1, 0.1}}}, limits, obj);
  EXPECT_EQ(r.feasible[2], feasibility::unresolved);
  EXPECT_EQ(r.best, 0);
  EXPECT_EQ(r.outcome, verdict::inconclusive);
  // Nothing admitted: abstain.
  r = rank_candidates({card_of("x", 2.0, 1.0)}, {{{0.1, 0.1}}}, limits, obj);
  EXPECT_EQ(r.outcome, verdict::abstain);
  EXPECT_EQ(r.best, -1);
  EXPECT_THROW(rank_candidates(cards, {{{0.1, 0.1}}}, limits, obj), std::invalid_argument);
}

// ── the whole query ────────────────────────────────────────────────────────────────────────────

struct query_outcome {
  int agree = 0, certified = 0, wrong_certificates = 0;
  double peak_error = 0;
};

// The candidate query: the actual system is the four-DGU chain, known by its measured channels over
// the last second. Each candidate adds the same DGU and a 0.4 ohm line at bus 0 or at bus 2.
// Predict both from the estimated state, card them (peak line current, voltage range, accumulated
// loss), and rank them: a peak line current of at most 0.5 A, then the lowest loss. The bounds are
// sampled: starts drawn from the fit's posterior (2 sigma), plus the transition's own error
// against RK4 at dt / 8.
query_outcome run_query(bool full_state, int episodes) {
  const law_ptr dgu = reference_dgu();
  const microgrid now = make_microgrid(dgu, 4, kChain);
  std::vector<microgrid> cands;
  for (int bus : {0, 2}) {
    std::vector<line_spec> lines = kChain;
    lines.push_back({bus, 4, 0.4, 0.8});
    cands.push_back(make_microgrid(dgu, 5, lines));
  }
  const microgrid lone = make_microgrid(dgu, 1, {});
  transition_opts o;
  transition_opts fine;
  fine.scheme = step_scheme::rk4;
  fine.dt = 0.01 / 8;
  query_outcome q;
  for (int ep = 0; ep < episodes; ++ep) {
    rng g(700 + ep);
    const vecd x_true = storage_from_efforts(now.system, sample_physical(g, 4, 3)).x;
    const vecd u_now = sample_inputs(g, 4);
    truth t = make_history(now, x_true, u_now, 100, 0.0, g);
    if (full_state) { // every readout channel measured
      transition_opts f2;
      f2.scheme = step_scheme::rk4;
      f2.dt = 0.01 / 16;
      t.h.channels.clear();
      for (int c = 0; c < now.system.state_dim(); ++c)
        t.h.channels.push_back(c);
      t.h.samples.clear();
      vecd x = x_true, e(now.system.state_dim());
      for (int k = 0; k <= 100; ++k) {
        if (k % 5 == 0) {
          now.system.readout(x.data(), e.data());
          t.h.samples.push_back(e);
        }
        if (k < 100)
          for (int j = 0; j < 16; ++j)
            step(now.system, f2, u_now.data(), x.data());
      }
    }
    const history_fit_options fo = fit_options(now, 0.0);
    const history_fit_result f = fit_history(now.system, t.h, fo);
    EXPECT_TRUE(f.admitted(now.system.state_dim())) << "episode " << ep;
    const std::vector<vecd> draws = posterior_states(now.system, t.h, fo, f, 8, 50 + ep, 2.0);
    EXPECT_GE(draws.size(), 6u);
    const vecd added =
        storage_from_efforts(lone.system, {uniform(g, 0.9, 1.1), uniform(g, 0.0, 0.2)}).x;
    vecd u = u_now;
    const vecd u_add = sample_inputs(g, 1);
    u.insert(u.end(), u_add.begin(), u_add.end());
    std::vector<action_card> cards, true_cards;
    std::vector<card_bounds> bounds;
    for (int c = 0; c < 2; ++c) {
      const microgrid &m = cands[c];
      const std::vector<card_entry> spec = {
          {"peak_line_current", summary::max_abs, line_current_channels(m)},
          {"min_voltage", summary::min, voltage_channels(m)},
          {"max_voltage", summary::max, voltage_channels(m)},
          {"loss", summary::total_loss, {}}};
      const std::map<std::string, vecd> supplied = {{"dgu4", added}, {"line3", {0.0}}};
      const vecd x0 = assign_state(now.system, f.x_now, m.system, supplied);
      const vecd x0_true = assign_state(now.system, t.x_now, m.system, supplied);
      cards.push_back(make_card(c == 0 ? "bus0" : "bus2",
                                consequences(m.system, rollout(m.system, o, x0, u, 200), u), spec));
      // The physical outcome: the true state, RK4 at dt / 8, its books summed over the model's
      // steps.
      const rollout_result rt = rollout(m.system, fine, x0_true, u, 200 * 8);
      rollout_result coarse;
      for (size_t k = 0; k < rt.x.size(); k += 8)
        coarse.x.push_back(rt.x[k]);
      for (int k = 0; k < 200; ++k) {
        books b{};
        for (int j = 0; j < 8; ++j) {
          const books &s = rt.ledger[size_t(k) * 8 + j];
          b.dH += s.dH;
          b.supplied += s.supplied;
          b.dissipated += s.dissipated;
          b.residual += s.residual;
        }
        coarse.ledger.push_back(b);
      }
      true_cards.push_back(
          make_card(cards.back().candidate, consequences(m.system, coarse, u), spec));
      std::vector<vecd> starts;
      for (const vecd &d : draws)
        starts.push_back(assign_state(now.system, d, m.system, supplied));
      bounds.push_back(sampled_bounds(m.system, o, x0, u, 200, spec, starts, {}, 8));
      q.peak_error = std::max(q.peak_error, std::fabs(cards[c].value("peak_line_current") -
                                                      true_cards[c].value("peak_line_current")));
    }
    const std::vector<card_limit> limits = {{"peak_line_current", 0.5, true}};
    const card_objective obj{{"loss"}, {1.0}, {}};
    const ranking r = rank_candidates(cards, bounds, limits, obj);
    const ranking rt = rank_candidates(true_cards, {{vecd(4, 0.0)}, {vecd(4, 0.0)}}, limits, obj);
    if (r.best == rt.best)
      ++q.agree;
    if (r.outcome == verdict::certified) {
      ++q.certified;
      if (r.best != rt.best)
        ++q.wrong_certificates;
    }
  }
  return q;
}

TEST(PhQuery, AQueryRanksTheAddedDgusConnections) {
  const int episodes = 4;
  const query_outcome v = run_query(false, episodes), full = run_query(true, episodes);
  std::cout << "PhQuery, voltages only: best matches the physical best in " << v.agree << "/"
            << episodes << ", certified " << v.certified << ", worst peak line-current error "
            << 1e3 * v.peak_error << " mA\nPhQuery, full state: " << full.agree << "/" << episodes
            << ", certified " << full.certified << ", worst peak error " << 1e3 * full.peak_error
            << " mA\n";
  RecordProperty("voltages_agree", v.agree);
  RecordProperty("voltages_certified", v.certified);
  RecordProperty("full_agree", full.agree);
  RecordProperty("full_certified", full.certified);
  // A certificate is never wrong about the physical outcome.
  EXPECT_EQ(v.wrong_certificates, 0);
  EXPECT_EQ(full.wrong_certificates, 0);
  // Measured on everything: the full state pins the prediction and certifies; voltages alone leave
  // tens of mA of current error and certify less.
  EXPECT_EQ(full.agree, episodes);
  EXPECT_GE(full.certified, v.certified);
  EXPECT_LT(full.peak_error, 1e-3);
  EXPECT_LT(v.peak_error, 0.1);
}

} // namespace

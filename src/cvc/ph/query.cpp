/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph queries (inc/cvc/ph/query.h): the history fit, consequences, action cards, the ranking
// and sampled bounds.

#include <algorithm>
#include <cmath>
#include <cvc/ph/query.h>
#include <cvc/ph/rng.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace cvc {
namespace ph {

namespace {

[[noreturn]] void bad(const std::string &what) { throw std::invalid_argument("cvc::ph: " + what); }

// Solves A x = b (n x n, row-major) in place by elimination with partial pivoting; false when
// singular.
bool solve(int n, vecd A, vecd &b) {
  for (int k = 0; k < n; ++k) {
    int piv = k;
    for (int r = k + 1; r < n; ++r)
      if (std::fabs(A[r * n + k]) > std::fabs(A[piv * n + k]))
        piv = r;
    if (!(std::fabs(A[piv * n + k]) > 0))
      return false;
    if (piv != k) {
      for (int c = 0; c < n; ++c)
        std::swap(A[k * n + c], A[piv * n + c]);
      std::swap(b[k], b[piv]);
    }
    for (int r = k + 1; r < n; ++r) {
      const double f = A[r * n + k] / A[k * n + k];
      for (int c = k; c < n; ++c)
        A[r * n + c] -= f * A[k * n + c];
      b[r] -= f * b[k];
    }
  }
  for (int k = n - 1; k >= 0; --k) {
    double s = b[k];
    for (int c = k + 1; c < n; ++c)
      s -= A[k * n + c] * b[c];
    b[k] = s / A[k * n + k];
  }
  return true;
}

// The lower Cholesky factor of a symmetric positive definite n x n matrix (row-major); false when
// it is not positive definite.
bool cholesky(int n, const vecd &A, vecd &L) {
  L.assign(size_t(n) * n, 0.0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j <= i; ++j) {
      double s = A[size_t(i) * n + j];
      for (int k = 0; k < j; ++k)
        s -= L[size_t(i) * n + k] * L[size_t(j) * n + k];
      if (i == j) {
        if (!(s > 0))
          return false;
        L[size_t(i) * n + i] = std::sqrt(s);
      } else {
        L[size_t(i) * n + j] = s / L[size_t(j) * n + j];
      }
    }
  return true;
}

// The eigenvalues of a symmetric n x n matrix, by cyclic Jacobi rotations.
vecd symmetric_eigenvalues(int n, vecd A) {
  for (int sweep = 0; sweep < 100; ++sweep) {
    double off = 0;
    for (int p = 0; p < n; ++p)
      for (int q = p + 1; q < n; ++q)
        off += A[p * n + q] * A[p * n + q];
    double diag = 0;
    for (int p = 0; p < n; ++p)
      diag += A[p * n + p] * A[p * n + p];
    if (off <= 1e-30 * (diag + 1e-300))
      break;
    for (int p = 0; p < n; ++p)
      for (int q = p + 1; q < n; ++q) {
        const double apq = A[p * n + q];
        if (apq == 0)
          continue;
        const double theta = (A[q * n + q] - A[p * n + p]) / (2 * apq);
        const double t = (theta >= 0 ? 1 : -1) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
        const double c = 1 / std::sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < n; ++k) { // columns p, q
          const double akp = A[k * n + p], akq = A[k * n + q];
          A[k * n + p] = c * akp - s * akq;
          A[k * n + q] = s * akp + c * akq;
        }
        for (int k = 0; k < n; ++k) { // rows p, q
          const double apk = A[p * n + k], aqk = A[q * n + k];
          A[p * n + k] = c * apk - s * aqk;
          A[q * n + k] = s * apk + c * aqk;
        }
      }
  }
  vecd ev(n);
  for (int p = 0; p < n; ++p)
    ev[p] = A[p * n + p];
  return ev;
}

double half_norm2(const vecd &r) {
  double s = 0;
  for (double v : r)
    s += v * v;
  return 0.5 * s;
}

void check_history(const composite &s, const history &h) {
  const size_t K = s.input_dim(), N = s.state_dim();
  for (const vecd &u : h.inputs)
    if (u.size() != K)
      bad("history: an input row has " + std::to_string(u.size()) + " values, the inputs " +
          std::to_string(K));
  if (h.sample_steps.size() != h.samples.size())
    bad("history: " + std::to_string(h.sample_steps.size()) + " sample steps but " +
        std::to_string(h.samples.size()) + " samples");
  int last = -1;
  for (int k : h.sample_steps) {
    if (k <= last || k > static_cast<int>(h.inputs.size()))
      bad("history: sample steps must increase within 0 .. steps");
    last = k;
  }
  for (int c : h.channels)
    if (c < 0 || c >= static_cast<int>(N))
      bad("history: channel " + std::to_string(c) + " is not a readout index");
  for (const vecd &y : h.samples)
    if (y.size() != h.channels.size())
      bad("history: a sample has " + std::to_string(y.size()) + " values for " +
          std::to_string(h.channels.size()) + " channels");
}

} // namespace

// ── 1. the state from a measured history ────────────────────────────────────────────────────────

std::vector<vecd> history_readouts(const composite &s, const history &h,
                                   const transition_opts &step_opts, const vecd &x_start,
                                   fit_space space) {
  check_history(s, h);
  std::vector<vecd> out;
  vecd x = x_start, e(s.state_dim());
  size_t next = 0;
  const int steps = static_cast<int>(h.inputs.size());
  for (int k = 0; k <= steps && next < h.sample_steps.size(); ++k) {
    if (h.sample_steps[next] == k) {
      if (space == fit_space::state)
        e = x;
      else
        s.readout(x.data(), e.data());
      vecd row;
      for (int c : h.channels)
        row.push_back(e[c]);
      out.push_back(std::move(row));
      ++next;
    }
    if (k < steps)
      step(s, step_opts, h.inputs[k].data(), x.data());
  }
  return out;
}

history_fit_result fit_history(const composite &s, const history &h, const history_fit_options &o) {
  check_history(s, h);
  const int N = s.state_dim();
  if (static_cast<int>(o.prior_mean.size()) != N || static_cast<int>(o.prior_scale.size()) != N)
    bad("fit_history: the prior needs state_dim means and scales");
  const double w = 1.0 / std::max(o.noise, o.noise_floor);
  const int data_rows = static_cast<int>(h.samples.size() * h.channels.size());
  const int rows = data_rows + N;

  history_fit_result res;
  // r(xi): the weighted data misfit, then the prior's rows. false when no state reaches the
  // efforts.
  const bool in_state = o.space == fit_space::state;
  auto residual = [&](const vecd &xi, vecd &r, vecd *x_start) {
    vecd e(N);
    for (int i = 0; i < N; ++i)
      e[i] = o.prior_mean[i] + o.prior_scale[i] * xi[i];
    storage_result st;
    if (in_state) {
      st.x = e;
    } else {
      st = storage_from_efforts(s, e);
      if (!st.converged)
        return false;
    }
    const std::vector<vecd> y = history_readouts(s, h, o.step, st.x, o.space);
    r.assign(rows, 0.0);
    int k = 0;
    for (size_t j = 0; j < y.size(); ++j)
      for (size_t c = 0; c < h.channels.size(); ++c)
        r[k++] = w * (y[j][c] - h.samples[j][c]);
    for (int i = 0; i < N; ++i)
      r[data_rows + i] = xi[i];
    for (double v : r)
      if (!std::isfinite(v))
        return false;
    if (x_start)
      *x_start = st.x;
    return true;
  };
  auto jacobian = [&](const vecd &xi, vecd &J) {
    J.assign(size_t(rows) * N, 0.0);
    vecd rp, rm, t = xi;
    for (int j = 0; j < N; ++j) {
      t[j] = xi[j] + o.jacobian_step;
      const bool a = residual(t, rp, nullptr);
      t[j] = xi[j] - o.jacobian_step;
      const bool b = residual(t, rm, nullptr);
      t[j] = xi[j];
      if (!a || !b)
        return false;
      for (int r = 0; r < rows; ++r)
        J[size_t(r) * N + j] = (rp[r] - rm[r]) / (2 * o.jacobian_step);
    }
    return true;
  };

  vecd xi(N, 0.0), r, J, x_start;
  if (!residual(xi, r, &x_start))
    throw std::runtime_error("cvc::ph: fit_history: no state reaches the prior mean's efforts");
  res.evals = 1;
  double cost = half_norm2(r);
  bool have_j = jacobian(xi, J);
  double lambda = 1e-3;
  vecd g(N), A(size_t(N) * N), rt, xt;
  auto gradient = [&]() {
    std::fill(g.begin(), g.end(), 0.0);
    for (int r_ = 0; r_ < rows; ++r_)
      for (int j = 0; j < N; ++j)
        g[j] += J[size_t(r_) * N + j] * r[r_];
    double m = 0;
    for (double v : g)
      m = std::max(m, std::fabs(v));
    return m;
  };
  while (have_j) {
    const double gmax = gradient();
    res.optimality = gmax;
    if (gmax <= o.gtol) {
      res.success = true;
      break;
    }
    if (res.evals >= o.max_evals)
      break;
    // (J^T J + lambda diag(J^T J)) dxi = -J^T r.
    std::fill(A.begin(), A.end(), 0.0);
    for (int r_ = 0; r_ < rows; ++r_)
      for (int i = 0; i < N; ++i) {
        const double ji = J[size_t(r_) * N + i];
        if (ji == 0)
          continue;
        for (int j = 0; j < N; ++j)
          A[size_t(i) * N + j] += ji * J[size_t(r_) * N + j];
      }
    vecd M = A, d(N);
    for (int i = 0; i < N; ++i) {
      M[size_t(i) * N + i] += lambda * std::max(A[size_t(i) * N + i], 1e-12);
      d[i] = -g[i];
    }
    if (!solve(N, M, d)) {
      lambda *= 10;
      continue;
    }
    vecd trial(N);
    double dn = 0, xn = 0;
    for (int i = 0; i < N; ++i) {
      trial[i] = xi[i] + d[i];
      dn += d[i] * d[i];
      xn += xi[i] * xi[i];
    }
    const bool ok = residual(trial, rt, &xt);
    ++res.evals;
    const double tcost = ok ? half_norm2(rt) : std::numeric_limits<double>::infinity();
    if (tcost < cost) {
      const double drop = cost - tcost;
      xi = trial;
      r = rt;
      x_start = xt;
      const double old = cost;
      cost = tcost;
      lambda = std::max(lambda / 3, 1e-12);
      have_j = jacobian(xi, J);
      if (drop <= o.ftol * old || std::sqrt(dn) <= o.xtol * (o.xtol + std::sqrt(xn))) {
        if (have_j)
          res.optimality = gradient();
        res.success = true;
        break;
      }
    } else {
      lambda *= 4;
      if (lambda > 1e16)
        break;
    }
  }
  res.xi = xi;
  res.cost = cost;
  res.x_start = x_start;
  // The rank of the weighted data rows (the prior's left out): singular values from J_d^T J_d.
  if (have_j && data_rows > 0) {
    vecd JtJ(size_t(N) * N, 0.0);
    for (int r_ = 0; r_ < data_rows; ++r_)
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          JtJ[size_t(i) * N + j] += J[size_t(r_) * N + i] * J[size_t(r_) * N + j];
    const vecd ev = symmetric_eigenvalues(N, JtJ);
    double smax = 0;
    for (double v : ev)
      smax = std::max(smax, std::sqrt(std::max(v, 0.0)));
    for (double v : ev)
      if (std::sqrt(std::max(v, 0.0)) > o.rank_rtol * smax)
        ++res.rank;
  }
  // The posterior covariance (J^T J)^-1, column by column.
  if (have_j) {
    vecd JtJ(size_t(N) * N, 0.0);
    for (int r_ = 0; r_ < rows; ++r_)
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          JtJ[size_t(i) * N + j] += J[size_t(r_) * N + i] * J[size_t(r_) * N + j];
    vecd cov(size_t(N) * N);
    bool ok = true;
    for (int j = 0; j < N && ok; ++j) {
      vecd e(N, 0.0);
      e[j] = 1.0;
      ok = solve(N, JtJ, e);
      for (int i = 0; i < N; ++i)
        cov[size_t(i) * N + j] = e[i];
    }
    if (ok)
      res.xi_covariance = cov;
  }
  // The state at the query time: the window run forward from the fitted start.
  res.x_now = x_start;
  for (const vecd &u : h.inputs)
    step(s, o.step, u.data(), res.x_now.data());
  return res;
}

std::vector<vecd> posterior_states(const composite &s, const history &h,
                                   const history_fit_options &o, const history_fit_result &fit,
                                   int count, std::uint64_t seed, double sigmas) {
  const int N = s.state_dim();
  if (fit.xi_covariance.size() != size_t(N) * N || fit.xi.size() != size_t(N))
    bad("posterior_states: the fit has no posterior covariance");
  vecd L;
  vecd sym = fit.xi_covariance;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < i; ++j)
      sym[size_t(i) * N + j] = sym[size_t(j) * N + i] =
          0.5 * (sym[size_t(i) * N + j] + sym[size_t(j) * N + i]);
  if (!cholesky(N, sym, L))
    bad("posterior_states: the posterior covariance is not positive definite");
  rng g(seed);
  std::vector<vecd> out;
  vecd z(N), e(N);
  for (int k = 0; k < count; ++k) {
    for (double &v : z)
      v = g.normal();
    for (int i = 0; i < N; ++i) {
      double xi = fit.xi[i];
      for (int j = 0; j <= i; ++j)
        xi += sigmas * L[size_t(i) * N + j] * z[j];
      e[i] = o.prior_mean[i] + o.prior_scale[i] * xi;
    }
    vecd x = e;
    if (o.space == fit_space::efforts) {
      const storage_result st = storage_from_efforts(s, e, fit.x_start);
      if (!st.converged)
        continue;
      x = st.x;
    }
    for (const vecd &u : h.inputs)
      step(s, o.step, u.data(), x.data());
    out.push_back(std::move(x));
  }
  return out;
}

// ── 3. consequences ─────────────────────────────────────────────────────────────────────────────

std::vector<consequence> consequences(const composite &s, const rollout_result &r, const vecd &u) {
  const size_t K = s.input_dim(), steps = r.ledger.size();
  const bool held = u.size() == K;
  if (!held && u.size() != K * steps)
    bad("consequences: inputs are one row or one per step");
  std::vector<consequence> out(steps);
  const vecd none;
  for (size_t k = 0; k < steps; ++k) {
    const double *uk = K == 0 ? none.data() : (held ? u.data() : u.data() + K * k);
    const vecd &x = r.x[k + 1];
    consequence &c = out[k];
    c.readout.assign(s.state_dim(), 0.0);
    s.readout(x.data(), c.readout.data());
    const power_terms p = s.powers(x.data(), uk);
    c.stored = p.stored;
    c.loss_power = p.loss;
    c.external_power = p.external;
    c.work = r.ledger[k].supplied;
    c.loss = r.ledger[k].dissipated;
    c.residual = r.ledger[k].residual;
  }
  return out;
}

// ── 4. action cards and the ranking ─────────────────────────────────────────────────────────────

double action_card::value(const std::string &name) const {
  for (size_t i = 0; i < names.size(); ++i)
    if (names[i] == name)
      return values[i];
  throw std::out_of_range("cvc::ph: the card has no entry '" + name + "'");
}

action_card make_card(const std::string &candidate, const std::vector<consequence> &c,
                      const std::vector<card_entry> &spec) {
  action_card card;
  card.candidate = candidate;
  for (const card_entry &e : spec) {
    double v = 0;
    switch (e.kind) {
    case summary::max_abs:
    case summary::final_value:
    case summary::min:
    case summary::max: {
      if (e.channels.empty())
        bad("card entry '" + e.name + "' needs channels");
      double lo = std::numeric_limits<double>::infinity(), hi = -lo, big = 0;
      const size_t first = e.kind == summary::final_value && !c.empty() ? c.size() - 1 : 0;
      for (size_t k = first; k < c.size(); ++k)
        for (int ch : e.channels) {
          if (ch < 0 || ch >= static_cast<int>(c[k].readout.size()))
            bad("card entry '" + e.name + "': channel " + std::to_string(ch) +
                " is not a readout index");
          const double y = c[k].readout[ch];
          lo = std::min(lo, y);
          hi = std::max(hi, y);
          big = std::max(big, std::fabs(y));
        }
      v = e.kind == summary::min ? lo : e.kind == summary::max ? hi : big;
      if (c.empty())
        v = 0;
      break;
    }
    case summary::total_loss:
      for (const consequence &k : c)
        v += k.loss;
      break;
    case summary::total_work:
      for (const consequence &k : c)
        v += k.work;
      break;
    case summary::max_residual:
      for (const consequence &k : c)
        v = std::max(v, std::fabs(k.residual));
      break;
    }
    card.names.push_back(e.name);
    card.values.push_back(v);
  }
  return card;
}

ranking rank_candidates(const std::vector<action_card> &cards,
                        const std::vector<card_bounds> &bounds,
                        const std::vector<card_limit> &limits, const card_objective &objective) {
  const size_t n = cards.size();
  if (bounds.size() != n)
    bad("rank_candidates: one set of bounds per card");
  if (objective.weights.size() != objective.entries.size() ||
      (!objective.scales.empty() && objective.scales.size() != objective.entries.size()))
    bad("rank_candidates: the objective needs one weight (and scale) per entry");
  auto index = [&](const action_card &c, const std::string &name) {
    for (size_t i = 0; i < c.names.size(); ++i)
      if (c.names[i] == name)
        return i;
    throw std::out_of_range("cvc::ph: the card has no entry '" + name + "'");
  };
  ranking r;
  r.feasible.assign(n, feasibility::admitted);
  r.score.assign(n, 0.0);
  r.score_bound.assign(n, 0.0);
  for (size_t a = 0; a < n; ++a) {
    const action_card &c = cards[a];
    if (bounds[a].delta.size() != c.values.size())
      bad("rank_candidates: candidate '" + c.candidate + "' needs one bound per card entry");
    for (const card_limit &l : limits) {
      const size_t i = index(c, l.entry);
      const double g = l.upper ? c.values[i] - l.bound : l.bound - c.values[i],
                   dg = bounds[a].delta[i];
      if (g - dg > 0)
        r.feasible[a] = feasibility::rejected;
      else if (g + dg > 0 && r.feasible[a] == feasibility::admitted)
        r.feasible[a] = feasibility::unresolved;
    }
    for (size_t k = 0; k < objective.entries.size(); ++k) {
      const size_t i = index(c, objective.entries[k]);
      const double s =
          objective.scales.empty() || objective.scales[k] == 0 ? 1.0 : objective.scales[k];
      r.score[a] += objective.weights[k] * c.values[i] / s;
      r.score_bound[a] += std::fabs(objective.weights[k]) * bounds[a].delta[i] / std::fabs(s);
    }
  }
  r.order.assign(n, std::vector<int>(n, 0));
  for (size_t a = 0; a < n; ++a)
    for (size_t b = 0; b < n; ++b)
      if (a != b && r.score[a] + r.score_bound[a] < r.score[b] - r.score_bound[b]) {
        r.order[a][b] = 1;
        r.order[b][a] = -1;
      }
  double worst = 0;
  for (size_t a = 0; a < n; ++a)
    if (r.feasible[a] == feasibility::admitted) {
      worst = std::max(worst, r.score_bound[a]);
      if (r.best < 0 || r.score[a] < r.score[r.best])
        r.best = static_cast<int>(a);
    }
  if (r.best < 0) {
    r.outcome = verdict::abstain;
    return r;
  }
  r.suboptimality = 2 * worst;
  bool beats_all = true;
  for (size_t b = 0; b < n; ++b)
    if (static_cast<int>(b) != r.best && r.feasible[b] != feasibility::rejected &&
        r.order[r.best][b] != 1)
      beats_all = false;
  r.outcome = beats_all ? verdict::certified : verdict::inconclusive;
  return r;
}

// ── sampled error bounds ────────────────────────────────────────────────────────────────────────

card_bounds sampled_bounds(const composite &s, const transition_opts &o, const vecd &x0,
                           const vecd &u, int steps, const std::vector<card_entry> &spec,
                           const std::vector<vecd> &starts,
                           const std::vector<const composite *> &alternatives, int refine) {
  auto card_of = [&](const composite &sys, const transition_opts &opts, const vecd &x, int n) {
    const rollout_result r = rollout(sys, opts, x, u, n);
    std::vector<consequence> c = consequences(sys, r, u);
    if (n != steps) { // a finer reference: keep the steps' ends only, with their books summed
      const int f = n / steps;
      std::vector<consequence> coarse(steps);
      for (int k = 0; k < steps; ++k) {
        coarse[k] = c[size_t(k + 1) * f - 1];
        coarse[k].work = coarse[k].loss = coarse[k].residual = 0;
        for (int j = 0; j < f; ++j) {
          coarse[k].work += c[size_t(k) * f + j].work;
          coarse[k].loss += c[size_t(k) * f + j].loss;
          coarse[k].residual += c[size_t(k) * f + j].residual;
        }
      }
      c = coarse;
    }
    return make_card("", c, spec);
  };
  const action_card nominal = card_of(s, o, x0, steps);
  card_bounds b;
  b.delta.assign(spec.size(), 0.0);
  auto widen = [&](vecd &acc, const action_card &c) {
    for (size_t i = 0; i < spec.size(); ++i)
      acc[i] = std::max(acc[i], std::fabs(c.values[i] - nominal.values[i]));
  };
  vecd from_start(spec.size(), 0.0), from_laws(spec.size(), 0.0), from_steps(spec.size(), 0.0);
  for (const vecd &x : starts)
    widen(from_start, card_of(s, o, x, steps));
  for (const composite *alt : alternatives)
    if (alt)
      widen(from_laws, card_of(*alt, o, x0, steps));
  if (refine > 0) {
    transition_opts fine;
    fine.scheme = step_scheme::rk4;
    fine.dt = o.dt / refine;
    const vecd uu = u.size() == size_t(s.input_dim()) ? u : vecd();
    if (!uu.empty()) // a fine reference needs the inputs held; per-step inputs skip it
      widen(from_steps, card_of(s, fine, x0, steps * refine));
  }
  for (size_t i = 0; i < spec.size(); ++i)
    b.delta[i] = from_start[i] + from_laws[i] + from_steps[i];
  return b;
}

const char *to_string(verdict v) {
  switch (v) {
  case verdict::certified:
    return "certified";
  case verdict::inconclusive:
    return "inconclusive";
  case verdict::abstain:
    return "abstain";
  }
  return "abstain";
}

const char *to_string(feasibility f) {
  switch (f) {
  case feasibility::admitted:
    return "admitted";
  case feasibility::rejected:
    return "rejected";
  case feasibility::unresolved:
    return "unresolved";
  }
  return "unresolved";
}

} // namespace ph
} // namespace cvc

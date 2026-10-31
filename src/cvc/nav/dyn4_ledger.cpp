/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/nav/dyn4_ledger.h>
#include <map>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

using diff::dyn4_params;
using diff::dyn4_state;

// At rest: |vx| below this share of the controller's top speed at both ends of a tick.
constexpr double kRestShare = 0.02;

dyn4_state<double> to_double(const dyn4_state<float> &s) {
  dyn4_state<double> d;
  for (int k = 0; k < dyn4_state<float>::size; ++k)
    d[k] = static_cast<double>(s[k]);
  return d;
}

// The ticks' states with the model's drive actuator re-run on their commands under p (each
// vehicle's from rest at its first tick); the rest of each state is the simulation's.
void rerun(const dyn4_params &p, const diff::bike_veh &v,
           const std::vector<dyn4_ledger_tick> &ticks, std::vector<dyn4_state<float>> &s0,
           std::vector<dyn4_state<float>> &s1) {
  s0.resize(ticks.size());
  s1.resize(ticks.size());
  std::map<int, float> held;
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    const dyn4_ledger_tick &k = ticks[t];
    const auto it = held.find(k.agent);
    s0[t] = k.s0;
    s0[t].a = k.first || it == held.end() ? (k.first ? 0.0f : k.s0.a) : it->second;
    s1[t] = k.s1;
    s1[t].a = diff::dyn4_drive_actuator(p, v, s0[t], k.a_cmd);
    held[k.agent] = s1[t].a;
  }
}

bool kept(const dyn4_ledger_tick &k, const diff::bike_veh &v, dyn4_books *b) {
  if (k.lines.contact) {
    if (b)
      ++b->skipped_contact;
    return false;
  }
  const double rest = kRestShare * v.vmax;
  if (std::fabs(k.s0.vx) < rest && std::fabs(k.s1.vx) < rest) {
    if (b)
      ++b->skipped_rest;
    return false;
  }
  return true;
}

double ratio(double plant, double model, double lo = 0.1, double hi = 10.0) {
  if (!(std::fabs(model) > 1e-30) || !(plant > 0.0) || !(model > 0.0))
    return 1.0;
  return std::min(hi, std::max(lo, plant / model));
}

// min |A x - y|^2 over x >= 0, two columns (the normal equations, a column dropped when its
// coefficient would be negative).
void nnls2(double a11, double a12, double a22, double b1, double b2, double &x1, double &x2) {
  const double det = a11 * a22 - a12 * a12;
  x1 = x2 = 0.0;
  if (det > 1e-30 * std::max(1.0, a11 * a22)) {
    x1 = (a22 * b1 - a12 * b2) / det;
    x2 = (a11 * b2 - a12 * b1) / det;
  }
  if (x1 >= 0.0 && x2 >= 0.0 && det > 0.0)
    return;
  // One column alone: the better of the two.
  const double y1 = a11 > 0.0 ? std::max(0.0, b1 / a11) : 0.0;
  const double y2 = a22 > 0.0 ? std::max(0.0, b2 / a22) : 0.0;
  // |A x - y|^2 - |y|^2 = x^T A^T A x - 2 x^T A^T y.
  const double f1 = y1 * y1 * a11 - 2.0 * y1 * b1, f2 = y2 * y2 * a22 - 2.0 * y2 * b2;
  if (f1 <= f2) {
    x1 = y1;
    x2 = 0.0;
  } else {
    x1 = 0.0;
    x2 = y2;
  }
}

// The model's one-tick residual at each kept tick, its step from the simulation's state against
// the state the simulation reached: the speed vx, the lateral speed vy and the yaw rate times the
// wheelbase (dyn4_books::twist_rms's).
void step_residual(const dyn4_params &p, const diff::bike_veh &v, float mu,
                   const std::vector<dyn4_ledger_tick> &ticks, const std::vector<char> &keep,
                   std::vector<double> &out) {
  std::vector<dyn4_state<float>> s0, s1;
  rerun(p, v, ticks, s0, s1);
  const diff::flat_ground ground{mu};
  const double L = static_cast<double>(p.lf) + p.lr;
  out.clear();
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    if (!keep[t])
      continue;
    dyn4_state<float> o;
    diff::dyn4_plant<float>(p, v, ground, s0[t], ticks[t].delta_cmd, ticks[t].a_cmd, o);
    out.push_back(static_cast<double>(o.vx) - ticks[t].s1.vx);
    out.push_back(static_cast<double>(o.vy) - ticks[t].s1.vy);
    out.push_back(L * (static_cast<double>(o.r) - ticks[t].s1.r));
  }
}

// Gauss-Newton with forward differences and Levenberg's damping on m <= 5 parameters x, each kept
// in [lo, hi]: least squares on residual(x, r). A step is halved until it lowers the cost, so the
// fit never ends worse than it started.
template <class F>
void gauss_newton(double *x, const double *lo, const double *hi, int m, int iters,
                  const F &residual) {
  constexpr int kMax = 5;
  std::vector<double> r0, rk[kMax], rt;
  const auto cost = [](const std::vector<double> &r) {
    double c = 0.0;
    for (const double e : r)
      c += e * e;
    return c;
  };
  for (int it = 0; it < iters; ++it) {
    residual(x, r0);
    const double c0 = cost(r0);
    for (int k = 0; k < m; ++k) {
      double xk[kMax];
      std::copy(x, x + m, xk);
      const double h = 1e-3 * std::max(1.0, std::fabs(x[k]));
      xk[k] += h;
      residual(xk, rk[k]);
      for (std::size_t t = 0; t < r0.size(); ++t)
        rk[k][t] = (rk[k][t] - r0[t]) / h;
    }
    double A[kMax][kMax + 1] = {};
    for (std::size_t t = 0; t < r0.size(); ++t)
      for (int a = 0; a < m; ++a) {
        A[a][m] -= rk[a][t] * r0[t];
        for (int b = 0; b < m; ++b)
          A[a][b] += rk[a][t] * rk[b][t];
      }
    // A parameter the ticks do not exercise (no braking) stays where it is.
    for (int a = 0; a < m; ++a)
      A[a][a] += 1e-6 * A[a][a] + 1e-12;
    // Gaussian elimination with partial pivoting.
    for (int c = 0; c < m; ++c) {
      int piv = c;
      for (int r = c + 1; r < m; ++r)
        if (std::fabs(A[r][c]) > std::fabs(A[piv][c]))
          piv = r;
      for (int j = 0; j <= m; ++j)
        std::swap(A[c][j], A[piv][j]);
      if (!(std::fabs(A[c][c]) > 0.0))
        return;
      for (int r = c + 1; r < m; ++r) {
        const double f = A[r][c] / A[c][c];
        for (int j = c; j <= m; ++j)
          A[r][j] -= f * A[c][j];
      }
    }
    double d[kMax];
    for (int c = m - 1; c >= 0; --c) {
      double acc = A[c][m];
      for (int j = c + 1; j < m; ++j)
        acc -= A[c][j] * d[j];
      d[c] = acc / A[c][c];
    }
    bool lowered = false;
    for (double step = 1.0; step > 1e-3 && !lowered; step *= 0.5) {
      double xt[kMax];
      for (int k = 0; k < m; ++k)
        xt[k] = std::min(hi[k], std::max(lo[k], x[k] + step * d[k]));
      residual(xt, rt);
      if (cost(rt) < c0) {
        std::copy(xt, xt + m, x);
        lowered = true;
      }
    }
    if (!lowered)
      return;
  }
}

// The model's lines over each kept tick, its slip proxy (trapezoid, x hdt), and its step energy
// basis (hdt softplus(port) at the end).
struct model_ticks {
  std::vector<plant_energy_lines> lines;
  std::vector<double> slip;
};

model_ticks model_lines(const dyn4_params &p, const diff::bike_veh &v, float mu,
                        const std::vector<dyn4_ledger_tick> &ticks, const std::vector<char> &keep) {
  std::vector<dyn4_state<float>> s0, s1;
  rerun(p, v, ticks, s0, s1);
  model_ticks m;
  m.lines.assign(ticks.size(), plant_energy_lines{});
  m.slip.assign(ticks.size(), 0.0);
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    if (!keep[t])
      continue;
    m.lines[t] = dyn4_tick_lines(p, v, mu, s0[t], s1[t]);
    dyn4_state<double> a = to_double(s0[t]), b = to_double(s1[t]);
    a.delta = b.delta;
    a.a = b.a;
    m.slip[t] = 0.5 * v.hdt * (dyn4_slip_proxy(p, v, mu, a) + dyn4_slip_proxy(p, v, mu, b));
  }
  return m;
}

double critical_damping(const dyn4_params &p, const diff::bike_veh &v, bool front) {
  const float kc = front ? p.k_f : p.k_r;
  const float m_corner = 0.25f * p.m;
  const float kk = kc > 0.0f ? kc : diff::dyn4_rigid_k(m_corner, v.hdt);
  return 2.0 * std::sqrt(static_cast<double>(kk) * m_corner);
}

} // namespace

double dyn4_line_totals::error() const {
  if (plant == 0.0)
    return model == 0.0 ? 0.0 : (model > 0.0 ? 1.0 : -1.0);
  return (model - plant) / std::fabs(plant);
}

plant_energy_lines dyn4_tick_lines(const dyn4_params &p, const diff::bike_veh &v, float mu,
                                   const dyn4_state<float> &s0, const dyn4_state<float> &s1) {
  const diff::flat_ground ground{mu};
  dyn4_state<double> a = to_double(s0);
  const dyn4_state<double> b = to_double(s1);
  const double h0 = diff::dyn4_ph_hamiltonian(p, v, ground, a);
  a.delta = b.delta;
  a.a = b.a;
  const diff::dyn4_ph_lines<double> la = diff::dyn4_ph_lines_at(p, v, ground, a),
                                    lb = diff::dyn4_ph_lines_at(p, v, ground, b);
  const double half = 0.5 * v.hdt;
  plant_energy_lines l;
  l.port = half * (la.port + lb.port);
  l.tire = half * (la.tire + lb.tire);
  l.drag = half * (la.drag + lb.drag);
  l.rolling = half * (la.rolling + lb.rolling);
  l.damper = half * (la.damper + lb.damper);
  l.stored = diff::dyn4_ph_hamiltonian(p, v, ground, b) - h0;
  l.residual = l.stored - (l.port - l.tire - l.drag - l.rolling - l.damper);
  return l;
}

double dyn4_slip_proxy(const dyn4_params &p, const diff::bike_veh &v, float mu,
                       const dyn4_state<double> &s) {
  const diff::flat_ground ground{mu};
  diff::dyn4_ph_parts<double> q;
  diff::dyn4_ph_parts_at(p, v, ground, s, q);
  double out = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double cap = q.W.mu[i] * q.W.Fz[i];
    if (cap > 1e-12)
      out += q.Fx[i] * q.Fx[i] * q.ui[i] / cap;
  }
  return out;
}

diff::dyn4_params calibrated(const dyn4_params &p, const dyn4_calibration &c) {
  dyn4_params q = p;
  q.drive_gain = c.drive_gain;
  q.brake_gain = c.brake_gain;
  q.resist_comp = c.resist_comp;
  if (c.tau_drive >= 0.0f)
    q.tau_drive = c.tau_drive;
  q.B_f = static_cast<float>(p.B_f * c.tire);
  q.B_r = static_cast<float>(p.B_r * c.tire);
  q.drag = static_cast<float>(p.drag * c.drag);
  q.c_rr = static_cast<float>(p.c_rr * c.rolling);
  if (c.c_f > 0.0f)
    q.c_f = c.c_f;
  if (c.c_r > 0.0f)
    q.c_r = c.c_r;
  return q;
}

dyn4_books dyn4_books_of(const dyn4_params &p0, const diff::bike_veh &v, float mu,
                         const dyn4_calibration &c, const std::vector<dyn4_ledger_tick> &ticks) {
  const dyn4_params p = calibrated(p0, c);
  dyn4_books b;
  std::vector<char> keep(ticks.size(), 0);
  for (std::size_t t = 0; t < ticks.size(); ++t)
    keep[t] = kept(ticks[t], v, &b) ? 1 : 0;
  const model_ticks m = model_lines(p, v, mu, ticks, keep);
  std::vector<dyn4_state<float>> s0, s1;
  rerun(p, v, ticks, s0, s1);
  const diff::flat_ground ground{mu};
  double ss_res = 0.0, sum = 0.0, sum2 = 0.0, tw2 = 0.0;
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    if (!keep[t])
      continue;
    ++b.ticks;
    const plant_energy_lines &J = ticks[t].lines, &M = m.lines[t];
    const double lon = c.tire_lon * m.slip[t];
    b.port.plant += J.port;
    b.port.model += M.port + lon;
    b.tire.plant += J.tire;
    b.tire.model += c.tire_lat * M.tire + lon;
    b.drag.plant += J.drag;
    b.drag.model += M.drag;
    b.rolling.plant += J.rolling;
    b.rolling.model += M.rolling;
    b.damper.plant += J.damper;
    b.damper.model += M.damper;
    b.stored.plant += J.stored;
    b.stored.model += M.stored;
    const double E = dyn4_step_energy(p, v, mu, c.price, s1[t]);
    b.drawn.plant += J.drawn;
    b.drawn.model += E;
    ss_res += (E - J.drawn) * (E - J.drawn);
    sum += J.drawn;
    sum2 += J.drawn * J.drawn;
    dyn4_state<float> o;
    diff::dyn4_plant<float>(p, v, ground, s0[t], ticks[t].delta_cmd, ticks[t].a_cmd, o);
    const double L = static_cast<double>(p.lf) + p.lr;
    const double ex = o.vx - ticks[t].s1.vx, ey = o.vy - ticks[t].s1.vy,
                 er = L * (o.r - ticks[t].s1.r);
    tw2 += ex * ex + ey * ey + er * er;
  }
  b.dissipated.plant = b.tire.plant + b.drag.plant + b.rolling.plant + b.damper.plant;
  b.dissipated.model = b.tire.model + b.drag.model + b.rolling.model + b.damper.model;
  if (b.ticks > 0) {
    const double n = static_cast<double>(b.ticks);
    const double ss_tot = sum2 - sum * sum / n;
    b.drawn_r2 = ss_tot > 0.0 ? 1.0 - ss_res / ss_tot : 0.0;
    b.twist_rms = std::sqrt(tw2 / n);
  }
  return b;
}

dyn4_calibration calibrate_dyn4(const dyn4_params &p0, const diff::bike_veh &v, float mu,
                                const std::vector<dyn4_ledger_tick> &ticks, int rounds) {
  std::vector<char> keep(ticks.size(), 0);
  std::size_t n_keep = 0;
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    keep[t] = kept(ticks[t], v, nullptr) ? 1 : 0;
    n_keep += keep[t] ? 1u : 0u;
  }
  if (n_keep == 0)
    throw std::invalid_argument("cvc::nav::calibrate_dyn4: no tick to fit (every one touched "
                                "something or was at rest)");
  dyn4_calibration c;
  // The dampers' factor scales the rates p0 has, a critically damped corner's (c 0) included.
  const double c_f0 = p0.c_f > 0.0f ? p0.c_f : critical_damping(p0, v, true);
  const double c_r0 = p0.c_r > 0.0f ? p0.c_r : critical_damping(p0, v, false);
  const dyn4_params &base = p0;
  c.before = dyn4_books_of(p0, v, mu, dyn4_calibration{}, ticks);

  for (int round = 0; round < std::max(1, rounds); ++round) {
    // Drag, rolling resistance and the dampers: the lines are linear in them.
    {
      const model_ticks m = model_lines(calibrated(base, c), v, mu, ticks, keep);
      double drag_J = 0, drag_M = 0, roll_J = 0, roll_M = 0, damp_J = 0, damp_M = 0;
      for (std::size_t t = 0; t < ticks.size(); ++t) {
        if (!keep[t])
          continue;
        const plant_energy_lines &J = ticks[t].lines, &M = m.lines[t];
        drag_J += J.drag;
        drag_M += M.drag;
        roll_J += J.rolling;
        roll_M += M.rolling;
        damp_J += J.damper;
        damp_M += M.damper;
      }
      c.drag = std::min(10.0, std::max(0.1, c.drag * ratio(drag_J, drag_M)));
      c.rolling = std::min(10.0, std::max(0.1, c.rolling * ratio(roll_J, roll_M)));
      c.damper = std::min(10.0, std::max(0.1, c.damper * ratio(damp_J, damp_M)));
      if (c.damper != 1.0) {
        c.c_f = static_cast<float>(c_f0 * c.damper);
        c.c_r = static_cast<float>(c_r0 * c.damper);
      }
    }

    // The drive (drive_gain, brake_gain, resist_comp, tau_drive) and the tires' stiffness
    // together, on the one-tick twist: the drive's force takes from what the friction ellipse
    // leaves the tires, so neither fits alone.
    {
      double x[5] = {c.drive_gain, c.brake_gain, c.resist_comp,
                     c.tau_drive >= 0.0f ? c.tau_drive : p0.tau_drive, c.tire};
      const double lo[5] = {0.25, 0.25, 0.0, 0.0, 0.1}, hi[5] = {4.0, 4.0, 1.5, 0.5, 10.0};
      gauss_newton(x, lo, hi, 5, 4, [&](const double *y, std::vector<double> &r) {
        dyn4_calibration t = c;
        t.drive_gain = static_cast<float>(y[0]);
        t.brake_gain = static_cast<float>(y[1]);
        t.resist_comp = static_cast<float>(y[2]);
        t.tau_drive = static_cast<float>(y[3]);
        t.tire = y[4];
        step_residual(calibrated(base, t), v, mu, ticks, keep, r);
      });
      c.drive_gain = static_cast<float>(x[0]);
      c.brake_gain = static_cast<float>(x[1]);
      c.resist_comp = static_cast<float>(x[2]);
      c.tau_drive = static_cast<float>(x[3]);
      c.tire = x[4];
    }
  }

  // The books' tire line: the model's lateral slip loss and the longitudinal slip, by NNLS.
  {
    const model_ticks m = model_lines(calibrated(base, c), v, mu, ticks, keep);
    double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    for (std::size_t t = 0; t < ticks.size(); ++t) {
      if (!keep[t])
        continue;
      const double J = ticks[t].lines.tire, M = m.lines[t].tire, L = m.slip[t];
      a11 += M * M;
      a12 += M * L;
      a22 += L * L;
      b1 += M * J;
      b2 += L * J;
    }
    double lat = 1.0, lon = 0.0;
    nnls2(a11, a12, a22, b1, b2, lat, lon);
    c.tire_lat = lat;
    c.tire_lon = lon;
  }

  // The price: drawn = drive hdt softplus(port(s1)) + idle hdt, by NNLS on the two columns.
  {
    const dyn4_params p = calibrated(base, c);
    std::vector<dyn4_state<float>> s0, s1;
    rerun(p, v, ticks, s0, s1);
    dyn4_energy_price unit;
    unit.drive = 1.0f; // the basis column: drawn at price (1, 0), softplus at its width
    double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    const double hdt = v.hdt;
    for (std::size_t t = 0; t < ticks.size(); ++t) {
      if (!keep[t])
        continue;
      const double x = dyn4_step_energy(p, v, mu, unit, s1[t]);
      const double y = ticks[t].lines.drawn;
      a11 += x * x;
      a12 += x * hdt;
      a22 += hdt * hdt;
      b1 += x * y;
      b2 += hdt * y;
    }
    double drive = 0.0, idle = 0.0;
    nnls2(a11, a12, a22, b1, b2, drive, idle);
    c.price.drive = static_cast<float>(drive);
    c.price.idle = static_cast<float>(idle);
  }
  c.after = dyn4_books_of(p0, v, mu, c, ticks);
  return c;
}

} // namespace nav
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph microgrids (inc/cvc/ph/microgrid.h): DGU and line laws, networks, physical states and
// trajectory errors.

#include <algorithm>
#include <cmath>
#include <cvc/ph/microgrid.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace cvc {
namespace ph {

namespace {

canonical_spec dgu_spec(double L, double R) {
  if (!(L > 0) || !(R >= 0))
    throw std::invalid_argument("cvc::ph: a DGU needs L > 0 and R >= 0");
  canonical_spec s;
  s.n = 1;
  s.mass = {L};
  s.damping = {R};
  s.ports = 1;
  s.B = {1.0, 0.0};
  s.inputs = 2;
  s.G = {1.0, 0.0, 0.0, 1.0};
  return s;
}

} // namespace

law_ptr linear_dgu(double C, double L, double R, conservative_scheme scheme) {
  if (!(C > 0))
    throw std::invalid_argument("cvc::ph: a DGU needs C > 0");
  canonical_spec s = dgu_spec(L, R);
  s.K = {1.0 / C};
  s.scheme = scheme;
  return std::make_shared<canonical_law>(std::move(s));
}

law_ptr nonlinear_dgu(vecd charge, double L, double R) {
  canonical_spec s = dgu_spec(L, R);
  s.capacitor = std::move(charge);
  s.scheme = conservative_scheme::implicit_midpoint;
  return std::make_shared<canonical_law>(std::move(s));
}

law_ptr reference_dgu() { return nonlinear_dgu({0.0, 2.2, 0.0, 2.2 / 3.0}, 1.8, 1.2); }

law_ptr rl_line(double R, double L) {
  if (!(L > 0) || !(R >= 0))
    throw std::invalid_argument("cvc::ph: a line needs L > 0 and R >= 0");
  linear_spec s;
  s.n = 1;
  s.Q = {1.0 / L};
  s.R = {R};
  s.ports = 1;
  s.B = {1.0};
  return std::make_shared<linear_law>(std::move(s));
}

microgrid make_microgrid(const std::vector<law_ptr> &dgus, const std::vector<line_spec> &lines) {
  microgrid g;
  const int n = static_cast<int>(dgus.size()), m = static_cast<int>(lines.size());
  for (int i = 0; i < n; ++i)
    g.buses.push_back(g.system.add_block(dgus[i], "dgu" + std::to_string(i)));
  vecd A(size_t(n) * size_t(m), 0.0);
  for (int k = 0; k < m; ++k) {
    const line_spec &l = lines[k];
    if (l.from < 0 || l.from >= n || l.to < 0 || l.to >= n || l.from == l.to)
      throw std::invalid_argument("cvc::ph: line " + std::to_string(k) + " joins buses " +
                                  std::to_string(l.from) + " and " + std::to_string(l.to));
    g.lines.push_back(g.system.add_block(rl_line(l.R, l.L), "line" + std::to_string(k)));
    A[size_t(l.from) * m + k] = -1.0;
    A[size_t(l.to) * m + k] = 1.0;
  }
  g.system.connect_incidence(g.buses, 0, g.lines, 0, A);
  return g;
}

microgrid make_microgrid(law_ptr dgu, int buses, const std::vector<line_spec> &lines) {
  return make_microgrid(std::vector<law_ptr>(std::max(buses, 0), std::move(dgu)), lines);
}

vecd physical_state(const microgrid &g, const vecd &x) {
  vecd e(g.system.state_dim());
  g.system.readout(x.data(), e.data());
  return e;
}

double peak_line_current(const microgrid &g, const std::vector<vecd> &x) {
  double peak = 0;
  vecd e(g.system.state_dim());
  for (size_t k = 1; k < x.size(); ++k) {
    g.system.readout(x[k].data(), e.data());
    for (int l : g.lines)
      peak = std::max(peak, std::fabs(e[g.system.state_offset(l)]));
  }
  return peak;
}

grid_errors compare_trajectories(const microgrid &g, const std::vector<vecd> &a,
                                 const std::vector<vecd> &b) {
  if (a.size() != b.size())
    throw std::invalid_argument("cvc::ph: compare_trajectories needs trajectories of one length");
  grid_errors r;
  double sv = 0, si = 0, sl = 0;
  size_t nv = 0, nl = 0;
  vecd ea(g.system.state_dim()), eb(g.system.state_dim());
  for (size_t k = 1; k < a.size(); ++k) {
    g.system.readout(a[k].data(), ea.data());
    g.system.readout(b[k].data(), eb.data());
    for (int i : g.buses) {
      const int o = g.system.state_offset(i);
      sv += (ea[o] - eb[o]) * (ea[o] - eb[o]);
      si += (ea[o + 1] - eb[o + 1]) * (ea[o + 1] - eb[o + 1]);
      ++nv;
    }
    for (int l : g.lines) {
      const int o = g.system.state_offset(l);
      sl += (ea[o] - eb[o]) * (ea[o] - eb[o]);
      ++nl;
    }
  }
  r.voltage_rmse = nv ? std::sqrt(sv / double(nv)) : 0.0;
  r.dgu_current_rmse = nv ? std::sqrt(si / double(nv)) : 0.0;
  r.line_current_rmse = nl ? std::sqrt(sl / double(nl)) : 0.0;
  r.peak_line_current_a = peak_line_current(g, a);
  r.peak_line_current_b = peak_line_current(g, b);
  r.peak_line_current_error = std::fabs(r.peak_line_current_a - r.peak_line_current_b);
  return r;
}

} // namespace ph
} // namespace cvc

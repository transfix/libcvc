/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph training (inc/cvc/ph/train.h): the trainable parameters of a canonical law, the laws over
// double and over the tape, and the isolated-DGU calibration.

#include <algorithm>
#include <cmath>
#include <cvc/ph/optim.h>
#include <cvc/ph/rng.h>
#include <cvc/ph/train.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace cvc {
namespace ph {

namespace {

[[noreturn]] void bad(const std::string &what) { throw std::invalid_argument("cvc::ph: " + what); }

void check_trainable(const canonical_spec &s) {
  if (!s.capacitor.empty())
    bad("training: a law with a capacitor curve is not trainable");
}

// Reads the next value of a flattened parameter vector.
struct reader {
  const vecd &v;
  size_t at = 0;
  double next() {
    if (at >= v.size())
      bad("with_parameters: too few parameters");
    return v[at++];
  }
};

template <class T, class Make> mlp_t<T> convert(const mlp &net, Make &&make) {
  mlp_t<T> out;
  for (const dense_layer &L : net.layers) {
    dense_t<T> d;
    d.in = L.in;
    d.out = L.out;
    d.act = L.act;
    for (double w : L.W)
      d.W.push_back(make(w));
    for (double b : L.b)
      d.b.push_back(make(b));
    out.layers.push_back(std::move(d));
  }
  return out;
}

// canonical_spec -> canonical_t<T>, each trainable parameter through make() in trainable order.
template <class T, class Make> canonical_t<T> convert_law(const canonical_spec &s, Make &&make) {
  check_trainable(s);
  canonical_t<T> c;
  c.n = s.n;
  const int n = s.n;
  if (!s.K.empty()) {
    c.K.assign(size_t(n) * n, T(0.0));
    for (int i = 0; i < n; ++i)
      for (int j = i; j < n; ++j) {
        const T k = make(s.K[size_t(i) * n + j]);
        c.K[size_t(i) * n + j] = k;
        c.K[size_t(j) * n + i] = k;
      }
  }
  for (double v : s.center)
    c.center.push_back(make(v));
  if (c.center.empty())
    c.center.assign(n, T(0.0));
  c.v_scale = s.v_scale;
  c.v_net = convert<T>(s.v_net, make);
  c.m_net = convert<T>(s.m_net, make);
  c.d_net = convert<T>(s.d_net, make);
  c.m_min = s.m_min;
  c.d_min = s.d_min;
  if (s.m_net.empty())
    for (double v : s.mass)
      c.mass.push_back(make(v));
  if (s.d_net.empty())
    for (double v : s.damping)
      c.damping.push_back(make(v));
  c.ports = s.ports;
  c.inputs = s.inputs;
  c.B = s.B;
  c.G = s.G;
  c.scheme = s.scheme;
  return c;
}

void write_net(mlp &net, reader &r) {
  for (dense_layer &L : net.layers) {
    for (double &w : L.W)
      w = r.next();
    for (double &b : L.b)
      b = r.next();
  }
}

// A one-DGU law's field, (J - R) e + G u, for the reference's RK4.
void block_field(const block_law &law, const double *x, const double *u, double *dx) {
  const int n = law.state_dim(), k = law.input_dim();
  vecd e(n), J(size_t(n) * n), R(size_t(n) * n), G(size_t(n) * k);
  law.effort(x, e.data());
  law.structure(x, J.data(), R.data());
  law.input_map(x, G.data());
  for (int i = 0; i < n; ++i) {
    double v = 0;
    for (int j = 0; j < n; ++j)
      v += (J[size_t(i) * n + j] - R[size_t(i) * n + j]) * e[j];
    for (int c = 0; c < k; ++c)
      v += G[size_t(i) * k + c] * u[c];
    dx[i] = v;
  }
}

void rk4(const block_law &law, double h, const double *u, double *x) {
  const int n = law.state_dim();
  vecd k1(n), k2(n), k3(n), k4(n), y(n);
  block_field(law, x, u, k1.data());
  for (int i = 0; i < n; ++i)
    y[i] = x[i] + 0.5 * h * k1[i];
  block_field(law, y.data(), u, k2.data());
  for (int i = 0; i < n; ++i)
    y[i] = x[i] + 0.5 * h * k2[i];
  block_field(law, y.data(), u, k3.data());
  for (int i = 0; i < n; ++i)
    y[i] = x[i] + h * k3[i];
  block_field(law, y.data(), u, k4.data());
  for (int i = 0; i < n; ++i)
    x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
}

// The six measured channels of a reference DGU at x (its R from structure()).
void reference_channels(const block_law &law, const double *x, const double *u, double *y) {
  double e[2], J[4], R[4];
  law.effort(x, e);
  law.structure(x, J, R);
  y[0] = x[0];
  y[1] = x[1];
  y[2] = u[1];
  y[3] = u[1] - R[3] * e[1];
  y[4] = e[0];
  y[5] = -e[1];
}

double softplus_inverse(double y) { return y > 20 ? y : std::log(std::expm1(y)); }

mlp random_net(int in, const std::vector<int> &hidden, int out, activation act, rng &g) {
  mlp net;
  int dim = in;
  for (int h : hidden) {
    dense_layer L;
    L.in = dim;
    L.out = h;
    L.act = act;
    const double scale = 1.0 / std::sqrt(double(dim));
    for (int i = 0; i < h * dim; ++i)
      L.W.push_back(scale * g.normal());
    L.b.assign(h, 0.0);
    net.layers.push_back(std::move(L));
    dim = h;
  }
  dense_layer last; // starts at zero: the net's output is its bias alone
  last.in = dim;
  last.out = out;
  last.W.assign(size_t(dim) * out, 0.0);
  last.b.assign(out, 0.0);
  net.layers.push_back(std::move(last));
  return net;
}

} // namespace

// ── the trainable parameters ────────────────────────────────────────────────────────────────────

vecd trainable_parameters(const canonical_spec &s) {
  vecd theta;
  convert_law<double>(s, [&](double v) {
    theta.push_back(v);
    return v;
  });
  return theta;
}

canonical_spec with_parameters(const canonical_spec &shape, const vecd &theta) {
  check_trainable(shape);
  canonical_spec s = shape;
  reader r{theta};
  const int n = s.n;
  if (!s.K.empty())
    for (int i = 0; i < n; ++i)
      for (int j = i; j < n; ++j)
        s.K[size_t(i) * n + j] = s.K[size_t(j) * n + i] = r.next();
  if (s.center.empty())
    s.center.assign(n, 0.0);
  else
    for (double &c : s.center)
      c = r.next();
  write_net(s.v_net, r);
  write_net(s.m_net, r);
  write_net(s.d_net, r);
  if (s.m_net.empty())
    for (double &m : s.mass)
      m = r.next();
  if (s.d_net.empty())
    for (double &d : s.damping)
      d = r.next();
  if (r.at != theta.size())
    bad("with_parameters: " + std::to_string(theta.size()) + " parameters for a law with " +
        std::to_string(r.at));
  return s;
}

canonical_t<double> law_over_double(const canonical_spec &s) {
  return convert_law<double>(s, [](double v) { return v; });
}

canonical_t<ad::var> law_over_tape(const canonical_spec &s, ad::tape &t,
                                   std::vector<ad::var> &vars) {
  vars.clear();
  return convert_law<ad::var>(s, [&](double v) {
    vars.push_back(ad::variable(t, v));
    return vars.back();
  });
}

// ── the isolated-DGU calibration ────────────────────────────────────────────────────────────────

dgu_dataset make_dgu_dataset(const block_law &reference, const dgu_dataset_options &o) {
  if (reference.state_dim() != 2 || reference.input_dim() != 2)
    bad("make_dgu_dataset: the reference is a DGU (state (q, phi), inputs (I_in, V_src))");
  if (o.trajectories < 1 || o.transitions < 1 || !(o.dt > 0) || o.refine < 1)
    bad("make_dgu_dataset: needs trajectories, transitions, dt > 0 and refine >= 1");
  dgu_dataset d;
  d.dt = o.dt;
  d.I_in = o.I_in;
  d.V_src = o.V_src;
  const double u[2] = {o.I_in, o.V_src};
  rng g(o.seed);
  for (int r = 0; r < o.trajectories; ++r) {
    const double v = o.v_lo + (o.v_hi - o.v_lo) * g.uniform();
    const double phi = o.phi_lo + (o.phi_hi - o.phi_lo) * g.uniform();
    // The charge whose voltage is v at this flux: Newton on dH/dq(q, phi) = v.
    double x[2] = {v, phi};
    for (int it = 0; it < 60; ++it) {
      double e[2];
      reference.effort(x, e);
      const double f = e[0] - v, h = 1e-6 * (1 + std::fabs(x[0]));
      double xp[2] = {x[0] + h, phi}, xm[2] = {x[0] - h, phi}, ep[2], em[2];
      reference.effort(xp, ep);
      reference.effort(xm, em);
      const double df = (ep[0] - em[0]) / (2 * h);
      if (!(std::fabs(df) > 0))
        bad("make_dgu_dataset: the reference's voltage does not depend on its charge");
      x[0] -= f / df;
      if (std::fabs(f) < 1e-13)
        break;
    }
    std::vector<vecd> run{vecd(x, x + 2)};
    for (int k = 0; k < o.transitions; ++k) {
      dgu_sample s;
      s.x[0] = x[0];
      s.x[1] = x[1];
      s.u[0] = u[0];
      s.u[1] = u[1];
      reference_channels(reference, x, u, s.y);
      for (int j = 0; j < o.refine; ++j)
        rk4(reference, o.dt / o.refine, u, x);
      reference_channels(reference, x, u, s.y_next);
      d.samples.push_back(s);
      run.push_back(vecd(x, x + 2));
    }
    d.runs.push_back(std::move(run));
  }
  return d;
}

linear_fit fit_linear_dgu(const dgu_dataset &d) {
  double qv = 0, vv = 0, pi = 0, ii = 0, di = 0;
  for (const dgu_sample &s : d.samples) {
    const double q = s.y[0], phi = s.y[1], v = s.y[4], i = -s.y[5], drop = s.y[2] - s.y[3];
    qv += q * v;
    vv += v * v;
    pi += phi * i;
    ii += i * i;
    di += drop * i;
  }
  if (!(vv > 0) || !(ii > 0))
    bad("fit_linear_dgu: the data have no voltage or no current");
  linear_fit f;
  f.C = qv / vv;
  f.L = pi / ii;
  f.R = di / ii;
  return f;
}

canonical_spec linear_dgu_spec(const linear_fit &fit) {
  canonical_spec s;
  s.n = 1;
  s.K = {1.0 / fit.C};
  s.center = {0.0};
  s.mass = {fit.L};
  s.damping = {fit.R};
  s.ports = 1;
  s.B = {1.0, 0.0};
  s.inputs = 2;
  s.G = {1.0, 0.0, 0.0, 1.0};
  s.scheme = conservative_scheme::implicit_midpoint;
  return s;
}

canonical_spec warm_start_dgu(const linear_fit &fit, const dgu_family &f) {
  if (!(fit.C > 0) || !(fit.L > f.m_min) || !(fit.R > f.d_min))
    bad("warm_start_dgu: the linear fit needs C > 0, L > m_min and R > d_min");
  rng g(f.seed);
  canonical_spec s = linear_dgu_spec(fit);
  s.mass.clear();
  s.damping.clear();
  s.v_scale = f.v_scale;
  s.v_net = random_net(1, f.v_hidden, 1, f.act, g);
  s.m_net = random_net(1, f.m_hidden, 1, f.act, g);
  s.m_net.layers.back().b = {softplus_inverse(fit.L - f.m_min)};
  s.m_min = f.m_min;
  s.d_net = random_net(1, f.d_hidden, 1, f.act, g);
  s.d_net.layers.back().b = {softplus_inverse(fit.R - f.d_min)};
  s.d_min = f.d_min;
  return s;
}

namespace {

// One sample's weighted loss terms on a tape; returns (constraint, step) values and leaves the
// combined loss's node in *out.
std::pair<double, double> sample_loss(const canonical_t<ad::var> &law, const dgu_sample &s,
                                      const dgu_loss_weights &w, double scale, ad::var *out) {
  using ad::var;
  var lc(0.0), ls(0.0);
  if (w.constraint != 0) {
    var x[2] = {var(s.x[0]), var(s.x[1])}, e[2], d[1];
    law.effort(x, e);
    law.damping_of(x, d);
    const double e1 = s.y[2], e2 = s.y[3], e3 = s.y[4], jV = s.y[5];
    const var r1 = e[1] + var(jV), r2 = var(e2 - e1) / d[0] - var(jV), r3 = e[0] - var(e3);
    lc = r1 * r1 + r2 * r2 + r3 * r3;
  }
  if (w.step != 0) {
    var x[2] = {var(s.x[0]), var(s.x[1])}, y[6];
    law.local_step(w.dt, w.nsub, s.u, x, w.solve);
    dgu_observe(law, x, s.u, y);
    for (int c = 0; c < 6; ++c) {
      const var r = y[c] - var(s.y_next[c]);
      ls += r * r;
    }
  }
  *out = var(scale) * (var(w.step) * ls + var(w.constraint) * lc);
  return {ad::value(lc), ad::value(ls)};
}

} // namespace

dgu_losses dgu_loss(const canonical_spec &law, const dgu_dataset &d,
                    const std::vector<int> &samples, const dgu_loss_weights &w, vecd *grad,
                    int threads) {
  std::vector<int> idx = samples;
  if (idx.empty())
    for (size_t i = 0; i < d.samples.size(); ++i)
      idx.push_back(static_cast<int>(i));
  const vecd theta = trainable_parameters(law);
  const size_t P = theta.size();
  const int T = std::max(1, std::min(threads, static_cast<int>(idx.size())));
  const double scale = 1.0 / double(idx.size());
  std::vector<vecd> g(T, vecd(grad ? P : 0, 0.0));
  std::vector<double> lc(T, 0.0), ls(T, 0.0);
  std::vector<std::string> errors(T);
  auto work = [&](int t) {
    try {
      ad::tape tape;
      std::vector<ad::var> vars;
      for (size_t k = t; k < idx.size(); k += T) {
        const int si = idx[k];
        if (si < 0 || si >= static_cast<int>(d.samples.size()))
          bad("dgu_loss: sample index out of range");
        tape.clear();
        const canonical_t<ad::var> net = law_over_tape(law, tape, vars);
        ad::var out;
        const auto v = sample_loss(net, d.samples[si], w, scale, &out);
        lc[t] += v.first;
        ls[t] += v.second;
        if (grad && !out.constant()) {
          tape.backward(out.i);
          for (size_t p = 0; p < P; ++p)
            g[t][p] += tape.adjoint(vars[p].i);
        }
      }
    } catch (const std::exception &e) {
      errors[t] = e.what();
    }
  };
  if (T == 1) {
    work(0);
  } else {
    std::vector<std::thread> pool;
    for (int t = 0; t < T; ++t)
      pool.emplace_back(work, t);
    for (std::thread &th : pool)
      th.join();
  }
  for (const std::string &e : errors)
    if (!e.empty())
      throw std::runtime_error(e);
  dgu_losses r;
  for (int t = 0; t < T; ++t) {
    r.constraint += lc[t] * scale;
    r.step += ls[t] * scale;
  }
  double sq = 0;
  for (double v : theta)
    sq += v * v;
  r.reg = w.reg * sq;
  r.total = w.step * r.step + w.constraint * r.constraint + r.reg;
  if (grad) {
    grad->assign(P, 0.0);
    for (int t = 0; t < T; ++t)
      for (size_t p = 0; p < P; ++p)
        (*grad)[p] += g[t][p];
    for (size_t p = 0; p < P; ++p)
      (*grad)[p] += 2 * w.reg * theta[p];
  }
  return r;
}

dgu_train_result train_dgu(const canonical_spec &init, const dgu_dataset &train,
                           const dgu_train_options &o) {
  if (train.samples.empty())
    bad("train_dgu: no samples");
  if (o.batch < 1 || o.updates < 0 || o.constraint_first < 0)
    bad("train_dgu: needs batch >= 1, updates >= 0 and constraint_first >= 0");
  vecd theta = trainable_parameters(init);
  adam opt;
  opt.init(theta.size());
  rng g(o.seed);
  dgu_train_result res;
  res.parameters = static_cast<int>(theta.size());
  vecd grad;
  std::vector<int> batch(std::min<size_t>(size_t(o.batch), train.samples.size()));
  const int total = o.constraint_first + o.updates;
  for (int u = 0; u < total; ++u) {
    for (int &b : batch)
      b = static_cast<int>(g.uniform() * double(train.samples.size())) %
          static_cast<int>(train.samples.size());
    dgu_loss_weights w = o.weights;
    if (u < o.constraint_first) { // L_pre = L_constraint + L_reg
      w.step = 0;
      w.constraint = 1;
    }
    const canonical_spec law = with_parameters(init, theta);
    const dgu_losses l = dgu_loss(law, train, batch, w, &grad, o.threads);
    bool finite = std::isfinite(l.total);
    for (double v : grad)
      finite = finite && std::isfinite(v);
    if (!finite)
      throw std::runtime_error("cvc::ph: train_dgu: the loss left the reals at update " +
                               std::to_string(u));
    opt.update(theta.data(), grad.data(), theta.size(), o.lr);
    if ((o.log_every > 0 && (u + 1) % o.log_every == 0) || u + 1 == total)
      res.history.push_back({u + 1, l});
  }
  res.law = with_parameters(init, theta);
  return res;
}

dgu_eval evaluate_dgu(const block_law &law, const block_law &reference, const dgu_dataset &test,
                      int nsub, const solve_opts &solve) {
  const double u[2] = {test.I_in, test.V_src};
  double sv = 0, si = 0;
  size_t count = 0;
  dgu_eval r;
  for (const std::vector<vecd> &run : test.runs) {
    vecd x = run.front();
    for (size_t k = 1; k < run.size(); ++k) {
      law.local_step(test.dt, nsub, u, x.data(), solve);
      double e[2], et[2];
      law.effort(x.data(), e);
      reference.effort(run[k].data(), et);
      if (!std::isfinite(e[0]) || !std::isfinite(e[1])) {
        r.finite = false;
        break;
      }
      sv += (e[0] - et[0]) * (e[0] - et[0]);
      si += (e[1] - et[1]) * (e[1] - et[1]);
      ++count;
    }
  }
  if (count) {
    r.voltage_rmse = std::sqrt(sv / double(count));
    r.current_rmse = std::sqrt(si / double(count));
  }
  return r;
}

} // namespace ph
} // namespace cvc

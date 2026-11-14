/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph composition (inc/cvc/ph/compose.h): component laws, the assembled pairing, the composed
// field, the split transition and its books, and the "cvc-ph-law/1" / "cvc-ph-system/1" files.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <cvc/ph/rng.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace ph {

namespace {

[[noreturn]] void bad(const std::string &what) { throw std::invalid_argument("cvc::ph: " + what); }

// Solves A x = b (n x n, row-major) in place by Gaussian elimination with partial pivoting; A is
// overwritten. Throws on a singular matrix.
void solve_in_place(int n, double *A, double *b) {
  for (int k = 0; k < n; ++k) {
    int piv = k;
    double best = std::fabs(A[k * n + k]);
    for (int r = k + 1; r < n; ++r)
      if (std::fabs(A[r * n + k]) > best) {
        best = std::fabs(A[r * n + k]);
        piv = r;
      }
    if (!(best > 0))
      throw std::runtime_error("cvc::ph: singular step matrix");
    if (piv != k) {
      for (int c = 0; c < n; ++c)
        std::swap(A[k * n + c], A[piv * n + c]);
      std::swap(b[k], b[piv]);
    }
    for (int r = k + 1; r < n; ++r) {
      const double f = A[r * n + k] / A[k * n + k];
      if (f == 0)
        continue;
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
}

// y = A x for A (r x c, row-major).
void matvec(int r, int c, const double *A, const double *x, double *y) {
  for (int i = 0; i < r; ++i) {
    double s = 0;
    for (int j = 0; j < c; ++j)
      s += A[i * c + j] * x[j];
    y[i] = s;
  }
}

// y = A^T x for A (r x c, row-major).
void matvec_t(int r, int c, const double *A, const double *x, double *y) {
  for (int j = 0; j < c; ++j)
    y[j] = 0;
  for (int i = 0; i < r; ++i)
    for (int j = 0; j < c; ++j)
      y[j] += A[i * c + j] * x[i];
}

double dot(int n, const double *a, const double *b) {
  double s = 0;
  for (int i = 0; i < n; ++i)
    s += a[i] * b[i];
  return s;
}

// torch's softplus (beta 1, threshold 20) and the logistic function.
double softplus(double z) { return z > 20 ? z : std::log1p(std::exp(z)); }
double sigmoid(double z) {
  if (z >= 0)
    return 1 / (1 + std::exp(-z));
  const double e = std::exp(z);
  return e / (1 + e);
}

void check_size(const vecd &v, size_t n, const std::string &what) {
  if (v.size() != n)
    bad(what + " has " + std::to_string(v.size()) + " entries, expected " + std::to_string(n));
}

void check_net(const mlp &net, int in, int out, const std::string &what) {
  if (net.empty())
    return;
  int dim = in;
  for (size_t l = 0; l < net.layers.size(); ++l) {
    const dense_layer &L = net.layers[l];
    if (L.in != dim)
      bad(what + " layer " + std::to_string(l) + " takes " + std::to_string(L.in) +
          " inputs, expected " + std::to_string(dim));
    if (L.out < 1)
      bad(what + " layer " + std::to_string(l) + " has no outputs");
    check_size(L.W, size_t(L.out) * size_t(L.in), what + " layer " + std::to_string(l) + " W");
    check_size(L.b, size_t(L.out), what + " layer " + std::to_string(l) + " b");
    dim = L.out;
  }
  if (dim != out)
    bad(what + " gives " + std::to_string(dim) + " outputs, expected " + std::to_string(out));
}

} // namespace

// ── block_law ───────────────────────────────────────────────────────────────────────────────────

void block_law::kick(double tau, const double *u, double *x) const {
  const int n = state_dim(), k = input_dim();
  if (k == 0)
    return;
  vecd G(size_t(n) * size_t(k)), g(n);
  input_map(x, G.data());
  matvec(n, k, G.data(), u, g.data());
  for (int i = 0; i < n; ++i)
    x[i] += tau * g[i];
}

int block_law::local_step(double dt, int nsub, const double *u, double *x,
                          const solve_opts &o) const {
  if (nsub < 1)
    bad("local_step needs nsub >= 1");
  const double d = dt / nsub;
  int worst = 0;
  bool failed = false;
  for (int r = 0; r < nsub; ++r) {
    kick(0.5 * d, u, x);
    damp(0.5 * d, x);
    const int it = conserve(d, x, o);
    failed = failed || it < 0;
    worst = std::max(worst, std::abs(it));
    damp(0.5 * d, x);
    kick(0.5 * d, u, x);
  }
  return failed ? -worst : worst;
}

// ── linear_law ──────────────────────────────────────────────────────────────────────────────────

linear_law::linear_law(linear_spec s) : s_(std::move(s)) {
  const int n = s_.n;
  if (n < 1)
    bad("linear law needs n >= 1");
  if (s_.ports < 0 || s_.inputs < 0)
    bad("linear law port and input counts must be >= 0");
  const size_t nn = size_t(n) * size_t(n);
  check_size(s_.Q, nn, "linear law Q");
  if (s_.J.empty())
    s_.J.assign(nn, 0.0);
  if (s_.R.empty())
    s_.R.assign(nn, 0.0);
  check_size(s_.J, nn, "linear law J");
  check_size(s_.R, nn, "linear law R");
  check_size(s_.B, size_t(n) * size_t(s_.ports), "linear law B");
  check_size(s_.G, size_t(n) * size_t(s_.inputs), "linear law G");
  for (int i = 0; i < n; ++i) {
    if (s_.R[i * n + i] < 0)
      bad("linear law R has a negative diagonal entry");
    for (int j = 0; j < n; ++j) {
      const double scale =
          1 + std::fabs(s_.J[i * n + j]) + std::fabs(s_.R[i * n + j]) + std::fabs(s_.Q[i * n + j]);
      if (std::fabs(s_.J[i * n + j] + s_.J[j * n + i]) > 1e-12 * scale)
        bad("linear law J is not skew");
      if (std::fabs(s_.R[i * n + j] - s_.R[j * n + i]) > 1e-12 * scale)
        bad("linear law R is not symmetric");
      if (std::fabs(s_.Q[i * n + j] - s_.Q[j * n + i]) > 1e-12 * scale)
        bad("linear law Q is not symmetric");
    }
  }
}

double linear_law::energy(const double *x) const {
  vecd e(s_.n);
  matvec(s_.n, s_.n, s_.Q.data(), x, e.data());
  return 0.5 * dot(s_.n, x, e.data());
}

void linear_law::effort(const double *x, double *e) const { matvec(s_.n, s_.n, s_.Q.data(), x, e); }

void linear_law::structure(const double *, double *J, double *R) const {
  std::copy(s_.J.begin(), s_.J.end(), J);
  std::copy(s_.R.begin(), s_.R.end(), R);
}

void linear_law::port_map(const double *, double *B) const {
  std::copy(s_.B.begin(), s_.B.end(), B);
}
void linear_law::input_map(const double *, double *G) const {
  std::copy(s_.G.begin(), s_.G.end(), G);
}

// x <- (I + c A)^-1 (I - c A) x with A = R Q and c = tau/2: the trapezoid rule for x' = -A x.
void linear_law::damp(double tau, double *x) const {
  const int n = s_.n;
  if (std::all_of(s_.R.begin(), s_.R.end(), [](double v) { return v == 0; }))
    return;
  const double c = 0.5 * tau;
  vecd A(size_t(n) * size_t(n), 0.0), lhs(A.size()), rhs(n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      for (int k = 0; k < n; ++k)
        A[i * n + j] += s_.R[i * n + k] * s_.Q[k * n + j];
  for (int i = 0; i < n; ++i) {
    double s = x[i];
    for (int j = 0; j < n; ++j) {
      s -= c * A[i * n + j] * x[j];
      lhs[i * n + j] = (i == j ? 1.0 : 0.0) + c * A[i * n + j];
    }
    rhs[i] = s;
  }
  solve_in_place(n, lhs.data(), rhs.data());
  std::copy(rhs.begin(), rhs.end(), x);
}

// Implicit midpoint for x' = J Q x, solved exactly: x <- (I - tau/2 A)^-1 (I + tau/2 A) x.
int linear_law::conserve(double tau, double *x, const solve_opts &) const {
  const int n = s_.n;
  if (std::all_of(s_.J.begin(), s_.J.end(), [](double v) { return v == 0; }))
    return 0;
  const double c = 0.5 * tau;
  vecd A(size_t(n) * size_t(n), 0.0), lhs(A.size()), rhs(n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      for (int k = 0; k < n; ++k)
        A[i * n + j] += s_.J[i * n + k] * s_.Q[k * n + j];
  for (int i = 0; i < n; ++i) {
    double s = x[i];
    for (int j = 0; j < n; ++j) {
      s += c * A[i * n + j] * x[j];
      lhs[i * n + j] = (i == j ? 1.0 : 0.0) - c * A[i * n + j];
    }
    rhs[i] = s;
  }
  solve_in_place(n, lhs.data(), rhs.data());
  std::copy(rhs.begin(), rhs.end(), x);
  return 0;
}

// ── mlp ─────────────────────────────────────────────────────────────────────────────────────────

void mlp::eval(const double *x, double *y, double *jac) const {
  if (layers.empty())
    bad("evaluating an empty network");
  const int in = layers.front().in;
  vecd a(x, x + in), T; // T: the tangent of a with respect to x, dim(a) x in
  if (jac) {
    T.assign(size_t(in) * size_t(in), 0.0);
    for (int i = 0; i < in; ++i)
      T[i * in + i] = 1.0;
  }
  vecd z, Tz;
  for (const dense_layer &L : layers) {
    z.assign(L.out, 0.0);
    for (int o = 0; o < L.out; ++o) {
      double s = L.b[o];
      for (int i = 0; i < L.in; ++i)
        s += L.W[o * L.in + i] * a[i];
      z[o] = s;
    }
    if (jac) {
      Tz.assign(size_t(L.out) * size_t(in), 0.0);
      for (int o = 0; o < L.out; ++o)
        for (int i = 0; i < L.in; ++i) {
          const double w = L.W[o * L.in + i];
          if (w == 0)
            continue;
          for (int c = 0; c < in; ++c)
            Tz[o * in + c] += w * T[i * in + c];
        }
    }
    a.resize(L.out);
    for (int o = 0; o < L.out; ++o) {
      const double v = z[o];
      double f = v, df = 1.0;
      switch (L.act) {
      case activation::identity:
        break;
      case activation::silu: {
        const double s = sigmoid(v);
        f = v * s;
        df = s * (1 + v * (1 - s));
        break;
      }
      case activation::tanh:
        f = std::tanh(v);
        df = 1 - f * f;
        break;
      case activation::softplus:
        f = softplus(v);
        df = v > 20 ? 1.0 : sigmoid(v);
        break;
      case activation::sigmoid:
        f = sigmoid(v);
        df = f * (1 - f);
        break;
      }
      a[o] = f;
      if (jac)
        for (int c = 0; c < in; ++c)
          Tz[o * in + c] *= df;
    }
    if (jac)
      T.swap(Tz);
  }
  std::copy(a.begin(), a.end(), y);
  if (jac)
    std::copy(T.begin(), T.end(), jac);
}

// ── canonical_law ───────────────────────────────────────────────────────────────────────────────

canonical_law::canonical_law(canonical_spec s) : s_(std::move(s)) {
  const int n = s_.n;
  if (n < 1)
    bad("canonical law needs n >= 1");
  if (s_.ports < 0 || s_.inputs < 0)
    bad("canonical law port and input counts must be >= 0");
  if (!s_.K.empty()) {
    check_size(s_.K, size_t(n) * size_t(n), "canonical law K");
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < i; ++j)
        if (std::fabs(s_.K[i * n + j] - s_.K[j * n + i]) > 1e-12 * (1 + std::fabs(s_.K[i * n + j])))
          bad("canonical law K is not symmetric");
  }
  if (s_.center.empty())
    s_.center.assign(n, 0.0);
  check_size(s_.center, size_t(n), "canonical law center");
  check_net(s_.v_net, n, 1, "canonical law potential net");
  if (s_.m_net.empty()) {
    check_size(s_.mass, size_t(n), "canonical law mass");
    for (double m : s_.mass)
      if (!(m > 0))
        bad("canonical law mass must be positive");
  } else {
    check_net(s_.m_net, n, n, "canonical law mass net");
    if (!(s_.m_min > 0))
      bad("canonical law mass net needs m_min > 0");
  }
  if (s_.d_net.empty()) {
    if (!s_.damping.empty()) {
      check_size(s_.damping, size_t(n), "canonical law damping");
      for (double d : s_.damping)
        if (d < 0)
          bad("canonical law damping must be >= 0");
    }
  } else {
    check_net(s_.d_net, n, n, "canonical law damping net");
    if (s_.d_min < 0)
      bad("canonical law damping net needs d_min >= 0");
  }
  check_size(s_.B, size_t(2 * n) * size_t(s_.ports), "canonical law B");
  check_size(s_.G, size_t(2 * n) * size_t(s_.inputs), "canonical law G");
  if (s_.scheme == conservative_scheme::leapfrog && !s_.m_net.empty())
    bad("leapfrog needs a constant mass; use implicit_midpoint with a mass net");
}

double canonical_law::potential(const double *q, double *grad) const {
  const int n = s_.n;
  double V = 0;
  if (grad)
    std::fill(grad, grad + n, 0.0);
  if (!s_.K.empty()) {
    vecd d(n), Kd(n);
    for (int i = 0; i < n; ++i)
      d[i] = q[i] - s_.center[i];
    matvec(n, n, s_.K.data(), d.data(), Kd.data());
    V += 0.5 * dot(n, d.data(), Kd.data());
    if (grad)
      for (int i = 0; i < n; ++i)
        grad[i] += Kd[i];
  }
  if (!s_.v_net.empty()) {
    double y = 0;
    vecd J(n);
    s_.v_net.eval(q, &y, grad ? J.data() : nullptr);
    V += s_.v_scale * y;
    if (grad)
      for (int i = 0; i < n; ++i)
        grad[i] += s_.v_scale * J[i];
  }
  return V;
}

void canonical_law::mass(const double *q, double *m, double *dm) const {
  const int n = s_.n;
  if (s_.m_net.empty()) {
    std::copy(s_.mass.begin(), s_.mass.end(), m);
    if (dm)
      std::fill(dm, dm + n * n, 0.0);
    return;
  }
  vecd z(n), J(dm ? size_t(n) * size_t(n) : 0);
  s_.m_net.eval(q, z.data(), dm ? J.data() : nullptr);
  for (int j = 0; j < n; ++j) {
    m[j] = softplus(z[j]) + s_.m_min;
    if (dm) {
      const double s = z[j] > 20 ? 1.0 : sigmoid(z[j]);
      for (int i = 0; i < n; ++i)
        dm[j * n + i] = s * J[j * n + i];
    }
  }
}

void canonical_law::damping(const double *q, double *d) const {
  const int n = s_.n;
  if (!s_.d_net.empty()) {
    s_.d_net.eval(q, d, nullptr);
    for (int j = 0; j < n; ++j)
      d[j] = softplus(d[j]) + s_.d_min;
  } else if (!s_.damping.empty()) {
    std::copy(s_.damping.begin(), s_.damping.end(), d);
  } else {
    std::fill(d, d + n, 0.0);
  }
}

double canonical_law::energy(const double *x) const {
  const int n = s_.n;
  vecd m(n);
  mass(x, m.data());
  double T = 0;
  for (int j = 0; j < n; ++j)
    T += 0.5 * x[n + j] * x[n + j] / m[j];
  return potential(x, nullptr) + T;
}

void canonical_law::effort(const double *x, double *e) const {
  const int n = s_.n;
  const double *q = x, *p = x + n;
  vecd m(n), dm(size_t(n) * size_t(n));
  const bool varying = !s_.m_net.empty();
  mass(q, m.data(), varying ? dm.data() : nullptr);
  potential(q, e);
  if (varying)
    for (int j = 0; j < n; ++j) {
      const double w = -0.5 * p[j] * p[j] / (m[j] * m[j]);
      for (int i = 0; i < n; ++i)
        e[i] += w * dm[j * n + i];
    }
  for (int j = 0; j < n; ++j)
    e[n + j] = p[j] / m[j];
}

void canonical_law::structure(const double *x, double *J, double *R) const {
  const int n = s_.n, N = 2 * n;
  std::fill(J, J + N * N, 0.0);
  std::fill(R, R + N * N, 0.0);
  for (int i = 0; i < n; ++i) {
    J[i * N + n + i] = 1.0;
    J[(n + i) * N + i] = -1.0;
  }
  vecd d(n);
  damping(x, d.data());
  for (int j = 0; j < n; ++j)
    R[(n + j) * N + n + j] = d[j];
}

void canonical_law::port_map(const double *, double *B) const {
  std::copy(s_.B.begin(), s_.B.end(), B);
}
void canonical_law::input_map(const double *, double *G) const {
  std::copy(s_.G.begin(), s_.G.end(), G);
}

void canonical_law::damp(double tau, double *x) const {
  const int n = s_.n;
  if (s_.d_net.empty() && s_.damping.empty())
    return;
  vecd m(n), d(n);
  mass(x, m.data());
  damping(x, d.data());
  const double c = 0.5 * tau;
  for (int j = 0; j < n; ++j) {
    const double a = c * d[j] / m[j];
    x[n + j] *= (1 - a) / (1 + a);
  }
}

int canonical_law::conserve(double tau, double *x, const solve_opts &o) const {
  const int n = s_.n, N = 2 * n;
  double *q = x, *p = x + n;
  vecd e(N);
  if (s_.scheme == conservative_scheme::leapfrog) {
    vecd g(n), m(n);
    mass(q, m.data());
    potential(q, g.data());
    for (int j = 0; j < n; ++j)
      p[j] -= 0.5 * tau * g[j];
    for (int j = 0; j < n; ++j)
      q[j] += tau * p[j] / m[j];
    potential(q, g.data());
    for (int j = 0; j < n; ++j)
      p[j] -= 0.5 * tau * g[j];
    return 0;
  }
  // Implicit midpoint: q+ = q + tau dH/dp(mid), p+ = p - tau dH/dq(mid), by Picard iteration from
  // the explicit-Euler prediction.
  vecd x1(N), x2(N), mid(N);
  effort(x, e.data());
  for (int j = 0; j < n; ++j) {
    x1[j] = q[j] + tau * e[n + j];
    x1[n + j] = p[j] - tau * e[j];
  }
  for (int it = 1; it <= o.max_iter; ++it) {
    for (int i = 0; i < N; ++i)
      mid[i] = 0.5 * (x[i] + x1[i]);
    effort(mid.data(), e.data());
    double dq = 0, dp = 0;
    for (int j = 0; j < n; ++j) {
      x2[j] = q[j] + tau * e[n + j];
      x2[n + j] = p[j] - tau * e[j];
      dq = std::max(dq, std::fabs(x2[j] - x1[j]));
      dp = std::max(dp, std::fabs(x2[n + j] - x1[n + j]));
    }
    x1.swap(x2);
    if (dq < o.tol && dp < o.tol) {
      std::copy(x1.begin(), x1.end(), x);
      return it;
    }
  }
  std::copy(x1.begin(), x1.end(), x);
  return -o.max_iter;
}

// ── system ──────────────────────────────────────────────────────────────────────────────────────

void composite::grow_pairing(int new_ny) {
  vecd C(size_t(new_ny) * size_t(new_ny), 0.0);
  for (int a = 0; a < ny_; ++a)
    for (int b = 0; b < ny_; ++b)
      C[a * new_ny + b] = C_[a * ny_ + b];
  C_.swap(C);
}

int composite::add_block(law_ptr law, std::string name) {
  if (!law)
    bad("add_block: null law");
  if (!name.empty() && find(name) >= 0)
    bad("add_block: duplicate block name '" + name + "'");
  const int ny = ny_ + law->port_dim();
  grow_pairing(ny);
  xoff_.push_back(nx_);
  uoff_.push_back(nu_);
  yoff_.push_back(ny_);
  nx_ += law->state_dim();
  nu_ += law->input_dim();
  ny_ = ny;
  laws_.push_back(std::move(law));
  names_.push_back(std::move(name));
  return blocks() - 1;
}

int composite::find(const std::string &name) const {
  for (int i = 0; i < blocks(); ++i)
    if (names_[i] == name)
      return i;
  return -1;
}

void composite::connect(int i, int a, int j, int b, double c) {
  if (i < 0 || i >= blocks() || j < 0 || j >= blocks())
    bad("connect: block index out of range");
  if (i == j)
    bad("connect: a block's ports do not pair with each other (C_ii = 0)");
  if (a < 0 || a >= laws_[i]->port_dim() || b < 0 || b >= laws_[j]->port_dim())
    bad("connect: port index out of range");
  const int r = yoff_[i] + a, s = yoff_[j] + b;
  C_[size_t(r) * ny_ + s] += c;
  C_[size_t(s) * ny_ + r] -= c;
}

void composite::connect_incidence(const std::vector<int> &buses, int bus_port,
                                  const std::vector<int> &lines, int line_port, const vecd &A) {
  const size_t n = buses.size(), m = lines.size();
  check_size(A, n * m, "incidence matrix");
  for (size_t k = 0; k < n; ++k)
    for (size_t l = 0; l < m; ++l)
      if (A[k * m + l] != 0)
        connect(buses[k], bus_port, lines[l], line_port, -A[k * m + l]);
}

void composite::connect_skew(const std::vector<int> &blk, int port, const vecd &C) {
  const size_t s = blk.size();
  check_size(C, s * s, "skew coupling matrix");
  for (size_t a = 0; a < s; ++a)
    for (size_t b = a + 1; b < s; ++b) {
      const double w = 0.5 * (C[a * s + b] - C[b * s + a]);
      if (w != 0)
        connect(blk[a], port, blk[b], port, w);
    }
}

double composite::energy(const double *x) const {
  double H = 0;
  for (int i = 0; i < blocks(); ++i)
    H += laws_[i]->energy(x + xoff_[i]);
  return H;
}

void composite::effort(const double *x, double *e) const {
  for (int i = 0; i < blocks(); ++i)
    laws_[i]->effort(x + xoff_[i], e + xoff_[i]);
}

void composite::outputs(const double *x, double *y) const {
  vecd e(nx_), B;
  effort(x, e.data());
  for (int i = 0; i < blocks(); ++i) {
    const block_law &L = *laws_[i];
    const int n = L.state_dim(), m = L.port_dim();
    if (m == 0)
      continue;
    B.resize(size_t(n) * size_t(m));
    L.port_map(x + xoff_[i], B.data());
    matvec_t(n, m, B.data(), e.data() + xoff_[i], y + yoff_[i]);
  }
}

namespace {

// e, y_hat and u_hat = -C y_hat at x.
void coupling_inputs(const composite &s, const double *x, vecd &e, vecd &y, vecd &uh) {
  e.assign(s.state_dim(), 0.0);
  y.assign(s.port_dim(), 0.0);
  uh.assign(s.port_dim(), 0.0);
  s.effort(x, e.data());
  vecd B;
  for (int i = 0; i < s.blocks(); ++i) {
    const block_law &L = s.law(i);
    const int n = L.state_dim(), m = L.port_dim();
    if (m == 0)
      continue;
    B.resize(size_t(n) * size_t(m));
    L.port_map(x + s.state_offset(i), B.data());
    matvec_t(n, m, B.data(), e.data() + s.state_offset(i), y.data() + s.port_offset(i));
  }
  const int ny = s.port_dim();
  const vecd &C = s.pairing();
  for (int a = 0; a < ny; ++a) {
    double v = 0;
    for (int b = 0; b < ny; ++b)
      v -= C[size_t(a) * ny + b] * y[b];
    uh[a] = v;
  }
}

} // namespace

void composite::field(const double *x, const double *u, double *dx) const {
  vecd e, y, uh, J, R, B, G, t;
  coupling_inputs(*this, x, e, y, uh);
  for (int i = 0; i < blocks(); ++i) {
    const block_law &L = *laws_[i];
    const int n = L.state_dim(), m = L.port_dim(), k = L.input_dim();
    const double *xi = x + xoff_[i], *ei = e.data() + xoff_[i];
    double *di = dx + xoff_[i];
    J.resize(size_t(n) * size_t(n));
    R.resize(J.size());
    L.structure(xi, J.data(), R.data());
    for (int r = 0; r < n; ++r) {
      double v = 0;
      for (int c = 0; c < n; ++c)
        v += (J[r * n + c] - R[r * n + c]) * ei[c];
      di[r] = v;
    }
    t.resize(n);
    if (m > 0) {
      B.resize(size_t(n) * size_t(m));
      L.port_map(xi, B.data());
      matvec(n, m, B.data(), uh.data() + yoff_[i], t.data());
      for (int r = 0; r < n; ++r)
        di[r] += t[r];
    }
    if (k > 0) {
      G.resize(size_t(n) * size_t(k));
      L.input_map(xi, G.data());
      matvec(n, k, G.data(), u + uoff_[i], t.data());
      for (int r = 0; r < n; ++r)
        di[r] += t[r];
    }
  }
}

void composite::coupling_field(const double *x, double *dx) const {
  vecd e, y, uh, B;
  coupling_inputs(*this, x, e, y, uh);
  for (int i = 0; i < blocks(); ++i) {
    const block_law &L = *laws_[i];
    const int n = L.state_dim(), m = L.port_dim();
    double *di = dx + xoff_[i];
    if (m == 0) {
      std::fill(di, di + n, 0.0);
      continue;
    }
    B.resize(size_t(n) * size_t(m));
    L.port_map(x + xoff_[i], B.data());
    matvec(n, m, B.data(), uh.data() + yoff_[i], di);
  }
}

power_terms composite::powers(const double *x, const double *u) const {
  power_terms p;
  vecd e, y, uh, J, R, G, t;
  coupling_inputs(*this, x, e, y, uh);
  p.stored = energy(x);
  p.internal = dot(ny_, uh.data(), y.data());
  for (int i = 0; i < blocks(); ++i) {
    const block_law &L = *laws_[i];
    const int n = L.state_dim(), k = L.input_dim();
    const double *xi = x + xoff_[i], *ei = e.data() + xoff_[i];
    J.resize(size_t(n) * size_t(n));
    R.resize(J.size());
    L.structure(xi, J.data(), R.data());
    t.resize(n);
    matvec(n, n, R.data(), ei, t.data());
    p.loss += dot(n, ei, t.data());
    if (k > 0) {
      G.resize(size_t(n) * size_t(k));
      L.input_map(xi, G.data());
      matvec(n, k, G.data(), u + uoff_[i], t.data());
      p.external += dot(n, ei, t.data());
    }
  }
  return p;
}

// ── transitions ─────────────────────────────────────────────────────────────────────────────────

void coupling_step(const composite &s, coupling_scheme c, double tau, double *x) {
  const int N = s.state_dim();
  vecd f(N);
  s.coupling_field(x, f.data());
  if (c == coupling_scheme::euler) {
    for (int i = 0; i < N; ++i)
      x[i] += tau * f[i];
    return;
  }
  vecd xm(N);
  for (int i = 0; i < N; ++i)
    xm[i] = x[i] + 0.5 * tau * f[i];
  s.coupling_field(xm.data(), f.data());
  for (int i = 0; i < N; ++i)
    x[i] += tau * f[i];
}

void step_rk4(const composite &s, double dt, const double *u, double *x) {
  const int N = s.state_dim();
  vecd k1(N), k2(N), k3(N), k4(N), y(N);
  s.field(x, u, k1.data());
  for (int i = 0; i < N; ++i)
    y[i] = x[i] + 0.5 * dt * k1[i];
  s.field(y.data(), u, k2.data());
  for (int i = 0; i < N; ++i)
    y[i] = x[i] + 0.5 * dt * k2[i];
  s.field(y.data(), u, k3.data());
  for (int i = 0; i < N; ++i)
    y[i] = x[i] + dt * k3[i];
  s.field(y.data(), u, k4.data());
  for (int i = 0; i < N; ++i)
    x[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
}

step_stats step_implicit_midpoint(const composite &s, double dt, const double *u, double *x,
                                  const solve_opts &o) {
  const int N = s.state_dim();
  vecd f(N), x1(N), x2(N), mid(N);
  s.field(x, u, f.data());
  for (int i = 0; i < N; ++i)
    x1[i] = x[i] + dt * f[i];
  step_stats st;
  st.converged = false;
  for (int it = 1; it <= o.max_iter; ++it) {
    for (int i = 0; i < N; ++i)
      mid[i] = 0.5 * (x[i] + x1[i]);
    s.field(mid.data(), u, f.data());
    double d = 0;
    for (int i = 0; i < N; ++i) {
      x2[i] = x[i] + dt * f[i];
      d = std::max(d, std::fabs(x2[i] - x1[i]));
    }
    x1.swap(x2);
    st.iterations = it;
    if (d < o.tol) {
      st.converged = true;
      break;
    }
  }
  std::copy(x1.begin(), x1.end(), x);
  return st;
}

step_stats step(const composite &s, const transition_opts &o, const double *u, double *x) {
  if (!(o.dt > 0))
    bad("step needs dt > 0");
  if (o.scheme == step_scheme::rk4) {
    step_rk4(s, o.dt, u, x);
    return {};
  }
  if (o.scheme == step_scheme::implicit_midpoint)
    return step_implicit_midpoint(s, o.dt, u, x, o.solve);
  if (o.substeps < 1 || o.local_substeps < 1)
    bad("step needs substeps >= 1 and local_substeps >= 1");
  step_stats st;
  const double d = o.dt / o.substeps;
  for (int r = 0; r < o.substeps; ++r) {
    coupling_step(s, o.coupling, 0.5 * d, x);
    for (int i = 0; i < s.blocks(); ++i) {
      const int it = s.law(i).local_step(d, o.local_substeps, u + s.input_offset(i),
                                         x + s.state_offset(i), o.solve);
      if (it < 0)
        st.converged = false;
      st.iterations = std::max(st.iterations, std::abs(it));
    }
    coupling_step(s, o.coupling, 0.5 * d, x);
  }
  return st;
}

books step_books(const composite &s, double dt, const double *x0, const double *x1,
                 const double *u) {
  const power_terms a = s.powers(x0, u), b = s.powers(x1, u);
  books k{};
  k.dH = b.stored - a.stored;
  k.supplied = 0.5 * dt * (a.external + b.external);
  k.dissipated = 0.5 * dt * (a.loss + b.loss);
  k.floor = k.grade = k.limiter = 0;
  k.residual = k.dH - k.supplied + k.dissipated;
  return k;
}

books step_books_midpoint(const composite &s, double dt, const double *x0, const double *x1,
                          const double *u) {
  const int N = s.state_dim();
  vecd mid(N);
  for (int i = 0; i < N; ++i)
    mid[i] = 0.5 * (x0[i] + x1[i]);
  const power_terms m = s.powers(mid.data(), u);
  books k{};
  k.dH = s.energy(x1) - s.energy(x0);
  k.supplied = dt * m.external;
  k.dissipated = dt * m.loss;
  k.floor = k.grade = k.limiter = 0;
  k.residual = k.dH - k.supplied + k.dissipated;
  return k;
}

rollout_result rollout(const composite &s, const transition_opts &o, const vecd &x0, const vecd &u,
                       int steps) {
  const size_t N = s.state_dim(), K = s.input_dim();
  check_size(x0, N, "rollout start state");
  if (steps < 0)
    bad("rollout needs steps >= 0");
  const bool held = u.size() == K;
  if (!held && u.size() != K * size_t(steps))
    bad("rollout inputs: give one row of " + std::to_string(K) + " or " + std::to_string(steps) +
        " rows");
  rollout_result r;
  r.x.reserve(steps + 1);
  r.x.push_back(x0);
  vecd x = x0;
  const vecd none;
  for (int k = 0; k < steps; ++k) {
    const double *uk = K == 0 ? none.data() : (held ? u.data() : u.data() + K * size_t(k));
    r.stats.push_back(step(s, o, uk, x.data()));
    r.ledger.push_back(step_books(s, o.dt, r.x.back().data(), x.data(), uk));
    r.x.push_back(x);
  }
  return r;
}

balance_check check_balance(const composite &s, const double *x, const double *u) {
  const int N = s.state_dim(), ny = s.port_dim();
  vecd e(N), f(N), y(ny);
  s.effort(x, e.data());
  s.field(x, u, f.data());
  s.outputs(x, y.data());
  const power_terms p = s.powers(x, u);
  const double rate = dot(N, e.data(), f.data());
  balance_check c;
  const double scale = std::max({1.0, std::fabs(p.loss), std::fabs(p.external), std::fabs(rate)});
  c.rate_error = std::fabs(rate - (p.external - p.loss)) / scale;
  const vecd &C = s.pairing();
  double mag = 1.0;
  for (int a = 0; a < ny; ++a)
    for (int b = 0; b < ny; ++b) {
      mag += std::fabs(y[a] * C[size_t(a) * ny + b] * y[b]);
      c.skew_error =
          std::max(c.skew_error, std::fabs(C[size_t(a) * ny + b] + C[size_t(b) * ny + a]));
    }
  c.internal_error = std::fabs(p.internal) / mag;
  return c;
}

balance_summary check_balance_random(const composite &s, int samples, unsigned seed, double scale) {
  rng g(seed);
  balance_summary out;
  vecd x(s.state_dim()), u(s.input_dim());
  for (int k = 0; k < samples; ++k) {
    for (double &v : x)
      v = scale * g.normal();
    for (double &v : u)
      v = scale * g.normal();
    const balance_check c = check_balance(s, x.data(), u.data());
    out.worst.rate_error = std::max(out.worst.rate_error, c.rate_error);
    out.worst.internal_error = std::max(out.worst.internal_error, c.internal_error);
    out.worst.skew_error = std::max(out.worst.skew_error, c.skew_error);
    ++out.samples;
  }
  return out;
}

// ── reports ─────────────────────────────────────────────────────────────────────────────────────

namespace {

std::string block_label(const composite &s, int i) {
  return s.name(i).empty() ? "#" + std::to_string(i) : s.name(i);
}

} // namespace

std::string describe(const composite &s) {
  std::ostringstream o;
  o << s.blocks() << " blocks, state " << s.state_dim() << ", ports " << s.port_dim() << ", inputs "
    << s.input_dim() << "\n";
  for (int i = 0; i < s.blocks(); ++i) {
    const block_law &L = s.law(i);
    o << "  " << block_label(s, i) << ": " << L.kind() << ", state " << L.state_dim() << " at "
      << s.state_offset(i) << ", ports " << L.port_dim() << ", inputs " << L.input_dim() << "\n";
  }
  // The pairing, one line per connected pair of ports (upper triangle).
  auto port_name = [&](int a) {
    int i = s.blocks() - 1;
    while (i > 0 && s.port_offset(i) > a)
      --i;
    return block_label(s, i) + "." + std::to_string(a - s.port_offset(i));
  };
  const int ny = s.port_dim();
  int pairs = 0;
  for (int a = 0; a < ny; ++a)
    for (int b = a + 1; b < ny; ++b)
      if (const double c = s.pairing()[size_t(a) * ny + b]; c != 0) {
        o << "  pair " << port_name(a) << " - " << port_name(b) << ": " << c << "\n";
        ++pairs;
      }
  if (pairs == 0)
    o << "  no pairings: the blocks evolve independently\n";
  return o.str();
}

std::string describe(const composite &s, const transition_opts &opts, const rollout_result &r) {
  std::ostringstream o;
  o.precision(9);
  const int steps = static_cast<int>(r.ledger.size());
  double supplied = 0, dissipated = 0, worst = 0;
  int iterations = 0, unconverged = 0;
  for (const books &b : r.ledger) {
    supplied += b.supplied;
    dissipated += b.dissipated;
    worst = std::max(worst, std::fabs(b.residual));
  }
  for (const step_stats &st : r.stats) {
    iterations = std::max(iterations, st.iterations);
    unconverged += st.converged ? 0 : 1;
  }
  const double H0 = s.energy(r.x.front().data()), H1 = s.energy(r.x.back().data());
  o << steps << " steps of " << opts.dt << " s (" << to_string(opts.scheme);
  if (opts.scheme == step_scheme::split)
    o << ", " << opts.substeps << " substeps, " << to_string(opts.coupling) << " coupling";
  o << ")\n";
  o << "  H: " << H0 << " -> " << H1 << " (dH " << H1 - H0 << ")\n";
  o << "  supplied " << supplied << ", dissipated " << dissipated << ", residual "
    << (H1 - H0) - supplied + dissipated << " (worst step " << worst << ")\n";
  o << "  solves: at most " << iterations << " iterations";
  if (unconverged > 0)
    o << ", " << unconverged << " steps stopped at the cap";
  o << "\n  end state:\n";
  for (int i = 0; i < s.blocks(); ++i) {
    o << "    " << block_label(s, i) << ":";
    for (int k = 0; k < s.law(i).state_dim(); ++k)
      o << " " << r.x.back()[s.state_offset(i) + k];
    o << "\n";
  }
  return o.str();
}

void write_csv(std::ostream &out, const composite &s, const transition_opts &opts,
               const rollout_result &r) {
  out << "t";
  for (int i = 0; i < s.blocks(); ++i)
    for (int k = 0; k < s.law(i).state_dim(); ++k)
      out << "," << block_label(s, i) << "." << k;
  out << ",H,supplied,dissipated,residual\n";
  char buf[32];
  auto num = [&](double v) {
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
  };
  for (size_t k = 0; k < r.x.size(); ++k) {
    out << num(opts.dt * double(k));
    for (double v : r.x[k])
      out << "," << num(v);
    out << "," << num(s.energy(r.x[k].data()));
    if (k == 0) {
      out << ",,,\n";
      continue;
    }
    const books &b = r.ledger[k - 1];
    out << "," << num(b.supplied);
    out << "," << num(b.dissipated);
    out << "," << num(b.residual) << "\n";
  }
}

// ── names ───────────────────────────────────────────────────────────────────────────────────────

const char *to_string(coupling_scheme c) {
  return c == coupling_scheme::euler ? "euler" : "midpoint";
}

const char *to_string(step_scheme s) {
  switch (s) {
  case step_scheme::split:
    return "split";
  case step_scheme::rk4:
    return "rk4";
  case step_scheme::implicit_midpoint:
    return "implicit_midpoint";
  }
  return "split";
}

coupling_scheme coupling_scheme_of(const std::string &s) {
  if (s == "euler")
    return coupling_scheme::euler;
  if (s == "midpoint")
    return coupling_scheme::midpoint;
  bad("unknown coupling scheme '" + s + "' (euler, midpoint)");
}

step_scheme step_scheme_of(const std::string &s) {
  if (s == "split")
    return step_scheme::split;
  if (s == "rk4")
    return step_scheme::rk4;
  if (s == "implicit_midpoint" || s == "midpoint")
    return step_scheme::implicit_midpoint;
  bad("unknown step scheme '" + s + "' (split, rk4, implicit_midpoint)");
}

// ── files ───────────────────────────────────────────────────────────────────────────────────────

namespace {

namespace pt = boost::property_tree;

constexpr const char *kLawFormat = "cvc-ph-law/1";
constexpr const char *kSystemFormat = "cvc-ph-system/1";

[[noreturn]] void file_error(const std::string &what) {
  throw std::runtime_error("cvc::ph: " + what);
}

double number(const pt::ptree &t, const std::string &what) {
  try {
    return std::stod(t.data());
  } catch (const std::exception &) {
    file_error(what + ": '" + t.data() + "' is not a number");
  }
}

// A number, or an array (of arrays) of numbers flattened in row-major order.
void flatten(const pt::ptree &t, vecd &out, const std::string &what) {
  if (t.empty()) {
    out.push_back(number(t, what));
    return;
  }
  for (const auto &kv : t)
    flatten(kv.second, out, what);
}

vecd numbers(const pt::ptree &t, const std::string &key, const std::string &what) {
  vecd out;
  if (auto c = t.get_child_optional(key))
    flatten(*c, out, what + "." + key);
  return out;
}

int integer(const pt::ptree &t, const std::string &key, int fallback, const std::string &what) {
  auto v = t.get_optional<std::string>(key);
  if (!v)
    return fallback;
  try {
    size_t used = 0;
    const int i = std::stoi(*v, &used);
    if (used != v->size())
      throw std::invalid_argument(*v);
    return i;
  } catch (const std::exception &) {
    file_error(what + "." + key + ": '" + *v + "' is not an integer");
  }
}

activation activation_of(const std::string &s, const std::string &what) {
  if (s.empty() || s == "identity" || s == "linear")
    return activation::identity;
  if (s == "silu")
    return activation::silu;
  if (s == "tanh")
    return activation::tanh;
  if (s == "softplus")
    return activation::softplus;
  if (s == "sigmoid")
    return activation::sigmoid;
  file_error(what + ": unknown activation '" + s + "'");
}

mlp read_net(const pt::ptree &t, const std::string &what) {
  mlp net;
  int l = 0;
  for (const auto &kv : t) {
    const std::string w = what + "[" + std::to_string(l++) + "]";
    const pt::ptree &layer = kv.second;
    dense_layer L;
    auto W = layer.get_child_optional("W");
    if (!W)
      file_error(w + " has no W");
    L.out = static_cast<int>(W->size());
    for (const auto &row : *W) {
      vecd r;
      flatten(row.second, r, w + ".W");
      if (L.in == 0)
        L.in = static_cast<int>(r.size());
      else if (static_cast<int>(r.size()) != L.in)
        file_error(w + ".W has rows of different lengths");
      L.W.insert(L.W.end(), r.begin(), r.end());
    }
    L.b = numbers(layer, "b", w);
    L.act = activation_of(layer.get<std::string>("act", ""), w);
    net.layers.push_back(std::move(L));
  }
  return net;
}

conservative_scheme scheme_of(const std::string &s, const std::string &what) {
  if (s.empty() || s == "implicit_midpoint")
    return conservative_scheme::implicit_midpoint;
  if (s == "leapfrog")
    return conservative_scheme::leapfrog;
  file_error(what + ": unknown scheme '" + s + "' (implicit_midpoint, leapfrog)");
}

law_ptr read_law(const pt::ptree &t, const std::string &what) {
  const std::string format = t.get<std::string>("format", kLawFormat);
  if (format != kLawFormat)
    file_error(what + ": format '" + format + "', expected " + kLawFormat);
  const std::string kind = t.get<std::string>("kind", "");
  try {
    if (kind == "linear") {
      linear_spec s;
      s.n = integer(t, "n", 0, what);
      s.ports = integer(t, "ports", 0, what);
      s.inputs = integer(t, "inputs", 0, what);
      s.Q = numbers(t, "Q", what);
      s.J = numbers(t, "J", what);
      s.R = numbers(t, "R", what);
      s.B = numbers(t, "B", what);
      s.G = numbers(t, "G", what);
      return std::make_shared<linear_law>(std::move(s));
    }
    if (kind == "canonical") {
      canonical_spec s;
      s.n = integer(t, "n", 1, what);
      s.ports = integer(t, "ports", 0, what);
      s.inputs = integer(t, "inputs", 0, what);
      s.B = numbers(t, "B", what);
      s.G = numbers(t, "G", what);
      s.scheme = scheme_of(t.get<std::string>("scheme", ""), what);
      if (auto v = t.get_child_optional("potential")) {
        s.K = numbers(*v, "K", what + ".potential");
        s.center = numbers(*v, "center", what + ".potential");
        s.v_scale = v->get<double>("scale", 1.0);
        if (auto net = v->get_child_optional("net"))
          s.v_net = read_net(*net, what + ".potential.net");
      }
      if (auto m = t.get_child_optional("mass")) {
        s.mass = numbers(*m, "diag", what + ".mass");
        s.m_min = m->get<double>("min", 0.0);
        if (auto net = m->get_child_optional("net"))
          s.m_net = read_net(*net, what + ".mass.net");
      }
      if (auto d = t.get_child_optional("damping")) {
        s.damping = numbers(*d, "diag", what + ".damping");
        s.d_min = d->get<double>("min", 0.0);
        if (auto net = d->get_child_optional("net"))
          s.d_net = read_net(*net, what + ".damping.net");
      }
      return std::make_shared<canonical_law>(std::move(s));
    }
  } catch (const std::invalid_argument &e) {
    file_error(what + ": " + e.what());
  } catch (const pt::ptree_error &e) {
    file_error(what + ": " + e.what());
  }
  file_error(what + ": unknown kind '" + kind + "' (linear, canonical)");
}

pt::ptree read_tree_file(const std::string &path) {
  std::ifstream in(path);
  if (!in)
    file_error("cannot open " + path);
  pt::ptree root;
  try {
    pt::read_json(in, root);
  } catch (const pt::json_parser_error &e) {
    file_error(path + ": " + e.what());
  }
  return root;
}

pt::ptree read_tree_string(const std::string &json, const std::string &what) {
  std::istringstream in(json);
  pt::ptree root;
  try {
    pt::read_json(in, root);
  } catch (const pt::json_parser_error &e) {
    file_error(what + ": " + e.what());
  }
  return root;
}

int block_of(const composite &s, const std::string &name, const std::string &what) {
  const int i = s.find(name);
  if (i < 0)
    file_error(what + ": no block named '" + name + "'");
  return i;
}

std::vector<int> block_list(const composite &s, const pt::ptree &t, const std::string &key,
                            const std::string &what) {
  std::vector<int> out;
  auto c = t.get_child_optional(key);
  if (!c)
    file_error(what + " has no " + key);
  for (const auto &kv : *c)
    out.push_back(block_of(s, kv.second.data(), what + "." + key));
  return out;
}

composite read_system(const pt::ptree &root, const std::filesystem::path &dir,
                      const std::string &what) {
  const std::string format = root.get<std::string>("format", "");
  if (format != kSystemFormat)
    file_error(what + ": format '" + format + "', expected " + kSystemFormat);
  std::map<std::string, law_ptr> laws;
  if (auto L = root.get_child_optional("laws"))
    for (const auto &kv : *L) {
      const std::string w = what + ".laws." + kv.first;
      if (kv.second.empty()) { // a path, relative to the system file
        std::filesystem::path p(kv.second.data());
        if (p.is_relative())
          p = dir / p;
        laws[kv.first] = read_law(read_tree_file(p.string()), p.string());
      } else {
        laws[kv.first] = read_law(kv.second, w);
      }
    }
  composite s;
  auto B = root.get_child_optional("blocks");
  if (!B)
    file_error(what + " has no blocks");
  try {
    for (const auto &kv : *B) {
      const std::string name = kv.second.get<std::string>("name", "");
      const std::string law = kv.second.get<std::string>("law", "");
      auto it = laws.find(law);
      if (it == laws.end())
        file_error(what + ": block '" + name + "' names no law '" + law + "'");
      s.add_block(it->second, name);
    }
    if (auto C = root.get_child_optional("couplings"))
      for (const auto &kv : *C) {
        const pt::ptree &c = kv.second;
        const std::string type = c.get<std::string>("type", "");
        const std::string w = what + ".couplings (" + type + ")";
        if (type == "pair") {
          s.connect(block_of(s, c.get<std::string>("a"), w), integer(c, "a_port", 0, w),
                    block_of(s, c.get<std::string>("b"), w), integer(c, "b_port", 0, w),
                    c.get<double>("c", 1.0));
        } else if (type == "incidence") {
          s.connect_incidence(block_list(s, c, "buses", w), integer(c, "bus_port", 0, w),
                              block_list(s, c, "lines", w), integer(c, "line_port", 0, w),
                              numbers(c, "A", w));
        } else if (type == "skew") {
          s.connect_skew(block_list(s, c, "blocks", w), integer(c, "port", 0, w),
                         numbers(c, "C", w));
        } else {
          file_error(w + ": unknown coupling type (pair, incidence, skew)");
        }
      }
  } catch (const std::invalid_argument &e) {
    file_error(what + ": " + e.what());
  } catch (const pt::ptree_error &e) {
    file_error(what + ": " + e.what());
  }
  return s;
}

} // namespace

law_ptr load_law(const std::string &path) { return read_law(read_tree_file(path), path); }
law_ptr parse_law(const std::string &json) {
  return read_law(read_tree_string(json, "law"), "law");
}

composite load_system(const std::string &path) {
  return read_system(read_tree_file(path), std::filesystem::path(path).parent_path(), path);
}

composite parse_system(const std::string &json) {
  return read_system(read_tree_string(json, "system"), std::filesystem::current_path(), "system");
}

} // namespace ph
} // namespace cvc

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
  if (!s_.capacitor.empty()) {
    if (s_.capacitor.size() < 2 || !(s_.capacitor[1] > 0))
      bad("canonical law capacitor needs capacitor[1] > 0");
    for (size_t j = 0; j < s_.capacitor.size(); ++j) {
      if (j % 2 == 0 && s_.capacitor[j] != 0)
        bad("canonical law capacitor must be odd: entry " + std::to_string(j) + " is not 0");
      if (j % 2 == 1 && s_.capacitor[j] < 0)
        bad("canonical law capacitor entry " + std::to_string(j) + " is negative");
    }
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
  if (!s_.cosine.empty()) {
    check_size(s_.cosine, size_t(n), "canonical law cosine");
    for (double a : s_.cosine)
      if (!std::isfinite(a))
        bad("canonical law cosine coefficients must be finite");
  }
  port_dim_ = s_.ports;
  input_dim_ = s_.inputs;
  auto check_points = [&](const std::vector<serial_chain> &points, int &dim, const char *what) {
    for (const serial_chain &c : points) {
      if (c.joints() != n || c.axis.size() != c.offset.size())
        bad(std::string("canonical law ") + what + ": a chain needs n offsets and n axes");
      for (const auto &a : c.axis)
        if (std::fabs(std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) - 1) > 1e-9)
          bad(std::string("canonical law ") + what + ": a joint axis is not a unit vector");
      if (c.components.empty())
        bad(std::string("canonical law ") + what + ": a point needs at least one component");
      for (int k : c.components)
        if (k < 0 || k > 2)
          bad(std::string("canonical law ") + what + ": components are 0, 1 and 2 (x, y, z)");
      dim += static_cast<int>(c.components.size());
    }
  };
  check_points(s_.port_points, port_dim_, "port point");
  check_points(s_.input_points, input_dim_, "input point");
  if (s_.scheme == conservative_scheme::leapfrog && !s_.m_net.empty())
    bad("leapfrog needs a constant mass; use implicit_midpoint with a mass net");
}

double canonical_law::capacitor_voltage(double q) const {
  const vecd &c = s_.capacitor;
  if (c.empty())
    return 0.0;
  // Newton on q(v) - q, an odd increasing polynomial, from the linear part's voltage. For q > 0
  // the curve is convex and that start is above the root, so the iterates fall to it (and
  // symmetrically below 0).
  double v = q / c[1];
  for (int it = 0; it < 100; ++it) {
    double f = -q, df = 0, pw = 1; // pw = v^j
    for (size_t j = 0; j < c.size(); ++j) {
      f += c[j] * pw;
      if (j + 1 < c.size())
        df += double(j + 1) * c[j + 1] * pw;
      pw *= v;
    }
    const double step = f / df;
    v -= step;
    if (std::fabs(step) <= 1e-15 * (1 + std::fabs(v)))
      break;
  }
  return v;
}

double canonical_law::potential(const double *q, double *grad) const {
  const int n = s_.n;
  double V = 0;
  if (grad)
    std::fill(grad, grad + n, 0.0);
  if (!s_.capacitor.empty())
    for (int i = 0; i < n; ++i) {
      const double v = capacitor_voltage(q[i]);
      double p = v * v; // v^(j + 1) for j = 1
      for (size_t j = 1; j < s_.capacitor.size(); ++j) {
        V += s_.capacitor[j] * double(j) / double(j + 1) * p;
        p *= v;
      }
      if (grad)
        grad[i] += v;
    }
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
  for (size_t i = 0; i < s_.cosine.size(); ++i) {
    V += s_.cosine[i] * (1 - std::cos(q[i]));
    if (grad)
      grad[i] += s_.cosine[i] * std::sin(q[i]);
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

void canonical_law::point_columns(const std::vector<serial_chain> &points, const double *q,
                                  int cols, int first, double *M) const {
  const int n = s_.n;
  vecd r(3), J(size_t(3) * size_t(n));
  int col = first;
  for (const serial_chain &c : points) {
    c.eval(q, r.data(), J.data());
    for (int k : c.components) {
      for (int j = 0; j < n; ++j) {
        M[size_t(j) * cols + col] = 0.0;
        M[size_t(n + j) * cols + col] = J[size_t(k) * n + j];
      }
      ++col;
    }
  }
}

// B (G) with its constant columns first: the spec's 2n x ports block is spread over the wider map.
void canonical_law::port_map(const double *x, double *B) const {
  const int N = 2 * s_.n, m = port_dim_, k = s_.ports;
  for (int r = 0; r < N; ++r)
    for (int c = 0; c < k; ++c)
      B[size_t(r) * m + c] = s_.B[size_t(r) * k + c];
  if (!s_.port_points.empty())
    point_columns(s_.port_points, x, m, k, B);
}
void canonical_law::input_map(const double *x, double *G) const {
  const int N = 2 * s_.n, m = input_dim_, k = s_.inputs;
  for (int r = 0; r < N; ++r)
    for (int c = 0; c < k; ++c)
      G[size_t(r) * m + c] = s_.G[size_t(r) * k + c];
  if (!s_.input_points.empty())
    point_columns(s_.input_points, x, m, k, G);
}

// ── serial_chain ────────────────────────────────────────────────────────────────────────────────

namespace {

using vec3d = std::array<double, 3>;
using mat3d = std::array<double, 9>; // row-major

vec3d mul(const mat3d &R, const vec3d &v) {
  return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
          R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}

mat3d mul(const mat3d &A, const mat3d &B) {
  mat3d C{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
}

// Rodrigues: the rotation by angle t about the unit axis a.
mat3d rotation(const vec3d &a, double t) {
  const double c = std::cos(t), s = std::sin(t), v = 1 - c;
  return {c + a[0] * a[0] * v,        a[0] * a[1] * v - a[2] * s, a[0] * a[2] * v + a[1] * s,
          a[1] * a[0] * v + a[2] * s, c + a[1] * a[1] * v,        a[1] * a[2] * v - a[0] * s,
          a[2] * a[0] * v - a[1] * s, a[2] * a[1] * v + a[0] * s, c + a[2] * a[2] * v};
}

} // namespace

void serial_chain::eval(const double *q, double *r, double *jac) const {
  const int n = joints();
  mat3d R{1, 0, 0, 0, 1, 0, 0, 0, 1};
  vec3d o{0, 0, 0};
  std::vector<vec3d> origin(n), w(n); // each joint's origin and axis, base frame
  for (int i = 0; i < n; ++i) {
    const vec3d d = mul(R, offset[i]);
    for (int k = 0; k < 3; ++k)
      o[k] += d[k];
    origin[i] = o;
    w[i] = mul(R, axis[i]);
    R = mul(R, rotation(axis[i], q[i]));
  }
  const vec3d d = mul(R, point);
  for (int k = 0; k < 3; ++k)
    r[k] = o[k] + d[k];
  if (!jac)
    return;
  // d r / d q_i = w_i x (r - origin_i).
  for (int i = 0; i < n; ++i) {
    const vec3d l{r[0] - origin[i][0], r[1] - origin[i][1], r[2] - origin[i][2]};
    jac[0 * n + i] = w[i][1] * l[2] - w[i][2] * l[1];
    jac[1 * n + i] = w[i][2] * l[0] - w[i][0] * l[2];
    jac[2 * n + i] = w[i][0] * l[1] - w[i][1] * l[0];
  }
}

// ── contact_law ─────────────────────────────────────────────────────────────────────────────────

contact_law::contact_law(double stiffness) : k_(stiffness) {
  if (!(stiffness > 0) || !std::isfinite(stiffness))
    bad("a contact needs stiffness > 0");
}

double contact_law::energy(const double *x) const {
  const double s = std::max(x[0], 0.0);
  return 0.5 * k_ * s * s;
}
void contact_law::effort(const double *x, double *e) const { e[0] = k_ * std::max(x[0], 0.0); }
void contact_law::structure(const double *, double *J, double *R) const {
  J[0] = 0.0;
  R[0] = 0.0;
}
void contact_law::port_map(const double *, double *B) const { B[0] = 1.0; }

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

void composite::connect_damper(int i, int a, double c, int gate) {
  if (i < 0 || i >= blocks())
    bad("connect_damper: block index out of range");
  if (a < 0 || a >= laws_[i]->port_dim())
    bad("connect_damper: port index out of range");
  if (!(c >= 0) || !std::isfinite(c))
    bad("connect_damper needs a finite c >= 0");
  if (gate >= blocks() || (gate >= 0 && laws_[gate]->state_dim() < 1))
    bad("connect_damper: the gate names no block with a state");
  dampers_.push_back(port_damper{yoff_[i] + a, c, gate < 0 ? -1 : gate});
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

// A damper's coefficient at x.
double damper_c(const composite &s, const port_damper &d, const double *x) {
  return d.gate < 0 ? d.c : d.c * std::max(x[s.state_offset(d.gate)], 0.0);
}

// e, y_hat and u_hat = -C y_hat - (dampers) y_hat at x. damped (when non-null) gets the dampers'
// loss sum c y^2.
void coupling_inputs(const composite &s, const double *x, vecd &e, vecd &y, vecd &uh,
                     double *damped = nullptr) {
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
  double loss = 0;
  for (const port_damper &d : s.dampers()) {
    const double c = damper_c(s, d, x);
    uh[d.port] -= c * y[d.port];
    loss += c * y[d.port] * y[d.port];
  }
  if (damped)
    *damped = loss;
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
  double damped = 0;
  coupling_inputs(*this, x, e, y, uh, &damped);
  p.stored = energy(x);
  p.internal = dot(ny_, uh.data(), y.data()) + damped;
  p.loss = damped;
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

namespace {

// The coupling field, plus G(x) u when u is given (the port subflow).
void subflow(const composite &s, const double *x, const double *u, double *f) {
  s.coupling_field(x, f);
  if (!u)
    return;
  vecd G, t;
  for (int i = 0; i < s.blocks(); ++i) {
    const block_law &L = s.law(i);
    const int n = L.state_dim(), k = L.input_dim();
    if (k == 0)
      continue;
    G.resize(size_t(n) * size_t(k));
    t.resize(n);
    L.input_map(x + s.state_offset(i), G.data());
    matvec(n, k, G.data(), u + s.input_offset(i), t.data());
    for (int r = 0; r < n; ++r)
      f[s.state_offset(i) + r] += t[r];
  }
}

// The implicit midpoint of the subflow, x+ = x + tau f((x + x+)/2), by Newton from the
// explicit-Euler prediction with a central-difference Jacobian of f. A subflow linear in the
// state (momentum ports at frozen q, linear components) has an exact difference Jacobian up to
// rounding, so one update solves it and a second confirms it.
void coupling_cayley(const composite &s, double tau, double *x, const double *u) {
  const int N = s.state_dim();
  vecd f(N), x1(N), mid(N), fp(N), fm(N), A(size_t(N) * N), g(N);
  subflow(s, x, u, f.data());
  for (int i = 0; i < N; ++i)
    x1[i] = x[i] + tau * f[i];
  for (int it = 0; it < 20; ++it) {
    for (int i = 0; i < N; ++i)
      mid[i] = 0.5 * (x[i] + x1[i]);
    subflow(s, mid.data(), u, f.data());
    // g(x1) = x1 - x - tau f(mid); dg/dx1 = I - tau/2 df/dx(mid).
    double gmax = 0, xmax = 0;
    for (int i = 0; i < N; ++i) {
      g[i] = x1[i] - x[i] - tau * f[i];
      gmax = std::max(gmax, std::fabs(g[i]));
      xmax = std::max(xmax, std::fabs(x1[i]));
    }
    if (gmax <= 1e-15 * (1 + xmax))
      break;
    for (int j = 0; j < N; ++j) {
      const double h = 1e-6 * (1 + std::fabs(mid[j])), m0 = mid[j];
      mid[j] = m0 + h;
      subflow(s, mid.data(), u, fp.data());
      mid[j] = m0 - h;
      subflow(s, mid.data(), u, fm.data());
      mid[j] = m0;
      for (int i = 0; i < N; ++i)
        A[size_t(i) * N + j] = (i == j ? 1.0 : 0.0) - 0.25 * tau * (fp[i] - fm[i]) / h;
    }
    solve_in_place(N, A.data(), g.data());
    double dmax = 0;
    for (int i = 0; i < N; ++i) {
      x1[i] -= g[i];
      dmax = std::max(dmax, std::fabs(g[i]));
    }
    if (dmax <= 1e-15 * (1 + xmax))
      break;
  }
  std::copy(x1.begin(), x1.end(), x);
}

} // namespace

void coupling_step(const composite &s, coupling_scheme c, double tau, double *x, const double *u) {
  if (c == coupling_scheme::cayley) {
    coupling_cayley(s, tau, x, u);
    return;
  }
  const int N = s.state_dim();
  vecd f(N);
  subflow(s, x, u, f.data());
  if (c == coupling_scheme::euler) {
    for (int i = 0; i < N; ++i)
      x[i] += tau * f[i];
    return;
  }
  vecd xm(N);
  for (int i = 0; i < N; ++i)
    xm[i] = x[i] + 0.5 * tau * f[i];
  subflow(s, xm.data(), u, f.data());
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
  const bool outside = o.order == split_order::inputs_outside;
  const bool with = o.order == split_order::inputs_with_coupling;
  const vecd none(s.input_dim(), 0.0);
  auto kicks = [&]() {
    for (int i = 0; i < s.blocks(); ++i)
      if (s.law(i).input_dim() > 0)
        s.law(i).kick(0.5 * d, u + s.input_offset(i), x + s.state_offset(i));
  };
  const double *port_u = with ? u : nullptr;
  for (int r = 0; r < o.substeps; ++r) {
    if (outside)
      kicks();
    coupling_step(s, o.coupling, 0.5 * d, x, port_u);
    for (int i = 0; i < s.blocks(); ++i) {
      const double *ui = (outside || with ? none.data() : u) + s.input_offset(i);
      const int it = s.law(i).local_step(d, o.local_substeps, ui, x + s.state_offset(i), o.solve);
      if (it < 0)
        st.converged = false;
      st.iterations = std::max(st.iterations, std::abs(it));
    }
    coupling_step(s, o.coupling, 0.5 * d, x, port_u);
    if (outside)
      kicks();
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

// ── states ──────────────────────────────────────────────────────────────────────────────────────

storage_result storage_from_efforts(const composite &s, const vecd &e, const vecd &x0, int max_iter,
                                    double tol, double cap) {
  const size_t N = s.state_dim();
  check_size(e, N, "storage_from_efforts efforts");
  if (!x0.empty())
    check_size(x0, N, "storage_from_efforts start");
  storage_result r;
  r.x = x0.empty() ? vecd(N, 0.0) : x0;
  r.converged = true;
  for (int i = 0; i < s.blocks(); ++i) {
    const block_law &L = s.law(i);
    const int n = L.state_dim(), off = s.state_offset(i);
    double *x = r.x.data() + off;
    const double *want = e.data() + off;
    vecd f(n), fp(n), fm(n), J(size_t(n) * size_t(n)), dx(n);
    bool done = false;
    double res = 0;
    int it = 0;
    for (;; ++it) {
      L.effort(x, f.data());
      res = 0;
      for (int k = 0; k < n; ++k) {
        f[k] -= want[k];
        res = std::max(res, std::fabs(f[k]));
      }
      if (res <= tol) {
        done = true;
        break;
      }
      if (it == max_iter)
        break;
      for (int c = 0; c < n; ++c) {
        const double h = 1e-6 * (1 + std::fabs(x[c])), keep = x[c];
        x[c] = keep + h;
        L.effort(x, fp.data());
        x[c] = keep - h;
        L.effort(x, fm.data());
        x[c] = keep;
        for (int k = 0; k < n; ++k)
          J[k * n + c] = (fp[k] - fm[k]) / (2 * h);
      }
      for (int k = 0; k < n; ++k)
        dx[k] = -f[k];
      try {
        solve_in_place(n, J.data(), dx.data());
      } catch (const std::runtime_error &) {
        break; // a singular effort Jacobian: no storage state reaches these efforts from here
      }
      double big = 0;
      for (double v : dx)
        big = std::max(big, std::fabs(v));
      const double scale = big > cap ? cap / big : 1.0;
      for (int k = 0; k < n; ++k)
        x[k] += scale * dx[k];
    }
    r.converged = r.converged && done;
    r.iterations = std::max(r.iterations, it);
    r.residual = std::max(r.residual, res);
  }
  return r;
}

vecd assign_state(const composite &now, const vecd &x_now, const composite &candidate,
                  const std::map<std::string, vecd> &added) {
  check_size(x_now, size_t(now.state_dim()), "assign_state actual state");
  vecd x(candidate.state_dim(), 0.0);
  for (int i = 0; i < candidate.blocks(); ++i) {
    const std::string &name = candidate.name(i);
    const int n = candidate.law(i).state_dim(), off = candidate.state_offset(i);
    const int j = name.empty() ? -1 : now.find(name);
    if (j >= 0) {
      if (now.law(j).state_dim() != n)
        bad("assign_state: block '" + name + "' has state " + std::to_string(n) +
            " in the candidate, " + std::to_string(now.law(j).state_dim()) + " now");
      std::copy(x_now.begin() + now.state_offset(j), x_now.begin() + now.state_offset(j) + n,
                x.begin() + off);
      continue;
    }
    auto it = added.find(name);
    if (it == added.end())
      bad("assign_state: candidate block '" + (name.empty() ? "#" + std::to_string(i) : name) +
          "' is not in the actual system and has no supplied state");
    check_size(it->second, size_t(n), "assign_state supplied state of '" + name + "'");
    std::copy(it->second.begin(), it->second.end(), x.begin() + off);
  }
  return x;
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
  for (const port_damper &d : s.dampers()) {
    o << "  damper " << port_name(d.port) << ": " << d.c;
    if (d.gate >= 0)
      o << " x [" << block_label(s, d.gate) << ".0]+";
    o << "\n";
  }
  if (pairs == 0 && s.dampers().empty())
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
  switch (c) {
  case coupling_scheme::euler:
    return "euler";
  case coupling_scheme::midpoint:
    return "midpoint";
  case coupling_scheme::cayley:
    return "cayley";
  }
  return "midpoint";
}

const char *to_string(split_order o) {
  switch (o) {
  case split_order::coupling_outside:
    return "coupling_outside";
  case split_order::inputs_outside:
    return "inputs_outside";
  case split_order::inputs_with_coupling:
    return "inputs_with_coupling";
  }
  return "coupling_outside";
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
  if (s == "cayley")
    return coupling_scheme::cayley;
  bad("unknown coupling scheme '" + s + "' (euler, midpoint, cayley)");
}

split_order split_order_of(const std::string &s) {
  if (s == "coupling_outside")
    return split_order::coupling_outside;
  if (s == "inputs_outside")
    return split_order::inputs_outside;
  if (s == "inputs_with_coupling")
    return split_order::inputs_with_coupling;
  bad("unknown split order '" + s + "' (coupling_outside, inputs_outside, inputs_with_coupling)");
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
    if (!t.data().empty()) // an empty array reads as a leaf with no data: no numbers
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

std::array<double, 3> triple(const pt::ptree &t, const std::string &what) {
  vecd v;
  flatten(t, v, what);
  if (v.size() != 3)
    file_error(what + " needs 3 numbers");
  return {v[0], v[1], v[2]};
}

// "key": [{"offset": [[x, y, z], ...], "axis": [[x, y, z], ...], "point": [x, y, z],
//          "components": [0, 1, 2]}, ...]
std::vector<serial_chain> read_points(const pt::ptree &t, const std::string &key,
                                      const std::string &what) {
  std::vector<serial_chain> out;
  auto P = t.get_child_optional(key);
  if (!P)
    return out;
  int k = 0;
  for (const auto &kv : *P) {
    const std::string w = what + "." + key + "[" + std::to_string(k++) + "]";
    const pt::ptree &c = kv.second;
    serial_chain ch;
    for (const char *f : {"offset", "axis"}) {
      auto rows = c.get_child_optional(f);
      if (!rows)
        file_error(w + " has no " + f);
      for (const auto &r : *rows)
        (std::string(f) == "offset" ? ch.offset : ch.axis).push_back(triple(r.second, w + "." + f));
    }
    if (auto p = c.get_child_optional("point"))
      ch.point = triple(*p, w + ".point");
    if (c.get_child_optional("components")) {
      ch.components.clear();
      for (double v : numbers(c, "components", w))
        ch.components.push_back(static_cast<int>(v));
    }
    out.push_back(std::move(ch));
  }
  return out;
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
        s.cosine = numbers(*v, "cosine", what + ".potential");
        s.capacitor = numbers(*v, "capacitor", what + ".potential");
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
      s.port_points = read_points(t, "port_points", what);
      s.input_points = read_points(t, "input_points", what);
      return std::make_shared<canonical_law>(std::move(s));
    }
    if (kind == "contact")
      return std::make_shared<contact_law>(t.get<double>("stiffness"));
  } catch (const std::invalid_argument &e) {
    file_error(what + ": " + e.what());
  } catch (const pt::ptree_error &e) {
    file_error(what + ": " + e.what());
  }
  file_error(what + ": unknown kind '" + kind + "' (linear, canonical, contact)");
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
        } else if (type == "damper") {
          const std::string gate = c.get<std::string>("gate", "");
          s.connect_damper(block_of(s, c.get<std::string>("block"), w), integer(c, "port", 0, w),
                           c.get<double>("c"), gate.empty() ? -1 : block_of(s, gate, w));
        } else {
          file_error(w + ": unknown coupling type (pair, incidence, skew, damper)");
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

namespace {

std::string num(double v) {
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

std::string quoted(const std::string &s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\')
      o += '\\';
    o += c;
  }
  return o + "\"";
}

void put_numbers(std::ostringstream &o, const char *key, const vecd &v) {
  o << ", " << quoted(key) << ": [";
  for (size_t i = 0; i < v.size(); ++i)
    o << (i ? ", " : "") << num(v[i]);
  o << "]";
}

void put_triple(std::ostringstream &o, const std::array<double, 3> &v) {
  o << "[" << num(v[0]) << ", " << num(v[1]) << ", " << num(v[2]) << "]";
}

void put_points(std::ostringstream &o, const char *key, const std::vector<serial_chain> &points) {
  if (points.empty())
    return;
  o << ", " << quoted(key) << ": [";
  for (size_t p = 0; p < points.size(); ++p) {
    const serial_chain &c = points[p];
    o << (p ? ", " : "") << "{\"offset\": [";
    for (size_t i = 0; i < c.offset.size(); ++i) {
      o << (i ? ", " : "");
      put_triple(o, c.offset[i]);
    }
    o << "], \"axis\": [";
    for (size_t i = 0; i < c.axis.size(); ++i) {
      o << (i ? ", " : "");
      put_triple(o, c.axis[i]);
    }
    o << "], \"point\": ";
    put_triple(o, c.point);
    o << ", \"components\": [";
    for (size_t i = 0; i < c.components.size(); ++i)
      o << (i ? ", " : "") << c.components[i];
    o << "]}";
  }
  o << "]";
}

const char *activation_name(activation a) {
  switch (a) {
  case activation::identity:
    return "identity";
  case activation::silu:
    return "silu";
  case activation::tanh:
    return "tanh";
  case activation::softplus:
    return "softplus";
  case activation::sigmoid:
    return "sigmoid";
  }
  return "identity";
}

void put_net(std::ostringstream &o, const mlp &net) {
  o << ", \"net\": [";
  for (size_t l = 0; l < net.layers.size(); ++l) {
    const dense_layer &L = net.layers[l];
    o << (l ? ", " : "") << "{\"W\": [";
    for (int r = 0; r < L.out; ++r) {
      o << (r ? ", " : "") << "[";
      for (int c = 0; c < L.in; ++c)
        o << (c ? ", " : "") << num(L.W[size_t(r) * L.in + c]);
      o << "]";
    }
    o << "]";
    put_numbers(o, "b", L.b);
    o << ", \"act\": " << quoted(activation_name(L.act)) << "}";
  }
  o << "]";
}

} // namespace

std::string law_json(const block_law &law) {
  std::ostringstream o;
  if (const auto *l = dynamic_cast<const linear_law *>(&law)) {
    const linear_spec &s = l->spec();
    o << "{\"format\": \"cvc-ph-law/1\", \"kind\": \"linear\", \"n\": " << s.n
      << ", \"ports\": " << s.ports << ", \"inputs\": " << s.inputs;
    put_numbers(o, "Q", s.Q);
    put_numbers(o, "J", s.J);
    put_numbers(o, "R", s.R);
    if (s.ports > 0)
      put_numbers(o, "B", s.B);
    if (s.inputs > 0)
      put_numbers(o, "G", s.G);
    o << "}";
    return o.str();
  }
  if (const auto *c = dynamic_cast<const canonical_law *>(&law)) {
    const canonical_spec &s = c->spec();
    o << "{\"format\": \"cvc-ph-law/1\", \"kind\": \"canonical\", \"n\": " << s.n
      << ", \"ports\": " << s.ports << ", \"inputs\": " << s.inputs << ", \"scheme\": \""
      << (s.scheme == conservative_scheme::leapfrog ? "leapfrog" : "implicit_midpoint") << "\"";
    if (s.ports > 0)
      put_numbers(o, "B", s.B);
    if (s.inputs > 0)
      put_numbers(o, "G", s.G);
    o << ", \"potential\": {\"scale\": " << num(s.v_scale);
    if (!s.K.empty()) {
      put_numbers(o, "K", s.K);
      put_numbers(o, "center", s.center);
    }
    if (!s.cosine.empty())
      put_numbers(o, "cosine", s.cosine);
    if (!s.capacitor.empty())
      put_numbers(o, "capacitor", s.capacitor);
    if (!s.v_net.empty())
      put_net(o, s.v_net);
    o << "}, \"mass\": {";
    if (s.m_net.empty()) {
      o << "\"min\": 0";
      put_numbers(o, "diag", s.mass);
    } else {
      o << "\"min\": " << num(s.m_min);
      put_net(o, s.m_net);
    }
    o << "}";
    if (!s.d_net.empty()) {
      o << ", \"damping\": {\"min\": " << num(s.d_min);
      put_net(o, s.d_net);
      o << "}";
    } else if (!s.damping.empty()) {
      o << ", \"damping\": {\"min\": 0";
      put_numbers(o, "diag", s.damping);
      o << "}";
    }
    put_points(o, "port_points", s.port_points);
    put_points(o, "input_points", s.input_points);
    o << "}";
    return o.str();
  }
  if (const auto *c = dynamic_cast<const contact_law *>(&law)) {
    o << "{\"format\": \"cvc-ph-law/1\", \"kind\": \"contact\", \"stiffness\": "
      << num(c->stiffness()) << "}";
    return o.str();
  }
  bad("law_json: a " + law.kind() + " law has no file form");
}

std::string system_json(const composite &s) {
  std::map<const block_law *, std::string> names;
  std::ostringstream o;
  o << "{\"format\": \"cvc-ph-system/1\", \"laws\": {";
  for (int i = 0; i < s.blocks(); ++i) {
    const block_law *l = &s.law(i);
    if (names.count(l))
      continue;
    const std::string name = "law" + std::to_string(names.size());
    o << (names.empty() ? "" : ", ") << quoted(name) << ": " << law_json(*l);
    names[l] = name;
  }
  auto block_name = [&](int i) { return s.name(i).empty() ? "b" + std::to_string(i) : s.name(i); };
  o << "}, \"blocks\": [";
  for (int i = 0; i < s.blocks(); ++i)
    o << (i ? ", " : "") << "{\"name\": " << quoted(block_name(i))
      << ", \"law\": " << quoted(names[&s.law(i)]) << "}";
  o << "], \"couplings\": [";
  // Each nonzero entry of the pairing's upper triangle, as the pair that made it.
  auto port_of = [&](int a) {
    int i = 0;
    while (i + 1 < s.blocks() && s.port_offset(i + 1) <= a)
      ++i;
    while (s.law(i).port_dim() == 0 || a >= s.port_offset(i) + s.law(i).port_dim())
      ++i;
    return std::make_pair(i, a - s.port_offset(i));
  };
  const int ny = s.port_dim();
  bool first = true;
  for (int a = 0; a < ny; ++a)
    for (int b = a + 1; b < ny; ++b) {
      const double c = s.pairing()[size_t(a) * ny + b];
      if (c == 0)
        continue;
      const auto pa = port_of(a), pb = port_of(b);
      o << (first ? "" : ", ") << "{\"type\": \"pair\", \"a\": " << quoted(block_name(pa.first))
        << ", \"a_port\": " << pa.second << ", \"b\": " << quoted(block_name(pb.first))
        << ", \"b_port\": " << pb.second << ", \"c\": " << num(c) << "}";
      first = false;
    }
  for (const port_damper &d : s.dampers()) {
    const auto pa = port_of(d.port);
    o << (first ? "" : ", ") << "{\"type\": \"damper\", \"block\": " << quoted(block_name(pa.first))
      << ", \"port\": " << pa.second << ", \"c\": " << num(d.c);
    if (d.gate >= 0)
      o << ", \"gate\": " << quoted(block_name(d.gate));
    o << "}";
    first = false;
  }
  o << "]}";
  return o.str();
}

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

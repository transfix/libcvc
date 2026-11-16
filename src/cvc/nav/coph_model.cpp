/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH's plant, potential and command (coph_model.h). Built with -ffp-contract=off (/fp:precise):
// the plant's closed forms reproduce the Python only uncontracted, in the Python's order.

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/nav/coph_model.h>
#include <limits>

namespace cvc {
namespace nav {

namespace {

// ---------------------------------------------------------------------------
// Double-double arithmetic (Dekker, Knuth), for the correctly rounded exp and expm1. Each value is
// hi + lo with hi = fl(hi + lo), so hi is the double nearest the pair.
// ---------------------------------------------------------------------------

struct dd {
  double hi, lo;
};

dd two_sum(double a, double b) {
  const double s = a + b;
  const double bb = s - a;
  return {s, (a - (s - bb)) + (b - bb)};
}

// Requires |a| >= |b| (or a = 0).
dd fast_two_sum(double a, double b) {
  const double s = a + b;
  return {s, b - (s - a)};
}

// Dekker's split: a = hi + lo, each with at most 26 significant bits.
void split(double a, double &hi, double &lo) {
  const double c = 134217729.0 * a; // 2^27 + 1
  hi = c - (c - a);
  lo = a - hi;
}

dd two_prod(double a, double b) {
  const double p = a * b;
  double ah, al, bh, bl;
  split(a, ah, al);
  split(b, bh, bl);
  return {p, ((ah * bh - p) + ah * bl + al * bh) + al * bl};
}

dd dd_add(dd a, dd b) {
  dd s = two_sum(a.hi, b.hi);
  const dd t = two_sum(a.lo, b.lo);
  s.lo += t.hi;
  s = fast_two_sum(s.hi, s.lo);
  s.lo += t.lo;
  return fast_two_sum(s.hi, s.lo);
}

dd dd_mul(dd a, dd b) {
  dd p = two_prod(a.hi, b.hi);
  p.lo += a.hi * b.lo + a.lo * b.hi;
  return fast_two_sum(p.hi, p.lo);
}

dd dd_div(dd a, double b) {
  const double q1 = a.hi / b;
  const dd p = two_prod(q1, b);
  dd s = two_sum(a.hi, -p.hi);
  s.lo -= p.lo;
  s.lo += a.lo;
  return fast_two_sum(q1, (s.hi + s.lo) / b);
}

// ln 2 = kLn2Hi + kLn2Mid + kLn2Lo to about 2^-140. kLn2Hi has 33 significant bits, so k kLn2Hi is
// exact for |k| < 2^20.
constexpr double kLn2Hi = 0x1.62e42fee00000p-1;
constexpr double kLn2Mid = 0x1.a39ef35793c76p-33;
constexpr double kLn2Lo = 0x1.cc01f97b57a08p-87;
constexpr double kInvLn2 = 0x1.71547652b82fep0;

// e^r - 1 for |r| <= ln 2 / 2 (r in double-double), r (1 + r/2 (1 + r/3 (... (1 + r/23)))). The
// first term left out, r^24 / 24!, is below 2^-110 of the result.
dd expm1_kernel(dd r) {
  dd t{1.0, 0.0};
  for (int i = 23; i >= 2; --i)
    t = dd_add({1.0, 0.0}, dd_div(dd_mul(r, t), static_cast<double>(i)));
  return dd_mul(r, t);
}

// x = k ln 2 + r with |r| <= ln 2 / 2 (just over, where x / ln 2 rounds to the nearer k).
struct reduced {
  double k;
  dd r;
};

reduced reduce(double x) {
  const double k = std::nearbyint(x * kInvLn2);
  const double r0 = x - k * kLn2Hi; // exact: k kLn2Hi is exact and within a factor 2 of x
  dd r = dd_add({r0, 0.0}, two_prod(-k, kLn2Mid));
  r = dd_add(r, {-k * kLn2Lo, 0.0});
  return {k, r};
}

constexpr double kExpMax = 709.782712893384; // the largest x with e^x finite

// log1p from det::log (Goldberg): with u = fl(1 + y), log(u) y / (u - 1) is within a few ULP.
double log1p_det(double y) {
  const double u = 1.0 + y;
  if (u == 1.0)
    return y;
  return algo::det::log(u) * (y / (u - 1.0));
}

// ---------------------------------------------------------------------------
// The scalar's math. The templates call these; C3's duals add their own.
// ---------------------------------------------------------------------------

template <class T> struct scalar;

template <> struct scalar<double> {
  static double value(double x) { return x; }
  static double exp(double x) { return algo::det::exp(x); }
  static double log1p(double x) { return log1p_det(x); }
  static double sqrt(double x) { return std::sqrt(x); }
  // The plant's: correctly rounded, as torch's are.
  static double plant_exp(double x) { return coph_exp(x); }
  static double plant_expm1(double x) { return coph_expm1(x); }
};

// torch.nn.functional.softplus (beta 1, threshold 20): z above 20, log1p(exp(z)) otherwise.
template <class T> T softplus(const T &z) {
  using S = scalar<T>;
  return S::value(z) > 20.0 ? z : S::log1p(S::exp(z));
}

// softplus'(z), as torch's backward computes it: e / (e + 1) with e = exp(z), 1 above 20.
template <class T> T softplus_slope(const T &z) {
  using S = scalar<T>;
  if (S::value(z) > 20.0)
    return T(1.0);
  const T e = S::exp(z);
  return e / (e + 1.0);
}

constexpr double kAgentRadius = 0.12; // clearance = surface distance - 0.12
constexpr double kBarrierReach = 0.35;
constexpr double kVelocityGain = 1.4; // command's -1.4 v
constexpr double kNormFloor = 1e-12;

// An obstacle's offset from its nearest surface point's core: q - (x, y) for a disk, q minus the
// nearest point of the segment for a slab (shapes.slab_offset). The surface distance is its length
// minus r, and its gradient is offset / length.
template <class T> coph_vec2_of<T> core_offset(const coph_vec2_of<T> &q, const coph_shape &o) {
  using S = scalar<T>;
  if (!o.slab)
    return {q[0] - o.x, q[1] - o.y};
  const double p0 = o.x - o.hx, p1 = o.y - o.hy; // the ends p = c - h, e = c + h
  const double e0 = o.x + o.hx, e1 = o.y + o.hy;
  const double d0 = e0 - p0, d1 = e1 - p1;
  const T raw = ((q[0] - p0) * d0 + (q[1] - p1) * d1) / (d0 * d0 + d1 * d1);
  const T t = S::value(raw) < 0.0 ? T(0.0) : (S::value(raw) > 1.0 ? T(1.0) : raw);
  return {q[0] - (p0 + t * d0), q[1] - (p1 + t * d1)};
}

template <class T> T length(const coph_vec2_of<T> &z) {
  return scalar<T>::sqrt(z[0] * z[0] + z[1] * z[1]);
}

// The barrier's argument for one obstacle: 6 (0.35 - clearance), with clearance = |offset| - r -
// 0.12.
template <class T> T barrier_argument(const T &distance, const coph_shape &o) {
  const T clearance = (distance - o.r) - kAgentRadius;
  return (kBarrierReach - clearance) * 6.0;
}

// The RBF field's raw sum, sum_k w_k phi_k, and (when grad is given) its gradient in q.
template <class T>
T field_raw(const coph_vec2_of<T> &q, const coph_field_view &f, coph_vec2_of<T> *grad) {
  using S = scalar<T>;
  T raw(0.0);
  if (grad)
    *grad = {T(0.0), T(0.0)};
  for (std::size_t k = 0; k < f.centers.size(); ++k) {
    const T dx = (q[0] - f.centers[k][0]) / f.width;
    const T dy = (q[1] - f.centers[k][1]) / f.width;
    const T phi = S::exp(-0.5 * (dx * dx + dy * dy));
    raw = raw + phi * f.weights[k];
    if (grad) {
      const T scale = phi * f.weights[k] / f.width; // d phi / dq = -phi (dx, dy) / width
      (*grad)[0] = (*grad)[0] - scale * dx;
      (*grad)[1] = (*grad)[1] - scale * dy;
    }
  }
  return raw;
}

// The shaping, policy[0] risk + policy[1] barrier, and (when grad is given) its gradient in q.
template <class T>
T shaping_of(const coph_vec2_of<T> &q, const coph_vec2_of<T> &policy, const coph_field_view &f,
             coph_vec2_of<T> *grad) {
  coph_vec2_of<T> raw_grad{};
  const T raw = field_raw(q, f, grad ? &raw_grad : nullptr);
  T shaping = policy[0] * (softplus(raw) / 4.0);
  if (grad) {
    const T slope = policy[0] * softplus_slope(raw) / 4.0;
    *grad = {slope * raw_grad[0], slope * raw_grad[1]};
  }
  if (f.obstacles.empty())
    return shaping;
  T barrier(0.0);
  coph_vec2_of<T> barrier_grad{T(0.0), T(0.0)};
  for (const coph_shape &o : f.obstacles) {
    const coph_vec2_of<T> z = core_offset(q, o);
    const T distance = length(z);
    const T arg = barrier_argument(distance, o);
    const T sp = softplus(arg);
    barrier = barrier + sp * sp;
    if (grad) {
      // d(sp^2)/dq = 2 sp sp'(arg) (-6) z / |z|
      const T scale = (sp * softplus_slope(arg)) * -12.0 / distance;
      barrier_grad[0] = barrier_grad[0] + scale * z[0];
      barrier_grad[1] = barrier_grad[1] + scale * z[1];
    }
  }
  shaping = shaping + policy[1] * barrier / 36.0;
  if (grad) {
    (*grad)[0] = (*grad)[0] + policy[1] * (barrier_grad[0] / 36.0);
    (*grad)[1] = (*grad)[1] + policy[1] * (barrier_grad[1] / 36.0);
  }
  return shaping;
}

} // namespace

// ---------------------------------------------------------------------------
// exp and expm1, correctly rounded
// ---------------------------------------------------------------------------

double coph_exp(double x) {
  if (std::isnan(x))
    return x;
  if (x > kExpMax)
    return std::numeric_limits<double>::infinity();
  if (x < -745.2) // below 2^-1075: rounds to 0
    return 0.0;
  if (x == 0.0)
    return 1.0;
  const reduced rd = reduce(x);
  const dd y = dd_add({1.0, 0.0}, expm1_kernel(rd.r));
  // Exact scaling of the rounded e^r, so correctly rounded unless the result is subnormal (there
  // it is rounded twice, within an ULP).
  return std::ldexp(y.hi, static_cast<int>(rd.k));
}

double coph_expm1(double x) {
  if (std::isnan(x) || x == 0.0) // keeps -0
    return x;
  if (x > kExpMax)
    return std::numeric_limits<double>::infinity();
  if (x < -38.0) // e^x < 2^-54, so -1 + e^x rounds to -1
    return -1.0;
  const reduced rd = reduce(x);
  const dd em = expm1_kernel(rd.r);
  if (rd.k == 0.0)
    return em.hi;
  const int k = static_cast<int>(rd.k);
  const dd y = dd_add({1.0, 0.0}, em); // e^r
  if (k > 1000)                        // the -1 lies far below the last bit
    return std::ldexp(y.hi, k);
  const dd scaled{std::ldexp(y.hi, k), std::ldexp(y.lo, k)}; // exact
  return dd_add(scaled, {-1.0, 0.0}).hi;
}

// ---------------------------------------------------------------------------
// The plant
// ---------------------------------------------------------------------------

template <class T>
coph_state_of<T> coph_flow(const coph_state_of<T> &x, const coph_vec2_of<T> &u, const T &damping,
                           double dt) {
  using S = scalar<T>;
  const T &d = damping;
  const T a = S::plant_exp(-d * dt);
  const T integral = -S::plant_expm1(-d * dt) / d;
  const T rest = dt - integral;
  return {(x[0] + integral * x[2]) + (rest * u[0]) / d,
          (x[1] + integral * x[3]) + (rest * u[1]) / d, a * x[2] + integral * u[0],
          a * x[3] + integral * u[1]};
}

coph_balance_terms coph_balance(const coph_state &x, const coph_vec2 &u, double damping,
                                double dt) {
  const double d = damping;
  const double steady0 = u[0] / d, steady1 = u[1] / d;
  const double transient0 = x[2] - steady0, transient1 = x[3] - steady1;
  const double first = -coph_expm1(-d * dt) / d;
  const double second = -coph_expm1(-2 * d * dt) / (2 * d);
  const coph_state after = coph_flow<double>(x, u, d, dt);
  const double ss = steady0 * steady0 + steady1 * steady1;
  const double st = steady0 * transient0 + steady1 * transient1;
  const double tt = transient0 * transient0 + transient1 * transient1;
  coph_balance_terms b;
  b.heat = d * ((ss * dt + 2 * st * first) + tt * second);
  b.work = u[0] * (after[0] - x[0]) + u[1] * (after[1] - x[1]);
  b.kinetic_change =
      ((after[2] * after[2] + after[3] * after[3]) - (x[2] * x[2] + x[3] * x[3])) / 2;
  b.residual = (b.kinetic_change - b.work) + b.heat;
  return b;
}

// ---------------------------------------------------------------------------
// The potential and the command
// ---------------------------------------------------------------------------

coph_field_view coph_field_of(const coph_stage &stage, const coph_config &cfg,
                              std::span<const coph_shape> obstacles) {
  coph_field_view f;
  f.centers = stage.centers;
  f.weights = stage.weights;
  f.width = cfg.rbf_width;
  f.obstacles = obstacles;
  return f;
}

coph_shaping coph_shaping_parameters(const coph_field_view &field, const coph_config &cfg) {
  double weight_sum = 0.0;
  for (double w : field.weights)
    weight_sum += std::fabs(w);
  const double risk_bound = coph_exp(-0.5) * weight_sum / (4 * field.width);
  double barrier_bound = 0.0;
  for (const coph_shape &o : field.obstacles)
    barrier_bound += softplus(6 * (o.r + 0.47)) / 3;
  coph_shaping s;
  s.k = cfg.shaping_k;
  s.ell = cfg.shaping_ell;
  s.eta = cfg.shaping_eta;
  s.bound = cfg.policy_bounds.second * (risk_bound + barrier_bound);
  s.alpha = s.bound == 0.0 ? 1.0 : std::min(1.0, 2 * s.eta * s.k * s.ell / s.bound);
  return s;
}

std::optional<coph_shaping> coph_shaping_for(const coph_field_view &field, const coph_config &cfg) {
  if (cfg.shaping_mode == "progress")
    return coph_shaping_parameters(field, cfg);
  return std::nullopt;
}

template <class T> T coph_risk(const coph_vec2_of<T> &q, const coph_field_view &field) {
  return softplus(field_raw<T>(q, field, nullptr)) / 4.0;
}

template <class T>
T coph_potential(const coph_vec2_of<T> &q, const coph_vec2 &target, const coph_vec2_of<T> &policy,
                 const coph_field_view &field, const std::optional<coph_shaping> &progress) {
  const T g0 = q[0] - target[0], g1 = q[1] - target[1];
  const T s = g0 * g0 + g1 * g1;
  const T shaping = shaping_of<T>(q, policy, field, nullptr);
  if (!progress)
    return 0.5 * s + shaping;
  const coph_shaping &p = *progress;
  return 0.5 * p.k * s + p.alpha * s / (s + p.ell * p.ell) * shaping;
}

template <class T>
coph_vec2_of<T> coph_grad_potential(const coph_vec2_of<T> &q, const coph_vec2 &target,
                                    const coph_vec2_of<T> &policy, const coph_field_view &field,
                                    const std::optional<coph_shaping> &progress) {
  const T g0 = q[0] - target[0], g1 = q[1] - target[1];
  coph_vec2_of<T> shaping_grad{};
  const T shaping = shaping_of<T>(q, policy, field, &shaping_grad);
  if (!progress) // V = 0.5 s + shaping
    return {g0 + shaping_grad[0], g1 + shaping_grad[1]};
  // V = 0.5 k s + m(s) shaping, m = alpha s / (s + ell^2), dm/dq = 2 alpha ell^2 (q - t) / (s +
  // ell^2)^2
  const coph_shaping &p = *progress;
  const double ell2 = p.ell * p.ell;
  const T s = g0 * g0 + g1 * g1;
  const T den = s + ell2;
  const T m = p.alpha * s / den;
  const T dm = 2.0 * p.alpha * ell2 / (den * den) * shaping;
  return {p.k * g0 + dm * g0 + m * shaping_grad[0], p.k * g1 + dm * g1 + m * shaping_grad[1]};
}

template <class T>
coph_command_of<T> coph_command(const coph_state_of<T> &x, const coph_vec2 &target,
                                const coph_vec2_of<T> &policy, const coph_field_view &field,
                                const std::optional<coph_shaping> &progress, double limit) {
  using S = scalar<T>;
  const coph_vec2_of<T> q{x[0], x[1]};
  const coph_vec2_of<T> grad = coph_grad_potential<T>(q, target, policy, field, progress);
  coph_command_of<T> c;
  c.proposed = {-grad[0] - kVelocityGain * x[2], -grad[1] - kVelocityGain * x[3]};
  const T norm = length(c.proposed);
  const T floor = S::value(norm) < kNormFloor ? T(kNormFloor) : norm;
  T factor = limit / floor;
  if (S::value(factor) > 1.0)
    factor = T(1.0);
  c.applied = {c.proposed[0] * factor, c.proposed[1] * factor};
  return c;
}

template coph_state_of<double> coph_flow<double>(const coph_state_of<double> &,
                                                 const coph_vec2_of<double> &, const double &,
                                                 double);
template double coph_risk<double>(const coph_vec2_of<double> &, const coph_field_view &);
template double coph_potential<double>(const coph_vec2_of<double> &, const coph_vec2 &,
                                       const coph_vec2_of<double> &, const coph_field_view &,
                                       const std::optional<coph_shaping> &);
template coph_vec2_of<double> coph_grad_potential<double>(const coph_vec2_of<double> &,
                                                          const coph_vec2 &,
                                                          const coph_vec2_of<double> &,
                                                          const coph_field_view &,
                                                          const std::optional<coph_shaping> &);
template coph_command_of<double>
coph_command<double>(const coph_state_of<double> &, const coph_vec2 &, const coph_vec2_of<double> &,
                     const coph_field_view &, const std::optional<coph_shaping> &, double);

// ---------------------------------------------------------------------------
// The energy account
// ---------------------------------------------------------------------------

void coph_energy::add(const coph_balance_terms &b) {
  work += b.work;
  heat += b.heat;
  kinetic_change += b.kinetic_change;
  residual += b.residual;
  ++steps;
}

void coph_energy::add_unallocated(const coph_vec2 &e, const coph_vec2 &q_before,
                                  const coph_vec2 &q_after) {
  unallocated_work += e[0] * (q_after[0] - q_before[0]) + e[1] * (q_after[1] - q_before[1]);
}

} // namespace nav
} // namespace cvc

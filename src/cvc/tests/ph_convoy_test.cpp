/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A measurement, not a feature: does a convoy hold its formation better when the formation
// linkage is a power-preserving port-Hamiltonian interconnection (cvc/ph/compose.h) than when each
// follower chases its slot? docs/roadmap/PH-CONVOY-FORMATION.md has the numbers this prints and
// what they mean. The model is point masses on a straight road, so the lateral axis is decoupled
// and stays at rest: heading-dependent attachment (real SE(2) vehicles) is outside it.
//
// The follow laws, all on the same plant (unit mass, linear drag, the engine cancelling its own
// drag from its own speed, net acceleration held to [-4, +2] m/s^2 along the road):
//
//   chase    slot chasing, as a trail-pursuit follower does it: a speed
//            v_w = clamp(0.4 (slot error), 0, 10) and a = clamp((v_w - v) / 0.8, -4, 2). No
//            reaction on the parent; it settles 2.5 v behind its slot (a 2.5 s time headway).
//   pd       a virtual spring-damper link pulling the follower only: the slot spring k plus a
//            dashpot branch (a Maxwell spring and dashpot), force on the child, none on the parent.
//   ph       the same link laws as pd, as cvc::ph storage blocks wired by connect_incidence from
//            the linkage: each link pulls its child and holds its parent back, so the coupling
//            does no net work. The lead is a velocity source (it drives its own profile and does
//            not feel the column), entering the links it starts as an external flow input.
//   ph_twin  chase's gains made power-preserving: pure spring links (k = 0.4 / 0.8) and the
//            follower's 1 / 0.8 speed damping on its absolute velocity, reactions on the parent.
//   maxwell  the microgrid line law taken literally: one storage state per axis, H = 1/2 k e^2,
//            R the link damping, so the spring and dashpot are in series and the link has no
//            rest length.
//   pd_lead, ph_lead
//            pd and ph plus a dashpot from every follower to the lead's speed (broadcast by it):
//            the same information for both, so the pair isolates what the reactions do.
//
// The cvc::ph runs step the composite itself (split step, midpoint coupling) and keep its books;
// an independent RK4 of the same equations checks the wiring. The column-length and gain studies
// take a minute: they run when CVC_PH_CONVOY_EXPERIMENTS=1.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ph/compose.h>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace cvc::ph;

#define PH_CONVOY_EXPERIMENT()                                                                     \
  if (!std::getenv("CVC_PH_CONVOY_EXPERIMENTS") ||                                                 \
      std::string(std::getenv("CVC_PH_CONVOY_EXPERIMENTS")) != "1")                                \
  GTEST_SKIP() << "an experiment: set CVC_PH_CONVOY_EXPERIMENTS=1 to run it"

// ── the plant and the follower's tuning ─────────────────────────────────────────────────────────

constexpr double kStandoff = 22.0;                // m: a slot's distance behind its parent
constexpr double kDrag = 0.02;                    // 1/s: rolling + aero, linearised near 8 m/s
constexpr double kAccel = 2.0, kBrake = 4.0;      // m/s^2: the acceleration and braking limits
constexpr double kLat = 2.5;                      // m/s^2: the lateral limit
constexpr double kGapGain = 0.4, kSpeedTau = 0.8; // chase's gap gain (1/s) and speed lag (s)
constexpr double kVmax = 10.0;                    // m/s: chase's top speed
constexpr double kDt = 0.01;
constexpr double kSettleBand = 1.0; // m: settled once the slot error stays this close to its end

double clip_long(double a) { return std::clamp(a, -kBrake, kAccel); }
double clip_lat(double a) { return std::clamp(a, -kLat, kLat); }

// ── formations: a linkage, each follower's formation_parent (cvc/nav/nav_stats.h) and slot ─────

struct topology {
  std::string name;
  std::vector<int> parent;                   // parent[0] = -1: the lead
  std::vector<std::array<double, 2>> offset; // slot relative to the parent: (along, lateral)
  int n() const { return static_cast<int>(parent.size()); }
  // Vehicles in i's subtree, i included.
  int subtree(int i) const {
    int c = 1;
    for (int j = 0; j < n(); ++j)
      if (parent[j] == i)
        c += subtree(j);
    return c;
  }
};

topology column(int n) {
  topology t{"column", {}, {}};
  for (int i = 0; i < n; ++i) {
    t.parent.push_back(i - 1);
    t.offset.push_back({i ? -kStandoff : 0.0, 0.0});
  }
  return t;
}

// A wedge as a tree: 1 and 2 off the lead, 3 behind 1, 4 behind 2.
topology wedge() {
  topology t{"wedge", {-1, 0, 0, 1, 2}, {}};
  t.offset = {{0, 0},
              {-kStandoff, kStandoff},
              {-kStandoff, -kStandoff},
              {-kStandoff, kStandoff},
              {-kStandoff, -kStandoff}};
  return t;
}

// ── scenarios: the lead's commanded speed steps at t0 ───────────────────────────────────────────

struct scenario {
  std::string name;
  double v0, v1, t0, T;
  double vref(double t) const { return t + 1e-9 >= t0 ? v1 : v0; }
};

const scenario kStep{"speed step 4->8 m/s", 4.0, 8.0, 2.0, 90.0};
const scenario kStop{"emergency stop 8->0 m/s", 8.0, 0.0, 2.0, 90.0};

// The lead's own speed loop, the one every law shares: v' = clip((v_ref - v) / 0.8).
double lead_accel(const scenario &sc, double t, double v) {
  return clip_long((sc.vref(t) - v) / kSpeedTau);
}

struct lead_track {
  vecd x, v; // at t_k = k dt
};

lead_track lead(const scenario &sc) {
  const int steps = static_cast<int>(std::lround(sc.T / kDt));
  lead_track L;
  double x = 0, v = sc.v0;
  for (int k = 0; k <= steps; ++k) {
    L.x.push_back(x);
    L.v.push_back(v);
    const double t = k * kDt;
    const double a1 = lead_accel(sc, t, v);
    const double a2 = lead_accel(sc, t + kDt / 2, v + kDt / 2 * a1);
    const double a3 = lead_accel(sc, t + kDt / 2, v + kDt / 2 * a2);
    const double a4 = lead_accel(sc, t + kDt, v + kDt * a3);
    x += kDt / 6 * (v + 2 * (v + kDt / 2 * a1) + 2 * (v + kDt / 2 * a2) + (v + kDt * a3));
    v += kDt / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
  }
  return L;
}

// ── the follow laws ─────────────────────────────────────────────────────────────────────────────

// A link is parallel branches over the same relative motion; branch b stores 1/2 k_b e_b^2 and
// relaxes at rate r_b = R_b k_b (0: a pure spring). Its force on the child is sum_b k_b e_b.
struct link_law_spec {
  std::vector<double> k, relax;
  double follower_damping = 0; // the follower's own damping on its absolute speed (ph_twin)
  double lead_damping = 0;     // a dashpot from every follower to the lead's speed (*_lead)
  double lead_corner = 8.0;    // ... a Maxwell branch relaxing this fast
};

// The slot spring k plus a dashpot branch: a Maxwell spring and dashpot that relaxes at `corner`,
// so below the corner it acts as a dashpot c = k2 / corner on the relative speed.
link_law_spec zener(double k = 0.5, double c = 1.25, double corner = 8.0) {
  link_law_spec s;
  s.k = {k, c * corner};
  s.relax = {0.0, corner};
  return s;
}
link_law_spec chase_twin() {
  link_law_spec s;
  s.k = {kGapGain / kSpeedTau};
  s.relax = {0.0};
  s.follower_damping = 1.0 / kSpeedTau;
  return s;
}
link_law_spec maxwell(double k = 0.5, double c = 1.25) {
  link_law_spec s;
  s.k = {k};
  s.relax = {k / c};
  return s;
}
link_law_spec with_lead(link_law_spec s, double c = 1.25) {
  s.lead_damping = c;
  return s;
}

enum class law { chase, pd, ph, ph_twin, maxwell, pd_lead, ph_lead };
const law kLaws[] = {law::chase,   law::pd,      law::ph,     law::ph_twin,
                     law::maxwell, law::pd_lead, law::ph_lead};

const char *name_of(law l) {
  switch (l) {
  case law::chase:
    return "chase";
  case law::pd:
    return "pd";
  case law::ph:
    return "ph";
  case law::ph_twin:
    return "ph_twin";
  case law::maxwell:
    return "maxwell";
  case law::pd_lead:
    return "pd_lead";
  case law::ph_lead:
    return "ph_lead";
  }
  return "?";
}

// The pH laws: links hold their parent back, so the coupling does no net work.
bool power_preserving(law l) { return l != law::chase && l != law::pd && l != law::pd_lead; }

link_law_spec spec_of(law l) {
  switch (l) {
  case law::ph_twin:
    return chase_twin();
  case law::maxwell:
    return maxwell();
  case law::pd_lead:
  case law::ph_lead:
    return with_lead(zener());
  default:
    return zener();
  }
}

// The slot error the law settles at, per unit speed (its time headway).
double headway(law l, const topology &tp, int i, const link_law_spec &ls) {
  if (l == law::chase)
    return 1.0 / kGapGain;
  if (l == law::ph_twin)
    return tp.subtree(i) * ls.follower_damping / ls.k[0];
  return 0.0;
}

// ── a run's record ──────────────────────────────────────────────────────────────────────────────

struct link_book {
  double h_peak = 0, h_end = 0, dH = 0;
  double received = 0;   // work its ports took in from the followers
  double dissipated = 0; // the dashpot branches' loss
};

// A run in the road's frame. The composite runs also keep the books of the formation's own energy,
// in the lead's frame: the followers' kinetic energy relative to the lead plus the links' stored
// energy.
struct trace {
  int n = 0, steps = 0;
  std::vector<vecd> x, y, vx, vy, f; // [vehicle][k]; f: engine force per kg along the road
  int saturated = 0;                 // follower-steps whose demand was clipped
  bool has_books = false;
  books total{};               // the composite's step_books, summed
  double worst_residual = 0;   // per step
  std::vector<link_book> link; // [child vehicle]
  link_book lead_dampers;      // all of them together
  // [vehicle]: the links' work on it, the lead's manoeuvre's (its inertial input), its own loss.
  std::vector<double> from_links, from_lead, own_loss;
  void resize(int vehicles, int nsteps) {
    n = vehicles;
    steps = nsteps;
    for (auto *v : {&x, &y, &vx, &vy, &f})
      v->assign(n, vecd(steps + 1, 0.0));
  }
};

// Starting positions: the lead at 0, each follower at its law's steady slot error at v0.
std::vector<std::array<double, 2>> start_positions(law l, const topology &tp,
                                                   const link_law_spec &ls, double v0) {
  std::vector<std::array<double, 2>> q(tp.n(), {0.0, 0.0});
  for (int i = 1; i < tp.n(); ++i) {
    const auto &p = q[tp.parent[i]];
    q[i] = {p[0] + tp.offset[i][0] - headway(l, tp, i, ls) * v0, p[1] + tp.offset[i][1]};
  }
  return q;
}

// ── RK4 of the follow laws written out (chase, pd, and the pH laws as a reference) ─────────────

struct rk4_model {
  law l;
  const topology &tp;
  const scenario &sc;
  link_law_spec ls;
  int nb() const { return static_cast<int>(ls.k.size()); }
  // State: per vehicle (x, y, vx, vy), per follower's link nb branches of (ex, ey), then per
  // follower its lead damper's (ex, ey).
  int link_off(int i) const { return 4 * tp.n() + (i - 1) * 2 * nb(); }
  int lead_off(int i) const { return 4 * tp.n() + (tp.n() - 1) * 2 * nb() + (i - 1) * 2; }
  int dim() const { return lead_off(tp.n()); }
  double lead_k() const { return ls.lead_damping * ls.lead_corner; }

  // The net accelerations the law demands (before the limits), along and across the road.
  void demand(const vecd &s, int i, double &ax, double &ay) const {
    const int p = tp.parent[i];
    if (l == law::chase) {
      const double sx = s[4 * p] + tp.offset[i][0] - s[4 * i];
      const double sy = s[4 * p + 1] + tp.offset[i][1] - s[4 * i + 1];
      const double wx = std::clamp(kGapGain * sx, 0.0, kVmax); // never backs up
      const double wy = std::clamp(kGapGain * sy, -kVmax, kVmax);
      ax = (wx - s[4 * i + 2]) / kSpeedTau;
      ay = (wy - s[4 * i + 3]) / kSpeedTau;
      return;
    }
    ax = -ls.follower_damping * s[4 * i + 2] + lead_k() * s[lead_off(i)];
    ay = -ls.follower_damping * s[4 * i + 3] + lead_k() * s[lead_off(i) + 1];
    auto force = [&](int c, double &fx, double &fy) {
      fx = fy = 0;
      for (int b = 0; b < nb(); ++b) {
        fx += ls.k[b] * s[link_off(c) + 2 * b];
        fy += ls.k[b] * s[link_off(c) + 2 * b + 1];
      }
    };
    double fx, fy;
    force(i, fx, fy);
    ax += fx;
    ay += fy;
    if (power_preserving(l))
      for (int c = 1; c < tp.n(); ++c)
        if (tp.parent[c] == i) {
          force(c, fx, fy);
          ax -= fx;
          ay -= fy;
        }
  }

  void deriv(double t, const vecd &s, vecd &d) const {
    d.assign(s.size(), 0.0);
    d[0] = s[2];
    d[1] = s[3];
    d[2] = lead_accel(sc, t, s[2]);
    for (int i = 1; i < tp.n(); ++i) {
      double ax, ay;
      demand(s, i, ax, ay);
      d[4 * i] = s[4 * i + 2];
      d[4 * i + 1] = s[4 * i + 3];
      d[4 * i + 2] = clip_long(ax);
      d[4 * i + 3] = clip_lat(ay);
      if (l == law::chase)
        continue;
      const int p = tp.parent[i];
      for (int b = 0; b < nb(); ++b) {
        const int o = link_off(i) + 2 * b;
        d[o] = s[4 * p + 2] - s[4 * i + 2] - ls.relax[b] * s[o];
        d[o + 1] = s[4 * p + 3] - s[4 * i + 3] - ls.relax[b] * s[o + 1];
      }
      const int o = lead_off(i);
      d[o] = s[2] - s[4 * i + 2] - ls.lead_corner * s[o];
      d[o + 1] = s[3] - s[4 * i + 3] - ls.lead_corner * s[o + 1];
    }
  }
};

trace run_rk4(law l, const topology &tp, const scenario &sc, const link_law_spec &ls) {
  rk4_model m{l, tp, sc, ls};
  const int steps = static_cast<int>(std::lround(sc.T / kDt));
  trace r;
  r.resize(tp.n(), steps);
  vecd s(m.dim(), 0.0), k1, k2, k3, k4, tmp(m.dim());
  const auto q0 = start_positions(l, tp, ls, sc.v0);
  for (int i = 0; i < tp.n(); ++i) {
    s[4 * i] = q0[i][0];
    s[4 * i + 1] = q0[i][1];
    s[4 * i + 2] = sc.v0;
  }
  if (l != law::chase) // the links start at the steady state: only the slot springs are loaded
    for (int i = 1; i < tp.n(); ++i)
      s[m.link_off(i)] = headway(l, tp, i, ls) * sc.v0;
  auto axpy = [&](const vecd &a, double h, const vecd &b) -> const vecd & {
    for (size_t j = 0; j < a.size(); ++j)
      tmp[j] = a[j] + h * b[j];
    return tmp;
  };
  for (int k = 0;; ++k) {
    for (int i = 0; i < tp.n(); ++i) {
      r.x[i][k] = s[4 * i];
      r.y[i][k] = s[4 * i + 1];
      r.vx[i][k] = s[4 * i + 2];
      r.vy[i][k] = s[4 * i + 3];
      double ax = lead_accel(sc, k * kDt, s[2]), ay = 0;
      if (i > 0) {
        m.demand(s, i, ax, ay);
        if (clip_long(ax) != ax)
          ++r.saturated;
        ax = clip_long(ax);
      }
      r.f[i][k] = ax + kDrag * s[4 * i + 2];
    }
    if (k == steps)
      break;
    const double t = k * kDt;
    m.deriv(t, s, k1);
    m.deriv(t + kDt / 2, axpy(s, kDt / 2, k1), k2);
    m.deriv(t + kDt / 2, axpy(s, kDt / 2, k2), k3);
    m.deriv(t + kDt, axpy(s, kDt, k3), k4);
    for (size_t j = 0; j < s.size(); ++j)
      s[j] += kDt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
  }
  return r;
}

// ── the pH formation as a cvc::ph composite ─────────────────────────────────────────────────────
//
// It is written in the lead's frame: a follower's q is its position less the lead's, p its
// momentum less the lead's speed, and the lead's acceleration is an inertial input on each
// follower. Every relative velocity then reaches a link through the pairing, as in the microgrid
// (vehicle velocity for bus voltage, link force for line current). Fed in as a flow input on the
// links off the lead instead, the lead's speed reaches them in the local step and the follower's
// in the coupling half-steps, so within every substep the link holds an elongation of +-v d / 2
// that its stiff branch turns into force; the slot it holds then creeps from the geometric one
// (9 cm in 90 s at 8 m/s, 2 substeps, the stiffest links here), at constant speed.

// A follower: q = (x, y), p = m v, H = 1/2 |p|^2 / m, damping its drag (plus ph_twin's speed
// damping), one coupling port and one input on each momentum.
law_ptr vehicle_law(double damping) {
  canonical_spec s;
  s.n = 2;
  s.mass = {1.0, 1.0};
  s.damping = {damping, damping};
  s.ports = 2;
  s.inputs = 2;
  s.B = {0, 0, 0, 0, 1, 0, 0, 1};
  s.G = s.B;
  s.scheme = conservative_scheme::leapfrog;
  return std::make_shared<canonical_law>(s);
}

// A link: per branch an elongation (ex, ey), H = 1/2 k_b |e_b|^2, R_b = relax_b / k_b. Every
// branch sees the port's relative velocity; the port's effort is the link force.
law_ptr link_law(const std::vector<double> &k, const std::vector<double> &relax) {
  const int nb = static_cast<int>(k.size()), n = 2 * nb;
  linear_spec s;
  s.n = n;
  s.ports = 2;
  s.Q.assign(n * n, 0.0);
  s.J.assign(n * n, 0.0);
  s.R.assign(n * n, 0.0);
  s.B.assign(n * 2, 0.0);
  for (int b = 0; b < nb; ++b)
    for (int a = 0; a < 2; ++a) {
      const int r = 2 * b + a;
      s.Q[r * n + r] = k[b];
      s.R[r * n + r] = relax[b] / k[b];
      s.B[r * 2 + a] = 1.0;
    }
  return std::make_shared<linear_law>(s);
}

struct formation {
  composite sys;
  std::vector<int> veh, lnk, damper; // the blocks of follower i, of the link into it, of its lead
                                     // damper (-1: none)
};

// The same law objects wired by the linkage: links are lines, followers are buses. A link off the
// lead, and a lead damper, has its follower as its only bus: the lead is at rest in its own frame.
formation build(const topology &tp, const law_ptr &veh, const law_ptr &lnk, const law_ptr &damper) {
  formation f;
  f.veh.assign(tp.n(), -1);
  f.lnk.assign(tp.n(), -1);
  f.damper.assign(tp.n(), -1);
  for (int i = 1; i < tp.n(); ++i)
    f.veh[i] = f.sys.add_block(veh, "veh" + std::to_string(i));
  for (int i = 1; i < tp.n(); ++i)
    f.lnk[i] =
        f.sys.add_block(lnk, "link" + std::to_string(tp.parent[i]) + "-" + std::to_string(i));
  if (damper)
    for (int i = 1; i < tp.n(); ++i)
      f.damper[i] = f.sys.add_block(damper, "lead-damper" + std::to_string(i));
  const int m = tp.n() - 1, lines = damper ? 2 * m : m;
  vecd A(size_t(m) * lines, 0.0); // followers x lines: +1 at the child, -1 at a follower parent
  std::vector<int> buses, ls;
  for (int i = 1; i < tp.n(); ++i) {
    buses.push_back(f.veh[i]);
    ls.push_back(f.lnk[i]);
    A[size_t(i - 1) * lines + (i - 1)] = 1.0;
    if (tp.parent[i] > 0)
      A[size_t(tp.parent[i] - 1) * lines + (i - 1)] = -1.0;
  }
  if (damper)
    for (int i = 1; i < tp.n(); ++i) {
      ls.push_back(f.damper[i]);
      A[size_t(i - 1) * lines + m + (i - 1)] = 1.0;
    }
  for (int axis = 0; axis < 2; ++axis)
    f.sys.connect_incidence(buses, axis, ls, axis, A);
  return f;
}

formation build(const topology &tp, const link_law_spec &ls) {
  return build(tp, vehicle_law(kDrag + ls.follower_damping), link_law(ls.k, ls.relax),
               ls.lead_damping > 0 ? link_law({ls.lead_damping * ls.lead_corner}, {ls.lead_corner})
                                   : nullptr);
}

// The pairing's inputs at x: u_hat = -C y_hat (port_dim).
vecd coupling_inputs(const composite &s, const vecd &x) {
  const int ny = s.port_dim();
  vecd y(ny), uh(ny, 0.0);
  s.outputs(x.data(), y.data());
  const vecd &C = s.pairing();
  for (int r = 0; r < ny; ++r)
    for (int c = 0; c < ny; ++c)
      uh[r] -= C[size_t(r) * ny + c] * y[c];
  return uh;
}

// Per block at x, u held: the port power received (u_hat . y_hat), the loss (e . R e) and the
// external inputs' power (e . G u).
struct block_powers {
  vecd port, loss, input;
};
block_powers powers_of(const composite &s, const vecd &x, const vecd &u) {
  block_powers P;
  const int nb = s.blocks();
  P.port.assign(nb, 0.0);
  P.loss.assign(nb, 0.0);
  P.input.assign(nb, 0.0);
  vecd e(s.state_dim()), y(s.port_dim());
  s.effort(x.data(), e.data());
  s.outputs(x.data(), y.data());
  const vecd uh = coupling_inputs(s, x);
  vecd J, R, G;
  for (int b = 0; b < nb; ++b) {
    const block_law &L = s.law(b);
    const int n = L.state_dim(), np = L.port_dim(), ni = L.input_dim();
    const int xo = s.state_offset(b), yo = s.port_offset(b), uo = s.input_offset(b);
    for (int j = 0; j < np; ++j)
      P.port[b] += uh[yo + j] * y[yo + j];
    J.assign(size_t(n) * n, 0.0);
    R.assign(size_t(n) * n, 0.0);
    G.assign(size_t(n) * ni, 0.0);
    L.structure(x.data() + xo, J.data(), R.data());
    L.input_map(x.data() + xo, G.data());
    for (int r = 0; r < n; ++r)
      for (int c = 0; c < n; ++c)
        P.loss[b] += e[xo + r] * R[size_t(r) * n + c] * e[xo + c];
    for (int r = 0; r < n; ++r)
      for (int c = 0; c < ni; ++c)
        P.input[b] += e[xo + r] * G[size_t(r) * ni + c] * u[uo + c];
  }
  return P;
}

trace run_ph(law l, const topology &tp, const scenario &sc, const link_law_spec &ls) {
  const formation f = build(tp, ls);
  const composite &s = f.sys;
  const lead_track L = lead(sc);
  const int steps = static_cast<int>(std::lround(sc.T / kDt));
  trace r;
  r.resize(tp.n(), steps);
  r.has_books = true;
  r.link.assign(tp.n(), {});
  r.from_links.assign(tp.n(), 0.0);
  r.from_lead.assign(tp.n(), 0.0);
  r.own_loss.assign(tp.n(), 0.0);

  vecd x(s.state_dim(), 0.0), u(s.input_dim(), 0.0);
  const auto q0 = start_positions(l, tp, ls, sc.v0); // the lead starts at 0: q0 is relative
  for (int i = 1; i < tp.n(); ++i) {
    const int o = s.state_offset(f.veh[i]);
    x[o] = q0[i][0];
    x[o + 1] = q0[i][1];
    x[s.state_offset(f.lnk[i])] = headway(l, tp, i, ls) * sc.v0;
  }
  const vecd xstart = x;
  transition_opts o;
  o.dt = kDt;
  o.substeps = 4;
  o.coupling = coupling_scheme::midpoint;
  for (int k = 0;; ++k) {
    // The inputs held over step k, per follower: minus the lead's acceleration, its engine
    // cancelling its own drag, ph_twin's speed damping on the lead's speed (the block damps only
    // the speed relative to it), and past the limits what the engine cannot deliver.
    const vecd uh = coupling_inputs(s, x);
    r.x[0][k] = L.x[k];
    r.vx[0][k] = L.v[k];
    r.f[0][k] = lead_accel(sc, k * kDt, L.v[k]) + kDrag * L.v[k];
    const double al = k < steps ? (L.v[k + 1] - L.v[k]) / kDt : 0.0;
    const double vl = k < steps ? (L.x[k + 1] - L.x[k]) / kDt : L.v[k];
    for (int i = 1; i < tp.n(); ++i) {
      const int xo = s.state_offset(f.veh[i]), uo = s.input_offset(f.veh[i]);
      const int yo = s.port_offset(f.veh[i]);
      const double wx = x[xo + 2], wy = x[xo + 3], vx = wx + L.v[k];
      r.x[i][k] = x[xo] + L.x[k];
      r.y[i][k] = x[xo + 1];
      r.vx[i][k] = vx;
      r.vy[i][k] = wy;
      // The net acceleration the law demands, in the road's frame.
      const double ax = uh[yo] - ls.follower_damping * vx;
      const double ay = uh[yo + 1] - ls.follower_damping * wy;
      if (clip_long(ax) != ax)
        ++r.saturated;
      u[uo] = -al + kDrag * wx - ls.follower_damping * vl + clip_long(ax) - ax;
      u[uo + 1] = kDrag * wy + clip_lat(ay) - ay;
      r.f[i][k] = clip_long(ax) + kDrag * vx;
    }
    if (k == steps)
      break;
    const vecd x0 = x;
    const block_powers P0 = powers_of(s, x0, u);
    step(s, o, u.data(), x.data());
    const block_powers P1 = powers_of(s, x, u);
    const books b = step_books(s, kDt, x0.data(), x.data(), u.data());
    r.total.dH += b.dH;
    r.total.supplied += b.supplied;
    r.total.dissipated += b.dissipated;
    r.total.residual += b.residual;
    r.worst_residual = std::max(r.worst_residual, std::fabs(b.residual));
    auto trap = [&](const vecd &p0, const vecd &p1, int bl) { return kDt / 2 * (p0[bl] + p1[bl]); };
    for (int i = 1; i < tp.n(); ++i) {
      const int bl = f.lnk[i], bv = f.veh[i], xo = s.state_offset(bv);
      r.link[i].received += trap(P0.port, P1.port, bl);
      r.link[i].dissipated += trap(P0.loss, P1.loss, bl);
      r.link[i].h_peak =
          std::max(r.link[i].h_peak, s.law(bl).energy(x.data() + s.state_offset(bl)));
      if (f.damper[i] >= 0) {
        r.lead_dampers.received += trap(P0.port, P1.port, f.damper[i]);
        r.lead_dampers.dissipated += trap(P0.loss, P1.loss, f.damper[i]);
      }
      r.from_links[i] += trap(P0.port, P1.port, bv);
      r.from_lead[i] += kDt / 2 * -al * (x0[xo + 2] + x[xo + 2]);
      r.own_loss[i] += trap(P0.loss, P1.loss, bv);
    }
  }
  for (int i = 1; i < tp.n(); ++i) {
    const int bl = f.lnk[i];
    link_book &lb = r.link[i];
    lb.h_end = s.law(bl).energy(x.data() + s.state_offset(bl));
    lb.dH = lb.h_end - s.law(bl).energy(xstart.data() + s.state_offset(bl));
    if (f.damper[i] >= 0) {
      const double h = s.law(f.damper[i]).energy(x.data() + s.state_offset(f.damper[i]));
      r.lead_dampers.h_end += h;
      r.lead_dampers.dH += h;
    }
  }
  return r;
}

trace run(law l, const topology &tp, const scenario &sc, const link_law_spec &ls) {
  return power_preserving(l) ? run_ph(l, tp, sc, ls) : run_rk4(l, tp, sc, ls);
}

// ── the measures ────────────────────────────────────────────────────────────────────────────────

struct follower_measures {
  double peak = 0;      // max |slot error| (m)
  double end = 0;       // slot error at T (signed: + behind its slot)
  double dev = 0;       // max |slot error - its end value|
  double settle = 0;    // s after t0 until it stays within kSettleBand of its end value
  double overshoot = 0; // past its end value, opposite the largest excursion, / that excursion
  double gamma = 0;     // max |slot error - headway v|: the spacing policy's error
  double vmin = 0;      // lowest speed (< 0: it backed up)
};

struct measures {
  std::vector<follower_measures> f; // [vehicle], 0 unused
  double peak = 0, settle = 0, overshoot = 0, end = 0;
  double min_link_gap = 1e300, min_pair = 1e300;
  double gamma_ratio = 0; // max over follower parents of gamma(child) / gamma(parent)
  double a_max = -1e300, a_min = 1e300, vmin = 1e300;
  double e_traction = 0, e_braking = 0; // followers' engine work, J/kg
  double sat_frac = 0;
};

measures measure(law l, const topology &tp, const scenario &sc, const link_law_spec &ls,
                 const trace &r) {
  measures m;
  m.f.assign(tp.n(), {});
  const int k0 = static_cast<int>(std::lround(sc.t0 / kDt));
  for (int i = 1; i < tp.n(); ++i) {
    const int p = tp.parent[i];
    follower_measures &fm = m.f[i];
    auto slot = [&](int k) { return r.x[p][k] + tp.offset[i][0] - r.x[i][k]; };
    auto slot_y = [&](int k) { return r.y[p][k] + tp.offset[i][1] - r.y[i][k]; };
    fm.end = slot(r.steps);
    fm.vmin = 1e300;
    int kp = 0;
    double last_out = sc.t0;
    for (int k = 0; k <= r.steps; ++k) {
      const double s = slot(k), sy = slot_y(k);
      fm.peak = std::max(fm.peak, std::hypot(s, sy));
      if (std::fabs(s - fm.end) > fm.dev) {
        fm.dev = std::fabs(s - fm.end);
        kp = k;
      }
      if (std::fabs(s - fm.end) > kSettleBand)
        last_out = k * kDt;
      fm.gamma = std::max(fm.gamma, std::fabs(s - headway(l, tp, i, ls) * r.vx[i][k]));
      fm.vmin = std::min(fm.vmin, r.vx[i][k]);
      m.min_link_gap =
          std::min(m.min_link_gap, std::hypot(r.x[p][k] - r.x[i][k], r.y[p][k] - r.y[i][k]));
      for (int j = 0; j < i; ++j)
        m.min_pair = std::min(m.min_pair, std::hypot(r.x[j][k] - r.x[i][k], r.y[j][k] - r.y[i][k]));
      const double a = r.f[i][k] - kDrag * r.vx[i][k];
      m.a_max = std::max(m.a_max, a);
      m.a_min = std::min(m.a_min, a);
      const double P = r.f[i][k] * r.vx[i][k];
      (P > 0 ? m.e_traction : m.e_braking) += std::fabs(P) * kDt;
    }
    fm.settle = std::max(0.0, last_out - sc.t0);
    const double sgn = slot(kp) - fm.end > 0 ? 1.0 : -1.0;
    for (int k = std::max(kp, k0); k <= r.steps; ++k)
      fm.overshoot = std::max(fm.overshoot, -sgn * (slot(k) - fm.end));
    fm.overshoot = fm.dev > 0 ? fm.overshoot / fm.dev : 0.0;
    m.peak = std::max(m.peak, fm.peak);
    m.settle = std::max(m.settle, fm.settle);
    m.overshoot = std::max(m.overshoot, fm.overshoot);
    m.end = std::max(m.end, std::fabs(fm.end));
    m.vmin = std::min(m.vmin, fm.vmin);
  }
  for (int i = 1; i < tp.n(); ++i)
    if (tp.parent[i] > 0 && m.f[tp.parent[i]].gamma > 1e-9)
      m.gamma_ratio = std::max(m.gamma_ratio, m.f[i].gamma / m.f[tp.parent[i]].gamma);
  m.sat_frac = double(r.saturated) / ((tp.n() - 1) * double(r.steps + 1));
  return m;
}

// ── reports ─────────────────────────────────────────────────────────────────────────────────────

void print_header(const topology &tp, const scenario &sc) {
  std::printf("\n[ph-convoy] %s, %d vehicles, %s (t0 %.0f s, %.0f s run)\n", tp.name.c_str(),
              tp.n(), sc.name.c_str(), sc.t0, sc.T);
  std::printf("[ph-convoy] %-8s %7s %7s %7s %7s %7s %7s %6s %6s %6s %6s %7s %7s %5s\n", "law",
              "peak", "end", "settle", "oversh", "gapmin", "pairmin", "gam_r", "amax", "amin",
              "vmin", "E+", "E-", "sat");
}

void print_row(const char *label, const measures &m) {
  std::printf("[ph-convoy] %-8s %7.2f %7.2f %7.1f %6.0f%% %7.2f %7.2f %6.2f %6.2f %6.2f %6.2f "
              "%7.1f %7.1f %4.1f%%\n",
              label, m.peak, m.end, m.settle, 100 * m.overshoot, m.min_link_gap, m.min_pair,
              m.gamma_ratio, m.a_max, m.a_min, m.vmin, m.e_traction, m.e_braking, 100 * m.sat_frac);
}

void print_followers(law l, const measures &m) {
  std::printf("[ph-convoy]   %-8s peak/settle/gamma by follower:", name_of(l));
  for (size_t i = 1; i < m.f.size(); ++i)
    std::printf("  %zu: %.2f/%.1f/%.2f", i, m.f[i].peak, m.f[i].settle, m.f[i].gamma);
  std::printf("\n");
}

void print_books(law l, const topology &tp, const trace &r) {
  std::printf("[ph-convoy]   %-8s books (J/kg, lead frame): dH %+.3f = supplied %+.3f - dissipated "
              "%.3f + residual %+.1e (worst step %.1e)\n",
              name_of(l), r.total.dH, r.total.supplied, r.total.dissipated, r.total.residual,
              r.worst_residual);
  for (int i = 1; i < tp.n(); ++i) {
    const link_book &b = r.link[i];
    std::printf("[ph-convoy]     link %d-%d: H peak %7.3f end %7.3f, took in %+8.3f, dissipated "
                "%7.3f | veh%d: lead's manoeuvre %+8.3f, links %+8.3f, own loss %7.3f\n",
                tp.parent[i], i, b.h_peak, b.h_end, b.received, b.dissipated, i, r.from_lead[i],
                r.from_links[i], r.own_loss[i]);
  }
  if (r.lead_dampers.received != 0)
    std::printf("[ph-convoy]     lead dampers: took in %+8.3f, dissipated %7.3f, end %7.3f\n",
                r.lead_dampers.received, r.lead_dampers.dissipated, r.lead_dampers.h_end);
}

struct cell {
  law l;
  trace r;
  measures m;
};

std::vector<cell> scenario_table(const topology &tp, const scenario &sc) {
  std::vector<cell> out;
  print_header(tp, sc);
  for (law l : kLaws) {
    const link_law_spec ls = spec_of(l);
    trace r = run(l, tp, sc, ls);
    const measures m = measure(l, tp, sc, ls, r);
    out.push_back({l, std::move(r), m});
    print_row(name_of(l), m);
  }
  for (const cell &c : out)
    print_followers(c.l, c.m);
  for (const cell &c : out)
    if (c.r.has_books)
      print_books(c.l, tp, c.r);
  return out;
}

const cell &of(const std::vector<cell> &t, law l) {
  for (const cell &c : t)
    if (c.l == l)
      return c;
  throw std::logic_error("no such law");
}

// ── tests ───────────────────────────────────────────────────────────────────────────────────────

// The composite is the system the RK4 writes out: same states to integration error, for every pH
// law and both wirings. Its pairing does no work, whichever linkage wires it.
TEST(PhConvoy, TheCompositeIsTheFormationItStandsFor) {
  for (const topology &tp : {column(5), wedge()})
    for (law l : kLaws) {
      if (!power_preserving(l))
        continue;
      for (const scenario &sc : {kStep, kStop}) {
        const link_law_spec ls = spec_of(l);
        const trace a = run_ph(l, tp, sc, ls), b = run_rk4(l, tp, sc, ls);
        double worst = 0;
        for (int i = 0; i < tp.n(); ++i)
          for (int k = 0; k <= a.steps; ++k)
            worst = std::max(worst, std::fabs(a.x[i][k] - b.x[i][k]));
        SCOPED_TRACE(tp.name + " " + name_of(l) + " " + sc.name);
        // Millimetres over 90 s: the split step's own error (second order), and the composite
        // holding the engines' drag cancelling and limits over each 10 ms step where the RK4 is
        // continuous (it scales with the speed change; more substeps do not remove it). A wrong
        // sign in the wiring is metres.
        EXPECT_LT(worst, 5e-3);
        EXPECT_LT(std::fabs(a.total.residual), 1e-3 * std::max(1.0, a.total.dissipated));
        EXPECT_NEAR(a.total.dH, a.total.supplied - a.total.dissipated + a.total.residual, 1e-9);
      }
    }
  const link_law_spec ls = with_lead(zener());
  const law_ptr veh = vehicle_law(kDrag), lnk = link_law(ls.k, ls.relax),
                dmp = link_law({ls.lead_damping * ls.lead_corner}, {ls.lead_corner});
  for (const topology &tp : {column(5), wedge()})
    for (const law_ptr &d : {law_ptr(), dmp}) {
      const formation f = build(tp, veh, lnk, d);
      const balance_summary b = check_balance_random(f.sys, 200, 7u, 3.0);
      EXPECT_LT(b.worst.internal_error, 1e-14) << tp.name;
      EXPECT_LT(b.worst.rate_error, 1e-12) << tp.name;
    }
}

// Why the composite is written in the lead's frame. One follower cruises 22 m behind a lead at
// 8 m/s, at rest in its slot, for 90 s. With the lead's speed fed to the link as a flow input, the
// split step lets the follower slip back from its slot although nothing changes, by an amount
// second order in the substep; in the lead's frame it stays put.
TEST(PhConvoy, AFlowInputOnALinkCreepsUnderTheSplitStep) {
  const link_law_spec ls = zener(2.0, 5.0);
  linear_spec flow = std::static_pointer_cast<const linear_law>(link_law(ls.k, ls.relax))->spec();
  flow.inputs = 2;
  flow.G = flow.B;
  auto creep = [&](bool lead_frame, int substeps) {
    composite c;
    const int v = c.add_block(vehicle_law(kDrag), "veh1");
    const int e = c.add_block(lead_frame ? link_law(ls.k, ls.relax)
                                         : law_ptr(std::make_shared<linear_law>(flow)),
                              "link0-1");
    for (int axis = 0; axis < 2; ++axis)
      c.connect_incidence({v}, axis, {e}, axis, {1.0});
    vecd x(c.state_dim(), 0.0), u(c.input_dim(), 0.0);
    const double speed = 8.0, x0 = -kStandoff;
    x[c.state_offset(v)] = x0;
    if (!lead_frame)
      x[c.state_offset(v) + 2] = speed;
    transition_opts o;
    o.dt = kDt;
    o.substeps = substeps;
    const int steps = static_cast<int>(std::lround(90.0 / kDt));
    for (int k = 0; k < steps; ++k) {
      u[c.input_offset(v)] = kDrag * x[c.state_offset(v) + 2];
      if (!lead_frame)
        u[c.input_offset(e)] = speed;
      step(c, o, u.data(), x.data());
    }
    const double lead_x = lead_frame ? 0.0 : speed * steps * kDt;
    return (lead_x + x0) - x[c.state_offset(v)]; // its slot error, + behind
  };
  const double flow2 = creep(false, 2), flow4 = creep(false, 4);
  std::printf("[ph-convoy] slot creep at 8 m/s over 90 s: flow input %.2e m (2 substeps), %.2e m "
              "(4); lead frame %.2e m\n",
              flow2, flow4, creep(true, 2));
  EXPECT_GT(flow2, 1e-2);
  EXPECT_NEAR(flow2 / flow4, 4.0, 0.2);
  EXPECT_LT(std::fabs(creep(true, 2)), 1e-12);
}

// The scenario matrix (5 vehicles, column and wedge, the step and the stop). The checks pin the
// write-up's comparisons, not its decimals.
TEST(PhConvoy, FollowLawsOnTheStepAndTheStop) {
  for (const topology &tp : {column(5), wedge()})
    for (const scenario &sc : {kStep, kStop}) {
      const auto t = scenario_table(tp, sc);
      SCOPED_TRACE(tp.name + " " + sc.name);
      const measures &chase = of(t, law::chase).m, &pd = of(t, law::pd).m, &ph = of(t, law::ph).m;
      const measures &pdl = of(t, law::pd_lead).m, &phl = of(t, law::ph_lead).m;
      // chase holds 2.5 v behind its slot, never backs up and never closes inside its standoff;
      // the constant-spacing links end on the slot.
      EXPECT_NEAR(chase.f[1].peak, 2.5 * std::max(sc.v0, sc.v1), 0.1);
      EXPECT_GE(chase.vmin, -1e-9);
      EXPECT_GT(chase.min_link_gap, kStandoff - 0.1);
      for (const measures *m : {&pd, &ph, &pdl, &phl})
        EXPECT_LT(m->end, 0.1);
      // The reactions, at equal information (pd against ph, pd_lead against ph_lead): a larger
      // peak, a longer settling and a closer approach, but no growth down the column.
      EXPECT_GT(ph.peak, pd.peak);
      EXPECT_GT(ph.settle, pd.settle);
      EXPECT_LT(ph.min_link_gap, pd.min_link_gap);
      EXPECT_GT(phl.peak, pdl.peak);
      EXPECT_GT(phl.settle, pdl.settle);
      EXPECT_LE(phl.min_link_gap, pdl.min_link_gap + 1e-6);
      EXPECT_GT(pd.gamma_ratio, 1.05);
      EXPECT_LT(ph.gamma_ratio, 1.0);
      // The lead's speed, not the pairing, is what beats chase: pd_lead settles first.
      EXPECT_LT(pdl.settle, chase.settle);
      // The line law taken literally has no rest length: the slot it ends on is not the slot.
      EXPECT_GT(of(t, law::maxwell).m.end, 5.0);
      // chase's headway made power-preserving piles up along the column: the link off the lead
      // carries every follower's damping behind it.
      EXPECT_NEAR(of(t, law::ph_twin).m.f[1].peak, tp.subtree(1) * 2.5 * std::max(sc.v0, sc.v1),
                  0.5);
    }
}

// How the column's response scales with its length, at the default links and at the stiffest,
// most damped ones of the gain study.
TEST(PhConvoy, ColumnLength) {
  PH_CONVOY_EXPERIMENT();
  struct variant {
    const char *label;
    law l;
    link_law_spec ls;
  };
  const variant vs[] = {{"chase", law::chase, zener()},
                        {"pd", law::pd, zener()},
                        {"ph", law::ph, zener()},
                        {"pd k2c5", law::pd, zener(2.0, 5.0)},
                        {"ph k2c5", law::ph, zener(2.0, 5.0)},
                        {"pd_lead", law::pd_lead, spec_of(law::pd_lead)},
                        {"ph_lead", law::ph_lead, spec_of(law::ph_lead)}};
  for (const scenario &base : {kStep, kStop})
    for (int n : {5, 10, 20}) {
      scenario sc = base;
      sc.T = 400.0;
      const topology tp = column(n);
      print_header(tp, sc);
      for (const variant &v : vs) {
        print_row(v.label, measure(v.l, tp, sc, v.ls, run(v.l, tp, sc, v.ls)));
      }
    }
}

// pd and ph over their link gains, so ph's showing is not an artefact of borrowing chase's.
TEST(PhConvoy, LinkGains) {
  PH_CONVOY_EXPERIMENT();
  const topology tp = column(5);
  for (const scenario &sc : {kStep, kStop}) {
    std::printf("\n[ph-convoy] link gains, column of 5, %s: peak / settle / gapmin / vmin / sat\n",
                sc.name.c_str());
    std::printf("[ph-convoy] %5s %5s | %-34s | %-34s\n", "k", "c", "pd", "ph");
    for (double k : {0.25, 0.5, 1.0, 2.0})
      for (double c : {0.5, 1.25, 2.5, 5.0}) {
        const link_law_spec ls = zener(k, c);
        const measures a = measure(law::pd, tp, sc, ls, run(law::pd, tp, sc, ls));
        const measures b = measure(law::ph, tp, sc, ls, run(law::ph, tp, sc, ls));
        std::printf("[ph-convoy] %5.2f %5.2f | %6.2f %5.1f %6.2f %6.2f %4.1f%% | %6.2f %5.1f %6.2f "
                    "%6.2f %4.1f%%\n",
                    k, c, a.peak, a.settle, a.min_link_gap, a.vmin, 100 * a.sat_frac, b.peak,
                    b.settle, b.min_link_gap, b.vmin, 100 * b.sat_frac);
      }
  }
}

} // namespace

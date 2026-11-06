/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// dyn_contact.h: compliant contact for the dynamic single-track plants (dyn_bike.h's dyn_plant and
// dyn_spin_plant), against a field (the scene's signed distance) and between vehicles, in
// port-Hamiltonian form. In SI, double: it is the live plant's (dyn_bike_plant), not the trainer's.
//
// The footprint. A vehicle's outline is a rectangle about its centre of gravity, `front` ahead,
// `rear` behind and `half_width` to each side (m), sampled at points l_k on its outline:
// per_side points along each long side and per_end along each end, the corners shared
// (dyn_footprint_points).
//
// Against a field. phi(P) is the field's signed distance at a world point (> 0 free, m) and
// grad phi its gradient (dyn_contact_field's sampler gives both). Each point k, at
// P_k = c + R(psi) l_k with d_k = phi(P_k), stores
//   psi_k = 1/2 k (s - d_k)^2 while d_k < s (0 beyond),      s the skin,
// damps its normal velocity with c_k = c (s - d_k) / s (Hunt-Crossley's: no damping at first touch,
// so a point leaving is never held back), and slides along the obstacle with Coulomb friction on
// the barrier's push, its direction smoothed: f_k = mu k (s - d_k) / sqrt(v_t^2 + slide^2), v_t
// the point's velocity along t = (-d phi/dy, d phi/dx). Its world force is
//   F_k = k (s - d_k) grad phi - c_k (grad phi . v_Pk) grad phi - f_k (t . v_Pk) t,
// and its body wrench (body axes, at the centre of gravity) is the external-port map's
// G_e,k^T F_k = (R^T F_k, l_k x R^T F_k) (the vehicle-pH paper's "Externally defined ports").
//
// In pH form. With the pose g = (c, psi) in the state, x = (g, p) on T*SE(2):
//   H = 1/2 v^T M v + Phi(g),   Phi = sum_k psi_k
//   x' = [[0, B], [-B^T, J(p)]] dH/dx - blockdiag(0, R + R_c) dH/dx + G u + ...
//   R_c = sum_k (c_k b_k b_k^T + f_k a_k a_k^T),  b_k = G_e,k^T grad phi_k,  a_k = G_e,k^T t_k
// with B the twist's map to the pose's rates. The barrier's force -B^T dPhi/dg is
// sum_k G_e,k^T k (s - d_k) grad phi_k (dyn_contact::Wc): it lives in H, through the canonical
// structure; the damper's and the friction's are -R_c v (dyn_contact::Wd), R_c positive
// semi-definite (the friction's coefficient is its secant through zero sliding), its power
// -v^T R_c v (dyn_contact::damping, its negative).
//
// The plant takes the contact's wrench as an external port (dyn_plant's `wrench`), held over the
// step, and books its power y_e^T u_e = v^T W (dyn_plant_books' port). The contact's books split
// that port into the barrier's change and the damper's work:
//   port = - (Phi(g1) - Phi(g0)) - damper's work + contact residual.
// Held from the step's start while the plant moves the pose with the step's new twist (symplectic
// Euler), the barrier's work differs from its change by second order per step; the damper's line is
// its held force's own work. So the books still close to second order.
//
// The damper and the friction, implicit (dyn_contact_implicit, the live plant's). R_c grows with
// the penetration, and the friction's secant through zero sliding is mu k (s - d) / slide, so in
// a fast impact hdt R_c / m can pass 2 at the live plant's 0.01 s substep (a Humvee into
// sim_world's grid field at 7.8 m/s, below). Held from the step's start, -R_c v0 then reverses the
// approach within a step and does work on the body (the books' damper line goes negative, the
// speed clamp takes the rest). So the plant reads them at the twist the step reaches under the
// bodies' inertia, the barrier's push and themselves, implicit Euler with the barrier explicit:
//   (M + hdt R_c) v_d = M v0 + hdt W_c,   W_d = -R_c v_d,
// M = diag(m, m, Iz) on the twist, R_c over each touching point's normal and tangent
// (dyn_contact_row): the dampers' c_k at the step's start, the friction's secants at the slide v_d
// reaches (a few solves). Coasting into an obstacle, the step ends at v_d itself. hdt R_c M^-1 may
// be any size: the approach is taken out in a step, never reversed past what the barrier gives
// back, and a slide is stopped, never driven back. Between vehicles R_c couples the pair, so the
// vehicles touching each other are solved together, and W_d stays equal and opposite at each
// point. The books are unchanged: W_d is held over the step, its line its own work.
//
// The barrier stays explicit, and its error grows with how far a step carries a point into it: a
// point that enters the skin within a step meets no force at the step's start, and its energy at
// the end is the residual's. So the live plant cuts a substep in which any point would close more
// than contact_step_depth on what it meets (dyn_contact_closing, dyn_contact_pair_closing) into
// pieces. A steep field closes faster: nav's grid SDF has |grad phi| = 2 in the cell across an
// obstacle's face (its distances run between cell centres), so there a barrier is 4x and a damper
// 8x as stiff per metre of approach as against a wall's exact distance.
//
// Between vehicles (dyn_contact_pair). Each vehicle's points meet the other's outline as a rounded
// rectangle's signed distance (corner radius `rounding`; its gradient is continuous for
// penetrations under it), both ways. Each point-outline force acts on both bodies, equal and
// opposite at the point, so the pair keeps its momentum; the barrier's energy depends only on
// their relative pose and is shared, half to each vehicle's books, as is the damper's power, which
// reads the points' relative normal velocity.

#ifndef __CVC_NAV_DETAIL_DYN_CONTACT_H__
#define __CVC_NAV_DETAIL_DYN_CONTACT_H__

#include <algorithm>
#include <cmath>
#include <vector>

namespace cvc {
namespace nav {
namespace diff {

// The defaults: a 2.7 t vehicle meeting a wall with an end's four points has a barrier frequency
// of ~21 rad/s (resolved at the live plant's 0.01 s substep: ~0.2 m past the skin at 5 m/s), and a
// damper that takes most of the approach (Hunt-Crossley's growth with the penetration, so nothing
// pulls at separation); the friction is a body panel's against masonry or another body, as
// dyn_bike_plant_options::chassis_friction's (0.3) against Jolt's default material.
struct dyn_contact_params {
  double k = 3.0e5;      // N/m: each point's barrier stiffness
  double c = 1.5e4;      // N s/m: each point's damping, at the full skin's depth
  double friction = 0.2; // the outline's friction along what it meets
  double slide = 0.05;   // m/s: the friction's smoothing of its sliding direction
  double skin = 0.05;    // m: the barrier's reach outside the outline
  int per_side = 7;      // points along each long side, the corners included
  int per_end = 4;       // along each end, the corners included
  double rounding = 0.3; // m: the outline's corner radius, as another vehicle meets it
};

// The outline about the centre of gravity (m).
struct dyn_footprint {
  double front = 2.0, rear = 2.0, half_width = 1.0;
};

struct dyn_point {
  double x = 0.0, y = 0.0;
};

// The footprint's points l_k, body axes, from the centre of gravity.
inline std::vector<dyn_point> dyn_footprint_points(const dyn_footprint &f,
                                                   const dyn_contact_params &p) {
  std::vector<dyn_point> pts;
  const int ns = std::max(2, p.per_side), ne = std::max(2, p.per_end);
  for (int i = 0; i < ns; ++i) {
    const double x = -f.rear + (f.front + f.rear) * i / (ns - 1);
    pts.push_back({x, f.half_width});
    pts.push_back({x, -f.half_width});
  }
  for (int j = 1; j < ne - 1; ++j) {
    const double y = -f.half_width + 2.0 * f.half_width * j / (ne - 1);
    pts.push_back({f.front, y});
    pts.push_back({-f.rear, y});
  }
  return pts;
}

// A body: its centre of gravity's world position and its heading, and its twist there (body axes).
struct dyn_body {
  double x = 0.0, y = 0.0, psi = 0.0, vx = 0.0, vy = 0.0, r = 0.0;
};

// The contact on one body: the barrier's and the damper's wrenches (body axes, at the centre of
// gravity: F_x, F_y, M_z), the barrier's energy (J), the damper's dissipation (W, >= 0) at the
// twist it read, and how many points are within the skin.
struct dyn_contact {
  double Wc[3] = {0.0, 0.0, 0.0}, Wd[3] = {0.0, 0.0, 0.0};
  double energy = 0.0, damping = 0.0;
  int touching = 0;
  // The total wrench, the plant's port input.
  void wrench(double W[3]) const {
    for (int i = 0; i < 3; ++i)
      W[i] = Wc[i] + Wd[i];
  }
};

// One damped direction of the contact (a touching point's normal, for its damper, or its tangent,
// for its friction): its coefficient (c_k or f_k, N s/m) and what a unit world force along it at
// the point is on each body it touches, as a wrench (body axes, at the centre of gravity): ja on
// body a, jb on body b (a pair's other vehicle, the force's opposite; b < 0 against a field). The
// direction's velocity is ja . v_a + jb . v_b, its force -coef (ja . v_a + jb . v_b) (ja, jb), so
// R_c = sum coef j j^T over the rows. a and b are the caller's indices (dyn_contact_implicit's). A
// friction row also has its Coulomb force, mu k (s - d) (coulomb, N; 0 on a damper's), and its
// smoothing, so its secant can be taken at another slide: coulomb / sqrt(v_t^2 + slide^2).
struct dyn_contact_row {
  int a = 0, b = -1;
  double coef = 0.0;
  double ja[3] = {0.0, 0.0, 0.0}, jb[3] = {0.0, 0.0, 0.0};
  double coulomb = 0.0, slide = 0.0;
};

// Adds a world force (fx, fy) at the world offset (rx, ry) from b's centre of gravity to W as a
// body wrench.
inline void dyn_contact_add(const dyn_body &b, double rx, double ry, double fx, double fy,
                            double W[3]) {
  const double c = std::cos(b.psi), s = std::sin(b.psi);
  W[0] += c * fx + s * fy;
  W[1] += -s * fx + c * fy;
  W[2] += rx * fy - ry * fx;
}

// A body point's world velocity, at world offset (rx, ry) from its centre of gravity.
inline void dyn_contact_velocity(const dyn_body &b, double rx, double ry, double &vx, double &vy) {
  const double c = std::cos(b.psi), s = std::sin(b.psi);
  vx = c * b.vx - s * b.vy - b.r * ry;
  vy = s * b.vx + c * b.vy + b.r * rx;
}

// A row's direction on body b: the wrench of a unit world force (fx, fy) at world offset (rx, ry).
inline void dyn_contact_direction(const dyn_body &b, double rx, double ry, double fx, double fy,
                                  double j[3]) {
  j[0] = j[1] = j[2] = 0.0;
  dyn_contact_add(b, rx, ry, fx, fy, j);
}

// The contact of body b's points against a field. The sampler F has
//   void at(double x, double y, double &phi, double &gx, double &gy) const
// giving the signed distance (m, > 0 free) at a world point and its gradient. Adds to out, and,
// with `rows`, each touching point's damped directions (dyn_contact_row, on body `index`).
template <class F>
inline void dyn_contact_field(const dyn_contact_params &p, const std::vector<dyn_point> &pts,
                              const F &field, const dyn_body &b, dyn_contact &out,
                              std::vector<dyn_contact_row> *rows = nullptr, int index = 0) {
  const double c = std::cos(b.psi), s = std::sin(b.psi);
  for (const dyn_point &l : pts) {
    const double rx = c * l.x - s * l.y, ry = s * l.x + c * l.y;
    double d, gx, gy;
    field.at(b.x + rx, b.y + ry, d, gx, gy);
    if (!(d < p.skin))
      continue;
    ++out.touching;
    const double pen = p.skin - d, fn = p.k * pen, cd = p.c * pen / p.skin;
    double vpx, vpy;
    dyn_contact_velocity(b, rx, ry, vpx, vpy);
    const double vn = gx * vpx + gy * vpy;
    // Friction along the obstacle: Coulomb on the barrier's push, its sliding direction smoothed
    // over `slide` (a secant coefficient, so a dissipation in R too).
    const double tx = -gy, ty = gx, vt = tx * vpx + ty * vpy;
    const double ft = p.friction * fn / std::sqrt(vt * vt + p.slide * p.slide);
    out.energy += 0.5 * p.k * pen * pen;
    out.damping += cd * vn * vn + ft * vt * vt;
    dyn_contact_add(b, rx, ry, fn * gx, fn * gy, out.Wc);
    dyn_contact_add(b, rx, ry, -cd * vn * gx - ft * vt * tx, -cd * vn * gy - ft * vt * ty, out.Wd);
    if (rows) {
      dyn_contact_row n, t;
      n.a = t.a = index;
      n.coef = cd;
      t.coef = ft;
      t.coulomb = p.friction * fn;
      t.slide = p.slide;
      dyn_contact_direction(b, rx, ry, gx, gy, n.ja);
      dyn_contact_direction(b, rx, ry, tx, ty, t.ja);
      rows->push_back(n);
      rows->push_back(t);
    }
  }
}

// A rounded rectangle's signed distance at (lx, ly) in its body's axes (from the centre of gravity:
// the footprint's outline with corners of radius rho, held to the half width and half length), and
// its gradient there (gx, gy, body axes).
inline double dyn_rounded_box_sdf(const dyn_footprint &f, double rho, double lx, double ly,
                                  double &gx, double &gy) {
  rho = std::max(0.0, std::min(rho, std::min(f.half_width, 0.5 * (f.front + f.rear))));
  const double cx0 = 0.5 * (f.front - f.rear);
  const double a = 0.5 * (f.front + f.rear) - rho, bb = f.half_width - rho;
  const double qx0 = lx - cx0, qy0 = ly;
  const double sx = qx0 >= 0.0 ? 1.0 : -1.0, sy = qy0 >= 0.0 ? 1.0 : -1.0;
  const double qx = std::fabs(qx0) - a, qy = std::fabs(qy0) - bb;
  if (qx > 0.0 && qy > 0.0) {
    const double len = std::sqrt(qx * qx + qy * qy);
    gx = sx * qx / len;
    gy = sy * qy / len;
    return len - rho;
  }
  if (qx > qy) {
    gx = sx;
    gy = 0.0;
    return qx - rho;
  }
  gx = 0.0;
  gy = sy;
  return qy - rho;
}

// One way of a pair: X's points against Y's rounded outline. Each point's force acts on X at the
// point and its opposite on Y; the energy and the dissipation go half to each. With `rows`, each
// touching point's damped directions, on bodies ix (X) and iy (Y).
inline void dyn_contact_one_way(const dyn_contact_params &p, const std::vector<dyn_point> &ptsX,
                                const dyn_body &X, const dyn_footprint &fy, const dyn_body &Y,
                                dyn_contact &cx, dyn_contact &cy,
                                std::vector<dyn_contact_row> *rows = nullptr, int ix = 0,
                                int iy = 1) {
  const double c = std::cos(X.psi), s = std::sin(X.psi);
  const double cY = std::cos(Y.psi), sY = std::sin(Y.psi);
  for (const dyn_point &l : ptsX) {
    const double rx = c * l.x - s * l.y, ry = s * l.x + c * l.y;
    const double px = X.x + rx, py = X.y + ry;
    // The point in Y's axes, and Y's outline's distance and gradient there (world).
    const double ox = px - Y.x, oy = py - Y.y;
    double glx, gly;
    const double d =
        dyn_rounded_box_sdf(fy, p.rounding, cY * ox + sY * oy, -sY * ox + cY * oy, glx, gly);
    if (!(d < p.skin))
      continue;
    const double gx = cY * glx - sY * gly, gy = sY * glx + cY * gly;
    ++cx.touching;
    ++cy.touching;
    const double pen = p.skin - d, fn = p.k * pen, cd = p.c * pen / p.skin;
    double vxx, vxy, vyx, vyy;
    dyn_contact_velocity(X, rx, ry, vxx, vxy);
    dyn_contact_velocity(Y, ox, oy, vyx, vyy);
    const double vn = gx * (vxx - vyx) + gy * (vxy - vyy);
    const double tx = -gy, ty = gx, vt = tx * (vxx - vyx) + ty * (vxy - vyy);
    const double ft = p.friction * fn / std::sqrt(vt * vt + p.slide * p.slide);
    const double e = 0.5 * p.k * pen * pen, w = cd * vn * vn + ft * vt * vt;
    cx.energy += 0.5 * e;
    cy.energy += 0.5 * e;
    cx.damping += 0.5 * w;
    cy.damping += 0.5 * w;
    const double dx = -cd * vn * gx - ft * vt * tx, dy = -cd * vn * gy - ft * vt * ty;
    dyn_contact_add(X, rx, ry, fn * gx, fn * gy, cx.Wc);
    dyn_contact_add(Y, ox, oy, -fn * gx, -fn * gy, cy.Wc);
    dyn_contact_add(X, rx, ry, dx, dy, cx.Wd);
    dyn_contact_add(Y, ox, oy, -dx, -dy, cy.Wd);
    if (rows) {
      dyn_contact_row n, t;
      n.a = t.a = ix;
      n.b = t.b = iy;
      n.coef = cd;
      t.coef = ft;
      t.coulomb = p.friction * fn;
      t.slide = p.slide;
      dyn_contact_direction(X, rx, ry, gx, gy, n.ja);
      dyn_contact_direction(Y, ox, oy, -gx, -gy, n.jb);
      dyn_contact_direction(X, rx, ry, tx, ty, t.ja);
      dyn_contact_direction(Y, ox, oy, -tx, -ty, t.jb);
      rows->push_back(n);
      rows->push_back(t);
    }
  }
}

// The contact between two vehicles, both ways; adds to ca (on a) and cb (on b), and, with `rows`,
// the damped directions on bodies ia (a) and ib (b).
inline void dyn_contact_pair(const dyn_contact_params &p, const std::vector<dyn_point> &pa,
                             const dyn_footprint &fa, const dyn_body &a,
                             const std::vector<dyn_point> &pb, const dyn_footprint &fb,
                             const dyn_body &b, dyn_contact &ca, dyn_contact &cb,
                             std::vector<dyn_contact_row> *rows = nullptr, int ia = 0, int ib = 1) {
  dyn_contact_one_way(p, pa, a, fb, b, ca, cb, rows, ia, ib);
  dyn_contact_one_way(p, pb, b, fa, a, cb, ca, rows, ib, ia);
}

// How fast body b's points close on a field (m/s of its distance, -grad phi . v_P: the field's
// gradient as it is, so a steeper field closes faster), the fastest of those within the skin or
// reaching it in hdt at that speed; 0 if none. The live plant's measure of how deep a step carries
// a point into the barrier (dyn_bike_plant_options::contact_step_depth).
template <class F>
inline double dyn_contact_closing(const dyn_contact_params &p, const std::vector<dyn_point> &pts,
                                  const F &field, const dyn_body &b, double hdt) {
  const double c = std::cos(b.psi), s = std::sin(b.psi);
  double fastest = 0.0;
  for (const dyn_point &l : pts) {
    const double rx = c * l.x - s * l.y, ry = s * l.x + c * l.y;
    double d, gx, gy, vpx, vpy;
    field.at(b.x + rx, b.y + ry, d, gx, gy);
    dyn_contact_velocity(b, rx, ry, vpx, vpy);
    const double closing = -(gx * vpx + gy * vpy);
    if (closing > fastest && d - hdt * closing < p.skin)
      fastest = closing;
  }
  return fastest;
}

// The same between two vehicles, both ways: the fastest any point of either closes on the other's
// rounded outline, at their relative velocity.
inline double dyn_contact_pair_closing(const dyn_contact_params &p,
                                       const std::vector<dyn_point> &pa, const dyn_footprint &fa,
                                       const dyn_body &a, const std::vector<dyn_point> &pb,
                                       const dyn_footprint &fb, const dyn_body &b, double hdt) {
  double fastest = 0.0;
  const auto one_way = [&](const std::vector<dyn_point> &ptsX, const dyn_body &X,
                           const dyn_footprint &fy, const dyn_body &Y) {
    const double c = std::cos(X.psi), s = std::sin(X.psi);
    const double cY = std::cos(Y.psi), sY = std::sin(Y.psi);
    for (const dyn_point &l : ptsX) {
      const double rx = c * l.x - s * l.y, ry = s * l.x + c * l.y;
      const double ox = X.x + rx - Y.x, oy = X.y + ry - Y.y;
      double glx, gly, vxx, vxy, vyx, vyy;
      const double d =
          dyn_rounded_box_sdf(fy, p.rounding, cY * ox + sY * oy, -sY * ox + cY * oy, glx, gly);
      const double gx = cY * glx - sY * gly, gy = sY * glx + cY * gly;
      dyn_contact_velocity(X, rx, ry, vxx, vxy);
      dyn_contact_velocity(Y, ox, oy, vyx, vyy);
      const double closing = -(gx * (vxx - vyx) + gy * (vxy - vyy));
      if (closing > fastest && d - hdt * closing < p.skin)
        fastest = closing;
    }
  };
  one_way(pa, a, fb, b);
  one_way(pb, b, fa, a);
  return fastest;
}

// A body in dyn_contact_implicit's solve: its inertia on the twist (v_x, v_y, r), diag(m, m, Iz);
// its twist at the step's start, v; the barriers' wrench on it, Wc, held over the step; and, out,
// the dampers' and the friction's wrench Wd (body axes, at the centre of gravity) and the twist vd
// they read it at (v where nothing touches it).
struct dyn_contact_inertia {
  double m = 1.0, Iz = 1.0;
  double v[3] = {0.0, 0.0, 0.0}, Wc[3] = {0.0, 0.0, 0.0};
  double Wd[3] = {0.0, 0.0, 0.0}, vd[3] = {0.0, 0.0, 0.0};
};

// Solves A x = b in place (x into b) for A symmetric positive definite, n x n row-major: Cholesky.
inline void dyn_contact_spd_solve(std::vector<double> &A, std::vector<double> &b, int n) {
  for (int j = 0; j < n; ++j) {
    double d = A[j * n + j];
    for (int k = 0; k < j; ++k)
      d -= A[j * n + k] * A[j * n + k];
    d = std::sqrt(d);
    A[j * n + j] = d;
    for (int i = j + 1; i < n; ++i) {
      double s = A[i * n + j];
      for (int k = 0; k < j; ++k)
        s -= A[i * n + k] * A[j * n + k];
      A[i * n + j] = s / d;
    }
  }
  for (int i = 0; i < n; ++i) {
    double s = b[i];
    for (int k = 0; k < i; ++k)
      s -= A[i * n + k] * b[k];
    b[i] = s / A[i * n + i];
  }
  for (int i = n - 1; i >= 0; --i) {
    double s = b[i];
    for (int k = i + 1; k < n; ++k)
      s -= A[k * n + i] * b[k];
    b[i] = s / A[i * n + i];
  }
}

// The dampers' and the friction's wrench over a step of hdt, implicit (see the header comment):
// (M + hdt R_c) v_d = M v + hdt W_c over the bodies, R_c the rows', and W_d = -R_c v_d. The bodies
// a pair's rows join are solved together (one system per group touching each other, 3 unknowns a
// body), so each row's force stays equal and opposite at its point. The friction's secant is taken
// at the slide it reaches, `iters` solves in all, each from the last's v_d (the rows' coef is the
// start's): read at the start's slide, a secant leaves a slide the step slows short of Coulomb's
// mu k (s - d); taken again it is a majorize-minimize step on the friction's dissipation
// potential, so the slide falls toward the implicit Euler step's at each. Fills each body's Wd and
// vd.
inline void dyn_contact_implicit(double hdt, const std::vector<dyn_contact_row> &rows,
                                 std::vector<dyn_contact_inertia> &bodies, int iters = 4) {
  iters = std::max(1, iters);
  const int n = static_cast<int>(bodies.size());
  for (dyn_contact_inertia &q : bodies)
    for (int i = 0; i < 3; ++i) {
      q.Wd[i] = 0.0;
      q.vd[i] = q.v[i];
    }
  // The groups: bodies joined by a pair's rows.
  std::vector<int> root(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    root[i] = i;
  const auto find = [&](int i) {
    while (root[i] != i) {
      root[i] = root[root[i]];
      i = root[i];
    }
    return i;
  };
  for (const dyn_contact_row &w : rows)
    if (w.b >= 0)
      root[find(w.a)] = find(w.b);
  std::vector<int> group(static_cast<std::size_t>(n), -1), slot(static_cast<std::size_t>(n), 0);
  std::vector<std::vector<int>> members;
  std::vector<std::vector<const dyn_contact_row *>> in;
  for (const dyn_contact_row &w : rows) {
    const int g0 = find(w.a);
    if (group[g0] < 0) {
      group[g0] = static_cast<int>(members.size());
      members.emplace_back();
      in.emplace_back();
    }
    in[group[g0]].push_back(&w);
  }
  for (int i = 0; i < n; ++i) {
    const int g0 = group[find(i)];
    if (g0 < 0)
      continue;
    slot[i] = static_cast<int>(members[g0].size());
    members[g0].push_back(i);
  }
  // A row's velocity at the bodies' vd.
  const auto velocity = [&](const dyn_contact_row &w) {
    const dyn_contact_inertia &qa = bodies[w.a];
    double vel = w.ja[0] * qa.vd[0] + w.ja[1] * qa.vd[1] + w.ja[2] * qa.vd[2];
    if (w.b >= 0) {
      const dyn_contact_inertia &qb = bodies[w.b];
      vel += w.jb[0] * qb.vd[0] + w.jb[1] * qb.vd[1] + w.jb[2] * qb.vd[2];
    }
    return vel;
  };
  std::vector<double> A, x, coef;
  for (std::size_t g0 = 0; g0 < members.size(); ++g0) {
    const int nb = static_cast<int>(members[g0].size()), N = 3 * nb;
    const std::vector<const dyn_contact_row *> &ws = in[g0];
    coef.resize(ws.size());
    for (std::size_t r = 0; r < ws.size(); ++r)
      coef[r] = ws[r]->coef;
    for (int it = 0; it < iters; ++it) {
      A.assign(static_cast<std::size_t>(N) * N, 0.0);
      x.assign(static_cast<std::size_t>(N), 0.0);
      for (int k = 0; k < nb; ++k) {
        const dyn_contact_inertia &q = bodies[members[g0][k]];
        const double M[3] = {q.m, q.m, q.Iz};
        for (int i = 0; i < 3; ++i) {
          A[(3 * k + i) * N + 3 * k + i] = M[i];
          x[3 * k + i] = M[i] * q.v[i] + hdt * q.Wc[i];
        }
      }
      for (std::size_t r = 0; r < ws.size(); ++r) {
        // The row's direction over the group's unknowns: ja on a's three, jb on b's.
        const dyn_contact_row &w = *ws[r];
        const int oa = 3 * slot[w.a], ob = w.b >= 0 ? 3 * slot[w.b] : -1;
        const double hc = hdt * coef[r];
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) {
            A[(oa + i) * N + oa + j] += hc * w.ja[i] * w.ja[j];
            if (ob >= 0) {
              A[(oa + i) * N + ob + j] += hc * w.ja[i] * w.jb[j];
              A[(ob + i) * N + oa + j] += hc * w.jb[i] * w.ja[j];
              A[(ob + i) * N + ob + j] += hc * w.jb[i] * w.jb[j];
            }
          }
      }
      dyn_contact_spd_solve(A, x, N);
      for (int k = 0; k < nb; ++k)
        for (int i = 0; i < 3; ++i)
          bodies[members[g0][k]].vd[i] = x[3 * k + i];
      // The friction's secants again, at the slide v_d reached, but for the last solve's.
      if (it + 1 < iters)
        for (std::size_t r = 0; r < ws.size(); ++r)
          if (ws[r]->coulomb > 0.0) {
            const double vt = velocity(*ws[r]);
            coef[r] = ws[r]->coulomb / std::sqrt(vt * vt + ws[r]->slide * ws[r]->slide);
          }
    }
    for (std::size_t r = 0; r < ws.size(); ++r) {
      const dyn_contact_row &w = *ws[r];
      const double f = -coef[r] * velocity(w);
      dyn_contact_inertia &qa = bodies[w.a];
      for (int i = 0; i < 3; ++i)
        qa.Wd[i] += f * w.ja[i];
      if (w.b >= 0) {
        dyn_contact_inertia &qb = bodies[w.b];
        for (int i = 0; i < 3; ++i)
          qb.Wd[i] += f * w.jb[i];
      }
    }
  }
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_DYN_CONTACT_H__

/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cmath>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/symbol.h>
#include <vector>

namespace cvc {
namespace lsys {

namespace {

constexpr double kDeg2Rad = 0.017453292519943295769;

vec3 add(const vec3 &a, const vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
vec3 scl(const vec3 &a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const vec3 &a, const vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
vec3 norm(const vec3 &a) {
  double n = std::sqrt(dot(a, a));
  return n > 1e-12 ? scl(a, 1.0 / n) : vec3{0, 0, 1};
}

// Rodrigues rotation of v about unit axis k by angle (radians).
vec3 rot(const vec3 &v, const vec3 &k, double ang) {
  const double c = std::cos(ang), s = std::sin(ang);
  return add(add(scl(v, c), scl(cross(k, v), s)), scl(k, dot(k, v) * (1.0 - c)));
}

struct state {
  vec3 pos{0, 0, 0};
  vec3 H{0, 0, 1}; // heading (growth / scope z)
  vec3 L{1, 0, 0}; // left (scope x)
  vec3 U{0, 1, 0}; // up (scope y)
  double width = 0.1;
  role rl = role::trunk;
  vec3 size{1, 1, 1};
};

struct walker {
  const ruleset &rs;
  const interp_options &opt;
  structure out;
  std::vector<state> stack;
  state st;
  bool have_bounds = false;

  walker(const ruleset &r, const interp_options &o) : rs(r), opt(o) {}

  void grow_bounds(const vec3 &p) {
    if (!have_bounds) {
      out.lo = out.hi = p;
      have_bounds = true;
      return;
    }
    out.lo.x = std::min(out.lo.x, p.x);
    out.lo.y = std::min(out.lo.y, p.y);
    out.lo.z = std::min(out.lo.z, p.z);
    out.hi.x = std::max(out.hi.x, p.x);
    out.hi.y = std::max(out.hi.y, p.y);
    out.hi.z = std::max(out.hi.z, p.z);
  }

  double param(const module_t &m, int i, double dflt) const {
    return i < m.nparams ? m.p[i] : dflt;
  }

  void yaw(double deg) {
    double a = deg * kDeg2Rad;
    st.H = norm(rot(st.H, st.U, a));
    st.L = norm(rot(st.L, st.U, a));
  }
  void pitch(double deg) {
    double a = deg * kDeg2Rad;
    st.H = norm(rot(st.H, st.L, a));
    st.U = norm(rot(st.U, st.L, a));
  }
  void roll_frame(double deg) {
    double a = deg * kDeg2Rad;
    st.L = norm(rot(st.L, st.H, a));
    st.U = norm(rot(st.U, st.H, a));
  }
  void relevel() {
    const vec3 up{0, 0, 1};
    vec3 l = cross(up, st.H);
    if (dot(l, l) < 1e-9)
      return; // heading is vertical; nothing to do
    st.L = norm(l);
    st.U = norm(cross(st.H, st.L));
  }

  void forward(const module_t &m, bool draw) {
    double len = param(m, 0, rs.step);
    vec3 a = st.pos;
    vec3 b = add(st.pos, scl(st.H, len));
    if (draw) {
      double r0 = st.width;
      st.width *= (1.0 - rs.taper);
      double r1 = st.width;
      out.segments.push_back({a, b, r0, r1, st.rl, m.level});
      grow_bounds(a);
      grow_bounds(b);
    }
    st.pos = b;
  }

  void emit_leaf(const module_t &m) {
    double sz = param(m, 0, opt.leaf_size);
    out.leaves.push_back({st.pos, st.H, sz, opt.default_leaf, m.level});
    // A leaf's canopy extends ~sz around pos; grow bounds by that.
    grow_bounds(add(st.pos, scl(st.H, sz)));
    grow_bounds(add(st.pos, {sz, sz, 0}));
    grow_bounds(add(st.pos, {-sz, -sz, 0}));
  }

  void emit_box(const module_t &m) {
    role rl = m.nparams > 0 ? static_cast<role>(static_cast<int>(m.p[0])) : st.rl;
    vec3 c = add(st.pos, scl(st.H, st.size.z * 0.5)); // base at pos, extends up along H
    obox b;
    b.center = c;
    b.axis[0] = st.L;
    b.axis[1] = st.U;
    b.axis[2] = st.H;
    b.half = {st.size.x * 0.5, st.size.y * 0.5, st.size.z * 0.5};
    b.rl = rl;
    b.level = m.level;
    out.boxes.push_back(b);
    // Bounds: the 8 corners.
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          vec3 corner = c;
          corner = add(corner, scl(b.axis[0], sx * b.half.x));
          corner = add(corner, scl(b.axis[1], sy * b.half.y));
          corner = add(corner, scl(b.axis[2], sz * b.half.z));
          grow_bounds(corner);
        }
  }

  void emit_paint(const module_t &m, paint2d::shape sh) {
    paint2d p;
    p.sh = sh;
    p.rl = m.nparams > 0 ? static_cast<role>(static_cast<int>(m.p[0])) : role::ground;
    p.a = st.pos;
    if (sh == paint2d::disc) {
      p.radius = param(m, 1, 1.0);
      p.b = st.pos;
    } else {
      p.radius = param(m, 1, 0.5);
      p.b = add(st.pos, scl(st.H, param(m, 2, 1.0)));
    }
    p.feather = param(m, 3, 0.0);
    out.paints.push_back(p);
  }

  void run(const word &w) {
    st.width = rs.width;
    st.rl = (rs.kind == asset_kind::building || rs.kind == asset_kind::rock) ? opt.default_solid
                                                                             : opt.default_stem;
    st.size = opt.scope_size;
    grow_bounds(st.pos);

    for (std::size_t i = 0; i < w.size(); ++i) {
      const module_t &m = w[i];
      if (!symbol_table::is_builtin(m.sym))
        continue; // leftover nonterminal: no geometry
      switch (static_cast<builtin>(m.sym)) {
      case builtin::fwd:
      case builtin::fwd_g:
        forward(m, true);
        break;
      case builtin::move:
        forward(m, false);
        break;
      case builtin::yaw_pos:
        yaw(param(m, 0, rs.angle_deg));
        break;
      case builtin::yaw_neg:
        yaw(-param(m, 0, rs.angle_deg));
        break;
      case builtin::pitch_dn:
        pitch(param(m, 0, rs.angle_deg));
        break;
      case builtin::pitch_up:
        pitch(-param(m, 0, rs.angle_deg));
        break;
      case builtin::roll_pos:
        roll_frame(param(m, 0, rs.angle_deg));
        break;
      case builtin::roll_neg:
        roll_frame(-param(m, 0, rs.angle_deg));
        break;
      case builtin::turn_180:
        yaw(180.0);
        break;
      case builtin::relevel:
        relevel();
        break;
      case builtin::tilt:
        pitch(param(m, 0, rs.tilt_deg));
        break;
      case builtin::roll:
        roll_frame(param(m, 0, rs.roll_deg));
        break;
      case builtin::leaf:
        emit_leaf(m);
        break;
      case builtin::set_width:
        st.width = param(m, 0, st.width);
        break;
      case builtin::set_class:
        st.rl = static_cast<role>(static_cast<int>(param(m, 0, static_cast<double>(st.rl))));
        break;
      case builtin::push:
        stack.push_back(st);
        break;
      case builtin::pop:
        if (!stack.empty()) {
          st = stack.back();
          stack.pop_back();
        }
        break;
      case builtin::s_trans: {
        vec3 d = add(add(scl(st.L, param(m, 0, 0)), scl(st.U, param(m, 1, 0))),
                     scl(st.H, param(m, 2, 0)));
        st.pos = add(st.pos, d);
        break;
      }
      case builtin::s_scale:
        st.size = {st.size.x * param(m, 0, 1), st.size.y * param(m, 1, 1),
                   st.size.z * param(m, 2, 1)};
        break;
      case builtin::s_rot:
        // Euler degrees about the scope's own axes.
        if (double a = param(m, 0, 0)) {
          st.U = norm(rot(st.U, st.L, a * kDeg2Rad));
          st.H = norm(rot(st.H, st.L, a * kDeg2Rad));
        }
        if (double a = param(m, 1, 0)) {
          st.L = norm(rot(st.L, st.U, a * kDeg2Rad));
          st.H = norm(rot(st.H, st.U, a * kDeg2Rad));
        }
        if (double a = param(m, 2, 0)) {
          st.L = norm(rot(st.L, st.H, a * kDeg2Rad));
          st.U = norm(rot(st.U, st.H, a * kDeg2Rad));
        }
        break;
      case builtin::s_box:
        emit_box(m);
        break;
      case builtin::paint_disc:
        emit_paint(m, paint2d::disc);
        break;
      case builtin::paint_wide:
      case builtin::paint_band:
        emit_paint(m, paint2d::capsule);
        break;
      default:
        break; // cut/poly/inst/subdiv/repeat/stamp2d: no direct geometry in v1
      }
    }
    if (!have_bounds)
      out.lo = out.hi = vec3{0, 0, 0};
  }
};

} // namespace

structure interpret(const ruleset &rs, const word &w, const interp_options &opt) {
  walker wk(rs, opt);
  wk.run(w);
  return std::move(wk.out);
}

} // namespace lsys
} // namespace cvc

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// field_wake.cpp — a fluid's wake: the bodies' waves on a window of ewave cells, one state per body
// that makes waves, each body reading the others' (see field_wake.h).

#include "field_wake.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cvc/core/exception.h>
#include <limits>
#include <stdexcept>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

namespace {

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("field: wake: " + what);
}

bool finite(double v) { return std::isfinite(v); }

const field_wake &checked(const field_wake &p) {
  need(p.cells >= 8 && p.cells <= 128 && (p.cells & (p.cells - 1)) == 0,
       "cells must be a power of two in 8 .. 128, got " + std::to_string(p.cells));
  need(finite(p.cell) && p.cell >= 0.05 && p.cell <= 100.0, "cell must be in 0.05 .. 100 m");
  need(finite(p.depth) && p.depth <= 1.0e4, "depth must be finite and at most 10 km (<= 0: deep)");
  need(finite(p.period) && p.period > 0.0 && p.period <= 10.0, "period must be in (0, 10] s");
  need(finite(p.damping_k2) && p.damping_k2 >= 0.0 && finite(p.damping) && p.damping >= 0.0,
       "damping_k2 and damping must be finite and >= 0");
  need(p.sponge_cells >= 0.0 && p.sponge_cells <= 0.5 * p.cells,
       "sponge_cells must be in [0, cells / 2]");
  need(finite(p.sponge_rate) && p.sponge_rate >= 0.0, "sponge_rate must be finite and >= 0");
  need(p.samples >= 1 && p.samples <= 8, "samples must be in 1 .. 8");
  need(p.sources >= 1 && p.sources <= 16, "sources must be in 1 .. 16");
  return p;
}

ocean::ewave_params engine_params(const field_wake &p, double gravity) {
  ocean::ewave_params e;
  e.n = p.cells;
  e.dx = p.cell;
  e.depth = p.depth > 0.0 ? p.depth : std::numeric_limits<double>::infinity();
  e.gravity = gravity > 0.0 ? gravity : 9.80665;
  e.damping_k2 = p.damping_k2;
  e.damping = p.damping;
  e.sponge_cells = p.sponge_cells;
  e.sponge_rate = p.sponge_rate;
  e.walls = ocean::ewave_walls::zero; // no mask: open water, the exact step
  e.ghost_cells = 0;
  return e;
}

constexpr int kQ = 6;           // what a state gives a reader per cell: h, u, v, w, gx, gy
constexpr double kFaded = 1e-6; // m: waves below this everywhere in a state are let go (zeroed)
constexpr double kWakeHeight = 1e-3; // m: the others' waves wake a sleeping body this high
constexpr double kFar = 1e12;        // cells: a coordinate this far off is not placed
constexpr double kBoatWeight = 4.0;  // where the window goes, a boat counts as this many bodies

template <class T> void put(std::string &out, const T &v) {
  out.append(reinterpret_cast<const char *>(&v), sizeof(v));
}
void put_values(std::string &out, const std::vector<double> &v) {
  out.append(reinterpret_cast<const char *>(v.data()), v.size() * sizeof(double));
}
template <class T> void get(const std::string &in, std::size_t &pos, T &v) {
  need(pos <= in.size() && sizeof(v) <= in.size() - pos, "the saved wake is truncated");
  std::memcpy(&v, in.data() + pos, sizeof(v));
  pos += sizeof(v);
}
void get_values(const std::string &in, std::size_t &pos, std::vector<double> &v, std::size_t n) {
  need(pos <= in.size() && n * sizeof(double) <= in.size() - pos, "the saved wake is truncated");
  v.resize(n);
  std::memcpy(v.data(), in.data() + pos, n * sizeof(double));
  pos += n * sizeof(double);
}

bool quiet(const std::vector<double> &a, const std::vector<double> &b) {
  for (std::size_t c = 0; c < a.size(); ++c)
    if (a[c] != 0.0 || b[c] != 0.0)
      return false;
  return true;
}

// What a body floats in, to cut its columns against: the sea held at the step's time, and on it the
// others' waves, tot - own on the window's cells (bilinear between them; 0 outside).
class wake_top final : public fluid_surface {
public:
  wake_top(const fluid_surface *sea, const std::vector<double> &tot, const std::vector<double> *own,
           int n, double x0, double y0, double dx)
      : _sea(sea), _tot(tot), _own(own), _n(n), _x0(x0), _y0(y0), _dx(dx) {}
  double level(double x, double y) const override { return _sea->level(x, y); }
  double height(double x, double y, double t) const override {
    surface_memo memo;
    return _sea->height_with(x, y, t, memo) + others(x, y);
  }
  vec3 velocity(const vec3 &p, double t) const override { return _sea->velocity(p, t); }
  std::uint64_t digest() const override { return _sea->digest(); }

private:
  double at(std::size_t c) const { return _own ? _tot[c] - (*_own)[c] : _tot[c]; }
  double others(double x, double y) const {
    const double fx = (x - _x0) / _dx, fy = (y - _y0) / _dx;
    if (!(fx >= 0.0) || !(fy >= 0.0) || !(fx <= _n - 1.0) || !(fy <= _n - 1.0))
      return 0.0;
    const int i = std::min(int(fx), _n - 2), j = std::min(int(fy), _n - 2);
    const double tx = fx - i, ty = fy - j;
    const std::size_t un = std::size_t(_n), c = std::size_t(j) * un + std::size_t(i);
    return (at(c) * (1.0 - tx) + at(c + 1) * tx) * (1.0 - ty) +
           (at(c + un) * (1.0 - tx) + at(c + un + 1) * tx) * ty;
  }
  const fluid_surface *_sea;
  const std::vector<double> &_tot;
  const std::vector<double> *_own;
  int _n;
  double _x0, _y0, _dx;
};

} // namespace

fluid_wake::fluid_wake(const field_wake &p, double gravity)
    : _p(checked(p)), _n(p.cells), _dx(p.cell), _gravity(gravity),
      _engine(std::make_unique<ocean::ewave>(engine_params(p, gravity))) {
  _total.assign(std::size_t(_n) * std::size_t(_n) * kQ, 0.0);
}

void fluid_wake::set_gravity(double gravity) {
  if (gravity == _gravity)
    return;
  _engine = std::make_unique<ocean::ewave>(engine_params(_p, gravity));
  _gravity = gravity;
}

int fluid_wake::own(std::uint32_t id) const {
  const auto it = std::lower_bound(_ids.begin(), _ids.end(), id);
  return it != _ids.end() && *it == id ? int(it - _ids.begin()) : -1;
}

bool fluid_wake::cell_of(double v, std::int64_t &out) const {
  const double f = v / _dx;
  if (!(std::fabs(f) < kFar))
    return false;
  out = std::llround(f);
  return true;
}

void fluid_wake::derive(state &s) const {
  const int n = _n;
  const std::size_t un = std::size_t(n);
  const double inv = 1.0 / (2.0 * _dx);
  s.f.assign(un * un * kQ, 0.0);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::size_t c = std::size_t(j) * un + std::size_t(i);
      double *f = s.f.data() + c * kQ;
      f[0] = s.h[c];
      f[3] = s.w[c];
      // Central differences inside; the border cells (deep in the sponge) give none.
      if (i > 0 && i < n - 1) {
        f[1] = (s.phi[c + 1] - s.phi[c - 1]) * inv;
        f[4] = (s.h[c + 1] - s.h[c - 1]) * inv;
      }
      if (j > 0 && j < n - 1) {
        f[2] = (s.phi[c + un] - s.phi[c - un]) * inv;
        f[5] = (s.h[c + un] - s.h[c - un]) * inv;
      }
    }
}

void fluid_wake::sum() {
  std::fill(_total.begin(), _total.end(), 0.0);
  _ids.clear();
  _byIndex.clear();
  for (const auto &e : _states) { // id order
    _ids.push_back(e.first);
    _byIndex.push_back(&e.second);
    const std::vector<double> &f = e.second.f;
    for (std::size_t c = 0; c < _total.size(); ++c)
      _total[c] += f[c];
  }
}

void fluid_wake::shift(int di, int dj) {
  // Cell (i, j) takes what was at (i + di, j + dj); what enters is flat (and no body's columns).
  const int n = _n;
  const std::size_t un = std::size_t(n);
  std::vector<double> tmp(un * un);
  const auto move = [&](std::vector<double> &a) {
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const int si = i + di, sj = j + dj;
        tmp[std::size_t(j) * un + std::size_t(i)] = si >= 0 && si < n && sj >= 0 && sj < n
                                                        ? a[std::size_t(sj) * un + std::size_t(si)]
                                                        : 0.0;
      }
    a.swap(tmp);
  };
  for (auto &e : _states) {
    move(e.second.h);
    move(e.second.phi);
    move(e.second.w);
    move(e.second.zeta);
  }
  _ci += di;
  _cj += dj;
}

bool fluid_wake::begin_step(double fixed_dt, double &dt) {
  // Whole steps nearest the period, at least one (the ratio bounded: the period is at most 10 s).
  const double ratio = std::min(std::max(_p.period / fixed_dt, 1.0), 1.0e6);
  const std::uint64_t k = static_cast<std::uint64_t>(std::llround(ratio));
  const bool due = _count % k == 0;
  ++_count;
  dt = static_cast<double>(k) * fixed_dt;
  return due;
}

void fluid_wake::advance(std::vector<wake_body> &bodies, const vec3 &lo, const vec3 &hi,
                         const std::function<const fluid_surface *()> &sea, double t, double T,
                         std::vector<std::uint32_t> &wake_up) {
  const int n = _n;
  const std::size_t un = std::size_t(n), cells = un * un;

  // Nothing to do while every body sleeps and no wave is left: the surface is not read (as the
  // step without a wake reads none then). The sleepers' columns are taken again when they wake.
  bool awake = false;
  for (const wake_body &b : bodies)
    awake = awake || (b.active && b.over);
  bool still = true;
  for (const auto &e : _states)
    still = still && quiet(e.second.h, e.second.phi);
  if (!awake && still) {
    for (auto &e : _states)
      e.second.rest = true;
    return;
  }

  // Which bodies are in the water: over the field's box, reaching below its top there.
  const fluid_surface *surface = sea();
  for (wake_body &b : bodies) {
    b.in_water = false;
    if (!b.over)
      continue;
    surface_memo memo;
    const double top = std::min(hi.z, surface->height_with(b.position.x, b.position.y, t, memo));
    b.in_water = b.lo[2] < top && b.hi[2] > lo.z;
  }

  // The states move on by T, one after another through the engine, and give dh/dt over it; one
  // whose waves are all below kFaded is let go (zeroed: it steps no more until a source).
  for (auto &e : _states) {
    state &s = e.second;
    if (quiet(s.h, s.phi)) {
      std::fill(s.w.begin(), s.w.end(), 0.0);
      continue;
    }
    _engine->set_state(s.h, s.phi);
    _engine->step(T);
    const std::vector<double> &h2 = _engine->height();
    double largest = 0.0;
    for (std::size_t c = 0; c < cells; ++c) {
      s.w[c] = (h2[c] - s.h[c]) / T;
      largest = std::max(largest, std::max(std::fabs(h2[c]), std::fabs(_engine->phi()[c])));
    }
    s.h = h2;
    s.phi = _engine->phi();
    if (largest < kFaded) {
      std::fill(s.h.begin(), s.h.end(), 0.0);
      std::fill(s.phi.begin(), s.phi.end(), 0.0);
      std::fill(s.w.begin(), s.w.end(), 0.0);
    }
  }

  // The window's place: where it holds the most bodies in the water, a boat counting kBoatWeight.
  // Each body proposes the bodies within the window's inner square round it (the window less its
  // sponge); the best proposal wins over what the window holds now only when it holds more, and
  // the window then centres on what it holds (moving by whole cells once that is cells / 8 off).
  const double inner = std::max(0.5 * n - _p.sponge_cells, 0.25 * n) * _dx;
  const auto weight = [](const wake_body &b) { return b.hull ? kBoatWeight : 1.0; };
  // The bodies in the water within `inner` of (cx, cy): their weight and their box's middle.
  const auto hold = [&](double cx, double cy, double &mx, double &my) {
    double w = 0.0, x0 = 0.0, x1 = 0.0, y0 = 0.0, y1 = 0.0;
    for (const wake_body &b : bodies) {
      if (!b.in_water || !b.outline || std::fabs(b.position.x - cx) > inner ||
          std::fabs(b.position.y - cy) > inner)
        continue;
      if (w == 0.0) {
        x0 = x1 = b.position.x;
        y0 = y1 = b.position.y;
      } else {
        x0 = std::min(x0, b.position.x);
        x1 = std::max(x1, b.position.x);
        y0 = std::min(y0, b.position.y);
        y1 = std::max(y1, b.position.y);
      }
      w += weight(b);
    }
    mx = 0.5 * (x0 + x1);
    my = 0.5 * (y0 + y1);
    return w;
  };
  double best = 0.0, bx = 0.0, by = 0.0;
  for (const wake_body &b : bodies) { // id order: the first of equals wins
    if (!b.in_water || !b.outline)
      continue;
    double mx, my;
    const double w = hold(b.position.x, b.position.y, mx, my);
    if (w > best) {
      best = w;
      bx = mx;
      by = my;
    }
  }
  int si0 = 0, si1 = n, sj0 = 0, sj1 = n; // the cells that were in the window before a move
  if (best > 0.0) {
    double cx = bx, cy = by;
    if (_placed) {
      const double mid = 0.5 * double(n - 1);
      double hx, hy;
      const double now = hold((double(_ci) + mid) * _dx, (double(_cj) + mid) * _dx, hx, hy);
      if (now >= best) { // keep what it holds
        cx = hx;
        cy = hy;
      }
    }
    std::int64_t ti = 0, tj = 0;
    if (cell_of(cx, ti) && cell_of(cy, tj)) {
      ti -= n / 2;
      tj -= n / 2;
      if (!_placed) {
        _placed = true;
        _ci = ti;
        _cj = tj;
      } else if (std::llabs(ti - _ci) > n / 8 || std::llabs(tj - _cj) > n / 8) {
        if (std::llabs(ti - _ci) >= n || std::llabs(tj - _cj) >= n) { // nothing carries over
          for (auto &e : _states) {
            std::fill(e.second.h.begin(), e.second.h.end(), 0.0);
            std::fill(e.second.phi.begin(), e.second.phi.end(), 0.0);
            std::fill(e.second.w.begin(), e.second.w.end(), 0.0);
            std::fill(e.second.zeta.begin(), e.second.zeta.end(), 0.0);
          }
          _ci = ti;
          _cj = tj;
          si1 = sj1 = 0;
        } else {
          const int di = int(ti - _ci), dj = int(tj - _cj);
          shift(di, dj);
          si0 = std::max(0, -di);
          si1 = std::min(n, n - di);
          sj0 = std::max(0, -dj);
          sj1 = std::min(n, n - dj);
        }
      }
    }
  }
  if (!_placed)
    return;

  // The sources.
  const double x0 = double(_ci) * _dx, y0 = double(_cj) * _dx, half = 0.5 * _dx;
  const double wx0 = x0 - half, wx1 = x0 + (double(n) - 0.5) * _dx;
  const double wy0 = y0 - half, wy1 = y0 + (double(n) - 0.5) * _dx;
  const auto inside = [&](const wake_body &b) {
    return b.hi[0] >= wx0 && b.lo[0] <= wx1 && b.hi[1] >= wy0 && b.lo[1] <= wy1;
  };
  _outside = 0;
  for (const wake_body &b : bodies)
    _outside += b.in_water && b.outline && !inside(b) ? 1 : 0;
  immersion_grid g;
  g.nx = g.ny = n;
  g.dx = _dx;
  g.origin_x = x0;
  g.origin_y = y0;
  g.samples = _p.samples;
  // The others' waves now (after the step and the move), for each body to float in: the sum of the
  // states' heights, less its own.
  std::vector<double> tot(cells, 0.0);
  for (const auto &e : _states)
    for (std::size_t c = 0; c < cells; ++c)
      tot[c] += e.second.h[c];
  std::vector<double> zeta(cells), dh(cells), tmp(cells);
  const auto columns = [&](const wake_body &b, const std::vector<double> *own) {
    std::fill(zeta.begin(), zeta.end(), 0.0);
    const wake_top top(surface, tot, own, n, x0, y0, _dx);
    immersion_water water;
    water.surface = &top;
    water.time = t;
    water.lo = lo;
    water.hi = hi;
    return add_immersion(*b.outline, b.position, b.rotation, water, g, zeta);
  };
  const auto find = [&](std::uint32_t id) -> const wake_body * {
    const auto it = std::lower_bound(bodies.begin(), bodies.end(), id,
                                     [](const wake_body &b, std::uint32_t v) { return b.id < v; });
    return it != bodies.end() && it->id == id ? &*it : nullptr;
  };
  int sourcing = 0;
  for (auto &e : _states) {
    state &s = e.second;
    if (!s.source)
      continue;
    const wake_body *b = find(e.first);
    if (!b || !b->outline || !inside(*b)) {
      s.source = false; // gone from the world or the window: its waves fade from here
      continue;
    }
    ++sourcing;
    if (!b->active) { // asleep: at rest as the world sees it, so it makes no waves
      s.rest = true;
      continue;
    }
    columns(*b, &s.h);
    if (s.rest) { // awake again: its columns now are its last
      s.zeta = zeta;
      s.rest = false;
      continue;
    }
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const std::size_t c = std::size_t(j) * un + std::size_t(i);
        const bool kept = i >= si0 && i < si1 && j >= sj0 && j < sj1;
        dh[c] = kept ? -(zeta[c] - s.zeta[c]) : 0.0;
      }
    // [1 2 1] / 4 along x, then y: the volume kept, edges under two cells taken out.
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const std::size_t c = std::size_t(j) * un + std::size_t(i);
        tmp[c] = 0.5 * dh[c] + 0.25 * ((i > 0 ? dh[c - 1] : 0.0) + (i < n - 1 ? dh[c + 1] : 0.0));
      }
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const std::size_t c = std::size_t(j) * un + std::size_t(i);
        s.h[c] +=
            0.5 * tmp[c] + 0.25 * ((j > 0 ? tmp[c - un] : 0.0) + (j < n - 1 ? tmp[c + un] : 0.0));
      }
    s.zeta.swap(zeta);
    zeta.resize(cells);
  }

  // New sources: the awake bodies in the water and the window without one, nearest the centre
  // first.
  std::vector<const wake_body *> fresh;
  for (const wake_body &b : bodies) {
    if (!b.in_water || !b.outline || !b.active || !inside(b))
      continue;
    const auto it = _states.find(b.id);
    if (it == _states.end() || !it->second.source)
      fresh.push_back(&b);
  }
  const double mx = x0 + 0.5 * double(n - 1) * _dx, my = y0 + 0.5 * double(n - 1) * _dx;
  std::stable_sort(fresh.begin(), fresh.end(), [&](const wake_body *a, const wake_body *b) {
    const double da =
        (a->position.x - mx) * (a->position.x - mx) + (a->position.y - my) * (a->position.y - my);
    const double db =
        (b->position.x - mx) * (b->position.x - mx) + (b->position.y - my) * (b->position.y - my);
    return da < db; // ties keep the id order
  });
  const auto largest = [&](const state &s) {
    double m = 0.0;
    for (std::size_t c = 0; c < cells; ++c)
      m = std::max(m, std::fabs(s.h[c]));
    return m;
  };
  for (const wake_body *b : fresh) {
    if (sourcing >= _p.sources)
      break;
    auto it = _states.find(b->id);
    if (it == _states.end()) {
      if (_states.size() >= std::size_t(_p.sources + kMaxFading)) {
        // Room: the faintest fading state goes (the lowest id among equals).
        auto drop = _states.end();
        double least = 0.0;
        for (auto o = _states.begin(); o != _states.end(); ++o) {
          if (o->second.source)
            continue;
          const double m = largest(o->second);
          if (drop == _states.end() || m < least) {
            drop = o;
            least = m;
          }
        }
        if (drop == _states.end())
          break;
        _states.erase(drop);
      }
      state s;
      s.body = b->id;
      s.h.assign(cells, 0.0);
      s.phi.assign(cells, 0.0);
      s.w.assign(cells, 0.0);
      it = _states.emplace(b->id, std::move(s)).first;
    }
    // Its columns now are its last: it makes waves from the next step on.
    columns(*b, &it->second.h);
    it->second.zeta = zeta;
    it->second.source = true;
    it->second.rest = false;
    ++sourcing;
  }

  // Faded states no body feeds go.
  for (auto it = _states.begin(); it != _states.end();) {
    if (!it->second.source && quiet(it->second.h, it->second.phi))
      it = _states.erase(it);
    else
      ++it;
  }

  for (auto &e : _states)
    derive(e.second);
  sum();

  // The sleeping bodies in the water the others' waves reach.
  for (const wake_body &b : bodies) {
    if (b.active || !b.in_water || !inside(b))
      continue;
    wake_sample ws;
    sample(own(b.id), b.position.x, b.position.y, ws);
    if (std::fabs(ws.h) > kWakeHeight)
      wake_up.push_back(b.id);
  }
}

void fluid_wake::sample(int own, double x, double y, wake_sample &out) const {
  out = wake_sample{};
  if (!_placed)
    return;
  const int n = _n;
  const double fx = (x - double(_ci) * _dx) / _dx, fy = (y - double(_cj) * _dx) / _dx;
  if (!(fx >= 0.0) || !(fy >= 0.0) || !(fx <= n - 1.0) || !(fy <= n - 1.0))
    return;
  const int i = std::min(int(fx), n - 2), j = std::min(int(fy), n - 2);
  const double tx = fx - i, ty = fy - j;
  const std::size_t un = std::size_t(n), c = std::size_t(j) * un + std::size_t(i);
  const double w00 = (1.0 - tx) * (1.0 - ty), w10 = tx * (1.0 - ty), w01 = (1.0 - tx) * ty,
               w11 = tx * ty;
  const std::size_t a = c * kQ, b = (c + 1) * kQ, d = (c + un) * kQ, e = (c + un + 1) * kQ;
  const double *t = _total.data();
  const double *o =
      own >= 0 && std::size_t(own) < _byIndex.size() ? _byIndex[own]->f.data() : nullptr;
  double v[kQ];
  for (int q = 0; q < kQ; ++q) {
    double s00 = t[a + q], s10 = t[b + q], s01 = t[d + q], s11 = t[e + q];
    if (o) {
      s00 -= o[a + q];
      s10 -= o[b + q];
      s01 -= o[d + q];
      s11 -= o[e + q];
    }
    v[q] = s00 * w00 + s10 * w10 + s01 * w01 + s11 * w11;
  }
  out.h = v[0];
  out.u = v[1];
  out.v = v[2];
  out.w = v[3];
  out.gx = v[4];
  out.gy = v[5];
}

wake_info fluid_wake::info() const {
  wake_info w;
  w.placed = _placed;
  w.x0 = double(_ci) * _dx;
  w.y0 = double(_cj) * _dx;
  w.cell = _dx;
  w.cells = _n;
  w.windows = int(_states.size());
  w.outside = _outside;
  for (const auto &e : _states)
    if (e.second.source) {
      body_id id;
      id.value = e.first;
      w.sources.push_back(id);
    }
  return w;
}

void fluid_wake::save(std::string &out) const {
  put(out, std::uint8_t(_placed ? 1 : 0));
  put(out, _ci);
  put(out, _cj);
  put(out, _count);
  put(out, std::uint32_t(_states.size()));
  for (const auto &e : _states) {
    put(out, e.first);
    put(out, std::uint8_t((e.second.source ? 1 : 0) | (e.second.rest ? 2 : 0)));
    put_values(out, e.second.h);
    put_values(out, e.second.phi);
    put_values(out, e.second.w);
    put_values(out, e.second.zeta);
  }
}

std::shared_ptr<fluid_wake::saved> fluid_wake::parse(const std::string &in,
                                                     std::size_t &pos) const {
  const std::size_t cells = std::size_t(_n) * std::size_t(_n);
  auto out = std::make_shared<saved>();
  std::uint8_t placed = 0;
  std::uint32_t count = 0;
  get(in, pos, placed);
  get(in, pos, out->ci);
  get(in, pos, out->cj);
  get(in, pos, out->count);
  get(in, pos, count);
  need(placed <= 1, "the saved wake is not one");
  need(count <= std::uint32_t(_p.sources + kMaxFading),
       "the saved wake has more states than it allows");
  out->placed = placed != 0;
  for (std::uint32_t k = 0; k < count; ++k) {
    state s;
    std::uint8_t flags = 0;
    get(in, pos, s.body);
    get(in, pos, flags);
    need(flags <= 3, "the saved wake is not one");
    s.source = (flags & 1) != 0;
    s.rest = (flags & 2) != 0;
    get_values(in, pos, s.h, cells);
    get_values(in, pos, s.phi, cells);
    get_values(in, pos, s.w, cells);
    get_values(in, pos, s.zeta, cells);
    derive(s);
    const std::uint32_t id = s.body;
    need(out->states.emplace(id, std::move(s)).second, "the saved wake names a body twice");
  }
  return out;
}

void fluid_wake::commit(saved &s) {
  _placed = s.placed;
  _ci = s.ci;
  _cj = s.cj;
  _count = s.count;
  _states.swap(s.states);
  _outside = 0;
  sum();
}

} // namespace detail
} // namespace physics
} // namespace cvc

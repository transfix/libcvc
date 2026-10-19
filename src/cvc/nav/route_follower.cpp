/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See route_follower.h.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>

namespace cvc {
namespace nav {

namespace {

constexpr double kPi = 3.14159265358979323846;

double dist(double ax, double ay, double bx, double by) { return std::hypot(bx - ax, by - ay); }

// The point `s` along the route from pose k, at most to pose `last`.
void along(const std::vector<route_pose> &p, int k, int last, double s, double &x, double &y) {
  x = p[k].x;
  y = p[k].y;
  for (int j = k; j < last; ++j) {
    const double seg = dist(p[j].x, p[j].y, p[j + 1].x, p[j + 1].y);
    if (s <= seg && seg > 0.0) {
      const double t = s / seg;
      x = p[j].x + t * (p[j + 1].x - p[j].x);
      y = p[j].y + t * (p[j + 1].y - p[j].y);
      return;
    }
    s -= seg;
    x = p[j + 1].x;
    y = p[j + 1].y;
  }
}

double remaining(const std::vector<route_pose> &p, int k, int last) {
  double s = 0.0;
  for (int j = k; j < last; ++j)
    s += dist(p[j].x, p[j].y, p[j + 1].x, p[j + 1].y);
  return s;
}

} // namespace

route_follower::route_follower(sim_world &sim, follower_options options)
    : _sim(sim), _o(options), _agents(static_cast<std::size_t>(sim.size())) {}

void route_follower::set(int i, const vehicle_route &r) {
  if (i < 0 || i >= static_cast<int>(_agents.size()))
    return;
  agent a;
  a.poses = r.poses;
  // A leg ends where the gear of the motion into the next pose changes, and at the route's end.
  const int n = static_cast<int>(a.poses.size());
  for (int k = 1; k + 1 < n; ++k)
    if (a.poses[k + 1].reverse != a.poses[k].reverse)
      a.leg_end.push_back(k);
  if (n > 0)
    a.leg_end.push_back(n - 1);
  if (n < 2)
    a.done = true;
  _agents[static_cast<std::size_t>(i)] = a;
}

void route_follower::clear(int i) {
  if (i >= 0 && i < static_cast<int>(_agents.size()))
    _agents[static_cast<std::size_t>(i)] = agent{};
}

bool route_follower::following(int i) const {
  return i >= 0 && i < static_cast<int>(_agents.size()) &&
         !_agents[static_cast<std::size_t>(i)].poses.empty();
}

int route_follower::leg(int i) const {
  return following(i) ? _agents[static_cast<std::size_t>(i)].leg : 0;
}

int route_follower::legs(int i) const {
  return following(i) ? static_cast<int>(_agents[static_cast<std::size_t>(i)].leg_end.size()) : 0;
}

bool route_follower::reversing(int i) const {
  if (!following(i) || done(i))
    return false;
  const agent &a = _agents[static_cast<std::size_t>(i)];
  const int b = a.leg == 0 ? 0 : a.leg_end[static_cast<std::size_t>(a.leg - 1)];
  const int e = a.leg_end[static_cast<std::size_t>(a.leg)];
  return a.poses[static_cast<std::size_t>(std::min(b + 1, e))].reverse;
}

bool route_follower::done(int i) const {
  return following(i) && _agents[static_cast<std::size_t>(i)].done;
}

void route_follower::apply() {
  const sim_world::config &cfg = _sim.current_config();
  const int n = _sim.size();
  std::vector<float> pos(2 * static_cast<std::size_t>(n)), head(n), spd(n);
  _sim.snapshot(pos.data(), head.data(), spd.data(), nullptr, nullptr);
  const double scale = cfg.scale, dt = cfg.veh.dt;
  const double wheelbase = _o.wheelbase > 0.0 ? _o.wheelbase : cfg.veh.L / scale;
  const double max_steer = _o.max_steer > 0.0 ? _o.max_steer : cfg.veh.delta_max;
  const auto to_norm_x = [&](double x) { return static_cast<float>((x - cfg.cx) * scale); };
  const auto to_norm_y = [&](double y) { return static_cast<float>((y - cfg.cy) * scale); };

  for (int i = 0; i < n && i < static_cast<int>(_agents.size()); ++i) {
    agent &a = _agents[static_cast<std::size_t>(i)];
    if (a.poses.empty() || a.done)
      continue;
    const double x = pos[2 * i], y = pos[2 * i + 1], th = head[i], v = spd[i];
    const std::vector<route_pose> &p = a.poses;
    const double step = std::max(1e-6, dist(p[0].x, p[0].y, p[1].x, p[1].y));
    const double tol = _o.cusp_tolerance > 0.0 ? _o.cusp_tolerance : step;
    const int legs = static_cast<int>(a.leg_end.size());
    const int b = a.leg == 0 ? 0 : a.leg_end[static_cast<std::size_t>(a.leg - 1)];
    const int e = a.leg_end[static_cast<std::size_t>(a.leg)];
    const int gear = p[static_cast<std::size_t>(std::min(b + 1, e))].reverse ? -1 : 1;
    const bool last = a.leg == legs - 1;

    // Progress: the nearest pose of this leg, ahead of where the agent last was.
    int best = std::max(a.progress, b);
    double best_d = dist(x, y, p[best].x, p[best].y);
    for (int k = best + 1; k <= e && k <= best + 64; ++k) {
      const double d = dist(x, y, p[k].x, p[k].y);
      if (d < best_d) {
        best_d = d;
        best = k;
      }
    }
    a.progress = best;

    if (last && gear > 0) {
      // The trained drive's leg: a carrot along the route, and the drive parks at its end.
      const double look = _o.lookahead > 0.0 ? _o.lookahead : 3.0 * step;
      const int ahead = std::max(1, static_cast<int>(std::lround(look / step)));
      if (a.carrot < 0 || (a.carrot < e && 2 * (a.carrot - a.progress) <= ahead)) {
        a.carrot = std::min(e, a.progress + ahead);
        _sim.retarget(i, to_norm_x(p[a.carrot].x), to_norm_y(p[a.carrot].y));
      }
      if (dist(x, y, p[e].x, p[e].y) <= tol)
        a.done = true;
      continue;
    }

    // A manoeuvre leg, driven geometrically to its end.
    const double to_end =
        remaining(p, a.progress, e) + (a.progress == e ? dist(x, y, p[e].x, p[e].y) : 0.0);
    const bool at_end = dist(x, y, p[e].x, p[e].y) <= tol;
    if (at_end) {
      // Stop here, then take the next leg (or, at the route's end, hand the agent to the drive).
      const double a_stop = std::min(std::max(-v / dt, -_o.accel), _o.accel);
      _sim.override_command(i, 0.0f, static_cast<float>(a_stop * scale));
      if (std::fabs(v) <= _o.stop_speed) {
        if (last) {
          a.done = true;
          _sim.retarget(i, to_norm_x(p[e].x), to_norm_y(p[e].y));
        } else {
          ++a.leg;
          a.progress = e;
        }
      }
      continue;
    }
    // Pure pursuit in the leg's gear, about the rear axle.
    const double look = std::max(wheelbase, 1.5 * step);
    double tx = 0.0, ty = 0.0;
    along(p, a.progress, e, look, tx, ty);
    const double phi = th + (gear < 0 ? kPi : 0.0); // the direction of travel
    const double dx = tx - x, dy = ty - y;
    const double lx = std::cos(phi) * dx + std::sin(phi) * dy;
    const double ly = -std::sin(phi) * dx + std::cos(phi) * dy;
    const double kappa = 2.0 * ly / std::max(lx * lx + ly * ly, 1e-9);
    double steer = std::atan(kappa * wheelbase);
    if (gear < 0)
      steer = -steer; // backing up, a left steer swings the travel direction right
    steer = std::min(std::max(steer, -max_steer), max_steer);
    const double v_want =
        gear * std::min(_o.manoeuvre_speed, std::sqrt(2.0 * _o.accel * std::max(0.0, to_end)));
    const double acc = std::min(std::max((v_want - v) / dt, -_o.accel), _o.accel);
    _sim.override_command(i, static_cast<float>(steer), static_cast<float>(acc * scale));
  }
}

} // namespace nav
} // namespace cvc

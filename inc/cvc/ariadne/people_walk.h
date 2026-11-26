/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// people_walk.h — a scripted person who walks a way among cars (ARIADNE_WORLDS.md §18.2, OW7d).
//
// A person a program walks (a gym's gym_person, a test's walker, an NPC) is given a velocity to
// walk at. Walked straight, they push into a car stopped across their way, and step in front of a
// moving one. walker_velocity() is what they walk at instead, from where the cars are and how they
// move (WorldVehicles::footprints): in each car's frame, the person's motion relative to the car
// over the next `horizon_s` seconds is a segment, and the car's box grown by the person's radius
// and a margin is what they must not enter. Clear of every car, the velocity wanted. Into a slow
// car (a parked one, one stopped for them), a step to the side, round its nearer end, at the pace
// wanted. Into a moving car, a wait, or a dodge out of its lane when standing would not do (the car
// would reach them where they stand). The same inputs give the same velocity everywhere.

#ifndef __CVC_ARIADNE_PEOPLE_WALK_H__
#define __CVC_ARIADNE_PEOPLE_WALK_H__

#include <algorithm>
#include <cmath>
#include <vector>

namespace cvc {
namespace ariadne {

// A car as a walker sees it: its chassis's centre and heading, its half-length and half-width (m)
// and its velocity (m/s).
struct walker_car {
  double x = 0.0, y = 0.0, yaw = 0.0;
  double half_length = 0.0, half_width = 0.0;
  double vx = 0.0, vy = 0.0;
};

struct walker_params {
  double radius = 0.35;   // m: the person
  double margin = 0.4;    // m: kept between them and a car's side
  double horizon_s = 2.0; // s: how far ahead the walk is looked at
  double slow = 0.5;      // m/s: a car slower than this is walked round; faster, waited for
  double dodge = 1.5;     // the pace of a dodge out of a moving car's lane, times the walk's
};

namespace walker_detail {
// Whether the segment from (x0, y0) by (dx, dy) meets the box [-hx, hx] x [-hy, hy] (slabs).
inline bool segment_hits_box(double x0, double y0, double dx, double dy, double hx, double hy) {
  double t0 = 0.0, t1 = 1.0;
  const double p[2] = {x0, y0}, d[2] = {dx, dy}, h[2] = {hx, hy};
  for (int k = 0; k < 2; ++k) {
    if (std::fabs(d[k]) < 1e-12) {
      if (p[k] < -h[k] || p[k] > h[k])
        return false;
      continue;
    }
    double a = (-h[k] - p[k]) / d[k], b = (h[k] - p[k]) / d[k];
    if (a > b)
      std::swap(a, b);
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    if (t0 > t1)
      return false;
  }
  return true;
}
} // namespace walker_detail

// The velocity (vx, vy, m/s) a person at (x, y) wanting to walk at (want_x, want_y) walks at among
// CARS (see above).
inline void walker_velocity(double x, double y, double want_x, double want_y,
                            const std::vector<walker_car> &cars, double &out_x, double &out_y,
                            const walker_params &p = {}) {
  out_x = want_x;
  out_y = want_y;
  const double pace = std::hypot(want_x, want_y);
  if (pace < 1e-9)
    return;
  const double grow = p.radius + p.margin;
  // The nearest car the walk runs into, in its own frame.
  const walker_car *in_way = nullptr;
  double nearest = 1e300;
  for (const walker_car &c : cars) {
    const double ch = std::cos(c.yaw), sh = std::sin(c.yaw);
    const double rx = x - c.x, ry = y - c.y;
    const double lx = rx * ch + ry * sh, ly = -rx * sh + ry * ch; // the person, in the car's frame
    const double vx = (want_x - c.vx) * ch + (want_y - c.vy) * sh;
    const double vy = -(want_x - c.vx) * sh + (want_y - c.vy) * ch; // relative to the car
    if (!walker_detail::segment_hits_box(lx, ly, vx * p.horizon_s, vy * p.horizon_s,
                                         c.half_length + grow, c.half_width + grow))
      continue;
    const double d = std::hypot(rx, ry);
    if (d < nearest) {
      nearest = d;
      in_way = &c;
    }
  }
  if (!in_way)
    return;
  const walker_car &c = *in_way;
  const double ch = std::cos(c.yaw), sh = std::sin(c.yaw);
  const double rx = x - c.x, ry = y - c.y;
  const double lx = rx * ch + ry * sh, ly = -rx * sh + ry * ch;
  const double car_speed = std::hypot(c.vx, c.vy);
  if (car_speed >= p.slow) {
    // A moving car: wait, unless it would reach them standing; then out of its lane, to the side
    // they are on.
    const double svx = -c.vx * ch - c.vy * sh, svy = c.vx * sh - c.vy * ch; // standing, relative
    const bool reached = walker_detail::segment_hits_box(
        lx, ly, svx * p.horizon_s, svy * p.horizon_s, c.half_length + grow, c.half_width + grow);
    if (!reached) {
      out_x = out_y = 0.0;
      return;
    }
    const double side = ly >= 0.0 ? 1.0 : -1.0; // the car's left (+) or right
    const double s = p.dodge * pace;
    out_x = -sh * side * s;
    out_y = ch * side * s;
    return;
  }
  // A slow car in the way, at the pace wanted. In the car's frame: walking across its length, round
  // its nearer end (along it toward that end, and out from its side so as not to scrape it);
  // walking along it, out to the side they are nearer, still going their way.
  const double wx = want_x * ch + want_y * sh, wy = -want_x * sh + want_y * ch;
  const double side = ly >= 0.0 ? 1.0 : -1.0;
  double dx = 0.0, dy = 0.0;
  if (std::fabs(wy) >= std::fabs(wx)) {
    const double end = lx >= 0.0 ? 1.0 : -1.0;
    const double out = std::fabs(ly) > c.half_width + grow ? 0.0 : side;
    dx = end;
    dy = 0.5 * out;
  } else {
    dx = 0.3 * (wx > 0.0 ? 1.0 : -1.0);
    dy = side;
  }
  const double n = std::hypot(dx, dy);
  dx /= n;
  dy /= n;
  out_x = (dx * ch - dy * sh) * pace;
  out_y = (dx * sh + dy * ch) * pace;
}

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_PEOPLE_WALK_H__

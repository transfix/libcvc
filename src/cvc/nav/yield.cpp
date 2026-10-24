/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See nav/yield.h and nav/intent.h.

#include <algorithm>
#include <cmath>
#include <cvc/nav/intent.h>
#include <cvc/nav/yield.h>

namespace cvc {
namespace nav {

yield_resolver priority_resolver(float width) {
  const float w = width > 0.0f ? width : 0.5f;
  return [w](const agent_intent &self, const agent_intent &other) {
    return 1.0f / (1.0f + std::exp(-(other.priority - self.priority) / w));
  };
}

float intent_path_distance(float x0, float y0, float h, const float *path, int n, float px,
                           float py, float &s, float &qx, float &qy, float &tx, float &ty) {
  tx = std::cos(h);
  ty = std::sin(h);
  qx = x0;
  qy = y0;
  s = 0.0f;
  float best = std::hypot(px - x0, py - y0), arc = 0.0f;
  float ax = x0, ay = y0;
  for (int k = 0; k < n; ++k) {
    const float bx = path[2 * k], by = path[2 * k + 1];
    const float ex = bx - ax, ey = by - ay, len2 = ex * ex + ey * ey;
    if (len2 > 1e-12f) {
      const float len = std::sqrt(len2);
      float u = ((px - ax) * ex + (py - ay) * ey) / len2;
      u = std::min(1.0f, std::max(0.0f, u));
      const float cx = ax + u * ex, cy = ay + u * ey, d = std::hypot(px - cx, py - cy);
      if (d < best) {
        best = d;
        qx = cx;
        qy = cy;
        s = arc + u * len;
        tx = ex / len;
        ty = ey / len;
      }
      arc += len;
    }
    ax = bx;
    ay = by;
  }
  return best;
}

namespace {

// `from`'s intent carried forward over its age at its speed: the pose and the path both.
agent_intent advanced(const agent_intent &a, double t) {
  agent_intent b = a;
  const float age = static_cast<float>(std::max(0.0, t - a.t));
  if (age > 0.0f && a.speed != 0.0f) {
    const float dx = a.speed * age * std::cos(a.heading), dy = a.speed * age * std::sin(a.heading);
    b.x += dx;
    b.y += dy;
    for (int k = 0; k < b.n_path; ++k) {
      b.path[2 * k] += dx;
      b.path[2 * k + 1] += dy;
    }
  }
  return b;
}

// Whether (px, py) lies ahead of the path's start: past it along the path, or beside it rather
// than behind it.
bool ahead_of(const agent_intent &a, float s, float px, float py, float clearance) {
  if (s > 0.0f)
    return true;
  const float c = std::cos(a.heading), sn = std::sin(a.heading);
  return (px - a.x) * c + (py - a.y) * sn > -0.5f * clearance;
}

} // namespace

yield_shift yield_agent(const yield_config &c, float clearance, const agent_intent &self,
                        const std::vector<agent_intent> &heard, double t,
                        const yield_resolver &resolve) {
  yield_shift out;
  const float C = clearance, H = c.horizon_s;
  for (const agent_intent &raw : heard) {
    const agent_intent j = advanced(raw, t);
    float s, qx, qy, tx, ty;
    // 1. Out of the way: self in j's path.
    float d = intent_path_distance(j.x, j.y, j.heading, j.path, j.n_path, self.x, self.y, s, qx, qy,
                                   tx, ty);
    if (d < C && ahead_of(j, s, self.x, self.y, C)) {
      const float t_arr = s / std::max(std::fabs(j.speed), c.min_speed);
      if (t_arr < H) {
        const float u = 1.0f - t_arr / H;
        const float w = resolve ? resolve(self, j) : 0.5f;
        float nx, ny;
        if (d > 1e-4f * C) {
          nx = (self.x - qx) / d;
          ny = (self.y - qy) / d;
        } else { // on the path: to its right
          nx = ty;
          ny = -tx;
        }
        const float k = c.gain * (C - d) * u * w;
        out.dx += k * nx;
        out.dy += k * ny;
        ++out.conflicts;
      }
    }
    // 2. Around: j in self's path.
    d = intent_path_distance(self.x, self.y, self.heading, self.path, self.n_path, j.x, j.y, s, qx,
                             qy, tx, ty);
    if (d < C && ahead_of(self, s, j.x, j.y, C)) {
      const float t_arr = s / std::max(std::fabs(self.speed), c.min_speed);
      if (t_arr < H) {
        const float u = 1.0f - t_arr / H;
        const float w = 1.0f - c.cooperation * (resolve ? resolve(j, self) : 0.5f);
        float nx, ny;
        if (d > 1e-4f * C) { // away from j, across the path
          nx = (qx - j.x) / d;
          ny = (qy - j.y) / d;
        } else { // j on the path: pass it on the left
          nx = -ty;
          ny = tx;
        }
        const float k = c.pass_gain * (C - d) * u * w;
        out.dx += k * nx;
        out.dy += k * ny;
        ++out.conflicts;
      }
    }
  }
  return out;
}

} // namespace nav
} // namespace cvc

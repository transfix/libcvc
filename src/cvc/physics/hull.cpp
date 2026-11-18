/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// hull.cpp — lofted hulls and a hull's waterline box (hull.h). box_hull is with the field step.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cvc/physics/hull.h>
#include <limits>
#include <stdexcept>
#include <string>

namespace cvc {
namespace physics {

namespace {

void need(bool ok, const std::string &what) {
  if (!ok)
    throw std::invalid_argument("loft_hull: " + what);
}

} // namespace

std::shared_ptr<const hull_mesh> loft_hull(const std::vector<hull_station> &stations) {
  need(stations.size() >= 2, "it needs at least 2 stations");
  const std::size_t k = stations.front().section.size();
  need(k >= 2, "each section needs at least 2 points (its keel and its sheer)");
  for (std::size_t i = 0; i < stations.size(); ++i) {
    const hull_station &s = stations[i];
    need(std::isfinite(s.x), "a station's x is not finite");
    need(i == 0 || s.x > stations[i - 1].x, "the stations' x must increase, stern to bow");
    need(s.section.size() == k, "every section needs the same number of points");
    for (const auto &p : s.section)
      need(std::isfinite(p[0]) && std::isfinite(p[1]) && p[0] >= 0.0,
           "a section point is not finite or has a negative half width");
  }
  // Each station a ring: its keel (on the centre line), starboard up to the sheer, then port
  // from the sheer down, so the ring runs counter-clockwise seen from ahead (+x).
  const std::size_t ring = 2 * k - 1;
  auto h = std::make_shared<hull_mesh>();
  h->vertices.reserve(stations.size() * ring + 2);
  for (const hull_station &s : stations) {
    h->vertices.push_back({s.x, 0.0, s.section[0][1]});
    for (std::size_t j = 1; j < k; ++j)
      h->vertices.push_back({s.x, s.section[j][0], s.section[j][1]});
    for (std::size_t j = k - 1; j >= 1; --j)
      h->vertices.push_back({s.x, -s.section[j][0], s.section[j][1]});
  }
  const auto at = [ring](std::size_t station, std::size_t r) {
    return static_cast<std::uint32_t>(station * ring + r % ring);
  };
  for (std::size_t i = 0; i + 1 < stations.size(); ++i)
    for (std::size_t r = 0; r < ring; ++r) {
      const std::uint32_t a = at(i, r), b = at(i, r + 1), c = at(i + 1, r + 1), d = at(i + 1, r);
      h->triangles.push_back({a, b, c});
      h->triangles.push_back({a, c, d});
    }
  // The end caps: fans about each end ring's centre, the transom facing aft, the stem ahead.
  for (const std::size_t i : {std::size_t{0}, stations.size() - 1}) {
    vec3 c;
    for (std::size_t r = 0; r < ring; ++r)
      c = c + h->vertices[at(i, r)];
    c = c * (1.0 / static_cast<double>(ring));
    const auto centre = static_cast<std::uint32_t>(h->vertices.size());
    h->vertices.push_back(c);
    for (std::size_t r = 0; r < ring; ++r) {
      if (i == 0)
        h->triangles.push_back({centre, at(i, r + 1), at(i, r)});
      else
        h->triangles.push_back({centre, at(i, r), at(i, r + 1)});
    }
  }
  return h;
}

std::optional<waterline_box> hull_waterline_box(const hull_mesh &h, double z) {
  // The outline at z: where each triangle crosses the plane, sampled every 2.5 cm, as (x, |y|).
  std::vector<std::array<double, 2>> outline;
  for (const auto &t : h.triangles) {
    double cut[2][2];
    int n = 0;
    for (int e = 0; e < 3 && n < 2; ++e) {
      const vec3 &a = h.vertices[t[e]], &b = h.vertices[t[(e + 1) % 3]];
      const double da = a.z - z, db = b.z - z;
      if ((da < 0.0) == (db < 0.0))
        continue;
      const double s = da / (da - db);
      cut[n][0] = a.x + s * (b.x - a.x);
      cut[n][1] = a.y + s * (b.y - a.y);
      ++n;
    }
    if (n < 2)
      continue;
    const double len = std::hypot(cut[1][0] - cut[0][0], cut[1][1] - cut[0][1]);
    const int m = 1 + static_cast<int>(len / 0.025);
    for (int i = 0; i <= m; ++i) {
      const double s = static_cast<double>(i) / m;
      outline.push_back({cut[0][0] + s * (cut[1][0] - cut[0][0]),
                         std::fabs(cut[0][1] + s * (cut[1][1] - cut[0][1]))});
    }
  }
  if (outline.empty())
    return std::nullopt;
  double x0 = std::numeric_limits<double>::max(), x1 = std::numeric_limits<double>::lowest();
  for (const auto &q : outline) {
    x0 = std::min(x0, q[0]);
    x1 = std::max(x1, q[0]);
  }
  // Its half width in bins along x (5 cm, or the length over 400), then the run of bins whose
  // least half width times their length is largest.
  const double bin = std::max(0.05, (x1 - x0) / 400.0);
  const int nb = std::max(1, static_cast<int>(std::ceil((x1 - x0) / bin)));
  std::vector<double> w(static_cast<std::size_t>(nb), 0.0);
  for (const auto &q : outline) {
    const int i = std::min(nb - 1, static_cast<int>((q[0] - x0) / bin));
    w[static_cast<std::size_t>(i)] = std::max(w[static_cast<std::size_t>(i)], q[1]);
  }
  double best = 0.0, best_w = 0.0;
  int best_a = 0, best_b = -1;
  for (int a = 0; a < nb; ++a) {
    double least = std::numeric_limits<double>::max();
    for (int b = a; b < nb; ++b) {
      least = std::min(least, w[static_cast<std::size_t>(b)]);
      if (!(least > 0.0))
        break;
      const double area = least * (b - a + 1);
      if (area > best) {
        best = area;
        best_w = least;
        best_a = a;
        best_b = b;
      }
    }
  }
  if (best_b < best_a)
    return std::nullopt;
  const double xa = x0 + best_a * bin, xb = std::min(x1, x0 + (best_b + 1) * bin);
  waterline_box box;
  box.x = 0.5 * (xa + xb);
  box.half_length = 0.5 * (xb - xa);
  box.half_beam = best_w;
  return box;
}

} // namespace physics
} // namespace cvc

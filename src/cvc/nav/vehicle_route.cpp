/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See vehicle_route.h.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/vehicle_route.h>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace cvc {
namespace nav {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// The distance from each cell centre to the nearest wall's surface (world units): the exact
// distance transform between cell centres, less half a cell for the wall's own extent. Sampled
// bilinearly between cells; outside the grid is a wall.
struct distance_field {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, dx = 1.0, dy = 1.0, cell = 1.0;
  std::vector<float> dist;

  distance_field(const std::uint8_t *occ, int r, int c, double x0, double y0, double x1, double y1)
      : rows(r), cols(c), min_x(x0), min_y(y0) {
    dx = cols > 1 ? (x1 - x0) / (cols - 1) : 1.0;
    dy = rows > 1 ? (y1 - y0) / (rows - 1) : 1.0;
    cell = 0.5 * (dx + dy);
    const std::vector<double> e2 = edt2_squared(occ, rows, cols);
    dist.resize(e2.size());
    for (std::size_t i = 0; i < e2.size(); ++i)
      dist[i] = static_cast<float>((std::sqrt(e2[i]) - 0.5) * cell);
  }

  double at(double x, double y) const {
    const double fc = (x - min_x) / dx, fr = (y - min_y) / dy;
    if (!(fc >= 0.0 && fr >= 0.0 && fc <= cols - 1 && fr <= rows - 1))
      return -0.5 * cell;
    const int c0 = std::min(static_cast<int>(fc), cols - 1),
              r0 = std::min(static_cast<int>(fr), rows - 1);
    const int c1 = std::min(c0 + 1, cols - 1), r1 = std::min(r0 + 1, rows - 1);
    const double tc = fc - c0, tr = fr - r0;
    const auto d = [&](int r, int c) {
      return static_cast<double>(dist[static_cast<std::size_t>(r) * cols + c]);
    };
    return (1.0 - tr) * ((1.0 - tc) * d(r0, c0) + tc * d(r0, c1)) +
           tr * ((1.0 - tc) * d(r1, c0) + tc * d(r1, c1));
  }

  int cell_of(double x, double y, int &r, int &c) const {
    c = static_cast<int>(std::lround((x - min_x) / dx));
    r = static_cast<int>(std::lround((y - min_y) / dy));
    if (r < 0 || c < 0 || r >= rows || c >= cols)
      return -1;
    return r * cols + c;
  }
};

double footprint_clearance(const distance_field &f, const route_vehicle &v, double x, double y,
                           double th) {
  if (v.disc_offsets.empty())
    return f.at(x, y) - v.disc_radius;
  const double c = std::cos(th), s = std::sin(th);
  double m = kInf;
  for (const double off : v.disc_offsets)
    m = std::min(m, f.at(x + off * c, y + off * s));
  return m - v.disc_radius;
}

// The shortest 8-connected distance (world units) from every free cell to the goal's cell,
// through free cells; infinity where the goal cannot be reached.
std::vector<double> distance_to_goal(const std::uint8_t *occ, const distance_field &f, int goal) {
  const int rows = f.rows, cols = f.cols;
  std::vector<double> d(static_cast<std::size_t>(rows) * cols, kInf);
  using item = std::pair<double, int>;
  std::priority_queue<item, std::vector<item>, std::greater<item>> q;
  d[static_cast<std::size_t>(goal)] = 0.0;
  q.push({0.0, goal});
  const double diag = std::sqrt(f.dx * f.dx + f.dy * f.dy);
  while (!q.empty()) {
    const item top = q.top();
    q.pop();
    if (top.first > d[static_cast<std::size_t>(top.second)])
      continue;
    const int r = top.second / cols, c = top.second % cols;
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc) {
        if (!dr && !dc)
          continue;
        const int nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols)
          continue;
        const int n = nr * cols + nc;
        if (occ[n])
          continue;
        const double w = top.first + (dr && dc ? diag : (dr ? f.dy : f.dx));
        if (w < d[static_cast<std::size_t>(n)]) {
          d[static_cast<std::size_t>(n)] = w;
          q.push({w, n});
        }
      }
  }
  return d;
}

// The pose after an arc of signed length l at curvature k from (x, y, th).
void arc(double x, double y, double th, double k, double l, double &ox, double &oy, double &oth) {
  if (std::fabs(k) < 1e-12) {
    ox = x + l * std::cos(th);
    oy = y + l * std::sin(th);
    oth = th;
    return;
  }
  oth = th + l * k;
  ox = x + (std::sin(oth) - std::sin(th)) / k;
  oy = y - (std::cos(oth) - std::cos(th)) / k;
}

double wrap(double a) {
  a = std::fmod(a, 2.0 * kPi);
  return a < 0.0 ? a + 2.0 * kPi : a;
}

struct node {
  double x, y, th, g;
  int parent;
  int dir;      // +1 forward, -1 backing up, 0 the start
  double steer; // the move's curvature over the tightest
  int depth;
};

} // namespace

double route_clearance(const std::uint8_t *occ, int rows, int cols, double min_x, double min_y,
                       double max_x, double max_y, const route_vehicle &v, const route_pose &p) {
  const distance_field f(occ, rows, cols, min_x, min_y, max_x, max_y);
  return footprint_clearance(f, v, p.x, p.y, p.heading);
}

vehicle_route plan_vehicle_route(const std::uint8_t *occ, int rows, int cols, double min_x,
                                 double min_y, double max_x, double max_y, const route_vehicle &v,
                                 const route_pose &start, double goal_x, double goal_y,
                                 const route_options &o) {
  vehicle_route out;
  if (!occ || rows < 2 || cols < 2)
    return out;
  const distance_field f(occ, rows, cols, min_x, min_y, max_x, max_y);
  // A move: 1.5 cells, but no more than 0.6 of the turning radius, so a coarse grid (a cell
  // larger than the vehicle) still turns in arcs it can resolve.
  double step = o.step;
  if (!(step > 0.0)) {
    step = 1.5 * f.cell;
    if (v.min_radius > 0.0)
      step = std::min(step, 0.6 * v.min_radius);
  }
  // The closed set's resolution: a move leaves its cell, whatever the grid's size.
  const double kres = step / 1.5;
  const std::uint64_t kcols =
      static_cast<std::uint64_t>(std::ceil((cols - 1) * f.dx / kres)) + 2;
  const double tol = o.goal_tolerance > 0.0 ? o.goal_tolerance : 2.0 * f.cell;
  const double radius = std::max(v.min_radius, 0.5 * step); // "any" turn: tight, not singular
  const double cusp = v.cusp_cost > 0.0 ? v.cusp_cost : 4.0 * step;
  const double comfort = o.comfort > 0.0 ? o.comfort : std::max(v.disc_radius, f.cell);
  const int headings = std::max(8, o.headings);
  const int samples = std::max(1, static_cast<int>(std::ceil(step / (0.5 * f.cell))));

  // The heuristic: the free-space distance to the goal, which never overestimates.
  int gr = 0, gc = 0;
  int goal_cell = f.cell_of(goal_x, goal_y, gr, gc);
  if (goal_cell < 0) {
    gr = std::clamp(gr, 0, rows - 1);
    gc = std::clamp(gc, 0, cols - 1);
    goal_cell = gr * cols + gc;
  }
  const std::vector<double> to_goal = distance_to_goal(occ, f, goal_cell);
  const double hw = std::max(1.0, o.heuristic_weight);
  const auto h = [&](double x, double y) {
    int r = 0, c = 0;
    const int i = f.cell_of(x, y, r, c);
    if (i < 0)
      return kInf;
    const double grid = to_goal[static_cast<std::size_t>(i)];
    return hw * std::max(grid - tol, std::hypot(goal_x - x, goal_y - y) - tol);
  };

  // A wedged start: its first moves may stay as close as it is, but no closer.
  const double c0 = footprint_clearance(f, v, start.x, start.y, start.heading);
  const bool wedged = c0 < o.margin;
  const int escape = o.escape >= 0
                         ? o.escape
                         : static_cast<int>(std::ceil(2.0 * (v.disc_radius + o.margin) / step)) + 2;

  std::vector<node> nodes;
  nodes.push_back({start.x, start.y, start.heading, 0.0, -1, 0, 0.0, 0});
  const auto key = [&](double x, double y, double th) -> std::uint64_t {
    const double kx = std::max(0.0, std::floor((x - min_x) / kres + 0.5));
    const double ky = std::max(0.0, std::floor((y - min_y) / kres + 0.5));
    const int b = static_cast<int>(wrap(th) / (2.0 * kPi) * headings) % headings;
    return (static_cast<std::uint64_t>(ky) * kcols + static_cast<std::uint64_t>(kx)) *
               static_cast<std::uint64_t>(headings) +
           static_cast<std::uint64_t>(b);
  };
  std::unordered_map<std::uint64_t, double> best;
  best[key(start.x, start.y, start.heading)] = 0.0;
  using item = std::pair<double, int>;
  std::priority_queue<item, std::vector<item>, std::greater<item>> open;
  open.push({h(start.x, start.y), 0});

  static const double kSteers[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
  int found = -1, nearest = 0;
  double nearest_h = h(start.x, start.y);
  while (!open.empty() && out.expansions < o.max_expansions) {
    const int i = open.top().second;
    open.pop();
    const node cur = nodes[static_cast<std::size_t>(i)];
    const auto it = best.find(key(cur.x, cur.y, cur.th));
    if (it != best.end() && cur.g > it->second + 1e-9)
      continue; // a cheaper way here was found after this one was queued
    if (std::hypot(goal_x - cur.x, goal_y - cur.y) <= tol) {
      found = i;
      break;
    }
    ++out.expansions;
    for (int dir = 1; dir >= -1; dir -= 2) {
      if (dir < 0 && !v.reverse)
        continue;
      for (const double s : kSteers) {
        const double k = s / radius;
        const double floor =
            wedged && cur.depth < escape ? std::min(o.margin, c0) - 1e-9 : o.margin;
        double nx = cur.x, ny = cur.y, nth = cur.th, clr = kInf;
        bool ok = true;
        for (int q = 1; q <= samples && ok; ++q) {
          double px, py, pth;
          arc(cur.x, cur.y, cur.th, k, dir * step * q / samples, px, py, pth);
          const double cq = footprint_clearance(f, v, px, py, pth);
          ok = cq >= floor;
          clr = std::min(clr, cq);
          nx = px;
          ny = py;
          nth = pth;
        }
        if (!ok)
          continue;
        const double hn = h(nx, ny);
        if (!std::isfinite(hn))
          continue;
        double cost = step * (dir < 0 ? v.reverse_cost : 1.0);
        if (cur.dir != 0 && dir != cur.dir)
          cost += cusp;
        cost += 0.05 * step * std::fabs(s);
        if (cur.dir == dir)
          cost += 0.2 * step * std::fabs(s - cur.steer);
        cost += o.comfort_cost * step * std::max(0.0, comfort - (clr - o.margin)) / comfort;
        const double g = cur.g + cost;
        const std::uint64_t kn = key(nx, ny, nth);
        const auto b = best.find(kn);
        if (b != best.end() && b->second <= g)
          continue;
        best[kn] = g;
        nodes.push_back({nx, ny, nth, g, i, dir, s, cur.depth + 1});
        const int ni = static_cast<int>(nodes.size()) - 1;
        if (hn < nearest_h) {
          nearest_h = hn;
          nearest = ni;
        }
        open.push({g + hn, ni});
      }
    }
  }

  out.reached = found >= 0;
  std::vector<int> chain;
  for (int i = out.reached ? found : nearest; i >= 0; i = nodes[static_cast<std::size_t>(i)].parent)
    chain.push_back(i);
  std::reverse(chain.begin(), chain.end());
  for (std::size_t k = 0; k < chain.size(); ++k) {
    const node &n = nodes[static_cast<std::size_t>(chain[k])];
    route_pose p;
    p.x = n.x;
    p.y = n.y;
    p.heading = n.th;
    p.reverse = k == 0 ? (chain.size() > 1 && nodes[static_cast<std::size_t>(chain[1])].dir < 0)
                       : n.dir < 0;
    if (k > 0) {
      out.length += step;
      if (n.dir < 0)
        out.reverse_length += step;
      if (k > 1 && n.dir != nodes[static_cast<std::size_t>(chain[k - 1])].dir)
        ++out.cusps;
    }
    out.poses.push_back(p);
  }
  return out;
}

} // namespace nav
} // namespace cvc

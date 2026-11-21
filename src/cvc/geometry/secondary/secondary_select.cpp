/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the legacy SecondaryStructures)
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

// SecondaryStructures' selection, on the skeleton graph: skel.cpp's filter and buildSheetGeometry
// (the sheets), helix.cpp's compute_helices, cluster and build_cylindrical_helices and skel.cpp's
// buildHelixGeometry (the helices), and buildSkeletonGeometry. The arithmetic is the legacy's, in
// its order, but for the fixes cvc/geometry/secondary_structures.h lists. LAPACK's dsyev, which
// the legacy fit called, is a Jacobi rotation here (a 3x3 symmetric matrix).

#include "secondary.h"

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <numeric>
#include <set>
#include <tuple>

namespace cvc::secondary_detail {

namespace {

constexpr int SUBDIV = 100;             // the cylinders' sides
constexpr double SAME_DIRECTION = 0.75; // cos^2 of 30 degrees: one run's edges turn less

using pt3 = std::array<double, 3>;

double dist2(const pt3 &a, const pt3 &b) {
  const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

// helix.cpp's dist(): the least distance between the ends of two segments.
double end_distance(const pt3 &p1, const pt3 &p2, const pt3 &p3, const pt3 &p4) {
  const double d = std::min({dist2(p1, p3), dist2(p1, p4), dist2(p2, p3), dist2(p2, p4)});
  return std::sqrt(d);
}

// cos^2 of the angle between two segments, as helix.cpp computed it.
double cos2(const pt3 &p1, const pt3 &p2, const pt3 &p3, const pt3 &p4) {
  const double v1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
  const double v2[3] = {p4[0] - p3[0], p4[1] - p3[1], p4[2] - p3[2]};
  const double d = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  const double s1 = v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2];
  const double s2 = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
  return std::fabs(d * d / (s1 * s2));
}

struct cedge {
  int a, b;
};

// helix.cpp's cluster(): the candidate points joined within `join`, each component's spanning
// edges (as its depth-first walk found them) split into runs of roughly one direction; a run's
// points, in index order. Runs with no edge are left out (the legacy fit divided by zero for
// them).
step_task cluster(const std::vector<pt3> &pos, double join, std::vector<std::vector<int>> &runs) {
  const int n = int(pos.size());
  // pos is in lexicographic order, so the points within `join` of i in x follow it.
  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
  for (int i = 0; i < n; i++) {
    SK_UNIT;
    for (int j = i + 1; j < n && pos[std::size_t(j)][0] - pos[std::size_t(i)][0] < join; j++)
      if (std::sqrt(dist2(pos[std::size_t(i)], pos[std::size_t(j)])) < join) {
        adj[std::size_t(i)].push_back(j);
        adj[std::size_t(j)].push_back(i);
      }
  }
  std::vector<char> visited(std::size_t(n), 0);
  std::vector<std::vector<cedge>> edge_comps;
  for (int i = 0; i < n; i++) {
    SK_UNIT;
    if (visited[std::size_t(i)])
      continue;
    std::vector<int> Q{i};
    visited[std::size_t(i)] = 1;
    std::vector<cedge> new_edge;
    while (!Q.empty()) {
      SK_UNIT;
      const int vid = Q.back();
      Q.pop_back();
      for (int uid : adj[std::size_t(vid)]) {
        if (visited[std::size_t(uid)])
          continue;
        Q.push_back(uid);
        new_edge.push_back({vid, uid});
        visited[std::size_t(uid)] = 1;
      }
    }
    edge_comps.push_back(std::move(new_edge));
  }
  // Split each component's edges into runs: take the first edge, then every later one roughly
  // parallel to and near the run's last edge, or to any edge already in the run; the rest are the
  // next round's (when more than two are left).
  std::vector<std::vector<cedge>> refined;
  for (std::vector<cedge> &comp : edge_comps) {
    SK_UNIT;
    if (comp.size() <= 1) {
      refined.push_back(comp);
      continue;
    }
    std::vector<cedge> a = comp, b, c;
    while (a.size() > 1) {
      SK_UNIT;
      b.push_back(a.front());
      a.erase(a.begin());
      pt3 p1 = pos[std::size_t(b.back().a)], p2 = pos[std::size_t(b.back().b)];
      for (const cedge &e : a) {
        SK_UNIT;
        const pt3 &p3 = pos[std::size_t(e.a)], &p4 = pos[std::size_t(e.b)];
        bool flag = false;
        if (cos2(p1, p2, p3, p4) >= SAME_DIRECTION && end_distance(p1, p2, p3, p4) < join) {
          b.push_back(e);
          p1 = p3;
          p2 = p4;
          flag = true;
        } else {
          for (std::size_t l = 0; l < b.size(); l++) {
            const pt3 &p5 = pos[std::size_t(b[l].a)], &p6 = pos[std::size_t(b[l].b)];
            if (cos2(p5, p6, p3, p4) >= SAME_DIRECTION && end_distance(p3, p4, p5, p6) < join) {
              flag = true;
              b.push_back(e);
              break;
            }
          }
        }
        if (!flag)
          c.push_back(e);
      }
      a.clear();
      if (c.size() > 2)
        a = c;
      c.clear();
      refined.push_back(b);
      b.clear();
    }
  }
  runs.clear();
  for (const std::vector<cedge> &r : refined) {
    SK_UNIT;
    std::set<int> pts;
    for (const cedge &e : r) {
      pts.insert(e.a);
      pts.insert(e.b);
    }
    if (pts.empty())
      continue;
    runs.emplace_back(pts.begin(), pts.end());
  }
}

struct axis {
  pt3 left{}, right{};
  double sq_length = 0;
  std::vector<int> run;
};

// build_cylindrical_helices' fit: the run's principal axis (the inertia tensor's least
// eigenvector) through its centroid, from its points' largest projection (left) to their smallest
// (right).
axis fit_axis(const std::vector<pt3> &pos, const std::vector<int> &run) {
  const double n = double(run.size());
  double cent[3] = {0, 0, 0};
  for (int k : run)
    for (int d = 0; d < 3; d++)
      cent[d] = cent[d] + pos[std::size_t(k)][std::size_t(d)];
  for (double &c : cent)
    c = c / n;
  double sXX = 0, sXY = 0, sXZ = 0, sYY = 0, sYZ = 0, sZZ = 0;
  for (int k : run) {
    const pt3 &p = pos[std::size_t(k)];
    const double dx = p[0] - cent[0], dy = p[1] - cent[1], dz = p[2] - cent[2];
    sXX += dx * dx;
    sXY += dx * dy;
    sXZ += dx * dz;
    sYY += dy * dy;
    sYZ += dy * dz;
    sZZ += dz * dz;
  }
  sXX /= n;
  sXY /= n;
  sXZ /= n;
  sYY /= n;
  sYZ /= n;
  sZZ /= n;
  const double inertia[3][3] = {
      {sYY + sZZ, -sXY, -sXZ}, {-sXY, sXX + sZZ, -sYZ}, {-sXZ, -sYZ, sXX + sYY}};
  const std::array<double, 3> a = smallest_eigenvector(inertia);
  double mx = 0, mn = 0;
  bool first = true;
  for (int k : run) {
    const pt3 &p = pos[std::size_t(k)];
    const double l = (p[0] - cent[0]) * a[0] + (p[1] - cent[1]) * a[1] + (p[2] - cent[2]) * a[2];
    if (first) {
      mx = mn = l;
      first = false;
    }
    mx = (mx < l) ? l : mx;
    mn = (mn > l) ? l : mn;
  }
  axis r;
  for (int d = 0; d < 3; d++) {
    r.left[std::size_t(d)] = mx * a[std::size_t(d)] + cent[d];
    r.right[std::size_t(d)] = mn * a[std::size_t(d)] + cent[d];
  }
  r.sq_length = dist2(r.left, r.right);
  r.run = run;
  return r;
}

// util.cpp's transform(): source rotated about the unit axis `ax` by the angle with cosine c and
// sine s, then moved by t.
void transform(std::vector<pt3> &target, const std::vector<pt3> &source, const pt3 &t,
               const pt3 &ax, double c, double s) {
  const double a0 = ax[0], a1 = ax[1], a2 = ax[2];
  const double m[3][4] = {
      {a0 * a0 * (1 - c) + c, a0 * a1 * (1 - c) - a2 * s, a0 * a2 * (1 - c) + a1 * s, t[0]},
      {a1 * a0 * (1 - c) + a2 * s, a1 * a1 * (1 - c) + c, a1 * a2 * (1 - c) - a0 * s, t[1]},
      {a0 * a2 * (1 - c) - a1 * s, a1 * a2 * (1 - c) + a0 * s, a2 * a2 * (1 - c) + c, t[2]}};
  target.clear();
  for (const pt3 &p : source)
    target.push_back({p[0] * m[0][0] + p[1] * m[0][1] + p[2] * m[0][2] + m[0][3],
                      p[0] * m[1][0] + p[1] * m[1][1] + p[2] * m[1][2] + m[1][3],
                      p[0] * m[2][0] + p[1] * m[2][1] + p[2] * m[2][2] + m[2][3]});
}

// A helix's cylinder: SUBDIV points on the rim at `start`, SUBDIV at `end`, then the two centres.
std::vector<pt3> cylinder_points(const pt3 &v0, const pt3 &v1, double radius) {
  std::vector<pt3> circle;
  for (int j = 0; j < SUBDIV; j++) {
    const double theta = 2 * M_PI / SUBDIV * j;
    circle.push_back({radius * std::cos(theta), radius * std::sin(theta), 0.0});
  }
  const pt3 a{v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
  // The rotation taking z to a: about cross(z, a), by the angle between them. An axis along z
  // (cross(z, a) = 0) is z itself or its opposite, which the legacy code turned into NaNs.
  pt3 raxis{-a[1], a[0], 0.0};
  const double rl = std::sqrt(raxis[0] * raxis[0] + raxis[1] * raxis[1] + raxis[2] * raxis[2]);
  const double al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  double c = 1.0, s = 0.0;
  if (rl > 0 && al > 0) {
    raxis = {(1. / rl) * raxis[0], (1. / rl) * raxis[1], (1. / rl) * raxis[2]};
    c = a[2] / al;
    s = std::sqrt(1 - c * c);
  } else {
    raxis = {1.0, 0.0, 0.0};
    c = a[2] < 0 ? -1.0 : 1.0;
  }
  std::vector<pt3> lower, upper;
  transform(lower, circle, v0, raxis, c, s);
  transform(upper, circle, {v0[0] + a[0], v0[1] + a[1], v0[2] + a[2]}, raxis, c, s);
  lower.insert(lower.end(), upper.begin(), upper.end());
  lower.push_back(v0);
  lower.push_back({v0[0] + a[0], v0[1] + a[1], v0[2] + a[2]});
  return lower;
}

geometry::color_t sheet_colour(int rank) {
  // skel.cpp's first six colours (blue, green, cyan, red, magenta, yellow), then a fixed palette
  // where the legacy code drew rand() colours; by sheet rank, not component index.
  static constexpr double palette[8][3] = {{0, 0, 1}, {0, 1, 0}, {0, 1, 1},       {1, 0, 0},
                                           {1, 0, 1}, {1, 1, 0}, {0.9, 0.6, 0.0}, {0.35, 0.7, 0.9}};
  const double *c = palette[rank % 8];
  return geometry::color_t{{c[0], c[1], c[2]}};
}

} // namespace

std::array<double, 3> smallest_eigenvector(const double m[3][3]) {
  double a[3][3], v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      a[i][j] = 0.5 * (m[i][j] + m[j][i]);
  for (int sweep = 0; sweep < 64; sweep++) {
    const double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
    const double diag = std::fabs(a[0][0]) + std::fabs(a[1][1]) + std::fabs(a[2][2]);
    if (off == 0 || off <= 1e-18 * diag)
      break;
    for (int p = 0; p < 2; p++)
      for (int q = p + 1; q < 3; q++) {
        if (a[p][q] == 0)
          continue;
        // Numerical Recipes' rotation zeroing a[p][q].
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        double t = std::fabs(theta) > 1e150 ? 0.5 / theta
                                            : (theta >= 0 ? 1.0 : -1.0) /
                                                  (std::fabs(theta) + std::sqrt(theta * theta + 1));
        const double c = 1.0 / std::sqrt(t * t + 1), s = t * c;
        const double apq = a[p][q];
        a[p][p] -= t * apq;
        a[q][q] += t * apq;
        a[p][q] = a[q][p] = 0.0;
        for (int r = 0; r < 3; r++) {
          if (r == p || r == q)
            continue;
          const double arp = a[r][p], arq = a[r][q];
          a[r][p] = a[p][r] = c * arp - s * arq;
          a[r][q] = a[q][r] = c * arq + s * arp;
        }
        for (int r = 0; r < 3; r++) {
          const double vrp = v[r][p], vrq = v[r][q];
          v[r][p] = c * vrp - s * vrq;
          v[r][q] = s * vrp + c * vrq;
        }
      }
  }
  int k = 0;
  for (int i = 1; i < 3; i++)
    if (a[i][i] < a[k][k])
      k = i;
  std::array<double, 3> e{v[0][k], v[1][k], v[2][k]};
  const double l = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
  int big = 0;
  for (int i = 1; i < 3; i++)
    if (std::fabs(e[std::size_t(i)]) > std::fabs(e[std::size_t(big)]))
      big = i;
  const double sign = e[std::size_t(big)] < 0 ? -1.0 : 1.0;
  for (double &x : e)
    x = sign * x / l;
  return e;
}

void check_skeleton(const secondary_skeleton &s) {
  const std::size_t n = s.points.size();
  for (std::size_t i = 0; i < s.edges.size(); i++)
    if (s.edges[i].a >= n || s.edges[i].b >= n)
      throw secondary_error("select_secondary_structures: edge " + std::to_string(i) +
                            " names a point past the skeleton's " + std::to_string(n));
  for (std::size_t i = 0; i < s.faces.size(); i++) {
    const secondary_skeleton::face &f = s.faces[i];
    if (f.points.empty())
      throw secondary_error("select_secondary_structures: polygon " + std::to_string(i) +
                            " has no points");
    for (std::uint32_t p : f.points)
      if (p >= n)
        throw secondary_error("select_secondary_structures: polygon " + std::to_string(i) +
                              " names a point past the skeleton's " + std::to_string(n));
    if (f.component < -1 || f.component >= int(s.component_area.size()))
      throw secondary_error("select_secondary_structures: polygon " + std::to_string(i) +
                            " names component " + std::to_string(f.component) + " of " +
                            std::to_string(s.component_area.size()));
  }
}

step_task select(app &ctx, const secondary_params &p, secondary_structures &r, stage_fn stage) {
  const secondary_skeleton &s = r.skeleton;
  r.helices.clear();
  r.sheets.clear();
  r.helix_geometry = geometry(ctx);
  r.sheet_geometry = geometry(ctx);
  r.skeleton_geometry = geometry(ctx);
  r.helix_of_tri.clear();
  r.sheet_of_tri.clear();
  r.helix_widths.clear();
  r.sheet_widths.clear();

  // ---- the sheets (filter, buildSheetGeometry) ----
  if (stage)
    stage("sheets", s.faces.size() + s.component_area.size() + 1);
  const std::size_t ncomp = s.component_area.size();
  std::vector<int> order(ncomp);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    return s.component_area[std::size_t(a)] > s.component_area[std::size_t(b)];
  });
  r.sheet_candidates = 0;
  for (double a : s.component_area)
    if (a >= p.sheet_min_area)
      r.sheet_candidates++;
  const int nsheets = std::max(0, std::min(p.sheet_count, r.sheet_candidates));
  std::vector<int> rank_of(ncomp, -1);
  for (int i = 0; i < nsheets; i++) {
    rank_of[std::size_t(order[std::size_t(i)])] = i;
    r.sheets.push_back(
        {order[std::size_t(i)], s.component_area[std::size_t(order[std::size_t(i)])], 0});
  }
  {
    geometry &g = r.sheet_geometry;
    for (const secondary_skeleton::face &f : s.faces) {
      SK_UNIT;
      r.sheet_widths.push_back(f.width);
      if (f.width < p.sheet_min_width || f.width > p.sheet_max_width)
        continue;
      if (f.component < 0 || rank_of[std::size_t(f.component)] < 0)
        continue;
      const int rank = rank_of[std::size_t(f.component)];
      const geometry::color_t colour = sheet_colour(rank);
      const std::size_t nv = f.points.size();
      double mid[3] = {0, 0, 0};
      for (std::uint32_t v : f.points)
        for (int d = 0; d < 3; d++)
          mid[d] = mid[d] + s.points[v][std::size_t(d)];
      const auto mid_index = geometry::index_t(g.points().size());
      g.points().push_back(
          geometry::point_t{{mid[0] / double(nv), mid[1] / double(nv), mid[2] / double(nv)}});
      g.colors().push_back(colour);
      const auto first = geometry::index_t(g.points().size());
      for (std::size_t vi = 0; vi < nv; vi++) {
        const auto cur = geometry::index_t(g.points().size());
        g.tris().push_back(geometry::tri_t{{cur, vi == nv - 1 ? first : cur + 1, mid_index}});
        r.sheet_of_tri.push_back(rank);
        const pt3 &q = s.points[f.points[vi]];
        g.points().push_back(geometry::point_t{{q[0], q[1], q[2]}});
        g.colors().push_back(colour);
      }
      r.sheets[std::size_t(rank)].polygons++;
    }
  }

  // ---- the helices (compute_helices, cluster, build_cylindrical_helices) ----
  if (stage)
    stage("helices", s.edges.size() + 4 * s.points.size() + 1);
  std::vector<std::vector<int>> point_comps(s.points.size());
  for (const secondary_skeleton::face &f : s.faces) {
    SK_UNIT;
    if (f.component < 0)
      continue;
    for (std::uint32_t v : f.points)
      point_comps[v].push_back(f.component);
  }
  for (std::vector<int> &c : point_comps) {
    std::sort(c.begin(), c.end());
    c.erase(std::unique(c.begin(), c.end()), c.end());
  }
  auto same_comp = [&](std::uint32_t a, std::uint32_t b) {
    const std::vector<int> &ca = point_comps[a], &cb = point_comps[b];
    std::size_t i = 0, j = 0;
    while (i < ca.size() && j < cb.size()) {
      if (ca[i] == cb[j])
        return true;
      if (ca[i] < cb[j])
        i++;
      else
        j++;
    }
    return false;
  };
  std::vector<char> is_candidate(s.points.size(), 0);
  for (const secondary_skeleton::edge &e : s.edges) {
    SK_UNIT;
    if (e.faces >= 1 || same_comp(e.a, e.b))
      continue;
    r.helix_widths.push_back(e.width);
    if (e.width < p.helix_min_width || e.width > p.helix_max_width)
      continue;
    is_candidate[e.a] = is_candidate[e.b] = 1;
  }
  std::vector<std::uint32_t> cand;
  for (std::size_t i = 0; i < s.points.size(); i++)
    if (is_candidate[i])
      cand.push_back(std::uint32_t(i));
  std::stable_sort(cand.begin(), cand.end(), [&](std::uint32_t a, std::uint32_t b) {
    const pt3 &pa = s.points[a], &pb = s.points[b];
    return std::tie(pa[0], pa[1], pa[2]) < std::tie(pb[0], pb[1], pb[2]);
  });
  std::vector<pt3> pos;
  pos.reserve(cand.size());
  for (std::uint32_t c : cand)
    pos.push_back(s.points[c]);
  std::vector<std::vector<int>> runs;
  SK_RUN(cluster(pos, p.helix_join_distance, runs));
  std::vector<axis> axes;
  for (const std::vector<int> &run : runs) {
    SK_UNIT;
    axis ax = fit_axis(pos, run);
    if (std::sqrt(ax.sq_length) < p.helix_min_length)
      continue;
    axes.push_back(std::move(ax));
  }
  std::stable_sort(axes.begin(), axes.end(),
                   [](const axis &a, const axis &b) { return a.sq_length > b.sq_length; });
  r.helix_candidates = int(axes.size());
  const std::size_t nhelices = std::size_t(std::max(0, std::min(p.helix_count, int(axes.size()))));

  // ---- the output ----
  if (stage)
    stage("output", nhelices + s.edges.size() + 1);
  {
    geometry &g = r.helix_geometry;
    const geometry::color_t magenta{{0.99, 0.0, 0.7}};
    for (std::size_t h = 0; h < nhelices; h++) {
      SK_UNIT;
      const axis &ax = axes[h];
      secondary_helix hx;
      hx.start = ax.left;
      hx.end = ax.right;
      hx.length = std::sqrt(ax.sq_length);
      hx.radius = p.helix_radius;
      for (int k : ax.run)
        hx.points.push_back(pos[std::size_t(k)]);
      r.helices.push_back(std::move(hx));
      const auto start = geometry::index_t(g.points().size());
      for (const pt3 &q : cylinder_points(ax.left, ax.right, p.helix_radius)) {
        g.points().push_back(geometry::point_t{{q[0], q[1], q[2]}});
        g.colors().push_back(magenta);
      }
      const geometry::index_t lower_cent = start + 2 * SUBDIV, upper_cent = start + 2 * SUBDIV + 1;
      for (int i = 0; i < SUBDIV; i++) {
        const int next_i = (i + 1) % SUBDIV;
        const geometry::index_t lower_i = start + geometry::index_t(i),
                                lower_next = start + geometry::index_t(next_i),
                                upper_i = start + geometry::index_t(i + SUBDIV),
                                upper_next = start + geometry::index_t(next_i + SUBDIV);
        g.tris().push_back(geometry::tri_t{{lower_i, lower_next, upper_i}});
        g.tris().push_back(geometry::tri_t{{lower_next, upper_next, upper_i}});
        g.tris().push_back(geometry::tri_t{{lower_i, lower_cent, lower_next}});
        g.tris().push_back(geometry::tri_t{{upper_i, upper_next, upper_cent}});
        for (int k = 0; k < 4; k++)
          r.helix_of_tri.push_back(int(h));
      }
    }
  }
  {
    geometry &g = r.skeleton_geometry;
    const geometry::color_t red{{1.0, 0.0, 0.0}};
    for (const pt3 &q : s.points) {
      g.points().push_back(geometry::point_t{{q[0], q[1], q[2]}});
      g.colors().push_back(red);
    }
    for (const secondary_skeleton::edge &e : s.edges) {
      SK_UNIT;
      g.lines().push_back(geometry::line_t{{e.a, e.b}});
    }
  }
}

} // namespace cvc::secondary_detail

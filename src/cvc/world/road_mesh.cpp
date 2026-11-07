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

// road_mesh.cpp — see road_mesh.h. GL-free; no RNG (a pure function of the network).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/world/road_mesh.h>
#include <vector>

namespace cvc {
namespace world {
namespace {

struct P2 {
  double x, y;
};

struct builder {
  cvc::geometry &g;
  std::uint64_t base() const { return g.const_points().size(); }
  void vert(double x, double y, double z, const double rgb[3]) {
    g.points().push_back(cvc::geometry::point_t{{x, y, z}});
    g.normals().push_back(cvc::geometry::normal_t{{0.0, 0.0, 1.0}}); // roads lie ~flat
    g.colors().push_back(cvc::geometry::color_t{{rgb[0], rgb[1], rgb[2]}});
  }
  void tri(std::uint64_t a, std::uint64_t b, std::uint64_t c) {
    g.tris().push_back(cvc::geometry::tri_t{{a, b, c}});
  }
};

// A run of segments of one width and kind between two ends (a junction, a dead end, a change of
// width) or round a loop: drawn as one strip, so a bend has no gap or overlap.
struct chain {
  std::vector<int> nodes;
  double width = 0.0;
  bool arterial = false;
  bool closed = false;
};

std::vector<chain> chains_of(const road_network &net, const std::vector<int> &deg,
                             std::vector<char> &end) {
  const int N = int(net.nodes.size());
  struct Arc {
    int seg, to;
  };
  std::vector<std::vector<Arc>> adj(N);
  for (int i = 0; i < int(net.segments.size()); ++i) {
    const road_segment &s = net.segments[i];
    if (s.a < 0 || s.b < 0 || s.a >= N || s.b >= N || s.a == s.b)
      continue;
    adj[s.a].push_back({i, s.b});
    adj[s.b].push_back({i, s.a});
  }
  // A node ends a chain unless exactly two roads of one width and kind pass through it.
  end.assign(N, 0);
  for (int v = 0; v < N; ++v) {
    if (deg[v] != 2) {
      end[v] = deg[v] > 0;
      continue;
    }
    const road_segment &s0 = net.segments[adj[v][0].seg], &s1 = net.segments[adj[v][1].seg];
    end[v] = s0.width != s1.width || s0.arterial != s1.arterial;
  }
  std::vector<char> used(net.segments.size(), 0);
  std::vector<chain> out;
  auto walk = [&](int from, const Arc &first) {
    chain c;
    c.width = net.segments[first.seg].width;
    c.arterial = net.segments[first.seg].arterial;
    c.nodes.push_back(from);
    Arc at = first;
    while (true) {
      used[at.seg] = 1;
      c.nodes.push_back(at.to);
      if (end[at.to] || at.to == from)
        break;
      const Arc *next = nullptr;
      for (const Arc &a : adj[at.to])
        if (!used[a.seg])
          next = &a;
      if (!next)
        break;
      at = *next;
    }
    c.closed = c.nodes.front() == c.nodes.back() && c.nodes.size() > 3;
    out.push_back(std::move(c));
  };
  for (int v = 0; v < N; ++v)
    if (end[v])
      for (const Arc &a : adj[v])
        if (!used[a.seg])
          walk(v, a);
  for (int v = 0; v < N; ++v) // loops with no end on them
    for (const Arc &a : adj[v])
      if (!used[a.seg])
        walk(v, a);
  return out;
}

// Chaikin corner cutting: each pass replaces a corner by two points a quarter of the way along its
// edges, so the bends of a stepped road round off. An open line keeps its two ends.
std::vector<P2> rounded(std::vector<P2> pts, bool closed, int passes) {
  for (int k = 0; k < passes && pts.size() >= 3; ++k) {
    std::vector<P2> q;
    const std::size_t edges = pts.size() - 1; // a closed line repeats its first point last
    if (!closed)
      q.push_back(pts.front());
    for (std::size_t i = 0; i < edges; ++i) {
      const P2 a = pts[i], b = pts[i + 1];
      if (closed || i > 0)
        q.push_back({0.75 * a.x + 0.25 * b.x, 0.75 * a.y + 0.25 * b.y});
      if (closed || i + 1 < edges)
        q.push_back({0.25 * a.x + 0.75 * b.x, 0.25 * a.y + 0.75 * b.y});
    }
    if (closed)
      q.push_back(q.front());
    else
      q.push_back(pts.back());
    pts.swap(q);
  }
  return pts;
}

// The line with `from` metres cut off its start and `to` off its end (empty if nothing is left).
std::vector<P2> trimmed(const std::vector<P2> &pts, double from, double to) {
  std::vector<double> s(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); ++i)
    s[i] = s[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  const double total = pts.empty() ? 0.0 : s.back();
  const double a = from, b = total - to;
  std::vector<P2> out;
  if (b - a < 0.5)
    return out;
  auto at = [&](double d) {
    const std::size_t i = std::min<std::size_t>(
        pts.size() - 2, std::size_t(std::upper_bound(s.begin(), s.end(), d) - s.begin()) - 1);
    const double seg = s[i + 1] - s[i], u = seg > 1e-12 ? (d - s[i]) / seg : 0.0;
    return P2{pts[i].x + (pts[i + 1].x - pts[i].x) * u, pts[i].y + (pts[i + 1].y - pts[i].y) * u};
  };
  out.push_back(at(a));
  for (std::size_t i = 0; i < pts.size(); ++i)
    if (s[i] > a && s[i] < b)
      out.push_back(pts[i]);
  out.push_back(at(b));
  return out;
}

// A strip of half width hw along the line, mitred at each bend (the miter held to twice the half
// width on a hairpin), at z(x, y).
template <class Z>
void strip(builder &b, const std::vector<P2> &pts, bool closed, double hw, const double rgb[3],
           const Z &z) {
  const std::size_t n = pts.size();
  if (n < 2)
    return;
  auto dir = [&](std::size_t i, std::size_t j) {
    const double dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y, l = std::hypot(dx, dy);
    return l > 1e-12 ? P2{dx / l, dy / l} : P2{0.0, 0.0};
  };
  const std::uint64_t v0 = b.base();
  for (std::size_t i = 0; i < n; ++i) {
    P2 tin{0, 0}, tout{0, 0};
    if (i > 0)
      tin = dir(i - 1, i);
    else if (closed)
      tin = dir(n - 2, n - 1);
    if (i + 1 < n)
      tout = dir(i, i + 1);
    else if (closed)
      tout = dir(0, 1);
    P2 t{tin.x + tout.x, tin.y + tout.y};
    const double tl = std::hypot(t.x, t.y);
    t = tl > 1e-12 ? P2{t.x / tl, t.y / tl} : (i > 0 ? tin : tout);
    const P2 nrm{-t.y, t.x};
    // The miter: as far out as keeps the edges parallel to both neighbouring pieces.
    const P2 side = i > 0 ? tin : tout;
    const double cosHalf = std::abs(nrm.x * -side.y + nrm.y * side.x);
    const double k = hw / std::max(0.5, cosHalf);
    const double lx = pts[i].x + nrm.x * k, ly = pts[i].y + nrm.y * k;
    const double rx = pts[i].x - nrm.x * k, ry = pts[i].y - nrm.y * k;
    b.vert(lx, ly, z(lx, ly), rgb);
    b.vert(rx, ry, z(rx, ry), rgb);
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const std::uint64_t l0 = v0 + 2 * i, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    b.tri(l0, r0, l1);
    b.tri(l1, r0, r1);
  }
}

} // namespace

cvc::geometry extrude_roads(const road_network &net, const road_mesh_params &p,
                            const road_height_fn &height) {
  cvc::geometry g;
  if (net.segments.empty())
    return g;
  builder b{g};
  const int N = int(net.nodes.size());
  auto ground = [&](double x, double y) { return height ? height(x, y) : 0.0; };
  auto road_z = [&](double x, double y) { return ground(x, y) + p.lift; };

  std::vector<int> deg(N, 0);
  std::vector<double> node_hw(N, 0.0);
  std::vector<char> node_art(N, 0);
  for (const road_segment &s : net.segments) {
    if (s.a < 0 || s.b < 0 || s.a >= N || s.b >= N || s.a == s.b)
      continue;
    for (int e : {s.a, s.b}) {
      ++deg[e];
      node_hw[e] = std::max(node_hw[e], 0.5 * s.width);
      if (s.arterial)
        node_art[e] = 1;
    }
  }
  std::vector<char> end;
  const std::vector<chain> runs = chains_of(net, deg, end);

  // Each run as one strip, its bends rounded; then its centre stripe, stopped short of the pad at
  // either end so it never crosses a junction.
  const double stripeZ = p.lift + p.centerline_lift;
  for (const chain &c : runs) {
    std::vector<P2> pts;
    for (int v : c.nodes)
      pts.push_back({net.nodes[v].x, net.nodes[v].y});
    pts = rounded(std::move(pts), c.closed, std::max(0, p.smooth_passes));
    const double *rgb = c.arterial ? p.arterial_rgb : p.local_rgb;
    // Into a pad, the strip stops where its edges meet the pad's rim, so its corners sit on the
    // rim and the pad fills the rest: no corner pokes out past a junction.
    const double hw = 0.5 * c.width;
    auto into_pad = [&](int v) {
      if (c.closed || !p.intersections || !end[v])
        return 0.0;
      const double r = node_hw[v];
      return std::sqrt(std::max(0.0, r * r - hw * hw));
    };
    std::vector<P2> body = trimmed(pts, into_pad(c.nodes.front()), into_pad(c.nodes.back()));
    if (body.empty())
      body = pts;
    strip(b, body, c.closed, hw, rgb, road_z);
    if (!p.centerline || c.width < p.centerline_min_width)
      continue;
    auto stop = [&](int v) {
      return c.closed ? 0.0 : (deg[v] >= 3 ? node_hw[v] + 1.0 : deg[v] == 1 ? 1.0 : 0.0);
    };
    const std::vector<P2> line = trimmed(pts, stop(c.nodes.front()), stop(c.nodes.back()));
    strip(b, line, false, 0.5 * p.centerline_width, p.centerline_rgb,
          [&](double x, double y) { return ground(x, y) + stripeZ; });
  }

  // A round pad at each junction and dead end (and where a road changes width), sized to the
  // widest road there and a hair above the strips it covers, so the strips' ends never show.
  if (p.intersections && p.pad_segments >= 3) {
    const double TWO_PI = 6.283185307179586476925286766559;
    for (int i = 0; i < N; ++i) {
      if (!end[i] || node_hw[i] <= 0.0)
        continue;
      const road_node &nd = net.nodes[i];
      const double *rgb = node_art[i] ? p.arterial_rgb : p.local_rgb;
      const double r = node_hw[i];
      auto pad_z = [&](double x, double y) { return road_z(x, y) + p.pad_lift; };
      const std::uint64_t c = b.base();
      b.vert(nd.x, nd.y, pad_z(nd.x, nd.y), rgb); // centre
      for (int k = 0; k < p.pad_segments; ++k) {
        const double a = TWO_PI * double(k) / double(p.pad_segments);
        const double x = nd.x + r * std::cos(a), y = nd.y + r * std::sin(a);
        b.vert(x, y, pad_z(x, y), rgb);
      }
      for (int k = 0; k < p.pad_segments; ++k) {
        const std::uint64_t r0 = c + 1 + k;
        const std::uint64_t r1 = c + 1 + (k + 1) % p.pad_segments;
        b.tri(c, r0, r1);
      }
    }
  }
  return g;
}

} // namespace world
} // namespace cvc

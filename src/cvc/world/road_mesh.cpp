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
#include <functional>
#include <vector>

namespace cvc {
namespace world {
namespace {

struct P2 {
  double x, y;
};

// A skirt: a wall from a road's edge down to below the ground (road_mesh_params::skirt_depth).
struct skirt {
  double depth = 0.0;              // below the ground; 0: none
  double rgb[3] = {0.0, 0.0, 0.0}; // its colour
  std::function<double(double, double)> ground;
};

struct builder {
  cvc::geometry &g;
  std::uint64_t base() const { return g.const_points().size(); }
  void vert(double x, double y, double z, const double rgb[3]) {
    vert(x, y, z, rgb, 0.0, 0.0, 1.0); // roads lie ~flat
  }
  void vert(double x, double y, double z, const double rgb[3], double nx, double ny, double nz) {
    g.points().push_back(cvc::geometry::point_t{{x, y, z}});
    g.normals().push_back(cvc::geometry::normal_t{{nx, ny, nz}});
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

// The line with points added along each piece longer than `step`, evenly, so none is longer.
std::vector<P2> densified(const std::vector<P2> &pts, double step) {
  if (step <= 0.0 || pts.size() < 2)
    return pts;
  std::vector<P2> out{pts.front()};
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double len = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    const int k = std::max(1, int(std::ceil(len / step - 1e-9)));
    for (int j = 1; j < k; ++j)
      out.push_back({pts[i - 1].x + (pts[i].x - pts[i - 1].x) * j / k,
                     pts[i - 1].y + (pts[i].y - pts[i - 1].y) * j / k});
    out.push_back(pts[i]);
  }
  return out;
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

// A wall along `top` (its points' x, y and road height), from there down to the skirt's depth
// below the ground, facing `out` (each point's outward horizontal normal).
struct edge_pt {
  double x, y, z, nx, ny; // nx, ny: the outward direction the wall faces (it winds by it)
};
P2 dir_of(const P2 &a, const P2 &b) {
  const double dx = b.x - a.x, dy = b.y - a.y, l = std::hypot(dx, dy);
  return l > 1e-12 ? P2{dx / l, dy / l} : P2{1.0, 0.0};
}
// A closed wall (a pad's rim, a loop road's edge) joins its last point back to its first rather
// than repeating it: a repeated vertex, alike in every attribute, is merged when a mesh is read
// back (assimp's JoinIdenticalVertices), and a world written again would not be the same bytes.
void wall(builder &b, std::vector<edge_pt> top, const skirt &s, bool closed = false) {
  if (closed && top.size() > 2 && top.front().x == top.back().x && top.front().y == top.back().y)
    top.pop_back();
  if (s.depth <= 0.0 || top.size() < 2)
    return;
  const std::uint64_t v0 = b.base();
  for (const edge_pt &e : top) {
    // Lit as the road's top is (its normal up), a shade darker: the side of a slab of pavement.
    // An up normal also comes back from a glb exactly, where a slanted one can move a bit each
    // time a mesh is written (assimp normalises normals as it writes them), and a world read back
    // must write the same bytes again.
    b.vert(e.x, e.y, e.z, s.rgb);
    b.vert(e.x, e.y, s.ground(e.x, e.y) - s.depth, s.rgb);
  }
  const std::size_t n = top.size(), pieces = closed ? n : n - 1;
  for (std::size_t i = 0; i < pieces; ++i) {
    const std::size_t j = (i + 1) % n;
    const std::uint64_t t0 = v0 + 2 * i, d0 = t0 + 1, t1 = v0 + 2 * j, d1 = t1 + 1;
    // Wound to face out: the edge runs along the road with the outward normal on its right.
    const double ex = top[j].x - top[i].x, ey = top[j].y - top[i].y;
    if (ex * top[i].ny - ey * top[i].nx >= 0.0) {
      b.tri(t0, t1, d0);
      b.tri(t1, d1, d0);
    } else {
      b.tri(t0, d0, t1);
      b.tri(t1, d0, d1);
    }
  }
}

// A strip of half width hw along the line, mitred at each bend (the miter held to twice the half
// width on a hairpin), at z(x, y). With a skirt, its two edges drop in walls to below the ground,
// and so do an open strip's two ends when `caps` (where no junction pad covers them).
template <class Z>
void strip(builder &b, const std::vector<P2> &pts, bool closed, double hw, const double rgb[3],
           const Z &z, const skirt &sk = {}, bool caps = false) {
  const std::size_t n = pts.size();
  if (n < 2)
    return;
  auto dir = [&](std::size_t i, std::size_t j) {
    const double dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y, l = std::hypot(dx, dy);
    return l > 1e-12 ? P2{dx / l, dy / l} : P2{0.0, 0.0};
  };
  const std::uint64_t v0 = b.base();
  std::vector<edge_pt> left, right;
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
    const double lz = z(lx, ly), rz = z(rx, ry);
    b.vert(lx, ly, lz, rgb);
    b.vert(rx, ry, rz, rgb);
    left.push_back({lx, ly, lz, nrm.x, nrm.y});
    right.push_back({rx, ry, rz, -nrm.x, -nrm.y});
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const std::uint64_t l0 = v0 + 2 * i, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    b.tri(l0, r0, l1);
    b.tri(l1, r0, r1);
  }
  if (sk.depth <= 0.0)
    return;
  wall(b, left, sk, closed);
  wall(b, right, sk, closed);
  if (!closed && caps) { // the two ends, facing back along the road and on along it
    const P2 a = dir_of(pts[0], pts[1]), e = dir_of(pts[n - 2], pts[n - 1]);
    wall(b,
         {{left.front().x, left.front().y, left.front().z, -a.x, -a.y},
          {right.front().x, right.front().y, right.front().z, -a.x, -a.y}},
         sk);
    wall(b,
         {{right.back().x, right.back().y, right.back().z, e.x, e.y},
          {left.back().x, left.back().y, left.back().z, e.x, e.y}},
         sk);
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
    skirt sk;
    if (p.skirt_depth > 0.0) {
      sk.depth = p.skirt_depth;
      for (int k = 0; k < 3; ++k)
        sk.rgb[k] = rgb[k] * p.skirt_shade;
      sk.ground = ground;
    }
    strip(b, densified(body, p.sample_step), c.closed, hw, rgb, road_z, sk,
          /*caps=*/!p.intersections || p.pad_segments < 3);
    if (!p.centerline || c.width < p.centerline_min_width)
      continue;
    auto stop = [&](int v) {
      return c.closed ? 0.0 : (deg[v] >= 3 ? node_hw[v] + 1.0 : deg[v] == 1 ? 1.0 : 0.0);
    };
    const std::vector<P2> line = trimmed(pts, stop(c.nodes.front()), stop(c.nodes.back()));
    strip(b, densified(line, p.sample_step), false, 0.5 * p.centerline_width, p.centerline_rgb,
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
      if (p.skirt_depth > 0.0) { // its rim drops to below the ground too, facing out
        skirt sk;
        sk.depth = p.skirt_depth;
        for (int k = 0; k < 3; ++k)
          sk.rgb[k] = rgb[k] * p.skirt_shade;
        sk.ground = ground;
        std::vector<edge_pt> rim;
        for (int k = 0; k < p.pad_segments; ++k) {
          const double a = TWO_PI * double(k) / double(p.pad_segments);
          const double x = nd.x + r * std::cos(a), y = nd.y + r * std::sin(a);
          rim.push_back({x, y, pad_z(x, y), std::cos(a), std::sin(a)});
        }
        wall(b, rim, sk, /*closed=*/true);
      }
    }
  }
  return g;
}

} // namespace world
} // namespace cvc

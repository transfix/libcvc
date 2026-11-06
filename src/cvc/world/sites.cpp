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

// sites.cpp — see sites.h. GL-free; hashed RNG only (no <random>, no stream state).

#include <cmath>
#include <cstdint>
#include <cvc/lsys/rng.h>
#include <cvc/world/sites.h>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace world {
namespace {

// An oriented rectangle: centre, unit along-axis u (half-extent hu) and perpendicular v
// (half-extent hv). Used for the cross-street no-overlap test (separating-axis theorem).
struct obb {
  double cx, cy, ux, uy, hu, hv; // v = (-uy, ux)
};

// Project obb onto a unit axis (ax, ay): returns the half-width of its shadow.
inline double proj_radius(const obb &o, double ax, double ay) {
  const double vx = -o.uy, vy = o.ux;
  return std::fabs((o.ux * ax + o.uy * ay)) * o.hu + std::fabs((vx * ax + vy * ay)) * o.hv;
}

// Do two oriented rectangles overlap? SAT over the four face normals (u1, v1, u2, v2). Touching
// (a shared wall) is not overlapping.
bool obb_overlap(const obb &a, const obb &b) {
  const double dx = b.cx - a.cx, dy = b.cy - a.cy;
  const double axes[4][2] = {{a.ux, a.uy}, {-a.uy, a.ux}, {b.ux, b.uy}, {-b.uy, b.ux}};
  for (auto &ax : axes) {
    const double sep = std::fabs(dx * ax[0] + dy * ax[1]);
    if (sep > proj_radius(a, ax[0], ax[1]) + proj_radius(b, ax[0], ax[1]) - 1e-6)
      return false; // a separating axis exists
  }
  return true;
}

// A stable lot id from its quantized position (metre grid). NEVER a loop counter — so adding or
// removing a road cannot move the archetype chosen for a lot on an untouched street.
inline std::uint64_t lot_id(double x, double y) {
  return cvc::lsys::cell_id(static_cast<std::int32_t>(std::lround(x)),
                            static_cast<std::int32_t>(std::lround(y)));
}

// A road is a polyline: a chain of points through degree-2 nodes, bounded by junctions (degree !=
// 2) or dead ends. Per-span width/arterial flag + cumulative arc length let us lay lots
// continuously.
struct polyline {
  std::vector<double> px, py; // vertices
  std::vector<double> span_w; // width of span i (px[i]..px[i+1])
  std::vector<char> span_art; // arterial flag of span i
  std::vector<double> cum;    // cum[i] = arc length up to vertex i (cum[0] = 0)
  std::vector<int> segs;      // the network segment of span i
  double length() const { return cum.empty() ? 0.0 : cum.back(); }
};

// Trace the network's segments into polylines, walking straight through degree-2 nodes and breaking
// at junctions / dead ends. Every segment lands in exactly one polyline (loops handled last).
std::vector<polyline> trace_polylines(const road_network &net) {
  const int N = int(net.nodes.size());
  std::vector<std::vector<std::pair<int, int>>> adj(N); // node -> (segment idx, other node)
  for (std::size_t si = 0; si < net.segments.size(); ++si) {
    const road_segment &s = net.segments[si];
    if (s.a < 0 || s.b < 0 || s.a >= N || s.b >= N)
      continue;
    adj[s.a].push_back({int(si), s.b});
    adj[s.b].push_back({int(si), s.a});
  }
  auto is_break = [&](int n) { return int(adj[n].size()) != 2; }; // junction or dead end

  std::vector<char> used(net.segments.size(), 0);
  std::vector<polyline> out;

  auto emit_span = [&](polyline &pl, int from, int seg, int to) {
    const road_segment &s = net.segments[seg];
    pl.px.push_back(net.nodes[to].x);
    pl.py.push_back(net.nodes[to].y);
    pl.span_w.push_back(s.width);
    pl.span_art.push_back(s.arterial ? 1 : 0);
    pl.segs.push_back(seg);
    const double dl =
        std::hypot(net.nodes[to].x - net.nodes[from].x, net.nodes[to].y - net.nodes[from].y);
    pl.cum.push_back(pl.cum.back() + dl);
  };

  // Chains rooted at every break node (junctions + dead ends).
  for (int j = 0; j < N; ++j) {
    if (!is_break(j))
      continue;
    for (auto &e : adj[j]) {
      if (used[e.first])
        continue;
      polyline pl;
      pl.px.push_back(net.nodes[j].x);
      pl.py.push_back(net.nodes[j].y);
      pl.cum.push_back(0.0);
      int cur = j, seg = e.first, nxt = e.second;
      while (true) {
        used[seg] = 1;
        emit_span(pl, cur, seg, nxt);
        cur = nxt;
        if (is_break(cur))
          break; // reached the next junction / dead end
        // continue along the OTHER segment at this degree-2 node
        const auto &a0 = adj[cur][0], &a1 = adj[cur][1];
        const std::pair<int, int> &go = (a0.first == seg) ? a1 : a0;
        if (used[go.first])
          break; // safety against a degenerate loop
        seg = go.first;
        nxt = go.second;
      }
      out.push_back(std::move(pl));
    }
  }
  // Pure loops (every node degree 2, no break): walk any remaining segment around to its start.
  for (std::size_t si = 0; si < net.segments.size(); ++si) {
    if (used[si])
      continue;
    const road_segment &s0 = net.segments[si];
    polyline pl;
    pl.px.push_back(net.nodes[s0.a].x);
    pl.py.push_back(net.nodes[s0.a].y);
    pl.cum.push_back(0.0);
    int start = s0.a, cur = s0.a, seg = int(si), nxt = s0.b;
    while (true) {
      used[seg] = 1;
      emit_span(pl, cur, seg, nxt);
      cur = nxt;
      if (cur == start || int(adj[cur].size()) != 2)
        break;
      const auto &a0 = adj[cur][0], &a1 = adj[cur][1];
      const std::pair<int, int> &go = (a0.first == seg) ? a1 : a0;
      if (used[go.first])
        break;
      seg = go.first;
      nxt = go.second;
    }
    out.push_back(std::move(pl));
  }
  return out;
}

} // namespace

std::vector<lot> layout_lots(const road_network &net, const site_params &sp,
                             const road_height_fn &height, const lot_density_fn &density) {
  std::vector<lot> out;
  if (net.segments.empty())
    return out;
  using cvc::lsys::stream;
  using cvc::lsys::uni;
  auto urbanity = [&](double x, double y) {
    return density ? std::max(0.0, std::min(1.0, density(x, y))) : 1.0;
  };

  // A coarse spatial hash of accepted lots (as OBBs) for the cross-segment overlap test.
  const double cell =
      std::max(8.0, std::max(sp.frontage * (1.0 + std::fabs(sp.frontage_jitter)), sp.lot_depth));
  std::unordered_map<std::uint64_t, std::vector<obb>> grid;
  auto key = [&](int cx, int cy) {
    return (std::uint64_t(std::uint32_t(cx)) << 32) | std::uint32_t(cy);
  };
  auto cell_of = [&](double x, double y, int &cx, int &cy) {
    cx = int(std::floor(x / cell));
    cy = int(std::floor(y / cell));
  };
  auto fits = [&](const obb &o) {
    int cx, cy;
    cell_of(o.cx, o.cy, cx, cy);
    for (int gx = cx - 1; gx <= cx + 1; ++gx)
      for (int gy = cy - 1; gy <= cy + 1; ++gy) {
        auto it = grid.find(key(gx, gy));
        if (it == grid.end())
          continue;
        for (const obb &e : it->second)
          if (obb_overlap(o, e))
            return false;
      }
    return true;
  };
  auto remember = [&](const obb &o) {
    int cx, cy;
    cell_of(o.cx, o.cy, cx, cy);
    grid[key(cx, cy)].push_back(o);
  };

  const double clear = std::max(0.0, sp.junction_clear);
  if (sp.frontage + sp.spacing <= 0.0 || sp.frontage <= 0.0)
    return out;

  const std::vector<polyline> roads = trace_polylines(net);
  std::vector<int> seg_road(net.segments.size(), -1); // network segment -> its polyline
  for (std::size_t ri = 0; ri < roads.size(); ++ri)
    for (int s : roads[ri].segs)
      seg_road[std::size_t(s)] = int(ri);

  // How far behind the setback line (at q, looking along (dx, dy)) the next OTHER road's edge is,
  // or max_block when none is that close.
  // Only a road whose edge the probe reaches counts, so each probe looks no further than the
  // widest road's half-width (a probe deep inside a big block would otherwise search far).
  double widest = 0.0;
  for (const road_segment &s : net.segments)
    widest = std::max(widest, s.width);
  const double reach = 0.5 * widest + 0.5;
  auto clear_behind = [&](double qx, double qy, double dx, double dy, int ri) {
    for (double s = 2.0; s <= sp.max_block; s += 2.0) {
      int si = -1;
      const double d = net.distance_within(qx + dx * s, qy + dy * s, reach, &si);
      if (si >= 0 && seg_road[std::size_t(si)] != ri && d <= 0.5 * net.segments[si].width + 0.5)
        return s;
    }
    return sp.max_block;
  };

  for (std::size_t ri = 0; ri < roads.size(); ++ri) {
    const polyline &pl = roads[ri];
    const double len = pl.length();
    if (len < sp.min_segment || pl.px.size() < 2)
      continue;
    // A stable id for the road (its two ends), for its lot widths and its sides' fill.
    const std::uint64_t road_id = lot_id(pl.px.front(), pl.py.front()) * 0x9E3779B97F4A7C15ull ^
                                  lot_id(pl.px.back(), pl.py.back());
    const double u_road = urbanity(pl.px[pl.px.size() / 2], pl.py[pl.py.size() / 2]);
    const double gap = sp.spacing * (1.0 - sp.core_spacing * u_road);

    // Keep both ends of the road clear of their junctions, then pack frontage slots (jittered
    // widths) down the middle.
    const double usable = len - 2.0 * clear;
    if (usable < sp.frontage * (1.0 - std::fabs(sp.frontage_jitter)))
      continue;
    std::vector<double> widths;
    double total = 0.0;
    for (std::uint32_t k = 0; k < 4096; ++k) {
      const double j =
          sp.frontage_jitter == 0.0
              ? 0.0
              : sp.frontage_jitter * uni(sp.seed, stream::floorplan, road_id, k, -1, 1);
      const double f = sp.frontage * (1.0 + j);
      const double next = total + (widths.empty() ? 0.0 : gap) + f;
      if (next > usable + 1e-9)
        break;
      widths.push_back(f);
      total = next;
    }
    if (widths.empty())
      continue;
    const double start = clear + 0.5 * (usable - total); // the first lot's near edge

    const int sides = sp.both_sides ? 2 : 1;
    // A street side is built or left empty as a whole (sparse_fill at urbanity 0).
    bool built[2] = {true, true};
    for (int side_i = 0; side_i < sides; ++side_i)
      built[side_i] = uni(sp.seed, stream::floorplan, road_id, 100000u + side_i) <
                      sp.sparse_fill + (1.0 - sp.sparse_fill) * u_road;

    double edge = start;
    for (std::size_t i = 0; i < widths.size(); ++i) {
      const double f = widths[i];
      const double arc = edge + 0.5 * f;
      edge += f + gap;
      // Locate the span containing this arc length and interpolate point + tangent + width.
      std::size_t sp_i = 0;
      while (sp_i + 1 < pl.cum.size() && pl.cum[sp_i + 1] < arc)
        ++sp_i;
      if (sp_i + 1 >= pl.px.size())
        continue;
      const double segLen = pl.cum[sp_i + 1] - pl.cum[sp_i];
      const double t = segLen > 1e-9 ? (arc - pl.cum[sp_i]) / segLen : 0.0;
      const double ax = pl.px[sp_i], ay = pl.py[sp_i];
      const double bx = pl.px[sp_i + 1], by = pl.py[sp_i + 1];
      const double px = ax + (bx - ax) * t, py = ay + (by - ay) * t;
      double ux = bx - ax, uy = by - ay;
      const double ul = std::hypot(ux, uy);
      if (ul < 1e-9)
        continue;
      ux /= ul;
      uy /= ul;
      const double nx = -uy, ny = ux;          // + side perpendicular
      const double hw = 0.5 * pl.span_w[sp_i]; // local road half-width
      const bool arterial = pl.span_art[sp_i] != 0;
      const double u = urbanity(px, py);
      const double setback = sp.setback * (1.0 - sp.core_setback * u);

      for (int side_i = 0; side_i < sides; ++side_i) {
        if (!built[side_i])
          continue;
        const double side = side_i == 0 ? 1.0 : -1.0;
        double depth = sp.lot_depth;
        if (sp.block_depth) {
          // Back onto the lots of the road behind: stop at the middle of the block.
          const double qx = px + nx * side * (hw + setback), qy = py + ny * side * (hw + setback);
          const double c = clear_behind(qx, qy, nx * side, ny * side, int(ri));
          depth = std::min(depth, 0.5 * (c - setback) - 0.5);
          if (depth < sp.min_depth)
            continue;
        }
        const double offset = hw + setback + 0.5 * depth;
        const double cx = px + nx * side * offset;
        const double cy = py + ny * side * offset;

        // Keep the whole footprint off every road (room from the kerb + clear of crossing streets):
        // test the centre and the road-ward front-edge midpoint.
        const double frontx = -side * nx, fronty = -side * ny; // unit vector toward the road
        const double fex = cx + frontx * 0.5 * depth;
        const double fey = cy + fronty * 0.5 * depth;
        if (net.on_or_near_road(cx, cy, sp.road_margin) ||
            net.on_or_near_road(fex, fey, sp.road_margin))
          continue;

        // The ground under the footprint: its centre, corners and edge midpoints.
        double gmin = 0.0, gmax = 0.0;
        if (height) {
          gmin = 1e30;
          gmax = -1e30;
          for (int a = -1; a <= 1; ++a)
            for (int b = -1; b <= 1; ++b) {
              const double sx = cx + ux * a * 0.5 * f + nx * b * 0.5 * depth;
              const double sy = cy + uy * a * 0.5 * f + ny * b * 0.5 * depth;
              const double h = height(sx, sy);
              gmin = std::min(gmin, h);
              gmax = std::max(gmax, h);
            }
          if ((sp.max_relief > 0.0 && gmax - gmin > sp.max_relief) || gmin < sp.min_ground)
            continue;
        }

        // local +Y (the building front) points toward the road: rotation by yaw maps
        // (0,1) -> (-sin, cos) = (frontx, fronty)  =>  yaw = atan2(-frontx, fronty).
        const double yaw = std::atan2(-frontx, fronty);

        obb o{cx, cy, ux, uy, 0.5 * f, 0.5 * depth};
        if (!fits(o))
          continue;
        remember(o);

        lot L;
        L.x = cx;
        L.y = cy;
        L.yaw = yaw;
        L.frontage = f;
        L.depth = depth;
        L.seg = int(ri);
        L.arterial = arterial;
        L.side = int(side);
        L.urbanity = u;
        L.has_ground = bool(height);
        L.ground_min = gmin;
        L.ground_max = gmax;
        out.push_back(L);
      }
    }
  }
  return out;
}

std::vector<building_instance> fit_buildings(const building_library &lib,
                                             const std::vector<lot> &lots, const fit_params &fp,
                                             const road_height_fn &height) {
  std::vector<building_instance> out;
  if (lib.empty())
    return out;
  out.reserve(lots.size());
  using cvc::lsys::irand;
  using cvc::lsys::stream;

  for (const lot &L : lots) {
    const std::uint64_t id = lot_id(L.x, L.y);
    const double usable_front = std::max(1e-3, L.frontage - 2.0 * fp.margin);
    const double usable_depth = std::max(1e-3, L.depth - 2.0 * fp.margin);
    // The scale at which archetype `i`'s footprint fits the lot (longer axis along the frontage),
    // or 0 for a degenerate one.
    auto fit_of = [&](int i) {
      const building_archetype &a = lib.archetypes[i];
      const double fw = 2.0 * a.footprint_x, fd = 2.0 * a.footprint_y;
      if (fw <= 1e-6 || fd <= 1e-6)
        return 0.0;
      return std::min(usable_front / std::max(fw, fd), usable_depth / std::min(fw, fd));
    };
    int idx = irand(fp.seed, stream::building, id, 0, 0, int(lib.size()) - 1);
    double scale = fit_of(idx);
    if (fp.true_scale) {
      // The first draw that fits near its own size, at most its own size.
      int found = -1;
      for (int k = 0; k < std::max(1, fp.candidates) && found < 0; ++k) {
        const int i =
            k == 0 ? idx : irand(fp.seed, stream::building, id, k, 0, int(lib.size()) - 1);
        if (fit_of(i) >= fp.min_true)
          found = i;
      }
      if (found < 0)
        continue; // nothing fits this lot at its real size: leave it empty
      idx = found;
      scale = std::min(1.0, fit_of(idx));
    } else {
      if (scale <= 0.0)
        continue;
      if (scale < fp.min_scale)
        scale = fp.min_scale;
      if (scale > fp.max_scale)
        scale = fp.max_scale;
    }

    // Orient the longer footprint axis ALONG the frontage. lot.yaw already aligns local +X with the
    // frontage, so if the archetype's long axis is its local X we keep yaw; otherwise turn 90
    // degrees.
    const building_archetype &a = lib.archetypes[idx];
    double yaw = L.yaw;
    if (a.footprint_y > a.footprint_x)
      yaw += M_PI / 2.0;

    const double z = L.has_ground ? L.ground_min : height ? height(L.x, L.y) : 0.0;
    building_instance bi;
    bi.archetype = idx;
    bi.x = L.x;
    bi.y = L.y;
    bi.z = z;
    bi.yaw = yaw;
    bi.scale = scale;
    out.push_back(bi);
  }
  return out;
}

} // namespace world
} // namespace cvc

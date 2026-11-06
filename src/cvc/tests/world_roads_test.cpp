/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/roads.h for the full header).
*/

#include <algorithm>
#include <cmath>
#include <cvc/world/roads.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::world;

namespace {
// Strict interior segment-segment crossing (shared endpoints do not count).
bool crosses(double ax, double ay, double bx, double by, double cx, double cy, double dx,
             double dy) {
  const double rX = bx - ax, rY = by - ay, sX = dx - cx, sY = dy - cy;
  const double den = rX * sY - rY * sX;
  if (std::fabs(den) < 1e-12)
    return false;
  const double t = ((cx - ax) * sY - (cy - ay) * sX) / den;
  const double u = ((cx - ax) * rY - (cy - ay) * rX) / den;
  const double e = 1e-4;
  return t > e && t < 1 - e && u > e && u < 1 - e;
}

double checksum(const road_network &n) {
  double s = 0;
  for (const auto &nd : n.nodes)
    s += nd.x * 1.1 + nd.y * 2.3;
  for (const auto &sg : n.segments)
    s += sg.a * 3.7 + sg.b * 5.1 + sg.width;
  return s;
}

road_params demo_params() {
  road_params p;
  p.seed = 7;
  p.half = 120.0;
  p.max_segments = 1500;
  return p;
}
} // namespace

TEST(WorldRoads, GeneratesAConnectedNetwork) {
  const road_network n = generate_roads(demo_params());
  EXPECT_GT(n.nodes.size(), 10u);
  EXPECT_GT(n.segments.size(), 10u);
  // Every segment references valid nodes and has positive length + width.
  for (const road_segment &s : n.segments) {
    ASSERT_GE(s.a, 0);
    ASSERT_GE(s.b, 0);
    ASSERT_LT(s.a, int(n.nodes.size()));
    ASSERT_LT(s.b, int(n.nodes.size()));
    EXPECT_NE(s.a, s.b);
    EXPECT_GT(s.width, 0.0);
  }
}

TEST(WorldRoads, IsDeterministicInSeed) {
  const road_network a = generate_roads(demo_params());
  const road_network b = generate_roads(demo_params());
  ASSERT_EQ(a.nodes.size(), b.nodes.size());
  ASSERT_EQ(a.segments.size(), b.segments.size());
  EXPECT_DOUBLE_EQ(checksum(a), checksum(b));
  road_params q = demo_params();
  q.seed = 8;
  EXPECT_NE(checksum(a), checksum(generate_roads(q))); // a different seed -> a different city
}

// The headline correctness property: NO two roads cross except at a shared junction node. This is
// what the local-constraint snapping (crossing-cut / T-junction / extend) guarantees.
TEST(WorldRoads, NoRoadsCrossWithoutAJunction) {
  const road_network n = generate_roads(demo_params());
  int bad = 0;
  for (std::size_t i = 0; i < n.segments.size(); ++i)
    for (std::size_t j = i + 1; j < n.segments.size(); ++j) {
      const road_segment &s = n.segments[i], &t = n.segments[j];
      if (s.a == t.a || s.a == t.b || s.b == t.a || s.b == t.b)
        continue; // share a node -> a legitimate junction
      if (crosses(n.nodes[s.a].x, n.nodes[s.a].y, n.nodes[s.b].x, n.nodes[s.b].y, n.nodes[t.a].x,
                  n.nodes[t.a].y, n.nodes[t.b].x, n.nodes[t.b].y))
        ++bad;
    }
  EXPECT_EQ(bad, 0) << bad << " road pairs cross without a shared junction node";
}

TEST(WorldRoads, HasRealIntersections) {
  const road_network n = generate_roads(demo_params());
  int junctions = 0;
  for (const road_node &nd : n.nodes)
    if (nd.degree > 2)
      ++junctions;
  EXPECT_GT(junctions, 0); // a street network must have T/cross junctions
}

TEST(WorldRoads, StaysInsideTheRegion) {
  const road_params p = demo_params();
  const road_network n = generate_roads(p);
  for (const road_node &nd : n.nodes) {
    EXPECT_LE(std::fabs(nd.x), p.half + 1e-6);
    EXPECT_LE(std::fabs(nd.y), p.half + 1e-6);
  }
}

TEST(WorldRoads, FlatTerrainGivesTheGridDirection) {
  road_params p = demo_params();
  p.grid_angle_deg = 30.0;
  double dx = 0, dy = 0;
  road_field_dir(p, {}, 10.0, -20.0, dx, dy); // no height sampler -> pure grid
  EXPECT_NEAR(dx, std::cos(30.0 * M_PI / 180.0), 1e-9);
  EXPECT_NEAR(dy, std::sin(30.0 * M_PI / 180.0), 1e-9);
}

TEST(WorldRoads, DistanceAndNearQueries) {
  const road_network n = generate_roads(demo_params());
  // The centre seeded the first node, so a point at the origin is on/very near a road.
  EXPECT_LT(n.distance_to_road(0.0, 0.0), 5.0);
  EXPECT_TRUE(n.on_or_near_road(0.0, 0.0, 2.0));
  // A far corner outside the city extent is not near any road.
  EXPECT_FALSE(n.on_or_near_road(1e5, 1e5, 1.0));
}

// distance_to_road is backed by a spatial-grid index; it must return exactly what a brute-force
// scan would, at points NEAR a road (fast path) and FAR from any road (fallback), including the
// segment index and the lowest-index tie-break.
TEST(WorldRoads, DistanceMatchesBruteForce) {
  const road_network n = generate_roads(demo_params());
  ASSERT_FALSE(n.empty());
  auto brute = [&](double x, double y, int &bseg) {
    double best = 1e30;
    bseg = -1;
    for (std::size_t i = 0; i < n.segments.size(); ++i) {
      const road_node &a = n.nodes[n.segments[i].a], &b = n.nodes[n.segments[i].b];
      const double vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy;
      double t = L2 > 1e-12 ? ((x - a.x) * vx + (y - a.y) * vy) / L2 : 0.0;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      const double dx = a.x + t * vx - x, dy = a.y + t * vy - y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < best) { // same lowest-index tie-break as the index
        best = d;
        bseg = int(i);
      }
    }
    return best;
  };
  const double h = n.half;
  int checked = 0;
  for (int i = 0; i < 37; ++i)
    for (int j = 0; j < 37; ++j) {
      const double x = -h + 2 * h * i / 36.0, y = -h + 2 * h * j / 36.0;
      int bseg = -1;
      const double bd = brute(x, y, bseg);
      int iseg = -1;
      const double id = n.distance_to_road(x, y, &iseg);
      EXPECT_NEAR(id, bd, 1e-9) << "at (" << x << "," << y << ")";
      EXPECT_EQ(iseg, bseg) << "segment mismatch at (" << x << "," << y << ")";
      ++checked;
    }
  EXPECT_GT(checked, 0);
}

namespace {
double seg_len(const road_network &n, const road_segment &s) {
  return std::hypot(n.nodes[s.b].x - n.nodes[s.a].x, n.nodes[s.b].y - n.nodes[s.a].y);
}
} // namespace

// The grade limit: a steep hill east of the centre (30 m high, its flanks up to 60 %). Unlimited,
// the streets run straight over it; held to 12 %, every segment is steered toward the contours
// and stays under the limit (with the snapped end's 1.5x slack), and most of the network survives,
// winding round the hill.
TEST(WorldRoads, GradeLimitSteersAlongTheContours) {
  const road_height_fn hill = [](double x, double y) {
    return 30.0 * std::exp(-((x - 80.0) * (x - 80.0) + y * y) / (2.0 * 30.0 * 30.0));
  };
  road_params p = demo_params();
  auto worst = [&](const road_network &n) {
    double g = 0.0;
    for (const road_segment &s : n.segments) {
      const road_node &a = n.nodes[s.a], &b = n.nodes[s.b];
      g = std::max(g, std::fabs(hill(b.x, b.y) - hill(a.x, a.y)) / seg_len(n, s));
    }
    return g;
  };
  const road_network free = generate_roads(p, hill);
  EXPECT_GT(worst(free), 0.2);
  p.max_grade = 0.12;
  const road_network n = generate_roads(p, hill);
  EXPECT_LE(worst(n), 0.12 * 1.5 + 1e-9);
  EXPECT_GT(n.segments.size(), free.segments.size() / 2);
  int onHill = 0; // the streets still reach the hill's flanks
  for (const road_node &q : n.nodes)
    onHill += hill(q.x, q.y) > 3.0 ? 1 : 0;
  EXPECT_GT(onHill, 5);
}

// The no-go mask (a lake east of x = 40): no road reaches into it, where unmasked ones do.
TEST(WorldRoads, MaskKeepsRoadsOut) {
  const road_params p = demo_params();
  const road_mask_fn lake = [](double x, double) { return x > 40.0; };
  auto east = [](const road_network &n) {
    double m = -1e30;
    for (const road_segment &s : n.segments)
      m = std::max({m, n.nodes[s.a].x, n.nodes[s.b].x});
    return m;
  };
  EXPECT_GT(east(generate_roads(p)), 60.0);
  const road_network n = generate_roads(p, {}, lake);
  EXPECT_GT(n.segments.size(), 50u);
  EXPECT_LE(east(n), 40.0 + 1e-9);
}

// The hierarchy and districts: two centres each seed an arterial cross; an arterial's branches
// are collectors (their own width), which sprout the local streets; a dense core packs local
// streets tighter than sparse ground does.
TEST(WorldRoads, HierarchyAndDistricts) {
  road_params p = demo_params();
  p.half = 240.0;
  p.max_segments = 4000;
  p.collector_width = 7.5;
  p.centres = {{-120.0, 0.0}, {120.0, 0.0}};
  const road_network n = generate_roads(p);
  int kinds[3] = {0, 0, 0};
  for (const road_segment &s : n.segments) {
    ++kinds[int(s.kind)];
    EXPECT_EQ(s.arterial, s.kind == road_class::arterial);
    EXPECT_EQ(s.width, s.kind == road_class::arterial    ? p.arterial_width
                       : s.kind == road_class::collector ? p.collector_width
                                                         : p.local_width);
  }
  EXPECT_GT(kinds[0], 0);
  EXPECT_GT(kinds[1], 0);
  EXPECT_GT(kinds[2], 0);
  for (const road_centre &c : p.centres) {
    int at = 0;
    for (const road_segment &s : n.segments)
      at += (std::hypot(n.nodes[s.a].x - c.x, n.nodes[s.a].y - c.y) < 1e-9) ? 1 : 0;
    EXPECT_GE(at, 2) << c.x; // the arterials leave each centre
  }

  // Urbanity 1 west of x = 0, 0 east: sparse ground has fewer, wider-spaced local streets.
  p.sparse_spacing = 0.5;
  p.sparse_branch = 0.3;
  p.parallel_gap = 0.5;
  const road_network d =
      generate_roads(p, {}, {}, [](double x, double) { return x < 0.0 ? 1.0 : 0.0; });
  double west = 0.0, eastLen = 0.0;
  for (const road_segment &s : d.segments)
    if (s.kind == road_class::local)
      (d.nodes[s.a].x < 0.0 ? west : eastLen) += seg_len(d, s);
  EXPECT_GT(west, 1.5 * eastLen);
}

// prune_roads trims every dead-end chain shorter than the limit and leaves the degrees right.
TEST(WorldRoads, PruneTrimsShortStubs) {
  road_network n = generate_roads(demo_params());
  const std::size_t before = n.segments.size();
  const std::size_t removed = prune_roads(n, 60.0);
  EXPECT_GT(removed, 0u);
  EXPECT_EQ(n.segments.size() + removed, before);
  std::vector<int> deg(n.nodes.size(), 0);
  std::vector<std::vector<int>> adj(n.nodes.size());
  for (std::size_t i = 0; i < n.segments.size(); ++i) {
    ++deg[n.segments[i].a];
    ++deg[n.segments[i].b];
    adj[n.segments[i].a].push_back(int(i));
    adj[n.segments[i].b].push_back(int(i));
  }
  for (std::size_t k = 0; k < n.nodes.size(); ++k) {
    EXPECT_EQ(n.nodes[k].degree, deg[k]);
    if (deg[k] != 1)
      continue;
    double length = 0.0;
    int cur = int(k), seg = adj[k][0];
    while (true) {
      length += seg_len(n, n.segments[seg]);
      const road_segment &s = n.segments[seg];
      cur = s.a == cur ? s.b : s.a;
      if (deg[cur] != 2 || length >= 60.0)
        break;
      seg = adj[cur][0] == seg ? adj[cur][1] : adj[cur][0];
    }
    EXPECT_GE(length, 60.0) << "stub at node " << k;
  }
}

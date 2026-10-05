/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/roads.h for the full header).
*/

#include <cmath>
#include <cvc/world/roads.h>
#include <gtest/gtest.h>

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

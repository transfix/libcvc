/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/road_mesh.h for the full header).
*/

#include <algorithm>
#include <cmath>
#include <cvc/world/road_mesh.h>
#include <cvc/world/roads.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::world;

namespace {

// A single straight east-west road from (-100,0) to (100,0).
road_network straight_road(double width = 11.0, bool arterial = true) {
  road_network net;
  net.half = 128.0;
  net.nodes.push_back(road_node{-100.0, 0.0, 1});
  net.nodes.push_back(road_node{100.0, 0.0, 1});
  road_segment s;
  s.a = 0;
  s.b = 1;
  s.width = width;
  s.arterial = arterial;
  net.segments.push_back(s);
  return net;
}

// A plus-shaped crossing (degree-4 junction at the origin).
road_network cross_roads(double width = 11.0) {
  road_network net;
  net.half = 128.0;
  net.nodes = {road_node{-100, 0, 1}, road_node{100, 0, 1}, road_node{0, -100, 1},
               road_node{0, 100, 1}, road_node{0, 0, 4}};
  auto seg = [&](int a, int b) {
    road_segment s;
    s.a = a;
    s.b = b;
    s.width = width;
    s.arterial = true;
    net.segments.push_back(s);
  };
  seg(0, 4);
  seg(4, 1);
  seg(2, 4);
  seg(4, 3);
  return net;
}

} // namespace

TEST(WorldRoadMesh, EmptyNetworkGivesEmptyMesh) {
  const cvc::geometry g = extrude_roads(road_network{});
  EXPECT_EQ(g.const_points().size(), 0u);
  EXPECT_EQ(g.const_tris().size(), 0u);
}

TEST(WorldRoadMesh, RibbonMatchesTheRoadWidthAndLength) {
  road_mesh_params p;
  p.intersections = false; // ribbon only, so the bbox is exactly the carriageway
  p.centerline = false;
  const cvc::geometry g = extrude_roads(straight_road(11.0), p);
  ASSERT_EQ(g.const_tris().size(), 2u); // one quad
  ASSERT_EQ(g.const_points().size(), 4u);
  const cvc::bounding_box bb = g.extents();
  EXPECT_NEAR(bb.maxy - bb.miny, 11.0, 1e-9); // perpendicular extent == width
  EXPECT_NEAR(bb.maxx - bb.minx, 200.0, 1e-9);
  EXPECT_NEAR(bb.minz, 0.08, 1e-9); // flat terrain + default lift
  // All normals point up.
  for (const auto &n : g.const_normals())
    EXPECT_NEAR(n[2], 1.0, 1e-12);
}

TEST(WorldRoadMesh, FollowsTheTerrainHeight) {
  road_mesh_params p;
  p.intersections = false;
  p.centerline = false;
  p.lift = 0.0;
  const auto h = [](double x, double) { return 0.1 * x; }; // a ramp in x
  const cvc::geometry g = extrude_roads(straight_road(11.0), p, h);
  ASSERT_EQ(g.const_points().size(), 4u);
  for (const auto &v : g.const_points())
    EXPECT_NEAR(v[2], 0.1 * v[0], 1e-9); // every vertex sits on the ramp
}

TEST(WorldRoadMesh, JunctionPadAddsGeometryAtTheCrossing) {
  road_mesh_params on, off;
  off.intersections = false;
  off.centerline = on.centerline = false;
  on.intersections = true;
  on.pad_segments = 12;
  const std::size_t tris_off = extrude_roads(cross_roads(), off).const_tris().size();
  const std::size_t tris_on = extrude_roads(cross_roads(), on).const_tris().size();
  EXPECT_EQ(tris_off, 8u); // 4 roads (junction to dead end), one quad each
  // A pad (12 tris) at the junction and a round cap at each of the 4 dead ends.
  EXPECT_EQ(tris_on, tris_off + 5u * 12u);
  // The pad is centred on the junction and reaches the arterial half-width (5.5 m).
  const cvc::geometry g = extrude_roads(cross_roads(), on);
  const cvc::bounding_box bb = g.extents();
  EXPECT_GE(bb.maxx, 100.0); // ribbons still reach the ends
}

TEST(WorldRoadMesh, ArterialsAreDarkerThanLocalStreets) {
  road_mesh_params p;
  p.intersections = false;
  const cvc::geometry art = extrude_roads(straight_road(11.0, /*arterial=*/true), p);
  const cvc::geometry loc = extrude_roads(straight_road(6.5, /*arterial=*/false), p);
  ASSERT_GT(art.const_colors().size(), 0u);
  ASSERT_GT(loc.const_colors().size(), 0u);
  EXPECT_LT(art.const_colors()[0][0], loc.const_colors()[0][0]); // darker red channel
}

TEST(WorldRoadMesh, IsDeterministicOnARealNetwork) {
  road_params rp;
  rp.half = 160.0;
  rp.seed = 3;
  rp.branch_chance = 0.15;
  const road_network net = generate_roads(rp);
  ASSERT_FALSE(net.empty());
  const cvc::geometry a = extrude_roads(net);
  const cvc::geometry b = extrude_roads(net);
  ASSERT_EQ(a.const_points().size(), b.const_points().size());
  ASSERT_GT(a.const_points().size(), 0u);
  for (std::size_t i = 0; i < a.const_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(a.const_points()[i][0], b.const_points()[i][0]);
    EXPECT_DOUBLE_EQ(a.const_points()[i][1], b.const_points()[i][1]);
    EXPECT_DOUBLE_EQ(a.const_points()[i][2], b.const_points()[i][2]);
  }
}

// A road with a right-angle bend in the middle is one strip: two quads that share the bend's pair
// of vertices (no gap, no overlap), each vertex half the width from both pieces of the centreline.
TEST(WorldRoadMesh, ABendIsOneMitredStrip) {
  road_network net;
  net.nodes = {road_node{0, 0, 1}, road_node{50, 0, 2}, road_node{50, 50, 1}};
  for (auto ab : {std::pair<int, int>{0, 1}, {1, 2}}) {
    road_segment s;
    s.a = ab.first;
    s.b = ab.second;
    s.width = 6.0;
    net.segments.push_back(s);
  }
  road_mesh_params p;
  p.intersections = false;
  p.centerline = false;
  p.smooth_passes = 0;
  const cvc::geometry g = extrude_roads(net, p);
  ASSERT_EQ(g.const_points().size(), 6u); // three pairs: the ends and the bend
  ASSERT_EQ(g.const_tris().size(), 4u);
  auto dist = [](double x, double y, double ax, double ay, double bx, double by) {
    const double vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
    const double t = std::max(0.0, std::min(1.0, ((x - ax) * vx + (y - ay) * vy) / l2));
    return std::hypot(ax + vx * t - x, ay + vy * t - y);
  };
  for (const auto &v : g.const_points()) {
    // Off the infinite lines of the two pieces: each vertex is exactly 3 m from the line(s) it
    // borders (the bend's pair from both).
    const double d0 = std::abs(v[1]), d1 = std::abs(v[0] - 50.0);
    EXPECT_TRUE(std::abs(d0 - 3.0) < 1e-9 || std::abs(d1 - 3.0) < 1e-9) << v[0] << "," << v[1];
    EXPECT_LE(std::min(dist(v[0], v[1], 0, 0, 50, 0), dist(v[0], v[1], 50, 0, 50, 50)),
              3.0 * std::sqrt(2.0) + 1e-9);
  }
  // Rounded, the bend gets more vertices and still one strip.
  p.smooth_passes = 2;
  const cvc::geometry r = extrude_roads(net, p);
  EXPECT_GT(r.const_points().size(), 6u);
  EXPECT_EQ(r.const_tris().size(), r.const_points().size() - 2);
}

// A road wide enough gets one dark yellow centre stripe, a hair above the surface and stopping
// short of the dead ends; a narrow lane gets none.
TEST(WorldRoadMesh, ACentreStripeRunsDownTheRoad) {
  road_mesh_params p;
  p.intersections = false;
  p.centerline = true;
  const cvc::geometry g = extrude_roads(straight_road(11.0), p);
  ASSERT_EQ(g.const_tris().size(), 4u); // the road and its stripe
  double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
  int yellow = 0;
  for (std::size_t i = 0; i < g.const_points().size(); ++i) {
    const auto &c = g.const_colors()[i];
    if (c[0] != p.centerline_rgb[0] || c[1] != p.centerline_rgb[1] || c[2] != p.centerline_rgb[2])
      continue;
    ++yellow;
    const auto &v = g.const_points()[i];
    minx = std::min(minx, v[0]);
    maxx = std::max(maxx, v[0]);
    miny = std::min(miny, v[1]);
    maxy = std::max(maxy, v[1]);
    EXPECT_NEAR(v[2], p.lift + p.centerline_lift, 1e-9);
  }
  EXPECT_EQ(yellow, 4);
  EXPECT_NEAR(maxy - miny, p.centerline_width, 1e-9);
  EXPECT_NEAR(minx, -99.0, 1e-9); // 1 m short of each dead end
  EXPECT_NEAR(maxx, 99.0, 1e-9);
  EXPECT_EQ(extrude_roads(straight_road(4.0), p).const_tris().size(), 2u); // too narrow: none
}

// Where a narrow street meets a wider road, the street's strip stops on the junction pad's rim, so
// no corner of it pokes out past the pad, and the pad sits a hair above it.
TEST(WorldRoadMesh, StripsEndOnThePadRim) {
  road_network net;
  net.nodes = {road_node{-100, 0, 1}, road_node{100, 0, 1}, road_node{0, 0, 3},
               road_node{0, 100, 1}};
  auto seg = [&](int a, int b, double w) {
    road_segment s;
    s.a = a;
    s.b = b;
    s.width = w;
    net.segments.push_back(s);
  };
  seg(0, 2, 11.0);
  seg(2, 1, 11.0);
  seg(2, 3, 6.0); // the side street
  road_mesh_params p;
  p.centerline = false;
  p.smooth_passes = 0;
  const cvc::geometry g = extrude_roads(net, p);
  double nearest = 1e9; // the side street's closest vertex to the junction
  for (const auto &v : g.const_points())
    if (std::abs(v[0]) <= 3.0 + 1e-9 && v[1] > 0.5 && std::abs(v[2] - p.lift) < 1e-9)
      nearest = std::min(nearest, std::hypot(v[0], v[1]));
  EXPECT_NEAR(nearest, 5.5, 1e-9); // its corners on the 5.5 m rim
  int pad = 0;
  for (const auto &v : g.const_points())
    pad += std::abs(v[2] - (p.lift + p.pad_lift)) < 1e-9;
  EXPECT_GT(pad, 0);
}

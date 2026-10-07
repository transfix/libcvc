/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/road_mesh.h for the full header).
*/

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
  on.intersections = true;
  on.pad_segments = 12;
  const std::size_t tris_off = extrude_roads(cross_roads(), off).const_tris().size();
  const std::size_t tris_on = extrude_roads(cross_roads(), on).const_tris().size();
  EXPECT_EQ(tris_off, 8u); // 4 segments * 2
  // Pads at the 4 dead-ends (degree 1 -> skipped) + 1 junction (degree 4 -> 12 tris).
  EXPECT_EQ(tris_on, tris_off + 12u);
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

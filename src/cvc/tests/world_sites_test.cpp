/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/sites.h for the full header).
*/

#include <cmath>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/roads.h>
#include <cvc/world/sites.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::world;

namespace {

// A single straight east-west arterial from (-100,0) to (100,0).
road_network straight_road(double width = 11.0) {
  road_network net;
  net.half = 128.0;
  net.nodes.push_back(road_node{-100.0, 0.0, 1});
  net.nodes.push_back(road_node{100.0, 0.0, 1});
  road_segment s;
  s.a = 0;
  s.b = 1;
  s.width = width;
  s.arterial = true;
  net.segments.push_back(s);
  return net;
}

// A plus-shaped crossing of two arterials meeting at the origin (a degree-4 junction).
road_network cross_roads(double width = 11.0) {
  road_network net;
  net.half = 128.0;
  net.nodes.push_back(road_node{-100.0, 0.0, 1}); // 0
  net.nodes.push_back(road_node{100.0, 0.0, 1});  // 1
  net.nodes.push_back(road_node{0.0, -100.0, 1}); // 2
  net.nodes.push_back(road_node{0.0, 100.0, 1});  // 3
  net.nodes.push_back(road_node{0.0, 0.0, 4});    // 4 junction
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

// Reconstruct a lot's footprint as four world-space corners (for overlap / room checks).
void lot_corners(const lot &L, double c[4][2]) {
  const double cs = std::cos(L.yaw), sn = std::sin(L.yaw);
  // local +X runs along the frontage, local +Y toward the road (depth).
  const double hx = 0.5 * L.frontage, hy = 0.5 * L.depth;
  const double lx[4] = {-hx, hx, hx, -hx}, ly[4] = {-hy, -hy, hy, hy};
  for (int i = 0; i < 4; ++i) {
    c[i][0] = L.x + lx[i] * cs - ly[i] * sn;
    c[i][1] = L.y + lx[i] * sn + ly[i] * cs;
  }
}

bool obb_overlap(const lot &A, const lot &B) {
  double ca[4][2], cb[4][2];
  lot_corners(A, ca);
  lot_corners(B, cb);
  const double axes[4][2] = {{std::cos(A.yaw), std::sin(A.yaw)},
                             {-std::sin(A.yaw), std::cos(A.yaw)},
                             {std::cos(B.yaw), std::sin(B.yaw)},
                             {-std::sin(B.yaw), std::cos(B.yaw)}};
  for (auto &ax : axes) {
    double a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
    for (int i = 0; i < 4; ++i) {
      const double pa = ca[i][0] * ax[0] + ca[i][1] * ax[1];
      const double pb = cb[i][0] * ax[0] + cb[i][1] * ax[1];
      a0 = std::min(a0, pa);
      a1 = std::max(a1, pa);
      b0 = std::min(b0, pb);
      b1 = std::max(b1, pb);
    }
    if (a1 < b0 - 1e-6 || b1 < a0 - 1e-6)
      return false; // separating axis
  }
  return true;
}

building_library two_archetypes() {
  building_library lib;
  building_archetype a;
  a.footprint_x = 4.0; // 8 x 5 footprint, 10 tall
  a.footprint_y = 2.5;
  a.height = 10.0;
  a.name = "a";
  lib.archetypes.push_back(a);
  building_archetype b;
  b.footprint_x = 3.0; // 6 x 6, 18 tall
  b.footprint_y = 3.0;
  b.height = 18.0;
  b.name = "b";
  lib.archetypes.push_back(b);
  return lib;
}

} // namespace

TEST(WorldSites, PlacesLotsOnBothSidesOfARoad) {
  site_params sp;
  const std::vector<lot> lots = layout_lots(straight_road(), sp);
  ASSERT_GT(lots.size(), 4u);
  int above = 0, below = 0;
  const double offset = 0.5 * 11.0 + sp.setback + 0.5 * sp.lot_depth;
  for (const lot &L : lots) {
    EXPECT_NEAR(std::fabs(L.y), offset, 1e-6); // centre sits exactly setback+depth/2 off the road
    (L.y > 0 ? above : below)++;
    EXPECT_LT(std::fabs(L.x), 100.0); // inside the segment
  }
  EXPECT_EQ(above, below); // symmetric both-sides placement
}

TEST(WorldSites, OneSideWhenRequested) {
  site_params sp;
  sp.both_sides = false;
  const std::vector<lot> lots = layout_lots(straight_road(), sp);
  ASSERT_GT(lots.size(), 2u);
  for (const lot &L : lots)
    EXPECT_GT(L.y, 0.0); // only the + perpendicular side
}

TEST(WorldSites, LotsStayOffEveryRoadAndClearOfTheJunction) {
  site_params sp;
  const road_network net = cross_roads();
  const std::vector<lot> lots = layout_lots(net, sp);
  ASSERT_GT(lots.size(), 4u);
  for (const lot &L : lots) {
    // No lot footprint corner, nor its centre, lies on a road (room between building and kerb,
    // and nothing parked in a crossing street).
    EXPECT_FALSE(net.on_or_near_road(L.x, L.y, 0.0));
    double c[4][2];
    lot_corners(L, c);
    for (int i = 0; i < 4; ++i)
      EXPECT_FALSE(net.on_or_near_road(c[i][0], c[i][1], 0.0))
          << "lot corner on a road near (" << L.x << "," << L.y << ")";
    // Clear of the degree-4 junction at the origin.
    EXPECT_GE(std::hypot(L.x, L.y), sp.junction_clear - 1e-6);
  }
}

TEST(WorldSites, LotsDoNotOverlap) {
  site_params sp;
  const std::vector<lot> lots = layout_lots(cross_roads(), sp);
  ASSERT_GT(lots.size(), 4u);
  for (std::size_t i = 0; i < lots.size(); ++i)
    for (std::size_t j = i + 1; j < lots.size(); ++j)
      EXPECT_FALSE(obb_overlap(lots[i], lots[j])) << "lots " << i << " and " << j << " overlap";
}

TEST(WorldSites, LotsFaceTheRoad) {
  const road_network net = cross_roads();
  const std::vector<lot> lots = layout_lots(net, site_params{});
  ASSERT_GT(lots.size(), 4u);
  for (const lot &L : lots) {
    // local +Y (the building front) rotated by yaw; stepping that way must approach the road.
    const double fx = -std::sin(L.yaw), fy = std::cos(L.yaw);
    const double d0 = net.distance_to_road(L.x, L.y);
    const double d1 = net.distance_to_road(L.x + fx * 2.0, L.y + fy * 2.0);
    EXPECT_LT(d1, d0) << "lot at (" << L.x << "," << L.y << ") does not face the road";
  }
}

TEST(WorldSites, LayoutIsDeterministic) {
  const road_network net = cross_roads();
  const std::vector<lot> a = layout_lots(net, site_params{});
  const std::vector<lot> b = layout_lots(net, site_params{});
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(a[i].x, b[i].x);
    EXPECT_DOUBLE_EQ(a[i].y, b[i].y);
    EXPECT_DOUBLE_EQ(a[i].yaw, b[i].yaw);
  }
}

TEST(WorldSites, RealRoadNetworkPlacesNonOverlappingLots) {
  // End-to-end against the actual generator on flat ground. A low branch chance gives streets that
  // run between junctions (realistic blocks) rather than a dense every-step crosshatch — see the
  // note in sites.h: a very high branch_chance leaves no room for buildings between intersections.
  road_params rp;
  rp.half = 200.0;
  rp.seed = 7;
  rp.branch_chance = 0.15;
  const road_network net = generate_roads(rp); // flat -> grid of blocks
  ASSERT_FALSE(net.empty());
  const std::vector<lot> lots = layout_lots(net, site_params{});
  EXPECT_GT(lots.size(), 20u);
  // Every lot keeps off the road surface (room between building and kerb, nothing in a crossing).
  for (const lot &L : lots)
    EXPECT_FALSE(net.on_or_near_road(L.x, L.y, 0.0));
}

TEST(WorldSites, FitProducesScaledOrientedInstances) {
  const std::vector<lot> lots = layout_lots(straight_road(), site_params{});
  ASSERT_GT(lots.size(), 2u);
  const building_library lib = two_archetypes();
  fit_params fp;
  const std::vector<building_instance> inst = fit_buildings(lib, lots, fp);
  ASSERT_EQ(inst.size(), lots.size());
  for (std::size_t i = 0; i < inst.size(); ++i) {
    const building_instance &bi = inst[i];
    ASSERT_GE(bi.archetype, 0);
    ASSERT_LT(bi.archetype, int(lib.size()));
    const building_archetype &a = lib.archetypes[bi.archetype];
    const double fw = 2.0 * a.footprint_x, fd = 2.0 * a.footprint_y;
    const double longd = std::max(fw, fd), shortd = std::min(fw, fd);
    // When not clamped, the scaled footprint fits inside the lot (minus margin) on both axes.
    if (bi.scale > fp.min_scale && bi.scale < fp.max_scale) {
      EXPECT_LE(longd * bi.scale, lots[i].frontage - 2.0 * fp.margin + 1e-6);
      EXPECT_LE(shortd * bi.scale, lots[i].depth - 2.0 * fp.margin + 1e-6);
    }
    EXPECT_DOUBLE_EQ(bi.z, 0.0); // no height fn -> ground 0
  }
}

TEST(WorldSites, FitUsesHeightAndIsDeterministic) {
  const std::vector<lot> lots = layout_lots(straight_road(), site_params{});
  const building_library lib = two_archetypes();
  const auto h = [](double, double) { return 7.5; };
  const std::vector<building_instance> a = fit_buildings(lib, lots, fit_params{}, h);
  const std::vector<building_instance> b = fit_buildings(lib, lots, fit_params{}, h);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].archetype, b[i].archetype);
    EXPECT_DOUBLE_EQ(a[i].z, 7.5);
  }
}

TEST(WorldSites, EmptyLibraryGivesNoBuildings) {
  const std::vector<lot> lots = layout_lots(straight_road(), site_params{});
  EXPECT_TRUE(fit_buildings(building_library{}, lots, fit_params{}).empty());
}

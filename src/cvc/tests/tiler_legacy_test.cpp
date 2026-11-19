/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The legacy ContourTiler's own unit tests (transfix/volrover src/ContourTiler/test_*.cpp, on
// CppUnitLite2, built by no target: their CMakeLists line was commented out), converted to
// googletest (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7 T2). They test the vendored tiler's internals
// (src/cvc/geometry/tiler) on its own kernel. What each legacy file became:
//
//   test_Tiling_region.cpp   14 tests  all converted
//   test_sweep_line.cpp      12 tests  all converted (the legacy checked nothing: each runs the
//                                      sweep; the intersections it finds are now checked too)
//   test_contour_utils.cpp    4 tests  3 converted; intersects_boundary2 reads test_data/test7.dat
//   test_augment.cpp          4 tests  augment2 and augment3 converted; offset1 and offset2 read
//                                      test_data/*.dat
//   test_intersection.cpp    65 tests  26 converted (the helpers: make_poly, sorting, trimming,
//                                      neighbours, exits, get_z, triangulate_safe, cutting); 37
//                                      call test_it(), whose body the legacy had commented out (so
//                                      they checked nothing) and which called a
//                                      remove_intersections() signature the library no longer had;
//                                      raw_equal1 needs reader_raw.h, not vendored; and
//                                      new_triangulate1 calls the CDT triangulate() the library
//                                      never called, gone with CGAL's CDT
//   test_tile.cpp            10 tests  Point_sorting1 converted; the rest use fixtures reading
//                                      test_data/test1.dat .. test14.dat
//   test_tile2.cpp           16 tests  not converted: each reads test_data/test<n>_<k>.dat
//   test_skeleton.cpp         9 tests  not converted: to_simple1 reads test_data/simple1.dat and
//                                      the others' calls were commented out (empty bodies)
//   test_Hierarchy.cpp        2 tests  not converted: they read test_data/*.dat
//   test_reader_gnuplot.cpp   2 tests  not converted: the gnuplot reader is not vendored
//   test_minkowski.cpp        1 test   not converted: minkowski.cpp (unused) is not vendored
//   test_mtile.cpp            0 tests
//
// test_data/ was never committed to the volrover repository. The tiler as a whole is tested by
// tiler_test.cpp, against the legacy library's own runs (tools/volrover-goldens).

#include "Contour.h"
#include "Distance_functor.h"
#include "Hierarchy.h"
#include "Tiling_region.h"
#include "Wedge.h"
#include "augment.h"
#include "contour_utils.h"
#include "cut.h"
#include "intersection.h"
#include "polygon_utils.h"
#include "print_utils.h"
#include "set_utils.h"
#include "sweep_line_visitors.h"
#include "tiler_operations.h"
#include "triangle_utils.h"

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cmath>
#include <gtest/gtest.h>
#include <list>
#include <string>
#include <vector>

using namespace cvc::tiler;

#ifndef CVC_WITH_PLANAR
#error tiler_legacy_test needs cvc::planar (the tiler is built only with it)
#endif

// --------------------------------------------------------------------------------------------
// From test_contour_utils.cpp
// --------------------------------------------------------------------------------------------
TEST(TilerLegacy_contour_utils, simple1) { EXPECT_TRUE(Point_2(3, 4) != Point_2(4, 5)); }

TEST(TilerLegacy_contour_utils, distance1) {
  Polygon_2 P;
  P.push_back(Point_2(0, 0));
  P.push_back(Point_2(1, 0));
  P.push_back(Point_2(2, 0));
  P.push_back(Point_2(2, 1));
  P.push_back(Point_2(2, 2));
  P.push_back(Point_2(1, 2));
  P.push_back(Point_2(0, 2));
  P.push_back(Point_2(0, 1));
  EXPECT_EQ(0, distance(P[0], P[0], P));
  EXPECT_EQ(1, distance(P[1], P[0], P));
  EXPECT_EQ(1, distance(P[0], P[1], P));
  EXPECT_EQ(1, distance(P[0], P[7], P));
  EXPECT_EQ(1, distance(P[7], P[0], P));
  EXPECT_EQ(4, distance(P[0], P[4], P));
  EXPECT_EQ(-1, distance(P[0], Point_2(3, 3), P));
  EXPECT_EQ(-1, distance(Point_2(3, 3), Point_2(3, 3), P));
}

TEST(TilerLegacy_contour_utils, intersects_boundary1) {
  Polygon_2 polygon;
  polygon.push_back(Point_2(0, 0));
  polygon.push_back(Point_2(10, 0));
  polygon.push_back(Point_2(10, 10));
  polygon.push_back(Point_2(0, 10));
  EXPECT_TRUE(intersects_boundary(Point_2(0, 0), polygon));
  EXPECT_TRUE(intersects_boundary(Point_2(5, 0), polygon));
  EXPECT_TRUE(!intersects_boundary(Point_2(1, 1), polygon));
}

// --------------------------------------------------------------------------------------------
// From test_Tiling_region.cpp
// --------------------------------------------------------------------------------------------
TEST(TilerLegacy_Tiling_region, Tiling_region1) {
  HTiling_region region = Wedge::LS(Point_2(0, 0), Point_2(1, 0), Point_2(2, 0), 0);
  EXPECT_TRUE(!region->contains(Point_2(0, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 1)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(!region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -1000)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region1_2) {
  HTiling_region region = Wedge::RS(Point_2(0, 0), Point_2(1, 0), Point_2(2, 0), 0);
  region = region->get_complement();
  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 0)));
  EXPECT_TRUE(region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 1)));
  EXPECT_TRUE(region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(!region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -1000)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region2) {
  HTiling_region region = Wedge::LS(Point_2(0, 0), Point_2(1, 0), Point_2(1, 1), 0);
  EXPECT_TRUE(!region->contains(Point_2(0, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 1000)));
  EXPECT_TRUE(region->contains(Point_2(0, 1)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(!region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -1000)));

  EXPECT_TRUE(region->contains(Segment_3(Point_2(0, 1), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(0.5, -10), Point_2(10, 0.5))));
}

TEST(TilerLegacy_Tiling_region, Tiling_region2_2) {
  HTiling_region region = Wedge::RS(Point_2(0, 0), Point_2(1, 0), Point_2(1, 1), 0);
  region = region->get_complement();
  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 1000)));
  EXPECT_TRUE(region->contains(Point_2(0, 1)));
  EXPECT_TRUE(region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(!region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, -1000)));

  EXPECT_TRUE(region->contains(Segment_3(Point_2(0, 1), Point_2(0, 2))));
  EXPECT_TRUE(region->contains(Segment_3(Point_2(-1, 0), Point_2(0, 2))));
  EXPECT_TRUE(region->contains(Segment_3(Point_2(-1, 0), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(0.5, -10), Point_2(10, 0.5))));
}

TEST(TilerLegacy_Tiling_region, Tiling_region3) {
  HTiling_region region = Wedge::LS(Point_2(1, 1), Point_2(1, 0), Point_2(0, 0), 0);
  EXPECT_TRUE(!region->contains(Point_2(0, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 0)));
  EXPECT_TRUE(region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 1000)));
  EXPECT_TRUE(!region->contains(Point_2(0, 1)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(-1000, -1000)));

  EXPECT_TRUE(!region->contains(Segment_3(Point_2(0, 1), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(2, 3))));
  EXPECT_TRUE(region->contains(Segment_3(Point_2(0.5, -10), Point_2(10, 0.5))));
}

TEST(TilerLegacy_Tiling_region, Tiling_region3_2) {
  HTiling_region region = Wedge::RS(Point_2(1, 1), Point_2(1, 0), Point_2(0, 0), 0);
  region = region->get_complement();
  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 0)));
  EXPECT_TRUE(region->contains(Point_2(1.5, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 1000)));
  EXPECT_TRUE(!region->contains(Point_2(0, 1)));
  EXPECT_TRUE(region->contains(Point_2(-1000, 0)));
  EXPECT_TRUE(!region->contains(Point_2(-1000, .0001)));
  EXPECT_TRUE(region->contains(Point_2(1000, 0)));
  EXPECT_TRUE(region->contains(Point_2(1000, .0001)));

  EXPECT_TRUE(region->contains(Point_2(1, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(1, -1000)));
  EXPECT_TRUE(region->contains(Point_2(1000, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(1000, -1000)));
  EXPECT_TRUE(region->contains(Point_2(-1000, -.0001)));
  EXPECT_TRUE(region->contains(Point_2(-1000, -1000)));

  EXPECT_TRUE(!region->contains(Segment_3(Point_2(0, 1), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(0, 2))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(1, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, 0), Point_2(2, 3))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1), Point_2(2, 3))));
  EXPECT_TRUE(region->contains(Segment_3(Point_2(0.5, -10), Point_2(10, 0.5))));
}

TEST(TilerLegacy_Tiling_region, Tiling_region4) {
  HTiling_region region0 = Wedge::LS(Point_2(3, -2), Point_2(0, 0), Point_2(-3, -2), 0);
  HTiling_region region1 = Wedge::LS(Point_2(2, 0), Point_2(0, 0), Point_2(-1, -2), 0);
  HTiling_region region = region0 & region1;
  EXPECT_TRUE(!region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(0, -1)));
  EXPECT_TRUE(region->contains(Point_2(1, -1)));
  EXPECT_TRUE(!region->contains(Point_2(3, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-2.5, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-2, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-3, -2)));
  EXPECT_TRUE(!region->contains(Point_2(2, -1)));
  EXPECT_TRUE(!region->contains(Point_2(-1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 1)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region4_2) {
  HTiling_region region0 = Wedge::LS(Point_2(3, -2), Point_2(0, 0), Point_2(-3, -2), 0);
  HTiling_region region1 = Wedge::LS(Point_2(2, 0), Point_2(0, 0), Point_2(-1, -2), 0);
  HTiling_region region = region0 | region1;
  EXPECT_TRUE(!region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(0, -1)));
  EXPECT_TRUE(region->contains(Point_2(1, -1)));
  EXPECT_TRUE(region->contains(Point_2(3, -2)));
  EXPECT_TRUE(region->contains(Point_2(-2.5, -2)));
  EXPECT_TRUE(region->contains(Point_2(-2, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-3, -2)));
  EXPECT_TRUE(region->contains(Point_2(2, -1)));
  EXPECT_TRUE(!region->contains(Point_2(-1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 1)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region4_3) {
  HTiling_region region0 = Wedge::LS(Point_2(3, -2), Point_2(0, 0), Point_2(-3, -2), 0);
  HTiling_region region1 = Wedge::LS(Point_2(2, 0), Point_2(0, 0), Point_2(-1, -2), 0);
  HTiling_region region = (region0 | region1);
  region = region->get_complement();
  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(!region->contains(Point_2(0, -1)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1)));
  EXPECT_TRUE(!region->contains(Point_2(3, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-2.5, -2)));
  EXPECT_TRUE(!region->contains(Point_2(-2, -2)));
  EXPECT_TRUE(region->contains(Point_2(-3, -2)));
  EXPECT_TRUE(!region->contains(Point_2(2, -1)));
  EXPECT_TRUE(region->contains(Point_2(-1, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 1)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region5) {
  HTiling_region region =
      Tiling_region::overlapping_vertex(Point_2(3, -2), Point_2(0, 0), Point_2(-3, -2),
                                        Point_2(2, 0), Point_2(0, 0), Point_2(-1, -2));
  EXPECT_TRUE(region->contains(Point_2(-2.5, -2)));
  EXPECT_TRUE(region->contains(Point_2(2, -1)));
  EXPECT_TRUE(!region->contains(Point_2(0, -1)));
  EXPECT_TRUE(!region->contains(Point_2(1, -1)));
  EXPECT_TRUE(region->contains(Point_2(2, -1)));
  EXPECT_TRUE(!region->contains(Point_2(-1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(1, 1)));

  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(3, -2)));
  EXPECT_TRUE(region->contains(Point_2(-3, -2)));
  EXPECT_TRUE(region->contains(Point_2(-2, -2)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region6) {
  HTiling_region region = Wedge::LS(Point_2(0, 8), Point_2(8, 8), Point_2(8, 0), 0);
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(8, 8), Point_2(9, 5))));
  region = Wedge::RS(Point_2(0, 8), Point_2(8, 8), Point_2(8, 0), 0);
  region = region->get_complement();
  EXPECT_TRUE(region->contains(Segment_3(Point_2(8, 8), Point_2(9, 5))));
}

TEST(TilerLegacy_Tiling_region, Tiling_region7) {
  HTiling_region region = Tiling_region::overlapping_vertex(
      Point_2(-1, 0), Point_2(0, 0), Point_2(1, 0), Point_2(1, 0), Point_2(0, 0), Point_2(-1, 0));
}

TEST(TilerLegacy_Tiling_region, Tiling_region8) {
  HTiling_region region = Tiling_region::overlapping_vertex(
      Point_2(-1, 0), Point_2(0, 0), Point_2(1, 0), Point_2(-1, 0), Point_2(0, 0), Point_2(1, 0));
  EXPECT_TRUE(region->contains(Point_2(0, 0)));
  EXPECT_TRUE(region->contains(Point_2(-1, 0)));
  EXPECT_TRUE(region->contains(Point_2(1, 0)));
  EXPECT_TRUE(!region->contains(Point_2(0, -1)));
  EXPECT_TRUE(!region->contains(Point_2(0, 1)));
}

TEST(TilerLegacy_Tiling_region, Tiling_region9) {
  HTiling_region region =
      Tiling_region::overlapping_vertex(Point_2(0, 1, 0), Point_2(0, 0, 0), Point_2(1, 0, 0),
                                        Point_2(0, -1, 1), Point_2(0, 0, 1), Point_2(-1, 0, 1));
  EXPECT_TRUE(region->contains(Point_2(0, 0, 0)));
  EXPECT_TRUE(!region->contains(Point_2(0, 0, 1)));
  EXPECT_TRUE(region->contains(Segment_3(Point_2(-1, -1, 0), Point_2(0, 0, 0))));
  EXPECT_TRUE(!region->contains(Segment_3(Point_2(-1, -1, 1), Point_2(0, 0, 1))));
}

// --------------------------------------------------------------------------------------------
// From test_sweep_line.cpp
// --------------------------------------------------------------------------------------------

// The legacy printed what the sweep found (a debug visitor, gone with the debug output). Here the
// sweep must agree with a brute-force test of every pair, in each of its filter modes: some
// intersection at all, and some other than two segments sharing an end.
template <typename Segment> void sl_test(vector<Segment> &segments) {
  bool any = false, proper = false;
  for (std::size_t i = 0; i < segments.size(); ++i)
    for (std::size_t j = i + 1; j < segments.size(); ++j) {
      const Segment_2 a(segments[i].source(), segments[i].target());
      const Segment_2 b(segments[j].source(), segments[j].target());
      if (!do_intersect(a, b))
        continue;
      any = true;
      // The sweep reports an end of one strictly inside the other (end-int), or two interiors
      // crossing (int-int); segments that only share ends meet end to end.
      auto inside = [](const Point_2 &p, const Segment_2 &s) {
        return s.has_on(p) && !xy_equal(p, s.source()) && !xy_equal(p, s.target());
      };
      const bool end_int = inside(a.source(), b) || inside(a.target(), b) ||
                           inside(b.source(), a) || inside(b.target(), a);
      const bool touches = b.has_on(a.source()) || b.has_on(a.target()) || a.has_on(b.source()) ||
                           a.has_on(b.target());
      proper = proper || end_int || !touches;
    }
  EXPECT_EQ(has_intersection(segments.begin(), segments.end(), true, true), any);
  EXPECT_EQ(has_intersection(segments.begin(), segments.end(), true, false), proper);
}

TEST(TilerLegacy_sweep_line, sweep_line1) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_2(0, 0), Point_2(0, 2)));
  segments.push_back(Segment_3(Point_2(1, 0), Point_2(2, 2)));
  segments.push_back(Segment_3(Point_2(2, 0), Point_2(4, 2)));
  segments.push_back(Segment_3(Point_2(0.5, 1), Point_2(2.5, 1)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line2) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_2(0, 0), Point_2(2, 0)));
  segments.push_back(Segment_3(Point_2(1, 0), Point_2(3, 0)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line3) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_2(0, 0), Point_2(0, 2)));
  segments.push_back(Segment_3(Point_2(0, 1), Point_2(0, 3)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line4) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_3(0, 0, 0), Point_3(3, 0, 0)));
  segments.push_back(Segment_3(Point_3(0, 0, 1), Point_3(3, 0, 1)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line5) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_2(0, 0), Point_2(2, 0)));
  segments.push_back(Segment_3(Point_2(1, 0), Point_2(2, 0)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line6) {
  vector<Segment_3> segments;
  segments.push_back(Segment_3(Point_2(0, 0), Point_2(1, 0)));
  segments.push_back(Segment_3(Point_2(1, 0), Point_2(2, 0)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line7) {
  Polygon_2 p;
  p.push_back(Point_2(5.83641, 2.91));
  p.push_back(Point_2(5.83641, 2.80082));
  p.push_back(Point_2(5.84909, 2.79131));
  p.push_back(Point_2(5.86178, 2.79131));
  p.push_back(Point_2(5.88714, 2.78655));
  p.push_back(Point_2(5.89983, 2.78497));
  p.push_back(Point_2(5.90617, 2.7818));
  p.push_back(Point_2(5.91251, 2.7818));
  p.push_back(Point_2(5.96166, 2.76594));
  p.push_back(Point_2(5.97434, 2.76753));
  p.push_back(Point_2(5.97434, 2.91));
  vector<Segment_2> segments(p.edges_begin(), p.edges_end());

  list<SL_intersection> intersections;
  get_intersections(segments.begin(), segments.end(), back_inserter(intersections), false, false);
}

TEST(TilerLegacy_sweep_line, sweep_line8) {
  Polygon_2 p;
  p.push_back(Point_2(0, 0));
  p.push_back(Point_2(0, 2));
  p.push_back(Point_2(2, 0));
  vector<Segment_2> segments(p.edges_begin(), p.edges_end());
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line9) {
  vector<Segment_2> segments;
  segments.push_back(Segment_2(Point_2(0, 0), Point_2(3, 0)));
  segments.push_back(Segment_2(Point_2(1, 0), Point_2(2, 0)));
  sl_test(segments);
}

TEST(TilerLegacy_sweep_line, sweep_line10) {
  Polygon_2 P, Q, Pe, Qe;
  P.push_back(Point_2(0, 0, 0));
  P.push_back(Point_2(4, 0, 0));
  P.push_back(Point_2(4, 4, 0));
  P.push_back(Point_2(0, 4, 0));

  Q.push_back(Point_2(1, 1, 1));
  Q.push_back(Point_2(5, 1, 1));
  Q.push_back(Point_2(4, 2, 1));
  Q.push_back(Point_2(4, 3, 1));
  Q.push_back(Point_2(3, 4, 1));
  Q.push_back(Point_2(1, 4, 1));

  std::list<Segment_2> segments;
  segments.insert(segments.end(), P.edges_begin(), P.edges_end());
  segments.insert(segments.end(), Q.edges_begin(), Q.edges_end());

  list<SL_intersection> ints;
  get_intersections(segments.begin(), segments.end(), back_inserter(ints), true, false);

  typedef boost::unordered_map<Segment_2, list<Point_2>> Map;
  Map map;
  get_interior_only(ints.begin(), ints.end(), map);
}

TEST(TilerLegacy_sweep_line, sweep_line11) {
  vector<Segment_2> segments0, segments1, all;
  segments0.push_back(Segment_2(Point_2(0, 0), Point_2(1, 0)));
  segments0.push_back(Segment_2(Point_2(1, 0), Point_2(1, 1)));
  segments0.push_back(Segment_2(Point_2(1, 1), Point_2(0, 1)));
  segments0.push_back(Segment_2(Point_2(0, 1), Point_2(0, 0)));

  segments1.push_back(Segment_2(Point_2(1, 1), Point_2(2, 1)));
  segments1.push_back(Segment_2(Point_2(2, 1), Point_2(2, 2)));
  segments1.push_back(Segment_2(Point_2(2, 2), Point_2(1, 2)));
  segments1.push_back(Segment_2(Point_2(1, 2), Point_2(1, 1)));

  all.insert(all.end(), segments0.begin(), segments0.end());
  all.insert(all.end(), segments1.begin(), segments1.end());
}

TEST(TilerLegacy_sweep_line, sweep_line12) {
  vector<Segment_2> segments0, segments1;
  segments0.push_back(Segment_2(Point_2(5.16883, 6.42268, 114), Point_2(5.16883, 6.41973, 114)));
  segments0.push_back(Segment_2(Point_2(5.16883, 6.41973, 114), Point_2(5.16883, 6.41678, 114)));
  segments1.push_back(Segment_2(Point_2(5.16883, 6.42268, 113), Point_2(5.16883, 6.41973, 113)));

  typedef list<SL_intersection>::const_iterator i_iter;
  list<SL_intersection> ints;
  get_intersections(segments0.begin(), segments0.end(), segments1.begin(), segments1.end(),
                    back_inserter(ints), true, false);
  cout << ints.size() << endl;

  typedef boost::unordered_map<Segment_2, list<Point_2>> Map;
  Map map;
  get_interior_only(ints.begin(), ints.end(), map);
}

// --------------------------------------------------------------------------------------------
// From test_augment.cpp
// --------------------------------------------------------------------------------------------
TEST(TilerLegacy_augment, augment2) {
  Polygon_2 P, Q, Pe, Qe;
  P.push_back(Point_2(0, 0, 0));
  P.push_back(Point_2(8, 0, 0));
  P.push_back(Point_2(8, 4, 0));
  P.push_back(Point_2(0, 4, 0));

  Q.push_back(Point_2(2, 1, 1));
  Q.push_back(Point_2(7, 1, 1));
  Q.push_back(Point_2(7, 5, 1));
  Q.push_back(Point_2(6, 5, 1));
  Q.push_back(Point_2(6, 2, 1));
  Q.push_back(Point_2(3, 2, 1));
  Q.push_back(Point_2(3, 5, 1));
  Q.push_back(Point_2(4, 5, 1));
  Q.push_back(Point_2(4, 3, 1));
  Q.push_back(Point_2(5, 3, 1));
  Q.push_back(Point_2(5, 6, 1));
  Q.push_back(Point_2(2, 6, 1));

  Pe.push_back(Point_3(0, 0, 0));
  Pe.push_back(Point_3(8, 0, 0));
  Pe.push_back(Point_3(8, 4, 0));
  Pe.push_back(Point_3(7, 4, 0));
  Pe.push_back(Point_3(6, 4, 0));
  Pe.push_back(Point_3(5, 4, 0));
  Pe.push_back(Point_3(4, 4, 0));
  Pe.push_back(Point_3(3, 4, 0));
  Pe.push_back(Point_3(2, 4, 0));
  Pe.push_back(Point_3(0, 4, 0));

  Qe.push_back(Point_3(2, 1, 1));
  Qe.push_back(Point_3(7, 1, 1));
  Qe.push_back(Point_3(7, 4, 1));
  Qe.push_back(Point_3(7, 5, 1));
  Qe.push_back(Point_3(6, 5, 1));
  Qe.push_back(Point_3(6, 4, 1));
  Qe.push_back(Point_3(6, 2, 1));
  Qe.push_back(Point_3(3, 2, 1));
  Qe.push_back(Point_3(3, 4, 1));
  Qe.push_back(Point_3(3, 5, 1));
  Qe.push_back(Point_3(4, 5, 1));
  Qe.push_back(Point_3(4, 4, 1));
  Qe.push_back(Point_3(4, 3, 1));
  Qe.push_back(Point_3(5, 3, 1));
  Qe.push_back(Point_3(5, 4, 1));
  Qe.push_back(Point_3(5, 6, 1));
  Qe.push_back(Point_3(2, 6, 1));
  Qe.push_back(Point_3(2, 4, 1));

  boost::tie(P, Q) = augment1(P, Q);
  EXPECT_EQ(Pe, P);
  EXPECT_EQ(Qe, Q);
}

TEST(TilerLegacy_augment, augment3) {
  Polygon_2 P, Q, Pe, Qe;
  P.push_back(Point_2(0, 0, 0));
  P.push_back(Point_2(4, 0, 0));
  P.push_back(Point_2(4, 4, 0));
  P.push_back(Point_2(0, 4, 0));

  Q.push_back(Point_2(1, 1, 1));
  Q.push_back(Point_2(5, 1, 1));
  Q.push_back(Point_2(4, 2, 1));
  Q.push_back(Point_2(4, 3, 1));
  Q.push_back(Point_2(3, 4, 1));
  Q.push_back(Point_2(1, 4, 1));

  Pe.push_back(Point_3(0, 0, 0));
  Pe.push_back(Point_3(4, 0, 0));
  Pe.push_back(Point_3(4, 1, 0));
  Pe.push_back(Point_3(4, 2, 0));
  Pe.push_back(Point_3(4, 3, 0));
  Pe.push_back(Point_3(4, 4, 0));
  Pe.push_back(Point_3(3, 4, 0));
  Pe.push_back(Point_3(1, 4, 0));
  Pe.push_back(Point_3(0, 4, 0));

  Qe.push_back(Point_3(1, 1, 1));
  Qe.push_back(Point_3(4, 1, 1));
  Qe.push_back(Point_3(5, 1, 1));
  Qe.push_back(Point_3(4, 2, 1));
  Qe.push_back(Point_3(4, 3, 1));
  Qe.push_back(Point_3(3, 4, 1));
  Qe.push_back(Point_3(1, 4, 1));

  boost::tie(P, Q) = augment1(P, Q);
  EXPECT_EQ(Pe, P);
  EXPECT_EQ(Qe, Q);
}

// --------------------------------------------------------------------------------------------
// From test_intersection.cpp
// --------------------------------------------------------------------------------------------
typedef boost::shared_ptr<Triangle> Triangle_handle;

Triangle_handle tile(const Point_3 &a, const Point_3 &b, const Point_3 &c) {
  return Triangle_handle(new Triangle(a, b, c));
}

template <typename Segment> Point_3 point(const Segment &seg, Number_type u) {
  if (u < 0 || u > 1)
    throw logic_error("Illegal u");
  const Point_3 &s = seg.source();
  const Point_3 &t = seg.target();
  return Point_3(s.x() + (t.x() - s.x()) * u, s.y() + (t.y() - s.y()) * u,
                 s.z() + (t.z() - s.z()) * u);
}

TEST(TilerLegacy_intersection, intersection_make_poly2) {
  Segment_3 seg[] = {Segment_3(Point_3(0, 0, 0), Point_3(10, 10, 0)),
                     Segment_3(Point_3(10, 10, 0), Point_3(0, 10, 0)),
                     Segment_3(Point_3(0, 10, 0), Point_3(0, 0, 0))};

  boost::unordered_map<Point_3, Segment_3_> points;
  for (int i = 0; i < 3; ++i) {
    points[point(seg[i], .5)] = Segment_3_(seg[i]);
    points[point(seg[i], .2)] = Segment_3_(seg[i]);
    points[point(seg[i], .8)] = Segment_3_(seg[i]);
    points[point(seg[i], .9)] = Segment_3_(seg[i]);
  }

  Triangle tri(seg[0].source(), seg[1].source(), seg[2].source());
  Polygon_2 poly = make_poly(tri, points);
}

TEST(TilerLegacy_intersection, intersection_sort) {
  Segment_3 seg(Point_3(0, 0, 0), Point_3(10, 10, 10));
  Point_3 pt_arr[] = {point(seg, .5), point(seg, .2), point(seg, .8), point(seg, .9)};

  Distance_functor<Point_3> f(seg.source());
  vector<Point_3> points(&(pt_arr[0]), &(pt_arr[3]));
  points.push_back(seg.source());
  points.push_back(seg.target());

  sort(points.begin(), points.end(), f);

  EXPECT_EQ(0, points[0].x());
  EXPECT_EQ(2, points[1].x());
  EXPECT_EQ(5, points[2].x());
  EXPECT_EQ(8, points[3].x());
  EXPECT_EQ(10, points[4].x());
}

TEST(TilerLegacy_intersection, intersection_poly) {
  Polygon_2 poly;
  poly.push_back(Point_3(0, 0, 0));
  poly.push_back(Point_3(2, 2, 0));
  poly.push_back(Point_3(5, 5, 0));
  poly.push_back(Point_3(10, 10, 0));
  poly.push_back(Point_3(0, 10, 0));
  poly.push_back(Point_3(0, 5, 0));
  poly.push_back(Point_3(0, 2, 0));

  list<Polyline_2> cuts;

  Polyline_2 c;
  c.push_back(Point_3(0, 2, 1));
  c.push_back(Point_3(2, 2, 1));
  cuts.push_back(c);

  c = Polyline_2();
  c.push_back(Point_3(0, 5, 1));
  c.push_back(Point_3(2, 5, 1));
  c.push_back(Point_3(2, 2, 1));
  cuts.push_back(c);

  c = Polyline_2();
  c.push_back(Point_3(2, 2, 1));
  c.push_back(Point_3(2, 5, 1));
  c.push_back(Point_3(5, 5, 1));
  cuts.push_back(c);

  Polygon_2 p0, p1, p00, p01, p10, p11;

  vector<Triangle> triangles;
  cut_into_triangles(poly, cuts.begin(), cuts.end(), back_inserter(triangles));
}

TEST(TilerLegacy_intersection, intersection_trim1) {
  Polygon_2 poly;
  poly.push_back(Point_3(10, 0, 0));
  poly.push_back(Point_3(10, 5, 0));
  poly.push_back(Point_3(5, 5, 0));
  poly.push_back(Point_3(5, 10, 0));
  poly.push_back(Point_3(0, 10, 0));
  poly.push_back(Point_3(0, 0, 0));

  Polyline_2 cut;
  cut.push_back(Point_3(0, 0, 0));
  cut.push_back(Point_3(5, 5, 0));
  cut.push_back(Point_3(5, 10, 0));

  Polyline_2 t1 = trim_forward(poly, cut);
  EXPECT_EQ(Point_3(0, 0, 0), t1[0]);
  EXPECT_EQ(Point_3(5, 5, 0), t1[1]);

  cut = Polyline_2(cut.rbegin(), cut.rend());
  Polyline_2 t2 = trim_backward(poly, cut);
  EXPECT_EQ(Point_3(5, 5, 0), t2[0]);
  EXPECT_EQ(Point_3(0, 0, 0), t2[1]);
}

TEST(TilerLegacy_intersection, intersection_trim2) {
  Polygon_2 poly;
  poly.push_back(Point_3(5, 5, 0));
  poly.push_back(Point_3(5, 10, 0));
  poly.push_back(Point_3(0, 10, 0));
  poly.push_back(Point_3(0, 0, 0));
  poly.push_back(Point_3(10, 0, 0));
  poly.push_back(Point_3(10, 5, 0));

  Polyline_2 cut;
  cut.push_back(Point_3(0, 0, 0));
  cut.push_back(Point_3(5, 5, 0));
  cut.push_back(Point_3(5, 10, 0));

  Polyline_2 t1 = trim_forward(poly, cut);
  EXPECT_EQ(Point_3(0, 0, 0), t1[0]);
  EXPECT_EQ(Point_3(5, 5, 0), t1[1]);

  cut = Polyline_2(cut.rbegin(), cut.rend());
  Polyline_2 t2 = trim_backward(poly, cut);
  EXPECT_EQ(Point_3(5, 5, 0), t2[0]);
  EXPECT_EQ(Point_3(0, 0, 0), t2[1]);
}

TEST(TilerLegacy_intersection, intersection_trim3) {
  Polygon_2 poly;
  poly.push_back(Point_3(5, 10, 0));
  poly.push_back(Point_3(0, 10, 0));
  poly.push_back(Point_3(0, 0, 0));
  poly.push_back(Point_3(10, 0, 0));
  poly.push_back(Point_3(10, 5, 0));
  poly.push_back(Point_3(5, 5, 0));

  Polyline_2 cut;
  cut.push_back(Point_3(0, 0, 0));
  cut.push_back(Point_3(5, 5, 0));
  cut.push_back(Point_3(5, 10, 0));

  Polyline_2 t1 = trim_forward(poly, cut);
  EXPECT_EQ(Point_3(0, 0, 0), t1[0]);
  EXPECT_EQ(Point_3(5, 5, 0), t1[1]);

  cut = Polyline_2(cut.rbegin(), cut.rend());
  Polyline_2 t2 = trim_backward(poly, cut);
  EXPECT_EQ(Point_3(5, 5, 0), t2[0]);
  EXPECT_EQ(Point_3(0, 0, 0), t2[1]);
}

TEST(TilerLegacy_intersection, intersection_trim4) {
  Polygon_2 poly;
  poly.push_back(Point_3(0, 10, 0));
  poly.push_back(Point_3(0, 0, 0));
  poly.push_back(Point_3(10, 0, 0));
  poly.push_back(Point_3(10, 5, 0));
  poly.push_back(Point_3(5, 5, 0));
  poly.push_back(Point_3(5, 10, 0));

  Polyline_2 cut;
  cut.push_back(Point_3(0, 0, 1));
  cut.push_back(Point_3(5, 5, 1));
  cut.push_back(Point_3(5, 10, 1));

  Polyline_2 t1 = trim_forward(poly, cut);
  EXPECT_EQ(Point_3(0, 0, 1), t1[0]);
  EXPECT_EQ(Point_3(5, 5, 1), t1[1]);

  cut = Polyline_2(cut.rbegin(), cut.rend());
  Polyline_2 t2 = trim_backward(poly, cut);
  EXPECT_EQ(Point_3(5, 5, 1), t2[0]);
  EXPECT_EQ(Point_3(0, 0, 1), t2[1]);
}

TEST(TilerLegacy_intersection, intersection_neighbor1) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, 0, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_neighbor2) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, -2, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_neighbor3) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, 0.5, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_exit1) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, 0, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  int yi = 0;
  int gi = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_exit2) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, -2, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  int yi = 0;
  int gi = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_exit3) {
  Triangle_handle ytile(new Triangle(Point_3(3, 2, 1), Point_3(1, 2, 0), Point_3(3, 0.5, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 4, 0)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  int yi = 0;
  int gi = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_exit4) {
  Triangle_handle ytile(new Triangle(Point_3(1, 1, 1), Point_3(1, -1, 0), Point_3(3, 1, 0)));
  Triangle_handle gtile(new Triangle(Point_3(0, 0, 0), Point_3(4, 0, 1), Point_3(0, 0, 1)));
  list<Triangle_handle> ytiles, gtiles;
  ytiles.push_back(ytile);
  gtiles.push_back(gtile);

  double yz = 0;
  double gz = 1;
  int yi = 0;
  int gi = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_exit26) {
  vector<Triangle_handle> yellow, green;

  Point_3 y[] = {
      Point_3(8, 4, 2),   // 0
      Point_3(0, 0, 1.5), // 1
      Point_3(8, 3, 2),   // 2
  };
  yellow.push_back(tile(y[0], y[1], y[2]));

  Point_3 g[] = {
      Point_3(0, 0, 1.5), // 0
      Point_3(6, 2, 2),   // 1
      Point_3(0, 4, 1.5), // 2
  };
  green.push_back(tile(g[0], g[1], g[2]));

  double yz = 0;
  double gz = 1;
  int yi = 0;
  int gi = 1;
  Z_adjustments<Triangle_handle> z_adj(yz, gz, 0.04);
}

TEST(TilerLegacy_intersection, intersection_get_z1) {
  Triangle_handle t(new Triangle(Point_3(0, 0, 0), Point_3(1, 0, 1), Point_3(0, 1, 0)));

  EXPECT_EQ(0, get_z(*t, Point_3(0, 0, 0)));
  EXPECT_EQ(0.5, get_z(*t, Point_3(0.5, 0, 0)));
  EXPECT_EQ(0, get_z(*t, Point_3(0, 0.5, 0)));
  EXPECT_EQ(0.5, get_z(*t, Point_3(0.5, 0.5, 0)));
  EXPECT_EQ(1, get_z(*t, Point_3(1, 0, 0)));
  EXPECT_EQ(0, get_z(*t, Point_3(0, 1, 0)));
  EXPECT_EQ(0.75, get_z(*t, Point_3(0.75, 0.25, 0)));
}

TEST(TilerLegacy_intersection, triangulate1) {
  Polygon_2 p;
  p.push_back(Point_2(0, 2.5));
  p.push_back(Point_2(1, 3));
  p.push_back(Point_2(1.5, 3.25));
  p.push_back(Point_2(0, 4));

  list<Triangle> nt;
  boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> point2edges;
  triangulate_safe(p, back_inserter(nt), point2edges);
}

TEST(TilerLegacy_intersection, triangulate2) {
  Polygon_2 p;
  p.push_back(Point_2(0, 0));
  p.push_back(Point_2(0.5, 0));
  p.push_back(Point_2(1, 0));
  p.push_back(Point_2(1, 1));
  p.push_back(Point_2(0, 1));

  boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> point2edges;
  point2edges[Point_3(0, 0, 0)].insert(Segment_3_undirected(Point_3(0, 1, 0), Point_3(0, 0, 0)));
  point2edges[Point_3(0, 0, 0)].insert(Segment_3_undirected(Point_3(0, 0, 0), Point_3(1, 0, 0)));
  point2edges[Point_3(0.5, 0, 0)].insert(Segment_3_undirected(Point_3(0, 0, 0), Point_3(1, 0, 0)));
  point2edges[Point_3(1, 0, 0)].insert(Segment_3_undirected(Point_3(0, 0, 0), Point_3(1, 0, 0)));
  point2edges[Point_3(1, 0, 0)].insert(Segment_3_undirected(Point_3(1, 0, 0), Point_3(1, 1, 0)));
  point2edges[Point_3(1, 1, 0)].insert(Segment_3_undirected(Point_3(1, 0, 0), Point_3(1, 1, 0)));
  point2edges[Point_3(1, 1, 0)].insert(Segment_3_undirected(Point_3(1, 1, 0), Point_3(0, 1, 0)));
  point2edges[Point_3(0, 1, 0)].insert(Segment_3_undirected(Point_3(1, 1, 0), Point_3(0, 1, 0)));
  point2edges[Point_3(0, 1, 0)].insert(Segment_3_undirected(Point_3(0, 1, 0), Point_3(0, 0, 0)));

  list<Triangle> nt;
  triangulate_safe(p, back_inserter(nt), point2edges);
}

TEST(TilerLegacy_intersection, getz1) {
  Triangle t(Point_3(4.5, 2.5, 0), Point_3(3, 5, 0), Point_3(2.5, 2.5, 1));
  Point_2 p(2.73333, 3.21429);

  Triangle_2 tri(Point_2(5, 2.5), Point_2(7, 0), Point_2(2.5, 2.5));
}

TEST(TilerLegacy_intersection, abc1) {

  Polygon_2 p;
  p.push_back(Point_3(2, 2, 1.5));
  p.push_back(Point_3(1, 3, 1.75));
  p.push_back(Point_3(1.5, 3.25, 1.625));
  p.push_back(Point_3(0, 4, 2));
  p.push_back(Point_3(0, 2.5, 2));
  p.push_back(Point_3(0, 2, 2));

  Polyline_2 cut;
  cut.push_back(Point_3(0, 2.5, 2));
  cut.push_back(Point_3(0.5, 2.5, 2));
  cut.push_back(Point_3(1, 3, 1.75));
  cut.push_back(Point_3(2, 2, 1.5));
  cut.push_back(Point_3(0, 2, 2));

  Polygon_2 a, b;
}

TEST(TilerLegacy_intersection, quad1) {

  Number_type r1, r2, c1, c2;

  // The legacy test expected false where a = b = 0; the library it tested throws there instead
  // (Z_adjustments.h), so the switched-off test would have failed against it.
  EXPECT_THROW(solve_quad(0, 0, 0, r1, r2, c1, c2), std::logic_error);

  EXPECT_TRUE(solve_quad(4, 0, 0, r1, r2, c1, c2));
  EXPECT_EQ(0, r1);
  EXPECT_EQ(0, r2);

  EXPECT_THROW(solve_quad(0, 0, 4, r1, r2, c1, c2), std::logic_error);

  EXPECT_TRUE(solve_quad(0, 4, 4, r1, r2, c1, c2));
  EXPECT_EQ(-1, r1);
  EXPECT_EQ(-1, r2);

  EXPECT_TRUE(solve_quad(0, 4, -4, r1, r2, c1, c2));
  EXPECT_EQ(1, r1);
  EXPECT_EQ(1, r2);

  EXPECT_TRUE(!solve_quad(1, 0, 4, r1, r2, c1, c2));

  EXPECT_TRUE(solve_quad(1, 0, -4, r1, r2, c1, c2));
  EXPECT_EQ(2, r1);
  EXPECT_EQ(-2, r2);

  EXPECT_TRUE(solve_quad(1, 1, -12, r1, r2, c1, c2));
  EXPECT_EQ(-4, r1);
  EXPECT_EQ(3, r2);

  EXPECT_TRUE(solve_quad(1, 1, -12, r1, r2, c1, c2));
  EXPECT_EQ(-4, r1);
  EXPECT_EQ(3, r2);

  EXPECT_TRUE(solve_quad(3.14, -22.31561999, 16.4101, r1, r2, c1, c2));
  EXPECT_TRUE(abs(r1 - 6.2738853503184711) < 0.001);
  EXPECT_TRUE(abs(r2 - .833) < 0.001);

  EXPECT_TRUE(solve_quad(0.276254000000000, -0.104644000000000, 0.009909640000000, r1, r2, c1, c2));
}

TEST(TilerLegacy_intersection, cut1) {
  Triangle triangle(Point_2(0, 0), Point_2(1, 0), Point_2(0, 1));
  Segment_3 edge(Point_2(0, 0), Point_2(1, 0));
  Point_3 point(0.5, 0, 0);
  Triangle t1, t2;

  boost::tie(t1, t2) = decompose_triangle(triangle, edge, point);

  EXPECT_TRUE(index(Point_2(0, 0), t1) > -1);
  EXPECT_TRUE(index(Point_2(0.5, 0), t1) > -1);
  EXPECT_TRUE(index(Point_2(0, 1), t1) > -1);

  EXPECT_TRUE(index(Point_2(1, 0), t2) > -1);
  EXPECT_TRUE(index(Point_2(0.5, 0), t2) > -1);
  EXPECT_TRUE(index(Point_2(0, 1), t2) > -1);

  cout << pp_tri(t1) << endl;
}

TEST(TilerLegacy_intersection, cut2) {
  Triangle triangle(Point_2(0, 0), Point_2(1, 0), Point_2(0, 1));
  Segment_3 edge(Point_2(0, 0), Point_2(1, 0));
  list<Point_3> points;
  points.push_back(Point_3(0.5, 0, 0));
  points.push_back(Point_3(0.75, 0, 0));
  boost::unordered_map<Segment_3_undirected, list<Point_3>> edge2points;

  edge2points[edge] = points;

  list<Triangle> triangles;

  decompose_triangle(triangle, edge2points, back_inserter(triangles));

  for (list<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
    cout << pp_tri(*it) << endl;
  }
}

TEST(TilerLegacy_intersection, cut3) {
  Triangle triangle(Point_2(1, 0), Point_2(0, 1), Point_2(0, 0));
  Segment_3 edge(Point_2(0, 0), Point_2(1, 0));
  list<Point_3> points;
  points.push_back(Point_3(0.5, 0, 0));
  points.push_back(Point_3(0.75, 0, 0));
  boost::unordered_map<Segment_3_undirected, list<Point_3>> edge2points;

  edge2points[edge] = points;

  list<Triangle> triangles;

  decompose_triangle(triangle, edge2points, back_inserter(triangles));

  for (list<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
    cout << pp_tri(*it) << endl;
  }
}

TEST(TilerLegacy_intersection, cut4) {
  Triangle triangle(Point_2(0, 1), Point_2(0, 0), Point_2(1, 0));
  Segment_3 edge(Point_2(0, 0), Point_2(1, 0));
  list<Point_3> points;
  points.push_back(Point_3(0.5, 0, 0));
  points.push_back(Point_3(0.75, 0, 0));
  boost::unordered_map<Segment_3_undirected, list<Point_3>> edge2points;

  edge2points[edge] = points;

  list<Triangle> triangles;

  decompose_triangle(triangle, edge2points, back_inserter(triangles));

  for (list<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
    cout << pp_tri(*it) << endl;
  }
}

TEST(TilerLegacy_intersection, cut5) {
  Triangle triangle(Point_2(0, 1), Point_2(0, 0), Point_2(1, 0));
  Segment_3 edge1(Point_2(0, 0), Point_2(1, 0));
  Segment_3 edge2(Point_2(1, 0), Point_2(0, 1));
  list<Point_3> points;
  points.push_back(Point_3(0.5, 0, 0));
  points.push_back(Point_3(0.75, 0, 0));
  boost::unordered_map<Segment_3_undirected, list<Point_3>> edge2points;

  edge2points[edge1] = points;
  edge2points[edge2] = points;

  list<Triangle> triangles;

  bool caught = false;
  try {
    decompose_triangle(triangle, edge2points, back_inserter(triangles));
  } catch (logic_error &e) {
    caught = true;
  }
  EXPECT_TRUE(caught);

  for (list<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
    cout << pp_tri(*it) << endl;
  }
}

// --------------------------------------------------------------------------------------------
// From test_tile.cpp
// --------------------------------------------------------------------------------------------

TEST(TilerLegacy_tile, Point_sorting1) {
  Point_2 point(3, 3, 0);
  vector<Point_2> points;
  points.push_back(Point_2(6, 3, 0));
  points.push_back(Point_2(2, 6, 0));
  points.push_back(Point_2(4, 1, 0));
  points.push_back(Point_2(2, 2, 0));

  std::sort(points.begin(), points.end());
  EXPECT_EQ(Point_2(2, 2, 0), points[0]);
  EXPECT_EQ(Point_2(2, 6, 0), points[1]);
  EXPECT_EQ(Point_2(4, 1, 0), points[2]);
  EXPECT_EQ(Point_2(6, 3, 0), points[3]);

  std::sort(points.begin(), points.end(), Dist_cmp(point));
  EXPECT_EQ(Point_2(2, 2, 0), points[0]);
  EXPECT_EQ(Point_2(4, 1, 0), points[1]);
  EXPECT_EQ(Point_2(6, 3, 0), points[2]);
  EXPECT_EQ(Point_2(2, 6, 0), points[3]);
}

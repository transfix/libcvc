/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/interp.h for the full header).
*/

#include <cmath>
#include <cvc/lsys/derive.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/parse.h>
#include <gtest/gtest.h>

using namespace cvc::lsys;

namespace {
ruleset R(const char *s) {
  parse_result p = parse_lsys(s);
  EXPECT_TRUE(p.ok);
  return p.rs;
}
} // namespace

TEST(LsysInterp, SingleForwardMakesOneUpwardSegment) {
  ruleset rs = R("kind: plant\nmode: parallel\nstep: 5\nwidth: 1\ntaper: 0\naxiom: F\n");
  derive_options o;
  o.generations = 0;
  structure s = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(s.segments.size(), std::size_t(1));
  EXPECT_NEAR(s.segments[0].a.z, 0.0, 1e-9);
  EXPECT_NEAR(s.segments[0].b.z, 5.0, 1e-9); // heading starts +Z
  EXPECT_NEAR(s.segments[0].r0, 1.0, 1e-9);
}

TEST(LsysInterp, RotationBendsIntoXY) {
  // +(90) yaws the heading fully into the ground plane; F then moves in XY.
  ruleset rs = R("kind: plant\nmode: parallel\nstep: 4\naxiom: +(90) F\n");
  derive_options o;
  o.generations = 0;
  structure s = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(s.segments.size(), std::size_t(1));
  EXPECT_NEAR(s.segments[0].b.z, 0.0, 1e-9);
  EXPECT_NEAR(std::hypot(s.segments[0].b.x, s.segments[0].b.y), 4.0, 1e-6);
}

TEST(LsysInterp, BracketsPushPopState) {
  // [ +(30) F ] F : two segments sharing the base; second goes straight up.
  ruleset rs = R("kind: plant\nmode: parallel\nstep: 3\naxiom: [ +(30) F ] F\n");
  derive_options o;
  o.generations = 0;
  structure s = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(s.segments.size(), std::size_t(2));
  // The second segment restored the origin and heading (straight up).
  EXPECT_NEAR(s.segments[1].a.x, 0.0, 1e-9);
  EXPECT_NEAR(s.segments[1].b.z, 3.0, 1e-9);
}

TEST(LsysInterp, TreeGrowsUpAndSpreads) {
  ruleset rs = R("kind: plant\nmode: parallel\nangle: 25\nstep: 5\nwidth: 1\ntaper: 0.1\n"
                 "param r = 0.7\naxiom: A(1)\n"
                 "A(s) : s > 0.06 -> ! F(5*s) [ +(25) A(s*r) ] [ -(25) A(s*r) ] L\n");
  derive_options o;
  o.generations = 6;
  structure s = interpret(rs, derive(rs, o).w);
  EXPECT_GT(s.segments.size(), std::size_t(5));
  EXPECT_GT(s.leaves.size(), std::size_t(0));
  EXPECT_GT(s.hi.z, 5.0);          // reaches upward
  EXPECT_GT(s.hi.x - s.lo.x, 2.0); // spreads horizontally
}

TEST(LsysInterp, BuildingBoxHasFootprintAndHeight) {
  ruleset rs = R("kind: building\nmode: sequential\naxiom: Scale(12,8,3) ;(6) Box(6)\n");
  derive_options o;
  structure s = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(s.boxes.size(), std::size_t(1));
  EXPECT_NEAR(s.boxes[0].half.x, 6.0, 1e-9);
  EXPECT_NEAR(s.boxes[0].half.y, 4.0, 1e-9);
  EXPECT_NEAR(s.boxes[0].half.z, 1.5, 1e-9);
  EXPECT_EQ(s.boxes[0].rl, role::wall_concrete);
  EXPECT_NEAR(s.boxes[0].center.z, 1.5, 1e-9); // base on the ground
}

TEST(LsysInterp, ScopeOpsBuildAndPaint) {
  // Exercise the scope alphabet + a paint terminal + relevel/turn.
  ruleset rs = R("kind: building\nmode: sequential\naxiom: Scale(6,4,3) Rot(0,0,30) ;(6) Box(6) "
                 "Trans(2,0,0) P(4, 1.5)\n");
  derive_options o;
  structure s = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(s.boxes.size(), std::size_t(1));
  // Rot(0,0,30) yawed the box: axis0 is no longer the world x-axis.
  EXPECT_GT(std::fabs(s.boxes[0].axis[0].y), 0.1);
  EXPECT_EQ(s.paints.size(), std::size_t(1)); // the P() ground stamp
  EXPECT_NEAR(s.paints[0].radius, 1.5, 1e-9);
}

TEST(LsysInterp, RelevereAndTurnDoNotCrash) {
  ruleset rs = R("kind: plant\nmode: parallel\nstep: 3\naxiom: & & & $ F | F\n");
  derive_options o;
  structure s = interpret(rs, derive(rs, o).w);
  EXPECT_EQ(s.segments.size(), std::size_t(2));
}

TEST(LsysInterp, DeterministicGeometry) {
  ruleset rs = R("kind: plant\nmode: parallel\nangle: 25\nstep: 5\nparam r = 0.7\naxiom: A(1)\n"
                 "A(s) : s > 0.06 -> F(5*s) [ +(25) A(s*r) ] [ -(25) A(s*r) ]\n");
  derive_options o;
  o.generations = 6;
  o.master_seed = 11;
  structure a = interpret(rs, derive(rs, o).w);
  structure b = interpret(rs, derive(rs, o).w);
  ASSERT_EQ(a.segments.size(), b.segments.size());
  for (std::size_t i = 0; i < a.segments.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.segments[i].a.x, b.segments[i].a.x);
    EXPECT_DOUBLE_EQ(a.segments[i].b.z, b.segments[i].b.z);
  }
}

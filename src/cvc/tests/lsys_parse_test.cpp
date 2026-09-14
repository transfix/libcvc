/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/parse.h for the full header).
*/

#include <cvc/lsys/io.h>
#include <cvc/lsys/parse.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace cvc::lsys;

namespace {
const char *kCanonical = "# cvc lsystem v1\n"
                         "# a tiny monopodial tree\n"
                         "name: pine\n"
                         "kind: plant\n"
                         "mode: parallel\n"
                         "contain: none\n"
                         "angle: 22.5\n"
                         "tilt: 30\n"
                         "roll: 90\n"
                         "step: 5\n"
                         "width: 0.7\n"
                         "taper: 0.1\n"
                         "preview_gen: 6\n"
                         "build_gen: 10\n"
                         "param r = 0.7\n"
                         "axiom: A(1)\n"
                         "A(s) : s > 0.05 -> F(s) [ +(25) A(s*r) ] [ -(25) A(s*r) ] T A(s*0.9)\n";
} // namespace

TEST(LsysParse, ByteExactRoundTrip) {
  parse_result pr = parse_lsys(kCanonical);
  ASSERT_TRUE(pr.ok);
  EXPECT_EQ(write_lsys(pr.rs), std::string(kCanonical));
}

TEST(LsysParse, HeadersAndParams) {
  parse_result pr = parse_lsys(kCanonical);
  ASSERT_TRUE(pr.ok);
  EXPECT_EQ(pr.rs.name, "pine");
  EXPECT_EQ(pr.rs.kind, asset_kind::plant);
  EXPECT_EQ(pr.rs.mode, derivation_mode::parallel);
  EXPECT_DOUBLE_EQ(pr.rs.angle_deg, 22.5);
  EXPECT_DOUBLE_EQ(pr.rs.params.get("r"), 0.7);
  EXPECT_EQ(pr.rs.axiom.size(), std::size_t(1));
  EXPECT_EQ(pr.rs.prods.size(), std::size_t(1));
  EXPECT_EQ(pr.rs.comments.size(), std::size_t(2));
}

TEST(LsysParse, ContextSensitiveIsNotGenNested) {
  parse_result pr = parse_lsys("axiom: A\nB < A > C -> F\n");
  ASSERT_TRUE(pr.ok);
  EXPECT_FALSE(pr.rs.gen_nested);
  EXPECT_EQ(pr.rs.prods[0].left_ctx.size(), std::size_t(1));
  EXPECT_EQ(pr.rs.prods[0].right_ctx.size(), std::size_t(1));
}

TEST(LsysParse, DeletingRuleIsNotGenNested) {
  parse_result pr = parse_lsys("axiom: A\nA -> \n"); // empty successor == delete
  ASSERT_TRUE(pr.ok);
  EXPECT_FALSE(pr.rs.gen_nested);
  EXPECT_TRUE(pr.rs.prods[0].deletes);
}

TEST(LsysParse, ProbabilityPrefix) {
  parse_result pr = parse_lsys("axiom: X\nX -> (0.4) A B\n");
  ASSERT_TRUE(pr.ok);
  EXPECT_FALSE(pr.rs.prods[0].probability.empty());
  EXPECT_EQ(pr.rs.prods[0].successor.size(), std::size_t(2));
}

TEST(LsysParse, BadLineIsAnError) {
  parse_result pr = parse_lsys("this is not valid\n");
  EXPECT_FALSE(pr.ok);
  EXPECT_FALSE(pr.diags.empty());
}

TEST(LsysParse, SemanticRoundTripThroughWriter) {
  parse_result a = parse_lsys(kCanonical);
  parse_result b = parse_lsys(write_lsys(a.rs));
  ASSERT_TRUE(b.ok);
  EXPECT_EQ(a.rs.prods.size(), b.rs.prods.size());
  EXPECT_EQ(a.rs.axiom.size(), b.rs.axiom.size());
  EXPECT_EQ(write_lsys(a.rs), write_lsys(b.rs));
}

TEST(LsysParse, FileRoundTrip) {
  parse_result a = parse_lsys(kCanonical);
  ASSERT_TRUE(a.ok);
  namespace fs = std::filesystem;
  fs::path p = fs::temp_directory_path() / "cvc_lsys_roundtrip.lsys";
  write_lsys_file(p.string(), a.rs);
  parse_result b = parse_lsys_file(p.string());
  ASSERT_TRUE(b.ok);
  EXPECT_EQ(write_lsys(b.rs), std::string(kCanonical)); // byte-exact through disk
  EXPECT_THROW(parse_lsys_file("/no/such/dir/missing.lsys"), std::runtime_error);
  fs::remove(p);
}

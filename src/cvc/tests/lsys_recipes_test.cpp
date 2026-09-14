/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/recipes.h for the full header).
*/

// Published module/geometry counts for the built-in recipes — the long-lived
// determinism golden. If a change to the engine or a recipe moves any of these
// numbers, that is a (possibly intended) behaviour change and must be reviewed.

#include <cvc/lsys/derive.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/io.h>
#include <cvc/lsys/recipes.h>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace cvc::lsys;

namespace {
struct golden {
  const char *name;
  int gen;
  std::uint64_t seed;
  std::size_t modules, segments, leaves, boxes;
  std::uint64_t hash;
};
const golden kGoldens[] = {
    {"pine_monopodial", 6, 0ull, 1459, 120, 114, 0, 12700701521940314076ull},
    {"oak_sympodial", 5, 0ull, 3026, 121, 121, 0, 3201695267255346932ull},
    {"birch_slender", 7, 0ull, 126, 19, 12, 0, 241344888783099111ull},
    {"shrub_bush", 5, 0ull, 1937, 121, 121, 0, 9440298262444396808ull},
    {"boulder_cluster", 1, 0ull, 16, 0, 0, 3, 743450476311489804ull},
    {"office_block", 1, 0ull, 14, 0, 0, 6, 16078765480892577479ull},
    {"brick_house", 1, 0ull, 6, 0, 0, 2, 2938555479066870159ull},
};
} // namespace

TEST(LsysRecipes, AllParseAndAreListed) {
  auto names = recipe_names();
  EXPECT_GE(names.size(), std::size_t(7));
  for (const std::string &n : names) {
    EXPECT_TRUE(has_recipe(n));
    ruleset rs = load_recipe(n); // throws if it fails to parse
    EXPECT_FALSE(rs.axiom.empty());
    // Every recipe writes back and re-parses (writer/parser agreement).
    parse_result pr = parse_lsys(write_lsys(rs));
    EXPECT_TRUE(pr.ok) << n;
  }
}

TEST(LsysRecipes, PublishedCountsAreStable) {
  for (const golden &g : kGoldens) {
    ruleset rs = load_recipe(g.name);
    derive_options o;
    o.generations = g.gen;
    o.master_seed = g.seed;
    derive_result d = derive(rs, o);
    structure s = interpret(rs, d.w);
    EXPECT_EQ(d.w.size(), g.modules) << g.name;
    EXPECT_EQ(s.segments.size(), g.segments) << g.name;
    EXPECT_EQ(s.leaves.size(), g.leaves) << g.name;
    EXPECT_EQ(s.boxes.size(), g.boxes) << g.name;
    EXPECT_EQ(d.w.content_hash(), g.hash) << g.name;
  }
}

TEST(LsysRecipes, EveryRecipeProducesGeometry) {
  for (const std::string &n : recipe_names()) {
    ruleset rs = load_recipe(n);
    derive_options o;
    o.generations = rs.kind == asset_kind::plant ? 5 : 2;
    structure s = interpret(rs, derive(rs, o).w);
    EXPECT_FALSE(s.empty()) << n;
  }
}

TEST(LsysRecipes, SourceAndUnknownHandling) {
  EXPECT_FALSE(recipe_source("pine_monopodial").empty());
  EXPECT_TRUE(recipe_source("no_such_recipe").empty());
  EXPECT_FALSE(has_recipe("no_such_recipe"));
  EXPECT_THROW(load_recipe("no_such_recipe"), std::out_of_range);
}

TEST(LsysRecipes, DerivationIsReproducible) {
  for (const std::string &n : recipe_names()) {
    ruleset rs = load_recipe(n);
    derive_options o;
    o.generations = rs.kind == asset_kind::plant ? 5 : 2;
    o.master_seed = 123;
    EXPECT_EQ(derive(rs, o).w.content_hash(), derive(rs, o).w.content_hash()) << n;
  }
}

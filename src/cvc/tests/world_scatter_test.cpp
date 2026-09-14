/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/scatter.h for the full header).
*/

// Placement determinism + stream independence (the insertion-stability repair of
// the predecessor's single mt19937): changing the tree count leaves the building
// placements byte-identical, because the categories draw from independent
// hashed streams keyed on stable cell ids, not a shared sequential generator.

#include <cvc/world/heightfield.h>
#include <cvc/world/scatter.h>
#include <cvc/world/surface.h>
#include <gtest/gtest.h>

using namespace cvc::world;

namespace {
heightfield flat() {
  heightfield_params p;
  p.amp_m = 2.0;
  p.seed = 9;
  return heightfield(p);
}
std::vector<placed_prop> run(const scatter_params &sp, std::uint64_t seed) {
  return scatter(-120, -120, 120, 120, sp, flat(), surface_registry::builtin(), seed);
}
} // namespace

TEST(WorldScatter, Deterministic) {
  scatter_params sp = scatter_params::defaults();
  auto a = run(sp, 42);
  auto b = run(sp, 42);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(a[i].x, b[i].x);
    EXPECT_DOUBLE_EQ(a[i].y, b[i].y);
    EXPECT_EQ(a[i].recipe, b[i].recipe);
  }
}

TEST(WorldScatter, StreamIndependenceUnderCountChange) {
  // Adding trees must not move any building. Buildings use stream::building,
  // trees use stream::placement — independent by construction.
  scatter_params a = scatter_params::defaults();
  scatter_params b = a;
  b.tree_count = a.tree_count + 25;
  auto pa = run(a, 7), pb = run(b, 7);
  auto buildings = [](const std::vector<placed_prop> &v) {
    std::vector<placed_prop> out;
    for (const placed_prop &p : v)
      if (p.recipe == "office_block" || p.recipe == "brick_house")
        out.push_back(p);
    return out;
  };
  auto ba = buildings(pa), bb = buildings(pb);
  ASSERT_EQ(ba.size(), bb.size());
  ASSERT_GT(ba.size(), std::size_t(0));
  for (std::size_t i = 0; i < ba.size(); ++i) {
    EXPECT_DOUBLE_EQ(ba[i].x, bb[i].x);
    EXPECT_DOUBLE_EQ(ba[i].y, bb[i].y);
  }
}

TEST(WorldScatter, SeedChangesPlacement) {
  scatter_params sp = scatter_params::defaults();
  auto a = run(sp, 1), b = run(sp, 2);
  ASSERT_EQ(a.size(), b.size());
  int moved = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i].x != b[i].x || a[i].y != b[i].y)
      ++moved;
  EXPECT_GT(moved, int(a.size()) / 2);
}

TEST(WorldScatter, PropsCarryTaggedFootprints) {
  scatter_params sp = scatter_params::defaults();
  auto props = run(sp, 3);
  ASSERT_GT(props.size(), std::size_t(0));
  bool saw_tree_disc = false, saw_building_box = false;
  const surface_registry &reg = surface_registry::builtin();
  for (const placed_prop &p : props) {
    for (const footprint &f : p.footprints) {
      EXPECT_LT(f.klass, reg.size());
      if (p.recipe == "pine_monopodial" && f.k == footprint::disc && f.occupied)
        saw_tree_disc = true;
      if ((p.recipe == "office_block" || p.recipe == "brick_house") && f.k == footprint::obox &&
          f.occupied)
        saw_building_box = true;
    }
  }
  EXPECT_TRUE(saw_tree_disc);
  EXPECT_TRUE(saw_building_box);
}

TEST(WorldScatter, EmptySpeciesPlacesNothing) {
  scatter_params sp; // all species empty
  EXPECT_TRUE(run(sp, 5).empty());
}

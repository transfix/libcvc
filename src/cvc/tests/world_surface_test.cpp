/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/surface.h for the full header).
*/

// The surface registry contract (roadmap §7.2a / §16.2): hard ⇒ rho == 0 for
// every ontology variant, a complete role -> class map, a stable ontology hash,
// and the rf_material seam to the shared 12-name vocabulary.

#include <cstring>
#include <cvc/world/surface.h>
#include <gtest/gtest.h>
#include <set>
#include <string>

using namespace cvc::world;

TEST(WorldSurface, HasThirtySevenClasses) {
  EXPECT_EQ(surface_registry::builtin().size(), std::size_t(37)); // 32 §16.2 + 5 building shells
}

TEST(WorldSurface, HardClassesCarryZeroRhoEveryVariant) {
  // The load-bearing invariant (§7.2a): every hard class carries rho == 0 in
  // EVERY ontology variant, so a rho=1 wall never bleeds through the consumer's
  // blur. (The max SOFT risk differs by variant: 0.90 for merged_default /
  // soft_vegetation, 0.97 for strict_water_mud which raises water_shallow.)
  for (const char *v : {"merged_default", "soft_vegetation", "strict_water_mud"}) {
    const surface_registry &reg = surface_registry::variant(v);
    for (std::uint16_t id = 0; id < reg.size(); ++id) {
      const surface_class &c = reg[id];
      if (c.hard)
        EXPECT_EQ(c.rho, 0.0f) << v << " class " << c.name;
    }
  }
  auto max_soft = [](const surface_registry &reg) {
    float m = 0;
    for (std::uint16_t id = 0; id < reg.size(); ++id)
      m = std::max(m, reg[id].rho);
    return m;
  };
  EXPECT_FLOAT_EQ(max_soft(surface_registry::variant("merged_default")), 0.90f);
  EXPECT_FLOAT_EQ(max_soft(surface_registry::variant("soft_vegetation")), 0.90f);
  EXPECT_FLOAT_EQ(max_soft(surface_registry::variant("strict_water_mud")), 0.97f);
}

TEST(WorldSurface, RoleMapIsCompleteAndSensible) {
  const surface_registry &reg = surface_registry::builtin();
  using role = cvc::lsys::role;
  EXPECT_STREQ(reg[reg.class_for_role(role::trunk)].name, "tree_trunk");
  EXPECT_STREQ(reg[reg.class_for_role(role::foliage)].name, "bush_cover");
  EXPECT_STREQ(reg[reg.class_for_role(role::rock)].name, "boulder");
  EXPECT_STREQ(reg[reg.class_for_role(role::water)].name, "water_deep");
  EXPECT_STREQ(reg[reg.class_for_role(role::wall_concrete)].rf_material, "reinforced_concrete");
  EXPECT_STREQ(reg[reg.class_for_role(role::wall_brick)].rf_material, "brick");
  EXPECT_STREQ(reg[reg.class_for_role(role::wall_drywall)].rf_material, "drywall");
  EXPECT_STREQ(reg[reg.class_for_role(role::glass)].rf_material, "glass");
  EXPECT_STREQ(reg[reg.class_for_role(role::metal)].rf_material, "metal");
  // Every role maps to a valid, distinct-enough class.
  for (int i = 0; i < int(role::_count); ++i)
    EXPECT_LT(reg.class_for_role(role(i)), reg.size());
}

TEST(WorldSurface, EveryRfMaterialIsInTheTwelveNameVocabulary) {
  // The names cvc::world emits must be resolvable by the DBG featurizer (plus the
  // one new name, "rock", that this work adds on the DBG side).
  const std::set<std::string> vocab = {"open_air",
                                       "brick",
                                       "reinforced_concrete",
                                       "glass",
                                       "wood",
                                       "foliage",
                                       "drywall",
                                       "metal",
                                       "soil",
                                       "water",
                                       "glass_laminated",
                                       "composite_panel",
                                       "rock"};
  const surface_registry &reg = surface_registry::builtin();
  for (std::uint16_t id = 0; id < reg.size(); ++id)
    EXPECT_EQ(vocab.count(reg[id].rf_material), std::size_t(1))
        << reg[id].name << " -> " << reg[id].rf_material;
}

TEST(WorldSurface, VariantsDifferOnlyInSoftRho) {
  const surface_registry &def = surface_registry::variant("merged_default");
  const surface_registry &soft = surface_registry::variant("soft_vegetation");
  EXPECT_NE(def.ontology_hash(), soft.ontology_hash());
  EXPECT_EQ(def.size(), soft.size());
  EXPECT_LT(soft[def.by_name("grass")].rho, def[def.by_name("grass")].rho);
  // hard set is identical across variants.
  for (std::uint16_t id = 0; id < def.size(); ++id)
    EXPECT_EQ(def[id].hard, soft[id].hard);
}

TEST(WorldSurface, ByNameAndJson) {
  const surface_registry &reg = surface_registry::builtin();
  EXPECT_EQ(reg[reg.by_name("water_deep")].hard, true);
  EXPECT_THROW(reg.by_name("no_such_class"), std::out_of_range);
  std::string j = reg.to_json();
  EXPECT_NE(j.find("\"ontology\""), std::string::npos);
  EXPECT_NE(j.find("building_concrete"), std::string::npos);
  EXPECT_NE(j.find("penetration_db_per_m"), std::string::npos);
}

TEST(WorldSurface, OntologyHashStable) {
  EXPECT_EQ(surface_registry::builtin().ontology_hash(),
            surface_registry::variant("merged_default").ontology_hash());
}

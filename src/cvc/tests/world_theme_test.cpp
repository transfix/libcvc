/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// world_theme_test — model assets (sidecars, the catalog, reading an asset's glb), world themes
// (the building and tree archetype draws, biome rules, JSON), and a themed generated world.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/model_assets.h>
#include <cvc/world/world_theme.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::world;
namespace fs = std::filesystem;

namespace {

// A sidecar for an asset with these glTF bounds (+Y up) and a LOD ladder of `errors`.
std::string sidecar(const std::string &name, const std::string &kind, const double lo[3],
                    const double hi[3], const std::vector<double> &errors = {0.0, 0.05, 0.4}) {
  std::string s =
      "{\"schema\": \"cvc-model-asset/1\", \"name\": \"" + name + "\", \"kind\": \"" + kind +
      "\", \"tags\": [\"t\"], \"units\": \"m\", \"up\": \"+y\", \"front\": \"+z\"," +
      " \"bounds\": {\"min\": [" + std::to_string(lo[0]) + ", " + std::to_string(lo[1]) + ", " +
      std::to_string(lo[2]) + "], \"max\": [" + std::to_string(hi[0]) + ", " +
      std::to_string(hi[1]) + ", " + std::to_string(hi[2]) + "]}, \"lods\": [";
  for (std::size_t k = 0; k < errors.size(); ++k)
    s += (k ? ", " : "") + std::string("{\"file\": \"") + name +
         (k ? ".lod" + std::to_string(k) : std::string()) +
         ".glb\", \"triangles\": " + std::to_string(100000 >> (2 * k)) +
         ", \"world_error_m\": " + std::to_string(errors[k]) + "}";
  return s + "]}";
}

// The packaged assets' real bounds (cy-pca/graphics-assets building-*, tree-*), as catalog entries
// without files: placement reads only the bounds.
struct real {
  const char *name, *kind;
  double lo[3], hi[3];
};
const real kReal[] = {
    {"BrickCornerShop", "building", {-4.463, 0.0, -6.429}, {4.456, 8.0, 6.423}},
    {"Brownstone", "building", {-6.09, 0.0, -7.779}, {6.08, 16.0, 7.756}},
    {"ModernOffice", "building", {-14.479, 0.0, -8.27}, {14.466, 18.0, 8.29}},
    {"OakTree", "tree", {-5.168, 0.0, -4.357}, {5.277, 10.0, 4.362}},
    {"YoungOakTree", "tree", {-4.591, 0.0, -2.853}, {4.6, 10.0, 2.909}},
    {"PalmTree", "tree", {-2.972, 0.0, -3.01}, {3.01, 8.0, 2.895}},
};

void add_real(model_catalog &c) {
  for (const real &r : kReal)
    c.add(parse_model_asset(sidecar(r.name, r.kind, r.lo, r.hi), ""));
}

world_theme builtin(const std::string &name) {
  for (const world_theme &t : builtin_world_themes())
    if (t.name == name)
      return t;
  throw std::runtime_error("no built-in theme " + name);
}

// A glTF binary: one triangle with uvs, no material.
std::vector<unsigned char> one_triangle_glb() {
  const float pos[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
  const float uv[6] = {0, 0, 1, 0, 0, 1};
  const std::uint32_t idx[3] = {0, 1, 2};
  std::vector<unsigned char> bin(sizeof pos + sizeof uv + sizeof idx);
  std::memcpy(bin.data(), pos, sizeof pos);
  std::memcpy(bin.data() + sizeof pos, uv, sizeof uv);
  std::memcpy(bin.data() + sizeof pos + sizeof uv, idx, sizeof idx);
  std::string js =
      "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
      "\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,"
      "\"TEXCOORD_0\":1},\"indices\":2}]}],\"accessors\":["
      "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
      "\"min\":[0,0,0],\"max\":[1,1,0]},"
      "{\"bufferView\":1,\"componentType\":5126,\"count\":3,\"type\":\"VEC2\"},"
      "{\"bufferView\":2,\"componentType\":5125,\"count\":3,\"type\":\"SCALAR\"}],"
      "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
      "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":24},"
      "{\"buffer\":0,\"byteOffset\":60,\"byteLength\":12}],"
      "\"buffers\":[{\"byteLength\":72}]}";
  while (js.size() % 4)
    js += ' ';
  std::vector<unsigned char> out;
  auto u32 = [&out](std::uint32_t v) {
    for (int i = 0; i < 4; ++i)
      out.push_back((v >> (8 * i)) & 0xFF);
  };
  u32(0x46546C67);
  u32(2);
  u32(std::uint32_t(12 + 8 + js.size() + 8 + bin.size()));
  u32(std::uint32_t(js.size()));
  u32(0x4E4F534A);
  out.insert(out.end(), js.begin(), js.end());
  u32(std::uint32_t(bin.size()));
  u32(0x004E4942);
  out.insert(out.end(), bin.begin(), bin.end());
  return out;
}

fs::path temp_dir(const std::string &tag) {
  fs::path d = fs::temp_directory_path() / ("cvc_world_theme_test_" + tag);
  std::error_code ec;
  fs::remove_all(d, ec);
  fs::create_directories(d);
  return d;
}

void write_file(const fs::path &p, const std::string &s) {
  std::ofstream(p, std::ios::binary) << s;
}

generated_world_params themed_world(const std::string &theme) {
  generated_world_params p;
  p.seed = 11;
  p.half_m = 300.0;
  p.trees = 220;
  p.rocks = 0;
  p.buildings = 60;
  p.tree_gen = 3;
  p.amp_m = 30.0;
  p.preset = int(world_preset::urban);
  p.edge = int(world_edge::shore); // a coast, for the palms
  p.theme = theme;
  return p;
}

} // namespace

// ── model assets ─────────────────────────────────────────────────────────────────────────────────

TEST(WorldModelAssets, ASidecarIsReadIntoTheWorldFrame) {
  const real &r = kReal[2]; // the office: 28.9 m across its front (glTF x), 16.6 m deep (z)
  const model_asset a = parse_model_asset(sidecar(r.name, r.kind, r.lo, r.hi), "/assets");
  EXPECT_EQ(a.name, "ModernOffice");
  EXPECT_EQ(a.kind, model_kind::building);
  EXPECT_TRUE(a.has_tag("t"));
  EXPECT_NEAR(a.half_x, 0.5 * (14.466 + 14.479), 1e-9); // across the front
  EXPECT_NEAR(a.half_y, 0.5 * (8.29 + 8.27), 1e-9);     // front to back
  EXPECT_NEAR(a.height, 18.0, 1e-9);
  EXPECT_NEAR(a.base_z, 0.0, 1e-12);
  EXPECT_NEAR(a.centre_x, -0.5 * (14.466 - 14.479), 1e-9); // glTF x is mirrored into world x
  ASSERT_EQ(a.lods.size(), 3u);
  EXPECT_EQ(a.lods[1].file, "ModernOffice.lod1.glb");
  EXPECT_EQ(fs::path(a.path(1)), fs::path("/assets") / "ModernOffice.lod1.glb");
  EXPECT_EQ(fs::path(a.path(9)), fs::path("/assets") / "ModernOffice.lod2.glb"); // clamped
  // model_to_world is a rotation: glTF's front (+z) is the world's +Y, its up (+y) the world's +Z.
  const double front[3] = {0, 0, 1}, up[3] = {0, 1, 0};
  double f[3], u[3];
  model_to_world(front, f);
  model_to_world(up, u);
  EXPECT_EQ(f[1], 1.0);
  EXPECT_EQ(u[2], 1.0);
  // And back through JSON.
  const model_asset b = parse_model_asset(model_asset_json(a), "/assets");
  EXPECT_NEAR(b.half_x, a.half_x, 1e-6);
  EXPECT_NEAR(b.half_y, a.half_y, 1e-6);
  EXPECT_NEAR(b.centre_x, a.centre_x, 1e-6);
  EXPECT_NEAR(b.centre_y, a.centre_y, 1e-6);
  EXPECT_EQ(b.lods.size(), a.lods.size());
  EXPECT_DOUBLE_EQ(b.lods[2].world_error_m, a.lods[2].world_error_m);
}

TEST(WorldModelAssets, ABadSidecarIsRefused) {
  const double lo[3] = {-1, 0, -1}, hi[3] = {1, 2, 1};
  const std::string good = sidecar("A", "tree", lo, hi);
  EXPECT_NO_THROW(parse_model_asset(good, ""));
  std::string s = good;
  s.replace(s.find("cvc-model-asset/1"), 17, "cvc-model-asset/9");
  EXPECT_THROW(parse_model_asset(s, ""), std::runtime_error);
  EXPECT_THROW(parse_model_asset(sidecar("A", "spaceship", lo, hi), ""), std::runtime_error);
  EXPECT_THROW(parse_model_asset(sidecar("A", "tree", lo, hi, {0.0, 0.5, 0.2}), ""),
               std::runtime_error); // an error that falls down the ladder
  EXPECT_THROW(parse_model_asset(sidecar("A", "tree", lo, hi, {0.1}), ""), std::runtime_error);
  EXPECT_THROW(parse_model_asset(sidecar("A", "tree", lo, hi, {}), ""), std::runtime_error);
  const double flat[3] = {1, 2, 1};
  EXPECT_THROW(parse_model_asset(sidecar("A", "tree", flat, hi), ""), std::runtime_error);
  EXPECT_THROW(parse_model_asset("{not json", ""), std::runtime_error);
}

TEST(WorldModelAssets, TheCatalogScansADirectoryAndKeepsWhatDoesNotParse) {
  const fs::path d = temp_dir("scan");
  for (int i = 0; i < 2; ++i)
    write_file(d / (std::string(kReal[i].name) + ".asset.json"),
               sidecar(kReal[i].name, kReal[i].kind, kReal[i].lo, kReal[i].hi));
  write_file(d / "Broken.asset.json", "{\"schema\": \"nope\"}");
  write_file(d / "ignored.json", "{}");
  model_catalog c;
  EXPECT_EQ(c.scan(d.string()), 2u);
  EXPECT_EQ(c.size(), 2u);
  EXPECT_EQ(c.errors().size(), 1u);
  EXPECT_EQ(c.names(), (std::vector<std::string>{"BrickCornerShop", "Brownstone"}));
  model_asset a;
  ASSERT_TRUE(c.find("Brownstone", a));
  EXPECT_EQ(fs::path(a.dir), d);
  EXPECT_FALSE(c.find("PalmTree", a));
  EXPECT_EQ(c.scan((d / "missing").string()), 0u);
  fs::remove_all(d);
}

TEST(WorldModelAssets, AnAssetsLodsReadAsModels) {
  cvc::app app;
  const fs::path d = temp_dir("read");
  const std::vector<unsigned char> glb = one_triangle_glb();
  for (const char *f : {"Tri.glb", "Tri.lod1.glb"})
    std::ofstream(d / f, std::ios::binary)
        .write(reinterpret_cast<const char *>(glb.data()), std::streamsize(glb.size()));
  const double lo[3] = {0, 0, 0}, hi[3] = {1, 1, 0.01};
  write_file(d / "Tri.asset.json", sidecar("Tri", "prop", lo, hi, {0.0, 0.1}));
  model_catalog c;
  ASSERT_EQ(c.scan(d.string()), 1u);
  model_asset a;
  ASSERT_TRUE(c.find("Tri", a));
  for (int k = 0; k < 2; ++k) {
    cvc::model m;
    try {
      m = cvc::read_model(app, a.path(k));
    } catch (const std::exception &e) {
      fs::remove_all(d);
      GTEST_SKIP() << "no glb reader in this build: " << e.what();
    }
    const cvc::geometry g = m.merged();
    EXPECT_EQ(g.num_tris(), 1u) << a.path(k);
    EXPECT_EQ(g.const_uvs().size(), 3u) << a.path(k);
  }
  fs::remove_all(d);
}

TEST(WorldModelAssets, TheAppsCatalogTakesAssetsAddedToIt) {
  cvc::app app;
  model_catalog &c = model_catalog_of(app);
  EXPECT_EQ(&c, &model_catalog_of(app)); // one per app
  add_real(c);
  EXPECT_TRUE(c.contains("PalmTree"));
  cvc::app other;
  for (const real &r : kReal) // another app's catalog is its own (whatever is installed)
    if (model_catalog_of(other).contains(r.name)) {
      model_asset a;
      model_catalog_of(other).find(r.name, a);
      EXPECT_FALSE(a.dir.empty()) << r.name << " is installed, not this test's";
    }
}

// ── themes ───────────────────────────────────────────────────────────────────────────────────────

TEST(WorldTheme, TheBuiltInsAreTheAppsAndARegisteredThemeReplacesOrAdds) {
  cvc::app app;
  const std::vector<std::string> names = world_theme_names(app);
  for (const char *n : {"temperate_town", "coastal_town", "tropical"})
    EXPECT_NE(std::find(names.begin(), names.end(), n), names.end()) << n;
  world_theme t;
  ASSERT_TRUE(find_world_theme(app, "coastal_town", t));
  EXPECT_FALSE(t.buildings.empty());
  EXPECT_FALSE(t.trees.empty());
  EXPECT_FALSE(find_world_theme(app, "desert", t));
  world_theme desert;
  desert.name = "desert";
  desert.trees = {tree_archetype_rule{}};
  desert.trees[0].asset = "PalmTree";
  register_world_theme(app, desert);
  ASSERT_TRUE(find_world_theme(app, "desert", t));
  EXPECT_EQ(t.trees.size(), 1u);
  desert.tree_share = 0.25;
  register_world_theme(app, desert); // replaces, does not add
  ASSERT_TRUE(find_world_theme(app, "desert", t));
  EXPECT_DOUBLE_EQ(t.tree_share, 0.25);
  EXPECT_EQ(world_theme_names(app).size(), names.size() + 1);
  cvc::app other; // the app's own
  EXPECT_FALSE(find_world_theme(other, "desert", t));
}

TEST(WorldTheme, AThemeRoundTripsThroughJsonAndABadOneIsRefused) {
  const world_theme &t = builtin("coastal_town");
  const world_theme u = world_theme_from_json(world_theme_json(t));
  EXPECT_EQ(u.name, t.name);
  ASSERT_EQ(u.buildings.size(), t.buildings.size());
  ASSERT_EQ(u.trees.size(), t.trees.size());
  for (std::size_t i = 0; i < t.buildings.size(); ++i) {
    EXPECT_EQ(u.buildings[i].asset, t.buildings[i].asset);
    EXPECT_EQ(u.buildings[i].span, t.buildings[i].span);
    EXPECT_DOUBLE_EQ(u.buildings[i].min_urbanity, t.buildings[i].min_urbanity);
  }
  for (std::size_t i = 0; i < t.trees.size(); ++i) {
    EXPECT_EQ(u.trees[i].asset, t.trees[i].asset);
    EXPECT_EQ(u.trees[i].biome.max_water_m, t.trees[i].biome.max_water_m); // infinity too
    EXPECT_EQ(u.trees[i].biome.min_height_m, t.trees[i].biome.min_height_m);
  }
  // Fields left out keep their defaults.
  const world_theme d = world_theme_from_json("{\"name\": \"x\", \"trees\": [{\"asset\": \"P\"}]}");
  EXPECT_DOUBLE_EQ(d.tree_share, world_theme().tree_share);
  EXPECT_TRUE(std::isinf(d.trees[0].biome.max_height_m));
  for (const char *bad :
       {"{\"buildings\": [{\"weight\": 1}]}", "{\"trees\": [{\"asset\": \"P\", \"weight\": -1}]}",
        "{\"tree_share\": 1.5}",
        "{\"buildings\": [{\"asset\": \"B\", \"min_urbanity\": 0.9, \"max_urbanity\": 0.1}]}",
        "{\"trees\": [{\"asset\": \"P\", \"biome\": {\"min_water_m\": 50, \"max_water_m\": 10}}]}",
        "{\"buildings\": [{\"asset\": \"B\", \"span\": 0}]}", "[1, 2"})
    EXPECT_THROW(world_theme_from_json(bad), std::runtime_error) << bad;
}

TEST(WorldTheme, ALotTakesAnArchetypeThatFitsItsUrbanityAndFrontage) {
  model_catalog c;
  add_real(c);
  world_theme t = builtin("temperate_town");
  t.building_share = 1.0;
  int shop = -1, brown = -1, office = -1;
  for (int i = 0; i < int(t.buildings.size()); ++i)
    (t.buildings[i].asset == "BrickCornerShop" ? shop
     : t.buildings[i].asset == "Brownstone"    ? brown
                                               : office) = i;
  std::map<int, int> outskirts, core;
  for (int i = 0; i < 400; ++i) {
    lot_view L;
    L.x = 37.0 * i;
    L.y = -11.0 * i;
    L.frontage = 18.0;
    L.depth = 30.0;
    L.urbanity = 0.1;
    const building_pick a = pick_building(t, c, L, 7);
    ++outskirts[a.archetype];
    if (a.archetype >= 0) {
      EXPECT_GE(a.scale, t.buildings[a.archetype].min_scale);
      EXPECT_LE(a.scale, t.buildings[a.archetype].max_scale);
    }
    L.urbanity = 0.9;
    ++core[pick_building(t, c, L, 7).archetype];
  }
  // Out of the cores only the corner shop's band holds; in them the shop's band does not.
  EXPECT_EQ(outskirts[shop], 400);
  EXPECT_EQ(core[shop], 0);
  EXPECT_GT(core[brown], 0);
  // The office is two lots wide: never on one lot, only on a span of two.
  EXPECT_EQ(core[office], 0);
  lot_view two;
  two.frontage = 36.0;
  two.depth = 30.0;
  two.urbanity = 0.9;
  two.lots = 2;
  int offices = 0;
  for (int i = 0; i < 50; ++i) {
    two.x = 13.0 * i;
    const building_pick b = pick_building(t, c, two, 7);
    ASSERT_EQ(b.archetype, office); // the only archetype allowed to span two lots
    EXPECT_LE(2.0 * 14.4725 * b.scale + 2.0, 36.0 + 1e-6); // and it fits, a metre clear a side
    ++offices;
  }
  EXPECT_EQ(offices, 50);
  // Nothing fits a narrow lot; a lot whose draw is over building_share keeps the massing.
  lot_view narrow;
  narrow.frontage = 6.0;
  narrow.depth = 30.0;
  narrow.urbanity = 0.5;
  EXPECT_EQ(pick_building(t, c, narrow, 7).archetype, -1);
  t.building_share = 0.0;
  lot_view any;
  any.frontage = 18.0;
  any.depth = 30.0;
  EXPECT_EQ(pick_building(t, c, any, 7).archetype, -1);
}

TEST(WorldTheme, ABuildingDrawIsKeyedOnItsSpotAndSkipsWhatIsNotInstalled) {
  model_catalog c;
  add_real(c);
  world_theme t = builtin("temperate_town");
  t.building_share = 1.0;
  lot_view L;
  L.x = 120.4;
  L.y = -55.0;
  L.frontage = 18.0;
  L.depth = 30.0;
  L.urbanity = 0.5;
  const building_pick a = pick_building(t, c, L, 3), b = pick_building(t, c, L, 3);
  EXPECT_EQ(a.archetype, b.archetype);
  EXPECT_EQ(a.scale, b.scale);
  // With only the brownstone installed, every lot that takes one takes the brownstone.
  model_catalog only;
  only.add(parse_model_asset(sidecar(kReal[1].name, kReal[1].kind, kReal[1].lo, kReal[1].hi), ""));
  for (int i = 0; i < 50; ++i) {
    L.x = 29.0 * i;
    const building_pick p = pick_building(t, only, L, 3);
    if (p.archetype >= 0)
      EXPECT_EQ(t.buildings[p.archetype].asset, "Brownstone");
  }
  model_catalog none;
  EXPECT_EQ(pick_building(t, none, L, 3).archetype, -1);
}

TEST(WorldTheme, PalmsGrowOnTheCoastAndOaksInland) {
  model_catalog c;
  add_real(c);
  world_theme t = builtin("coastal_town");
  t.tree_share = 1.0;
  std::set<std::string> coast, inland, steep;
  for (int i = 0; i < 300; ++i) {
    tree_site s;
    s.x = 17.0 * i;
    s.y = 3.0 * i;
    s.height_m = 2.0;
    s.slope_deg = 5.0;
    s.water_m = 20.0; // a beach
    const tree_pick p = pick_tree(t, c, s, 5);
    ASSERT_GE(p.archetype, 0);
    coast.insert(t.trees[p.archetype].asset);
    EXPECT_GE(p.yaw, 0.0);
    EXPECT_LT(p.yaw, 6.3);
    s.height_m = 25.0; // up the hill, far from the water
    s.water_m = 400.0;
    const tree_pick q = pick_tree(t, c, s, 5);
    ASSERT_GE(q.archetype, 0);
    inland.insert(t.trees[q.archetype].asset);
    s.slope_deg = 60.0; // a cliff: nothing the theme names grows there
    if (pick_tree(t, c, s, 5).archetype >= 0)
      steep.insert("something");
  }
  EXPECT_EQ(coast, (std::set<std::string>{"PalmTree"}));
  EXPECT_EQ(inland, (std::set<std::string>{"OakTree", "YoungOakTree"}));
  EXPECT_TRUE(steep.empty());
  // The temperate town plants no palm anywhere, and none on the beach at all.
  const world_theme &temperate = builtin("temperate_town");
  for (int i = 0; i < 100; ++i) {
    tree_site s;
    s.x = 23.0 * i;
    s.height_m = 1.0;
    s.water_m = 10.0;
    EXPECT_EQ(pick_tree(temperate, c, s, 5).archetype, -1);
  }
}

TEST(WorldTheme, BiomeBoundsAreInclusive) {
  biome_rule b;
  b.min_height_m = 1.0;
  b.max_height_m = 10.0;
  b.max_slope_deg = 20.0;
  b.min_water_m = 5.0;
  b.max_water_m = 50.0;
  tree_site s;
  s.height_m = 1.0;
  s.slope_deg = 20.0;
  s.water_m = 50.0;
  EXPECT_TRUE(biome_holds(b, s));
  s.water_m = 50.5;
  EXPECT_FALSE(biome_holds(b, s));
  s.water_m = 5.0;
  s.height_m = 0.99;
  EXPECT_FALSE(biome_holds(b, s));
  EXPECT_TRUE(biome_holds(biome_rule(), tree_site())); // the defaults hold anywhere
}

// ── a themed world ───────────────────────────────────────────────────────────────────────────────

TEST(WorldThemedGeneration, AThemedWorldPlacesModelBuildingsAndTrees) {
  cvc::app app;
  model_catalog_of(app).clear(); // this test's assets, not whatever is installed
  add_real(model_catalog_of(app));
  const generated_world plain = generate_world(app, themed_world(""));
  EXPECT_TRUE(plain.models.empty());
  EXPECT_TRUE(plain.model_assets.empty());
  const generated_world w = generate_world(app, themed_world("coastal_town"));
  ASSERT_FALSE(w.models.empty());
  EXPECT_GT(w.model_buildings, 0u);
  EXPECT_GT(w.model_trees, 0u);
  EXPECT_EQ(w.model_buildings + w.model_trees, w.models.size());
  EXPECT_LE(w.building_count, std::size_t(themed_world("").buildings));
  // Fewer L-system trees than the plain world: the theme took some of their spots.
  EXPECT_LT(w.props.const_points().size(), plain.props.const_points().size());
  std::size_t buildings = 0, palms = 0;
  for (const generated_world::model_instance &m : w.models) {
    ASSERT_GE(m.asset, 0);
    ASSERT_LT(m.asset, int(w.model_assets.size()));
    const model_asset &a = w.model_assets[m.asset];
    EXPECT_TRUE(std::isfinite(m.x) && std::isfinite(m.y) && std::isfinite(m.z));
    EXPECT_LE(std::fabs(m.x), w.params.half_m + 40.0);
    EXPECT_GT(m.scale, 0.5);
    if (a.kind == model_kind::building) {
      ++buildings;
      // On its site's highest ground (a plinth under it on a slope): not below the ground at its
      // origin, nor more than the most relief a model stands on (6 m) over it.
      EXPECT_GE(m.z, w.height(m.x, m.y) - 1.5) << a.name;
      EXPECT_LE(m.z, w.height(m.x, m.y) + 6.5) << a.name;
    } else {
      ASSERT_EQ(a.kind, model_kind::tree);
      EXPECT_NEAR(m.z, w.height(m.x, m.y), 1.0) << a.name; // standing on the ground
      if (a.name == "PalmTree") {
        ++palms;
        EXPECT_LE(m.z - w.params.water_level_m, 10.5); // the coast's palms stay low
      }
    }
  }
  EXPECT_EQ(buildings, w.model_buildings);
  EXPECT_GT(palms, 0u) << "a shore-edged coastal town has palms";
  // The model buildings collide, as boxes; the trees do not.
  const world_geometry g = w.geometry();
  bool boxes = false;
  for (const geometry_item &it : g.items())
    if (it.name == "model_buildings") {
      boxes = true;
      EXPECT_TRUE(it.has(role_collide));
      EXPECT_FALSE(it.has(role_render));
      EXPECT_EQ(it.geometry->num_tris(), 12u * w.model_buildings);
    }
  EXPECT_TRUE(boxes);
}

TEST(WorldThemedGeneration, AThemedWorldIsDeterministic) {
  cvc::app app;
  model_catalog_of(app).clear();
  add_real(model_catalog_of(app));
  const generated_world a = generate_world(app, themed_world("tropical")),
                        b = generate_world(app, themed_world("tropical"));
  ASSERT_EQ(a.models.size(), b.models.size());
  ASSERT_FALSE(a.models.empty());
  for (std::size_t i = 0; i < a.models.size(); ++i) {
    EXPECT_EQ(a.models[i].asset, b.models[i].asset);
    EXPECT_EQ(a.models[i].x, b.models[i].x);
    EXPECT_EQ(a.models[i].y, b.models[i].y);
    EXPECT_EQ(a.models[i].z, b.models[i].z);
    EXPECT_EQ(a.models[i].yaw, b.models[i].yaw);
    EXPECT_EQ(a.models[i].scale, b.models[i].scale);
  }
}

TEST(WorldThemedGeneration, WithoutItsAssetsAThemeBuildsAsIfItNamedNone) {
  cvc::app app;
  model_catalog_of(app).clear();
  const generated_world plain = generate_world(app, themed_world(""));
  const generated_world w = generate_world(app, themed_world("coastal_town"));
  EXPECT_TRUE(w.models.empty());
  EXPECT_EQ(w.building_count, plain.building_count);
  EXPECT_EQ(w.buildings.const_points().size(), plain.buildings.const_points().size());
  EXPECT_EQ(w.props.const_points().size(), plain.props.const_points().size());
}

TEST(WorldThemedGeneration, AnUnknownThemeIsAnError) {
  cvc::app app;
  EXPECT_THROW(generate_world(app, themed_world("no_such_theme")), std::invalid_argument);
}

TEST(WorldThemedGeneration, TheThemeIsInTheParamsJsonOnlyWhenSet) {
  const std::string none = generated_world_params_json(themed_world(""));
  EXPECT_EQ(none.find("theme"), std::string::npos); // a world without keeps its old JSON
  const std::string some = generated_world_params_json(themed_world("tropical"));
  EXPECT_NE(some.find("\"theme\""), std::string::npos);
  EXPECT_EQ(generated_world_params_from_json(some).theme, "tropical");
}

TEST(WorldThemedGeneration, ABundleCarriesTheModels) {
  cvc::app app;
  if (!cvc::can_write_model_to_memory(".glb"))
    GTEST_SKIP() << "no glb writer (Assimp off)";
  model_catalog_of(app).clear();
  add_real(model_catalog_of(app));
  const generated_world w = generate_world(app, themed_world("coastal_town"));
  ASSERT_FALSE(w.models.empty());
  const city_bundle_files files = generated_world_bundle(app, w, "test");
  ASSERT_EQ(files.count("models.json"), 1u);
  const city_bundle b = load_city_bundle(app, city_bundle_memory_reader(files), "themed");
  const bundle_world bw = world_from_bundle(app, b, city_bundle_memory_reader(files));
  ASSERT_TRUE(bw.world);
  const generated_world &r = *bw.world;
  EXPECT_EQ(r.params.theme, "coastal_town");
  ASSERT_EQ(r.models.size(), w.models.size());
  ASSERT_EQ(r.model_assets.size(), w.model_assets.size());
  EXPECT_EQ(r.model_buildings, w.model_buildings);
  for (std::size_t k = 0; k < w.model_assets.size(); ++k) {
    EXPECT_EQ(r.model_assets[k].name, w.model_assets[k].name);
    EXPECT_NEAR(r.model_assets[k].half_x, w.model_assets[k].half_x, 1e-6);
    EXPECT_EQ(r.model_assets[k].lods.size(), w.model_assets[k].lods.size());
  }
  for (std::size_t i = 0; i < w.models.size(); i += 7) {
    EXPECT_EQ(r.models[i].asset, w.models[i].asset);
    EXPECT_NEAR(r.models[i].x, w.models[i].x, 1e-6);
    EXPECT_NEAR(r.models[i].yaw, w.models[i].yaw, 1e-9);
  }
  // The model buildings' boxes are not drawn back as buildings.
  EXPECT_EQ(r.buildings.const_tris().size(),
            w.buildings.const_tris().size()); // the glb welds points
  // A world with no models writes no models.json.
  const generated_world plain = generate_world(app, themed_world(""));
  EXPECT_EQ(generated_world_bundle(app, plain, "test").count("models.json"), 0u);
}

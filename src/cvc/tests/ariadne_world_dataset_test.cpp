/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A generated world as a dataset (ARIADNE_WORLDS section 18.8, GB2): data-generate makes the world
// a version 2 city bundle would load as, a generated_world node's world is its dataset (its body
// that dataset's world, there when the world is built), Regenerate replaces it in place, and a
// document's data: block can name a made dataset that a data node's body stands on.

#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/core/app.h>
#include <cvc/core/types.h> // CVC_VERSION_STRING
#include <cvc/geometry/height_field.h>
#include <cvc/model/model_file_io.h> // can_write_model_to_memory
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/world_geometry.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::ariadne;

namespace {

#define REQUIRE_YAML()                                                                             \
  do {                                                                                             \
    if (!AppRuntime::have_yaml())                                                                  \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

// A world as a bundle needs the glb writer (Assimp). Without it a generated world's dataset is
// the world as made.
bool glb() { return cvc::can_write_model_to_memory(".glb"); }

std::string val(cvc::app &app, const std::string &path) {
  cvc::state *s = cvc::state::instance(app).findThroughLinks(path);
  return s ? s->value() : std::string();
}

// A small world: a few hundred metres, a handful of props.
const char *kParams = R"({"seed":5,"half_m":150,"trees":6,"rocks":3,"buildings":6})";

// The walls add_world_geometry puts around a terrain's edge, bodies of the terrain's node.
constexpr std::size_t kEdgeWalls = 4;

std::size_t collide_items(const cvc::world::world_geometry &g) {
  std::size_t n = 0;
  for (const cvc::world::geometry_item &it : g.items())
    if (it.has(cvc::world::role_collide))
      ++n;
  return n;
}

// The world kParams makes, made here, and its bundle's hash.
std::string direct_hash(cvc::app &app, const std::string &params,
                        cvc::world::generated_world *out = nullptr) {
  const cvc::world::building_library &library =
      generated_world_library(app, std::string(), std::string(), "test", nullptr);
  cvc::world::generated_world w = cvc::world::generate_world(
      app, cvc::world::generated_world_params_from_json(params), &library);
  const std::string hash =
      cvc::world::city_bundle_hash(cvc::world::generated_world_bundle(app, w, CVC_VERSION_STRING));
  if (out)
    *out = std::move(w);
  return hash;
}

} // namespace

// (data-generate world generated_world PARAMS): the world made on the store's worker, written as a
// version 2 bundle and decoded as a loaded city's is, the same bundle the world made directly
// writes. The same params name the same dataset; other params, another.
TEST(WorldDataset, DataGenerateMakesTheWorldItsBundleLoadsAs) {
  if (!glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  cvc::app app;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  DataSpec spec;
  spec.name = "world";
  spec.generate = "generated_world";
  spec.params = kParams;
  const std::string node = store->bind("", spec);
  ASSERT_TRUE(store->wait("data.world", 300.0)) << val(app, node + ".error");
  EXPECT_EQ(val(app, "data.world.status"), "ready");
  EXPECT_EQ(val(app, "data.world.as"), "city_bundle");
  EXPECT_EQ(val(app, "data.world.version"), "2");
  EXPECT_EQ(val(app, "data.world.generator"), "generated_world");
  EXPECT_EQ(val(app, "data.world.generator.seed"), "5");
  EXPECT_FALSE(val(app, "data.world.water_level").empty());
  EXPECT_FALSE(data_files(app, "data.world").empty()); // the bundle's files, as a load's

  std::shared_ptr<const cvc::world::generated_world> made =
      dataset_generated_world(app, "data.world");
  ASSERT_NE(made, nullptr);
  EXPECT_EQ(made->params.half_m, 150.0);
  cvc::world::generated_world direct;
  EXPECT_EQ(val(app, "data.world.hash"), direct_hash(app, kParams, &direct));

  // Its world is the bundle read back: the terrain's heights as stored, its collide items.
  std::shared_ptr<const cvc::world::world_geometry> world = dataset_world(app, "data.world");
  ASSERT_NE(world, nullptr);
  ASSERT_NE(world->terrain(), nullptr);
  EXPECT_EQ(world->terrain()->heights(), direct.terrain->heights());
  EXPECT_EQ(collide_items(*world), collide_items(direct.geometry()));

  DataSpec again = spec;
  again.name = "again";
  EXPECT_EQ(store->bind("", again), node); // the same params: the same dataset, loaded once
  DataSpec other = spec;
  other.params = R"({"seed":6,"half_m":150,"trees":6,"rocks":3,"buildings":6})";
  EXPECT_NE(data_key(other), data_key(spec));
}

TEST(WorldDataset, AnUnknownGeneratorFailsTheLoad) {
  cvc::app app;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  DataSpec spec;
  spec.name = "world";
  spec.generate = "no_such_generator";
  store->bind("", spec);
  EXPECT_FALSE(store->wait("data.world", 60.0));
  EXPECT_EQ(val(app, "data.world.status"), "error");
  EXPECT_NE(val(app, "data.world.error").find("no generator 'no_such_generator'"),
            std::string::npos)
      << val(app, "data.world.error");
  EXPECT_TRUE(has_data_generator(app, "generated_world"));
}

// A generated_world node's world is its dataset, data.<id>: ready when the world is built, with
// the node's bodies that dataset's world from the start. Regenerate makes a new world into the
// same dataset (its rev on), and the bodies take it at that sync.
TEST(WorldDataset, AGeneratedWorldNodeIsItsDataset) {
  REQUIRE_YAML();
  cvc::app app;
  LoadResult r = load_string(app, R"(
scene:
  physics: {}
  nodes:
    - node: world
      type: generated_world
      seed: 5
      half: 150
      trees: 6
      rocks: 3
      buildings: 6
      body: {}
)");
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(val(app, "data.world.status"), "ready");
  std::shared_ptr<const cvc::world::world_geometry> world = dataset_world(app, "data.world");
  ASSERT_NE(world, nullptr);
  EXPECT_EQ(w->world_geometry("world"), world);
  EXPECT_EQ(w->generated_world("world"), dataset_generated_world(app, "data.world"));
  EXPECT_EQ(w->bodies_of("world").size(), collide_items(*world) + kEdgeWalls);
  if (glb()) {
    EXPECT_EQ(val(app, "data.world.version"), "2");
    // The node's world is the one data-generate makes from its params (its preset's, then its
    // own), which the bundle's generator block holds.
    const std::string params =
        cvc::world::generated_world_params_json(w->generated_world("world")->params);
    EXPECT_EQ(cvc::world::generated_world_params_json(cvc::world::generated_world_params_from_json(
                  val(app, "data.world.generator.params"))),
              params);
    EXPECT_EQ(val(app, "data.world.hash"), direct_hash(app, params));
  }

  const std::string store = DataStore::of(app)->resolve("data.world");
  const int rev = std::stoi(val(app, "data.world.rev"));
  write<double>(app, "world.seed", 9);
  write<std::string>(app, "world.request", "regenerate");
  w->sync(&warnings);
  EXPECT_EQ(DataStore::of(app)->resolve("data.world"), store); // replaced in place
  EXPECT_EQ(std::stoi(val(app, "data.world.rev")), rev + 1);
  ASSERT_NE(w->generated_world("world"), nullptr);
  EXPECT_EQ(w->generated_world("world")->params.seed, 9u);
  EXPECT_EQ(w->world_geometry("world"), dataset_world(app, "data.world"));
  EXPECT_NE(w->world_geometry("world"), world);
  EXPECT_EQ(w->bodies_of("world").size(),
            collide_items(*dataset_world(app, "data.world")) + kEdgeWalls);
  if (glb())
    EXPECT_EQ(val(app, "data.world.generator.seed"), "9");
}

// A document's data: block names a made dataset (generate:, params:), and a data node stands on
// it as on a loaded city. Its params are written as YAML, so the dataset is another request than
// the JSON one, and the same world: the same bundle hash.
TEST(WorldDataset, ADocumentsMadeDatasetIsADataNodesWorld) {
  REQUIRE_YAML();
  cvc::app app;
  LoadResult r = load_string(app, R"(
data:
  world:
    generate: generated_world
    params: { seed: 5, half_m: 150, trees: 6, rocks: 3, buildings: 6 }
scene:
  physics: {}
  nodes:
    - node: city
      source: { data: world }
      body: {}
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.data.size(), 1u);
  EXPECT_EQ(r.data[0].generate, "generated_world");
  EXPECT_NE(r.data[0].params.find("\"seed\":\"5\""), std::string::npos) << r.data[0].params;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  for (const DataSpec &d : r.data)
    store->bind("", d);
  ASSERT_TRUE(store->wait("data.world", 300.0)) << val(app, "data.world.error");
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  std::shared_ptr<const cvc::world::world_geometry> world = dataset_world(app, "data.world");
  ASSERT_NE(world, nullptr);
  EXPECT_EQ(w->bodies_of("city").size(), collide_items(*world) + kEdgeWalls); // at build
  EXPECT_EQ(w->world_geometry("city"), world);
  EXPECT_NE(w->generated_world("city"), nullptr); // the world as made, beside the dataset
  if (glb())
    EXPECT_EQ(val(app, "data.world.hash"), direct_hash(app, kParams));
}

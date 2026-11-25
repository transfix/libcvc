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

#include <array>
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
#include <cvc/nav/world_grid.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <memory>
#include <random>
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
  // Its world is the dataset's, with the still water its level (0 m) puts under it.
  ASSERT_NE(w->world_geometry("city"), nullptr);
  EXPECT_EQ(w->world_geometry("city")->terrain(), world->terrain());
  EXPECT_NE(w->world_geometry("city")->find_volume("water"), nullptr);
  EXPECT_NE(w->generated_world("city"), nullptr); // the world as made, beside the dataset
  if (glb())
    EXPECT_EQ(val(app, "data.world.hash"), direct_hash(app, kParams));
}

// ── Region and water level on the data node (GB2 part 2) ──

// crop_world_geometry: the terrain to the samples that cover the region (its cell materials with
// it), a mesh to the faces whose centroid is inside (their face materials with them, the unused
// vertices dropped), and a region that misses the terrain or is backwards refused.
TEST(WorldRegion, ACropKeepsWhatIsInside) {
  cvc::world::world_geometry w;
  std::vector<float> h(11 * 11);
  std::vector<cvc::height_field::face_material_t> m(10 * 10);
  for (int r = 0; r < 11; ++r)
    for (int c = 0; c < 11; ++c)
      h[r * 11 + c] = float(r * 100 + c);
  for (int i = 0; i < 100; ++i)
    m[i] = static_cast<cvc::height_field::face_material_t>(i % 3);
  cvc::world::geometry_item terrain;
  terrain.name = "terrain";
  terrain.geometry = std::make_shared<const cvc::height_field>(11, 11, 0.0, 0.0, 10.0, 10.0, h, m);
  w.add(terrain);
  auto mesh = std::make_shared<cvc::geometry>();
  for (const auto &p : std::vector<std::array<double, 3>>{
           {30, 30, 0}, {32, 30, 0}, {30, 32, 0}, {90, 90, 0}, {92, 90, 0}, {90, 92, 0}})
    mesh->points().push_back({p[0], p[1], p[2]});
  mesh->tris().push_back({0, 1, 2});
  mesh->tris().push_back({3, 4, 5});
  mesh->face_materials().push_back(1);
  mesh->face_materials().push_back(2);
  cvc::world::geometry_item box;
  box.name = "box";
  box.geometry = mesh;
  w.add(box);

  const cvc::world::world_geometry c = cvc::world::crop_world_geometry(w, {25, 25, 55, 55});
  const std::shared_ptr<const cvc::height_field> t = c.terrain();
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->x0(), 20.0);
  EXPECT_EQ(t->y0(), 20.0);
  EXPECT_EQ(t->cols(), 5); // samples 2..6 cover 25..55
  EXPECT_EQ(t->rows(), 5);
  EXPECT_EQ(t->height_at(0, 0), float(2 * 100 + 2));
  EXPECT_EQ(t->height_at(4, 4), float(6 * 100 + 6));
  EXPECT_EQ(t->cell_material(1, 2), m[3 * 10 + 4]);
  const cvc::world::geometry_item *b = c.find("box");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->geometry->const_tris().size(), 1u);
  EXPECT_EQ(b->geometry->const_points().size(), 3u);
  ASSERT_EQ(b->geometry->const_face_materials().size(), 1u);
  EXPECT_EQ(b->geometry->const_face_materials()[0], 1);
  EXPECT_EQ(c.edge_walls(), w.edge_walls());

  // A region holding no mesh face drops the mesh; one off the terrain, or backwards, is refused.
  EXPECT_EQ(cvc::world::crop_world_geometry(w, {60, 60, 80, 80}).find("box"), nullptr);
  EXPECT_THROW(cvc::world::crop_world_geometry(w, {500, 500, 600, 600}), std::invalid_argument);
  EXPECT_THROW(cvc::world::crop_world_geometry(w, {50, 50, 20, 20}), std::invalid_argument);
}

namespace {

// A made world (kParams) under a data node with `extra` props.
std::unique_ptr<SceneWorld> data_node_world(cvc::app &app, const std::string &extra,
                                            std::vector<std::string> &warnings) {
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
)" + extra);
  EXPECT_TRUE(r.ok) << r.error;
  if (!r.ok)
    return nullptr;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  for (const DataSpec &d : r.data)
    store->bind("", d);
  EXPECT_TRUE(store->wait("data.world", 300.0)) << val(app, "data.world.error");
  static std::vector<LoadResult> keep; // the scene's nodes outlive the world built from them
  keep.push_back(std::move(r));
  return SceneWorld::from_scene(app, keep.back().scene, "", "physics", &warnings);
}

} // namespace

// region: on a data node cuts its dataset's world once: its bodies, the walls at the cut's edge
// and the navigator's grid all follow it, and the dataset itself is left whole.
TEST(WorldDataset, ARegionCutsTheBodiesTheWallsAndTheGrid) {
  REQUIRE_YAML();
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w =
      data_node_world(app, "      region: [-50, -40, 60, 70]\n", warnings);
  ASSERT_NE(w, nullptr);
  for (const std::string &m : warnings)
    EXPECT_EQ(m.find("region"), std::string::npos) << m;
  std::shared_ptr<const cvc::world::world_geometry> cut = w->world_geometry("city");
  ASSERT_NE(cut, nullptr);
  ASSERT_NE(cut->terrain(), nullptr);
  const std::shared_ptr<const cvc::height_field> t = cut->terrain();
  const double d = t->dx();
  EXPECT_LE(t->x0(), -50.0);
  EXPECT_GT(t->x0(), -50.0 - d);
  EXPECT_GE(t->max_x(), 60.0);
  EXPECT_LT(t->max_x(), 60.0 + d);
  EXPECT_LE(t->y0(), -40.0);
  EXPECT_GE(t->max_y(), 70.0);
  const std::shared_ptr<const cvc::world::world_geometry> whole = dataset_world(app, "data.world");
  EXPECT_GT(whole->terrain()->cols(), t->cols()); // the dataset is whole
  EXPECT_EQ(w->bodies_of("city").size(), collide_items(*cut) + kEdgeWalls);
  const std::shared_ptr<const cvc::nav::world_grid> g = w->grid("city", 2.0);
  ASSERT_NE(g, nullptr);
  EXPECT_GE(g->min_x, t->x0() - 1e-9);
  EXPECT_LE(g->max_x, t->max_x() + 1e-9);
  EXPECT_GE(g->min_y, t->y0() - 1e-9);
  EXPECT_LE(g->max_y, t->max_y() + 1e-9);
}

TEST(WorldDataset, ABadRegionWarnsAndKeepsTheWholeWorld) {
  REQUIRE_YAML();
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = data_node_world(app, "      region: [10, 10, 5]\n", warnings);
  ASSERT_NE(w, nullptr);
  bool said = false;
  for (const std::string &m : warnings)
    said = said || m.find("region: is [min_x, min_y, max_x, max_y]") != std::string::npos;
  EXPECT_TRUE(said);
  ASSERT_NE(w->world_geometry("city"), nullptr);
  EXPECT_EQ(w->world_geometry("city")->terrain(), dataset_world(app, "data.world")->terrain());
}

// water_level: on a data node is its one water knob: the world gets a still-water field up to it
// (inertial, from under the lowest ground), which a body in it reads, and the navigator's grid has
// its water at the same level.
TEST(WorldDataset, ADataNodesWaterLevelIsItsWater) {
  REQUIRE_YAML();
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = data_node_world(app, "      water_level: 2000\n", warnings);
  ASSERT_NE(w, nullptr);
  std::shared_ptr<const cvc::world::world_geometry> g = w->world_geometry("city");
  ASSERT_NE(g, nullptr);
  const cvc::world::volume_item *water = g->find_volume("water");
  ASSERT_NE(water, nullptr);
  EXPECT_EQ(water->field.kind, cvc::physics::field_kind::fluid);
  EXPECT_TRUE(water->field.inertial);
  const std::shared_ptr<const cvc::height_field> t = g->terrain();
  ASSERT_NE(t, nullptr);
  const double x = 0.5 * (t->x0() + t->max_x()), y = 0.5 * (t->y0() + t->max_y());
  EXPECT_TRUE(w->world().water_at({x, y, 1999.0}).wet);
  EXPECT_EQ(w->world().water_at({x, y, 1999.0}).density, 1000.0);
  EXPECT_FALSE(w->world().water_at({x, y, 2001.0}).wet);
  const std::shared_ptr<const cvc::nav::world_grid> grid = w->grid("city", 2.0);
  ASSERT_NE(grid, nullptr);
  EXPECT_TRUE(grid->has_water());
  EXPECT_EQ(grid->water_level, 2000.0);

  // Without one the dataset's level stands; with no ground under it there is no field.
  cvc::app dry_app;
  std::vector<std::string> dry_warnings;
  std::unique_ptr<SceneWorld> dry =
      data_node_world(dry_app, "      water_level: -5000\n", dry_warnings);
  ASSERT_NE(dry, nullptr);
  EXPECT_EQ(dry->world_geometry("city")->find_volume("water"), nullptr);
}

namespace {

// A fresh directory under the system's temp directory, removed afterwards.
struct SaveDir {
  std::filesystem::path dir;
  SaveDir() {
    std::random_device rd;
    dir = std::filesystem::temp_directory_path() /
          ("cvc-world-save-" + std::to_string(rd()) + std::to_string(rd()));
    std::filesystem::create_directories(dir);
  }
  ~SaveDir() {
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
  }
  std::string at(const std::string &rel) const { return (dir / rel).generic_string(); }
};

// The hash a dataset's own files give (city_bundle_hash over data_files).
std::string files_hash(cvc::app &app, const std::string &path) {
  cvc::world::city_bundle_files files;
  for (const std::string &f : data_files(app, path))
    files[f] = data_file(app, path, f);
  return cvc::world::city_bundle_hash(files);
}

} // namespace

// The terrain lab's Save (GB4, data_save_files): a world's dataset saved as a directory and as a
// zip, each the dataset's files, loads again as the same world (the same hash, the same terrain);
// with the exports it also holds export/, which the hash leaves out.
TEST(WorldDataset, ASavedWorldLoadsAgainAsTheSameWorld) {
  if (!glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  cvc::app app;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  DataSpec spec;
  spec.name = "world";
  spec.generate = "generated_world";
  spec.params = kParams;
  store->bind("", spec);
  ASSERT_TRUE(store->wait("data.world", 300.0)) << val(app, "data.world.error");
  const std::string hash = val(app, "data.world.hash");
  const std::vector<std::string> names = data_files(app, "data.world");
  SaveDir d;

  const DataSaved dir = data_save_files(app, "data.world", "saved", d.dir.string());
  EXPECT_EQ(std::filesystem::path(dir.where), d.dir / "saved");
  EXPECT_EQ(dir.files, names.size());
  for (const std::string &n : names)
    EXPECT_TRUE(std::filesystem::is_regular_file(d.dir / "saved" / n)) << n;
  const DataSaved zip = data_save_files(app, "data.world", d.at("saved.zip"));
  EXPECT_EQ(zip.files, names.size());
  EXPECT_EQ(zip.bytes, dir.bytes);
  std::string zbytes;
  {
    std::ifstream in(d.dir / "saved.zip", std::ios::binary);
    zbytes.assign(std::istreambuf_iterator<char>(in), {});
  }
  const std::vector<cvc::archive::file> unpacked = cvc::archive::unpack(zbytes);
  ASSERT_EQ(unpacked.size(), names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(unpacked[i].path, names[i]);
    EXPECT_EQ(*unpacked[i].bytes, *data_file(app, "data.world", names[i])) << names[i];
  }

  std::shared_ptr<const cvc::world::world_geometry> made = dataset_world(app, "data.world");
  for (const char *what : {"saved", "saved.zip"}) {
    DataSpec back;
    back.name = std::string("back_") + (what[5] ? "zip" : "dir");
    back.uri = d.at(what);
    back.as = "city_bundle";
    store->bind("", back);
    const std::string at = "data." + back.name;
    ASSERT_TRUE(store->wait(at, 120.0)) << what << ": " << val(app, at + ".error");
    EXPECT_EQ(val(app, at + ".hash"), hash) << what;
    EXPECT_EQ(files_hash(app, at), hash) << what;
    std::shared_ptr<const cvc::world::world_geometry> w = dataset_world(app, at);
    ASSERT_NE(w, nullptr) << what;
    EXPECT_EQ(w->terrain()->heights(), made->terrain()->heights()) << what;
    EXPECT_EQ(collide_items(*w), collide_items(*made)) << what;
  }

  DataSaveOptions o;
  o.exports = true;
  const DataSaved ex = data_save_files(app, "data.world", d.at("exported"), "", o);
  EXPECT_EQ(ex.files, names.size() + 3);
  for (const char *f : {"terrain.glb", "heightmap.png", "heightmap.json"})
    EXPECT_TRUE(std::filesystem::is_regular_file(d.dir / "exported" / "export" / f)) << f;
  DataSpec back;
  back.name = "back_exported";
  back.uri = d.at("exported");
  back.as = "city_bundle";
  store->bind("", back);
  ASSERT_TRUE(store->wait("data.back_exported", 120.0)) << val(app, "data.back_exported.error");
  EXPECT_EQ(val(app, "data.back_exported.hash"), hash);
  EXPECT_EQ(files_hash(app, "data.back_exported"), hash);

  EXPECT_THROW(data_save_files(app, "data.nothing", d.at("x")), std::runtime_error);
  EXPECT_THROW(data_save_files(app, "data.world", ""), std::runtime_error);
  EXPECT_THROW(data_save_files(app, "data.world", "x", "https://example.com/docs"),
               std::runtime_error);
}

// A generated_world node saves the world shown on a "save" request (world_controls' Save): to
// world.save.file, with export/ when world.save.exports is on; world.save.status says where, or
// why not.
TEST(WorldDataset, AGeneratedWorldNodeSavesOnRequest) {
  REQUIRE_YAML();
  if (!glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
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
)");
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(val(app, "world.save.file"), "world"); // its dataset's name
  EXPECT_EQ(val(app, "world.save.status"), "");
  const std::string seed = val(app, "data.world.generator.seed");
  SaveDir d;

  write<std::string>(app, "world.save.file", d.at("lab.zip"));
  write<std::string>(app, "world.request", "save");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "world.save.status").rfind("saved ", 0), 0u) << val(app, "world.save.status");
  EXPECT_TRUE(std::filesystem::is_regular_file(d.dir / "lab.zip"));
  EXPECT_EQ(val(app, "world.request"), "");
  EXPECT_EQ(val(app, "data.world.generator.seed"), seed); // a save is not a rebuild

  write<std::string>(app, "world.save.file", d.at("lab"));
  write<double>(app, "world.save.exports", 1);
  write<std::string>(app, "world.request", "save");
  w->sync(&warnings);
  EXPECT_TRUE(std::filesystem::is_regular_file(d.dir / "lab" / "world.json"));
  EXPECT_TRUE(std::filesystem::is_regular_file(d.dir / "lab" / "export" / "heightmap.png"));

  // A file in the way: the status says why.
  { std::ofstream(d.dir / "in_the_way") << "x"; }
  write<std::string>(app, "world.save.file", d.at("in_the_way/lab"));
  write<std::string>(app, "world.request", "save");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "world.save.status").rfind("error: ", 0), 0u) << val(app, "world.save.status");
}

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Version 2 city bundles (cvc/world/city_bundle.h, ARIADNE_WORLDS section 18.8): a world written
// as a bundle and read back is the same world, a generated world included, and writing what was
// read gives the same bytes, so its hash is the world's.

#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h>
#include <cvc/image/image_encode.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/surface.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <optional>
#include <sstream>
#include <string>

using namespace cvc::world;
namespace fs = std::filesystem;

namespace {

bool have_glb() { return cvc::can_write_model_to_memory(".glb"); }
bool have_jolt() { return cvc::physics::backend_available(cvc::physics::backend::jolt); }

std::uint16_t sid(const char *name) { return surface_registry::builtin().by_name(name); }

// A 6 x 5 terrain with cell surfaces, two buildings on two surfaces, and a lines item.
world_geometry small_world_geometry() {
  world_geometry w;
  w.set_edge_walls(false);
  std::vector<float> h;
  for (int r = 0; r < 5; ++r)
    for (int c = 0; c < 6; ++c)
      h.push_back(0.25f * r + 0.125f * c + (r * c % 3) * 0.5f);
  auto field = std::make_shared<cvc::height_field>(5, 6, -10.0, -8.0, 4.0, 4.0, h);
  std::vector<cvc::face_material_t> cells;
  for (int i = 0; i < 4 * 5; ++i)
    cells.push_back(i % 3 == 0 ? sid("asphalt") : sid("dirt"));
  field->set_cell_materials(cells);
  geometry_item t;
  t.name = "terrain";
  t.geometry = field;
  t.surface = sid("dirt");
  w.add(t);

  auto b = std::make_shared<cvc::geometry>();
  for (int k = 0; k < 8; ++k) {
    cvc::point_t p;
    p[0] = (k & 1) * 3.0 + 0.1;
    p[1] = ((k >> 1) & 1) * 2.0 - 1.3;
    p[2] = (k >> 2) * 5.0 + 0.7;
    b->points().push_back(p);
    cvc::color_t col;
    col[0] = 0.1 * k;
    col[1] = 0.5;
    col[2] = 1.0 - 0.1 * k;
    b->colors().push_back(col);
  }
  const int tris[4][3] = {{0, 1, 3}, {0, 3, 2}, {4, 5, 7}, {4, 7, 6}};
  for (const auto &tr : tris) {
    cvc::tri_t x;
    x[0] = tr[0];
    x[1] = tr[1];
    x[2] = tr[2];
    b->tris().push_back(x);
  }
  b->face_materials() = {sid("building_concrete"), sid("building_concrete"), sid("asphalt"),
                         sid("asphalt")};
  geometry_item bi;
  bi.name = "buildings";
  bi.geometry = b;
  bi.surface = sid("building_concrete");
  bi.position = {1.0, 2.0, 0.5};
  w.add(bi);

  auto n = std::make_shared<cvc::geometry>();
  for (int k = 0; k < 4; ++k) {
    cvc::point_t p;
    p[0] = k * 1.5;
    p[1] = 2.0;
    p[2] = 3.0 + k;
    n->points().push_back(p);
  }
  for (int k = 0; k < 3; ++k) {
    cvc::line_t l;
    l[0] = k;
    l[1] = k + 1;
    n->lines().push_back(l);
  }
  geometry_item ni;
  ni.name = "needles";
  ni.geometry = n;
  ni.roles = role_render;
  w.add(ni);
  return w;
}

generated_world_params small_params() {
  generated_world_params p;
  p.seed = 11;
  p.half_m = 200.0;
  p.trees = 40;
  p.rocks = 10;
  p.buildings = 16;
  p.tree_gen = 3;
  p.amp_m = 40.0;
  return p;
}

} // namespace

TEST(ModelGlb, WritesAndReadsBackExactly) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  cvc::model m;
  cvc::material mat;
  mat.name = "asphalt";
  mat.base_color = {{0.2, 0.2, 0.25, 1.0}};
  m.materials.push_back(mat);
  cvc::model::mesh tri;
  tri.material = 0;
  tri.name = "tri";
  for (int k = 0; k < 4; ++k) {
    cvc::point_t p;
    p[0] = k;
    p[1] = k * k * 0.5;
    p[2] = -k;
    tri.geom.points().push_back(p);
  }
  tri.geom.tris().push_back({{0, 1, 2}});
  tri.geom.tris().push_back({{0, 2, 3}});
  m.meshes.push_back(tri);
  cvc::model::mesh lines;
  lines.name = "lines";
  for (int k = 0; k < 3; ++k) {
    cvc::point_t p;
    p[0] = k;
    p[1] = 1;
    p[2] = 2;
    lines.geom.points().push_back(p);
  }
  lines.geom.lines().push_back({{0, 1}});
  lines.geom.lines().push_back({{1, 2}});
  m.meshes.push_back(lines);

  const std::string bytes = cvc::write_model_to_memory(app, m, ".glb");
  ASSERT_GT(bytes.size(), 12u);
  EXPECT_EQ(bytes.substr(0, 4), "glTF");
  EXPECT_EQ(cvc::write_model_to_memory(app, m, "glb"), bytes) << "the same model, other bytes";

  cvc::model_read_options exact;
  exact.exact = true;
  const cvc::model back =
      cvc::read_model_from_memory(app, bytes.data(), bytes.size(), ".glb", exact);
  ASSERT_EQ(back.meshes.size(), 2u);
  EXPECT_EQ(back.meshes[0].geom.const_points().size(), 4u);
  EXPECT_EQ(back.meshes[0].geom.const_tris().size(), 2u);
  EXPECT_TRUE(back.meshes[0].geom.const_normals().empty()) << "an exact read made normals";
  EXPECT_EQ(back.meshes[1].geom.const_lines().size(), 2u);
  for (std::size_t v = 0; v < 4; ++v)
    for (int k = 0; k < 3; ++k)
      EXPECT_DOUBLE_EQ(back.meshes[0].geom.const_points()[v][k],
                       static_cast<float>(tri.geom.const_points()[v][k]));
  ASSERT_GE(back.meshes[0].material, 0);
  EXPECT_EQ(back.materials[back.meshes[0].material].name, "asphalt");
}

TEST(CityBundleV2, AWorldReadsBackAsItWasWritten) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  const world_geometry w = small_world_geometry();
  city_bundle_write info;
  info.water_level = 1.25;
  info.region = city_bundle_region{-8.0, -6.0, 8.0, 6.0};
  info.generator.name = "test";
  info.generator.version = "1";
  info.generator.seed = 42;
  info.generator.params = "{\"a\": 1}";
  info.extra["notes.txt"] = std::make_shared<const std::string>("kept as written");
  const city_bundle_files files = city_bundle_bytes(app, w, info);
  for (const char *f : {"world.json", "terrain.json", "terrain.f32", "terrain_surfaces.u16",
                        "buildings.glb", "needles.glb", "notes.txt"})
    EXPECT_TRUE(files.count(f)) << f;

  const city_bundle b = load_city_bundle(app, city_bundle_memory_reader(files), "memory");
  EXPECT_EQ(b.version, 2);
  EXPECT_EQ(b.hash, city_bundle_hash(files));
  EXPECT_EQ(b.hash.size(), 64u);
  ASSERT_TRUE(b.water_level.has_value());
  EXPECT_DOUBLE_EQ(*b.water_level, 1.25);
  ASSERT_TRUE(b.region.has_value());
  EXPECT_DOUBLE_EQ(b.region->max_y, 6.0);
  EXPECT_EQ(b.generator.name, "test");
  EXPECT_EQ(b.generator.seed, 42u);
  EXPECT_FALSE(b.geometry.edge_walls());
  EXPECT_TRUE(b.buildings_loaded);

  // The terrain: the same heights, cell surfaces and grid, exactly.
  const auto t0 = w.terrain(), t1 = b.geometry.terrain();
  ASSERT_TRUE(t1);
  EXPECT_EQ(t1->rows(), t0->rows());
  EXPECT_EQ(t1->cols(), t0->cols());
  EXPECT_EQ(t1->x0(), t0->x0());
  EXPECT_EQ(t1->dy(), t0->dy());
  EXPECT_EQ(t1->heights(), t0->heights());
  EXPECT_EQ(t1->cell_materials(), t0->cell_materials());

  // The meshes: the same items, roles, surfaces, placement and faces.
  ASSERT_EQ(b.geometry.items().size(), w.items().size());
  for (const geometry_item &a : w.items()) {
    const geometry_item *c = b.geometry.find(a.name);
    ASSERT_NE(c, nullptr) << a.name;
    EXPECT_EQ(c->roles, a.roles) << a.name;
    EXPECT_EQ(c->surface, a.surface) << a.name;
    EXPECT_EQ(c->position, a.position) << a.name;
    if (a.height_field())
      continue;
    EXPECT_EQ(c->geometry->const_tris().size(), a.geometry->const_tris().size()) << a.name;
    EXPECT_EQ(c->geometry->const_lines().size(), a.geometry->const_lines().size()) << a.name;
    EXPECT_EQ(c->geometry->const_points().size(), a.geometry->const_points().size()) << a.name;
  }
  // Each face keeps its surface (the buildings' two).
  std::map<int, int> before, after;
  for (const auto m : w.find("buildings")->geometry->const_face_materials())
    ++before[m];
  for (const auto m : b.geometry.find("buildings")->geometry->const_face_materials())
    ++after[m];
  EXPECT_EQ(after, before);
  EXPECT_TRUE(b.geometry.find("needles")->geometry->const_face_materials().empty());

  // Written again from what was read: the same bytes, file by file.
  city_bundle_write again = info;
  const city_bundle_files files2 = city_bundle_bytes(app, b.geometry, again);
  ASSERT_EQ(files2.size(), files.size());
  for (const auto &[name, bytes] : files)
    EXPECT_EQ(*files2.at(name), *bytes) << name << " changed on a second write";

  // The hash follows the contents.
  city_bundle_files changed = files;
  std::string t = *changed.at("terrain.f32");
  t[0] ^= 1;
  changed["terrain.f32"] = std::make_shared<const std::string>(t);
  EXPECT_NE(city_bundle_hash(changed), city_bundle_hash(files));
}

TEST(CityBundleV2, ADirectoryReadsAsMemoryDoes) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  const city_bundle_files files = city_bundle_bytes(app, small_world_geometry());
  const auto dir = std::filesystem::temp_directory_path() / "cvc_city_bundle_v2_dir";
  std::filesystem::remove_all(dir);
  write_city_bundle(dir.string(), files);
  const city_bundle b = load_city_bundle(app, dir.string());
  EXPECT_EQ(b.version, 2);
  EXPECT_EQ(b.hash, city_bundle_hash(files));
  EXPECT_EQ(b.geometry.items().size(), 3u);
  std::filesystem::remove_all(dir);
}

TEST(CityBundleV2, BadBundlesSayWhy) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  city_bundle_files files = city_bundle_bytes(app, small_world_geometry());
  city_bundle_files missing = files;
  missing.erase("buildings.glb");
  try {
    load_city_bundle(app, city_bundle_memory_reader(missing), "memory");
    FAIL() << "a missing item file loaded";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("buildings.glb"), std::string::npos) << e.what();
  }
  city_bundle_files short_heights = files;
  short_heights["terrain.f32"] = std::make_shared<const std::string>("1234");
  EXPECT_THROW(load_city_bundle(app, city_bundle_memory_reader(short_heights), "memory"),
               std::runtime_error);
  world_geometry no_terrain;
  EXPECT_THROW(city_bundle_bytes(app, no_terrain), std::invalid_argument);
}

TEST(CityBundleV2, AGeneratedWorldIsTheSameWorldReadBack) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  const generated_world gw = generate_world(app, small_params());
  const city_bundle_files files = generated_world_bundle(app, gw, "test");
  const city_bundle b = load_city_bundle(app, city_bundle_memory_reader(files), "generated");
  EXPECT_EQ(b.generator.name, "generated_world");
  EXPECT_EQ(b.generator.seed, 11u);
  ASSERT_TRUE(b.water_level.has_value());
  EXPECT_DOUBLE_EQ(*b.water_level, gw.params.water_level_m);
  const generated_world_params back = generated_world_params_from_json(b.generator.params);
  EXPECT_EQ(back.seed, gw.params.seed);
  EXPECT_DOUBLE_EQ(back.half_m, gw.params.half_m);
  EXPECT_EQ(back.trees, gw.params.trees);
  EXPECT_EQ(back.city, gw.params.city);
  EXPECT_EQ(generated_world_params_json(back), generated_world_params_json(gw.params));

  // The terrain exactly; the meshes face for face.
  EXPECT_EQ(b.geometry.terrain()->heights(), gw.terrain->heights());
  EXPECT_EQ(b.geometry.terrain()->cell_materials(), gw.terrain->cell_materials());
  const world_geometry g = gw.geometry();
  for (const geometry_item &a : g.items()) {
    const geometry_item *c = b.geometry.find(a.name);
    ASSERT_NE(c, nullptr) << a.name;
    EXPECT_EQ(c->roles, a.roles) << a.name;
    if (!a.height_field())
      EXPECT_EQ(c->geometry->const_tris().size(), a.geometry->const_tris().size()) << a.name;
  }
  if (!gw.needles.const_points().empty())
    EXPECT_NE(b.geometry.find("needles"), nullptr);

  // What was read writes the same bytes again: the hash is the world's.
  city_bundle_write info;
  info.generator = b.generator;
  info.water_level = b.water_level;
  for (const char *extra : {"props.json", "streets.json", "ground.png"})
    if (files.count(extra))
      info.extra[extra] = files.at(extra);
  const city_bundle_files again = city_bundle_bytes(app, b.geometry, info);
  EXPECT_EQ(city_bundle_hash(again), b.hash);
  // And the same params make the same bundle.
  EXPECT_EQ(
      city_bundle_hash(generated_world_bundle(app, generate_world(app, small_params()), "test")),
      b.hash);
}

TEST(CityBundleV2, PhysicsRunsTheSameOnEveryLoad) {
  cvc::app app;
  if (!have_glb() || !have_jolt())
    GTEST_SKIP() << "needs glb and Jolt";
  const generated_world gw = generate_world(app, small_params());
  const city_bundle_files files = generated_world_bundle(app, gw, "test");
  auto run = [&](const world_geometry &g) {
    cvc::physics::world_config c;
    c.backend = cvc::physics::backend::jolt;
    cvc::physics::world w(c);
    cvc::physics::add_world_geometry(w, g);
    for (int i = 0; i < 5; ++i) {
      cvc::physics::body_desc d{cvc::physics::box{{0.5, 0.5, 0.5}}};
      d.position = {-40.0 + 20.0 * i, 10.0, gw.height(-40.0 + 20.0 * i, 10.0) + 6.0};
      w.add_body(d);
    }
    w.step(200);
    return w.state_hash();
  };
  const std::uint64_t a =
      run(load_city_bundle(app, city_bundle_memory_reader(files), "a").geometry);
  const std::uint64_t b =
      run(load_city_bundle(app, city_bundle_memory_reader(files), "b").geometry);
  EXPECT_EQ(a, b) << "two loads of one bundle simulate differently";

  // The loaded terrain is the generated one to the engine too.
  cvc::physics::world_config c;
  c.backend = cvc::physics::backend::jolt;
  cvc::physics::world w(c);
  cvc::physics::add_world_geometry(
      w, load_city_bundle(app, city_bundle_memory_reader(files), "c").geometry);
  double worst = 0;
  for (int i = -4; i <= 4; ++i)
    for (int j = -4; j <= 4; ++j) {
      const double x = 37.0 * i, y = 31.0 * j;
      const auto hit = w.raycast({x, y, 500.0}, {x, y, -500.0});
      if (hit && hit->point.z < gw.height(x, y) + 0.5) // the ground, not a roof or a tree
        worst = std::max(worst, std::fabs(hit->point.z - gw.height(x, y)));
    }
  EXPECT_LT(worst, 0.01);
}

// ── GB3: everything the renderer draws is in the bundle (world_from_bundle) ──

// A generated world's bundle holds the props' coarser rungs and their cells, the streets and the
// painted ground; read back, it is the world the renderer drew: the same meshes, cells, streets,
// counts, terrain chunks and water, rebuilt where they are derived. Written again with its ground,
// it is the same bundle.
TEST(CityBundleV2, AGeneratedWorldReadsBackAsTheWorldItDraws) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  generated_world_params p = small_params();
  p.water_level_m = 2.0; // some water, so the rebuilt water has something to match
  const generated_world gw = generate_world(app, p);
  ASSERT_FALSE(gw.prop_cells.empty());
  ASSERT_FALSE(gw.props_r1.const_tris().empty());
  const city_bundle_files files = generated_world_bundle(app, gw, "test");
  for (const char *f : {"props_r1.glb", "props_r2.glb", "props.json", "streets.json", "ground.png"})
    EXPECT_EQ(files.count(f), 1u) << f;
  const city_bundle b = load_city_bundle(app, city_bundle_memory_reader(files), "generated");
  const bundle_world bw = world_from_bundle(app, b, city_bundle_memory_reader(files));
  ASSERT_TRUE(bw.world);
  const generated_world &w = *bw.world;

  EXPECT_EQ(generated_world_params_json(w.params), generated_world_params_json(gw.params));
  EXPECT_EQ(w.terrain->heights(), gw.terrain->heights());
  EXPECT_EQ(w.terrain->cell_materials(), gw.terrain->cell_materials());
  auto same_faces = [](const cvc::geometry &a, const cvc::geometry &c, const char *what) {
    EXPECT_EQ(a.const_tris().size(), c.const_tris().size()) << what;
    EXPECT_EQ(a.const_lines().size(), c.const_lines().size()) << what;
    EXPECT_EQ(a.const_colors().empty(), c.const_colors().empty()) << what;
  };
  same_faces(w.buildings, gw.buildings, "buildings");
  same_faces(w.roads, gw.roads, "roads");
  same_faces(w.props, gw.props, "props");
  same_faces(w.foliage, gw.foliage, "foliage");
  same_faces(w.needles, gw.needles, "needles");
  same_faces(w.props_r1, gw.props_r1, "props_r1");
  same_faces(w.props_r2, gw.props_r2, "props_r2");

  // The cells: the same runs of faces and bounds; each cell's mesh is the cell's.
  EXPECT_DOUBLE_EQ(w.prop_cell_m, gw.prop_cell_m);
  ASSERT_EQ(w.prop_cells.size(), gw.prop_cells.size());
  for (std::size_t i = 0; i < w.prop_cells.size(); ++i) {
    const auto &a = w.prop_cells[i], &c = gw.prop_cells[i];
    for (int k = 0; k < 3; ++k) {
      EXPECT_DOUBLE_EQ(a.min[k], c.min[k]) << i;
      EXPECT_DOUBLE_EQ(a.max[k], c.max[k]) << i;
    }
    const std::pair<const generated_world::prop_span *, const generated_world::prop_span *>
        spans[] = {{&a.props, &c.props},
                   {&a.foliage, &c.foliage},
                   {&a.needles, &c.needles},
                   {&a.props_r1, &c.props_r1},
                   {&a.props_r2, &c.props_r2}};
    const cvc::geometry *meshes[][2] = {{&w.props, &gw.props},
                                        {&w.foliage, &gw.foliage},
                                        {&w.needles, &gw.needles},
                                        {&w.props_r1, &gw.props_r1},
                                        {&w.props_r2, &gw.props_r2}};
    for (int s = 0; s < 5; ++s) {
      EXPECT_EQ(spans[s].first->c0, spans[s].second->c0) << i << " " << s;
      EXPECT_EQ(spans[s].first->c1, spans[s].second->c1) << i << " " << s;
      const cvc::geometry got = prop_span_mesh(*meshes[s][0], *spans[s].first);
      const cvc::geometry want = prop_span_mesh(*meshes[s][1], *spans[s].second);
      EXPECT_EQ(got.const_tris().size(), want.const_tris().size()) << i << " " << s;
      EXPECT_EQ(got.const_lines().size(), want.const_lines().size()) << i << " " << s;
      EXPECT_LE(got.const_points().size(), want.const_points().size()) << i << " " << s;
      for (const auto &f : got.const_tris())
        for (int k = 0; k < 3; ++k)
          EXPECT_LT(std::size_t(f[k]), got.const_points().size()) << i << " " << s;
    }
  }

  // The streets and the city's counts.
  ASSERT_EQ(w.streets.nodes.size(), gw.streets.nodes.size());
  ASSERT_EQ(w.streets.segments.size(), gw.streets.segments.size());
  for (std::size_t i = 0; i < w.streets.segments.size(); ++i) {
    EXPECT_EQ(w.streets.segments[i].a, gw.streets.segments[i].a);
    EXPECT_EQ(w.streets.segments[i].b, gw.streets.segments[i].b);
    EXPECT_DOUBLE_EQ(w.streets.segments[i].width, gw.streets.segments[i].width);
    EXPECT_EQ(w.streets.segments[i].kind, gw.streets.segments[i].kind);
    EXPECT_EQ(w.streets.segments[i].arterial, gw.streets.segments[i].arterial);
  }
  for (std::size_t i = 0; i < w.streets.nodes.size(); ++i) {
    EXPECT_DOUBLE_EQ(w.streets.nodes[i].x, gw.streets.nodes[i].x);
    EXPECT_DOUBLE_EQ(w.streets.nodes[i].y, gw.streets.nodes[i].y);
  }
  EXPECT_EQ(w.building_count, gw.building_count);
  EXPECT_EQ(w.archetypes, gw.archetypes);
  EXPECT_EQ(w.districts, gw.districts);
  EXPECT_DOUBLE_EQ(w.road_length, gw.road_length);

  // Rebuilt from the stored terrain: the same chunks, and the same water.
  EXPECT_EQ(w.terrain_layout.chunk_rows, gw.terrain_layout.chunk_rows);
  EXPECT_EQ(w.terrain_layout.chunk_cols, gw.terrain_layout.chunk_cols);
  ASSERT_EQ(w.terrain_chunks.size(), gw.terrain_chunks.size());
  for (std::size_t i = 0; i < w.terrain_chunks.size(); ++i) {
    const auto &a = w.terrain_chunks[i], &c = gw.terrain_chunks[i];
    ASSERT_EQ(a.rungs.size(), c.rungs.size()) << i;
    EXPECT_EQ(a.error_m, c.error_m) << i;
    ASSERT_EQ(a.water.size(), c.water.size()) << i;
    for (std::size_t k = 0; k < a.rungs.size(); ++k) {
      EXPECT_EQ(a.rungs[k].const_tris().size(), c.rungs[k].const_tris().size()) << i;
      if (k < a.water.size())
        EXPECT_EQ(a.water[k].const_tris().size(), c.water[k].const_tris().size()) << i;
    }
  }
  EXPECT_GT(gw.water.wet_cells(), 0u);
  EXPECT_EQ(w.water.wet_cells(), gw.water.wet_cells());
  EXPECT_EQ(w.water_ground.size(), gw.water_ground.size());

  // The ground: the material painting at the bundle's size, texel for texel.
  ASSERT_EQ(bw.ground.width, world_ground_size);
  ASSERT_EQ(bw.ground.height, world_ground_size);
  EXPECT_TRUE(bw.ground.rgb == paint_surface(gw, surface_color::material, world_ground_size));

  // Written again, with its ground: the same bundle.
  const city_bundle_files again = generated_world_bundle(app, w, "test", &bw.ground);
  EXPECT_EQ(city_bundle_hash(again), b.hash);
  for (const auto &[name, bytes] : files)
    if (name != "world.json") {
      ASSERT_EQ(again.count(name), 1u) << name;
      EXPECT_TRUE(*again.at(name) == *bytes) << name;
    }
}

// A version 1 city (terrain.json, satellite.png) is a world to draw too: its terrain and chunks,
// no water (it has no level), and the satellite image as its ground.
TEST(CityBundleV2, AVersionOneCityIsAWorldToDraw) {
  cvc::app app;
  const fs::path dir = fs::temp_directory_path() / "cvc_gb3_v1_city";
  fs::remove_all(dir);
  fs::create_directories(dir);
  {
    std::ofstream tj(dir / "terrain.json");
    tj << R"({"bounds": {"min_x": 100, "min_y": -50, "max_x": 400, "max_y": 150},
  "rows": 5, "cols": 7, "grid": [)";
    for (int r = 0; r < 5; ++r) {
      tj << (r ? ", [" : "[");
      for (int c = 0; c < 7; ++c)
        tj << (c ? ", " : "") << (4 - r) + 0.5 * c;
      tj << "]";
    }
    tj << "]}";
  }
  {
    std::vector<unsigned char> px(16 * 8 * 3);
    for (int y = 0; y < 8; ++y)
      for (int x = 0; x < 16; ++x)
        px[std::size_t(y * 16 + x) * 3] = static_cast<unsigned char>(y * 30); // red rises south
    const std::string png = cvc::encode_png(
        cvc::image(16, 8, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px.data()));
    std::ofstream(dir / "satellite.png", std::ios::binary) << png;
  }
  city_bundle_options o;
  o.buildings = false;
  const city_bundle b = load_city_bundle(app, dir.string(), o);
  const bundle_world bw = world_from_bundle(app, b, city_bundle_dir_reader(dir.string()));
  ASSERT_TRUE(bw.world);
  const generated_world &w = *bw.world;
  EXPECT_EQ(w.terrain->rows(), 5);
  EXPECT_EQ(w.terrain->cols(), 7);
  EXPECT_DOUBLE_EQ(w.grid.min_x, 100.0);
  EXPECT_DOUBLE_EQ(w.grid.max_y, 150.0);
  EXPECT_DOUBLE_EQ(w.params.half_m, 150.0);
  EXPECT_EQ(w.params.terrain, "bundle");
  EXPECT_FALSE(w.terrain_chunks.empty());
  EXPECT_EQ(w.water.wet_cells(), 0u);
  EXPECT_LT(w.params.water_level_m, w.height_min);
  for (const terrain_chunk &ch : w.terrain_chunks)
    EXPECT_TRUE(ch.water.empty());
  EXPECT_TRUE(w.buildings.const_points().empty());
  EXPECT_TRUE(w.prop_cells.empty());
  // The satellite image, rows south first: the image's bottom row (red 210) is row 0.
  ASSERT_EQ(bw.ground.width, 16);
  ASSERT_EQ(bw.ground.height, 8);
  EXPECT_EQ(bw.ground.rgb[0], 210);
  EXPECT_EQ(bw.ground.rgb[std::size_t(7 * 16) * 3], 0);
  fs::remove_all(dir);
}

// A props.json or streets.json that does not fit the meshes says so.
TEST(CityBundleV2, ABadRendererFileSaysWhy) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  const generated_world gw = generate_world(app, small_params());
  city_bundle_files files = generated_world_bundle(app, gw, "test");
  const city_bundle b = load_city_bundle(app, city_bundle_memory_reader(files), "generated");
  auto why = [&](const city_bundle_files &f) {
    try {
      world_from_bundle(app, b, city_bundle_memory_reader(f));
    } catch (const std::runtime_error &e) {
      return std::string(e.what());
    }
    return std::string();
  };
  city_bundle_files bad = files;
  bad["props.json"] = std::make_shared<const std::string>(
      R"({"cell_m": 50, "cells": [[0, 0, 0, 1, 1, 1, 0, 99999999, 0, 0, 0, 0, 0, 0, 0, 0]]})");
  EXPECT_NE(why(bad).find("outside its mesh"), std::string::npos) << why(bad);
  bad = files;
  bad["props.json"] = std::make_shared<const std::string>(R"({"cells": [[1, 2, 3]]})");
  EXPECT_NE(why(bad).find("is not 16 numbers"), std::string::npos) << why(bad);
  bad = files;
  bad["streets.json"] = std::make_shared<const std::string>(
      R"({"nodes": [[0, 0, 1]], "segments": [[0, 5, 6, 0, false]]})");
  EXPECT_NE(why(bad).find("not in nodes"), std::string::npos) << why(bad);
  bad = files;
  bad["ground.png"] = std::make_shared<const std::string>("not a png");
  EXPECT_NE(why(bad).find("ground.png"), std::string::npos) << why(bad);
}

// A saved world's files for other software (GB4): the terrain as a glb with a mesh per surface its
// cells use, a 16-bit height map at full size with the north row at the top, and the JSON that
// reads it; under export/, which the hash leaves out and a directory keeps as a subdirectory.
TEST(CityBundleV2, AWorldsExportsAreItsTerrainForOtherSoftware) {
  cvc::app app;
  if (!have_glb())
    GTEST_SKIP() << "no glb writer (Assimp off)";
  const world_geometry w = small_world_geometry();
  const city_bundle_files ex = city_bundle_exports(app, w, 1.5);
  ASSERT_EQ(ex.size(), 3u);
  ASSERT_TRUE(ex.count("export/terrain.glb") && ex.count("export/heightmap.png") &&
              ex.count("export/heightmap.json"));
  const cvc::height_field &t = *w.terrain();
  float lo = t.heights().front(), hi = lo;
  for (float v : t.heights()) {
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }

  boost::property_tree::ptree j;
  {
    std::istringstream in(*ex.at("export/heightmap.json"));
    boost::property_tree::read_json(in, j);
  }
  EXPECT_EQ(j.get<int>("rows"), 5);
  EXPECT_EQ(j.get<int>("cols"), 6);
  EXPECT_DOUBLE_EQ(j.get<double>("bounds.min_x"), -10.0);
  EXPECT_DOUBLE_EQ(j.get<double>("bounds.max_y"), 8.0);
  EXPECT_DOUBLE_EQ(j.get<double>("min_height_m"), lo);
  EXPECT_DOUBLE_EQ(j.get<double>("max_height_m"), hi);
  EXPECT_DOUBLE_EQ(j.get<double>("water_level_m"), 1.5);
  EXPECT_EQ(j.get<int>("mesh.rows"), 5);
  EXPECT_EQ(j.get<int>("mesh.cols"), 6);

  // The height map: 6 x 5, row 0 the north row; an 8-bit read gives each sample's high byte.
  const std::string &png = *ex.at("export/heightmap.png");
  EXPECT_EQ(static_cast<unsigned char>(png[24]), 16u);
  const cvc::image img = cvc::decode_image(png.data(), png.size());
  ASSERT_EQ(img.width(), 6);
  ASSERT_EQ(img.height(), 5);
  const auto *px = static_cast<const unsigned char *>(img.data());
  for (int r = 0; r < 5; ++r)
    for (int c = 0; c < 6; ++c) {
      const double u = (t.height_at(r, c) - lo) / (hi - lo);
      const long v = std::lround(u * 65535.0);
      EXPECT_EQ(px[((4 - r) * 6 + c) * 4], v >> 8) << r << "," << c;
    }

  // The mesh: two triangles a cell, a mesh for each of the two surfaces, its material named by it.
  const std::string &glb = *ex.at("export/terrain.glb");
  const cvc::model m = cvc::read_model_from_memory(app, glb.data(), glb.size(), ".glb");
  std::size_t tris = 0;
  std::map<std::string, std::size_t> materials;
  for (const auto &mm : m.meshes) {
    tris += mm.geom.const_tris().size();
    ASSERT_GE(mm.material, 0);
    ++materials[m.materials[mm.material].name];
  }
  EXPECT_EQ(tris, 4u * 5u * 2u);
  EXPECT_EQ(m.meshes.size(), 2u);
  EXPECT_EQ(materials.count("asphalt"), 1u);
  EXPECT_EQ(materials.count("dirt"), 1u);

  // A mesh cap resamples the mesh over the same extent; the height map stays full size.
  const city_bundle_files capped = city_bundle_exports(app, w, std::nullopt, 3);
  {
    std::istringstream in(*capped.at("export/heightmap.json"));
    boost::property_tree::ptree s;
    boost::property_tree::read_json(in, s);
    EXPECT_EQ(s.get<int>("rows"), 5);
    EXPECT_EQ(s.get<int>("mesh.rows"), 3);
    EXPECT_EQ(s.get<int>("mesh.cols"), 3);
    EXPECT_FALSE(s.get_optional<double>("water_level_m"));
  }
  const std::string &sg = *capped.at("export/terrain.glb");
  const cvc::model sm = cvc::read_model_from_memory(app, sg.data(), sg.size(), ".glb");
  std::size_t stris = 0;
  for (const auto &mm : sm.meshes)
    stris += mm.geom.const_tris().size();
  EXPECT_EQ(stris, 2u * 2u * 2u);

  // Beside the bundle: the hash is the bundle's, and a directory reads as the same world.
  city_bundle_files files = city_bundle_bytes(app, w);
  const std::string hash = city_bundle_hash(files);
  for (const auto &[name, bytes] : ex)
    files[name] = bytes;
  EXPECT_EQ(city_bundle_hash(files), hash);
  const auto dir = fs::temp_directory_path() / "cvc_city_bundle_v2_exports";
  fs::remove_all(dir);
  write_city_bundle(dir.string(), files);
  EXPECT_TRUE(fs::is_regular_file(dir / "export" / "heightmap.png"));
  const city_bundle b = load_city_bundle(app, dir.string());
  EXPECT_EQ(b.hash, hash);
  EXPECT_EQ(b.geometry.items().size(), 3u);
  fs::remove_all(dir);
  EXPECT_THROW(write_city_bundle(dir.string(), {{"../escape", nullptr}}), std::invalid_argument);
  fs::remove_all(dir);
}

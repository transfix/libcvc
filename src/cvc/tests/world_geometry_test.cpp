/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// One world representation, G3 (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md): the
// surface table's contact physics, world_geometry, the city bundle loader, and
// the physics adapter. The renderer's adapter is tested in cvcGL
// (cvcgl_world_geometry_nodes).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/exception.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace cvc::world;

namespace {

// --- a synthetic city bundle -------------------------------------------------

// The terrain of the synthetic city: a tilted plane plus a bump, different along
// x and y so a mirrored or transposed grid is caught.
double ground(double x, double y) { return 0.05 * x - 0.02 * y + 0.3 * std::sin(0.2 * x); }

constexpr int kRows = 21, kCols = 31;
constexpr double kMinX = -30, kMaxX = 30, kMinY = -20, kMaxY = 20;

void write_terrain_json(const fs::path &dir, bool north_first) {
  std::ofstream o(dir / "terrain.json");
  o << "{\"bounds\": {\"max_x\": " << kMaxX << ", \"max_y\": " << kMaxY << ", \"min_x\": " << kMinX
    << ", \"min_y\": " << kMinY << "}, \"cols\": " << kCols << ", \"grid\": [";
  for (int i = 0; i < kRows; ++i) {
    const int r = north_first ? kRows - 1 - i : i; // file row i is world row r
    const double y = kMinY + r * (kMaxY - kMinY) / (kRows - 1);
    o << (i ? ", [" : "[");
    for (int c = 0; c < kCols; ++c) {
      const double x = kMinX + c * (kMaxX - kMinX) / (kCols - 1);
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.9g", ground(x, y));
      o << (c ? ", " : "") << buf;
    }
    o << "]";
  }
  o << "], \"rows\": " << kRows << "}";
}

struct glb_mesh {
  std::string name;
  std::vector<float> xyz;
  std::vector<std::uint32_t> indices;
};

// A box building standing on the terrain at (cx, cy): its base at the ground
// height under its centre, as the bundle generator writes them.
glb_mesh building(const std::string &name, double cx, double cy, double half, double height) {
  const double z0 = ground(cx, cy), z1 = z0 + height;
  glb_mesh m;
  m.name = name;
  for (int k = 0; k < 8; ++k) {
    m.xyz.push_back(static_cast<float>(cx + ((k & 1) ? half : -half)));
    m.xyz.push_back(static_cast<float>(cy + ((k & 2) ? half : -half)));
    m.xyz.push_back(static_cast<float>((k & 4) ? z1 : z0));
  }
  // Six faces, two triangles each, outward.
  m.indices = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
               2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  return m;
}

// A minimal binary glTF: one buffer, per mesh a POSITION and an index view, one
// node per mesh. No transforms, z up as the bundles have it.
void write_glb(const fs::path &path, const std::vector<glb_mesh> &meshes) {
  std::string bin;
  std::ostringstream views, accessors, gmeshes, nodes;
  int view = 0;
  for (std::size_t i = 0; i < meshes.size(); ++i) {
    const glb_mesh &m = meshes[i];
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (std::size_t k = 0; k < m.xyz.size(); ++k) {
      lo[k % 3] = std::min(lo[k % 3], m.xyz[k]);
      hi[k % 3] = std::max(hi[k % 3], m.xyz[k]);
    }
    const std::size_t pos_off = bin.size();
    bin.append(reinterpret_cast<const char *>(m.xyz.data()), m.xyz.size() * sizeof(float));
    const std::size_t idx_off = bin.size();
    bin.append(reinterpret_cast<const char *>(m.indices.data()),
               m.indices.size() * sizeof(std::uint32_t));
    const char *sep = i ? "," : "";
    views << sep << "{\"buffer\":0,\"byteOffset\":" << pos_off
          << ",\"byteLength\":" << m.xyz.size() * 4 << ",\"target\":34962},"
          << "{\"buffer\":0,\"byteOffset\":" << idx_off
          << ",\"byteLength\":" << m.indices.size() * 4 << ",\"target\":34963}";
    accessors << sep << "{\"bufferView\":" << view
              << ",\"componentType\":5126,\"count\":" << m.xyz.size() / 3
              << ",\"type\":\"VEC3\",\"min\":[" << lo[0] << "," << lo[1] << "," << lo[2]
              << "],\"max\":[" << hi[0] << "," << hi[1] << "," << hi[2] << "]},"
              << "{\"bufferView\":" << view + 1
              << ",\"componentType\":5125,\"count\":" << m.indices.size()
              << ",\"type\":\"SCALAR\"}";
    gmeshes << sep << "{\"name\":\"" << m.name
            << "\",\"primitives\":[{\"attributes\":{\"POSITION\":" << view
            << "},\"indices\":" << view + 1 << "}]}";
    nodes << sep << "{\"mesh\":" << i << "}";
    view += 2;
  }
  while (bin.size() % 4)
    bin.push_back('\0');
  std::ostringstream scene_nodes;
  for (std::size_t i = 0; i < meshes.size(); ++i)
    scene_nodes << (i ? "," : "") << i;
  std::string json = "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":" +
                     std::to_string(bin.size()) + "}],\"bufferViews\":[" + views.str() +
                     "],\"accessors\":[" + accessors.str() + "],\"meshes\":[" + gmeshes.str() +
                     "],\"nodes\":[" + nodes.str() + "],\"scenes\":[{\"nodes\":[" +
                     scene_nodes.str() + "]}],\"scene\":0}";
  while (json.size() % 4)
    json.push_back(' ');
  auto u32 = [](std::string &s, std::uint32_t v) {
    for (int k = 0; k < 4; ++k)
      s.push_back(static_cast<char>((v >> (8 * k)) & 0xff));
  };
  std::string out;
  u32(out, 0x46546C67u); // "glTF"
  u32(out, 2);
  u32(out, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
  u32(out, static_cast<std::uint32_t>(json.size()));
  u32(out, 0x4E4F534Au); // "JSON"
  out += json;
  u32(out, static_cast<std::uint32_t>(bin.size()));
  u32(out, 0x004E4942u); // "BIN\0"
  out += bin;
  std::ofstream(path, std::ios::binary).write(out.data(), static_cast<std::streamsize>(out.size()));
}

const std::vector<std::pair<double, double>> kSites = {{-12, 6}, {4, -9}, {18, 11}};

fs::path make_bundle(const std::string &tag, bool north_first = true, bool with_buildings = true) {
  const fs::path dir = fs::temp_directory_path() / ("cvc_city_bundle_test_" + tag);
  fs::remove_all(dir);
  fs::create_directories(dir);
  write_terrain_json(dir, north_first);
  if (with_buildings) {
    std::vector<glb_mesh> meshes;
    for (std::size_t i = 0; i < kSites.size(); ++i)
      meshes.push_back(
          building("bldg_" + std::to_string(i), kSites[i].first, kSites[i].second, 2.0, 8.0));
    write_glb(dir / "buildings.glb", meshes);
  }
  return dir;
}

bool have_jolt() { return cvc::physics::backend_available(cvc::physics::backend::jolt); }

} // namespace

// --- the surface table ------------------------------------------------------

TEST(WorldSurface, EveryClassHasContactPhysics) {
  const surface_registry &reg = surface_registry::builtin();
  for (std::size_t i = 0; i < reg.size(); ++i) {
    const surface_class &c = reg[static_cast<std::uint16_t>(i)];
    EXPECT_GE(c.mu, 0.0f) << c.name;
    EXPECT_LE(c.mu, 1.2f) << c.name;
    EXPECT_GE(c.rolling_resistance, 0.0f) << c.name;
    EXPECT_LE(c.rolling_resistance, 0.5f) << c.name;
    EXPECT_GE(c.restitution, 0.0f) << c.name;
    EXPECT_LE(c.restitution, 1.0f) << c.name;
  }
  EXPECT_FLOAT_EQ(reg[reg.by_name("asphalt")].mu, 0.9f);
  EXPECT_LT(reg[reg.by_name("snow")].mu, reg[reg.by_name("asphalt")].mu);
  EXPECT_GT(reg[reg.by_name("sand")].rolling_resistance,
            reg[reg.by_name("asphalt")].rolling_resistance);
  EXPECT_NE(reg.to_json().find("\"rolling_resistance\""), std::string::npos);
}

// --- world_geometry ---------------------------------------------------------

TEST(WorldGeometry, ItemsShareTheirGeometry) {
  world_geometry w;
  auto field = std::make_shared<cvc::height_field>(3, 3, 0, 0, 1, 1);
  w.add({"terrain", field});
  auto mesh = std::make_shared<cvc::geometry>();
  mesh->points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  mesh->tris().push_back({{0, 1, 2}});
  geometry_item roof{"roof", mesh, w.surfaces().by_name("building_metal")};
  roof.position = {{10.0, 0.0, 5.0}};
  w.add(roof);
  EXPECT_EQ(w.terrain(), field) << "the same object, not a copy";
  ASSERT_NE(w.find("roof"), nullptr);
  EXPECT_EQ(w.find("roof")->geometry, mesh);
  EXPECT_EQ(w.find("nothing"), nullptr);
  EXPECT_FALSE(w.find("roof")->height_field());
  const world_bounds b = w.bounds();
  EXPECT_DOUBLE_EQ(b.min[0], 0.0);
  EXPECT_DOUBLE_EQ(b.max[0], 11.0); // the placed roof
  EXPECT_DOUBLE_EQ(b.max[2], 5.0);
  EXPECT_EQ(w.surfaces_used(),
            (std::vector<std::uint16_t>{0, w.surfaces().by_name("building_metal")}));

  EXPECT_THROW(w.add({"roof", mesh}), std::invalid_argument);     // the name is taken
  EXPECT_THROW(w.add({"", mesh}), std::invalid_argument);         // no name
  EXPECT_THROW(w.add({"empty", nullptr}), std::invalid_argument); // no geometry
  EXPECT_THROW(w.add({"bad", mesh, 999}), std::invalid_argument); // no such surface
  geometry_item tilted{"tilted", mesh};
  tilted.rotation = {{2.0, 0.0, 0.0, 0.0}};
  EXPECT_THROW(w.add(tilted), std::invalid_argument); // not a unit rotation
}

// --- the city bundle loader --------------------------------------------------

TEST(CityBundle, TerrainRunsNorthFirstAndIsFlippedOnce) {
  const fs::path dir = make_bundle("north");
  const city_bundle city = load_city_bundle(dir.string());
  const auto field = city.geometry.terrain();
  ASSERT_TRUE(field);
  EXPECT_EQ(field->rows(), kRows);
  EXPECT_EQ(field->cols(), kCols);
  EXPECT_DOUBLE_EQ(field->x0(), kMinX);
  EXPECT_DOUBLE_EQ(field->max_y(), kMaxY);
  for (int r = 0; r < kRows; ++r)
    for (int c = 0; c < kCols; ++c) {
      const double x = field->x0() + c * field->dx(), y = field->y0() + r * field->dy();
      ASSERT_NEAR(field->height_at(r, c), ground(x, y), 1e-6) << r << ", " << c;
    }
  // A bundle written south first, loaded as one, is the same terrain.
  const fs::path south = make_bundle("south", false);
  city_bundle_options o;
  o.terrain_rows = city_bundle_options::row_order::south_first;
  const auto s = load_city_bundle(south.string(), o).geometry.terrain();
  EXPECT_EQ(s->heights(), field->heights());
  // Loaded the wrong way round, it is a different terrain: the option matters.
  EXPECT_NE(load_city_bundle(south.string()).geometry.terrain()->heights(), field->heights());
}

TEST(CityBundle, BuildingsStandOnTheTerrainWithTheirSurface) {
  const fs::path dir = make_bundle("buildings");
  const city_bundle city = load_city_bundle(dir.string());
  ASSERT_TRUE(city.buildings_loaded) << city.buildings_error;
  const geometry_item *b = city.geometry.find("buildings");
  ASSERT_NE(b, nullptr);
  const std::uint16_t concrete = city.geometry.surfaces().by_name("building_concrete");
  EXPECT_EQ(b->surface, concrete);
  ASSERT_TRUE(b->geometry->has_face_materials());
  EXPECT_EQ(b->geometry->num_faces(), 12u * kSites.size());
  for (std::size_t f = 0; f < b->geometry->num_faces(); ++f)
    EXPECT_EQ(b->geometry->face_material(f), concrete);
  // No lift: every building's lowest vertex is the terrain under its centre (the
  // bundle convention), so a demo that adds the terrain height again is wrong.
  const auto field = city.geometry.terrain();
  for (const auto &site : kSites) {
    double base = 1e30;
    for (const auto &p : b->geometry->const_points())
      if (std::fabs(p[0] - site.first) < 3 && std::fabs(p[1] - site.second) < 3)
        base = std::min(base, p[2]);
    EXPECT_NEAR(base, field->height(site.first, site.second), 0.15) << site.first;
  }
}

TEST(CityBundle, MissingPiecesAndBadFiles) {
  const fs::path bare = make_bundle("bare", true, false);
  const city_bundle city = load_city_bundle(bare.string());
  EXPECT_FALSE(city.buildings_loaded);
  EXPECT_NE(city.buildings_error.find("buildings.glb"), std::string::npos);
  EXPECT_EQ(city.geometry.items().size(), 1u); // the terrain alone is a world
  EXPECT_FALSE(city.land_cover_loaded);
  // Without land cover every cell is the terrain surface.
  EXPECT_TRUE(city.geometry.terrain()->cell_materials().empty());
  EXPECT_EQ(city.geometry.find("terrain")->surface, city.geometry.surfaces().by_name("dirt"));

  EXPECT_THROW(load_city_bundle((fs::temp_directory_path() / "cvc_no_such_bundle").string()),
               std::runtime_error);
  const fs::path bad = fs::temp_directory_path() / "cvc_city_bundle_test_bad";
  fs::create_directories(bad);
  std::ofstream(bad / "terrain.json") << "{\"bounds\": {\"min_x\": 0}, \"rows\": 2}";
  EXPECT_THROW(load_city_bundle(bad.string()), std::runtime_error);
  city_bundle_options o;
  o.building_surface = "no_such_surface";
  EXPECT_THROW(load_city_bundle(bare.string(), o), std::invalid_argument);
}

TEST(CityBundle, LandCoverGivesEachCellItsMajoritySurface) {
  const fs::path dir = make_bundle("cover");
  const surface_registry &reg = surface_registry::builtin();
  const std::uint16_t water = reg.by_name("water_shallow"), grass = reg.by_name("grass");
  city_bundle_options o;
  std::string seen_dir;
  o.land_cover = [&](const std::string &d, int rows, int cols) {
    seen_dir = d;
    // Water east of x = 0 (world row order: row 0 at the minimum y).
    std::vector<std::uint16_t> s(static_cast<std::size_t>(rows) * cols, grass);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c)
        if (kMinX + c * (kMaxX - kMinX) / (cols - 1) > 0.0)
          s[static_cast<std::size_t>(r) * cols + c] = water;
    return s;
  };
  const city_bundle city = load_city_bundle(dir.string(), o);
  EXPECT_EQ(seen_dir, dir.string());
  EXPECT_TRUE(city.land_cover_loaded);
  const auto field = city.geometry.terrain();
  EXPECT_EQ(field->material(10.0, 5.0), water);
  EXPECT_EQ(field->material(-10.0, -5.0), grass);
  // Samples are 2 m apart and the one at x = 0 is grass, so the cell over x in
  // [0, 2] has two grass and two water corners: a tie, its south-west corner's
  // grass. The cells either side are all one surface.
  EXPECT_EQ(field->material(-1.0, 0.5), grass);
  EXPECT_EQ(field->material(1.0, 0.5), grass);
  EXPECT_EQ(field->material(3.0, 0.5), water);

  o.land_cover = [](const std::string &, int, int) { return std::vector<std::uint16_t>(3, 0); };
  EXPECT_THROW(load_city_bundle(dir.string(), o), std::invalid_argument);
  EXPECT_EQ(cell_surfaces({1, 1, 2, 3}, 2, 2), (std::vector<cvc::face_material_t>{1}));
  EXPECT_EQ(cell_surfaces({1, 2, 3, 4}, 2, 2), (std::vector<cvc::face_material_t>{1}));
}

// The measured convention, on a real bundle when one is at hand:
// CVC_CITY_BUNDLE=<.../share/cvc-scenes/austin_south>. Every building's base sits
// on the terrain the loader builds (RMS <= 0.5 m; measured 0.46 m).
TEST(CityBundle, RealBundleBuildingsSitOnTheTerrain) {
  const char *dir = std::getenv("CVC_CITY_BUNDLE");
  if (!dir || !*dir)
    GTEST_SKIP() << "set CVC_CITY_BUNDLE to a city bundle directory to run this";
  city_bundle_options o;
  o.buildings = false; // the per-building check below reads the meshes itself
  const city_bundle city = load_city_bundle(dir, o);
  const auto field = city.geometry.terrain();
  const cvc::model m = cvc::read_model((fs::path(dir) / "buildings.glb").string());
  // A building is its walls and roof meshes ("bldg_NNNNNN_walls", "_roof").
  std::map<std::string, std::array<double, 4>> b; // min z, sum x, sum y, n
  for (const auto &mesh : m.meshes) {
    const std::string key = mesh.name.substr(0, mesh.name.rfind('_'));
    auto it = b.emplace(key, std::array<double, 4>{{1e30, 0, 0, 0}}).first;
    for (const auto &p : mesh.geom.const_points()) {
      it->second[0] = std::min(it->second[0], p[2]);
      it->second[1] += p[0];
      it->second[2] += p[1];
      it->second[3] += 1;
    }
  }
  double sq = 0;
  int n = 0;
  for (const auto &kv : b) {
    const double h = field->height(kv.second[1] / kv.second[3], kv.second[2] / kv.second[3]);
    if (std::isnan(h))
      continue;
    sq += (kv.second[0] - h) * (kv.second[0] - h);
    ++n;
  }
  ASSERT_GT(n, 100);
  const double rms = std::sqrt(sq / n);
  std::printf("[city-bundle] %d buildings, base minus terrain RMS %.3f m\n", n, rms);
  EXPECT_LE(rms, 0.5);
}

// --- the physics adapter -------------------------------------------------------

// Render, physics and nav read one terrain: the mesh's vertex, the engine's
// surface and height() agree at every grid sample.
TEST(PhysicsWorldGeometry, TheEngineCollidesWithTheBundlesGeometry) {
  if (!have_jolt())
    GTEST_SKIP() << "this libcvc was built without Jolt";
  const city_bundle city = load_city_bundle(make_bundle("physics").string());
  cvc::physics::world_config c;
  c.backend = cvc::physics::backend::jolt;
  cvc::physics::world w(c);
  const auto bodies = cvc::physics::add_world_geometry(w, city.geometry);
  ASSERT_EQ(bodies.bodies.size(), 2u);
  const auto terrain_body = bodies.find("terrain"), buildings_body = bodies.find("buildings");
  ASSERT_TRUE(terrain_body.valid());
  ASSERT_TRUE(buildings_body.valid());

  const auto field = city.geometry.terrain();
  const cvc::geometry &mesh = *field; // the renderer's view of the same object
  double worst = 0;
  int checked = 0;
  for (int r = 0; r < field->rows(); ++r)
    for (int cc = 0; cc < field->cols(); ++cc) {
      const double x = field->x0() + cc * field->dx(), y = field->y0() + r * field->dy();
      bool under_building = false;
      for (const auto &s : kSites)
        under_building |= std::fabs(x - s.first) < 2.5 && std::fabs(y - s.second) < 2.5;
      if (under_building)
        continue;
      const double render = mesh.const_points()[field->vertex_of(r, cc)][2];
      const double nav = field->height(x, y);
      const auto hit = w.raycast({x, y, 50.0}, {x, y, -50.0});
      ASSERT_TRUE(hit.has_value()) << x << ", " << y;
      EXPECT_EQ(hit->body, terrain_body);
      EXPECT_DOUBLE_EQ(render, nav);
      worst = std::max(worst, std::fabs(hit->point.z - nav));
      ++checked;
    }
  EXPECT_GT(checked, 500);
  EXPECT_LT(worst, 0.006) << "the engine's terrain is not the bundle's";

  // A building's roof is the buildings body, with the shell's material.
  const auto roof = w.raycast({kSites[0].first, kSites[0].second, 50.0},
                              {kSites[0].first, kSites[0].second, -50.0});
  ASSERT_TRUE(roof.has_value());
  EXPECT_EQ(roof->body, buildings_body);
  EXPECT_NEAR(roof->point.z, ground(kSites[0].first, kSites[0].second) + 8.0, 0.01);
  const cvc::physics::material shell = w.get_material(roof->material);
  EXPECT_EQ(shell.name, "building_concrete");
  EXPECT_FLOAT_EQ(
      shell.friction,
      city.geometry.surfaces()[city.geometry.surfaces().by_name("building_concrete")].mu);

  // A second world_geometry with the same surfaces reuses the materials.
  const std::size_t materials = w.material_count();
  cvc::physics::add_world_geometry(w, load_city_bundle(make_bundle("again").string()).geometry);
  EXPECT_EQ(w.material_count(), materials);
}

TEST(PhysicsWorldGeometry, OnlyCollideItemsBecomeBodies) {
  world_geometry g;
  auto field = std::make_shared<cvc::height_field>(3, 3, 0, 0, 1, 1);
  g.add({"terrain", field});
  auto mesh = std::make_shared<cvc::geometry>();
  mesh->points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  mesh->tris().push_back({{0, 1, 2}});
  geometry_item decoration{"decoration", mesh};
  decoration.roles = role_render;
  g.add(decoration);
  cvc::physics::world_config c;
  c.backend = cvc::physics::backend::null;
  cvc::physics::world w(c);
  const auto bodies = cvc::physics::add_world_geometry(w, g);
  EXPECT_EQ(bodies.bodies.size(), 1u);
  EXPECT_FALSE(bodies.find("decoration").valid());
  EXPECT_EQ(w.body_count(), 1u);
  EXPECT_EQ(w.get_material(bodies.materials[0]).name, "void_unknown");
}

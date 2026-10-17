/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// One world representation, G4 (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md): the
// navigator's planes from world_geometry, and land cover as world surfaces.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/land_cover.h>
#include <cvc/nav/material_raster.h>
#include <cvc/nav/world_planes.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;
using namespace cvc::world;

namespace {

// The demos' occupancy_from_model, verbatim (src/cvcGL/examples/nav_common.cpp
// before G4), as the reference mesh_footprint must reproduce bit for bit.
std::vector<std::uint8_t> reference_occupancy(const cvc::geometry &mesh, double min_x, double min_y,
                                              double max_x, double max_y, int nx, int ny) {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(nx) * ny, 0);
  const auto &pts = mesh.points();
  const auto &tris = mesh.tris();
  const double sx = (nx - 1) / (max_x - min_x), sy = (ny - 1) / (max_y - min_y);
  for (const auto &tri : tris) {
    double px[3], py[3];
    for (int k = 0; k < 3; ++k) {
      const auto &p = pts[tri[k]];
      px[k] = (p[0] - min_x) * sx;
      py[k] = (p[1] - min_y) * sy;
    }
    int c0 = static_cast<int>(std::floor(std::min({px[0], px[1], px[2]})));
    int c1 = static_cast<int>(std::ceil(std::max({px[0], px[1], px[2]})));
    int r0 = static_cast<int>(std::floor(std::min({py[0], py[1], py[2]})));
    int r1 = static_cast<int>(std::ceil(std::max({py[0], py[1], py[2]})));
    c0 = std::max(0, c0);
    r0 = std::max(0, r0);
    c1 = std::min(nx - 1, c1);
    r1 = std::min(ny - 1, r1);
    const double d = (py[1] - py[2]) * (px[0] - px[2]) + (px[2] - px[1]) * (py[0] - py[2]);
    if (std::fabs(d) < 1e-12)
      continue;
    const double inv = 1.0 / d;
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c) {
        const double x = c, y = r;
        const double a = ((py[1] - py[2]) * (x - px[2]) + (px[2] - px[1]) * (y - py[2])) * inv;
        const double be = ((py[2] - py[0]) * (x - px[2]) + (px[0] - px[2]) * (y - py[2])) * inv;
        if (a >= 0.0 && be >= 0.0 && 1.0 - a - be >= 0.0)
          occ[static_cast<std::size_t>(r) * nx + c] = 1;
      }
  }
  return occ;
}

// A box from (x0, y0) to (x1, y1), base z0, height h: the shape of a building.
void add_box(cvc::geometry &g, double x0, double y0, double x1, double y1, double z0, double h) {
  const auto base = static_cast<cvc::index_t>(g.num_points());
  for (int k = 0; k < 8; ++k)
    g.points().push_back({{(k & 1) ? x1 : x0, (k & 2) ? y1 : y0, (k & 4) ? z0 + h : z0}});
  const int idx[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                       2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  for (int t = 0; t < 12; ++t)
    g.tris().push_back({{base + idx[3 * t], base + idx[3 * t + 1], base + idx[3 * t + 2]}});
}

} // namespace

TEST(NavWorldPlanes, FootprintIsTheOldOccupancyBitForBit) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-120.0, 120.0);
  for (int trial = 0; trial < 20; ++trial) {
    cvc::geometry g;
    for (int b = 0; b < 25; ++b) {
      const double x = u(rng), y = u(rng);
      add_box(g, x, y, x + 3 + std::fabs(u(rng)) / 20, y + 2 + std::fabs(u(rng)) / 30, 0, 10);
    }
    for (int k = 0; k < 40; ++k) // and a loose triangle soup, some of it off the grid
      g.points().push_back({{u(rng) * 1.2, u(rng) * 1.2, 0.0}});
    for (int k = 0; k + 2 < 40; k += 3) {
      const auto n = static_cast<cvc::index_t>(g.num_points() - 40);
      g.tris().push_back({{n + k, n + k + 1, n + k + 2}});
    }
    const int nx = 64 + trial, ny = 48 + 2 * trial;
    EXPECT_EQ(cvc::nav::mesh_footprint(g, ny, nx, -100, -90, 100, 110),
              reference_occupancy(g, -100, -90, 100, 110, nx, ny))
        << "trial " << trial;
  }
}

TEST(NavWorldPlanes, PlanesComeFromTheWorldsSurfaces) {
  world_geometry w;
  const surface_registry &s = w.surfaces();
  const std::uint16_t asphalt = s.by_name("asphalt"), deep = s.by_name("water_deep");
  // 11 x 11 samples over [0, 10]^2; the east column of cells is deep water.
  std::vector<cvc::face_material_t> cells(100, asphalt);
  for (int r = 0; r < 10; ++r)
    cells[r * 10 + 9] = deep;
  std::vector<float> h(121);
  for (int r = 0; r < 11; ++r)
    for (int c = 0; c < 11; ++c)
      h[r * 11 + c] = 0.1f * c + 0.05f * r;
  auto field = std::make_shared<cvc::height_field>(11, 11, 0.0, 0.0, 1.0, 1.0, h, cells);
  w.add({"terrain", field});
  // A building over [2, 4.5]^2, a lower shed over [3.5, 5.5]^2 under its eave, and a
  // placed kiosk: a 1 m box at the origin moved to (7, 7). Every sample checked below
  // is inside or outside these, never on an edge.
  auto city = std::make_shared<cvc::geometry>();
  add_box(*city, 2.0, 2.0, 4.5, 4.5, 0.0, 12.0);
  add_box(*city, 3.5, 3.5, 5.5, 5.5, 0.0, 3.0);
  city->face_materials().assign(12, s.by_name("building_brick"));
  city->face_materials().resize(24, s.by_name("building_metal"));
  w.add({"buildings", city, s.by_name("building_concrete")});
  auto kiosk = std::make_shared<cvc::geometry>();
  add_box(*kiosk, -0.5, -0.5, 0.5, 0.5, 0.0, 2.0);
  geometry_item placed{"kiosk", kiosk, s.by_name("building_glass")};
  placed.position = {{7.0, 7.0, 0.0}};
  w.add(placed);
  geometry_item ghost{"ghost", kiosk}; // drawn only: no footprint for the navigator
  ghost.roles = role_render;
  ghost.position = {{1.0, 8.0, 0.0}};
  w.add(ghost);

  const cvc::nav::world_planes p = cvc::nav::rasterize(w, 11, 11);
  auto at = [&](const auto &v, int r, int c) { return v[static_cast<std::size_t>(r) * 11 + c]; };
  EXPECT_EQ(p.rows, 11);
  EXPECT_DOUBLE_EQ(p.max_x, 10.0);
  // Heights are the terrain's.
  EXPECT_FLOAT_EQ(at(p.height, 3, 6), static_cast<float>(field->height(6.0, 3.0)));
  // The building blocks its footprint and carries the highest face's surface.
  EXPECT_EQ(at(p.occupancy, 3, 3), 1);
  EXPECT_EQ(at(p.surface, 3, 3), s.by_name("building_brick"));
  // (4, 4) is under both: the tall building's roof is the higher face.
  EXPECT_EQ(at(p.surface, 4, 4), s.by_name("building_brick"));
  EXPECT_EQ(at(p.surface, 5, 5), s.by_name("building_metal")); // the shed alone
  // The placed kiosk blocks (7, 7), not the origin; the render-only ghost blocks nothing.
  EXPECT_EQ(at(p.occupancy, 7, 7), 1);
  EXPECT_EQ(at(p.occupancy, 0, 0), 0);
  EXPECT_EQ(at(p.occupancy, 8, 1), 0);
  // Open ground: the terrain's surface, its rho / mu, free.
  EXPECT_EQ(at(p.surface, 1, 6), asphalt);
  EXPECT_EQ(at(p.occupancy, 1, 6), 0);
  EXPECT_FLOAT_EQ(at(p.mu, 1, 6), s[asphalt].mu);
  EXPECT_FLOAT_EQ(at(p.risk_raw, 1, 6), s[asphalt].rho);
  // Deep water is hard, and hard is part of occupancy.
  EXPECT_EQ(at(p.hard, 5, 10), 1);
  EXPECT_EQ(at(p.occupancy, 5, 10), 1);
  EXPECT_EQ(at(p.surface, 5, 10), deep);

  // Inflation is the planner's 4-connected dilation.
  const cvc::nav::world_planes q = cvc::nav::rasterize(w, 11, 11, 1);
  EXPECT_EQ(q.occupancy, cvc::nav::inflate(p.occupancy.data(), 11, 11, 1));
  // No terrain, no extent to take.
  world_geometry empty;
  EXPECT_THROW(cvc::nav::rasterize(empty, 11, 11), std::invalid_argument);
  EXPECT_THROW(cvc::nav::rasterize(w, 1, 11, 0, 0, 10, 10), std::invalid_argument);
}

TEST(NavLandCover, PaletteAndSurfacesMeet) {
  const surface_registry &reg = surface_registry::builtin();
  EXPECT_EQ(cvc::nav::surface_for_land_cover(reg, cvc::nav::MAT_WATER, 0),
            reg.by_name("water_shallow"));
  EXPECT_EQ(cvc::nav::surface_for_land_cover(reg, cvc::nav::MAT_OPEN_AIR, 0),
            reg.by_name("asphalt"));
  EXPECT_EQ(cvc::nav::surface_for_land_cover(reg, 99, 7), 7u);
  // A surface's palette entry is its RF material's.
  EXPECT_EQ(cvc::nav::palette_id(reg[reg.by_name("dirt")]), cvc::nav::MAT_SOIL);
  EXPECT_EQ(cvc::nav::palette_id(reg[reg.by_name("building_brick")]), cvc::nav::MAT_BRICK);
  const std::vector<int> ids = cvc::nav::palette_ids(reg);
  ASSERT_EQ(ids.size(), reg.size());
  EXPECT_EQ(ids[reg.by_name("tall_grass")], cvc::nav::MAT_FOLIAGE);
  // The physics world's materials carry it.
  world_geometry g;
  g.add({"terrain", std::make_shared<cvc::height_field>(3, 3, 0, 0, 1, 1), reg.by_name("dirt")});
  cvc::physics::world_config c;
  c.backend = cvc::physics::backend::null;
  cvc::physics::world w(c);
  const auto bodies = cvc::physics::add_world_geometry(w, g, 0.005, ids);
  EXPECT_EQ(w.get_material(bodies.materials[reg.by_name("dirt")]).nav_material, cvc::nav::MAT_SOIL);
}

TEST(NavLandCover, AProviderWithNothingToClassifySaysNothing) {
  const fs::path dir = fs::temp_directory_path() / "cvc_nav_land_cover_none";
  fs::create_directories(dir);
  const auto provide = cvc::nav::land_cover_provider(surface_registry::builtin());
  EXPECT_TRUE(provide(dir.string(), 4, 4).empty());
  // A bundle's extent can be read without its heights (a masks-only bundle).
  std::ofstream(dir / "terrain.json")
      << R"({"rows":4,"cols":5,"bounds":{"min_x":-8,"min_y":-2,"max_x":8,"max_y":6}})";
  const terrain_extent e = read_terrain_extent(dir.string());
  EXPECT_EQ(e.rows, 4);
  EXPECT_EQ(e.cols, 5);
  EXPECT_DOUBLE_EQ(e.max_y, 6.0);
  fs::remove_all(dir);
}

// On a real bundle (CVC_CITY_BUNDLE=<.../austin_south>): land cover reaches the
// terrain's cells, and the navigator's occupancy is the buildings' footprint.
TEST(NavLandCover, RealBundleCellsCarryLandCover) {
  const char *dir = std::getenv("CVC_CITY_BUNDLE");
  if (!dir || !*dir)
    GTEST_SKIP() << "set CVC_CITY_BUNDLE to a city bundle directory to run this";
  city_bundle_options o;
  o.land_cover = cvc::nav::land_cover_provider(surface_registry::builtin());
  const city_bundle city = load_city_bundle(dir, o);
  ASSERT_TRUE(city.land_cover_loaded);
  const auto &cells = city.geometry.terrain()->cell_materials();
  ASSERT_FALSE(cells.empty());
  std::vector<int> count(surface_registry::builtin().size(), 0);
  for (const auto s : cells)
    ++count[s];
  int kinds = 0;
  for (const int n : count)
    kinds += n > 0;
  std::printf("[land-cover] %zu cells, %d surfaces\n", cells.size(), kinds);
  EXPECT_GE(kinds, 2) << "a city is more than one kind of ground";
  const auto planes = cvc::nav::rasterize(city.geometry, 96, 96);
  long blocked = 0;
  for (const auto v : planes.occupancy)
    blocked += v;
  EXPECT_GT(blocked, 0);
  EXPECT_LT(blocked, 96 * 96);
}

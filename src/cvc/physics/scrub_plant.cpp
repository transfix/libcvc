/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/scrub_plant.h>
#include <cvc/physics/world.h>
#include <cvc/world/world_geometry.h>
#include <memory>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace physics {

namespace {

// A rectangle of blocked cells: columns c0..c1 and rows r0..r1, inclusive.
struct cell_rect {
  int c0, c1, r0, r1;
};

// Each row's runs of blocked cells, a run merged with the one just above it when both span the
// same columns.
std::vector<cell_rect> wall_rects(const std::uint8_t *truth, int rows, int cols) {
  std::vector<cell_rect> out, open, next;
  for (int r = 0; r <= rows; ++r) {
    next.clear();
    if (r < rows) {
      const std::uint8_t *row = truth + static_cast<std::size_t>(r) * cols;
      for (int c = 0; c < cols;) {
        if (!row[c]) {
          ++c;
          continue;
        }
        int e = c;
        while (e + 1 < cols && row[e + 1])
          ++e;
        cell_rect q{c, e, r, r};
        for (cell_rect &o : open)
          if (o.c0 == c && o.c1 == e && o.r1 == r - 1) {
            q.r0 = o.r0;
            o.r1 = -1; // carried on
          }
        next.push_back(q);
        c = e + 1;
      }
    }
    for (const cell_rect &o : open)
      if (o.r1 >= 0)
        out.push_back(o);
    open.swap(next);
  }
  return out;
}

// A deployed run's plant: its host, and the geometry the host's world borrows.
struct world_plant {
  std::shared_ptr<const cvc::world::world_geometry> geometry;
  std::shared_ptr<plant_host> host;
};

} // namespace

int scrub_wall_boxes(const std::uint8_t *truth, int rows, int cols) {
  return truth && rows > 0 && cols > 0 ? static_cast<int>(wall_rects(truth, rows, cols).size()) : 0;
}

std::string scrub_default_plant() {
  return plant_host::available() ? "humvee" : cvc::nav::kScrubKinematic;
}

cvc::nav::scrub_plant_fn scrub_plant(plant_host_options o, double wall_height) {
  if (!(wall_height > 0.0))
    throw std::invalid_argument("cvc::physics::scrub_plant: the walls need a height above 0");
  return [o, wall_height](cvc::nav::sim_world &sim, const std::string &plant) {
    plant_host::check(plant); // an unknown name throws before anything changes
    auto host = std::make_shared<plant_host>(o);
    cvc::nav::scrub_plant_attached on;
    on.driving = host->use(sim, plant);
    on.owner = host;
    world *w = host->physics();
    if (!w || sim.rows() < 2 || sim.cols() < 2)
      return on; // the bicycle (no Jolt), or no grid to wall
    const cvc::nav::sim_world::config &c = sim.current_config();
    const double m = o.plant.metres_per_world_unit;
    const double dx = (c.max_x - c.min_x) / (sim.cols() - 1) * m,
                 dy = (c.max_y - c.min_y) / (sim.rows() - 1) * m;
    const material_id wall = w->add_material({"wall", 0.9f, 0.0f, 0.0f});
    for (const cell_rect &q : wall_rects(sim.truth(), sim.rows(), sim.cols())) {
      body_desc b;
      b.shape =
          box{{0.5 * (q.c1 - q.c0 + 1) * dx, 0.5 * (q.r1 - q.r0 + 1) * dy, 0.5 * wall_height}};
      b.motion = motion_type::static_;
      b.position = {c.min_x * m + 0.5 * (q.c0 + q.c1) * dx, c.min_y * m + 0.5 * (q.r0 + q.r1) * dy,
                    o.ground_z + 0.5 * wall_height};
      b.material = wall;
      w->add_body(b);
    }
    return on;
  };
}

cvc::nav::scrub_plant_fn scrub_plant(std::shared_ptr<const cvc::world::world_geometry> world,
                                     plant_host_options o, std::vector<int> nav_materials) {
  if (!world)
    throw std::invalid_argument("cvc::physics::scrub_plant: no world geometry");
  if (!o.plant.ground_height)
    if (std::shared_ptr<const cvc::height_field> t = world->terrain())
      // the vehicles start on the terrain, not at ground_z under it (a body that starts below a
      // height field falls through it); off it, or over a hole, at ground_z
      o.plant.ground_height = [t, z = o.ground_z](double x, double y) {
        const double h = t->height(x, y);
        return std::isfinite(h) ? h : z;
      };
  return [world, o, nav_materials](cvc::nav::sim_world &sim, const std::string &plant) {
    plant_host::check(plant);
    auto p = std::make_shared<world_plant>();
    p->geometry = world;
    p->host = std::make_shared<plant_host>(o);
    p->host->set_geometry(world.get(), nav_materials);
    cvc::nav::scrub_plant_attached on;
    on.driving = p->host->use(sim, plant);
    on.owner = p;
    return on;
  };
}

} // namespace physics
} // namespace cvc

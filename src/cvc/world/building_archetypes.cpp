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

// building_archetypes.cpp — see building_archetypes.h. GL-free; no RNG (deterministic).

#include <cmath>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h> // read_model
#include <cvc/world/building_archetypes.h>
#include <exception>
#include <map>
#include <string>
#include <vector>

namespace cvc {
namespace world {
namespace {

// The building id a mesh belongs to: its name up to the last '_'. "bldg_000123_walls" ->
// "bldg_000123". A name with no '_' is its own group.
std::string building_id(const std::string &name) {
  const std::size_t u = name.find_last_of('_');
  return u == std::string::npos ? name : name.substr(0, u);
}

// Translate a geometry in place by (dx,dy,dz).
void translate(cvc::geometry &g, double dx, double dy, double dz) {
  for (auto &p : g.points()) {
    p[0] += dx;
    p[1] += dy;
    p[2] += dz;
  }
}

} // namespace

building_library building_library_from_model(const cvc::model &m) {
  building_library lib;
  // Group meshes by building id, preserving first-seen order.
  std::map<std::string, int> index;  // id -> archetype slot
  std::vector<std::string> order;    // ids in first-seen order
  std::vector<cvc::geometry> merged; // accumulated geometry per group
  for (const cvc::model::mesh &ms : m.meshes) {
    if (ms.geom.const_points().empty())
      continue;
    const std::string id = building_id(ms.name);
    auto it = index.find(id);
    if (it == index.end()) {
      index[id] = int(merged.size());
      order.push_back(id);
      merged.push_back(ms.geom);
    } else {
      merged[it->second].merge(ms.geom);
    }
  }

  lib.archetypes.reserve(merged.size());
  for (std::size_t i = 0; i < merged.size(); ++i) {
    cvc::geometry &g = merged[i];
    if (g.const_points().empty())
      continue;
    const cvc::bounding_box bb = g.extents();
    const double cx = 0.5 * (bb.minx + bb.maxx), cy = 0.5 * (bb.miny + bb.maxy);
    translate(g, -cx, -cy, -bb.minz); // centre XY, base to z=0
    building_archetype a;
    a.name = order[i];
    a.footprint_x = 0.5 * (bb.maxx - bb.minx);
    a.footprint_y = 0.5 * (bb.maxy - bb.miny);
    a.height = bb.maxz - bb.minz;
    a.mesh = std::move(g);
    if (a.footprint_x > 1e-6 && a.footprint_y > 1e-6 && a.height > 1e-6)
      lib.archetypes.push_back(std::move(a));
  }
  return lib;
}

building_library load_building_library(const std::string &path) {
  try {
    cvc::model m = cvc::read_model(path);
    return building_library_from_model(m);
  } catch (const std::exception &) {
    return building_library{};
  }
}

cvc::geometry stamp_buildings(const building_library &lib,
                              const std::vector<building_instance> &instances) {
  cvc::geometry out;
  for (const building_instance &in : instances) {
    if (in.archetype < 0 || in.archetype >= int(lib.archetypes.size()))
      continue;
    const building_archetype &a = lib.archetypes[in.archetype];
    cvc::geometry g = a.mesh; // a transformed copy to merge in
    const double c = std::cos(in.yaw), s = std::sin(in.yaw);
    for (auto &p : g.points()) {
      const double x = p[0] * in.scale, y = p[1] * in.scale, z = p[2] * in.scale;
      p[0] = x * c - y * s + in.x; // scale -> yaw about +Z -> translate
      p[1] = x * s + y * c + in.y;
      p[2] = z + in.z;
    }
    // Rotate the normals too (scale/translate don't affect a unit normal; yaw does).
    if (g.const_normals().size() == g.const_points().size())
      for (auto &n : g.normals()) {
        const double nx = n[0], ny = n[1];
        n[0] = nx * c - ny * s;
        n[1] = nx * s + ny * c;
      }
    out.merge(g);
  }
  return out;
}

} // namespace world
} // namespace cvc

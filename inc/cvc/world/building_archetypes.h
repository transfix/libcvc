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

// building_archetypes.h — real building meshes as reusable, placeable archetypes.
//
// The published Austin city bundle the nav city demos load (buildings.glb) is ~5,600 real
// buildings, each authored as separate meshes (bldg_NNNNNN_walls + bldg_NNNNNN_roof). nav's load
// path immediately model::merged()s the whole thing into one triangle soup; this module instead
// keeps the buildings SEPARATE so a procedural city can draw from them as an archetype library —
// group the source meshes by their building id, merge + normalize each into one archetype (centred
// in XY, base lowered to z=0, with its footprint extent + height recorded), then stamp chosen
// archetypes along the road network (scaled/oriented to fit a lot). GL-free — it produces plain
// cvc::geometry.

#ifndef CVC_WORLD_BUILDING_ARCHETYPES_H
#define CVC_WORLD_BUILDING_ARCHETYPES_H

#include <cvc/geometry/geometry.h> // building_archetype owns a cvc::geometry by value
#include <string>
#include <vector>

namespace cvc {
class model; // cvc::read_model result (inc/cvc/model/model_file_io.h)

namespace world {

// One building, normalized: the mesh is centred in XY with its base on z=0, and its footprint
// half-extents + height (in the source authoring scale, metres) are recorded for lot fitting.
struct building_archetype {
  cvc::geometry mesh;
  double footprint_x = 0.0, footprint_y = 0.0; // half-extents in XY (metres)
  double height = 0.0;                         // metres
  std::string name;                            // source building id (e.g. "bldg_000123")
};

struct building_library {
  std::vector<building_archetype> archetypes;
  bool empty() const { return archetypes.empty(); }
  std::size_t size() const { return archetypes.size(); }
};

// Build a library from a loaded model. Meshes are grouped by their building id — the mesh name up
// to the last '_', so "bldg_000123_walls" and "bldg_000123_roof" become one archetype; a name with
// no
// '_' is its own archetype. Each group is merged + normalized. Empty/degenerate groups are dropped.
// First-seen id order is preserved (deterministic).
building_library building_library_from_model(const cvc::model &m);

// Load a building library from a glb/gltf file (wraps cvc::read_model). Empty on failure/missing
// file.
building_library load_building_library(const std::string &path);

// One placed building: which archetype, where (footprint-centre / base at x,y,z), its rotation
// about +Z, and a uniform scale.
struct building_instance {
  int archetype = 0;
  double x = 0.0, y = 0.0, z = 0.0;
  double yaw = 0.0; // radians about +Z
  double scale = 1.0;
};

// Stamp all instances into ONE merged cvc::geometry (scale, then yaw about +Z, then translate;
// indices offset per instance; normals rotated). An out-of-range archetype index is skipped.
// Deterministic and order-stable — the same instances always produce the same mesh.
cvc::geometry stamp_buildings(const building_library &lib,
                              const std::vector<building_instance> &instances);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_BUILDING_ARCHETYPES_H

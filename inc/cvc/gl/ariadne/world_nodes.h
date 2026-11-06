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

// world_nodes.h — the generated_world Ariadne scene node type: cvc::world::generate_world
// (inc/cvc/world/generated_world.h) drawn as a handful of merged meshes and its water surfaces.
//
//   - node: world
//     type: generated_world
//     seed: 5            # half, trees, rocks, buildings, tree_gen, amp, water_level, preset,
//     preset: standard   # ontology, color (material | risk | occupancy | height), terrain,
//     terrain: fbm       # city, terrain_file: each seeds a world.* state key that
//     city: districts    # world_controls.ari's panel drives live
//     look: soft
//     building_library: cvc://data/austin_south/buildings.glb   # optional: roads-first city
//
// `terrain:` and `city:` name cvc::world generators (fbm | dem | a host's; districts | none | a
// host's); `terrain_file:` is the dem generator's .npy, a URI or a path relative to the document.
// The node publishes the registered names (world.generators.terrain / .city, one per line) for the
// panel's combos. A generator that fails keeps the current world and says why in world.status.
//
// The node itself is the terrain: a GeometryNode with the surface texture (material, roads,
// plazas), its cvc::height_field published to RealizedScene::height_fields under the node's id, so
// `ground: <id>` (forest_trees, cloud_sky, shadows.focus) stands on it. Beside it, top-level nodes
// <id>_props, _buildings, _roads, _foliage, _needles (merged meshes) and _water (a water surface
// per terrain chunk with water in it, coloured by depth and on its chunk's LOD rung; `water: 0`
// leaves it out). They follow the node's visibility: a `visible:` on the node hides or shows the
// whole world. A change to a world.* key
// regenerates the world in place; a change of world.color only repaints the texture.
//
// `look: soft` is demo3's daylight materials (matte ground and concrete); `classic` is
// terrain_lab's. The building library is the Austin archetypes (cvc::world::load_building_library):
// the prop's URI, else $CVC_NAV_BUNDLE, else /bundle/buildings.glb (a wasm mount); without one the
// city's buildings are procedural massing.

#ifndef CVC_GL_ARIADNE_WORLD_NODES_H
#define CVC_GL_ARIADNE_WORLD_NODES_H

namespace cvc {
namespace gl {
namespace ariadne {

// Register the generated_world scene node type. Idempotent; call before realize_scene (the cvcGL
// extension bundle, register_cvcgl_extensions, does).
void register_world_node_types();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_WORLD_NODES_H

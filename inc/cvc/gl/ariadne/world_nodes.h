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
//
// `body: {}` on the node puts the world's colliders, its terrain and buildings, in the scene's
// physics world, and a new world replaces them. The params, keys, requests, stats and colliders are
// core libcvc's (cvc/ariadne/scene_generated_world.h), which a headless host runs the node with, so
// a document's generated world and its bodies are the same drawn or not.

#ifndef CVC_GL_ARIADNE_WORLD_NODES_H
#define CVC_GL_ARIADNE_WORLD_NODES_H

#include <memory>
#include <string>

class vtkRenderer;

namespace cvc {
class app;
namespace ariadne {
struct SceneNode;
}
namespace world {
struct generated_world;
}
namespace gl {
class GraphicsNode;
class SceneGraph;
namespace ariadne {

// Register the generated_world scene node type on `app`. Idempotent; call before realize_scene
// (the cvcGL extension bundle, register_cvcgl_extensions, does).
void register_world_node_types(cvc::app &app);

// ── A world dataset, drawn with the generated world's renderer ──────────────────────────────────
// (ARIADNE_WORLDS section 18.8, "one renderer"). A `source: { data }` node whose dataset is a
// world (a city bundle's `world`, Austin's or a saved generated world's) draws it as the
// generated_world node draws its own: the terrain as LOD chunks wearing the dataset's ground
// (ground.png, else the aerial image satellite.png; the terrain's surface colour without either),
// the props as LOD cells swaying in the scene's wind, the buildings and roads, and the water by
// depth with the sea or the rock past the edge. The drawn world is made from the dataset by
// cvc::world::world_from_bundle in a sliced job (natively on the background pool), and goes into
// the scene a piece at a time; a reloaded dataset's world replaces the shown one once it is all
// in. Its nodes are children of the data node's group: <id>_terrain (which holds the ground's
// texture; its chunks are its children) and the parts <id>_props, _foliage, _needles, _buildings,
// _roads, _water and _underside.
//
// The node's props: `look` (soft | classic), `water` (0 leaves the water out), `underside` (1
// draws the rock under a world with walls or no edge; 0, the default, leaves it out), `texture`
// (the water texture's size, 2048), `sway` (the trees' sway, 1) and `tint: satellite` (the
// buildings and roads coloured by the ground under them). Its progress is under <id>.draw.*: busy,
// progress, phase and status (an error says why, and the shown world stays).
//
// scene_realize makes a view for each data node at realize (make_data_world_view) and, as a
// dataset loads, sync_scene_data calls data_world_load for a world and data_world_clear for
// anything else; tick_scene calls data_world_tick each frame. Frame-thread only.
class DataWorldView;
std::shared_ptr<DataWorldView> make_data_world_view(SceneGraph &sg,
                                                    const cvc::ariadne::SceneNode &n,
                                                    const std::shared_ptr<GraphicsNode> &group,
                                                    const std::string &bind_prefix);
// Draw the world dataset at store path `store` (the city_bundle decoder's outputs and the
// dataset's files): a job starts, ending the one that was running.
void data_world_load(DataWorldView &v, const std::string &store);
// Draw nothing: the job ends and the shown world's nodes go (a few a frame, data_world_tick).
void data_world_clear(DataWorldView &v);
// A frame: a finished job is taken in, the world goes in or out a piece at a time, and the LOD
// follows `ren`'s camera (null: no LOD change).
void data_world_tick(DataWorldView &v, vtkRenderer *ren);
// Whether a job runs or a world is going in.
bool data_world_busy(const DataWorldView &v);
// Run the job and the install to their end now, and free what is on its way out: for tests and
// hosts that capture a settled frame. True when a world is shown.
bool data_world_finish(DataWorldView &v);
// The world shown (null before the first is in, or after data_world_clear).
std::shared_ptr<const cvc::world::generated_world> data_world_shown(const DataWorldView &v);
// <id>.draw.status: "drawing...", "error: ..." or empty.
std::string data_world_status(const DataWorldView &v);

// Register the water scene node type on `app` (water_node.cpp: a ground's sea, lakes and ponds as
// the GPU FFT ocean, OCEAN-AND-WEATHER-ROADMAP.md §11.2). Idempotent; the cvcGL extension bundle
// calls it.
void register_water_node_types(cvc::app &app);

// Register the boat scene node type on `app` (boat_node.cpp: a boat's hull drawn at its pose, the
// boat run by the scene's world, scene_boat.h; OCEAN-AND-WEATHER-ROADMAP.md D6). Idempotent; the
// cvcGL extension bundle calls it.
void register_boat_node_types(cvc::app &app);
// Register the weather scene node type on `app` (weather_node.cpp: the scene's weather model,
// cvc::world::weather, its storms in a cloud_sky's deck, OCEAN-AND-WEATHER-ROADMAP.md §9 F2).
// Idempotent; the cvcGL extension bundle calls it.
void register_weather_node_types(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_WORLD_NODES_H

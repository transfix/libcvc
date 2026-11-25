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

// A document's `type: generated_world` node in core libcvc: its params, its world.* keys, the
// requests a panel sends (Regenerate, Randomize, Stop), its stats and its colliders. Both hosts use
// these: cvcGL's realizer (world_nodes.cpp) draws the world and builds a new one as a sliced task,
// and a headless host (cvc::ariadne::SceneWorld::from_scene) runs it with no renderer
// (ARIADNE_WORLDS.md section 14, "Headless first"). The world itself is cvc::world::generate_world
// (cvc/world/generated_world.h).
//
//   GeneratedWorldRuntime rt(app, node, prefix, &warnings); // seeds world.*, makes the first world
//   rt.world();       // the cvc::world::generated_world: terrain, raster, roads, buildings, water
//   rt.colliders();   // what the node's body: collides with: the terrain and the buildings
//   rt.tick();        // a request or (world.auto) an edit makes a new world, at once
//
// Each world the node makes is also its dataset (ARIADNE_WORLDS section 18.8), <prefix>.data.<name>
// (the node's `data:` prop, else its id): the world written as a version 2 city bundle and read
// back, as `data-generate` makes one and as a loaded city is. Its physics is that dataset's
// world, so a generated world and a loaded one reach the physics, the navigator and the renderer
// the same way; `generated` beside it is the world as made (its raster, roads and props).

#ifndef __CVC_ARIADNE_SCENE_GENERATED_WORLD_H__
#define __CVC_ARIADNE_SCENE_GENERATED_WORLD_H__

#include <cstdint>
#include <cvc/ariadne/data.h>
#include <cvc/world/generated_world.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class app;

namespace world {
class world_geometry;
}

namespace ariadne {

struct ResolvedFile;
struct SceneNode;

// What a node's body: collides with when the node makes it at run time (a generated world, a
// dataset): static geometry, replaced whole. Whoever owns it bumps `revision` with each new
// geometry, and the scene's world swaps the node's bodies at its next sync, or, for a change timed
// on the world clock, at the physics step that `from_tick` names.
struct SceneColliders {
  std::shared_ptr<const cvc::world::world_geometry> geometry; // null: none
  std::uint64_t revision = 0;
  // When this revision goes in: 0 at the world's next sync; else at the first physics step that
  // ends past the world clock's tick `from_tick`, so every host and frame rate puts it in at the
  // same step (a water's sea keyframe in a scene with storms, scene_water.h). One whose step has
  // gone by (the physics was off) goes in at the next sync.
  std::uint64_t from_tick = 0;
  // Set by the scene's world each sync: the period (s of world-clock time) on which a runtime
  // whose changes follow the clock makes them (the water's sea while the scene has storms:
  // kSceneWindFrontPeriod), or 0 for whenever it ticks.
  double clock_period = 0.0;
};

// The params a generated_world node states: its preset's counts and relief, then the counts,
// relief, water level, ontology and generators it gives.
cvc::world::generated_world_params generated_world_params_from(const SceneNode &n);

// The building archetypes for node n: its `building_library` (a URI or a path relative to the
// document), else $CVC_NAV_BUNDLE (a buildings.glb, or a directory holding one), else the wasm
// mount /bundle/buildings.glb. The first that loads wins, each loaded once per process (with the
// model handlers of the app that first asks); empty when none does (the city's buildings are then
// procedural).
const cvc::world::building_library &generated_world_library(cvc::app &app, const SceneNode &n,
                                                            std::vector<std::string> *warnings);
// The same from a building_library URI (empty: none) relative to `base_dir`; `owner` names who
// asked in a warning.
const cvc::world::building_library &generated_world_library(cvc::app &app, const std::string &uri,
                                                            const std::string &base_dir,
                                                            const std::string &owner,
                                                            std::vector<std::string> *warnings);

// ── A generated world as a dataset (data.h; ARIADNE_WORLDS section 18.8) ──

// The data generator `generated_world` (register_data_generator's built-in): the world for
// d.spec.params (a JSON object: generated_world_params_json's fields, any left out the defaults,
// a `preset` first quick-setting the counts it names, and a `building_library` URI), relative to
// d.spec.base, as a version 2 city bundle's files; `generated` under d.node is the world as made.
std::vector<cvc::archive::file> generated_world_dataset(DataDecode &d);

// The name node n's world goes under as a dataset: its `data:` prop, else its id.
std::string generated_world_data_name(const SceneNode &n);

// World w, which node n made, as n's dataset, now: <bind_prefix>.data.<generated_world_data_name>
// linked to it (DataStore::put), its bundle's files decoded as a loaded city's are and
// `generated` the world as made. `bundle` is w's bundle when it was written already (off the
// frame thread: cvc::world::generated_world_bundle), else it is written here. Where no glb can be
// written (Assimp off), `bundle` is empty, or the bundle fails (it warns), the dataset is the
// world as made. Returns the store path.
std::string publish_generated_world(cvc::app &app, const std::string &bind_prefix,
                                    const SceneNode &n,
                                    std::shared_ptr<const cvc::world::generated_world> w,
                                    std::vector<std::string> *warnings,
                                    const cvc::world::city_bundle_files *bundle = nullptr);

// A dataset's world (its `world`, the colliders a body on it gets) and the generated world that
// made it (its `generated`), from its store path or a link to it; null when it has none.
std::shared_ptr<const cvc::world::world_geometry> dataset_world(cvc::app &app,
                                                                const std::string &path);
std::shared_ptr<const cvc::world::generated_world> dataset_generated_world(cvc::app &app,
                                                                           const std::string &path);

// The world.* keys a world_controls panel drives, as absolute state paths: world.<k>, or under the
// node's `keys:` prop (node_key) so two worlds keep separate panels.
struct GeneratedWorldKeys {
  std::string seed, half, trees, rocks, buildings, tree_gen, amp, water_level, preset, ontology,
      color, request, status, props, building_count, occupied, terrain, water, terrain_gen, city,
      terrain_file, edge, busy, progress, phase, autoregen, pending;
  // Save (a "save" request): world.save.file, world.save.exports, world.save.status.
  std::string save_file, save_exports, save_status;
};

// The keys for node n under `bind_prefix`, each seeded from `base` (and `color`, a
// cvc::world::surface_color) where it has no value yet; world.request, .pending, .busy, .progress
// and .phase start empty or 0, and world.generators.terrain / .city list the registered generators
// one per line.
GeneratedWorldKeys generated_world_keys(cvc::app &app, const SceneNode &n,
                                        const std::string &bind_prefix,
                                        const cvc::world::generated_world_params &base, int color);

// The params the keys hold now, `base` where a key has none.
cvc::world::generated_world_params
read_generated_world_params(cvc::app &app, const GeneratedWorldKeys &k,
                            const cvc::world::generated_world_params &base);

// The params as one string: two worlds from the same signature are the same world.
std::string generated_world_signature(const cvc::world::generated_world_params &p);

// p with its terrain_file a local path (a URI, or a path relative to `base_dir`); `f` keeps a
// fetched file alive until the world is made. Throws std::runtime_error when it cannot be had.
cvc::world::generated_world_params
resolve_generated_world_file(cvc::app &app, cvc::world::generated_world_params p,
                             const std::string &base_dir, ResolvedFile &f);

// The world for p, at once. On a failure (an unknown generator, a missing DEM) world.status says
// why, a warning names node n, and the world is made from the default generators instead, which
// the keys then name; `p` is left as the params of the world made.
std::shared_ptr<cvc::world::generated_world>
first_generated_world(cvc::app &app, const GeneratedWorldKeys &k, const SceneNode &n,
                      cvc::world::generated_world_params &p,
                      const cvc::world::building_library *library,
                      std::vector<std::string> *warnings);

// world.stats.{props, buildings, occupied, terrain} and world.status for w.
void publish_generated_world_stats(cvc::app &app, const GeneratedWorldKeys &k,
                                   const cvc::world::generated_world &w);

// What the keys ask for this frame. Regenerate (or Randomize, which steps world.seed first) asks
// for a new world from the keys as they are; so does any changed key while world.auto is on.
// Otherwise an edit only marks the world pending (world.pending). A new preset quick-sets the
// counts, relief and water level. Stop ends a job under way. `preset` is the preset the counts last
// came from and `sig` the signature of the world shown or being made; both are updated, and the
// request key is cleared.
struct GeneratedWorldPoll {
  bool stop = false;    // a "stop" request
  bool save = false;    // a "save" request: save_generated_world
  bool rebuild = false; // make a new world from `params`
  cvc::world::generated_world_params params;
};
GeneratedWorldPoll poll_generated_world(cvc::app &app, const GeneratedWorldKeys &k,
                                        const cvc::world::generated_world_params &base, int &preset,
                                        std::string &sig);

// A "save" request (world_controls' Save, ARIADNE_WORLDS section 18.8, GB4): the node's dataset at
// `data_path`, the world shown, saved as world.save.file (data_save_files: a directory beside the
// document, or a zip when it ends in .zip; in a browser a zip download), with its files for other
// software (export/: the terrain as a glb, a 16-bit height map) when world.save.exports is on.
// world.save.status says where it went, or "error: " and why. Returns true when it saved.
bool save_generated_world(cvc::app &app, const GeneratedWorldKeys &k, const std::string &data_path,
                          const std::string &base_dir);

// The world's colliders: its terrain and its buildings (cvc::world::generated_world::geometry, the
// collide items).
std::shared_ptr<const cvc::world::world_geometry>
generated_world_colliders(const cvc::world::generated_world &w);

// A generated_world node run with no renderer: its keys seeded, its first world made, and each
// tick() a request or (world.auto) an edit makes a new world at once, its colliders replaced.
// world.busy stays 0 (nothing runs in the background); Stop has nothing to end.
class GeneratedWorldRuntime {
public:
  GeneratedWorldRuntime(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                        std::vector<std::string> *warnings);
  ~GeneratedWorldRuntime();
  GeneratedWorldRuntime(const GeneratedWorldRuntime &) = delete;
  GeneratedWorldRuntime &operator=(const GeneratedWorldRuntime &) = delete;

  const std::string &node() const;
  std::shared_ptr<const cvc::world::generated_world> world() const;
  const std::shared_ptr<SceneColliders> &colliders() const;
  const GeneratedWorldKeys &keys() const;
  // The world's dataset (publish_generated_world): its store path. The colliders are its world.
  const std::string &data_path() const;

  // Read the keys and requests; make a new world when asked. True when one was made. A failure
  // keeps the world and says why in world.status (and in `warnings`).
  bool tick(std::vector<std::string> *warnings = nullptr);

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_GENERATED_WORLD_H__

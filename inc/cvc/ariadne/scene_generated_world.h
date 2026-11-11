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

#ifndef __CVC_ARIADNE_SCENE_GENERATED_WORLD_H__
#define __CVC_ARIADNE_SCENE_GENERATED_WORLD_H__

#include <cstdint>
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
// geometry, and the scene's world swaps the node's bodies at its next sync.
struct SceneColliders {
  std::shared_ptr<const cvc::world::world_geometry> geometry; // null: none
  std::uint64_t revision = 0;
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

// The world.* keys a world_controls panel drives, as absolute state paths: world.<k>, or under the
// node's `keys:` prop (node_key) so two worlds keep separate panels.
struct GeneratedWorldKeys {
  std::string seed, half, trees, rocks, buildings, tree_gen, amp, water_level, preset, ontology,
      color, request, status, props, building_count, occupied, terrain, water, terrain_gen, city,
      terrain_file, busy, progress, phase, autoregen, pending;
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
resolve_generated_world_file(cvc::world::generated_world_params p, const std::string &base_dir,
                             ResolvedFile &f);

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
  bool rebuild = false; // make a new world from `params`
  cvc::world::generated_world_params params;
};
GeneratedWorldPoll poll_generated_world(cvc::app &app, const GeneratedWorldKeys &k,
                                        const cvc::world::generated_world_params &base, int &preset,
                                        std::string &sig);

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

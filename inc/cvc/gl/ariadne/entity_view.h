/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// entity_view.h: a world's vehicles drawn at scale (FLEETS_AND_NIGHT_ROADMAP.md §2.2, FL2), the
// `entity_layer` scene node and the view that draws a set of entities, wherever they come from.
//
//   - node: city_bodies
//     type: entity_layer
//     of: { fleet: city }        # or { archetype: humvee } | { team: 3 } | all (the default)
//     layer: bodies              # the entities' bodies (the one layer yet)
//     lod: { near_cap: 10, px: [24, 8, 3] }   # or lod: false: every entity on the finest rung
//     promote: 8                 # drawn as their own nodes: the followed, then the nearest
//     threshold: 32              # a layer of at most this many draws each as its own node
//
// A bodies layer draws the entities it takes as one mesh per archetype and LOD rung: a fitted
// model's rungs are quadric-decimated copies of it (the finest textured, the others in each
// entity's colour), a box look has one. A boat (a dynamicboat vehicle) is drawn about its hull
// body's origin as the boat node draws it: its model fitted to its length overall, its keel on its
// hull's bottom (boat_fit_of), and a box look's template its hull, flat-shaded as add_hull_node
// draws it (its topsides and stripe in the entity's colours, its wet bottom its own; a box hull
// open, with its teak sole). A relay's client knows a boat's length overall and keel but not its
// hull, so its box look there is the box. Each frame the meshes are posed from the entities' poses
// and an entity off screen is drawn by no rung. The entities on screen are ranked by their size
// there, the largest taking the finest rung up to its cap (`near_cap` for the finest; 28 and 72
// for the next) while their radius on screen is at least that rung's `px`.
//
// Promotion: `promote` of a layer's entities are drawn as their own nodes, as a view draws every
// entity without the entity view (their wheels posed, their label): the ones followed (a camera's
// `track.target` naming `vehicle_<id>` or `entity_<id>`, `vehicles.watch`, the vehicle a client
// drives), then the nearest to the camera. A promoted entity keeps its place until another is a
// fifth nearer, so a camera moving among them does not swap them frame by frame. A layer with at
// most `threshold` entities promotes them all.
//
// An entity goes to the first bodies layer (in the document's order) whose `of:` takes it, a boat
// as a car does. One no layer takes, or one whose look no layer can draw (a mesh file without
// `fit`) keeps a node of its own. A view with no bodies layer has one over all of its entities
// (`entities`, the defaults above), so a world of up to 32 entities looks as it did before the
// entity view. The water node keeps its water out of the nearest boats' hulls however they are
// drawn: it reads where the world's boats are, not their nodes.
//
// The same view draws both holons' entities (ARIADNE_WORLDS.md §18): a scene's own world
// (update_entity_view: the world's vehicles posed from the physics binding, between the last two
// steps, whether the frame steps the world or a local holon on a worker does, world_worker.h), and
// a relay's on its clients (room_view: the entities section's identities, posed where the client's
// entity_playout shows them).

#ifndef CVC_GL_ARIADNE_ENTITY_VIEW_H
#define CVC_GL_ARIADNE_ENTITY_VIEW_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/world_rules.h> // vehicle_look
#include <cvc/gl/ariadne/vehicle_view.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

class vtkRenderer;

namespace cvc {
class app;
namespace ariadne {
struct SceneNode;
}
namespace gl {
class GraphicsNode;
class SceneGraph;
namespace ariadne {

struct RealizedScene;

// One entity as the view draws it: who it is, for a layer's of:, and how it looks.
struct entity_draw_info {
  std::uint64_t id = 0;
  std::string fleet, archetype;
  int team = 0;
  std::array<float, 3> color{0.85f, 0.70f, 0.20f}, color2{0.12f, 0.12f, 0.14f};
  cvc::ariadne::vehicle_look look; // a box look, or a fitted model's
  vehicle_shape shape;             // its chassis box and wheels
  double base_z = -1.0;            // a model's base below the chassis's centre (negative)
  double fit_length = 0.0;         // the length a model is fitted to; 0: shape.length
  // A boat's hull (its preset's, when the view knows it): its box look's template is the hull
  // (vehicle_look_mesh_of), and its hull while its model loads. Null: the box look's boxes.
  std::shared_ptr<const cvc::physics::boat_desc> boat;
  // Where it is now, when known: who is nearest until its first pose() says.
  std::array<double, 3> position{0.0, 0.0, 0.0};
  bool has_position = false;
};

// What a view draws now, for a test or a readout.
struct entity_view_stats {
  std::size_t layers = 0;    // bodies layers, the implicit one too
  std::size_t members = 0;   // entities the layers take
  std::size_t promoted = 0;  // of those, drawn as their own nodes
  std::size_t instanced = 0; // drawn by a rung in the last draw (on screen, not promoted)
  std::size_t meshes = 0;    // the rungs' mesh nodes
  std::size_t rebuilds = 0;  // times the layers were made (set_entities)
};

// The view of a set of entities: its bodies layers, who they promote, and their rungs. On the
// render thread. Each frame: set_entities when the entities changed (and when pending_settled()
// says a model came in), promote(), then pose() every posed_ids() entry and draw().
class entity_view {
public:
  // IMPLICIT_ID: the node the implicit layer draws under (a scene's `entities`, a room's
  // `room_entities`).
  explicit entity_view(std::string implicit_id = "entities");
  ~entity_view();
  entity_view(const entity_view &) = delete;
  entity_view &operator=(const entity_view &) = delete;

  // A bodies layer as an entity_layer node declares it (its props, its material:), drawn under
  // `group`. False (a warning says why) for a layer it does not draw.
  bool add_layer(const cvc::ariadne::SceneNode &node, const std::shared_ptr<GraphicsNode> &group,
                 std::vector<std::string> *warnings = nullptr);
  // Whether a document declared its layers (else set_entities makes the implicit one, under the
  // scene's root as the node implicit_id()).
  bool declared() const;
  const std::string &implicit_id() const;

  // The entities now: each to the first layer that takes it, and each layer's meshes made again.
  // `models` the view's fitted models (vehicle_view.h).
  void set_entities(SceneGraph &sg, vehicle_model_cache &models,
                    const std::vector<entity_draw_info> &entities);
  // The entities the rungs draw, in the order pose() takes them.
  const std::vector<std::uint64_t> &posed_ids() const;
  // Whether a look's model was loading at set_entities and has come in (or failed) since: call
  // set_entities again. Checked every 30th call.
  bool pending_settled(cvc::app &app, vehicle_model_cache &models);

  // Who is drawn as their own node this frame, from the camera `renderer` has and the ids
  // followed.
  void promote(vtkRenderer *renderer, const std::set<std::uint64_t> &followed);
  // Whether entity `id` is drawn as its own node: promoted, or in no layer.
  bool drawn_as_node(std::uint64_t id) const;
  // Whether who is drawn as their own node changed in the last promote() or set_entities().
  bool nodes_changed() const;

  // Entity posed_ids()[k]'s pose this frame (row-major 4x4, its chassis's centre), then draw().
  void pose(std::size_t k, const double m[16]);
  void draw();

  entity_view_stats stats() const;

  struct impl;

private:
  std::unique_ptr<impl> _impl;
};

// The ids a camera follows (`viewers.main.camera.track.target` naming `vehicle_<id>` or
// `entity_<id>`) or `vehicles.watch` names, under `prefix`.
std::set<std::uint64_t> followed_entity_ids(cvc::app &app, const std::string &prefix);

// Register "entity_layer" on `app`. Idempotent.
void register_entity_node_types(cvc::app &app);

// A scene's own world, each frame in tick_scene after the world steps and before
// sync_vehicle_markers: its view (RealizedScene::entities) given the world's entities when the
// vehicles' revision has moved (or a look's model has come in), and who is promoted. The binding's
// group of the instanced entities' chassis bodies poses the rungs in physics_scene_binding::
// present. A scene with vehicles and no bodies layer gets its implicit one here.
void update_entity_view(RealizedScene &realized, vtkRenderer *renderer);
// Whether a scene's entity `id` is drawn as its own node (vehicle_view.h).
bool entity_drawn_as_node(const RealizedScene &realized, std::uint64_t id);
// Whether who is drawn as their own node changed in the last update_entity_view.
bool entity_nodes_changed(const RealizedScene &realized);
entity_view_stats entity_view_stats_of(const RealizedScene &realized);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_ENTITY_VIEW_H

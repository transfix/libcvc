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

#ifndef __CVC_GL_ARIADNE_VEHICLE_VIEW_H__
#define __CVC_GL_ARIADNE_VEHICLE_VIEW_H__

#include <array>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/world_rules.h> // vehicle_look
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class geometry;
namespace physics {
struct boat_desc;
}
namespace ariadne {
struct entity_wheel;
}
namespace gl {
class GeometryNode;
class GraphicsNode;
class SceneGraph;
namespace ariadne {

struct RealizedScene;
struct drawn_wheel;         // a parted look's wheel as drawn (scene_realize.h)
struct vehicle_model_cache; // the vehicle models a realized scene has fitted

// ----------------
// A world's vehicles, drawn (ARIADNE_WORLDS.md §14, L2)
// ----------------
// The view half of the world runtime's vehicles (cvc::ariadne::WorldVehicles): each entity drawn
// on its chassis body, posed between the last two physics steps as the scene's other moving
// bodies are; and the `vehicle_preview` scene node, which draws the archetype a vehicle_selector
// has picked, alone on a studio set, turning.

// A vehicle's size, as the catalog publishes it (<prefix>.vehicles.archetypes.<id>.*).
struct vehicle_shape {
  double length = 4.84, width = 2.18, height = 0.9; // the chassis box (m)
  double wheelbase = 3.3, track = 1.82, wheel_radius = 0.45;
  double axle_x = 0.0; // the axles' middle, ahead of the chassis box's centre (m)
};

// How a vehicle model's file holds it (axes 0 x, 1 y, 2 z): `up` its up axis and `forward` the
// axis along its length, each -1 to find from its shape; `nose` the end of `forward` its nose is
// at, +1 or -1, 0 to find from its shape.
struct vehicle_fit_axes {
  int up = -1;
  int forward = -1;
  int nose = 0;
};

// A vehicle model's mesh made ready to draw on a chassis (a fitted file look): up +z, its base on
// z = 0; its length along x, its nose at +x, scaled to `length`; centred along and across. Found
// from its shape unless `axes` says: its up axis, the one it stands on (its least coordinate 0,
// within 2% of its largest extent, while on the other two it reaches past a tenth of its extent
// each way: a file built on its ground plane, Z-up or Y-up), else its shortest extent; its forward
// axis, the longer of the other two; its nose, the end whose top (its points in the top 40%)
// reaches less far from the middle (a hood before a cab, a bow before a cabin), or with too few
// points up there, the end with fewer points. A rotation, never a mirror. Throws
// std::runtime_error when the geometry has no points.
cvc::geometry fit_vehicle_geometry(cvc::geometry g, double length,
                                   const vehicle_fit_axes &axes = {});

// The scene node a vehicle is drawn as, about its chassis box's centre: a group `id` holding its
// look. The box look is the chassis box in `color`, a cabin in `color2` and four dark wheels; a
// file look is the mesh, placed by the look's scale, yaw and z.
cvc::ariadne::SceneNode vehicle_scene_node(const std::string &id,
                                           const cvc::ariadne::vehicle_look &look,
                                           const vehicle_shape &shape, const float color[3],
                                           const float color2[3]);

// The fitted vehicle models a view keeps (each model read and fitted once): one per view that
// draws vehicles, made here (a realized scene has one, room_view another).
std::shared_ptr<vehicle_model_cache> make_vehicle_model_cache();
// How far a fitted look's model has got, at a chassis length `length`. A file look with `fit` is
// ready (read and fitted, once) or failed. A data look (`kind` data: `path` in the package at
// `uri`, under `subpath`) asks the app's data store for its package (cvc::ariadne::DataStore,
// loaded once per request and cached), and is pending while it loads: draw the box or the hull
// meanwhile and ask again on a later frame; then ready, or, when the package cannot load (no
// network, no such package), a local install of the file by its name
// (cvc::ariadne::find_vehicle_model), else failed (a warning says why). Any other look has no
// fitted model: failed. attach_vehicle_model draws a ready one.
enum class vehicle_model_status { ready, pending, failed };
vehicle_model_status vehicle_model_state(cvc::app &app, vehicle_model_cache &cache,
                                         const cvc::ariadne::vehicle_look &look, double length,
                                         std::vector<std::string> *warnings = nullptr);
// A fitted file look (`look.kind` file, `look.fit`) or a data look under `group` as node `id`: the
// model scaled to the chassis's length, its base `base_z` below the group's origin (the ground
// under the chassis's centre, negative), turned and lifted by the look's yaw and z, in `color` when
// it has no texture. False when the model is not ready (vehicle_model_state says whether it is
// still loading or failed; a failure's warning says why); draw the box look then.
bool attach_vehicle_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                          const std::string &id, const cvc::ariadne::vehicle_look &look,
                          const vehicle_shape &shape, double base_z, const float color[3],
                          std::vector<std::string> *warnings = nullptr);

// ----------------
// A parted look (`chassis:`, `wheel:`, `rig:`; cvc::ariadne::vehicle_rig)
// ----------------
// A model in parts, fitted to its vehicle by its rig, each axis on its own: along x so the rig's
// axles lie on the vehicle's (its wheelbase over the rig's), across y so its track is the
// vehicle's, and up z so its wheels are the vehicle's size (the vehicle's wheel radius over the
// rig's); its ground (its hubs' mean height less its radius) on the ground under the chassis. An
// axis the rig's hubs do not spread along (one axle; every hub on the centre line) takes z's
// scale. Each wheel is the wheel mesh about its middle (its bounds' centre), scaled alike on every
// axis to the vehicle's wheel radius, at its hub; a right one (y < 0) mirrored as the rig says.

// How far a parted look's parts (its chassis, its wheel and its rig) have got: ready (each read
// once, in `cache`); pending while a data look's package loads (draw the box meanwhile and ask
// again on a later frame); failed when one cannot be read, or its rig is not one (a warning says
// why), or the look is not a parted one.
vehicle_model_status parted_model_state(cvc::app &app, vehicle_model_cache &cache,
                                        const cvc::ariadne::vehicle_look &look,
                                        std::vector<std::string> *warnings = nullptr);
// A ready parted look under `group` (a vehicle's node, about its chassis box's centre): the
// chassis as node `id`_chassis and each wheel, in the rig's order, as node `id`_wheel<k> (a group,
// its mesh `id`_wheel<k>_mesh under it), at rest; the chassis in `color` and the wheels a dark
// tyre's, where a part has no texture. `base_z` is the ground under the chassis, below the group's
// origin (negative). `wheels` (when not null) gets the wheels as drawn, for pose_vehicle_wheels.
// False when the parts are not ready (parted_model_state says why): draw the box look then.
bool attach_parted_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                         const std::string &id, const cvc::ariadne::vehicle_look &look,
                         const vehicle_shape &shape, double base_z, const float color[3],
                         std::vector<drawn_wheel> *wheels,
                         std::vector<std::string> *warnings = nullptr);
// Each drawn wheel's pose now (its node's matrix, row-major 4x4, in the chassis box's frame), from
// its vehicle's wheels as its plant or its poses have them (WorldVehicles::wheels): at its hub; at
// the height of the plant's wheel nearest it on its side (on its suspension, or the ground under
// it; at rest without one); rolled about its axle by that wheel's spin; and, when its rig steers
// it, turned about its upright by the steer of the plant's steered wheel nearest it (on its side
// first): a Jolt vehicle's or a dynamic4w's own angle, each wheel its own; a `pose` entity's, its
// pose's steer; none when the plant steers no wheel. Rz(steer) Ry(spin), as the box look's.
std::vector<std::array<double, 16>>
parted_wheel_poses(const std::vector<drawn_wheel> &wheels,
                   const std::vector<cvc::ariadne::entity_wheel> &plant);

// ----------------
// A scene node's model (`model:`)
// ----------------
// The look a scene node's `model:` value names (`where` and `instead` word its warnings: "ari:
// scene node 'crate_a'", "its own geometry"): `{ data: NAME, file: FILE }`, FILE in the package of
// the document's data: entry NAME (its uri and path: copied in, as resolve_vehicle_looks does); or
// a file; or a model's name found where the vehicle models are installed (`<name>.glb` too). Kind
// "" when it names none: `none`, or one that cannot be found (a warning says so).
cvc::ariadne::vehicle_look node_model_look(const cvc::ariadne::Value &model,
                                           const std::vector<cvc::ariadne::SceneData> &data,
                                           const std::string &where, const std::string &instead,
                                           std::vector<std::string> *warnings = nullptr);
// How far a look's model has got, fitted into the box lo..hi (a node's own geometry's bounds, in
// its coordinates): stood upright (glTF's +Y up turned to +Z, for a .glb or .gltf; any other file
// is taken as +Z up already) and scaled on each axis to fill the box. Ready, pending and failed as
// vehicle_model_state (a file or data look, kind file needing no `fit`), in the same cache.
vehicle_model_status boxed_model_state(cvc::app &app, vehicle_model_cache &cache,
                                       const cvc::ariadne::vehicle_look &look,
                                       const std::array<double, 3> &lo,
                                       const std::array<double, 3> &hi,
                                       std::vector<std::string> *warnings = nullptr);
// That model in place of `node`'s own geometry, its base colour texture on it (the node's colour
// then white, so it does not tint the texture). False when it is not ready. The node keeps its
// transform, and a body it has keeps the shape it was made with: set it after realize_physics (a
// scene tick), so the body is the node's own geometry, not the model's.
bool set_boxed_model(cvc::app &app, vehicle_model_cache &cache, GeometryNode &node,
                     const cvc::ariadne::vehicle_look &look, const std::array<double, 3> &lo,
                     const std::array<double, 3> &hi, std::vector<std::string> *warnings = nullptr);

// A boat's hull as drawn (OCEAN-AND-WEATHER-ROADMAP.md D5): its hull_mesh, flat-shaded, about the
// hull body's origin. Its topsides are `topsides`, with a band of `stripe` just over the waterline
// it floats at in fresh water; below that line it is a dark red, the hull darker where it is wet. A
// box hull (the presets) is drawn open: no deck, and a teak sole 0.12 m over the waterline, so the
// water node's exclusion is what keeps a wave out of its cockpit. The GeometryNode `id` under
// `parent` (top level, registered, when null). Null without a hull_mesh.
std::shared_ptr<GeometryNode> add_hull_node(SceneGraph &sg, const std::string &id,
                                            GraphicsNode *parent,
                                            const cvc::physics::boat_desc &boat,
                                            const float topsides[3], const float stripe[3]);

// Each frame after the world steps (tick_scene): when the vehicles' revision has moved, the
// entities' drawings are built again and bound to their chassis bodies. A boat's (dynamicboat)
// box look is its hull (add_hull_node), in its colours.
void sync_vehicle_markers(RealizedScene &realized, std::vector<std::string> *warnings);
// Each frame after the world steps: a box look's wheels, each where its plant has it
// (WorldVehicles::wheels): its height on its suspension or the ground under it, the front ones
// steered, every one rolled. A dynamic4w's four wheels each follow the ground under them. A
// parted look's wheels each as parted_wheel_poses has them. A one-piece model's wheels are the
// mesh's, and do not move.
void pose_vehicle_wheels(RealizedScene &realized);

// Each frame after the world steps: when the spawn points' revision has moved, their markers are
// built again: a pad in its team's colour with a pointer along its heading (white for any team),
// and a grey one for each suggestion, all tagged `spawns` (a view's show: / hide: picks them).
void sync_spawn_markers(RealizedScene &realized, std::vector<std::string> *warnings);

// `type: vehicle_preview` (props: `vehicles:`, the catalog's key root, default vehicles;
// `spin:`, degrees a second, default 20): the archetype `<vehicles>.selected` names, in the
// spawn keys' colours, at the node's position. Registered on `app`; idempotent.
void register_vehicle_preview_node(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_VEHICLE_VIEW_H__

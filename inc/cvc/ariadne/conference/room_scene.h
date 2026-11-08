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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_SCENE_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_SCENE_H__

#include <cstdint>
#include <cvc/ariadne/scene.h>
#include <string>
#include <vector>

namespace cvc {
class app;

namespace ariadne {
namespace conference {

// ----------------
// The shared room scene (ARIADNE_CONFERENCE_ROADMAP.md §5g, H1 slice 5)
// ----------------
// A server-authoritative 3D scene every client renders from its OWN camera. It lives in the
// replicated room tree beside the participants, as SCALAR leaves (the holon_sync idiom: they ride
// the existing "conf" interest filter with no transport changes and sidestep data()-object
// replication):
//
//   conf.rooms.<room>.scene.rev                          bumped by every scene write
//   conf.rooms.<room>.scene.env.background.{gradient, top.{r,g,b}, bottom.{r,g,b}}
//   conf.rooms.<room>.scene.env.chrome                   the SceneGraph diagnostic chrome (1/0)
//   conf.rooms.<room>.scene.env.shadows.{enabled, resolution, interval}   real-time shadow maps
//   conf.rooms.<room>.scene.env.lights.<id>.{present, kind, rig, azimuth, elevation, intensity,
//                                            cone, color.{r,g,b}, pos.{x,y,z}, target.{x,y,z}}
//   conf.rooms.<room>.scene.env.lights.<id>.{stage.{center.{x,y,z}, radius}, key.{intensity,
//                                            azimuth, elevation, cone}, fill, back, warmth,
//                                            environment, ambient, wash.{intensity, count,
//                                            height}}   a rig's knobs; empty = the preset's
//   conf.rooms.<room>.scene.objects.<id>.{present, source.{uri, primitive, plane_size,
//                                         box.{x,y,z}, radius}, transform.{pos,rot,scale}.{x,y,z},
//                                         material.{color.{r,g,b}, ambient, diffuse, specular,
//                                         specular_power}, visible}
//   conf.rooms.<room>.scene.objects.<id>.source.fit.{height, up_y}   only for an object that has
//                                         (or once had) a fit: -- a mesh asset stood on the ground
//
// The schema is the .ari scene's own (cvc::ariadne::Scene / SceneNode / SceneLight) restricted to
// the fields a shared room needs, so a client turns the live subtree back into a Scene and hands it
// to the same realizer a document scene uses: realize_scene(sg, read_room_scene(app, room)). The
// TREE, not a document, is the source of truth.
//
// AUTHORITY: the server is the sole writer of scene.* (clients only read it), by the same
// loopback-trusted convention as .pose (H1); enforcement is §5e/H2. Removal is SOFT (present = 0)
// for the reason room_leave is: a hard subtree delete is resurrected by any peer that still holds
// it. Ids must not contain cvc::state::SEPARATOR (std::invalid_argument otherwise).

// conf.rooms.<room-id>.scene
std::string room_scene_path(const std::string &room_id);

// The room's environment: the view background, the diagnostic chrome, and its lights.
struct room_scene_env {
  bool background_gradient = false;
  float background_top[3] = {0.09f, 0.10f, 0.12f};
  float background_bottom[3] = {0.02f, 0.02f, 0.03f};
  bool chrome = true;
  // Real-time shadows from the shadow-casting lights (directional / spot; a fill never casts),
  // re-baked every frame so moving avatars cast moving shadows. `shadow_resolution` is the map
  // size in pixels (bigger = crisper, costlier). Off = the room says nothing about shadows, so a
  // client keeps whatever its document set (the conference document turns them on).
  bool shadows = false;
  int shadow_resolution = 2048;
  // Frames between re-bakes (shadows: { update_interval }); 1 = every frame.
  int shadow_interval = 1;
  // Each light's explicit fields (kind, pos, target, cone, azimuth, elevation, intensity, color),
  // or a rig: preset with the tuning knobs it sets (stage, key, fill, back, warmth, environment,
  // ambient, wash, sun). An unset knob keeps the preset's value, as in a document.
  std::vector<cvc::ariadne::SceneLight> lights;
};

// Write the environment. Lights present before but absent from env.lights are soft-removed.
void write_scene_env(cvc::app &app, const std::string &room_id, const room_scene_env &env);

// Read the environment. Returns false (env untouched) if it was never written.
bool read_scene_env(cvc::app &app, const std::string &room_id, room_scene_env &env);

// Write (add or replace) one object. Replicated fields: id, the source (a URI in source_file, or a
// plane/box/sphere source_primitive with its size), the mesh's fit (has_fit, fit_height, fit_up_y;
// see SceneNode), the transform, the material colour/ambient/diffuse (+ specular when
// has_specular), and visible_default. type is always "geometry". An object without a fit writes
// no fit leaves, unless it is replacing one that had a fit, which it clears.
void write_scene_object(cvc::app &app, const std::string &room_id,
                        const cvc::ariadne::SceneNode &obj);

// Soft-remove one object (present = 0). A no-op if it was never written.
void remove_scene_object(cvc::app &app, const std::string &room_id, const std::string &id);

// The present objects, in id order.
std::vector<cvc::ariadne::SceneNode> read_scene_objects(cvc::app &app, const std::string &room_id);

// The scene revision: 0 when no scene was ever written, then +1 per write. A renderer re-reads and
// re-realizes only when it changes.
std::uint64_t scene_revision(cvc::app &app, const std::string &room_id);

// The whole room scene as a cvc::ariadne::Scene (objects -> nodes, env -> lights / chrome /
// background), ready for cvc::gl::ariadne::realize_scene. Empty (any() == false) if nothing was
// written.
cvc::ariadne::Scene read_room_scene(cvc::app &app, const std::string &room_id);

// The MVP environment the server seeds a room with (§5g "ground + grid + a reference object"): a
// gradient background, the diagnostic chrome off, a 20 m ground plane and a 1 m reference box
// standing on it at the origin. Z is up, as in every .ari scene. No lights and no shadows: those
// are the world's (default_world() adds the stock rig).
room_scene_env default_scene_env();
std::vector<cvc::ariadne::SceneNode> default_scene_objects();

// Seed ROOM-ID with the default scene if it has none yet (scene_revision == 0). Idempotent, so a
// restarted relay does not clobber a scene already in its tree. Returns true if it seeded.
bool seed_default_scene(cvc::app &app, const std::string &room_id);

// The large demo scene (arisrv --scene large): room to spread out, and meshes for LOD to work on.
// A SIDE_M ground plane (centred on the origin, like the default's), the default reference box,
// and the embedded Stanford bunny (source.uri "x.bunny", 69,473 triangles) fitted about 1.7 m tall
// at the centre of every cell of a GRID x GRID lattice over the plane -- 20 m apart at the defaults
// -- except a cell centred on the origin (an odd GRID's middle one), which is the reference box's
// and the avatars' spawn ring's. The bunnies are named "bunny_<i>_<j>" (cell column, row). They
// are meshes, so they have no physics (room_statics keeps only the plane and the box), and every
// one is the same asset, so a client builds one LOD pyramid for them all. Throws
// std::invalid_argument unless SIDE_M is finite and > 0 and GRID >= 1. The environment is
// default_scene_env().
std::vector<cvc::ariadne::SceneNode> large_demo_scene_objects(double side_m = 100.0, int grid = 5);

// Seed ROOM-ID with the large demo scene if it has none yet, exactly as seed_default_scene does
// with the default one (idempotent; true if it seeded).
bool seed_large_demo_scene(cvc::app &app, const std::string &room_id, double side_m = 100.0,
                           int grid = 5);

// ----------------
// Worlds (ARIADNE_WORLDS.md WD1)
// ----------------
// A room's world comes from a world document: arisrv --doc world.ari when the document has a
// scene:, else the stock world --scene names, cvc://worlds/default.ari or large.ari. The relay
// publishes what the room's wire carries of that scene, look included: the light rig and shadows
// are the world's (Q1), so every client draws the room under the rig the relay sends.

// A whole room scene: its environment and its objects.
struct room_world {
  room_scene_env env;
  std::vector<cvc::ariadne::SceneNode> objects;
};

// Write a whole scene in one revision: the environment, every object, a soft removal for each
// object present before but absent from WORLD, then scene.rev, once and last. Writing the same
// world again changes nothing but rev. Throws std::invalid_argument on a bad id.
void write_room_world(cvc::app &app, const std::string &room_id, const room_world &world);

// What the room's wire carries of a document's scene:. The environment: the background (when the
// scene sets one), the chrome (off unless the scene turns it on), the shadows and the lights,
// rigs and their knobs included. The objects: each top-level geometry node with a plane, box or
// sphere source, or a mesh by URI and its fit:, with its transform and plain material. The rest
// is left out, each with a warning in WARNINGS: other node types and sources, children,
// textures, shaders, a fit: up x or flip:, a visible: bind, shadow softness and focus. A body is
// published as a static, as room_statics builds every object (server-dynamic entities are WD3's),
// with a warning for one that is not static, and a physics: block is not simulated (the headless
// world runtime is ARIADNE_WORLDS.md §14's L0). A light or node without an id gets light_<i> /
// node_<i>.
room_world room_world_from_scene(const cvc::ariadne::Scene &scene,
                                 std::vector<std::string> *warnings = nullptr);

// The rig the stock worlds are lit by, as conference_roster.ari declared it before WD1: a
// three_point rig staged over the 20 m floor whose key spot casts the shadows, with fill, back,
// warmth and environment.
cvc::ariadne::SceneLight default_room_rig();

// The C++ twins of cvc://worlds/default.ari and cvc://worlds/large.ari: default_scene_objects()
// and large_demo_scene_objects() under default_scene_env(), lit by default_room_rig() with 2048 px
// shadows re-baked every frame. A relay that cannot find its world file seeds the twin, and a
// test pins each twin equal to its document.
room_world default_world();
room_world large_world(double side_m = 100.0, int grid = 5);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_SCENE_H__

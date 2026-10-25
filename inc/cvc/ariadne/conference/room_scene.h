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
//   conf.rooms.<room>.scene.env.shadows.{enabled, resolution}   real-time shadow maps
//   conf.rooms.<room>.scene.env.lights.<id>.{present, kind, rig, azimuth, elevation, intensity,
//                                            cone, color.{r,g,b}, pos.{x,y,z}, target.{x,y,z}}
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
  // Each light's explicit fields (kind, pos, target, cone, azimuth, elevation, intensity, color)
  // or a bare rig: preset name. Rig tuning knobs (stage/key/fill/...) are not replicated.
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
// standing on it at the origin. Z is up, as in every .ari scene. No lights and no shadows: the
// room's look is the client document's (conference_roster.ari declares its light rig + shadows).
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

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_SCENE_H__

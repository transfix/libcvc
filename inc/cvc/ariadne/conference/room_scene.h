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
#include <cvc/ariadne/section.h> // section_stamp
#include <cvc/world/generated_world.h> // generated_world_params
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
//   conf.rooms.<room>.scene.{epoch, at, rev}             a revisioned section's stamp (section.h):
//                                                        every scene write commits it, rev last
//   conf.rooms.<room>.scene.env.background.{gradient, top.{r,g,b}, bottom.{r,g,b}}
//   conf.rooms.<room>.scene.env.chrome                   the SceneGraph diagnostic chrome (1/0)
//   conf.rooms.<room>.scene.env.shadows.{enabled, resolution, interval}   real-time shadow maps
//   conf.rooms.<room>.scene.env.lights.<id>.{present, epoch, kind, rig, azimuth, elevation,
//                                            intensity, cone, color.{r,g,b}, pos.{x,y,z},
//                                            target.{x,y,z}}
//   conf.rooms.<room>.scene.env.lights.<id>.{stage.{center.{x,y,z}, radius}, key.{intensity,
//                                            azimuth, elevation, cone}, fill, back, warmth,
//                                            environment, ambient, wash.{intensity, count,
//                                            height}}   a rig's knobs; empty = the preset's
//   conf.rooms.<room>.scene.objects.<id>.{present, epoch, source.{uri, primitive, plane_size,
//                                         box.{x,y,z}, radius}, transform.{pos,rot,scale}.{x,y,z},
//                                         material.{color.{r,g,b}, ambient, diffuse, specular,
//                                         specular_power}, visible}
//   conf.rooms.<room>.scene.objects.<id>.source.fit.{height, up_y, epoch}   only for an object
//                                         that has (or once had) a fit: -- a mesh asset stood on
//                                         the ground
//
// The schema is the .ari scene's own (cvc::ariadne::Scene / SceneNode / SceneLight) restricted to
// the fields a shared room needs, so a client turns the live subtree back into a Scene and hands it
// to the same realizer a document scene uses: realize_scene(sg, read_room_scene(app, room)). The
// TREE, not a document, is the source of truth.
//
// AUTHORITY: the server is the sole writer of scene.* (clients only read it), by the same
// loopback-trusted convention as .pose (H1); enforcement is §5e/H2. Removal deletes the entry's
// subtree: one delete, which every client applies, and whose tombstone keeps a peer that still
// holds the entry from bringing it back (deletes replicate since transfix/libcvc#1473). A reader
// still takes only entries with present = 1, which a write sets last. Ids must not contain
// cvc::state::SEPARATOR (std::invalid_argument otherwise).
//
// RUNS (ARIADNE_WORLDS.md §3): a relay's tree starts empty, and a client that stays connected
// across a relay restart keeps what the old relay wrote. So the scene's stamp carries its writer's
// epoch (a restarted relay's revision 1 is not the old one's), and every light, object and fit
// is stamped with the epoch of the write that wrote it. A reader takes only the scene's own run:
// an entry stamped with another epoch is the old relay's, which the new one never removes
// because it never knew its id. An entry with no epoch (a writer from before epochs, or a --set)
// is taken, and so is everything when the scene has none. A plain light's rig knobs are never
// read: a rig writes all of them, so only an earlier light under the same id leaves any.

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

// Write the environment. Lights present before but absent from env.lights are removed.
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

// Remove one object from the room tree: one delete, which every client applies, and the revision
// bump. A no-op if it was never written.
void remove_scene_object(cvc::app &app, const std::string &room_id, const std::string &id);

// The present objects, in id order.
std::vector<cvc::ariadne::SceneNode> read_scene_objects(cvc::app &app, const std::string &room_id);

// The scene revision: 0 when no scene was ever written, then +1 per write, from 1 again in a new
// epoch (a restarted relay's tree starts empty).
std::uint64_t scene_revision(cvc::app &app, const std::string &room_id);

// The scene's stamp: its writer's epoch and its revision (`at` is always 0: the scene applies on
// arrival). A renderer or a holon re-reads the scene only when the stamp is not the same() as the
// one it last took, so a client that outlived a relay takes the new relay's revision 1 afresh.
section_stamp scene_stamp(cvc::app &app, const std::string &room_id);

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

// Write a whole scene in one revision: the environment, every object, a removal for each object
// present before but absent from WORLD, then the stamp, once and last, under EPOCH (0: this
// process's). Writing the same world again changes nothing but rev. Returns the stamp. Throws
// std::invalid_argument on a bad id.
section_stamp write_room_world(cvc::app &app, const std::string &room_id, const room_world &world,
                               std::uint64_t epoch = 0);

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

// The world document the relay runs (ARIADNE_WORLDS.md section 18.5, OW5c): the URI it was named
// by (a stock world's cvc://worlds/<name>.ari, a --doc as given) and the SHA-256 of its bytes, in
// lowercase hex. The room scene carries only what every client can draw; a client that holds the
// same document (its own bundle, section 12) builds the same world from it instead
// (world_replica), meshes, terrain and a generated world's buildings included. A world from a
// C++ twin has none.
//
//   conf.rooms.<room>.world_source.{uri, sha256}, stamped {epoch, at, rev} (section.h)
struct room_world_source {
  std::string uri, sha256;
  bool operator==(const room_world_source &) const = default;
};
std::string room_world_source_path(const std::string &room_id);
// Write it in one revision (EPOCH as section_writer::commit takes it). Returns the stamp.
section_stamp write_room_world_source(cvc::app &app, const std::string &room_id,
                                      const room_world_source &source, std::uint64_t epoch = 0);
// Read it; false (OUT untouched) if it was never written.
bool read_room_world_source(cvc::app &app, const std::string &room_id, room_world_source &out);
section_stamp room_world_source_stamp(cvc::app &app, const std::string &room_id);
// The SHA-256 of the file at PATH, in lowercase hex; "" when it cannot be read.
std::string file_sha256(const std::string &path);

// What the relay's world document has on now (ARIADNE_WORLDS.md section 18.5, OW5f): the scene's
// world that is on (a scene's worlds are on one at a time: SceneWorld::world_on, "" for a scene
// without worlds), and the params each generated_world node of it (of the scene, without worlds)
// made its world from (a
// Regenerate, a new seed or preset, a changed count). A client building the same document
// (world_replica) turns on the same world and makes the same generated worlds from these, so
// its statics stay the relay's after a switch or a Regenerate. The relay writes it whenever it
// changes (publish_world_on).
//
// And each water node's sea as its last keyframe (OW5q; SceneWaterKeyframe): the winds of the old
// and new keyframes (m/s, rad, m), the crossfade (clock seconds), the keyframe's count and the
// clock tick its field went in after. A sea lags its wind, and a storm's front moves the wind on
// the clock, so a client that came later cannot grow the relay's sea from the document; its
// replica makes the relay's from these (WaterRuntime::follow_sea). The doubles are written to 17
// digits, so they come back to the bit.
//
// And each weather node's storms as the relay has them now (OW5q): its events (weather.events, a
// live key on the relay) and its storm-wind peak with the tick it went in after, from which a
// client's replica makes the same air (the storms' gust fronts) and so the same sea.
//
//   conf.rooms.<room>.world_on.{world, generated, g<i>.{node, seed, half, trees, rocks, buildings,
//     tree_gen, amp, water_level, preset, ontology, cell, max_terrain_n, raster_n, sliver,
//     terrain, city, terrain_file, edge}, seas, s<i>.{node, keyframe, from_tick, from_speed,
//     from_dir, from_fetch, to_speed, to_dir, to_fetch, t0, t1}, weathers, w<i>.{node, events,
//     gust, gust_from}}, stamped {epoch, at, rev} (section.h)
struct room_generated_world {
  std::string node;
  cvc::world::generated_world_params params;
};
struct room_water_sea {
  std::string node;
  std::uint64_t keyframe = 0, from_tick = 0;
  double from_speed = 0.0, from_dir = 0.0, from_fetch = 0.0;
  double to_speed = 0.0, to_dir = 0.0, to_fetch = 0.0;
  double t0 = 0.0, t1 = 0.0;
  bool operator==(const room_water_sea &o) const;
};
struct room_weather {
  std::string node;
  std::string events; // the storms' events, cvc::world::format_weather_events' text
  double gust = 0.0;  // the storm-wind peak, m/s (WeatherFriction::gust_front)
  std::uint64_t gust_from = 0;
  bool operator==(const room_weather &o) const;
};
struct room_world_on {
  std::string world;
  std::vector<room_generated_world> generated; // in the world's node order
  std::vector<room_water_sea> seas;            // the water nodes', in the document's order
  std::vector<room_weather> weathers;          // the weather nodes', in the document's order
  bool operator==(const room_world_on &o) const;
};
std::string room_world_on_path(const std::string &room_id);
// Write it in one revision (EPOCH as section_writer::commit takes it). Returns the stamp.
section_stamp write_room_world_on(cvc::app &app, const std::string &room_id,
                                  const room_world_on &on, std::uint64_t epoch = 0);
// Read it; false (OUT untouched) if it was never written.
bool read_room_world_on(cvc::app &app, const std::string &room_id, room_world_on &out);
section_stamp room_world_on_stamp(cvc::app &app, const std::string &room_id);

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

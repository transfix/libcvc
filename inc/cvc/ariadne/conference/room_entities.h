/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A room's entities (ARIADNE_WORLDS.md §4, WD3): what moves in the room's world besides its
// participants, which today means the vehicles its relay's document runs (ServerDocument, its
// SceneWorld's WorldVehicles). Two halves:
//
// - Who they are: the room tree's entities section, conf.rooms.<room>.entities (a revisioned
//   section, section.h), one subtree per entity under entity_key(id):
//
//     conf.rooms.<room>.entities.e<id>.{ present, name, archetype, plant, controller, team,
//                                        color, color2, size, wheels, base_z, fit_length,
//                                        look.kind, look.path, look.scale, look.yaw, look.z,
//                                        look.fit }
//     conf.rooms.<room>.entities.{ epoch, at, rev }
//
//   written when they change: a spawn, a removal, a new world (WorldVehicles::revision). An entity
//   that went is removed from the tree (a delete), as the room's objects are; a reader takes only
//   the entities with present 1, which a write sets last.
// - Where they are: the pose stream (pose_stream.h), beside the participants, under entity_pid(id)
//   ("~e<id>": no participant's pid, a state name, starts with '~'). A client plays them out with a
//   holon of their own (holon_sync's entity_pump), the participants' playout without bodies.
#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_ENTITIES_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_ENTITIES_H__

#include <array>
#include <cstdint>
#include <cvc/ariadne/pose_stream.h> // pose_subject
#include <cvc/ariadne/section.h>
#include <cvc/ariadne/vehicle_drive.h>      // dynamic_drive
#include <cvc/ariadne/vehicle_kinematics.h> // kinematic_drive
#include <cvc/ariadne/world_rules.h> // vehicle_look
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class WorldVehicles;
namespace conference {

// An entity as a client draws it.
struct room_entity {
  std::uint64_t id = 0;
  std::string name, archetype, plant, controller;
  // The participant driving it (its pid), or "" for its controller (section 18.4).
  std::string driver;
  int team = 0;
  std::array<float, 3> color{0.85f, 0.70f, 0.20f};
  std::array<float, 3> color2{0.12f, 0.12f, 0.14f};
  // Its chassis box, full extents in metres (x forward, y left, z up), as its plant has it.
  std::array<double, 3> size{4.5, 2.0, 1.0};
  // Its wheels: the wheelbase, the track and a wheel's radius (m); a boat's are 0.
  std::array<double, 3> wheels{3.3, 1.82, 0.45};
  // The ground (or the water) under its chassis's centre, relative to that centre (m, negative):
  // where a fitted model's base goes. A `pose` entity's is its stand height below it
  // (stand_height), wherever its poses put it. A boat's (a floating plant's) is its keel: its
  // hull's bottom in its hull body's frame, whatever depth of water it is on.
  double base_z = -1.0;
  // The length a model look is fitted to (m): a boat's overall length (dyn_boat_desc::
  // length_overall), longer than its hull's box; 0 for the chassis's, size[0].
  double fit_length = 0.0;
  // Its archetype's look; a file's or an asset's path is its file name alone (the relay's own path
  // means nothing to a client, which finds the file its own way).
  vehicle_look look;
  // Whether a participant may get in (its archetype's drivable:), from within `reach` metres of its
  // chassis box, and the control scheme it is driven with (car, boat; vehicle_controls).
  bool drivable = false;
  double reach = 3.0;
  std::string controls;
  // The participant whose body it is (an avatar archetype that is a vehicle, holon::set_archetype;
  // "" none): no one else gets in it.
  std::string avatar;
  // On the kinematic plant: what a client needs to run it as the relay's world does, so the one
  // it drives is predicted (ARIADNE_WORLDS.md section 18.5, OW5b; WorldVehicles::kinematics). Not
  // valid() on another plant.
  kinematic_drive drive;
  // On the dynamic or dynamic4w plant: the same, for its model (OW5d, OW5e;
  // WorldVehicles::dynamics). Not valid() on another plant.
  dynamic_drive dynamics;
  bool operator==(const room_entity &) const = default;
};

// "e<id>", its subtree's name; "~e<id>", its pid on the pose stream; and back (false for a pid
// that is not an entity's).
std::string entity_key(std::uint64_t id);
std::string entity_pid(std::uint64_t id);
bool parse_entity_pid(const std::string &pid, std::uint64_t &id);
bool is_entity_pid(const std::string &pid);

// conf.rooms.<room>.entities.
std::string room_entities_path(const std::string &room);

// Write `entities` as the room's entities section: each one's fields, a removal (section_writer::
// remove) for every entity the section has that `entities` does not, then the revision
// (section_writer::commit, `epoch` as it takes it). Returns the stamp.
section_stamp write_room_entities(cvc::app &app, const std::string &room,
                                  const std::vector<room_entity> &entities,
                                  std::uint64_t epoch = 0);
// The present entities, by id; and the section's stamp, to tell when to read them again.
std::vector<room_entity> read_room_entities(cvc::app &app, const std::string &room);
section_stamp read_room_entities_stamp(cvc::app &app, const std::string &room);

// The entities `vehicles` runs, as the room carries them: the placed ones, in id order.
std::vector<room_entity> room_entities_of(const WorldVehicles &vehicles);
// Their poses now, as the pose stream's subjects (pid entity_pid(id)): the chassis's centre, its
// heading as a rotation about +z, and its velocity.
std::vector<pose_subject> entity_pose_subjects(const WorldVehicles &vehicles);

// The relay's half, one per room: each step(), the section rewritten when the vehicles changed,
// and their poses for the pose stream (room_pose_relay::step's `entities`).
class room_entity_relay {
public:
  // `vehicles` null (no document, or a world without vehicles): the section empties once.
  const std::vector<pose_subject> &step(cvc::app &app, const std::string &room,
                                        const WorldVehicles *vehicles);
  // The section's writes so far (each a whole rewrite).
  std::uint64_t writes() const { return _writes; }

private:
  const WorldVehicles *_vehicles = nullptr;
  std::uint64_t _rev = 0;
  bool _written = false;
  std::uint64_t _writes = 0;
  std::vector<pose_subject> _subjects;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_ENTITIES_H__

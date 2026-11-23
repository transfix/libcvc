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

#ifndef __CVC_ARIADNE_HOLON_SYNC_H__
#define __CVC_ARIADNE_HOLON_SYNC_H__

#include <cstdint>
#include <cvc/ariadne/avatar_physics.h> // avatar_static
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/conference/room_entities.h> // room_entity
#include <cvc/ariadne/conference/room_scene.h>    // room_world_on
#include <cvc/ariadne/holon.h>                    // holon_config
#include <cvc/ariadne/pose_stream.h>              // pose_stream_decoder
#include <cvc/ariadne/section.h>                  // section_stamp
#include <cvc/core/namespace.h>
#include <cvc/physics/body.h>
#include <cvc/state/pose_snapshot.h>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class app;
class height_field;
namespace nav {
struct world_grid;
}

namespace ariadne {

class holon;
class SceneWorld;
class world_replica;

// ----------------
// cvc::ariadne — holon <-> state-tree sync over SCALAR keys (H1 slice 4)
// ----------------
// Connects a cvc::ariadne::holon to the replicated conference room tree over plain
// SCALAR leaf keys, under the participant subtree the conference already uses
// (conf.rooms.<room>.participants.<pid>, via room_participant_path):
//
//   <participant>.pose.pos.{x,y,z}          authoritative, SERVER-written
//   <participant>.pose.orient.{w,x,y,z}
//   <participant>.pose.vel.lin.{x,y,z}
//   <participant>.pose.vel.ang.{x,y,z}
//   <participant>.pose.server_tick          the tick the pose is valid at
//   <participant>.intent.move.{x,y,z}       proposed, CLIENT-written
//   <participant>.intent.yaw                facing (radians about +Z), CLIENT-written
//   <participant>.intent.tick               the client tick it applies from (0 = at once)
//   <participant>.intent.{steer,throttle,brake,jump}  the drive axes and the walker's jump
//   <participant>.drive                     "<seq>:<entity>", a get-in (0: get out), CLIENT-written
//   <participant>.driving                   the entity it drives (0: none), SERVER-written
//   <participant>.drive_status              "<seq> ok" or "<seq> refused: <why>", SERVER-written
//
// Scalar leaves (not a typed body_state data() object) are used on purpose: they
// already journal and replicate through the existing "conf" interest filter with
// zero transport changes, and they sidestep the data()-object replication work
// (DISTRIBUTED_STATE §52). Each double is written round-trip-safe (17 significant
// digits), so a reader recovers it to within far less than the reconcile
// tolerance.
//
// AUTHORITY (the proposed/authoritative path pair): a client writes only its own
// .intent.*; the server reads every participant's .intent, integrates it into the
// holon, and is the SOLE writer of .pose + .server_tick. A client reads .pose
// read-only and reconciles. Writes go through a C++ host (these helpers / a verb),
// never a DSL (state-set), because the document chroot ui.docs.* is excluded from
// replication -- the same reason room-join is a C++ verb. A relay with bound
// writes (room_write_gate, arisrv's default) enforces the split: it binds each
// connection to the pid of its Hello, takes .intent.* only from that pid's own
// connection, and refuses any client's write to .pose. Nothing authenticates a pid
// yet (§5f), so a client can still claim one nobody holds.

// Write a participant's AUTHORITATIVE pose (server side): the body_state's pose +
// velocity leaves and the server_tick, each replicated. With HEARTBEAT_TICKS > 0 only
// the leaves whose value changed are written, and the server_tick only when something
// changed or HEARTBEAT_TICKS have passed since it was last written: an avatar standing
// still costs one replicated write per heartbeat, not fourteen per relay tick. 0 is the
// forced rewrite: every leaf every call, unchanged ones too (what re-hydrates a peer
// that missed them). Each written leaf journals ONE mutation: a changed value alone, a
// new (or still empty) leaf touched first so the state adapter observes it before its
// first value, and a forced unchanged leaf a touch.
void write_pose(cvc::app &app, const std::string &room, const std::string &pid,
                const cvc::physics::body_state &state, std::uint64_t server_tick,
                std::uint64_t heartbeat_ticks = 0);

// Read a participant's authoritative pose. Returns false (leaving out_* untouched)
// if the pose subtree is absent or malformed; true with out_state / out_server_tick
// filled otherwise.
bool read_pose(cvc::app &app, const std::string &room, const std::string &pid,
               cvc::physics::body_state &out_state, std::uint64_t &out_server_tick);

// Write a participant's PROPOSED intent (client side -- its OWN avatar only). TICK is
// the client tick its prediction applies it from (holon::queue_intent); 0 = untimed,
// the server applies it at once.
void write_intent(cvc::app &app, const std::string &room, const std::string &pid,
                  const avatar_intent &intent, std::uint64_t tick = 0);

// Read a participant's proposed intent (server side). Returns false if absent. When
// OUT_TICK is given it receives the intent's stamp (0 = untimed).
bool read_intent(cvc::app &app, const std::string &room, const std::string &pid,
                 avatar_intent &out_intent, std::uint64_t *out_tick = nullptr);

// CLIENT: the intent the tree holds for our own `pid`, unless it is older than the one we proposed
// last (stamped `proposed_tick`, what propose_intent returned; 0 = none yet). A stamp below it is
// an ECHO of an earlier proposal -- a relay's re-broadcast can carry our previous intent back after
// the newer one left -- and predicting with it would walk the avatar on input the server no longer
// has: false, and `out` is untouched. A stamp of 0 (an intent the document wrote,
// room-set-intent) is always taken.
bool read_current_intent(cvc::app &app, const std::string &room, const std::string &pid,
                         std::uint64_t proposed_tick, avatar_intent &out);

// GETTING IN AND OUT (ARIADNE_WORLDS.md section 18.6). CLIENT: ask to drive world entity ENTITY
// (0: get out) as our own PID. Each ask has the next seq (one more than the last in the tree), so
// asking again after a refusal is a new ask; returns it. The server answers in drive_status and,
// when it lets us in or out, in driving.
std::uint64_t request_drive(cvc::app &app, const std::string &room, const std::string &pid,
                            std::uint64_t entity);
// The entity PID drives as the server says (0: none).
std::uint64_t read_driving(cvc::app &app, const std::string &room, const std::string &pid);
// The server's answer to PID's last ask: its seq (0: none yet), whether it was let in or out, and
// why not.
struct drive_status {
  std::uint64_t seq = 0;
  bool ok = false;
  std::string why;
};
drive_status read_drive_status(cvc::app &app, const std::string &room, const std::string &pid);
// SERVER: answer PID's ask, if it has a new one (holon::get_in), and publish what it drives.
// server_pump does this for every participant.
void serve_drive(holon &h, cvc::app &app, const std::string &room, const std::string &pid);

// AVATARS THAT ARE VEHICLES (ARIADNE_WORLDS.md section 18.6, OW6c). CLIENT: ask for avatar
// archetype ARCHETYPE (the room's avatars.archetypes, by id; "" the first) as our own PID, beside
// our avatar's look (participants.<pid>.avatar.archetype). The server answers in archetype_status
// and, for one that is a vehicle, seats us in it (driving).
void request_archetype(cvc::app &app, const std::string &room, const std::string &pid,
                       const std::string &archetype);
// What PID asked for ("" for nothing yet, or the first).
std::string read_requested_archetype(cvc::app &app, const std::string &room,
                                     const std::string &pid);
// The server's answer: the archetype it ruled on ("" the first), whether PID has it now, and why
// not; `answered` false until the server has ruled on any.
struct archetype_status {
  bool answered = false;
  std::string archetype;
  bool ok = false;
  std::string why;
};
archetype_status read_archetype_status(cvc::app &app, const std::string &room,
                                       const std::string &pid);
// SERVER: give PID the archetype it asks for (holon::set_archetype; the first until it asks), and
// say so in archetype_status when it asked, was refused, or became a vehicle. server_pump does
// this for every participant, before serve_drive.
void serve_archetype(holon &h, cvc::app &app, const std::string &room, const std::string &pid);

// CLIENT: the vehicle a participant standing at FEET, facing FACING (rad from +x), would get into:
// the nearest drivable entity the playout shows whose chassis box is within its reach, one ahead
// of it before one behind. 0 for none; OUT_GAP gets its distance (m, feet to box, in plan).
class entity_playout;
std::uint64_t drive_target(const entity_playout &entities, const cvc::physics::vec3 &feet,
                           double facing, double *out_gap = nullptr);

// The holon configuration every conference room uses, on the relay and in every client (they must
// agree: the client predicts with the server's physics): rigid-body avatars in a Jolt world, under
// the room's rules -- the body, controller, veto, energy budget and gravity the relay publishes as
// sections (room_rules.h), which both pumps take into the holon from their activation ticks.
holon_config room_holon_config();
// ... at HZ ticks a second (a world's rate, holon_config_at; 0: the default 120 Hz). The relay
// builds its holon at the rate its world runs at.
holon_config room_holon_config(double hz);
// ... at ROOM's rate: the relay's clock section (room_rules.h's read_room_clock), the default
// 120 Hz while there is none. A client builds its holon from this.
holon_config room_holon_config(cvc::app &app, const std::string &room);

// Whether H runs at ROOM's rate (120 Hz while the room has no clock section). A host whose holon
// does not -- built before the room's tree arrived, or the relay restarted at another rate --
// builds it again from room_holon_config(app, room): a rate changes what every tick means, so a
// holon cannot switch to it in place.
bool on_room_clock(const holon &h, cvc::app &app, const std::string &room);

// The room scene's objects as physics statics (holon::set_statics): the ground plane as a thin
// slab under z = its height, boxes as boxes, spheres as spheres, each at its transform (Euler
// degrees, applied X then Y then Z). Objects with no physical shape (a mesh file) are skipped.
std::vector<avatar_static> room_statics(cvc::app &app, const std::string &room);

// Drive one authoritative frame of a SERVER holon against the replicated room
// tree: for every present participant (room_roster), add it to the holon if new,
// read its proposed .intent, feed it to the holon (a stamped one queued for its tick,
// holon::queue_intent; an untimed one at once); advance the holon one frame; then
// write every participant's authoritative .pose + .server_tick back to the tree
// (which replicates to the clients) -- what changed, plus a heartbeat every
// POSE_HEARTBEAT_S while nothing does (write_pose). The caller supplies the server holon
// and calls this once per relay tick. A participant who left (room_leave) is dropped
// from the holon, so a rejoin respawns on the spawn ring. wall_dt
// is forwarded to holon::advance: the default (-1) measures real elapsed wall time
// (what the relay wants); a test injects a fixed dt for a deterministic step count.
// WRITE_POSES false skips the pose write-back: a relay that streams poses to its clients
// (room_pose_relay) writes the tree's copy only now and then, for late joiners and the log.
// Clients that read their poses from the tree keep their clocks by the heartbeat, so a second
// suits them; a relay whose pose stream carries its own heartbeat can stretch the tree's, since a
// client takes no stale tick as a clock reading (holon::apply_authoritative). Keep it under
// client_pump's 10 s leftover window: a returning peer's leftover pose is told from a new one by
// its tick being at most that far below the newest one seen before it left.
// Each participant's reserve (room_avatar.h) is written either way: no snapshot carries it.
void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt = -1.0,
                 bool write_poses = true, double pose_heartbeat_s = 1.0);

// Drive one frame of a CLIENT holon against the replicated room tree (H1 slice 6), the mirror of
// server_pump: register every present participant (room_roster) -- SELF_PID as the local,
// predicted avatar, everyone else as an interpolated peer, each starting at its replicated pose
// when one exists, else pose-less until its first sample (holon::pose_known false: no peer body,
// nothing to draw) -- drop the ones that left, apply each replicated authoritative .pose (the
// holon ignores a sample it has already seen, so re-reading an unchanged pose every frame is
// harmless), then advance the holon one frame. Read the result with h.display_pose(pid). wall_dt
// as in server_pump. While the relay says SELF_PID drives a vehicle (driving, section 18.6), its
// pose is the vehicle's: with ENTITIES, and a vehicle whose room entity says how it runs (the
// kinematic plant's room_entity::drive), the holon predicts it (holon::predict_vehicle; section
// 18.5, OW5b); otherwise it is played out from the relay's samples as a peer is. Out again, the
// walker is predicted. With physics, both pumps (re)load the holon's
// statics from the room scene whenever its revision changes, and both take each of the room's rules
// sections (room_rules.h) whenever its stamp moves (holon::set_physics and the rest).
//
// With ENTITIES (the room's entities, pumped this frame first) and physics, every entity it shows
// that SELF_PID neither drives nor is (section 18.5, OW5) is held in the holon's world as a
// kinematic box where it is shown (holon::hold_entity), so the avatar we predict bumps into the
// room's vehicles and is pushed by them as on the relay; one no longer shown is released.
//
// With WORLD (OW5c), the statics are the world document's, as the relay's world has them, once
// WORLD holds that document (world_replica::ready): its meshes, terrain and a generated world's
// buildings, not only the room scene's plane, boxes and spheres. Until then, and without it, they
// are the room scene's (room_statics). client_pump pumps WORLD itself.
void client_pump(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                 double wall_dt = -1.0, const entity_playout *entities = nullptr,
                 world_replica *world = nullptr);

// A pose snapshot from the relay (room_pose_relay; transports' set_on_pose_snapshot): decode it
// with this connection's DECODER and give each participant the holon already has its sample, as
// a replicated pose would (holon::apply_authoritative: the local avatar reconciles, a peer joins
// its playout). One the holon does not have yet (the roster has not replicated it) is skipped;
// client_pump registers it, and its next entry is its first sample. Call it on the frame's thread,
// before client_pump. Returns the samples applied; 0 for a snapshot out of step (decoder::gaps).
class entity_playout;
//
// The room's entities ride the same snapshots under their entity pids (room_entities.h): with
// ENTITIES, each of their samples goes to it; without, they are skipped like any pid the holon
// does not have. They count among the samples applied.
std::size_t apply_pose_snapshot(holon &h, pose_stream_decoder &decoder,
                                const cvc::pose_snapshot &snap, entity_playout *entities = nullptr);
// The same for samples decoded elsewhere (the fast lane decodes, and acknowledges, on its own
// thread: room_lane_session::set_on_pose_samples), at the snapshot's SERVER_TICK.
std::size_t apply_pose_samples(holon &h, const std::vector<pose_sample> &samples,
                               std::uint64_t server_tick, entity_playout *entities = nullptr);

// CLIENT: the room's entities (room_entities.h; ARIADNE_WORLDS.md WD3), played out from the pose
// stream by a holon of their own: a client holon without physics, so each entity is shown as a
// peer avatar is (behind its newest sample by the playout's delay, eased between samples, carried
// on along its velocity past a late one) with no body in any world. Who they are comes from the
// room's entities section, read again when its revision moves.
class entity_playout {
public:
  explicit entity_playout(holon_config cfg = holon_config{});
  ~entity_playout();
  entity_playout(const entity_playout &) = delete;
  entity_playout &operator=(const entity_playout &) = delete;

  // A sample of entity pid `pid` at SERVER_TICK (apply_pose_samples gives them): the entity's first
  // one adds it to the playout. False for a pid that is not an entity's.
  bool apply(const std::string &pid, const cvc::physics::body_state &state,
             std::uint64_t server_tick);
  // One frame: the room's entities section (when its revision moved), an entity it no longer lists
  // dropped from the playout, then the playout advanced (`wall_dt` as client_pump's).
  void pump(cvc::app &app, const std::string &room, double wall_dt = -1.0);

  // The entities the section lists as present, by id; and the one with `id` (null for none).
  const std::vector<conference::room_entity> &entities() const { return _entities; }
  const conference::room_entity *find(std::uint64_t id) const;
  // The ids being played out (a sample has arrived), in id order.
  std::vector<std::uint64_t> shown() const;
  // Where entity `id` is shown this frame; false before its first sample. The one predicted (below)
  // is shown where it is predicted.
  bool pose(std::uint64_t id, cvc::physics::pose &out) const;
  // Entity `id` -- the vehicle the client drives, which its holon predicts (holon::predict_vehicle;
  // ARIADNE_WORLDS.md section 18.5, OW5b) -- shown at AT this frame instead of where its samples
  // put it; 0 for none. A host sets it each frame from the holon's display_pose.
  void show_predicted(std::uint64_t id, const cvc::physics::pose &at);
  // ... and its velocity, its newest sample's.
  bool state(std::uint64_t id, cvc::physics::body_state &out) const;
  // Bumps whenever the section's entities or the ones shown change (a view rebuilds on it).
  std::uint64_t revision() const { return _rev; }
  holon &playout() { return *_holon; }

private:
  std::unique_ptr<holon> _holon;
  section_stamp _stamp;
  std::vector<conference::room_entity> _entities;
  std::uint64_t _rev = 0;
  std::uint64_t _predicted = 0; // show_predicted's entity (0: none), and where
  cvc::physics::pose _predicted_at;
};

// SERVER: what the relay's world document has on now (room_world_on, ARIADNE_WORLDS.md section
// 18.5, OW5f): its world on and the params each of that world's generated_world nodes made its
// world from (every generated_world node's, in a scene without worlds).
conference::room_world_on world_on_of(const SceneWorld &world);
// Write it to the room when it is not what the room has (a switch, a Regenerate, the first
// call). True when it wrote. arisrv calls it every loop.
bool publish_world_on(cvc::app &app, const std::string &room, const SceneWorld &world);

// CLIENT: the relay's world, built from the same document (ARIADNE_WORLDS.md section 18.5, OW5c).
// The room names the document the relay runs (room_world_source: its URI and the SHA-256 of its
// bytes). When this client resolves the same URI to a file with the same hash -- a stock world it
// ships with, or a pack it was pointed at (section 12) -- it loads it and builds its scene's world
// as the relay does (SceneWorld::from_scene), and the holon's statics are that world's static
// bodies (a mesh's and a terrain's shapes) and the colliders its nodes make at run time (a
// generated world's terrain and buildings, as each becomes ready). Its vehicles and moving bodies
// are the relay's to run: the room's entities, not statics. A document it cannot resolve, or one
// whose hash differs, leaves it not ready, and the statics stay the room scene's. It follows what
// the relay has on (room_world_on, OW5f): the same world turned on, the same generated worlds
// made, and the statics again from them.
class world_replica {
public:
  world_replica();
  ~world_replica();
  world_replica(const world_replica &) = delete;
  world_replica &operator=(const world_replica &) = delete;

  // One frame: the room's world source (when its stamp moved: resolve, check, load and build),
  // what the relay has on (when its stamp moved, or the world was built again: the world turned
  // on, the generated worlds asked for), then the world's sync (a generated world's colliders as
  // they become ready), and the statics again when the world on or any node's colliders changed.
  void pump(cvc::app &app, const std::string &room);
  // Whether it holds the relay's document; its statics, and their stamp (its own epoch, a
  // revision more each time they change: holon::set_statics takes it).
  bool ready() const { return _ready; }
  const std::vector<avatar_static> &statics() const { return _statics; }
  const section_stamp &stamp() const { return _stamp; }
  // The water with them (OW5j), which a boat's model floats on: the fluid fields its world has
  // (a water node's), else the still water the relay adds for its boats over the world's grid
  // (boat_still_water). It changes with the statics, under their stamp.
  const std::vector<cvc::physics::field> &water() const { return _water; }
  // What it did with the source last: what it built, or why not.
  const std::string &status() const { return _status; }
  // The world it built (null when not ready).
  const SceneWorld *world() const { return _world.get(); }
  // The ground the relay's vehicles drive on, as this world has it (OW5g: SceneWorld::
  // vehicle_ground, read by vehicle_ground_height), and its revision, a new one each time its
  // grid or terrain changes (a switch, a Regenerate, another document); 0 and empty before it has
  // either. client_pump gives it to the holon (holon::set_vehicle_ground).
  std::function<double(double, double)> ground() const;
  std::uint64_t ground_revision() const { return _ground_rev; }

private:
  void build_statics(cvc::app &app);
  void follow(cvc::app &app, const std::string &room);
  section_stamp _source_stamp;
  section_stamp _on_stamp; // the room_world_on followed
  std::string _built_on;   // the world on when the statics were built
  void refresh_ground();
  std::shared_ptr<const cvc::nav::world_grid> _ground_grid;
  std::shared_ptr<const cvc::height_field> _ground_terrain;
  std::uint64_t _ground_rev = 0;
  std::unique_ptr<SceneWorld> _world;
  std::vector<std::string> _nodes; // the document's node ids (a node's colliders by id)
  bool _ready = false;
  std::map<std::string, std::uint64_t> _taken; // node -> the colliders' revision taken
  std::vector<avatar_static> _statics;
  std::vector<cvc::physics::field> _water;
  section_stamp _stamp;
  std::string _status;
};

// Propose the local avatar's intent (client side): the client holon predicts with it from its next
// tick, and it is written to <self>.intent stamped with that tick, for the server to validate and
// apply at the same tick (see holon::set_lead_ticks). Call it when the intent CHANGES (each call is
// a replicated write), not every frame. Returns the stamp (for read_current_intent).
std::uint64_t propose_intent(holon &h, cvc::app &app, const std::string &room,
                             const std::string &self_pid, const avatar_intent &intent);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_HOLON_SYNC_H__

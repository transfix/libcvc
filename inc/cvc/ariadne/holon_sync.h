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
#include <cvc/ariadne/holon.h>       // holon_config
#include <cvc/ariadne/pose_stream.h> // pose_stream_decoder
#include <cvc/core/namespace.h>
#include <cvc/physics/body.h>
#include <cvc/state/pose_snapshot.h>
#include <string>
#include <vector>

namespace cvc {

class app;

namespace ariadne {

class holon;

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

// The holon configuration every conference room uses, on the relay and in every client (they must
// agree: the client predicts with the server's physics): rigid-body avatars in a Jolt world.
holon_config room_holon_config();

// The room scene's objects as physics statics (holon::set_statics): the ground plane as a thin
// slab under z = its height, boxes as boxes, spheres as spheres, each at its transform (Euler
// degrees, applied X then Y then Z). Objects with no physical shape (a mesh file) are skipped.
std::vector<avatar_static> room_statics(cvc::app &app, const std::string &room);

// Drive one authoritative frame of a SERVER holon against the replicated room
// tree: for every present participant (room_roster), add it to the holon if new,
// read its proposed .intent, feed it to the holon (a stamped one queued for its tick,
// holon::queue_intent; an untimed one at once); advance the holon one frame; then
// write every participant's authoritative .pose + .server_tick back to the tree
// (which replicates to the clients) -- what changed, plus a heartbeat of about a
// second while nothing does (write_pose). The caller supplies the server holon
// and calls this once per relay tick. A participant who left (room_leave) is dropped
// from the holon, so a rejoin respawns on the spawn ring. wall_dt
// is forwarded to holon::advance: the default (-1) measures real elapsed wall time
// (what the relay wants); a test injects a fixed dt for a deterministic step count.
// WRITE_POSES false skips the pose write-back: a relay that streams poses to its clients
// (room_pose_relay) writes the tree's copy only now and then, for late joiners and the log.
void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt = -1.0,
                 bool write_poses = true);

// Drive one frame of a CLIENT holon against the replicated room tree (H1 slice 6), the mirror of
// server_pump: register every present participant (room_roster) -- SELF_PID as the local,
// predicted avatar, everyone else as an interpolated peer, each starting at its replicated pose
// when one exists, else pose-less until its first sample (holon::pose_known false: no peer body,
// nothing to draw) -- drop the ones that left, apply each replicated authoritative .pose (the
// holon ignores a sample it has already seen, so re-reading an unchanged pose every frame is
// harmless), then advance the holon one frame. Read the result with h.display_pose(pid). wall_dt
// as in server_pump. With physics, both pumps (re)load the holon's statics from the room scene
// whenever its revision changes.
void client_pump(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                 double wall_dt = -1.0);

// A pose snapshot from the relay (room_pose_relay; transports' set_on_pose_snapshot): decode it
// with this connection's DECODER and give each participant the holon already has its sample, as
// a replicated pose would (holon::apply_authoritative: the local avatar reconciles, a peer joins
// its playout). One the holon does not have yet (the roster has not replicated it) is skipped;
// client_pump registers it, and its next entry is its first sample. Call it on the frame's thread,
// before client_pump. Returns the samples applied; 0 for a snapshot out of step (decoder::gaps).
std::size_t apply_pose_snapshot(holon &h, pose_stream_decoder &decoder,
                                const cvc::pose_snapshot &snap);

// Propose the local avatar's intent (client side): the client holon predicts with it from its next
// tick, and it is written to <self>.intent stamped with that tick, for the server to validate and
// apply at the same tick (see holon::set_lead_ticks). Call it when the intent CHANGES (each call is
// a replicated write), not every frame. Returns the stamp (for read_current_intent).
std::uint64_t propose_intent(holon &h, cvc::app &app, const std::string &room,
                             const std::string &self_pid, const avatar_intent &intent);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_HOLON_SYNC_H__

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
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/core/namespace.h>
#include <cvc/physics/body.h>
#include <string>

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
// replication -- the same reason room-join is a C++ verb. The connection<->pid
// admission binding that would make "sole writer" cryptographically enforced is
// the §5e/mesh increment (H2); H1 is loopback-trusted sole-writer-by-convention.

// Write a participant's AUTHORITATIVE pose (server side): the body_state's pose +
// velocity leaves and the server_tick, each replicated (touch-before-value).
void write_pose(cvc::app &app, const std::string &room, const std::string &pid,
                const cvc::physics::body_state &state, std::uint64_t server_tick);

// Read a participant's authoritative pose. Returns false (leaving out_* untouched)
// if the pose subtree is absent or malformed; true with out_state / out_server_tick
// filled otherwise.
bool read_pose(cvc::app &app, const std::string &room, const std::string &pid,
               cvc::physics::body_state &out_state, std::uint64_t &out_server_tick);

// Write a participant's PROPOSED intent (client side -- its OWN avatar only).
void write_intent(cvc::app &app, const std::string &room, const std::string &pid,
                  const avatar_intent &intent);

// Read a participant's proposed intent (server side). Returns false if absent.
bool read_intent(cvc::app &app, const std::string &room, const std::string &pid,
                 avatar_intent &out_intent);

// Drive one authoritative frame of a SERVER holon against the replicated room
// tree: for every present participant (room_roster), add it to the holon if new,
// read its proposed .intent, feed it to the holon; advance the holon one frame;
// then write every participant's authoritative .pose + .server_tick back to the
// tree (which replicates to the clients). The caller supplies the server holon
// and calls this once per relay tick. A departed participant simply stops
// receiving intent/pose updates (reaping its holon slot is a follow-up). wall_dt
// is forwarded to holon::advance: the default (-1) measures real elapsed wall time
// (what the relay wants); a test injects a fixed dt for a deterministic step count.
void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt = -1.0);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_HOLON_SYNC_H__

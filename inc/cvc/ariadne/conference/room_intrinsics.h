/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_INTRINSICS_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_INTRINSICS_H__

// Ariadne DSL surface for the conference ROOM control plane (roadmap
// ARIADNE_CONFERENCE_ROADMAP.md §4b), namespace cvc::ariadne::conference. The conference is a LAYER
// on top of the general streaming substrate (cvc::ariadne::stream / cvc::gl::ariadne), which stays
// task-neutral; only the room/authority model is conference-specific and lives here. Opt-in,
// exactly like register_stream_intrinsics: a host/test calls register_room_intrinsics(app) to add
// the program-lane verbs:
//
//   (room-join ENDPOINT ROOM-ID [PROFILE])  Join ROOM-ID (PROFILE is a dict, e.g. {"name" …}).
//                                           Publishes my participant subtree into the shared room
//                                           tree and returns a handle dict
//                                           { "ok" "room" "pid" "token" "endpoint" } (or
//                                           { "ok" #f "error" }).
//   (room-roster ROOM)                      ROOM = the handle from (room-join). Returns a list of
//                                           participant dicts { "pid" "name" }, in pid order —
//                                           every PRESENT participant in the room tree, local and
//                                           (once replicated) remote.
//   (room-leave ROOM)                       Remove my participant subtree + drop the handle.
//                                           Returns { "ok" #t/#f }.
//
// THE ROOM TREE (roadmap §5a) is SHARED, not document-scoped: it lives at the absolute path
//   conf.rooms.<room-id>.participants.<pid>.{ profile.name, presence.joined }
// off the app root (cvc::state::SEPARATOR between segments), so every participant — each a
// different document, typically on a different node — converges on the SAME tree. Cross-node
// propagation is whatever state_cluster_shard the host installs (the verbs just read/write
// app.root(); replication is transparent). This FIRST slice is deliberately TRUST-ALL self-insert:
// a client writes its own participant subtree directly and the pid is the local cluster node id.
// Server-mediated writes, admission/pid-allocation, roster authority and presence reaping are Phase
// 2 (§5d) — this is the working skeleton they interpose on, and it matches the roadmap's "mesh
// works first; auth hardens later". The media wiring (room-publish/-subscribe over the gRPC
// bridge/relay) is a later slice.
//
// OWNERSHIP: the join handle is owned by the CALLING DOCUMENT's se::document_scope (reached via
// ictx.document), keyed by its token, exactly like (stream-open). It lives until (room-leave) or
// document teardown, when the Runtime clear()s the scope and the room table removes any participant
// subtree the author left joined (while the app + its state tree are still alive). Opening requires
// a document scope, so the verbs run on the action/resident lanes, not the load-time init: lane.

#include <string>
#include <vector>

namespace cvc {
class app;

namespace ariadne {
namespace conference {

void register_room_intrinsics(cvc::app &app);

// One present participant as seen in the (replicated) room tree.
struct room_participant {
  std::string pid;  // the participant id = its cluster node id (trust-all self-insert)
  std::string name; // profile.name, "" if unset
};

// The room model as reusable free functions. The DSL verbs wrap these (adding pid derivation + a
// document-scoped handle); they are exposed because the model is pure room-tree state with no
// document or GL dependency, so a host, a test, or the future server-side writer (§5d) can drive it
// directly. All operate on the SHARED tree off app.root() (conf.rooms.<room-id>.…), so cross-node
// propagation is whatever state_cluster_shard the host installs.

// The shared-tree path of a participant: conf.rooms.<room-id>.participants.<pid>.
std::string room_participant_path(const std::string &room_id, const std::string &pid);

// Trust-all self-insert: write PID's own participant subtree into ROOM-ID — presence.joined = "1"
// (which also materializes the <pid> node so the roster lists it) and, when NAME is non-empty,
// profile.name = NAME. Returns the participant path (room_participant_path). Idempotent per (room,
// pid): re-joining overwrites the same subtree.
std::string room_join(cvc::app &app, const std::string &room_id, const std::string &pid,
                      const std::string &name = std::string());

// Mark PID absent in ROOM-ID (a SOFT leave: presence.joined = "0"). This is a value update, so it
// replicates and converges cross-node — unlike a hard subtree delete, which a peer that still holds
// the participant resurrects on the next sync. The participant node lingers (present = false) until
// reaping; actual removal + presence/heartbeat reaping is Phase 2 (§5d). A no-op if never joined.
void room_leave(cvc::app &app, const std::string &room_id, const std::string &pid);

// Read the current roster of ROOM-ID directly from the shared room tree — the PRESENT participants
// (presence.joined == "1") and each one's profile. Usable standalone — no document scope needed —
// so the (room-roster) verb and tests both call it. Returns them in pid order; empty when the room
// has no present participants or does not exist in this app's tree yet.
std::vector<room_participant> room_roster(cvc::app &app, const std::string &room_id);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_INTRINSICS_H__

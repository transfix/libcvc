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
//   (room-leave ROOM)                       Mark me left (presence.joined = 0) + drop the handle.
//                                           A relay removes my participant subtree once I have
//                                           been away its grace (room_departures). Returns
//                                           { "ok" #t/#f }.
//
// and the chat verbs (room_chat.h; CHANNEL is "" for the room or a peer's pid): (chat-post ROOM
// TEXT [TO]), (chat-info ROOM [CHANNEL]), (chat-messages ROOM [CHANNEL [AFTER]]), (chat-more ROOM
// [CHANNEL]), (chat-conversations ROOM) and (chat-rev ROOM). docs/ARIADNE.md §5.16 has what each
// returns; the handle keeps the caller's room_chat_client.
//
// THE ROOM TREE (roadmap §5a) is SHARED, not document-scoped: it lives at the absolute path
//   conf.rooms.<room-id>.participants.<pid>.{ profile.name, presence.joined }
// off the app root (cvc::state::SEPARATOR between segments), so every participant — each a
// different document, typically on a different node — converges on the SAME tree. Cross-node
// propagation is whatever state_cluster_shard the host installs (the verbs just read/write
// app.root(); replication is transparent). A client writes its own participant subtree directly
// and the pid is the local cluster node id (self-insert, no allocation). A relay with bound writes
// (room_write_gate, arisrv's default) holds each connection to the pid of its Hello and takes from
// it only writes inside that participant, outside the relay's own keys there. The relay removes a
// departed participant's subtree once it has been away a grace (room_departures, arisrv
// --leave-grace-s). Admission with a credential, pid allocation and roster authority are Phase 2
// (§5d, §5f). The media wiring (room-publish/-subscribe over the gRPC bridge/relay) is a later
// slice.
//
// OWNERSHIP: the join handle is owned by the CALLING DOCUMENT's se::document_scope (reached via
// ictx.document), keyed by its token, exactly like (stream-open). It lives until (room-leave) or
// document teardown, when the Runtime clear()s the scope and the room table marks left any
// participant the author left joined (while the app + its state tree are still alive). Opening
// requires a document scope, so the verbs run on the action/resident lanes, not the load-time
// init: lane.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
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

// Mark PID absent in ROOM-ID: presence.joined = "0", so the roster drops it at once. What it
// published stays (its streams, avatar, device), so a participant back on the same pid (a client
// that reconnected) takes it up again by joining. On a relay, room_departures removes the
// subtree (room_remove) once the participant has been away the grace. A no-op if never joined.
void room_leave(cvc::app &app, const std::string &room_id, const std::string &pid);

// Remove PID's participant subtree from ROOM-ID. The removal replicates as one delete, which every
// peer applies, and its tombstone buries what PID wrote before it (state_sync_adapter.h). For the
// relay, the subtree's writer of last resort; a participant's own delete of its node is refused by
// room_write_gate. False when there was none.
bool room_remove(cvc::app &app, const std::string &room_id, const std::string &pid);

// A relay's departed participants (arisrv). Each one is marked left (room_leave) when its
// connection ends and noted here, and removed from the room tree (room_remove) once it has been
// away GRACE. One that comes back first, joined again or holding a connection, is kept. Owned by
// the relay's loop; not thread-safe.
class room_departures {
public:
  using clock = std::chrono::steady_clock;
  explicit room_departures(clock::duration grace) : _grace(grace) {}

  // PID's connection ended at NOW (a later departure restarts its wait).
  void left(const std::string &pid, clock::time_point now) { _left[pid] = now; }
  // Whether any participant's wait is over at NOW.
  bool due(clock::time_point now) const;
  // At NOW: remove from ROOM-ID each participant whose wait is over and that is still away (not
  // present, and not in CONNECTED, the pids that hold a connection); forget one that came back.
  // The removed pids, in pid order.
  std::vector<std::string> reap(cvc::app &app, const std::string &room_id,
                                const std::set<std::string> &connected, clock::time_point now);
  std::size_t waiting() const { return _left.size(); }
  clock::duration grace() const { return _grace; }

private:
  clock::duration _grace;
  std::map<std::string, clock::time_point> _left;
};

// Read the current roster of ROOM-ID directly from the shared room tree — the PRESENT participants
// (presence.joined == "1") and each one's profile. Usable standalone — no document scope needed —
// so the (room-roster) verb and tests both call it. Returns them in pid order; empty when the room
// has no present participants or does not exist in this app's tree yet.
std::vector<room_participant> room_roster(cvc::app &app, const std::string &room_id);

// ----------------
// Bridged users (docs/roadmap/ARIADNE_CHAT.md, "Bridged users")
// ----------------
// The users of another chat system (an NMDC hub's, say) that the relay's program bridges into the
// room: listed in the room's roster beside its participants, posting to and reading its chat, with
// no media and no avatar. The relay is their only writer, from what its program asks
// (pycvc.ariadne's host.room_presence / room_absent), in a subtree of the room tree apart from the
// participants, so the participants' code (presence, the area of interest, the holon) never sees
// them:
//
//   conf.rooms.<room>.bridge.rev                       moves with every change below (the
//                                                      relay's clock in ms, or one past the last)
//   conf.rooms.<room>.bridge.count                     how many are present
//   conf.rooms.<room>.bridge.users.<pid>.{present, uid, via, name, detail, e}
//
// `e` is the relay's epoch (its chat's, room_chat.h) when it wrote the user: one an earlier relay
// process wrote, which a client that outlived it still holds, is not listed.
// A bridged user's pid is "_<via>_<uid escaped>": VIA names the system (a lowercase letter, then
// up to 15 lowercase letters and digits), and every byte of the system's own id for the user (UID)
// that is not an ASCII letter or digit is written "_xx" (its hex). So two different users never
// share a pid, and the pid is a state name, which is what a chat channel is keyed by. It begins
// with '_', and a relay with bound writes takes no write from a participant whose pid does
// (room_write_gate), so none can join as one.
// A user who leaves is removed (one delete, as room_remove's), and one who comes back is written
// anew: the relay is their only writer, so nothing of theirs is lost.

struct room_bridged_user {
  std::string pid;    // bridged_pid(via, uid)
  std::string uid;    // the system's own id for the user (an NMDC nick)
  std::string via;    // the system ("nmdc")
  std::string name;   // what the roster and their posts show
  std::string detail; // a line about them from their system ("" for none)
};

struct bridge_limits {
  std::size_t uid_max = 128;       // bytes of a uid
  std::size_t name_max = 128;      // of a name, once cleaned
  std::size_t detail_max = 256;    // of a detail, once cleaned
  std::size_t present_max = 10000; // bridged users present in a room at once
};

// conf.rooms.<room>.bridge
std::string room_bridge_path(const std::string &room_id);
// Whether VIA names a system: [a-z][a-z0-9]{0,15}.
bool valid_bridge_via(const std::string &via);
// The pid of UID on VIA; "" when VIA is not valid_bridge_via or UID is empty.
std::string bridged_pid(const std::string &via, const std::string &uid);
// Whether PID has a bridged user's form (a leading '_'). Says nothing about whether one is present.
bool is_bridged_pid(const std::string &pid);

// UID on VIA is in ROOM-ID, shown as NAME ("" shows UID) with DETAIL. A repeat updates the name
// and detail. False, with the reason in WHY, for a VIA or UID that cannot be one (empty, too long,
// not UTF-8), or a new user past LIMITS.present_max. On the relay's thread (it is the writer).
bool room_bridge_presence(cvc::app &app, const std::string &room_id, const std::string &via,
                          const std::string &uid, const std::string &name,
                          const std::string &detail, std::string &why,
                          const bridge_limits &limits = {});
// UID on VIA left ROOM-ID: its node is removed. False when it was not present.
bool room_bridge_absent(cvc::app &app, const std::string &room_id, const std::string &via,
                        const std::string &uid);
// The bridged users present in ROOM-ID, in pid order, as this app's tree holds them.
std::vector<room_bridged_user> room_bridged(cvc::app &app, const std::string &room_id);
// The one present as PID; false when none is.
bool room_bridged_user_of(cvc::app &app, const std::string &room_id, const std::string &pid,
                          room_bridged_user &out);
// bridge.rev: 0 before the relay has bridged anyone.
std::uint64_t room_bridge_rev(cvc::app &app, const std::string &room_id);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_INTRINSICS_H__

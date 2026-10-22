/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdint>
#include <cstdlib>               // strtod (room-set-intent numeric args)
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/holon_sync.h>        // write_intent (room-set-intent verb)
#include <cvc/core/app.h>                  // app.root()
#include <cvc/state/state.h>               // cvc::state, SEPARATOR, findDescendant, children, value
#include <cvc/state/state_cluster_shard.h> // default_for / local_node_id (the trust-all pid)
#include <cvc/state/state_exec/builtins.h> // register_fn
#include <cvc/state/state_exec/intrinsics.h> // intrinsics_context, document_scope
#include <cvc/state/state_exec/types.h>      // value_t, make_dict, make_list
#include <initializer_list>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

// -------- room tree paths (roadmap §5a), joined with cvc::state::SEPARATOR --------
// Absolute off app.root(): conf.rooms.<room>.participants.<pid>.{profile.name, presence.joined}.
std::string join_path(std::initializer_list<const char *> parts) {
  std::string s;
  for (const char *p : parts) {
    if (!s.empty())
      s += cvc::state::SEPARATOR;
    s += p;
  }
  return s;
}
std::string participants_path(const std::string &room) {
  return join_path({"conf", "rooms", room.c_str(), "participants"});
}

// Set a leaf under the room tree so the write REPLICATES. Mirrors stream.cpp's descriptor
// materialization: creating a node fires the childChanged that makes it observable to the
// distributed-state adapter, but the FIRST write to a just-created node can race that attach and be
// lost (cf. state_transport_inproc_test's "set twice"). touch() fires childChanged with no content,
// so the value write that FOLLOWS is captured and replicates to peers.
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path); // create/navigate
  n.touch();                        // attach the distributed observer before the value write
  n.value(val);                     // captured -> replicates
}

// -------- small value_t helpers (same shapes as stream_intrinsics) --------
const se::value_t *dict_get(const std::vector<std::pair<std::string, se::value_t>> &d,
                            const std::string &k) {
  for (const auto &kv : d)
    if (kv.first == k)
      return &kv.second;
  return nullptr;
}
std::string get_str(const se::value_t *v, const std::string &def = std::string()) {
  if (v)
    if (const std::string *s = std::get_if<std::string>(&v->v))
      return *s;
  return def;
}
// A number from a value_t arg (double / int64 / numeric string), for the pose/intent verbs.
double get_num(const se::value_t *v, double def = 0.0) {
  if (!v)
    return def;
  if (const double *d = std::get_if<double>(&v->v))
    return *d;
  if (const std::int64_t *i = std::get_if<std::int64_t>(&v->v))
    return static_cast<double>(*i);
  if (const std::string *s = std::get_if<std::string>(&v->v))
    return std::strtod(s->c_str(), nullptr);
  return def;
}
se::value_t err(const std::string &msg) {
  return se::make_dict({{"ok", se::value_t(false)}, {"error", se::value_t(msg)}});
}
// A handle is the (room-join) dict (read "token") or a bare token string.
std::string handle_token(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&v.v))
    if (*dp)
      return get_str(dict_get(**dp, "token"));
  return std::string();
}

// -------- per-DOCUMENT owned room table (model B, mirroring stream_intrinsics' stream_table)
// ------ The join handle lives in the calling document's se::document_scope, so ownership follows
// the document's lifetime: a room joined here is retracted on (room-leave) or at document teardown,
// when ~room_table removes each still-joined participant subtree while the app + state tree are
// alive.
constexpr const char *kRoomSlot = "cvc.ariadne.room";
struct room_handle {
  std::string room_id;
  std::string self_pid;
  std::string self_path; // room_participant_path(room,pid): the handle token + removal path
};
struct room_table {
  std::mutex mu;
  cvc::app *app = nullptr; // set on first (room-join); needed to retract subtrees at teardown
  std::unordered_map<std::string, std::unique_ptr<room_handle>> by_token;

  void own(std::unique_ptr<room_handle> h) {
    std::lock_guard<std::mutex> lk(mu);
    by_token[h->self_path] = std::move(h); // token = self path, unique in the room tree
  }
  room_handle *find(const std::string &token) {
    std::lock_guard<std::mutex> lk(mu);
    auto it = by_token.find(token);
    return it == by_token.end() ? nullptr : it->second.get();
  }
  void erase(const std::string &token) {
    std::lock_guard<std::mutex> lk(mu);
    by_token.erase(token);
  }
  ~room_table() {
    // Document teardown: retract any room the author left joined. Single-threaded (the Runtime
    // clear()s the scope), so no lock; app + state tree are still alive here.
    if (app)
      for (auto &kv : by_token)
        room_leave(*app, kv.second->room_id, kv.second->self_pid);
  }
};

room_table *doc_rooms(se::document_scope *doc) {
  return doc ? doc->slot<room_table>(kRoomSlot) : nullptr;
}

} // namespace

// -------- the room model (reusable free functions; the verbs wrap these) --------

std::string room_participant_path(const std::string &room_id, const std::string &pid) {
  return participants_path(room_id) + cvc::state::SEPARATOR + pid;
}

std::string room_join(cvc::app &app, const std::string &room_id, const std::string &pid,
                      const std::string &name) {
  const std::string sp = room_participant_path(room_id, pid);
  // Write ONLY this participant's own subtree (the §5d client invariant, honoured voluntarily here;
  // enforcement is Phase 2). presence.joined also materializes the <pid> node so the roster
  // enumerates it even with no name.
  set_replicated(app, join_path({sp.c_str(), "presence", "joined"}), "1");
  if (!name.empty())
    set_replicated(app, join_path({sp.c_str(), "profile", "name"}), name);
  return sp;
}

void room_leave(cvc::app &app, const std::string &room_id, const std::string &pid) {
  // Soft leave: flip presence.joined to "0" (a replicating value update) rather than deleting the
  // subtree — a delete is resurrected by any peer that still holds the participant. The node
  // lingers (present = false) until Phase-2 reaping; room_roster filters it out.
  const std::string sp = room_participant_path(room_id, pid);
  set_replicated(app, join_path({sp.c_str(), "presence", "joined"}), "0");
}

std::vector<room_participant> room_roster(cvc::app &app, const std::string &room_id) {
  std::vector<room_participant> out;
  const std::string pp = participants_path(room_id);
  cvc::state *parts = app.root().findDescendant(pp);
  if (!parts)
    return out; // no such room (yet) in this app's tree
  // children() is a DEEP pre-order walk (no immediate-children accessor yet — §5c) and each name is
  // the descendant's FULL path from the root. The immediate children (the pids) are those with
  // exactly one more segment than pp, i.e. "<pp>.<pid>" with no further SEPARATOR.
  const std::string prefix = pp + cvc::state::SEPARATOR;
  for (const std::string &full : parts->children()) {
    if (full.rfind(prefix, 0) != 0)
      continue; // defensive: not under participants/
    const std::string rel = full.substr(prefix.size());
    if (rel.find(cvc::state::SEPARATOR) != std::string::npos)
      continue; // a deeper descendant (<pid>.presence, <pid>.profile.name, …): skip
    // Present-only: skip a participant that never joined or soft-left (presence.joined != "1").
    cvc::state *pr = app.root().findDescendant(join_path({full.c_str(), "presence", "joined"}));
    if (!pr || pr->value() != "1")
      continue;
    room_participant rp;
    rp.pid = rel;
    if (cvc::state *nm = app.root().findDescendant(join_path({full.c_str(), "profile", "name"})))
      rp.name = nm->value();
    out.push_back(std::move(rp));
  }
  return out;
}

// -------- the DSL surface (thin wrappers over the model + a document-scoped handle) --------

void register_room_intrinsics(cvc::app &app) {
  register_action_intrinsics([&app](std::shared_ptr<se::environment> env,
                                    se::intrinsics_context &ictx) {
    se::document_scope *doc = ictx.document; // per-document owner; null on the init: lane

    // (room-join ENDPOINT ROOM-ID [PROFILE]) -> handle dict { ok room pid token endpoint }.
    se::builtins::register_fn(
        env, "room-join", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-join: no document scope (join from an action or resident lane, "
                       "not the load-time init: lane)");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-join: internal: room-table slot unavailable");
          tbl->app = &app; // for teardown retraction
          if (args.size() < 2)
            return err("room-join: (room-join ENDPOINT ROOM-ID [PROFILE])");
          const std::string endpoint = get_str(&args[0]); // recorded; transport wiring is later
          const std::string room_id = get_str(&args[1]);
          if (room_id.empty())
            return err("room-join: ROOM-ID is required");
          std::string name;
          if (args.size() >= 3)
            if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&args[2].v))
              if (*dp)
                name = get_str(dict_get(**dp, "name"));

          // Trust-all self-insert: the pid is the local cluster node id (unique per node), or
          // "local" with no cluster installed (single-node). Phase 2 replaces this with a
          // server-allocated pid bound to the connection.
          std::string pid = "local";
          if (auto *shard = cvc::state_cluster_shard::default_for(app))
            if (!shard->local_node_id().empty())
              pid = shard->local_node_id();
          if (pid.find(cvc::state::SEPARATOR) != std::string::npos)
            return err("room-join: node id contains a path separator");

          const std::string sp = room_join(app, room_id, pid, name);
          auto rh = std::make_unique<room_handle>();
          rh->room_id = room_id;
          rh->self_pid = pid;
          rh->self_path = sp;
          tbl->own(std::move(rh));
          return se::make_dict({
              {"ok", se::value_t(true)},
              {"room", se::value_t(room_id)},
              {"pid", se::value_t(pid)},
              {"token", se::value_t(sp)},
              {"endpoint", se::value_t(endpoint)},
          });
        });

    // (room-roster ROOM) -> list of { pid name present }.
    se::builtins::register_fn(
        env, "room-roster", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-roster: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-roster: internal: room-table slot unavailable");
          if (args.empty())
            return err("room-roster: missing room handle or room-id");
          // Accept a (room-join) handle OR a bare room-id string, so an
          // N-party .ari can iterate with (room-roster (state-get
          // "conf.room")).
          room_handle *rh = tbl->find(handle_token(args[0]));
          const std::string room_id = rh ? rh->room_id : handle_token(args[0]);
          if (room_id.empty())
            return err("room-roster: ROOM must be a room-join handle or room-id");
          std::vector<se::value_t> list;
          for (const room_participant &p : room_roster(app, room_id))
            list.push_back(se::make_dict({
                {"pid", se::value_t(p.pid)},
                {"name", se::value_t(p.name)},
            }));
          return se::make_list(std::move(list));
        });

    // (room-leave ROOM) -> { ok #t/#f }.
    se::builtins::register_fn(env, "room-leave",
                              [&app, doc](std::span<const se::value_t> args) -> se::value_t {
                                if (!doc)
                                  return err("room-leave: no document scope");
                                room_table *tbl = doc_rooms(doc);
                                if (!tbl)
                                  return err("room-leave: internal: room-table slot unavailable");
                                if (args.empty())
                                  return err("room-leave: missing room handle");
                                const std::string token = handle_token(args[0]);
                                room_handle *rh = tbl->find(token);
                                if (!rh)
                                  return se::make_dict({{"ok", se::value_t(false)}});
                                room_leave(app, rh->room_id, rh->self_pid);
                                tbl->erase(token);
                                return se::make_dict({{"ok", se::value_t(true)}});
                              });

    // (room-set-intent HANDLE MOVE-X MOVE-Y MOVE-Z) -> { ok #t/#f }. The caller
    // PROPOSES its own avatar's desired motion by writing only its own .intent.*
    // (the server holon is the sole writer of the authoritative .pose). The handle
    // supplies the caller's pid, so a client cannot propose for another participant.
    se::builtins::register_fn(
        env, "room-set-intent", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-set-intent: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-set-intent: internal: room-table slot unavailable");
          if (args.size() < 4)
            return err("room-set-intent: (room-set-intent HANDLE MOVE-X MOVE-Y MOVE-Z)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err("room-set-intent: ROOM must be a room-join handle (needs the caller's pid)");
          cvc::ariadne::avatar_intent in;
          in.move = {get_num(&args[1]), get_num(&args[2]), get_num(&args[3])};
          cvc::ariadne::write_intent(app, rh->room_id, rh->self_pid, in);
          return se::make_dict({{"ok", se::value_t(true)}});
        });
  });
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

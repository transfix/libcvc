/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm> // max
#include <chrono>    // the chat verbs' steady clock
#include <cmath>     // atan2 (room-set-intent facing)
#include <cstdint>
#include <cstdio>                               // snprintf
#include <cstdlib>                              // strtod (room-set-intent numeric args)
#include <ctime>                                // a chat message's local clock time
#include <cvc/ariadne/ariadne.h>                // register_action_intrinsics
#include <cvc/ariadne/conference/room_avatar.h> // room-avatar-options / room-set-avatar
#include <cvc/ariadne/conference/room_chat.h>   // the chat-* verbs
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_net.h> // the roster entries carry each link's quality
#include <cvc/ariadne/holon_sync.h>          // write_intent (room-set-intent verb)
#include <cvc/core/app.h>                    // app.root()
#include <cvc/state/state.h>               // cvc::state, SEPARATOR, findDescendant, children, value
#include <cvc/state/state_cluster_shard.h> // default_for / local_node_id (the trust-all pid)
#include <cvc/state/state_exec/builtins.h> // register_fn
#include <cvc/state/state_exec/intrinsics.h> // intrinsics_context, document_scope
#include <cvc/state/state_exec/types.h>      // value_t, make_dict, make_list
#include <initializer_list>
#include <map>
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
// materialization: touch() fires the childChanged that makes a just-created node observable to the
// distributed-state adapter, with no content, so the value write that FOLLOWS is captured and
// replicates to peers. (The adapter now also journals a node's content when a write first shows it
// the node, so the first write to a fresh node replicates without the touch.)
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
  // What this participant holds of the room's chat (made by the first chat-* verb).
  std::unique_ptr<room_chat_client> chat;
  room_chat_client &chat_client(cvc::app &app) {
    if (!chat)
      chat = std::make_unique<room_chat_client>(app, room_id, self_pid);
    return *chat;
  }
};
struct room_table {
  std::mutex mu;
  cvc::app *app = nullptr; // set on first (room-join); needed to retract subtrees at teardown
  std::unordered_map<std::string, std::unique_ptr<room_handle>> by_token;
  // Each room's avatar catalog as last read, with the revision it was read at.
  struct catalog_entry {
    std::uint64_t rev = 0;
    std::shared_ptr<const avatar_catalog> catalog;
  };
  std::unordered_map<std::string, catalog_entry> catalogs;

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
  // ROOM's avatar catalog, for the avatar verbs a document calls every tick (conference_roster.ari
  // calls room-avatar-options twice and room-avatar once). A whole read walks every option's nodes
  // (a few hundred), so it reads one leaf, the catalog's revision, and the catalog again only when
  // that moved: write_avatar_catalog bumps it last, so a new revision means the options it covers
  // are here.
  std::shared_ptr<const avatar_catalog> catalog(cvc::app &app, const std::string &room) {
    const std::uint64_t rev = avatar_catalog_revision(app, room);
    {
      std::lock_guard<std::mutex> lk(mu);
      auto it = catalogs.find(room);
      if (it != catalogs.end() && it->second.rev == rev)
        return it->second.catalog;
    }
    // Read outside the lock. The revision was read first, so a write landing meanwhile leaves a
    // newer revision for the next call to see.
    auto cat = std::make_shared<const avatar_catalog>(read_avatar_catalog(app, room));
    std::lock_guard<std::mutex> lk(mu);
    catalogs[room] = catalog_entry{rev, cat};
    return cat;
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

double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// "14:05", the local time of a message's relay timestamp.
std::string clock_text(double t) {
  const std::time_t tt = static_cast<std::time_t>(t);
  std::tm tm{};
#if defined(_WIN32)
  if (localtime_s(&tm, &tt) != 0)
    return std::string();
#else
  if (!localtime_r(&tt, &tm))
    return std::string();
#endif
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%02d:%02d", tm.tm_hour, tm.tm_min);
  return buf;
}

// The chat verbs' CHANNEL argument at INDEX: "" (the room) when absent.
std::string channel_arg(std::span<const se::value_t> args, std::size_t index) {
  return args.size() > index ? get_str(&args[index]) : std::string();
}

} // namespace

// -------- the room model (reusable free functions; the verbs wrap these) --------

std::string room_participant_path(const std::string &room_id, const std::string &pid) {
  return participants_path(room_id) + cvc::state::SEPARATOR + pid;
}

std::string room_join(cvc::app &app, const std::string &room_id, const std::string &pid,
                      const std::string &name) {
  const std::string sp = room_participant_path(room_id, pid);
  // Write ONLY this participant's own subtree (the §5d client invariant; a relay with bound writes,
  // room_write_gate, refuses anything else). presence.joined also materializes the <pid> node so
  // the roster enumerates it even with no name.
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
          // Each entry also carries the participant's self-reported link quality (room_net.h):
          // raw numbers (negative = not measured yet) and roster-cell text.
          std::vector<se::value_t> list;
          for (const room_participant &p : room_roster(app, room_id)) {
            net_sample net;
            read_participant_net(app, room_id, p.pid, net);
            // Its camera is off (it sends no frames: show "camera off", not its last frame), and
            // the size the relay sends it video at ("" = not adapted yet).
            bool video_off = false;
            read_participant_video_off(app, room_id, p.pid, video_off);
            list.push_back(se::make_dict({
                {"video_off", se::value_t(std::string(video_off ? "1" : "0"))},
                {"video", se::value_t(read_relay_video_rung(app, room_id, p.pid))},
                {"pid", se::value_t(p.pid)},
                {"name", se::value_t(p.name)},
                {"rtt_ms", se::value_t(net.rtt_ms)},
                {"in_kbps", se::value_t(net.in_kbps)},
                {"out_kbps", se::value_t(net.out_kbps)},
                {"ping", se::value_t(format_rtt(net.rtt_ms))},
                {"down", se::value_t(format_rate(net.in_kbps))},
                {"up", se::value_t(format_rate(net.out_kbps))},
            }));
          }
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
            return err("room-set-intent: (room-set-intent HANDLE MOVE-X MOVE-Y MOVE-Z [YAW])");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err("room-set-intent: ROOM must be a room-join handle (needs the caller's pid)");
          cvc::ariadne::avatar_intent in;
          in.move = {get_num(&args[1]), get_num(&args[2]), get_num(&args[3])};
          // The facing: YAW (radians about +Z from +X) when given, else the way it walks.
          if (args.size() >= 5)
            in.yaw = get_num(&args[4]);
          else if (in.move.x != 0.0 || in.move.y != 0.0)
            in.yaw = std::atan2(in.move.y, in.move.x);
          cvc::ariadne::write_intent(app, rh->room_id, rh->self_pid, in);
          return se::make_dict({{"ok", se::value_t(true)}});
        });

    // (room-avatar-options ROOM KIND) -> list of labels. KIND is "color" or "accessory"; the
    // labels of what the room's catalog offers (room_avatar.h), in its order, the default first.
    // Empty until the relay's catalog has arrived. ROOM is a room-join handle or a room-id.
    se::builtins::register_fn(
        env, "room-avatar-options", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-avatar-options: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-avatar-options: internal: room-table slot unavailable");
          if (args.size() < 2)
            return err("room-avatar-options: (room-avatar-options ROOM KIND)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          const std::string room_id = rh ? rh->room_id : handle_token(args[0]);
          const std::string kind = get_str(&args[1]);
          if (kind != "color" && kind != "accessory")
            return err("room-avatar-options: KIND must be \"color\" or \"accessory\"");
          const std::shared_ptr<const avatar_catalog> cat = tbl->catalog(app, room_id);
          std::vector<se::value_t> list;
          if (kind == "color")
            for (const avatar_color_option &o : cat->colors)
              list.push_back(se::value_t(o.label));
          else
            for (const avatar_accessory_option &o : cat->accessories)
              list.push_back(se::value_t(o.label));
          return se::make_list(std::move(list));
        });

    // (room-avatar HANDLE) -> { color accessory stamina }: the labels of the caller's avatar as
    // every client draws it -- its choice resolved through the catalog, so the defaults until it
    // chose (and both "" until the catalog has arrived) -- and its reserve as the relay last
    // published it, a whole percent of the most it can hold ("" without an energy policy).
    se::builtins::register_fn(
        env, "room-avatar", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-avatar: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-avatar: internal: room-table slot unavailable");
          if (args.empty())
            return err("room-avatar: (room-avatar HANDLE)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err("room-avatar: HANDLE must be a room-join handle (needs the caller's pid)");
          const std::shared_ptr<const avatar_catalog> cat = tbl->catalog(app, rh->room_id);
          const avatar_look look = resolve_avatar_look(
              *cat, read_avatar_choice(app, rh->room_id, rh->self_pid), rh->self_pid);
          const avatar_color_option *c = cat->find_color(look.color_id);
          const avatar_accessory_option *a = cat->find_accessory(look.accessory_id);
          std::string stamina;
          avatar_energy_policy policy;
          double reserve = 0.0;
          std::uint64_t reserve_tick = 0;
          if (read_avatar_policy(app, rh->room_id, policy) && policy.enabled &&
              policy.reserve_max > 0.0 &&
              read_avatar_reserve(app, rh->room_id, rh->self_pid, reserve, reserve_tick))
            stamina = std::to_string(
                static_cast<long>(std::floor(100.0 * reserve / policy.reserve_max + 0.5)));
          return se::make_dict({
              {"color", se::value_t(c ? c->label : std::string())},
              {"accessory", se::value_t(a ? a->label : std::string())},
              {"stamina", se::value_t(stamina)},
          });
        });

    // (room-set-avatar HANDLE COLOR ACCESSORY) -> { ok #t }. The caller picks its own avatar from
    // the room's catalog: COLOR and ACCESSORY are option ids or labels. Writes only the caller's
    // own avatar.* choice (the handle supplies its pid); everyone, the caller included, draws
    // what it resolves to. An option the catalog does not offer is an error, not a write.
    se::builtins::register_fn(
        env, "room-set-avatar", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-set-avatar: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-set-avatar: internal: room-table slot unavailable");
          if (args.size() < 3)
            return err("room-set-avatar: (room-set-avatar HANDLE COLOR ACCESSORY)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err(
                "room-set-avatar: HANDLE must be a room-join handle (needs the caller's pid)");
          const std::shared_ptr<const avatar_catalog> cat = tbl->catalog(app, rh->room_id);
          const std::string color = get_str(&args[1]), accessory = get_str(&args[2]);
          const avatar_color_option *c = cat->find_color(color);
          if (!c)
            return err("room-set-avatar: the room offers no colour '" + color + "'");
          const avatar_accessory_option *a = cat->find_accessory(accessory);
          if (!a)
            return err("room-set-avatar: the room offers no accessory '" + accessory + "'");
          write_avatar_choice(app, rh->room_id, rh->self_pid, avatar_choice{c->id, a->id});
          return se::make_dict({{"ok", se::value_t(true)}});
        });

    // ---- chat (room_chat.h): CHANNEL is "" for the room, or a peer's pid for a DM ----
    // The handle's chat client, or null with ERROR set (each verb needs the caller's pid).
    const auto chat_of = [&app, doc](std::span<const se::value_t> args, const char *verb,
                                     std::string &error) -> room_chat_client * {
      error.clear();
      room_table *tbl = doc ? doc_rooms(doc) : nullptr;
      room_handle *rh = tbl && !args.empty() ? tbl->find(handle_token(args[0])) : nullptr;
      if (!rh) {
        error = std::string(verb) + ": ROOM must be a room-join handle (needs the caller's pid)";
        return nullptr;
      }
      return &rh->chat_client(app);
    };

    // (chat-post ROOM TEXT [TO]) -> { ok #t } or { ok #f error }. Posts TEXT to the room, or to
    // participant TO as a direct message. The relay may still refuse it (the room's rate limit, TO
    // having left): chat-info's error says so.
    se::builtins::register_fn(env, "chat-post",
                              [chat_of](std::span<const se::value_t> args) -> se::value_t {
                                std::string error;
                                room_chat_client *c = chat_of(args, "chat-post", error);
                                if (!c)
                                  return err(error);
                                if (args.size() < 2)
                                  return err("chat-post: (chat-post ROOM TEXT [TO])");
                                if (!c->post(get_str(&args[1]), channel_arg(args, 2), error))
                                  return err(error);
                                return se::make_dict({{"ok", se::value_t(true)}});
                              });

    // (chat-info ROOM [CHANNEL]) -> { available rev count oldest newest head first more loading
    // error }. `oldest` and `newest` are the seqs that begin and end the run (when count > 0).
    // Reads what this participant holds of CHANNEL and asks the relay for what keeps it complete
    // (the newest page the first time, a lost push). `rev` changes whenever the messages change, so
    // a tick re-reads chat-messages only then.
    se::builtins::register_fn(
        env, "chat-info", [chat_of](std::span<const se::value_t> args) -> se::value_t {
          std::string error;
          room_chat_client *c = chat_of(args, "chat-info", error);
          if (!c)
            return err(error);
          const room_chat_client::info i = c->refresh(channel_arg(args, 1), steady_seconds());
          return se::make_dict({
              {"available", se::value_t(i.available)},
              {"rev", se::value_t(static_cast<std::int64_t>(i.rev))},
              {"count", se::value_t(static_cast<std::int64_t>(i.count))},
              {"oldest", se::value_t(static_cast<std::int64_t>(i.oldest))},
              {"newest", se::value_t(static_cast<std::int64_t>(i.newest))},
              {"head", se::value_t(static_cast<std::int64_t>(i.head))},
              {"first", se::value_t(static_cast<std::int64_t>(i.first))},
              {"more", se::value_t(i.more)},
              {"loading", se::value_t(i.loading)},
              {"error", se::value_t(i.error)},
          });
        });

    // (chat-messages ROOM [CHANNEL [AFTER]]) -> list of { seq from name text t clock mine }, oldest
    // first: the contiguous run held as of the last chat-info, or with AFTER (a seq) only its
    // messages after that one. `clock` is the local "HH:MM" of the relay's timestamp `t`; `mine` is
    // #t for the caller's own messages.
    se::builtins::register_fn(
        env, "chat-messages", [chat_of](std::span<const se::value_t> args) -> se::value_t {
          std::string error;
          room_chat_client *c = chat_of(args, "chat-messages", error);
          if (!c)
            return err(error);
          std::vector<se::value_t> list;
          const std::vector<chat_message> run =
              args.size() > 2
                  ? c->messages(channel_arg(args, 1),
                                static_cast<std::uint64_t>(std::max(0.0, get_num(&args[2]))))
                  : c->messages(channel_arg(args, 1));
          for (const chat_message &m : run)
            list.push_back(se::make_dict({
                {"seq", se::value_t(static_cast<std::int64_t>(m.seq))},
                {"from", se::value_t(m.from)},
                {"name", se::value_t(m.name)},
                {"text", se::value_t(m.text)},
                {"t", se::value_t(m.t)},
                {"clock", se::value_t(clock_text(m.t))},
                {"mine", se::value_t(m.from == c->pid())},
            }));
          return se::make_list(std::move(list));
        });

    // (chat-rev ROOM) -> a number that changes with every push the relay makes to the caller (a
    // message, a page, a conversation's heads): a tick re-reads chat-conversations only then.
    se::builtins::register_fn(env, "chat-rev",
                              [chat_of](std::span<const se::value_t> args) -> se::value_t {
                                std::string error;
                                room_chat_client *c = chat_of(args, "chat-rev", error);
                                if (!c)
                                  return err(error);
                                return se::value_t(static_cast<std::int64_t>(c->pushes()));
                              });

    // (chat-more ROOM [CHANNEL]) -> { ok loading more }: asks the relay for the page before the
    // oldest message held. `loading` is #t while a request is in flight (one per channel); `more`
    // is #f once the run reaches the oldest message the relay keeps.
    se::builtins::register_fn(env, "chat-more",
                              [chat_of](std::span<const se::value_t> args) -> se::value_t {
                                std::string error;
                                room_chat_client *c = chat_of(args, "chat-more", error);
                                if (!c)
                                  return err(error);
                                const std::string channel = channel_arg(args, 1);
                                const double now = steady_seconds();
                                const bool loading = c->more(channel, now);
                                const room_chat_client::info i = c->refresh(channel, now);
                                return se::make_dict({{"ok", se::value_t(true)},
                                                      {"loading", se::value_t(loading)},
                                                      {"more", se::value_t(i.more)}});
                              });

    // (chat-conversations ROOM) -> list of { peer name head present }: the participants the
    // caller has direct messages with, in the order they started. `name` is the peer's current
    // name while it is in the room, else its pid.
    se::builtins::register_fn(
        env, "chat-conversations",
        [&app, chat_of](std::span<const se::value_t> args) -> se::value_t {
          std::string error;
          room_chat_client *c = chat_of(args, "chat-conversations", error);
          if (!c)
            return err(error);
          std::map<std::string, std::string> names;
          for (const room_participant &p : room_roster(app, c->room()))
            names[p.pid] = p.name;
          std::vector<se::value_t> list;
          for (const room_chat_client::conversation &v : c->conversations()) {
            auto it = names.find(v.peer);
            const bool present = it != names.end();
            list.push_back(se::make_dict({
                {"peer", se::value_t(v.peer)},
                {"name", se::value_t(present && !it->second.empty() ? it->second : v.peer)},
                {"head", se::value_t(static_cast<std::int64_t>(v.head))},
                {"present", se::value_t(present)},
            }));
          }
          return se::make_list(std::move(list));
        });
  });
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

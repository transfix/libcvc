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
#include <cvc/ariadne/conference/room_calls.h>  // room-call
#include <cvc/ariadne/conference/room_chat.h>   // the chat-* verbs
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_net.h> // the roster entries carry each link's quality
#include <cvc/ariadne/conference/room_rules.h> // room-avatar: the policy (stamina)
#include <cvc/ariadne/conference/room_write_gate.h> // room_own_state: the relay's keys
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
// the document's lifetime: a room joined here is left on (room-leave) or at document teardown,
// when ~room_table marks each still-joined participant left while the app + state tree are alive.
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
  // Each room's bridged users as last read, with the bridge.rev it was read at.
  struct bridged_entry {
    std::uint64_t rev = 0;
    std::shared_ptr<const std::vector<room_bridged_user>> users;
  };
  std::unordered_map<std::string, bridged_entry> bridged_users;

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
  // What a participant this document took out with (room-leave) had published (room_own_state),
  // by its token: a relay removes the node after its grace, so a later (room-join) here writes it
  // again (room_reassert).
  std::unordered_map<std::string, room_own_leaves> left_own;
  void keep_own(const std::string &token, room_own_leaves own) {
    std::lock_guard<std::mutex> lk(mu);
    left_own[token] = std::move(own);
  }
  room_own_leaves take_own(const std::string &token) {
    std::lock_guard<std::mutex> lk(mu);
    auto it = left_own.find(token);
    if (it == left_own.end())
      return {};
    room_own_leaves out = std::move(it->second);
    left_own.erase(it);
    return out;
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
  // ROOM's bridged users, read again only when bridge.rev moved (room_bridge_presence bumps it
  // after the user's fields), as the catalog is: (room-roster ROOM "all") runs every tick.
  std::shared_ptr<const std::vector<room_bridged_user>> bridged(cvc::app &app,
                                                                const std::string &room) {
    const std::uint64_t rev = room_bridge_rev(app, room);
    {
      std::lock_guard<std::mutex> lk(mu);
      auto it = bridged_users.find(room);
      if (it != bridged_users.end() && it->second.rev == rev && it->second.users)
        return it->second.users;
    }
    auto users = std::make_shared<const std::vector<room_bridged_user>>(room_bridged(app, room));
    std::lock_guard<std::mutex> lk(mu);
    bridged_users[room] = bridged_entry{rev, users};
    return users;
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
  if (name.empty())
    return sp;
  // On a relay the name is the relay's (client_registry.h; room_write_gate refuses the write), so
  // ask it with server.set: it answers by writing profile.name, made unique if another client
  // has the name. Without one (a mesh, or no relay joined), write it ourselves.
  const std::shared_ptr<room_call_client> calls = room_call_client::for_app(app);
  if (calls && calls->room() == room_id && calls->pid() == pid) {
    calls->call("server.set", se::make_list(),
                se::make_dict({{"username", se::value_t(name)}, {"unique", se::value_t(true)}}),
                std::string(), 0.0, [](const room_call_client::outcome &) {});
    return sp;
  }
  set_replicated(app, join_path({sp.c_str(), "profile", "name"}), name);
  return sp;
}

void room_leave(cvc::app &app, const std::string &room_id, const std::string &pid) {
  // presence.joined to "0": room_roster filters it out at once, and the rest stays for a
  // participant back on the same pid. A relay removes the subtree later (room_departures).
  const std::string sp = room_participant_path(room_id, pid);
  set_replicated(app, join_path({sp.c_str(), "presence", "joined"}), "0");
}

bool room_remove(cvc::app &app, const std::string &room_id, const std::string &pid) {
  cvc::state *parts = app.root().findDescendant(participants_path(room_id));
  return parts && parts->removeChild(pid);
}

namespace {
// PID's presence.joined in ROOM-ID: "1", "0", or "" when it has none.
std::string joined_of(cvc::app &app, const std::string &room_id, const std::string &pid) {
  cvc::state *pr = app.root().findDescendant(
      join_path({room_participant_path(room_id, pid).c_str(), "presence", "joined"}));
  return pr ? pr->value() : std::string();
}
} // namespace

std::vector<std::string> room_departures::reap(cvc::app &app, const std::string &room_id,
                                               const std::set<std::string> &connected,
                                               clock::time_point now) {
  // One marked left that holds a connection: a document's (room-leave), which closes nothing.
  // children() is a deep walk by full path (room_roster's note): the pids are one segment down.
  if (cvc::state *parts = app.root().findDescendant(participants_path(room_id))) {
    const std::string prefix = participants_path(room_id) + cvc::state::SEPARATOR;
    for (const std::string &full : parts->children()) {
      if (full.rfind(prefix, 0) != 0 ||
          full.find(cvc::state::SEPARATOR, prefix.size()) != std::string::npos)
        continue;
      const std::string pid = full.substr(prefix.size());
      if (!_left.count(pid) && joined_of(app, room_id, pid) == "0")
        _left[pid] = away{now, connected.count(pid) > 0};
    }
  }
  std::vector<std::string> out;
  for (auto it = _left.begin(); it != _left.end();) {
    const std::string &pid = it->first;
    if (joined_of(app, room_id, pid) == "1") {
      it = _left.erase(it); // back: its next departure starts a new wait
      continue;
    }
    away &a = it->second;
    if (connected.count(pid) != 0 && !a.connected) {
      a = away{now, true}; // reconnected but not joined yet: the whole grace to do it
      ++it;
      continue;
    }
    if (now - a.since < _grace) {
      ++it;
      continue;
    }
    if (room_remove(app, room_id, pid))
      out.push_back(pid);
    it = _left.erase(it);
  }
  return out;
}

room_own_leaves room_own_state(cvc::app &app, const std::string &room_id, const std::string &pid) {
  room_own_leaves out;
  cvc::state *node = app.root().findDescendant(room_participant_path(room_id, pid));
  if (!node)
    return out;
  const room_write_gate gate(room_id);
  const std::string prefix = room_participant_path(room_id, pid) + cvc::state::SEPARATOR;
  for (const std::string &full : node->children()) { // the whole subtree, by full path
    if (full.rfind(prefix, 0) != 0 ||
        !gate.may_write(pid, full, cvc::state_mutation_op::set_value)) // not the relay's
      continue;
    const std::string rel = full.substr(prefix.size());
    const std::string top = rel.substr(0, rel.find(cvc::state::SEPARATOR));
    if (top == "presence" || top == "net" || top == "intent" || top == "drive")
      continue;
    cvc::state *c = app.root().findDescendant(full);
    if (c && !c->value().empty())
      out.emplace(rel, c->value());
  }
  return out;
}

std::size_t room_reassert(cvc::app &app, const std::string &room_id, const std::string &pid,
                          const room_own_leaves &own) {
  const std::string base = room_participant_path(room_id, pid);
  std::size_t n = 0;
  for (const auto &[rel, v] : own) {
    const std::string full = base + cvc::state::SEPARATOR + rel;
    cvc::state *c = app.root().findDescendant(full);
    if (c && !c->value().empty() && c->value() != v)
      continue; // written anew since: that write went out
    set_replicated(app, full, v);
    ++n;
  }
  return n;
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

// -------- bridged users --------

namespace {

std::string bridge_users_path(const std::string &room_id) {
  return room_bridge_path(room_id) + cvc::state::SEPARATOR + "users";
}

std::string tree_leaf(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string();
}

// A write the relay's drain fans out, skipped when the leaf holds it already. Whether it wrote.
bool set_if_changed(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  if (n.value() == val)
    return false;
  n.touch();
  n.value(val);
  return true;
}

// bridge.rev: the wall clock in ms, or one past the last when that is not later, so a relay that
// restarts goes on past the revs an earlier one gave (a client may hold them still).
void bump_bridge_rev(cvc::app &app, const std::string &room_id) {
  const std::string path = room_bridge_path(room_id) + cvc::state::SEPARATOR + "rev";
  const auto now_ms =
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
  set_replicated(app, path, std::to_string(std::max(room_bridge_rev(app, room_id) + 1, now_ms)));
}

std::uint64_t leaf_u64(cvc::app &app, const std::string &path) {
  const std::string v = tree_leaf(app, path);
  char *end = nullptr;
  const unsigned long long n = v.empty() ? 0 : std::strtoull(v.c_str(), &end, 10);
  return !v.empty() && end && *end == '\0' ? static_cast<std::uint64_t>(n) : 0;
}

// bridge.count: how many are present, kept by the relay as they come and go, so a presence never
// walks every user to check the limit (a hub has thousands, and they all log in at once).
std::uint64_t bridge_count(cvc::app &app, const std::string &room_id) {
  return leaf_u64(app, room_bridge_path(room_id) + cvc::state::SEPARATOR + "count");
}
void set_bridge_count(cvc::app &app, const std::string &room_id, std::uint64_t n) {
  set_replicated(app, room_bridge_path(room_id) + cvc::state::SEPARATOR + "count",
                 std::to_string(n));
}

// The relay's epoch (the chat's, room_chat.h): a bridged user an earlier relay process wrote, which
// a client that outlived it still holds, is not this relay's.
std::string bridge_epoch(cvc::app &app, const std::string &room_id) {
  return tree_leaf(app, chat_root_path(room_id) + cvc::state::SEPARATOR + "epoch");
}

// TEXT as one line: cleaned as a chat post is (clean_chat_text), newlines and tabs made spaces.
// "" when nothing is left. False, with WHY, for text that is not UTF-8 or longer than MAX.
bool clean_line(std::string &text, std::size_t max, std::string &why) {
  for (char &c : text)
    if (c == '\n' || c == '\t' || c == '\r')
      c = ' ';
  std::string t = text;
  if (t.find_first_not_of(' ') == std::string::npos) {
    text.clear();
    return true;
  }
  if (!clean_chat_text(t, max, why))
    return false;
  const std::size_t lead = t.find_first_not_of(' ');
  text = t.substr(lead == std::string::npos ? t.size() : lead);
  return true;
}

room_bridged_user read_bridged(cvc::app &app, const std::string &base, const std::string &pid) {
  const auto at = [&](const char *k) {
    return tree_leaf(app, base + cvc::state::SEPARATOR + pid + cvc::state::SEPARATOR + k);
  };
  room_bridged_user u;
  u.pid = pid;
  u.uid = at("uid");
  u.via = at("via");
  u.name = at("name");
  u.detail = at("detail");
  if (u.name.empty())
    u.name = u.uid;
  return u;
}

} // namespace

std::string room_bridge_path(const std::string &room_id) {
  return join_path({"conf", "rooms", room_id.c_str(), "bridge"});
}

bool valid_bridge_via(const std::string &via) {
  if (via.empty() || via.size() > 16 || via[0] < 'a' || via[0] > 'z')
    return false;
  for (const char c : via)
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')))
      return false;
  return true;
}

std::string bridged_pid(const std::string &via, const std::string &uid) {
  if (!valid_bridge_via(via) || uid.empty())
    return std::string();
  static const char kHex[] = "0123456789abcdef";
  std::string out = "_" + via + "_";
  for (const char ch : uid) {
    const auto c = static_cast<unsigned char>(ch);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
      out += ch;
    } else {
      out += '_';
      out += kHex[c >> 4];
      out += kHex[c & 0xF];
    }
  }
  return out;
}

bool is_bridged_pid(const std::string &pid) { return !pid.empty() && pid[0] == '_'; }

bool room_bridge_presence(cvc::app &app, const std::string &room_id, const std::string &via,
                          const std::string &uid, const std::string &name,
                          const std::string &detail, std::string &why,
                          const bridge_limits &limits) {
  if (!valid_bridge_via(via)) {
    why = "via '" + via +
          "' is not a system's name (a lowercase letter, then up to 15 lowercase "
          "letters and digits)";
    return false;
  }
  if (uid.empty() || uid.size() > limits.uid_max) {
    why = uid.empty() ? "the user id is empty"
                      : "the user id is longer than " + std::to_string(limits.uid_max) + " bytes";
    return false;
  }
  std::string probe = uid, unused;
  if (!clean_chat_text(probe, uid.size(), unused)) {
    why = "the user id is blank or not UTF-8 text";
    return false;
  }
  std::string n = name, d = detail;
  if (!clean_line(n, limits.name_max, why) || !clean_line(d, limits.detail_max, why))
    return false;
  const std::string pid = bridged_pid(via, uid);
  const std::string base = bridge_users_path(room_id) + cvc::state::SEPARATOR + pid;
  const std::string present_path = base + cvc::state::SEPARATOR + "present";
  const bool was = tree_leaf(app, present_path) == "1" &&
                   tree_leaf(app, base + cvc::state::SEPARATOR + "e") == bridge_epoch(app, room_id);
  const std::uint64_t count = bridge_count(app, room_id);
  if (!was && count >= limits.present_max) {
    why = "the room has " + std::to_string(limits.present_max) + " bridged users already";
    return false;
  }
  if (!was)
    set_bridge_count(app, room_id, count + 1);
  bool changed = false;
  // The fields first and `present` last, so a client that sees it present has the rest.
  changed |= set_if_changed(app, base + cvc::state::SEPARATOR + "uid", uid);
  changed |= set_if_changed(app, base + cvc::state::SEPARATOR + "via", via);
  changed |= set_if_changed(app, base + cvc::state::SEPARATOR + "name", n);
  changed |= set_if_changed(app, base + cvc::state::SEPARATOR + "detail", d);
  changed |= set_if_changed(app, base + cvc::state::SEPARATOR + "e", bridge_epoch(app, room_id));
  changed |= set_if_changed(app, present_path, "1");
  if (changed)
    bump_bridge_rev(app, room_id);
  return true;
}

bool room_bridge_absent(cvc::app &app, const std::string &room_id, const std::string &via,
                        const std::string &uid) {
  const std::string pid = bridged_pid(via, uid);
  if (pid.empty())
    return false;
  const std::string present_path =
      bridge_users_path(room_id) + cvc::state::SEPARATOR + pid + cvc::state::SEPARATOR + "present";
  if (tree_leaf(app, present_path) != "1")
    return false;
  if (cvc::state *users = app.root().findDescendant(bridge_users_path(room_id)))
    users->removeChild(pid);
  const std::uint64_t count = bridge_count(app, room_id);
  set_bridge_count(app, room_id, count > 0 ? count - 1 : 0);
  bump_bridge_rev(app, room_id);
  return true;
}

std::vector<room_bridged_user> room_bridged(cvc::app &app, const std::string &room_id) {
  std::vector<room_bridged_user> out;
  const std::string base = bridge_users_path(room_id);
  cvc::pin_vector chain;
  const cvc::state::state_ptr users = app.root().findDescendantShared(base, &chain);
  if (!users)
    return out;
  std::vector<std::string> pids = users->childNames();
  std::sort(pids.begin(), pids.end());
  const std::string epoch = bridge_epoch(app, room_id);
  for (const std::string &pid : pids) {
    const std::string at = base + cvc::state::SEPARATOR + pid + cvc::state::SEPARATOR;
    if (tree_leaf(app, at + "present") != "1" || tree_leaf(app, at + "e") != epoch)
      continue;
    out.push_back(read_bridged(app, base, pid));
  }
  return out;
}

bool room_bridged_user_of(cvc::app &app, const std::string &room_id, const std::string &pid,
                          room_bridged_user &out) {
  if (!is_bridged_pid(pid) || !cvc::state::isValidStateName(pid))
    return false;
  const std::string base = bridge_users_path(room_id);
  const std::string at = base + cvc::state::SEPARATOR + pid + cvc::state::SEPARATOR;
  if (tree_leaf(app, at + "present") != "1" ||
      tree_leaf(app, at + "e") != bridge_epoch(app, room_id))
    return false;
  out = read_bridged(app, base, pid);
  return true;
}

std::uint64_t room_bridge_rev(cvc::app &app, const std::string &room_id) {
  return leaf_u64(app, room_bridge_path(room_id) + cvc::state::SEPARATOR + "rev");
}

// -------- the DSL surface (thin wrappers over the model + a document-scoped handle) --------

void register_room_intrinsics(cvc::app &app) {
  register_room_call_intrinsics(app); // room-call (room_calls.h): a host that joined a relay
  auto provider = [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
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
          // Back after a (room-leave): what it published then, which a relay may have removed.
          room_reassert(app, room_id, pid, tbl->take_own(sp));
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

    // (room-roster ROOM [KIND [LIMIT]]) -> list of { pid name bridged ... }. KIND: "participants"
    // (the default: who has media and an avatar), "bridged" (the users the relay's program bridges
    // in from another chat system, room_bridged) or "all" (the participants, then the bridged).
    // LIMIT: at most that many bridged users (a room may bridge in thousands).
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
          const std::string kind = args.size() > 1 ? get_str(&args[1]) : "participants";
          if (kind != "participants" && kind != "bridged" && kind != "all")
            return err("room-roster: KIND is \"participants\", \"bridged\" or \"all\"");
          std::vector<se::value_t> list;
          // Each entry also carries the participant's self-reported link quality (room_net.h):
          // raw numbers (negative = not measured yet) and roster-cell text.
          for (const room_participant &p :
               kind == "bridged" ? std::vector<room_participant>() : room_roster(app, room_id)) {
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
                {"bridged", se::value_t(std::string("0"))},
            }));
          }
          // A bridged user has no camera, microphone or link to show: their camera is off, and
          // where a participant's link is, their system (VIA) and its line about them (DETAIL).
          const std::shared_ptr<const std::vector<room_bridged_user>> bridged =
              kind != "participants" ? tbl->bridged(app, room_id) : nullptr;
          std::size_t limit = bridged ? bridged->size() : 0;
          if (args.size() > 2)
            limit = std::min(limit, static_cast<std::size_t>(std::max(0.0, get_num(&args[2]))));
          if (bridged)
            for (const room_bridged_user &u : std::span(bridged->data(), limit)) {
              list.push_back(se::make_dict({
                  {"video_off", se::value_t(std::string("1"))},
                  {"video", se::value_t(std::string())},
                  {"pid", se::value_t(u.pid)},
                  {"name", se::value_t(u.name)},
                  {"rtt_ms", se::value_t(-1.0)},
                  {"in_kbps", se::value_t(-1.0)},
                  {"out_kbps", se::value_t(-1.0)},
                  {"ping", se::value_t(std::string())},
                  {"down", se::value_t(std::string())},
                  {"up", se::value_t(std::string())},
                  {"bridged", se::value_t(std::string("1"))},
                  {"via", se::value_t(u.via)},
                  {"uid", se::value_t(u.uid)},
                  {"detail", se::value_t(u.detail)},
              }));
            }
          return se::make_list(std::move(list));
        });

    // (room-bridged-count ROOM) -> how many bridged users are in the room.
    se::builtins::register_fn(
        env, "room-bridged-count", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          room_table *tbl = doc ? doc_rooms(doc) : nullptr;
          if (!tbl || args.empty())
            return se::value_t(std::int64_t(0));
          room_handle *rh = tbl->find(handle_token(args[0]));
          const auto bridged = tbl->bridged(app, rh ? rh->room_id : handle_token(args[0]));
          return se::value_t(static_cast<std::int64_t>(bridged ? bridged->size() : 0));
        });

    // (room-bridged-rev ROOM) -> the room's bridge.rev (room_bridge_rev): it moves whenever a
    // bridged user comes, goes or changes, so a document rebuilds what it shows of them only then.
    se::builtins::register_fn(
        env, "room-bridged-rev", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          room_table *tbl = doc ? doc_rooms(doc) : nullptr;
          if (!tbl || args.empty())
            return se::value_t(std::int64_t(0));
          room_handle *rh = tbl->find(handle_token(args[0]));
          const std::string room_id = rh ? rh->room_id : handle_token(args[0]);
          return se::value_t(static_cast<std::int64_t>(room_bridge_rev(app, room_id)));
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
                                tbl->keep_own(token,
                                              room_own_state(app, rh->room_id, rh->self_pid));
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

    // (room-drive HANDLE ENTITY) -> { ok #t seq N }: the caller asks to get into world entity
    // ENTITY (its id in the room's entities) and drive it, or to get out with 0 (ARIADNE_WORLDS.md
    // section 18.6). The relay answers under the caller's drive_status (room-driving reads it):
    // a vehicle must be drivable and within its reach.
    se::builtins::register_fn(
        env, "room-drive", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-drive: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-drive: internal: room-table slot unavailable");
          if (args.size() < 2)
            return err("room-drive: (room-drive HANDLE ENTITY)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err("room-drive: HANDLE must be a room-join handle (needs the caller's pid)");
          const double entity = get_num(&args[1]);
          if (!(entity >= 0.0) || entity != std::floor(entity))
            return err("room-drive: ENTITY must be an entity id, or 0 to get out");
          const std::uint64_t seq = cvc::ariadne::request_drive(app, rh->room_id, rh->self_pid,
                                                                static_cast<std::uint64_t>(entity));
          return se::make_dict(
              {{"ok", se::value_t(true)}, {"seq", se::value_t(static_cast<double>(seq))}});
        });

    // (room-driving HANDLE) -> { driving N seq N ok #t/#f why "..." }: the entity the caller
    // drives as the relay says (0: none), and its answer to the caller's last room-drive.
    se::builtins::register_fn(
        env, "room-driving", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-driving: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-driving: internal: room-table slot unavailable");
          if (args.empty())
            return err("room-driving: (room-driving HANDLE)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err("room-driving: HANDLE must be a room-join handle (needs the caller's pid)");
          const cvc::ariadne::drive_status st =
              cvc::ariadne::read_drive_status(app, rh->room_id, rh->self_pid);
          return se::make_dict({
              {"driving", se::value_t(static_cast<double>(
                              cvc::ariadne::read_driving(app, rh->room_id, rh->self_pid)))},
              {"seq", se::value_t(static_cast<double>(st.seq))},
              {"ok", se::value_t(st.ok)},
              {"why", se::value_t(st.why)},
          });
        });

    // (room-archetypes ROOM) -> list of ids: the avatar archetypes the room's world offers
    // (avatars.archetypes), in its order, the default first; empty until they have arrived. One
    // may be a vehicle (ARIADNE_WORLDS.md section 18.6). ROOM is a room-join handle or a room-id.
    se::builtins::register_fn(
        env, "room-archetypes", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-archetypes: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-archetypes: internal: room-table slot unavailable");
          if (args.empty())
            return err("room-archetypes: (room-archetypes ROOM)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          const std::string room_id = rh ? rh->room_id : handle_token(args[0]);
          std::vector<avatar_archetype> list;
          read_avatar_archetypes(app, room_id, list);
          std::vector<se::value_t> ids;
          for (const avatar_archetype &a : list)
            ids.push_back(se::value_t(a.id));
          return se::make_list(std::move(ids));
        });

    // (room-set-archetype HANDLE ID) -> { ok #t }: the caller asks for avatar archetype ID ("" the
    // first). The relay rules on it (room-archetype reads its answer); one that is a vehicle is
    // the caller's body from then on, which it drives.
    se::builtins::register_fn(
        env, "room-set-archetype", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-set-archetype: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-set-archetype: internal: room-table slot unavailable");
          if (args.size() < 2)
            return err("room-set-archetype: (room-set-archetype HANDLE ID)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err(
                "room-set-archetype: HANDLE must be a room-join handle (needs the caller's pid)");
          const std::string id = get_str(&args[1]);
          if (!id.empty() && !cvc::state::isValidStateName(id))
            return err("room-set-archetype: ID must be an archetype's id");
          cvc::ariadne::request_archetype(app, rh->room_id, rh->self_pid, id);
          return se::make_dict({{"ok", se::value_t(true)}});
        });

    // (room-archetype HANDLE) -> { archetype "..." vehicle "..." ok #t/#f why "..." }: the
    // caller's archetype as the relay last ruled (the one asked for until it has; "" before
    // either: the first), the vehicle archetype it is when it is one ("" a person), and the
    // relay's answer to the last ask (ok #t until it refuses one).
    se::builtins::register_fn(
        env, "room-archetype", [&app, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("room-archetype: no document scope");
          room_table *tbl = doc_rooms(doc);
          if (!tbl)
            return err("room-archetype: internal: room-table slot unavailable");
          if (args.empty())
            return err("room-archetype: (room-archetype HANDLE)");
          room_handle *rh = tbl->find(handle_token(args[0]));
          if (!rh)
            return err(
                "room-archetype: HANDLE must be a room-join handle (needs the caller's pid)");
          const cvc::ariadne::archetype_status st =
              cvc::ariadne::read_archetype_status(app, rh->room_id, rh->self_pid);
          std::string id =
              st.answered && st.ok
                  ? st.archetype
                  : cvc::ariadne::read_requested_archetype(app, rh->room_id, rh->self_pid);
          std::vector<avatar_archetype> list;
          read_avatar_archetypes(app, rh->room_id, list);
          if (id.empty() && !list.empty())
            id = list.front().id;
          std::string vehicle;
          for (const avatar_archetype &a : list)
            if (a.id == id)
              vehicle = a.vehicle;
          return se::make_dict({
              {"archetype", se::value_t(id)},
              {"vehicle", se::value_t(vehicle)},
              {"ok", se::value_t(!st.answered || st.ok)},
              {"why", se::value_t(st.why)},
          });
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
          avatar_policy policy;
          double reserve = 0.0;
          std::uint64_t reserve_tick = 0;
          if (read_avatar_policy(app, rh->room_id, policy) && policy.energy.enabled &&
              policy.energy.reserve_max > 0.0 &&
              read_avatar_reserve(app, rh->room_id, rh->self_pid, reserve, reserve_tick))
            stamina = std::to_string(
                static_cast<long>(std::floor(100.0 * reserve / policy.energy.reserve_max + 0.5)));
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

    // (chat-messages ROOM [CHANNEL [AFTER]]) -> list of { seq from name text t clock mine via },
    // oldest first: the contiguous run held as of the last chat-info, or with AFTER (a seq) only
    // its messages after that one. `clock` is the local "HH:MM" of the relay's timestamp `t`;
    // `mine` is #t for the caller's own messages; `via` is the system a bridged user posted from
    // ("" for a participant's message).
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
                {"via", se::value_t(m.via)},
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
    // caller has direct messages with (participants and bridged users), in the order they
    // started. `name` is the peer's current name while it is in the room, else its pid.
    se::builtins::register_fn(
        env, "chat-conversations",
        [&app, doc, chat_of](std::span<const se::value_t> args) -> se::value_t {
          std::string error;
          room_chat_client *c = chat_of(args, "chat-conversations", error);
          if (!c)
            return err(error);
          std::map<std::string, std::string> names;
          for (const room_participant &p : room_roster(app, c->room()))
            names[p.pid] = p.name;
          // A bridged peer's name from the room's bridged users (in pid order).
          const auto bridged = doc_rooms(doc)->bridged(app, c->room());
          std::vector<se::value_t> list;
          for (const room_chat_client::conversation &v : c->conversations()) {
            if (bridged && is_bridged_pid(v.peer)) {
              auto b = std::lower_bound(
                  bridged->begin(), bridged->end(), v.peer,
                  [](const room_bridged_user &u, const std::string &pid) { return u.pid < pid; });
              if (b != bridged->end() && b->pid == v.peer)
                names[v.peer] = b->name;
            }
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
  };
  // Kind host_proxied (S1-Q7): the verbs walk and write the app-global room tree raw, and use a
  // room_handle (its chat client) outside room_table's lock, so in an `any` group they run on the
  // host thread. The body reads only the context and binds natives into the new env, so a spawned
  // child may run it on a worker. Keyed: a second runtime on the app keeps one copy.
  register_action_intrinsics(app, "cvc.ariadne.room", std::move(provider),
                             se::provider_affinity::host_proxied);
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

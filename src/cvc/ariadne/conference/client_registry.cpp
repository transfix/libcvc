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

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <cvc/ariadne/conference/client_registry.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_net.h>        // room_device_tag
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/state/state_link_stats.h> // state_link_report
#include <iterator>                     // std::next
#include <random>
#include <utility>

namespace se = cvc::state_exec;

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

constexpr double kRenameWindowS = 60.0;

// "<length>:<bytes>", so no two sequences of fields hash alike by concatenation.
void append_field(std::string &out, const std::string &field) {
  out += std::to_string(field.size());
  out.push_back(':');
  out += field;
}

std::string random_salt() {
  std::random_device rd;
  static const char *const kHex = "0123456789abcdef";
  std::string s;
  for (int i = 0; i < 32; ++i) {
    const unsigned v = rd() & 0xffu;
    s.push_back(kHex[v >> 4]);
    s.push_back(kHex[v & 0xf]);
  }
  return s;
}

std::string join(const std::string &a, const std::string &b) {
  return a + cvc::state::SEPARATOR + b;
}

// ASCII case folded, for "is this name taken" (two names that differ only in case look alike).
std::string folded(const std::string &s) {
  std::string out = s;
  for (char &c : out)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

// S cut to at most N bytes, never inside a UTF-8 sequence.
std::string utf8_prefix(const std::string &s, std::size_t n) {
  if (s.size() <= n)
    return s;
  while (n > 0 && (static_cast<unsigned char>(s[n]) & 0xC0) == 0x80)
    --n;
  return s.substr(0, n);
}

// Whether S is valid UTF-8 with no control character (C0, DEL or C1).
bool clean_utf8(const std::string &s) {
  const auto *p = reinterpret_cast<const unsigned char *>(s.data());
  const std::size_t n = s.size();
  for (std::size_t i = 0; i < n;) {
    const unsigned char c = p[i];
    if (c < 0x20 || c == 0x7F)
      return false;
    if (c < 0x80) {
      ++i;
      continue;
    }
    std::size_t len = 0;
    std::uint32_t cp = 0;
    if ((c & 0xE0) == 0xC0) {
      len = 2;
      cp = c & 0x1F;
    } else if ((c & 0xF0) == 0xE0) {
      len = 3;
      cp = c & 0x0F;
    } else if ((c & 0xF8) == 0xF0) {
      len = 4;
      cp = c & 0x07;
    } else {
      return false;
    }
    if (i + len > n)
      return false;
    for (std::size_t k = 1; k < len; ++k) {
      if ((p[i + k] & 0xC0) != 0x80)
        return false;
      cp = (cp << 6) | (p[i + k] & 0x3F);
    }
    // Overlong forms, surrogates, past U+10FFFF, and the C1 controls.
    if ((len == 2 && cp < 0x80) || (len == 3 && cp < 0x800) || (len == 4 && cp < 0x10000) ||
        (cp >= 0xD800 && cp <= 0xDFFF) || cp > 0x10FFFF || (cp >= 0x80 && cp <= 0x9F))
      return false;
    i += len;
  }
  return true;
}

// Changed-only, with the room tree's touch-before-value for a new or empty leaf (room_net.cpp's
// set_changed_once), so a replicated leaf's first write is not lost.
void put(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state *n = app.root().findDescendant(path);
  const std::string cur = n ? n->value() : std::string();
  if (n && cur == val)
    return;
  if (!cur.empty()) {
    n->value(val);
    return;
  }
  cvc::state &m = app.root()(path);
  m.touch();
  m.value(val);
}

void remove_child(cvc::app &app, const std::string &parent, const std::string &name) {
  if (cvc::state *n = app.root().findDescendant(parent))
    n->removeChild(name);
}

std::string seconds_text(double s) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%g", s);
  return buf;
}

// A client's limits as server.client answers them: only the ones set.
// A list of strings, as a value (a client's capabilities).
se::value_t strings_value(const std::vector<std::string> &v) {
  std::vector<se::value_t> out;
  for (const std::string &s : v)
    out.emplace_back(s);
  return se::make_list(std::move(out));
}

// The same, as one comma-separated leaf for the private mirror.
std::string comma_list(const std::vector<std::string> &v) {
  std::string out;
  for (const std::string &s : v)
    out += (out.empty() ? "" : ",") + s;
  return out;
}

se::value_t limits_value(const client_registry::client_limits &l) {
  std::vector<std::pair<std::string, se::value_t>> out;
  if (l.programs)
    out.emplace_back("programs", se::value_t(*l.programs));
  if (l.guests)
    out.emplace_back("guests", se::value_t(*l.guests));
  if (l.steps)
    out.emplace_back("steps", se::value_t(static_cast<std::int64_t>(l.steps)));
  if (l.time > 0.0)
    out.emplace_back("time", se::value_t(l.time));
  if (l.memory_mb)
    out.emplace_back("memory_mb", se::value_t(static_cast<std::int64_t>(l.memory_mb)));
  return se::make_dict(std::move(out));
}

} // namespace

client_registry::client_registry(options o) : _opt(std::move(o)) {
  if (_opt.salt.empty())
    _opt.salt = random_salt();
  if (_opt.max_clients == 0)
    _opt.max_clients = 1;
  if (_opt.max_connections == 0)
    _opt.max_connections = 1;
}

std::string client_registry::client_id(const std::string &device_id,
                                       const std::string &local_address,
                                       const std::string &public_address) const {
  std::string in = "cvc-client-id/1";
  in.push_back('\0');
  append_field(in, _opt.salt);
  if (!device_id.empty()) {
    append_field(in, "device");
    append_field(in, device_id);
  } else {
    append_field(in, "address");
    append_field(in, local_address);
    append_field(in, public_address);
  }
  // "c" first, so the id is a state name (a letter, then letters and digits) and keys a node.
  return "c" + cvc::sha256_hex(reinterpret_cast<const unsigned char *>(in.data()), in.size())
                   .substr(0, 31);
}

void client_registry::update(const std::vector<cvc::state_link_report> &links, double now_s,
                             const std::string &wall) {
  std::set<std::string> seen;
  for (const cvc::state_link_report &r : links) {
    // A relay's link to a client, once its Hello has named it.
    if (!r.accepted || r.remote_node_id.empty())
      continue;
    const std::string key = r.transport + "#" + std::to_string(r.connection);
    seen.insert(key);
    auto it = _links.find(key);
    if (it == _links.end()) {
      const std::string pub = address_host(r.remote_address);
      const std::string id = client_id(r.remote_device_id, r.remote_local_address, pub);
      record &rec = admit(id, now_s, wall);
      connection c;
      c.seq = rec.next_seq++;
      c.pid = r.remote_node_id;
      c.transport = r.transport;
      c.public_address = pub;
      c.local_address = r.remote_local_address;
      c.opened = wall;
      c.active = true;
      rec.connections.push_back(std::move(c));
      while (rec.connections.size() > _opt.max_connections)
        rec.connections.pop_front();
      take_client(rec);
      link_state ls;
      ls.client_id = id;
      ls.seq = rec.connections.back().seq;
      ls.pid = r.remote_node_id;
      ls.transport = r.transport;
      ls.connection = r.connection;
      ls.public_address = pub;
      if (!pub.empty()) {
        _to_locate.push_back({id, ls.seq, pub});
        while (_to_locate.size() > kLocateMax)
          _to_locate.pop_front();
      }
      it = _links.emplace(key, std::move(ls)).first;
      if (rec.banned)
        _to_kick.push_back({r.transport, r.connection});
      rec.dirty = rec.log_dirty = true;
      _dirty = true;
    }
    link_state &ls = it->second;
    ls.holds = r.holds_id;
    ls.secure = r.secure;
    record *rec = find_mut(ls.client_id);
    if (!rec)
      continue;
    if (!rec->active || rec->last_seen != wall)
      rec->dirty = _dirty = true;
    rec->active = true;
    rec->last_seen = wall;
    rec->seen_s = now_s;
    if (r.holds_id) {
      _holders[ls.pid] = key;
      // The device proves its key after its Hello, so its tag can come on a later pass.
      const bool verified = r.device_verified && !r.remote_device_id.empty();
      const std::string tag = verified ? room_device_tag(_opt.room, r.remote_device_id) : "";
      if (rec->verified != verified || rec->device != tag) {
        rec->verified = verified;
        rec->device = tag;
        rec->dirty = _dirty = true;
      }
    }
  }
  // The links gone since the last pass: their connections close. The net engine's are no link
  // report's, and close_engine ends them.
  for (auto it = _links.begin(); it != _links.end();) {
    if (seen.count(it->first) || it->second.engine) {
      ++it;
      continue;
    }
    if (record *rec = find_mut(it->second.client_id)) {
      bool any = false;
      for (connection &c : rec->connections) {
        if (c.seq == it->second.seq && c.active) {
          c.active = false;
          c.closed = wall;
        }
        any = any || c.active;
      }
      // A connection trimmed from the log may still be open: ask the links, not the log.
      for (const auto &[k, ls] : _links)
        if (k != it->first && (seen.count(k) || ls.engine) && ls.client_id == rec->id)
          any = true;
      rec->active = any;
      take_client(*rec);
      rec->dirty = rec->log_dirty = _dirty = true;
    }
    _admins.erase(it->first); // an elevation ends with its connection
    it = _links.erase(it);
  }
  // A pid whose holder went, or no longer holds it (a takeover), is held by no known link.
  // A link that no longer holds its pid loses its elevation with it.
  for (auto h = _holders.begin(); h != _holders.end();) {
    const auto l = _links.find(h->second);
    if (l == _links.end() || !l->second.holds || l->second.pid != h->first)
      h = _holders.erase(h);
    else
      ++h;
  }
  for (auto a = _admins.begin(); a != _admins.end();) {
    const auto l = _links.find(*a);
    if (l == _links.end() || !l->second.holds) {
      if (l != _links.end())
        if (record *rec = find_mut(l->second.client_id))
          rec->dirty = _dirty = true;
      a = _admins.erase(a);
    } else {
      ++a;
    }
  }
}

std::string client_registry::identity_key(const std::string &pid) const {
  const auto h = _holders.find(pid);
  if (h == _holders.end())
    return std::string();
  const auto l = _links.find(h->second);
  return l == _links.end() ? std::string() : l->second.client_id;
}

const client_registry::record *client_registry::find(const std::string &client_id) const {
  const auto it = _records.find(client_id);
  return it == _records.end() ? nullptr : &it->second;
}

client_registry::record *client_registry::find_mut(const std::string &id) {
  const auto it = _records.find(id);
  return it == _records.end() ? nullptr : &it->second;
}

const client_registry::record *client_registry::record_of(const std::string &pid) const {
  const std::string id = identity_key(pid);
  return id.empty() ? nullptr : find(id);
}

client_registry::record *client_registry::record_of_mut(const std::string &pid) {
  const std::string id = identity_key(pid);
  return id.empty() ? nullptr : find_mut(id);
}

bool client_registry::name_taken(const std::string &name, const std::string &self) const {
  const std::string f = folded(name);
  for (const auto &[id, rec] : _records)
    if (id != self && rec.active && folded(rec.username) == f)
      return true;
  return false;
}

std::string client_registry::unique_name(const std::string &name, const std::string &self) const {
  if (!name_taken(name, self))
    return name;
  for (int n = 2;; ++n) {
    const std::string suffix = "-" + std::to_string(n);
    const std::string candidate =
        utf8_prefix(name, kUsernameMax > suffix.size() ? kUsernameMax - suffix.size() : 0) + suffix;
    if (!name_taken(candidate, self))
      return candidate;
  }
}

client_registry::record &client_registry::admit(const std::string &id, double now_s,
                                                const std::string &wall) {
  if (record *rec = find_mut(id)) {
    // A returning client keeps its name, unless an active client took it meanwhile.
    if (!rec->active) {
      rec->username = unique_name(rec->username, id);
      rec->dirty = _dirty = true;
    }
    return *rec;
  }
  if (_records.size() >= _opt.max_clients)
    evict_one();
  record rec;
  rec.id = id;
  rec.username = unique_name("guest-" + id.substr(1, 4), id);
  rec.first_seen = wall;
  rec.last_seen = wall;
  rec.seen_s = now_s;
  _dirty = true;
  return _records.emplace(id, std::move(rec)).first->second;
}

void client_registry::evict_one() {
  // The inactive record seen longest ago. When every record is active, none goes: the cap then
  // bounds only the clients that have left.
  auto victim = _records.end();
  for (auto it = _records.begin(); it != _records.end(); ++it)
    if (!it->second.active &&
        (victim == _records.end() || it->second.seen_s < victim->second.seen_s))
      victim = it;
  if (victim != _records.end()) {
    _records.erase(victim);
    _dirty = true;
  }
}

bool client_registry::set_username(const std::string &pid, const std::string &requested,
                                   double now_s, refusal &why, bool make_unique) {
  record *rec = record_of_mut(pid);
  if (!rec) {
    why = {"unavailable", "this connection has no client record yet"};
    return false;
  }
  std::string name;
  if (!clean_username(requested, name)) {
    why = {"invalid", "a username is 1 to " + std::to_string(kUsernameMax) +
                          " bytes of text, with no control characters"};
    return false;
  }
  if (make_unique)
    name = unique_name(name, rec->id);
  if (name == rec->username)
    return true;
  if (name_taken(name, rec->id)) {
    why = {"invalid", "the username " + name + " is taken"};
    return false;
  }
  while (!rec->renames.empty() && now_s - rec->renames.front() >= kRenameWindowS)
    rec->renames.pop_front();
  if (rec->renames.size() >= kMaxRenames) {
    why = {"rate_limited",
           "a client may rename at most " + std::to_string(kMaxRenames) + " times a minute"};
    return false;
  }
  rec->username = name;
  rec->renames.push_back(now_s);
  rec->dirty = _dirty = true;
  return true;
}

se::value_t client_registry::whoami(const std::string &pid) const {
  const record *rec = record_of(pid);
  if (!rec)
    return se::value_t();
  return se::make_dict({{"client_id", se::value_t(rec->id)},
                        {"username", se::value_t(rec->username)},
                        {"verified", se::value_t(rec->verified)},
                        {"device", se::value_t(rec->device)},
                        {"admin", se::value_t(is_admin(pid))},
                        {"kind", se::value_t(rec->client.kind)},
                        {"capabilities", strings_value(rec->client.capabilities)}});
}

void client_registry::write(cvc::app &app, const std::set<std::string> &present) {
  for (const std::string &pid : present) {
    const record *rec = record_of(pid);
    if (!rec)
      continue;
    put(app, join(join(room_participant_path(_opt.room, pid), "profile"), "name"), rec->username);
  }
  if (_dirty)
    mirror(app);
}

void client_registry::write_name(cvc::app &app, const std::string &pid) {
  if (const record *rec = record_of(pid))
    put(app, join(join(room_participant_path(_opt.room, pid), "profile"), "name"), rec->username);
}

void client_registry::mirror(cvc::app &app) {
  const std::string root = kRoot;
  for (auto it = _mirrored.begin(); it != _mirrored.end();) {
    if (_records.count(*it)) {
      ++it;
      continue;
    }
    remove_child(app, root, *it); // an evicted record
    _mirrored_seqs.erase(*it);
    it = _mirrored.erase(it);
  }
  for (auto &[id, rec] : _records) {
    if (!rec.dirty && !rec.log_dirty)
      continue;
    const std::string base = join(root, id);
    _mirrored.insert(id);
    if (rec.dirty) {
      put(app, join(base, "username"), rec.username);
      put(app, join(base, "device"), rec.device);
      put(app, join(base, "verified"), rec.verified ? "1" : "0");
      put(app, join(base, "first_seen"), rec.first_seen);
      put(app, join(base, "last_seen"), rec.last_seen);
      put(app, join(base, "active"), rec.active ? "1" : "0");
      put(app, join(base, "admin"), record_admin(id) ? "1" : "0");
      put(app, join(base, "banned"), rec.banned ? "1" : "0");
      put(app, join(base, "rcon_failures"), std::to_string(rec.rcon_failures));
      put(app, join(base, "kind"), rec.client.kind);
      put(app, join(base, "capabilities"), comma_list(rec.client.capabilities));
      // Only the limits set, each under limits.<key>.
      const std::string lb = join(base, "limits");
      const client_limits &l = rec.limits;
      const auto limit = [&](const char *k, bool set, const std::string &text) {
        if (set)
          put(app, join(lb, k), text);
        else
          remove_child(app, lb, k);
      };
      limit("programs", l.programs.has_value(), l.programs.value_or(false) ? "true" : "false");
      limit("guests", l.guests.has_value(), l.guests.value_or(false) ? "true" : "false");
      limit("steps", l.steps != 0, std::to_string(l.steps));
      limit("time", l.time > 0.0, seconds_text(l.time));
      limit("memory_mb", l.memory_mb != 0, std::to_string(l.memory_mb));
      rec.dirty = false;
    }
    if (!rec.log_dirty)
      continue;
    rec.log_dirty = false;
    const std::string conns = join(base, "connections");
    std::set<std::uint64_t> now;
    for (const connection &c : rec.connections) {
      const std::string cb = join(conns, "n" + std::to_string(c.seq));
      put(app, join(cb, "pid"), c.pid);
      put(app, join(cb, "transport"), c.transport);
      put(app, join(cb, "public"), c.public_address);
      put(app, join(cb, "local"), c.local_address);
      put(app, join(cb, "opened"), c.opened);
      put(app, join(cb, "closed"), c.closed);
      put(app, join(cb, "active"), c.active ? "1" : "0");
      put(app, join(cb, "engine"), c.engine ? "1" : "0");
      put(app, join(cb, "key"), c.key);
      const std::string gb = join(cb, "geo");
      for (const auto &[k, v] :
           {std::pair<const char *, const std::string *>{"country", &c.geo.country},
            {"region", &c.geo.region},
            {"city", &c.geo.city},
            {"asn", &c.geo.asn}}) {
        if (v->empty())
          remove_child(app, gb, k);
        else
          put(app, join(gb, k), *v);
      }
      put(app, join(cb, "kind"), c.client.kind);
      put(app, join(cb, "version"), c.client.version);
      put(app, join(cb, "capabilities"), comma_list(c.client.capabilities));
      now.insert(c.seq);
    }
    std::set<std::uint64_t> &had = _mirrored_seqs[id];
    for (const std::uint64_t seq : had)
      if (!now.count(seq))
        remove_child(app, conns, "n" + std::to_string(seq)); // trimmed from the log
    had = std::move(now);
  }
  _dirty = false;
}

bool client_registry::is_admin(const std::string &pid) const {
  const auto h = _holders.find(pid);
  return h != _holders.end() && _admins.count(h->second) != 0;
}

bool client_registry::record_admin(const std::string &id) const {
  for (const std::string &key : _admins) {
    const auto l = _links.find(key);
    if (l != _links.end() && l->second.client_id == id)
      return true;
  }
  return false;
}

bool client_registry::set_admin(const std::string &pid, bool on) {
  const auto h = _holders.find(pid);
  if (h == _holders.end())
    return false;
  if (on)
    _admins.insert(h->second);
  else
    _admins.erase(h->second);
  if (record *rec = record_of_mut(pid))
    rec->dirty = _dirty = true;
  return true;
}

void client_registry::note_rcon_failure(const std::string &pid) {
  if (record *rec = record_of_mut(pid)) {
    ++rec->rcon_failures;
    rec->dirty = _dirty = true;
  }
}

bool client_registry::holder_of(const std::string &pid, holder &out) const {
  const auto h = _holders.find(pid);
  if (h == _holders.end())
    return false;
  const auto l = _links.find(h->second);
  if (l == _links.end())
    return false;
  out.public_address = l->second.public_address;
  out.secure = l->second.secure;
  return true;
}

std::vector<client_registry::link_ref>
client_registry::links_of(const std::string &id_or_pid) const {
  std::string id = id_or_pid;
  if (!find(id))
    id = identity_key(id_or_pid);
  std::vector<link_ref> out;
  if (id.empty())
    return out;
  for (const auto &[key, ls] : _links)
    if (ls.client_id == id)
      out.push_back({ls.transport, ls.engine ? 0 : ls.connection, ls.key});
  return out;
}

bool client_registry::ban(const std::string &id, bool on) {
  record *rec = find_mut(id);
  if (!rec)
    return false;
  if (rec->banned != on)
    rec->dirty = _dirty = true;
  rec->banned = on;
  return true;
}

std::vector<client_registry::link_ref> client_registry::take_banned_links() {
  std::vector<link_ref> out;
  out.swap(_to_kick);
  return out;
}

bool client_registry::rename(const std::string &id, const std::string &name, refusal &why) {
  record *rec = find_mut(id);
  if (!rec) {
    why = {"invalid", "no client " + id};
    return false;
  }
  std::string clean;
  if (!clean_username(name, clean)) {
    why = {"invalid", "a username is 1 to " + std::to_string(kUsernameMax) +
                          " bytes of text, with no control characters"};
    return false;
  }
  if (clean != rec->username && name_taken(clean, id)) {
    why = {"invalid", "the username " + clean + " is taken"};
    return false;
  }
  if (clean != rec->username)
    rec->dirty = _dirty = true;
  rec->username = clean;
  return true;
}

bool client_registry::set_limit(const std::string &id, const std::string &key,
                                const se::value_t &value, refusal &why) {
  record *rec = find_mut(id);
  if (!rec) {
    why = {"invalid", "no client " + id};
    return false;
  }
  const bool nil = std::holds_alternative<std::monostate>(value.v);
  const bool *b = std::get_if<bool>(&value.v);
  const std::int64_t *i = std::get_if<std::int64_t>(&value.v);
  const double *d = std::get_if<double>(&value.v);
  client_limits &l = rec->limits;
  if (key == "programs" || key == "guests") {
    // The DSL's false is nil, so nil takes the limit back; a false given as a boolean (from
    // Python, or (= 1 0)) refuses the kind.
    if (!nil && !b) {
      why = {"invalid", key + " is true or false"};
      return false;
    }
    std::optional<bool> &flag = key == "programs" ? l.programs : l.guests;
    flag = nil ? std::nullopt : std::optional<bool>(*b);
  } else if (key == "steps" || key == "memory_mb") {
    if (!nil && (!i || *i < 0)) {
      why = {"invalid", key + " is a whole number, not below 0"};
      return false;
    }
    (key == "steps" ? l.steps : l.memory_mb) = nil ? 0 : static_cast<std::uint64_t>(*i);
  } else if (key == "time") {
    const double s = i ? static_cast<double>(*i) : d ? *d : -1.0;
    if (!nil && !(s >= 0.0)) {
      why = {"invalid", "time is a number of seconds, not below 0"};
      return false;
    }
    l.time = nil ? 0.0 : s;
  } else {
    why = {"invalid", key + " is not a limit (programs, guests, steps, time, memory_mb)"};
    return false;
  }
  rec->dirty = _dirty = true;
  return true;
}

std::optional<client_registry::client_limits>
client_registry::limits(const std::string &pid) const {
  const record *rec = record_of(pid);
  if (!rec || rec->limits.empty())
    return std::nullopt;
  return rec->limits;
}

se::value_t client_registry::summary(const record &rec) const {
  return se::make_dict({{"client_id", se::value_t(rec.id)},
                        {"username", se::value_t(rec.username)},
                        {"active", se::value_t(rec.active)},
                        {"admin", se::value_t(record_admin(rec.id))},
                        {"banned", se::value_t(rec.banned)},
                        {"verified", se::value_t(rec.verified)},
                        {"device", se::value_t(rec.device)},
                        {"first_seen", se::value_t(rec.first_seen)},
                        {"last_seen", se::value_t(rec.last_seen)},
                        {"kind", se::value_t(rec.client.kind)},
                        {"capabilities", strings_value(rec.client.capabilities)}});
}

se::value_t client_registry::list_value() const {
  std::vector<const record *> recs;
  for (const auto &[id, rec] : _records)
    recs.push_back(&rec);
  std::stable_sort(recs.begin(), recs.end(), [](const record *a, const record *b) {
    if (a->active != b->active)
      return a->active;
    return a->seen_s > b->seen_s;
  });
  std::vector<se::value_t> out;
  for (const record *rec : recs)
    out.push_back(summary(*rec));
  return se::make_list(std::move(out));
}

se::value_t client_registry::record_value(const std::string &id) const {
  const record *rec = find(id);
  if (!rec)
    return se::value_t();
  se::value_t v = summary(*rec);
  std::vector<se::value_t> conns;
  for (const connection &c : rec->connections)
    conns.push_back(se::make_dict({{"pid", se::value_t(c.pid)},
                                   {"transport", se::value_t(c.transport)},
                                   {"public", se::value_t(c.public_address)},
                                   {"local", se::value_t(c.local_address)},
                                   {"opened", se::value_t(c.opened)},
                                   {"closed", se::value_t(c.closed)},
                                   {"active", se::value_t(c.active)},
                                   {"engine", se::value_t(c.engine)},
                                   {"key", se::value_t(c.key)},
                                   {"geo", se::make_dict({{"country", se::value_t(c.geo.country)},
                                                          {"region", se::value_t(c.geo.region)},
                                                          {"city", se::value_t(c.geo.city)},
                                                          {"asn", se::value_t(c.geo.asn)}})},
                                   {"kind", se::value_t(c.client.kind)},
                                   {"version", se::value_t(c.client.version)},
                                   {"capabilities", strings_value(c.client.capabilities)}}));
  se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v);
  (*d)->emplace_back("limits", limits_value(rec->limits));
  (*d)->emplace_back("rcon_failures", se::value_t(static_cast<std::int64_t>(rec->rcon_failures)));
  (*d)->emplace_back("connections", se::make_list(std::move(conns)));
  return v;
}

double rcon_guard::wait(const std::string &address, const std::string &client, double now_s) const {
  double w = 0.0;
  for (const std::string &key : {"a:" + address, "c:" + client}) {
    const auto it = _by.find(key);
    if (it != _by.end() && it->second.locked_until > now_s)
      w = std::max(w, it->second.locked_until - now_s);
  }
  return w;
}

void rcon_guard::fail_one(const std::string &key, double now_s) {
  if (_by.size() >= kMaxEntries && !_by.count(key)) {
    // Full: forget every entry that holds no lock now, so a flood of addresses can't grow it.
    for (auto it = _by.begin(); it != _by.end();)
      it = it->second.locked_until <= now_s ? _by.erase(it) : std::next(it);
    if (_by.size() >= kMaxEntries)
      return;
  }
  entry &e = _by[key];
  ++e.failures;
  e.last_s = now_s;
  if (e.failures >= kFreeTries) {
    e.lock_s = e.lock_s == 0.0 ? kFirstLockS : std::min(kMaxLockS, e.lock_s * 2.0);
    e.locked_until = now_s + e.lock_s;
  }
}

void rcon_guard::failed(const std::string &address, const std::string &client, double now_s) {
  fail_one("a:" + address, now_s);
  if (!client.empty())
    fail_one("c:" + client, now_s);
}

void rcon_guard::succeeded(const std::string &address, const std::string &client) {
  _by.erase("a:" + address);
  _by.erase("c:" + client);
}

// -------- a program's connections (ID5b) --------

std::string client_registry::engine_key(const std::string &key) { return "program#" + key; }

bool client_registry::valid_program_key(const std::string &key) {
  if (key.empty() || key.size() > 128)
    return false;
  for (const char ch : key)
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
          ch == '.' || ch == '_' || ch == ':' || ch == '-'))
      return false;
  return true;
}

void client_registry::take_client(record &rec) {
  for (auto it = rec.connections.rbegin(); it != rec.connections.rend(); ++it)
    if (it->active) {
      rec.client = it->client;
      return;
    }
  if (!rec.connections.empty())
    rec.client = rec.connections.back().client;
}

namespace {
bool client_word(const std::string &s, std::size_t max) {
  if (s.empty() || s.size() > max)
    return false;
  for (const char ch : s)
    if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' ||
          ch == '-'))
      return false;
  return true;
}

// At most MAX printable ASCII bytes of IN (a version, a geo field).
std::string printable(const std::string &in, std::size_t max) {
  std::string out;
  for (const char ch : in) {
    if (out.size() >= max)
      break;
    if (ch >= 0x20 && ch < 0x7f)
      out.push_back(ch);
  }
  return out;
}
} // namespace

client_registry::client_info client_registry::clean_client(const client_info &in) {
  client_info out;
  out.kind = client_word(in.kind, 32) ? in.kind : "unknown";
  out.version = printable(in.version, 64);
  for (const std::string &c : in.capabilities) {
    if (out.capabilities.size() >= 32)
      break;
    if (client_word(c, 64) &&
        std::find(out.capabilities.begin(), out.capabilities.end(), c) == out.capabilities.end())
      out.capabilities.push_back(c);
  }
  return out;
}

std::string client_registry::open_engine(const std::string &key, const std::string &transport,
                                         const std::string &public_address,
                                         const std::string &local_address,
                                         const client_info &client, const geo_info &geo,
                                         double now_s, const std::string &wall, refusal &why) {
  if (!valid_program_key(key) || _links.count(engine_key(key))) {
    why = {"invalid", valid_program_key(key) ? "that connection is open already"
                                             : "a key is 1-128 bytes of [A-Za-z0-9._:-]"};
    return std::string();
  }
  if (!client_word(transport, 32)) {
    why = {"invalid", "a transport is a short lowercase name"};
    return std::string();
  }
  const std::string pub = address_host(public_address);
  const std::string id = client_id(std::string(), local_address, pub);
  if (const record *known = find(id); known && known->banned) {
    why = {"forbidden", "this client is banned"};
    return std::string();
  }
  record &rec = admit(id, now_s, wall);
  connection c;
  c.seq = rec.next_seq++;
  c.transport = transport;
  c.public_address = pub;
  c.local_address = local_address;
  c.opened = wall;
  c.active = true;
  c.engine = true;
  c.key = key;
  c.client = clean_client(client);
  c.geo = {printable(geo.country, 64), printable(geo.region, 64), printable(geo.city, 64),
           printable(geo.asn, 64)};
  rec.connections.push_back(std::move(c));
  while (rec.connections.size() > _opt.max_connections)
    rec.connections.pop_front();
  link_state ls;
  ls.client_id = id;
  ls.seq = rec.connections.back().seq;
  ls.transport = transport;
  ls.public_address = pub;
  ls.secure = transport == "nmdcs" || transport == "tls";
  ls.engine = true;
  ls.key = key;
  _links.emplace(engine_key(key), std::move(ls));
  rec.active = true;
  rec.last_seen = wall;
  rec.seen_s = now_s;
  take_client(rec);
  rec.dirty = rec.log_dirty = _dirty = true;
  return id;
}

bool client_registry::name_engine(const std::string &key, const std::string &nick, refusal &why) {
  const auto l = _links.find(engine_key(key));
  record *rec = l == _links.end() ? nullptr : find_mut(l->second.client_id);
  if (!rec) {
    why = {"unavailable", "no such connection"};
    return false;
  }
  std::string name;
  if (!clean_username(nick, name, kProgramUsernameMax)) {
    why = {"invalid", "that is not a name"};
    return false;
  }
  if (name == rec->username)
    return true;
  if (name_taken(name, rec->id)) {
    why = {"invalid", "another client has that name"};
    return false;
  }
  rec->username = name;
  rec->dirty = _dirty = true;
  return true;
}

bool client_registry::update_engine(const std::string &key, const client_info &client) {
  const auto l = _links.find(engine_key(key));
  record *rec = l == _links.end() ? nullptr : find_mut(l->second.client_id);
  if (!rec)
    return false;
  for (connection &c : rec->connections)
    if (c.seq == l->second.seq)
      c.client = clean_client(client);
  take_client(*rec);
  rec->dirty = rec->log_dirty = _dirty = true;
  return true;
}

std::vector<client_registry::to_locate> client_registry::take_to_locate() {
  std::vector<to_locate> out(std::make_move_iterator(_to_locate.begin()),
                             std::make_move_iterator(_to_locate.end()));
  _to_locate.clear();
  return out;
}

bool client_registry::set_geo(const std::string &client_id, std::uint64_t seq,
                              const geo_info &geo) {
  record *rec = find_mut(client_id);
  if (!rec)
    return false;
  for (connection &c : rec->connections)
    if (c.seq == seq) {
      c.geo = {printable(geo.country, 64), printable(geo.region, 64), printable(geo.city, 64),
               printable(geo.asn, 64)};
      rec->log_dirty = _dirty = true;
      return true;
    }
  return false;
}

bool client_registry::close_engine(const std::string &key, const std::string &wall) {
  const auto l = _links.find(engine_key(key));
  if (l == _links.end())
    return false;
  if (record *rec = find_mut(l->second.client_id)) {
    for (connection &c : rec->connections)
      if (c.seq == l->second.seq && c.active) {
        c.active = false;
        c.closed = wall;
      }
    bool any = false;
    for (const auto &[k, ls] : _links)
      if (k != l->first && ls.client_id == rec->id)
        any = true;
    rec->active = any;
    take_client(*rec);
    rec->dirty = rec->log_dirty = _dirty = true;
  }
  _links.erase(l);
  return true;
}

bool client_registry::clean_username(const std::string &in, std::string &out, std::size_t max) {
  std::size_t b = 0, e = in.size();
  while (b < e && std::isspace(static_cast<unsigned char>(in[b])))
    ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(in[e - 1])))
    --e;
  std::string s = in.substr(b, e - b);
  if (s.empty() || s.size() > max || !clean_utf8(s))
    return false;
  out = std::move(s);
  return true;
}

std::string client_registry::address_host(const std::string &address) {
  std::string s = address;
  for (const char *scheme : {"ipv4:", "ipv6:"})
    if (s.rfind(scheme, 0) == 0) {
      s.erase(0, std::char_traits<char>::length(scheme));
      break;
    }
  // gRPC escapes the brackets of an IPv6 peer ("ipv6:%5B::1%5D:50051").
  for (const auto &[from, to] : {std::pair<const char *, const char *>{"%5B", "["},
                                 {"%5D", "]"},
                                 {"%5b", "["},
                                 {"%5d", "]"},
                                 {"%25", "%"}}) {
    for (std::size_t at = s.find(from); at != std::string::npos; at = s.find(from, at))
      s.replace(at, std::char_traits<char>::length(from), to);
  }
  if (!s.empty() && s.front() == '[') {
    const std::size_t close = s.find(']');
    return close == std::string::npos ? s : s.substr(1, close - 1);
  }
  // One colon: host:port. More: a bare IPv6 address, with no port to drop.
  if (std::count(s.begin(), s.end(), ':') == 1)
    return s.substr(0, s.find(':'));
  return s;
}

std::string client_registry::utc_iso(std::chrono::system_clock::time_point tp) {
  const std::time_t t = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};
#if defined(_WIN32)
  if (gmtime_s(&tm, &t) != 0)
    return std::string();
#else
  if (!gmtime_r(&t, &tm))
    return std::string();
#endif
  char buf[32];
  std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

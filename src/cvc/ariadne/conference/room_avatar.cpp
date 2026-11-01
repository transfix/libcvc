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
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_avatar.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/core/app.h>
#include <cvc/state/state.h> // cvc::state, SEPARATOR, findDescendant, children
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

// A path BASE.<segs...>, joined with cvc::state::SEPARATOR.
std::string sub(const std::string &base, std::initializer_list<std::string> segs) {
  std::string s = base;
  for (const std::string &seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

void check_id(const std::string &id) {
  if (id.empty() || id.find(cvc::state::SEPARATOR) != std::string::npos)
    throw std::invalid_argument("avatar option id must be non-empty and contain no '" +
                                std::string(cvc::state::SEPARATOR) + "': '" + id + "'");
}

// Shortest text that recovers the float exactly (9 significant digits round-trip a binary32).
std::string fmt(float f) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(f));
  return std::string(buf);
}

// touch-before-value, so a just-created node's first write is not lost to the distributed-state
// adapter's attach race (the room tree's set_replicated idiom).
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  n.touch();
  n.value(val);
}
void set_f(cvc::app &app, const std::string &path, float f) { set_replicated(app, path, fmt(f)); }
void set_b(cvc::app &app, const std::string &path, bool b) {
  set_replicated(app, path, b ? "1" : "0");
}
void set_v3(cvc::app &app, const std::string &path, const float v[3], const char *a = "x",
            const char *b = "y", const char *c = "z") {
  set_f(app, sub(path, {a}), v[0]);
  set_f(app, sub(path, {b}), v[1]);
  set_f(app, sub(path, {c}), v[2]);
}

cvc::state *find(cvc::app &app, const std::string &path) { return app.root().findDescendant(path); }
std::string get_s(cvc::app &app, const std::string &path, const std::string &def = {}) {
  cvc::state *n = find(app, path);
  return n ? n->value() : def;
}
float get_f(cvc::app &app, const std::string &path, float def) {
  cvc::state *n = find(app, path);
  return n && !n->value().empty() ? std::strtof(n->value().c_str(), nullptr) : def;
}
bool get_b(cvc::app &app, const std::string &path, bool def) {
  cvc::state *n = find(app, path);
  return n && !n->value().empty() ? n->value() == "1" : def;
}
void get_v3(cvc::app &app, const std::string &path, float v[3], const char *a = "x",
            const char *b = "y", const char *c = "z") {
  v[0] = get_f(app, sub(path, {a}), v[0]);
  v[1] = get_f(app, sub(path, {b}), v[1]);
  v[2] = get_f(app, sub(path, {c}), v[2]);
}

// The immediate children of PATH (names only). cvc::state::children() is a DEEP pre-order walk of
// full paths, so keep the entries exactly one segment below PATH (as room_roster does).
std::vector<std::string> child_ids(cvc::app &app, const std::string &path) {
  std::vector<std::string> out;
  cvc::state *n = find(app, path);
  if (!n)
    return out;
  const std::string prefix = path + cvc::state::SEPARATOR;
  for (const std::string &full : n->children()) {
    if (full.rfind(prefix, 0) != 0)
      continue;
    std::string rel = full.substr(prefix.size());
    if (rel.empty() || rel.find(cvc::state::SEPARATOR) != std::string::npos)
      continue;
    out.push_back(std::move(rel));
  }
  return out;
}

std::string colors_path(const std::string &room) {
  return sub(room_avatars_path(room), {"colors"});
}
std::string accessories_path(const std::string &room) {
  return sub(room_avatars_path(room), {"accessories"});
}
std::string choice_path(const std::string &room, const std::string &pid) {
  return sub(room_participant_path(room, pid), {"avatar"});
}

// The ids under PATH that are present, in their written order.
std::vector<std::string> present_in_order(cvc::app &app, const std::string &path) {
  std::vector<std::pair<int, std::string>> ids;
  for (std::string &id : child_ids(app, path)) {
    const std::string base = sub(path, {id});
    if (!get_b(app, sub(base, {"present"}), false))
      continue;
    const int order = static_cast<int>(get_f(app, sub(base, {"order"}), 0.0f));
    ids.emplace_back(order, std::move(id));
  }
  std::sort(ids.begin(), ids.end());
  std::vector<std::string> out;
  for (auto &p : ids)
    out.push_back(std::move(p.second));
  return out;
}

// Soft-remove (present = 0) the ids under PATH that KEEP does not name.
void retire_others(cvc::app &app, const std::string &path, const std::set<std::string> &keep) {
  for (const std::string &id : child_ids(app, path))
    if (!keep.count(id) && get_b(app, sub(path, {id, "present"}), false))
      set_b(app, sub(path, {id, "present"}), false);
}

void write_part(cvc::app &app, const std::string &base, const avatar_part &p) {
  set_replicated(app, sub(base, {"shape"}), p.shape);
  set_v3(app, sub(base, {"size"}), p.size);
  set_f(app, sub(base, {"radius"}), p.radius);
  set_v3(app, sub(base, {"pos"}), p.pos);
  set_replicated(app, sub(base, {"tint"}), p.tint);
  set_v3(app, sub(base, {"color"}), p.color, "r", "g", "b");
}

avatar_part read_part(cvc::app &app, const std::string &base) {
  avatar_part p;
  p.shape = get_s(app, sub(base, {"shape"}), p.shape);
  get_v3(app, sub(base, {"size"}), p.size);
  p.radius = get_f(app, sub(base, {"radius"}), p.radius);
  get_v3(app, sub(base, {"pos"}), p.pos);
  p.tint = get_s(app, sub(base, {"tint"}), p.tint);
  get_v3(app, sub(base, {"color"}), p.color, "r", "g", "b");
  return p;
}

avatar_part box(float sx, float sy, float sz, float x, float y, float z, const char *tint,
                float r = 1.0f, float g = 1.0f, float b = 1.0f) {
  avatar_part p;
  p.shape = "box";
  p.size[0] = sx, p.size[1] = sy, p.size[2] = sz;
  p.pos[0] = x, p.pos[1] = y, p.pos[2] = z;
  p.tint = tint;
  p.color[0] = r, p.color[1] = g, p.color[2] = b;
  return p;
}

avatar_part ball(float radius, float x, float y, float z, const char *tint, float r = 1.0f,
                 float g = 1.0f, float b = 1.0f) {
  avatar_part p;
  p.shape = "sphere";
  p.radius = radius;
  p.pos[0] = x, p.pos[1] = y, p.pos[2] = z;
  p.tint = tint;
  p.color[0] = r, p.color[1] = g, p.color[2] = b;
  return p;
}

} // namespace

const avatar_color_option *avatar_catalog::find_color(const std::string &id_or_label) const {
  for (const avatar_color_option &c : colors)
    if (c.id == id_or_label)
      return &c;
  for (const avatar_color_option &c : colors)
    if (!id_or_label.empty() && c.label == id_or_label)
      return &c;
  return nullptr;
}

const avatar_accessory_option *
avatar_catalog::find_accessory(const std::string &id_or_label) const {
  for (const avatar_accessory_option &a : accessories)
    if (a.id == id_or_label)
      return &a;
  for (const avatar_accessory_option &a : accessories)
    if (!id_or_label.empty() && a.label == id_or_label)
      return &a;
  return nullptr;
}

std::string room_avatars_path(const std::string &room_id) {
  return sub("conf", {"rooms", room_id, "avatars"});
}

std::uint64_t avatar_catalog_revision(cvc::app &app, const std::string &room_id) {
  const std::string v = get_s(app, sub(room_avatars_path(room_id), {"rev"}));
  return v.empty() ? 0 : std::strtoull(v.c_str(), nullptr, 10);
}

void write_avatar_catalog(cvc::app &app, const std::string &room_id, const avatar_catalog &c) {
  for (const avatar_color_option &o : c.colors)
    check_id(o.id);
  for (const avatar_accessory_option &o : c.accessories)
    check_id(o.id);

  std::set<std::string> keep;
  int order = 0;
  for (const avatar_color_option &o : c.colors) {
    const std::string base = sub(colors_path(room_id), {o.id});
    set_f(app, sub(base, {"order"}), static_cast<float>(order++));
    set_replicated(app, sub(base, {"label"}), o.label);
    set_b(app, sub(base, {"auto"}), o.automatic);
    set_v3(app, sub(base, {"color"}), o.color, "r", "g", "b");
    set_b(app, sub(base, {"present"}), true);
    keep.insert(o.id);
  }
  retire_others(app, colors_path(room_id), keep);

  keep.clear();
  order = 0;
  for (const avatar_accessory_option &o : c.accessories) {
    const std::string base = sub(accessories_path(room_id), {o.id});
    set_f(app, sub(base, {"order"}), static_cast<float>(order++));
    set_replicated(app, sub(base, {"label"}), o.label);
    for (std::size_t i = 0; i < o.parts.size(); ++i)
      write_part(app, sub(base, {"parts", std::to_string(i)}), o.parts[i]);
    // The count bounds what a reader takes, so parts left over from a longer earlier version are
    // never read.
    set_replicated(app, sub(base, {"parts", "count"}), std::to_string(o.parts.size()));
    set_b(app, sub(base, {"present"}), true);
    keep.insert(o.id);
  }
  retire_others(app, accessories_path(room_id), keep);

  // Written last, so a reader that sees the new revision also sees what it covers.
  set_replicated(
      app, sub(room_avatars_path(room_id), {"rev"}),
      std::to_string(static_cast<unsigned long long>(avatar_catalog_revision(app, room_id) + 1)));
}

avatar_catalog read_avatar_catalog(cvc::app &app, const std::string &room_id) {
  avatar_catalog c;
  for (const std::string &id : present_in_order(app, colors_path(room_id))) {
    const std::string base = sub(colors_path(room_id), {id});
    avatar_color_option o;
    o.id = id;
    o.label = get_s(app, sub(base, {"label"}), id);
    o.automatic = get_b(app, sub(base, {"auto"}), false);
    get_v3(app, sub(base, {"color"}), o.color, "r", "g", "b");
    c.colors.push_back(std::move(o));
  }
  for (const std::string &id : present_in_order(app, accessories_path(room_id))) {
    const std::string base = sub(accessories_path(room_id), {id});
    avatar_accessory_option o;
    o.id = id;
    o.label = get_s(app, sub(base, {"label"}), id);
    const long count =
        std::strtol(get_s(app, sub(base, {"parts", "count"}), "0").c_str(), nullptr, 10);
    for (long i = 0; i < count && i < static_cast<long>(kMaxAccessoryParts); ++i)
      o.parts.push_back(read_part(app, sub(base, {"parts", std::to_string(i)})));
    c.accessories.push_back(std::move(o));
  }
  return c;
}

avatar_catalog default_avatar_catalog() {
  avatar_catalog c;
  avatar_color_option a;
  a.id = "auto";
  a.label = "Automatic";
  a.automatic = true;
  c.colors.push_back(a);
  const struct {
    const char *id, *label;
    float r, g, b;
  } named[] = {
      {"gold", "Gold", 0.95f, 0.75f, 0.20f},     {"crimson", "Crimson", 0.80f, 0.15f, 0.20f},
      {"orange", "Orange", 0.95f, 0.50f, 0.15f}, {"lime", "Lime", 0.55f, 0.85f, 0.25f},
      {"teal", "Teal", 0.15f, 0.65f, 0.65f},     {"sky", "Sky", 0.35f, 0.65f, 0.95f},
      {"violet", "Violet", 0.55f, 0.35f, 0.85f}, {"pink", "Pink", 0.95f, 0.50f, 0.70f},
      {"slate", "Slate", 0.40f, 0.45f, 0.52f},   {"ivory", "Ivory", 0.92f, 0.90f, 0.84f},
  };
  for (const auto &n : named) {
    avatar_color_option o;
    o.id = n.id;
    o.label = n.label;
    o.color[0] = n.r, o.color[1] = n.g, o.color[2] = n.b;
    c.colors.push_back(o);
  }

  // Heights from the feet: the body's top is at kAvatarHeight (1.7 m), the visor's front at
  // x = 0.25 + 0.07. Every part stays inside the envelope (|x|, |y| <= 0.4, z <= 2.0).
  const float top = kAvatarHeight;
  c.accessories.push_back({"none", "None", {}});
  c.accessories.push_back({"cap",
                           "Cap",
                           {box(0.52f, 0.52f, 0.08f, 0.0f, 0.0f, top + 0.04f, "visor"),
                            box(0.14f, 0.40f, 0.03f, 0.32f, 0.0f, top + 0.015f, "visor")}});
  c.accessories.push_back({"tophat",
                           "Top hat",
                           {box(0.60f, 0.60f, 0.03f, 0.0f, 0.0f, top + 0.015f, "visor"),
                            box(0.38f, 0.38f, 0.26f, 0.0f, 0.0f, top + 0.16f, "visor")}});
  c.accessories.push_back({"antenna",
                           "Antenna",
                           {box(0.03f, 0.03f, 0.20f, 0.0f, 0.0f, top + 0.10f, "visor"),
                            ball(0.05f, 0.0f, 0.0f, top + 0.23f, "rgb", 0.95f, 0.25f, 0.20f)}});
  c.accessories.push_back(
      {"crest",
       "Crest",
       {box(0.44f, 0.06f, 0.14f, 0.0f, 0.0f, top + 0.07f, "rgb", 0.92f, 0.92f, 0.95f)}});
  c.accessories.push_back(
      {"halo",
       "Halo",
       {box(0.44f, 0.05f, 0.03f, 0.0f, 0.2f, top + 0.25f, "rgb", 0.98f, 0.85f, 0.35f),
        box(0.44f, 0.05f, 0.03f, 0.0f, -0.2f, top + 0.25f, "rgb", 0.98f, 0.85f, 0.35f),
        box(0.05f, 0.34f, 0.03f, 0.2f, 0.0f, top + 0.25f, "rgb", 0.98f, 0.85f, 0.35f),
        box(0.05f, 0.34f, 0.03f, -0.2f, 0.0f, top + 0.25f, "rgb", 0.98f, 0.85f, 0.35f)}});
  c.accessories.push_back({"backpack",
                           "Backpack",
                           {box(0.12f, 0.38f, 0.50f, -0.31f, 0.0f, 1.10f, "visor"),
                            box(0.04f, 0.30f, 0.12f, -0.38f, 0.0f, 0.98f, "body")}});
  return c;
}

bool seed_avatar_catalog(cvc::app &app, const std::string &room_id) {
  if (avatar_catalog_revision(app, room_id) != 0)
    return false;
  write_avatar_catalog(app, room_id, default_avatar_catalog());
  return true;
}

avatar_choice read_avatar_choice(cvc::app &app, const std::string &room_id,
                                 const std::string &pid) {
  avatar_choice c;
  const std::string base = choice_path(room_id, pid);
  c.color = get_s(app, sub(base, {"color"}));
  c.accessory = get_s(app, sub(base, {"accessory"}));
  return c;
}

void write_avatar_choice(cvc::app &app, const std::string &room_id, const std::string &pid,
                         const avatar_choice &choice) {
  const std::string base = choice_path(room_id, pid);
  set_replicated(app, sub(base, {"color"}), choice.color);
  set_replicated(app, sub(base, {"accessory"}), choice.accessory);
}

void auto_avatar_color(const std::string &pid, float out[3]) {
  std::uint32_t h = 2166136261u; // FNV-1a: the same pid colours the same on every client
  for (unsigned char c : pid)
    h = (h ^ c) * 16777619u;
  // A hue in [0, 360) that skips 30-75 degrees, the oranges and yellows of the Gold option, so an
  // automatic colour is not mistaken for someone who chose it. (The same hues the markers had when
  // only peers were coloured this way.)
  const std::uint32_t deg = h % 315u;
  const float hue = static_cast<float>(deg < 30u ? deg : deg + 45u) / 60.0f; // [0, 6)
  const float s = 0.55f, v = 0.85f, c = v * s;
  const float x = c * (1.0f - std::fabs(std::fmod(hue, 2.0f) - 1.0f));
  float r = 0, g = 0, b = 0;
  switch (static_cast<int>(hue)) {
  case 0:
    r = c, g = x;
    break;
  case 1:
    r = x, g = c;
    break;
  case 2:
    g = c, b = x;
    break;
  case 3:
    g = x, b = c;
    break;
  case 4:
    r = x, b = c;
    break;
  default:
    r = c, b = x;
    break;
  }
  out[0] = r + (v - c), out[1] = g + (v - c), out[2] = b + (v - c);
}

namespace {

std::string policy_path(const std::string &room) {
  return sub(room_avatars_path(room), {"policy"});
}
std::string reserve_path(const std::string &room, const std::string &pid) {
  return sub(room_participant_path(room, pid), {"energy"});
}

} // namespace

void write_avatar_policy(cvc::app &app, const std::string &room_id, const avatar_energy_policy &p) {
  const std::string e = sub(policy_path(room_id), {"energy"});
  set_b(app, sub(e, {"enabled"}), p.enabled);
  set_replicated(app, sub(e, {"force_max"}), fmt(static_cast<float>(p.force_max)));
  set_replicated(app, sub(e, {"power_max"}), fmt(static_cast<float>(p.power_max)));
  set_replicated(app, sub(e, {"power_rest"}), fmt(static_cast<float>(p.power_rest)));
  set_replicated(app, sub(e, {"reserve_max"}), fmt(static_cast<float>(p.reserve_max)));
  set_replicated(app, sub(e, {"transport_cost"}), fmt(static_cast<float>(p.transport_cost)));
  set_replicated(app, sub(e, {"empty"}),
                 p.empty == avatar_energy_policy::when_empty::stop ? "stop" : "rest");
  set_replicated(app, sub(e, {"recover_fraction"}), fmt(static_cast<float>(p.recover_fraction)));
  // Written last, so a reader that sees the new revision also sees what it covers.
  set_replicated(
      app, sub(policy_path(room_id), {"rev"}),
      std::to_string(static_cast<unsigned long long>(avatar_policy_revision(app, room_id) + 1)));
}

std::uint64_t avatar_policy_revision(cvc::app &app, const std::string &room_id) {
  const std::string v = get_s(app, sub(policy_path(room_id), {"rev"}));
  return v.empty() ? 0 : std::strtoull(v.c_str(), nullptr, 10);
}

bool read_avatar_policy(cvc::app &app, const std::string &room_id, avatar_energy_policy &out) {
  if (avatar_policy_revision(app, room_id) == 0)
    return false;
  const std::string e = sub(policy_path(room_id), {"energy"});
  avatar_energy_policy p;
  p.enabled = get_b(app, sub(e, {"enabled"}), p.enabled);
  p.force_max = get_f(app, sub(e, {"force_max"}), static_cast<float>(p.force_max));
  p.power_max = get_f(app, sub(e, {"power_max"}), static_cast<float>(p.power_max));
  p.power_rest = get_f(app, sub(e, {"power_rest"}), static_cast<float>(p.power_rest));
  p.reserve_max = get_f(app, sub(e, {"reserve_max"}), static_cast<float>(p.reserve_max));
  p.transport_cost = get_f(app, sub(e, {"transport_cost"}), static_cast<float>(p.transport_cost));
  p.empty = get_s(app, sub(e, {"empty"})) == "stop" ? avatar_energy_policy::when_empty::stop
                                                    : avatar_energy_policy::when_empty::rest;
  p.recover_fraction =
      get_f(app, sub(e, {"recover_fraction"}), static_cast<float>(p.recover_fraction));
  out = p;
  return true;
}

void write_avatar_reserve(cvc::app &app, const std::string &room_id, const std::string &pid,
                          double joules, std::uint64_t server_tick) {
  const std::string base = reserve_path(room_id, pid);
  set_replicated(app, sub(base, {"reserve"}), std::to_string(static_cast<long long>(joules + 0.5)));
  set_replicated(app, sub(base, {"server_tick"}),
                 std::to_string(static_cast<unsigned long long>(server_tick)));
}

bool read_avatar_reserve(cvc::app &app, const std::string &room_id, const std::string &pid,
                         double &joules, std::uint64_t &server_tick) {
  const std::string base = reserve_path(room_id, pid);
  const std::string r = get_s(app, sub(base, {"reserve"}));
  const std::string t = get_s(app, sub(base, {"server_tick"}));
  if (r.empty() || t.empty())
    return false;
  joules = std::strtod(r.c_str(), nullptr);
  server_tick = std::strtoull(t.c_str(), nullptr, 10);
  return true;
}

bool avatar_part_fits(const avatar_part &part) {
  float half[3];
  if (part.shape == "sphere") {
    half[0] = half[1] = half[2] = part.radius;
  } else if (part.shape == "box") {
    for (int i = 0; i < 3; ++i)
      half[i] = 0.5f * part.size[i];
  } else {
    return false; // a shape no renderer knows
  }
  for (int i = 0; i < 3; ++i)
    if (!(std::isfinite(half[i]) && half[i] > 0.0f && std::isfinite(part.pos[i])))
      return false;
  constexpr float kSlack = 1e-4f; // a part drawn flush with the envelope's face still fits
  return std::fabs(part.pos[0]) + half[0] <= kAccessoryReach + kSlack &&
         std::fabs(part.pos[1]) + half[1] <= kAccessoryReach + kSlack &&
         part.pos[2] - half[2] >= -kSlack && part.pos[2] + half[2] <= kAccessoryTop + kSlack;
}

avatar_look resolve_avatar_look(const avatar_catalog &catalog, const avatar_choice &choice,
                                const std::string &pid) {
  avatar_look look;
  const avatar_color_option *color = catalog.find_color(choice.color);
  if (!color && !catalog.colors.empty())
    color = &catalog.colors.front();
  if (color && !color->automatic) {
    std::copy(color->color, color->color + 3, look.body);
    look.color_id = color->id;
  } else {
    auto_avatar_color(pid, look.body);
    look.color_id = color ? color->id : std::string();
  }

  const avatar_accessory_option *acc = catalog.find_accessory(choice.accessory);
  if (!acc && !catalog.accessories.empty())
    acc = &catalog.accessories.front();
  if (acc) {
    look.accessory_id = acc->id;
    for (const avatar_part &p : acc->parts) {
      if (look.parts.size() >= kMaxAccessoryParts)
        break;
      if (!avatar_part_fits(p))
        continue;
      avatar_part q = p;
      if (q.tint == "body")
        std::copy(look.body, look.body + 3, q.color);
      else if (q.tint == "visor")
        std::copy(look.visor, look.visor + 3, q.color);
      look.parts.push_back(std::move(q));
    }
  }

  look.key = look.color_id + "|" + look.accessory_id + "|" + fmt(look.body[0]) + "," +
             fmt(look.body[1]) + "," + fmt(look.body[2]);
  for (const avatar_part &p : look.parts) {
    look.key += "|" + p.shape;
    for (float f : {p.size[0], p.size[1], p.size[2], p.radius, p.pos[0], p.pos[1], p.pos[2],
                    p.color[0], p.color[1], p.color[2]})
      look.key += "," + fmt(f);
  }
  return look;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

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
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_scene.h>
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
std::string sub(const std::string &base, std::initializer_list<const char *> segs) {
  std::string s = base;
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

void check_id(const std::string &id) {
  if (id.empty() || id.find(cvc::state::SEPARATOR) != std::string::npos)
    throw std::invalid_argument("room scene id must be non-empty and contain no '" +
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
void set_rgb(cvc::app &app, const std::string &path, const float v[3]) {
  set_v3(app, path, v, "r", "g", "b");
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
void get_rgb(cvc::app &app, const std::string &path, float v[3]) {
  get_v3(app, path, v, "r", "g", "b");
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

std::string env_path(const std::string &room) { return sub(room_scene_path(room), {"env"}); }
std::string lights_path(const std::string &room) { return sub(env_path(room), {"lights"}); }
std::string objects_path(const std::string &room) {
  return sub(room_scene_path(room), {"objects"});
}

// Written LAST by every scene write, so a reader that sees the new revision also sees the content
// it covers (the journal replicates in write order).
void bump_revision(cvc::app &app, const std::string &room) {
  set_replicated(app, sub(room_scene_path(room), {"rev"}),
                 std::to_string(static_cast<unsigned long long>(scene_revision(app, room) + 1)));
}

void write_light(cvc::app &app, const std::string &base, const cvc::ariadne::SceneLight &l) {
  set_replicated(app, sub(base, {"kind"}), l.kind);
  set_replicated(app, sub(base, {"rig"}), l.rig);
  set_f(app, sub(base, {"azimuth"}), l.azimuth);
  set_f(app, sub(base, {"elevation"}), l.elevation);
  set_f(app, sub(base, {"intensity"}), l.intensity);
  set_f(app, sub(base, {"cone"}), l.cone);
  set_rgb(app, sub(base, {"color"}), l.color);
  set_v3(app, sub(base, {"pos"}), l.pos);
  set_v3(app, sub(base, {"target"}), l.target);
  set_b(app, sub(base, {"present"}), true);
}

cvc::ariadne::SceneLight read_light(cvc::app &app, const std::string &base, const std::string &id) {
  cvc::ariadne::SceneLight l;
  l.id = id;
  l.kind = get_s(app, sub(base, {"kind"}), l.kind);
  l.rig = get_s(app, sub(base, {"rig"}));
  l.azimuth = get_f(app, sub(base, {"azimuth"}), l.azimuth);
  l.elevation = get_f(app, sub(base, {"elevation"}), l.elevation);
  l.intensity = get_f(app, sub(base, {"intensity"}), l.intensity);
  l.cone = get_f(app, sub(base, {"cone"}), l.cone);
  get_rgb(app, sub(base, {"color"}), l.color);
  get_v3(app, sub(base, {"pos"}), l.pos);
  get_v3(app, sub(base, {"target"}), l.target);
  return l;
}

} // namespace

std::string room_scene_path(const std::string &room_id) {
  return sub("conf", {"rooms", room_id.c_str(), "scene"});
}

void write_scene_env(cvc::app &app, const std::string &room_id, const room_scene_env &env) {
  const std::string e = env_path(room_id);
  set_b(app, sub(e, {"background", "gradient"}), env.background_gradient);
  set_rgb(app, sub(e, {"background", "top"}), env.background_top);
  set_rgb(app, sub(e, {"background", "bottom"}), env.background_bottom);
  set_b(app, sub(e, {"chrome"}), env.chrome);
  set_b(app, sub(e, {"shadows", "enabled"}), env.shadows);
  set_replicated(app, sub(e, {"shadows", "resolution"}), std::to_string(env.shadow_resolution));
  std::set<std::string> written;
  for (const cvc::ariadne::SceneLight &l : env.lights) {
    check_id(l.id);
    write_light(app, sub(lights_path(room_id), {l.id.c_str()}), l);
    written.insert(l.id);
  }
  for (const std::string &id : child_ids(app, lights_path(room_id)))
    if (!written.count(id))
      set_b(app, sub(lights_path(room_id), {id.c_str(), "present"}), false);
  bump_revision(app, room_id);
}

bool read_scene_env(cvc::app &app, const std::string &room_id, room_scene_env &env) {
  const std::string e = env_path(room_id);
  if (!find(app, sub(e, {"chrome"})))
    return false; // never written (chrome is always written with the env)
  room_scene_env out;
  out.background_gradient = get_b(app, sub(e, {"background", "gradient"}), false);
  get_rgb(app, sub(e, {"background", "top"}), out.background_top);
  get_rgb(app, sub(e, {"background", "bottom"}), out.background_bottom);
  out.chrome = get_b(app, sub(e, {"chrome"}), true);
  out.shadows = get_b(app, sub(e, {"shadows", "enabled"}), false);
  const std::string res = get_s(app, sub(e, {"shadows", "resolution"}));
  if (!res.empty())
    out.shadow_resolution = std::max(256, std::atoi(res.c_str()));
  for (const std::string &id : child_ids(app, lights_path(room_id))) {
    const std::string base = sub(lights_path(room_id), {id.c_str()});
    if (get_b(app, sub(base, {"present"}), false))
      out.lights.push_back(read_light(app, base, id));
  }
  env = std::move(out);
  return true;
}

void write_scene_object(cvc::app &app, const std::string &room_id,
                        const cvc::ariadne::SceneNode &obj) {
  check_id(obj.id);
  const std::string o = sub(objects_path(room_id), {obj.id.c_str()});
  set_replicated(app, sub(o, {"source", "uri"}), obj.source_file);
  set_replicated(app, sub(o, {"source", "primitive"}), obj.source_primitive);
  set_f(app, sub(o, {"source", "plane_size"}), obj.plane_size);
  set_v3(app, sub(o, {"source", "box"}), obj.box_size);
  set_f(app, sub(o, {"source", "radius"}), obj.sphere_radius);
  set_v3(app, sub(o, {"transform", "pos"}), obj.position);
  set_v3(app, sub(o, {"transform", "rot"}), obj.rotation);
  set_v3(app, sub(o, {"transform", "scale"}), obj.scale);
  set_rgb(app, sub(o, {"material", "color"}), obj.color);
  set_f(app, sub(o, {"material", "ambient"}), obj.ambient);
  set_f(app, sub(o, {"material", "diffuse"}), obj.diffuse);
  // Empty = unset, so a reader keeps GeometryNode's specular defaults (SceneNode::has_specular).
  set_replicated(app, sub(o, {"material", "specular"}), obj.has_specular ? fmt(obj.specular) : "");
  set_replicated(app, sub(o, {"material", "specular_power"}),
                 obj.has_specular ? fmt(obj.specular_power) : "");
  set_b(app, sub(o, {"visible"}), obj.visible_default);
  set_b(app, sub(o, {"present"}), true);
  bump_revision(app, room_id);
}

void remove_scene_object(cvc::app &app, const std::string &room_id, const std::string &id) {
  check_id(id);
  const std::string present = sub(objects_path(room_id), {id.c_str(), "present"});
  if (!find(app, present))
    return;
  set_b(app, present, false);
  bump_revision(app, room_id);
}

std::vector<cvc::ariadne::SceneNode> read_scene_objects(cvc::app &app, const std::string &room_id) {
  std::vector<cvc::ariadne::SceneNode> out;
  for (const std::string &id : child_ids(app, objects_path(room_id))) {
    const std::string o = sub(objects_path(room_id), {id.c_str()});
    if (!get_b(app, sub(o, {"present"}), false))
      continue;
    cvc::ariadne::SceneNode n;
    n.id = id;
    n.type = "geometry";
    n.source_file = get_s(app, sub(o, {"source", "uri"}));
    n.source_primitive = get_s(app, sub(o, {"source", "primitive"}));
    n.plane_size = get_f(app, sub(o, {"source", "plane_size"}), n.plane_size);
    get_v3(app, sub(o, {"source", "box"}), n.box_size);
    n.sphere_radius = get_f(app, sub(o, {"source", "radius"}), n.sphere_radius);
    n.has_transform = true;
    get_v3(app, sub(o, {"transform", "pos"}), n.position);
    get_v3(app, sub(o, {"transform", "rot"}), n.rotation);
    get_v3(app, sub(o, {"transform", "scale"}), n.scale);
    n.has_material = true;
    get_rgb(app, sub(o, {"material", "color"}), n.color);
    n.ambient = get_f(app, sub(o, {"material", "ambient"}), n.ambient);
    n.diffuse = get_f(app, sub(o, {"material", "diffuse"}), n.diffuse);
    if (!get_s(app, sub(o, {"material", "specular"})).empty()) {
      n.has_specular = true;
      n.specular = get_f(app, sub(o, {"material", "specular"}), n.specular);
      n.specular_power = get_f(app, sub(o, {"material", "specular_power"}), n.specular_power);
    }
    n.visible_default = get_b(app, sub(o, {"visible"}), true);
    out.push_back(std::move(n));
  }
  return out;
}

std::uint64_t scene_revision(cvc::app &app, const std::string &room_id) {
  const std::string r = get_s(app, sub(room_scene_path(room_id), {"rev"}));
  return r.empty() ? 0 : static_cast<std::uint64_t>(std::strtoull(r.c_str(), nullptr, 10));
}

cvc::ariadne::Scene read_room_scene(cvc::app &app, const std::string &room_id) {
  cvc::ariadne::Scene s;
  s.nodes = read_scene_objects(app, room_id);
  room_scene_env env;
  if (read_scene_env(app, room_id, env)) {
    s.lights = env.lights;
    s.has_chrome = true;
    s.chrome_visible = env.chrome;
    // Shadows only when the ROOM turns them on: a room that leaves them off must not switch off the
    // ones the client's document enabled (the conference document owns the look).
    if (env.shadows) {
      s.has_shadows = true;
      s.shadows_enabled = true;
      s.has_shadow_resolution = true;
      s.shadow_resolution = env.shadow_resolution;
      s.has_shadow_interval = true;
      s.shadow_interval = 1; // avatars move every frame: re-bake every frame
    }
    s.has_background = true;
    s.background_gradient = env.background_gradient;
    for (int i = 0; i < 3; ++i) {
      s.background_top[i] = env.background_top[i];
      s.background_bottom[i] = env.background_bottom[i];
    }
  }
  return s;
}

room_scene_env default_scene_env() {
  room_scene_env env;
  env.background_gradient = true;
  env.background_top[0] = 0.20f, env.background_top[1] = 0.24f, env.background_top[2] = 0.30f;
  env.background_bottom[0] = 0.05f, env.background_bottom[1] = 0.06f,
  env.background_bottom[2] = 0.08f;
  env.chrome = false; // the SceneGraph chrome is a debug overlay (bbox, axes, labels), not decor
  // No lights and no shadows: the room's look -- its light rig and real-time shadows -- is the
  // client document's (examples/ariadne/documents/conference_roster.ari, scene:), so every client
  // draws the shared objects under the rig its document declares.
  return env;
}

std::vector<cvc::ariadne::SceneNode> default_scene_objects() {
  cvc::ariadne::SceneNode ground;
  ground.id = "ground";
  ground.source_primitive = "plane";
  ground.plane_size = 20.0f; // metres; spans [-10, 10]^2 in XY at z = 0
  ground.color[0] = 0.32f, ground.color[1] = 0.34f, ground.color[2] = 0.38f;
  ground.ambient = 0.35f;
  ground.diffuse = 0.85f;

  cvc::ariadne::SceneNode ref;
  ref.id = "ref";
  ref.source_primitive = "box";
  ref.box_size[0] = ref.box_size[1] = ref.box_size[2] = 1.0f;
  ref.position[2] = 0.5f; // centred box: stand it on the ground
  ref.color[0] = 0.85f, ref.color[1] = 0.45f, ref.color[2] = 0.20f;
  ref.has_specular = true;
  ref.specular = 0.25f;
  ref.specular_power = 24.0f;
  return {ground, ref};
}

bool seed_default_scene(cvc::app &app, const std::string &room_id) {
  if (scene_revision(app, room_id) != 0)
    return false;
  write_scene_env(app, room_id, default_scene_env());
  for (const cvc::ariadne::SceneNode &n : default_scene_objects())
    write_scene_object(app, room_id, n);
  return true;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

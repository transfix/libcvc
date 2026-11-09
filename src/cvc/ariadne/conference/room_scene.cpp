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
// it covers (the journal replicates in write order). The scene is a revisioned section: its epoch
// tells a restarted relay's revision 1 from the one a client kept from the old relay. Its `at` is
// 0: the scene is cosmetic and applies on arrival.
section_stamp bump_revision(cvc::app &app, const std::string &room, std::uint64_t epoch = 0) {
  return section_writer(app, room_scene_path(room)).commit(0, epoch);
}

// The run an entry (an object, a light, an object's fit) was written in. A client that outlived a
// relay keeps that relay's entries, and the new relay never removes the ones it has no id for (it
// never knew them), so a reader takes only the scene's own run's: an entry stamped with another
// epoch is the old relay's.
void set_epoch(cvc::app &app, const std::string &base, std::uint64_t epoch) {
  set_replicated(app, sub(base, {"epoch"}), std::to_string(static_cast<unsigned long long>(epoch)));
}
// Whether the entry under BASE is of the run SCENE_EPOCH: stamped with it, or not stamped at all
// (a writer from before epochs, or a --set). A scene with no epoch has no runs to tell apart.
bool of_run(cvc::app &app, const std::string &base, std::uint64_t scene_epoch) {
  const std::string e = get_s(app, sub(base, {"epoch"}));
  return scene_epoch == 0 || e.empty() || std::strtoull(e.c_str(), nullptr, 10) == scene_epoch;
}
// The epoch a write stamps: EPOCH, or this process's when 0 (section_writer::commit's rule).
std::uint64_t run_epoch(std::uint64_t epoch) { return epoch != 0 ? epoch : section_epoch(); }

// A rig knob: its value when the light sets it, else empty (the preset's value).
void set_knob(cvc::app &app, const std::string &path, bool has, float f) {
  set_replicated(app, path, has ? fmt(f) : "");
}
bool get_knob(cvc::app &app, const std::string &path, float &f) {
  cvc::state *n = find(app, path);
  if (!n || n->value().empty())
    return false;
  f = std::strtof(n->value().c_str(), nullptr);
  return true;
}

// A rig's tuning knobs. Only a rig light writes them -- or a light replacing one, to clear them --
// so a plain light keeps exactly the leaves it always had. stage.radius, key.intensity and
// wash.intensity gate their groups.
void write_rig_knobs(cvc::app &app, const std::string &base, const cvc::ariadne::SceneLight &l) {
  if (l.rig.empty() && !find(app, sub(base, {"stage", "radius"})))
    return;
  const bool rig = !l.rig.empty();
  const bool stage = rig && l.has_stage, key = rig && l.has_key, wash = rig && l.has_wash;
  set_knob(app, sub(base, {"stage", "radius"}), stage, l.stage_radius);
  if (stage)
    set_v3(app, sub(base, {"stage", "center"}), l.stage_center);
  set_knob(app, sub(base, {"key", "intensity"}), key, l.key_intensity);
  set_knob(app, sub(base, {"key", "azimuth"}), key, l.key_azimuth);
  set_knob(app, sub(base, {"key", "elevation"}), key, l.key_elevation);
  set_knob(app, sub(base, {"key", "cone"}), key, l.key_cone);
  set_knob(app, sub(base, {"fill"}), rig && l.has_fill, l.fill);
  set_knob(app, sub(base, {"back"}), rig && l.has_back, l.back);
  set_knob(app, sub(base, {"warmth"}), rig && l.has_warmth, l.warmth);
  set_knob(app, sub(base, {"environment"}), rig && l.has_environment, l.environment);
  set_knob(app, sub(base, {"ambient"}), rig && l.has_rig_ambient, l.rig_ambient);
  set_knob(app, sub(base, {"wash", "intensity"}), wash, l.wash);
  set_replicated(app, sub(base, {"wash", "count"}), wash ? std::to_string(l.wash_count) : "");
  set_knob(app, sub(base, {"wash", "height"}), wash, l.wash_height);
  set_replicated(app, sub(base, {"sun"}), rig && l.has_sun ? (l.sun ? "1" : "0") : "");
}

void read_rig_knobs(cvc::app &app, const std::string &base, cvc::ariadne::SceneLight &l) {
  l.has_stage = get_knob(app, sub(base, {"stage", "radius"}), l.stage_radius);
  if (l.has_stage)
    get_v3(app, sub(base, {"stage", "center"}), l.stage_center);
  l.has_key = get_knob(app, sub(base, {"key", "intensity"}), l.key_intensity);
  if (l.has_key) {
    get_knob(app, sub(base, {"key", "azimuth"}), l.key_azimuth);
    get_knob(app, sub(base, {"key", "elevation"}), l.key_elevation);
    get_knob(app, sub(base, {"key", "cone"}), l.key_cone);
  }
  l.has_fill = get_knob(app, sub(base, {"fill"}), l.fill);
  l.has_back = get_knob(app, sub(base, {"back"}), l.back);
  l.has_warmth = get_knob(app, sub(base, {"warmth"}), l.warmth);
  l.has_environment = get_knob(app, sub(base, {"environment"}), l.environment);
  l.has_rig_ambient = get_knob(app, sub(base, {"ambient"}), l.rig_ambient);
  l.has_wash = get_knob(app, sub(base, {"wash", "intensity"}), l.wash);
  if (l.has_wash) {
    const std::string count = get_s(app, sub(base, {"wash", "count"}));
    if (!count.empty())
      l.wash_count = std::atoi(count.c_str());
    get_knob(app, sub(base, {"wash", "height"}), l.wash_height);
  }
  const std::string sun = get_s(app, sub(base, {"sun"}));
  l.has_sun = !sun.empty();
  l.sun = sun == "1";
}

void write_light(cvc::app &app, const std::string &base, const cvc::ariadne::SceneLight &l,
                 std::uint64_t epoch) {
  set_replicated(app, sub(base, {"kind"}), l.kind);
  set_replicated(app, sub(base, {"rig"}), l.rig);
  set_f(app, sub(base, {"azimuth"}), l.azimuth);
  set_f(app, sub(base, {"elevation"}), l.elevation);
  set_f(app, sub(base, {"intensity"}), l.intensity);
  set_f(app, sub(base, {"cone"}), l.cone);
  set_rgb(app, sub(base, {"color"}), l.color);
  set_v3(app, sub(base, {"pos"}), l.pos);
  set_v3(app, sub(base, {"target"}), l.target);
  write_rig_knobs(app, base, l);
  set_epoch(app, base, epoch);
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
  // A rig writes every knob (an unset one empty), so its knobs are always its own run's. A plain
  // light has none: what its leaves hold is an earlier light's, under this id.
  if (!l.rig.empty())
    read_rig_knobs(app, base, l);
  return l;
}

} // namespace

std::string room_scene_path(const std::string &room_id) {
  return sub("conf", {"rooms", room_id.c_str(), "scene"});
}

namespace {

// The writes behind write_scene_env / write_scene_object, without the revision bump: a caller
// that writes several bumps once, last.
void write_env_fields(cvc::app &app, const std::string &room_id, const room_scene_env &env,
                      std::uint64_t epoch) {
  for (const cvc::ariadne::SceneLight &l : env.lights)
    check_id(l.id); // before any write, so a bad id leaves the room as it was
  const std::string e = env_path(room_id);
  set_b(app, sub(e, {"background", "gradient"}), env.background_gradient);
  set_rgb(app, sub(e, {"background", "top"}), env.background_top);
  set_rgb(app, sub(e, {"background", "bottom"}), env.background_bottom);
  set_b(app, sub(e, {"chrome"}), env.chrome);
  set_b(app, sub(e, {"shadows", "enabled"}), env.shadows);
  set_replicated(app, sub(e, {"shadows", "resolution"}), std::to_string(env.shadow_resolution));
  set_replicated(app, sub(e, {"shadows", "interval"}),
                 std::to_string(std::max(1, env.shadow_interval)));
  std::set<std::string> written;
  for (const cvc::ariadne::SceneLight &l : env.lights) {
    write_light(app, sub(lights_path(room_id), {l.id.c_str()}), l, epoch);
    written.insert(l.id);
  }
  for (const std::string &id : child_ids(app, lights_path(room_id)))
    if (!written.count(id))
      set_b(app, sub(lights_path(room_id), {id.c_str(), "present"}), false);
}

void write_object_fields(cvc::app &app, const std::string &room_id,
                         const cvc::ariadne::SceneNode &obj, std::uint64_t epoch);

} // namespace

void write_scene_env(cvc::app &app, const std::string &room_id, const room_scene_env &env) {
  write_env_fields(app, room_id, env, run_epoch(0));
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
  const std::string every = get_s(app, sub(e, {"shadows", "interval"}));
  if (!every.empty())
    out.shadow_interval = std::max(1, std::atoi(every.c_str()));
  const std::uint64_t run = scene_stamp(app, room_id).epoch;
  for (const std::string &id : child_ids(app, lights_path(room_id))) {
    const std::string base = sub(lights_path(room_id), {id.c_str()});
    if (get_b(app, sub(base, {"present"}), false) && of_run(app, base, run))
      out.lights.push_back(read_light(app, base, id));
  }
  env = std::move(out);
  return true;
}

void write_scene_object(cvc::app &app, const std::string &room_id,
                        const cvc::ariadne::SceneNode &obj) {
  check_id(obj.id);
  write_object_fields(app, room_id, obj, run_epoch(0));
  bump_revision(app, room_id);
}

namespace {

void write_object_fields(cvc::app &app, const std::string &room_id,
                         const cvc::ariadne::SceneNode &obj, std::uint64_t epoch) {
  const std::string o = sub(objects_path(room_id), {obj.id.c_str()});
  set_replicated(app, sub(o, {"source", "uri"}), obj.source_file);
  set_replicated(app, sub(o, {"source", "primitive"}), obj.source_primitive);
  set_f(app, sub(o, {"source", "plane_size"}), obj.plane_size);
  set_v3(app, sub(o, {"source", "box"}), obj.box_size);
  set_f(app, sub(o, {"source", "radius"}), obj.sphere_radius);
  // Only an object with a fit -- or one replacing an object that had one, to clear it -- writes
  // these, so every other object keeps exactly the leaves it always had. Empty height = no fit.
  // The fit carries its own epoch: an object without one never clears a fit an old relay's object
  // of the same id left in a client's tree.
  const std::string fit = sub(o, {"source", "fit"});
  if (obj.has_fit || find(app, sub(fit, {"height"}))) {
    set_replicated(app, sub(fit, {"height"}), obj.has_fit ? fmt(obj.fit_height) : "");
    set_b(app, sub(fit, {"up_y"}), obj.has_fit && obj.fit_up_y);
    set_epoch(app, fit, epoch);
  }
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
  set_epoch(app, o, epoch);
  set_b(app, sub(o, {"present"}), true);
}

} // namespace

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
  const std::uint64_t run = scene_stamp(app, room_id).epoch;
  for (const std::string &id : child_ids(app, objects_path(room_id))) {
    const std::string o = sub(objects_path(room_id), {id.c_str()});
    if (!get_b(app, sub(o, {"present"}), false) || !of_run(app, o, run))
      continue;
    cvc::ariadne::SceneNode n;
    n.id = id;
    n.type = "geometry";
    n.source_file = get_s(app, sub(o, {"source", "uri"}));
    n.source_primitive = get_s(app, sub(o, {"source", "primitive"}));
    n.plane_size = get_f(app, sub(o, {"source", "plane_size"}), n.plane_size);
    get_v3(app, sub(o, {"source", "box"}), n.box_size);
    n.sphere_radius = get_f(app, sub(o, {"source", "radius"}), n.sphere_radius);
    if (!get_s(app, sub(o, {"source", "fit", "height"})).empty() &&
        of_run(app, sub(o, {"source", "fit"}), run)) {
      n.has_fit = true;
      n.fit_height = get_f(app, sub(o, {"source", "fit", "height"}), n.fit_height);
      n.fit_up_y = get_b(app, sub(o, {"source", "fit", "up_y"}), false);
    }
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
  return scene_stamp(app, room_id).rev;
}

section_stamp scene_stamp(cvc::app &app, const std::string &room_id) {
  return read_section_stamp(app, room_scene_path(room_id));
}

cvc::ariadne::Scene read_room_scene(cvc::app &app, const std::string &room_id) {
  cvc::ariadne::Scene s;
  s.nodes = read_scene_objects(app, room_id);
  room_scene_env env;
  if (read_scene_env(app, room_id, env)) {
    s.lights = env.lights;
    s.has_chrome = true;
    s.chrome_visible = env.chrome;
    // Shadows only when the ROOM turns them on: the look is the world's (WD1), but a room that
    // leaves them off must not switch off ones a client's own document enabled.
    if (env.shadows) {
      s.has_shadows = true;
      s.shadows_enabled = true;
      s.has_shadow_resolution = true;
      s.shadow_resolution = env.shadow_resolution;
      s.has_shadow_interval = true;
      s.shadow_interval = env.shadow_interval; // 1 by default: avatars move every frame
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
  // No lights and no shadows: the look is the world's, and default_world() adds the stock rig.
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

std::vector<cvc::ariadne::SceneNode> large_demo_scene_objects(double side_m, int grid) {
  if (!(std::isfinite(side_m) && side_m > 0.0))
    throw std::invalid_argument("large demo scene: side_m must be finite and > 0");
  if (grid < 1)
    throw std::invalid_argument("large demo scene: grid must be >= 1");
  std::vector<cvc::ariadne::SceneNode> out = default_scene_objects(); // {ground, ref}
  out[0].plane_size = static_cast<float>(side_m);

  cvc::ariadne::SceneNode bunny;
  bunny.source_file = "x.bunny"; // the embedded Stanford bunny: no asset to ship
  bunny.has_fit = true;          // a Y-up mesh about 15 cm tall, stood up Z-up and scaled
  bunny.fit_up_y = true;
  bunny.fit_height = 1.7f; // metres along its largest extent; it stands about that tall
  bunny.color[0] = 0.78f, bunny.color[1] = 0.74f, bunny.color[2] = 0.66f;
  bunny.ambient = 0.3f;
  bunny.diffuse = 0.8f;
  bunny.has_specular = true;
  bunny.specular = 0.2f;
  bunny.specular_power = 16.0f;

  const double spacing = side_m / grid;
  for (int j = 0; j < grid; ++j)
    for (int i = 0; i < grid; ++i) {
      // The middle cell of an odd grid sits on the origin: the reference box's.
      if (grid % 2 == 1 && i == grid / 2 && j == grid / 2)
        continue;
      cvc::ariadne::SceneNode b = bunny;
      b.id = "bunny_" + std::to_string(i) + "_" + std::to_string(j);
      b.position[0] = static_cast<float>(-0.5 * side_m + (i + 0.5) * spacing);
      b.position[1] = static_cast<float>(-0.5 * side_m + (j + 0.5) * spacing);
      out.push_back(std::move(b));
    }
  return out;
}

bool seed_large_demo_scene(cvc::app &app, const std::string &room_id, double side_m, int grid) {
  // Validate before the revision check, so a bad argument throws whether or not the room is
  // already seeded.
  const std::vector<cvc::ariadne::SceneNode> objects = large_demo_scene_objects(side_m, grid);
  if (scene_revision(app, room_id) != 0)
    return false;
  write_scene_env(app, room_id, default_scene_env());
  for (const cvc::ariadne::SceneNode &n : objects)
    write_scene_object(app, room_id, n);
  return true;
}

section_stamp write_room_world(cvc::app &app, const std::string &room_id, const room_world &world,
                               std::uint64_t epoch) {
  for (const cvc::ariadne::SceneNode &o : world.objects)
    check_id(o.id);
  const std::uint64_t run = run_epoch(epoch);
  write_env_fields(app, room_id, world.env, run); // checks the light ids before it writes
  std::set<std::string> written;
  for (const cvc::ariadne::SceneNode &o : world.objects) {
    write_object_fields(app, room_id, o, run);
    written.insert(o.id);
  }
  for (const std::string &id : child_ids(app, objects_path(room_id))) {
    const std::string present = sub(objects_path(room_id), {id.c_str(), "present"});
    if (!written.count(id) && get_b(app, present, false))
      set_b(app, present, false);
  }
  return bump_revision(app, room_id, run);
}

room_world room_world_from_scene(const cvc::ariadne::Scene &scene,
                                 std::vector<std::string> *warnings) {
  const auto warn = [warnings](std::string w) {
    if (warnings)
      warnings->push_back(std::move(w));
  };
  room_world w;
  w.env = default_scene_env(); // a scene that sets no background keeps the room's
  if (scene.has_background) {
    w.env.background_gradient = scene.background_gradient;
    for (int i = 0; i < 3; ++i) {
      w.env.background_top[i] = scene.background_top[i];
      w.env.background_bottom[i] = scene.background_bottom[i];
    }
  }
  w.env.chrome = scene.has_chrome && scene.chrome_visible;
  w.env.shadows = scene.has_shadows && scene.shadows_enabled;
  if (scene.has_shadow_resolution)
    w.env.shadow_resolution = scene.shadow_resolution;
  if (scene.has_shadow_interval)
    w.env.shadow_interval = scene.shadow_interval;
  if (scene.has_shadow_softness)
    warn("scene: shadows: softness is not carried by the room; left out");
  if (scene.shadow_focus.present)
    warn("scene: shadows: focus is not carried by the room; left out");
  // An id is a path segment in the room tree, so one holding the separator cannot be carried.
  const auto nameable = [&](const std::string &what, const std::string &id) {
    if (id.find(cvc::state::SEPARATOR) == std::string::npos)
      return true;
    warn("scene: " + what + " '" + id + "': its name holds '" + std::string(cvc::state::SEPARATOR) +
         "', which a room's id cannot; left out");
    return false;
  };
  for (std::size_t i = 0; i < scene.lights.size(); ++i) {
    cvc::ariadne::SceneLight l = scene.lights[i];
    if (l.id.empty())
      l.id = "light_" + std::to_string(i + 1);
    if (nameable("light", l.id))
      w.env.lights.push_back(std::move(l));
  }
  for (std::size_t i = 0; i < scene.nodes.size(); ++i) {
    cvc::ariadne::SceneNode n = scene.nodes[i];
    if (n.id.empty())
      n.id = "node_" + std::to_string(i + 1);
    if (!nameable("node", n.id))
      continue;
    const std::string who = "scene: node '" + n.id + "'";
    if (n.type != "geometry") {
      warn(who + ": a " + n.type + " node is not carried by the room; left out");
      continue;
    }
    const bool primitive = n.source_primitive == "plane" || n.source_primitive == "box" ||
                           n.source_primitive == "sphere";
    if (n.has_heightfield || !n.source_sdf_mesh.empty() || !n.source_data.empty() ||
        !n.source_asset.empty() || (!primitive && n.source_file.empty())) {
      warn(who + ": the room carries a plane, box or sphere, or a mesh by URI, and this source " +
           "is none of them; left out");
      continue;
    }
    if (!n.children.empty())
      warn(who + ": its " + std::to_string(n.children.size()) +
           " children are left out (the room's objects are flat)");
    if (!n.material_texture.empty())
      warn(who + ": its texture is not carried by the room; left out");
    if (n.shader.present)
      warn(who + ": its shader is not carried by the room; left out");
    if (n.has_fit && (n.fit_up == "x" || n.fit_flip))
      warn(who + ": fit: up x and flip are not carried by the room; left out");
    if (!n.visible_bind.empty())
      warn(who + ": its visible: bind is not carried by the room; left out");
    if (n.body.present) {
      const std::string motion = n.body.motion.empty() ? "dynamic" : n.body.motion;
      if (motion != "static")
        warn(who + ": its " + motion + " body is published as a static (server-dynamic " +
             "entities are WD3's)");
    }
    n.fit_up_y = n.fit_up_y || n.fit_up == "y";
    n.children.clear();
    w.objects.push_back(std::move(n));
  }
  if (scene.physics.present)
    warn("scene: physics: is not simulated from the document yet (the headless world runtime, "
         "L0); the holon builds its statics from the published objects");
  return w;
}

cvc::ariadne::SceneLight default_room_rig() {
  cvc::ariadne::SceneLight l;
  l.id = "room_rig";
  l.rig = "three_point";
  l.has_stage = true;
  l.stage_center[2] = 1.0f;
  l.stage_radius = 10.0f;
  l.has_key = true;
  l.key_intensity = 0.9f;
  l.key_azimuth = -35.0f;
  l.key_elevation = 55.0f;
  l.key_cone = 48.0f;
  l.has_fill = true;
  l.fill = 0.25f;
  l.has_back = true;
  l.back = 0.25f;
  l.has_warmth = true;
  l.warmth = 0.25f;
  l.has_environment = true;
  l.environment = 0.15f;
  l.has_rig_ambient = true;
  l.rig_ambient = 0.12f;
  return l;
}

namespace {

room_world lit_world(std::vector<cvc::ariadne::SceneNode> objects) {
  room_world w;
  w.env = default_scene_env();
  w.env.shadows = true;
  w.env.shadow_resolution = 2048;
  w.env.shadow_interval = 1;
  w.env.lights.push_back(default_room_rig());
  w.objects = std::move(objects);
  return w;
}

} // namespace

room_world default_world() { return lit_world(default_scene_objects()); }

room_world large_world(double side_m, int grid) {
  return lit_world(large_demo_scene_objects(side_m, grid));
}

} // namespace conference
} // namespace ariadne
} // namespace cvc

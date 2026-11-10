/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The world half of a scene, with no renderer (scene_world.h). The body rules were cvcGL's
// realize_physics; cvcGL now builds its world here too.

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/state/state.h>
#include <cvc/world/world_geometry.h>
#include <deque>
#include <exception>
#include <optional>
#include <set>
#include <sstream>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

bool node_has_world(const SceneNode &n) {
  if ((n.body.present && n.source_data.empty()) || !n.world.empty())
    return true;
  for (const SceneNode &c : n.children)
    if (node_has_world(c))
      return true;
  return false;
}

void world_matrices(const SceneNode &n, const double parent[16],
                    std::map<const SceneNode *, std::array<double, 16>> &out) {
  double local[16], w[16];
  scene_node_local_matrix(n, local);
  multiply_matrix4(parent, local, w);
  std::array<double, 16> &dst = out[&n];
  std::copy(w, w + 16, dst.begin());
  for (const SceneNode &c : n.children)
    world_matrices(c, w, out);
}

// A rigid pose and per-axis scale from a node's world matrix (row-major 4x4): the translation, the
// rotation with each column's length divided out, and those lengths. `skewed` is set when the
// columns are not orthogonal (a non-uniform scale under a rotated parent), which a rigid body
// cannot follow.
void decompose(const double m[16], ph::vec3 &t, ph::quat &q, double scale[3], bool &skewed,
               bool &mirrored) {
  t = {m[3], m[7], m[11]};
  double r[3][3];
  for (int c = 0; c < 3; ++c) {
    const double s = std::sqrt(m[c] * m[c] + m[4 + c] * m[4 + c] + m[8 + c] * m[8 + c]);
    scale[c] = s;
    for (int k = 0; k < 3; ++k)
      r[k][c] = s > 0.0 ? m[4 * k + c] / s : (k == c ? 1.0 : 0.0);
  }
  const auto col_dot = [&](int a, int b) {
    return r[0][a] * r[0][b] + r[1][a] * r[1][b] + r[2][a] * r[2][b];
  };
  skewed = std::fabs(col_dot(0, 1)) > 1e-6 || std::fabs(col_dot(0, 2)) > 1e-6 ||
           std::fabs(col_dot(1, 2)) > 1e-6;
  const double det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                     r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                     r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  mirrored = det < 0.0;
  // Shepperd's method.
  const double tr = r[0][0] + r[1][1] + r[2][2];
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q = {0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s};
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
    q = {(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s};
  } else if (r[1][1] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
    q = {(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s};
  } else {
    const double s = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
    q = {(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s};
  }
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q = {q.w / n, q.x / n, q.y / n, q.z / n};
}

ph::vec3 rotate(const ph::quat &q, const ph::vec3 &v) {
  // v' = q v q*
  const double tx = 2 * (q.y * v.z - q.z * v.y), ty = 2 * (q.z * v.x - q.x * v.z),
               tz = 2 * (q.x * v.y - q.y * v.x);
  return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
          v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

bool near_one(double s) { return std::fabs(s - 1.0) < 1e-6; }

std::string num(double v) {
  std::ostringstream o;
  o.precision(17);
  o << v;
  return o.str();
}

bool is_identity(const std::array<double, 16> &m) {
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      if (std::fabs(m[4 * r + c] - (r == c ? 1.0 : 0.0)) > 1e-12)
        return false;
  return true;
}

// What a data node's body: collides with, from the dataset at `store`: its source: { item }, else
// the first of its world, geometry and terrain it has (a volume has no body). Null when it has
// none of those.
std::shared_ptr<const cvc::world::world_geometry> dataset_colliders(cvc::state &root,
                                                                    const std::string &store,
                                                                    const std::string &item_key,
                                                                    const std::string &node) {
  const std::vector<std::string> keys =
      item_key.empty() ? std::vector<std::string>{"world", "geometry", "terrain", "volume"}
                       : std::vector<std::string>{item_key};
  boost::any obj;
  for (const std::string &k : keys)
    if (cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + k)) {
      obj = s->data();
      if (!obj.empty())
        break;
    }
  using world_ptr = std::shared_ptr<const cvc::world::world_geometry>;
  using geometry_ptr = std::shared_ptr<const cvc::geometry>;
  using field_ptr = std::shared_ptr<const cvc::height_field>;
  if (const world_ptr *w = boost::any_cast<world_ptr>(&obj))
    return *w;
  cvc::world::geometry_item item;
  item.name = node;
  if (const field_ptr *f = boost::any_cast<field_ptr>(&obj))
    item.geometry = *f;
  else if (const geometry_ptr *g = boost::any_cast<geometry_ptr>(&obj))
    item.geometry = *g;
  else
    return nullptr;
  auto out = std::make_shared<cvc::world::world_geometry>();
  out->add(std::move(item));
  return out;
}

// A data node's dataset, as the node names it: <prefix>.data.<name>, or an absolute state path.
std::string data_path(const std::string &bind_prefix, const SceneNode &n) {
  return resolve_bind(bind_prefix, n.source_data[0] == '/'
                                       ? n.source_data
                                       : std::string(kDataRoot) + "." + n.source_data);
}

const char *const kDataNeedsPhysics =
    "body: needs the scene's physics: block for a dataset's bodies";

} // namespace

bool scene_has_world(const Scene &scene) {
  if (scene.physics.present || scene.world.present)
    return true;
  for (const SceneNode &n : scene.nodes)
    if (node_has_world(n))
      return true;
  return false;
}

void scene_world_matrices(const Scene &scene,
                          std::map<const SceneNode *, std::array<double, 16>> &out) {
  double identity[16];
  for (int k = 0; k < 16; ++k)
    identity[k] = (k % 5 == 0) ? 1.0 : 0.0;
  for (const SceneNode &n : scene.nodes)
    world_matrices(n, identity, out);
}

cvc::world_clock_keys scene_clock_keys(const SceneClock &clock) {
  cvc::world_clock_keys k;
  const std::pair<const std::string *, std::string> named[] = {{&clock.speed_key, ".speed"},
                                                               {&clock.paused_key, ".paused"},
                                                               {&clock.time_key, ".time"},
                                                               {&clock.tick_key, ".tick"}};
  for (const auto &n : named) {
    const std::string &key = *n.first;
    const std::string &leaf = n.second;
    if (key.size() > leaf.size() && key.compare(key.size() - leaf.size(), leaf.size(), leaf) == 0) {
      k = cvc::world_clock_keys::under(key.substr(0, key.size() - leaf.size()));
      break;
    }
  }
  k.speed = clock.speed_key;
  k.paused = clock.paused_key;
  k.time = clock.time_key;
  k.tick = clock.tick_key;
  return k;
}

// ── SceneClockDriver ─────────────────────────────────────────────────────────────────────────────

struct SceneClockDriver::impl {
  impl(cvc::app &a, const SceneClock &c) : app(a), clock(c) {}
  cvc::app &app;
  SceneClock clock;
  std::unique_ptr<cvc::world_clock_binding> binding; // made by the first drive()
  std::chrono::steady_clock::time_point last{};
  bool primed = false;
};

SceneClockDriver::SceneClockDriver(cvc::app &app, const SceneClock &resolved)
    : _impl(std::make_unique<impl>(app, resolved)) {}
SceneClockDriver::~SceneClockDriver() = default;

cvc::world_clock::step_result SceneClockDriver::drive(double wall_dt) {
  impl &d = *_impl;
  if (!d.binding) {
    cvc::world_clock &wc = d.app.world_clock();
    // The declared initial rate and pause go on the clock before the binding exists: the binding
    // seeds every still-unset key from the clock, so a sim_transport widget shows them on frame 0,
    // and they stand for good on a lane the scene left unbound (an empty key).
    wc.set_scale(d.clock.scale);
    if (d.clock.paused)
      wc.set_mode(cvc::world_clock::mode::paused);
    else if (wc.current_mode() == cvc::world_clock::mode::paused)
      wc.set_mode(cvc::world_clock::mode::live);
    d.binding = std::make_unique<cvc::world_clock_binding>(d.app, wc, scene_clock_keys(d.clock));
  }
  // Wall delta: a caller's fixed dt (deterministic capture, a test) when non-negative, else the
  // real time since the last drive (no crawl under a slow frame rate).
  double dt = wall_dt;
  if (dt < 0.0) {
    const auto now = std::chrono::steady_clock::now();
    dt = d.primed ? std::chrono::duration<double>(now - d.last).count() : 0.0;
    d.last = now;
    d.primed = true;
  }
  // pull (speed / paused / mode / lockstep / step requests), advance, push (time, tick, ...).
  return d.binding->drive(dt);
}

// ── SceneWorld ───────────────────────────────────────────────────────────────────────────────────

struct SceneWorld::impl {
  cvc::app *app = nullptr;
  std::unique_ptr<ph::world> world;
  std::vector<SceneWorldBody> bodies;
  std::string stats; // "<stats_key>.stats", or empty
  std::optional<cvc::clock_follower> follower;
  ph::step_report last;

  // A node whose colliders come at run time: a generated world's, or a dataset's.
  struct tracked {
    std::string node;
    std::shared_ptr<SceneColliders> colliders;
    std::uint64_t applied = 0; // the revision its bodies are from (0: none yet)
    std::string item;          // a data node's source: { item }
    std::string data_path;     // a data node's dataset, or empty
    std::string target, rev;   // the store node and its rev last taken
  };
  std::vector<tracked> tracked_nodes;
  std::vector<std::unique_ptr<GeneratedWorldRuntime>> generated; // run here (from_scene)
  std::vector<std::string> warnings;                             // follow()'s syncs'

  // The scene's worlds (bind_worlds): each by name, its nodes and what each is, in the document's
  // order; the world on, the nodes of the others (their bodies out), and the keys.
  struct world_def {
    std::string name;
    std::vector<std::string> nodes, kinds;
  };
  std::vector<world_def> worlds;
  std::string world_key; // <prefix>.scene.world, or "" (no worlds)
  std::string on;
  std::set<std::string> off;
  double grid_cell = 0.0, grid_half = 0.0;
  std::uint64_t world_rev = 0;
  std::string on_sig;      // what the world on was made from at the last rev
  std::string refused;     // a source key's value that names no world, warned about once
  std::string enabled_key; // <stats_key>.enabled, or ""
  std::deque<double> step_history;
  // A terrain node's field as its body has it (placed), and a node's own bodies' XY extent (min_x,
  // min_y, max_x, max_y): what grid() makes a terrain's or a flat ground's grid from.
  std::map<std::string, std::shared_ptr<const cvc::height_field>> fields;
  std::map<std::string, std::array<double, 4>> footprints;

  const world_def *find_world(const std::string &name) const {
    for (const world_def &w : worlds)
      if (w.name == name)
        return &w;
    return nullptr;
  }
  void remove_bodies_of(const std::string &node) {
    for (auto it = bodies.begin(); it != bodies.end();) {
      if (it->node == node) {
        world->remove_body(it->id);
        it = bodies.erase(it);
      } else {
        ++it;
      }
    }
  }
  // Node `node`'s bodies back in the world, as build() made them (a tracked node's: its colliders
  // as they are now).
  void add_bodies_of(const std::string &node, std::vector<std::string> *w) {
    for (tracked &t : tracked_nodes)
      if (t.node == node && t.colliders->revision > 0)
        apply(t, w);
    for (const entry &e : entries) {
      if (e.tracked >= 0 || e.body.node != node)
        continue;
      SceneWorldBody b = e.body;
      try {
        b.id = world->add_body(e.desc);
      } catch (const std::exception &ex) {
        warn(w, "ari: scene node '" + node + "': body: " + ex.what());
        continue;
      }
      bodies.push_back(b);
    }
  }
  bool stepping() const {
    if (enabled_key.empty() || !app)
      return true;
    cvc::state *k = cvc::state::instance(*app).findDescendant(enabled_key);
    if (!k)
      return true;
    const std::string v = k->value();
    return !(v == "0" || v == "false");
  }

  // grid()'s last grid per node, and what it was made from.
  struct grid_cache {
    std::uint64_t revision = 0;
    double cell = 0.0, half = 0.0;
    bool made = false; // made from these (grid null: it failed, and warned once)
    std::shared_ptr<const cvc::nav::world_grid> grid;
  };
  std::map<std::string, grid_cache> grids;

  // How the world was built, for restart(): its config, its materials in order, and each body in
  // the order build() made it (a node's own, or a tracked node's colliders: its index).
  ph::world_config cfg;
  std::vector<ph::material> materials;
  struct entry {
    int tracked = -1; // a tracked node's index, or -1 for a node's own body
    ph::body_desc desc;
    SceneWorldBody body;
  };
  std::vector<entry> entries;

  // Swap node t's bodies for its colliders' current geometry.
  void apply(tracked &t, std::vector<std::string> *w) {
    for (auto it = bodies.begin(); it != bodies.end();) {
      if (it->node == t.node) {
        world->remove_body(it->id);
        it = bodies.erase(it);
      } else {
        ++it;
      }
    }
    t.applied = t.colliders->revision;
    if (!t.colliders->geometry)
      return;
    try {
      const ph::world_bodies made = ph::add_world_geometry(*world, *t.colliders->geometry);
      for (const auto &b : made.bodies) {
        SceneWorldBody body;
        body.node = t.node;
        body.id = b.second;
        body.moves = false;
        body.body_from_visual = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        bodies.push_back(body);
      }
    } catch (const std::exception &e) {
      warn(w, "ari: scene node '" + t.node + "': body: " + e.what());
    }
  }
};

SceneWorld::SceneWorld() : _impl(std::make_unique<impl>()) {}
SceneWorld::~SceneWorld() = default;

ph::world &SceneWorld::world() { return *_impl->world; }
const ph::world &SceneWorld::world() const { return *_impl->world; }
const std::vector<SceneWorldBody> &SceneWorld::bodies() const { return _impl->bodies; }

ph::body_id SceneWorld::body(const std::string &id) const {
  for (const SceneWorldBody &b : _impl->bodies)
    if (b.node == id)
      return b.id;
  return ph::body_id{};
}

std::unique_ptr<SceneWorld>
SceneWorld::build(cvc::app &app, const ScenePhysics &spec,
                  const std::vector<SceneBodyInput> &bodies,
                  const std::map<std::string, std::shared_ptr<const cvc::height_field>> &fields,
                  const std::string &stats_key, std::vector<std::string> *warnings) {
  ph::world_config cfg;
  cfg.gravity = {spec.gravity[0], spec.gravity[1], spec.gravity[2]};
  const bool have_jolt = ph::backend_available(ph::backend::jolt);
  if (spec.backend == "null") {
    cfg.backend = ph::backend::null;
  } else if (spec.backend == "jolt" || spec.backend == "auto" || spec.backend.empty()) {
    cfg.backend = have_jolt ? ph::backend::jolt : ph::backend::null;
    if (!have_jolt)
      warn(warnings, "ari: physics: this build has no Jolt; bodies are placed but never move "
                     "(the null backend)");
  } else {
    cfg.backend = have_jolt ? ph::backend::jolt : ph::backend::null;
    warn(warnings, "ari: physics: backend '" + spec.backend +
                       "' is not auto, jolt or null; using " + ph::backend_name(cfg.backend));
  }
  // One physics step is a whole number of the clock's quanta (a clock follower needs that).
  const double quantum = app.world_clock().fixed_dt();
  double steps = 1.0;
  if (spec.fixed_dt > 0.0) {
    steps = std::max(1.0, std::round(spec.fixed_dt / quantum));
    if (std::fabs(steps * quantum - spec.fixed_dt) > 1e-9)
      warn(warnings, "ari: physics: fixed_dt " + std::to_string(spec.fixed_dt) +
                         " s is not a whole number of the clock's " + std::to_string(quantum) +
                         " s quanta; using " + std::to_string(steps * quantum) + " s");
  }
  cfg.fixed_dt = steps * quantum;

  std::unique_ptr<SceneWorld> out(new SceneWorld());
  out->_impl->app = &app;
  if (!stats_key.empty()) {
    out->_impl->stats = stats_key + ".stats";
    out->_impl->enabled_key = stats_key + ".enabled";
    cvc::state &enabled = cvc::state::instance(app)(out->_impl->enabled_key);
    if (enabled.value().empty())
      enabled.value("1");
  }
  try {
    out->_impl->world = std::make_unique<ph::world>(cfg);
    out->_impl->cfg = cfg;
  } catch (const std::exception &e) {
    warn(warnings, std::string("ari: physics: ") + e.what());
    return nullptr;
  }
  ph::world &w = *out->_impl->world;

  std::map<std::string, ph::material_id> materials;
  for (const ScenePhysicsMaterial &m : spec.materials) {
    try {
      const ph::material made{m.name, static_cast<float>(m.friction),
                              static_cast<float>(m.restitution)};
      materials[m.name] = w.add_material(made);
      out->_impl->materials.push_back(made);
    } catch (const std::exception &e) {
      warn(warnings, "ari: physics: material '" + m.name + "': " + e.what());
    }
  }

  for (const SceneBodyInput &p : bodies) {
    const SceneNode &n = *p.spec;
    const SceneBody &b = n.body;
    const auto bad = [&](const std::string &m) {
      warn(warnings, "ari: scene node '" + n.id + "': body: " + m);
    };
    if (p.colliders || !p.data_path.empty()) {
      // Colliders made at run time: a generated world's now, a dataset's once it is ready (sync).
      if (!is_identity(p.world))
        bad("its colliders stay where they were made; the node's transform does not move them");
      if (!b.shape.empty() && b.shape != "auto")
        bad("shape '" + b.shape + "' is ignored: the body is the colliders the node makes");
      if (!b.motion.empty() && b.motion != "static")
        bad("its colliders are static");
      impl::tracked t;
      t.node = n.id;
      t.item = n.source_item;
      t.data_path = p.data_path;
      t.colliders = p.colliders ? p.colliders : std::make_shared<SceneColliders>();
      out->_impl->tracked_nodes.push_back(std::move(t));
      impl::entry e;
      e.tracked = static_cast<int>(out->_impl->tracked_nodes.size()) - 1;
      out->_impl->entries.push_back(std::move(e));
      if (p.colliders && p.colliders->revision > 0)
        out->_impl->apply(out->_impl->tracked_nodes.back(), warnings);
      continue;
    }
    if (!p.geometry) {
      bad("needs a geometry node; skipped");
      continue;
    }
    const cvc::geometry &geom = *p.geometry;

    ph::vec3 t;
    ph::quat q;
    double s[3];
    bool skewed = false, mirrored = false;
    decompose(p.world.data(), t, q, s, skewed, mirrored);
    if (skewed || mirrored)
      bad(std::string("the node's world transform is ") + (skewed ? "skewed" : "mirrored") +
          "; the body follows its rotation and per-axis scale only");

    std::string motion = b.motion.empty() ? (n.has_heightfield ? "static" : "dynamic") : b.motion;
    if (motion != "dynamic" && motion != "kinematic" && motion != "static") {
      bad("motion '" + motion + "' is not dynamic, kinematic or static; using dynamic");
      motion = "dynamic";
    }
    std::string shape = b.shape.empty() ? "auto" : b.shape;
    if (shape == "auto")
      shape = n.has_heightfield                ? "terrain"
              : n.source_primitive == "box"    ? "box"
              : n.source_primitive == "sphere" ? "sphere"
              : n.source_primitive == "plane"  ? "plane"
              : motion == "static"             ? "mesh"
                                               : "hull";

    ph::body_desc d;
    d.motion = motion == "static"      ? ph::motion_type::static_
               : motion == "kinematic" ? ph::motion_type::kinematic
                                       : ph::motion_type::dynamic;
    // The shape's centre in the node's (scaled) frame: the body sits there, and the node is drawn
    // at body pose x translate(-centre) x scale.
    ph::vec3 centre;
    const cvc::bounding_box e = geom.extents();
    const double half[3] = {0.5 * (e.maxx - e.minx) * s[0], 0.5 * (e.maxy - e.miny) * s[1],
                            0.5 * (e.maxz - e.minz) * s[2]};
    const ph::vec3 mid{0.5 * (e.minx + e.maxx) * s[0], 0.5 * (e.miny + e.maxy) * s[1],
                       0.5 * (e.minz + e.maxz) * s[2]};
    bool placed_by_grid = false;
    if (shape == "terrain") {
      auto it = fields.find(n.id);
      if (it == fields.end() || !it->second) {
        bad("shape 'terrain' needs a heightfield source; skipped");
        continue;
      }
      if (d.motion != ph::motion_type::static_)
        bad("a terrain is static; making it static");
      d.motion = ph::motion_type::static_;
      const bool rotated = std::fabs(std::fabs(q.w) - 1.0) > 1e-9;
      if (rotated || !near_one(s[0]) || !near_one(s[1]) || !near_one(s[2]))
        bad("a terrain collides unrotated and unscaled, at the node's position; draw it without "
            "rotation or scale so the ground you see is the ground bodies rest on");
      std::shared_ptr<const cvc::height_field> field = it->second;
      if (t.x != 0.0 || t.y != 0.0 || t.z != 0.0) {
        // The same grid moved to the node's position (the field itself stays the node's own).
        std::vector<float> h = field->heights();
        for (float &z : h)
          z += static_cast<float>(t.z);
        field = std::make_shared<cvc::height_field>(field->rows(), field->cols(), field->x0() + t.x,
                                                    field->y0() + t.y, field->dx(), field->dy(),
                                                    std::move(h), field->cell_materials());
      }
      d.shape = ph::terrain{field};
      out->_impl->fields[n.id] = field;
      placed_by_grid = true;
    } else if (shape == "box") {
      if (n.source_primitive == "box")
        d.shape = ph::box{
            {0.5 * n.box_size[0] * s[0], 0.5 * n.box_size[1] * s[1], 0.5 * n.box_size[2] * s[2]}};
      else {
        d.shape =
            ph::box{{std::max(half[0], 1e-3), std::max(half[1], 1e-3), std::max(half[2], 1e-3)}};
        centre = mid;
      }
    } else if (shape == "sphere") {
      if (!near_one(s[1] / s[0]) || !near_one(s[2] / s[0]))
        bad("a sphere scales uniformly; using the largest scale");
      const double smax = std::max(s[0], std::max(s[1], s[2]));
      if (n.source_primitive == "sphere") {
        d.shape = ph::sphere{n.sphere_radius * smax};
      } else {
        d.shape = ph::sphere{std::max({half[0], half[1], half[2], 1e-3})};
        centre = mid;
      }
    } else if (shape == "plane") {
      // A thin slab under the plane, its top face on the plane.
      const double thick = 0.05;
      d.shape = ph::box{{0.5 * n.plane_size * s[0], 0.5 * n.plane_size * s[1], thick}};
      centre = {0.0, 0.0, -thick};
    } else if (shape == "hull" || shape == "mesh") {
      if (shape == "mesh" && d.motion != ph::motion_type::static_) {
        bad("a mesh body is static; using its convex hull");
        shape = "hull";
      }
      if (shape == "hull") {
        ph::convex_hull h;
        for (const auto &pt : geom.const_points())
          h.points.push_back({pt[0] * s[0], pt[1] * s[1], pt[2] * s[2]});
        d.shape = std::move(h);
      } else {
        auto scaled = std::make_shared<cvc::geometry>(geom);
        if (!near_one(s[0]) || !near_one(s[1]) || !near_one(s[2]))
          for (auto &pt : scaled->points())
            pt = {pt[0] * s[0], pt[1] * s[1], pt[2] * s[2]};
        d.shape = ph::mesh{scaled};
      }
    } else {
      bad("shape '" + shape + "' is not auto, box, sphere, hull, mesh or terrain; skipped");
      continue;
    }

    if (!placed_by_grid) {
      d.position = t + rotate(q, centre);
      d.rotation = q;
    }
    d.mass = b.mass;
    d.density = b.density;
    if (!b.material.empty()) {
      auto mt = materials.find(b.material);
      if (mt != materials.end())
        d.material = mt->second;
      else
        bad("no physics material called '" + b.material + "'; using the default");
    }
    d.linear_velocity = {b.velocity[0], b.velocity[1], b.velocity[2]};
    d.angular_velocity = {b.angular_velocity[0], b.angular_velocity[1], b.angular_velocity[2]};
    if (b.has_linear_damping)
      d.linear_damping = b.linear_damping;
    if (b.has_angular_damping)
      d.angular_damping = b.angular_damping;

    ph::body_id id;
    try {
      id = w.add_body(d);
    } catch (const std::exception &ex) {
      bad(ex.what());
      continue;
    }
    SceneWorldBody made;
    made.node = n.id;
    made.id = id;
    made.moves = d.motion != ph::motion_type::static_;
    // body_from_visual = translate(-centre) x scale, row-major.
    made.body_from_visual = {s[0], 0, 0,    -centre.x, 0, s[1], 0, -centre.y,
                             0,    0, s[2], -centre.z, 0, 0,    0, 1};
    out->_impl->bodies.push_back(made);
    impl::entry ent;
    ent.desc = d;
    ent.body = made;
    out->_impl->entries.push_back(std::move(ent));
    // The node's footprint: its mesh's box, placed (a flat ground's grid covers it).
    std::array<double, 4> &fp =
        out->_impl->footprints.emplace(n.id, std::array<double, 4>{1e300, 1e300, -1e300, -1e300})
            .first->second;
    for (int k = 0; k < 8; ++k) {
      const double x = (k & 1) ? e.maxx : e.minx, y = (k & 2) ? e.maxy : e.miny,
                   z = (k & 4) ? e.maxz : e.minz;
      const double *m = p.world.data();
      const double wx = m[0] * x + m[1] * y + m[2] * z + m[3];
      const double wy = m[4] * x + m[5] * y + m[6] * z + m[7];
      fp = {std::min(fp[0], wx), std::min(fp[1], wy), std::max(fp[2], wx), std::max(fp[3], wy)};
    }
  }
  return out;
}

void SceneWorld::restart(std::vector<std::string> *warnings) {
  impl &d = *_impl;
  // The same world object, built afresh: a view holding it keeps a valid reference.
  *d.world = ph::world(d.cfg);
  for (const ph::material &m : d.materials)
    d.world->add_material(m);
  d.bodies.clear();
  for (impl::tracked &t : d.tracked_nodes)
    t.applied = 0;
  for (const impl::entry &e : d.entries) {
    if (e.tracked >= 0) { // the node's colliders as they are now (sync applies later ones)
      impl::tracked &t = d.tracked_nodes[static_cast<std::size_t>(e.tracked)];
      if (t.colliders->revision > 0 && !d.off.count(t.node))
        d.apply(t, warnings);
      continue;
    }
    if (d.off.count(e.body.node)) // a world that is off keeps out
      continue;
    SceneWorldBody b = e.body;
    try {
      b.id = d.world->add_body(e.desc);
    } catch (const std::exception &ex) {
      warn(warnings, "ari: scene node '" + b.node + "': body: " + ex.what());
      continue;
    }
    d.bodies.push_back(b);
  }
  d.follower.reset(); // the next follow() anchors on the clock again
  d.last = ph::step_report{};
  d.step_history.clear();
  publish_stats();
}

std::unique_ptr<SceneWorld> SceneWorld::from_scene(cvc::app &app, const Scene &scene,
                                                   const std::string &bind_prefix,
                                                   const std::string &stats_key,
                                                   std::vector<std::string> *warnings) {
  if (!scene_has_world(scene)) {
    // A data node's bodies need the physics: block (as cvcGL says when it draws the dataset).
    const std::function<void(const SceneNode &)> data = [&](const SceneNode &n) {
      if (n.body.present && !n.source_data.empty())
        warn(warnings, "ari: scene node '" + n.id + "': " + kDataNeedsPhysics);
      for (const SceneNode &c : n.children)
        data(c);
    };
    for (const SceneNode &n : scene.nodes)
      data(n);
    return nullptr;
  }
  std::map<const SceneNode *, std::array<double, 16>> matrices;
  scene_world_matrices(scene, matrices);

  std::vector<SceneBodyInput> inputs;
  std::map<std::string, std::shared_ptr<const cvc::height_field>> fields;
  std::map<std::string, std::shared_ptr<const cvc::geometry>> assets; // read once each
  std::set<std::string> failed_assets;

  std::vector<std::unique_ptr<GeneratedWorldRuntime>> generated;
  const std::function<void(const SceneNode &)> visit = [&](const SceneNode &n) {
    if (n.type == "generated_world" && n.source_data.empty()) {
      // The world runs the node: its world made now, its keys seeded as a renderer's would be.
      generated.push_back(std::make_unique<GeneratedWorldRuntime>(app, n, bind_prefix, warnings));
      if (n.body.present) {
        SceneBodyInput in;
        in.spec = &n;
        in.world = matrices[&n];
        in.colliders = generated.back()->colliders();
        inputs.push_back(std::move(in));
      }
    } else if (n.body.present && !n.source_data.empty()) {
      SceneBodyInput in;
      in.spec = &n;
      in.world = matrices[&n];
      in.data_path = data_path(bind_prefix, n);
      DataStore::of(app)->touch(in.data_path); // a lazy dataset loads now
      inputs.push_back(std::move(in));
    } else if (n.body.present) {
      SceneBodyInput in;
      in.spec = &n;
      in.world = matrices[&n];
      if (n.type == "geometry") {
        if (!n.source_asset.empty()) {
          auto hit = assets.find(n.source_asset);
          if (hit != assets.end()) {
            in.geometry = hit->second;
          } else if (!failed_assets.count(n.source_asset)) {
            const SceneAsset *asset = nullptr;
            for (const SceneAsset &a : scene.assets)
              if (a.id == n.source_asset)
                asset = &a;
            cvc::geometry g;
            std::shared_ptr<const cvc::height_field> field;
            if (!asset) {
              warn(warnings, "ari: scene node '" + n.id + "': no asset '" + n.source_asset +
                                 "' in the document's assets:");
              failed_assets.insert(n.source_asset);
            } else if (read_scene_node_geometry(asset->source, g, field, warnings)) {
              in.geometry = std::make_shared<const cvc::geometry>(std::move(g));
              assets[n.source_asset] = in.geometry;
            } else {
              failed_assets.insert(n.source_asset);
            }
          }
        } else {
          cvc::geometry g;
          std::shared_ptr<const cvc::height_field> field;
          if (read_scene_node_geometry(n, g, field, warnings)) {
            in.geometry = std::make_shared<const cvc::geometry>(std::move(g));
            if (field)
              fields[n.id] = field;
          }
        }
        if (in.geometry)
          inputs.push_back(std::move(in));
      } else {
        warn(warnings, "ari: scene node '" + n.id + "' (" + n.type +
                           "): body: only a geometry node's body runs without a renderer; "
                           "skipped");
      }
    }
    for (const SceneNode &c : n.children)
      visit(c);
  };
  for (const SceneNode &n : scene.nodes)
    visit(n);
  std::unique_ptr<SceneWorld> out = build(app, scene.physics, inputs, fields, stats_key, warnings);
  if (out) {
    out->_impl->generated = std::move(generated);
    out->bind_worlds(scene, resolve_bind(bind_prefix, "scene.world"), warnings);
  }
  return out;
}

std::vector<ph::body_id> SceneWorld::bodies_of(const std::string &id) const {
  std::vector<ph::body_id> out;
  for (const SceneWorldBody &b : _impl->bodies)
    if (b.node == id)
      out.push_back(b.id);
  return out;
}

std::shared_ptr<const cvc::world::generated_world>
SceneWorld::generated_world(const std::string &id) const {
  for (const auto &g : _impl->generated)
    if (g->node() == id)
      return g->world();
  return nullptr;
}

std::shared_ptr<const cvc::world::world_geometry>
SceneWorld::world_geometry(const std::string &id, std::uint64_t *revision) const {
  for (const impl::tracked &t : _impl->tracked_nodes)
    if (t.node == id && t.colliders) {
      if (revision)
        *revision = t.colliders->revision;
      return t.colliders->geometry;
    }
  if (revision)
    *revision = 0;
  return nullptr;
}

std::shared_ptr<const cvc::nav::world_grid> SceneWorld::grid(const std::string &id, double cell,
                                                             double half,
                                                             std::vector<std::string> *warnings) {
  std::uint64_t rev = 0;
  std::shared_ptr<const cvc::world::world_geometry> geometry = world_geometry(id, &rev);
  const auto field = _impl->fields.find(id);
  const auto footprint = _impl->footprints.find(id);
  if (!geometry && field == _impl->fields.end() && footprint == _impl->footprints.end())
    return nullptr;
  bool tracked = false;
  for (const impl::tracked &t : _impl->tracked_nodes)
    tracked = tracked || t.node == id;
  if (tracked && !geometry)
    return nullptr; // its world is still to come
  impl::grid_cache &c = _impl->grids[id];
  if (c.made && c.revision == rev && c.cell == cell && c.half == half)
    return c.grid;
  try {
    if (!geometry && field != _impl->fields.end()) { // a terrain: its field, as its body has it
      auto terrain = std::make_shared<cvc::world::world_geometry>();
      terrain->add({id, field->second});
      geometry = terrain;
    }
    if (geometry) {
      c.grid =
          std::make_shared<const cvc::nav::world_grid>(cvc::nav::make_grid(*geometry, cell, half));
    } else { // a flat ground: an open walled grid over its footprint
      std::array<double, 4> f = footprint->second;
      if (half > 0.0) {
        const double cx = 0.5 * (f[0] + f[2]), cy = 0.5 * (f[1] + f[3]);
        const double h = std::min({half, 0.5 * (f[2] - f[0]), 0.5 * (f[3] - f[1])});
        f = {cx - h, cy - h, cx + h, cy + h};
      }
      c.grid = std::make_shared<const cvc::nav::world_grid>(
          cvc::nav::flat_grid(f[0], f[1], f[2], f[3], cell));
    }
  } catch (const std::exception &e) {
    if (warnings)
      warnings->push_back("node '" + id + "': no grid: " + e.what());
    c.grid = nullptr;
  }
  c.made = true;
  c.revision = rev;
  c.cell = cell;
  c.half = half;
  return c.grid;
}

void SceneWorld::sync(std::vector<std::string> *warnings) {
  impl &d = *_impl;
  if (!d.world_key.empty() && d.app) {
    // The source key: another world's name switches to it.
    cvc::state *k =
        cvc::state::instance(*d.app).findDescendant(d.world_key + cvc::state::SEPARATOR + "source");
    const std::string want = k ? k->value() : std::string();
    if (!want.empty() && want != d.on) {
      if (d.find_world(want)) {
        set_world(want, warnings);
      } else {
        if (want != d.refused)
          warn(warnings,
               "ari: scene.world: no world called '" + want + "'; '" + d.on + "' stays on");
        d.refused = want;
      }
    }
  }
  for (const auto &g : d.generated)
    g->tick(warnings);
  cvc::state *root = d.app ? &cvc::state::instance(*d.app) : nullptr;
  for (impl::tracked &t : d.tracked_nodes) {
    if (!t.data_path.empty() && root) {
      // A dataset that is ready and new (its first load, a reload, a replacement): its colliders.
      if (cvc::state *node = root->findThroughLinks(t.data_path)) {
        const std::string store = node->fullName();
        cvc::state *status = root->findDescendant(store + cvc::state::SEPARATOR + "status");
        if (status && status->value() == "ready") {
          cvc::state *rev = root->findDescendant(store + cvc::state::SEPARATOR + "rev");
          const std::string r = rev ? rev->value() : std::string();
          if (store != t.target || r != t.rev) {
            t.target = store;
            t.rev = r;
            t.colliders->geometry = dataset_colliders(*root, store, t.item, t.node);
            ++t.colliders->revision;
          }
        }
      }
    }
    if (t.colliders->revision != t.applied && !d.off.count(t.node))
      d.apply(t, warnings);
  }
  publish_worlds(false);
}

void SceneWorld::bind_worlds(const Scene &scene, const std::string &world_key,
                             std::vector<std::string> *warnings) {
  impl &d = *_impl;
  d.worlds.clear();
  const std::function<void(const SceneNode &)> visit = [&](const SceneNode &n) {
    if (!n.world.empty()) {
      impl::world_def *w = nullptr;
      for (impl::world_def &x : d.worlds)
        if (x.name == n.world)
          w = &x;
      if (!w) {
        d.worlds.push_back({n.world, {}, {}});
        w = &d.worlds.back();
      }
      w->nodes.push_back(n.id);
      w->kinds.push_back(n.type == "generated_world" && n.source_data.empty() ? "generated"
                         : !n.source_data.empty()                             ? "data"
                         : n.has_heightfield                                  ? "terrain"
                                                                              : "flat");
    }
    for (const SceneNode &c : n.children)
      visit(c);
  };
  for (const SceneNode &n : scene.nodes)
    visit(n);
  if (d.worlds.empty())
    return;
  d.world_key = world_key;
  d.grid_cell = scene.world.cell;
  d.grid_half = scene.world.half;
  // The key's world if it names one (a host or --set may have), else the block's, else the first.
  std::string want;
  if (d.app && !world_key.empty())
    if (cvc::state *k = cvc::state::instance(*d.app).findDescendant(
            world_key + cvc::state::SEPARATOR + "source"))
      want = k->value();
  if (!d.find_world(want)) {
    if (!scene.world.source.empty() && !d.find_world(scene.world.source))
      warn(warnings, "ari: scene: world: no node has world: " + scene.world.source +
                         "; starting with '" + d.worlds.front().name + "'");
    want = d.find_world(scene.world.source) ? scene.world.source : d.worlds.front().name;
  }
  d.on.clear();
  set_world(want, warnings);
}

const std::string &SceneWorld::world_on() const { return _impl->on; }

bool SceneWorld::set_world(const std::string &name, std::vector<std::string> *warnings) {
  impl &d = *_impl;
  const impl::world_def *target = d.find_world(name);
  if (!target) {
    warn(warnings, "ari: scene.world: no world called '" + name + "'");
    return false;
  }
  // The others' nodes out, then this one's back in (a node in two worlds stays).
  std::set<std::string> keep(target->nodes.begin(), target->nodes.end());
  for (const impl::world_def &w : d.worlds)
    for (const std::string &node : w.nodes)
      if (!keep.count(node) && d.off.insert(node).second)
        d.remove_bodies_of(node);
  for (const std::string &node : target->nodes)
    if (d.off.erase(node))
      d.add_bodies_of(node, warnings);
  d.on = name;
  d.refused.clear();
  publish_worlds(true);
  return true;
}

void SceneWorld::publish_worlds(bool force) {
  impl &d = *_impl;
  if (d.world_key.empty() || !d.app)
    return;
  const impl::world_def *w = d.find_world(d.on);
  if (!w)
    return;
  // What the world on is now: its name and each tracked node's world revision.
  std::string sig = d.on;
  bool ready = true;
  for (const std::string &node : w->nodes)
    for (const impl::tracked &t : d.tracked_nodes)
      if (t.node == node) {
        sig += "|" + node + "@" + std::to_string(t.colliders->revision);
        ready = ready && t.colliders->geometry != nullptr;
      }
  if (!force && sig == d.on_sig)
    return;
  d.on_sig = sig;
  ++d.world_rev;
  cvc::state &root = cvc::state::instance(*d.app);
  const auto put = [&](const std::string &leaf, const std::string &v) {
    root(d.world_key + cvc::state::SEPARATOR + leaf).value(v);
  };
  std::string names, nodes, kind;
  for (const impl::world_def &x : d.worlds)
    names += (names.empty() ? "" : "\n") + x.name;
  for (std::size_t i = 0; i < w->nodes.size(); ++i) {
    nodes += (i ? "," : "") + w->nodes[i];
    if (kind.empty() || w->kinds[i] == "generated" || w->kinds[i] == "data")
      kind = w->kinds[i]; // a world with a generated or data node is that
  }
  put("sources", names);
  cvc::state &source = root(d.world_key + cvc::state::SEPARATOR + "source");
  if (source.value() != d.on)
    source.value(d.on);
  put("node", nodes);
  put("kind", kind);
  for (const impl::world_def &x : d.worlds)
    for (const std::string &node : x.nodes)
      put("on" + std::string(cvc::state::SEPARATOR) + node,
          std::find(w->nodes.begin(), w->nodes.end(), node) != w->nodes.end() ? "1" : "0");
  std::string summary;
  if (ready && d.grid_cell > 0.0) {
    for (const std::string &node : w->nodes) {
      std::shared_ptr<const cvc::nav::world_grid> g =
          grid(node, d.grid_cell, d.grid_half, &d.warnings);
      if (!g)
        continue;
      put("grid.rows", std::to_string(g->rows));
      put("grid.cols", std::to_string(g->cols));
      put("grid.cell", num(g->cell()));
      const std::size_t occupied = g->occupied_count();
      put("grid.occupied", std::to_string(occupied));
      std::ostringstream os;
      os << g->rows << " x " << g->cols << " at " << num(g->cell()) << " m, "
         << (100 * occupied / std::max<std::size_t>(1, std::size_t(g->rows) * g->cols))
         << "% blocked";
      summary = os.str();
      break;
    }
  }
  put("grid.summary", summary);
  put("ready", ready ? "1" : "0");
  put("status", d.on + " (" + kind + ", " + nodes + "): " +
                    (ready            ? "ready"
                     : kind == "data" ? "waiting for its dataset"
                                      : "waiting for its world"));
  put("rev", std::to_string(d.world_rev)); // last: a watcher of rev sees the rest written
}

std::vector<std::string> SceneWorld::take_warnings() {
  std::vector<std::string> out;
  out.swap(_impl->warnings);
  return out;
}

std::uint64_t SceneWorld::follow(const cvc::world_clock &clock,
                                 const std::function<void()> &after_step) {
  impl &d = *_impl;
  sync(&d.warnings);
  if (!d.follower) {
    d.follower = cvc::clock_follower::for_step(clock, d.world->config().fixed_dt);
    if (after_step)
      after_step();
    return 0;
  }
  const std::uint64_t n = d.follower->take(clock);
  if (n > 0 && !d.stepping())
    return 0; // physics off: the steps due are let go, and the world holds still
  for (std::uint64_t i = 1; i <= n; ++i) {
    d.last = d.world->step();
    d.step_history.push_back(d.last.step_ms);
    if (d.step_history.size() > 120)
      d.step_history.pop_front();
    // A view draws between the last two ticks, not the first and the last of a batch.
    if (after_step && i + 1 >= n)
      after_step();
  }
  if (n > 0)
    publish_stats();
  return n;
}

void SceneWorld::publish_stats() {
  impl &d = *_impl;
  if (d.stats.empty() || !d.app)
    return;
  cvc::state &root = cvc::state::instance(*d.app);
  const auto put = [&](const char *leaf, const std::string &v) {
    root(d.stats + "." + leaf).value(v);
  };
  put("tick", std::to_string(d.world->tick()));
  put("time", num(d.world->time()));
  put("step_ms", num(d.last.step_ms));
  put("bodies", std::to_string(d.world->body_count()));
  put("active_bodies", std::to_string(d.last.active_bodies));
  put("contact_events", std::to_string(d.last.contacts));
  put("saturated", d.last.saturated ? "true" : "false");
  std::string history;
  for (const double ms : d.step_history)
    history += (history.empty() ? "" : ",") + num(ms);
  put("step_ms_history", history);
  // A line per node with bodies, in the order they were made: its bodies and the ones awake.
  std::vector<std::string> order;
  std::map<std::string, std::pair<std::size_t, std::size_t>> per;
  for (const SceneWorldBody &b : d.bodies) {
    auto it = per.find(b.node);
    if (it == per.end()) {
      order.push_back(b.node);
      it = per.emplace(b.node, std::make_pair(std::size_t(0), std::size_t(0))).first;
    }
    ++it->second.first;
    if (b.moves && d.world->state(b.id).active)
      ++it->second.second;
  }
  std::string nodes;
  for (const std::string &node : order)
    nodes += (nodes.empty() ? "" : "\n") + node + "  " + std::to_string(per[node].first) + "  " +
             std::to_string(per[node].second);
  put("nodes", nodes);
}

} // namespace ariadne
} // namespace cvc

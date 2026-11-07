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
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
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
  if (n.body.present && n.source_data.empty())
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

} // namespace

bool scene_has_world(const Scene &scene) {
  if (scene.physics.present)
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
  if (!stats_key.empty())
    out->_impl->stats = stats_key + ".stats";
  try {
    out->_impl->world = std::make_unique<ph::world>(cfg);
  } catch (const std::exception &e) {
    warn(warnings, std::string("ari: physics: ") + e.what());
    return nullptr;
  }
  ph::world &w = *out->_impl->world;

  std::map<std::string, ph::material_id> materials;
  for (const ScenePhysicsMaterial &m : spec.materials) {
    try {
      materials[m.name] = w.add_material(
          {m.name, static_cast<float>(m.friction), static_cast<float>(m.restitution)});
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
  }
  return out;
}

std::unique_ptr<SceneWorld> SceneWorld::from_scene(cvc::app &app, const Scene &scene,
                                                   const std::string &stats_key,
                                                   std::vector<std::string> *warnings) {
  if (!scene_has_world(scene))
    return nullptr;
  std::map<const SceneNode *, std::array<double, 16>> matrices;
  scene_world_matrices(scene, matrices);

  std::vector<SceneBodyInput> inputs;
  std::map<std::string, std::shared_ptr<const cvc::height_field>> fields;
  std::map<std::string, std::shared_ptr<const cvc::geometry>> assets; // read once each
  std::set<std::string> failed_assets;

  const std::function<void(const SceneNode &)> visit = [&](const SceneNode &n) {
    if (n.body.present && n.source_data.empty()) {
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
  return build(app, scene.physics, inputs, fields, stats_key, warnings);
}

std::uint64_t SceneWorld::follow(const cvc::world_clock &clock,
                                 const std::function<void()> &after_step) {
  impl &d = *_impl;
  if (!d.follower) {
    d.follower = cvc::clock_follower::for_step(clock, d.world->config().fixed_dt);
    if (after_step)
      after_step();
    return 0;
  }
  const std::uint64_t n = d.follower->take(clock);
  for (std::uint64_t i = 1; i <= n; ++i) {
    d.last = d.world->step();
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
}

} // namespace ariadne
} // namespace cvc

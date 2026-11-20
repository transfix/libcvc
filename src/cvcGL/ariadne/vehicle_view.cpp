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
#include <cvc/ariadne/bind.h> // resolve_bind, read_or
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_spawns.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_labels.h> // entity_labels:
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/state/state.h>
#include <exception>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

ari::SceneNode box(const std::string &id, double lx, double ly, double lz, double x, double y,
                   double z, const float color[3]) {
  ari::SceneNode n;
  n.id = id;
  n.source_primitive = "box";
  n.box_size[0] = static_cast<float>(lx);
  n.box_size[1] = static_cast<float>(ly);
  n.box_size[2] = static_cast<float>(lz);
  n.has_material = true;
  std::copy(color, color + 3, n.color);
  n.ambient = 0.3f;
  n.diffuse = 0.8f;
  n.has_transform = true;
  n.position[0] = static_cast<float>(x);
  n.position[1] = static_cast<float>(y);
  n.position[2] = static_cast<float>(z);
  return n;
}

vehicle_shape shape_of(const cvc::physics::vehicle_desc &d) {
  vehicle_shape s;
  s.length = 2.0 * d.half_extents.x;
  s.width = 2.0 * d.half_extents.y;
  s.height = 2.0 * d.half_extents.z;
  s.wheelbase = d.wheelbase();
  s.track = d.track();
  s.wheel_radius = d.axles.empty() ? 0.4 : d.axles.front().wheel_radius;
  return s;
}

std::string get(cvc::state &root, const std::string &key) {
  cvc::state *k = root.findDescendant(key);
  return k ? k->value() : std::string();
}

double num(const std::string &s, double def) {
  try {
    return s.empty() ? def : std::stod(s);
  } catch (const std::exception &) {
    return def;
  }
}

void rgb(const std::string &s, float out[3]) {
  std::stringstream in(s);
  std::string part;
  for (int i = 0; i < 3 && std::getline(in, part, ','); ++i)
    out[i] = static_cast<float>(std::clamp(num(part, out[i]), 0.0, 1.0));
}

// A vehicle model made ready to draw on a chassis (fit_vehicle_geometry), as the nav demos'
// load_vehicle_template and pycvc_gl's VehicleModel have it, and its first base-colour texture.
// Read once per file and length.
struct fitted_model {
  cvc::geometry geometry;
  cvc::image texture;
};

} // namespace

// The models a realized scene has fitted, by file and length (null: it could not be read). The
// scene's own (RealizedScene::vehicle_models), on its render thread.
struct vehicle_model_cache {
  std::map<std::string, std::shared_ptr<const fitted_model>> models;
};

cvc::geometry fit_vehicle_geometry(cvc::geometry g, double length, const vehicle_fit_axes &axes) {
  const auto pts = g.const_points();
  if (pts.empty())
    throw std::runtime_error("it has no points");
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &v : pts)
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], static_cast<double>(v[k]));
      hi[k] = std::max(hi[k], static_cast<double>(v[k]));
    }
  const double ext[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
  const double big = std::max({ext[0], ext[1], ext[2]});
  const auto is_axis = [](int a) { return a >= 0 && a < 3; };
  int up = is_axis(axes.up) ? axes.up : -1;
  // The axis it stands on: its least coordinate 0 there, and on the other two it reaches past a
  // tenth of its extent each way (a file built on its ground plane, centred on it).
  for (int k = 0; k < 3 && up < 0; ++k) {
    bool stands = std::abs(lo[k]) <= 0.02 * big;
    for (int o = 0; o < 3; ++o)
      if (o != k && !(lo[o] < -0.1 * ext[o] && hi[o] > 0.1 * ext[o]))
        stands = false;
    if (stands)
      up = k;
  }
  if (up < 0) {
    up = 0;
    for (int k = 1; k < 3; ++k)
      if (ext[k] < ext[up])
        up = k;
  }
  int fwd = (up + 1) % 3, side = (up + 2) % 3;
  if (is_axis(axes.forward) && axes.forward != up) {
    fwd = axes.forward;
    side = 3 - up - fwd;
  } else if (ext[side] > ext[fwd]) {
    std::swap(fwd, side);
  }
  const double sc = ext[fwd] > 1e-9 ? length / ext[fwd] : 1.0;
  const double cf = 0.5 * (lo[fwd] + hi[fwd]), cs = 0.5 * (lo[side] + hi[side]);
  int front = axes.nose > 0 ? 1 : (axes.nose < 0 ? -1 : 0);
  if (front == 0) {
    long n_pos = 0, n_neg = 0, tt_pos = 0, tt_neg = 0;
    double top_pos = 0.0, top_neg = 0.0;
    const double top = lo[up] + 0.6 * ext[up];
    for (const auto &v : pts) {
      const double d = v[fwd] - cf;
      if (d > 0)
        ++n_pos;
      else
        ++n_neg;
      if (v[up] >= top) {
        if (d > 0) {
          top_pos += d;
          ++tt_pos;
        } else {
          top_neg += d;
          ++tt_neg;
        }
      }
    }
    front = n_pos < n_neg ? 1 : -1;
    if (tt_pos > 8 && tt_neg > 8)
      front = top_pos / tt_pos < std::abs(top_neg / tt_neg) ? 1 : -1;
  }
  // A rotation, never a mirror: an odd permutation of the axes (or a nose turned round) reflects
  // the mesh, which mirrors it and turns its triangles inside out. Flip the side axis to undo it.
  const bool even = (fwd + 1) % 3 == side; // (fwd, side, up) a cyclic order of (0, 1, 2)
  const double flip = (even ? 1.0 : -1.0) * front > 0 ? 1.0 : -1.0;
  auto &moved = g.points();
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const auto &v = pts[i];
    moved[i][0] = (v[fwd] - cf) * sc * front;
    moved[i][1] = (v[side] - cs) * sc * flip;
    moved[i][2] = (v[up] - lo[up]) * sc;
  }
  const auto nn = g.const_normals();
  if (nn.size() == pts.size()) {
    auto &turned = g.normals();
    for (std::size_t i = 0; i < nn.size(); ++i) {
      turned[i][0] = nn[i][fwd] * front;
      turned[i][1] = nn[i][side] * flip;
      turned[i][2] = nn[i][up];
    }
  }
  return g;
}

namespace {

std::shared_ptr<const fitted_model> fitted(cvc::app &app, vehicle_model_cache &cache,
                                           const std::string &path, double length,
                                           std::vector<std::string> *warnings) {
  const std::string key = path + "|" + std::to_string(length);
  if (auto it = cache.models.find(key); it != cache.models.end())
    return it->second;
  std::shared_ptr<fitted_model> out;
  try {
    cvc::model m = cvc::read_model(app, path);
    cvc::geometry g = m.merged();
    if (g.num_tris() == 0)
      throw std::runtime_error("it has no triangles");
    auto f = std::make_shared<fitted_model>();
    for (const auto &mat : m.materials)
      if (mat.has_base_color_texture()) {
        f->texture = mat.base_color_texture;
        break;
      }
    f->geometry = fit_vehicle_geometry(std::move(g), length);
    out = std::move(f);
  } catch (const std::exception &e) {
    if (warnings)
      warnings->push_back("ari: vehicle model '" + path + "': " + e.what() + "; drawn as its box");
  }
  cache.models[key] = out;
  return out;
}

// The fitted model under `group`: its base `base_z` below the group's origin (the ground under a
// chassis), turned and lifted as the look says. False when it cannot be read.
bool attach_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                  const std::string &id, const ari::vehicle_look &look, const vehicle_shape &s,
                  double base_z, const float color[3], std::vector<std::string> *warnings) {
  std::shared_ptr<const fitted_model> m = fitted(app, cache, look.path, s.length, warnings);
  if (!m)
    return false;
  std::shared_ptr<GeometryNode> node = group.createChild<GeometryNode>(id, m->geometry);
  if (!node)
    return false;
  node->setPosition(0.0, 0.0, base_z + look.z);
  node->setRotation(0.0, 0.0, look.yaw);
  node->setScale(look.scale, look.scale, look.scale);
  if (!m->texture.empty()) {
    node->setUseSingleColor(false);
    node->setTexture(m->texture, false);
  } else {
    node->setUseSingleColor(true);
    node->setColor(color[0], color[1], color[2]);
  }
  node->setAmbient(0.3);
  node->setDiffuse(0.8);
  return true;
}

} // namespace

std::shared_ptr<vehicle_model_cache> make_vehicle_model_cache() {
  return std::make_shared<vehicle_model_cache>();
}

bool attach_vehicle_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                          const std::string &id, const ari::vehicle_look &look,
                          const vehicle_shape &shape, double base_z, const float color[3],
                          std::vector<std::string> *warnings) {
  return attach_model(app, cache, group, id, look, shape, base_z, color, warnings);
}

ari::SceneNode vehicle_scene_node(const std::string &id, const ari::vehicle_look &look,
                                  const vehicle_shape &s, const float color[3],
                                  const float color2[3]) {
  ari::SceneNode g;
  g.id = id;
  g.type = "group";
  if (look.kind == "file" && look.fit)
    return g; // the fitted model goes in after (attach_model)
  if (look.kind == "file" && !look.path.empty()) {
    ari::SceneNode m;
    m.id = id + "_mesh";
    m.source_file = look.path;
    m.has_transform = true;
    m.position[2] = static_cast<float>(look.z);
    m.rotation[2] = static_cast<float>(look.yaw);
    m.scale[0] = m.scale[1] = m.scale[2] = static_cast<float>(look.scale);
    g.children.push_back(m);
    return g;
  }
  const double L = s.length, W = s.width, H = s.height;
  g.children.push_back(box(id + "_body", L, W, H, 0.0, 0.0, 0.0, color));
  g.children.push_back(
      box(id + "_cab", 0.42 * L, 0.9 * W, 0.75 * H, -0.12 * L, 0.0, 0.5 * H + 0.375 * H, color2));
  static const float tyre[3] = {0.07f, 0.07f, 0.08f};
  const double r = s.wheel_radius, ww = std::max(0.02, 0.16 * W);
  int k = 0;
  for (const double x : {0.5 * s.wheelbase, -0.5 * s.wheelbase})
    for (const double y : {0.5 * s.track, -0.5 * s.track})
      g.children.push_back(
          box(id + "_wheel" + std::to_string(k++), 2.0 * r, ww, 2.0 * r, x, y, -0.5 * H, tyre));
  return g;
}

std::shared_ptr<GeometryNode> add_hull_node(SceneGraph &sg, const std::string &id,
                                            GraphicsNode *parent,
                                            const cvc::physics::boat_desc &boat,
                                            const float topsides[3], const float stripe[3]) {
  if (!boat.hull.hull || boat.hull.hull->triangles.empty())
    return nullptr;
  const cvc::physics::hull_mesh &h = *boat.hull.hull;
  // The waterline it floats at, body z (fresh water); its mass the engine's when the hull names
  // none (the shape's volume at its density).
  double waterline = 0.0;
  try {
    double mass = boat.hull.mass;
    if (!(mass > 0.0))
      if (const auto *b = std::get_if<cvc::physics::box>(&boat.hull.shape))
        mass = boat.hull.density * 8.0 * b->half_extents.x * b->half_extents.y * b->half_extents.z;
    waterline = -cvc::physics::dyn_boat_plant::float_height(boat, mass, 1000.0);
  } catch (const std::exception &) {
  }
  static const float bottom[3] = {0.36f, 0.08f, 0.07f}; // wet antifouling
  cvc::geometry g(sg.appContext());
  auto &pts = g.points();
  auto &nrm = g.normals();
  auto &col = g.colors();
  auto &tris = g.tris();
  using idx = cvc::geometry::index_t;
  // A box hull is drawn as an open boat: no deck, a sole (its cockpit's floor) 0.12 m over the
  // waterline, so the water it keeps out of its hull (the water node's exclusion) is what would
  // otherwise show in the cockpit on a wave.
  const auto *hull_box = std::get_if<cvc::physics::box>(&boat.hull.shape);
  double top = -1e300, low = 1e300, x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
  for (const cvc::physics::vec3 &v : h.vertices) {
    top = std::max(top, v.z);
    low = std::min(low, v.z);
    x0 = std::min(x0, v.x);
    x1 = std::max(x1, v.x);
    y0 = std::min(y0, v.y);
    y1 = std::max(y1, v.y);
  }
  // Each triangle its own corners, so the faces shade flat.
  for (const auto &t : h.triangles) {
    const cvc::physics::vec3 &a = h.vertices[t[0]], &b = h.vertices[t[1]], &c = h.vertices[t[2]];
    const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 0.0) {
      nx /= len;
      ny /= len;
      nz /= len;
    }
    if (hull_box && nz > 0.9 && a.z >= top - 1e-9 && b.z >= top - 1e-9 && c.z >= top - 1e-9)
      continue; // the deck: open
    const idx base = static_cast<idx>(pts.size());
    for (const cvc::physics::vec3 *p : {&a, &b, &c}) {
      pts.push_back({p->x, p->y, p->z});
      nrm.push_back({nx, ny, nz});
      const double z = p->z - waterline;
      const float *rgb = z < 0.02 ? bottom : z < 0.10 ? stripe : topsides;
      col.push_back({rgb[0], rgb[1], rgb[2]});
    }
    tris.push_back({base, static_cast<idx>(base + 1), static_cast<idx>(base + 2)});
  }
  if (hull_box) { // the sole, teak, inside the hull's sides
    static const float teak[3] = {0.55f, 0.40f, 0.24f};
    const double z = std::min(std::max(waterline + 0.12, low + 0.05), top - 0.05), m = 0.04;
    const idx base = static_cast<idx>(pts.size());
    for (const auto &p :
         {std::array<double, 2>{x0 + m, y0 + m}, std::array<double, 2>{x1 - m, y0 + m},
          std::array<double, 2>{x1 - m, y1 - m}, std::array<double, 2>{x0 + m, y1 - m}}) {
      pts.push_back({p[0], p[1], z});
      nrm.push_back({0.0, 0.0, 1.0});
      col.push_back({teak[0], teak[1], teak[2]});
    }
    tris.push_back({base, static_cast<idx>(base + 1), static_cast<idx>(base + 2)});
    tris.push_back({base, static_cast<idx>(base + 2), static_cast<idx>(base + 3)});
  }
  std::shared_ptr<GeometryNode> node =
      parent ? parent->createChild<GeometryNode>(id, g)
             : std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(id, g));
  return node;
}

void sync_vehicle_markers(RealizedScene &r, std::vector<std::string> *warnings) {
  if (!r.world || !r.sg || !r.physics_binding)
    return;
  ari::WorldVehicles *v = r.world->vehicles();
  if (!v || v->revision() == r.vehicles_rev)
    return;
  r.vehicles_rev = v->revision();
  if (!r.vehicle_models)
    r.vehicle_models = std::make_shared<vehicle_model_cache>();
  const std::shared_ptr<vehicle_model_cache> models = r.vehicle_models;
  SceneGraph &sg = *r.sg;
  for (const auto &[node, chassis] : r.vehicle_markers) {
    if (chassis.valid() && r.physics_binding->bound(chassis))
      r.physics_binding->unbind(chassis);
    sg.removeGraphics(node);
  }
  r.vehicle_markers.clear();
  for (const ari::entity_info &e : v->entities()) {
    if (!e.placed)
      continue;
    const ari::vehicle_archetype *a = nullptr;
    for (const ari::vehicle_archetype &x : v->archetypes())
      if (x.id == e.spec.archetype)
        a = &x;
    if (!a)
      continue;
    const std::string id = "vehicle_" + std::to_string(e.id);
    if (ari::vehicle_plant_floats(a->plant) && (a->look.kind.empty() || a->look.kind == "box")) {
      // A boat: its hull, in its colours, bound to its hull body.
      std::shared_ptr<GraphicsNode> group = sg.addGraphics(id);
      try {
        const cvc::physics::dyn_boat_desc b = ari::boat_desc_of(*a, e.spec.mass, e.spec.max_speed);
        add_hull_node(sg, id + "_hull", group.get(), b.boat, e.spec.color.data(),
                      e.spec.color2.data());
        r.physics_binding->bind(e.chassis, group);
      } catch (const std::exception &ex) {
        if (warnings)
          warnings->push_back("ari: vehicle " + std::to_string(e.id) + ": " + ex.what());
      }
      apply_entity_label(sg, group, r.entity_labels,
                         {{"name", e.spec.name},
                          {"id", std::to_string(e.id)},
                          {"archetype", e.spec.archetype},
                          {"team", std::to_string(e.spec.team)}},
                         r.bind_prefix, warnings);
      r.vehicle_markers.emplace_back(id, e.chassis);
      continue;
    }
    const vehicle_shape shape = shape_of(ari::vehicle_desc_of(*a, e.spec.mass, e.spec.max_speed));
    ari::Scene one;
    one.nodes.push_back(
        vehicle_scene_node(id, a->look, shape, e.spec.color.data(), e.spec.color2.data()));
    realize_scene(sg, one, r.bind_prefix, warnings);
    std::shared_ptr<GraphicsNode> node = sg.getGraphics(id);
    if (!node)
      continue;
    if (a->look.kind == "file" && a->look.fit &&
        !attach_model(sg.appContext(), *models, *node, id + "_mesh", a->look, shape,
                      e.ground_z - e.z, e.spec.color.data(), warnings)) {
      sg.removeGraphics(id); // no model: its box
      ari::vehicle_look box;
      box.kind = "box";
      ari::Scene again;
      again.nodes.push_back(
          vehicle_scene_node(id, box, shape, e.spec.color.data(), e.spec.color2.data()));
      realize_scene(sg, again, r.bind_prefix, warnings);
      node = sg.getGraphics(id);
      if (!node)
        continue;
    }
    apply_entity_label(sg, node, r.entity_labels,
                       {{"name", e.spec.name},
                        {"id", std::to_string(e.id)},
                        {"archetype", e.spec.archetype},
                        {"team", std::to_string(e.spec.team)}},
                       r.bind_prefix, warnings);
    try {
      r.physics_binding->bind(e.chassis, node);
    } catch (const std::exception &ex) {
      if (warnings)
        warnings->push_back("ari: vehicle " + std::to_string(e.id) + ": " + ex.what());
    }
    r.vehicle_markers.emplace_back(id, e.chassis);
  }
}

void pose_vehicle_wheels(RealizedScene &r) {
  if (!r.world || !r.sg)
    return;
  ari::WorldVehicles *v = r.world->vehicles();
  if (!v || r.vehicle_markers.empty())
    return;
  SceneGraph &sg = *r.sg;
  for (const auto &[node, chassis] : r.vehicle_markers) {
    const std::shared_ptr<GraphicsNode> group = sg.getGraphics(node);
    const std::shared_ptr<GraphicsNode> first =
        group ? group->findChildByName(node + "_wheel0") : nullptr;
    if (!first) // a mesh look
      continue;
    const std::uint64_t id = std::stoull(node.substr(std::string("vehicle_").size()));
    const std::vector<ari::entity_wheel> wheels = v->wheels(id);
    if (wheels.empty())
      continue;
    // The look's four (front left, front right, rear left, rear right) each take the plant's wheel
    // nearest it.
    for (int k = 0; k < 4; ++k) {
      const std::shared_ptr<GraphicsNode> w =
          k == 0 ? first : group->findChildByName(node + "_wheel" + std::to_string(k));
      if (!w)
        continue;
      const double sx = k < 2 ? 1.0 : -1.0, sy = k % 2 == 0 ? 1.0 : -1.0;
      const ari::entity_wheel *best = nullptr;
      for (const ari::entity_wheel &e : wheels)
        if ((e.x >= 0.0) == (sx > 0.0) && (e.y >= 0.0) == (sy > 0.0) &&
            (!best || std::fabs(e.x) > std::fabs(best->x)))
          best = &e;
      if (!best)
        continue;
      // Turned about its own upright by its steer, at its centre, and rolled about its axle by its
      // spin (Rz(steer) Ry(spin)): the per-frame pose path (change-detected, published rather
      // than written).
      const double c = std::cos(best->steer), sn = std::sin(best->steer);
      if (best->spin == 0.0) {
        const double m[16] = {c,   -sn, 0.0, best->x, sn,  c,   0.0, best->y,
                              0.0, 0.0, 1.0, best->z, 0.0, 0.0, 0.0, 1.0};
        w->setPoseMatrix(m);
      } else {
        const double cs = std::cos(best->spin), ss = std::sin(best->spin);
        const double m[16] = {c * cs, -sn, c * ss, best->x, sn * cs, c,   sn * ss, best->y,
                              -ss,    0.0, cs,     best->z, 0.0,     0.0, 0.0,     1.0};
        w->setPoseMatrix(m);
      }
    }
  }
}

void sync_spawn_markers(RealizedScene &r, std::vector<std::string> *warnings) {
  if (!r.world || !r.sg)
    return;
  ari::WorldVehicles *v = r.world->vehicles();
  if (!v || v->spawns().revision() == r.spawns_rev)
    return;
  r.spawns_rev = v->spawns().revision();
  SceneGraph &sg = *r.sg;
  for (const std::string &id : r.spawn_markers)
    if (sg.getGraphics(id))
      sg.removeGraphics(id);
  r.spawn_markers.clear();
  static const float kTeams[8][3] = {
      {0.90f, 0.25f, 0.20f}, {0.20f, 0.45f, 0.90f}, {0.25f, 0.75f, 0.30f}, {0.95f, 0.75f, 0.15f},
      {0.65f, 0.30f, 0.85f}, {0.15f, 0.80f, 0.80f}, {0.95f, 0.50f, 0.15f}, {0.60f, 0.60f, 0.60f}};
  static const float kAny[3] = {0.95f, 0.95f, 0.95f}, kSuggested[3] = {0.55f, 0.55f, 0.58f};
  const auto marker = [&](const std::string &id, double x, double y, double z, double heading_deg,
                          const float color[3], bool blocked) {
    ari::SceneNode g;
    g.id = id;
    g.type = "group";
    g.tags = {"spawns"};
    g.has_transform = true;
    g.position[0] = static_cast<float>(x);
    g.position[1] = static_cast<float>(y);
    g.position[2] = static_cast<float>(z + 0.08);
    g.rotation[2] = static_cast<float>(heading_deg);
    const float dark[3] = {0.15f, 0.05f, 0.05f};
    g.children.push_back(box(id + "_pad", 3.0, 3.0, 0.12, 0.0, 0.0, 0.0, blocked ? dark : color));
    g.children.push_back(box(id + "_arrow", 2.2, 0.5, 0.16, 1.4, 0.0, 0.04, color));
    ari::Scene one;
    one.nodes.push_back(g);
    realize_scene(sg, one, r.bind_prefix, warnings);
    r.spawn_markers.push_back(id);
  };
  for (const ari::WorldSpawns::entry &e : v->spawns().points())
    marker("spawn_point_" + std::to_string(e.id), e.point.x, e.point.y, e.z, e.point.heading,
           e.point.team < 0 ? kAny : kTeams[e.point.team % 8], !e.free);
  int k = 0;
  for (const ari::spawn_point &p : v->spawns().suggestions()) {
    marker("spawn_suggest_" + std::to_string(k++), p.x, p.y, v->spawns().ground(p.x, p.y),
           p.heading, kSuggested, false);
  }
}

namespace {

std::shared_ptr<GraphicsNode> realize_vehicle_preview(SceneGraph &sg, const ari::SceneNode &n,
                                                      GraphicsNode *parent, RealizedScene &out,
                                                      std::vector<std::string> *warnings) {
  (void)warnings;
  std::shared_ptr<GraphicsNode> anchor = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  if (!anchor)
    return nullptr;
  cvc::app &app = sg.appContext();
  const std::string root = ari::resolve_bind(out.bind_prefix, n.props.str("vehicles", "vehicles"));
  const double spin = n.props.num("spin", 20.0);
  const std::string drawn = n.id + "__vehicle";
  const double at[3] = {n.position[0], n.position[1], n.position[2]};
  SceneGraph *graph = &sg;
  const std::string prefix = out.bind_prefix;
  auto last = std::make_shared<std::string>();
  if (!out.vehicle_models)
    out.vehicle_models = std::make_shared<vehicle_model_cache>();
  const std::shared_ptr<vehicle_model_cache> models = out.vehicle_models;
  std::weak_ptr<GraphicsNode> held = anchor;
  out.custom_ticks.push_back([=, &app](vtkRenderer *) {
    if (held.expired())
      return;
    cvc::state &st = cvc::state::instance(app);
    const std::string sel = get(st, root + ".selected");
    const std::string a = root + ".archetypes." + sel + ".";
    const std::string sig = sel + "|" + get(st, a + "length") + "|" + get(st, a + "width") + "|" +
                            get(st, a + "height") + "|" + get(st, a + "look.kind") + "|" +
                            get(st, a + "look.path") + "|" + get(st, a + "look.fit") + "|" +
                            get(st, root + ".spawn.color") + "|" + get(st, root + ".spawn.color2");
    if (sig != *last) {
      *last = sig;
      if (graph->getGraphics(drawn))
        graph->removeGraphics(drawn);
      if (!sel.empty() && !get(st, a + "length").empty()) {
        vehicle_shape s;
        s.length = num(get(st, a + "length"), s.length);
        s.width = num(get(st, a + "width"), s.width);
        s.height = num(get(st, a + "height"), s.height);
        s.wheelbase = num(get(st, a + "wheelbase"), s.wheelbase);
        s.track = num(get(st, a + "track"), s.track);
        s.wheel_radius = num(get(st, a + "wheel_radius"), s.wheel_radius);
        ari::vehicle_look look;
        look.kind = get(st, a + "look.kind");
        look.path = get(st, a + "look.path");
        look.scale = num(get(st, a + "look.scale"), 1.0);
        look.yaw = num(get(st, a + "look.yaw"), 0.0);
        look.z = num(get(st, a + "look.z"), 0.0);
        look.fit = get(st, a + "look.fit") == "1";
        float c1[3] = {0.85f, 0.7f, 0.2f}, c2[3] = {0.12f, 0.12f, 0.14f};
        rgb(get(st, root + ".spawn.color"), c1);
        rgb(get(st, root + ".spawn.color2"), c2);
        ari::Scene one;
        ari::SceneNode node = vehicle_scene_node(drawn, look, s, c1, c2);
        node.has_transform = true;
        // Standing on the set: the wheels' bottoms at the node's height.
        node.position[0] = static_cast<float>(at[0]);
        node.position[1] = static_cast<float>(at[1]);
        node.position[2] = static_cast<float>(at[2] + 0.5 * s.height + s.wheel_radius);
        const bool model = look.kind == "file" && look.fit;
        if (model) // the model stands on the set by itself: its base at the node's height
          node.position[2] = static_cast<float>(at[2]);
        one.nodes.push_back(node);
        realize_scene(*graph, one, prefix, nullptr);
        if (model)
          if (std::shared_ptr<GraphicsNode> g = graph->getGraphics(drawn))
            if (!attach_model(app, *models, *g, drawn + "_mesh", look, s, 0.0, c1, nullptr)) {
              graph->removeGraphics(drawn);
              look.kind = "box";
              ari::Scene again;
              ari::SceneNode b = vehicle_scene_node(drawn, look, s, c1, c2);
              b.has_transform = true;
              b.position[0] = static_cast<float>(at[0]);
              b.position[1] = static_cast<float>(at[1]);
              b.position[2] = static_cast<float>(at[2] + 0.5 * s.height + s.wheel_radius);
              again.nodes.push_back(b);
              realize_scene(*graph, again, prefix, nullptr);
            }
      }
    }
    if (std::shared_ptr<GraphicsNode> g = graph->getGraphics(drawn)) {
      const double t = app.world_clock().t();
      g->setRotation(0.0, 0.0, std::fmod(spin * t, 360.0));
    }
  });
  return anchor;
}

} // namespace

void register_vehicle_preview_node(cvc::app &app) {
  if (has_scene_node_type(app, "vehicle_preview"))
    return;
  register_scene_node_type(app, "vehicle_preview", realize_vehicle_preview);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

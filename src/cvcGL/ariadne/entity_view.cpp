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

// The entity view (entity_view.h): a set of entities drawn as one mesh per archetype and LOD rung,
// the nearest few promoted to their own nodes. The rungs are #1358's swarm agents'
// (swarm_nodes.cpp), posed in 3-D from each entity's pose rather than yawed on the plane.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // resolve_bind, read_or
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/entity_view.h>
#include <cvc/gl/ariadne/mesh_lod.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/image/image.h>
#include <cvc/lod/select.h>
#include <cvc/physics/pose_tracker.h>
#include <exception>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <vtkCamera.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;
namespace ph = cvc::physics;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPromote = 8;    // entities a layer draws as their own nodes
constexpr int kThreshold = 32; // a layer of at most this many draws them all so
constexpr int kNearCap = 10;   // the finest rung's entities
// An incumbent keeps its promotion until another is this much nearer (a fifth).
constexpr double kKeep = 0.8;

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// One bodies layer, as an entity_layer node declares it (or the implicit one).
struct layer_spec {
  std::string id;
  std::string of_kind = "all"; // all | fleet | archetype | team
  std::string of_value;
  bool lod = true;
  int near_cap = kNearCap;
  double px[3] = {24.0, 8.0, 3.0}; // the finer rungs' least radius on screen
  int promote = kPromote;
  int threshold = kThreshold;
  ari::SceneNode node; // its material: (opacity, specular, shadows)

  bool takes(const entity_draw_info &e) const {
    if (of_kind == "fleet")
      return e.fleet == of_value;
    if (of_kind == "archetype")
      return e.archetype == of_value;
    if (of_kind == "team")
      return std::to_string(e.team) == of_value;
    return true;
  }
};

// One rung of an archetype: `cap` copies of a template in one mesh, each where its entity is.
struct rung {
  std::vector<double> tv, tn;           // the template's points and normals [3V]
  std::vector<std::uint32_t> tt;        // its triangles
  std::vector<unsigned char> role, own; // [V], [3V]
  std::vector<float> uv;                // [2V] when textured
  int cap = 0;
  bool textured = false;
  std::weak_ptr<GeometryNode> node;
  std::vector<int> slots, prev;        // the members drawn, by slot
  std::size_t shown = ~std::size_t(0); // the slots it drew last
  std::vector<double> xyz, nrm;
  std::vector<unsigned char> rgb;

  int verts() const { return static_cast<int>(tv.size() / 3); }
};

// An entity a layer takes.
struct member {
  std::uint64_t id = 0;
  int arch = -1; // its archetype in the layer's archs
  unsigned char c1[3] = {0, 0, 0}, c2[3] = {0, 0, 0};
  bool promoted = false;
  bool posed = false;   // a pose since the layers were made (draw draws it)
  bool located = false; // x, y, z known (promote ranks it)
  double m[16] = {};    // its pose (row-major), the last given
  double x = 0.0, y = 0.0, z = 0.0;
};

// An archetype's draw in a layer: its rungs and its members.
struct arch_draw {
  std::string id;
  double radius = 1.0; // the chassis's bounding radius
  std::vector<rung> rungs;
  std::vector<int> members;
};

struct bodies_layer {
  layer_spec spec;
  bool implicit = false;
  std::weak_ptr<GraphicsNode> group;
  std::vector<member> members;
  std::vector<arch_draw> archs;
  bool instancing = false; // more than its threshold: the rungs draw the unpromoted
  std::vector<std::pair<float, int>> score;
};

unsigned char u8(float c) {
  return static_cast<unsigned char>(std::lround(std::clamp(c, 0.0f, 1.0f) * 255.0f));
}

// A template's normals from its triangles (area-weighted), for a decimated rung.
std::vector<double> normals_of(const std::vector<double> &v, const std::vector<std::uint32_t> &t) {
  std::vector<double> n(v.size(), 0.0);
  for (std::size_t k = 0; k + 2 < t.size(); k += 3) {
    const double *a = &v[3 * t[k]], *b = &v[3 * t[k + 1]], *c = &v[3 * t[k + 2]];
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
                 w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const double f[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                         u[0] * w[1] - u[1] * w[0]};
    for (int j = 0; j < 3; ++j)
      for (int a2 = 0; a2 < 3; ++a2)
        n[3 * t[k + j] + a2] += f[a2];
  }
  for (std::size_t i = 0; i + 2 < n.size(); i += 3) {
    const double len = std::sqrt(n[i] * n[i] + n[i + 1] * n[i + 1] + n[i + 2] * n[i + 2]);
    for (int a2 = 0; a2 < 3; ++a2)
      n[i + a2] = len > 0.0 ? n[i + a2] / len : (a2 == 2 ? 1.0 : 0.0);
  }
  return n;
}

// A bodies layer's spec from an entity_layer node.
layer_spec spec_of(const ari::SceneNode &n, std::vector<std::string> *warnings) {
  layer_spec s;
  s.id = n.id;
  s.node = n;
  s.node.children.clear();
  const ari::Value &p = n.props;
  if (const ari::Value *of = p.find("of")) {
    if (of->is_scalar() && (of->scalar == "all" || of->scalar.empty())) {
      s.of_kind = "all";
    } else if (of->is_map() && of->entries.size() == 1 &&
               (of->entries[0].first == "fleet" || of->entries[0].first == "archetype" ||
                of->entries[0].first == "team")) {
      s.of_kind = of->entries[0].first;
      s.of_value = of->entries[0].second.as_string();
    } else {
      warn(warnings, "ari: entity_layer '" + n.id +
                         "': of: is all, { fleet: NAME }, { archetype: ID } or { team: N }; it "
                         "takes all");
    }
  }
  if (const ari::Value *lod = p.find("lod")) {
    if (lod->is_scalar()) {
      s.lod = lod->as_bool(true);
    } else if (lod->is_map()) {
      s.near_cap = std::max(1, static_cast<int>(lod->num("near_cap", s.near_cap)));
      if (const ari::Value *px = lod->find("px"); px && px->is_seq())
        for (std::size_t k = 0; k < 3 && k < px->items.size(); ++k)
          s.px[k] = std::max(0.0, px->items[k].as_double(s.px[k]));
    }
  }
  s.promote = std::max(0, static_cast<int>(p.num("promote", s.promote)));
  s.threshold = std::max(0, static_cast<int>(p.num("threshold", s.threshold)));
  return s;
}

std::string rung_name(const layer_spec &s, const arch_draw &a, std::size_t k) {
  return s.id + "_" + a.id + "_lod" + std::to_string(k);
}

// The rungs of one archetype under `group`: a fitted model's four (the finest textured), else one.
void make_rungs(cvc::app &app, bodies_layer &L, arch_draw &a, const vehicle_look_mesh &mesh,
                GraphicsNode &group) {
  const int n = static_cast<int>(a.members.size());
  struct spec {
    double reduction;
    int cap;
  };
  std::vector<spec> specs;
  if (mesh.model && L.spec.lod)
    specs = {{0.00, std::max(1, std::min(n, L.spec.near_cap))},
             {0.62, std::min(n, 28)},
             {0.88, std::min(n, 72)},
             {0.985, n}};
  else
    specs = {{0.0, n}};
  a.rungs.assign(specs.size(), rung{});
  for (std::size_t k = 0; k < specs.size(); ++k) {
    rung &r = a.rungs[k];
    r.cap = std::max(1, specs[k].cap);
    r.tv = mesh.verts;
    r.tt = mesh.tris;
    r.tn = mesh.normals;
    r.role = mesh.role;
    r.own = mesh.own;
    r.textured = k == 0 && mesh.texture && !mesh.uvs.empty();
    if (r.textured)
      r.uv = mesh.uvs;
    if (specs[k].reduction > 0.0) {
      // Each colour role's triangles decimated on their own, so a wheeled look's wheels keep a
      // tyre's dark; a role's that collapse are kept as they are.
      std::vector<double> rv;
      std::vector<std::uint32_t> rt;
      std::vector<unsigned char> rrole, rown;
      bool coarser = false;
      std::vector<unsigned char> roles;
      for (std::size_t t = 0; t + 2 < mesh.tris.size(); t += 3) {
        const unsigned char q = mesh.role[mesh.tris[t]];
        if (std::find(roles.begin(), roles.end(), q) == roles.end())
          roles.push_back(q);
      }
      for (const unsigned char q : roles) {
        std::vector<std::int64_t> at(mesh.verts.size() / 3, -1);
        std::vector<std::uint32_t> from; // each of the part's points, its point in the mesh
        std::vector<double> pv;
        std::vector<std::uint32_t> pt;
        for (std::size_t t = 0; t + 2 < mesh.tris.size(); t += 3) {
          if (mesh.role[mesh.tris[t]] != q)
            continue;
          for (int c = 0; c < 3; ++c) {
            const std::uint32_t v = mesh.tris[t + c];
            if (at[v] < 0) {
              at[v] = static_cast<std::int64_t>(from.size());
              from.push_back(v);
              pv.insert(pv.end(),
                        {mesh.verts[3 * v], mesh.verts[3 * v + 1], mesh.verts[3 * v + 2]});
            }
            pt.push_back(static_cast<std::uint32_t>(at[v]));
          }
        }
        std::vector<double> dv;
        std::vector<std::uint32_t> dt;
        const bool ok = decimate_mesh(pv, pt, specs[k].reduction, dv, dt);
        coarser = coarser || ok;
        const std::vector<double> &uv = ok ? dv : pv;
        const std::vector<std::uint32_t> &ut = ok ? dt : pt;
        const auto base = static_cast<std::uint32_t>(rv.size() / 3);
        rv.insert(rv.end(), uv.begin(), uv.end());
        for (const std::uint32_t i : ut)
          rt.push_back(base + i);
        const std::size_t n = uv.size() / 3;
        rrole.insert(rrole.end(), n, q);
        for (std::size_t i = 0; i < n; ++i) { // a decimated part's points take its first's colour
          const std::uint32_t v = from[ok ? 0 : i];
          rown.insert(rown.end(), {mesh.own[3 * v], mesh.own[3 * v + 1], mesh.own[3 * v + 2]});
        }
      }
      if (coarser) {
        r.tv = std::move(rv);
        r.tt = std::move(rt);
        r.tn = normals_of(r.tv, r.tt);
        r.role = std::move(rrole);
        r.own = std::move(rown);
      }
    }
    if (!r.textured) // untextured: the model's points in its entity's colour
      std::replace(r.role.begin(), r.role.end(), static_cast<unsigned char>(3),
                   static_cast<unsigned char>(0));
    const int V = r.verts();
    const std::size_t total = static_cast<std::size_t>(r.cap) * static_cast<std::size_t>(V);
    cvc::geometry g(app);
    auto &pts = g.points();
    auto &cols = g.colors();
    auto &nrm = g.normals();
    auto &tris = g.tris();
    pts.reserve(total);
    cols.reserve(total);
    nrm.reserve(total);
    if (r.textured)
      g.uvs().reserve(total);
    for (int i = 0; i < r.cap; ++i) {
      const auto base =
          static_cast<cvc::geometry::index_t>(i) * static_cast<cvc::geometry::index_t>(V);
      for (int q = 0; q < V; ++q) {
        pts.push_back({0.0, 0.0, 0.0}); // collapsed until a draw places it
        nrm.push_back({r.tn[3 * q], r.tn[3 * q + 1], r.tn[3 * q + 2]});
        cols.push_back({1.0, 1.0, 1.0});
        if (r.textured)
          g.uvs().push_back({r.uv[2 * q], r.uv[2 * q + 1]});
      }
      for (std::size_t t = 0; t + 2 < r.tt.size(); t += 3)
        tris.push_back({base + r.tt[t], base + r.tt[t + 1], base + r.tt[t + 2]});
    }
    r.xyz.assign(total * 3, 0.0);
    r.nrm.assign(total * 3, 0.0);
    r.rgb.assign(total * 3, 255);
    const std::string name = rung_name(L.spec, a, k);
    if (std::shared_ptr<GraphicsNode> old = group.findChildByName(name))
      group.removeGraphicsChild(old);
    std::shared_ptr<GeometryNode> node = group.createChild<GeometryNode>(name, g);
    if (!node)
      continue;
    node->setUseSingleColor(false);
    node->setAmbient(0.3);
    node->setDiffuse(0.8);
    const ari::SceneNode &spec = L.spec.node;
    if (spec.has_specular) {
      node->setSpecular(spec.specular);
      node->setSpecularPower(spec.specular_power);
    }
    if (spec.has_opacity && spec.opacity_bind.empty())
      node->setOpacity(std::clamp(static_cast<double>(spec.opacity), 0.0, 1.0));
    if (spec.has_casts_shadow)
      node->setCastsShadow(spec.casts_shadow);
    if (r.textured)
      node->setTexture(*mesh.texture, false);
    r.node = node;
  }
}

} // namespace

std::set<std::uint64_t> followed_entity_ids(cvc::app &app, const std::string &prefix) {
  std::set<std::uint64_t> ids;
  const std::string target = ari::read_or<std::string>(
      app, ari::resolve_bind(prefix, "viewers.main.camera.track.target"), "");
  for (const std::string head : {"vehicle_", "entity_"}) {
    if (target.size() > head.size() && target.compare(0, head.size(), head) == 0) {
      char *end = nullptr;
      const unsigned long long v = std::strtoull(target.c_str() + head.size(), &end, 10);
      if (end && *end == '\0')
        ids.insert(v);
    }
  }
  std::stringstream in(
      ari::read_or<std::string>(app, ari::resolve_bind(prefix, "vehicles.watch"), ""));
  for (std::string part; std::getline(in, part, ',');) {
    char *end = nullptr;
    const unsigned long long v = std::strtoull(part.c_str(), &end, 10);
    if (end && end != part.c_str())
      ids.insert(v);
  }
  return ids;
}

struct entity_view::impl {
  std::string implicit_id;
  std::vector<bodies_layer> layers;
  bool declared = false;
  std::unordered_map<std::uint64_t, std::pair<int, int>> where; // entity -> (layer, member)
  std::vector<std::uint64_t> posed;                             // the instanced, pose()'s order
  std::vector<std::pair<int, int>> posed_at;                    // (layer, member) of each
  std::set<std::uint64_t> promoted;
  bool changed = false, members_moved = false;
  // A look's model loading at set_entities (its look, the length it is fitted to).
  std::vector<std::pair<ari::vehicle_look, double>> pending;
  long calls = 0;
  // The frame's camera (promote), for draw's culling and rungs.
  bool frustum = false;
  double fp[6][4] = {};
  bool have_eye = false;
  double eye[3] = {0.0, 0.0, 0.0};
  cvc::lod::view_params vp = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  entity_view_stats stats;

  void camera(vtkRenderer *ren) {
    frustum = false;
    have_eye = false;
    vtkCamera *cam = ren ? ren->GetActiveCamera() : nullptr;
    if (!cam)
      return;
    cam->GetPosition(eye);
    have_eye = true;
    for (int k = 0; k < 3; ++k)
      vp.eye[k] = eye[k];
    vp.tan_half_fov = std::tan(0.5 * cam->GetViewAngle() * kPi / 180.0);
    const int *sz = ren->GetSize();
    if (sz && sz[1] > 0)
      vp.viewport_h_px = sz[1];
    double p24[24];
    cam->GetFrustumPlanes(ren->GetTiledAspectRatio(), p24);
    for (int pl = 0; pl < 6; ++pl) {
      const double *P = p24 + 4 * pl;
      const double len = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
      const double inv = len > 0.0 ? 1.0 / len : 1.0;
      for (int k = 0; k < 4; ++k)
        fp[pl][k] = P[k] * inv;
    }
    frustum = true;
  }

  void draw_arch(bodies_layer &L, arch_draw &a) {
    // Each unpromoted member on screen scored by its size there; the largest take the finer rungs.
    L.score.clear();
    for (const int mi : a.members) {
      const member &m = L.members[static_cast<std::size_t>(mi)];
      if (m.promoted || !m.posed)
        continue;
      const double c[3] = {m.x, m.y, m.z};
      // The four side planes only: the near and far are the renderer's clipping range, which it
      // fits to what it draws, and an unposed rung is collapsed to a point, so a far plane fitted
      // to it would cut away the entities it is about to draw.
      bool in = true;
      if (frustum)
        for (int pl = 0; pl < 4 && in; ++pl)
          in = fp[pl][0] * c[0] + fp[pl][1] * c[1] + fp[pl][2] * c[2] + fp[pl][3] >= -a.radius;
      if (!in)
        continue;
      float px = 1.0e6f;
      if (have_eye)
        px = static_cast<float>(
            cvc::lod::screen_radius_px(a.radius, cvc::lod::bound_distance_m(c, a.radius, vp), vp));
      L.score.push_back({px, mi});
    }
    std::sort(L.score.begin(), L.score.end(),
              [](const auto &x, const auto &y) { return x.first > y.first; });
    std::size_t s = 0;
    for (std::size_t k = 0; k < a.rungs.size(); ++k) {
      rung &r = a.rungs[k];
      r.prev.swap(r.slots);
      r.slots.clear();
      const bool last = k + 1 == a.rungs.size();
      while (s < L.score.size() && r.slots.size() < static_cast<std::size_t>(r.cap) &&
             (last || L.score[s].first >= L.spec.px[std::min<std::size_t>(k, 2)]))
        r.slots.push_back(L.score[s++].second);
      // In member order, so an entity that stays on the rung keeps its slot (and its colours).
      std::sort(r.slots.begin(), r.slots.end());
      stats.instanced += r.slots.size();
      std::shared_ptr<GeometryNode> node = r.node.lock();
      if (!node || (r.slots.empty() && r.shown == 0))
        continue; // an empty rung that was empty: nothing to send
      r.shown = r.slots.size();
      pack(L, r);
      node->updateVertices(r.xyz);
      node->updateNormals(r.nrm);
      if (r.slots != r.prev) {
        const int V = r.verts();
        for (std::size_t j = 0; j < r.slots.size(); ++j) {
          if (j < r.prev.size() && r.prev[j] == r.slots[j])
            continue;
          const member &m = L.members[static_cast<std::size_t>(r.slots[j])];
          for (int q = 0; q < V; ++q) {
            const unsigned char role = r.role[static_cast<std::size_t>(q)];
            const unsigned char *col = role == 0 ? m.c1 : role == 1 ? m.c2 : &r.own[3 * q];
            const std::size_t o = (j * static_cast<std::size_t>(V) + q) * 3;
            r.rgb[o] = col[0];
            r.rgb[o + 1] = col[1];
            r.rgb[o + 2] = col[2];
          }
        }
        node->updateColors(r.rgb);
      }
    }
  }

  // Each slot's copy where its member is; the slots past them collapsed onto the first (zero-area
  // triangles, and a tight bounding box).
  static void pack(const bodies_layer &L, rung &r) {
    const int V = r.verts();
    double px = 0.0, py = 0.0, pz = 0.0;
    for (std::size_t j = 0; j < r.slots.size(); ++j) {
      const double *M = L.members[static_cast<std::size_t>(r.slots[j])].m;
      if (j == 0) {
        px = M[3];
        py = M[7];
        pz = M[11];
      }
      for (int q = 0; q < V; ++q) {
        const double *t = &r.tv[3 * static_cast<std::size_t>(q)];
        const double *n = &r.tn[3 * static_cast<std::size_t>(q)];
        const std::size_t o = (j * static_cast<std::size_t>(V) + q) * 3;
        r.xyz[o] = M[0] * t[0] + M[1] * t[1] + M[2] * t[2] + M[3];
        r.xyz[o + 1] = M[4] * t[0] + M[5] * t[1] + M[6] * t[2] + M[7];
        r.xyz[o + 2] = M[8] * t[0] + M[9] * t[1] + M[10] * t[2] + M[11];
        r.nrm[o] = M[0] * n[0] + M[1] * n[1] + M[2] * n[2];
        r.nrm[o + 1] = M[4] * n[0] + M[5] * n[1] + M[6] * n[2];
        r.nrm[o + 2] = M[8] * n[0] + M[9] * n[1] + M[10] * n[2];
      }
    }
    const std::size_t from = r.slots.size() * static_cast<std::size_t>(V) * 3;
    for (std::size_t o = from; o + 2 < r.xyz.size(); o += 3) {
      r.xyz[o] = px;
      r.xyz[o + 1] = py;
      r.xyz[o + 2] = pz;
    }
  }
};

entity_view::entity_view(std::string implicit_id) : _impl(std::make_unique<impl>()) {
  _impl->implicit_id = std::move(implicit_id);
}
entity_view::~entity_view() = default;

bool entity_view::add_layer(const ari::SceneNode &node, const std::shared_ptr<GraphicsNode> &group,
                            std::vector<std::string> *warnings) {
  const std::string kind = node.props.str("layer", "bodies");
  if (kind != "bodies") {
    warn(warnings, "ari: entity_layer '" + node.id + "': no layer '" + kind +
                       "' yet (bodies; flags, goals, trails, path, belief, dots, targets and "
                       "follow are to come)");
    return false;
  }
  bodies_layer L;
  L.spec = spec_of(node, warnings);
  L.group = group;
  _impl->layers.push_back(std::move(L));
  _impl->declared = true;
  return true;
}

bool entity_view::declared() const { return _impl->declared; }

const std::string &entity_view::implicit_id() const { return _impl->implicit_id; }

void entity_view::set_entities(SceneGraph &sg, vehicle_model_cache &models,
                               const std::vector<entity_draw_info> &entities) {
  impl &d = *_impl;
  ++d.stats.rebuilds;
  d.members_moved = true;
  if (d.layers.empty()) { // no entity_layer: one over all of them
    bodies_layer L;
    L.spec.id = d.implicit_id;
    L.implicit = true;
    d.layers.push_back(std::move(L));
  }
  cvc::app &app = sg.appContext();
  // The old rungs go; who was promoted stays (an incumbent keeps its place across a rebuild).
  for (bodies_layer &L : d.layers) {
    if (std::shared_ptr<GraphicsNode> g = L.group.lock())
      for (const arch_draw &a : L.archs)
        for (std::size_t k = 0; k < a.rungs.size(); ++k)
          if (std::shared_ptr<GraphicsNode> old = g->findChildByName(rung_name(L.spec, a, k)))
            g->removeGraphicsChild(old);
    L.members.clear();
    L.archs.clear();
    L.instancing = false;
  }
  d.where.clear();
  d.posed.clear();
  d.posed_at.clear();
  d.pending.clear();
  // Each archetype's mesh, made from its first entity's look and size.
  struct look {
    vehicle_look_mesh mesh;
    bool ok = false;
  };
  std::unordered_map<std::string, look> looks;
  for (const entity_draw_info &e : entities) {
    int li = -1;
    for (std::size_t k = 0; k < d.layers.size() && li < 0; ++k)
      if (d.layers[k].spec.takes(e))
        li = static_cast<int>(k);
    if (li < 0)
      continue;
    auto it = looks.find(e.archetype);
    if (it == looks.end()) {
      look lk;
      vehicle_shape fit = e.shape; // a model spans its fit length (a boat's overall length)
      if (e.fit_length > 0.0)
        fit.length = e.fit_length;
      const bool fitted = e.look.kind == "data" || (e.look.kind == "file" && e.look.fit);
      vehicle_model_status st = vehicle_look_mesh_of(app, models, e.look, fitted ? fit : e.shape,
                                                     e.base_z, lk.mesh, nullptr, e.boat.get());
      if (st == vehicle_model_status::pending) { // its boxes (a boat's hull) while it loads
        d.pending.push_back({e.look, fit.length});
        cvc::ariadne::vehicle_look box;
        box.kind = "box";
        st = vehicle_look_mesh_of(app, models, box, e.shape, e.base_z, lk.mesh, nullptr,
                                  e.boat.get());
      }
      lk.ok = st == vehicle_model_status::ready;
      it = looks.emplace(e.archetype, std::move(lk)).first;
    }
    if (!it->second.ok)
      continue; // a look no layer draws: its own node
    bodies_layer &L = d.layers[static_cast<std::size_t>(li)];
    const std::string aid = e.archetype.empty() ? std::string("default") : e.archetype;
    int ai = -1;
    for (std::size_t k = 0; k < L.archs.size() && ai < 0; ++k)
      if (L.archs[k].id == aid)
        ai = static_cast<int>(k);
    if (ai < 0) {
      arch_draw a;
      a.id = aid;
      const vehicle_shape &s = e.shape;
      const double l = std::max(s.length, e.fit_length); // a boat's model spans its length overall
      a.radius = 0.5 * std::sqrt(l * l + s.width * s.width + 4.0 * s.height * s.height);
      L.archs.push_back(std::move(a));
      ai = static_cast<int>(L.archs.size()) - 1;
    }
    member m;
    m.id = e.id;
    m.arch = ai;
    for (std::size_t k = 0; k < 3; ++k) {
      m.c1[k] = u8(e.color[k]);
      m.c2[k] = u8(e.color2[k]);
    }
    m.located = e.has_position;
    m.x = e.position[0];
    m.y = e.position[1];
    m.z = e.position[2];
    L.archs[static_cast<std::size_t>(ai)].members.push_back(static_cast<int>(L.members.size()));
    d.where[e.id] = {li, static_cast<int>(L.members.size())};
    L.members.push_back(m);
  }
  for (std::size_t li = 0; li < d.layers.size(); ++li) {
    bodies_layer &L = d.layers[li];
    if (L.members.size() <= static_cast<std::size_t>(L.spec.threshold))
      continue; // each drawn as its own node
    std::shared_ptr<GraphicsNode> group = L.group.lock();
    if (!group && L.implicit) {
      group = sg.getGraphics(L.spec.id);
      if (!group)
        group = sg.addGraphics(L.spec.id);
      L.group = group;
    }
    if (!group)
      continue;
    L.instancing = true;
    for (arch_draw &a : L.archs)
      make_rungs(app, L, a, looks[a.id == "default" ? std::string() : a.id].mesh, *group);
    for (std::size_t mi = 0; mi < L.members.size(); ++mi) {
      d.posed.push_back(L.members[mi].id);
      d.posed_at.push_back({static_cast<int>(li), static_cast<int>(mi)});
    }
  }
}

const std::vector<std::uint64_t> &entity_view::posed_ids() const { return _impl->posed; }

bool entity_view::pending_settled(cvc::app &app, vehicle_model_cache &models) {
  impl &d = *_impl;
  if (d.pending.empty() || ++d.calls % 30 != 0)
    return false;
  for (const auto &[look, length] : d.pending)
    if (vehicle_model_state(app, models, look, length) != vehicle_model_status::pending)
      return true;
  return false;
}

void entity_view::promote(vtkRenderer *renderer, const std::set<std::uint64_t> &followed) {
  impl &d = *_impl;
  d.camera(renderer);
  std::set<std::uint64_t> now;
  std::vector<std::pair<double, int>> near;
  for (bodies_layer &L : d.layers) {
    if (!L.instancing) {
      for (member &m : L.members) {
        m.promoted = true;
        now.insert(m.id);
      }
      continue;
    }
    const std::size_t k = static_cast<std::size_t>(L.spec.promote);
    near.clear();
    std::size_t taken = 0;
    for (std::size_t i = 0; i < L.members.size(); ++i) {
      member &m = L.members[i];
      m.promoted = false;
      if (followed.count(m.id)) {
        m.promoted = true; // whatever promote: says, a followed one has its own node
        ++taken;
        continue;
      }
      if (!d.have_eye || !(m.posed || m.located))
        continue;
      const double dx = m.x - d.eye[0], dy = m.y - d.eye[1], dz = m.z - d.eye[2];
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (d.promoted.count(m.id))
        dist *= kKeep;
      near.push_back({dist, static_cast<int>(i)});
    }
    const std::size_t want = taken < k ? k - taken : 0;
    if (near.size() > want)
      std::nth_element(near.begin(), near.begin() + static_cast<std::ptrdiff_t>(want), near.end());
    for (std::size_t j = 0; j < want && j < near.size(); ++j)
      L.members[static_cast<std::size_t>(near[j].second)].promoted = true;
    for (const member &m : L.members)
      if (m.promoted)
        now.insert(m.id);
  }
  d.changed = d.members_moved || now != d.promoted;
  d.members_moved = false;
  d.promoted = std::move(now);
  d.stats.layers = d.layers.size();
  d.stats.members = d.where.size();
  d.stats.promoted = 0;
  d.stats.meshes = 0;
  for (const bodies_layer &L : d.layers) {
    for (const member &m : L.members)
      d.stats.promoted += m.promoted ? 1 : 0;
    if (L.instancing)
      for (const arch_draw &a : L.archs)
        d.stats.meshes += a.rungs.size();
  }
}

bool entity_view::drawn_as_node(std::uint64_t id) const {
  const auto it = _impl->where.find(id);
  if (it == _impl->where.end())
    return true;
  const auto [l, i] = it->second;
  return _impl->layers[static_cast<std::size_t>(l)].members[static_cast<std::size_t>(i)].promoted;
}

bool entity_view::nodes_changed() const { return _impl->changed; }

void entity_view::pose(std::size_t k, const double m[16]) {
  impl &d = *_impl;
  if (k >= d.posed_at.size())
    return;
  const auto [l, i] = d.posed_at[k];
  member &mb = d.layers[static_cast<std::size_t>(l)].members[static_cast<std::size_t>(i)];
  std::copy(m, m + 16, mb.m);
  mb.x = m[3];
  mb.y = m[7];
  mb.z = m[11];
  mb.posed = true;
  mb.located = true;
}

void entity_view::draw() {
  impl &d = *_impl;
  d.stats.instanced = 0;
  for (bodies_layer &L : d.layers)
    if (L.instancing)
      for (arch_draw &a : L.archs)
        d.draw_arch(L, a);
}

entity_view_stats entity_view::stats() const { return _impl->stats; }

// ── A scene's own world ──────────────────────────────────────────────────────────────────────────

// What a realized scene keeps of its world's entities: the view, and the binding's group that
// poses its rungs.
struct scene_entities {
  std::shared_ptr<entity_view> view = std::make_shared<entity_view>();
  std::uint64_t rev = ~std::uint64_t(0);
  const void *binding = nullptr; // the physics binding the group is on
  physics_scene_binding::group_id group = 0;
  bool grouped = false;
};

namespace {

std::shared_ptr<GraphicsNode> realize_entity_layer(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent, RealizedScene &out,
                                                   std::vector<std::string> *warnings) {
  if (!out.entities)
    out.entities = std::make_shared<scene_entities>();
  std::shared_ptr<GraphicsNode> group = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  if (!group)
    return nullptr;
  if (!out.entities->view->add_layer(n, group, warnings)) {
    if (parent)
      parent->removeGraphicsChild(group);
    else
      sg.removeGraphics(n.id);
    return nullptr;
  }
  return group;
}

// The world's entities as the view draws them: each placed one, its look and size its archetype's.
// A boat (a floating plant's) is drawn about its hull body's origin, as its own node draws it
// (sync_vehicle_markers): its model fitted to its length overall, its keel on its hull's bottom,
// and its box look its hull; each boat archetype's preset made once (the view's template is made
// from an archetype's first entity). A boat in a parted look keeps its own node.
std::vector<entity_draw_info> world_entities(const ari::WorldVehicles &v,
                                             const std::vector<ari::entity_info> &ents) {
  struct boat_draw {
    boat_fit fit;
    std::shared_ptr<const ph::boat_desc> boat;
  };
  std::unordered_map<std::string, boat_draw> boats;
  std::vector<entity_draw_info> out;
  out.reserve(ents.size());
  for (const ari::entity_info &e : ents) {
    if (!e.placed)
      continue;
    const ari::vehicle_archetype *a = nullptr;
    for (const ari::vehicle_archetype &x : v.archetypes())
      if (x.id == e.spec.archetype)
        a = &x;
    if (!a)
      continue;
    const boat_draw *boat = nullptr;
    if (ari::vehicle_plant_floats(a->plant)) {
      auto it = boats.find(a->id);
      if (it == boats.end()) {
        boat_draw b;
        try {
          const ph::dyn_boat_desc desc = ari::boat_desc_of(*a, e.spec.mass, e.spec.max_speed);
          b.fit = boat_fit_of(desc);
          b.boat = std::make_shared<const ph::boat_desc>(desc.boat);
        } catch (const std::exception &) {
        }
        it = boats.emplace(a->id, std::move(b)).first;
      }
      if (!it->second.fit.ok || a->look.parted())
        continue; // one it cannot fit, or a model in parts (a car's rig): its own node
      boat = &it->second;
    }
    entity_draw_info d;
    d.id = e.id;
    d.fleet = e.fleet;
    d.archetype = a->id;
    d.team = e.spec.team;
    for (std::size_t k = 0; k < 3; ++k) {
      d.color[k] = e.spec.color[k];
      d.color2[k] = e.spec.color2[k];
    }
    d.look = a->look;
    if (boat) {
      d.shape = boat->fit.shape; // its hull's box, at its length overall
      d.base_z = boat->fit.keel;
      d.fit_length = boat->fit.shape.length;
      d.boat = boat->boat;
    } else {
      const ph::vehicle_desc desc = ari::vehicle_desc_of(*a, e.spec.mass, e.spec.max_speed);
      d.shape = vehicle_shape_of(desc);
      d.base_z = -ari::stand_height(desc);
    }
    d.position = {e.x, e.y, e.z};
    d.has_position = true;
    out.push_back(std::move(d));
  }
  return out;
}

} // namespace

void register_entity_node_types(cvc::app &app) {
  if (!has_scene_node_type(app, "entity_layer"))
    register_scene_node_type(app, "entity_layer", realize_entity_layer);
}

void update_entity_view(RealizedScene &r, vtkRenderer *renderer) {
  if (!r.world || !r.sg || !r.physics_binding || !r.app)
    return;
  ari::WorldVehicles *v = r.world->vehicles();
  if (!v)
    return;
  if (!r.entities)
    r.entities = std::make_shared<scene_entities>();
  scene_entities &se = *r.entities;
  entity_view &view = *se.view;
  if (!r.vehicle_models)
    r.vehicle_models = make_vehicle_model_cache();
  physics_scene_binding &binding = *r.physics_binding;
  if (v->revision() != se.rev || se.binding != r.physics_binding.get() ||
      view.pending_settled(*r.app, *r.vehicle_models)) {
    const std::vector<ari::entity_info> ents = v->entities();
    view.set_entities(*r.sg, *r.vehicle_models, world_entities(*v, ents));
    if (se.grouped && se.binding == r.physics_binding.get())
      binding.unbind_group(se.group);
    se.grouped = false;
    se.rev = v->revision();
    se.binding = r.physics_binding.get();
    // The binding poses the rungs: its group of their entities' chassis, in posed_ids()' order.
    std::unordered_map<std::uint64_t, ph::body_id> chassis;
    for (const ari::entity_info &e : ents)
      chassis[e.id] = e.chassis;
    std::vector<ph::body_id> ids;
    for (const std::uint64_t id : view.posed_ids())
      ids.push_back(chassis[id]);
    if (!ids.empty()) {
      const std::weak_ptr<entity_view> w = se.view;
      try {
        se.group =
            binding.bind_group(ids, [w, ids](const std::vector<ph::pose_tracker::sample> &samples) {
              const std::shared_ptr<entity_view> ev = w.lock();
              if (!ev)
                return;
              std::size_t c = 0;
              double m[16];
              for (const ph::pose_tracker::sample &s : samples) {
                while (c < ids.size() && ids[c] != s.id)
                  ++c;
                if (c == ids.size())
                  break;
                ph::pose_matrix(s.pose, m);
                ev->pose(c++, m);
              }
              ev->draw();
            });
        se.grouped = true;
      } catch (const std::exception &ex) {
        r.app->log(1, std::string("ari: entity view: ") + ex.what() + "\n");
      }
    }
  }
  view.promote(renderer, followed_entity_ids(*r.app, r.bind_prefix));
}

bool entity_drawn_as_node(const RealizedScene &r, std::uint64_t id) {
  return !r.entities || r.entities->view->drawn_as_node(id);
}

bool entity_nodes_changed(const RealizedScene &r) {
  return r.entities && r.entities->view->nodes_changed();
}

entity_view_stats entity_view_stats_of(const RealizedScene &r) {
  return r.entities ? r.entities->view->stats() : entity_view_stats{};
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

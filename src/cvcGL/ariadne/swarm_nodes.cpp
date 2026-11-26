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

// The nav_swarm and swarm_layer scene node types (swarm_nodes.h). What each layer draws, and how,
// is what examples/nav/nav_city_swarm.cpp and nav_city_drive.cpp drew by hand: one merged mesh a
// layer, every vehicle an instance of its template, streamed with updateVertices each frame.

#include <algorithm>
#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_swarm.h>
#include <cvc/ariadne/world_vehicles.h> // vehicle_look_named, vehicle_desc_of
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/mesh_lod.h> // decimate_mesh_by_role: the model's rungs
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/swarm_nodes.h>
#include <cvc/gl/ariadne/vehicle_view.h> // vehicle_look_mesh_of: the model's template
#include <cvc/image/image.h>
#include <cvc/lod/select.h>
#include <cvc/state/state.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkQuadricDecimation.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

constexpr double kPi = 3.14159265358979323846;

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// ---------------------------------------------------------------------------
// Instanced glyphs: one merged mesh, a template per vehicle, posed by streaming its points.
// ---------------------------------------------------------------------------
class glyphs {
public:
  // `verts` [3V] in the template's frame (forward +x, up +z, resting on z >= 0), `tris` [3T];
  // `color` [3n] (null: teal); `uvs` [2V] or null; `role` [V] and `own` [3V] (vehicle_look_mesh's)
  // or null: a point of role 2 in its own colour (a tyre's dark), the others in its instance's.
  cvc::geometry build(cvc::app &app, int n, const float *color, const std::vector<double> &verts,
                      const std::vector<std::uint32_t> &tris, double z = 0.0,
                      const std::vector<float> *uvs = nullptr,
                      const std::vector<unsigned char> *role = nullptr,
                      const std::vector<unsigned char> *own = nullptr) {
    tmpl_ = verts;
    tris_ = tris;
    z_ = z;
    uvs_.clear();
    if (uvs && uvs->size() * 3 == verts.size() * 2)
      uvs_ = *uvs;
    n_ = n;
    v_ = static_cast<int>(tmpl_.size() / 3);
    cvc::geometry g(app);
    auto &pts = g.points();
    auto &cols = g.colors();
    auto &tr = g.tris();
    const bool uv = uvs_.size() == static_cast<std::size_t>(v_) * 2;
    pts.reserve(static_cast<std::size_t>(n) * v_);
    cols.reserve(static_cast<std::size_t>(n) * v_);
    if (uv)
      g.uvs().reserve(static_cast<std::size_t>(n) * v_);
    for (int i = 0; i < n; ++i) {
      const auto base = static_cast<cvc::geometry::index_t>(i) * v_;
      for (int k = 0; k < v_; ++k)
        pts.push_back({tmpl_[3 * k], tmpl_[3 * k + 1], tmpl_[3 * k + 2] + z_});
      const double r = color ? color[3 * i] : 0.20, gg = color ? color[3 * i + 1] : 0.80,
                   bb = color ? color[3 * i + 2] : 0.75;
      for (int k = 0; k < v_; ++k)
        if (own_at(role, own, k))
          cols.push_back(
              {(*own)[3 * k] / 255.0, (*own)[3 * k + 1] / 255.0, (*own)[3 * k + 2] / 255.0});
        else
          cols.push_back({r, gg, bb});
      if (uv)
        for (int k = 0; k < v_; ++k)
          g.uvs().push_back({uvs_[2 * k], uvs_[2 * k + 1]});
      for (std::size_t t = 0; t + 2 < tris_.size(); t += 3)
        tr.push_back({base + tris_[t], base + tris_[t + 1], base + tris_[t + 2]});
    }
    xyz_.assign(static_cast<std::size_t>(n) * v_ * 3, 0.0);
    return g;
  }

  // Every instance yawed by its heading, at its position, lifted by z_off[i] (null: 0).
  const std::vector<double> &pack(const float *pos, const float *heading, const double *z_off) {
    for (int i = 0; i < n_; ++i) {
      const double ox = pos[2 * i], oy = pos[2 * i + 1], oz = z_off ? z_off[i] : 0.0;
      const double ch = std::cos(heading[i]), sh = std::sin(heading[i]);
      for (int k = 0; k < v_; ++k) {
        const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2];
        const std::size_t o = (static_cast<std::size_t>(i) * v_ + k) * 3;
        xyz_[o] = ox + ch * lx - sh * ly;
        xyz_[o + 1] = oy + sh * lx + ch * ly;
        xyz_[o + 2] = oz + z_ + lz;
      }
    }
    return xyz_;
  }

  // The `count` agents in idx[] into the first slots; the rest collapsed to a point at the first
  // (zero-area triangles, and a tight bounding box).
  const std::vector<double> &pack_some(const float *pos, const float *heading, const double *z_off,
                                       const std::uint32_t *idx, int count) {
    count = std::clamp(count, 0, n_);
    for (int s = 0; s < count; ++s) {
      const int i = static_cast<int>(idx[s]);
      const double ox = pos[2 * i], oy = pos[2 * i + 1], oz = z_off ? z_off[i] : 0.0;
      const double ch = std::cos(heading[i]), sh = std::sin(heading[i]);
      for (int k = 0; k < v_; ++k) {
        const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2];
        const std::size_t o = (static_cast<std::size_t>(s) * v_ + k) * 3;
        xyz_[o] = ox + ch * lx - sh * ly;
        xyz_[o + 1] = oy + sh * lx + ch * ly;
        xyz_[o + 2] = oz + z_ + lz;
      }
    }
    double px = 0.0, py = 0.0, pz = 0.0;
    if (count > 0) {
      const int i0 = static_cast<int>(idx[0]);
      px = pos[2 * i0];
      py = pos[2 * i0 + 1];
      pz = (z_off ? z_off[i0] : 0.0) + z_;
    }
    for (int s = count; s < n_; ++s)
      for (int k = 0; k < v_; ++k) {
        const std::size_t o = (static_cast<std::size_t>(s) * v_ + k) * 3;
        xyz_[o] = px;
        xyz_[o + 1] = py;
        xyz_[o + 2] = pz;
      }
    return xyz_;
  }

  int capacity() const { return n_; }
  int verts_per_instance() const { return v_; }

  // Whether template point k keeps its own colour (role 2, a tyre's dark).
  static bool own_at(const std::vector<unsigned char> *role, const std::vector<unsigned char> *own,
                     int k) {
    const auto i = static_cast<std::size_t>(k);
    return role && own && i < role->size() && 3 * i + 2 < own->size() && (*role)[i] == 2;
  }

private:
  int n_ = 0, v_ = 0;
  double z_ = 0.0;
  std::vector<double> tmpl_;
  std::vector<std::uint32_t> tris_;
  std::vector<float> uvs_;
  std::vector<double> xyz_;
};

// A flat arrow pointing +x, `size` long: the vehicle without a model.
void arrow_template(double size, std::vector<double> &v, std::vector<std::uint32_t> &t) {
  const double tip = size, back = -0.45 * size, half = 0.42 * size;
  v = {tip, 0.0, 0.0, back, half, 0.0, back, -half, 0.0};
  t = {0, 1, 2};
}

// The synthetic city's blocks: each building (4-connected component) one height in
// [1 - vary, 1 + vary] x `height` and a slight tint, only the faces open to the air.
cvc::geometry occupancy_walls(const ari::swarm_city &c, double height, const double rgb0[3],
                              double vary) {
  cvc::geometry g;
  const int rows = c.rows, cols = c.cols;
  if (rows < 2 || cols < 2)
    return g;
  const double dx = c.span() / (cols - 1), dy = (c.max_y - c.min_y) / (rows - 1);
  const auto at = [&](int r, int k) {
    return r >= 0 && k >= 0 && r < rows && k < cols &&
           c.occ[static_cast<std::size_t>(r) * cols + k] != 0;
  };
  std::vector<std::int32_t> comp(static_cast<std::size_t>(rows) * cols, -1);
  std::vector<std::uint32_t> seed;
  std::vector<std::uint32_t> stack;
  for (int r = 0; r < rows; ++r)
    for (int k = 0; k < cols; ++k) {
      const std::size_t i = static_cast<std::size_t>(r) * cols + k;
      if (!c.occ[i] || comp[i] >= 0)
        continue;
      const auto id = static_cast<std::int32_t>(seed.size());
      seed.push_back(static_cast<std::uint32_t>(i));
      comp[i] = id;
      stack.assign(1, static_cast<std::uint32_t>(i));
      while (!stack.empty()) {
        const std::uint32_t j = stack.back();
        stack.pop_back();
        const int jr = static_cast<int>(j) / cols, jc = static_cast<int>(j) % cols;
        const int nb[4][2] = {{jr - 1, jc}, {jr + 1, jc}, {jr, jc - 1}, {jr, jc + 1}};
        for (const auto &b : nb)
          if (at(b[0], b[1])) {
            const std::size_t q = static_cast<std::size_t>(b[0]) * cols + b[1];
            if (comp[q] < 0) {
              comp[q] = id;
              stack.push_back(static_cast<std::uint32_t>(q));
            }
          }
      }
    }
  const auto hash01 = [](std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return (x & 0xffffffU) / static_cast<double>(0x1000000U);
  };
  const auto quad = [&](const double p0[3], const double p1[3], const double p2[3],
                        const double p3[3], const double rgb[3]) {
    const auto base = static_cast<cvc::geometry::index_t>(g.points().size());
    for (const double *p : {p0, p1, p2, p3}) {
      g.points().push_back({p[0], p[1], p[2]});
      g.colors().push_back({rgb[0], rgb[1], rgb[2]});
    }
    g.tris().push_back({base, base + 1, base + 2});
    g.tris().push_back({base, base + 2, base + 3});
  };
  for (int r = 0; r < rows; ++r) {
    const double yc = c.min_y + r * dy;
    for (int k = 0; k < cols; ++k) {
      const std::size_t i = static_cast<std::size_t>(r) * cols + k;
      if (!c.occ[i])
        continue;
      const double xc = c.min_x + k * dx;
      const double x0 = xc - 0.5 * dx, x1 = xc + 0.5 * dx, y0 = yc - 0.5 * dy, y1 = yc + 0.5 * dy;
      double h = height, rgb[3] = {rgb0[0], rgb0[1], rgb0[2]};
      if (vary > 0.0 && comp[i] >= 0) {
        h = height * (1.0 - vary + 2.0 * vary * hash01(seed[comp[i]]));
        const double tint = 0.9 + 0.2 * hash01(seed[comp[i]] * 2654435761U);
        for (double &v : rgb)
          v = std::min(1.0, v * tint);
      }
      const double t0[3] = {x0, y0, h}, t1[3] = {x1, y0, h}, t2[3] = {x1, y1, h},
                   t3[3] = {x0, y1, h};
      quad(t0, t1, t2, t3, rgb);
      if (!at(r, k - 1)) {
        const double a[3] = {x0, y0, 0}, b[3] = {x0, y0, h}, e[3] = {x0, y1, h}, f[3] = {x0, y1, 0};
        quad(a, b, e, f, rgb);
      }
      if (!at(r, k + 1)) {
        const double a[3] = {x1, y0, 0}, b[3] = {x1, y1, 0}, e[3] = {x1, y1, h}, f[3] = {x1, y0, h};
        quad(a, b, e, f, rgb);
      }
      if (!at(r - 1, k)) {
        const double a[3] = {x0, y0, 0}, b[3] = {x1, y0, 0}, e[3] = {x1, y0, h}, f[3] = {x0, y0, h};
        quad(a, b, e, f, rgb);
      }
      if (!at(r + 1, k)) {
        const double a[3] = {x0, y1, 0}, b[3] = {x0, y1, h}, e[3] = {x1, y1, h}, f[3] = {x1, y1, 0};
        quad(a, b, e, f, rgb);
      }
    }
  }
  return g;
}

// A flat quad over the city at z, UVs over it (the fog texture's).
cvc::geometry ground_quad(const ari::swarm_city &c, double z, const double rgb[3]) {
  cvc::geometry g;
  const double corner[4][2] = {
      {c.min_x, c.min_y}, {c.max_x, c.min_y}, {c.max_x, c.max_y}, {c.min_x, c.max_y}};
  const double uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  for (int k = 0; k < 4; ++k) {
    g.points().push_back({corner[k][0], corner[k][1], z});
    g.colors().push_back({rgb[0], rgb[1], rgb[2]});
    g.uvs().push_back({uv[k][0], uv[k][1]});
  }
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  return g;
}

// A number prop, or a state key's number (read each frame, never seeded).
struct live_number {
  double value = 0.0;
  std::string key; // absolute; empty: the constant
  double read(cvc::app &app) const {
    return key.empty() ? value : ari::read_or<double>(app, key, value);
  }
};
live_number live_prop(const ari::Value &props, const std::string &name, double def,
                      const std::string &prefix) {
  live_number n;
  n.value = def;
  const ari::Value *v = props.find(name);
  if (!v)
    return n;
  if (v->is_map()) {
    n.value = v->num("default", def);
    const std::string b = v->str("bind");
    if (!b.empty())
      n.key = ari::resolve_bind(prefix, b);
    return n;
  }
  const double d = v->as_double(std::nan(""));
  if (std::isfinite(d))
    n.value = d;
  else if (!v->as_string().empty())
    n.key = ari::resolve_bind(prefix, v->as_string());
  return n;
}

struct swarm_view;

// One swarm_layer node.
struct layer {
  ari::SceneNode spec;
  std::weak_ptr<GraphicsNode> group;
  virtual ~layer() = default;
  // A new world is in: make the meshes for its agents.
  virtual void build(swarm_view &v) = 0;
  // Each frame, after the swarm's update. `fresh`: a newer navigator frame.
  virtual void tick(swarm_view &v, vtkRenderer *r, bool fresh) = 0;

  // A mesh child of the layer's node, with the defaults given and then the layer's material:.
  std::shared_ptr<GeometryNode> mesh(const std::string &suffix, const cvc::geometry &g,
                                     double ambient, double diffuse, double specular = 0.0,
                                     double power = 1.0) {
    std::shared_ptr<GraphicsNode> grp = group.lock();
    if (!grp)
      return nullptr;
    if (std::shared_ptr<GraphicsNode> old = grp->findChildByName(spec.id + suffix))
      grp->removeGraphicsChild(old);
    auto node = grp->createChild<GeometryNode>(spec.id + suffix, g);
    if (!node)
      return nullptr;
    node->setUseSingleColor(false);
    node->setAmbient(ambient);
    node->setDiffuse(diffuse);
    node->setSpecular(specular);
    node->setSpecularPower(power);
    // A layer's colours are its own (each vehicle's, each plane's): of material: it takes the
    // opacity, and the specular when given.
    if (spec.has_specular) {
      node->setSpecular(spec.specular);
      node->setSpecularPower(spec.specular_power);
    }
    if (spec.has_opacity && spec.opacity_bind.empty())
      node->setOpacity(std::clamp(static_cast<double>(spec.opacity), 0.0, 1.0));
    if (spec.has_casts_shadow)
      node->setCastsShadow(spec.casts_shadow);
    return node;
  }
  void drop(const std::string &suffix) {
    if (std::shared_ptr<GraphicsNode> grp = group.lock())
      if (std::shared_ptr<GraphicsNode> old = grp->findChildByName(spec.id + suffix))
        grp->removeGraphicsChild(old);
  }
};

// What a nav_swarm node keeps: its swarm, its city's source and its layers.
struct swarm_view {
  cvc::app &app;
  SceneGraph &sg;
  std::string id, prefix;
  std::shared_ptr<ari::SceneSwarm> swarm;
  std::vector<std::shared_ptr<layer>> layers;
  std::uint64_t gen = 0;
  long frame = 0;
  double speed0 =
      0.0; // the clock's speed at the first frame: the discovered walls' cycle is 5 s of it

  // The city: a dataset (`city:`), else the synthetic one.
  std::string city;       // the data: name
  std::string city_path;  // <prefix>.data.<city>
  std::string city_seen;  // the dataset's rev drawn from
  bool synthetic = false; // the synthetic city is in
  int grid = 96;
  double top = 0.0; // the city's highest point (z): the map markers fly over it

  std::string restarts_key, restarts_seen;
  std::string lockstep_key, follow_key, fps_key, bench_key;
  // The frame rate (its own: a host need not have an fps readout), smoothed over about a second
  // and written twice a second.
  std::chrono::steady_clock::time_point last_frame{};
  double fps = 0.0, since_fps = 0.0;
  double own_ms = 0.0; // this node's own time over the last 120 frames (the bench line)

  // The trails' rings, kept when a trails or path layer needs them: segments a vehicle, each two
  // points.
  int trail_k = 0;
  int trail_every = 3;
  std::vector<double> trail_xyz;
  std::vector<float> trail_last;
  std::vector<int> trail_wr;
  std::uint64_t trail_rev = 0;

  swarm_view(cvc::app &a, SceneGraph &s) : app(a), sg(s) {}

  int follow() const { return ari::read_or<int>(app, follow_key, -1); }

  // The dataset's world, once it is ready (and again each time it changes); the synthetic city
  // when there is none, or it failed.
  void watch_city() {
    if (city.empty()) {
      if (!synthetic) {
        synthetic = true;
        use(ari::synthetic_swarm_city(grid));
      }
      return;
    }
    const std::string status = ari::read_or<std::string>(app, city_path + ".status", "");
    if (status == "error") {
      if (!synthetic) {
        std::fprintf(stderr,
                     "nav_swarm '%s': the city %s failed to load (%s); the synthetic city\n",
                     id.c_str(), city.c_str(),
                     ari::read_or<std::string>(app, city_path + ".error", "").c_str());
        synthetic = true;
        use(ari::synthetic_swarm_city(grid));
      }
      return;
    }
    if (status != "ready")
      return;
    const std::string rev = ari::read_or<std::string>(app, city_path + ".rev", "");
    if (rev.empty() || rev == city_seen)
      return;
    city_seen = rev;
    cvc::state *w = ari::read_node(app, city_path + ".world");
    std::shared_ptr<const cvc::world::world_geometry> world;
    if (w) {
      try {
        world = boost::any_cast<std::shared_ptr<const cvc::world::world_geometry>>(w->data());
      } catch (const boost::bad_any_cast &) {
      }
    }
    if (!world) {
      std::fprintf(stderr, "nav_swarm '%s': the dataset %s has no world; the synthetic city\n",
                   id.c_str(), city.c_str());
      if (!synthetic) {
        synthetic = true;
        use(ari::synthetic_swarm_city(grid));
      }
      return;
    }
    const std::string name = ari::read_or<std::string>(
        app, city_path + ".canonical", ari::read_or<std::string>(app, city_path + ".uri", city));
    ari::swarm_city c = ari::swarm_city_from_world(*world, grid, name);
    if (c.rows < 2)
      return;
    top = c.max_z;
    synthetic = false;
    swarm->set_city(std::move(c));
  }

  void use(ari::swarm_city c) {
    top = c.max_z;
    swarm->set_city(std::move(c));
  }

  void keep_trails(const ari::SceneSwarm::frame &f, const ari::swarm_city &c) {
    if (trail_k <= 0)
      return;
    const int n = f.n;
    if (static_cast<int>(trail_wr.size()) != n) {
      trail_last.assign(f.pos.begin(), f.pos.end());
      trail_wr.assign(n, 0);
      trail_xyz.assign(static_cast<std::size_t>(3) * 2 * trail_k * n, 0.0);
      for (int i = 0; i < n; ++i) {
        const double z = f.ground[i] + 0.35;
        for (int s = 0; s < 2 * trail_k; ++s) {
          const std::size_t o = (static_cast<std::size_t>(i) * 2 * trail_k + s) * 3;
          trail_xyz[o] = f.pos[2 * i];
          trail_xyz[o + 1] = f.pos[2 * i + 1];
          trail_xyz[o + 2] = z;
        }
      }
      ++trail_rev;
      return;
    }
    if (frame % std::max(1, trail_every))
      return;
    bool any = false;
    for (int i = 0; i < n; ++i) {
      const float dx = f.pos[2 * i] - trail_last[2 * i],
                  dy = f.pos[2 * i + 1] - trail_last[2 * i + 1];
      const float m2 = dx * dx + dy * dy;
      if (m2 < 1.0f)
        continue;
      if (m2 > 9.0f) { // too far since the last sample: a chord would cut a corner through a wall
        trail_last[2 * i] = f.pos[2 * i];
        trail_last[2 * i + 1] = f.pos[2 * i + 1];
        continue;
      }
      const std::size_t v = (static_cast<std::size_t>(i) * trail_k + trail_wr[i]) * 2;
      trail_xyz[3 * v] = trail_last[2 * i];
      trail_xyz[3 * v + 1] = trail_last[2 * i + 1];
      trail_xyz[3 * v + 2] = c.ground(trail_last[2 * i], trail_last[2 * i + 1]) + 0.35;
      trail_xyz[3 * (v + 1)] = f.pos[2 * i];
      trail_xyz[3 * (v + 1) + 1] = f.pos[2 * i + 1];
      trail_xyz[3 * (v + 1) + 2] = f.ground[i] + 0.35;
      trail_last[2 * i] = f.pos[2 * i];
      trail_last[2 * i + 1] = f.pos[2 * i + 1];
      trail_wr[i] = (trail_wr[i] + 1) % trail_k;
      any = true;
    }
    if (any)
      ++trail_rev;
  }

  void measure() {
    const auto t = std::chrono::steady_clock::now();
    if (last_frame.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(t - last_frame).count();
      if (dt > 0.0) {
        const double a = dt / (dt + 1.0);
        fps = fps <= 0.0 ? 1.0 / dt : fps + a * (1.0 / dt - fps);
        since_fps += dt;
        if (since_fps >= 0.5) {
          since_fps = 0.0;
          char b[32];
          std::snprintf(b, sizeof b, "%.1f", fps);
          ari::write(app, fps_key, std::string(b));
        }
      }
    }
    last_frame = t;
    // `<keys>.bench` 1: the rate on stderr every 120 frames, for a benchmark that reads no window.
    if (frame % 120 == 30 && ari::read_bool_or(app, bench_key, 0)) {
      std::fprintf(stderr,
                   "nav_swarm %s: %.1f fps | %d agents | tick %ld | swarm %.2f ms a frame\n",
                   id.c_str(), fps, swarm->size(), swarm->now().tick, own_ms / 120.0);
      own_ms = 0.0;
    }
  }

  void tick(vtkRenderer *r) {
    ++frame;
    measure();
    const auto t0 = std::chrono::steady_clock::now();
    tick_swarm(r);
    own_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  }

  void tick_swarm(vtkRenderer *r) {
    if (speed0 <= 0.0)
      speed0 = std::max(1e-6, app.world_clock().scale());
    watch_city();
    const std::string restarts = ari::read_or<std::string>(app, restarts_key, "");
    if (restarts != restarts_seen) {
      const bool first = restarts_seen.empty() && frame == 1;
      restarts_seen = restarts;
      if (!first && swarm->has_city())
        swarm->restart(); // sim.restart: the swarm is built again, as the world is
    }
    const bool fresh = swarm->update(ari::read_bool_or(app, lockstep_key, 0) != 0);
    if (!swarm->running())
      return;
    if (swarm->generation() != gen) {
      gen = swarm->generation();
      trail_wr.clear();
      for (const auto &l : layers)
        l->build(*this);
    }
    if (fresh)
      keep_trails(swarm->now(), swarm->city());
    for (const auto &l : layers)
      l->tick(*this, r, fresh);
  }
};

// The nav_swarm nodes on an app, by their node, for their layers to find.
struct swarm_registry {
  static constexpr const char *app_slot = "cvc.gl.ariadne.nav_swarm";
  std::mutex m;
  std::map<const GraphicsNode *, std::weak_ptr<swarm_view>> views;
  void add(const GraphicsNode *node, const std::shared_ptr<swarm_view> &v) {
    std::lock_guard<std::mutex> lk(m);
    for (auto it = views.begin(); it != views.end();)
      it = it->second.expired() ? views.erase(it) : std::next(it);
    views[node] = v;
  }
  std::shared_ptr<swarm_view> find(const GraphicsNode *node) {
    std::lock_guard<std::mutex> lk(m);
    const auto it = views.find(node);
    return it == views.end() ? nullptr : it->second.lock();
  }
};

// ---------------------------------------------------------------------------
// The layers
// ---------------------------------------------------------------------------

// The vehicles.
struct agents_layer : layer {
  std::string look = "humvee";
  bool lod = true;
  int near_cap = 10;
  bool model_read = false;
  // The look's mesh, read once at a Humvee's own size (model_length, its chassis box's length):
  // drawn at the swarm's vehicle length, scaled alike on every axis.
  vehicle_look_mesh model;
  bool have_model = false;
  double model_length = 1.0;
  // The finer rungs' size floors: a vehicle takes rung k only while its radius on screen is at
  // least floor_px[k] (and the rung has room), so an overview, where every vehicle is a few pixels,
  // draws them all on the coarsest rung. lod_px: [rung 0, rung 1, rung 2].
  double floor_px[3] = {24.0, 8.0, 3.0};
  struct rung {
    glyphs g;
    std::weak_ptr<GeometryNode> node;
    int cap = 0;
    bool textured = false;
    std::vector<unsigned char> role, own; // the template's points' roles, [V] and [3V]
    std::vector<unsigned char> rgb;
    std::vector<std::uint32_t> idx;
    std::size_t shown = ~std::size_t(0); // vehicles it drew last frame
    // One instance's colours at p ([3V]): `c`, but where a point keeps its own (a tyre's dark).
    void paint(unsigned char *p, const unsigned char c[3]) const {
      const int V = g.verts_per_instance();
      for (int q = 0; q < V; ++q) {
        const unsigned char *from = glyphs::own_at(&role, &own, q) ? &own[3 * q] : c;
        p[3 * q] = from[0];
        p[3 * q + 1] = from[1];
        p[3 * q + 2] = from[2];
      }
    }
  };
  std::vector<rung> rungs; // one without lod: every vehicle, every frame
  bool use_lod = false;
  std::vector<unsigned char> color_u8, restyle;
  std::vector<std::pair<float, int>> score;

  // The look, resolved and read as a world vehicle's (vehicle_look_named; vehicle_look_mesh_of, as
  // an entity layer's template): `humvee` the Humvee in parts where they are installed (its wheels
  // at rest, in a tyre's dark), else the one-piece Humvee.glb; another model by its name or its
  // file. None of them (the box): arrows.
  void load_look(cvc::app &app) {
    model_read = true;
    if (look == "arrow" || look == "none")
      return;
    std::vector<std::string> why;
    const ari::vehicle_look resolved = ari::vehicle_look_named(look, &why);
    if (resolved.kind == "file") {
      vehicle_shape shape;
      try {
        shape = vehicle_shape_of(ari::vehicle_desc_of(ari::vehicle_archetype{})); // a Humvee's
      } catch (const std::exception &) {
      }
      model_length = shape.length;
      const std::shared_ptr<vehicle_model_cache> cache = make_vehicle_model_cache();
      have_model = vehicle_look_mesh_of(app, *cache, resolved, shape, 0.0, model, &why) ==
                       vehicle_model_status::ready &&
                   model.model;
    }
    for (const std::string &w : why)
      std::fprintf(stderr, "nav_swarm: %s\n", w.c_str());
    if (!have_model)
      std::fprintf(stderr, "nav_swarm: no vehicle model '%s'; arrows\n", look.c_str());
  }

  void build(swarm_view &v) override {
    ari::SceneSwarm &s = *v.swarm;
    if (!model_read)
      load_look(v.app);
    for (std::size_t k = 0; k < rungs.size(); ++k)
      drop("_lod" + std::to_string(k));
    drop("_body");
    rungs.clear();
    const int n = s.size();
    const double len = s.vehicle_length();
    std::vector<double> tv;
    std::vector<std::uint32_t> tt;
    std::vector<float> tuv;
    std::vector<unsigned char> trole, town;
    if (have_model) { // at the swarm's length, resting on z 0 (the look's base)
      const double k = model_length > 1e-9 ? len / model_length : 1.0;
      tv = model.verts;
      for (double &x : tv)
        x *= k;
      tt = model.tris;
      tuv = model.uvs;
      trole = model.role;
      town = model.own;
    } else {
      arrow_template(len, tv, tt);
    }
    const bool textured = have_model && !tuv.empty() && model.texture && !model.texture->empty();
    color_u8.resize(s.colors().size());
    for (std::size_t i = 0; i < color_u8.size(); ++i)
      color_u8[i] =
          static_cast<unsigned char>(std::lround(std::clamp(s.colors()[i], 0.0f, 1.0f) * 255.0f));
    use_lod = lod && have_model;
    if (!use_lod) {
      rungs.resize(1);
      rung &r = rungs[0];
      r.cap = n;
      r.textured = textured;
      r.role = trole;
      r.own = town;
      cvc::geometry g = r.g.build(v.app, n, s.colors().data(), tv, tt, have_model ? 0.0 : 0.6,
                                  textured ? &tuv : nullptr, &r.role, &r.own);
      auto node = mesh("_body", g, 0.32, 0.9, 0.2, 18.0);
      if (node && textured)
        node->setTexture(*model.texture, false);
      r.node = node;
      return;
    }
    // Finest first; the last rung takes every vehicle the others do not. A coarser rung's colour
    // roles each decimated on their own (decimate_mesh_by_role), so the tyres keep their dark.
    const struct {
      double reduction;
      int cap;
    } specs[] = {{0.00, std::max(1, std::min(n, near_cap))},
                 {0.62, std::min(n, 28)},
                 {0.88, std::min(n, 72)},
                 {0.985, n}};
    rungs.resize(4);
    for (int k = 0; k < 4; ++k) {
      rung &r = rungs[k];
      std::vector<double> rv = tv;
      std::vector<std::uint32_t> rt = tt;
      r.role = trole;
      r.own = town;
      if (specs[k].reduction > 0.0)
        decimate_mesh_by_role(tv, tt, trole, town, specs[k].reduction, rv, rt, r.role, r.own);
      r.cap = std::max(1, specs[k].cap);
      r.textured = k == 0 && textured;
      cvc::geometry g = r.g.build(v.app, r.cap, s.colors().data(), rv, rt, 0.0,
                                  r.textured ? &tuv : nullptr, &r.role, &r.own);
      auto node = mesh("_lod" + std::to_string(k), g, 0.34, 0.9, 0.18, 18.0);
      if (node && r.textured)
        node->setTexture(*model.texture, false);
      else
        r.rgb.assign(static_cast<std::size_t>(r.cap) * r.g.verts_per_instance() * 3, 180);
      r.node = node;
      r.idx.reserve(r.cap);
    }
  }

  void tick(swarm_view &v, vtkRenderer *ren, bool fresh) override {
    const ari::SceneSwarm::frame &f = v.swarm->now();
    if (f.n <= 0 || rungs.empty())
      return;
    if (!use_lod) {
      std::shared_ptr<GeometryNode> node = rungs[0].node.lock();
      if (!node || f.n != rungs[0].g.capacity())
        return;
      if (fresh)
        node->updateVertices(rungs[0].g.pack(f.pos.data(), f.heading.data(), f.ground.data()));
      // White while wall-following, green once there, else its colour: every second frame.
      if (fresh && !rungs[0].textured && v.frame % 2 == 0) {
        const int V = rungs[0].g.verts_per_instance();
        restyle.resize(static_cast<std::size_t>(3) * f.n * V);
        for (int i = 0; i < f.n; ++i) {
          unsigned char c[3];
          if (f.reached[i]) {
            c[0] = 110;
            c[1] = 190;
            c[2] = 130;
          } else if (f.mode[i] == 1) {
            c[0] = c[1] = c[2] = 255;
          } else {
            c[0] = color_u8[3 * i];
            c[1] = color_u8[3 * i + 1];
            c[2] = color_u8[3 * i + 2];
          }
          rungs[0].paint(restyle.data() + static_cast<std::size_t>(3) * V * i, c);
        }
        node->updateColors(restyle);
      }
      return;
    }
    // Each on-screen vehicle scored by its size on screen; the largest take the finest rungs.
    const double agent_r = 0.5 * v.swarm->vehicle_length();
    cvc::lod::view_params vp = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
    double fp[6][4];
    bool frustum = false;
    if (ren) {
      if (vtkCamera *cam = ren->GetActiveCamera()) {
        cam->GetPosition(vp.eye);
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
    }
    score.clear();
    for (int i = 0; i < f.n; ++i) {
      const double c[3] = {f.pos[2 * i], f.pos[2 * i + 1], f.ground[i]};
      bool in = true;
      if (frustum)
        for (int pl = 0; pl < 6 && in; ++pl)
          in = fp[pl][0] * c[0] + fp[pl][1] * c[1] + fp[pl][2] * c[2] + fp[pl][3] >= -agent_r;
      if (!in)
        continue; // off-screen: drawn by no rung
      const double d = cvc::lod::bound_distance_m(c, agent_r, vp);
      score.push_back({static_cast<float>(cvc::lod::screen_radius_px(agent_r, d, vp)), i});
    }
    std::sort(score.begin(), score.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    // Largest first: each finer rung takes the vehicles that are big enough for it, up to its cap,
    // and the coarsest the rest.
    std::size_t s = 0;
    for (std::size_t k = 0; k < rungs.size(); ++k) {
      rung &r = rungs[k];
      prev.swap(r.idx);
      r.idx.clear();
      const bool last = k + 1 == rungs.size();
      while (s < score.size() && (last || (r.idx.size() < static_cast<std::size_t>(r.cap) &&
                                           (k >= 3 || score[s].first >= floor_px[k]))))
        r.idx.push_back(static_cast<std::uint32_t>(score[s++].second));
      // In vehicle order, so a vehicle that stays on the rung stays in its slot (and its colour).
      std::sort(r.idx.begin(), r.idx.end());
      std::shared_ptr<GeometryNode> node = r.node.lock();
      if (!node)
        continue;
      // An empty rung that was empty: its slots are already collapsed, so nothing to send.
      if (r.idx.empty() && r.shown == 0)
        continue;
      r.shown = r.idx.size();
      node->updateVertices(r.g.pack_some(f.pos.data(), f.heading.data(), f.ground.data(),
                                         r.idx.data(), static_cast<int>(r.idx.size())));
      // The colours follow the vehicles in the slots: sent only when they changed hands.
      if (!r.textured && r.idx != prev) {
        const int V = r.g.verts_per_instance();
        for (std::size_t j = 0; j < r.idx.size(); ++j) {
          if (j < prev.size() && prev[j] == r.idx[j])
            continue;
          const int a = static_cast<int>(r.idx[j]);
          r.paint(r.rgb.data() + j * V * 3, &color_u8[3 * a]);
        }
        node->updateColors(r.rgb);
      }
    }
  }
  std::vector<std::uint32_t> prev; // a rung's last vehicles, while it takes its new ones
};

// A flag on a pole over each vehicle, flying to its left.
struct flags_layer : layer {
  glyphs g;
  std::weak_ptr<GeometryNode> node;
  void build(swarm_view &v) override {
    ari::SceneSwarm &s = *v.swarm;
    const double gsz = s.vehicle_length();
    const double ph = gsz * 1.5, pr = gsz * 0.03, fw = gsz * 0.7, fh = gsz * 0.4;
    const std::vector<double> fv = {-pr, -pr, 0.0,     pr,  -pr, 0.0, pr,  pr,  0.0,
                                    -pr, pr,  0.0,     -pr, -pr, ph,  pr,  -pr, ph,
                                    pr,  pr,  ph,      -pr, pr,  ph,  0.0, 0.0, ph - fh,
                                    0.0, -fw, ph - fh, 0.0, -fw, ph,  0.0, 0.0, ph};
    const std::vector<std::uint32_t> ft = {0, 2, 1, 0, 3, 2, 4,  5, 6,  4,  6, 7,  0, 1, 5,  0,
                                           5, 4, 1, 2, 6, 1, 6,  5, 2,  3,  7, 2,  7, 6, 3,  0,
                                           4, 3, 4, 7, 8, 9, 10, 8, 10, 11, 8, 10, 9, 8, 11, 10};
    node = mesh("_mesh", g.build(v.app, s.size(), s.colors().data(), fv, ft), 0.6, 0.7);
  }
  void tick(swarm_view &v, vtkRenderer *, bool fresh) override {
    const ari::SceneSwarm::frame &f = v.swarm->now();
    std::shared_ptr<GeometryNode> n = node.lock();
    if (fresh && n && f.n == g.capacity())
      n->updateVertices(g.pack(f.pos.data(), f.heading.data(), f.ground.data()));
  }
};

// A tall beacon over each goal, its apex on the spot; sunk once its vehicle is there.
struct goals_layer : layer {
  glyphs g;
  std::weak_ptr<GeometryNode> node;
  std::vector<double> xyz;
  std::uint64_t targets_seen = 0;
  std::vector<std::uint8_t> sunk;
  void build(swarm_view &v) override {
    ari::SceneSwarm &s = *v.swarm;
    const double span = s.city().span(), wall_h = s.wall_height();
    const double phys = s.vehicle_radius() / s.city().scale; // the vehicle's radius, world units
    const double gh = std::min(0.006 * span, 2.5 * phys);
    const double ght = std::min(1.2 * wall_h, 22.0 * phys);
    const std::vector<double> pv = {0,   0,  0,  -gh, -gh, ght, gh, -gh,
                                    ght, gh, gh, ght, -gh, gh,  ght};
    const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 1, 3, 2, 1, 4, 3};
    node = mesh("_mesh", g.build(v.app, s.size(), s.colors().data(), pv, pt), 0.5, 0.75, 0.15);
    targets_seen = ~std::uint64_t(0);
    sunk.assign(s.size(), 0);
  }
  void place(swarm_view &v) {
    ari::SceneSwarm &s = *v.swarm;
    const std::vector<float> &gw = s.goals();
    const int n = s.size();
    std::vector<float> zero(n, 0.0f);
    std::vector<double> z(n);
    for (int i = 0; i < n; ++i)
      z[i] = s.city().ground(gw[2 * i], gw[2 * i + 1]);
    xyz = g.pack(gw.data(), zero.data(), z.data());
    sunk.assign(n, 0);
  }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    std::shared_ptr<GeometryNode> nd = node.lock();
    ari::SceneSwarm &s = *v.swarm;
    if (!nd || s.size() != g.capacity())
      return;
    bool changed = false;
    if (s.targets_revision() != targets_seen) { // a target moved: its beacons go with it
      targets_seen = s.targets_revision();
      place(v);
      changed = true;
    }
    const ari::SceneSwarm::frame &f = s.now();
    if (v.frame % 5 == 0 && f.n == s.size()) {
      const int GV = g.verts_per_instance();
      for (int i = 0; i < f.n; ++i)
        if (f.reached[i] && !sunk[i]) {
          sunk[i] = 1;
          double *p = xyz.data() + static_cast<std::size_t>(3) * GV * i;
          for (int k = 0; k < GV; ++k)
            p[3 * k + 2] = -5.0;
          changed = true;
        }
    }
    if (changed)
      nd->updateVertices(xyz);
  }
};

// Line meshes over the trails' rings: every vehicle's (trails), or the followed one's (path).
cvc::geometry ring_lines(int n, int k, const float *color, double dim, const float *fixed_rgb) {
  cvc::geometry g;
  for (int i = 0; i < n; ++i)
    for (int s = 0; s < k; ++s)
      for (int e = 0; e < 2; ++e) {
        const std::size_t v = (static_cast<std::size_t>(i) * k + s) * 2 + e;
        g.points().push_back({0.0, 0.0, 0.0});
        if (fixed_rgb)
          g.colors().push_back({fixed_rgb[0], fixed_rgb[1], fixed_rgb[2]});
        else
          g.colors().push_back(
              {dim * color[3 * i], dim * color[3 * i + 1], dim * color[3 * i + 2]});
        if (e == 1)
          g.lines().push_back(
              {static_cast<cvc::geometry::index_t>(v - 1), static_cast<cvc::geometry::index_t>(v)});
      }
  return g;
}

struct trails_layer : layer {
  std::weak_ptr<GeometryNode> node;
  live_number width;
  std::uint64_t seen = 0;
  int n = 0;
  void build(swarm_view &v) override {
    n = v.swarm->size();
    auto nd =
        mesh("_lines", ring_lines(n, v.trail_k, v.swarm->colors().data(), 0.45, nullptr), 1.0, 0.0);
    if (nd) {
      nd->setRenderMode(GeometryRenderMode::LINES);
      nd->setLineWidth(width.read(v.app));
      nd->setDepthOffset(1.0);
      if (!spec.has_casts_shadow)
        nd->setCastsShadow(false);
    }
    node = nd;
    seen = 0;
  }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    std::shared_ptr<GeometryNode> nd = node.lock();
    if (!nd)
      return;
    nd->setLineWidth(width.read(v.app));
    if (v.trail_rev != seen && v.trail_xyz.size() == static_cast<std::size_t>(6) * v.trail_k * n) {
      seen = v.trail_rev;
      nd->updateVertices(v.trail_xyz);
    }
  }
};

struct path_layer : layer {
  std::weak_ptr<GeometryNode> node;
  live_number width;
  std::uint64_t seen = 0;
  int shown = -2;
  std::vector<double> xyz;
  void build(swarm_view &v) override {
    static const float yellow[3] = {1.0f, 0.85f, 0.10f};
    auto nd = mesh("_lines", ring_lines(1, v.trail_k, nullptr, 1.0, yellow), 1.0, 0.0);
    if (nd) {
      nd->setRenderMode(GeometryRenderMode::LINES);
      nd->setLineWidth(width.read(v.app));
      nd->setDepthOffset(2.0); // over the fleet's trails
      nd->setCastsShadow(false);
      nd->setVisible(false);
    }
    node = nd;
    seen = 0;
    shown = -2;
  }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    std::shared_ptr<GeometryNode> nd = node.lock();
    if (!nd)
      return;
    const int fol = v.follow();
    const int n = v.swarm->size();
    const bool on =
        fol >= 0 && fol < n && v.trail_xyz.size() == static_cast<std::size_t>(6) * v.trail_k * n;
    if ((on ? fol : -1) != shown) {
      shown = on ? fol : -1;
      nd->setVisible(on);
      seen = 0;
    }
    if (!on)
      return;
    nd->setLineWidth(width.read(v.app));
    if (v.trail_rev == seen)
      return;
    seen = v.trail_rev;
    const std::size_t m = static_cast<std::size_t>(6) * v.trail_k;
    xyz.assign(v.trail_xyz.begin() + static_cast<std::ptrdiff_t>(fol * m),
               v.trail_xyz.begin() + static_cast<std::ptrdiff_t>((fol + 1) * m));
    nd->updateVertices(xyz);
  }
};

// The walls the fleet has sensed: a box per discovered wall cell, see-through, in the colours of
// the planes that saw it, cycling where several did, with a gentle pulse.
struct belief_layer : layer {
  std::weak_ptr<GeometryNode> node;
  double height = 0.0;
  std::uint64_t seen = ~std::uint64_t(0);
  std::size_t cells = ~std::size_t(0);
  std::vector<int> vert_cell; // each vertex's index into the walls list
  std::vector<unsigned char> cell_rgb, rgb;
  void build(swarm_view &) override {
    drop("_mesh");
    node.reset();
    seen = ~std::uint64_t(0);
    cells = ~std::size_t(0);
    vert_cell.clear();
  }
  void paint(swarm_view &v, double phase) {
    const auto &walls = v.swarm->sensed_walls();
    const auto &pc = v.swarm->plane_colors();
    const int M = static_cast<int>(pc.size() / 3);
    const double pulse = 0.6 + 0.4 * (0.5 + 0.5 * std::sin(phase * 2.0 * kPi));
    cell_rgb.assign(walls.size() * 3, 0);
    for (std::size_t w = 0; w < walls.size(); ++w) {
      const auto &pl = walls[w].planes;
      const int ns = static_cast<int>(pl.size());
      if (ns == 0)
        continue;
      const double pos = phase * ns;
      const int a = static_cast<int>(pos) % ns, b = (a + 1) % ns;
      const double fr = pos - std::floor(pos);
      const int pa = std::clamp(pl[a], 0, std::max(0, M - 1)),
                pb = std::clamp(pl[b], 0, std::max(0, M - 1));
      for (int k = 0; k < 3; ++k) {
        const double c = M > 0 ? pc[3 * pa + k] * (1 - fr) + pc[3 * pb + k] * fr : 0.8;
        cell_rgb[3 * w + k] = static_cast<unsigned char>(std::clamp(c * 255.0 * pulse, 0.0, 255.0));
      }
    }
  }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    ari::SceneSwarm &s = *v.swarm;
    const double phase = std::fmod(v.app.world_clock().t() / (5.0 * v.speed0), 1.0);
    if (s.coverage_revision() != seen) {
      seen = s.coverage_revision();
      const auto &walls = s.sensed_walls();
      if (walls.size() != cells) { // more of the city sensed: mesh it again
        cells = walls.size();
        remesh(v, phase);
        return;
      }
    }
    std::shared_ptr<GeometryNode> nd = node.lock();
    if (nd && v.frame % 3 == 0 && !vert_cell.empty()) {
      paint(v, phase);
      rgb.resize(3 * vert_cell.size());
      for (std::size_t q = 0; q < vert_cell.size(); ++q)
        for (int k = 0; k < 3; ++k)
          rgb[3 * q + k] = cell_rgb[3 * vert_cell[q] + k];
      nd->updateColors(rgb);
    }
  }
  void remesh(swarm_view &v, double phase) {
    ari::SceneSwarm &s = *v.swarm;
    const ari::swarm_city &c = s.city();
    const auto &walls = s.sensed_walls();
    if (walls.empty()) {
      drop("_mesh");
      node.reset();
      vert_cell.clear();
      return;
    }
    const double h = height > 0.0 ? height : std::min(40.0, 0.013 * c.span());
    const double dx = c.span() / std::max(1, c.cols - 1),
                 dy = (c.max_y - c.min_y) / std::max(1, c.rows - 1);
    std::vector<std::uint8_t> sensed(static_cast<std::size_t>(c.rows) * c.cols, 0);
    for (const auto &w : walls)
      sensed[w.cell] = 1;
    const auto at = [&](int r, int k) {
      return r >= 0 && k >= 0 && r < c.rows && k < c.cols &&
             sensed[static_cast<std::size_t>(r) * c.cols + k];
    };
    paint(v, phase);
    cvc::geometry g;
    vert_cell.clear();
    const auto quad = [&](const double a[3], const double b[3], const double e[3],
                          const double f[3], int w) {
      const auto base = static_cast<cvc::geometry::index_t>(g.points().size());
      const double cr = cell_rgb[3 * w] / 255.0, cg = cell_rgb[3 * w + 1] / 255.0,
                   cb = cell_rgb[3 * w + 2] / 255.0;
      for (const double *q : {a, b, e, f}) {
        g.points().push_back({q[0], q[1], q[2]});
        g.colors().push_back({cr, cg, cb});
        vert_cell.push_back(w);
      }
      g.tris().push_back({base, base + 1, base + 2});
      g.tris().push_back({base, base + 2, base + 3});
    };
    for (std::size_t w = 0; w < walls.size(); ++w) {
      const int i = walls[w].cell, r = i / c.cols, k = i % c.cols;
      const double xc = c.min_x + k * dx, yc = c.min_y + r * dy;
      const double x0 = xc - 0.5 * dx, x1 = xc + 0.5 * dx, y0 = yc - 0.5 * dy, y1 = yc + 0.5 * dy;
      const double z0 = c.ground(xc, yc), z1 = z0 + h;
      const int wi = static_cast<int>(w);
      const double t0[3] = {x0, y0, z1}, t1[3] = {x1, y0, z1}, t2[3] = {x1, y1, z1},
                   t3[3] = {x0, y1, z1};
      quad(t0, t1, t2, t3, wi);
      if (!at(r, k - 1)) {
        const double a[3] = {x0, y0, z0}, b[3] = {x0, y1, z0}, e[3] = {x0, y1, z1},
                     f[3] = {x0, y0, z1};
        quad(a, b, e, f, wi);
      }
      if (!at(r, k + 1)) {
        const double a[3] = {x1, y1, z0}, b[3] = {x1, y0, z0}, e[3] = {x1, y0, z1},
                     f[3] = {x1, y1, z1};
        quad(a, b, e, f, wi);
      }
      if (!at(r - 1, k)) {
        const double a[3] = {x1, y0, z0}, b[3] = {x0, y0, z0}, e[3] = {x0, y0, z1},
                     f[3] = {x1, y0, z1};
        quad(a, b, e, f, wi);
      }
      if (!at(r + 1, k)) {
        const double a[3] = {x0, y1, z0}, b[3] = {x1, y1, z0}, e[3] = {x1, y1, z1},
                     f[3] = {x0, y1, z1};
        quad(a, b, e, f, wi);
      }
    }
    std::shared_ptr<GeometryNode> nd = node.lock();
    if (!nd) {
      nd = mesh("_mesh", g, 0.9, 0.2);
      if (nd) {
        if (!spec.has_opacity)
          nd->setOpacity(0.55); // see-through: the city shows through
        nd->setDepthOffset(1.0);
      }
      node = nd;
    } else {
      nd->setGeometry(g);
    }
  }
};

// Markers for a map: a pentagon per vehicle, or a disc in a dark ring per rally point, over the
// city's highest point.
struct dots_layer : layer {
  glyphs g;
  std::weak_ptr<GeometryNode> node;
  std::vector<double> z;
  void build(swarm_view &v) override {
    ari::SceneSwarm &s = *v.swarm;
    const double rr = std::max(s.vehicle_length() * 20.0, s.city().span() * 0.008);
    std::vector<double> pv;
    for (int k = 0; k < 5; ++k) {
      const double a = k * (2.0 * kPi / 5.0) - kPi / 2.0;
      pv.insert(pv.end(), {rr * std::cos(a), rr * std::sin(a), 0.0});
    }
    const std::vector<std::uint32_t> pt = {0, 1, 2, 0, 2, 3, 0, 3, 4};
    auto nd = mesh("_mesh", g.build(v.app, s.size(), s.colors().data(), pv, pt), 1.0, 0.0);
    if (nd && !spec.has_casts_shadow)
      nd->setCastsShadow(false);
    node = nd;
    z.assign(s.size(), v.top + 2.0);
  }
  void tick(swarm_view &v, vtkRenderer *, bool fresh) override {
    const ari::SceneSwarm::frame &f = v.swarm->now();
    std::shared_ptr<GeometryNode> nd = node.lock();
    if (fresh && nd && f.n == g.capacity() && static_cast<int>(z.size()) == f.n)
      nd->updateVertices(g.pack(f.pos.data(), f.heading.data(), z.data()));
  }
};

struct targets_layer : layer {
  std::weak_ptr<GeometryNode> node;
  std::uint64_t seen = ~std::uint64_t(0);
  void build(swarm_view &) override { seen = ~std::uint64_t(0); }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    ari::SceneSwarm &s = *v.swarm;
    if (s.targets_revision() == seen)
      return;
    seen = s.targets_revision();
    const auto &tp = s.targets();
    const auto &tc = s.target_colors();
    const double r = s.city().span() * 0.012, z = v.top + 4.0;
    cvc::geometry g;
    const auto disc = [&](double cx, double cy, double rad, double zz, const double rgb[3]) {
      const auto base = static_cast<cvc::geometry::index_t>(g.points().size());
      g.points().push_back({cx, cy, zz});
      g.colors().push_back({rgb[0], rgb[1], rgb[2]});
      const int seg = 24;
      for (int k = 0; k < seg; ++k) {
        const double a = 2.0 * kPi * k / seg;
        g.points().push_back({cx + rad * std::cos(a), cy + rad * std::sin(a), zz});
        g.colors().push_back({rgb[0], rgb[1], rgb[2]});
      }
      for (int k = 0; k < seg; ++k)
        g.tris().push_back({base, static_cast<cvc::geometry::index_t>(base + 1 + k),
                            static_cast<cvc::geometry::index_t>(base + 1 + (k + 1) % seg)});
    };
    static const double ring[3] = {0.03, 0.04, 0.055};
    for (std::size_t t = 0; t < tp.size() / 2; ++t) {
      const double rgb[3] = {tc[3 * t], tc[3 * t + 1], tc[3 * t + 2]};
      disc(tp[2 * t], tp[2 * t + 1], r * 1.3, z, ring); // a dark ring for contrast
      disc(tp[2 * t], tp[2 * t + 1], r, z + 0.5, rgb);
    }
    auto nd = mesh("_mesh", g, 1.0, 0.0);
    if (nd && !spec.has_casts_shadow)
      nd->setCastsShadow(false);
    node = nd;
  }
};

// The synthetic city: its blocks and its ground, which shows the fleet's fog under fog.
struct city_layer : layer {
  std::weak_ptr<GeometryNode> ground;
  cvc::image fog; // borrowed by the ground's texture (zero-copy) for the node's life
  std::uint64_t seen = ~std::uint64_t(0);
  bool fog_on = false;
  void build(swarm_view &v) override {
    ari::SceneSwarm &s = *v.swarm;
    drop("_walls");
    drop("_ground");
    ground.reset();
    const ari::swarm_city &c = s.city();
    if (c.real)
      return; // a real city: the data node draws it
    static const double wall_rgb[3] = {0.58, 0.58, 0.64}, ground_rgb[3] = {0.20, 0.23, 0.27};
    mesh("_walls", occupancy_walls(c, s.wall_height(), wall_rgb, 0.45), 0.35, 0.8, 0.1);
    auto gn = mesh("_ground", ground_quad(c, 0.0, ground_rgb), 0.65, 0.5);
    fog_on = s.fog();
    if (gn) {
      gn->setUseSingleColor(true);
      if (fog_on) {
        fog = cvc::image(c.cols, c.rows, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
        paint(s);
        gn->setColor(1, 1, 1);
        gn->setTexture(fog, true);
      } else {
        gn->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
      }
    }
    ground = gn;
    seen = s.coverage_revision();
  }
  // Never seen near black, seen dim, in view lit; image row 0 is the grid's top (max y) row.
  void paint(ari::SceneSwarm &s) {
    const ari::swarm_city &c = s.city();
    const auto &cov = s.coverage();
    unsigned char *px = fog.data();
    if (!px)
      return;
    for (int r = 0; r < c.rows; ++r)
      for (int k = 0; k < c.cols; ++k) {
        const std::size_t i = static_cast<std::size_t>(r) * c.cols + k;
        const int tier = i < cov.size() ? cov[i] : 0;
        unsigned char *p = px + 4 * (static_cast<std::size_t>(c.rows - 1 - r) * c.cols + k);
        const unsigned char t[3][3] = {{20, 22, 26}, {40, 44, 50}, {62, 68, 78}};
        p[0] = t[tier][0];
        p[1] = t[tier][1];
        p[2] = t[tier][2];
        p[3] = 255;
      }
  }
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    std::shared_ptr<GeometryNode> gn = ground.lock();
    if (!gn || !fog_on || v.swarm->coverage_revision() == seen)
      return;
    seen = v.swarm->coverage_revision();
    paint(*v.swarm);
    gn->texture_modified();
  }
};

// An empty node at the followed vehicle, heading its way (a track camera reads its -x as behind).
struct follow_layer : layer {
  void build(swarm_view &) override {}
  void tick(swarm_view &v, vtkRenderer *, bool) override {
    std::shared_ptr<GraphicsNode> grp = group.lock();
    const int i = v.follow();
    const ari::SceneSwarm::frame &f = v.swarm->now();
    if (!grp || i < 0 || i >= f.n)
      return;
    const double h = f.heading[i], c = std::cos(h), s = std::sin(h);
    const double m[16] = {c,   -s,  0.0, f.pos[2 * i], s,   c,   0.0, f.pos[2 * i + 1],
                          0.0, 0.0, 1.0, f.ground[i],  0.0, 0.0, 0.0, 1.0};
    grp->setPoseMatrix(m);
  }
};

std::shared_ptr<GraphicsNode> realize_nav_swarm(SceneGraph &sg, const ari::SceneNode &n,
                                                GraphicsNode *parent, RealizedScene &out,
                                                std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  auto view = std::make_shared<swarm_view>(app, sg);
  view->id = n.id;
  view->prefix = out.bind_prefix;
  const std::string keys = ari::resolve_bind(out.bind_prefix, n.props.str("keys", "swarm"));
  const ari::swarm_spec spec = ari::swarm_spec::from_props(n.props);
  view->grid = spec.grid;
  try {
    view->swarm = std::make_shared<ari::SceneSwarm>(app, keys, spec);
  } catch (const std::exception &e) {
    warn(warnings, "ari: nav_swarm '" + n.id + "': " + e.what());
    return nullptr;
  }
  view->city = n.props.str("city");
  if (!view->city.empty())
    view->city_path = ari::resolve_bind(out.bind_prefix, "data." + view->city);
  view->restarts_key = ari::resolve_bind(out.bind_prefix, "sim.restarts");
  view->restarts_seen = ari::read_or<std::string>(app, view->restarts_key, "");
  view->lockstep_key = keys + ".lockstep";
  view->follow_key = keys + ".follow";
  view->fps_key = keys + ".stats.fps";
  view->bench_key = keys + ".bench";
  std::shared_ptr<GraphicsNode> group = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  if (!group)
    return nullptr;
  app.slot<swarm_registry>().add(group.get(), view);
  out.custom_ticks.push_back([view](vtkRenderer *r) { view->tick(r); });
  return group;
}

std::shared_ptr<GraphicsNode> realize_swarm_layer(SceneGraph &sg, const ari::SceneNode &n,
                                                  GraphicsNode *parent, RealizedScene &out,
                                                  std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  std::shared_ptr<swarm_view> view = parent ? app.slot<swarm_registry>().find(parent) : nullptr;
  if (!view) {
    warn(warnings,
         "ari: swarm_layer '" + n.id + "': it draws a swarm, so it goes under a nav_swarm node");
    return nullptr;
  }
  const std::string kind = n.props.str("layer");
  std::shared_ptr<layer> l;
  if (kind == "agents") {
    auto a = std::make_shared<agents_layer>();
    a->look = n.props.str("look", "humvee");
    a->lod = n.props.flag("lod", true);
    a->near_cap = std::max(1, static_cast<int>(n.props.num("near_cap", 10)));
    if (const ari::Value *px = n.props.find("lod_px"); px && px->is_seq())
      for (std::size_t k = 0; k < 3 && k < px->items.size(); ++k)
        a->floor_px[k] = std::max(0.0, px->items[k].as_double(a->floor_px[k]));
    l = a;
  } else if (kind == "flags") {
    l = std::make_shared<flags_layer>();
  } else if (kind == "goals") {
    l = std::make_shared<goals_layer>();
  } else if (kind == "trails" || kind == "path") {
    view->trail_k =
        std::max(view->trail_k, std::clamp(static_cast<int>(n.props.num("segments", 12)), 1, 256));
    if (kind == "trails") {
      view->trail_every = std::max(1, static_cast<int>(n.props.num("every", 3)));
      auto t = std::make_shared<trails_layer>();
      t->width = live_prop(n.props, "width", 1.5, out.bind_prefix);
      l = t;
    } else {
      auto p = std::make_shared<path_layer>();
      p->width = live_prop(n.props, "width", 4.0, out.bind_prefix);
      l = p;
    }
  } else if (kind == "belief") {
    auto b = std::make_shared<belief_layer>();
    b->height = n.props.num("height", 0.0);
    l = b;
  } else if (kind == "dots") {
    l = std::make_shared<dots_layer>();
  } else if (kind == "targets") {
    l = std::make_shared<targets_layer>();
  } else if (kind == "city") {
    l = std::make_shared<city_layer>();
  } else if (kind == "follow") {
    l = std::make_shared<follow_layer>();
  } else {
    warn(warnings,
         "ari: swarm_layer '" + n.id + "': no layer '" + kind +
             "' (agents, flags, goals, trails, path, belief, dots, targets, city, follow)");
    return nullptr;
  }
  std::shared_ptr<GraphicsNode> group = parent->createChild(n.id);
  if (!group)
    return nullptr;
  l->spec = n;
  l->spec.children.clear();
  l->group = group;
  if (kind == "follow")
    sg.registerGraphics(n.id, group); // a track camera finds its target by id
  view->layers.push_back(l);
  return group;
}

} // namespace

void register_swarm_node_types(cvc::app &app) {
  if (!has_scene_node_type(app, "nav_swarm"))
    register_scene_node_type(app, "nav_swarm", realize_nav_swarm);
  if (!has_scene_node_type(app, "swarm_layer"))
    register_scene_node_type(app, "swarm_layer", realize_swarm_layer);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

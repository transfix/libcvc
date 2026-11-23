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
#include <array>
#include <cctype>
#include <cmath>
#include <cvc/ariadne/bind.h> // resolve_bind, read_or
#include <cvc/ariadne/data.h> // a data look's package
#include <cvc/ariadne/loader.h> // parse_value: a parted look's rig file
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_spawns.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/entity_view.h>  // entity_drawn_as_node: who has a node
#include <cvc/gl/ariadne/scene_labels.h> // entity_labels:
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/state/state.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

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
  if (!d.axles.empty()) {
    double lo = d.axles.front().x, hi = lo;
    for (const cvc::physics::axle_desc &a : d.axles) {
      lo = std::min(lo, a.x);
      hi = std::max(hi, a.x);
    }
    s.axle_x = 0.5 * (lo + hi);
  }
  return s;
}

// A wheel's pose at (x, y, z): turned about its upright by its steer and rolled about its axle by
// its spin, Rz(steer) Ry(spin), row-major.
std::array<double, 16> wheel_matrix(double x, double y, double z, double steer, double spin) {
  const double c = std::cos(steer), sn = std::sin(steer);
  const double cs = std::cos(spin), ss = std::sin(spin);
  return {c * cs, -sn, c * ss, x, sn * cs, c, sn * ss, y, -ss, 0.0, cs, z, 0.0, 0.0, 0.0, 1.0};
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

namespace {

// A parted look's parts as read, once per look: the chassis's and the wheel's meshes in the rig's
// frame (x forward, y left, z up), the wheel about its middle and, for a rig that reflects its
// right wheels, mirrored as well; each part's first base-colour texture; and the rig, in that
// frame, its radius found from the wheel mesh when it gives none.
struct parted_model {
  cvc::geometry chassis, wheel, wheel_reflected;
  cvc::image chassis_texture, wheel_texture;
  ari::vehicle_rig rig;
};

} // namespace

// The models a realized scene has fitted, by file and length (null: it could not be read), and
// the parted looks it has read, by look. The scene's own (RealizedScene::vehicle_models), on its
// render thread.
struct vehicle_model_cache {
  std::map<std::string, std::shared_ptr<const fitted_model>> models;
  std::map<std::string, std::shared_ptr<const parted_model>> parts;
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

// How a read model is fitted: to a vehicle's length (fit_vehicle_geometry), or into a box.
struct model_fit {
  double length = 0.0; // a vehicle's length
  bool box = false;    // into lo..hi instead, each axis scaled to fill it
  std::array<double, 3> lo{}, hi{};

  static model_fit vehicle(double length) {
    model_fit f;
    f.length = length;
    return f;
  }
  static model_fit into(const std::array<double, 3> &lo, const std::array<double, 3> &hi) {
    model_fit f;
    f.box = true;
    f.lo = lo;
    f.hi = hi;
    return f;
  }
  // Its part of a cache key.
  std::string key() const {
    if (!box)
      return std::to_string(length);
    std::string k = "box";
    for (int i = 0; i < 3; ++i)
      k += " " + std::to_string(lo[i]) + " " + std::to_string(hi[i]);
    return k;
  }
};

// A model's geometry in the box lo..hi: stood upright (a +Y-up file, `y_up`, turned about +X, so
// +Y goes to +Z: a rotation, never a mirror) and scaled on each axis to fill the box, its normals
// scaled inversely and made unit again.
cvc::geometry fit_into_box(cvc::geometry g, const std::array<double, 3> &lo,
                           const std::array<double, 3> &hi, bool y_up) {
  const auto pts = g.const_points();
  if (pts.empty())
    throw std::runtime_error("it has no points");
  const auto turn = [y_up](double x, double y, double z) {
    return y_up ? std::array<double, 3>{x, -z, y} : std::array<double, 3>{x, y, z};
  };
  double mlo[3] = {1e30, 1e30, 1e30}, mhi[3] = {-1e30, -1e30, -1e30};
  for (const auto &v : pts) {
    const std::array<double, 3> t = turn(v[0], v[1], v[2]);
    for (int k = 0; k < 3; ++k) {
      mlo[k] = std::min(mlo[k], t[k]);
      mhi[k] = std::max(mhi[k], t[k]);
    }
  }
  double s[3];
  for (int k = 0; k < 3; ++k)
    s[k] = mhi[k] - mlo[k] > 1e-12 ? (hi[k] - lo[k]) / (mhi[k] - mlo[k]) : 1.0;
  auto &moved = g.points();
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const std::array<double, 3> t = turn(pts[i][0], pts[i][1], pts[i][2]);
    for (int k = 0; k < 3; ++k)
      moved[i][k] = lo[k] + (t[k] - mlo[k]) * s[k];
  }
  const auto nn = g.const_normals();
  if (nn.size() == pts.size()) {
    auto &turned = g.normals();
    for (std::size_t i = 0; i < nn.size(); ++i) {
      std::array<double, 3> t = turn(nn[i][0], nn[i][1], nn[i][2]);
      double len = 0.0;
      for (int k = 0; k < 3; ++k) {
        t[k] = std::abs(s[k]) > 1e-12 ? t[k] / s[k] : t[k];
        len += t[k] * t[k];
      }
      len = len > 0.0 ? std::sqrt(len) : 1.0;
      for (int k = 0; k < 3; ++k)
        turned[i][k] = t[k] / len;
    }
  }
  return g;
}

// A glTF file's axes: +Y up (the glTF spec's).
bool y_up_file(const std::string &path) {
  std::string ext = std::filesystem::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext == ".glb" || ext == ".gltf";
}

// A read model (the file `path` names its format) fitted as `fit` says, with its first base-colour
// texture. Throws when it has no triangles.
std::shared_ptr<const fitted_model> fit_read(cvc::model &m, const model_fit &fit,
                                             const std::string &path) {
  cvc::geometry g = m.merged();
  if (g.num_tris() == 0)
    throw std::runtime_error("it has no triangles");
  auto f = std::make_shared<fitted_model>();
  for (const auto &mat : m.materials)
    if (mat.has_base_color_texture()) {
      f->texture = mat.base_color_texture;
      break;
    }
  f->geometry = fit.box ? fit_into_box(std::move(g), fit.lo, fit.hi, y_up_file(path))
                        : fit_vehicle_geometry(std::move(g), fit.length);
  return f;
}

// What a model stands in for when it cannot be drawn, for a warning: a vehicle's box, a node's own
// geometry.
std::string drawn_instead(const model_fit &fit) {
  return fit.box ? "drawn as its own geometry" : "drawn as its box";
}

std::shared_ptr<const fitted_model> fitted_file(cvc::app &app, vehicle_model_cache &cache,
                                                const std::string &path, const model_fit &fit,
                                                std::vector<std::string> *warnings) {
  const std::string key = path + "|" + fit.key();
  if (auto it = cache.models.find(key); it != cache.models.end())
    return it->second;
  std::shared_ptr<const fitted_model> out;
  try {
    cvc::model m = cvc::read_model(app, path);
    out = fit_read(m, fit, path);
  } catch (const std::exception &e) {
    if (warnings)
      warnings->push_back("ari: model '" + path + "': " + e.what() + "; " + drawn_instead(fit));
  }
  cache.models[key] = out;
  return out;
}

// The data store's request for a data look's package (its path: kept).
ari::DataSpec data_spec_of(const ari::vehicle_look &look) {
  ari::DataSpec s;
  s.uri = look.uri;
  s.path = look.subpath;
  return s;
}

// The store node of a data look's package, its load started unless it has loaded or failed: a
// failed load stays failed (DataStore::load would start it again, and a view polling it would
// fetch it on every frame).
std::string data_node_of(cvc::app &app, const ari::vehicle_look &look) {
  const ari::DataSpec spec = data_spec_of(look);
  const std::string node =
      std::string(ari::kDataRoot) + cvc::state::SEPARATOR + ari::data_key(spec);
  const std::string status = ari::data_status(app, node);
  if (status == "ready" || status == "error")
    return node;
  return ari::DataStore::of(app)->load(spec);
}

// A file in a loaded package: at its path, else by its name anywhere in the package.
ari::DataBytes package_file(cvc::app &app, const std::string &node, const std::string &file) {
  namespace fs = std::filesystem;
  ari::DataBytes bytes = ari::data_file(app, node, file);
  if (!bytes)
    for (const std::string &f : ari::data_files(app, node))
      if (fs::path(f).filename() == fs::path(file).filename())
        return ari::data_file(app, node, f);
  return bytes;
}

// A data look's model: from its package once it has loaded (`pending` while it loads), else a
// local install of the file by its name, else none.
std::shared_ptr<const fitted_model> fitted_data(cvc::app &app, vehicle_model_cache &cache,
                                                const ari::vehicle_look &look, const model_fit &fit,
                                                std::vector<std::string> *warnings, bool &pending) {
  namespace fs = std::filesystem;
  pending = false;
  const std::string key =
      "data|" + look.uri + "|" + look.subpath + "|" + look.path + "|" + fit.key();
  if (auto it = cache.models.find(key); it != cache.models.end())
    return it->second;
  const std::string node = data_node_of(app, look);
  const std::string status = ari::data_status(app, node);
  const std::string what =
      look.uri + (look.subpath.empty() ? "" : "/" + look.subpath) + "/" + look.path;
  std::shared_ptr<const fitted_model> out;
  if (status == "ready") {
    const ari::DataBytes bytes = package_file(app, node, look.path);
    try {
      if (!bytes)
        throw std::runtime_error("the package has no " + look.path);
      cvc::model m = cvc::read_model_from_memory(app, bytes->data(), bytes->size(),
                                                 fs::path(look.path).extension().string());
      out = fit_read(m, fit, look.path);
    } catch (const std::exception &e) {
      if (warnings)
        warnings->push_back("ari: model " + what + ": " + e.what() + "; " + drawn_instead(fit));
    }
  } else if (status == "error") {
    const std::string local = ari::find_vehicle_model(look.path);
    if (!local.empty()) {
      out = fitted_file(app, cache, local, fit, warnings);
    } else if (warnings) {
      const std::string error = get(cvc::state::instance(app), node + ".error");
      warnings->push_back("ari: model " + what + ": " +
                          (error.empty() ? std::string("its package did not load") : error) +
                          ", and it is not installed here; " + drawn_instead(fit));
    }
  } else {
    pending = true; // idle, fetching, unpacking or decoding
    return nullptr;
  }
  cache.models[key] = out;
  return out;
}

std::shared_ptr<const fitted_model> fitted(cvc::app &app, vehicle_model_cache &cache,
                                           const ari::vehicle_look &look, const model_fit &fit,
                                           std::vector<std::string> *warnings, bool &pending) {
  pending = false;
  if (look.parted()) // its parts (parted), not one model
    return nullptr;
  if (look.kind == "data")
    return fitted_data(app, cache, look, fit, warnings, pending);
  if (look.kind == "file" && (look.fit || fit.box))
    return fitted_file(app, cache, look.path, fit, warnings);
  return nullptr;
}

// Whether a look is drawn as a fitted model (attach_model), not as a mesh node or boxes: a
// one-piece model or a parted one.
bool fitted_look(const ari::vehicle_look &look) {
  return (look.kind == "file" && (look.fit || look.parted())) || look.kind == "data";
}

// ---- a parted look ----

// A cache key of a parted look's: where its parts are, and its rig given inline.
std::string parted_key(const ari::vehicle_look &look) {
  return "parts|" + look.kind + "|" + look.uri + "|" + look.subpath + "|" + look.chassis + "|" +
         look.wheel + "|" + look.rig_file + "|" +
         (look.rig.hubs.empty() ? std::string() : ari::vehicle_rig_text(look.rig));
}

// A file of a parted look's as bytes: from its package (a data look's loaded `node`), else from
// disk. Null when it is not there.
ari::DataBytes part_bytes(const std::string &node, cvc::app &app, const std::string &file) {
  if (!node.empty())
    return package_file(app, node, file);
  std::ifstream in(file, std::ios::binary);
  if (!in)
    return nullptr;
  return std::make_shared<const std::string>(std::istreambuf_iterator<char>(in),
                                             std::istreambuf_iterator<char>());
}

// A mesh turned from glTF's frame (+y up, +z forward, +x left) into the rig's (x forward, y left,
// z up): (x, y, z) to (z, x, y), a rotation.
void turn_y_up(cvc::geometry &g) {
  const auto pts = g.const_points();
  auto &moved = g.points();
  for (std::size_t i = 0; i < pts.size(); ++i)
    moved[i] = {pts[i][2], pts[i][0], pts[i][1]};
  const auto nn = g.const_normals();
  if (!nn.empty()) {
    auto &turned = g.normals();
    for (std::size_t i = 0; i < nn.size(); ++i)
      turned[i] = {nn[i][2], nn[i][0], nn[i][1]};
  }
}

// A part's mesh (its meshes merged) and its first base-colour texture, turned into the rig's frame.
cvc::geometry read_part(cvc::app &app, const std::string &node, const std::string &file, bool y_up,
                        cvc::image &texture) {
  namespace fs = std::filesystem;
  cvc::model m;
  if (node.empty()) {
    m = cvc::read_model(app, file);
  } else {
    const ari::DataBytes bytes = part_bytes(node, app, file);
    if (!bytes)
      throw std::runtime_error("the package has no " + file);
    m = cvc::read_model_from_memory(app, bytes->data(), bytes->size(),
                                    fs::path(file).extension().string());
  }
  cvc::geometry g = m.merged();
  if (g.num_tris() == 0)
    throw std::runtime_error(file + " has no triangles");
  for (const auto &mat : m.materials)
    if (mat.has_base_color_texture()) {
      texture = mat.base_color_texture;
      break;
    }
  if (y_up)
    turn_y_up(g);
  return g;
}

// A parted look's parts, read once: on disk (a file look), or from its package once it has loaded
// (`pending` while it loads; when it cannot load, the parts installed here by their names, as a
// one-piece data look falls back). Null when one cannot be read or its rig is not one (a warning
// says why).
std::shared_ptr<const parted_model> parted(cvc::app &app, vehicle_model_cache &cache,
                                           const ari::vehicle_look &look,
                                           std::vector<std::string> *warnings, bool &pending) {
  pending = false;
  if (!look.parted() || (look.kind != "file" && look.kind != "data"))
    return nullptr;
  const std::string key = parted_key(look);
  if (auto it = cache.parts.find(key); it != cache.parts.end())
    return it->second;
  ari::vehicle_look from = look; // where its parts are read: its package's node, or disk
  std::string node;
  if (look.kind == "data") {
    node = data_node_of(app, look);
    const std::string status = ari::data_status(app, node);
    if (status != "ready" && status != "error") {
      pending = true; // idle, fetching, unpacking or decoding
      return nullptr;
    }
    if (status == "error") { // installed here instead: under the package's directory's name
      node.clear();          // (share/cvc-scenes/rover: rover/chassis.glb), else by their own
      const std::string dir = std::filesystem::path(look.subpath).filename().string();
      for (std::string *part : {&from.chassis, &from.wheel, &from.rig_file})
        if (!part->empty()) {
          const std::string under =
              dir.empty() ? std::string() : ari::find_vehicle_model(dir + "/" + *part);
          *part = under.empty() ? ari::find_vehicle_model(*part) : under;
        }
      if (from.chassis.empty() || from.wheel.empty() ||
          (from.rig.hubs.empty() && from.rig_file.empty())) {
        if (warnings) {
          const std::string error =
              get(cvc::state::instance(app), data_node_of(app, look) + ".error");
          warnings->push_back("ari: parted model " + look.uri + ": " +
                              (error.empty() ? std::string("its package did not load") : error) +
                              ", and its parts are not installed here; drawn as its box");
        }
        cache.parts[key] = nullptr;
        return nullptr;
      }
    }
  }
  std::shared_ptr<parted_model> out = std::make_shared<parted_model>();
  try {
    out->rig = look.rig;
    if (out->rig.hubs.empty()) { // its rig's file
      const ari::DataBytes text = part_bytes(node, app, from.rig_file);
      if (!text)
        throw std::runtime_error("no rig " + from.rig_file);
      ari::Value v;
      std::string why;
      if (!ari::parse_value(*text, v, &why) || !ari::parse_vehicle_rig(v, out->rig, &why))
        throw std::runtime_error("rig " + from.rig_file + ": " + why);
    }
    ari::vehicle_rig &rig = out->rig;
    if (rig.y_up)
      for (std::array<double, 3> &h : rig.hubs)
        h = {h[2], h[0], h[1]};
    out->chassis = read_part(app, node, from.chassis, rig.y_up, out->chassis_texture);
    out->wheel = read_part(app, node, from.wheel, rig.y_up, out->wheel_texture);
    // The wheel about its middle; its radius, when the rig gives none, half its larger extent
    // across its axle (along x or z).
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const auto &v : out->wheel.const_points())
      for (int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], static_cast<double>(v[k]));
        hi[k] = std::max(hi[k], static_cast<double>(v[k]));
      }
    auto &pts = out->wheel.points();
    for (auto &v : pts)
      for (int k = 0; k < 3; ++k)
        v[k] -= 0.5 * (lo[k] + hi[k]);
    if (!(rig.radius > 0.0))
      rig.radius = 0.5 * std::max(hi[0] - lo[0], hi[2] - lo[2]);
    if (!(rig.radius > 0.0))
      throw std::runtime_error("its wheel has no size");
    if (rig.mirror == "reflect") { // mirrored across its xz plane, its triangles wound again
      cvc::geometry r = out->wheel;
      for (auto &v : r.points())
        v[1] = -v[1];
      for (auto &n : r.normals())
        n[1] = -n[1];
      for (auto &t : r.tris())
        std::swap(t[1], t[2]);
      out->wheel_reflected = std::move(r);
    }
  } catch (const std::exception &e) {
    if (warnings)
      warnings->push_back("ari: parted model (" + from.chassis + ", " + from.wheel +
                          "): " + e.what() + "; drawn as its box");
    out.reset();
  }
  cache.parts[key] = out;
  return out;
}

// How a parted model sits on a vehicle (the namespace comment in vehicle_view.h): box = s m + o
// for a point m of the chassis's mesh, each hub's place and its height at rest, and each wheel's
// scale.
struct parted_fit {
  std::array<double, 3> scale{1.0, 1.0, 1.0}, offset{0.0, 0.0, 0.0};
  double wheel_scale = 1.0;
  std::vector<std::array<double, 3>> hubs; // in the chassis box's frame
};

parted_fit fit_parts(const ari::vehicle_rig &rig, const vehicle_shape &s, double base_z) {
  parted_fit f;
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300}, mean_z = 0.0;
  for (const std::array<double, 3> &h : rig.hubs) {
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], h[k]);
      hi[k] = std::max(hi[k], h[k]);
    }
    mean_z += h[2] / static_cast<double>(rig.hubs.size());
  }
  const double sz = s.wheel_radius > 0.0 ? s.wheel_radius / rig.radius : 1.0;
  const double wb = hi[0] - lo[0], tr = hi[1] - lo[1];
  const double sx = wb > 1e-9 && s.wheelbase > 0.0 ? s.wheelbase / wb : sz;
  const double sy = tr > 1e-9 && s.track > 0.0 ? s.track / tr : sz;
  const double mx = 0.5 * (lo[0] + hi[0]), my = 0.5 * (lo[1] + hi[1]);
  const double ground = mean_z - rig.radius;
  f.scale = {sx, sy, sz};
  f.offset = {s.axle_x - sx * mx, -sy * my, base_z - sz * ground};
  f.wheel_scale = sz;
  for (const std::array<double, 3> &h : rig.hubs)
    f.hubs.push_back({sx * h[0] + f.offset[0], sy * h[1] + f.offset[1], sz * h[2] + f.offset[2]});
  return f;
}

void colour(GeometryNode &node, const cvc::image &texture, const float color[3]) {
  if (!texture.empty()) {
    node.setUseSingleColor(false);
    node.setTexture(texture, false);
  } else {
    node.setUseSingleColor(true);
    node.setColor(color[0], color[1], color[2]);
  }
  node.setAmbient(0.3);
  node.setDiffuse(0.8);
}

// A ready parted model under `group` as id_chassis and id_wheel<k> (vehicle_view.h). False when it
// is not ready.
bool attach_parts(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                  const std::string &id, const ari::vehicle_look &look, const vehicle_shape &s,
                  double base_z, const float color[3], std::vector<drawn_wheel> *wheels,
                  std::vector<std::string> *warnings) {
  bool pending = false;
  const std::shared_ptr<const parted_model> m = parted(app, cache, look, warnings, pending);
  if (!m)
    return false;
  const parted_fit f = fit_parts(m->rig, s, base_z);
  std::shared_ptr<GeometryNode> chassis =
      group.createChild<GeometryNode>(id + "_chassis", m->chassis);
  if (!chassis)
    return false;
  chassis->setPosition(f.offset[0], f.offset[1], f.offset[2]);
  chassis->setScale(f.scale[0], f.scale[1], f.scale[2]);
  colour(*chassis, m->chassis_texture, color);
  static const float tyre[3] = {0.07f, 0.07f, 0.08f};
  for (std::size_t k = 0; k < f.hubs.size(); ++k) {
    const std::array<double, 3> &h = f.hubs[k];
    const std::string w = id + "_wheel" + std::to_string(k);
    std::shared_ptr<GraphicsNode> hub = group.createChild(w);
    if (!hub)
      continue;
    const bool right = h[1] < 0.0;
    const bool reflect = right && m->rig.mirror == "reflect";
    std::shared_ptr<GeometryNode> mesh =
        hub->createChild<GeometryNode>(w + "_mesh", reflect ? m->wheel_reflected : m->wheel);
    if (!mesh)
      continue;
    if (right && m->rig.mirror == "turn") // before the scale: setRotation drops it
      mesh->setRotation(0.0, 0.0, 180.0); // half a turn about its upright: its outside to -y
    mesh->setScale(f.wheel_scale, f.wheel_scale, f.wheel_scale);
    colour(*mesh, m->wheel_texture, tyre);
    drawn_wheel d;
    d.node = hub;
    d.x = h[0];
    d.y = h[1];
    d.rest_z = h[2];
    d.steers = k < m->rig.steers.size() && m->rig.steers[k];
    const std::array<double, 16> rest = wheel_matrix(d.x, d.y, d.rest_z, 0.0, 0.0);
    hub->setPoseMatrix(rest.data());
    if (wheels)
      wheels->push_back(d);
  }
  return true;
}

// The fitted model under `group`: its base `base_z` below the group's origin (the ground under a
// chassis), turned and lifted as the look says. False when it cannot be read.
bool attach_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                  const std::string &id, const ari::vehicle_look &look, const vehicle_shape &s,
                  double base_z, const float color[3], std::vector<std::string> *warnings) {
  if (look.parted()) // its parts named after the group (id_chassis, id_wheel<k>), at rest
    return attach_parts(app, cache, group, group.getName(), look, s, base_z, color, nullptr,
                        warnings);
  bool pending = false;
  std::shared_ptr<const fitted_model> m =
      fitted(app, cache, look, model_fit::vehicle(s.length), warnings, pending);
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

vehicle_model_status vehicle_model_state(cvc::app &app, vehicle_model_cache &cache,
                                         const ari::vehicle_look &look, double length,
                                         std::vector<std::string> *warnings) {
  if (look.parted())
    return parted_model_state(app, cache, look, warnings);
  bool pending = false;
  const std::shared_ptr<const fitted_model> m =
      fitted(app, cache, look, model_fit::vehicle(length), warnings, pending);
  return m ? vehicle_model_status::ready
           : (pending ? vehicle_model_status::pending : vehicle_model_status::failed);
}

ari::vehicle_look node_model_look(const ari::Value &model, const std::vector<ari::SceneData> &data,
                                  const std::string &where, const std::string &instead,
                                  std::vector<std::string> *warnings) {
  namespace fs = std::filesystem;
  const auto warn = [&](const std::string &w) {
    if (warnings)
      warnings->push_back(where + ": model: " + w + "; drawn as " + instead);
  };
  ari::vehicle_look look;
  look.fit = true;
  if (model.is_map()) {
    const ari::Value *d = model.find("data"), *f = model.find("file");
    const std::string name = d ? d->as_string() : std::string();
    const std::string file = f ? f->as_string() : std::string();
    if (name.empty() || file.empty()) {
      warn("expects { data: NAME, file: FILE }");
      return look;
    }
    for (const ari::SceneData &e : data)
      if (e.name == name && !e.uri.empty()) {
        look.kind = "data";
        look.data = name;
        look.uri = e.uri;
        look.subpath = e.path;
        look.path = file;
        return look;
      }
    warn("no data: entry '" + name + "' in the document");
    return look;
  }
  const std::string name = model.as_string();
  if (name.empty() || name == "none")
    return look;
  std::error_code ec;
  std::string path;
  if (fs::is_regular_file(fs::path(name), ec))
    path = name;
  else
    path = ari::find_vehicle_model(name);
  if (path.empty() && fs::path(name).extension().empty())
    path = ari::find_vehicle_model(name + ".glb");
  if (path.empty()) {
    warn("no model '" + name + "'");
    return look;
  }
  look.kind = "file";
  look.path = path;
  return look;
}

vehicle_model_status boxed_model_state(cvc::app &app, vehicle_model_cache &cache,
                                       const ari::vehicle_look &look,
                                       const std::array<double, 3> &lo,
                                       const std::array<double, 3> &hi,
                                       std::vector<std::string> *warnings) {
  bool pending = false;
  const std::shared_ptr<const fitted_model> m =
      fitted(app, cache, look, model_fit::into(lo, hi), warnings, pending);
  return m ? vehicle_model_status::ready
           : (pending ? vehicle_model_status::pending : vehicle_model_status::failed);
}

bool set_boxed_model(cvc::app &app, vehicle_model_cache &cache, GeometryNode &node,
                     const ari::vehicle_look &look, const std::array<double, 3> &lo,
                     const std::array<double, 3> &hi, std::vector<std::string> *warnings) {
  bool pending = false;
  const std::shared_ptr<const fitted_model> m =
      fitted(app, cache, look, model_fit::into(lo, hi), warnings, pending);
  if (!m)
    return false;
  node.setGeometry(m->geometry);
  if (!m->texture.empty()) {
    node.setUseSingleColor(false);
    node.setTexture(m->texture, false);
    node.setColor(1.0, 1.0, 1.0); // the texture's own colours
  }
  return true;
}

vehicle_model_status parted_model_state(cvc::app &app, vehicle_model_cache &cache,
                                        const ari::vehicle_look &look,
                                        std::vector<std::string> *warnings) {
  bool pending = false;
  const std::shared_ptr<const parted_model> m = parted(app, cache, look, warnings, pending);
  return m ? vehicle_model_status::ready
           : (pending ? vehicle_model_status::pending : vehicle_model_status::failed);
}

bool attach_parted_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                         const std::string &id, const ari::vehicle_look &look,
                         const vehicle_shape &shape, double base_z, const float color[3],
                         std::vector<drawn_wheel> *wheels, std::vector<std::string> *warnings) {
  return attach_parts(app, cache, group, id, look, shape, base_z, color, wheels, warnings);
}

std::vector<std::array<double, 16>>
parted_wheel_poses(const std::vector<drawn_wheel> &wheels,
                   const std::vector<ari::entity_wheel> &plant) {
  // The plant's wheel nearest (x, y), on the same side first (y's sign), among those `ok` takes.
  const auto nearest = [&](const drawn_wheel &d, bool steered) -> const ari::entity_wheel * {
    const ari::entity_wheel *best = nullptr;
    double best_d = 0.0;
    bool best_side = false;
    for (const ari::entity_wheel &e : plant) {
      if (steered && !e.steered)
        continue;
      const bool side = (e.y < 0.0) == (d.y < 0.0);
      const double dd = (e.x - d.x) * (e.x - d.x) + (e.y - d.y) * (e.y - d.y);
      if (!best || (side && !best_side) || (side == best_side && dd < best_d)) {
        best = &e;
        best_d = dd;
        best_side = side;
      }
    }
    return best;
  };
  std::vector<std::array<double, 16>> out;
  out.reserve(wheels.size());
  for (const drawn_wheel &d : wheels) {
    const ari::entity_wheel *m = nearest(d, false);
    const ari::entity_wheel *s = d.steers ? nearest(d, true) : nullptr;
    out.push_back(
        wheel_matrix(d.x, d.y, m ? m->z : d.rest_z, s ? s->steer : 0.0, m ? m->spin : 0.0));
  }
  return out;
}

bool attach_vehicle_model(cvc::app &app, vehicle_model_cache &cache, GraphicsNode &group,
                          const std::string &id, const ari::vehicle_look &look,
                          const vehicle_shape &shape, double base_z, const float color[3],
                          std::vector<std::string> *warnings) {
  return attach_model(app, cache, group, id, look, shape, base_z, color, warnings);
}

vehicle_shape vehicle_shape_of(const cvc::physics::vehicle_desc &desc) { return shape_of(desc); }

namespace {

// A box of size l at c into `out` (flat: four corners and a normal per face), in role `role`.
void add_box(vehicle_look_mesh &out, const double c[3], const double l[3], unsigned char role,
             const unsigned char own[3]) {
  static const int kFaces[6][4][3] = {{{1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}},
                                      {{-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1}},
                                      {{1, 1, -1}, {-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}},
                                      {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}},
                                      {{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}},
                                      {{-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1}}};
  static const double kNormals[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                        {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  for (int f = 0; f < 6; ++f) {
    const auto base = static_cast<std::uint32_t>(out.verts.size() / 3);
    for (int k = 0; k < 4; ++k) {
      for (int a = 0; a < 3; ++a) {
        out.verts.push_back(c[a] + 0.5 * l[a] * kFaces[f][k][a]);
        out.normals.push_back(kNormals[f][a]);
      }
      out.role.push_back(role);
      out.own.insert(out.own.end(), own, own + 3);
    }
    out.tris.insert(out.tris.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
  }
}

} // namespace

vehicle_model_status vehicle_look_mesh_of(cvc::app &app, vehicle_model_cache &cache,
                                          const ari::vehicle_look &look, const vehicle_shape &s,
                                          double base_z, vehicle_look_mesh &out,
                                          std::vector<std::string> *warnings) {
  out = vehicle_look_mesh{};
  if (fitted_look(look)) {
    bool pending = false;
    const std::shared_ptr<const fitted_model> m =
        fitted(app, cache, look, model_fit::vehicle(s.length), warnings, pending);
    if (!m)
      return pending ? vehicle_model_status::pending : vehicle_model_status::failed;
    // As attach_model places it: scaled, turned by the look's yaw, its base base_z + z below.
    const cvc::geometry &g = m->geometry;
    const double yaw = look.yaw * 3.14159265358979323846 / 180.0;
    const double cy = std::cos(yaw), sy = std::sin(yaw), sc = look.scale, dz = base_z + look.z;
    const auto &pts = g.points();
    const bool normals = g.normals().size() == pts.size();
    const bool uvs = g.uvs().size() == pts.size();
    out.verts.reserve(3 * pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const double x = sc * pts[i][0], y = sc * pts[i][1], z = sc * pts[i][2];
      out.verts.insert(out.verts.end(), {cy * x - sy * y, sy * x + cy * y, z + dz});
      if (normals) {
        const auto &n = g.normals()[i];
        out.normals.insert(out.normals.end(), {cy * n[0] - sy * n[1], sy * n[0] + cy * n[1], n[2]});
      }
      if (uvs)
        out.uvs.insert(out.uvs.end(),
                       {static_cast<float>(g.uvs()[i][0]), static_cast<float>(g.uvs()[i][1])});
    }
    for (const auto &t : g.tris())
      out.tris.insert(out.tris.end(),
                      {static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]),
                       static_cast<std::uint32_t>(t[2])});
    if (!normals) { // area-weighted, from its triangles
      out.normals.assign(out.verts.size(), 0.0);
      for (std::size_t t = 0; t + 2 < out.tris.size(); t += 3) {
        const double *a = &out.verts[3 * out.tris[t]], *b = &out.verts[3 * out.tris[t + 1]],
                     *c = &out.verts[3 * out.tris[t + 2]];
        const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
                     v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const double n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                             u[0] * v[1] - u[1] * v[0]};
        for (int k = 0; k < 3; ++k)
          for (int a2 = 0; a2 < 3; ++a2)
            out.normals[3 * out.tris[t + k] + a2] += n[a2];
      }
      for (std::size_t i = 0; i + 2 < out.normals.size(); i += 3) {
        const double len =
            std::sqrt(out.normals[i] * out.normals[i] + out.normals[i + 1] * out.normals[i + 1] +
                      out.normals[i + 2] * out.normals[i + 2]);
        for (int a2 = 0; a2 < 3; ++a2)
          out.normals[i + a2] = len > 0.0 ? out.normals[i + a2] / len : (a2 == 2 ? 1.0 : 0.0);
      }
    }
    const bool textured = !m->texture.empty() && uvs;
    if (textured)
      out.texture = std::shared_ptr<const cvc::image>(m, &m->texture);
    else
      out.uvs.clear();
    static const unsigned char white[3] = {255, 255, 255};
    out.role.assign(pts.size(), textured ? 2 : 0);
    out.own.clear();
    for (std::size_t i = 0; i < pts.size(); ++i)
      out.own.insert(out.own.end(), white, white + 3);
    out.model = true;
    return out.tris.empty() ? vehicle_model_status::failed : vehicle_model_status::ready;
  }
  if (look.kind == "file" && !look.path.empty())
    return vehicle_model_status::failed; // a mesh node of its own (vehicle_scene_node)
  // The box look, as vehicle_scene_node lays it out.
  const double L = s.length, W = s.width, H = s.height;
  static const unsigned char tyre[3] = {18, 18, 20}, none[3] = {0, 0, 0};
  const double body_c[3] = {0.0, 0.0, 0.0}, body_l[3] = {L, W, H};
  add_box(out, body_c, body_l, 0, none);
  const double cab_c[3] = {-0.12 * L, 0.0, 0.5 * H + 0.375 * H},
               cab_l[3] = {0.42 * L, 0.9 * W, 0.75 * H};
  add_box(out, cab_c, cab_l, 1, none);
  const double r = s.wheel_radius, ww = std::max(0.02, 0.16 * W);
  for (const double x : {0.5 * s.wheelbase, -0.5 * s.wheelbase})
    for (const double y : {0.5 * s.track, -0.5 * s.track}) {
      const double wc[3] = {x, y, -0.5 * H}, wl[3] = {2.0 * r, ww, 2.0 * r};
      add_box(out, wc, wl, 2, tyre);
    }
  return vehicle_model_status::ready;
}

ari::SceneNode vehicle_scene_node(const std::string &id, const ari::vehicle_look &look,
                                  const vehicle_shape &s, const float color[3],
                                  const float color2[3]) {
  ari::SceneNode g;
  g.id = id;
  g.type = "group";
  if (fitted_look(look))
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
  if (!v)
    return;
  if (!r.vehicle_models)
    r.vehicle_models = std::make_shared<vehicle_model_cache>();
  const std::shared_ptr<vehicle_model_cache> models = r.vehicle_models;
  SceneGraph &sg = *r.sg;
  // All of them again when the vehicles' revision moved, or when the looks a box stood in for have
  // come in (drawn as boxes while a look's package loads); else only who the entity view promotes
  // or demotes (entity_view.h).
  bool all = v->revision() != r.vehicles_rev;
  if (!all && r.vehicle_models_pending) {
    all = true;
    for (const ari::vehicle_archetype &x : v->archetypes())
      if (x.look.kind == "data") {
        const std::string status =
            ari::data_status(sg.appContext(), data_node_of(sg.appContext(), x.look));
        if (status != "ready" && status != "error")
          all = false;
      }
  }
  if (!all && !entity_nodes_changed(r))
    return;
  std::set<std::uint64_t> drawn;
  if (all) {
    r.vehicles_rev = v->revision();
    r.vehicle_models_pending = false;
    for (const auto &[node, chassis] : r.vehicle_markers) {
      if (chassis.valid() && r.physics_binding->bound(chassis))
        r.physics_binding->unbind(chassis);
      sg.removeGraphics(node);
    }
    r.vehicle_markers.clear();
    r.vehicle_wheels.clear();
  } else { // the demoted go; the others stay as they are
    for (auto it = r.vehicle_markers.begin(); it != r.vehicle_markers.end();) {
      const std::uint64_t id = std::stoull(it->first.substr(std::string("vehicle_").size()));
      if (entity_drawn_as_node(r, id)) {
        drawn.insert(id);
        ++it;
        continue;
      }
      if (it->second.valid() && r.physics_binding->bound(it->second))
        r.physics_binding->unbind(it->second);
      sg.removeGraphics(it->first);
      r.vehicle_wheels.erase(it->first);
      it = r.vehicle_markers.erase(it);
    }
  }
  for (const ari::entity_info &e : v->entities()) {
    if (!e.placed || drawn.count(e.id) || !entity_drawn_as_node(r, e.id))
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
    const cvc::physics::vehicle_desc desc = ari::vehicle_desc_of(*a, e.spec.mass, e.spec.max_speed);
    const vehicle_shape shape = shape_of(desc);
    // A fitted model's base, below its chassis. A plant's chassis rides on the grid's ground, so
    // the base is that ground's depth under it. A `pose` entity's is wherever its poses put it, on
    // the terrain of whatever placed it, which the grid may not match (flat, or coarser), and the
    // view is rebuilt whenever an entity comes or goes: its model hangs its stand height below its
    // chassis, where it stands when a pose gives no z.
    const double base_z = e.posed ? -ari::stand_height(desc) : e.ground_z - e.z;
    ari::Scene one;
    one.nodes.push_back(
        vehicle_scene_node(id, a->look, shape, e.spec.color.data(), e.spec.color2.data()));
    realize_scene(sg, one, r.bind_prefix, warnings);
    std::shared_ptr<GraphicsNode> node = sg.getGraphics(id);
    if (!node)
      continue;
    std::vector<drawn_wheel> wheels; // a parted look's, posed each frame
    if (fitted_look(a->look) &&
        !(a->look.parted() ? attach_parts(sg.appContext(), *models, *node, id, a->look, shape,
                                          base_z, e.spec.color.data(), &wheels, warnings)
                           : attach_model(sg.appContext(), *models, *node, id + "_mesh", a->look,
                                          shape, base_z, e.spec.color.data(), warnings))) {
      if (vehicle_model_state(sg.appContext(), *models, a->look, shape.length) ==
          vehicle_model_status::pending)
        r.vehicle_models_pending = true;
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
    if (!wheels.empty())
      r.vehicle_wheels[id] = std::move(wheels);
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
    const std::uint64_t id = std::stoull(node.substr(std::string("vehicle_").size()));
    if (auto parts = r.vehicle_wheels.find(node); parts != r.vehicle_wheels.end()) {
      const std::vector<std::array<double, 16>> poses =
          parted_wheel_poses(parts->second, v->wheels(id));
      for (std::size_t k = 0; k < poses.size(); ++k)
        if (const std::shared_ptr<GraphicsNode> w = parts->second[k].node.lock())
          w->setPoseMatrix(poses[k].data()); // change-detected, published rather than written
      continue;
    }
    const std::shared_ptr<GraphicsNode> group = sg.getGraphics(node);
    const std::shared_ptr<GraphicsNode> first =
        group ? group->findChildByName(node + "_wheel0") : nullptr;
    if (!first) // a one-piece model
      continue;
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
      const std::array<double, 16> m =
          wheel_matrix(best->x, best->y, best->z, best->steer, best->spin);
      w->setPoseMatrix(m.data());
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
  // The look drawn as its box while its package loads, and its length: drawn again once loaded.
  auto loading = std::make_shared<std::optional<std::pair<ari::vehicle_look, double>>>();
  if (!out.vehicle_models)
    out.vehicle_models = std::make_shared<vehicle_model_cache>();
  const std::shared_ptr<vehicle_model_cache> models = out.vehicle_models;
  std::weak_ptr<GraphicsNode> held = anchor;
  out.custom_ticks.push_back([=, &app](vtkRenderer *) {
    if (held.expired())
      return;
    if (*loading && vehicle_model_state(app, *models, (*loading)->first, (*loading)->second) !=
                        vehicle_model_status::pending) {
      loading->reset();
      last->clear();
    }
    cvc::state &st = cvc::state::instance(app);
    const std::string sel = get(st, root + ".selected");
    const std::string a = root + ".archetypes." + sel + ".";
    const std::string sig = sel + "|" + get(st, a + "length") + "|" + get(st, a + "width") + "|" +
                            get(st, a + "height") + "|" + get(st, a + "look.kind") + "|" +
                            get(st, a + "look.path") + "|" + get(st, a + "look.uri") + "|" +
                            get(st, a + "look.fit") + "|" + get(st, a + "look.chassis") + "|" +
                            get(st, a + "look.wheel") + "|" + get(st, a + "look.rig") + "|" +
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
        look.uri = get(st, a + "look.uri");
        look.subpath = get(st, a + "look.subpath");
        look.scale = num(get(st, a + "look.scale"), 1.0);
        look.yaw = num(get(st, a + "look.yaw"), 0.0);
        look.z = num(get(st, a + "look.z"), 0.0);
        look.fit = get(st, a + "look.fit") == "1";
        look.chassis = get(st, a + "look.chassis");
        look.wheel = get(st, a + "look.wheel");
        const std::string rig = get(st, a + "look.rig"); // its file, or its text when inline
        ari::Value inline_rig;
        if (rig.find('{') == 0 && ari::parse_value(rig, inline_rig))
          ari::parse_vehicle_rig(inline_rig, look.rig);
        else
          look.rig_file = rig;
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
        const bool model = fitted_look(look);
        if (model) // the model stands on the set by itself: its base at the node's height
          node.position[2] = static_cast<float>(at[2]);
        one.nodes.push_back(node);
        realize_scene(*graph, one, prefix, nullptr);
        if (model)
          if (std::shared_ptr<GraphicsNode> g = graph->getGraphics(drawn))
            if (!attach_model(app, *models, *g, drawn + "_mesh", look, s, 0.0, c1, nullptr)) {
              if (vehicle_model_state(app, *models, look, s.length) ==
                  vehicle_model_status::pending)
                *loading = std::make_pair(look, s.length);
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

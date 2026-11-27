/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Version 2 city bundles (city_bundle.h, ARIADNE_WORLDS section 18.8): a world_geometry written as
// a manifest, the terrain's heights and cell surfaces, and a glb per mesh item, and read back.

#include "city_bundle_v2.h"

#include <algorithm>
#include <array>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/geometry/height_field.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h>
#include <cvc/image/image_encode.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/utility/archive.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/surface.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace world {

namespace {

namespace fs = std::filesystem;
using ptree = boost::property_tree::ptree;

// ── JSON out: written by hand, so numbers stay numbers (property_tree quotes them) at full
// precision, and the same values always write the same text. ──
std::string jstr(const std::string &v) {
  std::string o = "\"";
  for (const char c : v) {
    switch (c) {
    case '"':
      o += "\\\"";
      break;
    case '\\':
      o += "\\\\";
      break;
    case '\n':
      o += "\\n";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        char b[8];
        std::snprintf(b, sizeof b, "\\u%04x", static_cast<unsigned char>(c));
        o += b;
      } else {
        o.push_back(c);
      }
    }
  }
  return o + "\"";
}
std::string jnum(double v) {
  if (!std::isfinite(v))
    throw std::invalid_argument("city_bundle_bytes: a value is not finite");
  char b[32];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}
std::string jint(long long v) { return std::to_string(v); }
std::string juint(std::uint64_t v) { return std::to_string(v); }
std::string jbool(bool v) { return v ? "true" : "false"; }
// An object from (key, already-written value) pairs, in the order given.
std::string jobj(const std::vector<std::pair<std::string, std::string>> &kv,
                 const std::string &indent = "") {
  if (kv.empty())
    return "{}";
  std::string o = "{\n";
  for (std::size_t i = 0; i < kv.size(); ++i)
    o += indent + "  " + jstr(kv[i].first) + ": " + kv[i].second +
         (i + 1 < kv.size() ? ",\n" : "\n");
  return o + indent + "}";
}
std::string jarr(const std::vector<std::string> &vs) {
  std::string o = "[";
  for (std::size_t i = 0; i < vs.size(); ++i)
    o += (i ? ", " : "") + vs[i];
  return o + "]";
}

// ── JSON in: property_tree (every value a string; arrays are children with empty keys). ──
ptree parse_json(const std::string &text, const std::string &what) {
  ptree pt;
  std::istringstream in(text);
  try {
    boost::property_tree::read_json(in, pt);
  } catch (const std::exception &e) {
    throw std::runtime_error(what + " is not JSON: " + e.what());
  }
  return pt;
}
std::vector<std::string> jlist(const ptree &pt, const std::string &key) {
  std::vector<std::string> out;
  if (const auto c = pt.get_child_optional(key))
    for (const auto &kv : *c)
      out.push_back(kv.second.get_value<std::string>());
  return out;
}
std::vector<double> jdoubles(const ptree &pt, const std::string &key) {
  std::vector<double> out;
  for (const std::string &v : jlist(pt, key))
    out.push_back(std::stod(v));
  return out;
}

constexpr const char *kFormat = "cvc.city_bundle";
constexpr int kVersion = 2;

std::shared_ptr<const std::string> bytes_of(std::string s) {
  return std::make_shared<const std::string>(std::move(s));
}

std::string sha256_of(const std::string &s) {
  return sha256_hex(reinterpret_cast<const unsigned char *>(s.data()), s.size());
}

// Little-endian bytes of plain values, whatever the host's order.
template <typename T> void put_le(std::string &out, T v) {
  unsigned char b[sizeof(T)];
  std::memcpy(b, &v, sizeof(T));
  const std::uint16_t probe = 1;
  const bool little = *reinterpret_cast<const unsigned char *>(&probe) == 1;
  if (!little)
    std::reverse(b, b + sizeof(T));
  out.append(reinterpret_cast<const char *>(b), sizeof(T));
}

template <typename T> T get_le(const char *p) {
  unsigned char b[sizeof(T)];
  std::memcpy(b, p, sizeof(T));
  const std::uint16_t probe = 1;
  const bool little = *reinterpret_cast<const unsigned char *>(&probe) == 1;
  if (!little)
    std::reverse(b, b + sizeof(T));
  T v;
  std::memcpy(&v, b, sizeof(T));
  return v;
}

bool plain_file_name(const std::string &name) {
  if (name.empty() || name == "." || name == "..")
    return false;
  for (const char c : name)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.'))
      return false;
  return true;
}

std::string roles_json(std::uint8_t roles) {
  std::vector<std::string> r;
  if (roles & role_render)
    r.push_back(jstr("render"));
  if (roles & role_collide)
    r.push_back(jstr("collide"));
  if (roles & role_navigate)
    r.push_back(jstr("navigate"));
  return jarr(r);
}

std::uint8_t roles_of(const std::vector<std::string> &r, const std::string &where) {
  std::uint8_t out = 0;
  for (const std::string &s : r) {
    if (s == "render")
      out |= role_render;
    else if (s == "collide")
      out |= role_collide;
    else if (s == "navigate")
      out |= role_navigate;
    else
      throw std::runtime_error("load_city_bundle: " + where + ": unknown role '" + s + "'");
  }
  return out;
}

// One mesh of the item's faces on one surface (or its lines), its vertices the ones its faces use
// in the order the faces first use them. That is the order a glTF reader hands them back in, so
// what was read writes the same bytes again.
cvc::geometry faces_on(const cvc::geometry &g, const std::vector<std::size_t> &tris, bool lines) {
  cvc::geometry out;
  const std::size_t nv = g.const_points().size();
  std::vector<std::int64_t> remap(nv, -1);
  std::vector<std::size_t> order; // new index -> old
  auto use = [&](std::size_t v) {
    if (remap[v] < 0) {
      remap[v] = static_cast<std::int64_t>(order.size());
      order.push_back(v);
    }
  };
  if (lines)
    for (const auto &l : g.const_lines()) {
      use(l[0]);
      use(l[1]);
    }
  else
    for (const std::size_t t : tris)
      for (int k = 0; k < 3; ++k)
        use(g.const_tris()[t][k]);
  const bool normals = g.const_normals().size() == nv;
  const bool colors = g.const_colors().size() == nv;
  const bool uvs = g.const_uvs().size() == nv;
  for (const std::size_t v : order) {
    out.points().push_back(g.const_points()[v]);
    if (normals)
      out.normals().push_back(g.const_normals()[v]);
    if (colors)
      out.colors().push_back(g.const_colors()[v]);
    if (uvs)
      out.uvs().push_back(g.const_uvs()[v]);
  }
  if (lines) {
    for (const auto &l : g.const_lines()) {
      cvc::geometry::line_t nl;
      nl[0] = static_cast<cvc::geometry::index_t>(remap[l[0]]);
      nl[1] = static_cast<cvc::geometry::index_t>(remap[l[1]]);
      out.lines().push_back(nl);
    }
  } else {
    for (const std::size_t t : tris) {
      cvc::geometry::tri_t nt;
      for (int k = 0; k < 3; ++k)
        nt[k] = static_cast<cvc::geometry::index_t>(remap[g.const_tris()[t][k]]);
      out.tris().push_back(nt);
    }
  }
  return out;
}

// A mesh item as a glb: one mesh per surface its triangles use (in ascending surface order), the
// material named by the surface, and its lines as one more mesh on the item's surface.
std::string item_glb(app &ctx, const geometry_item &item, const surface_registry &surfaces) {
  const cvc::geometry &g = *item.geometry;
  const std::size_t nt = g.const_tris().size();
  const auto &fm = g.const_face_materials();
  const bool per_face = fm.size() == nt && nt > 0;
  std::map<std::uint16_t, std::vector<std::size_t>> by_surface;
  for (std::size_t t = 0; t < nt; ++t)
    by_surface[per_face ? fm[t] : item.surface].push_back(t);
  const bool has_lines = !g.const_lines().empty();

  cvc::model m;
  std::map<std::uint16_t, int> material_of;
  auto material_index = [&](std::uint16_t id) {
    auto hit = material_of.find(id);
    if (hit != material_of.end())
      return hit->second;
    cvc::material mat;
    mat.name = surfaces[id].name;
    for (int k = 0; k < 3; ++k)
      mat.base_color[k] = surfaces[id].albedo[k];
    mat.base_color[3] = 1.0;
    m.materials.push_back(mat);
    return material_of[id] = static_cast<int>(m.materials.size()) - 1;
  };
  for (const auto &[id, tris] : by_surface) {
    cvc::model::mesh mm;
    mm.geom = faces_on(g, tris, false);
    mm.material = material_index(id);
    mm.name = item.name + "." + surfaces[id].name;
    m.meshes.push_back(std::move(mm));
  }
  if (has_lines) {
    cvc::model::mesh mm;
    mm.geom = faces_on(g, {}, true);
    mm.material = material_index(item.surface);
    mm.name = item.name + ".lines";
    m.meshes.push_back(std::move(mm));
  }
  return write_model_to_memory(ctx, m, ".glb");
}

// Rows north first (row 0 at the maximum y), as terrain.json's grid: the world's rows reversed.
template <typename T>
std::vector<T> north_first(const std::vector<T> &rows_south_first, int rows, int cols) {
  std::vector<T> out(rows_south_first.size());
  for (int r = 0; r < rows; ++r)
    std::copy(rows_south_first.begin() + static_cast<std::size_t>(r) * cols,
              rows_south_first.begin() + static_cast<std::size_t>(r + 1) * cols,
              out.begin() + static_cast<std::size_t>(rows - 1 - r) * cols);
  return out;
}

std::shared_ptr<const std::string> need(const city_bundle_reader &read, const std::string &name,
                                        const std::string &label) {
  std::shared_ptr<const std::string> b = read(name);
  if (!b)
    throw std::runtime_error("load_city_bundle: " + label + ": world.json names " + name +
                             ", which the bundle does not have");
  return b;
}

} // namespace

city_bundle_files city_bundle_bytes(app &ctx, const world_geometry &world,
                                    const city_bundle_write &info) {
  const surface_registry &surfaces = world.surfaces();
  std::shared_ptr<const cvc::height_field> terrain;
  for (const geometry_item &it : world.items())
    if (!terrain && it.height_field())
      terrain = it.height_field();
  if (!terrain)
    throw std::invalid_argument("city_bundle_bytes: the world has no terrain (a height field)");

  city_bundle_files files;
  // The terrain: its extent and size (terrain.json, as version 1, with the exact origin and
  // spacing), its heights and cell surfaces north first.
  const int rows = terrain->rows(), cols = terrain->cols();
  files["terrain.json"] =
      bytes_of(jobj({{"bounds", jobj({{"min_x", jnum(terrain->x0())},
                                      {"min_y", jnum(terrain->y0())},
                                      {"max_x", jnum(terrain->max_x())},
                                      {"max_y", jnum(terrain->max_y())}},
                                     "  ")},
                     {"rows", jint(rows)},
                     {"cols", jint(cols)},
                     {"origin", jarr({jnum(terrain->x0()), jnum(terrain->y0())})},
                     {"spacing", jarr({jnum(terrain->dx()), jnum(terrain->dy())})},
                     {"grid_file", jstr("terrain.f32")}}) +
               "\n");
  {
    std::string h;
    h.reserve(terrain->heights().size() * 4);
    for (const float v : north_first(terrain->heights(), rows, cols))
      put_le<float>(h, v);
    files["terrain.f32"] = bytes_of(std::move(h));
  }
  const auto &cells = terrain->cell_materials();
  const bool cell_surfaces =
      cells.size() == static_cast<std::size_t>(rows - 1) * (cols - 1) && !cells.empty();
  if (cell_surfaces) {
    std::string c;
    c.reserve(cells.size() * 2);
    for (const auto v : north_first(cells, rows - 1, cols - 1))
      put_le<std::uint16_t>(c, v);
    files["terrain_surfaces.u16"] = bytes_of(std::move(c));
  }

  std::vector<std::string> items;
  std::set<std::string> written = {"world.json", "terrain.json", "terrain.f32",
                                   "terrain_surfaces.u16"};
  for (const geometry_item &it : world.items()) {
    if (!plain_file_name(it.name))
      throw std::invalid_argument("city_bundle_bytes: item '" + it.name +
                                  "' is not a plain file name");
    std::vector<std::pair<std::string, std::string>> ij = {
        {"name", jstr(it.name)},
        {"roles", roles_json(it.roles)},
        {"surface", jstr(surfaces[it.surface].name)}};
    if (it.height_field() == terrain) {
      ij.push_back({"terrain", jbool(true)});
      if (cell_surfaces)
        ij.push_back({"cell_surfaces", jstr("terrain_surfaces.u16")});
    } else if (it.height_field()) {
      throw std::invalid_argument("city_bundle_bytes: item '" + it.name +
                                  "' is a second height field; a bundle has one terrain");
    } else {
      const std::string file = it.name + ".glb";
      if (written.count(file))
        throw std::invalid_argument("city_bundle_bytes: item '" + it.name + "' would overwrite " +
                                    file);
      written.insert(file);
      files[file] = bytes_of(item_glb(ctx, it, surfaces));
      const auto &fm = it.geometry->const_face_materials();
      ij.push_back({"file", jstr(file)});
      ij.push_back(
          {"face_surfaces", jbool(fm.size() == it.geometry->const_tris().size() && !fm.empty())});
      ij.push_back(
          {"position", jarr({jnum(it.position[0]), jnum(it.position[1]), jnum(it.position[2])})});
      ij.push_back({"rotation", jarr({jnum(it.rotation[0]), jnum(it.rotation[1]),
                                      jnum(it.rotation[2]), jnum(it.rotation[3])})});
    }
    items.push_back(jobj(ij, "    "));
  }
  for (const auto &[name, b] : info.extra) {
    if (written.count(name) || !plain_file_name(name))
      throw std::invalid_argument("city_bundle_bytes: extra file '" + name +
                                  "' is one the bundle writes itself, or not a plain name");
    files[name] = b ? b : bytes_of(std::string());
  }

  std::vector<std::string> table;
  for (std::size_t i = 0; i < surfaces.size(); ++i)
    table.push_back(jstr(surfaces[static_cast<std::uint16_t>(i)].name));
  std::vector<std::string> extra;
  for (const auto &kv : info.extra)
    extra.push_back(jstr(kv.first));
  std::string item_list = "[\n";
  for (std::size_t i = 0; i < items.size(); ++i)
    item_list += "    " + items[i] + (i + 1 < items.size() ? ",\n" : "\n");
  item_list += "  ]";

  std::vector<std::pair<std::string, std::string>> m = {{"format", jstr(kFormat)},
                                                        {"version", jint(kVersion)},
                                                        {"ontology", jstr(surfaces.ontology())},
                                                        {"surfaces", jarr(table)},
                                                        {"edge_walls", jbool(world.edge_walls())},
                                                        {"items", item_list}};
  if (info.water_level)
    m.push_back({"water", jobj({{"level", jnum(*info.water_level)}}, "  ")});
  if (info.region)
    m.push_back({"region", jobj({{"min_x", jnum(info.region->min_x)},
                                 {"min_y", jnum(info.region->min_y)},
                                 {"max_x", jnum(info.region->max_x)},
                                 {"max_y", jnum(info.region->max_y)}},
                                "  ")});
  if (!info.generator.name.empty()) {
    const std::string params = info.generator.params.empty() ? "{}" : info.generator.params;
    parse_json(params, "city_bundle_bytes: the generator's params"); // it must be JSON
    m.push_back({"generator", jobj({{"name", jstr(info.generator.name)},
                                    {"version", jstr(info.generator.version)},
                                    {"seed", juint(info.generator.seed)},
                                    {"params", params}},
                                   "  ")});
  }
  m.push_back({"extra", jarr(extra)});
  m.push_back({"hash", jstr(city_bundle_hash(files))});
  files["world.json"] = bytes_of(jobj(m) + "\n");
  return files;
}

std::string city_bundle_hash(const city_bundle_files &files) {
  std::string list;
  for (const auto &[name, b] : files) { // a std::map: name order
    if (name == "world.json" || name.rfind("export/", 0) == 0)
      continue;
    list += name;
    list.push_back('\0');
    list += sha256_of(b ? *b : std::string());
    list.push_back('\n');
  }
  return sha256_of(list);
}

city_bundle_files city_bundle_exports(app &ctx, const world_geometry &world,
                                      std::optional<double> water_level, int max_samples) {
  const geometry_item *item = nullptr;
  for (const geometry_item &it : world.items())
    if (!item && it.height_field())
      item = &it;
  if (!item)
    throw std::invalid_argument("city_bundle_exports: the world has no terrain (a height field)");
  const std::shared_ptr<const cvc::height_field> t = item->height_field();
  const int rows = t->rows(), cols = t->cols();
  max_samples = std::max(2, max_samples);

  // The mesh: the terrain itself, or resampled over its extent to at most max_samples a side,
  // each cell taking the surface of the source cell under its centre.
  std::shared_ptr<const cvc::height_field> mesh = t;
  if (rows > max_samples || cols > max_samples) {
    const int mr = std::min(rows, max_samples), mc = std::min(cols, max_samples);
    const double sx = (t->max_x() - t->x0()) / (mc - 1), sy = (t->max_y() - t->y0()) / (mr - 1);
    const auto clamp_x = [&](double x) { return std::min(std::max(x, t->x0()), t->max_x()); };
    const auto clamp_y = [&](double y) { return std::min(std::max(y, t->y0()), t->max_y()); };
    std::vector<float> h(static_cast<std::size_t>(mr) * mc);
    for (int r = 0; r < mr; ++r)
      for (int c = 0; c < mc; ++c)
        h[static_cast<std::size_t>(r) * mc + c] =
            static_cast<float>(t->height(clamp_x(t->x0() + c * sx), clamp_y(t->y0() + r * sy)));
    std::vector<cvc::face_material_t> cells;
    if (!t->cell_materials().empty()) {
      cells.resize(static_cast<std::size_t>(mr - 1) * (mc - 1));
      for (int r = 0; r + 1 < mr; ++r)
        for (int c = 0; c + 1 < mc; ++c) {
          const int sr = std::min(t->cell_rows() - 1, int(((r + 0.5) * sy) / t->dy()));
          const int sc = std::min(t->cell_cols() - 1, int(((c + 0.5) * sx) / t->dx()));
          cells[static_cast<std::size_t>(r) * (mc - 1) + c] = t->cell_material(sr, sc);
        }
    }
    mesh = std::make_shared<const cvc::height_field>(mr, mc, t->x0(), t->y0(), sx, sy, std::move(h),
                                                     std::move(cells));
  }
  geometry_item as_mesh;
  as_mesh.name = "terrain";
  as_mesh.geometry = mesh;
  as_mesh.surface = item->surface;

  city_bundle_files files;
  files["export/terrain.glb"] = bytes_of(item_glb(ctx, as_mesh, world.surfaces()));

  // The height map: full size, north row at the top, the finite heights over 0..65535.
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (const float v : t->heights())
    if (std::isfinite(v)) {
      lo = std::min(lo, double(v));
      hi = std::max(hi, double(v));
    }
  if (!(lo <= hi))
    lo = hi = 0.0;
  cvc::image img(cols, rows, cvc::image::pixel_format::GRAY, cvc::image::data_type::u16);
  auto *px = reinterpret_cast<std::uint16_t *>(img.data());
  const double span = hi - lo;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const float v = t->heights()[static_cast<std::size_t>(r) * cols + c];
      const double u = std::isfinite(v) && span > 0.0 ? (double(v) - lo) / span : 0.0;
      px[static_cast<std::size_t>(rows - 1 - r) * cols + c] =
          static_cast<std::uint16_t>(std::lround(std::min(1.0, std::max(0.0, u)) * 65535.0));
    }
  files["export/heightmap.png"] = bytes_of(encode_png(img));

  std::vector<std::pair<std::string, std::string>> j = {
      {"format", jstr("cvc.heightmap")},
      {"image", jstr("heightmap.png")},
      {"rows", jint(rows)},
      {"cols", jint(cols)},
      {"north_first", jbool(true)},
      {"bounds", jobj({{"min_x", jnum(t->x0())},
                       {"min_y", jnum(t->y0())},
                       {"max_x", jnum(t->max_x())},
                       {"max_y", jnum(t->max_y())}},
                      "  ")},
      {"spacing", jarr({jnum(t->dx()), jnum(t->dy())})},
      {"min_height_m", jnum(lo)},
      {"max_height_m", jnum(hi)},
      {"height", jstr("min_height_m + value / 65535 * (max_height_m - min_height_m)")}};
  if (water_level)
    j.push_back({"water_level_m", jnum(*water_level)});
  j.push_back({"mesh", jobj({{"file", jstr("terrain.glb")},
                             {"rows", jint(mesh->rows())},
                             {"cols", jint(mesh->cols())}},
                            "  ")});
  files["export/heightmap.json"] = bytes_of(jobj(j) + "\n");
  return files;
}

void write_city_bundle(const std::string &dir, const city_bundle_files &files) {
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec)
    throw std::runtime_error("write_city_bundle: cannot make " + dir + ": " + ec.message());
  for (const auto &[given, b] : files) {
    const std::string name = cvc::archive::safe_path(given);
    if (name.empty())
      throw std::invalid_argument("write_city_bundle: '" + given + "' is not a path in the bundle");
    const fs::path p = fs::path(dir) / fs::path(name);
    if (name.find('/') != std::string::npos) {
      fs::create_directories(p.parent_path(), ec);
      if (ec)
        throw std::runtime_error("write_city_bundle: cannot make " + p.parent_path().string() +
                                 ": " + ec.message());
    }
    std::ofstream out(p, std::ios::binary);
    if (b)
      out.write(b->data(), static_cast<std::streamsize>(b->size()));
    if (!out)
      throw std::runtime_error("write_city_bundle: cannot write " + p.string());
  }
}

city_bundle_reader city_bundle_memory_reader(const city_bundle_files &files) {
  return [files](const std::string &name) {
    auto hit = files.find(name);
    return hit == files.end() ? std::shared_ptr<const std::string>() : hit->second;
  };
}

namespace detail {

city_bundle load_city_bundle_v2(app &ctx, const city_bundle_reader &read, const std::string &label,
                                const std::string &dir, const std::string &manifest_text) {
  try {
    const ptree m = parse_json(manifest_text, "world.json");
    if (m.get<std::string>("format", "") != kFormat || m.get<int>("version", 0) != kVersion)
      throw std::runtime_error("world.json is not a cvc.city_bundle version 2 manifest");
    const surface_registry &surfaces =
        surface_registry::variant(m.get<std::string>("ontology", "merged_default"));
    // The file's surface table, by index, to this build's ids.
    std::vector<std::uint16_t> ids;
    for (const std::string &n : jlist(m, "surfaces")) {
      std::uint16_t id = 0;
      if (!surfaces.find(n, id))
        throw std::runtime_error("surface '" + n + "' is not in the " +
                                 std::string(surfaces.ontology()) + " table");
      ids.push_back(id);
    }
    auto id_of_index = [&](std::size_t i) {
      if (i >= ids.size())
        throw std::runtime_error("a surface index is outside world.json's surfaces");
      return ids[i];
    };
    auto id_of_name = [&](const std::string &n) {
      std::uint16_t id = 0;
      if (!surfaces.find(n, id))
        throw std::runtime_error("surface '" + n + "' is not in the table");
      return id;
    };

    city_bundle out;
    out.dir = dir;
    out.version = kVersion;
    out.geometry = world_geometry(surfaces);
    out.geometry.set_edge_walls(m.get<bool>("edge_walls", true));
    out.hash = m.get<std::string>("hash", "");
    if (const auto w = m.get_optional<double>("water.level"))
      out.water_level = *w;
    if (m.get_child_optional("region"))
      out.region = city_bundle_region{m.get<double>("region.min_x"), m.get<double>("region.min_y"),
                                      m.get<double>("region.max_x"), m.get<double>("region.max_y")};
    if (const auto g = m.get_child_optional("generator")) {
      out.generator.name = g->get<std::string>("name", "");
      out.generator.version = g->get<std::string>("version", "");
      out.generator.seed = g->get<std::uint64_t>("seed", 0);
      if (const auto p = g->get_child_optional("params")) {
        std::ostringstream o;
        boost::property_tree::write_json(o, *p, false);
        out.generator.params = o.str();
      }
    }

    // The terrain.
    const ptree tj = parse_json(*need(read, "terrain.json", label), "terrain.json");
    const int rows = tj.get<int>("rows"), cols = tj.get<int>("cols");
    if (rows < 2 || cols < 2)
      throw std::runtime_error("terrain.json: needs at least 2 x 2 samples");
    const std::size_t n = static_cast<std::size_t>(rows) * cols;
    std::vector<float> heights(n);
    if (const auto gf = tj.get_optional<std::string>("grid_file")) {
      const auto b = need(read, *gf, label);
      if (b->size() != n * 4)
        throw std::runtime_error("terrain.json: " + *gf + " is not rows x cols float32s");
      for (std::size_t i = 0; i < n; ++i)
        heights[i] = get_le<float>(b->data() + 4 * i);
    } else {
      std::size_t i = 0;
      if (const auto grid = tj.get_child_optional("grid"))
        for (const auto &row : *grid)
          for (const auto &v : row.second)
            if (i < n)
              heights[i++] = v.second.get_value<float>();
      if (i != n)
        throw std::runtime_error("terrain.json: the grid is not rows x cols");
    }
    heights = north_first(heights, rows, cols); // reversing the rows is its own inverse
    out.min_x = tj.get<double>("bounds.min_x");
    out.min_y = tj.get<double>("bounds.min_y");
    out.max_x = tj.get<double>("bounds.max_x");
    out.max_y = tj.get<double>("bounds.max_y");
    const std::vector<double> origin = jdoubles(tj, "origin"), spacing = jdoubles(tj, "spacing");
    std::shared_ptr<cvc::height_field> field;
    if (origin.size() == 2 && spacing.size() == 2)
      field = std::make_shared<cvc::height_field>(rows, cols, origin[0], origin[1], spacing[0],
                                                  spacing[1], std::move(heights));
    else
      field = std::make_shared<cvc::height_field>(cvc::height_field::from_bounds(
          rows, cols, out.min_x, out.min_y, out.max_x, out.max_y, std::move(heights)));

    const auto items = m.get_child_optional("items");
    if (!items)
      throw std::runtime_error("world.json has no items");
    for (const auto &kv : *items) {
      const ptree &ij = kv.second;
      const std::string name = ij.get<std::string>("name");
      geometry_item it;
      it.name = name;
      it.roles = roles_of(jlist(ij, "roles"), "item '" + name + "'");
      it.surface = id_of_name(ij.get<std::string>("surface"));
      if (ij.get<bool>("terrain", false)) {
        if (const auto f = ij.get_optional<std::string>("cell_surfaces")) {
          const auto cb = need(read, *f, label);
          const std::size_t nc = static_cast<std::size_t>(rows - 1) * (cols - 1);
          if (cb->size() != nc * 2)
            throw std::runtime_error(*f + " is not one uint16 per terrain cell");
          std::vector<cvc::face_material_t> cells(nc);
          for (std::size_t i = 0; i < nc; ++i)
            cells[i] = id_of_index(get_le<std::uint16_t>(cb->data() + 2 * i));
          field->set_cell_materials(north_first(cells, rows - 1, cols - 1));
          out.land_cover_loaded = true;
        }
        it.geometry = field;
      } else {
        const std::string f = ij.get<std::string>("file");
        const auto gb = need(read, f, label);
        model_read_options exact;
        exact.exact = true;
        const cvc::model mdl = read_model_from_memory(ctx, gb->data(), gb->size(), ".glb", exact);
        const bool face_surfaces = ij.get<bool>("face_surfaces", false);
        auto g = std::make_shared<cvc::geometry>();
        bool first = true;
        for (const cvc::model::mesh &mm : mdl.meshes) {
          cvc::geometry part = mm.geom;
          if (face_surfaces && !part.const_tris().empty()) {
            if (mm.material < 0 || mm.material >= static_cast<int>(mdl.materials.size()))
              throw std::runtime_error(f + ": a mesh has no surface material");
            part.face_materials().assign(part.const_tris().size(),
                                         id_of_name(mdl.materials[mm.material].name));
          }
          if (first)
            *g = part;
          else
            g->merge(part);
          first = false;
        }
        if (!face_surfaces)
          g->face_materials().clear();
        it.geometry = g;
        const std::vector<double> p = jdoubles(ij, "position"), q = jdoubles(ij, "rotation");
        if (p.size() != 3 || q.size() != 4)
          throw std::runtime_error("item '" + name + "': position or rotation is malformed");
        it.position = {p[0], p[1], p[2]};
        it.rotation = {q[0], q[1], q[2], q[3]};
        if (name == "buildings")
          out.buildings_loaded = true;
      }
      out.geometry.add(std::move(it));
    }
    if (!out.buildings_loaded)
      out.buildings_error = "the bundle has no buildings";
    if (!dir.empty() && fs::exists(fs::path(dir) / "satellite.png"))
      out.satellite = (fs::path(dir) / "satellite.png").string();
    return out;
  } catch (const boost::property_tree::ptree_error &e) {
    throw std::runtime_error("load_city_bundle: " + label + ": " + e.what());
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    if (what.rfind("load_city_bundle:", 0) == 0)
      throw;
    throw std::runtime_error("load_city_bundle: " + label + ": " + what);
  }
}

} // namespace detail

// ── A generated world as a bundle ────────────────────────────────────────────────────────────────

std::string generated_world_params_json(const generated_world_params &p) {
  std::vector<std::pair<std::string, std::string>> kv = {
      {"seed", juint(p.seed)},
      {"half_m", jnum(p.half_m)},
      {"trees", jint(p.trees)},
      {"rocks", jint(p.rocks)},
      {"buildings", jint(p.buildings)},
      {"tree_gen", jint(p.tree_gen)},
      {"amp_m", jnum(p.amp_m)},
      {"water_level_m", jnum(p.water_level_m)},
      {"preset", jint(p.preset)},
      {"ontology", jint(p.ontology)},
      {"cell_m", jnum(p.cell_m)},
      {"max_terrain_n", jint(p.max_terrain_n)},
      {"raster_n", jint(p.raster_n)},
      {"waterline_sliver_m", jnum(p.waterline_sliver_m)},
      {"terrain", jstr(p.terrain)},
      {"city", jstr(p.city)},
      {"terrain_file", jstr(p.terrain_file)},
      {"edge", jint(p.edge)}};
  // A theme only when there is one: a world without keeps the JSON (and its bundle's hash) it had
  // before themes.
  if (!p.theme.empty())
    kv.push_back({"theme", jstr(p.theme)});
  return jobj(kv, "  ");
}

generated_world_params generated_world_params_from_json(const std::string &text) {
  return generated_world_params_from_json(text, generated_world_params());
}

generated_world_params generated_world_params_from_json(const std::string &text,
                                                        const generated_world_params &base) {
  const ptree j = parse_json(text, "generated_world_params_from_json");
  generated_world_params p = base;
  p.seed = j.get<std::uint64_t>("seed", p.seed);
  p.half_m = j.get<double>("half_m", p.half_m);
  p.trees = j.get<int>("trees", p.trees);
  p.rocks = j.get<int>("rocks", p.rocks);
  p.buildings = j.get<int>("buildings", p.buildings);
  p.tree_gen = j.get<int>("tree_gen", p.tree_gen);
  p.amp_m = j.get<double>("amp_m", p.amp_m);
  p.water_level_m = j.get<double>("water_level_m", p.water_level_m);
  p.preset = j.get<int>("preset", p.preset);
  p.ontology = j.get<int>("ontology", p.ontology);
  p.cell_m = j.get<double>("cell_m", p.cell_m);
  p.max_terrain_n = j.get<int>("max_terrain_n", p.max_terrain_n);
  p.raster_n = j.get<int>("raster_n", p.raster_n);
  p.waterline_sliver_m = j.get<double>("waterline_sliver_m", p.waterline_sliver_m);
  p.terrain = j.get<std::string>("terrain", p.terrain);
  p.city = j.get<std::string>("city", p.city);
  p.terrain_file = j.get<std::string>("terrain_file", p.terrain_file);
  p.edge = j.get<int>("edge", p.edge);
  p.theme = j.get<std::string>("theme", p.theme);
  return p;
}

namespace {

// ── What the renderer reads beside the meshes (generated_world_bundle, world_from_bundle) ──

// props.json: the props' cells, a row of numbers each, in kPropCellFields' order.
const char *const kPropCellFields[] = {"min_x",    "min_y",        "min_z",    "max_x",
                                       "max_y",    "max_z",        "props",    "props_end",
                                       "foliage",  "foliage_end",  "needles",  "needles_end",
                                       "props_r1", "props_r1_end", "props_r2", "props_r2_end"};
constexpr std::size_t kPropCellWidth = sizeof(kPropCellFields) / sizeof(kPropCellFields[0]);

std::string props_json(const generated_world &w) {
  std::vector<std::string> fields;
  for (const char *f : kPropCellFields)
    fields.push_back(jstr(f));
  std::string o = "{\n  \"cell_m\": " + jnum(w.prop_cell_m) + ",\n  \"fields\": " + jarr(fields) +
                  ",\n  \"cells\": [";
  for (std::size_t i = 0; i < w.prop_cells.size(); ++i) {
    const generated_world::prop_cell &c = w.prop_cells[i];
    std::vector<std::string> v;
    for (int k = 0; k < 3; ++k)
      v.push_back(jnum(c.min[k]));
    for (int k = 0; k < 3; ++k)
      v.push_back(jnum(c.max[k]));
    for (const generated_world::prop_span *s :
         {&c.props, &c.foliage, &c.needles, &c.props_r1, &c.props_r2}) {
      v.push_back(juint(s->c0));
      v.push_back(juint(s->c1));
    }
    o += (i ? ",\n    " : "\n    ") + jarr(v);
  }
  return o + (w.prop_cells.empty() ? "]\n}\n" : "\n  ]\n}\n");
}

// models.json: the theme's models (generated_world::model_assets, models): each asset as its
// sidecar (model_asset_json, without its directory: a reader finds the files by name), and each
// instance as [asset, x, y, z, yaw, scale].
std::string models_json(const generated_world &w) {
  std::string assets = "[", rows = "[";
  for (std::size_t i = 0; i < w.model_assets.size(); ++i) {
    std::string a = model_asset_json(w.model_assets[i]);
    while (!a.empty() && (a.back() == '\n' || a.back() == ' '))
      a.pop_back();
    assets += (i ? ",\n" : "\n") + a;
  }
  assets += w.model_assets.empty() ? "]" : "\n  ]";
  for (std::size_t i = 0; i < w.models.size(); ++i) {
    const generated_world::model_instance &m = w.models[i];
    rows += (i ? ",\n    " : "\n    ") +
            jarr({jint(m.asset), jnum(m.x), jnum(m.y), jnum(m.z), jnum(m.yaw), jnum(m.scale)});
  }
  rows += w.models.empty() ? "]" : "\n  ]";
  return jobj({{"buildings", juint(w.model_buildings)},
               {"trees", juint(w.model_trees)},
               {"fields",
                jarr({jstr("asset"), jstr("x"), jstr("y"), jstr("z"), jstr("yaw"), jstr("scale")})},
               {"assets", assets},
               {"instances", rows}},
              "  ") +
         "\n";
}

void read_models(const std::string &text, generated_world &w) {
  const ptree j = parse_json(text, "models.json");
  w.model_buildings = j.get<std::size_t>("buildings", 0);
  w.model_trees = j.get<std::size_t>("trees", 0);
  if (const auto as = j.get_child_optional("assets"))
    for (const auto &e : *as) {
      std::ostringstream o;
      boost::property_tree::write_json(o, e.second, false);
      // write_json writes every value as a string; the sidecar reader reads them back as numbers.
      w.model_assets.push_back(parse_model_asset(o.str(), std::string(), "models.json: an asset"));
    }
  if (const auto rs = j.get_child_optional("instances"))
    for (const auto &e : *rs) {
      std::vector<double> v;
      for (const auto &c : e.second)
        v.push_back(c.second.get_value<double>());
      if (v.size() != 6)
        throw std::runtime_error("models.json: an instance is not [asset, x, y, z, yaw, scale]");
      generated_world::model_instance m;
      m.asset = int(v[0]);
      if (m.asset < 0 || m.asset >= int(w.model_assets.size()) || v[0] != double(m.asset))
        throw std::runtime_error("models.json: an instance's asset is not in assets");
      m.x = v[1];
      m.y = v[2];
      m.z = v[3];
      m.yaw = v[4];
      m.scale = v[5];
      w.models.push_back(m);
    }
}

// streets.json: the street network and the city's counts.
std::string streets_json(const generated_world &w) {
  std::string nodes = "[", segs = "[";
  for (std::size_t i = 0; i < w.streets.nodes.size(); ++i) {
    const road_node &n = w.streets.nodes[i];
    nodes += (i ? ",\n    " : "\n    ") + jarr({jnum(n.x), jnum(n.y), jint(n.degree)});
  }
  nodes += w.streets.nodes.empty() ? "]" : "\n  ]";
  for (std::size_t i = 0; i < w.streets.segments.size(); ++i) {
    const road_segment &s = w.streets.segments[i];
    segs += (i ? ",\n    " : "\n    ") + jarr({jint(s.a), jint(s.b), jnum(s.width),
                                               jint(static_cast<int>(s.kind)), jbool(s.arterial)});
  }
  segs += w.streets.segments.empty() ? "]" : "\n  ]";
  return jobj({{"half", jnum(w.streets.half)},
               {"buildings", juint(w.building_count)},
               {"archetypes", jbool(w.archetypes)},
               {"districts", juint(w.districts)},
               {"road_length", jnum(w.road_length)},
               {"fields",
                jarr({jstr("a"), jstr("b"), jstr("width"), jstr("class"), jstr("arterial")})},
               {"nodes", nodes},
               {"segments", segs}}) +
         "\n";
}

// ground.png: the painted ground, north at the top (its rows are south first).
std::string ground_png(const world_ground &g) {
  if (g.width <= 0 || g.height <= 0 ||
      g.rgb.size() != static_cast<std::size_t>(g.width) * g.height * 3)
    throw std::invalid_argument("generated_world_bundle: the ground is not width x height RGB");
  const cvc::image img(g.width, g.height, cvc::image::pixel_format::RGB, cvc::image::data_type::u8,
                       g.rgb.data());
  return encode_png(img.flipped_vertical());
}

// The ground in ground.png, else satellite.png, rows south first; empty when there is neither.
world_ground read_ground(const city_bundle_reader &read) {
  std::string name = "ground.png";
  std::shared_ptr<const std::string> png = read(name);
  if (!png) {
    name = "satellite.png";
    png = read(name);
  }
  world_ground out;
  if (!png)
    return out;
  cvc::image_decode_limits lim;
  lim.max_side = 16384;
  lim.max_pixels = 16384u * 16384u;
  lim.max_bytes = 1024u * 1024u * 1024u;
  std::string why;
  cvc::image img = cvc::decode_image(png->data(), png->size(), lim, &why);
  if (img.empty())
    throw std::runtime_error("world_from_bundle: " + name + ": " + why);
  img = img.converted(cvc::image::pixel_format::RGB, cvc::image::data_type::u8).flipped_vertical();
  out.width = img.width();
  out.height = img.height();
  out.rgb.assign(img.data(), img.data() + img.size_bytes());
  return out;
}

// The faces of a cell's run use one run of points (a mesh read back keeps its faces' order and
// its points in the order its faces first use them): those are the cell's points.
void span_points(const cvc::geometry &g, generated_world::prop_span &s, bool lines,
                 const char *what) {
  const std::size_t n = lines ? g.const_lines().size() : g.const_tris().size();
  if (s.c1 < s.c0 || s.c1 > n)
    throw std::runtime_error(std::string("props.json: a cell's ") + what +
                             " run is outside its mesh");
  s.p0 = s.p1 = 0;
  if (s.c0 == s.c1)
    return;
  std::size_t lo = std::numeric_limits<std::size_t>::max(), hi = 0;
  for (std::size_t f = s.c0; f < s.c1; ++f)
    for (int k = 0; k < (lines ? 2 : 3); ++k) {
      const std::size_t v = lines ? g.const_lines()[f][k] : g.const_tris()[f][k];
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
  s.p0 = lo;
  s.p1 = hi + 1;
}

// A JSON array of numbers' values as text, each child's.
std::vector<std::string> row_of(const ptree &row) {
  std::vector<std::string> v;
  for (const auto &x : row)
    v.push_back(x.second.get_value<std::string>());
  return v;
}

void read_props(const std::string &text, generated_world &w) {
  const ptree j = parse_json(text, "props.json");
  w.prop_cell_m = j.get<double>("cell_m", 0.0);
  if (const auto cells = j.get_child_optional("cells"))
    for (const auto &kv : *cells) {
      const std::vector<std::string> v = row_of(kv.second);
      if (v.size() != kPropCellWidth)
        throw std::runtime_error("props.json: a cell is not " + std::to_string(kPropCellWidth) +
                                 " numbers");
      generated_world::prop_cell c;
      for (int k = 0; k < 3; ++k) {
        c.min[k] = std::stod(v[std::size_t(k)]);
        c.max[k] = std::stod(v[std::size_t(k) + 3]);
      }
      std::size_t at = 6;
      for (generated_world::prop_span *s :
           {&c.props, &c.foliage, &c.needles, &c.props_r1, &c.props_r2}) {
        s->c0 = static_cast<std::size_t>(std::stoull(v[at++]));
        s->c1 = static_cast<std::size_t>(std::stoull(v[at++]));
      }
      span_points(w.props, c.props, false, "props");
      span_points(w.foliage, c.foliage, false, "foliage");
      span_points(w.needles, c.needles, true, "needles");
      span_points(w.props_r1, c.props_r1, false, "props_r1");
      span_points(w.props_r2, c.props_r2, false, "props_r2");
      w.prop_cells.push_back(c);
    }
}

void read_streets(const std::string &text, generated_world &w) {
  const ptree j = parse_json(text, "streets.json");
  road_network net;
  net.half = j.get<double>("half", net.half);
  if (const auto nodes = j.get_child_optional("nodes"))
    for (const auto &kv : *nodes) {
      const std::vector<std::string> v = row_of(kv.second);
      if (v.size() != 3)
        throw std::runtime_error("streets.json: a node is not [x, y, degree]");
      road_node n;
      n.x = std::stod(v[0]);
      n.y = std::stod(v[1]);
      n.degree = std::stoi(v[2]);
      net.nodes.push_back(n);
    }
  if (const auto segs = j.get_child_optional("segments"))
    for (const auto &kv : *segs) {
      const std::vector<std::string> v = row_of(kv.second);
      if (v.size() != 5)
        throw std::runtime_error("streets.json: a segment is not [a, b, width, class, arterial]");
      road_segment s;
      s.a = std::stoi(v[0]);
      s.b = std::stoi(v[1]);
      if (s.a < 0 || s.b < 0 || std::size_t(s.a) >= net.nodes.size() ||
          std::size_t(s.b) >= net.nodes.size())
        throw std::runtime_error("streets.json: a segment's node is not in nodes");
      s.width = std::stod(v[2]);
      const int kind = std::stoi(v[3]);
      if (kind < 0 || kind > 2)
        throw std::runtime_error("streets.json: a segment's class is not 0, 1 or 2");
      s.kind = static_cast<road_class>(kind);
      s.arterial = v[4] == "true";
      net.segments.push_back(s);
    }
  w.streets = std::move(net);
  w.building_count = j.get<std::size_t>("buildings", 0);
  w.archetypes = j.get<bool>("archetypes", false);
  w.districts = j.get<std::size_t>("districts", 0);
  w.road_length = j.get<double>("road_length", 0.0);
}

// An item's mesh in world coordinates: its placement (a unit quaternion, then a move) applied.
cvc::geometry placed_mesh(const geometry_item &it) {
  const auto &q = it.rotation;
  const auto &t = it.position;
  if (q[0] == 1.0 && q[1] == 0.0 && q[2] == 0.0 && q[3] == 0.0 && t[0] == 0.0 && t[1] == 0.0 &&
      t[2] == 0.0)
    return *it.geometry;
  auto rot = [&q](double x, double y, double z) {
    // v' = v + 2 w (u x v) + 2 u x (u x v), u the quaternion's vector part
    const double cx = q[2] * z - q[3] * y, cy = q[3] * x - q[1] * z, cz = q[1] * y - q[2] * x;
    const double ccx = q[2] * cz - q[3] * cy, ccy = q[3] * cx - q[1] * cz,
                 ccz = q[1] * cy - q[2] * cx;
    return std::array<double, 3>{x + 2.0 * (q[0] * cx + ccx), y + 2.0 * (q[0] * cy + ccy),
                                 z + 2.0 * (q[0] * cz + ccz)};
  };
  cvc::geometry g = *it.geometry;
  for (auto &p : g.points()) {
    const auto r = rot(p[0], p[1], p[2]);
    p[0] = r[0] + t[0];
    p[1] = r[1] + t[1];
    p[2] = r[2] + t[2];
  }
  for (auto &n : g.normals()) {
    const auto r = rot(n[0], n[1], n[2]);
    n[0] = r[0];
    n[1] = r[1];
    n[2] = r[2];
  }
  return g;
}

} // namespace

city_bundle_files generated_world_bundle(app &ctx, const generated_world &w,
                                         const std::string &generator_version,
                                         const world_ground *ground) {
  world_geometry g = w.geometry();
  auto add_render = [&g](const char *name, const cvc::geometry &m) {
    if (m.const_points().empty())
      return;
    geometry_item it;
    it.name = name;
    it.geometry = std::make_shared<const cvc::geometry>(m);
    it.roles = role_render;
    g.add(std::move(it));
  };
  add_render("needles", w.needles);
  add_render("props_r1", w.props_r1);
  add_render("props_r2", w.props_r2);
  city_bundle_write info;
  info.generator.name = "generated_world";
  info.generator.version = generator_version;
  info.generator.seed = w.params.seed;
  info.generator.params = generated_world_params_json(w.params);
  info.water_level = w.params.water_level_m;
  if (!w.prop_cells.empty())
    info.extra["props.json"] = bytes_of(props_json(w));
  info.extra["streets.json"] = bytes_of(streets_json(w));
  if (!w.models.empty())
    info.extra["models.json"] = bytes_of(models_json(w));
  world_ground painted;
  if ((!ground || ground->empty()) && !w.raster.klass.empty()) {
    painted.width = painted.height = world_ground_size;
    painted.rgb = paint_surface(w, surface_color::material, world_ground_size);
    ground = &painted;
  }
  if (ground && !ground->empty())
    info.extra["ground.png"] = bytes_of(ground_png(*ground));
  return city_bundle_bytes(ctx, g, info);
}

bundle_world world_from_bundle(app &ctx, const city_bundle &b, const city_bundle_reader &read) {
  (void)ctx;
  const std::shared_ptr<const cvc::height_field> t0 = b.geometry.terrain();
  if (!t0)
    throw std::runtime_error("world_from_bundle: the bundle has no terrain");
  auto w = std::make_shared<generated_world>();
  w->surfaces = &b.geometry.surfaces();
  w->terrain = std::make_shared<cvc::height_field>(*t0);
  const cvc::height_field &t = *w->terrain;

  // The params: the generator's, else a world over the terrain's extent.
  const bool generated = b.generator.name == "generated_world";
  if (generated) {
    w->params = generated_world_params_from_json(b.generator.params);
  } else {
    w->params.half_m = 0.5 * std::max(t.max_x() - t.x0(), t.max_y() - t.y0());
    w->params.edge = int(b.geometry.edge_walls() ? world_edge::wall : world_edge::none);
    w->params.trees = w->params.rocks = w->params.buildings = 0;
    w->params.terrain = w->params.city = "bundle";
  }
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (const float z : t.heights())
    if (std::isfinite(z)) {
      lo = std::min(lo, double(z));
      hi = std::max(hi, double(z));
    }
  if (!std::isfinite(lo))
    lo = hi = 0.0;
  w->height_min = lo;
  w->height_max = hi;
  const bool wet = b.water_level.has_value() || generated;
  if (b.water_level)
    w->params.water_level_m = *b.water_level;
  else if (!generated)
    w->params.water_level_m = lo - 1000.0; // under all ground: no water
  w->grid.rows = t.rows();
  w->grid.cols = t.cols();
  w->grid.min_x = t.x0();
  w->grid.min_y = t.y0();
  w->grid.max_x = t.max_x();
  w->grid.max_y = t.max_y();

  // The meshes, by name; any other mesh item draws with the buildings.
  for (const geometry_item &it : b.geometry.items()) {
    if (it.height_field() || !it.geometry || !it.has(role_render)) // a collider alone is not drawn
      continue;
    cvc::geometry *to = it.name == "buildings"  ? &w->buildings
                        : it.name == "roads"    ? &w->roads
                        : it.name == "props"    ? &w->props
                        : it.name == "foliage"  ? &w->foliage
                        : it.name == "needles"  ? &w->needles
                        : it.name == "props_r1" ? &w->props_r1
                        : it.name == "props_r2" ? &w->props_r2
                                                : &w->buildings;
    cvc::geometry m = placed_mesh(it);
    if (to->const_points().empty())
      *to = std::move(m);
    else
      to->merge(m);
  }

  try {
    if (const auto s = read("streets.json"))
      read_streets(*s, *w);
    if (const auto p = read("props.json"))
      read_props(*p, *w);
    if (const auto m = read("models.json"))
      read_models(*m, *w);
  } catch (const boost::property_tree::ptree_error &e) {
    throw std::runtime_error(std::string("world_from_bundle: ") + e.what());
  } catch (const std::logic_error &e) { // a number that does not parse (stoi, stod)
    throw std::runtime_error(std::string("world_from_bundle: a number in streets.json or "
                                         "props.json: ") +
                             e.what());
  } catch (const std::runtime_error &e) {
    throw std::runtime_error(std::string("world_from_bundle: ") + e.what());
  }

  // Rebuilt from the stored terrain: the water, the ground under its columns, the render LOD.
  const double level = w->params.water_level_m;
  if (wet) {
    w->water = terrain_water_bodies(t, level);
    const int n = water_resolution(*w);
    const double half = w->params.half_m;
    w->water_ground.assign(std::size_t(n) * n, 0.0f);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        w->water_ground[std::size_t(j) * n + i] = static_cast<float>(
            t.height(-half + 2.0 * half * i / (n - 1), -half + 2.0 * half * j / (n - 1)));
  }
  w->terrain_layout = plan_terrain_chunks(t);
  const terrain_chunk_layout &L = w->terrain_layout;
  const int chunks = L.chunk_rows * L.chunk_cols;
  w->terrain_chunks.reserve(std::size_t(std::max(0, chunks)));
  for (int i = 0; i < chunks; ++i) {
    terrain_chunk ch = make_terrain_chunk(t, L, i / L.chunk_cols, i % L.chunk_cols);
    if (wet)
      ch.water = make_water_rungs(t, L, ch, level);
    w->terrain_chunks.push_back(std::move(ch));
  }

  bundle_world out;
  out.world = std::move(w);
  out.ground = read_ground(read);
  return out;
}

} // namespace world
} // namespace cvc

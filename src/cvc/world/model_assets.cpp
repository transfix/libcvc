/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// model_assets.cpp — see model_assets.h.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/process.h>
#include <cvc/world/model_assets.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace world {
namespace {

using boost::property_tree::ptree;
namespace fs = std::filesystem;

const char *const kKinds[] = {"building", "tree", "prop", "vehicle"};

std::string num(double v) {
  char b[64];
  std::snprintf(b, sizeof b, "%.10g", v);
  return b;
}
std::string quoted(const std::string &s) {
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\')
      o += '\\';
    if (static_cast<unsigned char>(c) < 0x20) {
      char b[8];
      std::snprintf(b, sizeof b, "\\u%04x", static_cast<unsigned char>(c));
      o += b;
      continue;
    }
    o += c;
  }
  return o + "\"";
}

std::vector<double> triple(const ptree &pt, const std::string &key, const std::string &what) {
  std::vector<double> v;
  if (const auto c = pt.get_child_optional(key))
    for (const auto &e : *c)
      v.push_back(e.second.get_value<double>());
  if (v.size() != 3)
    throw std::runtime_error(what + ": " + key + " is not three numbers");
  for (double x : v)
    if (!std::isfinite(x))
      throw std::runtime_error(what + ": " + key + " is not finite");
  return v;
}

} // namespace

const char *model_kind_name(model_kind k) {
  const int i = int(k);
  return (i >= 0 && i < 4) ? kKinds[i] : kKinds[2];
}

bool model_kind_from_name(const std::string &n, model_kind &out) {
  for (int i = 0; i < 4; ++i)
    if (n == kKinds[i]) {
      out = model_kind(i);
      return true;
    }
  return false;
}

bool model_asset::has_tag(const std::string &t) const {
  return std::find(tags.begin(), tags.end(), t) != tags.end();
}

std::string model_asset::path(int lod) const {
  if (lods.empty())
    return std::string();
  const int k = std::clamp(lod, 0, int(lods.size()) - 1);
  return dir.empty() ? lods[k].file : (fs::path(dir) / lods[k].file).string();
}

model_asset parse_model_asset(const std::string &json_text, const std::string &dir,
                              const std::string &what) {
  ptree pt;
  try {
    std::istringstream in(json_text);
    boost::property_tree::read_json(in, pt);
  } catch (const std::exception &e) {
    throw std::runtime_error(what + " is not JSON: " + e.what());
  }
  const std::string schema = pt.get<std::string>("schema", "");
  if (schema != "cvc-model-asset/1")
    throw std::runtime_error(what + ": schema '" + schema + "', not cvc-model-asset/1");
  model_asset a;
  a.dir = dir;
  a.name = pt.get<std::string>("name", "");
  if (a.name.empty())
    throw std::runtime_error(what + ": no name");
  const std::string kind = pt.get<std::string>("kind", "prop");
  if (!model_kind_from_name(kind, a.kind))
    throw std::runtime_error(what + " '" + a.name + "': unknown kind '" + kind + "'");
  if (const auto t = pt.get_child_optional("tags"))
    for (const auto &e : *t)
      a.tags.push_back(e.second.get_value<std::string>());
  const std::string up = pt.get<std::string>("up", "+y"),
                    front = pt.get<std::string>("front", "+z");
  if (up != "+y" || front != "+z")
    throw std::runtime_error(what + " '" + a.name + "': only up +y, front +z (glTF) is read");
  const std::vector<double> lo = triple(pt, "bounds.min", what),
                            hi = triple(pt, "bounds.max", what);
  for (int i = 0; i < 3; ++i)
    if (!(hi[i] > lo[i]))
      throw std::runtime_error(what + " '" + a.name + "': bounds are empty");
  // glTF (x, y, z) -> world (-x, z, y).
  a.half_x = 0.5 * (hi[0] - lo[0]);
  a.half_y = 0.5 * (hi[2] - lo[2]);
  a.height = hi[1] - lo[1];
  a.base_z = lo[1];
  a.centre_x = -0.5 * (lo[0] + hi[0]);
  a.centre_y = 0.5 * (lo[2] + hi[2]);
  double last = 0.0;
  if (const auto L = pt.get_child_optional("lods"))
    for (const auto &e : *L) {
      model_lod l;
      l.file = e.second.get<std::string>("file", "");
      l.triangles = e.second.get<std::uint64_t>("triangles", 0);
      l.world_error_m = e.second.get<double>("world_error_m", 0.0);
      l.texture_px = e.second.get_optional<int>("texture_px").value_or(0);
      if (l.file.empty())
        throw std::runtime_error(what + " '" + a.name + "': a LOD with no file");
      if (!std::isfinite(l.world_error_m) || l.world_error_m < last)
        throw std::runtime_error(what + " '" + a.name +
                                 "': LOD errors must not be negative or decrease");
      last = l.world_error_m;
      a.lods.push_back(l);
    }
  if (a.lods.empty())
    throw std::runtime_error(what + " '" + a.name + "': no LODs");
  return a;
}

model_asset read_model_asset(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("model asset " + path + ": cannot read it");
  std::ostringstream s;
  s << in.rdbuf();
  return parse_model_asset(s.str(), fs::path(path).parent_path().string(), "model asset " + path);
}

std::string model_asset_json(const model_asset &a) {
  // world (x, y, z) -> glTF (-x, z, y): the bounds back in the model frame.
  const double lo[3] = {-(a.centre_x + a.half_x), a.base_z, a.centre_y - a.half_y};
  const double hi[3] = {-(a.centre_x - a.half_x), a.base_z + a.height, a.centre_y + a.half_y};
  std::ostringstream o;
  o << "{\n  \"schema\": \"cvc-model-asset/1\",\n  \"name\": " << quoted(a.name)
    << ",\n  \"kind\": " << quoted(model_kind_name(a.kind)) << ",\n  \"tags\": [";
  for (std::size_t i = 0; i < a.tags.size(); ++i)
    o << (i ? ", " : "") << quoted(a.tags[i]);
  o << "],\n  \"units\": \"m\",\n  \"up\": \"+y\",\n  \"front\": \"+z\",\n"
    << "  \"origin\": \"bottom-centre\",\n  \"bounds\": {\"min\": [" << num(lo[0]) << ", "
    << num(lo[1]) << ", " << num(lo[2]) << "], \"max\": [" << num(hi[0]) << ", " << num(hi[1])
    << ", " << num(hi[2]) << "]},\n  \"lods\": [";
  for (std::size_t i = 0; i < a.lods.size(); ++i) {
    const model_lod &l = a.lods[i];
    o << (i ? ",\n    " : "\n    ") << "{\"file\": " << quoted(l.file)
      << ", \"triangles\": " << l.triangles << ", \"world_error_m\": " << num(l.world_error_m);
    if (l.texture_px > 0)
      o << ", \"texture_px\": " << l.texture_px;
    o << "}";
  }
  o << "\n  ]\n}\n";
  return o.str();
}

void model_catalog::add(const model_asset &a) {
  std::lock_guard<std::mutex> g(m_);
  assets_[a.name] = a;
}

std::size_t model_catalog::scan(const std::string &dir) {
  std::error_code ec;
  std::vector<fs::path> files;
  for (const fs::directory_entry &e : fs::directory_iterator(dir, ec)) {
    const std::string f = e.path().filename().string();
    const std::string suffix = ".asset.json";
    if (e.is_regular_file(ec) && f.size() > suffix.size() &&
        f.compare(f.size() - suffix.size(), suffix.size(), suffix) == 0)
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end()); // the same order on every platform
  std::size_t n = 0;
  for (const fs::path &p : files) {
    try {
      model_asset a = read_model_asset(p.string());
      add(a);
      ++n;
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> g(m_);
      errors_.push_back(e.what());
    }
  }
  return n;
}

bool model_catalog::find(const std::string &name, model_asset &out) const {
  std::lock_guard<std::mutex> g(m_);
  const auto it = assets_.find(name);
  if (it == assets_.end())
    return false;
  out = it->second;
  return true;
}

bool model_catalog::contains(const std::string &name) const {
  std::lock_guard<std::mutex> g(m_);
  return assets_.count(name) != 0;
}

std::vector<std::string> model_catalog::names() const {
  std::lock_guard<std::mutex> g(m_);
  std::vector<std::string> v;
  for (const auto &kv : assets_)
    v.push_back(kv.first);
  return v;
}

std::vector<std::string> model_catalog::errors() const {
  std::lock_guard<std::mutex> g(m_);
  return errors_;
}

std::size_t model_catalog::size() const {
  std::lock_guard<std::mutex> g(m_);
  return assets_.size();
}

void model_catalog::clear() {
  std::lock_guard<std::mutex> g(m_);
  assets_.clear();
  errors_.clear();
}

std::vector<std::string> model_search_dirs(cvc::app &) {
  std::error_code ec;
  std::vector<std::string> out;
  auto push = [&](const fs::path &p) {
    if (!p.empty() && fs::is_directory(p, ec)) {
      const std::string s = fs::weakly_canonical(p, ec).string();
      if (std::find(out.begin(), out.end(), s) == out.end())
        out.push_back(s);
    }
  };
  // Lowest priority first: the catalog's later scans replace what earlier ones found.
  std::vector<fs::path> prefixes;
  const fs::path self = cvc::process::self_path();
  if (!self.empty())
    prefixes.push_back(self.parent_path().parent_path());
  for (const char *v : {"CONDA_PREFIX", "CVCPKG_PREFIX", "CVC_PREFIX"})
    if (const char *e = std::getenv(v); e && *e)
      prefixes.emplace_back(e);
  for (const fs::path &p : prefixes)
    push(p / "share" / "cvc-scenes" / "shared");
  if (const char *b = std::getenv("CVC_NAV_BUNDLE"); b && *b)
    push(fs::path(b).parent_path() / "shared");
  if (const char *m = std::getenv("CVC_MODEL_PATH"); m && *m) {
#ifdef _WIN32
    const char sep = ';';
#else
    const char sep = ':';
#endif
    std::string s(m);
    std::vector<fs::path> dirs;
    for (std::size_t a = 0; a <= s.size();) {
      const std::size_t b = std::min(s.find(sep, a), s.size());
      if (b > a)
        dirs.emplace_back(s.substr(a, b - a));
      a = b + 1;
    }
    for (auto it = dirs.rbegin(); it != dirs.rend(); ++it) // the first listed wins
      push(*it);
  }
  return out;
}

namespace {
struct catalog_slot {
  static constexpr const char *app_slot = "cvc.world.model_catalog";
  model_catalog catalog;
  std::once_flag scanned;
};
} // namespace

model_catalog &model_catalog_of(cvc::app &app) {
  catalog_slot &s = app.slot<catalog_slot>();
  std::call_once(s.scanned, [&] {
    for (const std::string &d : model_search_dirs(app))
      s.catalog.scan(d);
  });
  return s.catalog;
}

} // namespace world
} // namespace cvc

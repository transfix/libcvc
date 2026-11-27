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

// world_theme.cpp — see world_theme.h. GL-free; every draw is a hash of the spot (deterministic).

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/lsys/rng.h>
#include <cvc/world/model_assets.h>
#include <cvc/world/world_theme.h>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace world {
namespace {

using boost::property_tree::ptree;
constexpr double kInf = std::numeric_limits<double>::infinity();

// The draws: d indices within lsys::stream::theme for a spot's element.
enum : std::uint32_t {
  kBuildShare = 0,
  kBuildPick = 1,
  kBuildScale = 2,
  kTreeShare = 10,
  kTreePick = 11,
  kTreeScale = 12,
  kTreeYaw = 13
};

std::uint64_t spot(double x, double y) {
  return lsys::cell_id(std::int32_t(std::lround(x)), std::int32_t(std::lround(y)));
}

// A weighted pick of one of `idx` (weights w[i]) with u in [0, 1); -1 when all weigh nothing.
template <class Rule>
int weighted(const std::vector<Rule> &rules, const std::vector<int> &idx, double u) {
  double total = 0.0;
  for (int i : idx)
    total += rules[i].weight;
  if (!(total > 0.0))
    return -1;
  double acc = 0.0;
  for (int i : idx) {
    acc += rules[i].weight / total;
    if (u < acc)
      return i;
  }
  return idx.back();
}

std::vector<world_theme> makeBuiltins() {
  auto bld = [](const char *asset, double w, double lo, double hi, int span = 1) {
    building_archetype_rule r;
    r.asset = asset;
    r.weight = w;
    r.min_urbanity = lo;
    r.max_urbanity = hi;
    r.span = span;
    return r;
  };
  auto tree = [](const char *asset, double w, double smin, double smax, biome_rule b) {
    tree_archetype_rule r;
    r.asset = asset;
    r.weight = w;
    r.min_scale = smin;
    r.max_scale = smax;
    r.biome = b;
    return r;
  };
  // The town's streets: corner shops out from the cores, brownstones toward them, the office
  // block (two lots wide) in them.
  const std::vector<building_archetype_rule> town = {bld("BrickCornerShop", 1.0, 0.0, 0.8),
                                                     bld("Brownstone", 1.2, 0.25, 1.0),
                                                     bld("ModernOffice", 0.8, 0.6, 1.0, 2)};
  biome_rule inland;
  inland.min_height_m = 1.5; // never on the beach
  inland.min_water_m = 25.0;
  inland.max_slope_deg = 30.0;
  biome_rule inlandFar = inland;
  inlandFar.min_water_m = 50.0;
  biome_rule coast;
  coast.min_height_m = 0.3;
  coast.max_height_m = 10.0;
  coast.max_water_m = 60.0;
  coast.max_slope_deg = 25.0;
  biome_rule warm;
  warm.min_height_m = 0.3;
  warm.max_slope_deg = 25.0;
  biome_rule hills;
  hills.min_height_m = 20.0;
  hills.max_slope_deg = 30.0;

  std::vector<world_theme> v;
  world_theme t;
  t.name = "temperate_town";
  t.description = "Brick shops, brownstones and offices; oaks inland, none on the beach.";
  t.buildings = town;
  t.trees = {tree("OakTree", 1.0, 0.75, 1.2, inland), tree("YoungOakTree", 0.35, 0.6, 1.0, inland)};
  v.push_back(t);

  t = world_theme();
  t.name = "coastal_town";
  t.description = "The town's streets; palms along the coast, oaks inland.";
  t.buildings = town;
  t.trees = {tree("PalmTree", 1.0, 0.7, 1.25, coast), tree("OakTree", 1.0, 0.75, 1.2, inlandFar),
             tree("YoungOakTree", 0.3, 0.6, 1.0, inlandFar)};
  v.push_back(t);

  t = world_theme();
  t.name = "tropical";
  t.description = "Palms everywhere but steep ground, oaks on the hills; a low-rise town.";
  t.buildings = {bld("BrickCornerShop", 1.0, 0.0, 1.0), bld("Brownstone", 0.8, 0.3, 1.0),
                 bld("ModernOffice", 0.5, 0.7, 1.0, 2)};
  t.tree_share = 0.75;
  t.trees = {tree("PalmTree", 1.0, 0.7, 1.3, warm), tree("OakTree", 0.15, 0.75, 1.1, hills)};
  v.push_back(t);
  return v;
}

struct theme_registry {
  static constexpr const char *app_slot = "cvc.world.themes";
  std::mutex m;
  std::vector<world_theme> themes = builtin_world_themes();
};

std::string num(double v) {
  if (!std::isfinite(v))
    return v > 0 ? "1e308" : "-1e308";
  char b[64];
  std::snprintf(b, sizeof b, "%.10g", v);
  return b;
}
std::string quoted(const std::string &s) {
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\')
      o += '\\';
    o += c;
  }
  return o + "\"";
}

// A number from a JSON value; +-1e308 and beyond read as +-infinity.
double jnum(const ptree &pt, const std::string &key, double dflt) {
  const double v = pt.get<double>(key, dflt);
  if (v >= 1e308)
    return kInf;
  if (v <= -1e308)
    return -kInf;
  return v;
}

void check(bool ok, const std::string &what, const std::string &msg) {
  if (!ok)
    throw std::runtime_error(what + ": " + msg);
}

} // namespace

const std::vector<world_theme> &builtin_world_themes() {
  static const std::vector<world_theme> v = makeBuiltins();
  return v;
}

void register_world_theme(cvc::app &app, const world_theme &t) {
  theme_registry &r = app.slot<theme_registry>();
  std::lock_guard<std::mutex> g(r.m);
  for (world_theme &e : r.themes)
    if (e.name == t.name) {
      e = t;
      return;
    }
  r.themes.push_back(t);
}

std::vector<std::string> world_theme_names(cvc::app &app) {
  theme_registry &r = app.slot<theme_registry>();
  std::lock_guard<std::mutex> g(r.m);
  std::vector<std::string> v;
  for (const world_theme &e : r.themes)
    v.push_back(e.name);
  return v;
}

bool find_world_theme(cvc::app &app, const std::string &name, world_theme &out) {
  theme_registry &r = app.slot<theme_registry>();
  std::lock_guard<std::mutex> g(r.m);
  for (const world_theme &e : r.themes)
    if (e.name == name) {
      out = e;
      return true;
    }
  return false;
}

std::string world_theme_json(const world_theme &t) {
  std::ostringstream o;
  o << "{\n  \"name\": " << quoted(t.name) << ",\n  \"description\": " << quoted(t.description)
    << ",\n  \"building_share\": " << num(t.building_share) << ",\n  \"buildings\": [";
  for (std::size_t i = 0; i < t.buildings.size(); ++i) {
    const building_archetype_rule &b = t.buildings[i];
    o << (i ? "," : "") << "\n    {\"asset\": " << quoted(b.asset)
      << ", \"weight\": " << num(b.weight) << ", \"min_urbanity\": " << num(b.min_urbanity)
      << ", \"max_urbanity\": " << num(b.max_urbanity) << ", \"min_scale\": " << num(b.min_scale)
      << ", \"max_scale\": " << num(b.max_scale) << ", \"span\": " << b.span << "}";
  }
  o << "\n  ],\n  \"tree_share\": " << num(t.tree_share) << ",\n  \"trees\": [";
  for (std::size_t i = 0; i < t.trees.size(); ++i) {
    const tree_archetype_rule &r = t.trees[i];
    const biome_rule &b = r.biome;
    o << (i ? "," : "") << "\n    {\"asset\": " << quoted(r.asset)
      << ", \"weight\": " << num(r.weight) << ", \"min_scale\": " << num(r.min_scale)
      << ", \"max_scale\": " << num(r.max_scale)
      << ", \"biome\": {\"min_height_m\": " << num(b.min_height_m)
      << ", \"max_height_m\": " << num(b.max_height_m)
      << ", \"max_slope_deg\": " << num(b.max_slope_deg)
      << ", \"min_water_m\": " << num(b.min_water_m) << ", \"max_water_m\": " << num(b.max_water_m)
      << "}}";
  }
  o << "\n  ]\n}\n";
  return o.str();
}

world_theme world_theme_from_json(const std::string &text, const std::string &what) {
  ptree pt;
  try {
    std::istringstream in(text);
    boost::property_tree::read_json(in, pt);
  } catch (const std::exception &e) {
    throw std::runtime_error(what + " is not JSON: " + e.what());
  }
  world_theme t;
  t.name = pt.get<std::string>("name", "");
  t.description = pt.get<std::string>("description", "");
  t.building_share = jnum(pt, "building_share", t.building_share);
  t.tree_share = jnum(pt, "tree_share", t.tree_share);
  check(t.building_share >= 0.0 && t.building_share <= 1.0, what, "building_share not in [0, 1]");
  check(t.tree_share >= 0.0 && t.tree_share <= 1.0, what, "tree_share not in [0, 1]");
  if (const auto bs = pt.get_child_optional("buildings"))
    for (const auto &e : *bs) {
      const ptree &j = e.second;
      building_archetype_rule b;
      b.asset = j.get<std::string>("asset", "");
      b.weight = jnum(j, "weight", b.weight);
      b.min_urbanity = jnum(j, "min_urbanity", b.min_urbanity);
      b.max_urbanity = jnum(j, "max_urbanity", b.max_urbanity);
      b.min_scale = jnum(j, "min_scale", b.min_scale);
      b.max_scale = jnum(j, "max_scale", b.max_scale);
      b.span = j.get<int>("span", b.span);
      check(!b.asset.empty(), what, "a building archetype with no asset");
      check(b.weight >= 0.0, what, "building '" + b.asset + "': a negative weight");
      check(b.min_urbanity <= b.max_urbanity, what, "building '" + b.asset + "': urbanity band");
      check(b.min_scale > 0.0 && b.min_scale <= b.max_scale, what,
            "building '" + b.asset + "': scale band");
      check(b.span >= 1, what, "building '" + b.asset + "': span under 1");
      t.buildings.push_back(b);
    }
  if (const auto ts = pt.get_child_optional("trees"))
    for (const auto &e : *ts) {
      const ptree &j = e.second;
      tree_archetype_rule r;
      r.asset = j.get<std::string>("asset", "");
      r.weight = jnum(j, "weight", r.weight);
      r.min_scale = jnum(j, "min_scale", r.min_scale);
      r.max_scale = jnum(j, "max_scale", r.max_scale);
      biome_rule &b = r.biome;
      b.min_height_m = jnum(j, "biome.min_height_m", b.min_height_m);
      b.max_height_m = jnum(j, "biome.max_height_m", b.max_height_m);
      b.max_slope_deg = jnum(j, "biome.max_slope_deg", b.max_slope_deg);
      b.min_water_m = jnum(j, "biome.min_water_m", b.min_water_m);
      b.max_water_m = jnum(j, "biome.max_water_m", b.max_water_m);
      check(!r.asset.empty(), what, "a tree archetype with no asset");
      check(r.weight >= 0.0, what, "tree '" + r.asset + "': a negative weight");
      check(r.min_scale > 0.0 && r.min_scale <= r.max_scale, what,
            "tree '" + r.asset + "': scale band");
      check(b.min_height_m <= b.max_height_m && b.min_water_m <= b.max_water_m, what,
            "tree '" + r.asset + "': an inverted biome band");
      t.trees.push_back(r);
    }
  return t;
}

building_pick pick_building(const world_theme &theme, const model_catalog &catalog,
                            const lot_view &lot, std::uint64_t seed, double margin) {
  building_pick out;
  const std::uint64_t el = spot(lot.x, lot.y);
  if (theme.buildings.empty() ||
      lsys::uni(seed, lsys::stream::theme, el, kBuildShare) >= theme.building_share)
    return out;
  std::vector<int> fits;
  std::vector<double> top; // the largest scale each fitting archetype takes
  for (int i = 0; i < int(theme.buildings.size()); ++i) {
    const building_archetype_rule &r = theme.buildings[i];
    model_asset a;
    if (r.weight <= 0.0 || r.span < lot.lots || lot.urbanity < r.min_urbanity ||
        lot.urbanity > r.max_urbanity || !catalog.find(r.asset, a) || a.half_x <= 0.0 ||
        a.half_y <= 0.0)
      continue;
    const double s = std::min({r.max_scale, (0.5 * lot.frontage - margin) / a.half_x,
                               (lot.depth - 2.0 * margin) / (2.0 * a.half_y)});
    if (s >= r.min_scale) {
      fits.push_back(i);
      top.push_back(s);
    }
  }
  if (fits.empty())
    return out;
  const int k =
      weighted(theme.buildings, fits, lsys::uni(seed, lsys::stream::theme, el, kBuildPick));
  if (k < 0)
    return out;
  const double hi = top[std::find(fits.begin(), fits.end(), k) - fits.begin()];
  out.archetype = k;
  out.scale =
      lsys::uni(seed, lsys::stream::theme, el, kBuildScale, theme.buildings[k].min_scale, hi);
  return out;
}

bool biome_holds(const biome_rule &b, const tree_site &s) {
  return s.height_m >= b.min_height_m && s.height_m <= b.max_height_m &&
         s.slope_deg <= b.max_slope_deg && s.water_m >= b.min_water_m && s.water_m <= b.max_water_m;
}

tree_pick pick_tree(const world_theme &theme, const model_catalog &catalog, const tree_site &site,
                    std::uint64_t seed) {
  tree_pick out;
  const std::uint64_t el = spot(site.x, site.y);
  if (theme.trees.empty() ||
      lsys::uni(seed, lsys::stream::theme, el, kTreeShare) >= theme.tree_share)
    return out;
  std::vector<int> ok;
  for (int i = 0; i < int(theme.trees.size()); ++i) {
    const tree_archetype_rule &r = theme.trees[i];
    if (r.weight > 0.0 && biome_holds(r.biome, site) && catalog.contains(r.asset))
      ok.push_back(i);
  }
  if (ok.empty())
    return out;
  const int k = weighted(theme.trees, ok, lsys::uni(seed, lsys::stream::theme, el, kTreePick));
  if (k < 0)
    return out;
  out.archetype = k;
  out.scale = lsys::uni(seed, lsys::stream::theme, el, kTreeScale, theme.trees[k].min_scale,
                        theme.trees[k].max_scale);
  out.yaw = lsys::uni(seed, lsys::stream::theme, el, kTreeYaw, 0.0, 6.283185307179586);
  return out;
}

} // namespace world
} // namespace cvc

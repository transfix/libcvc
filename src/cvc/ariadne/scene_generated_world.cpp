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

// scene_generated_world.cpp — a generated_world node in core (see scene_generated_world.h).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <map>
#include <mutex>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace world = cvc::world;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// The building library at `path`, loaded once per path (the Austin bundle is thousands of meshes).
// Empty when the path is empty or holds none.
const world::building_library &library_at(cvc::app &app, const std::string &path) {
  static std::mutex m;
  static std::map<std::string, std::shared_ptr<const world::building_library>> cache;
  static const world::building_library none;
  if (path.empty())
    return none;
  std::lock_guard<std::mutex> lock(m);
  auto it = cache.find(path);
  if (it == cache.end())
    it = cache
             .emplace(path, std::make_shared<const world::building_library>(
                                world::load_building_library(app, path)))
             .first;
  return *it->second;
}

std::string joined(const std::vector<std::string> &v) {
  std::string s;
  for (const std::string &e : v)
    s += (s.empty() ? "" : "\n") + e;
  return s;
}

} // namespace

world::generated_world_params generated_world_params_from(const SceneNode &n) {
  world::generated_world_params b;
  b.seed = static_cast<std::uint64_t>(n.props.num("seed", double(b.seed)));
  b.half_m = n.props.num("half", b.half_m);
  b.preset = world::world_preset_from_name(n.props.str("preset", "standard"));
  world::apply_world_preset(b); // the preset's counts and relief, then any the document states
  b.trees = static_cast<int>(n.props.num("trees", b.trees));
  b.rocks = static_cast<int>(n.props.num("rocks", b.rocks));
  b.buildings = static_cast<int>(n.props.num("buildings", b.buildings));
  b.tree_gen = static_cast<int>(n.props.num("tree_gen", b.tree_gen));
  b.amp_m = n.props.num("amp", b.amp_m);
  b.water_level_m = n.props.num("water_level", b.water_level_m);
  const std::string onto = n.props.str("ontology");
  for (int i = 0; i < 3; ++i)
    if (onto == world::world_ontology_name(i))
      b.ontology = i;
  b.terrain = n.props.str("terrain", b.terrain);
  b.city = n.props.str("city", b.city);
  b.terrain_file = n.props.str("terrain_file", b.terrain_file);
  return b;
}

const world::building_library &generated_world_library(cvc::app &app, const SceneNode &n,
                                                       std::vector<std::string> *warnings) {
  std::vector<std::string> paths;
  if (const std::string uri = n.props.str("building_library"); !uri.empty()) {
    ResolvedFile f = resolve_to_file(uri, n.base_dir);
    if (f.ok)
      paths.push_back(f.path);
    else
      warn(warnings, "ari: generated_world '" + n.id + "': building_library " + f.error);
  }
  if (const char *env = std::getenv("CVC_NAV_BUNDLE"); env && *env) {
    const std::string e(env);
    paths.insert(paths.end(), {e, e + "/buildings.glb", e + "/austin_south/buildings.glb"});
  }
  paths.push_back("/bundle/buildings.glb");
  for (const std::string &p : paths) {
    const world::building_library &L = library_at(app, p);
    if (!L.empty())
      return L;
  }
  return library_at(app, "");
}

GeneratedWorldKeys generated_world_keys(cvc::app &app, const SceneNode &n,
                                        const std::string &bind_prefix,
                                        const world::generated_world_params &b, int color) {
  GeneratedWorldKeys k;
  auto key = [&](const char *leaf) { return resolve_bind(bind_prefix, node_key(n, leaf)); };
  auto seed_key = [&](const char *leaf, double def) {
    const std::string path = key(leaf);
    read_or_seed<double>(app, path, def);
    return path;
  };
  auto seed_str = [&](const char *leaf, const std::string &def) {
    const std::string path = key(leaf);
    read_or_seed<std::string>(app, path, def);
    return path;
  };
  k.seed = seed_key("world.seed", double(b.seed));
  k.half = seed_key("world.half", b.half_m);
  k.trees = seed_key("world.trees", b.trees);
  k.rocks = seed_key("world.rocks", b.rocks);
  k.buildings = seed_key("world.buildings", b.buildings);
  k.tree_gen = seed_key("world.tree_gen", b.tree_gen);
  k.amp = seed_key("world.amp", b.amp_m);
  k.water_level = seed_key("world.water_level", b.water_level_m);
  k.preset = seed_key("world.preset", b.preset);
  k.ontology = seed_key("world.ontology", b.ontology);
  k.color = seed_key("world.color", color);
  k.request = seed_str("world.request", std::string());
  // An edit waits for Regenerate unless world.auto is on (the node's auto_regenerate prop seeds
  // it); world.pending says the settings differ from the world shown or being made.
  k.autoregen = seed_key("world.auto", n.props.num("auto_regenerate", 0.0));
  k.pending = key("world.pending");
  write<int>(app, k.pending, 0);
  k.status = key("world.status");
  k.props = key("world.stats.props");
  k.building_count = key("world.stats.buildings");
  k.occupied = key("world.stats.occupied");
  k.terrain = key("world.stats.terrain");
  k.busy = key("world.busy");
  k.progress = key("world.progress");
  k.phase = key("world.phase");
  write<int>(app, k.busy, 0);
  write<int>(app, k.progress, 0);
  write<std::string>(app, k.phase, std::string());
  k.terrain_gen = seed_str("world.terrain", b.terrain);
  k.city = seed_str("world.city", b.city);
  k.terrain_file = seed_str("world.terrain_file", b.terrain_file);
  // The generators a panel can offer (a host may have registered its own), one per line.
  write<std::string>(app, key("world.generators.terrain"),
                     joined(world::terrain_generator_names()));
  write<std::string>(app, key("world.generators.city"), joined(world::city_generator_names()));
  return k;
}

world::generated_world_params read_generated_world_params(cvc::app &app,
                                                          const GeneratedWorldKeys &k,
                                                          const world::generated_world_params &b) {
  world::generated_world_params p = b;
  p.seed = static_cast<std::uint64_t>(read_or<double>(app, k.seed, double(b.seed)));
  p.half_m = read_or<double>(app, k.half, b.half_m);
  p.trees = static_cast<int>(read_or<double>(app, k.trees, b.trees));
  p.rocks = static_cast<int>(read_or<double>(app, k.rocks, b.rocks));
  p.buildings = static_cast<int>(read_or<double>(app, k.buildings, b.buildings));
  p.tree_gen = static_cast<int>(read_or<double>(app, k.tree_gen, b.tree_gen));
  p.amp_m = read_or<double>(app, k.amp, b.amp_m);
  p.water_level_m = read_or<double>(app, k.water_level, b.water_level_m);
  p.preset = static_cast<int>(read_or<double>(app, k.preset, b.preset));
  p.ontology = static_cast<int>(read_or<double>(app, k.ontology, b.ontology));
  p.terrain = read_or<std::string>(app, k.terrain_gen, b.terrain);
  p.city = read_or<std::string>(app, k.city, b.city);
  p.terrain_file = read_or<std::string>(app, k.terrain_file, b.terrain_file);
  return p;
}

std::string generated_world_signature(const world::generated_world_params &p) {
  using std::to_string;
  return to_string((unsigned long long)p.seed) + "|" + to_string(p.half_m) + "|" +
         to_string(p.trees) + "|" + to_string(p.rocks) + "|" + to_string(p.buildings) + "|" +
         to_string(p.tree_gen) + "|" + to_string(p.amp_m) + "|" + to_string(p.water_level_m) + "|" +
         to_string(p.preset) + "|" + to_string(p.ontology) + "|" + p.terrain + "|" + p.city + "|" +
         p.terrain_file;
}

world::generated_world_params resolve_generated_world_file(world::generated_world_params p,
                                                           const std::string &base_dir,
                                                           ResolvedFile &f) {
  if (!p.terrain_file.empty()) {
    f = resolve_to_file(p.terrain_file, base_dir);
    if (!f.ok)
      throw std::runtime_error("terrain_file " + f.error);
    p.terrain_file = f.path;
  }
  return p;
}

std::shared_ptr<world::generated_world>
first_generated_world(cvc::app &app, const GeneratedWorldKeys &k, const SceneNode &n,
                      world::generated_world_params &p, const world::building_library *library,
                      std::vector<std::string> *warnings) {
  try {
    ResolvedFile f;
    return std::make_shared<world::generated_world>(
        world::generate_world(resolve_generated_world_file(p, n.base_dir, f), library));
  } catch (const std::exception &e) {
    // The document's generators failed (an unknown name, a missing DEM): start from the default
    // ones, so there is a world, and say why.
    const std::string why = std::string("error: ") + e.what();
    write<std::string>(app, k.status, why);
    warn(warnings, "ari: generated_world '" + n.id + "': " + why);
  }
  const world::generated_world_params def;
  p.terrain = def.terrain;
  p.city = def.city;
  p.terrain_file.clear();
  write<std::string>(app, k.terrain_gen, p.terrain);
  write<std::string>(app, k.city, p.city);
  write<std::string>(app, k.terrain_file, std::string());
  return std::make_shared<world::generated_world>(world::generate_world(p, library));
}

void publish_generated_world_stats(cvc::app &app, const GeneratedWorldKeys &k,
                                   const world::generated_world &w) {
  write<int>(app, k.props, w.model ? int(w.model->props().size()) : 0);
  write<int>(app, k.building_count, int(w.building_count));
  const double occ = 100.0 * double(w.raster.occupied_count()) /
                     double(std::max<std::size_t>(1, w.raster.klass.size()));
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.1f%%", occ);
  write<std::string>(app, k.occupied, buf);
  const int n = w.terrain ? w.terrain->rows() : 0;
  write<std::string>(app, k.terrain, std::to_string(n) + " x " + std::to_string(n));
  std::string status = w.params.terrain + " terrain, " + w.params.city + " city";
  if (w.building_count)
    status += w.archetypes ? ", Austin archetypes" : ", procedural buildings";
  write<std::string>(app, k.status, status);
}

GeneratedWorldPoll poll_generated_world(cvc::app &app, const GeneratedWorldKeys &k,
                                        const world::generated_world_params &base, int &preset,
                                        std::string &sig) {
  GeneratedWorldPoll out;
  // Commands: Regenerate forces a rebuild, Randomize steps the seed, Stop ends the running job
  // (the world shown stays; the keys keep what was asked, so nothing restarts until one changes).
  const std::string req = read_or<std::string>(app, k.request, std::string());
  bool force = false;
  if (!req.empty()) {
    write<std::string>(app, k.request, std::string());
    if (req == "stop") {
      out.stop = true;
    } else {
      if (req == "randomize") {
        const auto s = static_cast<std::uint64_t>(read_or<double>(app, k.seed, 0.0));
        write<double>(app, k.seed,
                      double((s * 6364136223846793005ull + 1442695040888963407ull) % 100000));
      }
      force = true;
    }
  }
  world::generated_world_params p = read_generated_world_params(app, k, base);
  if (p.preset != preset) { // a new preset quick-sets the counts, relief and water level
    preset = p.preset;
    world::apply_world_preset(p);
    write<double>(app, k.trees, p.trees);
    write<double>(app, k.rocks, p.rocks);
    write<double>(app, k.buildings, p.buildings);
    write<double>(app, k.amp, p.amp_m);
    write<double>(app, k.water_level, p.water_level_m);
  }
  const std::string now = generated_world_signature(p);
  const bool live = read_bool_or(app, k.autoregen, 0) != 0;
  if (force || (live && now != sig)) {
    sig = now;
    out.rebuild = true;
  }
  write<int>(app, k.pending, now != sig ? 1 : 0);
  out.params = std::move(p);
  return out;
}

std::shared_ptr<const world::world_geometry>
generated_world_colliders(const world::generated_world &w) {
  return std::make_shared<const world::world_geometry>(w.geometry());
}

// ── GeneratedWorldRuntime ────────────────────────────────────────────────────────────────────────

struct GeneratedWorldRuntime::impl {
  cvc::app *app = nullptr;
  std::string node;
  std::string base_dir;
  GeneratedWorldKeys keys;
  world::generated_world_params base;
  const world::building_library *library = nullptr;
  std::shared_ptr<const world::generated_world> world;
  std::shared_ptr<SceneColliders> colliders = std::make_shared<SceneColliders>();
  std::string sig;
  int preset = 0;
};

GeneratedWorldRuntime::GeneratedWorldRuntime(cvc::app &app, const SceneNode &n,
                                             const std::string &bind_prefix,
                                             std::vector<std::string> *warnings)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = &app;
  d.node = n.id;
  d.base_dir = n.base_dir;
  d.base = generated_world_params_from(n);
  d.library = &generated_world_library(app, n, warnings);
  const char *const colors[] = {"material", "risk", "occupancy", "height"};
  int color = 0;
  for (int i = 0; i < 4; ++i)
    if (n.props.str("color", "material") == colors[i])
      color = i;
  d.keys = generated_world_keys(app, n, bind_prefix, d.base, color);
  world::generated_world_params p0 = read_generated_world_params(app, d.keys, d.base);
  d.preset = p0.preset;
  std::shared_ptr<world::generated_world> w =
      first_generated_world(app, d.keys, n, p0, d.library, warnings);
  d.sig = generated_world_signature(p0);
  d.world = w;
  d.colliders->geometry = generated_world_colliders(*w);
  d.colliders->revision = 1;
  publish_generated_world_stats(app, d.keys, *w);
  write<int>(app, d.keys.progress, 100);
}

GeneratedWorldRuntime::~GeneratedWorldRuntime() = default;

const std::string &GeneratedWorldRuntime::node() const { return _impl->node; }
std::shared_ptr<const world::generated_world> GeneratedWorldRuntime::world() const {
  return _impl->world;
}
const std::shared_ptr<SceneColliders> &GeneratedWorldRuntime::colliders() const {
  return _impl->colliders;
}
const GeneratedWorldKeys &GeneratedWorldRuntime::keys() const { return _impl->keys; }

bool GeneratedWorldRuntime::tick(std::vector<std::string> *warnings) {
  impl &d = *_impl;
  cvc::app &app = *d.app;
  const GeneratedWorldPoll poll = poll_generated_world(app, d.keys, d.base, d.preset, d.sig);
  if (!poll.rebuild)
    return false;
  try {
    ResolvedFile f;
    auto w = std::make_shared<world::generated_world>(
        world::generate_world(resolve_generated_world_file(poll.params, d.base_dir, f), d.library));
    d.world = w;
    d.colliders->geometry = generated_world_colliders(*w);
    ++d.colliders->revision;
    publish_generated_world_stats(app, d.keys, *w);
    write<int>(app, d.keys.progress, 100);
    return true;
  } catch (const std::exception &e) {
    const std::string why = std::string("error: ") + e.what();
    write<std::string>(app, d.keys.status, why);
    warn(warnings, "ari: generated_world '" + d.node + "': " + why);
    return false;
  }
}

} // namespace ariadne
} // namespace cvc

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
#include <boost/any.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/conference/room_calls.h> // GeneratedWorldRelay: server.regenerate
#include <cvc/ariadne/conference/room_scene.h> // GeneratedWorldRelay: room_world_on
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/core/types.h>          // CVC_VERSION_STRING
#include <cvc/model/model_file_io.h> // can_write_model_to_memory
#include <cvc/state/state.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace world = cvc::world;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// Each app's building libraries, loaded once per path (the Austin bundle is thousands of meshes).
struct building_libraries {
  static constexpr const char *app_slot = "cvc.ariadne.building-libraries";
  std::mutex m;
  std::map<std::string, std::shared_ptr<const world::building_library>> cache;
  const world::building_library none;
};

// The building library at `path`, loaded once per path and app. Empty when the path is empty or
// holds none.
const world::building_library &library_at(cvc::app &app, const std::string &path) {
  building_libraries &libs = app.slot<building_libraries>();
  if (path.empty())
    return libs.none;
  std::lock_guard<std::mutex> lock(libs.m);
  auto it = libs.cache.find(path);
  if (it == libs.cache.end())
    it = libs.cache
             .emplace(path, std::make_shared<const world::building_library>(
                                world::load_building_library(app, path)))
             .first;
  return *it->second;
}

// A bundle's files as a dataset's, in name order.
std::vector<cvc::archive::file> as_files(const world::city_bundle_files &bundle) {
  std::vector<cvc::archive::file> out;
  for (const auto &[name, bytes] : bundle)
    out.push_back({name, bytes});
  return out;
}

// `generated` under the dataset: the world as made, which the renderer draws until it draws any
// world dataset (GB3) and SceneWorld::generated_world reads.
void put_generated(DataDecode &d, std::shared_ptr<const world::generated_world> w) {
  cvc::state &g = d.out("generated");
  g.data(boost::any(std::move(w)));
  g.value(std::string("generated_world"));
}

// The world as made, as a dataset's outputs: where it cannot be written as a bundle.
void put_made(DataDecode &d, const world::generated_world &w) {
  auto geometry = std::make_shared<const world::world_geometry>(w.geometry());
  d.out("world").data(boost::any(geometry));
  d.out("world").value(std::string("world_geometry"));
  if (std::shared_ptr<const cvc::height_field> field = geometry->terrain()) {
    cvc::state &t = d.out("terrain");
    t("rows").value(field->rows());
    t("cols").value(field->cols());
    t.data(boost::any(field));
    t.value(std::string("height_field"));
  }
  if (const world::geometry_item *b = geometry->find("buildings")) {
    d.out("buildings").data(boost::any(b->geometry));
    d.out("buildings").value(std::string("geometry"));
  }
  d.out("water_level").value(std::to_string(w.params.water_level_m));
}

template <class T>
std::shared_ptr<const T> output_of(cvc::app &app, const std::string &path, const char *child) {
  cvc::state *n = cvc::state::instance(app).findThroughLinks(path);
  if (!n)
    return nullptr;
  cvc::state *c =
      cvc::state::instance(app).findDescendant(n->fullName() + cvc::state::SEPARATOR + child);
  if (!c)
    return nullptr;
  const boost::any obj = c->data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&obj))
    return *p;
  return nullptr;
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
  b.edge = std::max(0, world::world_edge_from_name(n.props.str("edge", "wall")));
  return b;
}

const world::building_library &generated_world_library(cvc::app &app, const SceneNode &n,
                                                       std::vector<std::string> *warnings) {
  return generated_world_library(app, n.props.str("building_library"), n.base_dir,
                                 "generated_world '" + n.id + "'", warnings);
}

const world::building_library &generated_world_library(cvc::app &app, const std::string &uri,
                                                       const std::string &base_dir,
                                                       const std::string &owner,
                                                       std::vector<std::string> *warnings) {
  std::vector<std::string> paths;
  if (!uri.empty()) {
    ResolvedFile f = resolve_to_file(app, uri, base_dir);
    if (f.ok)
      paths.push_back(f.path);
    else
      warn(warnings, "ari: " + owner + ": building_library " + f.error);
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
  k.edge = seed_key("world.edge", b.edge); // cvc::world::world_edge: 0 wall, 1 shore, 2 none
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
  k.water = key("world.stats.water");
  k.busy = key("world.busy");
  k.progress = key("world.progress");
  k.phase = key("world.phase");
  write<int>(app, k.busy, 0);
  write<int>(app, k.progress, 0);
  write<std::string>(app, k.phase, std::string());
  k.terrain_gen = seed_str("world.terrain", b.terrain);
  k.city = seed_str("world.city", b.city);
  k.terrain_file = seed_str("world.terrain_file", b.terrain_file);
  // Save: where (the node's save_as prop, else its dataset's name: a directory beside the
  // document), whether with the exports, and how it went.
  k.save_file = seed_str("world.save.file", n.props.str("save_as", generated_world_data_name(n)));
  k.save_exports = seed_key("world.save.exports", 0.0);
  k.save_status = key("world.save.status");
  write<std::string>(app, k.save_status, std::string());
  k.server_status = key("world.server.status");
  write<std::string>(app, k.server_status, std::string());
  k.server_password = key("world.server.password");
  write<std::string>(app, k.server_password, std::string());
  // The generators a panel can offer (a host may have registered its own), one per line.
  write<std::string>(app, key("world.generators.terrain"),
                     joined(world::terrain_generator_names(app)));
  write<std::string>(app, key("world.generators.city"), joined(world::city_generator_names(app)));
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
  p.edge = std::clamp(static_cast<int>(read_or<double>(app, k.edge, b.edge)), 0, 2);
  return p;
}

std::string generated_world_signature(const world::generated_world_params &p) {
  using std::to_string;
  return to_string((unsigned long long)p.seed) + "|" + to_string(p.half_m) + "|" +
         to_string(p.trees) + "|" + to_string(p.rocks) + "|" + to_string(p.buildings) + "|" +
         to_string(p.tree_gen) + "|" + to_string(p.amp_m) + "|" + to_string(p.water_level_m) + "|" +
         to_string(p.preset) + "|" + to_string(p.ontology) + "|" + p.terrain + "|" + p.city + "|" +
         p.terrain_file + "|" + to_string(p.edge);
}

world::generated_world_params resolve_generated_world_file(cvc::app &app,
                                                           world::generated_world_params p,
                                                           const std::string &base_dir,
                                                           ResolvedFile &f) {
  if (!p.terrain_file.empty()) {
    f = resolve_to_file(app, p.terrain_file, base_dir);
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
        world::generate_world(app, resolve_generated_world_file(app, p, n.base_dir, f), library));
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
  return std::make_shared<world::generated_world>(world::generate_world(app, p, library));
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
  write<std::string>(app, k.water, world::water_bodies_summary(w.water));
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
    } else if (req == "save") {
      out.save = true;
    } else if (req == "rcon") {
      out.rcon = true;
    } else {
      out.asked = req != "follow";
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
    if (!force)
      out.asked = true; // an edit with world.auto on
    sig = now;
    out.rebuild = true;
  }
  write<int>(app, k.pending, now != sig ? 1 : 0);
  out.params = std::move(p);
  return out;
}

void write_generated_world_keys(cvc::app &app, const GeneratedWorldKeys &k,
                                const world::generated_world_params &p) {
  write<double>(app, k.seed, double(p.seed));
  write<double>(app, k.half, p.half_m);
  write<double>(app, k.trees, double(p.trees));
  write<double>(app, k.rocks, double(p.rocks));
  write<double>(app, k.buildings, double(p.buildings));
  write<double>(app, k.tree_gen, double(p.tree_gen));
  write<double>(app, k.amp, p.amp_m);
  write<double>(app, k.water_level, p.water_level_m);
  write<double>(app, k.ontology, double(p.ontology));
  write<double>(app, k.edge, double(p.edge));
  write<std::string>(app, k.terrain_gen, p.terrain);
  write<std::string>(app, k.city, p.city);
  write<std::string>(app, k.terrain_file, p.terrain_file);
}

// ── GB5: a generated world while this host is joined to a relay ─────────────────────────────────

struct GeneratedWorldRelay::impl {
  section_stamp on_stamp; // the world_on last looked at
  std::string room;       // ... in this room
  // The relay's reply to the last ask, from a reader thread; follow() writes it.
  std::mutex mu;
  std::string reply;
  bool replied = false;
};

GeneratedWorldRelay::GeneratedWorldRelay() : _impl(std::make_shared<impl>()) {}
GeneratedWorldRelay::~GeneratedWorldRelay() = default;

namespace {
// The relay's params for `node`, when this host is joined to a relay whose world_on lists it.
bool relay_runs(cvc::app &app, const std::string &node, std::string *room,
                conference::room_generated_world *out) {
  const std::shared_ptr<conference::room_call_client> c =
      conference::room_call_client::for_app(app);
  if (!c || node.empty())
    return false;
  conference::room_world_on on;
  if (!conference::read_room_world_on(app, c->room(), on))
    return false;
  for (conference::room_generated_world &g : on.generated)
    if (g.node == node) {
      if (room)
        *room = c->room();
      if (out)
        *out = std::move(g);
      return true;
    }
  return false;
}
} // namespace

bool GeneratedWorldRelay::follow(cvc::app &app, const GeneratedWorldKeys &k,
                                 const std::string &node, int &preset, const std::string &sig) {
  impl &d = *_impl;
  {
    std::lock_guard<std::mutex> lk(d.mu);
    if (d.replied) {
      d.replied = false;
      write<std::string>(app, k.server_status, d.reply);
    }
  }
  const std::shared_ptr<conference::room_call_client> c =
      conference::room_call_client::for_app(app);
  if (!c)
    return false;
  const section_stamp s = conference::room_world_on_stamp(app, c->room());
  if (c->room() == d.room && s.same(d.on_stamp))
    return false;
  d.room = c->room();
  d.on_stamp = s;
  conference::room_generated_world g;
  if (!relay_runs(app, node, nullptr, &g) || generated_world_signature(g.params) == sig)
    return false;
  // The preset first, moved along with the poll's, so it does not quick-set the counts over these.
  write<double>(app, k.preset, double(g.params.preset));
  preset = g.params.preset;
  write_generated_world_keys(app, k, g.params);
  write<std::string>(app, k.request, "follow");
  write<std::string>(app, k.server_status,
                     "the server's world (seed " + std::to_string(g.params.seed) + ")");
  return true;
}

bool GeneratedWorldRelay::ask(cvc::app &app, const GeneratedWorldKeys &k, const std::string &node,
                              const world::generated_world_params &p) {
  if (!relay_runs(app, node, nullptr, nullptr))
    return false;
  const std::shared_ptr<conference::room_call_client> c =
      conference::room_call_client::for_app(app);
  if (!c)
    return false;
  namespace se = cvc::state_exec;
  write<std::string>(app, k.server_status, "asking the server for a new world");
  const std::weak_ptr<impl> self = _impl;
  c->call("server.regenerate",
          se::make_list({se::value_t(node), se::value_t(world::generated_world_params_json(p))}),
          se::make_dict({}), std::string(), 30.0,
          [self](const conference::room_call_client::outcome &o) {
            const std::shared_ptr<impl> d = self.lock();
            if (!d)
              return;
            std::lock_guard<std::mutex> lk(d->mu);
            d->reply = o.ok ? std::string("the server is making the new world")
                            : (o.code == "forbidden" ? "refused: " : "error: " + o.code + ": ") +
                                  o.message;
            d->replied = true;
          });
  return true;
}

bool GeneratedWorldRelay::login(cvc::app &app, const GeneratedWorldKeys &k) {
  const std::string pw = read_or<std::string>(app, k.server_password, std::string());
  write<std::string>(app, k.server_password, std::string());
  const std::shared_ptr<conference::room_call_client> c =
      conference::room_call_client::for_app(app);
  if (!c) {
    write<std::string>(app, k.server_status, "not joined to a server");
    return false;
  }
  namespace se = cvc::state_exec;
  write<std::string>(app, k.server_status, "logging in");
  const std::weak_ptr<impl> self = _impl;
  c->call("server.rcon", se::make_list({se::value_t(pw)}), se::make_dict({}), std::string(), 30.0,
          [self](const conference::room_call_client::outcome &o) {
            const std::shared_ptr<impl> d = self.lock();
            if (!d)
              return;
            std::lock_guard<std::mutex> lk(d->mu);
            d->reply = o.ok ? std::string("an admin of this server: Regenerate makes its world")
                            : "refused: " + o.message;
            d->replied = true;
          });
  return true;
}

bool save_generated_world(cvc::app &app, const GeneratedWorldKeys &k, const std::string &data_path,
                          const std::string &base_dir) {
  const std::string file = read_or<std::string>(app, k.save_file, std::string());
  try {
    if (data_path.empty())
      throw std::runtime_error("the world has no dataset yet");
    DataSaveOptions o;
    o.exports = read_bool_or(app, k.save_exports, 0) != 0;
    const DataSaved s = data_save_files(app, data_path, file, base_dir, o);
    char mb[32];
    std::snprintf(mb, sizeof mb, "%.1f", double(s.bytes) / (1024.0 * 1024.0));
    write<std::string>(app, k.save_status,
                       "saved " + s.where + " (" + std::to_string(s.files) + " files, " + mb +
                           " MB)");
    return true;
  } catch (const std::exception &e) {
    write<std::string>(app, k.save_status, std::string("error: ") + e.what());
    return false;
  }
}

std::shared_ptr<const world::world_geometry>
generated_world_colliders(const world::generated_world &w) {
  return std::make_shared<const world::world_geometry>(w.geometry());
}

// ── A generated world as a dataset ──────────────────────────────────────────────────────────────

std::vector<cvc::archive::file> generated_world_dataset(DataDecode &d) {
  const std::string text = d.spec.params.empty() ? std::string("{}") : d.spec.params;
  namespace pt = boost::property_tree;
  pt::ptree j;
  try {
    std::istringstream in(text);
    pt::read_json(in, j);
  } catch (const std::exception &e) {
    throw std::runtime_error("generate: generated_world: params are not a JSON object: " +
                             std::string(e.what()));
  }
  // A preset first quick-sets its counts, relief and water level; the fields given then win.
  world::generated_world_params base;
  if (const auto preset = j.get_optional<int>("preset")) {
    base.preset = *preset;
    world::apply_world_preset(base);
  }
  const world::generated_world_params p = world::generated_world_params_from_json(text, base);
  const world::building_library &library =
      generated_world_library(d.app, j.get<std::string>("building_library", ""), d.spec.base,
                              "data." + d.spec.name, nullptr);
  ResolvedFile f;
  auto w = std::make_shared<const world::generated_world>(world::generate_world(
      d.app, resolve_generated_world_file(d.app, p, d.spec.base, f), &library));
  put_generated(d, w);
  return as_files(world::generated_world_bundle(d.app, *w, CVC_VERSION_STRING));
}

std::string generated_world_data_name(const SceneNode &n) { return n.props.str("data", n.id); }

std::string publish_generated_world(cvc::app &app, const std::string &bind_prefix,
                                    const SceneNode &n,
                                    std::shared_ptr<const world::generated_world> w,
                                    std::vector<std::string> *warnings,
                                    const world::city_bundle_files *bundle) {
  DataSpec spec;
  spec.name = generated_world_data_name(n);
  spec.generate = "generated_world";
  // Named by where it is bound, so each Regenerate replaces this node's dataset in place.
  spec.uri = "generated_world:" +
             resolve_bind(bind_prefix, std::string(kDataRoot) + cvc::state::SEPARATOR + spec.name);
  spec.params = world::generated_world_params_json(w->params);
  spec.base = n.base_dir;
  std::shared_ptr<DataStore> store = DataStore::of(app);
  if (bundle ? !bundle->empty() : cvc::can_write_model_to_memory(".glb")) {
    try {
      return store->put(
          bind_prefix, spec,
          as_files(bundle ? *bundle : world::generated_world_bundle(app, *w, CVC_VERSION_STRING)),
          [w](DataDecode &d) { put_generated(d, w); });
    } catch (const std::exception &e) {
      warn(warnings, "ari: generated_world '" + n.id + "': its world as a bundle: " + e.what() +
                         "; its physics is the world as made");
    }
  }
  return store->put(bind_prefix, spec, {}, [w](DataDecode &d) {
    put_generated(d, w);
    put_made(d, *w);
  });
}

std::shared_ptr<const world::world_geometry> dataset_world(cvc::app &app, const std::string &path) {
  return output_of<world::world_geometry>(app, path, "world");
}

std::shared_ptr<const world::generated_world> dataset_generated_world(cvc::app &app,
                                                                      const std::string &path) {
  return output_of<world::generated_world>(app, path, "generated");
}

// ── GeneratedWorldRuntime ────────────────────────────────────────────────────────────────────────

struct GeneratedWorldRuntime::impl {
  cvc::app *app = nullptr;
  std::string node;
  std::string prefix;              // the bind prefix: the dataset's link is under it
  std::shared_ptr<SceneNode> spec; // the node, for its dataset's name and directory
  std::string data_path;           // the world's dataset (publish_generated_world)
  std::string base_dir;
  GeneratedWorldKeys keys;
  world::generated_world_params base;
  const world::building_library *library = nullptr;
  std::shared_ptr<const world::generated_world> world;
  std::shared_ptr<SceneColliders> colliders = std::make_shared<SceneColliders>();
  std::string sig;
  int preset = 0;
  GeneratedWorldRelay relay; // GB5: the relay's world, while joined to one that runs this node

  // A new world: its dataset, and the colliders its world.
  void take(std::shared_ptr<const world::generated_world> w, std::vector<std::string> *warnings) {
    world = w;
    data_path = publish_generated_world(*app, prefix, *spec, w, warnings);
    std::shared_ptr<const world::world_geometry> g = dataset_world(*app, data_path);
    colliders->geometry = g ? g : generated_world_colliders(*w);
    ++colliders->revision;
  }
};

GeneratedWorldRuntime::GeneratedWorldRuntime(cvc::app &app, const SceneNode &n,
                                             const std::string &bind_prefix,
                                             std::vector<std::string> *warnings)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = &app;
  d.node = n.id;
  d.prefix = bind_prefix;
  d.spec = std::make_shared<SceneNode>(n);
  d.spec->children.clear();
  d.base_dir = n.base_dir;
  d.base = generated_world_params_from(n);
  d.library = &generated_world_library(app, n, warnings);
  const char *const colors[] = {"material", "risk", "occupancy", "height", "water"};
  int color = 0;
  for (int i = 0; i < 5; ++i)
    if (n.props.str("color", "material") == colors[i])
      color = i;
  d.keys = generated_world_keys(app, n, bind_prefix, d.base, color);
  world::generated_world_params p0 = read_generated_world_params(app, d.keys, d.base);
  d.preset = p0.preset;
  std::shared_ptr<world::generated_world> w =
      first_generated_world(app, d.keys, n, p0, d.library, warnings);
  d.sig = generated_world_signature(p0);
  d.take(w, warnings);
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
const std::string &GeneratedWorldRuntime::data_path() const { return _impl->data_path; }

bool GeneratedWorldRuntime::tick(std::vector<std::string> *warnings) {
  impl &d = *_impl;
  cvc::app &app = *d.app;
  d.relay.follow(app, d.keys, d.node, d.preset, d.sig);
  const std::string shown = d.sig;
  const GeneratedWorldPoll poll = poll_generated_world(app, d.keys, d.base, d.preset, d.sig);
  if (poll.save)
    save_generated_world(app, d.keys, d.data_path, d.base_dir);
  if (poll.rcon)
    d.relay.login(app, d.keys);
  if (!poll.rebuild)
    return false;
  if (poll.asked && d.relay.ask(app, d.keys, d.node, poll.params)) {
    d.sig = shown; // the relay makes it, and world_on brings it back
    return false;
  }
  try {
    ResolvedFile f;
    auto w = std::make_shared<world::generated_world>(world::generate_world(
        app, resolve_generated_world_file(app, poll.params, d.base_dir, f), d.library));
    d.take(w, warnings);
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

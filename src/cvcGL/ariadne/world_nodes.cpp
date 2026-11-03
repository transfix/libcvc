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

// world_nodes.cpp — the generated_world scene node type (see world_nodes.h). A thin cvcGL wrapper:
// cvc::world::generate_world builds the world; this puts its merged meshes, surface texture and
// water into GraphicsNodes, and regenerates them when the world.* keys change. A regeneration or a
// repaint is a sliced task on the app's scheduler (cvc/core/sliced_task.h: world_builder, then
// surface_painter, a few milliseconds a frame), so the page keeps drawing the old world while the
// new one is made; world.progress / world.phase show how far it is, and a request "stop" ends it.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // live node params: read_or_seed / read_or / write / resolve_bind
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/sliced_task.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/image/image.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/generated_world.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;
namespace world = cvc::world;

constexpr int WATER_NZ = 22;
const char *const kColorNames[] = {"material", "risk", "occupancy", "height"};

int color_from(const std::string &s, int def) {
  for (int i = 0; i < 4; ++i)
    if (s == kColorNames[i])
      return i;
  return def;
}

// The building library at `path`, loaded once per path (the Austin bundle is thousands of meshes).
// Empty when the path is empty or holds none.
const world::building_library &library_at(const std::string &path) {
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
                                world::load_building_library(path)))
             .first;
  return *it->second;
}

// Where the archetypes come from: the node's `building_library` (a URI or a path relative to the
// document), else $CVC_NAV_BUNDLE (a buildings.glb, or a directory holding one), else the wasm
// mount /bundle/buildings.glb. The first that loads wins; with none the buildings are procedural.
const world::building_library &find_library(const ari::SceneNode &n,
                                            std::vector<std::string> *warnings) {
  std::vector<std::string> paths;
  if (const std::string uri = n.props.str("building_library"); !uri.empty()) {
    ari::ResolvedFile f = ari::resolve_to_file(uri, n.base_dir);
    if (f.ok)
      paths.push_back(f.path);
    else if (warnings)
      warnings->push_back("ari: generated_world '" + n.id + "': building_library " + f.error);
  }
  if (const char *env = std::getenv("CVC_NAV_BUNDLE"); env && *env) {
    const std::string e(env);
    paths.insert(paths.end(), {e, e + "/buildings.glb", e + "/austin_south/buildings.glb"});
  }
  paths.push_back("/bundle/buildings.glb");
  for (const std::string &p : paths) {
    const world::building_library &L = library_at(p);
    if (!L.empty())
      return L;
  }
  return library_at("");
}

// The world.* keys a world_controls panel drives, as absolute state paths.
struct WorldKeys {
  std::string seed, half, trees, rocks, buildings, tree_gen, amp, water_level, preset, ontology,
      color, request, status, props, building_count, occupied, terrain, terrain_gen, city,
      terrain_file, busy, progress, phase;
};

// A regeneration (a new world, then its texture) or a repaint (the texture of the shown world).
// Its task writes only this; the tick takes it in once `done` is set.
struct WorldJob {
  bool rebuild = true;
  int color = 0;
  std::atomic<bool> done{false};
  std::atomic<int> progress{0};                  // percent
  std::atomic<const char *> phase{nullptr};      // a static string, or null
  std::shared_ptr<world::generated_world> world; // rebuild: made here; repaint: the shown one
  std::vector<unsigned char> texture;
  std::string error; // why it failed; empty when it did not
};

// Everything the tick needs, in one shared handle.
struct WorldRT {
  cvc::app *app = nullptr;
  WorldKeys k;
  world::generated_world_params base; // the authored props
  std::string base_dir;               // the document's, for a relative terrain_file
  const world::building_library *library = nullptr;
  std::shared_ptr<world::generated_world> w; // the world shown
  std::string sig; // the params the current world (or the one being made) is from
  int preset = 0;  // the preset the counts last came from
  int color = 0;   // the colour the shown texture was painted in
  int texture_size = 2048;
  bool soft = true;     // look: soft (demo3's) or classic (terrain_lab's)
  int water_stride = 4; // refill the water every this many frames
  long frame = 0;
  std::shared_ptr<cvc::height_field> field; // published in RealizedScene::height_fields
  std::weak_ptr<GeometryNode> terrain, props, buildings, roads, foliage, needles;
  std::weak_ptr<VolumeNode> water;
  int water_n = 0;
  std::shared_ptr<WorldJob> job; // the running job, or null
  int task = -1;                 // its sliced task's pid

  ~WorldRT() {
    if (task >= 0 && app)
      cvc::cancel_sliced_job(*app, task);
  }
};

world::generated_world_params read_live(WorldRT &rt) {
  cvc::app &app = *rt.app;
  world::generated_world_params p = rt.base;
  p.seed = static_cast<std::uint64_t>(ari::read_or<double>(app, rt.k.seed, double(rt.base.seed)));
  p.half_m = ari::read_or<double>(app, rt.k.half, rt.base.half_m);
  p.trees = static_cast<int>(ari::read_or<double>(app, rt.k.trees, rt.base.trees));
  p.rocks = static_cast<int>(ari::read_or<double>(app, rt.k.rocks, rt.base.rocks));
  p.buildings = static_cast<int>(ari::read_or<double>(app, rt.k.buildings, rt.base.buildings));
  p.tree_gen = static_cast<int>(ari::read_or<double>(app, rt.k.tree_gen, rt.base.tree_gen));
  p.amp_m = ari::read_or<double>(app, rt.k.amp, rt.base.amp_m);
  p.water_level_m = ari::read_or<double>(app, rt.k.water_level, rt.base.water_level_m);
  p.preset = static_cast<int>(ari::read_or<double>(app, rt.k.preset, rt.base.preset));
  p.ontology = static_cast<int>(ari::read_or<double>(app, rt.k.ontology, rt.base.ontology));
  p.terrain = ari::read_or<std::string>(app, rt.k.terrain_gen, rt.base.terrain);
  p.city = ari::read_or<std::string>(app, rt.k.city, rt.base.city);
  p.terrain_file = ari::read_or<std::string>(app, rt.k.terrain_file, rt.base.terrain_file);
  return p;
}

std::string params_sig(const world::generated_world_params &p) {
  using std::to_string;
  return to_string((unsigned long long)p.seed) + "|" + to_string(p.half_m) + "|" +
         to_string(p.trees) + "|" + to_string(p.rocks) + "|" + to_string(p.buildings) + "|" +
         to_string(p.tree_gen) + "|" + to_string(p.amp_m) + "|" + to_string(p.water_level_m) + "|" +
         to_string(p.preset) + "|" + to_string(p.ontology) + "|" + p.terrain + "|" + p.city + "|" +
         p.terrain_file;
}

std::string joined(const std::vector<std::string> &v) {
  std::string s;
  for (const std::string &e : v)
    s += (s.empty() ? "" : "\n") + e;
  return s;
}

// p's terrain_file as a local path (a URI or a path relative to the document); `f` keeps a fetched
// URI's temp file alive until the world is made. Throws when it cannot be had.
world::generated_world_params
resolve_terrain_file(const WorldRT &rt, world::generated_world_params p, ari::ResolvedFile &f) {
  if (!p.terrain_file.empty()) {
    f = ari::resolve_to_file(p.terrain_file, rt.base_dir);
    if (!f.ok)
      throw std::runtime_error("terrain_file " + f.error);
    p.terrain_file = f.path;
  }
  return p;
}

// Generate p's world into rt.w at once (the document's first world, made while it loads). On a
// failure (an unknown generator, a missing DEM) the world is kept, the status says why, and
// false is returned.
bool regenerate(WorldRT &rt, const world::generated_world_params &p) {
  try {
    ari::ResolvedFile f;
    rt.w = std::make_shared<world::generated_world>(
        world::generate_world(resolve_terrain_file(rt, p, f), rt.library));
    return true;
  } catch (const std::exception &e) {
    ari::write<std::string>(*rt.app, rt.k.status, std::string("error: ") + e.what());
    return false;
  }
}

cvc::image surface_image(int size, const std::vector<unsigned char> &rgb) {
  return cvc::image(size, size, cvc::image::pixel_format::RGB, cvc::image::data_type::u8,
                    rgb.data());
}

cvc::volume water_volume(cvc::app &app, const WorldRT &rt, double t) {
  const world::generated_world &w = *rt.w;
  const std::vector<float> f = world::water_field(w, t, rt.water_n, WATER_NZ);
  const double h = w.params.half_m;
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(f.data()),
                     cvc::dimension(rt.water_n, rt.water_n, WATER_NZ), cvc::Float,
                     cvc::bounding_box(-h, -h, world::water_floor(w), h, h, world::water_top(w)));
}

void publish_stats(WorldRT &rt) {
  cvc::app &app = *rt.app;
  const world::generated_world &w = *rt.w;
  ari::write<int>(app, rt.k.props, w.model ? int(w.model->props().size()) : 0);
  ari::write<int>(app, rt.k.building_count, int(w.building_count));
  const double occ = 100.0 * double(w.raster.occupied_count()) /
                     double(std::max<std::size_t>(1, w.raster.klass.size()));
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.1f%%", occ);
  ari::write<std::string>(app, rt.k.occupied, buf);
  const int n = w.terrain ? w.terrain->rows() : 0;
  ari::write<std::string>(app, rt.k.terrain, std::to_string(n) + " x " + std::to_string(n));
  std::string status = w.params.terrain + " terrain, " + w.params.city + " city";
  if (w.building_count)
    status += w.archetypes ? ", Austin archetypes" : ", procedural buildings";
  ari::write<std::string>(app, rt.k.status, status);
}

// Style each mesh node the first time it exists. soft = demo3's daylight look (matte ground and
// concrete, a little specular on the buildings); classic = terrain_lab's.
void style_terrain(GeometryNode &t, bool soft) {
  t.setUseSingleColor(true); // the colour is the baked texture
  t.setColor(1.0, 1.0, 1.0);
  t.setAmbient(soft ? 0.4 : 0.5);
  t.setDiffuse(soft ? 0.85 : 0.95);
  t.setSpecular(0.0);
}
void style_solid(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false); // per-vertex material colour
  g.setAmbient(soft ? 0.36 : 0.45);
  g.setDiffuse(soft ? 0.7 : 0.85);
}
void style_buildings(GeometryNode &g, bool soft, bool archetypes) {
  if (soft && archetypes) { // demo3's matte city
    g.setUseSingleColor(true);
    g.setColor(0.74, 0.72, 0.68);
    g.setAmbient(0.3);
    g.setDiffuse(0.7);
    g.setSpecular(0.04);
    return;
  }
  style_solid(g, soft);
}
void style_roads(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false);
  g.setAmbient(soft ? 0.36 : 0.5);
  g.setDiffuse(0.7);
}
void style_foliage(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false);
  g.setAmbient(soft ? 0.45 : 0.6);
  g.setDiffuse(0.7);
}
void style_needles(GeometryNode &g) {
  g.setRenderMode(GeometryRenderMode::LINES);
  g.setUseSingleColor(true);
  g.setColor(0.12, 0.34, 0.14);
  g.setAmbient(0.55);
  g.setDiffuse(0.75);
}

// A merged mesh as the top-level node `name`: replaced in place, created and styled the first
// time, emptied (not removed: a later world may fill it) when the world has none.
std::weak_ptr<GeometryNode> put_mesh(SceneGraph &sg, const std::string &name,
                                     const cvc::geometry &g, std::weak_ptr<GeometryNode> have,
                                     const std::function<void(GeometryNode &)> &style) {
  if (auto n = have.lock()) {
    n->setGeometry(g);
    style(*n); // a LINES mesh's mode and the colour flags survive a setGeometry only re-applied
    return n;
  }
  auto n = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(name, g));
  if (n)
    style(*n);
  return n;
}

// Push rt.w (and its texture, painted in rt.color) into the nodes, creating the beside-nodes the
// first time.
void apply_world(SceneGraph &sg, const std::string &id, WorldRT &rt,
                 const std::vector<unsigned char> &texture) {
  const world::generated_world &w = *rt.w;
  if (auto t = rt.terrain.lock()) {
    t->setGeometry(cvc::geometry(*w.terrain));
    style_terrain(*t, rt.soft);
    t->setTexture(surface_image(rt.texture_size, texture), false);
  }
  *rt.field = *w.terrain; // the published ground follows the world
  const bool soft = rt.soft, arch = w.archetypes;
  rt.props = put_mesh(sg, id + "_props", w.props, rt.props,
                      [soft](GeometryNode &g) { style_solid(g, soft); });
  rt.buildings = put_mesh(sg, id + "_buildings", w.buildings, rt.buildings,
                          [soft, arch](GeometryNode &g) { style_buildings(g, soft, arch); });
  rt.roads = put_mesh(sg, id + "_roads", w.roads, rt.roads,
                      [soft](GeometryNode &g) { style_roads(g, soft); });
  rt.foliage = put_mesh(sg, id + "_foliage", w.foliage, rt.foliage,
                        [soft](GeometryNode &g) { style_foliage(g, soft); });
  rt.needles = put_mesh(sg, id + "_needles", w.needles, rt.needles,
                        [](GeometryNode &g) { style_needles(g); });
  rt.water_n = world::water_resolution(w);
  if (auto v = rt.water.lock())
    v->setVolume(water_volume(sg.appContext(), rt, rt.app->world_clock().t()));
  publish_stats(rt);
}

// A WorldJob as a sliced task: a rebuild steps a world_builder and then paints the new world's
// texture; a repaint only paints. Natively each step runs on the background pool, on
// single-threaded wasm a few milliseconds a frame on the scheduler thread; either way it writes
// only its job (a repaint reads the shown world, which nothing changes while it runs), and the
// tick takes the job in once `done` is set. The builder's own stages name the phase.
class WorldTask : public cvc::sliced_job {
public:
  WorldTask(std::shared_ptr<WorldJob> job, const world::generated_world_params &p,
            const world::building_library *library, std::shared_ptr<ari::ResolvedFile> file,
            int texture_size)
      : _job(std::move(job)), _file(std::move(file)), _size(texture_size) {
    if (_job->rebuild)
      _builder = std::make_unique<world::world_builder>(p, library);
  }

  bool step(std::size_t budget) override {
    if (done())
      return true;
    try {
      if (_builder) {
        _builder->step(budget);
        set(_builder->phase(), kBuildShare * _builder->progress());
        if (_builder->done()) {
          _job->world = std::make_shared<world::generated_world>(_builder->take());
          _builder.reset();
          _file.reset(); // a fetched DEM's temp file is read
        }
        return done();
      }
      if (!_painter)
        _painter = std::make_unique<world::surface_painter>(
            *_job->world, world::surface_color(_job->color), _size);
      const bool ready = _painter->step(budget);
      const double from = _job->rebuild ? kBuildShare : 0.0;
      set("paint", from + (1.0 - from) * _painter->progress());
      if (ready) {
        _job->texture = _painter->take();
        finish();
      }
    } catch (const std::exception &e) {
      _job->error = e.what();
      finish();
    } catch (...) {
      _job->error = "unknown exception";
      finish();
    }
    return done();
  }
  bool done() const override { return _finished.load() || _cancelled.load(); }
  void cancel() override { _cancelled = true; }
  bool cancelled() const override { return _cancelled.load(); }
  const char *phase() const override { return _phase.load(); }
  double progress() const override { return _progress.load(); }
  std::string status() const override { return _finished.load() ? "done" : "generating"; }

private:
  static constexpr double kBuildShare = 0.85; // a rebuild's progress before its painting

  void set(const char *phase, double progress) {
    _phase = phase;
    _progress = std::max(_progress.load(), std::min(progress, 0.999));
    _job->phase.store(phase, std::memory_order_relaxed);
    _job->progress = static_cast<int>(100.0 * _progress.load());
  }
  void finish() {
    _progress = 1.0;
    _finished = true;
    _job->done.store(true, std::memory_order_release);
  }

  std::shared_ptr<WorldJob> _job;
  std::shared_ptr<ari::ResolvedFile> _file;
  int _size;
  std::unique_ptr<world::world_builder> _builder;
  std::unique_ptr<world::surface_painter> _painter;
  std::atomic<const char *> _phase{"terrain"};
  std::atomic<double> _progress{0.0};
  std::atomic<bool> _finished{false}, _cancelled{false};
};

// End the running job, if any: its task stops at its next check and the shown world stays.
void stop_world_job(WorldRT &rt) {
  if (rt.task >= 0)
    cvc::cancel_sliced_job(*rt.app, rt.task);
  rt.task = -1;
  rt.job.reset();
  ari::write<int>(*rt.app, rt.k.busy, 0);
  ari::write<int>(*rt.app, rt.k.progress, 0);
  ari::write<std::string>(*rt.app, rt.k.phase, std::string());
}

// Start a rebuild of p's world (rebuild) or a repaint of the shown one, in `color`, ending the job
// that was running.
void launch_world_job(WorldRT &rt, bool rebuild, const world::generated_world_params &p,
                      int color) {
  cvc::app &a = *rt.app;
  stop_world_job(rt);
  auto job = std::make_shared<WorldJob>();
  job->rebuild = rebuild;
  job->color = color;
  if (!rebuild)
    job->world = rt.w;
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  ari::write<std::string>(a, rt.k.status, rebuild ? "generating..." : "painting...");
  auto file = std::make_shared<ari::ResolvedFile>();
  world::generated_world_params rp;
  try {
    rp = resolve_terrain_file(rt, p, *file);
  } catch (const std::exception &e) {
    job->error = e.what();
    job->done.store(true, std::memory_order_release);
    return;
  }
  cvc::sliced_task_options o;
  o.kind = "generated_world";
  o.subject = std::string(rebuild ? "generate " : "paint ") + rt.k.request;
  o.owner = "generated_world:" + rt.k.request;
  std::string why;
  rt.task = cvc::schedule_sliced_job(
      a, std::make_shared<WorldTask>(job, rp, rt.library, std::move(file), rt.texture_size), o,
      &why);
  if (rt.task < 0) {
    job->error = "the job could not start: " + why;
    job->done.store(true, std::memory_order_release);
  }
}

// Take a finished job in: a new world into the nodes, or the shown world's new texture. A failed
// one keeps what is shown and says why.
void apply_world_job(SceneGraph &sg, const std::string &id, WorldRT &rt) {
  cvc::app &a = *rt.app;
  std::shared_ptr<WorldJob> j = std::move(rt.job);
  rt.task = -1;
  ari::write<int>(a, rt.k.busy, 0);
  ari::write<std::string>(a, rt.k.phase, std::string());
  if (!j->error.empty()) {
    ari::write<int>(a, rt.k.progress, 0);
    ari::write<std::string>(a, rt.k.status, "error: " + j->error);
    return;
  }
  ari::write<int>(a, rt.k.progress, 100);
  rt.color = j->color;
  if (j->rebuild) {
    rt.w = j->world;
    apply_world(sg, id, rt, j->texture);
  } else {
    if (auto t = rt.terrain.lock())
      t->setTexture(surface_image(rt.texture_size, j->texture), false);
    publish_stats(rt);
  }
}

// type: generated_world — see world_nodes.h.
std::shared_ptr<GraphicsNode> realize_generated_world(SceneGraph &sg, const ari::SceneNode &n,
                                                      GraphicsNode *parent, RealizedScene &out,
                                                      std::vector<std::string> *warnings) {
  auto rt = std::make_shared<WorldRT>();
  rt->app = &sg.appContext();
  cvc::app &app = *rt->app;
  world::generated_world_params &b = rt->base;
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
  rt->base_dir = n.base_dir;
  rt->color = color_from(n.props.str("color", "material"), 0);
  rt->texture_size = std::max(256, std::min(4096, int(n.props.num("texture", 2048.0))));
  rt->soft = n.props.str("look", "soft") != "classic";
  rt->water_stride = std::max(1, int(n.props.num("water_stride", 4.0)));
  const bool water = n.props.num("water", 1.0) != 0.0;
  rt->library = &find_library(n, warnings);

  const std::string pref = sg.getStatePrefix();
  // world.*, or under the node's `keys:` prop (node_key) so two worlds keep separate panels.
  auto key = [&](const char *k) { return ari::resolve_bind(pref, ari::node_key(n, k)); };
  auto seed_key = [&](const char *k, double def) {
    const std::string path = key(k);
    ari::read_or_seed<double>(app, path, def);
    return path;
  };
  rt->k.seed = seed_key("world.seed", double(b.seed));
  rt->k.half = seed_key("world.half", b.half_m);
  rt->k.trees = seed_key("world.trees", b.trees);
  rt->k.rocks = seed_key("world.rocks", b.rocks);
  rt->k.buildings = seed_key("world.buildings", b.buildings);
  rt->k.tree_gen = seed_key("world.tree_gen", b.tree_gen);
  rt->k.amp = seed_key("world.amp", b.amp_m);
  rt->k.water_level = seed_key("world.water_level", b.water_level_m);
  rt->k.preset = seed_key("world.preset", b.preset);
  rt->k.ontology = seed_key("world.ontology", b.ontology);
  rt->k.color = seed_key("world.color", rt->color);
  rt->k.request = key("world.request");
  ari::read_or_seed<std::string>(app, rt->k.request, std::string());
  rt->k.status = key("world.status");
  rt->k.props = key("world.stats.props");
  rt->k.building_count = key("world.stats.buildings");
  rt->k.occupied = key("world.stats.occupied");
  rt->k.terrain = key("world.stats.terrain");
  rt->k.busy = key("world.busy");
  rt->k.progress = key("world.progress");
  rt->k.phase = key("world.phase");
  ari::write<int>(app, rt->k.busy, 0);
  ari::write<int>(app, rt->k.progress, 0);
  ari::write<std::string>(app, rt->k.phase, std::string());
  auto seed_str = [&](const char *k, const std::string &def) {
    const std::string path = key(k);
    ari::read_or_seed<std::string>(app, path, def);
    return path;
  };
  rt->k.terrain_gen = seed_str("world.terrain", b.terrain);
  rt->k.city = seed_str("world.city", b.city);
  rt->k.terrain_file = seed_str("world.terrain_file", b.terrain_file);
  // The generators a panel can offer (a host may have registered its own), one per line.
  ari::write<std::string>(app, key("world.generators.terrain"),
                          joined(world::terrain_generator_names()));
  ari::write<std::string>(app, key("world.generators.city"), joined(world::city_generator_names()));

  world::generated_world_params p0 = read_live(*rt);
  rt->preset = p0.preset;
  rt->color = int(ari::read_or<double>(app, rt->k.color, rt->color));
  if (!regenerate(*rt, p0)) {
    // The document's generators failed (an unknown name, a missing DEM): start from the default
    // ones, so there is a world to show, and say why.
    const std::string why = ari::read_or<std::string>(app, rt->k.status, std::string());
    if (warnings)
      warnings->push_back("ari: generated_world '" + n.id + "': " + why);
    const world::generated_world_params def;
    p0.terrain = def.terrain;
    p0.city = def.city;
    p0.terrain_file.clear();
    ari::write<std::string>(app, rt->k.terrain_gen, p0.terrain);
    ari::write<std::string>(app, rt->k.city, p0.city);
    ari::write<std::string>(app, rt->k.terrain_file, std::string());
    rt->w = std::make_shared<world::generated_world>(world::generate_world(p0, rt->library));
  }
  rt->sig = params_sig(p0);

  // The node is the terrain; its height_field is what `ground: <id>` and a body stand on.
  rt->field = std::make_shared<cvc::height_field>(*rt->w->terrain);
  out.height_fields[n.id] = rt->field;
  const cvc::geometry ground(*rt->w->terrain);
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, ground) : sg.addGraphics(n.id, ground);
  auto tnode = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (!tnode) {
    if (warnings)
      warnings->push_back("ari: generated_world '" + n.id + "': could not create the terrain");
    return gn;
  }
  rt->terrain = tnode;
  if (water) {
    rt->water_n = world::water_resolution(*rt->w);
    auto v = sg.addGraphics(n.id + "_water", water_volume(app, *rt, 0.0));
    if (v) {
      std::vector<double> col, op;
      world::water_transfer(0.0, col, op);
      v->setTransferFunction(col, op);
      v->setShading(true);
      v->setAmbient(0.18);
      v->setDiffuse(0.72);
      v->setSpecular(0.85);
      v->setSpecularPower(70.0);
      rt->water = v;
    }
  }
  apply_world(sg, n.id, *rt,
              world::paint_surface(*rt->w, world::surface_color(rt->color), rt->texture_size));

  SceneGraph *sgp = &sg;
  const std::string id = n.id;
  out.custom_ticks.push_back([rt, sgp, id](vtkRenderer *) {
    if (rt->terrain.expired())
      return;
    cvc::app &app = *rt->app;
    const long f = rt->frame++;
    // Commands: Regenerate forces a rebuild, Randomize steps the seed, Stop ends the running job
    // (the world shown stays; the keys keep what was asked, so nothing restarts until one changes).
    const std::string req = ari::read_or<std::string>(app, rt->k.request, std::string());
    bool force = false;
    if (!req.empty()) {
      ari::write<std::string>(app, rt->k.request, std::string());
      if (req == "stop") {
        if (rt->job) {
          stop_world_job(*rt);
          ari::write<std::string>(app, rt->k.status, "stopped: the previous world is shown");
        }
      } else {
        if (req == "randomize") {
          const auto s = static_cast<std::uint64_t>(ari::read_or<double>(app, rt->k.seed, 0.0));
          ari::write<double>(
              app, rt->k.seed,
              double((s * 6364136223846793005ull + 1442695040888963407ull) % 100000));
        }
        force = true;
      }
    }
    world::generated_world_params p = read_live(*rt);
    if (p.preset != rt->preset) { // a new preset quick-sets the counts and relief
      rt->preset = p.preset;
      world::apply_world_preset(p);
      ari::write<double>(app, rt->k.trees, p.trees);
      ari::write<double>(app, rt->k.rocks, p.rocks);
      ari::write<double>(app, rt->k.buildings, p.buildings);
      ari::write<double>(app, rt->k.amp, p.amp_m);
    }
    // A changed key (or a command) starts a rebuild, ending any job under way; a new colour alone
    // repaints the shown world, once no rebuild is running (a rebuild paints in the colour asked
    // when it started, and a later change is repainted after it).
    const std::string sig = params_sig(p);
    const int c = int(ari::read_or<double>(app, rt->k.color, rt->color));
    if (force || sig != rt->sig) {
      rt->sig = sig;
      launch_world_job(*rt, true, p, c);
    } else if (!rt->job ? c != rt->color : !rt->job->rebuild && rt->job->color != c) {
      launch_world_job(*rt, false, p, c);
    }
    if (rt->job) {
      ari::write<int>(app, rt->k.progress, rt->job->progress.load());
      const char *ph = rt->job->phase.load(std::memory_order_relaxed);
      ari::write<std::string>(app, rt->k.phase,
                              ph ? std::string(ph) + " " +
                                       std::to_string(rt->job->progress.load()) + "%"
                                 : std::string());
      if (rt->job->done.load(std::memory_order_acquire))
        apply_world_job(*sgp, id, *rt);
    }
    // The water rolls on the world clock (Paused freezes it).
    if (f % rt->water_stride == 0)
      if (auto v = rt->water.lock()) {
        const double t = app.world_clock().t();
        v->updateScalars(world::water_field(*rt->w, t, rt->water_n, WATER_NZ));
        std::vector<double> col, op;
        world::water_transfer(t, col, op);
        v->setTransferFunction(col, op);
      }
  });
  return gn;
}

} // namespace

void register_world_node_types() {
  register_scene_node_type("generated_world", realize_generated_world);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

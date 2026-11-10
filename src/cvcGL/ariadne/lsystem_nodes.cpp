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

// lsystem_nodes.cpp — the custom scene node realizers (see lsystem_nodes.h). Thin cvcGL wrappers:
// they read a node's props, call the cvc::lsys generators, and wire the resulting mesh/field into a
// GeometryNode/VolumeNode (+ a per-frame tick). The procedural generation itself lives in
// cvc::lsys.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/bind.h> // live node params: read_or_seed / read_or / resolve_bind
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/lsys/forest.h>
#include <cvc/lsys/water.h>
#include <cvc/volslice/slicer.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/world/cloud_layer.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPropCollection.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextureObject.h>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

// Live node params (§9 — control panels). Each node type reads its procedural settings from state
// keys under a per-type prefix (forest.* / sea.* / sky.*, or the node's `keys:` prop: node_key),
// SEEDED from the authored .ari prop so the document's value is the starting point and a reusable
// control panel (bound to the same keys) then drives it live. seed_key() runs once at realize
// (before any widget renders, so the prop default wins over a widget's def:); read_key() is the
// per-frame follower the tick uses.
std::string seed_key(cvc::app &app, const std::string &prefix, const ari::SceneNode &n,
                     const char *key, double def) {
  const std::string path = ari::resolve_bind(prefix, ari::node_key(n, key));
  ari::read_or_seed<double>(app, path, def); // seed the key from the prop default
  return path;                               // hand back the resolved absolute path for the tick
}
inline double read_key(cvc::app &app, const std::string &path, double def) {
  return ari::read_or<double>(app, path, def);
}

// The ground a node stands on (its `ground:` prop): a heightfield node's own cvc::height_field
// (RealizedScene::height_fields), so the forest and the sea floor sit on the
// triangles the renderer draws and a body rests on. For any other grid-shaped mesh (res x res
// row-major, as a loaded grid may be), the nearest vertex of the grid its vertex count implies.
// Sampled in the ground node's own coordinates, before its transform.
struct TerrainHeights {
  std::shared_ptr<const cvc::height_field> field;
  cvc::geometry mesh; // the ground node's mesh (a copy sharing its arrays): colours, the fallback
  int res = 0;        // the fallback grid's size (0: none)
  double minx = 0, miny = 0, step = 1;
  bool ok() const { return field || res > 1; }
  void build(SceneGraph &sg, const RealizedScene &out, const std::string &ground) {
    auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground));
    if (!gnode || !gnode->getGeometry())
      return;
    mesh = *gnode->getGeometry();
    const auto it = out.height_fields.find(ground);
    if (it != out.height_fields.end()) {
      field = it->second;
      return;
    }
    const std::size_t n = mesh.const_points().size();
    res = static_cast<int>(std::lround(std::sqrt(double(n))));
    if (res < 2 || std::size_t(res) * res != n) {
      res = 0;
      return;
    }
    double maxx = -1e30, maxy = -1e30;
    minx = miny = 1e30;
    for (const auto &p : mesh.const_points()) {
      minx = std::min(minx, p[0]);
      maxx = std::max(maxx, p[0]);
      miny = std::min(miny, p[1]);
      maxy = std::max(maxy, p[1]);
    }
    step = (maxx - minx) / (res - 1);
  }
  // The vertex nearest (x, y), or -1.
  std::int64_t vertex(double x, double y) const {
    if (field) {
      const int c = std::max(
          0, std::min(field->cols() - 1, int(std::lround((x - field->x0()) / field->dx()))));
      const int r = std::max(
          0, std::min(field->rows() - 1, int(std::lround((y - field->y0()) / field->dy()))));
      return field->vertex_of(r, c);
    }
    if (res < 2)
      return -1;
    const int i = std::max(0, std::min(res - 1, int(std::lround((x - minx) / step))));
    const int j = std::max(0, std::min(res - 1, int(std::lround((y - miny) / step))));
    return std::int64_t(j) * res + i;
  }
  // The ground height at (x, y), clamped to the grid at its edges; 0 without a ground.
  double at(double x, double y) const {
    if (field) {
      x = std::max(field->x0(), std::min(field->max_x(), x));
      y = std::max(field->y0(), std::min(field->max_y(), y));
      const double h = field->height(x, y);
      return std::isnan(h) ? 0.0 : h;
    }
    const std::int64_t v = vertex(x, y);
    return v < 0 ? 0.0 : mesh.const_points()[std::size_t(v)][2];
  }
};

// Flatten a mesh's points into a flat x,y,z buffer (the wind tick's working buffer).
std::vector<double> flatten_points(const cvc::geometry &g) {
  std::vector<double> b(g.const_points().size() * 3);
  for (std::size_t v = 0; v < g.const_points().size(); ++v) {
    b[v * 3] = g.const_points()[v][0];
    b[v * 3 + 1] = g.const_points()[v][1];
    b[v * 3 + 2] = g.const_points()[v][2];
  }
  return b;
}

// The pine needle colour (the needle-LINES node is single-coloured).
constexpr double C_NEEDLE_R = 0.137, C_NEEDLE_G = 0.557, C_NEEDLE_B = 0.137;

// Live-tweak state for a forest node (forest_controls.ari binds forest.*). Held in one shared_ptr
// so the regen tick captures a single handle.
struct ForestKeys {
  std::string seed, count, span, sea_level, species, scale, length, radius, levels, branches,
      pine_scale, wind;
};
struct ForestRT {
  cvc::lsys::forest_params base;  // the authored prop defaults
  ForestKeys k;                   // resolved absolute state-key paths
  cvc::lsys::height_fn height_at; // terrain sampler (static)
  std::weak_ptr<GeometryNode> wood, needle;
  std::vector<double> woodBind, needleBind, workW, workN; // route-C wind buffers
  cvc::lsys::forest_wind wind;                            // current wind re-pose data
  std::string sig; // last structural signature (regen trigger)
  cvc::app *app = nullptr;
};
// Read the live forest params from the bound keys (falling back to the authored defaults).
cvc::lsys::forest_params read_live_fp(ForestRT &rt) {
  cvc::lsys::forest_params p = rt.base;
  p.seed = static_cast<std::uint64_t>(read_key(*rt.app, rt.k.seed, double(rt.base.seed)));
  p.count = static_cast<int>(read_key(*rt.app, rt.k.count, rt.base.count));
  p.span = read_key(*rt.app, rt.k.span, rt.base.span);
  p.sea_level = read_key(*rt.app, rt.k.sea_level, rt.base.sea_level);
  const int s = static_cast<int>(read_key(*rt.app, rt.k.species, double(int(rt.base.species))));
  p.species = s == 1   ? cvc::lsys::species_mix::pine
              : s == 2 ? cvc::lsys::species_mix::branchy
                       : cvc::lsys::species_mix::mix;
  p.scale = read_key(*rt.app, rt.k.scale, rt.base.scale);
  p.length = read_key(*rt.app, rt.k.length, rt.base.length);
  p.radius = read_key(*rt.app, rt.k.radius, rt.base.radius);
  p.levels = static_cast<int>(read_key(*rt.app, rt.k.levels, rt.base.levels));
  p.branches = static_cast<int>(read_key(*rt.app, rt.k.branches, rt.base.branches));
  p.pine_scale = read_key(*rt.app, rt.k.pine_scale, rt.base.pine_scale);
  return p;
}
// A signature of the STRUCTURAL params (everything but wind) — a change triggers a regen.
std::string forest_sig(const cvc::lsys::forest_params &p) {
  using std::to_string;
  return to_string((unsigned long long)p.seed) + "|" + to_string(p.count) + "|" +
         to_string(p.span) + "|" + to_string(p.sea_level) + "|" + to_string(int(p.species)) + "|" +
         to_string(p.scale) + "|" + to_string(p.length) + "|" + to_string(p.radius) + "|" +
         to_string(p.levels) + "|" + to_string(p.branches) + "|" + to_string(p.pine_scale);
}

// type: forest_trees — scatter a forest of the demo's two species on a ground node, via
// cvc::lsys::grow_forest. props: count, seed, ground, sea_level, span, species(mix|pine|branchy),
// the branchy knobs (length/radius/levels/branches/scale), pine_scale, and wind (0 disables sway).
std::shared_ptr<GraphicsNode> realize_forest_trees(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent, RealizedScene &out,
                                                   std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: forest_trees '" + n.id + "': " + m);
  };

  // Procedural params: authored in the .ari (props) AND live via forest_controls.ari (binds
  // forest.*). Seed the keys from the props, grow the initial forest, then the tick re-reads them
  // and REGENERATES the mesh when a structural param changes (forest_controls' structural sliders
  // are `commit: release` → one regen per edit). `wind` is cheap and applied every frame.
  auto rt = std::make_shared<ForestRT>();
  rt->app = &sg.appContext();
  rt->base.seed = static_cast<std::uint64_t>(n.props.num("seed", 1337.0));
  rt->base.count = static_cast<int>(n.props.num("count", 60.0));
  rt->base.span = n.props.num("span", 100.0);
  rt->base.sea_level = n.props.num("sea_level", 0.5);
  const std::string species0 = n.props.str("species").empty() ? "mix" : n.props.str("species");
  rt->base.species = species0 == "pine"      ? cvc::lsys::species_mix::pine
                     : species0 == "branchy" ? cvc::lsys::species_mix::branchy
                                             : cvc::lsys::species_mix::mix;
  rt->base.scale = n.props.num("scale", 1.0);
  rt->base.length = n.props.num("length", 6.0);
  rt->base.radius = n.props.num("radius", 0.7);
  rt->base.levels = static_cast<int>(n.props.num("levels", 4.0));
  rt->base.branches = static_cast<int>(n.props.num("branches", 3.0));
  rt->base.pine_scale = n.props.num("pine_scale", 1.35);
  const double windDef = n.props.num("wind", 1.0);

  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  auto heights = std::make_shared<TerrainHeights>();
  heights->build(sg, out, ground);
  if (!heights->ok())
    warn("ground node '" + ground + "' is not a realized heightfield — planting on a flat plane");
  rt->height_at = [heights](double x, double y) { return heights->ok() ? heights->at(x, y) : 0.0; };

  cvc::app &app = sg.appContext();
  const std::string pref = sg.getStatePrefix();
  rt->k.seed = seed_key(app, pref, n, "forest.seed", double(rt->base.seed));
  rt->k.count = seed_key(app, pref, n, "forest.count", rt->base.count);
  rt->k.span = seed_key(app, pref, n, "forest.span", rt->base.span);
  rt->k.sea_level = seed_key(app, pref, n, "forest.sea_level", rt->base.sea_level);
  rt->k.species = seed_key(app, pref, n, "forest.species", double(int(rt->base.species)));
  rt->k.scale = seed_key(app, pref, n, "forest.scale", rt->base.scale);
  rt->k.length = seed_key(app, pref, n, "forest.length", rt->base.length);
  rt->k.radius = seed_key(app, pref, n, "forest.radius", rt->base.radius);
  rt->k.levels = seed_key(app, pref, n, "forest.levels", rt->base.levels);
  rt->k.branches = seed_key(app, pref, n, "forest.branches", rt->base.branches);
  rt->k.pine_scale = seed_key(app, pref, n, "forest.pine_scale", rt->base.pine_scale);
  rt->k.wind = seed_key(app, pref, n, "forest.wind", windDef);

  // Grow the initial forest — ALWAYS with wind records, so enabling wind live later works.
  const cvc::lsys::forest_params fp0 = read_live_fp(*rt);
  cvc::geometry wood, needle;
  cvc::lsys::grow_forest(fp0, rt->height_at, wood, needle, &rt->wind);
  if (wood.const_points().empty()) {
    warn("no trees planted (no dry land above sea_level in the scatter span)");
    return nullptr;
  }
  rt->sig = forest_sig(fp0);

  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, wood) : sg.addGraphics(n.id, wood);
  auto woodNode = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (woodNode)
    woodNode->setUseSingleColor(false); // per-vertex wood colour
  // Always create the needle node (even if empty for a branchy-only forest) so a live species
  // change to pine/mix can fill it.
  auto needleNode =
      std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id + "_needles", needle));
  if (needleNode) {
    needleNode->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
    needleNode->setUseSingleColor(true);
    needleNode->setColor(C_NEEDLE_R, C_NEEDLE_G, C_NEEDLE_B);
  }
  rt->wood = woodNode;
  rt->needle = needleNode;
  rt->woodBind = flatten_points(wood);
  rt->needleBind = flatten_points(needle);
  rt->workW = rt->woodBind;
  rt->workN = rt->needleBind;

  out.custom_ticks.push_back([rt](vtkRenderer *) {
    auto w = rt->wood.lock();
    if (!w)
      return;
    const cvc::lsys::forest_params p = read_live_fp(*rt);
    const double windLive = read_key(*rt->app, rt->k.wind, 1.0);
    // Structural change -> regrow the mesh in place (route-C buffers rebuilt to match).
    const std::string sig = forest_sig(p);
    if (sig != rt->sig) {
      rt->sig = sig;
      cvc::geometry wood2, needle2;
      cvc::lsys::forest_wind wind2;
      cvc::lsys::grow_forest(p, rt->height_at, wood2, needle2, &wind2);
      if (!wood2.const_points().empty()) {
        w->setGeometry(wood2);
        w->setUseSingleColor(false);
        if (auto nn = rt->needle.lock()) {
          nn->setGeometry(needle2);
          nn->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
          nn->setUseSingleColor(true);
          nn->setColor(C_NEEDLE_R, C_NEEDLE_G, C_NEEDLE_B);
        }
        rt->woodBind = flatten_points(wood2);
        rt->needleBind = flatten_points(needle2);
        rt->workW = rt->woodBind;
        rt->workN = rt->needleBind;
        rt->wind = std::move(wind2);
      }
    }
    // Wind re-pose every frame (world/sim time, so Paused freezes the sway). windLive == 0 holds
    // the bind pose; the branchy trees have no sway records and sit untouched.
    if (windLive != 0.0 && !rt->wind.trees.empty()) {
      const double t = rt->app->world_clock().t();
      rt->workW = rt->woodBind;
      rt->workN = rt->needleBind;
      cvc::lsys::repose_forest(rt->wind, t, windLive, rt->workW, rt->workN);
      w->updateVertices(rt->workW);
      if (auto nn = rt->needle.lock())
        nn->updateVertices(rt->workN);
    }
  });
  return gn;
}

// ───────────────────────────── the sea: a travelling-wave VolumeNode ─────────────────────────────
// type: wave_sea — a water-depth VolumeNode re-filled each frame from cvc::lsys::sea_field so the
// sea rolls. props: half, sea_level, wave_amp, ground (the terrain node to sit the water on).
std::shared_ptr<GraphicsNode> realize_wave_sea(SceneGraph &sg, const ari::SceneNode &n,
                                               GraphicsNode *parent, RealizedScene &out,
                                               std::vector<std::string> *warnings) {
  auto sp = std::make_shared<cvc::lsys::sea_params>();
  sp->half = n.props.num("half", 120.0);
  sp->sea_level = n.props.num("sea_level", 0.0);
  sp->wave_amp = n.props.num("wave_amp", 2.40);
  const double half = sp->half;
  const double floorZ = cvc::lsys::sea_floor(*sp), topZ = cvc::lsys::sea_top(*sp);
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  // Live controls (sea_controls.ari binds sea.wave_amp / sea.sea_level): the refill tick reads
  // these each frame, so the amplitude/level are tweakable live (the field is re-filled anyway, no
  // regen).
  const std::string pref = sg.getStatePrefix();
  const std::string ampKey = seed_key(sg.appContext(), pref, n, "sea.wave_amp", sp->wave_amp);
  const std::string lvlKey = seed_key(sg.appContext(), pref, n, "sea.sea_level", sp->sea_level);

  // Terrain height at each sea-grid column (constant), sampled from the realized heightfield.
  TerrainHeights heights;
  heights.build(sg, out, ground);
  auto terr = std::make_shared<std::vector<float>>(static_cast<std::size_t>(sp->n) * sp->n);
  for (int j = 0; j < sp->n; ++j)
    for (int i = 0; i < sp->n; ++i) {
      const double x = -half + 2.0 * half * i / (sp->n - 1);
      const double y = -half + 2.0 * half * j / (sp->n - 1);
      (*terr)[j * sp->n + i] = static_cast<float>(heights.ok() ? heights.at(x, y) : floorZ);
    }

  // Time-varying blue-water transfer function (opacity breathes with the crests).
  auto seaTF = [](std::vector<double> &color, std::vector<double> &opacity, double t) {
    const double k = 0.0100 + 0.0015 * std::sin(t * 0.9);
    color = {0.00, 0.42, 0.78, 0.74, 0.25, 0.14, 0.55, 0.66,
             0.60, 0.04, 0.26, 0.46, 1.00, 0.01, 0.09, 0.22};
    opacity = {0.00, 0.0, 0.12, k * 0.45, 0.55, k, 1.00, k * 2.0};
  };

  auto field = std::make_shared<std::vector<float>>();
  cvc::lsys::sea_field(*sp, *terr, 0.0, *field);
  cvc::volume vol(sg.appContext(), reinterpret_cast<const unsigned char *>(field->data()),
                  cvc::dimension(sp->n, sp->n, sp->nz), cvc::Float,
                  cvc::bounding_box(-half, -half, floorZ, half, half, topZ));
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<VolumeNode>(n.id, vol) : sg.addGraphics(n.id, vol);
  auto vnode = std::dynamic_pointer_cast<VolumeNode>(gn);
  if (!vnode) {
    if (warnings)
      warnings->push_back("ari: wave_sea '" + n.id + "': could not create a VolumeNode");
    return gn;
  }
  {
    std::vector<double> col, op;
    seaTF(col, op, 0.0);
    vnode->setTransferFunction(col, op);
  }
  vnode->setShading(true);
  vnode->setAmbient(0.35);
  vnode->setDiffuse(0.75);

  // Per-frame: re-fill the depth field (the wave rolls) in world time; breathe the TF on a stride.
  auto frame = std::make_shared<long>(0);
  cvc::app *app = &sg.appContext();
  std::weak_ptr<VolumeNode> wn = vnode;
  out.custom_ticks.push_back(
      [wn, field, terr, sp, seaTF, frame, app, ampKey, lvlKey](vtkRenderer *) {
        auto v = wn.lock();
        if (!v)
          return;
        const long f = (*frame)++;
        const double t = app->world_clock().t();
        if (f % 2 == 0) {
          sp->wave_amp = read_key(*app, ampKey, sp->wave_amp); // live from sea_controls
          sp->sea_level = read_key(*app, lvlKey, sp->sea_level);
          cvc::lsys::sea_field(*sp, *terr, t, *field);
          v->updateScalars(*field);
        }
        if (f % 16 == 0) {
          std::vector<double> col, op;
          seaTF(col, op, t);
          v->setTransferFunction(col, op);
        }
      });
  return gn;
}

// ─────────────────────── the sky: a world-scale cloud deck (cvc::world::cloud_layer)
// ─────────────────────── A few L-system cumulus maps scattered over a periodic grid of cells that
// covers the ground (cloud_layer.h). The clouds nearest the camera are drawn by a small pool of
// VolumeNodes, the rest by one mesh of view-aligned slices sampling a coarse opacity texture (the
// far slices, also called the shells below); a cloud passing between the two crossfades. The
// deck's shade is a periodic mask that the receivers' shaders read by world position and apply to
// the direct light alone, so a ground keeps its own texture. Which cells hold a cloud evolves on
// the world clock (the layer's cellular automaton, coupled to the ground's windward flanks); a
// cloud coming or going fades, and its volume, its shell slices and its shade all take the same
// weight. The clouds are lit by the sun they shade the ground from: a near volume's colour is the
// sunlight left after its own cloud's optical depth toward the sun (cloud_layer::cell_sun_tau), a
// far shell's after the whole deck's, so the tops are white and the undersides and the down-sun
// flanks grey.
constexpr int SKY_N = 60, SKY_NZ = 28;
constexpr double SKY_BASE = 74.0, SKY_TOP = 122.0, SKY_HALF = 150.0;
constexpr double CLOUD_DRIFT = 3.0;    // m/s along the wind at drift 1
constexpr double CLOUD_MORPH_S = 30.0; // seconds per variant change at morph 1
constexpr double CLOUD_EMPTY = 0.22;
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0;
constexpr double SHADE_FLOOR =
    0.2;                           // the shade: floor + (1 - floor) e^(-k tau), on the direct light
constexpr double SHADE_GAIN = 4.0; // the shade's optical depth over the volume's (see shade_k)
constexpr double LIGHT_GAIN = 2.0; // the clouds' self-shade's optical depth over the volume's
// The far shells' self-shade over the near volumes': their coarse opacity is box-filtered, so a far
// cloud is more see-through and shows more of its darker depths; at 0.6 a cloud keeps its tone as
// it passes between the two (native snapshots of terrain_lab from the side and from above).
constexpr double FAR_LIGHT = 0.6;
constexpr int LIGHT_SAMPLES = 4;   // a far shell march sample's samples toward the sun
constexpr int NEAR_CLOUDS = 4;     // the near VolumeNodes
constexpr int MAX_NEAR = 8;        // as many as the shells can cut out (cloudCut0..7)
constexpr int STRIDE = 3;          // frames between the near volumes' work
constexpr int REFRESH = 60;        // frames between shade and far composites (the morph is slow)
constexpr int SLICES = 8;          // frames a composite's cells, then its rows, are spread over
constexpr int AREA_STRIDE = 60;    // frames between looks at the ground's extent
constexpr int FADE_FRAMES = 45;    // a cloud's near <-> far crossfade
constexpr double FAR_REACH = 4000.0; // how far past the ground the far clouds reach, metres
constexpr int FAR_SLICES = 64;       // the far slices, at most (VolSliceNode::distance_slicing)
constexpr double FAR_SLICE_GROWTH = 0.06; // each far slice at least this much further than the last
constexpr double FAR_SLICE_START = 0.1;   // the nearest far slice, in cloud half-widths
// A far slab's longest run along a ray, over its depth: each plane is cut taller than the deck by
// that much of its slab (slab_pad). From above, a plane facing the camera crosses the thin deck in
// a narrow band, and far out a ray can enter the deck before the next plane and leave it past this
// one; its slab still holds that stretch, so the plane must reach the ray where the slab begins.
constexpr double FAR_SLAB_REACH = 1.5;
const char *CLOUD_INPUTS = "cloud_sky"; // the shared shader inputs' key on a mesh

// Where the deck's code goes in a lit mesh's fragment shader (vertexVC is there whenever a mesh
// is lit by the scene's lights). A receiver rewrites VTK's lighting sum, which both its mappers
// write (as gl_FragData[0] or fragOutput0), after VTK has generated it
// (cvc::gl::GeometryNode::addFragmentShaderPostReplacement): only there are the direct terms,
// diffuse and specular, apart (the WebGL2 mapper scopes them to its lighting block). The shells
// overwrite the colour after the lighting: a replacement of //VTK::Light::Impl is applied before
// VTK fills the anchor in, and VTK's lighting code ends with the anchor again, so the text after
// it runs once the colour is written.
const char *LIGHT_SUM = "vec4(ambientColor + diffuse + specular, opacity)";

// A receiver: the deck's shade at the base-plane point of the fragment's sun ray, on the direct
// light. cloudVCWC = view -> world, cloudXf = (wind x, wind y, 1 / period, deck base),
// cloudSunR = sun.xy / sun.z. The shade is re-composited every REFRESH frames as the clouds
// change; it crossfades from the last composite (cloudShadePrev) to the newest over the frames
// to the next (cloudShadeMix 0 -> 1), so the shadows change shape smoothly instead of jumping.
const char *SHADE_DEC = R"(//VTK::Light::Dec
uniform highp sampler2D cloudShadeTex;
uniform highp sampler2D cloudShadePrev;
uniform float cloudShadeMix;
uniform mat4 cloudVCWC;
uniform vec4 cloudXf;
uniform vec3 cloudSunR;
uniform float cloudShadeOn;
float cloudShade(vec3 pVC) {
  if (cloudShadeOn <= 0.0) return 1.0;
  vec3 w = (cloudVCWC * vec4(pVC, 1.0)).xyz;
  vec2 uv = (w.xy + (cloudXf.w - w.z) * cloudSunR.xy - cloudXf.xy) * cloudXf.z;
  float s = mix(texture(cloudShadePrev, uv).r, texture(cloudShadeTex, uv).r, cloudShadeMix);
  return mix(1.0, s, cloudShadeOn);
}
)";
const char *SHADE_SUM =
    "vec4(ambientColor + cloudShade(vertexVC.xyz) * (diffuse + specular), opacity)";

// The far clouds are a VolSliceNode: its volume the deck's coarse opacity, sliced by distance
// (VolSliceNode::setDistanceSlicing): planes facing the camera, cut to the deck's box and blended
// back to front, each a fixed fraction further than the last, since the deck is kilometres wide and
// the detail that matters is near. Its sample shader is the clouds' own (setSampleShader, below). A
// slice stands for the slab of deck behind it, up to the next slice: each fragment marches its view
// ray across that slab through the coarse opacity, sums the optical depth (as the volume would
// integrate it), takes the volumes' colour ramp by the sunlight that reaches each sample
// (cloudRamp, cloudSun below), thins out with distance from the camera (cloudFar = camera xy, fade
// start, fade end) and leaves out what a near volume draws (cloudCutK = its cloud's centre xy,
// half-width, weight). cloudFarZ = (deck base, 1 / deck depth, the optical depth per metre at full
// opacity, camera z).
std::string rampGlsl(); // the colour ramp the near volumes look up (skyTransfer), in GLSL
std::string shellDec() {
  // VolSliceNode's sample shader (setSampleShader): the far opacity is its volume
  // (volsliceVolumeTex, repeating across the pattern), volsliceMC the fragment's world point (the
  // node is unmoved), volsliceEye / volsliceView the camera and volsliceGrowth the slab's depth
  // over its slice's distance.
  std::string s = R"(uniform vec4 cloudXf;
uniform vec4 cloudFar;
uniform vec4 cloudFarZ;
uniform vec4 cloudSun;
const int CLOUD_SAMPLES = 6; // a slab's samples at most, about 4 per deck depth of its run
)";
  s += "const int CLOUD_LIGHT_SAMPLES = " + std::to_string(LIGHT_SAMPLES) + ";\n";
  s += rampGlsl();
  for (int k = 0; k < MAX_NEAR; ++k)
    s += "uniform vec4 cloudCut" + std::to_string(k) + ";\n";
  s += R"(float cloudCut(vec2 p, vec4 c) {
  vec2 d = abs(p - c.xy) / max(c.z, 1.0);
  return 1.0 - c.w * (1.0 - smoothstep(0.8, 1.0, max(d.x, d.y)));
}
vec4 cloudShell(vec3 w) {
  // The slab behind this slice, along the view ray: its depth along the view direction is
  // volsliceGrowth times the slice's distance, and the ray crosses it obliquely off the centre.
  vec3 e = volsliceEye;
  vec3 v = normalize(w - e);
  float seg = dot(w - e, volsliceView) * volsliceGrowth / max(dot(v, volsliceView), 0.05);
  // Only the stretch of it inside the deck holds cloud: march that (the slice may lie above or
  // below the deck: FAR_SLAB_REACH).
  float t0 = 0.0, t1 = seg;
  if (abs(v.z) > 1e-5) {
    float ta = (cloudFarZ.x - w.z) / v.z, tb = (cloudFarZ.x + 1.0 / cloudFarZ.y - w.z) / v.z;
    t0 = max(t0, min(ta, tb));
    t1 = min(t1, max(ta, tb));
  } else if (w.z < cloudFarZ.x || w.z > cloudFarZ.x + 1.0 / cloudFarZ.y) {
    t1 = t0;
  }
  if (t1 <= t0)
    return vec4(0.0);
  float run = t1 - t0;
  // The sunlight. Each march sample that holds cloud marches on toward the sun to the deck's top
  // through the coarse opacity (the whole deck's, so a cloud up-sun shades this one too), as the
  // near volumes' self-shade does through their own map; cloudSun = (the sun's direction, the
  // light's optical depth per metre of a far byte at 255). The samples are composited front to
  // back, so a pixel takes the light where its ray meets the cloud.
  float tr = 1.0;
  vec3 lit = vec3(0.0);
  // Samples by the run: far out a slab can span a whole cloud, whose sunlit top must still come
  // first; near, a slab is a few metres and one sample does.
  float ns = clamp(ceil(4.0 * run * cloudFarZ.y), 1.0, float(CLOUD_SAMPLES));
  float stepK = cloudFarZ.z * run / ns;
  for (int i = 0; i < CLOUD_SAMPLES; ++i) {
    if (float(i) >= ns)
      break;
    vec3 q = w + v * (t0 + run * (float(i) + 0.5) / ns);
    float hz = clamp((q.z - cloudFarZ.x) * cloudFarZ.y, 0.0, 1.0);
    float dq = texture(volsliceVolumeTex, vec3((q.xy - cloudXf.xy) * cloudXf.z, hz)).r;
    if (dq < 0.004)
      continue;
    float tq = 0.0;
    if (cloudSun.w > 0.0) {
      float len = max(0.0, cloudFarZ.x + 1.0 / cloudFarZ.y - q.z) / max(cloudSun.z, 0.05);
      for (int j = 0; j < CLOUD_LIGHT_SAMPLES; ++j) {
        vec3 r = q + cloudSun.xyz * (len * (float(j) + 0.5) / float(CLOUD_LIGHT_SAMPLES));
        float hr = clamp((r.z - cloudFarZ.x) * cloudFarZ.y, 0.0, 1.0);
        tq += texture(volsliceVolumeTex, vec3((r.xy - cloudXf.xy) * cloudXf.z, hr)).r;
      }
      tq *= len / float(CLOUD_LIGHT_SAMPLES);
    }
    float ai = 1.0 - exp(-stepK * dq);
    lit += tr * ai * cloudRamp(exp(-cloudSun.w * tq));
    tr *= 1.0 - ai;
  }
  vec3 c = tr < 0.9999 ? lit / (1.0 - tr) : cloudRamp(1.0);
  float a = 1.0 - tr;
  a *= 1.0 - smoothstep(cloudFar.z, cloudFar.w, length(w.xy - cloudFar.xy));
)";
  for (int k = 0; k < MAX_NEAR; ++k)
    s += "  a *= cloudCut(w.xy, cloudCut" + std::to_string(k) + ");\n";
  s += R"(  return vec4(c, a);
}
)";
  return s;
}
const char *SHELL_BODY = "  gl_FragData[0] = cloudShell(volsliceMC);";

// A cloud's colour by the sunlight that reaches it (1 in full sun, toward 0 deep in its own
// shade): the shade a grey-blue the sky still lights, the sunlit side white. The near volumes look
// it up in their colour transfer function, the far shells compute it (cloudRamp), so a cloud
// passing between the two keeps its colours.
constexpr double LIGHT_RAMP[3][4] = {
    {0.0, 0.48, 0.53, 0.62}, {0.5, 0.78, 0.82, 0.89}, {1.0, 1.00, 1.00, 1.00}};

void skyTransfer(double w, std::vector<double> &color, std::vector<double> &opacity) {
  // The colour over the light (the volumes' first component), the opacity over the density (the
  // second); the shadow and the shells integrate the same opacity.
  color.clear();
  for (const auto &pt : LIGHT_RAMP)
    color.insert(color.end(), pt, pt + 4);
  opacity = {0.0, 0.0, CLOUD_EMPTY, 0.0, 0.55, 0.26 * w, 1.0, 0.54 * w};
}

// The ramp in GLSL, from the same table.
std::string rampGlsl() {
  auto v3 = [](const double *c) {
    return "vec3(" + std::to_string(c[1]) + ", " + std::to_string(c[2]) + ", " +
           std::to_string(c[3]) + ")";
  };
  auto segment = [&v3](const double *a, const double *b) {
    return "mix(" + v3(a) + ", " + v3(b) + ", (l - " + std::to_string(a[0]) + ") / " +
           std::to_string(b[0] - a[0]) + ")";
  };
  const double(&R)[3][4] = LIGHT_RAMP;
  return "vec3 cloudRamp(float l) {\n  l = clamp(l, 0.0, 1.0);\n  return l < " +
         std::to_string(R[1][0]) + " ? " + segment(R[0], R[1]) + " : " + segment(R[1], R[2]) +
         ";\n}\n";
}

// A composite spread over frames: its cells a share at a time, then its bytes, then one upload.
struct Composite {
  int phase = 0; // 0 idle, 1 adding cells, 2 converting rows
  std::size_t next = 0;
  double m = 0.0;   // the morph clock it was started at
  long started = 0; // the frame it was started on
  std::vector<float> acc;
  std::vector<unsigned char> bytes;
};

struct SkyRT {
  cvc::app *app = nullptr;
  std::unique_ptr<cvc::world::cloud_layer> layer;
  std::string driftKey, morphKey, shadowKey, evolveKey, oroKey;
  std::vector<std::string>
      receivers; // the nodes whose meshes (and their children's) take the shade
  std::shared_ptr<const cvc::height_field> ground; // its extent is the area, unless `area` is set
  // The height field the orographic coupling reads (the ground's, area or not), and the samples it
  // was last given: a regenerated ground is given again.
  std::shared_ptr<const cvc::height_field> relief;
  const float *reliefData = nullptr;
  std::size_t reliefSize = 0;
  double windX = 1.0, windY = 0.0; // the wind's heading
  // The pattern covers `reach` around the ground's centre (ox, oy): the ground's half-width plus
  // the horizon; its period is wider than that by a cloud on each side. The far slices reach
  // further, `farReach` around it, where the pattern repeats, and the near volumes draw the copies
  // nearest the camera (camX, camY) out there: a camera at the ground's edge or past it is beside
  // copies of the clouds, not the ones over the ground, and those must be the volumes.
  double ox = 0.0, oy = 0.0, area = 0.0, horizon = 0.0, reach = 0.0, farReach = 0.0;
  double camX = 0.0, camY = 0.0;
  double farHorizon = FAR_REACH;
  double shadeK = 0.1, shadeFloor = SHADE_FLOOR;
  double lightK = 0.0; // the self-shade: optical depth per (opacity x metre) toward the sun
  double farK = 1.0;   // the far deck's optical depth per metre at the table's largest opacity
  struct Near {
    std::weak_ptr<VolumeNode> node;
    int cell = -1;  // the cell drawn, or -1
    int weight = 0; // frames into its fade-in, 0..FADE_FRAMES
    bool leaving = false;
    double shown = -1.0; // the opacity scale its transfer function was last given
  };
  std::vector<Near> near;
  int nextMorph = 0;
  std::weak_ptr<VolSliceNode> shells; // the far slices
  double camFwd[3] = {0.0, 0.0, 0.0}; // the camera's unit view direction this frame
  std::shared_ptr<GeometryNode::ShaderInputs> inputs;
  // Two of each, uploaded in turn, so an upload never waits on the GPU drawing the other.
  vtkSmartPointer<vtkTextureObject> shadeTex[2];
  int shadeAt = 0;
  long shadeSwap = 0; // the frame the newest shade went up: its crossfade from the last starts
  Composite shade, far;
  std::vector<float> field, sunTau, light;
  long frame = 0;
  double lastT = -1.0, driftX = 0.0, driftY = 0.0, morph = 0.0;
  std::uint64_t wiredTree = ~std::uint64_t(0); // the scene's treeVersion() at the last wiring
  bool snap = true;  // the next assignment shows its clouds at once (the first frame, a new layout)
  bool fresh = true; // the next composites are done at once (the first frame, a new layout)
  bool shadeWas = false;
};

// The world xy of a cell's cloud: its pattern centre moved by the wind, the periodic copy nearest
// the camera (held to the far slices' box, where the pattern is drawn).
void cloudCentre(const SkyRT &r, const cvc::world::cloud_cell &c, double &wx, double &wy) {
  const double P = r.layer->period_m();
  const double rx = std::clamp(r.camX, r.ox - r.farReach, r.ox + r.farReach);
  const double ry = std::clamp(r.camY, r.oy - r.farReach, r.oy + r.farReach);
  wx = c.cx + r.driftX;
  wy = c.cy + r.driftY;
  wx += P * std::round((rx - wx) / P);
  wy += P * std::round((ry - wy) / P);
}

// The ground's centre and half-width (its height field's box).
void groundBox(const cvc::height_field &g, double &ox, double &oy, double &area) {
  ox = 0.5 * (g.x0() + g.max_x());
  oy = 0.5 * (g.y0() + g.max_y());
  area = 0.5 * std::max(g.max_x() - g.x0(), g.max_y() - g.y0());
}

// Lay the deck over the ground: the reach, the period, the shells; every cloud placed afresh.
void layDeck(SkyRT &r) {
  r.reach = r.area + r.horizon;
  r.farReach = std::max(r.reach, r.area + r.farHorizon);
  r.layer->set_area(r.reach + r.layer->params().extent);
  for (SkyRT::Near &v : r.near) {
    v.cell = -1;
    v.leaving = false;
    v.weight = 0;
    if (auto vn = v.node.lock())
      vn->setVisible(false);
  }
  r.snap = true;
  r.fresh = true;
  r.shade.phase = r.far.phase = 0;
}

// Give the layer the ground's relief for its orographic coupling, when it is new or regenerated;
// true when it was given.
bool giveRelief(SkyRT &r) {
  if (!r.relief || r.relief->rows() < 2 || r.relief->cols() < 2)
    return false;
  const std::vector<float> &h = r.relief->heights();
  if (h.data() == r.reliefData && h.size() == r.reliefSize)
    return false;
  r.reliefData = h.data();
  r.reliefSize = h.size();
  r.layer->set_ground(h, r.relief->rows(), r.relief->cols(), r.relief->x0(), r.relief->y0(),
                      r.relief->dx(), r.relief->dy());
  return true;
}

// Give `g` and every mesh under it the deck's shade, once each.
void wireReceivers(SkyRT &r, GraphicsNode &g) {
  if (auto *m = dynamic_cast<GeometryNode *>(&g); m && !m->hasSharedShaderInputs(CLOUD_INPUTS)) {
    m->addFragmentShaderReplacement("//VTK::Light::Dec", SHADE_DEC);
    m->addFragmentShaderPostReplacement(LIGHT_SUM, SHADE_SUM);
    m->setSharedShaderInputs(CLOUD_INPUTS, r.inputs);
  }
  for (const std::shared_ptr<GraphicsNode> &c : g.getGraphicsChildren())
    if (c)
      wireReceivers(r, *c);
}

// VTK draws the volume layer in the renderer's prop order, and the far slices draw in it
// (GeometryNode::setDrawWithVolumes), each blended over what is drawn before it. Keep that order
// [the scene's other props..., the far slices, the near volumes far to near from the eye]: the far
// clouds over the water, the near clouds over the far ones, and of two near clouds the nearer over
// the other. A node shown again rejoins the renderer at the end (SceneNode::setVisible), and the
// near clouds' distances change as the camera moves: when the order is off, move the far slices,
// then each near volume from the farthest, to the end. Two near clouds at much the same distance
// (within NEAR_ORDER_SLACK) count as in order, so they do not trade places frame after frame.
constexpr double NEAR_ORDER_SLACK = 0.05;
void orderDeck(SkyRT &r, vtkRenderer *ren, const double eye[3]) {
  auto sh = r.shells.lock();
  vtkProp *sp = sh ? sh->prop() : nullptr;
  vtkPropCollection *props = ren->GetViewProps();
  if (!sp || !props->IsItemPresent(sp))
    return;
  std::vector<std::pair<double, vtkProp *>> nearProps; // (distance from the eye, prop)
  for (const SkyRT::Near &v : r.near)
    if (auto vn = v.node.lock())
      if (vtkProp *p = vn->prop(); p && props->IsItemPresent(p)) {
        const double *b = p->GetBounds();
        const double c[3] = {0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), 0.5 * (b[4] + b[5])};
        nearProps.emplace_back(std::sqrt((c[0] - eye[0]) * (c[0] - eye[0]) +
                                         (c[1] - eye[1]) * (c[1] - eye[1]) +
                                         (c[2] - eye[2]) * (c[2] - eye[2])),
                               p);
      }
  // In order: the far slices, then nothing but the near volumes, far to near.
  std::vector<double> after;
  bool seen = false, ok = true;
  vtkCollectionSimpleIterator it;
  props->InitTraversal(it);
  while (vtkProp *p = props->GetNextProp(it)) {
    if (p == sp) {
      seen = true;
      continue;
    }
    if (!seen)
      continue;
    const auto n =
        std::find_if(nearProps.begin(), nearProps.end(),
                     [p](const std::pair<double, vtkProp *> &e) { return e.second == p; });
    if (n == nearProps.end())
      ok = false;
    else
      after.push_back(n->first);
  }
  for (std::size_t k = 1; ok && k < after.size(); ++k)
    ok = after[k] <= after[k - 1] * (1.0 + NEAR_ORDER_SLACK);
  if (ok && after.size() == nearProps.size())
    return;
  std::sort(nearProps.begin(), nearProps.end(),
            [](const std::pair<double, vtkProp *> &a, const std::pair<double, vtkProp *> &b) {
              return a.first > b.first;
            });
  ren->RemoveViewProp(sp);
  ren->AddViewProp(sp);
  for (const auto &e : nearProps) {
    ren->RemoveViewProp(e.second);
    ren->AddViewProp(e.second);
  }
}

// A near volume shows a cell's cloud at the morph clock: its sunlight (the colour) and its density
// (the opacity), the light e^(-lightK tau) of the cloud's own optical depth toward the sun.
void showCell(SkyRT &r, VolumeNode &vn, int cell) {
  const cvc::world::cloud_layer &L = *r.layer;
  L.cell_field(L.cells()[cell], r.morph, r.field);
  L.cell_sun_tau(L.cells()[cell], r.morph, r.sunTau);
  r.light.resize(r.sunTau.size());
  for (std::size_t k = 0; k < r.sunTau.size(); ++k)
    r.light[k] = static_cast<float>(std::exp(-r.lightK * r.sunTau[k]));
  vn.updateScalars(r.light, r.field);
}

// A near volume's opacity: its near <-> far crossfade times its cloud's own fade in or out.
void setNearWeight(const SkyRT &r, SkyRT::Near &v) {
  double w = double(v.weight) / FADE_FRAMES;
  if (v.cell >= 0)
    w *= r.layer->cells()[v.cell].weight();
  if (std::fabs(w - v.shown) < 0.01 && !(w == 0.0 && v.shown != 0.0) &&
      !(w == 1.0 && v.shown != 1.0))
    return;
  v.shown = w;
  if (auto vn = v.node.lock()) {
    std::vector<double> col, op;
    skyTransfer(w, col, op);
    vn->setTransferFunction(col, op);
  }
}

// The near volumes go to the clouds nearest the camera (a cloud already drawn counts a little
// nearer, so two at much the same distance do not trade places), held or still fading out; the
// others are far slices. A cloud wholly out of the camera's view (`fwd`, its unit direction;
// `halfView`, the half-angle of its view's diagonal, radians) counts OUT_OF_VIEW_FACTOR times as
// far, so the volumes go to the clouds it sees; VIEW_MARGIN more of a turn still counts as seen.
constexpr double OUT_OF_VIEW_FACTOR = 4.0;
constexpr double VIEW_MARGIN = 0.26; // radians, about 15 degrees
void assignNear(SkyRT &r, double camx, double camy, double camz, const double fwd[3],
                double halfView) {
  const cvc::world::cloud_layer &L = *r.layer;
  const double zc = 0.5 * (L.params().base + L.params().top);
  const double out = L.params().extent + r.farReach; // a cloud wholly past the deck: not drawn
  std::vector<std::pair<double, int>> order;
  for (std::size_t k = 0; k < L.cells().size(); ++k) {
    const cvc::world::cloud_cell &c = L.cells()[k];
    if (!c.occupied && c.presence <= 0.0)
      continue;
    double wx, wy;
    cloudCentre(r, c, wx, wy);
    if (std::fabs(wx - r.ox) > out || std::fabs(wy - r.oy) > out)
      continue;
    double d = std::sqrt((wx - camx) * (wx - camx) + (wy - camy) * (wy - camy) +
                         (zc - camz) * (zc - camz));
    if (d > 1.5 * L.params().extent) {
      const double c = std::clamp(
          ((wx - camx) * fwd[0] + (wy - camy) * fwd[1] + (zc - camz) * fwd[2]) / d, -1.0, 1.0);
      const double size = std::asin(std::min(1.0, 1.5 * L.params().extent / d));
      if (std::acos(c) - size > halfView + VIEW_MARGIN)
        d *= OUT_OF_VIEW_FACTOR;
    }
    for (const SkyRT::Near &v : r.near)
      if (v.cell == int(k) && !v.leaving)
        d *= 0.8;
    order.emplace_back(d, int(k));
  }
  const std::size_t want = std::min(order.size(), r.near.size());
  std::partial_sort(order.begin(), order.begin() + want, order.end());
  std::vector<int> target;
  for (std::size_t k = 0; k < want; ++k)
    target.push_back(order[k].second);
  for (SkyRT::Near &v : r.near)
    if (v.cell >= 0 && std::find(target.begin(), target.end(), v.cell) == target.end())
      v.leaving = true;
  for (int cell : target) {
    bool drawn = false;
    for (SkyRT::Near &v : r.near)
      if (v.cell == cell) {
        v.leaving = false; // back before it faded out
        drawn = true;
      }
    if (drawn)
      continue;
    for (SkyRT::Near &v : r.near) {
      auto vn = v.node.lock();
      if (v.cell >= 0 || !vn)
        continue;
      v.cell = cell;
      v.leaving = false;
      v.weight = r.snap ? FADE_FRAMES : 0;
      showCell(r, *vn, cell);
      setNearWeight(r, v);
      vn->setVisible(true);
      break;
    }
  }
  r.snap = false;
}

// One frame's share of a composite: start one every REFRESH frames (at once when `now`), add a
// share of the cells, then convert a share of the rows; true when the bytes are ready to upload.
template <class Add, class Rows>
bool stepComposite(Composite &c, long f, double morph, bool now, std::size_t cells, int rows,
                   Add add, Rows convert) {
  if (c.phase == 0) {
    if (!now && f - c.started < REFRESH)
      return false;
    c.phase = 1;
    c.next = 0;
    c.m = morph;
    c.started = f;
    c.acc.clear();
  }
  const int slices = now ? 1 : SLICES;
  if (c.phase == 1) {
    const std::size_t share = (cells + slices - 1) / slices;
    add(c.m, c.next, share, c.acc);
    c.next += share;
    if (c.next < cells)
      return false;
    c.phase = 2;
    c.next = 0;
    if (!now)
      return false;
  }
  const int share = (rows + slices - 1) / slices;
  convert(c.acc, int(c.next), share, c.bytes);
  c.next += std::size_t(share);
  if (int(c.next) < rows)
    return false;
  c.phase = 0;
  return true;
}

// type: cloud_sky — a drifting, morphing, evolving cloud deck over the ground (see above).
// props: ground (the node whose extent the deck covers and whose meshes take its shade), receivers
// (more nodes that take the shade), area (the half-width to cover; default the ground's, else
// half), extent (a cloud's half-width), base and top (the deck's heights), spacing (cell size),
// coverage (the share of cells with a cloud), variants, seed, sun_az / sun_el (the light the
// clouds are lit from and cast along: the scene's key), near (volumes, at most 8), layers (the far
// opacity's levels), horizon (how far past the ground the near volumes draw clouds; by default a
// cloud's half-width or the shadow's throw: they are in the scene's bounds, which the camera fits
// and clips to), far_reach (how far past the ground the far clouds reach; 4 km by default, out of
// the bounds), shade_k / shade_floor (the shade's strength and its darkest), light_k (the clouds'
// own shade: optical depth per metre of opacity toward the sun; 0 lights them evenly), shadow (0
// starts the shade off), wind_az (the heading the wind blows toward, degrees as sun_az: 90, the
// default, is +x), evolve (the weather's rate: 0 holds the clouds where they are), orographic (how
// strongly windward flanks gather cloud: 0 none, 1 the default) and fade (seconds a cloud takes to
// come or go).
std::shared_ptr<GraphicsNode> realize_cloud_sky(SceneGraph &sg, const ari::SceneNode &n,
                                                GraphicsNode *parent, RealizedScene &out,
                                                std::vector<std::string> *warnings) {
  auto rt = std::make_shared<SkyRT>();
  cvc::app &app0 = sg.appContext();
  rt->app = &app0;
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  if (const auto it = out.height_fields.find(ground); it != out.height_fields.end())
    rt->ground = it->second;
  cvc::world::cloud_layer_params p;
  p.seed = static_cast<std::uint64_t>(n.props.num("seed", 20.0));
  p.variants = std::max(1, int(n.props.num("variants", 4.0)));
  p.n = SKY_N;
  p.nz = SKY_NZ;
  p.extent = n.props.num("extent", SKY_HALF);
  p.base = n.props.num("base", SKY_BASE);
  p.top = std::max(p.base + 1.0, n.props.num("top", SKY_TOP));
  p.spacing = n.props.num("spacing", 0.0);
  p.coverage = n.props.num("coverage", 0.6);
  p.sun_az = n.props.num("sun_az", SUN_AZ);
  p.sun_el = n.props.num("sun_el", SUN_EL);
  p.far_layers = std::max(1, int(n.props.num("layers", 10.0)));
  p.wind_az = n.props.num("wind_az", 90.0);
  p.orographic = std::max(0.0, n.props.num("orographic", 1.0));
  p.fade_s = std::max(0.0, n.props.num("fade", 20.0));
  std::vector<double> col;
  skyTransfer(1.0, col, p.opacity); // the shadow and the shells integrate what the volume shows
  rt->relief = rt->ground;          // the orographic coupling reads it even when the area is fixed
  if (n.props.has("area") || !rt->ground) {
    rt->area = n.props.num("area", n.props.num("half", p.extent));
    rt->ground.reset(); // a fixed area: the ground's extent is not followed
  } else {
    groundBox(*rt->ground, rt->ox, rt->oy, rt->area);
  }
  // By default the shells reach past the ground as far as a cloud's width or its shadow's throw,
  // whichever is more, so every shadow on the ground has its cloud drawn.
  constexpr double kPi = 3.14159265358979323846;
  const double throw_m =
      (p.top - p.base) / std::tan(std::min(89.0, std::max(5.0, p.sun_el)) * kPi / 180.0);
  rt->horizon = n.props.num("horizon", std::max(p.extent, throw_m));
  rt->farHorizon = n.props.num("far_reach", FAR_REACH);
  rt->reach = rt->area + rt->horizon;
  rt->farReach = std::max(rt->reach, rt->area + rt->farHorizon);
  p.area = rt->reach + p.extent;
  rt->layer = std::make_unique<cvc::world::cloud_layer>(p);
  {
    const double az = p.wind_az * kPi / 180.0;
    rt->windX = std::sin(az);
    rt->windY = -std::cos(az);
  }
  // The first deck settles over the ground's flanks.
  if (giveRelief(*rt))
    rt->layer->warm(cvc::world::cloud_layer::WARM_STEPS);
  // The volume's opacity is per 1/100 of its box's diagonal (VolumeNode::setVolume), so that is
  // the length a unit of the integrated opacity stands for; -ln(1 - a) ~ 1.1 a over the table.
  // The shade takes SHADE_GAIN times that: the volume's thin outskirts read as solid white from
  // the side (a long sight line), and a cumulus that looks solid should shade like one.
  const double unit =
      std::sqrt(8.0 * p.extent * p.extent + (p.top - p.base) * (p.top - p.base)) / 100.0;
  rt->shadeK = n.props.num("shade_k", SHADE_GAIN * 1.1 / unit);
  rt->shadeFloor = n.props.num("shade_floor", SHADE_FLOOR);
  // The clouds' own shade, in the same units: LIGHT_GAIN times the volume's optical depth. As with
  // the ground's shade, the volume's thin outskirts read as solid from the side, and a cloud that
  // looks solid should grey the far side of itself like one (terrain_lab's deck: about 0.3).
  rt->lightK = std::max(0.0, n.props.num("light_k", LIGHT_GAIN * 1.1 / unit));
  rt->farK = 1.1 * rt->layer->opacity_max() / unit;

  // The near volumes: the first carries the node's id. Each spans one cloud's map, placed each
  // frame where the wind has taken its cloud; hidden while it draws none.
  const int nearCount = std::clamp(int(n.props.num("near", double(NEAR_CLOUDS))), 1, MAX_NEAR);
  const std::vector<float> empty(std::size_t(SKY_N) * SKY_N * SKY_NZ, 0.0f);
  std::shared_ptr<GraphicsNode> first;
  for (int k = 0; k < nearCount; ++k) {
    const std::string id = k == 0 ? n.id : n.id + "_near" + std::to_string(k);
    cvc::volume vol(app0, reinterpret_cast<const unsigned char *>(empty.data()),
                    cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
                    cvc::bounding_box(-p.extent, -p.extent, p.base, p.extent, p.extent, p.top));
    std::shared_ptr<GraphicsNode> gn =
        parent ? parent->createChild<VolumeNode>(id, vol) : sg.addGraphics(id, vol);
    auto vnode = std::dynamic_pointer_cast<VolumeNode>(gn);
    if (!vnode) {
      if (warnings)
        warnings->push_back("ari: cloud_sky '" + n.id + "': could not create a VolumeNode");
      return gn;
    }
    if (k == 0)
      first = gn;
    vnode->setShading(
        false); // VTK's gradient lighting reads as storm cloud here: the tint is the TF's
    vnode->setAmbient(0.95);
    vnode->setDiffuse(0.35);
    vnode->setSpecular(0.0);
    vnode->setVolumetricScattering(0.0);
    vnode->setVisible(false);
    SkyRT::Near v;
    v.node = vnode;
    rt->near.push_back(v);
    setNearWeight(*rt, rt->near.back());
  }

  // The far slices: a VolSliceNode, drawn translucent over everything the near volumes do not
  // draw. Its volume (the far opacity) comes with the first composite.
  rt->inputs = std::make_shared<GeometryNode::ShaderInputs>();
  {
    const std::string id = n.id + "_far";
    GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
    auto shells =
        std::make_shared<VolSliceNode>(app0, pr->getState().fullName() + ".children." + id, id);
    // With the volumes: VTK lays every volume over the translucent layer whatever its depth, so
    // a water volume covered the clouds above it. In the volume layer the slices come after the
    // scene's other volumes (the water: the deck is above sea level, so in front of it from any
    // camera above it) and before the near cloud volumes (nearer the camera); orderDeck keeps
    // the renderer's props in that order. Set before the node joins the scene, so it leaves the
    // renderer's translucency as it is (VolSliceNode::addToRenderer).
    shells->setDrawWithVolumes(true);
    shells->setCastsShadow(false);
    // Out of the bounds the camera fits and clips to, so the slices can reach the horizon; a
    // vertex past the far plane is drawn on it.
    shells->setUseBounds(false);
    shells->setClampToFarPlane(true);
    cvc::volslice::render_settings vs;
    vs.window_min = 0.0; // the far opacity's bytes as they are
    vs.window_max = 255.0;
    vs.tf_auto_domain = false;
    vs.filter = cvc::volslice::interpolation::linear;
    shells->setConfig(vs);
    shells->setVolumeWrap(true, true, false); // the pattern repeats in x and y
    VolSliceNode::distance_slicing ds;
    ds.start = FAR_SLICE_START * p.extent;
    ds.growth = FAR_SLICE_GROWTH;
    ds.max_planes = FAR_SLICES;
    ds.slab_pad = FAR_SLAB_REACH;
    shells->setDistanceSlicing(ds);
    shells->setSampleShader(shellDec(), SHELL_BODY);
    shells->setSharedShaderInputs(CLOUD_INPUTS, rt->inputs);
    shells->setVisible(false); // until its opacity is on the GPU
    pr->addGraphicsChild(std::static_pointer_cast<GraphicsNode>(shells));
    if (!parent)
      sg.registerGraphics(id, shells);
    rt->shells = shells;
  }

  // Live controls (sky_controls.ari binds sky.drift / sky.morph / sky.evolve / sky.orographic /
  // sky.shadow): drift and morph scale the wind and the morph rate, evolve the weather's (the
  // automaton's steps), orographic the windward flanks' pull; shadow turns the shade on and off.
  // The cloud SHAPES are made once (the maps); regenerating them is seconds of work, out of scope
  // for a slider.
  const std::string pref = sg.getStatePrefix();
  rt->driftKey = seed_key(app0, pref, n, "sky.drift", 1.0);
  rt->morphKey = seed_key(app0, pref, n, "sky.morph", 1.0);
  rt->shadowKey = seed_key(app0, pref, n, "sky.shadow", n.props.num("shadow", 1.0));
  rt->evolveKey = seed_key(app0, pref, n, "sky.evolve", std::max(0.0, n.props.num("evolve", 1.0)));
  rt->oroKey = seed_key(app0, pref, n, "sky.orographic", p.orographic);
  rt->receivers.push_back(ground);
  if (const ari::Value *rv = n.props.find("receivers"); rv && rv->is_seq())
    for (const ari::Value &v : rv->items)
      if (!v.scalar.empty())
        rt->receivers.push_back(v.scalar);

  SceneGraph *sgp = &sg;
  out.custom_ticks.push_back([rt, sgp](vtkRenderer *ren) {
    SkyRT &r = *rt;
    cvc::world::cloud_layer &L = *r.layer;
    cvc::app &app = *r.app;
    const long f = r.frame++;

    // The wind and the morph run on the world clock (Paused freezes them), at the live rates.
    const double t = app.world_clock().t();
    const double dt = r.lastT < 0.0 ? 0.0 : std::max(0.0, t - r.lastT);
    r.lastT = t;
    const double ds = dt * CLOUD_DRIFT * read_key(app, r.driftKey, 1.0);
    r.driftX += ds * r.windX;
    r.driftY += ds * r.windY;
    r.morph += dt / CLOUD_MORPH_S * read_key(app, r.morphKey, 1.0);

    // New meshes under the receivers (a regenerated world, an LOD rung made late) take the shade
    // on the frame after they join the scene: adding a node anywhere in it bumps its
    // treeVersion(), so the receivers are walked then and never on a timer. The version is read
    // before the walk, so a mesh added during it is found on the next frame.
    if (const std::uint64_t tv = sgp->treeVersion(); tv != r.wiredTree) {
      r.wiredTree = tv;
      for (const std::string &id : r.receivers)
        if (auto g = sgp->getGraphics(id))
          wireReceivers(r, *g);
    }
    // A grown (or moved) ground re-lays the deck over it.
    if (f % AREA_STRIDE == 0) {
      if (r.ground) {
        double ox, oy, area;
        groundBox(*r.ground, ox, oy, area);
        if (std::fabs(area - r.area) > 1.0 || std::fabs(ox - r.ox) > 1.0 ||
            std::fabs(oy - r.oy) > 1.0) {
          r.ox = ox;
          r.oy = oy;
          r.area = area;
          layDeck(r);
        }
      }
      giveRelief(r); // a regenerated ground: the flanks move, the weather follows
    }
    const double P = L.period_m();
    for (double *d : {&r.driftX, &r.driftY}) {
      *d = std::fmod(*d, P);
      if (*d < 0.0)
        *d += P;
    }

    // The weather: the automaton steps on the world clock at the live rate, over the ground the
    // wind has brought each cell to; its clouds fade in and out.
    L.set_drift(r.driftX, r.driftY);
    L.set_orographic(std::max(0.0, read_key(app, r.oroKey, L.params().orographic)));
    L.evolve(dt, std::max(0.0, read_key(app, r.evolveKey, 1.0)));

    if (!ren || !ren->IsActiveCameraCreated())
      return;
    vtkCamera *cam = ren->GetActiveCamera();
    double cp[3], fp[3];
    cam->GetPosition(cp);
    cam->GetFocalPoint(fp);
    r.camX = cp[0]; // the near volumes draw the clouds' copies nearest the camera (cloudCentre)
    r.camY = cp[1];
    {
      const double fwd[3] = {fp[0] - cp[0], fp[1] - cp[1], fp[2] - cp[2]};
      const double fl = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
      for (int k = 0; k < 3; ++k)
        r.camFwd[k] = fl > 0.0 ? fwd[k] / fl : 0.0;
    }

    // The near volumes: who draws what, the fades, one morph refresh per stride.
    if (r.snap || f % STRIDE == 0) {
      // The view's diagonal half-angle, from its vertical one and the window's aspect.
      int *sz = ren->GetSize();
      const double aspect = sz && sz[1] > 0 ? double(sz[0]) / sz[1] : 1.0;
      const double halfView =
          std::atan(std::tan(0.5 * cam->GetViewAngle() * 3.14159265358979323846 / 180.0) *
                    std::sqrt(1.0 + aspect * aspect));
      assignNear(r, cp[0], cp[1], cp[2], r.camFwd, halfView);
      for (SkyRT::Near &v : r.near) {
        if (v.cell < 0)
          continue;
        v.weight =
            v.leaving ? std::max(0, v.weight - STRIDE) : std::min(FADE_FRAMES, v.weight + STRIDE);
        setNearWeight(r, v); // the near crossfade and the cloud's own fade
        if (v.leaving && v.weight == 0) {
          v.cell = -1;
          v.leaving = false;
          if (auto vn = v.node.lock())
            vn->setVisible(false);
        }
      }
      for (std::size_t k = 0; k < r.near.size(); ++k) {
        SkyRT::Near &v = r.near[(r.nextMorph + k) % r.near.size()];
        if (v.cell < 0)
          continue;
        if (auto vn = v.node.lock())
          showCell(r, *vn, v.cell);
        r.nextMorph = int((r.nextMorph + k + 1) % r.near.size());
        break;
      }
    }
    GeometryNode::ShaderInputs &in = *r.inputs;
    for (std::size_t k = 0; k < r.near.size(); ++k) {
      SkyRT::Near &v = r.near[k];
      std::array<float, 4> cut{0.0f, 0.0f, 1.0f, 0.0f};
      if (v.cell >= 0) {
        double wx, wy;
        cloudCentre(r, L.cells()[v.cell], wx, wy);
        if (auto vn = v.node.lock())
          vn->setPosition(wx, wy, 0.0);
        cut = {float(wx), float(wy), float(L.params().extent), float(v.weight) / FADE_FRAMES};
      }
      in.f4["cloudCut" + std::to_string(k)] = cut;
    }
    // Once the near clouds are placed (one shown this frame rejoined the renderer at the end).
    orderDeck(r, ren, cp);

    // The GPU side: the shade and the far opacity, re-composited a share a frame as the clouds
    // morph (all at once on the first frame and after a new layout).
    auto *ow = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
    if (!ow || !ow->GetInitialized())
      return;
    if (!r.shadeTex[0])
      for (int k = 0; k < 2; ++k) {
        r.shadeTex[k] = vtkSmartPointer<vtkTextureObject>::New();
        r.shadeTex[k]->SetContext(ow);
      }
    const bool shadeOn = read_key(app, r.shadowKey, 1.0) != 0.0;
    const bool now = r.fresh;
    r.fresh = false;
    const int S = L.shadow_size(), F = L.far_size();
    const bool shadeNow = now || !r.shadeWas;
    if (shadeOn && stepComposite(
                       r.shade, f, r.morph, shadeNow, L.cells().size(), S,
                       [&L](double m, std::size_t a, std::size_t n, std::vector<float> &acc) {
                         L.add_shadow(m, a, n, acc);
                       },
                       [&L, &r](const std::vector<float> &acc, int row0, int rows,
                                std::vector<unsigned char> &out) {
                         L.shade_rows(acc, r.shadeK, r.shadeFloor, row0, rows, out);
                       })) {
      // The new composite goes into the other texture; the one shown until now stays as the
      // crossfade's start (none on the first composite, or one done at once: it just shows).
      const auto shown = in.tex.find("cloudShadeTex");
      vtkTextureObject *last = shown != in.tex.end() && !shadeNow ? shown->second.Get() : nullptr;
      vtkTextureObject *tex = r.shadeTex[r.shadeAt ^= 1];
      tex->Create2DFromRaw(S, S, 1, VTK_UNSIGNED_CHAR, r.shade.bytes.data());
      tex->SetWrapS(vtkTextureObject::Repeat);
      tex->SetWrapT(vtkTextureObject::Repeat);
      tex->SetMinificationFilter(vtkTextureObject::Linear);
      tex->SetMagnificationFilter(vtkTextureObject::Linear);
      in.tex["cloudShadeTex"] = tex;
      in.tex["cloudShadePrev"] = last ? last : tex;
      r.shadeSwap = f;
    }
    r.shadeWas = shadeOn;
    in.f["cloudShadeMix"] = float(std::clamp(double(f - r.shadeSwap) / REFRESH, 0.0, 1.0));
    if (stepComposite(
            r.far, f, r.morph, now, L.cells().size(), L.params().far_layers,
            [&L](double m, std::size_t a, std::size_t n, std::vector<float> &acc) {
              L.add_far(m, a, n, {}, acc);
            },
            [&L](const std::vector<float> &acc, int slice0, int slices,
                 std::vector<unsigned char> &out) { L.far_bytes(acc, slice0, slices, out); })) {
      // The far slices' volume: one period of the pattern (the sample shader reads it repeating,
      // by world position), its box the deck's out to the far reach, which they slice.
      if (auto sh = r.shells.lock()) {
        const cvc::world::cloud_layer_params &lp = L.params();
        sh->setVolume(cvc::volume(app, r.far.bytes.data(), cvc::dimension(F, F, lp.far_layers),
                                  cvc::UChar,
                                  cvc::bounding_box(r.ox - r.farReach, r.oy - r.farReach, lp.base,
                                                    r.ox + r.farReach, r.oy + r.farReach, lp.top)));
        sh->setVisible(true);
      }
    }
    if (auto sh = r.shells.lock())
      sh->tick(); // the slices, cut again whenever the camera moves or turns, and the upload

    // The per-frame inputs: the wind, the camera (view -> world for the fragments), the sun.
    const cvc::world::cloud_layer_params &p = L.params();
    double sx, sy, sz;
    L.sun_dir(sx, sy, sz);
    vtkNew<vtkMatrix4x4> vcwc;
    vtkMatrix4x4::Invert(cam->GetModelViewTransformMatrix(), vcwc);
    std::array<double, 16> &m4 = in.m4["cloudVCWC"];
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        m4[i * 4 + j] = vcwc->GetElement(i, j);
    in.f4["cloudXf"] = {float(r.driftX), float(r.driftY), float(1.0 / P), float(p.base)};
    in.f3["cloudSunR"] = {float(sx / sz), float(sy / sz), 0.0f};
    in.f["cloudShadeOn"] = shadeOn && in.tex.count("cloudShadeTex") ? 1.0f : 0.0f;
    in.f4["cloudFar"] = {float(cp[0]), float(cp[1]), float(0.5 * r.farReach), float(r.farReach)};
    in.f4["cloudSun"] = {float(sx), float(sy), float(sz),
                         float(FAR_LIGHT * r.lightK * L.opacity_max())};
    in.f4["cloudFarZ"] = {float(p.base), float(1.0 / (p.top - p.base)), float(r.farK),
                          float(cp[2])};
  });
  return first;
}

} // namespace

void register_lsystem_node_types() {
  register_scene_node_type("forest_trees", realize_forest_trees);
  register_scene_node_type("wave_sea", realize_wave_sea);
  register_scene_node_type("cloud_sky", realize_cloud_sky);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

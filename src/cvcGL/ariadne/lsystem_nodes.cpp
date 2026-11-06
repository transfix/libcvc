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
#include <cvc/ariadne/bind.h> // live node params: read_or_seed / read_or / resolve_bind
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/lsys/forest.h>
#include <cvc/lsys/water.h>
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
// VolumeNodes, the rest by one mesh of translucent horizontal shells sampling a coarse opacity
// texture; a cloud passing between the two crossfades. The deck's shade is a periodic mask that the
// receivers' shaders read by world position and apply to the direct light alone, so a ground keeps
// its own texture.
constexpr int SKY_N = 60, SKY_NZ = 28;
constexpr double SKY_BASE = 74.0, SKY_TOP = 122.0, SKY_HALF = 150.0;
constexpr double CLOUD_DRIFT = 3.0;    // m/s along +x at drift 1
constexpr double CLOUD_MORPH_S = 30.0; // seconds per variant change at morph 1
constexpr double CLOUD_EMPTY = 0.22;
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0;
constexpr double SHADE_FLOOR =
    0.2;                           // the shade: floor + (1 - floor) e^(-k tau), on the direct light
constexpr double SHADE_GAIN = 4.0; // the shade's optical depth over the volume's (see shade_k)
constexpr int NEAR_CLOUDS = 4;     // the near VolumeNodes
constexpr int MAX_NEAR = 8;        // as many as the shells can cut out (cloudCut0..7)
constexpr int STRIDE = 3;          // frames between the near volumes' work
constexpr int REFRESH = 60;        // frames between shade and far composites (the morph is slow)
constexpr int SLICES = 8;          // frames a composite's cells, then its rows, are spread over
constexpr int AREA_STRIDE = 60;    // frames between looks at the ground's extent and receivers
constexpr int FADE_FRAMES = 45;    // a cloud's near <-> far crossfade
constexpr double FAR_REACH = 4000.0;    // how far past the ground the far shells reach, metres
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
// cloudSunR = sun.xy / sun.z.
const char *SHADE_DEC = R"(//VTK::Light::Dec
uniform highp sampler2D cloudShadeTex;
uniform mat4 cloudVCWC;
uniform vec4 cloudXf;
uniform vec3 cloudSunR;
uniform float cloudShadeOn;
float cloudShade(vec3 pVC) {
  if (cloudShadeOn <= 0.0) return 1.0;
  vec3 w = (cloudVCWC * vec4(pVC, 1.0)).xyz;
  vec2 b = w.xy + (cloudXf.w - w.z) * cloudSunR.xy - cloudXf.xy;
  return mix(1.0, texture(cloudShadeTex, b * cloudXf.z).r, cloudShadeOn);
}
)";
const char *SHADE_SUM =
    "vec4(ambientColor + cloudShade(vertexVC.xyz) * (diffuse + specular), opacity)";

// The far shells: each fragment reads its slice of the coarse opacity and turns it into the
// slice's optical depth along the view ray (as the volume would integrate it), takes the volumes'
// colour ramp (grey-blue thin, white dense; a little lighter with height), thins out toward the
// horizon and with distance from the camera (cloudFar = camera xy, fade start, fade end) and
// leaves out what a near volume draws (cloudCutK = its cloud's centre xy, half-width, weight).
// cloudFarZ = (deck base, 1 / deck depth, a slice's vertical optical depth at full opacity,
// camera z).
std::string shellDec() {
  std::string s = R"(//VTK::Light::Dec
uniform highp sampler3D cloudFarTex;
uniform mat4 cloudVCWC;
uniform vec4 cloudXf;
uniform vec4 cloudFar;
uniform vec4 cloudFarZ;
uniform float cloudFarL;
const int CLOUD_SAMPLES = 6;
const float CLOUD_SEG = 24.0;
)";
  for (int k = 0; k < MAX_NEAR; ++k)
    s += "uniform vec4 cloudCut" + std::to_string(k) + ";\n";
  s += R"(float cloudCut(vec2 p, vec4 c) {
  vec2 d = abs(p - c.xy) / max(c.z, 1.0);
  return 1.0 - c.w * (1.0 - smoothstep(0.8, 1.0, max(d.x, d.y)));
}
vec4 cloudShell(vec3 pVC) {
  vec3 w = (cloudVCWC * vec4(pVC, 1.0)).xyz;
  float h = clamp((w.z - cloudFarZ.x) * cloudFarZ.y, 0.0, 1.0);
  // A slice stands for the slab of deck around it: march the view ray across that slab (at most
  // CLOUD_SEG slabs' depth of it) and sum its optical depth, so the slices join up into one cloud
  // however edge-on they are seen, instead of parting into bands; the last degrees fade into haze.
  vec3 v = normalize(w - vec3(cloudFar.xy, cloudFarZ.w));
  float mu = abs(v.z);
  float slab = 1.0 / (cloudFarZ.y * cloudFarL);
  float seg = min(slab / max(mu, 1e-3), CLOUD_SEG * slab);
  float d = 0.0;
  for (int i = 0; i < CLOUD_SAMPLES; ++i) {
    vec3 q = w + v * seg * ((float(i) + 0.5) / float(CLOUD_SAMPLES) - 0.5);
    float hq = clamp((q.z - cloudFarZ.x) * cloudFarZ.y, 0.0, 1.0);
    d += texture(cloudFarTex, vec3((q.xy - cloudXf.xy) * cloudXf.z, hq)).r;
  }
  d /= float(CLOUD_SAMPLES);
  float a = (1.0 - exp(-cloudFarZ.z * d * seg / slab)) * smoothstep(0.0, 0.03, mu);
  a *= 1.0 - smoothstep(cloudFar.z, cloudFar.w, length(w.xy - cloudFar.xy));
)";
  for (int k = 0; k < MAX_NEAR; ++k)
    s += "  a *= cloudCut(w.xy, cloudCut" + std::to_string(k) + ");\n";
  s +=
      R"(  vec3 c = mix(vec3(0.72, 0.78, 0.88), vec3(1.0), clamp(0.35 + 0.25 * h + 0.9 * d, 0.0, 1.0));
  return vec4(c, a);
}
)";
  return s;
}
const char *SHELL_CLAMP = R"(//VTK::PositionVC::Impl
  if (gl_Position.w > 0.0) gl_Position.z = min(gl_Position.z, 0.999 * gl_Position.w);
)";
const char *SHELL_AFTER = R"(//VTK::Light::Impl
  gl_FragData[0] = cloudShell(vertexVC.xyz);
)";

void skyTransfer(double w, std::vector<double> &color, std::vector<double> &opacity) {
  // Thin cloud a soft grey-blue, dense cloud white: the shells' colour follows the same ramp.
  color = {0.0, 0.72, 0.78, 0.88, 0.45, 0.90, 0.93, 0.97, 1.0, 1.00, 1.00, 1.00};
  opacity = {0.0, 0.0, CLOUD_EMPTY, 0.0, 0.55, 0.26 * w, 1.0, 0.54 * w};
}

// One horizontal quad per far slice, at the slice's middle, `reach` around (cx, cy); the top
// slice first, so a camera under the deck blends them far to near.
cvc::geometry shellMesh(const cvc::world::cloud_layer &L, double cx, double cy, double reach) {
  cvc::geometry g;
  const cvc::world::cloud_layer_params &p = L.params();
  for (int k = p.far_layers - 1; k >= 0; --k) {
    const double z = p.base + (k + 0.5) * (p.top - p.base) / p.far_layers;
    const auto v = static_cast<cvc::geometry::index_t>(g.points().size());
    g.points().push_back({cx - reach, cy - reach, z});
    g.points().push_back({cx + reach, cy - reach, z});
    g.points().push_back({cx + reach, cy + reach, z});
    g.points().push_back({cx - reach, cy + reach, z});
    g.tris().push_back({v, v + 1, v + 2});
    g.tris().push_back({v, v + 2, v + 3});
  }
  return g;
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
  std::string driftKey, morphKey, shadowKey;
  std::vector<std::string>
      receivers; // the nodes whose meshes (and their children's) take the shade
  std::shared_ptr<const cvc::height_field> ground; // its extent is the area, unless `area` is set
  // The near volumes draw clouds within `reach` around the ground's centre (ox, oy): the ground's
  // half-width plus the horizon. The pattern's period is wider than that by a cloud on each side,
  // so one copy of each of those clouds is drawn: the one nearest the centre. The far shells reach
  // further, `farReach` around it, where the pattern repeats.
  double ox = 0.0, oy = 0.0, area = 0.0, horizon = 0.0, reach = 0.0, farReach = 0.0;
  double farHorizon = FAR_REACH;
  double shadeK = 0.1, shadeFloor = SHADE_FLOOR;
  double farK = 1.0; // a far slice's vertical optical depth at the table's largest opacity
  struct Near {
    std::weak_ptr<VolumeNode> node;
    int cell = -1;  // the cell drawn, or -1
    int weight = 0; // frames into its fade-in, 0..FADE_FRAMES
    bool leaving = false;
  };
  std::vector<Near> near;
  int nextMorph = 0;
  std::weak_ptr<GeometryNode> shells;
  std::shared_ptr<GeometryNode::ShaderInputs> inputs;
  // Two of each, uploaded in turn, so an upload never waits on the GPU drawing the other.
  vtkSmartPointer<vtkTextureObject> shadeTex[2], farTex[2];
  int shadeAt = 0, farAt = 0;
  Composite shade, far;
  std::vector<float> field;
  long frame = 0;
  double lastT = -1.0, driftX = 0.0, morph = 0.0;
  bool snap = true;  // the next assignment shows its clouds at once (the first frame, a new layout)
  bool fresh = true; // the next composites are done at once (the first frame, a new layout)
  bool shadeWas = false;
};

// The world xy of a cell's cloud: its pattern centre moved by the wind, the periodic copy nearest
// the deck's centre.
void cloudCentre(const SkyRT &r, const cvc::world::cloud_cell &c, double &wx, double &wy) {
  const double P = r.layer->period_m();
  wx = c.cx + r.driftX;
  wy = c.cy;
  wx += P * std::round((r.ox - wx) / P);
  wy += P * std::round((r.oy - wy) / P);
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
  if (auto sh = r.shells.lock())
    sh->setGeometry(shellMesh(*r.layer, r.ox, r.oy, r.farReach));
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

void setNearWeight(SkyRT::Near &v) {
  if (auto vn = v.node.lock()) {
    std::vector<double> col, op;
    skyTransfer(double(v.weight) / FADE_FRAMES, col, op);
    vn->setTransferFunction(col, op);
  }
}

// The near volumes go to the clouds nearest the camera (a cloud already drawn counts a little
// nearer, so two at much the same distance do not trade places); the others are far shells.
void assignNear(SkyRT &r, double camx, double camy, double camz) {
  const cvc::world::cloud_layer &L = *r.layer;
  const double zc = 0.5 * (L.params().base + L.params().top);
  const double out = L.params().extent + r.reach; // a cloud wholly past the deck's edge: not drawn
  std::vector<std::pair<double, int>> order;
  for (std::size_t k = 0; k < L.cells().size(); ++k) {
    const cvc::world::cloud_cell &c = L.cells()[k];
    if (!c.occupied)
      continue;
    double wx, wy;
    cloudCentre(r, c, wx, wy);
    if (std::fabs(wx - r.ox) > out || std::fabs(wy - r.oy) > out)
      continue;
    double d = std::sqrt((wx - camx) * (wx - camx) + (wy - camy) * (wy - camy) +
                         (zc - camz) * (zc - camz));
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
      L.cell_field(L.cells()[cell], r.morph, r.field);
      vn->updateScalars(r.field);
      setNearWeight(v);
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

// type: cloud_sky — a drifting, morphing cloud deck over the ground (see above).
// props: ground (the node whose extent the deck covers and whose meshes take its shade), receivers
// (more nodes that take the shade), area (the half-width to cover; default the ground's, else
// half), extent (a cloud's half-width), base and top (the deck's heights), spacing (cell size),
// coverage (the share of cells with a cloud), variants, seed, sun_az / sun_el (the light the
// clouds are lit from and cast along: the scene's key), near (volumes, at most 8), layers (far
// shells), horizon (how far past the ground the near volumes draw clouds; by default a cloud's
// half-width or the shadow's throw: they are in the scene's bounds, which the camera fits and
// clips to), far_reach (how far past the ground the far shells reach; 4 km by default, out of the
// bounds),
// shade_k / shade_floor (the shade's strength and its darkest), shadow (0 starts the shade off).
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
  std::vector<double> col;
  skyTransfer(1.0, col, p.opacity); // the shadow and the shells integrate what the volume shows
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
  // The volume's opacity is per 1/100 of its box's diagonal (VolumeNode::setVolume), so that is
  // the length a unit of the integrated opacity stands for; -ln(1 - a) ~ 1.1 a over the table.
  // The shade takes SHADE_GAIN times that: the volume's thin outskirts read as solid white from
  // the side (a long sight line), and a cumulus that looks solid should shade like one.
  const double unit =
      std::sqrt(8.0 * p.extent * p.extent + (p.top - p.base) * (p.top - p.base)) / 100.0;
  rt->shadeK = n.props.num("shade_k", SHADE_GAIN * 1.1 / unit);
  rt->shadeFloor = n.props.num("shade_floor", SHADE_FLOOR);
  rt->farK = 1.1 * rt->layer->opacity_max() * (p.top - p.base) / p.far_layers / unit;

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
    setNearWeight(rt->near.back());
  }

  // The far shells: one mesh, drawn translucent over everything the near volumes do not draw.
  rt->inputs = std::make_shared<GeometryNode::ShaderInputs>();
  {
    const std::string id = n.id + "_far";
    const cvc::geometry none; // the mesh comes once the shells are out of the bounds (below)
    std::shared_ptr<GraphicsNode> gn =
        parent ? parent->createChild<GeometryNode>(id, none) : sg.addGraphics(id, none);
    if (auto shells = std::dynamic_pointer_cast<GeometryNode>(gn)) {
      shells->setCastsShadow(false);
      shells->setOpacity(0.99); // the translucent pass; the shader writes the real alpha
      shells->setSpecular(0.0);
      shells->addFragmentShaderReplacement("//VTK::Light::Dec", shellDec());
      shells->addFragmentShaderReplacement("//VTK::Light::Impl", SHELL_AFTER);
      // Out of the bounds the camera fits and clips to, so the shells can reach the horizon; a
      // vertex past the far plane is drawn on it (its depth only: where it lands on screen, and
      // the world point the fragments read, are unchanged).
      shells->setUseBounds(false);
      shells->setGeometry(shellMesh(*rt->layer, rt->ox, rt->oy, rt->farReach));
      shells->addVertexShaderReplacement("//VTK::PositionVC::Impl", SHELL_CLAMP);
      shells->setSharedShaderInputs(CLOUD_INPUTS, rt->inputs);
      shells->setVisible(false); // until its opacity is on the GPU
      rt->shells = shells;
    }
  }

  // Live controls (sky_controls.ari binds sky.drift / sky.morph / sky.shadow): drift and morph
  // scale the wind and the morph rate; shadow turns the shade on and off. The cloud SHAPES are made
  // once (the maps); regenerating them is seconds of work, out of scope for a slider.
  const std::string pref = sg.getStatePrefix();
  rt->driftKey = seed_key(app0, pref, n, "sky.drift", 1.0);
  rt->morphKey = seed_key(app0, pref, n, "sky.morph", 1.0);
  rt->shadowKey = seed_key(app0, pref, n, "sky.shadow", n.props.num("shadow", 1.0));
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
    r.driftX += dt * CLOUD_DRIFT * read_key(app, r.driftKey, 1.0);
    r.morph += dt / CLOUD_MORPH_S * read_key(app, r.morphKey, 1.0);

    // A grown (or moved) ground re-lays the deck over it; new meshes under the receivers (a
    // regenerated world, an LOD rung made late) take the shade.
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
      for (const std::string &id : r.receivers)
        if (auto g = sgp->getGraphics(id))
          wireReceivers(r, *g);
    }
    const double P = L.period_m();
    r.driftX = std::fmod(r.driftX, P);
    if (r.driftX < 0.0)
      r.driftX += P;

    if (!ren || !ren->IsActiveCameraCreated())
      return;
    vtkCamera *cam = ren->GetActiveCamera();
    double cp[3];
    cam->GetPosition(cp);

    // The near volumes: who draws what, the fades, one morph refresh per stride.
    if (r.snap || f % STRIDE == 0) {
      assignNear(r, cp[0], cp[1], cp[2]);
      for (SkyRT::Near &v : r.near) {
        if (v.cell < 0)
          continue;
        const int was = v.weight;
        v.weight =
            v.leaving ? std::max(0, v.weight - STRIDE) : std::min(FADE_FRAMES, v.weight + STRIDE);
        if (v.weight != was)
          setNearWeight(v);
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
        if (auto vn = v.node.lock()) {
          L.cell_field(L.cells()[v.cell], r.morph, r.field);
          vn->updateScalars(r.field);
        }
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

    // The GPU side: the shade and the far opacity, re-composited a share a frame as the clouds
    // morph (all at once on the first frame and after a new layout).
    auto *ow = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
    if (!ow || !ow->GetInitialized())
      return;
    if (!r.shadeTex[0])
      for (int k = 0; k < 2; ++k) {
        r.shadeTex[k] = vtkSmartPointer<vtkTextureObject>::New();
        r.shadeTex[k]->SetContext(ow);
        r.farTex[k] = vtkSmartPointer<vtkTextureObject>::New();
        r.farTex[k]->SetContext(ow);
      }
    const bool shadeOn = read_key(app, r.shadowKey, 1.0) != 0.0;
    const bool now = r.fresh;
    r.fresh = false;
    const int S = L.shadow_size(), F = L.far_size();
    if (shadeOn && stepComposite(
                       r.shade, f, r.morph, now || !r.shadeWas, L.cells().size(), S,
                       [&L](double m, std::size_t a, std::size_t n, std::vector<float> &acc) {
                         L.add_shadow(m, a, n, acc);
                       },
                       [&L, &r](const std::vector<float> &acc, int row0, int rows,
                                std::vector<unsigned char> &out) {
                         L.shade_rows(acc, r.shadeK, r.shadeFloor, row0, rows, out);
                       })) {
      vtkTextureObject *tex = r.shadeTex[r.shadeAt ^= 1];
      tex->Create2DFromRaw(S, S, 1, VTK_UNSIGNED_CHAR, r.shade.bytes.data());
      tex->SetWrapS(vtkTextureObject::Repeat);
      tex->SetWrapT(vtkTextureObject::Repeat);
      tex->SetMinificationFilter(vtkTextureObject::Linear);
      tex->SetMagnificationFilter(vtkTextureObject::Linear);
      in.tex["cloudShadeTex"] = tex;
    }
    r.shadeWas = shadeOn;
    if (stepComposite(
            r.far, f, r.morph, now, L.cells().size(), L.params().far_layers,
            [&L](double m, std::size_t a, std::size_t n, std::vector<float> &acc) {
              L.add_far(m, a, n, {}, acc);
            },
            [&L](const std::vector<float> &acc, int slice0, int slices,
                 std::vector<unsigned char> &out) { L.far_bytes(acc, slice0, slices, out); })) {
      vtkTextureObject *tex = r.farTex[r.farAt ^= 1];
      tex->Create3DFromRaw(F, F, L.params().far_layers, 1, VTK_UNSIGNED_CHAR, r.far.bytes.data());
      tex->SetWrapS(vtkTextureObject::Repeat);
      tex->SetWrapT(vtkTextureObject::Repeat);
      tex->SetWrapR(vtkTextureObject::ClampToEdge);
      tex->SetMinificationFilter(vtkTextureObject::Linear);
      tex->SetMagnificationFilter(vtkTextureObject::Linear);
      in.tex["cloudFarTex"] = tex;
      if (auto sh = r.shells.lock())
        sh->setVisible(true);
    }

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
    in.f4["cloudXf"] = {float(r.driftX), 0.0f, float(1.0 / P), float(p.base)};
    in.f3["cloudSunR"] = {float(sx / sz), float(sy / sz), 0.0f};
    in.f["cloudShadeOn"] = shadeOn && in.tex.count("cloudShadeTex") ? 1.0f : 0.0f;
    in.f4["cloudFar"] = {float(cp[0]), float(cp[1]), float(0.5 * r.farReach), float(r.farReach)};
    in.f["cloudFarL"] = float(p.far_layers);
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

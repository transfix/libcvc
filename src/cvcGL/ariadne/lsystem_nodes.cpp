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
#include <cmath>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/image/image.h>
#include <cvc/lsys/cloud.h>
#include <cvc/lsys/forest.h>
#include <cvc/lsys/water.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

// Sample a realized heightfield GeometryNode: make_heightfield lays out a regular row-major res×res
// grid, so a nearest-cell lookup gives the ground height (and band colour) at (x, y). Ported from
// the demo so the forest plants on the island's actual relief.
struct TerrainHeights {
  const cvc::geometry *geom = nullptr;
  int res = 0;
  double minx = 0, miny = 0, step = 1;
  bool ok() const { return geom && res > 1; }
  void build(const cvc::geometry *g) {
    geom = g;
    if (!g || g->const_points().empty())
      return;
    const std::size_t n = g->const_points().size();
    res = static_cast<int>(std::lround(std::sqrt(double(n))));
    if (res < 2 || std::size_t(res) * res != n) {
      res = 0;
      return;
    }
    double maxx = -1e30, maxy = -1e30;
    minx = miny = 1e30;
    for (const auto &p : g->const_points()) {
      minx = std::min(minx, p[0]);
      maxx = std::max(maxx, p[0]);
      miny = std::min(miny, p[1]);
      maxy = std::max(maxy, p[1]);
    }
    step = (maxx - minx) / (res - 1);
  }
  std::size_t index(double x, double y) const {
    const int i = std::max(0, std::min(res - 1, int(std::lround((x - minx) / step))));
    const int j = std::max(0, std::min(res - 1, int(std::lround((y - miny) / step))));
    return std::size_t(j) * res + i;
  }
  double at(double x, double y) const { return ok() ? geom->const_points()[index(x, y)][2] : 0.0; }
  // The terrain's per-vertex band colour at (x,y) (nearest grid vertex), or white if the mesh has
  // no colours — so the cloud-shadow texture can carry albedo×shadow and keep the ground coloured.
  void color_at(double x, double y, double &r, double &g, double &b) const {
    if (!ok() || geom->const_colors().size() != geom->const_points().size()) {
      r = g = b = 1.0;
      return;
    }
    const auto &c = geom->const_colors()[index(x, y)];
    r = c[0];
    g = c[1];
    b = c[2];
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

  cvc::lsys::forest_params fp;
  fp.seed = static_cast<std::uint64_t>(n.props.num("seed", 1337.0));
  fp.count = static_cast<int>(n.props.num("count", 60.0));
  fp.span = n.props.num("span", 100.0);
  fp.sea_level = n.props.num("sea_level", 0.5);
  const std::string species = n.props.str("species").empty() ? "mix" : n.props.str("species");
  fp.species = species == "pine"      ? cvc::lsys::species_mix::pine
               : species == "branchy" ? cvc::lsys::species_mix::branchy
                                      : cvc::lsys::species_mix::mix;
  fp.scale = n.props.num("scale", 1.0);
  fp.length = n.props.num("length", 6.0);
  fp.radius = n.props.num("radius", 0.7);
  fp.levels = static_cast<int>(n.props.num("levels", 4.0));
  fp.branches = static_cast<int>(n.props.num("branches", 3.0));
  fp.pine_scale = n.props.num("pine_scale", 1.35);
  const double wind = n.props.num("wind", 1.0);

  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  auto heights = std::make_shared<TerrainHeights>();
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights->build(gnode->getGeometry());
  if (!heights->ok())
    warn("ground node '" + ground + "' is not a realized heightfield — planting on a flat plane");
  const cvc::lsys::height_fn height_at = [heights](double x, double y) {
    return heights->ok() ? heights->at(x, y) : 0.0;
  };

  cvc::geometry wood, needle;
  auto windData = std::make_shared<cvc::lsys::forest_wind>();
  const cvc::lsys::forest_result res =
      cvc::lsys::grow_forest(fp, height_at, wood, needle, wind != 0.0 ? windData.get() : nullptr);
  if (wood.const_points().empty()) {
    warn("no trees planted (no dry land above sea_level in the scatter span)");
    return nullptr;
  }

  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, wood) : sg.addGraphics(n.id, wood);
  auto woodNode = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (woodNode)
    woodNode->setUseSingleColor(false); // per-vertex wood colour

  std::shared_ptr<GeometryNode> needleNode;
  if (!needle.const_points().empty()) {
    needleNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id + "_needles", needle));
    if (needleNode) {
      needleNode->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
      needleNode->setUseSingleColor(true);
      needleNode->setColor(C_NEEDLE_R, C_NEEDLE_G, C_NEEDLE_B);
    }
  }

  // Wind: re-pose the pines into the merged buffers each frame (route C — two buffer uploads for
  // the whole forest). World (simulation) time from the scene clock, so pausing the sim freezes the
  // sway. The branchy trees have no sway records and sit untouched at bind pose.
  if (wind != 0.0 && !windData->trees.empty() && woodNode) {
    auto woodBind = std::make_shared<std::vector<double>>(flatten_points(wood));
    auto needleBind = std::make_shared<std::vector<double>>(flatten_points(needle));
    auto workW = std::make_shared<std::vector<double>>(*woodBind);
    auto workN = std::make_shared<std::vector<double>>(*needleBind);
    cvc::app *app = &sg.appContext();
    std::weak_ptr<GeometryNode> ww = woodNode, wn = needleNode;
    out.custom_ticks.push_back([=](vtkRenderer *) {
      auto w = ww.lock();
      if (!w)
        return;
      const double t = app->world_clock().t();
      *workW = *woodBind;
      *workN = *needleBind;
      cvc::lsys::repose_forest(*windData, t, wind, *workW, *workN);
      w->updateVertices(*workW);
      if (auto nn = wn.lock())
        nn->updateVertices(*workN);
    });
  }
  (void)res;
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

  // Terrain height at each sea-grid column (constant), sampled from the realized heightfield.
  TerrainHeights heights;
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights.build(gnode->getGeometry());
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
  out.custom_ticks.push_back([wn, field, terr, sp, seaTF, frame, app](vtkRenderer *) {
    auto v = wn.lock();
    if (!v)
      return;
    const long f = (*frame)++;
    const double t = app->world_clock().t();
    if (f % 2 == 0) {
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

// ───────────────────────────── the sky: a drifting cloud VolumeNode ──────────────────────────────
constexpr int SKY_N = 60, SKY_NZ = 28;
constexpr double SKY_BASE = 74.0, SKY_TOP = 122.0, SKY_HALF = 150.0;
constexpr double CLOUD_DRIFT = 3.0, CLOUD_MORPH_S = 60.0;
constexpr int CLOUD_MAPS = 2, CLOUD_DEPTH = 6;
constexpr double CLOUD_FLOOR = 0.10, CLOUD_EMPTY = 0.22;
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0;
constexpr int SHADOW_RES = 96;
constexpr double SHADOW_PROJ_EL = 66.0, SHADOW_K = 0.13, SHADOW_FLOOR = 0.55;

inline std::size_t skyIdx(int z, int y, int x) { return cvc::lsys::cloud_index(SKY_N, x, y, z); }

// Two L-system cloud maps (cvc::lsys::cloud_field) crossfaded + scrolled at runtime — the cheap
// animation the library leaves to the consumer.
struct SkyModel {
  std::vector<std::vector<float>> maps;
  double norm = 1.0;
  std::vector<float> raw(double shift, double morph) const {
    const int N = SKY_N, NZ = SKY_NZ;
    long mi = static_cast<long>(std::floor(morph));
    int i = static_cast<int>(((mi % CLOUD_MAPS) + CLOUD_MAPS) % CLOUD_MAPS);
    int j = (i + 1) % CLOUD_MAPS;
    double u = morph - std::floor(morph);
    u = u * u * (3.0 - 2.0 * u);
    long ks = static_cast<long>(std::floor(shift));
    double fsh = shift - ks;
    int k0 = static_cast<int>(((ks % N) + N) % N), k1 = (k0 + 1) % N;
    const std::vector<float> &A = maps[i], &B = maps[j];
    std::vector<float> out(static_cast<std::size_t>(NZ) * N * N);
    for (int z = 0; z < NZ; ++z)
      for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
          int sx0 = ((x - k0) % N + N) % N, sx1 = ((x - k1) % N + N) % N;
          auto mix = [&](int xx) {
            return (1.0 - u) * A[skyIdx(z, y, xx)] + u * B[skyIdx(z, y, xx)];
          };
          double vold = (1.0 - fsh) * mix(sx0) + fsh * mix(sx1);
          double lump = std::min(1.0, std::max(0.0, (vold - CLOUD_FLOOR) / (1.0 - CLOUD_FLOOR)));
          out[skyIdx(z, y, x)] = static_cast<float>(lump * lump);
        }
    return out;
  }
  std::vector<float> field(double shift, double morph) const {
    std::vector<float> r = raw(shift, morph);
    const double inv = 1.0 / norm;
    for (float &v : r)
      v = static_cast<float>(v * inv);
    return r;
  }
};
SkyModel buildSky() {
  SkyModel sky;
  for (int m = 0; m < CLOUD_MAPS; ++m) {
    cvc::lsys::cloud_params cp;
    cp.seed = 20;
    cp.variant = static_cast<std::uint32_t>(m);
    cp.n = SKY_N;
    cp.nz = SKY_NZ;
    cp.half = SKY_HALF;
    cp.base = SKY_BASE;
    cp.top = SKY_TOP;
    cp.sun_az = SUN_AZ;
    cp.sun_el = SUN_EL;
    cp.depth = CLOUD_DEPTH;
    sky.maps.push_back(cvc::lsys::cloud_field(cp));
  }
  double norm = 0.0;
  for (int c = 0; c < SKY_N; c += 8)
    for (double mo : {0.0, 0.5, 1.0}) {
      std::vector<float> r = sky.raw(double(c), mo);
      for (float v : r)
        norm = std::max(norm, double(v));
    }
  sky.norm = norm > 0 ? norm : 1.0;
  return sky;
}
void skyTransfer(std::vector<double> &color, std::vector<double> &opacity) {
  color = {0.0, 0.80, 0.85, 0.93, 0.45, 0.94, 0.96, 0.98, 1.0, 1.00, 1.00, 1.00};
  opacity = {0.0, 0.0, CLOUD_EMPTY, 0.0, 0.55, 0.26, 1.0, 0.54};
}

struct Vec3 {
  double x, y, z;
};
Vec3 sunDir(double azDeg, double elDeg) {
  const double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}
float sampleSky(const std::vector<float> &field, double wx, double wy, double wz, double skyHalf) {
  if (wz < SKY_BASE || wz > SKY_TOP)
    return 0.0f;
  double fx = (wx + skyHalf) / (2.0 * skyHalf) * (SKY_N - 1);
  double fy = (wy + skyHalf) / (2.0 * skyHalf) * (SKY_N - 1);
  double fz = (wz - SKY_BASE) / (SKY_TOP - SKY_BASE) * (SKY_NZ - 1);
  if (fx < 0 || fx > SKY_N - 1 || fy < 0 || fy > SKY_N - 1)
    return 0.0f;
  int x0 = (int)fx, y0 = (int)fy, z0 = (int)fz;
  int x1 = std::min(x0 + 1, SKY_N - 1), y1 = std::min(y0 + 1, SKY_N - 1),
      z1 = std::min(z0 + 1, SKY_NZ - 1);
  double tx = fx - x0, ty = fy - y0, tz = fz - z0;
  auto V = [&](int x, int y, int z) { return (double)field[skyIdx(z, y, x)]; };
  double c00 = V(x0, y0, z0) * (1 - tx) + V(x1, y0, z0) * tx;
  double c10 = V(x0, y1, z0) * (1 - tx) + V(x1, y1, z0) * tx;
  double c01 = V(x0, y0, z1) * (1 - tx) + V(x1, y0, z1) * tx;
  double c11 = V(x0, y1, z1) * (1 - tx) + V(x1, y1, z1) * tx;
  double c0 = c00 * (1 - ty) + c10 * ty, c1 = c01 * (1 - ty) + c11 * ty;
  return (float)(c0 * (1 - tz) + c1 * tz);
}
// Bake the grey cloud shadow × terrain albedo over the footprint [-half,half]² into an RGB buffer.
void bakeCloudShadow(const std::vector<float> &field, Vec3 sun, double half,
                     const TerrainHeights &terr, std::vector<unsigned char> &rgb) {
  const double L = std::sqrt(sun.x * sun.x + sun.y * sun.y + sun.z * sun.z);
  Vec3 Ln{sun.x / L, sun.y / L, sun.z / L};
  const int STEPS = 22;
  rgb.resize(static_cast<std::size_t>(SHADOW_RES) * SHADOW_RES * 3);
  for (int ty = 0; ty < SHADOW_RES; ++ty) {
    double y = -half + 2.0 * half * ty / (SHADOW_RES - 1);
    for (int tx = 0; tx < SHADOW_RES; ++tx) {
      double x = -half + 2.0 * half * tx / (SHADOW_RES - 1);
      double z0 = terr.ok() ? terr.at(x, y) : 0.0;
      double tEntry = (SKY_BASE - z0) / Ln.z, tExit = (SKY_TOP - z0) / Ln.z;
      double ds = (tExit - tEntry) / STEPS, tau = 0.0;
      for (int i = 0; i < STEPS; ++i) {
        double t = tEntry + (i + 0.5) * ds;
        tau += sampleSky(field, x + t * Ln.x, y + t * Ln.y, z0 + t * Ln.z, half) * ds;
      }
      double s = SHADOW_FLOOR + (1.0 - SHADOW_FLOOR) * std::exp(-SHADOW_K * tau);
      double ar, ag, ab;
      terr.color_at(x, y, ar, ag, ab); // keep the terrain's band albedo under the shadow
      std::size_t o = (static_cast<std::size_t>(ty) * SHADOW_RES + tx) * 3;
      rgb[o] = (unsigned char)std::min(255.0, std::max(0.0, ar * s * 255.0));
      rgb[o + 1] = (unsigned char)std::min(255.0, std::max(0.0, ag * s * 255.0));
      rgb[o + 2] = (unsigned char)std::min(255.0, std::max(0.0, ab * s * 255.0));
    }
  }
}

// type: cloud_sky — a drifting cloud VolumeNode (two cvc::lsys::cloud_field maps crossfaded +
// scrolled) over the island, plus a baked cloud→ground shadow textured onto the terrain.
// props: ground, half, shadow (0 disables the ground shadow).
std::shared_ptr<GraphicsNode> realize_cloud_sky(SceneGraph &sg, const ari::SceneNode &n,
                                                GraphicsNode *parent, RealizedScene &out,
                                                std::vector<std::string> *warnings) {
  auto sky = std::make_shared<SkyModel>(buildSky());
  cvc::volume vol(sg.appContext(),
                  reinterpret_cast<const unsigned char *>(sky->field(0.0, 0.0).data()),
                  cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
                  cvc::bounding_box(-SKY_HALF, -SKY_HALF, SKY_BASE, SKY_HALF, SKY_HALF, SKY_TOP));
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<VolumeNode>(n.id, vol) : sg.addGraphics(n.id, vol);
  auto vnode = std::dynamic_pointer_cast<VolumeNode>(gn);
  if (!vnode) {
    if (warnings)
      warnings->push_back("ari: cloud_sky '" + n.id + "': could not create a VolumeNode");
    return gn;
  }
  vnode->setShading(false);
  vnode->setAmbient(0.95);
  vnode->setDiffuse(0.35);
  vnode->setSpecular(0.0);
  vnode->setVolumetricScattering(0.0);
  {
    std::vector<double> col, op;
    skyTransfer(col, op);
    vnode->setTransferFunction(col, op);
  }
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  const double half = n.props.num("half", 120.0);
  const bool doShadow = n.props.num("shadow", 1.0) != 0.0;
  auto terr = std::make_shared<TerrainHeights>();
  std::weak_ptr<GeometryNode> wterrain;
  if (doShadow)
    if (auto tnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground))) {
      terr->build(tnode->getGeometry());
      wterrain = tnode;
    }
  auto rgb = std::make_shared<std::vector<unsigned char>>();

  auto frame = std::make_shared<long>(0);
  cvc::app *app = &sg.appContext();
  std::weak_ptr<VolumeNode> wn = vnode;
  out.custom_ticks.push_back([wn, sky, frame, app, terr, wterrain, rgb, half](vtkRenderer *) {
    auto v = wn.lock();
    if (!v)
      return;
    const long f = (*frame)++;
    if (f % 3 != 0) // the 60³ crossfade + shadow bake run on a 3-frame stride (slow drift)
      return;
    const double t = app->world_clock().t();
    const double shift = t * CLOUD_DRIFT * SKY_N / (2.0 * SKY_HALF);
    const double morph = t / CLOUD_MORPH_S * CLOUD_MAPS;
    std::vector<float> field = sky->field(shift, morph);
    v->updateScalars(field);
    if (auto tn = wterrain.lock(); tn && terr->ok()) {
      bakeCloudShadow(field, sunDir(SUN_AZ, SHADOW_PROJ_EL), half, *terr, *rgb);
      tn->setTexture(cvc::image(SHADOW_RES, SHADOW_RES, cvc::image::pixel_format::RGB,
                                cvc::image::data_type::u8, rgb->data()));
    }
  });
  return gn;
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

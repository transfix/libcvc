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

// water_node.cpp — the Ariadne `water` node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §11.2,
// O15): a ground's sea, lakes and ponds drawn as the GPU FFT ocean lsystem_coast drew.
//
//   - { id: water, type: water, world: world, level: 0 }
//
// What it draws, everything below `level` on the ground (a generated_world's height field, or any
// height-field node's, `world:` / `ground:`):
//   - near: a grid `near` metres each way from the camera (a vertex every 2 near / (cells - 1)),
//     displaced by three FFT cascades (OceanFFT, the wind sea's h0 from cvc::ocean::build_h0),
//     each fading out over `shore_fade` metres of depth toward the shore and toward the grid's
//     edge. It moves with the camera by whole cells, so the waves do not swim.
//   - far: a flat grid over the ground's extent with the same shading, for the water past the
//     near grid; it gives way inside the near grid.
// Both shade with lsystem_coast's Fresnel water (sky reflection, sun glint, shallows tinted by
// depth, Jacobian whitecaps with a frayed edge), the sun the renderer's first scene light. Dry
// ground (above the level) discards its fragments. A generated_world under a water node wants
// `water: 0`, so its flat water rungs do not draw over this.
//
// Live keys (water.*, or the node's `keys:`; the water_controls.ari panel): wind_speed (m/s),
// wind_dir (degrees, the way the waves travel, from +x toward +y), fetch (m), chop (the
// cascades' sideways moves), wave_amp (a height scale), foam_bias / foam_ramp / foam_fray (the
// whitecaps' Jacobian threshold, its ramp and the noise on its edge) and shore_fade (m). A change
// to the wind rebuilds the cascades' h0 (tens of milliseconds).
//
// The cascades need float render targets (WebGL2's EXT_color_buffer_float). Without them, or
// before the window is up, the water draws as flat blue.

#include "OceanFFT.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_water.h>           // its physics: WaterRuntime
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/ocean/whitecaps.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

constexpr int kFftN = 256;                            // each cascade's texels a side
constexpr double kTiles[3] = {251.0, 83.0, 29.0};     // lsystem_coast's tiles (non-harmonic)
constexpr std::uint64_t kCascadeSeed = 0xC0A57ull;    // and its seed
constexpr double kDefaultSun[3] = {0.35, 0.25, 0.90}; // when the renderer has no scene light

// ── shaders: lsystem_coast's ocean (examples/cvcgl/lsystem_coast.cpp), without its shore layer,
// wake and boat; the grid's world origin is uCenter (its node's translation) ─────────────────────

// The cascades, sampled where lsystem_coast samples them (texel centres on world positions that
// are whole multiples of tile / n), each faded by the depth under it over uShoreFade and, on the
// near grid, toward its edge.
const char *const kWaterVsDec =
    "//VTK::PositionVC::Dec\n"
    "out vec3 vWaterPos;\n"
    "out vec2 vWaterRest;\n"
    "out vec3 vWaterN;\n"
    "out float vWaterJac;\n"
    "out float vWaterDepth;\n"
    "uniform sampler2D uDisp0; uniform sampler2D uDisp1; uniform sampler2D uDisp2;\n"
    "uniform float uTile0; uniform float uTile1; uniform float uTile2;\n"
    "uniform float uHalfTexel; uniform float uWaveAmp; uniform float uLevel;\n"
    "uniform float uShoreFade; uniform float uWaves; uniform float uNearHalf;\n"
    "uniform vec3 uCenter;\n";
const char *const kWaterVsFuncs =
    "vec4 wDisp(sampler2D t, vec2 w, float tile){ return textureLod(t, w/tile + uHalfTexel, 0.0); "
    "}\n"
    "float wDepth(vec2 w){ return uLevel - cvc_hf_height(w); }\n"
    "vec4 wSum(vec2 w, float k){\n"
    "  vec4 a=wDisp(uDisp0,w,uTile0), b=wDisp(uDisp1,w,uTile1), c=wDisp(uDisp2,w,uTile2);\n"
    "  vec3 j = vec3(1.0) + k*(vec3(a.w,b.w,c.w) - vec3(1.0));\n" // a faded fold folds less
    "  return vec4(k*(a.xyz+b.xyz+c.xyz), min(min(j.x,j.y),j.z));\n"
    "}\n"
    "float wHt(vec2 w, float k){ return "
    "k*(wDisp(uDisp0,w,uTile0).y+wDisp(uDisp1,w,uTile1).y+wDisp(uDisp2,w,uTile2).y); }\n";
const char *const kWaterVsImpl =
    "  vec4 dispMC = vertexMC;\n"
    "  vec2 w = vertexMC.xy + uCenter.xy;\n"
    "  float depth = wDepth(w);\n"
    "  float edge = max(abs(vertexMC.x), abs(vertexMC.y)) / max(uNearHalf, 1e-3);\n"
    "  float k = uWaves * clamp(depth / max(uShoreFade, 1e-3), 0.0, 1.0)\n"
    "            * (1.0 - smoothstep(0.75, 1.0, edge));\n"
    "  vec4 od = wSum(w, k);\n"
    "  float hpx = uWaveAmp*wHt(w+vec2(1.0,0.0), k), hmx = uWaveAmp*wHt(w-vec2(1.0,0.0), k);\n"
    "  float hpy = uWaveAmp*wHt(w+vec2(0.0,1.0), k), hmy = uWaveAmp*wHt(w-vec2(0.0,1.0), k);\n"
    "  vWaterN = normalize(vec3(-(hpx-hmx)*0.5, -(hpy-hmy)*0.5, 1.0));\n"
    "  dispMC.xy += vec2(od.x, od.z);\n"
    "  dispMC.z += uWaveAmp*od.y;\n"
    "  vertexVCVSOutput = MCVCMatrix * dispMC;\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "  vWaterPos = vec3(dispMC.xy + uCenter.xy, dispMC.z);\n"
    "  vWaterRest = w;\n"
    "  vWaterJac = od.w;\n"
    "  vWaterDepth = depth;\n";

const char *const kWaterFsDec =
    "//VTK::Normal::Dec\n"
    "in vec3 vWaterPos;\n"
    "in vec2 vWaterRest;\n"
    "in vec3 vWaterN;\n"
    "in float vWaterJac;\n"
    "in float vWaterDepth;\n"
    "uniform vec3 uCamPos; uniform vec3 uSunDir;\n"
    "uniform float uFoamBias; uniform float uFoamRamp; uniform float uFoamFray;\n"
    "uniform float uLevelF;\n"
    "uniform vec3 uCutMin; uniform vec3 uCutMax;\n"    // the far grid gives way inside the near one
    "uniform vec3 uClipMin; uniform vec3 uClipMax;\n"; // no water past the world, unless its rim is
                                                       // sea
const char *const kWaterFsFuncs =
    "float whash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }\n"
    "float wnoise(vec2 p){\n"
    "  vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  return mix(mix(whash(i),whash(i+vec2(1,0)),f.x),\n"
    "             mix(whash(i+vec2(0,1)),whash(i+vec2(1,1)),f.x), f.y);\n"
    "}\n";
// Consumes //VTK::Light::Impl (VTK's lighting is replaced) and writes the final colour.
const char *const kWaterFsLight =
    // The depth at the fragment, from the bed itself: the far grid's cells are metres across.
    "  float depthF = uLevelF - cvc_hf_height(vWaterRest);\n"
    "  if (depthF < -0.25) discard;\n" // dry ground: the terrain draws there
    "  if (any(lessThan(vWaterRest, uClipMin.xy)) || any(greaterThan(vWaterRest, uClipMax.xy))) "
    "discard;\n"
    "  if (all(greaterThan(vWaterRest, uCutMin.xy)) && all(lessThan(vWaterRest, uCutMax.xy))) "
    "discard;\n"
    "  vec3 N = normalize(vWaterN);\n"
    "  vec3 Vd = normalize(uCamPos - vWaterPos);\n"
    "  float NdotV = max(dot(N,Vd), 0.0);\n"
    "  float Fr = 0.02 + 0.98*pow(1.0-NdotV, 5.0);\n"
    "  float NdotL = max(dot(N, uSunDir), 0.0);\n"
    "  float lit = 0.30 + 0.70*NdotL;\n"
    "  vec3 Rr = 2.0*NdotV*N - Vd;\n"
    "  float skyT = clamp(0.5+0.5*Rr.z, 0.0, 1.0);\n"
    "  float glint = pow(max(dot(Rr, uSunDir), 0.0), 200.0)*1.8;\n"
    "  vec3 deep = vec3(0.05,0.22,0.38)*lit;\n"
    "  vec3 sky = mix(vec3(0.66,0.72,0.78), vec3(0.26,0.50,0.80), skyT);\n"
    "  vec3 col = mix(deep, sky, Fr) + glint*vec3(1.0,0.96,0.88);\n"
    // Shallow water is translucent: the bed shows through, fading with depth (Beer-Lambert).
    "  float clarity = exp(-max(depthF,0.0)*0.20);\n"
    "  vec3 shoal = mix(vec3(0.12,0.46,0.54), vec3(0.66,0.64,0.50), clarity*0.55);\n"
    "  col = mix(col, shoal, clarity*0.85);\n"
    // Whitecaps where the cascades fold (cvc/ocean/whitecaps.h has the threshold's meaning).
    "  float fray = 0.65*wnoise(vWaterRest*1.9) + 0.35*wnoise(vWaterRest*4.3 + vec2(17.0, 31.0));\n"
    "  float foam = clamp((uFoamBias - vWaterJac + uFoamFray*(fray - 0.5))/uFoamRamp, 0.0, 1.0);\n"
    "  foam = foam*foam*(3.0-2.0*foam);\n"
    "  col = mix(col, vec3(0.93,0.96,0.97), foam);\n"
    "  gl_FragData[0] = vec4(col, 1.0);\n";

// ── meshes ───────────────────────────────────────────────────────────────────────────────────

// A flat cells x cells-vertex grid at z, from (x0, y0) to (x1, y1).
cvc::geometry flat_grid(cvc::app &app, int cells, double x0, double y0, double x1, double y1,
                        double z) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  pts.reserve(std::size_t(cells) * std::size_t(cells));
  for (int j = 0; j < cells; ++j) {
    const double y = y0 + (y1 - y0) * j / (cells - 1);
    for (int i = 0; i < cells; ++i) {
      pts.push_back({x0 + (x1 - x0) * i / (cells - 1), y, z});
      cols.push_back({0.05, 0.20, 0.32}); // the flat fallback's colour
    }
  }
  auto &tris = g.tris();
  using idx = cvc::geometry::index_t;
  for (int j = 0; j < cells - 1; ++j)
    for (int i = 0; i < cells - 1; ++i) {
      const idx v = idx(j * cells + i);
      tris.push_back({v, idx(v + 1), idx(v + cells)});
      tris.push_back({idx(v + 1), idx(v + cells + 1), idx(v + cells)});
    }
  return g;
}

// ── runtime ──────────────────────────────────────────────────────────────────────────────────

struct Cascade {
  std::unique_ptr<OceanFFT> fft;
  cvc::ocean::cascade band;
  cvc::ocean::h0_field h0; // what fft was given (the whitecaps' threshold is found from it)
};

struct WaterRT {
  std::weak_ptr<GeometryNode> near, far;
  std::shared_ptr<const cvc::height_field> ground;
  std::unique_ptr<HeightFieldTexture> bed;
  const float *bedData = nullptr; // the ground's heights the bed was made from (regenerates)
  std::size_t bedSize = 0;
  std::vector<Cascade> cascades;
  std::vector<double> lastSea; // the wind keys h0 was built for
  bool ready = false, failed = false;
  double level = 0.0, nearHalf = 192.0, cell = 2.0;
  double cx = 0.0, cy = 0.0; // the near grid's centre (whole cells)
  bool placed = false;
  // The world's edge: when the ground's whole rim is under water (a generated_world's `edge:
  // shore`, an island), the sea goes on `outer` metres past it; otherwise the water stops at the
  // ground's edge (a wall, or the world falling away). Found again after a regenerate.
  double outer = 6000.0;
  int farCells = 160;
  bool rimSea = false, rimKnown = false;
  std::array<double, 4> farExtent{0.0, 0.0, 0.0, 0.0}; // the far grid's x0, y0, x1, y1
  // The whitecaps' threshold for Monahan's share (cvc/ocean/whitecaps.h), found again once the
  // keys it depends on have held for kFoamSettle frames.
  double foamBias = 0.8;
  std::vector<double> foamSig, foamPending;
  int foamSettled = 0;
  // the keys' resolved paths
  std::string windKey, dirKey, fetchKey, chopKey, ampKey, biasKey, rampKey, frayKey, fadeKey;
  std::string capsKey;
  // With `body:`, the water in the scene's physics world (scene_water.h): one fluid field over the
  // ground, its surface the sea drawn here, rebuilt as the wind or the ground changes.
  std::unique_ptr<cvc::ariadne::WaterRuntime> physics;
};

constexpr int kFoamSettle = 10;

double key(cvc::app &app, const std::string &path, double def) {
  return ari::read_or<double>(app, path, def);
}

std::string seed(cvc::app &app, const std::string &prefix, const ari::SceneNode &n, const char *k,
                 double def) {
  const std::string path = ari::resolve_bind(prefix, ari::node_key(n, k));
  ari::read_or_seed<double>(app, path, def);
  return path;
}

cvc::ocean::sea_state sea_state_of(cvc::app &app, const WaterRT &rt) {
  cvc::ocean::sea_state s;
  s.wind.speed_mps = std::max(key(app, rt.windKey, 9.0), 0.0);
  s.wind.fetch_m = std::max(key(app, rt.fetchKey, 120000.0), 400.0);
  s.wind.dir_rad = key(app, rt.dirKey, 30.0) * 3.14159265358979323846 / 180.0;
  s.swell.hs_m = 0.0; // the wind sea alone (a shore layer would carry a swell)
  return s;
}

// The renderer's first switched-on scene light, as the direction toward it.
std::array<double, 3> sun_direction(vtkRenderer *ren) {
  std::array<double, 3> d{kDefaultSun[0], kDefaultSun[1], kDefaultSun[2]};
  if (!ren || !ren->GetLights())
    return d;
  vtkLightCollection *lights = ren->GetLights();
  lights->InitTraversal();
  while (vtkLight *l = lights->GetNextItem()) {
    if (!l->GetSwitch() || !l->LightTypeIsSceneLight() || l->GetPositional())
      continue;
    double p[3], f[3];
    l->GetPosition(p);
    l->GetFocalPoint(f);
    const double v[3] = {p[0] - f[0], p[1] - f[1], p[2] - f[2]};
    const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 1e-9)
      return {v[0] / len, v[1] / len, v[2] / len};
  }
  return d;
}

void style_flat(GeometryNode &g) {
  g.setUseSingleColor(true);
  g.setColor(0.05, 0.20, 0.32);
  g.setAmbient(0.6);
  g.setDiffuse(0.5);
  g.setSpecular(0.5);
  g.setSpecularPower(60.0);
}

// The shaders, the cascades' textures and the bed on one of the water's meshes.
void style_ocean(GeometryNode &g, const WaterRT &rt, vtkTextureObject *bed) {
  const char *disp[3] = {"uDisp0", "uDisp1", "uDisp2"};
  const char *tile[3] = {"uTile0", "uTile1", "uTile2"};
  for (std::size_t i = 0; i < rt.cascades.size() && i < 3; ++i) {
    g.setShaderTexture(disp[i], rt.cascades[i].fft->displacement());
    g.setShaderUniformf(tile[i], static_cast<float>(rt.cascades[i].band.tile_m));
  }
  g.setShaderUniformf("uHalfTexel", 0.5f / static_cast<float>(kFftN));
  g.setShaderUniformf("uLevel", static_cast<float>(rt.level));
  g.setShaderUniformf("uLevelF", static_cast<float>(rt.level));
  g.setShaderUniformf("uNearHalf", static_cast<float>(rt.nearHalf));
  g.setShaderTexture(HeightFieldTexture::samplerName(), bed);
  g.setShaderUniform3f(HeightFieldTexture::originName(), static_cast<float>(rt.bed->x0()),
                       static_cast<float>(rt.bed->y0()), 0.0f);
  g.setShaderUniform3f(HeightFieldTexture::spacingName(), static_cast<float>(rt.bed->dx()),
                       static_cast<float>(rt.bed->dy()), 0.0f);
  g.addVertexShaderReplacement("//VTK::PositionVC::Dec", std::string(kWaterVsDec) +
                                                             HeightFieldTexture::glsl() +
                                                             kWaterVsFuncs);
  g.addVertexShaderReplacement("//VTK::PositionVC::Impl", kWaterVsImpl);
  g.addFragmentShaderReplacement(
      "//VTK::Normal::Dec", std::string(kWaterFsDec) + HeightFieldTexture::glsl() + kWaterFsFuncs);
  g.addFragmentShaderReplacement("//VTK::Light::Impl", kWaterFsLight);
}

// Make the bed texture's heights the ground's (again after a regenerate); true when they changed.
bool refresh_bed(WaterRT &rt) {
  const cvc::height_field &hf = *rt.ground;
  const std::vector<float> &h = hf.heights();
  if (h.data() == rt.bedData && h.size() == rt.bedSize && rt.bed)
    return false;
  if (!rt.bed || rt.bed->nx() != hf.cols() || rt.bed->ny() != hf.rows() ||
      rt.bed->x0() != hf.x0() || rt.bed->y0() != hf.y0() || rt.bed->dx() != hf.dx() ||
      rt.bed->dy() != hf.dy())
    rt.bed = std::make_unique<HeightFieldTexture>(hf.cols(), hf.rows(), hf.x0(), hf.y0(), hf.dx(),
                                                  hf.dy());
  rt.bed->setHeights(h);
  rt.bedData = h.data();
  rt.bedSize = h.size();
  return true;
}

// Whether the ground's rim is under water all round (98 % of its samples, so a stray rock at the
// edge does not cut the sea off).
bool rim_is_sea(const cvc::height_field &hf, double level) {
  const int rows = hf.rows(), cols = hf.cols();
  std::size_t wet = 0, all = 0;
  auto count = [&](int r, int c) {
    ++all;
    if (hf.height_at(r, c) < level)
      ++wet;
  };
  for (int c = 0; c < cols; ++c) {
    count(0, c);
    count(rows - 1, c);
  }
  for (int r = 1; r < rows - 1; ++r) {
    count(r, 0);
    count(r, cols - 1);
  }
  return all > 0 && wet >= std::size_t(0.98 * double(all));
}

// The far grid over the ground, and `outer` metres past it when the rim is sea; the water clipped
// to the ground's rectangle otherwise. When the rim changed (or the first time).
void place_far(WaterRT &rt, GeometryNode &far, GeometryNode &near) {
  const cvc::height_field &hf = *rt.ground;
  const bool sea = rim_is_sea(hf, rt.level);
  const double m = sea ? rt.outer : 0.0;
  const double x0 = hf.x0() - m, y0 = hf.y0() - m, x1 = hf.max_x() + m, y1 = hf.max_y() + m;
  const std::array<double, 4> extent{x0, y0, x1, y1};
  if (rt.rimKnown && sea == rt.rimSea && extent == rt.farExtent)
    return;
  const bool first = !rt.rimKnown;
  rt.rimSea = sea;
  rt.rimKnown = true;
  rt.farExtent = extent;
  if (!first) {
    const int n = rt.farCells;
    std::vector<double> xyz;
    xyz.reserve(std::size_t(n) * std::size_t(n) * 3);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        xyz.push_back(x0 + (x1 - x0) * i / (n - 1));
        xyz.push_back(y0 + (y1 - y0) * j / (n - 1));
        xyz.push_back(rt.level);
      }
    far.updateVertices(xyz);
  }
  const float big = 1e30f;
  for (GeometryNode *g : {&far, &near}) {
    if (sea) {
      g->setShaderUniform3f("uClipMin", -big, -big, 0.0f);
      g->setShaderUniform3f("uClipMax", big, big, 0.0f);
    } else {
      g->setShaderUniform3f("uClipMin", static_cast<float>(hf.x0()), static_cast<float>(hf.y0()),
                            0.0f);
      g->setShaderUniform3f("uClipMax", static_cast<float>(hf.max_x()),
                            static_cast<float>(hf.max_y()), 0.0f);
    }
  }
}

// The cascades and their textures, once the window is up; false when float targets are missing.
bool start_ocean(cvc::app &app, WaterRT &rt, vtkOpenGLRenderWindow *ow) {
  const cvc::ocean::sea_state s = sea_state_of(app, rt);
  const std::vector<cvc::ocean::cascade> bands =
      cvc::ocean::make_cascades({kTiles[0], kTiles[1], kTiles[2]}, kCascadeSeed);
  for (const cvc::ocean::cascade &b : bands) {
    Cascade c;
    c.band = b;
    c.fft = std::make_unique<OceanFFT>(kFftN);
    c.fft->depth_m = 1000.0f;
    c.fft->chop = static_cast<float>(key(app, rt.chopKey, 1.15));
    if (!c.fft->init(ow))
      return false;
    c.h0 = cvc::ocean::build_h0(kFftN, c.band, s);
    c.fft->setH0(c.h0);
    rt.cascades.push_back(std::move(c));
  }
  rt.lastSea = {s.wind.speed_mps, s.wind.fetch_m, s.wind.dir_rad};
  return true;
}

// What the whitecaps' threshold depends on: the wind (through h0), the chop, the ramp, the fray
// and the share of Monahan's cover asked for.
std::vector<double> foam_signature(cvc::app &app, const WaterRT &rt) {
  const cvc::ocean::sea_state s = sea_state_of(app, rt);
  return {s.wind.speed_mps,
          s.wind.fetch_m,
          s.wind.dir_rad,
          key(app, rt.chopKey, 1.15),
          key(app, rt.rampKey, 0.1),
          key(app, rt.frayKey, 0.12),
          key(app, rt.capsKey, 1.0)};
}

// The threshold that makes the open water's foam cover Monahan's share for the wind (times
// water.whitecaps), on the near grid's vertices (cvc/ocean/whitecaps.h; about 0.1-0.25 s).
double find_foam_bias(cvc::app &app, const WaterRT &rt) {
  std::vector<cvc::ocean::h0_field> hs;
  for (const Cascade &c : rt.cascades)
    hs.push_back(c.h0);
  cvc::ocean::foam_params p;
  p.chop = key(app, rt.chopKey, 1.15);
  p.ramp = std::max(key(app, rt.rampKey, 0.1), 1e-3);
  p.fray = std::max(key(app, rt.frayKey, 0.12), 0.0);
  p.mesh_m = rt.cell;
  const double share = std::max(key(app, rt.capsKey, 1.0), 0.0) *
                       cvc::ocean::monahan_coverage(sea_state_of(app, rt).wind.speed_mps);
  return cvc::ocean::foam_field(hs, p).bias_for(share);
}

void tick_water(cvc::app &app, WaterRT &rt, vtkRenderer *ren) {
  auto near = rt.near.lock();
  auto far = rt.far.lock();
  if (!near || !far)
    return;
  if (rt.physics)
    rt.physics->tick(); // the field follows the keys and the ground (a regenerate)
  if (!rt.ready && !rt.failed) {
    auto *ow = ren ? vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()) : nullptr;
    if (!ow || !ow->GetInitialized())
      return;
    refresh_bed(rt);
    vtkTextureObject *bed = rt.bed ? rt.bed->prepare(ow) : nullptr;
    if (!bed || !start_ocean(app, rt, ow)) {
      rt.failed = true; // no float render targets: the far grid's flat blue stays, alone
      rt.cascades.clear();
      return;
    }
    style_ocean(*near, rt, bed);
    style_ocean(*far, rt, bed);
    far->setShaderUniform3f("uCenter", 0.0f, 0.0f, 0.0f);
    far->setShaderUniformf("uWaves", 0.0f); // flat: its cells are metres across
    near->setShaderUniformf("uWaves", 1.0f);
    near->setShaderUniform3f("uCutMin", 1e30f, 1e30f, 0.0f); // the near grid gives way to none
    near->setShaderUniform3f("uCutMax", -1e30f, -1e30f, 0.0f);
    near->setVisible(true);
    far->setVisible(true);
    rt.foamBias = find_foam_bias(app, rt);
    rt.foamSig = rt.foamPending = foam_signature(app, rt);
    rt.ready = true;
  }
  if (!rt.ready)
    return;

  // A regenerated world: the bed again.
  if (refresh_bed(rt)) {
    auto *ow = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
    if (vtkTextureObject *bed = ow ? rt.bed->prepare(ow) : nullptr)
      for (GeometryNode *g : {near.get(), far.get()}) {
        g->setShaderTexture(HeightFieldTexture::samplerName(), bed);
        g->setShaderUniform3f(HeightFieldTexture::originName(), static_cast<float>(rt.bed->x0()),
                              static_cast<float>(rt.bed->y0()), 0.0f);
        g->setShaderUniform3f(HeightFieldTexture::spacingName(), static_cast<float>(rt.bed->dx()),
                              static_cast<float>(rt.bed->dy()), 0.0f);
      }
    place_far(rt, *far, *near); // the edge may have changed (world.edge), or the world's size
  }

  // The wind: h0 again when it changed; the cascades to the world's time.
  const cvc::ocean::sea_state s = sea_state_of(app, rt);
  const std::vector<double> sea{s.wind.speed_mps, s.wind.fetch_m, s.wind.dir_rad};
  const bool rebuild = sea != rt.lastSea;
  const float chop = static_cast<float>(key(app, rt.chopKey, 1.15));
  const double t = app.world_clock().t();
  for (Cascade &c : rt.cascades) {
    if (rebuild) {
      c.h0 = cvc::ocean::build_h0(kFftN, c.band, s);
      c.fft->setH0(c.h0);
    }
    c.fft->chop = chop;
    c.fft->step(t);
  }
  if (rebuild)
    rt.lastSea = sea;
  // The whitecaps' threshold, once its keys have settled (a slider drag does not stall each frame).
  std::vector<double> foamNow = foam_signature(app, rt);
  if (foamNow != rt.foamSig) {
    if (foamNow != rt.foamPending) {
      rt.foamPending = std::move(foamNow);
      rt.foamSettled = 0;
    } else if (++rt.foamSettled >= kFoamSettle) {
      rt.foamBias = find_foam_bias(app, rt);
      rt.foamSig = rt.foamPending;
    }
  }

  // The near grid follows the camera by whole cells.
  double cam[3] = {0.0, 0.0, 0.0};
  if (vtkCamera *c = ren->GetActiveCamera())
    c->GetPosition(cam);
  const double qx = std::round(cam[0] / rt.cell) * rt.cell,
               qy = std::round(cam[1] / rt.cell) * rt.cell;
  if (!rt.placed || std::abs(qx - rt.cx) > 0.25 * rt.nearHalf ||
      std::abs(qy - rt.cy) > 0.25 * rt.nearHalf) {
    rt.cx = qx;
    rt.cy = qy;
    rt.placed = true;
    near->setPosition(rt.cx, rt.cy, 0.0);
    near->setShaderUniform3f("uCenter", static_cast<float>(rt.cx), static_cast<float>(rt.cy), 0.0f);
    // the far grid gives way just inside the near grid's edge, where its waves have faded out
    const float h = static_cast<float>(0.97 * rt.nearHalf);
    far->setShaderUniform3f("uCutMin", static_cast<float>(rt.cx) - h, static_cast<float>(rt.cy) - h,
                            0.0f);
    far->setShaderUniform3f("uCutMax", static_cast<float>(rt.cx) + h, static_cast<float>(rt.cy) + h,
                            0.0f);
  }

  const std::array<double, 3> sun = sun_direction(ren);
  const float amp = static_cast<float>(key(app, rt.ampKey, 1.0));
  const double fixedBias = key(app, rt.biasKey, -1.0); // >= 0 fixes the threshold by hand
  const float bias = static_cast<float>(fixedBias >= 0.0 ? fixedBias : rt.foamBias);
  const float ramp = static_cast<float>(std::max(key(app, rt.rampKey, 0.1), 1e-3));
  const float fray = static_cast<float>(std::max(key(app, rt.frayKey, 0.12), 0.0));
  const float fade = static_cast<float>(std::max(key(app, rt.fadeKey, 3.0), 1e-3));
  for (GeometryNode *g : {near.get(), far.get()}) {
    g->setShaderUniform3f("uCamPos", static_cast<float>(cam[0]), static_cast<float>(cam[1]),
                          static_cast<float>(cam[2]));
    g->setShaderUniform3f("uSunDir", static_cast<float>(sun[0]), static_cast<float>(sun[1]),
                          static_cast<float>(sun[2]));
    g->setShaderUniformf("uWaveAmp", amp);
    g->setShaderUniformf("uFoamBias", bias);
    g->setShaderUniformf("uFoamRamp", ramp);
    g->setShaderUniformf("uFoamFray", fray);
    g->setShaderUniformf("uShoreFade", fade);
  }
}

std::shared_ptr<GraphicsNode> realize_water(SceneGraph &sg, const ari::SceneNode &n,
                                            GraphicsNode *parent, RealizedScene &out,
                                            std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> group = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  group->setContributesToSceneBounds(false); // the ground frames the view (see the meshes below)
  const std::string world = n.props.str("world");
  const std::string groundId = n.props.str("ground").empty() ? world : n.props.str("ground");
  auto hf = out.height_fields.find(groundId);
  if (groundId.empty() || hf == out.height_fields.end() || !hf->second || hf->second->rows() < 2 ||
      hf->second->cols() < 2) {
    if (warnings)
      warnings->push_back("ari: water '" + n.id + "': no height field '" + groundId +
                          "' (world: or ground: names a generated_world or height-field node)");
    return group;
  }
  cvc::app &app = sg.appContext();
  auto rt = std::make_shared<WaterRT>();
  rt->ground = hf->second;
  rt->level = n.props.num("level", 0.0);
  rt->nearHalf = std::max(n.props.num("near", 192.0), 16.0);
  const int cells = std::clamp(static_cast<int>(n.props.num("cells", 192.0)), 16, 512);
  rt->cell = 2.0 * rt->nearHalf / (cells - 1);

  const std::string pref = sg.getStatePrefix();
  rt->windKey = seed(app, pref, n, "water.wind_speed", n.props.num("wind_speed", 9.0));
  rt->dirKey = seed(app, pref, n, "water.wind_dir", n.props.num("wind_dir", 30.0));
  rt->fetchKey = seed(app, pref, n, "water.fetch", n.props.num("fetch", 120000.0));
  rt->chopKey = seed(app, pref, n, "water.chop", n.props.num("chop", 1.15));
  rt->ampKey = seed(app, pref, n, "water.wave_amp", n.props.num("wave_amp", 1.0));
  rt->biasKey = seed(app, pref, n, "water.foam_bias", n.props.num("foam_bias", -1.0));
  rt->capsKey = seed(app, pref, n, "water.whitecaps", n.props.num("whitecaps", 1.0));
  rt->rampKey = seed(app, pref, n, "water.foam_ramp", n.props.num("foam_ramp", 0.1));
  rt->frayKey = seed(app, pref, n, "water.foam_fray", n.props.num("foam_fray", 0.12));
  rt->fadeKey = seed(app, pref, n, "water.shore_fade", n.props.num("shore_fade", 3.0));

  const cvc::height_field &g = *rt->ground;
  rt->outer = std::max(n.props.num("outer", 6000.0), 0.0);
  rt->farCells = std::clamp(static_cast<int>(n.props.num("far_cells", 160.0)), 2, 1024);
  const double m = rim_is_sea(g, rt->level) ? rt->outer : 0.0;
  auto far = std::dynamic_pointer_cast<GeometryNode>(
      group->createChild<GeometryNode>("far", flat_grid(app, rt->farCells, g.x0() - m, g.y0() - m,
                                                        g.max_x() + m, g.max_y() + m, rt->level)));
  auto near = std::dynamic_pointer_cast<GeometryNode>(
      group->createChild<GeometryNode>("near", flat_grid(app, cells, -rt->nearHalf, -rt->nearHalf,
                                                         rt->nearHalf, rt->nearHalf, rt->level)));
  if (!far || !near) {
    if (warnings)
      warnings->push_back("ari: water '" + n.id + "': could not create its meshes");
    return group;
  }
  for (GeometryNode *m : {far.get(), near.get()}) {
    style_flat(*m);
    m->setCastsShadow(false);
    m->setForceOpaque(true);
    // The ground frames the view, not the water: the sea past a shore-edged world runs `outer`
    // metres out, and the near grid moves with the camera. They stay in VTK's bounds, so the
    // clipping range holds them.
    m->setContributesToSceneBounds(false);
    // vertexMC in the shaders is the mesh's own coordinates (world xy for the far grid), from the
    // first upload on: set before the mesh ever draws.
    m->disableCoordinateShiftScale();
  }
  near->setVisible(false); // shown once the cascades are up (or the flat fallback is chosen)
  rt->near = near;
  rt->far = far;
  place_far(*rt, *far, *near); // the clip for the edge the grid was just built for

  // `body:`: the water's field in the scene's physics world, over the same ground (whose shared
  // height field a regenerated world overwrites in place: the runtime sees its new heights).
  if (n.body.present) {
    std::shared_ptr<const cvc::height_field> ground = rt->ground;
    rt->physics =
        std::make_unique<cvc::ariadne::WaterRuntime>(app, n, pref, [ground]() { return ground; });
    out.colliders[n.id] = rt->physics->colliders();
  }

  cvc::app *ap = &app;
  out.custom_ticks.push_back([ap, rt](vtkRenderer *ren) { tick_water(*ap, *rt, ren); });
  return group;
}

} // namespace

void register_water_node_types(cvc::app &app) {
  register_scene_node_type(app, "water", realize_water);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

// lsystem_coast — a pure-C++ cvcGL island + GPU FFT-ocean demo. It is the full
// lsystem_forest island scene (heightfield terrain, L-system forest, drifting fBm
// clouds, camera-relative sun, StageLighting rig, shadows, CameraController + ImGui
// controls) with the crude wave-volume sea REPLACED by a real spectral ocean: the
// ABYSSAL-style GPU FFT ocean core (OceanFFT), the port target from
// docs/roadmap/OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md. lsystem_forest stays the
// untouched performance control (Lab roadmap §1.3); coast is where the water lives.
//
// NO SINGLETON: owns an explicit cvc::app and injects it into the scene, so the
// whole thing runs under one app the caller controls — logging, state and the
// state publisher all flow through it (cvcGL has no process-wide context).
//
// The ocean: 3 JONSWAP cascades (swell + wind-waves + ripples) run as a fragment-
// shader butterfly IFFT (OceanFFT, WebGL2-portable — no compute shaders), summed on
// a STATIC grid whose vertex shader samples the live displacement textures and whose
// fragment shader runs a Fresnel water BRDF: sky reflection, sun glint, Jacobian
// (wave-fold) foam, a depth-tinted shallow band, and animated SHORE FOAM that hugs
// the waterline and washes up/down the beach as the swell rises and falls. Driven by
// cvc::world_clock; every lever lives in the state tree at coast.water.* / coast.clock.*
// (scriptable from Python and live in the ImGui "Water" panel). No per-frame CPU
// readback / updateVertices — the WASM-viable path (GeometryNode::setShaderTexture).
//
// The rest of the island (verbatim from lsystem_forest): a heightfield terrain
// (matte + procedural fragment bump map), an L-system FOREST on the dry land (merged
// route-C to one wood + one needle actor, wind re-posed each frame), a drifting SKY
// of L-system + fBm clouds (casts a soft ground dapple), a gradient sky with a
// camera-relative sun, a StageLighting rig, and shadows.
//
// Run (onscreen, navigable):   lsystem_coast
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer;
//   F toggles the FPS HUD (state coast.viewers.main.hud.*).
// Verify (offscreen, headless): lsystem_coast --offscreen --frames 30 --png out.png
// Cinematic capture (mp4-ready): lsystem_coast --capture fly --frames 1800 --fps 30 \
//   --width 1920 --height 1080 --out frames   (also --capture orbit; then encode `frames/`)

#define _USE_MATH_DEFINES // MSVC: make <cmath> define M_PI (and friends)
#include "OceanFFT.h"
#include "demos_menu.h" // the gallery's Demos menu

#include <algorithm>
#include <boost/program_options.hpp>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/shore.h>
#include <cvc/state/state.h>
#include <cvc/world/grid.h>
#include <cvc/world/water_bodies.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/image/image.h>
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h> // SceneGraph.h only forward-declares it
#include <emscripten.h>
#endif
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextureObject.h>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::VolumeNode;

namespace {

constexpr double HALF = 120.0; // terrain spans [-HALF, HALF]
constexpr int TN = 96;         // heightfield resolution
constexpr double PEAK = 34.0, SHELF = -9.0, SEA_LEVEL = 0.0;

// Island heightfield: a central dome that drops below sea level at the rim, plus
// two octaves of relief (matches the Python demo's base terrain before patches).
double terrainH(double x, double y) {
  double r2 = x * x + y * y;
  double h = PEAK * std::exp(-r2 / (0.34 * HALF * HALF)) + SHELF;
  h += 4.5 * std::sin(x * 0.045) * std::cos(y * 0.041);
  h += 2.2 * std::sin(x * 0.11 + 1.3) * std::sin(y * 0.097);
  return h;
}

// Base terrain albedo at (x,y): grass, blending to rock on the peaks and sand at
// the waterline. Shared by the mesh colours AND the cloud-shadow texture (which
// carries albedo × shadow, so it reads correctly whether VTK modulates or replaces
// the vertex colour with the texture).
cvc::geometry::color_t terrainAlbedo(double x, double y) {
  double h = terrainH(x, y);
  const double rock[3] = {0.46, 0.45, 0.43}, grass[3] = {0.27, 0.44, 0.19},
               sand[3] = {0.68, 0.62, 0.44};
  double rockw = std::min(1.0, std::max(0.0, (h - 18.0) / 14.0));
  double shore = std::max(0.0, 1.0 - std::fabs(h - SEA_LEVEL) / 4.5);
  cvc::geometry::color_t c;
  for (int k = 0; k < 3; ++k)
    c[k] = (grass[k] * (1.0 - rockw) + rock[k] * rockw) * (1.0 - shore) + sand[k] * shore;
  return c;
}

cvc::geometry buildTerrain(cvc::app &app) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &uvs = g.uvs(); // world (x,y) -> [0,1]^2, so the cloud-shadow texture aligns
  for (int j = 0; j < TN; ++j) {
    double y = -HALF + 2.0 * HALF * j / (TN - 1);
    for (int i = 0; i < TN; ++i) {
      double x = -HALF + 2.0 * HALF * i / (TN - 1);
      double h = terrainH(x, y);
      pts.push_back({x, y, h});
      uvs.push_back({(x + HALF) / (2.0 * HALF), (y + HALF) / (2.0 * HALF)});
      cols.push_back(terrainAlbedo(x, y));
    }
  }
  auto &tris = g.tris();
  for (int j = 0; j < TN - 1; ++j)
    for (int i = 0; i < TN - 1; ++i) {
      cvc::geometry::index_t v = j * TN + i;
      tris.push_back({v, v + 1, static_cast<cvc::geometry::index_t>(v + TN)});
      tris.push_back({static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + TN + 1),
                      static_cast<cvc::geometry::index_t>(v + TN)});
    }
  return g;
}

// The writable fragment-normal at the //VTK::Normal::Impl anchor differs by
// mapper: desktop vtkOpenGLPolyDataMapper declares a local `normalVCVSOutput`
// shadowing the varying; the GLES3 mapper (used under Emscripten/WebGL2)
// instead declares `normalizedNormalVCVSOutput` and never shadows the varying,
// so assigning `normalVCVSOutput` there is an ESSL 'can't modify an input'
// error. Target the variant's actual local; desktop GLSL is unchanged.
#ifdef __EMSCRIPTEN__
#define CVC_FS_NORMAL "normalizedNormalVCVSOutput"
#else
#define CVC_FS_NORMAL "normalVCVSOutput"
#endif

// Procedural fragment bump map for the ground (verbatim GLSL from the Python demo:
// value-noise height, normal perturbed by its surface gradient — Mikkelsen's
// tangent-free method). Needs world-space vertexMC, hence disableCoordinateShiftScale.
const char *GROUND_GLSL =
    "float ghash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float gnoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  float a=ghash(i), b=ghash(i+vec2(1.,0.)), c=ghash(i+vec2(0.,1.)), d=ghash(i+vec2(1.,1.));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y);\n"
    "}\n"
    "float groundH(vec3 p){\n"
    "  vec2 q = p.xy * 0.35;\n"
    "  float f = 0.0, a = 0.5, fr = 1.0;\n"
    "  for (int i = 0; i < 5; i++){ f += a*gnoise(q*fr); a *= 0.5; fr *= 2.03; }\n"
    "  return f;\n"
    "}\n";

void addTerrainBump(GeometryNode &node) {
  node.disableCoordinateShiftScale(); // vertexMC in the shader becomes world xy
  node.addVertexShaderReplacement("//VTK::Normal::Dec", "//VTK::Normal::Dec\nout vec3 gCoord;");
  node.addVertexShaderReplacement("//VTK::PositionVC::Impl",
                                  "//VTK::PositionVC::Impl\n  gCoord = vertexMC.xyz;");
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Dec", std::string("//VTK::Normal::Dec\nin vec3 gCoord;\n") + GROUND_GLSL);
  node.addFragmentShaderReplacement("//VTK::Normal::Impl",
                                    "//VTK::Normal::Impl\n"
                                    "  {\n"
                                    "    float h = groundH(gCoord);\n"
                                    "    vec3 sS = dFdx(vertexVC.xyz);\n"
                                    "    vec3 sT = dFdy(vertexVC.xyz);\n"
                                    "    vec3 vn = " CVC_FS_NORMAL ";\n"
                                    "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
                                    "    float det = dot(sS, R1);\n"
                                    "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
                                    "    " CVC_FS_NORMAL " = normalize(abs(det)*vn - 1.4*sg);\n"
                                    "  }\n");
}

// ── the swash on the beach (OCEAN-AND-WEATHER-ROADMAP.md A3, decision D4) ─────
// Drawn in the terrain's fragment shader, not as geometry: no z-fighting with the sand,
// on every LOD rung. Each wave's front runs up the sand to Hunt's runup (the shore
// layer's runup field, carried up the beach from the waterline) and drains back; under
// it a thin film of water with foam at its edge; behind it the sand stays dark and
// glossy, drying over coast.water.dry_time seconds. The arithmetic is
// cvc::ocean::shore_layer::swash's, so the CPU knows where it is wet. It reads the shore
// layer's two textures, and runs after VTK's texture (albedo x cloud shade) is applied.
const char *SWASH_FS_DEC =
    "//VTK::Light::Dec\n"
    "uniform highp sampler2D uSwashWave; uniform highp sampler2D uSwashDir;\n"
    "uniform vec3 uSwashOrigin; uniform vec3 uSwashSpacing;\n"
    "uniform float uSwashOmega; uniform float uSwashTime; uniform float uSwashPhase0;\n"
    "uniform float uDryTime; uniform float uSwashLevel; uniform float uSwashFoam;\n"
    "vec4 swGrid4(highp sampler2D tex, vec2 p){\n"
    "  ivec2 sz = textureSize(tex, 0);\n"
    "  vec2 f = (p - uSwashOrigin.xy) / uSwashSpacing.xy;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec4 a = mix(texelFetch(tex, i0, 0), texelFetch(tex, ivec2(i1.x, i0.y), 0), fr.x);\n"
    "  vec4 b = mix(texelFetch(tex, ivec2(i0.x, i1.y), 0), texelFetch(tex, i1, 0), fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    // (film, lace, wet) of the ground at p
    "vec3 oSwash(vec3 p){\n"
    "  float z = p.z - uSwashLevel;\n"
    "  if (z <= 0.0) return vec3(0.0, 0.0, 1.0);\n"
    "  float R = swGrid4(uSwashDir, p.xy).z;\n"
    "  if (R <= 0.0 || z >= R) return vec3(0.0);\n"
    "  float th = uSwashOmega*uSwashTime - swGrid4(uSwashWave, p.xy).x + uSwashPhase0;\n"
    "  float front = R*(0.5 + 0.5*cos(th));\n"
    "  float alpha = acos(clamp(2.0*z/R - 1.0, -1.0, 1.0));\n"
    "  float w = th - 6.2831853*floor(th/6.2831853 + 0.5);\n"
    "  if (abs(w) <= alpha) {\n"
    "    float film = clamp((front - z)/max(0.15*R, 0.05), 0.0, 1.0);\n"
    "    float lace = 1.0 - smoothstep(0.0, max(0.12*R, 0.03), front - z);\n"
    "    return vec3(film, lace, 1.0);\n"
    "  }\n"
    "  float since = w - alpha; if (since < 0.0) since += 6.2831853;\n"
    "  return vec3(0.0, 0.0, exp(-since/uSwashOmega/max(uDryTime, 1e-3)));\n"
    "}\n";
const char *SWASH_FS_IMPL =
    "//VTK::TCoord::Impl\n"
    "  {\n"
    "    vec3 sw = oSwash(gCoord);\n"
    "    vec3 c = gl_FragData[0].rgb;\n"
    "    c *= mix(1.0, 0.62, sw.z);\n" // wet sand is darker
    "    vec3 Vd = normalize(-vertexVC.xyz);\n"
    "    float rim = pow(1.0 - max(dot(normalize(" CVC_FS_NORMAL "), Vd), 0.0), 4.0);\n"
    "    c = mix(c, vec3(0.70, 0.78, 0.86), (0.18 + 0.4*rim)*sw.z);\n" // and glossy
    "    c = mix(c, vec3(0.42, 0.58, 0.64), 0.5*sw.x);\n"              // the film's water
    "    c = mix(c, vec3(0.95, 0.97, 0.98), clamp(0.9*sw.y*uSwashFoam, 0.0, 1.0));\n" // its edge
    "    gl_FragData[0].rgb = c;\n"
    "  }\n";

void addTerrainSwash(GeometryNode &node) {
  node.addFragmentShaderReplacement("//VTK::Light::Dec", SWASH_FS_DEC);
  node.addFragmentShaderReplacement("//VTK::TCoord::Impl", SWASH_FS_IMPL);
}

// ── L-system trees (a faithful C++ port of the Python demo's tree grammar) ────
// Each tree is grown from the grammar into a module hierarchy, merged into ONE
// wood mesh + ONE needle mesh (route C), and re-posed every frame by re-running
// the wind cascade over the merged vertices (GeometryNode::updateVertices) — one
// actor per mesh, wind intact. Procedural bark rides on the wood normals.

struct Vec3d {
  double x = 0, y = 0, z = 0;
};
struct Mat4 { // row-major 4x4, as GraphicsNode/setTransform takes
  double m[16];
  double &at(int r, int c) { return m[r * 4 + c]; }
  double at(int r, int c) const { return m[r * 4 + c]; }
};
Mat4 mIdent() {
  Mat4 M{};
  for (int i = 0; i < 4; ++i)
    M.at(i, i) = 1.0;
  return M;
}
Mat4 mMul(const Mat4 &a, const Mat4 &b) {
  Mat4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0;
      for (int k = 0; k < 4; ++k)
        s += a.at(i, k) * b.at(k, j);
      r.at(i, j) = s;
    }
  return r;
}
Mat4 mRot(double ang, double x, double y, double z) {
  double c = std::cos(ang), s = std::sin(ang), k = 1.0 - c;
  Mat4 M = mIdent();
  M.at(0, 0) = c + k * x * x;
  M.at(0, 1) = k * x * y - s * z;
  M.at(0, 2) = k * x * z + s * y;
  M.at(1, 0) = k * x * y + s * z;
  M.at(1, 1) = c + k * y * y;
  M.at(1, 2) = k * y * z - s * x;
  M.at(2, 0) = k * x * z - s * y;
  M.at(2, 1) = k * y * z + s * x;
  M.at(2, 2) = c + k * z * z;
  return M;
}
Mat4 mTrans(double x, double y, double z) {
  Mat4 M = mIdent();
  M.at(0, 3) = x;
  M.at(1, 3) = y;
  M.at(2, 3) = z;
  return M;
}
Vec3d xform(const Mat4 &M, Vec3d p) { // p @ R^T + t  ==  R@p + t
  return {M.at(0, 0) * p.x + M.at(0, 1) * p.y + M.at(0, 2) * p.z + M.at(0, 3),
          M.at(1, 0) * p.x + M.at(1, 1) * p.y + M.at(1, 2) * p.z + M.at(1, 3),
          M.at(2, 0) * p.x + M.at(2, 1) * p.y + M.at(2, 2) * p.z + M.at(2, 3)};
}

const char *TREE_RULES[5] = {"FF[RL1][RR2][RRR3]F[RL3][RR1][RRR2]RFLR0", "FL[T[RF]2]R[TRFL]RTFL4",
                             "FL[TRF3]RFLRTFL2", "FL[TFL2RFL]R[T[RFLF3]]RTFL2",
                             "FL[TRFL4]RFLRTFL4"};
constexpr double YROTATE = 10.0, TILT = 120.0, MICRO_TILT = 1.0e-4;
constexpr double T_SCALE = 0.9, T_RADSCALE = 0.6, T_LENGTH = 5.0, T_RADIUS = 0.7;
constexpr int BASE_TRI = 5, NEEDLES = 9;
constexpr double LEAF_LEN = 4.0, LEAF_RAD = 1.0;
constexpr int SWAY_LEVELS = 2;
const int MATURITY[7] = {1, 2, 2, 3, 3, 3, 4};
const Vec3d C_WOOD_LIGHT{0.6549, 0.4901, 0.2392}, C_WOOD_DARK{0.3607, 0.2510, 0.2000};
const Vec3d C_NEEDLE{0.1373, 0.5568, 0.1373};

struct Seg {
  Mat4 m;
  double len, rad;
};
struct Leaf {
  Mat4 m;
  double sc;
};
struct Module {
  int parent;
  int level;
  Mat4 hang;
  std::vector<Seg> segs;
  std::vector<Leaf> leaves;
};

int expandTree(const std::string &rule, int depth, double scale, double radscale, int parent,
               int level, std::vector<Module> &out, const Mat4 &tMicro, const Mat4 &tTilt,
               const Mat4 &tRoll) {
  int me = static_cast<int>(out.size());
  out.push_back(Module{parent, level, mIdent(), {}, {}});
  Mat4 cur = mIdent();
  std::vector<Mat4> stack;
  double segLen = T_LENGTH * scale, segRad = T_RADIUS * radscale;
  Mat4 step = mTrans(0.0, segLen, 0.0);
  for (char ch : rule) {
    if (ch == 'F') {
      cur = mMul(cur, tMicro);
      out[me].segs.push_back({cur, segLen, segRad});
      cur = mMul(cur, step);
    } else if (ch == '[') {
      stack.push_back(cur);
    } else if (ch == ']') {
      cur = stack.back();
      stack.pop_back();
    } else if (ch == 'L') {
      out[me].leaves.push_back({cur, scale});
    } else if (ch == 'R') {
      cur = mMul(cur, tRoll);
    } else if (ch == 'T') {
      cur = mMul(cur, tTilt);
    } else if (std::isdigit((unsigned char)ch) && depth > 1) {
      int child = expandTree(TREE_RULES[ch - '0'], depth - 1, scale * T_SCALE,
                             radscale * T_RADSCALE, me, level + 1, out, tMicro, tTilt, tRoll);
      out[child].hang = cur;
    }
  }
  return me;
}

// The unit cylinder (_CYL) ring + triangle topology + per-vertex wood colour.
struct CylTopo {
  std::vector<Vec3d> ringUnit; // BASE_TRI points on the unit circle in XZ
  std::vector<cvc::geometry::index_t> tris;
  std::vector<Vec3d> colors; // 2*BASE_TRI+2 per-vertex wood colours
};
CylTopo cylTopo() {
  CylTopo c;
  for (int i = 0; i < BASE_TRI; ++i) {
    double a = i * 2.0 * M_PI / BASE_TRI;
    c.ringUnit.push_back({std::cos(a), 0.0, std::sin(a)});
  }
  for (int i = 0; i < BASE_TRI; ++i) {
    int b0 = 1 + i, b1 = 1 + (i + 1) % BASE_TRI;
    int t0 = BASE_TRI + 2 + i, t1 = BASE_TRI + 2 + (i + 1) % BASE_TRI;
    int idx[12] = {0, b0, b1, BASE_TRI + 1, t1, t0, b0, t1, b1, b0, t0, t1};
    for (int k = 0; k < 12; ++k)
      c.tris.push_back(idx[k]);
  }
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  return c;
}

struct ModRec {
  int parent;
  Mat4 hang;
  bool swayer;
  std::vector<Vec3d> localWood; // module-frame wood verts
  int wOff;
  std::vector<Vec3d> localNeedle;
  int nOff;
};
struct Tree {
  std::vector<ModRec> mods; // wOff/nOff index the MERGED forest buffers, not a per-tree one
  double phase = 0, sway = 0;
};

// Bark shader (verbatim GLSL from the latest master demo): vertical furrows around
// the branch (angle of the bind-pose normal) + axial variation, perturbing the
// normal by the height's surface gradient. Keeps the per-vertex wood colour.
const char *BARK_GLSL =
    "float bhash(vec2 p){ return fract(sin(dot(p, vec2(41.3, 289.1))) * 43758.5); }\n"
    "float bnoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  return mix(mix(bhash(i), bhash(i+vec2(1,0)), f.x),\n"
    "             mix(bhash(i+vec2(0,1)), bhash(i+vec2(1,1)), f.x), f.y);\n"
    "}\n"
    "float barkH(vec3 nrm, float z){\n"
    "  float ang = atan(nrm.y, nrm.x);\n"
    "  float f = 0.0;\n"
    "  f += 0.6*sin(ang*10.0 + 1.5*sin(z*0.7));\n"
    "  f += 0.3*sin(ang*23.0 + z*0.4);\n"
    "  f += 0.3*bnoise(vec2(ang*4.0, z*1.2));\n"
    "  return f;\n"
    "}\n";
void addBark(GeometryNode &node) {
  node.disableCoordinateShiftScale();
  node.addVertexShaderReplacement("//VTK::Normal::Dec",
                                  "//VTK::Normal::Dec\nout vec3 bNrm;\nout vec3 bPos;");
  node.addVertexShaderReplacement(
      "//VTK::PositionVC::Impl",
      "//VTK::PositionVC::Impl\n  bNrm = normalMC; bPos = vertexMC.xyz;");
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Dec",
      std::string("//VTK::Normal::Dec\nin vec3 bNrm;\nin vec3 bPos;\n") + BARK_GLSL);
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Impl",
      "//VTK::Normal::Impl\n"
      "  {\n"
      "    float h = barkH(normalize(bNrm), bPos.z);\n"
      "    vec3 sS = dFdx(vertexVC.xyz), sT = dFdy(vertexVC.xyz), vn = " CVC_FS_NORMAL ";\n"
      "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
      "    float det = dot(sS, R1);\n"
      "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
      "    " CVC_FS_NORMAL " = normalize(abs(det)*vn - 1.2*sg);\n"
      "  }\n");
}

// The tree turtle stands on +Y; the world is Z-up.
Mat4 treeUp() { return mRot(M_PI / 2.0, 1.0, 0.0, 0.0); }

// Grow one tree's geometry INTO the shared forest meshes (wg = wood, ng = needle),
// recording per-module re-pose records whose offsets index those SHARED buffers.
// The whole forest is merged to ONE wood actor + ONE needle actor (route C across
// trees): the wind re-poses every tree into two big buffers uploaded once per frame,
// not 64 tiny per-actor uploads — 64 draw calls collapse to 2 in both the main pass
// and the shadow-map bake.
Tree buildTree(cvc::app &app, double px, double py, double pz, const std::vector<Module> &mods,
               const CylTopo &cyl, const std::vector<Vec3d> &nring, cvc::geometry &wg,
               cvc::geometry &ng) {
  Tree tree;
  Mat4 tUp = treeUp();
  std::vector<Mat4> world(mods.size());
  int wCur = static_cast<int>(wg.points().size()); // GLOBAL offsets into the merged forest mesh
  int nCur = static_cast<int>(ng.points().size());
  for (size_t i = 0; i < mods.size(); ++i) {
    const Module &mod = mods[i];
    Mat4 hang = (mod.parent < 0) ? mMul(mTrans(px, py, pz), tUp) : mod.hang;
    world[i] = (mod.parent < 0) ? hang : mMul(world[mod.parent], hang);
    ModRec rec;
    rec.parent = mod.parent;
    rec.hang = hang;
    rec.swayer = mod.level <= SWAY_LEVELS;
    rec.wOff = wCur;
    rec.nOff = nCur;
    // wood: one _CYL per segment, in the module's local frame
    for (const Seg &s : mod.segs) {
      Vec3d loc[2 * BASE_TRI + 2];
      loc[0] = {0, 0, 0};
      loc[BASE_TRI + 1] = {0, s.len, 0};
      for (int r = 0; r < BASE_TRI; ++r) {
        loc[1 + r] = {cyl.ringUnit[r].x * s.rad, 0.0, cyl.ringUnit[r].z * s.rad};
        loc[BASE_TRI + 2 + r] = {cyl.ringUnit[r].x * s.rad, s.len, cyl.ringUnit[r].z * s.rad};
      }
      cvc::geometry::index_t base = wg.points().size();
      for (int v = 0; v < 2 * BASE_TRI + 2; ++v) {
        Vec3d p = xform(s.m, loc[v]); // module-frame vertex
        rec.localWood.push_back(p);
        Vec3d w = xform(world[i], p); // bind-pose world vertex
        wg.points().push_back({w.x, w.y, w.z});
        wg.colors().push_back({cyl.colors[v].x, cyl.colors[v].y, cyl.colors[v].z});
      }
      for (size_t k = 0; k < cyl.tris.size(); k += 3)
        wg.tris().push_back({base + cyl.tris[k], base + cyl.tris[k + 1], base + cyl.tris[k + 2]});
    }
    // needles: one star per leaf
    for (const Leaf &lf : mod.leaves) {
      cvc::geometry::index_t base = ng.points().size();
      Vec3d root = xform(lf.m, {0, 0, 0});
      rec.localNeedle.push_back({0, 0, 0});
      ng.points().push_back(
          {xform(world[i], root).x, xform(world[i], root).y, xform(world[i], root).z});
      for (int t = 0; t < NEEDLES; ++t) {
        Vec3d tip{nring[t].x * LEAF_RAD * lf.sc, LEAF_LEN * lf.sc, nring[t].z * LEAF_RAD * lf.sc};
        Vec3d pm = xform(lf.m, tip);
        rec.localNeedle.push_back(pm);
        Vec3d w = xform(world[i], pm);
        ng.points().push_back({w.x, w.y, w.z});
        ng.lines().push_back({base, static_cast<cvc::geometry::index_t>(base + 1 + t)});
      }
    }
    wCur = static_cast<int>(wg.points().size());
    nCur = static_cast<int>(ng.points().size());
    tree.mods.push_back(std::move(rec));
  }
  return tree;
}

// Seed a flat [x,y,z,...] buffer from a merged mesh's bind-pose points.
std::vector<double> flattenPoints(const cvc::geometry &g) {
  std::vector<double> buf(g.points().size() * 3);
  for (size_t v = 0; v < g.points().size(); ++v) {
    buf[v * 3] = g.points()[v][0];
    buf[v * 3 + 1] = g.points()[v][1];
    buf[v * 3 + 2] = g.points()[v][2];
  }
  return buf;
}

// ── the afternoon sun (a flat-lit disc + faint halo, far out) ────────────────
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0;
constexpr double SUN_REF_DIST = 900.0; // reference distance for the sun's angular size
constexpr double SUN_DISC_R = 26.0;    // disc radius at SUN_REF_DIST (~1.6° angular)
constexpr double SUN_DEPTH_CAP =
    150.0; // max sun-direction depth — keeps the sun within the
           // shadow-map depth range so it never self-shadows the island
Vec3d vnorm(Vec3d a) {
  double l = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  return l > 1e-12 ? Vec3d{a.x / l, a.y / l, a.z / l} : Vec3d{0, 0, 1};
}
Vec3d vcross(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3d sunDir(double azDeg, double elDeg) {
  double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}
cvc::geometry discGeom(cvc::app &app, Vec3d c, Vec3d normal, double radius, int seg = 48) {
  Vec3d n = vnorm(normal);
  Vec3d up = std::fabs(n.z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
  Vec3d u = vnorm(vcross(n, up));
  Vec3d v = vcross(n, u);
  cvc::geometry g(app);
  g.points().push_back({c.x, c.y, c.z});
  for (int i = 0; i < seg; ++i) {
    double th = i * 2.0 * M_PI / seg;
    g.points().push_back({c.x + radius * (std::cos(th) * u.x + std::sin(th) * v.x),
                          c.y + radius * (std::cos(th) * u.y + std::sin(th) * v.y),
                          c.z + radius * (std::cos(th) * u.z + std::sin(th) * v.z)});
  }
  for (int i = 0; i < seg; ++i)
    g.tris().push_back({0, static_cast<cvc::geometry::index_t>(1 + i),
                        static_cast<cvc::geometry::index_t>(1 + (i + 1) % seg)});
  return g;
}
// Sun disc + halo, built at the ORIGIN facing back down the sun ray. main()
// repositions them to (cameraEye + sunDir·SUN_CAM_DIST) every frame, so the sun
// sits a constant distance AHEAD of the camera in the fixed world sun direction —
// effectively at infinity: always in the sky where the light comes from, and never
// able to drift between the camera and the island (the old fixed-world disc did,
// eclipsing the scene as the camera came around). Flat-lit (ambient only), so it
// is a bright disc from any angle and is immune to shadows.
void addSun(cvc::app &app, SceneGraph &sg) {
  Vec3d d = sunDir(SUN_AZ, SUN_EL);
  Vec3d face{-d.x, -d.y, -d.z}; // faces back toward the camera
  sg.addGraphics("sun", discGeom(app, {0, 0, 0}, face, SUN_DISC_R));
  auto disc = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun"));
  disc->setColor(1.0, 0.97, 0.88);
  disc->setAmbient(1.0);
  disc->setDiffuse(0.0);
  disc->setSpecular(0.0);
  // Opacity just under 1 puts the disc in the TRANSLUCENT bucket, so the opaque
  // shadow-map bake skips it (an opaque billboard 900 units away would stretch the
  // light's depth range and self-shadow the island to black). Visually still solid.
  disc->setOpacity(0.99);
  sg.addGraphics("sun_halo", discGeom(app, {0, 0, 0}, face, SUN_DISC_R * 3.4));
  auto halo = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun_halo"));
  halo->setColor(1.0, 0.90, 0.72);
  halo->setAmbient(1.0);
  halo->setDiffuse(0.0);
  halo->setSpecular(0.0);
  halo->setOpacity(0.22);
}

// ── the ocean: a GPU spectral FFT sea (OceanFFT) on a static grid ────────────
// The crude wave-volume sea is replaced by a real ABYSSAL-style FFT ocean. The sea
// is a STATIC flat grid at z = SEA_LEVEL that extends past the island to the horizon;
// its vertex shader samples the live OceanFFT displacement textures (bound via
// GeometryNode::setShaderTexture) and displaces on the GPU, and its fragment shader
// runs the water BRDF. No CPU readback / per-frame updateVertices — the WASM path.
constexpr double OCEAN_HALF = 220.0; // ocean overhangs the island out to the horizon
constexpr int OCEAN_N = 224;         // ocean grid resolution (verts per side)
constexpr int FFT_N = 256;           // OceanFFT working resolution (ABYSSAL default)

// A flat ocean grid at z = SEA_LEVEL. Its (x,y) are fixed; the GPU displaces z (and
// choppy x/y) from the FFT textures each frame — the mesh itself never changes.
cvc::geometry buildOcean(cvc::app &app) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  for (int j = 0; j < OCEAN_N; ++j) {
    double y = -OCEAN_HALF + 2.0 * OCEAN_HALF * j / (OCEAN_N - 1);
    for (int i = 0; i < OCEAN_N; ++i) {
      double x = -OCEAN_HALF + 2.0 * OCEAN_HALF * i / (OCEAN_N - 1);
      pts.push_back({x, y, SEA_LEVEL});
      cols.push_back({0.05, 0.20, 0.32}); // fallback colour if the BRDF is unavailable
    }
  }
  auto &tris = g.tris();
  for (int j = 0; j < OCEAN_N - 1; ++j)
    for (int i = 0; i < OCEAN_N - 1; ++i) {
      cvc::geometry::index_t v = j * OCEAN_N + i;
      tris.push_back({v, static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N)});
      tris.push_back({static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N)});
    }
  return g;
}

// ── GPU ocean shaders (no-readback path) ──────────────────────────────────────
// Vertex shader: sum the 3 cascade displacement textures (each tiled at its own
// metre scale) and displace a mutable copy of vertexMC (immutable `in` under
// disableCoordinateShiftScale). World XY comes straight from vertexMC.
//
// The shore (OCEAN-AND-WEATHER-ROADMAP.md A1-A2): the seabed is a HeightFieldTexture
// of terrainH (cvc_hf_height, the same bilinear grid on the CPU and the GPU) and the
// shore grid an RGBA32F texture (distance to shore, offshore normal, slope) from
// cvc::world::water_bodies. Each cascade fades out as the still water shoals, over
// uFade (its band's half wavelength, capped by coast.water.shore_fade), so on dry
// ground nothing moves and at the waterline the sea is still; and a vertex never
// moves sideways more than 0.8 of the way to the shore, so no vertex crosses the
// waterline. Pieces of the declaration are spliced in at run time (oceanVsDec).
const char *OCEAN_VS_DEC_HEAD =
    "//VTK::PositionVC::Dec\n"
    "out vec3 vWorldPos;\n"
    "out vec3 vOceanN;\n"
    "out float vOceanJac;\n"
    "uniform sampler2D uDisp0; uniform sampler2D uDisp1; uniform sampler2D uDisp2;\n"
    "uniform float uTile0; uniform float uTile1; uniform float uTile2;\n"
    "uniform float uHalfTexel;\n" // 0.5 / FFT_N: sample the texel nearest a point
    "uniform float uWaveAmp; uniform float uChop;\n"
    "uniform float uLevel; uniform vec3 uFade;\n"
    "uniform highp sampler2D uShoreGrid; uniform vec3 uShoreGridOrigin; uniform vec3 "
    "uShoreGridSpacing;\n"
    // the shore layer (cvc::ocean::shore_layer, on the shore grid): (phase, amplitude, reflected,
    // steepness) and (direction x, y, wavenumber, breaking), and its trains
    "uniform highp sampler2D uShoreWave; uniform highp sampler2D uShoreDir;\n"
    "uniform float uSwellOmega; uniform float uShoreTime;\n"
    "uniform vec3 uTrainRatio; uniform vec3 uTrainAmp; uniform vec3 uTrainPhase;\n"
    "uniform vec3 uTrainPhaseR;\n"
    "out float vShoreBreak;\n";
const char *OCEAN_VS_DEC_FUNCS =
    // a shore-grid texture at p, bilinear by hand (WebGL2 filters no float texture)
    "vec4 oGrid4(highp sampler2D tex, vec2 p){\n"
    "  ivec2 sz = textureSize(tex, 0);\n"
    "  vec2 f = (p - uShoreGridOrigin.xy) / uShoreGridSpacing.xy;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec4 a = mix(texelFetch(tex, i0, 0), texelFetch(tex, ivec2(i1.x, i0.y), 0), fr.x);\n"
    "  vec4 b = mix(texelFetch(tex, ivec2(i0.x, i1.y), 0), texelFetch(tex, i1, 0), fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    "float oShoreDist(vec2 p){ return oGrid4(uShoreGrid, p).r; }\n"
    // the shore layer at p: (height, sideways x, sideways y, breaking on a crest), the sum
    // cvc::ocean::shore_layer::evaluate does on the CPU
    "vec4 oShoreLayer(vec2 p){\n"
    "  vec4 w = oGrid4(uShoreWave, p);\n"
    "  if (w.y <= 0.0 && w.z <= 0.0) return vec4(0.0);\n"
    "  vec4 d = oGrid4(uShoreDir, p);\n"
    "  vec2 dir = d.xy; float dl = length(dir); if (dl > 1e-6) dir /= dl;\n"
    "  float eta = 0.0, h = 0.0, crest = -1.0;\n"
    "  for (int i = 0; i < 3; ++i) {\n"
    "    float r = uTrainRatio[i];\n"
    "    float th = uSwellOmega*r*uShoreTime - r*w.x + uTrainPhase[i];\n"
    "    float thr = uSwellOmega*r*uShoreTime + r*w.x + uTrainPhaseR[i];\n"
    "    eta += uTrainAmp[i]*(w.y*cos(th) + w.z*cos(thr));\n"
    "    h += uTrainAmp[i]*w.w*w.y*sin(th);\n"
    "    crest = max(crest, cos(th));\n"
    "  }\n"
    "  return vec4(eta, h*dir, d.w*smoothstep(0.55, 1.0, crest));\n"
    "}\n"
    "float oShoreEta(vec2 p){\n"
    "  vec4 w = oGrid4(uShoreWave, p);\n"
    "  if (w.y <= 0.0 && w.z <= 0.0) return 0.0;\n"
    "  float eta = 0.0;\n"
    "  for (int i = 0; i < 3; ++i) {\n"
    "    float r = uTrainRatio[i];\n"
    "    eta += uTrainAmp[i]*(w.y*cos(uSwellOmega*r*uShoreTime - r*w.x + uTrainPhase[i])\n"
    "                       + w.z*cos(uSwellOmega*r*uShoreTime + r*w.x + uTrainPhaseR[i]));\n"
    "  }\n"
    "  return eta;\n"
    "}\n"
    // each cascade's weight at p: 0 on dry ground, rising to 1 as the still water deepens
    "vec3 oDepthW(vec2 p){\n"
    "  float d = uLevel - cvc_hf_height(p);\n"
    "  return vec3(smoothstep(0.0,uFade.x,d), smoothstep(0.0,uFade.y,d), "
    "smoothstep(0.0,uFade.z,d));\n"
    "}\n"
    // a cascade's displacement at world w. Texel i holds the field at i * tile / N and the sampler
    // is nearest (WebGL2 filters no float texture), so half a texel over picks the texel nearest
    // w rather than the one at or before it: the steps then straddle the field instead of trailing
    // it by half a texel, which halves how far the picture is from the sea physics feels
    // (cvcgl_ocean_band: 106 mm RMS to 47 mm, OCEAN-AND-WEATHER-ROADMAP.md O7b).
    "vec4 oDisp(sampler2D t, vec2 w, float tile){ return textureLod(t, w/tile + uHalfTexel, 0.0); "
    "}\n"
    "vec4 oSum(vec2 w, vec3 k){\n"
    "  vec4 a=oDisp(uDisp0,w,uTile0), b=oDisp(uDisp1,w,uTile1), c=oDisp(uDisp2,w,uTile2);\n"
    "  vec3 j = vec3(1.0) + k*(vec3(a.w,b.w,c.w) - vec3(1.0));\n" // a faded fold folds less
    "  return vec4(k.x*a.xyz+k.y*b.xyz+k.z*c.xyz, min(min(j.x,j.y),j.z));\n"
    "}\n"
    "float oHt(vec2 w, vec3 k){ return "
    "k.x*oDisp(uDisp0,w,uTile0).y+k.y*oDisp(uDisp1,w,uTile1).y+k.z*oDisp(uDisp2,w,uTile2).y; }\n";
const char *OCEAN_VS_IMPL =
    "  vec4 dispMC = vertexMC;\n"
    "  vec2 owxy = dispMC.xy;\n"
    "  vec3 ok = oDepthW(owxy);\n"
    "  vec4 od = oSum(owxy, ok);\n"
    // the swell at the coast, at its true size: uWaveAmp scales the cascades only, so the shore
    // layer's breaking cap (crest to trough under 0.78 of the depth) holds at any wave height
    "  vec4 osl = oShoreLayer(owxy);\n"
    "  float hpx=uWaveAmp*oHt(owxy+vec2(1.0,0.0),ok)+oShoreEta(owxy+vec2(1.0,0.0));\n"
    "  float hmx=uWaveAmp*oHt(owxy-vec2(1.0,0.0),ok)+oShoreEta(owxy-vec2(1.0,0.0));\n"
    "  float hpy=uWaveAmp*oHt(owxy+vec2(0.0,1.0),ok)+oShoreEta(owxy+vec2(0.0,1.0));\n"
    "  float hmy=uWaveAmp*oHt(owxy-vec2(0.0,1.0),ok)+oShoreEta(owxy-vec2(0.0,1.0));\n"
    "  vec3 oN = normalize(vec3(-(hpx-hmx)*0.5, -(hpy-hmy)*0.5, 1.0));\n"
    "  vec2 dh = uChop*vec2(od.x, od.z) + osl.yz;\n"
    "  vShoreBreak = osl.w;\n"
    "  float dlim = 0.8*max(oShoreDist(owxy), 0.0), dlen = length(dh);\n"
    "  if (dlen > dlim) dh *= dlim/max(dlen, 1e-6);\n"
    "  dispMC.xy += dh;\n"
    "  dispMC.z  += uWaveAmp*od.y + osl.x;\n"
    "  vertexVCVSOutput = MCVCMatrix * dispMC;\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "  vWorldPos = dispMC.xyz;\n"
    "  vOceanN = oN;\n"
    "  vOceanJac = od.w;\n";
// The depth over which a cascade fades out toward the waterline (OCEAN-AND-WEATHER-ROADMAP.md
// A2): half the wavelength its band's energy peaks at, since a wave feels the bottom in water
// shallower than that, but never more than `cap` (coast.water.shore_fade). The band's energy
// peaks at the sea's peak (the swell's, when there is one) if that falls inside the band, else
// at the band's edge nearer it. The cap matters here: this island stands on a shelf about 9 m
// deep out to the horizon, so fading the 60 m peak waves over 30 m of depth would calm the
// whole sea; until the shore layer (O4) carries their energy into the shallows, they fade only
// near the beach.
double cascadeFade(const cvc::ocean::cascade &c, const cvc::ocean::sea_state &s, double cap) {
  double w = cvc::ocean::wind_sea_jonswap(s.wind).peak_omega;
  if (s.swell.hs_m > 1e-3)
    w = std::min(w, cvc::ocean::swell_jonswap(s.swell).peak_omega);
  const double kPeak = w * w / cvc::ocean::gravity; // deep water
  const double k = std::min(std::max(kPeak, c.k_min), c.k_max);
  return std::max(0.05, std::min(cap, 3.14159265358979323846 / k)); // lambda / 2 = pi / k
}

// The vertex shader's declarations: the head, the seabed's sampler (HeightFieldTexture),
// then the functions that read both.
std::string oceanVsDec() {
  return std::string(OCEAN_VS_DEC_HEAD) + cvc::gl::HeightFieldTexture::glsl() + OCEAN_VS_DEC_FUNCS;
}
// Fragment shader: a Fresnel water BRDF + a depth-tinted shallow band + animated
// shore foam. The seabed is the vertex shader's HeightFieldTexture (cvc_hf_height),
// so the shader knows the ground at any world (x,y) and can compute the LOCAL water
// depth (surface z − seabed) — the waterline is depth 0, and because the surface z
// rises and falls with the swell, the thin-water band (and its foam) washes up and
// down the beach as the waves roll in. onoise() breaks the foam edge into surf.
const char *OCEAN_FS_DEC_HEAD =
    "//VTK::Normal::Dec\n"
    "in vec3 vWorldPos;\n"
    "in vec3 vOceanN;\n"
    "in float vOceanJac;\n"
    "in float vShoreBreak;\n" // the shore layer's breaking, on its crests
    "uniform vec3 uCamPos; uniform vec3 uSunDir; uniform float uFoamBias;\n"
    "uniform float uTime; uniform float uShore;\n";
const char *OCEAN_FS_DEC_FUNCS =
    "float ohash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }\n"
    "float onoise(vec2 p){\n"
    "  vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  return mix(mix(ohash(i),ohash(i+vec2(1,0)),f.x),\n"
    "             mix(ohash(i+vec2(0,1)),ohash(i+vec2(1,1)),f.x), f.y);\n"
    "}\n";
std::string oceanFsDec() {
  return std::string(OCEAN_FS_DEC_HEAD) + cvc::gl::HeightFieldTexture::glsl() + OCEAN_FS_DEC_FUNCS;
}
// Consumes //VTK::Light::Impl (VTK's lighting is replaced): writes the final water
// colour to gl_FragData[0] directly.
const char *OCEAN_FS_LIGHT =
    "  vec3 N = normalize(vOceanN);\n"
    "  vec3 Vd = normalize(uCamPos - vWorldPos);\n"
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
    // Local water depth = surface z − seabed. Shallow water is TRANSLUCENT: the sandy
    // bottom shows through, attenuated by depth (Beer–Lambert clarity), so the shoals
    // read a bright sandy turquoise that deepens to open blue offshore.
    "  float depth = vWorldPos.z - cvc_hf_height(vWorldPos.xy);\n"
    "  float clarity = exp(-max(depth,0.0)*0.20);\n"
    "  vec3 sand = vec3(0.66,0.64,0.50);\n"
    "  vec3 shoal = mix(vec3(0.12,0.46,0.54), sand, clarity*0.55);\n"
    "  col = mix(col, shoal, clarity*0.85);\n"
    // Jacobian (wave-fold) foam on the open sea.
    "  float foam = clamp((uFoamBias - vOceanJac)/0.85, 0.0, 1.0); foam=foam*foam*(3.0-2.0*foam);\n"
    // shore foam: a bright, noisy band right at the waterline, washing up the beach.
    "  float band = 1.0 - smoothstep(0.0, 1.8, abs(depth));\n"
    "  float nz = onoise(vWorldPos.xy*0.4 + vec2(uTime*0.6, uTime*0.35));\n"
    "  float shoreFoam = clamp(band*(0.55+0.75*nz)*step(-0.4, depth)*uShore, 0.0, 1.0);\n"
    "  foam = max(foam, shoreFoam);\n"
    // breakers: the shore layer's breaking crests, frayed like the shore band
    "  foam = max(foam, clamp(vShoreBreak*(0.7+0.6*nz)*uShore, 0.0, 1.0));\n"
    "  col = mix(col, vec3(0.93,0.96,0.97), foam);\n"
    "  gl_FragData[0] = vec4(col, 1.0);\n";

// ── the sky: a drifting, evolving cloud slab grown by an L-system 3-D turtle ──
// A faithful C++ port of the Python demo's cloud volume. The turtle moves in 3-D
// and deposits a Gaussian BALL per F (so a puff is round, not a column); two
// independent skies are grown, then per frame crossfaded (the cloud changes SHAPE
// as it travels) and sub-cell scrolled along +x (smooth drift at any speed). The
// slab is thin in z relative to x/y, faded at all six faces so nothing is cut
// square, and normalised on a high percentile so a few overlaps don't set the
// scale. Empty sky is pinned EXACTLY transparent; only dense cores composite.
constexpr int SKY_N = 60, SKY_NZ = 28; // finer grid for fBm detail, but sized for realtime
constexpr double SKY_BASE = 74.0, SKY_TOP = 122.0; // a deep slab, so puffs are round
constexpr double SKY_HALF = 150.0;                 // overhangs the island a little
constexpr double CLOUD_DRIFT = 3.0;                // world units / second (gentle)
constexpr double CLOUD_MORPH_S = 60.0; // seconds per crossfade (slow, so it barely pulses)
constexpr int CLOUD_MAPS = 2, CLOUD_DEPTH = 6;
constexpr double CLOUD_TURN = 32.0, CLOUD_STEP0 = 8.1, CLOUD_STEP_DECAY = 0.9; // step/puff
constexpr double CLOUD_PUFF0 = 8.8, CLOUD_PUFF_DECAY = 0.88;                   // scaled with SKY_N
constexpr double CLOUD_FLOOR = 0.10, CLOUD_EMPTY = 0.22;
const char *CLOUD_AXIOM = "[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]";
const char *cloudRule(char c) {
  switch (c) {
  case 'A':
    return "FF[+<B]^F[-<C]<F[+<C]vFA"; // anvil-ward drift, throws B/C fringes
  case 'B':
    return "F[+<F]F<[-<F]vB";
  case 'C':
    return "^<F[+<F][-<F]^<FC";
  default:
    return nullptr;
  }
}
inline size_t skyIdx(int z, int y, int x) { // field is (nz, ny, nx), C order
  return (static_cast<size_t>(z) * SKY_N + y) * SKY_N + x;
}
// numpy-style linear percentile of an already-sorted array.
double percentileSorted(const std::vector<float> &a, double q) {
  if (a.empty())
    return 0.0;
  double rank = (q / 100.0) * (a.size() - 1);
  size_t lo = static_cast<size_t>(std::floor(rank));
  if (lo + 1 >= a.size())
    return a.back();
  return a[lo] + (rank - lo) * (a[lo + 1] - a[lo]);
}
// ── 3-D value-noise fBm, for the clouds' fractal fluff ────────────────────────
// A cheap integer-lattice hash -> smooth trilinear value noise -> a few octaves.
// Summing octaves (each finer + fainter) gives fractal Brownian motion: the same
// billowing detail at every scale, which is what turns a smooth blob into a
// cauliflower cloud with wispy edges.
double vhash3(int x, int y, int z) {
  unsigned h = static_cast<unsigned>(x * 374761393 + y * 668265263 + z * 1274126177);
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffffffu) / double(0x1000000);
}
double vnoise3(double x, double y, double z) {
  int xi = (int)std::floor(x), yi = (int)std::floor(y), zi = (int)std::floor(z);
  double fx = x - xi, fy = y - yi, fz = z - zi;
  auto sm = [](double t) { return t * t * (3.0 - 2.0 * t); };
  fx = sm(fx);
  fy = sm(fy);
  fz = sm(fz);
  auto L = [](double a, double b, double t) { return a + (b - a) * t; };
  double x00 = L(vhash3(xi, yi, zi), vhash3(xi + 1, yi, zi), fx);
  double x10 = L(vhash3(xi, yi + 1, zi), vhash3(xi + 1, yi + 1, zi), fx);
  double x01 = L(vhash3(xi, yi, zi + 1), vhash3(xi + 1, yi, zi + 1), fx);
  double x11 = L(vhash3(xi, yi + 1, zi + 1), vhash3(xi + 1, yi + 1, zi + 1), fx);
  return L(L(x00, x10, fy), L(x01, x11, fy), fz);
}
double fbm3(double x, double y, double z, int octaves) {
  double f = 0.0, amp = 0.5, tot = 0.0, fr = 1.0;
  for (int i = 0; i < octaves; ++i) {
    f += amp * vnoise3(x * fr, y * fr, z * fr);
    tot += amp;
    amp *= 0.5;
    fr *= 2.02;
  }
  return f / tot; // [0,1]
}
// Grow ONE 3-D cloud field: run the turtle, splat a Gaussian ball per F, normalise
// on the 99.9th percentile, then fade at the faces (hanning^0.5 in x/y, sine in z).
std::vector<float> walkClouds(std::mt19937 &rng) {
  std::uniform_real_distribution<double> U(0.0, 1.0);
  auto uni = [&](double a, double b) { return a + (b - a) * U(rng); };
  const int N = SKY_N, NZ = SKY_NZ;
  std::vector<float> field(static_cast<size_t>(NZ) * N * N, 0.0f);
  // z is squashed: the slab is much thinner than it is wide, so a world-round puff
  // spans far fewer cells vertically.
  const double zscale = (double(N) / NZ) * ((SKY_TOP - SKY_BASE) / (2.0 * SKY_HALF));

  double x = uni(0.25, 0.75) * N, y = uni(0.3, 0.7) * N, z = NZ * 0.42;
  double head = uni(0.0, 360.0);
  double step = CLOUD_STEP0, puff = CLOUD_PUFF0, climb = 0.0;
  int depth = 0;
  struct St {
    double x, y, z, head, step, puff, climb;
    int depth;
  };
  std::vector<St> stack;
  std::deque<char> todo(CLOUD_AXIOM, CLOUD_AXIOM + std::strlen(CLOUD_AXIOM));
  int guard = 0;
  while (!todo.empty() && guard < 4000) {
    ++guard;
    char c = todo.front();
    todo.pop_front();
    if (c == 'F') {
      x = std::fmod(x + step * std::cos(head * M_PI / 180.0), double(N));
      if (x < 0)
        x += N; // Python % always lands in [0, N)
      y = std::min(std::max(y + step * std::sin(head * M_PI / 180.0), 0.0), double(N - 1));
      z = std::min(std::max(z + climb, 1.0), double(NZ - 2));
      const double p2 = 2.0 * puff * puff;
      for (int gz = 0; gz < NZ; ++gz) {
        double dz = (gz - z) * zscale, dz2 = dz * dz;
        for (int gy = 0; gy < N; ++gy) {
          double dy = gy - y, dyz2 = dy * dy + dz2;
          for (int gx = 0; gx < N; ++gx) {
            double dx = std::fabs(gx - x);
            dx = std::min(dx, double(N) - dx); // nearest image across the seam
            field[skyIdx(gz, gy, gx)] += static_cast<float>(std::exp(-(dx * dx + dyz2) / p2));
          }
        }
      }
    } else if (c == '+') {
      head += CLOUD_TURN;
    } else if (c == '-') {
      head -= CLOUD_TURN;
    } else if (c == '<') {
      puff *= CLOUD_PUFF_DECAY;
    } else if (c == '^') {
      climb += 0.55; // a turret climbs as it goes
    } else if (c == 'v') {
      climb -= 0.45; // a fringe sags away underneath
    } else if (c == '[') {
      stack.push_back({x, y, z, head, step, puff, climb, depth});
    } else if (c == ']') {
      if (!stack.empty()) {
        St s = stack.back();
        stack.pop_back();
        x = s.x;
        y = s.y;
        z = s.z;
        head = s.head;
        step = s.step;
        puff = s.puff;
        climb = s.climb;
        depth = s.depth;
      }
    } else if (cloudRule(c) && depth < CLOUD_DEPTH) {
      ++depth;
      step *= CLOUD_STEP_DECAY;
      const char *r = cloudRule(c);
      todo.insert(todo.begin(), r, r + std::strlen(r)); // list(rule) + todo
    }
  }
  // Normalise on a high percentile, not the max: one overlap must not set the scale.
  std::vector<float> sorted(field);
  std::sort(sorted.begin(), sorted.end());
  double m = percentileSorted(sorted, 99.9);
  if (m > 0)
    for (float &v : field)
      v = std::min(1.0f, std::max(0.0f, v / static_cast<float>(m)));
  // Fade at the slab faces (hanning^0.5 across x/y, a half-sine across z), and in
  // the same pass modulate the density with fractal (fBm) noise so the smooth
  // Gaussian base billows into cauliflower lumps and frays into wisps at the edges
  // — the fluffy, self-similar texture of a real cumulus rather than a soft blob.
  std::vector<double> w(N), zf(NZ);
  for (int i = 0; i < N; ++i)
    w[i] = std::sqrt(0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1)));
  for (int k = 0; k < NZ; ++k)
    zf[k] = std::sin(0.12 + (M_PI - 0.24) * k / (NZ - 1));
  for (int gz = 0; gz < NZ; ++gz)
    for (int gy = 0; gy < N; ++gy)
      for (int gx = 0; gx < N; ++gx) {
        float &v = field[skyIdx(gz, gy, gx)];
        v *= static_cast<float>(std::min(w[gy], w[gx]) * zf[gz]);
        if (v > 0.0f) {
          double d = fbm3(gx * 0.30, gy * 0.30, gz * 0.62, 5); // [0,1] fractal detail
          v = static_cast<float>(std::min(1.0, std::max(0.0, v * (0.32 + 1.5 * d))));
        }
      }
  // Cheap BAKED top-light: march each voxel toward the sun, accumulate the cloud
  // density in the way, and thin the voxel by that shadow. The sunlit crowns keep
  // full density (solid + white); the self-shadowed undersides drop toward a floor
  // (softer, more translucent, a cool wash of sky through them) — a real sense of
  // depth WITHOUT VTK's volume gradient-shade, which grays this fractal field.
  {
    Vec3d sd = sunDir(SUN_AZ, SUN_EL); // toward the sun (world)
    double ux = sd.x * N / (2.0 * SKY_HALF), uy = sd.y * N / (2.0 * SKY_HALF),
           uz = sd.z * NZ / (SKY_TOP - SKY_BASE); // sun step in grid cells
    double ul = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= ul;
    uy /= ul;
    uz /= ul;
    auto samp = [&](double cx, double cy, double cz) -> double {
      if (cx < 0 || cx > N - 1 || cy < 0 || cy > N - 1 || cz < 0 || cz > NZ - 1)
        return 0.0;
      int x0 = (int)cx, y0 = (int)cy, z0 = (int)cz;
      int x1 = std::min(x0 + 1, N - 1), y1 = std::min(y0 + 1, N - 1), z1 = std::min(z0 + 1, NZ - 1);
      double tx = cx - x0, ty = cy - y0, tz = cz - z0;
      auto V = [&](int x, int y, int z) { return (double)field[skyIdx(z, y, x)]; };
      double c00 = V(x0, y0, z0) * (1 - tx) + V(x1, y0, z0) * tx;
      double c10 = V(x0, y1, z0) * (1 - tx) + V(x1, y1, z0) * tx;
      double c01 = V(x0, y0, z1) * (1 - tx) + V(x1, y0, z1) * tx;
      double c11 = V(x0, y1, z1) * (1 - tx) + V(x1, y1, z1) * tx;
      return (c00 * (1 - ty) + c10 * ty) * (1 - tz) + (c01 * (1 - ty) + c11 * ty) * tz;
    };
    const int LSTEPS = 7;
    const double LK = 0.95, LFLOOR = 0.72,
                 LSTEP = 1.6; // floor kept high: shade without fraying apart
    std::vector<float> lit(field.size());
    for (int gz = 0; gz < NZ; ++gz)
      for (int gy = 0; gy < N; ++gy)
        for (int gx = 0; gx < N; ++gx) {
          size_t o = skyIdx(gz, gy, gx);
          double v = field[o];
          if (v <= 0.0) {
            lit[o] = 0.0f;
            continue;
          }
          double tau = 0.0;
          for (int s = 1; s <= LSTEPS; ++s)
            tau += samp(gx + ux * s * LSTEP, gy + uy * s * LSTEP, gz + uz * s * LSTEP) * LSTEP;
          lit[o] = static_cast<float>(v * (LFLOOR + (1.0 - LFLOOR) * std::exp(-LK * tau)));
        }
    field.swap(lit);
  }
  return field;
}
// Holds the grown cloud maps + a normalisation peak; samples a continuous field.
struct SkyModel {
  std::vector<std::vector<float>> maps;
  double norm = 1.0;

  // Density at a CONTINUOUS scroll offset (sub-cell) and crossfade position.
  std::vector<float> raw(double shift, double morph) const {
    const int N = SKY_N, NZ = SKY_NZ;
    long mi = static_cast<long>(std::floor(morph));
    int i = static_cast<int>(((mi % CLOUD_MAPS) + CLOUD_MAPS) % CLOUD_MAPS);
    int j = (i + 1) % CLOUD_MAPS;
    double u = morph - std::floor(morph);
    u = u * u * (3.0 - 2.0 * u); // smoothstep the crossfade
    long ks = static_cast<long>(std::floor(shift));
    double f = shift - ks;
    int k0 = static_cast<int>(((ks % N) + N) % N), k1 = (k0 + 1) % N;
    const std::vector<float> &A = maps[i], &B = maps[j];
    std::vector<float> out(static_cast<size_t>(NZ) * N * N);
    for (int z = 0; z < NZ; ++z)
      for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
          // crossfade the two skies, then blend adjacent integer x-rolls (np.roll
          // axis=2: rolled[x] = vol[(x-k) mod N]).
          int sx0 = ((x - k0) % N + N) % N, sx1 = ((x - k1) % N + N) % N;
          auto mix = [&](int xx) {
            return (1.0 - u) * A[skyIdx(z, y, xx)] + u * B[skyIdx(z, y, xx)];
          };
          double vol = (1.0 - f) * mix(sx0) + f * mix(sx1);
          double lump = std::min(1.0, std::max(0.0, (vol - CLOUD_FLOOR) / (1.0 - CLOUD_FLOOR)));
          out[skyIdx(z, y, x)] = static_cast<float>(lump * lump); // dense cores, empty fringes
        }
    return out;
  }
  std::vector<float> field(double shift, double morph) const {
    std::vector<float> r = raw(shift, morph);
    double inv = 1.0 / norm;
    for (float &v : r)
      v = static_cast<float>(v * inv);
    return r;
  }
};
SkyModel buildSky() {
  SkyModel sky;
  // A grown map can land its dense core near a faded face, leaving it nearly
  // transparent once the floor is subtracted and squared — so the sky would fade
  // in only as the morph reaches the other map. Seed 20 grows BOTH maps centred
  // (worst-case field peak ~0.85), so cloud is visible from the very first frame.
  std::mt19937 rng(20u);
  for (int m = 0; m < CLOUD_MAPS; ++m)
    sky.maps.push_back(walkClouds(rng));
  double norm = 0.0; // sweep shift/morph; scale so the densest cloud lands near 1
  for (int c = 0; c < SKY_N; c += 8)
    for (double mo : {0.0, 0.5, 1.0}) {
      std::vector<float> r = sky.raw(double(c), mo);
      for (float v : r)
        norm = std::max(norm, double(v));
    }
  sky.norm = norm > 0 ? norm : 1.0;
  return sky;
}
cvc::volume skyVolume(cvc::app &app, const std::vector<float> &field) {
  return cvc::volume(
      app, reinterpret_cast<const unsigned char *>(field.data()),
      cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
      cvc::bounding_box(-SKY_HALF, -SKY_HALF, SKY_BASE, SKY_HALF, SKY_HALF, SKY_TOP));
}
// Empty sky must be EXACTLY invisible (alpha pinned flat at 0 up to CLOUD_EMPTY,
// not ramped — a ramp still accumulates over the ~180 samples a ray takes through
// the slab); cores must reach ~1 so a puff composites white, not as dirty smoke.
void skyTransfer(std::vector<double> &color, std::vector<double> &opacity) {
  // Cool blue-grey at the low (self-shadowed / thin) end, warming to pure white at
  // the dense, sunlit cores — so the baked top-light reads as cool shaded undersides
  // under bright crowns rather than a flat white slab.
  color = {0.0, 0.80, 0.85, 0.93, 0.45, 0.94, 0.96, 0.98, 1.0, 1.00, 1.00, 1.00};
  opacity = {0.0, 0.0, CLOUD_EMPTY, 0.0, 0.55, 0.26, 1.0, 0.54};
}

// ── cloud → ground shadow: a volumetric shadow, ray-marched through the slab ─────
// For each ground point we march the SAME density field the volume shows, up
// through the slab, accumulate optical depth τ, and store transmittance exp(-k·τ)
// into a texture the terrain samples — a per-fragment directional light pass for the
// cloud, computed once per cloud update. Floored so shaded ground still catches
// skylight rather than going black.
//
// The march is NOT along the true sun (elevation 34°): that low an angle throws the
// shadow ~145 units — clear off the 120-unit island, onto the untextured sea, so the
// ground under the cloud read as unshadowed. We march a STEEPER pseudo-sun instead
// (same azimuth, high elevation), which lands the shadow on the island just off nadir
// beneath the cloud. That is where the eye expects a cloud's shadow, and it is the
// usual stylisation — a small, honest lie about the sun angle for a legible shadow.
constexpr int SHADOW_RES = 96;          // soft shadow -> low-res map is plenty (was 192)
constexpr double SHADOW_PROJ_EL = 66.0; // steeper than the sun's 34° so it lands on-island
constexpr double SHADOW_K = 0.13;       // optical-depth -> darkness (a soft, clear patch)
constexpr double SHADOW_FLOOR = 0.55;   // shaded ground keeps 55% of its light

// Trilinear sample of a sky density field at a WORLD point (0 outside the slab).
float sampleSky(const std::vector<float> &field, double wx, double wy, double wz) {
  if (wz < SKY_BASE || wz > SKY_TOP)
    return 0.0f;
  double fx = (wx + SKY_HALF) / (2.0 * SKY_HALF) * (SKY_N - 1);
  double fy = (wy + SKY_HALF) / (2.0 * SKY_HALF) * (SKY_N - 1);
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

// Bake albedo × cloud-shadow over the terrain footprint [-HALF,HALF]^2 into an RGB
// buffer (SHADOW_RES^2) the terrain samples as its surface colour. Carrying the
// albedo here (not just a grey shadow) keeps the terrain coloured whether VTK
// modulates or replaces the vertex colour with the texture — and at texel rather
// than vertex resolution, so the shading is finer than the mesh.
void computeCloudShadow(const std::vector<float> &field, Vec3d sun,
                        std::vector<unsigned char> &rgb) {
  Vec3d L = vnorm(sun); // unit direction toward the sun (L.z > 0)
  const int STEPS = 22;
  rgb.resize((size_t)SHADOW_RES * SHADOW_RES * 3);
  for (int ty = 0; ty < SHADOW_RES; ++ty) {
    double y = -HALF + 2.0 * HALF * ty / (SHADOW_RES - 1);
    for (int tx = 0; tx < SHADOW_RES; ++tx) {
      double x = -HALF + 2.0 * HALF * tx / (SHADOW_RES - 1);
      double z0 = terrainH(x, y);
      double tEntry = (SKY_BASE - z0) / L.z, tExit = (SKY_TOP - z0) / L.z;
      double ds = (tExit - tEntry) / STEPS;
      double tau = 0.0;
      for (int i = 0; i < STEPS; ++i) {
        double t = tEntry + (i + 0.5) * ds;
        tau += sampleSky(field, x + t * L.x, y + t * L.y, z0 + t * L.z) * ds;
      }
      double s = SHADOW_FLOOR + (1.0 - SHADOW_FLOOR) * std::exp(-SHADOW_K * tau);
      cvc::geometry::color_t a = terrainAlbedo(x, y);
      size_t o = ((size_t)ty * SHADOW_RES + tx) * 3;
      for (int k = 0; k < 3; ++k)
        rgb[o + k] = (unsigned char)std::min(255.0, std::max(0.0, a[k] * s * 255.0));
    }
  }
}

// Re-run one tree's wind cascade, writing its posed vertices into the SHARED forest
// buffers at this tree's offsets. No upload here — the caller re-poses every tree
// and then uploads the two merged buffers ONCE (updateVertices) per frame.
// `windScale` multiplies every tree's own sway amplitude, so the UI slider is one
// global knob over a forest whose trees each keep their individual amplitude and
// phase. 1.0 is the tuned default; 0 is dead calm.
void reposeTree(const Tree &tree, double t, std::vector<double> &woodBuf,
                std::vector<double> &needleBuf, double windScale = 1.0) {
  double a = tree.sway * windScale * std::sin(1.3 * t + tree.phase);
  Mat4 sway = mRot(a, 0.0, 1.0, 0.0); // TREE_AXIS = +Y (tree-local)
  std::vector<Mat4> world(tree.mods.size());
  for (size_t i = 0; i < tree.mods.size(); ++i) {
    const ModRec &m = tree.mods[i];
    Mat4 local = m.swayer ? mMul(m.hang, sway) : m.hang;
    world[i] = (m.parent < 0) ? local : mMul(world[m.parent], local);
    int wo = m.wOff;
    for (const Vec3d &p : m.localWood) {
      Vec3d w = xform(world[i], p);
      woodBuf[wo * 3] = w.x;
      woodBuf[wo * 3 + 1] = w.y;
      woodBuf[wo * 3 + 2] = w.z;
      ++wo;
    }
    int no = m.nOff;
    for (const Vec3d &p : m.localNeedle) {
      Vec3d w = xform(world[i], p);
      needleBuf[no * 3] = w.x;
      needleBuf[no * 3 + 1] = w.y;
      needleBuf[no * 3 + 2] = w.z;
      ++no;
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  bool offscreen = false, noShadows = false, verbose = false;
  int frames = 0, width = 1280, height = 800;
  double fps = 30.0;
  std::string png, captureStr, outDir;
  po::options_description desc(
      "lsystem_coast — a pure-C++ cvcGL island + GPU FFT-ocean demo\nOptions");
  desc.add_options()("help,h", "show this help and exit")                                   //
      ("offscreen", po::bool_switch(&offscreen), "render offscreen (no window)")            //
      ("no-shadows", po::bool_switch(&noShadows), "disable tree shadows (a shadow map)")    //
      ("verbose,v", po::bool_switch(&verbose), "show cvcGL debug logging (off by default)") //
      ("frames", po::value<int>(&frames)->default_value(0),                                 //
       "stop after N frames (0 = run until the window closes)")                             //
      ("png", po::value<std::string>(&png)->default_value(""),                              //
       "after the run, write the final frame to this PNG")                                  //
      ("capture", po::value<std::string>(&captureStr)->default_value("none"),               //
       "cinematic capture path: none | orbit | fly (forces --offscreen)")                   //
      ("width", po::value<int>(&width)->default_value(1280), "render width in pixels")      //
      ("height", po::value<int>(&height)->default_value(800), "render height in pixels")    //
      ("fps", po::value<double>(&fps)->default_value(30.0),                                 //
       "capture frames per second (drives the synthetic clock)")                            //
      ("out", po::value<std::string>(&outDir)->default_value("frames"),                     //
       "capture output directory for the numbered PNGs");
  bool no_ui = false, shoreCheck = false;
  std::vector<std::string> sets;
  desc.add_options()("shore-check", po::bool_switch(&shoreCheck),
                     "after the run, check that no displaced ocean vertex lands on dry ground "
                     "(prints the counts; exit code 3 if one does)");
  desc.add_options()("set", po::value<std::vector<std::string>>(&sets)->composing(),
                     "KEY=VALUE: set a state knob before the scene seeds it (repeatable), e.g. "
                     "--set coast.water.wind_speed=3 --set coast.water.swell_hs=2");
  desc.add_options()("no-ui", po::bool_switch(&no_ui),
                     "hide the ImGui overlay (default: hidden while capturing)");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << "\n\n" << desc << "\n";
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  enum class Capture { None, Orbit, Fly } capture = Capture::None;
  if (captureStr == "orbit")
    capture = Capture::Orbit;
  else if (captureStr == "fly")
    capture = Capture::Fly;
  else if (captureStr != "none") {
    std::cerr << "unknown --capture '" << captureStr << "' (want none | orbit | fly)\n";
    return 2;
  }
  const bool capturing = capture != Capture::None;
  if (capturing) {
    offscreen = true; // a capture path always renders offscreen to numbered PNGs
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
  }

  // Own the app and inject it — no global/singleton context.
  cvc::app app;
  // --set: the knobs seed a default only where a key is empty, so these win.
  for (const std::string &kv : sets) {
    const std::size_t eq = kv.find('=');
    if (eq == std::string::npos || eq == 0) {
      std::cerr << "--set wants KEY=VALUE, got '" << kv << "'\n";
      return 2;
    }
    const std::string key = kv.substr(0, eq), val = kv.substr(eq + 1);
    try {
      cvc::state::instance(app)(key).value(std::stod(val));
    } catch (const std::exception &) {
      cvc::state::instance(app)(key).value(val);
    }
  }
  // The scene's nodes log through THIS injected app (not a process-wide singleton),
  // so we set the console verbosity right here: show only errors + warnings, unless
  // --verbose asks for the full per-frame cvcGL debug trace. Levels: a message at
  // level L prints when L < log_verbosity (0=error, 1=warning, 2=info, 3+=debug).
  app.properties("system.log_verbosity", verbose ? "6" : "2");
  SceneGraph sg(app, "coast");

  sg.addGraphics("terrain", buildTerrain(app));
  auto terrain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"));
  // The surface colour comes from the cloud-shadow texture (albedo × shadow), so
  // the mesh itself is flat white — the texture is the sole albedo whether VTK
  // modulates or replaces the base colour with it.
  terrain->setUseSingleColor(true);
  terrain->setColor(1.0, 1.0, 1.0);
  terrain->setAmbient(0.45); // keep the ground bright where the sun / cloud shade it
  terrain->setDiffuse(0.85);
  addTerrainBump(*terrain);

  // Plant an L-system forest on the dry land. Every tree's geometry is merged into
  // ONE wood mesh + ONE needle mesh for the WHOLE forest (route C across trees):
  // 32 trees -> 2 actors, so the per-frame wind is 2 vertex uploads and 2 draw
  // calls (main pass + shadow bake), not 64. This is the single biggest realtime
  // win — the per-actor upload/draw overhead dominated the frame.
  CylTopo cyl = cylTopo();
  std::vector<Vec3d> nring;
  for (int t = 0; t < NEEDLES; ++t) {
    double a = t * 2.0 * M_PI / NEEDLES;
    nring.push_back({std::cos(a), 0.0, std::sin(a)});
  }
  Mat4 tMicro = mRot(TILT * MICRO_TILT, 0, 0, 1), tTilt = mRot(TILT, 0, 0, 1),
       tRoll = mRot(YROTATE, 0, 1, 0);
  std::mt19937 rng(20260817u);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::vector<Tree> forest;
  cvc::geometry forestWood(app), forestNeedle(app); // the merged forest meshes
  const int MAX_TREES = 55;
  int planted = 0;
  for (int gy = -4; gy <= 4 && planted < MAX_TREES; ++gy)
    for (int gx = -4; gx <= 4 && planted < MAX_TREES; ++gx) {
      double x = gx * 24.0 + (u01(rng) - 0.5) * 16.0;
      double y = gy * 24.0 + (u01(rng) - 0.5) * 16.0;
      double h = terrainH(x, y);
      if (h < SEA_LEVEL + 1.5)
        continue;
      double size = 0.32 + 0.43 * u01(rng);
      int maturity = MATURITY[rng() % 7];
      std::vector<Module> mods;
      expandTree(TREE_RULES[0], maturity, size, size, -1, 1, mods, tMicro, tTilt, tRoll);
      Tree tr = buildTree(app, x, y, h, mods, cyl, nring, forestWood, forestNeedle);
      tr.phase = u01(rng) * 2.0 * M_PI;
      tr.sway = 0.020 + 0.016 * u01(rng);
      forest.push_back(std::move(tr));
      ++planted;
    }

  // One wood actor (per-vertex colour + procedural bark) and one needle actor for
  // the whole forest. High material ambient softens the shadow map's self-shadowing
  // on the thin trunks/line needles: a fully self-shadowed sample still keeps this
  // much light, so aliased shadows read as a gentle dapple, not harsh dark speckle.
  sg.addGraphics("coast_wood", forestWood);
  auto woodNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("coast_wood"));
  woodNode->setUseSingleColor(false);
  woodNode->setAmbient(0.5);
  woodNode->setDiffuse(0.8);
  addBark(*woodNode);
  std::vector<double> woodBuf = flattenPoints(forestWood);
  std::shared_ptr<GeometryNode> needleNode;
  std::vector<double> needleBuf;
  if (forestNeedle.points().size()) {
    sg.addGraphics("coast_needle", forestNeedle);
    needleNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("coast_needle"));
    needleNode->setRenderMode(GeometryRenderMode::LINES);
    needleNode->setUseSingleColor(true);
    needleNode->setColor(C_NEEDLE.x, C_NEEDLE.y, C_NEEDLE.z);
    needleNode->setAmbient(0.55);
    needleNode->setDiffuse(0.75);
    needleBuf = flattenPoints(forestNeedle);
  }

  // The ocean (a GPU FFT sea) is added AFTER the camera is framed on the island —
  // its grid spans out to the horizon (±OCEAN_HALF), and folding that into the
  // framing bounds would shrink the island. See the "ocean" block below the camera.

  // The sky: a drifting, evolving cloud slab, high above the island. Unlike the
  // sun (430 units out — pure scenery, added after framing) the cloud ceiling is
  // part of the scene the eye takes in, so it IS folded into the framing bounds:
  // the island then fills the lower frame with the clouds drifting overhead. Real
  // voxels first, then the node, then the transfer function (same order as the sea).
  SkyModel sky = buildSky();
  auto skyNode = sg.addGraphics("coast_sky", skyVolume(app, sky.field(0.0, 0.0)));
  // The billowed field HAS shape, so a directional light picks out the tops and
  // leaves the undersides dim — most of what makes cloud read as volume, not fog.
  // Ambient stays high so the shadowed side is grey-blue rather than black.
  // DIFFUSE shading, not volumetric scattering: the fractal (fBm) detail baked into
  // the field gives the density sharp gradients, so a plain directional diffuse term
  // picks out the fluffy billows — bright cauliflower tops, gently shaded hollows —
  // WITHOUT the dark rim the multi-scatter model draped around every blob. Ambient
  // is high so the cloud stays bright and white (a fair-weather cumulus, not a storm
  // cloud) and never gets a black outline. No scattering (0) => no self-shadow crust.
  skyNode->setShading(false);
  skyNode->setAmbient(0.95);
  skyNode->setDiffuse(0.35);
  skyNode->setSpecular(0.0);
  skyNode->setVolumetricScattering(0.0);
  {
    std::vector<double> col, op;
    skyTransfer(col, op);
    skyNode->setTransferFunction(col, op);
  }

  // The cloud's shadow on the ground: bake the initial transmittance into the
  // terrain's texture (kept in sync with the drifting cloud each update below).
  std::vector<unsigned char> shadowBuf;
  computeCloudShadow(sky.field(0.0, 0.0), sunDir(SUN_AZ, SHADOW_PROJ_EL), shadowBuf);
  {
    cvc::image shadowImg(SHADOW_RES, SHADOW_RES, cvc::image::pixel_format::RGB,
                         cvc::image::data_type::u8, shadowBuf.data());
    terrain->setTexture(shadowImg, false);
  }

  // STAGE RIG, not a sun. Two directional lights used to stand in for an
  // afternoon sun and a sky fill, and they were the reason the shadows were
  // mush: VTK bakes a directional light's shadow map with a parallel projection
  // fitted to the WHOLE scene bounding box, and this scene's box includes the
  // sea plane and a sun billboard 900 units out. Nearly every shadow texel was
  // being spent on empty water.
  //
  // A spot is baked with a perspective projection whose view angle is its cone,
  // so aiming the rig at the ISLAND concentrates the map where the trees are.
  // The stage radius is the island, deliberately not the scene.
  cvc::gl::StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.0, 120.0); // the island, not the sea and not the sky
  rig.applyPreset(cvc::gl::StageLighting::Preset::ThreePoint);
  // Keep the old warm-key/cool-fill feel, now as a rig rather than a sky.
  rig.setKey(1.15, -52.0, 34.0, 34.0);
  rig.setWarmth(0.45);

  SceneRenderer view(sg, width, height, offscreen, "main");
  // The loop yields to the browser once per frame (emscripten_sleep(0) at its end), so VTK
  // need not yield again inside every render (FrameYield::App: VTK 9.5's in-render
  // emscripten_sleep off; a no-op natively).
  view.setFrameYield(SceneRenderer::FrameYield::App);
  // A real sky, not a flat void: a vertical gradient background (hazy horizon at
  // the bottom, deep blue at the zenith). Done on the renderer rather than as a
  // sky sphere on purpose — an enclosing sky sphere would occlude the directional
  // light in the shadow-map pass and plunge the whole island into shadow. The sun
  // itself is a camera-relative disc (below), so the two together read as sky.
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.66, 0.71, 0.74);  // horizon (screen bottom)
  view.renderer()->SetBackground2(0.23, 0.44, 0.80); // zenith (screen top)

  // Tree-cast shadows on (--no-shadows to drop them). A high-res map keeps the thin
  // trunks/needles from tearing badly; the trees' high material ambient (set above)
  // softens the residual self-shadow speckle that a shadow map always gets on line/
  // thin geometry. Capture re-bakes EVERY frame so the wind-blown shadows don't snap.
  const bool shadows = !noShadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 1 : 3);
  }

  // Built-in navigation, framed on the island.
  CameraController cam(view);
  cvc::bounding_box b = sg.computeGraphicsBounds();
  cam.frameBounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);

  // ── the ocean: a GPU spectral FFT sea (OceanFFT), added AFTER framing ────────
  // Added here, past frameBounds, so its ±OCEAN_HALF grid (out to the horizon)
  // doesn't shrink the island in the frame. Hide the lab decorations first so the
  // GL-init render below doesn't flash them, then render ONCE so the window's
  // OpenGLInit has run — vtkTextureObject only reports float-RT support after that
  // (otherwise Create2DFromRaw sees IF=0 and we would wrongly fall back to a flat sea).
  sg.setDiagnosticChromeVisible(false);
  view.render();

  // Water knobs live in the state tree at coast.water.* / coast.clock.* — scriptable
  // from Python (cvc.state) and driven live by the ImGui "Water" panel; the loop reads
  // them each frame. knob() seeds a default only when the key is empty (so a value set
  // before this point — e.g. from a script — wins).
  auto knob = [&](const char *path, double def) -> double {
    cvc::state &nd = cvc::state::instance(app)(path);
    if (nd.value().empty())
      nd.value(def);
    return nd.value<double>();
  };

  // The sea state (cvc::ocean, OCEAN-AND-WEATHER-ROADMAP.md B1 / E1): a wind sea and a
  // swell, ONE spectrum split across 3 cascades (swell + wind-waves + ripples, tiles
  // at non-harmonic sizes) by wavenumber band, so no band is counted twice, each with
  // its own noise. Its h0 is built on the CPU (cvc::ocean::build_h0) and uploaded, the
  // numbers the physics band will evolve too. Each cascade's amplitude is its artistic
  // weight, the knob coast.water.weight_long / _mid / _short (the 251, 83 and 29 m
  // tiles), baked into h0 so physics sees it too; the shader sums the cascades
  // unweighted. All default to 1, the one physical spectrum. (The old weights, 0.55
  // and 0.32, tamed cascades that each held a whole spectrum, the peak included;
  // band-limited, the small ones hold only short waves, and those weights left the sea
  // glassy.) The swell (coast.water.swell_*, 1.2 m at 10 s by default) is not in the
  // cascades: the shore layer below carries it, refracting, shoaling and breaking on this
  // coast (fftSea leaves it out of h0). The island's shelf is shallow out to the horizon,
  // so the swell is the shore layer's everywhere here.
  auto seaState = [&] {
    cvc::ocean::sea_state s;
    s.wind.speed_mps = knob("coast.water.wind_speed", 9.0);
    s.wind.fetch_m = knob("coast.water.fetch", 120000.0);
    s.wind.dir_rad =
        std::atan2(knob("coast.water.wind_dir_z", 0.55), knob("coast.water.wind_dir_x", 1.0));
    s.depth_m = knob("coast.water.depth", 1000.0);
    s.swell.hs_m = knob("coast.water.swell_hs", 1.2);
    s.swell.tp_s = knob("coast.water.swell_period", 10.0);
    s.swell.dir_rad =
        std::atan2(knob("coast.water.swell_dir_z", 0.3), knob("coast.water.swell_dir_x", 1.0));
    return s;
  };
  // The sea state the cascades draw: the wind sea alone (the shore layer has the swell).
  auto fftSea = [](cvc::ocean::sea_state s) {
    s.swell.hs_m = 0.0;
    return s;
  };
  // The swell the shore layer draws.
  auto swellOf = [](const cvc::ocean::sea_state &s) {
    cvc::ocean::shore_swell sw;
    sw.hs_m = s.swell.hs_m;
    sw.tp_s = s.swell.tp_s;
    sw.dir_rad = s.swell.dir_rad;
    return sw;
  };
  // The cascades' weights, long to short.
  const char *const kBandWeight[3] = {"coast.water.weight_long", "coast.water.weight_mid",
                                      "coast.water.weight_short"};
  // Every knob h0 reads, to tell when it must be rebuilt: the sea state's, then the weights.
  auto seaSignature = [&](const cvc::ocean::sea_state &s) {
    return std::vector<double>{s.wind.speed_mps,
                               s.wind.fetch_m,
                               s.wind.dir_rad,
                               s.depth_m,
                               s.swell.hs_m,
                               s.swell.tp_s,
                               s.swell.dir_rad,
                               knob(kBandWeight[0], 1.0),
                               knob(kBandWeight[1], 1.0),
                               knob(kBandWeight[2], 1.0)};
  };
  struct Cascade {
    std::unique_ptr<OceanFFT> fft;
    cvc::ocean::cascade band;
  };
  std::vector<Cascade> cascades;
  {
    std::vector<cvc::ocean::cascade> bands =
        cvc::ocean::make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
    const cvc::ocean::sea_state s0 = seaState();
    for (std::size_t i = 0; i < bands.size(); ++i) {
      Cascade cc;
      cc.band = bands[i];
      cc.band.amplitude = knob(kBandWeight[i], 1.0);
      cc.fft = std::make_unique<OceanFFT>(FFT_N);
      cc.fft->depth_m = static_cast<float>(s0.depth_m);
      cc.fft->chop = static_cast<float>(knob("coast.water.chop", 1.15));
      cc.fft->setH0(cvc::ocean::build_h0(FFT_N, cc.band, fftSea(s0)));
      cascades.push_back(std::move(cc));
    }
  }
  knob("coast.water.wave_amp", 1.2);   // vertical scale (raw FFT height is metres)
  knob("coast.water.choppiness", 1.0); // horizontal (Gerstner) displacement scale
  knob("coast.water.foam_bias", 0.48); // Jacobian threshold below which wave foam appears
  knob("coast.water.shore_foam", 1.0); // shoreline surf intensity (0 disables it)
  knob("coast.clock.scale", 1.0);      // world seconds per wall second (0 pauses the sea)
  knob("coast.clock.paused", 0.0);

  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(view.renderWindow());
  bool oceanReady = ow != nullptr;
  for (auto &c : cascades)
    oceanReady = oceanReady && c.fft->init(ow);
  if (oceanReady) {
    const bool stpass = cascades[0].fft->selfTest();
    std::printf("OceanFFT self-test: %s (%zu-cascade spectral FFT at %dx%d)\n",
                stpass ? "PASS" : "FAIL", cascades.size(), FFT_N, FFT_N);
  } else {
    std::printf("OceanFFT unavailable (no float render targets?) — rendering a flat sea.\n");
  }

  // The seabed and the shore (OCEAN-AND-WEATHER-ROADMAP.md A1, W0.2): the island's ground
  // sampled every metre over the ocean (terrainH, which the terrain mesh is built from),
  // its bodies of water below SEA_LEVEL and their shore grid (cvc::world::water_bodies).
  // The seabed goes to the GPU as a HeightFieldTexture, the shore grid as an RGBA32F
  // texture (distance to the shore, + offshore; the offshore normal; the slope).
  const cvc::world::grid_spec seaGrid = cvc::world::grid_spec::window(0.0, 0.0, OCEAN_HALF, 1.0);
  std::vector<float> seabed(seaGrid.count());
  for (int r = 0; r < seaGrid.rows; ++r)
    for (int c = 0; c < seaGrid.cols; ++c)
      seabed[std::size_t(r) * seaGrid.cols + c] =
          static_cast<float>(terrainH(seaGrid.world_x(c), seaGrid.world_y(r)));
  cvc::world::water_bodies_params shoreParams;
  shoreParams.level_m = SEA_LEVEL;
  shoreParams.normal_smooth_m = 8.0; // about the ripples' wavelength; O4 smooths for the swell
  const cvc::world::water_bodies shore =
      cvc::world::build_water_bodies(seabed, seaGrid, shoreParams);
  std::printf("coast: %s on a %d x %d shore grid\n",
              cvc::world::water_bodies_summary(shore).c_str(), seaGrid.cols, seaGrid.rows);
  cvc::gl::HeightFieldTexture seabedTex(seaGrid.cols, seaGrid.rows, seaGrid.min_x, seaGrid.min_y,
                                        seaGrid.cell_w(), seaGrid.cell_h());
  seabedTex.setHeights(seabed);
  vtkSmartPointer<vtkTextureObject> shoreTex;

  // The shore layer (OCEAN-AND-WEATHER-ROADMAP.md A3): the swell refracting over this shelf,
  // shoaling, breaking on the beaches that face it and reflecting off the steep shores, on the
  // shore grid. The vertex shader draws its trains from two RGBA32F textures; the CPU evaluates
  // the same sum (cvc::ocean::shore_layer::evaluate). Rebuilt when a swell knob changes.
  cvc::ocean::shore_grid shoreGrid;
  shoreGrid.rows = seaGrid.rows;
  shoreGrid.cols = seaGrid.cols;
  shoreGrid.min_x = seaGrid.min_x;
  shoreGrid.min_y = seaGrid.min_y;
  shoreGrid.cell = seaGrid.cell_w();
  shoreGrid.depth = shore.depth;
  shoreGrid.shore_m = shore.shore_m;
  shoreGrid.normal_x = shore.normal_x;
  shoreGrid.normal_y = shore.normal_y;
  cvc::ocean::shore_layer_params shoreLayerParams;
  shoreLayerParams.trains = 3; // the shader sums three
  shoreLayerParams.seed = 0x5E11ull;
  auto buildShoreLayer = [&](const cvc::ocean::sea_state &s) {
    const auto t0 = std::chrono::steady_clock::now();
    shoreLayerParams.dry_time_s = knob("coast.water.dry_time", 6.0); // wet sand's drying time
    cvc::ocean::shore_layer L =
        cvc::ocean::shore_layer::build(shoreGrid, swellOf(s), shoreLayerParams);
    std::printf(
        "coast: shore layer for a %.1f m, %.1f s swell in %.0f ms\n", s.swell.hs_m, s.swell.tp_s,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    return L;
  };
  cvc::ocean::shore_layer shoreLayer = buildShoreLayer(seaState());
  auto swellSignature = [&](const cvc::ocean::sea_state &s) {
    return std::vector<double>{s.swell.hs_m, s.swell.tp_s, s.swell.dir_rad,
                               knob("coast.water.dry_time", 6.0)};
  };
  std::vector<double> lastSwell = swellSignature(seaState());
  vtkSmartPointer<vtkTextureObject> shoreWaveTex, shoreDirTex;

  // The ocean grid + GPU displace/BRDF shaders (no per-frame CPU readback — the
  // WASM-viable path). The cascade displacement textures are bound live.
  sg.addGraphics("ocean", buildOcean(app));
  auto ocean = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("ocean"));
  ocean->setUseSingleColor(true); // colour comes from the BRDF, not the vertex colours
  if (oceanReady) {
    ocean->disableCoordinateShiftScale(); // vertexMC.xy == world xy
    const char *dispName[3] = {"uDisp0", "uDisp1", "uDisp2"};
    const char *tileName[3] = {"uTile0", "uTile1", "uTile2"};
    for (size_t i = 0; i < cascades.size() && i < 3; ++i) {
      ocean->setShaderTexture(dispName[i], cascades[i].fft->displacement());
      ocean->setShaderUniformf(tileName[i], static_cast<float>(cascades[i].band.tile_m));
    }
    ocean->setShaderUniformf("uHalfTexel", 0.5f / static_cast<float>(FFT_N));
    // The seabed and the shore grid (the context is current: the cascades just made theirs).
    std::vector<float> shoreTexels(seaGrid.count() * 4);
    for (std::size_t i = 0; i < seaGrid.count(); ++i) {
      shoreTexels[i * 4] = shore.shore_m[i];
      shoreTexels[i * 4 + 1] = shore.normal_x[i];
      shoreTexels[i * 4 + 2] = shore.normal_y[i];
      shoreTexels[i * 4 + 3] = shore.slope[i];
    }
    shoreTex = vtkSmartPointer<vtkTextureObject>::New();
    shoreTex->SetContext(ow);
    vtkTextureObject *bedTex = seabedTex.prepare(ow);
    if (!bedTex ||
        !shoreTex->Create2DFromRaw(seaGrid.cols, seaGrid.rows, 4, VTK_FLOAT, shoreTexels.data()))
      std::printf("coast: the seabed or shore texture failed; the shore is not damped\n");
    shoreTex->SetMinificationFilter(vtkTextureObject::Nearest);
    shoreTex->SetMagnificationFilter(vtkTextureObject::Nearest);
    shoreTex->SetWrapS(vtkTextureObject::ClampToEdge);
    shoreTex->SetWrapT(vtkTextureObject::ClampToEdge);
    ocean->setShaderTexture(cvc::gl::HeightFieldTexture::samplerName(), bedTex);
    ocean->setShaderUniform3f(cvc::gl::HeightFieldTexture::originName(),
                              static_cast<float>(seaGrid.min_x), static_cast<float>(seaGrid.min_y),
                              0.0f);
    ocean->setShaderUniform3f(cvc::gl::HeightFieldTexture::spacingName(),
                              static_cast<float>(seaGrid.cell_w()),
                              static_cast<float>(seaGrid.cell_h()), 0.0f);
    ocean->setShaderTexture("uShoreGrid", shoreTex);
    ocean->setShaderUniform3f("uShoreGridOrigin", static_cast<float>(seaGrid.min_x),
                              static_cast<float>(seaGrid.min_y), 0.0f);
    ocean->setShaderUniform3f("uShoreGridSpacing", static_cast<float>(seaGrid.cell_w()),
                              static_cast<float>(seaGrid.cell_h()), 0.0f);
    ocean->setShaderUniformf("uLevel", static_cast<float>(SEA_LEVEL));
    ocean->addVertexShaderReplacement("//VTK::PositionVC::Dec", oceanVsDec());
    ocean->addVertexShaderReplacement("//VTK::PositionVC::Impl", OCEAN_VS_IMPL);
    ocean->addFragmentShaderReplacement("//VTK::Normal::Dec", oceanFsDec());
    ocean->addFragmentShaderReplacement("//VTK::Light::Impl", OCEAN_FS_LIGHT);
  } else {
    ocean->setColor(0.05, 0.20, 0.32); // a flat blue fallback sea
    ocean->setAmbient(0.6);
    ocean->setDiffuse(0.5);
  }
  // The shore layer to the GPU: its two textures and its trains (on the render thread, the
  // context current). Again whenever the swell changes.
  auto uploadShoreLayer = [&] {
    if (!oceanReady)
      return;
    auto floatTexture = [&](vtkSmartPointer<vtkTextureObject> &tex, std::vector<float> texels) {
      if (!tex) {
        tex = vtkSmartPointer<vtkTextureObject>::New();
        tex->SetContext(ow);
      }
      if (!tex->Create2DFromRaw(shoreLayer.cols, shoreLayer.rows, 4, VTK_FLOAT, texels.data()))
        std::printf("coast: a shore layer texture failed\n");
      tex->SetMinificationFilter(vtkTextureObject::Nearest);
      tex->SetMagnificationFilter(vtkTextureObject::Nearest);
      tex->SetWrapS(vtkTextureObject::ClampToEdge);
      tex->SetWrapT(vtkTextureObject::ClampToEdge);
    };
    floatTexture(shoreWaveTex, shoreLayer.wave_texels());
    floatTexture(shoreDirTex, shoreLayer.dir_texels());
    ocean->setShaderTexture("uShoreWave", shoreWaveTex);
    ocean->setShaderTexture("uShoreDir", shoreDirTex);
    ocean->setShaderUniformf("uSwellOmega", static_cast<float>(shoreLayer.omega));
    float v[4][3] = {};
    for (std::size_t i = 0; i < shoreLayer.trains.size() && i < 3; ++i) {
      v[0][i] = static_cast<float>(shoreLayer.trains[i].ratio);
      v[1][i] = static_cast<float>(shoreLayer.trains[i].amplitude);
      v[2][i] = static_cast<float>(shoreLayer.trains[i].phase);
      v[3][i] = static_cast<float>(shoreLayer.trains[i].phase_reflected);
    }
    const char *names[4] = {"uTrainRatio", "uTrainAmp", "uTrainPhase", "uTrainPhaseR"};
    for (int j = 0; j < 4; ++j)
      ocean->setShaderUniform3f(names[j], v[j][0], v[j][1], v[j][2]);
    // The swash reads the same two textures on the terrain.
    terrain->setShaderTexture("uSwashWave", shoreWaveTex);
    terrain->setShaderTexture("uSwashDir", shoreDirTex);
    terrain->setShaderUniform3f("uSwashOrigin", static_cast<float>(shoreLayer.min_x),
                                static_cast<float>(shoreLayer.min_y), 0.0f);
    terrain->setShaderUniform3f("uSwashSpacing", static_cast<float>(shoreLayer.cell),
                                static_cast<float>(shoreLayer.cell), 0.0f);
    terrain->setShaderUniformf("uSwashOmega", static_cast<float>(shoreLayer.omega));
    terrain->setShaderUniformf(
        "uSwashPhase0",
        shoreLayer.trains.empty() ? 0.0f : static_cast<float>(shoreLayer.trains.front().phase));
    terrain->setShaderUniformf("uDryTime", static_cast<float>(shoreLayer.dry_time_s));
    terrain->setShaderUniformf("uSwashLevel", static_cast<float>(SEA_LEVEL));
  };
  if (oceanReady)
    addTerrainSwash(*terrain); // the swash on the sand, from the shore layer's textures
  uploadShoreLayer();

  // The app's world clock drives EVERY animated layer by world time (the sea's FFT, the
  // tree wind, the cloud drift and its ground shadow): correct speed at any frame rate,
  // one pause and one speed for all of them, deterministic under a fixed-frame capture.
  // The frame loop drives it through a binding whose keys are this demo's own knobs
  // (coast.clock.scale / .paused, publishing coast.clock.t), plus mode / step / tick /
  // ... beside them under coast.clock.
  cvc::world_clock &clock = app.world_clock();
  cvc::world_clock_keys clockKeys = cvc::world_clock_keys::under("coast.clock");
  clockKeys.speed = "coast.clock.scale";
  clockKeys.time = "coast.clock.t";
  cvc::world_clock_binding clockDrive(app, clock, clockKeys);
  std::vector<double> lastSea = seaSignature(seaState());

  // FPS readout drawn by the scene itself ('f' toggles; state
  // coast.viewers.main.hud.*). Off for captures so frames stay clean.
  cvc::gl::FpsHud hud(view);

  // Touch gestures work with or without the ImGui overlay (phones have no mouse:
  // pinch = zoom, two-finger drag = pan/turn). Declared UNCONDITIONALLY — the
  // render loop calls touch.update() and reads uiWind every frame regardless of
  // whether the ImGui control panel was compiled in (a -DCVC_ENABLE_IMGUI=OFF
  // build otherwise left both undeclared: the Windows/native no-UI build gap).
  cvc::gl::TouchGestures touch(view, cam);
  float uiWind = 1.0f; // wind strength; the ImGui "wind" slider drives it when built

  // Dear ImGui overlay — real widgets, same code path native and in the browser.
  // Everything the callback touches lives in this scope; ImGui is immediate-mode,
  // so there is no widget tree to keep in sync.
#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  // The overlay creates its Dear ImGui context INSIDE libcvc. This .exe links its
  // own static copy of imgui (imgui exports no symbols), so its ImGui::* calls use a
  // SEPARATE, uninitialised context — the first ImGui::Begin below would dereference
  // a null GImGui and crash. Adopt the overlay's context so the whole process shares
  // one. (cvc::gl::ui::* widgets already run inside libcvc, but the window + menu bar
  // here are raw ImGui::* from this module, so they need the shared context.)
  ImGui::SetCurrentContext(ui.imguiContext());
  // A capture is a deliverable: no control panel in the frames unless asked.
  ui.setVisible(!no_ui && !capturing && !offscreen);
  bool uiScene = false;   // the library's ScenePanel window
  bool uiLighting = true; // the StageLightingPanel window
  // "Ocean" panel mirrors, seeded from the state knobs. The sliders write straight
  // back to the state tree (coast.water.* / coast.clock.*), so ImGui, Python and the
  // render loop all agree on one source of truth.
  float uiOceanWind = static_cast<float>(knob("coast.water.wind_speed", 9.0));
  float uiSwellHs = static_cast<float>(knob("coast.water.swell_hs", 1.2));
  float uiSwellTp = static_cast<float>(knob("coast.water.swell_period", 10.0));
  float uiBand[3] = {static_cast<float>(knob(kBandWeight[0], 1.0)),
                     static_cast<float>(knob(kBandWeight[1], 1.0)),
                     static_cast<float>(knob(kBandWeight[2], 1.0))};
  float uiWaveAmp = static_cast<float>(knob("coast.water.wave_amp", 1.2));
  float uiChop = static_cast<float>(knob("coast.water.choppiness", 1.0));
  float uiFoam = static_cast<float>(knob("coast.water.foam_bias", 0.48));
  float uiShore = static_cast<float>(knob("coast.water.shore_foam", 1.0));
  float uiClock = static_cast<float>(knob("coast.clock.scale", 1.0));
  bool uiPaused = knob("coast.clock.paused", 0.0) > 0.5;
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      cvc_examples::demos_menu("lsystem_coast");
      if (ImGui::BeginMenu("Scene")) {
        // The library's menu: shadows, shadow-map resolution, bake interval and
        // the diagnostic chrome, all read back from the scene rather than from
        // demo-side mirrors that can drift out of agreement with it.
        cvc::gl::ui::SceneMenuItems(sg, &uiScene, &uiLighting);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::SetNextWindowPos(ImVec2(12, 34), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("lsystem_coast");
    ImGui::Text("%zu trees", forest.size());
    // Tree wind is this demo's own knob. Shadows used to be duplicated here over
    // the same local bool as the menu item, and BOTH discarded the return of
    // setShadowsEnabled(), so either could leave a tick reading ON over a scene
    // with shadows off. They now come from the library's Scene menu and panel,
    // which read the scene back.
    ImGui::SliderFloat("tree wind", &uiWind, 0.0f, 3.0f);

    // The water controls write straight to the state tree; the render loop reads
    // those knobs each frame (and rebuilds the FFT spectrum only when wind changes).
    ImGui::SeparatorText("Ocean");
    auto wput = [&](const char *path, double v) { cvc::state::instance(app)(path).value(v); };
    if (ImGui::SliderFloat("wind speed", &uiOceanWind, 0.0f, 28.0f, "%.1f m/s"))
      wput("coast.water.wind_speed", uiOceanWind);
    if (ImGui::SliderFloat("swell height", &uiSwellHs, 0.0f, 6.0f, "%.1f m"))
      wput("coast.water.swell_hs", uiSwellHs);
    if (ImGui::SliderFloat("swell period", &uiSwellTp, 4.0f, 18.0f, "%.1f s"))
      wput("coast.water.swell_period", uiSwellTp);
    // The cascades' weights (1 = the physical spectrum), long waves to ripples.
    const char *const bandLabel[3] = {"long waves", "mid waves", "ripples"};
    for (int i = 0; i < 3; ++i)
      if (ImGui::SliderFloat(bandLabel[i], &uiBand[i], 0.0f, 2.0f, "x %.2f"))
        wput(kBandWeight[i], uiBand[i]);
    if (ImGui::SliderFloat("wave height", &uiWaveAmp, 0.0f, 3.0f))
      wput("coast.water.wave_amp", uiWaveAmp);
    if (ImGui::SliderFloat("choppiness", &uiChop, 0.0f, 2.0f))
      wput("coast.water.choppiness", uiChop);
    if (ImGui::SliderFloat("wave foam", &uiFoam, 0.0f, 1.2f))
      wput("coast.water.foam_bias", uiFoam);
    if (ImGui::SliderFloat("shore foam", &uiShore, 0.0f, 2.0f))
      wput("coast.water.shore_foam", uiShore);
    if (ImGui::SliderFloat("time scale", &uiClock, 0.0f, 4.0f))
      wput("coast.clock.scale", uiClock);
    if (ImGui::Checkbox("pause waves", &uiPaused))
      wput("coast.clock.paused", uiPaused ? 1.0 : 0.0);
    ImGui::End();

    // The lighting controls are the LIBRARY's panel, not demo-local code — the
    // same call lights any cvcGL scene.
    cvc::gl::ui::ScenePanel(sg, &uiScene);
    cvc::gl::ui::StageLightingPanel(rig, &uiLighting);
  });
#endif
  if (offscreen)
    hud.setEnabled(false);

  // The sun is added AFTER framing — folding a far billboard into the bounds would
  // shrink the island. It is repositioned relative to the camera every frame so it
  // reads as an infinite sun in the light's direction.
  addSun(app, sg);
  auto sunNode = sg.getGraphics("sun");
  auto sunHalo = sg.getGraphics("sun_halo");
  const Vec3d sdir = sunDir(SUN_AZ, SUN_EL);
  // Place the sun along the sun ray FROM the eye, but CAP its sun-direction depth
  // (its coordinate along sdir) at SUN_DEPTH_CAP. A sun billboard farther out in
  // sdir would push the shadow-map's depth range past the scene, leaving the
  // island at the far end with no depth precision — it then self-shadows to black.
  // Capping keeps the sun within the scene's depth span (so it never darkens it),
  // holds its angular size constant via scale, and hides it when the camera looks
  // away from the sun (dd <= 0). Never occludes the island (it is up the sun ray).
  auto placeSky = [&](const Vec3d &eye) {
    const double eDotS = eye.x * sdir.x + eye.y * sdir.y + eye.z * sdir.z;
    const double dd = std::min(SUN_REF_DIST, SUN_DEPTH_CAP - eDotS);
    const bool vis = dd > 1.0;
    sunNode->setVisible(vis);
    sunHalo->setVisible(vis);
    if (!vis)
      return;
    const double sc = dd / SUN_REF_DIST; // angular size stays SUN_DISC_R / SUN_REF_DIST
    sunNode->setScale(sc, sc, sc);
    sunHalo->setScale(sc, sc, sc);
    sunNode->setPosition(eye.x + sdir.x * dd, eye.y + sdir.y * dd, eye.z + sdir.z * dd);
    sunHalo->setPosition(eye.x + sdir.x * dd, eye.y + sdir.y * dd, eye.z + sdir.z * dd);
  };

  // This is an island, not a lab bench: hide cvcGL's default grid, axis, AND the
  // scene bounding box (the root NullGraphic shows its yellow bbox by default).
  sg.setDiagnosticChromeVisible(false);
  sg.processEvents();

  std::printf("lsystem_coast: %s, terrain %dx%d, shadows %s. Tab=orbit/fly, WASD+mouse=fly.\n",
              offscreen ? "offscreen" : "onscreen", TN, TN, shadows ? "on" : "unavailable");

  // Cinematic capture paths (--capture) drive the camera over --frames at a fixed
  // synthetic dt (so cloud drift / sea / wind play back at real time regardless of
  // render speed), writing a numbered PNG per frame to --out:
  //   orbit — a slow 360° turntable around the island.
  //   fly   — a scripted Catmull-Rom flight: in low over the sea, close fly-bys,
  //           then a rising reveal (eye keyframes below; the target stays near the
  //           centre so the island is framed throughout).
  const std::vector<Vec3d> flyEye = {{-270, -240, 50}, {-160, -80, 42}, {-40, 90, 70},
                                     {110, 120, 95},   {190, -30, 120}, {60, -210, 165}};
  const std::vector<Vec3d> flyTgt = {{0, 0, 42}, {0, 0, 42}, {0, 0, 45},
                                     {0, 0, 48}, {0, 0, 50}, {0, 0, 58}};
  auto crInterp = [](const std::vector<Vec3d> &p, double s) { // Catmull-Rom, ends clamped
    int m = (int)p.size();
    double x = s * (m - 1);
    int i = std::min((int)std::floor(x), m - 2);
    if (i < 0)
      i = 0;
    double u = x - i, u2 = u * u, u3 = u2 * u;
    auto seg = [&](double a, double b, double c, double d) {
      return 0.5 * (2 * b + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u2 +
                    (-a + 3 * b - 3 * c + d) * u3);
    };
    const Vec3d &p0 = p[std::max(i - 1, 0)], &p1 = p[i], &p2 = p[i + 1],
                &p3 = p[std::min(i + 2, m - 1)];
    return Vec3d{seg(p0.x, p1.x, p2.x, p3.x), seg(p0.y, p1.y, p2.y, p3.y),
                 seg(p0.z, p1.z, p2.z, p3.z)};
  };
  // Orbit drives the CameraController's azimuth through the state tree; read the
  // framed starting azimuth so the turn begins from the default 3/4 view.
  const std::string azPath = CameraController::viewerStatePath("coast", "main") + ".orbit.azimuth";
  double orbitAz0 = 0.0;
  if (capture == Capture::Orbit) {
    try {
      orbitAz0 = cvc::state::instance(app)(azPath).value<double>();
    } catch (...) {
    }
  }

  auto start = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  int n = 0;
  double fpsLast = 0.0; // realtime FPS readout (interactive)
  long fpsFrames = 0;
  // Interactive update cadence, decoupled by how fast each thing actually moves: the
  // wind ripples quickest, the cloud drifts slowly, its ground shadow slower still. A
  // capture refreshes everything EVERY frame (smooth, no stepping). The tree wind
  // re-poses every other frame — imperceptibly stepped for a gentle sway, and it
  // roughly halves the wind's per-frame cost. The GPU ocean evolves every frame (its
  // FFT is a handful of fragment passes — cheap — and any stride would stutter it).
  const int WIND_STRIDE = 2, CLOUD_STRIDE = 8, SHADOW_STRIDE = 16;
  while (!view.windowClosed() && !cvc_examples::demo_switch_requested()) {
    double t, dt;
    if (capturing) { // fixed synthetic clock -> real-time playback of the animation
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      dt = t - last;
      last = t;
    }
    view.processUIEvents();
    touch.update(); // apply any pinch/two-finger gesture from this frame
    // World time for this frame: the one drive of the app clock (a capture passes its
    // fixed 1/fps). Every animated layer below reads `wt`.
    clockDrive.drive(dt);
    const double wt = clock.t();
    // Wind: re-pose every tree into the two MERGED forest buffers, then upload each
    // ONCE (not per tree). Interactive playback re-poses every other frame — a gentle
    // sway is imperceptibly stepped at that cadence and it roughly halves the wind's
    // cost; a capture re-poses every frame so nothing stutters.
    if (capturing || frame % WIND_STRIDE == 0) {
      for (Tree &tr : forest)
        reposeTree(tr, wt, woodBuf, needleBuf, uiWind);
      woodNode->updateVertices(woodBuf);
      if (needleNode)
        needleNode->updateVertices(needleBuf);
    }
    // The cloud volume refreshes IN PLACE (updateScalars), NOT via setVolume —
    // setVolume re-imports the whole field every call (realloc + full range rescan +
    // transfer-function reset + heavy logging), which is what pinned the framerate at
    // ~8 fps. Its transfer function is constant, so only its scalars move. (The sea is
    // no longer a volume — it is the GPU FFT ocean, evolved below.)
    const bool cloudDue = capturing || frame % CLOUD_STRIDE == 1;
    const bool shadowDue = capturing || frame % SHADOW_STRIDE == 3;

    // Ocean: evolve the FFT cascades to world time on the GPU (no readback), then push
    // the live uniforms — the mesh shader displaces + shades straight from the bound
    // displacement textures.
    if (oceanReady) {
      // Rebuild h0 on the CPU only when a spectrum knob or a weight actually changes
      // (not free: the band's texels of each cascade, tens of milliseconds).
      const cvc::ocean::sea_state sea = seaState();
      std::vector<double> sig = seaSignature(sea);
      if (sig != lastSea) {
        for (std::size_t i = 0; i < cascades.size(); ++i) {
          Cascade &c = cascades[i];
          c.band.amplitude = knob(kBandWeight[i], 1.0);
          c.fft->depth_m = static_cast<float>(sea.depth_m);
          c.fft->setH0(cvc::ocean::build_h0(FFT_N, c.band, fftSea(sea)));
        }
        lastSea = std::move(sig);
      }
      // And the shore layer when the swell changes (a fast march over the shore grid, about a
      // hundred milliseconds).
      std::vector<double> swellSig = swellSignature(sea);
      if (swellSig != lastSwell) {
        shoreLayer = buildShoreLayer(sea);
        uploadShoreLayer();
        lastSwell = std::move(swellSig);
      }
      for (auto &c : cascades) {
        c.fft->chop = static_cast<float>(knob("coast.water.chop", 1.15));
        c.fft->step(wt);
      }
      double camp[3] = {0, 0, 0};
      if (view.renderer() && view.renderer()->GetActiveCamera())
        view.renderer()->GetActiveCamera()->GetPosition(camp);
      ocean->setShaderUniformf("uWaveAmp", static_cast<float>(knob("coast.water.wave_amp", 1.2)));
      ocean->setShaderUniformf("uChop", static_cast<float>(knob("coast.water.choppiness", 1.0)));
      ocean->setShaderUniform3f("uCamPos", static_cast<float>(camp[0]), static_cast<float>(camp[1]),
                                static_cast<float>(camp[2]));
      ocean->setShaderUniform3f("uSunDir", static_cast<float>(sdir.x), static_cast<float>(sdir.y),
                                static_cast<float>(sdir.z));
      ocean->setShaderUniformf("uFoamBias",
                               static_cast<float>(knob("coast.water.foam_bias", 0.48)));
      ocean->setShaderUniformf("uShore", static_cast<float>(knob("coast.water.shore_foam", 1.0)));
      ocean->setShaderUniformf("uTime", static_cast<float>(wt));
      ocean->setShaderUniformf("uShoreTime", static_cast<float>(wt));
      terrain->setShaderUniformf("uSwashTime", static_cast<float>(wt));
      terrain->setShaderUniformf("uSwashFoam",
                                 static_cast<float>(knob("coast.water.shore_foam", 1.0)));
      const double shoreFade = knob("coast.water.shore_fade", 3.0);
      const cvc::ocean::sea_state windSea = fftSea(sea);
      ocean->setShaderUniform3f(
          "uFade", static_cast<float>(cascadeFade(cascades[0].band, windSea, shoreFade)),
          static_cast<float>(cascadeFade(cascades[1].band, windSea, shoreFade)),
          static_cast<float>(cascadeFade(cascades[2].band, windSea, shoreFade)));
    }
    if (cloudDue || shadowDue) {
      double shift = wt * CLOUD_DRIFT * SKY_N / (2.0 * SKY_HALF);
      double morph = wt / CLOUD_MORPH_S * CLOUD_MAPS;
      std::vector<float> skyF = sky.field(shift, morph);
      if (cloudDue)
        skyNode->updateScalars(skyF); // sky drift + morph
      if (shadowDue) {                // move the cloud's soft ground shadow with it
        computeCloudShadow(skyF, sunDir(SUN_AZ, SHADOW_PROJ_EL), shadowBuf);
        cvc::image shadowImg(SHADOW_RES, SHADOW_RES, cvc::image::pixel_format::RGB,
                             cvc::image::data_type::u8, shadowBuf.data());
        terrain->setTexture(shadowImg, false);
      }
    }
    if (capture == Capture::Fly) {
      double s = frames > 1 ? double(frame) / (frames - 1) : 0.0;
      s = s * s * (3.0 - 2.0 * s); // ease in/out over the whole flight
      Vec3d e = crInterp(flyEye, s), tg = crInterp(flyTgt, s);
      view.setCamera(e.x, e.y, e.z, tg.x, tg.y, tg.z, 0, 0, 1, 42.0, 1.0, 4000.0);
      placeSky(e);
    } else {
      if (capture == Capture::Orbit && frames > 0) // one full slow turn over the run
        cvc::state::instance(app)(azPath).value(orbitAz0 + 360.0 * double(frame) / frames);
      cam.update(dt);
      double ep[3], fp[3], upv[3];
      cam.getPose(ep, fp, upv);
      placeSky({ep[0], ep[1], ep[2]});
      view.renderer()->ResetCameraClippingRange(); // reach the sun billboard
    }
    if (capturing) {
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", outDir.c_str(), frame);
      view.writePNG(path);
    } else {
      view.render();
      // Realtime FPS readout (interactive only) — averaged over ~1 s.
      ++fpsFrames;
      if (t - fpsLast >= 1.0) {
        std::printf("\r%.1f fps  (%zu trees, shadows %s, %dx%d)          ",
                    fpsFrames / (t - fpsLast), forest.size(), shadows ? "on" : "off", width,
                    height);
        std::fflush(stdout);
        fpsLast = t;
        fpsFrames = 0;
      }
    }
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    // Single-threaded browser build: the scene's publisher has no worker
    // thread, so drain it here at frame cadence.
    sg.publisher().flush();
#endif
    // Yield to the browser event loop every frame (Asyncify) — input events
    // fire and the canvas presents during this sleep.
    emscripten_sleep(0);
#endif
    ++frame;
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);

  // The waterline check (OCEAN-AND-WEATHER-ROADMAP.md O3's exit): every ocean vertex offshore
  // is displaced as the vertex shader displaces it, from the cascades read back at the last
  // frame's time and sampled as the GPU samples them (the texel nearest the point, wrapped: the
  // shader's oDisp), and counted when
  // it lands on dry ground: under the old shader (every cascade at full strength, no limit) and
  // under this one (faded with depth, limited to 0.8 of the way to the shore). It checks the
  // shader's arithmetic and the textures it reads, on the CPU; the GPU's own run of it is seen
  // in the frames.
  int rc = 0;
  if (shoreCheck) {
    if (!oceanReady) {
      std::printf("shore check: no GPU ocean (no float render targets)\n");
      rc = 2;
    } else {
      std::vector<std::vector<float>> disp;
      for (auto &c : cascades)
        disp.push_back(c.fft->readbackDisplacement());
      const cvc::ocean::sea_state sea = fftSea(seaState()); // the cascades' wind sea
      const double fadeCap = knob("coast.water.shore_fade", 3.0);
      const double chop = knob("coast.water.choppiness", 1.0);
      const double tNow = clock.t();
      auto at = [&](std::size_t ci, double x, double y) -> const float * {
        const double tile = cascades[ci].band.tile_m;
        double u = x / tile + 0.5 / FFT_N, v = y / tile + 0.5 / FFT_N; // oDisp's half texel
        u -= std::floor(u);
        v -= std::floor(v);
        const int i = std::min(FFT_N - 1, static_cast<int>(u * FFT_N));
        const int j = std::min(FFT_N - 1, static_cast<int>(v * FFT_N));
        return &disp[ci][(static_cast<std::size_t>(j) * FFT_N + i) * 4];
      };
      auto smooth = [](double e, double d) {
        const double t = std::min(1.0, std::max(0.0, d / e));
        return t * t * (3.0 - 2.0 * t);
      };
      long offshore = 0, oldCross = 0, newCross = 0;
      double oldMax = 0.0, newMax = 0.0;
      for (int j = 0; j < OCEAN_N; ++j)
        for (int i = 0; i < OCEAN_N; ++i) {
          const double x = -OCEAN_HALF + 2.0 * OCEAN_HALF * i / (OCEAN_N - 1);
          const double y = -OCEAN_HALF + 2.0 * OCEAN_HALF * j / (OCEAN_N - 1);
          const double s0 = shore.sample(shore.shore_m, x, y);
          if (!(s0 > 0.0))
            continue;
          ++offshore;
          const double depth = SEA_LEVEL - seabedTex.sample(x, y);
          double ox = 0.0, oz = 0.0, nx = 0.0, nz = 0.0;
          for (std::size_t ci = 0; ci < cascades.size(); ++ci) {
            const float *t = at(ci, x, y);
            const double k = smooth(cascadeFade(cascades[ci].band, sea, fadeCap), depth);
            ox += t[0];
            oz += t[2];
            nx += k * t[0];
            nz += k * t[2];
          }
          ox *= chop;
          oz *= chop;
          nx *= chop;
          nz *= chop;
          const cvc::ocean::shore_layer::sample sl = shoreLayer.evaluate(x, y, tNow);
          nx += sl.dx; // the swell's Gerstner move, before the limit as in the shader
          nz += sl.dy;
          const double lim = 0.8 * s0, len = std::hypot(nx, nz);
          if (len > lim) {
            nx *= lim / len;
            nz *= lim / len;
          }
          oldCross += shore.sample(shore.shore_m, x + ox, y + oz) <= 0.0 ? 1 : 0;
          newCross += shore.sample(shore.shore_m, x + nx, y + nz) <= 0.0 ? 1 : 0;
          if (s0 < 10.0) { // the sideways moves within 10 m of the shore
            oldMax = std::max(oldMax, std::hypot(ox, oz));
            newMax = std::max(newMax, std::hypot(nx, nz));
          }
        }
      std::printf("shore check at t = %.2f s: %ld vertices offshore; onto dry ground, the old "
                  "shader moves %ld, this one %ld; within 10 m of the shore the largest sideways "
                  "move was %.2f m, now %.2f m\n",
                  tNow, offshore, oldCross, newCross, oldMax, newMax);
      // The shore layer: where it breaks and where it reflects.
      std::size_t wet = 0, breaking = 0, reflecting = 0;
      double maxA = 0.0;
      for (std::size_t i = 0; i < shoreLayer.amplitude.size(); ++i) {
        if (!(shoreGrid.depth[i] > 0.0f))
          continue;
        ++wet;
        breaking += shoreLayer.breaking[i] > 0.0f ? 1 : 0;
        reflecting += shoreLayer.reflection[i] > 0.5f && shoreLayer.reflected[i] > 0.01f ? 1 : 0;
        maxA = std::max(maxA, double(shoreLayer.amplitude[i]));
      }
      std::printf("shore layer: deep amplitude %.2f m, largest %.2f m; of %zu wet cells %zu break "
                  "and %zu reflect (K_r > 0.5)\n",
                  shoreLayer.deep_amplitude, maxA, wet, breaking, reflecting);
      // The swash on the beach now: what the terrain's shader draws, evaluated on the CPU.
      std::size_t reach = 0, filmed = 0, damp = 0;
      double maxR = 0.0;
      for (int r = 0; r < seaGrid.rows; ++r)
        for (int c = 0; c < seaGrid.cols; ++c) {
          const std::size_t i = std::size_t(r) * seaGrid.cols + c;
          const double z = seabed[i] - SEA_LEVEL;
          if (!(z > 0.0))
            continue;
          const double R = shoreLayer.runup[i];
          maxR = std::max(maxR, R);
          if (!(z < R))
            continue;
          ++reach;
          const cvc::ocean::shore_layer::swash_sample sw =
              shoreLayer.swash(seaGrid.world_x(c), seaGrid.world_y(r), tNow, seabed[i], SEA_LEVEL);
          filmed += sw.film > 0.0 ? 1 : 0;
          damp += sw.wet > 0.5 ? 1 : 0;
        }
      std::printf("swash: runup up to %.2f m; %zu beach cells within its reach, %zu under the film "
                  "now, %zu wet (> 0.5)\n",
                  maxR, reach, filmed, damp);
      rc = newCross > 0 ? 3 : 0;
    }
  }
  hud.detach();
  cam.detach(); // stop receiving events before teardown
  return rc;
}

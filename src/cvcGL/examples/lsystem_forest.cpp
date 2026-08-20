// lsystem_forest — a pure-C++ cvcGL demo (port of scripts/examples/lsystem_forest.py
// from the volrover repo). Drives cvcGL directly: an island the eye can fly over,
// navigable with the built-in CameraController (orbit / Quake-fly / cinematic track).
//
// NO SINGLETON: owns an explicit cvc::app and injects it into the scene, so the
// whole thing runs under one app the caller controls — logging, state and the
// state publisher all flow through it (cvcGL has no process-wide context).
//
// The island: a heightfield terrain (matte + a procedural fragment bump map), an
// L-system FOREST (each tree grown from the grammar, merged route-C to one wood +
// one needle actor, wind re-posed every frame via updateVertices, procedural bark
// on the wood), a SEA volume (depth under a travelling wave), a drifting SKY of
// L-system + fBm clouds (a 3-D turtle grows two density fields, then fractal noise
// frays them into fluffy cumulus; crossfaded + scrolled so the cloud evolves as it
// travels; casts a soft ground dapple), a gradient sky with a camera-relative sun,
// and shadows — all navigable with the built-in CameraController.
//
// Run (onscreen, navigable):   lsystem_forest
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer.
// Verify (offscreen, headless): lsystem_forest --offscreen --frames 30 --png out.png
// Cinematic capture (mp4-ready): lsystem_forest --capture fly --frames 1800 --fps 30 \
//   --width 1920 --height 1080 --out frames   (also --capture orbit; then encode `frames/`)

#include <algorithm>
#include <boost/program_options.hpp>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
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
#include <vtkRenderer.h>

using cvc::gl::CameraController;

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

// ── the sea: a volume whose field is depth under a travelling wave ───────────
constexpr int SEA_N = 56, SEA_NZ = 18;
constexpr double SEA_FLOOR = SEA_LEVEL - 20.0, SEA_TOP = SEA_LEVEL + 5.0;
constexpr double WAVE_AMP = 1.15; // overall wave height (reduced; crested multi-signal below)

// A choppier sea than a lone sine: four travelling waves at different headings,
// wavelengths and INCOMMENSURATE speeds (so the combined period reads erratic, never
// obviously repeating), each CRESTED — a sharpened sine (s^2.4) that pinches the peaks
// and broadens the troughs, so the water rolls and crests instead of undulating.
double seaSurface(double x, double y, double t) {
  struct Wave {
    double hx, hy, len, omega, amp;
  };
  static const Wave W[] = {{0.86, 0.51, 58.0, 0.52, 1.00},
                           {-0.30, 0.95, 37.0, 0.93, 0.55},
                           {0.99, -0.16, 26.0, 1.37, 0.32},
                           {0.42, 0.91, 71.0, 0.40, 0.62}};
  double h = 0.0, wsum = 0.0;
  for (const Wave &w : W) {
    double k = 2.0 * M_PI / w.len;
    double s = 0.5 + 0.5 * std::sin(k * (w.hx * x + w.hy * y) - w.omega * t); // [0,1]
    h += w.amp * std::pow(s, 2.4); // crest: pinch peaks, broaden troughs
    wsum += w.amp;
  }
  // h/wsum is the weighted-average crest (mean ~0.29); centre it so the calm water
  // sits at SEA_LEVEL and crests poke up.
  return SEA_LEVEL + WAVE_AMP * (h / wsum - 0.29);
}

std::vector<float> seaField(double t) {
  std::vector<float> f(static_cast<size_t>(SEA_N) * SEA_N * SEA_NZ, 0.0f);
  for (int k = 0; k < SEA_NZ; ++k) {
    double z = SEA_FLOOR + (SEA_TOP - SEA_FLOOR) * k / (SEA_NZ - 1);
    for (int j = 0; j < SEA_N; ++j) {
      double y = -HALF + 2.0 * HALF * j / (SEA_N - 1);
      for (int i = 0; i < SEA_N; ++i) {
        double x = -HALF + 2.0 * HALF * i / (SEA_N - 1);
        double surf = seaSurface(x, y, t);
        double below = surf - z, above = z - terrainH(x, y);
        double depth =
            (below > 0.0 && above > 0.0) ? std::min(1.0, std::max(0.0, below / 6.0)) : 0.0;
        f[static_cast<size_t>(k) * SEA_N * SEA_N + j * SEA_N + i] = static_cast<float>(depth);
      }
    }
  }
  return f;
}
void seaTransfer(std::vector<double> &color, std::vector<double> &opacity, double t) {
  double k = 0.0100 + 0.0015 * std::sin(t * 0.9);
  color = {0.00, 0.42, 0.78, 0.74, 0.25, 0.14, 0.55, 0.66,
           0.60, 0.04, 0.26, 0.46, 1.00, 0.01, 0.09, 0.22};
  opacity = {0.00, 0.0, 0.12, k * 0.45, 0.55, k, 1.00, k * 2.0};
}
cvc::volume seaVolume(cvc::app &app, const std::vector<float> &field) {
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(field.data()),
                     cvc::dimension(SEA_N, SEA_N, SEA_NZ), cvc::Float,
                     cvc::bounding_box(-HALF, -HALF, SEA_FLOOR, HALF, HALF, SEA_TOP));
}

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
void reposeTree(const Tree &tree, double t, std::vector<double> &woodBuf,
                std::vector<double> &needleBuf) {
  double a = tree.sway * std::sin(1.3 * t + tree.phase);
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
  po::options_description desc("lsystem_forest — a pure-C++ cvcGL island demo\nOptions");
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
  // The scene's nodes log through THIS injected app (not a process-wide singleton),
  // so we set the console verbosity right here: show only errors + warnings, unless
  // --verbose asks for the full per-frame cvcGL debug trace. Levels: a message at
  // level L prints when L < log_verbosity (0=error, 1=warning, 2=info, 3+=debug).
  app.properties("system.log_verbosity", verbose ? "6" : "2");
  SceneGraph sg(app, "forest");

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
  sg.addGraphics("forest_wood", forestWood);
  auto woodNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("forest_wood"));
  woodNode->setUseSingleColor(false);
  woodNode->setAmbient(0.5);
  woodNode->setDiffuse(0.8);
  addBark(*woodNode);
  std::vector<double> woodBuf = flattenPoints(forestWood);
  std::shared_ptr<GeometryNode> needleNode;
  std::vector<double> needleBuf;
  if (forestNeedle.points().size()) {
    sg.addGraphics("forest_needle", forestNeedle);
    needleNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("forest_needle"));
    needleNode->setRenderMode(GeometryRenderMode::LINES);
    needleNode->setUseSingleColor(true);
    needleNode->setColor(C_NEEDLE.x, C_NEEDLE.y, C_NEEDLE.z);
    needleNode->setAmbient(0.55);
    needleNode->setDiffuse(0.75);
    needleBuf = flattenPoints(forestNeedle);
  }

  // The sea: a volume filling the space between the seabed and a travelling wave,
  // translucent in the shallows and opaque offshore, so the water sits only in the
  // hollows and the sand shows through where it is shallow.
  auto seaNode = sg.addGraphics("sea", seaVolume(app, seaField(0.0)));
  {
    std::vector<double> col, op;
    seaTransfer(col, op, 0.0);
    seaNode->setTransferFunction(col, op);
  }
  seaNode->setShading(true); // water is a lit surface; the swell throws a sun glint
  seaNode->setAmbient(0.18);
  seaNode->setDiffuse(0.72);
  seaNode->setSpecular(0.85);
  seaNode->setSpecularPower(70.0);

  // The sky: a drifting, evolving cloud slab, high above the island. Unlike the
  // sun (430 units out — pure scenery, added after framing) the cloud ceiling is
  // part of the scene the eye takes in, so it IS folded into the framing bounds:
  // the island then fills the lower frame with the clouds drifting overhead. Real
  // voxels first, then the node, then the transfer function (same order as the sea).
  SkyModel sky = buildSky();
  auto skyNode = sg.addGraphics("forest_sky", skyVolume(app, sky.field(0.0, 0.0)));
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

  // Afternoon sun (warm) + a dim cool sky fill from the opposite side.
  sg.addDirectionalLight(-52.0, 34.0, 1.0, 0.94, 0.82, 1.15);
  sg.addDirectionalLight(128.0, 52.0, 0.55, 0.66, 0.85, 0.70);

  SceneRenderer view(sg, width, height, offscreen, "main");
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
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.getGraphicsRoot()->setShowBBox(false);
  sg.processEvents();

  std::printf("lsystem_forest: %s, terrain %dx%d, shadows %s. Tab=orbit/fly, WASD+mouse=fly.\n",
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
  const std::string azPath = CameraController::viewerStatePath("forest", "main") + ".orbit.azimuth";
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
  // Interactive update cadence, decoupled by how fast each thing actually moves:
  // the wind ripples quickest, then the sea, the cloud drifts slowly, its ground
  // shadow slower still. A capture refreshes everything EVERY frame (smooth, no
  // stepping). The tree wind re-poses every other frame — imperceptibly stepped for
  // a gentle sway and it roughly halves the wind's per-frame cost; the sea's stride
  // also bounds how often its (not-free) transfer function is rebuilt for the glint.
  const int WIND_STRIDE = 2, SEA_STRIDE = 4, CLOUD_STRIDE = 8, SHADOW_STRIDE = 16;
  while (!view.windowClosed()) {
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
    // Wind: re-pose every tree into the two MERGED forest buffers, then upload each
    // ONCE (not per tree). Interactive playback re-poses every other frame — a gentle
    // sway is imperceptibly stepped at that cadence and it roughly halves the wind's
    // cost; a capture re-poses every frame so nothing stutters.
    if (capturing || frame % WIND_STRIDE == 0) {
      for (Tree &tr : forest)
        reposeTree(tr, t, woodBuf, needleBuf);
      woodNode->updateVertices(woodBuf);
      if (needleNode)
        needleNode->updateVertices(needleBuf);
    }
    // The volumes refresh IN PLACE (updateScalars), NOT via setVolume — setVolume
    // re-imports the whole field every call (realloc + full range rescan + transfer-
    // function reset + heavy logging), which is what pinned the framerate at ~8 fps.
    // The sea's transfer function is time-varying (the sun glint), so it is re-applied
    // (cheap); the cloud's is constant, so only its scalars move.
    const bool seaDue = capturing || frame % SEA_STRIDE == 0;
    const bool cloudDue = capturing || frame % CLOUD_STRIDE == 1;
    const bool shadowDue = capturing || frame % SHADOW_STRIDE == 3;
    if (seaDue) { // travelling wave
      seaNode->updateScalars(seaField(t));
      std::vector<double> col, op;
      seaTransfer(col, op, t);
      seaNode->setTransferFunction(col, op);
    }
    if (cloudDue || shadowDue) {
      double shift = t * CLOUD_DRIFT * SKY_N / (2.0 * SKY_HALF);
      double morph = t / CLOUD_MORPH_S * CLOUD_MAPS;
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
    // Browser: yield to the event loop every frame (Asyncify) — input events
    // fire and the canvas presents during this sleep. The scene's publisher has
    // no worker thread here, so drain it at the same cadence.
    sg.publisher().flush();
    emscripten_sleep(0);
#endif
    ++frame;
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);
  cam.detach(); // stop receiving events before teardown
  return 0;
}

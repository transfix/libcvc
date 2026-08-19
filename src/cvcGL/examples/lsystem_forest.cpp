// lsystem_forest — a pure-C++ cvcGL demo (port of scripts/examples/lsystem_forest.py
// from the volrover repo). Drives cvcGL directly: an island the eye can fly over,
// navigable with the built-in CameraController (orbit / Quake-fly / cinematic track).
//
// NO SINGLETON: owns an explicit cvc::app and injects it into the scene, so the
// whole thing runs under one app the caller controls (cvc::gl::context() unused).
//
// The island: a heightfield terrain (matte + a procedural fragment bump map), an
// L-system FOREST (each tree grown from the grammar, merged route-C to one wood +
// one needle actor, wind re-posed every frame via updateVertices, procedural bark
// on the wood), a SEA volume (depth under a travelling wave), the afternoon sun
// (disc + a directional light) and striped shadows — all navigable with the
// built-in CameraController. (The Python demo's cloud/sky volume is the one part
// not yet ported; it needs the cloud L-system grammar.)
//
// Run (onscreen, navigable):   lsystem_forest
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer.
// Verify (offscreen, headless): lsystem_forest --offscreen --frames 30 --png out.png

#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

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

cvc::geometry buildTerrain(cvc::app &app) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  const cvc::geometry::color_t rock = {0.46, 0.45, 0.43};
  const cvc::geometry::color_t grass = {0.27, 0.44, 0.19};
  const cvc::geometry::color_t sand = {0.68, 0.62, 0.44};
  for (int j = 0; j < TN; ++j) {
    double y = -HALF + 2.0 * HALF * j / (TN - 1);
    for (int i = 0; i < TN; ++i) {
      double x = -HALF + 2.0 * HALF * i / (TN - 1);
      double h = terrainH(x, y);
      pts.push_back({x, y, h});
      // shade by height: sand at the waterline, grass above, rock on the peaks
      cvc::geometry::color_t c = grass;
      double rockw = std::min(1.0, std::max(0.0, (h - 18.0) / 14.0));
      for (int k = 0; k < 3; ++k)
        c[k] = grass[k] * (1.0 - rockw) + rock[k] * rockw;
      double shore = std::max(0.0, 1.0 - std::fabs(h - SEA_LEVEL) / 4.5);
      for (int k = 0; k < 3; ++k)
        c[k] = c[k] * (1.0 - shore) + sand[k] * shore;
      cols.push_back(c);
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
  node.addFragmentShaderReplacement("//VTK::Normal::Dec",
                                    std::string("//VTK::Normal::Dec\nin vec3 gCoord;\n") +
                                        GROUND_GLSL);
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Impl",
      "//VTK::Normal::Impl\n"
      "  {\n"
      "    float h = groundH(gCoord);\n"
      "    vec3 sS = dFdx(vertexVC.xyz);\n"
      "    vec3 sT = dFdy(vertexVC.xyz);\n"
      "    vec3 vn = normalVCVSOutput;\n"
      "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
      "    float det = dot(sS, R1);\n"
      "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
      "    normalVCVSOutput = normalize(abs(det)*vn - 1.4*sg);\n"
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
  M.at(0, 0) = c + k * x * x; M.at(0, 1) = k * x * y - s * z; M.at(0, 2) = k * x * z + s * y;
  M.at(1, 0) = k * x * y + s * z; M.at(1, 1) = c + k * y * y; M.at(1, 2) = k * y * z - s * x;
  M.at(2, 0) = k * x * z - s * y; M.at(2, 1) = k * y * z + s * x; M.at(2, 2) = c + k * z * z;
  return M;
}
Mat4 mTrans(double x, double y, double z) {
  Mat4 M = mIdent();
  M.at(0, 3) = x; M.at(1, 3) = y; M.at(2, 3) = z;
  return M;
}
Vec3d xform(const Mat4 &M, Vec3d p) { // p @ R^T + t  ==  R@p + t
  return {M.at(0, 0) * p.x + M.at(0, 1) * p.y + M.at(0, 2) * p.z + M.at(0, 3),
          M.at(1, 0) * p.x + M.at(1, 1) * p.y + M.at(1, 2) * p.z + M.at(1, 3),
          M.at(2, 0) * p.x + M.at(2, 1) * p.y + M.at(2, 2) * p.z + M.at(2, 3)};
}

const char *TREE_RULES[5] = {"FF[RL1][RR2][RRR3]F[RL3][RR1][RRR2]RFLR0",
                             "FL[T[RF]2]R[TRFL]RTFL4", "FL[TRF3]RFLRTFL2",
                             "FL[TFL2RFL]R[T[RFLF3]]RTFL2", "FL[TRFL4]RFLRTFL4"};
constexpr double YROTATE = 10.0, TILT = 120.0, MICRO_TILT = 1.0e-4;
constexpr double T_SCALE = 0.9, T_RADSCALE = 0.6, T_LENGTH = 5.0, T_RADIUS = 0.7;
constexpr int BASE_TRI = 5, NEEDLES = 9;
constexpr double LEAF_LEN = 4.0, LEAF_RAD = 1.0;
constexpr int SWAY_LEVELS = 2;
const int MATURITY[7] = {1, 2, 2, 3, 3, 3, 4};
const Vec3d C_WOOD_LIGHT{0.6549, 0.4901, 0.2392}, C_WOOD_DARK{0.3607, 0.2510, 0.2000};
const Vec3d C_NEEDLE{0.1373, 0.5568, 0.1373};

struct Seg { Mat4 m; double len, rad; };
struct Leaf { Mat4 m; double sc; };
struct Module { int parent; int level; Mat4 hang; std::vector<Seg> segs; std::vector<Leaf> leaves; };

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
  std::vector<ModRec> mods;
  double phase = 0, sway = 0;
  std::vector<double> woodBuf, needleBuf; // flat xyz, for updateVertices
  std::shared_ptr<GeometryNode> woodNode, needleNode;
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
  node.addVertexShaderReplacement("//VTK::PositionVC::Impl",
                                  "//VTK::PositionVC::Impl\n  bNrm = normalMC; bPos = vertexMC.xyz;");
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Dec", std::string("//VTK::Normal::Dec\nin vec3 bNrm;\nin vec3 bPos;\n") +
                                BARK_GLSL);
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Impl",
      "//VTK::Normal::Impl\n"
      "  {\n"
      "    float h = barkH(normalize(bNrm), bPos.z);\n"
      "    vec3 sS = dFdx(vertexVC.xyz), sT = dFdy(vertexVC.xyz), vn = normalVCVSOutput;\n"
      "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
      "    float det = dot(sS, R1);\n"
      "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
      "    normalVCVSOutput = normalize(abs(det)*vn - 1.2*sg);\n"
      "  }\n");
}

// The tree turtle stands on +Y; the world is Z-up.
Mat4 treeUp() { return mRot(M_PI / 2.0, 1.0, 0.0, 0.0); }

// Grow + merge one tree, add its wood + needle actors, install bark. Returns the
// per-module re-pose records for the wind.
Tree buildTree(cvc::app &app, SceneGraph &sg, const std::string &name, double px, double py,
               double pz, const std::vector<Module> &mods, const CylTopo &cyl,
               const std::vector<Vec3d> &nring) {
  Tree tree;
  Mat4 tUp = treeUp();
  std::vector<Mat4> world(mods.size());
  cvc::geometry wg(app), ng(app);
  int wCur = 0, nCur = 0;
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
        Vec3d p = xform(s.m, loc[v]);        // module-frame vertex
        rec.localWood.push_back(p);
        Vec3d w = xform(world[i], p);        // bind-pose world vertex
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
      ng.points().push_back({xform(world[i], root).x, xform(world[i], root).y,
                             xform(world[i], root).z});
      for (int t = 0; t < NEEDLES; ++t) {
        Vec3d tip{nring[t].x * LEAF_RAD * lf.sc, LEAF_LEN * lf.sc, nring[t].z * LEAF_RAD * lf.sc};
        Vec3d pm = xform(lf.m, tip);
        rec.localNeedle.push_back(pm);
        Vec3d w = xform(world[i], pm);
        ng.points().push_back({w.x, w.y, w.z});
        ng.lines().push_back({base, static_cast<cvc::geometry::index_t>(base + 1 + t)});
      }
    }
    wCur = wg.points().size();
    nCur = ng.points().size();
    tree.mods.push_back(std::move(rec));
  }
  // wood actor (per-vertex colour + bark)
  sg.addGraphics(name, wg);
  tree.woodNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(name));
  tree.woodNode->setUseSingleColor(false);
  addBark(*tree.woodNode);
  tree.woodBuf.resize(wg.points().size() * 3);
  for (size_t v = 0; v < wg.points().size(); ++v) {
    tree.woodBuf[v * 3] = wg.points()[v][0];
    tree.woodBuf[v * 3 + 1] = wg.points()[v][1];
    tree.woodBuf[v * 3 + 2] = wg.points()[v][2];
  }
  // needle actor (single colour lines)
  if (ng.points().size()) {
    std::string nn = name + "_n";
    sg.addGraphics(nn, ng);
    tree.needleNode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(nn));
    tree.needleNode->setRenderMode(GeometryRenderMode::LINES);
    tree.needleNode->setUseSingleColor(true);
    tree.needleNode->setColor(C_NEEDLE.x, C_NEEDLE.y, C_NEEDLE.z);
    tree.needleBuf.resize(ng.points().size() * 3);
    for (size_t v = 0; v < ng.points().size(); ++v) {
      tree.needleBuf[v * 3] = ng.points()[v][0];
      tree.needleBuf[v * 3 + 1] = ng.points()[v][1];
      tree.needleBuf[v * 3 + 2] = ng.points()[v][2];
    }
  }
  return tree;
}

// ── the afternoon sun (a flat-lit disc + faint halo, far out) ────────────────
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0, SUN_DIST = 430.0, SUN_R = 13.0;
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
void addSun(cvc::app &app, SceneGraph &sg) {
  Vec3d d = sunDir(SUN_AZ, SUN_EL);
  Vec3d c{d.x * SUN_DIST, d.y * SUN_DIST, d.z * SUN_DIST};
  Vec3d face{-d.x, -d.y, -d.z}; // face the origin (where the camera orbits)
  sg.addGraphics("sun", discGeom(app, c, face, SUN_R));
  auto disc = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun"));
  disc->setColor(1.0, 0.97, 0.88);
  disc->setAmbient(1.0);
  disc->setDiffuse(0.0);
  disc->setSpecular(0.0); // flat-lit — a sun must not be a dark disc lit from behind
  Vec3d ch{c.x * 1.02, c.y * 1.02, c.z * 1.02};
  sg.addGraphics("sun_halo", discGeom(app, ch, face, SUN_R * 3.2));
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
constexpr double WAVE_AMP = 1.6, WAVE_LEN = 46.0, WAVE_SPEED = 7.0;

std::vector<float> seaField(double t) {
  std::vector<float> f(static_cast<size_t>(SEA_N) * SEA_N * SEA_NZ, 0.0f);
  for (int k = 0; k < SEA_NZ; ++k) {
    double z = SEA_FLOOR + (SEA_TOP - SEA_FLOOR) * k / (SEA_NZ - 1);
    for (int j = 0; j < SEA_N; ++j) {
      double y = -HALF + 2.0 * HALF * j / (SEA_N - 1);
      for (int i = 0; i < SEA_N; ++i) {
        double x = -HALF + 2.0 * HALF * i / (SEA_N - 1);
        double phase = (2.0 * M_PI / WAVE_LEN) * (x + 0.6 * y);
        double surf = SEA_LEVEL + WAVE_AMP * (std::sin(phase - WAVE_SPEED * t * 0.1) +
                                               0.45 * std::sin(1.7 * phase + WAVE_SPEED * t * 0.13));
        double below = surf - z, above = z - terrainH(x, y);
        double depth = (below > 0.0 && above > 0.0) ? std::min(1.0, std::max(0.0, below / 6.0)) : 0.0;
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

// Re-run the wind cascade and blit the posed vertices, one updateVertices each.
void reposeTree(Tree &tree, double t) {
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
      tree.woodBuf[wo * 3] = w.x;
      tree.woodBuf[wo * 3 + 1] = w.y;
      tree.woodBuf[wo * 3 + 2] = w.z;
      ++wo;
    }
    int no = m.nOff;
    for (const Vec3d &p : m.localNeedle) {
      Vec3d w = xform(world[i], p);
      tree.needleBuf[no * 3] = w.x;
      tree.needleBuf[no * 3 + 1] = w.y;
      tree.needleBuf[no * 3 + 2] = w.z;
      ++no;
    }
  }
  tree.woodNode->updateVertices(tree.woodBuf);
  if (tree.needleNode)
    tree.needleNode->updateVertices(tree.needleBuf);
}

} // namespace

int main(int argc, char **argv) {
  bool offscreen = false;
  int frames = 0;
  std::string png;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(argv[i], "--frames") && i + 1 < argc)
      frames = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--png") && i + 1 < argc)
      png = argv[++i];
  }

  // Own the app and inject it — no global/singleton context.
  cvc::app app;
  SceneGraph sg(app, "forest");

  sg.addGraphics("terrain", buildTerrain(app));
  auto terrain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"));
  terrain->setUseSingleColor(false);
  addTerrainBump(*terrain);

  // Plant an L-system forest on the dry land, each tree merged to ONE wood actor
  // (+ ONE needle actor) with procedural bark, re-posed by the wind each frame.
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
      Tree tr = buildTree(app, sg, "tree" + std::to_string(planted), x, y, h, mods, cyl, nring);
      tr.phase = u01(rng) * 2.0 * M_PI;
      tr.sway = 0.020 + 0.016 * u01(rng);
      forest.push_back(std::move(tr));
      ++planted;
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

  // Afternoon sun (warm) + a dim cool sky fill from the opposite side.
  sg.addDirectionalLight(-52.0, 34.0, 1.0, 0.94, 0.82, 1.0);
  sg.addDirectionalLight(128.0, 52.0, 0.55, 0.66, 0.85, 0.55);

  SceneRenderer view(sg, 1280, 800, offscreen, "main");
  view.setBackground(0.62, 0.76, 0.92);

  // Shadows on (a render target now exists); re-baked every 3rd frame (cheap).
  const bool shadows = sg.setShadowsEnabled(true);
  sg.setShadowUpdateInterval(3);

  // Built-in navigation, framed on the island.
  CameraController cam(view);
  cvc::bounding_box b = sg.computeGraphicsBounds();
  cam.frameBounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);

  // The sun disc is added AFTER framing — it sits ~430 units out, and folding it
  // into the bounds would push the camera reset back far enough to shrink the
  // island to a speck. It is scenery, not world.
  addSun(app, sg);

  // Hide the lab grid/axis box (do it last + pump, so the visibility sticks).
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  sg.processEvents();

  std::printf("lsystem_forest: %s, terrain %dx%d, shadows %s. Tab=orbit/fly, WASD+mouse=fly.\n",
              offscreen ? "offscreen" : "onscreen", TN, TN, shadows ? "on" : "unavailable");

  auto start = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  int n = 0;
  const int VOL_STRIDE = 3; // refresh the sea every 3rd frame (upload is not free)
  while (!view.windowClosed()) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    double dt = t - last;
    last = t;
    view.processUIEvents();
    for (Tree &tr : forest) // wind: re-pose every tree (route C updateVertices)
      reposeTree(tr, t);
    if (frame % VOL_STRIDE == 0) { // travelling wave; setVolume RESETS the TF, so re-apply
      seaNode->setVolume(seaVolume(app, seaField(t)));
      std::vector<double> col, op;
      seaTransfer(col, op, t);
      seaNode->setTransferFunction(col, op);
    }
    cam.update(dt);
    view.render();
    ++frame;
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);
  cam.detach(); // stop receiving events before teardown
  return 0;
}

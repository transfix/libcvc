// lsystem_forest_ari — the cvcGL lsystem_forest demo, ported to Ariadne. The island, sky-gradient,
// lighting rig and shadows are authored declaratively in lsystem_forest.ari on the reusable scene
// DSL (a procedural `heightfield` terrain with height-band colours); the procedural L-system TREES
// are a custom scene-node type this host registers (`type: forest_trees`), the one piece that is
// not expressible declaratively. The trees are planted on the terrain by sampling the realized
// heightfield mesh, so the .ari owns the island and the host owns only the generators.
//
// Everything below the forest_trees registration is the stock cvcGL Ariadne runner (cf.
// ariadne_hello / ariadne_stream): load the doc, realize the scene, frame the camera, render — a
// window, or a one-frame offscreen PNG for verification.
//
//   lsystem_forest_ari lsystem_forest.ari --component-path src/cvc/ariadne/components
//   lsystem_forest_ari lsystem_forest.ari --offscreen --png forest.png --component-path …

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <vtkRenderer.h> // the scene's background gradient reaches the renderer directly (§16.1)

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;

namespace {

// ─────────────────────────── tiny 3-vector + an L-system tree ───────────────────────────

struct V3 {
  double x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double len(V3 a) { return std::sqrt(dot(a, a)); }
V3 norm(V3 a) {
  double l = len(a);
  return l > 1e-12 ? a * (1.0 / l) : V3{0, 0, 1};
}

// A deterministic little RNG so a given seed always grows the same forest.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
  uint64_t next() {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return s;
  }
  double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double a, double b) { return a + (b - a) * uniform(); }
};

const V3 C_WOOD_LIGHT{0.655, 0.490, 0.239};
const V3 C_WOOD_DARK{0.361, 0.251, 0.200};
const V3 C_NEEDLE{0.137, 0.557, 0.137};

// ───────── the ORIGINAL pine-tree L-system (ported from lsystem_forest.cpp) ─────────
// The demo's own conifer grammar: a turtle grows a module hierarchy (cylinder trunk/branch
// segments + needle "stars"), merged into a wood (triangles) + needle (lines) mesh. Kept
// alongside the branchy species above so both tree types appear in the forest.
struct Mat4 {
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
V3 xform(const Mat4 &M, V3 p) {
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
const int MATURITY[7] = {1, 2, 2, 3, 3, 3, 4};

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
    } else if (std::isdigit(static_cast<unsigned char>(ch)) && depth > 1) {
      int child = expandTree(TREE_RULES[ch - '0'], depth - 1, scale * T_SCALE,
                             radscale * T_RADSCALE, me, level + 1, out, tMicro, tTilt, tRoll);
      out[child].hang = cur;
    }
  }
  return me;
}

// The unit-cylinder ring topology + per-vertex wood colours (vertex 0 = bottom cap centre,
// 1..BASE_TRI = bottom ring, BASE_TRI+1 = top cap centre, rest = top ring).
struct CylTopo {
  std::vector<V3> ringUnit;
  std::vector<unsigned> tris;
  std::vector<V3> colors;
};
CylTopo cyl_topo() {
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
      c.tris.push_back(static_cast<unsigned>(idx[k]));
  }
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  return c;
}

// Grow ONE pine at base (px,py,pz), trunk scale `size`, into the shared wood (tris, per-vertex
// colour) + needle (lines) meshes — the original demo's conifer, placed on the island.
void grow_pine(cvc::geometry &wood, cvc::geometry &needle, double px, double py, double pz,
               double size, int maturity, const CylTopo &cyl, const std::vector<V3> &nring,
               const Mat4 &tMicro, const Mat4 &tTilt, const Mat4 &tRoll) {
  std::vector<Module> mods;
  expandTree(TREE_RULES[0], maturity, size, size, -1, 1, mods, tMicro, tTilt, tRoll);
  const Mat4 tUp = mRot(M_PI / 2.0, 1.0, 0.0, 0.0); // turtle +Y -> world Z-up
  std::vector<Mat4> world(mods.size());
  for (size_t i = 0; i < mods.size(); ++i) {
    const Module &mod = mods[i];
    Mat4 hang = (mod.parent < 0) ? mMul(mTrans(px, py, pz), tUp) : mod.hang;
    world[i] = (mod.parent < 0) ? hang : mMul(world[mod.parent], hang);
    for (const Seg &s : mod.segs) {
      V3 loc[2 * BASE_TRI + 2];
      loc[0] = {0, 0, 0};
      loc[BASE_TRI + 1] = {0, s.len, 0};
      for (int r = 0; r < BASE_TRI; ++r) {
        loc[1 + r] = {cyl.ringUnit[r].x * s.rad, 0.0, cyl.ringUnit[r].z * s.rad};
        loc[BASE_TRI + 2 + r] = {cyl.ringUnit[r].x * s.rad, s.len, cyl.ringUnit[r].z * s.rad};
      }
      const unsigned base = static_cast<unsigned>(wood.points().size());
      for (int v = 0; v < 2 * BASE_TRI + 2; ++v) {
        V3 w = xform(world[i], xform(s.m, loc[v]));
        wood.points().push_back({w.x, w.y, w.z});
        wood.colors().push_back({cyl.colors[v].x, cyl.colors[v].y, cyl.colors[v].z});
      }
      for (size_t k = 0; k < cyl.tris.size(); k += 3)
        wood.tris().push_back({base + cyl.tris[k], base + cyl.tris[k + 1], base + cyl.tris[k + 2]});
    }
    for (const Leaf &lf : mod.leaves) {
      const unsigned base = static_cast<unsigned>(needle.points().size());
      V3 wr = xform(world[i], xform(lf.m, {0, 0, 0}));
      needle.points().push_back({wr.x, wr.y, wr.z});
      for (int t = 0; t < NEEDLES; ++t) {
        V3 tip{nring[t].x * LEAF_RAD * lf.sc, LEAF_LEN * lf.sc, nring[t].z * LEAF_RAD * lf.sc};
        V3 w = xform(world[i], xform(lf.m, tip));
        needle.points().push_back({w.x, w.y, w.z});
        needle.lines().push_back(
            {static_cast<uint64_t>(base), static_cast<uint64_t>(base + 1 + t)});
      }
    }
  }
}

// Append a tapered cylinder (p0->p1, radii r0->r1) as a RING-sided tube with per-vertex colour.
void add_cylinder(cvc::geometry &g, V3 p0, V3 p1, double r0, double r1, int ring, V3 col) {
  V3 axis = norm(p1 - p0);
  // An orthonormal basis (u,v) perpendicular to the axis.
  V3 ref = std::fabs(axis.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
  V3 u = norm(cross(axis, ref));
  V3 v = norm(cross(axis, u));
  const size_t base = g.points().size();
  for (int k = 0; k < ring; ++k) {
    const double a = 2.0 * M_PI * k / ring;
    const V3 dir = u * std::cos(a) + v * std::sin(a);
    const V3 b0 = p0 + dir * r0, b1 = p1 + dir * r1;
    g.points().push_back({b0.x, b0.y, b0.z});
    g.points().push_back({b1.x, b1.y, b1.z});
    // No normals here: both tree species leave the merged wood mesh normal-less so
    // GeometryNode::setGeometry runs ONE vtkPolyDataNormals pass over the whole forest
    // (mixing hand-set and absent normals would be inconsistent).
    g.colors().push_back({col.x, col.y, col.z});
    g.colors().push_back({col.x, col.y, col.z});
  }
  for (int k = 0; k < ring; ++k) {
    const unsigned a0 = static_cast<unsigned>(base + 2 * k);
    const unsigned a1 = a0 + 1;
    const unsigned b0 = static_cast<unsigned>(base + 2 * ((k + 1) % ring));
    const unsigned b1 = b0 + 1;
    g.tris().push_back({a0, b0, b1});
    g.tris().push_back({a0, b1, a1});
  }
}

// Grow one recursive tree into `g` at `basePt`: a tapered trunk that splits into `branches` child
// limbs at each of `levels`, tips coloured as needles. Compact, not the full grammar —
// recognizable.
void grow_tree(cvc::geometry &g, Rng &rng, V3 basePt, V3 dir, double length, double radius,
               int level, int levels, int branches) {
  const V3 tip = basePt + dir * length;
  const double tipR = radius * 0.62;
  const bool leaf = (level >= levels);
  // Wood darkens toward the trunk, lightens toward the twigs; the final twigs read as needles.
  V3 col = leaf
               ? C_NEEDLE
               : C_WOOD_DARK + (C_WOOD_LIGHT - C_WOOD_DARK) * (double(level) / std::max(1, levels));
  add_cylinder(g, basePt, tip, radius, tipR, level == 0 ? 7 : 5, col);
  if (leaf)
    return;
  // Spawn child limbs around the tip, tilted out from the parent direction.
  V3 ref = std::fabs(dir.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
  V3 u = norm(cross(dir, ref));
  V3 v = norm(cross(dir, u));
  const int n = branches + (rng.uniform() < 0.4 ? 1 : 0);
  for (int b = 0; b < n; ++b) {
    const double az = 2.0 * M_PI * (b + rng.range(-0.2, 0.2)) / n;
    const double tilt = rng.range(0.45, 0.80); // radians out from the parent axis
    const V3 out = u * std::cos(az) + v * std::sin(az);
    const V3 cdir = norm(dir * std::cos(tilt) + out * std::sin(tilt));
    grow_tree(g, rng, tip, cdir, length * rng.range(0.62, 0.78), radius * 0.58, level + 1, levels,
              branches);
  }
}

// A height sampler over an already-realized heightfield GeometryNode: the mesh is a regular
// row-major res×res grid (make_heightfield), so nearest-grid-cell lookup gives the terrain height
// at (x,y).
struct TerrainHeights {
  const cvc::geometry *geom = nullptr;
  int res = 0;
  double minx = 0, miny = 0, step = 1;
  bool ok() const { return geom && res > 1; }
  void build(const cvc::geometry *g) {
    geom = g;
    if (!g || g->const_points().empty())
      return;
    const size_t n = g->const_points().size();
    res = static_cast<int>(std::lround(std::sqrt(double(n))));
    if (res < 2 || size_t(res) * res != n) {
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
  double at(double x, double y) const {
    if (!ok())
      return 0.0;
    int i = int(std::lround((x - minx) / step));
    int j = int(std::lround((y - miny) / step));
    i = std::max(0, std::min(res - 1, i));
    j = std::max(0, std::min(res - 1, j));
    return geom->const_points()[size_t(j) * res + i][2];
  }
};

// The `type: forest_trees` realizer: scatter `count` trees on dry land of the named ground node,
// of two SPECIES — the original demo's conifer `pine` (wood cylinders + needle-line stars) and the
// compact `branchy` broadleaf — merged into one wood mesh (+ a needle-line mesh for the pines).
// props: count, seed, ground, sea_level, span, species(mix|pine|branchy), plus the branchy knobs
// (length, radius, levels, branches, scale) and the pine knob (pine_scale).
std::shared_ptr<GraphicsNode> realize_forest_trees(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent,
                                                   cvc::gl::ariadne::RealizedScene &,
                                                   std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: forest_trees '" + n.id + "': " + m);
  };
  const int count = static_cast<int>(n.props.num("count", 60.0));
  const uint64_t seed = static_cast<uint64_t>(n.props.num("seed", 1337.0));
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  const double seaLevel = n.props.num("sea_level", 0.5);
  const double span = n.props.num("span", 100.0); // half-extent to scatter across
  const std::string species = n.props.str("species").empty() ? "mix" : n.props.str("species");
  const double scale = n.props.num("scale", 1.0);
  const double baseLen = n.props.num("length", 6.0) * scale;
  const double baseRad = n.props.num("radius", 0.7) * scale;
  const int levels = static_cast<int>(n.props.num("levels", 4.0));
  const int branches = static_cast<int>(n.props.num("branches", 3.0));
  const double pineScale = n.props.num("pine_scale", 1.35);

  TerrainHeights heights;
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights.build(gnode->getGeometry());
  if (!heights.ok())
    warn("ground node '" + ground + "' is not a realized heightfield — planting on a flat plane");

  // The pine turtle's rotation matrices + ring topology, built once (verbatim from the original).
  const CylTopo cyl = cyl_topo();
  std::vector<V3> nring;
  for (int t = 0; t < NEEDLES; ++t) {
    double a = t * 2.0 * M_PI / NEEDLES;
    nring.push_back({std::cos(a), 0.0, std::sin(a)});
  }
  const Mat4 tMicro = mRot(TILT * MICRO_TILT, 0, 0, 1), tTilt = mRot(TILT, 0, 0, 1),
             tRoll = mRot(YROTATE, 0, 1, 0);

  cvc::geometry wood, needle;
  Rng rng(seed);
  int planted = 0, pines = 0;
  for (int attempt = 0; attempt < count * 6 && planted < count; ++attempt) {
    const double x = rng.range(-span, span);
    const double y = rng.range(-span, span);
    if (std::sqrt(x * x + y * y) > span)
      continue;
    const double z = heights.at(x, y);
    if (z < seaLevel + 0.5)
      continue; // only dry land above the waterline
    const bool pine = (species == "pine") || (species == "mix" && (planted % 2 == 0));
    if (pine) {
      const double size = (0.55 + 0.4 * rng.uniform()) * pineScale;
      const int maturity = MATURITY[rng.next() % 7];
      grow_pine(wood, needle, x, y, z, size, maturity, cyl, nring, tMicro, tTilt, tRoll);
      ++pines;
    } else {
      grow_tree(wood, rng, V3{x, y, z}, V3{0, 0, 1}, baseLen * rng.range(0.8, 1.25), baseRad, 0,
                levels, branches);
    }
    ++planted;
  }
  if (wood.points().empty()) {
    warn("no trees planted (no dry land above sea_level in the scatter span)");
    return nullptr;
  }

  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, wood) : sg.addGraphics(n.id, wood);
  if (auto geo = std::dynamic_pointer_cast<GeometryNode>(gn))
    geo->setUseSingleColor(false); // per-vertex wood colour
  // The pine needles are LINES — their own single-coloured node alongside the wood.
  if (!needle.points().empty()) {
    if (auto nn =
            std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id + "_needles", needle))) {
      nn->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
      nn->setUseSingleColor(true);
      nn->setColor(C_NEEDLE.x, C_NEEDLE.y, C_NEEDLE.z);
    }
  }
  std::printf("[lsystem_forest_ari] planted %d trees (%d pine, %zu wood tris, %zu needle lines)\n",
              planted, pines, wood.tris().size(), needle.lines().size());
  return gn;
}

// ───────────────── the sea: a travelling-wave volume (ported from the original) ─────────────────
// A choppier sea than one sine: four crested travelling waves at incommensurate speeds/headings.
double sea_surface(double x, double y, double t, double seaLevel, double waveAmp) {
  struct Wave {
    double hx, hy, len, omega, amp;
  };
  static const Wave W[] = {{0.86, 0.51, 58.0, 0.52, 1.00},
                           {-0.30, 0.95, 37.0, 0.93, 0.55},
                           {0.99, -0.16, 26.0, 1.37, 0.32},
                           {0.42, 0.91, 71.0, 0.40, 0.62}};
  double h = 0.0, peak = 0.0;
  for (const Wave &w : W) {
    const double k = 2.0 * M_PI / w.len;
    const double s = 0.5 + 0.5 * std::sin(k * (w.hx * x + w.hy * y) - w.omega * t);
    h += w.amp * std::pow(s, 2.4); // crest: pinch peaks, broaden troughs
    peak = w.amp > peak ? w.amp : peak;
  }
  double crest = h / (peak > 1e-9 ? peak : 1.0);
  if (crest > 1.0)
    crest = 1.0;
  return seaLevel + waveAmp * (crest - 0.72);
}

// The `type: wave_sea` realizer: a VolumeNode whose scalar field is the water depth under the
// travelling-wave surface and above the terrain, re-filled each frame so the sea rolls and crests.
// props: ground (terrain node), half (world half-extent), sea_level, wave_amp.
std::shared_ptr<GraphicsNode> realize_wave_sea(SceneGraph &sg, const ari::SceneNode &n,
                                               GraphicsNode *parent,
                                               cvc::gl::ariadne::RealizedScene &out,
                                               std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: wave_sea '" + n.id + "': " + m);
  };
  constexpr int SEA_N = 56, SEA_NZ = 18;
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  const double half = n.props.num("half", 120.0);
  const double seaLevel = n.props.num("sea_level", 0.0);
  const double waveAmp = n.props.num("wave_amp", 2.40);
  const double seaFloor = seaLevel - 20.0, seaTop = seaLevel + 5.0;

  // Terrain height at each sea-grid column (constant), sampled from the realized heightfield.
  TerrainHeights heights;
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights.build(gnode->getGeometry());
  auto terr = std::make_shared<std::vector<float>>(SEA_N * SEA_N);
  for (int j = 0; j < SEA_N; ++j)
    for (int i = 0; i < SEA_N; ++i) {
      const double x = -half + 2.0 * half * i / (SEA_N - 1);
      const double y = -half + 2.0 * half * j / (SEA_N - 1);
      (*terr)[j * SEA_N + i] = static_cast<float>(heights.ok() ? heights.at(x, y) : seaFloor);
    }

  auto fill = [=](std::vector<float> &f, double t) {
    f.assign(static_cast<size_t>(SEA_N) * SEA_N * SEA_NZ, 0.0f);
    for (int k = 0; k < SEA_NZ; ++k) {
      const double z = seaFloor + (seaTop - seaFloor) * k / (SEA_NZ - 1);
      for (int j = 0; j < SEA_N; ++j)
        for (int i = 0; i < SEA_N; ++i) {
          const double x = -half + 2.0 * half * i / (SEA_N - 1);
          const double y = -half + 2.0 * half * j / (SEA_N - 1);
          const double surf = sea_surface(x, y, t, seaLevel, waveAmp);
          const double below = surf - z, above = z - (*terr)[j * SEA_N + i];
          f[static_cast<size_t>(k) * SEA_N * SEA_N + j * SEA_N + i] =
              (below > 0.0 && above > 0.0)
                  ? static_cast<float>(std::min(1.0, std::max(0.0, below / 6.0)))
                  : 0.0f;
        }
    }
  };
  // Time-varying blue-water transfer function (opacity breathes with the crests).
  auto tf = [](std::vector<double> &color, std::vector<double> &opacity, double t) {
    const double k = 0.0100 + 0.0015 * std::sin(t * 0.9);
    color = {0.00, 0.42, 0.78, 0.74, 0.25, 0.14, 0.55, 0.66,
             0.60, 0.04, 0.26, 0.46, 1.00, 0.01, 0.09, 0.22};
    opacity = {0.00, 0.0, 0.12, k * 0.45, 0.55, k, 1.00, k * 2.0};
  };

  auto field = std::make_shared<std::vector<float>>();
  fill(*field, 0.0);
  cvc::volume vol(sg.appContext(), reinterpret_cast<const unsigned char *>(field->data()),
                  cvc::dimension(SEA_N, SEA_N, SEA_NZ), cvc::Float,
                  cvc::bounding_box(-half, -half, seaFloor, half, half, seaTop));
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<cvc::gl::VolumeNode>(n.id, vol) : sg.addGraphics(n.id, vol);
  auto vnode = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(gn);
  if (!vnode) {
    warn("could not create a VolumeNode for the sea");
    return gn;
  }
  {
    std::vector<double> col, op;
    tf(col, op, 0.0);
    vnode->setTransferFunction(col, op);
  }
  vnode->setShading(true);
  vnode->setAmbient(0.35);
  vnode->setDiffuse(0.75);

  // Per-frame: advance time, re-fill the depth field (the wave rolls) and breathe the TF.
  auto frame = std::make_shared<long>(0);
  std::weak_ptr<cvc::gl::VolumeNode> wn = vnode;
  out.custom_ticks.push_back([wn, field, fill, tf, frame](vtkRenderer *) {
    auto v = wn.lock();
    if (!v)
      return;
    const double t = static_cast<double>((*frame)++) / 60.0;
    fill(*field, t);
    v->updateScalars(*field);
    std::vector<double> col, op;
    tf(col, op, t);
    v->setTransferFunction(col, op);
  });
  std::printf("[lsystem_forest_ari] sea: %dx%dx%d volume\n", SEA_N, SEA_N, SEA_NZ);
  return gn;
}

// ───────── the sky: a drifting L-system + fBm cloud volume (ported from the original) ─────────
constexpr int SKY_N = 60, SKY_NZ = 28;
constexpr double SKY_BASE = 74.0, SKY_TOP = 122.0, SKY_HALF = 150.0;
constexpr double CLOUD_DRIFT = 3.0, CLOUD_MORPH_S = 60.0;
constexpr int CLOUD_MAPS = 2, CLOUD_DEPTH = 6;
constexpr double CLOUD_TURN = 32.0, CLOUD_STEP0 = 8.1, CLOUD_STEP_DECAY = 0.9;
constexpr double CLOUD_PUFF0 = 8.8, CLOUD_PUFF_DECAY = 0.88;
constexpr double CLOUD_FLOOR = 0.10, CLOUD_EMPTY = 0.22;
constexpr double SUN_AZ = -52.0, SUN_EL = 34.0;
const char *CLOUD_AXIOM = "[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]";
const char *cloudRule(char c) {
  switch (c) {
  case 'A':
    return "FF[+<B]^F[-<C]<F[+<C]vFA";
  case 'B':
    return "F[+<F]F<[-<F]vB";
  case 'C':
    return "^<F[+<F][-<F]^<FC";
  default:
    return nullptr;
  }
}
inline size_t skyIdx(int z, int y, int x) {
  return (static_cast<size_t>(z) * SKY_N + y) * SKY_N + x;
}
double percentileSorted(const std::vector<float> &a, double q) {
  if (a.empty())
    return 0.0;
  double rank = (q / 100.0) * (a.size() - 1);
  size_t lo = static_cast<size_t>(std::floor(rank));
  if (lo + 1 >= a.size())
    return a.back();
  return a[lo] + (rank - lo) * (a[lo + 1] - a[lo]);
}
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
  return f / tot;
}
V3 sunDir(double azDeg, double elDeg) {
  double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}
std::vector<float> walkClouds(std::mt19937 &rng) {
  std::uniform_real_distribution<double> U(0.0, 1.0);
  auto uni = [&](double a, double b) { return a + (b - a) * U(rng); };
  const int N = SKY_N, NZ = SKY_NZ;
  std::vector<float> field(static_cast<size_t>(NZ) * N * N, 0.0f);
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
        x += N;
      y = std::min(std::max(y + step * std::sin(head * M_PI / 180.0), 0.0), double(N - 1));
      z = std::min(std::max(z + climb, 1.0), double(NZ - 2));
      const double p2 = 2.0 * puff * puff;
      for (int gz = 0; gz < NZ; ++gz) {
        double dz = (gz - z) * zscale, dz2 = dz * dz;
        for (int gy = 0; gy < N; ++gy) {
          double dy = gy - y, dyz2 = dy * dy + dz2;
          for (int gx = 0; gx < N; ++gx) {
            double dx = std::fabs(gx - x);
            dx = std::min(dx, double(N) - dx);
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
      climb += 0.55;
    } else if (c == 'v') {
      climb -= 0.45;
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
      todo.insert(todo.begin(), r, r + std::strlen(r));
    }
  }
  std::vector<float> sorted(field);
  std::sort(sorted.begin(), sorted.end());
  double m = percentileSorted(sorted, 99.9);
  if (m > 0)
    for (float &v : field)
      v = std::min(1.0f, std::max(0.0f, v / static_cast<float>(m)));
  std::vector<double> wf(N), zf(NZ);
  for (int i = 0; i < N; ++i)
    wf[i] = std::sqrt(0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1)));
  for (int k = 0; k < NZ; ++k)
    zf[k] = std::sin(0.12 + (M_PI - 0.24) * k / (NZ - 1));
  for (int gz = 0; gz < NZ; ++gz)
    for (int gy = 0; gy < N; ++gy)
      for (int gx = 0; gx < N; ++gx) {
        float &v = field[skyIdx(gz, gy, gx)];
        v *= static_cast<float>(std::min(wf[gy], wf[gx]) * zf[gz]);
        if (v > 0.0f) {
          double d = fbm3(gx * 0.30, gy * 0.30, gz * 0.62, 5);
          v = static_cast<float>(std::min(1.0, std::max(0.0, v * (0.32 + 1.5 * d))));
        }
      }
  { // cheap baked top-light: thin each voxel by the cloud density toward the sun
    V3 sd = sunDir(SUN_AZ, SUN_EL);
    double ux = sd.x * N / (2.0 * SKY_HALF), uy = sd.y * N / (2.0 * SKY_HALF),
           uz = sd.z * NZ / (SKY_TOP - SKY_BASE);
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
    const double LK = 0.95, LFLOOR = 0.72, LSTEP = 1.6;
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
    double f = shift - ks;
    int k0 = static_cast<int>(((ks % N) + N) % N), k1 = (k0 + 1) % N;
    const std::vector<float> &A = maps[i], &B = maps[j];
    std::vector<float> out(static_cast<size_t>(NZ) * N * N);
    for (int z = 0; z < NZ; ++z)
      for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
          int sx0 = ((x - k0) % N + N) % N, sx1 = ((x - k1) % N + N) % N;
          auto mix = [&](int xx) {
            return (1.0 - u) * A[skyIdx(z, y, xx)] + u * B[skyIdx(z, y, xx)];
          };
          double vol = (1.0 - f) * mix(sx0) + f * mix(sx1);
          double lump = std::min(1.0, std::max(0.0, (vol - CLOUD_FLOOR) / (1.0 - CLOUD_FLOOR)));
          out[skyIdx(z, y, x)] = static_cast<float>(lump * lump);
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
  std::mt19937 rng(20u);
  for (int m = 0; m < CLOUD_MAPS; ++m)
    sky.maps.push_back(walkClouds(rng));
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

// The `type: cloud_sky` realizer: a drifting cloud VolumeNode over the island (two L-system + fBm
// maps crossfaded and scrolled each frame). No tunable props — the sky is self-contained.
std::shared_ptr<GraphicsNode> realize_cloud_sky(SceneGraph &sg, const ari::SceneNode &n,
                                                GraphicsNode *parent,
                                                cvc::gl::ariadne::RealizedScene &out,
                                                std::vector<std::string> *warnings) {
  auto sky = std::make_shared<SkyModel>(buildSky());
  cvc::volume vol(sg.appContext(),
                  reinterpret_cast<const unsigned char *>(sky->field(0.0, 0.0).data()),
                  cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
                  cvc::bounding_box(-SKY_HALF, -SKY_HALF, SKY_BASE, SKY_HALF, SKY_HALF, SKY_TOP));
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<cvc::gl::VolumeNode>(n.id, vol) : sg.addGraphics(n.id, vol);
  auto vnode = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(gn);
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
  auto frame = std::make_shared<long>(0);
  std::weak_ptr<cvc::gl::VolumeNode> wn = vnode;
  out.custom_ticks.push_back([wn, sky, frame](vtkRenderer *) {
    auto v = wn.lock();
    if (!v)
      return;
    const double t = static_cast<double>((*frame)++) / 60.0;
    const double shift = t * CLOUD_DRIFT * SKY_N / (2.0 * SKY_HALF);
    const double morph = t / CLOUD_MORPH_S * CLOUD_MAPS;
    v->updateScalars(sky->field(shift, morph));
  });
  std::printf("[lsystem_forest_ari] sky: %dx%dx%d cloud volume\n", SKY_N, SKY_N, SKY_NZ);
  return gn;
}

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--png")) {
      png = next("");
      offscreen = true;
    } else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 2;

  cvc::app app;
  SceneGraph sg(app, "forest");
  SceneRenderer view(sg, width, height, offscreen, "main");
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  bool quit = false;
  rt.on("quit", [&] { quit = true; });

  // The forest's non-declarative pieces: the procedural L-system trees and the travelling-wave sea.
  cvc::gl::ariadne::register_scene_node_type("forest_trees", realize_forest_trees);
  cvc::gl::ariadne::register_scene_node_type("wave_sea", realize_wave_sea);
  cvc::gl::ariadne::register_scene_node_type("cloud_sky", realize_cloud_sky);

  if (docPath.empty()) {
    std::printf("[lsystem_forest_ari] FATAL: give a .ari document (e.g. lsystem_forest.ari)\n");
    return 1;
  }
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::printf("[lsystem_forest_ari] load failed: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("[lsystem_forest_ari]   %s\n", w.c_str());

  std::vector<std::string> init_errs;
  ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs);
  rt.set_tick_program(lr.on_tick_script);
  rt.set_root(std::move(lr.root));

  cvc::gl::ariadne::RealizedScene realized;
  if (lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &sw);
    for (const std::string &w : sw)
      std::printf("[lsystem_forest_ari]   %s\n", w.c_str());
    std::printf("[lsystem_forest_ari] scene: %zu node(s)\n", realized.created.size());
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
    // Apply the scene's background (a VIEW property): a solid colour, or a top→bottom gradient
    // reached through the renderer (SceneRenderer exposes only a flat setBackground).
    if (lr.scene.has_background) {
      const auto &t = lr.scene.background_top;
      const auto &b = lr.scene.background_bottom;
      if (lr.scene.background_gradient) {
        if (vtkRenderer *r = view.renderer()) {
          r->SetGradientBackground(true);
          r->SetBackground(b[0], b[1], b[2]);  // bottom
          r->SetBackground2(t[0], t[1], t[2]); // top
        }
      } else {
        view.setBackground(t[0], t[1], t[2]);
      }
    }
  }

  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain();
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 60.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("[lsystem_forest_ari] captured %ld frame(s)%s%s\n", frames,
                png.empty() ? "" : " -> ", png.c_str());
  } else {
    std::puts("[lsystem_forest_ari] running — close the window or Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render();
    }
  }
  return 0;
}

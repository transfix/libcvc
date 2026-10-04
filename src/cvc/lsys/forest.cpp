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

// forest.cpp — tree generators (see forest.h). Ported in shape from the lsystem_forest demo so
// the pine + branchy look exactly as they do there, with the one principled change the library
// mandates: every random draw goes through the hashed RNG (rng.h) keyed on a stable element id,
// not a sequential generator, so the scatter is reproducible and insertion-stable.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/lsys/forest.h>
#include <cvc/lsys/rng.h>

namespace cvc {
namespace lsys {
namespace {

// ── tiny vec3 algebra (operates on the public cvc::lsys::vec3) ────────────────────────────────
vec3 operator+(vec3 a, vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
vec3 operator-(vec3 a, vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
vec3 operator*(vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
vec3 cross(vec3 a, vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double length3(vec3 a) { return std::sqrt(dot(a, a)); }
vec3 normalize3(vec3 a) {
  const double l = length3(a);
  return l > 1e-12 ? a * (1.0 / l) : vec3{0, 0, 1};
}

// ── a 4x4 (row-major), only the ops the turtle needs ──────────────────────────────────────────
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
  const double c = std::cos(ang), s = std::sin(ang), k = 1.0 - c;
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
vec3 xform(const Mat4 &M, vec3 p) {
  return {M.at(0, 0) * p.x + M.at(0, 1) * p.y + M.at(0, 2) * p.z + M.at(0, 3),
          M.at(1, 0) * p.x + M.at(1, 1) * p.y + M.at(1, 2) * p.z + M.at(1, 3),
          M.at(2, 0) * p.x + M.at(2, 1) * p.y + M.at(2, 2) * p.z + M.at(2, 3)};
}
std::array<double, 16> to_array(const Mat4 &M) {
  std::array<double, 16> a{};
  for (int i = 0; i < 16; ++i)
    a[i] = M.m[i];
  return a;
}
Mat4 from_array(const std::array<double, 16> &a) {
  Mat4 M{};
  for (int i = 0; i < 16; ++i)
    M.m[i] = a[i];
  return M;
}

// ── wood/foliage palette (verbatim from the demo) ─────────────────────────────────────────────
const vec3 C_WOOD_LIGHT{0.655, 0.490, 0.239};
const vec3 C_WOOD_DARK{0.361, 0.251, 0.200};
const vec3 C_NEEDLE{0.137, 0.557, 0.137};

// ── the pine conifer grammar (verbatim) ───────────────────────────────────────────────────────
const char *TREE_RULES[5] = {"FF[RL1][RR2][RRR3]F[RL3][RR1][RRR2]RFLR0", "FL[T[RF]2]R[TRFL]RTFL4",
                             "FL[TRF3]RFLRTFL2", "FL[TFL2RFL]R[T[RFLF3]]RTFL2",
                             "FL[TRFL4]RFLRTFL4"};
constexpr double YROTATE = 10.0, TILT = 120.0, MICRO_TILT = 1.0e-4;
constexpr double T_SCALE = 0.9, T_RADSCALE = 0.6, T_LENGTH = 5.0, T_RADIUS = 0.7;
constexpr int BASE_TRI = 5, NEEDLES = 9;
constexpr double LEAF_LEN = 4.0, LEAF_RAD = 1.0;
const int MATURITY[7] = {1, 2, 2, 3, 3, 3, 4};
constexpr int SWAY_LEVELS = 2;

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

// Rewrite one rule string into modules (recursing into a child rule on a digit). No RNG — the
// pine grammar is deterministic; its only variation is the maturity (recursion depth) chosen per
// tree by the caller.
int expandTree(const std::string &rule, int depth, double scale, double radscale, int parent,
               int level, std::vector<Module> &out, const Mat4 &tMicro, const Mat4 &tTilt,
               const Mat4 &tRoll) {
  const int me = static_cast<int>(out.size());
  out.push_back(Module{parent, level, mIdent(), {}, {}});
  Mat4 cur = mIdent();
  std::vector<Mat4> stack;
  const double segLen = T_LENGTH * scale, segRad = T_RADIUS * radscale;
  const Mat4 step = mTrans(0.0, segLen, 0.0);
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
      const int child = expandTree(TREE_RULES[ch - '0'], depth - 1, scale * T_SCALE,
                                   radscale * T_RADSCALE, me, level + 1, out, tMicro, tTilt, tRoll);
      out[child].hang = cur;
    }
  }
  return me;
}

// The unit cylinder ring topology + per-vertex wood colours (vertex 0 = bottom cap centre,
// 1..BASE_TRI = bottom ring, BASE_TRI+1 = top cap centre, rest = top ring).
struct CylTopo {
  std::vector<vec3> ringUnit;
  std::vector<std::uint64_t> tris;
  std::vector<vec3> colors;
};
CylTopo cyl_topo() {
  CylTopo c;
  for (int i = 0; i < BASE_TRI; ++i) {
    const double a = i * 2.0 * M_PI / BASE_TRI;
    c.ringUnit.push_back({std::cos(a), 0.0, std::sin(a)});
  }
  for (int i = 0; i < BASE_TRI; ++i) {
    const int b0 = 1 + i, b1 = 1 + (i + 1) % BASE_TRI;
    const int t0 = BASE_TRI + 2 + i, t1 = BASE_TRI + 2 + (i + 1) % BASE_TRI;
    const int idx[12] = {0, b0, b1, BASE_TRI + 1, t1, t0, b0, t1, b1, b0, t0, t1};
    for (int k = 0; k < 12; ++k)
      c.tris.push_back(static_cast<std::uint64_t>(idx[k]));
  }
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  c.colors.push_back(C_WOOD_LIGHT);
  for (int i = 0; i < BASE_TRI; ++i)
    c.colors.push_back(C_WOOD_DARK);
  return c;
}

std::vector<vec3> needle_ring() {
  std::vector<vec3> nring;
  for (int t = 0; t < NEEDLES; ++t) {
    const double a = t * 2.0 * M_PI / NEEDLES;
    nring.push_back({std::cos(a), 0.0, std::sin(a)});
  }
  return nring;
}

// The three turtle rotation matrices (built once per forest).
struct TurtleRot {
  Mat4 micro, tilt, roll;
};
TurtleRot turtle_rot() {
  return {mRot(TILT * MICRO_TILT, 0, 0, 1), mRot(TILT, 0, 0, 1), mRot(YROTATE, 0, 1, 0)};
}

inline void push_point(cvc::geometry &g, vec3 p) {
  g.points().push_back(cvc::geometry::point_t{{p.x, p.y, p.z}});
}
inline void push_color(cvc::geometry &g, vec3 c) {
  g.colors().push_back(cvc::geometry::color_t{{c.x, c.y, c.z}});
}

// Grow one pine into the shared wood (tris + per-vertex colour) + needle (lines) meshes, recording
// per-module re-pose data into `out_tree` when non-null. Verbatim topology from the demo.
void grow_pine_impl(cvc::geometry &wood, cvc::geometry &needle, double px, double py, double pz,
                    double size, int maturity, const CylTopo &cyl, const std::vector<vec3> &nring,
                    const TurtleRot &rot, forest_wind::tree *out_tree) {
  std::vector<Module> mods;
  expandTree(TREE_RULES[0], maturity, size, size, -1, 1, mods, rot.micro, rot.tilt, rot.roll);
  const Mat4 tUp = mRot(M_PI / 2.0, 1.0, 0.0, 0.0); // turtle +Y -> world Z-up
  std::vector<Mat4> world(mods.size());
  for (std::size_t i = 0; i < mods.size(); ++i) {
    const Module &mod = mods[i];
    const Mat4 hang = (mod.parent < 0) ? mMul(mTrans(px, py, pz), tUp) : mod.hang;
    world[i] = (mod.parent < 0) ? hang : mMul(world[mod.parent], hang);
    forest_wind::mod_rec rec;
    rec.parent = mod.parent;
    rec.hang = to_array(hang);
    rec.swayer = mod.level <= SWAY_LEVELS;
    rec.w_off = static_cast<int>(wood.points().size());
    rec.n_off = static_cast<int>(needle.points().size());
    for (const Seg &s : mod.segs) {
      vec3 loc[2 * BASE_TRI + 2];
      loc[0] = {0, 0, 0};
      loc[BASE_TRI + 1] = {0, s.len, 0};
      for (int r = 0; r < BASE_TRI; ++r) {
        loc[1 + r] = {cyl.ringUnit[r].x * s.rad, 0.0, cyl.ringUnit[r].z * s.rad};
        loc[BASE_TRI + 2 + r] = {cyl.ringUnit[r].x * s.rad, s.len, cyl.ringUnit[r].z * s.rad};
      }
      const std::uint64_t base = static_cast<std::uint64_t>(wood.points().size());
      for (int v = 0; v < 2 * BASE_TRI + 2; ++v) {
        const vec3 p = xform(s.m, loc[v]); // module-frame vertex (what the wind cascade re-poses)
        if (out_tree)
          rec.local_wood.push_back(p);
        push_point(wood, xform(world[i], p));
        push_color(wood, cyl.colors[v]);
      }
      for (std::size_t k = 0; k < cyl.tris.size(); k += 3)
        wood.tris().push_back(cvc::geometry::tri_t{
            {base + cyl.tris[k], base + cyl.tris[k + 1], base + cyl.tris[k + 2]}});
    }
    for (const Leaf &lf : mod.leaves) {
      const std::uint64_t base = static_cast<std::uint64_t>(needle.points().size());
      const vec3 root = xform(lf.m, {0, 0, 0});
      if (out_tree)
        rec.local_needle.push_back(root);
      push_point(needle, xform(world[i], root));
      for (int t = 0; t < NEEDLES; ++t) {
        const vec3 tip{nring[t].x * LEAF_RAD * lf.sc, LEAF_LEN * lf.sc,
                       nring[t].z * LEAF_RAD * lf.sc};
        const vec3 pm = xform(lf.m, tip);
        if (out_tree)
          rec.local_needle.push_back(pm);
        push_point(needle, xform(world[i], pm));
        needle.lines().push_back(
            cvc::geometry::line_t{{base, base + 1 + static_cast<std::uint64_t>(t)}});
      }
    }
    if (out_tree)
      out_tree->mods.push_back(std::move(rec));
  }
}

// Append a tapered cylinder (p0->p1, radii r0->r1) as a ring-sided tube with per-vertex colour.
void add_cylinder(cvc::geometry &g, vec3 p0, vec3 p1, double r0, double r1, int ring, vec3 col) {
  const vec3 axis = normalize3(p1 - p0);
  const vec3 ref = std::fabs(axis.z) < 0.9 ? vec3{0, 0, 1} : vec3{1, 0, 0};
  const vec3 u = normalize3(cross(axis, ref));
  const vec3 v = normalize3(cross(axis, u));
  const std::uint64_t base = static_cast<std::uint64_t>(g.points().size());
  for (int k = 0; k < ring; ++k) {
    const double a = 2.0 * M_PI * k / ring;
    const vec3 dir = u * std::cos(a) + v * std::sin(a);
    push_point(g, p0 + dir * r0);
    push_point(g, p1 + dir * r1);
    push_color(g, col);
    push_color(g, col);
  }
  for (int k = 0; k < ring; ++k) {
    const std::uint64_t a0 = base + 2u * static_cast<std::uint64_t>(k);
    const std::uint64_t a1 = a0 + 1u;
    const std::uint64_t b0 = base + 2u * static_cast<std::uint64_t>((k + 1) % ring);
    const std::uint64_t b1 = b0 + 1u;
    g.tris().push_back(cvc::geometry::tri_t{{a0, b0, b1}});
    g.tris().push_back(cvc::geometry::tri_t{{a0, b1, a1}});
  }
}

// Grow one recursive branchy limb. `node` is a stable derivation id (NOT a loop counter); children
// get path_id(node, b), so a sub-branch's jitter is independent of its siblings and reproducible.
void grow_branchy_rec(cvc::geometry &g, std::uint64_t seed, std::uint64_t node, vec3 basePt,
                      vec3 dir, double length, double radius, int level, int levels, int branches) {
  const vec3 tip = basePt + dir * length;
  const double tipR = radius * 0.62;
  const bool leaf = (level >= levels);
  const vec3 col =
      leaf ? C_NEEDLE
           : C_WOOD_DARK + (C_WOOD_LIGHT - C_WOOD_DARK) * (double(level) / std::max(1, levels));
  add_cylinder(g, basePt, tip, radius, tipR, level == 0 ? 7 : 5, col);
  if (leaf)
    return;
  const vec3 ref = std::fabs(dir.z) < 0.9 ? vec3{0, 0, 1} : vec3{1, 0, 0};
  const vec3 u = normalize3(cross(dir, ref));
  const vec3 v = normalize3(cross(dir, u));
  const int n = branches + (uni(seed, stream::param_jitter, node, 0) < 0.4 ? 1 : 0);
  for (int b = 0; b < n; ++b) {
    const std::uint32_t d = 1u + 3u * static_cast<std::uint32_t>(b);
    const double az = 2.0 * M_PI * (b + uni(seed, stream::param_jitter, node, d, -0.2, 0.2)) / n;
    const double tilt = uni(seed, stream::param_jitter, node, d + 1, 0.45, 0.80);
    const double lenFactor = uni(seed, stream::param_jitter, node, d + 2, 0.62, 0.78);
    const vec3 out = u * std::cos(az) + v * std::sin(az);
    const vec3 cdir = normalize3(dir * std::cos(tilt) + out * std::sin(tilt));
    grow_branchy_rec(g, seed, path_id(node, static_cast<std::uint32_t>(b)), tip, cdir,
                     length * lenFactor, radius * 0.58, level + 1, levels, branches);
  }
}

} // namespace

void grow_pine(cvc::geometry &wood, cvc::geometry &needle, double bx, double by, double bz,
               double size, int maturity, forest_wind::tree *out_tree, double phase, double sway) {
  const CylTopo cyl = cyl_topo();
  const std::vector<vec3> nring = needle_ring();
  const TurtleRot rot = turtle_rot();
  grow_pine_impl(wood, needle, bx, by, bz, size, maturity, cyl, nring, rot, out_tree);
  if (out_tree) {
    out_tree->phase = phase;
    out_tree->sway = sway;
  }
}

void grow_branchy(cvc::geometry &wood, std::uint64_t seed, double bx, double by, double bz,
                  double length, double radius, int levels, int branches) {
  grow_branchy_rec(wood, seed, /*node=*/seed, vec3{bx, by, bz}, vec3{0, 0, 1}, length, radius, 0,
                   levels, branches);
}

forest_result grow_forest(const forest_params &p, const height_fn &height_at, cvc::geometry &wood,
                          cvc::geometry &needle, forest_wind *wind) {
  const CylTopo cyl = cyl_topo();
  const std::vector<vec3> nring = needle_ring();
  const TurtleRot rot = turtle_rot();
  const double baseLen = p.length * p.scale;
  const double baseRad = p.radius * p.scale;

  forest_result res;
  const int cap = std::max(1, p.count) * 6;
  for (int a = 0; a < cap && res.planted < p.count; ++a) {
    const std::uint64_t el = static_cast<std::uint64_t>(a); // STABLE element id = the attempt index
    const double x = uni(p.seed, stream::placement, el, 0, -p.span, p.span);
    const double y = uni(p.seed, stream::placement, el, 1, -p.span, p.span);
    if (std::sqrt(x * x + y * y) > p.span)
      continue;
    const double z = height_at ? height_at(x, y) : 0.0;
    if (z < p.sea_level + 0.5)
      continue; // dry land above the waterline only
    const bool pine = (p.species == species_mix::pine) ||
                      (p.species == species_mix::mix && (res.planted % 2 == 0));
    if (pine) {
      const double size = (0.55 + 0.4 * uni(p.seed, stream::size, el, 0)) * p.pine_scale;
      const int maturity = MATURITY[irand(p.seed, stream::maturity, el, 0, 0, 6)];
      const double phase = uni(p.seed, stream::phase, el, 0) * 2.0 * M_PI;
      const double sway = 0.020 + 0.016 * uni(p.seed, stream::sway, el, 0);
      if (wind) {
        wind->trees.emplace_back();
        forest_wind::tree &tr = wind->trees.back();
        grow_pine_impl(wood, needle, x, y, z, size, maturity, cyl, nring, rot, &tr);
        tr.phase = phase;
        tr.sway = sway;
      } else {
        grow_pine_impl(wood, needle, x, y, z, size, maturity, cyl, nring, rot, nullptr);
      }
      ++res.pines;
    } else {
      const double lenJitter = uni(p.seed, stream::size, el, 1, 0.8, 1.25);
      // A per-tree branchy seed, mixed from the forest seed + the attempt id, so each tree's
      // branch jitter is independent and insertion-stable.
      const std::uint64_t treeSeed = detail::splitmix(p.seed ^ path_id(el, 0xB2A4u));
      grow_branchy_rec(wood, treeSeed, /*node=*/treeSeed, vec3{x, y, z}, vec3{0, 0, 1},
                       baseLen * lenJitter, baseRad, 0, p.levels, p.branches);
    }
    ++res.planted;
  }
  return res;
}

void repose_forest(const forest_wind &w, double t, double wind_scale, std::vector<double> &wood_buf,
                   std::vector<double> &needle_buf) {
  for (const forest_wind::tree &tree : w.trees) {
    const double a = tree.sway * wind_scale * std::sin(1.3 * t + tree.phase);
    const Mat4 sway = mRot(a, 0.0, 1.0, 0.0); // tree-local +Y axis
    std::vector<Mat4> world(tree.mods.size());
    for (std::size_t i = 0; i < tree.mods.size(); ++i) {
      const forest_wind::mod_rec &m = tree.mods[i];
      const Mat4 hang = from_array(m.hang);
      const Mat4 local = m.swayer ? mMul(hang, sway) : hang;
      world[i] = (m.parent < 0) ? local : mMul(world[static_cast<std::size_t>(m.parent)], local);
      int wo = m.w_off;
      for (const vec3 &p : m.local_wood) {
        const vec3 wv = xform(world[i], p);
        if (static_cast<std::size_t>(wo) * 3 + 2 < wood_buf.size()) {
          wood_buf[wo * 3] = wv.x;
          wood_buf[wo * 3 + 1] = wv.y;
          wood_buf[wo * 3 + 2] = wv.z;
        }
        ++wo;
      }
      int no = m.n_off;
      for (const vec3 &p : m.local_needle) {
        const vec3 wv = xform(world[i], p);
        if (static_cast<std::size_t>(no) * 3 + 2 < needle_buf.size()) {
          needle_buf[no * 3] = wv.x;
          needle_buf[no * 3 + 1] = wv.y;
          needle_buf[no * 3 + 2] = wv.z;
        }
        ++no;
      }
    }
  }
}

} // namespace lsys
} // namespace cvc

// geometry_simplify_test -- the QEM half-edge decimator (cvc::simplify).
//
// Covers: a flat sheet collapses to near-zero triangles at ~zero world error; a
// bumpy sheet reduces toward the triangle target without NaNs, degenerate faces,
// or triangle-count growth; the boundary and the bounding box are preserved; the
// pooled setup is bit-identical to the serial setup; and a no-op target returns
// the input untouched.
//
// The metric half runs on an unwelded "box city" (separate wall and roof meshes,
// closed sheds, a round tower on a podium) with preserve_boundary on: the
// reported world_error is a sampled symmetric Hausdorff distance that scales
// linearly with the mesh, matches an independent brute-force measurement
// (changed centroids included) and holds the pixel budget at the switch radius
// select derives from it; input normals survive bit-for-bit; progressive
// snapshots equal independent runs bit-for-bit; seam welding keeps roofs sealed
// to their walls. Fine parts in a large scene keep their own scale -- they do
// not weld into one another under the default tolerance, and what an explicit
// one welds shut is still measured -- a position two parts share welds to
// both, and a near-coincident join stays closed, also when float32 rounding
// far from the origin, a sliver or a point at a join vertex sets its sides
// apart. Splits whose attributes agree
// weld into exactly the welded mesh (a soup or a flat-shaded sheet: no crack,
// no orphaned vertex, no uv sheared off its position); genuine attribute seams
// -- color blocks, a crease inside one component (also across -0/+0), the
// edges of a closed box, a round patch -- collapse only along themselves,
// never open, never trade attributes, keep their line and still reach the
// target; a double-sided sheet keeps its border; and seamed spheres never fold
// two faces together or put a third on an edge. Collapse ties survive scaling
// and moving the mesh; a far away part cannot change how another decimates;
// the last triangle is never removed; needles are not measured and non-finite
// input is never measured as exact; coincident surfaces measure 0 far from the
// origin; and neither a huge triangle under fine detail, a pile of coincident
// or near-coincident corners nor a finely tessellated part in a large scene
// slows the measurement or the weld.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/simplify.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <functional>
#include <gtest/gtest.h>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <utility>
#include <vector>

using cvc::geometry;

namespace {

const double kPi = 3.14159265358979323846;

// n x n vertex grid in the z=0 plane, triangulated (2*(n-1)^2 tris). If `amp`>0,
// displace z by amp*sin*cos so the sheet is genuinely non-planar.
geometry make_grid(cvc::app &ctx, int n, double amp) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      double z = amp * std::sin(0.7 * i) * std::cos(0.9 * j);
      P.push_back({double(i), double(j), z});
    }
  geometry::tris_t &T = g.tris();
  auto idx = [n](int i, int j) { return std::uint64_t(i * n + j); };
  for (int i = 0; i < n - 1; ++i)
    for (int j = 0; j < n - 1; ++j) {
      T.push_back({idx(i, j), idx(i + 1, j), idx(i, j + 1)});
      T.push_back({idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)});
    }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

bool finite_pt(const geometry::point_t &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// A mesh is well-formed if every position is finite and no triangle repeats an
// index or points out of range.
void expect_wellformed(const geometry &g) {
  for (const auto &p : g.const_points())
    ASSERT_TRUE(finite_pt(p));
  const auto np = g.const_points().size();
  for (const auto &t : g.const_tris()) {
    EXPECT_LT(t[0], np);
    EXPECT_LT(t[1], np);
    EXPECT_LT(t[2], np);
    EXPECT_TRUE(t[0] != t[1] && t[1] != t[2] && t[0] != t[2]) << "degenerate triangle";
  }
}

// ---------------------------------------------------------------------------
// Box-city fixture. Every part is its own connected component (its own vertex
// indices, touching its neighbours only by coincident positions, exactly like
// the per-part meshes of an imported city model). Each vertex's color is a
// (part id, role, 0) tag so a test can tell the parts apart after
// simplification; its uv is (source index, 0) so a surviving vertex can be
// traced back to the input.

enum role { WALL = 1, ROOF = 2, SHED = 3 };

struct city {
  geometry g;
  int nparts = 0;
  explicit city(cvc::app &ctx) : g(ctx) {}

  std::uint64_t vert(double x, double y, double z, int part, int r) {
    geometry::points_t &P = g.points();
    g.colors().push_back({double(part), double(r), 0.0});
    g.uvs().push_back({double(P.size()), 0.0});
    P.push_back({x, y, z});
    return P.size() - 1;
  }
  void quad(std::uint64_t a, std::uint64_t b, std::uint64_t c, std::uint64_t d) {
    g.tris().push_back({a, b, c});
    g.tris().push_back({a, c, d});
  }

  // Closed ring of footprint points (counter-clockwise from above) extruded from
  // z0 to z0+h in `levels` bands: a welded wall strip, open at top and bottom.
  void walls(const std::vector<std::array<double, 2>> &ring, double z0, double h, int levels) {
    const int part = nparts++;
    const std::size_t n = ring.size();
    std::vector<std::vector<std::uint64_t>> id(levels + 1, std::vector<std::uint64_t>(n));
    for (int j = 0; j <= levels; ++j)
      for (std::size_t i = 0; i < n; ++i)
        id[j][i] = vert(ring[i][0], ring[i][1], z0 + h * j / levels, part, WALL);
    for (int j = 0; j < levels; ++j)
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t k = (i + 1) % n;
        quad(id[j][i], id[j][k], id[j + 1][k], id[j + 1][i]);
      }
  }

  // Rectangle footprint ring with nx / ny segments along x / y.
  static std::vector<std::array<double, 2>> rect(double x0, double y0, double sx, double sy, int nx,
                                                 int ny) {
    std::vector<std::array<double, 2>> r;
    for (int i = 0; i < nx; ++i)
      r.push_back({x0 + sx * i / nx, y0});
    for (int i = 0; i < ny; ++i)
      r.push_back({x0 + sx, y0 + sy * i / ny});
    for (int i = nx; i > 0; --i)
      r.push_back({x0 + sx * i / nx, y0 + sy});
    for (int i = ny; i > 0; --i)
      r.push_back({x0, y0 + sy * i / ny});
    return r;
  }

  // A box building: rectangle walls plus a SEPARATE flat roof grid whose rim
  // vertices coincide with the walls' top ring.
  void building(double x0, double y0, double z0, double sx, double sy, double h, int nx, int ny,
                int levels) {
    walls(rect(x0, y0, sx, sy, nx, ny), z0, h, levels);
    const int part = nparts++;
    std::vector<std::vector<std::uint64_t>> id(nx + 1, std::vector<std::uint64_t>(ny + 1));
    for (int i = 0; i <= nx; ++i)
      for (int j = 0; j <= ny; ++j)
        id[i][j] = vert(x0 + sx * i / nx, y0 + sy * j / ny, z0 + h, part, ROOF);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j)
        quad(id[i][j], id[i + 1][j], id[i + 1][j + 1], id[i][j + 1]);
  }

  // A round tower: an n-gon wall strip plus a SEPARATE fan roof on its top ring.
  void tower(double cx, double cy, double z0, double r, double h, int n, int levels) {
    std::vector<std::array<double, 2>> ring;
    for (int i = 0; i < n; ++i)
      ring.push_back({cx + r * std::cos(2 * kPi * i / n), cy + r * std::sin(2 * kPi * i / n)});
    walls(ring, z0, h, levels);
    const int part = nparts++;
    const std::uint64_t c = vert(cx, cy, z0 + h, part, ROOF);
    std::vector<std::uint64_t> rim(n);
    for (int i = 0; i < n; ++i)
      rim[i] = vert(ring[i][0], ring[i][1], z0 + h, part, ROOF);
    for (int i = 0; i < n; ++i)
      g.tris().push_back({c, rim[i], rim[(i + 1) % n]});
  }

  // A closed 12-triangle shed.
  void shed(double x0, double y0, double z0, double sx, double sy, double h) {
    const int part = nparts++;
    std::uint64_t v[8];
    for (int k = 0; k < 8; ++k)
      v[k] = vert(x0 + ((k & 1) ? sx : 0.0), y0 + ((k & 2) ? sy : 0.0), z0 + ((k & 4) ? h : 0.0),
                  part, SHED);
    quad(v[0], v[2], v[3], v[1]); // bottom (-z)
    quad(v[4], v[5], v[7], v[6]); // top (+z)
    quad(v[0], v[1], v[5], v[4]); // -y
    quad(v[2], v[6], v[7], v[3]); // +y
    quad(v[0], v[4], v[6], v[2]); // -x
    quad(v[1], v[3], v[7], v[5]); // +x
  }
};

// The fixture: a few box buildings of different sizes, sheds, and a round tower
// standing on a podium building, placed away from the origin.
geometry box_city(cvc::app &ctx, double scale = 1.0) {
  city c(ctx);
  const double ox = 310.0, oy = -145.0, oz = 12.0;
  c.building(ox + 0, oy + 0, oz, 20, 14, 9, 8, 6, 4);
  c.building(ox + 26, oy + 2, oz, 12, 18, 23, 5, 7, 8);
  c.building(ox + 4, oy + 22, oz, 16, 9, 6, 6, 4, 3);
  c.shed(ox + 40, oy + 0, oz, 3, 2.5, 2.2);
  c.shed(ox + 44.5, oy + 1, oz, 2, 2, 2.6);
  c.shed(ox + 23, oy + 26, oz, 4, 3, 2);
  c.building(ox + 30, oy + 24, oz, 24, 20, 7, 8, 7, 2); // podium
  c.tower(ox + 42, oy + 34, oz + 7, 5.5, 30, 24, 10);   // tower on the podium roof
  geometry &g = c.g;
  for (auto &p : g.points())
    for (int k = 0; k < 3; ++k)
      p[k] *= scale;
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

// Deterministic, deliberately non-geometric per-vertex normals (unit length),
// so a carried normal cannot be confused with a recomputed one.
void paint_normals(geometry &g) {
  geometry::normals_t &N = g.normals();
  N.clear();
  for (std::size_t i = 0; i < g.const_points().size(); ++i) {
    const double a = 0.37 * double(i), b = 0.11 * double(i);
    N.push_back({std::cos(a) * std::sin(b), std::sin(a) * std::sin(b), std::cos(b)});
  }
}

template <class V> bool bitwise_equal(const V &a, const V &b) {
  if (a.size() != b.size())
    return false;
  return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(a[0])) == 0;
}

// ---------------------------------------------------------------------------
// Independent brute-force Hausdorff: a different point-to-triangle routine
// (plane projection + barycentric inside test, else nearest edge) and an
// O(N x M) scan, sharing no code with the library's grid-accelerated one.

typedef std::array<double, 3> v3;
v3 to3(const geometry::point_t &p) { return {p[0], p[1], p[2]}; }
v3 vsub(const v3 &a, const v3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
double vdot(const v3 &a, const v3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
v3 vcross(const v3 &a, const v3 &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double brute_seg(const v3 &p, const v3 &a, const v3 &b) {
  const v3 ab = vsub(b, a);
  const double l2 = vdot(ab, ab);
  double t = l2 > 0 ? vdot(vsub(p, a), ab) / l2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  const v3 q = {a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]};
  return std::sqrt(vdot(vsub(p, q), vsub(p, q)));
}

double brute_tri(const v3 &p, const v3 &a, const v3 &b, const v3 &c) {
  const v3 n = vcross(vsub(b, a), vsub(c, a));
  const double nn = vdot(n, n);
  if (nn > 0) {
    const double s = vdot(vsub(p, a), n) / nn; // signed distance / |n|
    const v3 q = {p[0] - s * n[0], p[1] - s * n[1], p[2] - s * n[2]};
    // q inside iff it is on the inner side of all three edges
    const double wa = vdot(vcross(vsub(b, q), vsub(c, q)), n);
    const double wb = vdot(vcross(vsub(c, q), vsub(a, q)), n);
    const double wc = vdot(vcross(vsub(a, q), vsub(b, q)), n);
    if (wa >= 0 && wb >= 0 && wc >= 0)
      return std::fabs(s) * std::sqrt(nn);
  }
  return std::min(brute_seg(p, a, b), std::min(brute_seg(p, b, c), brute_seg(p, c, a)));
}

double brute_dist(const v3 &p, const geometry &g) {
  double best = std::numeric_limits<double>::infinity();
  const auto &P = g.const_points();
  for (const auto &t : g.const_tris())
    best = std::min(best, brute_tri(p, to3(P[t[0]]), to3(P[t[1]]), to3(P[t[2]])));
  return best;
}

// Samples of a surface. `dense` == 0: the library's documented sampling (used
// vertices, unique edge midpoints, face centroids). `dense` > 0: a barycentric
// lattice of `dense` steps per edge on every triangle.
std::vector<v3> brute_samples(const geometry &g, int dense) {
  const auto &P = g.const_points();
  std::vector<v3> s;
  if (dense == 0) {
    std::set<std::uint64_t> used;
    std::set<std::pair<std::uint64_t, std::uint64_t>> edges;
    for (const auto &t : g.const_tris())
      for (int k = 0; k < 3; ++k) {
        used.insert(t[k]);
        edges.insert(
            std::make_pair(std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3])));
      }
    for (std::uint64_t v : used)
      s.push_back(to3(P[v]));
    for (const auto &e : edges) {
      const v3 a = to3(P[e.first]), b = to3(P[e.second]);
      s.push_back({0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), 0.5 * (a[2] + b[2])});
    }
    for (const auto &t : g.const_tris()) {
      const v3 a = to3(P[t[0]]), b = to3(P[t[1]]), c = to3(P[t[2]]);
      s.push_back({(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3});
    }
    return s;
  }
  for (const auto &t : g.const_tris()) {
    const v3 a = to3(P[t[0]]), b = to3(P[t[1]]), c = to3(P[t[2]]);
    for (int i = 0; i <= dense; ++i)
      for (int j = 0; i + j <= dense; ++j) {
        const double u = double(i) / dense, v = double(j) / dense, w = 1.0 - u - v;
        s.push_back({w * a[0] + u * b[0] + v * c[0], w * a[1] + u * b[1] + v * c[1],
                     w * a[2] + u * b[2] + v * c[2]});
      }
  }
  return s;
}

double brute_hausdorff(const geometry &a, const geometry &b, int dense) {
  double h = 0.0;
  for (const v3 &p : brute_samples(a, dense))
    h = std::max(h, brute_dist(p, b));
  for (const v3 &p : brute_samples(b, dense))
    h = std::max(h, brute_dist(p, a));
  return h;
}

// Largest gap between each roof and the walls it sits on: sample every roof
// boundary edge against that building's wall triangles, and every wall edge on
// the top ring against the roof triangles. Parts are told apart by their color
// tag; a roof's walls are the part created just before it.
double max_roof_gap(const geometry &g) {
  const auto &P = g.const_points();
  const auto &C = g.const_colors();
  std::map<int, geometry> parts;
  for (const auto &t : g.const_tris()) {
    const int part = int(C[t[0]][0]);
    auto it = parts.find(part);
    if (it == parts.end())
      it = parts.insert(std::make_pair(part, geometry(g.ctx()))).first;
    it->second.tris().push_back(t);
  }
  auto role_of = [&](int part) {
    for (std::size_t i = 0; i < C.size(); ++i)
      if (int(C[i][0]) == part)
        return int(C[i][1]);
    return 0;
  };
  // boundary edges of one part, optionally restricted to a z level
  auto rim = [&](const geometry &part, double z, bool at_z) {
    std::map<std::pair<std::uint64_t, std::uint64_t>, int> cnt;
    for (const auto &t : part.const_tris())
      for (int k = 0; k < 3; ++k)
        ++cnt[std::make_pair(std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3]))];
    std::vector<std::pair<v3, v3>> out;
    for (const auto &e : cnt)
      if (e.second == 1 && (!at_z || (P[e.first.first][2] == z && P[e.first.second][2] == z)))
        out.push_back(std::make_pair(to3(P[e.first.first]), to3(P[e.first.second])));
    return out;
  };
  double gap = 0.0;
  for (auto &kv : parts) {
    auto walls = parts.find(kv.first - 1);
    if (role_of(kv.first) != ROOF || walls == parts.end())
      continue;
    geometry roof = kv.second, wall = walls->second;
    roof.points() = P;
    wall.points() = P;
    const double ztop = P[roof.const_tris()[0][0]][2];
    auto side = [&](const std::vector<std::pair<v3, v3>> &edges, const geometry &other) {
      for (const auto &e : edges)
        for (int s = 0; s <= 8; ++s) {
          const double t = s / 8.0;
          const v3 p = {e.first[0] + t * (e.second[0] - e.first[0]),
                        e.first[1] + t * (e.second[1] - e.first[1]),
                        e.first[2] + t * (e.second[2] - e.first[2])};
          gap = std::max(gap, brute_dist(p, other));
        }
    };
    side(rim(roof, 0.0, false), wall);
    side(rim(wall, ztop, true), roof);
  }
  return gap;
}

// Parts of a given role that still have triangles.
int parts_with_role(const geometry &g, int r) {
  std::set<int> parts;
  for (const auto &t : g.const_tris())
    if (int(g.const_colors()[t[0]][1]) == r)
      parts.insert(int(g.const_colors()[t[0]][0]));
  return int(parts.size());
}

} // namespace

TEST(GeometrySimplify, FlatSheetCollapsesAtZeroError) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 12, 0.0); // planar
  cvc::simplify_params p;
  p.target_ratio = 0.05;
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_LT(s.num_tris(), g.num_tris());
  // Every collapse on a plane is free: the error must be ~0.
  EXPECT_NEAR(r.world_error, 0.0, 1e-6);
  // A plane can decimate hard (down toward a couple of triangles).
  EXPECT_LE(s.num_tris(), g.num_tris() / 4);
}

TEST(GeometrySimplify, BumpySheetReducesCleanly) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 24, 1.5); // non-planar
  const std::uint64_t in = g.num_tris();
  cvc::simplify_params p;
  p.target_ratio = 0.25;
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_EQ(r.in_tris, in);
  EXPECT_EQ(r.out_tris, s.num_tris());
  EXPECT_LT(s.num_tris(), in);      // it actually reduced
  EXPECT_GT(s.num_tris(), in / 20); // but did not annihilate the surface
  EXPECT_GT(r.world_error, 0.0);    // a curved sheet pays real error
  EXPECT_GT(r.collapses, 0u);
}

TEST(GeometrySimplify, PreservesBoundingBox) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 20, 1.0);
  cvc::bounding_box b0 = g.extents();
  cvc::simplify_params p;
  p.target_ratio = 0.1;
  p.preserve_boundary = true;
  geometry s = cvc::simplify(g, p);
  cvc::bounding_box b1 = s.extents();
  // Boundary preservation keeps the border vertices, so the XY extent is intact.
  EXPECT_NEAR(b0.minx, b1.minx, 1e-9);
  EXPECT_NEAR(b0.miny, b1.miny, 1e-9);
  EXPECT_NEAR(b0.maxx, b1.maxx, 1e-9);
  EXPECT_NEAR(b0.maxy, b1.maxy, 1e-9);
}

TEST(GeometrySimplify, PoolMatchesSerialBitForBit) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 40, 2.0); // ~3000 tris -> the pool actually fans out
  cvc::simplify_params p;
  p.target_ratio = 0.3;
  cvc::simplify_result rs, rp;
  geometry serial = cvc::simplify(g, p, &rs, nullptr);
  geometry pooled = cvc::simplify(g, p, &rp, &ctx.computePool());

  ASSERT_EQ(serial.num_points(), pooled.num_points());
  ASSERT_EQ(serial.num_tris(), pooled.num_tris());
  EXPECT_EQ(rs.out_tris, rp.out_tris);
  EXPECT_DOUBLE_EQ(rs.world_error, rp.world_error);
  for (std::size_t i = 0; i < serial.const_points().size(); ++i)
    for (int k = 0; k < 3; ++k)
      EXPECT_DOUBLE_EQ(serial.const_points()[i][k], pooled.const_points()[i][k]);
  for (std::size_t i = 0; i < serial.const_tris().size(); ++i)
    for (int k = 0; k < 3; ++k)
      EXPECT_EQ(serial.const_tris()[i][k], pooled.const_tris()[i][k]);
}

TEST(GeometrySimplify, NoOpWhenTargetAboveInput) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 10, 1.0);
  cvc::simplify_params p;
  p.target_ratio = 1.0; // ask for everything
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  EXPECT_EQ(s.num_tris(), g.num_tris());
  EXPECT_EQ(r.collapses, 0u);
  EXPECT_EQ(r.world_error, 0.0);

  cvc::simplify_params p2;
  p2.target_tris = g.num_tris() + 100; // explicit over-target
  geometry s2 = cvc::simplify(g, p2);
  EXPECT_EQ(s2.num_tris(), g.num_tris());
}

TEST(GeometrySimplify, EmptyMeshIsSafe) {
  cvc::app ctx;
  geometry g(ctx);
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, cvc::simplify_params(), &r);
  EXPECT_EQ(s.num_tris(), 0u);
  EXPECT_EQ(r.in_tris, 0u);
}

TEST(GeometrySimplify, ErrorBudgetStopsEarly) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 24, 1.5);
  cvc::simplify_params p;
  p.target_tris = 2;  // ask to go all the way down...
  p.max_error = 0.05; // ...but cap the per-collapse error proxy
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_TRUE(r.hit_error_limit);
  EXPECT_GT(s.num_tris(), 2u); // the error cap kept it above the tri target
  EXPECT_GT(r.world_error, 0.0);

  // A looser budget goes further and pays more (measured) error.
  cvc::simplify_params loose = p;
  loose.max_error = 0.4;
  cvc::simplify_result rl;
  geometry sl = cvc::simplify(g, loose, &rl);
  EXPECT_LT(sl.num_tris(), s.num_tris());
  EXPECT_GE(rl.world_error, r.world_error);
}

// --- metric ----------------------------------------------------------------

namespace {

double bbox_diag(const geometry &g) {
  const cvc::bounding_box b = g.extents();
  const double dx = b.maxx - b.minx, dy = b.maxy - b.miny, dz = b.maxz - b.minz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// A small city (one box building, a round tower, a shed) for brute-force checks.
geometry mini_city(cvc::app &ctx) {
  city c(ctx);
  c.building(-6, -4, 1.5, 8, 6, 5, 4, 3, 2);
  c.tower(9, 2, 1.5, 3, 11, 12, 3);
  c.shed(-3, 6, 1.5, 2.5, 2, 2);
  c.g.set_geometry_type(geometry::SURFACE_TRI);
  return c.g;
}

} // namespace

// B1: the error is a length. Scaling the whole box city 10x (walls, roofs and
// sheds all carrying preserve_boundary constraints) must scale world_error 10x,
// at every depth where the error is real rather than round-off.
TEST(GeometrySimplify, WorldErrorScalesWithTheMesh) {
  cvc::app ctx;
  const geometry g1 = box_city(ctx, 1.0), g10 = box_city(ctx, 10.0);
  const double floor = 1e-3 * bbox_diag(g1);
  struct Case {
    std::uint64_t target;
    bool weld;
  };
  // Welded, the city loses nothing down to ~146 triangles; unwelded, its parts
  // decimate on their own.
  const Case cases[] = {{132, true}, {106, true},  {85, true},
                        {61, true},  {118, false}, {76, false}};
  for (const Case &c : cases) {
    cvc::simplify_params p;
    p.target_tris = c.target;
    p.weld_seams = c.weld;
    ASSERT_TRUE(p.preserve_boundary);
    cvc::simplify_result r1, r10;
    cvc::simplify(g1, p, &r1);
    cvc::simplify(g10, p, &r10);
    ASSERT_GT(r1.world_error, floor) << "target " << c.target << ": error is only round-off";
    EXPECT_NEAR(r10.world_error / (10.0 * r1.world_error), 1.0, 0.05)
        << "target " << c.target << " weld " << c.weld;
  }
}

// world_error is the documented sampled symmetric Hausdorff distance: it equals
// an independent O(N x M) scan over the same samples and the public
// sampled_hausdorff() of the source and the result, and a much denser sampling
// finds at most the documented sampling slack more.
TEST(GeometrySimplify, WorldErrorIsTheSampledHausdorffDistance) {
  cvc::app ctx;
  const geometry g = mini_city(ctx);
  struct Case {
    std::uint64_t target;
    bool weld;
  };
  for (const Case &c : {Case{50, true}, Case{40, true}, Case{45, false}}) {
    cvc::simplify_params p;
    p.target_tris = c.target;
    p.weld_seams = c.weld;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    SCOPED_TRACE(testing::Message() << "target " << c.target << " weld " << c.weld);
    ASSERT_GT(r.world_error, 0.1);

    const double brute = brute_hausdorff(g, s, 0);
    EXPECT_NEAR(r.world_error, brute, 1e-9 * brute);
    EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));

    // A 12-step barycentric lattice contains every sample, so it can only find
    // more -- and by no more than the sample spacing, which is under half the
    // longest edge of either surface (0.3x, for the midpoint subdivision).
    double longest = 0.0;
    for (const geometry *m : {&g, &s})
      for (const auto &t : m->const_tris())
        for (int k = 0; k < 3; ++k) {
          const v3 e = vsub(to3(m->const_points()[t[k]]), to3(m->const_points()[t[(k + 1) % 3]]));
          longest = std::max(longest, std::sqrt(vdot(e, e)));
        }
    const double dense = brute_hausdorff(g, s, 12);
    EXPECT_GE(dense, r.world_error - 1e-12);
    EXPECT_LE(dense, r.world_error + 0.3 * longest);
  }
}

// A changed source triangle's centroid is a sample too. Here it is the one
// that sees the error: a sliver (D, K, x) lies along the x axis, D a hair off
// K, next to three upright triangles standing on K, on the middle of Kx, and on
// x. The one collapse, D onto K, removes the sliver; each of its vertices and
// edge midpoints is on (or a hair off) what remains, but its centroid is ~1
// away -- and world_error reports that, as sampled_hausdorff does.
TEST(GeometrySimplify, WorldErrorSamplesChangedCentroids) {
  cvc::app ctx;
  geometry g(ctx);
  g.points() = {{0, 1e-3, 0}, {0, 0, 0},   {6, 0, 0},   // the sliver: D, K, x
                {0, 0, 0},    {-1, 0, 2},  {1, 0, 2},   // standing on K
                {3, 0, 0},    {2.5, 0, 2}, {3.5, 0, 2}, // on the middle of Kx
                {6, 0, 0},    {5, 0, 2},   {7, 0, 2}};  // on x
  g.tris() = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};
  g.set_geometry_type(geometry::SURFACE_TRI);
  for (bool weld : {false, true}) {
    cvc::simplify_params p;
    p.target_tris = 3;
    p.weld_seams = weld;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    SCOPED_TRACE(testing::Message() << "weld " << weld);
    ASSERT_EQ(r.collapses, 1u);
    ASSERT_EQ(s.num_tris(), 3u);
    const v3 centroid = {2.0, 1e-3 / 3.0, 0.0};
    EXPECT_NEAR(r.world_error, brute_dist(centroid, s), 1e-12);
    EXPECT_GT(r.world_error, 0.9);
    EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
  }
}

// The grid search must return exact nearest distances even when the other
// surface is far away (many empty shells) or offset outside its bounds.
TEST(GeometrySimplify, SampledHausdorffMatchesBruteForce) {
  cvc::app ctx;
  const geometry bumpy = make_grid(ctx, 14, 1.8);
  geometry flat = make_grid(ctx, 5, 0.0);
  for (auto &p : flat.points()) { // a coarse sheet, shifted and tilted away from the bumps
    p[0] = 3.1 * p[0] - 2.0;
    p[1] = 2.7 * p[1] + 5.0;
    p[2] = 0.3 * p[0] + 4.0;
  }
  const double h = cvc::sampled_hausdorff(bumpy, flat);
  EXPECT_NEAR(h, brute_hausdorff(bumpy, flat, 0), 1e-9 * h);
  EXPECT_EQ(h, cvc::sampled_hausdorff(flat, bumpy)); // symmetric
  EXPECT_EQ(h, cvc::sampled_hausdorff(bumpy, flat, &ctx.computePool()));

  // Quads are fan-triangulated: a quad sheet is the same surface as its tris.
  geometry quads(ctx);
  quads.points() = {{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}};
  quads.quads().push_back({0, 1, 2, 3});
  geometry tris(ctx);
  tris.points() = quads.const_points();
  tris.tris().push_back({0, 1, 2});
  tris.tris().push_back({0, 2, 3});
  EXPECT_EQ(cvc::sampled_hausdorff(quads, tris), 0.0);

  // Zero-area triangles are measured as their edges: a sliver lying on a sheet
  // is at distance 0 from it, and the sheet's far corner is measured to the
  // sliver's farthest-reaching edge.
  geometry sliver(ctx);
  sliver.points() = {{0.5, 0.5, 0}, {1.5, 1.5, 0}, {1.0, 1.0, 0}}; // collinear
  sliver.tris().push_back({0, 1, 2});
  const double hs = cvc::sampled_hausdorff(tris, sliver);
  EXPECT_NEAR(hs, brute_hausdorff(tris, sliver, 0), 1e-12);
  EXPECT_GT(hs, 1.0);

  // No surface on one side: nothing bounds the error.
  const geometry empty(ctx);
  EXPECT_EQ(cvc::sampled_hausdorff(empty, empty), 0.0);
  EXPECT_TRUE(std::isinf(cvc::sampled_hausdorff(bumpy, empty)));
}

// Half-edge collapse never moves a survivor, so its input normal is still exact:
// by default it is carried bit-for-bit (a rung switch does not re-shade), and
// recompute_normals=true replaces it with freshly computed smooth normals.
TEST(GeometrySimplify, CarriesSourceNormalsBitForBit) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 20, 1.2);
  paint_normals(g);
  for (std::size_t i = 0; i < g.const_points().size(); ++i)
    g.uvs().push_back({double(i), 0.0}); // trace each survivor to its source
  const geometry::normals_t &src = g.const_normals();

  cvc::simplify_params p;
  p.target_ratio = 0.2;
  geometry s = cvc::simplify(g, p);
  ASSERT_LT(s.num_points(), g.num_points());
  ASSERT_EQ(s.const_normals().size(), s.num_points());
  for (std::size_t i = 0; i < s.num_points(); ++i) {
    const std::size_t from = std::size_t(s.const_uvs()[i][0]);
    EXPECT_EQ(0, std::memcmp(&s.const_normals()[i], &src[from], sizeof(src[from])))
        << "vertex " << i << " (source " << from << ")";
  }

  p.recompute_normals = true;
  geometry r = cvc::simplify(g, p);
  geometry expect = r;
  expect.compute_normals();
  EXPECT_TRUE(bitwise_equal(r.const_normals(), expect.const_normals()));
  EXPECT_TRUE(bitwise_equal(r.const_points(), s.const_points())); // only the normals differ

  // An input without normals gets computed ones either way.
  geometry bare = make_grid(ctx, 20, 1.2);
  geometry b = cvc::simplify(bare, cvc::simplify_params());
  EXPECT_EQ(b.const_normals().size(), b.num_points());
}

// One progressive pass == one independent simplify() per target, bit for bit
// (points, tris, uvs, colors, normals and every result field), for targets in
// any order, with duplicates, an above-input no-op, targets on both sides of
// where the welded city starts to lose geometry, and an error budget that stops
// the pass part-way.
TEST(GeometrySimplify, ProgressiveMatchesIndependentRuns) {
  cvc::app ctx;
  geometry g = box_city(ctx);
  paint_normals(g);
  const std::uint64_t n = g.num_tris();
  const std::vector<std::uint64_t> targets = {600, n + 5, 1200, 290, 60, 290, 250, n};

  for (double budget : {-1.0, 0.35}) {
    cvc::simplify_params p;
    p.max_error = budget;
    std::vector<cvc::simplify_result> res;
    const std::vector<geometry> snaps = cvc::simplify_progressive(g, targets, p, &res);
    ASSERT_EQ(snaps.size(), targets.size());
    ASSERT_EQ(res.size(), targets.size());
    bool any_limited = false;
    for (std::size_t k = 0; k < targets.size(); ++k) {
      cvc::simplify_params pk = p;
      pk.target_tris = targets[k];
      cvc::simplify_result rk;
      const geometry one = cvc::simplify(g, pk, &rk);
      const geometry &snap = snaps[k];
      SCOPED_TRACE(testing::Message() << "budget " << budget << " target " << targets[k]);
      EXPECT_TRUE(bitwise_equal(snap.const_points(), one.const_points()));
      EXPECT_TRUE(bitwise_equal(snap.const_tris(), one.const_tris()));
      EXPECT_TRUE(bitwise_equal(snap.const_uvs(), one.const_uvs()));
      EXPECT_TRUE(bitwise_equal(snap.const_colors(), one.const_colors()));
      EXPECT_TRUE(bitwise_equal(snap.const_normals(), one.const_normals()));
      EXPECT_EQ(res[k].in_tris, rk.in_tris);
      EXPECT_EQ(res[k].out_tris, rk.out_tris);
      EXPECT_EQ(res[k].collapses, rk.collapses);
      EXPECT_EQ(0, std::memcmp(&res[k].world_error, &rk.world_error, sizeof(double)));
      EXPECT_EQ(res[k].hit_error_limit, rk.hit_error_limit);
      EXPECT_EQ(res[k].seam_vertices, rk.seam_vertices);
      any_limited = any_limited || rk.hit_error_limit;
    }
    EXPECT_EQ(any_limited, budget >= 0.0) << "the budget must actually bind part-way";
  }
  // No targets, no work.
  EXPECT_TRUE(cvc::simplify_progressive(g, std::vector<std::uint64_t>()).empty());
}

// The pool fans the face quadrics, the seam search and the Hausdorff samples;
// none of it may change a bit of any snapshot or its error.
TEST(GeometrySimplify, ProgressivePoolMatchesSerial) {
  cvc::app ctx;
  geometry g = box_city(ctx);
  paint_normals(g);
  const std::vector<std::uint64_t> targets = {1000, 500, 300, 137};
  for (bool weld : {true, false}) {
    cvc::simplify_params p;
    p.weld_seams = weld;
    std::vector<cvc::simplify_result> rs, rp;
    const std::vector<geometry> serial = cvc::simplify_progressive(g, targets, p, &rs, nullptr);
    const std::vector<geometry> pooled =
        cvc::simplify_progressive(g, targets, p, &rp, &ctx.computePool());
    for (std::size_t k = 0; k < targets.size(); ++k) {
      SCOPED_TRACE(testing::Message() << "weld " << weld << " target " << targets[k]);
      EXPECT_TRUE(bitwise_equal(serial[k].const_points(), pooled[k].const_points()));
      EXPECT_TRUE(bitwise_equal(serial[k].const_tris(), pooled[k].const_tris()));
      EXPECT_TRUE(bitwise_equal(serial[k].const_normals(), pooled[k].const_normals()));
      EXPECT_EQ(0, std::memcmp(&rs[k].world_error, &rp[k].world_error, sizeof(double)));
      EXPECT_EQ(rs[k].seam_vertices, rp[k].seam_vertices);
    }
    EXPECT_GT(rs.back().world_error, 0.0);
  }
}

// Unwelded roofs sit on their walls through coincident vertices only. Welded
// (the default), the roof rims and the walls' top rings collapse together: they
// thin, yet no roof lifts off its walls however hard the city is decimated.
// Without welding the two rims thin independently and open a gap -- which
// proves the gap measurement bites.
TEST(GeometrySimplify, SeamWeldKeepsRoofsOnWalls) {
  cvc::app ctx;
  const geometry g = box_city(ctx);
  const double eps = 1e-6 * bbox_diag(g); // no more than the default seam_epsilon
  ASSERT_LT(max_roof_gap(g), eps);
  // every roof rim vertex and every wall top-ring vertex: 2 x (28+24+20+30+24)
  const std::uint64_t rim = 126;
  for (std::uint64_t target : {137u, 86u}) {
    cvc::simplify_params p;
    p.target_tris = target;
    cvc::simplify_result on, off;
    const geometry welded = cvc::simplify(g, p, &on);
    p.weld_seams = false;
    const geometry loose = cvc::simplify(g, p, &off);
    EXPECT_EQ(on.seam_vertices, 2 * rim);
    EXPECT_EQ(off.seam_vertices, 0u);
    EXPECT_LE(on.out_tris, target);
    ASSERT_EQ(parts_with_role(welded, ROOF), 5) << "every roof is still there to measure";
    EXPECT_LE(max_roof_gap(welded), eps) << "target " << target;
    EXPECT_GT(max_roof_gap(loose), 100.0 * eps) << "target " << target;
    // the seams were decimated, not frozen: fewer roof vertices survive than
    // the rims alone held
    std::uint64_t roof_verts = 0;
    for (const auto &c : welded.const_colors())
      roof_verts += int(c[1]) == ROOF ? 1 : 0;
    EXPECT_LT(roof_verts, rim) << "target " << target;
  }
}

// seam_epsilon is the coincidence tolerance: a roof lifted by a hair is still
// seamed to its walls within a looser tolerance, and not within a tighter one.
TEST(GeometrySimplify, SeamEpsilonIsTheCoincidenceTolerance) {
  cvc::app ctx;
  city c(ctx);
  c.building(0, 0, 0, 10, 8, 6, 3, 2, 2);
  geometry g = c.g;
  g.set_geometry_type(geometry::SURFACE_TRI);
  for (std::size_t i = 0; i < g.const_points().size(); ++i)
    if (int(g.const_colors()[i][1]) == ROOF)
      g.points()[i][2] += 1e-5;
  cvc::simplify_params p;
  p.target_ratio = 0.5;
  cvc::simplify_result r;
  p.seam_epsilon = 1e-4;
  cvc::simplify(g, p, &r);
  EXPECT_EQ(r.seam_vertices, 20u); // the 10-vertex rim on both parts
  p.seam_epsilon = 1e-6;
  cvc::simplify(g, p, &r);
  EXPECT_EQ(r.seam_vertices, 0u);
  p.seam_epsilon = 0.0; // exact coincidence only
  cvc::simplify(c.g, p, &r);
  EXPECT_EQ(r.seam_vertices, 20u);
  p.seam_epsilon = -1.0; // the default: 1e-6 of this building's ~14 m extent
  cvc::simplify(g, p, &r);
  EXPECT_EQ(r.seam_vertices, 20u);

  // Two parts whose touching corners (0.002 apart) straddle a boundary of the
  // coincidence search's cells (2 * seam_epsilon = 1/32 wide, starting at the
  // bounding box's corner) are still found.
  geometry pair(ctx);
  pair.points() = {{0, 0, 0}, {2.999, 0, 0}, {0, 1, 0}, {3.001, 0, 0}, {5, 0, 0}, {3.001, 1, 0}};
  pair.tris().push_back({0, 1, 2});
  pair.tris().push_back({3, 4, 5});
  cvc::simplify_params q;
  q.target_tris = 1;
  q.seam_epsilon = 1.0 / 64.0;
  cvc::simplify(pair, q, &r);
  EXPECT_EQ(r.seam_vertices, 2u);

  // The default tolerance is capped at each vertex by 1e-3 of its median
  // edge (2 m or more here -- not the 1 km edge each touching corner also
  // has), so a far away part that stretches the extent to 100 km (1e-6 of
  // which is 0.1 m) does not weld two parts 5 mm apart -- while a 1 mm
  // contact still welds.
  for (double gap : {5e-3, 1e-3}) {
    geometry scene(ctx);
    scene.points() = {
        {0, 0, 0},     {3 - gap, 0, 0}, {0, 1, 0},       {3, 0, 0},   {5, 0, 0},
        {3, 1, 0},     {1e5, 0, 0},     {1e5 + 1, 0, 0}, {1e5, 1, 0}, {3 - gap, 0, -1000.0},
        {3, 0, 1000.0}};
    scene.tris() = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {1, 2, 9}, {3, 5, 10}};
    cvc::simplify_params d;
    d.target_tris = 2;
    cvc::simplify(scene, d, &r);
    EXPECT_EQ(r.seam_vertices, gap < 2e-3 ? 2u : 0u) << "gap " << gap;
  }
  // Nor does the floor for float32 rounding -- 4 float32 epsilons of the
  // coordinates, ~0.5 m 1000 km out -- ever pass the scene-wide 1e-6 cap:
  // there, two parts 1 cm apart in a scene 5 m across stay apart.
  const double o = 1e6;
  geometry farMesh(ctx);
  farMesh.points() = {{o, 0, 0},     {o + 2.99, 0, 0}, {o, 1, 0},
                      {o + 3, 0, 0}, {o + 5, 0, 0},    {o + 3, 1, 0}};
  farMesh.tris() = {{0, 1, 2}, {3, 4, 5}};
  cvc::simplify_params d;
  d.target_tris = 1;
  cvc::simplify(farMesh, d, &r);
  EXPECT_EQ(r.seam_vertices, 0u);
}

namespace {

// Triangles of `g` with a corner above z_min, left (x < x_split) and right of
// x_split.
std::pair<int, int> tris_above(const geometry &g, double z_min, double x_split) {
  std::pair<int, int> n(0, 0);
  for (const auto &t : g.const_tris()) {
    const auto &p = g.const_points()[t[0]];
    if (p[2] > z_min)
      ++(p[0] < x_split ? n.first : n.second);
  }
  return n;
}

} // namespace

// Fine parts in a large scene keep their own scale. A bumpy terrain 3 km
// across (50 m edges) carries, 20 m up, a 10 x 10 array of separate closed
// 2 mm tetrahedra 3 mm apart, and a flat-shaded 20 x 20 sheet at 1 mm spacing
// (per-face normals: every vertex split). The scene-wide default tolerance
// (1e-6 of the extent, ~4 mm) spans both parts' own edges, but the default is
// capped at each vertex by 1e-3 of its median edge: no tetrahedron welds to
// its neighbours, no sheet vertex to the next, and the parts stay in place
// (bar what a collapse or two takes). An explicit 4 mm tolerance does weld
// them -- their triangles weld shut and leave the result -- and world_error
// still reports exactly what that lost: it is the sampled Hausdorff distance
// to the input either way, and every rung's error in a pyramid is at least
// that.
TEST(GeometrySimplify, FinePartsInALargeSceneKeepTheirOwnScale) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 60, 5.0);
  for (auto &q : g.points()) {
    q[0] *= 50.0;
    q[1] *= 50.0;
  }
  g.normals().assign(g.num_points(), {0.0, 0.0, 1.0});
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j) {
      const double x = 1000.0 + 3e-3 * i, y = 1000.0 + 3e-3 * j, z = 20.0, s = 2e-3;
      const std::uint64_t o = g.num_points();
      g.points().insert(g.points().end(), {{x, y, z}, {x + s, y, z}, {x, y + s, z}, {x, y, z + s}});
      g.normals().insert(g.normals().end(), 4, {0.0, 0.0, 1.0});
      g.tris().insert(
          g.tris().end(),
          {{o, o + 2, o + 1}, {o, o + 1, o + 3}, {o, o + 3, o + 2}, {o + 1, o + 2, o + 3}});
    }
  const geometry sheet = make_grid(ctx, 20, 0.5);
  for (const auto &t : sheet.const_tris()) {
    const std::uint64_t o = g.num_points();
    v3 c[3];
    for (int k = 0; k < 3; ++k) {
      const auto &q = sheet.const_points()[t[k]];
      c[k] = {1100.0 + 1e-3 * q[0], 1000.0 + 1e-3 * q[1], 20.0 + 1e-3 * q[2]};
      g.points().push_back({c[k][0], c[k][1], c[k][2]});
    }
    const v3 nrm = vcross(vsub(c[1], c[0]), vsub(c[2], c[0]));
    const double l = std::sqrt(vdot(nrm, nrm));
    g.normals().insert(g.normals().end(), 3, {nrm[0] / l, nrm[1] / l, nrm[2] / l});
    g.tris().push_back({o, o + 1, o + 2});
  }
  g.set_geometry_type(geometry::SURFACE_TRI);
  const int tets = 400, flat = int(sheet.num_tris());
  ASSERT_EQ(tris_above(g, 10.0, 1050.0), std::make_pair(tets, flat));

  cvc::simplify_params p;
  p.target_tris = g.num_tris() - 2; // a collapse or two
  cvc::simplify_result r;
  const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
  ASSERT_LE(r.collapses, 2u);
  const std::pair<int, int> kept = tris_above(s, 10.0, 1050.0);
  EXPECT_GE(kept.first, tets - 4);
  EXPECT_GE(kept.second, flat - 4);
  EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));

  cvc::lod::pyramid_params pp;
  const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(g, pp, &ctx.computePool());
  ASSERT_GE(pyr.rungs.size(), 3u);
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    EXPECT_GE(pyr.world_error_m[k], cvc::sampled_hausdorff(g, pyr.rungs[k])) << "rung " << k;

  p.seam_epsilon = 4e-3;
  const geometry w = cvc::simplify(g, p, &r, &ctx.computePool());
  EXPECT_EQ(tris_above(w, 10.0, 1050.0), std::make_pair(0, 0));
  EXPECT_EQ(r.collapses, 0u); // welding them shut already met the target
  EXPECT_GT(r.world_error, 10.0);
  EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, w));
}

// A position two parts share exactly (their vertices there differ in color, so
// the parts stay two) holds a vertex of each, so a vertex within tolerance of
// it welds to it through either part -- even one of the same part as one of
// them, which could never weld to that part's own vertex there. Here part B
// folds back so that its vertex h lies 1e-4 from its corner g, where part A's
// corner also sits: h welds to A's corner, and so to both, whichever part
// lists its vertex at g last.
TEST(GeometrySimplify, APositionTwoPartsShareWeldsToBoth) {
  cvc::app ctx;
  for (bool a_first : {true, false}) {
    geometry g(ctx);
    const geometry::point_t pa[3] = {{0, 0, 0}, {-2, 1, 0}, {-2, -1, 0}};
    const geometry::point_t pb[4] = {{0, 0, 0}, {2, 1, 0}, {2, -1, 0}, {1e-4, 0, 0}};
    auto add_a = [&]() {
      const std::uint64_t o = g.num_points();
      g.points().insert(g.points().end(), pa, pa + 3);
      g.colors().insert(g.colors().end(), 3, {1.0, 0.0, 0.0});
      g.tris().push_back({o, o + 1, o + 2});
    };
    auto add_b = [&]() {
      const std::uint64_t o = g.num_points();
      g.points().insert(g.points().end(), pb, pb + 4);
      g.colors().insert(g.colors().end(), 4, {0.0, 1.0, 0.0});
      g.tris().push_back({o, o + 1, o + 2});
      g.tris().push_back({o + 1, o + 3, o + 2});
    };
    if (a_first) {
      add_a();
      add_b();
    } else {
      add_b();
      add_a();
    }
    g.set_geometry_type(geometry::SURFACE_TRI);
    cvc::simplify_params p;
    p.target_tris = 1;
    p.seam_epsilon = 1e-3;
    cvc::simplify_result r;
    cvc::simplify(g, p, &r);
    EXPECT_EQ(r.seam_vertices, 3u) << "A first " << a_first; // A's g, B's g and h
  }
}

// 300 copies of a unit triangle whose corners are jittered by up to 1e-5, and
// among their first corners 300 tiny triangles jittered as much -- at the
// origin, where the tolerance's float32 floor is nil: the tiny parts'
// tolerance (1e-3 of their own ~1e-5 edges) keeps them apart, and keeps the
// big parts' corners from being joined a whole cluster at a time -- so every
// pair of big corners there is found on its own, ~45000 pairs, far more than
// the search holds at once (it keeps a spanning set of them). The big parts
// still weld into one triangle, the tiny ones stay apart, and a pooled run
// matches a serial one bit for bit.
TEST(GeometrySimplify, NearCoincidentPileOfMixedScalesWelds) {
  cvc::app ctx;
  std::mt19937_64 rng(11);
  auto jitter = [&]() { return 1e-5 * double(rng() >> 11) / 9007199254740992.0; };
  geometry g = make_grid(ctx, 20, 0.5);
  for (auto &q : g.points())
    q[2] += 3.0; // clear of the pile
  const double C[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0.5}};
  for (int i = 0; i < 600; ++i) {
    const bool tiny = i % 2 == 1;
    const std::uint64_t o = g.points().size();
    for (int c = 0; c < 3; ++c) {
      const double *b = C[tiny ? 0 : c];
      g.points().push_back({b[0] + jitter(), b[1] + jitter(), b[2] + jitter()});
    }
    g.tris().push_back({o, o + 1, o + 2});
  }
  cvc::simplify_params p;
  p.target_ratio = 0.5;
  cvc::simplify_result rs, rp;
  const geometry serial = cvc::simplify(g, p, &rs);
  const geometry pooled = cvc::simplify(g, p, &rp, &ctx.computePool());
  EXPECT_EQ(rs.seam_vertices, 900u); // the big copies' corners; no tiny one
  EXPECT_EQ(rp.seam_vertices, rs.seam_vertices);
  // 2200 positions searched (the grid's 400 and the pile's 1800): a block's
  // pairs pass the search's first budget (4 per position), and it trims them
  EXPECT_GT(rs.weld_pairs_held, 4u * 1024u);
  EXPECT_LE(rs.weld_pairs_held, 4u * 1024u + 2200u);
  EXPECT_EQ(rp.weld_pairs_held, rs.weld_pairs_held);
  EXPECT_TRUE(bitwise_equal(serial.const_points(), pooled.const_points()));
  EXPECT_TRUE(bitwise_equal(serial.const_tris(), pooled.const_tris()));
  EXPECT_EQ(0, std::memcmp(&rs.world_error, &rp.world_error, sizeof(double)));
}

// Two 60 x 60 sheets meet along a column, and the second sheet's copy of the
// column is moved 1e-9 along it: every one of its 60 positions is found by the
// near-coincidence search among the sheets' 7200, and the join stays closed as
// the sheets decimate -- every join row one side keeps, the other keeps too.
TEST(GeometrySimplify, NearCoincidentJoinStaysClosed) {
  cvc::app ctx;
  const int n = 60;
  const geometry a = make_grid(ctx, n, 0.4);
  geometry g = a;
  const std::uint64_t o = g.num_points();
  for (auto q : a.const_points()) {
    q[0] += n - 1;
    if (q[0] == n - 1)
      q[1] += 1e-9;
    g.points().push_back(q);
  }
  for (const auto &t : a.const_tris())
    g.tris().push_back({t[0] + o, t[1] + o, t[2] + o});
  for (auto &q : g.points())
    if (q[0] == n - 1)
      q[2] = 0.0; // the sheets agree along the join
  cvc::simplify_params p;
  p.target_ratio = 0.1;
  cvc::simplify_result r;
  const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
  EXPECT_EQ(r.seam_vertices, 2u * n);
  expect_wellformed(s);
  // join rows each side keeps, told apart by the jitter
  std::set<double> left, right;
  for (const auto &q : s.const_points())
    if (q[0] == n - 1)
      (q[1] == std::round(q[1]) ? left : right).insert(std::round(q[1]));
  EXPECT_LT(left.size(), std::size_t(n)) << "the join was decimated, not frozen";
  EXPECT_EQ(left, right);
  EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
}

namespace {

// Two n x n sheets of edge h from (X, X), the second continuing the first
// past the column x = X + h (n - 1) that both list a copy of, with a bump that
// agrees along it. `right(i, j, p)` may move the second sheet's point p (its
// column i, row j) before it is stored. `ground` > 0 adds a 2-triangle ground
// that wide, 5 below the sheets: it widens the scene's extent.
geometry joined_sheets(cvc::app &ctx, int n, double X, double h, double ground,
                       const std::function<void(int, int, geometry::point_t &)> &right) {
  geometry g(ctx);
  for (int s = 0; s < 2; ++s) {
    const std::uint64_t o = g.num_points();
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) {
        const int c = s == 0 ? i : n - 1 + i;
        geometry::point_t p = {X + h * c, X + h * j,
                               0.3 * h * std::sin(0.7 * c) * std::cos(0.9 * j)};
        if (s == 1)
          right(i, j, p);
        g.points().push_back(p);
      }
    for (int i = 0; i + 1 < n; ++i)
      for (int j = 0; j + 1 < n; ++j) {
        const std::uint64_t a = o + std::uint64_t(i * n + j), b = a + std::uint64_t(n);
        g.tris().push_back({a, b, a + 1});
        g.tris().push_back({b, b + 1, a + 1});
      }
  }
  if (ground > 0.0) {
    const std::uint64_t o = g.num_points();
    g.points().insert(g.points().end(),
                      {{0, 0, -5}, {ground, 0, -5}, {ground, ground, -5}, {0, ground, -5}});
    g.tris().insert(g.tris().end(), {{o, o + 1, o + 2}, {o, o + 2, o + 3}});
  }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

// The rows of joined_sheets' join column the first sheet keeps, and the rows
// one sheet keeps and the other does not (a crack). The first sheet's copy of
// the column sits exactly on it; the second's, moved off it by less than
// `off`, is told apart by that.
std::pair<int, int> join_rows(const geometry &g, int n, double X, double h, double off) {
  const double xj = X + h * (n - 1);
  std::set<long> first, second;
  for (const auto &q : g.const_points()) {
    if (std::fabs(q[0] - xj) > off || q[1] < X - off || q[1] > X + h * (n - 1) + off)
      continue;
    const long row = std::lround((q[1] - X) / h);
    (q[0] == xj && q[1] == X + h * double(row) ? first : second).insert(row);
  }
  std::vector<long> crack;
  std::set_symmetric_difference(first.begin(), first.end(), second.begin(), second.end(),
                                std::back_inserter(crack));
  return std::make_pair(int(first.size()), int(crack.size()));
}

} // namespace

// A join far from the origin whose two sides were rounded apart: the second
// sheet is stored at float32 precision 2.5 km out, so its copy of the join
// column is off the first's by up to ~1.1e-4 -- over 2e-3 of the 5 cm edge,
// past the 1e-3 local cap -- yet within 4 float32 epsilons of the coordinates
// (~1.2e-3), the tolerance's floor there. (A ground 3 km wide lifts the
// scene-wide cap, 1e-6 of the extent, above that floor.) The join welds, and
// stays closed as the sheets decimate.
TEST(GeometrySimplify, JoinRoundedApartFarFromTheOriginStaysClosed) {
  cvc::app ctx;
  const int n = 40;
  const double X = 2500.0, h = 0.05;
  double moved = 0.0; // the farthest a join point was rounded off the first sheet's copy
  const geometry g = joined_sheets(ctx, n, X, h, 3000.0, [&](int i, int, geometry::point_t &p) {
    double d2 = 0.0;
    for (int k = 0; k < 3; ++k) {
      const double f = double(float(p[k]));
      d2 += (f - p[k]) * (f - p[k]);
      p[k] = f;
    }
    if (i == 0)
      moved = std::max(moved, std::sqrt(d2));
  });
  ASSERT_GT(moved, 2e-3 * h);
  cvc::simplify_params p;
  p.target_ratio = 0.02;
  cvc::simplify_result r;
  const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
  EXPECT_EQ(r.seam_vertices, 2u * n);
  expect_wellformed(s);
  const std::pair<int, int> rows = join_rows(s, n, X, h, 1e-3);
  EXPECT_LT(rows.first, n) << "the join was decimated, not frozen";
  EXPECT_EQ(rows.second, 0) << "join rows only one side keeps";
}

// A join vertex's tolerance is set by its median edge, not its shortest one,
// and zero-length edges do not count. The second sheet's copy of the join
// column is moved 2e-5 off the first's: within 1e-3 of the 0.5 m edges around
// it, beyond the float32 floor this near the origin. Its first column of cells
// is a 1 mm sliver -- 1e-3 of that would not reach -- or the first sheet's
// join vertices each carry a point (a triangle with all three corners there,
// whose edges have no length). Either way the join welds, and stays closed as
// the sheets decimate.
TEST(GeometrySimplify, JoinVertexWithASliverOrAPointStaysClosed) {
  cvc::app ctx;
  const int n = 40;
  const double h = 0.5, d = 2e-5, xj = h * (n - 1);
  for (int kind = 0; kind < 2; ++kind) {
    SCOPED_TRACE(kind == 0 ? "sliver" : "point");
    geometry g = joined_sheets(ctx, n, 0.0, h, 0.0, [&](int i, int j, geometry::point_t &p) {
      if (i == 1 && kind == 0) // the sliver column, level with the join
        p = {xj + 1e-3, h * j, 0.3 * h * std::sin(0.7 * (n - 1)) * std::cos(0.9 * j)};
      if (i == 0) {
        p[0] += j % 2 ? d : -d;
        p[1] += 0.5 * d;
      }
    });
    if (kind == 1)
      for (int j = 0; j < n; ++j) {
        const std::uint64_t v = std::uint64_t((n - 1) * n + j), o = g.num_points();
        g.points().insert(g.points().end(), 2, g.const_points()[v]);
        g.tris().push_back({v, o, o + 1});
      }
    cvc::simplify_params p;
    p.target_ratio = 0.2;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
    EXPECT_EQ(r.seam_vertices, (kind == 0 ? 2u : 4u) * n); // a point's corners too
    expect_wellformed(s);
    const std::pair<int, int> rows = join_rows(s, n, 0.0, h, 4.0 * d);
    EXPECT_LT(rows.first, n) << "the join was decimated, not frozen";
    EXPECT_EQ(rows.second, 0) << "join rows only one side keeps";
  }
}

// --- welding split faces, and attribute seams --------------------------------

namespace {

// Every point is used by a triangle: a result carries no orphaned vertices.
void expect_all_referenced(const geometry &g) {
  std::vector<char> used(g.num_points(), 0);
  for (const auto &t : g.const_tris())
    used[t[0]] = used[t[1]] = used[t[2]] = 1;
  EXPECT_EQ(std::count(used.begin(), used.end(), 0), 0) << "points no triangle uses";
}

// uv = (x, y) of each position: a planar-projected texture.
geometry with_planar_uv(geometry g) {
  g.uvs().clear();
  for (const auto &p : g.const_points())
    g.uvs().push_back({p[0], p[1]});
  return g;
}

// Triangle corners whose uv is not exactly their position's (x, y): a
// planar-projected texture sheared off the surface it was projected onto.
int sheared_uv_corners(const geometry &g) {
  int bad = 0;
  for (const auto &t : g.const_tris())
    for (int k = 0; k < 3; ++k) {
      const auto &p = g.const_points()[t[k]];
      const auto &uv = g.const_uvs()[t[k]];
      bad += uv[0] != p[0] || uv[1] != p[1] ? 1 : 0;
    }
  return bad;
}

// make_grid(n, amp) with uv = (index, 0) -- which traces any vertex back to
// its source -- and painted normals.
geometry painted_grid(cvc::app &ctx, int n, double amp) {
  geometry g = make_grid(ctx, n, amp);
  for (std::size_t i = 0; i < g.num_points(); ++i)
    g.uvs().push_back({double(i), 0.0});
  paint_normals(g);
  return g;
}

// `g` with every run of `group` consecutive triangles given its own copies of
// their vertices -- make_grid's quad pairs (group 2) are then a flat-shaded
// export, single triangles (group 1) a triangle soup. Each copy carries its
// source vertex's uv, color and normal, so the attributes agree across every
// split. `in_place`: a vertex's first copy keeps its index and later copies are
// appended; otherwise the copies are numbered afresh, in the order the
// triangles meet them.
geometry split_faces(const geometry &g, std::size_t group, bool in_place) {
  geometry s(g.ctx());
  const bool uv = g.const_uvs().size() == g.num_points(),
             col = g.const_colors().size() == g.num_points(),
             nrm = g.const_normals().size() == g.num_points();
  auto copy_of = [&](std::uint64_t v) {
    s.points().push_back(g.const_points()[v]);
    if (uv)
      s.uvs().push_back(g.const_uvs()[v]);
    if (col)
      s.colors().push_back(g.const_colors()[v]);
    if (nrm)
      s.normals().push_back(g.const_normals()[v]);
    return std::uint64_t(s.num_points() - 1);
  };
  std::vector<char> claimed(g.num_points(), 0);
  if (in_place)
    for (std::uint64_t v = 0; v < g.num_points(); ++v)
      copy_of(v);
  const auto &T = g.const_tris();
  for (std::size_t f = 0; f < T.size(); f += group) {
    std::map<std::uint64_t, std::uint64_t> copy;
    for (std::size_t t = f; t < std::min(T.size(), f + group); ++t) {
      std::array<std::uint64_t, 3> c;
      for (int k = 0; k < 3; ++k) {
        const std::uint64_t v = T[t][k];
        auto it = copy.find(v);
        if (it == copy.end()) {
          const bool mine = in_place && !claimed[v];
          claimed[v] = 1;
          it = copy.insert(std::make_pair(v, mine ? v : copy_of(v))).first;
        }
        c[k] = it->second;
      }
      s.tris().push_back({c[0], c[1], c[2]});
    }
  }
  s.set_geometry_type(geometry::SURFACE_TRI);
  return s;
}

// Cracks in a make_grid(n) sheet: edges, keyed by their endpoints' POSITIONS,
// that bound a single triangle yet do not lie on the sheet's outer border.
int interior_cracks(const geometry &g, int n) {
  const auto &P = g.const_points();
  auto key = [&](std::uint64_t v) { return std::make_pair(P[v][0], P[v][1]); };
  std::map<std::pair<std::pair<double, double>, std::pair<double, double>>, int> count;
  for (const auto &t : g.const_tris())
    for (int k = 0; k < 3; ++k) {
      auto a = key(t[k]), b = key(t[(k + 1) % 3]);
      ++count[std::make_pair(std::min(a, b), std::max(a, b))];
    }
  const double hi = double(n - 1);
  int cracks = 0;
  for (const auto &e : count) {
    const auto &a = e.first.first, &b = e.first.second;
    const bool border = (a.first == b.first && (a.first == 0.0 || a.first == hi)) ||
                        (a.second == b.second && (a.second == 0.0 || a.second == hi));
    cracks += e.second == 1 && !border ? 1 : 0;
  }
  return cracks;
}

// make_grid(n, amp) with uv = xy, cut into R x R blocks of quads: each block
// owns its vertices and colors them (block i, block j, 0). The block borders
// are attribute seams -- the color jumps there, the uv does not -- and every
// vertex inside a block welds to its neighbours' copies.
geometry blocks(cvc::app &ctx, int n, double amp, int R) {
  const geometry g = make_grid(ctx, n, amp);
  geometry s(ctx);
  std::map<std::pair<int, std::uint64_t>, std::uint64_t> copy;
  const auto &T = g.const_tris();
  for (std::size_t t = 0; t < T.size(); ++t) {
    const int q = int(t / 2), bi = (q / (n - 1)) / R, bj = (q % (n - 1)) / R;
    std::array<std::uint64_t, 3> c;
    for (int k = 0; k < 3; ++k) {
      const auto key = std::make_pair(bi * 1000 + bj, T[t][k]);
      auto it = copy.find(key);
      if (it == copy.end()) {
        const auto &p = g.const_points()[T[t][k]];
        s.points().push_back(p);
        s.uvs().push_back({p[0], p[1]});
        s.colors().push_back({double(bi), double(bj), 0.0});
        it = copy.insert(std::make_pair(key, std::uint64_t(s.num_points() - 1))).first;
      }
      c[k] = it->second;
    }
    s.tris().push_back({c[0], c[1], c[2]});
  }
  s.set_geometry_type(geometry::SURFACE_TRI);
  return s;
}

// Faces of a blocks() mesh that mix blocks, or reach outside their own block.
int faces_off_their_block(const geometry &g, int R) {
  const auto &P = g.const_points();
  const auto &C = g.const_colors();
  int bad = 0;
  for (const auto &t : g.const_tris()) {
    bool off = C[t[1]] != C[t[0]] || C[t[2]] != C[t[0]];
    const double x0 = C[t[0]][0] * R, y0 = C[t[0]][1] * R;
    for (int k = 0; k < 3; ++k)
      off = off || P[t[k]][0] < x0 || P[t[k]][0] > x0 + R || P[t[k]][1] < y0 || P[t[k]][1] > y0 + R;
    bad += off ? 1 : 0;
  }
  return bad;
}

} // namespace

// Input vertices bit-identical in position and in every carried attribute are
// one vertex: a sheet split apart per quad, or all the way down to a triangle
// soup, whose uv and normals agree across every split decimates EXACTLY as the
// welded sheet does -- the same points, triangles, uvs, normals and error, bit
// for bit.
TEST(GeometrySimplify, SplitsWhoseAttributesAgreeWeldExactly) {
  cvc::app ctx;
  const geometry sheet = painted_grid(ctx, 24, 1.5);
  cvc::simplify_params p;
  p.target_ratio = 0.2;
  cvc::simplify_result rw;
  const geometry welded = cvc::simplify(sheet, p, &rw);
  ASSERT_GT(rw.world_error, 0.0);
  EXPECT_EQ(rw.seam_vertices, 0u);
  for (std::size_t group : {2u, 1u}) {
    SCOPED_TRACE(testing::Message() << "group " << group);
    const geometry g = split_faces(sheet, group, true);
    ASSERT_GT(g.num_points(), sheet.num_points());
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
    EXPECT_TRUE(bitwise_equal(s.const_points(), welded.const_points()));
    EXPECT_TRUE(bitwise_equal(s.const_tris(), welded.const_tris()));
    EXPECT_TRUE(bitwise_equal(s.const_uvs(), welded.const_uvs()));
    EXPECT_TRUE(bitwise_equal(s.const_normals(), welded.const_normals()));
    EXPECT_EQ(r.out_tris, rw.out_tris);
    EXPECT_EQ(0, std::memcmp(&r.world_error, &rw.world_error, sizeof(double)));
    // every copy is welded to another but those at the sheet's corners that
    // only one part touches: 4 per-quad, 2 in the soup (the diagonal's ends)
    EXPECT_EQ(r.seam_vertices, g.num_points() - (group == 2 ? 4u : 2u));
  }
}

// The same splits numbered afresh (the order an exporter writes a soup in):
// they decimate to the target without cracks, every result vertex is used and
// carries its source's uv and normal bit-for-bit, and world_error is the
// Hausdorff distance to the split input. Without welding they fall apart.
TEST(GeometrySimplify, WeldDecimatesSplitFacesWithoutCracks) {
  cvc::app ctx;
  const int n = 24;
  const geometry sheet = painted_grid(ctx, n, 1.5);
  for (std::size_t group : {2u, 1u}) {
    const geometry g = split_faces(sheet, group, false);
    SCOPED_TRACE(testing::Message() << "group " << group);
    ASSERT_EQ(interior_cracks(g, n), 0);
    cvc::simplify_params p;
    p.target_ratio = 0.2;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
    expect_wellformed(s);
    expect_all_referenced(s);
    const std::uint64_t target = std::uint64_t(std::llround(0.2 * double(g.num_tris())));
    EXPECT_LE(r.out_tris, target);
    EXPECT_GE(r.out_tris + 2, target); // an interior collapse removes two triangles
    EXPECT_EQ(r.seam_vertices, g.num_points() - (group == 2 ? 4u : 2u));
    EXPECT_EQ(interior_cracks(s, n), 0);
    EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
    for (std::size_t i = 0; i < s.num_points(); ++i) {
      const std::size_t from = std::size_t(s.const_uvs()[i][0]);
      EXPECT_EQ(0, std::memcmp(&s.const_normals()[i], &sheet.const_normals()[from],
                               sizeof(sheet.const_normals()[from])));
      EXPECT_EQ(0, std::memcmp(&s.const_points()[i], &sheet.const_points()[from],
                               sizeof(sheet.const_points()[from])));
    }
    // The same split mesh without welding: every part decimates on its own and
    // the sheet tears.
    p.weld_seams = false;
    const geometry torn = cvc::simplify(g, p);
    EXPECT_GT(interior_cracks(torn, n), 0);
  }
}

// A planar-projected texture (uv = xy) is continuous across every split, so a
// triangle soup and a flat-shaded (split per quad) sheet decimate as the welded
// sheet, and every result corner keeps uv == xy exactly: no uv is ever
// re-attached to another position, so the texture cannot shear.
TEST(GeometrySimplify, PlanarUvIsNeverSheared) {
  cvc::app ctx;
  const geometry sheet = with_planar_uv(make_grid(ctx, 30, 1.5));
  for (std::size_t group : {1u, 2u}) {
    const geometry g = split_faces(sheet, group, false);
    ASSERT_EQ(sheared_uv_corners(g), 0);
    for (double ratio : {0.5, 0.2, 0.05}) {
      SCOPED_TRACE(testing::Message() << "group " << group << " ratio " << ratio);
      cvc::simplify_params p;
      p.target_ratio = ratio;
      cvc::simplify_result r;
      const geometry s = cvc::simplify(g, p, &r);
      EXPECT_LE(r.out_tris, std::uint64_t(std::llround(ratio * double(g.num_tris()))));
      EXPECT_EQ(sheared_uv_corners(s), 0);
      expect_all_referenced(s);
    }
  }
}

// Genuinely different attributes stay put. Blocks of a sheet that each own
// their vertices and color decimate inside themselves and along their borders
// -- the seams thin -- yet no face mixes two blocks or reaches outside its own,
// no crack opens along a seam, and uv stays exactly xy. Faces stay in their
// blocks without the border and seam constraint planes too (preserve_boundary
// off), where only the wedge map guards the seams (the sheet's open border then
// erodes, so cracks are not counted there; see SeamsOfAClosedSurfaceNeverOpen).
TEST(GeometrySimplify, AttributeSeamsCollapseOnlyAlongThemselves) {
  cvc::app ctx;
  const int n = 41, R = 8;
  for (int variant = 0; variant < 3; ++variant) {
    const double amp = variant == 0 ? 0.0 : 0.5;
    const bool constrained = variant < 2;
    const geometry g = blocks(ctx, n, amp, R);
    ASSERT_EQ(faces_off_their_block(g, R), 0);
    // positions on the block borders (inside the sheet)
    auto on_seams = [&](const geometry &m) {
      std::set<std::pair<double, double>> at;
      for (const auto &q : m.const_points())
        if ((std::fmod(q[0], R) == 0.0 || std::fmod(q[1], R) == 0.0) && q[0] > 0 && q[1] > 0 &&
            q[0] < n - 1 && q[1] < n - 1)
          at.insert(std::make_pair(q[0], q[1]));
      return at.size();
    };
    for (double ratio : {0.2, 0.05}) {
      SCOPED_TRACE(testing::Message()
                   << "amp " << amp << " constrained " << constrained << " ratio " << ratio);
      cvc::simplify_params p;
      p.target_ratio = ratio;
      p.preserve_boundary = constrained;
      cvc::simplify_result r;
      const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
      expect_wellformed(s);
      expect_all_referenced(s);
      EXPECT_LE(r.out_tris, std::uint64_t(std::llround(ratio * double(g.num_tris()))));
      EXPECT_EQ(faces_off_their_block(s, R), 0);
      if (constrained)
        EXPECT_EQ(interior_cracks(s, n), 0);
      EXPECT_EQ(sheared_uv_corners(s), 0);
      EXPECT_LT(on_seams(s), on_seams(g) / 2) << "the seams were decimated, not frozen";
      EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
    }
  }
}

// A hard-edge crease that does NOT sever the mesh: the vertices of one grid
// column are split for the first rows only, the faces left of the crease use
// the copies, and the copies carry other normals. The crease is a seam inside
// ONE connected component, and it ends inside the sheet. It stays closed and
// keeps its sides' normals apart as the sheet decimates, with or without the
// constraint planes, and also when each copy's z is the other signed zero of
// its original's (-0 and +0 are one position). Split further into a triangle
// soup, it decimates exactly as it does unsplit. And when the copies'
// attributes agree with their originals, they weld away and the result is the
// uncreased sheet's, bit for bit.
TEST(GeometrySimplify, CreaseInsideOneComponentStaysClosed) {
  cvc::app ctx;
  const int n = 30, m = 15, rows = 20; // the crease: column m, rows [0, rows)
  struct Case {
    double amp;
    bool constrained, agree, flip_zero;
  };
  for (const Case &c : {Case{1.2, true, false, false}, Case{0.0, true, false, false},
                        Case{0.0, false, false, false}, Case{1.2, true, true, false},
                        Case{0.0, true, false, true}}) {
    SCOPED_TRACE(testing::Message() << "amp " << c.amp << " constrained " << c.constrained
                                    << " attributes agree " << c.agree << " -0/+0 " << c.flip_zero);
    const geometry sheet = painted_grid(ctx, n, c.amp);
    geometry g = sheet;
    std::vector<std::int64_t> copy_of(g.num_points(), -1);
    for (int i = 0; i < rows; ++i) {
      const std::uint64_t v = std::uint64_t(m * n + i); // make_grid: index i * n + j at (i, j)
      copy_of[v] = std::int64_t(g.num_points());
      g.points().push_back(g.points()[v]);
      if (c.flip_zero) { // z is a signed zero on the flat sheet
        ASSERT_EQ(g.points().back()[2], 0.0);
        g.points().back()[2] = -g.points().back()[2];
      }
      g.uvs().push_back(g.uvs()[v]);
      auto nv = g.normals()[v];
      if (!c.agree)
        nv = {-nv[0], -nv[1], -nv[2]};
      g.normals().push_back(nv);
    }
    for (auto &t : g.tris()) {
      const double cx = (g.points()[t[0]][0] + g.points()[t[1]][0] + g.points()[t[2]][0]) / 3.0;
      if (cx < m)
        for (int k = 0; k < 3; ++k)
          if (t[k] < sheet.num_points() && copy_of[t[k]] >= 0)
            t[k] = std::uint64_t(copy_of[t[k]]);
    }
    ASSERT_EQ(interior_cracks(g, n), 0);
    cvc::simplify_params p;
    p.target_ratio = 0.15;
    p.preserve_boundary = c.constrained;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    expect_wellformed(s);
    expect_all_referenced(s);
    EXPECT_EQ(r.seam_vertices, 2u * rows);
    if (c.constrained) // (unconstrained, the sheet's open border erodes)
      EXPECT_EQ(interior_cracks(s, n), 0);
    // The same mesh as a triangle soup (each vertex's first copy keeping its
    // index) is that mesh: its copies weld back exactly, crease and all.
    const geometry soup = cvc::simplify(split_faces(g, 1, true), p);
    EXPECT_TRUE(bitwise_equal(soup.const_points(), s.const_points()));
    EXPECT_TRUE(bitwise_equal(soup.const_tris(), s.const_tris()));
    EXPECT_TRUE(bitwise_equal(soup.const_uvs(), s.const_uvs()));
    EXPECT_TRUE(bitwise_equal(soup.const_normals(), s.const_normals()));
    if (c.agree) {
      const geometry plain = cvc::simplify(sheet, p);
      EXPECT_TRUE(bitwise_equal(s.const_points(), plain.const_points()));
      EXPECT_TRUE(bitwise_equal(s.const_tris(), plain.const_tris()));
      EXPECT_TRUE(bitwise_equal(s.const_normals(), plain.const_normals()));
      continue;
    }
    // Every corner on the crease carries the normal of its face's side: the
    // copy's (flipped) left of the crease, the original's right of it. (A face
    // with corners on both sides -- past the crease's end -- or on the crease
    // line alone has no side.)
    int wrong = 0, crease = 0;
    const auto &P = s.const_points();
    for (const auto &t : s.const_tris()) {
      bool left = false, right = false;
      for (int k = 0; k < 3; ++k) {
        left = left || P[t[k]][0] < m;
        right = right || P[t[k]][0] > m;
      }
      if (left == right)
        continue;
      for (int k = 0; k < 3; ++k) {
        const std::size_t from = std::size_t(s.const_uvs()[t[k]][0]);
        if (P[t[k]][0] != m || copy_of[from] < 0)
          continue; // not a crease vertex
        ++crease;
        auto want = sheet.const_normals()[from];
        if (left)
          want = {-want[0], -want[1], -want[2]};
        wrong += std::memcmp(&s.const_normals()[t[k]], &want, sizeof(want)) != 0 ? 1 : 0;
      }
    }
    EXPECT_GT(crease, 0);
    EXPECT_EQ(wrong, 0);
    // and the crease was decimated along itself, not frozen
    std::set<double> crease_rows;
    for (const auto &q : P)
      if (q[0] == m && q[1] < rows)
        crease_rows.insert(q[1]);
    EXPECT_LT(crease_rows.size(), std::size_t(rows));
  }
}

// A closed bumpy box whose six faces each own their vertices, color and a
// planar-projected uv (the twelve box edges are seams where both jump), and
// whose faces are further split into a soup: no seam may open anywhere -- every
// edge of the result keeps a triangle on each side -- no face takes another
// face's color, every uv stays the projection of its own position, and all of
// it holds with the constraint planes off, where only the wedge map guards the
// seams.
TEST(GeometrySimplify, SeamsOfAClosedSurfaceNeverOpen) {
  cvc::app ctx;
  const int n = 8;
  geometry box(ctx);
  for (int ax = 0; ax < 3; ++ax)
    for (int side = 0; side < 2; ++side) {
      const int face = 2 * ax + side;
      const std::uint64_t o = box.num_points();
      for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j) {
          const double u = double(i) / n, v = double(j) / n;
          const double bump = (i > 0 && i < n && j > 0 && j < n)
                                  ? 0.1 * std::sin(7 * u + ax) * std::cos(5 * v + side)
                                  : 0.0;
          geometry::point_t q;
          q[ax] = side ? 1.0 + bump : -bump;
          q[(ax + 1) % 3] = u;
          q[(ax + 2) % 3] = v;
          box.points().push_back(q);
          box.colors().push_back({double(face), 0.0, 0.0});
          box.uvs().push_back({u, v});
        }
      auto id = [&](int i, int j) { return o + std::uint64_t(i * (n + 1) + j); };
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
          if (side == 0) {
            box.tris().push_back({id(i, j), id(i, j + 1), id(i + 1, j)});
            box.tris().push_back({id(i + 1, j), id(i, j + 1), id(i + 1, j + 1)});
          } else {
            box.tris().push_back({id(i, j), id(i + 1, j), id(i, j + 1)});
            box.tris().push_back({id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)});
          }
        }
    }
  box.set_geometry_type(geometry::SURFACE_TRI);
  const geometry g = split_faces(box, 1, false);
  // edges keyed by position with a triangle count, and faces whose uv is not
  // the projection of their positions onto their own face's plane
  auto open_edges = [](const geometry &m) {
    std::map<std::pair<std::array<double, 3>, std::array<double, 3>>, int> count;
    const auto &P = m.const_points();
    for (const auto &t : m.const_tris())
      for (int k = 0; k < 3; ++k) {
        std::array<double, 3> a = {P[t[k]][0], P[t[k]][1], P[t[k]][2]};
        std::array<double, 3> b = {P[t[(k + 1) % 3]][0], P[t[(k + 1) % 3]][1],
                                   P[t[(k + 1) % 3]][2]};
        ++count[std::make_pair(std::min(a, b), std::max(a, b))];
      }
    int open = 0;
    for (const auto &e : count)
      open += e.second == 1 ? 1 : 0;
    return open;
  };
  auto off_face = [](const geometry &m) {
    int bad = 0;
    for (const auto &t : m.const_tris())
      for (int k = 0; k < 3; ++k) {
        const auto &c = m.const_colors()[t[k]];
        const int ax = int(c[0]) / 2;
        const auto &q = m.const_points()[t[k]];
        const auto &uv = m.const_uvs()[t[k]];
        bad += c != m.const_colors()[t[0]] || uv[0] != q[(ax + 1) % 3] || uv[1] != q[(ax + 2) % 3]
                   ? 1
                   : 0;
      }
    return bad;
  };
  ASSERT_EQ(open_edges(g), 0);
  ASSERT_EQ(off_face(g), 0);
  for (bool constrained : {true, false})
    for (double ratio : {0.3, 0.1, 0.03}) {
      SCOPED_TRACE(testing::Message() << "constrained " << constrained << " ratio " << ratio);
      cvc::simplify_params p;
      p.target_ratio = ratio;
      p.preserve_boundary = constrained;
      cvc::simplify_result r;
      const geometry s = cvc::simplify(g, p, &r);
      expect_wellformed(s);
      expect_all_referenced(s);
      EXPECT_LE(r.out_tris, std::max<std::uint64_t>(
                                12, std::uint64_t(std::llround(ratio * double(g.num_tris())))));
      EXPECT_EQ(open_edges(s), 0);
      EXPECT_EQ(off_face(s), 0);
      EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
    }
}

// A seam's triangle sides carry constraint planes, as open borders do. On a
// flat sheet every other collapse is free, yet a round patch that owns its own
// vertices and color keeps its exact area however hard the sheet decimates:
// the seam slides only along itself and its corners hold. (Without the
// constraint the patch shrinks, and at 5% it is gone.)
TEST(GeometrySimplify, SeamConstraintsHoldAColorBoundary) {
  cvc::app ctx;
  const int n = 41;
  const double c = 0.5 * (n - 1), radius = 11.3;
  const geometry sheet = make_grid(ctx, n, 0.0);
  geometry g(ctx);
  std::map<std::pair<int, std::uint64_t>, std::uint64_t> copy;
  const auto &T = sheet.const_tris();
  for (std::size_t t = 0; t < T.size(); ++t) {
    const int q = int(t / 2);
    const int in = std::hypot(q / (n - 1) + 0.5 - c, q % (n - 1) + 0.5 - c) < radius ? 1 : 0;
    std::array<std::uint64_t, 3> v;
    for (int k = 0; k < 3; ++k) {
      auto it = copy.find(std::make_pair(in, T[t][k]));
      if (it == copy.end()) {
        g.points().push_back(sheet.const_points()[T[t][k]]);
        g.colors().push_back({double(in), 0.0, 0.0});
        it = copy.insert(std::make_pair(std::make_pair(in, T[t][k]), g.num_points() - 1)).first;
      }
      v[k] = it->second;
    }
    g.tris().push_back({v[0], v[1], v[2]});
  }
  g.set_geometry_type(geometry::SURFACE_TRI);
  auto patch_area = [](const geometry &m) {
    double a = 0.0;
    for (const auto &t : m.const_tris())
      if (m.const_colors()[t[0]][0] == 1.0) {
        const v3 e = vsub(to3(m.const_points()[t[1]]), to3(m.const_points()[t[0]]));
        const v3 f = vsub(to3(m.const_points()[t[2]]), to3(m.const_points()[t[0]]));
        a += 0.5 * std::sqrt(vdot(vcross(e, f), vcross(e, f)));
      }
    return a;
  };
  const double area = patch_area(g);
  ASSERT_GT(area, 350.0);
  for (double ratio : {0.1, 0.05}) {
    cvc::simplify_params p;
    p.target_ratio = ratio;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    EXPECT_LE(r.out_tris, std::uint64_t(std::llround(ratio * double(g.num_tris()))));
    EXPECT_NEAR(patch_area(s), area, 1e-9 * area) << "ratio " << ratio;
    EXPECT_EQ(r.world_error, 0.0); // a flat sheet stays the flat sheet
  }
}

// An edge is collapsed in the cheaper direction the seams allow: when the
// wedge map forbids dropping one end, dropping the other may still be clean.
// On a sheet cut into small color blocks -- seams everywhere -- that is what
// reaches the target, with or without the constraint planes, while no face
// leaves its block and uv stays exactly xy. (So hard a target erodes the
// sheet's corners; seams that stay closed are AttributeSeamsCollapseOnly-
// AlongThemselves' to check.)
TEST(GeometrySimplify, SeamedBlocksReachTheirTarget) {
  cvc::app ctx;
  const int n = 31, R = 3;
  const geometry g = blocks(ctx, n, 0.5, R);
  struct Case {
    bool constrained;
    double ratio;
  };
  for (const Case &c : {Case{false, 0.1}, Case{false, 0.05}, Case{true, 0.05}}) {
    SCOPED_TRACE(testing::Message() << "constrained " << c.constrained << " ratio " << c.ratio);
    cvc::simplify_params p;
    p.target_ratio = c.ratio;
    p.preserve_boundary = c.constrained;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    expect_wellformed(s);
    EXPECT_LE(r.out_tris, std::uint64_t(std::llround(c.ratio * double(g.num_tris()))));
    EXPECT_EQ(faces_off_their_block(s, R), 0);
    EXPECT_EQ(sheared_uv_corners(s), 0);
  }
}

// A double-sided sheet lists every triangle twice, front and back, over the
// same vertices. Each border side then bounds two coincident triangles, and is
// still an open border, so it is pinned: the flat sheet decimates hard, both
// sides alike, yet keeps its corners and its exact extent (unpinned, the free
// collapses of a flat sheet eat its corners).
TEST(GeometrySimplify, DoubleSidedSheetKeepsItsBorder) {
  cvc::app ctx;
  const geometry one = make_grid(ctx, 20, 0.0);
  geometry g = one;
  for (const auto &t : one.const_tris())
    g.tris().push_back({t[0], t[2], t[1]});
  cvc::simplify_params p;
  p.target_ratio = 0.1;
  cvc::simplify_result r;
  const geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_LE(r.out_tris, std::uint64_t(std::llround(0.1 * double(g.num_tris()))));
  const cvc::bounding_box b0 = g.extents(), b1 = s.extents();
  EXPECT_EQ(b1.minx, b0.minx);
  EXPECT_EQ(b1.maxx, b0.maxx);
  EXPECT_EQ(b1.miny, b0.miny);
  EXPECT_EQ(b1.maxy, b0.maxy);
  EXPECT_EQ(r.world_error, 0.0);
  // every face's back is there too
  std::map<std::array<std::uint64_t, 3>, int> faces;
  for (const auto &t : s.const_tris()) {
    std::array<std::uint64_t, 3> f = {t[0], t[1], t[2]};
    std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
    ++faces[f];
  }
  for (const auto &f : faces) {
    const auto back = faces.find({f.first[0], f.first[2], f.first[1]});
    EXPECT_TRUE(back != faces.end() && back->second == f.second);
  }
}

namespace {

// A closed bumpy sphere cut into regions, from `rng` (its raw output only, so
// the mesh is the same on every platform): its faces go to the nearest of K
// seed vertices (by centroid), each region owns its vertices, a color
// (region, 0, 0) and a uv (3x + region / 4, y) -- so region borders are seams
// where both jump -- and faces are listed in a shuffled order. With `soup`, a
// face owns its vertices outright one time in two; with `salt`, one face in
// ten goes to one of three extra regions.
geometry seamed_sphere(cvc::app &ctx, std::mt19937_64 &rng, bool soup, bool salt) {
  const int nu = 8 + int(rng() % 20), nv = 6 + int(rng() % 16);
  const double bump = double(rng() % 3) * 0.1;
  geometry base(ctx);
  auto &P = base.points();
  P.push_back({0, 0, 1});
  for (int i = 1; i < nv; ++i)
    for (int j = 0; j < nu; ++j) {
      const double th = kPi * i / nv, ph = 2 * kPi * j / nu;
      const double r = 1 + bump * std::sin(5 * th) * std::cos(3 * ph);
      P.push_back(
          {r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)});
    }
  P.push_back({0, 0, -1});
  auto id = [&](int i, int j) { return std::uint64_t(1 + (i - 1) * nu + (j % nu)); };
  const std::uint64_t south = P.size() - 1;
  auto &T = base.tris();
  for (int j = 0; j < nu; ++j)
    T.push_back({0, id(1, j), id(1, j + 1)});
  for (int i = 1; i < nv - 1; ++i)
    for (int j = 0; j < nu; ++j) {
      T.push_back({id(i, j), id(i + 1, j), id(i + 1, j + 1)});
      T.push_back({id(i, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  for (int j = 0; j < nu; ++j)
    T.push_back({id(nv - 1, j), south, id(nv - 1, j + 1)});

  const int K = 1 + int(rng() % 8);
  std::vector<v3> seeds;
  for (int k = 0; k < K; ++k)
    seeds.push_back(to3(P[rng() % P.size()]));
  std::vector<std::size_t> order(T.size());
  std::iota(order.begin(), order.end(), std::size_t(0));
  for (std::size_t i = order.size() - 1; i > 0; --i)
    std::swap(order[i], order[rng() % (i + 1)]);
  geometry g(ctx);
  std::map<std::pair<int, std::uint64_t>, std::uint64_t> copy;
  for (std::size_t f : order) {
    const auto &t = T[f];
    v3 c = {0, 0, 0};
    for (int k = 0; k < 3; ++k)
      for (int a = 0; a < 3; ++a)
        c[a] += P[t[k]][a] / 3.0;
    int reg = 0;
    for (int k = 1; k < K; ++k)
      if (vdot(vsub(c, seeds[k]), vsub(c, seeds[k])) <
          vdot(vsub(c, seeds[reg]), vsub(c, seeds[reg])))
        reg = k;
    if (salt && rng() % 10 == 0)
      reg = K + int(rng() % 3);
    const int key = soup && rng() % 2 ? 1000 + int(f) : reg;
    std::array<std::uint64_t, 3> v;
    for (int k = 0; k < 3; ++k) {
      auto it = copy.find(std::make_pair(key, t[k]));
      if (it == copy.end()) {
        const auto &q = P[t[k]];
        g.points().push_back(q);
        g.colors().push_back({double(reg), 0.0, 0.0});
        g.uvs().push_back({3.0 * q[0] + 0.25 * reg, q[1]});
        it = copy.insert(std::make_pair(std::make_pair(key, t[k]), g.num_points() - 1)).first;
      }
      v[k] = it->second;
    }
    g.tris().push_back({v[0], v[1], v[2]});
  }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

} // namespace

// Collapses keep the link condition over the welded topology: on closed,
// seamed spheres decimated to 60%, 30% and 10% of their triangles -- with and
// without the constraint planes -- no two faces ever fold onto the same three
// positions and no edge gains a third triangle; every edge keeps a triangle
// on each side, no face mixes regions, and every uv stays on its position.
TEST(GeometrySimplify, SeamedSpheresNeverFoldOntoThemselves) {
  cvc::app ctx;
  std::mt19937_64 rng(1000);
  int checked = 0;
  for (int cs = 0; cs < 48; ++cs) {
    const int mode = int(rng() % 3);
    const geometry g = seamed_sphere(ctx, rng, mode == 1, mode == 2);
    const std::uint64_t n = g.num_tris();
    const std::vector<std::uint64_t> targets = {n * 6 / 10, n * 3 / 10, n / 10};
    for (bool constrained : {true, false}) {
      cvc::simplify_params p;
      p.preserve_boundary = constrained;
      std::vector<cvc::simplify_result> res;
      const std::vector<geometry> snaps = cvc::simplify_progressive(g, targets, p, &res);
      for (std::size_t k = 0; k < targets.size(); ++k) {
        SCOPED_TRACE(testing::Message() << "case " << cs << " mode " << mode << " constrained "
                                        << constrained << " target " << targets[k]);
        const geometry &s = snaps[k];
        const auto &P = s.const_points();
        std::map<std::pair<v3, v3>, int> edges;
        std::map<std::array<v3, 3>, int> faces;
        int mixed = 0, sheared = 0;
        for (const auto &t : s.const_tris()) {
          std::array<v3, 3> f = {to3(P[t[0]]), to3(P[t[1]]), to3(P[t[2]])};
          for (int e = 0; e < 3; ++e) {
            ++edges[std::make_pair(std::min(f[e], f[(e + 1) % 3]), std::max(f[e], f[(e + 1) % 3]))];
            const auto &c = s.const_colors()[t[e]];
            const auto &uv = s.const_uvs()[t[e]];
            mixed += c != s.const_colors()[t[0]] ? 1 : 0;
            sheared += uv[0] != 3.0 * P[t[e]][0] + 0.25 * c[0] || uv[1] != P[t[e]][1] ? 1 : 0;
          }
          std::sort(f.begin(), f.end());
          ++faces[f];
        }
        int doubled = 0, unpaired = 0;
        for (const auto &f : faces)
          doubled += f.second > 1 ? 1 : 0;
        for (const auto &e : edges)
          unpaired += e.second != 2 ? 1 : 0;
        EXPECT_EQ(doubled, 0);
        EXPECT_EQ(unpaired, 0);
        EXPECT_EQ(mixed, 0);
        EXPECT_EQ(sheared, 0);
        expect_all_referenced(s);
        ++checked;
      }
    }
  }
  EXPECT_EQ(checked, 48 * 6);
}

// --- locality ----------------------------------------------------------------

// A collapse is costed in its own vertex's frame, with a round-off band made of
// local lengths, so geometry elsewhere -- here one far away triangle that only
// stretches the bounding box -- cannot change how a part decimates: the part's
// result is bit-identical with or without it.
TEST(GeometrySimplify, DistantGeometryDoesNotChangeALocalResult) {
  cvc::app ctx;
  const geometry g = make_grid(ctx, 40, 0.01); // 1 cm bumps on a 39 m sheet
  cvc::simplify_params p;
  p.target_tris = g.num_tris() / 4;
  cvc::simplify_result r;
  const geometry alone = cvc::simplify(g, p, &r);
  ASSERT_GT(r.world_error, 0.0);
  for (double dist : {200.0, 1e4, 1e6}) {
    geometry f = g;
    const std::uint64_t o = f.points().size();
    f.points().push_back({dist, dist, 0.0});
    f.points().push_back({dist + 1.0, dist, 0.0});
    f.points().push_back({dist, dist + 1.0, 0.0});
    f.tris().push_back({o, o + 1, o + 2});
    cvc::simplify_params q = p;
    q.target_tris = p.target_tris + 1; // the far triangle survives
    const geometry both = cvc::simplify(f, q, nullptr, &ctx.computePool());
    SCOPED_TRACE(testing::Message() << "far " << dist);
    ASSERT_EQ(both.num_tris(), alone.num_tris() + 1);
    ASSERT_EQ(both.num_points(), alone.num_points() + 3);
    // The part's vertices and triangles come first; the far triangle is last.
    const geometry::points_t head(both.const_points().begin(),
                                  both.const_points().begin() + alone.num_points());
    const geometry::tris_t tris(both.const_tris().begin(), both.const_tris().end() - 1);
    EXPECT_TRUE(bitwise_equal(head, alone.const_points()));
    EXPECT_TRUE(bitwise_equal(tris, alone.const_tris()));
  }
}

namespace {

geometry transformed(const geometry &g, double s, double tx, double ty, double tz) {
  geometry h = g;
  for (auto &q : h.points()) {
    q[0] = q[0] * s + tx;
    q[1] = q[1] * s + ty;
    q[2] = q[2] * s + tz;
  }
  return h;
}

} // namespace

// Geometric ties stay ties when the mesh is scaled or moved. On a tilted flat
// sheet (every coordinate irrational, so each transform rounds them afresh)
// every interior collapse is free: its cost is pure round-off, which the snap
// band sends to exactly 0, so the index tie-break decides -- identically under
// a power-of-2 scale, a non-dyadic scale, and translations near and far. And a
// bumpy sheet with near-tie costs, on coordinates that the power-of-2 scales
// and dyadic translations below leave exact, costs every collapse in its own
// frame from the same differences, so its order cannot change either. The
// triangles come out bit-identical in every case.
TEST(GeometrySimplify, CollapseOrderSurvivesScaleAndTranslation) {
  cvc::app ctx;
  struct Xf {
    double s, tx, ty, tz;
  };
  const Xf any[] = {{1024, 0, 0, 0}, {0.125, 0, 0, 0},     {7, 0, 0, 0}, {1e-3, 0, 0, 0},
                    {1, 1e3, 0, 0},  {1, 3.3e6, -2e5, 10}, {1, 5, 5, 5}};
  const Xf exact[] = {
      {1024, 0, 0, 0}, {0.125, 0, 0, 0}, {1, 4096, -1024, 64}, {1, 3.5, 7.25, -2.5}};
  for (double tilt : {0.3, 1.1}) {
    geometry g = make_grid(ctx, 31, 0.0);
    const double ca = std::cos(tilt), sa = std::sin(tilt), cb = std::cos(0.7 * tilt),
                 sb = std::sin(0.7 * tilt);
    for (auto &q : g.points()) {
      const double x = q[0] * ca, z = q[0] * sa, y = q[1];
      q = {x, y * cb - z * sb, y * sb + z * cb};
    }
    for (double ratio : {0.3, 0.1}) {
      cvc::simplify_params p;
      p.target_ratio = ratio;
      const geometry ref = cvc::simplify(g, p);
      for (const Xf &x : any) {
        SCOPED_TRACE(testing::Message() << "tilt " << tilt << " ratio " << ratio << " scale " << x.s
                                        << " shift " << x.tx);
        EXPECT_TRUE(
            bitwise_equal(cvc::simplify(transformed(g, x.s, x.tx, x.ty, x.tz), p).const_tris(),
                          ref.const_tris()));
      }
    }
  }
  for (double amp : {1e-3, 0.2}) {
    geometry g = make_grid(ctx, 31, amp);
    for (auto &q : g.points())
      q[2] = std::ldexp(std::round(std::ldexp(q[2], 12)), -12); // multiples of 2^-12
    cvc::simplify_params p;
    p.target_ratio = 0.1;
    const geometry ref = cvc::simplify(g, p);
    for (const Xf &x : exact) {
      SCOPED_TRACE(testing::Message() << "amp " << amp << " scale " << x.s << " shift " << x.tx);
      EXPECT_TRUE(bitwise_equal(
          cvc::simplify(transformed(g, x.s, x.tx, x.ty, x.tz), p).const_tris(), ref.const_tris()));
    }
  }
}

// --- edge cases --------------------------------------------------------------

// The loop never removes the last triangle, so even a target of 0 leaves a
// non-empty result with a finite (measured) error.
TEST(GeometrySimplify, NeverRemovesTheLastTriangle) {
  cvc::app ctx;
  geometry tri(ctx);
  tri.points() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  tri.tris().push_back({0, 1, 2});
  geometry tet(ctx);
  tet.points() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  tet.tris() = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  const geometry sheet = make_grid(ctx, 8, 0.0);
  for (const geometry *g : std::vector<const geometry *>{&tri, &tet, &sheet}) {
    cvc::simplify_params p;
    p.target_ratio = 0.0;
    p.preserve_boundary = false;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(*g, p, &r);
    expect_wellformed(s);
    EXPECT_GE(s.num_tris(), 1u);
    EXPECT_EQ(r.out_tris, s.num_tris());
    EXPECT_TRUE(std::isfinite(r.world_error));
    EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(*g, s));
    std::vector<cvc::simplify_result> rp;
    cvc::simplify_progressive(*g, {0u}, p, &rp);
    EXPECT_EQ(rp[0].out_tris, r.out_tris);
  }
}

// The Hausdorff queries run over a bounding-volume hierarchy, so a ground quad
// 10 km wide under 5 cm detail (triangle areas ~10^11 apart) costs about what
// the detail alone does. (A uniform grid sized from the mean triangle area piles
// the detail into a few cells and goes quadratic: ~100x slower here.)
TEST(GeometrySimplify, MeasureCostIgnoresTriangleSizeSpread) {
  cvc::app ctx;
  geometry detail = make_grid(ctx, 80, 6.0);
  for (auto &q : detail.points()) {
    q[0] *= 0.05;
    q[1] *= 0.05;
    q[2] = 1.0 + 0.05 * q[2];
  }
  geometry scene = detail;
  const std::uint64_t o = scene.points().size();
  const double G = 5000.0;
  scene.points().push_back({-G, -G, 0.0});
  scene.points().push_back({G, -G, 0.0});
  scene.points().push_back({G, G, 0.0});
  scene.points().push_back({-G, G, 0.0});
  scene.tris().push_back({o, o + 1, o + 2});
  scene.tris().push_back({o, o + 2, o + 3});
  cvc::simplify_params p;
  p.target_ratio = 0.3;
  auto timed = [&](const geometry &g, cvc::simplify_result &r) {
    const auto t0 = std::chrono::steady_clock::now();
    cvc::simplify(g, p, &r);
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  cvc::simplify_result ra, rb;
  const double ta = timed(detail, ra), tb = timed(scene, rb);
  EXPECT_GT(ra.world_error, 0.0);
  EXPECT_GT(rb.world_error, 0.0);
  EXPECT_LT(tb, 10.0 * ta + 0.5) << "detail alone " << ta << " s, under the ground quad " << tb
                                 << " s";
}

namespace {

// Best of three simplify() runs with seam welding on, and with it off,
// interleaved so a change in machine load hits both alike.
std::pair<double, double> weld_on_off_seconds(const geometry &g, cvc::simplify_result &on,
                                              double seam_epsilon = -1.0) {
  double t[2] = {1e9, 1e9};
  for (int rep = 0; rep < 3; ++rep)
    for (int weld = 1; weld >= 0; --weld) {
      cvc::simplify_params p;
      p.target_ratio = 0.5;
      p.weld_seams = weld != 0;
      p.seam_epsilon = seam_epsilon;
      cvc::simplify_result r;
      const auto t0 = std::chrono::steady_clock::now();
      cvc::simplify(g, p, &r);
      t[weld] = std::min(
          t[weld], std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
      if (weld)
        on = r;
    }
  return std::make_pair(t[1], t[0]);
}

} // namespace

// The weld finds bit-identical positions by sorting, so a pile of 21000 triangle
// corners on one point costs what the same mesh costs unwelded. (Testing every
// pair in a shared hash cell made it ~25x slower.) The pile's triangles weld
// into points and leave the result.
TEST(GeometrySimplify, WeldCostIgnoresAPileOfCoincidentCorners) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 100, 0.5);
  for (int i = 0; i < 7000; ++i) {
    const std::uint64_t o = g.points().size();
    for (int c = 0; c < 3; ++c)
      g.points().push_back({50.0, 50.0, 3.0});
    g.tris().push_back({o, o + 1, o + 2});
  }
  cvc::simplify_result on;
  const std::pair<double, double> t = weld_on_off_seconds(g, on);
  EXPECT_EQ(on.seam_vertices, 21000u);
  EXPECT_LT(t.first, 3.0 * t.second + 0.1)
      << "welded " << t.first << " s, unwelded " << t.second << " s";
}

// A 1 mm-tessellated part inside a 3 km scene, whose scene-wide tolerance
// (1e-6 of the extent, ~4.5 mm) spans many of the part's vertices, costs what
// the scene costs unwelded: the part's vertices search only within 1e-3 of
// their own edges, and the search skips subtrees of a vertex's own component.
// (A cell scan compared every pair of the part: ~4x.)
TEST(GeometrySimplify, WeldCostIgnoresFineDetailInALargeScene) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 160, 5.0);
  for (auto &q : g.points()) {
    q[0] *= 20.0;
    q[1] *= 20.0;
  }
  const geometry part = make_grid(ctx, 120, 0.5);
  const std::uint64_t o = g.points().size();
  for (const auto &q : part.const_points())
    g.points().push_back({1000.0 + 1e-3 * q[0], 1000.0 + 1e-3 * q[1], 50.0 + 1e-3 * q[2]});
  for (const auto &t : part.const_tris())
    g.tris().push_back({t[0] + o, t[1] + o, t[2] + o});
  cvc::simplify_result on;
  const std::pair<double, double> t = weld_on_off_seconds(g, on);
  EXPECT_EQ(on.seam_vertices, 0u);
  EXPECT_LT(t.first, 2.5 * t.second + 0.1)
      << "welded " << t.first << " s, unwelded " << t.second << " s";
  // An explicit 5 cm tolerance, for every vertex, reaches thousands of the
  // part's vertices from each of them, but the search skips subtrees of a
  // vertex's own component: it still finds nothing to weld, at no extra cost.
  const std::pair<double, double> te = weld_on_off_seconds(g, on, 0.05);
  EXPECT_EQ(on.seam_vertices, 0u);
  EXPECT_LT(te.first, 2.5 * te.second + 0.1)
      << "welded " << te.first << " s, unwelded " << te.second << " s";
}

// Near-coincident parts: 7000 copies of a unit triangle, each corner jittered
// by up to 1e-5 -- distinct positions, every copy its own part -- weld into
// one triangle, and so do 7000 tiny triangles jittered about one point under
// an explicit tolerance. Neither costs much more than the mesh unwelded: the
// search joins each cluster of mutually close positions once instead of
// pairing them all, and keeps only a spanning set of the pairs it finds.
// (Storing every pair took O(k^2) time and memory: ~8x and 2.3 GB at 8000.)
TEST(GeometrySimplify, WeldCostIgnoresAJitteredPile) {
  cvc::app ctx;
  std::mt19937_64 rng(7);
  auto jitter = [&]() { return 1e-5 * double(rng() >> 11) / 9007199254740992.0; };
  for (int tiny = 0; tiny < 2; ++tiny) {
    geometry g = make_grid(ctx, 100, 0.5);
    const double C[3][3] = {{50, 50, 3}, {51, 50, 3}, {50, 51, 3.5}};
    for (int i = 0; i < 7000; ++i) {
      const std::uint64_t o = g.points().size();
      for (int c = 0; c < 3; ++c) {
        const double *b = C[tiny ? 0 : c];
        g.points().push_back({b[0] + jitter(), b[1] + jitter(), b[2] + jitter()});
      }
      g.tris().push_back({o, o + 1, o + 2});
    }
    cvc::simplify_result on;
    const std::pair<double, double> t = weld_on_off_seconds(g, on, tiny ? 1e-4 : -1.0);
    SCOPED_TRACE(testing::Message() << (tiny ? "tiny, explicit tolerance" : "unit, default"));
    EXPECT_EQ(on.seam_vertices, 21000u);
    EXPECT_LT(t.first, 3.0 * t.second + 0.1)
        << "welded " << t.first << " s, unwelded " << t.second << " s";
  }
}

// A coordinate the measure cannot use -- infinite, NaN, or finite but so large
// that a squared distance overflows -- makes sampled_hausdorff +infinity, never
// 0, and simplify returns such a mesh unchanged at that unbounded error: its
// coarse rung would otherwise be chosen at every distance.
TEST(GeometrySimplify, NonFiniteInputIsNeverMeasuredAsExact) {
  cvc::app ctx;
  const geometry clean = make_grid(ctx, 30, 0.5);
  for (double bad :
       {std::numeric_limits<double>::infinity(), 1e200, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(testing::Message() << "bad coordinate " << bad);
    geometry g = clean;
    g.points()[17][2] = bad;
    EXPECT_TRUE(std::isinf(cvc::sampled_hausdorff(g, clean)));
    EXPECT_TRUE(std::isinf(cvc::sampled_hausdorff(clean, g)));
    EXPECT_TRUE(std::isinf(cvc::sampled_hausdorff(g, g)));
    cvc::simplify_params p;
    p.target_ratio = 0.2;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r);
    EXPECT_TRUE(std::isinf(r.world_error));
    EXPECT_EQ(r.out_tris, r.in_tris);
    EXPECT_EQ(r.collapses, 0u);
    EXPECT_EQ(s.num_tris(), g.num_tris());
    EXPECT_EQ(0, std::memcmp(s.const_points().data(), g.const_points().data(),
                             g.num_points() * sizeof(g.const_points()[0])));
  }
  // A non-finite vertex no triangle uses is not part of the surface.
  geometry stray = clean;
  stray.points().push_back({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_EQ(cvc::sampled_hausdorff(stray, clean), 0.0);
  cvc::simplify_result r;
  cvc::simplify(stray, cvc::simplify_params(), &r);
  EXPECT_TRUE(std::isfinite(r.world_error));
  EXPECT_LT(r.out_tris, r.in_tris);
}

// A triangle with two corners at one position is a zero-area needle, a
// segment: it is left out of the result AND of the surface world_error
// measures, so a needle standing off a sheet neither survives nor pins every
// rung's error at its length -- the sheet decimates exactly as it does without
// it. An input made only of needles is returned unchanged, never emptied. (A
// triangle that only welds shut within the tolerance is measured; see
// FinePartsInALargeSceneKeepTheirOwnScale.)
TEST(GeometrySimplify, ZeroAreaNeedlesAreNotMeasured) {
  cvc::app ctx;
  const geometry sheet = make_grid(ctx, 10, 0.2);
  geometry needle = sheet;
  const std::uint64_t o = needle.points().size();
  needle.points().push_back({3.0, 3.0, 4.0});
  needle.points().push_back({3.0, 3.0, 4.0});
  needle.tris().push_back({o, o + 1, 11});
  cvc::simplify_params p;
  p.target_tris = 120;
  cvc::simplify_result rs, rn;
  const geometry a = cvc::simplify(sheet, p, &rs), b = cvc::simplify(needle, p, &rn);
  ASSERT_GT(rs.world_error, 0.0);
  EXPECT_TRUE(bitwise_equal(a.const_points(), b.const_points()));
  EXPECT_TRUE(bitwise_equal(a.const_tris(), b.const_tris()));
  EXPECT_EQ(rn.world_error, rs.world_error);
  EXPECT_EQ(rn.world_error, cvc::sampled_hausdorff(sheet, b));
  EXPECT_GT(cvc::sampled_hausdorff(needle, b), 3.0); // the needle is 4 up

  geometry slivers(ctx);
  slivers.points() = {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 5, 0}, {0, 5, 0}, {2, 2, 0}};
  slivers.tris() = {{0, 1, 2}, {3, 4, 5}};
  slivers.set_geometry_type(geometry::SURFACE_TRI);
  p.target_tris = 1;
  cvc::simplify_result r;
  const geometry kept = cvc::simplify(slivers, p, &r);
  EXPECT_EQ(kept.num_tris(), 2u);
  EXPECT_EQ(r.out_tris, 2u);
  EXPECT_EQ(r.world_error, 0.0);
  EXPECT_TRUE(bitwise_equal(kept.const_tris(), slivers.const_tris()));

  // A point -- all three corners at one position -- has no edge to set a
  // weld tolerance by: it does not weld to the sheet vertex 1e-7 from it,
  // and the sheet decimates exactly as it does alone.
  geometry point = sheet;
  const std::uint64_t q = point.points().size();
  geometry::point_t at = sheet.const_points()[33];
  at[2] += 1e-7;
  point.points().insert(point.points().end(), 3, at);
  point.tris().push_back({q, q + 1, q + 2});
  p.target_tris = 120;
  const geometry c = cvc::simplify(point, p, &r);
  EXPECT_EQ(r.seam_vertices, 3u); // the point's own corners, not vertex 33
  EXPECT_TRUE(bitwise_equal(a.const_points(), c.const_points()));
  EXPECT_TRUE(bitwise_equal(a.const_tris(), c.const_tris()));
}

// Round-off is judged against the coordinates' magnitude as well as the
// extent, so a surface and a copy of it (its triangles reversed and flipped)
// measure exactly 0 however far from the origin they sit.
TEST(GeometrySimplify, CoincidentSurfacesMeasureZeroFarFromTheOrigin) {
  cvc::app ctx;
  for (double offset : {0.0, 3.3e6, 6e6, 5e7, 1e9}) {
    geometry g = make_grid(ctx, 20, 0.3);
    for (auto &q : g.points())
      for (int k = 0; k < 3; ++k)
        q[k] = 0.05 * q[k] + offset;
    geometry h = g;
    std::reverse(h.tris().begin(), h.tris().end());
    for (auto &t : h.tris())
      std::swap(t[0], t[1]);
    EXPECT_EQ(cvc::sampled_hausdorff(g, g), 0.0) << "offset " << offset;
    EXPECT_EQ(cvc::sampled_hausdorff(g, h), 0.0) << "offset " << offset;
  }
}

// --- the LOD contract ----------------------------------------------------------

namespace {

// `g` with every triangle split into 4^levels by repeated midpoint subdivision:
// the same surface, sampled far more densely by sampled_hausdorff.
geometry subdivide(const geometry &g, int levels) {
  geometry s = g;
  for (int l = 0; l < levels; ++l) {
    geometry next(g.ctx());
    next.points() = s.const_points();
    std::map<std::pair<std::uint64_t, std::uint64_t>, std::uint64_t> mid;
    auto midpoint = [&](std::uint64_t a, std::uint64_t b) {
      const auto key = std::make_pair(std::min(a, b), std::max(a, b));
      auto it = mid.find(key);
      if (it == mid.end()) {
        const auto &P = s.const_points();
        next.points().push_back(
            {0.5 * (P[a][0] + P[b][0]), 0.5 * (P[a][1] + P[b][1]), 0.5 * (P[a][2] + P[b][2])});
        it = mid.insert(std::make_pair(key, std::uint64_t(next.points().size() - 1))).first;
      }
      return it->second;
    };
    for (const auto &t : s.const_tris()) {
      const std::uint64_t ab = midpoint(t[0], t[1]), bc = midpoint(t[1], t[2]),
                          ca = midpoint(t[2], t[0]);
      next.tris().push_back({t[0], ab, ca});
      next.tris().push_back({ab, t[1], bc});
      next.tris().push_back({ca, bc, t[2]});
      next.tris().push_back({ab, bc, ca});
    }
    s = next;
  }
  return s;
}

} // namespace

// lod_final's P1 check, on the box city: at the switch radius select derives
// from a rung's world_error_m, the rung's DENSELY sampled Hausdorff distance to
// the source (both surfaces subdivided 16x first) costs at most 1.05x the pixel
// budget. (On this feature-aligned fixture the worst point is a sampled corner;
// see sampled_hausdorff for why a strongly curved surface can fall short of
// that.)
TEST(GeometrySimplify, SwitchRadiusHoldsThePixelBudgetOnTheBoxCity) {
  cvc::app ctx;
  const geometry g = box_city(ctx);
  const geometry dense_src = subdivide(g, 2);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 6; // targets 857 .. 27; lossless down to ~146
  pp.mesh_ratio = 0.5;
  pp.mesh_min_tris = 16;
  const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(g, pp, &ctx.computePool());
  const cvc::lod::view_params v = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  int lossy = 0;
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k) {
    const double err = pyr.world_error_m[k];
    SCOPED_TRACE(testing::Message() << "rung " << k << " err " << err);
    if (err == 0.0) {
      // coplanar collapses only: the rung lies on the source and vice versa
      EXPECT_EQ(cvc::sampled_hausdorff(g, pyr.rungs[k]), 0.0);
      continue;
    }
    ++lossy;
    const double dense = cvc::sampled_hausdorff(dense_src, subdivide(pyr.rungs[k], 2));
    const double r = cvc::lod::switch_radius_m(err, v);
    EXPECT_LE(cvc::lod::screen_error_px(dense, r, v), 1.05 * v.desired_pixel_error)
        << "dense " << dense;
  }
  EXPECT_GE(lossy, 3);
}

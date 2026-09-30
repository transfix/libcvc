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
// linearly with the mesh, matches an independent brute-force measurement and
// holds the pixel budget at the switch radius select derives from it; input
// normals survive bit-for-bit; progressive snapshots equal independent runs
// bit-for-bit; seam welding keeps roofs sealed to their walls and decimates
// faces split for hard edges (down to a triangle soup) without cracks; a far
// away part cannot change how another decimates; the last triangle is never
// removed; and a huge triangle under fine detail does not slow the measurement.

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
#include <gtest/gtest.h>
#include <limits>
#include <map>
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
  // coincidence search's hash cells (64 * seam_epsilon = 1 wide, starting at the
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

  // The default tolerance is capped by the median edge (1e-3 of it), so a far
  // away part that stretches the extent to 100 km (1e-6 of which is 0.1 m) does
  // not weld two parts 5 mm apart -- while a 0.1 mm contact still welds.
  for (double gap : {5e-3, 1e-4}) {
    geometry scene(ctx);
    scene.points() = {{0, 0, 0}, {3 - gap, 0, 0}, {0, 1, 0},       {3, 0, 0},  {5, 0, 0},
                      {3, 1, 0}, {1e5, 0, 0},     {1e5 + 1, 0, 0}, {1e5, 1, 0}};
    scene.tris() = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
    cvc::simplify_params d;
    d.target_tris = 2;
    cvc::simplify(scene, d, &r);
    EXPECT_EQ(r.seam_vertices, gap < 1e-3 ? 2u : 0u) << "gap " << gap;
  }
}

// --- welding split faces ----------------------------------------------------

namespace {

// `g` with every run of `group` consecutive triangles given its own copies of
// their vertices -- make_grid's quad pairs (group 2) are then a flat-shaded
// export, single triangles (group 1) a triangle soup. Each copy's uv traces it
// to its index here; normals are painted.
geometry split_faces(const geometry &g, std::size_t group) {
  geometry s(g.ctx());
  const auto &T = g.const_tris();
  for (std::size_t f = 0; f < T.size(); f += group) {
    std::map<std::uint64_t, std::uint64_t> copy;
    for (std::size_t t = f; t < std::min(T.size(), f + group); ++t) {
      std::array<std::uint64_t, 3> c;
      for (int k = 0; k < 3; ++k) {
        auto it = copy.find(T[t][k]);
        if (it == copy.end()) {
          it = copy.insert(std::make_pair(T[t][k], std::uint64_t(s.points().size()))).first;
          s.points().push_back(g.const_points()[T[t][k]]);
          s.uvs().push_back({double(it->second), 0.0});
        }
        c[k] = it->second;
      }
      s.tris().push_back({c[0], c[1], c[2]});
    }
  }
  paint_normals(s);
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

} // namespace

// Faces split apart for hard-edge normals -- per quad, or all the way down to a
// triangle soup, where EVERY vertex coincides with another part's -- decimate
// like the welded sheet they form: to the target, without cracks, and with
// every surviving vertex (including those moved onto a kept position) carrying
// its own source normal bit-for-bit. Without welding they fall apart.
TEST(GeometrySimplify, WeldDecimatesSplitFacesWithoutCracks) {
  cvc::app ctx;
  const int n = 24;
  const geometry sheet = make_grid(ctx, n, 1.5);
  for (std::size_t group : {2u, 1u}) {
    const geometry g = split_faces(sheet, group);
    SCOPED_TRACE(testing::Message() << "group " << group);
    ASSERT_EQ(interior_cracks(g, n), 0);
    cvc::simplify_params p;
    p.target_ratio = 0.2;
    cvc::simplify_result r;
    const geometry s = cvc::simplify(g, p, &r, &ctx.computePool());
    expect_wellformed(s);
    const std::uint64_t target = std::uint64_t(std::llround(0.2 * double(g.num_tris())));
    EXPECT_LE(r.out_tris, target);
    EXPECT_GE(r.out_tris + 2, target); // an interior collapse removes two triangles
    // every copy is welded to another but those at the sheet's corners that
    // only one part touches: 4 per-quad, 2 in the soup (the diagonal's ends)
    EXPECT_EQ(r.seam_vertices, g.num_points() - (group == 2 ? 4u : 2u));
    EXPECT_EQ(interior_cracks(s, n), 0);
    EXPECT_EQ(r.world_error, cvc::sampled_hausdorff(g, s));
    for (std::size_t i = 0; i < s.num_points(); ++i) {
      const std::size_t from = std::size_t(s.const_uvs()[i][0]);
      EXPECT_EQ(0, std::memcmp(&s.const_normals()[i], &g.const_normals()[from],
                               sizeof(g.const_normals()[from])));
    }
    // The same split mesh without welding: every part decimates on its own and
    // the sheet tears.
    p.weld_seams = false;
    const geometry torn = cvc::simplify(g, p);
    EXPECT_GT(interior_cracks(torn, n), 0);
  }
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
  for (double far : {200.0, 1e4, 1e6}) {
    geometry f = g;
    const std::uint64_t o = f.points().size();
    f.points().push_back({far, far, 0.0});
    f.points().push_back({far + 1.0, far, 0.0});
    f.points().push_back({far, far + 1.0, 0.0});
    f.tris().push_back({o, o + 1, o + 2});
    cvc::simplify_params q = p;
    q.target_tris = p.target_tris + 1; // the far triangle survives
    const geometry both = cvc::simplify(f, q, nullptr, &ctx.computePool());
    SCOPED_TRACE(testing::Message() << "far " << far);
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

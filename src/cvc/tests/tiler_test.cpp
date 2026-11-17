// Temporary smoke test while the port comes up.
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/tiler.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <sstream>

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

namespace {

std::vector<double> circle(double cx, double cy, double r, int n) {
  std::vector<double> xy;
  for (int i = 0; i < n; ++i) {
    const double a = 2 * 3.14159265358979323846 * i / n;
    xy.push_back(std::ldexp(std::round(std::ldexp(cx + r * std::cos(a), 10)), -10));
    xy.push_back(std::ldexp(std::round(std::ldexp(cy + r * std::sin(a), 10)), -10));
  }
  return xy;
}

using P3 = std::array<double, 3>;
using Tri = std::array<P3, 3>;

struct mesh {
  std::vector<P3> v;
  std::vector<std::array<std::size_t, 3>> t;
};

mesh read_tiles(const std::string &path) {
  mesh m;
  std::ifstream in(path);
  std::size_t nv = 0, nt = 0;
  in >> nv >> nt;
  m.v.resize(nv);
  for (auto &p : m.v)
    in >> p[0] >> p[1] >> p[2];
  m.t.resize(nt);
  for (auto &t : m.t)
    in >> t[0] >> t[1] >> t[2];
  return m;
}

mesh from_geometry(const cvc::geometry &g) {
  mesh m;
  for (const auto &p : g.points())
    m.v.push_back({p[0], p[1], p[2]});
  for (const auto &t : g.tris())
    m.t.push_back({std::size_t(t[0]), std::size_t(t[1]), std::size_t(t[2])});
  return m;
}

bool on_slice(const P3 &p) { return p[2] == std::floor(p[2]); }

// A triangle of contour vertices, as its sorted corners (orientation kept apart).
std::set<Tri> contour_tris(const mesh &m) {
  std::set<Tri> out;
  for (const auto &t : m.t) {
    Tri tr{m.v[t[0]], m.v[t[1]], m.v[t[2]]};
    if (!on_slice(tr[0]) || !on_slice(tr[1]) || !on_slice(tr[2]))
      continue;
    std::sort(tr.begin(), tr.end());
    out.insert(tr);
  }
  return out;
}

} // namespace

TEST(Tiler, Smoke) {
  cvc::contour_stack s;
  for (int k = 0; k < 4; ++k)
    s.add(k, "tube", circle(0.01 * k, 0, 1 + 0.05 * (k % 2), 16));
  cvc::tile_result r = cvc::tile_contours(s);
  for (const auto &w : r.warnings)
    std::printf("warning: %s\n", w.c_str());
  ASSERT_EQ(r.components.size(), 1u);
  const cvc::geometry &g = r.components.at("tube");
  std::printf("points %zu tris %zu\n", g.points().size(), g.tris().size());
  EXPECT_GT(g.tris().size(), 0u);
}

TEST(Tiler, DebugGoldens) {
  const std::string inputs = std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/tiler/";
  const std::string goldens = std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/tiler/";
  struct c {
    const char *name;
    int first, last;
    double delta;
    bool ri;
    std::vector<const char *> comps;
  };
  for (const c &cs : {c{"tube", 1, 6, 0.01, false, {"a001"}},
                      c{"branch", 0, 7, 0.01, false, {"d001", "r001"}},
                      c{"touching", 1, 5, 0.05, true, {"a002", "a003"}}}) {
    cvc::app app;
    cvc::contour_read_params rp;
    rp.first = cs.first;
    rp.last = cs.last;
    cvc::contour_stack stack = cvc::read_contours(
        app, inputs + cs.name + "/" + cs.name + ".ser", rp);
    cvc::tile_params p;
    p.curation_delta = cs.delta;
    p.collinear_epsilon = 1e-15;
    p.remove_intersections = cs.ri;
    p.z_first = cs.first;
    p.z_last = cs.last;
    p.cap_ends = false;
    cvc::tile_result r = cvc::tile_contours(stack, p);
    std::printf("== %s: %zu warnings\n", cs.name, r.warnings.size());
    for (const auto &w : r.warnings)
      std::printf("   warning: %s\n", w.c_str());
    for (const char *comp : cs.comps) {
      const mesh legacy = read_tiles(goldens + cs.name + "." + comp + ".tiles.txt");
      const mesh port = r.components.count(comp) ? from_geometry(r.components.at(comp)) : mesh{};
      const auto lc = contour_tris(legacy), pc = contour_tris(port);
      std::size_t common = 0;
      for (const auto &t : lc)
        common += pc.count(t);
      std::printf("  %s: legacy %zu v %zu t (%zu contour tris); port %zu v %zu t (%zu contour "
                  "tris); common contour tris %zu\n",
                  comp, legacy.v.size(), legacy.t.size(), lc.size(), port.v.size(), port.t.size(),
                  pc.size(), common);
      for (const mesh *m : {&legacy, &port}) {
        std::set<double> zs;
        for (const P3 &v : m->v)
          if (!on_slice(v) || v[2] == 0)
            zs.insert(v[2]);
        std::printf("    %s fill z:", m == &legacy ? "legacy" : "port");
        for (double z : zs)
          std::printf(" %.4g", z);
        std::printf("\n");
      }
    }
  }
}

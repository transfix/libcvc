// Temporary smoke test while the port comes up.
#include <cmath>
#include <cstdio>
#include <cvc/geometry/tiler.h>
#include <gtest/gtest.h>

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

// ---- temporary internals harness ----
#include "Contour.h"
#include "Slice.h"
#include "tiler_operations.h"
#include "print_utils.h"
#include "sweep_line_visitors.h"
#include "polygon_utils.h"

TEST(Tiler, DebugInternals) {
  using namespace cvc::tiler;
  std::vector<Contour_handle> bottom, top;
  auto make = [](double z, int idbase) {
    Polygon_2 P;
    std::vector<double> xy = circle(0, 0, 1, 8);
    for (std::size_t i = 0; i < xy.size() / 2; ++i)
      P.push_back(Point_2(xy[2 * i], xy[2 * i + 1], z));
    return Contour::create(P, Contour::Info(z, "tube", "tube"));
  };
  bottom.push_back(make(0, 0));
  top.push_back(make(1, 100));
  {
    const Polygon_2 &P = bottom[0]->polygon(), &Q = top[0]->polygon();
    std::list<Segment_2> s0(P.edges_begin(), P.edges_end()), s1(Q.edges_begin(), Q.edges_end());
    std::printf("has_intersection %d boundaries_intersect %d relation %d do_intersect(e0,e0) %d\n",
                int(has_intersection(s0.begin(), s0.end(), s1.begin(), s1.end(), true, true)),
                int(boundaries_intersect(P, Q, true)), int(cvc::tiler::relation(P, Q, false)),
                int(do_intersect(*P.edges_begin(), *Q.edges_begin())));
    std::printf("can_tile %d\n", int(can_tile(bottom[0], top[0], Hierarchy(bottom.begin(), bottom.end(), Hierarchy_policy::FORCE_CCW), Hierarchy(top.begin(), top.end(), Hierarchy_policy::FORCE_CCW), -1)));
  }
  TW_handle w = pre_tile(bottom.begin(), bottom.end(), top.begin(), top.end(), false, -1);
  std::printf("correspondences: bottom %zu top %zu\n", w->correspondences.count(bottom[0]),
              w->correspondences.count(top[0]));
  for (const Contour_handle &c : w->contours) {
    for (auto vit = c->polygon().vertices_begin(); vit != c->polygon().vertices_end(); ++vit) {
      Point_3 v = w->vertices.get(c, vit);
      std::printf("v %s region %s otv %s\n", pp(v).c_str(), pp(*w->tiling_regions[v]).c_str(),
                  w->otv_table.contains(v) ? pp(w->otv_table[v]).c_str() : "-");
    }
  }
}

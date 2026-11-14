/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/roads.h for the full header).
*/

#include <algorithm>
#include <cmath>
#include <cvc/world/road_grade.h>
#include <cvc/world/roads.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vector>

using namespace cvc::world;

namespace {
// Strict interior segment-segment crossing (shared endpoints do not count).
bool crosses(double ax, double ay, double bx, double by, double cx, double cy, double dx,
             double dy) {
  const double rX = bx - ax, rY = by - ay, sX = dx - cx, sY = dy - cy;
  const double den = rX * sY - rY * sX;
  if (std::fabs(den) < 1e-12)
    return false;
  const double t = ((cx - ax) * sY - (cy - ay) * sX) / den;
  const double u = ((cx - ax) * rY - (cy - ay) * rX) / den;
  const double e = 1e-4;
  return t > e && t < 1 - e && u > e && u < 1 - e;
}

double checksum(const road_network &n) {
  double s = 0;
  for (const auto &nd : n.nodes)
    s += nd.x * 1.1 + nd.y * 2.3;
  for (const auto &sg : n.segments)
    s += sg.a * 3.7 + sg.b * 5.1 + sg.width;
  return s;
}

road_params demo_params() {
  road_params p;
  p.seed = 7;
  p.half = 120.0;
  p.max_segments = 1500;
  return p;
}
} // namespace

TEST(WorldRoads, GeneratesAConnectedNetwork) {
  const road_network n = generate_roads(demo_params());
  EXPECT_GT(n.nodes.size(), 10u);
  EXPECT_GT(n.segments.size(), 10u);
  // Every segment references valid nodes and has positive length + width.
  for (const road_segment &s : n.segments) {
    ASSERT_GE(s.a, 0);
    ASSERT_GE(s.b, 0);
    ASSERT_LT(s.a, int(n.nodes.size()));
    ASSERT_LT(s.b, int(n.nodes.size()));
    EXPECT_NE(s.a, s.b);
    EXPECT_GT(s.width, 0.0);
  }
}

TEST(WorldRoads, IsDeterministicInSeed) {
  const road_network a = generate_roads(demo_params());
  const road_network b = generate_roads(demo_params());
  ASSERT_EQ(a.nodes.size(), b.nodes.size());
  ASSERT_EQ(a.segments.size(), b.segments.size());
  EXPECT_DOUBLE_EQ(checksum(a), checksum(b));
  road_params q = demo_params();
  q.seed = 8;
  EXPECT_NE(checksum(a), checksum(generate_roads(q))); // a different seed -> a different city
}

// The headline correctness property: NO two roads cross except at a shared junction node. This is
// what the local-constraint snapping (crossing-cut / T-junction / extend) guarantees.
TEST(WorldRoads, NoRoadsCrossWithoutAJunction) {
  const road_network n = generate_roads(demo_params());
  int bad = 0;
  for (std::size_t i = 0; i < n.segments.size(); ++i)
    for (std::size_t j = i + 1; j < n.segments.size(); ++j) {
      const road_segment &s = n.segments[i], &t = n.segments[j];
      if (s.a == t.a || s.a == t.b || s.b == t.a || s.b == t.b)
        continue; // share a node -> a legitimate junction
      if (crosses(n.nodes[s.a].x, n.nodes[s.a].y, n.nodes[s.b].x, n.nodes[s.b].y, n.nodes[t.a].x,
                  n.nodes[t.a].y, n.nodes[t.b].x, n.nodes[t.b].y))
        ++bad;
    }
  EXPECT_EQ(bad, 0) << bad << " road pairs cross without a shared junction node";
}

TEST(WorldRoads, HasRealIntersections) {
  const road_network n = generate_roads(demo_params());
  int junctions = 0;
  for (const road_node &nd : n.nodes)
    if (nd.degree > 2)
      ++junctions;
  EXPECT_GT(junctions, 0); // a street network must have T/cross junctions
}

TEST(WorldRoads, StaysInsideTheRegion) {
  const road_params p = demo_params();
  const road_network n = generate_roads(p);
  for (const road_node &nd : n.nodes) {
    EXPECT_LE(std::fabs(nd.x), p.half + 1e-6);
    EXPECT_LE(std::fabs(nd.y), p.half + 1e-6);
  }
}

TEST(WorldRoads, FlatTerrainGivesTheGridDirection) {
  road_params p = demo_params();
  p.grid_angle_deg = 30.0;
  double dx = 0, dy = 0;
  road_field_dir(p, {}, 10.0, -20.0, dx, dy); // no height sampler -> pure grid
  EXPECT_NEAR(dx, std::cos(30.0 * M_PI / 180.0), 1e-9);
  EXPECT_NEAR(dy, std::sin(30.0 * M_PI / 180.0), 1e-9);
}

TEST(WorldRoads, DistanceAndNearQueries) {
  const road_network n = generate_roads(demo_params());
  // The centre seeded the first node, so a point at the origin is on/very near a road.
  EXPECT_LT(n.distance_to_road(0.0, 0.0), 5.0);
  EXPECT_TRUE(n.on_or_near_road(0.0, 0.0, 2.0));
  // A far corner outside the city extent is not near any road.
  EXPECT_FALSE(n.on_or_near_road(1e5, 1e5, 1.0));
}

// distance_to_road is backed by a spatial-grid index; it must return exactly what a brute-force
// scan would, at points NEAR a road (fast path) and FAR from any road (fallback), including the
// segment index and the lowest-index tie-break.
TEST(WorldRoads, DistanceMatchesBruteForce) {
  const road_network n = generate_roads(demo_params());
  ASSERT_FALSE(n.empty());
  auto brute = [&](double x, double y, int &bseg) {
    double best = 1e30;
    bseg = -1;
    for (std::size_t i = 0; i < n.segments.size(); ++i) {
      const road_node &a = n.nodes[n.segments[i].a], &b = n.nodes[n.segments[i].b];
      const double vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy;
      double t = L2 > 1e-12 ? ((x - a.x) * vx + (y - a.y) * vy) / L2 : 0.0;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      const double dx = a.x + t * vx - x, dy = a.y + t * vy - y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < best) { // same lowest-index tie-break as the index
        best = d;
        bseg = int(i);
      }
    }
    return best;
  };
  const double h = n.half;
  int checked = 0;
  for (int i = 0; i < 37; ++i)
    for (int j = 0; j < 37; ++j) {
      const double x = -h + 2 * h * i / 36.0, y = -h + 2 * h * j / 36.0;
      int bseg = -1;
      const double bd = brute(x, y, bseg);
      int iseg = -1;
      const double id = n.distance_to_road(x, y, &iseg);
      EXPECT_NEAR(id, bd, 1e-9) << "at (" << x << "," << y << ")";
      EXPECT_EQ(iseg, bseg) << "segment mismatch at (" << x << "," << y << ")";
      ++checked;
    }
  EXPECT_GT(checked, 0);
}

namespace {
double seg_len(const road_network &n, const road_segment &s) {
  return std::hypot(n.nodes[s.b].x - n.nodes[s.a].x, n.nodes[s.b].y - n.nodes[s.a].y);
}
} // namespace

// The grade limit: a steep hill east of the centre (30 m high, its flanks up to 60 %). Unlimited,
// the streets run straight over it; held to 12 %, every segment is steered toward the contours
// and stays under the limit (with the snapped end's 1.5x slack), and most of the network survives,
// winding round the hill.
TEST(WorldRoads, GradeLimitSteersAlongTheContours) {
  const road_height_fn hill = [](double x, double y) {
    return 30.0 * std::exp(-((x - 80.0) * (x - 80.0) + y * y) / (2.0 * 30.0 * 30.0));
  };
  road_params p = demo_params();
  auto worst = [&](const road_network &n) {
    double g = 0.0;
    for (const road_segment &s : n.segments) {
      const road_node &a = n.nodes[s.a], &b = n.nodes[s.b];
      g = std::max(g, std::fabs(hill(b.x, b.y) - hill(a.x, a.y)) / seg_len(n, s));
    }
    return g;
  };
  const road_network free = generate_roads(p, hill);
  EXPECT_GT(worst(free), 0.2);
  p.max_grade = 0.12;
  const road_network n = generate_roads(p, hill);
  EXPECT_LE(worst(n), 0.12 * 1.5 + 1e-9);
  EXPECT_GT(n.segments.size(), free.segments.size() / 2);
  int onHill = 0; // the streets still reach the hill's flanks
  for (const road_node &q : n.nodes)
    onHill += hill(q.x, q.y) > 3.0 ? 1 : 0;
  EXPECT_GT(onHill, 5);
}

// The no-go mask (a lake east of x = 40): no road reaches into it, where unmasked ones do.
TEST(WorldRoads, MaskKeepsRoadsOut) {
  const road_params p = demo_params();
  const road_mask_fn lake = [](double x, double) { return x > 40.0; };
  auto east = [](const road_network &n) {
    double m = -1e30;
    for (const road_segment &s : n.segments)
      m = std::max({m, n.nodes[s.a].x, n.nodes[s.b].x});
    return m;
  };
  EXPECT_GT(east(generate_roads(p)), 60.0);
  const road_network n = generate_roads(p, {}, lake);
  EXPECT_GT(n.segments.size(), 50u);
  EXPECT_LE(east(n), 40.0 + 1e-9);
}

// The hierarchy and districts: two centres each seed an arterial cross; an arterial's branches
// are collectors (their own width), which sprout the local streets; a dense core packs local
// streets tighter than sparse ground does.
TEST(WorldRoads, HierarchyAndDistricts) {
  road_params p = demo_params();
  p.half = 240.0;
  p.max_segments = 4000;
  p.collector_width = 7.5;
  p.centres = {{-120.0, 0.0}, {120.0, 0.0}};
  const road_network n = generate_roads(p);
  int kinds[3] = {0, 0, 0};
  for (const road_segment &s : n.segments) {
    ++kinds[int(s.kind)];
    EXPECT_EQ(s.arterial, s.kind == road_class::arterial);
    EXPECT_EQ(s.width, s.kind == road_class::arterial    ? p.arterial_width
                       : s.kind == road_class::collector ? p.collector_width
                                                         : p.local_width);
  }
  EXPECT_GT(kinds[0], 0);
  EXPECT_GT(kinds[1], 0);
  EXPECT_GT(kinds[2], 0);
  for (const road_centre &c : p.centres) {
    int at = 0;
    for (const road_segment &s : n.segments)
      at += (std::hypot(n.nodes[s.a].x - c.x, n.nodes[s.a].y - c.y) < 1e-9) ? 1 : 0;
    EXPECT_GE(at, 2) << c.x; // the arterials leave each centre
  }

  // Urbanity 1 west of x = 0, 0 east: sparse ground has fewer, wider-spaced local streets.
  p.sparse_spacing = 0.5;
  p.sparse_branch = 0.3;
  p.parallel_gap = 0.5;
  const road_network d =
      generate_roads(p, {}, {}, [](double x, double) { return x < 0.0 ? 1.0 : 0.0; });
  double west = 0.0, eastLen = 0.0;
  for (const road_segment &s : d.segments)
    if (s.kind == road_class::local)
      (d.nodes[s.a].x < 0.0 ? west : eastLen) += seg_len(d, s);
  EXPECT_GT(west, 1.5 * eastLen);
}

// prune_roads trims every dead-end chain shorter than the limit and leaves the degrees right.
TEST(WorldRoads, PruneTrimsShortStubs) {
  road_network n = generate_roads(demo_params());
  const std::size_t before = n.segments.size();
  const std::size_t removed = prune_roads(n, 60.0);
  EXPECT_GT(removed, 0u);
  EXPECT_EQ(n.segments.size() + removed, before);
  std::vector<int> deg(n.nodes.size(), 0);
  std::vector<std::vector<int>> adj(n.nodes.size());
  for (std::size_t i = 0; i < n.segments.size(); ++i) {
    ++deg[n.segments[i].a];
    ++deg[n.segments[i].b];
    adj[n.segments[i].a].push_back(int(i));
    adj[n.segments[i].b].push_back(int(i));
  }
  for (std::size_t k = 0; k < n.nodes.size(); ++k) {
    EXPECT_EQ(n.nodes[k].degree, deg[k]);
    if (deg[k] != 1)
      continue;
    double length = 0.0;
    int cur = int(k), seg = adj[k][0];
    while (true) {
      length += seg_len(n, n.segments[seg]);
      const road_segment &s = n.segments[seg];
      cur = s.a == cur ? s.b : s.a;
      if (deg[cur] != 2 || length >= 60.0)
        break;
      seg = adj[cur][0] == seg ? adj[cur][1] : adj[cur][0];
    }
    EXPECT_GE(length, 60.0) << "stub at node " << k;
  }
}

// ── road grading (road_grade.h) ─────────────────────────────────────────────────────────────────

namespace {

// A ground from a height function, for the grading tests.
class fn_ground : public terrain_source {
public:
  explicit fn_ground(std::function<double(double, double)> h) : h_(std::move(h)) {}
  double sample(double x, double y) const override { return h_(x, y); }
  terrain_bounds bounds() const override { return {-500, -500, 500, 500}; }
  double relief_m() const override { return 20.0; }

private:
  std::function<double(double, double)> h_;
};

// A road of `n` segments along x from x0 to x1 at y, width w, appended to `net`. Returns its nodes.
std::vector<int> add_road(road_network &net, double x0, double x1, double y, int n, double w) {
  std::vector<int> ids;
  for (int i = 0; i <= n; ++i) {
    ids.push_back(int(net.nodes.size()));
    net.nodes.push_back(road_node{x0 + (x1 - x0) * i / n, y, (i == 0 || i == n) ? 1 : 2});
  }
  for (int i = 0; i < n; ++i) {
    road_segment s;
    s.a = ids[std::size_t(i)];
    s.b = ids[std::size_t(i) + 1];
    s.width = w;
    net.segments.push_back(s);
  }
  return ids;
}

// A rolling, bumpy ground: a 4% ramp, 1.5 m bumps every 20 m, and a side slope of 30%.
double bumpy(double x, double y) { return 0.04 * x + 1.5 * std::sin(x * 0.314) + 0.3 * y + 5.0; }

} // namespace

// The road over bumpy ground is held to its grade, changes grade gradually, and its surface is
// level across, though the ground under it climbs 30% sideways and bumps 1.5 m every 20 m.
TEST(WorldRoadGrade, ARoadIsSmoothAlongAndLevelAcross) {
  road_network net;
  const std::vector<int> ids = add_road(net, -200.0, 200.0, 0.0, 80, 6.0); // 5 m segments
  const auto ground = std::make_shared<fn_ground>(bumpy);
  road_grade_params p;
  const std::vector<double> z = grade_road_profile(net, *ground, p);
  ASSERT_EQ(z.size(), net.nodes.size());
  double worstGrade = 0, worstChange = 0, groundChange = 0;
  for (std::size_t i = 1; i < ids.size(); ++i) {
    const double g = (z[ids[i]] - z[ids[i - 1]]) / 5.0;
    worstGrade = std::max(worstGrade, std::abs(g));
    if (i >= 2) {
      const double g0 = (z[ids[i - 1]] - z[ids[i - 2]]) / 5.0;
      worstChange = std::max(worstChange, std::abs(g - g0));
      const double h2 = bumpy(net.nodes[ids[i]].x, 0), h1 = bumpy(net.nodes[ids[i - 1]].x, 0),
                   h0 = bumpy(net.nodes[ids[i - 2]].x, 0);
      groundChange = std::max(groundChange, std::abs((h2 - h1) - (h1 - h0)) / 5.0);
    }
  }
  EXPECT_LE(worstGrade, p.max_grade + 1e-6);
  EXPECT_LT(worstChange, 0.25 * groundChange)
      << "the grade changes far more gently than the ground";
  // Never far from the ground it is laid on: through the 1.5 m bumps, a little low (roads cut
  // more than they bank).
  for (int i : ids)
    EXPECT_LT(std::abs(z[std::size_t(i)] - bumpy(net.nodes[std::size_t(i)].x, 0)), 2.5);

  // Level across: within 2 cm edge to edge, 0.3% (exactly level mid-segment, a little off
  // within a metre of a node, where the next segment blends in), where the ground tilts 30%.
  const graded_terrain g(ground, net, z, p);
  for (double x = -150.0; x <= 150.0; x += 0.73) {
    const double c = g.sample(x, 0.0), l = g.sample(x, 2.9), r = g.sample(x, -2.9);
    EXPECT_NEAR(l, c, 0.02) << x;
    EXPECT_NEAR(r, c, 0.02) << x;
  }
  EXPECT_NEAR(g.sample(12.5, 2.9), g.sample(12.5, -2.9), 1e-9) << "mid-segment: exact";
  // Far from the road the ground is its own; between, it eases back without a step.
  EXPECT_EQ(g.sample(0.0, 80.0), bumpy(0.0, 80.0));
  EXPECT_FALSE(g.graded_at(0.0, 80.0));
  EXPECT_TRUE(g.graded_at(0.0, 3.0));
  double prev = g.sample(10.0, 0.0);
  for (double y = 0.25; y < 60.0; y += 0.25) {
    const double h = g.sample(10.0, y);
    EXPECT_LT(std::abs(h - prev), 0.5) << y;
    prev = h;
  }
}

// A road crossing a dip under the water stays above it, and the ground under it is raised to it.
TEST(WorldRoadGrade, ARoadStaysAboveTheWater) {
  road_network net;
  add_road(net, -100.0, 100.0, 0.0, 40, 6.0);
  const auto ground = std::make_shared<fn_ground>([](double x, double) {
    return 3.0 - 6.0 * std::exp(-x * x / 400.0); // a 3 m-deep hollow at x = 0, the sea at 0
  });
  road_grade_params p;
  p.water_level = 0.0;
  const std::vector<double> z = grade_road_profile(net, *ground, p);
  for (double zi : z)
    EXPECT_GE(zi, p.water_level + p.water_clearance - 1e-9);
  const graded_terrain g(ground, net, z, p);
  for (double x = -30.0; x <= 30.0; x += 2.5)
    EXPECT_GE(g.sample(x, 2.0), p.water_level + p.water_clearance - 1e-9) << x;
  EXPECT_LT(g.sample(0.0, 40.0), 0.0) << "away from the road the hollow is still under water";
}

// A graded ground keeps no working state between samples: threads sampling it at once, over two
// roads side by side and the open ground around, each get what one thread gets alone.
TEST(WorldRoadGrade, SamplingOnThreadsAtOnceIsSamplingAlone) {
  road_network net;
  add_road(net, -100.0, 100.0, 0.0, 40, 6.0);
  add_road(net, -100.0, 100.0, 9.0, 40, 6.0);
  const auto ground = std::make_shared<fn_ground>(
      [](double x, double y) { return 4.0 + 0.05 * x + 2.0 * std::sin(0.07 * y); });
  road_grade_params p;
  const graded_terrain g(ground, net, grade_road_profile(net, *ground, p), p);
  const auto sweep = [&g](int k) {
    std::vector<double> out;
    for (double y = -30.0 + k; y <= 40.0; y += 0.9)
      for (double x = -120.0; x <= 120.0; x += 3.7)
        out.push_back(g.graded_at(x, y) ? g.sample(x, y) : -1e9 + g.sample(x, y));
    return out;
  };
  constexpr int kThreads = 4;
  std::vector<std::vector<double>> alone(kThreads), together(kThreads);
  for (int k = 0; k < kThreads; ++k)
    alone[k] = sweep(k);
  std::vector<std::thread> threads;
  for (int k = 0; k < kThreads; ++k)
    threads.emplace_back([&, k] { together[k] = sweep(k); });
  for (std::thread &t : threads)
    t.join();
  for (int k = 0; k < kThreads; ++k)
    EXPECT_EQ(together[k], alone[k]) << k;
}

// Two roads that pass close without meeting agree in height, so neither tilts toward the other
// and the ground between them does not step; a junction is near level.
TEST(WorldRoadGrade, CloseRoadsAgreeAndJunctionsAreLevel) {
  road_network net;
  const std::vector<int> a = add_road(net, -100.0, 100.0, 0.0, 20, 6.0);
  const std::vector<int> b = add_road(net, -100.0, 100.0, 8.0, 20, 6.0); // 2 m between edges
  const auto ground = std::make_shared<fn_ground>([](double, double y) { return 1.2 * y; });
  road_grade_params p;
  const std::vector<double> z = grade_road_profile(net, *ground, p);
  const double gap = 8.0 - 6.0 - 2.0 * p.flat_margin;
  for (std::size_t i = 0; i < a.size(); ++i)
    EXPECT_LE(std::abs(z[std::size_t(b[i])] - z[std::size_t(a[i])]),
              std::max(0.0, gap) * p.side_slope + 1e-3)
        << i;

  // A T: a side road leaving the middle of a road climbs no faster than junction_grade at first.
  road_network t;
  const std::vector<int> main = add_road(t, -50.0, 50.0, 0.0, 10, 6.0);
  const int mid = main[5];
  t.nodes[std::size_t(mid)].degree = 3;
  const int up = int(t.nodes.size());
  t.nodes.push_back(road_node{0.0, 10.0, 1});
  road_segment s;
  s.a = mid;
  s.b = up;
  s.width = 6.0;
  t.segments.push_back(s);
  const auto steep = std::make_shared<fn_ground>([](double, double y) { return 0.5 * y; });
  const std::vector<double> zt = grade_road_profile(t, *steep, p);
  EXPECT_LE(std::abs(zt[std::size_t(up)] - zt[std::size_t(mid)]) / 10.0, p.junction_grade + 1e-6);
}

// A road keeps to a gentle hill and cuts through a steep one, taking more off the crest than it
// raises at the foot.
TEST(WorldRoadGrade, AGentleHillIsFollowedAndASteepOneIsCut) {
  road_network net;
  const std::vector<int> ids = add_road(net, -300.0, 300.0, 0.0, 100, 6.0); // 6 m segments
  road_grade_params p;
  // A broad 4 m rise: its grade (5%) and its bend (0.0013 per metre) are within the limits.
  auto gentle = [](double x, double) { return 4.0 * std::exp(-x * x / 6400.0); };
  const std::vector<double> zg = grade_road_profile(net, fn_ground(gentle), p);
  for (int i : ids)
    EXPECT_NEAR(zg[std::size_t(i)], gentle(net.nodes[std::size_t(i)].x, 0.0), 0.05)
        << "followed as it is at x " << net.nodes[std::size_t(i)].x;
  // A 30 m hill climbing up to 30%: held to 12% it is cut through, deeper at the crest than any
  // bank at its foot.
  auto steep = [](double x, double) { return 30.0 * std::exp(-x * x / 3600.0); };
  const std::vector<double> zs = grade_road_profile(net, fn_ground(steep), p);
  double cut = 0.0, fill = 0.0, worst = 0.0;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const double d = zs[std::size_t(ids[i])] - steep(net.nodes[std::size_t(ids[i])].x, 0.0);
    cut = std::max(cut, -d);
    fill = std::max(fill, d);
    if (i > 0)
      worst =
          std::max(worst, std::abs(zs[std::size_t(ids[i])] - zs[std::size_t(ids[i - 1])]) / 6.0);
  }
  EXPECT_LE(worst, p.max_grade + 1e-6);
  EXPECT_GT(cut, 5.0) << "the crest is cut";
  EXPECT_LT(fill, 0.5 * cut) << "more cut at the crest than bank at the foot";
}

// Beside a road the ground keeps its own height unless it is above the cut or below the bank
// that open at side_slope from the road's edge: a road at grade along a ridge leaves the slopes
// either side as they are, and a road cut through a hill has the hill rise from its edge.
TEST(WorldRoadGrade, TheGroundBesideARoadIsCutOrBankedNeverDug) {
  road_network net;
  add_road(net, -100.0, 100.0, 0.0, 20, 6.0);
  road_grade_params p;
  const double flat = 3.0 + p.flat_margin;
  // A ridge along the road, level across its corridor and falling 40% either side of it.
  const auto ridge = std::make_shared<fn_ground>(
      [flat](double, double y) { return 20.0 - 0.4 * std::max(0.0, std::abs(y) - flat); });
  const graded_terrain gr(ridge, net, std::vector<double>(net.nodes.size(), 20.0), p);
  for (double y = flat; y < 50.0; y += 0.5)
    EXPECT_NEAR(gr.sample(10.0, y), 20.0 - 0.4 * (y - flat), 1e-9)
        << "the slope is untouched at " << y;
  // The same road 10 m down in a hill: its sides climb at side_slope until they meet the hill.
  const auto hill = std::make_shared<fn_ground>([](double, double) { return 30.0; });
  const graded_terrain gh(hill, net, std::vector<double>(net.nodes.size(), 20.0), p);
  EXPECT_NEAR(gh.sample(10.0, 0.0), 20.0, 1e-9);
  EXPECT_NEAR(gh.sample(10.0, flat), 20.0, 1e-9);
  for (double y = flat; y < 60.0; y += 0.5) {
    const double h = gh.sample(10.0, y);
    EXPECT_NEAR(h, std::min(30.0, 20.0 + p.side_slope * (y - flat)), 1e-9) << y;
    EXPECT_GE(h, 20.0) << "never below the road beside it";
  }
}

// Subdividing keeps every node where it was and splits each long segment evenly.
TEST(WorldRoadGrade, SubdivideSplitsLongSegmentsOnly) {
  road_network net;
  add_road(net, 0.0, 100.0, 0.0, 2, 6.0); // two 50 m segments
  net.segments[1].arterial = true;
  const road_network d = subdivide_roads(net, 8.0);
  ASSERT_EQ(d.segments.size(), 14u); // ceil(50 / 8) = 7 each
  for (std::size_t i = 0; i < net.nodes.size(); ++i) {
    EXPECT_EQ(d.nodes[i].x, net.nodes[i].x);
    EXPECT_EQ(d.nodes[i].y, net.nodes[i].y);
  }
  int arterial = 0;
  for (const road_segment &s : d.segments) {
    EXPECT_LE(std::hypot(d.nodes[std::size_t(s.b)].x - d.nodes[std::size_t(s.a)].x,
                         d.nodes[std::size_t(s.b)].y - d.nodes[std::size_t(s.a)].y),
              8.0 + 1e-9);
    EXPECT_EQ(s.width, 6.0);
    arterial += s.arterial;
  }
  EXPECT_EQ(arterial, 7);
  EXPECT_EQ(subdivide_roads(net, 60.0).segments.size(), 2u) << "short enough: unchanged";
}

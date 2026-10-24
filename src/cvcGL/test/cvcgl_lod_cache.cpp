// cvcgl_lod_cache -- LodPyramidCache: mesh LOD ladders built once per distinct
// mesh, off the frame, and handed to the LodGraphicsNodes that wait for them.
//
//   * one build per mesh -- two nodes showing two reads of the same asset share
//     one build, both get the rungs, and a later node is a cache hit;
//   * the key -- content, not storage or triangle order, plus every mesh-build
//     parameter;
//   * progressive attach -- rung 0 draws the moment a node is attached, and the
//     landing build keeps that very rung-0 node;
//   * deferred lanes (what a build without threads gets) -- exactly one build
//     per poll(), on the polling thread, and nothing between polls;
//   * teardown -- destroying the cache with builds queued cancels them instead
//     of running them;
//   * the build cap -- a mesh over it keeps its one rung and says so, once;
//   * waiting nodes are held weakly, and a re-attached node stops waiting for
//     its earlier mesh;
//   * a FEATURE-ON ring of bunnies, one build for all of them, coarsening with
//     distance.
//
// All of that is headless. The last section renders offscreen -- rung 0 on
// screen before the build lands, a styled coarse rung after -- and skips itself
// when a plain control node cannot be rasterised, unless CVC_REQUIRE_RENDER=1
// makes that a failure.
//
// NOT assert(): these tests build Release (NDEBUG), where assert() compiles to
// nothing and a "passing" run would prove nothing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/scene_realize.h> // fit_to_ground
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <vtkProp.h>

using cvc::gl::GeometryNode;
using cvc::gl::lod_stats;
using cvc::gl::LodGraphicsNode;
using cvc::gl::LodPyramidCache;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

int g_failures = 0;
void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

constexpr double kPi = 3.14159265358979323846;

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// An n x n heightfield over a `size` square at (x0, y0), with a gentle smooth
// relief so decimation has detail to remove: 2 n^2 triangles. A different
// (x0, y0) is a different mesh to the cache.
cvc::geometry grid_mesh(cvc::app &app, double x0, double y0, double size, int n) {
  cvc::geometry g(app);
  cvc::geometry::points_t &P = g.points();
  for (int j = 0; j <= n; ++j)
    for (int i = 0; i <= n; ++i) {
      const double u = double(i) / n, v = double(j) / n;
      const double z =
          size * (0.06 * std::sin(7.0 * u) * std::cos(5.0 * v) + 0.03 * std::sin(17.0 * (u + v)));
      P.push_back({x0 + size * u, y0 + size * v, z});
    }
  cvc::geometry::tris_t &T = g.tris();
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::uint64_t a = std::uint64_t(j) * (n + 1) + i, b = a + 1, c = a + (n + 1), d = c + 1;
      T.push_back({a, b, d});
      T.push_back({a, d, c});
    }
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

// The embedded Stanford bunny (69,473 triangles), stood up Z-up and 1.7 m tall
// the way a scene's `fit:` does, so its rung errors are in metres. Each call is
// a fresh read with its own storage.
bool g_bunny_ok = true;
cvc::geometry bunny(cvc::app &) {
  try {
    return cvc::gl::ariadne::fit_to_ground(cvc::read_geometry("x.bunny"), true, 1.7f);
  } catch (const std::exception &e) {
    std::printf("  cannot read the embedded bunny: %s\n", e.what());
    g_bunny_ok = false;
    return cvc::geometry();
  }
}

// Poll until every build has landed; false on a timeout.
bool wait_for_builds(LodPyramidCache &cache, double timeout_s = 240.0) {
  const auto t0 = std::chrono::steady_clock::now();
  for (;;) {
    cache.poll();
    if (cache.in_flight() == 0)
      return true;
    if (ms_since(t0) > 1000.0 * timeout_s)
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

// Which rung actor is visible (-1 none, -2 several).
int visible_rung(const LodGraphicsNode &lod) {
  int which = -1;
  for (int k = 0; k < lod.rungCount(); ++k)
    if (lod.rung(k)->prop()->GetVisibility())
      which = which == -1 ? k : -2;
  return which;
}

cvc::lod::view_params view_at(double x, double y, double z) {
  cvc::lod::view_params v = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.eye[0] = x;
  v.eye[1] = y;
  v.eye[2] = z;
  return v;
}

std::shared_ptr<LodGraphicsNode> add_lod(SceneGraph &sg, const std::string &name) {
  return sg.getGraphicsRoot()->addGraphicsChild<LodGraphicsNode>(name);
}

cvc::lod::pyramid_params grid_params() {
  cvc::lod::pyramid_params p;
  p.preserve_boundary = false; // a grid patch is nearly all boundary otherwise
  return p;
}

// --- one build per mesh --------------------------------------------------------

void test_same_mesh_builds_once(cvc::app &app, LodPyramidCache &cache) {
  std::printf("same mesh builds once\n");
  SceneGraph sg(app, "lcsame");
  auto a = add_lod(sg, "a");
  auto b = add_lod(sg, "b");
  const cvc::geometry m1 = bunny(app);
  const cvc::geometry m2 = bunny(app); // the same asset read again: its own storage
  CHECK(g_bunny_ok && m1.num_tris() == 69473);
  CHECK(LodPyramidCache::default_max_build_tris() >= m1.num_tris()); // this build builds it

  CHECK(!cache.attach(a, m1)); // a miss: rung 0 now, the rest when built
  CHECK(!cache.attach(b, m2)); // joins the build already started for m1
  CHECK(cache.builds() == 1 && cache.in_flight() == 1 && cache.cached() == 0);
  CHECK(a->rungCount() == 1 && b->rungCount() == 1);

  const auto t0 = std::chrono::steady_clock::now();
  CHECK(wait_for_builds(cache));
  std::printf("  bunny pyramid: %.0f ms on the lane (%.0f ms to land)\n", cache.last_build_ms(),
              ms_since(t0));
  CHECK(cache.builds() == 1 && cache.cached() == 1 && cache.in_flight() == 0);
  CHECK(a->rungCount() > 1 && b->rungCount() == a->rungCount());
  CHECK(a->rungTriangles(a->rungCount() - 1) < a->rungTriangles(0));

  // A later node with a third read of the bunny is a hit: the whole ladder at
  // once, no build.
  auto c = add_lod(sg, "c");
  CHECK(cache.attach(c, bunny(app)));
  CHECK(c->rungCount() == a->rungCount());
  CHECK(cache.builds() == 1 && cache.in_flight() == 0);
  CHECK(cache.poll() == 0);
  CHECK(cache.take_warnings().empty());
}

// --- the key ---------------------------------------------------------------------

void test_keys(cvc::app &app) {
  std::printf("keys: content + mesh-build params\n");
  const cvc::geometry g = grid_mesh(app, 0.0, 0.0, 10.0, 16);
  const cvc::lod::pyramid_params p = cvc::lod::pyramid_params();
  const std::uint64_t k = LodPyramidCache::key_of(g, p);
  CHECK(LodPyramidCache::key_of(g, p) == k);

  // Content, not storage or order: a deep copy, and the same triangles listed
  // backwards, are the same mesh.
  cvc::geometry deep(app);
  deep.copy(g, /*deepCopy=*/true);
  CHECK(LodPyramidCache::key_of(deep, p) == k);
  cvc::geometry reversed = deep;
  std::reverse(reversed.tris().begin(), reversed.tris().end());
  CHECK(LodPyramidCache::key_of(reversed, p) == k);

  // A centimetre away is another mesh.
  CHECK(LodPyramidCache::key_of(grid_mesh(app, 0.01, 0.0, 10.0, 16), p) != k);

  // Every parameter a mesh build reads is in the key...
  auto differs = [&](void (*edit)(cvc::lod::pyramid_params &)) {
    cvc::lod::pyramid_params q = p;
    edit(q);
    return LodPyramidCache::key_of(g, q) != k;
  };
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.max_rungs = 2; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.mesh_ratio = 0.5; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.mesh_min_tris = 128; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.preserve_boundary = false; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.weld_seams = false; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.seam_epsilon = 0.01; }));
  CHECK(differs([](cvc::lod::pyramid_params &q) { q.recompute_normals = true; }));
  // ...and the volume/image ones are not.
  CHECK(!differs([](cvc::lod::pyramid_params &q) {
    q.vol_factor = 3;
    q.img_factor = 4;
    q.img_world_per_texel = 2.0;
  }));

  // Two caches with different params build their own pyramids of one mesh.
  cvc::lod::pyramid_params coarse = p;
  coarse.mesh_ratio = 0.2;
  LodPyramidCache c1(p, LodPyramidCache::default_max_build_tris(), cvc::lane_mode::deferred);
  LodPyramidCache c2(coarse, LodPyramidCache::default_max_build_tris(), cvc::lane_mode::deferred);
  SceneGraph sg(app, "lckeys");
  CHECK(!c1.attach(add_lod(sg, "a"), g));
  CHECK(!c2.attach(add_lod(sg, "b"), g));
  CHECK(c1.builds() == 1 && c2.builds() == 1);
}

// --- progressive attach --------------------------------------------------------

void test_base_shown_before_build(cvc::app &app) {
  std::printf("rung 0 before the build\n");
  // Deferred, so the build cannot land behind the test's back.
  LodPyramidCache cache(grid_params(), LodPyramidCache::default_max_build_tris(),
                        cvc::lane_mode::deferred);
  SceneGraph sg(app, "lcbase");
  auto lod = add_lod(sg, "lod");
  const cvc::geometry g = grid_mesh(app, 0.0, 0.0, 10.0, 32);
  CHECK(!cache.attach(lod, g));
  CHECK(lod->rungCount() == 1 && lod->activeRung() == 0 && visible_rung(*lod) == 0);
  CHECK(lod->rungTriangles(0) == g.num_tris());
  CHECK(lod->rung(0)->getGeometry() && lod->rung(0)->getGeometry()->num_points() == g.num_points());
  CHECK(cache.in_flight() == 1 && cache.cached() == 0);
  // A deferred lane has no thread: nothing builds between polls.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  CHECK(lod->rungCount() == 1 && cache.in_flight() == 1);
  // A selection on a one-rung ladder has nowhere to go.
  CHECK(sg.selectLOD(view_at(0.0, 0.0, 1.0e6)) == 0 && visible_rung(*lod) == 0);
}

void test_rung0_kept_by_append(cvc::app &app) {
  std::printf("the landing build keeps rung 0\n");
  LodPyramidCache cache(grid_params(), LodPyramidCache::default_max_build_tris(),
                        cvc::lane_mode::deferred);
  SceneGraph sg(app, "lckeep");
  auto lod = add_lod(sg, "lod");
  int styled = 0;
  lod->setRungStyle([&styled](GeometryNode &g) {
    ++styled;
    g.setMetadata("styled", true);
  });
  const cvc::geometry g = grid_mesh(app, 0.0, 0.0, 10.0, 32);
  cache.attach(lod, g);
  const std::shared_ptr<GeometryNode> rung0 = lod->rung(0);
  vtkProp *const actor0 = rung0->prop();
  CHECK(styled == 1);

  CHECK(cache.poll() == 1);
  CHECK(lod->rungCount() > 1);
  CHECK(lod->rung(0) == rung0 && lod->rung(0)->prop() == actor0); // same node, same actor
  CHECK(styled == lod->rungCount()); // only the new rungs were styled, each once
  for (int k = 0; k < lod->rungCount(); ++k)
    CHECK(lod->rung(k)->hasMetadata("styled"));
  CHECK(lod->rungError(0) == 0.0 && lod->rungError(1) > 0.0);
  CHECK(lod->rungTriangles(1) < lod->rungTriangles(0));

  // The cached rung 0 is the mesh the node drew: a hit draws the same triangles.
  auto hit = add_lod(sg, "hit");
  CHECK(cache.attach(hit, g));
  CHECK(hit->rungCount() == lod->rungCount());
  CHECK(hit->rungTriangles(0) == lod->rungTriangles(0));
}

// --- deferred lanes ------------------------------------------------------------

void test_deferred_one_build_per_poll(cvc::app &app) {
  std::printf("deferred: one build per poll\n");
  LodPyramidCache cache(grid_params(), LodPyramidCache::default_max_build_tris(),
                        cvc::lane_mode::deferred);
  SceneGraph sg(app, "lcdefer");
  std::vector<std::shared_ptr<LodGraphicsNode>> nodes;
  for (int i = 0; i < 3; ++i) {
    nodes.push_back(add_lod(sg, "n" + std::to_string(i)));
    CHECK(!cache.attach(nodes.back(), grid_mesh(app, 20.0 * i, 0.0, 10.0, 32)));
  }
  CHECK(cache.builds() == 3 && cache.in_flight() == 3 && cache.cached() == 0);
  for (int k = 1; k <= 3; ++k) {
    CHECK(cache.poll() == 1);
    CHECK(cache.in_flight() == std::size_t(3 - k) && cache.cached() == std::size_t(k));
    // In attach order: the k-th node has its rungs, the next one not yet.
    CHECK(nodes[k - 1]->rungCount() > 1);
    if (k < 3)
      CHECK(nodes[k]->rungCount() == 1);
  }
  CHECK(cache.poll() == 0 && cache.builds() == 3);
}

// --- teardown --------------------------------------------------------------------

void test_destroy_with_queued_builds(cvc::app &app) {
  std::printf("teardown with builds queued\n");
  const int N = 8;
  const cvc::lod::pyramid_params p = grid_params();
  std::vector<cvc::geometry> meshes;
  for (int i = 0; i < N; ++i)
    meshes.push_back(grid_mesh(app, 100.0 * i, 0.0, 50.0, 96)); // 18,432 triangles each
  // What one of those builds costs here, now.
  auto t0 = std::chrono::steady_clock::now();
  const cvc::lod::mesh_pyramid probe = cvc::lod::build_mesh_pyramid(meshes[0], p, nullptr);
  const double one = ms_since(t0);
  CHECK(probe.rungs.size() > 1);

  for (const cvc::lane_mode mode : {cvc::lane_mode::deferred, cvc::lane_mode::threaded}) {
    const bool deferred = mode == cvc::lane_mode::deferred;
    SceneGraph sg(app, deferred ? "lcdown0" : "lcdown1");
    std::vector<std::shared_ptr<LodGraphicsNode>> nodes;
    auto cache =
        std::make_unique<LodPyramidCache>(p, LodPyramidCache::default_max_build_tris(), mode);
    for (int i = 0; i < N; ++i) {
      nodes.push_back(add_lod(sg, "n" + std::to_string(i)));
      cache->attach(nodes.back(), meshes[i]);
    }
    CHECK(cache->in_flight() == std::size_t(N));
    t0 = std::chrono::steady_clock::now();
    cache.reset();
    const double down = ms_since(t0);
    std::printf("  %s: destroyed with %d builds queued in %.1f ms (one build: %.1f ms)\n",
                deferred ? "deferred" : "threaded", N, down, one);
    // Deferred, nothing has run and nothing may: well under one build. Threaded,
    // at most the build the worker had already started, never the queue.
    CHECK(deferred ? down < one : down < 0.5 * N * one);
    for (const auto &n : nodes)
      CHECK(n->rungCount() == 1); // no build landed on a dead cache
  }
}

// --- the build cap ---------------------------------------------------------------

void test_over_cap(cvc::app &app) {
  std::printf("build cap\n");
  CHECK(LodPyramidCache::default_max_build_tris() ==
        (cvc::async_lane_has_threads ? 2000000u : 60000u));
  LodPyramidCache cache(grid_params(), /*max_build_tris=*/1000, cvc::lane_mode::deferred);
  SceneGraph sg(app, "lccap");
  const cvc::geometry big = grid_mesh(app, 0.0, 0.0, 10.0, 32); // 2,048 triangles
  auto lod = add_lod(sg, "big");
  CHECK(!cache.attach(lod, big));
  CHECK(lod->rungCount() == 1 && visible_rung(*lod) == 0);
  CHECK(cache.builds() == 0 && cache.in_flight() == 0 && cache.cached() == 0);
  std::vector<std::string> w = cache.take_warnings();
  CHECK(w.size() == 1);
  if (!w.empty()) {
    std::printf("  warning: %s\n", w[0].c_str());
    CHECK(w[0].find("2048") != std::string::npos && w[0].find("1000") != std::string::npos);
  }
  CHECK(cache.take_warnings().empty()); // taken
  CHECK(cache.poll() == 0 && lod->rungCount() == 1);

  // The same mesh again: one rung, no second warning, still no build.
  auto again = add_lod(sg, "again");
  CHECK(!cache.attach(again, big));
  CHECK(again->rungCount() == 1 && cache.take_warnings().empty() && cache.builds() == 0);

  // Under the cap builds as usual.
  auto small = add_lod(sg, "small");
  CHECK(!cache.attach(small, grid_mesh(app, 0.0, 0.0, 10.0, 16))); // 512 triangles
  CHECK(cache.builds() == 1 && cache.poll() == 1 && small->rungCount() > 1);
}

// --- waiting nodes -------------------------------------------------------------

void test_waiters(cvc::app &app) {
  std::printf("waiting nodes: weak, and re-attach forgets\n");
  LodPyramidCache cache(grid_params(), LodPyramidCache::default_max_build_tris(),
                        cvc::lane_mode::deferred);
  // A node destroyed before its build lands: the build still lands, for later.
  {
    auto gone = std::make_shared<LodGraphicsNode>(app, "test.gone", "gone");
    cache.attach(gone, grid_mesh(app, 0.0, 0.0, 10.0, 32));
  }
  CHECK(cache.in_flight() == 1);
  CHECK(cache.poll() == 1 && cache.cached() == 1);

  // A node attached to B, then to C, shows C: B's landing passes it by.
  SceneGraph sg(app, "lcwait");
  auto lod = add_lod(sg, "lod");
  const cvc::geometry b = grid_mesh(app, 20.0, 0.0, 10.0, 32);
  const cvc::geometry c = grid_mesh(app, 40.0, 0.0, 10.0, 32);
  cache.attach(lod, b);
  cache.attach(lod, c);
  CHECK(cache.builds() == 3 && cache.in_flight() == 2);
  auto shows_c = [&]() {
    const cvc::geometry *r0 = lod->rung(0) ? lod->rung(0)->getGeometry() : nullptr;
    return r0 && r0->num_points() == c.num_points() &&
           r0->const_points()[0][0] == c.const_points()[0][0];
  };
  CHECK(shows_c());
  CHECK(cache.poll() == 1); // B
  CHECK(lod->rungCount() == 1 && shows_c());
  CHECK(cache.poll() == 1); // C
  CHECK(lod->rungCount() > 1 && shows_c());
  CHECK(cache.cached() == 3);
}

// --- FEATURE-ON: a ring of bunnies -----------------------------------------------

void test_bunny_ring(cvc::app &app, LodPyramidCache &cache) {
  std::printf("feature-on: 25 bunnies on a 60 m ring\n");
  if (!g_bunny_ok) {
    CHECK(false);
    return;
  }
  SceneGraph sg(app, "lcring");
  const cvc::geometry b = bunny(app);
  const int N = 25;
  const double radius = 30.0;
  std::vector<std::shared_ptr<LodGraphicsNode>> nodes;
  for (int i = 0; i < N; ++i) {
    auto n = add_lod(sg, "bunny" + std::to_string(i));
    const double a = 2.0 * kPi * i / N;
    n->setPosition(radius * std::cos(a), radius * std::sin(a), 0.0);
    cache.attach(n, b);
    nodes.push_back(n);
  }
  CHECK(wait_for_builds(cache));
  CHECK(cache.builds() == 1); // the first test's build, for every instance

  // From eye height at the centre, every bunny is 30 m away.
  lod_stats st;
  sg.selectLOD(view_at(0.0, 0.0, 1.7), &st);
  CHECK(st.nodes == N && st.hidden == 0);
  CHECK(st.full_tris == std::uint64_t(N) * b.num_tris());
  CHECK(st.drawn_tris < st.full_tris);
  std::printf("  centre: %llu of %llu tris (%.3f)\n", (unsigned long long)st.drawn_tris,
              (unsigned long long)st.full_tris, double(st.drawn_tris) / double(st.full_tris));

  // Beside the first bunny: it alone goes back to full detail.
  sg.selectLOD(view_at(radius + 1.0, 0.0, 1.0), &st);
  CHECK(nodes[0]->selectedRung() == 0);
  CHECK(nodes[N / 2]->selectedRung() > 0);
  CHECK(st.drawn_tris < st.full_tris && st.drawn_tris >= b.num_tris());
}

// --- offscreen renders -------------------------------------------------------------

long red_pixels(SceneRenderer &sr, long *lit = nullptr) {
  long red = 0, any = 0;
  const std::vector<unsigned char> px = sr.frameRGB();
  for (std::size_t i = 0; i + 2 < px.size(); i += 3) {
    const int r = px[i], g = px[i + 1], b = px[i + 2];
    if (r + g + b > 60)
      ++any;
    if (r > 150 && g < 80 && b < 80)
      ++red;
  }
  if (lit)
    *lit = any;
  return red;
}

void flat_red(GeometryNode &g) {
  g.setColor(1.0, 0.0, 0.0);
  g.setAmbient(1.0); // flat: no dependence on the lights
  g.setDiffuse(0.0);
  g.setSpecular(0.0);
}

// Asked of a CONTROL -- a plain GeometryNode in a scene of its own -- never of
// the LOD node under test, whose regression would otherwise look like "no GL".
bool can_rasterise(cvc::app &app) {
  SceneGraph sg(app, "lccontrol");
  sg.setDiagnosticChromeVisible(false);
  auto node = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("control");
  node->setGeometry(grid_mesh(app, -2.0, -2.0, 4.0, 4));
  flat_red(*node);
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(0, -6, 6, 0, 0, 0, 0, 0, 1, 40.0, 0.5, 100.0);
  long lit = 0;
  red_pixels(sr, &lit);
  return lit > 0;
}

void test_offscreen(cvc::app &app) {
  std::printf("offscreen render\n");
  if (!can_rasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("  CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      CHECK(false);
    } else {
      std::printf("  skipped: this build did not rasterise (the plain control drew nothing)\n");
    }
    return;
  }
  // The default params pin the patch's border, so every rung covers the same pixels.
  LodPyramidCache cache(cvc::lod::pyramid_params(), LodPyramidCache::default_max_build_tris(),
                        cvc::lane_mode::deferred);
  SceneGraph sg(app, "lcrender");
  sg.setDiagnosticChromeVisible(false);
  auto lod = add_lod(sg, "lod");
  lod->setRungStyle(flat_red);
  cache.attach(lod, grid_mesh(app, -2.0, -2.0, 4.0, 32));
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(0, -6, 6, 0, 0, 0, 0, 0, 1, 40.0, 0.5, 100.0);

  // Before the build: rung 0 is on screen.
  const long before = red_pixels(sr);
  CHECK(before > 0);
  // After it, with the selection forced coarse: a styled coarse rung draws in
  // its place, covering about the same pixels (the border is pinned; only the
  // relief inside it is coarser).
  CHECK(cache.poll() == 1 && lod->rungCount() > 1);
  sg.selectLOD(view_at(0.0, 0.0, 1.0e6));
  CHECK(lod->selectedRung() > 0 && visible_rung(*lod) == lod->selectedRung());
  const long after = red_pixels(sr);
  std::printf("  red pixels: rung 0 %ld, rung %d %ld\n", before, lod->selectedRung(), after);
  CHECK(after > 0 && std::abs(after - before) * 5 <= before);
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  // One threaded cache for the bunny, so the ring reuses the first test's build.
  LodPyramidCache bunnies;
  test_same_mesh_builds_once(app, bunnies);
  test_keys(app);
  test_base_shown_before_build(app);
  test_rung0_kept_by_append(app);
  test_deferred_one_build_per_poll(app);
  test_destroy_with_queued_builds(app);
  test_over_cap(app);
  test_waiters(app);
  test_bunny_ring(app, bunnies);
  test_offscreen(app);
  std::printf("%s: cvcgl_lod_cache (%d check%s failed)\n", g_failures == 0 ? "PASS" : "FAIL",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}

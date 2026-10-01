// SceneGraph::computeGraphicsBounds -- the world-bounds walk behind the grid --
// made cheaper without changing its answer, and skipped while nothing shows it.
//
// The walk ran, per root child: a dynamic_pointer_cast<LightNode>, a
// dynamic_cast<NullGraphicNode> inside getCombinedBoundingBox, and a
// vtkMatrix4x4 New + DeepCopy (getWorldTransform) to push 8 corners through.
// In the demo3 wasm profile one walk over ~110 nodes cost 0.6-1 ms in Firefox,
// ~27% of it the matrix copies and ~23% the casts. Now: flags instead of casts,
// and the node's cached world matrix (getCombinedWorldBoundingBox).
//
// Pins (headless):
//   A. the result is BIT-IDENTICAL to the old algorithm (replicated here) over
//      randomised hierarchies: rotations, scales, translations, nesting,
//      NullGraphicNode groups with and without their own box, lights;
//   B. a grow-only walk is skipped while the grid, the axis and the root's own
//      box are all hidden, and runs when any is shown -- through the
//      SceneGraph setters or not (the grid node itself, setShowBBox) -- so the
//      grid and the root box still enclose the content;
//   C. with the grid shown, moves still cost one walk per pump.
//   D. (informational) old vs new walk time.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/AxisNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <limits>
#include <random>
#include <string>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::LightNode;
using cvc::gl::NullGraphicNode;
using cvc::gl::SceneGraph;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

// ---- the walk as it was (SceneGraph.cpp / GraphicsNode.cpp before) ----------
static cvc::bounding_box oldCombined(const GraphicsNode &n) {
  const NullGraphicNode *nullNode = dynamic_cast<const NullGraphicNode *>(&n);
  bool includeOwnBounds = true;
  if (nullNode)
    includeOwnBounds = nullNode->getIncludeOwnBounds();
  double acc_minx = std::numeric_limits<double>::max(), acc_miny = acc_minx, acc_minz = acc_minx;
  double acc_maxx = std::numeric_limits<double>::lowest(), acc_maxy = acc_maxx, acc_maxz = acc_maxx;
  if (includeOwnBounds) {
    cvc::bounding_box own = n.getBoundingBox();
    acc_minx = own[0];
    acc_miny = own[1];
    acc_minz = own[2];
    acc_maxx = own[3];
    acc_maxy = own[4];
    acc_maxz = own[5];
  }
  for (const auto &child : n.getGraphicsChildren()) {
    if (!child)
      continue;
    cvc::bounding_box c = oldCombined(*child);
    if (c[0] > c[3] || c[1] > c[4] || c[2] > c[5])
      continue;
    vtkMatrix4x4 *t = child->getTransform();
    double corners[8][3] = {{c[0], c[1], c[2]}, {c[3], c[1], c[2]}, {c[0], c[4], c[2]},
                            {c[3], c[4], c[2]}, {c[0], c[1], c[5]}, {c[3], c[1], c[5]},
                            {c[0], c[4], c[5]}, {c[3], c[4], c[5]}};
    double minx = std::numeric_limits<double>::max(), miny = minx, minz = minx;
    double maxx = std::numeric_limits<double>::lowest(), maxy = maxx, maxz = maxx;
    for (int i = 0; i < 8; ++i) {
      double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0}, out[4];
      t->MultiplyPoint(in, out);
      minx = std::min(minx, out[0]);
      miny = std::min(miny, out[1]);
      minz = std::min(minz, out[2]);
      maxx = std::max(maxx, out[0]);
      maxy = std::max(maxy, out[1]);
      maxz = std::max(maxz, out[2]);
    }
    acc_minx = std::min(acc_minx, minx);
    acc_miny = std::min(acc_miny, miny);
    acc_minz = std::min(acc_minz, minz);
    acc_maxx = std::max(acc_maxx, maxx);
    acc_maxy = std::max(acc_maxy, maxy);
    acc_maxz = std::max(acc_maxz, maxz);
  }
  if (acc_minx > acc_maxx || acc_miny > acc_maxy || acc_minz > acc_maxz)
    return cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
  return cvc::bounding_box(acc_minx, acc_miny, acc_minz, acc_maxx, acc_maxy, acc_maxz);
}

static cvc::bounding_box oldWalk(SceneGraph &sg) {
  cvc::bounding_box combined;
  bool first = true;
  for (const auto &child : sg.getGraphicsRoot()->getGraphicsChildren()) {
    if (!child)
      continue;
    if (child.get() == static_cast<GraphicsNode *>(sg.getGridNode().get()) ||
        child.get() == static_cast<GraphicsNode *>(sg.getAxisNode().get()))
      continue;
    if (std::dynamic_pointer_cast<LightNode>(child))
      continue;
    cvc::bounding_box c = oldCombined(*child);
    if (c[0] > c[3] || c[1] > c[4] || c[2] > c[5])
      continue;
    vtkSmartPointer<vtkMatrix4x4> w = child->getWorldTransform();
    double corners[8][3] = {{c[0], c[1], c[2]}, {c[3], c[1], c[2]}, {c[0], c[4], c[2]},
                            {c[3], c[4], c[2]}, {c[0], c[1], c[5]}, {c[3], c[1], c[5]},
                            {c[0], c[4], c[5]}, {c[3], c[4], c[5]}};
    double minx = std::numeric_limits<double>::max(), miny = minx, minz = minx;
    double maxx = std::numeric_limits<double>::lowest(), maxy = maxx, maxz = maxx;
    for (int i = 0; i < 8; ++i) {
      double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0}, out[4];
      w->MultiplyPoint(in, out);
      minx = std::min(minx, out[0]);
      miny = std::min(miny, out[1]);
      minz = std::min(minz, out[2]);
      maxx = std::max(maxx, out[0]);
      maxy = std::max(maxy, out[1]);
      maxz = std::max(maxz, out[2]);
    }
    if (first) {
      combined = cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz);
      first = false;
    } else {
      combined[0] = std::min(combined[0], minx);
      combined[1] = std::min(combined[1], miny);
      combined[2] = std::min(combined[2], minz);
      combined[3] = std::max(combined[3], maxx);
      combined[4] = std::max(combined[4], maxy);
      combined[5] = std::max(combined[5], maxz);
    }
  }
  return combined;
}

// ---- scenes ----------------------------------------------------------------
static cvc::geometry box(double x0, double y0, double z0, double s) {
  cvc::geometry g;
  for (int k = 0; k < 8; ++k)
    g.points().push_back({x0 + s * (k & 1), y0 + s * ((k >> 1) & 1), z0 + s * ((k >> 2) & 1)});
  g.tris() = {{{0, 1, 3}}, {{0, 3, 2}}, {{4, 7, 5}}, {{4, 6, 7}}};
  return g;
}

// A random affine pose: rotation (three Euler angles), per-axis scale, translation.
static void randomPose(std::mt19937 &rng, double m[16]) {
  std::uniform_real_distribution<double> ang(-3.14159, 3.14159), sc(0.25, 3.0), tr(-500.0, 500.0);
  const double a = ang(rng), b = ang(rng), c = ang(rng);
  const double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b);
  const double cc = std::cos(c), sc_ = std::sin(c);
  const double R[3][3] = {{cb * cc, -cb * sc_, sb},
                          {sa * sb * cc + ca * sc_, -sa * sb * sc_ + ca * cc, -sa * cb},
                          {-ca * sb * cc + sa * sc_, ca * sb * sc_ + sa * cc, ca * cb}};
  const double s[3] = {sc(rng), sc(rng), sc(rng)};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      m[i * 4 + j] = R[i][j] * s[j];
    m[i * 4 + 3] = tr(rng);
  }
  m[12] = m[13] = m[14] = 0.0;
  m[15] = 1.0;
}

static void buildRandom(SceneGraph &sg, std::mt19937 &rng, int nodes) {
  std::uniform_int_distribution<int> kind(0, 9);
  std::uniform_real_distribution<double> pos(-100.0, 100.0), size(0.5, 20.0);
  std::vector<std::shared_ptr<GraphicsNode>> parents{sg.getGraphicsRoot()};
  for (int i = 0; i < nodes; ++i) {
    std::uniform_int_distribution<std::size_t> pick(0, parents.size() - 1);
    std::shared_ptr<GraphicsNode> parent = parents[pick(rng)];
    const std::string name = "n" + std::to_string(i);
    const int k = kind(rng);
    std::shared_ptr<GraphicsNode> node;
    if (k == 0 && parent == sg.getGraphicsRoot()) {
      node = sg.addLight(name); // a light at the root, far out
      node->setPosition(5000.0, -5000.0, 3000.0);
      continue;
    } else if (k <= 2) {
      auto g = parent->addGraphicsChild<NullGraphicNode>(name);
      g->setIncludeOwnBounds(k == 2);
      node = g;
    } else {
      auto g = parent->addGraphicsChild<GeometryNode>(name);
      g->setGeometry(box(pos(rng), pos(rng), pos(rng), size(rng)));
      node = g;
    }
    double m[16];
    randomPose(rng, m);
    node->setTransform(m);
    parents.push_back(node);
  }
}

static bool same(const cvc::bounding_box &a, const cvc::bounding_box &b) {
  for (int i = 0; i < 6; ++i)
    if (!(a[i] == b[i]))
      return false;
  return true;
}

static bool encloses(const cvc::bounding_box &outer, const cvc::bounding_box &inner) {
  for (int i = 0; i < 3; ++i)
    if (inner[i] < outer[i] - 1e-9 || inner[i + 3] > outer[i + 3] + 1e-9)
      return false;
  return true;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  std::printf("A. bit-identical to the old walk\n");
  {
    std::mt19937 rng(20261001);
    int identical = 0, scenes = 40;
    for (int sIdx = 0; sIdx < scenes; ++sIdx) {
      SceneGraph sg(app, "walk" + std::to_string(sIdx));
      buildRandom(sg, rng, 30 + sIdx);
      sg.processEvents();
      if (same(sg.computeGraphicsBounds(), oldWalk(sg)))
        ++identical;
      else
        std::printf("    scene %d differs\n", sIdx);
      // and every node's own combined box
      bool nodesSame = true;
      for (const auto &n : sg.getAllGraphicsOfType<GraphicsNode>())
        if (n && !same(n->getCombinedBoundingBox(), oldCombined(*n)))
          nodesSame = false;
      if (!nodesSame)
        std::printf("    scene %d: a node's combined box differs\n", sIdx);
      identical -= nodesSame ? 0 : 1;
    }
    chk(identical == scenes, std::to_string(identical) + "/" + std::to_string(scenes) +
                                 " random scenes give the same bounds, bit for bit");
  }
  {
    SceneGraph sg(app, "walklights");
    sg.addGraphics("a", box(0, 0, 0, 10));
    auto light = sg.addLight("sun");
    light->setPosition(1.0e5, 1.0e5, 1.0e5);
    sg.processEvents();
    const cvc::bounding_box b = sg.computeGraphicsBounds();
    chk(b[3] <= 10.0 && b[5] <= 10.0, "a far light still does not stretch the bounds");
    auto group = sg.getGraphicsRoot()->addGraphicsChild<NullGraphicNode>("frame");
    group->setBounds(-50, -50, -50, 50, 50, 50);
    group->setSyncBoundsWithChildren(false);
    group->setIncludeOwnBounds(false);
    chk(same(sg.computeGraphicsBounds(), oldWalk(sg)), "a group framing only its children: same");
    group->setIncludeOwnBounds(true);
    chk(sg.computeGraphicsBounds()[0] == -50.0 && same(sg.computeGraphicsBounds(), oldWalk(sg)),
        "and one including its own box: same");
  }

  std::printf("B. no walk while the grid and axis are hidden\n");
  {
    SceneGraph sg(app, "walkhidden");
    auto a = sg.addGraphics("a", box(0, 0, 0, 10));
    auto b = sg.addGraphics("b", box(20, 0, 0, 10));
    sg.processEvents();
    sg.setDiagnosticChromeVisible(false);
    const std::uint64_t w0 = sg.boundsWalkCount();
    for (int f = 0; f < 10; ++f) {
      a->setPosition(100.0 * f, 0.0, 0.0);
      b->setPosition(0.0, -50.0 * f, 0.0);
      sg.processEvents();
    }
    chk(sg.boundsWalkCount() == w0, "10 pumps of moves under a hidden grid: " +
                                        std::to_string(sg.boundsWalkCount() - w0) + " walks");
    sg.setGridVisible(true);
    chk(sg.boundsWalkCount() == w0 + 1, "showing the grid walks once");
    chk(encloses(sg.getGridNode()->bounds(), sg.computeGraphicsBounds()),
        "and the grid encloses the content");
    sg.processEvents();
    const std::uint64_t wShown = sg.boundsWalkCount();
    sg.setAxisVisible(true);
    chk(sg.boundsWalkCount() == wShown, "nothing deferred: showing the axis adds no walk");

    std::printf("C. with the grid shown, a pump of moves is one walk\n");
    const std::uint64_t w1 = sg.boundsWalkCount();
    a->setPosition(-300.0, 0.0, 0.0);
    b->setPosition(0.0, 400.0, 0.0);
    sg.processEvents();
    chk(sg.boundsWalkCount() == w1 + 1, "one walk");
    chk(encloses(sg.getGridNode()->bounds(), sg.computeGraphicsBounds()), "grid follows");

    sg.setAxisVisible(false);
    sg.setGridVisible(false);
    a->setPosition(-900.0, 0.0, 0.0);
    sg.processEvents();
    sg.setAxisVisible(true); // the axis alone also needs the box
    chk(encloses(sg.getGridNode()->bounds(), sg.computeGraphicsBounds()),
        "showing the axis catches the box up too");
  }

  std::printf("B2. however the box is shown again\n");
  {
    SceneGraph sg(app, "walkcatchup");
    auto a = sg.addGraphics("a", box(0, 0, 0, 10));
    sg.processEvents();
    // Grid and axis hidden, the root's own box still shown: it follows moves.
    sg.setGridVisible(false);
    sg.setAxisVisible(false);
    chk(sg.getGraphicsRoot()->getShowBBox(), "the root box is shown by default");
    a->setPosition(200.0, 0.0, 0.0);
    sg.processEvents();
    chk(encloses(sg.getGraphicsRoot()->getBoundingBox(), sg.computeGraphicsBounds()),
        "with only the root box shown, the root bounds follow a move");
    const std::string rootBounds =
        cvc::state::instance(app)("walkcatchup.graphics.root.bounds").value();
    chk(rootBounds.find("210") != std::string::npos,
        "and so does its state key (" + rootBounds + ")");

    // Everything hidden: deferred.
    sg.getGraphicsRoot()->setShowBBox(false);
    const std::uint64_t w0 = sg.boundsWalkCount();
    a->setPosition(-400.0, 0.0, 0.0);
    sg.processEvents();
    chk(sg.boundsWalkCount() == w0, "all hidden: no walk");
    // Shown through the node, not SceneGraph::setGridVisible: the pump catches up.
    sg.getGridNode()->setVisible(true);
    sg.processEvents();
    chk(sg.boundsWalkCount() == w0 + 1, "the grid node shown directly: the pump walks once");
    chk(encloses(sg.getGridNode()->bounds(), sg.computeGraphicsBounds()),
        "and the grid encloses the content");

    sg.getGridNode()->setVisible(false);
    a->setPosition(0.0, 600.0, 0.0);
    sg.processEvents();
    sg.getGraphicsRoot()->setShowBBox(true); // the root box alone
    sg.processEvents();
    chk(encloses(sg.getGraphicsRoot()->getBoundingBox(), sg.computeGraphicsBounds()),
        "showing the root box catches its bounds up");
  }

  std::printf("D. cost (informational)\n");
  {
    SceneGraph sg(app, "walkbench");
    for (int i = 0; i < 110; ++i) {
      auto n = sg.addGraphics("v" + std::to_string(i), box(0, 0, 0, 4));
      n->setPosition(10.0 * i, 3.0 * i, 0.0);
    }
    sg.processEvents();
    const int reps = 2000;
    cvc::bounding_box sink;
    auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < reps; ++r)
      sink = oldWalk(sg);
    auto t1 = std::chrono::steady_clock::now();
    for (int r = 0; r < reps; ++r)
      sink = sg.computeGraphicsBounds();
    auto t2 = std::chrono::steady_clock::now();
    auto us = [&](auto a, auto b) {
      return std::chrono::duration<double, std::micro>(b - a).count() / reps;
    };
    std::printf("  110 nodes: old walk %.1f us, new walk %.1f us (%.3f)\n", us(t0, t1), us(t1, t2),
                sink[0]);
    chk(same(sg.computeGraphicsBounds(), oldWalk(sg)), "bench scene: same bounds");
  }

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}

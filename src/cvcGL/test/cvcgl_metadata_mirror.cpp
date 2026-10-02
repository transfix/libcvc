// GraphicsNode::setMetadata keeps a READ-ONLY mirror of the node's metadata in
// the state tree ("<node>.metadata.<key>"), so a state browser can show it but a
// dashboard or script cannot edit it (the node never reads it back).
//
// The mirror used to be locked against its own owner: after the first write
// every later setMetadata threw read_only_error, which was swallowed. The tree
// kept the FIRST value forever, and each setGeometry paid for ~17 exceptions --
// about 40% of a wall re-mesh in the demo3 wasm profile. This pins:
//   A. the mirror follows the node: a second setGeometry updates it, and it
//      stays read-only to everyone else;
//   B. an unchanged value costs nothing: no state write, no signal;
//   C. Python's int (a long) and the other integer/float types are mirrored;
//   D. a metadata change asks for no redraw (it changes nothing drawn).
// Headless: no GL context.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <string>
#include <vector>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

// n x n grid of points as quads: 4 + ... vertices, so the count is controllable.
static cvc::geometry grid(int n, double scale) {
  cvc::geometry g;
  for (int j = 0; j <= n; ++j)
    for (int i = 0; i <= n; ++i)
      g.points().push_back({scale * i, scale * j, 0.0});
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const unsigned a = static_cast<unsigned>(j * (n + 1) + i);
      g.tris().push_back({a, a + 1, a + static_cast<unsigned>(n) + 2});
      g.tris().push_back({a, a + static_cast<unsigned>(n) + 2, a + static_cast<unsigned>(n) + 1});
    }
  return g;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::state &root = cvc::state::instance(app);

  std::printf("A. the mirror follows the node and stays locked to others\n");
  GeometryNode node(app, "test.meta", "meta");
  node.setGeometry(grid(1, 1.0)); // 4 vertices
  cvc::state &nv = root("test.meta.metadata.num_vertices");
  chk(nv.value() == "4", "first setGeometry mirrored num_vertices = 4 (got " + nv.value() + ")");
  chk(nv.readOnly(), "mirror is read-only");

  node.setGeometry(grid(2, 1.0)); // 9 vertices
  chk(nv.value() == "9",
      "second setGeometry UPDATED the mirror to 9 (got " + nv.value() + "; was stuck at 4)");
  chk(nv.readOnly(), "and it is read-only again");
  chk(root("test.meta.metadata.bbox_max_x").value<double>() == 2.0, "bbox_max_x followed too");

  bool threw = false;
  try {
    nv.value(std::string("123"));
  } catch (const cvc::read_only_error &) {
    threw = true;
  }
  chk(threw && nv.value() == "9", "an outside write is refused, value unchanged");

  node.setMetadata("label", std::string("first"));
  node.setMetadata("label", std::string("second"));
  chk(root("test.meta.metadata.label").value() == "second", "string metadata updates");
  node.setMetadata("cstr", "abc");
  node.setMetadata("cstr", "abd");
  chk(root("test.meta.metadata.cstr").value() == "abd", "const char* metadata updates");

  std::printf("B. an unchanged value costs nothing\n");
  int signals = 0;
  boost::signals2::scoped_connection conn =
      root("test.meta.metadata").childChanged.connect([&](const std::string &) { ++signals; });
  const auto mod0 = nv.lastMod();
  node.setGeometry(grid(2, 1.0)); // identical mesh: every key unchanged
  chk(signals == 0, "re-setting an identical mesh fired " + std::to_string(signals) +
                        " metadata signals (want 0)");
  chk(nv.lastMod() == mod0, "and wrote nothing");
  node.setMetadata("label", std::string("second"));
  chk(signals == 0, "unchanged setMetadata fired nothing");
  node.setGeometry(grid(3, 1.0)); // 16 vertices: some keys change
  chk(signals > 0 && nv.value() == "16", "a changed mesh does signal, and lands (16)");

  std::printf("C. integer / float types are mirrored\n");
  node.setMetadata("py_int", std::any(7L)); // what pycvc's set_metadata passes for an int
  chk(root("test.meta.metadata.py_int").value() == "7", "long mirrored");
  node.setMetadata("py_int", std::any(8L));
  chk(root("test.meta.metadata.py_int").value() == "8", "long updates");
  node.setMetadata("count", std::any(static_cast<unsigned long>(5)));
  chk(root("test.meta.metadata.count").value() == "5", "unsigned long mirrored");
  node.setMetadata("ratio", std::any(0.5f));
  chk(root("test.meta.metadata.ratio").value<double>() == 0.5, "float mirrored");
  node.setMetadata("opaque", std::any(std::vector<int>{1, 2}));
  const std::vector<std::string> mirrored = root("test.meta.metadata").children();
  chk(node.hasMetadata("opaque") &&
          std::find(mirrored.begin(), mirrored.end(), "opaque") == mirrored.end(),
      "an unmirrorable type stays on the node only");

  std::printf("D. metadata asks for no redraw\n");
  {
    SceneGraph sg(app, "metascene");
    auto g = sg.addGraphics("g", grid(1, 1.0));
    sg.processEvents();
    (void)sg.checkAndResetRenderNeeded();
    g->setMetadata("note", std::string("x"));
    g->setMetadata("note", std::string("y"));
    chk(!sg.checkAndResetRenderNeeded(), "setMetadata requested no render");
  }

  std::printf("E. cost (informational)\n");
  {
    GeometryNode bench(app, "test.metabench", "bench");
    const int keys = 17, reps = 200;
    for (int k = 0; k < keys; ++k)
      bench.setMetadata("k" + std::to_string(k), 0.0);
    auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < reps; ++r)
      for (int k = 0; k < keys; ++k)
        bench.setMetadata("k" + std::to_string(k), 0.0); // unchanged
    auto t1 = std::chrono::steady_clock::now();
    for (int r = 0; r < reps; ++r)
      for (int k = 0; k < keys; ++k)
        bench.setMetadata("k" + std::to_string(k), static_cast<double>(r + 1)); // changed
    auto t2 = std::chrono::steady_clock::now();
    auto us = [](auto a, auto b) {
      return std::chrono::duration<double, std::micro>(b - a).count();
    };
    std::printf("  unchanged: %.2f us/key   changed: %.2f us/key\n", us(t0, t1) / (keys * reps),
                us(t1, t2) / (keys * reps));
    chk(root("test.metabench.metadata.k3").value<double>() == reps, "bench keys landed");
  }

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}

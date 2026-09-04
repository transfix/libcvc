// Not a test: measures what it costs to POSE a deep graphics hierarchy.
//
// The shape is taken from the volrover3 lsystem_forest example, which is what
// exposed the problem: 70 trees, each an L-system expanded into ~21 modules of
// its own GeometryNode, nested about four levels deep. Animating that meant
// calling setTransform on the tree roots every frame.
//
// The cost that used to dominate was not the number of nodes posed, it was that
// every descendant reached by the cascade called getWorldTransform(), which
// recursed back UP to the root re-multiplying the whole chain and allocating a
// vtkMatrix4x4 at every level. A subtree pose was therefore O(nodes * depth) in
// both multiplies and allocations. GraphicsNode now caches the world matrix and
// refreshes it top-down during the cascade, so it is O(nodes).
//
// Build the target and run it to re-measure on any machine, rather than trusting
// a number quoted in a commit message.
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <memory>
#include <string>
#include <vector>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

namespace {

constexpr int TREES = 70;
constexpr int BRANCH = 3; // children per module
constexpr int DEPTH = 4;  // module levels, as the forest's maturities average

cvc::geometry little_mesh() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  const double xyz[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  for (auto &v : xyz) {
    p[0] = v[0];
    p[1] = v[1];
    p[2] = v[2];
    g.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  g.tris().push_back(t);
  return g;
}

void grow(const std::shared_ptr<GraphicsNode> &parent, const cvc::geometry &mesh, int depth,
          int &counter, std::vector<std::shared_ptr<GraphicsNode>> &all) {
  if (depth <= 0)
    return;
  for (int i = 0; i < BRANCH; ++i) {
    auto child = parent->addGraphicsChild<GeometryNode>("m" + std::to_string(counter++));
    child->setGeometry(mesh);
    child->setPosition(0.0, 1.0, 0.0);
    all.push_back(child);
    grow(child, mesh, depth - 1, counter, all);
  }
}

double time_ms(const std::vector<std::shared_ptr<GraphicsNode>> &nodes, int reps) {
  auto t0 = std::chrono::steady_clock::now();
  for (int r = 0; r < reps; ++r) {
    const double a = 0.001 * (r % 5);
    for (const auto &n : nodes)
      n->setPosition(a, 1.0, 0.0);
  }
  auto dt =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return dt / reps;
}

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app);
  const cvc::geometry mesh = little_mesh();

  std::vector<std::shared_ptr<GraphicsNode>> roots, all;
  int counter = 0;
  auto t0 = std::chrono::steady_clock::now();
  for (int t = 0; t < TREES; ++t) {
    auto root = sg.addGraphics("tree" + std::to_string(t), mesh);
    roots.push_back(root);
    all.push_back(root);
    grow(root, mesh, DEPTH - 1, counter, all);
  }
  const double build = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  std::printf("built %zu nodes (%d trees x %zu modules, depth %d) in %.1f s\n", all.size(), TREES,
              all.size() / TREES, DEPTH, build);
  std::printf("  pose roots only (%zu): %7.2f ms/frame\n", roots.size(), time_ms(roots, 30));
  std::printf("  pose every node (%zu): %7.2f ms/frame\n", all.size(), time_ms(all, 5));
  return 0;
}

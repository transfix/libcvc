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
//
// The flat section at the end is the per-frame vehicle shape instead: N
// registered nodes posed by a full matrix every frame, setTransform against
// setPoseMatrix, with the frame's processEvents() (where the scene-bounds walk
// runs) amortised into the per-call figure.
#include <chrono>
#include <cmath>
#include <cstdint>
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

// The per-frame vehicle-pose shape: FLAT registered nodes, every one posed with
// a full matrix each frame, then ONE processEvents() -- which is where the
// scene's deferred world-bounds work lands. Returns microseconds per pose call,
// with the frame's pump amortised over the calls, so what one call drags into
// the frame is counted, not just what it costs inline.
template <typename Pose>
double flat_pose_us(SceneGraph &sg, const std::vector<std::shared_ptr<GraphicsNode>> &nodes,
                    int frames, Pose &&pose) {
  auto t0 = std::chrono::steady_clock::now();
  for (int f = 0; f < frames; ++f) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      // A yaw plus a translation, different per node and per frame (row-major).
      const double th = 0.01 * static_cast<double>(f + static_cast<int>(i));
      const double c = std::cos(th), s = std::sin(th);
      const double m[16] = {c,   -s,  0.0, static_cast<double>(i), //
                            s,   c,   0.0, 0.01 * f,               //
                            0.0, 0.0, 1.0, 0.0,                    //
                            0.0, 0.0, 0.0, 1.0};
      pose(*nodes[i], m);
    }
    sg.processEvents();
  }
  const double us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  return us / (static_cast<double>(frames) * static_cast<double>(nodes.size()));
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

  // Flat: FLAT_NODES registered vehicles, each posed by a full matrix per frame.
  {
    constexpr int FLAT_NODES = 100;
    constexpr int FRAMES = 100;
    cvc::app flatApp;
    SceneGraph flat(flatApp);
    std::vector<std::shared_ptr<GraphicsNode>> vehicles;
    for (int i = 0; i < FLAT_NODES; ++i)
      vehicles.push_back(flat.addGraphics("v" + std::to_string(i), mesh));
    flat.processEvents(); // drain the registration work before timing
    std::printf("flat %d registered nodes, pose all + one processEvents per frame:\n", FLAT_NODES);
    std::uint64_t w0 = flat.boundsWalkCount();
    const double setTransformUs = flat_pose_us(
        flat, vehicles, FRAMES, [](GraphicsNode &n, const double *m) { n.setTransform(m); });
    std::printf("  setTransform:  %8.2f us/call  (%.1f bounds walks/frame)\n", setTransformUs,
                static_cast<double>(flat.boundsWalkCount() - w0) / FRAMES);
    w0 = flat.boundsWalkCount();
    const double setPoseUs = flat_pose_us(
        flat, vehicles, FRAMES, [](GraphicsNode &n, const double *m) { n.setPoseMatrix(m); });
    std::printf("  setPoseMatrix: %8.2f us/call  (%.1f bounds walks/frame)\n", setPoseUs,
                static_cast<double>(flat.boundsWalkCount() - w0) / FRAMES);
    // What the one remaining walk per frame costs on its own.
    constexpr int WALKS = 200;
    auto t0w = std::chrono::steady_clock::now();
    for (int k = 0; k < WALKS; ++k)
      (void)flat.computeGraphicsBounds();
    const double walkUs =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0w).count() /
        WALKS;
    std::printf("  one bounds walk over %d nodes: %.2f us\n", FLAT_NODES, walkUs);
  }
  return 0;
}

// SceneGraph::treeVersion(): bumped when a node joins or leaves the tree, and only then.
//
// cloud_sky gives its receivers' meshes the deck's shade, and a receiver grows meshes after it is
// realized (an LOD rung made late, a regenerated world's parts). It used to walk every receiver
// once a second to find them; now it walks when treeVersion() moves. So the counter must move for
// every way a node can join or leave (else a new mesh never takes the shade) and must hold still
// for the per-frame changes that keep a node's decoration (else the walk runs every frame).
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes assert() expand to
// nothing and every check below would pass vacuously. Undefine it before <cassert>.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <memory>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

static cvc::geometry makeTri() {
  cvc::geometry tri;
  cvc::geometry::point_t p;
  for (int i = 0; i < 3; ++i) {
    p[0] = i;
    p[1] = i * 2;
    p[2] = 1;
    tri.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  tri.tris().push_back(t);
  return tri;
}

int main() {
  cvc::app app;
  SceneGraph sg(app);
  std::uint64_t v = sg.treeVersion();

  // Registering a graphics node.
  auto root = sg.addGraphics("root", makeTri());
  assert(root);
  assert(sg.treeVersion() != v && "registering a node changes the tree");
  v = sg.treeVersion();

  // A child added under a node in the scene, and a grandchild under it.
  auto child = root->addGraphicsChild<GeometryNode>("child");
  assert(sg.treeVersion() != v && "a child added in the scene changes the tree");
  v = sg.treeVersion();
  auto grandchild = child->addGraphicsChild<GeometryNode>("grandchild");
  assert(sg.treeVersion() != v && "a grandchild added in the scene changes the tree");
  v = sg.treeVersion();

  // What keeps a node's decoration: a move, a hide, a geometry swap.
  child->setPosition(1.0, 2.0, 3.0);
  child->setVisible(false);
  child->setVisible(true);
  child->setGeometry(makeTri());
  assert(sg.treeVersion() == v && "moves, visibility and geometry swaps leave the tree alone");

  // A subtree built off the scene changes nothing until it joins, then changes it once joined.
  auto loose = std::make_shared<GeometryNode>(app, "loose", "loose");
  loose->addGraphicsChild<GeometryNode>("loose_child");
  assert(sg.treeVersion() == v && "building a subtree off the scene leaves the tree alone");
  root->addGraphicsChild(loose);
  assert(sg.treeVersion() != v && "a subtree joining the scene changes the tree");
  v = sg.treeVersion();

  // Leaving: a child removed, then a registered node removed.
  child->removeGraphicsChild(grandchild);
  assert(sg.treeVersion() != v && "a child removed from the scene changes the tree");
  v = sg.treeVersion();
  sg.removeGraphics("root");
  assert(sg.treeVersion() != v && "removing a registered node changes the tree");

  std::printf("cvcgl_tree_version: PASSED\n");
  return 0;
}

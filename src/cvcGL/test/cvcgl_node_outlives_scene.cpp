// Lifetime guard: a scene node may outlive the SceneGraph that built it.
//
// removeGraphics() hands a node back to whoever still holds a shared_ptr, and a
// host — the Python bindings, a viewer keeping a handle — can hold it well past
// the scene itself. Every raw back-pointer the node keeps then names freed
// memory. The one that bit was the SceneGraph pointer, because a pose write
// goes through the scene's publisher:
//
//   ___pthread_mutex_lock (mutex=0x8)
//   cvc::gl::state_publisher::publish(...)
//   cvc::gl::GraphicsNode::setPosition(double,double,double)
//
// SceneNode now pairs that pointer with a weak handle on the scene's lifetime,
// so getSceneGraph() reports nullptr the moment the scene is gone and every
// caller takes the no-scene path it already had (setPosition writes state
// directly). ~GraphicsNode orphans its children for the same reason one level
// down: m_parent feeds updateTransform(), which setPosition also runs.
//
// The cases below are the ways a node can be left holding one: removed from the
// scene before it died, still attached when it died, a descendant of a held
// node, and a LightNode (whose notifyScene() dereferences the scene too).
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <memory>
#include <string>

using cvc::gl::GraphicsNode;
using cvc::gl::LightNode;
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

// A node's pose lands in the app's state tree, which outlives any one scene. It
// is read back here to prove the no-scene fallback actually ran: a value that
// arrives means setPosition() took the direct write, not the publisher.
static std::string posOf(const std::shared_ptr<GraphicsNode> &n) {
  return n->getState("position").value();
}

int main() {
  cvc::app app;
  const cvc::geometry tri = makeTri();

  // Case A — the reported crash. Removed from the scene, then held past it.
  {
    std::shared_ptr<GraphicsNode> held;
    {
      SceneGraph sg(app);
      held = sg.addGraphics("removed", tri);
      assert(held);
      // While the scene lives the node knows it — without this the rest of the
      // file could pass on a getSceneGraph() that always answered nullptr.
      assert(held->getSceneGraph() == &sg && "attached node must see its scene");
      held->setPosition(1.0, 2.0, 3.0);
      sg.removeGraphics("removed");
      assert(!sg.hasGraphics("removed"));
      assert(held->getSceneGraph() == &sg && "a removed node still has a live scene");
    }
    // The scene is gone; only `held` keeps the node alive.
    assert(held->getSceneGraph() == nullptr && "scene death must clear the back-pointer");
    held->setPosition(4.0, 5.0, 6.0); // used to lock the freed publisher's mutex
    assert(posOf(held) == "4,5,6" && "pose must fall back to a direct state write");
  }

  // Case B — never removed: still attached when the scene was destroyed. This is
  // the shape a host hits without thinking about it (`n = addGraphics(...)`,
  // scene goes away, `n` is still in hand), and it also drops the node's parent
  // out from under it.
  {
    std::shared_ptr<GraphicsNode> held;
    {
      SceneGraph sg(app);
      held = sg.addGraphics("attached", tri);
      assert(held && held->getSceneGraph() == &sg);
    }
    assert(held->getSceneGraph() == nullptr);
    held->setPosition(7.0, 8.0, 9.0); // setPosition -> updateTransform -> m_parent
    assert(posOf(held) == "7,8,9");
    held->setVisible(false); // the requestRender() path off the same pointer
    held->setVisible(true);
  }

  // Case C — a descendant. setSceneGraph() propagates down the subtree, so the
  // handle has to reach the grandchild too, and the child must survive its
  // parent being released with the scene.
  {
    std::shared_ptr<GraphicsNode> child;
    {
      SceneGraph sg(app);
      auto parent = sg.addGraphics("group");
      assert(parent);
      child = parent->createChild<cvc::gl::GeometryNode>("leaf", tri);
      assert(child && child->getSceneGraph() == &sg);
    }
    assert(child->getSceneGraph() == nullptr);
    child->setPosition(-1.0, -2.0, -3.0);
    assert(posOf(child) == "-1,-2,-3");
  }

  // Case D — a light. LightNode::notifyScene() asks the scene to rebuild its
  // whole light set on every edit, so an orphaned light dereferences the scene
  // on a path the plain graphics nodes never take.
  {
    std::shared_ptr<LightNode> light;
    {
      SceneGraph sg(app);
      light = sg.addLight("sun");
      assert(light && light->getSceneGraph() == &sg);
    }
    assert(light->getSceneGraph() == nullptr);
    light->setPosition(10.0, 11.0, 12.0); // -> notifyScene() -> no scene, no rebuild
    light->setVisible(false);             // LightNode::setVisible also notifies
    light->setColor(0.25, 0.5, 0.75);
  }

  // Case E — outliving the scene by a lot, with the node used the whole time.
  // The handle is per-scene, so a node re-homed into a SECOND scene must follow
  // that one and not resurrect the first.
  {
    std::shared_ptr<GraphicsNode> held;
    {
      SceneGraph first(app);
      held = first.addGraphics("moved", tri);
      first.removeGraphics("moved");
    }
    assert(held->getSceneGraph() == nullptr);
    {
      SceneGraph second(app, "cvcgl2");
      held->setSceneGraph(&second);
      assert(held->getSceneGraph() == &second && "re-homing must take the new scene");
      held->setPosition(13.0, 14.0, 15.0);
    }
    assert(held->getSceneGraph() == nullptr && "the second scene's death detaches it too");
    held->setPosition(16.0, 17.0, 18.0);
    assert(posOf(held) == "16,17,18");
  }

  std::printf("cvcGL node-outlives-scene: OK (removed, attached, child, light, re-homed)\n");
  return 0;
}

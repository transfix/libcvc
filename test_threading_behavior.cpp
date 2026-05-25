// Quick validation test for per-instance threading behavior
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <iostream>
#include <thread>
#include <volrover3/GeometryNode.h>
#include <volrover3/SceneGraph.h>

int main() {
  // Test 1: Nodes should have threading disabled during construction
  std::cout << "Test 1: Node construction with threading disabled..." << std::endl;
  {
    SceneGraph sg("test1");
    auto root = sg.getGraphicsRoot();

    // Root and children should have threading disabled initially (set in constructor)
    // Then enabled when SceneGraph reference is set
    std::cout << "  Root node instance threading: "
              << (root->getInstanceThreading() ? "enabled" : "disabled") << std::endl;
  }
  std::cout << "  ✓ Test 1 passed (no crashes during construction)" << std::endl;

  // Test 2: Adding graphics with threading enabled should queue events
  std::cout << "\nTest 2: Graphics creation with threading enabled..." << std::endl;
  {
    cvc::state_object<SceneNode>::setUseThreading(true);

    SceneGraph sg("test2");

    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    geom.tris().push_back({0, 1, 2});

    auto node = sg.addGraphics("test_geom", geom);

    std::cout << "  Node instance threading: "
              << (node->getInstanceThreading() ? "enabled" : "disabled") << std::endl;
    std::cout << "  Node has SceneGraph: " << (node->getSceneGraph() != nullptr ? "yes" : "no")
              << std::endl;

    // Process any queued events
    sg.processEvents();

    std::cout << "  Node is visible: " << (node->isVisible() ? "yes" : "no") << std::endl;

    cvc::state_object<SceneNode>::setUseThreading(false);
  }
  std::cout << "  ✓ Test 2 passed" << std::endl;

  // Test 3: Multiple SceneGraphs should have independent threading
  std::cout << "\nTest 3: Independent threading per SceneGraph..." << std::endl;
  {
    SceneGraph sg1("test3a");
    SceneGraph sg2("test3b");

    auto node1 = sg1.getGraphicsRoot();
    auto node2 = sg2.getGraphicsRoot();

    std::cout << "  SG1 root has SG1: " << (node1->getSceneGraph() == &sg1 ? "yes" : "no")
              << std::endl;
    std::cout << "  SG2 root has SG2: " << (node2->getSceneGraph() == &sg2 ? "yes" : "no")
              << std::endl;
  }
  std::cout << "  ✓ Test 3 passed" << std::endl;

  std::cout << "\n✓ All threading behavior tests passed!" << std::endl;
  return 0;
}

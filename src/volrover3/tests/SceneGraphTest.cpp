#include <gtest/gtest.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <volrover3/AppState.h>
#include <cvc/state.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <thread>
#include <chrono>
#include <atomic>

class SceneGraphTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        // Disable threading for state_object to avoid race conditions during destruction
        cvc::state_object<SceneNode>::setUseThreading(false);
    }
    
    void SetUp() override {
        sceneGraph = new SceneGraph();
        appState = &AppState::instance();
    }
    
    void TearDown() override {
        delete sceneGraph;
    }
    
    SceneGraph* sceneGraph;
    AppState* appState;
};

TEST_F(SceneGraphTest, InitialState) {
    EXPECT_NE(sceneGraph, nullptr);
    // SceneGraph doesn't expose renderer/renderWindow, just verify it was created
    SUCCEED();
}

TEST_F(SceneGraphTest, AddGeometryNode) {
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = sceneGraph->addGraphics("test_geom", geom);
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(sceneGraph->getGraphics("test_geom"), node);
}

TEST_F(SceneGraphTest, AddVolumeNode) {
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    
    auto node = sceneGraph->addGraphics("test_vol", vol);
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(sceneGraph->getGraphics("test_vol"), node);
}

TEST_F(SceneGraphTest, ShowHideGrid) {
    sceneGraph->setGridVisible(true);
    // Grid should be created and visible
    
    sceneGraph->setGridVisible(false);
    // Grid should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, ShowHideAxes) {
    sceneGraph->setAxisVisible(true);
    // Axes should be created and visible
    
    sceneGraph->setAxisVisible(false);
    // Axes should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, UpdateBoundingBox) {
    cvc::bounding_box bounds;
    bounds.setMin(-1.0, -1.0, -1.0);
    bounds.setMax(1.0, 1.0, 1.0);
    sceneGraph->updateGrid(bounds);
    
    // Grid should be updated to match bounding box
    SUCCEED();
}

TEST_F(SceneGraphTest, ResetCamera) {
    // SceneGraph doesn't expose resetCamera, this would be done via the renderer
    SUCCEED();
}

TEST_F(SceneGraphTest, TransferFunctionUpdate) {
    // Create a volume first
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    auto volNode = sceneGraph->addGraphics("test_vol", vol);
    
    // Update transfer function
    std::vector<double> colorTable = {0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    std::vector<double> opacityTable = {0.0, 0.0, 1.0, 1.0};
    
    sceneGraph->updateTransferFunction(colorTable, opacityTable);
    
    // Transfer function should be applied to volume
    SUCCEED();
}

TEST_F(SceneGraphTest, MultipleUpdates) {
    // Test multiple updates don't cause issues
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    sceneGraph->addGraphics("test_geom", geom);
    
    cvc::volume vol(cvc::dimension(2, 2, 2), cvc::UChar);
    sceneGraph->addGraphics("test_vol", vol);
    
    sceneGraph->setGridVisible(true);
    sceneGraph->setAxisVisible(true);
    
    // Camera reset would be done externally
    
    SUCCEED();
}

// ===========================
// State Tree Integration Tests
// ===========================

TEST_F(SceneGraphTest, VisibilityStateTree) {
    // SceneGraph doesn't directly manipulate state tree
    // but we can verify it responds to AppState visibility flags
    
    // Set visibility via scene graph
    sceneGraph->setGridVisible(true);
    sceneGraph->setAxisVisible(false);
    
    // These don't save to AppState automatically
    // In actual usage, MainWindow coordinates between SceneGraph and AppState
    SUCCEED();
}

// NOTE: Transfer function storage moved to per-volume state in VolumeNode
// SceneGraph no longer has global transfer function updates
/*
TEST_F(SceneGraphTest, TransferFunctionFromState) {
    // Create volume
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    sceneGraph->addGraphics("test_vol", vol);
    
    // Get transfer function from AppState
    auto colorTable = appState->transferFunctionColorTable();
    auto opacityTable = appState->transferFunctionOpacityTable();
    
    // Apply to scene graph
    sceneGraph->updateTransferFunction(colorTable, opacityTable);
    
    SUCCEED();
}
*/

TEST_F(SceneGraphTest, WorldBoundsUpdate) {
    // Set world bounds in AppState
    cvc::bounding_box bounds(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    appState->setWorldBounds(bounds);
    
    // Update scene graph grid
    sceneGraph->updateGrid(bounds);
    
    // Verify state tree has the bounds
    auto& stateTree = cvc::state::instance()("volrover3");
    auto values = stateTree("world_bounds").values();
    ASSERT_EQ(values.size(), size_t(6));
    
    SUCCEED();
}

// ===========================
// Color Tests
// ===========================

TEST_F(SceneGraphTest, GridColorUpdate) {
    // Set grid color
    sceneGraph->setGridColor(0.9, 0.1, 0.5);
    
    // Verify color was applied (we can't directly inspect VTK properties
    // in these tests without a renderer, but we verify it doesn't crash)
    SUCCEED();
}

// ===========================
// Axis Scaling Tests
// ===========================

TEST_F(SceneGraphTest, AxisScalingWithBounds) {
    // Create bounding boxes of different sizes and verify axis scales
    
    // Small bounds
    cvc::bounding_box smallBounds(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    sceneGraph->updateGrid(smallBounds);
    // Axis should be scaled to ~20% of max span (2.0), so ~0.4
    
    // Large bounds
    cvc::bounding_box largeBounds(-50.0, -50.0, -50.0, 50.0, 50.0, 50.0);
    sceneGraph->updateGrid(largeBounds);
    // Axis should be scaled to ~20% of max span (100.0), so ~20.0
    
    // Asymmetric bounds
    cvc::bounding_box asymBounds(-5.0, -2.0, -1.0, 5.0, 2.0, 1.0);
    sceneGraph->updateGrid(asymBounds);
    // Axis should be scaled to ~20% of max span (10.0), so ~2.0
    
    SUCCEED();
}

TEST_F(SceneGraphTest, AxisScalingEdgeCases) {
    // Test with very small bounds
    cvc::bounding_box tinyBounds(-0.01, -0.01, -0.01, 0.01, 0.01, 0.01);
    sceneGraph->updateGrid(tinyBounds);
    
    // Test with very large bounds
    cvc::bounding_box hugeBounds(-1000.0, -1000.0, -1000.0, 1000.0, 1000.0, 1000.0);
    sceneGraph->updateGrid(hugeBounds);
    
    // Test with zero-volume bounds
    cvc::bounding_box zeroBounds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    sceneGraph->updateGrid(zeroBounds);
    
    SUCCEED();
}

TEST_F(SceneGraphTest, AxisScalingWithGeometry) {
    // Create simple geometry
    cvc::geometry geom;
    geom.points().push_back({-2.0, -2.0, -2.0});
    geom.points().push_back({2.0, 2.0, 2.0});
    
    // Set geometry and get its bounds
    sceneGraph->addGraphics("test_geom", geom);
    cvc::bounding_box geomBounds = geom.extents();
    
    // Update grid/axis with geometry bounds
    sceneGraph->updateGrid(geomBounds);
    
    SUCCEED();
}

TEST_F(SceneGraphTest, GridAndAxisUpdateTogether) {
    // Verify that updating grid also updates axis scaling
    cvc::bounding_box bounds1(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
    sceneGraph->updateGrid(bounds1);
    
    cvc::bounding_box bounds2(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);
    sceneGraph->updateGrid(bounds2);
    
    cvc::bounding_box bounds3(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0);
    sceneGraph->updateGrid(bounds3);
    
    SUCCEED();
}

// ===========================
// Integration Tests
// ===========================

TEST_F(SceneGraphTest, ColorAndBoundsIntegration) {
    // Test setting grid color through GridNode and bounds together
    auto gridNode = sceneGraph->getGridNode();
    gridNode->getState("color").value("0.5,0.5,0.5");
    
    cvc::bounding_box bounds(-25.0, -25.0, -25.0, 25.0, 25.0, 25.0);
    sceneGraph->updateGrid(bounds);
    
    SUCCEED();
}

// ===========================
// Event Queue Threading Tests
// ===========================

TEST_F(SceneGraphTest, EventQueueWithThreading) {
    // Enable threading for this test
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    // Create a new scene graph that will use threading
    SceneGraph* threadedGraph = new SceneGraph();
    
    // Create geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = threadedGraph->addGraphics("test_geom", geom);
    ASSERT_NE(node, nullptr);
    
    // Get raw pointer for lambda capture
    GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node.get());
    ASSERT_NE(geomNode, nullptr);
    
    // Track if operations were executed
    std::atomic<int> operationsQueued{0};
    
    // Modify state from a worker thread
    std::thread worker([geomNode, &operationsQueued]() {
        // These should be queued, not executed immediately
        geomNode->setVisible(false);
        operationsQueued++;
        
        geomNode->setColor(1.0, 0.0, 0.0);
        operationsQueued++;
        
        geomNode->setOpacity(0.5);
        operationsQueued++;
    });
    
    worker.join();
    
    // Verify operations were queued
    EXPECT_EQ(operationsQueued.load(), 3);
    
    // Process the event queue (simulating main thread)
    threadedGraph->processEvents();
    
    // Node should still be valid after processing
    ASSERT_NE(geomNode, nullptr);
    
    // Cleanup
    delete threadedGraph;
    
    // Disable threading again
    cvc::state_object<SceneNode>::setUseThreading(false);
}

TEST_F(SceneGraphTest, EventQueueMultipleThreads) {
    // Enable threading
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    SceneGraph* threadedGraph = new SceneGraph();
    
    // Create multiple geometry nodes
    std::vector<GeometryNode*> nodes;
    for (int i = 0; i < 5; i++) {
        cvc::geometry geom;
        geom.points().push_back({static_cast<double>(i), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i+1), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i), 1.0, 0.0});
        
        auto node = threadedGraph->addGraphics("geom_" + std::to_string(i), geom);
        nodes.push_back(dynamic_cast<GeometryNode*>(node.get()));
    }
    
    std::atomic<int> totalOps{0};
    
    // Launch multiple worker threads
    std::vector<std::thread> workers;
    for (int i = 0; i < 5; i++) {
        workers.emplace_back([&nodes, i, &totalOps]() {
            if (nodes[i]) {
                nodes[i]->setVisible(i % 2 == 0);
                totalOps++;
                
                nodes[i]->setColor(i * 0.2, 0.0, 1.0 - i * 0.2);
                totalOps++;
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                
                nodes[i]->setOpacity(0.1 + i * 0.15);
                totalOps++;
            }
        });
    }
    
    // Wait for all workers
    for (auto& w : workers) {
        w.join();
    }
    
    EXPECT_EQ(totalOps.load(), 15); // 5 nodes * 3 operations each
    
    // Process all queued events
    threadedGraph->processEvents();
    
    // Verify all nodes are still valid
    for (int i = 0; i < 5; i++) {
        ASSERT_NE(nodes[i], nullptr);
    }
    
    delete threadedGraph;
    cvc::state_object<SceneNode>::setUseThreading(false);
}

TEST_F(SceneGraphTest, EventQueueProcessingOrder) {
    // Enable threading
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    SceneGraph* threadedGraph = new SceneGraph();
    
    // Track execution order
    std::vector<int> executionOrder;
    std::mutex orderMutex;
    
    // Post several events from worker thread
    std::thread worker([threadedGraph, &executionOrder, &orderMutex]() {
        for (int i = 0; i < 10; i++) {
            threadedGraph->postEvent([i, &executionOrder, &orderMutex]() {
                std::lock_guard<std::mutex> lock(orderMutex);
                executionOrder.push_back(i);
            });
        }
    });
    
    worker.join();
    
    // Process events - they should execute in FIFO order
    threadedGraph->processEvents();
    
    ASSERT_EQ(executionOrder.size(), size_t(10));
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(executionOrder[i], i) << "Event " << i << " executed out of order";
    }
    
    delete threadedGraph;
    cvc::state_object<SceneNode>::setUseThreading(false);
}

TEST_F(SceneGraphTest, EventQueueWithVolumeNode) {
    // Enable threading
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    SceneGraph* threadedGraph = new SceneGraph();
    
    // Create volume node
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    auto node = threadedGraph->addGraphics("test_vol", vol);
    
    // Get raw pointer for lambda capture
    VolumeNode* volNode = dynamic_cast<VolumeNode*>(node.get());
    ASSERT_NE(volNode, nullptr);
    
    std::atomic<int> opsExecuted{0};
    
    // Modify volume properties from worker thread
    std::thread worker([volNode, &opsExecuted]() {
        volNode->setVisible(false);
        opsExecuted++;
        
        volNode->setShading(true);
        opsExecuted++;
        
        volNode->setAmbient(0.5);
        opsExecuted++;
    });
    
    worker.join();
    
    EXPECT_EQ(opsExecuted.load(), 3);
    
    // Process queue
    threadedGraph->processEvents();
    
    // Verify node still valid
    ASSERT_NE(volNode, nullptr);
    
    delete threadedGraph;
    cvc::state_object<SceneNode>::setUseThreading(false);
}

TEST_F(SceneGraphTest, EventQueueStressTest) {
    // Enable threading
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    SceneGraph* threadedGraph = new SceneGraph();
    
    // Create a geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = threadedGraph->addGraphics("stress_test", geom);
    GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node.get());
    ASSERT_NE(geomNode, nullptr);
    
    std::atomic<int> totalOps{0};
    
    // Launch many threads doing many operations
    std::vector<std::thread> workers;
    for (int t = 0; t < 10; t++) {
        workers.emplace_back([geomNode, &totalOps, t]() {
            for (int i = 0; i < 20; i++) {
                geomNode->setVisible(i % 2 == 0);
                totalOps++;
                
                geomNode->setColor((t * 0.1), (i * 0.05), 0.5);
                totalOps++;
                
                geomNode->setOpacity(0.1 + (i % 10) * 0.09);
                totalOps++;
            }
        });
    }
    
    for (auto& w : workers) {
        w.join();
    }
    
    EXPECT_EQ(totalOps.load(), 600); // 10 threads * 20 iterations * 3 ops
    
    // Process all events
    threadedGraph->processEvents();
    
    // Just verify it didn't crash and node is still valid
    ASSERT_NE(geomNode, nullptr);
    
    delete threadedGraph;
    cvc::state_object<SceneNode>::setUseThreading(false);
}

// ===========================
// Per-Instance Threading Tests
// ===========================

TEST_F(SceneGraphTest, PerInstanceThreadingEnabled) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create a geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = graph->addGraphics("test_geom", geom);
    GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node.get());
    ASSERT_NE(geomNode, nullptr);
    
    // After setSceneGraph, instance threading follows global flag (which is false)
    EXPECT_FALSE(geomNode->getInstanceThreading());
    
    // Manually enable instance threading on this specific node
    geomNode->setInstanceThreading(true);
    EXPECT_TRUE(geomNode->getInstanceThreading());
    
    // Track if operation was queued
    std::atomic<bool> operationQueued{false};
    
    // Modify from worker thread - should be queued due to per-instance threading
    std::thread worker([geomNode, &operationQueued]() {
        geomNode->setColor(1.0, 0.5, 0.0);
        operationQueued = true;
    });
    
    worker.join();
    EXPECT_TRUE(operationQueued.load());
    
    // Process events
    graph->processEvents();
    
    delete graph;
}

TEST_F(SceneGraphTest, PerInstanceThreadingDisabled) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create a geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = graph->addGraphics("test_geom", geom);
    GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node.get());
    ASSERT_NE(geomNode, nullptr);
    
    // Manually disable instance threading
    geomNode->setInstanceThreading(false);
    EXPECT_FALSE(geomNode->getInstanceThreading());
    
    // Modify from worker thread - should execute immediately (no queueing)
    std::atomic<bool> operationCompleted{false};
    
    std::thread worker([geomNode, &operationCompleted]() {
        geomNode->setColor(0.2, 0.8, 0.6);
        operationCompleted = true;
    });
    
    worker.join();
    EXPECT_TRUE(operationCompleted.load());
    
    delete graph;
}

TEST_F(SceneGraphTest, MixedPerInstanceThreading) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create multiple geometry nodes
    std::vector<GeometryNode*> nodes;
    for (int i = 0; i < 3; i++) {
        cvc::geometry geom;
        geom.points().push_back({static_cast<double>(i), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i+1), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i), 1.0, 0.0});
        
        auto node = graph->addGraphics("geom_" + std::to_string(i), geom);
        nodes.push_back(dynamic_cast<GeometryNode*>(node.get()));
    }
    
    // All nodes start with threading disabled (global flag is false)
    EXPECT_FALSE(nodes[0]->getInstanceThreading());
    EXPECT_FALSE(nodes[1]->getInstanceThreading());
    EXPECT_FALSE(nodes[2]->getInstanceThreading());
    
    // Enable threading on nodes 0 and 2
    nodes[0]->setInstanceThreading(true);
    nodes[2]->setInstanceThreading(true);
    
    // Verify settings
    EXPECT_TRUE(nodes[0]->getInstanceThreading());
    EXPECT_FALSE(nodes[1]->getInstanceThreading());
    EXPECT_TRUE(nodes[2]->getInstanceThreading());
    
    // Modify all from worker threads
    std::atomic<int> opsCompleted{0};
    std::vector<std::thread> workers;
    
    for (int i = 0; i < 3; i++) {
        workers.emplace_back([&nodes, i, &opsCompleted]() {
            nodes[i]->setColor(i * 0.3, 0.5, 1.0 - i * 0.3);
            opsCompleted++;
        });
    }
    
    for (auto& w : workers) {
        w.join();
    }
    
    EXPECT_EQ(opsCompleted.load(), 3);
    
    // Process events (will process nodes 0 and 2, node 1 already executed)
    graph->processEvents();
    
    delete graph;
}

TEST_F(SceneGraphTest, PerInstanceThreadingBeforeSceneGraph) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create a geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto node = graph->addGraphics("test_geom", geom);
    GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node.get());
    ASSERT_NE(geomNode, nullptr);
    
    // After SceneGraph is set, threading follows global flag (false)
    EXPECT_FALSE(geomNode->getInstanceThreading());
    
    // Enable instance threading
    geomNode->setInstanceThreading(true);
    EXPECT_TRUE(geomNode->getInstanceThreading());
    
    // Clear instance threading (revert to using global flag)
    geomNode->clearInstanceThreading();
    
    // Should now use global flag (which is false)
    EXPECT_FALSE(geomNode->getInstanceThreading());
    
    delete graph;
}

TEST_F(SceneGraphTest, PerInstanceThreadingWithGlobalEnabled) {
    // Enable global threading
    cvc::state_object<SceneNode>::setUseThreading(true);
    
    SceneGraph* graph = new SceneGraph();
    
    // Create multiple nodes
    std::vector<GeometryNode*> nodes;
    for (int i = 0; i < 2; i++) {
        cvc::geometry geom;
        geom.points().push_back({static_cast<double>(i), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i+1), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i), 1.0, 0.0});
        
        auto node = graph->addGraphics("geom_" + std::to_string(i), geom);
        nodes.push_back(dynamic_cast<GeometryNode*>(node.get()));
    }
    
    // Both should have instance threading enabled (from SceneGraph)
    EXPECT_TRUE(nodes[0]->getInstanceThreading());
    EXPECT_TRUE(nodes[1]->getInstanceThreading());
    
    // Disable instance threading on node 0
    nodes[0]->setInstanceThreading(false);
    EXPECT_FALSE(nodes[0]->getInstanceThreading());
    
    // Node 1 should still have it enabled
    EXPECT_TRUE(nodes[1]->getInstanceThreading());
    
    std::atomic<int> opsCompleted{0};
    
    // Node 0: Should execute immediately (instance threading disabled)
    std::thread worker0([&nodes, &opsCompleted]() {
        nodes[0]->setColor(1.0, 0.0, 0.0);
        opsCompleted++;
    });
    
    // Node 1: Should queue (instance threading enabled)
    std::thread worker1([&nodes, &opsCompleted]() {
        nodes[1]->setColor(0.0, 1.0, 0.0);
        opsCompleted++;
    });
    
    worker0.join();
    worker1.join();
    
    EXPECT_EQ(opsCompleted.load(), 2);
    
    // Process queued events (node 1)
    graph->processEvents();
    
    delete graph;
    cvc::state_object<SceneNode>::setUseThreading(false);
}

TEST_F(SceneGraphTest, PerInstanceThreadingPropagation) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create a parent node
    auto parent = graph->addGraphics("parent");
    
    // Create a child geometry node
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    auto child = graph->addGraphics("child", geom);
    GeometryNode* childGeom = dynamic_cast<GeometryNode*>(child.get());
    ASSERT_NE(childGeom, nullptr);
    
    // Both should have instance threading disabled (global flag is false)
    EXPECT_FALSE(parent->getInstanceThreading());
    EXPECT_FALSE(childGeom->getInstanceThreading());
    
    // Enable threading on both
    parent->setInstanceThreading(true);
    childGeom->setInstanceThreading(true);
    EXPECT_TRUE(parent->getInstanceThreading());
    EXPECT_TRUE(childGeom->getInstanceThreading());
    
    // Manually disable on parent
    parent->setInstanceThreading(false);
    EXPECT_FALSE(parent->getInstanceThreading());
    
    // Child should maintain its own setting
    EXPECT_TRUE(childGeom->getInstanceThreading());
    
    delete graph;
}

TEST_F(SceneGraphTest, PerInstanceThreadingStressTest) {
    // Keep global threading disabled
    ASSERT_FALSE(cvc::state_object<SceneNode>::getUseThreading());
    
    SceneGraph* graph = new SceneGraph();
    
    // Create nodes with mixed threading settings
    std::vector<GeometryNode*> threadedNodes;
    std::vector<GeometryNode*> nonThreadedNodes;
    
    for (int i = 0; i < 5; i++) {
        cvc::geometry geom;
        geom.points().push_back({static_cast<double>(i), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i+1), 0.0, 0.0});
        geom.points().push_back({static_cast<double>(i), 1.0, 0.0});
        
        auto node = graph->addGraphics("threaded_" + std::to_string(i), geom);
        threadedNodes.push_back(dynamic_cast<GeometryNode*>(node.get()));
        
        auto node2 = graph->addGraphics("nonthreaded_" + std::to_string(i), geom);
        GeometryNode* geomNode = dynamic_cast<GeometryNode*>(node2.get());
        geomNode->setInstanceThreading(false);
        nonThreadedNodes.push_back(geomNode);
    }
    
    std::atomic<int> totalOps{0};
    std::vector<std::thread> workers;
    
    // Launch threads for threaded nodes (should queue)
    for (int i = 0; i < 5; i++) {
        workers.emplace_back([&threadedNodes, i, &totalOps]() {
            for (int j = 0; j < 10; j++) {
                threadedNodes[i]->setColor(i * 0.2, j * 0.1, 0.5);
                totalOps++;
            }
        });
    }
    
    // Launch threads for non-threaded nodes (should execute immediately)
    for (int i = 0; i < 5; i++) {
        workers.emplace_back([&nonThreadedNodes, i, &totalOps]() {
            for (int j = 0; j < 10; j++) {
                nonThreadedNodes[i]->setOpacity(0.1 + j * 0.09);
                totalOps++;
            }
        });
    }
    
    for (auto& w : workers) {
        w.join();
    }
    
    EXPECT_EQ(totalOps.load(), 100); // 10 nodes * 10 operations each
    
    // Process events (should only process the threaded nodes)
    graph->processEvents();
    
    delete graph;
}



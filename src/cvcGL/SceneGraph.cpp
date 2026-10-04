#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/AxisNode.h>
#include <cvc/gl/BBoxNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneEventSink.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneNode.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <unordered_map>
#include <vtkActor.h>
#include <vtkCameraPass.h>
#include <vtkDataObject.h>
#include <vtkGPUVolumeRayCastMapper.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkMapper.h>
#include <vtkMultiVolume.h>
#include <vtkObjectFactory.h>
#include <vtkOverlayPass.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderState.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkTranslucentPass.h>
#include <vtkVolumetricPass.h>
#include <vtkWeakPointer.h>

namespace cvc {
namespace gl {

SceneGraph::SceneGraph(cvc::app &ctx, const std::string &statePrefix)
    : m_renderer(nullptr), m_ctx(ctx), m_statePrefix(statePrefix), m_gridNode(nullptr),
      m_axisNode(nullptr),
      // Not make_shared: the nodes' weak_ptrs would keep its storage allocated
      // after the last owner let go, hiding a use-after-free from ASan/valgrind.
      m_events(new SceneEventSink()), m_graphicsRoot(nullptr), m_nullGraphic(nullptr),
      m_multiVolumeRenderingEnabled(false) {
  // This scene's own state publisher, running under the injected app — node poses
  // publish through it (SceneGraph::publisher()), coalesced off the render path.
  // Started eagerly, like the scene's pump, and drained in the destructor.
  // Single-threaded wasm cannot start the worker thread: writers still enqueue
  // (bounded, coalesced) and the embedder drains via publisher().flush(). A
  // -pthread wasm build (CVC_WASM_PTHREADS) has real threads and starts it.
  m_publisher = std::make_unique<cvc::gl::state_publisher>(m_ctx);
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
  m_publisher->start();
#endif

  // Create null graphic as THE root graphics node (all graphics go under this)
  // State path: {statePrefix}.graphics.root
  std::string rootStatePath = statePrefix + ".graphics.root";
  m_nullGraphic = std::make_shared<NullGraphicNode>(m_ctx, rootStatePath, "root");

  // Attach the root (and, recursively, its children) to this SceneGraph so their
  // runOnMainThread() work marshals through this scene's pump / owner thread.
  m_nullGraphic->setSceneGraph(this);

  m_nullGraphic->setShowBBox(true);                          // Show bbox by default
  m_nullGraphic->setBounds(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5); // Default unit cube when empty
  m_nullGraphic->setIncludeOwnBounds(
      true); // Include root bounds in visualization (will change to false when children are added)
  m_graphicsRoot = m_nullGraphic; // NullGraphic IS the graphics root
  m_rootNodes.push_back(m_graphicsRoot);

  // Create grid and axis as graphics children of the root null graphic
  // They will live in the null graphic's coordinate system and state tree
  m_gridNode = m_nullGraphic->template addGraphicsChild<GridNode>("grid");
  m_axisNode = m_nullGraphic->template addGraphicsChild<AxisNode>("axis");

  // GridNode and AxisNode initialize their own default state and colors
}

SceneGraph::~SceneGraph() {
  // Process any remaining events before shutdown
  processEvents();

  // Drain the publisher while the nodes are still alive and consistent: stop the
  // worker, then flush queued poses to the state tree so none are dropped. Done
  // here (not left to the member dtor) so the flush's valueChanged fires against
  // live nodes, exactly like a normal flush, rather than mid-teardown.
  if (m_publisher) {
    m_publisher->stop();
    m_publisher->flush();
  }

  if (m_renderer) {
    for (auto &node : m_rootNodes) {
      node->removeFromRenderer(m_renderer);
    }
  }

  // Detach every node from this scene. Dropping the lifetime token does it in one
  // move and, unlike walking m_rootNodes, it reaches the nodes that are NOT in
  // the graph any more: removeGraphics() hands a node back to its caller, and a
  // caller that outlives the scene (a Python proxy, say) would otherwise be left
  // holding a back-pointer to freed memory — a later setPosition() then locks the
  // destroyed publisher's mutex. From here on those nodes see no scene and take
  // their own no-scene path (GraphicsNode::setPosition writes state directly).
  m_alive.reset();
  // THEN close the event sink, in this order. A producer thread that fetched
  // the sink before the reset above holds it alive and may still be on its way
  // to post. A post that lands before close() is queued and dropped by it, as
  // events queued to a dying scene always were. A post after close() is
  // refused -- and because the token is already gone, the refused producer
  // already sees getSceneGraph() == nullptr: runOnMainThread then runs its work
  // inline on the no-scene path (never against this dying scene), and a
  // streaming node keeps what it staged pending for a later attach. Closing
  // first would leave a window where a refused post runs owner-thread work
  // inline on the producer while getSceneGraph() still returns this scene.
  m_events->close();
}

void SceneGraph::postEvent(std::function<void()> callback) { m_events->post(std::move(callback)); }

void SceneGraph::postEventCoalesced(const void *key, std::function<void()> callback) {
  m_events->postCoalesced(key, std::move(callback));
}

void SceneGraph::processEvents() {
  // The queued callbacks first. If one throws, the rest stay queued for the next
  // call (SceneEventSink::processEvents) and so does this follow-up.
  m_events->processEvents();

  // Then the world-bounds follow-up for every tracked node that moved since the
  // last pump -- including the moves the drain above just applied (a pose
  // marshalled from another thread) -- in ONE walk. Moves only raise the flag
  // (see trackNodeBounds); a frame that poses N vehicles used to run N walks
  // here, 2N through setTransform.
  if (m_boundsDirty.exchange(false, std::memory_order_acq_rel)) {
    onGraphicsBoundsChanged();
    // A move is something to draw even when the grid did not have to grow. The
    // slot requested a render when it raised the flag; ask again so a host that
    // consumed that request before pumping still draws the latest poses.
    requestRender();
  }

  // A grow-only walk skipped while nothing showed the world box: run it now if
  // something does again, however it was shown -- setGridVisible (which also
  // catches up at once), the grid/axis node's own setVisible or "visible"
  // state key, setBBoxesVisible, the root's setShowBBox.
  if (m_boundsGrowDeferred.load(std::memory_order_relaxed) && worldBoundsShown() &&
      m_boundsGrowDeferred.exchange(false, std::memory_order_relaxed)) {
    onGraphicsBoundsChanged();
    requestRender();
  }
}

void SceneGraph::requestRender() { m_events->requestRender(); }

bool SceneGraph::checkAndResetRenderNeeded() { return m_events->checkAndResetRenderNeeded(); }

bool SceneGraph::onOwnerThread() const { return m_events->onOwnerThread(); }

void SceneGraph::adoptOwnerThread() { m_events->adoptOwnerThread(); }

void SceneGraph::setRenderer(vtkRenderer *renderer) {
  // The shadow chain belongs to one renderer and its maps to that renderer's
  // window: take it off the old one while the window is still up (a closing
  // ViewportManager detaches its scenes before it finalizes the window).
  const bool moving = renderer != m_renderer;
  if (moving)
    removeShadowPasses();

  if (m_renderer) {
    for (auto &node : m_rootNodes) {
      node->removeFromRenderer(m_renderer);
    }
  }

  m_renderer = renderer;

  if (m_renderer) {
    for (auto &node : m_rootNodes) {
      node->addToRenderer(m_renderer);
    }
    applyLights(); // the scene's lighting follows it onto the new renderer
    if (moving && m_shadowsEnabled)
      installShadowPasses(); // and so do its shadows
  }
}

void SceneGraph::update() {
  for (auto &node : m_rootNodes) {
    node->update();
  }
}

void SceneGraph::setGridVisible(bool visible) {
  m_gridNode->setVisible(visible);
  if (visible)
    catchUpWorldBounds();
}

bool SceneGraph::gridVisible() const { return m_gridNode && m_gridNode->isVisible(); }

void SceneGraph::setAxisVisible(bool visible) {
  m_axisNode->setVisible(visible);
  if (visible)
    catchUpWorldBounds();
}

bool SceneGraph::worldBoundsShown() const {
  // What draws the world box: the grid, the axis (sized from it), and the
  // graphics root's own box (updateGrid sets the root's bounds; shown by default).
  return gridVisible() || axisVisible() || (m_nullGraphic && m_nullGraphic->getShowBBox());
}

void SceneGraph::catchUpWorldBounds() {
  // Moves made while nothing showed the world box: grow it over them now,
  // before the grid that shows it is drawn. Inline on the owner thread (updateGrid
  // touches VTK actors); from another thread the next processEvents() does it.
  if (!m_boundsGrowDeferred.exchange(false, std::memory_order_relaxed))
    return;
  if (onOwnerThread()) {
    onGraphicsBoundsChanged();
  } else {
    m_boundsDirty.store(true, std::memory_order_release);
  }
  requestRender();
}

// The axis node is private and had no accessor at all, so a control could set
// axis visibility but never read it back to draw its own tick.
bool SceneGraph::axisVisible() const { return m_axisNode && m_axisNode->isVisible(); }

void SceneGraph::setBBoxesVisible(bool visible) {
  // getAllGraphicsOfType<GraphicsNode>() already walks the whole tree from
  // m_graphicsRoot INCLUSIVE, so this is a convenience wrapper over existing
  // traversal rather than a second recursion to keep in step.
  for (const auto &n : getAllGraphicsOfType<GraphicsNode>())
    if (n)
      n->setShowBBox(visible);
}

bool SceneGraph::bboxesVisible() const {
  for (const auto &n : const_cast<SceneGraph *>(this)->getAllGraphicsOfType<GraphicsNode>())
    if (n && n->getShowBBox())
      return true;
  return false;
}

void SceneGraph::setExtentLabelsVisible(bool visible) {
  for (const auto &n : getAllGraphicsOfType<GraphicsNode>())
    if (n)
      n->setShowExtentLabels(visible);
}

bool SceneGraph::extentLabelsVisible() const {
  for (const auto &n : const_cast<SceneGraph *>(this)->getAllGraphicsOfType<GraphicsNode>())
    if (n && n->getShowExtentLabels())
      return true;
  return false;
}

void SceneGraph::setDiagnosticChromeVisible(bool visible) {
  setGridVisible(visible);
  setAxisVisible(visible);
  setBBoxesVisible(visible);
  setExtentLabelsVisible(visible);
}

bool SceneGraph::diagnosticChromeVisible() const {
  return gridVisible() || axisVisible() || bboxesVisible() || extentLabelsVisible();
}

void SceneGraph::setGridColor(double r, double g, double b) { m_gridNode->setColor(r, g, b); }

void SceneGraph::updateGrid(const cvc::bounding_box &bounds) {
  // Update the null graphic's own bounds
  m_nullGraphic->setBounds(bounds);

  // Get combined bounds of null graphic (respecting children's local coordinate systems)
  // This automatically excludes grid and axis as they're just visualization helpers
  cvc::bounding_box combinedBounds = m_nullGraphic->getCombinedBoundingBox();

  // Update grid to match combined bounds
  m_gridNode->setBounds(combinedBounds);
  m_worldBounds = combinedBounds; // track for grow-only recompute on node moves
  markContentChanged();           // the set of graphics changed, or the box grew

  // Scale axis length to be proportional to combined bounding box size
  double spanX = combinedBounds[3] - combinedBounds[0];
  double spanY = combinedBounds[4] - combinedBounds[1];
  double spanZ = combinedBounds[5] - combinedBounds[2];
  double maxSpan = std::max({spanX, spanY, spanZ});

  // Set axis to be about 20% of the maximum span
  double axisLength = maxSpan * 0.2;
  if (axisLength > 0.0) {
    m_axisNode->setAxisLength(axisLength);
  }
}

void SceneGraph::setGridPlaneVisibility(bool yz, bool xz, bool xy) {
  m_gridNode->setYZPlaneVisible(yz);
  m_gridNode->setXZPlaneVisible(xz);
  m_gridNode->setXYPlaneVisible(xy);
}

void SceneGraph::setGridDivisions(int x, int y, int z) { m_gridNode->setGridDivisions(x, y, z); }

void SceneGraph::setGridTickIntervals(int x, int y, int z) {
  m_gridNode->setTickIntervals(x, y, z);
}

void SceneGraph::setGridPlaneColors(double yzR, double yzG, double yzB, double xzR, double xzG,
                                    double xzB, double xyR, double xyG, double xyB) {
  m_gridNode->setYZPlaneColor(yzR, yzG, yzB);
  m_gridNode->setXZPlaneColor(xzR, xzG, xzB);
  m_gridNode->setXYPlaneColor(xyR, xyG, xyB);
}

void SceneGraph::setGridTickLabelProperties(double r, double g, double b, int fontSize) {
  m_gridNode->setTickLabelColor(r, g, b);
  m_gridNode->setTickLabelFontSize(fontSize);
}

void SceneGraph::updateTransferFunction(const std::vector<double> &colorTable,
                                        const std::vector<double> &opacityTable) {
  // Apply transfer function to all volume nodes
  auto volumes = getAllVolumeGraphics();
  for (auto &volNode : volumes) {
    volNode->setTransferFunction(colorTable, opacityTable);
  }
}

cvc::bounding_box SceneGraph::computeGraphicsBounds() const {
  m_boundsWalks.fetch_add(1, std::memory_order_relaxed); // boundsWalkCount()
  cvc::bounding_box combinedBounds;
  bool first = true;

  // Process each direct child of the graphics root
  // Each child's getCombinedBoundingBox() already includes its descendants recursively
  if (m_graphicsRoot) {
    for (const auto &childPtr : m_graphicsRoot->getGraphicsChildren()) {
      const GraphicsNode *child = childPtr.get();
      if (!child)
        continue;

      // Skip grid and axis nodes - they don't contribute to scene bounds
      if (child == m_gridNode.get() || child == m_axisNode.get()) {
        continue;
      }

      // Skip LIGHTS. A light draws nothing, but it is a node with a transform,
      // so its (empty) box lands at the light's position and drags the scene
      // bounds out to it. The overhead fill sits at 3x the stage radius, which
      // lifted the whole scene box — and with it the orbit centre that
      // frameBounds() derives, so the camera sat raised and could not be brought
      // back down. (A flag on the node, not a dynamic_cast per child per walk.)
      if (!child->contributesToSceneBounds())
        continue;

      // This child and its descendants in WORLD space: its combined local box
      // through its CACHED world matrix, the same 8-corner AABB re-fit this walk
      // used to do on a fresh vtkMatrix4x4 copy (getWorldTransform) per child.
      // Bit-identical for the affine transforms a scene holds (w stays 1).
      const cvc::bounding_box b = child->getCombinedWorldBoundingBox();

      // Skip invalid bounding boxes (an invalid box comes back unchanged)
      if (b[0] > b[3] || b[1] > b[4] || b[2] > b[5]) {
        continue;
      }

      // Merge with combined bounds
      if (first) {
        combinedBounds = b;
        first = false;
      } else {
        combinedBounds[0] = std::min(combinedBounds[0], b[0]);
        combinedBounds[1] = std::min(combinedBounds[1], b[1]);
        combinedBounds[2] = std::min(combinedBounds[2], b[2]);
        combinedBounds[3] = std::max(combinedBounds[3], b[3]);
        combinedBounds[4] = std::max(combinedBounds[4], b[4]);
        combinedBounds[5] = std::max(combinedBounds[5], b[5]);
      }
    }
  }

  return combinedBounds;
}

// Multi-object graphics management
std::shared_ptr<GraphicsNode> SceneGraph::addGraphics(const std::string &name,
                                                      const cvc::geometry &geom) {
  cvc::thread_info ti(m_ctx, BOOST_CURRENT_FUNCTION);

  // Check if name already exists
  if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
    m_ctx.log(0,
              "SceneGraph::addGraphics: Graphics object '" + name + "' already exists, replacing");
    removeGraphics(name);
  }

  // Create new geometry node using template factory (automatically creates proper state path)
  auto graphicsNode = m_graphicsRoot->addGraphicsChild<GeometryNode>(name);
  graphicsNode->setGeometry(geom);

  // One registration path for every node kind: it owns the lookup map, the
  // bounds tracking AND the world-bounds refresh. Inlining those here is what
  // let the grid miss added geometry for so long.
  registerGraphics(name, graphicsNode);

  // Remove null graphic since we now have real graphics
  removeNullGraphicIfPresent();

  // Notify dialogs that children collection has changed
  try {
    m_graphicsRoot->getState("children").touch();
  } catch (...) {
    // State might not exist yet
  }

  // Emit signal for dialogs
  graphicsChanged();

  return graphicsNode;
}

std::shared_ptr<GraphicsNode> SceneGraph::addGraphics(const std::string &name) {
  cvc::thread_info ti(m_ctx, BOOST_CURRENT_FUNCTION);

  // Check if name already exists
  if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
    m_ctx.log(0,
              "SceneGraph::addGraphics: Graphics object '" + name + "' already exists, replacing");
    removeGraphics(name);
  }

  // Create new empty geometry node using template factory (automatically creates proper state path)
  auto graphicsNode = m_graphicsRoot->addGraphicsChild<GeometryNode>(name);

  registerGraphics(name, graphicsNode);

  // Remove null graphic since we now have real graphics
  removeNullGraphicIfPresent();

  // Notify dialogs that children collection has changed
  try {
    m_graphicsRoot->getState("children").touch();
  } catch (...) {
    // State might not exist yet
  }

  // Emit signal for dialogs
  graphicsChanged();

  return graphicsNode;
}

bool SceneGraph::hasGraphics(const std::string &name) const {
  return m_graphicsNodes.find(name) != m_graphicsNodes.end();
}

void SceneGraph::removeGraphics(const std::string &name) {
  cvc::thread_info ti(m_ctx, BOOST_CURRENT_FUNCTION);

  auto it = m_graphicsNodes.find(name);
  if (it == m_graphicsNodes.end()) {
    m_ctx.log(0, "SceneGraph::removeGraphics: Graphics object '" + name + "' not found");
    return;
  }

  auto graphicsNode = it->second;

  // Unlink from the graphics root and drop it from the lookup map. This may drop
  // the last reference and destroy the node. That is safe: scene nodes run their
  // state handlers synchronously (no handler thread can be touching this node),
  // and any main-thread callback still queued for it is weak-guarded (see
  // SceneNode::runOnMainThread), so it becomes a no-op once the node is gone.
  // No drain or join is needed — teardown here is race-free by construction.
  m_graphicsRoot->removeGraphicsChild(graphicsNode);
  m_graphicsNodes.erase(it);

  // Explicitly notify state tree that children have changed
  // This triggers dataChanged signals that dialogs are listening to
  try {
    m_graphicsRoot->getState("children").touch();
  } catch (...) {
    // State might not exist yet during initialization
  }

  // Emit signal for dialogs
  graphicsChanged();

  // If scene is now empty, add null graphic back
  ensureNullGraphicIfEmpty();

  // The set of graphics changed: let the grid shrink back to what is left.
  refreshWorldBounds();
}

std::shared_ptr<GraphicsNode> SceneGraph::getGraphics(const std::string &name) {
  auto it = m_graphicsNodes.find(name);
  if (it != m_graphicsNodes.end()) {
    return it->second;
  }
  return nullptr;
}

void SceneGraph::registerGraphics(const std::string &name, std::shared_ptr<GraphicsNode> node) {
  if (node) {
    m_graphicsNodes[name] = node;
    trackNodeBounds(node);
    // The set of graphics changed: the grid must enclose the new node, and
    // tracking alone would not do it (that only reacts to later MOVES).
    refreshWorldBounds();
  }
}

void SceneGraph::trackNodeBounds(const std::shared_ptr<GraphicsNode> &node) {
  if (!node)
    return;
  // When the node moves, FLAG the world bounds as stale; processEvents() runs
  // the recompute once, on the owner thread (updateGrid touches the VTK
  // grid/axis actors), however many nodes moved. transformChanged fires on the
  // mover's thread, so this only touches atomics and the render flag. The first
  // move since the last pump also requests a render, which the per-move
  // postEvent used to do as a side effect. The connection is owned here and dies
  // with the SceneGraph, so the captured `this` is safe.
  m_boundsConns.push_back(node->transformChanged.connect([this](GraphicsNode *) {
    markContentChanged(); // whether or not it left the world box
    if (!m_boundsDirty.exchange(true, std::memory_order_acq_rel))
      requestRender();
  }));
}

void SceneGraph::refreshWorldBounds() {
  // The grid is sized ONLY from here and from onGraphicsBoundsChanged(). Before
  // this existed the latter was the only caller, and it fires on
  // GraphicsNode::transformChanged — so the grid tracked geometry that MOVED and
  // was blind to geometry that was merely ADDED. A scene built once and never
  // animated therefore drew a grid that did not enclose its own contents, while
  // the graphics root's bbox (which auto-syncs to its children) did — which is
  // how the two came to disagree on screen.
  //
  // Compounding it, addGraphics() did not call registerGraphics(): it inlined
  // the same map-assign + trackNodeBounds pair, so four of five registration
  // sites had drifted apart and a hook added to one of them reached none of the
  // others. They all funnel through registerGraphics() now.
  //
  // Pinned by cvcgl_grid_bounds (8 failing checks before this change).
  //
  // Marshalled onto the owner thread: updateGrid() touches VTK actors.
  //
  // Coalesced: while one recompute is queued, further adds/removes ride on it --
  // it reads the scene when it RUNS, so it sees them all. Building a scene of N
  // nodes therefore costs one walk instead of N walks of up to N nodes each.
  if (m_boundsRefreshQueued.exchange(true, std::memory_order_acq_rel))
    return;
  postEvent([this]() {
    m_boundsRefreshQueued.store(false, std::memory_order_release);
    // This authoritative walk sees every move made so far, so a grow-only
    // follow-up still flagged for them would walk again for nothing. A node
    // that moves AFTER this point (a later event in the same drain) raises the
    // flag again and is covered at the end of processEvents().
    m_boundsDirty.store(false, std::memory_order_release);
    updateGrid(computeGraphicsBounds());
    requestRender();
  });
}

void SceneGraph::onGraphicsBoundsChanged() {
  // The world box sizes the grid, the axis and the graphics root's own box.
  // With all three hidden nothing on screen follows it, so the walk waits until
  // one is shown (catchUpWorldBounds, or the end of processEvents) -- for a
  // scene posing registered nodes every frame under hidden chrome that is a
  // whole walk of the scene per frame saved. Meanwhile the root's bounds (and
  // its "<prefix>.graphics.root.bounds" state key) do not grow over moves;
  // computeGraphicsBounds() is the live answer. A refresh (add/remove) still
  // runs: it is rare.
  if (!worldBoundsShown()) {
    m_boundsGrowDeferred.store(true, std::memory_order_relaxed);
    return;
  }
  cvc::bounding_box b = computeGraphicsBounds();
  // Grow-only: only resize the grid when graphics have moved OUTSIDE the current
  // world box (a node "left" it) — so in-bounds animation never jitters the grid.
  bool outside = false;
  for (int i = 0; i < 3; ++i)
    if (b[i] < m_worldBounds[i] || b[i + 3] > m_worldBounds[i + 3])
      outside = true;
  if (!outside)
    return;
  cvc::bounding_box grown;
  for (int i = 0; i < 3; ++i) {
    grown[i] = std::min(b[i], m_worldBounds[i]);
    grown[i + 3] = std::max(b[i + 3], m_worldBounds[i + 3]);
  }
  updateGrid(grown);
  requestRender();
}

// Volume graphics management
std::shared_ptr<VolumeNode> SceneGraph::addGraphics(const std::string &name,
                                                    const cvc::volume &vol) {
  cvc::thread_info ti(m_ctx, BOOST_CURRENT_FUNCTION);

  // Check if name already exists
  if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
    m_ctx.log(0, "SceneGraph::addGraphics: Volume '" + name + "' already exists, replacing");
    removeGraphics(name);
  }

  // Create new volume node using template factory (automatically creates proper state path)
  auto volumeNode = m_graphicsRoot->addGraphicsChild<VolumeNode>(name);
  volumeNode->setVolume(vol);

  registerGraphics(name, volumeNode);

  // Remove null graphic since we now have real graphics
  removeNullGraphicIfPresent();

  // Update multi-volume rendering if needed
  updateVolumeRendering();

  // Notify dialogs that children collection has changed
  try {
    m_graphicsRoot->getState("children").touch();
  } catch (...) {
    // State might not exist yet
  }

  // Emit signal for dialogs
  graphicsChanged();

  return volumeNode;
}

cvc::bounding_box SceneGraph::computeVolumeBounds() const {
  cvc::bounding_box combinedBounds;
  bool first = true;

  // Helper function to process volume graphics nodes recursively
  std::function<void(const std::shared_ptr<GraphicsNode> &)> processBounds =
      [&](const std::shared_ptr<GraphicsNode> &node) {
        if (!node)
          return;

        // Check if this is a VolumeNode
        if (auto volNode = std::dynamic_pointer_cast<VolumeNode>(node)) {
          // Get volume if available
          if (volNode->hasVolume() && volNode->getVolume()) {
            cvc::bounding_box volBounds = volNode->getVolume()->boundingBox();

            if (first) {
              combinedBounds = volBounds;
              first = false;
            } else {
              // Expand to include this volume
              combinedBounds[0] = std::min(combinedBounds[0], volBounds[0]);
              combinedBounds[1] = std::min(combinedBounds[1], volBounds[1]);
              combinedBounds[2] = std::min(combinedBounds[2], volBounds[2]);
              combinedBounds[3] = std::max(combinedBounds[3], volBounds[3]);
              combinedBounds[4] = std::max(combinedBounds[4], volBounds[4]);
              combinedBounds[5] = std::max(combinedBounds[5], volBounds[5]);
            }
          }
        }

        // Process children recursively
        for (const auto &child : node->getGraphicsChildren()) {
          processBounds(child);
        }
      };

  // Start from unified graphics root (includes volumes)
  if (m_graphicsRoot) {
    processBounds(m_graphicsRoot);
  }

  return combinedBounds;
}

void SceneGraph::enableMultiVolumeRendering(bool enable) {
  if (m_multiVolumeRenderingEnabled == enable) {
    return; // No change
  }

  m_multiVolumeRenderingEnabled = enable;

  if (enable) {
    setupMultiVolumeRendering();
  } else {
    teardownMultiVolumeRendering();
  }
}

bool SceneGraph::isMultiVolumeRenderingEnabled() const { return m_multiVolumeRenderingEnabled; }

void SceneGraph::setupMultiVolumeRendering() {
  if (!m_renderer) {
    return;
  }

  // Create multi-volume if not already created
  if (!m_multiVolume) {
    m_multiVolume = vtkSmartPointer<vtkMultiVolume>::New();
  }

  // Collect all volume graphics nodes
  auto allVolumes = getAllVolumeGraphics();

  if (allVolumes.size() <= 1) {
    return; // No need for multi-volume rendering with 0 or 1 volume
  }

  // TODO: Implement proper multi-volume rendering with GraphicsNode architecture
  // For now, individual volumes are rendered separately
  // Remove individual volume props from renderer
  /*
  for (const auto& volNode : allVolumes) {
      volNode->removeFromRenderer(m_renderer);
  }

  // Add all volumes to the multi-volume
  int port = 0;
  for (const auto& volNode : allVolumes) {
      // Note: vtkMultiVolume SetVolume takes a port number, not a transform
      // Transforms should be already applied to individual vtkVolume actors
      m_multiVolume->SetVolume(vol, port++);
  }

  // Add multi-volume to renderer
  m_renderer->AddViewProp(m_multiVolume);
  */
}

void SceneGraph::teardownMultiVolumeRendering() {
  if (!m_renderer || !m_multiVolume) {
    return;
  }

  // TODO: Implement proper multi-volume teardown with GraphicsNode architecture
  // For now, individual volumes are rendered separately
  /*
  // Remove multi-volume from renderer
  m_renderer->RemoveViewProp(m_multiVolume);

  // Re-add individual volume props
  auto allVolumes = getAllVolumeGraphics();
  for (const auto& volNode : allVolumes) {
      volNode->addToRenderer(m_renderer);
  }
  */
}

void SceneGraph::updateVolumeRendering() {
  if (!m_renderer) {
    return;
  }

  size_t volumeCount = getVolumeGraphicsCount();

  // Enable multi-volume rendering if we have more than 1 volume
  if (volumeCount > 1 && !m_multiVolumeRenderingEnabled) {
    enableMultiVolumeRendering(true);
  } else if (volumeCount <= 1 && m_multiVolumeRenderingEnabled) {
    enableMultiVolumeRendering(false);
  }
}

void SceneGraph::ensureNullGraphicIfEmpty() {
  // With new architecture: NullGraphicNode IS the graphics root, always present
  // No need to add/remove it
}

void SceneGraph::removeNullGraphicIfPresent() {
  // With new architecture: NullGraphicNode IS the graphics root, always present
  // No need to add/remove it
}

// ── lighting ────────────────────────────────────────────────────────────────
// Directional lights, owned by the SCENE rather than by a renderer, so they
// survive setRenderer and can be re-applied to a second renderer. Azimuth is a
// compass bearing (0 = +Y, growing towards +X) and elevation is degrees above
// the horizon, which is how you actually describe a sun; the unit vector is
// derived here so callers never hand-roll the trigonometry.

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
// Far enough away that vtkLight's position reads as a direction. VTK has no
// "infinite" directional light: a scene light is positional-or-not, and a
// non-positional one uses position-minus-focal-point as its direction.
constexpr double kSunDistance = 1.0e4;
} // namespace

std::shared_ptr<cvc::gl::LightNode> SceneGraph::addLight(const std::string &name) {
  cvc::thread_info ti(m_ctx, BOOST_CURRENT_FUNCTION);
  if (m_graphicsNodes.find(name) != m_graphicsNodes.end())
    removeGraphics(name);
  // Same factory the other nodes use, so the light gets a proper state path AND
  // lands in the graphics child list — which is what the light traversal walks.
  auto node = m_graphicsRoot->addGraphicsChild<cvc::gl::LightNode>(name);
  m_graphicsNodes[name] = node;
  removeNullGraphicIfPresent();
  try {
    m_graphicsRoot->getState("children").touch();
  } catch (...) {
  }
  applyLights(); // a new light lights the scene immediately
  return node;
}

void SceneGraph::lightsChanged() { applyLights(); }

void SceneGraph::beginLightBatch() { ++m_lightBatchDepth; }

void SceneGraph::endLightBatch() {
  if (m_lightBatchDepth > 0 && --m_lightBatchDepth == 0 && m_lightsDirty) {
    m_lightsDirty = false;
    applyLights();
  }
}

void SceneGraph::applyLights() {
  // Deferred inside a batch: rebuilding the whole light set per edit is what
  // turns a rig update into a shadow-bake storm.
  if (m_lightBatchDepth > 0) {
    m_lightsDirty = true;
    return;
  }
  if (!m_renderer)
    return;
  m_renderer->RemoveAllLights();
  for (const auto &l : m_lights) {
    vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
    light->SetLightTypeToSceneLight();
    if (l.kind == LightDesc::Kind::Spot) {
      // Positional + a cone under 90 degrees. Both matter: VTK's shadow baker
      // skips any positional light whose cone reaches 90, and it is the cone
      // that concentrates the shadow map onto the lit area.
      light->SetPositional(true);
      light->SetPosition(l.px, l.py, l.pz);
      light->SetFocalPoint(l.tx, l.ty, l.tz);
      light->SetConeAngle(l.cone);
    } else {
      light->SetPositional(false); // directional: only the direction matters
      const double ce = std::cos(l.el * kDeg), se = std::sin(l.el * kDeg);
      light->SetPosition(kSunDistance * ce * std::sin(l.az * kDeg),
                         -kSunDistance * ce * std::cos(l.az * kDeg), kSunDistance * se);
      light->SetFocalPoint(0.0, 0.0, 0.0);
    }
    light->SetColor(l.r, l.g, l.b);
    light->SetIntensity(l.intensity);
    m_renderer->AddLight(light);
  }
  // LIGHTS THAT ARE NODES. Gathered from the graph, so a light is lit simply by
  // being in the scene — and its position is the node's WORLD transform, which
  // is what makes a light parented to a moving actor travel with it.
  std::size_t nodeLights = 0, nodeLightsDefined = 0;
  for (const auto &ln : getAllGraphicsOfType<cvc::gl::LightNode>()) {
    if (!ln)
      continue;
    ++nodeLightsDefined;
    if (!ln->isVisible()) // hiding a light turns it off, as expected
      continue;
    ++nodeLights;
    double px, py, pz, tx, ty, tz, r, g, b;
    ln->worldPosition(px, py, pz);
    ln->target(tx, ty, tz);
    ln->color(r, g, b);
    vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
    light->SetLightTypeToSceneLight();
    if (ln->kind() == cvc::gl::LightNode::Kind::Directional) {
      light->SetPositional(false);
      light->SetPosition(px, py, pz);
      light->SetFocalPoint(tx, ty, tz);
    } else {
      light->SetPositional(true);
      light->SetPosition(px, py, pz);
      light->SetFocalPoint(tx, ty, tz);
      light->SetConeAngle(ln->cone());
    }
    light->SetColor(r, g, b);
    light->SetIntensity(ln->intensity());
    m_renderer->AddLight(light);
  }

  // With NO lights defined at all, hand the renderer back its default headlight
  // rather than leaving the scene unlit. But if the scene HAS light nodes and
  // they are merely hidden, respect that: silently substituting a headlight for
  // the light someone just switched off makes hiding look broken.
  if (m_lights.empty() && nodeLightsDefined == 0)
    m_renderer->CreateLight();
  requestRender();
}

int SceneGraph::addDirectionalLight(double azimuthDeg, double elevationDeg, double r, double g,
                                    double b, double intensity) {
  LightDesc d;
  d.id = m_nextLightId++;
  d.kind = LightDesc::Kind::Directional;
  d.az = azimuthDeg;
  d.el = elevationDeg;
  d.r = r;
  d.g = g;
  d.b = b;
  d.intensity = intensity;
  m_lights.push_back(d);
  applyLights();
  return d.id;
}

namespace {
// VTK drops a positional light with cone >= 90 from the shadow bake, so a cone
// of "90" means no shadow rather than a wide one. Clamp instead of surprising.
inline double clampCone(double deg) { return deg < 0.5 ? 0.5 : (deg > 89.5 ? 89.5 : deg); }
} // namespace

int SceneGraph::addSpotLight(double x, double y, double z, double tx, double ty, double tz,
                             double coneDeg, double r, double g, double b, double intensity) {
  LightDesc d;
  d.id = m_nextLightId++;
  d.kind = LightDesc::Kind::Spot;
  d.px = x;
  d.py = y;
  d.pz = z;
  d.tx = tx;
  d.ty = ty;
  d.tz = tz;
  d.cone = clampCone(coneDeg);
  d.r = r;
  d.g = g;
  d.b = b;
  d.intensity = intensity;
  m_lights.push_back(d);
  applyLights();
  return d.id;
}

int SceneGraph::addFillLight(double x, double y, double z, double tx, double ty, double tz,
                             double r, double g, double b, double intensity) {
  LightDesc d;
  d.id = m_nextLightId++;
  d.kind = LightDesc::Kind::Spot;
  d.px = x;
  d.py = y;
  d.pz = z;
  d.tx = tx;
  d.ty = ty;
  d.tz = tz;
  d.cone = 90.0; // >= 90 on purpose: VTK excludes it from the shadow bake
  d.r = r;
  d.g = g;
  d.b = b;
  d.intensity = intensity;
  m_lights.push_back(d);
  applyLights();
  return d.id;
}

void SceneGraph::setLightPosition(int id, double x, double y, double z) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.px = x;
      l.py = y;
      l.pz = z;
      applyLights();
      return;
    }
}

void SceneGraph::setLightTarget(int id, double tx, double ty, double tz) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.tx = tx;
      l.ty = ty;
      l.tz = tz;
      applyLights();
      return;
    }
}

void SceneGraph::setLightCone(int id, double coneDeg) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.cone = clampCone(coneDeg);
      applyLights();
      return;
    }
}

bool SceneGraph::lightCastsShadow(int id) const {
  for (const auto &l : m_lights)
    if (l.id == id)
      return l.kind == LightDesc::Kind::Directional || l.cone < 90.0;
  return false;
}

void SceneGraph::setLightDirection(int id, double azimuthDeg, double elevationDeg) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.az = azimuthDeg;
      l.el = elevationDeg;
      applyLights();
      return;
    }
}

void SceneGraph::setLightColor(int id, double r, double g, double b) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.r = r;
      l.g = g;
      l.b = b;
      applyLights();
      return;
    }
}

void SceneGraph::setLightIntensity(int id, double intensity) {
  for (auto &l : m_lights)
    if (l.id == id) {
      l.intensity = intensity;
      applyLights();
      return;
    }
}

void SceneGraph::removeLight(int id) {
  for (auto it = m_lights.begin(); it != m_lights.end(); ++it)
    if (it->id == id) {
      m_lights.erase(it);
      applyLights();
      return;
    }
}

void SceneGraph::clearLights() {
  m_lights.clear();
  applyLights();
}

std::size_t SceneGraph::numLights() const { return m_lights.size(); }

// Free a shadow pass's GL objects against the window they were made in, before
// the last reference to the pass goes. Only the baker holds any -- its FBO and
// shadow maps -- and vtkShadowMapBakerPass's destructor frees neither: it logs
// "FrameBufferObject/ShadowMaps/LightCameras should have been deleted in
// ReleaseGraphicsResources()" and they leak. GL, so the owner thread only.
// False when there is no window to free them against.
static bool releaseShadowGL(vtkRenderer *ren, vtkRenderPass *pass) {
  vtkRenderWindow *w = ren ? ren->GetRenderWindow() : nullptr;
  if (!w || !pass)
    return false;
  if (!w->IsCurrent())
    w->MakeCurrent();
  pass->ReleaseGraphicsResources(w);
  return true;
}

// A shadow baker that only re-bakes every Nth frame. The base pass re-renders the
// whole scene depth from every light whenever geometry has moved; for a scene that
// deforms every frame (a swaying forest) that is the dominant cost, yet the shadows
// barely change between frames. On a skip frame we call SetUpToDate() instead of
// baking, which leaves the last-baked depth maps in place and tells the downstream
// vtkShadowMapPass they are current, so it samples them — a stale-by-a-frame shadow
// that is invisible for slow motion. Interval is read live from the SceneGraph.
class StridedShadowBaker : public vtkShadowMapBakerPass {
public:
  static StridedShadowBaker *New();
  vtkTypeMacro(StridedShadowBaker, vtkShadowMapBakerPass);

  int Interval = 1;
  // Set by SceneGraph::invalidateShadowBake(): bake on the next Render() however
  // the interval falls, then clear. The cadence counter is left alone.
  bool ForceNext = false;

  // GL objects made by a bake: the FBO and the shadow maps. None before the
  // first bake (or after a release), when the window may have no context yet.
  bool HoldsGLObjects() const {
    return this->FrameBufferObject != nullptr || this->ShadowMaps != nullptr;
  }

  // The map size, applied to maps already baked too. VTK reads Resolution only
  // when it creates a map, so a baker that has baked would go on drawing into
  // its old maps: free them and the FBO against ren's window, and make the next
  // Render() bake new ones (VTK's own test then sees every light as changed).
  void SetMapResolution(unsigned int pixels, vtkRenderer *ren) {
    if (pixels == this->Resolution)
      return;
    this->SetResolution(pixels);
    if (!HoldsGLObjects() || !releaseShadowGL(ren, this))
      return;
    this->LastRenderTime = vtkTimeStamp();
    ForceNext = true;
  }

  void Render(const vtkRenderState *s) override {
    // Skipping a bake is only safe while the SET OF SHADOW-CASTING LIGHTS is unchanged.
    //
    // vtkShadowMapBakerPass sizes its ShadowMaps vector inside Render(), and
    // only when NeedUpdate is set. SetUpToDate() clears NeedUpdate without
    // resizing, so on a skipped frame the downstream vtkShadowMapPass walks the
    // renderer's CURRENT light list while indexing the PREVIOUS bake's vector:
    //
    //     map = (*GetShadowMaps())[shadowingLightIndex];   // no bounds check
    //     map->Activate();
    //
    // Add a caster (raise the wash count, un-hide a light, "All on") and that
    // index runs past size() and dereferences garbage. REMOVE one and the loop
    // just ends early with every index valid — which is exactly why decreasing
    // the wash count looked fine and increasing it did not.
    const std::size_t casters = countShadowCasters(s);
    const bool due = (Interval <= 1) || (m_counter % static_cast<unsigned long>(Interval)) == 0;
    // On a due frame, bake only if something that can change the maps changed
    // (shadowInputsChanged). VTK alone re-bakes when ANY prop changed: a vehicle
    // marked a non-caster moving, a fog texture repainted, an overlay restyled.
    const bool inputsChanged = due && shadowInputsChanged(s);
    if (ForceNext || casters != m_bakedCasters || inputsChanged) {
      // VTK's own test can miss what ours saw: a prop that started or stopped
      // casting has not itself changed. Make it bake whatever its test says.
      if (inputsChanged)
        this->LastRenderTime = vtkTimeStamp();
      this->Superclass::Render(s); // bakes if anything at all changed; resizes ShadowMaps
      m_bakedCasters = casters;
      ForceNext = false;
      if (this->GetNeedUpdate())
        noteBake(s);
    } else {
      this->SetUpToDate(); // reuse the last-baked maps this frame
    }
    ++m_counter;
  }

protected:
  StridedShadowBaker() = default;

  // What the downstream pass will count when it walks the light list: a light
  // that is switched on AND satisfies LightCreatesShadow (positional with a
  // cone below 90, or directional). Must match that predicate exactly, or the
  // guard above protects the wrong number.
  std::size_t countShadowCasters(const vtkRenderState *s) {
    if (!s || !s->GetRenderer())
      return 0;
    vtkLightCollection *lc = s->GetRenderer()->GetLights();
    if (!lc)
      return 0;
    std::size_t n = 0;
    lc->InitTraversal();
    while (vtkLight *l = lc->GetNextItem())
      if (l->GetSwitch() && this->LightCreatesShadow(l))
        ++n;
    return n;
  }

  // The props whose depth the maps hold: visible, and not marked non-casters
  // (GraphicsNode::setCastsShadow). Identified by count and two order-free
  // sums of their addresses, so one joining or leaving shows.
  struct CasterSet {
    std::size_t count = 0;
    std::uint64_t sum = 0, mix = 0;
    void add(const vtkProp *p) {
      std::uint64_t z = reinterpret_cast<std::uintptr_t>(p) + 0x9e3779b97f4a7c15ull; // splitmix64
      z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
      z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
      sum += reinterpret_cast<std::uintptr_t>(p);
      mix += z ^ (z >> 31);
      ++count;
    }
    bool operator!=(const CasterSet &o) const {
      return count != o.count || sum != o.sum || mix != o.mix;
    }
  };

  // vtkShadowMapBakerPass's own test, narrowed to what the maps depend on: a
  // light (as VTK checks them), or a CASTING prop -- changed since the last
  // bake, joined or left. A non-caster's MTime is ignored, which is the point:
  // moving or repainting one never forces a bake. (The bake itself draws only
  // casters, through ShadowCasterFilterPass.)
  bool shadowInputsChanged(const vtkRenderState *s) {
    vtkRenderer *r = s ? s->GetRenderer() : nullptr;
    if (!r || !m_baked)
      return true;
    const vtkMTimeType t = m_lastBake.GetMTime();
    vtkLightCollection *lights = r->GetLights();
    if (lights->GetMTime() > t)
      return true;
    vtkCollectionSimpleIterator lit;
    lights->InitTraversal(lit);
    while (vtkLight *l = lights->GetNextLight(lit))
      if (l->GetMTime() > t)
        return true;
    CasterSet now;
    vtkPropCollection *props = r->GetViewProps();
    vtkCollectionSimpleIterator pit;
    props->InitTraversal(pit);
    while (vtkProp *p = props->GetNextProp(pit)) {
      if (!p->GetVisibility() || !GraphicsNode::propCastsShadow(p))
        continue;
      if (p->GetMTime() > t || casterGeometryChanged(p, t))
        return true;
      now.add(p);
    }
    // Forget mappers no longer drawn (a key whose address a new mapper reuses
    // is harmless: the new one's MTime differs, so its input is looked up).
    if (m_mapperInputs.size() > 2 * now.count + 64)
      m_mapperInputs.clear();
    return now != m_bakedSet;
  }

  // vtkActor::GetMTime() leaves out the mapper and its input, so a caster
  // deformed in place (GeometryNode::updateVertices / updateNormals /
  // updateColors -- a swaying tree) changes only those. Their MTimes, read
  // as they are: no Update(), no GetRedrawMTime().
  //
  // Finding the input is an executive lookup (vtkAlgorithm::GetInputDataObject:
  // port checks and two information-key reads), several times the cost of the
  // rest of this check, and it runs for every caster on every due frame. The
  // input changes only through SetInputData / SetInputConnection, which mark
  // the mapper modified, so it is looked up again only when the mapper's MTime
  // moves (or the input it named is gone).
  bool casterGeometryChanged(vtkProp *p, vtkMTimeType t) {
    auto *actor = vtkActor::SafeDownCast(p);
    vtkMapper *mapper = actor ? actor->GetMapper() : nullptr;
    if (!mapper)
      return false;
    const vtkMTimeType mapperTime = mapper->GetMTime();
    if (mapperTime > t)
      return true;
    MapperInput &in = m_mapperInputs[mapper];
    if (in.mapperTime != mapperTime || !in.input) {
      in.mapperTime = mapperTime;
      in.input = mapper->GetInputDataObject(0, 0);
    }
    vtkDataObject *input = in.input;
    return input && input->GetMTime() > t;
  }

  void noteBake(const vtkRenderState *s) {
    m_lastBake.Modified();
    m_baked = true;
    m_bakedSet = CasterSet();
    if (vtkRenderer *r = s ? s->GetRenderer() : nullptr) {
      vtkPropCollection *props = r->GetViewProps();
      vtkCollectionSimpleIterator pit;
      props->InitTraversal(pit);
      while (vtkProp *p = props->GetNextProp(pit))
        if (p->GetVisibility() && GraphicsNode::propCastsShadow(p))
          m_bakedSet.add(p);
    }
  }

private:
  unsigned long m_counter = 0;
  // (std::size_t)-1 so the first frame always bakes rather than matching 0.
  std::size_t m_bakedCasters = static_cast<std::size_t>(-1);
  bool m_baked = false;    // has a real bake happened yet
  vtkTimeStamp m_lastBake; // when it did
  CasterSet m_bakedSet;    // the casters it drew
  // casterGeometryChanged's cache: per mapper, its MTime when its input was
  // last looked up, and that input (weak: the mapper owns it).
  struct MapperInput {
    vtkMTimeType mapperTime = 0;
    vtkWeakPointer<vtkDataObject> input;
  };
  std::unordered_map<const vtkMapper *, MapperInput> m_mapperInputs;
  StridedShadowBaker(const StridedShadowBaker &) = delete;
  void operator=(const StridedShadowBaker &) = delete;
};
vtkStandardNewMacro(StridedShadowBaker);

// The baker's depth pass, restricted to the props that cast. vtkShadowMapBakerPass
// hands its opaque sequence every visible prop; this one passes on only those
// GraphicsNode::propCastsShadow() accepts, so a non-caster is left out of the
// maps rather than frozen into them at its last-baked pose.
class ShadowCasterFilterPass : public vtkRenderPass {
public:
  static ShadowCasterFilterPass *New();
  vtkTypeMacro(ShadowCasterFilterPass, vtkRenderPass);

  vtkSmartPointer<vtkRenderPass> Delegate; // VTK's camera -> lights -> opaque sequence

  void Render(const vtkRenderState *s) override {
    this->NumberOfRenderedProps = 0;
    if (!this->Delegate || !s)
      return;
    m_props.clear();
    vtkProp **all = s->GetPropArray();
    for (int i = 0; i < s->GetPropArrayCount(); ++i)
      if (GraphicsNode::propCastsShadow(all[i]))
        m_props.push_back(all[i]);
    vtkRenderState casters(s->GetRenderer());
    casters.SetPropArrayAndCount(m_props.data(), static_cast<int>(m_props.size()));
    casters.SetFrameBuffer(s->GetFrameBuffer());
    casters.SetRequiredKeys(s->GetRequiredKeys());
    this->Delegate->Render(&casters);
    this->NumberOfRenderedProps = this->Delegate->GetNumberOfRenderedProps();
  }

  void ReleaseGraphicsResources(vtkWindow *w) override {
    if (this->Delegate)
      this->Delegate->ReleaseGraphicsResources(w);
  }

protected:
  ShadowCasterFilterPass() = default;

private:
  std::vector<vtkProp *> m_props;
  ShadowCasterFilterPass(const ShadowCasterFilterPass &) = delete;
  void operator=(const ShadowCasterFilterPass &) = delete;
};
vtkStandardNewMacro(ShadowCasterFilterPass);

// Shadow settings mirrored into cvc::state at "<prefix>.shadows". Created lazily
// (the ctor runs before the renderer exists) and guarded against re-entry: a
// state write calls the setter, which would otherwise write state again.
void SceneGraph::syncShadowState() {
  if (m_applyingShadowState)
    return;
  // Capture the CURRENT values BEFORE creating the settings object. Its
  // constructor seeds its own defaults, which fires handleStateChanged, which
  // calls back into the setters — so building `v` afterwards would read values
  // the seeding had already clobbered (a setShadowResolution(2048) landed as
  // 1024, the default). The guard also covers construction, because that
  // callback re-enters this function and would otherwise create a second
  // settings object while the first is still being built.
  cvc::gl::ShadowSettings::Values v;
  v.enabled = m_shadowsEnabled;
  v.resolution = m_shadowResolution;
  v.interval = m_shadowInterval;

  if (!m_shadowSettings) {
    m_applyingShadowState = true;
    m_shadowSettings = std::make_unique<cvc::gl::ShadowSettings>(
        m_ctx, cvc::gl::ShadowSettings::sceneStatePath(m_statePrefix),
        [this](cvc::gl::ShadowSettings::Values nv) {
          auto apply = [this, nv]() {
            m_applyingShadowState = true;
            setShadowsEnabled(nv.enabled);
            setShadowResolution(nv.resolution);
            setShadowUpdateInterval(nv.interval);
            m_applyingShadowState = false;
          };
          // The setters build render passes and free GL objects, so they run on
          // the owner thread; a write from elsewhere (a replicated peer, a script
          // thread) is applied at the next processEvents().
          if (onOwnerThread())
            apply();
          else
            postEvent(apply);
        });
    m_applyingShadowState = false;
  }
  m_shadowSettings->set(v);
}

void SceneGraph::setShadowUpdateInterval(int frames) {
  m_shadowInterval = frames < 1 ? 1 : frames;
  if (auto *b = StridedShadowBaker::SafeDownCast(m_shadowBaker))
    b->Interval = m_shadowInterval;
  syncShadowState();
  requestRender();
}

void SceneGraph::invalidateShadowBake() {
  if (auto *b = StridedShadowBaker::SafeDownCast(m_shadowBaker)) {
    b->ForceNext = true;
    requestRender();
  }
}

void SceneGraph::setShadowResolution(int pixels) {
  m_shadowResolution = pixels < 64 ? 64 : pixels;
  if (auto *b = StridedShadowBaker::SafeDownCast(m_shadowBaker))
    b->SetMapResolution(static_cast<unsigned int>(m_shadowResolution), m_renderer);
  syncShadowState();
  requestRender();
}

bool SceneGraph::setShadowsEnabled(bool enabled) {
  if (!m_renderer) {
    m_shadowsEnabled = false;
    return false;
  }
  if (enabled && m_shadowPass && m_renderer->GetPass() == m_shadowPass) {
    // Already on: keep the chain and its baked maps. Each setter's state echo
    // lands here, so rebuilding would replace the baker on every interval or
    // resolution change.
    m_shadowsEnabled = true;
    return true;
  }
  removeShadowPasses();
  m_shadowsEnabled = false;
  if (!enabled) {
    m_renderer->SetPass(nullptr);
    syncShadowState();
    requestRender();
    return true;
  }
  installShadowPasses();
  m_shadowsEnabled = true;
  syncShadowState();
  requestRender();
  return true;
}

void SceneGraph::installShadowPasses() {
  // VTK's shadow maps are render PASSES, not a renderer flag: the baker renders
  // the scene once per light into a depth map, and the shadow pass consumes
  // those while drawing. They have to sit inside a camera pass, or the light's
  // view never gets set up.
  vtkSmartPointer<StridedShadowBaker> baker = vtkSmartPointer<StridedShadowBaker>::New();
  baker->Interval = m_shadowInterval;
  baker->SetResolution(m_shadowResolution); // crisper than VTK's low 256 default
  // Bake casters only (GraphicsNode::setCastsShadow): wrap VTK's own depth pass.
  vtkSmartPointer<ShadowCasterFilterPass> casterFilter =
      vtkSmartPointer<ShadowCasterFilterPass>::New();
  casterFilter->Delegate = baker->GetOpaqueSequence();
  baker->SetOpaqueSequence(casterFilter);
  vtkSmartPointer<vtkShadowMapPass> shadows = vtkSmartPointer<vtkShadowMapPass>::New();
  shadows->SetShadowMapBakerPass(baker);

  // The shadow pass draws only the OPAQUE layer (with shadows). On its own it
  // silently drops everything else — translucent geometry, volumes, 2-D overlays —
  // so a scene with a VolumeNode (sea, cloud slab, any transfer-function volume)
  // renders as if the volume weren't there the moment shadows are switched on.
  // Follow it with the rest of VTK's standard layer order so the full scene draws:
  // translucent geometry, then volumes (ray-cast over the shadowed opaque depth,
  // so they are correctly occluded by terrain yet composite over open sky), then
  // the 2-D overlay layer (captions, axis/grid labels).
  vtkSmartPointer<vtkTranslucentPass> translucent = vtkSmartPointer<vtkTranslucentPass>::New();
  vtkSmartPointer<vtkVolumetricPass> volumetric = vtkSmartPointer<vtkVolumetricPass>::New();
  vtkSmartPointer<vtkOverlayPass> overlay = vtkSmartPointer<vtkOverlayPass>::New();

  vtkSmartPointer<vtkRenderPassCollection> passes = vtkSmartPointer<vtkRenderPassCollection>::New();
  passes->AddItem(baker);
  passes->AddItem(shadows);
  passes->AddItem(translucent);
  passes->AddItem(volumetric);
  passes->AddItem(overlay);
  vtkSmartPointer<vtkSequencePass> seq = vtkSmartPointer<vtkSequencePass>::New();
  seq->SetPasses(passes);
  vtkSmartPointer<vtkCameraPass> cam = vtkSmartPointer<vtkCameraPass>::New();
  cam->SetDelegatePass(seq);

  m_renderer->SetPass(cam);
  m_shadowPass = cam;
  m_shadowBaker = baker;
}

void SceneGraph::removeShadowPasses() {
  auto *baker = StridedShadowBaker::SafeDownCast(m_shadowBaker);
  if (m_renderer && m_shadowPass) {
    if (baker && baker->HoldsGLObjects())
      releaseShadowGL(m_renderer, m_shadowPass);
    if (m_renderer->GetPass() == m_shadowPass)
      m_renderer->SetPass(nullptr);
  }
  m_shadowPass = nullptr;
  m_shadowBaker = nullptr;
}

int SceneGraph::selectLOD(const cvc::lod::view_params &view, lod_stats *stats) {
  if (stats)
    stats->reset();
  int changes = 0;
  for (const auto &node : getAllGraphicsOfType<LodGraphicsNode>()) {
    if (!node || node->rungCount() == 0)
      continue;
    // Compare ACTIVE rungs: a node's first selection landing on rung 0, which it
    // was already drawing, is not a switch.
    const int before = node->activeRung();
    const int rung = m_lodEnabled ? node->select(view) : node->setRung(0);
    if (rung != before)
      ++changes;
    if (!stats)
      continue;
    ++stats->nodes;
    if (static_cast<std::size_t>(rung) >= stats->rung_nodes.size())
      stats->rung_nodes.resize(static_cast<std::size_t>(rung) + 1, 0);
    ++stats->rung_nodes[static_cast<std::size_t>(rung)];
    if (!node->isVisibleInHierarchy()) {
      ++stats->hidden;
      continue;
    }
    stats->drawn_tris += node->rungTriangles(rung);
    stats->full_tris += node->rungTriangles(0);
  }
  if (stats)
    stats->changes = changes;
  if (changes > 0)
    requestRender();
  return changes;
}

void SceneGraph::setLODEnabled(bool enabled) {
  if (m_lodEnabled == enabled)
    return;
  m_lodEnabled = enabled;
  requestRender();
}

} // namespace gl
} // namespace cvc

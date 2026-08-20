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
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <vtkCameraPass.h>
#include <vtkGPUVolumeRayCastMapper.h>
#include <vtkLight.h>
#include <vtkMultiVolume.h>
#include <vtkObjectFactory.h>
#include <vtkOverlayPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkTranslucentPass.h>
#include <vtkVolumetricPass.h>

SceneGraph::SceneGraph(cvc::app &ctx, const std::string &statePrefix)
    : m_renderer(nullptr), m_ctx(ctx), m_statePrefix(statePrefix),
      m_ownerThread(std::this_thread::get_id()), m_gridNode(nullptr), m_axisNode(nullptr),
      m_graphicsRoot(nullptr), m_nullGraphic(nullptr), m_multiVolumeRenderingEnabled(false),
      m_renderNeeded(false) {
  // This scene's own state publisher, running under the injected app — node poses
  // publish through it (SceneGraph::publisher()), coalesced off the render path.
  // Started eagerly, like the scene's pump, and drained in the destructor.
  // Single-threaded wasm cannot start the worker thread: writers still enqueue
  // (bounded, coalesced) and the embedder drains via publisher().flush().
  m_publisher = std::make_unique<cvc::gl::state_publisher>(m_ctx);
#ifndef __EMSCRIPTEN__
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

  // Clear SceneGraph reference from all nodes
  for (auto &node : m_rootNodes) {
    node->setSceneGraph(nullptr);
  }
}

void SceneGraph::postEvent(std::function<void()> callback) {
  std::lock_guard<std::mutex> lock(m_eventQueueMutex);
  m_eventQueue.push(std::move(callback));
  m_renderNeeded = true;
}

void SceneGraph::processEvents() {
  // Process all pending events on the main thread
  // Extract all events while holding the lock, then execute without lock
  std::queue<std::function<void()>> events;
  {
    std::lock_guard<std::mutex> lock(m_eventQueueMutex);
    std::swap(events, m_eventQueue);
  }

  // Execute all events on the main thread
  while (!events.empty()) {
    auto &callback = events.front();
    if (callback) {
      callback();
    }
    events.pop();
  }
}

void SceneGraph::requestRender() {
  std::lock_guard<std::mutex> lock(m_eventQueueMutex);
  m_renderNeeded = true;
}

bool SceneGraph::checkAndResetRenderNeeded() {
  std::lock_guard<std::mutex> lock(m_eventQueueMutex);
  bool needed = m_renderNeeded;
  m_renderNeeded = false;
  return needed;
}

void SceneGraph::setRenderer(vtkRenderer *renderer) {
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
  }
}

void SceneGraph::update() {
  for (auto &node : m_rootNodes) {
    node->update();
  }
}

void SceneGraph::setGridVisible(bool visible) { m_gridNode->setVisible(visible); }

void SceneGraph::setAxisVisible(bool visible) { m_axisNode->setVisible(visible); }

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
  cvc::bounding_box combinedBounds;
  bool first = true;

  // Process each direct child of the graphics root
  // Each child's getCombinedBoundingBox() already includes its descendants recursively
  if (m_graphicsRoot) {
    for (const auto &child : m_graphicsRoot->getGraphicsChildren()) {
      if (!child)
        continue;

      // Skip grid and axis nodes - they don't contribute to scene bounds
      if (child.get() == m_gridNode.get() || child.get() == m_axisNode.get()) {
        continue;
      }

      // Get combined bbox of this child (includes all its descendants in local space)
      cvc::bounding_box childBBox = child->getCombinedBoundingBox();

      // Skip invalid bounding boxes
      if (childBBox[0] > childBBox[3] || childBBox[1] > childBBox[4] ||
          childBBox[2] > childBBox[5]) {
        continue;
      }

      // Apply world transform to the bounding box by transforming all 8 corners
      vtkSmartPointer<vtkMatrix4x4> worldTransform = child->getWorldTransform();

      double corners[8][3] = {
          {childBBox[0], childBBox[1], childBBox[2]}, // min, min, min
          {childBBox[3], childBBox[1], childBBox[2]}, // max, min, min
          {childBBox[0], childBBox[4], childBBox[2]}, // min, max, min
          {childBBox[3], childBBox[4], childBBox[2]}, // max, max, min
          {childBBox[0], childBBox[1], childBBox[5]}, // min, min, max
          {childBBox[3], childBBox[1], childBBox[5]}, // max, min, max
          {childBBox[0], childBBox[4], childBBox[5]}, // min, max, max
          {childBBox[3], childBBox[4], childBBox[5]}  // max, max, max
      };

      // Transform all corners and find new axis-aligned bounds
      double minx = std::numeric_limits<double>::max();
      double miny = std::numeric_limits<double>::max();
      double minz = std::numeric_limits<double>::max();
      double maxx = std::numeric_limits<double>::lowest();
      double maxy = std::numeric_limits<double>::lowest();
      double maxz = std::numeric_limits<double>::lowest();

      for (int i = 0; i < 8; ++i) {
        double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
        double out[4];
        worldTransform->MultiplyPoint(in, out);

        minx = std::min(minx, out[0]);
        miny = std::min(miny, out[1]);
        minz = std::min(minz, out[2]);
        maxx = std::max(maxx, out[0]);
        maxy = std::max(maxy, out[1]);
        maxz = std::max(maxz, out[2]);
      }

      // Merge with combined bounds
      if (first) {
        combinedBounds = cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz);
        first = false;
      } else {
        combinedBounds[0] = std::min(combinedBounds[0], minx);
        combinedBounds[1] = std::min(combinedBounds[1], miny);
        combinedBounds[2] = std::min(combinedBounds[2], minz);
        combinedBounds[3] = std::max(combinedBounds[3], maxx);
        combinedBounds[4] = std::max(combinedBounds[4], maxy);
        combinedBounds[5] = std::max(combinedBounds[5], maxz);
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

  // Add to lookup map
  m_graphicsNodes[name] = graphicsNode;
  trackNodeBounds(graphicsNode);

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

  // Add to lookup map
  m_graphicsNodes[name] = graphicsNode;
  trackNodeBounds(graphicsNode);

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
  }
}

void SceneGraph::trackNodeBounds(const std::shared_ptr<GraphicsNode> &node) {
  if (!node)
    return;
  // When the node moves, marshal the world-bounds recompute onto the owner thread
  // (updateGrid touches the VTK grid/axis actors). The connection is owned here
  // and dies with the SceneGraph, so the captured `this` is safe.
  m_boundsConns.push_back(node->transformChanged.connect(
      [this](GraphicsNode *) { postEvent([this]() { onGraphicsBoundsChanged(); }); }));
}

void SceneGraph::onGraphicsBoundsChanged() {
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
  m_renderNeeded = true;
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

  // Add to lookup map
  m_graphicsNodes[name] = volumeNode;
  trackNodeBounds(volumeNode);

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

void SceneGraph::applyLights() {
  if (!m_renderer)
    return;
  m_renderer->RemoveAllLights();
  for (const auto &l : m_lights) {
    vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
    light->SetLightTypeToSceneLight();
    light->SetPositional(false); // directional: only the direction matters
    const double ce = std::cos(l.el * kDeg), se = std::sin(l.el * kDeg);
    light->SetPosition(kSunDistance * ce * std::sin(l.az * kDeg),
                       -kSunDistance * ce * std::cos(l.az * kDeg), kSunDistance * se);
    light->SetFocalPoint(0.0, 0.0, 0.0);
    light->SetColor(l.r, l.g, l.b);
    light->SetIntensity(l.intensity);
    m_renderer->AddLight(light);
  }
  // With no lights of our own, hand the renderer back its default headlight
  // rather than leaving the scene unlit.
  if (m_lights.empty())
    m_renderer->CreateLight();
  requestRender();
}

int SceneGraph::addDirectionalLight(double azimuthDeg, double elevationDeg, double r, double g,
                                    double b, double intensity) {
  LightDesc d{m_nextLightId++, azimuthDeg, elevationDeg, r, g, b, intensity};
  m_lights.push_back(d);
  applyLights();
  return d.id;
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

  void Render(const vtkRenderState *s) override {
    if (Interval <= 1 || (m_counter % static_cast<unsigned long>(Interval)) == 0) {
      this->Superclass::Render(s); // real bake
    } else {
      this->SetUpToDate(); // reuse the last-baked maps this frame
    }
    ++m_counter;
  }

protected:
  StridedShadowBaker() = default;

private:
  unsigned long m_counter = 0;
  StridedShadowBaker(const StridedShadowBaker &) = delete;
  void operator=(const StridedShadowBaker &) = delete;
};
vtkStandardNewMacro(StridedShadowBaker);

void SceneGraph::setShadowUpdateInterval(int frames) {
  m_shadowInterval = frames < 1 ? 1 : frames;
  if (auto *b = StridedShadowBaker::SafeDownCast(m_shadowBaker))
    b->Interval = m_shadowInterval;
  requestRender();
}

void SceneGraph::setShadowResolution(int pixels) {
  m_shadowResolution = pixels < 64 ? 64 : pixels;
  if (m_shadowBaker)
    m_shadowBaker->SetResolution(m_shadowResolution);
  requestRender();
}

bool SceneGraph::setShadowsEnabled(bool enabled) {
  m_shadowsEnabled = false;
  if (!m_renderer)
    return false;
  if (!enabled) {
    m_renderer->SetPass(nullptr);
    m_shadowBaker = nullptr;
    requestRender();
    return true;
  }
  // VTK's shadow maps are render PASSES, not a renderer flag: the baker renders
  // the scene once per light into a depth map, and the shadow pass consumes
  // those while drawing. They have to sit inside a camera pass, or the light's
  // view never gets set up.
  vtkSmartPointer<StridedShadowBaker> baker = vtkSmartPointer<StridedShadowBaker>::New();
  baker->Interval = m_shadowInterval;
  baker->SetResolution(m_shadowResolution); // crisper than VTK's low 256 default
  m_shadowBaker = baker;                    // kept so the interval/resolution stay live
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
  m_shadowsEnabled = true;
  requestRender();
  return true;
}

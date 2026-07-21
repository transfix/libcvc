#ifndef SCENEGRAPH_H
#define SCENEGRAPH_H

#include <boost/signals2.hpp>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/bounding_box.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkMultiVolume;
class SceneNode;
class NullGraphicNode;
class GridNode;
class AxisNode;
class BBoxNode;

namespace cvc {
class geometry;
class volume;
class state;
} // namespace cvc

class SceneGraph {
public:
  SceneGraph(const std::string &statePrefix = "cvcgl");
  ~SceneGraph();

  // Get the state prefix for this scene graph
  std::string getStatePrefix() const { return m_statePrefix; }

  void setRenderer(vtkRenderer *renderer);
  void update();

  // Process pending events on the main thread
  // This MUST be called regularly from the main event loop
  void processEvents();

  // Post a callback to be executed on the main thread during processEvents()
  // This is thread-safe and can be called from any thread
  void postEvent(std::function<void()> callback);

  // True when called from this SceneGraph's owner thread — the thread that
  // constructed it and drives processEvents() (main/GUI thread, or the sole
  // thread in a headless/scripted context). Node work initiated on the owner
  // thread runs inline; work from any other thread is marshalled through the
  // event queue. See SceneNode::runOnMainThread().
  bool onOwnerThread() const { return std::this_thread::get_id() == m_ownerThread; }

  // Rebind the owner thread to the caller. Use only if a different thread will
  // henceforth own the scene and drive processEvents().
  void adoptOwnerThread() { m_ownerThread = std::this_thread::get_id(); }

  // Check if a render is needed and reset the flag
  bool checkAndResetRenderNeeded();

  // Multi-object graphics management (unified for both geometry and volumes)
  std::shared_ptr<GraphicsNode> addGraphics(const std::string &name, const cvc::geometry &geom);
  std::shared_ptr<VolumeNode> addGraphics(const std::string &name, const cvc::volume &vol);
  std::shared_ptr<GraphicsNode>
  addGraphics(const std::string &name); // Empty graphics node for hierarchy
  bool hasGraphics(const std::string &name) const;
  void removeGraphics(const std::string &name);
  std::shared_ptr<GraphicsNode> getGraphics(const std::string &name);
  std::shared_ptr<GraphicsNode> getGraphicsRoot() { return m_graphicsRoot; }
  std::shared_ptr<GridNode> getGridNode() { return m_gridNode; }
  const std::map<std::string, std::shared_ptr<GraphicsNode>> &getAllGraphics() const {
    return m_graphicsNodes;
  }
  void registerGraphics(const std::string &name,
                        std::shared_ptr<GraphicsNode> node); // For manual registration

  // Generic templated method to recursively get all graphics nodes of a specific type
  template <typename T> std::vector<std::shared_ptr<T>> getAllGraphicsOfType() const {
    std::vector<std::shared_ptr<T>> result;

    // Helper lambda for recursive traversal
    std::function<void(std::shared_ptr<GraphicsNode>)> collectNodes;
    collectNodes = [&](std::shared_ptr<GraphicsNode> node) {
      if (!node)
        return;

      // Check if this node is of type T
      auto typedNode = std::dynamic_pointer_cast<T>(node);
      if (typedNode) {
        result.push_back(typedNode);
      }

      // Recursively check all children
      for (const auto &child : node->getGraphicsChildren()) {
        collectNodes(child);
      }
    };

    // Start traversal from graphics root
    if (m_graphicsRoot) {
      collectNodes(m_graphicsRoot);
    }

    return result;
  }

  // Convenience wrappers for common types
  std::vector<std::shared_ptr<VolumeNode>> getAllVolumeGraphics() const {
    return getAllGraphicsOfType<VolumeNode>();
  }
  size_t getVolumeGraphicsCount() const { return getAllVolumeGraphics().size(); }
  std::vector<std::shared_ptr<GeometryNode>> getAllGeometryGraphics() const {
    return getAllGraphicsOfType<GeometryNode>();
  }
  size_t getGeometryGraphicsCount() const { return getAllGeometryGraphics().size(); }

  // Multi-volume rendering control
  void enableMultiVolumeRendering(bool enable);
  bool isMultiVolumeRenderingEnabled() const;

  // Scene element visibility
  void setGridVisible(bool visible);
  void setAxisVisible(bool visible);

  // Scene element colors
  void setGridColor(double r, double g, double b);

  // Grid plane visibility
  void setGridPlaneVisibility(bool yz, bool xz, bool xy);

  // Grid divisions
  void setGridDivisions(int x, int y, int z);

  // Grid tick intervals
  void setGridTickIntervals(int x, int y, int z);

  // Per-plane grid colors
  void setGridPlaneColors(double yzR, double yzG, double yzB, double xzR, double xzG, double xzB,
                          double xyR, double xyG, double xyB);

  // Grid tick label properties
  void setGridTickLabelProperties(double r, double g, double b, int fontSize);

  // Update grid to match bounds
  void updateGrid(const cvc::bounding_box &bounds);

  // Compute combined bounding box of all graphics
  cvc::bounding_box computeGraphicsBounds() const;

  // Compute combined bounding box of all volumes
  cvc::bounding_box computeVolumeBounds() const;

  // Transfer function update
  void updateTransferFunction(const std::vector<double> &colorTable,
                              const std::vector<double> &opacityTable);

  // Signal emitted when graphics are added or removed
  boost::signals2::signal<void()> graphicsChanged;

private:
  vtkRenderer *m_renderer;
  std::string m_statePrefix;
  std::thread::id m_ownerThread; // thread that owns the scene / drives the pump

  std::shared_ptr<GridNode> m_gridNode;
  std::shared_ptr<AxisNode> m_axisNode;

  // Event queue for thread-safe main thread execution
  std::queue<std::function<void()>> m_eventQueue;
  std::mutex m_eventQueueMutex;
  bool m_renderNeeded;

  std::vector<std::shared_ptr<SceneNode>> m_rootNodes;

  // Multi-object graphics system (includes both geometry and volume graphics)
  std::shared_ptr<GraphicsNode> m_graphicsRoot; // Root node for all graphics
  std::map<std::string, std::shared_ptr<GraphicsNode>> m_graphicsNodes; // Flat lookup by name
  std::shared_ptr<NullGraphicNode> m_nullGraphic; // Placeholder when scene is empty

  // Multi-volume rendering state
  bool m_multiVolumeRenderingEnabled;
  vtkSmartPointer<vtkMultiVolume> m_multiVolume; // For multi-volume rendering when needed

  // Private helper methods for multi-volume rendering
  void setupMultiVolumeRendering();
  void teardownMultiVolumeRendering();
  void updateVolumeRendering();

  // Null graphic management
  void ensureNullGraphicIfEmpty();
  void removeNullGraphicIfPresent();
};

#endif // SCENEGRAPH_H

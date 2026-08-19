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
class vtkShadowMapBakerPass;
class SceneNode;
class NullGraphicNode;
class GridNode;
class AxisNode;
class BBoxNode;

namespace cvc {
class app;
class geometry;
class volume;
class state;
} // namespace cvc

class SceneGraph {
public:
  // Default: runs under cvcGL's own process-wide app (cvc::gl::context()).
  // Preserved verbatim so existing consumers (VolRover) are unaffected.
  SceneGraph(const std::string &statePrefix = "cvcgl");

  // Injected-app ctor: run this scene under a HOST-provided cvc::app so its
  // scene state lives in the host's state tree (cvc::state::instance(ctx)),
  // sharing one context with Python / the host. The default ctor delegates
  // here with cvc::gl::context(). `ctx` must outlive this SceneGraph.
  SceneGraph(cvc::app &ctx, const std::string &statePrefix);

  ~SceneGraph();

  // Get the state prefix for this scene graph
  std::string getStatePrefix() const { return m_statePrefix; }

  // The app whose state tree / thread pool this scene runs under. A viewer's
  // CameraController roots its state in this same tree so it shares the scene's
  // reactive state graph.
  cvc::app &appContext() const { return m_ctx; }

  void setRenderer(vtkRenderer *renderer);

  // ── lighting ──────────────────────────────────────────────────────────────
  // Lighting used to be entirely the host's business: nothing below the
  // renderer could touch it, so a scene had no way to say "this is an
  // afternoon" and a script had no way at all. Lights live here rather than on
  // the renderer so they SURVIVE setRenderer — the scene owns its lighting and
  // re-applies it to whatever renderer it is attached to, including a second
  // one.
  //
  // Directional only, described the way you actually think about a sun:
  // azimuth (compass bearing, 0 = +Y, growing towards +X) and elevation
  // (degrees above the horizon). Returns an id for later edits.
  int addDirectionalLight(double azimuthDeg, double elevationDeg, double r = 1.0, double g = 1.0,
                          double b = 1.0, double intensity = 1.0);
  void setLightDirection(int id, double azimuthDeg, double elevationDeg);
  void setLightColor(int id, double r, double g, double b);
  void setLightIntensity(int id, double intensity);
  void removeLight(int id);
  void clearLights();
  std::size_t numLights() const;

  // Shadow mapping, via VTK's shadow-map render passes. Returns false if there
  // is no render target yet, so a caller can fall back rather than silently
  // render unshadowed.
  //
  // Requires lit geometry, which now comes for free: GeometryNode generates
  // point normals for any triangle mesh that arrives without them. Before that,
  // the shadow snippet spliced into VTK's lit-surface template failed to compile
  // ("'vertexVC' : undeclared identifier") because a normal-less mesh takes the
  // UNLIT shader path, so the declaration it references never appeared. With
  // normals present the program builds and the passes draw.
  bool setShadowsEnabled(bool enabled);
  bool shadowsEnabled() const { return m_shadowsEnabled; }

  // How often the shadow map is re-baked, in frames. The shadow baker re-renders
  // the whole scene depth from every light whenever geometry moves, so at high
  // deformation rates (e.g. a forest swaying every frame) it dominates. But
  // shadows from slow motion barely change frame-to-frame, so baking every Nth
  // frame and reusing the map between is visually indistinguishable and much
  // cheaper — the shadow analogue of decoupling a simulation from the frame rate.
  // n=1 (default) bakes every frame; n>1 bakes every n-th frame and samples the
  // last-baked map in between. Takes effect immediately, no need to toggle shadows.
  void setShadowUpdateInterval(int frames);
  int shadowUpdateInterval() const { return m_shadowInterval; }
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

  // Mark the scene as needing a render, WITHOUT rendering. The host's frame
  // loop picks it up via checkAndResetRenderNeeded(). Nodes reacting to state
  // changes must use this rather than calling vtkRenderWindow::Render()
  // themselves: a synchronous render per state change turns N per-frame
  // property updates into N full scene renders per displayed frame (measured
  // in VolRover3: ~5 ms per update on a city-scale scene — 63 animated nodes
  // dragged the app from 60 FPS to under 3).
  void requestRender();

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

  // Lights are kept as descriptions, not just vtkLights, so they can be
  // re-created against a renderer attached later (or a second one).
  struct LightDesc {
    int id;
    double az, el, r, g, b, intensity;
  };
  std::vector<LightDesc> m_lights;
  int m_nextLightId = 1;
  bool m_shadowsEnabled = false;
  int m_shadowInterval = 1;                             // re-bake every N frames
  vtkSmartPointer<vtkShadowMapBakerPass> m_shadowBaker; // held so the interval is live
  void applyLights();
  cvc::app &m_ctx; // app whose state tree / thread pool this scene runs under
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

  // World-bounds tracking: the grid box GROWS to follow a node that moves out of
  // it. Each registered node's transformChanged signal is connected (connections
  // owned here, so they die with the scene) to marshal onGraphicsBoundsChanged()
  // onto the owner thread; grow-only so in-bounds animation never resizes the grid.
  cvc::bounding_box m_worldBounds;
  std::vector<boost::signals2::scoped_connection> m_boundsConns;
  void trackNodeBounds(const std::shared_ptr<GraphicsNode> &node);
  void onGraphicsBoundsChanged();

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

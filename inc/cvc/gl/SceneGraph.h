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
namespace gl {
class state_publisher;
class ShadowSettings;
class LightNode;
} // namespace gl
} // namespace cvc

class SceneGraph {
public:
  // Run this scene under a HOST-provided cvc::app so its scene state lives in the
  // host's state tree (cvc::state::instance(ctx)), sharing one context with
  // Python / the host. There is no default (singleton) ctor: every scene names
  // the app it runs under. `ctx` must outlive this SceneGraph.
  SceneGraph(cvc::app &ctx, const std::string &statePrefix = "cvcgl");

  ~SceneGraph();

  // Get the state prefix for this scene graph
  std::string getStatePrefix() const { return m_statePrefix; }

  // The app whose state tree / thread pool this scene runs under. A viewer's
  // CameraController roots its state in this same tree so it shares the scene's
  // reactive state graph.
  cvc::app &appContext() const { return m_ctx; }

  // This scene's state publisher — GraphicsNode poses publish through it so state
  // writes are coalesced off the render path (see state_publisher). Owned by the
  // scene and running under appContext(); never a process-wide singleton.
  cvc::gl::state_publisher &publisher() { return *m_publisher; }

  void setRenderer(vtkRenderer *renderer);

  // ── lighting ──────────────────────────────────────────────────────────────
  // Lighting used to be entirely the host's business: nothing below the
  // renderer could touch it, so a scene had no way to say "this is an
  // afternoon" and a script had no way at all. Lights live here rather than on
  // the renderer so they SURVIVE setRenderer — the scene owns its lighting and
  // re-applies it to whatever renderer it is attached to, including a second
  // one.
  //
  // DIRECTIONAL — a sun, described the way you actually think about one:
  // azimuth (compass bearing, 0 = +Y, growing towards +X) and elevation
  // (degrees above the horizon). Returns an id for later edits.
  //
  // SHADOW COST, and why you may want a spot instead: VTK bakes a directional
  // light's shadow map with a PARALLEL projection fitted to the WHOLE scene
  // bounding box (vtkShadowMapBakerPass::BuildCameraLight — parallelScale =
  // max(bboxWidth, bboxHeight)/2). One map is therefore stretched across
  // everything in the scene, so a big scene with small detail gets a handful of
  // shadow texels per object and the shadows read as mush. A wide sky dome or a
  // distant billboard silently makes every shadow in the scene worse.
  int addDirectionalLight(double azimuthDeg, double elevationDeg, double r = 1.0, double g = 1.0,
                          double b = 1.0, double intensity = 1.0);
  void setLightDirection(int id, double azimuthDeg, double elevationDeg);

  // SPOT — a positional light at (x, y, z) aimed at (tx, ty, tz), limited to a
  // cone. This is the one that gives crisp shadows: VTK bakes a spot's shadow
  // map with a PERSPECTIVE projection whose view angle is the cone, so the map's
  // texels land only inside the lit cone instead of being spread over the scene.
  // Narrow the cone at a subject and the shadow sharpens accordingly.
  //
  // coneDeg is the half-angle and MUST stay below 90: VTK's
  // LightCreatesShadow() drops any positional light with cone >= 90 from the
  // bake entirely, so a "cone" of 90+ silently means no shadow at all. Values
  // are clamped into (0, 89.5] for you.
  int addSpotLight(double x, double y, double z, double tx, double ty, double tz,
                   double coneDeg = 30.0, double r = 1.0, double g = 1.0, double b = 1.0,
                   double intensity = 1.0);
  // FILL — a positional light that deliberately casts NO shadow, for lighting
  // the environment beyond the rig's cones (open water, sky, distant scenery).
  //
  // It works by turning VTK's shadow rule into a feature: LightCreatesShadow()
  // drops any positional light whose cone reaches 90 degrees, so a wide fill is
  // free in the shadow bake. This is the one place a cone >= 90 is correct, and
  // why addSpotLight() clamps below 90 while this does not.
  //
  // Needed because a rig of tight cones leaves everything outside them
  // completely unlit — which is invisible on matte ground but obvious on water,
  // where the specular highlight simply vanishes at grazing view angles because
  // the water it would land on receives no light at all.
  int addFillLight(double x, double y, double z, double tx, double ty, double tz, double r = 1.0,
                   double g = 1.0, double b = 1.0, double intensity = 0.25);

  void setLightPosition(int id, double x, double y, double z);
  void setLightTarget(int id, double tx, double ty, double tz);
  void setLightCone(int id, double coneDeg);

  // BATCHING. Every add/remove/set below rebuilds the renderer's entire light
  // set (VTK gives no way to mutate one light in place), and each rebuild makes
  // the shadow baker re-bake a full-resolution map per casting light. A rig that
  // replaces 14 lights therefore pays ~14x14 light rebuilds and a shadow storm —
  // which is a multi-second freeze, not a hitch.
  //
  // Wrap a batch of light edits in these and the rebuild happens ONCE at the
  // end. Nested begins are counted, so callers can compose safely.
  // A LightNode changed and the renderer's light set must be rebuilt. Called by
  // LightNode itself; honours the batch below, so moving a whole rig is one
  // rebuild.
  // Add a LIGHT as a scene-graph node. This is the way to make a light — it is
  // registered like any other node, so it appears in the hierarchy, can be
  // parented (parent it to a vehicle and the lamp rides along), carries its own
  // state, and is switched off by hiding it.
  std::shared_ptr<cvc::gl::LightNode> addLight(const std::string &name);

  void lightsChanged();

  void beginLightBatch();
  void endLightBatch();

  void setLightColor(int id, double r, double g, double b);
  void setLightIntensity(int id, double intensity);
  void removeLight(int id);
  void clearLights();
  std::size_t numLights() const;
  // True when the light exists and is a shadow-casting kind (see the cone note).
  bool lightCastsShadow(int id) const;

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
  // Shadow settings are ALSO cvc::state, at "<scene prefix>.shadows"
  // (enabled/resolution/interval), so they are scriptable like everything
  // else; these setters and the state stay in sync in both directions.
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

  // Shadow-map texture resolution (pixels per side). VTK defaults to a low 256,
  // which aliases thin casters (a forest of trunks/needles shows torn, "inverted"
  // shadows) and speckles broad surfaces with self-shadow acne. Larger is crisper
  // but costs VRAM/fill. Takes effect immediately.
  void setShadowResolution(int pixels);
  int shadowResolution() const { return m_shadowResolution; }
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

  // ---- diagnostic chrome ---------------------------------------------------
  // The grid, the origin axis marker, and the per-node bounding boxes with their
  // extent labels. Invaluable while you are asking "why does this look wrong?",
  // and wrong in a captured frame or a shipped view.
  //
  // The bbox/label setters are SCENE-WIDE. GraphicsNode::setShowBBox() and
  // setShowExtentLabels() touch only the node they are called on, which is a
  // sharp edge: six call sites across the demos did
  // `getGraphicsRoot()->setShowBBox(false)` and silently left every child's box
  // AND every extent label switched on. That is also the world-bounds box the
  // constructor turns on (m_nullGraphic IS the graphics root), which is why
  // setGridVisible(false) alone never suppressed it.
  void setGridVisible(bool visible);
  bool gridVisible() const;
  void setAxisVisible(bool visible);
  bool axisVisible() const;
  // Applied to every graphics node, root included.
  void setBBoxesVisible(bool visible);
  void setExtentLabelsVisible(bool visible);
  // All four at once — what a "presentation mode" or an offscreen capture wants.
  void setDiagnosticChromeVisible(bool visible);

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
  cvc::bounding_box computeGraphicsBounds() const; // public: lights are excluded

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
    enum class Kind { Directional, Spot };
    int id;
    Kind kind = Kind::Directional;
    double az = 0, el = 45;        // Directional: where in the sky it hangs
    double px = 0, py = 0, pz = 0; // Spot: position
    double tx = 0, ty = 0, tz = 0; // Spot: what it is aimed at
    double cone = 30.0;            // Spot: half-angle, < 90 or VTK drops the shadow
    double r = 1, g = 1, b = 1, intensity = 1;
  };
  void syncShadowState(); // mirror the shadow setters into cvc::state
  std::unique_ptr<cvc::gl::ShadowSettings> m_shadowSettings;
  bool m_applyingShadowState = false; // re-entry guard: state -> setter -> state
  int m_lightBatchDepth = 0;          // >0 defers applyLights()
  bool m_lightsDirty = false;         // an edit happened while batching
  std::vector<LightDesc> m_lights;
  int m_nextLightId = 1;
  bool m_shadowsEnabled = false;
  int m_shadowInterval = 1;                             // re-bake every N frames
  int m_shadowResolution = 1024;                        // shadow-map pixels per side
  vtkSmartPointer<vtkShadowMapBakerPass> m_shadowBaker; // held so the interval is live
  void applyLights();
  cvc::app &m_ctx; // app whose state tree / thread pool this scene runs under
  std::unique_ptr<cvc::gl::state_publisher> m_publisher; // scene-owned, runs under m_ctx
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
  // Authoritative world-bounds recompute, for when the SET of graphics changes
  // (add/remove) rather than one of them moving. onGraphicsBoundsChanged() is
  // deliberately grow-only so in-bounds animation cannot jitter the grid; that
  // is wrong here — an added node must be enclosed even from a degenerate start,
  // and a removed one should let the grid shrink back.
  void refreshWorldBounds();

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

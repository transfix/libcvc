// VolRenNode — drop-in scene node for the cvc::volren software raycaster.
//
// Renders one or more cvc::volume objects through cvc::volren::raycaster and
// composites the result into the VTK scene as a textured quad:
//
//  - COLOR: the raycast frame (straight-alpha RGBA8; the raycaster's internal
//    background is forced to black so alpha carries all compositing) uploaded
//    through GeometryNode's zero-copy texture path; the quad renders in the
//    TRANSLUCENT pass so the scene shows through where the volume is thin.
//  - DEPTH: the frame's eye-space depth map goes in as an R32F texture and a
//    //VTK::Depth::Impl fragment replacement converts it to window z with the
//    live camera's near/far (pushed as uniforms every tick), so volume pixels
//    depth-test PER PIXEL against the opaque scene — geometry intersecting
//    the volume occludes it correctly.
//  - TRANSFORM: the node's composed scene-graph world matrix (parent chain,
//    the standard `matrix`/position/rotation/scale keys) is multiplied into
//    each volume's volume_settings::model_transform before every raycast, so
//    the volume follows the scene graph like any other node.
//
// The raycast itself runs on a worker thread (one frame in flight, latest
// camera wins) and re-renders when the camera, node transform, or settings
// change — or every frame with setContinuous(true).  On single-threaded wasm
// builds the raycast runs synchronously inside tick().
//
// Settings are state-tree-bound: the node owns a cvc::volren::state_settings
// at "<node state path>.volren" (renderer + per-volume settings; its
// camera/raster fields are ignored — the LIVE viewer camera drives those),
// plus "volren.resolution_scale" and "volren.continuous" keys of its own.
//
// Usage (once per frame, before SceneRenderer::render()):
//     auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolRenNode>("vol");
//     node->addVolume(sdf_volume, settings);
//     ... in the render loop: node->tick();
#ifndef CVC_GL_VOLRENNODE_H
#define CVC_GL_VOLRENNODE_H

#include <cvc/gl/GeometryNode.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/state_settings.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class vtkTextureObject;

namespace cvc {
namespace gl {

class VolRenNode : public GeometryNode {
public:
  VolRenNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolRenNode() override;

  // Volumes are shallow copies (buffers pinned during each raycast).
  // Settings changes go through the state tree (the source of truth).
  std::size_t addVolume(const cvc::volume &vol,
                        cvc::volren::volume_settings vs = cvc::volren::volume_settings());
  void clearVolumes();
  std::size_t volumeCount() const;
  cvc::volren::volume_settings volumeConfig(std::size_t index) const;
  void setVolumeConfig(std::size_t index, const cvc::volren::volume_settings &vs);
  cvc::volren::render_settings renderConfig() const;
  void setRenderConfig(const cvc::volren::render_settings &rs);

  // Raycast raster = viewport * scale, clamped to [0.05, 1.0]; the quad
  // upscales.  The main performance knob.  State key: volren.resolution_scale.
  void setResolutionScale(double scale);
  double resolutionScale() const;

  // Re-raycast every tick instead of only when camera/transform/settings
  // change.  State key: volren.continuous.
  void setContinuous(bool on);
  bool continuous() const;

  // Which raycaster backend to ask for.  Defaults to the raycaster's own
  // default (CPU); backend::automatic uses CUDA when the device and the scene
  // both allow it and silently falls back otherwise.
  void setBackend(cvc::volren::backend b);
  cvc::volren::backend backendUsed() const;

  // Call once per frame BEFORE the viewer's render(): snapshots the live
  // camera + composed world transform, refreshes the depth-conversion
  // uniforms, and (a)synchronously raycasts when needed.  Returns true when a
  // freshly raycast frame was applied to the quad since the previous tick.
  bool tick();

  // Perf/testing introspection.
  double lastRenderSeconds() const { return m_lastRenderSeconds.load(); }
  std::uint64_t framesRendered() const { return m_framesRendered.load(); }

  // Union of the registered volumes' boxes under their model transforms
  // (this node's own scene transform is applied by the scene graph).
  cvc::bounding_box getBoundingBox() const override;

protected:
  // The quad is glued to the raycast camera pose, not the scene transform;
  // the scene transform instead feeds the raycaster's model matrices.
  void applyTransformToVTK() override;
  void handleStateChanged(const std::string &childState) override;

private:
  struct snapshot; // camera + composed matrix + settings for one raycast
  struct worker;

  void seedOwnState();
  bool buildSnapshot(snapshot &out); // owner thread; false when not renderable yet
  void applyFrame(const cvc::volren::frame &f, const snapshot &snap); // owner thread
  void pushDepthUniforms();          // owner thread, every tick
  void ensureQuad();
  void launchOrRun(const snapshot &snap);

  mutable std::mutex m_configMutex; // guards m_snapshotSettings + m_volumes
  cvc::volren::state_settings::snapshot m_snapshotSettings; // settings source of truth
  std::vector<cvc::volume> m_volumes;
  std::unique_ptr<cvc::volren::state_settings> m_stateSettings;
  std::uint64_t m_settingsVersion = 0; // bumped on every settings/volume change

  std::unique_ptr<worker> m_worker;   // owns the raycaster + render thread
  std::atomic<double> m_lastRenderSeconds{0.0};
  std::atomic<std::uint64_t> m_framesRendered{0};

  double m_resolutionScale = 0.5;
  bool m_continuous = false;
  std::atomic<int> m_backendUsed{0}; // cvc::volren::backend ordinal of the last frame

  // Last-applied raycast identity (owner thread only).
  std::uint64_t m_appliedVersion = ~0ull;
  std::array<double, 16> m_appliedMatrix{};
  std::array<double, 11> m_appliedCamera{}; // eye3 focal3 up3 fov scale
  int m_appliedW = 0, m_appliedH = 0;

  // GL-side resources (owner thread only).
  cvc::image m_colorImage; // persistent aliased texture buffer
  vtkSmartPointer<vtkTextureObject> m_depthTexture;
  bool m_quadReady = false;
  bool m_frameAppliedSinceTick = false;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLRENNODE_H

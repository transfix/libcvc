// VolRenNode — drop-in scene node for the cvc::volren software raycaster.
//
// Renders one or more cvc::volume objects through cvc::volren::raycaster and
// composites the result into the VTK scene as a textured quad:
//
//  - COLOR: the raycast frame (RGBA8, PREMULTIPLIED -- the raycaster's internal
//    background is forced to black, so alpha carries all compositing) uploaded
//    verbatim through GeometryNode's zero-copy texture path and un-premultiplied
//    in the fragment shader, after the texture filter, because bilinear
//    interpolation is only linear in premultiplied space (filtering straight
//    alpha darkens every silhouette).  The quad renders in the TRANSLUCENT pass
//    so the scene shows through where the volume is thin.
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
//
// ---------------------------------------------------------------------------
// WHY THIS DERIVES FROM GeometryNode AND NOT FROM VolumeNode
// ---------------------------------------------------------------------------
// The name says "volume", so VolumeNode looks like the obvious base.  It is
// not, because VolumeNode is not a general volume abstraction: it is a thin
// wrapper around vtkSmartVolumeMapper, i.e. VTK's OWN GPU raycaster.  Its
// whole job is to hand a vtkImageData plus transfer functions to VTK and let
// VTK render them.
//
// By the time this node draws, the rendering is already done: cvc::volren has
// produced a finished 2D RGBA image and a depth map on the CPU or in CUDA.
// There is nothing left for a volume mapper to consume.  What the node needs
// instead is exactly the machinery GeometryNode already has, and VolumeNode
// has none of:
//
//   - setTexture(img, zeroCopy) + texture_modified(): per-frame color upload
//     that ALIASES the cvc::image buffer instead of copying it;
//   - addFragmentShaderReplacement(): how the gl_FragDepth write and the
//     un-premultiply are injected into VTK's shader;
//   - setShaderTexture(): binding the R32F depth map;
//   - updateVertices(): re-posing the quad across the camera frustum.
//
// Deriving from VolumeNode would mean inheriting a vtkVolume prop and a mapper
// that would then have to be actively suppressed, in exchange for nothing.
// The class models a volume, but its VTK-side representation is an image on a
// quad -- and a textured quad is a GeometryNode.
//
// KNOWN WART of that choice: public inheritance also exposes GeometryNode's
// mesh API -- setGeometry(), updateVertices(), updateColors(), setRenderMode()
// and friends -- which are meaningless here and will corrupt the quad if
// called.  They are NOT part of this node's contract; treat them as private.
// The clean alternatives both cost more than they are worth today: private
// inheritance breaks addGraphicsChild<T>(), which needs a public GraphicsNode
// for the shared_ptr conversion, and composition would mean duplicating the
// texture/shader plumbing listed above.
//
// What IS deliberately shared with VolumeNode is the settings encoding: the
// transfer function round-trips through the same "transfer_function.color" /
// ".opacity" flat-CSV state keys, so one editor drives either renderer.
#ifndef CVC_GL_VOLRENNODE_H
#define CVC_GL_VOLRENNODE_H

#include <atomic>
#include <condition_variable>
#include <cvc/gl/GeometryNode.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/state_settings.h>
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

  // Raycast raster = viewport * scale, clamped to
  // [MinResolutionScale, MaxResolutionScale]; the quad rescales to fill the
  // viewport either way.  The main performance knob.  State key:
  // volren.resolution_scale.
  //
  // Above 1.0 it is an anti-aliasing knob too: the quad's texture filter is
  // bilinear, so a 2x raster lands exactly four texels under each screen pixel
  // and the two bilinear taps average all four (a screen pixel center falls on
  // the midpoint between texel centers, weights 0.5/0.5 in both axes) -- an
  // exact 2x2 box downsample.  That is why MaxResolutionScale is 2.0 and not
  // more: at 3x, bilinear still reads 4 of the 9 texels under the pixel, so a
  // third of the rays paid for would be discarded.  Past 2x, supersample is
  // the knob that averages every ray it casts.
  //
  // Distinct from render_settings::supersample, the EDGE-QUALITY knob, which
  // this node carries through the state tree like every other renderer
  // setting: scale trades output resolution for latency (below 1.0 everything,
  // edges included, gets blurrier), supersampling spends n^2 rays per pixel to
  // anti-alias silhouettes at whatever raster the scale picked.  Both are
  // priced in rays and they compose -- see docs/VOLREN_API.md.
  static constexpr double MinResolutionScale = 0.05;
  static constexpr double MaxResolutionScale = 2.0;
  void setResolutionScale(double scale);
  double resolutionScale() const;

  // Thin accessors for the renderer settings a UI reaches for; each one is a
  // read-modify-write of renderConfig() and therefore goes through the
  // embedded state_settings (the source of truth) exactly like a full
  // setRenderConfig() would.
  //
  // Anti-aliasing: n x n rays per pixel, n^2 x the cost, output size
  // unchanged.  Must be in [1, cvc::volren::limits::max_supersample] --
  // render() rejects anything else rather than silently clamping, so a bad
  // value throws on the worker thread and the node keeps the previous frame.
  void setSupersample(int n);
  int supersample() const;

  // Volumetric shadows.  Off by default; enabling costs one extra light-view
  // raycast per casting light, CACHED across camera motion (the fingerprint
  // excludes the camera), so orbiting is free and a scene change pays one
  // rebuild frame.  The volume shadows itself and the other registered
  // volumes; it does not yet cast onto cvcGL scene geometry (docs/VOLREN_API.md
  // spells out the two VTK-side blockers).
  void setShadowsEnabled(bool on);
  bool shadowsEnabled() const;
  cvc::volren::shadow_settings shadowConfig() const;
  void setShadowConfig(const cvc::volren::shadow_settings &ss);

  // Announce that a registered volume's voxels changed UNDER THE SAME BUFFER
  // (an in-place write through the unchecked legacy voxels::data_ptr()).  The
  // node re-copies its volumes into the raycaster every raycast, so every
  // write through the supported cvc::volume API copy-on-writes and needs no
  // announcement; this is the escape hatch for the one case that does not.
  //
  // Deferred, not immediate: the raycaster lives on the worker thread, so this
  // only arms a flag that the worker consumes after re-registering the volumes
  // and before marching.  It drops the CUDA resident blocks AND (through the
  // content generation those blocks are stamped with) the cached shadow maps.
  void invalidateVolumeData();

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

  // True when the frame ON SCREEN was raycast from the current camera,
  // transform and settings, and no further raycast is pending.  Updated by
  // tick().  Poll this before capturing a frame: "no new frame this tick" is
  // ambiguous between settled and still-working, and under load the latter is
  // what you get.
  bool converged() const { return m_converged.load(std::memory_order_relaxed); }

  // Perf/testing introspection.
  double lastRenderSeconds() const { return m_lastRenderSeconds.load(); }
  std::uint64_t framesRendered() const { return m_framesRendered.load(); }
  // Raster of the last APPLIED frame, 0 before the first one.  This is the
  // authoritative ray count for a readout: viewport * resolutionScale()
  // rounded and floored the way buildSnapshot() does it, not re-derived.
  int raycastWidth() const { return m_appliedW; }  // owner thread
  int raycastHeight() const { return m_appliedH; } // owner thread

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
  void pushDepthUniforms();                                           // owner thread, every tick
  void ensureQuad();
  void launchOrRun(const snapshot &snap);

  mutable std::mutex m_configMutex;                         // guards m_snapshotSettings + m_volumes
  cvc::volren::state_settings::snapshot m_snapshotSettings; // settings source of truth
  std::vector<cvc::volume> m_volumes;
  std::unique_ptr<cvc::volren::state_settings> m_stateSettings;
  std::uint64_t m_settingsVersion = 0; // bumped on every settings/volume change

  std::unique_ptr<worker> m_worker; // owns the raycaster + render thread
  std::atomic<double> m_lastRenderSeconds{0.0};
  std::atomic<std::uint64_t> m_framesRendered{0};

  double m_resolutionScale = 0.5;
  bool m_continuous = false;
  std::atomic<int> m_backendUsed{0}; // cvc::volren::backend ordinal of the last frame
  std::atomic<bool> m_converged{false};

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

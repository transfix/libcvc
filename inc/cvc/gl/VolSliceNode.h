// VolSliceNode — drop-in scene node for the cvc::volslice view-aligned slice
// renderer, the port of VolumeRover2's classic slice-compositing volume view.
//
// The third volume-rendering path in cvcGL, beside VolumeNode (VTK's own GPU
// raycaster) and VolRenNode (the cvc::volren software raycaster).  One node
// renders ONE volume — like VolumeNode, and unlike VolRenNode's embedded
// multi-volume raycaster — and the scene graph composes several nodes.
//
// How it draws: every frame whose camera, node transform, or settings changed,
// tick() runs cvc::volslice::compute_slices() (the legacy plane sweep) with
// the live camera's local->clip matrix and rewrites this node's polydata with
// the back-to-front slice fans.  The volume is a normalized R8 3D texture, the
// transfer function a 256x1 RGBA LUT texture, and a fragment-shader
// replacement samples volume-then-LUT per pixel — exactly the legacy ARB
// fragment program's dependent lookup, in GLSL.  Slice texcoords are not
// uploaded at all: they are an affine map of local position, computed in the
// vertex shader from vertexMC (two uniforms), which sidesteps VTK's
// texture-conditional tcoord plumbing.
//
// ORDER-DEPENDENT BLENDING — the scene-wide effect to know about:
// slice compositing is sequential by construction (each slice multiplies what
// is behind it by 1-alpha), but VTK 9's default translucent pass is
// order-INDEPENDENT (vtkRenderer::UseOIT, weighted accumulation with
// rgb=(ONE,ONE)): under it the slice stack averages into an X-ray look
// instead of compositing (measured, not theory).  addToRenderer() therefore
// switches its vtkRenderer to sequential translucency (UseOITOff), which VTK
// renders with the legacy renderer's exact blend state
// (SRC_ALPHA/ONE_MINUS_SRC_ALPHA, depth-write off; verified by GL-state
// probe).  Other translucent actors in the scene then depth-sort as props,
// pre-VTK9 style.  The toggle is NOT restored on removeFromRenderer — other
// slice nodes may share the renderer.
//
// Settings are state-tree-bound: the node owns a cvc::volslice::state_settings
// at "<node state path>.volslice" (quality / max_planes / near_plane /
// interpolation / opacity_correction / window / transfer_function.*, the
// shared VolumeNode/volren TF encoding).  There are no node-private keys.
//
// WHY THIS DERIVES FROM GeometryNode AND NOT FROM VolumeNode: the same
// reasoning as VolRenNode (see VolRenNode.h) — VolumeNode is a thin wrapper
// around vtkSmartVolumeMapper, useless to a renderer that produces its own
// primitives, while GeometryNode owns exactly the machinery needed here:
// m_polyData for the per-frame slice fans, addFragment/VertexShaderReplacement
// for the sampling shader, setShaderTexture/setShaderUniform* for the 3D
// volume and LUT textures.  The same KNOWN WART applies: GeometryNode's mesh
// API (setGeometry, updateVertices, ...) is exposed but NOT part of this
// node's contract — calling it corrupts the slice geometry.
//
// Usage (once per frame, before SceneRenderer::render()):
//     auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("vol");
//     node->setVolume(density_volume);
//     ... in the render loop: node->tick();
#ifndef CVC_GL_VOLSLICENODE_H
#define CVC_GL_VOLSLICENODE_H

#include <array>
#include <cstdint>
#include <cvc/gl/GeometryNode.h>
#include <cvc/volslice/settings.h>
#include <cvc/volslice/state_settings.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <mutex>
#include <optional>

class vtkTextureObject;

namespace cvc {
namespace gl {

class VolSliceNode : public GeometryNode {
public:
  VolSliceNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolSliceNode() override;

  // The volume is a shallow copy (copy-on-write; the texture upload pins the
  // buffer for its duration).  Replaces any previous volume.
  void setVolume(const cvc::volume &vol);
  bool hasVolume() const;

  // Settings round-trip through the state tree (the source of truth).
  cvc::volslice::render_settings config() const;
  void setConfig(const cvc::volslice::render_settings &s);

  // Recompute slices / re-upload textures if anything changed since the last
  // call.  Call once per frame before rendering (the VolRenNode contract).
  // Returns true if the on-screen geometry changed.
  bool tick();

  // Slice planes drawn by the last tick() — the legacy
  // getNumberOfPlanesRendered(), the number the demos' HUDs show.
  std::size_t planesRendered() const { return m_planesRendered; }

  cvc::bounding_box getBoundingBox() const override;
  void addToRenderer(vtkRenderer *renderer) override;

private:
  void rebuildSliceGeometry(const cvc::volslice::mat4 &localToClip,
                            const cvc::volslice::render_settings &s);
  bool uploadVolumeTexture(const cvc::volslice::render_settings &s);
  bool uploadTransferFunction(const cvc::volslice::render_settings &s);
  // The normalization window actually in effect: explicit window if set,
  // else the volume's data range (see settings.h).
  std::pair<double, double> effectiveWindow(const cvc::volslice::render_settings &s) const;

  std::unique_ptr<cvc::volslice::state_settings> m_stateSettings;
  std::uint64_t m_settingsVersion = 0; // bumped by the state apply callback
  mutable std::mutex m_volumeMutex;
  std::optional<cvc::volume> m_volume;
  std::uint64_t m_volumeVersion = 0;

  vtkSmartPointer<vtkTextureObject> m_volumeTexture;
  vtkSmartPointer<vtkTextureObject> m_tfTexture;

  // Change detection for tick(): what the on-screen geometry was built from.
  std::array<double, 16> m_appliedMatrix{};
  std::uint64_t m_appliedSettings = std::uint64_t(-1);
  std::uint64_t m_appliedVolume = std::uint64_t(-1);
  std::uint64_t m_uploadedVolume = std::uint64_t(-1);
  std::uint64_t m_uploadedTf = std::uint64_t(-1);
  double m_uploadedWindowLo = 0.0, m_uploadedWindowHi = 0.0;
  int m_uploadedFilter = -1;

  std::size_t m_planesRendered = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLSLICENODE_H

// VolSliceNode -- the cvcGL node for cvc::volslice (see the header for the
// architecture and the OIT note).
#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/volslice/slicer.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropCollection.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextureObject.h>

namespace cvc {
namespace gl {

namespace {

// Vertex-stage declarations: the slice texcoord is an affine map of the
// LOCAL-space vertex, so it is computed here from vertexMC instead of being
// uploaded as a tcoord array (see the header).
const char *kVertexUniformsDec = "//VTK::CustomUniforms::Dec\n"
                                 "uniform vec3 volsliceTexScale;\n"
                                 "uniform vec3 volsliceTexOffset;\n"
                                 "out vec3 volsliceSTP;\n";

// The anchor is re-emitted first: GeometryNode registers replacements with
// replaceFirst=true, so dropping it would delete VTK's own position code --
// the wasm low-memory mapper populates anchors the desktop path leaves empty
// (see docs/VOLREN_API.md's shader-replacement note).
const char *kVertexPositionImpl = "//VTK::PositionVC::Impl\n"
                                  "  volsliceSTP = vertexMC.xyz * volsliceTexScale"
                                  " + volsliceTexOffset;\n";

// `highp` on the samplers is for GLES3/WebGL2, where sampler3D has NO default
// precision (unqualified fails to compile) and sampler2D defaults to lowp --
// whose 8-bit fraction cannot address the LUT's 1/512-texel centers.  Desktop
// GLSL accepts the qualifiers as no-ops, so the native build cannot catch
// either mistake (the VOLREN_API.md wasm-shader lesson again).
const char *kFragmentUniformsDec = "//VTK::CustomUniforms::Dec\n"
                                   "uniform highp sampler3D volsliceVolumeTex;\n"
                                   "uniform highp sampler2D volsliceTfTex;\n"
                                   "uniform float volsliceAlphaExp;\n"
                                   "in vec3 volsliceSTP;\n";

// The legacy ARB fragment program in GLSL: sample the normalized density,
// look it up in the 256-entry LUT (texel-center indexing so density 0 and 1
// hit entries 0 and 255 exactly), optionally correct the slice alpha for the
// actual inter-plane spacing, and OVERWRITE the fragment color -- the node
// draws unlit white underneath, so nothing of VTK's shading survives.
const char *kFragmentSampleImpl =
    "//VTK::TCoord::Impl\n"
    "  float vsDensity = texture(volsliceVolumeTex, volsliceSTP).r;\n"
    "  vec4 vsColor = texture(volsliceTfTex,\n"
    "                         vec2((vsDensity * 255.0 + 0.5) / 256.0, 0.5));\n"
    "  vsColor.a = 1.0 - pow(1.0 - vsColor.a, volsliceAlphaExp);\n"
    "  gl_FragData[0] = vsColor;\n";

cvc::volslice::mat4 to_mat4(vtkMatrix4x4 *m) {
  cvc::volslice::mat4 out;
  if (m)
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        out.m[r * 4 + c] = m->GetElement(r, c);
  return out;
}

} // namespace

VolSliceNode::VolSliceNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GeometryNode(ctx, statePath, name) {
  m_stateSettings = std::make_unique<cvc::volslice::state_settings>(
      ctx, stateName("volslice"), [this](const cvc::volslice::render_settings &) {
        ++m_settingsVersion;
        if (m_sceneGraph)
          m_sceneGraph->requestRender();
      });

  // Unlit: the fragment replacement overwrites the color, and white
  // ambient-only underneath keeps every earlier shader stage harmless
  // (the VolRenNode quad trick).
  setUseSingleColor(true);
  setColor(1.0, 1.0, 1.0);
  setAmbient(1.0);
  setDiffuse(0.0);
  setSpecular(0.0);

  addVertexShaderReplacement("//VTK::CustomUniforms::Dec", kVertexUniformsDec);
  addVertexShaderReplacement("//VTK::PositionVC::Impl", kVertexPositionImpl);
  addFragmentShaderReplacement("//VTK::CustomUniforms::Dec", kFragmentUniformsDec);
  addFragmentShaderReplacement("//VTK::TCoord::Impl", kFragmentSampleImpl);
  setShaderUniform3f("volsliceTexScale", 1.f, 1.f, 1.f);
  setShaderUniform3f("volsliceTexOffset", 0.f, 0.f, 0.f);
  setShaderUniformf("volsliceAlphaExp", 1.f);

  // The slice stack must blend sequentially; an "opaque" classification would
  // give it depth writes and no blending at all.  There is no vtkTexture on
  // the actor and opacity is 1.0, so VTK needs telling explicitly.
  if (auto *actor = vtkActor::SafeDownCast(getProp())) {
    actor->ForceTranslucentOn();
    // The vertex shader computes the volume texcoord from vertexMC, which is
    // only the TRUE model coordinate while the mapper's AUTO shift-scale is
    // off: with it on (it engages for boxes big enough or far enough from
    // the origin -- a volume at z in [0,100] triggered it, the unit ball in
    // the tests did not), the VBO holds re-centered coordinates and the
    // compensation lives in MCDCMatrix, so the texcoord map reads garbage
    // and the whole stack samples one corner of the volume.  Found as "the
    // demo bunny is invisible but the test ball renders" -- density 0 at the
    // sampled corner made every slice fully transparent.
    if (auto *pdm = vtkPolyDataMapper::SafeDownCast(actor->GetMapper()))
      pdm->SetVBOShiftScaleMethod(vtkPolyDataMapper::ShiftScaleMethodType::DISABLE_SHIFT_SCALE);
  }
}

VolSliceNode::~VolSliceNode() = default;

void VolSliceNode::setVolume(const cvc::volume &vol) {
  {
    std::lock_guard<std::mutex> lock(m_volumeMutex);
    m_volume = vol; // shallow copy (copy-on-write buffers)
    ++m_volumeVersion;
  }
  if (m_sceneGraph)
    m_sceneGraph->requestRender();
}

bool VolSliceNode::hasVolume() const {
  std::lock_guard<std::mutex> lock(m_volumeMutex);
  return m_volume.has_value();
}

cvc::volslice::render_settings VolSliceNode::config() const { return m_stateSettings->get(); }

void VolSliceNode::setConfig(const cvc::volslice::render_settings &s) {
  m_stateSettings->set(s);
  ++m_settingsVersion;
  if (m_sceneGraph)
    m_sceneGraph->requestRender();
}

cvc::bounding_box VolSliceNode::getBoundingBox() const {
  std::lock_guard<std::mutex> lock(m_volumeMutex);
  if (m_volume)
    return m_volume->boundingBox();
  return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

void VolSliceNode::addToRenderer(vtkRenderer *renderer) {
  GeometryNode::addToRenderer(renderer);
  if (renderer && renderer->GetUseOIT()) {
    // See the header: OIT would average the slice stack instead of
    // compositing it.  Deliberately NOT restored on removal -- other slice
    // nodes may share this renderer.
    renderer->UseOITOff();
    app().log(3, "VolSliceNode: switched renderer to sequential translucency "
                 "(UseOITOff) -- slice compositing is order-dependent");
  }
}

std::pair<double, double>
VolSliceNode::effectiveWindow(const cvc::volslice::render_settings &s) const {
  // Caller holds m_volumeMutex.
  double lo = s.window_min, hi = s.window_max;
  if (!(hi > lo)) { // unset (0,0) or degenerate: fall back to the data range
    lo = m_volume->min();
    hi = m_volume->max();
  }
  if (!(hi > lo))
    hi = lo + 1.0; // constant volume: any non-empty window works
  return {lo, hi};
}

bool VolSliceNode::uploadVolumeTexture(const cvc::volslice::render_settings &s) {
  if (!m_renderer || !m_renderer->GetRenderWindow())
    return false;
  auto *oglWin = vtkOpenGLRenderWindow::SafeDownCast(m_renderer->GetRenderWindow());
  if (!oglWin || !oglWin->GetInitialized())
    return false;

  std::lock_guard<std::mutex> lock(m_volumeMutex);
  if (!m_volume)
    return false;
  const auto [lo, hi] = effectiveWindow(s);
  const bool dataDirty =
      m_uploadedVolume != m_volumeVersion || m_uploadedWindowLo != lo || m_uploadedWindowHi != hi;
  const bool filterDirty = m_uploadedFilter != int(s.filter);
  if (!dataDirty && !filterDirty)
    return true;

  if (!m_volumeTexture) {
    m_volumeTexture = vtkSmartPointer<vtkTextureObject>::New();
    m_volumeTexture->SetContext(oglWin);
  }

  if (dataDirty) {
    // The legacy UChar coercion: normalize [lo,hi] into a byte texture.  R8 is
    // filterable everywhere (an R32F upload would silently lose LINEAR
    // filtering on WebGL2 without OES_texture_float_linear).
    const auto w = unsigned(m_volume->XDim()), h = unsigned(m_volume->YDim()),
               d = unsigned(m_volume->ZDim());
    std::vector<unsigned char> bytes(std::size_t(w) * h * d);
    const double scale = 255.0 / (hi - lo);
    std::size_t idx = 0;
    for (unsigned k = 0; k < d; ++k)
      for (unsigned j = 0; j < h; ++j)
        for (unsigned i = 0; i < w; ++i) {
          double v = ((*m_volume)(i, j, k) - lo) * scale;
          bytes[idx++] = static_cast<unsigned char>(v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v));
        }
    m_volumeTexture->Create3DFromRaw(w, h, d, 1, VTK_UNSIGNED_CHAR, bytes.data());
    m_volumeTexture->SetWrapS(vtkTextureObject::ClampToEdge);
    m_volumeTexture->SetWrapT(vtkTextureObject::ClampToEdge);
    m_volumeTexture->SetWrapR(vtkTextureObject::ClampToEdge);
    m_uploadedVolume = m_volumeVersion;
    m_uploadedWindowLo = lo;
    m_uploadedWindowHi = hi;
  }

  const bool linear = s.filter == cvc::volslice::interpolation::linear;
  m_volumeTexture->SetMinificationFilter(linear ? vtkTextureObject::Linear
                                                : vtkTextureObject::Nearest);
  m_volumeTexture->SetMagnificationFilter(linear ? vtkTextureObject::Linear
                                                 : vtkTextureObject::Nearest);
  m_uploadedFilter = int(s.filter);
  setShaderTexture("volsliceVolumeTex", m_volumeTexture);
  return true;
}

bool VolSliceNode::uploadTransferFunction(const cvc::volslice::render_settings &s) {
  if (!m_renderer || !m_renderer->GetRenderWindow())
    return false;
  auto *oglWin = vtkOpenGLRenderWindow::SafeDownCast(m_renderer->GetRenderWindow());
  if (!oglWin || !oglWin->GetInitialized())
    return false;

  double lo, hi;
  {
    std::lock_guard<std::mutex> lock(m_volumeMutex);
    if (!m_volume)
      return false;
    std::tie(lo, hi) = effectiveWindow(s);
  }
  if (m_uploadedTf == m_settingsVersion && m_uploadedWindowLo == lo && m_uploadedWindowHi == hi &&
      m_tfTexture)
    return true;

  // 256 RGBA8 entries over the normalization window, so LUT index i
  // corresponds exactly to density byte i (the legacy coldentbl layout).
  std::array<unsigned char, 256 * 4> lut{};
  for (int i = 0; i < 256; ++i) {
    const double v = lo + (hi - lo) * (double(i) / 255.0);
    const cvc::volslice::rgba_f c = s.tf.sample(v);
    lut[i * 4 + 0] = static_cast<unsigned char>(std::fmin(1.f, std::fmax(0.f, c.r)) * 255.f + 0.5f);
    lut[i * 4 + 1] = static_cast<unsigned char>(std::fmin(1.f, std::fmax(0.f, c.g)) * 255.f + 0.5f);
    lut[i * 4 + 2] = static_cast<unsigned char>(std::fmin(1.f, std::fmax(0.f, c.b)) * 255.f + 0.5f);
    lut[i * 4 + 3] = static_cast<unsigned char>(std::fmin(1.f, std::fmax(0.f, c.a)) * 255.f + 0.5f);
  }

  if (!m_tfTexture) {
    m_tfTexture = vtkSmartPointer<vtkTextureObject>::New();
    m_tfTexture->SetContext(oglWin);
  }
  m_tfTexture->Create2DFromRaw(256, 1, 4, VTK_UNSIGNED_CHAR, lut.data());
  m_tfTexture->SetWrapS(vtkTextureObject::ClampToEdge);
  m_tfTexture->SetWrapT(vtkTextureObject::ClampToEdge);
  m_tfTexture->SetMinificationFilter(vtkTextureObject::Linear);
  m_tfTexture->SetMagnificationFilter(vtkTextureObject::Linear);
  m_uploadedTf = m_settingsVersion;
  setShaderTexture("volsliceTfTex", m_tfTexture);
  return true;
}

void VolSliceNode::rebuildSliceGeometry(const cvc::volslice::mat4 &localToClip,
                                        const cvc::volslice::render_settings &s) {
  cvc::volslice::box3d box;
  {
    std::lock_guard<std::mutex> lock(m_volumeMutex);
    const cvc::bounding_box bb = m_volume->boundingBox();
    box.min = {bb.minx, bb.miny, bb.minz};
    box.max = {bb.maxx, bb.maxy, bb.maxz};
  }

  const cvc::volslice::slice_geometry geo =
      cvc::volslice::compute_slices(localToClip, box, s.slices);
  m_planesRendered = geo.planes();

  // Rebuild the mesh: fan-triangulated slices, appended in sweep (= draw)
  // order -- GL rasterizes cells in order, which IS the back-to-front
  // compositing.  Topology changes with every camera move (fan counts vary as
  // planes cross corners), so this goes through setGeometry() rather than the
  // fixed-topology updateVertices() path.
  cvc::geometry mesh;
  mesh.points().reserve(geo.vertices());
  for (std::size_t v = 0; v < geo.vertices(); ++v) {
    cvc::geometry::point_t pt;
    pt[0] = geo.positions[v * 3 + 0];
    pt[1] = geo.positions[v * 3 + 1];
    pt[2] = geo.positions[v * 3 + 2];
    mesh.points().push_back(pt);
  }
  for (std::size_t p = 0; p < geo.planes(); ++p) {
    const auto off = geo.fan_offset[p], cnt = geo.fan_count[p];
    for (std::uint32_t v = 1; v + 1 < cnt; ++v) {
      cvc::geometry::tri_t t;
      t[0] = off;
      t[1] = off + v;
      t[2] = off + v + 1;
      mesh.tris().push_back(t);
    }
  }
  setGeometry(mesh);

  // Texcoord map: stp = p * scale + offset over the volume box.
  const double dx = box.max.x - box.min.x, dy = box.max.y - box.min.y, dz = box.max.z - box.min.z;
  const double sx = dx > 0 ? 1.0 / dx : 1.0, sy = dy > 0 ? 1.0 / dy : 1.0,
               sz = dz > 0 ? 1.0 / dz : 1.0;
  setShaderUniform3f("volsliceTexScale", float(sx), float(sy), float(sz));
  setShaderUniform3f("volsliceTexOffset", float(-box.min.x * sx), float(-box.min.y * sy),
                     float(-box.min.z * sz));

  // Opacity correction (see volslice/types.h): exponent = actual spacing over
  // the spacing at DEFAULT quality for this box, so TF alpha is authored at
  // default quality and other qualities change sharpness, not density.
  float alphaExp = 1.f;
  if (s.opacity_correction && geo.plane_spacing > 0.0) {
    cvc::volslice::slice_params ref; // defaults
    const double longest = std::fmax(dx, std::fmax(dy, dz));
    if (longest > 0.0) {
      const double minr = std::fmin(dx, std::fmin(dy, dz)) / longest;
      const double refN = 2.0 * (10.0 + ref.max_planes * ref.quality * ref.quality * ref.quality);
      const double refSpacing = minr / refN * longest;
      if (refSpacing > 0.0)
        alphaExp = float(geo.plane_spacing / refSpacing);
    }
  }
  setShaderUniformf("volsliceAlphaExp", alphaExp);
}

void VolSliceNode::depthSortSliceProps(vtkRenderer *renderer,
                                       const std::vector<VolSliceNode *> &nodes) {
  if (!renderer || nodes.size() < 2)
    return;
  vtkCamera *cam = renderer->GetActiveCamera();
  if (!cam)
    return;
  double pos[3], dop[3];
  cam->GetPosition(pos);
  cam->GetDirectionOfProjection(dop);

  // View depth of each node's transformed box center.
  struct entry {
    double depth;
    VolSliceNode *node;
  };
  std::vector<entry> order;
  order.reserve(nodes.size());
  for (VolSliceNode *n : nodes) {
    if (!n)
      continue;
    const cvc::bounding_box bb = n->getBoundingBox();
    double c[4] = {(bb.minx + bb.maxx) * 0.5, (bb.miny + bb.maxy) * 0.5, (bb.minz + bb.maxz) * 0.5,
                   1.0};
    if (vtkMatrix4x4 *w = n->getWorldTransform())
      w->MultiplyPoint(c, c);
    const double d = (c[0] - pos[0]) * dop[0] + (c[1] - pos[1]) * dop[1] + (c[2] - pos[2]) * dop[2];
    order.push_back({d, n});
  }
  // Back to front: FARTHEST first in the prop list.
  std::stable_sort(order.begin(), order.end(),
                   [](const entry &a, const entry &b) { return a.depth > b.depth; });

  // Only touch the renderer when the relative order actually changed --
  // Remove+Add is how VTK exposes reordering, and it dirties the renderer.
  bool sorted = true;
  int lastIndex = -1;
  vtkPropCollection *props = renderer->GetViewProps();
  for (const entry &e : order) {
    vtkProp *p = e.node->getProp();
    const int idx = props->IsItemPresent(p); // 1-based; 0 = absent
    if (idx == 0 || idx <= lastIndex) {
      sorted = false;
      break;
    }
    lastIndex = idx;
  }
  if (sorted)
    return;
  for (const entry &e : order) {
    vtkProp *p = e.node->getProp();
    if (renderer->GetViewProps()->IsItemPresent(p)) {
      p->Register(renderer); // keep alive across the Remove/Add hop
      renderer->RemoveViewProp(p);
      renderer->AddViewProp(p);
      p->UnRegister(renderer);
    }
  }
}

bool VolSliceNode::tick() {
  if (!m_renderer)
    return false;
  vtkCamera *cam = m_renderer->GetActiveCamera();
  if (!cam || !hasVolume())
    return false;

  const cvc::volslice::render_settings s = m_stateSettings->get();

  bool changed = false;
  changed |= uploadVolumeTexture(s);
  // uploadVolumeTexture returning false just means GL is not ready yet;
  // "changed" only matters for geometry, tracked below.
  changed = false;
  uploadTransferFunction(s);

  // local -> clip = (world -> clip) * (local -> world).  The same matrices
  // VTK's vertex shader will use, so the sweep sees exactly the on-screen
  // orientation (the legacy renderer read these from the GL matrix stack).
  const double aspect = m_renderer->GetTiledAspectRatio();
  cvc::volslice::mat4 worldToClip =
      to_mat4(cam->GetCompositeProjectionTransformMatrix(aspect, -1, 1));
  cvc::volslice::mat4 localToWorld = to_mat4(getWorldTransform());
  const cvc::volslice::mat4 localToClip = worldToClip * localToWorld;

  const bool stale = localToClip.m != m_appliedMatrix || m_appliedSettings != m_settingsVersion ||
                     m_appliedVolume != m_volumeVersion;
  if (stale) {
    rebuildSliceGeometry(localToClip, s);
    m_appliedMatrix = localToClip.m;
    m_appliedSettings = m_settingsVersion;
    m_appliedVolume = m_volumeVersion;
    changed = true;
  }
  return changed;
}

} // namespace gl
} // namespace cvc

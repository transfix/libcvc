#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolRenNode.h>
#include <limits>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextureObject.h>

namespace cvc {
namespace gl {

namespace {

// GLSL injected at //VTK::CustomUniforms::Dec (top of the fragment shader).
//
// CRITICAL: the replacement RE-EMITS the anchor.  GeometryNode registers
// replacements with replaceFirst=true, i.e. ahead of VTK's own substitutions,
// so a replacement that drops the anchor also deletes whatever VTK would have
// put there.  The desktop polydata mapper leaves this anchor empty, but
// wasm's vtkOpenGLLowMemoryPolyDataMapper declares `primitiveSize` and
// `usesEdgeValues` in it -- eating the anchor made every fragment shader fail
// to compile in the browser while the desktop build stayed green.
const char *kDepthUniformsDec = "uniform sampler2D volrenDepthTex;\n"
                                "uniform float volrenNear;\n"
                                "uniform float volrenFar;\n"
                                "uniform int volrenPersp;\n"
                                "//VTK::CustomUniforms::Dec\n";

// GLSL injected at //VTK::Depth::Impl: convert the raycaster's eye-space
// depth to window z with the LIVE camera's clip range (VTK refits the range
// every frame, so this must use uniforms, not baked values).  Misses carry
// +inf eye depth -> clamps to 1.0, and their alpha-0 texels are discarded by
// the template's alpha test anyway.
//
// The anchor is re-emitted FIRST (same reason as above) so any mapper-supplied
// depth code still runs; ours follows and therefore wins the gl_FragDepth
// write.
const char *kDepthImpl =
    "//VTK::Depth::Impl\n"
    "  float vrEyeDepth = texture(volrenDepthTex, tcoordVCVSOutput).r;\n"
    "  float vrZ;\n"
    "  if (volrenPersp == 1)\n"
    "    vrZ = volrenFar / (volrenFar - volrenNear) * (1.0 - volrenNear / vrEyeDepth);\n"
    "  else\n"
    "    vrZ = (vrEyeDepth - volrenNear) / (volrenFar - volrenNear);\n"
    "  gl_FragDepth = clamp(vrZ, 0.0, 1.0);\n";

// GLSL injected at //VTK::TCoord::Impl: undo the PREMULTIPLIED alpha of the
// raycast texture, after VTK has sampled and modulated it.
//
// The raycaster's background is forced to black, so frame::color's RGB is the
// volume's color already multiplied by its own alpha.  Uploading that verbatim
// and dividing here -- rather than un-premultiplying on the CPU before the
// upload -- is what makes the GL texture FILTER correct: bilinear interpolation
// is only linear in premultiplied space.  Filtered straight alpha reads the
// RGB=0 of a transparent texel as if it were black paint, so every silhouette
// picks up a dark fringe: minified (resolution_scale > 1) a half-covered screen
// pixel gets half the volume's color at half alpha and composites at a QUARTER,
// magnified (scale < 1) the same halo appears smeared over several pixels.
//
// The anchor is re-emitted first (GeometryNode registers replacements with
// replaceFirst=true, so dropping it would delete VTK's own texture sampling),
// and the divide is by the FINAL fragment alpha -- which is the texture's alpha
// exactly because the node draws this quad unlit at opacity 1 (see the
// constructor).  A quad at any other opacity would have that opacity divided
// straight back out.
const char *kUnpremultiplyImpl = "//VTK::TCoord::Impl\n"
                                 "  if (gl_FragData[0].a > 0.0)\n"
                                 "    gl_FragData[0].rgb /= gl_FragData[0].a;\n";

cvc::volren::mat4 to_mat4(vtkMatrix4x4 *m) {
  cvc::volren::mat4 out;
  if (m)
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        out.m[r * 4 + c] = m->GetElement(r, c);
  return out;
}

} // namespace

// Camera + composed transform + settings captured on the owner thread for one
// raycast.  Everything the worker needs, nothing shared.
struct VolRenNode::snapshot {
  cvc::volren::camera cam;
  cvc::volren::mat4 node_world;
  cvc::volren::state_settings::snapshot settings;
  std::vector<cvc::volume> volumes;
  std::uint64_t version = 0;
  std::array<double, 11> camera_key{};
  double near_z = 0.1, far_z = 1000.0;
};

// The raycast worker: owns the raycaster (and thus its private thread pool)
// and a single render thread with a one-slot latest-wins job queue.  On
// single-threaded wasm builds there is no thread — run() executes inline.
struct VolRenNode::worker {
  explicit worker(cvc::app &ctx) : rc(ctx) {
#if defined(__EMSCRIPTEN_PTHREADS__)
    // Bounded fan-out: demos pre-spawn a small pthread pool; an unbounded
    // hardware_concurrency()-1 pool can deadlock waiting on threads the
    // browser has not started yet.
    pool = std::make_unique<cvc::thread_pool>(2);
    rc.set_thread_pool(pool.get());
#endif
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    thread = std::thread([this] { loop(); });
#endif
  }

  ~worker() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
      has_job = false;
    }
    cv.notify_all();
    if (thread.joinable())
      thread.join();
  }

  // Latest camera wins: overwrite any not-yet-started job.
  void submit(snapshot snap) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      job = std::move(snap);
      has_job = true;
    }
    cv.notify_one();
  }

  bool busy() const { return in_flight.load(std::memory_order_acquire); }

  // One raycast; used directly on single-threaded wasm.
  cvc::volren::frame run(const snapshot &snap, double &seconds) {
    rc.clear_volumes();
    for (std::size_t i = 0; i < snap.volumes.size(); ++i) {
      cvc::volren::volume_settings vs;
      if (i < snap.settings.volumes.size())
        vs = snap.settings.volumes[i];
      // Scene placement: node world matrix composed over the volume's own.
      vs.model_transform = snap.node_world * vs.model_transform;
      rc.add_volume(snap.volumes[i], vs);
    }
    // AFTER re-registering, never before: registration deliberately does not
    // draw a fresh content generation (the node clears and re-adds every
    // frame, so stamping there would make every frame a cache miss), which
    // means clear_volumes() resets the generations an announcement has to
    // bump.  Bumping them also busts the shadow-map fingerprint, which reads
    // the same generations.
    if (invalidate_pending.exchange(false, std::memory_order_acq_rel))
      rc.invalidate_device_volumes();
    rc.settings() = snap.settings.settings;
    // Alpha carries all compositing; the scene provides the background.
    rc.settings().background = {0.f, 0.f, 0.f};
    rc.view() = snap.cam;

    const auto start = std::chrono::steady_clock::now();
    cvc::volren::frame f = rc.render();
    seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    used_backend = static_cast<int>(rc.backend_used());
    return f;
  }

  cvc::volren::raycaster rc;
  std::atomic<int> used_backend{0}; // backend ordinal of the last completed render
  std::unique_ptr<cvc::thread_pool> pool;
  std::thread thread;
  std::mutex mutex;
  std::condition_variable cv;
  snapshot job;
  bool has_job = false;
  bool stopping = false;
  std::atomic<bool> in_flight{false};
  // Armed by the owner thread (invalidateVolumeData), consumed by whichever
  // thread runs the raycast; never touches the raycaster off-thread.
  std::atomic<bool> invalidate_pending{false};

  // Set by the node; called from the worker thread with the finished frame.
  std::function<void(cvc::volren::frame, snapshot, double)> on_frame;

  void loop() {
    for (;;) {
      snapshot snap;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return has_job || stopping; });
        if (stopping)
          return;
        snap = std::move(job);
        has_job = false;
        in_flight.store(true, std::memory_order_release);
      }
      double seconds = 0.0;
      try {
        cvc::volren::frame f = run(snap, seconds);
        if (on_frame)
          on_frame(std::move(f), std::move(snap), seconds);
      } catch (...) {
        // A raycast failure (e.g. a setting made degenerate mid-flight) drops
        // the frame; the node keeps showing the previous one.
        //
        // The catch-all is REQUIRED, not lazy: cvc::volren_error comes from
        // CVC_DEF_EXCEPTION, and cvc::exception derives from boost::exception
        // ALONE -- it is not a std::exception. Every throw out of
        // raycaster::render() is one, so `catch (const std::exception &)` here
        // caught nothing and let the exception escape the worker's
        // std::thread, which is an immediate std::terminate().
      }
      in_flight.store(false, std::memory_order_release);
    }
  }
};

VolRenNode::VolRenNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GeometryNode(ctx, statePath, name) {
  m_stateSettings = std::make_unique<cvc::volren::state_settings>(
      ctx, stateName("volren"), [this](const cvc::volren::state_settings::snapshot &s) {
        std::lock_guard<std::mutex> lock(m_configMutex);
        m_snapshotSettings = s;
        ++m_settingsVersion;
      });
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_snapshotSettings = m_stateSettings->get();
  }
  seedOwnState();

  m_worker = std::make_unique<worker>(ctx);
  m_worker->on_frame = [this](cvc::volren::frame f, snapshot snap, double seconds) {
    m_lastRenderSeconds.store(seconds);
    m_backendUsed.store(m_worker->used_backend.load());
    // Marshal to the owner thread; weak-guarded, so a dying node drops it.
    auto shared_f = std::make_shared<cvc::volren::frame>(std::move(f));
    auto shared_s = std::make_shared<snapshot>(std::move(snap));
    runOnMainThread([this, shared_f, shared_s] {
      applyFrame(*shared_f, *shared_s);
      if (m_sceneGraph)
        m_sceneGraph->requestRender();
    });
  };

  // The quad shows the raycast image as-is: unlit (ambient-only, white).
  setUseSingleColor(true);
  setColor(1.0, 1.0, 1.0);
  setAmbient(1.0);
  setDiffuse(0.0);
  setSpecular(0.0);

  addFragmentShaderReplacement("//VTK::CustomUniforms::Dec", kDepthUniformsDec);
  addFragmentShaderReplacement("//VTK::Depth::Impl", kDepthImpl);
  addFragmentShaderReplacement("//VTK::TCoord::Impl", kUnpremultiplyImpl);
  setShaderUniformf("volrenNear", 0.1f);
  setShaderUniformf("volrenFar", 1000.f);
  setShaderUniformi("volrenPersp", 1);
}

VolRenNode::~VolRenNode() { m_worker.reset(); }

void VolRenNode::seedOwnState() {
  getState("volren.resolution_scale").value(m_resolutionScale);
  getState("volren.continuous").value(m_continuous ? 1 : 0);
}

std::size_t VolRenNode::addVolume(const cvc::volume &vol, cvc::volren::volume_settings vs) {
  cvc::volren::state_settings::snapshot next;
  std::size_t index = 0;
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_volumes.push_back(vol);
    m_snapshotSettings.volumes.push_back(std::move(vs));
    ++m_settingsVersion;
    next = m_snapshotSettings;
    index = m_volumes.size() - 1;
  }
  m_stateSettings->set(next); // mirror to the tree (does not re-fire apply)
  return index;
}

void VolRenNode::clearVolumes() {
  cvc::volren::state_settings::snapshot next;
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_volumes.clear();
    m_snapshotSettings.volumes.clear();
    ++m_settingsVersion;
    next = m_snapshotSettings;
  }
  m_stateSettings->set(next);
}

std::size_t VolRenNode::volumeCount() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_volumes.size();
}

cvc::volren::volume_settings VolRenNode::volumeConfig(std::size_t index) const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  if (index >= m_snapshotSettings.volumes.size())
    throw cvc::volren_error("VolRenNode: volume index out of range");
  return m_snapshotSettings.volumes[index];
}

void VolRenNode::setVolumeConfig(std::size_t index, const cvc::volren::volume_settings &vs) {
  cvc::volren::state_settings::snapshot next;
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index >= m_snapshotSettings.volumes.size())
      throw cvc::volren_error("VolRenNode: volume index out of range");
    m_snapshotSettings.volumes[index] = vs;
    ++m_settingsVersion;
    next = m_snapshotSettings;
  }
  m_stateSettings->set(next);
}

cvc::volren::render_settings VolRenNode::renderConfig() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings;
}

void VolRenNode::setRenderConfig(const cvc::volren::render_settings &rs) {
  cvc::volren::state_settings::snapshot next;
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_snapshotSettings.settings = rs;
    ++m_settingsVersion;
    next = m_snapshotSettings;
  }
  m_stateSettings->set(next);
}

void VolRenNode::setResolutionScale(double scale) {
  getState("volren.resolution_scale")
      .value(std::clamp(scale, MinResolutionScale, MaxResolutionScale));
}

double VolRenNode::resolutionScale() const { return m_resolutionScale; }

void VolRenNode::setSupersample(int n) {
  cvc::volren::render_settings rs = renderConfig();
  rs.supersample = n;
  setRenderConfig(rs);
}

int VolRenNode::supersample() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.supersample;
}

void VolRenNode::setShadowsEnabled(bool on) {
  cvc::volren::render_settings rs = renderConfig();
  rs.shadows.enabled = on;
  setRenderConfig(rs);
}

bool VolRenNode::shadowsEnabled() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows.enabled;
}

cvc::volren::shadow_settings VolRenNode::shadowConfig() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows;
}

void VolRenNode::setShadowConfig(const cvc::volren::shadow_settings &ss) {
  cvc::volren::render_settings rs = renderConfig();
  rs.shadows = ss;
  setRenderConfig(rs);
}

void VolRenNode::invalidateVolumeData() {
  m_worker->invalidate_pending.store(true, std::memory_order_release);
  // The displayed frame is stale by definition, and nothing else changed, so
  // bump the version to make tick() relaunch.  No state write: the voxels
  // moved, not a setting, and the tree does not carry them.
  std::lock_guard<std::mutex> lock(m_configMutex);
  ++m_settingsVersion;
}

void VolRenNode::setContinuous(bool on) { getState("volren.continuous").value(on ? 1 : 0); }

void VolRenNode::setBackend(cvc::volren::backend b) { m_worker->rc.set_backend(b); }

cvc::volren::backend VolRenNode::backendUsed() const {
  return static_cast<cvc::volren::backend>(m_backendUsed.load());
}

bool VolRenNode::continuous() const { return m_continuous; }

cvc::bounding_box VolRenNode::getBoundingBox() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  cvc::bounding_box out(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
  bool first = true;
  for (std::size_t i = 0; i < m_volumes.size(); ++i) {
    const cvc::bounding_box &b = m_volumes[i].boundingBox();
    cvc::volren::mat4 mt;
    if (i < m_snapshotSettings.volumes.size())
      mt = m_snapshotSettings.volumes[i].model_transform;
    for (int corner = 0; corner < 8; ++corner) {
      const cvc::volren::vec3d p =
          mt.transform_point({corner & 1 ? b.maxx : b.minx, corner & 2 ? b.maxy : b.miny,
                              corner & 4 ? b.maxz : b.minz});
      if (first) {
        out.minx = out.maxx = p.x;
        out.miny = out.maxy = p.y;
        out.minz = out.maxz = p.z;
        first = false;
      } else {
        out.minx = std::min(out.minx, p.x);
        out.miny = std::min(out.miny, p.y);
        out.minz = std::min(out.minz, p.z);
        out.maxx = std::max(out.maxx, p.x);
        out.maxy = std::max(out.maxy, p.y);
        out.maxz = std::max(out.maxz, p.z);
      }
    }
  }
  return out;
}

void VolRenNode::applyTransformToVTK() {
  // Deliberately do NOT move the actor: the quad is glued to the raycast
  // camera pose.  The scene transform reaches the volume through the
  // composed matrix captured in tick() — a moved ancestor changes the
  // snapshot and triggers a re-raycast.
}

void VolRenNode::handleStateChanged(const std::string &childState) {
  if (childState.rfind("volren.", 0) == 0) {
    if (childState == "volren.resolution_scale" || childState == "volren.continuous") {
      try {
        m_resolutionScale = std::clamp(getState("volren.resolution_scale").value<double>(),
                                       MinResolutionScale, MaxResolutionScale);
        m_continuous = getState("volren.continuous").value<int>() != 0;
      } catch (...) {
        // Partially-initialised state. Catch-all because the state tree throws
        // BOTH boost::bad_lexical_cast (a std::exception) and
        // cvc::type_conversion_error (CVC_DEF_EXCEPTION -> boost::exception,
        // which is NOT a std::exception).
      }
    }
    // Everything else under volren.* belongs to the embedded state_settings
    // object, which observes its own subtree; do not forward to the base.
    return;
  }
  GeometryNode::handleStateChanged(childState);
}

bool VolRenNode::buildSnapshot(snapshot &out) {
  if (!m_renderer)
    return false;
  vtkCamera *cam = m_renderer->GetActiveCamera();
  if (!cam)
    return false;
  const int *size = m_renderer->GetSize();
  if (!size || size[0] < 2 || size[1] < 2)
    return false;

  double eye[3], focal[3], up[3];
  cam->GetPosition(eye);
  cam->GetFocalPoint(focal);
  cam->GetViewUp(up);

  const int w = std::max(2, int(std::lround(size[0] * m_resolutionScale)));
  const int h = std::max(2, int(std::lround(size[1] * m_resolutionScale)));

  out.cam = cvc::volren::camera::from_pose(eye, focal, up, cam->GetViewAngle(), w, h);
  if (cam->GetParallelProjection()) {
    out.cam.projection = cvc::volren::camera::projection_type::orthographic;
    out.cam.parallel_scale = cam->GetParallelScale();
  }

  double clip[2];
  cam->GetClippingRange(clip);
  out.near_z = clip[0];
  out.far_z = clip[1];

  out.node_world = to_mat4(m_worldMatrix);

  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (m_volumes.empty())
      return false;
    out.settings = m_snapshotSettings;
    out.volumes = m_volumes;
    out.version = m_settingsVersion;
  }

  out.camera_key = {out.cam.eye[0],   out.cam.eye[1],       out.cam.eye[2],        out.cam.focal[0],
                    out.cam.focal[1], out.cam.focal[2],     out.cam.up[0],         out.cam.up[1],
                    out.cam.up[2],    out.cam.vfov_degrees, out.cam.parallel_scale};
  return true;
}

void VolRenNode::pushDepthUniforms() {
  if (!m_renderer)
    return;
  vtkCamera *cam = m_renderer->GetActiveCamera();
  if (!cam)
    return;
  double clip[2];
  cam->GetClippingRange(clip);
  setShaderUniformf("volrenNear", float(clip[0]));
  setShaderUniformf("volrenFar", float(clip[1]));
  setShaderUniformi("volrenPersp", cam->GetParallelProjection() ? 0 : 1);
}

void VolRenNode::ensureQuad() {
  if (m_quadReady)
    return;
  // Placeholder unit quad; applyFrame() re-poses it to hug the raycast
  // camera's frustum, in the order {bottom-left, bottom-right, top-right,
  // top-left}.
  //
  // UV orientation, the subtle bit: the raycast image is TOP-LEFT origin, so
  // texture v=0 is its top row -- and GeometryNode flips V in the TCoords for
  // exactly that reason (m_textureFlipV).  To land the image's top row on the
  // screen's top edge we must therefore supply the INVERSE of the wanted
  // mapping: v=0 on the bottom corners, v=1 on the top ones.  Getting this
  // backwards renders the frame upside down -- invisible on a vertically
  // symmetric test volume, glaring on a bunny.
  cvc::geometry quad(app());
  auto &pts = quad.points();
  pts = {{-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0}, {0.5, 0.5, 0.0}, {-0.5, 0.5, 0.0}};
  auto &uvs = quad.uvs();
  uvs = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  auto &tris = quad.tris();
  tris = {{0, 1, 2}, {0, 2, 3}};
  quad.set_geometry_type(cvc::geometry::SURFACE_TRI);
  setGeometry(quad);
  m_quadReady = true;
}

void VolRenNode::applyFrame(const cvc::volren::frame &f, const snapshot &snap) {
  const cvc::image &color = f.color;
  const cvc::image &depth = f.depth;
  const int w = color.width();
  const int h = color.height();
  if (w < 1 || h < 1)
    return;

  ensureQuad();

  // Re-pose the quad across the raycast camera's frustum at the distance of
  // the scene bounds' center, so the image stays world-anchored (and sorts
  // sanely among translucent actors).  gl_FragDepth supplies the real depth.
  {
    const cvc::volren::view_basis basis = snap.cam.basis();
    const cvc::volren::vec3d eye(snap.cam.eye);
    const cvc::volren::vec3d forward = -basis.back;
    // Distance to the (composed) volume center, clamped inside the clip range.
    cvc::volren::vec3d center{0, 0, 0};
    if (!snap.volumes.empty()) {
      const cvc::bounding_box &b = snap.volumes.front().boundingBox();
      cvc::volren::mat4 mt = snap.node_world;
      if (!snap.settings.volumes.empty())
        mt = snap.node_world * snap.settings.volumes.front().model_transform;
      center = mt.transform_point(
          {0.5 * (b.minx + b.maxx), 0.5 * (b.miny + b.maxy), 0.5 * (b.minz + b.maxz)});
    }
    double d = dot(center - eye, forward);
    d = std::min(std::max(d, snap.near_z * 1.01), snap.far_z * 0.99);

    double half_h, half_w;
    if (snap.cam.projection == cvc::volren::camera::projection_type::perspective) {
      half_h = d * std::tan(0.5 * snap.cam.vfov_degrees * 3.14159265358979323846 / 180.0);
    } else {
      half_h = snap.cam.parallel_scale;
    }
    half_w = half_h * snap.cam.aspect();

    const cvc::volren::vec3d c = eye + forward * d;
    const cvc::volren::vec3d r = basis.right * half_w;
    const cvc::volren::vec3d u = basis.true_up * half_h;
    const cvc::volren::vec3d corners[4] = {c - r - u, c + r - u, c + r + u, c - r + u};
    std::vector<double> xyz;
    xyz.reserve(12);
    for (const auto &p : corners) {
      xyz.push_back(p.x);
      xyz.push_back(p.y);
      xyz.push_back(p.z);
    }
    updateVertices(xyz);
  }

  // Color: copy the frame VERBATIM into the persistent aliased buffer.  The
  // raycaster's background is black, so its RGB is already premultiplied by
  // alpha, and kUnpremultiplyImpl divides it back out in the fragment shader --
  // AFTER the texture filter, which is the only place the division is
  // interpolation-correct.  (The buffer is aliased zero-copy by the texture,
  // so this writes through it rather than re-creating a texture per frame; the
  // frame itself is a temporary the worker hands over.)
  if (m_colorImage.width() != w || m_colorImage.height() != h) {
    m_colorImage = cvc::image(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    std::memset(m_colorImage.data(), 0, m_colorImage.size_bytes());
    setTexture(m_colorImage, /*zeroCopy=*/true);
  }
  std::memcpy(m_colorImage.storage().get(), color.data(), std::size_t(w) * h * 4);
  texture_modified();

  // Depth: (re)upload as an R32F texture.  Needs a live, initialized GL
  // window (lazy — guaranteed after the first scene render).
  if (m_renderer && m_renderer->GetRenderWindow()) {
    auto *oglWin = vtkOpenGLRenderWindow::SafeDownCast(m_renderer->GetRenderWindow());
    if (oglWin && oglWin->GetInitialized()) {
      if (!m_depthTexture) {
        m_depthTexture = vtkSmartPointer<vtkTextureObject>::New();
        m_depthTexture->SetContext(oglWin);
      }
      m_depthTexture->Create2DFromRaw(static_cast<unsigned int>(w), static_cast<unsigned int>(h), 1,
                                      VTK_FLOAT, const_cast<unsigned char *>(depth.data()));
      m_depthTexture->SetWrapS(vtkTextureObject::ClampToEdge);
      m_depthTexture->SetWrapT(vtkTextureObject::ClampToEdge);
      m_depthTexture->SetMinificationFilter(vtkTextureObject::Nearest);
      m_depthTexture->SetMagnificationFilter(vtkTextureObject::Nearest);
      setShaderTexture("volrenDepthTex", m_depthTexture);
    }
  }

  m_appliedVersion = snap.version;
  m_appliedMatrix = snap.node_world.m;
  m_appliedCamera = snap.camera_key;
  m_appliedW = w;
  m_appliedH = h;
  m_framesRendered.fetch_add(1);
  m_frameAppliedSinceTick = true;
}

void VolRenNode::launchOrRun(const snapshot &snap) {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
  // Single-threaded wasm: raycast synchronously at the (reduced) raster.
  double seconds = 0.0;
  try {
    cvc::volren::frame f = m_worker->run(snap, seconds);
    m_lastRenderSeconds.store(seconds);
    applyFrame(f, snap);
    if (m_sceneGraph)
      m_sceneGraph->requestRender();
  } catch (...) {
    // Same contract as the worker thread: drop the frame, keep the last one.
    // Catch-all because cvc::volren_error is not a std::exception.
  }
#else
  m_worker->submit(snap);
#endif
}

bool VolRenNode::tick() {
  m_frameAppliedSinceTick = false;
  pushDepthUniforms();

  snapshot snap;
  if (!buildSnapshot(snap))
    return false;

  const bool camera_changed = snap.camera_key != m_appliedCamera || snap.cam.width != m_appliedW ||
                              snap.cam.height != m_appliedH;
  const bool matrix_changed = snap.node_world.m != m_appliedMatrix;
  const bool settings_changed = snap.version != m_appliedVersion;
  const bool stale = camera_changed || matrix_changed || settings_changed;

  // Converged == what is ON SCREEN matches the live camera, transform and
  // settings, and nothing is still cooking.  "No new frame arrived" is NOT the
  // same thing: while a raycast is in flight tick() deliberately does not
  // relaunch, so a slow frame looks exactly like a settled one from the
  // outside.  Anything that waits for a finished image -- an offscreen PNG
  // capture, a screenshot test -- must poll this, or under load it captures
  // whatever stale frame happened to be applied first.
  m_converged.store(!stale && !m_worker->busy(), std::memory_order_relaxed);

  if ((m_continuous || stale) && !m_worker->busy()) {
    launchOrRun(snap);
  }
  return m_frameAppliedSinceTick;
}

} // namespace gl
} // namespace cvc

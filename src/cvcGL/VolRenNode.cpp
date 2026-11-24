#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h> // newPolyDataMapper: a view's quad, as the node's own
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolRenNode.h>
#include <functional>
#include <limits>
#include <map>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h> // GL_VIEWPORT

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
//
// volrenViewport is the draw's viewport size and volrenOrigin its lower-left
// corner, both in the pixels gl_FragCoord counts: (0, 0) for the node's own
// renderer, which fills its window, and a view's rect where it draws into the
// window under the UI (viewProp's views, onViewShader). The quad's program is
// shared by every renderer that draws it, so each draw sets every one of these.
const char *kDepthUniformsDec = "uniform sampler2D volrenDepthTex;\n"
                                "uniform sampler2D volrenColorTex;\n"
                                "uniform vec3 volrenViewport;\n"
                                "uniform vec3 volrenOrigin;\n"
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
    // The quad is a screen-filling billboard (poseQuad sizes it to the frustum),
    // so the raycast texel under this fragment is its normalized window position.
    // Deriving the UV from gl_FragCoord -- rather than a VTK texcoord varying --
    // is what keeps this working under the wasm/GLES mapper, which does NOT emit
    // tcoordVCVSOutput for this actor (empty //VTK::TCoord::Dec): referencing it
    // failed the whole fragment shader to compile and fell back to a flat quad.
    // Y is flipped: the raycast image is top-left origin, gl_FragCoord is bottom.
    // Relative to the draw's viewport (volrenOrigin), which is the window's
    // corner only for a renderer that fills it.
    "  vec2 vrFrag = gl_FragCoord.xy - volrenOrigin.xy;\n"
    "  vec2 vrUV = vec2(vrFrag.x, volrenViewport.y - vrFrag.y) / volrenViewport.xy;\n"
    "  float vrEyeDepth = texture(volrenDepthTex, vrUV).r;\n"
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
const char *kUnpremultiplyImpl =
    "//VTK::TCoord::Impl\n"
    // Sample the raycast colour OURSELVES from the same screen-space UV, rather
    // than leaning on VTK's texture path: the wasm/GLES mapper leaves this
    // actor's //VTK::TCoord::Dec/Impl empty, so VTK never sampled the image at
    // all (the quad showed flat material colour).  Straight-alpha result; the
    // premultiplied RGB is divided back out here (bilinear filtering is only
    // linear in premultiplied space -- see the note below).  Alpha-0 texels are
    // discarded by the template's alpha test that follows.
    "  vec2 vrFragC = gl_FragCoord.xy - volrenOrigin.xy;\n"
    "  vec2 vrUVc = vec2(vrFragC.x, volrenViewport.y - vrFragC.y) / volrenViewport.xy;\n"
    "  vec4 vrCol = texture(volrenColorTex, vrUVc);\n"
    // Opaque quad (see the ForceOpaqueOn below): alpha-test the silhouette and
    // emit full coverage. Emitting the raycast alpha here instead would let the
    // opaque pass composite half-covered edge texels at fractional coverage over
    // black -- a dark fringe the premultiplied blend never had (cvcgl_volren_node
    // guards this). The color is the straight (un-premultiplied) volume colour.
    "  if (vrCol.a < 0.5) discard;\n"
    "  gl_FragData[0] = vec4(vrCol.rgb / vrCol.a, 1.0);\n";

cvc::volren::mat4 to_mat4(vtkMatrix4x4 *m) {
  cvc::volren::mat4 out;
  if (m)
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        out.m[r * 4 + c] = m->GetElement(r, c);
  return out;
}

} // namespace

// A view's quad (VolRenNode::viewProp). Two things set it apart from a plain vtkActor:
//  - its draw's shader textures (the view's own picture and depth map, bound by onViewShader)
//    let go of their texture units once the draw is done, as the node's own actor's do
//    (GeometryNode::bindShaderTextures);
//  - its bounds are the volumes' box in the world, not the quad's. The quad spans the camera's
//    whole frustum, so as bounds it would frame a view's camera on the frustum and keep the
//    culler from ever leaving it out; the box is what the view shows, and the quad (at the box's
//    centre along the view direction) lies inside the clipping range a renderer fits to it.
class VolRenViewActor : public vtkOpenGLActor {
public:
  static VolRenViewActor *New();
  vtkTypeMacro(VolRenViewActor, vtkOpenGLActor);
  void Render(vtkRenderer *ren, vtkMapper *mapper) override {
    vtkOpenGLActor::Render(ren, mapper);
    for (auto &t : Bound)
      if (t)
        t->Deactivate();
    Bound.clear();
  }
  using vtkOpenGLActor::GetBounds;
  double *GetBounds() override { return VolumeBounds; }

  std::vector<vtkWeakPointer<vtkTextureObject>> Bound; // activated for the draw under way
  double VolumeBounds[6] = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

protected:
  VolRenViewActor() = default;
  ~VolRenViewActor() override = default;

private:
  VolRenViewActor(const VolRenViewActor &) = delete;
  void operator=(const VolRenViewActor &) = delete;
};
vtkStandardNewMacro(VolRenViewActor);

// A view of this node from another renderer (viewProp): its quad, its picture and depth map, and
// what the picture was raycast from. Owner thread, but for onViewShader, which reads the textures
// and the renderer during that renderer's draw (on the owner thread too).
struct VolRenNode::view_quad {
  VolRenNode *node = nullptr;
  std::uint64_t id = 0; // the worker's slot; never 0, the node's own renderer's
  vtkWeakPointer<vtkRenderer> renderer;
  vtkSmartPointer<VolRenViewActor> actor;
  vtkSmartPointer<vtkPolyDataMapper> mapper;
  vtkSmartPointer<vtkPolyData> poly;
  vtkSmartPointer<vtkCallbackCommand> shader_cb;
  vtkSmartPointer<vtkTextureObject> color, depth;

  std::uint64_t applied_version = ~0ull;
  std::array<double, 16> applied_matrix{};
  std::array<double, 11> applied_camera{};
  int applied_w = 0, applied_h = 0;
  std::vector<double> applied_clip;

  std::uint64_t launched = 0, applied = 0;
  bool has_frame = false; // a picture has landed: the quad has something to show
  bool converged = false;
  bool drawn = false; // viewProp ran since the last tick()

  ~view_quad() {
    // A renderer may still hold the actor until its mirror re-syncs: it must not reach this view
    // through the callback once it is gone, nor show a picture nothing updates.
    if (mapper && shader_cb)
      mapper->RemoveObserver(shader_cb);
    if (actor)
      actor->VisibilityOff();
  }
};

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
  std::vector<double> clip_key; // the scene clip planes appended to the cut planes, flat
  std::uint64_t view = 0;       // whose raycast: 0 the node's own renderer, else a view's id
};

// The raycast worker: owns the raycaster (and thus its private thread pool)
// and a single render thread with a one-slot latest-wins job queue PER
// RENDERER (the node's own, slot 0, and each view's).  The slots take turns,
// so a view's camera moving never replaces another renderer's job, and every
// job shares the one raycaster: one thread pool rather than one per view, the
// shadow maps it caches (they do not depend on the camera), and the app's
// CUDA device cache.  On single-threaded wasm builds there is no thread —
// run() executes inline.
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
      for (auto &s : slots)
        s.second.has_job = false;
    }
    cv.notify_all();
    if (thread.joinable())
      thread.join();
  }

  // Latest camera wins, per renderer: overwrite that renderer's not-yet-started job.
  void submit(snapshot snap) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      slot &s = slots[snap.view];
      s.job = std::move(snap);
      s.has_job = true;
      s.dropped = false;
    }
    cv.notify_one();
  }

  // A view went: its job not yet started goes; one in flight lands, and the owner drops it.
  void drop(std::uint64_t view) {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = slots.find(view);
    if (it == slots.end())
      return;
    if (it->second.in_flight) {
      it->second.has_job = false;
      it->second.dropped = true;
    } else {
      slots.erase(it);
    }
  }

  // A job of `view`'s waiting or running.
  bool busy(std::uint64_t view) const {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = slots.find(view);
    return it != slots.end() && (it->second.has_job || it->second.in_flight);
  }

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
  mutable std::mutex mutex;
  std::condition_variable cv;
  // One renderer's job queue: its latest job not yet started, and whether one is running.
  struct slot {
    snapshot job;
    bool has_job = false;
    bool in_flight = false;
    bool dropped = false; // its view went while a job of its was running
  };
  std::map<std::uint64_t, slot> slots; // by renderer: 0 the node's own, else a view's id
  std::uint64_t last_served = 0;
  bool stopping = false;
  // Armed by the owner thread (invalidateVolumeData), consumed by whichever
  // thread runs the raycast; never touches the raycaster off-thread.  The
  // raycaster is shared, so one consumption serves every renderer.
  std::atomic<bool> invalidate_pending{false};

  // Set by the node; called from the worker thread with the finished frame.
  std::function<void(cvc::volren::frame, snapshot, double)> on_frame;

  // The next slot to serve, under the lock: the first one after the last served that has a job,
  // wrapping around, so every renderer gets its turn however often another one submits.
  std::map<std::uint64_t, slot>::iterator next_slot() {
    auto from = slots.upper_bound(last_served);
    for (auto it = from; it != slots.end(); ++it)
      if (it->second.has_job)
        return it;
    for (auto it = slots.begin(); it != from; ++it)
      if (it->second.has_job)
        return it;
    return slots.end();
  }

  void loop() {
    for (;;) {
      snapshot snap;
      std::uint64_t view = 0;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return stopping || next_slot() != slots.end(); });
        if (stopping)
          return;
        auto it = next_slot();
        snap = std::move(it->second.job);
        it->second.has_job = false;
        it->second.in_flight = true;
        view = it->first;
        last_served = view;
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
      std::lock_guard<std::mutex> lock(mutex);
      auto it = slots.find(view);
      if (it != slots.end()) {
        it->second.in_flight = false;
        if (it->second.dropped && !it->second.has_job)
          slots.erase(it);
      }
    }
  }
};

VolRenNode::VolRenNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GeometryNode(ctx, statePath, name) {
  m_stateSettings = std::make_unique<cvc::volren::state_settings>(
      ctx, stateName("volren"), [this](const cvc::volren::state_settings::snapshot &s) {
        {
          std::lock_guard<std::mutex> lock(m_configMutex);
          m_snapshotSettings = s;
          ++m_settingsVersion;
        }
        // A volume's model_transform may have moved the box. After the lock:
        // getBoundingBox() takes it.
        ownBoundsChanged();
      });
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_snapshotSettings = m_stateSettings->get();
  }
  seedOwnState();

  m_worker = std::make_unique<worker>(ctx);
  // The callback reaches the worker through a RAW pointer, never through
  // m_worker.  ~VolRenNode calls m_worker.reset(), and reset() stores nullptr
  // into the unique_ptr BEFORE it runs the deleter -- that is, before
  // ~worker() joins the render thread.  A raycast still in flight at that
  // moment therefore returns into this callback with m_worker already null,
  // and `m_worker->used_backend` is a null dereference.  `w` is safe for
  // exactly the same reason the reach-back was not: the callback runs inside
  // worker::loop(), which is what the join is waiting for, so the worker
  // OBJECT outlives every call even though the unique_ptr no longer names it.
  //
  // Reproduced (SIGSEGV, 3/3 runs) on volren_bunny --bunnies 9 --volume-shadows
  // --deep-shadows --shell --soft-shadows 4 --ao 0.7 --rig --continuous, where
  // a 220 ms raycast makes "a frame in flight at teardown" the normal case
  // rather than a narrow race; lighter scenes finish first and hide it.
  worker *const w = m_worker.get();
  m_worker->on_frame = [this, w](cvc::volren::frame f, snapshot snap, double seconds) {
    m_lastRenderSeconds.store(seconds);
    m_backendUsed.store(w->used_backend.load());
    // Marshal to the owner thread; weak-guarded, so a dying node drops it.
    auto shared_f = std::make_shared<cvc::volren::frame>(std::move(f));
    auto shared_s = std::make_shared<snapshot>(std::move(snap));
    runOnMainThread([this, shared_f, shared_s] {
      if (shared_s->view == 0)
        applyFrame(*shared_f, *shared_s);
      else
        applyViewFrame(*shared_f, *shared_s);
      if (SceneGraph *sg = getSceneGraph())
        sg->requestRender();
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
  // The node's own renderer fills its window. Set on every draw of the shared program, as each
  // view sets its own (onViewShader).
  setShaderUniform3f("volrenOrigin", 0.f, 0.f, 0.f);

  // Write depth (occlude the scene) rather than sort as a translucent actor.
  // The raycaster already composited the volume's own translucency into this
  // image; the quad just needs to occupy the depth buffer at the volume's
  // front-surface depth so scene geometry -- notably the translucent world-grid
  // GL_LINES -- composites against the bunny by DEPTH, not draw order (the
  // translucent pass masks depth writes, so a translucent quad silently drew
  // over any line the sort placed behind it). gl_FragDepth is written; alpha-0
  // background texels are discarded by the template's alpha test.
  if (auto *actor = vtkActor::SafeDownCast(getProp()))
    actor->ForceOpaqueOn();
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
  ownBoundsChanged();
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
  ownBoundsChanged();
}

std::size_t VolRenNode::volumeCount() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_volumes.size();
}

void VolRenNode::setVolume(std::size_t index, const cvc::volume &vol) {
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index >= m_volumes.size())
      throw cvc::volren_error("VolRenNode: volume index out of range");
    m_volumes[index] = vol;
    ++m_settingsVersion; // the next tick raycasts the new voxels
  }
  ownBoundsChanged();
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
  ownBoundsChanged(); // the volume's model_transform may have moved the box
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

void VolRenNode::setShadowMode(cvc::volren::shadow_mode m) {
  cvc::volren::shadow_settings sh = shadowConfig();
  sh.mode = m;
  setShadowConfig(sh);
}

cvc::volren::shadow_mode VolRenNode::shadowMode() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows.mode;
}

void VolRenNode::setDeepShadowSlices(int slices) {
  cvc::volren::shadow_settings sh = shadowConfig();
  sh.depth_slices = slices;
  setShadowConfig(sh);
}

int VolRenNode::deepShadowSlices() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows.depth_slices;
}

void VolRenNode::setSoftShadows(float radiusTexels, int taps) {
  cvc::volren::shadow_settings sh = shadowConfig();
  sh.pcf_radius = radiusTexels;
  sh.pcf_taps = taps;
  setShadowConfig(sh);
}

float VolRenNode::softShadowRadius() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows.pcf_radius;
}

int VolRenNode::softShadowTaps() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shadows.pcf_taps;
}

void VolRenNode::setAmbientLevel(float a) {
  cvc::volren::render_settings rs = renderConfig();
  rs.ambient = a;
  setRenderConfig(rs);
}

float VolRenNode::ambientLevel() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.ambient;
}

cvc::volren::hemisphere_ambient VolRenNode::hemisphereConfig() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.ambient_hemisphere;
}

void VolRenNode::setHemisphereConfig(const cvc::volren::hemisphere_ambient &h) {
  cvc::volren::render_settings rs = renderConfig();
  rs.ambient_hemisphere = h;
  setRenderConfig(rs);
}

cvc::volren::ao_settings VolRenNode::occlusionConfig() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.ao;
}

void VolRenNode::setOcclusionConfig(const cvc::volren::ao_settings &ao) {
  cvc::volren::render_settings rs = renderConfig();
  rs.ao = ao;
  setRenderConfig(rs);
}

void VolRenNode::setShadingGain(float g) {
  cvc::volren::render_settings rs = renderConfig();
  rs.shading_gain = g;
  setRenderConfig(rs);
}

float VolRenNode::shadingGain() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.shading_gain;
}

void VolRenNode::setSpecularLevel(float s) {
  cvc::volren::render_settings rs = renderConfig();
  rs.specular = s;
  setRenderConfig(rs);
}

float VolRenNode::specularLevel() const {
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_snapshotSettings.settings.specular;
}

void VolRenNode::applyClipPlanes(vtkPlaneCollection *planes) {
  m_sceneClip = planes; // read into the cut planes at the next tick (buildSnapshot)
  if (SceneGraph *sg = getSceneGraph())
    sg->requestRender();
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

cvc::world_units::coordinate VolRenNode::volumePointToReal(std::size_t index, double ox, double oy,
                                                           double oz,
                                                           const cvc::world_units &units) const {
  cvc::volren::mat4 mt; // defaults to identity
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index >= m_volumes.size())
      throw cvc::index_out_of_bounds("VolRenNode::volumePointToReal: volume index out of range");
    if (index < m_snapshotSettings.volumes.size())
      mt = m_snapshotSettings.volumes[index].model_transform;
  }
  // volume-object -> this node's local frame (the per-volume model transform)...
  const cvc::volren::vec3d pl = mt.transform_point({ox, oy, oz});
  // ...node-local -> world (the parent chain, inherited)...
  const double local[3] = {pl.x, pl.y, pl.z};
  double world[3];
  localToWorld(local, world);
  // ...world -> canonical metres -> the active display unit.
  return units.world_point_to_real(world[0], world[1], world[2]);
}

cvc::world_units::coordinate VolRenNode::volumeRealDimensions(std::size_t index,
                                                              const cvc::world_units &units) const {
  cvc::bounding_box b;
  cvc::volren::mat4 mt; // defaults to identity
  {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index >= m_volumes.size())
      throw cvc::index_out_of_bounds("VolRenNode::volumeRealDimensions: volume index out of range");
    b = m_volumes[index].boundingBox();
    if (index < m_snapshotSettings.volumes.size())
      mt = m_snapshotSettings.volumes[index].model_transform;
  }
  // Fit a world-space AABB around the 8 corners taken through model_transform
  // (object -> node-local) then localToWorld (node-local -> world).
  double mn[3] = {0, 0, 0}, mx[3] = {0, 0, 0};
  for (int c = 0; c < 8; ++c) {
    const cvc::volren::vec3d p = mt.transform_point(
        {c & 1 ? b.maxx : b.minx, c & 2 ? b.maxy : b.miny, c & 4 ? b.maxz : b.minz});
    const double local[3] = {p.x, p.y, p.z};
    double w[3];
    localToWorld(local, w);
    for (int a = 0; a < 3; ++a) {
      if (c == 0 || w[a] < mn[a])
        mn[a] = w[a];
      if (c == 0 || w[a] > mx[a])
        mx[a] = w[a];
    }
  }
  return units.world_point_to_real(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);
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

bool VolRenNode::buildSnapshot(snapshot &out, vtkRenderer *ren, std::uint64_t view) {
  if (!ren)
    return false;
  vtkCamera *cam = ren->GetActiveCamera();
  if (!cam)
    return false;
  out.view = view;
  const int *size = ren->GetSize();
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

  // Scene clip planes (this node's and its ancestors') join the cut planes. Both
  // are world space with the same keep side; read every tick, so a plane moved
  // in place re-raycasts like a moved camera.
  out.clip_key.clear();
  if (m_sceneClip) {
    m_sceneClip->InitTraversal();
    while (vtkPlane *p = m_sceneClip->GetNextItem()) {
      cvc::volren::cut_plane cp;
      p->GetOrigin(cp.point.data());
      p->GetNormal(cp.normal.data());
      out.settings.settings.cut_planes.push_back(cp);
      out.clip_key.insert(out.clip_key.end(), cp.point.begin(), cp.point.end());
      out.clip_key.insert(out.clip_key.end(), cp.normal.begin(), cp.normal.end());
    }
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
  if (const int *sz = m_renderer->GetSize())
    setShaderUniform3f("volrenViewport", float(std::max(1, sz[0])), float(std::max(1, sz[1])), 0.f);
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

// Pose the billboard across the LIVE camera's frustum, at the distance of the
// composed volume centre.  Called every tick (not only when a fresh raycast
// lands) so during camera motion the quad keeps hugging the current view
// instead of showing the last raycast camera's frustum skewed into the new
// one -- the "you can see the quad distort" artifact on threaded builds.
// gl_FragDepth still supplies the real per-pixel depth.
namespace {
// The billboard's corners for `cam` (bottom-left, bottom-right, top-right, top-left; x y z each):
// across its frustum at the distance `center` lies along its view direction, clamped inside
// [near, far].
std::vector<double> quad_corners(const cvc::volren::camera &cam, const cvc::volren::vec3d &center,
                                 double near_z, double far_z) {
  const cvc::volren::view_basis basis = cam.basis();
  const cvc::volren::vec3d eye(cam.eye);
  const cvc::volren::vec3d forward = -basis.back;
  double d = dot(center - eye, forward);
  d = std::min(std::max(d, near_z * 1.01), far_z * 0.99);
  const double half_h = cam.projection == cvc::volren::camera::projection_type::perspective
                            ? d * std::tan(0.5 * cam.vfov_degrees * 3.14159265358979323846 / 180.0)
                            : cam.parallel_scale;
  const double half_w = half_h * cam.aspect();
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
  return xyz;
}

// The (composed) centre of the snapshot's first volume: where the billboard is posed.
template <class Snapshot> cvc::volren::vec3d volume_center(const Snapshot &snap) {
  cvc::volren::vec3d center{0, 0, 0};
  if (!snap.volumes.empty()) {
    const cvc::bounding_box &b = snap.volumes.front().boundingBox();
    cvc::volren::mat4 mt = snap.node_world;
    if (!snap.settings.volumes.empty())
      mt = snap.node_world * snap.settings.volumes.front().model_transform;
    center = mt.transform_point(
        {0.5 * (b.minx + b.maxx), 0.5 * (b.miny + b.maxy), 0.5 * (b.minz + b.maxz)});
  }
  return center;
}
} // namespace

void VolRenNode::poseQuad(const snapshot &snap) {
  ensureQuad();
  updateVertices(quad_corners(snap.cam, volume_center(snap), snap.near_z, snap.far_z));
}

void VolRenNode::applyFrame(const cvc::volren::frame &f, const snapshot &snap) {
  const cvc::image &color = f.color;
  const cvc::image &depth = f.depth;
  const int w = color.width();
  const int h = color.height();
  if (w < 1 || h < 1)
    return;

  // The quad is posed every tick by poseQuad() against the LIVE camera, so it
  // always hugs the current frustum (bug: a billboard posed for the raycast
  // camera and viewed from a moved camera reads as a skewed, visible quad).
  // applyFrame only refreshes the texture the quad shows.
  ensureQuad();

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
      // Colour: upload as an RGBA8 custom texture and sample it ourselves in the
      // shader.  VTK's built-in per-actor texture path is empty under the wasm
      // mapper (//VTK::TCoord::Dec/Impl come back blank), so the built-in colour
      // texture never reaches the fragment shader there -- the quad showed flat
      // material colour and, worse, our depth code referenced the resulting-
      // undeclared tcoordVCVSOutput and failed the whole shader to compile.
      // LINEAR filtering keeps the premultiplied-alpha bilinear read correct.
      if (!m_colorTexObj) {
        m_colorTexObj = vtkSmartPointer<vtkTextureObject>::New();
        m_colorTexObj->SetContext(oglWin);
      }
      m_colorTexObj->Create2DFromRaw(static_cast<unsigned int>(w), static_cast<unsigned int>(h), 4,
                                     VTK_UNSIGNED_CHAR, const_cast<unsigned char *>(color.data()));
      m_colorTexObj->SetWrapS(vtkTextureObject::ClampToEdge);
      m_colorTexObj->SetWrapT(vtkTextureObject::ClampToEdge);
      m_colorTexObj->SetMinificationFilter(vtkTextureObject::Linear);
      m_colorTexObj->SetMagnificationFilter(vtkTextureObject::Linear);
      setShaderTexture("volrenColorTex", m_colorTexObj);
    }
  }

  m_appliedVersion = snap.version;
  m_appliedMatrix = snap.node_world.m;
  m_appliedClip = snap.clip_key;
  m_appliedCamera = snap.camera_key;
  m_appliedW = w;
  m_appliedH = h;
  m_framesRendered.fetch_add(1);
  m_frameAppliedSinceTick = true;
}

// A view's raycast landed: its picture and depth map, as applyFrame's for the node's own quad
// (RGBA8 premultiplied, sampled linear and un-premultiplied in the shader; eye-space depth as R32F,
// sampled nearest), in textures of its own on the view's render window.
void VolRenNode::applyViewFrame(const cvc::volren::frame &f, const snapshot &snap) {
  view_quad *v = nullptr;
  for (auto &q : m_views)
    if (q->id == snap.view)
      v = q.get();
  if (!v || !v->renderer) // the view went while its raycast ran
    return;
  const cvc::image &color = f.color;
  const cvc::image &depth = f.depth;
  const int w = color.width();
  const int h = color.height();
  if (w < 1 || h < 1)
    return;
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(v->renderer->GetRenderWindow());
  if (!win || !win->GetInitialized())
    return;
  const auto upload = [win, w, h](vtkSmartPointer<vtkTextureObject> &tex, int components, int type,
                                  const unsigned char *data, int filter) {
    if (!tex)
      tex = vtkSmartPointer<vtkTextureObject>::New();
    if (tex->GetContext() != win)
      tex->SetContext(win);
    tex->Create2DFromRaw(static_cast<unsigned int>(w), static_cast<unsigned int>(h), components,
                         type, const_cast<unsigned char *>(data));
    tex->SetWrapS(vtkTextureObject::ClampToEdge);
    tex->SetWrapT(vtkTextureObject::ClampToEdge);
    tex->SetMinificationFilter(filter);
    tex->SetMagnificationFilter(filter);
  };
  upload(v->color, 4, VTK_UNSIGNED_CHAR, color.data(), vtkTextureObject::Linear);
  upload(v->depth, 1, VTK_FLOAT, depth.data(), vtkTextureObject::Nearest);

  v->applied_version = snap.version;
  v->applied_matrix = snap.node_world.m;
  v->applied_clip = snap.clip_key;
  v->applied_camera = snap.camera_key;
  v->applied_w = w;
  v->applied_h = h;
  ++v->applied;
  v->has_frame = true;
  // Whether it shows the view's live camera is for the next viewProp to say (the camera may have
  // moved while it ran).
  v->converged = false;
  updateViewsConverged();
}

void VolRenNode::launchOrRun(const snapshot &snap) {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
  // Single-threaded wasm: raycast synchronously at the (reduced) raster.
  double seconds = 0.0;
  try {
    cvc::volren::frame f = m_worker->run(snap, seconds);
    m_lastRenderSeconds.store(seconds);
    if (snap.view == 0)
      applyFrame(f, snap);
    else
      applyViewFrame(f, snap);
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
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

  // The views: those whose renderer went go, with any job of theirs not yet started. A view not
  // drawn since the last tick (left out by a tag, throttled this frame, its widget gone) has
  // nothing on its way to the screen unless a raycast of its is still running.
  pruneViews();
  for (auto &v : m_views) {
    if (!v->drawn)
      v->converged = !m_worker->busy(v->id);
    v->drawn = false;
  }
  updateViewsConverged();

  snapshot snap;
  if (!buildSnapshot(snap, m_renderer, 0))
    return false;

  // Keep the billboard hugging the LIVE camera every frame, decoupled from
  // raycast arrival, so motion never shows a skewed/edge-visible quad.
  poseQuad(snap);

  const bool camera_changed = snap.camera_key != m_appliedCamera || snap.cam.width != m_appliedW ||
                              snap.cam.height != m_appliedH;
  const bool matrix_changed = snap.node_world.m != m_appliedMatrix;
  const bool settings_changed = snap.version != m_appliedVersion;
  const bool clip_changed = snap.clip_key != m_appliedClip;
  const bool stale = camera_changed || matrix_changed || settings_changed || clip_changed;

  // Converged == what is ON SCREEN matches the live camera, transform and
  // settings, and nothing is still cooking.  "No new frame arrived" is NOT the
  // same thing: while a raycast is in flight tick() deliberately does not
  // relaunch, so a slow frame looks exactly like a settled one from the
  // outside.  Anything that waits for a finished image -- an offscreen PNG
  // capture, a screenshot test -- must poll this, or under load it captures
  // whatever stale frame happened to be applied first.
  const bool busy = m_worker->busy(0);
  m_converged.store(!stale && !busy, std::memory_order_relaxed);

  if ((m_continuous || stale) && !busy) {
    launchOrRun(snap);
  }
  return m_frameAppliedSinceTick;
}

// ---- views -------------------------------------------------------------------------------------

vtkActor *VolRenNode::viewProp(vtkRenderer *view) {
  if (!view || !view->GetActiveCamera())
    return nullptr;
  if (view == m_renderer)
    return actor(); // the node's own renderer draws the node's own quad (tick)
  pruneViews();
  view_quad *v = nullptr;
  for (auto &q : m_views)
    if (q->renderer.GetPointer() == view)
      v = q.get();
  if (!v) {
    if (volumeCount() == 0)
      return nullptr;
    auto q = std::make_unique<view_quad>();
    q->node = this;
    q->id = m_nextViewId++;
    q->renderer = view;
    // The quad: four corners posed below on every call, two triangles. No texture coordinates:
    // the shader reads the picture at the fragment's place in the view (kDepthImpl).
    vtkNew<vtkPoints> pts;
    pts->SetNumberOfPoints(4);
    for (vtkIdType i = 0; i < 4; ++i)
      pts->SetPoint(i, 0.0, 0.0, 0.0);
    vtkNew<vtkCellArray> tris;
    const vtkIdType t0[3] = {0, 1, 2}, t1[3] = {0, 2, 3};
    tris->InsertNextCell(3, t0);
    tris->InsertNextCell(3, t1);
    q->poly = vtkSmartPointer<vtkPolyData>::New();
    q->poly->SetPoints(pts);
    q->poly->SetPolys(tris);
    q->mapper = newPolyDataMapper(app());
    q->mapper->SetInputData(q->poly);
    q->mapper->ScalarVisibilityOff();
    q->actor = vtkSmartPointer<VolRenViewActor>::New();
    q->actor->SetMapper(q->mapper);
    vtkActor *own = actor();
    q->actor->SetShaderProperty(own->GetShaderProperty()); // the depth and colour replacements
    q->actor->SetProperty(own->GetProperty());             // unlit white, as the node's
    q->actor->ForceOpaqueOn();                             // writes depth (see the constructor)
    q->actor->SetPickable(false);
    // A picture raycast for the view's camera is no shadow seen from a light: it casts none.
    GraphicsNode::setPropCastsShadow(app(), q->actor, false);
    // Its uniforms and textures, the view's own, set on each of its draws.
    q->shader_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    q->shader_cb->SetClientData(q.get());
    q->shader_cb->SetCallback(&VolRenNode::onViewShader);
    q->mapper->AddObserver(vtkCommand::UpdateShaderEvent, q->shader_cb);
    m_views.push_back(std::move(q));
    v = m_views.back().get();
  }
  v->drawn = true;
  vtkActor *own = actor();
  if (v->actor->GetShaderProperty() != own->GetShaderProperty())
    v->actor->SetShaderProperty(own->GetShaderProperty());
  if (v->actor->GetProperty() != own->GetProperty())
    v->actor->SetProperty(own->GetProperty());
  v->actor->SetVisibility(own->GetVisibility());
  const bool shown = own->GetVisibility() && view->GetDraw();

  snapshot snap;
  if (!buildSnapshot(snap, view, v->id)) {
    // Nothing to raycast (no volume, a view too small to draw): nothing to wait for either.
    v->converged = !m_worker->busy(v->id);
    updateViewsConverged();
    return v->has_frame ? v->actor.GetPointer() : nullptr;
  }

  // The quad hugs the view's live camera on every draw, as the node's own does (poseQuad), and its
  // bounds are the volumes' box (VolRenViewActor).
  {
    const std::vector<double> xyz =
        quad_corners(snap.cam, volume_center(snap), snap.near_z, snap.far_z);
    vtkPoints *pts = v->poly->GetPoints();
    for (vtkIdType i = 0; i < 4; ++i)
      pts->SetPoint(i, xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    pts->Modified();
    v->poly->Modified();
    const cvc::bounding_box bb = getWorldBoundingBox();
    const double b[6] = {bb.minx, bb.maxx, bb.miny, bb.maxy, bb.minz, bb.maxz};
    if (!std::equal(b, b + 6, v->actor->VolumeBounds)) {
      std::copy(b, b + 6, v->actor->VolumeBounds);
      v->actor->Modified();
    }
  }

  // What it shows is what this snapshot would raycast: the same camera and raster, transform,
  // clip planes and settings.
  const auto on_screen = [&] {
    return v->has_frame && snap.camera_key == v->applied_camera && snap.cam.width == v->applied_w &&
           snap.cam.height == v->applied_h && snap.node_world.m == v->applied_matrix &&
           snap.version == v->applied_version && snap.clip_key == v->applied_clip;
  };
  bool busy = m_worker->busy(v->id);
  if (shown && (m_continuous || !on_screen()) && !busy) {
    ++v->launched;
    launchOrRun(snap); // on single-threaded wasm, applied by the time it returns
    busy = m_worker->busy(v->id);
  }
  // A hidden view shows nothing, so only a raycast still running keeps it from converging.
  v->converged = shown ? on_screen() && !busy : !busy;
  updateViewsConverged();
  return v->has_frame ? v->actor.GetPointer() : nullptr;
}

void VolRenNode::releaseViewProp(vtkRenderer *view) {
  for (auto it = m_views.begin(); it != m_views.end();) {
    if ((*it)->renderer && (*it)->renderer.GetPointer() != view) {
      ++it;
      continue;
    }
    m_worker->drop((*it)->id);
    it = m_views.erase(it);
  }
  updateViewsConverged();
}

void VolRenNode::pruneViews() {
  for (auto it = m_views.begin(); it != m_views.end();) {
    if ((*it)->renderer) {
      ++it;
      continue;
    }
    m_worker->drop((*it)->id);
    it = m_views.erase(it);
  }
}

void VolRenNode::updateViewsConverged() {
  bool all = true;
  for (const auto &v : m_views)
    all = all && v->converged;
  m_viewsConverged.store(all, std::memory_order_relaxed);
}

std::size_t VolRenNode::viewCount() const { return m_views.size(); }

bool VolRenNode::viewStats(vtkRenderer *view, view_stats &out) const {
  for (const auto &v : m_views)
    if (v->renderer && v->renderer.GetPointer() == view) {
      out.launched = v->launched;
      out.applied = v->applied;
      out.width = v->applied_w;
      out.height = v->applied_h;
      out.converged = v->converged;
      return true;
    }
  return false;
}

// A view's quad is drawing (its mapper's UpdateShaderEvent): its uniforms and textures, which the
// program it shares with every other renderer drawing this node needs set on each draw. The depth
// conversion takes the view camera's near and far as they are for this draw, and the picture is
// read relative to the draw's viewport in the framebuffer it draws into (vtkOpenGLState's): the
// view's rect in the window, or wherever else its renderer draws.
void VolRenNode::onViewShader(vtkObject *caller, unsigned long, void *clientData, void *callData) {
  auto *v = static_cast<view_quad *>(clientData);
  auto *program = static_cast<vtkShaderProgram *>(callData);
  if (!v || !v->node || !program || !v->renderer)
    return;
  vtkRenderer *ren = v->renderer;
  vtkCamera *cam = ren->GetActiveCamera();
  if (!cam)
    return;
  double clip[2];
  cam->GetClippingRange(clip);
  int vp[4] = {0, 0, 1, 1};
  if (auto *win = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
      win && win->GetState())
    win->GetState()->vtkglGetIntegerv(GL_VIEWPORT, vp);
  else
    ren->GetTiledSizeAndOrigin(&vp[2], &vp[3], &vp[0], &vp[1]);
  const auto setf = [program](const char *name, float x) {
    if (program->IsUniformUsed(name))
      program->SetUniformf(name, x);
  };
  const auto set3f = [program](const char *name, float x, float y) {
    const float xyz[3] = {x, y, 0.f};
    if (program->IsUniformUsed(name))
      program->SetUniform3f(name, xyz);
  };
  setf("volrenNear", static_cast<float>(clip[0]));
  setf("volrenFar", static_cast<float>(clip[1]));
  if (program->IsUniformUsed("volrenPersp"))
    program->SetUniformi("volrenPersp", cam->GetParallelProjection() ? 0 : 1);
  set3f("volrenViewport", static_cast<float>(std::max(1, vp[2])),
        static_cast<float>(std::max(1, vp[3])));
  set3f("volrenOrigin", static_cast<float>(vp[0]), static_cast<float>(vp[1]));
  if (v->color && v->depth)
    v->node->bindShaderTextures(program,
                                {{"volrenDepthTex", v->depth}, {"volrenColorTex", v->color}},
                                caller, &v->actor->Bound);
}

} // namespace gl
} // namespace cvc

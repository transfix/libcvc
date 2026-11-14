// AriRuntime — see inc/cvc/gl/ariadne/AriRuntime.h. The thin cvcGL composer over the
// backend-neutral cvc::ariadne::AppRuntime: an ImGui-over-VTK backend, a GL SceneAdapter
// (realize_scene / tick_scene over the SceneRenderer's SceneGraph), and the AppRuntime
// that owns the neutral bundling. Only the GL-driving frame bits (input pump, camera, the
// draw) live here.

#include <algorithm>
#include <chrono>
#include <cvc/ariadne/ariadne.h>                // Runtime::document_scope()
#include <cvc/ariadne/bind.h>                   // sync_scene_visibility
#include <cvc/ariadne/stream/stream_channel.h>  // subscribe / deliver_mode
#include <cvc/ariadne/stream/stream_registry.h> // stream_registry::for_app / lookup
#include <cvc/ariadne/stream_intrinsics.h>      // register_stream_intrinsics: streams by default
#include <cvc/core/app.h>                       // cvc::app
#include <cvc/gl/CameraController.h>            // cam.update / frameBounds
#include <cvc/gl/GeometryNode.h>                // GeometryNode (gl-bind-stream target)
#include <cvc/gl/ImGuiOverlay.h>                // overlay draw callback
#include <cvc/gl/SceneGraph.h>      // scene().computeGraphicsBounds()/appContext()/prefix
#include <cvc/gl/SceneRenderer.h>   // view.render()/processUIEvents()/renderer()/windowClosed()
#include <cvc/gl/Viewport.h>        // a scenes: entry's own viewport
#include <cvc/gl/ViewportManager.h> // ... added beside the main one
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/bundles.h>                // register_streaming / register_window_widgets
#include <cvc/gl/ariadne/extensions.h>             // register_cvcgl_extensions
#include <cvc/gl/ariadne/scene_realize.h>          // realize_scene / RealizedScene / tick_scene
#include <cvc/gl/ariadne/scene_views.h>            // the viewport widget
#include <cvc/gl/ariadne/stream_node.h>            // register_stream_node_type: type: stream
#include <cvc/gl/ariadne/stream_texture_binding.h> // StreamTextureBinding (gl-bind-stream sink)
#include <cvc/gl/ariadne/stream_verbs.h>     // StreamBindingSink / register_gl_stream_intrinsics
#include <cvc/state/state_exec/async_host.h> // wait_idle
#include <cvc/state/state_exec/intrinsics.h> // document_scope::slot
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <vtkRenderWindow.h> // (fullscreen ON): the window covers the screen
#include <vtkRenderer.h>     // the scene background: a gradient reaches the renderer directly
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#ifdef CVC_ENABLE_SDL
#include "../SdlWindow.h" // the canvas cvcGL's SDL window draws into
#endif

// clang-format off: JavaScript bodies
// The canvas covers the screen (requestFullscreen), or the page leaves fullscreen: 0 when the
// browser has no such call. A browser grants it only within a user's activation (a click a frame
// ago has it), and says no by rejecting the promise, which is_fullscreen then shows.
EM_JS(int, cvcgl_canvas_fullscreen, (const char *selector, int on), {
  var c = document.querySelector(UTF8ToString(selector));
  try {
    if (on) {
      var req = c && (c.requestFullscreen || c.webkitRequestFullscreen);
      if (!req)
        return 0;
      var p = req.call(c);
      if (p && p.catch)
        p.catch(function() {});
    } else if (document.fullscreenElement || document.webkitFullscreenElement) {
      (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    }
    return 1;
  } catch (e) {
    return 0;
  }
});
EM_JS(int, cvcgl_canvas_is_fullscreen, (), {
  return document.fullscreenElement || document.webkitFullscreenElement ? 1 : 0;
});
// clang-format on
#endif

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// The GL implementation of the neutral SceneAdapter seam: realize a parsed Ariadne Scene
// into the SceneRenderer's SceneGraph, frame the camera to it, and service it per frame.
// Everything GL/VTK about running an .ari app is confined to this class.
// Also a StreamBindingSink (stream_verbs.h): the (gl-bind-stream) verb, resolved to THIS document
// via its document_scope, calls bind_stream() here on the render thread, and this adapter owns +
// ticks the resulting StreamTextureBinding alongside the declarative ones.
// A scene's background (a view property) on renderer r: a solid colour, or a top-to-bottom
// gradient. False when the scene sets none.
bool apply_background(vtkRenderer *r, const ari::Scene &scene) {
  if (!scene.has_background || !r)
    return false;
  const auto &t = scene.background_top;
  const auto &b = scene.background_bottom;
  r->SetGradientBackground(scene.background_gradient);
  if (scene.background_gradient) {
    r->SetBackground(b[0], b[1], b[2]);  // bottom
    r->SetBackground2(t[0], t[1], t[2]); // top
  } else {
    r->SetBackground(t[0], t[1], t[2]);
  }
  return true;
}

class GlSceneAdapter : public ari::SceneAdapter, public StreamBindingSink {
public:
  GlSceneAdapter(SceneRenderer &view, CameraController &cam) : view_(&view), cam_(&cam) {}
  ~GlSceneAdapter() override { clear_scenes(); }

  // A `scenes:` entry: a SceneGraph of its own (its nodes, lights, shadows, background) attached
  // to a viewport of its own, `anchor`, which the views of it mirror. The anchor is not shown; the
  // views draw a pixel of it under one of them when its shadows need baking (scene_views.cpp).
  struct OtherScene {
    std::string id, anchor;
    std::unique_ptr<SceneGraph> sg;
    RealizedScene realized;
  };
  const OtherScene *other_scene(const std::string &id) const {
    for (const auto &s : scenes_)
      if (s->id == id)
        return s.get();
    return nullptr;
  }

  // Required scene-node customs (cvcGL's registry) before realizing; a missing one skips the scene.
  bool verify(const ari::LoadResult &loaded, std::vector<std::string> *errors) override {
    return verify_scene_customs(view_->scene().appContext(), loaded, errors);
  }

  // A VTK renderer: a node's shader: block compiles here (gl.scene.shaders).
  bool runs_shaders() const override { return true; }

  void realize(const ari::Scene &scene, const std::string &prefix,
               std::vector<std::string> *warns) override {
    // A reload rebuilds every node, so drop the imperative (gl-bind-stream) bindings now (render
    // thread) rather than letting them tick a stale node for a frame. They are runtime-only state
    // tied to the previous node graph, not part of the document, so they do NOT survive a reload;
    // re-run the binding action after the reload to re-establish them.
    verb_bindings_.clear();
    SceneGraph &sg = view_->scene();
    save_view(sg);
    RealizeOptions options;
    options.assets = &assets_; // each asset read once, for this scene and the document's scenes:
    realized_ = realize_scene(sg, scene, prefix, warns, options);
    haveScene_ = true;
    for (const ari::SceneLight &l : scene.lights)
      if (l.rig.empty()) // a rig's lights belong to (and leave with) its StageLighting
        lights_.push_back(l.id);
    const cvc::bounding_box bb = view_->scene().computeGraphicsBounds();
    if (!bb.isNull())
      cam_->frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
    // The scene's background (a view property): a solid colour, or a top-to-bottom gradient set on
    // the renderer (SceneRenderer exposes only a flat setBackground).
    apply_background(view_->renderer(), scene);
  }

  // The document's other scenes: each realized into a SceneGraph of its own, under state
  // <prefix>.scenes.<id> (its nodes at <prefix>.scenes.<id>.graphics...), while its widgets and its
  // nodes' binds share the document's prefix. Its own viewport carries its lights, shadows and
  // background; viewports that name the scene mirror it.
  void realize_scenes(const std::vector<ari::Scene> &scenes, const std::string &prefix,
                      std::vector<std::string> *warns) override {
    ViewportManager &vm = view_->viewportManager();
    for (const ari::Scene &scene : scenes) {
      auto s = std::make_unique<OtherScene>();
      s->id = scene.id;
      s->anchor = "scene." + scene.id;
      if (other_scene(scene.id) || vm.hasViewport(s->anchor)) {
        if (warns)
          warns->push_back("ari: scenes: '" + scene.id + "' is declared twice; the first is used");
        continue;
      }
      s->sg =
          std::make_unique<SceneGraph>(view_->scene().appContext(), prefix + ".scenes." + s->id);
      s->sg->setDiagnosticChromeVisible(false); // its grid and axes only when it asks (chrome:)
      const double none[4] = {0.0, 0.0, 0.0, 0.0};
      Viewport &vp = vm.addSceneViewport(s->anchor, *s->sg, none, 1);
      vp.setVisible(false);
      vp.setInputEnabled(false);
      // The main view's background unless the scene sets its own.
      if (!apply_background(vp.renderer(), scene))
        if (vtkRenderer *main = view_->renderer()) {
          double bg[3], bg2[3];
          main->GetBackground(bg);
          main->GetBackground2(bg2);
          vp.renderer()->SetBackground(bg);
          vp.renderer()->SetBackground2(bg2);
          vp.renderer()->SetGradientBackground(main->GetGradientBackground());
        }
      RealizeOptions options;
      options.assets = &assets_;
      options.physics_key = "scenes." + scene.id + ".physics"; // beside its nodes' state
      s->realized = realize_scene(*s->sg, scene, prefix, warns, options);
      scenes_.push_back(std::move(s));
    }
  }

  // The last document's scene comes down as room_view::clear_scene takes a room's down: its nodes
  // and free lights, then the RealizedScene (a rig's own lights leave with its StageLighting, the
  // physics world with its bodies). The view goes back to how the host set it up.
  void clear() override {
    verb_bindings_.clear();
    clear_scenes();
    assets_ = SceneAssetCache{};
    if (chromeHidden_) { // no_scene() hid the host's chrome: the next document starts with it again
      view_->scene().setDiagnosticChromeVisible(chrome_);
      chromeHidden_ = false;
    }
    if (!haveScene_)
      return;
    SceneGraph &sg = view_->scene();
    for (const std::string &id : realized_.created)
      sg.removeGraphics(id);
    for (const std::string &id : lights_)
      sg.removeGraphics(id);
    lights_.clear();
    realized_ = RealizedScene{};
    haveScene_ = false;
    if (vtkRenderer *r = view_->renderer())
      r->SetGradientBackground(false);
    view_->setBackground(background_[0], background_[1], background_[2]);
    if (sg.shadowsEnabled() != shadows_)
      sg.setShadowsEnabled(shadows_);
    sg.setDiagnosticChromeVisible(chrome_);
  }

  // A document without a scene (an app's forms) is its widgets alone: no grid, axes or bounds box
  // behind them. A later document's scene sets its own (scene.chrome, on by default); Settings can
  // still turn them on.
  void no_scene() override {
    SceneGraph &sg = view_->scene();
    save_view(sg);
    sg.setDiagnosticChromeVisible(false);
    chromeHidden_ = true;
  }

  const RealizedScene *realized() const { return haveScene_ ? &realized_ : nullptr; }
  void set_tick_dt(double dt) { tick_dt_ = dt; } // < 0: the scene clock measures wall time

  void sync_visibility(cvc::app &app) override {
    if (haveScene_)
      ari::sync_scene_visibility(app, realized_.visibility); // bound visible: -> node .visible
    for (auto &s : scenes_)
      ari::sync_scene_visibility(app, s->realized.visibility);
  }

  bool restart(std::vector<std::string> *warns) override {
    return haveScene_ && restart_scene_world(realized_, warns);
  }

  void tick() override {
    if (haveScene_) // declarative stream / volren / volslice service, and the scene clock
      tick_scene(realized_, view_->renderer(), tick_dt_);
    if (!scenes_.empty() && !view_->isClosed()) {
      ViewportManager &vm = view_->viewportManager();
      for (auto &s : scenes_)
        if (vm.hasViewport(s->anchor))
          tick_scene(s->realized, vm.viewport(s->anchor).renderer(), tick_dt_);
    }
    // Service the imperative (gl-bind-stream) bindings here, on the render thread, right after the
    // declarative ones. They are NOT in realized_.custom_ticks (that vector is rebuilt wholesale on
    // every realize()); this adapter owns them so they survive across frames and are torn down with
    // the adapter. Reap a binding whose node has gone away: a vanished node expires the binding's
    // weak_ptr, its next tick() unsubscribes, subscribed() turns false, and we erase it here (its
    // dtor runs on this render thread) — so repeated binds / node churn can't grow the map forever.
    for (auto it = verb_bindings_.begin(); it != verb_bindings_.end();) {
      it->second->tick();
      if (!it->second->subscribed())
        it = verb_bindings_.erase(it);
      else
        ++it;
    }
  }

  // StreamBindingSink — bind TOKEN's open stream onto the GeometryNode NODE-ID. Render thread
  // (called inline from the action lane). "" on success, else a "gl-bind-stream: ..." diagnostic.
  std::string bind_stream(const std::string &node_id, const std::string &token) override {
    auto node =
        std::dynamic_pointer_cast<cvc::gl::GeometryNode>(view_->scene().getGraphics(node_id));
    if (!node)
      return "gl-bind-stream: no GeometryNode '" + node_id +
             "' in the scene (NODE-ID must name a top-level geometry node)";
    // Resolve + subscribe the NEW token BEFORE touching any existing binding on this node, so a
    // failed rebind (unknown token, or pool full) leaves a working binding intact rather than
    // dropping it. The replace below only happens once the new subscription is in hand.
    cvc::app &app = view_->scene().appContext();
    cvc::ariadne::stream::stream_channel *ch =
        cvc::ariadne::stream::stream_registry::for_app(app).lookup(token);
    if (!ch)
      return "gl-bind-stream: no live stream for token '" + token + "' (open it first)";
    std::shared_ptr<cvc::ariadne::stream::subscription> sub =
        ch->subscribe(cvc::ariadne::stream::deliver_mode::latest);
    if (!sub)
      return "gl-bind-stream: stream '" + token +
             "' cannot admit another subscriber (frame pool full)";
    node->setUseSingleColor(false); // show the streamed texture, not a flat material colour
    // Install the new binding, replacing any prior one on this node. operator[] destroys the old
    // binding (unsubscribing it) only now that the new one holds its subscription, so the swap is
    // non-destructive on failure. A same-token rebind briefly holds two subscribers across this
    // line, so it needs pool room (expected_subscribers >= 2); on a pool sized to one the subscribe
    // above fails cleanly and the existing binding keeps running (rebinding a node to the stream it
    // already shows is a no-op anyway).
    verb_bindings_[node_id] = std::make_shared<StreamTextureBinding>(
        app, token, std::weak_ptr<cvc::gl::GeometryNode>(node), std::move(sub));
    return {}; // ok — first tick() applies a frame this same frame
  }

private:
  // The document's other scenes come down: their viewports (the views of them are gone first:
  // AriRuntime::load clears its SceneViews before the load), then their graphs.
  void clear_scenes() {
    if (!view_->isClosed()) {
      ViewportManager &vm = view_->viewportManager();
      for (auto &s : scenes_)
        if (vm.hasViewport(s->anchor))
          vm.removeViewport(s->anchor);
    }
    scenes_.clear();
  }

  // The view as the host set it up, once, for clear() to go back to.
  void save_view(SceneGraph &sg) {
    if (viewSaved_)
      return;
    if (vtkRenderer *r = view_->renderer())
      r->GetBackground(background_);
    shadows_ = sg.shadowsEnabled();
    chrome_ = sg.diagnosticChromeVisible();
    viewSaved_ = true;
  }

  SceneRenderer *view_;
  CameraController *cam_;
  RealizedScene
      realized_; // owns StageLighting rigs; outlives the render loop (adapter outlives use)
  bool haveScene_ = false;
  bool chromeHidden_ = false; // no_scene() hid the chrome; clear() puts the host's back
  double tick_dt_ = -1.0;
  std::vector<std::string> lights_; // the realized free (non-rig) lights, which clear() removes
  SceneAssetCache assets_;          // the document's assets, read once for all its scenes
  std::vector<std::unique_ptr<OtherScene>> scenes_; // the document's scenes: entries
  // The view before the first realize(): what clear() restores.
  bool viewSaved_ = false;
  double background_[3] = {0.0, 0.0, 0.0};
  bool shadows_ = false, chrome_ = true;
  // (gl-bind-stream) bindings, keyed by node id (one per node; a rebind replaces). Owned here — a
  // per-document instance member on the scene seam, NOT realized_.custom_ticks (rebuilt on reload)
  // and NOT a process-static table. Destroyed with the adapter on the render thread.
  std::unordered_map<std::string, std::shared_ptr<StreamTextureBinding>> verb_bindings_;
};

} // namespace

// ── construction / teardown ─────────────────────────────────────────────────────────────────────

AriRuntime::AriRuntime(SceneRenderer &view, CameraController &cam, ImGuiOverlay &ui)
    : view_(&view), cam_(&cam), overlay_(&ui), app_(view.scene().appContext()),
      scene_(std::make_unique<GlSceneAdapter>(view, cam)),
      app_rt_(app_, view.scene().getStatePrefix(), scene_.get()) {
  // set_backend before install (install captures the Runtime and wires the overlay draw callback to
  // rt.render(), so the tree walk is driven by VTK's render pass — never by the host). backend_ is
  // declared before app_rt_, so the Runtime it points at is destroyed first.
  app_rt_.set_backend(backend_);
  backend_.install(app_rt_.runtime(), *overlay_);
  // (fullscreen ON) and the §5.13 fullscreen view: the window covers the screen. Natively the VTK
  // window (cvcGL's SDL window: the desktop's mode, borderless); in a browser the canvas, through
  // requestFullscreen, which the page grants within the click's user activation.
  backend_.set_fullscreen_handler(
      [this](bool on) {
        vtkRenderWindow *w = view_->renderWindow();
#ifdef __EMSCRIPTEN__
        const char *canvas = "#canvas";
#ifdef CVC_ENABLE_SDL
        if (const SdlGLRenderWindow *sdl = SdlGLRenderWindow::SafeDownCast(w))
          canvas = sdl->GetCanvasSelector();
#endif
        return cvcgl_canvas_fullscreen(canvas, on ? 1 : 0) != 0;
#else
        if (!w || w->GetOffScreenRendering()) // a snapshot keeps its size
          return false;
        w->SetFullScreen(on ? 1 : 0);
        return true;
#endif
      },
      [this] {
#ifdef __EMSCRIPTEN__
        return cvcgl_canvas_is_fullscreen() != 0;
#else
        vtkRenderWindow *w = view_->renderWindow();
        return w && w->GetFullScreen() != 0;
#endif
      });

  // `type: viewport`: a view of the scene in a window. The draw callback hides, once the walk is
  // done, any view whose widget was not drawn this frame.
  // A viewport's scene: the document's scene: ("") in the main view, or a scenes: entry in its own.
  auto scenes = [this](const std::string &id, SceneSlot &out) {
    auto *adapter = static_cast<GlSceneAdapter *>(scene_.get());
    if (id.empty()) {
      out.sg = &view_->scene();
      out.realized = adapter->realized();
      out.source = view_->viewportManager().primary().name();
      out.anchor = false;
      return true;
    }
    const GlSceneAdapter::OtherScene *s = adapter->other_scene(id);
    if (!s)
      return false;
    out.sg = s->sg.get();
    out.realized = &s->realized;
    out.source = s->anchor;
    out.anchor = true;
    return true;
  };
  views_ = std::make_unique<SceneViews>(view, backend_, scenes, &cam);
  views_->register_widget();
  {
    cvc::ariadne::Runtime *r = &app_rt_.runtime();
    SceneViews *v = views_.get();
    overlay_->setDrawCallback([r, v] {
      r->render();
      v->end_walk();
    });
  }

  // cvcGL's custom scene node types and shader presets (forest_trees, mesh_lab, ...), so a
  // Python-hosted document realizes the same node types ariadne_hello does. Idempotent.
  register_cvcgl_extensions(app_);

  // (gl-bind-stream): register the app's verb, and publish THIS document's GL sink (the adapter)
  // into its Runtime's document_scope so the verb routes here via ictx.document — no capture in
  // the provider, correct per-document routing. Teardown order makes this safe: member
  // order (scene_ before app_rt_) means ~AriRuntime destroys app_rt_ (hence the Runtime + its
  // document_scope, dropping this handle, and ~Runtime closes the document's streams) BEFORE
  // scene_, so the handle never outlives the adapter and each verb binding's dtor unsubscribes
  // against an already-closed (null-lookup) channel. Thread contract: this teardown — like the
  // whole scene, the declarative bindings, and StreamTextureBinding in general — must run on the
  // render thread; the host destroys the AriRuntime there (the shared single-thread contract, not
  // new to this verb). Must run before the first load()/drain().
  register_gl_stream_intrinsics(app_);
  // Streams and rasters by default, on every host built on this (arigl, arigl-web, ariglx and
  // pycvc_gl's Host): (stream-open ...) and (raster-open ...), and `type: stream` nodes. A node's
  // material: { texture: { stream } } needs nothing registered. Idempotent.
  cvc::ariadne::register_stream_intrinsics(app_);
  register_stream_node_type(app_);
  if (auto *sink = dynamic_cast<StreamBindingSink *>(scene_.get()))
    app_rt_.runtime().document_scope().slot<GlStreamSinkHandle>(kGlStreamSinkSlot)->sink = sink;
}

AriRuntime::~AriRuntime() {
  // Drop the overlay's draw callback (which captured the Runtime) BEFORE any member is destroyed,
  // so a stray render during Python GC can't call rt.render() on a half-torn-down Runtime.
  if (overlay_)
    overlay_->setDrawCallback([] {});
  views_->clear(); // the viewports, while the view they are in is still the host's
}

// ── setup / document (forward to AppRuntime) ─────────────────────────────────────────────────────

void AriRuntime::add_component_path(const std::string &dir) { app_rt_.add_component_path(dir); }

std::string AriRuntime::enable_net(const cvc::ariadne::NetHostConfig &config) {
  return app_rt_.enable_net(config);
}

void AriRuntime::on(const std::string &event, std::function<void()> handler) {
  app_rt_.on(event, std::move(handler));
}

void AriRuntime::register_verb(const std::string &name, std::function<void()> fn) {
  app_rt_.register_verb(name, std::move(fn));
}

void AriRuntime::register_async_verb(const std::string &name, std::function<std::string()> work) {
  app_rt_.register_async_verb(name, std::move(work));
}

void AriRuntime::register_async_verb(const std::string &name, const std::string &done_channel,
                                     std::function<std::string()> work) {
  app_rt_.register_async_verb(name, done_channel, std::move(work));
}

void AriRuntime::register_function(const std::string &name, cvc::state_exec::native_fn fn) {
  app_rt_.register_function(name, std::move(fn));
}

void AriRuntime::register_async_function(const std::string &name,
                                         cvc::state_exec::async_marshal marshal,
                                         const std::string &done_channel) {
  app_rt_.register_async_function(name, std::move(marshal), done_channel);
}

void AriRuntime::register_quit() { app_rt_.register_quit(); }
int AriRuntime::exit_code() const { return app_rt_.exit_code(); }

void AriRuntime::enable_streaming() {
  register_streaming(app_);
  register_window_widgets(backend_, app_);
}

std::vector<std::string> AriRuntime::load(const std::string &path) {
  views_->clear(); // the last document's views; the new one's widgets make their own
  std::vector<std::string> warnings = app_rt_.load(path);
  // §5.14 on a phone the panels start hidden so a scene isn't buried; a document with no scene
  // (a feed, a form) is nothing but its panels, so it starts with them shown.
  overlay_->setPhoneHidesPanels(realized() != nullptr);
  return warnings;
}

void AriRuntime::set_root(ari::Widget root) { app_rt_.set_root(std::move(root)); }

std::string AriRuntime::prefix() const { return view_->scene().getStatePrefix(); }

void AriRuntime::run_program(const std::string &script) { app_rt_.run_program(script); }

void AriRuntime::run_program(const std::string &script,
                             std::function<void(const std::string &)> done) {
  app_rt_.run_program(script, std::move(done));
}

void AriRuntime::publish_image(const std::string &name, const cvc::image &img) {
  backend_.set_image(name, img);
}

void AriRuntime::drop_image(const std::string &name) { backend_.drop_image(name); }

bool AriRuntime::has_function(const std::string &name) const {
  return const_cast<ari::AppRuntime &>(app_rt_).runtime().has_function(name);
}

// ── per-frame (GL driving around the neutral drain) ──────────────────────────────────────────────

namespace {
// frame() and render() each mark the runtime busy for their call. A host handler, verb or function
// runs inside one of them, so one that calls frame(), render(), step() or run() would re-enter the
// drain or the draw mid-walk; it gets a std::logic_error instead.
class FrameGuard {
public:
  FrameGuard(bool &busy, const char *what) : busy_(busy) {
    if (busy_)
      throw std::logic_error(std::string("AriRuntime::") + what +
                             ": called from inside a frame (a host handler, verb or function)");
    busy_ = true;
  }
  ~FrameGuard() { busy_ = false; }
  FrameGuard(const FrameGuard &) = delete;
  FrameGuard &operator=(const FrameGuard &) = delete;

private:
  bool &busy_;
};
} // namespace

double AriRuntime::frameDt(double dt_seconds) {
  const double now =
      std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  const double last = lastFrameAt_;
  lastFrameAt_ = now;
  if (fixedClock_ || last < 0.0)
    return dt_seconds;
  return std::clamp(now - last, 0.0, 0.25);
}

void AriRuntime::frame(double dt_seconds) {
  FrameGuard guard(inFrame_, "frame");
  const double moveDt = frameDt(dt_seconds); // the camera's and the views' time step
  view_->processUIEvents(); // VTK interactor -> ImGui (widget input + camera nav)
  if (inputPump_)
    inputPump_(); // the host's own input, delivered this frame
  cam_->update(moveDt);
  static_cast<GlSceneAdapter *>(scene_.get())->set_tick_dt(fixedClock_ ? dt_seconds : -1.0);
  if (fixedClock_) { // the UI's frame clock (a spinner) advances by dt too
    app_rt_.runtime().set_frame_clock(fixedSeconds_);
    fixedSeconds_ += dt_seconds;
  }
  app_rt_.drain(); // queued Ariadne actions + scheduler slice + (via the adapter) scene sync/tick
  views_->update(moveDt); // what each viewport widget's view leaves out, and its camera
}

void AriRuntime::render() {
  FrameGuard guard(inFrame_, "render");
  view_->render(); // fires rt.render() via the overlay draw callback
}

void AriRuntime::step(double dt_seconds) {
  frame(dt_seconds);
  render();
}

bool AriRuntime::should_close() const { return app_rt_.should_close() || view_->windowClosed(); }

void AriRuntime::request_close() { app_rt_.request_close(); }

void AriRuntime::wait_idle() {
  if (inFrame_)
    throw std::logic_error("AriRuntime::wait_idle: called from inside a frame");
  app_.exec_async_host().wait_idle();
}

void AriRuntime::run(double fps) {
  if (inFrame_)
    throw std::logic_error("AriRuntime::run: called from inside a frame (a host handler, verb or "
                           "function)");
  const double dt = fps > 0.0 ? 1.0 / fps : 1.0 / 120.0;
  while (!should_close())
    step(dt);
}

// ── input seam / diagnostics (forward to AppRuntime) ─────────────────────────────────────────────

void AriRuntime::post_key(bool down, const std::string &key, int mods, bool repeat) {
  app_rt_.post_key(down, key, mods, repeat);
}

void AriRuntime::post_pointer(int kind, double x, double y, double dx, double dy, int button,
                              int clicks) {
  app_rt_.post_pointer(kind, x, y, dx, dy, button, clicks);
}

void AriRuntime::post_input(const ari::InputEvent &e) { app_rt_.runtime().post_input(e); }

bool AriRuntime::post_signal(const std::string &signal, const std::string &source) {
  return app_rt_.post_signal(signal, source);
}

void AriRuntime::set_input_pump(std::function<void()> pump) { inputPump_ = std::move(pump); }
void AriRuntime::set_fixed_clock(bool fixed) { fixedClock_ = fixed; }

ari::AppRuntime &AriRuntime::app_runtime() { return app_rt_; }
cvc::gl::ImGuiBackend &AriRuntime::backend() { return backend_; }
SceneViews &AriRuntime::views() { return *views_; }

const RealizedScene *AriRuntime::realized() const {
  return static_cast<const GlSceneAdapter *>(scene_.get())->realized();
}

std::vector<std::string> AriRuntime::take_warnings() { return app_rt_.take_warnings(); }
bool AriRuntime::reload_if_changed() { return app_rt_.reload_if_changed(); }
bool AriRuntime::have_yaml() { return ari::AppRuntime::have_yaml(); }

} // namespace ariadne
} // namespace gl
} // namespace cvc

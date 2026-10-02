// AriRuntime — see inc/cvc/gl/ariadne/AriRuntime.h. The thin cvcGL composer over the
// backend-neutral cvc::ariadne::AppRuntime: an ImGui-over-VTK backend, a GL SceneAdapter
// (realize_scene / tick_scene over the SceneRenderer's SceneGraph), and the AppRuntime
// that owns the neutral bundling. Only the GL-driving frame bits (input pump, camera, the
// draw) live here.

#include <cvc/ariadne/ariadne.h>                // Runtime::document_scope()
#include <cvc/ariadne/bind.h>                   // sync_scene_visibility
#include <cvc/ariadne/stream/stream_channel.h>  // subscribe / deliver_mode
#include <cvc/ariadne/stream/stream_registry.h> // stream_registry::for_app / lookup
#include <cvc/core/app.h>                       // cvc::app
#include <cvc/core/state_exec/intrinsics.h>     // document_scope::slot
#include <cvc/gl/CameraController.h>            // cam.update / frameBounds
#include <cvc/gl/GeometryNode.h>                // GeometryNode (gl-bind-stream target)
#include <cvc/gl/ImGuiOverlay.h>                // overlay draw callback
#include <cvc/gl/SceneGraph.h>    // scene().computeGraphicsBounds()/appContext()/prefix
#include <cvc/gl/SceneRenderer.h> // view.render()/processUIEvents()/renderer()/windowClosed()
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h>          // realize_scene / RealizedScene / tick_scene
#include <cvc/gl/ariadne/stream_texture_binding.h> // StreamTextureBinding (gl-bind-stream sink)
#include <cvc/gl/ariadne/stream_verbs.h> // StreamBindingSink / register_gl_stream_intrinsics
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
class GlSceneAdapter : public ari::SceneAdapter, public StreamBindingSink {
public:
  GlSceneAdapter(SceneRenderer &view, CameraController &cam) : view_(&view), cam_(&cam) {}

  void realize(const ari::Scene &scene, const std::string &prefix,
               std::vector<std::string> *warns) override {
    // A reload rebuilds every node, so drop the imperative (gl-bind-stream) bindings now (render
    // thread) rather than letting them tick a stale node for a frame. They are runtime-only state
    // tied to the previous node graph, not part of the document, so they do NOT survive a reload;
    // re-run the binding action after the reload to re-establish them.
    verb_bindings_.clear();
    realized_ = realize_scene(view_->scene(), scene, prefix, warns);
    haveScene_ = true;
    const cvc::bounding_box bb = view_->scene().computeGraphicsBounds();
    if (!bb.isNull())
      cam_->frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  }

  void sync_visibility(cvc::app &app) override {
    if (haveScene_)
      ari::sync_scene_visibility(app, realized_.visibility); // bound visible: -> node .visible
  }

  void tick() override {
    if (haveScene_)
      tick_scene(realized_, view_->renderer()); // declarative stream / volren / volslice service
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
  SceneRenderer *view_;
  CameraController *cam_;
  RealizedScene
      realized_; // owns StageLighting rigs; outlives the render loop (adapter outlives use)
  bool haveScene_ = false;
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

  // (gl-bind-stream): register the process-global verb, and publish THIS document's GL sink (the
  // adapter) into its Runtime's document_scope so the verb routes here via ictx.document — no
  // process-global capture, correct per-document routing. Teardown order makes this safe: member
  // order (scene_ before app_rt_) means ~AriRuntime destroys app_rt_ (hence the Runtime + its
  // document_scope, dropping this handle, and ~Runtime closes the document's streams) BEFORE
  // scene_, so the handle never outlives the adapter and each verb binding's dtor unsubscribes
  // against an already-closed (null-lookup) channel. Thread contract: this teardown — like the
  // whole scene, the declarative bindings, and StreamTextureBinding in general — must run on the
  // render thread; the host destroys the AriRuntime there (the shared single-thread contract, not
  // new to this verb). Must run before the first load()/drain().
  register_gl_stream_intrinsics();
  if (auto *sink = dynamic_cast<StreamBindingSink *>(scene_.get()))
    app_rt_.runtime().document_scope().slot<GlStreamSinkHandle>(kGlStreamSinkSlot)->sink = sink;
}

AriRuntime::~AriRuntime() {
  // Drop the overlay's draw callback (which captured the Runtime) BEFORE any member is destroyed,
  // so a stray render during Python GC can't call rt.render() on a half-torn-down Runtime.
  if (overlay_)
    overlay_->setDrawCallback([] {});
}

// ── setup / document (forward to AppRuntime) ─────────────────────────────────────────────────────

void AriRuntime::add_component_path(const std::string &dir) { app_rt_.add_component_path(dir); }

void AriRuntime::on(const std::string &event, std::function<void()> handler) {
  app_rt_.on(event, std::move(handler));
}

void AriRuntime::register_verb(const std::string &name, std::function<void()> fn) {
  app_rt_.register_verb(name, std::move(fn));
}

void AriRuntime::register_async_verb(const std::string &name, const std::string &done_channel,
                                     std::function<std::string()> work) {
  app_rt_.register_async_verb(name, done_channel, std::move(work));
}

std::vector<std::string> AriRuntime::load(const std::string &path) { return app_rt_.load(path); }

void AriRuntime::set_root(ari::Widget root) { app_rt_.set_root(std::move(root)); }

// ── per-frame (GL driving around the neutral drain) ──────────────────────────────────────────────

void AriRuntime::frame(double dt_seconds) {
  view_->processUIEvents(); // VTK interactor -> ImGui (widget input + camera nav)
  cam_->update(dt_seconds);
  app_rt_.drain(); // queued Ariadne actions + scheduler slice + (via the adapter) scene sync/tick
}

void AriRuntime::render() { view_->render(); } // fires rt.render() via the overlay draw callback

void AriRuntime::step(double dt_seconds) {
  frame(dt_seconds);
  render();
}

bool AriRuntime::should_close() const { return app_rt_.should_close() || view_->windowClosed(); }

void AriRuntime::request_close() { app_rt_.request_close(); }

void AriRuntime::run(double fps) {
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

std::vector<std::string> AriRuntime::take_warnings() { return app_rt_.take_warnings(); }
bool AriRuntime::reload_if_changed() { return app_rt_.reload_if_changed(); }
bool AriRuntime::have_yaml() { return ari::AppRuntime::have_yaml(); }

} // namespace ariadne
} // namespace gl
} // namespace cvc

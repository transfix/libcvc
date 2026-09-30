// AriRuntime — see inc/cvc/gl/ariadne/AriRuntime.h. The thin cvcGL composer over the
// backend-neutral cvc::ariadne::AppRuntime: an ImGui-over-VTK backend, a GL SceneAdapter
// (realize_scene / tick_scene over the SceneRenderer's SceneGraph), and the AppRuntime
// that owns the neutral bundling. Only the GL-driving frame bits (input pump, camera, the
// draw) live here.

#include <cvc/ariadne/bind.h>        // sync_scene_visibility
#include <cvc/core/app.h>            // cvc::app
#include <cvc/gl/CameraController.h> // cam.update / frameBounds
#include <cvc/gl/ImGuiOverlay.h>     // overlay draw callback
#include <cvc/gl/SceneGraph.h>       // scene().computeGraphicsBounds()/appContext()/prefix
#include <cvc/gl/SceneRenderer.h>    // view.render()/processUIEvents()/renderer()/windowClosed()
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h> // realize_scene / RealizedScene / tick_scene
#include <memory>
#include <string>
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
class GlSceneAdapter : public ari::SceneAdapter {
public:
  GlSceneAdapter(SceneRenderer &view, CameraController &cam) : view_(&view), cam_(&cam) {}

  void realize(const ari::Scene &scene, const std::string &prefix,
               std::vector<std::string> *warns) override {
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
      tick_scene(realized_, view_->renderer()); // volren/volslice per-frame service
  }

private:
  SceneRenderer *view_;
  CameraController *cam_;
  RealizedScene
      realized_; // owns StageLighting rigs; outlives the render loop (adapter outlives use)
  bool haveScene_ = false;
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

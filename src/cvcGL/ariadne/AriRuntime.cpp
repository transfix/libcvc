// AriRuntime — see inc/cvc/gl/ariadne/AriRuntime.h. Bundles the ariadne_hello.cpp wiring behind one
// object so a host (native demo or the pycvc_gl binding) runs a full .ari app with a few calls.

#include <cvc/ariadne/bind.h>   // sync_scene_visibility
#include <cvc/ariadne/loader.h> // load_file / LoadResult / run_init
#include <cvc/ariadne/uri.h>    // register_cvc_uri_handler
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <stdexcept>
#include <utility>

#ifdef CVC_STATE_EXEC
#include <cvc/core/async_task.h>                 // launch_pool_task (register_async_verb)
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/builtins.h>        // register_fn
#include <cvc/core/state_exec/intrinsics.h>      // intrinsics_context, resolve_channel_key
#include <cvc/core/state_exec/types.h>           // value_t
#include <memory>
#include <span>
#endif

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

AriRuntime::AriRuntime(SceneRenderer &view, CameraController &cam, ImGuiOverlay &ui)
    : view_(&view), cam_(&cam), overlay_(&ui), app_(view.scene().appContext()),
      rt_(app_, view.scene().getStatePrefix()) {
  // Order matters: set_backend before install (install captures &rt_ and wires the overlay draw
  // callback to rt_.render(), so the tree walk is driven by VTK's render pass — never by the host).
  rt_.set_backend(&backend_);
  backend_.install(rt_, *overlay_);
}

AriRuntime::~AriRuntime() {
  // Drop the overlay's draw callback (which captured &rt_) BEFORE any member is destroyed, so a
  // stray render during Python GC can't call rt_.render() on a half-torn-down Runtime. backend_ is
  // declared before rt_, so rt_ is destroyed first; this closes the remaining dangling window.
  if (overlay_)
    overlay_->setDrawCallback([] {});
}

void AriRuntime::add_component_path(const std::string &dir) { componentPaths_.push_back(dir); }

void AriRuntime::on(const std::string &event, std::function<void()> handler) {
  rt_.on(event, std::move(handler));
}

bool AriRuntime::have_state_exec() { return ari::have_state_exec(); }
bool AriRuntime::have_yaml() { return ari::have_yaml(); }

// ── host verbs on the app-wide scheduler ───────────────────────────────────────────────────────

void AriRuntime::register_verb(const std::string &name, std::function<void()> fn) {
  syncVerbs_.emplace_back(name, std::move(fn));
  ensure_intrinsics();
}

void AriRuntime::register_async_verb(const std::string &name, const std::string &done_channel,
                                     std::function<std::string()> work) {
  asyncVerbs_.emplace_back(name, done_channel, std::move(work));
  ensure_intrinsics();
}

#ifdef CVC_STATE_EXEC
void AriRuntime::ensure_intrinsics() {
  if (intrinsicsRegistered_ || !have_state_exec())
    return;
  intrinsicsRegistered_ = true;
  cvc::app *app = &app_;
  // Snapshot the verbs registered so far. register_action_intrinsics providers run when a program
  // env is built; capture by value so late registrations before load() are all visible.
  auto syncs = syncVerbs_;
  auto asyncs = asyncVerbs_;
  ari::register_action_intrinsics([app, syncs,
                                   asyncs](std::shared_ptr<cvc::state_exec::environment> env,
                                           cvc::state_exec::intrinsics_context &ictx) {
    namespace se = cvc::state_exec;
    for (const auto &v : syncs) {
      std::function<void()> fn = v.second;
      se::builtins::register_fn(env, v.first, [fn](std::span<const se::value_t>) -> se::value_t {
        fn(); // host side effect, on the drain thread
        return se::value_t(std::monostate{});
      });
    }
    const std::string root = ictx.root_path;
    for (const auto &v : asyncs) {
      const std::string chan = se::resolve_channel_key(root, std::get<1>(v));
      std::function<std::string()> work = std::get<2>(v);
      // (name ...) kicks `work` on a pool worker and posts its string result to done_channel;
      // the .ari program reads it with (msg-recv "<done_channel>"). resolve the channel on the
      // scheduler thread (never walk state from a worker).
      se::builtins::register_fn(
          env, std::get<0>(v), [app, chan, work](std::span<const se::value_t>) -> se::value_t {
            app->compute_async(
                1, [](int) {},
                [app, chan, work] {
                  std::string result;
                  try {
                    result = work();
                  } catch (...) {
                    result = "";
                  }
                  app->exec_scheduler().post_message(chan, se::value_t(result));
                });
            return se::value_t(std::monostate{});
          });
    }
  });
}
#else
void AriRuntime::ensure_intrinsics() {}
#endif

// ── document ────────────────────────────────────────────────────────────────────────────────────

std::vector<std::string> AriRuntime::load(const std::string &path) {
  ari::register_cvc_uri_handler(componentPaths_); // BEFORE load (cvc:// import resolver)
  ari::LoadResult lr = ari::load_file(path);
  if (!lr.ok)
    throw std::runtime_error(lr.error.empty() ? ("AriRuntime.load: failed to load " + path)
                                              : lr.error);

  const std::string prefix = view_->scene().getStatePrefix();
  ari::run_init(app_, prefix, lr.init_script, nullptr); // §init: seeds win over widget defaults
  rt_.set_tick_program(lr.on_tick_script);              // empty = no resident (safe)
  rt_.set_key_program(lr.on_key_script);
  rt_.set_pointer_program(lr.on_pointer_script);
  if (lr.has_channels_block) {
    std::vector<std::string> declared, global;
    for (const ari::ChannelDecl &c : lr.channels) {
      declared.push_back(c.name);
      if (c.shared)
        global.push_back(c.name);
    }
    rt_.set_channel_policy(declared, global, lr.lint.channels == ari::LintConfig::Mode::Strict,
                           lr.lint.quiet);
  }

  if (lr.scene.any()) {
    std::vector<std::string> warns;
    realized_ = realize_scene(view_->scene(), lr.scene, prefix, &warns);
    haveScene_ = true;
    for (std::string &w : warns)
      lr.warnings.push_back(std::move(w));
    const cvc::bounding_box bb = view_->scene().computeGraphicsBounds();
    if (!bb.isNull())
      cam_->frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  }

  rt_.set_root(std::move(lr.root)); // applied at the next frame boundary
  sources_ = std::move(lr.sources);
  stamps_ = std::move(lr.source_stamps);
  return lr.warnings;
}

void AriRuntime::set_root(ari::Widget root) { rt_.set_root(std::move(root)); }

// ── per-frame ─────────────────────────────────────────────────────────────────────────────────

void AriRuntime::frame(double dt_seconds) {
  view_->processUIEvents(); // VTK interactor -> ImGui (widget input + camera nav)
  cam_->update(dt_seconds);
  rt_.drain(); // run queued Ariadne actions on the host thread + pump the scheduler a slice
  if (haveScene_) {
    ari::sync_scene_visibility(app_, realized_.visibility); // bound visible: -> node .visible
    tick_scene(realized_, view_->renderer());               // volren/volslice per-frame service
  }
  (void)rt_.take_reactive_warnings(); // drop read-lane diagnostics (available via take_warnings())
}

void AriRuntime::render() { view_->render(); } // fires rt_.render() via the overlay draw callback

void AriRuntime::step(double dt_seconds) {
  frame(dt_seconds);
  render();
}

bool AriRuntime::should_close() const { return quit_ || view_->windowClosed(); }

void AriRuntime::request_close() { quit_ = true; }

void AriRuntime::run(double fps) {
  const double dt = fps > 0.0 ? 1.0 / fps : 1.0 / 120.0;
  while (!should_close())
    step(dt);
}

// ── input seam (VTK-sourced; no-op producer until a caller feeds post_*) ─────────────────────────

void AriRuntime::post_key(bool down, const std::string &key, int mods, bool repeat) {
  ari::InputEvent e;
  e.kind = down ? ari::InputEvent::Kind::KeyDown : ari::InputEvent::Kind::KeyUp;
  e.key = key;
  e.mods = static_cast<unsigned>(mods);
  e.repeat = repeat;
  rt_.post_input(e);
}

void AriRuntime::post_pointer(int kind, double x, double y, double dx, double dy, int button,
                              int clicks) {
  ari::InputEvent e;
  e.kind = static_cast<ari::InputEvent::Kind>(kind);
  e.x = static_cast<float>(x);
  e.y = static_cast<float>(y);
  e.dx = static_cast<float>(dx);
  e.dy = static_cast<float>(dy);
  e.button = button;
  e.clicks = clicks;
  rt_.post_input(e);
}

// ── diagnostics ───────────────────────────────────────────────────────────────────────────────

std::vector<std::string> AriRuntime::take_warnings() { return rt_.take_reactive_warnings(); }

bool AriRuntime::reload_if_changed() {
  if (sources_.empty() || !ari::sources_changed(sources_, stamps_))
    return false;
  // Re-load from the first source (the document root); tolerate a transient failure.
  try {
    load(sources_.front());
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

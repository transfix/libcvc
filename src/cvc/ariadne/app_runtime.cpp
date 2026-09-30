// AppRuntime — see inc/cvc/ariadne/app_runtime.h. The backend-neutral bundling of a
// full .ari app (Runtime + loader + program lanes + verbs + per-frame drain); the 3D
// scene, if any, is delegated to a SceneAdapter, so this file pulls in no renderer.

#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/backend.h> // cvc::ariadne::Backend (set_backend)
#include <cvc/ariadne/loader.h>  // load_file / LoadResult / ChannelDecl / sources_changed
#include <cvc/ariadne/uri.h>     // register_cvc_uri_handler
#include <cvc/core/app.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/builtins.h>        // register_fn
#include <cvc/core/state_exec/intrinsics.h>      // intrinsics_context, resolve_channel_key
#include <cvc/core/state_exec/types.h>           // value_t
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace ariadne {

AppRuntime::AppRuntime(cvc::app &app, std::string prefix, SceneAdapter *scene)
    : app_(app), prefix_(std::move(prefix)), scene_(scene), rt_(app, prefix_) {}

AppRuntime::~AppRuntime() = default;

void AppRuntime::set_backend(Backend &backend) { rt_.set_backend(&backend); }
Runtime &AppRuntime::runtime() { return rt_; }

void AppRuntime::add_component_path(const std::string &dir) { componentPaths_.push_back(dir); }

void AppRuntime::on(const std::string &event, std::function<void()> handler) {
  rt_.on(event, std::move(handler));
}

bool AppRuntime::have_yaml() { return cvc::ariadne::have_yaml(); }

// ── host verbs on the app-wide scheduler ────────────────────────────────────────────────────────

void AppRuntime::register_verb(const std::string &name, std::function<void()> fn) {
  syncVerbs_.emplace_back(name, std::move(fn));
  ensure_intrinsics();
}

void AppRuntime::register_async_verb(const std::string &name, const std::string &done_channel,
                                     std::function<std::string()> work) {
  asyncVerbs_.emplace_back(name, done_channel, std::move(work));
  ensure_intrinsics();
}

void AppRuntime::ensure_intrinsics() {
  if (intrinsicsRegistered_)
    return;
  intrinsicsRegistered_ = true;
  cvc::app *app = &app_;
  // Snapshot the verbs registered so far. register_action_intrinsics providers run when a program
  // env is built; capture by value so late registrations before load() are all visible.
  auto syncs = syncVerbs_;
  auto asyncs = asyncVerbs_;
  register_action_intrinsics([app, syncs, asyncs](std::shared_ptr<cvc::state_exec::environment> env,
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

// ── document ────────────────────────────────────────────────────────────────────────────────────

std::vector<std::string> AppRuntime::load(const std::string &path) {
  register_cvc_uri_handler(componentPaths_); // BEFORE load (cvc:// import resolver)
  LoadResult lr = load_file(path);
  if (!lr.ok)
    throw std::runtime_error(lr.error.empty() ? ("AppRuntime.load: failed to load " + path)
                                              : lr.error);

  run_init(app_, prefix_, lr.init_script, nullptr); // §init: seeds win over widget defaults
  rt_.set_tick_program(lr.on_tick_script);          // empty = no resident (safe)
  rt_.set_key_program(lr.on_key_script);
  rt_.set_pointer_program(lr.on_pointer_script);
  if (lr.has_channels_block) {
    std::vector<std::string> declared, global;
    for (const ChannelDecl &c : lr.channels) {
      declared.push_back(c.name);
      if (c.global) // app-root-global allowlist keys on `global`, not `shared`
        global.push_back(c.name);
    }
    rt_.set_channel_policy(declared, global, lr.lint.channels == LintConfig::Mode::Strict,
                           lr.lint.quiet);
  }

  if (lr.scene.any()) {
    if (scene_) {
      std::vector<std::string> warns;
      scene_->realize(lr.scene, prefix_, &warns);
      for (std::string &w : warns)
        lr.warnings.push_back(std::move(w));
    } else {
      lr.warnings.push_back("AppRuntime.load: '" + path +
                            "' declares a scene but no SceneAdapter is set; scene ignored");
    }
  }

  rt_.set_root(std::move(lr.root)); // applied at the next frame boundary
  sources_ = std::move(lr.sources);
  stamps_ = std::move(lr.source_stamps);
  return lr.warnings;
}

void AppRuntime::set_root(Widget root) { rt_.set_root(std::move(root)); }

// ── per-frame (neutral core) ─────────────────────────────────────────────────────────────────────

void AppRuntime::drain() {
  rt_.drain(); // run queued Ariadne actions on this thread + pump the scheduler a slice
  if (scene_) {
    scene_->sync_visibility(app_); // bound visible: -> node .visible
    scene_->tick();                // volren/volslice per-frame service
  }
  (void)rt_.take_reactive_warnings(); // drop read-lane diagnostics (available via take_warnings())
}

bool AppRuntime::should_close() const { return quit_; }
void AppRuntime::request_close() { quit_ = true; }

// ── input seam ──────────────────────────────────────────────────────────────────────────────────

void AppRuntime::post_key(bool down, const std::string &key, int mods, bool repeat) {
  InputEvent e;
  e.kind = down ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
  e.key = key;
  e.mods = static_cast<unsigned>(mods);
  e.repeat = repeat;
  rt_.post_input(e);
}

void AppRuntime::post_pointer(int kind, double x, double y, double dx, double dy, int button,
                              int clicks) {
  InputEvent e;
  e.kind = static_cast<InputEvent::Kind>(kind);
  e.x = static_cast<float>(x);
  e.y = static_cast<float>(y);
  e.dx = static_cast<float>(dx);
  e.dy = static_cast<float>(dy);
  e.button = button;
  e.clicks = clicks;
  rt_.post_input(e);
}

// ── diagnostics ─────────────────────────────────────────────────────────────────────────────────

std::vector<std::string> AppRuntime::take_warnings() { return rt_.take_reactive_warnings(); }

bool AppRuntime::reload_if_changed() {
  if (sources_.empty() || !sources_changed(sources_, stamps_))
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
} // namespace cvc

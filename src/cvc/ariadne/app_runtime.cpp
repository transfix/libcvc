// AppRuntime — see inc/cvc/ariadne/app_runtime.h. The backend-neutral bundling of a
// full .ari app (Runtime + loader + program lanes + verbs + per-frame drain); the 3D
// scene, if any, is delegated to a SceneAdapter, so this file pulls in no renderer.

#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/backend.h>  // cvc::ariadne::Backend (set_backend)
#include <cvc/ariadne/doc_args.h> // the document's command line
#include <cvc/ariadne/loader.h>  // load_file / LoadResult / ChannelDecl / sources_changed
#include <cvc/ariadne/requires.h> // requires:, free_heads (what a document needs from its host)
#include <cvc/ariadne/uri.h>     // register_cvc_uri_handler
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/state/state_exec/builtins.h>        // register_fn
#include <cvc/state/state_exec/intrinsics.h>      // intrinsics_context, resolve_channel_key
#include <cvc/state/state_exec/types.h>           // value_t
#include <map>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace ariadne {

AppRuntime::AppRuntime(cvc::app &app, std::string prefix, SceneAdapter *scene)
    : app_(app), prefix_(std::move(prefix)), scene_(scene), rt_(app, prefix_),
      verbs_(std::make_shared<Verbs>()) {
  rt_.set_scene_capable(scene_ != nullptr); // the gl.scene capability (requires:, has?)
}

AppRuntime::~AppRuntime() {
  verbs_->sync.clear(); // the provider outlives us in the registry: it must bind nothing of ours
  verbs_->async.clear();
}

void AppRuntime::set_backend(Backend &backend) { rt_.set_backend(&backend); }
Runtime &AppRuntime::runtime() { return rt_; }

void AppRuntime::add_component_path(const std::string &dir) { componentPaths_.push_back(dir); }

void AppRuntime::on(const std::string &event, std::function<void()> handler) {
  rt_.on(event, std::move(handler));
}

bool AppRuntime::have_yaml() { return cvc::ariadne::have_yaml(); }

// ── host verbs on the app-wide scheduler ────────────────────────────────────────────────────────

void AppRuntime::register_verb(const std::string &name, std::function<void()> fn) {
  verbs_->sync.emplace_back(name, std::move(fn));
  ensure_intrinsics();
  rt_.refresh_program_names(); // has? / requires: see the new verb
}

void AppRuntime::register_async_verb(const std::string &name, const std::string &done_channel,
                                     std::function<std::string()> work) {
  verbs_->async.emplace_back(name, done_channel, std::move(work));
  ensure_intrinsics();
  rt_.refresh_program_names();
}

void AppRuntime::ensure_intrinsics() {
  if (intrinsicsRegistered_)
    return;
  intrinsicsRegistered_ = true;
  cvc::app *app = &app_;
  // register_action_intrinsics providers run each time a program env is built. Share the verb
  // table rather than copy it, so every verb registered so far is bound, not only the first.
  std::shared_ptr<Verbs> verbs = verbs_;
  register_action_intrinsics([app, verbs](std::shared_ptr<cvc::state_exec::environment> env,
                                          cvc::state_exec::intrinsics_context &ictx) {
    namespace se = cvc::state_exec;
    for (const auto &v : verbs->sync) {
      std::function<void()> fn = v.second;
      se::builtins::register_fn(env, v.first, [fn](std::span<const se::value_t>) -> se::value_t {
        fn(); // host side effect, on the drain thread
        return se::value_t(std::monostate{});
      });
    }
    const std::string root = ictx.root_path;
    for (const auto &v : verbs->async) {
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

// ── requires: (roadmap §7.6) ───────────────────────────────────────────────────────────────────

namespace {

// How a requires: name is described in a message: a capability token says what it means.
std::string describe_requirement(const std::string &name) {
  for (const CapabilityToken &t : capability_tokens())
    if (name == t.name)
      return name + " (" + t.meaning + ")";
  return name;
}

// Every program in a loaded document, with where it sits, for the free-head scan.
void collect_programs(const Widget &w, std::vector<std::pair<std::string, std::string>> &out) {
  const std::string where = "widget '" + (w.id.empty() ? w.label : w.id) + "'";
  for (const std::string *p :
       {&w.on, &w.on_click, &w.on_hover, &w.on_drag, &w.on_drag_start, &w.on_drag_end})
    if (!p->empty())
      out.emplace_back(where, *p);
  for (const Widget &c : w.children)
    collect_programs(c, out);
}

// The load-time preflight. requires: entries the host lacks fail the load (all named in one
// error) or, when optional, warn. Then every name a program calls that the host does not provide
// gets one warning: today that program would stop with "undefined symbol" the first time it runs.
// A name the document already listed in requires: is not repeated.
void check_host_requirements(const Runtime &rt, LoadResult &lr) {
  std::set<std::string> declared;
  std::vector<std::string> missing;
  for (const HostRequirement &req : lr.requirements) {
    declared.insert(req.name);
    if (rt.has(req.name))
      continue;
    if (req.optional)
      lr.warnings.push_back("optional requirement " + describe_requirement(req.name) +
                            " is not available on this host");
    else
      missing.push_back(describe_requirement(req.name));
  }
  if (!missing.empty()) {
    std::string list;
    for (const std::string &m : missing)
      list += (list.empty() ? "" : ", ") + m;
    throw std::runtime_error("this host does not provide what the document requires: " + list);
  }

  std::vector<std::pair<std::string, std::string>> programs;
  const std::pair<const char *, const std::string *> lanes[] = {
      {"init:", &lr.init_script},
      {"on_tick:", &lr.on_tick_script},
      {"on_key:", &lr.on_key_script},
      {"on_pointer:", &lr.on_pointer_script},
      {"on_signal:", &lr.on_signal_script}};
  for (const auto &[where, script] : lanes)
    if (!script->empty())
      programs.emplace_back(where, *script);
  collect_programs(lr.root, programs);
  std::map<std::string, std::vector<std::string>> unprovided; // name -> where it is called
  for (const auto &[where, script] : programs)
    for (const std::string &name : free_heads(script))
      if (!declared.count(name) && !rt.has_function(name)) {
        std::vector<std::string> &at = unprovided[name];
        if (at.empty() || at.back() != where)
          at.push_back(where);
      }
  for (const auto &[name, at] : unprovided) {
    std::string where;
    for (std::size_t i = 0; i < at.size() && i < 3; ++i)
      where += (i ? ", " : "") + at[i];
    if (at.size() > 3)
      where += " and " + std::to_string(at.size() - 3) + " more";
    lr.warnings.push_back(where + " calls (" + name +
                          " ...), which this host does not provide; that program stops there "
                          "when it runs (list it in requires: to make this an error, or as "
                          "optional if the program checks (has? \"" +
                          name + "\") first)");
  }
}

} // namespace

// ── document ────────────────────────────────────────────────────────────────────────────────────

void AppRuntime::set_document_args(std::vector<std::string> argv, std::string invocation,
                                   bool validate) {
  docArgv_ = std::move(argv);
  invocation_ = std::move(invocation);
  validateArgs_ = validate;
}

bool AppRuntime::has_args_block() const { return hasArgsBlock_; }

std::vector<std::string> AppRuntime::load(const std::string &path) {
  register_cvc_uri_handler(componentPaths_); // BEFORE load (cvc:// import resolver)
  LoadResult lr = load_file(path);
  if (!lr.ok)
    throw std::runtime_error(lr.error.empty() ? ("AppRuntime.load: failed to load " + path)
                                              : lr.error);
  check_host_requirements(rt_, lr); // requires: and the calls programs make (§7.6)

  // The document's command line goes on the tree before init: runs, so init can read it.
  hasArgsBlock_ = lr.has_args_block;
  DocumentArgs parsed;
  if (lr.has_args_block) {
    parsed = parse_document_args(lr.args, validateArgs_ ? docArgv_ : std::vector<std::string>());
    if (validateArgs_ && (parsed.help || !parsed.errors.empty())) {
      const std::string about = !lr.meta.description.empty() ? lr.meta.description : lr.meta.name;
      std::string usage =
          format_document_usage(invocation_.empty() ? path : invocation_, about, lr.args);
      if (parsed.help)
        throw document_args_error("help requested", std::move(usage), true);
      std::string what;
      for (const std::string &e : parsed.errors)
        what += (what.empty() ? "" : "; ") + e;
      throw document_args_error(what, std::move(usage), false);
    }
    parsed.errors.clear(); // a check: a required argument nobody passed is not the document's fault
  }
  publish_document_args(app_, prefix_, docArgv_, lr.has_args_block ? &lr.args : nullptr,
                        lr.has_args_block ? &parsed : nullptr);

  std::vector<std::string> initErrors;
  run_init(app_, prefix_, lr.init_script, &initErrors, // §init: seeds win over widget defaults
           [this](const std::string &n) { return rt_.has(n); });
  for (std::string &e : initErrors) // a failed init: is not fatal, but it must not be silent
    lr.warnings.push_back(std::move(e));
  rt_.set_tick_program(lr.on_tick_script);          // empty = no resident (safe)
  rt_.set_key_program(lr.on_key_script);
  rt_.set_pointer_program(lr.on_pointer_script);
  rt_.set_signal_program(lr.on_signal_script);
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
      if (scene_->verify(lr, &warns)) // a missing required node custom: no half-built scene
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
  // Keep the read-lane diagnostics for take_warnings(); the Runtime reports each one only once.
  for (std::string &w : rt_.take_reactive_warnings())
    pendingWarnings_.push_back(std::move(w));
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

bool AppRuntime::post_signal(const std::string &signal, const std::string &source) {
  return rt_.post_signal(signal, source);
}

// ── diagnostics ─────────────────────────────────────────────────────────────────────────────────

std::vector<std::string> AppRuntime::take_warnings() {
  std::vector<std::string> out = std::move(pendingWarnings_);
  pendingWarnings_.clear();
  for (std::string &w : rt_.take_reactive_warnings())
    out.push_back(std::move(w));
  return out;
}

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

// Ariadne core walker + runtime (cvc::ariadne). Pure libcvc: NO ImGui/VTK.
//
// The walk drives a pluggable Backend (backend.h): the core owns the retained
// tree, the reconcile boundary, and ALL cvc::state binding; the backend only
// draws value-in / edit-out. For each bound widget the core reads (and seeds) the
// state value, hands it to a backend primitive, and writes back ONLY when the
// backend reports the edit committed — so a slider drag costs one state write on
// release, not one per frame (writes fan out to observers / replicated peers).

#include <cvc/ariadne/ariadne.h>

#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#ifdef CVC_STATE_EXEC
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/parser.h> // parse, parse_error
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h> // §4 read-lane predicate evaluator
#endif

#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// read_or_seed<T> / write<T> / resolve_bind now live in cvc/ariadne/bind.h so the
// widget walk here and the scene binder (§9) share ONE definition and can't drift.
// They are used unqualified below (same cvc::ariadne namespace).

// Read a state value as a plain string with no seeding (read-only Text view).
std::string read_string(cvc::app &ctx, const std::string &path) {
  try {
    return cvc::state::instance(ctx)(path).value();
  } catch (const std::exception &) {
    return "<unset>";
  }
}

} // namespace

// --- custom widget type registry (§ extensibility) --------------------------
namespace {
std::mutex &widget_mutex() {
  static std::mutex m;
  return m;
}
std::unordered_map<std::string, WidgetEmitFn> &widget_registry() {
  static std::unordered_map<std::string, WidgetEmitFn> r;
  return r;
}
WidgetEmitFn lookup_widget(const std::string &type) {
  std::lock_guard<std::mutex> lock(widget_mutex());
  auto it = widget_registry().find(type);
  return it != widget_registry().end() ? it->second : WidgetEmitFn{};
}
} // namespace

void register_widget_type(const std::string &type, WidgetEmitFn emit) {
  if (!emit || type.empty())
    return;
  std::lock_guard<std::mutex> lock(widget_mutex());
  widget_registry()[type] = std::move(emit);
}

bool has_widget_type(const std::string &type) {
  std::lock_guard<std::mutex> lock(widget_mutex());
  return widget_registry().find(type) != widget_registry().end();
}

// --- the init: block runner (state_exec, gated by CVC_STATE_EXEC) ------------

bool have_state_exec() {
#ifdef CVC_STATE_EXEC
  return true;
#else
  return false;
#endif
}

bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors) {
  const auto err = [&](const std::string &m) {
    if (errors)
      errors->push_back(m);
  };
  if (script.empty())
    return true; // nothing to run
#ifndef CVC_STATE_EXEC
  (void)app;
  (void)prefix;
  err("ari: init: script present but this libcvc was built without state_exec "
      "(CVC_STATE_EXEC=OFF) — the script did not run");
  return false;
#else
  namespace se = cvc::state_exec;
  try {
    // All of these must outlive execute()+run(): the intrinsics capture &ictx by
    // pointer. A synchronous run-to-completion in this one scope satisfies that.
    se::scheduler sched;
    se::memory_tracker tracker;
    auto proc = se::make_process();
    proc->pid = 1;
    proc->status = se::process_status::ready;
    se::intrinsics_context ictx;
    cvc::state &root = cvc::state::instance(app);
    ictx.sched = &sched;
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    se::apply_chroot(ictx, root, prefix); // scope writes under the document prefix
    auto env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    // Bound the init so a looping or blocking script (e.g. an accidental infinite
    // loop, or a (msg-recv ...) with no sender) can never hang the app at load: cap
    // the process (check_limits kills a runner at the cap) AND the run loop (breaks out
    // even when the sole process is blocked, which check_limits can't catch). A
    // load-time seed/compute should finish well within these.
    static constexpr uint64_t kInitMaxSteps = 10'000'000;
    static constexpr double kInitMaxSeconds = 5.0;
    se::execute_options opts;
    opts.env = env;
    opts.max_steps = kInitMaxSteps;
    opts.max_time = kInitMaxSeconds;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    sched.run(kInitMaxSteps, kInitMaxSeconds);   // bounded run to completion
    if (auto info = sched.get_process_info(pid)) {
      // Success is ONLY a normal finish. Any other terminal/non-terminal state — killed
      // (runtime error / resource limit) or still ready/running/waiting after the cap
      // (it blocked or looped past the budget) — is a reported failure, not silent.
      if (info->status != se::process_status::terminated) {
        err("ari: init: script did not complete normally (a runtime error, a resource "
            "limit, or it blocked/looped past the init budget)");
        return false;
      }
    }
    return true;
  } catch (const std::exception &e) {
    err(std::string("ari: init: script failed: ") + e.what());
    return false;
  }
#endif
}

// --- §4 read-lane: the per-frame reactive predicate evaluator ----------------
#ifdef CVC_STATE_EXEC
namespace {
namespace se = cvc::state_exec;

// Evaluates a widget's read-lane expressions (visible_when, …) each frame on ONE
// long-lived stackless evaluator over a DEFAULT-DENY environment: the pure builtins
// plus ONLY the side-effect-free state *readers*. The writers (state-set/delete/…),
// scheduler ops (spawn/kill/msg-*), state watches, and I/O are never installed, so a
// predicate that tries one hits an unbound symbol and fails (fail-safe) instead of
// mutating state or blocking. Each distinct expression source is parsed ONCE and
// cached; every evaluation is capped in BOTH steps and wall-time and never throws out.
//
// state-get keys are prefix-relative (the context is chroot'd to the document prefix),
// so `(state-get "demo.n")` reads the SAME key a widget `bind: demo.n` resolves to.
class ReactiveEngine {
public:
  ReactiveEngine(cvc::app &app, const std::string &prefix) {
    // The intrinsics capture &ctx_ by pointer; sched_/tracker_/proc_ back it and share
    // this object's lifetime. No scheduler op is ever exposed, so the scheduler stays
    // idle — it exists only to give the readers a well-formed context (mirrors run_init).
    proc_ = se::make_process();
    proc_->pid = 1;
    proc_->status = se::process_status::ready;
    cvc::state &root = cvc::state::instance(app);
    ctx_.sched = &sched_;
    ctx_.tracker = &tracker_;
    ctx_.proc = proc_;
    ctx_.pid = 1;
    se::apply_chroot(ctx_, root, prefix);

    env_ = se::builtins::make_default_environment();
    // Copy ONLY the read-only intrinsics out of a full registration (allowlist =
    // default-deny). Anything not listed here is simply never bound in env_.
    se::environment_ptr full = se::builtins::make_default_environment();
    se::register_intrinsics(full, &ctx_);
    static const char *const kReaders[] = {
        "state-get",       "state-exists",     "state-children", "state-data-get",
        "state-root-path", "state-has-expiry", "state-is-expired"};
    for (const char *name : kReaders)
      if (const se::value_t *v = full->lookup(name))
        env_->set(name, *v);
    ev_ = std::make_unique<se::stackless_evaluator>(env_);
  }

  struct Outcome {
    bool value;        // predicate result, or the caller's `dflt` on any failure
    std::string error; // empty on success; a diagnostic otherwise
  };

  // Evaluate `src` as a boolean predicate. Fail-safe: a parse error, a runtime error
  // (e.g. an unbound writer symbol, a type error), or a step/time-cap overrun returns
  // {dflt, <why>} rather than throwing.
  Outcome eval_bool(const std::string &src, bool dflt) {
    const se::value_t *expr = compile(src);
    if (!expr)
      return {dflt, "ari: visible_when: parse error in \"" + src + "\" — widget hidden"};
    try {
      se::evaluator_state st = ev_->create_state(*expr);
      const se::value_t r = ev_->run(st, kMaxSteps, kMaxSeconds);
      if (!st.done)
        return {dflt, "ari: visible_when: \"" + src +
                          "\" exceeded the per-frame budget (step/time cap) — widget hidden"};
      return {r.is_truthy(), std::string()};
    } catch (const std::exception &e) {
      return {dflt, "ari: visible_when: \"" + src + "\" failed at eval (" + e.what() +
                        ") — widget hidden"};
    }
  }

private:
  // Parse once; cache the AST. A parse failure caches std::nullopt so a broken predicate
  // is not re-parsed every frame. Returns nullptr on a (cached) parse failure.
  const se::value_t *compile(const std::string &src) {
    auto it = compiled_.find(src);
    if (it != compiled_.end())
      return it->second ? &*it->second : nullptr;
    try {
      se::value_t v = se::parse(src);
      auto ins = compiled_.emplace(src, std::move(v));
      return &*ins.first->second;
    } catch (const se::parse_error &) {
      compiled_.emplace(src, std::nullopt);
      return nullptr;
    }
  }

  static constexpr uint64_t kMaxSteps = 200'000; // per-eval step cap (predicates are tiny)
  static constexpr double kMaxSeconds = 0.02;    // per-eval wall-time backstop

  se::scheduler sched_;
  se::memory_tracker tracker_;
  std::shared_ptr<se::process> proc_;
  se::intrinsics_context ctx_;
  se::environment_ptr env_;
  std::unique_ptr<se::stackless_evaluator> ev_;
  std::unordered_map<std::string, std::optional<se::value_t>> compiled_;
};
} // namespace
#endif // CVC_STATE_EXEC

struct Runtime::Impl {
  cvc::app &app;
  Backend *backend = nullptr;
  std::string prefix;

  Widget root;
  Widget pending;
  bool has_pending = false;

  std::unordered_map<std::string, std::function<void()>> handlers;
  std::vector<std::string> queued_events;

  // §4 read-lane: the reactive predicate evaluator (lazily built on first use so a UI
  // with no reactive fields pays nothing), plus de-duplicated diagnostics surfaced by
  // take_reactive_warnings(). The warnings live regardless of state_exec (the OFF path
  // also warns once). reactive_warned keeps the dedup set across drains.
#ifdef CVC_STATE_EXEC
  std::unique_ptr<ReactiveEngine> reactive;
#endif
  std::vector<std::string> reactive_warnings;
  std::set<std::string> reactive_warned;

  Impl(cvc::app &a, std::string p) : app(a), prefix(std::move(p)) {}

  // Resolve a widget bind path to an absolute cvc::state path. Delegates to the
  // shared rule (bind.h) so widget binds and scene `visible:` binds collide on the
  // same key for the same relative path.
  std::string resolve(const std::string &bind) const { return resolve_bind(prefix, bind); }

  void enqueue(const std::string &event) {
    if (!event.empty())
      queued_events.push_back(event);
  }

  void emit(const Widget &w);
  void emit_children(const Widget &w);
  void emit_container(const Widget &w); // lay children out per w.layout (§3.0.3b)
  void render();

  // §4 read-lane: is `w` shown this frame? True when it has no visible_when; otherwise
  // the predicate's result (fail-safe HIDDEN on a state_exec build, fail-safe SHOWN on a
  // build without state_exec — hiding every reactive widget would gut a minimal build).
  bool visible(const Widget &w);
  void warn_once(const std::string &msg) {
    if (reactive_warned.insert(msg).second)
      reactive_warnings.push_back(msg);
  }
};

bool Runtime::Impl::visible(const Widget &w) {
  if (w.visible_when.empty())
    return true;
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  const ReactiveEngine::Outcome o = reactive->eval_bool(w.visible_when, /*dflt=*/false);
  if (!o.error.empty())
    warn_once(o.error);
  return o.value;
#else
  warn_once("ari: visible_when on '" + (w.label.empty() ? w.id : w.label) +
            "' ignored — this libcvc was built without state_exec (CVC_STATE_EXEC=OFF)");
  return true;
#endif
}

void Runtime::Impl::emit_children(const Widget &w) {
  for (const Widget &c : w.children)
    emit(c);
}

// Lay a container's children out per its layout (§3.0.3b): a Grid/Horizontal
// layout flows them into the backend's sized tracks; anything else is a plain
// vertical stack. Used for both a Window's body and a Group.
void Runtime::Impl::emit_container(const Widget &w) {
  Backend &b = *backend;
  if (w.layout.kind == LayoutKind::Grid || w.layout.kind == LayoutKind::Horizontal) {
    const std::string &id = w.id.empty() ? w.label : w.id;
    if (b.begin_grid(w.layout, id.c_str())) {
      for (const Widget &c : w.children) {
        b.grid_next_cell();
        emit(c);
      }
      b.end_grid();
    }
  } else {
    emit_children(w);
  }
}

void Runtime::Impl::emit(const Widget &w) {
  if (!visible(w))
    return; // §4 read-lane: a falsy visible_when hides this widget and its whole subtree
  Backend &b = *backend;
  const char *label = w.label.c_str();
  switch (w.kind) {
  case Kind::Menubar:
    if (b.begin_main_menu_bar()) {
      emit_children(w);
      b.end_main_menu_bar();
    }
    break;

  case Kind::Menu:
    if (b.begin_menu(label)) {
      emit_children(w);
      b.end_menu();
    }
    break;

  case Kind::MenuItemToggle: {
    const std::string path = resolve(w.bind);
    const bool cur = read_or_seed<int>(app, path, w.bdef ? 1 : 0) != 0;
    const BoolEdit e = b.menu_item_toggle(label, cur);
    if (e.committed)
      write<int>(app, path, e.value ? 1 : 0);
    break;
  }

  case Kind::MenuItemAction:
    if (b.menu_item_action(label))
      enqueue(w.on);
    break;

  case Kind::Window: {
    // The stable identity is the id (defaults to the label); the backend keeps a
    // window's geometry keyed on it. Begin/End pair unconditionally.
    const std::string &id = w.id.empty() ? w.label : w.id;
    const bool visible = b.begin_window(w.label.c_str(), id.c_str(), w.size, w.frame_border);
    if (visible) {
      b.push_id(id.c_str());
      emit_container(w); // §3.0.3b: the window body honours w.layout
      b.pop_id();
    }
    b.end_window();
    break;
  }

  case Kind::Group:
    emit_container(w); // §3.0.3b: a group honours w.layout (grid / vertical)
    break;

  case Kind::Text:
    if (w.literal_text)
      b.text_line(label);
    else
      b.text_value(label, read_string(app, resolve(w.bind)));
    break;

  case Kind::Separator:
    b.separator();
    break;

  case Kind::Checkbox: {
    const std::string path = resolve(w.bind);
    const bool cur = read_or_seed<int>(app, path, w.bdef ? 1 : 0) != 0;
    const BoolEdit e = b.checkbox(label, cur);
    if (e.committed)
      write<int>(app, path, e.value ? 1 : 0);
    break;
  }

  case Kind::SliderInt: {
    const std::string path = resolve(w.bind);
    const int cur = read_or_seed<int>(app, path, w.idef);
    const IntEdit e = b.slider_int(label, path.c_str(), cur, w.ilo, w.ihi);
    if (e.committed)
      write<int>(app, path, e.value);
    break;
  }

  case Kind::SliderFloat: {
    const std::string path = resolve(w.bind);
    const double cur = read_or_seed<double>(app, path, w.def);
    const DoubleEdit e = b.slider_double(label, path.c_str(), cur, w.lo, w.hi, w.fmt.c_str());
    if (e.committed)
      write<double>(app, path, e.value);
    break;
  }

  case Kind::Combo: {
    if (w.options.empty())
      break;
    const std::string path = resolve(w.bind);
    const std::string fallback = w.sdef.empty() ? w.options.front() : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, fallback);
    int idx = 0;
    for (std::size_t i = 0; i < w.options.size(); ++i)
      if (w.options[i] == cur) {
        idx = static_cast<int>(i);
        break;
      }
    const IndexEdit e = b.combo(label, idx, w.options);
    if (e.changed && e.index >= 0 && e.index < static_cast<int>(w.options.size()))
      write<std::string>(app, path, w.options[e.index]); // store the TEXT, not an index
    break;
  }

  case Kind::Button:
    if (b.button(label))
      enqueue(w.on);
    break;

  case Kind::Custom: {
    // Three ways a custom widget renders, in priority order:
    //  1. a COMPOSITIONAL fn (register_widget_type) — composes built-ins through a
    //     context that reuses the core's binding; works on every backend;
    //  2. else the BACKEND's novel-primitive escape (custom_widget) — the core reads
    //     the bound value, hands it in, and writes back on the committed edge;
    //  3. else a placeholder (neither knows this type).
    if (const WidgetEmitFn fn = lookup_widget(w.custom_type)) {
      WidgetEmitContext ctx;
      ctx.emit = [this](const Widget &child) { emit(child); };
      ctx.read = [this](const std::string &bind) { return read_string(app, resolve(bind)); };
      ctx.fire = [this](const std::string &event) { enqueue(event); };
      fn(w, ctx);
      break;
    }
    const std::string path = w.bind.empty() ? std::string() : resolve(w.bind);
    const std::string cur = w.bind.empty() ? std::string() : read_string(app, path);
    const CustomEdit e = b.custom_widget(w.custom_type.c_str(), cur, w);
    if (!e.handled)
      b.text_line(("[" + w.custom_type + "?]").c_str()); // neither composed nor backend-drawn
    else if (e.committed && !w.bind.empty())
      write<std::string>(app, path, e.value);
    break;
  }
  }
}

void Runtime::Impl::render() {
  if (!backend)
    return;
  // Reconcile commit boundary (roadmap §11.5.1): a queued tree swap is applied
  // here, at the top of the frame, before any node is emitted — never mid-walk.
  if (has_pending) {
    root = std::move(pending);
    pending = Widget{};
    has_pending = false;
  }
  backend->begin_frame();
  emit(root);
  backend->end_frame();
}

// --------------------------------------------------------------------------

Runtime::Runtime(cvc::app &app, std::string prefix) : m_(new Impl(app, std::move(prefix))) {}

Runtime::~Runtime() = default;

void Runtime::set_backend(Backend *backend) { m_->backend = backend; }

void Runtime::set_root(Widget root) {
  m_->pending = std::move(root);
  m_->has_pending = true;
}

void Runtime::on(std::string event, std::function<void()> handler) {
  m_->handlers[std::move(event)] = std::move(handler);
}

void Runtime::render() { m_->render(); }

void Runtime::drain() {
  // Run the queued action events on the host thread, then clear. Never called
  // from inside render() (the deferred intent-buffer discipline, §7.2/§4.7).
  std::vector<std::string> events;
  events.swap(m_->queued_events);
  for (const std::string &e : events) {
    auto it = m_->handlers.find(e);
    if (it != m_->handlers.end() && it->second)
      it->second();
  }
}

std::vector<std::string> Runtime::take_reactive_warnings() {
  std::vector<std::string> out;
  out.swap(m_->reactive_warnings); // reactive_warned stays -> still deduped across drains
  return out;
}

} // namespace ariadne
} // namespace cvc

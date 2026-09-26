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
#include <cvc/core/state_exec/parser.h> // parse_error
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#endif

#include <exception>
#include <functional>
#include <mutex>
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
    se::execute_options opts;
    opts.env = env;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    sched.run();                                  // run to completion
    if (auto info = sched.get_process_info(pid)) {
      if (info->status == se::process_status::killed) {
        err("ari: init: script terminated abnormally (runtime error or resource limit)");
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

struct Runtime::Impl {
  cvc::app &app;
  Backend *backend = nullptr;
  std::string prefix;

  Widget root;
  Widget pending;
  bool has_pending = false;

  std::unordered_map<std::string, std::function<void()>> handlers;
  std::vector<std::string> queued_events;

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
};

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
    // A registered custom widget type composes built-in widgets through a context
    // that reuses the core's binding; an unregistered type shows a placeholder. The
    // fn drives only built-ins, so a custom widget renders on every backend.
    const WidgetEmitFn fn = lookup_widget(w.custom_type);
    if (fn) {
      WidgetEmitContext ctx;
      ctx.emit = [this](const Widget &child) { emit(child); };
      ctx.read = [this](const std::string &bind) { return read_string(app, resolve(bind)); };
      ctx.fire = [this](const std::string &event) { enqueue(event); };
      fn(w, ctx);
    } else {
      b.text_line(("[" + w.custom_type + "?]").c_str()); // unregistered: visible, not silent
    }
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

} // namespace ariadne
} // namespace cvc

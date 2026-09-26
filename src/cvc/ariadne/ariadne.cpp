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
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#include <exception>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// Read a state value, seeding it with `def` when the path has no value yet.
// (Lifted verbatim from cvc::gl::ui — this is the pure-cvc::state half that now
// lives in the backend-neutral core.) Never throws into a render frame.
template <typename T> T read_or_seed(cvc::app &ctx, const std::string &path, const T &def) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    const std::string raw = s.value();
    if (raw.empty()) {
      s.value(def);
      return def;
    }
    return s.value<T>();
  } catch (const std::exception &) {
    return def; // unreadable/unconvertible: fall back, never throw into a frame
  }
}

template <typename T> void write(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
    // read-only or otherwise unwritable — the widget just won't stick.
  }
}

// Read a state value as a plain string with no seeding (read-only Text view).
std::string read_string(cvc::app &ctx, const std::string &path) {
  try {
    return cvc::state::instance(ctx)(path).value();
  } catch (const std::exception &) {
    return "<unset>";
  }
}

} // namespace

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

  // Resolve a widget bind path to an absolute cvc::state path.
  //   leading '/'  -> app-root-absolute (strip the '/')
  //   else, prefix -> "<prefix>.<bind>"    (splice, never hand-concat elsewhere)
  //   else         -> bind as-is (app-root-relative)
  std::string resolve(const std::string &bind) const {
    if (bind.empty())
      return bind;
    if (bind[0] == '/')
      return bind.substr(1);
    if (prefix.empty())
      return bind;
    return prefix + cvc::state::SEPARATOR + bind;
  }

  void enqueue(const std::string &event) {
    if (!event.empty())
      queued_events.push_back(event);
  }

  void emit(const Widget &w);
  void emit_children(const Widget &w);
  void render();
};

void Runtime::Impl::emit_children(const Widget &w) {
  for (const Widget &c : w.children)
    emit(c);
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
    const bool visible = b.begin_window(w.label.c_str(), id.c_str());
    if (visible) {
      b.push_id(id.c_str());
      emit_children(w);
      b.pop_id();
    }
    b.end_window();
    break;
  }

  case Kind::Group:
    emit_children(w);
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

// Ariadne — the cvcGL UI & scene DSL (cvc::gl::ariadne). P0 walker + runtime.
//
// The walk uses ONLY the cvc::gl::ui:: pointer-free Dear ImGui subset
// (ImGuiBinding.h): the state-bound widgets (SliderDouble/SliderInt/Checkbox/
// MenuItem/Combo/Text) and the structural primitives (BeginMainMenuBar/BeginMenu/
// Begin/…). That keeps this TU imgui-header-free, binds cvcGL's own ImGui
// context, and degrades to safe no-ops when libcvc is built without
// CVC_ENABLE_IMGUI (every ui:: entry is then an inert stub).

#include <cvc/gl/ariadne/ariadne.h>

#include <cvc/gl/ImGuiBinding.h> // cvc::gl::ui::*
#include <cvc/gl/ImGuiOverlay.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ui = cvc::gl::ui;

struct Runtime::Impl {
  cvc::app &app;
  ImGuiOverlay &overlay;
  std::string prefix;

  Widget root;
  Widget pending;
  bool has_pending = false;

  std::unordered_map<std::string, std::function<void()>> handlers;
  std::vector<std::string> queued_events;
  bool in_walk = false;

  Impl(cvc::app &a, ImGuiOverlay &o, std::string p) : app(a), overlay(o), prefix(std::move(p)) {}

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
    return prefix + "." + bind;
  }

  void enqueue(const std::string &event) {
    if (!event.empty())
      queued_events.push_back(event);
  }

  void emit(const Widget &w);
  void emit_children(const Widget &w);
  void walk();
};

void Runtime::Impl::emit_children(const Widget &w) {
  for (const Widget &c : w.children)
    emit(c);
}

void Runtime::Impl::emit(const Widget &w) {
  const char *label = w.label.c_str();
  switch (w.kind) {
  case Kind::Menubar:
    if (ui::BeginMainMenuBar()) {
      emit_children(w);
      ui::EndMainMenuBar();
    }
    break;

  case Kind::Menu:
    if (ui::BeginMenu(label)) {
      emit_children(w);
      ui::EndMenu();
    }
    break;

  case Kind::MenuItemToggle:
    ui::MenuItem(app, label, resolve(w.bind), w.bdef);
    break;

  case Kind::MenuItemAction:
    if (ui::MenuItemClicked(label))
      enqueue(w.on);
    break;

  case Kind::Window: {
    // Stable ImGui id via the "Visible title###stable.id" trick, so two windows
    // that happen to share a title don't merge (roadmap §3.3). End() is called
    // unconditionally per ImGui's contract, even when Begin() returns false.
    std::string name = w.label;
    name += "###";
    name += w.id.empty() ? w.label : w.id;
    if (ui::Begin(name.c_str())) {
      ui::PushId(w.id.empty() ? w.label.c_str() : w.id.c_str());
      emit_children(w);
      ui::PopId();
    }
    ui::End();
    break;
  }

  case Kind::Group:
    emit_children(w);
    break;

  case Kind::Text:
    if (w.literal_text)
      ui::TextLine(label);
    else
      ui::Text(app, label, resolve(w.bind));
    break;

  case Kind::Separator:
    ui::Separator();
    break;

  case Kind::Checkbox:
    ui::Checkbox(app, label, resolve(w.bind), w.bdef);
    break;

  case Kind::SliderInt:
    ui::SliderInt(app, label, resolve(w.bind), w.ilo, w.ihi, w.idef);
    break;

  case Kind::SliderFloat:
    ui::SliderDouble(app, label, resolve(w.bind), w.lo, w.hi, w.def, w.fmt.c_str());
    break;

  case Kind::Combo:
    ui::Combo(app, label, resolve(w.bind), w.options, w.sdef);
    break;

  case Kind::Button:
    if (ui::Button(label))
      enqueue(w.on);
    break;
  }
}

void Runtime::Impl::walk() {
  // Reconcile commit boundary (roadmap §11.5.1): a queued tree swap is applied
  // here, at the top of the frame, before any node is emitted — never mid-walk.
  if (has_pending) {
    root = std::move(pending);
    pending = Widget{};
    has_pending = false;
  }
  in_walk = true;
  emit(root);
  in_walk = false;
}

// --------------------------------------------------------------------------

Runtime::Runtime(cvc::app &app, ImGuiOverlay &overlay, std::string prefix)
    : m_(new Impl(app, overlay, std::move(prefix))) {}

Runtime::~Runtime() = default;

void Runtime::set_root(Widget root) {
  m_->pending = std::move(root);
  m_->has_pending = true;
}

void Runtime::install() {
  Impl *impl = m_.get();
  m_->overlay.setDrawCallback([impl] { impl->walk(); });
}

void Runtime::on(std::string event, std::function<void()> handler) {
  m_->handlers[std::move(event)] = std::move(handler);
}

void Runtime::drain() {
  // Run the queued action events on the host thread, then clear. Never called
  // from inside the walk (the deferred intent-buffer discipline, §7.2/§4.7).
  std::vector<std::string> events;
  events.swap(m_->queued_events);
  for (const std::string &e : events) {
    auto it = m_->handlers.find(e);
    if (it != m_->handlers.end() && it->second)
      it->second();
  }
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

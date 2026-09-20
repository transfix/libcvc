// pycvc_imgui.i — cvcGL Dear ImGui overlay + the state-bound cvc::gl::ui panels,
// so Python can build cvcGL HUDs/UIs WITHOUT ever calling raw ImGui::.
//
// A sub-interface %include'd into pycvc_gl.i AFTER the CameraController /
// StageLighting / ScreenTextHud / SceneGraph / SceneRenderer wraps, so every
// referenced type resolves. It reuses pycvc_gl.i's file-scope PyCallable ->
// std::function<void()> typemap (the one SceneGraph::postEvent uses) for the
// ImGuiOverlay draw callback, and the %import'd StringVector for Combo's options.
//
// Raw ImGui:: is deliberately NOT exposed: the _pycvc_gl extension is a separate
// binary with its own null GImGui, so calling ImGui:: from Python would crash
// (the static-context trap). Python draws HUDs by (a) setting a draw callback and
// (b) calling the state-bound ui_* widgets/panels below, whose bodies run INSIDE
// cvcGL against the overlay's own ImGui context. A curated raw subset, if ever
// needed, belongs in cvcGL's ImGuiBinding, not here.

%{
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/ImGuiBinding.h>
%}

// ── ImGuiOverlay: the per-viewer Dear ImGui integration ─────────────────────
// Not a state_object (no new 401s). imguiContext() returns an opaque
// ImGuiContext* Python never needs (it issues no raw ImGui) — ignore it so the
// opaque pointer never enters the type table. setDrawCallback(std::function)
// binds via the in-scope callable typemap (a long-lived stored callback: it is
// re-invoked EVERY rendered frame, so the Python draw code must be cheap and must
// not raise — an exception is printed and swallowed per frame). attachCamera ties
// the HUD to the shared state-driven controller, the same seam in C++ and Python.
%ignore cvc::gl::ImGuiOverlay::imguiContext; // opaque ImGuiContext*
// Keep the injected viewer alive: ~ImGuiOverlay detaches its window observers.
%pythonappend cvc::gl::ImGuiOverlay::ImGuiOverlay %{
    if args: self._pycvc_keepalive = args[0]
%}
// Keep the attached camera alive: the overlay holds it by raw pointer.
%pythonappend cvc::gl::ImGuiOverlay::attachCamera %{
    if args: self._pycvc_camera = args[0]
%}
%include "cvc/gl/ImGuiOverlay.h"

// ── cvc::gl::ui: state-bound widgets (read/write cvc::state) ─────────────────
// Group A — clean signatures, renamed to ui_* : Text/Combo/MenuItem/Checkbox are
// foot-gun leaf names in the flat (nspace-less) module. cvc::app& marshals from
// the shared_ptr<cvc::app> proxy (as CameraController's app& ctor does); Combo's
// vector<string> uses the %import'd StringVector. ALL %rename/%ignore MUST precede
// the header %include (a %rename after it is a redefinition; a wrapper body before
// it will not compile).
%rename(ui_slider_double) cvc::gl::ui::SliderDouble;
%rename(ui_slider_int) cvc::gl::ui::SliderInt;
%rename(ui_drag_double) cvc::gl::ui::DragDouble;
%rename(ui_checkbox) cvc::gl::ui::Checkbox;
%rename(ui_menu_item) cvc::gl::ui::MenuItem;
%rename(ui_combo) cvc::gl::ui::Combo;
%rename(ui_text) cvc::gl::ui::Text;
%rename(ui_camera_menu_items) cvc::gl::ui::CameraMenuItems;
// Group B — the composite panels take bool* open out-params SWIG can't express;
// ignore them and re-expose below as no-open + *_open (returns the updated flag).
%ignore cvc::gl::ui::StageLightingPanel;
%ignore cvc::gl::ui::ScenePanel;
%ignore cvc::gl::ui::SceneMenuItems;
// Curated raw immediate-mode subset -> imgui_* (disjoint from the ui_* helpers
// and from ImGuiOverlay's methods; those that shadow a state-bound helper carry a
// distinct C++ suffix — TextLine/CheckboxValue/SliderIntValue/... — so no clash).
// Value-in/value-out + no pointers, so they marshal in the flat module directly.
// Legal only inside an ImGuiOverlay draw callback (see the header). Renames MUST
// precede the %include.
%rename(imgui_begin) cvc::gl::ui::Begin;
%rename(imgui_end) cvc::gl::ui::End;
%rename(imgui_same_line) cvc::gl::ui::SameLine;
%rename(imgui_separator) cvc::gl::ui::Separator;
%rename(imgui_spacing) cvc::gl::ui::Spacing;
%rename(imgui_push_id) cvc::gl::ui::PushId;
%rename(imgui_push_id_int) cvc::gl::ui::PushIdInt;
%rename(imgui_pop_id) cvc::gl::ui::PopId;
%rename(imgui_text) cvc::gl::ui::TextLine;
%rename(imgui_text_disabled) cvc::gl::ui::TextDisabledLine;
%rename(imgui_button) cvc::gl::ui::Button;
%rename(imgui_small_button) cvc::gl::ui::SmallButton;
%rename(imgui_selectable) cvc::gl::ui::Selectable;
%rename(imgui_checkbox) cvc::gl::ui::CheckboxValue;
%rename(imgui_slider_float) cvc::gl::ui::SliderFloatValue;
%rename(imgui_slider_int) cvc::gl::ui::SliderIntValue;
%rename(imgui_drag_float) cvc::gl::ui::DragFloatValue;
%rename(imgui_begin_main_menu_bar) cvc::gl::ui::BeginMainMenuBar;
%rename(imgui_end_main_menu_bar) cvc::gl::ui::EndMainMenuBar;
%rename(imgui_begin_menu_bar) cvc::gl::ui::BeginMenuBar;
%rename(imgui_end_menu_bar) cvc::gl::ui::EndMenuBar;
%rename(imgui_begin_menu) cvc::gl::ui::BeginMenu;
%rename(imgui_end_menu) cvc::gl::ui::EndMenu;
%rename(imgui_menu_item) cvc::gl::ui::MenuItemClicked;
%rename(imgui_menu_item_toggle) cvc::gl::ui::MenuItemToggle;
%rename(imgui_collapsing_header) cvc::gl::ui::CollapsingHeaderOpen;
%rename(imgui_begin_disabled) cvc::gl::ui::BeginDisabled;
%rename(imgui_end_disabled) cvc::gl::ui::EndDisabled;
%rename(imgui_is_item_hovered) cvc::gl::ui::IsItemHovered;
%rename(imgui_set_tooltip) cvc::gl::ui::SetTooltipText;
%include "cvc/gl/ImGuiBinding.h"

// Group B wrappers (after the include, so the ignored declarations exist to call).
%inline %{
namespace pycvc {
// ScenePanel — SceneGraph shadow/chrome controls.
void ui_scene_panel(cvc::gl::SceneGraph &sg, bool own_window = true) {
  cvc::gl::ui::ScenePanel(sg, nullptr, own_window);
}
bool ui_scene_panel_open(cvc::gl::SceneGraph &sg, bool open, bool own_window = true) {
  bool o = open;
  cvc::gl::ui::ScenePanel(sg, &o, own_window);
  return o; // the (possibly window-close-toggled) open flag
}
// StageLightingPanel — the rig control surface.
void ui_stage_lighting_panel(cvc::gl::StageLighting &rig, bool own_window = true) {
  cvc::gl::ui::StageLightingPanel(rig, nullptr, own_window);
}
bool ui_stage_lighting_panel_open(cvc::gl::StageLighting &rig, bool open, bool own_window = true) {
  bool o = open;
  cvc::gl::ui::StageLightingPanel(rig, &o, own_window);
  return o;
}
// SceneMenuItems — the standard "Scene" menu contents (call inside a begun menu).
void ui_scene_menu_items(cvc::gl::SceneGraph &sg) {
  cvc::gl::ui::SceneMenuItems(sg, nullptr, nullptr);
}
// ..._open form: pass the two panel-open flags, get back (scene_open, lighting_open).
PyObject *ui_scene_menu_items_open(cvc::gl::SceneGraph &sg, bool scene_open, bool lighting_open) {
  bool s = scene_open, l = lighting_open;
  cvc::gl::ui::SceneMenuItems(sg, &s, &l);
  return Py_BuildValue("(OO)", s ? Py_True : Py_False, l ? Py_True : Py_False);
}
} // namespace pycvc
%}

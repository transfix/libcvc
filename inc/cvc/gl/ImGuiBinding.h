#ifndef CVC_GL_IMGUI_BINDING_H
#define CVC_GL_IMGUI_BINDING_H

#include <string>
#include <vector>

namespace cvc {
class app;

namespace gl {

// --------------------------
// cvc::gl::ui — state-bound widgets
// --------------------------
// ImGui widgets that read and write cvc::state instead of a local variable.
// This is what makes a cvcGL UI *scriptable*: the same value the slider shows
// can be set from a script, a config file, a debugger, or a replicated peer, and
// the widget follows — and dragging the widget writes back, so the rest of the
// app (and any state observer) sees it.
//
//     namespace u = cvc::gl::ui;
//     u::SliderDouble(app, "Move speed", "viewers.main.camera.settings.move_speed",
//                     1.0, 200.0);
//     u::Checkbox(app, "Shadows", "demo.shadows");
//     u::Combo(app, "Belief", "demo.swarm.belief", {"shared","grouped","private"});
//
// Each returns the usual ImGui "changed this frame" bool, so existing ImGui
// muscle memory transfers. Mix freely with plain ImGui::* calls.
//
// WRITE POLICY: the widget edits a cached copy every frame and commits to state
// only when the edit FINISHES (ImGui::IsItemDeactivatedAfterEdit), so dragging a
// slider is one state write, not sixty per second — state writes fan out to
// observers and (with replication) to peers, so per-frame writes are a real
// cost. Checkbox/MenuItem/Combo commit immediately (they are discrete).
//
// PATHS are cvc::state paths. A leading '/' is absolute; anything else is used
// as-is, so callers that want a scene-relative path should pass the full prefix
// (e.g. sg.statePrefix() + ".viewers.main.camera.settings.move_speed").
//
// All of these are no-ops returning false when libcvc is built without
// CVC_ENABLE_IMGUI, so callers need no #ifdef.
namespace ui {

// Float/double slider bound to `path` (created with `def` if missing).
bool SliderDouble(cvc::app &ctx, const char *label, const std::string &path, double lo, double hi,
                  double def = 0.0, const char *fmt = "%.3f");
bool SliderInt(cvc::app &ctx, const char *label, const std::string &path, int lo, int hi,
               int def = 0);
// Drag variants for unbounded values.
bool DragDouble(cvc::app &ctx, const char *label, const std::string &path, double speed = 1.0,
                double def = 0.0, const char *fmt = "%.3f");

// Boolean state ("1"/"0"), as a checkbox or a menu item with a tick.
bool Checkbox(cvc::app &ctx, const char *label, const std::string &path, bool def = false);
bool MenuItem(cvc::app &ctx, const char *label, const std::string &path, bool def = false);

// String state chosen from a fixed list — the state stores the OPTION TEXT
// (e.g. "grouped"), not an index, so it stays readable in the state tree.
bool Combo(cvc::app &ctx, const char *label, const std::string &path,
           const std::vector<std::string> &options, const std::string &def = std::string());

// Read-only display of whatever a state path currently holds (for values the
// app owns and the UI should only report).
void Text(cvc::app &ctx, const char *label, const std::string &path);

} // namespace ui
} // namespace gl
} // namespace cvc

#endif // CVC_GL_IMGUI_BINDING_H

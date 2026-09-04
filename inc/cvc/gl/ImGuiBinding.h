#ifndef CVC_GL_IMGUI_BINDING_H
#define CVC_GL_IMGUI_BINDING_H

#include <string>
#include <vector>

// SceneGraph is a global-scope type, not cvc::gl::SceneGraph — declaring it
// inside the namespace below would name a different, never-defined class.

namespace cvc {
class app;

namespace gl {
class SceneGraph;

class CameraController;
class StageLighting;

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

// ---- composite panels ------------------------------------------------------
// Ready-made control surfaces for the cvcGL objects an app is most likely to
// want on screen. They are ordinary ImGui content: call them inside a window you
// opened, or let them open their own.

// Full control surface for a StageLighting rig — preset picker, key/fill/back/
// wash intensities, key angle and cone, ambient, warmth, and a live count of the
// rig's SHADOW-CASTING lights (each one costs a whole scene depth re-render per
// bake, so it is worth having on screen while you tune).
//
// Every control writes the rig's cvc::state, so the same edits are reachable
// from a script, a config file or a replicated peer. Pass ownWindow=false to
// embed the controls in a window you already began.
void StageLightingPanel(StageLighting &rig, bool *open = nullptr, bool ownWindow = true);

// Control surface for the SceneGraph itself: shadows (on, map resolution, bake
// interval) and the diagnostic chrome (grid, origin axis, per-node bounding
// boxes, extent labels).
//
// Every demo was hand-rolling some subset of this — the same "Scene" menu with
// the same Shadows item, plus a local bool mirroring sg.shadowsEnabled() that
// could desync the moment setShadowsEnabled() returned false for want of a
// render target. The shadow controls here bind cvc::state paths directly
// (ShadowSettings::sceneStatePath), so a script, a config file or a replicated
// peer moves the same knobs; the chrome controls read their own state back
// rather than tracking a copy.
//
// Pass ownWindow=false to embed in a window you already began.
void ScenePanel(SceneGraph &sg, bool *open = nullptr, bool ownWindow = true);

// The standard "Scene" menu contents — shadows and the chrome toggles, plus an
// optional tick that opens ScenePanel/StageLightingPanel. Call INSIDE a menu you
// have begun, so a host can put it wherever its menu bar already is.
void SceneMenuItems(SceneGraph &sg, bool *scenePanelOpen = nullptr,
                    bool *lightingPanelOpen = nullptr);

// Camera settings for a menu — look sensitivity, move speed, inverted pitch and
// whether a primary drag pans. Call INSIDE a menu you have begun, same contract
// as SceneMenuItems.
//
// Deliberately NOT a mode picker. Entering the 2-D map view means FRAMING it —
// bounds, margin, the height to return to in 3-D — which only the app knows, so
// a mode item here would leave the camera in Map with the app still drawing its
// perspective framing, and the app's own "2-D map" tick reading the old value.
// Mode stays with whoever owns the framing.
//
// moveSpeedMax/moveSpeedDefault vary per app (a city walk and a continent flyover
// want different ranges), so they are parameters rather than baked in.
void CameraMenuItems(CameraController &cam, double moveSpeedMax = 400.0,
                     double moveSpeedDefault = 40.0);

} // namespace ui
} // namespace gl
} // namespace cvc

#endif // CVC_GL_IMGUI_BINDING_H

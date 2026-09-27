#ifndef CVC_GL_ARIADNE_SCENE_PANELS_H
#define CVC_GL_ARIADNE_SCENE_PANELS_H

// Register the built-in scene UI COMPOSITES as Ariadne custom widget types on an ImGui backend, so
// a .ari document places them DECLARATIVELY instead of the host hand-writing the ImGui draw calls
// every demo used to repeat (bunny_shadow.cpp:222-231). Each is a Kind::Custom widget whose type
// is:
//
//   type: scene_menu           -> cvc::gl::ui::SceneMenuItems(sg, &sceneOpen, &lightingOpen)
//                                  (place as a child of a `menu:` — it emits items INTO that menu)
//   type: scene_panel          -> cvc::gl::ui::ScenePanel(sg, &sceneOpen)          (its own window)
//   type: stage_lighting_panel -> cvc::gl::ui::StageLightingPanel(*rig, &lightingOpen) (own window;
//   needs rig)
//
// The panel open-flags are SHARED between scene_menu (which toggles them) and the panels (which
// read them), so a menu tick opens/closes its panel — the exact wiring the demos hand-rolled. Every
// control the composites draw writes cvc::state directly, so the panels need no `bind:`.
//
// Call AFTER realize_scene (so a realized StageLighting rig exists to pass) and BEFORE the render
// loop. `backend`, `sg`, and `*rig` must outlive the backend's draw callbacks (the registered draw
// fns hold them by reference / pointer). Pass rig = nullptr when the scene has no rig; then
// `stage_lighting_panel` is simply not registered and a document using it draws the core
// placeholder.
//
// ImGui-only. On a non-ImGui backend nothing here is registered; and built without CVC_ENABLE_IMGUI
// the cvc::gl::ui:: composites are inert stubs, so this still links and no-ops. A document that
// must also load on a terminal backend should gate these widgets with a `requires:`/`customs:`
// preflight.

namespace cvc {
namespace gl {
class ImGuiBackend;
class SceneGraph;
class StageLighting;

namespace ariadne {

void register_scene_panels(cvc::gl::ImGuiBackend &backend, cvc::gl::SceneGraph &sg,
                           cvc::gl::StageLighting *rig = nullptr);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_SCENE_PANELS_H

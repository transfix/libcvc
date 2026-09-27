// Ariadne scene UI composites as declarable custom widgets (roadmap §9/§16.1b). See scene_panels.h.
// The one place the reusable ScenePanel/StageLightingPanel/SceneMenuItems wiring lives, so a demo
// host adds all three to a .ari document with a single register_scene_panels() call.

#include <cvc/ariadne/backend.h> // CustomEdit
#include <cvc/ariadne/widget.h>
#include <cvc/gl/ImGuiBinding.h> // cvc::gl::ui::{SceneMenuItems,ScenePanel,StageLightingPanel}
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_panels.h>
#include <memory>
#include <string>

namespace cvc {
namespace gl {
namespace ariadne {

void register_scene_panels(ImGuiBackend &backend, SceneGraph &sg, StageLighting *rig) {
  // The panel open-flags, shared by all three composites so the menu's ticks drive the panels. Held
  // in a shared_ptr captured by each draw fn, so they outlive as long as the backend holds them and
  // are freed when the widgets are re-registered / the backend is destroyed.
  struct Flags {
    bool scene = false;
    bool lighting = false;
  };
  auto flags = std::make_shared<Flags>();

  // The "Scene" menu contents — place as a Custom child of a `menu:`; it emits items INTO the
  // begun menu. Passes the lighting flag only when there is a rig, so the lighting tick appears
  // exactly when stage_lighting_panel is available.
  backend.register_widget(
      "scene_menu", [&sg, flags, rig](const std::string &, const cvc::ariadne::Widget &) {
        cvc::gl::ui::SceneMenuItems(sg, &flags->scene, rig ? &flags->lighting : nullptr);
        return cvc::ariadne::CustomEdit{}; // handled set true by the backend
      });

  backend.register_widget("scene_panel",
                          [&sg, flags](const std::string &, const cvc::ariadne::Widget &) {
                            cvc::gl::ui::ScenePanel(sg, &flags->scene); // its own window
                            return cvc::ariadne::CustomEdit{};
                          });

  if (rig) {
    backend.register_widget("stage_lighting_panel",
                            [rig, flags](const std::string &, const cvc::ariadne::Widget &) {
                              cvc::gl::ui::StageLightingPanel(*rig, &flags->lighting);
                              return cvc::ariadne::CustomEdit{};
                            });
  }
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

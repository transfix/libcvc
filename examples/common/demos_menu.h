// demos_menu — the gallery's "Demos" menu, so every demo can open any other from its main menu.
//
//   if (ImGui::BeginMainMenuBar()) {
//     cvc_examples::demos_menu(app, "bunny_shadow");   // first, so it sits at the left of the bar
//     ...the demo's own menus...
//     ImGui::EndMainMenuBar();
//   }
//   while (!view.windowClosed() && !cvc_examples::demo_switch_requested(app)) { ... }
//
// The list is examples/web/demos.json's gallery order, built into the program. In the browser it is
// narrowed to the demos the gallery actually deployed (the page's gallery.js says which), and
// picking one moves the page to that demo's page (an .ari demo's is arigl's, ?doc=<doc>). Natively
// it lists the demos whose program sits beside this one (an .ari demo is arigl on its document,
// with the gallery's catalog so it has the Demos menu too), picking one starts it, and
// demo_switch_requested() turns true so this demo can close.
//
// What the menu keeps between frames (how to open each demo, and whether one was started) is the
// app's, like everything else (an app slot), so the demo's own app is passed to both calls.
//
// The caller's ImGui context must be current (the demos already adopt ImGuiOverlay's).
#pragma once

namespace cvc {
class app;
}

namespace cvc_examples {

// Draw the menu inside an open main menu bar, with `self` (a gallery slug) ticked.
void demos_menu(cvc::app &app, const char *self);

// True once demos_menu started another demo natively for this app; never true in the browser.
bool demo_switch_requested(cvc::app &app);

} // namespace cvc_examples

#ifndef CVC_GL_IMGUI_OVERLAY_H
#define CVC_GL_IMGUI_OVERLAY_H

#include <functional>
#include <memory>

struct ImGuiContext; // Dear ImGui (global namespace) — forward-declared so this
                     // header stays imgui-free for consumers that don't draw UI.

namespace cvc {
namespace gl {
class SceneRenderer;

// --------------
// ImGuiOverlay
// --------------
// Dear ImGui inside a cvcGL viewer: menus, panels, sliders, plots — real UI, in
// the same code path natively and in WebAssembly.
//
// USAGE is one object plus one callback. Construct it on a SceneRenderer and set
// a draw function; the function runs once per rendered frame, inside VTK's own
// render pass, and you write plain immediate-mode ImGui in it:
//
//     cvc::gl::ImGuiOverlay ui(view);
//     ui.setDrawCallback([&] {
//       if (ImGui::BeginMainMenuBar()) {
//         if (ImGui::BeginMenu("View")) {
//           ImGui::MenuItem("Trails", nullptr, &showTrails);
//           ImGui::EndMenu();
//         }
//         ImGui::EndMainMenuBar();
//       }
//       ImGui::Begin("Swarm");
//       ImGui::SliderFloat("speed", &speed, 0.1f, 4.0f);
//       ImGui::Text("arrived %ld/%d", arrived, N);
//       ImGui::End();
//     });
//
// HOW IT HOOKS IN (the vtkDearImGuiInjector pattern, BSD-3, by Jaswant
// Panchumarti — the approach is his; this is a cvcGL-native reimplementation):
//   * render: observers on the render window's StartEvent (ImGui NewFrame) and
//     RenderEvent (draw the ImGui draw lists). Because the overlay is drawn
//     INSIDE VTK's render, it appears in offscreen captures too — no separate
//     compositing path.
//   * input: an interceptor observer at priority 1.0 on the CURRENT
//     vtkInteractorStyle, so events route VTK interactor -> ImGui -> the style.
//     ImGui does NOT take the interactor's single style slot, so
//     CameraController keeps working exactly as before; when ImGui wants the
//     mouse/keyboard (io.WantCaptureMouse / WantCaptureKeyboard) the event is
//     swallowed and the camera does not see it. That is why clicking a widget
//     doesn't also orbit the scene.
//   * backend: stock imgui_impl_opengl3 only. There is no ImGui "platform"
//     backend (no GLFW/SDL) — ImGuiIO is fed from the VTK interactor directly,
//     which is what makes the WebAssembly build work: VTK owns the canvas.
//
// WASM: the same code path. imgui_impl_opengl3 is compiled with
// IMGUI_IMPL_OPENGL_ES3 under Emscripten (WebGL2). Mouse and rendering work;
// KEYBOARD input does not reach the browser build today, so design UI to be
// mouse-drivable (menus, sliders, buttons — not text entry).
//
// Requires libcvc built with CVC_ENABLE_IMGUI=ON; without it the class still
// exists but is inert (enabled() == false), so consuming code needs no #ifdef.
class CameraController;

class ImGuiOverlay {
public:
  explicit ImGuiOverlay(SceneRenderer &viewer);
  ~ImGuiOverlay(); // detaches observers and shuts ImGui down

  ImGuiOverlay(const ImGuiOverlay &) = delete;
  ImGuiOverlay &operator=(const ImGuiOverlay &) = delete;

  // Your UI. Called once per rendered frame between ImGui::NewFrame() and
  // Render(); just call ImGui::* inside it.
  void setDrawCallback(std::function<void()> draw);

  // The overlay's Dear ImGui context (nullptr if libcvc was built without
  // CVC_ENABLE_IMGUI or setup failed). The overlay CREATES the context inside
  // libcvc, so a program in ANOTHER binary module (a demo .exe that links its own
  // static copy of imgui — the usual case, because imgui exports no symbols) has a
  // SEPARATE, uninitialised ImGui context: its ImGui::* calls would dereference a
  // null GImGui and crash. Adopt this context once, right after construction, so
  // the whole process shares one:
  //     ImGui::SetCurrentContext(ui.imguiContext());
  // (Only needed for raw ImGui::* calls from your own module; the cvc::gl::ui::*
  // widgets already run inside libcvc's context.)
  ImGuiContext *imguiContext() const;

  // Cooperate with a fly/orbit camera. Optional, but fixes two real problems:
  //  * while the camera holds POINTER CAPTURE (Quake fly warps the cursor to the
  //    centre every frame) the absolute mouse position is meaningless, so ImGui
  //    hit-testing would be garbage and stray clicks would hit random widgets —
  //    the overlay stops consuming the mouse entirely until capture is released;
  //  * when the UI TAKES the keyboard, any movement key still held is released
  //    on the camera, otherwise a key pressed before the handoff and released
  //    after it stays "held" and the camera drifts forever.
  // The overlay keeps only a raw pointer; the camera must outlive it (declare
  // the camera first, as in every cvcGL example).
  void attachCamera(CameraController &cam);

  // Master switch (the overlay stops drawing AND stops capturing input).
  void setVisible(bool on);
  bool visible() const;

  // False when libcvc was built without CVC_ENABLE_IMGUI, or setup failed
  // (e.g. no GL context yet) — every method is then a safe no-op.
  bool enabled() const;

  // True while ImGui is consuming the pointer / keys — a host loop can use these
  // to suppress its own hotkeys (the CAMERA is gated automatically).
  bool wantsMouse() const;
  bool wantsKeyboard() const;

  // ---- scale, for fingers --------------------------------------------------
  // Multiply the whole UI — fonts AND widget metrics (padding, scrollbars, grab
  // sizes, hit slop). Defaults to 1.0 with a mouse and 2.0 on a touch device
  // (detected in the browser via ontouchstart / maxTouchPoints), because widgets
  // sized for a cursor are unusable with a fingertip.
  //
  // Text stays CRISP when scaled: ImGui 1.92 re-bakes glyphs at the final pixel
  // size instead of stretching one baked size. Prefer a few discrete values
  // (1.0 / 1.5 / 2.0) over a continuous slider — each distinct size bakes and
  // uploads a new glyph atlas, which in the browser also costs a staging copy —
  // and never animate it.
  void setUiScale(float scale);
  float uiScale() const;

  // Touch mode changes INPUT handling as well as size. A lifted finger hovers
  // nothing, unlike a mouse that stays where it was left, so on release the
  // pointer is moved off-screen. Without that the last touched widget stays
  // hovered forever, ImGui keeps reporting that it wants the mouse, and the
  // camera never receives another event — one tap locks it out permanently.
  void setTouchMode(bool on);
  bool touchMode() const;

  // ---- the floating show/hide button ---------------------------------------
  // A large translucent circular button in the bottom-right corner that hides
  // and restores your panels, so the UI never has to sit on the scene when you
  // just want to look at it. On by default, and on a device with no keyboard it
  // is the only practical way to get the UI back.
  void setToggleButtonEnabled(bool on);
  bool toggleButtonEnabled() const;
  // Whether the panels (your draw callback) are currently shown. The button
  // flips this; you may drive it yourself as well.
  void setPanelsOpen(bool on);
  bool panelsOpen() const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_IMGUI_OVERLAY_H

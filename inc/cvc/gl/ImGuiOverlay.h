#ifndef CVC_GL_IMGUI_OVERLAY_H
#define CVC_GL_IMGUI_OVERLAY_H

#include <functional>
#include <memory>
#include <string>

struct ImGuiContext; // Dear ImGui (global namespace) — forward-declared so this
                     // header stays imgui-free for consumers that don't draw UI.
struct ImGuiIO;

namespace cvc {
class app;
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
// IMGUI_IMPL_OPENGL_ES3 under Emscripten (WebGL2). Mouse and rendering work.
// KEYBOARD events from the VTK interactor are now translated to ImGui (text
// entry, editing keys, Ctrl-shortcuts — see the intercept() key handling), so
// text entry works natively. In the BROWSER the canvas's interactor (SDL's, or
// VTK's canvas window's) takes the DOM key events typed into the canvas, which
// therefore has to hold the focus (tabindex).
//
// A PHONE has no keyboard to send them, and a browser raises its own only for
// a focused text field, which a canvas is not. So when a text field is tapped
// (a touch, not a mouse click) the overlay focuses a hidden <input> it adds to
// the page, at the field's caret, and types what the system keyboard puts there
// into ImGui; the field's deactivation blurs it, which puts the keyboard away.
// While it has the focus the canvas does not, so neither VTK nor SDL sees those
// keys and typing a name never walks. With a mouse nothing changes.
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

  // ---- UI font glyph coverage (roadmap §17.4 / §17.7 phase 2) ---------------
  // The overlay's default font is a scalable ASCII face (ProggyForever) covering
  // only Basic Latin, so accented Latin, Cyrillic, Greek, CJK and symbols render
  // as boxes. Point it at a real font — ImGui 1.92 loads that font's glyphs on
  // demand — to fix that regardless of which text a UI shows. The settings are the
  // app's: they reach the overlays of viewers on `app`, and no other app's. Call
  // these BEFORE constructing an overlay (the font is loaded when the ImGui context
  // is built).
  // When the overlay builds its context it resolves a font in this order:
  //   1. setUiFontMemory() if set (for wasm / an embedded font), else
  //   2. setUiFontPath(), else 3. the CVC_UI_FONT environment variable, else
  //   4. a search of common system fonts (DejaVu / Noto / Arial Unicode / …),
  //   5. the bundled DejaVu Sans Mono (share/libcvc/fonts: a browser build's font),
  //   6. fall back to the ASCII face.
  // An unfound or unloadable font is a logged fallback, never a hard failure.
  // Under that font it merges the fallbacks (setUiFontFallbacks): DejaVu Sans Mono's symbols, and
  // emoji in colour (the bundled Twemoji, else the system's colour emoji font) where ImGui draws
  // with FreeType, which a build with the cvcpkg imgui 1.92.9+cvc.4 and FreeType does.
  static void setUiFontPath(cvc::app &app, const std::string &ttfPath);

  // Whether the overlay draws with FreeType and merges the fallback fonts (default on). Off:
  // ImGui's own stb_truetype and the UI font alone, as before colour emoji, which a golden-image
  // test that pins its pixels wants. Call it before constructing an overlay.
  static void setUiFontFallbacks(cvc::app &app, bool on);

  // Load `app`'s UI font and its fallbacks into `io.Fonts`, as an overlay does when it builds its
  // context: for an ImGui context of a host's own, or a test's. Without imgui, nothing.
  static void loadUiFonts(cvc::app &app, ImGuiIO &io);

  // Supply the UI font as an in-memory TTF (a build with no system fonts — wasm —
  // or to embed one). The buffer must outlive every overlay. Pass compressed=true
  // for data produced by ImGui's binary_to_compressed_c.
  static void setUiFontMemory(cvc::app &app, const void *ttf, int len, bool compressed = false);

  // Your UI. Called once per rendered frame between ImGui::NewFrame() and
  // Render(); just call ImGui::* inside it.
  void setDrawCallback(std::function<void()> draw);

  // Annotations that belong to the SCENE rather than the UI -- e.g. names over
  // 3D objects. Called every rendered frame like the draw callback, but also
  // while the panels are hidden (the toggle, a phone's default), and BEFORE it,
  // so draw into ImGui::GetBackgroundDrawList() and the UI's windows cover it.
  void setSceneDrawCallback(std::function<void()> draw);

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
  // (e.g. no GL context yet) — every method is then a safe no-op. Also false
  // once ImGui's GL backend has failed to initialise on the render window's
  // context (checked at the first render: e.g. VTK's EGL fallback with no X
  // display, which imgui's GLX-based loader cannot reach).
  bool enabled() const;

  // True while ImGui is consuming the pointer / keys — a host loop can use these
  // to suppress its own hotkeys (the CAMERA is gated automatically).
  bool wantsMouse() const;
  bool wantsKeyboard() const;

  // ---- scale, for fingers --------------------------------------------------
  // The UI SIZE: multiply the whole UI — fonts AND widget metrics (padding,
  // scrollbars, grab sizes, hit slop) — and the Ariadne backend's authored px
  // (a window's size:, a column's width) with it. 1.0 is the UI at its own size
  // in the display's logical pixels (CSS pixels in a browser): the overlay
  // multiplies it by the pixel ratio below, so a phone's dense screen gets the
  // same physical size as a laptop's. Defaults to 1.0, with a mouse and on a
  // touch device alike (detected in the browser via ontouchstart /
  // maxTouchPoints; touch mode widens the hit areas instead). It is
  // the state key "<scene>.viewers.<viewer>.ui.scale", which AppRuntime's
  // Controls window offers as "UI size" (arigl). A change takes effect at the
  // next frame and scales the open windows with it.
  //
  // Text stays CRISP when scaled: ImGui 1.92 re-bakes glyphs at the final pixel
  // size instead of stretching one baked size. Prefer a few discrete values
  // (1.0 / 1.25 / 1.5 / 2.0) over a continuous slider — each distinct size bakes
  // and uploads a new glyph atlas, which in the browser also costs a staging
  // copy — and never animate it.
  void setUiScale(float scale);
  float uiScale() const;

  // Device pixels per logical pixel. The browser build sizes its canvas in
  // device pixels (VTK: CSS size x window.devicePixelRatio, about 3 on a phone),
  // so without this the UI would come out a third of its size there. Read from
  // devicePixelRatio every frame in the browser (browser zoom changes it, and
  // the UI follows), 1.0 natively. A positive value overrides it; 0 goes back to
  // the display's.
  void setPixelRatio(float ratio);
  float pixelRatio() const;

  // Touch mode changes INPUT handling as well as size. A lifted finger hovers
  // nothing, unlike a mouse that stays where it was left, so on release the
  // pointer is moved off-screen. Without that the last touched widget stays
  // hovered forever, ImGui keeps reporting that it wants the mouse, and the
  // camera never receives another event — one tap locks it out permanently.
  // It also widens every widget's hit area, raises the drag threshold, and moves
  // windows by their title bar only (a finger that slips off a widget must not
  // carry its window away).
  void setTouchMode(bool on);
  bool touchMode() const;

  // ---- scrolling with a finger ------------------------------------------------
  // In touch mode a finger's drag across a window's content scrolls it, and a
  // fling leaves it gliding until friction stops it, as a phone's lists do
  // (cvc::gl::TouchScroll says which drags count: a slider, a field being
  // edited, a title bar or a scene view keeps its own). On by default; the
  // state key "<scene>.viewers.<viewer>.ui.touch_scroll". The friction is per
  // second: a glide's speed falls as exp(-friction * t), so a fling covers its
  // speed / friction. 2 (the default) is iOS's normal deceleration, 10 its fast
  // one, 0 glides on to the end. The state key ".ui.scroll_friction", which a
  // document may set or bind a slider to, and AppRuntime's Controls window
  // offers in touch mode (arigl).
  void setTouchScrollEnabled(bool on);
  bool touchScrollEnabled() const;
  void setScrollFriction(float perSecond);
  float scrollFriction() const;

  // ---- frame clock ----------------------------------------------------------
  // ImGui's per-frame delta (io.DeltaTime: hover delays, the text cursor blink,
  // modal dimming) comes from the wall clock by default. A positive value pins it
  // instead, so a frame sequence draws the same pixels on every run — what a
  // golden-image test needs. 0 (the default) returns to the wall clock.
  void setFixedDeltaTime(float seconds);
  float fixedDeltaTime() const;

  // ---- the floating show/hide button ---------------------------------------
  // A large translucent circular button in the bottom-right corner that hides
  // and restores your panels, so the UI never has to sit on the scene when you
  // just want to look at it. On by default, and on a device with no keyboard it
  // is the only practical way to get the UI back, so it stays above the windows
  // even when one is dragged over its corner. The panels start hidden when the
  // view's short side is under 500 logical pixels (a phone), so a 3D scene is not
  // buried, unless setPhoneHidesPanels(false).
  void setToggleButtonEnabled(bool on);
  bool toggleButtonEnabled() const;
  // Whether the panels (your draw callback) are currently shown. The button
  // flips this; you may drive it yourself as well.
  void setPanelsOpen(bool on);
  bool panelsOpen() const;
  // Whether a phone's view starts with the panels hidden (on by default). A UI-only document has
  // no scene for its panels to bury, so AriRuntime turns it off for a document without one.
  // Turned off after the first frame hid them, it shows them again, unless the user (or
  // setPanelsOpen) has chosen since.
  void setPhoneHidesPanels(bool on);
  bool phoneHidesPanels() const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_IMGUI_OVERLAY_H

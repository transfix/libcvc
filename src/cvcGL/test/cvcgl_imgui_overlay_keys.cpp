// ImGuiOverlay's keys from VTK's own window: a field's undo and select all with the control flag,
// with ImGui's Mac behaviours (io.ConfigMacOSXBehaviors) forced on and off, so it runs on any
// platform. VTK's window has one control flag, and its Cocoa view sets it for Cmd or Ctrl. ImGui
// on a Mac swaps Cmd and Ctrl as keys come in, so there the flag must go in as Super to come out
// as ImGui's shortcut key. Sent as Ctrl, Cmd+Z did nothing and typed a "z". The keys go through
// the interactor as VTK's windows send them:
//  - Cocoa: Cmd's press is a Control_L with the flag set; Cmd+Z types "z" (a CharEvent, which
//    ImGui must drop); Ctrl+Z types ^Z, which the overlay drops itself;
//  - X11 and Win32: Ctrl+Z, with ^Z.
// The interactor is never initialized and the overlay never renders: the test runs ImGui's frames
// on the overlay's context itself, with no window and no GL.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/ImGuiBackend.h> // setImGuiContext: cvcGL's imgui sees this context
#include <memory>
#include <string>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

#ifdef CVC_ENABLE_IMGUI

// One overlay on an offscreen view, with an interactor of its own (an offscreen view has none),
// and a field that has the keyboard.
struct Keys {
  cvc::app app;
  cvc::gl::SceneGraph sg{app, "overlay_keys"};
  cvc::gl::SceneRenderer view{sg, 320, 240, /*offscreen=*/true, "main"};
  vtkSmartPointer<vtkRenderWindowInteractor> iren;
  std::unique_ptr<cvc::gl::ImGuiOverlay> ui;
  char buf[64] = {};
  bool focus = true;
  bool ctrl = false, super = false; // ImGui's io.KeyCtrl / io.KeySuper at the last frame

  explicit Keys(bool mac) {
    iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(view.renderWindow()); // before the overlay, which looks for it
    ui = std::make_unique<cvc::gl::ImGuiOverlay>(view);
    ui->setTouchMode(false);
    ui->setVisible(true);
    cvc::gl::ImGuiBackend::setImGuiContext(ui->imguiContext());
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigMacOSXBehaviors = mac;
    io.DisplaySize = ImVec2(320.0f, 240.0f);
    // As the overlay's renderer does: the fonts (added at size 0) are baked as they are drawn.
    // No GL: nothing uploads them.
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    frame();
  }
  ~Keys() { cvc::gl::ImGuiBackend::setImGuiContext(nullptr); }

  void frame() {
    ImGuiIO &io = ImGui::GetIO();
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(300.0f, 100.0f));
    ImGui::Begin("Keys");
    if (focus) {
      ImGui::SetKeyboardFocusHere();
      focus = false;
    }
    ImGui::InputText("##field", buf, sizeof buf);
    ImGui::End();
    ImGui::Render();
    ctrl = io.KeyCtrl;
    super = io.KeySuper;
  }

  // A key event as VTK's window sends it: KEYSYM with the control flag CTRL and the character
  // CODE, which a press also sends as a CharEvent when it has one. Then a frame.
  void send(bool press, const char *keysym, int ctrlFlag, char code) {
    iren->SetKeyEventInformation(ctrlFlag, 0, code, 1, keysym);
    iren->InvokeEvent(press ? vtkCommand::KeyPressEvent : vtkCommand::KeyReleaseEvent, nullptr);
    if (press && code != '\0')
      iren->InvokeEvent(vtkCommand::CharEvent, nullptr);
    frame();
  }
  void type(const char *text) {
    for (const char *p = text; *p; ++p) {
      const char sym[2] = {*p, '\0'};
      send(true, sym, 0, *p);
      send(false, sym, 0, *p);
    }
  }
  // The control flag held over a letter: Control_L's press (with the flag set, as Cocoa's
  // flagsChanged has it), the letter, typing CODE, and Control_L's release. Records whether
  // ImGui had its Ctrl and its Super while the letter was down.
  bool heldCtrl = false, heldSuper = false;
  void chord(const char *letter, char code) {
    send(true, "Control_L", 1, '\0');
    send(true, letter, 1, code);
    heldCtrl = ctrl;
    heldSuper = super;
    send(false, letter, 1, code);
    send(false, "Control_L", 0, '\0');
  }
  std::string text() const { return buf; }
};

void run(bool mac) {
  const std::string on = mac ? " (Mac behaviours on)" : " (Mac behaviours off)";
  Keys k(mac);
  if (!k.ui->enabled()) {
    chk(false, "the overlay is enabled" + on);
    return;
  }
  k.type("abc");
  chk(k.text() == "abc", "typed characters reach the field: '" + k.text() + "'" + on);

  if (mac) {
    // Cmd+Z: the flag, and a "z" the field must not take.
    k.chord("z", 'z');
    chk(k.heldCtrl && !k.heldSuper,
        "VTK's control flag is ImGui's shortcut key (io.KeyCtrl), not Super" + on);
    chk(k.text() == "ab",
        "Cmd+Z undoes the last character, typing no 'z': '" + k.text() + "'" + on);
    // Cocoa sets the flag for Ctrl as well: Ctrl+Z, typing ^Z, undoes too.
    k.chord("z", '\x1a');
    chk(k.text() == "a", "Ctrl+Z, the same flag, undoes too: '" + k.text() + "'" + on);
    // Cmd+A selects all, so the next character replaces it.
    k.type("bc");
    k.chord("a", 'a');
    k.type("x");
    chk(k.text() == "x", "Cmd+A selects all, typing no 'a': '" + k.text() + "'" + on);
  } else {
    k.chord("z", '\x1a');
    chk(k.heldCtrl && !k.heldSuper, "VTK's control flag is ImGui's Ctrl" + on);
    chk(k.text() == "ab", "Ctrl+Z undoes the last character: '" + k.text() + "'" + on);
    k.chord("a", '\x01');
    k.type("x");
    chk(k.text() == "x", "Ctrl+A selects all: '" + k.text() + "'" + on);
  }

  // What the overlay sent before: ImGuiMod_Ctrl, which ImGui's Mac behaviours turn into Super, so
  // Z does not undo there (and does elsewhere).
  k.type("yz");
  ImGuiIO &io = ImGui::GetIO();
  io.AddKeyEvent(ImGuiMod_Ctrl, true);
  io.AddKeyEvent(ImGuiKey_Z, true);
  k.frame();
  io.AddKeyEvent(ImGuiKey_Z, false);
  io.AddKeyEvent(ImGuiMod_Ctrl, false);
  k.frame();
  chk(k.text() == (mac ? "xyz" : "xy"), std::string("ImGuiMod_Ctrl+Z itself ") +
                                            (mac ? "does not undo" : "undoes") + ": '" + k.text() +
                                            "'" + on);
}

#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  run(/*mac=*/true);
  run(/*mac=*/false);
  std::printf("%s\n", failures ? "FAILED" : "all passed");
  return failures ? 1 : 0;
#endif
}

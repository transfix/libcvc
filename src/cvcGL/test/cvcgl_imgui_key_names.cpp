// The keys ImGuiOverlay hands ImGui, under both names VTK gives them: its own windows' (X11's:
// "Return", "BackSpace", "Left") and a browser's (KeyboardEvent.key: "Enter", "Backspace",
// "ArrowLeft"). In a browser Enter and Backspace never reached a text field before the second set.
#include <cstdio>
#include <string>
#ifdef CVC_ENABLE_IMGUI
#include "../ImGuiKeyNames.h"
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  using cvc::gl::imguiKeyForVtkKeySym;
  struct Pair {
    const char *x11, *browser;
    ImGuiKey key;
    const char *what;
  };
  const Pair pairs[] = {
      {"Return", "Enter", ImGuiKey_Enter, "Enter"},
      {"BackSpace", "Backspace", ImGuiKey_Backspace, "Backspace"},
      {"Left", "ArrowLeft", ImGuiKey_LeftArrow, "Left"},
      {"Right", "ArrowRight", ImGuiKey_RightArrow, "Right"},
      {"Up", "ArrowUp", ImGuiKey_UpArrow, "Up"},
      {"Down", "ArrowDown", ImGuiKey_DownArrow, "Down"},
      {"Prior", "PageUp", ImGuiKey_PageUp, "Page Up"},
      {"Next", "PageDown", ImGuiKey_PageDown, "Page Down"},
      {"space", " ", ImGuiKey_Space, "Space"},
      {"Shift_L", "Shift", ImGuiKey_LeftShift, "Shift"},
      {"Control_L", "Control", ImGuiKey_LeftCtrl, "Ctrl"},
      {"Alt_L", "Alt", ImGuiKey_LeftAlt, "Alt"},
      {"Escape", "Escape", ImGuiKey_Escape, "Escape"},
      {"Tab", "Tab", ImGuiKey_Tab, "Tab"},
      {"Delete", "Delete", ImGuiKey_Delete, "Delete"},
      {"Home", "Home", ImGuiKey_Home, "Home"},
  };
  for (const Pair &p : pairs) {
    chk(imguiKeyForVtkKeySym(p.x11) == p.key,
        std::string(p.what) + " from VTK's own window (\"" + p.x11 + "\")");
    chk(imguiKeyForVtkKeySym(p.browser) == p.key,
        std::string(p.what) + " from a browser (\"" + p.browser + "\")");
  }
  chk(imguiKeyForVtkKeySym("a") == ImGuiKey_A && imguiKeyForVtkKeySym("A") == ImGuiKey_A,
      "a letter, either case");
  chk(imguiKeyForVtkKeySym("NoSuchKey") == ImGuiKey_None &&
          imguiKeyForVtkKeySym("") == ImGuiKey_None &&
          imguiKeyForVtkKeySym(nullptr) == ImGuiKey_None,
      "an unknown name, an empty one and none map to no key");
  std::printf("cvcgl_imgui_key_names: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

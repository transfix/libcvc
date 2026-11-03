// ImGuiKeyNames -- a VTK key event's name as a Dear ImGui key (internal to cvcGL).
//
// ImGuiOverlay feeds ImGui the keys a viewer's interactor reports. VTK's own windows name them as
// X11 does ("Return", "BackSpace", "Prior"/"Next" for page up/down, "Control_L", ...); in a browser
// VTK's interactor passes the page's KeyboardEvent.key ("Enter", "Backspace", "ArrowLeft",
// "PageUp", "Control", " "), so both spellings are here. Only the keys ImGui needs for text
// editing, navigation and shortcuts are mapped; anything else is ImGuiKey_None. Printable text does
// not come through here: it arrives as a CharEvent, fed with io.AddInputCharacter.
#ifndef CVC_GL_IMGUI_KEY_NAMES_H
#define CVC_GL_IMGUI_KEY_NAMES_H

#include <cstring>
#include <imgui.h>

namespace cvc {
namespace gl {

inline ImGuiKey imguiKeyForVtkKeySym(const char *sym) {
  if (!sym || !sym[0])
    return ImGuiKey_None;
  // Single-character syms: letters, digits and the punctuation VTK reports
  // literally ("a", "5", "-").
  if (!sym[1]) {
    const char c = sym[0];
    if (c >= 'a' && c <= 'z')
      return static_cast<ImGuiKey>(ImGuiKey_A + (c - 'a'));
    if (c >= 'A' && c <= 'Z')
      return static_cast<ImGuiKey>(ImGuiKey_A + (c - 'A'));
    if (c >= '0' && c <= '9')
      return static_cast<ImGuiKey>(ImGuiKey_0 + (c - '0'));
    switch (c) {
    case ' ': // the browser's name for it (VTK's own windows say "space")
      return ImGuiKey_Space;
    case '-':
      return ImGuiKey_Minus;
    case '=':
      return ImGuiKey_Equal;
    case '[':
      return ImGuiKey_LeftBracket;
    case ']':
      return ImGuiKey_RightBracket;
    case '\\':
      return ImGuiKey_Backslash;
    case ';':
      return ImGuiKey_Semicolon;
    case '\'':
      return ImGuiKey_Apostrophe;
    case '`':
      return ImGuiKey_GraveAccent;
    case ',':
      return ImGuiKey_Comma;
    case '.':
      return ImGuiKey_Period;
    case '/':
      return ImGuiKey_Slash;
    default:
      return ImGuiKey_None;
    }
  }
  struct Entry {
    const char *sym;
    ImGuiKey key;
  };
  // VTK's own windows name keys as X11 does ("Return", "BackSpace", "Left"). In a browser VTK's
  // interactor passes the page's KeyboardEvent.key instead ("Enter", "Backspace", "ArrowLeft"):
  // without those names Enter and Backspace never reached a text field there (typed characters
  // did, through the CharEvent).
  static const Entry kMap[] = {
      {"Return", ImGuiKey_Enter},
      {"Enter", ImGuiKey_Enter},
      {"Backspace", ImGuiKey_Backspace},
      {"ArrowLeft", ImGuiKey_LeftArrow},
      {"ArrowRight", ImGuiKey_RightArrow},
      {"ArrowUp", ImGuiKey_UpArrow},
      {"ArrowDown", ImGuiKey_DownArrow},
      {"PageUp", ImGuiKey_PageUp},
      {"PageDown", ImGuiKey_PageDown},
      {"Shift", ImGuiKey_LeftShift},
      {"Control", ImGuiKey_LeftCtrl},
      {"Alt", ImGuiKey_LeftAlt},
      {"Meta", ImGuiKey_LeftSuper},
      {"KP_Enter", ImGuiKey_KeypadEnter},
      {"Escape", ImGuiKey_Escape},
      {"Tab", ImGuiKey_Tab},
      {"BackSpace", ImGuiKey_Backspace},
      {"Delete", ImGuiKey_Delete},
      {"Insert", ImGuiKey_Insert},
      {"space", ImGuiKey_Space},
      {"Left", ImGuiKey_LeftArrow},
      {"Right", ImGuiKey_RightArrow},
      {"Up", ImGuiKey_UpArrow},
      {"Down", ImGuiKey_DownArrow},
      {"Home", ImGuiKey_Home},
      {"End", ImGuiKey_End},
      {"Prior", ImGuiKey_PageUp},
      {"Next", ImGuiKey_PageDown},
      {"Shift_L", ImGuiKey_LeftShift},
      {"Shift_R", ImGuiKey_RightShift},
      {"Control_L", ImGuiKey_LeftCtrl},
      {"Control_R", ImGuiKey_RightCtrl},
      {"Alt_L", ImGuiKey_LeftAlt},
      {"Alt_R", ImGuiKey_RightAlt},
      {"Super_L", ImGuiKey_LeftSuper},
      {"Super_R", ImGuiKey_RightSuper},
      {"F1", ImGuiKey_F1},
      {"F2", ImGuiKey_F2},
      {"F3", ImGuiKey_F3},
      {"F4", ImGuiKey_F4},
      {"F5", ImGuiKey_F5},
      {"F6", ImGuiKey_F6},
      {"F7", ImGuiKey_F7},
      {"F8", ImGuiKey_F8},
      {"F9", ImGuiKey_F9},
      {"F10", ImGuiKey_F10},
      {"F11", ImGuiKey_F11},
      {"F12", ImGuiKey_F12},
  };
  for (const Entry &e : kMap)
    if (std::strcmp(sym, e.sym) == 0)
      return e.key;
  return ImGuiKey_None;
}

} // namespace gl
} // namespace cvc

#endif // CVC_GL_IMGUI_KEY_NAMES_H

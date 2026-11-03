// Rebindable keys (§4.6). A document names what its keys do in a `keys:` block, and a host adds
// bindings of its own (arigl: the camera's). Each binding's keys live in a state key, so the Keys
// window AppRuntime puts in every document it mounts -- or anything else -- can change them, and
// Runtime::post_input runs a binding's `on` when one of its keys is pressed.
//
//   keys:
//     next:
//       label: Next item
//       key: [Right, PageDown]        # one key, or a list
//       on: (state-set "item" (+ (int (state-get "item")) 1))
#ifndef CVC_ARIADNE_KEYS_H
#define CVC_ARIADNE_KEYS_H

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

struct KeyBinding {
  std::string id;                // its name; a document's keys live at keys.<id>
  std::string label;             // what the Keys window calls it ("" = the id)
  std::string group;             // the Keys window's heading for it ("" = the document's own)
  std::vector<std::string> keys; // its default keys, e.g. {"Right", "PageDown"}
  std::string on;   // what a press does: a program or a host event ("" = something else reads the
                    // keys, as the camera reads its own)
  std::string path; // the state key holding its keys ("" = keys.<id>; a leading '/' is absolute)
};

// The pointer and camera settings a host's camera reads, for the Controls window to change beside
// the keys (AppRuntime::set_host_mouse). Each is a state path, as KeyBinding::path is (a leading
// '/' is absolute); "" leaves its row out. The window writes what its widgets write: the look as
// the string "0" or "1", the inverts as 0 or 1, the sensitivity and distance as doubles; the host's
// camera reads them, and seeds them (arigl's: CameraController's settings.* and orbit.distance).
struct MouseControls {
  std::string look;        // 1: mouse look (once the scene is clicked the mouse turns the view,
                           // Escape gives the mouse back); 0: drag to look, click the scene
  std::string steer; // 1: in drag to look, a drag on the scene steers (what the host walks faces
                     // the point under the pointer) and the right button turns the view instead
  std::string steer_label = "Drag to steer"; // its checkbox, in the host's words
  std::string sensitivity; // degrees of turn per pixel of motion
  double sensitivity_lo = 0.02, sensitivity_hi = 1.0;
  std::string invert_y; // 1: moving the mouse up looks down
  std::string invert_x; // 1: moving the mouse right turns left
  std::string distance; // the camera's distance from what it orbits (the wheel changes it too)
  double distance_lo = 1.5, distance_hi = 40.0;
  std::string note; // a line under them, what the look does here ("" = none)
};

// The Controls window's id, its open state, and the state key naming the binding whose Change
// button is waiting for a key ("" = none; it counts only while the window is open). Under the
// document's prefix, beside the window's other tree.<id> state, so no binding id can collide with
// them. (Its id is from when it held only the keys.)
inline constexpr const char *kKeysWindow = "ariadne_keys";
inline constexpr const char *kKeysWindowOpen = "tree.ariadne_keys.open";
inline constexpr const char *kKeyCapture = "tree.ariadne_keys.capture";

// Where a binding's keys live, before resolving against the document's prefix.
inline std::string key_binding_path(const KeyBinding &b) {
  return b.path.empty() ? "keys." + b.id : b.path;
}

// What the Keys window shows for a binding: its keys, or "press a key..." while its Change waits.
// The runtime keeps it (Runtime::drain), so the window's rows are bound texts and cost no reactive
// evaluation.
inline std::string key_shown_path(const KeyBinding &b) {
  return std::string("tree.") + kKeysWindow + ".shown." + b.id;
}
inline constexpr const char *kKeyPrompt = "press a key...";

// A binding's keys as stored ("Right, PageDown"), and back. The comma key is stored as "Comma"
// (stored_key_name), so a comma always separates.
std::string join_keys(const std::vector<std::string> &keys);
std::vector<std::string> split_keys(std::string_view stored);

// One spelling for a key whoever names it, to compare a pressed key with a binding: SDL's names
// ("Left Shift", "Space", "PageUp", ",", "Keypad 1") and VTK's X11 keysyms ("Shift_L", "space",
// "Prior", "comma", "KP_1") come out the same, and letters fold to lower case. Not for showing.
std::string canonical_key(std::string_view name);

// How a pressed key is stored in a binding: its name, but "Comma" for "," (see join_keys); ""
// for a key with no name, which cannot be bound.
std::string stored_key_name(std::string_view pressed);

// True when `key` is one of the keys stored at a binding (`stored`, as join_keys writes it).
bool key_matches(std::string_view stored, std::string_view key);

// The host's UI size, for the Controls window to change beside the keys (AppRuntime::
// set_host_ui_size): the state path holding it, a number (1 = the UI at its own size; a leading
// '/' is absolute, as KeyBinding::path is), and the sizes offered, each its label and the value
// written, spelled as the host writes the number back. arigl's is cvc::gl::ImGuiOverlay's
// <scene>.viewers.main.ui.scale, which the overlay applies at the next frame.
struct UiSizeControl {
  std::string path; // "" = no UI size row
  std::vector<std::pair<std::string, std::string>> sizes = {
      {"75%", "0.75"}, {"100%", "1"}, {"125%", "1.25"}, {"150%", "1.5"}, {"200%", "2"}};
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_KEYS_H

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

// The Keys window's id, its open state, and the state key naming the binding whose Change button
// is waiting for a key ("" = none; it counts only while the window is open). Under the document's
// prefix, beside the window's other tree.<id> state, so no binding id can collide with them.
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

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_KEYS_H

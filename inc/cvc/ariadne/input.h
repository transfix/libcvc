// cvc::ariadne::InputEvent — the runtime-level input event the host feeds to Runtime::post_input.
//
// This is a PURE-LIBCVC type (cvc::ariadne knows nothing of SDL / cvcGL / VTK — the dependency
// runs one way, cvcGL -> libcvc). A host input source (e.g. cvc::gl::SdlInput, which produces a
// field-identical cvc::gl::InputEvent) translates its events into this and calls
// Runtime::post_input each frame; the runtime serializes it to a state_exec dict and delivers it
// to the on_key / on_pointer resident handlers (§4.6). Keeping the type here lets the runtime
// consume input without any windowing/toolkit dependency.
#ifndef CVC_ARIADNE_INPUT_H
#define CVC_ARIADNE_INPUT_H

#include <string>

namespace cvc {
namespace ariadne {

// One normalized input event. Field meaning is keyed by `kind`; unused fields are 0/empty. Mirrors
// cvc::gl::InputEvent (the SDL producer) so a host translation is a plain field copy.
struct InputEvent {
  enum class Kind {
    KeyDown,
    KeyUp,
    MouseMove,
    MouseButtonDown,
    MouseButtonUp,
    MouseWheel,
  };
  Kind kind = Kind::KeyDown;

  // Keyboard (KeyDown/KeyUp): `key` is the human key name (e.g. "A", "Escape", "Space", "Left");
  // `mods` a modifier bitmask; `repeat` marks an auto-repeat.
  std::string key;
  unsigned mods = 0;
  bool repeat = false;

  // Pointer (MouseMove/MouseButton*/MouseWheel): `x`,`y` are window coordinates. For MouseMove,
  // `dx`,`dy` are the relative motion; for MouseWheel, `dx`,`dy` are the scroll amounts. `button`
  // is the 1-based button index and `clicks` the click count (MouseButton* only).
  float x = 0.0f, y = 0.0f;
  float dx = 0.0f, dy = 0.0f;
  int button = 0;
  int clicks = 0;

  // The event kind as a stable lowercase string, used as the "kind" field of the delivered dict
  // and by post_input to route keyboard vs pointer events to their channels. An on_* body reads
  // it with (get-attr event "kind").
  const char *kind_name() const {
    switch (kind) {
    case Kind::KeyDown:
      return "key_down";
    case Kind::KeyUp:
      return "key_up";
    case Kind::MouseMove:
      return "mouse_move";
    case Kind::MouseButtonDown:
      return "mouse_button_down";
    case Kind::MouseButtonUp:
      return "mouse_button_up";
    case Kind::MouseWheel:
      return "mouse_wheel";
    }
    return "unknown";
  }

  bool is_keyboard() const { return kind == Kind::KeyDown || kind == Kind::KeyUp; }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_INPUT_H

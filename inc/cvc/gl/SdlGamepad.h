// SdlGamepad — cvcGL's SDL-backed gamepad (ARIADNE_WORLDS.md section 18.6, OW6b).
//
// The first gamepad plugged in, read by the names a control binding gives its inputs
// (cvc/ariadne/controls.h): the buttons A, B, X, Y (by position, Xbox-style), LB, RB, Back, Start,
// Guide, LeftStick, RightStick, DPadUp, DPadDown, DPadLeft, DPadRight (0 or 1); the triggers LT,
// RT (0 to 1); the sticks LeftX, LeftY, RightX, RightY (-1 to 1, Y down). SDL is confined to
// SdlGamepad.cpp; without SDL (CVC_ENABLE_SDL off) there is never a gamepad.
//
// One per host that reads it (no process-wide state): SDL's gamepad subsystem is refcounted, so
// two hosts each hold it while they live.
#ifndef CVC_GL_SDL_GAMEPAD_H
#define CVC_GL_SDL_GAMEPAD_H

#include <string>
#include <vector>

namespace cvc {
namespace gl {

class SdlGamepad {
public:
  SdlGamepad() = default;
  ~SdlGamepad();
  SdlGamepad(const SdlGamepad &) = delete;
  SdlGamepad &operator=(const SdlGamepad &) = delete;

  // Once a frame, on the thread that runs SDL's events: opens the first gamepad connected (one
  // plugged in later is found within a second), lets go of one unplugged, and reads the open one.
  // True while one is open.
  bool update();
  bool connected() const { return pad_ != nullptr; }
  // The open gamepad's name ("Xbox Series X Controller"); empty when none.
  std::string name() const;

  // An input's reading by its name (above), as of the last update(); 0 for a name it does not
  // know, or with no gamepad.
  double input(const std::string &name) const;
  // The inputs pressed at the last update() that were not before, by name: a button gone down, a
  // trigger or a stick's half ("LeftY-") past half way. What the Controls window binds
  // (cvc::ariadne::Runtime::post_pad_press).
  const std::vector<std::string> &pressed() const { return pressed_; }

private:
  bool inited_ = false, failed_ = false; // SDL's gamepad subsystem: held, or would not start
  void *pad_ = nullptr;                  // the open SDL_Gamepad
  int scan_ = 0;                         // updates until the next look for a gamepad
  std::vector<std::string> down_;        // the inputs past half way at the last update()
  std::vector<std::string> pressed_;     // ... and not at the one before
  void read_pressed();
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SDL_GAMEPAD_H

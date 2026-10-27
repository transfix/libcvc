// SdlInput — cvcGL's SDL-backed input source (keyboard / mouse; gamepad + audio/mic/camera later).
//
// The first piece of the §4.6 input seam: it turns SDL's OS input events into a small, SDL-free
// normalized InputEvent that the Ariadne runtime's on_* resident handlers (on_key/on_click/…) can
// consume — a real cross-platform source (native AND wasm/wasm-mt, via the sdl3 cvcpkg dep) instead
// of a bespoke Backend key API. SDL itself is confined to SdlInput.cpp; this header exposes no SDL
// types, so a cvcGL consumer needs no SDL headers.
//
// Build gating: with SDL present (CVC_ENABLE_SDL, set by src/cvcGL/CMakeLists.txt when it finds
// SDL3), SdlInput is live; without it, init() returns false and poll() returns only the VTK keys
// attach() observes. See cvcGLConfig / the cvcgl recipe's sdl3 dependency.
#ifndef CVC_GL_SDL_INPUT_H
#define CVC_GL_SDL_INPUT_H

#include <string>
#include <vector>

class vtkRenderWindow; // VTK (global namespace)
class vtkRenderWindowInteractor;
class vtkCallbackCommand;

namespace cvc {
namespace gl {

// One normalized input event, backend-agnostic (SDL is the current producer). Field meaning is
// keyed by `kind`; unused fields are 0/empty.
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

  // Keyboard (KeyDown/KeyUp): `key` is the human key name (SDL_GetKeyName — e.g. "A", "Escape",
  // "Space", "Left"); `mods` is the SDL_Keymod bitmask; `repeat` marks an auto-repeat.
  std::string key;
  unsigned mods = 0;
  bool repeat = false;

  // Pointer (MouseMove/MouseButton*/MouseWheel): `x`,`y` are window coordinates. For MouseMove,
  // `dx`,`dy` are the relative motion; for MouseWheel, `dx`,`dy` are the scroll amounts. `button`
  // is the 1-based SDL button index and `clicks` the click count (MouseButton* only).
  float x = 0.0f, y = 0.0f;
  float dx = 0.0f, dy = 0.0f;
  int button = 0;
  int clicks = 0;
};

class SdlInput {
public:
  SdlInput() = default;
  ~SdlInput();
  SdlInput(const SdlInput &) = delete;
  SdlInput &operator=(const SdlInput &) = delete;

  // Initialize the SDL events subsystem (events only — no window/video, so this is headless-safe;
  // a windowed host that already SDL_Init(VIDEO)'d just shares the refcounted subsystem). Returns
  // true when SDL input is live, false in a build without SDL or if SDL init fails.
  bool init();

  // True once init() succeeded (and this build has SDL).
  bool available() const;

  // Take the keyboard (and pointer) of the VTK window a cvcGL host draws in. SDL only delivers keys
  // to a window of its own, so on Windows this wraps the host's HWND, and in the browser listens on
  // the page for the keys typed into VTK's canvas; the host's own input goes on as before. Where
  // SDL cannot take VTK's window (X11, Wayland, macOS, a build without SDL, or with
  // $CVC_SDLINPUT_VTK_KEYS=1), poll() returns VTK's own key events instead, named as SDL names
  // them: the presses ImGuiOverlay leaves to the scene, every release, and no pointer.
  // False until VTK has made its window and interactor: call again on a later frame.
  bool attach(vtkRenderWindow *window);
  bool attached() const;

  // Drain the SDL event queue, returning the translated keyboard/mouse events (other event types —
  // quit, window, etc. — are left for the host to handle). Empty when not available.
  std::vector<InputEvent> poll();

  // Release the events subsystem (refcounted; safe to call more than once). Called by the dtor.
  void shutdown();

private:
  bool inited_ = false;
  bool video_ = false;           // SDL_INIT_VIDEO, which attach() needs
  void *window_ = nullptr;       // attach(): the SDL_Window wrapping the host's
  std::vector<InputEvent> keys_; // the browser's keys (attach()'s event filter), or VTK's
  vtkRenderWindowInteractor *vtkKeys_ = nullptr; // attach() without SDL's window: VTK's keys,
  vtkCallbackCommand *vtkKeyCb_ = nullptr;       // observed by this
  unsigned long vtkKeyTags_[2] = {0, 0};
  std::vector<std::string> held_; // VTK's keys down, to mark a repeat
  bool attach_vtk_keys(vtkRenderWindow *window);
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SDL_INPUT_H

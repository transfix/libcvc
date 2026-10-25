// SdlInput — SDL3-backed input source. SDL is confined here; the header is SDL-free.
// Compiled always (the file glob picks it up); the SDL body is behind CVC_ENABLE_SDL, so a build
// without SDL yields only the VTK interactor's keys (attach()), and init() false.

#include <algorithm>
#include <cstdlib>
#include <cvc/gl/SdlInput.h>
#include <map>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#endif

namespace cvc {
namespace gl {

namespace {

// A VTK key event, as SDL would have named it: VTK's keysyms are X11's ("a", "space", "Prior",
// "comma", "Shift_L", "KP_1", "Caps_Lock") where SDL says "A", "Space", "PageUp", ",",
// "Left Shift", "Keypad 1", "CapsLock" (the arrows, "Return", "Escape", "Tab", "Home", "End",
// "Insert", "Delete", "Pause" and F1.. agree). Modifiers as SDL's left-hand bits, the only ones
// VTK tells apart from none.
std::string sdl_key_name(const char *sym) {
  static const std::map<std::string, std::string> sdl = {{"space", "Space"},
                                                         {"Prior", "PageUp"},
                                                         {"Page_Up", "PageUp"},
                                                         {"Next", "PageDown"},
                                                         {"Page_Down", "PageDown"},
                                                         {"BackSpace", "Backspace"},
                                                         {"Shift_L", "Left Shift"},
                                                         {"Shift_R", "Right Shift"},
                                                         {"Control_L", "Left Ctrl"},
                                                         {"Control_R", "Right Ctrl"},
                                                         {"Alt_L", "Left Alt"},
                                                         {"Alt_R", "Right Alt"},
                                                         {"Super_L", "Left GUI"},
                                                         {"Super_R", "Right GUI"},
                                                         {"Caps_Lock", "CapsLock"},
                                                         {"Num_Lock", "Numlock"},
                                                         {"Scroll_Lock", "ScrollLock"},
                                                         {"Print", "PrintScreen"},
                                                         {"KP_Enter", "Keypad Enter"},
                                                         {"KP_Add", "Keypad +"},
                                                         {"KP_Subtract", "Keypad -"},
                                                         {"KP_Multiply", "Keypad *"},
                                                         {"KP_Divide", "Keypad /"},
                                                         {"KP_Decimal", "Keypad ."},
                                                         {"KP_Equal", "Keypad ="},
                                                         {"comma", ","},
                                                         {"period", "."},
                                                         {"slash", "/"},
                                                         {"backslash", "\\"},
                                                         {"semicolon", ";"},
                                                         {"apostrophe", "'"},
                                                         {"quoteright", "'"},
                                                         {"bracketleft", "["},
                                                         {"bracketright", "]"},
                                                         {"minus", "-"},
                                                         {"equal", "="},
                                                         {"grave", "`"},
                                                         {"quoteleft", "`"},
                                                         {"plus", "+"},
                                                         {"less", "<"},
                                                         {"greater", ">"},
                                                         {"question", "?"},
                                                         {"colon", ":"},
                                                         {"quotedbl", "\""},
                                                         {"exclam", "!"},
                                                         {"at", "@"},
                                                         {"numbersign", "#"},
                                                         {"dollar", "$"},
                                                         {"percent", "%"},
                                                         {"asciicircum", "^"},
                                                         {"ampersand", "&"},
                                                         {"asterisk", "*"},
                                                         {"parenleft", "("},
                                                         {"parenright", ")"},
                                                         {"underscore", "_"},
                                                         {"braceleft", "{"},
                                                         {"braceright", "}"},
                                                         {"bar", "|"},
                                                         {"asciitilde", "~"}};
  if (!sym[1] && sym[0] >= 'a' && sym[0] <= 'z')
    return std::string(1, static_cast<char>(sym[0] - 'a' + 'A'));
  if (sym[0] == 'K' && sym[1] == 'P' && sym[2] == '_' && sym[3] >= '0' && sym[3] <= '9' && !sym[4])
    return std::string("Keypad ") + sym[3];
  const auto it = sdl.find(sym);
  return it == sdl.end() ? std::string(sym) : it->second;
}

void on_vtk_key(vtkObject *caller, unsigned long eid, void *keys, void *) {
  auto *iren = static_cast<vtkRenderWindowInteractor *>(caller);
  const char *sym = iren->GetKeySym();
  if (!sym || !*sym)
    return;
  InputEvent ie;
  ie.kind = eid == vtkCommand::KeyPressEvent ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
  ie.key = sdl_key_name(sym);
  ie.mods = (iren->GetShiftKey() ? 0x0001u : 0u) | (iren->GetControlKey() ? 0x0040u : 0u) |
            (iren->GetAltKey() ? 0x0100u : 0u); // SDL_KMOD_LSHIFT, _LCTRL, _LALT
  static_cast<std::vector<InputEvent> *>(keys)->push_back(std::move(ie));
}

// $CVC_SDLINPUT_VTK_KEYS set (and not "0"): VTK's keys even where SDL could take the window.
bool vtk_keys_forced() {
  const char *v = std::getenv("CVC_SDLINPUT_VTK_KEYS");
  return v && *v && std::string(v) != "0";
}

} // namespace

#ifdef CVC_ENABLE_SDL
namespace {

InputEvent key_event(const SDL_Event &e) {
  InputEvent ie;
  ie.kind = (e.type == SDL_EVENT_KEY_DOWN) ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
  const char *name = SDL_GetKeyName(e.key.key); // "A", "Escape", "Space", "Left", …
  ie.key = name ? name : "";
  ie.mods = static_cast<unsigned>(e.key.mod);
  ie.repeat = e.key.repeat;
  return ie;
}

#ifdef __EMSCRIPTEN__
// In the browser SDL calls preventDefault() on every key event it queues, which would swallow the
// keypress the browser makes from a keydown -- the one VTK turns into typed characters for ImGui.
// A key event a filter drops is not prevented, so the filter keeps the keys for poll() itself:
// those typed into VTK's canvas (SDL listens on the window, so it hears the page's other fields
// too).
bool SDLCALL keep_keys(void *keys, SDL_Event *e) {
  if (e->type != SDL_EVENT_KEY_DOWN && e->type != SDL_EVENT_KEY_UP)
    return true;
  if (MAIN_THREAD_EM_ASM_INT({ return document.activeElement == Module['canvas'] ? 1 : 0; }))
    static_cast<std::vector<InputEvent> *>(keys)->push_back(key_event(*e));
  return false;
}
#endif

} // namespace
#endif

bool SdlInput::init() {
#ifdef CVC_ENABLE_SDL
  if (inited_)
    return true;
  // Events subsystem only: no window/video, so this works headless (and a windowed host that
  // already initialized SDL_INIT_VIDEO just shares the refcounted subsystem). SDL3 returns bool.
  if (!SDL_InitSubSystem(SDL_INIT_EVENTS))
    return false;
  inited_ = true;
  return true;
#else
  return false;
#endif
}

bool SdlInput::available() const {
#ifdef CVC_ENABLE_SDL
  return inited_;
#else
  return false;
#endif
}

bool SdlInput::attach(vtkRenderWindow *host) {
  if (attached())
    return true;
  if (!host)
    return false;
#if defined(CVC_ENABLE_SDL) && (defined(_WIN32) || defined(__EMSCRIPTEN__))
  if (vtk_keys_forced())
    return attach_vtk_keys(host);
  if (!init())
    return false;
#ifndef __EMSCRIPTEN__
  void *native = host->GetGenericWindowId();
  if (!native)
    return false; // VTK makes its window at the first render: try again on a later frame
#endif
  if (!video_) {
    // SDL's video is here only to wrap the host's window, so it leaves the process as the host made
    // it, unless the app said otherwise: the screen still sleeps, and the DPI awareness stays.
    if (!SDL_GetHint(SDL_HINT_VIDEO_ALLOW_SCREENSAVER))
      SDL_SetHint(SDL_HINT_VIDEO_ALLOW_SCREENSAVER, "1");
    // SDL 3 still reads SDL_WINDOWS_DPI_AWARENESS, though SDL_hints.h no longer names it; with none
    // of its values ("permonitorv2", ...) it leaves the process's awareness alone.
    if (!SDL_GetHint("SDL_WINDOWS_DPI_AWARENESS"))
      SDL_SetHint("SDL_WINDOWS_DPI_AWARENESS", "");
    if (!SDL_InitSubSystem(SDL_INIT_VIDEO))
      return false;
    video_ = true;
  }
  const SDL_PropertiesID props = SDL_CreateProperties();
#if defined(__EMSCRIPTEN__)
  // SDL sizes the canvas its window is made on, and zeroes it when the window goes, which would
  // fight VTK over its canvas: so SDL's window is a hidden canvas of its own. Its keyboard stays
  // SDL's default, the browser window, where VTK's canvas's keys bubble up to: SDL first removes
  // every key listener on its keyboard element, so on the canvas it would take VTK's keys away.
  MAIN_THREAD_EM_ASM({
    if (!document.getElementById('cvcgl-sdl-input')) {
      const c = document.createElement('canvas');
      c.id = 'cvcgl-sdl-input';
      c.width = c.height = 1;
      c.style.cssText =
          'position:fixed;left:0;top:0;width:1px;height:1px;visibility:hidden;pointer-events:none';
      document.body.appendChild(c);
    }
  });
  SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_EMSCRIPTEN_CANVAS_ID_STRING,
                        "#cvcgl-sdl-input");
  SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, 1);
  SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, 1);
#else
  // SDL subclasses the window's procedure and passes every message on to VTK's.
  SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER, native);
#endif
  window_ = SDL_CreateWindowWithProperties(props);
  SDL_DestroyProperties(props);
#ifdef __EMSCRIPTEN__
  if (window_)
    SDL_SetEventFilter(keep_keys, &keys_);
#endif
  return window_ != nullptr;
#else
  // No SDL window over VTK's here. On X11 SDL would select XInput2 pointer events on it from its
  // own display connection, and the server then sends the clicks to SDL instead of VTK; on Wayland
  // SDL makes a window of its own; on macOS VTK hands out an NSView where SDL wraps an NSWindow.
  return attach_vtk_keys(host);
#endif
}

// The keys are VTK's. A press is observed below ImGuiOverlay (1.0), which stops one ImGui takes,
// and above the camera's interactor style (0.0), which still gets every one; a release above
// ImGuiOverlay, so a key the scene saw go down never stays held.
bool SdlInput::attach_vtk_keys(vtkRenderWindow *host) {
  vtkRenderWindowInteractor *iren = host->GetInteractor();
  if (!iren)
    return false;
  vtkKeys_ = iren;
  vtkKeys_->Register(nullptr);
  vtkKeyCb_ = vtkCallbackCommand::New();
  vtkKeyCb_->SetCallback(on_vtk_key);
  vtkKeyCb_->SetClientData(&keys_);
  vtkKeyTags_[0] = iren->AddObserver(vtkCommand::KeyPressEvent, vtkKeyCb_, 0.5f);
  vtkKeyTags_[1] = iren->AddObserver(vtkCommand::KeyReleaseEvent, vtkKeyCb_, 2.0f);
  return true;
}

bool SdlInput::attached() const { return window_ != nullptr || vtkKeys_ != nullptr; }

std::vector<InputEvent> SdlInput::poll() {
  std::vector<InputEvent> out;
#ifdef CVC_ENABLE_SDL
  if (inited_)
    SDL_PumpEvents(); // the browser's keys reach keys_ through keep_keys as they are pumped
#endif
  out.swap(keys_); // those, or VTK's
  if (vtkKeys_) {
    // A held key repeats. X11 sends each repeat as a release and a press together (VTK does not
    // ask for detectable autorepeat), so that pair is one repeat; elsewhere VTK repeats the press.
    std::vector<InputEvent> keys;
    for (std::size_t i = 0; i < out.size(); ++i) {
      InputEvent &e = out[i];
      const auto held = std::find(held_.begin(), held_.end(), e.key);
      if (e.kind == InputEvent::Kind::KeyUp) {
        if (i + 1 < out.size() && out[i + 1].kind == InputEvent::Kind::KeyDown &&
            out[i + 1].key == e.key && held != held_.end()) {
          out[i + 1].repeat = true;
          continue;
        }
        if (held != held_.end())
          held_.erase(held);
      } else if (held != held_.end()) {
        e.repeat = true;
      } else {
        held_.push_back(e.key);
      }
      keys.push_back(std::move(e));
    }
    out.swap(keys);
  }
#ifdef CVC_ENABLE_SDL
  if (!inited_)
    return out;
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP:
      out.push_back(key_event(e));
      break;
    case SDL_EVENT_MOUSE_MOTION: {
      InputEvent ie;
      ie.kind = InputEvent::Kind::MouseMove;
      ie.x = e.motion.x;
      ie.y = e.motion.y;
      ie.dx = e.motion.xrel;
      ie.dy = e.motion.yrel;
      out.push_back(ie);
      break;
    }
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_BUTTON_UP: {
      InputEvent ie;
      ie.kind = (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) ? InputEvent::Kind::MouseButtonDown
                                                        : InputEvent::Kind::MouseButtonUp;
      ie.x = e.button.x;
      ie.y = e.button.y;
      ie.button = e.button.button;
      ie.clicks = e.button.clicks;
      out.push_back(ie);
      break;
    }
    case SDL_EVENT_MOUSE_WHEEL: {
      InputEvent ie;
      ie.kind = InputEvent::Kind::MouseWheel;
      ie.x = e.wheel.mouse_x;
      ie.y = e.wheel.mouse_y;
      ie.dx = e.wheel.x;
      ie.dy = e.wheel.y;
      out.push_back(ie);
      break;
    }
    default:
      break; // quit / window / text / gamepad-etc. — the host owns those (v1 = keyboard + mouse)
    }
  }
#endif
  return out;
}

void SdlInput::shutdown() {
  if (vtkKeys_) {
    for (const unsigned long tag : vtkKeyTags_)
      vtkKeys_->RemoveObserver(tag);
    vtkKeyCb_->Delete();
    vtkKeyCb_ = nullptr;
    vtkKeys_->UnRegister(nullptr);
    vtkKeys_ = nullptr;
    held_.clear();
  }
#ifdef CVC_ENABLE_SDL
  if (window_) {
#ifdef __EMSCRIPTEN__
    SDL_SetEventFilter(nullptr, nullptr);
#endif
    // A wrapped window is the host's: SDL gives its window procedure back and leaves it standing.
    SDL_DestroyWindow(static_cast<SDL_Window *>(window_));
    window_ = nullptr;
  }
  if (video_) {
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    video_ = false;
  }
  if (inited_) {
    SDL_QuitSubSystem(SDL_INIT_EVENTS);
    inited_ = false;
  }
#endif
}

SdlInput::~SdlInput() { shutdown(); }

} // namespace gl
} // namespace cvc

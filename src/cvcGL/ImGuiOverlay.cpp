// ImGuiOverlay — Dear ImGui inside a cvcGL viewer (see the header).
//
// The hook pattern (render on the window's Start/Render events, input via an
// interceptor on the current interactor style) follows Jaswant Panchumarti's
// vtkDearImGuiInjector (BSD-3-Clause); this is a cvcGL-native reimplementation
// against SceneRenderer, not a vendored copy.
//
// Without CVC_ENABLE_IMGUI the whole class compiles to inert stubs, so consumers
// never need an #ifdef.

#include <cvc/gl/ImGuiOverlay.h>

#ifdef CVC_ENABLE_IMGUI

#include "ImGuiKeyNames.h" // VTK's key names (X11's, or a browser's) as ImGui keys

#include <cvc/gl/CameraController.h>
#include <cvc/gl/Clipboard.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Settings.h>
#include <imgui.h>
#include <imgui_internal.h> // the toggle's z-order; carrying windows along with a new UI size
// The GL loader must match VTK's context: WebGL2/GLES3 in the browser build.
#if defined(__EMSCRIPTEN__) && !defined(IMGUI_IMPL_OPENGL_ES3)
#define IMGUI_IMPL_OPENGL_ES3
#endif
#include <backends/imgui_impl_opengl3.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
// Not emscripten_run_script_int: that is an eval, which a page with a Content-Security-Policy
// without 'unsafe-eval' refuses (arigl-web may be served under one).
// clang-format off: a JavaScript body
EM_JS(int, cvcgl_is_touch_device, (), {
  return ('ontouchstart' in globalThis ||
          (typeof navigator !== 'undefined' && navigator.maxTouchPoints > 0)) ? 1 : 0;
});

// Device pixels per CSS pixel: what VTK multiplies the canvas's CSS size by.
EM_JS(double, cvcgl_device_pixel_ratio, (), {
  var r = globalThis.devicePixelRatio;
  return r > 0 ? r : 1.0;
});

// The system keyboard (see the header). A phone raises its keyboard only for a focused text
// field, so this adds a hidden <input> beside the canvas, focuses it while an ImGui text field is
// active after a touch, and queues what the keyboard does to it for cvcgl_ime_take: a code point
// (> 0) per character typed, or a key (< 0, kImeKeys). The state lives on the canvas element, as
// TouchGestures' does, so each viewer has its own.
//
// Text is read from what the keyboard did to the field, not from key events: a phone's keyboard
// sends keyCode 229 for most keys, composes words (Android), and replaces them (autocorrect,
// suggestions). Each change is compared with the last: Backspaces for what went, then what came.
// The field starts as two spaces so a Backspace always has something to delete (a keyboard sends
// nothing for a Backspace in an empty field), and is reset to them when it runs short or long.
//
// iOS raises the keyboard only for a focus made inside a user gesture. VTK turns a touchstart into
// a press that ImGui takes at the next frame, so by the time the finger lifts the field has become
// active: the touchend handler focuses the input then. A field that becomes active with no finger
// down (one that took the text by itself, or a very quick tap) is focused at once, which raises the
// keyboard on Android; on iOS the next tap on it does.
EM_JS(void, cvcgl_ime_install, (const char *sel), {
  var el = document.querySelector(UTF8ToString(sel)) || document.getElementById('canvas') ||
           document.querySelector('canvas');
  if (!el || el.__cvcglIme)
    return;
  var SENT = '  ';
  var inp = document.createElement('input');
  inp.type = 'text';
  inp.autocomplete = 'off';
  inp.spellcheck = false;
  inp.tabIndex = -1;
  inp.setAttribute('autocorrect', 'off');
  inp.setAttribute('autocapitalize', 'off');
  inp.setAttribute('enterkeyhint', 'done');
  inp.setAttribute('aria-hidden', 'true');
  // Invisible but laid out, since a field with no box cannot take the focus; 16px, or iOS zooms the
  // page into it. Style properties, not a style attribute, which a Content-Security-Policy refuses.
  var s = inp.style;
  s.position = 'fixed';
  s.left = '0px';
  s.top = '0px';
  s.width = '1px';
  s.height = '16px';
  s.margin = '0';
  s.padding = '0';
  s.border = '0';
  s.outline = 'none';
  s.opacity = '0';
  s.color = 'transparent';
  s.background = 'transparent';
  s.caretColor = 'transparent';
  s.fontSize = '16px';
  s.pointerEvents = 'none';
  document.body.appendChild(inp);
  var st = {inp: inp, q: [], last: SENT, want: false, touch: false, touching: false,
            composing: false, mine: false, fullH: 0};
  el.__cvcglIme = st;
  var vv = globalThis.visualViewport;
  function viewH() { return vv ? vv.height * vv.scale : 0; }
  function reset() {
    inp.value = SENT;
    st.last = SENT;
    try { inp.setSelectionRange(SENT.length, SENT.length); } catch (e) {}
  }
  function sync() {
    var cur = inp.value, prev = st.last, p = 0, n = Math.min(cur.length, prev.length);
    while (p < n && cur.charCodeAt(p) == prev.charCodeAt(p))
      ++p;
    if (p > 0 && (prev.charCodeAt(p - 1) & 0xFC00) == 0xD800)
      --p; // not between the halves of a surrogate pair
    Array.from(prev.slice(p)).forEach(function() { st.q.push(-1); });
    Array.from(cur.slice(p)).forEach(function(c) {
      var cp = c.codePointAt(0);
      st.q.push(cp == 10 || cp == 13 ? -2 : cp);
    });
    st.last = cur;
    if (!st.composing && (cur.length < SENT.length || cur.length > 64))
      reset();
  }
  // Focus the input. Already focused with the keyboard up (a tap that moved the caret): leave it.
  // Focused with no keyboard (focused outside a gesture on iOS, or the keyboard was put away):
  // blur and focus again, inside this gesture. The view shrinks by a keyboard's height while one
  // is up; with no visualViewport, trust the focus.
  st.open = function() {
    if (document.activeElement == inp) {
      if (!vv || viewH() < 0.8 * st.fullH) {
        reset();
        return;
      }
      st.mine = true;
      inp.blur();
      st.mine = false;
    } else {
      st.fullH = viewH();
    }
    reset();
    inp.focus(); // and the browser scrolls it, at the field's caret, into view
  };
  st.close = function() {
    if (document.activeElement != inp)
      return;
    st.mine = true;
    inp.blur();
    st.mine = false;
    try { el.focus({preventScroll: true}); } catch (e) {} // keys go to the canvas again
  };
  var KEYS = {Enter: -2, Tab: -3, Escape: -4, ArrowLeft: -5, ArrowRight: -6, ArrowUp: -7,
              ArrowDown: -8, Home: -9, End: -10, Delete: -11};
  inp.addEventListener('keydown', function(e) {
    if (e.isComposing || e.keyCode == 229 || e.ctrlKey || e.metaKey || e.altKey)
      return;
    var k = KEYS[e.key];
    if (k == undefined)
      return; // characters and Backspace: from what they do to the field
    st.q.push(k);
    e.preventDefault();
    if (k == -2)
      reset();
  });
  inp.addEventListener('input', sync);
  inp.addEventListener('compositionstart', function() { st.composing = true; });
  inp.addEventListener('compositionend', function() {
    st.composing = false;
    sync();
  });
  // A finger (or a pen) wants the system keyboard; a mouse types through the canvas as before.
  var opt = {capture: true, passive: true};
  document.addEventListener('pointerdown', function(e) { st.touch = e.pointerType != 'mouse'; }, opt);
  el.addEventListener('touchstart', function() {
    st.touch = true;
    st.touching = true;
  }, opt);
  el.addEventListener('touchend', function(e) {
    st.touching = e.touches.length > 0;
    if (st.want && !st.touching)
      st.open();
  }, opt);
  el.addEventListener('touchcancel', function(e) { st.touching = e.touches.length > 0; }, opt);
});

// ImGui's text-input state, when it changes (Platform_SetImeDataFn): the input goes over the
// caret, so the browser keeps the field in view above the keyboard, and opens or closes with it.
EM_JS(void, cvcgl_ime_set, (const char *sel, int want, double x, double y, double h), {
  var el = document.querySelector(UTF8ToString(sel)) || document.getElementById('canvas') ||
           document.querySelector('canvas');
  var st = el && el.__cvcglIme;
  if (!st)
    return;
  var r = el.getBoundingClientRect(), k = el.width > 0 ? r.width / el.width : 1;
  st.inp.style.left = (r.left + x * k) + 'px';
  st.inp.style.top = (r.top + y * k) + 'px';
  st.inp.style.height = Math.max(16, h * k) + 'px';
  if (want && !st.want && st.touch && !st.touching)
    st.open();
  else if (!want && st.want)
    st.close();
  st.want = !!want;
});

// Drain up to `cap` of the keyboard's queued events into out[]; returns how many.
EM_JS(int, cvcgl_ime_take, (const char *sel, int *out, int cap), {
  var el = document.querySelector(UTF8ToString(sel)) || document.getElementById('canvas') ||
           document.querySelector('canvas');
  var st = el && el.__cvcglIme;
  if (!st)
    return 0;
  var n = Math.min(st.q.length, cap);
  for (var i = 0; i < n; ++i)
    setValue(out + 4 * i, st.q[i], 'i32');
  st.q.splice(0, n);
  return n;
});
// clang-format on
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility> // std::pair: the look cue's two strokes
#include <vector>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>
#include <vtkWeakPointer.h>
#include <vtk_glad.h> // GL_DRAW_FRAMEBUFFER

namespace cvc {
namespace gl {

namespace {

// ---- UI font configuration (roadmap §17.4 / §17.7 phase 2) -----------------
// Set via ImGuiOverlay::setUiFont*(); consumed by load_ui_font() when the ImGui
// context is built. ImGui 1.92 bakes glyphs on demand, so a covering font loaded
// here fixes non-ASCII coverage with no glyph-range wrangling.
std::string g_uiFontPath;
const void *g_uiFontMem = nullptr;
int g_uiFontMemLen = 0;
bool g_uiFontMemCompressed = false;

// Set once ImGui's OpenGL backend has failed to initialise on a current context. Process-wide on
// purpose: it mirrors the state of imgui's GL loader, which is itself process-global, and after
// one failure the next Init (from this overlay or any later one) gets past the loader and crashes
// on a null GL_VERSION string.
bool g_glBackendFailed = false;

bool file_readable(const char *path) {
  if (!path || !*path)
    return false;
  if (FILE *f = std::fopen(path, "rb")) {
    std::fclose(f);
    return true;
  }
  return false;
}

// Common broad-coverage system fonts (Latin + Latin-Ext + Cyrillic + Greek +
// symbols; DejaVu/Noto also carry many more). Returns "" if none is present.
std::string find_system_font() {
  static const char *const kCandidates[] = {
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",     // Debian/Ubuntu
      "/usr/share/fonts/dejavu/DejaVuSans.ttf",              // Fedora
      "/usr/share/fonts/TTF/DejaVuSans.ttf",                 // Arch
      "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", // Noto
      "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
      "/System/Library/Fonts/Supplemental/Arial Unicode.ttf", // macOS (very broad)
      "/Library/Fonts/Arial Unicode.ttf",
      "/System/Library/Fonts/Helvetica.ttc",
      "C:\\Windows\\Fonts\\arial.ttf", // Windows
      "C:\\Windows\\Fonts\\segoeui.ttf",
  };
  for (const char *c : kCandidates)
    if (file_readable(c))
      return c;
  return std::string();
}

// Load the UI font in priority order (memory -> path -> $CVC_UI_FONT -> system),
// falling back to the scalable ASCII face. An unfound/unloadable font is a logged
// fallback, never a failure. size_pixels 0 => sized from FontSizeBase at draw time.
void load_ui_font(ImGuiIO &io) {
  if (g_uiFontMem && g_uiFontMemLen > 0) {
    ImFont *f = nullptr;
    if (g_uiFontMemCompressed) {
      f = io.Fonts->AddFontFromMemoryCompressedTTF(g_uiFontMem, g_uiFontMemLen, 0.0f);
    } else {
      ImFontConfig cfg;
      cfg.FontDataOwnedByAtlas = false; // the buffer belongs to the caller
      f = io.Fonts->AddFontFromMemoryTTF(const_cast<void *>(g_uiFontMem), g_uiFontMemLen, 0.0f,
                                         &cfg);
    }
    if (f) {
      std::fprintf(stderr, "cvcGL: UI font <in-memory, %d bytes>\n", g_uiFontMemLen);
      return;
    }
    std::fprintf(stderr, "cvcGL: in-memory UI font failed to load — trying a path/system font\n");
  }

  std::string path = g_uiFontPath;
  if (path.empty())
    if (const char *e = std::getenv("CVC_UI_FONT"))
      path = e;
  if (path.empty())
    path = find_system_font();

  if (!path.empty()) {
    if (io.Fonts->AddFontFromFileTTF(path.c_str(), 0.0f)) {
      std::fprintf(stderr, "cvcGL: UI font %s\n", path.c_str());
      return;
    }
    std::fprintf(stderr, "cvcGL: could not load UI font '%s' — using the ASCII face\n",
                 path.c_str());
  }

  io.Fonts->AddFontDefaultVector(); // ASCII-only fallback (Basic Latin)
}

// Every pointer/wheel event we route through ImGui before VTK's style sees it.
const unsigned long kMouseEvents[] = {vtkCommand::MouseMoveEvent,
                                      vtkCommand::LeftButtonPressEvent,
                                      vtkCommand::LeftButtonReleaseEvent,
                                      vtkCommand::MiddleButtonPressEvent,
                                      vtkCommand::MiddleButtonReleaseEvent,
                                      vtkCommand::RightButtonPressEvent,
                                      vtkCommand::RightButtonReleaseEvent,
                                      vtkCommand::MouseWheelForwardEvent,
                                      vtkCommand::MouseWheelBackwardEvent,
                                      vtkCommand::EnterEvent,
                                      vtkCommand::LeaveEvent};
const unsigned long kKeyEvents[] = {vtkCommand::KeyPressEvent, vtkCommand::KeyReleaseEvent,
                                    vtkCommand::CharEvent};

// Is this a touch device? In the browser, ask the browser. Natively we assume
// not: a desktop touchscreen still has a mouse, and doubling the UI for someone
// with a mouse would be obnoxious. Callers can always override with setUiScale.
bool detectTouchDevice() {
#ifdef __EMSCRIPTEN__
  return cvcgl_is_touch_device() != 0;
#else
  return false;
#endif
}

// The UI size a touch device starts at: a laptop's UI a quarter larger, in the display's logical
// pixels -- 20 px text, and with the touch slop a button's hit area about 38 px tall (a fingertip
// wants 40 or so), on a phone too.
constexpr float kTouchUiScale = 1.25f;

// A phone's view: under this many logical pixels on its short side, the panels start hidden.
constexpr float kPhoneShortSide = 500.0f;

float clampUiScale(float scale) { return scale < 0.25f ? 0.25f : scale > 4.0f ? 4.0f : scale; }

// Device pixels per logical pixel (see setPixelRatio).
float detectPixelRatio() {
#ifdef __EMSCRIPTEN__
  const double r = cvcgl_device_pixel_ratio();
  return r > 0.0 ? static_cast<float>(r) : 1.0f;
#else
  return 1.0f;
#endif
}

#ifdef __EMSCRIPTEN__
// VTK's canvas, which TouchGestures listens on as well.
constexpr const char *kCanvas = "#canvas";

// The system keyboard's keys (cvcgl_ime_install's KEYS), by minus their code.
const ImGuiKey kImeKeys[] = {ImGuiKey_None,       ImGuiKey_Backspace, ImGuiKey_Enter,
                             ImGuiKey_Tab,        ImGuiKey_Escape,    ImGuiKey_LeftArrow,
                             ImGuiKey_RightArrow, ImGuiKey_UpArrow,   ImGuiKey_DownArrow,
                             ImGuiKey_Home,       ImGuiKey_End,       ImGuiKey_Delete};

// What the system keyboard typed since the last frame, in order: characters as text, the rest
// as a key's press and release (ImGui trickles them across frames, in order).
void feedSystemKeyboard() {
  ImGuiIO &io = ImGui::GetIO();
  int ev[64];
  for (int n; (n = cvcgl_ime_take(kCanvas, ev, 64)) > 0;)
    for (int i = 0; i < n; ++i) {
      if (ev[i] > 0) {
        io.AddInputCharacter(static_cast<unsigned int>(ev[i]));
        continue;
      }
      const int k = -ev[i];
      if (k < static_cast<int>(sizeof(kImeKeys) / sizeof(kImeKeys[0])) && k > 0) {
        io.AddKeyEvent(kImeKeys[k], true);
        io.AddKeyEvent(kImeKeys[k], false);
      }
    }
}
#endif

// A big, round, translucent show/hide button pinned to the bottom-right corner.
//
// ImGui has no round button, so this is an InvisibleButton (whose hit area is
// its bounding square — a feature for fingers) with a circle drawn over it. It
// lives in its own window because that is what makes ImGui claim the pointer
// over it; a foreground draw-list would paint fine but let clicks fall through
// to the camera. Zero window padding keeps the claimed rect exactly the size of
// the button, so it steals as little of the scene as possible.
bool floatingCircleToggle(const char *id, bool *open, float diameter, const char *glyph) {
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  const float margin = ImGui::GetFontSize() * 0.75f; // scales with the UI
  ImGui::SetNextWindowPos(
      ImVec2(vp->WorkPos.x + vp->WorkSize.x - margin, vp->WorkPos.y + vp->WorkSize.y - margin),
      ImGuiCond_Always, ImVec2(1.0f, 1.0f)); // pivot: the window's own corner
  ImGui::SetNextWindowBgAlpha(0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
  bool clicked = false;
  if (ImGui::Begin(id, nullptr, flags)) {
    // Above every window, even one dragged over this corner: on a phone it is the way back to the
    // scene. Not above a popup, though (a menu, or a combo's list open over the corner).
    if (!ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
      ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##hit", ImVec2(diameter, diameter));
    clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const bool hovered = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
    const ImVec2 c(p0.x + diameter * 0.5f, p0.y + diameter * 0.5f);
    const float r = diameter * 0.5f;
    ImDrawList *dl = ImGui::GetWindowDrawList();
    // GetColorU32 with an alpha multiplier respects style.Alpha; a raw IM_COL32
    // would not. Translucent so the scene still reads through the button.
    const ImU32 fill = ImGui::GetColorU32(held      ? ImGuiCol_ButtonActive
                                          : hovered ? ImGuiCol_ButtonHovered
                                                    : ImGuiCol_Button,
                                          held      ? 0.85f
                                          : hovered ? 0.70f
                                                    : 0.40f);
    dl->AddCircleFilled(c, r, fill, 0);
    dl->AddCircle(c, r, ImGui::GetColorU32(ImGuiCol_Border, 0.55f), 0,
                  diameter * 0.03f > 1.0f ? diameter * 0.03f : 1.0f);
    const ImVec2 ts = ImGui::CalcTextSize(glyph);
    dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f),
                ImGui::GetColorU32(ImGuiCol_Text, 0.9f), glyph);
  }
  ImGui::End();
  ImGui::PopStyleVar();
  if (clicked && open)
    *open = !*open;
  return clicked;
}
} // namespace

struct ImGuiOverlay::Impl {
  vtkWeakPointer<vtkRenderWindow> window;
  vtkWeakPointer<vtkRenderWindowInteractor> interactor;
  vtkSmartPointer<vtkCallbackCommand> beginCmd, renderCmd, interceptCmd;
  std::function<void()> draw;
  std::function<void()> sceneDraw; // setSceneDrawCallback: every frame, panels or not
  ImGuiContext *ctx = nullptr;
  bool ready = false;   // ImGui context created
  bool glReady = false; // GL backend initialized (deferred to first render)
  bool visible = true;  // caller intent
  CameraController *cam = nullptr;
  bool hadKeyboard = false; // for the rising edge of WantCaptureKeyboard
  // Where the windows drawn last frame are (x0 y0 x1 y1 each, ImGui's pixels): a press there is
  // the UI's, though ImGui hears of the pointer only in its next frame.
  std::vector<float> uiRegions;
  ImGuiStyle pristineStyle; // the unscaled style, captured once
  bool haveStyle = false;
  float appliedScale = -1.0f; // the style's scale (uiScale x pixelRatio); applyScale's guard
  bool appliedTouch = false;
  float uiScale = 1.0f;    // the UI size, in logical pixels
  float pixelRatio = 1.0f; // device pixels per logical pixel
  float ratioOverride = 0; // setPixelRatio; 0 = the display's
  float windowRescale = 1; // the open windows' share of a scale change, for the next frame
  bool touchMode = false;
  bool panelsOpen = true;               // the floating toggle drives this
  bool showToggle = true;               // draw the floating show/hide button
  bool sizedOnce = false;               // first frame with a real DisplaySize
  std::unique_ptr<UiSettings> settings; // the state mirror
  bool applyingState = false;           // re-entry guard
  bool frameOpen = false;
  double lastTime = 0.0;
  float fixedDt = 0.0f; // > 0: io.DeltaTime pinned (setFixedDeltaTime), else the wall clock

  // ---- render hooks -------------------------------------------------------
  // The GL backend can only be initialized once a context is CURRENT, which VTK
  // does not guarantee until it starts rendering — so it is done lazily on the
  // first StartEvent (in the ctor it fails with "Failed to initialize OpenGL
  // loader"), and the overlay stays inert until then.
  //
  // A failure with the context current is final and is not retried, by this
  // overlay or any other (g_glBackendFailed). On a context imgui's own GL
  // loader cannot reach (VTK's EGL fallback when there is no X display; the
  // loader resolves through GLX) the first Init fails, and a second one gets
  // past the loader and crashes on a null GL_VERSION string. The overlay then
  // stays off and enabled() reports false.
  bool ensureBackend() {
    if (glReady)
      return true;
    if (g_glBackendFailed || !window)
      return false;
    window->MakeCurrent();
    if (!window->IsCurrent())
      return false;
    glReady = ImGui_ImplOpenGL3_Init();
    if (!glReady) {
      g_glBackendFailed = true;
      std::fprintf(stderr, "cvcGL: ImGui's OpenGL backend could not initialise on this GL "
                           "context; the UI overlay is off\n");
    }
    return glReady;
  }

  // StartEvent: begin an ImGui frame and run the user's UI code.
  void beginFrame() {
    if (!ready || !visible || !window || !ensureBackend()) {
      if (cam && !uiRegions.empty()) { // no UI drawn: no window takes a press
        uiRegions.clear();
        cam->setUiRegions(uiRegions);
      }
      return;
    }
    int *sz = window->GetSize();
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(sz[0]), static_cast<float>(sz[1]));
    // ImGui requires a positive frame delta (it asserts on <= 0 and produces no
    // usable frame otherwise). VTK gives us no clock, so keep our own.
    const double now = vtkTimerLog::GetUniversalTime();
    io.DeltaTime = (lastTime > 0.0) ? static_cast<float>(now - lastTime) : (1.0f / 60.0f);
    if (io.DeltaTime <= 0.0f)
      io.DeltaTime = 1.0f / 60.0f;
    if (fixedDt > 0.0f)
      io.DeltaTime = fixedDt;
    lastTime = now;
    // Scale changes (the UI size, the display's pixel ratio) land here, between frames: the style
    // must not change under a frame's widgets, and the open windows come along.
    pixelRatio = ratioOverride > 0.0f ? ratioOverride : detectPixelRatio();
    applyScale();
    if (windowRescale != 1.0f) {
      rescaleWindows(windowRescale);
      windowRescale = 1.0f;
    }
#ifdef __EMSCRIPTEN__
    feedSystemKeyboard();
#endif
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
    frameOpen = true;
    // On a SMALL viewport (a phone), start with the panels hidden. Finger-sized
    // widgets on a 400px-wide screen would otherwise bury the scene the moment
    // it loads — the UI has to be reachable, not permanently in the way. The
    // toggle button is always there to bring it back. Decided once, on the
    // first frame that has a real size, so a later resize does not fight the
    // user's own choice. In logical pixels: a phone's canvas is ~1200 device
    // pixels across. Under 500 on the short side, either way up: phones are
    // 360-430, and a laptop's browser canvas can be under 600 (577 on a
    // 1366 x 768 screen), whose panels should start open.
    if (!sizedOnce && io.DisplaySize.x > 1.0f && io.DisplaySize.y > 1.0f) {
      sizedOnce = true;
      const float shortSide =
          io.DisplaySize.x < io.DisplaySize.y ? io.DisplaySize.x : io.DisplaySize.y;
      if (shortSide / pixelRatio < kPhoneShortSide) {
        panelsOpen = false;
        syncSettings(); // as the button's change is: ui.panels_open says what shows
      }
    }
    if (sceneDraw) // scene annotations: under the UI, and shown with the panels hidden too
      sceneDraw();
    if (cam && cam->mouseLookEnabled() && cam->mode() == CameraController::Mode::Orbit)
      drawLookCue(cam->pointerCapture());
    // The toggle is drawn FIRST so it is always reachable, even when the panels
    // it hides would have covered this corner.
    if (showToggle) {
      const float d = 34.0f * (appliedScale > 0.0f ? appliedScale : 1.0f);
      const bool was = panelsOpen;
      floatingCircleToggle("##cvcgl_ui_toggle", &panelsOpen, d, panelsOpen ? "X" : "=");
      if (panelsOpen != was)
        syncSettings(); // the button is a setting change like any other
    }
    if (draw && panelsOpen)
      draw();
  }

  // RenderEvent: VTK has drawn the scene — composite the ImGui draw lists on
  // top, inside the same render, so offscreen captures include the UI.
  void renderFrame() {
    if (!frameOpen)
      return;
    findUiRegions();
    if (cam) // a press over a window is the UI's, not the camera's (a browser's pointer lock)
      cam->setUiRegions(uiRegions);
    ImGui::Render();
    // Draw into VTK's OWN render framebuffer: at RenderEvent the scene has been
    // drawn there but not yet resolved/blitted out, so compositing here lands on
    // top in the window AND in offscreen captures. Without this the draw goes to
    // whatever happens to be bound and silently disappears. On wasm the same
    // chain ends at the WebGL default framebuffer (FrameBlitMode BlitToCurrent),
    // so this path is identical in the browser.
    auto *oglWin = vtkOpenGLRenderWindow::SafeDownCast(window);
    vtkOpenGLState *ostate = oglWin ? oglWin->GetState() : nullptr;
    if (ostate)
      ostate->PushFramebufferBindings();
    if (oglWin && oglWin->GetRenderFramebuffer())
      oglWin->GetRenderFramebuffer()->Bind(GL_DRAW_FRAMEBUFFER);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (ostate)
      ostate->PopFramebufferBindings();
    frameOpen = false;
  }

  // Every window drawn this frame, grown by the margin around a window that counts as on it (its
  // resize border). Child windows are inside theirs; a tooltip takes no input.
  void findUiRegions() {
    uiRegions.clear();
    ImGuiContext &g = *ImGui::GetCurrentContext();
    const float pad = g.WindowsBorderHoverPadding;
    for (const ImGuiWindow *w : g.Windows) {
      if (!w->Active || w->Hidden ||
          (w->Flags & (ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_NoMouseInputs)))
        continue;
      uiRegions.insert(uiRegions.end(), {w->Pos.x - pad, w->Pos.y - pad, w->Pos.x + w->Size.x + pad,
                                         w->Pos.y + w->Size.y + pad});
    }
  }

  bool overUiRegion(float x, float y) const {
    for (std::size_t i = 0; i + 3 < uiRegions.size(); i += 4)
      if (x >= uiRegions[i] && y >= uiRegions[i + 1] && x < uiRegions[i + 2] &&
          y < uiRegions[i + 3])
        return true;
    return false;
  }

  // Mirror every UI setting into cvc::state, and let a state write drive us.
  void syncSettings() {
    if (!settings || applyingState)
      return;
    UiSettings::Values v;
    v.visible = visible;
    v.scale = uiScale;
    v.touchMode = touchMode;
    v.panelsOpen = panelsOpen;
    v.toggleButton = showToggle;
    settings->set(v);
  }

  // ---- scaling -------------------------------------------------------------
  // Re-derive the whole style from the pristine copy at the UI size times the
  // pixel ratio. ScaleAllSizes() multiplies IN PLACE (and truncates to
  // integers), so calling it repeatedly compounds: five calls at 2x give
  // FramePadding 128 instead of 8. Always start from the unscaled style.
  void applyScale() {
    const float scale = uiScale * pixelRatio;
    if (!haveStyle || (scale == appliedScale && touchMode == appliedTouch))
      return;
    if (appliedScale > 0.0f)
      windowRescale *= scale / appliedScale;
    appliedScale = scale;
    appliedTouch = touchMode;
    ImGuiStyle &style = ImGui::GetStyle();
    style = pristineStyle;
    style.ScaleAllSizes(scale);
    // ScaleAllSizes cannot grow this one: it defaults to (0,0) and 0 * scale is
    // still 0. It is the slop around a widget's hit rect, which is exactly what
    // a fingertip needs.
    if (touchMode)
      style.TouchExtraPadding = ImVec2(4.0f * scale, 4.0f * scale);
    // Fonts are dynamic in 1.92 (re-baked at the final size), so this is both
    // crisp and safe to change at runtime. io.FontGlobalScale is OBSOLETE and
    // asserts against these fields — do not use it. Their product is the whole
    // UI's scale, which the Ariadne backend applies to a document's px as well.
    style.FontScaleMain = uiScale;
    style.FontScaleDpi = pixelRatio;
    ImGuiIO &io = ImGui::GetIO();
    // Fingers wobble; the 6px default drag threshold turns a tap into a drag.
    // Logical pixels, as the rest of the scale.
    io.MouseDragThreshold = (touchMode ? 10.0f : 6.0f) * pixelRatio;
    io.ConfigWindowsMoveFromTitleBarOnly = touchMode;
  }

  // The UI's scale changed under its open windows: carry them along, place and
  // size, as ImGui does for a monitor's DPI, and let each table take its
  // authored column widths again, at the new size.
  static void rescaleWindows(float factor) {
    ImGui::ScaleWindowsInViewport(static_cast<ImGuiViewportP *>(ImGui::GetMainViewport()), factor);
    ImGuiContext &g = *GImGui;
    for (int i = 0; i < g.Tables.GetMapSize(); ++i)
      if (ImGuiTable *t = g.Tables.TryGetMapData(i))
        ImGui::TableResetSettings(t);
  }

  // Mouse look's cue, under the UI's windows: a small cross at the centre while the mouse turns
  // the view, and below the centre how to have it do that while it does not.
  void drawLookCue(bool looking) {
    ImDrawList *dl = ImGui::GetBackgroundDrawList();
    const ImVec2 size = ImGui::GetIO().DisplaySize;
    const float k = appliedScale > 0.0f ? appliedScale : 1.0f;
    const ImVec2 c(size.x * 0.5f, size.y * 0.5f);
    if (looking) {
      const float out = 8.0f * k, in = 3.0f * k;
      // A dark line under a light one, so it shows on any background.
      for (const auto &[col, width] :
           {std::pair<ImU32, float>{IM_COL32(0, 0, 0, 150), 3.5f * k},
            std::pair<ImU32, float>{IM_COL32(255, 255, 255, 230), 1.5f * k}}) {
        dl->AddLine(ImVec2(c.x - out, c.y), ImVec2(c.x - in, c.y), col, width);
        dl->AddLine(ImVec2(c.x + in, c.y), ImVec2(c.x + out, c.y), col, width);
        dl->AddLine(ImVec2(c.x, c.y - out), ImVec2(c.x, c.y - in), col, width);
        dl->AddLine(ImVec2(c.x, c.y + in), ImVec2(c.x, c.y + out), col, width);
      }
      return;
    }
    // Just under the middle, where the click goes (a corner is where the windows are).
    const char *hint = "Click the scene to look around with the mouse. Esc gives the mouse back.";
    const ImVec2 ts = ImGui::CalcTextSize(hint);
    const ImVec2 p(c.x - 0.5f * ts.x, c.y + 40.0f * k);
    const float pad = 8.0f * k;
    dl->AddRectFilled(ImVec2(p.x - pad, p.y - 0.5f * pad),
                      ImVec2(p.x + ts.x + pad, p.y + ts.y + 0.5f * pad), IM_COL32(0, 0, 0, 130),
                      4.0f * k);
    dl->AddText(p, IM_COL32(235, 235, 235, 235), hint);
  }

  // ---- input --------------------------------------------------------------
  // Feed ImGui the interactor's pointer state, then decide whether VTK's style
  // is allowed to see this event. Returning true ABORTS the style's handling.
  bool intercept(unsigned long eid) {
    if (!ready || !visible || !interactor)
      return false;
    // Quake pointer capture: the cursor is warped to the centre each frame, so
    // absolute positions are meaningless — hand everything to the camera and let
    // ImGui idle until capture is released.
    if (cam && cam->pointerCapture())
      return false;
    ImGuiIO &io = ImGui::GetIO();
    // Tag the source BEFORE the events it applies to: AddMouseSourceEvent is not
    // an event, it is a latch stamped onto subsequent mouse events, and it is
    // initialised once at context creation rather than per frame. Left unset,
    // every event stays tagged with whatever was latched last.
    io.AddMouseSourceEvent(touchMode ? ImGuiMouseSource_TouchScreen : ImGuiMouseSource_Mouse);
    int x = 0, y = 0;
    interactor->GetEventPosition(x, y);
    int *sz = interactor->GetSize();
    // VTK's origin is bottom-left, ImGui's is top-left.
    io.AddMousePosEvent(static_cast<float>(x), static_cast<float>(sz[1] - y));

    switch (eid) {
    case vtkCommand::LeftButtonPressEvent:
      io.AddMouseButtonEvent(0, true);
      break;
    case vtkCommand::LeftButtonReleaseEvent:
      io.AddMouseButtonEvent(0, false);
      // A finger that lifts does not hover anywhere, but a mouse does — so
      // without this the last touched widget stays hovered FOREVER, which keeps
      // io.WantCaptureMouse true, which makes intercept() abort every event from
      // here on: one tap and the camera is locked out permanently. Clearing the
      // position is what the Android backend does for the same reason.
      //
      // This relies on io.ConfigInputTrickleEventQueue (default true): with
      // trickling off, the clear collapses into the same frame as the release,
      // the release lands on a non-hovered widget, and the tap is silently lost.
      // Never turn trickling off here.
      if (touchMode)
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      break;
    case vtkCommand::LeaveEvent:
      // The pointer left the window — same reasoning, for the desktop build.
      io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      break;
    case vtkCommand::RightButtonPressEvent:
      io.AddMouseButtonEvent(1, true);
      break;
    case vtkCommand::RightButtonReleaseEvent:
      io.AddMouseButtonEvent(1, false);
      break;
    case vtkCommand::MiddleButtonPressEvent:
      io.AddMouseButtonEvent(2, true);
      break;
    case vtkCommand::MiddleButtonReleaseEvent:
      io.AddMouseButtonEvent(2, false);
      break;
    case vtkCommand::MouseWheelForwardEvent:
      io.AddMouseWheelEvent(0.0f, 1.0f);
      break;
    case vtkCommand::MouseWheelBackwardEvent:
      io.AddMouseWheelEvent(0.0f, -1.0f);
      break;
    case vtkCommand::KeyPressEvent:
    case vtkCommand::KeyReleaseEvent: {
      // Keyboard, for text entry / editing keys / shortcuts inside ImGui (a REPL
      // or code-editor panel). Without this ImGui never receives keys, so an
      // active InputText sets WantCaptureKeyboard yet no character ever lands.
      // VTK exposes only aggregate modifier state (no left/right split), so feed
      // ImGui the combined mods; it derives io.KeyCtrl/Shift/Alt from these.
      const bool down = (eid == vtkCommand::KeyPressEvent);
      io.AddKeyEvent(ImGuiMod_Ctrl, interactor->GetControlKey() != 0);
      io.AddKeyEvent(ImGuiMod_Shift, interactor->GetShiftKey() != 0);
      io.AddKeyEvent(ImGuiMod_Alt, interactor->GetAltKey() != 0);
      const ImGuiKey key = imguiKeyForVtkKeySym(interactor->GetKeySym());
      if (key != ImGuiKey_None)
        io.AddKeyEvent(key, down);
      break;
    }
    case vtkCommand::CharEvent: {
      // The resolved character for text input. VTK's CharEvent carries it in the
      // key code; ImGui wants a codepoint. ASCII covers the REPL/editor case;
      // full IME / non-ASCII would need translation VTK's CharEvent doesn't give.
      const unsigned char c = static_cast<unsigned char>(interactor->GetKeyCode());
      if (c >= 32 && c != 127)
        io.AddInputCharacter(static_cast<unsigned int>(c));
      break;
    }
    default:
      break;
    }

    const bool keyEvent = (eid == vtkCommand::KeyPressEvent || eid == vtkCommand::KeyReleaseEvent ||
                           eid == vtkCommand::CharEvent);
    // Rising edge of "UI owns the keyboard": drop the camera's held keys so a
    // key released while the UI has focus can't leave it drifting.
    const bool wantsKeys = io.WantCaptureKeyboard;
    if (cam && wantsKeys && !hadKeyboard)
      cam->releaseHeldKeys();
    hadKeyboard = wantsKeys;
    // Swallow the event only when ImGui actually wants it — otherwise the
    // CameraController must keep receiving everything it always did. A press
    // on a window is ImGui's even before its next frame has seen the pointer
    // get there (a quick move and click): it is where ImGui will take it.
    if (keyEvent)
      return io.WantCaptureKeyboard;
    const bool press = eid == vtkCommand::LeftButtonPressEvent ||
                       eid == vtkCommand::RightButtonPressEvent ||
                       eid == vtkCommand::MiddleButtonPressEvent;
    return io.WantCaptureMouse ||
           (press && overUiRegion(static_cast<float>(x), static_cast<float>(sz[1] - y)));
  }

  static void onBegin(vtkObject *, unsigned long, void *cd, void *) {
    static_cast<Impl *>(cd)->beginFrame();
  }
  static void onRender(vtkObject *, unsigned long, void *cd, void *) {
    static_cast<Impl *>(cd)->renderFrame();
  }
  static void onIntercept(vtkObject *caller, unsigned long eid, void *cd, void *) {
    auto *self = static_cast<Impl *>(cd);
    (void)caller;
    // Abort => lower-priority observers (the interactor style, hence
    // CameraController) never see this event. Only when ImGui wants it.
    if (auto *cmd = self->interceptCmd.Get())
      cmd->SetAbortFlagOnExecute(self->intercept(eid) ? 1 : 0);
  }
};

ImGuiOverlay::ImGuiOverlay(SceneRenderer &viewer) : m_impl(new Impl) {
  m_impl->window = viewer.renderWindow();
  if (!m_impl->window)
    return;
  m_impl->interactor = m_impl->window->GetInteractor();

  IMGUI_CHECKVERSION();
  m_impl->ctx = ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr; // don't litter imgui.ini next to the binary
  ImGui::StyleColorsDark();

  // Be explicit about the font. AddFontDefault() is a HEURISTIC that picks the
  // 13px bitmap ProggyClean below an expected size of 15 and the scalable
  // ProggyForever at or above it — and it runs on the first frame, before any
  // scale we set later. Left implicit we silently get the bitmap face, which is
  // sharp at 13px and ugly at every other size. Ask for the vector face and give
  // it a logical base size; final pixels = FontSizeBase * FontScaleMain, and
  // ImGui 1.92 re-bakes glyphs at that exact size (no stretching, so scaling up
  // stays crisp). load_ui_font() picks a covering font (system / configured /
  // embedded) so non-ASCII text renders, and falls back to that ASCII vector face
  // when none is available (roadmap §17.4 / §17.7 phase 2).
  load_ui_font(io);
  ImGui::GetStyle().FontSizeBase = 16.0f;

  // Remember the unscaled style: ScaleAllSizes multiplies IN PLACE and rounds,
  // so it must never be applied twice. applyUiScale() always re-derives from
  // this pristine copy rather than scaling the live style again.
  m_impl->pristineStyle = ImGui::GetStyle();
  m_impl->haveStyle = true;

  // Touch devices get a bigger UI by default — the widget sizes that suit a
  // mouse are hard to hit with a finger. And the display's pixel ratio from
  // the start, so the first frame is already at its size.
  m_impl->touchMode = detectTouchDevice();
  m_impl->uiScale = m_impl->touchMode ? kTouchUiScale : 1.0f;
  m_impl->pixelRatio = detectPixelRatio();
  m_impl->applyScale();

  // Every UI setting is reachable as cvc::state, like the camera and the HUD.
  // A write lands at the next frame (beginFrame), as the setters' do.
  Impl *impl = m_impl.get();
  m_impl->settings = std::make_unique<UiSettings>(
      viewer.scene().appContext(),
      UiSettings::viewerStatePath(viewer.scene().getStatePrefix(), viewer.name()),
      [impl](UiSettings::Values v) {
        impl->applyingState = true;
        impl->visible = v.visible;
        impl->touchMode = v.touchMode;
        impl->panelsOpen = v.panelsOpen;
        impl->showToggle = v.toggleButton;
        impl->uiScale = clampUiScale(static_cast<float>(v.scale));
        impl->applyingState = false;
      });
  m_impl->syncSettings();

  // Copy/paste. There is no ImGui platform backend under us (VTK owns the
  // window), so nothing would supply these except on Windows, where imgui core
  // already implements the Win32 clipboard — cvc::gl::clipboard forwards to it
  // there and implements the rest (X11 selections, NSPasteboard, and in the
  // browser writeText for copy + the page's paste events for read).
  ImGuiPlatformIO &pio = ImGui::GetPlatformIO();
  pio.Platform_SetClipboardTextFn = [](ImGuiContext *, const char *text) {
    cvc::gl::clipboard::set(text ? text : "");
  };
  pio.Platform_GetClipboardTextFn = [](ImGuiContext *) -> const char * {
    // ImGui reads this immediately, so hand back storage that outlives the call
    // but is replaced on the next paste.
    static std::string buf;
    buf = cvc::gl::clipboard::get();
    return buf.c_str();
  };
#ifdef __EMSCRIPTEN__
  // The system keyboard, for a phone (see the header): ImGui reports here, at the end of each
  // frame where it changed, whether a text field wants text and where its caret is.
  cvcgl_ime_install(kCanvas);
  pio.Platform_SetImeDataFn = [](ImGuiContext *, ImGuiViewport *, ImGuiPlatformImeData *d) {
    cvcgl_ime_set(kCanvas, d->WantTextInput || d->WantVisible ? 1 : 0, d->InputPos.x, d->InputPos.y,
                  d->InputLineHeight);
  };
#endif

  m_impl->ready = true; // GL backend is initialized lazily (see ensureBackend)

  m_impl->beginCmd = vtkSmartPointer<vtkCallbackCommand>::New();
  m_impl->beginCmd->SetCallback(&Impl::onBegin);
  m_impl->beginCmd->SetClientData(m_impl.get());
  m_impl->window->AddObserver(vtkCommand::StartEvent, m_impl->beginCmd);

  m_impl->renderCmd = vtkSmartPointer<vtkCallbackCommand>::New();
  m_impl->renderCmd->SetCallback(&Impl::onRender);
  m_impl->renderCmd->SetClientData(m_impl.get());
  // RenderEvent: fired after the renderers have drawn but BEFORE VTK copies the
  // result frame, so the scene's framebuffer is still bound and the UI composites
  // on top — natively and in offscreen captures. (EndEvent is too late: the
  // result frame has already been copied out.)
  m_impl->window->AddObserver(vtkCommand::RenderEvent, m_impl->renderCmd);

  // Observe the INTERACTOR at priority 1.0 — NEVER the interactor style.
  // vtkInteractorStyle::ProcessEvents is
  //     if (HandleObservers && HasObserver(evt)) InvokeEvent(evt) else OnMouseMove()
  // so merely REGISTERING an observer on the style diverts every event into the
  // observer branch forever: OnMouseMove/OnLeftButtonDown are never called again
  // and orbit/pan/zoom/fly die the moment an overlay is constructed. Observing
  // the interactor leaves the style untouched; the camera is suppressed only by
  // the abort flag, and only when ImGui actually wants the input.
  if (m_impl->interactor) {
    m_impl->interceptCmd = vtkSmartPointer<vtkCallbackCommand>::New();
    m_impl->interceptCmd->SetCallback(&Impl::onIntercept);
    m_impl->interceptCmd->SetClientData(m_impl.get());
    for (unsigned long e : kMouseEvents)
      m_impl->interactor->AddObserver(e, m_impl->interceptCmd, 1.0);
    for (unsigned long e : kKeyEvents)
      m_impl->interactor->AddObserver(e, m_impl->interceptCmd, 1.0);
  }
}

ImGuiOverlay::~ImGuiOverlay() {
  if (m_impl->window) {
    if (m_impl->beginCmd)
      m_impl->window->RemoveObservers(vtkCommand::StartEvent, m_impl->beginCmd);
    if (m_impl->renderCmd)
      m_impl->window->RemoveObservers(vtkCommand::RenderEvent, m_impl->renderCmd);
  }
  if (m_impl->interactor && m_impl->interceptCmd) {
    for (unsigned long e : kMouseEvents)
      m_impl->interactor->RemoveObservers(e, m_impl->interceptCmd);
    for (unsigned long e : kKeyEvents)
      m_impl->interactor->RemoveObservers(e, m_impl->interceptCmd);
  }
  if (m_impl->glReady)
    ImGui_ImplOpenGL3_Shutdown();
  if (m_impl->ctx)
    ImGui::DestroyContext(m_impl->ctx);
}

void ImGuiOverlay::setDrawCallback(std::function<void()> draw) { m_impl->draw = std::move(draw); }
void ImGuiOverlay::setSceneDrawCallback(std::function<void()> draw) {
  m_impl->sceneDraw = std::move(draw);
}
ImGuiContext *ImGuiOverlay::imguiContext() const { return m_impl->ctx; }

void ImGuiOverlay::setUiScale(float scale) {
  m_impl->uiScale = clampUiScale(scale); // applied at the next frame (beginFrame)
  m_impl->syncSettings();
}
float ImGuiOverlay::uiScale() const { return m_impl->uiScale; }

void ImGuiOverlay::setPixelRatio(float ratio) {
  m_impl->ratioOverride = ratio > 0.0f ? ratio : 0.0f;
  m_impl->pixelRatio = ratio > 0.0f ? ratio : detectPixelRatio();
}
float ImGuiOverlay::pixelRatio() const { return m_impl->pixelRatio; }

void ImGuiOverlay::setTouchMode(bool on) {
  m_impl->touchMode = on;
  setUiScale(on ? kTouchUiScale : 1.0f); // also syncs
}
bool ImGuiOverlay::touchMode() const { return m_impl->touchMode; }

void ImGuiOverlay::setFixedDeltaTime(float seconds) {
  m_impl->fixedDt = seconds > 0.0f ? seconds : 0.0f;
}
float ImGuiOverlay::fixedDeltaTime() const { return m_impl->fixedDt; }

void ImGuiOverlay::setToggleButtonEnabled(bool on) {
  m_impl->showToggle = on;
  m_impl->syncSettings();
}
bool ImGuiOverlay::toggleButtonEnabled() const { return m_impl->showToggle; }
void ImGuiOverlay::setPanelsOpen(bool on) {
  m_impl->panelsOpen = on;
  m_impl->syncSettings();
}
bool ImGuiOverlay::panelsOpen() const { return m_impl->panelsOpen; }
void ImGuiOverlay::attachCamera(CameraController &cam) { m_impl->cam = &cam; }
void ImGuiOverlay::setVisible(bool on) {
  m_impl->visible = on;
  m_impl->syncSettings();
}
bool ImGuiOverlay::visible() const { return m_impl->visible; }
bool ImGuiOverlay::enabled() const { return m_impl->ready && !g_glBackendFailed; }
bool ImGuiOverlay::wantsMouse() const {
  return m_impl->ready && m_impl->visible && ImGui::GetIO().WantCaptureMouse;
}
bool ImGuiOverlay::wantsKeyboard() const {
  return m_impl->ready && m_impl->visible && ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiOverlay::setUiFontPath(const std::string &ttfPath) { g_uiFontPath = ttfPath; }

void ImGuiOverlay::setUiFontMemory(const void *ttf, int len, bool compressed) {
  g_uiFontMem = ttf;
  g_uiFontMemLen = len;
  g_uiFontMemCompressed = compressed;
}

} // namespace gl
} // namespace cvc

#else // !CVC_ENABLE_IMGUI — inert stubs so consumers need no #ifdef

namespace cvc {
namespace gl {

struct ImGuiOverlay::Impl {};

ImGuiOverlay::ImGuiOverlay(SceneRenderer &) : m_impl(new Impl) {}
ImGuiOverlay::~ImGuiOverlay() = default;
void ImGuiOverlay::setUiFontPath(const std::string &) {}
void ImGuiOverlay::setUiFontMemory(const void *, int, bool) {}
void ImGuiOverlay::setDrawCallback(std::function<void()>) {}
void ImGuiOverlay::setSceneDrawCallback(std::function<void()>) {}
ImGuiContext *ImGuiOverlay::imguiContext() const { return nullptr; }
// Every method in this block exists so consumers never need an #ifdef (see the
// file header). attachCamera was the one omission: the nav demos call it
// unguarded — exactly as the contract invites — so the three of them failed to
// link in the only configuration that builds without imgui, the libcvc-examples
// publish lane. Nothing to do without an overlay: with no imgui there is no
// input to hand off, so the camera never needs telling about it.
void ImGuiOverlay::attachCamera(CameraController &) {}
void ImGuiOverlay::setVisible(bool) {}
bool ImGuiOverlay::visible() const { return false; }
bool ImGuiOverlay::enabled() const { return false; }
void ImGuiOverlay::setUiScale(float) {}
float ImGuiOverlay::uiScale() const { return 1.0f; }
void ImGuiOverlay::setPixelRatio(float) {}
float ImGuiOverlay::pixelRatio() const { return 1.0f; }
void ImGuiOverlay::setTouchMode(bool) {}
bool ImGuiOverlay::touchMode() const { return false; }
void ImGuiOverlay::setFixedDeltaTime(float) {}
float ImGuiOverlay::fixedDeltaTime() const { return 0.0f; }
void ImGuiOverlay::setToggleButtonEnabled(bool) {}
bool ImGuiOverlay::toggleButtonEnabled() const { return false; }
void ImGuiOverlay::setPanelsOpen(bool) {}
bool ImGuiOverlay::panelsOpen() const { return true; }
bool ImGuiOverlay::wantsMouse() const { return false; }
bool ImGuiOverlay::wantsKeyboard() const { return false; }

} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI

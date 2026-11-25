// Clipboard — system copy/paste for cvcGL (see the header for the platform
// caveats, which are real and not uniform).
//
// There is no ImGui platform backend under us (the window is cvcGL's), so nothing
// supplies clipboard callbacks by default except on Windows, where Dear ImGui's
// core already ships a complete Win32 implementation. This file fills the gap
// everywhere else: SDL's clipboard under cvcGL's SDL window, and the platform's
// own under VTK's window.

#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/Clipboard.h>
#include <string>
#if defined(CVC_ENABLE_SDL) && !defined(__EMSCRIPTEN__)
#include <SDL3/SDL.h>
#endif

#if defined(__EMSCRIPTEN__)
#include <emscripten.h>

// The EM_JS bodies are JavaScript, which clang-format would mangle (it splits `===`).
// clang-format off

// COPY: navigator.clipboard.writeText is async and gated on transient
// activation (Firefox/Safari want a gesture every time; Chromium remembers the
// permission). We fire and forget — the write happens, we just cannot report
// per-call success synchronously. document.execCommand('copy') is the fallback
// for older/permission-less contexts.
EM_JS(void, cvcgl_clip_write, (const char *text), {
  var s = UTF8ToString(text);
  var C = window.__cvcglClip;
  if (C && C.copied && C.copied.text === s && Date.now() - C.copied.t < 3000) {
    C.copied = null; // the pointer release that picked this Copy has written it (arm)
    return;
  }
  try {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(s).catch(function() { cvcglClipFallback(s); });
      return;
    }
  } catch (e) {
  }
  cvcglClipFallback(s);
});

// PASTE: ImGui's getter is SYNCHRONOUS; navigator.clipboard.readText() is not,
// and in Firefox/Safari it prompts on every read. So instead of lying or
// blocking, we cache what the PAGE receives from a real paste event (Ctrl+V,
// long-press → Paste), which the browser delivers synchronously to a listener.
// A paste the app never received reads back as empty.
EM_JS(void, cvcgl_clip_install, (), {
  if (window.cvcglClipFallback)
    return;
  window.__cvcglClipText = '';
  window.cvcglClipFallback = function(s) {
    // Legacy path: a throwaway textarea + execCommand('copy').
    var ta = document.createElement('textarea');
    ta.value = s;
    ta.setAttribute('readonly', '');
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.appendChild(ta);
    ta.select();
    try {
      document.execCommand('copy');
    } catch (e) {
    }
    document.body.removeChild(ta);
  };
  // A real paste event is the one moment the browser hands us clipboard text
  // synchronously and without a permission prompt.
  document.addEventListener(
      'paste', function(e) {
        try {
          window.__cvcglClipText = (e.clipboardData || window.clipboardData).getData('text');
        } catch (err) {
        }
      });
  // A context menu's Copy and Paste (arm): the item under the pointer, and the clipboard call
  // made inside the pointer release over it. A mouse's press must have been over the same item
  // too, as a click's is; a finger's has no hover before it lands, so its release is enough.
  var C = window.__cvcglClip = {arm: null, downArm: null, copied: null, gesture: null, reads: {},
                                next: 1};
  C.read = function() {
    var h = {state: 0, text: ''};
    try {
      navigator.clipboard.readText().then(function(t) {
        h.text = t;
        h.state = 1;
      }, function() { h.state = 2; });
    } catch (e) {
      h.state = 2; // no clipboard API (an insecure page, an old browser)
    }
    return h;
  };
  window.addEventListener('pointerdown', function(e) {
    C.downArm = e.button === 0 ? C.arm : null;
  }, true);
  window.addEventListener(
      'pointerup', function(e) {
        var a = C.arm;
        if (!a || e.button !== 0 || (a !== C.downArm && e.pointerType === 'mouse'))
          return;
        if (a.copy !== null) {
          C.copied = {text: a.copy, t: Date.now()};
          try {
            navigator.clipboard.writeText(a.copy).catch(function() { C.copied = null; });
          } catch (err) {
            C.copied = null; // set() tries again, with its fallback
          }
        }
        if (a.paste)
          C.gesture = {h: C.read(), t: Date.now()};
      }, true);
});

EM_JS(void, cvcgl_clip_arm, (const char *copy, int paste), {
  var C = window.__cvcglClip;
  C.arm = copy || paste ? {copy: copy ? UTF8ToString(copy) : null, paste: !!paste} : null;
});

EM_JS(int, cvcgl_clip_begin_read, (), {
  var C = window.__cvcglClip;
  var h;
  if (C.gesture && Date.now() - C.gesture.t < 3000)
    h = C.gesture.h; // the read the release over Paste started
  else
    h = C.read();
  C.gesture = null;
  var id = C.next++;
  C.reads[id] = h;
  return id;
});

// 0 pending, 1 done (its text is cvcgl_clip_read_text's), 2 failed or unknown.
EM_JS(int, cvcgl_clip_poll_read, (int id), {
  var h = window.__cvcglClip.reads[id];
  return h ? h.state : 2;
});

EM_JS(char *, cvcgl_clip_read_text, (int id), {
  var C = window.__cvcglClip;
  var s = C.reads[id] ? C.reads[id].text : '';
  delete C.reads[id];
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

EM_JS(char *, cvcgl_clip_read, (), {
  var s = window.__cvcglClipText || '';
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});
// clang-format on

#elif defined(_WIN32)
// Nothing here: Dear ImGui core already implements the Win32 clipboard
// (OpenClipboard/CF_UNICODETEXT) as its default handler, so we forward to it
// rather than maintain a second copy. See canCopy()/canPaste().

#elif defined(__APPLE__)
// Implemented in Clipboard_mac.mm (NSPasteboard) when the Cocoa bits are built.
extern "C" bool cvcgl_mac_clip_set(const char *);
std::string cvcgl_mac_clip_get(); // the pasteboard's text, empty when it has none

#else
// ---- X11 -------------------------------------------------------------------
// X11 has no clipboard SERVER: a selection is owned by a live client, which
// answers requests for its contents. So "copy" means "become the owner of
// CLIPBOARD", and text vanishes when the app exits unless a clipboard manager
// takes over. That is X11's model, not a shortcut here.
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <cstring>
#include <mutex>
#include <thread>

namespace {

struct X11Clip {
  Display *dpy = nullptr;
  Window win = 0;
  Atom clipboard = 0, utf8 = 0, target = 0;
  std::string owned; // what we currently offer as CLIPBOARD owner
  bool ok = false;

  X11Clip() {
    dpy = XOpenDisplay(nullptr);
    if (!dpy)
      return;
    const int scr = DefaultScreen(dpy);
    // An unmapped 1x1 window is enough to own a selection and receive requests.
    win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 0, 0, 1, 1, 0, 0, 0);
    clipboard = XInternAtom(dpy, "CLIPBOARD", False);
    utf8 = XInternAtom(dpy, "UTF8_STRING", False);
    target = XInternAtom(dpy, "CVCGL_CLIP", False);
    ok = true;
  }
  ~X11Clip() {
    if (dpy) {
      if (win)
        XDestroyWindow(dpy, win);
      XCloseDisplay(dpy);
    }
  }

  // Answer SelectionRequest events for as long as we own the selection. Called
  // opportunistically (non-blocking) so a paste from another app works while
  // the render loop keeps running.
  void pump() {
    if (!ok)
      return;
    while (XPending(dpy)) {
      XEvent ev;
      XNextEvent(dpy, &ev);
      if (ev.type != SelectionRequest)
        continue;
      const XSelectionRequestEvent &rq = ev.xselectionrequest;
      XSelectionEvent resp{};
      resp.type = SelectionNotify;
      resp.display = rq.display;
      resp.requestor = rq.requestor;
      resp.selection = rq.selection;
      resp.target = rq.target;
      resp.time = rq.time;
      resp.property = None;
      if (rq.target == utf8 || rq.target == XA_STRING) {
        XChangeProperty(dpy, rq.requestor, rq.property, rq.target, 8, PropModeReplace,
                        reinterpret_cast<const unsigned char *>(owned.c_str()),
                        static_cast<int>(owned.size()));
        resp.property = rq.property;
      }
      XSendEvent(dpy, rq.requestor, True, NoEventMask, reinterpret_cast<XEvent *>(&resp));
      XFlush(dpy);
    }
  }

  bool set(const std::string &s) {
    if (!ok)
      return false;
    owned = s;
    XSetSelectionOwner(dpy, clipboard, win, CurrentTime);
    XFlush(dpy);
    return XGetSelectionOwner(dpy, clipboard) == win;
  }

  std::string get() {
    if (!ok)
      return {};
    if (XGetSelectionOwner(dpy, clipboard) == win)
      return owned; // we own it: skip the round trip
    XConvertSelection(dpy, clipboard, utf8, target, win, CurrentTime);
    XFlush(dpy);
    // Bounded wait: the owner is another process and may be slow or gone. This
    // runs on the render thread, so a hang would freeze the app — cap it.
    for (int i = 0; i < 100; ++i) {
      XEvent ev;
      if (XCheckTypedWindowEvent(dpy, win, SelectionNotify, &ev)) {
        if (ev.xselection.property == None)
          return {};
        Atom type = 0;
        int fmt = 0;
        unsigned long items = 0, bytes = 0;
        unsigned char *data = nullptr;
        if (XGetWindowProperty(dpy, win, target, 0, 1 << 20, True, AnyPropertyType, &type, &fmt,
                               &items, &bytes, &data) == Success &&
            data) {
          std::string out(reinterpret_cast<char *>(data), items);
          XFree(data);
          return out;
        }
        return {};
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return {}; // timed out: better empty than frozen
  }
};

// An app's X connection for the clipboard: made on its first copy or paste and closed with the
// app. Each app is its own X client, so the one that copied last owns CLIPBOARD (another app of
// the process pasting asks it, as any other client would). The mutex serialises the app's calls:
// Xlib is not thread-safe on a connection without XInitThreads.
struct X11ClipSlot {
  static constexpr const char *app_slot = "cvc.gl.clipboard.x11";
  std::mutex m;
  X11Clip clip;
};

} // namespace
#endif

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace cvc {
namespace gl {
namespace clipboard {

// Under cvcGL's SDL window (SdlWindow.h) SDL's video is up, and its clipboard is the one to use on
// every desktop platform, Wayland included.
#if defined(CVC_ENABLE_SDL) && !defined(__EMSCRIPTEN__)
static bool sdl_clipboard() { return SDL_WasInit(SDL_INIT_VIDEO) != 0; }
#endif

bool set(cvc::app &app, const std::string &text) {
  (void)app; // the X11 clipboard's; the other platforms' need none
#if defined(CVC_ENABLE_SDL) && !defined(__EMSCRIPTEN__)
  if (sdl_clipboard())
    return SDL_SetClipboardText(text.c_str());
#endif
#if defined(__EMSCRIPTEN__)
  cvcgl_clip_install();
  cvcgl_clip_write(text.c_str());
  return true;
#elif defined(_WIN32)
#ifdef CVC_ENABLE_IMGUI
  ImGui::SetClipboardText(text.c_str()); // imgui core's Win32 implementation
  return true;
#else
  (void)text;
  return false;
#endif
#elif defined(__APPLE__)
  return cvcgl_mac_clip_set(text.c_str());
#else
  X11ClipSlot &x = app.slot<X11ClipSlot>();
  std::lock_guard<std::mutex> lock(x.m);
  return x.clip.set(text);
#endif
}

std::string get(cvc::app &app) {
  (void)app; // the X11 clipboard's; the other platforms' need none
#if defined(CVC_ENABLE_SDL) && !defined(__EMSCRIPTEN__)
  if (sdl_clipboard()) {
    char *t = SDL_GetClipboardText();
    std::string s = t ? t : "";
    SDL_free(t);
    return s;
  }
#endif
#if defined(__EMSCRIPTEN__)
  cvcgl_clip_install();
  char *p = cvcgl_clip_read();
  std::string s = p ? p : "";
  std::free(p);
  return s;
#elif defined(_WIN32)
#ifdef CVC_ENABLE_IMGUI
  const char *t = ImGui::GetClipboardText();
  return t ? t : "";
#else
  return {};
#endif
#elif defined(__APPLE__)
  return cvcgl_mac_clip_get();
#else
  X11ClipSlot &x = app.slot<X11ClipSlot>();
  std::lock_guard<std::mutex> lock(x.m);
  x.clip.pump(); // answer any pending requests before asking
  return x.clip.get();
#endif
}

bool canCopy() {
#if defined(_WIN32) && !defined(CVC_ENABLE_IMGUI)
  return false;
#else
  return true;
#endif
}

bool canPaste() {
#if defined(__EMSCRIPTEN__)
  // Only via a real paste event the page received — never an arbitrary read.
  return true;
#elif defined(_WIN32) && !defined(CVC_ENABLE_IMGUI)
  return false;
#else
  return true;
#endif
}

bool readIsAsync() {
#if defined(__EMSCRIPTEN__)
  return true;
#else
  return false;
#endif
}

void arm(const std::string *copy, bool paste) {
#if defined(__EMSCRIPTEN__)
  cvcgl_clip_install();
  cvcgl_clip_arm(copy ? copy->c_str() : nullptr, paste ? 1 : 0);
#else
  (void)copy;
  (void)paste;
#endif
}

int beginRead() {
#if defined(__EMSCRIPTEN__)
  cvcgl_clip_install();
  return cvcgl_clip_begin_read();
#else
  return 0;
#endif
}

ReadState pollRead(int ticket, std::string &text) {
#if defined(__EMSCRIPTEN__)
  if (ticket <= 0)
    return ReadState::Failed;
  const int st = cvcgl_clip_poll_read(ticket);
  if (st == 0)
    return ReadState::Pending;
  char *p = cvcgl_clip_read_text(ticket); // spends the ticket either way
  if (st == 1)
    text = p ? p : "";
  std::free(p);
  return st == 1 ? ReadState::Done : ReadState::Failed;
#else
  (void)ticket;
  (void)text;
  return ReadState::Failed;
#endif
}

} // namespace clipboard
} // namespace gl
} // namespace cvc

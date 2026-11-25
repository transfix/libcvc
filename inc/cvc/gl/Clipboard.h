#ifndef CVC_GL_CLIPBOARD_H
#define CVC_GL_CLIPBOARD_H

#include <string>

namespace cvc {
class app;
namespace gl {

// -----------
// Clipboard
// -----------
// System clipboard access for cvcGL apps — and the copy/paste ImGui uses.
//
//     cvc::gl::clipboard::set(app, "hello");     // copy
//     std::string s = cvc::gl::clipboard::get(app);
//
// ImGuiOverlay installs these into ImGui automatically, so Ctrl+C / Ctrl+V in an
// InputText Just Work wherever the platform allows it. There is no ImGui
// platform backend under us (the window is cvcGL's, SDL's or VTK's), so this
// is the layer that fills that gap.
//
// PLATFORM REALITY — this is not uniform, and pretending otherwise would be a
// lie:
//   * Under cvcGL's SDL3 window (the onscreen window on Windows, Linux and
//     macOS when cvcGL has SDL3): SDL's clipboard, Wayland included. The
//     platform notes below are for VTK's own window ($CVCGL_SDL_WINDOW=0, or a
//     cvcGL without SDL3) and the browser.
//   * Windows: Dear ImGui's core already ships a complete Win32 implementation
//     (OpenClipboard/CF_UNICODETEXT), so we leave ImGui's default in place and
//     these functions forward to it. Nothing to do, nothing to maintain.
//   * X11/Linux: implemented here (an X11 selection owner + a request round
//     trip). Note the X11 rule that a selection belongs to a live client: text
//     copied from a cvcGL app disappears when the app exits, unless a clipboard
//     manager takes ownership. That is X11, not a bug here.
//   * macOS: NSPasteboard, when built with the Cocoa bits.
//   * WebAssembly: COPY works (navigator.clipboard.writeText). PASTE cannot be
//     synchronous — ImGui's getter must return immediately, while the browser's
//     readText() is async, secure-context-only and permission/gesture gated
//     (Firefox and Safari prompt on EVERY read). So get() returns the most
//     recent text the PAGE saw via a real paste event (Ctrl+V / long-press
//     Paste), which the browser delivers synchronously to a listener. A paste
//     the app never received returns empty rather than blocking or lying.
//
// available() tells you which half you actually have, so a UI can grey out a
// Paste button instead of silently doing nothing.
namespace clipboard {

// Copy `text` to the system clipboard. Returns false when the platform refused
// (e.g. a browser without transient activation). `app` holds what the platform
// needs for it: on X11 the app's own X connection, which owns the selection
// until another client copies or the app is destroyed.
bool set(cvc::app &app, const std::string &text);

// The clipboard's current text, or "" when unavailable (see the wasm note).
std::string get(cvc::app &app);

// What this build/platform can actually do.
bool canCopy();
bool canPaste();

// A Copy or Paste chosen from a context menu (docs/ARIADNE.md §5.28), in a browser. Natively
// get() and set() answer at once, so a menu calls them and none of this is needed: readIsAsync()
// is false there, and the rest do nothing.
//
// In a browser a menu's Paste cannot use get(): no paste event came with it. It asks
// navigator.clipboard.readText() instead, which answers later. Chrome asks the user once for the
// permission; Firefox and Safari show their own Paste button each time, and Safari answers only
// within the click itself. ImGui sees a click a frame after the browser does, so arm() says what
// the item under the pointer is (a Copy and the text it copies, or a Paste), and a page listener
// makes the call inside the pointer release over it. beginRead() then takes the read that release
// started, or starts one (still within the click's user activation, which is enough for Chrome
// and Firefox), and pollRead() says when it answered. set() skips a text the release has just
// written.
enum class ReadState { Pending, Done, Failed };
bool readIsAsync();
// `copy`: the text a Copy under the pointer copies (null: none); `paste`: a Paste is under it.
// Call it when the item under the pointer changes, and with (nullptr, false) when there is none.
void arm(const std::string *copy, bool paste);
// A read's ticket, for pollRead (0 natively).
int beginRead();
// Pending until the browser answers; then Done with `text` filled, or Failed (the permission or
// the prompt refused, no clipboard API, an insecure page). The ticket is spent once it answers.
ReadState pollRead(int ticket, std::string &text);

} // namespace clipboard
} // namespace gl
} // namespace cvc

#endif // CVC_GL_CLIPBOARD_H

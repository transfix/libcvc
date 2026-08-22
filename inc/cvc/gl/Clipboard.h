#ifndef CVC_GL_CLIPBOARD_H
#define CVC_GL_CLIPBOARD_H

#include <string>

namespace cvc {
namespace gl {

// -----------
// Clipboard
// -----------
// System clipboard access for cvcGL apps — and the copy/paste ImGui uses.
//
//     cvc::gl::clipboard::set("hello");     // copy
//     std::string s = cvc::gl::clipboard::get();
//
// ImGuiOverlay installs these into ImGui automatically, so Ctrl+C / Ctrl+V in an
// InputText Just Work wherever the platform allows it. There is no platform
// backend under us (VTK owns the window and exposes no clipboard API), so this
// is the layer that fills that gap.
//
// PLATFORM REALITY — this is not uniform, and pretending otherwise would be a
// lie:
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
// (e.g. a browser without transient activation).
bool set(const std::string &text);

// The clipboard's current text, or "" when unavailable (see the wasm note).
std::string get();

// What this build/platform can actually do.
bool canCopy();
bool canPaste();

} // namespace clipboard
} // namespace gl
} // namespace cvc

#endif // CVC_GL_CLIPBOARD_H

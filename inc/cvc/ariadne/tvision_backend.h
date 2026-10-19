#ifndef CVC_ARIADNE_TVISION_BACKEND_H
#define CVC_ARIADNE_TVISION_BACKEND_H

// TvisionBackend — the Ariadne terminal backend with a native desktop (roadmap §16.4 / §16.5 row
// 5). Turbo Vision (magiblot's tvision) owns the terminal and draws the chrome natively: real
// movable, resizable, overlapping windows with close and zoom boxes, a pull-down menu bar, and a
// status line. It is the terminal counterpart of Ariadne's default `free` desktop layout.
//
// What is inside each window is the FTXUI backend's. A TvisionBackend is an FtxuiBackend in hosted
// mode (FtxuiBackend::Options::hosted): every widget's layout, focus, editing and commit rule is
// shared with the FTXUI backend, which lays each window's body out offscreen for tvision to draw
// and takes the window's input in its own cells. FTXUI is only a layout engine here; it never
// touches the terminal.
//
// Built when libcvc finds tvision (+ ncursesw) and FTXUI and builds shared (CVC_ARIADNE_TVISION —
// the feature flag the roadmap keeps for tvision's Borland-derived licensing). Otherwise
// available() is false and run() returns at once. tvision keeps process-wide state (one
// TApplication per process), so only one TvisionBackend may run at a time.
//
// Keys, inside the desktop: tvision's own (F10 menu, F6 next window, Alt-X exit, Ctrl-F5 move /
// resize, the mouse on frames) plus the FTXUI backend's inside a window (Tab, arrows, Enter /
// Space, text entry, ...). Ctrl-C is SIGINT for the document's on_signal:, as in the FTXUI loop.

#include <cvc/ariadne/ftxui_backend.h>

namespace cvc {
namespace ariadne {

class TvisionBackend : public FtxuiBackend {
public:
  TvisionBackend();
  explicit TvisionBackend(Options opts); // opts.hosted is forced on

  Capabilities capabilities() const override;

  // Run the tvision desktop on the terminal until a quit (Alt-X, a quit action, should_close) or an
  // interrupt the document does not handle; the same RunOptions and signal routing as
  // FtxuiBackend::run. These hide FtxuiBackend's run overloads: a TvisionBackend always runs
  // tvision.
  RunExit run(Runtime &rt);
  RunExit run(Runtime &rt, const RunOptions &opts);
  RunExit run(AppRuntime &app);
  RunExit run(AppRuntime &app, const RunOptions &opts);

  // Whether this build has the tvision backend.
  static bool available();

private:
  RunExit run_desktop(Runtime &rt, const RunOptions &opts, const std::function<void()> &drain,
                      const std::function<bool()> &should_close,
                      const std::function<void(const TermEvent &)> &unhandled);
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_TVISION_BACKEND_H

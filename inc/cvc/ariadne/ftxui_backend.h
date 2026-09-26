#ifndef CVC_ARIADNE_FTXUI_BACKEND_H
#define CVC_ARIADNE_FTXUI_BACKEND_H

// FtxuiBackend — a pure-terminal Ariadne backend built on FTXUI. It proves the
// Backend seam (roadmap §16): it renders the SAME cvc::ariadne Widget tree the
// ImGui/VTK backend renders, with no ImGui, no VTK, no GL — just a terminal.
//
// It is the framework-owned-loop counterpart to the host-pumped ImGui backend:
// capabilities().owns_loop == true (FTXUI's ScreenInteractive owns the event
// loop and drives Runtime::render()), where cvc::gl::ImGuiBackend is
// owns_loop == false (VTK's render pass drives it). Exercising both across one
// interface is exactly what the seam had to support.
//
// Optional: compiled with the real FTXUI only when libcvc is built with it
// (CVC_ARIADNE_FTXUI, via find_package(ftxui)); otherwise every method is an
// inert stub and available() is false, so libcvc still builds without FTXUI.
//
// The FTXUI types are hidden behind a Pimpl, so this header pulls in no FTXUI
// headers — a consumer needs FTXUI only to LINK a build that uses it.

#include <memory>
#include <string>
#include <vector>

#include <cvc/ariadne/backend.h>

namespace cvc {
namespace ariadne {

class Runtime;

class FtxuiBackend : public Backend {
public:
  FtxuiBackend();
  ~FtxuiBackend() override;

  FtxuiBackend(const FtxuiBackend &) = delete;
  FtxuiBackend &operator=(const FtxuiBackend &) = delete;

  Capabilities capabilities() const override;

  void begin_frame() override;
  void end_frame() override;

  bool begin_main_menu_bar() override;
  void end_main_menu_bar() override;
  bool begin_menu(const char *label) override;
  void end_menu() override;
  bool begin_window(const char *title, const char *id, const Size &size, float border) override;
  void end_window() override;
  bool begin_grid(const Layout &layout, const char *id) override;
  void grid_next_cell() override;
  void end_grid() override;
  void push_id(const char *id) override;
  void pop_id() override;

  void text_line(const char *text) override;
  void text_value(const char *label, const std::string &value) override;
  void separator() override;
  bool button(const char *label) override;
  bool menu_item_action(const char *label) override;

  BoolEdit menu_item_toggle(const char *label, bool current) override;
  BoolEdit checkbox(const char *label, bool current) override;
  IntEdit slider_int(const char *label, const char *key, int current, int lo, int hi) override;
  DoubleEdit slider_double(const char *label, const char *key, double current, double lo, double hi,
                           const char *fmt) override;
  IndexEdit combo(const char *label, int current_index,
                  const std::vector<std::string> &options) override;

  // Render the current tree to a plain string (headless — for tests / capture /
  // an SSH log). Calls rt.render() to (re)build the frame, then rasterizes it at
  // `width` columns. This is the seam proof that needs no TTY.
  std::string render_to_string(Runtime &rt, int width = 80);

  // Run FTXUI's interactive event loop on a real terminal (owns_loop): each
  // frame calls rt.render() then rt.drain(). Blocks until the loop exits (e.g. a
  // "quit" action, wired via on()). A no-op in a build without FTXUI.
  void run(Runtime &rt);

  // Whether this build was compiled with FTXUI.
  static bool available();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_FTXUI_BACKEND_H

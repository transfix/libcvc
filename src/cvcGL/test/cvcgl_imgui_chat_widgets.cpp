// ImGuiBackend's text_view and message_input (docs/ARIADNE.md §5.14) on a bare ImGui context (no
// GL), driven by synthesized keys and the mouse wheel: Enter sends and keeps the focus, Shift+Enter
// adds a line and the field grows; the log starts at its bottom, the wheel takes it to the top
// (on_top), lines added above keep it in place, a text that fits asks once, and a log of height 0
// leaves room for the field under it.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <string>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace ari = cvc::ariadne;

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

#ifdef CVC_ENABLE_IMGUI
// The ImGui backend, keeping where the log and the field were drawn and whether the window had to
// scroll to fit them.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  ImVec2 log_min, log_max, field_size;
  float window_overflow = 0.0f;
  ari::TextViewEvent text_view(const char *label, const char *key, const std::string &text,
                               const ari::TextViewOptions &o) override {
    const ari::TextViewEvent e = ImGuiBackend::text_view(label, key, text, o);
    log_min = ImGui::GetItemRectMin();
    log_max = ImGui::GetItemRectMax();
    return e;
  }
  ari::MessageEdit message_input(const char *label, const char *key, const std::string &current,
                                 const ari::MessageInputOptions &o) override {
    const ari::MessageEdit e = ImGuiBackend::message_input(label, key, current, o);
    field_size = ImGui::GetItemRectSize();
    window_overflow = ImGui::GetScrollMaxY();
    return e;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, ""};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800.0f, 600.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    frames(3);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame() {
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    rt.drain();
  }
  void frames(int n) {
    for (int i = 0; i < n; ++i)
      frame();
  }
  void key(ImGuiKey k, bool shift = false) {
    ImGuiIO &io = ImGui::GetIO();
    if (shift)
      io.AddKeyEvent(ImGuiMod_Shift, true);
    io.AddKeyEvent(k, true);
    frame();
    io.AddKeyEvent(k, false);
    if (shift)
      io.AddKeyEvent(ImGuiMod_Shift, false);
    frame();
  }
  void type(const char *s) {
    ImGui::GetIO().AddInputCharactersUTF8(s);
    frame();
  }
  std::string val(const char *path) { return cvc::state::instance(app)(path).value(); }
  void set(const char *path, const std::string &v) { cvc::state::instance(app)(path).value(v); }
};

std::string lines(int from, int to) {
  std::string out;
  for (int i = from; i < to; ++i)
    out += (out.empty() ? "" : "\n") + std::string("line ") + std::to_string(i);
  return out;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget log =
      ari::text_view("##log", "log", "(state-set \"tops\" (str (+ (int (state-get \"tops\")) 1)))");
  ari::Widget say = ari::message_input(
      "##say", "draft",
      "(state-set \"sent\" (str-concat (state-get \"sent\") \"|\" (get-attr event \"text\")))");
  say.autofocus = true;
  ari::Widget win = ari::window("Chat", {log, say});
  win.size.w = {ari::Unit::Px, 420.0f};
  win.size.h = {ari::Unit::Px, 360.0f};
  win.has_pos = true;
  win.pos_x = win.pos_y = 20.0f;
  Session s(win);
  s.set("tops", "0");
  s.set("sent", "");
  s.set("log", lines(0, 200));
  s.frames(4);

  // The field: Enter sends, Shift+Enter adds a line, the focus stays.
  const float one_line = s.be.field_size.y;
  s.type("hello");
  s.key(ImGuiKey_Enter, /*shift=*/true);
  s.type("world");
  const float two_lines = s.be.field_size.y;
  chk(two_lines > one_line + 0.5f * ImGui::GetTextLineHeight(),
      "a second line makes the field taller (" + std::to_string(one_line) + " -> " +
          std::to_string(two_lines) + ")");
  s.key(ImGuiKey_Enter);
  s.frames(3);
  chk(s.val("sent") == "|hello\nworld",
      "Enter sends the two lines as event.text: " + s.val("sent"));
  chk(s.val("draft").empty(), "and the draft is cleared");
  s.frames(2);
  chk(s.be.field_size.y < two_lines, "the empty field is one line again");
  s.type("again");
  s.key(ImGuiKey_Enter);
  s.frames(3);
  chk(s.val("sent") == "|hello\nworld|again", "the focus stayed in the field: " + s.val("sent"));
  chk(s.be.window_overflow < 1.0f, "the log fills the window less the field: nothing overflows");

  // The log: it starts at its bottom, so it has not asked for more.
  chk(s.val("tops") == "0", "a log at its bottom does not ask for older lines");
  ImGuiIO &io = ImGui::GetIO();
  const ImVec2 mid((s.be.log_min.x + s.be.log_max.x) * 0.5f,
                   (s.be.log_min.y + s.be.log_max.y) * 0.5f);
  io.AddMousePosEvent(mid.x, mid.y);
  s.frame();
  for (int i = 0; i < 40 && s.val("tops") == "0"; ++i) {
    io.AddMouseWheelEvent(0.0f, 10.0f);
    s.frame();
  }
  s.frames(2);
  chk(s.val("tops") != "0", "wheeled to the top, it asks for older lines (on_top)");
  const std::string after_top = s.val("tops");

  // Older lines above: the view stays where it was, so it is not at the top any more.
  s.set("log", lines(-50, 200));
  s.frames(6);
  chk(s.val("tops") == after_top, "lines added above keep it in place: no new on_top");
  for (int i = 0; i < 60 && s.val("tops") == after_top; ++i) {
    io.AddMouseWheelEvent(0.0f, 10.0f);
    s.frame();
  }
  chk(s.val("tops") != after_top, "back at the (new) top, it asks again");

  // A text that fits asks once.
  s.set("tops", "0");
  s.set("log", lines(0, 3));
  s.frames(8);
  chk(s.val("tops") == "1", "a text that fits asks for more once: " + s.val("tops"));
  s.set("log", "");
  s.set("tops", "0");
  s.frames(4);
  chk(s.val("tops") == "0", "an empty log does not");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

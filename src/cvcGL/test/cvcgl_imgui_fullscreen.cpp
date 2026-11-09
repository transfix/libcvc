// ImGuiBackend's fullscreen view (§5.13) on a bare ImGui context (no GL): a tap on a picture with
// fullscreen: true, through ImGui's own mouse input, asks the host's handler to make the window
// cover the screen and draws the picture alone, as wide as the view; Esc puts both back. And
// (clipboard-set TEXT) and (fullscreen ON) are there for a program.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <string>
#include <vector>
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
// The ImGui backend, noting where the picture and the text were drawn this frame. The picture is
// laid out as a box of its size rather than uploaded: there is no GL here (texture upload is
// draw_image's own path, which cvcgl_stream_texture covers).
struct RecordingBackend : cvc::gl::ImGuiBackend {
  ImVec2 pic_min, pic_max;
  float pic_w = 0.0f;
  bool drew_text = false;
  bool draw_image(const char *, float width) override {
    ImGui::Dummy(ImVec2(width, width * 0.5f));
    pic_min = ImGui::GetItemRectMin();
    pic_max = ImGui::GetItemRectMax();
    pic_w = width;
    return true;
  }
  bool draw_image_fit(const char *name, float box_w, float) override {
    return draw_image(name, box_w);
  }
  void text_line(const char *text) override {
    ImGuiBackend::text_line(text);
    drew_text = true;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  std::vector<bool> asked; // the fullscreen handler's calls
  bool os = false;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 700.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    be.set_fullscreen_handler(
        [this](bool on) {
          asked.push_back(on);
          os = on;
          return true;
        },
        [this] { return os; });
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(990.0f, 690.0f), false);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  void frame(ImVec2 mouse, bool down, bool esc = false) {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, down);
    io.AddKeyEvent(ImGuiKey_Escape, esc);
    io.DeltaTime = 1.0f / 60.0f;
    be.drew_text = false;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    for (int i = 0; i < 20; ++i)
      rt.drain();
  }
  void tap(ImVec2 at) {
    frame(at, false);
    frame(at, true);
    frame(at, false);
    frame(at, false);
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget pic = ari::image_widget("pic", "photo", 64.0);
  pic.fullscreen_control = true;
  Session s(ari::window("Post", {pic, ari::text("the post's text"),
                                 ari::button("Copy", "(state-set \"copied\" (if (clipboard-set "
                                                     "\"https://tx.wtf\") \"yes\" \"no\"))")}));
  cvc::image img(8, 8, cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
  s.be.set_image("photo", img);
  s.frame(ImVec2(990.0f, 690.0f), false);
  chk(s.be.capabilities().fullscreen && s.be.capabilities().fullscreen_view,
      "with a handler, the window can cover the screen (ui.fullscreen, image.fullscreen)");
  chk(s.rt.has("clipboard-set") == s.be.capabilities().clipboard,
      "(has? \"clipboard-set\") follows the clipboard");
  chk(s.be.pic_w == 64.0f && s.be.drew_text, "the picture at its own width, among the rest");

  const ImVec2 c((s.be.pic_min.x + s.be.pic_max.x) * 0.5f,
                 (s.be.pic_min.y + s.be.pic_max.y) * 0.5f);
  s.tap(c);
  chk(!s.asked.empty() && s.asked.back(), "a tap on it asks for the window's fullscreen");
  chk(s.be.pic_w > 900.0f, "and shows it as wide as the view (" + std::to_string(s.be.pic_w) + ")");
  chk(!s.be.drew_text, "alone");

  s.frame(ImVec2(990.0f, 690.0f), false, /*esc=*/true);
  s.frame(ImVec2(990.0f, 690.0f), false);
  s.frame(ImVec2(990.0f, 690.0f), false);
  chk(!s.os && s.asked.size() >= 2 && !s.asked.back(), "Esc puts the window back");
  chk(s.be.pic_w == 64.0f && s.be.drew_text, "and the picture back among the rest");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

// ImGuiBackend's badge: on a picture (§5.11) on a bare ImGui context (no GL): the picture dimmed,
// the text centred on it, both on the window's draw list, so a tap on the label is the picture's
// click. An empty badge draws nothing over it.
#include <cmath>
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
// The ImGui backend with the picture laid out as a box of its size rather than uploaded: there is
// no GL here (texture upload is draw_image's own path, which cvcgl_stream_texture covers).
struct BoxBackend : cvc::gl::ImGuiBackend {
  ImVec2 pic_min, pic_max;
  const ImDrawList *dl = nullptr; // the picture's window's, kept until the next frame
  bool draw_image(const char *, float width) override {
    ImGui::Dummy(ImVec2(width, width * 0.75f));
    pic_min = ImGui::GetItemRectMin();
    pic_max = ImGui::GetItemRectMax();
    dl = ImGui::GetWindowDrawList();
    return true;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  BoxBackend be;
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
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(790.0f, 590.0f), false);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  void frame(ImVec2 mouse, bool down) {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, down);
    io.DeltaTime = 1.0f / 60.0f;
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

  // What the last frame drew over the picture, on its window's draw list: a dimming rectangle
  // exactly its size, and the bounds of the white vertices inside it (the text).
  bool dimmed = false;
  ImVec2 text_min{1e9f, 1e9f}, text_max{-1e9f, -1e9f};
  void scan() {
    dimmed = false;
    text_min = ImVec2(1e9f, 1e9f);
    text_max = ImVec2(-1e9f, -1e9f);
    const ImDrawList *l = be.dl;
    if (!l)
      return;
    const ImVec2 a = be.pic_min, z = be.pic_max;
    bool at_a = false, at_z = false;
    for (const ImDrawVert &v : l->VtxBuffer) {
      if (v.col == IM_COL32(0, 0, 0, 140)) {
        at_a = at_a || (v.pos.x == a.x && v.pos.y == a.y);
        at_z = at_z || (v.pos.x == z.x && v.pos.y == z.y);
      }
      const bool inside = v.pos.x > a.x && v.pos.x < z.x && v.pos.y > a.y && v.pos.y < z.y;
      if (inside && v.col == IM_COL32(255, 255, 255, 255)) {
        text_min = ImVec2(std::fmin(text_min.x, v.pos.x), std::fmin(text_min.y, v.pos.y));
        text_max = ImVec2(std::fmax(text_max.x, v.pos.x), std::fmax(text_max.y, v.pos.y));
      }
    }
    dimmed = at_a && at_z;
  }
  bool texted() const { return text_max.x > text_min.x; }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget pic = ari::image_widget("tile", "photo", 160.0);
  pic.badge = "(state-get \"more\")";
  pic.on_click = "(state-set \"opened\" \"1\")";
  Session s(ari::window("Post", {ari::text("the post's text"), pic}));
  cvc::state::instance(s.app)("ui.more").value(std::string("+3"));
  s.frame(ImVec2(790.0f, 590.0f), false);
  s.scan();
  chk(s.be.capabilities().badge && s.rt.has("image.badge"),
      "the backend shows badges (image.badge)");
  chk(s.dimmed, "the picture is dimmed under its badge");
  chk(s.texted(), "the text is drawn on the picture");
  const ImVec2 pc((s.be.pic_min.x + s.be.pic_max.x) * 0.5f,
                  (s.be.pic_min.y + s.be.pic_max.y) * 0.5f);
  const ImVec2 tc((s.text_min.x + s.text_max.x) * 0.5f, (s.text_min.y + s.text_max.y) * 0.5f);
  chk(std::fabs(tc.x - pc.x) < 3.0f && std::fabs(tc.y - pc.y) < 6.0f,
      "centred on it (text " + std::to_string(tc.x) + "," + std::to_string(tc.y) + ", picture " +
          std::to_string(pc.x) + "," + std::to_string(pc.y) + ")");
  chk(s.text_max.y - s.text_min.y > ImGui::GetFontSize(), "larger than a line of text");

  s.tap(pc);
  chk(s.val("opened") == "1", "a tap on the label is the picture's click");

  cvc::state::instance(s.app)("ui.more").value(std::string());
  s.frame(ImVec2(790.0f, 590.0f), false);
  s.scan();
  chk(!s.dimmed && !s.texted(), "an empty badge draws nothing over it");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

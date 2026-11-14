// ImGuiBackend's rich text (§5.20) on a bare ImGui context (no GL): a text: with runs: wraps to the
// width it has (size: bounding it), and each link in it is an item of its own, clicked through
// ImGui's own mouse input. A tap on a link runs on_link with its href, not the card around it; a
// tap on the paragraph's plain text is the card's; a press that drags (a finger scrolling a feed)
// is nobody's; keyboard navigation reaches a link and Enter activates it.
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
// The ImGui backend, noting where the paragraph was drawn and what it reported.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  ImVec2 min, max;
  int hovered = -1;
  ari::RichTextEvent rich_text(const std::vector<ari::TextRun> &runs,
                               const ari::RichTextOptions &o) override {
    const ari::RichTextEvent ev = ImGuiBackend::rich_text(runs, o);
    min = ImGui::GetItemRectMin();
    max = ImGui::GetItemRectMax();
    hovered = ev.hovered;
    return ev;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 700.0f);
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(990.0f, 690.0f), false);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  void frame(ImVec2 mouse, bool down, ImGuiKey key = ImGuiKey_None, bool key_down = false) {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, down);
    if (key != ImGuiKey_None)
      io.AddKeyEvent(key, key_down);
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    for (int i = 0; i < 20; ++i)
      rt.drain();
  }
  void tap(ImVec2 from, ImVec2 to) {
    frame(from, false);
    frame(from, true);
    frame(to, true);
    frame(to, false);
    frame(to, false);
  }
  void press(ImGuiKey key) {
    const ImVec2 away(990.0f, 690.0f);
    frame(away, false, key, true);
    frame(away, false, key, false);
    frame(away, false);
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
  void set(const std::string &key, const std::string &v) {
    cvc::state::instance(app)("ui." + key).value(v);
  }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget p = ari::text("");
  p.has_runs = true;
  p.runs = {ari::TextRunSpec{"Open this", "", "/post/7"},
            ari::TextRunSpec{" and some ordinary words after it, in "},
            ari::TextRunSpec{"bold", "bold"},
            ari::TextRunSpec{" and "},
            ari::TextRunSpec{"italic", "italic"},
            ari::TextRunSpec{", that wrap onto more lines than one."}};
  p.on_link = "(state-set \"opened\" (get-attr event \"href\"))";
  p.size.w = {ari::Unit::Px, 240.0f}; // size: 240, the widest it wraps at
  ari::Widget card = ari::group({p});
  card.on_click = "(state-set \"card\" \"1\")";
  ari::Widget feed = ari::window("Feed", {card, ari::text("below")});
  feed.size.w = {ari::Unit::Px, 500.0f};
  feed.size.h = {ari::Unit::Px, 300.0f};
  feed.has_pos = true;
  feed.pos_x = feed.pos_y = 20.0f;
  Session s(feed);
  const float lh = ImGui::GetTextLineHeight();
  const ImVec2 mn = s.be.min, mx = s.be.max;
  chk(mx.x - mn.x <= 241.0f && mx.x - mn.x > 120.0f,
      "it wraps at its size: (" + std::to_string(mx.x - mn.x) + " px)");
  chk(mx.y - mn.y >= 3.0f * lh - 0.5f,
      "on three lines or more (" + std::to_string((mx.y - mn.y) / lh) + ")");

  const ImVec2 link(mn.x + 6.0f, mn.y + lh * 0.5f);
  s.frame(link, false);
  chk(s.be.hovered == 0, "the pointer over the link is over link 0");
  s.tap(link, link);
  chk(s.val("opened") == "/post/7", "a tap on the link runs on_link (" + s.val("opened") + ")");
  chk(s.val("card").empty(), "and not the card's on_click (" + s.val("card") + ")");

  s.set("opened", "");
  const ImVec2 plain(mn.x + 6.0f, mx.y - lh * 0.5f); // the last line starts in plain text
  s.tap(plain, plain);
  chk(s.val("card") == "1", "a tap on its plain text is the card's (" + s.val("card") + ")");
  chk(s.val("opened").empty(), "not a link's (" + s.val("opened") + ")");

  s.set("card", "");
  s.tap(link, ImVec2(link.x, link.y + 3.0f * lh)); // pressed on the link, dragged away
  chk(s.val("opened").empty(), "a press that drags opens nothing (" + s.val("opened") + ")");

  s.set("opened", "");
  s.press(ImGuiKey_Tab); // keyboard navigation: the link is the first stop
  s.press(ImGuiKey_Enter);
  chk(s.val("opened") == "/post/7", "Tab then Enter activates the link (" + s.val("opened") + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

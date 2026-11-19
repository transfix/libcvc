// ImGuiBackend's item groups (§5.19) on a bare ImGui context (no GL): a group with on_click is one
// item over its whole area, clicked through ImGui's own mouse input. A tap on its text is the
// group's; a tap on a button inside it is the button's alone; a press that drags further than
// ImGui's drag threshold (a finger scrolling a feed) is nobody's. on_hover runs while the pointer
// is over any of it and on_hover_end once it leaves.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <map>
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
// The ImGui backend, noting where each text line and button and the group were drawn.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Rect {
    ImVec2 min, max;
    ImVec2 centre() const { return ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f); }
  };
  std::map<std::string, Rect> at;
  void note(const std::string &name) {
    at[name] = Rect{ImGui::GetItemRectMin(), ImGui::GetItemRectMax()};
  }
  void text_line(const char *text) override {
    ImGuiBackend::text_line(text);
    note(text);
  }
  bool button(const char *label) override {
    const bool hit = ImGuiBackend::button(label);
    note(label);
    return hit;
  }
  void end_item_group() override {
    ImGuiBackend::end_item_group();
    note("group");
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
  // Press at `from`, release at `to`, then let the UI settle.
  void tap(ImVec2 from, ImVec2 to) {
    frame(from, false);
    frame(from, true);
    frame(to, true);
    frame(to, false);
    frame(to, false);
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
  ari::Widget card =
      ari::group({ari::text("card text"), ari::button("Like", "(state-set \"liked\" \"1\")"),
                  ari::text("more card text")});
  card.on_click = "(state-set \"opened\" \"1\")";
  card.on_hover = "(state-set \"over\" \"1\")";
  card.on_hover_end = "(state-set \"left\" \"1\")";
  card.hover_highlight = true;
  // A widget of its own with on_click (as a feed's picture or video is): a tap, not a press.
  ari::Widget picture = ari::text("a picture");
  picture.on_click = "(state-set \"viewed\" \"1\")";
  ari::Widget feed =
      ari::window("Feed", {card, ari::text("below"), picture, ari::text("under the picture")});
  feed.size.w = {ari::Unit::Px, 500.0f};
  feed.size.h = {ari::Unit::Px, 300.0f};
  feed.has_pos = true;
  feed.pos_x = feed.pos_y = 20.0f;
  Session s(feed);
  chk(s.be.at.count("group") && s.be.at.count("card text") && s.be.at.count("Like"),
      "the group and what it holds are drawn");
  const RecordingBackend::Rect g = s.be.at["group"];
  chk(g.min.y <= s.be.at["card text"].min.y && g.max.y >= s.be.at["more card text"].max.y,
      "the group's item covers all of it");
  chk(s.be.at["below"].min.y >= g.max.y, "the widget after it is laid out below it");
  chk(g.max.x > s.be.at["more card text"].max.x + 100.0f,
      "it spans the row it has, wider than its text (a card's whole row)");

  const ImVec2 text = s.be.at["card text"].centre();
  s.tap(text, text);
  chk(s.val("opened") == "1", "a tap on its text opens it (" + s.val("opened") + ")");
  chk(s.val("over") == "1", "on_hover ran while the pointer was over it");

  s.set("opened", "");
  const ImVec2 like = s.be.at["Like"].centre();
  s.tap(like, like);
  chk(s.val("liked") == "1", "a tap on its button is the button's (" + s.val("liked") + ")");
  chk(s.val("opened").empty(), "and not the group's (" + s.val("opened") + ")");

  s.set("opened", "");
  const ImVec2 lower = s.be.at["more card text"].centre();
  s.tap(text, lower); // released over the group, but dragged two lines on the way
  chk(s.val("opened").empty(),
      "a press that drags is a scroll, not a click (" + s.val("opened") + ")");

  s.set("left", "");
  s.frame(like, true); // holding the button inside it: still over the group
  s.frame(like, true);
  s.frame(like, false);
  s.frame(text, false);
  chk(s.val("left").empty(),
      "on_hover_end waits while the pointer is over it (" + s.val("left") + ")");
  s.frame(ImVec2(990.0f, 690.0f), false);
  s.frame(ImVec2(990.0f, 690.0f), false);
  chk(s.val("left") == "1", "on_hover_end runs once the pointer leaves (" + s.val("left") + ")");

  // A widget's own on_click (§4.6) is a tap too: a finger that lands on a picture to scroll the
  // feed doesn't open it, though the press was on it.
  chk(s.be.at.count("a picture") && s.be.at.count("under the picture"),
      "the widget with on_click is drawn");
  const ImVec2 pic = s.be.at["a picture"].centre();
  const ImVec2 under = s.be.at["under the picture"].centre();
  s.frame(pic, false);
  s.frame(pic, true); // the press alone runs nothing
  chk(s.val("viewed").empty(), "a press on it runs nothing yet (" + s.val("viewed") + ")");
  s.frame(pic, false);
  s.frame(pic, false);
  chk(s.val("viewed") == "1",
      "released over it, unmoved: its on_click runs (" + s.val("viewed") + ")");
  s.set("viewed", "");
  s.tap(pic, under); // pressed on it, dragged a line down before letting go
  chk(s.val("viewed").empty(),
      "a press on it that drags is a scroll, not a click (" + s.val("viewed") + ")");
  s.set("viewed", "");
  s.tap(under, pic); // pressed elsewhere, released over it
  chk(s.val("viewed").empty(),
      "a press that went down elsewhere is not its click (" + s.val("viewed") + ")");

  chk(s.be.capabilities().item_groups, "ImGuiBackend reports item groups (ui.group_click)");
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

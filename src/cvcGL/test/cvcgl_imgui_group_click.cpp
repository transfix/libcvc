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
  bool button_with(const char *label, const ari::ButtonLook &look) override {
    const bool hit = ImGuiBackend::button_with(label, look);
    note(label);
    selected[label] = look.selected;
    return hit;
  }
  std::map<std::string, bool> selected; // §5.27 each button_with's look.selected, last drawn
  bool maximized = false;               // §5.9 the last window's maximize, as the core read it
  bool next_window_maximize(bool *m) override {
    const bool can = ImGuiBackend::next_window_maximize(m);
    maximized = *m;
    return can;
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
  explicit Session(ari::Widget root, ImVec2 display = ImVec2(1000.0f, 700.0f)) {
    ImGui::SetCurrentContext(ctx); // CreateContext keeps another session's current
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = display;
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
  std::setvbuf(stdout, nullptr, _IONBF, 0); // each check shows, even before an abort
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

  // §5.27 a row of choices: the chosen one drawn pressed in (button_with), a tap runs its on: as on
  // any button, and choosing another moves no button.
  {
    ari::Widget following = ari::button("Following", "(state-set \"mode\" \"following\")");
    following.selected = "(= (state-get \"mode\") \"following\")";
    ari::Widget everyone = ari::button("Everyone", "(state-set \"mode\" \"everyone\")");
    everyone.selected = "(= (state-get \"mode\") \"everyone\")";
    ari::Widget row = ari::group({following, everyone});
    row.layout.kind = ari::LayoutKind::Horizontal;
    ari::Widget bar = ari::window("Bar", {row});
    bar.has_pos = true;
    bar.pos_x = bar.pos_y = 20.0f;
    Session c(bar);
    c.set("mode", "following");
    c.frame(ImVec2(990.0f, 690.0f), false);
    chk(c.be.selected["Following"] && !c.be.selected["Everyone"],
        "the chosen button is drawn selected, the other not");
    const RecordingBackend::Rect before = c.be.at["Everyone"];
    const ImVec2 every = before.centre();
    c.tap(every, every);
    chk(c.val("mode") == "everyone", "a tap on the other runs its on: (" + c.val("mode") + ")");
    chk(c.be.selected["Everyone"] && !c.be.selected["Following"], "and it is drawn selected now");
    chk(c.be.at["Everyone"].min.x == before.min.x && c.be.at["Everyone"].max.x == before.max.x,
        "choosing it moved no button");
    chk(c.be.capabilities().button_selected && c.be.capabilities().button_icon,
        "ImGuiBackend reports button.selected and button.icon");
    chk(!c.be.view_is_phone(), "a 1000 x 700 view is not a phone's");
  }

  // §5.9 `maximized: phone`: a window starts maximized on a phone's view (under 500 px on its short
  // side), and as laid out on a wider one; the user's restore writes the key, which then holds.
  {
    ari::Widget feed = ari::window("Main", {ari::text("a feed")});
    feed.id = "main";
    feed.start_maximized = "phone";
    Session phone(feed, ImVec2(390.0f, 760.0f));
    chk(phone.be.view_is_phone(), "a 390 x 760 view is a phone's");
    chk(phone.be.maximized, "maximized: phone starts maximized on a phone");
    phone.set("tree.main.maximized", "0");
    phone.frame(ImVec2(380.0f, 750.0f), false);
    chk(!phone.be.maximized, "a restore (the key written 0) holds");
    Session wide(feed);
    chk(!wide.be.maximized, "maximized: phone opens as laid out on a wide view");
    feed.start_maximized = "true";
    Session always(feed);
    chk(always.be.maximized, "maximized: true starts maximized anywhere");
  }

  // §5.8 a narrow window (a phone's): a horizontal row's cells flow onto the next line, and a
  // line of text wider than the window wraps, so nothing runs past its right edge.
  {
    std::vector<ari::Widget> cells;
    for (const char *name : {"One", "Two", "Three", "Four", "Five", "Six"})
      cells.push_back(ari::button(name, "(state-set \"x\" \"1\")"));
    ari::Widget row = ari::group(std::move(cells));
    row.layout.kind = ari::LayoutKind::Horizontal;
    const std::string line = "a line of text a good deal wider than the narrow window it is in";
    ari::Widget narrow = ari::window("Narrow", {row, ari::text(line), ari::text("short")});
    narrow.size.w = {ari::Unit::Px, 200.0f};
    narrow.size.h = {ari::Unit::Px, 400.0f};
    narrow.has_pos = true;
    narrow.pos_x = narrow.pos_y = 20.0f;
    Session n(narrow);
    const float right = 20.0f + 200.0f; // the window's right edge
    chk(n.be.at["Six"].min.y > n.be.at["One"].min.y, "the row's last cell went to a later line");
    bool inside = true;
    for (const char *name : {"One", "Two", "Three", "Four", "Five", "Six"})
      inside = inside && n.be.at[name].max.x <= right;
    chk(inside, "every cell of the row is inside the window");
    const RecordingBackend::Rect t = n.be.at[line];
    chk(t.max.y - t.min.y > 1.5f * ImGui::GetTextLineHeight(), "the long line wrapped");
    chk(t.max.x <= right, "and stays inside the window");
    const RecordingBackend::Rect s2 = n.be.at["short"];
    chk(s2.max.y - s2.min.y < 1.5f * ImGui::GetTextLineHeight(), "a line that fits is one line");
  }
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

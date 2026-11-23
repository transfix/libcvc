// ImGuiBackend's right-click menus (§5.28) on a bare ImGui context (no GL), opened through ImGui's
// own input. A right-click on a group opens the group's menu, not its window's; on the window's
// empty part the window's; over no window the document's (the main view). A right-drag (a pan)
// opens nothing. A touch screen's long press opens it when the finger lifts, the Menu key or
// Shift+F10 at the pointer. A click on an item runs it with the menu's event, and the group's own
// on_click still works.
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
// The ImGui backend, noting where each text line and menu item was drawn, and which menu is open.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Rect {
    ImVec2 min, max;
    ImVec2 centre() const { return ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f); }
  };
  std::map<std::string, Rect> at;
  std::string open; // the menu drawn open this frame ("" none)
  void note(const std::string &name) {
    at[name] = Rect{ImGui::GetItemRectMin(), ImGui::GetItemRectMax()};
  }
  void text_line(const char *text) override {
    ImGuiBackend::text_line(text);
    note(text);
  }
  bool menu_item_action(const char *label) override {
    const bool hit = ImGuiBackend::menu_item_action(label);
    note(label);
    return hit;
  }
  bool begin_context_menu(const char *id, ari::ContextMenuOver over,
                          ari::ContextMenuOpen &opened) override {
    const bool shown = ImGuiBackend::begin_context_menu(id, over, opened);
    if (shown)
      open = id;
    return shown;
  }
  void begin_frame() override {
    ImGuiBackend::begin_frame();
    open.clear();
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    ImGui::SetCurrentContext(ctx);
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 700.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h);
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(990.0f, 690.0f));
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  // One frame with the pointer at `mouse`, `button` changed to `down` (button < 0: none).
  void frame(ImVec2 mouse, int button = -1, bool down = false) {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    if (button >= 0)
      io.AddMouseButtonEvent(button, down);
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    for (int i = 0; i < 20; ++i)
      rt.drain();
  }
  void key(ImGuiKey k, bool down) { ImGui::GetIO().AddKeyEvent(k, down); }
  // A click of `button` at `from`, released at `to`, then a frame to settle.
  void click(int button, ImVec2 from, ImVec2 to) {
    frame(from);
    frame(from, button, true);
    frame(to);
    frame(to, button, false);
    frame(to);
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
  void set(const std::string &key, const std::string &v) {
    cvc::state::instance(app)("ui." + key).value(v);
  }
};

// An item noting the menu's event under `tag`.
ari::Widget noting(const std::string &label, const std::string &tag) {
  return ari::menu_action(label, "(begin (state-set \"" + tag +
                                     ".on\" (get-attr event \"on\"))"
                                     " (state-set \"" +
                                     tag +
                                     ".via\" (get-attr event \"via\"))"
                                     " (state-set \"" +
                                     tag + ".px\" (int (get-attr event \"px\"))))");
}
#endif

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget card = ari::group({ari::text("card text"), ari::text("more card text")});
  card.id = "card";
  card.on_click = "(state-set \"opened\" \"1\")";
  card = ari::with_context_menu(card, {noting("Open card", "card"), ari::separator(),
                                       ari::menu_action("Hide card", "(state-set \"hid\" \"1\")")});
  ari::Widget feed = ari::window("Feed", {card, ari::text("below the card")});
  feed.id = "feed";
  feed.size.w = {ari::Unit::Px, 420.0f};
  feed.size.h = {ari::Unit::Px, 300.0f};
  feed.has_pos = true;
  feed.pos_x = feed.pos_y = 40.0f;
  feed = ari::with_context_menu(feed, {noting("Feed settings", "win")});
  ari::Widget root = ari::group({feed});
  root = ari::with_context_menu(root, {noting("Spawn here", "main")}, "scene");
  Session s(root);
  chk(s.be.capabilities().context_menu, "ImGuiBackend reports ui.context_menu");
  chk(s.be.at.count("card text") && s.be.at.count("below the card"), "the window is drawn");
  chk(s.be.open.empty(), "no menu is open at first");

  const ImVec2 text = s.be.at["card text"].centre();
  s.click(ImGuiMouseButton_Right, text, text);
  chk(s.be.open == "ctx:card",
      "a right-click on the card opens the card's menu (" + s.be.open + ")");
  chk(s.be.at.count("Open card") && s.be.at.count("Hide card"), "its items are drawn");
  chk(!s.be.at.count("Feed settings"), "not the window's");
  chk(s.val("opened").empty(), "a right-click is not the card's on_click");
  const ImVec2 item = s.be.at["Open card"].centre();
  s.click(ImGuiMouseButton_Left, item, item);
  chk(s.val("card.on") == "group", "the item ran with the menu's event (" + s.val("card.on") + ")");
  chk(s.val("card.via") == "pointer", "opened by the pointer (" + s.val("card.via") + ")");
  chk(!s.val("card.px").empty() && s.val("card.px") != "0",
      "where, from the card's left (" + s.val("card.px") + ")");
  chk(s.be.open.empty(), "the item closed it");

  s.click(ImGuiMouseButton_Left, text, text);
  chk(s.val("opened") == "1", "the card's on_click still runs on a tap (" + s.val("opened") + ")");

  // A right-drag (a pan in a view) opens nothing.
  const ImVec2 lower = s.be.at["more card text"].centre();
  s.click(ImGuiMouseButton_Right, text, ImVec2(lower.x + 60.0f, lower.y));
  chk(s.be.open.empty(), "a right-drag opens no menu (" + s.be.open + ")");

  // The window's empty part: the window's menu.
  const RecordingBackend::Rect below = s.be.at["below the card"];
  const ImVec2 empty(below.min.x + 20.0f, below.max.y + 60.0f);
  s.click(ImGuiMouseButton_Right, empty, empty);
  chk(s.be.open == "ctx:feed", "below its content, the window's menu (" + s.be.open + ")");
  s.frame(empty, ImGuiMouseButton_Left, true); // a press outside a popup closes it
  s.frame(empty, ImGuiMouseButton_Left, false);
  s.frame(empty);
  chk(s.be.open.empty(), "a press elsewhere closes it");

  // Over no window: the document's own, over the main view.
  const ImVec2 bare(900.0f, 600.0f);
  s.click(ImGuiMouseButton_Right, bare, bare);
  chk(s.be.open == "ctx:main", "over no window, the document's menu (" + s.be.open + ")");
  const ImVec2 spawn = s.be.at["Spawn here"].centre();
  s.click(ImGuiMouseButton_Left, spawn, spawn);
  chk(s.val("main.on") == "main", "its item ran over the main view (" + s.val("main.on") + ")");

  // The Menu key at the pointer.
  s.frame(text);
  s.key(ImGuiKey_Menu, true);
  s.frame(text);
  s.key(ImGuiKey_Menu, false);
  s.frame(text);
  chk(s.be.open == "ctx:card", "the Menu key opens the menu under the pointer (" + s.be.open + ")");
  s.frame(bare, ImGuiMouseButton_Left, true);
  s.frame(bare, ImGuiMouseButton_Left, false);
  s.frame(bare);

  // A touch screen's long press: open when the finger lifts, not before.
  s.set("opened", "");
  ImGui::GetIO().AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  s.frame(text);
  s.frame(text, ImGuiMouseButton_Left, true);
  for (int f = 0; f < 40; ++f) // two thirds of a second
    s.frame(text);
  chk(s.be.open.empty(), "a long press opens nothing while the finger is down");
  s.frame(text, ImGuiMouseButton_Left, false);
  s.frame(text);
  chk(s.be.open == "ctx:card", "it opens when the finger lifts (" + s.be.open + ")");
  const ImVec2 hide = s.be.at["Hide card"].centre();
  s.frame(hide);
  s.frame(hide, ImGuiMouseButton_Left, true);
  s.frame(hide, ImGuiMouseButton_Left, false);
  s.frame(hide);
  chk(s.val("hid") == "1", "a tap on its item runs it (" + s.val("hid") + ")");
  ImGui::GetIO().AddMouseSourceEvent(ImGuiMouseSource_Mouse);
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

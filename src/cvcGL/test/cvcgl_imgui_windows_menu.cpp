// ImGuiBackend + the built-in Windows menu (docs/ARIADNE.md §5.9) on a bare ImGui context (no GL):
// a window closed with its X is not drawn, and clicking its item in the Windows menu (mouse events
// through ImGui, as a user would) opens it again, in its place among the windows with no pos:. A
// document with a closable window and no menubar gets a menubar for it.
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
#include <imgui_internal.h> // ImGuiWindow: is a window drawn this frame
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
// The ImGui backend, noting where it drew each menu title (while its menu is closed: an open menu's
// last item is its popup) and each menu item, so the test can click them.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  std::map<std::string, ImVec2> at;
  void mark(const char *label) {
    const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
    at[label] = ImVec2((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
  }
  bool begin_menu(const char *label) override {
    const bool open = ImGuiBackend::begin_menu(label);
    if (!open)
      mark(label);
    return open;
  }
  ari::BoolEdit menu_item_toggle(const char *label, bool current) override {
    const ari::BoolEdit e = ImGuiBackend::menu_item_toggle(label, current);
    mark(label);
    return e;
  }
};

ImGuiWindow *window_of(const std::string &id) {
  for (ImGuiWindow *w : GImGui->Windows) {
    const std::string n = w->Name;
    const std::size_t at = n.find("###");
    if (at != std::string::npos && n.substr(at + 3) == id)
      return w;
  }
  return nullptr;
}

bool drawn(const std::string &id) {
  const ImGuiWindow *w = window_of(id);
  return w && w->WasActive && w->Active;
}

bool overlap(const ImGuiWindow *a, const ImGuiWindow *b) {
  return a->Pos.x < b->Pos.x + b->Size.x && b->Pos.x < a->Pos.x + a->Size.x &&
         a->Pos.y < b->Pos.y + b->Size.y && b->Pos.y < a->Pos.y + a->Size.y;
}

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
  }
  // Move over `label` (where it was last drawn), press, release, and let the UI settle a frame.
  bool click(const std::string &label) {
    const auto it = be.at.find(label);
    if (it == be.at.end())
      return false;
    const ImVec2 p = it->second;
    frame(p, false);
    frame(p, true);
    frame(p, false);
    frame(p, false);
    return true;
  }
  std::string open_state(const std::string &id) {
    return cvc::state::instance(app)("ui.tree." + id + ".open").value();
  }
};

ari::Widget panel(const std::string &title, const std::string &id) {
  ari::Widget w = ari::window(title, {ari::text("body of " + title)});
  w.id = id;
  w.closable = true;
  return w;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  {
    Session s(ari::group({ari::menubar({ari::menu("File", {ari::menu_action("Quit", "quit")})}),
                          panel("Alpha", "alpha"), panel("Beta", "beta")}));
    chk(drawn("alpha") && drawn("beta"), "both windows open");
    chk(s.be.at.count("File") && s.be.at.count("Windows"),
        "the menubar has the document's File menu and the Windows menu");
    if (s.be.at.count("File") && s.be.at.count("Windows"))
      chk(s.be.at["File"].x < s.be.at["Windows"].x, "Windows comes after the document's menus");

    // Close Alpha, as its X does.
    cvc::state::instance(s.app)("ui.tree.alpha.open").value(std::string("0"));
    s.frame(ImVec2(990.0f, 690.0f), false);
    s.frame(ImVec2(990.0f, 690.0f), false);
    chk(!drawn("alpha") && drawn("beta"), "a closed window is not drawn");

    chk(s.click("Windows"), "click the Windows menu");
    chk(s.be.at.count("Alpha") && s.be.at.count("Beta"), "it lists both windows");
    chk(s.click("Alpha"), "click Alpha's item");
    chk(s.open_state("alpha") == "1",
        "Alpha's open state is back to 1 (" + s.open_state("alpha") + ")");
    s.frame(ImVec2(990.0f, 690.0f), false);
    s.frame(ImVec2(990.0f, 690.0f), false);
    chk(drawn("alpha"), "and Alpha is drawn again");
    // Neither has a pos:, so both stack down the left (ARIADNE.md §5.8): Alpha comes back to its
    // place in that column, and Beta, which moved up while it was closed, moves down again.
    const ImGuiWindow *a = window_of("alpha"), *b = window_of("beta");
    chk(a && b && !overlap(a, b), "Alpha takes its place in the column again, not over Beta");

    // The menu closes a window as well.
    chk(s.click("Windows") && s.click("Beta"), "untick Beta in the Windows menu");
    s.frame(ImVec2(990.0f, 690.0f), false);
    chk(s.open_state("beta") == "0" && !drawn("beta"), "Beta closes");
  }
  {
    // No menubar in the document, but a closable window: the Windows menu brings its own bar.
    Session s(ari::group({panel("Solo", "solo")}));
    chk(s.be.at.count("Windows") == 1, "a menubar with just the Windows menu");
    const ImGuiWindow *bar = ImGui::FindWindowByName("##MainMenuBar");
    chk(bar && bar->Active, "drawn as ImGui's main menu bar");
    const ImGuiWindow *solo = window_of("solo");
    chk(bar && solo && solo->Pos.y >= bar->Pos.y + bar->Size.y,
        "and the window stands below it, not under it");
  }

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

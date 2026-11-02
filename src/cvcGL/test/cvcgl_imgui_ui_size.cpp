// ImGuiBackend at a phone's UI size, on a bare ImGui context (no window, no GL). ImGuiOverlay
// scales the style by the UI size (style.FontScaleMain) and the display's pixel ratio
// (FontScaleDpi); this sets them as it does:
//  - a document's px scale with them: a window's size: and pos:, a column's width, the area a
//    group's background: takes;
//  - a window opens inside the view: an authored size larger than the view stops at its edge, and
//    an authored pos past the edge moves in until all of the window shows (a window that fits its
//    content: its smallest fit at first, then all of it once measured);
//  - the geometry persisted at tree.<id>.geometry is in the document's px, not the device's;
//  - at UI size 1 and pixel ratio 1 nothing changes: the same document on a laptop opens where it
//    says, at the size it says.
// And the overlay's UI size reaches it: UiSettings hands its owner only edits made by others, so
// a touch device's size isn't lost to the settings' own defaults before the first frame.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow, ImGuiTable: sizes and places
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
ImGuiWindow *find(const char *id) {
  return ImGui::FindWindowByName((std::string("###") + id).c_str());
}

bool near(float a, float b) { return std::fabs(a - b) <= 1.0f; }

std::string vec(ImVec2 v) {
  return std::to_string(static_cast<int>(v.x)) + "," + std::to_string(static_cast<int>(v.y));
}

// The conference's layout, cut down: My Camera has a pos: and a size:, Users a pos: past a
// phone's width and no size, and a row of px columns as AppRuntime's Controls window lays out.
// Card holds a group with a background: 120 px tall (its picture a name no host published, so
// only the area is drawn).
ari::Widget conference() {
  ari::Widget cam = ari::window("My Camera", {ari::text("self view")});
  cam.id = "cam";
  cam.has_pos = true;
  cam.pos_x = 20.0f;
  cam.pos_y = 40.0f;
  cam.size.w = ari::Extent{ari::Unit::Px, 360.0f};
  cam.size.h = ari::Extent{ari::Unit::Px, 560.0f};
  ari::Widget row = ari::group({ari::text("Forward"), ari::text("W"), ari::text("Change")});
  row.id = "row";
  row.layout.kind = ari::LayoutKind::Grid;
  row.layout.col_widths = {ari::Track{ari::Unit::Px, 100.0f}, ari::Track{ari::Unit::Px, 80.0f},
                           ari::Track{}};
  ari::Widget users = ari::window("Users", {row});
  users.id = "users";
  users.has_pos = true;
  users.pos_x = 390.0f;
  users.pos_y = 40.0f;
  ari::Widget banner = ari::group({ari::text("Grace Hopper")});
  banner.size.h = ari::Extent{ari::Unit::Px, 120.0f};
  banner.background.src = "no-such-picture";
  ari::Widget card = ari::window("Card", {banner});
  card.id = "card";
  card.has_pos = true;
  card.pos_x = 20.0f;
  card.pos_y = 620.0f;
  card.size.w = ari::Extent{ari::Unit::Px, 300.0f};
  card.size.h = ari::Extent{ari::Unit::Px, 200.0f};
  return ari::group({cam, users, card});
}

// Each group background's area, as the base lays it out.
struct Backend : cvc::gl::ImGuiBackend {
  std::vector<ImVec2> backdrop_size;
  void end_backdrop() override {
    ImGuiBackend::end_backdrop();
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    backdrop_size.push_back(ImVec2(mx.x - mn.x, mx.y - mn.y));
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  Backend be;
  Session(ImVec2 display, float uiSize, float pixelRatio) {
    // libcvcGL may carry its own copy of imgui (a static imgui, as on Windows): hand it this one.
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = display;
    io.IniFilename = nullptr;
    ImGuiStyle &style = ImGui::GetStyle(); // as ImGuiOverlay::applyScale leaves it
    style.ScaleAllSizes(uiSize * pixelRatio);
    style.FontScaleMain = uiSize;
    style.FontScaleDpi = pixelRatio;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(conference());
    for (int f = 0; f < 4; ++f)
      frame();
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  void frame() {
    be.backdrop_size.clear();
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
  }
  std::string geometry(const std::string &id) {
    return cvc::state::instance(app)("ui.tree." + id + ".geometry").value();
  }
};

// The row's table: the one with three columns.
const ImGuiTable *row_table() {
  ImGuiContext &g = *GImGui;
  for (int i = 0; i < g.Tables.GetMapSize(); ++i)
    if (const ImGuiTable *t = g.Tables.TryGetMapData(i))
      if (t->ColumnsCount == 3)
        return t;
  return nullptr;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  {
    // A phone: 390 x 844 CSS px at a pixel ratio of 3, with a touch device's UI size, 1.25.
    const float k = 1.25f * 3.0f;
    const ImVec2 view(1170.0f, 2532.0f);
    Session s(view, 1.25f, 3.0f);
    const ImGuiWindow *cam = find("cam"), *users = find("users");
    chk(cam && users, "both windows are drawn");
    if (cam && users) {
      chk(near(cam->Size.x, view.x) && near(cam->Size.y, 560.0f * k),
          "My Camera's 360 x 560 px are 1350 x 2100 device px, stopped at the view's 1170 (" +
              vec(cam->Size) + ")");
      chk(cam->Pos.x >= 0.0f && cam->Pos.x + cam->Size.x <= view.x + 0.5f &&
              near(cam->Pos.y, 40.0f * k),
          "and it opens all in view, at its pos:'s height (" + vec(cam->Pos) + ")");
      chk(users->Pos.x >= 0.0f && users->Pos.x + users->Size.x <= view.x + 0.5f &&
              users->Pos.x < 390.0f * k && near(users->Pos.y, 40.0f * k),
          "Users' pos: is past the phone's edge: it moves in until all of it shows (" +
              vec(users->Pos) + " + " + vec(users->Size) + ")");
    }
    const ImGuiTable *t = row_table();
    chk(t && near(t->Columns[0].WidthGiven, 100.0f * k) &&
            near(t->Columns[1].WidthGiven, 80.0f * k),
        "the row's 100 and 80 px columns at the UI's scale (" +
            (t ? std::to_string(t->Columns[0].WidthGiven) + ", " +
                     std::to_string(t->Columns[1].WidthGiven)
               : std::string("no table")) +
            ")");
    chk(s.be.backdrop_size.size() == 1 && s.be.backdrop_size[0].y >= 120.0f * k - 0.5f,
        "the group's background: takes its 120 px at the UI's scale (" +
            (s.be.backdrop_size.empty() ? std::string("none") : vec(s.be.backdrop_size[0])) + ")");
    chk(s.geometry("cam") == "0,40,312,560",
        "the persisted geometry is in the document's px (" + s.geometry("cam") + ")");
  }
  {
    // A laptop: UI size 1, pixel ratio 1 -- the document's own numbers.
    Session s(ImVec2(1600.0f, 900.0f), 1.0f, 1.0f);
    const ImGuiWindow *cam = find("cam"), *users = find("users");
    chk(cam && near(cam->Pos.x, 20.0f) && near(cam->Pos.y, 40.0f) && near(cam->Size.x, 360.0f) &&
            near(cam->Size.y, 560.0f),
        "on a laptop My Camera opens at its pos: and size:");
    chk(users && near(users->Pos.x, 390.0f) && near(users->Pos.y, 40.0f), "and Users at its pos:");
    const ImGuiTable *t = row_table();
    chk(t && near(t->Columns[0].WidthGiven, 100.0f) && near(t->Columns[1].WidthGiven, 80.0f),
        "the columns at their px");
    chk(s.be.backdrop_size.size() == 1 && near(s.be.backdrop_size[0].y, 120.0f),
        "the background's area at its 120 px (" +
            (s.be.backdrop_size.empty() ? std::string("none") : vec(s.be.backdrop_size[0])) + ")");
    chk(s.geometry("cam") == "20,40,360,560", "persisted as authored (" + s.geometry("cam") + ")");
  }
  {
    // ImGuiOverlay picks 1.25 on a touch device, then makes its UiSettings, which seed their
    // defaults (1, no touch). Handed back to the overlay, those would run a phone at 1.
    using cvc::gl::UiSettings;
    cvc::app app;
    int applied = 0;
    UiSettings::Values last;
    UiSettings ui(app, "ui.size_test", [&](UiSettings::Values v) {
      ++applied;
      last = v;
    });
    chk(applied == 0, "UiSettings' seed of its defaults isn't handed to the owner");
    UiSettings::Values touch;
    touch.scale = 1.25;
    touch.touchMode = true;
    ui.set(touch);
    chk(applied == 0, "nor is the owner's own set()");
    cvc::state::instance(app)("ui.size_test.scale").value(1.5);
    chk(applied == 1 && last.scale == 1.5 && last.touchMode,
        "an edit from outside (the Controls window's UI size) is, with the owner's other keys");
  }

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

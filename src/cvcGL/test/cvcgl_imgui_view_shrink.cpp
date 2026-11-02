// ImGuiBackend when the view gets smaller after its windows opened (a browser window made smaller,
// a phone turned on its side, fullscreen left), on a bare ImGui context (no window, no GL):
//  - a window that showed in full moves back in until all of it shows, no larger than the view;
//  - a window that fits its content moves in too;
//  - one the user left partly outside the view stays where it is;
//  - the view growing again moves nothing.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <functional>
#include <string>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow: where each window is
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

std::string rect(const ImGuiWindow *w) {
  return std::to_string(static_cast<int>(w->Pos.x)) + "," +
         std::to_string(static_cast<int>(w->Pos.y)) + " " +
         std::to_string(static_cast<int>(w->Size.x)) + "x" +
         std::to_string(static_cast<int>(w->Size.y));
}

bool inside(const ImGuiWindow *w, ImVec2 view) {
  return w->Pos.x >= -0.5f && w->Pos.y >= -0.5f && w->Pos.x + w->Size.x <= view.x + 0.5f &&
         w->Pos.y + w->Size.y <= view.y + 0.5f;
}

// The conference's layout, cut down: My Camera with a pos: and a size:, Users with a pos: and
// no size (it fits its content), Card with a pos: low on a laptop's view.
ari::Widget conference() {
  ari::Widget cam = ari::window("My Camera", {ari::text("self view")});
  cam.id = "cam";
  cam.has_pos = true;
  cam.pos_x = 20.0f;
  cam.pos_y = 40.0f;
  cam.size.w = ari::Extent{ari::Unit::Px, 360.0f};
  cam.size.h = ari::Extent{ari::Unit::Px, 560.0f};
  ari::Widget users = ari::window("Users", {ari::text("in the room: 3"), ari::text("alice")});
  users.id = "users";
  users.has_pos = true;
  users.pos_x = 1300.0f;
  users.pos_y = 40.0f;
  ari::Widget card = ari::window("Card", {ari::text("Grace Hopper")});
  card.id = "card";
  card.has_pos = true;
  card.pos_x = 20.0f;
  card.pos_y = 620.0f;
  card.size.w = ari::Extent{ari::Unit::Px, 300.0f};
  card.size.h = ari::Extent{ari::Unit::Px, 200.0f};
  return ari::group({cam, users, card});
}

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  cvc::gl::ImGuiBackend be;
  explicit Session(ImVec2 display) {
    // libcvcGL may carry its own copy of imgui (a static imgui, as on Windows): hand it this one.
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = display;
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(conference());
    frames(4);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  void frame(const std::function<void()> &before = {}) {
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    if (before)
      before();
    rt.render();
    ImGui::Render();
  }
  void frames(int n) {
    for (int f = 0; f < n; ++f)
      frame();
  }
  // The view becomes `display`, as a canvas resized between frames.
  void resize(ImVec2 display) {
    ImGui::GetIO().DisplaySize = display;
    frames(3);
  }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("cvcgl_imgui_view_shrink: SKIP (libcvc built without ImGui)\n");
  return 0;
#else
  std::printf("== a laptop's view, then a shorter one, then a phone's width ==\n");
  {
    Session s(ImVec2(1600.0f, 900.0f));
    ImGuiWindow *cam = find("cam"), *users = find("users"), *card = find("card");
    chk(cam && users && card, "the three windows are drawn");
    if (!cam || !users || !card)
      return 1;
    chk(near(cam->Pos.y, 40.0f) && near(card->Pos.y, 620.0f) && near(users->Pos.x, 1300.0f),
        "at 1600 x 900 every window is where its pos: says (cam " + rect(cam) + ", card " +
            rect(card) + ", users " + rect(users) + ")");

    const ImVec2 laptop(1350.0f, 579.0f); // a 1366 x 768 screen's browser canvas
    s.resize(laptop);
    chk(inside(cam, laptop) && inside(card, laptop) && inside(users, laptop),
        "at 1350 x 579 every window shows in full (cam " + rect(cam) + ", card " + rect(card) +
            ", users " + rect(users) + ")");
    chk(near(cam->Size.y, 560.0f) && near(cam->Pos.x, 20.0f),
        "My Camera keeps its size and its x, and only moves up (" + rect(cam) + ")");
    chk(near(card->Pos.y + card->Size.y, 579.0f) && near(card->Pos.x, 20.0f),
        "Card moves up to the bottom edge, no further (" + rect(card) + ")");

    const ImVec2 short_view(1350.0f, 450.0f);
    s.resize(short_view);
    chk(inside(cam, short_view) && near(cam->Size.y, 450.0f),
        "at 1350 x 450, My Camera (560 tall) shrinks to the view (" + rect(cam) + ")");

    const ImVec2 phone(500.0f, 450.0f);
    s.resize(phone);
    chk(inside(users, phone),
        "at 500 x 450 Users, which fits its content, moves in from the right (" + rect(users) +
            ")");

    const ImVec2 cam_was = cam->Pos, card_was = card->Pos, users_was = users->Pos;
    s.resize(ImVec2(1600.0f, 900.0f));
    chk(cam->Pos.x == cam_was.x && cam->Pos.y == cam_was.y && card->Pos.x == card_was.x &&
            card->Pos.y == card_was.y && users->Pos.x == users_was.x && users->Pos.y == users_was.y,
        "back at 1600 x 900 nothing moves");
  }

  std::printf("== a window the user left partly outside the view ==\n");
  {
    Session s(ImVec2(1600.0f, 900.0f));
    s.frame([] { ImGui::SetWindowPos("Card###card", ImVec2(20.0f, 800.0f)); });
    s.frames(2);
    ImGuiWindow *card = find("card"), *users = find("users");
    if (!card || !users)
      return 1;
    chk(near(card->Pos.x, 20.0f) && near(card->Pos.y, 800.0f),
        "Card is where the user put it, past the bottom edge (" + rect(card) + ")");
    // Narrower and a little shorter: Card still shows its top 30 px (ImGui itself keeps some of a
    // window on screen), and Users, which showed in full, now runs past the right edge.
    const ImVec2 view(1350.0f, 830.0f);
    s.resize(view);
    chk(near(card->Pos.x, 20.0f) && near(card->Pos.y, 800.0f),
        "at 1350 x 830 Card stays where the user put it (" + rect(card) + ")");
    chk(inside(users, view), "Users, which showed in full, moves in (" + rect(users) + ")");
  }

  std::printf("cvcgl_imgui_view_shrink: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

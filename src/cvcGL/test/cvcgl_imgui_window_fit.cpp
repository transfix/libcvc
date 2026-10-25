// ImGuiBackend's window sizing and placement on a bare ImGui context (no window, no GL):
//  - a window with no size: fits its widgets, though §11.4 persists its geometry every frame (the
//    size persisted after its first, unmeasured frame used to stop the fit at ImGui's 32 px);
//  - windows with no pos: stack down the left of the viewport instead of on top of one another,
//    until the user moves one;
//  - a window with a size keeps it and scrolls both ways when its widgets need more.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <functional>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow: size, position, scrollbars
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
ari::Widget panel(const std::string &title, const std::string &id, int sliders,
                  const std::string &label = "Setting number ") {
  std::vector<ari::Widget> kids;
  for (int i = 0; i < sliders; ++i)
    kids.push_back(
        ari::slider_float(label + std::to_string(i), id + ".s" + std::to_string(i), 0.0, 1.0, 0.5));
  ari::Widget w = ari::window(title, std::move(kids));
  w.id = id;
  return w;
}

ImGuiWindow *find(const char *id) {
  return ImGui::FindWindowByName((std::string("###") + id).c_str());
}

bool overlap(const ImGuiWindow *a, const ImGuiWindow *b) {
  return a->Pos.x < b->Pos.x + b->Size.x && b->Pos.x < a->Pos.x + a->Size.x &&
         a->Pos.y < b->Pos.y + b->Size.y && b->Pos.y < a->Pos.y + a->Size.y;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ImGuiContext *ctx = ImGui::CreateContext();
  // libcvcGL may carry its own copy of imgui (a static imgui, as on Windows): hand it this context.
  cvc::gl::ImGuiBackend::setImGuiContext(ctx);
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800.0f, 600.0f);
  io.IniFilename = nullptr;
  unsigned char *px = nullptr;
  int w = 0, h = 0;
  io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
  {
    cvc::app app;
    ari::Runtime rt(app, "ui");
    cvc::gl::ImGuiBackend be;
    rt.set_backend(&be);
    ari::Widget sized = panel("Sized", "sized", 12, "A label too long for this window, number ");
    sized.size.w = ari::Extent{ari::Unit::Px, 200.0f};
    sized.size.h = ari::Extent{ari::Unit::Px, 90.0f};
    sized.has_pos = true;
    sized.pos_x = 500.0f;
    sized.pos_y = 20.0f;
    rt.set_root(ari::group({panel("First", "fit_a", 3), panel("Second", "fit_b", 6),
                            panel("Third", "fit_c", 2), sized}));
    const auto frames = [&](int n, const std::function<void()> &before = {}) {
      for (int f = 0; f < n; ++f) {
        io.DeltaTime = 1.0f / 60.0f;
        ImGui::NewFrame();
        if (before)
          before();
        rt.render();
        ImGui::Render();
      }
    };
    frames(4);

    ImGuiWindow *a = find("fit_a"), *b = find("fit_b"), *c = find("fit_c"), *s = find("sized");
    chk(a && b && c && s, "every window was drawn");
    if (!(a && b && c && s))
      return 1;
    for (ImGuiWindow *x : {a, b, c})
      chk(x->Size.x > 200.0f && x->Size.y > 60.0f && !x->ScrollbarX && !x->ScrollbarY,
          std::string(x->Name) + " fits its sliders: " + std::to_string(int(x->Size.x)) + " x " +
              std::to_string(int(x->Size.y)) + " px, no scrollbars");
    chk(b->Size.y > a->Size.y, "the window with more sliders is the taller one");
    chk(!overlap(a, b) && !overlap(b, c) && !overlap(a, c),
        "the windows with no pos stand apart, not on top of one another");
    chk(a->Pos.x == b->Pos.x && a->Pos.y < b->Pos.y && b->Pos.y < c->Pos.y,
        "they stack down one column, in document order");

    chk(s->Size.x == 200.0f && s->Size.y == 90.0f, "the sized window keeps its size");
    chk(s->Pos.x == 500.0f && s->Pos.y == 20.0f, "and its authored pos");
    chk(s->ScrollbarY, "and scrolls down to the sliders it cannot show");
    chk(s->ScrollbarX, "and across to the labels it cannot show");

    // The user drags the second window away: it stays where they put it, and the third moves up
    // into its place.
    const ImVec2 dropped(300.0f, 300.0f);
    frames(1, [&] { ImGui::SetWindowPos("###fit_b", dropped); });
    frames(3);
    chk(b->Pos.x == dropped.x && b->Pos.y == dropped.y, "a window the user moved stays put");
    chk(c->Pos.y < dropped.y && !overlap(a, c), "the flow closes up behind it");
  }
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
  ImGui::DestroyContext(ctx);
  std::printf("cvcgl_imgui_window_fit: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

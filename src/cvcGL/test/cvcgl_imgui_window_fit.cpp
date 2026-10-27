// ImGuiBackend's window sizing and placement on a bare ImGui context (no window, no GL):
//  - a window with no size: fits its widgets, though §11.4 persists its geometry every frame (the
//    size persisted after its first, unmeasured frame used to stop the fit at ImGui's 32 px);
//  - windows with no pos: stack down the left of the viewport instead of on top of one another,
//    until the user moves one; below a window that stands where it is; never past the right edge;
//  - a window with a size keeps it and scrolls both ways when its widgets need more;
//  - a split-pane window with no size is as wide as its panes' widgets;
//  - after Runtime::forget_layout (a new document), windows ImGui already has are laid out again;
//  - an authored pos is below the main menu bar, from the first frame.
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

ari::Widget placed(ari::Widget w, float x, float y, float width, float height) {
  w.has_pos = true;
  w.pos_x = x;
  w.pos_y = y;
  w.size.w = ari::Extent{ari::Unit::Px, width};
  w.size.h = ari::Extent{ari::Unit::Px, height};
  return w;
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
    const ImVec2 b_was = b->Pos, c_was = c->Pos;
    const ImVec2 dropped(300.0f, 300.0f);
    frames(1, [&] { ImGui::SetWindowPos("###fit_b", dropped); });
    frames(3);
    chk(b->Pos.x == dropped.x && b->Pos.y == dropped.y, "a window the user moved stays put");
    chk(c->Pos.x == b_was.x && c->Pos.y == b_was.y && c_was.y > b_was.y,
        "the flow closes up behind it: the third window is where the second was");

    // A window with a place of its own is in the flow's way: the flow goes below it.
    rt.set_root(ari::group({placed(panel("Fixed", "fixed", 1), 10.0f, 10.0f, 260.0f, 120.0f),
                            panel("Flowing", "flowing", 2)}));
    frames(4);
    ImGuiWindow *fixed = find("fixed"), *flowing = find("flowing");
    chk(fixed && flowing, "the fixed and the flowing window were drawn");
    if (!(fixed && flowing))
      return 1;
    chk(fixed->Pos.x == 10.0f && fixed->Pos.y == 10.0f, "the fixed window stands where it is");
    chk(!overlap(fixed, flowing) && flowing->Pos.y >= fixed->Pos.y + fixed->Size.y,
        "the flowing window goes below it, not over it");

    // More windows than columns fit in: the last column overlaps rather than leave the screen.
    io.DisplaySize = ImVec2(400.0f, 300.0f);
    rt.set_root(ari::group({panel("Tall one", "tall_1", 8), panel("Tall two", "tall_2", 8),
                            panel("Tall three", "tall_3", 8), panel("Tall four", "tall_4", 8)}));
    frames(5);
    for (const char *id : {"tall_1", "tall_2", "tall_3", "tall_4"}) {
      const ImGuiWindow *t = find(id);
      chk(t && t->Pos.x >= 0.0f && t->Pos.x + t->Size.x <= io.DisplaySize.x + 0.5f,
          std::string(id) + " is on screen: x " + (t ? std::to_string(int(t->Pos.x)) : "?") +
              " .. " + (t ? std::to_string(int(t->Pos.x + t->Size.x)) : "?"));
    }
    io.DisplaySize = ImVec2(800.0f, 600.0f);

    // A split-pane window with no size: its panes fit their widgets, and so it fits them.
    ari::Widget split = panel("Split", "split", 2);
    split.layout.row_heights = {ari::Track{ari::Unit::Px, 60.0f}, ari::Track{ari::Unit::Px, 60.0f}};
    split.layout.resizable = true;
    rt.set_root(ari::group({split}));
    frames(4);
    const ImGuiWindow *sp = find("split");
    chk(sp && sp->Size.x > 200.0f, "a split-pane window with no size fits its panes' widgets: " +
                                       (sp ? std::to_string(int(sp->Size.x)) : std::string("?")) +
                                       " px wide");

    // A new document whose windows ImGui already has, from the last one: its authored pos and
    // size apply, and a window the user moved out of the flow is back in it.
    frames(1, [&] { ImGui::SetWindowPos("###fit_a", ImVec2(500.0f, 400.0f)); });
    rt.forget_layout();
    rt.set_root(ari::group({panel("First", "fit_a", 3),
                            placed(panel("Sized", "sized", 2), 200.0f, 150.0f, 220.0f, 120.0f)}));
    frames(4);
    chk(a->Pos.x == 10.0f && a->Pos.y == 10.0f, "the moved window is back at the top of the flow");
    chk(s->Pos.x == 200.0f && s->Pos.y == 150.0f, "the other takes its new authored pos");
    chk(s->Size.x == 220.0f && s->Size.y == 120.0f, "and its new authored size");

    // An authored pos is in the work area: below the main menu bar, from the very first frame.
    rt.forget_layout();
    rt.set_root(ari::group({ari::menubar({ari::menu("File", {ari::menu_action("Quit", "quit")})}),
                            placed(panel("Below", "below_bar", 1), 24.0f, 6.0f, 200.0f, 80.0f)}));
    frames(2); // placed in the first; the second's WorkPos has the bar (it lags a frame)
    const ImGuiWindow *below = find("below_bar");
    const float bar = ImGui::GetMainViewport()->WorkPos.y;
    chk(below && bar > 0.0f && below->Pos.y == bar + 6.0f && below->Pos.x == 24.0f,
        "the authored pos is under the menu bar (" + std::to_string(int(bar)) + " px): y " +
            (below ? std::to_string(int(below->Pos.y)) : std::string("?")));
  }
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
  ImGui::DestroyContext(ctx);
  std::printf("cvcgl_imgui_window_fit: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

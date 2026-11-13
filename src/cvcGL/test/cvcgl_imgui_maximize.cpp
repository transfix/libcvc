// ImGuiBackend's maximize button (§5.9) on a bare ImGui context (no GL): a tap on the square in a
// window's title bar, through ImGui's own mouse input, fills the window to the view, and another
// puts it back where it was at its size. tree.<id>.maximized follows and drives it, and the
// window's persisted geometry stays where it goes back to. A `maximizable: false` window has none.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <string>
#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // ImVec2 + - *
#include <imgui.h>
#include <imgui_internal.h> // FindWindowByName, TitleBarRect
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
struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  cvc::gl::ImGuiBackend be;
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
  void frames(int n) {
    for (int i = 0; i < n; ++i)
      frame(ImVec2(990.0f, 690.0f), false);
  }
  void tap(ImVec2 at) {
    frame(at, false);
    frame(at, true);
    frame(at, false);
    frame(at, false);
  }
  void double_click(ImVec2 at) {
    frame(at, false);
    frame(at, true);
    frame(at, false);
    frame(at, true);
    frame(at, false);
    frame(at, false);
  }
  // A press at `from`, the pointer moved to `to` in steps with the button held, and a release.
  void drag(ImVec2 from, ImVec2 to) {
    frame(from, false);
    frame(from, true);
    for (int i = 1; i <= 8; ++i)
      frame(ImVec2(from.x + (to.x - from.x) * i / 8.0f, from.y + (to.y - from.y) * i / 8.0f), true);
    frame(to, false);
    frame(to, false);
  }
  // The pointer resting at `at` for `seconds`, nothing pressed.
  void rest(ImVec2 at, float seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 60.0f) + 1; ++i)
      frame(at, false);
  }
  // Where the close button is: ImGui's, at the right of the title bar.
  ImVec2 close_button(const char *name) {
    ImGuiWindow *w = window(name);
    if (!w)
      return ImVec2(-1.0f, -1.0f);
    const ImGuiStyle &st = ImGui::GetStyle();
    const float sz = ImGui::GetFontSize();
    const ImRect bar = w->TitleBarRect();
    return ImVec2(bar.Max.x - st.FramePadding.x - sz * 0.5f,
                  bar.Min.y + st.FramePadding.y + sz * 0.5f);
  }
  // How many vertices the window drew inside the maximize button: one square, or more for two.
  int glyph_vertices(const char *name) {
    ImGuiWindow *w = window(name);
    if (!w)
      return -1;
    const ImVec2 c = button(name, true);
    const float h = ImGui::GetFontSize() * 0.5f;
    const ImRect bb(c.x - h, c.y - h, c.x + h, c.y + h);
    int n = 0;
    for (const ImDrawVert &v : w->DrawList->VtxBuffer)
      if (bb.Contains(v.pos))
        ++n;
    return n;
  }
  // The tooltip showing now, if any: its window (ImGui names them ##Tooltip_<n>).
  ImGuiWindow *tooltip() {
    for (ImGuiWindow *w : GImGui->Windows)
      if ((w->Flags & ImGuiWindowFlags_Tooltip) && w->Active)
        return w;
    return nullptr;
  }
  // Whether the tooltip showing is as wide as one holding just `text`.
  bool tooltip_says(const char *text) {
    const ImGuiWindow *t = tooltip();
    const float want = ImGui::CalcTextSize(text).x + 2.0f * ImGui::GetStyle().WindowPadding.x;
    return t && std::fabs(t->SizeFull.x - want) < 1.5f;
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
  ImGuiWindow *window(const char *name) { return ImGui::FindWindowByName(name); }
  // Where the maximize button is: left of the close button, as ImGui places that.
  ImVec2 button(const char *name, bool closable) {
    ImGuiWindow *w = window(name);
    if (!w)
      return ImVec2(-1.0f, -1.0f);
    const ImGuiStyle &st = ImGui::GetStyle();
    const float sz = ImGui::GetFontSize();
    const ImRect bar = w->TitleBarRect();
    float right = bar.Max.x - st.FramePadding.x;
    if (closable)
      right -= sz + st.ItemInnerSpacing.x;
    return ImVec2(right - sz * 0.5f, bar.Min.y + st.FramePadding.y + sz * 0.5f);
  }
};

bool near(ImVec2 a, ImVec2 b) { return std::fabs(a.x - b.x) < 1.5f && std::fabs(a.y - b.y) < 1.5f; }
std::string str(ImVec2 v) { return std::to_string(v.x) + "," + std::to_string(v.y); }
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget post = ari::window("Post", {ari::text("the post's text")});
  post.id = "post";
  post.has_pos = true;
  post.pos_x = 100.0f;
  post.pos_y = 80.0f;
  post.size.w = ari::Extent{ari::Unit::Px, 300.0f};
  post.size.h = ari::Extent{ari::Unit::Px, 200.0f};
  post.closable = true;
  ari::Widget pinned = ari::window("Pinned", {ari::text("fixed")});
  pinned.id = "pinned";
  pinned.has_pos = true;
  pinned.pos_x = 500.0f;
  pinned.pos_y = 400.0f;
  pinned.size.w = ari::Extent{ari::Unit::Px, 240.0f};
  pinned.size.h = ari::Extent{ari::Unit::Px, 120.0f};
  pinned.maximizable = false; // and below, no close button either: a window with neither
  pinned.closable = false;
  ari::Widget plain = ari::window("Plain", {ari::text("defaults")}); // nothing said: both buttons
  plain.id = "plain";
  plain.has_pos = true;
  plain.pos_x = 600.0f;
  plain.pos_y = 80.0f;
  plain.size.w = ari::Extent{ari::Unit::Px, 200.0f};
  plain.size.h = ari::Extent{ari::Unit::Px, 100.0f};
  chk(plain.closable && plain.maximizable, "a window has a close and a maximize button by default");
  Session s(ari::group({post, pinned, plain}));
  chk(s.be.capabilities().maximize && s.rt.has("ui.maximize"),
      "the backend maximizes (ui.maximize)");
  ImGuiWindow *w = s.window("Post###post");
  if (!w) {
    chk(false, "the window is there");
    return 1;
  }
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  const ImVec2 pos0 = w->Pos, size0 = w->SizeFull;
  // Its pos: is under the menu bar (the Windows menu, for a closable window).
  chk(vp->WorkPos.y > 0.0f && near(pos0, ImVec2(100.0f, 80.0f + vp->WorkPos.y)) &&
          near(size0, ImVec2(300.0f, 200.0f)),
      "the window at its place and size (" + str(pos0) + " " + str(size0) + ")");
  const std::string geometry = s.val("tree.post.geometry");

  s.tap(s.button("Post###post", true));
  s.frames(2);
  chk(s.val("tree.post.maximized") == "1", "a tap on the square: tree.post.maximized is 1");
  chk(near(w->Pos, vp->WorkPos) && near(w->SizeFull, vp->WorkSize),
      "the window fills the view under the menu bar (" + str(w->Pos) + " " + str(w->SizeFull) +
          ")");
  chk(s.val("tree.post.geometry") == geometry, "its persisted geometry is where it goes back to");

  s.tap(s.button("Post###post", true));
  s.frames(2);
  chk(s.val("tree.post.maximized") == "0", "another tap: 0");
  chk(near(w->Pos, pos0) && near(w->SizeFull, size0),
      "back at its place and size (" + str(w->Pos) + " " + str(w->SizeFull) + ")");

  cvc::state::instance(s.app)("ui.tree.post.maximized").value(std::string("1")); // the document's
  s.frames(3);
  chk(near(w->Pos, vp->WorkPos) && near(w->SizeFull, vp->WorkSize), "the document maximizes it");
  cvc::state::instance(s.app)("ui.tree.post.maximized").value(std::string("0"));
  s.frames(3);
  chk(near(w->Pos, pos0) && near(w->SizeFull, size0), "and puts it back");

  ImGuiWindow *p = s.window("Pinned###pinned");
  const ImVec2 ppos = p ? p->Pos : ImVec2(0.0f, 0.0f);
  s.tap(s.button("Pinned###pinned", false));
  s.frames(2);
  chk(p && near(p->Pos, ppos) && s.val("tree.pinned.maximized").empty(),
      "maximizable: false has no button");
  s.tap(s.close_button("Pinned###pinned"));
  s.frames(2);
  chk(p && p->Active && s.val("tree.pinned.open") != "0", "closable: false has no close button");

  // The glyph: one square, two while maximized; resting on it names what it does.
  const ImVec2 away(990.0f, 690.0f);
  s.frames(2);
  const int one = s.glyph_vertices("Post###post");
  s.rest(s.button("Post###post", true), 0.6f);
  chk(s.tooltip_says("Maximize"), "resting on the button: a Maximize tooltip");
  s.tap(s.button("Post###post", true));
  s.rest(away, 0.1f);
  const int two = s.glyph_vertices("Post###post");
  chk(one > 0 && two > one, "maximized, the button draws two squares (" + std::to_string(one) +
                                " -> " + std::to_string(two) + " vertices)");
  s.rest(s.button("Post###post", true), 0.6f);
  chk(s.tooltip_says("Restore"), "and its tooltip says Restore");
  s.tap(s.button("Post###post", true));
  s.rest(away, 0.1f);
  chk(s.val("tree.post.maximized") == "0" && s.glyph_vertices("Post###post") == one,
      "restored: one square again");
  s.rest(s.close_button("Post###post"), 0.6f);
  chk(s.tooltip_says("Close"), "resting on the X: a Close tooltip");
  s.rest(away, 0.1f);

  // A double click on the title bar maximizes the window, and another puts it back; it doesn't
  // collapse it, as ImGui's own double click would.
  const ImVec2 title(w->Pos.x + 120.0f, w->TitleBarRect().GetCenter().y);
  s.double_click(title);
  s.frames(2);
  chk(s.val("tree.post.maximized") == "1" && !w->Collapsed && near(w->Pos, vp->WorkPos) &&
          near(w->SizeFull, vp->WorkSize),
      "a double click on the title bar maximizes it, not collapsed");
  s.double_click(ImVec2(vp->WorkPos.x + 120.0f, w->TitleBarRect().GetCenter().y));
  s.frames(2);
  chk(s.val("tree.post.maximized") == "0" && !w->Collapsed && near(w->Pos, pos0) &&
          near(w->SizeFull, size0),
      "another puts it back (" + str(w->Pos) + " " + str(w->SizeFull) + ")");

  // Resized by its corner while maximized, it is no longer maximized, and that size is its own: a
  // maximize and a restore come back to it.
  s.tap(s.button("Post###post", true));
  s.frames(2);
  const ImVec2 corner = vp->WorkPos + vp->WorkSize - ImVec2(3.0f, 3.0f);
  s.drag(corner, corner - ImVec2(300.0f, 200.0f));
  s.frames(2);
  const ImVec2 resized = vp->WorkSize - ImVec2(300.0f, 200.0f);
  chk(s.val("tree.post.maximized") == "0", "resized by its corner: no longer maximized");
  chk(near(w->Pos, vp->WorkPos) && near(w->SizeFull, resized),
      "at the size it was given (" + str(w->Pos) + " " + str(w->SizeFull) + ")");
  chk(s.glyph_vertices("Post###post") == one, "its button is the maximize square again");
  chk(s.val("tree.post.geometry") != geometry, "and that geometry is persisted");
  s.tap(s.button("Post###post", true));
  s.frames(2);
  chk(s.val("tree.post.maximized") == "1" && near(w->SizeFull, vp->WorkSize), "maximized again");
  s.tap(s.button("Post###post", true));
  s.frames(2);
  chk(s.val("tree.post.maximized") == "0" && near(w->Pos, vp->WorkPos) &&
          near(w->SizeFull, resized),
      "restored to the size it was resized to (" + str(w->Pos) + " " + str(w->SizeFull) + ")");

  // Dragged by its title bar while maximized, it comes back to its own size under the pointer,
  // held where along the title bar it was grabbed. A press on the title bar alone doesn't.
  s.tap(s.button("Post###post", true));
  s.frames(2);
  const ImVec2 grab(vp->WorkPos.x + vp->WorkSize.x * 0.25f, w->TitleBarRect().GetCenter().y);
  s.tap(grab);
  s.frames(2);
  chk(s.val("tree.post.maximized") == "1" && near(w->Pos, vp->WorkPos),
      "a press on the title bar leaves it maximized");
  s.frames(30); // past the double-click time: the drag's press is a click of its own
  const ImVec2 to = grab + ImVec2(150.0f, 120.0f);
  s.drag(grab, to);
  s.frames(2);
  chk(s.val("tree.post.maximized") == "0", "dragged by its title bar: no longer maximized");
  chk(near(w->SizeFull, resized), "at its own size (" + str(w->SizeFull) + ")");
  const float held = to.x - w->Pos.x;
  chk(std::fabs(held - resized.x * 0.25f) < 2.0f &&
          std::fabs(to.y - w->Pos.y - (grab.y - vp->WorkPos.y)) < 2.0f,
      "under the pointer, a quarter of the way along (" + str(w->Pos) + ")");

  // The close button closes it (tree.post.open), and the document opens it again.
  s.tap(s.close_button("Post###post"));
  s.frames(2);
  chk(s.val("tree.post.open") == "0" && !w->Active, "the X closes it");
  cvc::state::instance(s.app)("ui.tree.post.open").value(std::string("1"));
  s.frames(2);
  chk(w->Active && near(w->SizeFull, resized), "and it opens again at its size");

  // A window with nothing said has both buttons.
  ImGuiWindow *pl = s.window("Plain###plain");
  const ImVec2 plpos = pl ? pl->Pos : ImVec2(0.0f, 0.0f);
  s.tap(s.button("Plain###plain", true));
  s.frames(2);
  chk(pl && s.val("tree.plain.maximized") == "1" && near(pl->SizeFull, vp->WorkSize),
      "the plain window maximizes");
  s.tap(s.button("Plain###plain", true));
  s.frames(2);
  chk(pl && near(pl->Pos, plpos), "and restores");
  s.tap(s.close_button("Plain###plain"));
  s.frames(2);
  chk(pl && s.val("tree.plain.open") == "0" && !pl->Active, "and closes");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

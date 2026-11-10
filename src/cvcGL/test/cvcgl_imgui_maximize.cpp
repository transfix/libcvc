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
  pinned.maximizable = false;
  Session s(ari::group({post, pinned}));
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

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

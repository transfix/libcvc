// ImGuiBackend's tab bar and list (Kind::Tabs, Kind::List) on a bare ImGui context (no GL),
// through ImGui's own mouse input:
//   - a click on a tab writes its title to the bound key and shows its subtree; a write to the key
//     from outside selects that tab;
//   - a click on a row selects it; Add appends a row; Remove deletes the selected one, and is
//     disabled while nothing is selected.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
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
// The ImGui backend, noting where each tab bar and list starts and the last item each drew, so the
// test can aim its clicks; and which texts were drawn this frame.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Where {
    ImVec2 start, last_min, last_max;
  };
  std::map<std::string, Where> at;
  std::map<std::string, int> texts;
  ari::IndexEdit tab_bar(const char *label, const char *id, int current,
                         const std::vector<std::string> &labels) override {
    Where w;
    w.start = ImGui::GetCursorScreenPos();
    const ari::IndexEdit e = ImGuiBackend::tab_bar(label, id, current, labels);
    w.last_min = ImGui::GetItemRectMin();
    w.last_max = ImGui::GetItemRectMax();
    at[id] = w;
    return e;
  }
  ari::ListEdit list(const char *label, const char *id, int current,
                     const std::vector<std::string> &items, const ari::ListOptions &o) override {
    Where w;
    w.start = ImGui::GetCursorScreenPos();
    const ari::ListEdit e = ImGuiBackend::list(label, id, current, items, o);
    w.last_min = ImGui::GetItemRectMin();
    w.last_max = ImGui::GetItemRectMax();
    at[label] = w;
    return e;
  }
  void text_line(const char *t) override {
    ++texts[t];
    ImGuiBackend::text_line(t);
  }
};

const char *kDoc = R"(meta: { name: tabs and list, min_libcvc: 3.7.0 }
windows:
  - window: Panel
    pos: [10, 10]
    size: [600, 600]
    children:
      - type: tabs
        id: side
        bind: panel
        children:
          - { type: group, tab: Probe, children: [ { text: probe body } ] }
          - { type: group, tab: Classes, children: [ { text: classes body } ] }
      - list: Point classes
        bind: classes
        add: true
        remove: true
        rows: 4
        new_item: { name: "class {n}" }
)";

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 700.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h);
    const ari::LoadResult r = ari::load_string(app, kDoc);
    for (const std::string &warn : r.warnings)
      chk(false, "loads without warnings: " + warn);
    rt.set_backend(&be);
    rt.set_root(r.root);
    for (int f = 0; f < 4; ++f)
      frame(ImVec2(990.0f, 690.0f), false);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame(ImVec2 mouse, bool down) {
    be.texts.clear();
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, down);
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    rt.drain();
  }
  void click(ImVec2 p) {
    frame(p, false);
    frame(p, true);
    frame(p, false);
    frame(p, false);
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
  Session s;
  chk(s.be.at.count("side") && s.be.at.count("Point classes"), "the tab bar and the list draw");
  chk(s.val("panel") == "Probe", "the first tab is seeded (" + s.val("panel") + ")");
  chk(s.be.texts.count("probe body") && !s.be.texts.count("classes body"),
      "only the selected tab's subtree draws");

  // The second tab: past the first, whose width is its title's plus the frame padding.
  const ImGuiStyle &style = ImGui::GetStyle();
  const RecordingBackend::Where bar = s.be.at["side"];
  const float first = ImGui::CalcTextSize("Probe").x + 2.0f * style.FramePadding.x;
  const ImVec2 second(bar.start.x + first + style.ItemInnerSpacing.x +
                          0.5f * (ImGui::CalcTextSize("Classes").x + 2.0f * style.FramePadding.x),
                      bar.start.y + 0.5f * ImGui::GetFrameHeight());
  s.click(second);
  chk(s.val("panel") == "Classes", "a click on a tab writes its title (" + s.val("panel") + ")");
  chk(s.be.texts.count("classes body") && !s.be.texts.count("probe body"), "and shows its subtree");
  s.set("panel", "Probe");
  s.frame(ImVec2(990.0f, 690.0f), false);
  s.frame(ImVec2(990.0f, 690.0f), false);
  chk(s.be.texts.count("probe body") && s.val("panel") == "Probe",
      "a write from outside selects that tab, and the bar does not undo it");

  // The list: rows, a click selects, Add, Remove.
  s.set("classes", "2");
  s.set("classes.0.name", "membrane");
  s.set("classes.1.name", "background");
  s.frame(ImVec2(990.0f, 690.0f), false);
  const RecordingBackend::Where lst = s.be.at["Point classes"];
  const float row = ImGui::GetTextLineHeightWithSpacing();
  // The list's caption line, then its frame; row 1 is the second line inside it.
  const ImVec2 row1(lst.start.x + 40.0f, lst.start.y + ImGui::GetTextLineHeightWithSpacing() +
                                             style.FramePadding.y + 1.5f * row);
  s.click(row1);
  chk(s.val("classes.current") == "1", "a click selects row 1 (" + s.val("classes.current") + ")");
  // Remove is the last item the list drew.
  const RecordingBackend::Where after = s.be.at["Point classes"];
  const ImVec2 remove((after.last_min.x + after.last_max.x) * 0.5f,
                      (after.last_min.y + after.last_max.y) * 0.5f);
  s.click(remove);
  chk(s.val("classes") == "1" && s.val("classes.0.name") == "membrane",
      "Remove deletes the selected row (" + s.val("classes") + " left)");
  // Add sits left of Remove.
  const float add_w = ImGui::CalcTextSize("Add").x + 2.0f * style.FramePadding.x;
  const ImVec2 add(after.last_min.x - style.ItemSpacing.x - 0.5f * add_w, remove.y);
  s.click(add);
  chk(s.val("classes") == "2" && s.val("classes.1.name") == "class 2" &&
          s.val("classes.current") == "1",
      "Add appends a row and selects it (" + s.val("classes.1.name") + ")");
  // Nothing selected: Remove is disabled.
  s.set("classes.current", "-1");
  s.frame(ImVec2(990.0f, 690.0f), false);
  s.click(remove);
  chk(s.val("classes") == "2", "with nothing selected Remove does nothing");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

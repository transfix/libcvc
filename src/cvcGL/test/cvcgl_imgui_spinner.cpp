// ImGuiBackend's spinner (Kind::Spinner) on a bare ImGui context (no GL): a turning arc a text line
// high, then the label. The arc's vertices move with the frame clock (Runtime::set_frame_clock)
// and come round again after a second; disabled, it stands still; visible_when hides it.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <map>
#include <string>
#include <vector>
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
// The ImGui backend, keeping what each spinner put in the window's draw list (its arc's vertex
// positions) and where its label went.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Drawn {
    std::vector<ImVec2> arc;
    ImVec2 start, label_min;
    int calls = 0;
  };
  std::map<std::string, Drawn> at;
  void spinner(const char *label, double seconds, bool ascii) override {
    ImDrawList *dl = ImGui::GetWindowDrawList();
    const int v0 = dl->VtxBuffer.Size;
    Drawn &d = at[label];
    d.start = ImGui::GetCursorScreenPos();
    ImGuiBackend::spinner(label, seconds, ascii);
    d.label_min = ImGui::GetItemRectMin();
    d.arc.clear();
    for (int i = v0; i < dl->VtxBuffer.Size; ++i)
      d.arc.push_back(dl->VtxBuffer[i].pos);
    ++d.calls;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800.0f, 600.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    for (int f = 0; f < 3; ++f)
      frame(0.0);
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame(double clock) {
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    rt.set_frame_clock(clock);
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
  }
};

// Whether two arcs are drawn the same (vertex for vertex, within a hair).
bool same(const std::vector<ImVec2> &a, const std::vector<ImVec2> &b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (std::fabs(a[i].x - b[i].x) > 0.01f || std::fabs(a[i].y - b[i].y) > 0.01f)
      return false;
  return true;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget off = ari::spinner("Off");
  off.enabled_when = "(= 1 2)";
  ari::Widget hidden = ari::spinner("Hidden");
  hidden.visible_when = "(= 1 2)";
  Session s(ari::window("Spinners", {ari::spinner("Loading"), off, hidden}));

  s.frame(0.0);
  const std::vector<ImVec2> at0 = s.be.at["Loading"].arc, off0 = s.be.at["Off"].arc;
  chk(at0.size() > 8, "the spinner draws an arc (" + std::to_string(at0.size()) + " vertices)");
  const float line = ImGui::GetTextLineHeight();
  const RecordingBackend::Drawn d = s.be.at["Loading"];
  chk(d.label_min.x >= d.start.x + line && std::fabs(d.label_min.y - d.start.y) < 0.5f * line,
      "the label follows the arc on its line");
  s.frame(0.25);
  chk(!same(at0, s.be.at["Loading"].arc), "a quarter second on, the arc has turned");
  chk(same(off0, s.be.at["Off"].arc), "a disabled spinner stands still");
  s.frame(1.0);
  chk(same(at0, s.be.at["Loading"].arc), "a second on, it has come round again");
  chk(s.be.at.count("Hidden") == 0, "visible_when hides one");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

// ImGuiBackend's radio (Kind::Radio) on a bare ImGui context (no GL): a radio stacks its options
// under the caption and a radio_row puts them on the caption's line; a click on an option, through
// ImGui's own mouse input, stores it (or its mapped value) and runs the radio's on:.
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
// The ImGui backend, noting where each radio starts (its caption) and where its last option was
// drawn (ImGui's last item after the radio), so the test can compare the two and click the option.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Where {
    ImVec2 start, last_min, last_max;
  };
  std::map<std::string, Where> at;
  ari::IndexEdit radio(const char *label, int current_index,
                       const std::vector<std::string> &options, bool row) override {
    Where w;
    w.start = ImGui::GetCursorScreenPos();
    const ari::IndexEdit e = ImGuiBackend::radio(label, current_index, options, row);
    w.last_min = ImGui::GetItemRectMin();
    w.last_max = ImGui::GetItemRectMax();
    at[label] = w;
    return e;
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
  // Press and release over the last option of radio `label`, then let the UI settle a frame.
  bool click_last(const std::string &label) {
    const auto it = be.at.find(label);
    if (it == be.at.end())
      return false;
    const ImVec2 p((it->second.last_min.x + it->second.last_max.x) * 0.5f,
                   (it->second.last_min.y + it->second.last_max.y) * 0.5f);
    frame(p, false);
    frame(p, true);
    frame(p, false);
    frame(p, false);
    return true;
  }
  std::string val(const std::string &key) { return cvc::state::instance(app)("ui." + key).value(); }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget stacked = ari::radio("Mode", "mode", {"alpha", "beta", "gamma"});
  stacked.on = "(state-set \"picked\" (state-get \"mode\"))";
  ari::Widget row = ari::radio("Samples", "ss", {"Off", "2x", "4x"}, "0", /*row*/ true);
  row.values = {"0", "2", "4"};
  Session s(ari::window("Radios", {stacked, row}));

  chk(s.be.at.count("Mode") && s.be.at.count("Samples"), "both radios are drawn");
  chk(s.val("mode") == "alpha", "the stacked radio seeds its first option (" + s.val("mode") + ")");
  const float line = ImGui::GetTextLineHeight();
  const RecordingBackend::Where m = s.be.at["Mode"], r = s.be.at["Samples"];
  chk(m.last_min.y > m.start.y + 2.0f * line, "a radio stacks its options under the caption");
  chk(r.last_min.y < r.start.y + line, "a radio_row keeps its options on the caption's line");

  chk(s.click_last("Mode"), "click the stacked radio's last option");
  chk(s.val("mode") == "gamma", "it stores the option (" + s.val("mode") + ")");
  s.rt.drain();
  chk(s.val("picked") == "gamma", "and runs the radio's on: (" + s.val("picked") + ")");

  chk(s.click_last("Samples"), "click the row's last option");
  chk(s.val("ss") == "4", "it stores the mapped value, not the label (" + s.val("ss") + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

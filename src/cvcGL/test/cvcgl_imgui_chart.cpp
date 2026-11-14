// ImGuiBackend's chart (Kind::Chart) on a bare ImGui context (no GL): the canvas is the last item
// and as tall as `height`, it draws its series, a click on a legend entry hides that series (and
// another shows it again), hovering adds a crosshair, and with `pick:` a click at the plot's right
// edge writes the last x.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <string>
#include <utility>
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
// The ImGui backend, keeping where each chart started, its canvas (the last item) and how many
// vertices it drew.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  ImVec2 start, canvas_min, canvas_max;
  int vertices = 0;
  ari::ChartEvent event;
  ari::ChartEvent chart(const char *label, const ari::ChartSpec &spec) override {
    ImDrawList *dl = ImGui::GetWindowDrawList();
    const int v0 = dl->VtxBuffer.Size;
    start = ImGui::GetCursorScreenPos();
    event = ImGuiBackend::chart(label, spec);
    canvas_min = ImGui::GetItemRectMin();
    canvas_max = ImGui::GetItemRectMax();
    vertices = dl->VtxBuffer.Size - v0;
    return event;
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
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame() {
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
  }
  void move(ImVec2 p) {
    ImGui::GetIO().AddMousePosEvent(p.x, p.y);
    frame();
  }
  // A click at p: hover, press, release, a frame each.
  void click(ImVec2 p) {
    move(p);
    ImGui::GetIO().AddMouseButtonEvent(0, true);
    frame();
    ImGui::GetIO().AddMouseButtonEvent(0, false);
    frame();
  }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget c = ari::chart("Loss", "learn");
  c.log_y = "1";
  c.pick = "seek";
  c.x_label = "step";
  ari::Widget win = ari::window("Charts", {c});
  win.has_pos = true;
  win.size.w = {ari::Unit::Px, 600.0f};
  win.size.h = {ari::Unit::Px, 400.0f};
  Session s(win);
  cvc::state &st = cvc::state::instance(s.app);
  st("ui.learn.s.0.label").value("train");
  st("ui.learn.s.0.x").value("0,10,20,30,40,50,60,70,80,90,100");
  st("ui.learn.s.0.y").value("2,1.2,0.8,0.5,0.4,0.3,0.25,0.2,0.18,0.17,0.16");
  st("ui.learn.s.1.label").value("held-out");
  st("ui.learn.s.1.x").value("0,50,100");
  st("ui.learn.s.1.y").value("2.1,,0.3");
  st("ui.learn.s.1.lo").value("1.9,,0.25");
  st("ui.learn.s.1.hi").value("2.3,,0.35");
  st("ui.learn.m.0.x").value("90");
  st("ui.learn.m.0.kind").value("best");
  for (int f = 0; f < 3; ++f)
    s.frame();

  const float line = ImGui::GetTextLineHeight();
  const ImGuiStyle &style = ImGui::GetStyle();
  const float canvas_h = s.be.canvas_max.y - s.be.canvas_min.y;
  chk(std::fabs(canvas_h - 160.0f) < 0.5f,
      "the canvas is the last item, `height` tall (" + std::to_string(canvas_h) + ")");
  chk(s.be.canvas_min.y >= s.be.start.y + 2.0f * line,
      "the label and the legend come above the canvas");
  const int drawn = s.be.vertices;
  chk(drawn > 200, "it draws its axes, band, lines and marker (" + std::to_string(drawn) + ")");

  // The legend's entries: a swatch (18 px) and the label, two item spacings apart, on the line
  // under the chart's label. A click on held-out's hides it, so less is drawn; a second shows it.
  const float row = s.be.start.y + line + style.ItemSpacing.y + 0.5f * line;
  const float first_w = 18.0f + style.ItemInnerSpacing.x + ImGui::CalcTextSize("train").x;
  const ImVec2 second(s.be.start.x + first_w + 2.0f * style.ItemSpacing.x + 6.0f, row);
  s.move(ImVec2(5.0f, 590.0f)); // off the chart: no crosshair
  const int shown = s.be.vertices;
  s.click(second);
  s.move(ImVec2(5.0f, 590.0f));
  chk(s.be.vertices < shown, "a click on a legend entry hides its series (" +
                                 std::to_string(shown) + " -> " + std::to_string(s.be.vertices) +
                                 " vertices)");
  s.click(second);
  s.move(ImVec2(5.0f, 590.0f));
  chk(s.be.vertices == shown, "a second click shows it again");

  // Hovering the canvas adds a crosshair and a dot on each shown series.
  const ImVec2 mid(0.5f * (s.be.canvas_min.x + s.be.canvas_max.x),
                   0.5f * (s.be.canvas_min.y + s.be.canvas_max.y));
  s.move(mid);
  chk(s.be.vertices > shown, "hovering draws a crosshair and the nearest points");

  // A click just inside the plot's right edge (14 px in from the canvas') picks the last x.
  chk(st("ui.seek").value().empty(), "nothing picked yet");
  s.click(ImVec2(s.be.canvas_max.x - 15.0f, mid.y));
  chk(st("ui.seek").value() == "100",
      "a click picks the nearest x (" + st("ui.seek").value() + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

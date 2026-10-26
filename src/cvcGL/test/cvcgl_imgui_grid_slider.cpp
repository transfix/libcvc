// A slider in a grid cell holds a steady width (ImGuiBackend's reserve_label_width), on a bare
// ImGui context (no window, no GL). The conference roster's volume slider sat in an auto-width
// column: ImGui re-fits such a column to its content each frame, and the slider took ~65% of the
// column, so the column shrank every frame until a 48 px floor snapped it back and the slider could
// not be dragged. A slider in a cell now fills the cell, so an auto column settles at once.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <string>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiContext::Tables: the drawn column widths
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
// One roster row: six fixed columns, then the slider's column `last` (Auto or a fixed width).
ari::Widget row(const ari::Track &last) {
  ari::Widget g = ari::group({ari::text("name"), ari::text("12 ms"), ari::text("1.2M"),
                              ari::text("1.2M"), ari::text("c"), ari::text("m"),
                              ari::slider_float("##vol", "peer.vol", 0.0, 2.0, 1.0)});
  g.id = "row";
  g.layout.kind = ari::LayoutKind::Grid;
  for (float w : {110.0f, 52.0f, 52.0f, 52.0f, 30.0f, 30.0f})
    g.layout.col_widths.push_back(ari::Track{ari::Unit::Px, w});
  g.layout.col_widths.push_back(last);
  return g;
}

// The slider column's drawn width on each of `frames` frames, in a `host_w` px window.
std::vector<float> slider_column_widths(const ari::Track &last, float host_w, int frames) {
  ImGuiContext *ctx = ImGui::CreateContext();
  // libcvcGL may carry its own copy of imgui (a static imgui, as on Windows): hand it this context.
  cvc::gl::ImGuiBackend::setImGuiContext(ctx);
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1200.0f, 600.0f);
  io.IniFilename = nullptr;
  unsigned char *px = nullptr;
  int w = 0, h = 0;
  io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
  std::vector<float> out;
  {
    cvc::app app;
    ari::Runtime rt(app, "ui");
    cvc::gl::ImGuiBackend be;
    rt.set_backend(&be);
    rt.set_root(ari::group({row(last)}));
    for (int f = 0; f < frames; ++f) {
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      ImGui::SetNextWindowSize(ImVec2(host_w, 200.0f));
      ImGui::Begin("host");
      rt.render();
      ImGui::End();
      ImGui::Render();
      // The context's one 7-column table is the row (read through ctx, not GImGui: the test and
      // libcvcGL may each have their own imgui globals).
      float width = -1.0f;
      for (int i = 0; i < ctx->Tables.GetMapSize(); ++i)
        if (ImGuiTable *t = ctx->Tables.TryGetMapData(i))
          if (t->ColumnsCount == 7)
            width = t->Columns[6].WidthGiven;
      out.push_back(width);
    }
  }
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
  ImGui::DestroyContext(ctx);
  return out;
}

// Whether the last `n` widths are all within half a pixel of each other.
bool settled(const std::vector<float> &v, std::size_t n) {
  if (v.size() < n)
    return false;
  for (std::size_t i = v.size() - n; i < v.size(); ++i)
    if (std::fabs(v[i] - v.back()) > 0.5f)
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
  // The roster before the fix: an auto-width slider column in a roomy window.
  const std::vector<float> autow =
      slider_column_widths(ari::Track{ari::Unit::Auto, 0.0f}, 520.0f, 30);
  chk(settled(autow, 20), "an auto-width slider column settles (last " +
                              std::to_string(autow.empty() ? -1.0f : autow.back()) + " px)");
  chk(!autow.empty() && autow.back() >= 40.0f, "and stays wide enough to drag");

  // The roster after the fix: a fixed 72 px slider column, in the 470 px window.
  const std::vector<float> fixed =
      slider_column_widths(ari::Track{ari::Unit::Px, 72.0f}, 470.0f, 10);
  chk(settled(fixed, 8), "a fixed slider column holds its width");
  chk(!fixed.empty() && std::fabs(fixed.back() - 72.0f) < 1.0f,
      "at its authored 72 px (got " + std::to_string(fixed.empty() ? -1.0f : fixed.back()) + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

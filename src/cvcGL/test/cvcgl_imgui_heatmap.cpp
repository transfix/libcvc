// ImGuiBackend's heatmap (Kind::Heatmap) on a bare ImGui context (no GL), checked by its pixels:
// the frames' draw data are rasterized here (each triangle's colours interpolated, times the font
// atlas where it samples one, blended over), and the test reads the colours back. Each cell is
// filled with the ramp's colour at its value (clamped outside the ramp), a marked cell has a
// triangle in the mark's colour in its top-right corner, the picked cell is outlined, hovering a
// cell shows its tip in a tooltip, and a click on a cell writes the pick.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <string>
#include <utility>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h> // FindWindowByName: the tooltip's window
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
constexpr int kW = 640, kH = 480;

// ImGui's draw data, rasterized: what a renderer would show, without one.
struct Canvas {
  std::vector<float> rgb = std::vector<float>(kW * kH * 3, 0.0f);

  void draw(const ImDrawData *dd) {
    std::fill(rgb.begin(), rgb.end(), 0.0f);
    for (const ImDrawList *dl : dd->CmdLists)
      for (const ImDrawCmd &cmd : dl->CmdBuffer) {
        if (cmd.UserCallback)
          continue;
        const ImTextureData *tex = cmd.TexRef._TexData;
        for (unsigned i = 0; i + 2 < cmd.ElemCount; i += 3) {
          const ImDrawIdx *ix = dl->IdxBuffer.Data + cmd.IdxOffset + i;
          tri(dl->VtxBuffer[cmd.VtxOffset + ix[0]], dl->VtxBuffer[cmd.VtxOffset + ix[1]],
              dl->VtxBuffer[cmd.VtxOffset + ix[2]], cmd.ClipRect, tex);
        }
      }
  }

  static float channel(ImU32 c, int shift) {
    return static_cast<float>((c >> shift) & 0xff) / 255.f;
  }

  // One triangle: each pixel whose centre it covers, its colour the vertices' mixed by where the
  // centre falls, times the texel there (the atlas' white pixel for a fill), blended over.
  void tri(const ImDrawVert &a, const ImDrawVert &b, const ImDrawVert &c, const ImVec4 &clip,
           const ImTextureData *tex) {
    const float x0 = std::max({clip.x, std::min({a.pos.x, b.pos.x, c.pos.x}), 0.0f});
    const float x1 = std::min({clip.z, std::max({a.pos.x, b.pos.x, c.pos.x}), float(kW)});
    const float y0 = std::max({clip.y, std::min({a.pos.y, b.pos.y, c.pos.y}), 0.0f});
    const float y1 = std::min({clip.w, std::max({a.pos.y, b.pos.y, c.pos.y}), float(kH)});
    const auto edge = [](const ImVec2 &p, const ImVec2 &q, float x, float y) {
      return (q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x);
    };
    const float area = edge(a.pos, b.pos, c.pos.x, c.pos.y);
    if (area == 0.0f)
      return;
    for (int y = static_cast<int>(std::floor(y0)); y < static_cast<int>(std::ceil(y1)); ++y)
      for (int x = static_cast<int>(std::floor(x0)); x < static_cast<int>(std::ceil(x1)); ++x) {
        const float px = x + 0.5f, py = y + 0.5f;
        if (px < clip.x || px >= clip.z || py < clip.y || py >= clip.w)
          continue;
        const float wa = edge(b.pos, c.pos, px, py) / area;
        const float wb = edge(c.pos, a.pos, px, py) / area;
        const float wc = 1.0f - wa - wb;
        if (wa < 0.0f || wb < 0.0f || wc < 0.0f)
          continue;
        float col[4];
        for (int k = 0; k < 4; ++k)
          col[k] =
              wa * channel(a.col, 8 * k) + wb * channel(b.col, 8 * k) + wc * channel(c.col, 8 * k);
        if (tex && tex->Pixels) {
          const float u = wa * a.uv.x + wb * b.uv.x + wc * c.uv.x;
          const float v = wa * a.uv.y + wb * b.uv.y + wc * c.uv.y;
          const int tx = std::clamp(static_cast<int>(u * tex->Width), 0, tex->Width - 1);
          const int ty = std::clamp(static_cast<int>(v * tex->Height), 0, tex->Height - 1);
          const unsigned char *t = tex->Pixels + (tx + ty * tex->Width) * tex->BytesPerPixel;
          if (tex->BytesPerPixel == 4)
            for (int k = 0; k < 4; ++k)
              col[k] *= t[k] / 255.0f;
          else
            col[3] *= t[0] / 255.0f;
        }
        float *d = &rgb[(y * kW + x) * 3];
        for (int k = 0; k < 3; ++k)
          d[k] = col[k] * col[3] + d[k] * (1.0f - col[3]);
      }
  }

  // The pixel at (x, y) as 0..255.
  void at(float x, float y, int out[3]) const {
    const float *d = &rgb[(static_cast<int>(y) * kW + static_cast<int>(x)) * 3];
    for (int k = 0; k < 3; ++k)
      out[k] = static_cast<int>(std::lround(d[k] * 255.0f));
  }
  bool is(float x, float y, int r, int g, int b, int tol = 1) const {
    int p[3];
    at(x, y, p);
    return std::abs(p[0] - r) <= tol && std::abs(p[1] - g) <= tol && std::abs(p[2] - b) <= tol;
  }
  std::string text(float x, float y) const {
    int p[3];
    at(x, y, p);
    return "(" + std::to_string(p[0]) + "," + std::to_string(p[1]) + "," + std::to_string(p[2]) +
           ")";
  }
};

// The ImGui backend, keeping where the heatmap's item starts and the tooltips it is asked for.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  ImVec2 origin;
  std::vector<std::string> tips;
  ari::HeatmapEvent heatmap(const char *label, const ari::HeatmapSpec &spec) override {
    origin = ImGui::GetCursorScreenPos(); // the label "##q" draws nothing
    return ImGuiBackend::heatmap(label, spec);
  }
  void set_tooltip(const char *text) override {
    tips.emplace_back(text ? text : "");
    ImGuiBackend::set_tooltip(text);
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  Canvas canvas;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(float(kW), float(kH));
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
    be.tips.clear();
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
    canvas.draw(ImGui::GetDrawData());
  }
  void move(ImVec2 p) {
    ImGui::GetIO().AddMousePosEvent(p.x, p.y);
    frame();
  }
  void click(ImVec2 p) {
    move(p);
    ImGui::GetIO().AddMouseButtonEvent(0, true);
    frame();
    ImGui::GetIO().AddMouseButtonEvent(0, false);
    frame();
  }
};

ari::HeatmapStop stop(double at, int r, int g, int b) {
  ari::HeatmapStop s;
  s.at = at;
  s.rgb[0] = r / 255.0f;
  s.rgb[1] = g / 255.0f;
  s.rgb[2] = b / 255.0f;
  return s;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  // Red at 0, amber at 10, green at 20; 30 px cells, 2 px apart; the mark blue.
  ari::Widget hm =
      ari::heatmap("##q", "q", {stop(0, 200, 0, 0), stop(10, 240, 160, 0), stop(20, 0, 160, 0)});
  hm.heatmap.cell_size = 30.0f;
  hm.heatmap.gap = 2.0f;
  hm.heatmap.mark_rgb[0] = 0.0f;
  hm.heatmap.mark_rgb[1] = 0.0f;
  hm.heatmap.mark_rgb[2] = 1.0f;
  hm.pick = "sel";
  ari::Widget win = ari::window("Heat", {hm});
  win.has_pos = true;
  win.pos_x = 20.0f;
  win.pos_y = 20.0f;
  win.size.w = {ari::Unit::Px, 400.0f};
  win.size.h = {ari::Unit::Px, 300.0f};
  Session s(win);
  cvc::state &st = cvc::state::instance(s.app);
  st("ui.q.rows").value("2");
  st("ui.q.cols").value("3");
  st("ui.q.cells.0.0.v").value("0");
  st("ui.q.cells.0.1.v").value("5");
  st("ui.q.cells.0.2.v").value("40"); // past the ramp: green
  st("ui.q.cells.1.0.v").value("-3"); // before it: red
  st("ui.q.cells.1.1.v").value("15");
  st("ui.q.cells.1.1.mark").value("1");
  st("ui.q.cells.1.1.tip").value("pair 1-1");
  st("ui.q.cells.1.2.v").value("10");
  s.move(ImVec2(5.0f, 470.0f)); // off the window
  for (int f = 0; f < 3; ++f)
    s.frame();

  const ImVec2 o = s.be.origin;
  const float pitch = 32.0f;
  const auto centre = [&](int r, int c) {
    return ImVec2(o.x + c * pitch + 15.0f, o.y + r * pitch + 15.0f);
  };
  const auto cell_is = [&](int r, int c, int R, int G, int B, const std::string &what) {
    const ImVec2 p = centre(r, c);
    chk(s.canvas.is(p.x, p.y, R, G, B), what + " (" + s.canvas.text(p.x, p.y) + " at " +
                                            std::to_string(r) + "," + std::to_string(c) + ")");
  };
  cell_is(0, 0, 200, 0, 0, "0 is the first stop's red");
  cell_is(0, 1, 220, 80, 0, "5 is half way from red to amber");
  cell_is(0, 2, 0, 160, 0, "40, past the ramp, is the last stop's green");
  cell_is(1, 0, 200, 0, 0, "-3, before it, is the first stop's red");
  cell_is(1, 2, 240, 160, 0, "10 is the middle stop's amber");
  // (1, 1) is 15: half way from amber to green, (120, 160, 0), except its top-right corner.
  const ImVec2 m = ImVec2(o.x + pitch + 0.0f, o.y + pitch + 0.0f); // its top-left
  chk(s.canvas.is(m.x + 4.0f, m.y + 26.0f, 120, 160, 0),
      "15: amber to green, half way (" + s.canvas.text(m.x + 4.0f, m.y + 26.0f) + ")");
  chk(s.canvas.is(m.x + 27.0f, m.y + 3.0f, 0, 0, 255),
      "the mark: a blue triangle in its top-right corner (" +
          s.canvas.text(m.x + 27.0f, m.y + 3.0f) + ")");
  chk(s.canvas.is(m.x + 15.0f, m.y + 15.0f, 120, 160, 0), "... and only there");
  chk(!s.canvas.is(o.x + 31.0f, o.y + 15.0f, 200, 0, 0) &&
          !s.canvas.is(o.x + 31.0f, o.y + 15.0f, 220, 80, 0),
      "the gap between cells is neither cell's colour");

  // Hovering (1, 1) asks for its tip, and ImGui draws a tooltip there.
  const ImVec2 hover = centre(1, 1);
  s.move(hover);
  s.frame(); // a tooltip appears the frame after it is asked for
  chk(s.be.tips == std::vector<std::string>{"pair 1-1"}, "hovering a cell shows its tip");
  ImGuiWindow *tw = ImGui::FindWindowByName("##Tooltip_00");
  chk(tw && tw->Active, "ImGui opened the tooltip");
  if (tw && tw->Active) {
    const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_PopupBg];
    const ImVec2 inside(tw->Pos.x + 2.0f, tw->Pos.y + tw->Size.y - 2.0f);
    int p[3];
    s.canvas.at(inside.x, inside.y, p);
    chk(std::abs(p[0] - static_cast<int>(bg.x * bg.w * 255.0f)) <= 12,
        "its window is drawn over the grid (" + s.canvas.text(inside.x, inside.y) + ")");
  }
  s.move(centre(0, 0));
  s.frame();
  chk(s.be.tips.empty(), "a cell with no tip shows none");

  // A click on (0, 2) picks it, and the picked cell is outlined in the text colour.
  s.click(centre(0, 2));
  chk(st("ui.sel").value() == "0,2", "a click picks the cell (" + st("ui.sel").value() + ")");
  s.move(ImVec2(5.0f, 470.0f));
  const ImVec4 tc = ImGui::GetStyle().Colors[ImGuiCol_Text];
  const ImVec2 edge(o.x + 2 * pitch + 15.0f, o.y - 1.0f); // the outline's middle, above the cell
  chk(s.canvas.is(edge.x, edge.y, static_cast<int>(tc.x * 255), static_cast<int>(tc.y * 255),
                  static_cast<int>(tc.z * 255), 3),
      "the picked cell is outlined (" + s.canvas.text(edge.x, edge.y) + ")");
  cell_is(0, 2, 0, 160, 0, "its inside keeps its colour");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}

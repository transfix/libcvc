/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ImGuiBackend::heatmap — Kind::Heatmap on the ImGui draw list, with no GL of its own (desktop GL
// and WebGL2 alike). The label, then one item over the rest: the column labels above the grid, the
// row labels at its left, the cells (each in the colour the core gave it, or the frame colour
// with no value), their text in black or white, whichever reads on the cell, a marked cell's
// corner triangle, an outline round the picked cell and a fainter one round the hovered cell, and
// the legend (the ramp as a bar, its stops labelled) under the grid or at its right. The item is
// ImGui's last when the call returns, so the tooltip the core shows for the hovered cell attaches
// to it. The core owns every cvc::state read and write.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal asserts it)
#include <imgui.h>
#include <imgui_internal.h> // FindRenderedTextEnd: the label up to its "##"
#endif

namespace cvc {
namespace gl {

namespace ariadne = cvc::ariadne;

#ifdef CVC_ENABLE_IMGUI
namespace {

// A document's px at the UI's size (as ImGuiBackend.cpp's ui_px).
float px(float v) {
  const ImGuiStyle &s = ImGui::GetStyle();
  return v * s.FontScaleMain * s.FontScaleDpi;
}

ImU32 rgb_u32(const float rgb[3]) {
  const auto c = [](float v) {
    return static_cast<unsigned>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
  };
  return IM_COL32(c(rgb[0]), c(rgb[1]), c(rgb[2]), 0xff);
}

// Black or white, whichever reads on `rgb`.
ImU32 text_on(const float rgb[3]) {
  const float l = 0.2126f * rgb[0] + 0.7152f * rgb[1] + 0.0722f * rgb[2];
  return l > 0.55f ? IM_COL32(0, 0, 0, 0xff) : IM_COL32(0xff, 0xff, 0xff, 0xff);
}

std::string stop_text(double v) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.6g", v == 0.0 ? 0.0 : v);
  return buf;
}

// `text` centred in [a, z], clipped to it.
void centred(ImDrawList *dl, ImVec2 a, ImVec2 z, ImU32 color, const std::string &text) {
  if (text.empty())
    return;
  const ImVec2 ts = ImGui::CalcTextSize(text.c_str());
  const ImVec2 at(std::max(a.x, 0.5f * (a.x + z.x - ts.x)), 0.5f * (a.y + z.y - ts.y));
  dl->PushClipRect(a, z, true);
  dl->AddText(at, color, text.c_str());
  dl->PopClipRect();
}

} // namespace
#endif

ariadne::HeatmapEvent ImGuiBackend::heatmap(const char *label, const ariadne::HeatmapSpec &spec) {
#ifdef CVC_ENABLE_IMGUI
  ariadne::HeatmapEvent ev;
  const char *end = label ? ImGui::FindRenderedTextEnd(label) : label;
  if (label && end != label)
    ImGui::TextUnformatted(label, end);
  ImGui::PushID(label ? label : "heatmap");

  const ImGuiStyle &style = ImGui::GetStyle();
  const float line_h = ImGui::GetTextLineHeight();
  const float cs = std::max(px(spec.cell_size), 1.0f), gap = std::max(px(spec.gap), 0.0f);
  const float pitch = cs + gap;
  const int rows = spec.rows, cols = spec.cols;
  const float grid_w = cols > 0 ? cols * pitch - gap : 0.0f;
  const float grid_h = rows > 0 ? rows * pitch - gap : 0.0f;

  // The row labels' column (right-aligned to the grid) and the column labels' row.
  float label_w = 0.0f;
  for (const std::string &l : spec.row_labels)
    label_w = std::max(label_w, ImGui::CalcTextSize(l.c_str()).x);
  if (label_w > 0.0f)
    label_w += style.ItemInnerSpacing.x;
  const float head_h = spec.col_labels.empty() ? 0.0f : line_h + style.ItemInnerSpacing.y;

  // The legend: a bar the grid's width (at least 120 px) with the stops' values under it, or a
  // column the grid's height (at least four lines) with them at its right.
  const bool legend = spec.legend != ariadne::HeatmapLegend::Off && !spec.ramp.empty();
  const float bar = px(10.0f);
  float stops_w = 0.0f;
  for (const ariadne::HeatmapStop &s : spec.ramp)
    stops_w = std::max(stops_w, ImGui::CalcTextSize(stop_text(s.at).c_str()).x);
  float legend_w = 0.0f, legend_h = 0.0f;
  if (legend && spec.legend == ariadne::HeatmapLegend::Bottom)
    legend_h = style.ItemSpacing.y + bar + style.ItemInnerSpacing.y + line_h;
  else if (legend)
    legend_w = style.ItemSpacing.x + bar + style.ItemInnerSpacing.x + stops_w;

  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 grid0(origin.x + label_w, origin.y + head_h);
  float total_w = label_w + grid_w + legend_w, total_h = head_h + grid_h + legend_h;
  if (legend && spec.legend == ariadne::HeatmapLegend::Bottom)
    total_w = std::max(total_w, label_w + px(120.0f));
  if (legend && spec.legend == ariadne::HeatmapLegend::Right)
    total_h = std::max(total_h, head_h + 4.0f * line_h);
  ImGui::InvisibleButton("##heatmap", ImVec2(std::max(total_w, 1.0f), std::max(total_h, 1.0f)));
  const bool hovered = ImGui::IsItemHovered();
  const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);

  // The cell under the pointer: none over a gap or off the grid.
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  int hr = -1, hc = -1;
  if (hovered && pitch > 0.0f) {
    const float dx = mouse.x - grid0.x, dy = mouse.y - grid0.y;
    const int c = static_cast<int>(std::floor(dx / pitch)),
              r = static_cast<int>(std::floor(dy / pitch));
    if (dx >= 0.0f && dy >= 0.0f && c < cols && r < rows && dx - c * pitch < cs &&
        dy - r * pitch < cs) {
      hr = r;
      hc = c;
    }
  }

  ImDrawList *dl = ImGui::GetWindowDrawList();
  const ImU32 text = ImGui::GetColorU32(ImGuiCol_Text);
  const ImU32 empty = ImGui::GetColorU32(ImGuiCol_FrameBg);
  const auto cell_min = [&](int r, int c) {
    return ImVec2(grid0.x + c * pitch, grid0.y + r * pitch);
  };

  for (int c = 0; c < cols && static_cast<std::size_t>(c) < spec.col_labels.size(); ++c) {
    const ImVec2 a(grid0.x + c * pitch, origin.y);
    centred(dl, a, ImVec2(a.x + cs, a.y + line_h), text, spec.col_labels[c]);
  }
  for (int r = 0; r < rows && static_cast<std::size_t>(r) < spec.row_labels.size(); ++r) {
    const std::string &l = spec.row_labels[r];
    const ImVec2 ts = ImGui::CalcTextSize(l.c_str());
    const float y = grid0.y + r * pitch + 0.5f * (cs - ts.y);
    dl->AddText(ImVec2(grid0.x - style.ItemInnerSpacing.x - ts.x, y), text, l.c_str());
  }

  const float mark = std::max(px(5.0f), 0.35f * cs);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const ariadne::HeatmapCell &cell = spec.cells[static_cast<std::size_t>(r) * cols + c];
      const ImVec2 a = cell_min(r, c), z(a.x + cs, a.y + cs);
      dl->AddRectFilled(a, z, cell.has_value ? rgb_u32(cell.rgb) : empty);
      if (!cell.text.empty())
        centred(dl, a, z,
                cell.has_value ? text_on(cell.rgb) : ImGui::GetColorU32(ImGuiCol_TextDisabled),
                cell.text);
      if (cell.mark)
        dl->AddTriangleFilled(ImVec2(z.x - mark, a.y), ImVec2(z.x, a.y), ImVec2(z.x, a.y + mark),
                              rgb_u32(spec.mark_rgb));
    }
  if (spec.sel_row >= 0 && spec.sel_row < rows && spec.sel_col >= 0 && spec.sel_col < cols) {
    const ImVec2 a = cell_min(spec.sel_row, spec.sel_col);
    const float t = px(2.0f);
    dl->AddRect(ImVec2(a.x - 0.5f * t, a.y - 0.5f * t),
                ImVec2(a.x + cs + 0.5f * t, a.y + cs + 0.5f * t), text, 0.0f, 0, t);
  }
  if (hr >= 0) {
    const ImVec2 a = cell_min(hr, hc);
    dl->AddRect(a, ImVec2(a.x + cs, a.y + cs),
                (text & ~IM_COL32_A_MASK) | (0x99u << IM_COL32_A_SHIFT));
  }

  if (legend) {
    // The bar: each stretch between two stops a gradient, placed by the stops' values.
    const double lo = spec.ramp.front().at, hi = spec.ramp.back().at;
    const bool bottom = spec.legend == ariadne::HeatmapLegend::Bottom;
    const ImVec2 b0 = bottom ? ImVec2(grid0.x, grid0.y + grid_h + style.ItemSpacing.y)
                             : ImVec2(grid0.x + grid_w + style.ItemSpacing.x, grid0.y);
    const float len = bottom ? std::max(grid_w, px(120.0f)) : std::max(grid_h, 4.0f * line_h);
    // Along the bar: left to right, or bottom to top (the high values up).
    const auto at = [&](double v) {
      const float f = hi > lo ? static_cast<float>((v - lo) / (hi - lo)) : 0.0f;
      return bottom ? b0.x + f * len : b0.y + (1.0f - f) * len;
    };
    if (spec.ramp.size() == 1) {
      const ImVec2 z = bottom ? ImVec2(b0.x + len, b0.y + bar) : ImVec2(b0.x + bar, b0.y + len);
      dl->AddRectFilled(b0, z, rgb_u32(spec.ramp[0].rgb));
    }
    for (std::size_t i = 1; i < spec.ramp.size(); ++i) {
      const ImU32 ca = rgb_u32(spec.ramp[i - 1].rgb), cb = rgb_u32(spec.ramp[i].rgb);
      const float p = at(spec.ramp[i - 1].at), q = at(spec.ramp[i].at);
      if (bottom)
        dl->AddRectFilledMultiColor(ImVec2(p, b0.y), ImVec2(q, b0.y + bar), ca, cb, cb, ca);
      else
        dl->AddRectFilledMultiColor(ImVec2(b0.x, q), ImVec2(b0.x + bar, p), cb, cb, ca, ca);
    }
    for (const ariadne::HeatmapStop &s : spec.ramp) {
      const std::string t = stop_text(s.at);
      const ImVec2 ts = ImGui::CalcTextSize(t.c_str());
      const float p = at(s.at);
      if (bottom) {
        dl->AddLine(ImVec2(p, b0.y + bar), ImVec2(p, b0.y + bar + style.ItemInnerSpacing.y), text);
        const float x = std::clamp(p - 0.5f * ts.x, b0.x, b0.x + len - ts.x);
        dl->AddText(ImVec2(x, b0.y + bar + style.ItemInnerSpacing.y), text, t.c_str());
      } else {
        dl->AddLine(ImVec2(b0.x + bar, p), ImVec2(b0.x + bar + style.ItemInnerSpacing.x, p), text);
        const float y = std::clamp(p - 0.5f * ts.y, b0.y, b0.y + len - ts.y);
        dl->AddText(ImVec2(b0.x + bar + style.ItemInnerSpacing.x, y), text, t.c_str());
      }
    }
  }

  if (hr >= 0) {
    ev.hovered = true;
    ev.row = hr;
    ev.col = hc;
    if (clicked) {
      ev.clicked = true;
      ev.click_row = hr;
      ev.click_col = hc;
    }
  }
  ImGui::PopID();
  return ev;
#else
  return Backend::heatmap(label, spec);
#endif
}

} // namespace gl
} // namespace cvc

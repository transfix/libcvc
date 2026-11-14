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

// ImGuiBackend::chart — Kind::Chart on the ImGui draw list, with no GL of its own (desktop GL and
// WebGL2 alike). Top to bottom: the label, the legend (top, or a column at the right), and the
// canvas: y ticks (log decades on a log axis) and x ticks with the x label, then the bands, the
// lines (broken at each gap), the markers and the cursor. The canvas is ImGui's last item when the
// call returns, so a document's tooltip / on_click handlers attach to it. Hovering the canvas
// shows a crosshair and a readout of each shown series at its nearest point; with `pick:` a click
// reports the nearest x. Clicking a legend entry hides or shows its series; which are hidden is
// this backend's, per chart (keyed by the chart's ImGui ID). The core owns every cvc::state read
// and write.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <limits>
#include <string>
#include <utility>
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

// The categorical slots, in a fixed order (never cycled: a slot past them draws muted), each
// stepped for a light and a dark surface: blue, orange, aqua, yellow, magenta, green, violet, red.
constexpr unsigned kSlots = 8;
constexpr unsigned kLight[kSlots] = {0x2a78d6, 0xeb6834, 0x1baf7a, 0xeda100,
                                     0xe87ba4, 0x008300, 0x6250d6, 0xe34948};
constexpr unsigned kDark[kSlots] = {0x3987e5, 0xd95926, 0x199e70, 0xc98500,
                                    0xd55181, 0x008300, 0x9085e9, 0xe66767};

ImU32 with_alpha(ImU32 c, float a) {
  const unsigned alpha = static_cast<unsigned>(std::clamp(a, 0.0f, 1.0f) * 255.0f + 0.5f);
  return (c & ~IM_COL32_A_MASK) | (alpha << IM_COL32_A_SHIFT);
}

ImU32 series_color(const ariadne::ChartSeries &s, std::size_t i) {
  if (s.has_rgb)
    return ImGui::GetColorU32(ImVec4(s.rgb[0], s.rgb[1], s.rgb[2], 1.0f));
  const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
  const bool light = 0.2126f * bg.x + 0.7152f * bg.y + 0.0722f * bg.z > 0.5f;
  const std::size_t slot = s.color >= 0 ? static_cast<std::size_t>(s.color) : i;
  if (slot >= kSlots)
    return ImGui::GetColorU32(ImGuiCol_TextDisabled);
  const unsigned rgb = (light ? kLight : kDark)[slot];
  return IM_COL32((rgb >> 16) & 0xff, (rgb >> 8) & 0xff, rgb & 0xff, 0xff);
}

// A tick label: up to six significant digits, and no "-0".
std::string tick_text(double v) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.6g", v == 0.0 ? 0.0 : v);
  return buf;
}
// A decade's label: 10^e as a plain number near 1, else 1e<e>.
std::string decade_text(int e) {
  if (e >= -3 && e <= 4)
    return tick_text(std::pow(10.0, e));
  return "1e" + std::to_string(e);
}

double nice_step(double range, int ticks) {
  const double raw = range / std::max(1, ticks);
  if (!(raw > 0.0) || !std::isfinite(raw))
    return 1.0;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  const double f = raw / mag;
  return (f <= 1.0 ? 1.0 : f <= 2.0 ? 2.0 : f <= 5.0 ? 5.0 : 10.0) * mag;
}

struct Tick {
  double at;        // in axis units (the log on a log axis)
  std::string text; // "": a minor tick, a fainter rule with no label
};

// Ticks over [lo, hi], in axis units. A log axis spanning a decade or more ticks whole decades;
// a narrower one ticks nice values between, placed at their logs.
std::vector<Tick> axis_ticks(double lo, double hi, bool log, int max_ticks) {
  std::vector<Tick> out;
  if (!(hi > lo))
    return out;
  if (log && hi - lo >= 1.0) {
    const int step = std::max(1, static_cast<int>(std::ceil((hi - lo) / std::max(1, max_ticks))));
    for (int e = static_cast<int>(std::ceil(lo)); e <= static_cast<int>(std::floor(hi)); ++e)
      if (e % step == 0)
        out.push_back({static_cast<double>(e), decade_text(e)});
    // A few decades: the 2..9 between them too, as fainter rules with no label.
    if (step == 1 && hi - lo <= 4.0)
      for (int e = static_cast<int>(std::floor(lo)); e <= static_cast<int>(std::ceil(hi)); ++e)
        for (int m = 2; m <= 9; ++m) {
          const double at = e + std::log10(static_cast<double>(m));
          if (at >= lo && at <= hi)
            out.push_back({at, std::string()});
        }
    return out;
  }
  const double a = log ? std::pow(10.0, lo) : lo, z = log ? std::pow(10.0, hi) : hi;
  const double step = nice_step(z - a, max_ticks);
  for (double v = std::ceil(a / step) * step; v <= z + step * 1e-9; v += step) {
    const double snapped = std::fabs(v) < step * 1e-9 ? 0.0 : v;
    if (log && !(snapped > 0.0))
      continue;
    out.push_back({log ? std::log10(snapped) : snapped, tick_text(snapped)});
    if (out.size() > 64)
      break;
  }
  return out;
}

// A polyline in `line`'s pattern: dashes and dots are laid along its length, carried across its
// joints so a dense series still reads as dashed.
void stroke(ImDrawList *dl, const std::vector<ImVec2> &pts, ImU32 col, float thick,
            ariadne::ChartLine line) {
  if (pts.empty())
    return;
  if (line == ariadne::ChartLine::Points || pts.size() == 1) {
    const float r = line == ariadne::ChartLine::Points ? px(2.5f) : thick;
    for (const ImVec2 &p : pts)
      dl->AddCircleFilled(p, r, col, 8);
    return;
  }
  if (line == ariadne::ChartLine::Solid) {
    dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), col, thick, 0);
    return;
  }
  const float on = line == ariadne::ChartLine::Dashed ? px(6.0f) : px(1.5f);
  const float off = line == ariadne::ChartLine::Dashed ? px(4.0f) : px(3.0f);
  float phase = 0.0f; // how far into the current on+off period
  for (std::size_t k = 1; k < pts.size(); ++k) {
    const ImVec2 a = pts[k - 1], b = pts[k];
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    float t = 0.0f;
    while (t < len) {
      const bool drawing = phase < on;
      const float left = (drawing ? on : on + off) - phase;
      const float step = std::min(left, len - t);
      if (drawing) {
        const ImVec2 p0(a.x + dx * (t / len), a.y + dy * (t / len));
        const ImVec2 p1(a.x + dx * ((t + step) / len), a.y + dy * ((t + step) / len));
        dl->AddLine(p0, p1, col, thick);
      }
      t += step;
      phase += step;
      if (phase >= on + off)
        phase = 0.0f;
    }
  }
}

// A vertical rule in a marker's pattern.
void rule(ImDrawList *dl, float x, float y0, float y1, ImU32 col, float thick,
          ariadne::ChartLine line) {
  stroke(dl, {ImVec2(x, y0), ImVec2(x, y1)}, col, thick, line);
}

ariadne::ChartLine marker_line(ariadne::ChartMarkerKind k) {
  switch (k) {
  case ariadne::ChartMarkerKind::Best:
    return ariadne::ChartLine::Solid;
  case ariadne::ChartMarkerKind::Elbow:
    return ariadne::ChartLine::Dashed;
  case ariadne::ChartMarkerKind::Plateau:
  case ariadne::ChartMarkerKind::Note:
    break;
  }
  return ariadne::ChartLine::Dotted;
}

// A legend swatch: a short stroke of the series' line in its colour (dimmed while hidden).
void swatch(ImDrawList *dl, ImVec2 at, float w, float h, ImU32 col, ariadne::ChartLine line) {
  const float y = at.y + 0.5f * h;
  if (line == ariadne::ChartLine::Points)
    dl->AddCircleFilled(ImVec2(at.x + 0.5f * w, y), px(3.0f), col, 10);
  else
    stroke(dl, {ImVec2(at.x, y), ImVec2(at.x + w, y)}, col, px(2.0f), line);
}

std::string series_key(const ariadne::ChartSeries &s, std::size_t i) {
  return s.label.empty() ? "#" + std::to_string(i) : s.label;
}
std::string series_name(const ariadne::ChartSeries &s, std::size_t i) {
  return s.label.empty() ? "series " + std::to_string(i) : s.label;
}

// The index of the finite point of `s` nearest `x`, or -1.
long nearest(const ariadne::ChartSeries &s, double x) {
  long best = -1;
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < s.x.size() && k < s.y.size(); ++k) {
    if (!std::isfinite(s.x[k]) || !std::isfinite(s.y[k]))
      continue;
    const double e = std::fabs(s.x[k] - x);
    if (e < d) {
      d = e;
      best = static_cast<long>(k);
    }
  }
  return best;
}

} // namespace
#endif

ariadne::ChartEvent ImGuiBackend::chart(const char *label, const ariadne::ChartSpec &spec) {
#ifdef CVC_ENABLE_IMGUI
  ariadne::ChartEvent ev;
  const char *end = label ? ImGui::FindRenderedTextEnd(label) : label;
  ImGui::PushID(label ? label : "chart");
  std::unordered_set<std::string> &hidden = m_chartHidden[ImGui::GetID("##chart")];

  const std::size_t n = spec.series.size();
  std::vector<char> shown(n, 1);
  std::vector<ImU32> colors(n);
  for (std::size_t i = 0; i < n; ++i) {
    shown[i] = hidden.count(series_key(spec.series[i], i)) ? 0 : 1;
    colors[i] = series_color(spec.series[i], i);
  }

  const ImGuiStyle &style = ImGui::GetStyle();
  const float line_h = ImGui::GetTextLineHeight();
  const float sw_w = px(18.0f), gap = style.ItemInnerSpacing.x;
  const ImU32 text = ImGui::GetColorU32(ImGuiCol_Text);
  const ImU32 muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
  ImDrawList *dl = ImGui::GetWindowDrawList();

  if (label && end != label)
    ImGui::TextUnformatted(label, end);

  // One legend entry: a swatch and the label, a click hiding or showing the series.
  const bool with_legend = spec.legend != ariadne::ChartLegend::Off && n >= 2;
  const auto legend_entry = [&](std::size_t i) {
    const std::string name = series_name(spec.series[i], i);
    const ImVec2 size(sw_w + gap + ImGui::CalcTextSize(name.c_str()).x, line_h);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    ImGui::PushID(static_cast<int>(i));
    if (ImGui::InvisibleButton("##legend", size)) {
      const std::string key = series_key(spec.series[i], i);
      if (shown[i])
        hidden.insert(key);
      else
        hidden.erase(key);
      shown[i] = !shown[i];
    }
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip(shown[i] ? "Click to hide %s" : "Click to show %s", name.c_str());
    ImGui::PopID();
    swatch(dl, at, sw_w, line_h, shown[i] ? colors[i] : with_alpha(colors[i], 0.3f),
           spec.series[i].line);
    dl->AddText(ImVec2(at.x + sw_w + gap, at.y), shown[i] ? text : muted, name.c_str());
    return size;
  };

  if (with_legend && spec.legend == ariadne::ChartLegend::Top) {
    const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    for (std::size_t i = 0; i < n; ++i) {
      if (i > 0) {
        const float w = sw_w + gap + ImGui::CalcTextSize(series_name(spec.series[i], i).c_str()).x;
        ImGui::SameLine(0.0f, style.ItemSpacing.x * 2.0f);
        if (ImGui::GetCursorScreenPos().x + w > right)
          ImGui::NewLine();
      }
      legend_entry(i);
    }
  }

  // The canvas, and a legend column beside it.
  float legend_w = 0.0f;
  if (with_legend && spec.legend == ariadne::ChartLegend::Right)
    for (std::size_t i = 0; i < n; ++i)
      legend_w = std::max(
          legend_w, sw_w + gap + ImGui::CalcTextSize(series_name(spec.series[i], i).c_str()).x);
  const float avail = ImGui::GetContentRegionAvail().x;
  const float w =
      std::max(avail - (legend_w > 0.0f ? legend_w + style.ItemSpacing.x : 0.0f), px(120.0f));
  float h = std::max(px(spec.height), 4.0f * line_h);
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  if (legend_w > 0.0f) {
    for (std::size_t i = 0; i < n; ++i) {
      ImGui::SetCursorScreenPos(
          ImVec2(origin.x + w + style.ItemSpacing.x,
                 origin.y + static_cast<float>(i) * (line_h + style.ItemSpacing.y)));
      legend_entry(i);
    }
    h = std::max(h, static_cast<float>(n) * (line_h + style.ItemSpacing.y));
    ImGui::SetCursorScreenPos(origin);
  }
  ImGui::InvisibleButton("##chart", ImVec2(w, h));
  const bool hovered = ImGui::IsItemHovered();
  const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
  dl->AddRectFilled(origin, ImVec2(origin.x + w, origin.y + h),
                    ImGui::GetColorU32(ImGuiCol_FrameBg), style.FrameRounding);

  const ariadne::ChartBounds b = ariadne::chart_bounds(spec, shown);
  if (!b.ok) {
    const char *none = n == 0 ? "no series" : "no data";
    const ImVec2 ts = ImGui::CalcTextSize(none);
    dl->AddText(ImVec2(origin.x + 0.5f * (w - ts.x), origin.y + 0.5f * (h - ts.y)), muted, none);
    ImGui::PopID();
    return ev;
  }

  // The axes: data ranges (a flat one widened), y padded by 5%.
  double xlo = b.x0, xhi = b.x1, ylo = b.y0, yhi = b.y1;
  if (!(xhi > xlo))
    xlo -= 0.5, xhi += 0.5;
  if (!(yhi - ylo > 1e-12 * std::max(1.0, std::fabs(ylo))))
    ylo -= 0.5, yhi += 0.5;
  const double pad = 0.05 * (yhi - ylo);
  ylo -= pad;
  yhi += pad;

  const float top = px(4.0f), bottom = line_h + px(4.0f) + (spec.x_label.empty() ? 0.0f : line_h);
  const float plot_h = std::max(h - top - bottom, line_h);
  const std::vector<Tick> yt =
      axis_ticks(ylo, yhi, spec.log_y, std::max(2, static_cast<int>(plot_h / (2.2f * line_h))));
  float left = 0.0f;
  for (const Tick &t : yt)
    left = std::max(left, ImGui::CalcTextSize(t.text.c_str()).x);
  left += px(8.0f);
  const float right_pad = px(14.0f);
  const ImVec2 p0(origin.x + left, origin.y + top);
  const ImVec2 p1(origin.x + std::max(left + px(40.0f), w - right_pad), p0.y + plot_h);
  const auto X = [&](double x) {
    return static_cast<float>(p0.x + (x - xlo) / (xhi - xlo) * (p1.x - p0.x));
  };
  const auto Y = [&](double axis_v) {
    return static_cast<float>(p1.y - (axis_v - ylo) / (yhi - ylo) * (p1.y - p0.y));
  };
  const auto on_axis = [&](double v) { return spec.log_y ? std::log10(v) : v; };
  const std::vector<Tick> xt =
      axis_ticks(xlo, xhi, false, std::max(2, static_cast<int>((p1.x - p0.x) / px(70.0f))));

  // The grid and the tick labels, recessive.
  const ImU32 grid = with_alpha(ImGui::GetColorU32(ImGuiCol_Border), 0.45f);
  const ImU32 minor = with_alpha(ImGui::GetColorU32(ImGuiCol_Border), 0.2f);
  for (const Tick &t : yt) {
    const float y = Y(t.at);
    dl->AddLine(ImVec2(p0.x, y), ImVec2(p1.x, y), t.text.empty() ? minor : grid);
    if (t.text.empty())
      continue;
    const ImVec2 ts = ImGui::CalcTextSize(t.text.c_str());
    dl->AddText(ImVec2(p0.x - px(4.0f) - ts.x, y - 0.5f * ts.y), muted, t.text.c_str());
  }
  for (const Tick &t : xt) {
    const float x = X(t.at);
    dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), grid);
    const ImVec2 ts = ImGui::CalcTextSize(t.text.c_str());
    dl->AddText(ImVec2(x - 0.5f * ts.x, p1.y + px(2.0f)), muted, t.text.c_str());
  }
  dl->AddRect(p0, p1, grid);
  if (!spec.x_label.empty()) {
    const ImVec2 ts = ImGui::CalcTextSize(spec.x_label.c_str());
    dl->AddText(ImVec2(0.5f * (p0.x + p1.x - ts.x), p1.y + px(2.0f) + line_h), muted,
                spec.x_label.c_str());
  }

  dl->PushClipRect(p0, p1, true);
  // Bands under the lines: a quad between each pair of neighbouring points that both have one.
  for (std::size_t i = 0; i < n; ++i) {
    const ariadne::ChartSeries &s = spec.series[i];
    if (!shown[i] || s.lo.size() != s.y.size() || s.hi.size() != s.y.size())
      continue;
    const ImU32 fill = with_alpha(colors[i], 0.3f);
    const auto edge = [&](std::size_t k, float &ya, float &yb) {
      if (k >= s.x.size() || !std::isfinite(s.x[k]) || !std::isfinite(s.hi[k]))
        return false;
      const bool floor = spec.log_y ? !(s.lo[k] > 0.0) : false;
      if (!floor && !std::isfinite(s.lo[k]))
        return false;
      ya = floor ? p1.y : Y(on_axis(s.lo[k]));
      yb = Y(on_axis(s.hi[k]));
      return true;
    };
    for (std::size_t k = 1; k < s.y.size(); ++k) {
      float a0 = 0, a1 = 0, b0 = 0, b1 = 0;
      if (!edge(k - 1, a0, a1) || !edge(k, b0, b1))
        continue;
      const float xa = X(s.x[k - 1]), xb = X(s.x[k]);
      dl->AddQuadFilled(ImVec2(xa, a1), ImVec2(xb, b1), ImVec2(xb, b0), ImVec2(xa, a0), fill);
    }
  }
  // The lines, each run of finite points one polyline: a gap breaks it.
  const float thick = px(2.0f);
  for (std::size_t i = 0; i < n; ++i) {
    const ariadne::ChartSeries &s = spec.series[i];
    if (!shown[i])
      continue;
    std::vector<ImVec2> run;
    for (std::size_t k = 0; k <= s.y.size() && k <= s.x.size(); ++k) {
      const bool ok =
          k < s.y.size() && k < s.x.size() && std::isfinite(s.x[k]) && std::isfinite(s.y[k]);
      if (ok) {
        run.emplace_back(X(s.x[k]), Y(on_axis(s.y[k])));
        continue;
      }
      stroke(dl, run, colors[i], thick, s.line);
      run.clear();
    }
  }
  // The markers: a rule in the text colour, its pattern saying its kind, and its label at the top.
  for (std::size_t j = 0; j < spec.markers.size(); ++j) {
    const ariadne::ChartMarker &m = spec.markers[j];
    if (m.x < xlo || m.x > xhi)
      continue;
    const float x = X(m.x);
    const bool best = m.kind == ariadne::ChartMarkerKind::Best;
    rule(dl, x, p0.y, p1.y, best ? text : with_alpha(text, 0.7f), best ? px(1.5f) : px(1.0f),
         marker_line(m.kind));
    const std::string tag = m.label.empty() ? ariadne::chart_marker_kind_name(m.kind) : m.label;
    const ImVec2 ts = ImGui::CalcTextSize(tag.c_str());
    const float tx = x + ts.x + px(4.0f) > p1.x ? x - ts.x - px(3.0f) : x + px(3.0f);
    dl->AddText(ImVec2(tx, p0.y + px(2.0f) + static_cast<float>(j % 2) * line_h),
                best ? text : muted, tag.c_str());
  }
  // The play-head: a line with a handle at the top, so it is not taken for a marker.
  if (spec.has_cursor && spec.cursor >= xlo && spec.cursor <= xhi) {
    const float x = X(spec.cursor), r = px(5.0f);
    dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), text, px(2.0f));
    dl->AddTriangleFilled(ImVec2(x - r, p0.y), ImVec2(x + r, p0.y), ImVec2(x, p0.y + 1.5f * r),
                          text);
  }

  // Hover: a crosshair and each shown series at its point nearest the pointer.
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  const bool inside = mouse.x >= p0.x && mouse.x <= p1.x && mouse.y >= p0.y && mouse.y <= p1.y;
  const double xm = xlo + (mouse.x - p0.x) / (p1.x - p0.x) * (xhi - xlo);
  if (hovered && inside) {
    dl->AddLine(ImVec2(mouse.x, p0.y), ImVec2(mouse.x, p1.y), with_alpha(text, 0.35f));
    std::vector<std::pair<std::size_t, long>> at;
    for (std::size_t i = 0; i < n; ++i) {
      if (!shown[i])
        continue;
      const long k = nearest(spec.series[i], xm);
      if (k < 0)
        continue;
      at.emplace_back(i, k);
      const ariadne::ChartSeries &s = spec.series[i];
      const ImVec2 pt(X(s.x[k]), Y(on_axis(s.y[k])));
      dl->AddCircleFilled(pt, px(4.0f), ImGui::GetColorU32(ImGuiCol_FrameBg), 12);
      dl->AddCircleFilled(pt, px(3.0f), colors[i], 12);
    }
    dl->PopClipRect();
    ImGui::BeginTooltip();
    ImGui::Text("%s %s", spec.x_label.empty() ? "x" : spec.x_label.c_str(), tick_text(xm).c_str());
    for (const auto &[i, k] : at) {
      const ariadne::ChartSeries &s = spec.series[i];
      const ImVec2 cur = ImGui::GetCursorScreenPos();
      ImGui::Dummy(ImVec2(sw_w, line_h));
      swatch(ImGui::GetWindowDrawList(), cur, sw_w, line_h, colors[i], s.line);
      ImGui::SameLine();
      std::string row = series_name(s, i) + ": " + tick_text(s.y[k]);
      if (static_cast<std::size_t>(k) < s.lo.size() && static_cast<std::size_t>(k) < s.hi.size() &&
          std::isfinite(s.hi[k]))
        row += "  [" + tick_text(s.lo[k]) + ", " + tick_text(s.hi[k]) + "]";
      if (s.x[k] != xm)
        row += "  at " + tick_text(s.x[k]);
      ImGui::TextUnformatted(row.c_str());
    }
    for (const ariadne::ChartMarker &m : spec.markers)
      if (std::fabs(X(m.x) - mouse.x) <= px(6.0f))
        ImGui::TextDisabled("%s %s at %s", ariadne::chart_marker_kind_name(m.kind), m.label.c_str(),
                            tick_text(m.x).c_str());
    ImGui::EndTooltip();
  } else {
    dl->PopClipRect();
  }

  // A pick: the x of the nearest point of a shown series.
  if (spec.pick && clicked && inside) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      if (!shown[i])
        continue;
      const long k = nearest(spec.series[i], xm);
      if (k >= 0 && std::fabs(spec.series[i].x[k] - xm) < best) {
        best = std::fabs(spec.series[i].x[k] - xm);
        ev.picked = true;
        ev.x = spec.series[i].x[k];
      }
    }
  }
  ImGui::PopID();
  return ev;
#else
  return Backend::chart(label, spec);
#endif
}

} // namespace gl
} // namespace cvc

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

// ImGuiBackend::tf_editor — the native transfer-function editor every GL Ariadne host draws, a port
// of VolumeRover2's ColorTable2 onto the ImGui draw list. It uses no GL of its own, so the same
// code runs on desktop GL and WebGL2. Geometry, hit testing, the drag gesture and every edit come
// from the GL-free cvc::tf editor (cvc/tf/editor.h) shared with the composed fallback and the
// tests; this file lays the widget out, routes ImGui input into that gesture, draws, and reports
// the edit. The Ariadne core owns every cvc::state read and write.
//
// Top to bottom: the label, the 20 px zoom strip (the whole axis and the two range bars), the
// canvas (the zoom window), a data-unit axis line, the spectrum legend, the overlay status and the
// per-isocontour information table. The canvas is left as ImGui's last item when the call returns,
// so a document's tooltip / on_click / on_drag handlers attach to it rather than to the table.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/tf/editor.h>
#include <cvc/tf/model.h>
#include <cvc/tf/overlays.h>
#include <cvc/tf/vinay.h>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal asserts it)
#include <imgui.h>
#include <imgui_internal.h> // FindRenderedTextEnd; GImGui->LastItemData (the canvas as last item)
#endif

namespace cvc {
namespace gl {

namespace tf_editor_detail {

// What one editor instance keeps between frames. A namespace-scope struct (the session type nested
// in ImGuiBackend is private) so this file's helpers can take it.
struct session {
  int drawn_frame = -1; // the ImGui frame it last drew in (tells a second same-key instance apart)
  tf::gesture gesture;
  tf::handle selected; // the node Delete and the context menu act on: the last one clicked
  // The bound model as this session last saw or wrote (commit) it: what the handles it holds across
  // frames (selected, menu_handle, color_target) index into.
  tf::model expected;
  tf::model work;       // a delayed-update drag's in-flight model (interactive_updates: false)
  bool working = false; // `work` is live
  // Where the context menu was opened and what was under it, so its actions apply there and not
  // wherever the pointer is when an item is chosen.
  double menu_t = 0.0;
  float menu_a = 0.f;
  tf::handle menu_handle;
  // The handle under the pointer and since when: a hover tooltip waits for the pointer to settle,
  // so sweeping across the canvas does not flash one per bar it crosses.
  tf::handle hot;
  double hot_since = 0.0;
  // The colour popup: its target (a colour node by index, an isocontour by id) and the edit.
  bool open_color = false;
  tf::handle color_target;
  float color_rgba[4] = {1.f, 1.f, 1.f, 1.f};
  float color_ref[4] = {1.f, 1.f, 1.f, 1.f};
  // The .vinay import / export popup.
  bool open_file = false;
  bool file_export = false;
  char file_path[1024] = {};
  std::string file_error;
};

} // namespace tf_editor_detail

struct ImGuiBackend::TfEditorSession : tf_editor_detail::session {};

#ifdef CVC_ENABLE_IMGUI
namespace {

using tf_editor_detail::session;

constexpr float kStripHeight = 20.f;    // ColorTable2's zoom strip
constexpr float kDefaultHeight = 160.f; // the canvas when the document sets no height
constexpr float kMinWidth = 64.f;

ImU32 col(float r, float g, float b, float a = 1.f) {
  return ImGui::GetColorU32(ImVec4(r, g, b, a)); // honours style.Alpha (a disabled editor greys)
}
ImU32 col(const float rgb[3], float a = 1.f) { return col(rgb[0], rgb[1], rgb[2], a); }

double clamp01(double t) { return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); }

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}
std::string num_or_dash(double v) { return std::isfinite(v) ? num(v) : std::string("-"); }

// ImDrawList's polyline and outline-rect calls swapped their thickness and flags arguments in
// 1.92.8; the old order still compiles there (deprecated) but not under
// IMGUI_DISABLE_OBSOLETE_FUNCTIONS, and the new order means something else before it.
void polyline(ImDrawList *dl, const std::vector<ImVec2> &pts, ImU32 c, float thickness) {
  if (pts.size() < 2)
    return;
#if IMGUI_VERSION_NUM >= 19280
  dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), c, thickness, ImDrawFlags_None);
#else
  dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), c, ImDrawFlags_None, thickness);
#endif
}
void outline(ImDrawList *dl, const ImVec2 &a, const ImVec2 &b, ImU32 c, float thickness) {
#if IMGUI_VERSION_NUM >= 19280
  dl->AddRect(a, b, c, 0.0f, thickness, ImDrawFlags_None);
#else
  dl->AddRect(a, b, c, 0.0f, ImDrawFlags_None, thickness);
#endif
}

bool is_node(tf::part k) {
  return k == tf::part::color || k == tf::part::alpha || k == tf::part::isocontour;
}

// Re-anchor a handle on `m`: an isocontour by id (the list re-sorts as one moves), a colour or
// alpha node by index. Invalid once the node is gone (deleted, or replaced from outside).
tf::handle resolve(const tf::model &m, tf::handle h) {
  switch (h.kind) {
  case tf::part::color:
    return (h.index >= 0 && h.index < static_cast<int>(m.color.size())) ? h : tf::handle{};
  case tf::part::alpha:
    return (h.index >= 0 && h.index < static_cast<int>(m.alpha.size())) ? h : tf::handle{};
  case tf::part::isocontour: {
    if (h.id.empty() && h.index >= 0 && h.index < static_cast<int>(m.isocontours.size()))
      h.id = m.isocontours[static_cast<std::size_t>(h.index)].id;
    h.index = m.find_isocontour(h.id);
    return h.index >= 0 ? h : tf::handle{};
  }
  case tf::part::zoom_lo:
  case tf::part::zoom_hi:
    return h;
  default:
    return {};
  }
}

// The index in `to` of the node at `index` in `from`, found by position (unique in a normalized
// list, and at least kMinNodeGap apart where the editor made them), or -1 when it is gone. The
// string form keeps positions to 9 significant digits, so a model that went through it (a peer's
// or a script's read-modify-write) may have moved them by ~1e-9; the nearest within half the gap
// is the same node.
template <typename List> int same_node(const List &from, const List &to, int index) {
  if (index < 0 || index >= static_cast<int>(from.size()))
    return -1;
  const double at = from[static_cast<std::size_t>(index)].at;
  int best = -1;
  double best_d = 0.5 * tf::kMinNodeGap;
  for (std::size_t i = 0; i < to.size(); ++i) {
    const double d = std::fabs(to[i].at - at);
    if (d <= best_d && (best < 0 || d < best_d)) {
      best = static_cast<int>(i);
      best_d = d;
    }
  }
  return best;
}

// Carry a handle taken on `from` over to `to`, which replaced it (an outside write) or inserted or
// removed nodes (an edit): an isocontour by id, a colour or alpha node by position, so a node
// inserted or removed ahead of it does not hand its index to a neighbour. resolve() alone only
// bounds-checks the index. Invalid once the node is gone (deleted, or moved by the other writer).
tf::handle reanchor(const tf::model &from, const tf::model &to, tf::handle h) {
  switch (h.kind) {
  case tf::part::color:
    h.index = same_node(from.color, to.color, h.index);
    return h.index >= 0 ? h : tf::handle{};
  case tf::part::alpha:
    h.index = same_node(from.alpha, to.alpha, h.index);
    return h.index >= 0 ? h : tf::handle{};
  case tf::part::isocontour:
    if (h.id.empty())
      h = resolve(from, h); // pin the id on the model the index was taken on
    return resolve(to, h);
  default:
    return resolve(to, h);
  }
}

// What tf::remove would accept (colour / alpha endpoints are pinned), for greying the menu item.
bool deletable(const tf::model &m, const tf::handle &h) {
  switch (h.kind) {
  case tf::part::color:
    return h.index > 0 && h.index + 1 < static_cast<int>(m.color.size());
  case tf::part::alpha:
    return h.index > 0 && h.index + 1 < static_cast<int>(m.alpha.size());
  case tf::part::isocontour:
    return h.index >= 0;
  default:
    return false;
  }
}

// A preset, Reset or an imported .vinay replaces the look (colour, opacity, isocontours) but keeps
// the domain and resolution, which describe the data and its consumers (the widget offers no
// control for them), and the cubing flag, which ColorTable2 kept outside the table too.
void replace_look(tf::model &m, const tf::model &look) {
  m.color = look.color;
  m.alpha = look.alpha;
  m.isocontours = look.isocontours;
}

// The meshed interval's two bounding isocontours (TfEditorInput's interval_a / interval_b) as
// indices into the model being drawn, the lower bar first; invalid when either is not in it.
struct Interval {
  int lo = -1, hi = -1;
  bool valid() const { return lo >= 0 && hi >= 0; }
  // "lower" / "upper" when isocontour `i` bounds the interval, else nullptr.
  const char *bound(int i) const {
    return !valid() ? nullptr : (i == lo ? "lower" : (i == hi ? "upper" : nullptr));
  }
};

Interval find_interval(const tf::model &m, const std::string &a, const std::string &b) {
  Interval r;
  const int ia = a.empty() ? -1 : m.find_isocontour(a);
  const int ib = b.empty() ? -1 : m.find_isocontour(b);
  if (ia < 0 || ib < 0 || ia == ib)
    return r;
  const bool a_low = m.isocontours[static_cast<std::size_t>(ia)].at <=
                     m.isocontours[static_cast<std::size_t>(ib)].at;
  r.lo = a_low ? ia : ib;
  r.hi = a_low ? ib : ia;
  return r;
}

// The input state of the item just submitted.
struct Item {
  ImVec2 min, max;
  bool hovered = false, active = false, activated = false;
  static Item last() {
    Item i;
    i.min = ImGui::GetItemRectMin();
    i.max = ImGui::GetItemRectMax();
    i.hovered = ImGui::IsItemHovered();
    i.active = ImGui::IsItemActive();
    i.activated = ImGui::IsItemActivated();
    return i;
  }
};

// ---- drawing --------------------------------------------------------------------------------

// The colour ramp over normalized [t0, t1] into [left, right] x [top, bottom]: one two-colour
// gradient quad per colour segment, exactly the piecewise-linear RGB the baked table holds (a hard
// edge — two nodes at one position — stays hard). `x_of_t` maps t into the target's x.
template <typename X>
void draw_ramp(ImDrawList *dl, const tf::model &m, double t0, double t1, float left, float right,
               float top, float bottom, X x_of_t) {
  if (m.color.size() < 2) { // never after normalize(); ColorTable2 drew a grey ramp
    const float g0 = static_cast<float>(t0), g1 = static_cast<float>(t1);
    dl->AddRectFilledMultiColor(ImVec2(left, top), ImVec2(right, bottom), col(g0, g0, g0),
                                col(g1, g1, g1), col(g1, g1, g1), col(g0, g0, g0));
    return;
  }
  for (std::size_t i = 0; i + 1 < m.color.size(); ++i) {
    const tf::color_node &p = m.color[i], &q = m.color[i + 1];
    const double a = std::max(p.at, t0), b = std::min(q.at, t1);
    if (b <= a)
      continue;
    const double span = q.at - p.at;
    auto lerp = [&](double t, float out[3]) {
      const float u = span > 0.0 ? static_cast<float>((t - p.at) / span) : 0.f;
      out[0] = p.r + (q.r - p.r) * u;
      out[1] = p.g + (q.g - p.g) * u;
      out[2] = p.b + (q.b - p.b) * u;
    };
    float ca[3], cb[3];
    lerp(a, ca);
    lerp(b, cb);
    // Run the window's first and last segments out to the edges, so no sub-pixel seam shows there.
    const float xa = a <= t0 ? left : x_of_t(a);
    const float xb = b >= t1 ? right : x_of_t(b);
    const ImU32 ua = col(ca), ub = col(cb);
    dl->AddRectFilledMultiColor(ImVec2(xa, top), ImVec2(xb, bottom), ua, ub, ub, ua);
  }
}

// The histogram under the zoom window. Bins are uniform over the data range [rlo, rhi], which an
// explicit domain need not match, so each bin is placed through the domain mapping. Heights follow
// tf::render_preview: ColorTable2's linear scale over [smallest bin, top], the top clipped to 1.5x
// the second-largest bin (a background spike would flatten everything else) unless that clips a
// lone bin away, or log(1 + count). Bins are folded into pixel columns (max), so a fine histogram
// costs at most one quad per column.
void draw_histogram(ImDrawList *dl, const tf::layout &l, const tf::view &v,
                    const std::vector<double> &bins, double rlo, double rhi, double lo, double hi) {
  const std::size_t n = bins.size();
  if (n == 0 || l.canvas_w < 1.f)
    return;
  double top = 0.0, second = 0.0, low = HUGE_VAL;
  for (double b : bins) {
    if (!std::isfinite(b))
      continue;
    if (b > top) {
      second = top;
      top = b;
    } else if (b > second) {
      second = b;
    }
    low = std::min(low, b);
  }
  if (!std::isfinite(low))
    return;
  std::vector<float> frac(n, 0.f);
  if (v.log_histogram) {
    const double d = std::log1p(std::max(top, 0.0));
    if (d <= 0.0)
      return;
    for (std::size_t i = 0; i < n; ++i)
      if (std::isfinite(bins[i]))
        frac[i] = static_cast<float>(std::log1p(std::max(bins[i], 0.0)) / d);
  } else {
    if (n >= 2 && top > 1.5 * second && 1.5 * second > low)
      top = 1.5 * second;
    if (top <= low)
      return;
    for (std::size_t i = 0; i < n; ++i)
      if (std::isfinite(bins[i]))
        frac[i] = static_cast<float>(clamp01((bins[i] - low) / (top - low)));
  }
  const int w = std::max(1, static_cast<int>(std::ceil(l.canvas_w)));
  std::vector<float> column(static_cast<std::size_t>(w), 0.f);
  const double step = (rhi - rlo) / static_cast<double>(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (frac[i] <= 0.f)
      continue;
    const double t0 = tf::to_normalized(rlo + step * static_cast<double>(i), lo, hi);
    const double t1 = tf::to_normalized(rlo + step * static_cast<double>(i + 1), lo, hi);
    if (std::max(t0, t1) < v.zoom_lo || std::min(t0, t1) > v.zoom_hi)
      continue;
    // Clamped before the int conversion: a bin far outside a narrow explicit domain maps far off.
    const float x0 = std::clamp(tf::x_of(l, v, t0) - l.canvas_x, -1.f, l.canvas_w + 1.f);
    const float x1 = std::clamp(tf::x_of(l, v, t1) - l.canvas_x, -1.f, l.canvas_w + 1.f);
    int c0 = static_cast<int>(std::floor(std::min(x0, x1)));
    int c1 = static_cast<int>(std::ceil(std::max(x0, x1))) - 1;
    c1 = std::max(c1, c0);
    c0 = std::max(c0, 0);
    c1 = std::min(c1, w - 1);
    for (int c = c0; c <= c1; ++c)
      column[static_cast<std::size_t>(c)] = std::max(column[static_cast<std::size_t>(c)], frac[i]);
  }
  // Over the colour ramp ColorTable2 shaded translucently; over the flat grey it drew solid black.
  const ImU32 ink = v.show_color ? col(0.25f, 0.25f, 0.25f, 0.4f) : col(0.f, 0.f, 0.f);
  const float bottom = l.canvas_y + l.canvas_h;
  for (int c = 0; c < w;) {
    const float f = column[static_cast<std::size_t>(c)];
    int e = c;
    while (e + 1 < w && column[static_cast<std::size_t>(e + 1)] == f)
      ++e;
    if (f > 0.f)
      dl->AddRectFilled(ImVec2(l.canvas_x + static_cast<float>(c), tf::y_of(l, f)),
                        ImVec2(l.canvas_x + static_cast<float>(e + 1), bottom), ink);
    c = e + 1;
  }
}

// The contour tree: each arc from (value, layout x) to (value, layout x), the value along the TF
// axis and the layout down the canvas inside ColorTable2's 10% inset. Black over a light halo so it
// reads over both ends of a ramp; every halo first, so no halo paints over an earlier arc.
void draw_tree(ImDrawList *dl, const tf::layout &l, const tf::view &v, const tf::tree_overlay &tree,
               double lo, double hi) {
  const int nv = static_cast<int>(tree.vertices.size());
  auto y_of_layout = [&](float x) {
    return l.canvas_y + (1.f - x) * (l.canvas_h - 1.f) * 0.8f + l.canvas_h * 0.1f;
  };
  std::vector<std::pair<ImVec2, ImVec2>> arcs;
  arcs.reserve(tree.edges.size());
  for (const auto &e : tree.edges) {
    if (e.first < 0 || e.first >= nv || e.second < 0 || e.second >= nv)
      continue;
    const tf::tree_overlay::vertex &a = tree.vertices[static_cast<std::size_t>(e.first)];
    const tf::tree_overlay::vertex &b = tree.vertices[static_cast<std::size_t>(e.second)];
    if (!std::isfinite(a.value) || !std::isfinite(b.value))
      continue;
    const double ta = tf::to_normalized(a.value, lo, hi), tb = tf::to_normalized(b.value, lo, hi);
    if (std::max(ta, tb) < v.zoom_lo || std::min(ta, tb) > v.zoom_hi)
      continue;
    arcs.emplace_back(ImVec2(tf::x_of(l, v, ta), y_of_layout(a.x)),
                      ImVec2(tf::x_of(l, v, tb), y_of_layout(b.x)));
  }
  const ImU32 halo = col(1.f, 1.f, 1.f, 0.45f), ink = col(0.f, 0.f, 0.f);
  for (const auto &a : arcs)
    dl->AddLine(a.first, a.second, halo, 3.f);
  for (const auto &a : arcs)
    dl->AddLine(a.first, a.second, ink, 1.f);
}

// The contour spectrum's four signature functions, each normalized to the canvas height on its own
// (they have unrelated units), sampled uniformly over the data range. A non-finite sample breaks
// the curve rather than dragging it to an edge; a constant function carries nothing and is skipped
// (ColorTable2 divided by zero), as in tf::render_preview.
void draw_spectrum(ImDrawList *dl, const tf::layout &l, const tf::view &v,
                   const tf::spectrum_overlay &sp, double rlo, double rhi, double lo, double hi) {
  struct Curve {
    const std::vector<double> *f;
    float r, g, b;
  };
  const Curve curves[] = {{&sp.area, 1.f, 0.f, 0.f},
                          {&sp.min_volume, 0.f, 1.f, 0.f},
                          {&sp.max_volume, 0.f, 0.f, 1.f},
                          {&sp.gradient, 1.f, 1.f, 0.f}};
  std::vector<ImVec2> pts;
  for (const Curve &c : curves) {
    const std::vector<double> &f = *c.f;
    const std::size_t n = f.size();
    if (n < 2)
      continue;
    double fmin = HUGE_VAL, fmax = -HUGE_VAL;
    for (double s : f)
      if (std::isfinite(s)) {
        fmin = std::min(fmin, s);
        fmax = std::max(fmax, s);
      }
    const double span = fmax - fmin;
    if (!(span > 0.0))
      continue;
    const ImU32 ink = col(c.r, c.g, c.b);
    pts.clear();
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(f[i])) {
        polyline(dl, pts, ink, 1.5f);
        pts.clear();
        continue;
      }
      const double value = rlo + (rhi - rlo) * static_cast<double>(i) / static_cast<double>(n - 1);
      const double t = tf::to_normalized(value, lo, hi);
      pts.emplace_back(tf::x_of(l, v, t), tf::y_of(l, static_cast<float>((f[i] - fmin) / span)));
    }
    polyline(dl, pts, ink, 1.5f);
  }
}

// A ColorTable2 bar: a 2 px full-height line and a 10x10 handle on the bottom edge, outlined.
void draw_bar(ImDrawList *dl, float x, float top, float bottom, ImU32 line, ImU32 fill, ImU32 edge,
              bool hot, bool selected) {
  dl->AddRectFilled(ImVec2(x - 1.f, top), ImVec2(x + 1.f, bottom), hot ? col(1.f, 1.f, 1.f) : line);
  const float h = tf::kHandleHalf;
  const ImVec2 a(x - h, bottom - 1.f - 2.f * h), b(x + h, bottom - 1.f);
  dl->AddRectFilled(a, b, fill);
  dl->AddRect(a, b, edge);
  if (selected)
    outline(dl, a - ImVec2(2.f, 2.f), b + ImVec2(2.f, 2.f), col(1.f, 1.f, 1.f), 2.f);
  else if (hot)
    dl->AddRect(a - ImVec2(1.f, 1.f), b + ImVec2(1.f, 1.f), col(1.f, 1.f, 1.f));
}

bool same(const tf::handle &a, tf::part kind, int index) {
  return a.kind == kind && a.index == index;
}

// The opacity curve and its nodes. With opacity_cubed the table holds a^3, which is not
// piecewise-linear in the nodes, so the curve is sampled; the nodes (what a drag moves) still sit
// at their control values, joined by a faint control polygon.
void draw_opacity(ImDrawList *dl, const tf::layout &l, const tf::view &v, const tf::model &m,
                  const tf::handle &hot, const tf::handle &selected) {
  std::vector<ImVec2> ctrl;
  ctrl.reserve(m.alpha.size());
  for (const tf::alpha_node &n : m.alpha)
    ctrl.emplace_back(tf::x_of(l, v, n.at), tf::y_of(l, n.a));
  const ImU32 halo = col(1.f, 1.f, 1.f, 0.45f), ink = col(0.f, 0.f, 0.f);
  if (m.opacity_cubed) {
    polyline(dl, ctrl, col(0.f, 0.f, 0.f, 0.35f), 1.f);
    const int n = std::max(2, static_cast<int>(l.canvas_w / 2.f) + 1);
    std::vector<ImVec2> curve;
    curve.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      const float x = l.canvas_x + l.canvas_w * static_cast<float>(i) / static_cast<float>(n - 1);
      curve.emplace_back(x, tf::y_of(l, m.alpha_at(tf::t_of(l, v, x))));
    }
    polyline(dl, curve, halo, 3.5f);
    polyline(dl, curve, ink, 1.5f);
  } else {
    polyline(dl, ctrl, halo, 3.5f);
    polyline(dl, ctrl, ink, 1.5f);
  }
  const ImVec2 h(tf::kHandleHalf, tf::kHandleHalf);
  for (std::size_t i = 0; i < ctrl.size(); ++i) {
    const int idx = static_cast<int>(i);
    dl->AddRectFilled(ctrl[i] - h, ctrl[i] + h, col(0.f, 0.f, 1.f));
    if (same(selected, tf::part::alpha, idx))
      outline(dl, ctrl[i] - h - ImVec2(1.f, 1.f), ctrl[i] + h + ImVec2(1.f, 1.f),
              col(1.f, 1.f, 1.f), 2.f);
    else if (same(hot, tf::part::alpha, idx))
      dl->AddRect(ctrl[i] - h, ctrl[i] + h, col(1.f, 1.f, 0.f));
  }
}

// The zoom strip: the whole axis's ramp, the part the canvas does not show dimmed, and the two grey
// range bars that set the window.
void draw_strip(ImDrawList *dl, const tf::layout &l, const tf::view &v, const tf::model &m,
                const tf::handle &hot) {
  const ImVec2 a(l.strip_x, l.strip_y), b(l.strip_x + l.strip_w, l.strip_y + l.strip_h);
  dl->PushClipRect(a, b, true);
  draw_ramp(dl, m, 0.0, 1.0, a.x, b.x, a.y, b.y, [&](double t) { return tf::strip_x_of(l, t); });
  const float xl = tf::strip_x_of(l, v.zoom_lo), xr = tf::strip_x_of(l, v.zoom_hi);
  const ImU32 dim = col(0.f, 0.f, 0.f, 0.45f);
  if (xl > a.x)
    dl->AddRectFilled(a, ImVec2(xl, b.y), dim);
  if (xr < b.x)
    dl->AddRectFilled(ImVec2(xr, a.y), b, dim);
  dl->PopClipRect();
  // The bars may overhang the strip by half a handle at the axis ends.
  const ImVec2 pad(tf::kHandleHalf + 1.f, 0.f);
  dl->PushClipRect(a - pad, b + pad, true);
  const ImU32 grey = col(0.5f, 0.5f, 0.5f), dark = col(0.15f, 0.15f, 0.15f);
  draw_bar(dl, xl, a.y, b.y, grey, grey, dark, hot.kind == tf::part::zoom_lo, false);
  draw_bar(dl, xr, a.y, b.y, grey, grey, dark, hot.kind == tf::part::zoom_hi, false);
  dl->PopClipRect();
  dl->AddRect(a, b, ImGui::GetColorU32(ImGuiCol_Border));
}

// The canvas, far to near in ColorTable2's depth order (which is also the pick order, nearest
// first: opacity nodes, isocontour bars, colour bars), so what the pointer grabs is what is on top.
void draw_canvas(ImDrawList *dl, const tf::layout &l, const tf::view &v, const tf::model &m,
                 const tf::overlays &ov, double lo, double hi, double rlo, double rhi,
                 const tf::handle &hot, const tf::handle &selected, const Interval &ivl) {
  const ImVec2 a(l.canvas_x, l.canvas_y), b(l.canvas_x + l.canvas_w, l.canvas_y + l.canvas_h);
  dl->PushClipRect(a, b, true);
  if (v.show_color)
    draw_ramp(dl, m, v.zoom_lo, v.zoom_hi, a.x, b.x, a.y, b.y,
              [&](double t) { return tf::x_of(l, v, t); });
  else
    dl->AddRectFilled(a, b, col(0.5f, 0.5f, 0.5f));
  if (v.show_histogram && !ov.histogram.empty())
    draw_histogram(dl, l, v, ov.histogram, rlo, rhi, lo, hi);
  // The meshed interval: the band between its two bars, a translucent magenta (the bars' colour)
  // over the ramp and histogram, under the curves; through the zoom window, hidden with the bars.
  if (v.show_isocontours && ivl.valid()) {
    // Clamped to just past the canvas: a bar far outside a narrow zoom window maps far off.
    const auto x = [&](int i) {
      return std::clamp(tf::x_of(l, v, m.isocontours[static_cast<std::size_t>(i)].at), a.x - 1.f,
                        b.x + 1.f);
    };
    const float x0 = x(ivl.lo), x1 = x(ivl.hi);
    if (x1 > x0)
      dl->AddRectFilled(ImVec2(x0, a.y), ImVec2(x1, b.y), col(1.f, 0.f, 1.f, tf::kIntervalAlpha));
  }
  if (v.show_contour_tree && !ov.tree.empty())
    draw_tree(dl, l, v, ov.tree, lo, hi);
  if (v.show_contour_spectrum && !ov.spectrum.empty())
    draw_spectrum(dl, l, v, ov.spectrum, rlo, rhi, lo, hi);
  dl->PopClipRect();

  // Handles may overhang the canvas by half their size (the endpoints sit on its edges), but not
  // onto the zoom strip above or the axis line below.
  const ImVec2 pad(tf::kHandleHalf + 1.f,
                   std::min(tf::kHandleHalf + 1.f, ImGui::GetStyle().ItemSpacing.y));
  dl->PushClipRect(a - pad, b + pad, true);
  if (v.show_color) {
    const ImU32 cyan = col(0.f, 1.f, 1.f), red = col(1.f, 0.f, 0.f);
    for (std::size_t i = 0; i < m.color.size(); ++i) {
      const int idx = static_cast<int>(i);
      draw_bar(dl, tf::x_of(l, v, m.color[i].at), a.y, b.y, cyan, red, cyan,
               same(hot, tf::part::color, idx), same(selected, tf::part::color, idx));
    }
  }
  if (v.show_isocontours) {
    const ImU32 magenta = col(1.f, 0.f, 1.f), green = col(0.f, 1.f, 0.f);
    for (std::size_t i = 0; i < m.isocontours.size(); ++i) {
      const tf::isocontour &iso = m.isocontours[i];
      const int idx = static_cast<int>(i);
      const float x = tf::x_of(l, v, iso.at);
      draw_bar(dl, x, a.y, b.y, magenta, green, magenta, same(hot, tf::part::isocontour, idx),
               same(selected, tf::part::isocontour, idx));
      // The surface's own colour (its override, else the ramp) in a swatch at the top of the bar:
      // ColorTable2's isocontours had none, but the model's can.
      float rgb[3];
      m.isocontour_rgb(iso, rgb);
      const ImVec2 s0(x - 4.f, a.y + 1.f), s1(x + 4.f, a.y + 9.f);
      dl->AddRectFilled(s0, s1, col(rgb));
      dl->AddRect(s0, s1, magenta);
    }
  }
  if (v.show_opacity)
    draw_opacity(dl, l, v, m, hot, selected);
  dl->PopClipRect();
  dl->AddRect(a, b, ImGui::GetColorU32(ImGuiCol_Border));
}

// Under the canvas: the data values at the zoom window's ends and, while the pointer is over the
// canvas, the value and opacity under it (where a double-click or Add would put a node).
void draw_axis(ImDrawList *dl, const tf::layout &l, const tf::view &v, double lo, double hi,
               bool hovered, const ImVec2 &mouse) {
  const ImVec2 p(l.canvas_x, ImGui::GetCursorScreenPos().y);
  ImGui::Dummy(ImVec2(l.canvas_w, ImGui::GetTextLineHeight()));
  const ImU32 faint = ImGui::GetColorU32(ImGuiCol_TextDisabled);
  const std::string left = num(tf::to_data(v.zoom_lo, lo, hi));
  const std::string right = num(tf::to_data(v.zoom_hi, lo, hi));
  const float lw = ImGui::CalcTextSize(left.c_str()).x, rw = ImGui::CalcTextSize(right.c_str()).x;
  dl->AddText(p, faint, left.c_str());
  if (lw + rw + 8.f < l.canvas_w)
    dl->AddText(ImVec2(p.x + l.canvas_w - rw, p.y), faint, right.c_str());
  if (!hovered)
    return;
  char a[16];
  std::snprintf(a, sizeof(a), "%.3f", clamp01(tf::a_of(l, mouse.y)));
  const std::string mid =
      num(tf::to_data(clamp01(tf::t_of(l, v, mouse.x)), lo, hi)) + "  opacity " + a;
  const float mw = ImGui::CalcTextSize(mid.c_str()).x;
  const float x = p.x + 0.5f * (l.canvas_w - mw);
  if (x > p.x + lw + 8.f && x + mw + 8.f < p.x + l.canvas_w - rw)
    dl->AddText(ImVec2(x, p.y), ImGui::GetColorU32(ImGuiCol_Text), mid.c_str());
}

void draw_legend(ImDrawList *dl) {
  struct Entry {
    const char *name;
    float r, g, b;
  };
  const Entry entries[] = {{"area", 1.f, 0.f, 0.f},
                           {"min volume", 0.f, 1.f, 0.f},
                           {"max volume", 0.f, 0.f, 1.f},
                           {"gradient", 1.f, 1.f, 0.f}};
  const float lh = ImGui::GetTextLineHeight(), sz = std::floor(lh * 0.6f);
  bool first = true;
  for (const Entry &e : entries) {
    if (!first)
      ImGui::SameLine();
    first = false;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 s0(p.x, p.y + 0.5f * (lh - sz));
    dl->AddRectFilled(s0, s0 + ImVec2(sz, sz), col(e.r, e.g, e.b));
    ImGui::Dummy(ImVec2(sz, lh));
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(e.name);
  }
}

// ColorTable2's InfoDialog as a table: one row per isocontour, isovalues in data units; the meshed
// interval's bounds are marked in the id column.
void info_table(const tf::model &m, const tf::overlays &ov, double lo, double hi,
                const Interval &ivl) {
  if (m.isocontours.empty()) {
    ImGui::TextDisabled("No isocontours (right-click > Add > Isocontour)");
    return;
  }
  const ImGuiTableFlags flags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  if (!ImGui::BeginTable("##info", 7, flags))
    return;
  const char *const heads[] = {"id",      "isovalue", "area",      "min vol",
                               "max vol", "gradient", "components"};
  for (const char *h : heads)
    ImGui::TableSetupColumn(h);
  ImGui::TableHeadersRow();
  auto cell = [](const std::string &s) {
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(s.c_str());
  };
  for (std::size_t i = 0; i < m.isocontours.size(); ++i) {
    const tf::isocontour &iso = m.isocontours[i];
    const tf::isocontour_info info = tf::describe(m, iso, lo, hi, &ov);
    ImGui::TableNextRow();
    const char *bound = ivl.bound(static_cast<int>(i));
    cell(bound ? iso.id + " (interval " + (bound[0] == 'l' ? "lo)" : "hi)") : iso.id);
    cell(num(info.isovalue));
    cell(num_or_dash(info.area));
    cell(num_or_dash(info.min_volume));
    cell(num_or_dash(info.max_volume));
    cell(num_or_dash(info.gradient));
    cell(info.components >= 0 ? std::to_string(info.components) : std::string("-"));
  }
  ImGui::EndTable();
}

void swatch(const float rgb[3]) {
  const float sz = ImGui::GetTextLineHeight();
  ImGui::ColorButton("##swatch", ImVec4(rgb[0], rgb[1], rgb[2], 1.f),
                     ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
                     ImVec2(sz, sz));
  ImGui::SameLine();
}

// The hovered (or dragged) handle's values, positions in data units.
void tooltip(const tf::model &m, const tf::handle &h, const tf::view &v, const tf::overlays &ov,
             double lo, double hi, const Interval &ivl) {
  ImGui::BeginTooltip();
  switch (h.kind) {
  case tf::part::alpha: {
    const tf::alpha_node &n = m.alpha[static_cast<std::size_t>(h.index)];
    ImGui::Text("Opacity node %d", h.index + 1);
    ImGui::Text("value %s", num(tf::to_data(n.at, lo, hi)).c_str());
    if (m.opacity_cubed)
      ImGui::Text("opacity %.3f (cubed %.3f)", n.a, n.a * n.a * n.a);
    else
      ImGui::Text("opacity %.3f", n.a);
    break;
  }
  case tf::part::color: {
    const tf::color_node &n = m.color[static_cast<std::size_t>(h.index)];
    const float rgb[3] = {n.r, n.g, n.b};
    ImGui::Text("Colour node %d", h.index + 1);
    ImGui::Text("value %s", num(tf::to_data(n.at, lo, hi)).c_str());
    swatch(rgb);
    ImGui::Text("rgb %.3f, %.3f, %.3f", n.r, n.g, n.b);
    break;
  }
  case tf::part::isocontour: {
    const tf::isocontour &iso = m.isocontours[static_cast<std::size_t>(h.index)];
    const tf::isocontour_info info = tf::describe(m, iso, lo, hi, &ov);
    float rgb[3];
    m.isocontour_rgb(iso, rgb);
    ImGui::Text("Isocontour %s", iso.id.c_str());
    if (const char *bound = ivl.bound(h.index))
      ImGui::Text("%s bound of the meshed interval", bound);
    ImGui::Text("isovalue %s", num(info.isovalue).c_str());
    swatch(rgb);
    ImGui::Text("rgb %.3f, %.3f, %.3f%s", rgb[0], rgb[1], rgb[2], iso.has_rgb ? "" : " (ramp)");
    ImGui::Text("opacity %.3f", m.isocontour_opacity(iso));
    if (std::isfinite(info.area)) {
      ImGui::Text("area %s", num(info.area).c_str());
      ImGui::Text("min volume %s", num_or_dash(info.min_volume).c_str());
      ImGui::Text("max volume %s", num_or_dash(info.max_volume).c_str());
      ImGui::Text("gradient %s", num_or_dash(info.gradient).c_str());
    }
    if (info.components >= 0)
      ImGui::Text("components %d", info.components);
    break;
  }
  case tf::part::zoom_lo:
    ImGui::Text("Zoom from %s", num(tf::to_data(v.zoom_lo, lo, hi)).c_str());
    break;
  case tf::part::zoom_hi:
    ImGui::Text("Zoom to %s", num(tf::to_data(v.zoom_hi, lo, hi)).c_str());
    break;
  default:
    break;
  }
  ImGui::EndTooltip();
}

// ---- popups ---------------------------------------------------------------------------------

// The right-click menu. Model edits land in `m` (the caller commits them at once), view toggles
// in `v`.
void context_menu(session &s, tf::model &m, tf::view &v, bool &model_edit, bool &view_edit) {
  const tf::handle h = resolve(m, s.menu_handle);
  if (ImGui::BeginMenu("Add")) {
    // A node inserted ahead of the selected one shifts its index: keep the selection on it.
    const bool want_alpha = ImGui::MenuItem("Alpha node");
    const bool want_color = ImGui::MenuItem("Colour node");
    if (want_alpha || want_color) {
      const tf::model before = m;
      if (want_alpha ? tf::add_alpha(m, s.menu_t, s.menu_a) : tf::add_color(m, s.menu_t)) {
        model_edit = true;
        s.selected = reanchor(before, m, s.selected);
      }
    }
    if (ImGui::MenuItem("Isocontour")) {
      const std::string id = tf::add_isocontour(m, s.menu_t);
      if (!id.empty()) {
        model_edit = true;
        tf::handle iso;
        iso.kind = tf::part::isocontour;
        iso.id = id;
        s.selected = resolve(m, iso);
      }
    }
    ImGui::EndMenu();
  }
  const bool colourable = h.kind == tf::part::color || h.kind == tf::part::isocontour;
  if (ImGui::MenuItem("Edit colour...", nullptr, false, colourable)) {
    float rgb[3] = {1.f, 1.f, 1.f};
    if (h.kind == tf::part::color) {
      const tf::color_node &n = m.color[static_cast<std::size_t>(h.index)];
      rgb[0] = n.r;
      rgb[1] = n.g;
      rgb[2] = n.b;
    } else {
      m.isocontour_rgb(m.isocontours[static_cast<std::size_t>(h.index)], rgb);
    }
    for (int c = 0; c < 3; ++c)
      s.color_rgba[c] = s.color_ref[c] = rgb[c];
    s.color_rgba[3] = s.color_ref[3] = 1.f;
    s.color_target = h;
    s.open_color = true; // opened after this menu closes, so it is not the menu's child
  }
  const bool overridden =
      h.kind == tf::part::isocontour && m.isocontours[static_cast<std::size_t>(h.index)].has_rgb;
  if (ImGui::MenuItem("Use ramp colour", nullptr, false, overridden) &&
      tf::clear_isocontour_rgb(m, h.id))
    model_edit = true;
  if (ImGui::MenuItem("Delete", nullptr, false, deletable(m, h)) && tf::remove(m, h)) {
    model_edit = true;
    s.selected = {};
  }
  ImGui::Separator();
  if (ImGui::BeginMenu("Display")) {
    bool *const toggles[] = {&v.show_color,        &v.show_opacity,  &v.show_isocontours,
                             &v.show_histogram,    &v.log_histogram, &v.show_contour_spectrum,
                             &v.show_contour_tree, &v.show_info};
    const char *const names[] = {"Transfer function", "Opacity function",    "Isocontours",
                                 "Histogram",         "Log scale histogram", "Contour spectrum",
                                 "Contour tree",      "Contour information"};
    for (std::size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i)
      if (ImGui::MenuItem(names[i], nullptr, toggles[i]))
        view_edit = true;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Presets")) {
    for (const std::string &name : tf::model::preset_names())
      if (ImGui::MenuItem(name.c_str())) {
        replace_look(m, tf::model::preset(name));
        model_edit = true;
        s.selected = {};
      }
    ImGui::EndMenu();
  }
  bool cubed = m.opacity_cubed;
  if (ImGui::MenuItem("Opacity cubed", nullptr, &cubed)) {
    m.opacity_cubed = cubed;
    model_edit = true;
  }
  if (ImGui::MenuItem("Reset")) {
    replace_look(m, tf::model::default_model());
    model_edit = true;
    s.selected = {};
  }
  if (ImGui::MenuItem("Reset zoom", nullptr, false, v.zoom_lo > 0.0 || v.zoom_hi < 1.0)) {
    v.zoom_lo = 0.0;
    v.zoom_hi = 1.0;
    view_edit = true;
  }
  ImGui::Separator();
  if (ImGui::MenuItem("Import .vinay...")) {
    s.open_file = true;
    s.file_export = false;
    s.file_error.clear();
  }
  if (ImGui::MenuItem("Export .vinay...")) {
    s.open_file = true;
    s.file_export = true;
    s.file_error.clear();
  }
}

// Edit colour: a picker showing the current colour beside the original. OK applies; Cancel (or a
// click outside) leaves the node as it was — ColorTable2's dialog turned it black.
void colour_popup(session &s, tf::model &m, bool &model_edit) {
  if (!ImGui::BeginPopup("##colour"))
    return;
  const tf::handle h = resolve(m, s.color_target);
  if (h.kind == tf::part::isocontour)
    ImGui::Text("Isocontour %s", h.id.c_str());
  else
    ImGui::TextUnformatted("Colour node");
  ImGui::ColorPicker4("##picker", s.color_rgba, ImGuiColorEditFlags_NoAlpha, s.color_ref);
  ImGui::BeginDisabled(!h.valid()); // the node went away while the popup was open
  if (ImGui::Button("OK")) {
    if (h.kind == tf::part::color && tf::set_color(m, h.index, s.color_rgba))
      model_edit = true;
    else if (h.kind == tf::part::isocontour && tf::set_isocontour_rgb(m, h.id, s.color_rgba))
      model_edit = true;
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel"))
    ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

// Import / Export .vinay: a path field; a failure stays in the popup as text.
void vinay_popup(session &s, tf::model &m, bool &model_edit) {
  if (!ImGui::BeginPopup("##vinay"))
    return;
  ImGui::TextUnformatted(s.file_export ? "Export .vinay" : "Import .vinay");
  const float w = ImGui::GetFontSize() * 24.f;
  if (ImGui::IsWindowAppearing())
    ImGui::SetKeyboardFocusHere();
  ImGui::SetNextItemWidth(w);
  const bool enter = ImGui::InputText("##path", s.file_path, sizeof(s.file_path),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
  if (!s.file_error.empty()) {
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + w);
    ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f), "%s", s.file_error.c_str());
    ImGui::PopTextWrapPos();
  }
  if (ImGui::Button("OK") || enter) {
    const std::string path = s.file_path;
    if (path.empty()) {
      s.file_error = "enter a file path";
    } else {
      try {
        if (s.file_export) {
          tf::write_vinay_file(path, m);
        } else {
          replace_look(m, tf::read_vinay_file(path));
          model_edit = true;
          s.selected = {}; // as Reset and the presets: an index into the old look names nothing
        }
        s.file_error.clear();
        ImGui::CloseCurrentPopup();
      } catch (const std::exception &ex) {
        s.file_error = ex.what();
      }
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel"))
    ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

} // namespace
#endif

cvc::ariadne::TfEdit ImGuiBackend::tf_editor(const char *label, const char *key,
                                             const cvc::ariadne::TfEditorInput &in) {
#ifndef CVC_ENABLE_IMGUI
  (void)label;
  (void)key;
  (void)in;
  return {}; // drawn == false: the core composes an editor from its built-in widgets
#else
  static const tf::model kDefaultModel = tf::model::default_model();
  static const tf::overlays kNoOverlays;
  const tf::model &bound = in.model ? *in.model : kDefaultModel;
  const tf::overlays &ov = in.overlays ? *in.overlays : kNoOverlays;
  const double lo = in.lo, hi = in.hi > in.lo ? in.hi : in.lo + 1.0;
  // The overlays are sampled over the data range they were published with, which an explicit
  // domain need not match.
  double rlo = lo, rhi = hi;
  if (ov.has_range && ov.range_hi > ov.range_lo) {
    rlo = ov.range_lo;
    rhi = ov.range_hi;
  }

  const std::string skey = key ? key : "";
  // The id scope hashes the WHOLE key. PushID(const char *) would hash it as a label, which
  // restarts at "###": an instance id that fell back to a "##name" label makes the key
  // "<path>###name", and two such editors on different models would share every id below (a press
  // on one would also press the other).
  const std::string id_key = skey.empty() ? std::string("tf_editor") : skey;
  ImGui::PushID(static_cast<int>(ImHashData(id_key.data(), id_key.size())));
  // Keyed by the bind path AND the ImGui id, so two windows showing one model each get their own
  // drag and selection. Two editors on one model in the SAME window share both; the later ones are
  // told apart by their order this frame (and get their own ImGui ids too), or a press on one would
  // also press — and steal the grab of — the other.
  const std::string base = skey + "#" + std::to_string(ImGui::GetID("##session"));
  const int frame = ImGui::GetFrameCount();
  int dup = 0;
  std::shared_ptr<TfEditorSession> keep;
  for (;; ++dup) {
    std::shared_ptr<TfEditorSession> &slot =
        m_tfSessions[dup ? base + "#" + std::to_string(dup) : base];
    if (!slot)
      slot = std::make_shared<TfEditorSession>();
    if (slot->drawn_frame != frame) {
      slot->drawn_frame = frame;
      keep = slot;
      break;
    }
  }
  if (dup)
    ImGui::PushID(dup);
  TfEditorSession &s = *keep;

  cvc::ariadne::TfEdit e;
  e.drawn = true;
  e.view = in.view;
  tf::view &v = e.view;
  tf::clamp_zoom(v);

  // A write from outside (a script, another view, a replicated peer) replaced the model since this
  // session last saw it. The handles it holds index into the old lists, where a node inserted or
  // removed ahead of one hands its index to a neighbour: carry the selection and what the open menu
  // and colour picker were opened on over to the same nodes (dropping any that are gone), so their
  // Delete / OK act on that node or on nothing. A node drag is dropped outright. (During a delayed
  // drag the handles index `work`, which has `expected`'s nodes in the same order.)
  if (bound != s.expected) {
    s.selected = reanchor(s.expected, bound, s.selected);
    s.menu_handle = reanchor(s.expected, bound, s.menu_handle);
    s.color_target = reanchor(s.expected, bound, s.color_target);
    if (s.gesture.active() && is_node(s.gesture.grabbed().kind)) {
      s.gesture.cancel();
      s.working = false;
    }
    s.expected = bound;
  }
  tf::model m = s.working ? s.work : bound;
  bool edited = false, commit = false;

  if (label && *label) {
    const char *end = ImGui::FindRenderedTextEnd(label);
    if (end != label)
      ImGui::TextUnformatted(label, end);
  }
  const float width = std::max(ImGui::GetContentRegionAvail().x, kMinWidth);
  const float height = in.height > 0.f ? std::max(in.height, 2.f * kStripHeight) : kDefaultHeight;

  tf::layout l;
  ImGui::InvisibleButton("##zoom", ImVec2(width, kStripHeight));
  const Item strip = Item::last();
  l.strip_x = strip.min.x;
  l.strip_y = strip.min.y;
  l.strip_w = strip.max.x - strip.min.x;
  l.strip_h = strip.max.y - strip.min.y;
  ImGui::InvisibleButton("##canvas", ImVec2(width, height));
  const Item canvas = Item::last();
  const ImGuiLastItemData canvas_item = GImGui->LastItemData;
  l.canvas_x = canvas.min.x;
  l.canvas_y = canvas.min.y;
  l.canvas_w = canvas.max.x - canvas.min.x;
  l.canvas_h = canvas.max.y - canvas.min.y;

  const ImGuiIO &io = ImGui::GetIO();
  const ImVec2 mouse = io.MousePos;
  const bool held = strip.active || canvas.active;

  // Press: grab what is under the pointer. A double-click on empty canvas adds an opacity node
  // there and grabs it, so the same press can go on to drag it.
  if (strip.activated || canvas.activated) {
    const tf::handle h = s.gesture.press(m, v, l, mouse.x, mouse.y);
    if (is_node(h.kind)) {
      s.selected = h;
      s.working = !in.interactive;
    } else if (canvas.activated && !h.valid()) {
      s.selected = {};
      if (v.show_opacity && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) &&
          tf::add_alpha(m, clamp01(tf::t_of(l, v, mouse.x)),
                        static_cast<float>(clamp01(tf::a_of(l, mouse.y))))) {
        edited = commit = true;
        const tf::handle added = s.gesture.press(m, v, l, mouse.x, mouse.y);
        if (is_node(added.kind)) {
          s.selected = added;
          s.working = !in.interactive;
        }
      }
    }
  }
  // Drag, past ImGui's drag threshold so a click does not nudge the node it selects.
  if (s.gesture.active() && held && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    const tf::view before = v;
    if (s.gesture.drag(m, v, l, mouse.x, mouse.y)) {
      edited = true;
      if (in.interactive)
        commit = true;
    }
    if (v != before)
      e.view_changed = true;
    if (is_node(s.gesture.grabbed().kind))
      s.selected = s.gesture.grabbed();
  }
  // Release — or a drag lost because the widget was not drawn while the button came up. A delayed
  // update commits here, once, and only if the drag changed anything.
  if (s.gesture.active() && !held) {
    if (s.working && m != s.expected)
      edited = commit = true;
    s.gesture.release();
    s.working = false;
  }
  s.selected = resolve(m, s.selected);

  if (canvas.hovered && !s.gesture.active() && s.selected.valid() && !io.WantTextInput &&
      (ImGui::IsKeyPressed(ImGuiKey_Delete, false) ||
       ImGui::IsKeyPressed(ImGuiKey_Backspace, false))) {
    if (tf::remove(m, s.selected))
      edited = commit = true;
    s.selected = {};
  }

  if (canvas.hovered && !s.gesture.active() && ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
    s.menu_t = clamp01(tf::t_of(l, v, mouse.x));
    s.menu_a = static_cast<float>(clamp01(tf::a_of(l, mouse.y)));
    s.menu_handle = resolve(m, tf::hit_test(m, v, l, mouse.x, mouse.y));
    if (is_node(s.menu_handle.kind))
      s.selected = s.menu_handle;
    ImGui::OpenPopup("##menu");
  }
  // The popups run before the canvas draws so a menu edit shows in this very frame. Menu, picker
  // and file edits are discrete: they commit at once.
  bool popup_edit = false;
  if (ImGui::BeginPopup("##menu")) {
    context_menu(s, m, v, popup_edit, e.view_changed);
    ImGui::EndPopup();
  }
  if (s.open_color) {
    ImGui::OpenPopup("##colour");
    s.open_color = false;
  }
  colour_popup(s, m, popup_edit);
  if (s.open_file) {
    ImGui::OpenPopup("##vinay");
    s.open_file = false;
  }
  vinay_popup(s, m, popup_edit);
  if (popup_edit)
    edited = commit = true;

  if (commit) {
    m.normalize(); // what the core will store, so the next frame's model compares equal
    s.expected = m;
  }
  if (s.working)
    s.work = m;
  s.selected = resolve(m, s.selected);

  tf::handle hot;
  if (s.gesture.active())
    hot = resolve(m, s.gesture.grabbed());
  else if (canvas.hovered || strip.hovered)
    hot = resolve(m, tf::hit_test(m, v, l, mouse.x, mouse.y));

  const double now = ImGui::GetTime();
  if (hot.kind != s.hot.kind || hot.index != s.hot.index)
    s.hot_since = now;
  s.hot = hot;

  // The meshed interval on the model as drawn (a bar dragged this frame takes the band with it).
  const Interval ivl = find_interval(m, in.interval_a, in.interval_b);
  ImDrawList *dl = ImGui::GetWindowDrawList();
  draw_strip(dl, l, v, m, hot);
  draw_canvas(dl, l, v, m, ov, lo, hi, rlo, rhi, hot, s.selected, ivl);
  // While dragging, the tooltip is the live readout; on hover it waits for the pointer to settle.
  if (hot.valid() && (s.gesture.active() || now - s.hot_since >= ImGui::GetStyle().HoverDelayShort))
    tooltip(m, hot, v, ov, lo, hi, ivl);

  draw_axis(dl, l, v, lo, hi, canvas.hovered && !s.gesture.active(), mouse);
  if (v.show_contour_spectrum && !ov.spectrum.empty())
    draw_legend(dl);
  if (!ov.status.empty())
    ImGui::TextDisabled("%s", ov.status.c_str());
  if (v.show_info)
    info_table(m, ov, lo, hi, ivl);

  // Hand the canvas back as the last item: the core attaches the document's tooltip and on_click /
  // on_hover / on_drag handlers to "the item just drawn", which should be the canvas, not the
  // table or axis line under it.
  GImGui->LastItemData = canvas_item;
  if (dup)
    ImGui::PopID();
  ImGui::PopID();

  e.changed = edited;
  e.committed = commit;
  e.model = std::move(m);
  return e;
#endif
}

} // namespace gl
} // namespace cvc

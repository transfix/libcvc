/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/tf/editor.h>
#include <cvc/tf/overlays.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace tf {

namespace {

double clamp01(double t) { return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); }
float clamp01f(float t) { return t < 0.f ? 0.f : (t > 1.f ? 1.f : t); }

// The pixel extent a length maps across (ColorTable2 mapped [0,1] onto 0..w-1, so both ends land
// on a pixel).
float extent(float len) { return len > 1.f ? len - 1.f : 0.f; }

// The view's zoom window, falling back to the whole axis if it is empty or not finite (a view
// that never went through clamp_zoom()).
void window(const view &v, double &lo, double &span) {
  lo = v.zoom_lo;
  span = v.zoom_hi - v.zoom_lo;
  if (!std::isfinite(lo) || !std::isfinite(span) || span <= 0.0) {
    lo = 0.0;
    span = 1.0;
  }
}

// Where `t` goes in a list sorted by `at` (after any node at the same position).
template <typename List> std::size_t insert_pos(const List &nodes, double t) {
  std::size_t i = 0;
  while (i < nodes.size() && nodes[i].at <= t)
    ++i;
  return i;
}

template <typename List> bool crowded(const List &nodes, double t) {
  for (const auto &n : nodes)
    if (std::fabs(n.at - t) < kMinNodeGap)
      return true;
  return false;
}

// An interior node's new position, kept kMinNodeGap inside its neighbours. Returns false when the
// neighbours leave it no room (it then stays put).
template <typename List> bool clamp_between(const List &nodes, std::size_t i, double &t) {
  const double lo = nodes[i - 1].at + kMinNodeGap, hi = nodes[i + 1].at - kMinNodeGap;
  if (lo > hi)
    return false;
  t = std::min(std::max(t, lo), hi);
  return true;
}

// A vertical bar at screen x over [top, bottom] (inclusive pixel rows): grabbable within kBarGrab
// along its whole height, or on its 10x10 handle sitting on the bottom row.
bool bar_hit(float bx, float top, float bottom, float x, float y) {
  const float dx = std::fabs(x - bx);
  if (dx <= kBarGrab && y >= top && y <= bottom + 1.f)
    return true;
  const float cy = bottom - kHandleHalf;
  return dx <= kHandleHalf && std::fabs(y - cy) <= kHandleHalf;
}

// The node of an n-long colour / alpha list that hit(i) accepts: the topmost (highest index),
// except that a pinned endpoint yields to an interior node under the same point. A node dragged
// against an endpoint stops kMinNodeGap (a fraction of a pixel) short of it, so the two share every
// point; were the endpoint picked, the node could never be grabbed, selected or deleted again. The
// endpoint is pickable once the node is dragged off it. -1 when nothing is hit.
template <typename Hit> int pick_node(std::size_t n, Hit hit) {
  int endpoint = -1;
  for (std::size_t i = n; i-- > 0;) {
    if (!hit(i))
      continue;
    if (i > 0 && i + 1 < n)
      return static_cast<int>(i);
    if (endpoint < 0)
      endpoint = static_cast<int>(i);
  }
  return endpoint;
}

// The range the overlays' samples / bins are uniform over: theirs when published, else [lo, hi].
void overlay_range(const overlays &ov, double lo, double hi, double &rlo, double &rhi) {
  rlo = ov.has_range ? ov.range_lo : lo;
  rhi = ov.has_range ? ov.range_hi : hi;
}

// One spectrum function at fractional sample s (clamped to the samples).
double sample_at(const std::vector<double> &f, double s) {
  if (f.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const double last = static_cast<double>(f.size() - 1);
  s = std::min(std::max(s, 0.0), last);
  const std::size_t i = static_cast<std::size_t>(s);
  if (i + 1 >= f.size())
    return f.back();
  const double u = s - static_cast<double>(i);
  return f[i] + (f[i + 1] - f[i]) * u;
}

// --- the preview raster ------------------------------------------------------------------
struct color3 {
  float r, g, b;
};

std::uint8_t to_u8(float c) { return static_cast<std::uint8_t>(std::lround(clamp01f(c) * 255.f)); }

// The bytes of a w x h RGBA8 raster (w, h > 0), or 0 when that many do not fit a byte vector. The
// bound is checked before w*h*4 is formed, so the product can never wrap a 32-bit size_t (wasm32,
// Win32), and every pixel index y*w + x below it fits too.
std::size_t raster_bytes(int w, int h) {
  const std::size_t uw = static_cast<std::size_t>(w), uh = static_cast<std::size_t>(h);
  const std::size_t max_px = std::vector<std::uint8_t>().max_size() / 4;
  return uw <= max_px / uh ? uw * uh * 4 : 0;
}

struct raster {
  int w, h;
  std::vector<std::uint8_t> px;

  // `bytes` is raster_bytes(w_, h_), non-zero.
  raster(int w_, int h_, std::size_t bytes) : w(w_), h(h_), px(bytes, 255) {}

  void blend(int x, int y, color3 c, float a = 1.f) {
    if (x < 0 || y < 0 || x >= w || y >= h)
      return;
    std::uint8_t *p = &px[(static_cast<std::size_t>(y) * w + x) * 4];
    const float k = 1.f - a;
    p[0] = to_u8(p[0] / 255.f * k + c.r * a);
    p[1] = to_u8(p[1] / 255.f * k + c.g * a);
    p[2] = to_u8(p[2] / 255.f * k + c.b * a);
    p[3] = 255;
  }

  // A 1 px line, clipped to the raster first (Liang-Barsky) so a segment far outside a zoomed
  // window costs nothing.
  void line(double x0, double y0, double x1, double y1, color3 c) {
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1))
      return;
    double u0 = 0.0, u1 = 1.0;
    const double dx = x1 - x0, dy = y1 - y0;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {x0 + 0.5, w - 0.5 - x0, y0 + 0.5, h - 0.5 - y0};
    for (int k = 0; k < 4; ++k) {
      if (p[k] == 0.0) {
        if (q[k] < 0.0)
          return;
        continue;
      }
      const double r = q[k] / p[k];
      if (p[k] < 0.0)
        u0 = std::max(u0, r);
      else
        u1 = std::min(u1, r);
      if (u0 > u1)
        return;
    }
    const double ax = x0 + u0 * dx, ay = y0 + u0 * dy;
    const double bx = x0 + u1 * dx, by = y0 + u1 * dy;
    const int steps = static_cast<int>(std::ceil(std::max(std::fabs(bx - ax), std::fabs(by - ay))));
    for (int s = 0; s <= steps; ++s) {
      const double u = steps ? static_cast<double>(s) / steps : 0.0;
      blend(static_cast<int>(std::lround(ax + (bx - ax) * u)),
            static_cast<int>(std::lround(ay + (by - ay) * u)), c);
    }
  }
};

} // namespace

bool view::operator==(const view &o) const {
  return zoom_lo == o.zoom_lo && zoom_hi == o.zoom_hi && show_color == o.show_color &&
         show_opacity == o.show_opacity && show_isocontours == o.show_isocontours &&
         show_histogram == o.show_histogram && show_contour_spectrum == o.show_contour_spectrum &&
         show_contour_tree == o.show_contour_tree && log_histogram == o.log_histogram &&
         show_info == o.show_info;
}

void clamp_zoom(view &v) {
  double lo = std::isfinite(v.zoom_lo) ? clamp01(v.zoom_lo) : 0.0;
  double hi = std::isfinite(v.zoom_hi) ? clamp01(v.zoom_hi) : 1.0;
  if (lo > hi)
    std::swap(lo, hi);
  // The tolerance keeps the clamp idempotent: a window built as hi = lo + kMinZoomSpan may come
  // out an ulp short, and must not be re-centred on every call.
  if (hi - lo < kMinZoomSpan * (1.0 - 1e-9)) {
    const double c = 0.5 * (lo + hi);
    if (c - 0.5 * kMinZoomSpan < 0.0) {
      lo = 0.0;
      hi = kMinZoomSpan;
    } else if (c + 0.5 * kMinZoomSpan > 1.0) {
      lo = 1.0 - kMinZoomSpan;
      hi = 1.0;
    } else {
      lo = c - 0.5 * kMinZoomSpan;
      hi = lo + kMinZoomSpan;
    }
  }
  v.zoom_lo = lo;
  v.zoom_hi = hi;
}

// --- mapping ------------------------------------------------------------------------------

float x_of(const layout &l, const view &v, double t) {
  double lo, span;
  window(v, lo, span);
  return l.canvas_x + static_cast<float>((t - lo) / span * extent(l.canvas_w));
}

double t_of(const layout &l, const view &v, float x) {
  double lo, span;
  window(v, lo, span);
  const float e = extent(l.canvas_w);
  return e > 0.f ? lo + static_cast<double>(x - l.canvas_x) / e * span : lo;
}

float y_of(const layout &l, float a) { return l.canvas_y + (1.f - a) * extent(l.canvas_h); }

float a_of(const layout &l, float y) {
  const float e = extent(l.canvas_h);
  return e > 0.f ? 1.f - (y - l.canvas_y) / e : 0.f;
}

float strip_x_of(const layout &l, double t) {
  return l.strip_x + static_cast<float>(t * extent(l.strip_w));
}

double strip_t_of(const layout &l, float x) {
  const float e = extent(l.strip_w);
  return e > 0.f ? static_cast<double>(x - l.strip_x) / e : 0.0;
}

// --- hit testing ---------------------------------------------------------------------------

handle hit_test(const model &m, const view &v, const layout &l, float x, float y) {
  handle h;
  if (l.strip_h > 0.f && l.strip_w > 0.f && y >= l.strip_y && y <= l.strip_y + l.strip_h &&
      x >= l.strip_x - kHandleHalf && x <= l.strip_x + l.strip_w + kHandleHalf) {
    const float top = l.strip_y, bottom = l.strip_y + l.strip_h - 1.f;
    const float xl = strip_x_of(l, v.zoom_lo), xh = strip_x_of(l, v.zoom_hi);
    const bool on_lo = bar_hit(xl, top, bottom, x, y), on_hi = bar_hit(xh, top, bottom, x, y);
    // The bars can sit a few px apart: take the nearer; on a tie, the one with room to move.
    const float dl = std::fabs(x - xl), dh = std::fabs(x - xh);
    if (on_lo && (!on_hi || dl < dh || (dl == dh && v.zoom_hi >= 1.0)))
      h.kind = part::zoom_lo;
    else if (on_hi)
      h.kind = part::zoom_hi;
    return h;
  }
  if (x < l.canvas_x - kHandleHalf || x > l.canvas_x + l.canvas_w + kHandleHalf ||
      y < l.canvas_y - kHandleHalf || y > l.canvas_y + l.canvas_h + kHandleHalf)
    return h;
  const float top = l.canvas_y, bottom = l.canvas_y + l.canvas_h - 1.f;
  // Nearest layer first (ColorTable2's depth order); within a layer the last drawn is on top, but
  // a colour / alpha endpoint yields to an interior node (pick_node).
  if (v.show_opacity) {
    const int i = pick_node(m.alpha.size(), [&](std::size_t k) {
      const float nx = x_of(l, v, m.alpha[k].at), ny = y_of(l, m.alpha[k].a);
      return std::fabs(x - nx) <= kHandleHalf && std::fabs(y - ny) <= kHandleHalf;
    });
    if (i >= 0) {
      h.kind = part::alpha;
      h.index = i;
      return h;
    }
  }
  if (v.show_isocontours)
    for (std::size_t i = m.isocontours.size(); i-- > 0;)
      if (bar_hit(x_of(l, v, m.isocontours[i].at), top, bottom, x, y)) {
        h.kind = part::isocontour;
        h.index = static_cast<int>(i);
        h.id = m.isocontours[i].id;
        return h;
      }
  if (v.show_color) {
    const int i = pick_node(m.color.size(), [&](std::size_t k) {
      return bar_hit(x_of(l, v, m.color[k].at), top, bottom, x, y);
    });
    if (i >= 0) {
      h.kind = part::color;
      h.index = i;
    }
  }
  return h;
}

// --- edits -------------------------------------------------------------------------------------

bool add_alpha(model &m, double t, float a) {
  if (std::isnan(t) || std::isnan(a))
    return false;
  t = clamp01(t);
  if (crowded(m.alpha, t))
    return false;
  alpha_node n;
  n.at = t;
  n.a = clamp01f(a);
  m.alpha.insert(m.alpha.begin() + static_cast<std::ptrdiff_t>(insert_pos(m.alpha, t)), n);
  return true;
}

bool add_color(model &m, double t) {
  if (std::isnan(t))
    return false;
  t = clamp01(t);
  if (crowded(m.color, t))
    return false;
  float rgb[3];
  m.color_at(t, rgb); // ColorTable2's ADD_COLOR: the ramp is unchanged until the node is edited
  color_node n;
  n.at = t;
  n.r = rgb[0];
  n.g = rgb[1];
  n.b = rgb[2];
  m.color.insert(m.color.begin() + static_cast<std::ptrdiff_t>(insert_pos(m.color, t)), n);
  return true;
}

std::string add_isocontour(model &m, double t) {
  if (std::isnan(t))
    return std::string();
  isocontour iso;
  iso.id = m.next_isocontour_id();
  iso.at = clamp01(t);
  const std::size_t i = insert_pos(m.isocontours, iso.at);
  m.isocontours.insert(m.isocontours.begin() + static_cast<std::ptrdiff_t>(i), iso);
  return m.isocontours[i].id;
}

bool remove(model &m, const handle &h) {
  switch (h.kind) {
  case part::color:
    if (h.index <= 0 || h.index + 1 >= static_cast<int>(m.color.size()))
      return false; // an endpoint, or not a node
    m.color.erase(m.color.begin() + h.index);
    return true;
  case part::alpha:
    if (h.index <= 0 || h.index + 1 >= static_cast<int>(m.alpha.size()))
      return false;
    m.alpha.erase(m.alpha.begin() + h.index);
    return true;
  case part::isocontour: {
    const int i = h.id.empty() ? h.index : m.find_isocontour(h.id);
    if (i < 0 || i >= static_cast<int>(m.isocontours.size()))
      return false;
    m.isocontours.erase(m.isocontours.begin() + i);
    return true;
  }
  default:
    return false;
  }
}

bool set_color(model &m, int color_index, const float rgb[3]) {
  if (!rgb || color_index < 0 || color_index >= static_cast<int>(m.color.size()) ||
      std::isnan(rgb[0]) || std::isnan(rgb[1]) || std::isnan(rgb[2]))
    return false;
  color_node &n = m.color[static_cast<std::size_t>(color_index)];
  const float r = clamp01f(rgb[0]), g = clamp01f(rgb[1]), b = clamp01f(rgb[2]);
  if (n.r == r && n.g == g && n.b == b)
    return false;
  n.r = r;
  n.g = g;
  n.b = b;
  return true;
}

bool set_isocontour_rgb(model &m, const std::string &id, const float rgb[3]) {
  const int i = m.find_isocontour(id);
  if (!rgb || i < 0 || std::isnan(rgb[0]) || std::isnan(rgb[1]) || std::isnan(rgb[2]))
    return false;
  isocontour &iso = m.isocontours[static_cast<std::size_t>(i)];
  const float r = clamp01f(rgb[0]), g = clamp01f(rgb[1]), b = clamp01f(rgb[2]);
  if (iso.has_rgb && iso.r == r && iso.g == g && iso.b == b)
    return false;
  iso.has_rgb = true;
  iso.r = r;
  iso.g = g;
  iso.b = b;
  return true;
}

bool clear_isocontour_rgb(model &m, const std::string &id) {
  const int i = m.find_isocontour(id);
  if (i < 0 || !m.isocontours[static_cast<std::size_t>(i)].has_rgb)
    return false;
  isocontour &iso = m.isocontours[static_cast<std::size_t>(i)];
  // Back to the defaults too, so the model equals one read back from its string (which carries no
  // rgb for a ramp-coloured isocontour).
  iso.has_rgb = false;
  iso.r = iso.g = iso.b = 1.f;
  return true;
}

bool move_color(model &m, int index, double t) {
  const int n = static_cast<int>(m.color.size());
  if (index <= 0 || index + 1 >= n || std::isnan(t))
    return false; // endpoints are pinned in t (their colour edits through set_color)
  const std::size_t i = static_cast<std::size_t>(index);
  t = clamp01(t);
  if (!clamp_between(m.color, i, t) || t == m.color[i].at)
    return false;
  m.color[i].at = t;
  return true;
}

bool move_alpha(model &m, int index, double t, float a) {
  const int n = static_cast<int>(m.alpha.size());
  if (index < 0 || index >= n || std::isnan(t) || std::isnan(a))
    return false;
  const std::size_t i = static_cast<std::size_t>(index);
  alpha_node &node = m.alpha[i];
  double at = node.at;
  if (index > 0 && index + 1 < n) {
    double want = clamp01(t);
    if (clamp_between(m.alpha, i, want))
      at = want;
  }
  a = clamp01f(a);
  if (at == node.at && a == node.a)
    return false;
  node.at = at;
  node.a = a;
  return true;
}

bool move_isocontour(model &m, const std::string &id, double t) {
  const int i = m.find_isocontour(id);
  if (i < 0 || std::isnan(t))
    return false;
  t = clamp01(t);
  if (m.isocontours[static_cast<std::size_t>(i)].at == t)
    return false;
  isocontour iso = std::move(m.isocontours[static_cast<std::size_t>(i)]);
  m.isocontours.erase(m.isocontours.begin() + i);
  iso.at = t;
  const std::size_t j = insert_pos(m.isocontours, t);
  m.isocontours.insert(m.isocontours.begin() + static_cast<std::ptrdiff_t>(j), std::move(iso));
  return true;
}

// --- the drag gesture -----------------------------------------------------------------------

handle gesture::press(const model &m, const view &v, const layout &l, float x, float y) {
  _grab = hit_test(m, v, l, x, y);
  _dx = _dy = 0.f;
  _view_changed = _model_changed = false;
  switch (_grab.kind) {
  case part::alpha: {
    const alpha_node &n = m.alpha[static_cast<std::size_t>(_grab.index)];
    _dx = x - x_of(l, v, n.at);
    _dy = y - y_of(l, n.a);
    break;
  }
  case part::color:
    _dx = x - x_of(l, v, m.color[static_cast<std::size_t>(_grab.index)].at);
    break;
  case part::isocontour:
    _dx = x - x_of(l, v, m.isocontours[static_cast<std::size_t>(_grab.index)].at);
    break;
  case part::zoom_lo:
    _dx = x - strip_x_of(l, v.zoom_lo);
    break;
  case part::zoom_hi:
    _dx = x - strip_x_of(l, v.zoom_hi);
    break;
  case part::none:
    break;
  }
  return _grab;
}

bool gesture::drag(model &m, view &v, const layout &l, float x, float y) {
  bool changed = false;
  switch (_grab.kind) {
  case part::color:
    changed = move_color(m, _grab.index, clamp01(t_of(l, v, x - _dx)));
    break;
  case part::alpha:
    changed = move_alpha(m, _grab.index, clamp01(t_of(l, v, x - _dx)), clamp01f(a_of(l, y - _dy)));
    break;
  case part::isocontour:
    changed = move_isocontour(m, _grab.id, clamp01(t_of(l, v, x - _dx)));
    if (changed)
      _grab.index = m.find_isocontour(_grab.id);
    break;
  case part::zoom_lo:
  case part::zoom_hi: {
    const double t = clamp01(strip_t_of(l, x - _dx));
    double lo = v.zoom_lo, hi = v.zoom_hi;
    if (_grab.kind == part::zoom_lo)
      lo = std::max(0.0, std::min(t, hi - kMinZoomSpan));
    else
      hi = std::min(1.0, std::max(t, lo + kMinZoomSpan));
    if (lo != v.zoom_lo || hi != v.zoom_hi) {
      v.zoom_lo = lo;
      v.zoom_hi = hi;
      _view_changed = true;
    }
    break;
  }
  case part::none:
    break;
  }
  if (changed)
    _model_changed = true;
  return changed;
}

void gesture::release() {
  _grab = handle();
  _dx = _dy = 0.f;
}

// --- information about one isocontour -------------------------------------------------------

isocontour_info describe(const model & /*m*/, const isocontour &iso, double lo, double hi,
                         const overlays *ov) {
  isocontour_info info;
  info.id = iso.id;
  info.at = iso.at;
  info.isovalue = to_data(iso.at, lo, hi);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  info.area = info.min_volume = info.max_volume = info.gradient = nan;
  if (!ov)
    return info;
  const spectrum_overlay &s = ov->spectrum;
  const std::size_t n = s.size();
  if (n > 0 && s.min_volume.size() == n && s.max_volume.size() == n && s.gradient.size() == n) {
    double rlo, rhi;
    overlay_range(*ov, lo, hi, rlo, rhi);
    const double pos =
        rhi > rlo ? (info.isovalue - rlo) / (rhi - rlo) * static_cast<double>(n - 1) : 0.0;
    info.area = sample_at(s.area, pos);
    info.min_volume = sample_at(s.min_volume, pos);
    info.max_volume = sample_at(s.max_volume, pos);
    info.gradient = sample_at(s.gradient, pos);
  }
  if (!ov->tree.empty())
    info.components = ov->tree.components_at(info.isovalue);
  return info;
}

// --- the meshed interval --------------------------------------------------------------------

bool interval_isocontours(const model &m, const std::string &interval, int &a, int &b) {
  for (std::size_t c = interval.find(','); c != std::string::npos; c = interval.find(',', c + 1)) {
    const std::string ia = interval.substr(0, c), ib = interval.substr(c + 1);
    if (ia.empty() || ib.empty() || ia == ib)
      continue;
    const int i = m.find_isocontour(ia), j = m.find_isocontour(ib);
    if (i >= 0 && j >= 0) {
      a = i;
      b = j;
      return true;
    }
  }
  return false;
}

// --- the raster preview ---------------------------------------------------------------------

std::vector<std::uint8_t> render_preview(const model &m, const view &v, const overlays *ov,
                                         double lo, double hi, int w, int h) {
  return render_preview(m, v, ov, lo, hi, w, h, preview_options());
}

std::vector<std::uint8_t> render_preview(const model &m, const view &v, const overlays *ov,
                                         double lo, double hi, int w, int h,
                                         const preview_options &opt) {
  if (w <= 0 || h <= 0)
    return {};
  const std::size_t bytes = raster_bytes(w, h);
  if (bytes == 0)
    return {}; // more than a vector can hold (see editor.h)
  if (!(hi > lo))
    hi = lo + 1.0; // effective_domain()'s rule, so the data <-> t mapping stays invertible
  raster r(w, h, bytes);
  layout l;
  l.canvas_w = static_cast<float>(w);
  l.canvas_h = static_cast<float>(h);
  const color3 black{0.f, 0.f, 0.f};

  // Background: the colour ramp under the zoom window, one sample per column.
  for (int x = 0; x < w; ++x) {
    const double t = clamp01(t_of(l, v, static_cast<float>(x)));
    color3 c{0.5f, 0.5f, 0.5f};
    float a = 1.f;
    if (v.show_color) {
      float rgb[3];
      m.color_at(t, rgb);
      c = color3{rgb[0], rgb[1], rgb[2]};
      if (opt.alpha_checker)
        a = clamp01f(m.alpha_at(t));
    }
    for (int y = 0; y < h; ++y) {
      if (a < 1.f) {
        const float check = ((x / 4 + y / 4) & 1) ? 0.75f : 0.45f;
        r.blend(x, y, color3{check, check, check});
      }
      r.blend(x, y, c, a);
    }
  }

  double rlo = lo, rhi = hi;
  if (ov)
    overlay_range(*ov, lo, hi, rlo, rhi);
  const bool has_range = rhi > rlo && std::isfinite(rlo) && std::isfinite(rhi);
  // The column a data value lands in.
  auto x_of_value = [&](double d) { return x_of(l, v, to_normalized(d, lo, hi)); };

  // Histogram: bars from the bottom, following the zoom window (ColorTable2's did not).
  if (v.show_histogram && ov && !ov->histogram.empty() && has_range) {
    const std::vector<double> &bins = ov->histogram;
    const std::size_t n = bins.size();
    double hmax = 0.0, hsecond = 0.0, hmin = std::numeric_limits<double>::infinity();
    for (const double c : bins) {
      if (c > hmax) {
        hsecond = hmax;
        hmax = c;
      } else if (c > hsecond) {
        hsecond = c;
      }
      hmin = std::min(hmin, c);
    }
    // ColorTable2's linear scale: a spike (the background value, typically) is clipped at 1.5x the
    // second-largest bin so the rest stays readable; the smallest bin is the baseline. Unlike
    // ColorTable2, a lone non-empty bin (second-largest at the baseline) is not clipped away.
    double top = hmax, base = hmin;
    if (n >= 2 && hmax > 1.5 * hsecond && 1.5 * hsecond > hmin)
      top = 1.5 * hsecond;
    if (v.log_histogram) {
      top = std::log1p(std::max(hmax, 0.0));
      base = 0.0;
    }
    if (top > base) {
      auto height = [&](double c) {
        const double f =
            v.log_histogram ? std::log1p(std::max(c, 0.0)) / top : (c - base) / (top - base);
        return std::min(std::max(f, 0.0), 1.0);
      };
      const color3 bar = v.show_color ? color3{0.25f, 0.25f, 0.25f} : black;
      const float alpha = v.show_color ? 0.4f : 1.f;
      const double scale = static_cast<double>(n) / (rhi - rlo);
      for (int x = 0; x < w; ++x) {
        // Every bin the column covers, so a zoomed-out canvas does not alias spikes away.
        const double d0 = to_data(t_of(l, v, x - 0.5f), lo, hi);
        const double d1 = to_data(t_of(l, v, x + 0.5f), lo, hi);
        if (d1 < rlo || d0 > rhi)
          continue;
        const double b0 = std::floor((std::max(d0, rlo) - rlo) * scale);
        const double b1 = std::floor((std::min(d1, rhi) - rlo) * scale);
        const std::size_t i0 = static_cast<std::size_t>(std::min(std::max(b0, 0.0), n - 1.0));
        const std::size_t i1 = static_cast<std::size_t>(std::min(std::max(b1, 0.0), n - 1.0));
        double f = 0.0;
        for (std::size_t i = i0; i <= i1; ++i)
          f = std::max(f, height(bins[i]));
        if (f <= 0.0)
          continue;
        const int y_top = static_cast<int>(std::lround((1.0 - f) * (h - 1)));
        for (int y = y_top; y < h; ++y)
          r.blend(x, y, bar, alpha);
      }
    }
  }

  // The meshed interval: every column from one bounding bar's to the other's, a translucent
  // magenta (the bars' colour) over the ramp and the histogram, under the curves and the bars.
  // Hidden with the isocontours.
  if (v.show_isocontours) {
    const int ia = opt.interval_a.empty() ? -1 : m.find_isocontour(opt.interval_a);
    const int ib = opt.interval_b.empty() ? -1 : m.find_isocontour(opt.interval_b);
    if (ia >= 0 && ib >= 0 && ia != ib) {
      const double ta = m.isocontours[static_cast<std::size_t>(ia)].at;
      const double tb = m.isocontours[static_cast<std::size_t>(ib)].at;
      // Clamped before rounding: under a narrow zoom window a bar's column can leave long's range.
      const auto column = [&](double t) {
        const float fx = x_of(l, v, t);
        return static_cast<int>(std::lround(
            std::isfinite(fx) ? std::min(std::max(fx, -1.f), static_cast<float>(w)) : -1.f));
      };
      const int x0 = std::max(column(std::min(ta, tb)), 0);
      const int x1 = std::min(column(std::max(ta, tb)), w - 1);
      for (int x = x0; x <= x1; ++x)
        for (int y = 0; y < h; ++y)
          r.blend(x, y, color3{1.f, 0.f, 1.f}, kIntervalAlpha);
    }
  }

  // Contour tree: value along the TF axis, layout x vertically with ColorTable2's 10% inset.
  if (v.show_contour_tree && ov && !ov->tree.empty()) {
    const tree_overlay &t = ov->tree;
    const int nv = static_cast<int>(t.vertices.size());
    auto tree_y = [&](float x) { return (1.0 - x) * (h - 1) * 0.8 + h * 0.1; };
    for (const auto &e : t.edges) {
      if (e.first < 0 || e.second < 0 || e.first >= nv || e.second >= nv)
        continue;
      const tree_overlay::vertex &a = t.vertices[static_cast<std::size_t>(e.first)];
      const tree_overlay::vertex &b = t.vertices[static_cast<std::size_t>(e.second)];
      r.line(x_of_value(a.value), tree_y(a.x), x_of_value(b.value), tree_y(b.x), black);
    }
  }

  // Contour spectrum: each function min-max normalized over the full height.
  if (v.show_contour_spectrum && ov && !ov->spectrum.empty() && has_range) {
    const spectrum_overlay &s = ov->spectrum;
    const struct {
      const std::vector<double> *f;
      color3 c;
    } curves[] = {{&s.area, {1.f, 0.f, 0.f}},
                  {&s.min_volume, {0.f, 1.f, 0.f}},
                  {&s.max_volume, {0.f, 0.f, 1.f}},
                  {&s.gradient, {1.f, 1.f, 0.f}}};
    for (const auto &curve : curves) {
      const std::vector<double> &f = *curve.f;
      if (f.size() < 2)
        continue;
      const auto mm = std::minmax_element(f.begin(), f.end());
      const double fmin = *mm.first, span = *mm.second - *mm.first;
      if (!(span > 0.0))
        continue; // a constant function carries nothing (ColorTable2 divided by zero)
      const double last = static_cast<double>(f.size() - 1);
      for (std::size_t i = 0; i + 1 < f.size(); ++i) {
        const double d0 = rlo + (rhi - rlo) * (static_cast<double>(i) / last);
        const double d1 = rlo + (rhi - rlo) * (static_cast<double>(i + 1) / last);
        r.line(x_of_value(d0), (1.0 - (f[i] - fmin) / span) * (h - 1), x_of_value(d1),
               (1.0 - (f[i + 1] - fmin) / span) * (h - 1), curve.c);
      }
    }
  }

  // Isocontour bars: ColorTable2's magenta (the inverse of its green handle) over the full height,
  // so a bar reads on any ramp colour, with a green foot.
  if (v.show_isocontours) {
    const int foot = std::min(std::max(h / 8, 2), 5);
    for (const isocontour &iso : m.isocontours) {
      // A bar off the canvas is skipped before it is rounded: under a zoom window narrower than
      // clamp_zoom() allows its column can leave long's range, where lround() is unspecified (the
      // UCRT returns 0: a stray bar at column 0).
      const float fx = x_of(l, v, iso.at);
      if (!(fx >= -2.f && fx <= static_cast<float>(w) + 1.f))
        continue;
      const int x = static_cast<int>(std::lround(fx));
      for (int y = 0; y < h; ++y)
        r.blend(x, y, color3{1.f, 0.f, 1.f});
      for (int y = h - foot; y < h; ++y)
        for (int dx = -1; dx <= 1; ++dx)
          r.blend(x + dx, y, color3{0.f, 1.f, 0.f});
    }
  }

  // The opacity curve through the nodes (drawn linear, as ColorTable2 did, even when cubed).
  if (v.show_opacity)
    for (std::size_t i = 0; i + 1 < m.alpha.size(); ++i)
      r.line(x_of(l, v, m.alpha[i].at), y_of(l, m.alpha[i].a), x_of(l, v, m.alpha[i + 1].at),
             y_of(l, m.alpha[i + 1].a), black);

  return std::move(r.px);
}

} // namespace tf
} // namespace cvc

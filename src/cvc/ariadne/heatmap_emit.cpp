/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "heatmap_emit.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kMaxSide = 256; // rows or cols read at most

std::string trim(const std::string &s) {
  std::size_t a = 0, z = s.size();
  while (a < z && std::isspace(static_cast<unsigned char>(s[a])))
    ++a;
  while (z > a && std::isspace(static_cast<unsigned char>(s[z - 1])))
    --z;
  return s.substr(a, z - a);
}

// One number, whatever LC_NUMERIC says; NaN when the text is empty or not a number.
double parse_number(const std::string &raw) {
  const std::string t = trim(raw);
  if (t.empty())
    return kNaN;
  std::istringstream in(t);
  in.imbue(std::locale::classic());
  double v = kNaN;
  if (!(in >> v))
    return kNaN;
  in >> std::ws;
  return in.eof() ? v : kNaN;
}

// A count: a whole number >= 0, else 0.
int parse_count(const std::string &raw) {
  const double v = parse_number(raw);
  return std::isfinite(v) && v > 0.0 ? static_cast<int>(std::min(v, 1e9)) : 0;
}

// A number for a person to read: `digits` significant digits, and no "-0".
std::string short_number(double d, int digits) {
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os.precision(digits);
  os << (d == 0.0 ? 0.0 : d);
  return os.str();
}

std::string replace_all(std::string s, const std::string &token, const std::string &rep) {
  for (std::size_t pos = s.find(token); pos != std::string::npos;
       pos = s.find(token, pos + rep.size()))
    s.replace(pos, token.size(), rep);
  return s;
}

// What a state path holds, read through link holes; "" when it has nothing.
std::string read_text(cvc::app &app, const std::string &path) {
  try {
    cvc::state *n = read_node(app, path);
    return n ? n->value() : std::string();
  } catch (const std::exception &) {
    return std::string();
  }
}

bool is_expr(const std::string &s) {
  for (char c : s) {
    if (std::isspace(static_cast<unsigned char>(c)))
      continue;
    return c == '(';
  }
  return false;
}

// The width of `s` in a monospaced terminal: its code points (the labels may be UTF-8).
std::size_t text_width(const std::string &s) {
  std::size_t n = 0;
  for (unsigned char c : s)
    if ((c & 0xC0) != 0x80)
      ++n;
  return n;
}

std::string pad_left(const std::string &s, std::size_t w) {
  const std::size_t n = text_width(s);
  return n >= w ? s : std::string(w - n, ' ') + s;
}
std::string pad_right(const std::string &s, std::size_t w) {
  const std::size_t n = text_width(s);
  return n >= w ? s : s + std::string(w - n, ' ');
}

// Reads the subtree through link holes, noting when a hop was a link: a write to a link's target
// is signalled under the target's path, not this subtree's, so such a heatmap is read every frame.
struct Reader {
  bool live = false;
  std::vector<cvc::state::state_ptr> pins;

  cvc::state *find(cvc::state *from, const std::string &rel) {
    if (!from)
      return nullptr;
    pins.clear();
    try {
      cvc::state::state_ptr n = from->findThroughLinksShared(rel, &pins);
      for (const cvc::state::state_ptr &p : pins)
        if (p && p->isLink()) {
          live = true;
          break;
        }
      return n.get(); // the tree owns it; nothing is removed during the walk's read
    } catch (const std::exception &) {
      return nullptr;
    }
  }
  std::string text(cvc::state *from, const std::string &rel) {
    cvc::state *n = find(from, rel);
    return n ? n->value() : std::string();
  }
};

// A label list: <name>.<i> children, or comma-separated text at <name>. Empty when there is none;
// else one for each of `n`.
std::vector<std::string> read_labels(Reader &rd, cvc::state *base, const char *name, int n) {
  cvc::state *node = rd.find(base, name);
  if (!node || n <= 0)
    return {};
  std::vector<std::string> out;
  if (node->numChildren() > 0) {
    for (int i = 0; i < n; ++i)
      out.push_back(rd.text(node, std::to_string(i)));
  } else {
    const std::string v = node->value();
    if (trim(v).empty())
      return {};
    for (std::size_t i = 0; i <= v.size() && static_cast<int>(out.size()) < n;) {
      const std::size_t j = std::min(v.find(',', i), v.size());
      out.push_back(trim(v.substr(i, j - i)));
      i = j + 1;
    }
  }
  out.resize(static_cast<std::size_t>(n));
  return out;
}

bool same_ramp(const std::vector<HeatmapStop> &a, const std::vector<HeatmapStop> &b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i].at != b[i].at || a[i].rgb[0] != b[i].rgb[0] || a[i].rgb[1] != b[i].rgb[1] ||
        a[i].rgb[2] != b[i].rgb[2])
      return false;
  return true;
}

// Read the subtree at `base` into the cache's spec.
void read_heatmap(HeatmapContext &ctx, const Widget &w, const std::string &base,
                  HeatmapCache &cache) {
  Reader rd;
  HeatmapSpec &spec = cache.spec;
  spec.cells.clear();
  spec.row_labels.clear();
  spec.col_labels.clear();
  cache.tips.clear();
  cvc::state *root = rd.find(&cvc::state::instance(ctx.app), base);
  int rows = root ? parse_count(rd.text(root, "rows")) : 0;
  int cols = root ? parse_count(rd.text(root, "cols")) : 0;
  if (rows > kMaxSide || cols > kMaxSide) {
    ctx.warn_once("ari: heatmap '" + w.label + "' is " + std::to_string(rows) + " x " +
                  std::to_string(cols) + " — the first " + std::to_string(kMaxSide) +
                  " of each drawn");
    rows = std::min(rows, kMaxSide);
    cols = std::min(cols, kMaxSide);
  }
  if (rows == 0 || cols == 0)
    rows = cols = 0;
  spec.rows = rows;
  spec.cols = cols;
  spec.cells.resize(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
  cache.tips.resize(spec.cells.size());
  cvc::state *cells = rows > 0 ? rd.find(root, "cells") : nullptr;
  for (int r = 0; r < rows && cells; ++r) {
    cvc::state *row = rd.find(cells, std::to_string(r));
    for (int c = 0; c < cols && row; ++c) {
      cvc::state *cell = rd.find(row, std::to_string(c));
      if (!cell)
        continue;
      const std::size_t k = static_cast<std::size_t>(r) * cols + c;
      HeatmapCell &out = spec.cells[k];
      // Its value is `v`, or, with no children at all, the cell's own value.
      cvc::state *vn = rd.find(cell, "v");
      const double v = parse_number(vn                         ? vn->value()
                                    : cell->numChildren() == 0 ? cell->value()
                                                               : "");
      out.has_value = heatmap_color(w.heatmap.ramp, v, out.rgb);
      out.v = out.has_value ? v : 0.0;
      out.text = rd.text(cell, "text");
      out.mark = to_bool_int(rd.text(cell, "mark"), 0) != 0;
      cache.tips[k] = rd.text(cell, "tip");
    }
  }
  if (w.heatmap.row_labels)
    spec.row_labels = read_labels(rd, root, "row_labels", rows);
  if (w.heatmap.col_labels)
    spec.col_labels = read_labels(rd, root, "col_labels", cols);
  cache.live = rd.live;
}

HeatmapCellEvent cell_event(const HeatmapSpec &spec, int r, int c) {
  HeatmapCellEvent e;
  e.row = r;
  e.col = c;
  const HeatmapCell &cell = spec.cells[static_cast<std::size_t>(r) * spec.cols + c];
  e.has_value = cell.has_value;
  e.v = cell.v;
  e.text = cell.text;
  e.mark = cell.mark;
  if (static_cast<std::size_t>(r) < spec.row_labels.size())
    e.row_label = spec.row_labels[r];
  if (static_cast<std::size_t>(c) < spec.col_labels.size())
    e.col_label = spec.col_labels[c];
  return e;
}

} // namespace

bool heatmap_color(const std::vector<HeatmapStop> &ramp, double v, float rgb[3]) {
  if (!std::isfinite(v))
    return false;
  if (ramp.empty()) {
    rgb[0] = rgb[1] = rgb[2] = 0.5f;
    return true;
  }
  const HeatmapStop *a = &ramp.front(), *b = a;
  if (v >= ramp.back().at) {
    a = b = &ramp.back();
  } else if (v > ramp.front().at) {
    for (std::size_t i = 1; i < ramp.size(); ++i)
      if (v < ramp[i].at) {
        a = &ramp[i - 1];
        b = &ramp[i];
        break;
      }
  }
  const double span = b->at - a->at;
  const float t = span > 0.0 ? static_cast<float>((v - a->at) / span) : 0.0f;
  for (int i = 0; i < 3; ++i)
    rgb[i] = a->rgb[i] + (b->rgb[i] - a->rgb[i]) * t;
  return true;
}

bool parse_heatmap_color(const std::string &raw, float rgb[3]) {
  const std::string t = trim(raw);
  if (!t.empty() && t[0] == '#') {
    const std::string h = t.substr(1);
    if ((h.size() != 6 && h.size() != 3) ||
        !std::all_of(h.begin(), h.end(), [](unsigned char c) { return std::isxdigit(c) != 0; }))
      return false;
    const unsigned long n = std::strtoul(h.c_str(), nullptr, 16);
    if (h.size() == 6) {
      rgb[0] = static_cast<float>((n >> 16) & 0xff) / 255.0f;
      rgb[1] = static_cast<float>((n >> 8) & 0xff) / 255.0f;
      rgb[2] = static_cast<float>(n & 0xff) / 255.0f;
    } else {
      rgb[0] = static_cast<float>((n >> 8) & 0xf) / 15.0f;
      rgb[1] = static_cast<float>((n >> 4) & 0xf) / 15.0f;
      rgb[2] = static_cast<float>(n & 0xf) / 15.0f;
    }
    return true;
  }
  float out[3];
  int i = 0;
  for (std::size_t p = 0; p <= t.size(); ++i) {
    const std::size_t q = std::min(t.find(',', p), t.size());
    const double v = parse_number(t.substr(p, q - p));
    if (i >= 3 || !std::isfinite(v))
      return false;
    out[i] = static_cast<float>(std::clamp(v, 0.0, 1.0));
    p = q + 1;
  }
  if (i != 3)
    return false;
  std::copy(out, out + 3, rgb);
  return true;
}

std::string heatmap_tooltip_tokens(const std::string &tmpl, const HeatmapCellEvent &cell) {
  if (tmpl.find('{') == std::string::npos)
    return tmpl;
  std::string s = tmpl;
  s = replace_all(s, "{row_label}", cell.row_label);
  s = replace_all(s, "{col_label}", cell.col_label);
  s = replace_all(s, "{row}", std::to_string(cell.row));
  s = replace_all(s, "{col}", std::to_string(cell.col));
  s = replace_all(s, "{v}", cell.has_value ? short_number(cell.v, 6) : std::string());
  s = replace_all(s, "{text}", cell.text);
  s = replace_all(s, "{mark}", cell.mark ? "1" : "0");
  return s;
}

// The text heatmap (the default for every backend without one of its own).
HeatmapEvent Backend::heatmap(const char *label, const HeatmapSpec &spec) {
  if (label && *label)
    text_line(label);
  const auto cell_text = [](const HeatmapCell &c) {
    std::string t = !c.text.empty() ? c.text : c.has_value ? short_number(c.v, 4) : ".";
    return c.mark ? t + "*" : t;
  };
  // Each column as wide as its label and its widest cell; the row labels as wide as the widest.
  std::size_t lw = 0;
  for (const std::string &l : spec.row_labels)
    lw = std::max(lw, text_width(l));
  std::vector<std::size_t> cw(static_cast<std::size_t>(spec.cols), 1);
  for (int c = 0; c < spec.cols; ++c) {
    if (static_cast<std::size_t>(c) < spec.col_labels.size())
      cw[c] = std::max(cw[c], text_width(spec.col_labels[c]));
    for (int r = 0; r < spec.rows; ++r)
      cw[c] = std::max(cw[c], text_width(cell_text(spec.cells[r * spec.cols + c])));
  }
  const auto finish = [this](std::string line) {
    while (!line.empty() && line.back() == ' ')
      line.pop_back();
    text_line(line.c_str());
  };
  if (!spec.col_labels.empty()) {
    std::string line = std::string(lw, ' ');
    for (int c = 0; c < spec.cols; ++c)
      line += (lw > 0 || c > 0 ? " " : "") + pad_left(spec.col_labels[c], cw[c]);
    finish(line);
  }
  for (int r = 0; r < spec.rows; ++r) {
    std::string line = static_cast<std::size_t>(r) < spec.row_labels.size()
                           ? pad_right(spec.row_labels[r], lw)
                           : std::string(lw, ' ');
    for (int c = 0; c < spec.cols; ++c)
      line +=
          (lw > 0 || c > 0 ? " " : "") + pad_left(cell_text(spec.cells[r * spec.cols + c]), cw[c]);
    finish(line);
  }
  if (spec.legend != HeatmapLegend::Off && !spec.ramp.empty()) {
    std::string stops;
    for (const HeatmapStop &s : spec.ramp) {
      char hex[8];
      std::snprintf(hex, sizeof hex, "#%02x%02x%02x",
                    static_cast<int>(std::lround(std::clamp(s.rgb[0], 0.0f, 1.0f) * 255.0f)),
                    static_cast<int>(std::lround(std::clamp(s.rgb[1], 0.0f, 1.0f) * 255.0f)),
                    static_cast<int>(std::lround(std::clamp(s.rgb[2], 0.0f, 1.0f) * 255.0f)));
      stops += (stops.empty() ? "" : ", ") + short_number(s.at, 6) + " " + hex;
    }
    text_value("legend", stops);
  }
  return {};
}

bool emit_heatmap(HeatmapContext &ctx, const Widget &w) {
  const std::string base =
      w.bind.empty() ? std::string() : cvc::state::normalize_path(ctx.resolve(w.bind));
  HeatmapCache &cache = ctx.caches[base + "#" + w.label];
  cache.used = ctx.frame;

  // Read the subtree again only when it was written (the generation is taken before the read, so
  // a write during it is seen next frame), the ramp changed, or a link was followed last time.
  const std::uint64_t gen = base.empty() ? 0 : ctx.written(base);
  const bool labels_changed =
      w.heatmap.row_labels != cache.row_labels || w.heatmap.col_labels != cache.col_labels;
  if (!cache.built || cache.live || gen != cache.gen ||
      !same_ramp(cache.spec.ramp, w.heatmap.ramp) || labels_changed) {
    cache.row_labels = w.heatmap.row_labels;
    cache.col_labels = w.heatmap.col_labels;
    const std::uint64_t revision = cache.spec.revision + 1;
    if (base.empty()) {
      cache.spec = HeatmapSpec{};
      cache.tips.clear();
    } else {
      read_heatmap(ctx, w, base, cache);
    }
    cache.spec.revision = revision;
    cache.spec.ramp = w.heatmap.ramp; // the cells' colours and the legend's
    cache.gen = gen;
    cache.built = true;
  }

  HeatmapSpec &spec = cache.spec;
  spec.legend = w.heatmap.legend;
  spec.cell_size = w.heatmap.cell_size;
  spec.gap = w.heatmap.gap;
  std::copy(w.heatmap.mark_rgb, w.heatmap.mark_rgb + 3, spec.mark_rgb);
  spec.interactive = !w.pick.empty() || !w.on_click.empty();
  spec.sel_row = spec.sel_col = -1;
  const std::string pick = w.pick.empty() ? std::string() : ctx.resolve(w.pick);
  if (!pick.empty()) {
    const double r = parse_number(read_text(ctx.app, pick + ".row"));
    const double c = parse_number(read_text(ctx.app, pick + ".col"));
    if (r >= 0.0 && r < spec.rows && c >= 0.0 && c < spec.cols) {
      spec.sel_row = static_cast<int>(r);
      spec.sel_col = static_cast<int>(c);
    }
  }

  const HeatmapEvent e = ctx.backend.heatmap(w.label.c_str(), spec);
  const auto valid = [&](int r, int c) {
    return r >= 0 && r < spec.rows && c >= 0 && c < spec.cols;
  };
  if (e.hovered && valid(e.row, e.col)) {
    const HeatmapCellEvent cell = cell_event(spec, e.row, e.col);
    if (!w.on_hover.empty())
      ctx.enqueue_cell(w.on_hover, cell);
    // The document's tooltip, its tokens filled in (then computed, if it is an expression), else
    // the cell's own tip.
    std::string tip;
    if (!w.tooltip.empty()) {
      tip = heatmap_tooltip_tokens(w.tooltip, cell);
      if (is_expr(tip))
        tip = ctx.eval_or_literal(tip);
    } else {
      tip = cache.tips[static_cast<std::size_t>(e.row) * spec.cols + e.col];
    }
    if (!tip.empty())
      ctx.backend.set_tooltip(tip.c_str());
  }
  if (e.clicked && valid(e.click_row, e.click_col)) {
    const HeatmapCellEvent cell = cell_event(spec, e.click_row, e.click_col);
    if (!pick.empty()) {
      write<std::string>(ctx.app, pick + ".row", std::to_string(cell.row));
      write<std::string>(ctx.app, pick + ".col", std::to_string(cell.col));
      write<std::string>(ctx.app, pick, std::to_string(cell.row) + "," + std::to_string(cell.col));
      if (!w.on.empty()) // after the writes, so its handler reads the picked cell
        ctx.enqueue_cell(w.on, cell);
    }
    if (!w.on_click.empty())
      ctx.enqueue_cell(w.on_click, cell);
    return true;
  }
  return false;
}

void sweep_heatmap_caches(std::unordered_map<std::string, HeatmapCache> &caches,
                          std::uint64_t frame, std::uint64_t max_age) {
  for (auto it = caches.begin(); it != caches.end();)
    it = frame - it->second.used > max_age ? caches.erase(it) : std::next(it);
}

} // namespace ariadne
} // namespace cvc

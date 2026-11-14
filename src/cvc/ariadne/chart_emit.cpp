/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "chart_emit.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kMaxSeries = 64;   // a plot subtree's s.<i> read at most
constexpr std::size_t kMaxMarkers = 256; // and its m.<j>
constexpr int kSparkColumns = 32;        // the text chart's columns
// The text chart's levels, lowest first; a column with no finite value is a space.
constexpr char kSparkLevels[] = "_.-:=+*#";

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
  std::string t = trim(raw);
  if (!t.empty() && t[0] == '+')
    t.erase(0, 1);
  if (t.empty())
    return kNaN;
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  double v = kNaN;
  const std::from_chars_result r = std::from_chars(t.data(), t.data() + t.size(), v);
  return r.ec == std::errc() && r.ptr == t.data() + t.size() ? v : kNaN;
#else
  std::istringstream in(t);
  in.imbue(std::locale::classic());
  double v = kNaN;
  if (!(in >> v))
    return kNaN;
  in >> std::ws;
  return in.eof() ? v : kNaN;
#endif
}

// The shortest text that reads back as `d` (what a pick writes).
std::string format_number(double d) {
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[64];
  const std::to_chars_result r = std::to_chars(buf, buf + sizeof buf, d);
  if (r.ec == std::errc())
    return std::string(buf, r.ptr);
#endif
  for (int prec = 15; prec <= 17; ++prec) {
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os.precision(prec);
    os << d;
    if (prec == 17 || parse_number(os.str()) == d)
      return os.str();
  }
  return std::string();
}

// A number for a person to read (the text chart): four significant digits.
std::string short_number(double d) {
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os.precision(4);
  os << d;
  return os.str();
}

bool truthy(const std::string &s) { return to_bool_int(s, 0) != 0; }

// What a state path holds, read through link holes; "" when it has nothing.
std::string read_text(cvc::app &app, const std::string &path) {
  try {
    cvc::state *n = read_node(app, path);
    return n ? n->value() : std::string();
  } catch (const std::exception &) {
    return std::string();
  }
}

bool exists(cvc::app &app, const std::string &path) {
  try {
    return read_node(app, path) != nullptr;
  } catch (const std::exception &) {
    return false;
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

// Where a series comes from, and the text it is read from. A path holding a `y` child is a series
// subtree (x, y, lo, hi, label, style, as host.set_plot writes one); otherwise the path's value (or
// a computed expression's) is its y, and its x is 0, 1, 2, ...
struct SeriesText {
  std::string x, y, lo, hi, label, style;
  std::string joined() const {
    std::string j;
    j.reserve(x.size() + y.size() + lo.size() + hi.size() + label.size() + style.size() + 6);
    for (const std::string *s : {&x, &y, &lo, &hi, &label, &style}) {
      j += *s;
      j += '\x1f';
    }
    return j;
  }
};

SeriesText read_series(ChartContext &ctx, const std::string &bind, bool absolute) {
  SeriesText t;
  if (!absolute && is_expr(bind)) {
    t.y = ctx.eval_text(bind);
    return t;
  }
  const std::string path = absolute ? bind : ctx.resolve(bind);
  if (exists(ctx.app, path + ".y")) {
    t.x = read_text(ctx.app, path + ".x");
    t.y = read_text(ctx.app, path + ".y");
    t.lo = read_text(ctx.app, path + ".lo");
    t.hi = read_text(ctx.app, path + ".hi");
    t.label = read_text(ctx.app, path + ".label");
    t.style = read_text(ctx.app, path + ".style");
  } else {
    t.y = read_text(ctx.app, path);
  }
  return t;
}

// Parse a series from its text: x (0, 1, 2, ... when it has none), y, the band, and its style.
ChartSeries parse_series(ChartContext &ctx, const Widget &w, const SeriesText &t,
                         std::size_t index) {
  ChartSeries s;
  s.label = t.label;
  s.y = parse_chart_numbers(t.y);
  if (trim(t.x).empty()) {
    s.x.resize(s.y.size());
    for (std::size_t i = 0; i < s.x.size(); ++i)
      s.x[i] = static_cast<double>(i);
  } else {
    s.x = parse_chart_numbers(t.x);
  }
  const std::string name = "ari: chart '" + w.label + "' series " +
                           (t.label.empty() ? std::to_string(index) : "'" + t.label + "'");
  if (s.x.size() != s.y.size()) {
    ctx.warn_once(name + " has " + std::to_string(s.x.size()) + " x for " +
                  std::to_string(s.y.size()) + " y — the first of each drawn");
    const std::size_t n = std::min(s.x.size(), s.y.size());
    s.x.resize(n);
    s.y.resize(n);
  }
  for (std::size_t i = 0; i < s.x.size(); ++i)
    if (!std::isfinite(s.x[i]))
      s.y[i] = kNaN; // a point with no x is a gap, as one with no y is
  if (!trim(t.lo).empty() || !trim(t.hi).empty()) {
    s.lo = parse_chart_numbers(t.lo);
    s.hi = parse_chart_numbers(t.hi);
    if (s.lo.size() != s.y.size() || s.hi.size() != s.y.size()) {
      ctx.warn_once(name + " has a band of " + std::to_string(s.lo.size()) + " lo and " +
                    std::to_string(s.hi.size()) + " hi for " + std::to_string(s.y.size()) +
                    " y — no band drawn");
      s.lo.clear();
      s.hi.clear();
    }
  }
  for (const std::string &bad : parse_chart_style(t.style, s))
    ctx.warn_once(name + " style '" + bad +
                  "' is not a line (solid, dashed, dotted, points), c0..c7 or #rrggbb");
  return s;
}

// The series as the backend gets it: on a log axis a y <= 0 is a gap, and so is a band point
// whose hi is <= 0.
ChartSeries for_axis(ChartSeries s, bool log_y) {
  if (!log_y)
    return s;
  for (double &y : s.y)
    if (!(y > 0.0))
      y = kNaN;
  for (std::size_t i = 0; i < s.hi.size(); ++i)
    if (!(s.hi[i] > 0.0))
      s.lo[i] = s.hi[i] = kNaN;
  return s;
}

std::string read_markers(ChartContext &ctx, const std::string &path) {
  std::string j;
  for (std::size_t m = 0; m < kMaxMarkers; ++m) {
    const std::string p = path + ".m." + std::to_string(m);
    if (!exists(ctx.app, p))
      break;
    j += read_text(ctx.app, p + ".x") + '\x1f' + read_text(ctx.app, p + ".label") + '\x1f' +
         read_text(ctx.app, p + ".kind") + '\x1e';
  }
  return j;
}

std::vector<ChartMarker> parse_markers(ChartContext &ctx, const Widget &w,
                                       const std::string &path) {
  std::vector<ChartMarker> out;
  for (std::size_t m = 0; m < kMaxMarkers; ++m) {
    const std::string p = path + ".m." + std::to_string(m);
    if (!exists(ctx.app, p))
      break;
    ChartMarker mk;
    mk.x = parse_number(read_text(ctx.app, p + ".x"));
    mk.label = read_text(ctx.app, p + ".label");
    const std::string kind = trim(read_text(ctx.app, p + ".kind"));
    mk.kind = chart_marker_kind(kind);
    if (!kind.empty() && kind != chart_marker_kind_name(mk.kind))
      ctx.warn_once("ari: chart '" + w.label + "' marker kind '" + kind +
                    "' is not best, elbow, plateau or note — note");
    if (std::isfinite(mk.x))
      out.push_back(std::move(mk));
  }
  return out;
}

bool chart_log_y(ChartContext &ctx, const Widget &w) {
  const std::string &v = w.log_y;
  if (v.empty() || v == "0")
    return false;
  if (v == "1")
    return true;
  return truthy(is_expr(v) ? ctx.eval_text(v) : read_text(ctx.app, ctx.resolve(v)));
}

} // namespace

std::vector<double> parse_chart_numbers(const std::string &text) {
  std::vector<double> out;
  if (text.empty())
    return out;
  for (std::size_t i = 0; i <= text.size();) {
    const std::size_t j = std::min(text.find(',', i), text.size());
    out.push_back(parse_number(text.substr(i, j - i)));
    i = j + 1;
  }
  return out;
}

std::vector<std::string> parse_chart_style(const std::string &style, ChartSeries &s) {
  std::vector<std::string> unknown;
  std::string tok;
  const auto take = [&](const std::string &t) {
    if (t.empty())
      return;
    std::string l = t;
    std::transform(l.begin(), l.end(), l.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (l == "solid" || l == "line")
      s.line = ChartLine::Solid;
    else if (l == "dashed")
      s.line = ChartLine::Dashed;
    else if (l == "dotted")
      s.line = ChartLine::Dotted;
    else if (l == "points")
      s.line = ChartLine::Points;
    else if (l.size() == 2 && l[0] == 'c' && l[1] >= '0' && l[1] <= '7')
      s.color = l[1] - '0';
    else if (l.size() == 7 && l[0] == '#' &&
             std::all_of(l.begin() + 1, l.end(),
                         [](unsigned char c) { return std::isxdigit(c) != 0; })) {
      const unsigned long rgb = std::stoul(l.substr(1), nullptr, 16);
      s.has_rgb = true;
      s.rgb[0] = static_cast<float>((rgb >> 16) & 0xff) / 255.0f;
      s.rgb[1] = static_cast<float>((rgb >> 8) & 0xff) / 255.0f;
      s.rgb[2] = static_cast<float>(rgb & 0xff) / 255.0f;
    } else
      unknown.push_back(t);
  };
  for (char c : style) {
    if (c == ',' || std::isspace(static_cast<unsigned char>(c))) {
      take(tok);
      tok.clear();
    } else
      tok += c;
  }
  take(tok);
  return unknown;
}

const char *chart_marker_kind_name(ChartMarkerKind kind) {
  switch (kind) {
  case ChartMarkerKind::Best:
    return "best";
  case ChartMarkerKind::Elbow:
    return "elbow";
  case ChartMarkerKind::Plateau:
    return "plateau";
  case ChartMarkerKind::Note:
    break;
  }
  return "note";
}

ChartMarkerKind chart_marker_kind(const std::string &name) {
  if (name == "best")
    return ChartMarkerKind::Best;
  if (name == "elbow")
    return ChartMarkerKind::Elbow;
  if (name == "plateau")
    return ChartMarkerKind::Plateau;
  return ChartMarkerKind::Note;
}

ChartBounds chart_bounds(const ChartSpec &spec, const std::vector<char> &shown) {
  ChartBounds b;
  double x0 = std::numeric_limits<double>::infinity(), x1 = -x0, y0 = x0, y1 = -x0;
  const auto take_y = [&](double v) {
    if (!std::isfinite(v) || (spec.log_y && !(v > 0.0)))
      return;
    v = spec.log_y ? std::log10(v) : v;
    y0 = std::min(y0, v);
    y1 = std::max(y1, v);
  };
  for (std::size_t i = 0; i < spec.series.size(); ++i) {
    if (i < shown.size() && !shown[i])
      continue;
    const ChartSeries &s = spec.series[i];
    for (std::size_t k = 0; k < s.y.size() && k < s.x.size(); ++k) {
      if (!std::isfinite(s.x[k]) || !std::isfinite(s.y[k]))
        continue;
      x0 = std::min(x0, s.x[k]);
      x1 = std::max(x1, s.x[k]);
      take_y(s.y[k]);
      if (k < s.lo.size() && k < s.hi.size()) {
        take_y(s.lo[k]);
        take_y(s.hi[k]);
      }
    }
  }
  if (!(y0 <= y1))
    return b; // no finite y: nothing to show (the markers alone place no curve)
  for (const ChartMarker &m : spec.markers) {
    x0 = std::min(x0, m.x);
    x1 = std::max(x1, m.x);
  }
  b.ok = true;
  b.x0 = x0;
  b.x1 = x1;
  b.y0 = y0;
  b.y1 = y1;
  return b;
}

// The text chart (the default for every backend without one of its own: the terminals).
ChartEvent Backend::chart(const char *label, const ChartSpec &spec) {
  const ChartBounds b = chart_bounds(spec);
  std::string head = label ? label : "";
  std::string notes;
  if (spec.log_y)
    notes = "log y";
  if (b.ok) {
    if (!notes.empty())
      notes += "; ";
    notes += (spec.x_label.empty() ? std::string("x") : spec.x_label) + " " + short_number(b.x0) +
             ".." + short_number(b.x1);
  }
  if (!notes.empty())
    head += head.empty() ? notes : " (" + notes + ")";
  if (!head.empty())
    text_line(head.c_str());

  // The names padded to one width, so the sparklines line up in a monospaced terminal.
  const auto name_of = [&](std::size_t i) {
    return spec.series[i].label.empty() ? "series " + std::to_string(i) : spec.series[i].label;
  };
  std::size_t width = 0;
  for (std::size_t i = 0; i < spec.series.size(); ++i)
    width = std::max(width, name_of(i).size());
  for (std::size_t i = 0; i < spec.series.size(); ++i) {
    const ChartSeries &s = spec.series[i];
    const std::string name = name_of(i);
    // Each column holds the mean of the finite y (their logs, on a log axis) whose x falls in it.
    std::vector<double> sum(kSparkColumns, 0.0);
    std::vector<int> count(kSparkColumns, 0);
    double last = kNaN, lo = kNaN, hi = kNaN;
    for (std::size_t k = 0; k < s.y.size() && k < s.x.size(); ++k) {
      const double x = s.x[k], y = s.y[k];
      if (!std::isfinite(x) || !std::isfinite(y) || !b.ok)
        continue;
      last = y;
      lo = std::isnan(lo) ? y : std::min(lo, y);
      hi = std::isnan(hi) ? y : std::max(hi, y);
      const double f = b.x1 > b.x0 ? (x - b.x0) / (b.x1 - b.x0) : 1.0;
      const int c = std::clamp(static_cast<int>(f * kSparkColumns), 0, kSparkColumns - 1);
      sum[c] += spec.log_y ? std::log10(y) : y;
      ++count[c];
    }
    std::string line = std::string(width - name.size(), ' ') + "[";
    for (int c = 0; c < kSparkColumns; ++c) {
      if (count[c] == 0) {
        line += ' ';
        continue;
      }
      const double v = sum[c] / count[c];
      double f = b.y1 > b.y0 ? (v - b.y0) / (b.y1 - b.y0) : 0.5;
      f = std::clamp(f, 0.0, 1.0);
      line += kSparkLevels[static_cast<int>(f * 7.0 + 0.5)];
    }
    line += "]";
    if (std::isnan(last))
      line += " no data";
    else
      line += " " + short_number(last) + " (" + short_number(lo) + ".." + short_number(hi) + ")";
    text_value(name.c_str(), line);
  }
  for (const ChartMarker &m : spec.markers)
    text_value(chart_marker_kind_name(m.kind),
               "x=" + short_number(m.x) + (m.label.empty() ? "" : " " + m.label));
  if (spec.has_cursor)
    text_value("cursor", "x=" + short_number(spec.cursor));
  return {};
}

void emit_chart(ChartContext &ctx, const Widget &w) {
  // The instance: the paths it reads (resolved, so a module mounted twice has two) and its label.
  const std::string base = w.bind.empty() ? std::string() : ctx.resolve(w.bind);
  std::string key = base + "#" + w.label;
  for (const ChartSeriesBind &sb : w.chart_series)
    key += "#" + (is_expr(sb.bind) ? sb.bind : ctx.resolve(sb.bind));
  ChartCache &cache = ctx.caches[key];
  cache.used = ctx.frame;

  // Every source's text, this frame: the plot subtree's series, then the `series:` entries.
  std::vector<SeriesText> texts;
  if (!base.empty())
    for (std::size_t i = 0; i < kMaxSeries; ++i) {
      const std::string p = base + ".s." + std::to_string(i);
      if (!exists(ctx.app, p))
        break;
      texts.push_back(read_series(ctx, p, /*absolute=*/true));
    }
  for (const ChartSeriesBind &sb : w.chart_series) {
    SeriesText t = read_series(ctx, sb.bind, /*absolute=*/false);
    if (!sb.label.empty())
      t.label = sb.label;
    if (!sb.style.empty())
      t.style = sb.style;
    texts.push_back(std::move(t));
  }

  bool changed = !cache.built || cache.sources.size() != texts.size();
  cache.sources.resize(texts.size());
  for (std::size_t i = 0; i < texts.size(); ++i) {
    std::string j = texts[i].joined();
    if (cache.built && j == cache.sources[i].text)
      continue;
    cache.sources[i].text = std::move(j);
    cache.sources[i].series = parse_series(ctx, w, texts[i], i);
    changed = true;
  }
  if (!base.empty()) {
    std::string m = read_markers(ctx, base);
    if (!cache.built || m != cache.markers_text) {
      cache.markers_text = std::move(m);
      cache.markers = parse_markers(ctx, w, base);
      changed = true;
    }
  }
  const bool log_y = chart_log_y(ctx, w);
  if (changed || log_y != cache.log_y) {
    cache.spec.series.clear();
    for (const ChartCache::Source &src : cache.sources)
      cache.spec.series.push_back(for_axis(src.series, log_y));
    cache.spec.markers = cache.markers;
    cache.log_y = cache.spec.log_y = log_y;
    cache.built = true;
  }

  ChartSpec &spec = cache.spec;
  spec.legend = w.legend;
  spec.x_label = w.x_label;
  spec.height = static_cast<float>(w.plot_height);
  spec.pick = !w.pick.empty();
  spec.has_cursor = false;
  if (!w.cursor.empty()) {
    const double c = parse_number(is_expr(w.cursor) ? ctx.eval_text(w.cursor)
                                                    : read_text(ctx.app, ctx.resolve(w.cursor)));
    spec.has_cursor = std::isfinite(c);
    spec.cursor = spec.has_cursor ? c : 0.0;
  }

  const ChartEvent e = ctx.backend.chart(w.label.c_str(), spec);
  if (e.picked && !w.pick.empty() && std::isfinite(e.x)) {
    write<std::string>(ctx.app, ctx.resolve(w.pick), format_number(e.x));
    if (!w.on.empty()) // after the write, so its handler reads the picked x
      ctx.enqueue(w.on);
  }
}

void sweep_chart_caches(std::unordered_map<std::string, ChartCache> &caches, std::uint64_t frame,
                        std::uint64_t max_age) {
  for (auto it = caches.begin(); it != caches.end();)
    it = frame - it->second.used > max_age ? caches.erase(it) : std::next(it);
}

} // namespace ariadne
} // namespace cvc

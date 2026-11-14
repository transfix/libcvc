/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Runtime's half of Kind::Chart (private to the Ariadne core): read each series and marker of a
// chart from cvc::state, parse them (only when their text changes), hand the backend a ChartSpec,
// and write a pick back. Kept out of ariadne.cpp so the walk stays a dispatcher.

#ifndef CVC_ARIADNE_CHART_EMIT_H
#define CVC_ARIADNE_CHART_EMIT_H

#include <cstdint>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

// One chart instance's parse cache (keyed by the instance: its resolved bind, label and series
// binds). A series is re-parsed only when the text it was parsed from changes, and the spec handed
// to the backend is rebuilt only when a series, a marker or the log axis changed.
struct ChartCache {
  struct Source {
    std::string text; // what the series was parsed from, its fields joined
    ChartSeries series;
  };
  std::vector<Source> sources;
  std::string markers_text;
  std::vector<ChartMarker> markers;
  bool log_y = false;
  bool built = false;
  ChartSpec spec;         // what the backend is handed
  std::uint64_t used = 0; // the frame that last drew it (sweep_chart_caches)
};

struct ChartContext {
  cvc::app &app;
  Backend &backend;
  std::function<std::string(const std::string &)> resolve;   // bind -> state path (this scope)
  std::function<std::string(const std::string &)> eval_text; // a computed value's text
  std::function<void(const std::string &)> enqueue;          // fire the chart's `on:`
  std::function<void(const std::string &)> warn_once;        // a de-duplicated diagnostic
  std::unordered_map<std::string, ChartCache> &caches;
  std::uint64_t frame = 0;
};

// Emit one Kind::Chart widget.
void emit_chart(ChartContext &ctx, const Widget &w);

// Forget the caches of charts no frame drew for `max_age` frames (a closed window, a repeat's rows
// gone).
void sweep_chart_caches(std::unordered_map<std::string, ChartCache> &caches, std::uint64_t frame,
                        std::uint64_t max_age = 300);

// A series' `style:`: space- or comma-separated tokens, each a line (solid, dashed, dotted,
// points), a palette slot (c0..c7) or a colour (#rrggbb). Returns the tokens it did not know.
std::vector<std::string> parse_chart_style(const std::string &style, ChartSeries &s);

// Comma-separated numbers as doubles, oldest first: an empty or unreadable item is NaN, so a gap
// keeps its place. Locale-independent.
std::vector<double> parse_chart_numbers(const std::string &text);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_CHART_EMIT_H

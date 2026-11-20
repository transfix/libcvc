/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Runtime's half of Kind::Heatmap (private to the Ariadne core): read a heatmap's subtree from
// cvc::state when something in it was written, colour its cells from the ramp, hand the backend a
// HeatmapSpec, and turn what the backend reports (the hovered and the clicked cell) into the
// tooltip, the pick's writes and the handlers. Kept out of ariadne.cpp so the walk stays a
// dispatcher.

#ifndef CVC_ARIADNE_HEATMAP_EMIT_H
#define CVC_ARIADNE_HEATMAP_EMIT_H

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

// One heatmap instance's cache (keyed by its resolved bind and its label): the spec as last read,
// and the write generation it was read at. It is read again only when a write at, under or above
// the subtree moved that generation, when the ramp (spec.ramp, what the cells were coloured with)
// or the labels changed, or every frame (`live`) when a link was followed while reading it, since
// a write to a link's target is signalled under the target's path.
struct HeatmapCache {
  bool built = false;
  bool live = false;
  std::uint64_t gen = 0;
  bool row_labels = true, col_labels = true; // the `labels:` they were read with
  std::vector<std::string> tips;             // each cell's `tip`, beside spec.cells
  HeatmapSpec spec;
  std::uint64_t used = 0; // the frame that last drew it (sweep_heatmap_caches)
};

// The cell a handler is given (as `event`) and a tooltip's tokens name.
struct HeatmapCellEvent {
  int row = 0, col = 0;
  bool has_value = false;
  double v = 0.0;
  std::string text, row_label, col_label;
  bool mark = false;
};

struct HeatmapContext {
  cvc::app &app;
  Backend &backend;
  std::function<std::string(const std::string &)> resolve;         // bind -> state path
  std::function<std::string(const std::string &)> eval_or_literal; // a computed tooltip's text
  std::function<std::uint64_t(const std::string &)> written;       // a subtree's write generation
  std::function<void(const std::string &, const HeatmapCellEvent &)> enqueue_cell; // a handler
  std::function<void(const std::string &)> warn_once; // a de-duplicated diagnostic
  std::unordered_map<std::string, HeatmapCache> &caches;
  std::uint64_t frame = 0;
};

// Emit one Kind::Heatmap widget. True when a cell was clicked this frame.
bool emit_heatmap(HeatmapContext &ctx, const Widget &w);

// Forget the caches of heatmaps no frame drew for `max_age` frames.
void sweep_heatmap_caches(std::unordered_map<std::string, HeatmapCache> &caches,
                          std::uint64_t frame, std::uint64_t max_age = 300);

// A tooltip template with the cell's tokens filled in: {row}, {col}, {v} (six significant digits;
// "" for no value), {text}, {row_label}, {col_label} and {mark} (1 or 0).
std::string heatmap_tooltip_tokens(const std::string &tmpl, const HeatmapCellEvent &cell);

// A colour as a document writes one: "#rrggbb", "#rgb", or "r,g,b" with each 0..1. False when it
// is none of them.
bool parse_heatmap_color(const std::string &text, float rgb[3]);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_HEATMAP_EMIT_H

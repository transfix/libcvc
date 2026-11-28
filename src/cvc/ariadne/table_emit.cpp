/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "table_emit.h"

#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <string>

namespace cvc {
namespace ariadne {

int table_sort_dir(const std::string &sort, const std::string &key) {
  if (key.empty())
    return 0;
  if (sort == key)
    return 1;
  if (sort.size() == key.size() + 1 && sort.front() == '-' && sort.compare(1, key.size(), key) == 0)
    return -1;
  return 0;
}

std::string table_sort_after_click(const std::string &sort, const std::string &key) {
  return table_sort_dir(sort, key) == 1 ? "-" + key : key;
}

std::string table_header_text(const std::string &label, int dir, bool unicode) {
  if (dir > 0)
    return label + (unicode ? " \xE2\x96\xB2" : " ^"); // U+25B2
  if (dir < 0)
    return label + (unicode ? " \xE2\x96\xBC" : " v"); // U+25BC
  return label;
}

// The default header: its text as a line, and the click the backend reports on it.
bool Backend::table_header(const std::string &label, int dir, bool sortable) {
  text_line(table_header_text(label, dir, unicode_glyphs()).c_str());
  return sortable && item_clicked();
}

bool emit_table_header(TableHeaderContext &ctx, const Widget &w) {
  if (!w.table)
    return false;
  bool changed = false;
  for (const TableColumn &c : w.table->columns) {
    ctx.backend.grid_next_cell();
    const bool sortable = !c.sort.empty();
    const int dir = table_sort_dir(ctx.sort, c.sort);
    if (!ctx.backend.table_header(c.label, dir, sortable) || !sortable)
      continue;
    ctx.sort = table_sort_after_click(ctx.sort, c.sort);
    write<std::string>(ctx.app, ctx.sort_path, ctx.sort);
    changed = true;
    if (!w.on.empty() && ctx.enqueue) // after the write, so the handler sees the new sort
      ctx.enqueue(w.on);
  }
  return changed;
}

} // namespace ariadne
} // namespace cvc

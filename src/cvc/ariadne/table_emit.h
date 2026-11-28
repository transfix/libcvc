/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Runtime's half of Kind::Table (private to the Ariadne core): the sort value's format
// ("name", "-name", ""), what a click on a header makes of it, and the header row. The rows are
// the walk's (ariadne.cpp, WalkFrame::Op::TableRows), since their cells are widgets it draws.

#ifndef CVC_ARIADNE_TABLE_EMIT_H
#define CVC_ARIADNE_TABLE_EMIT_H

#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <functional>
#include <string>

namespace cvc {
class app;
namespace ariadne {

// Which way the sort value `sort` sorts by `key`: 1 ascending ("key"), -1 descending ("-key"), 0
// not by it (another key, none, or an empty `key`).
int table_sort_dir(const std::string &sort, const std::string &key);

// The sort after a click on the header of the column that sorts by `key`: descending when it was
// that column ascending, else that column ascending.
std::string table_sort_after_click(const std::string &sort, const std::string &key);

struct TableHeaderContext {
  cvc::app &app;
  Backend &backend;
  std::string sort_path; // the sort's state path (the table's bind, resolved)
  std::string sort;      // its value this frame
  std::function<void(const std::string &)> enqueue; // the table's on:, after a change
};

// The header row of table `w`: a cell per column (Backend::grid_next_cell, then table_header),
// inside the grid the caller opened. A click on a sortable header writes the next sort to
// ctx.sort_path (and ctx.sort), then enqueues `w.on`. True when a click changed the sort.
bool emit_table_header(TableHeaderContext &ctx, const Widget &w);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_TABLE_EMIT_H

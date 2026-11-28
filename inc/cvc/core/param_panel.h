/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A params struct's Ariadne panel, from its cvc::param_table (cvc/core/param_table.h): one more
// generator beside the state keys, CLI, pycvc and DSL ones, so a port's window starts from its
// table (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §7.5, PP1).
//
//   cvc::param_panel_spec s;
//   s.stem = "skeleton";                                      // skeleton_panel.ari
//   s.keys = "graphics.root.children.mesh.mesh_ops.skeleton"; // where the fields sit by default
//   s.title = "Skeleton";
//   std::string ari = cvc::param_panel_ari<cvc::skeleton_params>(s);
//
// The text is a `units:` fragment with two units:
//   <stem>_params  the widgets alone (a group), args { keys }: one widget per field, in table
//                  order, bound to "{keys}.<field>", the key params_to_state / params_from_state
//                  read and seed;
//   <stem>_panel   a window holding them, args { keys, title, id }, with a `before` slot above the
//                  widgets and an `after` slot below, where a port's window puts its run and stop
//                  buttons and its result picker;
//   <stem>_<group>_params
//                  for each group of dotted fields (gdtv.lambda, gdtv.iterations: the group gdtv),
//                  those fields' widgets alone, args { keys }, so a window can show one group at a
//                  time (a `visible_when` on a group around the include).
//
// The widget a field gets (param_panel_widget):
//   bool    checkbox, its default as def;
//   enum    combo over the enumerators (their names in words as the options, the names
//           themselves as the stored values);
//   string  input_text;
//   list    input_text, in param_table.h's list form ([1, 2.5]);
//   int     slider_int over the table's range when it has both ends and spans at most 1000;
//           otherwise input_int, clamped to whichever ends the table has;
//   number  slider_float. Over the table's range when it has both ends (log when the low end is
//           above 0 and the high end at least 1000 times it). A table with an open end (no bound,
//           or the 1e300 stand-in) gives no range to draw, so the generator makes one up from the
//           default and says so in the tooltip; the schema has no input_float until SDK-5.
//           Each slider takes `fmt: "%g"`; the schema has no step. A number whose default is
//           infinite or NaN (a threshold that is off) is an input_text instead: no slider holds
//           it, and the widget seeds the key with its default.
// A slider whose range is the table's own takes `range: key`, so a node that writes the key's
// "min" and "max" properties (as params_describe_state does, or from its data) moves it. Every
// widget's tooltip starts with the field's name, then its doc, unit and range.
//
// The shipped panels are these texts checked in under src/cvc/ariadne/components/params/
// (cvc://components/params/<stem>_panel.ari), with params/INDEX.ari from param_panels_index();
// src/cvc/tests/param_panel_test.cpp lists the tables and fails when a file is not current.
// Everything here holds no state of its own.

#ifndef CVC_CORE_PARAM_PANEL_H
#define CVC_CORE_PARAM_PANEL_H

#include <cvc/core/param_table.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

// What a panel is called, where its keys sit by default, and what its comments say.
struct param_panel_spec {
  std::string stem;   // file <stem>_panel.ari; units <stem>_params and <stem>_panel
  std::string keys;   // the default key root (the units' `keys` arg)
  std::string title;  // the window's default title
  std::string table;  // the struct, for the comments ("cvc::skeleton_params")
  std::string header; // where it is declared ("inc/cvc/geometry/skeleton_params.h")
  std::string where;  // where its keys live on a node, in a few words; "" for none yet
};

// The widget a field gets.
struct param_widget {
  std::string type;  // checkbox | combo | input_text | slider_int | input_int | slider_float
  std::string label; // the field's name in words: '.' and '_' as spaces, the first letter upper
  double lo = std::numeric_limits<double>::quiet_NaN(); // NaN: none
  double hi = std::numeric_limits<double>::quiet_NaN();
  bool range_from_key = false; // the slider's range follows the key's "min" / "max" properties
  bool guessed_range = false;  // a slider_float over a range the table does not give
  bool log = false;
  std::vector<std::string> options; // combo: what it shows
  std::vector<std::string> values;  // combo: what it stores
  std::string def;                  // the default, as the widget's def: takes it ("" none)
  std::string tooltip;
};

// A panel's spec with its table's fields.
struct param_panel_table {
  param_panel_spec spec;
  std::vector<param_field_info> fields;
};

template <class P> param_panel_table param_panel_of(param_panel_spec spec) {
  return {std::move(spec), param_field_infos<P>()};
}

// The widget for `f`, as the top of this file says.
param_widget param_panel_widget(const param_field_info &f);

// The groups of dotted fields, in table order: "gdtv" for gdtv.lambda. Each has a unit of its own.
std::vector<std::string> param_panel_groups(const std::vector<param_field_info> &fields);

// The panel's .ari text for a table's fields.
std::string param_panel_ari(const param_panel_spec &spec,
                            const std::vector<param_field_info> &fields);

template <class P> std::string param_panel_ari(const param_panel_spec &spec) {
  return param_panel_ari(spec, param_field_infos<P>());
}

// The params/ collection's INDEX.ari: an entry for each panel's units.
std::string param_panels_index(const std::vector<param_panel_table> &panels);

} // namespace cvc

#endif // CVC_CORE_PARAM_PANEL_H

/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A params struct's Ariadne panel from its table's fields. See cvc/core/param_panel.h.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/param_panel.h>
#include <limits>
#include <string>
#include <vector>

namespace cvc {

namespace {

// A bound the table really gives: finite, and not the 1e300 that stands in for none.
constexpr double kStandIn = 1e300;

bool real_bound(double v) { return std::isfinite(v) && std::fabs(v) < kStandIn; }

// A bound the table writes as a "min" / "max" property that is a stand-in, not a range.
bool stand_in(double v) { return std::isfinite(v) && std::fabs(v) >= kStandIn; }

// A number as text, the same on every platform, so a panel generated anywhere matches the checked-in
// file: an integer as an integer, anything else the shortest of %.15g, %.16g and %.17g that reads
// back as the same double (%g choosing fixed or exponent form). Not format_double, which is the
// shortest form where floating to_chars exists ("1e+05") and %g's where it does not ("100000").
std::string num(double v) {
  if (!std::isfinite(v))
    return param_detail::format_double(v);
  if (v == std::floor(v) && std::fabs(v) < 1e15)
    return std::to_string(static_cast<long long>(v));
  std::string s;
  for (int prec = 15; prec <= 17; ++prec) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*g", prec, v);
    s = b;
    for (char &c : s)
      if (c == ',')
        c = '.'; // a decimal-comma locale
    double back = 0.0;
    if (param_detail::parse_double(s, back) && back == v)
      break;
  }
  return s;
}

// `v` to 6 significant digits.
double round6(double v) {
  char b[64];
  std::snprintf(b, sizeof b, "%.6g", v);
  std::string s = b;
  for (char &c : s)
    if (c == ',')
      c = '.';
  double out = v;
  param_detail::parse_double(s, out);
  return out;
}

// A number field's default in num()'s form.
std::string num_text(const std::string &text) {
  double d = 0.0;
  return param_detail::parse_double(text, d) ? num(d) : text;
}

std::string int_text(double v) { return std::to_string(static_cast<long long>(v)); }

// The field's name in words: '.' and '_' as spaces, the first letter upper.
std::string words(const std::string &name) {
  std::string s = name;
  for (char &c : s)
    if (c == '.' || c == '_')
      c = ' ';
  if (!s.empty() && s[0] >= 'a' && s[0] <= 'z')
    s[0] = static_cast<char>(s[0] - 'a' + 'A');
  return s;
}

// A YAML double-quoted scalar.
std::string quoted(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += (c == '\n' || c == '\r' || c == '\t') ? ' ' : c;
  }
  return out + '"';
}

// A flow list of quoted scalars.
std::string quoted_list(const std::vector<std::string> &items) {
  std::string out = "[";
  for (std::size_t i = 0; i < items.size(); ++i)
    out += (i ? ", " : "") + quoted(items[i]);
  return out + "]";
}

// The field's range as the docs table writes it ("0 .. 1", "0 .. "), or "".
std::string range_text(const param_field_info &f) {
  if (std::isnan(f.min) && std::isnan(f.max))
    return "";
  return (std::isnan(f.min) ? std::string() : num(f.min)) + " .. " +
         (std::isnan(f.max) ? std::string() : num(f.max));
}

double parse_or_nan(const std::string &text) {
  double d = 0.0;
  return param_detail::parse_double(text, d) ? d : std::numeric_limits<double>::quiet_NaN();
}

} // namespace

param_widget param_panel_widget(const param_field_info &f) {
  param_widget w;
  w.label = words(f.name);

  // The tooltip: the name first (so it never reads as an expression), then the doc, unit, range.
  std::string tip = f.name;
  if (!f.doc.empty())
    tip += ": " + f.doc;
  std::string extra = f.unit;
  const std::string range = range_text(f);
  if (!range.empty())
    extra += (extra.empty() ? "" : "; ") + range;
  if (!extra.empty())
    tip += " (" + extra + ")";

  if (f.kind == "bool") {
    w.type = "checkbox";
    w.def = f.default_text == "1" ? "true" : "false";
  } else if (f.kind == "enum") {
    w.type = "combo";
    for (const std::string &c : f.choices) {
      w.options.push_back(words(c));
      w.values.push_back(c);
    }
    w.def = f.default_text;
  } else if (f.kind == "string") {
    w.type = "input_text";
    w.def = f.default_text;
  } else if (f.kind == "list") {
    w.type = "input_text";
    w.def = f.default_text;
    tip += "; a list, as [1, 2.5]";
    if (!f.choices.empty()) {
      tip += " of";
      for (const std::string &c : f.choices)
        tip += " " + c;
    }
  } else if (f.kind == "int") {
    constexpr double imin = static_cast<double>(std::numeric_limits<int>::min());
    constexpr double imax = static_cast<double>(std::numeric_limits<int>::max());
    const bool has_lo = std::isfinite(f.min), has_hi = std::isfinite(f.max);
    const double lo = has_lo ? std::clamp(f.min, imin, imax) : imin;
    const double hi = has_hi ? std::clamp(f.max, imin, imax) : imax;
    if (has_lo && has_hi && hi - lo <= 1000.0) {
      w.type = "slider_int";
      w.range_from_key = true;
    } else {
      w.type = "input_int";
    }
    if (has_lo || has_hi) {
      w.lo = lo;
      w.hi = hi;
    }
    w.def = f.default_text;
  } else if (!std::isfinite(parse_or_nan(f.default_text))) {
    // A number whose default is infinite or NaN (a threshold that is off): no slider holds it, and
    // the widget seeds its key with its default, so the field takes the value as text.
    w.type = "input_text";
    w.def = f.default_text;
    tip += "; a number, or " + f.default_text;
  } else { // number
    w.type = "slider_float";
    const double d = parse_or_nan(f.default_text);
    if (real_bound(f.min) && real_bound(f.max) && f.min < f.max) {
      w.lo = f.min;
      w.hi = f.max;
      w.log = f.min > 0.0 && f.max >= 1000.0 * f.min;
    } else {
      // An open end: a span from the default (ten times it, at least 1) past the end the table
      // gives, or around 0. A guess, so to 6 significant digits (14.4, not 14.399999999999999).
      const double m = round6(std::max(10.0 * std::fabs(d), 1.0));
      double lo = real_bound(f.min) ? f.min : -m;
      double hi = real_bound(f.max) ? f.max : (lo < m ? m : lo + m);
      if (!real_bound(f.min) && real_bound(f.max) && lo >= hi)
        lo = hi - m;
      w.lo = lo;
      w.hi = hi;
      w.guessed_range = true;
      tip += "; the table sets no " +
             std::string(real_bound(f.min) || real_bound(f.max) ? "other end" : "range") +
             ", so the slider's is a guess";
    }
    // range: key reads the properties the table writes, unless one of them is a stand-in.
    w.range_from_key = !stand_in(f.min) && !stand_in(f.max);
    w.def = num_text(f.default_text);
  }
  w.tooltip = tip;
  return w;
}

namespace {

// The group a dotted field is in ("gdtv" for gdtv.lambda), or "".
std::string group_of(const std::string &name) {
  const std::size_t dot = name.find('.');
  return dot == std::string::npos ? std::string() : name.substr(0, dot);
}

// A unit of `fields`' widgets, a group, args { keys }.
std::string widgets_unit(const std::string &name, const std::string &keys,
                         const std::vector<param_field_info> &fields) {
  std::string o;
  o += "  " + name + ":\n";
  o += "    args: { keys: " + quoted(keys) + " }\n";
  o += "    type: group\n";
  if (fields.empty())
    return o + "    children: []\n";
  o += "    children:\n";
  for (const param_field_info &f : fields) {
    const param_widget w = param_panel_widget(f);
    o += "      - " + w.type + ": " + quoted(w.label) + "\n";
    o += "        bind: " + quoted("{keys}." + f.name) + "\n";
    if (w.type == "combo") {
      o += "        options: " + quoted_list(w.options) + "\n";
      o += "        values: " + quoted_list(w.values) + "\n";
    }
    if (!std::isnan(w.lo)) {
      const bool ints = w.type == "slider_int" || w.type == "input_int";
      o += "        lo: " + (ints ? int_text(w.lo) : num(w.lo)) + "\n";
      o += "        hi: " + (ints ? int_text(w.hi) : num(w.hi)) + "\n";
    }
    if (w.log)
      o += "        log: true\n";
    if (w.range_from_key)
      o += "        range: key\n";
    if (w.type == "slider_float")
      o += "        fmt: \"%g\"\n";
    if (!w.def.empty() || w.type == "input_text")
      o += "        def: " + (w.type == "input_text" || w.type == "combo" ? quoted(w.def) : w.def) +
           "\n";
    o += "        tooltip: " + quoted(w.tooltip) + "\n";
  }
  return o;
}

} // namespace

std::vector<std::string> param_panel_groups(const std::vector<param_field_info> &fields) {
  std::vector<std::string> groups;
  for (const param_field_info &f : fields) {
    const std::string g = group_of(f.name);
    if (!g.empty() && std::find(groups.begin(), groups.end(), g) == groups.end())
      groups.push_back(g);
  }
  return groups;
}

std::string param_panel_ari(const param_panel_spec &spec,
                            const std::vector<param_field_info> &fields) {
  const std::string group = spec.stem + "_params";
  const std::string window = spec.stem + "_panel";
  std::string o;
  o += "# " + window + ".ari — GENERATED from cvc::param_table<" + spec.table + ">";
  o += spec.header.empty() ? std::string("\n") : " (" + spec.header + ")\n";
  o += "# by cvc::param_panel_ari (inc/cvc/core/param_panel.h). Do not edit: "
       "src/cvc/tests/param_panel_test.cpp\n"
       "# fails when it is not current, and writes it again when run with "
       "CVC_WRITE_PARAM_PANELS=1.\n#\n";
  o += "# One widget per field, in table order, bound to <keys>.<field>: the keys the table's\n"
       "# params_to_state and params_from_state read and seed (docs/PORT_PARAMETERS.md).\n";
  if (!spec.where.empty())
    o += "# On a node: " + spec.where + ".\n";
  o += "#\n# Units:\n";
  o += "#   " + window + "   a window: args keys, title, id; slots before and after the widgets\n";
  o += "#   " + group + "  the widgets alone, to include in a window of your own: args keys\n";
  const std::vector<std::string> groups = param_panel_groups(fields);
  for (const std::string &g : groups)
    o += "#   " + spec.stem + "_" + g + "_params  the " + g +
         ".* fields' widgets alone: args keys\n";
  o += "#\n";
  o += "# Args (every one has a default, so a bare include works):\n";
  o += "#   keys   the key root the fields sit under (default " + spec.keys + ")\n";
  o += "#   title  the window's title (default " + spec.title + ")\n";
  o += "#   id     the instance id: the window's geometry persists under tree.<id> (default " +
       window + ")\n#\n";
  o += "# Use it:\n";
  o += "#     import: cvc://components/params/" + window + ".ari\n";
  o += "#     windows:\n";
  o += "#       - include: " + window + "\n";
  o += "#         args: { keys: " + spec.keys + " }\n";
  o += "#         slots:\n";
  o += "#           after:\n";
  o += "#             - button: Run\n";
  o += "#               on: ...\n\n";

  o += "units:\n";
  o += widgets_unit(group, spec.keys, fields);
  for (const std::string &g : groups) {
    std::vector<param_field_info> in;
    for (const param_field_info &f : fields)
      if (group_of(f.name) == g)
        in.push_back(f);
    o += "\n" + widgets_unit(spec.stem + "_" + g + "_params", spec.keys, in);
  }
  o += "\n";
  o += "  " + window + ":\n";
  o += "    args: { keys: " + quoted(spec.keys) + ", title: " + quoted(spec.title) +
       ", id: " + window + " }\n";
  o += "    window: \"{title}\"\n";
  o += "    id: \"{id}\"\n";
  o += "    children:\n";
  o += "      - slot: before\n";
  o += "      - include: " + group + "\n";
  o += "        args: { keys: \"{keys}\" }\n";
  o += "      - slot: after\n";
  return o;
}

std::string param_panels_index(const std::vector<param_panel_table> &panels) {
  std::string o;
  o += "# INDEX.ari — the params/ collection: a panel for each parameter table, GENERATED by\n"
       "# cvc::param_panels_index (inc/cvc/core/param_panel.h). Do not edit:\n"
       "# src/cvc/tests/param_panel_test.cpp fails when it is not current, and writes it again "
       "when run\n"
       "# with CVC_WRITE_PARAM_PANELS=1.\n#\n"
       "# Each <stem>_panel.ari has a window unit, <stem>_panel, and the same widgets with no "
       "window,\n"
       "# <stem>_params, both taking the key root the table's fields sit under as `keys`, and a\n"
       "# <stem>_<group>_params unit for each group of dotted fields. A port's window includes one "
       "and\n"
       "# adds its run and stop buttons and its result picker.\n\n";
  o += "meta:\n";
  o += "  name: Parameter panels\n";
  o += "  description: A panel for each cvc::param_table, generated from the table "
       "(import via cvc://components/params/…).\n";
  o += "  min_libcvc: 4.0.10\n\n";
  o += "catalog:\n";
  for (const param_panel_table &p : panels) {
    const param_panel_spec &s = p.spec;
    const std::string window = s.stem + "_panel";
    const std::string group = s.stem + "_params";
    const std::string binds = quoted("<prefix>.{keys}.*");
    o += "  - component: " + window + "\n";
    o += "    file: " + window + ".ari\n";
    o += "    unit: " + window + "\n";
    o += "    nodes: []\n";
    o += "    binds: " + binds + "\n";
    o += "    for: " +
         quoted(s.table + " as a window" + (s.where.empty() ? "" : "; on a node: " + s.where)) +
         "\n";
    o += "    args: { keys: " + quoted(s.keys) + ", title: " + quoted(s.title) + ", id: " + window +
         " }\n";
    o += "  - component: " + group + "\n";
    o += "    file: " + window + ".ari\n";
    o += "    unit: " + group + "\n";
    o += "    nodes: []\n";
    o += "    binds: " + binds + "\n";
    o += "    for: " + quoted(s.table + "'s widgets with no window: include it in a window") + "\n";
    o += "    args: { keys: " + quoted(s.keys) + " }\n";
    for (const std::string &g : param_panel_groups(p.fields)) {
      const std::string unit = s.stem + "_" + g + "_params";
      o += "  - component: " + unit + "\n";
      o += "    file: " + window + ".ari\n";
      o += "    unit: " + unit + "\n";
      o += "    nodes: []\n";
      o += "    binds: " + quoted("<prefix>.{keys}." + g + ".*") + "\n";
      o += "    for: " + quoted(s.table + "'s " + g + ".* widgets alone") + "\n";
      o += "    args: { keys: " + quoted(s.keys) + " }\n";
    }
  }
  return o;
}

} // namespace cvc

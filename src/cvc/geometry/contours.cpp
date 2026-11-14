/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::contour_stack, its checks, and reading and writing it through the app's handlers. See
// cvc/geometry/contours.h. The formats themselves are in contour_formats.cpp.

#include "contour_formats.h"

#include <algorithm>
#include <boost/regex.hpp>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/contours.h>
#include <set>
#include <sstream>
#include <utility>

namespace cvc {

namespace {

// The app's contour handlers: app.slot<T>() makes one per app, with the built-in formats, the
// first time it is asked (§1.3: no process-wide table).
struct contour_io_slot {
  static constexpr const char *app_slot = "cvc.geometry.contour_handlers";
  file_handler_registry<contour_file_io> handlers;
  contour_io_slot() { contour_formats::register_builtin(handlers); }
};

bool same_z(double a, double b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? sep : "") + v[i];
  return out;
}

// The patterns as regular expressions (boost::regex, whose few statics libcvc already has; a
// libstdc++ std::regex would add its own writable tables to the scan, §1.3); a pattern that isn't
// one adds a problem (and is skipped).
std::vector<boost::regex> compile(const char *field, const std::vector<std::string> &patterns,
                                  std::vector<std::string> *problems) {
  std::vector<boost::regex> out;
  for (std::size_t i = 0; i < patterns.size(); ++i) {
    try {
      out.emplace_back(patterns[i], boost::regex::perl);
    } catch (const boost::regex_error &e) {
      if (problems)
        problems->push_back(std::string(field) + "[" + std::to_string(i) + "]: '" + patterns[i] +
                            "' is not a regular expression (" + e.what() + ")");
    }
  }
  return out;
}

bool any_match(const std::vector<boost::regex> &res, const std::string &name) {
  for (const boost::regex &r : res)
    if (boost::regex_match(name, r))
      return true;
  return false;
}

// Keep what `p` selects (contours.h: contour_read_params).
void select(contour_stack &s, const contour_read_params &p) {
  const std::vector<boost::regex> inc = compile("include", p.include, nullptr);
  const std::vector<boost::regex> exc = compile("exclude", p.exclude, nullptr);
  const auto keep = [&](const std::string &name) {
    return (inc.empty() || any_match(inc, name)) && !any_match(exc, name);
  };
  for (auto it = s.slices.begin(); it != s.slices.end();) {
    if ((p.first >= 0 && it->first < p.first) || (p.last >= 0 && it->first > p.last)) {
      it = s.slices.erase(it);
      continue;
    }
    auto &comps = it->second.components;
    for (auto c = comps.begin(); c != comps.end();) {
      if (!keep(c->first)) {
        c = comps.erase(c);
        continue;
      }
      auto &polys = c->second;
      polys.erase(std::remove_if(polys.begin(), polys.end(),
                                 [&](const contour_polygon &poly) {
                                   return poly.size() < static_cast<std::size_t>(p.min_points);
                                 }),
                  polys.end());
      c = polys.empty() ? comps.erase(c) : std::next(c);
    }
    ++it;
  }
  s.components.erase(std::remove_if(s.components.begin(), s.components.end(),
                                    [&](const contour_component &c) { return !keep(c.name); }),
                     s.components.end());
  if (p.z_spacing > 0)
    s.z_spacing = p.z_spacing;
}

} // namespace

// ---- the types ----

double contour_polygon::signed_area() const {
  const std::size_t n = size();
  double a = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    a += xy[2 * i] * xy[2 * j + 1] - xy[2 * j] * xy[2 * i + 1];
  }
  return 0.5 * a;
}

bool contour_slice::operator==(const contour_slice &o) const {
  return components == o.components && same_z(z, o.z);
}

double contour_stack::slice_z(int index) const {
  auto it = slices.find(index);
  if (it != slices.end() && !std::isnan(it->second.z))
    return it->second.z;
  return z_origin + static_cast<double>(index) * z_spacing;
}

void contour_stack::add(int index, const std::string &component, const std::vector<double> &xy) {
  if (component.empty())
    throw std::invalid_argument("contour_stack::add: the component needs a name");
  if (xy.size() % 2 != 0)
    throw std::invalid_argument("contour_stack::add: " + std::to_string(xy.size()) +
                                " coordinates, not x, y pairs");
  if (component_index(component) < 0) {
    contour_component c;
    c.name = component;
    components.push_back(std::move(c));
  }
  contour_polygon poly;
  poly.xy = xy;
  slices[index].components[component].push_back(std::move(poly));
}

void contour_stack::set_component(const contour_component &c) {
  if (c.name.empty())
    throw std::invalid_argument("contour_stack::set_component: the component needs a name");
  const int i = component_index(c.name);
  if (i < 0)
    components.push_back(c);
  else
    components[static_cast<std::size_t>(i)] = c;
}

int contour_stack::component_index(const std::string &name) const {
  for (std::size_t i = 0; i < components.size(); ++i)
    if (components[i].name == name)
      return static_cast<int>(i);
  return -1;
}

std::vector<contour_polygon> contour_stack::polygons(int index,
                                                     const std::string &component) const {
  auto s = slices.find(index);
  if (s == slices.end())
    return {};
  auto c = s->second.components.find(component);
  return c == s->second.components.end() ? std::vector<contour_polygon>() : c->second;
}

std::vector<int> contour_stack::indices() const {
  std::vector<int> out;
  out.reserve(slices.size());
  for (const auto &kv : slices)
    out.push_back(kv.first);
  return out;
}

std::vector<std::string> contour_stack::component_names() const {
  std::vector<std::string> out;
  out.reserve(components.size());
  for (const contour_component &c : components)
    out.push_back(c.name);
  return out;
}

std::size_t contour_stack::polygon_count() const {
  std::size_t n = 0;
  for (const auto &s : slices)
    for (const auto &c : s.second.components)
      n += c.second.size();
  return n;
}

std::size_t contour_stack::vertex_count() const {
  std::size_t n = 0;
  for (const auto &s : slices)
    for (const auto &c : s.second.components)
      for (const contour_polygon &p : c.second)
        n += p.size();
  return n;
}

bool contour_stack::operator==(const contour_stack &o) const {
  return slices == o.slices && z_spacing == o.z_spacing && z_origin == o.z_origin &&
         components == o.components;
}

std::vector<std::string> contour_problems(const contour_stack &s) {
  std::vector<std::string> problems;
  if (!std::isfinite(s.z_spacing) || s.z_spacing <= 0)
    problems.push_back("z_spacing = " + param_detail::format_double(s.z_spacing) +
                       " is not a positive number");
  if (!std::isfinite(s.z_origin))
    problems.push_back("z_origin = " + param_detail::format_double(s.z_origin) +
                       " is not a number");
  std::set<std::string> names;
  for (const contour_component &c : s.components) {
    if (c.name.empty())
      problems.push_back("a component has no name");
    else if (!names.insert(c.name).second)
      problems.push_back("two components are named '" + c.name + "'");
    bool ok = c.color.size() == 3;
    for (double v : c.color)
      ok = ok && std::isfinite(v) && v >= 0.0 && v <= 1.0;
    if (!ok)
      problems.push_back("component '" + c.name + "': its colour is not three values in [0, 1]");
  }
  for (const auto &sl : s.slices) {
    const std::string at = "slice " + std::to_string(sl.first);
    if (!std::isnan(sl.second.z) && !std::isfinite(sl.second.z))
      problems.push_back(at + ": z = " + param_detail::format_double(sl.second.z) +
                         " is not a number");
    for (const auto &c : sl.second.components) {
      const std::string who = at + ", component '" + c.first + "'";
      if (!names.count(c.first))
        problems.push_back(who + ": no such component in the stack's components");
      for (std::size_t i = 0; i < c.second.size(); ++i) {
        const contour_polygon &p = c.second[i];
        const std::string poly = who + ", polygon " + std::to_string(i);
        if (p.xy.size() % 2 != 0)
          problems.push_back(poly + ": " + std::to_string(p.xy.size()) +
                             " coordinates, not x, y pairs");
        else if (p.size() < 3)
          problems.push_back(poly + ": " + std::to_string(p.size()) +
                             " vertices; a polygon needs three");
        if (!std::all_of(p.xy.begin(), p.xy.end(), [](double v) { return std::isfinite(v); }))
          problems.push_back(poly + ": a coordinate is not a number");
      }
    }
  }
  return problems;
}

void param_table<contour_read_params>::check(const contour_read_params &p,
                                             std::vector<std::string> &problems) {
  if (p.first >= 0 && p.last >= 0 && p.first > p.last)
    problems.push_back("first = " + std::to_string(p.first) +
                       " is after last = " + std::to_string(p.last));
  compile("include", p.include, &problems);
  compile("exclude", p.exclude, &problems);
}

// ---- handlers ----

contour_stack contour_file_io::read(app &, const std::string &filename,
                                    const contour_read_params &) const {
  throw contour_file_error(id() + ": can't read '" + filename + "'");
}

void contour_file_io::write(app &, const contour_stack &, const std::string &filename) const {
  throw contour_file_error(id() + ": can't write '" + filename + "'");
}

file_handler_registry<contour_file_io> &contour_handlers(app &ctx) {
  return ctx.slot<contour_io_slot>().handlers;
}

namespace {
std::vector<std::string> extensions_where(app &ctx, bool (contour_file_io::*can)() const) {
  std::set<std::string> out;
  for (const auto &h : contour_handlers(ctx).all())
    if (((*h).*can)())
      for (const std::string &e : h->extensions())
        out.insert(e);
  return std::vector<std::string>(out.begin(), out.end());
}
} // namespace

std::vector<std::string> contour_read_extensions(app &ctx) {
  return extensions_where(ctx, &contour_file_io::can_read);
}

std::vector<std::string> contour_write_extensions(app &ctx) {
  return extensions_where(ctx, &contour_file_io::can_write);
}

contour_stack read_contours(app &ctx, const std::string &filename, const contour_read_params &p) {
  const std::vector<std::string> bad = params_validate(p);
  if (!bad.empty())
    throw std::invalid_argument("read_contours: " + join(bad, "; "));
  const std::string ext = contour_formats::extension_of(filename);
  std::vector<std::string> errors;
  bool any = false;
  for (const auto &h : contour_handlers(ctx).for_extension(ext)) {
    if (!h->can_read())
      continue;
    any = true;
    try {
      contour_stack s = h->read(ctx, filename, p);
      select(s, p);
      return s;
    } catch (const std::exception &e) {
      errors.push_back(e.what());
    }
  }
  if (!any)
    throw contour_file_error("read_contours: '" + filename + "': no reader for '" + ext +
                             "' (this app reads " + join(contour_read_extensions(ctx), ", ") + ")");
  throw contour_file_error(errors.size() == 1 ? errors.front() : join(errors, "; "));
}

void write_contours(app &ctx, const contour_stack &s, const std::string &filename) {
  const std::vector<std::string> bad = contour_problems(s);
  if (!bad.empty())
    throw std::invalid_argument("write_contours: '" + filename + "': " + join(bad, "; "));
  const std::string ext = contour_formats::extension_of(filename);
  for (const auto &h : contour_handlers(ctx).for_extension(ext))
    if (h->can_write())
      return h->write(ctx, s, filename);
  throw contour_file_error("write_contours: '" + filename + "': no writer for '" + ext +
                           "' (this app writes " + join(contour_write_extensions(ctx), ", ") + ")");
}

} // namespace cvc

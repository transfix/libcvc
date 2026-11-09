/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/section.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h> // cvc::state, SEPARATOR, findDescendant
#include <random>
#include <string>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

std::string join(const std::string &base, std::initializer_list<std::string> rel) {
  std::string s = base;
  for (const std::string &seg : rel) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// Changed-only: a valued leaf takes a new value alone (one journaled mutation), an unchanged one
// costs nothing, and a new or still empty leaf is touched first so the distributed-state adapter
// observes it before its value.
void set_changed(cvc::app &app, const std::string &path, const std::string &val) {
  if (cvc::state *n = app.root().findDescendant(path); n && !n->value().empty()) {
    if (n->value() != val)
      n->value(val);
    return;
  }
  cvc::state &n = app.root()(path);
  n.touch();
  n.value(val);
}

std::string get(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string();
}

std::uint64_t get_uint(cvc::app &app, const std::string &path) {
  const std::string v = get(app, path);
  return v.empty() ? 0 : std::strtoull(v.c_str(), nullptr, 10);
}

std::string utext(std::uint64_t v) { return std::to_string(static_cast<unsigned long long>(v)); }

} // namespace

std::uint64_t section_epoch() {
  static const std::uint64_t epoch = [] {
    std::random_device rd;
    std::uint64_t v = (static_cast<std::uint64_t>(rd()) << 32) ^ rd();
    v ^= static_cast<std::uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    v &= (std::uint64_t{1} << 53) - 1;
    return v ? v : 1;
  }();
  return epoch;
}

section_stamp read_section_stamp(cvc::app &app, const std::string &base) {
  section_stamp s;
  s.epoch = get_uint(app, join(base, {"epoch"}));
  s.at = get_uint(app, join(base, {"at"}));
  s.rev = get_uint(app, join(base, {"rev"}));
  return s;
}

section_writer::section_writer(cvc::app &app, std::string base)
    : _app(app), _base(std::move(base)) {}

std::string section_writer::path(std::initializer_list<std::string> rel) const {
  return join(_base, rel);
}

void section_writer::set(std::initializer_list<std::string> rel, const std::string &value) {
  set_changed(_app, path(rel), value);
}

void section_writer::set_double(std::initializer_list<std::string> rel, double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v); // 17 significant digits recover a binary64
  set(rel, buf);
}

void section_writer::set_uint(std::initializer_list<std::string> rel, std::uint64_t v) {
  set(rel, utext(v));
}

void section_writer::set_flag(std::initializer_list<std::string> rel, bool b) {
  set(rel, b ? "1" : "0");
}

section_stamp section_writer::commit(std::uint64_t at, std::uint64_t epoch) {
  const section_stamp before = read_section_stamp(_app, _base);
  section_stamp s;
  s.epoch = epoch != 0 ? epoch : section_epoch();
  s.at = at;
  s.rev = before.epoch == s.epoch ? before.rev + 1 : 1;
  set_uint({"epoch"}, s.epoch);
  set_uint({"at"}, s.at);
  // Last, and always a new value: what a reader keys on.
  cvc::state &rev = _app.root()(path({"rev"}));
  rev.touch();
  rev.value(utext(s.rev));
  return s;
}

section_reader::section_reader(cvc::app &app, std::string base)
    : _app(app), _base(std::move(base)) {}

std::string section_reader::path(std::initializer_list<std::string> rel) const {
  return join(_base, rel);
}

bool section_reader::has(std::initializer_list<std::string> rel) const {
  return !get(_app, path(rel)).empty();
}

std::string section_reader::text(std::initializer_list<std::string> rel,
                                 const std::string &def) const {
  const std::string v = get(_app, path(rel));
  return v.empty() ? def : v;
}

double section_reader::number(std::initializer_list<std::string> rel, double def) const {
  const std::string v = get(_app, path(rel));
  return v.empty() ? def : std::strtod(v.c_str(), nullptr);
}

std::uint64_t section_reader::uint(std::initializer_list<std::string> rel,
                                   std::uint64_t def) const {
  const std::string v = get(_app, path(rel));
  return v.empty() ? def : std::strtoull(v.c_str(), nullptr, 10);
}

bool section_reader::flag(std::initializer_list<std::string> rel, bool def) const {
  const std::string v = get(_app, path(rel));
  return v.empty() ? def : v == "1";
}

} // namespace ariadne
} // namespace cvc

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

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "json.h"

#include <algorithm>
#include <cstdio>
#include <cvc/tf/vinay.h>
#include <fstream>
#include <initializer_list>
#include <istream>
#include <ostream>
#include <sstream>

namespace cvc {
namespace tf {

namespace {

const char kMagic[] = "Anthony and Vinay are Great.";

bool blank(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
}

// Whitespace-separated tokens: ColorTable2 split on single spaces only, so a tab or a doubled
// space threw.
std::vector<std::string> tokens(const std::string &line) {
  std::vector<std::string> out;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && blank(line[i]))
      ++i;
    const std::size_t b = i;
    while (i < line.size() && !blank(line[i]))
      ++i;
    if (i > b)
      out.emplace_back(line, b, i - b);
  }
  return out;
}

// The form labels are matched in: blank runs collapsed, ends trimmed, ASCII lowered.
std::string canon(const std::string &line) {
  std::string out;
  for (const std::string &t : tokens(line)) {
    if (!out.empty())
      out += ' ';
    for (const char c : t)
      out += (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
  }
  return out;
}

// What a line is when quoted in a message: trimmed, and cut short.
std::string quote(const std::string &line) {
  const std::size_t b = line.find_first_not_of(" \t\r\n");
  std::string t = b == std::string::npos ? std::string()
                                         : line.substr(b, line.find_last_not_of(" \t\r\n") - b + 1);
  if (t.size() > 60)
    t = t.substr(0, 57) + "...";
  return "'" + t + "'";
}

enum class section { none, alpha, color, iso };

struct section_info {
  const char *name;  // as ColorTable2 wrote it
  const char *label; // the line after the count
  std::size_t fields;
  const char *shape;
};

const section_info &info(section s) {
  static const section_info alpha{"Alphamap", "Position and opacity", 2, "<position> <opacity>"};
  static const section_info color{"ColorMap", "Position and RGB", 4, "<position> <r> <g> <b>"};
  static const section_info iso{"IsocontourMap", "Position", 1, "<position>"};
  return s == section::alpha ? alpha : s == section::color ? color : iso;
}

section section_of(const std::string &canonical) {
  if (canonical == "alphamap")
    return section::alpha;
  if (canonical == "colormap")
    return section::color;
  if (canonical == "isocontourmap")
    return section::iso;
  return section::none;
}

bool parse_count(const std::string &line, std::size_t &n) {
  const std::vector<std::string> t = tokens(line);
  // Nine digits keep the arithmetic in range on wasm32's 32-bit size_t; real files hold dozens.
  if (t.size() != 1 || t[0].empty() || t[0].size() > 9 ||
      !std::all_of(t[0].begin(), t[0].end(), [](char c) { return c >= '0' && c <= '9'; }))
    return false;
  n = static_cast<std::size_t>(std::stoul(t[0]));
  return true;
}

bool parse_fields(const std::string &line, std::size_t fields, double *out) {
  const std::vector<std::string> t = tokens(line);
  if (t.size() != fields)
    return false;
  for (std::size_t i = 0; i < fields; ++i)
    if (!json::parse_number(t[i], out[i]))
      return false;
  return true;
}

struct parser {
  std::vector<std::string> lines;
  std::size_t i = 0; // index of the next unread line
  std::string where; // "<path>, " for files
  std::vector<std::string> *warnings = nullptr;

  std::string at(std::size_t idx) const {
    return where + "line " + std::to_string(idx + 1) +
           (idx >= lines.size() ? " (end of file)" : "");
  }
  void warn(std::size_t idx, const std::string &msg) {
    if (warnings)
      warnings->push_back(at(idx) + ": " + msg);
  }
  [[noreturn]] void error(std::size_t idx, const std::string &msg) const {
    throw vinay_error(at(idx) + ": " + msg);
  }
  bool eof() const { return i >= lines.size(); }
  std::string cur() const { return eof() ? std::string() : canon(lines[i]); }
  // ColorTable2 read every line of a section back to back; blank lines are skipped here.
  void skip_blank() {
    while (!eof() && canon(lines[i]).empty())
      ++i;
  }

  // A fixed label line ("Number of nodes", "Position and opacity", ...). A missing one is only a
  // warning, and when the line in its place is already the next datum it is not consumed.
  template <class IsDatum>
  void label(const std::string &text, const std::string &after, IsDatum is_datum) {
    skip_blank();
    if (cur() == canon(text)) {
      ++i;
      return;
    }
    if (eof() || is_datum(lines[i]) || section_of(cur()) != section::none) {
      warn(i, "missing '" + text + "' after " + after);
      return;
    }
    warn(i, "expected '" + text + "', found " + quote(lines[i]));
    ++i;
  }
};

model parse(std::istream &in, std::vector<std::string> *warnings, const std::string &where) {
  parser p;
  p.warnings = warnings;
  p.where = where;
  for (std::string line; std::getline(in, line);) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    p.lines.push_back(std::move(line));
  }
  if (!p.lines.empty() && p.lines[0].compare(0, 3, "\xEF\xBB\xBF") == 0)
    p.lines[0].erase(0, 3); // a UTF-8 BOM from a Windows editor

  // ColorTable2 only printed a mismatch here and read on; so does this, but a first line that
  // is already a section header is kept as one rather than swallowed as the "magic".
  bool magic = false;
  if (!p.lines.empty() && canon(p.lines[0]) == canon(kMagic)) {
    magic = true;
    p.i = 1;
  } else if (!p.lines.empty() && section_of(canon(p.lines[0])) != section::none) {
    p.warn(0, std::string("missing the '") + kMagic + "' line");
  } else {
    p.warn(0, std::string("expected '") + kMagic + "', found " +
                  (p.lines.empty() ? std::string("nothing") : quote(p.lines[0])));
    p.i = p.lines.empty() ? 0 : 1;
  }

  model m;
  std::vector<std::pair<double, std::size_t>> isos; // position, line index
  bool any_section = false;
  while (!p.eof()) {
    const std::string c = p.cur();
    if (c.empty()) {
      ++p.i;
      continue;
    }
    const section kind = section_of(c);
    if (kind == section::none) {
      p.warn(p.i, "ignored " + quote(p.lines[p.i]));
      ++p.i;
      continue;
    }
    any_section = true;
    const section_info &s = info(kind);
    const std::size_t header = p.i++;
    const std::string after = std::string("'") + s.name + "'";
    std::size_t count = 0;
    p.label("Number of nodes", after, [](const std::string &l) {
      std::size_t n = 0;
      return parse_count(l, n);
    });
    p.skip_blank();
    if (p.eof() || !parse_count(p.lines[p.i], count))
      p.error(p.eof() ? header : p.i,
              std::string("expected the node count of the '") + s.name + "' section" +
                  (p.eof() ? std::string() : ", found " + quote(p.lines[p.i])));
    ++p.i;
    p.label(s.label, "the " + after + " count", [&s](const std::string &l) {
      double d[4];
      return parse_fields(l, s.fields, d);
    });
    for (std::size_t k = 0; k < count; ++k) {
      p.skip_blank();
      if (p.eof())
        p.error(p.i, std::string("the '") + s.name + "' section at line " +
                         std::to_string(header + 1) + " declares " + std::to_string(count) +
                         " nodes but the file ends after " + std::to_string(k));
      double d[4] = {0.0, 0.0, 0.0, 0.0};
      if (!parse_fields(p.lines[p.i], s.fields, d))
        p.error(p.i, std::string("expected a '") + s.name + "' node \"" + s.shape + "\", found " +
                         quote(p.lines[p.i]));
      if (kind == section::alpha) {
        alpha_node n;
        n.at = d[0];
        n.a = static_cast<float>(d[1]);
        m.alpha.push_back(n);
      } else if (kind == section::color) {
        color_node n;
        n.at = d[0];
        n.r = static_cast<float>(d[1]);
        n.g = static_cast<float>(d[2]);
        n.b = static_cast<float>(d[3]);
        m.color.push_back(n);
      } else {
        isos.emplace_back(d[0], p.i);
      }
      ++p.i;
    }
  }
  if (!magic && !any_section)
    throw vinay_error(where + "line 1: not a .vinay file (no '" + kMagic +
                      "' line and no Alphamap / ColorMap / IsocontourMap section)");

  // The format has no ids. Name them in ascending position, so a model written by write_vinay
  // with ids iso1..isoN reads back identically; equal positions collapse as they did in
  // ColorTable2's position-keyed set.
  std::stable_sort(isos.begin(), isos.end(),
                   [](const auto &a, const auto &b) { return a.first < b.first; });
  for (std::size_t k = 0; k < isos.size(); ++k) {
    if (k && isos[k].first == isos[k - 1].first) {
      p.warn(isos[k].second, "duplicate isocontour position dropped");
      continue;
    }
    isocontour iso;
    iso.at = isos[k].first;
    iso.id = "iso" + std::to_string(m.isocontours.size() + 1);
    m.isocontours.push_back(std::move(iso));
  }
  m.normalize();
  return m;
}

void append_alpha(std::string &s, const alpha_node &n) {
  json::append_exact(s, n.at);
  s += ' ';
  json::append_exact_float(s, n.a);
  s += '\n';
}

void append_color(std::string &s, const color_node &n) {
  json::append_exact(s, n.at);
  for (const float c : {n.r, n.g, n.b}) {
    s += ' ';
    json::append_exact_float(s, c);
  }
  s += '\n';
}

// ColorTable2's order: the first node, the last node, then the interior ones.
template <class Node, class Append>
void append_section(std::string &s, const char *name, const char *label, const std::vector<Node> &v,
                    Append append) {
  s += name;
  s += "\nNumber of nodes\n";
  s += std::to_string(v.size());
  s += '\n';
  s += label;
  s += '\n';
  if (v.empty())
    return;
  append(s, v.front());
  if (v.size() > 1)
    append(s, v.back());
  for (std::size_t i = 1; i + 1 < v.size(); ++i)
    append(s, v[i]);
}

} // namespace

model read_vinay(std::istream &in, std::vector<std::string> *warnings) {
  return parse(in, warnings, std::string());
}

model read_vinay_string(const std::string &text, std::vector<std::string> *warnings) {
  std::istringstream in(text);
  return parse(in, warnings, std::string());
}

model read_vinay_file(const std::string &path, std::vector<std::string> *warnings) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw vinay_error("cannot open '" + path + "'");
  return parse(in, warnings, path + ", ");
}

void write_vinay(std::ostream &out, const model &m) { out << write_vinay_string(m); }

std::string write_vinay_string(const model &m) {
  model c = m;
  c.normalize(); // guarantees both endpoints, which the first/last/interior layout assumes
  std::string s = kMagic;
  s += '\n';
  append_section(s, "Alphamap", "Position and opacity", c.alpha, append_alpha);
  append_section(s, "ColorMap", "Position and RGB", c.color, append_color);
  s += "IsocontourMap\nNumber of nodes\n";
  s += std::to_string(c.isocontours.size());
  s += "\nPosition\n";
  for (const isocontour &iso : c.isocontours) {
    json::append_exact(s, iso.at);
    s += '\n';
  }
  return s;
}

void write_vinay_file(const std::string &path, const model &m) {
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw vinay_error("cannot open '" + path + "' for writing");
  write_vinay(out, m);
  out.flush();
  if (!out)
    throw vinay_error("failed writing '" + path + "'");
}

std::string write_ctbl_string(const model &m, int size) {
  model c = m;
  c.normalize();
  size = std::max(size, 2);
  // Honours the model's opacity_cubed; ColorTable2 hard-coded cubing here ("this must be fixed").
  const std::vector<std::uint8_t> t = c.bake_rgba8(size);
  std::string s = std::to_string(size);
  s += '\n';
  char row[32];
  for (int i = 0; i < size; ++i) {
    const std::uint8_t *e = &t[static_cast<std::size_t>(i) * 4];
    std::snprintf(row, sizeof(row), "%d %d %d %d\n", e[0], e[1], e[2], e[3]);
    s += row;
  }
  return s;
}

} // namespace tf
} // namespace cvc

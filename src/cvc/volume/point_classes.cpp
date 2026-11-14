/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::point_classes, its checks, and its formats on each app's handlers. See
// cvc/volume/point_classes.h. The legacy code the formats come from is VolumeGridRover's
// (transfix/volrover, VolumeGridRover.cpp: savePointClassesSlot and loadPointClassesSlot, and
// PointClassFile.h's content handler, which the Qt6 migration stubbed out).

#include "../utility/xml_lite.h"

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/volume/point_classes.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

namespace cvc {

namespace {

namespace fs = std::filesystem;

// The palette new classes take their colours from, as 0xRRGGBB (ColorBrewer's Set1): each a
// whole number of 255ths, so a .vgr file (#rrggbb) keeps it exactly.
constexpr unsigned kPalette[] = {0xe41a1c, 0x377eb8, 0x4daf4a, 0x984ea3,
                                 0xff7f00, 0xffff33, 0xa65628, 0xf781bf};

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? sep : "") + v[i];
  return out;
}

std::string lower(std::string s) {
  for (char &c : s)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return s;
}

std::string extension_of(const std::string &filename) {
  const std::string name = fs::path(filename).filename().string();
  const std::size_t dot = name.find_last_of('.');
  return dot == std::string::npos || dot == 0 ? std::string() : lower(name.substr(dot));
}

std::string read_text(const std::string &who, const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw point_class_file_error(who + ": can't open '" + path.string() + "'");
  std::ostringstream ss;
  ss << in.rdbuf();
  if (in.bad())
    throw point_class_file_error(who + ": can't read '" + path.string() + "'");
  return ss.str();
}

void write_text(const std::string &who, const fs::path &path, const std::string &text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out)
    throw point_class_file_error(who + ": can't open '" + path.string() + "' for writing");
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.close();
  if (!out)
    throw point_class_file_error(who + ": can't write '" + path.string() + "'");
}

std::vector<std::string> tokens(const std::string &text) {
  std::vector<std::string> out;
  std::string cur;
  for (const char c : text) {
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      if (!cur.empty())
        out.push_back(std::move(cur));
      cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty())
    out.push_back(std::move(cur));
  return out;
}

// Drop the points `c` lists again, keeping the first of each.
void drop_repeats(point_class &c) {
  std::set<std::tuple<int, int, int>> seen;
  std::vector<int> out;
  out.reserve(c.points.size());
  for (std::size_t n = 0; n + 2 < c.points.size(); n += 3)
    if (seen.insert({c.points[n], c.points[n + 1], c.points[n + 2]}).second)
      out.insert(out.end(), {c.points[n], c.points[n + 1], c.points[n + 2]});
  c.points.swap(out);
}

bool to_int(const std::string &s, int &out) {
  long long v = 0;
  if (!param_detail::parse_integer(s, v) || v < std::numeric_limits<int>::min() ||
      v > std::numeric_limits<int>::max())
    return false;
  out = static_cast<int>(v);
  return true;
}

// "#rrggbb" (QColor::name()) or "#rgb", as r, g, b in [0, 1].
bool parse_color(const std::string &s, std::vector<double> &out) {
  const auto hex = [](char c) {
    return c >= '0' && c <= '9'   ? c - '0'
           : c >= 'a' && c <= 'f' ? c - 'a' + 10
           : c >= 'A' && c <= 'F' ? c - 'A' + 10
                                  : -1;
  };
  if (s.size() != 7 && s.size() != 4)
    return false;
  if (s[0] != '#')
    return false;
  std::vector<double> c(3);
  for (int k = 0; k < 3; ++k) {
    int v = 0;
    if (s.size() == 7) {
      const int hi = hex(s[1 + 2 * k]), lo = hex(s[2 + 2 * k]);
      if (hi < 0 || lo < 0)
        return false;
      v = hi * 16 + lo;
    } else {
      const int d = hex(s[1 + k]);
      if (d < 0)
        return false;
      v = d * 17;
    }
    c[static_cast<std::size_t>(k)] = v / 255.0;
  }
  out = c;
  return true;
}

std::string color_name(const std::vector<double> &c) {
  const char *digits = "0123456789abcdef";
  std::string out = "#";
  for (double v : c) {
    const int b = static_cast<int>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0));
    out += digits[b >> 4];
    out += digits[b & 15];
  }
  return out;
}

// ---- VolumeGridRover's .vgr ----
//
//   <!DOCTYPE pointclassdoc>
//   <pointclassdoc>
//    <pointclass name="seeds" color="#ff0000" variable="0" timestep="0" radius="1">
//     <point>12 40 7</point>
//    </pointclass>
//   </pointclassdoc>
//
// The radius is libcvc's; a legacy file has none (1 then). A point or a class listed twice is
// read once.

class vgr_io : public point_class_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return true; }

  point_classes read(app &, const std::string &filename,
                     const point_class_read_params &) const override {
    const std::string who = "point classes '" + filename + "'";
    xml_lite::element doc;
    try {
      doc = xml_lite::parse(read_text(who, fs::path(filename)));
    } catch (const xml_lite::parse_error &e) {
      throw point_class_file_error(who + ": not XML (" + e.what() + ")");
    }
    if (doc.name != "pointclassdoc")
      throw point_class_file_error(who + ": not a point class file (no <pointclassdoc>)");
    point_classes pc;
    for (const xml_lite::element &e : doc.children) {
      if (e.name != "pointclass")
        continue;
      const std::string at = who + ", line " + std::to_string(e.line);
      point_class c;
      c.name = e.attribute("name", "");
      if (c.name.empty())
        throw point_class_file_error(at + ": a <pointclass> has no name");
      const std::string col = e.attribute("color", "");
      if (!parse_color(col, c.color))
        throw point_class_file_error(at + ": class '" + c.name + "': color '" + col +
                                     "' is not #rrggbb");
      if (!to_int(e.attribute("variable", "0"), c.variable) || c.variable < 0)
        throw point_class_file_error(at + ": class '" + c.name + "': variable '" +
                                     e.attribute("variable", "") + "' is not an index");
      if (!to_int(e.attribute("timestep", "0"), c.timestep) || c.timestep < 0)
        throw point_class_file_error(at + ": class '" + c.name + "': timestep '" +
                                     e.attribute("timestep", "") + "' is not an index");
      const std::string r = e.attribute("radius", "1");
      if (!param_detail::parse_double(r, c.radius) || !std::isfinite(c.radius) || c.radius <= 0)
        throw point_class_file_error(at + ": class '" + c.name + "': radius '" + r +
                                     "' is not a positive number");
      int k = pc.class_index(c.name, c.variable, c.timestep);
      if (k < 0) {
        pc.classes.push_back(c);
        k = static_cast<int>(pc.classes.size()) - 1;
      }
      for (const xml_lite::element &pt : e.children) {
        if (pt.name != "point")
          continue;
        const std::vector<std::string> t = tokens(pt.text);
        int ijk[3] = {0, 0, 0};
        if (t.size() != 3 || !to_int(t[0], ijk[0]) || !to_int(t[1], ijk[1]) ||
            !to_int(t[2], ijk[2]) || ijk[0] < 0 || ijk[1] < 0 || ijk[2] < 0)
          throw point_class_file_error(who + ", line " + std::to_string(pt.line) + ": class '" +
                                       c.name + "': point '" + pt.text +
                                       "' is not three voxel indices");
        point_class &dst = pc.classes[static_cast<std::size_t>(k)];
        dst.points.insert(dst.points.end(), {ijk[0], ijk[1], ijk[2]});
      }
    }
    for (point_class &c : pc.classes)
      drop_repeats(c);
    return pc;
  }

  void write(app &, const point_classes &pc, const std::string &filename) const override {
    std::string out = "<!DOCTYPE pointclassdoc>\n<pointclassdoc>\n";
    for (const point_class &c : pc.classes) {
      out += " <pointclass name=\"" + xml_lite::escape(c.name) + "\" color=\"" +
             color_name(c.color) + "\" variable=\"" + std::to_string(c.variable) +
             "\" timestep=\"" + std::to_string(c.timestep) + "\" radius=\"" +
             param_detail::format_double(c.radius) + "\">\n";
      for (std::size_t i = 0; i < c.size(); ++i)
        out += "  <point>" + std::to_string(c.points[3 * i]) + " " +
               std::to_string(c.points[3 * i + 1]) + " " + std::to_string(c.points[3 * i + 2]) +
               "</point>\n";
      out += " </pointclass>\n";
    }
    out += "</pointclassdoc>\n";
    write_text("point classes '" + filename + "'", fs::path(filename), out);
  }

private:
  std::string _id = "vgr_point_classes : v1";
  std::vector<std::string> _ext{".vgr"};
};

// ---- VolumeGridRover's .pts: per class, a count and that many x y z object coordinates ----
//
// The classes are "Class 0", "Class 1", ... with the palette's colours (the legacy drew random
// ones from srand(time(NULL))). With the volume's dims and bounding box, a coordinate x becomes
// int((x - xmin) / (xmax - xmin) * (xdim - 1)), clamped to the volume, as the legacy computed it;
// without them, the nearest index.

class pts_io : public point_class_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return false; }

  point_classes read(app &, const std::string &filename,
                     const point_class_read_params &p) const override {
    const std::string who = "point list '" + filename + "'";
    const std::vector<std::string> t = tokens(read_text(who, fs::path(filename)));
    const bool grid = p.dims.size() == 3 && p.bbox.size() == 6;
    point_classes pc;
    std::size_t i = 0;
    const auto next = [&](const std::string &what) -> const std::string & {
      if (i >= t.size())
        throw point_class_file_error(who + ": the file ends before " + what);
      return t[i++];
    };
    while (i < t.size()) {
      const std::string cls = "Class " + std::to_string(pc.classes.size());
      int count = 0;
      const std::string &ct = next("a class's point count");
      if (!to_int(ct, count) || count < 0)
        throw point_class_file_error(who + ": " + cls + ": '" + ct + "' is not a point count");
      point_class c;
      c.name = cls;
      c.color = point_class_palette(pc.classes.size());
      c.variable = p.variable;
      c.timestep = p.timestep;
      for (int n = 0; n < count; ++n) {
        int ijk[3] = {0, 0, 0};
        for (int a = 0; a < 3; ++a) {
          const std::string what = cls + ", point " + std::to_string(n);
          const std::string &s = next(what);
          double x = 0.0;
          if (!param_detail::parse_double(s, x) || !std::isfinite(x))
            throw point_class_file_error(who + ": " + what + ": '" + s + "' is not a number");
          double v = 0.0;
          if (grid) {
            const double lo = p.bbox[static_cast<std::size_t>(a)];
            const double hi = p.bbox[static_cast<std::size_t>(a) + 3];
            const int dim = p.dims[static_cast<std::size_t>(a)];
            v = std::trunc((x - lo) / (hi - lo) * (dim - 1));
            v = std::clamp(v, 0.0, static_cast<double>(dim - 1));
          } else {
            v = std::round(x);
          }
          if (std::abs(v) > std::numeric_limits<int>::max())
            throw point_class_file_error(who + ": " + what + ": " + s + " is out of range");
          ijk[a] = static_cast<int>(v);
        }
        c.points.insert(c.points.end(), {ijk[0], ijk[1], ijk[2]});
      }
      drop_repeats(c);
      pc.classes.push_back(std::move(c));
    }
    return pc;
  }

private:
  std::string _id = "vgr_point_list : v1";
  std::vector<std::string> _ext{".pts"};
};

// The app's point-class handlers: app.slot<T>() makes one per app, with the built-in formats,
// the first time it is asked (§1.3: no process-wide table).
struct point_class_io_slot {
  static constexpr const char *app_slot = "cvc.volume.point_class_handlers";
  file_handler_registry<point_class_file_io> handlers;
  point_class_io_slot() {
    handlers.insert(boost::shared_ptr<point_class_file_io>(new vgr_io));
    handlers.insert(boost::shared_ptr<point_class_file_io>(new pts_io));
  }
};

std::vector<std::string> extensions_where(app &ctx, bool (point_class_file_io::*can)() const) {
  std::set<std::string> out;
  for (const auto &h : point_class_handlers(ctx).all())
    if (((*h).*can)())
      for (const std::string &e : h->extensions())
        out.insert(e);
  return std::vector<std::string>(out.begin(), out.end());
}

std::string where_class(const point_class &c) {
  std::string s = "class '" + c.name + "'";
  if (c.variable != 0 || c.timestep != 0)
    s += " (variable " + std::to_string(c.variable) + ", timestep " + std::to_string(c.timestep) +
         ")";
  return s;
}

} // namespace

// ---- the types ----

bool point_class::contains(int i, int j, int k) const {
  for (std::size_t n = 0; n + 2 < points.size(); n += 3)
    if (points[n] == i && points[n + 1] == j && points[n + 2] == k)
      return true;
  return false;
}

int point_classes::class_index(const std::string &name, int variable, int timestep) const {
  for (std::size_t i = 0; i < classes.size(); ++i)
    if (classes[i].name == name && classes[i].variable == variable &&
        classes[i].timestep == timestep)
      return static_cast<int>(i);
  return -1;
}

bool point_classes::add(const std::string &name, int i, int j, int k, int variable, int timestep) {
  if (name.empty())
    throw std::invalid_argument("point_classes::add: the class needs a name");
  if (i < 0 || j < 0 || k < 0)
    throw std::invalid_argument("point_classes::add: (" + std::to_string(i) + ", " +
                                std::to_string(j) + ", " + std::to_string(k) +
                                ") is not a voxel index");
  if (variable < 0 || timestep < 0)
    throw std::invalid_argument("point_classes::add: a negative variable or timestep");
  int c = class_index(name, variable, timestep);
  if (c < 0) {
    point_class pc;
    pc.name = name;
    pc.color = point_class_palette(classes.size());
    pc.variable = variable;
    pc.timestep = timestep;
    classes.push_back(std::move(pc));
    c = static_cast<int>(classes.size()) - 1;
  }
  point_class &dst = classes[static_cast<std::size_t>(c)];
  if (dst.contains(i, j, k))
    return false;
  dst.points.insert(dst.points.end(), {i, j, k});
  return true;
}

bool point_classes::remove(const std::string &name, int i, int j, int k, int variable,
                           int timestep) {
  const int c = class_index(name, variable, timestep);
  if (c < 0)
    return false;
  std::vector<int> &pts = classes[static_cast<std::size_t>(c)].points;
  for (std::size_t n = 0; n + 2 < pts.size(); n += 3)
    if (pts[n] == i && pts[n + 1] == j && pts[n + 2] == k) {
      pts.erase(pts.begin() + static_cast<std::ptrdiff_t>(n),
                pts.begin() + static_cast<std::ptrdiff_t>(n + 3));
      return true;
    }
  return false;
}

void point_classes::set_class(const point_class &c) {
  if (c.name.empty())
    throw std::invalid_argument("point_classes::set_class: the class needs a name");
  const int i = class_index(c.name, c.variable, c.timestep);
  if (i < 0)
    classes.push_back(c);
  else
    classes[static_cast<std::size_t>(i)] = c;
}

std::vector<std::string> point_classes::class_names() const {
  std::vector<std::string> out;
  out.reserve(classes.size());
  for (const point_class &c : classes)
    out.push_back(c.name);
  return out;
}

std::size_t point_classes::point_count() const {
  std::size_t n = 0;
  for (const point_class &c : classes)
    n += c.size();
  return n;
}

std::vector<double> point_class_palette(std::size_t n) {
  const unsigned rgb = kPalette[n % (sizeof kPalette / sizeof kPalette[0])];
  return {((rgb >> 16) & 0xff) / 255.0, ((rgb >> 8) & 0xff) / 255.0, (rgb & 0xff) / 255.0};
}

std::vector<std::string> point_class_problems(const point_classes &pc,
                                              const std::vector<int> &dims) {
  std::vector<std::string> problems;
  if (!dims.empty() && dims.size() != 3)
    problems.push_back("dims: " + std::to_string(dims.size()) + " values, not three (x, y, z)");
  std::set<std::tuple<std::string, int, int>> seen;
  for (const point_class &c : pc.classes) {
    const std::string who = where_class(c);
    if (c.name.empty())
      problems.push_back("a class has no name");
    else if (!seen.insert({c.name, c.variable, c.timestep}).second)
      problems.push_back("two classes are " + who);
    bool ok = c.color.size() == 3;
    for (double v : c.color)
      ok = ok && std::isfinite(v) && v >= 0.0 && v <= 1.0;
    if (!ok)
      problems.push_back(who + ": its colour is not three values in [0, 1]");
    if (!std::isfinite(c.radius) || c.radius <= 0)
      problems.push_back(who + ": radius " + param_detail::format_double(c.radius) +
                         " is not a positive number");
    if (c.variable < 0 || c.timestep < 0)
      problems.push_back(who + ": a negative variable or timestep");
    if (c.points.size() % 3 != 0) {
      problems.push_back(who + ": " + std::to_string(c.points.size()) +
                         " indices, not i, j, k triples");
      continue;
    }
    std::set<std::tuple<int, int, int>> pts;
    for (std::size_t n = 0; n < c.size(); ++n) {
      const int i = c.points[3 * n], j = c.points[3 * n + 1], k = c.points[3 * n + 2];
      const std::string at =
          "(" + std::to_string(i) + ", " + std::to_string(j) + ", " + std::to_string(k) + ")";
      if (i < 0 || j < 0 || k < 0)
        problems.push_back(who + ": point " + at + " is not a voxel index");
      else if (dims.size() == 3 && (i >= dims[0] || j >= dims[1] || k >= dims[2]))
        problems.push_back(who + ": point " + at + " is outside the " + std::to_string(dims[0]) +
                           "x" + std::to_string(dims[1]) + "x" + std::to_string(dims[2]) +
                           " volume");
      if (!pts.insert({i, j, k}).second)
        problems.push_back(who + ": point " + at + " is listed twice");
    }
  }
  return problems;
}

// ---- handlers ----

point_classes point_class_file_io::read(app &, const std::string &filename,
                                        const point_class_read_params &) const {
  throw point_class_file_error(id() + ": can't read '" + filename + "'");
}

void point_class_file_io::write(app &, const point_classes &, const std::string &filename) const {
  throw point_class_file_error(id() + ": can't write '" + filename + "'");
}

file_handler_registry<point_class_file_io> &point_class_handlers(app &ctx) {
  return ctx.slot<point_class_io_slot>().handlers;
}

std::vector<std::string> point_class_read_extensions(app &ctx) {
  return extensions_where(ctx, &point_class_file_io::can_read);
}

std::vector<std::string> point_class_write_extensions(app &ctx) {
  return extensions_where(ctx, &point_class_file_io::can_write);
}

point_classes read_point_classes(app &ctx, const std::string &filename,
                                 const point_class_read_params &p) {
  const std::vector<std::string> bad = params_validate(p);
  if (!bad.empty())
    throw std::invalid_argument("read_point_classes: " + join(bad, "; "));
  const std::string ext = extension_of(filename);
  std::vector<std::string> errors;
  bool any = false;
  for (const auto &h : point_class_handlers(ctx).for_extension(ext)) {
    if (!h->can_read())
      continue;
    any = true;
    try {
      point_classes pc = h->read(ctx, filename, p);
      const std::vector<std::string> problems = point_class_problems(pc, p.dims);
      if (!problems.empty())
        throw point_class_file_error("read_point_classes: '" + filename +
                                     "': " + join(problems, "; "));
      return pc;
    } catch (const std::exception &e) {
      errors.push_back(e.what());
    }
  }
  if (!any)
    throw point_class_file_error("read_point_classes: '" + filename + "': no reader for '" + ext +
                                 "' (this app reads " +
                                 join(point_class_read_extensions(ctx), ", ") + ")");
  throw point_class_file_error(errors.size() == 1 ? errors.front() : join(errors, "; "));
}

void write_point_classes(app &ctx, const point_classes &pc, const std::string &filename) {
  const std::vector<std::string> bad = point_class_problems(pc);
  if (!bad.empty())
    throw std::invalid_argument("write_point_classes: '" + filename + "': " + join(bad, "; "));
  const std::string ext = extension_of(filename);
  for (const auto &h : point_class_handlers(ctx).for_extension(ext))
    if (h->can_write())
      return h->write(ctx, pc, filename);
  throw point_class_file_error("write_point_classes: '" + filename + "': no writer for '" + ext +
                               "' (this app writes " +
                               join(point_class_write_extensions(ctx), ", ") + ")");
}

} // namespace cvc

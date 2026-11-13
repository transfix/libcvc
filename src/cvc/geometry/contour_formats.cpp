/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The built-in contour_stack formats (cvc/geometry/contours.h lists them). Each is one
// contour_file_io; none keeps any state. The legacy code they replace is VolumeRover's
// (transfix/volrover): ContourTiler's tinyxml Reconstruct reader (xml/Ser_reader.cpp), the Tiling
// program's configuration (Tiling/parse_config.cpp, README.TILING) and VolumeGridRover's .contour
// and .pcd exports (VolumeGridRover.cpp, saveContoursSlot).

#include "contour_formats.h"

#include "../utility/xml_lite.h"

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <climits>
#include <cmath>
#include <cvc/core/param_table.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

namespace cvc {
namespace contour_formats {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;

namespace {

// ---- text, numbers, files ----

std::string num(double d) { return param_detail::format_double(d); }

std::string lower(std::string s) {
  for (char &c : s)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return s;
}

bool finite_number(const std::string &s, double &out) {
  double d = 0.0;
  if (!param_detail::parse_double(s, d) || !std::isfinite(d))
    return false;
  out = d;
  return true;
}

// Whitespace-separated tokens; commas separate too when `commas`.
std::vector<std::string> tokens(const std::string &text, bool commas = false) {
  std::vector<std::string> out;
  std::string cur;
  for (const char c : text) {
    const bool sep = c == ' ' || c == '\t' || c == '\r' || c == '\n' || (commas && c == ',');
    if (sep) {
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

// Reads a whitespace-separated stream of numbers, naming the file in its errors.
class token_reader {
public:
  token_reader(std::string who, const std::string &text)
      : _who(std::move(who)), _toks(tokens(text)) {}
  bool done() const { return _i >= _toks.size(); }
  double number(const std::string &what) {
    const std::string &t = next(what);
    double d = 0.0;
    if (!finite_number(t, d))
      throw contour_file_error(_who + ": " + what + ": '" + t + "' is not a number");
    return d;
  }
  long long integer(const std::string &what) {
    const std::string &t = next(what);
    long long v = 0;
    if (!param_detail::parse_integer(t, v))
      throw contour_file_error(_who + ": " + what + ": '" + t + "' is not an integer");
    return v;
  }

private:
  const std::string &next(const std::string &what) {
    if (done())
      throw contour_file_error(_who + ": the file ends before " + what);
    return _toks[_i++];
  }
  std::string _who;
  std::vector<std::string> _toks;
  std::size_t _i = 0;
};

std::string read_text(const std::string &who, const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw contour_file_error(who + ": can't open '" + path.string() + "'");
  std::ostringstream ss;
  ss << in.rdbuf();
  if (in.bad())
    throw contour_file_error(who + ": can't read '" + path.string() + "'");
  return ss.str();
}

void write_text(const std::string &who, const fs::path &path, const std::string &text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out)
    throw contour_file_error(who + ": can't open '" + path.string() + "' for writing");
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.close();
  if (!out)
    throw contour_file_error(who + ": can't write '" + path.string() + "'");
}

fs::path dir_of(const fs::path &p) { return p.has_parent_path() ? p.parent_path() : fs::path("."); }

bool all_digits(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
}

// The slice's polygons, every component's, in the stack's component order.
std::vector<const contour_polygon *> slice_polygons(const contour_stack &s,
                                                    const contour_slice &sl) {
  std::vector<const contour_polygon *> out;
  for (const contour_component &c : s.components) {
    auto it = sl.components.find(c.name);
    if (it != sl.components.end())
      for (const contour_polygon &p : it->second)
        out.push_back(&p);
  }
  return out;
}

void add_component(contour_stack &s, const std::string &name) {
  if (s.component_index(name) < 0) {
    contour_component c;
    c.name = name;
    s.components.push_back(std::move(c));
  }
}

// Drop a last vertex that repeats the first: the closing edge is implied.
void drop_closing_vertex(std::vector<double> &xy) {
  const std::size_t n = xy.size();
  if (n >= 4 && xy[n - 2] == xy[0] && xy[n - 1] == xy[1])
    xy.resize(n - 2);
}

// ---- XML (Reconstruct) ----

xml_lite::element parse_xml(const std::string &who, const fs::path &path) {
  try {
    return xml_lite::parse(read_text(who, path));
  } catch (const xml_lite::parse_error &e) {
    throw contour_file_error(who + ": '" + path.string() + "' is not XML (" + e.what() + ")");
  }
}

// A Reconstruct section transform: x' = a0 + a1 x + a2 y + a3 xy + a4 x^2 + a5 y^2 (and y' with
// b). Reconstruct stores the polynomial that takes series (world) coordinates to the section's own,
// and a trace in the section's own, so a trace's world coordinates are the polynomial's inverse
// (as PyReconstruct reads them: Transform.xy_inverse). The legacy reader subtracted a0 and applied
// the rest forwards, which is the inverse only for a translation.
struct section_transform {
  double a[6] = {0, 1, 0, 0, 0, 0};
  double b[6] = {0, 0, 1, 0, 0, 0};

  bool affine() const {
    return a[3] == 0 && a[4] == 0 && a[5] == 0 && b[3] == 0 && b[4] == 0 && b[5] == 0;
  }
  void forward(double x, double y, double &u, double &v) const {
    u = a[0] + a[1] * x + a[2] * y + a[3] * x * y + a[4] * x * x + a[5] * y * y;
    v = b[0] + b[1] * x + b[2] * y + b[3] * x * y + b[4] * x * x + b[5] * y * y;
  }
  // The (x, y) that forward() takes to (u, v): exact for an affine transform, Newton's method from
  // the affine part's inverse otherwise. False when there is none (a singular Jacobian).
  bool inverse(double u, double v, double &x, double &y) const {
    const double det = a[1] * b[2] - a[2] * b[1];
    if (det == 0 || !std::isfinite(det))
      return false;
    x = (b[2] * (u - a[0]) - a[2] * (v - b[0])) / det;
    y = (a[1] * (v - b[0]) - b[1] * (u - a[0])) / det;
    if (affine())
      return true;
    for (int it = 0; it < 64; ++it) {
      double u0 = 0.0, v0 = 0.0;
      forward(x, y, u0, v0);
      const double j11 = a[1] + a[3] * y + 2 * a[4] * x, j12 = a[2] + a[3] * x + 2 * a[5] * y;
      const double j21 = b[1] + b[3] * y + 2 * b[4] * x, j22 = b[2] + b[3] * x + 2 * b[5] * y;
      const double d = j11 * j22 - j12 * j21;
      if (d == 0 || !std::isfinite(d))
        return false;
      const double du = u - u0, dv = v - v0;
      const double dx = (j22 * du - j12 * dv) / d, dy = (j11 * dv - j21 * du) / d;
      x += dx;
      y += dy;
      if (std::abs(dx) + std::abs(dy) <= 1e-15 * (1.0 + std::abs(x) + std::abs(y)))
        return true;
    }
    double u0 = 0.0, v0 = 0.0;
    forward(x, y, u0, v0);
    return std::abs(u - u0) + std::abs(v - v0) <= 1e-9 * (1.0 + std::abs(u) + std::abs(v));
  }
};

void parse_coefficients(const std::string &who, const char *name, const std::string &text,
                        double (&out)[6]) {
  const std::vector<std::string> t = tokens(text);
  if (t.empty() || t.size() > 6)
    throw contour_file_error(who + ": " + name + " has " + std::to_string(t.size()) +
                             " coefficients, not six");
  for (std::size_t i = 0; i < 6; ++i) {
    out[i] = 0.0;
    if (i < t.size() && !finite_number(t[i], out[i]))
      throw contour_file_error(who + ": " + name + ": '" + t[i] + "' is not a number");
  }
}

// The section files of the series `ser` (dir/name.ser): dir/name.<n>, by n.
std::map<int, fs::path> section_files(const fs::path &ser) {
  const std::string stem = ser.stem().string() + ".";
  std::map<int, fs::path> out;
  std::error_code ec;
  for (fs::directory_iterator it(dir_of(ser), ec), end; !ec && it != end; it.increment(ec)) {
    const std::string name = it->path().filename().string();
    if (name.size() <= stem.size() || name.compare(0, stem.size(), stem) != 0)
      continue;
    const std::string digits = name.substr(stem.size());
    long long n = 0;
    std::error_code fe;
    if (all_digits(digits) && param_detail::parse_integer(digits, n) && n <= INT_MAX &&
        it->is_regular_file(fe))
      out[static_cast<int>(n)] = it->path();
  }
  return out;
}

class reconstruct_io : public contour_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return true; }

  contour_stack read(app &, const std::string &filename,
                     const contour_read_params &p) const override {
    const std::string who = "Reconstruct series '" + filename + "'";
    const fs::path ser(filename);
    if (parse_xml(who, ser).name != "Series")
      throw contour_file_error(who + ": not a Reconstruct series (no <Series> element)");
    contour_stack s;
    s.z_spacing = 0.05; // the legacy reader's thickness when a section gives none
    bool spaced = false;
    for (const auto &kv : section_files(ser)) {
      if ((p.first >= 0 && kv.first < p.first) || (p.last >= 0 && kv.first > p.last))
        continue;
      read_section(s, kv.first, kv.second, p, spaced);
    }
    return s;
  }

  void write(app &, const contour_stack &s, const std::string &filename) const override {
    const std::string who = "Reconstruct series '" + filename + "'";
    const fs::path ser(filename);
    if (!s.slices.empty() && s.slices.begin()->first < 0)
      throw contour_file_error(who + ": slice " + std::to_string(s.slices.begin()->first) +
                               ": Reconstruct numbers its sections from 0");
    const int first = s.slices.empty() ? 0 : s.slices.begin()->first;
    write_text(who, ser,
               "<?xml version=\"1.0\"?>\n<!DOCTYPE Series SYSTEM \"series.dtd\">\n\n"
               "<Series index=\"" +
                   std::to_string(first) +
                   "\" viewport=\"0 0 0.00254\" units=\"microns\">\n</Series>\n");
    const fs::path dir = dir_of(ser);
    const std::string stem = ser.stem().string();
    for (const auto &sl : s.slices) {
      std::string x = "<?xml version=\"1.0\"?>\n<!DOCTYPE Section SYSTEM \"section.dtd\">\n\n"
                      "<Section index=\"" +
                      std::to_string(sl.first) + "\" thickness=\"" + num(s.z_spacing) +
                      "\" alignLocked=\"true\">\n"
                      "<Transform dim=\"0\"\n xcoef=\" 0 1 0 0 0 0\"\n ycoef=\" 0 0 1 0 0 0\">\n";
      for (const contour_component &c : s.components) {
        auto it = sl.second.components.find(c.name);
        if (it == sl.second.components.end())
          continue;
        const std::string rgb = num(c.color[0]) + " " + num(c.color[1]) + " " + num(c.color[2]);
        for (const contour_polygon &poly : it->second) {
          x += "<Contour name=\"" + xml_lite::escape(c.name) + "\" hidden=\"" +
               (c.visible ? "false" : "true") + "\" closed=\"true\" simplified=\"true\" border=\"" +
               rgb + "\" fill=\"" + rgb + "\" mode=\"9\"\n points=\"";
          for (std::size_t i = 0; i < poly.size(); ++i)
            x += num(poly.xy[2 * i]) + " " + num(poly.xy[2 * i + 1]) + ",\n\t";
          x += "\"/>\n";
        }
      }
      x += "</Transform>\n\n</Section>\n";
      write_text(who, dir / (stem + "." + std::to_string(sl.first)), x);
    }
    // Writing a series replaces it: this series' section files the stack doesn't have go.
    for (const auto &kv : section_files(ser))
      if (!s.slices.count(kv.first)) {
        std::error_code ec;
        fs::remove(kv.second, ec);
      }
  }

private:
  static void read_section(contour_stack &s, int n, const fs::path &path,
                           const contour_read_params &p, bool &spaced) {
    const std::string who = "Reconstruct section '" + path.string() + "'";
    const xml_lite::element section = parse_xml(who, path);
    if (section.name != "Section")
      throw contour_file_error(who + ": no <Section> element");
    contour_slice &slice = s.slices[n]; // a section with no traces is still a slice
    double thickness = 0.0;
    if (!spaced && finite_number(section.attribute("thickness", ""), thickness) && thickness > 0) {
      s.z_spacing = thickness;
      spaced = true;
    }
    for (const xml_lite::element &tf : section.children) {
      if (tf.name != "Transform" || tf.child("Image"))
        continue; // a transform holding an image holds the image's outline, not an object
      section_transform t;
      if (p.apply_transforms) {
        parse_coefficients(who, "xcoef", tf.attribute("xcoef", "0 1 0 0 0 0"), t.a);
        parse_coefficients(who, "ycoef", tf.attribute("ycoef", "0 0 1 0 0 0"), t.b);
      }
      for (const xml_lite::element &c : tf.children) {
        if (c.name != "Contour")
          continue;
        const std::string name = c.attribute("name", "");
        if (name.empty())
          throw contour_file_error(who + ": a <Contour> has no name");
        if (lower(c.attribute("closed", "true")) == "false" && !p.keep_open)
          continue;
        const std::vector<std::string> pts = tokens(c.attribute("points", ""), true);
        if (pts.size() % 2 != 0)
          throw contour_file_error(who + ": contour '" + name + "': " + std::to_string(pts.size()) +
                                   " coordinates, not x, y pairs");
        contour_polygon poly;
        poly.xy.reserve(pts.size());
        for (std::size_t i = 0; i < pts.size(); i += 2) {
          double u = 0.0, v = 0.0, x = 0.0, y = 0.0;
          if (!finite_number(pts[i], u) || !finite_number(pts[i + 1], v))
            throw contour_file_error(who + ": contour '" + name + "': '" + pts[i] + " " +
                                     pts[i + 1] + "' is not a point");
          if (!t.inverse(u, v, x, y))
            throw contour_file_error(who + ": contour '" + name +
                                     "': its transform has no inverse at (" + pts[i] + ", " +
                                     pts[i + 1] + ")");
          poly.xy.push_back(x);
          poly.xy.push_back(y);
        }
        drop_closing_vertex(poly.xy);
        if (s.component_index(name) < 0) {
          contour_component comp;
          comp.name = name;
          const std::vector<std::string> rgb = tokens(c.attribute("border", ""));
          std::vector<double> col(3);
          bool ok = rgb.size() == 3;
          for (std::size_t i = 0; ok && i < 3; ++i)
            ok = finite_number(rgb[i], col[i]) && col[i] >= 0.0 && col[i] <= 1.0;
          if (ok)
            comp.color = col;
          comp.visible = lower(c.attribute("hidden", "false")) != "true";
          s.components.push_back(std::move(comp));
        }
        slice.components[name].push_back(std::move(poly));
      }
    }
  }

  std::string _id = "reconstruct_ser : v1";
  std::vector<std::string> _ext{".ser"};
};

// ---- the Tiling program's .config + .pts ----

class tiling_config_io : public contour_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return true; }

  contour_stack read(app &, const std::string &filename,
                     const contour_read_params &p) const override {
    const std::string who = "Tiling configuration '" + filename + "'";
    const fs::path cfg(filename);
    std::string text = read_text(who, cfg);
    // Everything after # to the end of a line is a comment.
    std::string clean;
    bool comment = false;
    for (const char c : text) {
      if (c == '\n')
        comment = false;
      else if (c == '#')
        comment = true;
      if (!comment)
        clean += c;
    }
    const std::vector<std::string> t = tokens(clean);
    std::string prefix, suffix;
    bool have_prefix = false, have_range = false, have_detz = false;
    long long beg = 0, end = 0;
    double detz = 0.0, scalez = 1.0;
    for (std::size_t i = 0; i < t.size(); ++i) {
      const std::string &k = t[i];
      // A key's value; a key with none is followed by the next key (which ends in ':').
      const auto value = [&](bool required) -> std::string {
        if (i + 1 < t.size() && t[i + 1].back() != ':')
          return t[++i];
        if (required)
          throw contour_file_error(who + ": " + k + " has no value");
        return std::string();
      };
      const auto number = [&](double &out) {
        const std::string v = value(true);
        if (!finite_number(v, out))
          throw contour_file_error(who + ": " + k + " '" + v + "' is not a number");
      };
      const auto integer = [&](long long &out) {
        const std::string v = value(true);
        if (!param_detail::parse_integer(v, out))
          throw contour_file_error(who + ": " + k + " '" + v + "' is not an integer");
      };
      if (k == "PREFIX:") {
        prefix = value(true);
        have_prefix = true;
      } else if (k == "SUFFIX:") {
        suffix = value(false);
      } else if (k == "SLICE_RANGE:") {
        integer(beg);
        integer(end);
        have_range = true;
      } else if (k == "DETZ:") {
        number(detz);
        have_detz = true;
      } else if (k == "SCALEZ:") {
        number(scalez);
      }
      // Every other key (OUTPUT_FILE_NAME:, NON_VALID:, ...) is the tiler's, not the contours'.
    }
    if (!have_prefix)
      throw contour_file_error(who + ": no PREFIX:");
    if (!have_range)
      throw contour_file_error(who + ": no SLICE_RANGE:");
    if (beg < 0 || end < beg || end > INT_MAX)
      throw contour_file_error(who + ": SLICE_RANGE " + std::to_string(beg) + " " +
                               std::to_string(end) + " is not a range of slice numbers");
    if (have_detz && detz <= 0)
      throw contour_file_error(who + ": DETZ " + num(detz) + " is not positive");
    if (scalez <= 0)
      throw contour_file_error(who + ": SCALEZ " + num(scalez) + " is not positive");
    // PREFIX is relative to the configuration's directory, not the working directory.
    fs::path pre(prefix);
    if (pre.is_relative())
      pre = dir_of(cfg) / pre;
    std::string name = fs::path(prefix).filename().string();
    if (name.empty())
      name = "contours";

    contour_stack s;
    add_component(s, name);
    std::map<int, double> zs; // each slice's z, from its vertices
    for (long long ln = beg; ln <= end; ++ln) {
      const int n = static_cast<int>(ln);
      if ((p.first >= 0 && n < p.first) || (p.last >= 0 && n > p.last))
        continue;
      const fs::path file(pre.string() + std::to_string(n) + suffix);
      const std::string fwho = who + ", slice '" + file.string() + "'";
      token_reader in(fwho, read_text(fwho, file));
      contour_slice &slice = s.slices[n];
      bool have_z = false;
      double z0 = 0.0;
      while (!in.done()) {
        const long long count = in.integer("a contour's vertex count");
        if (count < 0)
          throw contour_file_error(fwho + ": a contour of " + std::to_string(count) + " vertices");
        contour_polygon poly;
        for (long long v = 0; v < count; ++v) {
          const std::string at = "vertex " + std::to_string(v) + " of a contour";
          const double x = in.number(at), y = in.number(at), z = in.number(at);
          if (!have_z) {
            z0 = z;
            have_z = true;
          } else if (std::abs(z - z0) > 1e-9 * (1.0 + std::abs(z0))) {
            throw contour_file_error(fwho + ": vertices at z = " + num(z0) + " and z = " + num(z) +
                                     " (a slice is one plane)");
          }
          poly.xy.push_back(x);
          poly.xy.push_back(y);
        }
        if (count > 0)
          slice.components[name].push_back(std::move(poly));
      }
      if (have_z)
        zs[n] = z0;
    }
    if (have_detz) {
      s.z_spacing = detz;
      s.z_origin = 0.0;
    } else if (zs.size() >= 2) {
      const auto f = zs.begin();
      const auto l = std::prev(zs.end());
      const double sp = (l->second - f->second) / static_cast<double>(l->first - f->first);
      if (sp > 0 && std::isfinite(sp)) {
        s.z_spacing = sp;
        s.z_origin = f->second - static_cast<double>(f->first) * sp;
        for (const auto &kv : zs)
          if (std::abs(s.z_origin + kv.first * sp - kv.second) > 1e-9 * (1.0 + std::abs(kv.second)))
            s.slices[kv.first].z = kv.second;
      } else {
        for (const auto &kv : zs)
          s.slices[kv.first].z = kv.second;
      }
    } else if (zs.size() == 1) {
      s.z_origin = zs.begin()->second - static_cast<double>(zs.begin()->first);
    }
    if (scalez != 1.0) {
      s.z_spacing *= scalez;
      s.z_origin *= scalez;
      for (auto &kv : s.slices)
        kv.second.z *= scalez; // NaN stays NaN
    }
    return s;
  }

  void write(app &, const contour_stack &s, const std::string &filename) const override {
    const std::string who = "Tiling configuration '" + filename + "'";
    if (s.slices.empty())
      throw contour_file_error(who + ": a Tiling series needs at least one slice");
    const int first = s.slices.begin()->first, last = s.slices.rbegin()->first;
    if (first < 0)
      throw contour_file_error(who + ": slice " + std::to_string(first) +
                               ": the Tiling program numbers slices from 0");
    const fs::path cfg(filename);
    const std::string stem = cfg.stem().string();
    if (stem.empty() || stem.find_first_of(" \t#") != std::string::npos)
      throw contour_file_error(who + ": the Tiling program can't name slice files after '" + stem +
                               "'");
    bool regular = s.z_origin == 0.0;
    for (const auto &sl : s.slices)
      regular = regular && std::isnan(sl.second.z);
    std::string c = "# Contours for the Tiling program, written by libcvc. PREFIX names the .pts\n"
                    "# files beside this one.\n"
                    "PREFIX: " +
                    stem + "\nSUFFIX: .pts\nSLICE_RANGE: " + std::to_string(first) + " " +
                    std::to_string(last) + "\n";
    if (regular)
      c += "DETZ: " + num(s.z_spacing) + "\n";
    write_text(who, cfg, c);
    const fs::path dir = dir_of(cfg);
    // The program needs every slice in the range: an empty one is a single point giving its z.
    for (int n = first; n <= last; ++n) {
      const double z = s.slice_z(n);
      std::string t;
      auto it = s.slices.find(n);
      const std::vector<const contour_polygon *> polys =
          it == s.slices.end() ? std::vector<const contour_polygon *>()
                               : slice_polygons(s, it->second);
      for (const contour_polygon *poly : polys) {
        t += std::to_string(poly->size()) + "\n";
        for (std::size_t i = 0; i < poly->size(); ++i)
          t += num(poly->xy[2 * i]) + " " + num(poly->xy[2 * i + 1]) + " " + num(z) + "\n";
      }
      if (polys.empty())
        t = "1\n0 0 " + num(z) + "\n";
      write_text(who, dir / (stem + std::to_string(n) + ".pts"), t);
    }
  }

private:
  std::string _id = "tiling_config : v1";
  std::vector<std::string> _ext{".config"};
};

// ---- Tao Ju's ctr2suf .contour ----
//
//   <planes>
//   per plane: a b c d             the plane a x + b y + c z = d (VolumeGridRover wrote 0 0 1 z)
//              <vertices> <edges>
//              x y z               per vertex
//              i j left right      per edge, from vertex i to vertex j, with the material on
//                                  each side
//
// A component is a material: the writer gives component k (in the stack's order) material k + 1
// on the side its polygon encloses and 0 on the other, so a counter-clockwise polygon's edges are
// "k+1 0" and a clockwise one's "0 k+1". The reader names a material's component "material_<m>":
// the format has no names or colours. VolumeGridRover wrote one object a file, with "1 2" on every
// edge; an edge with two non-zero materials is the left one's.

class ctr2suf_io : public contour_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return true; }

  contour_stack read(app &, const std::string &filename,
                     const contour_read_params &p) const override {
    const std::string who = "ctr2suf contours '" + filename + "'";
    token_reader in(who, read_text(who, fs::path(filename)));
    const long long planes = in.integer("the number of planes");
    if (planes < 0)
      throw contour_file_error(who + ": " + std::to_string(planes) + " planes");
    struct plane_polys {
      double z;
      std::map<long long, std::vector<contour_polygon>> by_material;
    };
    std::vector<plane_polys> read_planes;
    std::set<long long> materials;
    for (long long k = 0; k < planes; ++k) {
      const std::string at = "plane " + std::to_string(k);
      const double a = in.number(at), b = in.number(at), c = in.number(at), d = in.number(at);
      if (a != 0 || b != 0 || c == 0)
        throw contour_file_error(who + ": " + at + " is not a plane z = constant");
      plane_polys pp;
      pp.z = d / c;
      const long long nv = in.integer(at + "'s vertex count");
      const long long ne = in.integer(at + "'s edge count");
      if (nv < 0 || ne < 0)
        throw contour_file_error(who + ": " + at + ": a negative count");
      std::vector<double> vx(static_cast<std::size_t>(nv)), vy(vx.size());
      for (long long v = 0; v < nv; ++v) {
        vx[static_cast<std::size_t>(v)] = in.number(at + ", vertex " + std::to_string(v));
        vy[static_cast<std::size_t>(v)] = in.number(at + ", vertex " + std::to_string(v));
        in.number(at + ", vertex " + std::to_string(v)); // its z: the plane's
      }
      struct edge {
        long long i, j, m;
        bool used = false;
      };
      std::vector<edge> edges;
      std::multimap<std::pair<long long, long long>, std::size_t> from; // (material, i) -> edge
      for (long long e = 0; e < ne; ++e) {
        const std::string ea = at + ", edge " + std::to_string(e);
        const long long i = in.integer(ea), j = in.integer(ea);
        const long long l = in.integer(ea), r = in.integer(ea);
        if (i < 0 || j < 0 || i >= nv || j >= nv)
          throw contour_file_error(who + ": " + ea + ": no vertex " +
                                   std::to_string(i < 0 || i >= nv ? i : j));
        if (l == 0 && r == 0)
          throw contour_file_error(who + ": " + ea + " has material 0 on both sides");
        const long long m = l != 0 ? l : r;
        from.emplace(std::make_pair(m, i), edges.size());
        edges.push_back({i, j, m});
      }
      // Chain each material's edges into closed polygons, in the edges' direction.
      for (std::size_t e0 = 0; e0 < edges.size(); ++e0) {
        if (edges[e0].used)
          continue;
        const long long m = edges[e0].m, start = edges[e0].i;
        contour_polygon poly;
        std::size_t e = e0;
        bool closed = false;
        while (true) {
          edges[e].used = true;
          poly.xy.push_back(vx[static_cast<std::size_t>(edges[e].i)]);
          poly.xy.push_back(vy[static_cast<std::size_t>(edges[e].i)]);
          if (edges[e].j == start) {
            closed = true;
            break;
          }
          std::size_t nxt = edges.size();
          auto range = from.equal_range(std::make_pair(m, edges[e].j));
          for (auto it = range.first; it != range.second; ++it)
            if (!edges[it->second].used) {
              nxt = it->second;
              break;
            }
          if (nxt == edges.size())
            break;
          e = nxt;
        }
        if (!closed && !p.keep_open)
          throw contour_file_error(who + ": " + at + ": the edges of material " +
                                   std::to_string(m) + " don't close");
        materials.insert(m);
        pp.by_material[m].push_back(std::move(poly));
      }
      read_planes.push_back(std::move(pp));
    }
    // Slice indices from the planes' z: the spacing given, or the smallest gap between planes.
    std::vector<double> zs;
    for (const plane_polys &pp : read_planes)
      zs.push_back(pp.z);
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    double spacing = p.z_spacing;
    if (!(spacing > 0)) {
      spacing = 0.0;
      for (std::size_t i = 1; i < zs.size(); ++i)
        if (spacing == 0.0 || zs[i] - zs[i - 1] < spacing)
          spacing = zs[i] - zs[i - 1];
      if (!(spacing > 0))
        spacing = 1.0;
    }
    contour_stack s;
    s.z_spacing = spacing;
    for (long long m : materials)
      add_component(s, "material_" + std::to_string(m));
    std::map<int, double> plane_of; // index -> its plane's z, to catch two planes on one index
    for (plane_polys &pp : read_planes) {
      const double q = std::round(pp.z / spacing);
      if (std::abs(q) > INT_MAX)
        throw contour_file_error(who + ": the plane z = " + num(pp.z) + " is too far out");
      const int n = static_cast<int>(q);
      auto seen = plane_of.find(n);
      if (seen != plane_of.end() && seen->second != pp.z)
        throw contour_file_error(who + ": the planes z = " + num(seen->second) + " and z = " +
                                 num(pp.z) + " fall on one slice at spacing " + num(spacing));
      plane_of[n] = pp.z;
      contour_slice &slice = s.slices[n];
      if (std::abs(n * spacing - pp.z) > 1e-9 * (1.0 + std::abs(pp.z)))
        slice.z = pp.z;
      for (auto &kv : pp.by_material) {
        auto &dst = slice.components["material_" + std::to_string(kv.first)];
        for (contour_polygon &poly : kv.second)
          dst.push_back(std::move(poly));
      }
    }
    return s;
  }

  void write(app &, const contour_stack &s, const std::string &filename) const override {
    const std::string who = "ctr2suf contours '" + filename + "'";
    std::string out;
    std::size_t planes = 0;
    for (const auto &sl : s.slices) {
      const std::vector<const contour_polygon *> polys = slice_polygons(s, sl.second);
      if (polys.empty())
        continue;
      ++planes;
      const double z = s.slice_z(sl.first);
      std::size_t nv = 0;
      for (const contour_polygon *poly : polys)
        nv += poly->size();
      out += "0 0 1 " + num(z) + "\n" + std::to_string(nv) + " " + std::to_string(nv) + "\n";
      for (const contour_polygon *poly : polys)
        for (std::size_t i = 0; i < poly->size(); ++i)
          out += num(poly->xy[2 * i]) + " " + num(poly->xy[2 * i + 1]) + " " + num(z) + "\n";
      std::size_t base = 0;
      for (const contour_component &c : s.components) {
        auto it = sl.second.components.find(c.name);
        if (it == sl.second.components.end())
          continue;
        const std::string m = std::to_string(s.component_index(c.name) + 1);
        for (const contour_polygon &poly : it->second) {
          const std::string sides = poly.signed_area() < 0 ? "0 " + m : m + " 0";
          for (std::size_t i = 0; i < poly.size(); ++i)
            out += std::to_string(base + i) + " " + std::to_string(base + (i + 1) % poly.size()) +
                   " " + sides + "\n";
          base += poly.size();
        }
      }
    }
    write_text(who, fs::path(filename), std::to_string(planes) + "\n" + out);
  }

private:
  std::string _id = "ctr2suf_contour : v1";
  std::vector<std::string> _ext{".contour"};
};

// ---- VolumeGridRover's .pcd: the vertex count, then every vertex as x y z ----

class pcd_io : public contour_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return false; }
  bool can_write() const override { return true; }

  void write(app &, const contour_stack &s, const std::string &filename) const override {
    std::string out = std::to_string(s.vertex_count()) + "\n";
    for (const auto &sl : s.slices) {
      const double z = s.slice_z(sl.first);
      for (const contour_polygon *poly : slice_polygons(s, sl.second))
        for (std::size_t i = 0; i < poly->size(); ++i)
          out += num(poly->xy[2 * i]) + " " + num(poly->xy[2 * i + 1]) + " " + num(z) + "\n";
    }
    write_text("point cloud '" + filename + "'", fs::path(filename), out);
  }

private:
  std::string _id = "vgr_pcd : v1";
  std::vector<std::string> _ext{".pcd"};
};

// ---- libcvc's own: JSON ----
//
//   {"format": "cvc.contour_stack", "version": 1, "z_spacing": 0.05, "z_origin": 0,
//    "components": [{"name": "a", "color": [1, 0, 0], "visible": true}],
//    "slices": [{"index": 3, "contours": {"a": [[x0, y0, x1, y1, ...], ...]}},
//               {"index": 4, "z": 0.25, "contours": {}}]}
//
// Numbers are written in their shortest round-trip form, so a stack reads back bit for bit.

std::string json_string(const std::string &s) {
  std::string out = "\"";
  for (const char ch : s) {
    const unsigned char c = static_cast<unsigned char>(ch);
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (c < 0x20) {
        const char *hex = "0123456789abcdef";
        out += "\\u00";
        out += hex[c >> 4];
        out += hex[c & 15];
      } else {
        out += ch;
      }
    }
  }
  return out + "\"";
}

class cvcc_io : public contour_file_io {
public:
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return true; }

  contour_stack read(app &, const std::string &filename,
                     const contour_read_params &) const override {
    const std::string who = "libcvc contours '" + filename + "'";
    std::istringstream in(read_text(who, fs::path(filename)));
    pt::ptree root;
    try {
      pt::read_json(in, root);
    } catch (const pt::json_parser_error &e) {
      throw contour_file_error(who + ": not JSON (" + e.message() + " at line " +
                               std::to_string(e.line()) + ")");
    }
    try {
      const std::string format = root.get<std::string>("format", "");
      if (format != "cvc.contour_stack")
        throw contour_file_error(who + ": not a libcvc contour stack (format '" + format + "')");
      long long version = 0;
      if (!param_detail::parse_integer(root.get<std::string>("version", ""), version) ||
          version < 1)
        throw contour_file_error(who + ": no version");
      if (version > 1)
        throw contour_file_error(who + ": version " + std::to_string(version) +
                                 " is newer than this reader's 1");
      const pt::ptree none;
      contour_stack s;
      s.z_spacing = number(who, root.get_child("z_spacing"), "z_spacing");
      if (root.get_child_optional("z_origin"))
        s.z_origin = number(who, root.get_child("z_origin"), "z_origin");
      for (const auto &c : root.get_child("components", none)) {
        contour_component comp;
        comp.name = c.second.get<std::string>("name");
        comp.color.clear();
        for (const auto &v : c.second.get_child("color", none))
          comp.color.push_back(number(who, v.second, "component '" + comp.name + "': color"));
        comp.visible = c.second.get<std::string>("visible", "true") != "false";
        s.components.push_back(std::move(comp));
      }
      for (const auto &sl : root.get_child("slices", none)) {
        long long index = 0;
        const std::string is = sl.second.get<std::string>("index", "");
        if (!param_detail::parse_integer(is, index) || index < INT_MIN || index > INT_MAX)
          throw contour_file_error(who + ": a slice's index '" + is + "' is not an integer");
        const int n = static_cast<int>(index);
        if (s.slices.count(n))
          throw contour_file_error(who + ": two slices have index " + std::to_string(n));
        contour_slice &slice = s.slices[n];
        if (sl.second.get_child_optional("z"))
          slice.z = number(who, sl.second.get_child("z"), "slice " + std::to_string(n) + ": z");
        for (const auto &comp : sl.second.get_child("contours", none)) {
          auto &dst = slice.components[comp.first];
          for (const auto &poly : comp.second) {
            contour_polygon p;
            for (const auto &v : poly.second)
              p.xy.push_back(
                  number(who, v.second, "slice " + std::to_string(n) + ", '" + comp.first + "'"));
            dst.push_back(std::move(p));
          }
        }
      }
      return s;
    } catch (const pt::ptree_error &e) {
      throw contour_file_error(who + ": " + e.what());
    }
  }

  void write(app &, const contour_stack &s, const std::string &filename) const override {
    std::string out = "{\n  \"format\": \"cvc.contour_stack\",\n  \"version\": 1,\n"
                      "  \"z_spacing\": " +
                      num(s.z_spacing) + ",\n  \"z_origin\": " + num(s.z_origin) +
                      ",\n  \"components\": [";
    for (std::size_t i = 0; i < s.components.size(); ++i) {
      const contour_component &c = s.components[i];
      out += std::string(i ? "," : "") + "\n    {\"name\": " + json_string(c.name) +
             ", \"color\": [" + num(c.color[0]) + ", " + num(c.color[1]) + ", " + num(c.color[2]) +
             "], \"visible\": " + (c.visible ? "true" : "false") + "}";
    }
    out += s.components.empty() ? "],\n" : "\n  ],\n";
    out += "  \"slices\": [";
    bool first_slice = true;
    for (const auto &sl : s.slices) {
      out += std::string(first_slice ? "" : ",") + "\n    {\"index\": " + std::to_string(sl.first);
      first_slice = false;
      if (!std::isnan(sl.second.z))
        out += ", \"z\": " + num(sl.second.z);
      out += ", \"contours\": {";
      bool first_comp = true;
      for (const contour_component &c : s.components) {
        auto it = sl.second.components.find(c.name);
        if (it == sl.second.components.end())
          continue;
        out += std::string(first_comp ? "" : ",") + "\n      " + json_string(c.name) + ": [";
        first_comp = false;
        for (std::size_t k = 0; k < it->second.size(); ++k) {
          out += std::string(k ? ", " : "") + "[";
          const std::vector<double> &xy = it->second[k].xy;
          for (std::size_t i = 0; i < xy.size(); ++i)
            out += (i ? ", " : "") + num(xy[i]);
          out += "]";
        }
        out += "]";
      }
      out += first_comp ? "}}" : "\n    }}";
    }
    out += s.slices.empty() ? "]\n}\n" : "\n  ]\n}\n";
    write_text("libcvc contours '" + filename + "'", fs::path(filename), out);
  }

private:
  static double number(const std::string &who, const pt::ptree &v, const std::string &what) {
    double d = 0.0;
    if (!finite_number(v.data(), d))
      throw contour_file_error(who + ": " + what + ": '" + v.data() + "' is not a number");
    return d;
  }

  std::string _id = "cvc_contours_json : v1";
  std::vector<std::string> _ext{".cvcc"};
};

} // namespace

std::string extension_of(const std::string &filename) {
  const std::string name = fs::path(filename).filename().string();
  const std::size_t dot = name.find_last_of('.');
  if (dot == std::string::npos || dot == 0)
    return std::string();
  return lower(name.substr(dot));
}

void register_builtin(file_handler_registry<contour_file_io> &r) {
  r.insert(boost::shared_ptr<contour_file_io>(new reconstruct_io));
  r.insert(boost::shared_ptr<contour_file_io>(new tiling_config_io));
  r.insert(boost::shared_ptr<contour_file_io>(new ctr2suf_io));
  r.insert(boost::shared_ptr<contour_file_io>(new pcd_io));
  r.insert(boost::shared_ptr<contour_file_io>(new cvcc_io));
}

} // namespace contour_formats
} // namespace cvc

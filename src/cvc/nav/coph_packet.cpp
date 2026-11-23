/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's packets (C4 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.4); see coph_packet.h.

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coph_packet.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace nav {

namespace {

#if !(defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L)
// One more or one less in the last place of a digit string (empty when it runs out).
std::string step_last_digit(std::string digits, int dir) {
  for (int i = static_cast<int>(digits.size()) - 1; i >= 0; --i) {
    if (dir > 0 && digits[i] == '9') {
      digits[i] = '0';
    } else if (dir < 0 && digits[i] == '0') {
      digits[i] = '9';
    } else {
      digits[i] = static_cast<char>(digits[i] + dir);
      return digits[0] == '0' ? std::string() : digits;
    }
  }
  return std::string();
}
#endif

// The shortest decimal digits that read back as mag (> 0, finite) and the decimal exponent of
// the first: mag ~= d1.d2d3... x 10^exp10.
void shortest_digits(double mag, std::string &digits, int &exp10) {
  digits.clear();
  exp10 = 0;
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[64];
  const std::to_chars_result r =
      std::to_chars(buf, buf + sizeof buf, mag, std::chars_format::scientific);
  const char *c = buf;
  for (; c != r.ptr && *c != 'e'; ++c)
    if (*c >= '0' && *c <= '9')
      digits += *c;
  exp10 = std::atoi(std::string(c + 1, static_cast<const char *>(r.ptr)).c_str());
#else
  // %.*e gives the correctly rounded p digits. At a power of two the rounding interval is
  // lopsided, so the p-digit neighbour may read back when the rounded digits do not (as in
  // state_exec's format_number_shortest_fallback). Candidates are read back as
  // "<digits>e<exp>", with no radix character, so the locale does not matter.
  for (int p = 1; p <= 17 && digits.empty(); ++p) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*e", p - 1, mag);
    std::string m;
    const char *c = buf;
    for (; *c && *c != 'e' && *c != 'E'; ++c)
      if (*c >= '0' && *c <= '9')
        m += *c;
    const int e = *c ? std::atoi(c + 1) : 0;
    const auto reads_back = [&](const std::string &cand) {
      const std::string text = cand + "e" + std::to_string(e - (p - 1));
      return std::strtod(text.c_str(), nullptr) == mag;
    };
    for (const std::string &cand : {m, step_last_digit(m, -1), step_last_digit(m, +1)})
      if (!cand.empty() && reads_back(cand)) {
        digits = cand;
        exp10 = e;
        break;
      }
  }
#endif
  while (digits.size() > 1 && digits.back() == '0')
    digits.pop_back();
}

void write_number(std::string &out, double v) {
  if (!std::isfinite(v))
    throw std::domain_error("coph_packet::json: Out of range float values are not JSON compliant");
  out += coph_repr(v);
}

void write_string(std::string &out, const std::string &s) {
  static const char hex[] = "0123456789abcdef";
  auto unit = [&](unsigned u) {
    out += "\\u";
    out += hex[(u >> 12) & 15];
    out += hex[(u >> 8) & 15];
    out += hex[(u >> 4) & 15];
    out += hex[u & 15];
  };
  out += '"';
  for (std::size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) {
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
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      default:
        if (c < 0x20)
          unit(c);
        else
          out += static_cast<char>(c);
      }
      ++i;
      continue;
    }
    // ensure_ascii: a UTF-8 sequence as its code point, \uXXXX (a surrogate pair above the BMP).
    int len = c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : c >= 0xC0 ? 2 : 1;
    unsigned cp = len == 4 ? c & 0x07 : len == 3 ? c & 0x0F : len == 2 ? c & 0x1F : c;
    if (i + len > s.size())
      len = 1;
    for (int k = 1; k < len; ++k)
      cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
    if (cp >= 0x10000) {
      cp -= 0x10000;
      unit(0xD800 + (cp >> 10));
      unit(0xDC00 + (cp & 0x3FF));
    } else {
      unit(cp);
    }
    i += static_cast<std::size_t>(len);
  }
  out += '"';
}

void write_pair(std::string &out, const coph_vec2 &v) {
  out += '[';
  write_number(out, v[0]);
  out += ',';
  write_number(out, v[1]);
  out += ']';
}

void write_strings(std::string &out, const std::vector<std::string> &v) {
  out += '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      out += ',';
    write_string(out, v[i]);
  }
  out += ']';
}

} // namespace

std::string coph_repr(double v) {
  if (std::isnan(v))
    return "nan";
  if (std::isinf(v))
    return v < 0 ? "-inf" : "inf";
  const std::string sign = std::signbit(v) ? "-" : "";
  if (v == 0)
    return sign + "0.0";
  std::string digits;
  int e = 0;
  shortest_digits(std::fabs(v), digits, e);
  const int n = static_cast<int>(digits.size());
  if (-4 <= e && e < 16) {
    if (e >= n - 1)
      return sign + digits + std::string(static_cast<std::size_t>(e - (n - 1)), '0') + ".0";
    if (e >= 0)
      return sign + digits.substr(0, e + 1) + "." + digits.substr(e + 1);
    return sign + "0." + std::string(static_cast<std::size_t>(-e - 1), '0') + digits;
  }
  std::string out = sign + digits.substr(0, 1);
  if (n > 1)
    out += "." + digits.substr(1);
  const int ae = e < 0 ? -e : e;
  out += std::string(e < 0 ? "e-" : "e+") + (ae < 10 ? "0" : "") + std::to_string(ae);
  return out;
}

std::string coph_packet::json() const {
  // Keys in code-point order: json.dumps(..., sort_keys=True).
  std::string out = "{\"H\":";
  write_number(out, H);
  out += ",\"V_task\":";
  write_number(out, V_task);
  out += ",\"acknowledged_sources\":";
  write_strings(out, acknowledged_sources);
  out += ",\"anchors\":[";
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    const coph_anchor &a = anchors[i];
    out += i ? ",[" : "[";
    const double values[] = {a.x, a.y, a.value, a.gx, a.gy};
    for (int k = 0; k < 5; ++k) {
      if (k)
        out += ',';
      write_number(out, values[k]);
    }
    out += ']';
  }
  out += "],\"bounds\":[";
  for (int k = 0; k < 4; ++k) {
    if (k)
      out += ',';
    write_number(out, bounds[k]);
  }
  out += "],\"convention\":";
  write_string(out, convention);
  out += ",\"covariance\":[";
  for (std::size_t i = 0; i < covariance.size(); ++i) {
    out += i ? ",[" : "[";
    for (std::size_t k = 0; k < covariance[i].size(); ++k) {
      if (k)
        out += ',';
      write_number(out, covariance[i][k]);
    }
    out += ']';
  }
  out += "],\"expires\":";
  write_number(out, expires);
  if (maze) {
    out += ",\"geometry\":[";
    for (std::size_t i = 0; i < geometry.size(); ++i) {
      const coph_geometry_report &g = geometry[i];
      out += i ? ",[" : "[";
      write_number(out, g.shape.x);
      out += ',';
      write_number(out, g.shape.y);
      out += ',';
      write_number(out, g.shape.r);
      if (g.shape.slab) {
        out += ',';
        write_number(out, g.shape.hx);
        out += ',';
        write_number(out, g.shape.hy);
      }
      for (double v : {g.sensor[0], g.sensor[1], g.sensor_radius, g.time}) {
        out += ',';
        write_number(out, v);
      }
      out += ',';
      write_string(out, g.source);
      out += ']';
    }
    out += "],\"geometry_schema\":" + std::to_string(geometry_schema);
  }
  out += ",\"goal\":";
  write_pair(out, goal);
  out += ",\"identity\":";
  write_string(out, identity);
  out += ",\"p\":";
  write_pair(out, p);
  out += ",\"q\":";
  write_pair(out, q);
  out += ",\"recipient\":";
  write_string(out, recipient);
  out += ",\"schema\":" + std::to_string(schema);
  out += ",\"sender\":";
  write_string(out, sender);
  out += ",\"sources\":";
  write_strings(out, sources);
  out += ",\"stage\":" + std::to_string(stage);
  out += ",\"support_points\":[";
  for (std::size_t i = 0; i < support_points.size(); ++i) {
    if (i)
      out += ',';
    write_pair(out, support_points[i]);
  }
  out += "],\"timestamp\":";
  write_number(out, timestamp);
  out += ",\"uncertainty\":";
  write_string(out, uncertainty);
  out += ",\"v\":";
  write_pair(out, v);
  out += ",\"version\":" + std::to_string(version) + "}";
  return out;
}

} // namespace nav
} // namespace cvc

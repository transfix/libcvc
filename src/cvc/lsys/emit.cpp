/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/lsys/emit.h>
#include <cvc/lsys/symbol.h>
#include <sstream>

namespace cvc {
namespace lsys {

stats compute_stats(const ruleset &rs, const word &w, const structure &s) {
  stats st;
  st.modules = w.size();
  st.level_counts = w.level_counts();
  for (std::size_t i = 0; i < w.size(); ++i)
    ++st.by_symbol[rs.syms.name(w[i].sym)];
  st.segments = s.segments.size();
  st.leaves = s.leaves.size();
  st.boxes = s.boxes.size();
  st.paints = s.paints.size();
  return st;
}

std::size_t svg_element_count(const structure &s) {
  return s.segments.size() + s.leaves.size() + s.boxes.size();
}

namespace {

// Project a world point to 2D (u = horizontal, v = vertical, world-up).
void project(const vec3 &p, projection proj, double &u, double &v) {
  switch (proj) {
  case projection::front:
    u = p.x;
    v = p.z;
    break;
  case projection::side:
    u = p.y;
    v = p.z;
    break;
  case projection::top:
    u = p.x;
    v = p.y;
    break;
  }
}

const char *role_color(role r) {
  switch (r) {
  case role::trunk:
  case role::branch:
  case role::wood_solid:
    return "#7a5a34";
  case role::foliage:
    return "#2f8f38";
  case role::rock:
    return "#767068";
  case role::ground:
    return "#8a7a55";
  case role::water:
    return "#2a5a86";
  case role::wall_concrete:
    return "#8f8d88";
  case role::wall_brick:
    return "#9c5a44";
  case role::wall_drywall:
    return "#cfc9bf";
  case role::glass:
    return "#a0c0cc";
  case role::metal:
    return "#6f7276";
  default:
    return "#888888";
  }
}

std::string level_color(std::uint8_t lvl) {
  // simple hue ramp by level
  static const char *ramp[] = {"#3b4cc0", "#5977e3", "#7b9ff9", "#a7c5fe",
                               "#f2cbb7", "#f0a07e", "#e26952", "#b40426"};
  return ramp[lvl % 8];
}

} // namespace

std::string emit_svg(const structure &s, const svg_options &opt) {
  // Compute projected bounds.
  double minu = 1e300, minv = 1e300, maxu = -1e300, maxv = -1e300;
  auto acc = [&](const vec3 &p) {
    double u, v;
    project(p, opt.proj, u, v);
    minu = std::min(minu, u);
    minv = std::min(minv, v);
    maxu = std::max(maxu, u);
    maxv = std::max(maxv, v);
  };
  for (const segment &sg : s.segments) {
    acc(sg.a);
    acc(sg.b);
  }
  for (const leaf &lf : s.leaves)
    acc(lf.pos);
  for (const obox &b : s.boxes) {
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          vec3 c = b.center;
          c.x += sx * b.half.x * b.axis[0].x + sy * b.half.y * b.axis[1].x +
                 sz * b.half.z * b.axis[2].x;
          c.y += sx * b.half.x * b.axis[0].y + sy * b.half.y * b.axis[1].y +
                 sz * b.half.z * b.axis[2].y;
          c.z += sx * b.half.x * b.axis[0].z + sy * b.half.y * b.axis[1].z +
                 sz * b.half.z * b.axis[2].z;
          acc(c);
        }
  }
  if (minu > maxu) {
    minu = minv = 0;
    maxu = maxv = 1;
  }
  const double w = maxu - minu > 1e-9 ? maxu - minu : 1.0;
  const double h = maxv - minv > 1e-9 ? maxv - minv : 1.0;
  const double m = opt.margin_px;
  const double sx = (opt.width_px - 2 * m) / w;
  const int height_px = static_cast<int>((opt.width_px - 2 * m) * (h / w) + 2 * m);
  const double sy = (height_px - 2 * m) / h;

  auto X = [&](double u) { return m + (u - minu) * sx; };
  auto Y = [&](double v) { return m + (maxv - v) * sy; }; // flip: world-up -> screen-down

  std::ostringstream o;
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << opt.width_px << "\" height=\""
    << height_px << "\" viewBox=\"0 0 " << opt.width_px << " " << height_px << "\">\n";
  o << "<rect width=\"100%\" height=\"100%\" fill=\"#101418\"/>\n";
  char buf[256];

  for (const segment &sg : s.segments) {
    double u0, v0, u1, v1;
    project(sg.a, opt.proj, u0, v0);
    project(sg.b, opt.proj, u1, v1);
    std::string col = opt.color == svg_color::klass   ? std::string(role_color(sg.rl))
                      : opt.color == svg_color::level ? level_color(sg.level)
                                                      : std::string("#c8b48a");
    double sw = std::min(30.0, std::max(0.6, 0.5 * (sg.r0 + sg.r1) * ((sx + sy) * 0.5)));
    std::snprintf(buf, sizeof(buf),
                  "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"%s\" "
                  "stroke-width=\"%.2f\" stroke-linecap=\"round\"/>\n",
                  X(u0), Y(v0), X(u1), Y(v1), col.c_str(), sw);
    o << buf;
  }
  for (const leaf &lf : s.leaves) {
    double u, v;
    project(lf.pos, opt.proj, u, v);
    const char *col = opt.color == svg_color::klass ? role_color(lf.rl) : "#2f8f38";
    double rad = std::max(1.0, 0.5 * lf.size * ((sx + sy) * 0.5));
    std::snprintf(
        buf, sizeof(buf),
        "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"%s\" fill-opacity=\"0.55\"/>\n", X(u),
        Y(v), rad, col);
    o << buf;
  }
  for (const obox &b : s.boxes) {
    // Draw the projected bounding rectangle of the 8 corners.
    double a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
    for (int i = -1; i <= 1; i += 2)
      for (int j = -1; j <= 1; j += 2)
        for (int k = -1; k <= 1; k += 2) {
          vec3 c = b.center;
          c.x +=
              i * b.half.x * b.axis[0].x + j * b.half.y * b.axis[1].x + k * b.half.z * b.axis[2].x;
          c.y +=
              i * b.half.x * b.axis[0].y + j * b.half.y * b.axis[1].y + k * b.half.z * b.axis[2].y;
          c.z +=
              i * b.half.x * b.axis[0].z + j * b.half.y * b.axis[1].z + k * b.half.z * b.axis[2].z;
          double u, v;
          project(c, opt.proj, u, v);
          a0 = std::min(a0, u);
          a1 = std::max(a1, u);
          b0 = std::min(b0, v);
          b1 = std::max(b1, v);
        }
    const char *col = role_color(b.rl);
    std::snprintf(buf, sizeof(buf),
                  "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" fill=\"%s\" "
                  "fill-opacity=\"0.7\" stroke=\"#202020\" stroke-width=\"1\"/>\n",
                  X(a0), Y(b1), (a1 - a0) * sx, (b1 - b0) * sy, col);
    o << buf;
  }
  o << "</svg>\n";
  return o.str();
}

std::string emit_off(const structure &s, int sides) {
  if (sides < 3)
    sides = 3;
  std::vector<vec3> V;
  std::vector<std::vector<int>> F;

  auto ortho = [](const vec3 &d, vec3 &e0, vec3 &e1) {
    vec3 up = std::fabs(d.z) < 0.9 ? vec3{0, 0, 1} : vec3{1, 0, 0};
    vec3 a{d.y * up.z - d.z * up.y, d.z * up.x - d.x * up.z, d.x * up.y - d.y * up.x};
    double na = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (na < 1e-9) {
      e0 = {1, 0, 0};
      e1 = {0, 1, 0};
      return;
    }
    e0 = {a.x / na, a.y / na, a.z / na};
    vec3 b{d.y * e0.z - d.z * e0.y, d.z * e0.x - d.x * e0.z, d.x * e0.y - d.y * e0.x};
    double nb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    e1 = {b.x / nb, b.y / nb, b.z / nb};
  };

  for (const segment &sg : s.segments) {
    vec3 d{sg.b.x - sg.a.x, sg.b.y - sg.a.y, sg.b.z - sg.a.z};
    double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (len < 1e-9)
      continue;
    d = {d.x / len, d.y / len, d.z / len};
    vec3 e0, e1;
    ortho(d, e0, e1);
    int base = static_cast<int>(V.size());
    for (int i = 0; i < sides; ++i) {
      double ang = 2.0 * 3.14159265358979323846 * i / sides;
      double cx = std::cos(ang), cy = std::sin(ang);
      V.push_back({sg.a.x + sg.r0 * (cx * e0.x + cy * e1.x),
                   sg.a.y + sg.r0 * (cx * e0.y + cy * e1.y),
                   sg.a.z + sg.r0 * (cx * e0.z + cy * e1.z)});
      V.push_back({sg.b.x + sg.r1 * (cx * e0.x + cy * e1.x),
                   sg.b.y + sg.r1 * (cx * e0.y + cy * e1.y),
                   sg.b.z + sg.r1 * (cx * e0.z + cy * e1.z)});
    }
    for (int i = 0; i < sides; ++i) {
      int a = base + 2 * i, b = base + 2 * i + 1;
      int c = base + 2 * ((i + 1) % sides), dd = base + 2 * ((i + 1) % sides) + 1;
      F.push_back({a, b, dd, c});
    }
  }
  for (const obox &b : s.boxes) {
    int base = static_cast<int>(V.size());
    for (int i = -1; i <= 1; i += 2)
      for (int j = -1; j <= 1; j += 2)
        for (int k = -1; k <= 1; k += 2) {
          vec3 c = b.center;
          c.x +=
              i * b.half.x * b.axis[0].x + j * b.half.y * b.axis[1].x + k * b.half.z * b.axis[2].x;
          c.y +=
              i * b.half.x * b.axis[0].y + j * b.half.y * b.axis[1].y + k * b.half.z * b.axis[2].y;
          c.z +=
              i * b.half.x * b.axis[0].z + j * b.half.y * b.axis[1].z + k * b.half.z * b.axis[2].z;
          V.push_back(c);
        }
    // corner index: bit0=k, bit1=j, bit2=i mapping (i,j,k in {-1,1})
    auto idx = [&](int i, int j, int k) { return base + ((i > 0) << 2 | (j > 0) << 1 | (k > 0)); };
    F.push_back({idx(-1, -1, -1), idx(1, -1, -1), idx(1, 1, -1), idx(-1, 1, -1)});
    F.push_back({idx(-1, -1, 1), idx(1, -1, 1), idx(1, 1, 1), idx(-1, 1, 1)});
    F.push_back({idx(-1, -1, -1), idx(-1, -1, 1), idx(-1, 1, 1), idx(-1, 1, -1)});
    F.push_back({idx(1, -1, -1), idx(1, -1, 1), idx(1, 1, 1), idx(1, 1, -1)});
    F.push_back({idx(-1, -1, -1), idx(-1, -1, 1), idx(1, -1, 1), idx(1, -1, -1)});
    F.push_back({idx(-1, 1, -1), idx(-1, 1, 1), idx(1, 1, 1), idx(1, 1, -1)});
  }

  std::ostringstream o;
  o << "OFF\n" << V.size() << " " << F.size() << " 0\n";
  char buf[128];
  for (const vec3 &v : V) {
    std::snprintf(buf, sizeof(buf), "%.6g %.6g %.6g\n", v.x, v.y, v.z);
    o << buf;
  }
  for (const auto &f : F) {
    o << f.size();
    for (int id : f)
      o << " " << id;
    o << "\n";
  }
  return o.str();
}

} // namespace lsys
} // namespace cvc

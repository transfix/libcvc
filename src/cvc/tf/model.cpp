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
#include <cctype>
#include <cmath>
#include <cvc/tf/model.h>
#include <utility>

namespace cvc {
namespace tf {

namespace {

constexpr int kDefaultResolution = 256;
constexpr int kMaxResolution = 1 << 16;

// The inverted tests send NaN to 0, like the renderers' LUT lookups do.
double clamp01(double v) { return v > 0.0 ? (v < 1.0 ? v : 1.0) : 0.0; }
float clamp01f(float v) { return v > 0.f ? (v < 1.f ? v : 1.f) : 0.f; }

// Normalized -> data units with the ends EXACT: lo + (hi - lo) * 1 can land an ulp short of hi,
// and a lowered endpoint that reads back as 0.9999999999999999 would make normalize() append a
// spurious white endpoint at 1.
double data_at(double t, double lo, double hi) {
  if (t <= 0.0)
    return lo;
  if (t >= 1.0)
    return hi;
  return lo + (hi - lo) * t;
}

// effective_domain's widening, so lowering and from_tables invert each other on any range.
void widen(double &lo, double &hi) {
  if (!(hi > lo))
    hi = lo + 1.0;
}

// Index of the first node past t (std::upper_bound on `at`).
template <class Node> std::size_t upper(const std::vector<Node> &v, double t) {
  return static_cast<std::size_t>(
      std::upper_bound(v.begin(), v.end(), t, [](double x, const Node &n) { return x < n.at; }) -
      v.begin());
}

// Where t falls in a position-sorted list: the bracketing nodes [i0, i1] and the blend u. Clamps
// to the end nodes; never divides by a zero span (t < v[i1].at strictly whenever i0 != i1).
template <class Node>
void locate(const std::vector<Node> &v, double t, std::size_t &i0, std::size_t &i1, double &u) {
  t = clamp01(t);
  u = 0.0;
  if (t <= v.front().at) {
    i0 = i1 = 0;
    return;
  }
  if (t >= v.back().at) {
    i0 = i1 = v.size() - 1;
    return;
  }
  i1 = upper(v, t);
  i0 = i1 - 1;
  u = (t - v[i0].at) / (v[i1].at - v[i0].at);
}

double mix(double a, double b, double u) { return a + (b - a) * u; }

void color_d(const model &m, double t, double rgb[3]) {
  if (m.color.empty()) {
    rgb[0] = rgb[1] = rgb[2] = 0.0;
    return;
  }
  std::size_t i0, i1;
  double u;
  locate(m.color, t, i0, i1, u);
  const color_node &a = m.color[i0], &b = m.color[i1];
  rgb[0] = clamp01(mix(a.r, b.r, u));
  rgb[1] = clamp01(mix(a.g, b.g, u));
  rgb[2] = clamp01(mix(a.b, b.b, u));
}

double alpha_d(const model &m, double t) {
  if (m.alpha.empty())
    return 0.0;
  std::size_t i0, i1;
  double u;
  locate(m.alpha, t, i0, i1, u);
  // Clamp BEFORE cubing, as ColorTable2's getTable did.
  const double a = clamp01(mix(m.alpha[i0].a, m.alpha[i1].a, u));
  return m.opacity_cubed ? a * a * a : a;
}

// Sort by position (stable: the first of equal positions wins, as ColorTable2's position-keyed
// std::set kept the first insert), drop duplicates, insert the 0/1 endpoints by ColorTable2's
// asymmetric rule, then prune what lies outside [0,1]. `low` / `high` are the nodes inserted
// when no node at all lies at or beyond 0 / 1.
template <class Node> void tidy(std::vector<Node> &v, const Node &low, const Node &high) {
  v.erase(std::remove_if(v.begin(), v.end(), [](const Node &n) { return std::isnan(n.at); }),
          v.end());
  std::stable_sort(v.begin(), v.end(), [](const Node &a, const Node &b) { return a.at < b.at; });
  v.erase(
      std::unique(v.begin(), v.end(), [](const Node &a, const Node &b) { return a.at == b.at; }),
      v.end());
  const auto lower = [&v](double x) {
    return std::lower_bound(v.begin(), v.end(), x,
                            [](const Node &n, double y) { return n.at < y; });
  };
  // A missing 0 copies the first node at or past 0 -- even one past 1, which is ColorTable2's
  // behaviour too; with nothing at or past 0 it is `low`.
  auto it = lower(0.0);
  if (it == v.end() || it->at > 0.0) {
    Node n = it == v.end() ? low : *it;
    n.at = 0.0;
    v.insert(it, n);
  }
  // A missing 1 is `high` (white / opacity 1), NOT a copy of the last node, unless a node lies
  // past 1, whose value is then pulled in.
  it = lower(1.0);
  if (it == v.end() || it->at > 1.0) {
    Node n = it == v.end() ? high : *it;
    n.at = 1.0;
    v.insert(it, n);
  }
  v.erase(
      std::remove_if(v.begin(), v.end(), [](const Node &n) { return n.at < 0.0 || n.at > 1.0; }),
      v.end());
  for (Node &n : v)
    if (n.at == 0.0)
      n.at = 0.0; // -0 would print as "-0"
}

// The gap (normalized) from_tables opens between the points of a hard step -- table points at the
// same scalar, which cvc::volren::transfer_function samples as a step -- so normalize(), which
// keeps only the first of equal positions, keeps both. It is the hair tf_binding's separate_steps
// opens (span * 1e-9 in data units) when it lowers a step for the renderers, so lowering gives the
// step back as the tables the renderers already got for it.
constexpr double kStepGap = 1e-9;

// Sort a raw table's nodes by position (stable: table order breaks ties, the order the renderers
// read a step in) and pull each run of equal positions apart by kStepGap, never past the next
// node: the first of the run stays and the rest move up -- or, for a run at or past the top end
// (where moving up would leave [0,1]), the last stays and the rest move down. NaN positions are
// dropped (normalize() would drop them anyway). Positions a few ulps apart can still merge.
template <class Node> void keep_steps(std::vector<Node> &v) {
  v.erase(std::remove_if(v.begin(), v.end(), [](const Node &n) { return std::isnan(n.at); }),
          v.end());
  std::stable_sort(v.begin(), v.end(), [](const Node &a, const Node &b) { return a.at < b.at; });
  for (std::size_t i = 0; i < v.size();) {
    std::size_t j = i + 1;
    while (j < v.size() && v[j].at == v[i].at)
      ++j;
    const std::size_t run = j - i;
    if (run > 1) {
      const double t = v[i].at;
      const bool down = t >= 1.0;
      const double room = down ? (i > 0 ? t - v[i - 1].at : kStepGap * double(run))
                               : (j < v.size() ? v[j].at - t : kStepGap * double(run));
      const double gap = std::min(kStepGap, room / double(run));
      for (std::size_t k = 1; k < run; ++k) {
        if (down)
          v[j - 1 - k].at = t - gap * double(k);
        else
          v[i + k].at = t + gap * double(k);
      }
    }
    i = j;
  }
}

color_node cnode(double at, float r, float g, float b) {
  color_node n;
  n.at = at;
  n.r = r;
  n.g = g;
  n.b = b;
  return n;
}

alpha_node anode(double at, float a) {
  alpha_node n;
  n.at = at;
  n.a = a;
  return n;
}

model make(std::vector<color_node> color, std::vector<alpha_node> alpha) {
  model m;
  m.color = std::move(color);
  m.alpha = std::move(alpha);
  m.normalize();
  return m;
}

// volrover3's starting opacity for its colour presets: a linear ramp with a draggable midpoint.
std::vector<alpha_node> linear_ramp() {
  return {anode(0.0, 0.f), anode(0.5, 0.5f), anode(1.0, 1.f)};
}

std::vector<color_node> rainbow_colors() {
  return {cnode(0.0, 0.f, 0.f, 1.f), cnode(0.25, 0.f, 1.f, 1.f), cnode(0.5, 0.f, 1.f, 0.f),
          cnode(0.75, 1.f, 1.f, 0.f), cnode(1.0, 1.f, 0.f, 0.f)};
}

// --- the JSON reader --------------------------------------------------------------------------

bool fail(std::string *error, const std::string &msg) {
  if (error)
    *error = "tf model: " + msg;
  return false;
}

bool present(const json::value *v) { return v && v->k != json::value::kind::null; }

// `v` as exactly `n` numbers.
bool numbers(const json::value &v, std::size_t n, double *out) {
  if (!v.is_array() || v.arr.size() != n)
    return false;
  for (std::size_t i = 0; i < n; ++i) {
    if (!v.arr[i].is_number())
      return false;
    out[i] = v.arr[i].num;
  }
  return true;
}

bool number_member(const json::value &obj, const char *key, double &out) {
  const json::value *m = obj.find(key);
  if (!m || !m->is_number())
    return false;
  out = m->num;
  return true;
}

// [at,r,g,b] or {"at":t,"rgb":[r,g,b]}
bool read_color(const json::value &v, color_node &n) {
  double d[4];
  if (numbers(v, 4, d)) {
    n = cnode(d[0], float(d[1]), float(d[2]), float(d[3]));
    return true;
  }
  const json::value *rgb = v.find("rgb");
  if (v.is_object() && number_member(v, "at", d[0]) && rgb && numbers(*rgb, 3, d + 1)) {
    n = cnode(d[0], float(d[1]), float(d[2]), float(d[3]));
    return true;
  }
  return false;
}

// [at,a] or {"at":t,"a":a}
bool read_alpha(const json::value &v, alpha_node &n) {
  double d[2];
  if (numbers(v, 2, d) || (v.is_object() && number_member(v, "at", d[0]) &&
                           (number_member(v, "a", d[1]) || number_member(v, "opacity", d[1])))) {
    n = anode(d[0], float(d[1]));
    return true;
  }
  return false;
}

// A bare position, or {"id":..,"at":..[,"rgb":[r,g,b]][,"opacity":o]}.
bool read_isocontour(const json::value &v, isocontour &iso) {
  if (v.is_number()) {
    iso.at = v.num;
    return true;
  }
  if (!v.is_object() || !number_member(v, "at", iso.at))
    return false;
  if (const json::value *id = v.find("id"); present(id)) {
    if (id->is_string())
      iso.id = id->str;
    else if (id->is_number()) // a YAML `id: 3` that went through a generic converter
      json::append_number(iso.id, id->num, 15);
    else
      return false;
  }
  if (const json::value *rgb = v.find("rgb"); present(rgb)) {
    double d[3];
    if (!numbers(*rgb, 3, d))
      return false;
    iso.has_rgb = true;
    iso.r = float(d[0]);
    iso.g = float(d[1]);
    iso.b = float(d[2]);
  }
  if (const json::value *op = v.find("opacity"); present(op)) {
    if (!op->is_number())
      return false;
    iso.has_opacity = true;
    iso.opacity = float(op->num);
  }
  return true;
}

// The roadmap §10.1 raw-table form {"size":N,"rgba":[r,g,b,a, ...]}: one colour and one alpha
// node per entry, entry i at i/(N-1) (the bake convention, so baking at N reproduces the table).
bool read_table(const json::value &t, model &m, std::string *error) {
  const json::value *rgba = t.find("rgba");
  if (!t.is_object() || !rgba || !rgba->is_array() || rgba->arr.size() % 4 != 0 ||
      rgba->arr.empty())
    return fail(error, "table must be {\"size\":N,\"rgba\":[r,g,b,a,...]} with N >= 1 entries");
  const std::size_t n = rgba->arr.size() / 4;
  double size = 0.0;
  if (number_member(t, "size", size) && size != double(n)) {
    std::string declared;
    json::append_number(declared, size, 9);
    return fail(error, "table size " + declared + " does not match its " + std::to_string(n) +
                           " rgba entries");
  }
  for (const json::value &e : rgba->arr)
    if (!e.is_number())
      return fail(error, "table rgba entries must be numbers");
  for (std::size_t i = 0; i < n; ++i) {
    const double at = n > 1 ? double(i) / double(n - 1) : 0.0;
    const json::value *c = &rgba->arr[4 * i];
    m.color.push_back(cnode(at, float(c[0].num), float(c[1].num), float(c[2].num)));
    m.alpha.push_back(anode(at, float(c[3].num)));
  }
  if (n == 1) { // one entry is a constant, not a ramp up to normalize()'s white endpoint
    m.color.push_back(m.color.back());
    m.color.back().at = 1.0;
    m.alpha.push_back(m.alpha.back());
    m.alpha.back().at = 1.0;
  }
  return true;
}

// Positions are written exactly (json::append_exact): a consumer scales them by a data range that
// can be billions wide (a uint32 volume), where 9 digits would move an isovalue or a node a user
// placed in data units by whole units, and an edit that only re-writes the string would then shift
// what the renderers draw. Colours / opacities are bounded perceptual values: 6 digits (<= 5e-7)
// sit far below any LUT's resolution.
void append_position(std::string &s, double at) { json::append_exact(s, at); }

void append_color(std::string &s, const color_node &n) {
  s += '[';
  append_position(s, n.at);
  s += ',';
  json::append_number(s, n.r, 6);
  s += ',';
  json::append_number(s, n.g, 6);
  s += ',';
  json::append_number(s, n.b, 6);
  s += ']';
}

} // namespace

// --- model ------------------------------------------------------------------------------------

model model::default_model() {
  return make({cnode(0.0, 0.f, 0.f, 0.f), cnode(0.5, 1.f, 0.f, 0.f), cnode(0.75, 1.f, 1.f, 0.f),
               cnode(1.0, 0.f, 1.f, 0.f)},
              {anode(0.0, 0.f), anode(0.25, 0.75f), anode(0.75, 0.25f), anode(1.0, 1.f)});
}

model model::preset(const std::string &name) {
  if (name == "grayscale")
    return make({cnode(0.0, 0.f, 0.f, 0.f), cnode(1.0, 1.f, 1.f, 1.f)}, linear_ramp());
  if (name == "rainbow")
    return make(rainbow_colors(), linear_ramp());
  if (name == "hot")
    return make({cnode(0.0, 0.f, 0.f, 0.f), cnode(0.33, 1.f, 0.f, 0.f), cnode(0.66, 1.f, 1.f, 0.f),
                 cnode(1.0, 1.f, 1.f, 1.f)},
                linear_ramp());
  if (name == "cool")
    return make({cnode(0.0, 0.f, 1.f, 1.f), cnode(1.0, 1.f, 0.f, 1.f)}, linear_ramp());
  if (name == "xray") // volrover3's X-Ray was Grayscale; a faint ramp makes it read as one
    return make({cnode(0.0, 0.f, 0.f, 0.f), cnode(1.0, 1.f, 1.f, 1.f)},
                {anode(0.0, 0.f), anode(0.5, 0.125f), anode(1.0, 0.25f)});
  if (name == "isolines") {
    model m = make(rainbow_colors(), {anode(0.0, 0.f), anode(1.0, 0.f)});
    isocontour lo, hi;
    lo.id = "iso1";
    lo.at = 0.25;
    hi.id = "iso2";
    hi.at = 0.75;
    m.isocontours = {lo, hi};
    m.normalize();
    return m;
  }
  return default_model();
}

const std::vector<std::string> &model::preset_names() {
  static const std::vector<std::string> names = {"default", "grayscale", "rainbow", "hot",
                                                 "cool",    "xray",      "isolines"};
  return names;
}

void model::normalize() {
  for (color_node &n : color) {
    n.r = clamp01f(n.r);
    n.g = clamp01f(n.g);
    n.b = clamp01f(n.b);
  }
  for (alpha_node &n : alpha)
    n.a = clamp01f(n.a);
  tidy(color, cnode(0.0, 0.f, 0.f, 0.f), cnode(1.0, 1.f, 1.f, 1.f));
  tidy(alpha, anode(0.0, 0.f), anode(1.0, 1.f));

  for (isocontour &iso : isocontours) {
    iso.at = clamp01(iso.at); // also NaN -> 0 and -0 -> 0
    iso.r = clamp01f(iso.r);
    iso.g = clamp01f(iso.g);
    iso.b = clamp01f(iso.b);
    iso.opacity = clamp01f(iso.opacity);
  }
  // Ids in LIST order, so the earlier of two duplicates keeps its id (a pasted copy is renamed,
  // not the original a renderer already keys on). Every candidate is checked against all ids,
  // later ones included, so a rename never collides and a second pass changes nothing.
  for (std::size_t i = 0; i < isocontours.size(); ++i) {
    isocontour &iso = isocontours[i];
    if (iso.id.empty()) {
      iso.id = next_isocontour_id();
      continue;
    }
    bool dup = false;
    for (std::size_t j = 0; j < i && !dup; ++j)
      dup = isocontours[j].id == iso.id;
    if (!dup)
      continue;
    for (std::size_t k = 2;; ++k) {
      const std::string cand = iso.id + "_" + std::to_string(k);
      if (find_isocontour(cand) < 0) {
        iso.id = cand;
        break;
      }
    }
  }
  std::stable_sort(isocontours.begin(), isocontours.end(),
                   [](const isocontour &a, const isocontour &b) { return a.at < b.at; });

  if (!domain.automatic) {
    if (!std::isfinite(domain.min) || !std::isfinite(domain.max)) {
      domain = domain_spec{};
    } else if (domain.min > domain.max) {
      std::swap(domain.min, domain.max);
    }
  }
  if (resolution < 2)
    resolution = kDefaultResolution;
  else if (resolution > kMaxResolution)
    resolution = kMaxResolution;
}

void model::color_at(double t, float rgb[3]) const {
  double d[3];
  color_d(*this, t, d);
  rgb[0] = float(d[0]);
  rgb[1] = float(d[1]);
  rgb[2] = float(d[2]);
}

float model::alpha_at(double t) const { return float(alpha_d(*this, t)); }

rgba model::sample(double t) const {
  double d[3];
  color_d(*this, t, d);
  rgba s;
  s.r = float(d[0]);
  s.g = float(d[1]);
  s.b = float(d[2]);
  s.a = float(alpha_d(*this, t));
  return s;
}

void model::isocontour_rgb(const isocontour &iso, float rgb[3]) const {
  if (iso.has_rgb) {
    rgb[0] = iso.r;
    rgb[1] = iso.g;
    rgb[2] = iso.b;
  } else {
    color_at(iso.at, rgb);
  }
}

float model::isocontour_opacity(const isocontour &iso) const {
  return iso.has_opacity ? iso.opacity : 1.f;
}

std::vector<rgba> model::bake(int size) const {
  std::vector<rgba> out;
  if (size < 2)
    return out;
  out.resize(static_cast<std::size_t>(size));
  for (int i = 0; i < size; ++i)
    out[static_cast<std::size_t>(i)] = sample(double(i) / double(size - 1));
  return out;
}

std::vector<std::uint8_t> model::bake_rgba8(int size) const {
  std::vector<std::uint8_t> out;
  if (size < 2)
    return out;
  out.resize(static_cast<std::size_t>(size) * 4);
  // Quantized from the double samples, so an entry is round(v * 255) of the exact ramp value.
  const auto q = [](double v) { return static_cast<std::uint8_t>(clamp01(v) * 255.0 + 0.5); };
  for (int i = 0; i < size; ++i) {
    const double t = double(i) / double(size - 1);
    double rgb[3];
    color_d(*this, t, rgb);
    std::uint8_t *e = &out[static_cast<std::size_t>(i) * 4];
    e[0] = q(rgb[0]);
    e[1] = q(rgb[1]);
    e[2] = q(rgb[2]);
    e[3] = q(alpha_d(*this, t));
  }
  return out;
}

std::string model::next_isocontour_id() const {
  for (std::size_t n = 1;; ++n) {
    std::string id = "iso" + std::to_string(n);
    if (find_isocontour(id) < 0)
      return id;
  }
}

int model::find_isocontour(const std::string &id) const {
  for (std::size_t i = 0; i < isocontours.size(); ++i)
    if (isocontours[i].id == id)
      return static_cast<int>(i);
  return -1;
}

// Compares what the model MEANS: an isocontour colour / opacity without its override flag, and
// the bounds of an automatic domain, are not part of it (to_string drops them too).
bool model::operator==(const model &o) const {
  if (color.size() != o.color.size() || alpha.size() != o.alpha.size() ||
      isocontours.size() != o.isocontours.size())
    return false;
  for (std::size_t i = 0; i < color.size(); ++i) {
    const color_node &a = color[i], &b = o.color[i];
    if (a.at != b.at || a.r != b.r || a.g != b.g || a.b != b.b)
      return false;
  }
  for (std::size_t i = 0; i < alpha.size(); ++i)
    if (alpha[i].at != o.alpha[i].at || alpha[i].a != o.alpha[i].a)
      return false;
  for (std::size_t i = 0; i < isocontours.size(); ++i) {
    const isocontour &a = isocontours[i], &b = o.isocontours[i];
    if (a.id != b.id || a.at != b.at || a.has_rgb != b.has_rgb || a.has_opacity != b.has_opacity)
      return false;
    if (a.has_rgb && (a.r != b.r || a.g != b.g || a.b != b.b))
      return false;
    if (a.has_opacity && a.opacity != b.opacity)
      return false;
  }
  if (domain.automatic != o.domain.automatic)
    return false;
  if (!domain.automatic && (domain.min != o.domain.min || domain.max != o.domain.max))
    return false;
  return opacity_cubed == o.opacity_cubed && resolution == o.resolution;
}

void effective_domain(const model &m, bool has_range, double range_lo, double range_hi, double &lo,
                      double &hi) {
  if (!m.domain.automatic && std::isfinite(m.domain.min) && std::isfinite(m.domain.max)) {
    lo = m.domain.min;
    hi = m.domain.max;
  } else if (has_range && std::isfinite(range_lo) && std::isfinite(range_hi)) {
    lo = range_lo;
    hi = range_hi;
  } else {
    lo = 0.0;
    hi = 1.0;
  }
  widen(lo, hi);
}

// --- the state string -------------------------------------------------------------------------

std::string to_string(const model &m) {
  std::string s;
  s.reserve(160 + 48 * (m.color.size() + m.alpha.size() + m.isocontours.size()));
  s += "{\"color\":[";
  for (std::size_t i = 0; i < m.color.size(); ++i) {
    if (i)
      s += ',';
    append_color(s, m.color[i]);
  }
  s += "],\"alpha\":[";
  for (std::size_t i = 0; i < m.alpha.size(); ++i) {
    if (i)
      s += ',';
    s += '[';
    append_position(s, m.alpha[i].at);
    s += ',';
    json::append_number(s, m.alpha[i].a, 6);
    s += ']';
  }
  s += "],\"isocontours\":[";
  for (std::size_t i = 0; i < m.isocontours.size(); ++i) {
    const isocontour &iso = m.isocontours[i];
    if (i)
      s += ',';
    s += "{\"id\":";
    json::append_string(s, iso.id);
    s += ",\"at\":";
    append_position(s, iso.at);
    if (iso.has_rgb) {
      s += ",\"rgb\":[";
      json::append_number(s, iso.r, 6);
      s += ',';
      json::append_number(s, iso.g, 6);
      s += ',';
      json::append_number(s, iso.b, 6);
      s += ']';
    }
    if (iso.has_opacity) {
      s += ",\"opacity\":";
      json::append_number(s, iso.opacity, 6);
    }
    s += '}';
  }
  s += "],\"domain\":";
  if (m.domain.automatic) {
    s += "\"auto\"";
  } else {
    // Data units, not a normalized position: written exactly, since a volume's range can need
    // every digit (a 1e6-wide range at 9 digits would move the ends by whole data units).
    s += '[';
    json::append_exact(s, m.domain.min);
    s += ',';
    json::append_exact(s, m.domain.max);
    s += ']';
  }
  s += ",\"opacity_cubed\":";
  s += m.opacity_cubed ? "true" : "false";
  s += ",\"resolution\":";
  s += std::to_string(m.resolution);
  s += '}';
  return s;
}

bool from_string(const std::string &s, model &out, std::string *error) {
  if (std::all_of(s.begin(), s.end(), [](char c) { return std::isspace((unsigned char)c) != 0; })) {
    out = model::default_model();
    return true;
  }
  json::value root;
  std::string jerr;
  if (!json::parse(s, root, &jerr))
    return fail(error, jerr);
  if (!root.is_object())
    return fail(error, "expected a JSON object");

  // Missing keys keep default_model()'s values, as the scene loader's transfer_function: does,
  // so "", "{}" and a document giving only some keys agree; a key that is present replaces.
  model m = model::default_model();
  if (const json::value *dims = root.find("dims"); present(dims)) {
    if (!dims->is_number())
      return fail(error, "dims must be a number");
    if (dims->num != 1.0) {
      std::string n;
      json::append_number(n, dims->num, 9);
      return fail(error, "dims " + n + " is not supported (only 1D transfer functions are)");
    }
  }
  const json::value *color = root.find("color");
  const json::value *alpha = root.find("alpha");
  if (present(color)) {
    if (!color->is_array())
      return fail(error, "color must be an array");
    m.color.clear();
    for (std::size_t i = 0; i < color->arr.size(); ++i) {
      color_node n;
      if (!read_color(color->arr[i], n))
        return fail(error, "color[" + std::to_string(i) +
                               "] must be [at,r,g,b] or {\"at\":t,\"rgb\":[r,g,b]}");
      m.color.push_back(n);
    }
  }
  if (present(alpha)) {
    if (!alpha->is_array())
      return fail(error, "alpha must be an array");
    m.alpha.clear();
    for (std::size_t i = 0; i < alpha->arr.size(); ++i) {
      alpha_node n;
      if (!read_alpha(alpha->arr[i], n))
        return fail(error, "alpha[" + std::to_string(i) + "] must be [at,a] or {\"at\":t,\"a\":a}");
      m.alpha.push_back(n);
    }
  }
  // The baked form is only a fallback: explicit node lists are the source of truth.
  if (const json::value *table = root.find("table");
      present(table) && !present(color) && !present(alpha)) {
    m.color.clear();
    m.alpha.clear();
    if (!read_table(*table, m, error))
      return false;
  }
  if (const json::value *isos = root.find("isocontours"); present(isos)) {
    if (!isos->is_array())
      return fail(error, "isocontours must be an array");
    for (std::size_t i = 0; i < isos->arr.size(); ++i) {
      isocontour iso;
      if (!read_isocontour(isos->arr[i], iso))
        return fail(error, "isocontours[" + std::to_string(i) +
                               "] must be a position or {\"id\":..,\"at\":t[,\"rgb\":[r,g,b]]"
                               "[,\"opacity\":o]}");
      m.isocontours.push_back(std::move(iso));
    }
  }
  if (const json::value *d = root.find("domain"); present(d)) {
    double mm[2];
    if (d->is_string() && d->str == "auto") {
      m.domain.automatic = true;
    } else if (numbers(*d, 2, mm) || (d->is_object() && number_member(*d, "min", mm[0]) &&
                                      number_member(*d, "max", mm[1]))) {
      m.domain.automatic = false;
      m.domain.min = mm[0];
      m.domain.max = mm[1];
    } else {
      return fail(error, "domain must be \"auto\", [min,max] or {\"min\":lo,\"max\":hi}");
    }
  }
  if (const json::value *c = root.find("opacity_cubed"); present(c)) {
    if (c->k == json::value::kind::boolean)
      m.opacity_cubed = c->b;
    else if (c->is_number())
      m.opacity_cubed = c->num != 0.0;
    else
      return fail(error, "opacity_cubed must be a boolean");
  }
  if (const json::value *r = root.find("resolution"); present(r)) {
    if (!r->is_number())
      return fail(error, "resolution must be a number");
    // Out-of-range values are left for normalize() to repair rather than rejected.
    m.resolution = !(r->num >= 0.0)                  ? 0
                   : r->num > double(kMaxResolution) ? kMaxResolution
                                                     : static_cast<int>(std::lround(r->num));
  }
  m.normalize();
  out = std::move(m);
  return true;
}

// --- lowering ---------------------------------------------------------------------------------

bool transparent(const model &m) {
  return std::none_of(m.alpha.begin(), m.alpha.end(),
                      [](const alpha_node &n) { return n.a > 0.f; });
}

std::vector<double> color_table(const model &m, double lo, double hi) {
  std::vector<double> out;
  if (transparent(m))
    return out;
  out.reserve(m.color.size() * 4);
  for (const color_node &n : m.color) {
    out.push_back(data_at(n.at, lo, hi));
    out.push_back(n.r);
    out.push_back(n.g);
    out.push_back(n.b);
  }
  return out;
}

std::vector<double> opacity_table(const model &m, double lo, double hi, int cubed_samples) {
  std::vector<double> out;
  if (transparent(m))
    return out;
  if (!m.opacity_cubed) {
    out.reserve(m.alpha.size() * 2);
    for (const alpha_node &n : m.alpha) {
      out.push_back(data_at(n.at, lo, hi));
      out.push_back(n.a);
    }
    return out;
  }
  // A cubed ramp is not piecewise-linear in the node positions, so it is resampled -- on a uniform
  // grid MERGED with every node position. The grid alone would cut any feature narrower than its
  // spacing (a one-node opacity spike between two grid points would lower to its sampled flanks);
  // with the nodes in, every node's own (cubed) opacity is in the table, and the renderers'
  // linear interpolation only approximates the cubic between them.
  const int n = cubed_samples > 0 ? std::max(cubed_samples, 2)
                                  : (m.resolution >= 2 ? std::min(m.resolution, kMaxResolution)
                                                       : kDefaultResolution);
  std::vector<double> ts;
  ts.reserve(static_cast<std::size_t>(n) + m.alpha.size());
  for (int i = 0; i < n; ++i)
    ts.push_back(double(i) / double(n - 1));
  for (const alpha_node &a : m.alpha)
    if (a.at >= 0.0 && a.at <= 1.0) // NaN too: positions outside [0,1] only shape the ends
      ts.push_back(a.at);
  std::sort(ts.begin(), ts.end());
  ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
  out.reserve(ts.size() * 2);
  for (const double t : ts) {
    out.push_back(data_at(t, lo, hi));
    out.push_back(alpha_d(m, t));
  }
  return out;
}

std::vector<double> isovalues(const model &m, double lo, double hi) {
  std::vector<double> out;
  out.reserve(m.isocontours.size());
  for (const isocontour &iso : m.isocontours)
    out.push_back(data_at(iso.at, lo, hi));
  return out;
}

model from_tables(const std::vector<double> &ct, const std::vector<double> &ot, double lo,
                  double hi) {
  widen(lo, hi);
  model m;
  for (std::size_t i = 0; i + 3 < ct.size(); i += 4)
    m.color.push_back(
        cnode(to_normalized(ct[i], lo, hi), float(ct[i + 1]), float(ct[i + 2]), float(ct[i + 3])));
  for (std::size_t i = 0; i + 1 < ot.size(); i += 2)
    m.alpha.push_back(anode(to_normalized(ot[i], lo, hi), float(ot[i + 1])));
  keep_steps(m.color);
  keep_steps(m.alpha);
  // An empty table means what it means to the renderers (cvc::volren::transfer_function samples
  // an empty ramp as transparent black), not normalize()'s black->white / 0->1 ramps -- so a
  // transparent model, which lowers to empty tables, comes back transparent.
  if (m.color.empty())
    m.color = {cnode(0.0, 0.f, 0.f, 0.f), cnode(1.0, 0.f, 0.f, 0.f)};
  if (m.alpha.empty())
    m.alpha = {anode(0.0, 0.f), anode(1.0, 0.f)};
  m.normalize();
  return m;
}

model from_coupled_points(const std::vector<double> &value_rgba, double lo, double hi) {
  std::vector<double> ct, ot;
  for (std::size_t i = 0; i + 4 < value_rgba.size(); i += 5) {
    ct.insert(ct.end(), {value_rgba[i], value_rgba[i + 1], value_rgba[i + 2], value_rgba[i + 3]});
    ot.insert(ot.end(), {value_rgba[i], value_rgba[i + 4]});
  }
  return from_tables(ct, ot, lo, hi);
}

std::string format_csv(const std::vector<double> &v) {
  std::string s;
  s.reserve(v.size() * 24);
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      s += ',';
    json::append_number(s, v[i], 17); // the renderer keys' codec: precision 17, exact round trip
  }
  return s;
}

bool parse_csv(const std::string &s, std::vector<double> &out) {
  out.clear();
  if (s.empty())
    return true;
  std::vector<double> vals;
  const char *p = s.data(), *const end = s.data() + s.size();
  for (;;) {
    const char *q = std::find(p, end, ',');
    const char *b = p, *e = q;
    while (b < e && std::isspace(static_cast<unsigned char>(*b)))
      ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(e[-1])))
      --e;
    double v = 0.0;
    if (!json::parse_number(b, e, v))
      return false;
    vals.push_back(v);
    if (q == end)
      break;
    p = q + 1;
  }
  out = std::move(vals);
  return true;
}

} // namespace tf
} // namespace cvc

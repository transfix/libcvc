/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "json.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cvc/geometry/geometry.h>
#include <cvc/state/state.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/contour_mesh.h>
#include <cvc/volume/contour_spectrum.h>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace tf {

namespace {

struct layer_name {
  unsigned bit;
  const char *name;
};
const layer_name kLayers[] = {{overlay_histogram, "histogram"},
                              {overlay_contour_spectrum, "contour_spectrum"},
                              {overlay_contour_tree, "contour_tree"}};

std::string trimmed(const std::string &s) {
  std::size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
    ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    --e;
  return s.substr(b, e - b);
}

// An array of finite numbers.
bool read_numbers(const json::value &v, std::vector<double> &out) {
  if (!v.is_array())
    return false;
  out.clear();
  out.reserve(v.arr.size());
  for (const json::value &e : v.arr) {
    if (!e.is_number() || !std::isfinite(e.num))
      return false;
    out.push_back(e.num);
  }
  return true;
}

void append_numbers(std::string &out, const std::vector<double> &v) {
  out += '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      out += ',';
    json::append_exact(out, v[i]);
  }
  out += ']';
}

// The text of an exception without cvc::exception's "cvc::<name> exception: " preamble.
std::string reason(const std::exception &e) {
  std::string w = e.what();
  const std::string tag = " exception: ";
  const std::size_t at = w.find(tag);
  if (w.compare(0, 5, "cvc::") == 0 && at != std::string::npos)
    w.erase(0, at + tag.size());
  return w;
}

void note_unavailable(std::string &status, const std::string &what) {
  status += status.empty() ? "unavailable: " : "; ";
  status += what;
}

// Call f with `raw` as a pointer to the voxel type; false for Undefined.
template <typename F> bool with_typed(data_type type, const unsigned char *raw, F &&f) {
  switch (type) {
  case UChar:
    f(raw);
    return true;
  case UShort:
    f(reinterpret_cast<const unsigned short *>(raw));
    return true;
  case UInt:
    f(reinterpret_cast<const unsigned int *>(raw));
    return true;
  case Float:
    f(reinterpret_cast<const float *>(raw));
    return true;
  case Double:
    f(reinterpret_cast<const double *>(raw));
    return true;
  case UInt64:
    f(reinterpret_cast<const uint64 *>(raw));
    return true;
  case Char:
    f(reinterpret_cast<const signed char *>(raw));
    return true;
  case Int:
    f(reinterpret_cast<const int *>(raw));
    return true;
  case Int64:
    f(reinterpret_cast<const int64 *>(raw));
    return true;
  case Undefined:
    break;
  }
  return false;
}

// voxels::histogram's binning (bin = trunc((v - lo) / (hi - lo) * (bins - 1)), skipping NaN and
// values outside [lo, hi], e.g. a file-header min/max narrower than the data; do_histogram puts a
// constant volume all in the middle bin) over voxels [b, e) of the raw buffer. It is not
// voxels::histogram itself because that caches into the shared volume, which a worker must not
// write.
template <typename T>
void bin_values(const T *p, uint64 b, uint64 e, double lo, double hi, std::vector<double> &bins) {
  const double range = hi - lo;
  const double last = static_cast<double>(bins.size() - 1);
  const std::size_t top = bins.size() - 1;
  for (uint64 i = b; i < e; ++i) {
    const double v = static_cast<double>(p[i]);
    if (!(v >= lo && v <= hi))
      continue;
    const std::size_t bin = static_cast<std::size_t>(((v - lo) / range) * last);
    bins[std::min(bin, top)] += 1.0;
  }
}

// voxels::calcMinMax's loop over voxels [b, e). It starts min and max at voxel 0 and compares in
// the voxel type; for every type but the 64-bit integers that is the same as comparing in double
// (each value converts exactly, and a NaN compares false either way), so `lo` / `hi` hold the
// running min and max as doubles.
template <typename T> void scan_range(const T *p, uint64 b, uint64 e, double &lo, double &hi) {
  for (uint64 i = b; i < e; ++i) {
    const double v = static_cast<double>(p[i]);
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
  }
}

// UInt64 and Int64 values beyond 2^53 do not convert exactly, so they are compared as calcMinMax
// compares them, in their own type.
template <typename T> void scan_range_exact(const T *p, uint64 b, uint64 e, T &lo, T &hi) {
  for (uint64 i = b; i < e; ++i) {
    const T v = p[i];
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
  }
}

// --- overlay_job's phases -----------------------------------------------------------------------

enum class stage { range, histogram, spectrum, tree_build, tree_simplify, tree_layout, done };

const char *const kPhaseRange = "range: scanning";
const char *const kPhaseHistogram = "histogram: binning";
const char *const kPhaseSpectrum = "contour spectrum";
const char *const kPhaseTree = "contour tree";
const char *const kPhaseSimplify = "contour tree: simplifying";
const char *const kPhaseLayout = "contour tree: laying out";
const char *const kPhaseDone = "done";
const char *const kPhaseCancelled = "cancelled";

struct phase_name {
  const char *builder; // the builder's phase()
  const char *full;    // the job's
};
const phase_name kSpectrumPhases[] = {{"domain", "contour spectrum: domain"},
                                      {"resampling", "contour spectrum: resampling"},
                                      {"integrating", "contour spectrum: integrating"}};
const phase_name kTreePhases[] = {{"preparing", "contour tree: preparing"},
                                  {"sorting", "contour tree: sorting"},
                                  {"join tree", "contour tree: join tree"},
                                  {"split tree", "contour tree: split tree"},
                                  {"merging", "contour tree: merging"}};

template <std::size_t N>
const char *full_phase(const phase_name (&names)[N], const char *builder, const char *fallback) {
  for (const phase_name &p : names)
    if (std::strcmp(p.builder, builder) == 0)
      return p.full;
  return fallback;
}

// A run to the end (budget 0) goes in slices of this many units, so a cancel lands within one.
constexpr std::size_t kRunToEndSlice = std::size_t(1) << 16;
// A paced step (budget > 0) steps the tree builder and simplifier in pieces of at most this many
// units, checking their phase between pieces: a step overruns into a costlier phase by at most
// one piece (~1 ms of sweeping at ~1 us a vertex).
constexpr std::size_t kBuilderPiece = 1024;

std::atomic<std::size_t> g_jobs{0};

// Write `v` to the child `name` unless it already holds it.
void put(state &tf, const char *name, const std::string &v) {
  state &s = tf(name);
  if (s.value() != v)
    s.value(v);
}

std::string get(state &tf, const char *name) {
  state *s = tf.findDescendant(name);
  return s ? s->resolvedValue() : std::string();
}

} // namespace

int tree_overlay::components_at(double value) const {
  if (edges.empty() || !std::isfinite(value))
    return 0;
  const int nv = static_cast<int>(vertices.size());
  double top = -std::numeric_limits<double>::infinity();
  for (const vertex &v : vertices)
    top = std::max(top, v.value);
  int n = 0, at_top = 0;
  for (const auto &e : edges) {
    if (e.first < 0 || e.second < 0 || e.first >= nv || e.second >= nv)
      continue;
    const double a = vertices[static_cast<std::size_t>(e.first)].value;
    const double b = vertices[static_cast<std::size_t>(e.second)].value;
    const double lo = std::min(a, b), hi = std::max(a, b);
    if (lo <= value && value < hi)
      ++n;
    if (hi == top && lo < top)
      ++at_top;
  }
  // The half-open rule leaves the level set at the global maximum with no spanning arc; it is the
  // maxima ending there from below (arcs inside a plateau at the top are pieces of one component),
  // as cvc::contour_tree::components_at counts them.
  return value == top ? at_top : n;
}

unsigned overlay_bits_from_csv(const std::string &csv) {
  unsigned bits = 0;
  std::size_t start = 0;
  while (start <= csv.size()) {
    std::size_t end = csv.find(',', start);
    if (end == std::string::npos)
      end = csv.size();
    const std::string tok = trimmed(csv.substr(start, end - start));
    for (const layer_name &l : kLayers)
      if (tok == l.name)
        bits |= l.bit;
    start = end + 1;
  }
  return bits;
}

std::string overlay_bits_to_csv(unsigned bits) {
  std::string out;
  for (const layer_name &l : kLayers)
    if (bits & l.bit) {
      if (!out.empty())
        out += ',';
      out += l.name;
    }
  return out;
}

std::string range_to_string(double lo, double hi) {
  std::string out;
  json::append_exact(out, lo);
  out += ',';
  json::append_exact(out, hi);
  return out;
}

bool range_from_string(const std::string &s, double &lo, double &hi) {
  const std::string t = trimmed(s);
  if (t.empty())
    return false;
  json::value v;
  if (!json::parse(t[0] == '[' ? t : "[" + t + "]", v))
    return false;
  std::vector<double> n;
  if (!read_numbers(v, n) || n.size() != 2 || n[0] > n[1])
    return false;
  lo = n[0];
  hi = n[1];
  return true;
}

std::string histogram_to_string(const std::vector<double> &bins) {
  std::string out;
  append_numbers(out, bins);
  return out;
}

bool histogram_from_string(const std::string &s, std::vector<double> &bins) {
  json::value v;
  std::vector<double> n;
  bool ok = json::parse(s, v) && read_numbers(v, n);
  for (std::size_t i = 0; ok && i < n.size(); ++i)
    ok = n[i] >= 0.0;
  if (!ok) {
    bins.clear();
    return false;
  }
  bins = std::move(n);
  return true;
}

std::string spectrum_to_string(const spectrum_overlay &s) {
  std::string out = "{\"area\":";
  append_numbers(out, s.area);
  out += ",\"min_volume\":";
  append_numbers(out, s.min_volume);
  out += ",\"max_volume\":";
  append_numbers(out, s.max_volume);
  out += ",\"gradient\":";
  append_numbers(out, s.gradient);
  out += '}';
  return out;
}

bool spectrum_from_string(const std::string &s, spectrum_overlay &out) {
  json::value v;
  spectrum_overlay r;
  bool ok = json::parse(s, v) && v.is_object();
  const struct {
    const char *key;
    std::vector<double> spectrum_overlay::*fn;
  } fns[] = {{"area", &spectrum_overlay::area},
             {"min_volume", &spectrum_overlay::min_volume},
             {"max_volume", &spectrum_overlay::max_volume},
             {"gradient", &spectrum_overlay::gradient}};
  for (const auto &f : fns) {
    if (!ok)
      break;
    const json::value *a = v.find(f.key);
    ok = a && read_numbers(*a, r.*f.fn) && (r.*f.fn).size() == r.area.size();
  }
  if (!ok) {
    out = spectrum_overlay();
    return false;
  }
  out = std::move(r);
  return true;
}

std::string tree_to_string(const tree_overlay &t) {
  std::string out = "{\"vertices\":[";
  for (std::size_t i = 0; i < t.vertices.size(); ++i) {
    if (i)
      out += ',';
    out += '[';
    json::append_exact_float(out, t.vertices[i].x);
    out += ',';
    json::append_exact(out, t.vertices[i].value);
    out += ']';
  }
  out += "],\"edges\":[";
  for (std::size_t i = 0; i < t.edges.size(); ++i) {
    if (i)
      out += ',';
    out += '[' + std::to_string(t.edges[i].first) + ',' + std::to_string(t.edges[i].second) + ']';
  }
  out += "]}";
  return out;
}

bool tree_from_string(const std::string &s, tree_overlay &out) {
  json::value v;
  tree_overlay r;
  bool ok = json::parse(s, v) && v.is_object();
  const json::value *verts = ok ? v.find("vertices") : nullptr;
  const json::value *edges = ok ? v.find("edges") : nullptr;
  ok = verts && edges && verts->is_array() && edges->is_array();
  std::vector<double> pair;
  for (std::size_t i = 0; ok && i < verts->arr.size(); ++i) {
    ok = read_numbers(verts->arr[i], pair) && pair.size() == 2;
    if (ok) {
      tree_overlay::vertex vx;
      vx.x = static_cast<float>(pair[0]);
      vx.value = pair[1];
      r.vertices.push_back(vx);
    }
  }
  const double nv = static_cast<double>(r.vertices.size());
  for (std::size_t i = 0; ok && i < edges->arr.size(); ++i) {
    ok = read_numbers(edges->arr[i], pair) && pair.size() == 2;
    for (int k = 0; ok && k < 2; ++k)
      ok = pair[k] >= 0.0 && pair[k] < nv && pair[k] == std::floor(pair[k]);
    if (ok)
      r.edges.emplace_back(static_cast<int>(pair[0]), static_cast<int>(pair[1]));
  }
  if (!ok) {
    out = tree_overlay();
    return false;
  }
  out = std::move(r);
  return true;
}

// --- overlay_job -------------------------------------------------------------------------------

struct overlay_job::impl {
  unsigned bits = 0;
  overlay_options opt;
  uint64 max_dim = 0;
  // A shallow copy shares the voxels but has its own min/max and histogram caches, so nothing here
  // writes into the caller's object while the render thread may be reading it. Let go of once the
  // last builder holds its own copy.
  std::optional<volume> vol;
  data_type type = Undefined;
  uint64 n = 0;
  // Or a field on a mesh: its values stand in for the voxels (one double each).
  std::shared_ptr<const contour_mesh> mesh;

  std::atomic<bool> cancel_flag{false};
  std::size_t piece = kBuilderPiece; // step_builder's piece (unbounded when running to the end)
  stage st = stage::done;
  bool was_cancelled = false; // a step saw the flag and stopped
  overlays ov;

  // range: calcMinMax's scan, a slice at a time
  bool range_started = false;
  bool range_scans = false; // the sliced scan runs (else the range was set: O(1))
  uint64 pos = 0;
  double dlo = 0.0, dhi = 0.0; // every type but UInt64 and Int64
  uint64 ulo = 0, uhi = 0;     // UInt64
  int64 ilo = 0, ihi = 0;      // Int64
  double lo = 0.0, hi = 0.0;

  // histogram (moved into ov once complete, so result() shows only finished layers)
  bool hist_started = false;
  std::vector<double> bins;

  std::optional<contour_spectrum_builder> spectrum;
  std::optional<contour_tree_builder> tree;
  std::optional<mesh_contour_spectrum_builder> mspectrum;
  std::optional<mesh_contour_tree_builder> mtree;
  std::optional<contour_tree_simplifier> simplifier;
  contour_tree built;

  // progress(): each phase weighted by its expected cost, in voxel-scan units
  double w_range = 0.0, w_hist = 0.0, w_spec = 0.0, w_tree = 0.0, w_total = 0.0;
  double w_done = 0.0; // the weight of the phases behind us
  bool from_mesh = false; // made on a mesh (kept after the mesh is let go of)
  mutable double last_progress = 0.0;

  const volume &cvol() const { return *vol; } // const access: the non-const operator* would COW

  impl(const volume &v, unsigned b, const overlay_options &o) : bits(b & overlay_all), opt(o) {
    max_dim = opt.max_dim > 0 ? static_cast<uint64>(opt.max_dim) : 0;
    vol.emplace(v);
    type = cvol().voxelType();
    n = cvol().XDim() * cvol().YDim() * cvol().ZDim();
    if (n == 0 || type == Undefined) {
      note_unavailable(ov.status, "the volume is empty");
      vol.reset();
      return;
    }
    st = stage::range;

    // Weights: a voxel scanned is ~1 unit; the spectrum and the tree cost a few hundred such units
    // per voxel of the (resampled) grid they work on.
    double m = 1.0;
    for (const uint64 d : {cvol().XDim(), cvol().YDim(), cvol().ZDim()})
      m *= static_cast<double>(max_dim > 0 && d > max_dim ? max_dim : d);
    const bool cached = cvol().minIsSet() && cvol().maxIsSet();
    w_range = cached ? 1.0 : static_cast<double>(n); // calcMinMax scans every voxel type
    w_hist = (bits & overlay_histogram) ? static_cast<double>(n) : 0.0;
    w_spec = (bits & overlay_contour_spectrum) ? 200.0 * m : 0.0;
    w_tree = (bits & overlay_contour_tree) ? 200.0 * m : 0.0;
    w_total = w_range + w_hist + w_spec + w_tree;
  }

  impl(std::shared_ptr<const contour_mesh> field, unsigned b, const overlay_options &o)
      : bits(b & overlay_all), opt(o), mesh(std::move(field)), from_mesh(true) {
    if (!mesh || !mesh->mesh || mesh->values.empty()) {
      note_unavailable(ov.status, "the mesh has no values");
      mesh.reset();
      return;
    }
    type = Double;
    n = mesh->values.size();
    st = stage::range;
    // A value scanned is ~1 unit; a tetrahedron integrated ~40 plus its samples (a triangle ~20),
    // a vertex of the tree ~20 (its sort, sweeps and merge).
    const geometry &g = *mesh->mesh;
    const double elements = !g.const_tets().empty() ? 40.0 * double(g.const_tets().size())
                                                    : 20.0 * double(g.const_tris().size());
    w_range = opt.has_range ? 1.0 : static_cast<double>(n);
    w_hist = (bits & overlay_histogram) ? static_cast<double>(n) : 0.0;
    w_spec = (bits & overlay_contour_spectrum) ? elements : 0.0;
    w_tree = (bits & overlay_contour_tree) ? 20.0 * static_cast<double>(n) : 0.0;
    w_total = w_range + w_hist + w_spec + w_tree;
  }

  const double *values() const { return mesh ? mesh->values.data() : nullptr; }

  // Let go of everything that holds voxels or working memory.
  void release() {
    spectrum.reset();
    tree.reset();
    mspectrum.reset();
    mtree.reset();
    mesh.reset();
    simplifier.reset();
    built = contour_tree();
    bins = std::vector<double>();
    vol.reset();
  }

  void finish() {
    release();
    st = stage::done;
  }

  void stop_cancelled() {
    was_cancelled = true;
    finish();
  }

  // An error outside a layer's own handling (compute_overlays's outer catch): note it and stop.
  void fail(const std::string &why) {
    note_unavailable(ov.status, why);
    finish();
  }

  // Move on past the phase that just ended (`from`), skipping phases that are not requested.
  void next_after(stage from) {
    switch (from) {
    case stage::range:
      w_done += w_range;
      break;
    case stage::histogram:
      w_done += w_hist;
      break;
    case stage::spectrum:
      w_done += w_spec;
      break;
    default:
      w_done += w_tree;
      break;
    }
    stage s = from;
    for (;;) {
      if (s == stage::range)
        s = stage::histogram;
      else if (s == stage::histogram)
        s = stage::spectrum;
      else if (s == stage::spectrum)
        s = stage::tree_build;
      else
        s = stage::done;
      if (s == stage::histogram && !(bits & overlay_histogram))
        continue;
      if (s == stage::spectrum && !(bits & overlay_contour_spectrum))
        continue;
      if (s == stage::tree_build && !(bits & overlay_contour_tree))
        continue;
      break;
    }
    st = s;
    if (st == stage::done)
      finish();
  }

  // Each phase works with up to `left` units and returns what is left of them (a phase that ends
  // returns at least one unit less: its last bookkeeping; the spectrum counts its units exactly, a
  // tree builder or simplifier that finishes ends the step). advance() ends the step at any phase
  // change.
  std::size_t do_range(std::size_t left) {
    if (!range_started) {
      range_started = true;
      if (opt.has_range) {
        lo = std::min(opt.range_lo, opt.range_hi);
        hi = std::max(opt.range_lo, opt.range_hi);
        return end_range(left);
      }
      if (from_mesh) { // the finite values' extrema, a slice at a time
        dlo = HUGE_VAL;
        dhi = -HUGE_VAL;
        range_scans = true;
        pos = 0;
        return left;
      }
      // vol.min()/vol.max() on a shallow copy: the set values when both were set on `vol` (the copy
      // takes them only then), else what calcMinMax would compute -- a scan of every voxel, which
      // runs here a slice at a time instead of in one call.
      if (cvol().minIsSet() && cvol().maxIsSet()) {
        lo = cvol().min();
        hi = cvol().max();
        return end_range(left);
      }
      const unsigned char *raw = *cvol();
      if (!raw) {
        note_unavailable(ov.status, "no voxel data");
        finish();
        return left;
      }
      // calcMinMax's start: voxel 0, exactly, in the voxel type.
      if (type == UInt64)
        ulo = uhi = reinterpret_cast<const uint64 *>(raw)[0];
      else if (type == Int64)
        ilo = ihi = reinterpret_cast<const int64 *>(raw)[0];
      else
        with_typed(type, raw, [&](auto p) { dlo = dhi = static_cast<double>(p[0]); });
      range_scans = true;
      pos = 0;
    }
    if (from_mesh) {
      const uint64 chunk = std::min<uint64>(left, n - pos);
      const double *v = values();
      for (uint64 i = pos; i < pos + chunk; ++i)
        if (std::isfinite(v[i])) {
          dlo = std::min(dlo, v[i]);
          dhi = std::max(dhi, v[i]);
        }
      pos += chunk;
      left -= static_cast<std::size_t>(chunk);
      if (pos < n)
        return left;
      lo = dlo;
      hi = dhi;
      return end_range(left);
    }
    const unsigned char *raw = *cvol();
    const uint64 chunk = std::min<uint64>(left, n - pos);
    if (type == UInt64)
      scan_range_exact(reinterpret_cast<const uint64 *>(raw), pos, pos + chunk, ulo, uhi);
    else if (type == Int64)
      scan_range_exact(reinterpret_cast<const int64 *>(raw), pos, pos + chunk, ilo, ihi);
    else
      with_typed(type, raw, [&](auto p) { scan_range(p, pos, pos + chunk, dlo, dhi); });
    pos += chunk;
    left -= static_cast<std::size_t>(chunk);
    if (pos < n)
      return left;
    lo = type == UInt64 ? static_cast<double>(ulo) : type == Int64 ? static_cast<double>(ilo) : dlo;
    hi = type == UInt64 ? static_cast<double>(uhi) : type == Int64 ? static_cast<double>(ihi) : dhi;
    // The copy (which the builders copy in turn) now caches what calcMinMax would have left in it;
    // the caller's cache is never written.
    vol->min(lo);
    vol->max(hi);
    return end_range(left);
  }

  std::size_t end_range(std::size_t left) {
    if (!std::isfinite(lo) || !std::isfinite(hi)) {
      note_unavailable(ov.status, from_mesh ? "the field has no finite value"
                                            : "the volume's value range is not finite");
      finish();
      return left;
    }
    ov.has_range = true;
    ov.range_lo = lo;
    ov.range_hi = hi;
    next_after(stage::range);
    return left > 0 ? left - 1 : 0;
  }

  std::size_t do_histogram(std::size_t left) {
    if (!hist_started) {
      hist_started = true;
      bins.assign(static_cast<std::size_t>(std::max(opt.histogram_bins, 1)), 0.0);
      if (!(hi > lo)) {
        bins[bins.size() / 2] = static_cast<double>(n);
        return end_histogram(left);
      }
      if (!from_mesh && !*cvol()) {
        bins.clear();
        note_unavailable(ov.status, "histogram: no voxel data");
        next_after(stage::histogram);
        return left;
      }
      pos = 0;
    }
    const uint64 chunk = std::min<uint64>(left, n - pos);
    if (from_mesh)
      bin_values(values(), pos, pos + chunk, lo, hi, bins);
    else
      with_typed(type, *cvol(), [&](auto p) { bin_values(p, pos, pos + chunk, lo, hi, bins); });
    pos += chunk;
    left -= static_cast<std::size_t>(chunk);
    return pos < n ? left : end_histogram(left);
  }

  std::size_t end_histogram(std::size_t left) {
    ov.histogram = std::move(bins);
    bins = std::vector<double>();
    next_after(stage::histogram);
    return left > 0 ? left - 1 : 0;
  }

  std::size_t do_spectrum(std::size_t left) {
    if (from_mesh)
      return do_mesh_spectrum(left);
    try {
      if (!spectrum) {
        contour_spectrum_options so;
        so.samples = static_cast<std::size_t>(std::max(opt.spectrum_samples, 2));
        so.has_domain = true; // the same [lo, hi] as the range and histogram, so the layers align
        so.domain_min = lo;
        so.domain_max = hi;
        so.max_dim = max_dim;
        spectrum.emplace(cvol(), so);
        if (!(bits & overlay_contour_tree))
          vol.reset(); // the builder has its own copy
      }
      const uint64 before = spectrum->work_done();
      if (!spectrum->step(left))
        return 0;
      const uint64 used = spectrum->work_done() - before;
      contour_spectrum cs = spectrum->take();
      spectrum.reset();
      ov.spectrum.area = std::move(cs.surface_area);
      ov.spectrum.min_volume = std::move(cs.min_volume);
      ov.spectrum.max_volume = std::move(cs.max_volume);
      ov.spectrum.gradient = std::move(cs.gradient);
      next_after(stage::spectrum);
      return used >= left ? 0 : left - static_cast<std::size_t>(used);
    } catch (const std::exception &e) {
      spectrum.reset();
      ov.spectrum = spectrum_overlay();
      note_unavailable(ov.status, "contour spectrum: " + reason(e));
      next_after(stage::spectrum);
      return 0;
    }
  }

  std::size_t do_mesh_spectrum(std::size_t left) {
    try {
      if (!mspectrum) {
        contour_spectrum_options so;
        so.samples = static_cast<std::size_t>(std::max(opt.spectrum_samples, 2));
        so.has_domain = true; // aligned with the range and histogram, as on a volume
        so.domain_min = lo;
        so.domain_max = hi;
        mspectrum.emplace(mesh, so);
        if (!(bits & overlay_contour_tree))
          mesh.reset(); // the builder holds it
      }
      const uint64 before = mspectrum->work_done();
      if (!mspectrum->step(left))
        return 0;
      const uint64 used = mspectrum->work_done() - before;
      contour_spectrum cs = mspectrum->take();
      mspectrum.reset();
      ov.spectrum.area = std::move(cs.surface_area);
      ov.spectrum.min_volume = std::move(cs.min_volume);
      ov.spectrum.max_volume = std::move(cs.max_volume);
      ov.spectrum.gradient = std::move(cs.gradient);
      next_after(stage::spectrum);
      return used >= left ? 0 : left - static_cast<std::size_t>(used);
    } catch (const std::exception &e) {
      mspectrum.reset();
      ov.spectrum = spectrum_overlay();
      note_unavailable(ov.status, "contour spectrum: " + reason(e));
      next_after(stage::spectrum);
      return 0;
    }
  }

  // Step a tree builder or simplifier with up to `left` units (`left` keeps what is not used), in
  // pieces of at most `piece` units, stopping after the piece in which its phase changes: a
  // vertex swept costs a few hundred times a key sorted, so a budget sized for one phase must not
  // run on into the next (advance() then ends the step, and a caller pacing by time re-sizes it).
  // True once it is done.
  template <class Builder> bool step_builder(Builder &b, std::size_t &left) {
    const char *const phase0 = b.phase();
    while (left > 0) {
      const std::size_t n = std::min(left, piece);
      const bool finished = b.step(n);
      left -= n;
      if (finished)
        return true;
      if (std::strcmp(b.phase(), phase0) != 0 || cancel_flag.load(std::memory_order_relaxed))
        return false;
    }
    return false;
  }

  void tree_failed(const std::exception &e) {
    tree.reset();
    mtree.reset();
    simplifier.reset();
    built = contour_tree();
    ov.tree = tree_overlay();
    note_unavailable(ov.status, "contour tree: " + reason(e));
    next_after(stage::tree_build);
  }

  std::size_t do_tree(std::size_t left) {
    try {
      if (st == stage::tree_build) {
        if (from_mesh) {
          if (!mtree) {
            mtree.emplace(mesh, contour_tree_options());
            mesh.reset(); // the builder holds it
          }
          if (!step_builder(*mtree, left))
            return left;
          built = mtree->take();
          mtree.reset();
        } else {
          if (!tree) {
            contour_tree_options to;
            to.max_dim = max_dim;
            tree.emplace(cvol(), to);
            vol.reset(); // the builder has its own copy
          }
          if (!step_builder(*tree, left))
            return left;
          built = tree->take();
          tree.reset();
        }
        const std::size_t cap = static_cast<std::size_t>(std::max(opt.tree_max_nodes, 0));
        if (cap > 0 && built.nodes.size() > cap) {
          simplifier.emplace(contour_tree_simplifier::to_nodes(std::move(built), cap));
          built = contour_tree();
          st = stage::tree_simplify;
        } else {
          st = stage::tree_layout;
        }
        return 0;
      }
      if (st == stage::tree_simplify) {
        if (!step_builder(*simplifier, left))
          return left;
        built = simplifier->take();
        simplifier.reset();
        st = stage::tree_layout;
        return 0;
      }
      // tree_layout: one-shot, O(nodes) of the (simplified) tree
      if (built.empty()) {
        note_unavailable(ov.status, from_mesh ? "contour tree: the field is constant"
                                              : "contour tree: the volume is constant");
      } else {
        const std::vector<float> xs = layout_contour_tree(built);
        tree_overlay t;
        t.vertices.resize(built.nodes.size());
        for (std::size_t i = 0; i < built.nodes.size(); ++i) {
          t.vertices[i].x = i < xs.size() ? xs[i] : 0.5f;
          t.vertices[i].value = built.nodes[i].value;
        }
        t.edges.reserve(built.arcs.size());
        for (const contour_tree::arc &a : built.arcs)
          t.edges.emplace_back(static_cast<int>(a.hi), static_cast<int>(a.lo));
        ov.tree = std::move(t);
      }
      built = contour_tree();
      next_after(stage::tree_build);
      return left > 0 ? left - 1 : 0;
    } catch (const std::exception &e) {
      tree_failed(e);
      return 0;
    }
  }

  // One step of up to `budget` (> 0) units; true once done (or cancelled). The step ends where the
  // phase changes (see phase()), so its units are all of one cost.
  bool advance(std::size_t budget) {
    std::size_t left = budget;
    const char *const phase0 = phase();
    while (st != stage::done && left > 0) {
      if (cancel_flag.load(std::memory_order_relaxed)) {
        stop_cancelled();
        break;
      }
      if (phase() != phase0)
        break;
      switch (st) {
      case stage::range:
        left = do_range(left);
        break;
      case stage::histogram:
        left = do_histogram(left);
        break;
      case stage::spectrum:
        left = do_spectrum(left);
        break;
      case stage::tree_build:
      case stage::tree_simplify:
      case stage::tree_layout:
        left = do_tree(left);
        break;
      case stage::done:
        break;
      }
    }
    if (st != stage::done && cancel_flag.load(std::memory_order_relaxed))
      stop_cancelled();
    return st == stage::done;
  }

  const char *phase() const {
    if (cancel_flag.load(std::memory_order_relaxed) || was_cancelled)
      return kPhaseCancelled;
    switch (st) {
    case stage::range:
      return kPhaseRange;
    case stage::histogram:
      return kPhaseHistogram;
    case stage::spectrum:
      if (from_mesh)
        return full_phase(kSpectrumPhases, mspectrum ? mspectrum->phase() : "integrating",
                          kPhaseSpectrum);
      return full_phase(kSpectrumPhases, spectrum ? spectrum->phase() : "domain", kPhaseSpectrum);
    case stage::tree_build:
      if (from_mesh)
        return full_phase(kTreePhases, mtree ? mtree->phase() : "preparing", kPhaseTree);
      return full_phase(kTreePhases, tree ? tree->phase() : "preparing", kPhaseTree);
    case stage::tree_simplify:
      return kPhaseSimplify;
    case stage::tree_layout:
      return kPhaseLayout;
    case stage::done:
      break;
    }
    return kPhaseDone;
  }

  // How far the current phase's layer is, in [0,1].
  double layer_fraction() const {
    switch (st) {
    case stage::range:
      return range_scans && n > 0 ? static_cast<double>(pos) / static_cast<double>(n) : 0.0;
    case stage::histogram:
      return hist_started && n > 0 ? static_cast<double>(pos) / static_cast<double>(n) : 0.0;
    case stage::spectrum:
      return spectrum ? spectrum->progress() : mspectrum ? mspectrum->progress() : 0.0;
    case stage::tree_build:
      return tree ? 0.9 * tree->progress() : mtree ? 0.9 * mtree->progress() : 0.0;
    case stage::tree_simplify:
      return 0.9 + 0.1 * (simplifier ? simplifier->progress() : 0.0);
    case stage::tree_layout:
      return 1.0;
    case stage::done:
      break;
    }
    return 1.0;
  }

  double current_weight() const {
    switch (st) {
    case stage::range:
      return w_range;
    case stage::histogram:
      return w_hist;
    case stage::spectrum:
      return w_spec;
    case stage::tree_build:
    case stage::tree_simplify:
    case stage::tree_layout:
      return w_tree;
    case stage::done:
      break;
    }
    return 0.0;
  }
};

overlay_job::overlay_job(const volume &vol, unsigned bits, const overlay_options &opt)
    : _impl(new impl(vol, bits, opt)) {
  g_jobs.fetch_add(1, std::memory_order_relaxed);
}

overlay_job::overlay_job(std::shared_ptr<const contour_mesh> mesh, unsigned bits,
                         const overlay_options &opt)
    : _impl(new impl(std::move(mesh), bits, opt)) {
  g_jobs.fetch_add(1, std::memory_order_relaxed);
}

overlay_job::~overlay_job() { g_jobs.fetch_sub(1, std::memory_order_relaxed); }

std::size_t overlay_job::instances() { return g_jobs.load(std::memory_order_relaxed); }

bool overlay_job::step(std::size_t budget) {
  impl &m = *_impl;
  if (m.st == stage::done)
    return true;
  try {
    if (budget > 0) {
      m.piece = kBuilderPiece;
      return m.advance(budget);
    }
    m.piece = std::numeric_limits<std::size_t>::max();
    while (!m.advance(kRunToEndSlice)) {
    }
  } catch (const std::exception &e) {
    m.fail(reason(e));
  } catch (...) {
    m.fail("an unknown error");
  }
  return true;
}

bool overlay_job::done() const {
  return _impl->st == stage::done || _impl->cancel_flag.load(std::memory_order_relaxed);
}

void overlay_job::cancel() { _impl->cancel_flag.store(true, std::memory_order_relaxed); }

bool overlay_job::cancelled() const { return _impl->cancel_flag.load(std::memory_order_relaxed); }

unsigned overlay_job::bits() const { return _impl->bits; }

double overlay_job::progress() const {
  const impl &m = *_impl;
  if (m.st == stage::done && !m.was_cancelled)
    return m.last_progress = 1.0;
  if (m.was_cancelled || m.w_total <= 0.0)
    return m.last_progress;
  const double p = (m.w_done + m.current_weight() * m.layer_fraction()) / m.w_total;
  m.last_progress = std::max(m.last_progress, std::min(p, 1.0));
  return m.last_progress;
}

const char *overlay_job::phase() const { return _impl->phase(); }

std::string overlay_job::status() const {
  const impl &m = *_impl;
  const char *ph = m.phase();
  if (ph == kPhaseCancelled)
    return ph;
  if (m.st == stage::done)
    return m.ov.status;
  if (m.st == stage::tree_layout)
    return ph;
  const double f = std::clamp(m.layer_fraction(), 0.0, 1.0);
  const int pct = std::min(99, static_cast<int>(f * 100.0));
  return std::string(ph) + ' ' + std::to_string(pct) + '%';
}

const overlays &overlay_job::result() const { return _impl->ov; }

overlays overlay_job::take() {
  overlays out = std::move(_impl->ov);
  _impl->ov = overlays();
  return out;
}

overlays compute_overlays(const volume &vol, unsigned bits, const overlay_options &opt) {
  try {
    overlay_job job(vol, bits, opt);
    job.step(0);
    return job.take();
  } catch (const std::exception &e) {
    overlays ov;
    note_unavailable(ov.status, reason(e));
    return ov;
  } catch (...) {
    overlays ov;
    note_unavailable(ov.status, "an unknown error");
    return ov;
  }
}

overlays compute_overlays(const contour_mesh &mesh, unsigned bits, const overlay_options &opt) {
  try {
    // Borrowed for the call: an aliasing pointer that owns nothing.
    overlay_job job(
        std::shared_ptr<const contour_mesh>(std::shared_ptr<const contour_mesh>(), &mesh), bits,
        opt);
    job.step(0);
    return job.take();
  } catch (const std::exception &e) {
    overlays ov;
    note_unavailable(ov.status, reason(e));
    return ov;
  } catch (...) {
    overlays ov;
    note_unavailable(ov.status, "an unknown error");
    return ov;
  }
}

void publish_overlays(state &tf, const overlays &ov) {
  if (ov.has_range)
    put(tf, "range", range_to_string(ov.range_lo, ov.range_hi));
  if (!ov.histogram.empty())
    put(tf, "histogram", histogram_to_string(ov.histogram));
  if (!ov.spectrum.empty())
    put(tf, "contour_spectrum", spectrum_to_string(ov.spectrum));
  if (!ov.tree.empty())
    put(tf, "contour_tree", tree_to_string(ov.tree));
  put(tf, "overlay_status", ov.status);
}

void clear_overlays(state &tf, unsigned bits) {
  for (const auto &layer : {std::make_pair(overlay_histogram, "histogram"),
                            std::make_pair(overlay_contour_spectrum, "contour_spectrum"),
                            std::make_pair(overlay_contour_tree, "contour_tree")})
    if ((bits & layer.first) && !get(tf, layer.second).empty())
      put(tf, layer.second, std::string());
}

overlays read_overlays(state &tf) {
  overlays ov;
  ov.has_range = range_from_string(get(tf, "range"), ov.range_lo, ov.range_hi);
  histogram_from_string(get(tf, "histogram"), ov.histogram);
  spectrum_from_string(get(tf, "contour_spectrum"), ov.spectrum);
  tree_from_string(get(tf, "contour_tree"), ov.tree);
  ov.status = get(tf, "overlay_status");
  return ov;
}

} // namespace tf
} // namespace cvc

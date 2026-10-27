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

// mesh_nodes.cpp -- the mesh_lab / fe_lab scene node realizers (see mesh_nodes.h). Each copies the
// forest_trees idiom (seed keys from props, poll them every frame) with node-scoped keys like
// VolRenNode's, and runs the cvc::mesh_ops / cvc::fem kernels on a per-node async_lane.
//
// Threading. The tick runs on the render thread; it marshals every input into a job before the
// submit and applies the job's result after it finishes. A job never touches cvc::state, VTK or
// the runtime object: it captures only its handoff struct and immutable inputs. Capturing the
// runtime would let the last reference drop on the lane thread, and ~async_lane would then join
// itself. Teardown sets each job's cancel flag; a heat solve stops at its next step, a queued job
// returns at once, and the lane's destructor waits for the running one.

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/fem.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/geometry/simplify.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/mesh_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h> // the isovalue as an editable transfer-function isocontour
#include <cvc/state/state.h>
#include <cvc/tf/model.h>          // field colours: a transfer function's colour ramp
#include <cvc/tf/overlays.h>       // ... and the range / histogram its editor shows
#include <cvc/utility/algorithm.h> // cvc::tetrahedralize (CVC_ENABLE_MESHER), element quality
#include <cvc/volume/volume.h>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// A value for a status line: %g, six significant digits.
std::string fmt_value(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

// The region and bounds a mesh job meshes, read from fe.* as fe_lab reads them for the job
// (fe_mesh_signature and launch_fe_job share this, so the two never disagree).
struct FeBounds {
  bool interval = false;
  double isovalue = 0.0, isovalue2 = 0.0;
};
FeBounds read_fe_bounds(cvc::app &a, const std::string &region, const std::string &isovalue,
                        const std::string &isovalue2) {
  FeBounds b;
  b.interval = cvc::ariadne::read_or<std::string>(a, region, "solid") == "interval";
  b.isovalue = cvc::ariadne::read_or<double>(a, isovalue, 0.0);
  b.isovalue2 = cvc::ariadne::read_or<double>(a, isovalue2, b.isovalue);
  return b;
}
// "solid <isovalue>" or "interval <lo> <hi>" (sorted, as the mesher sorts them), exact (%.17g).
std::string bounds_signature(const FeBounds &b) {
  const auto exact = [](double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
  };
  if (!b.interval)
    return "solid " + exact(b.isovalue);
  return "interval " + exact(std::min(b.isovalue, b.isovalue2)) + " " +
         exact(std::max(b.isovalue, b.isovalue2));
}

} // namespace

std::string fe_mesh_signature(cvc::app &app, const std::string &fe_base) {
  using cvc::ariadne::resolve_bind;
  return bounds_signature(read_fe_bounds(app, resolve_bind(fe_base, "mesh.region"),
                                         resolve_bind(fe_base, "mesh.isovalue"),
                                         resolve_bind(fe_base, "mesh.isovalue2")));
}

// --- the interval mesher (fe_lab's mesh.region: interval) ----------------------------------------
//
// One helper for every interval mesh: cvc::tetrahedralize2's interval overload, which meshes the
// shell lo <= v <= hi as the solid min(v - lo, hi - v) >= 0 (LBIE's TETRA2 mode, since
// transfix/libcvc#808), so both isosurfaces are the boundary. The helper only chooses the two
// bounds it passes, by fe_lab's rule (mesh_nodes.h, docs/TRANSFER_FUNCTIONS.md section 6):
//  1. A bound no sample lies beyond -- at or past an end of the value range: a tf_editor bar at the
//     end of its axis, `interval: [0.5, 2]` on a 0/1 mask -- bounds nothing: it adds no sheet and
//     no crease, and the other bound's isosurface lies where the trilinear field crosses it, as a
//     solid mesh's does. It is passed on moved far enough past the data that the other bound's
//     side of min(v - lo, hi - v) wins at every sample. With neither bound passed every sample is
//     inside: the whole box.
//  2. The interval is inclusive. A bound samples sit exactly on (integer or plateau data at an
//     integer bound, an SDF through a grid point) is passed on half way to the nearest value beyond
//     it -- for integer data the half-integer, as a label mesher draws it: [1, 3] on labels is
//     [0.5, 3.5] -- since LBIE counts a sample on its isovalue as outside.
// LBIE reads the samples and the bounds as floats, so the helper compares them in float: "on a
// bound" means equal to it in float.
cvc::geometry tetrahedralize_interval(const cvc::volume &vol, double a, double b, int improve,
                                      int improve_iterations) {
  const double lo = std::min(a, b), hi = std::max(a, b);
  const std::string what = "the interval [" + fmt_value(lo) + ", " + fmt_value(hi) + "]";
  const float flo = static_cast<float>(lo), fhi = static_cast<float>(hi);
  if (!(fhi > flo)) // equal bounds (in float), or a NaN
    throw std::runtime_error("no tets: " + what + " is empty (the two isovalues are equal)");
#ifdef CVC_ENABLE_MESHER
  // Which bounds the data passes, whether samples sit exactly on them, the nearest finite value
  // beyond each, and how many samples lie in the interval.
  bool below = false, above = false, on_lo = false, on_hi = false;
  float vmin = std::numeric_limits<float>::infinity(), vmax = -vmin;
  float next_lo = -std::numeric_limits<float>::infinity(), next_hi = -next_lo;
  std::uint64_t inside = 0;
  const std::uint64_t n = vol.XDim() * vol.YDim() * vol.ZDim();
  for (std::uint64_t i = 0; i < n; ++i) {
    const float v = static_cast<float>(vol(i));
    if (std::isnan(v))
      continue;
    const bool finite = std::isfinite(v);
    if (finite) {
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
    }
    if (v < flo) {
      below = true;
      if (finite)
        next_lo = std::max(next_lo, v);
    } else if (v > fhi) {
      above = true;
      if (finite)
        next_hi = std::min(next_hi, v);
    } else {
      ++inside;
      on_lo = on_lo || v == flo;
      on_hi = on_hi || v == fhi;
    }
  }
  if (inside == 0)
    throw std::runtime_error(
        "no tets: " + what + " does not cut the volume" +
        (vmin <= vmax ? " (its values span [" + fmt_value(vmin) + ", " + fmt_value(vmax) + "])"
                      : std::string()));
  // Rule 2: half way to the nearest value beyond (or that value itself when no float lies
  // between: the samples on the bound stay inside, the ones beyond sit on LBIE's isovalue, out).
  const auto halfway = [](float bound, float beyond) {
    const float m = static_cast<float>(0.5 * (double(bound) + double(beyond)));
    return m > std::min(bound, beyond) && m < std::max(bound, beyond) ? m : beyond;
  };
  double L = on_lo && below && std::isfinite(next_lo) ? halfway(flo, next_lo) : flo;
  double H = on_hi && above && std::isfinite(next_hi) ? halfway(fhi, next_hi) : fhi;
  // Rule 1: H - v >= v - L at every sample for H >= 2 vmax - L (and the same below), so the
  // passed bound alone decides the shape.
  if (!below && !above) {
    const double s = std::max(double(vmax) - double(vmin), 1.0);
    L = double(vmin) - s;
    H = double(vmax) + s;
  } else if (!above) {
    H = 2.0 * double(vmax) - L;
  } else if (!below) {
    L = 2.0 * double(vmin) - H;
  }
  cvc::geometry tm =
      cvc::tetrahedralize2(vol, L, H, cvc::DUALLIB, static_cast<cvc::improvement_method>(improve),
                           cvc::BSPLINE_CONVOLUTION, improve_iterations);
  if (tm.const_tets().empty())
    throw std::runtime_error("no tets: the mesher found nothing in " + what +
                             " (a shell thinner than a voxel?)");
  return tm;
#else
  (void)vol;
  (void)improve;
  (void)improve_iterations;
  throw std::runtime_error("mesher unavailable");
#endif
}

#ifdef CVC_ENABLE_LIBIGL
namespace {

namespace ari = cvc::ariadne;

// --- shared helpers ------------------------------------------------------------------------------

std::string seed_num(cvc::app &a, const std::string &base, const char *key, double def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<double>(a, p, def);
  return p;
}
std::string seed_str(cvc::app &a, const std::string &base, const char *key,
                     const std::string &def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<std::string>(a, p, def);
  return p;
}
std::string seed_bool(cvc::app &a, const std::string &base, const char *key, bool def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_bool_or_seed(a, p, def ? 1 : 0);
  return p;
}
std::string prop_str(const ari::Value &props, const char *key, const char *def) {
  const std::string s = props.str(key);
  return s.empty() ? std::string(def) : s;
}

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
std::string fmt_ms(double ms) { return std::to_string(static_cast<long long>(std::lround(ms))); }
std::string fmt_g(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3g", v);
  return b;
}

// A count or index read from state, where any program or prop can write "nan", "inf" or "1e30":
// NaN is the default and the rest is clamped in double BEFORE the conversion (converting an
// out-of-range double to an integer is undefined behaviour, and a saturated count would hang).
int to_int_clamped(double v, int lo, int hi, int def) {
  if (std::isnan(v))
    return def;
  return static_cast<int>(std::clamp(v, static_cast<double>(lo), static_cast<double>(hi)));
}
std::uint64_t to_count_clamped(double v, double lo, double hi, std::uint64_t def) {
  if (std::isnan(v))
    return def;
  return static_cast<std::uint64_t>(std::clamp(v, lo, hi));
}
double clamp01(double v, double def) { return std::isnan(v) ? def : std::clamp(v, 0.0, 1.0); }

std::size_t tri_count(const cvc::geometry &g) {
  return g.const_tris().size() + 2 * g.const_quads().size();
}

// Min/max of the finite values ((NaN, NaN) when there are none).
std::pair<double, double> finite_range(const std::vector<double> &v) {
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (double x : v)
    if (std::isfinite(x)) {
      lo = std::min(lo, x);
      hi = std::max(hi, x);
    }
  if (lo > hi)
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  return {lo, hi};
}

cvc::colormap_kind colormap_or_viridis(const std::string &name) {
  cvc::colormap_kind k = cvc::colormap_kind::VIRIDIS;
  cvc::colormap_from_string(name, k);
  return k;
}

// The node's `material:` block, as the built-in geometry node applies it.
void apply_material(GeometryNode &g, const ari::SceneNode &n) {
  if (!n.has_material)
    return;
  g.setUseSingleColor(n.use_single_color);
  g.setColor(n.color[0], n.color[1], n.color[2]);
  g.setAmbient(n.ambient);
  g.setDiffuse(n.diffuse);
  if (n.has_specular) {
    g.setSpecular(n.specular);
    g.setSpecularPower(n.specular_power);
  }
}

std::shared_ptr<GeometryNode> make_geometry_node(SceneGraph &sg, GraphicsNode *parent,
                                                 const std::string &id, const cvc::geometry &g) {
  return parent ? parent->createChild<GeometryNode>(id, g)
                : std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(id, g));
}

// Replace a node's mesh. setGeometry re-derives the render mode from the geometry type, so the
// mode the user picked (e.g. a wireframe) is put back unless `keep_mode` is false.
void swap_geometry(GeometryNode &node, const cvc::geometry &g, bool keep_mode = true) {
  const GeometryRenderMode mode = node.getRenderMode();
  node.setGeometry(g);
  if (keep_mode)
    node.setRenderMode(mode);
}

// Show a scalar field, or go back to the material colour. use_single_color wins over a field in
// GeometryNode, so it is switched off while a field shows and restored after.
void show_field(GeometryNode &node, const std::vector<double> &field, cvc::colormap_kind cm,
                double lo, double hi) {
  node.setUseSingleColor(false);
  node.setScalarField(field, cm, lo, hi);
}
void hide_field(GeometryNode &node, bool single_color) {
  if (node.hasScalarField())
    node.clearScalarField();
  node.setUseSingleColor(single_color);
}

// --- field colours -------------------------------------------------------------------------------
//
// The colours a node's scalar field is drawn with, as a transfer function: the cvc::tf::model at
// "<node>.field_colors" (one string; a tf_editor widget edits it). Its colour ramp is baked into
// the table every GeometryNode showing the field maps through (setColorTable); its opacity curve
// and isocontours are not used. A named colormap key (fe_lab's view.colormap, mesh_lab's
// color.colormap) seeds it and resets it to that colormap's ramp whenever the key changes, so a
// document or a combo still picks a colormap, and an edit in the editor stands until the next one.
// An explicit domain in the model is the range the field is coloured over. Beside the key the node
// publishes .range (what the colours span under an `auto` domain) and .histogram (the shown values
// over it): the editor's axis and histogram.
struct FieldColors {
  std::string key;           // <node>.field_colors
  std::string colormap_seen; // the colormap key's value as last followed
  std::string model_seen;    // the model string last taken in
  cvc::tf::model model;      // ... parsed (the last one that parsed)
  std::vector<float> table;  // its colour ramp, baked: model.resolution RGB entries
  std::string published;     // what .range / .histogram were last published from
};

constexpr int kFieldColorBins = 128;

// A named colormap as a transfer function: its ramp sampled at 33 nodes, opaque.
cvc::tf::model colormap_model(cvc::colormap_kind k) {
  constexpr int kNodes = 33;
  std::vector<double> t(kNodes);
  for (int i = 0; i < kNodes; ++i)
    t[std::size_t(i)] = double(i) / double(kNodes - 1);
  const std::vector<unsigned char> rgb = cvc::colormap_rgb(t, k, 0.0, 1.0);
  cvc::tf::model m;
  if (rgb.size() == 3 * t.size())
    for (std::size_t i = 0; i < t.size(); ++i)
      m.color.push_back({t[i], rgb[3 * i] / 255.f, rgb[3 * i + 1] / 255.f, rgb[3 * i + 2] / 255.f});
  m.alpha = {{0.0, 1.f}, {1.0, 1.f}};
  m.normalize();
  return m;
}

// Follow the colormap key (a change resets the model to that colormap's ramp) and the model (a
// change re-bakes the table). True when the table or the model's domain changed; a model that does
// not parse keeps the last good colours.
bool follow_field_colors(cvc::app &a, FieldColors &fc, const std::string &colormap_key) {
  const std::string cm = ari::read_or<std::string>(a, colormap_key, "viridis");
  if (cm != fc.colormap_seen) {
    fc.colormap_seen = cm;
    ari::write<std::string>(a, fc.key, cvc::tf::to_string(colormap_model(colormap_or_viridis(cm))));
  }
  const std::string s = ari::read_or<std::string>(a, fc.key, "");
  if (s == fc.model_seen)
    return false;
  fc.model_seen = s;
  cvc::tf::model m;
  if (!cvc::tf::from_string(s, m))
    return false;
  m.normalize();
  const cvc::tf::domain_spec &d0 = fc.model.domain, &d1 = m.domain;
  const bool domain_moved =
      d0.automatic != d1.automatic || (!d1.automatic && (d0.min != d1.min || d0.max != d1.max));
  std::vector<float> table;
  for (const cvc::tf::rgba &c : m.bake(m.resolution))
    table.insert(table.end(), {c.r, c.g, c.b});
  const bool changed = fc.table.empty() || table != fc.table || domain_moved;
  fc.model = std::move(m);
  fc.table = std::move(table);
  return changed;
}

// Seed <node>.field_colors with the colormap's ramp when it is empty (a value already there -- a
// restored session, a hot reload's predecessor -- stands) and take it in.
void init_field_colors(cvc::app &a, FieldColors &fc, const std::string &key,
                       const std::string &colormap_key) {
  fc.key = key;
  fc.colormap_seen = ari::read_or<std::string>(a, colormap_key, "viridis");
  if (ari::read_or<std::string>(a, key, "").empty())
    ari::write<std::string>(
        a, key, cvc::tf::to_string(colormap_model(colormap_or_viridis(fc.colormap_seen))));
  follow_field_colors(a, fc, colormap_key);
}

// The range a field whose colours would otherwise span [lo, hi] is coloured over: the model's
// explicit domain, else [lo, hi]. And a signature of that choice, for a view's colour signature.
std::pair<double, double> field_colors_range(const FieldColors &fc, double lo, double hi) {
  if (!fc.model.domain.automatic)
    return {fc.model.domain.min, fc.model.domain.max};
  return {lo, hi};
}
std::string field_colors_sig(const FieldColors &fc) {
  return fc.model.domain.automatic
             ? std::string("auto")
             : cvc::tf::range_to_string(fc.model.domain.min, fc.model.domain.max);
}

// Publish .range = [lo, hi] (the colours' span under an `auto` domain) and .histogram (`values`
// over it, kFieldColorBins bins) beside the key, unless they were last published from the same
// range and `sig` (what names the values).
void publish_field_colors(cvc::app &a, FieldColors &fc, const std::vector<double> &values,
                          double lo, double hi, const std::string &sig) {
  if (!std::isfinite(lo) || !std::isfinite(hi))
    return;
  if (hi < lo)
    std::swap(lo, hi);
  const std::string p = cvc::tf::range_to_string(lo, hi) + "|" + sig;
  if (p == fc.published)
    return;
  fc.published = p;
  cvc::tf::overlays ov;
  ov.has_range = true;
  ov.range_lo = lo;
  ov.range_hi = hi;
  ov.histogram.assign(kFieldColorBins, 0.0);
  const double span = hi - lo;
  for (const double v : values) {
    if (!std::isfinite(v) || v < lo || v > hi)
      continue;
    const int b = span > 0.0 ? std::min(kFieldColorBins - 1,
                                        static_cast<int>((v - lo) / span * kFieldColorBins))
                             : kFieldColorBins / 2;
    ov.histogram[std::size_t(b)] += 1.0;
  }
  try {
    cvc::tf::publish_overlays(cvc::state::instance(a)(fc.key), ov);
  } catch (const std::exception &) {
  }
}

// Show or hide a child of a node. The document can bind only the node's own visibility, so a
// hidden parent hides its children too. The key write is a no-op when unchanged, and
// SceneNode::propagateVisible flips the child's flag without touching its key (showing the
// parent again forces the child on), so reconcile the flag itself as well.
void show_child(cvc::app &a, GeometryNode *child, bool want) {
  if (!child)
    return;
  ari::write<int>(a, child->stateName("visible"), want ? 1 : 0);
  if (child->isVisible() != want)
    child->setVisible(want);
}

// The wall time of a node's last run of work: from the first command taken while idle to idle
// again (every chained command and the jobs they start), published as stats.work_ms.
struct WorkTimer {
  bool running = false;
  std::chrono::steady_clock::time_point t0;
  void start() {
    if (!running) {
      running = true;
      t0 = std::chrono::steady_clock::now();
    }
  }
  // At the end of a tick: when the node has gone idle, publish the time and stop.
  void settle(cvc::app &a, bool idle, const std::string &key) {
    if (running && idle) {
      running = false;
      ari::write<double>(a, key, std::round(ms_since(t0)));
    }
  }
};

// Run queued work on the render thread when the lane has no thread (single-threaded wasm).
void pump(cvc::async_lane &lane) {
  if (!lane.threaded())
    lane.run_deferred(1);
}

// --- requests ------------------------------------------------------------------------------------

// A request key holds one command or several separated by spaces ("load smooth"). The tick takes
// the first and leaves the rest in the key, so each runs after the job before it finishes.
std::string first_word(const std::string &s, std::string *rest) {
  const std::size_t b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {
    rest->clear();
    return std::string();
  }
  const std::size_t e = s.find_first_of(" \t", b);
  const std::size_t r = e == std::string::npos ? e : s.find_first_not_of(" \t", e);
  *rest = r == std::string::npos ? std::string() : s.substr(r);
  return s.substr(b, e == std::string::npos ? e : e - b);
}

// The first command waiting in `key`, consumed ("" when none waits).
std::string take_request(cvc::app &a, const std::string &key) {
  const std::string req = ari::read_or<std::string>(a, key, "");
  if (req.empty())
    return req;
  std::string rest;
  const std::string first = first_word(req, &rest);
  ari::write<std::string>(a, key, rest);
  return first;
}

// --- sources -------------------------------------------------------------------------------------

// Where a node's data comes from, as its source.* keys spell it. A relative `file` resolves
// against `base_dir` (the document's directory).
struct SourceSpec {
  std::string type; // fe_lab: sdf | volume; mesh_lab: mesh
  std::string file;
  int dim = 0;           // sdf: the grid's N; volume: resample to N^3 (0 = as stored)
  std::string algorithm; // sdf: v1 | v2 | igl
  char up = 'z';         // the dataset's up axis, turned to +z
  bool flip = false;     // then turned upside down
  double height = 0.0;   // > 0: stood on z = 0, centred, its largest extent scaled to this
  std::string base_dir;  // where a relative file resolves
  std::string sig() const {
    return type + "|" + file + "|" + std::to_string(dim) + "|" + algorithm + "|" + up + "|" +
           (flip ? "1" : "0") + "|" + std::to_string(height) + "|" + base_dir;
  }
};

char up_axis(const std::string &s) {
  const char c =
      s.empty() ? 'z' : static_cast<char>(std::tolower(static_cast<unsigned char>(s[0])));
  return (c == 'x' || c == 'y') ? c : 'z';
}

// A source path from a node's key: a URI, an absolute path, or a path relative to the document's
// directory. A relative path found only in the working directory (where a document loaded from a
// string, or run from the repository root, finds its data) is read from there. The built-in
// meshes (stanford.bunny) are read by their extension, so they need no file at all.
ari::ResolvedFile resolve_source_path(const std::string &uri, const std::string &base_dir) {
  ari::ResolvedFile rf = ari::resolve_to_file(uri, base_dir);
  if (rf.ok && !base_dir.empty()) {
    std::error_code ec;
    if (!std::filesystem::exists(rf.path, ec)) {
      ari::ResolvedFile cwd = ari::resolve_to_file(uri, std::string());
      if (cwd.ok && std::filesystem::exists(cwd.path, ec))
        return cwd;
    }
  }
  return rf;
}

// A source's mesh, read and placed: turned to z-up (and flipped), then fitted when height > 0.
cvc::geometry load_source_mesh(const SourceSpec &s) {
  if (s.file.empty())
    throw std::runtime_error("source.file is empty");
  const ari::ResolvedFile f = resolve_source_path(s.file, s.base_dir);
  if (!f.ok)
    throw std::runtime_error(f.error);
  cvc::geometry g = cvc::read_geometry(f.path);
  g = s.height > 0.0 ? fit_to_ground(g, s.up, s.flip, static_cast<float>(s.height))
                     : turn_up(g, s.up, s.flip);
  if (g.const_normals().size() != g.const_points().size())
    cvc::compute_vertex_normals(g);
  return g;
}

// Where a volume source's elements are put: its up axis turned to +z (and flipped), then scaled
// by `s` and moved by `t`. A volume has no mesh to fit before it is meshed, so fit: is applied to
// the elements; an SDF source's mesh was fitted already, and it stays the identity.
struct Placement {
  char up = 'z';
  bool flip = false;
  double s = 1.0;
  double t[3] = {0.0, 0.0, 0.0};
  bool identity() const {
    return up == 'z' && !flip && s == 1.0 && t[0] == 0.0 && t[1] == 0.0 && t[2] == 0.0;
  }
  void apply(const double in[3], double out[3]) const {
    turn_up_point(up, flip, in, out);
    for (int k = 0; k < 3; ++k)
      out[k] = s * out[k] + t[k];
  }
  void apply(cvc::geometry &g) const {
    if (identity())
      return;
    for (auto &p : g.points()) {
      const double in[3] = {p[0], p[1], p[2]};
      double o[3];
      apply(in, o);
      p = {{o[0], o[1], o[2]}};
    }
  }
};

// The placement that turns a volume's box to z-up and, when height > 0, fits it like fit:.
Placement fit_box(const cvc::bounding_box &b, char up, bool flip, double height) {
  Placement P;
  P.up = up;
  P.flip = flip;
  double lo[3], hi[3];
  for (int k = 0; k < 3; ++k) {
    lo[k] = std::numeric_limits<double>::infinity();
    hi[k] = -lo[k];
  }
  for (int i = 0; i < 8; ++i) {
    const double c[3] = {(i & 1) ? b.maxx : b.minx, (i & 2) ? b.maxy : b.miny,
                         (i & 4) ? b.maxz : b.minz};
    double w[3];
    turn_up_point(up, flip, c, w);
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], w[k]);
      hi[k] = std::max(hi[k], w[k]);
    }
  }
  if (height > 0.0) {
    const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
    P.s = ext > 0.0 ? height / ext : 1.0;
    P.t[0] = -0.5 * (lo[0] + hi[0]) * P.s;
    P.t[1] = -0.5 * (lo[1] + hi[1]) * P.s;
    P.t[2] = -lo[2] * P.s;
  }
  return P;
}

// The placement that fits points already turned z-up like fit: (none when height <= 0).
Placement fit_points(const cvc::geometry::points_t &P, double height) {
  Placement F;
  if (!(height > 0.0) || P.empty())
    return F;
  double lo[3], hi[3];
  for (int k = 0; k < 3; ++k) {
    lo[k] = std::numeric_limits<double>::infinity();
    hi[k] = -lo[k];
  }
  for (const auto &p : P)
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  F.s = ext > 0.0 ? height / ext : 1.0;
  F.t[0] = -0.5 * (lo[0] + hi[0]) * F.s;
  F.t[1] = -0.5 * (lo[1] + hi[1]) * F.s;
  F.t[2] = -lo[2] * F.s;
  return F;
}

// The twelve edges of a box, as lines.
cvc::geometry box_lines(const cvc::bounding_box &b) {
  cvc::geometry g;
  for (int i = 0; i < 8; ++i)
    g.points().push_back(
        {(i & 1) ? b.maxx : b.minx, (i & 2) ? b.maxy : b.miny, (i & 4) ? b.maxz : b.minz});
  const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                        {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto &l : e)
    g.lines().push_back({static_cast<cvc::index_t>(l[0]), static_cast<cvc::index_t>(l[1])});
  return g;
}

// --- mesh_lab ------------------------------------------------------------------------------------

bool is_curvature_mode(const std::string &m) {
  return m == "mean" || m == "gaussian" || m == "k1" || m == "k2";
}
bool is_color_job_mode(const std::string &m) {
  return is_curvature_mode(m) || m == "geodesic" || m == "modes";
}
bool is_mesh_op(const std::string &op) {
  return op == "smooth" || op == "decimate" || op == "repair" || op == "orient";
}

// The fewest eigenpairs an eigenmode job computes, so stepping through the first few modes of
// one mesh reuses one solve.
constexpr int kMinSurfaceModes = 10;
constexpr int kMinVolumeModes = 4;
constexpr int kMaxModeIndex = 50;

// Everything a job reads, copied out of state on the render thread before the submit.
struct MeshParams {
  cvc::smooth_params smooth;
  std::uint64_t target_faces = 5000;
  cvc::repair_params repair;
  std::string color_mode;
  cvc::index_t source = 0;
  std::shared_ptr<const cvc::geodesic_solver> solver;  // cached for this mesh version, or null
  int mode = 1;                                        // modes: which eigenfunction
  std::shared_ptr<const cvc::fem::eigen_result> eigen; // cached for this mesh version, or null
  SourceSpec src;                                      // load
};

// The lane -> tick handoff. Results are written before `done` is set and read after.
struct MeshJob {
  std::string op; // smooth | decimate | repair | orient | color | load
  std::uint64_t version = 0;
  std::atomic<bool> done{false};
  std::atomic<bool> cancel{false};
  std::shared_ptr<const cvc::geometry> geom; // the new mesh (mesh ops, load)
  std::string source_sig;                    // load: the source it read
  std::vector<double> field;                 // the colour scalar (op == "color")
  double lo = 0, hi = 0;
  std::shared_ptr<const cvc::geodesic_solver> solver;
  std::shared_ptr<const cvc::fem::eigen_result> eigen;
  std::string status;
};

struct MeshKeys {
  std::string request, busy, status, nv, nf;
  std::string smooth_method, smooth_iterations, smooth_lambda, smooth_fix_boundary;
  std::string target_faces, weld, remove_degenerate, remove_unreferenced, orient;
  std::string color_mode, colormap, color_min, color_max, geodesic_source, mode_index;
  std::string source_file, source_up, source_flip, source_height, work_ms;
};

struct MeshLabRT {
  cvc::app *app = nullptr;
  std::weak_ptr<GeometryNode> node;
  MeshKeys k;
  std::string base_dir;
  bool single_color = true; // the material's use_single_color, restored when no field shows
  std::shared_ptr<const cvc::geometry> source, current; // immutable snapshots
  std::string source_sig;    // the source keys `source` was read from ("" for a primitive)
  std::uint64_t version = 0; // bumps on every mesh change
  cvc::index_t top_vertex = 0;
  std::string field_sig; // "<version>|<mode>[|<source or index>]" the shown colour was made for
  std::string colormap;  // the colormap last applied
  FieldColors fc;        // <node>.field_colors: the colours the field maps through
  double field_lo = std::numeric_limits<double>::quiet_NaN(); // the shown field's own range
  double field_hi = std::numeric_limits<double>::quiet_NaN();
  std::shared_ptr<const cvc::geodesic_solver> solver;
  std::uint64_t solver_version = 0;
  std::shared_ptr<const cvc::fem::eigen_result> eigen;
  std::uint64_t eigen_version = 0;
  std::shared_ptr<MeshJob> job;
  WorkTimer work;
  std::unique_ptr<cvc::async_lane> lane; // last: destroyed (drained, joined) before the rest

  ~MeshLabRT() {
    if (job)
      job->cancel = true;
  }
};

SourceSpec mesh_source_from_keys(const MeshLabRT &rt) {
  cvc::app &a = *rt.app;
  SourceSpec s;
  s.type = "mesh";
  s.file = ari::read_or<std::string>(a, rt.k.source_file, "");
  s.up = up_axis(ari::read_or<std::string>(a, rt.k.source_up, "z"));
  s.flip = ari::read_bool_or(a, rt.k.source_flip, 0) != 0;
  const double h = ari::read_or<double>(a, rt.k.source_height, 0.0);
  s.height = std::isfinite(h) && h > 0.0 ? h : 0.0;
  s.base_dir = rt.base_dir;
  return s;
}

void run_mesh_job(MeshJob &job, const cvc::geometry &in, const MeshParams &p) {
  const auto t0 = std::chrono::steady_clock::now();
  try {
    if (job.cancel) {
      job.status = "cancelled";
    } else if (job.op == "load") {
      auto g = std::make_shared<cvc::geometry>(load_source_mesh(p.src));
      if (tri_count(*g) == 0)
        throw std::runtime_error("'" + p.src.file + "' has no triangles");
      job.status = "loaded " + p.src.file + ": " + std::to_string(g->const_points().size()) +
                   " vertices, " + std::to_string(tri_count(*g)) + " faces (" +
                   fmt_ms(ms_since(t0)) + " ms)";
      job.source_sig = p.src.sig();
      job.geom = g;
    } else if (job.op == "color") {
      if (p.color_mode == "geodesic") {
        std::shared_ptr<const cvc::geodesic_solver> s = p.solver;
        if (!s)
          s = std::make_shared<cvc::geodesic_solver>(in);
        job.solver = s;
        job.field = s->distance({p.source});
        const std::pair<double, double> r = finite_range(job.field);
        job.lo = 0.0;
        job.hi = r.second;
      } else if (p.color_mode == "modes") {
        // Laplace-Beltrami eigenfunctions of the surface; mode 0 is the constant.
        std::shared_ptr<const cvc::fem::eigen_result> e = p.eigen;
        if (!e || static_cast<int>(e->vectors.size()) <= p.mode)
          e = std::make_shared<cvc::fem::eigen_result>(cvc::fem::laplacian_eigenmodes(
              in, std::max(p.mode + 1, kMinSurfaceModes), cvc::fem::domain::SURFACE));
        job.eigen = e;
        if (static_cast<int>(e->vectors.size()) <= p.mode)
          throw std::runtime_error("the eigensolver returned only " +
                                   std::to_string(e->vectors.size()) + " modes");
        job.field = e->vectors[static_cast<std::size_t>(p.mode)];
        const std::pair<double, double> r = finite_range(job.field);
        job.lo = r.first;
        job.hi = r.second;
      } else {
        const cvc::curvature_kind kind = p.color_mode == "gaussian" ? cvc::curvature_kind::GAUSSIAN
                                         : p.color_mode == "k1" ? cvc::curvature_kind::MAX_PRINCIPAL
                                         : p.color_mode == "k2" ? cvc::curvature_kind::MIN_PRINCIPAL
                                                                : cvc::curvature_kind::MEAN;
        job.field = cvc::vertex_curvature(in, kind);
        const std::pair<double, double> r = cvc::robust_range(job.field);
        job.lo = r.first;
        job.hi = r.second;
      }
      job.status = "coloured by " +
                   (p.color_mode == "modes" ? "mode " + std::to_string(p.mode) : p.color_mode) +
                   " (" + fmt_ms(ms_since(t0)) + " ms)";
    } else {
      auto g = std::make_shared<cvc::geometry>();
      if (job.op == "decimate") {
        cvc::simplify_params sp;
        sp.target_tris = p.target_faces;
        cvc::simplify_result sr;
        *g = cvc::simplify(in, sp, &sr);
        job.status = "decimated " + std::to_string(sr.in_tris) + " -> " +
                     std::to_string(sr.out_tris) + " faces";
      } else {
        g->copy(in, true);
        if (job.op == "smooth") {
          cvc::smooth(*g, p.smooth);
          job.status = "smoothed (" + std::to_string(p.smooth.iterations) + " iterations)";
        } else if (job.op == "repair") {
          const cvc::repair_report r = cvc::repair(*g, p.repair);
          job.status = "repaired: " + std::to_string(r.vertices_removed) + " vertices and " +
                       std::to_string(r.faces_removed) + " faces removed, " +
                       std::to_string(r.faces_flipped) + " faces flipped";
        } else {
          job.status = "oriented: " + std::to_string(cvc::orient_outward(*g)) + " faces flipped";
        }
      }
      job.geom = g;
      job.status += " (" + fmt_ms(ms_since(t0)) + " ms)";
    }
  } catch (const std::exception &e) {
    job.geom.reset();
    job.field.clear();
    job.status = std::string("error: ") + e.what();
  } catch (...) {
    job.geom.reset();
    job.field.clear();
    job.status = "error: unknown exception";
  }
  job.done.store(true, std::memory_order_release);
}

// geodesic.source: a vertex index (clamped to #V-1), or -1 for the highest vertex (the crown of a
// head, the tip of an ear).
cvc::index_t geodesic_source(MeshLabRT &rt) {
  const std::size_t nv = rt.current->const_points().size();
  const double s = ari::read_or<double>(*rt.app, rt.k.geodesic_source, 0.0);
  if (nv == 0 || std::isnan(s))
    return 0;
  if (s < 0.0)
    return rt.top_vertex;
  return static_cast<cvc::index_t>(std::min(s, static_cast<double>(nv - 1)));
}

int mode_index(cvc::app &a, const std::string &key) {
  return to_int_clamped(ari::read_or<double>(a, key, 1.0), 1, kMaxModeIndex, 1);
}

void launch_mesh_job(MeshLabRT &rt, const std::string &op) {
  cvc::app &a = *rt.app;
  MeshParams p;
  const std::string method = ari::read_or<std::string>(a, rt.k.smooth_method, "cotan");
  p.smooth.method = method == "uniform"  ? cvc::smooth_params::UNIFORM_LAPLACIAN
                    : method == "taubin" ? cvc::smooth_params::TAUBIN
                                         : cvc::smooth_params::COTAN_IMPLICIT;
  p.smooth.iterations =
      to_int_clamped(ari::read_or<double>(a, rt.k.smooth_iterations, 3.0), 1, 1000, 3);
  p.smooth.lambda = ari::read_or<double>(a, rt.k.smooth_lambda, 1e-3);
  p.smooth.fix_boundary = ari::read_bool_or(a, rt.k.smooth_fix_boundary, 0) != 0;
  p.target_faces =
      to_count_clamped(ari::read_or<double>(a, rt.k.target_faces, 5000.0), 1.0, 1e12, 5000);
  p.repair.weld_epsilon = ari::read_bool_or(a, rt.k.weld, 1) ? 0.0 : -1.0;
  p.repair.remove_degenerate = ari::read_bool_or(a, rt.k.remove_degenerate, 1) != 0;
  p.repair.remove_unreferenced = ari::read_bool_or(a, rt.k.remove_unreferenced, 1) != 0;
  p.repair.orient = ari::read_bool_or(a, rt.k.orient, 1) != 0;
  p.color_mode = ari::read_or<std::string>(a, rt.k.color_mode, "none");
  p.source = geodesic_source(rt);
  if (rt.solver && rt.solver_version == rt.version)
    p.solver = rt.solver;
  p.mode = mode_index(a, rt.k.mode_index);
  if (rt.eigen && rt.eigen_version == rt.version)
    p.eigen = rt.eigen;
  if (op == "load")
    p.src = mesh_source_from_keys(rt);

  auto job = std::make_shared<MeshJob>();
  job->op = op;
  job->version = rt.version;
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  if (op != "color")
    ari::write<std::string>(a, rt.k.status,
                            op == "load" ? "loading " + p.src.file + "..." : op + "...");
  std::shared_ptr<const cvc::geometry> in = rt.current;
  if (!rt.lane->submit([job, in, p]() { run_mesh_job(*job, *in, p); })) {
    job->status = "error: the mesh_lab lane is stopped";
    job->done.store(true, std::memory_order_release);
  }
}

void publish_mesh_stats(MeshLabRT &rt) {
  ari::write<double>(*rt.app, rt.k.nv, static_cast<double>(rt.current->const_points().size()));
  ari::write<double>(*rt.app, rt.k.nf, static_cast<double>(tri_count(*rt.current)));
}

// Make `g` the current mesh (its stats published), without touching the node.
void adopt_mesh(MeshLabRT &rt, std::shared_ptr<const cvc::geometry> g) {
  rt.current = std::move(g);
  ++rt.version;
  const auto &P = rt.current->const_points();
  rt.top_vertex = 0;
  for (std::size_t i = 1; i < P.size(); ++i)
    if (P[i][2] > P[rt.top_vertex][2])
      rt.top_vertex = static_cast<cvc::index_t>(i);
  publish_mesh_stats(rt);
}

void set_mesh(MeshLabRT &rt, GeometryNode &node, std::shared_ptr<const cvc::geometry> g) {
  adopt_mesh(rt, std::move(g));
  hide_field(node, rt.single_color); // the colour follower recomputes it for the new mesh
  swap_geometry(node, *rt.current);
}

void apply_mesh_job(MeshLabRT &rt, GeometryNode &node, MeshJob &j) {
  cvc::app &a = *rt.app;
  if (j.op == "color") {
    if (j.solver) {
      rt.solver = j.solver;
      rt.solver_version = j.version;
    }
    if (j.eigen) {
      rt.eigen = j.eigen;
      rt.eigen_version = j.version;
    }
    if (!j.field.empty()) {
      rt.colormap = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
      rt.field_lo = j.lo;
      rt.field_hi = j.hi;
      const std::pair<double, double> r = field_colors_range(rt.fc, j.lo, j.hi);
      show_field(node, j.field, colormap_or_viridis(rt.colormap), r.first, r.second);
      ari::write<double>(a, rt.k.color_min, r.first);
      ari::write<double>(a, rt.k.color_max, r.second);
      publish_field_colors(a, rt.fc, j.field, j.lo, j.hi, rt.field_sig);
    }
  } else if (j.op == "load") {
    if (j.geom) {
      rt.source = j.geom;
      rt.source_sig = j.source_sig;
      set_mesh(rt, node, rt.source);
    }
  } else if (j.geom) {
    set_mesh(rt, node, j.geom);
  }
  ari::write<std::string>(a, rt.k.status, j.status);
  ari::write<int>(a, rt.k.busy, 0);
}

// Follow color.mode / color.colormap / geodesic.source / modes.index: a changed signature
// recomputes the scalar off-thread; a colormap change alone recolours in place.
void follow_mesh_color(MeshLabRT &rt, GeometryNode &node) {
  cvc::app &a = *rt.app;
  const std::string mode = ari::read_or<std::string>(a, rt.k.color_mode, "none");
  std::string sig = std::to_string(rt.version) + "|" + mode;
  if (mode == "geodesic")
    sig += "|" + std::to_string(geodesic_source(rt));
  else if (mode == "modes")
    sig += "|" + std::to_string(mode_index(a, rt.k.mode_index));
  if (sig != rt.field_sig) {
    rt.field_sig = sig;
    if (is_color_job_mode(mode)) {
      launch_mesh_job(rt, "color");
      return;
    }
    if (!mode.empty() && mode != "none")
      ari::write<std::string>(a, rt.k.status, "unknown color.mode '" + mode + "'");
    hide_field(node, rt.single_color);
    rt.field_lo = rt.field_hi = std::numeric_limits<double>::quiet_NaN();
    ari::write<std::string>(a, rt.k.color_min, "-");
    ari::write<std::string>(a, rt.k.color_max, "-");
  }
  // The field's colours: color.colormap resets <node>.field_colors to its ramp, and an edited
  // ramp or domain recolours in place.
  rt.colormap = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
  if (follow_field_colors(a, rt.fc, rt.k.colormap)) {
    node.setColorTable(rt.fc.table);
    if (node.hasScalarField() && std::isfinite(rt.field_lo)) {
      const std::pair<double, double> r = field_colors_range(rt.fc, rt.field_lo, rt.field_hi);
      node.setScalarRange(r.first, r.second);
      ari::write<double>(a, rt.k.color_min, r.first);
      ari::write<double>(a, rt.k.color_max, r.second);
    }
  }
}

// One command from the request key. True when it started a job (the rest waits for it).
bool run_mesh_request(MeshLabRT &rt, GeometryNode &node, const std::string &req) {
  cvc::app &a = *rt.app;
  if (is_mesh_op(req)) {
    launch_mesh_job(rt, req);
    return true;
  }
  if (req == "load" || req == "reload") {
    // load reads the source.* keys again only when they changed; otherwise it is a reset, so a
    // gallery stepping between items of one mesh never re-reads its file.
    const SourceSpec s = mesh_source_from_keys(rt);
    if (s.file.empty()) {
      ari::write<std::string>(a, rt.k.status, "source.file is empty: nothing to load");
    } else if (req == "reload" || s.sig() != rt.source_sig) {
      launch_mesh_job(rt, "load");
      return true;
    } else {
      // Back to the source mesh; when it is still the one shown, keep its caches (an
      // eigendecomposition, a geodesic solver) by leaving the mesh version alone.
      if (rt.current != rt.source)
        set_mesh(rt, node, rt.source);
      ari::write<std::string>(a, rt.k.status, "the source mesh (unchanged)");
    }
  } else if (req == "reset") {
    set_mesh(rt, node, rt.source);
    ari::write<std::string>(a, rt.k.status, "reset to the source mesh");
  } else if (req == "curvature") {
    // Colour by curvature now: switch a non-curvature mode to mean, or recompute this one.
    if (is_curvature_mode(ari::read_or<std::string>(a, rt.k.color_mode, "none")))
      rt.field_sig.clear();
    else
      ari::write<std::string>(a, rt.k.color_mode, "mean");
  } else {
    ari::write<std::string>(a, rt.k.status, "unknown request '" + req + "'");
  }
  return false;
}

void tick_mesh_lab(MeshLabRT &rt) {
  std::shared_ptr<GeometryNode> node = rt.node.lock();
  if (!node)
    return;
  cvc::app &a = *rt.app;
  pump(*rt.lane);
  if (rt.job) {
    if (!rt.job->done.load(std::memory_order_acquire))
      return; // one job at a time; a request waits in its key
    std::shared_ptr<MeshJob> j = std::move(rt.job);
    apply_mesh_job(rt, *node, *j);
  }
  // Instant commands run back to back; one that starts a job leaves the rest waiting.
  for (int i = 0; i < 16; ++i) {
    const std::string req = take_request(a, rt.k.request);
    if (req.empty())
      break;
    rt.work.start();
    if (run_mesh_request(rt, *node, req))
      return;
  }
  follow_mesh_color(rt, *node);
  rt.work.settle(a, !rt.job, rt.k.work_ms);
}

// type: mesh_lab
std::shared_ptr<GraphicsNode> realize_mesh_lab(SceneGraph &sg, const ari::SceneNode &n,
                                               GraphicsNode *parent, RealizedScene &out,
                                               std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: mesh_lab '" + n.id + "': " + m);
  };
  // A file source is read through the same path a `load` request takes (a relative path resolves
  // against the document's directory); a primitive or heightfield like a geometry node.
  const bool from_file = n.source_primitive.empty() && !n.has_heightfield && !n.source_file.empty();
  SourceSpec spec;
  spec.type = "mesh";
  spec.file = from_file ? n.source_file : std::string();
  spec.up = fit_up_axis(n);
  spec.flip = n.fit_flip;
  spec.height = n.has_fit ? n.fit_height : 0.0;
  spec.base_dir = n.base_dir;
  cvc::geometry geom;
  if (from_file) {
    try {
      geom = load_source_mesh(spec);
    } catch (const std::exception &e) {
      warn(e.what());
      return nullptr;
    }
  } else if (!read_node_geometry(sg, n, geom, warnings)) {
    return nullptr;
  }
  if (tri_count(geom) == 0) {
    warn("the source has no triangles");
    return nullptr;
  }
  auto rt = std::make_shared<MeshLabRT>();
  try {
    rt->lane = std::make_unique<cvc::async_lane>("mesh_lab." + n.id);
  } catch (const std::exception &e) {
    warn(std::string("could not start its worker lane: ") + e.what());
    return nullptr;
  }
  std::shared_ptr<GeometryNode> g = make_geometry_node(sg, parent, n.id, geom);
  if (!g)
    return nullptr;
  apply_material(*g, n);
  rt->app = &sg.appContext();
  rt->node = g;
  rt->base_dir = n.base_dir;
  rt->single_color = g->getUseSingleColor();

  cvc::app &a = *rt->app;
  const std::string base = g->stateName("mesh_ops");
  const ari::Value &pr = n.props;
  MeshKeys &k = rt->k;
  k.request = seed_str(a, base, "request", "");
  k.busy = ari::resolve_bind(base, "busy");
  k.status = ari::resolve_bind(base, "status");
  k.nv = ari::resolve_bind(base, "stats.vertices");
  k.nf = ari::resolve_bind(base, "stats.faces");
  k.work_ms = ari::resolve_bind(base, "stats.work_ms");
  k.source_file = seed_str(a, base, "source.file", spec.file);
  k.source_up = seed_str(a, base, "source.up", std::string(1, spec.up));
  k.source_flip = seed_bool(a, base, "source.flip", spec.flip);
  k.source_height = seed_num(a, base, "source.height", spec.height);
  k.smooth_method = seed_str(a, base, "smooth.method", prop_str(pr, "smooth_method", "cotan"));
  k.smooth_iterations = seed_num(a, base, "smooth.iterations", pr.num("smooth_iterations", 3));
  k.smooth_lambda = seed_num(a, base, "smooth.lambda", pr.num("smooth_lambda", 1e-3));
  k.smooth_fix_boundary = seed_bool(a, base, "smooth.fix_boundary", pr.flag("fix_boundary"));
  k.target_faces = seed_num(a, base, "decimate.target_faces", pr.num("target_faces", 5000));
  k.weld = seed_bool(a, base, "repair.weld", pr.flag("weld", true));
  k.remove_degenerate =
      seed_bool(a, base, "repair.remove_degenerate", pr.flag("remove_degenerate", true));
  k.remove_unreferenced =
      seed_bool(a, base, "repair.remove_unreferenced", pr.flag("remove_unreferenced", true));
  k.orient = seed_bool(a, base, "repair.orient", pr.flag("orient", true));
  k.color_mode = seed_str(a, base, "color.mode", prop_str(pr, "color_mode", "none"));
  k.colormap = seed_str(a, base, "color.colormap", prop_str(pr, "colormap", "viridis"));
  k.color_min = ari::resolve_bind(base, "color.min");
  k.color_max = ari::resolve_bind(base, "color.max");
  k.geodesic_source = seed_num(a, base, "geodesic.source", pr.num("geodesic_source", 0));
  k.mode_index = seed_num(a, base, "modes.index", pr.num("mode", 1));

  auto snapshot = std::make_shared<cvc::geometry>();
  snapshot->copy(geom, true);
  rt->source = snapshot;
  rt->source_sig = from_file ? spec.sig() : std::string();
  adopt_mesh(*rt, snapshot); // the node already shows it
  ari::write<int>(a, k.busy, 0);
  ari::write<std::string>(a, k.status, "idle");
  ari::write<std::string>(a, k.color_min, "-");
  ari::write<std::string>(a, k.color_max, "-");
  rt->colormap = ari::read_or<std::string>(a, k.colormap, "viridis");
  init_field_colors(a, rt->fc, g->stateName("field_colors"), k.colormap);
  g->setColorTable(rt->fc.table);
  // Keys written before the scene was realized (an init: program) name another source: read it.
  if (from_file && mesh_source_from_keys(*rt).sig() != rt->source_sig)
    ari::write<std::string>(a, k.request, "load");

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick_mesh_lab(*rt); });
  return g;
}

// --- fe_lab: element cut-away (the elements view)
// -------------------------------------------------

// One element's colour value: size = log2 of its edge scale (the cube root of its volume), so each
// LBIE octree level is one band; quality = a tet's minimum dihedral angle (degrees) or a hex's
// scaled Jacobian (1 = a cube).
double element_value(const cvc::geometry &m, std::size_t e, bool hex, bool quality) {
  const auto &X = m.const_points();
  if (hex) {
    cvc::geometry::point_t v[8];
    for (int k = 0; k < 8; ++k)
      v[k] = X[m.const_hexs()[e][k]];
    return quality ? cvc::hex_scaled_jacobian(v)
                   : std::log2(std::cbrt(std::abs(cvc::hex_volume(v))) + 1e-300);
  }
  const auto &t = m.const_tets()[e];
  const auto &a = X[t[0]], &b = X[t[1]], &c = X[t[2]], &d = X[t[3]];
  if (quality)
    return cvc::tet_min_dihedral_angle(a, b, c, d);
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
               v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]},
               w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  const double vol =
      std::abs(u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) +
               u[2] * (v[0] * w[1] - v[1] * w[0])) /
      6.0;
  return std::log2(std::cbrt(vol) + 1e-300);
}

struct Cutaway {
  cvc::geometry faces, edges;
  std::vector<double> values; // per point of faces
  double lo = 0.0, hi = 0.0;  // their finite range
};

// The elements whose centroid c is on the kept side of the plane x[axis] = at (side * (c - at)
// >= 0; axis < 0 keeps them all), drawn as their exposed faces. Each face gets its own copies of
// its corners so it carries its element's value as a flat colour, wound outward so it lights from
// outside; `edges` outlines every exposed face (no quad diagonals). With `values` false every
// value is 0: the element mesh's outer boundary as a plain surface.
Cutaway cut_away(const cvc::geometry &m, int axis, double at, double side, bool quality,
                 bool values) {
  static const int kTetFaces[4][4] = {{1, 2, 3, 0}, {0, 3, 2, 0}, {0, 1, 3, 0}, {0, 2, 1, 0}};
  static const int kHexFaces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                      {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
  const auto &X = m.const_points();
  const bool hex = !m.const_hexs().empty();
  const std::size_t n = hex ? m.const_hexs().size() : m.const_tets().size();
  const int nc = hex ? 8 : 4, nf = hex ? 6 : 4, fv = hex ? 4 : 3;
  const auto corner = [&](std::size_t e, int k) {
    return hex ? m.const_hexs()[e][k] : m.const_tets()[e][k];
  };
  const auto face_corner = [&](std::size_t e, int f, int i) {
    return corner(e, hex ? kHexFaces[f][i] : kTetFaces[f][i]);
  };

  std::vector<cvc::point_t> centre(n);
  std::vector<char> keep(n);
  for (std::size_t e = 0; e < n; ++e) {
    cvc::point_t c{{0, 0, 0}};
    for (int k = 0; k < nc; ++k)
      for (int j = 0; j < 3; ++j)
        c[j] += X[corner(e, k)][j] / nc;
    centre[e] = c;
    keep[e] = axis < 0 || side * (c[axis] - at) >= 0;
  }

  // A face is exposed when exactly one kept element has it.
  using key_t = std::array<cvc::index_t, 4>;
  struct key_hash {
    std::size_t operator()(const key_t &k) const {
      std::size_t h = 0;
      for (cvc::index_t v : k)
        h = h * 1000003u ^ std::hash<cvc::index_t>{}(v);
      return h;
    }
  };
  const auto key = [&](std::size_t e, int f) {
    key_t k;
    k.fill(std::numeric_limits<cvc::index_t>::max());
    for (int i = 0; i < fv; ++i)
      k[i] = face_corner(e, f, i);
    std::sort(k.begin(), k.begin() + fv);
    return k;
  };
  std::unordered_map<key_t, int, key_hash> count;
  count.reserve(n * nf);
  for (std::size_t e = 0; e < n; ++e)
    if (keep[e])
      for (int f = 0; f < nf; ++f)
        ++count[key(e, f)];

  Cutaway cw;
  auto &P = cw.faces.points();
  for (std::size_t e = 0; e < n; ++e) {
    if (!keep[e])
      continue;
    const double value = values ? element_value(m, e, hex, quality) : 0.0;
    for (int f = 0; f < nf; ++f) {
      if (count[key(e, f)] != 1)
        continue;
      cvc::point_t p[4];
      for (int i = 0; i < fv; ++i)
        p[i] = X[face_corner(e, f, i)];
      // Wind it outward (away from its element's centre). The normal of the winding p0 p1 p2
      // (p3): (p1 - p0) x (p2 - p0), or for a quad its diagonals' (p2 - p0) x (p3 - p1).
      double u[3], w[3];
      for (int j = 0; j < 3; ++j) {
        u[j] = fv == 4 ? p[2][j] - p[0][j] : p[1][j] - p[0][j];
        w[j] = fv == 4 ? p[3][j] - p[1][j] : p[2][j] - p[0][j];
      }
      const double nrm[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                             u[0] * w[1] - u[1] * w[0]};
      double outward = 0;
      for (int j = 0; j < 3; ++j)
        outward += nrm[j] * (p[0][j] - centre[e][j]);
      if (outward < 0)
        std::reverse(p, p + fv);
      const auto base = static_cast<cvc::index_t>(P.size());
      for (int i = 0; i < fv; ++i) {
        P.push_back(p[i]);
        cw.values.push_back(value);
        cw.edges.lines().push_back({{base + static_cast<cvc::index_t>(i),
                                     base + static_cast<cvc::index_t>((i + 1) % fv)}});
      }
      cw.faces.tris().push_back({{base, base + 1, base + 2}});
      if (fv == 4)
        cw.faces.tris().push_back({{base, base + 2, base + 3}});
    }
  }
  cw.edges.points() = P;
  const std::pair<double, double> r = finite_range(cw.values);
  cw.lo = r.first;
  cw.hi = r.second;
  return cw;
}

// --- fe_lab: the field on the input surface ------------------------------------------------------

// A field on the element mesh's boundary sampled at the vertices of the smooth surface it
// approximates (an SDF source's own mesh): each vertex's closest boundary triangle and its
// barycentric weights, built once per mesh so a heat solve's every snapshot maps cheaply.
struct Transfer {
  std::vector<std::array<cvc::index_t, 3>> corner;
  std::vector<std::array<double, 3>> weight;
};

std::shared_ptr<const Transfer> make_transfer(const cvc::geometry &boundary,
                                              const cvc::geometry &surface) {
  const cvc::closest_point_result cp = cvc::closest_points(boundary, surface.const_points());
  const auto &P = boundary.const_points();
  auto t = std::make_shared<Transfer>();
  const std::size_t n = surface.const_points().size();
  t->corner.resize(n);
  t->weight.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    std::array<cvc::index_t, 3> &c = t->corner[i];
    std::array<double, 3> &w = t->weight[i];
    c = {{0, 0, 0}};
    w = {{0.0, 0.0, 0.0}};
    if (cp.face[i] < 0)
      continue;
    const auto &tri = boundary.const_tris()[static_cast<std::size_t>(cp.face[i])];
    c = {{tri[0], tri[1], tri[2]}};
    double e0[3], e1[3], e2[3];
    for (int k = 0; k < 3; ++k) {
      e0[k] = P[tri[1]][k] - P[tri[0]][k];
      e1[k] = P[tri[2]][k] - P[tri[0]][k];
      e2[k] = cp.points[i][k] - P[tri[0]][k];
    }
    const auto dot = [](const double *a, const double *b) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    const double d00 = dot(e0, e0), d01 = dot(e0, e1), d11 = dot(e1, e1), d20 = dot(e2, e0),
                 d21 = dot(e2, e1), den = d00 * d11 - d01 * d01;
    if (!(den > 0)) { // a degenerate triangle: take its first corner
      w = {{1.0, 0.0, 0.0}};
      continue;
    }
    const double b1 = (d11 * d20 - d01 * d21) / den, b2 = (d00 * d21 - d01 * d20) / den;
    w = {{1.0 - b1 - b2, b1, b2}};
  }
  return t;
}

std::vector<double> apply_transfer(const Transfer &t, const std::vector<double> &f) {
  std::vector<double> out(t.corner.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    const auto &c = t.corner[i];
    const auto &w = t.weight[i];
    out[i] = w[0] * f[c[0]] + w[1] * f[c[1]] + w[2] * f[c[2]];
  }
  return out;
}

// --- fe_lab --------------------------------------------------------------------------------------

struct FeParams {
  double isovalue = 0.0, isovalue2 = 0.0;
  bool inside_below = true; // mesh {v < isovalue} (an SDF, negative inside) vs {v > isovalue}
  bool interval = false;    // mesh the shell between isovalue and isovalue2 instead
  bool hex = false;         // hexahedra (the elements view only) instead of tetrahedra
  int improve = 0;
  int improve_iterations = 1;
  Placement place;                            // a volume source's fit:, applied to the elements
  std::string mesh_sig;                       // what the mesh is made from (FeLabRT::mesh_sig)
  std::shared_ptr<const cvc::geometry> input; // an SDF source's own (fitted) mesh, or null
  std::string problem = "poisson";
  double f = 1.0;
  cvc::fem::heat_params heat;
  int mode = 1;                                        // modes: which eigenmode (>= 1)
  std::shared_ptr<const cvc::fem::eigen_result> eigen; // cached for this mesh, or null
  SourceSpec source;                                   // load
};

// A loaded source: the volume, where its elements go, and the box shown until the first mesh.
struct FeLoaded {
  std::shared_ptr<const cvc::volume> volume;
  std::shared_ptr<const cvc::geometry> input; // an SDF source's own (fitted) mesh, or null
  Placement place;
  cvc::geometry placeholder;
  std::string sig, status;
};

struct FeJob {
  std::string op;         // load | mesh | solve
  bool stoppable = false; // a heat solve: the only job that polls `cancel` mid-run
  std::string running;    // the status shown while it runs ("meshing...", "heat...")
  std::atomic<bool> done{false};
  std::atomic<bool> cancel{false};
  std::atomic<int> progress{0}; // percent
  std::shared_ptr<const FeLoaded> loaded;
  std::shared_ptr<const cvc::geometry> tetmesh, surface;
  bool hex = false;
  double quality_min = 0.0, quality_mean = 0.0;
  std::string mesh_sig;
  std::shared_ptr<const Transfer> transfer;    // boundary -> `smooth`
  std::shared_ptr<const cvc::geometry> smooth; // the surface the elements approximate, or null
  std::vector<double> field;
  std::shared_ptr<const cvc::fem::eigen_result> eigen;
  std::string status;
  // Heat solves publish intermediate fields here (lane writes, tick takes).
  std::mutex snap_mtx;
  std::vector<double> snapshot;
  bool fresh = false;
};

struct SliceJob {
  std::atomic<bool> done{false};
  bool colored = false;
  std::shared_ptr<const cvc::geometry> slice;
};

struct CutJob {
  std::atomic<bool> done{false};
  std::uint64_t mesh_version = 0;
  std::shared_ptr<const Cutaway> cut;
};

struct FeKeys {
  std::string request, busy, stoppable, status, progress, tets, hexes, nv, quality_min,
      quality_mean, field_min, field_max, work_ms, last_mesh;
  std::string source_type, source_file, source_dim, source_algorithm, source_up, source_flip,
      source_height, auto_mesh, auto_solve;
  std::string region, isovalue, isovalue2, inside, improve, improve_iterations, elements;
  std::string problem, poisson_f, heat_kappa, heat_dt, heat_steps, mode_index;
  std::string colormap, show_surface, show_slice, slice_axis, slice_offset, surface_opacity;
  std::string view_mode, color_by, cut_side, ghost, fixed_range, range_min, range_max, boundary;
};

struct FeLabRT {
  cvc::app *app = nullptr;
  std::weak_ptr<GeometryNode> node, slice_node, edges_node, ghost_node;
  FeKeys k;
  std::string base_dir;
  bool single_color = true;
  cvc::volume volume;
  // `volume`'s shared owner, a new one per load: the transfer-function binding follows it.
  std::shared_ptr<const cvc::volume> shared_volume;
  bool have_volume = false;  // false until a source has loaded
  bool load_pending = false; // the realize-time SDF, computed by the first tick's load
  Placement place;
  cvc::geometry placeholder; // the volume's bounds as lines, shown until the first mesh
  std::shared_ptr<const cvc::geometry> input; // an SDF source's own (fitted) mesh, or null
  std::string source_sig;                     // the source keys `volume` was loaded from
  std::uint64_t source_version = 0;
  std::shared_ptr<const cvc::geometry> tetmesh, surface;
  bool hex = false;
  std::shared_ptr<const Transfer> transfer;    // the boundary's field onto `smooth`, or null
  std::shared_ptr<const cvc::geometry> smooth; // the surface the elements approximate, or null
  std::vector<double> input_field;             // the field there, for field_version input_version
  std::uint64_t input_version = 0;
  std::string mesh_sig, mesh_summary;
  double lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0}; // mesh bounds
  std::vector<double> field;
  double field_lo = 0, field_hi = 0;
  std::shared_ptr<const cvc::fem::eigen_result> eigen; // for the current mesh
  std::uint64_t mesh_version = 0, field_version = 0;
  std::string main_sig, color_sig, slice_sig, slice_cm, slice_rsig, cut_sig, ghost_sig;
  FieldColors
      fc; // <node>.field_colors: the colours the field, the cut and the elements map through
  std::shared_ptr<const Cutaway> cut;
  std::uint64_t cut_mesh_version = 0, cut_serial = 0;
  double opacity = -1.0, ghost_opacity = -1.0;
  std::shared_ptr<FeJob> job;
  std::shared_ptr<SliceJob> slice_job;
  std::shared_ptr<CutJob> cut_job;
  WorkTimer work;
  std::unique_ptr<cvc::async_lane> lane;       // loads, meshing, solves
  std::unique_ptr<cvc::async_lane> slice_lane; // cuts, so they follow a long solve

  explicit FeLabRT(cvc::app &a) : app(&a), volume(a) {}
  ~FeLabRT() {
    if (job)
      job->cancel = true;
  }
};

cvc::sdf_algorithm sdf_algorithm_named(const std::string &name) {
  if (name == "v1")
    return cvc::SDF_V1;
  if (name == "igl" && cvc::mesh_ops_available())
    return cvc::SDF_IGL;
  return cvc::SDF_V2;
}

// Read a source into a volume: the SDF of a mesh (fitted first), or a volume file (resampled when
// dim > 0, fitted as its elements are made). Throws on failure.
std::shared_ptr<FeLoaded> load_fe_source(cvc::app &app, const SourceSpec &s) {
  auto L = std::make_shared<FeLoaded>();
  if (s.type == "sdf") {
    // The SDF is taken in the mesh's own units (turned z-up), and fit: moves the elements after
    // meshing, as it does a volume's: LBIE's octree refines by an error in the volume's value
    // units, so scaling the SDF to the fit height would refine a small model everywhere. So the
    // isovalues are in the mesh's own units too.
    SourceSpec turned = s;
    turned.height = 0.0;
    const cvc::geometry raw = load_source_mesh(turned);
    const int d = s.dim > 0 ? s.dim : 64;
    L->volume = std::make_shared<cvc::volume>(
        sdf_volume_of_mesh(app, raw, d, sdf_algorithm_named(s.algorithm)));
    L->place = fit_points(raw.const_points(), s.height);
    auto shown = std::make_shared<cvc::geometry>();
    shown->copy(raw, true);
    L->place.apply(*shown);
    L->input = shown;
    L->status = "loaded: the " + std::to_string(d) + "^3 SDF of " + s.file;
  } else if (s.type == "volume") {
    if (s.file.empty())
      throw std::runtime_error("source.file is empty");
    const ari::ResolvedFile f = resolve_source_path(s.file, s.base_dir);
    if (!f.ok)
      throw std::runtime_error(f.error);
    auto v = std::make_shared<cvc::volume>(app, f.path);
    if (s.dim > 0)
      v->resize(cvc::dimension(s.dim, s.dim, s.dim));
    L->place = fit_box(v->boundingBox(), s.up, s.flip, s.height);
    L->status = "loaded: " + s.file + " (" + std::to_string(v->XDim()) + "x" +
                std::to_string(v->YDim()) + "x" + std::to_string(v->ZDim()) + ")";
    L->volume = v;
  } else {
    throw std::runtime_error("unknown source.type '" + s.type + "' (sdf | volume)");
  }
  L->placeholder = box_lines(L->volume->boundingBox());
  L->place.apply(L->placeholder);
  L->sig = s.sig();
  return L;
}

// An SDF source's mesh, placed, and the box its field will span -- everything but the field, so
// the scene can frame it before the SDF is computed.
std::shared_ptr<FeLoaded> frame_sdf_source(const SourceSpec &s) {
#ifndef CVC_ENABLE_SDF
  throw std::runtime_error("source: { sdf } needs a CVC_ENABLE_SDF build");
#endif
  auto L = std::make_shared<FeLoaded>();
  SourceSpec turned = s;
  turned.height = 0.0;
  const cvc::geometry raw = load_source_mesh(turned);
  L->place = fit_points(raw.const_points(), s.height);
  auto shown = std::make_shared<cvc::geometry>();
  shown->copy(raw, true);
  L->place.apply(*shown);
  L->input = shown;
  const cvc::bounding_box e = raw.extents(); // sdf_volume_of_mesh's grid: padded 10%, cubic
  const double half = 0.55 * std::max({e.maxx - e.minx, e.maxy - e.miny, e.maxz - e.minz});
  const double cx = 0.5 * (e.minx + e.maxx), cy = 0.5 * (e.miny + e.maxy),
               cz = 0.5 * (e.minz + e.maxz);
  L->placeholder = box_lines(
      cvc::bounding_box(cx - half, cy - half, cz - half, cx + half, cy + half, cz + half));
  L->place.apply(L->placeholder);
  L->sig = s.sig();
  return L;
}

#ifdef CVC_ENABLE_MESHER
// -v as a Float volume on the same grid: LBIE meshes the side ABOVE its isovalue.
cvc::volume negated(const cvc::volume &v) {
  const std::uint64_t n = v.XDim() * v.YDim() * v.ZDim();
  std::vector<float> buf(n);
  for (std::uint64_t i = 0; i < n; ++i)
    buf[i] = static_cast<float>(-v(i));
  return cvc::volume(v.ctx(), reinterpret_cast<const unsigned char *>(buf.data()),
                     v.voxel_dimensions(), cvc::Float, v.boundingBox());
}
#endif

void run_fe_mesh(FeJob &job, const cvc::volume &vol, const FeParams &p) {
#ifdef CVC_ENABLE_MESHER
  if (p.hex && p.interval)
    throw std::runtime_error("interval meshing makes tetrahedra (mesh.elements: tet)");
  // LBIE meshes {v > isovalue}. For the inside of an SDF (negative inside) mesh -v above
  // -isovalue, so mesh.isovalue stays in the volume's own units either way. An interval is meshed
  // in the volume's own units whatever mesh.inside says.
  const cvc::volume v = p.inside_below && !p.interval ? negated(vol) : vol;
  const double a = p.inside_below ? -p.isovalue : p.isovalue;
  const auto improve = static_cast<cvc::improvement_method>(p.improve);
  auto tm = std::make_shared<cvc::geometry>();
  std::string where;
  if (p.interval) {
    // The shell between the two isovalues (it throws, saying why, when there is none).
    *tm = tetrahedralize_interval(vol, p.isovalue, p.isovalue2, p.improve, p.improve_iterations);
    where = " in [" + fmt_value(std::min(p.isovalue, p.isovalue2)) + ", " +
            fmt_value(std::max(p.isovalue, p.isovalue2)) + "]";
  } else if (p.hex) {
    *tm = cvc::hexahedralize(v, a, cvc::DUALLIB, improve, cvc::BSPLINE_CONVOLUTION,
                             p.improve_iterations);
  } else {
    *tm = cvc::tetrahedralize(v, a, cvc::DUALLIB, improve, cvc::BSPLINE_CONVOLUTION,
                              p.improve_iterations);
  }
  const std::size_t n = p.hex ? tm->const_hexs().size() : tm->const_tets().size();
  if (n == 0)
    throw std::runtime_error(p.hex ? "no hexes: the isovalue does not cut the volume"
                                   : "no tets: the isovalue does not cut the volume");
  p.place.apply(*tm);
  std::shared_ptr<cvc::geometry> surf;
  cvc::quality_stats q{};
  if (p.hex) {
    surf = std::make_shared<cvc::geometry>(cut_away(*tm, -1, 0.0, 1.0, false, false).faces);
    q = cvc::compute_hex_quality_stats(tm->const_hexs(), tm->const_points(),
                                       cvc::HEX_SCALED_JACOBIAN);
  } else {
    cvc::orient_tets(*tm);
    surf = std::make_shared<cvc::geometry>(cvc::tet_boundary_surface(*tm));
    q = cvc::compute_tet_quality_stats(tm->const_tets(), tm->const_points(), cvc::TET_MIN_ANGLE);
    // The field shows more smoothly on the surface the elements approximate than on their own
    // boundary: an SDF source's mesh, or a volume's isosurface. Not for an interval mesh, whose
    // inner boundary is on neither.
    if (!p.interval) {
      std::shared_ptr<const cvc::geometry> smooth = p.input;
      if (!smooth) {
        auto g = std::make_shared<cvc::geometry>(cvc::iso(v, a, cvc::DUALLIB));
        p.place.apply(*g);
        g->normals().clear();
        cvc::compute_vertex_normals(*g);
        if (tri_count(*g) > 0)
          smooth = g;
      }
      if (smooth) {
        job.transfer = make_transfer(*surf, *smooth);
        job.smooth = smooth;
      }
    }
  }
  // The normals come from the faces' windings: LBIE's gradient normals point the wrong way on -v,
  // and an interval's boundary has two sheets, each wound outward from the shell (the inner
  // sheet's normals face the cavity), where g = min(v - lo, hi - v) has a crease.
  cvc::compute_vertex_normals(*surf);
  job.status = "meshed: " + std::to_string(n) + (p.hex ? " hexes" : " tets") + where;
  job.tetmesh = tm;
  job.surface = surf;
  job.hex = p.hex;
  job.quality_min = q.min;
  job.quality_mean = q.mean;
  job.mesh_sig = p.mesh_sig;
#else
  (void)vol;
  (void)p;
  job.status = "mesher unavailable";
#endif
}

void run_fe_solve(FeJob &job, const cvc::geometry &tm, const FeParams &p) {
  const cvc::fem::domain vol = cvc::fem::domain::VOLUME;
  const cvc::geometry::points_t &P = tm.const_points();
  if (p.problem == "modes") {
    // Free-vibration (Neumann) modes of the solid; mode 0 is the constant.
    std::shared_ptr<const cvc::fem::eigen_result> e = p.eigen;
    if (!e || static_cast<int>(e->vectors.size()) <= p.mode)
      e = std::make_shared<cvc::fem::eigen_result>(
          cvc::fem::laplacian_eigenmodes(tm, std::max(p.mode + 1, kMinVolumeModes), vol));
    job.eigen = e;
    if (static_cast<int>(e->vectors.size()) <= p.mode)
      throw std::runtime_error("the eigensolver returned only " +
                               std::to_string(e->vectors.size()) + " modes");
    job.field = e->vectors[static_cast<std::size_t>(p.mode)];
    job.status = "modes: mode " + std::to_string(p.mode) + ", eigenvalue " +
                 fmt_g(e->values[static_cast<std::size_t>(p.mode)]);
    return;
  }
  const std::vector<cvc::index_t> bnd = cvc::fem::boundary_vertices(tm, vol);
  if (p.problem == "heat") {
    // A hot floor: boundary vertices in the lowest 5% of the height held at 1, the rest of the
    // boundary insulated, the body starting at 0.
    double zlo = std::numeric_limits<double>::infinity(), zhi = -zlo;
    for (cvc::index_t v : bnd) {
      zlo = std::min(zlo, P[v][2]);
      zhi = std::max(zhi, P[v][2]);
    }
    cvc::fem::dirichlet bc;
    for (cvc::index_t v : bnd)
      if (P[v][2] <= zlo + 0.05 * (zhi - zlo))
        bc.vertices.push_back(v);
    bc.values.push_back(1.0);
    if (bc.vertices.empty())
      throw std::runtime_error("no boundary vertices to heat");
    const int steps = std::max(1, p.heat.steps);
    const int every = std::max(1, steps / 20);
    int last = 0;
    const cvc::fem::heat_progress progress = [&job, &last, steps,
                                              every](int s, const std::vector<double> &u) {
      last = s;
      job.progress = static_cast<int>(100.0 * s / steps);
      if (s % every == 0 && s < steps) {
        std::lock_guard<std::mutex> lock(job.snap_mtx);
        job.snapshot = u;
        job.fresh = true;
      }
      return !job.cancel.load();
    };
    job.field =
        cvc::fem::solve_heat(tm, std::vector<double>(P.size(), 0.0), p.heat, bc, vol, progress);
    job.status = job.cancel ? "heat: stopped at step " + std::to_string(last)
                            : "heat: " + std::to_string(steps) + " steps";
  } else if (p.problem == "poisson") {
    cvc::fem::dirichlet bc;
    bc.vertices = bnd;
    bc.values.push_back(0.0);
    job.field = cvc::fem::solve_poisson(tm, {p.f}, bc, vol);
    job.status = "poisson: solved";
  } else {
    throw std::runtime_error("unknown problem '" + p.problem + "' (poisson | heat | modes)");
  }
}

void run_fe_job(FeJob &job, cvc::app *app, const cvc::volume &vol,
                std::shared_ptr<const cvc::geometry> tm, const FeParams &p) {
  const auto t0 = std::chrono::steady_clock::now();
  try {
    if (job.cancel) {
      job.status = "cancelled";
    } else if (job.op == "load") {
      std::shared_ptr<FeLoaded> L = load_fe_source(*app, p.source);
      job.status = L->status;
      job.loaded = L;
    } else if (job.op == "mesh") {
      run_fe_mesh(job, vol, p);
    } else {
      run_fe_solve(job, *tm, p);
    }
    if (!job.cancel)
      job.status += " (" + fmt_ms(ms_since(t0)) + " ms)";
  } catch (const std::exception &e) {
    job.loaded.reset();
    job.tetmesh.reset();
    job.field.clear();
    job.status = std::string("error: ") + e.what();
  } catch (...) {
    job.loaded.reset();
    job.tetmesh.reset();
    job.field.clear();
    job.status = "error: unknown exception";
  }
  job.progress = 100;
  job.done.store(true, std::memory_order_release);
}

SourceSpec fe_source_from_keys(const FeLabRT &rt) {
  cvc::app &a = *rt.app;
  SourceSpec s;
  s.type = ari::read_or<std::string>(a, rt.k.source_type, "sdf");
  s.file = ari::read_or<std::string>(a, rt.k.source_file, "");
  s.dim = to_int_clamped(ari::read_or<double>(a, rt.k.source_dim, 0.0), 0, 1024, 0);
  s.algorithm = ari::read_or<std::string>(a, rt.k.source_algorithm, "v2");
  s.up = up_axis(ari::read_or<std::string>(a, rt.k.source_up, "z"));
  s.flip = ari::read_bool_or(a, rt.k.source_flip, 0) != 0;
  const double h = ari::read_or<double>(a, rt.k.source_height, 0.0);
  s.height = std::isfinite(h) && h > 0.0 ? h : 0.0;
  s.base_dir = rt.base_dir;
  return s;
}

bool hex_elements(cvc::app &a, const FeKeys &k) {
  return ari::read_or<std::string>(a, k.elements, "tet") == "hex";
}

// What a mesh is made from: the loaded source and every mesh.* key. A `load` whose mesh would
// come out the same keeps it.
std::string fe_mesh_sig(const FeLabRT &rt) {
  cvc::app &a = *rt.app;
  const auto num = [&](const std::string &key, double def) {
    return std::to_string(ari::read_or<double>(a, key, def));
  };
  return std::to_string(rt.source_version) + "|" + num(rt.k.isovalue, 0.0) + "|" +
         ari::read_or<std::string>(a, rt.k.inside, "below") + "|" +
         ari::read_or<std::string>(a, rt.k.region, "solid") + "|" + num(rt.k.isovalue2, 0.0) + "|" +
         (hex_elements(a, rt.k) ? "hex" : "tet") + "|" + num(rt.k.improve, 0.0) + "|" +
         num(rt.k.improve_iterations, 1.0);
}

void launch_fe_job(FeLabRT &rt, const std::string &op) {
  cvc::app &a = *rt.app;
  FeParams p;
  const FeBounds b = read_fe_bounds(a, rt.k.region, rt.k.isovalue, rt.k.isovalue2);
  p.isovalue = b.isovalue;
  p.inside_below = ari::read_or<std::string>(a, rt.k.inside, "below") != "above";
  p.interval = b.interval;
  p.isovalue2 = b.isovalue2;
  p.hex = hex_elements(a, rt.k);
  p.improve = to_int_clamped(ari::read_or<double>(a, rt.k.improve, 0), 0, 5, 0);
  p.improve_iterations =
      to_int_clamped(ari::read_or<double>(a, rt.k.improve_iterations, 1), 0, 100, 1);
  p.place = rt.place;
  p.mesh_sig = fe_mesh_sig(rt);
  p.input = rt.input;
  p.problem = ari::read_or<std::string>(a, rt.k.problem, "poisson");
  p.f = ari::read_or<double>(a, rt.k.poisson_f, 1.0);
  // heat.kappa is relative to the domain: the diffusivity used is kappa * extent^2, so the same
  // settings spread heat across the whole body whatever the model's scale.
  const double ext = std::max({rt.hi[0] - rt.lo[0], rt.hi[1] - rt.lo[1], rt.hi[2] - rt.lo[2]});
  p.heat.kappa = ari::read_or<double>(a, rt.k.heat_kappa, 1.0) * ext * ext;
  p.heat.dt = ari::read_or<double>(a, rt.k.heat_dt, 0.01);
  p.heat.steps = to_int_clamped(ari::read_or<double>(a, rt.k.heat_steps, 100), 1, 1000000, 100);
  p.mode = mode_index(a, rt.k.mode_index);
  p.eigen = rt.eigen;
  if (op == "load")
    p.source = fe_source_from_keys(rt);

  auto job = std::make_shared<FeJob>();
  job->op = op;
  // Only a heat solve polls the cancel flag (its progress callback); loading, tet meshing and the
  // other solves run to the end, so `stop` is refused for them rather than claimed.
  job->stoppable = op == "solve" && p.problem == "heat";
  job->running = op == "load"   ? "loading " + p.source.file + "..."
                 : op == "mesh" ? "meshing..."
                                : p.problem + "...";
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  ari::write<int>(a, rt.k.stoppable, job->stoppable ? 1 : 0);
  ari::write<int>(a, rt.k.progress, 0);
  ari::write<std::string>(a, rt.k.status, job->running);
  if (op == "mesh") // whoever asked: the transfer-function binding's debounced remesh compares it
    ari::write<std::string>(a, rt.k.last_mesh, bounds_signature(b));
  const cvc::volume vol = rt.volume;
  std::shared_ptr<const cvc::geometry> tm = rt.tetmesh;
  cvc::app *app = rt.app;
  if (!rt.lane->submit([job, app, vol, tm, p]() { run_fe_job(*job, app, vol, tm, p); })) {
    job->status = "error: the fe_lab lane is stopped";
    job->done.store(true, std::memory_order_release);
  }
}

void set_field(FeLabRT &rt, std::vector<double> field) {
  rt.field = std::move(field);
  const std::pair<double, double> r = finite_range(rt.field);
  rt.field_lo = r.first;
  rt.field_hi = r.second;
  ++rt.field_version;
  ari::write<double>(*rt.app, rt.k.field_min, rt.field_lo);
  ari::write<double>(*rt.app, rt.k.field_max, rt.field_hi);
}

void clear_field(FeLabRT &rt) {
  rt.field.clear();
  ++rt.field_version;
  ari::write<std::string>(*rt.app, rt.k.field_min, "-");
  ari::write<std::string>(*rt.app, rt.k.field_max, "-");
}

void publish_fe_stats(FeLabRT &rt, double quality_min, double quality_mean) {
  cvc::app &a = *rt.app;
  const std::size_t n = !rt.tetmesh ? 0
                        : rt.hex    ? rt.tetmesh->const_hexs().size()
                                    : rt.tetmesh->const_tets().size();
  ari::write<double>(a, rt.k.tets, rt.hex ? 0.0 : static_cast<double>(n));
  ari::write<double>(a, rt.k.hexes, rt.hex ? static_cast<double>(n) : 0.0);
  ari::write<double>(a, rt.k.nv,
                     rt.tetmesh ? static_cast<double>(rt.tetmesh->const_points().size()) : 0.0);
  if (rt.tetmesh) {
    ari::write<double>(a, rt.k.quality_min, quality_min);
    ari::write<double>(a, rt.k.quality_mean, quality_mean);
  } else {
    ari::write<std::string>(a, rt.k.quality_min, "-");
    ari::write<std::string>(a, rt.k.quality_mean, "-");
  }
}

// Drop the mesh and everything made from it (a new source, or a failed load of one).
void clear_fe_mesh(FeLabRT &rt) {
  rt.tetmesh.reset();
  rt.surface.reset();
  rt.transfer.reset();
  rt.smooth.reset();
  rt.hex = false;
  rt.mesh_sig.clear();
  rt.mesh_summary.clear();
  rt.eigen.reset();
  rt.cut.reset();
  ++rt.mesh_version;
  clear_field(rt);
  publish_fe_stats(rt, 0.0, 0.0);
}

// The source volume's value range: [0, 1] before one has loaded, at least 1 wide.
void fe_value_range(FeLabRT &rt, double &lo, double &hi) {
  lo = 0.0;
  hi = 1.0;
  if (rt.have_volume) {
    try {
      lo = rt.volume.min();
      hi = rt.volume.max();
    } catch (...) {
    }
  }
  if (!(hi > lo))
    hi = lo + 1.0;
}

// The default second bound: a tenth of the value range from the isovalue on the side a solid mesh
// fills (below it for inside below -- an SDF's inner skin -- above it for a density), but no more
// than half way to that end of the range; the other side when the isovalue sits at (or past) that
// end.
double default_isovalue2(FeLabRT &rt) {
  cvc::app &a = *rt.app;
  double vlo = 0.0, vhi = 1.0;
  fe_value_range(rt, vlo, vhi);
  const double iso = ari::read_or<double>(a, rt.k.isovalue, 0.0);
  const bool below = ari::read_or<std::string>(a, rt.k.inside, "below") != "above";
  const double d = 0.1 * (vhi - vlo);
  const auto step = [&](bool down) {
    const double room = down ? iso - vlo : vhi - iso;
    return room > 0.0 ? std::min(d, 0.5 * room) : 0.0;
  };
  bool down = below;
  double s = step(down);
  if (!(s > 0.0)) {
    down = !down;
    s = step(down);
  }
  if (!(s > 0.0))
    s = d;
  return down ? iso - s : iso + s;
}

void apply_fe_loaded(FeLabRT &rt, const FeLoaded &L) {
  rt.volume = *L.volume;
  rt.shared_volume = L.volume;
  rt.have_volume = true;
  rt.input = L.input;
  rt.place = L.place;
  rt.placeholder = L.placeholder;
  rt.source_sig = L.sig;
  ++rt.source_version;
  clear_fe_mesh(rt);
  // An SDF computed after realize: mesh.isovalue2's default needed its range (realize_fe_lab).
  if (ari::read_or<std::string>(*rt.app, rt.k.isovalue2, "").empty())
    ari::write<double>(*rt.app, rt.k.isovalue2, default_isovalue2(rt));
}

void apply_fe_mesh(FeLabRT &rt, const FeJob &j) {
  rt.tetmesh = j.tetmesh;
  rt.surface = j.surface;
  rt.transfer = j.transfer;
  rt.smooth = j.smooth;
  rt.hex = j.hex;
  rt.mesh_sig = j.mesh_sig;
  for (int d = 0; d < 3; ++d) {
    rt.lo[d] = std::numeric_limits<double>::infinity();
    rt.hi[d] = -rt.lo[d];
  }
  for (const cvc::geometry::point_t &p : rt.surface->const_points())
    for (int d = 0; d < 3; ++d) {
      rt.lo[d] = std::min(rt.lo[d], p[d]);
      rt.hi[d] = std::max(rt.hi[d], p[d]);
    }
  ++rt.mesh_version;
  rt.eigen.reset();
  rt.cut.reset();
  clear_field(rt);
  const std::size_t n = rt.hex ? rt.tetmesh->const_hexs().size() : rt.tetmesh->const_tets().size();
  rt.mesh_summary = std::to_string(n) + (rt.hex ? " hexes" : " tets");
  publish_fe_stats(rt, j.quality_min, j.quality_mean);
}

// After a load (or a load that found its source unchanged), unless a request already waits:
// re-mesh when auto.mesh is on and the mesh is stale, else solve when auto.solve is on.
void after_fe_load(FeLabRT &rt) {
  cvc::app &a = *rt.app;
  if (!ari::read_or<std::string>(a, rt.k.request, "").empty())
    return;
  const bool auto_mesh = ari::read_bool_or(a, rt.k.auto_mesh, 0) != 0;
  const bool auto_solve = ari::read_bool_or(a, rt.k.auto_solve, 0) != 0;
  if (auto_mesh && (!rt.tetmesh || fe_mesh_sig(rt) != rt.mesh_sig))
    ari::write<std::string>(a, rt.k.request, "mesh");
  else if (auto_solve && rt.tetmesh && !rt.hex)
    ari::write<std::string>(a, rt.k.request, "solve");
}

void apply_fe_job(FeLabRT &rt, FeJob &j) {
  cvc::app &a = *rt.app;
  if (j.op == "load" && j.loaded) {
    apply_fe_loaded(rt, *j.loaded);
  } else if (j.op == "mesh" && j.tetmesh) {
    apply_fe_mesh(rt, j);
  } else if (j.op == "solve") {
    if (j.eigen)
      rt.eigen = j.eigen; // the mesh cannot change under a solve: one job runs at a time
    if (!j.field.empty())
      set_field(rt, std::move(j.field));
  }
  ari::write<std::string>(a, rt.k.status, j.status);
  ari::write<int>(a, rt.k.progress, 100);
  ari::write<int>(a, rt.k.stoppable, 0);
  ari::write<int>(a, rt.k.busy, 0);
  if (j.op == "load" && j.loaded) {
    after_fe_load(rt);
  } else if (j.op == "mesh" && j.tetmesh && !rt.hex &&
             ari::read_bool_or(a, rt.k.auto_solve, 0) != 0 &&
             ari::read_or<std::string>(a, rt.k.request, "").empty()) {
    // auto.solve queues a solve only when nothing is waiting: a request written while the mesh
    // job ran (a re-mesh after an isovalue edit, a solve) runs next instead -- a click is never
    // lost. A `stop` never waits (the tick consumes it at once), and a waiting mesh auto-solves in
    // turn.
    ari::write<std::string>(a, rt.k.request, "solve");
  }
}

void run_slice_job(SliceJob &job, const cvc::geometry &tm, const std::vector<double> &field,
                   const cvc::geometry::vector_t &normal, double offset) {
  try {
    cvc::geometry t(tm);
    t.functions() = field; // interpolated onto the cut (an empty field clears it)
    auto s = std::make_shared<cvc::geometry>(cvc::slice_tets(t, normal, offset));
    cvc::compute_vertex_normals(*s);
    job.slice = s;
  } catch (...) {
    job.slice.reset();
  }
  job.done.store(true, std::memory_order_release);
}

void run_cut_job(CutJob &job, const cvc::geometry &tm, int axis, double at, double side,
                 bool quality) {
  try {
    auto c = std::make_shared<Cutaway>(cut_away(tm, axis, at, side, quality, true));
    cvc::compute_vertex_normals(c->faces); // every face has its own corners: flat shading
    job.cut = c;
  } catch (...) {
    job.cut.reset();
  }
  job.done.store(true, std::memory_order_release);
}

// Keep the node, its cut and their colours in step with the view keys and the latest field.
//   view.mode field:    the field on the boundary surface (translucent while a slice shows) and
//                       on the <id>_slice cut;
//   view.mode elements: the elements on one side of the cut plane (all of them with the slice
//                       off), each flat-coloured by view.color_by and outlined by <id>_edges, with
//                       the outer surface as a ghost (<id>_ghost) at view.ghost opacity.
void follow_fe_view(FeLabRT &rt, GeometryNode &node, GeometryNode *slice, GeometryNode *edges,
                    GeometryNode *ghost) {
  cvc::app &a = *rt.app;
  const bool show_surface = ari::read_bool_or(a, rt.k.show_surface, 1) != 0;
  const bool show_slice = ari::read_bool_or(a, rt.k.show_slice, 1) != 0;
  const bool elements = ari::read_or<std::string>(a, rt.k.view_mode, "field") == "elements";
  const std::string cm_name = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
  const cvc::colormap_kind cm = colormap_or_viridis(cm_name);
  const bool fixed = ari::read_bool_or(a, rt.k.fixed_range, 0) != 0;
  const double rmin = ari::read_or<double>(a, rt.k.range_min, 0.0);
  const double rmax = ari::read_or<double>(a, rt.k.range_max, 1.0);
  // The field's colours (view.colormap resets them to its ramp): a changed ramp recolours every
  // node showing the field in place. The colour range: <node>.field_colors' explicit domain, else
  // view.fixed_range's, else the values' own (auto_range, what the editor's axis spans).
  if (follow_field_colors(a, rt.fc, rt.k.colormap)) {
    node.setColorTable(rt.fc.table);
    if (slice)
      slice->setColorTable(rt.fc.table);
  }
  const std::string rsig = (fixed ? fmt_g(rmin) + "," + fmt_g(rmax) : std::string("auto")) + "|" +
                           field_colors_sig(rt.fc);
  const auto auto_range = [&](double lo, double hi) {
    return fixed ? std::make_pair(rmin, rmax) : std::make_pair(lo, hi);
  };
  const auto range = [&](double lo, double hi) {
    const std::pair<double, double> r = auto_range(lo, hi);
    return field_colors_range(rt.fc, r.first, r.second);
  };

  // A finished cut-away (of this mesh) replaces the shown one.
  if (rt.cut_job && rt.cut_job->done.load(std::memory_order_acquire)) {
    std::shared_ptr<CutJob> cj = std::move(rt.cut_job);
    if (cj->cut && cj->mesh_version == rt.mesh_version) {
      rt.cut = cj->cut;
      rt.cut_mesh_version = cj->mesh_version;
      ++rt.cut_serial;
      if (edges) {
        swap_geometry(*edges, rt.cut->edges, false);
        edges->setRenderMode(GeometryRenderMode::LINES);
      }
    }
  }
  const bool cut_ready = elements && rt.surface && rt.cut && rt.cut_mesh_version == rt.mesh_version;
  // The boundary is drawn as the smooth surface the elements approximate (view.boundary input:
  // an SDF source's mesh, or the volume's isosurface), the field carried onto it; else as the
  // element mesh's own boundary.
  const bool on_input = rt.surface && rt.transfer && rt.smooth &&
                        ari::read_or<std::string>(a, rt.k.boundary, "input") != "mesh";

  // What the node itself shows: the volume's bounds before the first mesh, the cut-away elements,
  // or the boundary surface.
  const std::string msig = !rt.surface ? "p|" + std::to_string(rt.source_version)
                           : cut_ready ? "e|" + std::to_string(rt.cut_serial)
                                       : "s|" + std::to_string(rt.mesh_version) +
                                             (show_surface ? "|1" : "|0") + (on_input ? "i" : "m");
  if (msig != rt.main_sig) {
    rt.main_sig = msig;
    rt.color_sig.clear();
    hide_field(node, rt.single_color);
    if (!rt.surface) {
      swap_geometry(node, rt.placeholder, false);
      node.setRenderMode(GeometryRenderMode::LINES);
    } else if (cut_ready) {
      swap_geometry(node, rt.cut->faces, false);
    } else if (!show_surface) {
      swap_geometry(node, cvc::geometry(), false);
    } else {
      swap_geometry(node, on_input ? *rt.smooth : *rt.surface, false);
    }
  }
  if (cut_ready) {
    const std::string csig = msig + "|" + cm_name + "|" + rsig;
    if (csig != rt.color_sig) {
      rt.color_sig = csig;
      const std::pair<double, double> r = range(rt.cut->lo, rt.cut->hi);
      show_field(node, rt.cut->values, cm, r.first, r.second);
      const std::pair<double, double> ar = auto_range(rt.cut->lo, rt.cut->hi);
      publish_field_colors(a, rt.fc, rt.cut->values, ar.first, ar.second,
                           "e|" + std::to_string(rt.cut_serial));
    }
  } else if (rt.surface && show_surface) {
    const std::string csig =
        msig + "|" + std::to_string(rt.field_version) + "|" + cm_name + "|" + rsig;
    if (csig != rt.color_sig) {
      rt.color_sig = csig;
      const bool have = rt.tetmesh && rt.field.size() == rt.tetmesh->const_points().size() &&
                        rt.field.size() == rt.surface->const_points().size();
      if (have) {
        const std::pair<double, double> r = range(rt.field_lo, rt.field_hi);
        if (on_input) {
          if (rt.input_version != rt.field_version || rt.input_field.empty()) {
            rt.input_field = apply_transfer(*rt.transfer, rt.field);
            rt.input_version = rt.field_version;
          }
          show_field(node, rt.input_field, cm, r.first, r.second);
        } else {
          show_field(node, rt.field, cm, r.first, r.second);
        }
        // The editor's histogram: the whole solution (the cut shows its inside too).
        const std::pair<double, double> ar = auto_range(rt.field_lo, rt.field_hi);
        publish_field_colors(a, rt.fc, rt.field, ar.first, ar.second,
                             "f|" + std::to_string(rt.field_version));
      } else {
        hide_field(node, rt.single_color);
      }
    }
  }
  // A visible cut needs to be seen through the surface; the elements are opaque.
  double opacity = 1.0;
  if (!elements && show_slice && rt.surface && !rt.hex)
    opacity = clamp01(ari::read_or<double>(a, rt.k.surface_opacity, 0.35), 0.35);
  if (opacity != rt.opacity) {
    rt.opacity = opacity;
    node.setOpacity(opacity);
  }

  const bool shown = node.isVisibleInHierarchy();
  show_child(a, edges, shown && cut_ready);
  if (ghost) {
    const double g =
        elements && rt.surface ? clamp01(ari::read_or<double>(a, rt.k.ghost, 0.0), 0.0) : 0.0;
    if (g > 0.0) {
      const std::string gsig = std::to_string(rt.mesh_version);
      if (gsig != rt.ghost_sig) {
        rt.ghost_sig = gsig;
        swap_geometry(*ghost, *rt.surface, false);
      }
      if (g != rt.ghost_opacity) {
        rt.ghost_opacity = g;
        ghost->setOpacity(g);
      }
    }
    show_child(a, ghost, shown && g > 0.0);
  }

  const int axis = to_int_clamped(ari::read_or<double>(a, rt.k.slice_axis, 0), 0, 2, 0);
  const double s = std::max(-1.0, std::min(1.0, ari::read_or<double>(a, rt.k.slice_offset, 0.0)));
  const double at = 0.5 * (rt.lo[axis] + rt.hi[axis]) + s * 0.5 * (rt.hi[axis] - rt.lo[axis]);

  // The elements view's cut-away.
  if (elements && rt.tetmesh && !rt.cut_job) {
    const double side = ari::read_or<double>(a, rt.k.cut_side, 1.0) < 0.0 ? -1.0 : 1.0;
    const bool quality = ari::read_or<std::string>(a, rt.k.color_by, "size") == "quality";
    const std::string sig = std::to_string(rt.mesh_version) + "|" +
                            (show_slice ? std::to_string(axis) : std::string("-")) + "|" +
                            std::to_string(s) + "|" + std::to_string(side) + "|" +
                            (quality ? "q" : "s");
    if (sig != rt.cut_sig) {
      rt.cut_sig = sig;
      auto cj = std::make_shared<CutJob>();
      cj->mesh_version = rt.mesh_version;
      rt.cut_job = cj;
      std::shared_ptr<const cvc::geometry> tm = rt.tetmesh;
      const int cut_axis = show_slice ? axis : -1;
      if (!rt.slice_lane->submit([cj, tm, cut_axis, at, side, quality]() {
            run_cut_job(*cj, *tm, cut_axis, at, side, quality);
          }))
        cj->done.store(true, std::memory_order_release);
    }
  }

  // The field view's cut.
  if (!slice)
    return;
  show_child(a, slice, shown && !elements && show_slice && rt.surface && !rt.hex);
  if (rt.slice_job && rt.slice_job->done.load(std::memory_order_acquire)) {
    std::shared_ptr<SliceJob> sj = std::move(rt.slice_job);
    hide_field(*slice, rt.single_color);
    swap_geometry(*slice, sj->slice ? *sj->slice : cvc::geometry(), false);
    rt.slice_rsig.clear();
    if (sj->slice && sj->colored &&
        sj->slice->const_functions().size() == sj->slice->const_points().size()) {
      const std::pair<double, double> r = range(rt.field_lo, rt.field_hi);
      show_field(*slice, sj->slice->const_functions(), cm, r.first, r.second);
      rt.slice_cm = cm_name;
      rt.slice_rsig = rsig;
    }
  }
  if (slice->hasScalarField()) {
    if (cm_name != rt.slice_cm) {
      rt.slice_cm = cm_name;
      slice->setColorMap(cm);
    }
    if (rsig != rt.slice_rsig) {
      rt.slice_rsig = rsig;
      const std::pair<double, double> r = range(rt.field_lo, rt.field_hi);
      slice->setScalarRange(r.first, r.second);
    }
  }
  if (elements || !rt.tetmesh || rt.hex || !show_slice || rt.slice_job)
    return;
  const std::string sig = std::to_string(rt.mesh_version) + "|" + std::to_string(rt.field_version) +
                          "|" + std::to_string(axis) + "|" + std::to_string(s);
  if (sig == rt.slice_sig)
    return;
  rt.slice_sig = sig;
  cvc::geometry::vector_t normal = {{0.0, 0.0, 0.0}};
  normal[axis] = 1.0;
  auto sj = std::make_shared<SliceJob>();
  sj->colored = rt.field.size() == rt.tetmesh->const_points().size();
  rt.slice_job = sj;
  std::shared_ptr<const cvc::geometry> tm = rt.tetmesh;
  const std::vector<double> field = sj->colored ? rt.field : std::vector<double>();
  if (!rt.slice_lane->submit(
          [sj, tm, field, normal, at]() { run_slice_job(*sj, *tm, field, normal, at); }))
    sj->done.store(true, std::memory_order_release);
}

// One command from the request key, with no job running. Starts at most one job.
void run_fe_request(FeLabRT &rt, const std::string &req) {
  cvc::app &a = *rt.app;
  if (req == "load" || req == "reload") {
    // load reads the source.* keys again only when they changed, so stepping between problems
    // on one dataset keeps its volume (and, with auto.mesh, its mesh).
    const SourceSpec s = fe_source_from_keys(rt);
    if (req == "reload" || s.sig() != rt.source_sig) {
      launch_fe_job(rt, "load");
    } else {
      ari::write<std::string>(
          a, rt.k.status, rt.tetmesh ? "ready: " + rt.mesh_summary : std::string("volume ready"));
      after_fe_load(rt);
    }
  } else if (req == "mesh") {
#ifdef CVC_ENABLE_MESHER
    if (rt.have_volume)
      launch_fe_job(rt, "mesh");
    else
      ari::write<std::string>(a, rt.k.status, "no volume to mesh: load a source first");
#else
    ari::write<std::string>(a, rt.k.status, "mesher unavailable");
#endif
  } else if (req == "solve") {
    if (!rt.tetmesh)
      ari::write<std::string>(a, rt.k.status, "tet-mesh the volume first");
    else if (rt.hex)
      ari::write<std::string>(a, rt.k.status,
                              "a solve needs tetrahedra: this mesh is hexahedra (set "
                              "mesh.elements to tet and mesh again)");
    else
      launch_fe_job(rt, "solve");
  } else if (req == "stop") {
    ari::write<std::string>(a, rt.k.status, "nothing to stop");
  } else {
    ari::write<std::string>(a, rt.k.status, "unknown request '" + req + "'");
  }
}

void tick_fe_lab(FeLabRT &rt) {
  std::shared_ptr<GeometryNode> node = rt.node.lock();
  if (!node)
    return;
  std::shared_ptr<GeometryNode> slice = rt.slice_node.lock();
  std::shared_ptr<GeometryNode> edges = rt.edges_node.lock();
  std::shared_ptr<GeometryNode> ghost = rt.ghost_node.lock();
  cvc::app &a = *rt.app;
  pump(*rt.lane);
  pump(*rt.slice_lane);

  if (rt.job) {
    FeJob &j = *rt.job;
    std::string rest;
    if (first_word(ari::read_or<std::string>(a, rt.k.request, ""), &rest) == "stop") {
      ari::write<std::string>(a, rt.k.request, rest); // what follows the stop waits for the job
      if (j.stoppable) {
        j.cancel = true; // the heat solve's progress callback sees it at its next step
        ari::write<std::string>(a, rt.k.status, "stopping...");
      } else {
        // Loading, tet meshing and the other solves cannot be interrupted: say so, and that it
        // still runs.
        ari::write<std::string>(a, rt.k.status, j.running + " (only a heat solve can be stopped)");
      }
    }
    {
      std::vector<double> snap;
      {
        std::lock_guard<std::mutex> lock(j.snap_mtx);
        if (j.fresh) {
          snap.swap(j.snapshot);
          j.fresh = false;
        }
      }
      if (!snap.empty())
        set_field(rt, std::move(snap));
    }
    ari::write<int>(a, rt.k.progress, j.progress.load());
    if (j.done.load(std::memory_order_acquire)) {
      std::shared_ptr<FeJob> done = std::move(rt.job);
      apply_fe_job(rt, *done);
    }
  } else if (rt.load_pending) {
    // The realize-time SDF: computed now, before any request (which waits for it).
    rt.load_pending = false;
    rt.work.start();
    launch_fe_job(rt, "load");
  } else {
    // Instant commands run back to back; one that starts a job leaves the rest waiting.
    for (int i = 0; i < 16 && !rt.job; ++i) {
      const std::string req = take_request(a, rt.k.request);
      if (req.empty())
        break;
      rt.work.start();
      run_fe_request(rt, req);
    }
  }
  follow_fe_view(rt, *node, slice.get(), edges.get(), ghost.get());
  rt.work.settle(a, !rt.job && ari::read_or<std::string>(a, rt.k.request, "").empty(),
                 rt.k.work_ms);
}

// The source the node's own source: / fit: declare, before any key exists.
SourceSpec fe_source_from_node(const ari::SceneNode &n) {
  SourceSpec s;
  const bool sdf = !n.source_sdf_mesh.empty();
  s.type = sdf ? "sdf" : "volume";
  s.file = sdf ? n.source_sdf_mesh : n.source_file;
  s.dim = sdf ? n.sdf_dim : to_int_clamped(n.props.num("resample", 0.0), 0, 1024, 0);
  s.algorithm = n.sdf_algorithm.empty() ? std::string("v2") : n.sdf_algorithm;
  s.up = fit_up_axis(n);
  s.flip = n.fit_flip;
  s.height = n.has_fit ? n.fit_height : 0.0;
  s.base_dir = n.base_dir;
  return s;
}

// type: fe_lab
std::shared_ptr<GraphicsNode> realize_fe_lab(SceneGraph &sg, const ari::SceneNode &n,
                                             GraphicsNode *parent, RealizedScene &out,
                                             std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: fe_lab '" + n.id + "': " + m);
  };
  if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
    warn("has no source (a volume file or { sdf: { mesh, dim } })");
    return nullptr;
  }
  auto rt = std::make_shared<FeLabRT>(sg.appContext());
  // The node's own source: is read now, on this thread, so the scene frames it: a volume file
  // whole, and for an SDF only its mesh -- the field itself, the slow part, is computed by the
  // first tick's load on the lane, so the document shows at once. A `load` request reads the
  // source.* keys again on the lane.
  const SourceSpec spec = fe_source_from_node(n);
  try {
    const std::shared_ptr<FeLoaded> loaded =
        spec.type == "sdf" ? frame_sdf_source(spec) : load_fe_source(*rt->app, spec);
    if (loaded->volume) {
      rt->volume = *loaded->volume;
      rt->shared_volume = loaded->volume;
      rt->have_volume = true;
      rt->source_sig = loaded->sig;
    } else {
      rt->load_pending = true;
    }
    rt->input = loaded->input;
    rt->place = loaded->place;
    rt->placeholder = loaded->placeholder;
    rt->source_version = 1;
    rt->lane = std::make_unique<cvc::async_lane>("fe_lab." + n.id);
    rt->slice_lane = std::make_unique<cvc::async_lane>("fe_lab." + n.id + ".slice");
  } catch (const std::exception &e) {
    warn(e.what());
    return nullptr;
  }
  rt->base_dir = n.base_dir;
  std::shared_ptr<GeometryNode> g = make_geometry_node(sg, parent, n.id, rt->placeholder);
  if (!g)
    return nullptr;
  g->setRenderMode(GeometryRenderMode::LINES);
  apply_material(*g, n);
  std::shared_ptr<GeometryNode> s = g->createChild<GeometryNode>(n.id + "_slice", cvc::geometry());
  if (s)
    apply_material(*s, n);
  // The elements view's outlines (drawn over their faces) and the ghost of the outer surface.
  std::shared_ptr<GeometryNode> e = g->createChild<GeometryNode>(n.id + "_edges", cvc::geometry());
  if (e) {
    e->setRenderMode(GeometryRenderMode::LINES);
    e->setUseSingleColor(true);
    e->setColor(0.04, 0.04, 0.05);
    e->setLineWidth(1.0);
    e->setDepthOffset(4.0);
    e->setVisible(false);
  }
  std::shared_ptr<GeometryNode> gh = g->createChild<GeometryNode>(n.id + "_ghost", cvc::geometry());
  if (gh) {
    gh->setUseSingleColor(true);
    gh->setColor(0.8, 0.82, 0.86);
    gh->setVisible(false);
  }
  rt->node = g;
  rt->slice_node = s;
  rt->edges_node = e;
  rt->ghost_node = gh;
  rt->single_color = g->getUseSingleColor();
  rt->main_sig = "p|" + std::to_string(rt->source_version); // the placeholder is already up

  cvc::app &a = *rt->app;
  const std::string base = g->stateName("fe");
  const ari::Value &pr = n.props;
  FeKeys &k = rt->k;
  k.request = seed_str(a, base, "request", pr.flag("auto_mesh") ? "mesh" : "");
  k.busy = ari::resolve_bind(base, "busy");
  k.stoppable = ari::resolve_bind(base, "stoppable");
  k.status = ari::resolve_bind(base, "status");
  k.progress = ari::resolve_bind(base, "progress");
  k.tets = ari::resolve_bind(base, "stats.tets");
  k.hexes = ari::resolve_bind(base, "stats.hexes");
  k.nv = ari::resolve_bind(base, "stats.vertices");
  k.quality_min = ari::resolve_bind(base, "stats.quality_min");
  k.quality_mean = ari::resolve_bind(base, "stats.quality_mean");
  k.field_min = ari::resolve_bind(base, "field.min");
  k.field_max = ari::resolve_bind(base, "field.max");
  k.work_ms = ari::resolve_bind(base, "stats.work_ms");
  k.last_mesh = ari::resolve_bind(base, "last_mesh");
  k.source_type = seed_str(a, base, "source.type", spec.type);
  k.source_file = seed_str(a, base, "source.file", spec.file);
  k.source_dim = seed_num(a, base, "source.dim", spec.dim);
  k.source_algorithm = seed_str(a, base, "source.algorithm", spec.algorithm);
  k.source_up = seed_str(a, base, "source.up", std::string(1, spec.up));
  k.source_flip = seed_bool(a, base, "source.flip", spec.flip);
  k.source_height = seed_num(a, base, "source.height", spec.height);
  k.auto_mesh = seed_bool(a, base, "auto.mesh", pr.flag("auto_mesh"));
  k.auto_solve = seed_bool(a, base, "auto.solve", pr.flag("auto_solve"));
  // `interval: [a, b]` is shorthand for region interval + both bounds (`interval: true`, region
  // interval alone); explicit region / isovalue / isovalue2 props win over it.
  bool ivl_region = false, ivl_bounds = false;
  double ivl[2] = {0.0, 0.0};
  if (const ari::Value *iv = pr.find("interval")) {
    bool ok = false;
    if (iv->is_seq() && iv->items.size() == 2) {
      ivl[0] = iv->items[0].as_double(std::nan(""));
      ivl[1] = iv->items[1].as_double(std::nan(""));
      ok = ivl_region = ivl_bounds = std::isfinite(ivl[0]) && std::isfinite(ivl[1]);
    } else if (iv->is_scalar()) {
      for (const char *f : {"true", "yes", "on", "1", "false", "no", "off", "0"})
        ok = ok || iv->scalar == f;
      ivl_region = ok && iv->as_bool();
    }
    if (!ok)
      warn("interval: wants [lo, hi] (two numbers) or a flag; ignored");
  }
  k.region =
      seed_str(a, base, "mesh.region", prop_str(pr, "region", ivl_region ? "interval" : "solid"));
  k.isovalue = seed_num(a, base, "mesh.isovalue", pr.num("isovalue", ivl_bounds ? ivl[0] : 0.0));
  k.inside = seed_str(a, base, "mesh.inside",
                      prop_str(pr, "inside", n.source_sdf_mesh.empty() ? "above" : "below"));
  // The second bound: the prop, the b of `interval: [a, b]`, or default_isovalue2 -- which needs
  // the volume's range, so for an SDF still to be computed its load sets it (apply_fe_loaded)
  // unless someone has by then.
  if (pr.find("isovalue2") || ivl_bounds || rt->have_volume)
    k.isovalue2 = seed_num(a, base, "mesh.isovalue2",
                           pr.num("isovalue2", ivl_bounds ? ivl[1] : default_isovalue2(*rt)));
  else
    k.isovalue2 = ari::resolve_bind(base, "mesh.isovalue2");
  k.improve = seed_num(a, base, "mesh.improve", pr.num("improve", 0));
  k.improve_iterations =
      seed_num(a, base, "mesh.improve_iterations", pr.num("improve_iterations", 1));
  k.elements = seed_str(a, base, "mesh.elements", prop_str(pr, "elements", "tet"));
  k.problem = seed_str(a, base, "problem", prop_str(pr, "problem", "poisson"));
  k.poisson_f = seed_num(a, base, "poisson.f", pr.num("poisson_f", 1.0));
  k.heat_kappa = seed_num(a, base, "heat.kappa", pr.num("heat_kappa", 1.0));
  k.heat_dt = seed_num(a, base, "heat.dt", pr.num("heat_dt", 0.01));
  k.heat_steps = seed_num(a, base, "heat.steps", pr.num("heat_steps", 100));
  k.mode_index = seed_num(a, base, "modes.index", pr.num("mode", 1));
  k.colormap = seed_str(a, base, "view.colormap", prop_str(pr, "colormap", "viridis"));
  k.show_surface = seed_bool(a, base, "view.show_surface", pr.flag("show_surface", true));
  k.show_slice = seed_bool(a, base, "view.show_slice", pr.flag("show_slice", true));
  k.slice_axis = seed_num(a, base, "view.slice_axis", pr.num("slice_axis", 0));
  k.slice_offset = seed_num(a, base, "view.slice_offset", pr.num("slice_offset", 0.0));
  k.surface_opacity = seed_num(a, base, "view.surface_opacity", pr.num("surface_opacity", 0.35));
  k.view_mode = seed_str(a, base, "view.mode", prop_str(pr, "view", "field"));
  k.boundary = seed_str(a, base, "view.boundary", prop_str(pr, "boundary", "input"));
  k.color_by = seed_str(a, base, "view.color_by", prop_str(pr, "color_by", "size"));
  k.cut_side = seed_num(a, base, "view.cut_side", pr.num("cut_side", 1.0));
  k.ghost = seed_num(a, base, "view.ghost", pr.num("ghost", 0.0));
  k.fixed_range = seed_bool(a, base, "view.fixed_range", pr.flag("fixed_range"));
  k.range_min = seed_num(a, base, "view.range_min", pr.num("range_min", 0.0));
  k.range_max = seed_num(a, base, "view.range_max", pr.num("range_max", 1.0));
  ari::write<int>(a, k.busy, 0);
  ari::write<int>(a, k.stoppable, 0);
  ari::write<int>(a, k.progress, 0);
  publish_fe_stats(*rt, 0.0, 0.0);
  ari::write<std::string>(
      a, k.status, rt->load_pending ? "loading " + spec.file + "..." : std::string("volume ready"));
  ari::write<std::string>(a, k.field_min, "-");
  ari::write<std::string>(a, k.field_max, "-");
  ari::write<std::string>(a, k.last_mesh, std::string());
  init_field_colors(a, rt->fc, g->stateName("field_colors"), k.colormap);
  g->setColorTable(rt->fc.table);
  if (s)
    s->setColorTable(rt->fc.table);
  // Keys written before the scene was realized (an init: program) name another source: load it
  // first. A seeded request (auto_mesh) runs after it. (A pending load reads the keys anyway.)
  if (!rt->load_pending && fe_source_from_keys(*rt).sig() != rt->source_sig) {
    const std::string waiting = ari::read_or<std::string>(a, k.request, "");
    ari::write<std::string>(a, k.request, waiting.empty() ? "load" : "load " + waiting);
  }

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick_fe_lab(*rt); });

  // <node>.transfer_function: the isovalue as isocontour "iso1" over the source volume's range --
  // and, for an interval, isovalue2 as "iso2" -- so a tf_editor moves them (and, debounced,
  // re-meshes). Colour and opacity stay transparent: fe_lab colours its field by a named colormap,
  // not by a transfer function. Attached after fe_lab's own tick, so a remesh request it writes is
  // taken on the next frame. The binding follows the volume each load brings (an SDF computed
  // after this, a `load` of another source): the bars move to where the keys' values sit on its
  // range.
  {
    double vlo = 0.0, vhi = 1.0;
    fe_value_range(*rt, vlo, vhi);
    tf_binding_options o;
    o.seed = cvc::tf::model::default_model();
    o.seed.alpha = {{0.0, 0.0f}, {1.0, 0.0f}};
    // A bound past the volume's range (legal: `interval: [0.5, 2]` is "0.5 and up" on a 0/1 mask)
    // is a bar pinned at that end of the axis, which the binding takes as standing for it: the key
    // keeps its true value (tf_binding.h).
    const auto bar = [&](const char *id, const std::string &key) {
      cvc::tf::isocontour iso;
      iso.id = id;
      iso.at =
          std::clamp(cvc::tf::to_normalized(ari::read_or<double>(a, key, 0.0), vlo, vhi), 0.0, 1.0);
      return iso;
    };
    o.seed.isocontours = {bar("iso1", k.isovalue)};
    if (ari::read_or<std::string>(a, k.region, "solid") == "interval")
      o.seed.isocontours.push_back(bar("iso2", k.isovalue2));
    o.seed.normalize();
    o.seed_is_applied = true; // fe.mesh.isovalue (and isovalue2) already hold them
    o.fe_isocontour = "iso1";
    o.fe_isocontour2 = "iso2";
    o.auto_remesh = pr.flag("auto_remesh", o.auto_remesh);
    o.remesh_debounce = pr.num("remesh_debounce", o.remesh_debounce);
    const std::weak_ptr<FeLabRT> wrt = rt;
    o.volume_source = [wrt]() -> std::shared_ptr<const cvc::volume> {
      const std::shared_ptr<FeLabRT> r = wrt.lock();
      return r ? r->shared_volume : nullptr;
    };
    attach_transfer_function(out, g, tf_target::fe_lab, rt->volume, std::move(o), warnings);
  }
  return g;
}

} // namespace
#endif // CVC_ENABLE_LIBIGL

void register_mesh_node_types() {
#ifdef CVC_ENABLE_LIBIGL
  register_scene_node_type("mesh_lab", realize_mesh_lab);
  register_scene_node_type("fe_lab", realize_fe_lab);
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
